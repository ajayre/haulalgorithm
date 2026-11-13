using System;
using System.Diagnostics;
using System.Globalization;
using System.Threading;

using HaulBuilder;

using FieldSolver;

namespace FieldSolverProof
{
    /// <summary>
    /// Solves one field with the real HaulBuilder engine and prints every run.
    /// It is also the worked example of a caller: it builds the request, passes a
    /// progress sink and a cancel token, handles a solve that ends early, and
    /// builds the plan from the returned options.
    /// </summary>
    static internal class Program
    {
        /// <summary>Exit code when the solve finished and any verification held.</summary>
        private const Int32 EXIT_OK = 0;

        /// <summary>Exit code when the returned options did not reproduce the best run.</summary>
        private const Int32 EXIT_VERIFY_FAILED = 1;

        /// <summary>Exit code for a bad command line.</summary>
        private const Int32 EXIT_USAGE = 2;

        /// <summary>Exit code when the solve was cancelled before its first run finished.</summary>
        private const Int32 EXIT_CANCELLED = 3;

        /// <summary>Prefix of the argument that cancels the solve after a number of seconds.</summary>
        private const String CANCEL_AFTER_PREFIX = "cancelafter=";

        /// <summary>Runs one solve.</summary>
        /// <param name="Args">
        /// The field file, then in any order: <c>thorough</c> for the thorough search,
        /// <c>verify</c> to build the plan from the returned options and compare it
        /// with the best run, and <c>cancelafter=SECONDS</c> to cancel the solve.
        /// </param>
        /// <returns>Zero on success; see the exit code constants.</returns>
        static private Int32 Main
            (
            String[] Args
            )
        {
            if (Args.Length < 1)
            {
                Console.Error.WriteLine("Usage: FieldSolverProof <field.agd> [thorough] [verify] [cancelafter=SECONDS]");
                return EXIT_USAGE;
            }
            Boolean Thorough = false;
            Boolean Verify = false;
            Double CancelAfterSeconds = 0.0;
            for (Int32 Index = 1; Index < Args.Length; Index += 1)
            {
                if (String.Equals(Args[Index], "thorough", StringComparison.OrdinalIgnoreCase)) Thorough = true;
                else if (String.Equals(Args[Index], "verify", StringComparison.OrdinalIgnoreCase)) Verify = true;
                else if (Args[Index].StartsWith(CANCEL_AFTER_PREFIX, StringComparison.OrdinalIgnoreCase)
                    && Double.TryParse(
                        Args[Index].Substring(CANCEL_AFTER_PREFIX.Length),
                        NumberStyles.Float,
                        CultureInfo.InvariantCulture,
                        out CancelAfterSeconds)
                    && (CancelAfterSeconds > 0.0))
                {
                    // Parsed into CancelAfterSeconds by the condition above.
                }
                else
                {
                    Console.Error.WriteLine("Unknown argument: " + Args[Index]);
                    return EXIT_USAGE;
                }
            }

            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = Args[0],
            };
            FieldSolverRequest Request = new FieldSolverRequest(Options);
            // Intent: thorough search still derives lane axes from this field's design slope.
            if (Thorough) Request.SearchSpace = FieldSolverSearchSpace.CreateThorough(Options);

            using (CancellationTokenSource Stop = new CancellationTokenSource())
            {
                Stopwatch SinceCancel = new Stopwatch();
                // The stopwatch starts when the token fires, whoever fires it.
                using (Stop.Token.Register(() => SinceCancel.Start()))
                {
                    ConsoleCancelEventHandler OnCtrlC = (Sender, Event) =>
                    {
                        // Keep the process alive so the runs made so far can be printed.
                        Event.Cancel = true;
                        Stop.Cancel();
                    };
                    Console.CancelKeyPress += OnCtrlC;
                    try
                    {
                        if (CancelAfterSeconds > 0.0) Stop.CancelAfter(TimeSpan.FromSeconds(CancelAfterSeconds));
                        FieldSolverResult Result = new FieldSolverEngine().Solve(Request, new ConsoleProgress(), Stop.Token);
                        PrintResult(Result);
                        if (Result.Status == FieldSolverStatus.Cancelled)
                        {
                            Console.WriteLine(String.Format(
                                CultureInfo.InvariantCulture,
                                "cancelled: returned {0:F1} s after the cancel request",
                                SinceCancel.Elapsed.TotalSeconds));
                        }
                        // A cancelled solve still returns options that can be run.
                        return Verify ? VerifyBestOptions(Result) : EXIT_OK;
                    }
                    catch (OperationCanceledException)
                    {
                        // Cancelled before the caller's own settings were measured:
                        // there is no result, and the caller keeps its settings.
                        Console.WriteLine(String.Format(
                            CultureInfo.InvariantCulture,
                            "cancelled before the first run finished: stopped {0:F1} s after the cancel request",
                            SinceCancel.Elapsed.TotalSeconds));
                        return EXIT_CANCELLED;
                    }
                    finally
                    {
                        Console.CancelKeyPress -= OnCtrlC;
                    }
                }
            }
        }

        /// <summary>
        /// Builds the plan from the returned options with one more engine run and
        /// checks that it has the distance the solver measured.
        /// </summary>
        /// <param name="Result">Finished solve.</param>
        /// <returns><see cref="EXIT_OK"/> when the distances are equal, otherwise <see cref="EXIT_VERIFY_FAILED"/>.</returns>
        static private Int32 VerifyBestOptions
            (
            FieldSolverResult Result
            )
        {
            // This is the call a user interface makes after the user accepts the
            // settings: the caller's own output switches are still in these options.
            HaulBuilderResult Direct = new HaulBuilderEngine().Run(Result.BestOptions);
            Double DirectDistanceM = Direct.DetailedGuidanceTime!.DistanceM;
            // The engine is deterministic, so the two numbers must be the same number.
            Boolean Same = DirectDistanceM == Result.Best.DistanceM;
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "verify: BestOptions distance_m={0:R} solver best distance_m={1:R} equal={2}",
                DirectDistanceM,
                Result.Best.DistanceM,
                Same));
            return Same ? EXIT_OK : EXIT_VERIFY_FAILED;
        }

        /// <summary>Prints the summary line and one line per run.</summary>
        /// <param name="Result">Finished solve.</param>
        static private void PrintResult
            (
            FieldSolverResult Result
            )
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "status={0} runs={1}/{2} best=run {3} saving_m={4:F1} saving_percent={5:F3} elapsed_s={6:F1} required_cut_bcy={7:F3} required_fill_bcy={8:F3}",
                Result.Status,
                Result.Candidates.Count,
                Result.PlannedRunCount,
                Result.Best.RunIndex,
                Result.SavingM,
                Result.SavingPercent,
                Result.ElapsedSeconds,
                Result.Baseline.RequiredCutBcy,
                Result.Baseline.RequiredFillBcy));
            foreach (FieldSolverCandidate Candidate in Result.Candidates)
            {
                Console.WriteLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  run {0,2} changes={1} ok={2} limits={3} distance_m={4:F1} unfinished_bcy={5:F3} unfinished_percent={6:F4} charge_m={7:F1} score_m={8:F1} assists={9} hours={10:F3} deferred_cut_bcy={11:F3} deferred_fill_bcy={12:F3} run_s={13:F1} [{14}] {15}",
                    Candidate.RunIndex,
                    Candidate.ChangeCount,
                    Candidate.Succeeded,
                    Candidate.WithinLimits,
                    Candidate.DistanceM,
                    Candidate.UnfinishedBcy,
                    Candidate.UnfinishedPercent,
                    Candidate.UnfinishedChargeM,
                    Candidate.ScoreM,
                    Candidate.ManualAssistCount,
                    Candidate.DeterministicHours,
                    Candidate.DeferredCutBcy,
                    Candidate.DeferredFillBcy,
                    Candidate.ElapsedSeconds,
                    Candidate.Settings,
                    Candidate.FailureMessage));
            }
            Console.WriteLine("best settings: " + Result.Best.Settings);
        }
    }
}
