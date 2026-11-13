using System;
using System.Globalization;
using System.IO;
using System.Threading;

using HaulBuilder;

using FieldSolver;

namespace FieldSolverCli
{
    /// <summary>
    /// FieldSolver.exe. Solves one field and prints every run.
    /// </summary>
    static internal class Program
    {
        /// <summary>Exit code when the solve finished.</summary>
        private const Int32 EXIT_OK = 0;

        /// <summary>Exit code when the engine or the solver failed.</summary>
        private const Int32 EXIT_FAILED = 1;

        /// <summary>Exit code for a bad command line or a missing field file.</summary>
        private const Int32 EXIT_USAGE = 2;

        /// <summary>Exit code when the solve was cancelled before its first run finished.</summary>
        private const Int32 EXIT_CANCELLED = 3;

        /// <summary>Reads the switches and solves the field.</summary>
        /// <param name="Args">The field file. <c>-h</c> or <c>--help</c> prints the options.</param>
        /// <returns>Zero on success. See the exit code constants.</returns>
        static private Int32 Main
            (
            String[] Args
            )
        {
            CommandLine Line = CommandLine.Parse(Args);
            if (Line.ShowHelp)
            {
                // Help was asked for, so a later bad argument does not hide the option list.
                CommandLine.WriteHelp(Console.Out);
                return EXIT_OK;
            }
            if (Line.Error != null)
            {
                Console.Error.WriteLine(Line.Error);
                Console.Error.WriteLine("Use -h or --help to list the options.");
                return EXIT_USAGE;
            }
            String FieldPath = Line.FieldPath ?? "";
            if (!File.Exists(FieldPath))
            {
                Console.Error.WriteLine("Field file not found: " + FieldPath);
                return EXIT_USAGE;
            }

            FieldSolverRequest Request = Line.CreateRequest();
            using (CancellationTokenSource Stop = new CancellationTokenSource())
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
                    FieldSolverResult Result = new FieldSolverEngine().Solve(Request, new ConsoleProgress(), Stop.Token);
                    PrintResult(Result);
                    return EXIT_OK;
                }
                catch (OperationCanceledException)
                {
                    Console.Error.WriteLine("Cancelled before the first run finished.");
                    return EXIT_CANCELLED;
                }
                catch (Exception Ex) when ((Ex is ArgumentException) || (Ex is InvalidOperationException) || (Ex is TimeoutException))
                {
                    Console.Error.WriteLine(Ex.Message);
                    return EXIT_FAILED;
                }
                finally
                {
                    Console.CancelKeyPress -= OnCtrlC;
                }
            }
        }

        /// <summary>Prints the summary line, one line per run, and the best settings.</summary>
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
