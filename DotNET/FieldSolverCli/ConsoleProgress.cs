using System;
using System.Diagnostics;
using System.Globalization;

using FieldSolver;

namespace FieldSolverCli
{
    /// <summary>
    /// Prints one line when a run starts and one when it finishes, with the times
    /// of the engine's first and last progress reports in between.
    /// </summary>
    internal sealed class ConsoleProgress : IProgress<FieldSolverProgress>
    {
        /// <summary>Bytes in one megabyte.</summary>
        private const Int64 BYTES_PER_MB = 1048576;

        /// <summary>Run progress value that means the engine reported completion.</summary>
        private const Double FULL_PERCENT = 100.0;

        /// <summary>Value of a time that has not been taken yet.</summary>
        private const Double NOT_SET = -1.0;

        /// <summary>Serializes the reports, which also arrive from engine worker threads.</summary>
        private readonly Object _Gate = new Object();

        /// <summary>Wall clock of the run in progress.</summary>
        private readonly Stopwatch _RunClock = new Stopwatch();

        /// <summary>Index of the run whose start was last printed.</summary>
        private Int32 _StartedRun;

        /// <summary>Count of engine progress reports received during the run.</summary>
        private Int32 _EngineReports;

        /// <summary>Seconds into the run at the first engine report, or <see cref="NOT_SET"/>.</summary>
        private Double _FirstEngineSeconds;

        /// <summary>Seconds into the run when the engine first reported 100 percent, or <see cref="NOT_SET"/>.</summary>
        private Double _FullSeconds;

        /// <summary>Initializes the printer.</summary>
        internal ConsoleProgress
            (
            )
        {
            _FirstEngineSeconds = NOT_SET;
            _FullSeconds = NOT_SET;
        }

        /// <summary>Prints run starts and finishes, and times the engine reports in between.</summary>
        /// <param name="Value">Snapshot from the solver.</param>
        public void Report
            (
            FieldSolverProgress Value
            )
        {
            lock (_Gate)
            {
                if (Value.Completed != null)
                {
                    PrintFinished(Value);
                    return;
                }
                if (Value.RunIndex != _StartedRun)
                {
                    // The solver's own report that a run is starting.
                    _StartedRun = Value.RunIndex;
                    _EngineReports = 0;
                    _FirstEngineSeconds = NOT_SET;
                    _FullSeconds = NOT_SET;
                    _RunClock.Restart();
                    Console.WriteLine("  started run " + Value.RunIndex + " [" + Value.Settings + "]");
                    return;
                }
                // Everything else is the engine's guidance progress, passed through.
                _EngineReports += 1;
                if (_FirstEngineSeconds < 0.0) _FirstEngineSeconds = _RunClock.Elapsed.TotalSeconds;
                if ((_FullSeconds < 0.0) && (Value.RunPercentComplete >= FULL_PERCENT))
                {
                    _FullSeconds = _RunClock.Elapsed.TotalSeconds;
                }
            }
        }

        /// <summary>Prints the line for a finished run, with the memory the process holds.</summary>
        /// <param name="Value">Snapshot whose <c>Completed</c> is set.</param>
        private void PrintFinished
            (
            FieldSolverProgress Value
            )
        {
            using (Process Current = Process.GetCurrentProcess())
            {
                Console.WriteLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "  finished run {0}/{1} overall={2:F1}% run_s={3:F1} first_engine_report_s={4:F1} engine_100_percent_s={5:F1} engine_reports={6} managed_mb={7} peak_working_set_mb={8} best_so_far=run {9}",
                    Value.RunIndex,
                    Value.PlannedRunCount,
                    Value.OverallPercentComplete,
                    _RunClock.Elapsed.TotalSeconds,
                    _FirstEngineSeconds,
                    _FullSeconds,
                    _EngineReports,
                    GC.GetTotalMemory(false) / BYTES_PER_MB,
                    Current.PeakWorkingSet64 / BYTES_PER_MB,
                    Value.BestSoFar?.RunIndex ?? 0));
            }
        }
    }
}
