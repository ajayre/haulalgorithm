using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// The production candidate runner, and the only class in this assembly that
    /// calls the HaulBuilder engine. It also releases what a finished run leaves
    /// in memory, so a solve of many runs needs no more memory than one run.
    /// </summary>
    internal sealed class HaulBuilderCandidateRunner : ICandidateRunner
    {
        /// <summary>Initializes the runner.</summary>
        public HaulBuilderCandidateRunner
            (
            )
        {
        }

        /// <summary>Runs the HaulBuilder engine once, measures the plan, and frees the run's memory.</summary>
        /// <param name="Options">Complete options for the run.</param>
        /// <param name="Progress">Optional sink for the engine's guidance progress.</param>
        /// <param name="CancellationToken">Token the engine observes while it builds guidance.</param>
        /// <returns>The measurements of the finished plan.</returns>
        /// <exception cref="InvalidOperationException">Thrown when the engine returns a result without guidance timing, metrics, or a deferred-work summary.</exception>
        public CandidateMeasurement Run
            (
            HaulBuilderOptions Options,
            IProgress<GuidanceBuildProgress>? Progress,
            CancellationToken CancellationToken
            )
        {
            try
            {
                return Measure(Options, Progress, CancellationToken);
            }
            finally
            {
                // The engine keys its connector cache by the grid object of each run,
                // so nothing cached can serve the next run, and nothing else frees it.
                TransitPlanner.ResetCache();
                // The finished plan is unreachable now. Collect it before the next run
                // allocates, so two plans never occupy memory at the same time.
                GC.Collect();
            }
        }

        /// <summary>Runs the engine and reads the solver's numbers from its result.</summary>
        /// <param name="Options">Complete options for the run.</param>
        /// <param name="Progress">Optional sink for the engine's guidance progress.</param>
        /// <param name="CancellationToken">Token the engine observes while it builds guidance.</param>
        /// <returns>The measurements of the finished plan.</returns>
        /// <exception cref="InvalidOperationException">Thrown when the result lacks guidance timing, metrics, or a deferred-work summary.</exception>
        [MethodImpl(MethodImplOptions.NoInlining)]
        static private CandidateMeasurement Measure
            (
            HaulBuilderOptions Options,
            IProgress<GuidanceBuildProgress>? Progress,
            CancellationToken CancellationToken
            )
        {
            // Not inlined: the engine result must die with this stack frame so the
            // caller's collection can reclaim it.
            Stopwatch Clock = Stopwatch.StartNew();
            HaulBuilderResult Result = new HaulBuilderEngine().Run(Options, Progress, CancellationToken);
            Clock.Stop();
            // Whole-job distance exists only when exact guidance was built.
            GuidanceTiming.Result Timing = Result.DetailedGuidanceTime
                ?? throw new InvalidOperationException("The engine returned no guidance timing.");
            HaulAnalysisMetrics Metrics = Result.Metrics
                ?? throw new InvalidOperationException("The engine returned no metrics.");
            HaulDeferredWorkSummary Deferred = Result.DeferredWorkSummary
                ?? throw new InvalidOperationException("The engine returned no deferred-work summary.");
            return new CandidateMeasurement
            {
                DistanceM = Timing.DistanceM,
                LoadedDistanceM = Timing.LoadedDistanceM,
                UnloadedDistanceM = Timing.UnloadedDistanceM,
                DeterministicHours = Metrics.DeterministicHours,
                RequiredCutBcy = Deferred.RequiredCutBcy,
                RequiredFillBcy = Deferred.RequiredFillBcy,
                DeferredCutBcy = Deferred.DeferredCutBcy,
                DeferredFillBcy = Deferred.DeferredFillBcy,
                ManualAssistCount = Result.LifecycleGuidance?.ManualAssistCount ?? 0,
                ElapsedSeconds = Clock.Elapsed.TotalSeconds,
            };
        }
    }
}
