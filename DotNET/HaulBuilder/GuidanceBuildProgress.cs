using System;

namespace HaulBuilder
{
    /// <summary>
    /// Progress snapshot reported while exact production guidance is built.
    /// </summary>
    public sealed class GuidanceBuildProgress
    {
        private const Double CoverageShare = 0.10;
        private const Double SequencingShare = 0.80;
        private const Double LifecycleShare = 0.10;

        /// <summary>Initializes one guidance-build progress snapshot.</summary>
        /// <param name="Phase">Active guidance-build phase name.</param>
        /// <param name="Completed">Completed work units in the active phase.</param>
        /// <param name="Total">Total work units in the active phase.</param>
        /// <param name="PercentComplete">Overall progress from 0.0 through 100.0.</param>
        public GuidanceBuildProgress
            (
            String Phase,
            Int32 Completed,
            Int32 Total,
            Double PercentComplete
            )
        {
            this.Phase = Phase ?? String.Empty;
            this.Completed = Completed;
            this.Total = Total;
            this.PercentComplete = PercentComplete;
        }

        /// <summary>Gets the active guidance-build phase name.</summary>
        public String Phase { get; }

        /// <summary>Gets completed work units in the active phase.</summary>
        public Int32 Completed { get; }

        /// <summary>Gets total work units in the active phase.</summary>
        public Int32 Total { get; }

        /// <summary>Gets overall progress from 0.0 through 100.0 inclusive.</summary>
        public Double PercentComplete { get; }

        /// <summary>Maps planner coverage or sequencing progress to an overall percentage.</summary>
        /// <param name="Phase">Planner phase label.</param>
        /// <param name="Completed">Completed units in the active phase.</param>
        /// <param name="Total">Total units in the active phase.</param>
        /// <returns>Normalized progress snapshot for API callers.</returns>
        internal static GuidanceBuildProgress FromPlannerPhase
            (
            String Phase,
            Int32 Completed,
            Int32 Total
            )
        {
            String NormalizedPhase = NormalizePhase(Phase);
            Double PhaseStart = NormalizedPhase switch
            {
                "coverage" => 0.0,
                "sequencing" => CoverageShare * 100.0,
                "lifecycle" => (CoverageShare + SequencingShare) * 100.0,
                _ => CoverageShare * 100.0,
            };
            Double PhaseShare = NormalizedPhase switch
            {
                "coverage" => CoverageShare,
                "sequencing" => SequencingShare,
                "lifecycle" => LifecycleShare,
                _ => SequencingShare,
            };
            Double PhasePercent = Total > 0
                ? ((Double)Completed / Total) * PhaseShare * 100.0
                : PhaseShare * 100.0;
            return new GuidanceBuildProgress(
                NormalizedPhase,
                Completed,
                Total,
                Math.Min(100.0, PhaseStart + PhasePercent));
        }

        /// <summary>Maps lifecycle construction progress to an overall percentage.</summary>
        /// <param name="Completed">Completed lifecycle passes.</param>
        /// <param name="Total">Total lifecycle passes.</param>
        /// <returns>Normalized progress snapshot for API callers.</returns>
        internal static GuidanceBuildProgress FromLifecyclePhase
            (
            Int32 Completed,
            Int32 Total
            )
        {
            Double PhaseStart = (CoverageShare + SequencingShare) * 100.0;
            Double PhasePercent = Total > 0
                ? ((Double)Completed / Total) * LifecycleShare * 100.0
                : LifecycleShare * 100.0;
            return new GuidanceBuildProgress(
                "lifecycle",
                Completed,
                Total,
                Math.Min(100.0, PhaseStart + PhasePercent));
        }

        /// <summary>Normalizes planner phase labels to stable API names.</summary>
        /// <param name="Phase">Raw planner phase label.</param>
        /// <returns>Stable phase name.</returns>
        private static String NormalizePhase
            (
            String Phase
            )
        {
            if (String.IsNullOrEmpty(Phase))
            {
                return "sequencing";
            }
            if (Phase.StartsWith("coverage", StringComparison.Ordinal))
            {
                return "coverage";
            }
            if (Phase.StartsWith("sequencing", StringComparison.Ordinal))
            {
                return "sequencing";
            }
            if (Phase.StartsWith("lifecycle", StringComparison.Ordinal))
            {
                return "lifecycle";
            }
            return Phase;
        }
    }
}
