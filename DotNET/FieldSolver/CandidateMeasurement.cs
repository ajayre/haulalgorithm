using System;

namespace FieldSolver
{
    /// <summary>
    /// The numbers the solver keeps from one engine run. The engine result itself
    /// is dropped as soon as these are read, because it holds the whole plan.
    /// </summary>
    internal sealed class CandidateMeasurement
    {
        /// <summary>Gets whole-job driven centerline metres.</summary>
        public Double DistanceM { get; init; }

        /// <summary>Gets driven metres with a load on.</summary>
        public Double LoadedDistanceM { get; init; }

        /// <summary>Gets driven metres with no load.</summary>
        public Double UnloadedDistanceM { get; init; }

        /// <summary>Gets deterministic guidance hours.</summary>
        public Double DeterministicHours { get; init; }

        /// <summary>Gets required cut in BCY.</summary>
        public Double RequiredCutBcy { get; init; }

        /// <summary>Gets required fill in BCY of bank cut.</summary>
        public Double RequiredFillBcy { get; init; }

        /// <summary>Gets deferred cut in BCY, surplus included.</summary>
        public Double DeferredCutBcy { get; init; }

        /// <summary>Gets deferred fill in BCY, shortfall included.</summary>
        public Double DeferredFillBcy { get; init; }

        /// <summary>Gets the manual assist count.</summary>
        public Int32 ManualAssistCount { get; init; }

        /// <summary>Gets wall-clock seconds the run took.</summary>
        public Double ElapsedSeconds { get; init; }
    }
}
