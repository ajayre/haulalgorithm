using System;

namespace FieldSolver
{
    /// <summary>
    /// One engine run made by the solver: the settings it used, what the engine
    /// reported, and the score the solver ranks it by.
    /// </summary>
    public sealed class FieldSolverCandidate
    {
        /// <summary>Gets the 1-based position of this run in the solve.</summary>
        public Int32 RunIndex { get; init; }

        /// <summary>Gets the eight settings this run used.</summary>
        public FieldSolverSettings Settings { get; init; } = new FieldSolverSettings();

        /// <summary>Gets how many of the eight settings differ from the baseline.</summary>
        public Int32 ChangeCount { get; init; }

        /// <summary>Gets whether the engine returned a complete plan.</summary>
        public Boolean Succeeded { get; init; }

        /// <summary>Gets the engine's exception type and message when the run failed, otherwise empty.</summary>
        public String FailureMessage { get; init; } = String.Empty;

        /// <summary>Gets whole-job driven centerline metres, cleanup routes included.</summary>
        public Double DistanceM { get; init; }

        /// <summary>Gets driven metres with a load on.</summary>
        public Double LoadedDistanceM { get; init; }

        /// <summary>Gets driven metres with no load.</summary>
        public Double UnloadedDistanceM { get; init; }

        /// <summary>Gets deterministic guidance hours.</summary>
        public Double DeterministicHours { get; init; }

        /// <summary>Gets the cut the field requires, in BCY.</summary>
        public Double RequiredCutBcy { get; init; }

        /// <summary>Gets the fill the field requires, in BCY of bank cut.</summary>
        public Double RequiredFillBcy { get; init; }

        /// <summary>
        /// Gets the cut the engine reports as not done, in BCY. It includes the
        /// surplus that no plan can place when the field has more cut than fill.
        /// </summary>
        public Double DeferredCutBcy { get; init; }

        /// <summary>
        /// Gets the fill the engine reports as not done, in BCY. It includes the
        /// shortfall that no plan can supply when the field has less cut than fill.
        /// </summary>
        public Double DeferredFillBcy { get; init; }

        /// <summary>
        /// Gets the soil this plan could have moved and did not, in BCY: the larger
        /// of deferred cut less the surplus and deferred fill less the shortfall.
        /// </summary>
        public Double UnfinishedBcy { get; init; }

        /// <summary>
        /// Gets unfinished soil as a percentage of the soil the field allows to be
        /// moved, which is the smaller of required cut and required fill.
        /// </summary>
        public Double UnfinishedPercent { get; init; }

        /// <summary>Gets the number of manual assists in the guidance.</summary>
        public Int32 ManualAssistCount { get; init; }

        /// <summary>Gets the metres charged for the unfinished soil.</summary>
        public Double UnfinishedChargeM { get; init; }

        /// <summary>Gets the ranking score in metres: driven distance plus the unfinished charge. Lower is better.</summary>
        public Double ScoreM { get; init; }

        /// <summary>Gets whether the plan is inside the request's hard limits. Always true for the baseline.</summary>
        public Boolean WithinLimits { get; init; }

        /// <summary>Gets the wall-clock seconds the engine run took.</summary>
        public Double ElapsedSeconds { get; init; }
    }
}
