using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>Result and diagnostics from one complete optimized allocation.</summary>
    public sealed class HaulOptimizationResult
    {
        /// <summary>Initializes one optimization result.</summary>
        public HaulOptimizationResult
            (
            PassResult PassResult,
            HaulLaneBuilder.LaneAxis Axis,
            Int32 CutLaneCount,
            Int32 FillLaneCount,
            Double TotalAssignedBcy,
            Double UnmatchedCutBcy,
            Double UnmatchedFillBcy,
            IReadOnlyList<HaulDeferredWork> DeferredWork,
            HaulPlanValidation Validation,
            Int32 PairRefinementSwaps = 0,
            Double PairRefinementBeforeM = 0.0,
            Double PairRefinementAfterM = 0.0,
            Int32 CompactPreflightFailures = 0,
            Int32 CompactOmittedCycles = 0
            )
        {
            this.PassResult = PassResult;
            this.Axis = Axis;
            this.CutLaneCount = CutLaneCount;
            this.FillLaneCount = FillLaneCount;
            this.TotalAssignedBcy = TotalAssignedBcy;
            this.UnmatchedCutBcy = UnmatchedCutBcy;
            this.UnmatchedFillBcy = UnmatchedFillBcy;
            this.DeferredWork = DeferredWork;
            this.Validation = Validation;
            this.PairRefinementSwaps = PairRefinementSwaps;
            this.PairRefinementBeforeM = PairRefinementBeforeM;
            this.PairRefinementAfterM = PairRefinementAfterM;
            this.CompactPreflightFailures = CompactPreflightFailures;
            this.CompactOmittedCycles = CompactOmittedCycles;
        }

        /// <summary>Gets generated passes.</summary>
        public PassResult PassResult { get; }

        /// <summary>Gets selected lane axis.</summary>
        public HaulLaneBuilder.LaneAxis Axis { get; }

        /// <summary>Gets source cut lane count.</summary>
        public Int32 CutLaneCount { get; }

        /// <summary>Gets destination fill lane count.</summary>
        public Int32 FillLaneCount { get; }

        /// <summary>Gets assigned balanced bank volume.</summary>
        public Double TotalAssignedBcy { get; }

        /// <summary>Gets unmatched cut bank volume.</summary>
        public Double UnmatchedCutBcy { get; }

        /// <summary>Gets unmatched fill bank volume.</summary>
        public Double UnmatchedFillBcy { get; }

        /// <summary>Gets every exact unassigned cell-depth layer.</summary>
        public IReadOnlyList<HaulDeferredWork> DeferredWork { get; }

        /// <summary>Gets exact plan validation.</summary>
        public HaulPlanValidation Validation { get; }

        /// <summary>Gets accepted equal-load fill-pair exchanges.</summary>
        public Int32 PairRefinementSwaps { get; }

        /// <summary>Gets the pair proxy before refinement in metres.</summary>
        public Double PairRefinementBeforeM { get; }

        /// <summary>Gets the pair proxy after refinement in metres.</summary>
        public Double PairRefinementAfterM { get; }

        /// <summary>Gets compact-guidance preflight failures before omission.</summary>
        public Int32 CompactPreflightFailures { get; }

        /// <summary>Gets whole balanced cycles explicitly deferred after preflight.</summary>
        public Int32 CompactOmittedCycles { get; }
    }
}
