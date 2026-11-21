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
            Int32 CompactOmittedCycles = 0,
            Int32 EdgeCompletionCycles = 0,
            Int32 EdgeOmittedCycles = 0
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
            this.EdgeCompletionCycles = EdgeCompletionCycles;
            this.EdgeOmittedCycles = EdgeOmittedCycles;
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

        /// <summary>Gets retained residual edge cycles.</summary>
        public Int32 EdgeCompletionCycles { get; }

        /// <summary>Gets residual compact-failing cycles left for cleanup.</summary>
        public Int32 EdgeOmittedCycles { get; }

        /// <summary>
        /// Gets or sets the search state. Hauling is the flow ribbon.
        /// Clusters, thin-cut, and correcting build no flow loads. Later rebuilds must
        /// keep the state, because the residual flow pass would otherwise
        /// scrape the dirt those states leave.
        /// </summary>
        public HaulSearchState State { get; set; }

        /// <summary>
        /// Gets or sets whether the search left hauling. Later rebuilds of
        /// this result must keep the flag, because the residual flow pass
        /// would otherwise scrape the dirt clusters or correcting leave.
        /// </summary>
        public Boolean EnteredClusterMode { get; set; }

        /// <summary>
        /// Gets or sets the found-order route where hauling handed off to clusters.
        /// That route is not built. Zero when the trailing mean never falls
        /// below the cluster fraction.
        /// </summary>
        public Int32 ClusterModeStartsAtRoute { get; set; }

        /// <summary>
        /// Gets or sets the found-order route where clusters handed off to thin-cut.
        /// That route is not built. Zero when cluster hauling did not stop
        /// with no haulable work.
        /// </summary>
        public Int32 ThinCutStartsAtRoute { get; set; }

        /// <summary>
        /// Gets or sets the found-order route where thin-cut handed off to correcting.
        /// That route is not built. Zero when thin-cut did not finish.
        /// </summary>
        public Int32 CorrectingStartsAtRoute { get; set; }

        /// <summary>
        /// Gets or sets the field after the coverage passes, in metres.
        /// Positive is high. Invalid cells are NaN. Null when correcting did not run.
        /// A rebuild of this result must keep it, because surplus spread is not a haul.
        /// </summary>
        public Double[,]? CorrectingSurfaceM { get; set; }

        /// <summary>
        /// Copies the search state onto a rebuilt result.
        /// </summary>
        /// <param name="Previous">The result that recorded the state.</param>
        public void CopyClusterModeFrom
            (
            HaulOptimizationResult Previous
            )
        {
            // Rebuilding the pass list must not forget which state the search reached.
            State = Previous.State;
            EnteredClusterMode = Previous.EnteredClusterMode;
            ClusterModeStartsAtRoute = Previous.ClusterModeStartsAtRoute;
            ThinCutStartsAtRoute = Previous.ThinCutStartsAtRoute;
            CorrectingStartsAtRoute = Previous.CorrectingStartsAtRoute;
            CorrectingSurfaceM = Previous.CorrectingSurfaceM;
        }
    }
}
