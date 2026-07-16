using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Completes the residual field-edge ledger without perturbing the accepted
    /// interior allocation.  Compact-infeasible residual cycles remain explicit
    /// manual-cleanup work rather than weakening guidance constraints.
    /// </summary>
    public static class HaulResidualEdgeCompleter
    {
        /// <summary>Diagnostics and retained edge cycles.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one residual completion result.</summary>
            public Result
                (
                List<Pass> Passes,
                Int32 GeneratedCycles,
                Int32 CompactFailures,
                Int32 OmittedCycles,
                Int32 PairRefinementSwaps,
                Int32 PairRepairSwaps
                )
            {
                this.Passes = Passes;
                this.GeneratedCycles = GeneratedCycles;
                this.CompactFailures = CompactFailures;
                this.OmittedCycles = OmittedCycles;
                this.PairRefinementSwaps = PairRefinementSwaps;
                this.PairRepairSwaps = PairRepairSwaps;
            }

            /// <summary>Gets retained automatic edge cycles.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets generated cycles before compact omission.</summary>
            public Int32 GeneratedCycles { get; }

            /// <summary>Gets compact failures before bounded omission.</summary>
            public Int32 CompactFailures { get; }

            /// <summary>Gets cycles explicitly left for manual cleanup.</summary>
            public Int32 OmittedCycles { get; }

            /// <summary>Gets accepted equal-load fill swaps.</summary>
            public Int32 PairRefinementSwaps { get; }

            /// <summary>Gets accepted compact pair-repair swaps.</summary>
            public Int32 PairRepairSwaps { get; }
        }

        /// <summary>Builds and validates the residual edge batch.</summary>
        static public Result Complete
            (
            IReadOnlyList<Pass> InteriorPasses,
            Grid Grid,
            FlowField Flow,
            HaulWorkLedger FullLedger,
            HaulLaneBuilder.LaneAxis Axis,
            Double MaximumLoadBcy,
            Int32 MaximumRunsPerKind,
            Double FlowPenaltyWeight,
            Int32 PairRefinementRounds,
            HaulPairDistanceRefiner.PairMetric PairMetric,
            Double BladeWidthM,
            Boolean GradeReturn,
            ObstacleField? Obstacles = null,
            Double? HeadingAxisDeg = null
            )
        {
            if (InteriorPasses == null)
            {
                throw new ArgumentNullException(nameof(InteriorPasses));
            }
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            HaulWorkLedger ResidualLedger =
                FullLedger.BuildResidual(InteriorPasses);
            HaulOptimizationResult Generated = HaulCycleOptimizer.Optimize(
                Grid,
                Flow,
                ResidualLedger,
                Axis,
                MaximumLoadBcy,
                MaximumRunsPerKind,
                DeferEdgeWork: false,
                FlowPenaltyWeight,
                ActiveObstacles,
                HeadingAxisDeg);
            List<Pass> EdgePasses = Generated.PassResult.Passes;
            Int32 PairSwaps = 0;
            if ((PairRefinementRounds > 0) && (EdgePasses.Count > 1))
            {
                HaulPairDistanceRefiner.Result Refined =
                    HaulPairDistanceRefiner.Refine(
                        EdgePasses,
                        Grid,
                        PairRefinementRounds,
                        PairMetric,
                        Seed: 1,
                        Obstacles: ActiveObstacles);
                EdgePasses = Refined.Passes;
                PairSwaps = Refined.Swaps;
            }
            HaulPairRepair.Result PairRepair = HaulPairRepair.Repair(
                EdgePasses,
                Grid,
                BladeWidthM,
                GradeReturn,
                Obstacles: ActiveObstacles);
            EdgePasses = PairRepair.Passes;
            Int32 FailureCount = PairRepair.FinalFailures;
            Int32 OmittedCycles = 0;
            if (FailureCount > 0)
            {
                HaulDifficultCycleFilter.Result Filter =
                    HaulDifficultCycleFilter.Apply(
                        EdgePasses,
                        Grid,
                        BladeWidthM,
                        GradeReturn,
                        FailureCount,
                        HaulDifficultCycleFilter.OmissionPriority.SimplestFirst);
                EdgePasses = Filter.Passes;
                OmittedCycles = Filter.OmittedCycles;
            }
            HaulPlanValidation Validation = FullLedger.ValidateComplete(
                InteriorPasses.Concat(EdgePasses).ToList(),
                MaximumLoadBcy,
                Grid.MaxDepthMm / 1000.0,
                AllowDeferredWork: true);
            if (!Validation.Success)
            {
                throw new InvalidOperationException(
                    "Residual edge completion failed the full ledger: "
                    + String.Join("; ", Validation.Issues.Take(10)));
            }
            return new Result(
                EdgePasses,
                Generated.PassResult.Passes.Count,
                FailureCount,
                OmittedCycles,
                PairSwaps,
                PairRepair.SwapCount);
        }
    }
}
