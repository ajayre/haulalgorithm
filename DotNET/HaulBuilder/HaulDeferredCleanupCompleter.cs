using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Rebuilds guidance-deferred cut/fill scraps into end-of-job haul cycles,
    /// grouping nearby work by lane/area before appending to the route.
    /// </summary>
    static public class HaulDeferredCleanupCompleter
    {
        /// <summary>Diagnostics and retained cleanup cycles.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one cleanup completion result.</summary>
            /// <param name="Passes">Area-grouped cleanup cycles.</param>
            /// <param name="CutCells">Deferred cut cells consumed into the cleanup ledger.</param>
            /// <param name="FillCells">Deferred fill cells consumed into the cleanup ledger.</param>
            /// <param name="RemainingDeferred">Unpaired leftovers still needing manual cleanup.</param>
            /// <param name="AssignedCutBcy">Bank cut volume placed into cleanup cycles.</param>
            /// <param name="AssignedFillBcy">Bank fill volume placed into cleanup cycles.</param>
            public Result
                (
                List<Pass> Passes,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
                IReadOnlyList<HaulDeferredWork> RemainingDeferred,
                Double AssignedCutBcy,
                Double AssignedFillBcy
                )
            {
                this.Passes = Passes;
                this.CutCells = CutCells;
                this.FillCells = FillCells;
                this.RemainingDeferred = RemainingDeferred;
                this.AssignedCutBcy = AssignedCutBcy;
                this.AssignedFillBcy = AssignedFillBcy;
            }

            /// <summary>Gets cleanup cycles to append at end of job.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets cut cells taken from guidance deferral into the cleanup ledger.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells { get; }

            /// <summary>Gets fill cells taken from guidance deferral into the cleanup ledger.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells { get; }

            /// <summary>Gets leftovers that could not form balanced cleanup cycles.</summary>
            public IReadOnlyList<HaulDeferredWork> RemainingDeferred { get; }

            /// <summary>Gets assigned cleanup cut volume in BCY.</summary>
            public Double AssignedCutBcy { get; }

            /// <summary>Gets assigned cleanup fill volume in BCY.</summary>
            public Double AssignedFillBcy { get; }
        }

        /// <summary>
        /// Builds area-grouped cleanup cycles from guidance-deferred pass cells.
        /// </summary>
        /// <param name="GuidancePasses">Per-pass guidance plans with short/failed omissions.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Flow">Transport flow field.</param>
        /// <param name="MaximumLoadBcy">Scraper capacity in bank cubic yards.</param>
        /// <param name="MaximumRunsPerKind">Nearby lanes grouped per cycle side.</param>
        /// <param name="FlowPenaltyWeight">Flow-aware pair cost weight.</param>
        /// <param name="HeadingAxisDeg">Optional user lane heading.</param>
        /// <param name="Obstacles">Optional exclusion obstacles.</param>
        /// <param name="ToleranceMm">Ledger depth tolerance.</param>
        /// <returns>Cleanup cycles plus any still-unpaired deferred work.</returns>
        static public Result Complete
            (
            IReadOnlyList<LifecyclePlanner.PassPlan> GuidancePasses,
            Grid Grid,
            FlowField Flow,
            Double MaximumLoadBcy,
            Int32 MaximumRunsPerKind,
            Double FlowPenaltyWeight,
            Double? HeadingAxisDeg = null,
            ObstacleField? Obstacles = null,
            Double ToleranceMm = Constants.DEFAULT_TOLERANCE_MM
            )
        {
            if (GuidancePasses == null)
            {
                throw new ArgumentNullException(nameof(GuidancePasses));
            }
            if (Grid == null)
            {
                throw new ArgumentNullException(nameof(Grid));
            }
            if (Flow == null)
            {
                throw new ArgumentNullException(nameof(Flow));
            }

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            foreach (LifecyclePlanner.PassPlan PassPlan in GuidancePasses)
            {
                CutCells.AddRange(PassPlan.CutShortOmittedCells);
                FillCells.AddRange(PassPlan.FillShortOmittedCells);
            }
            if ((CutCells.Count == 0) || (FillCells.Count == 0))
            {
                // Intent: cleanup cycles need both sides; unpaired scraps stay deferred.
                IReadOnlyList<HaulDeferredWork> Unpaired = BuildUnpairedDeferred(
                    Grid, CutCells, FillCells);
                return new Result(
                    new List<Pass>(),
                    CutCells,
                    FillCells,
                    Unpaired,
                    0.0,
                    0.0);
            }

            HaulWorkLedger CleanupLedger = HaulWorkLedger.FromAllocatedCells(
                Grid,
                CutCells,
                FillCells,
                ToleranceMm);
            // Intent: group nearby deferred lanes into local area cycles, then accept
            // any scraps that still cannot pair as remaining manual cleanup.
            HaulOptimizationResult Optimized = HaulCycleOptimizer.Optimize(
                Grid,
                Flow,
                CleanupLedger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                MaximumLoadBcy,
                MaximumRunsPerKind,
                DeferEdgeWork: false,
                FlowPenaltyWeight,
                Obstacles ?? EMPTY_OBSTACLES,
                HeadingAxisDeg,
                AllowAnyDeferredRemainder: true);

            List<HaulDeferredWork> Remaining = Optimized.DeferredWork
                .Select(Entry => new HaulDeferredWork(
                    Entry.Kind,
                    Entry.Cell,
                    Entry.LayerIndex,
                    Entry.DepthM,
                    Entry.VolumeBcy,
                    HaulDeferredWorkReasons.GUIDANCE_SEQUENCE_FAILED))
                .ToList();
            return new Result(
                Optimized.PassResult.Passes,
                CutCells,
                FillCells,
                Remaining,
                Optimized.TotalAssignedBcy,
                Optimized.TotalAssignedBcy);
        }

        /// <summary>
        /// Builds deferred entries when only one side of the scrap ledger exists.
        /// </summary>
        /// <param name="Grid">Field grid for volume conversion.</param>
        /// <param name="CutCells">Deferred cut cells.</param>
        /// <param name="FillCells">Deferred fill cells.</param>
        /// <returns>Deferred-work list for the non-empty side.</returns>
        private static IReadOnlyList<HaulDeferredWork> BuildUnpairedDeferred
            (
            Grid Grid,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells
            )
        {
            Double CutBcyPerDepthM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillBcyPerDepthM = CutBcyPerDepthM * Grid.Compaction;
            List<HaulDeferredWork> Output = new List<HaulDeferredWork>();
            for (Int32 Index = 0; Index < CutCells.Count; Index += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = CutCells[Index];
                if (Entry.DepthM <= 0.0)
                {
                    continue;
                }
                Output.Add(new HaulDeferredWork(
                    HaulLaneBuilder.WorkKind.Cut,
                    Entry.Cell,
                    Index,
                    Entry.DepthM,
                    Entry.DepthM * CutBcyPerDepthM,
                    HaulDeferredWorkReasons.GUIDANCE_SEQUENCE_FAILED));
            }
            for (Int32 Index = 0; Index < FillCells.Count; Index += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = FillCells[Index];
                if (Entry.DepthM <= 0.0)
                {
                    continue;
                }
                Output.Add(new HaulDeferredWork(
                    HaulLaneBuilder.WorkKind.Fill,
                    Entry.Cell,
                    Index,
                    Entry.DepthM,
                    Entry.DepthM * FillBcyPerDepthM,
                    HaulDeferredWorkReasons.GUIDANCE_SEQUENCE_FAILED));
            }
            return Output;
        }
    }
}
