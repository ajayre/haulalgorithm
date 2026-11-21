using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies end-of-job cleanup cycles from guidance-deferred scraps.
    /// </summary>
    public sealed class HaulDeferredCleanupCompleterTests
    {
        /// <summary>
        /// Verifies deferred cut/fill scraps are regrouped into cleanup haul cycles.
        /// </summary>
        [Fact]
        public void Complete_GroupsDeferredScrapsIntoCleanupPasses
            (
            )
        {
            Grid Grid = MakeGrid();
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            const Double DepthM = 0.04;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutA =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillA =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Bx = 2; Bx <= 6; Bx += 1)
            {
                CutA.Add(((Bx, 4), DepthM));
                FillA.Add(((Bx, 8), DepthM / Grid.Compaction));
            }
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutB =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillB =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Bx = 12; Bx <= 16; Bx += 1)
            {
                CutB.Add(((Bx, 4), DepthM));
                FillB.Add(((Bx, 8), DepthM / Grid.Compaction));
            }

            List<LifecyclePlanner.PassPlan> GuidancePasses =
                new List<LifecyclePlanner.PassPlan>
                {
                    MakeDeferredPassPlan(CutA, FillA),
                    MakeDeferredPassPlan(CutB, FillB),
                };

            HaulDeferredCleanupCompleter.Result Cleanup =
                HaulDeferredCleanupCompleter.Complete(
                    GuidancePasses,
                    Grid,
                    Flow,
                    MaximumLoadBcy: 50.0,
                    MaximumRunsPerKind: 4,
                    FlowPenaltyWeight: 0.2,
                    HeadingAxisDeg: 90.0);

            Assert.NotEmpty(Cleanup.Passes);
            Assert.True(Cleanup.AssignedCutBcy > 0.0);
            Double AssignedCutCells = Cleanup.Passes.Sum(
                Pass => Pass.CutCells.Count);
            Assert.True(AssignedCutCells >= 8);
        }

        /// <summary>
        /// Verifies a fill-only short list returns those cells once and no cycles,
        /// which is the input the empty-cleanup branch must not count again.
        /// </summary>
        [Fact]
        public void Complete_FillOnlyShortList_ReturnsThoseCellsOnce
            (
            )
        {
            Grid Grid = MakeGrid();
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((4, 8), 0.02),
                    ((5, 8), 0.03),
                };
            List<LifecyclePlanner.PassPlan> GuidancePasses =
                new List<LifecyclePlanner.PassPlan>
                {
                    MakeDeferredPassPlan(
                        new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                        FillCells),
                };

            HaulDeferredCleanupCompleter.Result Cleanup =
                HaulDeferredCleanupCompleter.Complete(
                    GuidancePasses,
                    Grid,
                    Flow,
                    MaximumLoadBcy: 50.0,
                    MaximumRunsPerKind: 4,
                    FlowPenaltyWeight: 0.2,
                    HeadingAxisDeg: 90.0);

            Assert.Empty(Cleanup.Passes);
            Assert.Equal(2, Cleanup.RemainingDeferred.Count);
            Double ExpectedBcy = FillCells.Sum(Entry => Entry.DepthM)
                * Grid.AreaM2
                * Constants.M3_TO_CY
                * Grid.Compaction;
            Double ActualBcy = Cleanup.RemainingDeferred.Sum(Entry => Entry.VolumeBcy);
            Assert.InRange(Math.Abs(ActualBcy - ExpectedBcy), 0.0, 1e-9);
        }

        /// <summary>
        /// Verifies a synthetic ledger can be built from deferred cell lists.
        /// </summary>
        [Fact]
        public void FromAllocatedCells_PreservesSuppliedVolumes
            (
            )
        {
            Grid Grid = MakeGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((3, 3), 0.02),
                    ((4, 3), 0.03),
                };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((3, 6), 0.02),
                };

            HaulWorkLedger Ledger = HaulWorkLedger.FromAllocatedCells(
                Grid, CutCells, FillCells);

            Double CutBcy = CutCells.Sum(
                Entry => Entry.DepthM * Ledger.CutBcyPerDepthM);
            Double FillBcy = FillCells.Sum(
                Entry => Entry.DepthM * Ledger.FillBcyPerDepthM);
            Assert.Equal(CutBcy, Ledger.TotalCutBcy, 9);
            Assert.Equal(FillBcy, Ledger.TotalFillBcy, 9);
        }

        /// <summary>Builds an empty-sequence pass plan carrying deferred cells.</summary>
        /// <param name="CutCells">Deferred cut cells.</param>
        /// <param name="FillCells">Deferred fill cells.</param>
        /// <returns>Pass plan marked as fully deferred.</returns>
        private static LifecyclePlanner.PassPlan MakeDeferredPassPlan
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells
            )
        {
            RunSequencer.Result Empty = new RunSequencer.Result(
                true,
                new List<RunSequencer.SequencedRun>(),
                new List<TransitPlanner.Result>(),
                0.0,
                0.0,
                String.Empty);
            return new LifecyclePlanner.PassPlan(
                Empty,
                Empty,
                CutShortOmittedCells: CutCells,
                FillShortOmittedCells: FillCells,
                DeferredEntirePass: true);
        }

        /// <summary>Creates a small all-valid test grid.</summary>
        /// <returns>Grid for cleanup tests.</returns>
        private static Grid MakeGrid
            (
            )
        {
            const Int32 SIZE = 20;
            Boolean[,] Valid = new Boolean[SIZE, SIZE];
            for (Int32 By = 0; By < SIZE; By += 1)
            {
                for (Int32 Bx = 0; Bx < SIZE; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                SIZE,
                SIZE,
                500000.0,
                4000000.0,
                12,
                true,
                0.0,
                0.0,
                new Double[SIZE, SIZE],
                new Double[SIZE, SIZE],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
