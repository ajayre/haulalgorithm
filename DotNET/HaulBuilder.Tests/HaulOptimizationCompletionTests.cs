using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    public sealed class HaulOptimizationCompletionTests
    {
        [Fact]
        public void DeferredSummarySeparatesCleanupFromMaterialSurplus()
        {
            List<HaulDeferredWork> Deferred = new List<HaulDeferredWork>
            {
                new HaulDeferredWork(
                    HaulLaneBuilder.WorkKind.Cut,
                    (2, 3), 0, 0.01, 185.778, "test"),
                new HaulDeferredWork(
                    HaulLaneBuilder.WorkKind.Fill,
                    (2, 3), 0, 0.01, 65.549, "test"),
            };

            HaulDeferredWorkSummary Summary = HaulDeferredWorkSummary.Create(
                RequiredCutBcy: 1000.0,
                RequiredFillBcy: 879.771,
                DeferredCutBcy: 185.778,
                DeferredFillBcy: 65.549,
                DeferredWork: Deferred);

            Assert.Equal(120.229, Summary.MaterialSurplusCutBcy, 6);
            Assert.Equal(65.549, Summary.ManualCleanupBcy, 6);
            Assert.Equal(2, Summary.DeferredEntryCount);
            Assert.Equal(1, Summary.DeferredCellCount);
            Assert.Equal(100.0 * 65.549 / 879.771,
                Summary.DeferredFillPercent, 10);
            Assert.Equal(100.0 * 65.549 / 879.771,
                Summary.ManualCleanupPercentOfRequiredFill, 10);
        }

        [Fact]
        public void ConsolidationRemovesUnderfilledCycleAndConservesBothLedgers()
        {
            Grid Grid = MakeGrid(60, 200.0);
            Double BcyPerDepthM = Grid.AreaM2 * Constants.M3_TO_CY;
            List<Pass> Source = new List<Pass>
            {
                MakePass(Grid, (30, 11), (30, 47), 0.03, BcyPerDepthM),
                MakePass(Grid, (30, 12), (30, 46), 0.10, BcyPerDepthM),
                MakePass(Grid, (32, 10), (32, 48), 0.10, BcyPerDepthM),
            };
            foreach (Pass Pass in Source)
            {
                GuidancePlanner.CompactCycleResult Compact =
                    GuidancePlanner.EvaluateCompactCycle(
                        Pass, Grid, BladeM: 1.0, GradeReturn: false);
                Assert.True(Compact.Success, Compact.FailureReason);
            }
            Pass Combined = Passes.BuildPass(
                Grid,
                Source[0].CutCells.Concat(Source[1].CutCells).ToList(),
                Source[0].FillCells.Concat(Source[1].FillCells).ToList(),
                Source[0].LoadBcy + Source[1].LoadBcy,
                Obstacles.EMPTY_OBSTACLES,
                Ordered: false);
            GuidancePlanner.CompactCycleResult CombinedCompact =
                GuidancePlanner.EvaluateCompactCycle(
                    Combined, Grid, BladeM: 1.0, GradeReturn: false);
            Assert.True(CombinedCompact.Success, CombinedCompact.FailureReason);
            Double BeforeDistanceM = Source.Sum(Pass =>
                GuidancePlanner.EvaluateCompactCycle(
                    Pass, Grid, BladeM: 1.0, GradeReturn: false).DistanceM);
            Double CombinedDistanceM = CombinedCompact.DistanceM
                + GuidancePlanner.EvaluateCompactCycle(
                    Source[2], Grid, BladeM: 1.0, GradeReturn: false).DistanceM;
            Assert.True(CombinedDistanceM < BeforeDistanceM,
                $"before={BeforeDistanceM:R} after={CombinedDistanceM:R}");

            HaulRouteConsolidationOptimizer.Result Result =
                HaulRouteConsolidationOptimizer.Optimize(
                    Source,
                    Grid,
                    MaximumLoadBcy: 0.20,
                    MaximumDepthM: 0.20,
                    BladeWidthM: 1.0,
                    GradeReturn: false,
                    MaximumGroupRoutes: 3,
                    MaximumRemovedCycles: 1);

            Assert.Equal(1, Result.RemovedCycles);
            Assert.Equal(2, Result.Passes.Count);
            Assert.True(Result.AfterGuidanceM < Result.BeforeGuidanceM);
            AssertWorkEqual(Source, Result.Passes, FillKind: false);
            AssertWorkEqual(Source, Result.Passes, FillKind: true);
            Assert.Equal(Source.Sum(Pass => Pass.LoadBcy),
                Result.Passes.Sum(Pass => Pass.LoadBcy), 10);
        }

        [Fact]
        public void ResidualEdgeCompletionKeepsFullLedgerValid()
        {
            Grid Grid = MakeGrid(10, 61.0);
            Grid.Existing[0, 0] = 0.061;
            Grid.Existing[9, 9] = -0.061;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);

            HaulResidualEdgeCompleter.Result Result =
                HaulResidualEdgeCompleter.Complete(
                    Array.Empty<Pass>(),
                    Grid,
                    ZeroFlow(Grid),
                    Ledger,
                    HaulLaneBuilder.LaneAxis.NorthSouth,
                    MaximumLoadBcy: 0.10,
                    MaximumRunsPerKind: 12,
                    FlowPenaltyWeight: 0.20,
                    PairRefinementRounds: 1,
                    PairMetric: HaulPairDistanceRefiner.PairMetric.Centroid,
                    BladeWidthM: 1.0,
                    GradeReturn: true);

            Assert.True(Result.GeneratedCycles > 0);
            HaulPlanValidation Validation = Ledger.ValidateComplete(
                Result.Passes, 0.10, 0.061, AllowDeferredWork: true);
            Assert.True(Validation.Success,
                String.Join(Environment.NewLine, Validation.Issues));
        }

        private static Pass MakePass(
            Grid Grid,
            (Int32 Bx, Int32 By) Cut,
            (Int32 Bx, Int32 By) Fill,
            Double DepthM,
            Double BcyPerDepthM)
        {
            return Passes.BuildPass(
                Grid,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (Cut, DepthM),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (Fill, DepthM),
                },
                DepthM * BcyPerDepthM,
                Obstacles.EMPTY_OBSTACLES,
                Ordered: false);
        }

        private static void AssertWorkEqual(
            IEnumerable<Pass> First,
            IEnumerable<Pass> Second,
            Boolean FillKind)
        {
            Dictionary<(Int32 Bx, Int32 By), Double> A = First
                .SelectMany(Pass => FillKind ? Pass.FillCells : Pass.CutCells)
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            Dictionary<(Int32 Bx, Int32 By), Double> B = Second
                .SelectMany(Pass => FillKind ? Pass.FillCells : Pass.CutCells)
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            Assert.Equal(A.Keys.OrderBy(Cell => Cell), B.Keys.OrderBy(Cell => Cell));
            foreach ((Int32 Bx, Int32 By) Cell in A.Keys)
            {
                Assert.Equal(A[Cell], B[Cell], 10);
            }
        }

        private static Grid MakeGrid(Int32 Size, Double MaximumDepthMm)
        {
            Boolean[,] Valid = new Boolean[Size, Size];
            for (Int32 By = 0; By < Size; By += 1)
            {
                for (Int32 Bx = 0; Bx < Size; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0, Size, Size, 0.0, 0.0, 12, true, 0.0, 0.0,
                new Double[Size, Size],
                new Double[Size, Size],
                Valid,
                1.0,
                1.0,
                MaximumDepthMm,
                new List<List<(Double X, Double Y)>>());
        }

        private static FlowField ZeroFlow(Grid Grid) => new FlowField(
            new Double[Grid.Height, Grid.Width],
            new Double[Grid.Height, Grid.Width],
            new Double[Grid.Height, Grid.Width],
            new Double[Grid.Height, Grid.Width],
            new Boolean[Grid.Height, Grid.Width],
            "test");
    }
}
