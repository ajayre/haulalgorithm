using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    public sealed class HaulCycleOptimizerTests
    {
        [Fact]
        public void ProductionDefaultRemainsLegacyUntilExactGatePasses
            (
            )
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();

            Assert.False(Options.EnableHaulOptimization);
            Assert.True(Options.HaulOptimizationDeferEdgeWork);
            Assert.True(Options.HaulOptimizationCompleteDeferredEdge);
            Assert.Equal(12, Options.HaulOptimizationEdgeMaxRunsPerKind);
            Assert.True(Options.EnableHaulRouteConsolidation);
            Assert.Equal(8, Options.HaulOptimizationRouteConsolidationGroupRoutes);
            Assert.Equal(50, Options.HaulOptimizationRouteConsolidationMaximumCycles);
            Assert.Equal(10, Options.HaulOptimizationMaxRunsPerKind);
            Assert.Equal(8, Options.HaulOptimizationPairRefinementRounds);
            Assert.False(Options.EnableHaulCycleChainOptimization);
            Assert.Equal(1, Options.HaulOptimizationCycleChainRounds);
            Assert.Equal(1,
                Options.HaulOptimizationCycleChainCandidatesPerRoute);
            Assert.Equal(200, Options.HaulOptimizationReorderWindow);
            Assert.Equal(0, Options.HaulOptimizationMaximumCompactOmissions);
            Assert.Equal(
                HaulDifficultCycleFilter.OmissionPriority.MostFragmentedFirst,
                Options.HaulOptimizationCompactOmissionPriority);
            Assert.Equal(
                HaulLaneBuilder.LaneAxis.NorthSouth,
                Options.HaulOptimizationAxis);
        }

        [Fact]
        public void ProducesBalancedCapacityBoundedCompletePlan
            (
            )
        {
            Grid Grid = MakeBalancedGrid();
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);
            FlowField Flow = ZeroFlow(Grid);

            HaulOptimizationResult Result = HaulCycleOptimizer.Optimize(
                Grid,
                Flow,
                Ledger,
                HaulLaneBuilder.LaneAxis.EastWest,
                0.20,
                2,
                false,
                0.20,
                Obstacles.EMPTY_OBSTACLES);

            Assert.True(Result.Validation.Success, String.Join(Environment.NewLine, Result.Validation.Issues));
            Assert.NotEmpty(Result.PassResult.Passes);
            Assert.Empty(Result.DeferredWork);
            Assert.All(Result.PassResult.Passes, Pass =>
            {
                Double CutBcy = Pass.CutCells.Sum(
                    Entry => Entry.DepthM * Ledger.CutBcyPerDepthM);
                Double FillBcy = Pass.FillCells.Sum(
                    Entry => Entry.DepthM * Ledger.FillBcyPerDepthM);
                Assert.InRange(CutBcy, 0.0, 0.20000001);
                Assert.InRange(FillBcy, CutBcy - 1e-8, CutBcy + 1e-8);
                Assert.Equal(
                    Pass.CutCells.Count,
                    Pass.CutCells.Select(Entry => Entry.Cell).Distinct().Count());
                Assert.Equal(
                    Pass.FillCells.Count,
                    Pass.FillCells.Select(Entry => Entry.Cell).Distinct().Count());
            });
            HaulPlanValidation Exact = Ledger.ValidateComplete(
                Result.PassResult.Passes, 0.20, 0.061, false);
            Assert.True(Exact.Success, String.Join(Environment.NewLine, Exact.Issues));
        }

        [Fact]
        public void ArbitraryAxisProducesBalancedPlanAndPreservesAxisMetadata
            (
            )
        {
            Grid Grid = MakeBalancedGrid();
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);

            HaulOptimizationResult Result = HaulCycleOptimizer.Optimize(
                Grid,
                ZeroFlow(Grid),
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                0.20,
                2,
                false,
                0.20,
                Obstacles.EMPTY_OBSTACLES,
                HeadingAxisDeg: 217.5);

            Assert.True(
                Result.Validation.Success,
                String.Join(Environment.NewLine, Result.Validation.Issues));
            Assert.NotEmpty(Result.PassResult.Passes);
            Assert.All(Result.PassResult.Passes, Pass =>
            {
                Assert.Equal(37.5, Pass.CutHeadingDeg!.Value, 12);
                Assert.Equal(37.5, Pass.FillHeadingDeg!.Value, 12);
            });
            HaulPlanValidation Exact = Ledger.ValidateComplete(
                Result.PassResult.Passes, 0.20, 0.061, false);
            Assert.True(
                Exact.Success,
                String.Join(Environment.NewLine, Exact.Issues));
        }

        [Fact]
        public void EdgeDeferralIsExplicitAndNeverCountedAsCompleted
            (
            )
        {
            Grid Grid = MakeBalancedGrid();
            Grid.Existing[0, 0] = 0.061;
            Grid.Existing[9, 9] = -0.061;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);

            HaulOptimizationResult Result = HaulCycleOptimizer.Optimize(
                Grid,
                ZeroFlow(Grid),
                Ledger,
                HaulLaneBuilder.LaneAxis.EastWest,
                0.20,
                2,
                true,
                0.20,
                Obstacles.EMPTY_OBSTACLES);

            Assert.Contains(Result.DeferredWork, Entry =>
                Entry.Kind == HaulLaneBuilder.WorkKind.Cut
                && Entry.Cell == (0, 0)
                && Entry.Reason == "FIELD_EDGE_HEADING");
            Assert.Contains(Result.DeferredWork, Entry =>
                Entry.Kind == HaulLaneBuilder.WorkKind.Fill
                && Entry.Cell == (9, 9)
                && Entry.Reason == "FIELD_EDGE_HEADING");
            Assert.True(Result.UnmatchedCutBcy > 0.0);
            Assert.True(Result.UnmatchedFillBcy > 0.0);
        }

        private static Grid MakeBalancedGrid
            (
            )
        {
            const Int32 Size = 10;
            Double[,] Existing = new Double[Size, Size];
            Boolean[,] Valid = new Boolean[Size, Size];
            for (Int32 By = 0; By < Size; By += 1)
            {
                for (Int32 Bx = 0; Bx < Size; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            for (Int32 Bx = 2; Bx <= 5; Bx += 1)
            {
                Existing[3, Bx] = 0.122;
                Existing[6, Bx] = -0.122;
            }
            return new Grid(
                1.0,
                Size,
                Size,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                Existing,
                new Double[Size, Size],
                Valid,
                1.3,
                1.0,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        private static FlowField ZeroFlow
            (
            Grid Grid
            ) => new FlowField(
                new Double[Grid.Height, Grid.Width],
                new Double[Grid.Height, Grid.Width],
                new Double[Grid.Height, Grid.Width],
                new Double[Grid.Height, Grid.Width],
                new Boolean[Grid.Height, Grid.Width],
                "test");
    }
}
