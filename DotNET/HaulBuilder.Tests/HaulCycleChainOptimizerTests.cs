using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    public sealed class HaulCycleChainOptimizerTests
    {
        [Fact]
        public void WholeFillSwapReducesProductionObjectiveAndConservesWork()
        {
            Grid Grid = MakeGrid(70);
            Double Load = 0.10 * Grid.AreaM2 * Constants.M3_TO_CY;
            TimeParams Parameters = new TimeParams(
                LoadingSpeedMph: 3.0,
                HaulSpeedMph: 6.0,
                DumpingSpeedMph: 3.0,
                ReturnSpeedMph: 10.0,
                AccelTimeS: 2.0,
                DecelTimeS: 2.0,
                FieldEfficiency: 1.0);
            List<Pass> Source = FindImprovingPair(Grid, Load, Parameters);
            (Int32 Bx, Int32 By) ExpectedFirstFill =
                Source[1].FillCells.Single().Cell;
            (Int32 Bx, Int32 By) ExpectedSecondFill =
                Source[0].FillCells.Single().Cell;

            HaulCycleChainOptimizer.Result Result =
                HaulCycleChainOptimizer.Optimize(
                    Source,
                    Grid,
                    BladeWidthM: 1.0,
                    GradeReturn: false,
                    DirectionAware: false,
                    ReturnSpeedMph: 10.0,
                    TurnSpeedMph: 5.0,
                    TurningCircleFt: 20.0,
                    Parameters,
                    MaximumRounds: 1,
                    ExactCandidatesPerRoute: 1,
                    MinimumImprovementSeconds: 0.01);

            Assert.Equal(1, Result.AcceptedSwaps);
            Assert.True(Result.AfterObjectiveSeconds
                < Result.BeforeObjectiveSeconds);
            Assert.True(Result.AfterProxySeconds < Result.BeforeProxySeconds);
            Assert.Contains(Result.Passes[0].FillCells,
                Entry => Entry.Cell == ExpectedFirstFill);
            Assert.Contains(Result.Passes[1].FillCells,
                Entry => Entry.Cell == ExpectedSecondFill);
            AssertWorkEqual(Source, Result.Passes, FillKind: false);
            AssertWorkEqual(Source, Result.Passes, FillKind: true);
        }

        [Fact]
        public void FieldAuditBoundsDoNotExceedMeasuredPortfolioCosts()
        {
            Grid Grid = MakeGrid(70);
            Double Load = 0.10 * Grid.AreaM2 * Constants.M3_TO_CY;
            List<Pass> Source = new List<Pass>
            {
                MakePass(Grid, (30, 11), (30, 40), Load),
                MakePass(Grid, (30, 55), (30, 30), Load),
                MakePass(Grid, (40, 15), (40, 43), Load),
            };

            HaulFieldWideAudit.Result Result = HaulFieldWideAudit.Analyze(
                Source,
                Grid,
                GradeReturn: false,
                DirectionAware: false,
                ReturnSpeedMph: 10.0,
                TurnSpeedMph: 5.0,
                TurningCircleFt: 20.0);

            Assert.True(Result.TransitionLowerBoundSeconds
                <= Result.CurrentTransitionSeconds + 1e-9);
            Assert.True(Result.CutFillLowerBoundM
                <= Result.CurrentCutFillCentroidM + 1e-9);
            Assert.True(Result.JointLowerBoundEquivalentM
                <= Result.CurrentJointEquivalentM + 1e-9);
        }

        private static Pass MakePass(
            Grid Grid,
            (Int32 Bx, Int32 By) Cut,
            (Int32 Bx, Int32 By) Fill,
            Double Load)
        {
            return Passes.BuildPass(
                Grid,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (Cut, 0.10),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (Fill, 0.10),
                },
                Load,
                Obstacles.EMPTY_OBSTACLES,
                Ordered: false);
        }

        private static List<Pass> FindImprovingPair(
            Grid Grid,
            Double Load,
            TimeParams Parameters)
        {
            Routing.TransitionCostEvaluator Transition =
                new Routing.TransitionCostEvaluator(
                    GradeReturn: false,
                    DirectionAware: false,
                    ReturnSpeedMph: 10.0,
                    TurnSpeedMph: 5.0,
                    TurningCircleFt: 20.0);
            for (Int32 FirstFillY = 5; FirstFillY <= 60; FirstFillY += 5)
            {
                for (Int32 SecondFillY = 5;
                    SecondFillY <= 60;
                    SecondFillY += 5)
                {
                    if (FirstFillY == SecondFillY) continue;
                    Pass First = MakePass(
                        Grid, (30, 11), (30, FirstFillY), Load);
                    Pass Second = MakePass(
                        Grid, (30, 55), (30, SecondFillY), Load);
                    Pass SwappedFirst = MakePass(
                        Grid, (30, 11), (30, SecondFillY), Load);
                    Pass SwappedSecond = MakePass(
                        Grid, (30, 55), (30, FirstFillY), Load);
                    GuidancePlanner.CompactCycleResult[] Compact =
                    {
                        GuidancePlanner.EvaluateCompactCycle(
                            First, Grid, 1.0, false),
                        GuidancePlanner.EvaluateCompactCycle(
                            Second, Grid, 1.0, false),
                        GuidancePlanner.EvaluateCompactCycle(
                            SwappedFirst, Grid, 1.0, false),
                        GuidancePlanner.EvaluateCompactCycle(
                            SwappedSecond, Grid, 1.0, false),
                    };
                    if (Compact.Any(Result => !Result.Success)) continue;
                    Double BeforeProxy = TimeModel.EstimatePassTime(
                        Grid, First, Parameters)
                        + TimeModel.EstimatePassTime(Grid, Second, Parameters);
                    Double AfterProxy = TimeModel.EstimatePassTime(
                        Grid, SwappedFirst, Parameters)
                        + TimeModel.EstimatePassTime(
                            Grid, SwappedSecond, Parameters);
                    Double BeforeCompact = Compact[0].DistanceM
                        + Compact[1].DistanceM;
                    Double AfterCompact = Compact[2].DistanceM
                        + Compact[3].DistanceM;
                    Double BeforeObjective = BeforeProxy
                        + Transition.CostSeconds(First, Second);
                    Double AfterObjective = AfterProxy
                        + Transition.CostSeconds(
                            SwappedFirst, SwappedSecond);
                    if ((AfterProxy <= BeforeProxy + 1e-9)
                        && (AfterCompact <= BeforeCompact + 1e-9)
                        && (AfterObjective < BeforeObjective - 0.01))
                    {
                        return new List<Pass> { First, Second };
                    }
                }
            }
            throw new InvalidOperationException(
                "Synthetic fixture could not find a Pareto-improving exchange.");
        }

        private static void AssertWorkEqual(
            IEnumerable<Pass> First,
            IEnumerable<Pass> Second,
            Boolean FillKind)
        {
            Dictionary<(Int32 Bx, Int32 By), Double> A = First
                .SelectMany(Pass => FillKind ? Pass.FillCells : Pass.CutCells)
                .ToDictionary(Entry => Entry.Cell, Entry => Entry.DepthM);
            Dictionary<(Int32 Bx, Int32 By), Double> B = Second
                .SelectMany(Pass => FillKind ? Pass.FillCells : Pass.CutCells)
                .ToDictionary(Entry => Entry.Cell, Entry => Entry.DepthM);
            Assert.Equal(A.Keys.OrderBy(Cell => Cell), B.Keys.OrderBy(Cell => Cell));
            foreach ((Int32 Bx, Int32 By) Cell in A.Keys)
            {
                Assert.Equal(A[Cell], B[Cell], 10);
            }
        }

        private static Grid MakeGrid(Int32 Size)
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
                200.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
