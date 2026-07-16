using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    public sealed class HaulMultiRouteExchangeOptimizerTests
    {
        [Fact]
        public void ThreeRoutePoolMovesRemoteFillToRouteAlreadyWorkingThere
            (
            )
        {
            Grid Grid = MakeGrid();
            List<Pass> Source = MakeThreeRouteCycle(Grid);

            HaulMultiRouteExchangeOptimizer.Result Result =
                HaulMultiRouteExchangeOptimizer.Optimize(
                    Source,
                    Grid,
                    MaximumDepthM: 0.2,
                    MaximumRounds: 2,
                    MaximumGroupRoutes: 3,
                    MaximumAcceptedGroups: 4,
                    MinimumOutlierDistanceM: 3.0,
                    MinimumImprovementM: 0.1);

            Assert.True(Result.ConservationValidated);
            Assert.True(Result.AcceptedGroups > 0);
            Assert.True(Result.AfterM < Result.BeforeM);
            Assert.DoesNotContain(
                Result.Passes[0].FillCells,
                Entry => Entry.Cell == (0, 10));
            Assert.Contains(
                Result.Passes[1].FillCells,
                Entry => Entry.Cell == (0, 10));
            AssertExactWork(Source, Result.Passes, FillKind: false);
            AssertExactWork(Source, Result.Passes, FillKind: true);
            for (Int32 Index = 0; Index < Source.Count; Index += 1)
            {
                Assert.Equal(
                    Source[Index].CutCells.Sum(Entry => Entry.DepthM),
                    Result.Passes[Index].CutCells.Sum(Entry => Entry.DepthM),
                    10);
                Assert.Equal(
                    Source[Index].FillCells.Sum(Entry => Entry.DepthM),
                    Result.Passes[Index].FillCells.Sum(Entry => Entry.DepthM),
                    10);
                Assert.Equal(Source[Index].LoadBcy, Result.Passes[Index].LoadBcy, 10);
                Assert.DoesNotContain(
                    Result.Passes[Index].CutCells
                        .GroupBy(Entry => Entry.Cell),
                    Group => Group.Count() > 1);
                Assert.DoesNotContain(
                    Result.Passes[Index].FillCells
                        .GroupBy(Entry => Entry.Cell),
                    Group => Group.Count() > 1);
                Assert.All(Result.Passes[Index].CutCells, Entry =>
                    Assert.True(Entry.DepthM <= 0.2 + 1e-9));
                Assert.All(Result.Passes[Index].FillCells, Entry =>
                    Assert.True(Entry.DepthM <= 0.2 + 1e-9));
            }
        }

        [Fact]
        public void ExchangeIsDeterministic
            (
            )
        {
            Grid Grid = MakeGrid();
            List<Pass> Source = MakeThreeRouteCycle(Grid);

            HaulMultiRouteExchangeOptimizer.Result First = Optimize(Source, Grid);
            HaulMultiRouteExchangeOptimizer.Result Second = Optimize(Source, Grid);

            Assert.Equal(First.AcceptedGroups, Second.AcceptedGroups);
            Assert.Equal(First.BeforeM, Second.BeforeM, 10);
            Assert.Equal(First.AfterM, Second.AfterM, 10);
            Assert.Equal(
                Signature(First.Passes),
                Signature(Second.Passes));
        }

        [Fact]
        public void UnprofitableTransactionLeavesEveryRouteUnchanged
            (
            )
        {
            Grid Grid = MakeGrid();
            List<Pass> Source = MakeThreeRouteCycle(Grid);

            HaulMultiRouteExchangeOptimizer.Result Result =
                HaulMultiRouteExchangeOptimizer.Optimize(
                    Source,
                    Grid,
                    MaximumDepthM: 0.2,
                    MaximumRounds: 1,
                    MaximumGroupRoutes: 3,
                    MaximumAcceptedGroups: 2,
                    MinimumOutlierDistanceM: 3.0,
                    MinimumImprovementM: 10000.0);

            Assert.Equal(0, Result.AcceptedGroups);
            Assert.True(Result.RejectedGroups > 0);
            Assert.Equal(Signature(Source), Signature(Result.Passes));
        }

        private static HaulMultiRouteExchangeOptimizer.Result Optimize
            (
            IReadOnlyList<Pass> Source,
            Grid Grid
            ) => HaulMultiRouteExchangeOptimizer.Optimize(
                Source,
                Grid,
                MaximumDepthM: 0.2,
                MaximumRounds: 2,
                MaximumGroupRoutes: 3,
                MaximumAcceptedGroups: 4,
                MinimumOutlierDistanceM: 3.0,
                MinimumImprovementM: 0.1);

        private static List<Pass> MakeThreeRouteCycle
            (
            Grid Grid
            ) => new List<Pass>
            {
                MakePass(
                    Grid,
                    new[] { ((1, 0), 0.10) },
                    new[] { ((0, 0), 0.05), ((0, 10), 0.05) }),
                MakePass(
                    Grid,
                    new[] { ((1, 10), 0.15) },
                    new[] { ((0, 9), 0.10), ((1, 1), 0.05) }),
                MakePass(
                    Grid,
                    new[] { ((2, 0), 0.10) },
                    new[] { ((2, 0), 0.05), ((2, 1), 0.05) }),
            };

        private static Pass MakePass
            (
            Grid Grid,
            IEnumerable<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cut,
            IEnumerable<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fill
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells =
                Cut.ToList();
            return Passes.BuildPass(
                Grid,
                CutCells,
                Fill.ToList(),
                CutCells.Sum(Entry => Entry.DepthM),
                Obstacles.EMPTY_OBSTACLES,
                Ordered: false);
        }

        private static void AssertExactWork
            (
            IEnumerable<Pass> First,
            IEnumerable<Pass> Second,
            Boolean FillKind
            )
        {
            Dictionary<(Int32 Bx, Int32 By), Double> FirstDepth = First
                .SelectMany(Pass => FillKind ? Pass.FillCells : Pass.CutCells)
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(
                    Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            Dictionary<(Int32 Bx, Int32 By), Double> SecondDepth = Second
                .SelectMany(Pass => FillKind ? Pass.FillCells : Pass.CutCells)
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(
                    Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            Assert.Equal(FirstDepth.Keys.OrderBy(Cell => Cell),
                SecondDepth.Keys.OrderBy(Cell => Cell));
            foreach ((Int32 Bx, Int32 By) Cell in FirstDepth.Keys)
            {
                Assert.Equal(FirstDepth[Cell], SecondDepth[Cell], 10);
            }
        }

        private static String Signature
            (
            IEnumerable<Pass> Passes
            ) => String.Join(
                "|",
                Passes.Select(Pass => String.Join(
                    ";",
                    Pass.CutCells.Select(Entry => String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "C{0},{1},{2:R}",
                        Entry.Cell.Bx,
                        Entry.Cell.By,
                        Entry.DepthM))
                    .Concat(Pass.FillCells.Select(Entry => String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "F{0},{1},{2:R}",
                        Entry.Cell.Bx,
                        Entry.Cell.By,
                        Entry.DepthM))))));

        private static Grid MakeGrid
            (
            )
        {
            const Int32 Size = 12;
            Boolean[,] Valid = new Boolean[Size, Size];
            for (Int32 By = 0; By < Size; By += 1)
            {
                for (Int32 Bx = 0; Bx < Size; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
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
                new Double[Size, Size],
                new Double[Size, Size],
                Valid,
                1.0,
                1.0,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
