using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    public sealed class HaulPairDistanceRefinerTests
    {
        [Fact]
        public void EqualLoadExchangeReducesObjectiveWithoutChangingWork
            (
            )
        {
            Grid Grid = MakeGrid();
            List<Pass> Source = new List<Pass>
            {
                MakePass(Grid, (1, 1), (8, 8)),
                MakePass(Grid, (8, 1), (1, 8)),
            };

            HaulPairDistanceRefiner.Result Result =
                HaulPairDistanceRefiner.Refine(
                    Source, Grid, MaximumRounds: 8);

            Assert.Equal(1, Result.Swaps);
            Assert.True(Result.AfterM < Result.BeforeM);
            Assert.Equal(
                Source.SelectMany(Pass => Pass.CutCells)
                    .Select(Entry => Entry.Cell).OrderBy(Cell => Cell).ToArray(),
                Result.Passes.SelectMany(Pass => Pass.CutCells)
                    .Select(Entry => Entry.Cell).OrderBy(Cell => Cell).ToArray());
            Assert.Equal(
                Source.SelectMany(Pass => Pass.FillCells)
                    .Select(Entry => Entry.Cell).OrderBy(Cell => Cell).ToArray(),
                Result.Passes.SelectMany(Pass => Pass.FillCells)
                    .Select(Entry => Entry.Cell).OrderBy(Cell => Cell).ToArray());
            Assert.All(Result.Passes, Pass =>
                Assert.Equal(0.1, Pass.LoadBcy, 10));
        }

        [Fact]
        public void CanonicalRefinementIsDeterministic
            (
            )
        {
            Grid Grid = MakeGrid();
            List<Pass> Source = new List<Pass>
            {
                MakePass(Grid, (1, 1), (8, 8)),
                MakePass(Grid, (8, 1), (1, 8)),
                MakePass(Grid, (2, 2), (7, 7)),
                MakePass(Grid, (7, 2), (2, 7)),
            };

            HaulPairDistanceRefiner.Result First =
                HaulPairDistanceRefiner.Refine(Source, Grid);
            HaulPairDistanceRefiner.Result Second =
                HaulPairDistanceRefiner.Refine(Source, Grid);

            Assert.Equal(First.Swaps, Second.Swaps);
            Assert.Equal(First.AfterM, Second.AfterM, 10);
            Assert.Equal(
                First.Passes.Select(Pass => Pass.FillCells[0].Cell),
                Second.Passes.Select(Pass => Pass.FillCells[0].Cell));
        }

        private static Pass MakePass
            (
            Grid Grid,
            (Int32 Bx, Int32 By) Cut,
            (Int32 Bx, Int32 By) Fill
            ) => Passes.BuildPass(
                Grid,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (Cut, 0.1),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (Fill, 0.1),
                },
                0.1,
                Obstacles.EMPTY_OBSTACLES,
                Ordered: true);

        private static Grid MakeGrid
            (
            )
        {
            const Int32 Size = 10;
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
