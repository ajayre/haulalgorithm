using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    /// <summary>Checks pairwise trades that pull interleaved fill cells into solid runs.</summary>
    public sealed class HaulRouteTradeOptimizerTests
    {
        /// <summary>
        /// Two loads whose fill blocks alternate along one line become two solid runs.
        /// Each load keeps its volume. A load outside the gap is left alone.
        /// </summary>
        [Fact]
        public void InterleavedFillsBecomeTwoSolidRuns
            (
            )
        {
            const Int32 WIDTH = 200;
            const Int32 HEIGHT = 40;
            const Int32 FILL_ROW = 24;
            const Int32 CUT_ROW = 8;
            Grid Grid = MakeGrid(WIDTH, HEIGHT);
            // Blocks of 40 m sit in the order west, gap, east. Each load owns every
            // other block, so the pair is interleaved. The cuts sit at the end each
            // load should keep after the trade.
            Pass West = Load(
                Grid,
                Range(10, 49, CUT_ROW),
                Range(10, 49, FILL_ROW).Concat(Range(90, 129, FILL_ROW)).ToArray());
            Pass East = Load(
                Grid,
                Range(130, 169, CUT_ROW),
                Range(50, 89, FILL_ROW).Concat(Range(130, 169, FILL_ROW)).ToArray());
            Pass Outside = Load(
                Grid,
                Range(10, 20, 36),
                Range(10, 20, 37));
            HashSet<(Int32 Bx, Int32 By)> OutsideFill = Outside.FillCells
                .Select(Entry => Entry.Cell)
                .ToHashSet();
            HashSet<(Int32 Bx, Int32 By)> BeforeCells = West.FillCells
                .Concat(East.FillCells)
                .Select(Entry => Entry.Cell)
                .ToHashSet();

            HaulRouteTradeOptimizer.Result Result = HaulRouteTradeOptimizer.Optimize(
                new[] { West, East, Outside },
                Grid,
                BladeWidthM: 2.6,
                MaximumDepthM: 0.061,
                GradeReturn: true,
                MaximumPasses: 4,
                Obstacles: Obstacles.EMPTY_OBSTACLES);

            Assert.True(Result.AcceptedTrades >= 1, Describe(Result, West, East, Grid));
            Assert.Equal(
                OutsideFill,
                Result.Passes[2].FillCells.Select(Entry => Entry.Cell).ToHashSet());
            List<List<Int32>> Blocks = Result.Passes
                .Take(2)
                .Select(Pass => Pass.FillCells
                    .Select(Entry => Entry.Cell.Bx)
                    .OrderBy(Value => Value)
                    .ToList())
                .OrderBy(Columns => Columns[0])
                .ToList();
            Assert.Equal(Enumerable.Range(10, 80).ToList(), Blocks[0]);
            Assert.Equal(Enumerable.Range(90, 80).ToList(), Blocks[1]);
            Assert.Equal(
                BeforeCells,
                Result.Passes
                    .Take(2)
                    .SelectMany(Pass => Pass.FillCells.Select(Entry => Entry.Cell))
                    .ToHashSet());
            Assert.All(Result.Passes.Take(2), Pass =>
            {
                Assert.Equal(80, Pass.FillCells.Count);
                Assert.Equal(4.0, Pass.FillCells.Sum(Entry => Entry.DepthM), 6);
            });
        }

        /// <summary>
        /// Two solid blocks that already sit end to end are not a trade.
        /// They are not counted as a refusal.
        /// </summary>
        [Fact]
        public void SolidBlocksEndToEndAreLeftAlone
            (
            )
        {
            Grid Grid = MakeGrid(120, 20);
            Pass West = Load(Grid, Range(10, 40, 4), Range(10, 40, 12));
            Pass East = Load(Grid, Range(50, 80, 4), Range(50, 80, 12));
            List<(Int32 Bx, Int32 By)> WestFill = West.FillCells
                .Select(Entry => Entry.Cell)
                .OrderBy(Cell => Cell.Bx)
                .ToList();
            List<(Int32 Bx, Int32 By)> EastFill = East.FillCells
                .Select(Entry => Entry.Cell)
                .OrderBy(Cell => Cell.Bx)
                .ToList();

            HaulRouteTradeOptimizer.Result Result = HaulRouteTradeOptimizer.Optimize(
                new[] { West, East },
                Grid,
                BladeWidthM: 2.6,
                MaximumDepthM: 0.061,
                GradeReturn: true,
                MaximumPasses: 4,
                Obstacles: Obstacles.EMPTY_OBSTACLES);

            Assert.Equal(0, Result.AcceptedTrades);
            Assert.Equal(0, Result.RejectedTrades);
            Assert.Equal(
                WestFill,
                Result.Passes[0].FillCells.Select(Entry => Entry.Cell).OrderBy(Cell => Cell.Bx));
            Assert.Equal(
                EastFill,
                Result.Passes[1].FillCells.Select(Entry => Entry.Cell).OrderBy(Cell => Cell.Bx));
        }

        /// <summary>
        /// Two loads that hold successive layers of the same cells are not stacked
        /// into one pass. One pass cannot take more than the machine depth.
        /// </summary>
        [Fact]
        public void StackedLayersStaySplitAtMaxDepth
            (
            )
        {
            Grid Grid = MakeGrid(80, 20);
            (Int32 Bx, Int32 By)[] Fill = Range(10, 40, 12);
            Pass First = Load(Grid, Range(10, 40, 4), Fill, 0.061);
            Pass Second = Load(Grid, Range(10, 40, 6), Fill, 0.061);

            HaulRouteTradeOptimizer.Result Result = HaulRouteTradeOptimizer.Optimize(
                new[] { First, Second },
                Grid,
                BladeWidthM: 2.6,
                MaximumDepthM: 0.061,
                GradeReturn: true,
                MaximumPasses: 4,
                Obstacles: Obstacles.EMPTY_OBSTACLES);

            Assert.Equal(0, Result.AcceptedTrades);
            Assert.Equal(0, Result.RejectedTrades);
            Assert.All(Result.Passes, Pass =>
            {
                Assert.All(Pass.CutCells, Entry => Assert.True(Entry.DepthM <= 0.061 + 1e-9));
                Assert.All(Pass.FillCells, Entry => Assert.True(Entry.DepthM <= 0.061 + 1e-9));
                Assert.Equal(0.061 * Fill.Length, Pass.FillCells.Sum(Entry => Entry.DepthM), 6);
            });
        }

        /// <summary>Formats a refusal so a spacing change can be judged from the message.</summary>
        /// <param name="Result">Trade result.</param>
        /// <param name="West">Western load before the trade.</param>
        /// <param name="East">Eastern load before the trade.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>Accepted, rejected, and the before-cycle reasons.</returns>
        private static String Describe
            (
            HaulRouteTradeOptimizer.Result Result,
            Pass West,
            Pass East,
            Grid Grid
            )
        {
            GuidancePlanner.CompactCycleResult Left =
                GuidancePlanner.EvaluateCompactCycle(West, Grid, 2.6, true);
            GuidancePlanner.CompactCycleResult Right =
                GuidancePlanner.EvaluateCompactCycle(East, Grid, 2.6, true);
            return $"accepted={Result.AcceptedTrades} rejected={Result.RejectedTrades} "
                + $"west={Left.Success}:{Left.FailureReason} "
                + $"cuts={Left.CutRunCount} fills={Left.FillRunCount} "
                + $"east={Right.Success}:{Right.FailureReason} "
                + $"cuts={Right.CutRunCount} fills={Right.FillRunCount}";
        }

        /// <summary>Builds an open grid of one-metre cells.</summary>
        /// <param name="Width">Columns.</param>
        /// <param name="Height">Rows.</param>
        /// <returns>The grid.</returns>
        private static Grid MakeGrid
            (
            Int32 Width,
            Int32 Height
            )
        {
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                Width,
                Height,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[Height, Width],
                new Double[Height, Width],
                Valid,
                1.0,
                1.0,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>One east-west strip of cells on a row.</summary>
        /// <param name="First">First column, inclusive.</param>
        /// <param name="Last">Last column, inclusive.</param>
        /// <param name="Row">Row index.</param>
        /// <returns>The cells.</returns>
        private static (Int32 Bx, Int32 By)[] Range
            (
            Int32 First,
            Int32 Last,
            Int32 Row
            )
        {
            return Enumerable.Range(First, Last - First + 1)
                .Select(Column => (Column, Row))
                .ToArray();
        }

        /// <summary>Builds one load from cut and fill cells.</summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cut">Cut cells.</param>
        /// <param name="Fill">Fill cells.</param>
        /// <param name="DepthM">Depth of every cell. Defaults to a layer under the pass limit.</param>
        /// <returns>The load.</returns>
        private static Pass Load
            (
            Grid Grid,
            (Int32 Bx, Int32 By)[] Cut,
            (Int32 Bx, Int32 By)[] Fill,
            Double DepthM = 0.05
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = Cut
                .Select(Cell => (Cell, DepthM))
                .ToList();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells = Fill
                .Select(Cell => (Cell, DepthM))
                .ToList();
            return Passes.BuildPass(
                Grid,
                CutCells,
                FillCells,
                CutCells.Sum(Entry => Entry.DepthM),
                Obstacles.EMPTY_OBSTACLES,
                Ordered: false);
        }
    }
}
