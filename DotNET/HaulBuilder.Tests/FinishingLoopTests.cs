using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// The finishing loop cuts to the target, fills to the target, and uses the
    /// tolerance cap only for the field's imbalance.
    /// </summary>
    public sealed class FinishingLoopTests
    {
        /// <summary>Bowl used by the production scraper, in bank cubic yards.</summary>
        private const Double CAPACITY_BCY = 20.0 / 1.3;

        /// <summary>Grade tolerance used by these fields, in millimetres.</summary>
        private const Double TOLERANCE_MM = 10.0;

        /// <summary>
        /// One to three cells at 50 mm or less are left for the plane.
        /// The loop takes no move and stops at planing.
        /// </summary>
        [Fact]
        public void SmallBumpsAreLeftForThePlane()
        {
            HaulOptimizationResult Result = Run(RidgeGrid());
            Assert.NotNull(Result.Finish);
            FinishingLoop.Report Report = Result.Finish!;
            Assert.Equal(HaulSearchState.Planing, Result.State);
            Assert.Equal(FinishingLoop.PLANING, Report.StopReason);
            Assert.Equal(0.0, Report.FinalHaulErrorBcy, 6);
            Assert.Empty(Result.PassResult.Passes);
            Assert.True(Result.Validation.Success, String.Join(" | ", Result.Validation.Issues));
        }

        /// <summary>
        /// A hole with a nearer high and a farther high. The move that fills the
        /// hole takes the nearer high. The field ends inside the tolerance.
        /// </summary>
        [Fact]
        public void DistantHoleIsFilledByTheCheapestMoveThatReachesIt()
        {
            HaulOptimizationResult Result = Run(HoleGrid());
            Assert.NotNull(Result.Finish);
            Assert.NotNull(Result.CorrectingSurfaceM);
            FinishingLoop.Report Report = Result.Finish!;
            Double[,] Surface = Result.CorrectingSurfaceM!;
            Assert.Equal(HaulSearchState.Planing, Result.State);
            Assert.Equal(0.0, Report.FinalHaulErrorBcy, 6);
            Assert.True(Result.Validation.Success, String.Join(" | ", Result.Validation.Issues));
            Assert.True(Math.Abs(Surface[4, 16]) < 1e-4);
            Pass? Fill = null;
            for (Int32 Index = 0; Index < Result.PassResult.Passes.Count; Index += 1)
            {
                Pass Pass = Result.PassResult.Passes[Index];
                for (Int32 Cell = 0; Cell < Pass.FillCells.Count; Cell += 1)
                {
                    if ((Pass.FillCells[Cell].Cell.By == 4) && (Pass.FillCells[Cell].Cell.Bx == 16))
                    {
                        Fill = Pass;
                    }
                }
            }
            Assert.NotNull(Fill);
            Boolean Near = false;
            Boolean Far = false;
            for (Int32 Cell = 0; Cell < Fill.CutCells.Count; Cell += 1)
            {
                if ((Fill.CutCells[Cell].Cell.By == 4) && (Fill.CutCells[Cell].Cell.Bx == 6))
                {
                    Near = true;
                }
                if ((Fill.CutCells[Cell].Cell.By == 1) && (Fill.CutCells[Cell].Cell.Bx == 1))
                {
                    Far = true;
                }
            }
            Assert.True(Near);
            Assert.False(Far);
        }

        /// <summary>
        /// No highs, and the hole is deeper than the cap can supply. The hole is
        /// partly filled and the stop reason is the deficit cap.
        /// </summary>
        [Fact]
        public void DeficitSourceFillsWhatTheCapAllowsAndReportsTheRest()
        {
            HaulOptimizationResult Result = Run(DeficitGrid());
            Assert.NotNull(Result.Finish);
            Assert.NotNull(Result.CorrectingSurfaceM);
            FinishingLoop.Report Report = Result.Finish!;
            Double[,] Surface = Result.CorrectingSurfaceM!;
            Assert.Equal(HaulSearchState.Planing, Result.State);
            Assert.Equal(0.0, Report.FinalHaulErrorBcy, 6);
            Assert.True(Report.FinalErrorBcy > 0.0);
            Assert.True(Result.Validation.Success, String.Join(" | ", Result.Validation.Issues));
            Double Hole = Surface[1, 1];
            Assert.True(Hole < -0.010);
            Assert.True(Hole > -0.061 + 0.005);
            Double CapM = (TOLERANCE_MM / 3.0) / 1000.0;
            Boolean Lowered = false;
            for (Int32 Row = 0; Row < 4; Row += 1)
            {
                for (Int32 Col = 0; Col < 4; Col += 1)
                {
                    if ((Row == 1) && (Col == 1))
                    {
                        continue;
                    }
                    Double Residual = Surface[Row, Col];
                    Assert.True(Residual <= 1e-6);
                    Assert.True(Residual >= -CapM - 1e-6);
                    if (Residual < -1e-6)
                    {
                        Lowered = true;
                    }
                }
            }
            Assert.True(Lowered);
        }

        /// <summary>
        /// The same field twice keeps the same moves, in the same order.
        /// </summary>
        [Fact]
        public void SameInputProducesTheSameMoveSequence()
        {
            HaulOptimizationResult First = Run(HoleGrid());
            HaulOptimizationResult Second = Run(HoleGrid());
            Assert.Equal(First.PassResult.Passes.Count, Second.PassResult.Passes.Count);
            for (Int32 Index = 0; Index < First.PassResult.Passes.Count; Index += 1)
            {
                Pass Left = First.PassResult.Passes[Index];
                Pass Right = Second.PassResult.Passes[Index];
                Assert.Equal(Left.Stage, Right.Stage);
                Assert.Equal(Left.LoadBcy, Right.LoadBcy, 9);
                Assert.Equal(Left.CutCells.Count, Right.CutCells.Count);
                Assert.Equal(Left.CutCells[0].Cell, Right.CutCells[0].Cell);
                Assert.Equal(Left.FillCells[0].Cell, Right.FillCells[0].Cell);
            }
        }

        /// <summary>
        /// The lazy queue keeps the same moves as pricing every candidate again.
        /// </summary>
        [Fact]
        public void LazyQueueMatchesTheEagerLoop()
        {
            Compare(HoleGrid(), HoleGrid());
            Compare(LongStrip(), LongStrip());
        }

        /// <summary>Runs the lazy loop and the eager loop and checks the move lists.</summary>
        /// <param name="LazyGrid">Field for the lazy loop.</param>
        /// <param name="EagerGrid">A second copy of the same field.</param>
        private static void Compare
            (
            Grid LazyGrid,
            Grid EagerGrid
            )
        {
            HaulOptimizationResult Lazy = Run(LazyGrid);
            Environment.SetEnvironmentVariable("FINISHING_LOOP_EAGER", "1");
            HaulOptimizationResult Eager;
            try
            {
                Eager = Run(EagerGrid);
            }
            finally
            {
                Environment.SetEnvironmentVariable("FINISHING_LOOP_EAGER", null);
            }
            Assert.Equal(Eager.Finish.FinalErrorBcy, Lazy.Finish.FinalErrorBcy, 6);
            Assert.Equal(Eager.Finish.FinalHaulErrorBcy, Lazy.Finish.FinalHaulErrorBcy, 6);
            Assert.Equal(Eager.PassResult.Passes.Count, Lazy.PassResult.Passes.Count);
            for (Int32 Index = 0; Index < Eager.PassResult.Passes.Count; Index += 1)
            {
                Pass Left = Eager.PassResult.Passes[Index];
                Pass Right = Lazy.PassResult.Passes[Index];
                Assert.Equal(Left.Stage, Right.Stage);
                Assert.Equal(Left.LoadBcy, Right.LoadBcy, 9);
                Assert.Equal(Left.CutCells.Count, Right.CutCells.Count);
                Assert.Equal(Left.CutCells[0].Cell, Right.CutCells[0].Cell);
                Assert.Equal(Left.FillCells[0].Cell, Right.FillCells[0].Cell);
            }
        }

        /// <summary>A strip longer than the plane, deep enough to haul.</summary>
        /// <returns>The grid.</returns>
        private static Grid LongStrip()
        {
            Grid Grid = MakeGrid(6, 16);
            for (Int32 Col = 1; Col <= 8; Col += 1)
            {
                Grid.Existing[2, Col] = 0.061;
            }
            for (Int32 Col = 10; Col <= 14; Col += 1)
            {
                Grid.Existing[2, Col] = -0.050;
            }
            return Grid;
        }

        /// <summary>A ridge of three cells and no holes.</summary>
        /// <returns>The grid.</returns>
        private static Grid RidgeGrid()
        {
            Grid Grid = MakeGrid(8, 10);
            Grid.Existing[3, 2] = 0.040;
            Grid.Existing[3, 3] = 0.040;
            Grid.Existing[3, 4] = 0.040;
            return Grid;
        }

        /// <summary>A hole, a nearer high on its row, and a farther high on another row.</summary>
        /// <returns>The grid.</returns>
        private static Grid HoleGrid()
        {
            Grid Grid = MakeGrid(6, 18);
            Grid.Existing[1, 1] = 0.050;
            Grid.Existing[4, 6] = 0.070;
            Grid.Existing[4, 16] = -0.040;
            return Grid;
        }

        /// <summary>One deep hole and no highs. The cap cannot supply the hole.</summary>
        /// <returns>The grid.</returns>
        private static Grid DeficitGrid()
        {
            Grid Grid = MakeGrid(4, 4);
            Grid.Existing[1, 1] = -0.061;
            return Grid;
        }

        /// <summary>Runs the finishing loop. The window is on, so the ribbon path stays closed.</summary>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The search result.</returns>
        private static HaulOptimizationResult Run
            (
            Grid Grid
            )
        {
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, TOLERANCE_MM);
            return FlowRibbonSearch.Optimize(
                Grid,
                EastFlow(Grid),
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                CAPACITY_BCY,
                2,
                true,
                0.2,
                null,
                null,
                false,
                2.5908,
                45.0,
                1,
                0.5);
        }

        /// <summary>Builds an empty valid grid.</summary>
        /// <param name="Height">Rows.</param>
        /// <param name="Width">Columns.</param>
        /// <returns>The grid.</returns>
        private static Grid MakeGrid
            (
            Int32 Height,
            Int32 Width
            )
        {
            Double[,] Existing = new Double[Height, Width];
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                2.5908,
                Width,
                Height,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                Existing,
                new Double[Height, Width],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>A flow that points east on every cell.</summary>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The flow.</returns>
        private static FlowField EastFlow
            (
            Grid Grid
            )
        {
            Double[,] Phi = new Double[Grid.Height, Grid.Width];
            Double[,] Fx = new Double[Grid.Height, Grid.Width];
            Double[,] Fy = new Double[Grid.Height, Grid.Width];
            Double[,] Magnitude = new Double[Grid.Height, Grid.Width];
            Boolean[,] HasArrow = new Boolean[Grid.Height, Grid.Width];
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Fx[By, Bx] = 1.0;
                    Magnitude[By, Bx] = 1.0;
                    HasArrow[By, Bx] = true;
                }
            }
            return new FlowField(Phi, Fx, Fy, Magnitude, HasArrow, "test");
        }
    }
}
