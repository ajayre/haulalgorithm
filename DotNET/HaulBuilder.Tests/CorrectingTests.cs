using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Coverage passes. One line balances a cut and a fill. A full bowl carries
    /// and drops. Surplus mode spreads a ridge at most 3 mm per cell, up to
    /// +10 mm. A deep pocket takes one 100 mm lift per coverage. Cut, fill,
    /// and surplus share one bank total.
    /// </summary>
    public sealed class CorrectingTests
    {
        /// <summary>Cell edge used by the line checks, in metres.</summary>
        private const Double CELL_M = 2.5;

        /// <summary>Pan width. The pass spacing is this minus the 0.3 m overlap.</summary>
        private const Double PAN_M = 2.5;

        /// <summary>
        /// Ten cells at +40 mm and ten at -40 mm, one line, equal bank.
        /// Every cell finishes inside +/-5 mm.
        /// </summary>
        [Fact]
        public void OneLineBalancesHighsAndLows()
        {
            const Int32 Count = 20;
            Double[,] Residual = new Double[1, Count];
            for (Int32 Col = 0; Col < 10; Col += 1)
            {
                Residual[0, Col] = 40.0;
            }
            for (Int32 Col = 10; Col < Count; Col += 1)
            {
                Residual[0, Col] = -40.0;
            }
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(1, Count),
                East(1, Count),
                North(1, Count),
                Arrows(1, Count),
                CELL_M,
                PAN_M,
                BowlBcy(100.0),
                1.0);
            Assert.Equal(1, Result.CoveragesRun);
            for (Int32 Col = 0; Col < Count; Col += 1)
            {
                Assert.InRange(Residual[0, Col], -5.0, 5.0);
            }
        }

        /// <summary>
        /// Sixty cells at +40 mm are more than one bowl, then sixty at -40 mm.
        /// The bowl fills, the rest of the high cells are carried past, and the
        /// load is dropped on the lows. No high cell is cut through the target.
        /// The cells left high are in the group report. Correcting stops there.
        /// </summary>
        [Fact]
        public void FullBowlCarriesAndDrops()
        {
            const Int32 Highs = 60;
            const Int32 Lows = 60;
            const Int32 Count = Highs + Lows;
            Double[,] Residual = new Double[1, Count];
            for (Int32 Col = 0; Col < Highs; Col += 1)
            {
                Residual[0, Col] = 40.0;
            }
            for (Int32 Col = Highs; Col < Count; Col += 1)
            {
                Residual[0, Col] = -40.0;
            }
            // One coverage, so a second pass cannot hide the cells the bowl skipped.
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(1, Count),
                East(1, Count),
                North(1, Count),
                Arrows(1, Count),
                CELL_M,
                PAN_M,
                BowlBcy(10.0),
                1.0,
                null,
                1);
            Int32 StillHigh = 0;
            Int32 Raised = 0;
            for (Int32 Col = 0; Col < Count; Col += 1)
            {
                if (Col < Highs)
                {
                    Assert.True(Residual[0, Col] >= -0.5);
                    if (Residual[0, Col] > 30.0)
                    {
                        StillHigh += 1;
                    }
                }
                else
                {
                    Assert.True(Residual[0, Col] >= -40.5);
                    if (Residual[0, Col] > -5.0)
                    {
                        Raised += 1;
                    }
                }
            }
            Assert.True(StillHigh > 0);
            Assert.True(Raised > 0);
            Assert.False(Result.ReadyForPlaning);
            Assert.NotEmpty(Result.FailedGroups);
            Assert.Contains(Result.Coverages[0].Groups, Group => Group.PeakMm >= 35.0);
            Assert.True(Result.Coverages[0].DistanceM > 0.0);
            // One strip the length of the field. There is no turn to make.
            Assert.Equal(0, Result.Coverages[0].Turns);
            Assert.True(Result.Coverages[0].Hours > 0.0);
        }

        /// <summary>
        /// The field is on the target except a 10-cell ridge at +150 mm.
        /// Surplus mode cuts the ridge to the target and spreads that soil
        /// at most 3 mm per cell, so nothing finishes above +10 mm.
        /// The cut, the spread, and any load left in the bowl are the same volume.
        /// </summary>
        [Fact]
        public void SurplusModeCutsTheRidgeAndSpreadsThinly()
        {
            const Int32 Ridge = 10;
            // 3 mm on one cell is 0.01875 m3. The ridge is 9.375 m3, so 500 cells take it.
            const Int32 Grade = 500;
            const Int32 Count = Ridge + Grade;
            Double[,] Residual = new Double[1, Count];
            for (Int32 Col = 0; Col < Ridge; Col += 1)
            {
                Residual[0, Col] = 150.0;
            }
            Double Area = CELL_M * CELL_M;
            Double StartM3 = Ridge * 0.150 * Area;
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(1, Count),
                East(1, Count),
                North(1, Count),
                Arrows(1, Count),
                CELL_M,
                PAN_M,
                BowlBcy(100.0),
                1.0);
            Double EndM3 = 0.0;
            for (Int32 Col = 0; Col < Count; Col += 1)
            {
                Assert.InRange(Residual[0, Col], -0.5, Correcting.SURPLUS_MAX_MM + 0.5);
                if (Col < Ridge)
                {
                    Assert.InRange(Residual[0, Col], -0.5, 0.5);
                }
                else
                {
                    Assert.InRange(Residual[0, Col], 0.0, Correcting.SURPLUS_STEP_MM + 0.5);
                }
                EndM3 += (Residual[0, Col] / 1000.0) * Area;
            }
            Assert.Equal(StartM3, EndM3 + Result.BowlLeftM3, 4);
            Assert.Equal(
                Result.CutRemovedM3,
                Result.FillPlacedM3 + Result.SurplusSpreadM3 + Result.BowlLeftM3,
                4);
        }

        /// <summary>
        /// Grade cells between a cut and a fill stay put while the fill need
        /// is still at least a quarter bowl. The load is kept for the lows.
        /// </summary>
        [Fact]
        public void GradeStaysPutWhileFillRemains()
        {
            const Int32 Count = 44;
            Double[,] Residual = new Double[1, Count];
            for (Int32 Col = 0; Col < 4; Col += 1)
            {
                Residual[0, Col] = 40.0;
                Residual[0, 40 + Col] = -40.0;
            }
            // Four low cells are 1.0 m3. A 3 m3 bowl keeps that above a quarter bowl.
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(1, Count),
                East(1, Count),
                North(1, Count),
                Arrows(1, Count),
                CELL_M,
                PAN_M,
                BowlBcy(3.0),
                1.0,
                null,
                1);
            for (Int32 Col = 4; Col < 40; Col += 1)
            {
                Assert.InRange(Residual[0, Col], -0.5, 0.5);
            }
            for (Int32 Col = 40; Col < Count; Col += 1)
            {
                Assert.InRange(Residual[0, Col], -0.5, 0.5);
            }
            Assert.Equal(0.0, Result.SurplusSpreadM3, 3);
            Assert.Equal(0.0, Result.BowlLeftM3, 3);
        }

        /// <summary>
        /// Three cells at -250 mm take one 100 mm lift on the first coverage
        /// and finish at -150 mm. The second coverage lifts them again.
        /// </summary>
        [Fact]
        public void DeepPocketTakesOneLiftPerCoverage()
        {
            const Int32 Highs = 12;
            const Int32 Pocket = 3;
            const Int32 Count = Highs + Pocket;
            Double[,] Residual = new Double[1, Count];
            for (Int32 Col = 0; Col < Highs; Col += 1)
            {
                Residual[0, Col] = 40.0;
            }
            for (Int32 Col = Highs; Col < Count; Col += 1)
            {
                Residual[0, Col] = -250.0;
            }
            // The bowl holds the first eight cells. The pocket is on the same line.
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(1, Count),
                East(1, Count),
                North(1, Count),
                Arrows(1, Count),
                CELL_M,
                PAN_M,
                BowlBcy(2.0),
                1.0);
            Assert.Equal(2, Result.CoveragesRun);
            Double FirstSum = 0.0;
            Double FirstLow = 0.0;
            Double[,] First = Result.Coverages[0].ResidualMm;
            for (Int32 Col = Highs; Col < Count; Col += 1)
            {
                // The drive places one 100 mm lift. The leftover also goes into the pocket.
                Assert.InRange(First[0, Col], -151.0, -120.0);
                FirstSum += First[0, Col];
                if (First[0, Col] < FirstLow)
                {
                    FirstLow = First[0, Col];
                }
            }
            Assert.True(FirstLow <= -149.0);
            Double SecondSum = 0.0;
            for (Int32 Col = Highs; Col < Count; Col += 1)
            {
                Assert.True(Residual[0, Col] <= 0.5);
                SecondSum += Residual[0, Col];
            }
            Assert.True(SecondSum > (FirstSum + 50.0));
        }

        /// <summary>
        /// Cut bank equals fill bank plus surplus bank. The fill uses the
        /// bank factor, so 100 mm of compacted lift is not 100 mm of bank.
        /// </summary>
        [Fact]
        public void CutEqualsFillPlusSurplus()
        {
            const Int32 Highs = 4;
            const Int32 Count = Highs + 1;
            Double[,] Residual = new Double[1, Count];
            for (Int32 Col = 0; Col < Highs; Col += 1)
            {
                Residual[0, Col] = 40.0;
            }
            Residual[0, Highs] = -200.0;
            // One coverage. A 3 m3 bowl keeps the hole above a quarter bowl after the 100 mm lift.
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(1, Count),
                East(1, Count),
                North(1, Count),
                Arrows(1, Count),
                CELL_M,
                PAN_M,
                BowlBcy(3.0),
                1.2,
                null,
                1);
            // The 100 mm lift uses 0.75 m3 at 1.2. The leftover 0.25 m3 goes back into the hole.
            Assert.InRange(Residual[0, Highs], -68.0, -65.0);
            Assert.Equal(1.0, Result.CutRemovedM3, 3);
            Assert.Equal(1.0, Result.FillPlacedM3, 3);
            Assert.Equal(0.0, Result.SurplusSpreadM3, 3);
            Assert.Equal(0.0, Result.BowlLeftM3, 3);
            Assert.Equal(
                Result.CutRemovedM3,
                Result.FillPlacedM3 + Result.SurplusSpreadM3 + Result.BowlLeftM3,
                6);
        }

        /// <summary>
        /// The passes recorded for the ledger are the net cut and the fill placed
        /// on lows. They balance, stay inside the required depth, and the check passes.
        /// </summary>
        [Fact]
        public void RecordedPassesBalanceInTheLedger()
        {
            const Int32 Count = 20;
            Grid Grid = MakeGrid(Count);
            for (Int32 Col = 0; Col < 10; Col += 1)
            {
                Grid.Existing[0, Col] = 0.040;
                Grid.Existing[0, Col + 10] = -0.040;
            }
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);
            Double[,] CutLeft = (Double[,])Ledger.RequiredCutDepthM.Clone();
            Double[,] FillLeft = (Double[,])Ledger.RequiredFillDepthM.Clone();
            List<Pass> Passes = new List<Pass>();
            Correcting.Apply(
                Grid,
                EastFlow(Grid),
                Ledger,
                CutLeft,
                FillLeft,
                BowlBcy(100.0),
                PAN_M,
                Passes);
            Assert.NotEmpty(Passes);
            HaulPlanValidation Validation = Ledger.ValidateComplete(
                Passes,
                BowlBcy(100.0),
                Grid.MaxDepthMm / 1000.0,
                true);
            Assert.True(Validation.Success, String.Join(" | ", Validation.Issues));
        }

        /// <summary>
        /// Bowl capacity in bank cubic yards for a volume given in cubic metres.
        /// </summary>
        /// <param name="CubicMetres">Bank cubic metres.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double BowlBcy
            (
            Double CubicMetres
            )
        {
            return CubicMetres * ResidualClusters.M3_TO_CY;
        }

        /// <summary>Every cell is on the field.</summary>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <returns>The mask.</returns>
        private static Boolean[,] Mask
            (
            Int32 Rows,
            Int32 Cols
            )
        {
            Boolean[,] Cells = new Boolean[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    Cells[Row, Col] = true;
                }
            }
            return Cells;
        }

        /// <summary>Unit east arrows, so a single row is one west-to-east line.</summary>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <returns>The east component.</returns>
        private static Double[,] East
            (
            Int32 Rows,
            Int32 Cols
            )
        {
            Double[,] Cells = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    Cells[Row, Col] = 1.0;
                }
            }
            return Cells;
        }

        /// <summary>Zero north component.</summary>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <returns>The north component.</returns>
        private static Double[,] North
            (
            Int32 Rows,
            Int32 Cols
            )
        {
            return new Double[Rows, Cols];
        }

        /// <summary>One row of valid cells at grade.</summary>
        /// <param name="Cols">Column count.</param>
        /// <returns>The grid.</returns>
        private static Grid MakeGrid
            (
            Int32 Cols
            )
        {
            Double[,] Existing = new Double[1, Cols];
            Boolean[,] Valid = new Boolean[1, Cols];
            for (Int32 Col = 0; Col < Cols; Col += 1)
            {
                Valid[0, Col] = true;
            }
            return new Grid(
                CELL_M,
                Cols,
                1,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                Existing,
                new Double[1, Cols],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>East arrows on every cell.</summary>
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
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    Fx[Row, Col] = 1.0;
                    Magnitude[Row, Col] = 1.0;
                    HasArrow[Row, Col] = true;
                }
            }
            return new FlowField(Phi, Fx, Fy, Magnitude, HasArrow, "test");
        }

        /// <summary>Every cell has an arrow.</summary>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <returns>The arrow mask.</returns>
        private static Boolean[,] Arrows
            (
            Int32 Rows,
            Int32 Cols
            )
        {
            return Mask(Rows, Cols);
        }
    }
}
