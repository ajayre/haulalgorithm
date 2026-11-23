using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Coverage passes. One line balances a cut and a fill. The headland lap
    /// is a second pass in the same coverage. Surplus sinks onto cells at the
    /// target, up to one third of the tolerance. A deep pocket takes one
    /// 100 mm lift per visit. Cut, fill, and surplus share one bank total.
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
        /// The first strip fills the bowl and drops it on the lows. The headland
        /// lap is the second strip and finishes the rest. No high is cut through
        /// the target. The lap is the one turn.
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
            // One coverage. The headland lap is inside it, so both strips run.
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(1, Count),
                East(1, Count),
                North(1, Count),
                Arrows(1, Count),
                CELL_M,
                PAN_M,
                BowlBcy(10.0),
                1.0);
            for (Int32 Col = 0; Col < Count; Col += 1)
            {
                Assert.InRange(Residual[0, Col], -0.5, 0.5);
            }
            Assert.True(Result.ReadyForPlaning);
            Assert.Empty(Result.FailedGroups);
            // Two strips of 119 steps, plus the return from the far end to the start.
            Assert.InRange(Result.Coverages[0].DistanceM, 850.0, 950.0);
            Assert.Equal(1, Result.Coverages[0].Turns);
            Assert.True(Result.Coverages[0].Hours > 0.0);
        }

        /// <summary>
        /// The field is on the target except a 10-cell ridge at +150 mm.
        /// The ridge is cut to the target. The surplus sinks onto cells at the
        /// target, at most one third of the 10 mm tolerance. The cut, the
        /// spread, and any load left in the bowl are the same volume.
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
            Double CapMm = 10.0 / 3.0;
            Double EndM3 = 0.0;
            for (Int32 Col = 0; Col < Count; Col += 1)
            {
                Assert.InRange(Residual[0, Col], -0.5, CapMm + 0.05);
                if (Col < Ridge)
                {
                    Assert.InRange(Residual[0, Col], -0.5, 0.5);
                }
                else
                {
                    Assert.InRange(Residual[0, Col], 0.0, CapMm + 0.05);
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
                1.0);
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
        /// Three cells at -250 mm. The flow strip places one 100 mm lift.
        /// The headland lap spends the rest of the cut on the same coverage,
        /// so the pocket is about -50, -90, and -127 mm. Coverages continue
        /// while that pocket fails the planing gate and each pass reduces
        /// the haul error by at least one percent.
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
            Assert.True(Result.CoveragesRun > 2);
            Assert.False(String.IsNullOrEmpty(Result.StopReason));
            Double[,] First = Result.Coverages[0].ResidualMm;
            // Flow strip: 100 mm, then the leftover 20 mm on the last cell.
            // Headland: the last 1.0 m3 of cut, so 100 mm and 60 mm. The last
            // cell is still low with an empty bowl, so one cap of source follows.
            Assert.InRange(First[0, Highs], -51.0, -49.0);
            Assert.InRange(First[0, Highs + 1], -91.0, -89.0);
            Assert.InRange(First[0, Highs + 2], -127.0, -126.0);
            Double FirstSum = First[0, Highs] + First[0, Highs + 1] + First[0, Highs + 2];
            Double SecondSum = 0.0;
            for (Int32 Col = Highs; Col < Count; Col += 1)
            {
                // Later coverages keep lifting the pocket.
                Assert.True(Residual[0, Col] > First[0, Col]);
                SecondSum += Residual[0, Col];
            }
            Assert.True(SecondSum > FirstSum);
            Assert.True(
                (Result.StopReason == Correcting.STOP_PLANING)
                || (Result.StopReason == Correcting.STOP_HAUL_ERROR));
        }

        /// <summary>
        /// Cut bank equals fill bank plus surplus bank. The fill uses the
        /// bank factor, so 100 mm of compacted lift is not 100 mm of bank.
        /// Later coverages keep cutting cells at the target until each one
        /// is down at the cap. The hole is still deeper than 50 mm, so the
        /// stop reason is the haul error.
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
            // The opening cut is 1 m3 and the hole needs 1.5 m3 of bank.
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(1, Count),
                East(1, Count),
                North(1, Count),
                Arrows(1, Count),
                CELL_M,
                PAN_M,
                BowlBcy(3.0),
                1.2);
            // The first coverage leaves the hole near -63.9 mm. Later coverages
            // use the other cells at the target, and the hole finishes near -55.6 mm.
            Assert.InRange(Result.Coverages[0].ResidualMm[0, Highs], -64.0, -63.7);
            Assert.InRange(Residual[0, Highs], -56.0, -55.0);
            for (Int32 Col = 0; Col < Highs; Col += 1)
            {
                Assert.InRange(Residual[0, Col], (-10.0 / 3.0) - 0.05, (-10.0 / 3.0) + 0.05);
            }
            Assert.Equal(Correcting.STOP_HAUL_ERROR, Result.StopReason);
            Assert.Equal(0.0, Result.SurplusSpreadM3, 3);
            Assert.Equal(0.0, Result.BowlLeftM3, 3);
            Assert.True(Result.CutRemovedM3 > 1.0);
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
        /// The low is two lines away, so the leftover load leaves the strip,
        /// drops, and returns to the next line. Both legs are in the distance,
        /// and the hour estimate uses that distance.
        /// </summary>
        [Fact]
        public void LeftoverLoadHaulsToADistantLow()
        {
            const Int32 Rows = 3;
            const Int32 Cols = 8;
            Double[,] Residual = new Double[Rows, Cols];
            Residual[0, 0] = 40.0;
            Residual[2, 0] = -40.0;
            CorrectingResult Result = Correcting.Run(
                Residual,
                Mask(Rows, Cols),
                East(Rows, Cols),
                North(Rows, Cols),
                Arrows(Rows, Cols),
                CELL_M,
                PAN_M,
                BowlBcy(10.0),
                1.0);
            Assert.InRange(Residual[0, 0], -0.5, 0.5);
            Assert.InRange(Residual[2, 0], -0.5, 0.5);
            Assert.Equal(0.0, Result.BowlLeftM3, 6);
            Assert.False(Result.UnbalancedSurplus);
            Assert.Equal(Correcting.STOP_PLANING, Result.StopReason);
            // Without the out-and-back the coverage is about 133 m. With both legs it is 167 m.
            Assert.InRange(Result.Coverages[0].DistanceM, 160.0, 175.0);
            Double Hours = ((Result.Coverages[0].DistanceM / Correcting.V_PASS_MS)
                + (Result.Coverages[0].Turns * Correcting.T_TURN_S)) / 3600.0;
            Assert.Equal(Hours, Result.Coverages[0].Hours, 9);
        }

        /// <summary>
        /// Every cell is high, so the bowl cannot find a low. The load that
        /// does not fit under the cap is reported as unbalanced surplus.
        /// </summary>
        [Fact]
        public void LeftoverWithNoLowIsUnbalancedSurplus()
        {
            const Int32 Count = 4;
            Double[,] Residual = new Double[1, Count];
            for (Int32 Col = 0; Col < Count; Col += 1)
            {
                Residual[0, Col] = 150.0;
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
            Assert.True(Result.UnbalancedSurplus);
            Assert.True(Result.BowlLeftM3 > 1.0);
            Assert.Equal(Correcting.STOP_PLANING, Result.StopReason);
            Double CapMm = 10.0 / 3.0;
            for (Int32 Col = 0; Col < Count; Col += 1)
            {
                Assert.InRange(Residual[0, Col], 0.0, CapMm + 0.05);
            }
            Assert.Equal(
                Result.CutRemovedM3,
                Result.FillPlacedM3 + Result.SurplusSpreadM3 + Result.BowlLeftM3,
                4);
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
