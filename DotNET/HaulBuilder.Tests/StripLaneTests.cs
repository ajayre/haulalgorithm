using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Area trips for cluster and thin-cut. Dirt moves in blade-wide bands parallel to the heading.
    /// The fullest band is taken first. A centreline is drawn for each band that moved.
    /// </summary>
    public sealed class StripLaneTests
    {
        /// <summary>Shop cell edge in metres. One cell is one blade.</summary>
        private const Double CELL_M = 2.5908;

        /// <summary>Bowl used by the production scraper, in bank cubic yards.</summary>
        private const Double BOWL_BCY = 20.0 / 1.3;

        /// <summary>
        /// A 30 m patch at 61 mm is four equal bands. One full bowl fills three of them,
        /// so the piece takes three passes and two loops, and the payload is the whole bowl.
        /// </summary>
        [Fact]
        public void LongPatchAtOneLiftIsThreePasses()
        {
            const Int32 Rows = 8;
            const Int32 Cols = 40;
            Grid Grid = Sheet(Rows, Cols, 300.0, 1.0);
            Double[,] Residual = new Double[Rows, Cols];
            List<(Int32 Row, Int32 Col)> CutCells = new List<(Int32 Row, Int32 Col)>();
            List<(Int32 Row, Int32 Col)> FillCells = new List<(Int32 Row, Int32 Col)>();
            for (Int32 Row = 2; Row <= 5; Row += 1)
            {
                for (Int32 Col = 2; Col <= 13; Col += 1)
                {
                    Residual[Row, Col] = StripLanes.CUT_DEPTH_CAP_MM;
                    CutCells.Add((Row, Col));
                }
                for (Int32 Col = 22; Col <= 29; Col += 1)
                {
                    // Deep enough that one 100 mm spread can take a share of the bowl.
                    Residual[Row, Col] = -200.0;
                    FillCells.Add((Row, Col));
                }
            }
            StripLanes.Ground Field = FieldOf(Grid, Residual, 1.0);
            // 12 cells at 2.5908 m is 31.09 m. A full bowl at 61 mm is about 75 m of blade.
            StripLanes.Result Cut = Assert.IsType<StripLanes.Result>(StripLanes.Cut(
                Field, CutCells, 1.0, 0.0, BOWL_BCY));
            StripLanes.Result Spread = Assert.IsType<StripLanes.Result>(StripLanes.Fill(
                Field, FillCells, 1.0, 0.0, Cut.Bcy));
            StripLanes.Match(Cut, Spread, Field);
            Assert.Equal(3, Cut.PassCount);
            Assert.Equal(2, Cut.Loops);
            Assert.Equal(3, Cut.Lines.Count);
            Assert.Equal(BOWL_BCY, Cut.Bcy, 3);
            Assert.Equal(BandSeconds(Cut), Cut.Seconds, 6);
            Assert.All(Cut.Cells, Lift => Assert.True(Lift.DepthM <= (StripLanes.CUT_DEPTH_CAP_MM / 1000.0) + 1e-9));
            Assert.All(Cut.Cells, Lift => Assert.True(Covered(Cut.Lines, Lift.Cell.Bx, Lift.Cell.By, Grid)));
        }

        /// <summary>
        /// The arm of an L runs across the heading, so each arm cell is its own band.
        /// Those cells are taller than the stem, and they stay until their band is opened.
        /// Every cell that loses volume lies under a drawn centreline.
        /// </summary>
        [Fact]
        public void ArmIsCutOnlyWhenItsBandIsChosen()
        {
            const Int32 Rows = 8;
            const Int32 Cols = 12;
            Grid Grid = Sheet(Rows, Cols, 300.0, 1.0);
            Double[,] Residual = new Double[Rows, Cols];
            List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>();
            for (Int32 Col = 0; Col < 8; Col += 1)
            {
                Residual[2, Col] = 40.0;
                Cells.Add((2, Col));
            }
            // The arm is one cell wide and three bands tall. Each cell is a deeper cut than the stem.
            (Int32 Row, Int32 Col)[] Arm = new (Int32 Row, Int32 Col)[]
            {
                (3, 0),
                (4, 0),
                (5, 0),
            };
            foreach ((Int32 Row, Int32 Col) Cell in Arm)
            {
                Residual[Cell.Row, Cell.Col] = StripLanes.CUT_DEPTH_CAP_MM;
                Cells.Add(Cell);
            }
            StripLanes.Ground Field = FieldOf(Grid, Residual, 1.0);
            Double Area = CELL_M * CELL_M;
            Double StemBcy = 8.0 * 0.040 * Area * ResidualClusters.M3_TO_CY;
            Double ArmBcy = 0.061 * Area * ResidualClusters.M3_TO_CY;
            StripLanes.Result Short = Assert.IsType<StripLanes.Result>(StripLanes.Cut(
                Field, Cells, 1.0, 0.0, StemBcy * 0.5));
            Assert.Equal(1, Short.PassCount);
            Assert.Single(Short.Lines);
            Assert.Equal(BandSeconds(Short), Short.Seconds, 6);
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Short.Cells)
            {
                Assert.Equal(2, Lift.Cell.By);
                Assert.True(Covered(Short.Lines, Lift.Cell.Bx, Lift.Cell.By, Grid));
            }
            foreach ((Int32 Row, Int32 Col) Cell in Arm)
            {
                Assert.DoesNotContain(Short.Cells, Lift => (Lift.Cell.Bx == Cell.Col) && (Lift.Cell.By == Cell.Row));
                Assert.False(Covered(Short.Lines, Cell.Col, Cell.Row, Grid));
            }
            // The stem band is full, so the next fullest band is the first arm cell.
            StripLanes.Result Wider = Assert.IsType<StripLanes.Result>(StripLanes.Cut(
                Field, Cells, 1.0, 0.0, StemBcy + (ArmBcy * 0.5)));
            Assert.Equal(2, Wider.PassCount);
            Assert.Equal(2, Wider.Lines.Count);
            Assert.Contains(Wider.Cells, Lift => (Lift.Cell.Bx == 0) && (Lift.Cell.By == 3));
            Assert.DoesNotContain(Wider.Cells, Lift => (Lift.Cell.Bx == 0) && (Lift.Cell.By == 4));
            Assert.DoesNotContain(Wider.Cells, Lift => (Lift.Cell.Bx == 0) && (Lift.Cell.By == 5));
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Wider.Cells)
            {
                Assert.True(Covered(Wider.Lines, Lift.Cell.Bx, Lift.Cell.By, Grid));
            }
            Assert.False(Covered(Wider.Lines, 0, 4, Grid));
            Assert.False(Covered(Wider.Lines, 0, 5, Grid));
        }

        /// <summary>
        /// A fill cell rises by at most one spread, and never above the target.
        /// The grid allows 300 mm, so the 100 mm spread is the cap that binds.
        /// </summary>
        [Fact]
        public void FillStopsAtTheSpreadLiftAndTheTarget()
        {
            const Int32 Rows = 6;
            const Int32 Cols = 8;
            Grid Grid = Sheet(Rows, Cols, 300.0, 1.0);
            Double[,] Residual = new Double[Rows, Cols];
            Residual[2, 2] = -250.0;
            Residual[2, 3] = -40.0;
            Residual[2, 4] = 0.0;
            List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>
            {
                (2, 2),
                (2, 3),
                (2, 4),
            };
            StripLanes.Ground Field = FieldOf(Grid, Residual, 1.0);
            StripLanes.Result Spread = Assert.IsType<StripLanes.Result>(StripLanes.Fill(
                Field, Cells, 1.0, 0.0, BOWL_BCY));
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Spread.Cells)
            {
                Assert.True(Lift.DepthM <= StripLanes.SPREAD_LIFT_M + 1e-9);
                Assert.True(Lift.DepthM <= (-Residual[Lift.Cell.By, Lift.Cell.Bx] / 1000.0) + 1e-9);
            }
            StripLanes.Apply(Field, Spread.Cells, -1);
            Assert.Equal(-150.0, Field.ResidualMm[2, 2], 3);
            Assert.Equal(0.0, Field.ResidualMm[2, 3], 3);
            Assert.Equal(0.0, Field.ResidualMm[2, 4], 3);
            Assert.True(Field.ResidualMm[2, 2] <= 1e-6);
            Assert.True(Field.ResidualMm[2, 3] <= 1e-6);
            Assert.True(Field.ResidualMm[2, 4] <= 1e-6);
        }

        /// <summary>
        /// After compaction, the bank yards cut equal the bank yards placed, within a tenth of a yard.
        /// The fill is smaller than the cut, so the cut is trimmed to the fill.
        /// </summary>
        [Fact]
        public void CutVolumeMatchesThePlacement()
        {
            const Int32 Rows = 4;
            const Int32 Cols = 16;
            const Double Compaction = 1.2;
            Grid Grid = Sheet(Rows, Cols, StripLanes.CUT_DEPTH_CAP_MM, Compaction);
            Double[,] Residual = new Double[Rows, Cols];
            List<(Int32 Row, Int32 Col)> CutCells = new List<(Int32 Row, Int32 Col)>();
            List<(Int32 Row, Int32 Col)> FillCells = new List<(Int32 Row, Int32 Col)>();
            for (Int32 Col = 1; Col <= 8; Col += 1)
            {
                Residual[1, Col] = StripLanes.CUT_DEPTH_CAP_MM;
                CutCells.Add((1, Col));
            }
            for (Int32 Col = 11; Col <= 13; Col += 1)
            {
                Residual[1, Col] = -StripLanes.CUT_DEPTH_CAP_MM;
                FillCells.Add((1, Col));
            }
            StripLanes.Ground Field = FieldOf(Grid, Residual, Compaction);
            StripLanes.Result Cut = Assert.IsType<StripLanes.Result>(StripLanes.Cut(
                Field, CutCells, 1.0, 0.0, BOWL_BCY));
            StripLanes.Result Spread = Assert.IsType<StripLanes.Result>(StripLanes.Fill(
                Field, FillCells, 1.0, 0.0, Cut.Bcy));
            StripLanes.Match(Cut, Spread, Field);
            Double Area = CELL_M * CELL_M;
            Double CutBank = 0.0;
            Double FillBank = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Cut.Cells)
            {
                CutBank += Lift.DepthM * Area * ResidualClusters.M3_TO_CY;
                Assert.True(Lift.DepthM <= (StripLanes.CUT_DEPTH_CAP_MM / 1000.0) + 1e-9);
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Spread.Cells)
            {
                FillBank += Lift.DepthM * Area * ResidualClusters.M3_TO_CY * Compaction;
                Assert.True(Lift.DepthM <= StripLanes.SPREAD_LIFT_M + 1e-9);
                Assert.True(Lift.DepthM <= (Grid.MaxDepthMm / 1000.0) + 1e-9);
            }
            Assert.True(CutBank > 0.0);
            Assert.True(Math.Abs(CutBank - FillBank) <= 0.1);
            Assert.True(Math.Abs(Cut.Bcy - Spread.Bcy) <= 0.1);
            StripLanes.Apply(Field, Cut.Cells, 1);
            StripLanes.Apply(Field, Spread.Cells, -1);
            for (Int32 Col = 1; Col <= 8; Col += 1)
            {
                Assert.True(Field.ResidualMm[1, Col] >= -1e-4);
            }
            for (Int32 Col = 11; Col <= 13; Col += 1)
            {
                Assert.True(Field.ResidualMm[1, Col] <= 1e-4);
            }
        }

        /// <summary>
        /// Seconds from the drawn band lengths: the sum of the lengths over cut speed, plus a loop between bands.
        /// </summary>
        /// <param name="Side">One side of a trip.</param>
        /// <returns>The predicted seconds.</returns>
        private static Double BandSeconds
            (
            StripLanes.Result Side
            )
        {
            Double Run = 0.0;
            foreach (StripLanes.PassLine Line in Side.Lines)
            {
                Double Dx = Line.X2 - Line.X1;
                Double Dy = Line.Y2 - Line.Y1;
                Run += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            return (Run / StripLanes.V_CUT_MPS) + ((Side.Lines.Count - 1) * StripLanes.T_LOOP_S);
        }

        /// <summary>
        /// True when a centreline passes over the cell. Half a cell of slack covers the band width.
        /// </summary>
        /// <param name="Lines">Drawn band centrelines.</param>
        /// <param name="Col">Column.</param>
        /// <param name="Row">Row.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>True when the cell lies under a line.</returns>
        private static Boolean Covered
            (
            IReadOnlyList<StripLanes.PassLine> Lines,
            Int32 Col,
            Int32 Row,
            Grid Grid
            )
        {
            Double Cx = Grid.MinX + ((Col + 0.5) * Grid.GridSizeM);
            Double Cy = Grid.MinY + ((Row + 0.5) * Grid.GridSizeM);
            Double Half = (CELL_M * 0.5) + 0.05;
            foreach (StripLanes.PassLine Line in Lines)
            {
                Double Dx = Line.X2 - Line.X1;
                Double Dy = Line.Y2 - Line.Y1;
                Double Len2 = (Dx * Dx) + (Dy * Dy);
                if (Len2 < 1e-12)
                {
                    continue;
                }
                Double T = (((Cx - Line.X1) * Dx) + ((Cy - Line.Y1) * Dy)) / Len2;
                if ((T < -0.02) || (T > 1.02))
                {
                    continue;
                }
                Double Px = Line.X1 + (T * Dx);
                Double Py = Line.Y1 + (T * Dy);
                Double Dist2 = ((Cx - Px) * (Cx - Px)) + ((Cy - Py) * (Cy - Py));
                if (Dist2 <= (Half * Half))
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// A valid sheet. The residual is supplied separately, so the elevations stay flat.
        /// </summary>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="MaxDepthMm">Deepest one pass may cut or spread.</param>
        /// <param name="Compaction">Fill bank yards per loose yard.</param>
        /// <returns>The grid.</returns>
        private static Grid Sheet
            (
            Int32 Rows,
            Int32 Cols,
            Double MaxDepthMm,
            Double Compaction
            )
        {
            Boolean[,] Valid = new Boolean[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    Valid[Row, Col] = true;
                }
            }
            return new Grid(
                CELL_M,
                Cols,
                Rows,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[Rows, Cols],
                new Double[Rows, Cols],
                Valid,
                1.3,
                Compaction,
                MaxDepthMm,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>
        /// Ground with a zero grade line, so cut and fill stop on the target.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Residual">Residual in millimetres.</param>
        /// <param name="Compaction">Fill bank factor.</param>
        /// <returns>The ground.</returns>
        private static StripLanes.Ground FieldOf
            (
            Grid Grid,
            Double[,] Residual,
            Double Compaction
            )
        {
            return new StripLanes.Ground(Grid, Residual, Grid.Valid, Compaction, 0.0);
        }
    }
}
