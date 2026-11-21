namespace HaulBuilder.Tests
{
    /// <summary>
    /// Checks that a cluster trip hauls the near dirt and leaves toward the fill.
    /// </summary>
    public sealed class ResidualHaulTests
    {
        /// <summary>
        /// A partial load takes the cells nearest the fill, so the haul is the short gap.
        /// </summary>
        [Fact]
        public void PartialLoadTakesTheCellsNearestTheFill()
        {
            Grid Grid = Sheet(20, 8);
            Double[,] Residual = new Double[8, 20];
            (Int32 Row, Int32 Col)[] CutCells = new (Int32 Row, Int32 Col)[6];
            for (Int32 Col = 0; Col < 6; Col += 1)
            {
                Residual[3, 4 + Col] = 200.0;
                CutCells[Col] = (3, 4 + Col);
            }
            Residual[3, 14] = -200.0;
            Residual[3, 15] = -200.0;
            Double CellBcy = 0.15 * 1.0 * ResidualClusters.M3_TO_CY;
            ResidualPairPiece[] Pieces = new[]
            {
                new ResidualPairPiece("C1", "cut", CellBcy * 6.0, 150.0, 3.0, 6.5, 6.0, CutCells),
                new ResidualPairPiece(
                    "F1",
                    "fill",
                    CellBcy * 2.0,
                    150.0,
                    3.0,
                    14.5,
                    2.0,
                    new[] { (3, 14), (3, 15) }),
            };
            ResidualPairTrip[] Trips = new[]
            {
                new ResidualPairTrip(
                    1,
                    new[] { new ResidualPairLeg("C1", CellBcy * 2.0) },
                    new[] { new ResidualPairLeg("F1", CellBcy * 2.0) },
                    CellBcy * 2.0,
                    8.0,
                    1.0),
            };
            ResidualHaul Haul = Assert.Single(ResidualHauls.Build(
                Residual, Grid.Valid, Pieces, Trips, Grid, 1.0));
            Assert.Equal(2, Haul.CutCells.Count);
            Assert.Contains(Haul.CutCells, Cell => (Cell.Cell.Bx == 8) && (Cell.Cell.By == 3));
            Assert.Contains(Haul.CutCells, Cell => (Cell.Cell.Bx == 9) && (Cell.Cell.By == 3));
            Assert.Equal(2, Haul.FillCells.Count);
            Assert.True(Haul.HaulM < 8.0, "haul " + Haul.HaulM.ToString("F2"));
            Assert.True(Haul.CutRuns > 0);
            Assert.True(Haul.FillRuns > 0);
            Double MaxDepthM = Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            Assert.All(Haul.CutCells, Cell => Assert.True(Cell.DepthM <= MaxDepthM + 1e-9));
        }

        /// <summary>
        /// A cell deeper than one pass is cut in lifts, and no lift exceeds the maximum depth.
        /// </summary>
        [Fact]
        public void CutDepthStaysWithinThePassLimit()
        {
            Grid Grid = Sheet(4, 4, Constants.DEFAULT_MAX_DEPTH_MM);
            Double[,] Residual = new Double[4, 4];
            Residual[1, 1] = 200.0;
            Residual[1, 3] = -200.0;
            Double DepthM = (200.0 - Constants.DEFAULT_TOLERANCE_MM) / 1000.0;
            Double CellBcy = DepthM * 1.0 * ResidualClusters.M3_TO_CY;
            ResidualPairPiece[] Pieces = new[]
            {
                new ResidualPairPiece("C1", "cut", CellBcy, 150.0, 1.0, 1.0, 1.0, new[] { (1, 1) }),
                new ResidualPairPiece("F1", "fill", CellBcy, 150.0, 1.0, 3.0, 1.0, new[] { (1, 3) }),
            };
            ResidualPairTrip[] Trips = new[]
            {
                new ResidualPairTrip(
                    1,
                    new[] { new ResidualPairLeg("C1", CellBcy) },
                    new[] { new ResidualPairLeg("F1", CellBcy) },
                    CellBcy,
                    2.0,
                    1.0),
            };
            ResidualHaul Haul = Assert.Single(ResidualHauls.Build(
                Residual, Grid.Valid, Pieces, Trips, Grid, 1.0));
            Double MaxDepthM = Constants.DEFAULT_MAX_DEPTH_MM / Constants.MILLIMETERS_PER_METER;
            Assert.True(Haul.CutCells.Count > 1);
            Double CutDepthM = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double LiftM) Cut in Haul.CutCells)
            {
                Assert.Equal(1, Cut.Cell.Bx);
                Assert.Equal(1, Cut.Cell.By);
                Assert.True(Cut.LiftM <= MaxDepthM + 1e-9, "cut " + Cut.LiftM.ToString("F4"));
                CutDepthM += Cut.LiftM;
            }
            Assert.Equal(DepthM, CutDepthM, 6);
            Assert.Equal(CellBcy, Haul.LoadedBcy, 6);
            Assert.True(Haul.LoadedBcy <= CellBcy + 1e-6);
        }

        /// <summary>
        /// A cell is cut and filled until it is inside the grade tolerance.
        /// Depth between the 50 mm cluster floor and that tolerance is taken too.
        /// A cell already inside the tolerance is left alone.
        /// </summary>
        [Fact]
        public void HaulStopsAtTheGradeTolerance()
        {
            Grid Grid = Sheet(4, 4, Constants.DEFAULT_MAX_DEPTH_MM);
            Double[,] Residual = new Double[4, 4];
            Residual[1, 1] = 80.0;
            Residual[2, 1] = 40.0;
            Residual[2, 2] = 8.0;
            Residual[1, 3] = -80.0;
            Double ToleranceMm = Constants.DEFAULT_TOLERANCE_MM;
            (Int32 Row, Int32 Col)[] CutCells = new[] { (1, 1), (2, 1), (2, 2) };
            (Int32 Row, Int32 Col)[] FillCells = new[] { (1, 3) };
            Double CutBcy = ResidualHauls.LevelVolumeBcy(
                CutCells, "cut", Residual, Grid, 1.0, ToleranceMm);
            Double FillBcy = ResidualHauls.LevelVolumeBcy(
                FillCells, "fill", Residual, Grid, 1.0, ToleranceMm);
            ResidualPairPiece[] Pieces = new[]
            {
                new ResidualPairPiece("C1", "cut", CutBcy, 70.0, 1.5, 1.0, 2.0, CutCells),
                new ResidualPairPiece("F1", "fill", FillBcy, 70.0, 1.0, 3.0, 1.0, FillCells),
            };
            ResidualPairTrip[] Trips = new[]
            {
                new ResidualPairTrip(
                    1,
                    new[] { new ResidualPairLeg("C1", CutBcy) },
                    new[] { new ResidualPairLeg("F1", FillBcy) },
                    CutBcy,
                    2.0,
                    1.0),
            };
            ResidualHaul Haul = Assert.Single(ResidualHauls.Build(
                Residual, Grid.Valid, Pieces, Trips, Grid, 1.0, ToleranceMm));
            Double MaxDepthM = Constants.DEFAULT_MAX_DEPTH_MM / Constants.MILLIMETERS_PER_METER;
            Double DeepCutM = 0.0;
            Double ShallowCutM = 0.0;
            Double FillM = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double LiftM) Cut in Haul.CutCells)
            {
                Assert.True(Cut.LiftM <= MaxDepthM + 1e-9);
                Assert.False((Cut.Cell.Bx == 2) && (Cut.Cell.By == 2));
                if ((Cut.Cell.Bx == 1) && (Cut.Cell.By == 1))
                {
                    DeepCutM += Cut.LiftM;
                }
                if ((Cut.Cell.Bx == 1) && (Cut.Cell.By == 2))
                {
                    ShallowCutM += Cut.LiftM;
                }
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double LiftM) Fill in Haul.FillCells)
            {
                Assert.True(Fill.LiftM <= MaxDepthM + 1e-9);
                FillM += Fill.LiftM;
            }
            Assert.Equal((80.0 - ToleranceMm) / 1000.0, DeepCutM, 6);
            Assert.Equal((40.0 - ToleranceMm) / 1000.0, ShallowCutM, 6);
            Assert.Equal((80.0 - ToleranceMm) / 1000.0, FillM, 6);
            Assert.True(DeepCutM > ((80.0 - ResidualClusters.FLOOR_MM) / 1000.0));
        }

        /// <summary>
        /// A small all-valid sheet. Elevations are flat because the residual is passed separately.
        /// </summary>
        /// <param name="Width">Columns.</param>
        /// <param name="Height">Rows.</param>
        /// <param name="MaxDepthMm">Deepest one pass may cut or spread, in millimetres.</param>
        /// <returns>The grid.</returns>
        private static Grid Sheet
            (
            Int32 Width,
            Int32 Height,
            Double MaxDepthMm = 300.0
            )
        {
            Double[,] Elevation = new Double[Height, Width];
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 Row = 0; Row < Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Width; Col += 1)
                {
                    Valid[Row, Col] = true;
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
                Elevation,
                Elevation,
                Valid,
                1.3,
                1.0,
                MaxDepthMm,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
