using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Thin-cut after cluster hauling. Cut tiles and fill tiles are paired.
    /// A tile under a quarter bowl adds no trips. Shallow holes that never
    /// seed a cluster are still filled when a thin cut is in reach.
    /// </summary>
    public sealed class ThinCutTests
    {
        /// <summary>Shop cell edge in metres.</summary>
        private const Double CELL_M = 2.5908;

        /// <summary>Bowl used by the production scraper, in bank cubic yards.</summary>
        private const Double BOWL_BCY = 20.0 / 1.3;

        /// <summary>
        /// A 16 by 16 sheet at 30 mm cannot seed a cut cluster. Three cells at
        /// 300 mm are a fill cluster, so thin-cut hauls the nearest tiles and
        /// the pocket finishes on the target.
        /// </summary>
        [Fact]
        public void NearestTilesFillThePocket()
        {
            const Int32 Rows = 16;
            const Int32 Cols = 19;
            Grid Grid = MakeGrid(Rows, Cols);
            Double[,] CutLeft = new Double[Rows, Cols];
            Double[,] FillLeft = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < 16; Row += 1)
            {
                for (Int32 Col = 0; Col < 16; Col += 1)
                {
                    CutLeft[Row, Col] = 0.030;
                }
            }
            // The pocket sits on the east edge of the sheet, beside tile column 1.
            FillLeft[0, 16] = 0.300;
            FillLeft[0, 17] = 0.300;
            FillLeft[0, 18] = 0.300;
            List<Pass> Passes = new List<Pass>();
            ThinCutStop Stop = ThinCut.Run(
                Grid,
                HaulWorkLedger.Build(Grid, 10.0),
                CutLeft,
                FillLeft,
                BOWL_BCY,
                Passes);
            Assert.Equal(ThinCut.NO_HAULABLE_WORK, Stop.Reason);
            Assert.True(Stop.TripsDone > 0);
            Assert.True(Stop.MeanPayloadBcy > 0.0);
            Assert.Equal(HaulSearchState.Correcting, ThinCut.Handoff(Stop));
            Assert.NotEmpty(Passes);
            foreach (Pass Trip in Passes)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Trip.CutCells)
                {
                    // Tile T0.1 is rows 0-7 and columns 8-15. That is the tile next to the pocket.
                    Assert.InRange(Lift.Cell.By, 0, 7);
                    Assert.InRange(Lift.Cell.Bx, 8, 15);
                }
            }
            Assert.Equal(0.030, CutLeft[0, 0], 9);
            Assert.Equal(0.0, FillLeft[0, 16], 6);
            Assert.Equal(0.0, FillLeft[0, 17], 6);
            Assert.Equal(0.0, FillLeft[0, 18], 6);
        }

        /// <summary>
        /// The cluster plan is balanced, and the same cells are also tiles.
        /// Thin-cut hauls the tiles until a fresh plan has no trips.
        /// The deep fill does not stay at its starting depth.
        /// </summary>
        [Fact]
        public void TilesRunWhenTheClusterPlanIsBalanced()
        {
            const Int32 Rows = 16;
            const Int32 Cols = 40;
            Grid Grid = MakeGrid(Rows, Cols);
            Double[,] CutLeft = new Double[Rows, Cols];
            Double[,] FillLeft = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < 16; Row += 1)
            {
                for (Int32 Col = 0; Col < 16; Col += 1)
                {
                    CutLeft[Row, Col] = 0.030;
                }
            }
            // Equal bank yards: 6 cut cells, and 5 fill cells with compaction 1.2, both at 200 mm.
            for (Int32 Col = 0; Col < 6; Col += 1)
            {
                CutLeft[0, 24 + Col] = 0.200;
            }
            for (Int32 Col = 0; Col < 5; Col += 1)
            {
                FillLeft[0, 32 + Col] = 0.200;
            }
            List<Pass> Passes = new List<Pass>();
            ThinCutStop Stop = ThinCut.Run(
                Grid,
                HaulWorkLedger.Build(Grid, 10.0),
                CutLeft,
                FillLeft,
                BOWL_BCY,
                Passes);
            Assert.Equal(ThinCut.NO_HAULABLE_WORK, Stop.Reason);
            Assert.True(Stop.TripsDone > 0);
            Assert.NotEmpty(Passes);
            Assert.True(FillLeft[0, 32] < 0.200);
            Assert.Equal(HaulSearchState.Correcting, ThinCut.Handoff(Stop));
        }

        /// <summary>
        /// Two 4 by 6 pockets at −105 mm are not deep enough to seed a cluster.
        /// A thin cut about 80 m away is paired with both, and both finish
        /// inside ±30 mm.
        /// </summary>
        [Fact]
        public void ShallowPocketsTakeTheDistantThinCut()
        {
            const Int32 Rows = 20;
            const Int32 Cols = 60;
            Grid Grid = MakeGrid(Rows, Cols);
            Double[,] CutLeft = new Double[Rows, Cols];
            Double[,] FillLeft = new Double[Rows, Cols];
            // About 80 m from the east edge of the pockets to the first cut cell.
            Int32 CutCol = 5 + (Int32)Math.Round(80.0 / CELL_M);
            for (Int32 Row = 0; Row < 16; Row += 1)
            {
                for (Int32 Col = 0; Col < 16; Col += 1)
                {
                    CutLeft[Row, CutCol + Col] = 0.040;
                }
            }
            for (Int32 Row = 0; Row < 4; Row += 1)
            {
                for (Int32 Col = 0; Col < 6; Col += 1)
                {
                    FillLeft[Row, Col] = 0.105;
                    FillLeft[8 + Row, Col] = 0.105;
                }
            }
            List<Pass> Passes = new List<Pass>();
            ThinCutStop Stop = ThinCut.Run(
                Grid,
                HaulWorkLedger.Build(Grid, 10.0),
                CutLeft,
                FillLeft,
                BOWL_BCY,
                Passes);
            Assert.Equal(ThinCut.NO_HAULABLE_WORK, Stop.Reason);
            Assert.True(Stop.TripsDone > 0);
            Assert.NotEmpty(Passes);
            for (Int32 Row = 0; Row < 4; Row += 1)
            {
                for (Int32 Col = 0; Col < 6; Col += 1)
                {
                    Assert.InRange(FillLeft[Row, Col] * 1000.0, 0.0, 30.0);
                    Assert.InRange(FillLeft[8 + Row, Col] * 1000.0, 0.0, 30.0);
                    Assert.True(FillLeft[Row, Col] < 0.150);
                    Assert.True(FillLeft[8 + Row, Col] < 0.150);
                }
            }
        }

        /// <summary>
        /// Every tile is under a quarter bowl. The fill cluster is unchanged,
        /// no trip is driven, and the reported SHORT fill is the fill that was already there.
        /// </summary>
        [Fact]
        public void ThinTilesLeaveShortUnchanged()
        {
            const Int32 Rows = 8;
            const Int32 Cols = 16;
            Grid Grid = MakeGrid(Rows, Cols);
            Double[,] CutLeft = new Double[Rows, Cols];
            Double[,] FillLeft = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < 8; Col += 1)
                {
                    CutLeft[Row, Col] = 0.015;
                }
            }
            List<(Int32 Row, Int32 Col)> FillCells = new List<(Int32 Row, Int32 Col)>();
            for (Int32 Col = 0; Col < 6; Col += 1)
            {
                FillLeft[0, 8 + Col] = 0.200;
                FillCells.Add((0, 8 + Col));
            }
            Double[,] ResidualMm = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    ResidualMm[Row, Col] = (CutLeft[Row, Col] - FillLeft[Row, Col]) * 1000.0;
                }
            }
            Double ExpectedShort = ResidualHauls.LevelVolumeBcy(
                FillCells,
                "fill",
                ResidualMm,
                Grid,
                Grid.Compaction,
                0.0);
            List<Pass> Passes = new List<Pass>();
            ThinCutStop Stop = ThinCut.Run(
                Grid,
                HaulWorkLedger.Build(Grid, 10.0),
                CutLeft,
                FillLeft,
                BOWL_BCY,
                Passes);
            Assert.Equal(ThinCut.NO_HAULABLE_WORK, Stop.Reason);
            Assert.Equal(0, Stop.TripsDone);
            Assert.Equal(ExpectedShort, Stop.ShortFillBcy, 6);
            Assert.Equal(0.0, Stop.MeanPayloadBcy, 9);
            Assert.Empty(Passes);
            Assert.Equal(0.015, CutLeft[0, 0], 9);
            Assert.Equal(0.200, FillLeft[0, 8], 9);
        }

        /// <summary>
        /// A valid grid at the shop cell size. Depth is supplied by the caller.
        /// </summary>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <returns>The grid.</returns>
        private static Grid MakeGrid
            (
            Int32 Rows,
            Int32 Cols
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
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
