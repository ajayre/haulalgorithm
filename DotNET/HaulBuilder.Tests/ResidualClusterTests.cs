using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Checks residual clustering: the deep seed, the floor, the one-cell bridge,
    /// the half-bowl cutoff, and the two-bowl piece split.
    /// </summary>
    public sealed class ResidualClusterTests
    {
        /// <summary>Cell edge used by the synthetic grids, in metres.</summary>
        private const Double CELL_M = 2.5;

        /// <summary>Pan capacity used by the synthetic grids, in bank cubic yards.</summary>
        private const Double BOWL_BCY = 15.38;

        /// <summary>Fill volume factor used by the fill mirror.</summary>
        private const Double FILL_TO_BANK = 1.2;

        /// <summary>
        /// One interior 5 by 5 block at 200 mm is one cut cluster.
        /// The haul volume is 25 cells times 0.150 m times 6.25 m2.
        /// </summary>
        [Fact]
        public void OneBlockIsOneCutCluster()
        {
            Double[,] Residual = Zeros(9, 9);
            Boolean[,] Valid = AllValid(9, 9);
            Fill(Residual, 2, 6, 2, 6, 200);
            ResidualClusterReport Report = ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
            ResidualCluster Cluster = Assert.Single(Report.Clusters);
            Assert.Equal("C1", Cluster.Id);
            Assert.Equal("cut", Cluster.Sign);
            Assert.Equal(25, Cluster.CellCount);
            Double Metres = 25.0 * 0.150 * 6.25;
            Assert.Equal(23.44, Math.Round(Metres, 2));
            Assert.Equal(Metres, Cluster.HaulVolumeM3, 9);
            Assert.Equal(Metres * ResidualClusters.M3_TO_CY, Cluster.HaulVolumeBcy, 9);
            Assert.Equal(0.0, Report.Diagnostics.CutHaulOutsideBcy, 9);
            Assert.Equal(25, Report.Diagnostics.DeepCutAfterBuffer);
            ResidualCluster Piece = Assert.Single(Report.Pieces);
            Assert.Equal("C1", Piece.Id);
            Assert.Equal(25, Piece.CellCount);
            Assert.Equal(Cluster.HaulVolumeBcy, Piece.HaulVolumeBcy, 9);
        }

        /// <summary>
        /// A one-cell gap is bridged into one cluster. A three-cell gap stays two clusters.
        /// </summary>
        [Fact]
        public void OneCellGapJoinsAndThreeCellGapSplits()
        {
            ResidualClusterReport Joined = TwoBlocks(1);
            ResidualCluster JoinedCluster = Assert.Single(Joined.Clusters);
            Assert.Contains(JoinedCluster.Cells, Cell => (Cell.Row == 2) && (Cell.Col == 2));
            Assert.Contains(JoinedCluster.Cells, Cell => (Cell.Row == 2) && (Cell.Col == 8));
            Double OneBlock = 25.0 * 0.150 * 6.25 * ResidualClusters.M3_TO_CY;
            Assert.Equal(OneBlock * 2.0, JoinedCluster.HaulVolumeBcy, 9);

            ResidualClusterReport Split = TwoBlocks(3);
            Assert.Equal(2, Split.Clusters.Count);
            Assert.Equal("C1", Split.Clusters[0].Id);
            Assert.Equal("C2", Split.Clusters[1].Id);
            Assert.Equal(25, Split.Clusters[0].CellCount);
            Assert.Equal(25, Split.Clusters[1].CellCount);
            Assert.True(Split.Clusters[0].MinCol < Split.Clusters[1].MinCol);
        }

        /// <summary>
        /// A deep core pulls in the shallower ring that is still above the floor.
        /// </summary>
        [Fact]
        public void RingAboveTheFloorJoinsTheCore()
        {
            Double[,] Residual = Zeros(11, 11);
            Boolean[,] Valid = AllValid(11, 11);
            Fill(Residual, 1, 9, 1, 9, 80);
            Fill(Residual, 3, 7, 3, 7, 200);
            ResidualClusterReport Report = ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
            ResidualCluster Cluster = Assert.Single(Report.Clusters);
            Assert.Equal(81, Cluster.CellCount);
        }

        /// <summary>
        /// A plateau below the seed threshold is not a cluster.
        /// </summary>
        [Fact]
        public void PlateauWithoutADeepCellIsDropped()
        {
            Double[,] Residual = Zeros(20, 20);
            Boolean[,] Valid = AllValid(20, 20);
            Fill(Residual, 0, 19, 0, 19, 80);
            ResidualClusterReport Report = ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
            Assert.Empty(Report.Clusters);
            Assert.Equal(0, Report.Diagnostics.DeepCutBeforeBuffer);
        }

        /// <summary>
        /// Speckle no deeper than 60 mm has no seed, so it forms no cluster.
        /// </summary>
        [Fact]
        public void SpeckleBelowTheSeedIsDropped()
        {
            Double[,] Residual = Zeros(30, 30);
            Boolean[,] Valid = AllValid(30, 30);
            Random Random = new Random(42);
            for (Int32 Row = 0; Row < 30; Row += 1)
            {
                for (Int32 Col = 0; Col < 30; Col += 1)
                {
                    Residual[Row, Col] = Random.NextDouble() * 60.0;
                }
            }
            ResidualClusterReport Report = ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
            Assert.Empty(Report.Clusters);
            Assert.Equal(0, Report.Diagnostics.DeepCutBeforeBuffer);
        }

        /// <summary>
        /// A block that fills the field keeps the outer ring. Those cells are clustered.
        /// </summary>
        [Fact]
        public void EdgeBlockKeepsItsOuterRing()
        {
            Double[,] Residual = Zeros(5, 5);
            Boolean[,] Valid = AllValid(5, 5);
            Fill(Residual, 0, 4, 0, 4, 200);
            ResidualClusterReport Report = ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
            ResidualCluster Cluster = Assert.Single(Report.Clusters);
            Assert.Equal(25, Cluster.CellCount);
            Assert.Equal(25, Report.Diagnostics.DeepCutBeforeBuffer);
            Assert.Equal(25, Report.Diagnostics.DeepCutAfterBuffer);
            Double Metres = 25.0 * 0.150 * 6.25;
            Assert.Equal(Metres, Cluster.HaulVolumeM3, 9);
        }

        /// <summary>
        /// The same block below target is one fill cluster, and the yards include the fill factor.
        /// </summary>
        [Fact]
        public void FillBlockScalesByTheBankFactor()
        {
            Double[,] Residual = Zeros(9, 9);
            Boolean[,] Valid = AllValid(9, 9);
            Fill(Residual, 2, 6, 2, 6, -200);
            ResidualClusterReport Report = ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
            ResidualCluster Cluster = Assert.Single(Report.Clusters);
            Assert.Equal("F1", Cluster.Id);
            Assert.Equal("fill", Cluster.Sign);
            Assert.Equal(25, Cluster.CellCount);
            Double Metres = 25.0 * 0.150 * 6.25;
            Assert.Equal(Metres, Cluster.HaulVolumeM3, 9);
            Assert.Equal(Metres * ResidualClusters.M3_TO_CY * FILL_TO_BANK, Cluster.HaulVolumeBcy, 9);
            Assert.Equal(25, Report.Diagnostics.DeepFillAfterBuffer);
            Assert.True(Report.Pieces.Count > 1);
            AssertPiecesPartition(Cluster, Report.Pieces);
        }

        /// <summary>
        /// A deep block under half a bowl is discarded, and its haul volume is reported outside the clusters.
        /// </summary>
        [Fact]
        public void BlockUnderHalfABowlIsLeftOut()
        {
            Double[,] Residual = Zeros(4, 4);
            Boolean[,] Valid = AllValid(4, 4);
            Fill(Residual, 1, 2, 1, 2, 200);
            ResidualClusterReport Report = ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
            Assert.Empty(Report.Clusters);
            Double Metres = 4.0 * 0.150 * 6.25;
            Double Yards = Metres * ResidualClusters.M3_TO_CY;
            Assert.True(Yards < (0.5 * BOWL_BCY));
            Assert.Equal(0.0, Report.Diagnostics.CutHaulBcy, 9);
            Assert.Equal(Yards, Report.Diagnostics.CutHaulOutsideBcy, 9);
        }

        /// <summary>
        /// A 4 by 40 strip at 200 mm is 150 m3, about 196 BCY. It stays one cluster
        /// and splits into contiguous pieces of at most two bowls, ordered from
        /// the end that holds the first cell.
        /// </summary>
        [Fact]
        public void StripOverTwoBowlsSplitsIntoContiguousPieces()
        {
            // The strip sits one cell in from the grid edge. All 160 cells are in the cluster.
            Double[,] Residual = Zeros(6, 42);
            Boolean[,] Valid = AllValid(6, 42);
            Fill(Residual, 1, 4, 1, 40, 200);
            ResidualClusterReport Report = ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
            ResidualCluster Cluster = Assert.Single(Report.Clusters);
            Assert.Equal("C1", Cluster.Id);
            Assert.Equal(160, Cluster.CellCount);
            Double Metres = 160.0 * 0.150 * 6.25;
            Assert.Equal(150.0, Metres);
            Assert.Equal(Metres * ResidualClusters.M3_TO_CY, Cluster.HaulVolumeBcy, 9);
            Assert.True(Report.Pieces.Count > 1);
            Double Limit = ResidualClusters.MAX_PIECE_BOWLS * BOWL_BCY;
            Assert.Equal(30.76, Limit, 2);
            AssertPiecesPartition(Cluster, Report.Pieces);
            Assert.Contains(Report.Pieces[0].Cells, Cell => (Cell.Row == 1) && (Cell.Col == 1));
            Assert.Contains(Report.Pieces[Report.Pieces.Count - 1].Cells, Cell => (Cell.Row == 1) && (Cell.Col == 40));
            for (Int32 Index = 0; Index < Report.Pieces.Count; Index += 1)
            {
                ResidualCluster Piece = Report.Pieces[Index];
                String Expected = "C1." + (Index + 1).ToString(CultureInfo.InvariantCulture);
                Assert.Equal(Expected, Piece.Id);
                Assert.True(Piece.HaulVolumeBcy <= Limit);
                AssertContiguous(Piece);
                if (Index > 0)
                {
                    Assert.True(Piece.CentroidCol > Report.Pieces[Index - 1].CentroidCol);
                }
            }
        }

        /// <summary>
        /// Draws the pieces on the Shop 2 field at route 2888, the ground
        /// left when hauling hands off to clusters.
        /// </summary>
        [Fact]
        public void Shop2Route2888ClustersAreDrawn()
        {
            String AgdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = AgdPath,
                SkipGuidance = true,
                IncludeGuidanceCsv = false,
            };
            Options.Optimization.EnableCycleChainOptimization = false;
            Options.Optimization.PairRefinementRounds = 0;
            HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);
            Assert.NotNull(Result.PassResult);
            Assert.NotNull(Result.Grid);
            Assert.NotNull(Result.HaulOptimization);
            Assert.Equal(HaulSearchState.Correcting, Result.State);
            Assert.Equal(2888, Result.ClusterModeStartsAtRoute);
            // The picture is the ground when hauling hands off, before cluster trips.
            Int32 FlowPasses = Result.ClusterModeStartsAtRoute - 1;
            Assert.Equal(2887, FlowPasses);
            Grid Grid = Result.Grid;
            Double[,] Surface = Grid.Delta();
            for (Int32 Index = 0; Index < FlowPasses; Index += 1)
            {
                Pass Pass = Result.PassResult.Passes[Index];
                foreach (var Cut in Pass.CutCells)
                {
                    Surface[Cut.Cell.By, Cut.Cell.Bx] -= Cut.DepthM;
                }
                foreach (var Fill in Pass.FillCells)
                {
                    Surface[Fill.Cell.By, Fill.Cell.Bx] += Fill.DepthM;
                }
            }
            Double[,] ResidualMm = new Double[Grid.Height, Grid.Width];
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    Double Metres = Surface[Row, Col];
                    ResidualMm[Row, Col] = Grid.Valid[Row, Col] && !Double.IsNaN(Metres) ? Metres * 1000.0 : 0.0;
                }
            }
            Double BowlBcy = 20.0 / 1.3;
            ResidualClusterReport Report = ResidualClusters.Build(
                ResidualMm,
                Grid.Valid,
                Grid.GridSizeM,
                BowlBcy,
                Grid.Compaction);
            Report.Print(Console.Out);
            String Folder = Path.GetFullPath(Path.Combine(
                AppContext.BaseDirectory,
                "..",
                "..",
                "..",
                "..",
                "Fixtures",
                "FlowRibbonProof",
                "shop2-2026-11-20-found"));
            Directory.CreateDirectory(Folder);
            String PngPath = Path.Combine(Folder, "route-2888-clusters.png");
            String JsonPath = Path.Combine(Folder, "route-2888-clusters.json");
            ResidualClusters.SavePng(PngPath, ResidualMm, Grid.Valid, Report.Pieces, 16);
            ResidualClusters.SaveJson(JsonPath, Report);
            Double PieceLimit = ResidualClusters.MAX_PIECE_BOWLS * BowlBcy;
            Assert.All(Report.Pieces, Piece => Assert.True(Piece.HaulVolumeBcy <= PieceLimit));
            Double PieceHaul = 0.0;
            foreach (ResidualCluster Piece in Report.Pieces)
            {
                PieceHaul += Piece.HaulVolumeBcy;
            }
            Assert.Equal(
                Report.Diagnostics.CutHaulBcy + Report.Diagnostics.FillHaulBcy,
                PieceHaul,
                4);
            // A flat residual compresses well, so the check is the sheet size, not a byte floor.
            Byte[] Header = File.ReadAllBytes(PngPath);
            Assert.True(Header.Length > 10000);
            Assert.Equal((UInt32)(Grid.Width * 16), ReadPngDimension(Header, 16));
            Assert.Equal((UInt32)(Grid.Height * 16), ReadPngDimension(Header, 20));
            Assert.True(Report.Clusters.Count > 0);
        }

        /// <summary>
        /// The pieces partition the parent, stay within two bowls, and their haul sums to the parent.
        /// </summary>
        /// <param name="Cluster">Parent cluster.</param>
        /// <param name="Pieces">Pieces cut from that parent. The caller passes only that parent's pieces.</param>
        private static void AssertPiecesPartition
            (
            ResidualCluster Cluster,
            IReadOnlyList<ResidualCluster> Pieces
            )
        {
            Double Limit = ResidualClusters.MAX_PIECE_BOWLS * BOWL_BCY;
            HashSet<(Int32 Row, Int32 Col)> Left = new HashSet<(Int32 Row, Int32 Col)>(Cluster.Cells);
            Double Sum = 0.0;
            foreach (ResidualCluster Piece in Pieces)
            {
                Assert.True(Piece.HaulVolumeBcy <= Limit);
                Sum += Piece.HaulVolumeBcy;
                foreach ((Int32 Row, Int32 Col) Cell in Piece.Cells)
                {
                    Assert.True(Left.Remove(Cell));
                }
            }
            Assert.Empty(Left);
            Assert.Equal(Cluster.HaulVolumeBcy, Sum, 6);
        }

        /// <summary>
        /// Every cell of the piece touches another through an edge. A split strip must not scatter.
        /// </summary>
        /// <param name="Piece">Piece to walk.</param>
        private static void AssertContiguous
            (
            ResidualCluster Piece
            )
        {
            HashSet<(Int32 Row, Int32 Col)> Left = new HashSet<(Int32 Row, Int32 Col)>(Piece.Cells);
            Queue<(Int32 Row, Int32 Col)> Queue = new Queue<(Int32 Row, Int32 Col)>();
            (Int32 Row, Int32 Col) Start = Piece.Cells[0];
            Assert.True(Left.Remove(Start));
            Queue.Enqueue(Start);
            Int32[] DeltaRow = { -1, 0, 1, 0 };
            Int32[] DeltaCol = { 0, -1, 0, 1 };
            while (Queue.Count > 0)
            {
                (Int32 Row, Int32 Col) Cell = Queue.Dequeue();
                for (Int32 Neighbour = 0; Neighbour < 4; Neighbour += 1)
                {
                    (Int32 Row, Int32 Col) Next = (Cell.Row + DeltaRow[Neighbour], Cell.Col + DeltaCol[Neighbour]);
                    if (Left.Remove(Next))
                    {
                        Queue.Enqueue(Next);
                    }
                }
            }
            Assert.Empty(Left);
        }

        /// <summary>
        /// Two 5 by 5 blocks of 200 mm on one row, with a gap of the given width.
        /// </summary>
        /// <param name="Gap">Empty columns between the blocks.</param>
        /// <returns>The cluster report.</returns>
        private static ResidualClusterReport TwoBlocks
            (
            Int32 Gap
            )
        {
            Int32 Cols = 2 + 5 + Gap + 5 + 2;
            Double[,] Residual = Zeros(9, Cols);
            Boolean[,] Valid = AllValid(9, Cols);
            Fill(Residual, 2, 6, 2, 6, 200);
            Int32 Second = 2 + 5 + Gap;
            Fill(Residual, 2, 6, Second, Second + 4, 200);
            return ResidualClusters.Build(Residual, Valid, CELL_M, BOWL_BCY, FILL_TO_BANK);
        }

        /// <summary>A zero grid.</summary>
        /// <param name="Rows">Rows.</param>
        /// <param name="Cols">Columns.</param>
        /// <returns>The grid.</returns>
        private static Double[,] Zeros
            (
            Int32 Rows,
            Int32 Cols
            )
        {
            return new Double[Rows, Cols];
        }

        /// <summary>A mask that is true on every cell.</summary>
        /// <param name="Rows">Rows.</param>
        /// <param name="Cols">Columns.</param>
        /// <returns>The mask.</returns>
        private static Boolean[,] AllValid
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
            return Valid;
        }

        /// <summary>Reads one big-endian PNG dimension from the IHDR chunk.</summary>
        /// <param name="Png">PNG bytes, IHDR included.</param>
        /// <param name="Offset">16 for width, 20 for height.</param>
        /// <returns>The dimension in pixels.</returns>
        private static UInt32 ReadPngDimension
            (
            Byte[] Png,
            Int32 Offset
            )
        {
            return ((UInt32)Png[Offset] << 24)
                | ((UInt32)Png[Offset + 1] << 16)
                | ((UInt32)Png[Offset + 2] << 8)
                | Png[Offset + 3];
        }

        /// <summary>Paints an inclusive rectangle.</summary>
        /// <param name="Residual">Grid to paint.</param>
        /// <param name="Row0">First row.</param>
        /// <param name="Row1">Last row.</param>
        /// <param name="Col0">First column.</param>
        /// <param name="Col1">Last column.</param>
        /// <param name="Millimetres">Residual in millimetres.</param>
        private static void Fill
            (
            Double[,] Residual,
            Int32 Row0,
            Int32 Row1,
            Int32 Col0,
            Int32 Col1,
            Double Millimetres
            )
        {
            for (Int32 Row = Row0; Row <= Row1; Row += 1)
            {
                for (Int32 Col = Col0; Col <= Col1; Col += 1)
                {
                    Residual[Row, Col] = Millimetres;
                }
            }
        }
    }
}
