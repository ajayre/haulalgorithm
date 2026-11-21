namespace HaulBuilder.Tests
{
    /// <summary>
    /// Checks cut-to-fill pairing: the non-crossing assignment, the balance nodes,
    /// the deeper cut, the long-haul-first schedule, and a repeatable trip file.
    /// </summary>
    public sealed class ResidualPairingTests
    {
        /// <summary>Pan capacity used by the synthetic pairs, in bank cubic yards.</summary>
        private const Double BOWL_BCY = 15.38;

        /// <summary>Cell edge. One metre makes the centroid indexes the distances.</summary>
        private const Double CELL_M = 1.0;

        /// <summary>Pan width used by the synthetic pairs, in metres.</summary>
        private const Double PAN_M = 2.5;

        /// <summary>
        /// Equal pieces pair with the near fill. The crossing hauls are longer, so they are not used.
        /// </summary>
        [Fact]
        public void EqualPiecesDoNotCross()
        {
            ResidualPairPlan Plan = ResidualPairing.Plan(Square(), CELL_M, BOWL_BCY, PAN_M);
            Assert.Equal("balanced", Plan.BalanceKind);
            ResidualPairFlow[] Real = RealFlows(Plan);
            Assert.Equal(2, Real.Length);
            Assert.Contains(Real, Flow => (Flow.CutPiece == "A") && (Flow.FillPiece == "C"));
            Assert.Contains(Real, Flow => (Flow.CutPiece == "B") && (Flow.FillPiece == "D"));
            Assert.DoesNotContain(Real, Flow => (Flow.CutPiece == "A") && (Flow.FillPiece == "D"));
            Assert.DoesNotContain(Real, Flow => (Flow.CutPiece == "B") && (Flow.FillPiece == "C"));
            Assert.All(Plan.Trips, Trip =>
            {
                String Cut = Assert.Single(Trip.Cuts).Id;
                String Fill = Assert.Single(Trip.Fills).Id;
                Boolean Near = ((Cut == "A") && (Fill == "C"))
                    || ((Cut == "B") && (Fill == "D"));
                Assert.True(Near);
            });
        }

        /// <summary>
        /// Cut left over after the fill is full goes to LOCAL and is not scheduled.
        /// </summary>
        [Fact]
        public void ExtraCutIsLeftLocalAndNotScheduled()
        {
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("Cut", "cut", 30.0, 150, 0, 0),
                Piece("Fill", "fill", 20.0, 150, 100, 0),
            };
            ResidualPairPlan Plan = ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M);
            Assert.Equal(ResidualPairing.LOCAL_ID, Plan.BalanceKind);
            Assert.Equal(10.0, Plan.BalanceBcy, 6);
            Assert.Contains(Plan.Flows, Flow => (Flow.FillPiece == ResidualPairing.LOCAL_ID) && (Math.Abs(Flow.VolumeBcy - 10.0) < 0.11));
            Assert.DoesNotContain(Plan.Trips, Trip => Trip.Fills.Any(Fill => Fill.Id == ResidualPairing.LOCAL_ID));
            Assert.InRange(Payload(Plan), 19.9, 20.11);
        }

        /// <summary>
        /// Fill that the cuts cannot cover is charged to SHORT and is not scheduled.
        /// </summary>
        [Fact]
        public void ExtraFillIsShortAndNotScheduled()
        {
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("Cut", "cut", 20.0, 150, 0, 0),
                Piece("Fill", "fill", 30.0, 150, 100, 0),
            };
            ResidualPairPlan Plan = ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M);
            Assert.Equal(ResidualPairing.SHORT_ID, Plan.BalanceKind);
            Assert.Equal(10.0, Plan.BalanceBcy, 6);
            Assert.Contains(Plan.Flows, Flow => (Flow.CutPiece == ResidualPairing.SHORT_ID) && (Math.Abs(Flow.VolumeBcy - 10.0) < 0.11));
            Assert.DoesNotContain(Plan.Trips, Trip => Trip.Cuts.Any(Cut => Cut.Id == ResidualPairing.SHORT_ID));
            Assert.InRange(Payload(Plan), 19.9, 20.11);
        }

        /// <summary>
        /// Two cuts at one place compete for one fill. The deeper cut loads faster, so it is chosen.
        /// </summary>
        [Fact]
        public void DeeperCutIsChosenWhenOnlyOneFits()
        {
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("Deep", "cut", BOWL_BCY, 300, 0, 0),
                Piece("Shallow", "cut", BOWL_BCY, 60, 0, 0),
                Piece("F", "fill", BOWL_BCY, 150, 50, 0),
            };
            ResidualPairPlan Plan = ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M);
            ResidualPairFlow Chosen = Assert.Single(Plan.Flows, Flow => Flow.FillPiece == "F");
            Assert.Equal("Deep", Chosen.CutPiece);
            Assert.Contains(Plan.Flows, Flow => (Flow.CutPiece == "Shallow") && (Flow.FillPiece == ResidualPairing.LOCAL_ID));
            Assert.All(Plan.Trips, Trip => Assert.Equal("Deep", Assert.Single(Trip.Cuts).Id));
        }

        /// <summary>
        /// The longer haul is scheduled before the shorter one.
        /// </summary>
        [Fact]
        public void LongerHaulIsScheduledFirst()
        {
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("FarCut", "cut", BOWL_BCY, 150, 0, 0),
                Piece("FarFill", "fill", BOWL_BCY, 150, 0, 400),
                Piece("NearCut", "cut", BOWL_BCY, 150, 1000, 0),
                Piece("NearFill", "fill", BOWL_BCY, 150, 1000, 100),
            };
            ResidualPairPlan Plan = ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M);
            Assert.True(Plan.Trips.Count >= 2);
            Boolean SeenNear = false;
            foreach (ResidualPairTrip Trip in Plan.Trips)
            {
                if (Trip.DistanceM < 150.0)
                {
                    SeenNear = true;
                }
                else if (SeenNear)
                {
                    Assert.Fail("A long haul was scheduled after a short one.");
                }
            }
            Assert.True(SeenNear);
            Assert.InRange(Plan.Trips[0].DistanceM, 399.0, 401.0);
        }

        /// <summary>
        /// Yards split across two fill pieces of one cluster ride on one trip, nearest piece first.
        /// </summary>
        [Fact]
        public void OneCutSpreadsAcrossOneFillClusterInOneTrip()
        {
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("C1.1", "cut", BOWL_BCY, 150, 0, 0),
                Piece("F1.3", "fill", 10.0, 150, 10, 0),
                Piece("F1.4", "fill", 5.38, 150, 40, 0),
            };
            ResidualPairPlan Plan = ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M);
            ResidualPairTrip Trip = Assert.Single(Plan.Trips);
            Assert.Equal("C1.1", Assert.Single(Trip.Cuts).Id);
            Assert.Equal(2, Trip.Fills.Count);
            Assert.Equal("F1.3", Trip.Fills[0].Id);
            Assert.Equal(10.0, Trip.Fills[0].Bcy, 2);
            Assert.Equal("F1.4", Trip.Fills[1].Id);
            Assert.InRange(Trip.Fills[1].Bcy, 5.3, 5.45);
            Assert.InRange(Trip.PayloadBcy, BOWL_BCY - 0.05, BOWL_BCY + 0.05);
        }

        /// <summary>
        /// Two remainders of one cut cluster that share an edge, both aimed at F1, pack into one trip.
        /// </summary>
        [Fact]
        public void AdjacentRemaindersShareATrip()
        {
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("C1.1", "cut", 7.0, 150, 0, 0, (0, 0)),
                Piece("C1.2", "cut", 7.0, 150, 0, 10, (0, 1)),
                Piece("F1.1", "fill", 14.0, 150, 50, 5),
            };
            ResidualPairPlan Plan = ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M);
            ResidualPairTrip Trip = Assert.Single(Plan.Trips);
            Assert.Equal(14.0, Trip.PayloadBcy, 2);
            Assert.Equal(2, Trip.Cuts.Count);
            Assert.Equal("C1.1", Trip.Cuts[0].Id);
            Assert.Equal(7.0, Trip.Cuts[0].Bcy, 2);
            Assert.Equal("C1.2", Trip.Cuts[1].Id);
            Assert.Equal(7.0, Trip.Cuts[1].Bcy, 2);
            Assert.Equal("F1.1", Assert.Single(Trip.Fills).Id);
            Assert.Equal(14.0, Trip.Fills[0].Bcy, 2);
            Assert.Empty(Plan.Remainders);
        }

        /// <summary>
        /// A 14.5 BCY partial takes what it needs from an edge-adjacent 1.33 BCY remainder.
        /// The flow quantum keeps 1.3 of that 1.33, so the bowl takes 0.88 and 0.42 is reported.
        /// </summary>
        [Fact]
        public void PartialTripTopsUpFromAnAdjacentRemainder()
        {
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("C2.10", "cut", 14.5, 150, 0, 0, (5, 5)),
                Piece("C2.11", "cut", 1.33, 150, 0, 1, (5, 6)),
                Piece("F1.1", "fill", 15.83, 150, 0, 80),
            };
            ResidualPairPlan Plan = ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M);
            ResidualPairTrip Trip = Assert.Single(Plan.Trips);
            Assert.Equal(BOWL_BCY, Trip.PayloadBcy, 2);
            Assert.Equal(2, Trip.Cuts.Count);
            Assert.Equal("C2.10", Trip.Cuts[0].Id);
            Assert.Equal(14.5, Trip.Cuts[0].Bcy, 2);
            Assert.Equal("C2.11", Trip.Cuts[1].Id);
            Assert.InRange(Trip.Cuts[1].Bcy, 0.85, 0.90);
            ResidualPairRemainder Left = Assert.Single(Plan.Remainders);
            Assert.Equal("C2.11", Left.CutPiece);
            Assert.InRange(Left.VolumeBcy, 0.40, 0.46);
        }

        /// <summary>
        /// A remainder under a quarter bowl, with no neighbour to join, is reported and not scheduled.
        /// </summary>
        [Fact]
        public void LoneShortRemainderGoesToCorrecting()
        {
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("C3", "cut", 2.0, 150, 0, 0),
                Piece("F9", "fill", 2.0, 150, 30, 0),
            };
            ResidualPairPlan Plan = ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M);
            Assert.Empty(Plan.Trips);
            ResidualPairRemainder Left = Assert.Single(Plan.Remainders);
            Assert.Equal(2.0, Left.VolumeBcy, 2);
            Assert.Equal("C3", Left.CutPiece);
            Assert.Equal("F9", Left.FillPiece);
        }

        /// <summary>
        /// The same pieces produce the same trip file.
        /// </summary>
        [Fact]
        public void SamePiecesProduceTheSameTripFile()
        {
            ResidualPairPiece[] Pieces = Square();
            String First = ResidualPairing.TripsJson(ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M).Trips);
            String Second = ResidualPairing.TripsJson(ResidualPairing.Plan(Pieces, CELL_M, BOWL_BCY, PAN_M).Trips);
            Assert.Equal(First, Second);
        }

        /// <summary>
        /// Runs Shop 2 through correcting and writes the field after those trips.
        /// The route count includes the cluster trips, the thin-cut trips, and the coverage passes.
        /// The engine prints the handoff reports.
        /// </summary>
        [Fact]
        public void Shop2Route2888PairsAreDrawn()
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
            Assert.Equal(HaulSearchState.Correcting, Result.State);
            Assert.Equal(2888, Result.ClusterModeStartsAtRoute);
            Assert.True(Result.CorrectingStartsAtRoute > Result.ClusterModeStartsAtRoute);
            Assert.True(Result.PassResult.Passes.Count >= 2887);
            Grid Grid = Result.Grid;
            // The coverage residual. Surplus spread on a grade cell is not recorded as a haul.
            Double[,]? Found = Result.HaulOptimization?.CorrectingSurfaceM;
            if (Found == null)
            {
                throw new InvalidOperationException("Correcting did not record a surface.");
            }
            Double[,] Surface = Found;
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
            String FieldPath = Path.Combine(Folder, "route-2888-after-surplus.png");
            ResidualHauls.SaveFieldPng(FieldPath, Surface, Grid.Valid, 16);
            String SurfacePath = Path.Combine(Path.GetTempPath(), "route-surplus-surface.bin");
            using (BinaryWriter Writer = new BinaryWriter(File.Create(SurfacePath)))
            {
                Writer.Write(Grid.Height);
                Writer.Write(Grid.Width);
                Writer.Write(Grid.GridSizeM);
                for (Int32 Row = 0; Row < Grid.Height; Row += 1)
                {
                    for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                    {
                        Writer.Write(Grid.Valid[Row, Col] ? Surface[Row, Col] : Double.NaN);
                    }
                }
            }
            String LoadsPath = Path.Combine(Path.GetTempPath(), "route-surplus-loads.txt");
            using (StreamWriter Loads = new StreamWriter(LoadsPath))
            {
                foreach (Pass Pass in Result.PassResult.Passes)
                {
                    Loads.WriteLine(Pass.LoadBcy.ToString("R", System.Globalization.CultureInfo.InvariantCulture));
                }
            }
            Double AreaM2 = Grid.AreaM2;
            Double CutLeft = 0.0;
            Double FillLeft = 0.0;
            Double HighestM = Double.NegativeInfinity;
            Double LowestM = Double.PositiveInfinity;
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    if (!Grid.Valid[Row, Col] || Double.IsNaN(Surface[Row, Col]))
                    {
                        continue;
                    }
                    Double Metres = Surface[Row, Col];
                    HighestM = Math.Max(HighestM, Metres);
                    LowestM = Math.Min(LowestM, Metres);
                    if (Metres > 0.0)
                    {
                        CutLeft += Metres * AreaM2 * ResidualClusters.M3_TO_CY;
                    }
                    else if (Metres < 0.0)
                    {
                        FillLeft += -Metres * AreaM2 * ResidualClusters.M3_TO_CY * Grid.Compaction;
                    }
                }
            }
            Console.WriteLine("routes: {0}", Result.PassResult.Passes.Count);
            Console.WriteLine("cluster starts at route: {0}", Result.ClusterModeStartsAtRoute);
            Console.WriteLine("thin-cut starts at route: {0}", Result.ThinCutStartsAtRoute);
            Console.WriteLine("state: {0}", Result.State);
            Console.WriteLine("correcting starts at route: {0}", Result.CorrectingStartsAtRoute);
            TimeParams Time = new TimeParams(
                Options.CutSpeedMph,
                Options.LoadedTravelSpeedMph,
                Options.FillSpeedMph,
                Options.UnloadedTravelSpeedMph,
                Options.AccelerationTimeS,
                Options.DecelerationTimeS,
                Options.FieldEfficiency);
            Double FlowS = 0.0;
            Double ClusterS = 0.0;
            Double ThinS = 0.0;
            Double CorrectS = 0.0;
            IReadOnlyList<Pass> Timed = Result.PassResult.Passes;
            for (Int32 Index = 0; Index < Timed.Count; Index += 1)
            {
                // Found order matches the stage boundaries. OrderedPasses does not.
                Double Seconds = TimeModel.EstimatePassTime(Grid, Timed[Index], Time);
                Int32 Route = Index + 1;
                if (Route < Result.ClusterModeStartsAtRoute)
                {
                    FlowS += Seconds;
                }
                else if (Route < Result.ThinCutStartsAtRoute)
                {
                    ClusterS += Seconds;
                }
                else if (Route < Result.CorrectingStartsAtRoute)
                {
                    ThinS += Seconds;
                }
                else
                {
                    CorrectS += Seconds;
                }
            }
            Console.WriteLine("flow hours: {0:R}", FlowS / 3600.0);
            Console.WriteLine("cluster hours: {0:R}", ClusterS / 3600.0);
            Console.WriteLine("thin-cut hours: {0:R}", ThinS / 3600.0);
            Console.WriteLine("correcting pass hours: {0:R}", CorrectS / 3600.0);
            Console.WriteLine(
                "after correcting: high {0:F1} mm low {1:F1} mm cut {2:F1} bcy fill {3:F1} bcy",
                HighestM * 1000.0,
                LowestM * 1000.0,
                CutLeft,
                FillLeft);
            Byte[] FieldHeader = File.ReadAllBytes(FieldPath);
            Assert.True(FieldHeader.Length > 10000);
            Assert.Equal((UInt32)(Grid.Width * 16), ReadPngDimension(FieldHeader, 16));
            Assert.Equal((UInt32)(Grid.Height * 16), ReadPngDimension(FieldHeader, 20));
        }

        /// <summary>
        /// The trip sheet draws one arrow per trip, including a shared load from two cut pieces.
        /// </summary>
        [Fact]
        public void TripMapWritesOneArrowSheet()
        {
            Double[,] Residual = new Double[6, 8];
            Boolean[,] Valid = new Boolean[6, 8];
            for (Int32 Row = 0; Row < 6; Row += 1)
            {
                for (Int32 Col = 0; Col < 8; Col += 1)
                {
                    Valid[Row, Col] = true;
                }
            }
            ResidualPairPiece[] Pieces = new[]
            {
                Piece("C1.1", "cut", 7.0, 150, 1, 1),
                Piece("C1.2", "cut", 7.0, 150, 1, 3),
                Piece("F1.1", "fill", 14.0, 150, 4, 2),
                Piece("A", "cut", BOWL_BCY, 150, 2, 6),
                Piece("C", "fill", BOWL_BCY, 150, 5, 6),
            };
            ResidualPairTrip[] Trips = new[]
            {
                new ResidualPairTrip(
                    1,
                    new[] { new ResidualPairLeg("C1.1", 7.0), new ResidualPairLeg("C1.2", 7.0) },
                    new[] { new ResidualPairLeg("F1.1", 14.0) },
                    14.0,
                    10.0,
                    1.0),
                new ResidualPairTrip(
                    2,
                    new[] { new ResidualPairLeg("A", BOWL_BCY) },
                    new[] { new ResidualPairLeg("C", BOWL_BCY) },
                    BOWL_BCY,
                    20.0,
                    1.0),
            };
            String PngPath = Path.Combine(Path.GetTempPath(), "haul-trip-map.png");
            ResidualPairing.SaveTripPng(PngPath, Residual, Valid, Trips, Pieces, BOWL_BCY, 16);
            Byte[] Header = File.ReadAllBytes(PngPath);
            Assert.True(Header.Length > 100);
            Assert.Equal((UInt32)(8 * 16), ReadPngDimension(Header, 16));
            Assert.Equal((UInt32)(6 * 16), ReadPngDimension(Header, 20));
        }

        /// <summary>
        /// The four corners of the non-crossing test. Distances are in metres because the cell is 1 m.
        /// </summary>
        /// <returns>Two cuts and two fills, one bowl each.</returns>
        private static ResidualPairPiece[] Square()
        {
            return new[]
            {
                Piece("A", "cut", BOWL_BCY, 150, 0, 0),
                Piece("B", "cut", BOWL_BCY, 150, 0, 100),
                Piece("C", "fill", BOWL_BCY, 150, 200, 0),
                Piece("D", "fill", BOWL_BCY, 150, 200, 100),
            };
        }

        /// <summary>
        /// Builds one synthetic piece. The long axis is long enough that one bowl is one strip.
        /// </summary>
        /// <param name="Id">Piece id.</param>
        /// <param name="Sign">"cut" or "fill".</param>
        /// <param name="Bcy">Haul volume.</param>
        /// <param name="MeanMm">Mean depth above the floor.</param>
        /// <param name="Row">Centroid row.</param>
        /// <param name="Col">Centroid column.</param>
        /// <param name="Cells">Member cells. Adjacent pieces share an edge through these.</param>
        /// <returns>The piece.</returns>
        private static ResidualPairPiece Piece
            (
            String Id,
            String Sign,
            Double Bcy,
            Double MeanMm,
            Double Row,
            Double Col,
            params (Int32 Row, Int32 Col)[] Cells
            )
        {
            return new ResidualPairPiece(Id, Sign, Bcy, MeanMm, Row, Col, 1000.0, Cells);
        }

        /// <summary>
        /// Flows that are not the balance nodes.
        /// </summary>
        /// <param name="Plan">Plan to filter.</param>
        /// <returns>The real cut-to-fill flows.</returns>
        private static ResidualPairFlow[] RealFlows
            (
            ResidualPairPlan Plan
            )
        {
            return Plan.Flows
                .Where(Flow => (Flow.CutPiece != ResidualPairing.SHORT_ID) && (Flow.FillPiece != ResidualPairing.LOCAL_ID))
                .ToArray();
        }

        /// <summary>
        /// Sum of the scheduled payloads.
        /// </summary>
        /// <param name="Plan">Plan to total.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double Payload
            (
            ResidualPairPlan Plan
            )
        {
            Double Sum = 0.0;
            foreach (ResidualPairTrip Trip in Plan.Trips)
            {
                Sum += Trip.PayloadBcy;
            }
            return Sum;
        }

        /// <summary>
        /// Mean of the depth outside the grade tolerance. Bridged cells inside it add nothing.
        /// </summary>
        /// <param name="Piece">Piece whose cells are averaged.</param>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="ToleranceMm">Grade tolerance in millimetres.</param>
        /// <returns>Millimetres outside the tolerance.</returns>
        private static Double AboveToleranceMean
            (
            ResidualCluster Piece,
            Double[,] ResidualMm,
            Double ToleranceMm
            )
        {
            Double Sum = 0.0;
            foreach ((Int32 Row, Int32 Col) Cell in Piece.Cells)
            {
                Double Depth = ResidualMm[Cell.Row, Cell.Col];
                if (Piece.Sign == "fill")
                {
                    Depth = -Depth;
                }
                if (Depth < 0.0)
                {
                    Depth = 0.0;
                }
                Sum += Math.Max(Depth - ToleranceMm, 0.0);
            }
            return Sum / Piece.Cells.Count;
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
    }
}
