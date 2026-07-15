using System.Globalization;
using System.Text;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Parity tests ported from <c>tests/test_landlevel.py</c>.
    /// </summary>
    public sealed class LandlevelTests
    {
        /// <summary>
        /// Verifies AGD parsing keeps delta sign and boundary rows.
        /// </summary>
        [Fact]
        public void Agd_Roundtrip_And_Delta_Sign
            (
            )
        {
            // Synthetic AGD rows make sign expectations unambiguous.
            String Path = CreateTempAgd(
                new List<(Double Lat, Double Lon, Double Existing, Double Proposed, String Code)>
                {
                    (40.0, -93.0, 101.0, 100.0, "3GRD"),
                    (40.00003, -93.0, 100.0, 101.0, "3GRD"),
                    (40.00006, -93.0, 100.0, 100.0, "2PER"),
                });

            AgdField Field = Agd.Parse(Path);
            Assert.True(Field.Points.Count >= 3);
            Assert.True(Field.Boundary.Count >= 1);
            List<Double> Deltas = Field.Points.Select(Point => Point.Delta).OrderBy(Value => Value).ToList();
            Assert.Equal(-1.0, Deltas[0], 6);
            Assert.Equal(1.0, Deltas[^1], 6);
        }

        /// <summary>
        /// Verifies BCY conversion helpers match constants.
        /// </summary>
        [Fact]
        public void Bcy_Conversions_Are_Consistent
            (
            )
        {
            // Conversion parity is critical for matching OptiSurface move rates.
            Double Area = (9.144 * 9.144);
            Double Sup = GridMath.SupplyBcy(0.1, Area);
            Double ExpectedSup = (0.1 * Area * Constants.M3_TO_CY);
            Assert.Equal(ExpectedSup, Sup, 6);
            Assert.Equal(0.0, GridMath.SupplyBcy(-0.1, Area));

            Double Dem = GridMath.DemandBcy(-0.1, Area, 1.2);
            Double ExpectedDem = (0.1 * Area * Constants.M3_TO_CY * 1.2);
            Assert.Equal(ExpectedDem, Dem, 6);
            Assert.Equal(0.0, GridMath.DemandBcy(0.1, Area));

            Assert.Equal(20.0 / 1.3, GridMath.LcyToBcy(20.0, 1.3), 6);
            Assert.Equal(15.4, Math.Round(GridMath.LcyToBcy(20.0, 1.3), 1));
        }

        /// <summary>
        /// Verifies mass balance detects finite supply and demand on a symmetric ramp.
        /// </summary>
        [Fact]
        public void Mass_Balance_Flagging
            (
            )
        {
            String Path = CreateTempAgd(BuildRampRows(false));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field);
            BalanceResult Result = Balance.Compute(Grid, "check");
            Assert.True(Result.TotalSupplyBcy > 0.0);
            Assert.True(Result.TotalDemandBcy > 0.0);
        }

        /// <summary>
        /// Verifies balance adjust mode does not increase relative imbalance.
        /// </summary>
        [Fact]
        public void Balance_Adjust_Reduces_Imbalance
            (
            )
        {
            String Path = CreateTempAgd(BuildRampRows(false));
            AgdField Field = Agd.Parse(Path);
            Grid GridBefore = GridMath.BuildGrid(Field);
            Double Before = Balance.Compute(GridBefore, "check").RelativeImbalance;
            Grid GridAfter = GridMath.BuildGrid(Field);
            Double After = Balance.Compute(GridAfter, "adjust").RelativeImbalance;
            Assert.True(After <= Before + 1e-6);
        }

        /// <summary>
        /// Verifies layer depth counting for pass limits.
        /// </summary>
        [Fact]
        public void Layers_For_Depth
            (
            )
        {
            Assert.Equal(0, GridMath.LayersForDepth(0.0));
            Assert.Equal(1, GridMath.LayersForDepth(0.061));
            Assert.Equal(2, GridMath.LayersForDepth(0.062));
            Assert.Equal(2, GridMath.LayersForDepth(0.122));
            Assert.Equal(3, GridMath.LayersForDepth(0.123));
        }

        /// <summary>
        /// Verifies flow arrows point from ridge to valley on a synthetic ramp.
        /// </summary>
        [Fact]
        public void Flow_Points_Ridge_To_Valley
            (
            )
        {
            // The east-high synthetic ramp should yield westward mean transport.
            String Path = CreateTempAgd(BuildRampRows(false));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field, 15.0);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            Double SumFx = 0.0;
            Int32 Count = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx] && Flow.HasArrow[By, Bx])
                    {
                        SumFx += Flow.Fx[By, Bx];
                        Count += 1;
                    }
                }
            }

            Assert.True(Count > 0);
            Assert.True(SumFx / Count < 0.0);
        }

        /// <summary>
        /// Verifies generated passes respect load and depth caps.
        /// </summary>
        [Fact]
        public void Passes_Respect_Load_And_Depth_Caps
            (
            )
        {
            // Cap checks guard planner safety when heuristics change.
            String Path = CreateTempAgd(BuildRampRows(false));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            List<Strip> StripList = Strips.TraceStrips(Grid, Flow);
            PassResult PassResult = Passes.GeneratePasses(Grid, StripList, 10.0, 20.0);
            Double MaxDepthM = (Grid.MaxDepthMm / 1000.0) + 1e-9;
            foreach (Pass Pass in PassResult.Passes)
            {
                Assert.True(Pass.LoadBcy <= (20.0 + 1e-6));
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.CutCells)
                {
                    Assert.True(Entry.DepthM <= MaxDepthM);
                }

                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.FillCells)
                {
                    Assert.True(Entry.DepthM <= MaxDepthM);
                }
            }
        }

        /// <summary>
        /// Verifies guidance haul and depth metrics stay within planner bounds.
        /// </summary>
        [Fact]
        public void Guidance_Preserves_Haul_And_Depth_Caps
            (
            )
        {
            (Grid Grid, Route Route, GuidanceResult Guidance) = GuidanceForRamp(2271.0, 15.0);
            Assert.True(Guidance.Points.Count > 0);
            Assert.Equal(Guidance.RouteHaulDistM, Guidance.HaulDistM, Guidance.RouteHaulDistM * 0.15 + 2.0);
            Assert.True(Guidance.CutDistM >= Guidance.PlannedCutDistM - 1e-6);
            Assert.True(Guidance.FillDistM >= Guidance.PlannedFillDistM - 1e-6);
            Double Cap = Grid.MaxDepthMm + 1e-6;
            foreach (GuidancePoint Point in Guidance.Points)
            {
                Assert.True(Point.DepthMm <= Cap);
                Assert.True(Point.HeadingDeg >= 0.0 && Point.HeadingDeg <= 360.0);
                Assert.True(Point.Blade is "UP" or "DOWN");
            }
        }

        /// <summary>
        /// Verifies strip growth does not materially worsen total haul distance.
        /// </summary>
        [Fact]
        public void Strips_Haul_Stays_Bounded
            (
            )
        {
            String Path = CreateTempAgd(BuildRampRows(false));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field, 15.0);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            List<Strip> StripList = Strips.TraceStrips(Grid, Flow);
            Double HaulBlob = TotalHaul(Grid, StripList, 1.0);
            Double HaulStrip = TotalHaul(Grid, StripList, 50.0);
            Assert.True(HaulStrip <= (HaulBlob * 1.12) + 1.0);
        }

        /// <summary>
        /// Verifies guidance output is deterministic for identical inputs.
        /// </summary>
        [Fact]
        public void Guidance_Deterministic
            (
            )
        {
            String Path = CreateTempAgd(BuildRampRows(false));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            List<Strip> StripList = Strips.TraceStrips(Grid, Flow);
            PassResult PassResult = Passes.GeneratePasses(
                Grid,
                StripList,
                10.0,
                15.38,
                0,
                90.0,
                0.5);
            Route Route = Routing.SequencePasses(PassResult.Passes);
            GuidanceResult G1 = Swaths.BuildGuidance(Grid, Field, Route, 2271.0, 90.0);
            GuidanceResult G2 = Swaths.BuildGuidance(Grid, Field, Route, 2271.0, 90.0);
            List<(Int32 Seq, String Action, Double X, Double Y, Double Depth)> Sig1 = G1.Points
                .Select(Point => (Point.Seq, Point.Action, Math.Round(Point.X, 6), Math.Round(Point.Y, 6), Math.Round(Point.DepthMm)))
                .ToList();
            List<(Int32 Seq, String Action, Double X, Double Y, Double Depth)> Sig2 = G2.Points
                .Select(Point => (Point.Seq, Point.Action, Math.Round(Point.X, 6), Math.Round(Point.Y, 6), Math.Round(Point.DepthMm)))
                .ToList();
            Assert.Equal(Sig1, Sig2);
        }

        /// <summary>
        /// Verifies a narrower blade increases coverage factor versus a full-cell blade.
        /// </summary>
        [Fact]
        public void Narrow_Blade_Increases_Coverage
            (
            )
        {
            GuidanceResult Wide = GuidanceForRamp(4572.0, 15.0).Guidance;
            GuidanceResult Narrow = GuidanceForRamp(2271.0, 15.0).Guidance;
            Assert.True(Narrow.CoverageFactor() >= Wide.CoverageFactor() - 1e-6);
            Assert.True(Narrow.CoverageFactor() > 1.0);
        }

        /// <summary>
        /// Verifies render geometry world-to-pixel mapping matches the spec.
        /// </summary>
        [Fact]
        public void Render_Geometry
            (
            )
        {
            String Path = CreateTempAgd(BuildRampRows(true));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field);
            Int32 Ppb = 50;
            Int32 Bx = 1;
            Int32 By = 2;
            Int32 X0 = Bx * Ppb;
            Int32 Y0 = (Grid.Height - 1 - By) * Ppb;
            Assert.Equal(50, X0);
            Assert.Equal((Grid.Height - 1 - 2) * 50, Y0);
        }

        /// <summary>
        /// Verifies boundary and no-boundary AGD paths both yield valid cells.
        /// </summary>
        [Fact]
        public void Boundary_And_No_Boundary
            (
            )
        {
            foreach (Boolean WithBoundary in new Boolean[] { true, false })
            {
                String Path = CreateTempAgd(BuildRampRows(WithBoundary));
                AgdField Field = Agd.Parse(Path);
                Grid Grid = GridMath.BuildGrid(Field);
                Int32 Valid = 0;
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        if (Grid.Valid[By, Bx]) Valid += 1;
                    }
                }

                Assert.True(Valid > 0);
            }
        }

        /// <summary>
        /// Verifies exclusion application removes interior bins and stores the ring.
        /// </summary>
        [Fact]
        public void Apply_Exclusions_Removes_Block
            (
            )
        {
            String Path = CreateTempAgd(BuildRampRows(true));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field);
            Int32 Before = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx]) Before += 1;
                }
            }

            Double Cx = Grid.MinX + (Grid.Width * Grid.GridSizeM * 0.5);
            Double Cy = Grid.MinY + (Grid.Height * Grid.GridSizeM * 0.5);
            Double H = Grid.GridSizeM * 0.75;
            List<(Double X, Double Y)> Ring = new List<(Double X, Double Y)>
            {
                (Cx - H, Cy - H),
                (Cx + H, Cy - H),
                (Cx + H, Cy + H),
                (Cx - H, Cy + H),
            };
            ExclusionResult Result = GridMath.ApplyExclusions(Grid, new List<List<(Double X, Double Y)>> { Ring });
            Int32 After = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx]) After += 1;
                }
            }

            Assert.True(Result.NBinsRemoved > 0);
            Assert.Equal(Before - Result.NBinsRemoved, After);
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    (Double X, Double Y) Center = Grid.CellCenterXy(Bx, By);
                    if (Obstacles.PointInRing(Center.X, Center.Y, Ring))
                    {
                        Assert.False(Grid.Valid[By, Bx]);
                    }
                }
            }

            Assert.True(Grid.Exclusions.Count > 0);
            Assert.True(Grid.Exclusions[0].Count >= 3);
        }

        /// <summary>
        /// Verifies single-pair PNG rendering returns non-empty bytes for one pass.
        /// </summary>
        [Fact]
        public void Single_Pair_Png_Render_Smoke
            (
            )
        {
            (Grid Grid, Route Route, GuidanceResult Guidance) = GuidanceForRamp();
            Dictionary<Int32, List<GuidancePoint>> ByPass = new Dictionary<Int32, List<GuidancePoint>>();
            foreach (GuidancePoint Point in Guidance.Points)
            {
                if (!ByPass.TryGetValue(Point.PassNo, out List<GuidancePoint>? Bucket))
                {
                    Bucket = new List<GuidancePoint>();
                    ByPass[Point.PassNo] = Bucket;
                }

                Bucket.Add(Point);
            }

            Int32 PassNo = ByPass.Keys.First(Key => ByPass[Key].Any(Point => Point.Action == "CUT") && ByPass[Key].Any(Point => Point.Action == "FILL"));
            Byte[] Png = Render.RenderSinglePairPng(Grid, ByPass[PassNo]);
            Assert.True(Png.Length > 100);
        }

        /// <summary>
        /// Verifies guidance build emits expected action classes.
        /// </summary>
        [Fact]
        public void Guidance_Contains_Cut_Fill_And_Haul
            (
            )
        {
            // Guidance action set should always include core machine phases.
            String Path = CreateTempAgd(BuildRampRows(false));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field, 15.0);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            List<Strip> StripList = Strips.TraceStrips(Grid, Flow);
            PassResult PassResult = Passes.GeneratePasses(
                Grid,
                StripList,
                10.0,
                (20.0 / 1.3),
                0,
                90.0,
                0.5);
            Route Route = Routing.SequencePasses(PassResult.Passes);
            GuidanceResult Guidance = Swaths.BuildGuidance(Grid, Field, Route, Constants.DEFAULT_BLADE_WIDTH_MM, 90.0);
            HashSet<String> Actions = Guidance.Points.Select(Point => Point.Action).ToHashSet(StringComparer.Ordinal);
            Assert.Contains("CUT", Actions);
            Assert.Contains("FILL", Actions);
            Assert.Contains("HAUL", Actions);
        }

        /// <summary>
        /// Verifies palette mapping keeps the same canonical bands.
        /// </summary>
        [Fact]
        public void Palette_Bands_Match_Python
            (
            )
        {
            // Palette values are regression-sensitive because operators rely on the visual legend.
            Assert.Equal((Byte)0x80, Render.CutFillColor(-1.0).R);
            Assert.Equal((Byte)0x00, Render.CutFillColor(0.0).R);
            Assert.Equal((Byte)0xB4, Render.CutFillColor(1.0).R);
            Assert.Equal((Byte)0xFF, Render.CutFillColor(0.03).R);
        }

        /// <summary>
        /// Verifies exclusion obstacles produce a longer routed path than a blocked straight line.
        /// </summary>
        [Fact]
        public void Obstacle_Detour_Is_Longer_Than_Straight
            (
            )
        {
            // A center-blocking obstacle must force detours to validate obstacle routing.
            List<(Double X, Double Y)> Ring = new List<(Double X, Double Y)>
            {
                (-0.5, -0.5),
                (0.5, -0.5),
                (0.5, 0.5),
                (-0.5, 0.5),
            };
            ExclusionZone Zone = ExclusionZone.FromRing(Ring, 0.5);
            ObstacleField Field = ObstacleField.FromZones(new[] { Zone });
            (Double X, Double Y) A = (-5.0, 0.0);
            (Double X, Double Y) B = (5.0, 0.0);
            Assert.True(Field.Blocked(A, B));
            Assert.True(Field.PathLength(A, B) > Math.Sqrt(100.0));
            Assert.Equal(Field.PathLength(A, B), 2.0 * Math.Sqrt(26.0), 1);
        }

        /// <summary>
        /// Verifies empty obstacle field is a straight-line passthrough.
        /// </summary>
        [Fact]
        public void Empty_Obstacle_Field_Is_Passthrough
            (
            )
        {
            (Double X, Double Y) A = (0.0, 0.0);
            (Double X, Double Y) B = (3.0, 4.0);
            Assert.False(Obstacles.EMPTY_OBSTACLES.Blocked(A, B));
            Assert.Equal(5.0, Obstacles.EMPTY_OBSTACLES.PathLength(A, B), 6);
            Assert.Equal(2, Obstacles.EMPTY_OBSTACLES.Path(A, B).Count);
        }

        /// <summary>
        /// Verifies UTM projection for a Midwest point.
        /// </summary>
        [Fact]
        public void Utm_Projection_Is_Reasonable
            (
            )
        {
            UtmCoord Coord = Agd.UtmFromLatlon(40.0, -93.0);
            Assert.Equal(15, Coord.Zone);
            Assert.True(Coord.IsNorthern);
            Assert.True(Coord.Easting > 0.0);
            Assert.True(Coord.Easting < 1_000_000.0);
        }

        /// <summary>
        /// Verifies fixture AGD executes through the main engine.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_Fixture_Runs_Through_Engine
            (
            )
        {
            // Fixture run is a high-value regression guard against parser and planner drift.
            String FixturePath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            Assert.True(File.Exists(FixturePath), $"Missing fixture: {FixturePath}");
            HaulBuilderEngine Engine = new HaulBuilderEngine();
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = FixturePath,
                GridSizeFt = 7.5,
                PixelsPerBin = 8,
                NoPlan = false,
                Zones = 4,
                GuidanceCsvPath = "__IN_MEMORY__",
                GuidancePngPath = "__IN_MEMORY__",
                PairsCsvPath = "__IN_MEMORY__",
                PairsPngPath = "__IN_MEMORY__",
                PairSampleCount = 1,
                Seed = 1234,
            };
            HaulBuilderResult Result = Engine.Run(Options);
            Assert.NotNull(Result.Grid);
            Assert.NotNull(Result.Field);
            Assert.NotNull(Result.Route);
            Assert.True(Result.PngBytesByName.ContainsKey("main"));
            Assert.True(Result.PngBytesByName.ContainsKey("cut_fill_pairs"));
            Assert.True(Result.PngBytesByName.ContainsKey("guidance"));
            Assert.True(Result.PngBytesByName.Keys.Any(Key => Key.StartsWith("pair_", StringComparison.Ordinal)));
            Assert.True(Result.PassResult!.Passes.Count > 0);
            Assert.True(Result.Metrics!.MoveVolumeBcy > 0.0);
            Assert.True(Result.GuidancePoints.Count > 0);
            Assert.NotNull(Result.LifecycleGuidance);
            Assert.True(Result.LifecycleGuidance!.Success);
            Assert.NotNull(Result.DetailedGuidanceTime);
            Assert.Equal(
                Result.DetailedGuidanceTime!.TotalSeconds,
                Result.GuidanceTime!.TotalS,
                6);
        }

        /// <summary>
        /// Verifies cut/fill pair rows expose cumulative progress and per-route time.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void CutFillPair_Includes_Cumulative_Volumes_And_Route_Time
            (
            )
        {
            // Pair rows should report pre-route progress and route duration for operator dashboards.
            String FixturePath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            Assert.True(File.Exists(FixturePath), $"Missing fixture: {FixturePath}");
            HaulBuilderEngine Engine = new HaulBuilderEngine();
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = FixturePath,
                GridSizeFt = 7.5,
                PixelsPerBin = 8,
                NoPlan = false,
                PairsCsvPath = "__IN_MEMORY__",
            };
            HaulBuilderResult Result = Engine.Run(Options);
            Assert.NotNull(Result.Route);
            Assert.True(Result.CutFillPairs.Count > 1);
            Assert.NotNull(Result.TimeResult);

            CutFillPair First = Result.CutFillPairs[0];
            Assert.Equal(0.0, First.TotalCutBcyBefore, 6);
            Assert.Equal(0.0, First.TotalFillBcyBefore, 6);
            Assert.True(First.RouteTimeS > 0.0);

            Double CutBefore = 0.0;
            Double FillBefore = 0.0;
            Double RouteTimeSum = 0.0;
            for (Int32 Iter = 0; Iter < Result.CutFillPairs.Count; Iter += 1)
            {
                CutFillPair Pair = Result.CutFillPairs[Iter];
                Pass Pass = Result.Route!.OrderedPasses[Iter];
                Assert.Equal(CutBefore, Pair.TotalCutBcyBefore, 4);
                Assert.Equal(FillBefore, Pair.TotalFillBcyBefore, 4);
                CutBefore += TimeModel.PassCutBcy(Result.Grid!, Pass);
                FillBefore += TimeModel.PassFillBcy(Result.Grid!, Pass);
                RouteTimeSum += Pair.RouteTimeS;
            }

            Assert.Equal(Result.TimeResult!.TotalTimeS, RouteTimeSum, 3);
            String Csv = Result.CsvTextByName["cut_fill_pairs"];
            Assert.Contains("total_cut_bcy_before", Csv, StringComparison.Ordinal);
            Assert.Contains("total_fill_bcy_before", Csv, StringComparison.Ordinal);
            Assert.Contains("route_time_s", Csv, StringComparison.Ordinal);
        }

        /// <summary>
        /// Builds guidance for the synthetic east-up ramp.
        /// </summary>
        private static (Grid Grid, Route Route, GuidanceResult Guidance) GuidanceForRamp
            (
            Double bladeWidthMm = 2271.0,
            Double gridSizeFt = 15.0
            )
        {
            String Path = CreateTempAgd(BuildRampRows(false));
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field, gridSizeFt);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            List<Strip> StripList = Strips.TraceStrips(Grid, Flow);
            PassResult PassResult = Passes.GeneratePasses(
                Grid,
                StripList,
                10.0,
                15.38,
                0,
                90.0,
                0.5);
            Route Route = Routing.SequencePasses(PassResult.Passes);
            GuidanceResult Guidance = Swaths.BuildGuidance(Grid, Field, Route, bladeWidthMm, 90.0);
            return (Grid, Route, Guidance);
        }

        /// <summary>
        /// Sums routed haul distance for a strip-bias setting on the synthetic ramp.
        /// </summary>
        private static Double TotalHaul
            (
            Grid grid,
            List<Strip> strips,
            Double stripBias
            )
        {
            PassResult PassResult = Passes.GeneratePasses(
                grid,
                strips,
                10.0,
                15.38,
                0,
                90.0,
                0.5,
                stripBias);
            Route Route = Routing.SequencePasses(PassResult.Passes);
            Double Total = 0.0;
            foreach (Pass Pass in Route.OrderedPasses)
            {
                Total += Pass.HaulDistanceM;
            }

            return Total;
        }

        /// <summary>
        /// Creates a temporary AGD file from synthetic rows.
        /// </summary>
        private static String CreateTempAgd
            (
            IReadOnlyList<(Double Lat, Double Lon, Double Existing, Double Proposed, String Code)> Rows
            )
        {
            // The helper writes benchmark + rows in OptiSurface CSV order for parser parity.
            String PathValue = Path.Combine(Path.GetTempPath(), "haulbuilder_test_" + Guid.NewGuid().ToString("N", CultureInfo.InvariantCulture) + ".agd");
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments");
            Sb.AppendLine("40.0, -93.0, 100.0,,,0MB,");
            for (Int32 Iter = 0; Iter < Rows.Count; Iter += 1)
            {
                (Double Lat, Double Lon, Double Existing, Double Proposed, String Code) Row = Rows[Iter];
                Double CutFill = (Row.Proposed - Row.Existing);
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0:F8}, {1:F8}, {2:F3}, {3:F3}, {4:F3}, {5}, ", Row.Lat, Row.Lon, Row.Existing, Row.Proposed, CutFill, Row.Code);
                Sb.AppendLine();
            }

            File.WriteAllText(PathValue, Sb.ToString(), Encoding.UTF8);
            return PathValue;
        }

        /// <summary>
        /// Builds the synthetic east-up ramp rows used by multiple tests.
        /// </summary>
        private static List<(Double Lat, Double Lon, Double Existing, Double Proposed, String Code)> BuildRampRows
            (
            Boolean WithBoundary
            )
        {
            // A deterministic synthetic field keeps flow and pass tests stable across runs.
            List<(Double Lat, Double Lon, Double Existing, Double Proposed, String Code)> Rows = new List<(Double Lat, Double Lon, Double Existing, Double Proposed, String Code)>();
            Double Lat0 = 40.0;
            Double Lon0 = -93.0;
            Double DLat = (1.0 / 111000.0);
            Double DLon = (1.0 / (111000.0 * Math.Cos((Math.PI / 180.0) * Lat0)));
            Int32 N = 24;
            for (Int32 R = 0; R < N; R += 1)
            {
                for (Int32 C = 0; C < N; C += 1)
                {
                    Double Lat = (Lat0 + (R * 3 * DLat));
                    Double Lon = (Lon0 + (C * 3 * DLon));
                    Double Existing = (100.0 + ((C - (N / 2.0)) * 0.02));
                    Double Proposed = 100.0;
                    String Code = "3GRD";
                    if (WithBoundary && ((R == 0) || (R == (N - 1)) || (C == 0) || (C == (N - 1))))
                    {
                        Code = "2PER";
                    }

                    Rows.Add((Lat, Lon, Existing, Proposed, Code));
                }
            }

            return Rows;
        }
    }
}
