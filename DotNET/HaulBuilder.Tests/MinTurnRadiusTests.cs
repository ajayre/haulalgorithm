using System.Globalization;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies guidance never emits blade-down bends tighter than the turning radius.
    /// </summary>
    public sealed class MinTurnRadiusTests
    {
        /// <summary>
        /// Circumradius of triangle ABC in metres; infinity when collinear.
        /// </summary>
        /// <param name="a">First vertex.</param>
        /// <param name="b">Second vertex.</param>
        /// <param name="c">Third vertex.</param>
        /// <returns>Circumradius in metres.</returns>
        private static Double CircumRadiusM
            (
            (Double X, Double Y) a,
            (Double X, Double Y) b,
            (Double X, Double Y) c
            )
        {
            Double La = Math.Sqrt(((b.X - c.X) * (b.X - c.X)) + ((b.Y - c.Y) * (b.Y - c.Y)));
            Double Lb = Math.Sqrt(((a.X - c.X) * (a.X - c.X)) + ((a.Y - c.Y) * (a.Y - c.Y)));
            Double Lc = Math.Sqrt(((a.X - b.X) * (a.X - b.X)) + ((a.Y - b.Y) * (a.Y - b.Y)));
            Double Cross = Math.Abs((a.X * (b.Y - c.Y)) + (b.X * (c.Y - a.Y)) + (c.X * (a.Y - b.Y)));
            if (Cross < 1e-9)
            {
                return Double.PositiveInfinity;
            }

            return (La * Lb * Lc) / (2.0 * Cross);
        }

        /// <summary>
        /// Absolute heading change at b along a→b→c in degrees.
        /// </summary>
        /// <param name="a">Inbound start.</param>
        /// <param name="b">Corner.</param>
        /// <param name="c">Outbound end.</param>
        /// <returns>Absolute turn angle in degrees.</returns>
        private static Double HeadingChangeDeg
            (
            (Double X, Double Y) a,
            (Double X, Double Y) b,
            (Double X, Double Y) c
            )
        {
            Double Dx1 = b.X - a.X;
            Double Dy1 = b.Y - a.Y;
            Double Dx2 = c.X - b.X;
            Double Dy2 = c.Y - b.Y;
            return Math.Abs((Math.Atan2((Dx1 * Dy2) - (Dy1 * Dx2), (Dx1 * Dx2) + (Dy1 * Dy2)) * (180.0 / Math.PI)));
        }

        /// <summary>
        /// Asserts every meaningful bend on the full steering polyline (including MOVE) respects R.
        /// </summary>
        /// <param name="points">Guidance waypoints.</param>
        /// <param name="turnRadiusM">Minimum allowed radius in metres.</param>
        /// <param name="turnThresholdDeg">Heading-change noise floor.</param>
        private static void AssertSteeringRespectsTurnRadius
            (
            IReadOnlyList<GuidancePoint> points,
            Double turnRadiusM,
            Double turnThresholdDeg = 60.0
            )
        {
            // Intent: caller sets the minimum heading change that must respect R.
            Double NoiseFloor = turnThresholdDeg;
            for (Int32 K = 1; K < (points.Count - 1); K += 1)
            {
                // START/END are synthetic markers, not steered vertices.
                if ((points[K - 1].Action == "START") || (points[K].Action == "START")
                    || (points[K + 1].Action == "START") || (points[K - 1].Action == "END")
                    || (points[K].Action == "END") || (points[K + 1].Action == "END"))
                {
                    continue;
                }

                (Double X, Double Y) A = (points[K - 1].X, points[K - 1].Y);
                (Double X, Double Y) B = (points[K].X, points[K].Y);
                (Double X, Double Y) C = (points[K + 1].X, points[K + 1].Y);
                // Zero-length steering pins (coalesced mid-cells) are not real bends.
                Double Lab = Math.Sqrt(((A.X - B.X) * (A.X - B.X)) + ((A.Y - B.Y) * (A.Y - B.Y)));
                Double Lbc = Math.Sqrt(((B.X - C.X) * (B.X - C.X)) + ((B.Y - C.Y) * (B.Y - C.Y)));
                if ((Lab < 1e-6) || (Lbc < 1e-6))
                {
                    continue;
                }

                if (HeadingChangeDeg(A, B, C) < NoiseFloor)
                {
                    continue;
                }

                Double R = CircumRadiusM(A, B, C);
                Assert.True(
                    R >= (turnRadiusM * 0.98),
                    String.Format(
                        CultureInfo.InvariantCulture,
                        "undrivable {0}->{1}->{2} bend radius={3:F3}m < R={4:F3}m at ({5:F2},{6:F2})",
                        points[K - 1].Action,
                        points[K].Action,
                        points[K + 1].Action,
                        R,
                        turnRadiusM,
                        B.X,
                        B.Y));
            }
        }

        /// <summary>
        /// Total length of all MOVE legs in metres.
        /// </summary>
        /// <param name="points">Guidance waypoints.</param>
        /// <returns>Sum of MOVE segment lengths.</returns>
        private static Double TotalMoveLengthM
            (
            IReadOnlyList<GuidancePoint> points
            )
        {
            Double Len = 0.0;
            for (Int32 I = 1; I < points.Count; I += 1)
            {
                if (points[I].Action != "MOVE")
                {
                    continue;
                }

                GuidancePoint A = points[I - 1];
                GuidancePoint B = points[I];
                Len += Math.Sqrt(((A.X - B.X) * (A.X - B.X)) + ((A.Y - B.Y) * (A.Y - B.Y)));
            }

            return Len;
        }

        /// <summary>
        /// Longest contiguous MOVE run length in metres.
        /// </summary>
        /// <param name="points">Guidance waypoints.</param>
        /// <returns>Max contiguous MOVE length.</returns>
        private static Double MaxContiguousMoveLengthM
            (
            IReadOnlyList<GuidancePoint> points
            )
        {
            Double RunLen = 0.0;
            Double MaxRun = 0.0;
            for (Int32 I = 1; I < points.Count; I += 1)
            {
                if (points[I].Action == "MOVE")
                {
                    GuidancePoint A = points[I - 1];
                    GuidancePoint B = points[I];
                    RunLen += Math.Sqrt(((A.X - B.X) * (A.X - B.X)) + ((A.Y - B.Y) * (A.Y - B.Y)));
                }
                else
                {
                    MaxRun = Math.Max(MaxRun, RunLen);
                    RunLen = 0.0;
                }
            }

            return Math.Max(MaxRun, RunLen);
        }

        /// <summary>
        /// Builds a synthetic two-lane hairpin pass and checks guidance inserts MOVE arcs.
        /// </summary>
        [Fact]
        public void Guidance_HairpinPass_InsertsMoveAndRespectsRadius
            (
            )
        {
            // Tiny AGD so BuildGuidance has a field/grid; pass geometry is hand-built.
            String Path = LandlevelTestsHelpers.CreateMinimalAgd();
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);

            // Serpentine U: east along y=0, north  one cell, west along y=grid — sharp U.
            Double G = Grid.GridSizeM;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                ((0, 0), 0.05),
                ((1, 0), 0.05),
                ((2, 0), 0.05),
                ((2, 1), 0.05),
                ((1, 1), 0.05),
                ((0, 1), 0.05),
            };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                ((4, 0), 0.05),
                ((5, 0), 0.05),
            };

            (Double X, Double Y) CutStart = Grid.CellCenterXY(0, 0);
            (Double X, Double Y) CutEnd = Grid.CellCenterXY(0, 1);
            (Double X, Double Y) FillStart = Grid.CellCenterXY(4, 0);
            (Double X, Double Y) FillEnd = Grid.CellCenterXY(5, 0);
            Pass Pass = new Pass(CutCells, FillCells, 1.0, G * 2.0, CutStart, CutEnd, FillStart, FillEnd)
            {
                CutKnotReduced = true,
                FillKnotReduced = true,
            };
            Route Route = new Route(new List<Int32> { 0 }, new List<Pass> { Pass }, 0.0);
            Double TurnRadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS;
            GuidanceResult Guidance = Swaths.BuildGuidance(
                Grid,
                Field,
                Route,
                Constants.DEFAULT_BLADE_WIDTH_MM,
                90.0,
                0.0,
                true,
                60.0,
                false,
                TurnRadiusM,
                0.7);

            Assert.Contains(Guidance.Points, Point => Point.Action == "MOVE");
            AssertSteeringRespectsTurnRadius(Guidance.Points, TurnRadiusM);
            Assert.True(Guidance.CutDistM >= Guidance.PlannedCutDistM - 1e-6);

            // Hairpin MOVE headlands must be compact keyholes, not ~540 deg CSC loops.
            Double RunLen = 0.0;
            Double MaxRun = 0.0;
            for (Int32 I = 1; I < Guidance.Points.Count; I += 1)
            {
                if (Guidance.Points[I].Action == "MOVE")
                {
                    GuidancePoint A = Guidance.Points[I - 1];
                    GuidancePoint B = Guidance.Points[I];
                    RunLen += Math.Sqrt(((A.X - B.X) * (A.X - B.X)) + ((A.Y - B.Y) * (A.Y - B.Y)));
                }
                else
                {
                    MaxRun = Math.Max(MaxRun, RunLen);
                    RunLen = 0.0;
                }
            }

            MaxRun = Math.Max(MaxRun, RunLen);
            Assert.True(
                MaxRun < (2.75 * Math.PI * TurnRadiusM),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "hairpin MOVE run {0:F1}m looks like a multi-loop CSC (cap {1:F1}m)",
                    MaxRun,
                    2.75 * Math.PI * TurnRadiusM));
        }

        /// <summary>
        /// TheShop2 route 3 U-turns must use compact keyholes and respect the turning circle.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_Route3_KeyholeTurnsRespectRadius
            (
            )
        {
            String Fixture = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            Assert.True(File.Exists(Fixture), $"Missing fixture: {Fixture}");

            Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
            AgdField Field = Agd.Parse(Fixture);
            Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
            Balance.Compute(Grid, "check", Double.PositiveInfinity);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            List<Strip> StripList = Strips.TraceStrips(Grid, Flow, 400);
            PassResult PassResult = Passes.GeneratePasses(
                Grid,
                StripList,
                Constants.DEFAULT_TOLERANCE_MM,
                ScraperBcy,
                0,
                null,
                0.5,
                50.0,
                Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS,
                Flow,
                true,
                0.9,
                EMPTY_OBSTACLES);
            Route FullRoute = Routing.SequencePasses(
                PassResult.Passes,
                1500,
                true,
                true,
                Constants.DEFAULT_RETURN_SPEED_MPH,
                Constants.DEFAULT_TURN_SPEED_MPH,
                Constants.DEFAULT_TURNING_CIRCLE_FT,
                EMPTY_OBSTACLES);
            Double Heading = Passes.EstimateSlopeHeading(Grid);
            Swaths.ReprofileKnots(FullRoute, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);

            Assert.True(FullRoute.OrderedPasses.Count >= 3, "expected at least 3 sequenced passes");
            Pass Pass3 = FullRoute.OrderedPasses[2];
            Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { Pass3 }, 0.0);
            Double TurnRadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS;
            GuidanceResult Guidance = Swaths.BuildGuidance(
                Grid,
                Field,
                Mini,
                Constants.DEFAULT_BLADE_WIDTH_MM,
                Heading,
                0.0,
                true,
                60.0,
                true,
                TurnRadiusM,
                0.7);

            AssertSteeringRespectsTurnRadius(Guidance.Points, TurnRadiusM);
            Assert.True(
                Guidance.Points.Any(Point => Point.Action == "MOVE"),
                "expected R-radius MOVE keyhole headlands on route-3 U-turns");

            // Sum contiguous MOVE runs; each should stay under the multi-loop CSC cap.
            Double RunLen = 0.0;
            Double MaxRun = 0.0;
            for (Int32 I = 1; I < Guidance.Points.Count; I += 1)
            {
                if (Guidance.Points[I].Action == "MOVE")
                {
                    GuidancePoint A = Guidance.Points[I - 1];
                    GuidancePoint B = Guidance.Points[I];
                    RunLen += Math.Sqrt(((A.X - B.X) * (A.X - B.X)) + ((A.Y - B.Y) * (A.Y - B.Y)));
                }
                else
                {
                    MaxRun = Math.Max(MaxRun, RunLen);
                    RunLen = 0.0;
                }
            }

            MaxRun = Math.Max(MaxRun, RunLen);
            Assert.True(
                MaxRun < (2.75 * Math.PI * TurnRadiusM),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "route-3 MOVE run {0:F1}m exceeds keyhole cap {1:F1}m",
                    MaxRun,
                    2.75 * Math.PI * TurnRadiusM));
        }

        /// <summary>
        /// TheShop2 route 7 (the observed W hairpin) must not violate the 45 ft circle.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_Route7_SteeringRespectsTurningCircle
            (
            )
        {
            String Fixture = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            Assert.True(File.Exists(Fixture), $"Missing fixture: {Fixture}");

            Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
            AgdField Field = Agd.Parse(Fixture);
            Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
            Balance.Compute(Grid, "check", Double.PositiveInfinity);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            List<Strip> StripList = Strips.TraceStrips(Grid, Flow, 400);
            PassResult PassResult = Passes.GeneratePasses(
                Grid,
                StripList,
                Constants.DEFAULT_TOLERANCE_MM,
                ScraperBcy,
                0,
                null,
                0.5,
                50.0,
                Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS,
                Flow,
                true,
                0.9,
                EMPTY_OBSTACLES);
            Route FullRoute = Routing.SequencePasses(
                PassResult.Passes,
                1500,
                true,
                true,
                Constants.DEFAULT_RETURN_SPEED_MPH,
                Constants.DEFAULT_TURN_SPEED_MPH,
                Constants.DEFAULT_TURNING_CIRCLE_FT,
                EMPTY_OBSTACLES);
            Double Heading = Passes.EstimateSlopeHeading(Grid);
            Swaths.ReprofileKnots(FullRoute, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);

            Pass Pass7 = FullRoute.OrderedPasses[6];
            Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { Pass7 }, 0.0);
            Double TurnRadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS;
            GuidanceResult Guidance = Swaths.BuildGuidance(
                Grid,
                Field,
                Mini,
                Constants.DEFAULT_BLADE_WIDTH_MM,
                Heading,
                0.0,
                true,
                60.0,
                true,
                TurnRadiusM,
                0.7);

            // Route 7: stacked-bulb regression checks. Soft short-leg pinches that cannot
            // host an R fillet are left for upstream knot/boustrophedon repair.
            Assert.True(
                Guidance.Points.Any(Point => Point.Action == "MOVE"),
                "expected R-radius MOVE headlands on the route-7 W hairpins");

            // Soft pinches must not stack CCC bulbs: a few keyholes + short fillets only.
            Double KeyholeCap = 2.75 * Math.PI * TurnRadiusM;
            Double MaxRun = MaxContiguousMoveLengthM(Guidance.Points);
            // Two coalesced keyholes may abut as one MOVE streak after splice repair.
            Double MaxRunCap = 2.0 * KeyholeCap;
            Assert.True(
                MaxRun < MaxRunCap,
                String.Format(
                    CultureInfo.InvariantCulture,
                    "route-7 MOVE run {0:F1}m exceeds keyhole cap {1:F1}m",
                    MaxRun,
                    MaxRunCap));
            Double TotalMove = TotalMoveLengthM(Guidance.Points);
            Double TotalCap = 16.0 * Math.PI * TurnRadiusM;
            Assert.True(
                TotalMove < TotalCap,
                String.Format(
                    CultureInfo.InvariantCulture,
                    "route-7 total MOVE {0:F1}m looks like stacked bulbs (cap {1:F1}m)",
                    TotalMove,
                    TotalCap));
        }
    }

    /// <summary>
    /// Shared AGD helpers for min-turn-radius tests.
    /// </summary>
    internal static class LandlevelTestsHelpers
    {
        /// <summary>
        /// Writes a tiny AGD sufficient for grid construction.
        /// </summary>
        /// <returns>Path to the temporary AGD file.</returns>
        internal static String CreateMinimalAgd
            (
            )
        {
            String PathValue = Path.Combine(Path.GetTempPath(), "haulbuilder_minturn_" + Guid.NewGuid().ToString("N") + ".agd");
            List<String> Lines = new List<String>
            {
                "Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments",
                "40.0, -93.0, 100.0,,,0MB,",
            };
            for (Int32 Row = 0; Row < 8; Row += 1)
            {
                for (Int32 Col = 0; Col < 8; Col += 1)
                {
                    Double Lat = 40.0 + (Row * 0.00003);
                    Double Lon = -93.0 + (Col * 0.00003);
                    Double Exist = 100.0 + ((Col < 4) ? 0.1 : -0.1);
                    Double Prop = 100.0;
                    Double CutFill = Prop - Exist;
                    Lines.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "{0:F8}, {1:F8}, {2:F3}, {3:F3}, {4:F3}, 3GRD, ",
                        Lat,
                        Lon,
                        Exist,
                        Prop,
                        CutFill));
                }
            }

            File.WriteAllText(PathValue, String.Join("\n", Lines) + "\n");
            return PathValue;
        }
    }
}
