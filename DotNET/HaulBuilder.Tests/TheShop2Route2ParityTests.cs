using System.Globalization;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Parity checks that TheShop2 sequenced route 2 matches the current Python plan.
    /// </summary>
    public sealed class TheShop2Route2ParityTests
    {
        /// <summary>
        /// Python/C# agreed pass count for TheShop2 at 7.5 ft flow-aligned
        /// (after no-within-pass-restack multi-drop).
        /// </summary>
        private const Int32 PythonPassCount = 3102;

        /// <summary>
        /// Sequenced index of route 2 in the shared Python/C# order.
        /// </summary>
        private const Int32 PythonRoute2OrderIndex = 2199;

        /// <summary>
        /// Route 2 haul distance (metres) after sequencing + knot reprofile.
        /// </summary>
        private const Double PythonRoute2HaulM = 63.763;

        /// <summary>
        /// Route 2 cut-cell entry count (equals unique; one bite per cell per load).
        /// </summary>
        private const Int32 PythonRoute2CutEntries = 5;

        /// <summary>
        /// Route 2 unique cut cells.
        /// </summary>
        private const Int32 PythonRoute2UniqueCutCells = 5;

        /// <summary>
        /// Route 2 first cut cell.
        /// </summary>
        private static readonly (Int32 Bx, Int32 By) PythonRoute2FirstCut = (64, 2);

        /// <summary>
        /// Resolves the TheShop2 fixture path under the test output directory.
        /// </summary>
        /// <returns>Absolute path to <c>TheShop2_2ft.agd</c>.</returns>
        private static String FixturePath
            (
            )
        {
            String PathValue = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            Assert.True(File.Exists(PathValue), $"Missing fixture: {PathValue}");
            return PathValue;
        }

        /// <summary>
        /// Builds the default TheShop2 flow-aligned plan used by Python parity dumps.
        /// </summary>
        /// <param name="reprofileKnots">Whether to run knot reprofile.</param>
        /// <returns>Generated passes, sequenced route, and grid.</returns>
        private static (PassResult Passes, Route Route, Grid Grid) PlanTheShop2
            (
            Boolean reprofileKnots = true
            )
        {
            Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
            AgdField Field = Agd.Parse(FixturePath());
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
            Route Route = Routing.SequencePasses(
                PassResult.Passes,
                1500,
                true,
                true,
                Constants.DEFAULT_RETURN_SPEED_MPH,
                Constants.DEFAULT_TURN_SPEED_MPH,
                Constants.DEFAULT_TURNING_CIRCLE_FT,
                EMPTY_OBSTACLES);
            if (reprofileKnots)
            {
                Double Heading = Passes.EstimateSlopeHeading(Grid);
                Swaths.ReprofileKnots(Route, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);
            }

            return (PassResult, Route, Grid);
        }

        /// <summary>
        /// Verifies pass count matches Python.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_PassCount_MatchesPython
            (
            )
        {
            (PassResult PassResult, Route Route, Grid Grid) = PlanTheShop2(reprofileKnots: false);
            Assert.Equal(PythonPassCount, PassResult.Passes.Count);
        }

        /// <summary>
        /// Verifies the NN sequencer picks the same route-2 successor as Python.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_RouteOrder_SecondPassMatchesPython
            (
            )
        {
            (PassResult PassResult, Route Route, Grid Grid) = PlanTheShop2(reprofileKnots: false);
            Assert.True(Route.Order.Count >= 2);
            Assert.Equal(0, Route.Order[0]);
            Assert.Equal(PythonRoute2OrderIndex, Route.Order[1]);
        }

        /// <summary>
        /// Verifies sequenced route 2 cut geometry matches Python.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_Route2_MatchesPythonCutGeometry
            (
            )
        {
            (PassResult PassResult, Route Route, Grid Grid) = PlanTheShop2(reprofileKnots: true);
            Assert.True(Route.OrderedPasses.Count >= 2);
            Pass Route2 = Route.OrderedPasses[1];
            Int32 Unique = Route2.CutCells.Select(Entry => (Entry.Cell.Bx, Entry.Cell.By)).Distinct().Count();
            (Int32 Bx, Int32 By) First = Route2.CutCells[0].Cell;

            Assert.Equal(PythonRoute2CutEntries, Route2.CutCells.Count);
            Assert.Equal(PythonRoute2UniqueCutCells, Unique);
            Assert.Equal(PythonRoute2HaulM, Route2.HaulDistanceM, 0.05);
            Assert.True(
                (First.Bx == PythonRoute2FirstCut.Bx) && (First.By == PythonRoute2FirstCut.By),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "Route 2 first cut cell was ({0},{1}); Python expects ({2},{3}). Order[1]={4}",
                    First.Bx,
                    First.By,
                    PythonRoute2FirstCut.Bx,
                    PythonRoute2FirstCut.By,
                    Route.Order[1]));
        }

        /// <summary>
        /// Verifies route-2 guidance emits a real multi-waypoint CUT sweep.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_Route2_GuidanceHasCutSweep
            (
            )
        {
            (PassResult PassResult, Route Route, Grid Grid) = PlanTheShop2(reprofileKnots: true);
            AgdField Field = Agd.Parse(FixturePath());
            Pass Route2 = Route.OrderedPasses[1];
            Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { Route2 }, 0.0);
            Double Heading = Passes.EstimateSlopeHeading(Grid);
            GuidanceResult Guidance = Swaths.BuildGuidance(Grid, Field, Mini, Constants.DEFAULT_BLADE_WIDTH_MM, Heading);
            Int32 CutCount = Guidance.Points.Count(Point => Point.Action == "CUT");
            Assert.True(
                CutCount >= 3,
                String.Format(
                    CultureInfo.InvariantCulture,
                    "Expected a multi-waypoint CUT sweep on route 2; got {0} CUT points (haul_m={1:F3}).",
                    CutCount,
                    Route2.HaulDistanceM));
        }
    }
}
