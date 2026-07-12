using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Guards that a single load never re-enters the same cut/fill cell (no hop-back restack).
    /// </summary>
    public sealed class NoWithinPassRestackTests
    {
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
        /// Every cut and fill cell appears at most once per pass at 8.5 ft flow-aligned.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_8p5ft_NoWithinPassCellRestack
            (
            )
        {
            Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
            AgdField Field = Agd.Parse(FixturePath());
            Grid Grid = GridMath.BuildGrid(Field, 8.5);
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

            for (Int32 Index = 0; Index < PassResult.Passes.Count; Index += 1)
            {
                Pass Pass = PassResult.Passes[Index];
                Int32 CutUniq = Pass.CutCells.Select(Entry => (Entry.Cell.Bx, Entry.Cell.By)).Distinct().Count();
                Int32 FillUniq = Pass.FillCells.Select(Entry => (Entry.Cell.Bx, Entry.Cell.By)).Distinct().Count();
                Assert.True(
                    Pass.CutCells.Count == CutUniq,
                    $"Pass {Index} restacked cut cells: ncut={Pass.CutCells.Count} uniq={CutUniq}");
                Assert.True(
                    Pass.FillCells.Count == FillUniq,
                    $"Pass {Index} restacked fill cells: nfill={Pass.FillCells.Count} uniq={FillUniq}");
            }
        }
    }
}