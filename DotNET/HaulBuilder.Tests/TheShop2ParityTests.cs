using Xunit;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Parity checks against Python on the <c>TheShop2_2ft.agd</c> fixture (plan stage only for speed).
    /// </summary>
    public sealed class TheShop2ParityTests
    {
        /// <summary>
        /// Verifies pass count and moved BCY match Python on TheShop2 at 7.5 ft flow-aligned planning.
        /// </summary>
        [Fact]
        [Trait("Category", "Slow")]
        public void TheShop2_Plan_Matches_Python_Metrics
            (
            )
        {
            String FixturePath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            Assert.True(File.Exists(FixturePath), $"Missing fixture: {FixturePath}");
            HaulBuilderEngine Engine = new HaulBuilderEngine();
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = FixturePath,
                GridSizeFt = 7.5,
                Zones = 0,
                MinLoadFillPct = 90.0,
            };
            HaulBuilderResult Result = Engine.Run(Options);
            // Shared Python/C# plan after no-within-pass-restack (7.5 ft, flow-aligned, min_load 90%):
            // 3102 passes, ~46449 BCY, ~210171 m haul, ~225 h.
            Assert.Equal(3102, Result.PassResult!.Passes.Count);
            Assert.Equal(46449.0, Result.Metrics!.MoveVolumeBcy, 1.0);
            Assert.Equal(210170.7, Result.TimeResult!.HaulingDistM, 5.0);
            Assert.Equal(225.0, Result.Metrics.TotalHours, 1.0);
        }
    }
}
