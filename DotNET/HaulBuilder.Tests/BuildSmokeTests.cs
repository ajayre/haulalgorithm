namespace HaulBuilder.Tests
{
    /// <summary>
    /// Smoke tests for solution build metadata and shared fixtures.
    /// </summary>
    public sealed class BuildSmokeTests
    {
        /// <summary>
        /// Verifies the assembly exposes a version string for CLI parity.
        /// </summary>
        [Fact]
        public void Version_IsDefined
            (
            )
        {
            Assert.False(String.IsNullOrEmpty(Constants.VERSION));
            Assert.Equal("0.0.1", Constants.VERSION);
        }

        /// <summary>
        /// Verifies the TheShop2 regression AGD fixture is deployed with the test assembly.
        /// </summary>
        [Fact]
        public void TheShop2_Agd_Fixture_Exists
            (
            )
        {
            String FixturePath = Path.Combine(
                AppContext.BaseDirectory,
                "Fixtures",
                "TheShop2_2ft.agd");
            Assert.True(File.Exists(FixturePath), $"Missing fixture: {FixturePath}");
            String Content = File.ReadAllText(FixturePath);
            Assert.Contains("Latitude", Content, StringComparison.OrdinalIgnoreCase);
        }
    }
}
