using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies sequential route guidance PNG generation.
    /// </summary>
    public sealed class RouteGuidanceImageTests
    {
        /// <summary>
        /// Verifies the first N routes each receive one pair PNG key.
        /// </summary>
        [Fact]
        public void RouteGuidanceImageCount_EmitsOnePngPerRoute
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                const Int32 Requested = 3;
                HaulBuilderOptions Options = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                    RouteGuidanceImageCount = Requested,
                    // Tiny fixture runs are shorter than production half-equipment length.
                    MinimumWorkLengthFt = 2.0,
                };
                Options.Optimization.EnableCycleChainOptimization = false;
                Options.Optimization.EnableRouteConsolidation = false;
                Options.Optimization.PairRefinementRounds = 0;
                Options.Optimization.RouteReorderWindow = 0;

                HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);
                Int32 RouteCount = Result.Route?.OrderedPasses.Count ?? 0;
                Int32 Expected = Math.Min(Requested, RouteCount);

                for (Int32 RouteNumber = 1; RouteNumber <= Expected; RouteNumber += 1)
                {
                    String Key = String.Format(
                        CultureInfo.InvariantCulture,
                        "pair_{0:D4}",
                        RouteNumber);
                    Assert.True(Result.PngBytesByName.ContainsKey(Key));
                    Assert.NotEmpty(Result.PngBytesByName[Key]);
                }

                Assert.False(Result.PngBytesByName.ContainsKey("pair_0004"));
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>
        /// Writes a tiny balanced AGD sufficient for a fast guidance smoke run.
        /// </summary>
        /// <returns>Path to a temporary AGD file.</returns>
        private static String CreateMinimalAgd
            (
            )
        {
            String PathValue = Path.Combine(
                Path.GetTempPath(),
                "haulbuilder_routeguidance_"
                    + Guid.NewGuid().ToString("N")
                    + ".agd");
            List<String> Lines = new List<String>
            {
                "Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments",
                "40.0, -93.0, 100.0,,,0MB,",
            };
            for (Int32 Row = 0; Row < 24; Row += 1)
            {
                for (Int32 Col = 0; Col < 24; Col += 1)
                {
                    Double Lat = 40.0 + (Row * 0.00003);
                    Double Lon = -93.0 + (Col * 0.00003);
                    Double Exist = 100.0 + ((Col < 12) ? 0.1 : -0.1);
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
