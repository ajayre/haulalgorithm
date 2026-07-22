using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies fast preview runs omit exact guidance while retaining haul plans.
    /// </summary>
    public sealed class SkipGuidanceTests
    {
        /// <summary>
        /// Verifies SkipGuidance returns routes, pair artifacts, and TimeModel metrics
        /// without populating operator guidance waypoints.
        /// </summary>
        [Fact]
        public void SkipGuidance_ReturnsPlanWithoutGuidancePoints
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                HaulBuilderOptions Options = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                    SkipGuidance = true,
                    IncludeCutFillPairCsv = true,
                    IncludeCutFillPairPng = true,
                    IncludeGuidancePng = true,
                };
                Options.Optimization.EnableCycleChainOptimization = false;
                Options.Optimization.EnableRouteConsolidation = false;
                Options.Optimization.PairRefinementRounds = 0;
                Options.Optimization.RouteReorderWindow = 0;

                HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);

                Assert.NotNull(Result.PassResult);
                Assert.True(Result.PassResult.Passes.Count > 0);
                Assert.NotNull(Result.Route);
                Assert.Equal(
                    Result.PassResult.Passes.Count,
                    Result.Route.OrderedPasses.Count);
                Assert.NotNull(Result.Metrics);
                Assert.True(Result.Metrics.DeterministicHours > 0.0);
                Assert.Empty(Result.GuidancePoints);
                Assert.Null(Result.Guidance);
                Assert.Null(Result.LifecycleGuidance);
                Assert.Null(Result.DetailedGuidanceTime);
                Assert.True(Result.CutFillPairs.Count > 0);
                Assert.True(Result.CsvTextByName.ContainsKey("cut_fill_pairs"));
                Assert.True(Result.PngBytesByName.ContainsKey("cut_fill_pairs"));
                Assert.False(Result.CsvTextByName.ContainsKey("guidance"));
                Assert.False(Result.PngBytesByName.ContainsKey("guidance"));
                Assert.Contains(
                    Result.Messages,
                    Message => Message.Contains("SkipGuidance", StringComparison.Ordinal));
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>
        /// Verifies ponding audit still runs when exact guidance is deferred.
        /// </summary>
        [Fact]
        public void SkipGuidance_AllowsPondingAuditWithoutGuidance
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                HaulBuilderOptions Options = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                    SkipGuidance = true,
                };
                Options.Optimization.EnableCycleChainOptimization = false;
                Options.Optimization.EnableRouteConsolidation = false;
                Options.Optimization.PairRefinementRounds = 0;
                Options.Optimization.RouteReorderWindow = 0;
                Options.Ponding.Audit = true;
                Options.Ponding.IncludeMap = true;

                HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);

                Assert.NotNull(Result.Ponding);
                Assert.NotNull(Result.Ponding.FinalAudit);
                Assert.True(Result.CsvTextByName.ContainsKey("ponding"));
                Assert.True(Result.PngBytesByName.ContainsKey("ponding"));
                Assert.Empty(Result.GuidancePoints);
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>
        /// Writes a tiny balanced AGD sufficient for a fast haul-plan smoke run.
        /// </summary>
        /// <returns>Path to a temporary AGD file.</returns>
        private static String CreateMinimalAgd
            (
            )
        {
            String PathValue = Path.Combine(
                Path.GetTempPath(),
                "haulbuilder_skipguidance_"
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
