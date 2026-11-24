using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies that a finished engine run leaves nothing in the process-wide
    /// connector caches, so a caller that runs the engine many times needs no
    /// cleanup call between runs.
    /// </summary>
    [Collection(EngineStateCollection.NAME)]
    public sealed class EngineCacheReleaseTests
    {
        /// <summary>Rows and columns of the minimal field lattice.</summary>
        private const Int32 LATTICE_SIZE = 24;

        /// <summary>First column that needs fill; the columns before it need cut.</summary>
        private const Int32 FILL_COLUMN = 12;

        /// <summary>Latitude of the first row, in degrees.</summary>
        private const Double LATITUDE_DEG = 40.0;

        /// <summary>Longitude of the first column, in degrees.</summary>
        private const Double LONGITUDE_DEG = -93.0;

        /// <summary>Spacing of the survey points, in degrees.</summary>
        private const Double STEP_DEG = 0.00003;

        /// <summary>Design elevation, in metres.</summary>
        private const Double DESIGN_M = 100.0;

        /// <summary>Height of the existing ground above or below the design, in metres.</summary>
        private const Double OFFSET_M = 0.1;

        /// <summary>
        /// Verifies two runs in a row each leave the connector caches empty and
        /// return the same plan.
        /// </summary>
        [Fact]
        public void Run_Twice_LeavesNoCacheEntriesAndRepeatsThePlan
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                Double FirstDistanceM = RunOnce(AgdPath);
                AssertCachesEmpty();

                Double SecondDistanceM = RunOnce(AgdPath);
                AssertCachesEmpty();

                // Releasing the caches must not change a plan.
                Assert.Equal(FirstDistanceM, SecondDistanceM);
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>Runs the engine once on the minimal field.</summary>
        /// <param name="AgdPath">Field file.</param>
        /// <returns>Whole-job driven metres.</returns>
        static private Double RunOnce
            (
            String AgdPath
            )
        {
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = AgdPath,
            };
            HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);
            Assert.NotNull(Result.DetailedGuidanceTime);
            return Result.DetailedGuidanceTime.DistanceM;
        }

        /// <summary>Asserts that no connector result is cached in the process.</summary>
        static private void AssertCachesEmpty
            (
            )
        {
            (Int64 Hits, Int64 Misses, Int32 Entries, Int32 RetainedEntries, Int64 Clears) Statistics =
                TransitPlanner.GetCacheStatistics();
            Assert.Equal(0, Statistics.Entries);
            Assert.Equal(0, Statistics.RetainedEntries);
        }

        /// <summary>
        /// Writes a tiny AGD whose west half needs cut and whose east half needs fill.
        /// </summary>
        /// <returns>Path to a temporary AGD file.</returns>
        static private String CreateMinimalAgd
            (
            )
        {
            String PathValue = Path.Combine(
                Path.GetTempPath(),
                "haulbuilder_cacherelease_"
                    + Guid.NewGuid().ToString("N")
                    + ".agd");
            List<String> Lines = new List<String>
            {
                "Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments",
                "40.0, -93.0, 100.0,,,0MB,",
            };
            for (Int32 Row = 0; Row < LATTICE_SIZE; Row += 1)
            {
                for (Int32 Column = 0; Column < LATTICE_SIZE; Column += 1)
                {
                    Double Existing = DESIGN_M + ((Column < FILL_COLUMN) ? OFFSET_M : -OFFSET_M);
                    Lines.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "{0:F8}, {1:F8}, {2:F3}, {3:F3}, {4:F3}, 3GRD, ",
                        LATITUDE_DEG + (Row * STEP_DEG),
                        LONGITUDE_DEG + (Column * STEP_DEG),
                        Existing,
                        DESIGN_M,
                        DESIGN_M - Existing));
                }
            }

            File.WriteAllText(PathValue, String.Join("\n", Lines) + "\n");
            return PathValue;
        }
    }
}
