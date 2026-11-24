using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies minimum blade-down work length policy and end-of-job deferral.
    /// </summary>
    public sealed class MinimumWorkLengthTests
    {
        /// <summary>
        /// Verifies zero MinimumWorkLengthFt resolves to half the equipment length.
        /// </summary>
        [Fact]
        public void Options_ResolveMinimumWorkLength_DefaultsToHalfEquipment
            (
            )
        {
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                EquipmentLengthFt = 40.0,
                MinimumWorkLengthFt = 0.0,
            };

            Assert.Equal(20.0, Options.ResolveMinimumWorkLengthFt(), 9);
            Assert.Equal(
                20.0 * Constants.FEET_TO_METERS,
                Options.ResolveMinimumWorkLengthM(),
                9);
        }

        /// <summary>
        /// Verifies the minimum-length switch off resolves to zero whatever length is set.
        /// </summary>
        [Fact]
        public void Options_ResolveMinimumWorkLength_OffIgnoresTheLength
            (
            )
        {
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                EquipmentLengthFt = 40.0,
                MinimumWorkLengthFt = 12.0,
                EnableMinimumWorkLength = false,
            };

            // Zero is the coverage planner's keep-every-run value.
            Assert.Equal(0.0, Options.ResolveMinimumWorkLengthFt());
            Assert.Equal(0.0, Options.ResolveMinimumWorkLengthM());
        }

        /// <summary>
        /// Verifies an explicit override wins over the half-equipment default.
        /// </summary>
        [Fact]
        public void Options_ResolveMinimumWorkLength_UsesOverride
            (
            )
        {
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                EquipmentLengthFt = 40.0,
                MinimumWorkLengthFt = 12.0,
            };

            Assert.Equal(12.0, Options.ResolveMinimumWorkLengthFt(), 9);
        }

        /// <summary>
        /// Verifies compact preflight uses the same minimum length gate as coverage.
        /// </summary>
        [Fact]
        public void EvaluateCompactCycle_OmitsSubMinimumRuns
            (
            )
        {
            Grid Grid = MakeGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Bx = 2; Bx <= 10; Bx += 1)
            {
                CutCells.Add(((Bx, 5), 0.02));
            }
            CutCells.Add(((2, 12), 0.03));
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Bx = 2; Bx <= 10; Bx += 1)
            {
                FillCells.Add(((Bx, 8), 0.02));
            }
            Pass Pass = new Pass(
                CutCells,
                FillCells,
                1.0,
                10.0,
                Grid.CellCenterXy(2, 5),
                Grid.CellCenterXy(10, 5),
                Grid.CellCenterXy(2, 8),
                Grid.CellCenterXy(10, 8));

            CoveragePlanner.PlanResult CutWithoutGate = CoveragePlanner.Plan(
                CutCells, Grid, 2.0, null, Pass.FillStartXy, 0.0);
            CoveragePlanner.PlanResult CutWithGate = CoveragePlanner.Plan(
                CutCells, Grid, 2.0, null, Pass.FillStartXy, 3.0);
            GuidancePlanner.CompactCycleResult CompactWithGate =
                GuidancePlanner.EvaluateCompactCycle(
                    Pass, Grid, 2.0, GradeReturn: true, MinimumWorkLengthM: 3.0);

            Assert.True(CutWithoutGate.Runs.Count > CutWithGate.Runs.Count);
            Assert.NotEmpty(CutWithGate.ShortRunOmittedCells);
            Assert.All(
                CutWithGate.Runs,
                Run => Assert.True(
                    Run.LengthM + Constants.NUMERIC_COMPARISON_TOLERANCE >= 3.0));
            Assert.True(CompactWithGate.CutRunCount <= CutWithGate.Runs.Count);
        }

        /// <summary>
        /// Verifies a full engine run with a high minimum length defers short-run cells.
        /// </summary>
        [Fact]
        public void Run_DefersShortGuidanceRunsBelowMinimumLength
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                HaulBuilderOptions Options = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                    // Force aggressive scrap omission on this tiny fixture.
                    MinimumWorkLengthFt = 30.0,
                };
                Options.Optimization.EnableCycleChainOptimization = false;
                Options.Optimization.EnableRouteConsolidation = false;
                Options.Optimization.PairRefinementRounds = 0;
                Options.Optimization.RouteReorderWindow = 0;

                HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);

                Assert.NotNull(Result.Guidance);
                Assert.NotNull(Result.DeferredWorkSummary);
                Assert.NotNull(Result.HaulOptimization);
                Assert.Contains(
                    Result.HaulOptimization.DeferredWork,
                    Entry => Entry.Reason
                        == HaulDeferredWorkReasons.BELOW_MINIMUM_WORK_LENGTH);
                Assert.Contains(
                    Result.Messages,
                    Message => Message.Contains(
                        "cleanup",
                        StringComparison.OrdinalIgnoreCase));
                String Appended = Result.Messages.First(
                    Message => Message.StartsWith("Appended ", StringComparison.Ordinal));
                Int32 Count = Int32.Parse(
                    Appended.Split(' ')[1],
                    CultureInfo.InvariantCulture);
                // The word "route(s)" contains a parenthesis; the volume is the last one.
                Int32 BcyWord = Appended.LastIndexOf(" BCY", StringComparison.Ordinal);
                Int32 Open = Appended.LastIndexOf('(', BcyWord);
                Double Bcy = Double.Parse(
                    Appended.Substring(Open + 1, BcyWord - Open - 1),
                    CultureInfo.InvariantCulture);
                Assert.True(Result.CleanupRouteCount > 0);
                Assert.True(Result.CleanupRouteBcy > 0.0);
                Assert.Equal(Count, Result.CleanupRouteCount);
                Assert.Equal(Bcy, Result.CleanupRouteBcy, 3);
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>
        /// Verifies switching the minimum-length rule off matches a tiny positive length.
        /// </summary>
        [Fact]
        public void Run_MinimumWorkLengthOff_MatchesTinyPositiveLength
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                HaulBuilderOptions Off = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                    EnableMinimumWorkLength = false,
                    MinimumWorkLengthFt = 21.5,
                };
                HaulBuilderOptions Tiny = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                    MinimumWorkLengthFt = 0.001,
                };
                HaulBuilderResult OffResult = new HaulBuilderEngine().Run(Off);
                HaulBuilderResult TinyResult = new HaulBuilderEngine().Run(Tiny);
                Assert.NotNull(OffResult.DetailedGuidanceTime);
                Assert.NotNull(TinyResult.DetailedGuidanceTime);
                Assert.Equal(
                    OffResult.DetailedGuidanceTime.DistanceM,
                    TinyResult.DetailedGuidanceTime.DistanceM);
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>
        /// Verifies an out-of-balance field records the cut/fill warning in Messages.
        /// </summary>
        [Fact]
        public void Run_ReportsCutFillImbalanceWarning
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                HaulBuilderOptions Options = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                };
                HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);
                Assert.Contains(
                    Result.Messages,
                    Message => Message.StartsWith(
                        "WARNING: cut/fill imbalance",
                        StringComparison.Ordinal));
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>Creates a small all-valid test grid.</summary>
        /// <returns>Grid for compact coverage checks.</returns>
        private static Grid MakeGrid
            (
            )
        {
            const Int32 SIZE = 20;
            Boolean[,] Valid = new Boolean[SIZE, SIZE];
            for (Int32 By = 0; By < SIZE; By += 1)
            {
                for (Int32 Bx = 0; Bx < SIZE; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                SIZE,
                SIZE,
                500000.0,
                4000000.0,
                12,
                true,
                0.0,
                0.0,
                new Double[SIZE, SIZE],
                new Double[SIZE, SIZE],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
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
                "haulbuilder_minworklength_"
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
