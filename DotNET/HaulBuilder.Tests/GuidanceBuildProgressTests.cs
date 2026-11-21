using System.Globalization;
using System.Reflection;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies guidance-build progress reporting and cancellation on the public API.
    /// </summary>
    public sealed class GuidanceBuildProgressTests
    {
        /// <summary>
        /// Verifies phase weights map to the documented overall percentage ranges.
        /// </summary>
        [Fact]
        public void FromPlannerPhase_MapsCoverageSequencingAndLifecycle
            (
            )
        {
            GuidanceBuildProgress CoverageStart =
                GuidanceBuildProgress.FromPlannerPhase("coverage", 0, 10);
            GuidanceBuildProgress CoverageDone =
                GuidanceBuildProgress.FromPlannerPhase("coverage", 10, 10);
            GuidanceBuildProgress SequencingStart =
                GuidanceBuildProgress.FromPlannerPhase("sequencing", 0, 10);
            GuidanceBuildProgress SequencingDone =
                GuidanceBuildProgress.FromPlannerPhase("sequencing", 10, 10);
            GuidanceBuildProgress LifecycleDone =
                GuidanceBuildProgress.FromPlannerPhase("lifecycle", 10, 10);

            Assert.Equal("coverage", CoverageStart.Phase);
            Assert.Equal(0.0, CoverageStart.PercentComplete, 3);
            Assert.Equal(10.0, CoverageDone.PercentComplete, 3);
            Assert.Equal(10.0, SequencingStart.PercentComplete, 3);
            Assert.Equal(90.0, SequencingDone.PercentComplete, 3);
            Assert.Equal(100.0, LifecycleDone.PercentComplete, 3);
        }

        /// <summary>
        /// Verifies the engine exposes optional progress and cancellation parameters.
        /// </summary>
        [Fact]
        public void EngineRunExposesGuidanceProgressAndCancellation
            (
            )
        {
            MethodInfo Run = typeof(HaulBuilderEngine).GetMethod(
                "Run",
                BindingFlags.Public | BindingFlags.Instance)!;
            ParameterInfo[] Parameters = Run.GetParameters();

            Assert.Equal(3, Parameters.Length);
            Assert.Equal(typeof(HaulBuilderOptions), Parameters[0].ParameterType);
            Assert.Equal(typeof(IProgress<GuidanceBuildProgress>), Parameters[1].ParameterType);
            Assert.Equal(typeof(CancellationToken), Parameters[2].ParameterType);
            Assert.True(Parameters[1].HasDefaultValue);
            Assert.True(Parameters[2].HasDefaultValue);
        }

        /// <summary>
        /// Verifies a pre-cancelled token aborts guidance construction immediately.
        /// </summary>
        [Fact]
        public void Run_ThrowsWhenGuidanceCancellationRequested
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                HaulBuilderOptions Options = CreateFastGuidanceOptions(AgdPath);
                using CancellationTokenSource Source = new CancellationTokenSource();
                Source.Cancel();

                Assert.Throws<OperationCanceledException>(() =>
                    new HaulBuilderEngine().Run(
                        Options,
                        CancellationToken: Source.Token));
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>
        /// Verifies guidance progress reports monotonic overall percentages.
        /// </summary>
        [Fact]
        public void Run_ReportsGuidanceProgressPercentages
            (
            )
        {
            String AgdPath = CreateMinimalAgd();
            try
            {
                HaulBuilderOptions Options = CreateFastGuidanceOptions(AgdPath);
                List<GuidanceBuildProgress> Snapshots = new List<GuidanceBuildProgress>();
                SyncGuidanceProgress Progress = new SyncGuidanceProgress(
                    Snapshot => Snapshots.Add(Snapshot));

                HaulBuilderResult Result = new HaulBuilderEngine().Run(
                    Options,
                    GuidanceProgress: Progress);

                Assert.NotNull(Result.Guidance);
                Assert.NotEmpty(Snapshots);
                Assert.Equal("coverage", Snapshots[0].Phase);
                Assert.True(Snapshots[^1].PercentComplete >= 90.0);
                for (Int32 Index = 1; Index < Snapshots.Count; Index += 1)
                {
                    Assert.True(
                        Snapshots[Index].PercentComplete
                            >= Snapshots[Index - 1].PercentComplete - 0.001);
                }
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>
        /// Builds fast guidance options for smoke tests.
        /// </summary>
        /// <param name="AgdPath">Temporary AGD path.</param>
        /// <returns>Options tuned for a short guidance run.</returns>
        private static HaulBuilderOptions CreateFastGuidanceOptions
            (
            String AgdPath
            )
        {
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = AgdPath,
                // Tiny fixture runs are shorter than production half-equipment length.
                MinimumWorkLengthFt = 2.0,
            };
            Options.Optimization.EnableCycleChainOptimization = false;
            Options.Optimization.EnableRouteConsolidation = false;
            Options.Optimization.PairRefinementRounds = 0;
            Options.Optimization.RouteReorderWindow = 0;
            return Options;
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
                "haulbuilder_guidanceprogress_"
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

        /// <summary>Collects progress synchronously for deterministic test assertions.</summary>
        private sealed class SyncGuidanceProgress : IProgress<GuidanceBuildProgress>
        {
            private readonly Action<GuidanceBuildProgress> Handler;

            /// <summary>Initializes one synchronous progress sink.</summary>
            /// <param name="Handler">Snapshot handler.</param>
            public SyncGuidanceProgress
                (
                Action<GuidanceBuildProgress> Handler
                )
            {
                this.Handler = Handler;
            }

            /// <summary>Records one progress snapshot immediately.</summary>
            /// <param name="Value">Progress snapshot.</param>
            public void Report
                (
                GuidanceBuildProgress Value
                )
            {
                Handler(Value);
            }
        }
    }
}
