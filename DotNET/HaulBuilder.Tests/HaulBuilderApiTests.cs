using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Contains automated checks for haul builder api.
    /// </summary>

    public sealed class HaulBuilderApiTests
    {
        /// <summary>
        /// Verifies production options expose only supported inputs.
        /// </summary>

        [Fact]
        public void ProductionOptionsExposeOnlySupportedInputs
            (
            )
        {
            String[] Expected =
            {
                nameof(HaulBuilderOptions.AccelerationTimeS),
                nameof(HaulBuilderOptions.AgdPath),
                nameof(HaulBuilderOptions.AnimationMode),
                nameof(HaulBuilderOptions.AnimationPixelsPerCell),
                nameof(HaulBuilderOptions.AnimationStride),
                nameof(HaulBuilderOptions.BladeWidthMm),
                nameof(HaulBuilderOptions.CutFillRatio),
                nameof(HaulBuilderOptions.CutSpeedMph),
                nameof(HaulBuilderOptions.DecelerationTimeS),
                nameof(HaulBuilderOptions.EnablePivot),
                nameof(HaulBuilderOptions.EquipmentLengthFt),
                nameof(HaulBuilderOptions.EquipmentWidthFt),
                nameof(HaulBuilderOptions.ExcludePolygonSpecs),
                nameof(HaulBuilderOptions.FieldEfficiency),
                nameof(HaulBuilderOptions.FillSpeedMph),
                nameof(HaulBuilderOptions.FlowArrowStrideCells),
                nameof(HaulBuilderOptions.GradeSpeedMph),
                nameof(HaulBuilderOptions.GridSizeFt),
                nameof(HaulBuilderOptions.IncludeAnimationFrames),
                nameof(HaulBuilderOptions.IncludeAnimationGif),
                nameof(HaulBuilderOptions.IncludeCutFillPairCsv),
                nameof(HaulBuilderOptions.IncludeCutFillPairPng),
                nameof(HaulBuilderOptions.IncludeGuidancePng),
                nameof(HaulBuilderOptions.IncludePivotAnimationFrames),
                nameof(HaulBuilderOptions.IncludePivotAnimationGif),
                nameof(HaulBuilderOptions.LaneAxisHeadingDeg),
                nameof(HaulBuilderOptions.LoadedTravelSpeedMph),
                nameof(HaulBuilderOptions.MaxDepthMm),
                nameof(HaulBuilderOptions.MinimumTurningDiameterFt),
                nameof(HaulBuilderOptions.Optimization),
                nameof(HaulBuilderOptions.PixelsPerCell),
                nameof(HaulBuilderOptions.Ponding),
                nameof(HaulBuilderOptions.PivotAnimationStride),
                nameof(HaulBuilderOptions.PivotCenter),
                nameof(HaulBuilderOptions.PivotClearanceFt),
                nameof(HaulBuilderOptions.PivotMode),
                nameof(HaulBuilderOptions.PivotMoveCost),
                nameof(HaulBuilderOptions.PivotPixelsPerCell),
                nameof(HaulBuilderOptions.PivotWheelbaseFt),
                nameof(HaulBuilderOptions.RandomRouteImageCount),
                nameof(HaulBuilderOptions.RandomRouteTripletCount),
                nameof(HaulBuilderOptions.RandomSeed),
                nameof(HaulBuilderOptions.ScraperCapacityLcy),
                nameof(HaulBuilderOptions.SkipHaulPlan),
                nameof(HaulBuilderOptions.SkipGuidance),
                nameof(HaulBuilderOptions.SwellFactor),
                nameof(HaulBuilderOptions.ToleranceMm),
                nameof(HaulBuilderOptions.TurnSpeedMph),
                nameof(HaulBuilderOptions.UnloadedTravelSpeedMph),
                nameof(HaulBuilderOptions.WaypointTurnThresholdDeg),
                nameof(HaulBuilderOptions.ZoneCount),
            };

            Assert.Equal(
                Expected.OrderBy(Name => Name),
                PublicProperties(typeof(HaulBuilderOptions)));
        }

        /// <summary>
        /// Verifies optimizer options expose only measured production tuning.
        /// </summary>

        [Fact]
        public void OptimizerOptionsExposeOnlyMeasuredProductionTuning
            (
            )
        {
            String[] Expected =
            {
                nameof(HaulOptimizationOptions.CycleChainCandidatesPerRoute),
                nameof(HaulOptimizationOptions.CycleChainMinimumImprovementSeconds),
                nameof(HaulOptimizationOptions.CycleChainRounds),
                nameof(HaulOptimizationOptions.EdgeMaxLaneSlicesPerCycle),
                nameof(HaulOptimizationOptions.EnableCycleChainOptimization),
                nameof(HaulOptimizationOptions.EnableMultiRouteExchange),
                nameof(HaulOptimizationOptions.EnableRouteConsolidation),
                nameof(HaulOptimizationOptions.FlowPenaltyWeight),
                nameof(HaulOptimizationOptions.MaxLaneSlicesPerCycle),
                nameof(HaulOptimizationOptions.MultiRouteExchangeGroupSize),
                nameof(HaulOptimizationOptions.MultiRouteExchangeMaximumGroups),
                nameof(HaulOptimizationOptions.MultiRouteExchangeMinimumImprovementM),
                nameof(HaulOptimizationOptions.MultiRouteExchangeRounds),
                nameof(HaulOptimizationOptions.PairRefinementRounds),
                nameof(HaulOptimizationOptions.RouteConsolidationGroupSize),
                nameof(HaulOptimizationOptions.RouteConsolidationMaximumCycles),
                nameof(HaulOptimizationOptions.RouteReorderWindow),
            };

            Assert.Equal(
                Expected.OrderBy(Name => Name),
                PublicProperties(typeof(HaulOptimizationOptions)));
        }

        /// <summary>
        /// Verifies ponding options expose the supported C# controls and remain
        /// disabled unless a caller explicitly enables audit or optimization.
        /// </summary>

        [Fact]
        public void PondingOptionsExposeSupportedInputsWithSafeDefaults
            (
            )
        {
            String[] Expected =
            {
                nameof(PondingOptions.Audit),
                nameof(PondingOptions.CheckpointInterval),
                nameof(PondingOptions.DesignRainfallIn),
                nameof(PondingOptions.FailOnWarn),
                nameof(PondingOptions.IncludeMap),
                nameof(PondingOptions.MapPixelsPerCell),
                nameof(PondingOptions.NoImplicitAudit),
                nameof(PondingOptions.ObjectiveMode),
                nameof(PondingOptions.Optimize),
                nameof(PondingOptions.OutletPolygonSpecs),
                nameof(PondingOptions.OutletSpecs),
                nameof(PondingOptions.SpillToleranceFt),
                nameof(PondingOptions.TargetSpillOutlets),
                nameof(PondingOptions.WarnDepthFt),
                nameof(PondingOptions.WarnVolumeBcy),
                nameof(PondingOptions.Weight),
                nameof(PondingOptions.WhiteBoxToolsPath),
                nameof(PondingOptions.WhiteBoxToolsWorkDirectory),
            };
            PondingOptions Options = new PondingOptions();

            Assert.Equal(
                Expected.OrderBy(Name => Name),
                PublicProperties(typeof(PondingOptions)));
            Assert.False(Options.Audit);
            Assert.False(Options.Optimize);
            Assert.Equal(PondingObjectiveMode.PeakVolume, Options.ObjectiveMode);
            Assert.True(Options.TargetSpillOutlets);
            Assert.Equal(3600.0, Options.Weight);
        }

        /// <summary>
        /// Verifies engine does not expose legacy lowercase run alias.
        /// </summary>

        [Fact]
        public void EngineDoesNotExposeLegacyLowercaseRunAlias
            (
            )
        {
            MethodInfo[] PublicMethods = typeof(HaulBuilderEngine)
                .GetMethods(BindingFlags.Public | BindingFlags.Instance
                    | BindingFlags.DeclaredOnly);

            MethodInfo Run = Assert.Single(
                PublicMethods,
                Method => Method.Name == "Run");
            Assert.Equal(typeof(HaulBuilderResult), Run.ReturnType);
            Assert.DoesNotContain(
                PublicMethods,
                Method => Method.Name == "run");
        }

        /// <summary>
        /// Verifies public api does not expose python style method aliases.
        /// </summary>

        [Fact]
        public void PublicApiDoesNotExposePythonStyleMethodAliases
            (
            )
        {
            Type[] ApiTypes =
            {
                typeof(HaulBuilderEngine),
                typeof(Animate),
                typeof(Render),
            };

            Assert.All(ApiTypes, Type => Assert.DoesNotContain(
                Type.GetMethods(BindingFlags.Public | BindingFlags.Static
                    | BindingFlags.Instance | BindingFlags.DeclaredOnly),
                Method => Method.Name.Contains(
                    '_', StringComparison.Ordinal)));
        }

        /// <summary>
        /// Verifies retired diagnostics and result aliases are not public api.
        /// </summary>

        [Fact]
        public void RetiredDiagnosticsAndResultAliasesAreNotPublicApi
            (
            )
        {
            Assert.False(typeof(Passes).IsPublic);
            Assert.False(typeof(GuidanceTime).IsPublic);

            String[] ResultProperties = PublicProperties(
                typeof(HaulBuilderResult)).ToArray();
            Assert.DoesNotContain("GuidanceTime", ResultProperties);
            Assert.DoesNotContain("EdgePatchOmission", ResultProperties);

            String[] MetricProperties = PublicProperties(
                typeof(HaulAnalysisMetrics)).ToArray();
            Assert.DoesNotContain("TotalHours", MetricProperties);
            Assert.DoesNotContain("MoveRateBcyPerHour", MetricProperties);
        }

        /// <summary>
        /// Verifies engine rejects invalid production inputs before planning.
        /// </summary>

        [Fact]
        public void EngineRejectsInvalidProductionInputsBeforePlanning
            (
            )
        {
            String FixturePath = Path.Combine(
                AppContext.BaseDirectory,
                "Fixtures",
                "TheShop2_2ft.agd");
            HaulBuilderEngine Engine = new HaulBuilderEngine();

            Assert.Throws<ArgumentOutOfRangeException>(() => Engine.Run(
                new HaulBuilderOptions
                {
                    AgdPath = FixturePath,
                    FieldEfficiency = 0.0,
                }));
            Assert.Throws<ArgumentOutOfRangeException>(() => Engine.Run(
                new HaulBuilderOptions
                {
                    AgdPath = FixturePath,
                    CutSpeedMph = 0.0,
                }));
            Assert.Throws<ArgumentOutOfRangeException>(() => Engine.Run(
                new HaulBuilderOptions
                {
                    AgdPath = FixturePath,
                    LaneAxisHeadingDeg = Double.NaN,
                }));
            Assert.Throws<ArgumentException>(() => Engine.Run(
                new HaulBuilderOptions
                {
                    AgdPath = FixturePath,
                    AnimationMode = "invalid",
                }));
        }

        /// <summary>
        /// Verifies public properties.
        /// </summary>
        /// <param name="Type">The type used by the operation.</param>
        /// <returns>The resulting public properties.</returns>

        private static IEnumerable<String> PublicProperties
            (
            Type Type
            )
        {
            return Type.GetProperties(BindingFlags.Public
                    | BindingFlags.Instance
                    | BindingFlags.DeclaredOnly)
                .Select(Property => Property.Name)
                .OrderBy(Name => Name);
        }
    }
}
