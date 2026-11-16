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
                nameof(HaulBuilderOptions.EnableMinimumWorkLength),
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
                nameof(HaulBuilderOptions.IncludeGuidanceCsv),
                nameof(HaulBuilderOptions.IncludeGuidancePng),
                nameof(HaulBuilderOptions.IncludePivotAnimationFrames),
                nameof(HaulBuilderOptions.IncludePivotAnimationGif),
                nameof(HaulBuilderOptions.LaneAxisHeadingDeg),
                nameof(HaulBuilderOptions.LoadedTravelSpeedMph),
                nameof(HaulBuilderOptions.MaxDepthMm),
                nameof(HaulBuilderOptions.MinimumTurningDiameterFt),
                nameof(HaulBuilderOptions.MinimumWorkLengthFt),
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
                nameof(HaulBuilderOptions.RouteGuidanceImageCount),
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
                nameof(HaulOptimizationOptions.EnableEarlyFullStripBanding),
                nameof(HaulOptimizationOptions.EnableMultiRouteExchange),
                nameof(HaulOptimizationOptions.EnableRouteConsolidation),
                nameof(HaulOptimizationOptions.FlowPenaltyWeight),
                nameof(HaulOptimizationOptions.MaxLaneSlicesPerCycle),
                nameof(HaulOptimizationOptions.MultiRouteExchangeGroupSize),
                nameof(HaulOptimizationOptions.MultiRouteExchangeMaximumGroups),
                nameof(HaulOptimizationOptions.MultiRouteExchangeMinimumImprovementM),
                nameof(HaulOptimizationOptions.MultiRouteExchangeRounds),
                nameof(HaulOptimizationOptions.OptimizeRouteOrderByDistance),
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
        /// Verifies Clone copies every option and shares no list or option group.
        /// </summary>
        [Fact]
        public void CloneCopiesEveryOption
            (
            )
        {
            HaulBuilderOptions Source = new HaulBuilderOptions();
            MutateOptions(Source);
            HaulBuilderOptions Copy = Source.Clone();
            AssertOptionsEqual(Source, Copy);
            Assert.NotSame(Source.Optimization, Copy.Optimization);
            Assert.NotSame(Source.Ponding, Copy.Ponding);
            Assert.NotSame(Source.ExcludePolygonSpecs, Copy.ExcludePolygonSpecs);
            Assert.NotSame(Source.Ponding.OutletSpecs, Copy.Ponding.OutletSpecs);
            Assert.NotSame(Source.Ponding.OutletPolygonSpecs, Copy.Ponding.OutletPolygonSpecs);
        }

        /// <summary>
        /// Gives every settable option a non-default value and adds one list entry.
        /// </summary>
        /// <param name="Target">Options object or nested option group.</param>
        static private void MutateOptions
            (
            Object Target
            )
        {
            foreach (PropertyInfo Property in Target.GetType().GetProperties(
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly))
            {
                if (Property.PropertyType == typeof(List<String>))
                {
                    List<String>? Items = Property.GetValue(Target) as List<String>;
                    if (Items == null)
                    {
                        Assert.Fail("Missing list " + Property.Name);
                    }
                    Items.Add("x");
                    continue;
                }
                if (Property.GetSetMethod() == null)
                {
                    Object? Nested = Property.GetValue(Target);
                    if ((Nested != null) && Property.PropertyType.IsClass)
                    {
                        MutateOptions(Nested);
                        continue;
                    }
                    Assert.Fail("Unhandled get-only property " + Property.Name);
                }
                Property.SetValue(
                    Target,
                    DifferentValue(Property.PropertyType, Property.GetValue(Target)));
            }
        }

        /// <summary>Builds a value different from the current one for a known option type.</summary>
        /// <param name="Type">Property type.</param>
        /// <param name="Current">Current value; a null number counts as zero.</param>
        /// <returns>A different value of the same type.</returns>
        static private Object DifferentValue
            (
            Type Type,
            Object? Current
            )
        {
            Type Underlying = Nullable.GetUnderlyingType(Type) ?? Type;
            if (Underlying == typeof(Double))
            {
                Double Number = (Current == null) ? 0.0 : (Double)Current;
                return Number + 3.0;
            }
            if (Underlying == typeof(Int32))
            {
                Int32 Number = (Current == null) ? 0 : (Int32)Current;
                return Number + 3;
            }
            if (Underlying == typeof(Boolean))
            {
                Boolean Flag = (Current != null) && (Boolean)Current;
                return !Flag;
            }
            if (Underlying == typeof(String))
            {
                String Text = (Current as String) ?? String.Empty;
                return Text + "x";
            }
            if (Underlying.IsEnum)
            {
                Array Values = Enum.GetValues(Underlying);
                Object? Value = Values.GetValue(Values.Length - 1);
                if (Value == null)
                {
                    throw new InvalidOperationException("Missing enum value " + Type);
                }
                return Value;
            }
            throw new InvalidOperationException("Unhandled option type " + Type);
        }

        /// <summary>Asserts every public property of two option objects is equal.</summary>
        /// <param name="Source">Mutated options.</param>
        /// <param name="Copy">Clone of the mutated options.</param>
        static private void AssertOptionsEqual
            (
            Object Source,
            Object Copy
            )
        {
            foreach (PropertyInfo Property in Source.GetType().GetProperties(
                BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly))
            {
                Object? Left = Property.GetValue(Source);
                Object? Right = Property.GetValue(Copy);
                if (Property.PropertyType == typeof(List<String>))
                {
                    Assert.Equal((IEnumerable<String>?)Left, (IEnumerable<String>?)Right);
                    continue;
                }
                if ((Property.GetSetMethod() == null)
                    && Property.PropertyType.IsClass
                    && (Property.PropertyType != typeof(String)))
                {
                    if ((Left == null) || (Right == null))
                    {
                        Assert.Fail("Missing nested options " + Property.Name);
                    }
                    AssertOptionsEqual(Left, Right);
                    continue;
                }
                Assert.Equal(Left, Right);
            }
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
