using System.Globalization;
using System.Linq;
using System.Text;

namespace HaulBuilder
{
    /// <summary>
    /// Production end-to-end in-memory haul-planning pipeline.
    /// </summary>
    public sealed class HaulBuilderEngine
    {
        private const Double M2_PER_ACRE = 4046.8564224;
        private const Double METERS_PER_YARD = 0.9144;
        private const Double METERS_PER_MILE = 1609.344;

        /// <summary>
        /// Executes a full haul-planning run and returns typed in-memory results.
        /// </summary>
        /// <param name="options">Validated production planning options.</param>
        /// <returns>Structured pipeline outputs and render payloads.</returns>
        /// <exception cref="ArgumentNullException">Thrown when options is null.</exception>
        /// <exception cref="ArgumentException">Thrown when required options are invalid.</exception>
        public HaulBuilderResult Run
            (
            HaulBuilderOptions options
            )
        {
            // Options are copied so later caller mutations cannot alter an already produced result.
            if (options is null) throw new ArgumentNullException(nameof(options));
            if (String.IsNullOrWhiteSpace(options.AgdPath)) throw new ArgumentException("AGD path is required.", nameof(options));
            if (!File.Exists(options.AgdPath)) throw new ArgumentException($"AGD file not found: {options.AgdPath}", nameof(options));
            ValidateOptions(options);

            HaulBuilderOptions Opt = CloneOptions(options);
            HaulBuilderResult Result = new HaulBuilderResult
            {
                Options = Opt,
            };

            AgdField Field = Agd.Parse(Opt.AgdPath);
            Grid Grid = GridMath.BuildGrid(Field, Opt.GridSizeFt, Opt.SwellFactor, Opt.CutFillRatio, Opt.MaxDepthMm);
            Result.Field = Field;
            Result.Grid = Grid;

            ObstacleField Obstacles = ResolveExclusions(Result, Field, Grid, Opt);
            Double BalanceThreshold = Obstacles.Active ? Double.PositiveInfinity : Balance.DEFAULT_IMBALANCE_THRESHOLD;
            BalanceResult BalanceResult = Balance.Compute(Grid, "check", BalanceThreshold);
            Result.Balance = BalanceResult;

            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            Result.Flow = Flow;
            Result.PngBytesByName["main"] = Render.RenderPng(
                Grid, Flow, Opt.PixelsPerCell, Opt.FlowArrowStrideCells);

            if (Opt.SkipHaulPlan)
            {
                Result.Messages.Add("Planning stages skipped because SkipHaulPlan was set.");
                return Result;
            }

            Double HeadingDeg = HaulLaneBuilder.NormalizeAxisHeadingDeg(
                Opt.LaneAxisHeadingDeg);
            Double ScraperBcy = (Opt.ScraperCapacityLcy / Opt.SwellFactor);

            PassResult PassResult;
            {
                HaulWorkLedger Ledger = HaulWorkLedger.Build(
                    Grid, Opt.ToleranceMm);
                HaulOptimizationResult Optimized =
                    HaulCycleOptimizer.Optimize(
                        Grid,
                        Flow,
                        Ledger,
                        HaulLaneBuilder.LaneAxis.UserHeading,
                        ScraperBcy,
                        Opt.Optimization.MaxLaneSlicesPerCycle,
                        DeferEdgeWork: true,
                        Opt.Optimization.FlowPenaltyWeight,
                        Obstacles,
                        HeadingDeg);
                List<Pass> OptimizedPasses = Optimized.PassResult.Passes;
                HaulPairDistanceRefiner.Result? PairRefinement = null;
                if (Opt.Optimization.PairRefinementRounds > 0)
                {
                    PairRefinement = HaulPairDistanceRefiner.Refine(
                        OptimizedPasses,
                        Grid,
                        Opt.Optimization.PairRefinementRounds,
                        HaulPairDistanceRefiner.PairMetric.Centroid,
                        Seed: 0,
                        Obstacles: Obstacles);
                    OptimizedPasses = PairRefinement.Passes;
                }
                HaulResidualEdgeCompleter.Result? EdgeCompletion = null;
                EdgeCompletion = HaulResidualEdgeCompleter.Complete(
                        OptimizedPasses,
                        Grid,
                        Flow,
                        Ledger,
                        HaulLaneBuilder.LaneAxis.UserHeading,
                        ScraperBcy,
                        Opt.Optimization.EdgeMaxLaneSlicesPerCycle,
                        Opt.Optimization.FlowPenaltyWeight,
                        Opt.Optimization.PairRefinementRounds,
                        HaulPairDistanceRefiner.PairMetric.Centroid,
                        Opt.BladeWidthMm / 1000.0,
                        GradeReturn: true,
                        Obstacles,
                        HeadingDeg);
                OptimizedPasses = OptimizedPasses
                    .Concat(EdgeCompletion.Passes)
                    .ToList();
                HaulMultiRouteExchangeOptimizer.Result? MultiRouteExchange = null;
                HaulRouteConsolidationOptimizer.Result? RouteConsolidation = null;
                if (Opt.Optimization.EnableRouteConsolidation)
                {
                    RouteConsolidation =
                        HaulRouteConsolidationOptimizer.Optimize(
                            OptimizedPasses,
                            Grid,
                            ScraperBcy,
                            Grid.MaxDepthMm / 1000.0,
                            Opt.BladeWidthMm / 1000.0,
                            GradeReturn: true,
                            Opt.Optimization.RouteConsolidationGroupSize,
                            Opt.Optimization.RouteConsolidationMaximumCycles,
                            Obstacles);
                    OptimizedPasses = RouteConsolidation.Passes;
                    Result.HaulRouteConsolidation = RouteConsolidation;
                }
                if (Opt.Optimization.EnableMultiRouteExchange)
                {
                    MultiRouteExchange = HaulMultiRouteExchangeOptimizer.Optimize(
                        OptimizedPasses,
                        Grid,
                        Grid.MaxDepthMm / 1000.0,
                        Opt.Optimization.MultiRouteExchangeRounds,
                        Opt.Optimization.MultiRouteExchangeGroupSize,
                        Opt.Optimization.MultiRouteExchangeMaximumGroups,
                        MinimumImprovementM:
                            Opt.Optimization.MultiRouteExchangeMinimumImprovementM,
                        BladeWidthM: Opt.BladeWidthMm / 1000.0,
                        GradeReturn: true,
                        Obstacles: Obstacles);
                    OptimizedPasses = MultiRouteExchange.Passes;
                    Result.HaulMultiRouteExchange = MultiRouteExchange;
                }
                HaulDifficultCycleFilter.Result? DifficultFilter = null;
                if ((PairRefinement != null)
                    || (EdgeCompletion != null)
                    || (RouteConsolidation != null)
                    || (MultiRouteExchange != null)
                    || (DifficultFilter != null))
                {
                    IReadOnlyList<HaulDeferredWork> Deferred =
                        PreserveExistingDeferredReasons(
                            Ledger.BuildDeferredWork(
                                OptimizedPasses,
                                DifficultFilter == null
                                    ? "OPTIMIZER_UNASSIGNED"
                                    : "GUIDANCE_COMPACT_PREFLIGHT"),
                            Optimized.DeferredWork);
                    Double UnmatchedCutBcy = Deferred
                        .Where(Entry =>
                            Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
                        .Sum(Entry => Entry.VolumeBcy);
                    Double UnmatchedFillBcy = Deferred
                        .Where(Entry =>
                            Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
                        .Sum(Entry => Entry.VolumeBcy);
                    HaulPlanValidation Validation = Ledger.ValidateComplete(
                        OptimizedPasses,
                        ScraperBcy,
                        Grid.MaxDepthMm / 1000.0,
                        AllowDeferredWork: Deferred.Count > 0);
                    Optimized = new HaulOptimizationResult(
                        new PassResult(
                            OptimizedPasses,
                            Optimized.PassResult.Rounds,
                            UnmatchedCutBcy,
                            UnmatchedFillBcy),
                        Optimized.Axis,
                        Optimized.CutLaneCount,
                        Optimized.FillLaneCount,
                        OptimizedPasses.Sum(Pass => Pass.LoadBcy),
                        UnmatchedCutBcy,
                        UnmatchedFillBcy,
                        Deferred,
                        Validation,
                        PairRefinement?.Swaps ?? 0,
                        PairRefinement?.BeforeM ?? 0.0,
                        PairRefinement?.AfterM ?? 0.0,
                        DifficultFilter?.FailureCount ?? 0,
                        DifficultFilter?.OmittedCycles ?? 0,
                        EdgeCompletion?.Passes.Count ?? 0,
                        EdgeCompletion?.OmittedCycles ?? 0);
                }
                if (!Optimized.Validation.Success)
                {
                    throw new InvalidOperationException(
                        "Optimized haul allocation failed ledger validation: "
                        + String.Join("; ", Optimized.Validation.Issues.Take(10)));
                }
                Result.HaulOptimization = Optimized;
                PassResult = Optimized.PassResult;
                Result.DeferredWorkSummary = HaulDeferredWorkSummary.Create(
                    Ledger.TotalCutBcy,
                    Ledger.TotalFillBcy,
                    Optimized.UnmatchedCutBcy,
                    Optimized.UnmatchedFillBcy,
                    Optimized.DeferredWork);
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Global haul optimization generated {0} cycles with {1:F3} BCY cut and {2:F3} BCY fill explicitly deferred.",
                    PassResult.Passes.Count,
                    Optimized.UnmatchedCutBcy,
                    Optimized.UnmatchedFillBcy));
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Automatic haul omits {0:F3} BCY of balanced work for manual cleanup ({1:F4}% of required fill); the remaining {2:F3} BCY deferred cut is material surplus.",
                    Result.DeferredWorkSummary.ManualCleanupBcy,
                    Result.DeferredWorkSummary.ManualCleanupPercentOfRequiredFill,
                    Result.DeferredWorkSummary.MaterialSurplusCutBcy));
                if (EdgeCompletion != null)
                {
                    Result.Messages.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Residual edge completion retained {0} of {1} generated cycles and left {2} compact-infeasible cycles for manual cleanup.",
                        EdgeCompletion.Passes.Count,
                        EdgeCompletion.GeneratedCycles,
                        EdgeCompletion.OmittedCycles));
                }
                if (Optimized.PairRefinementSwaps > 0)
                {
                    Result.Messages.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Equal-load pair refinement accepted {0} swaps and reduced its geometry proxy from {1:F1} m to {2:F1} m.",
                        Optimized.PairRefinementSwaps,
                        Optimized.PairRefinementBeforeM,
                        Optimized.PairRefinementAfterM));
                }
                if ((MultiRouteExchange != null)
                    && (MultiRouteExchange.AcceptedGroups > 0))
                {
                    Result.Messages.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Multi-route exchange committed {0} closed groups across {1} routes and reduced its compactness proxy from {2:F1} m to {3:F1} m; exact conservation={4}.",
                        MultiRouteExchange.AcceptedGroups,
                        MultiRouteExchange.RoutesChanged,
                        MultiRouteExchange.BeforeM,
                        MultiRouteExchange.AfterM,
                        MultiRouteExchange.ConservationValidated));
                }
                if ((RouteConsolidation != null)
                    && (RouteConsolidation.RemovedCycles > 0))
                {
                    Result.Messages.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Capacity-flexible consolidation removed {0} cycles in {1} attempts and reduced accepted compact-guidance distance from {2:F1} m to {3:F1} m.",
                        RouteConsolidation.RemovedCycles,
                        RouteConsolidation.CandidateAttempts,
                        RouteConsolidation.BeforeGuidanceM,
                        RouteConsolidation.AfterGuidanceM));
                }
                if (Optimized.CompactOmittedCycles > 0)
                {
                    Result.Messages.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Compact guidance preflight explicitly deferred {0} of {1} failing cycles for later analysis.",
                        Optimized.CompactOmittedCycles,
                        Optimized.CompactPreflightFailures));
                }
            }
            Result.PassResult = PassResult;

            Route Route = Routing.SequencePasses(
                PassResult.Passes,
                1500,
                GradeReturn: true,
                DirectionAware: true,
                Opt.UnloadedTravelSpeedMph,
                Opt.TurnSpeedMph,
                Opt.MinimumTurningDiameterFt,
                Obstacles,
                StartCandidateCount: 8);
            Result.Route = Route;

            Int32 ActiveReorderWindow = Opt.Optimization.RouteReorderWindow;
            if (ActiveReorderWindow > 0)
            {
                Routing.RefineRouteOrder(
                    Route,
                    ActiveReorderWindow,
                    GradeReturn: true,
                    DirectionAware: true,
                    Opt.UnloadedTravelSpeedMph,
                    Opt.TurnSpeedMph,
                    Opt.MinimumTurningDiameterFt,
                    12,
                    Obstacles);
            }

            GuidancePlanner.BuildResult? PrecomputedGuidancePlan = null;
            if (Opt.Optimization.EnableCycleChainOptimization)
            {
                Route BaselineRoute = Route;
                PassResult BaselinePassResult = PassResult;
                TimeParams ChainTimeParameters = new TimeParams(
                    Opt.CutSpeedMph,
                    Opt.LoadedTravelSpeedMph,
                    Opt.FillSpeedMph,
                    Opt.UnloadedTravelSpeedMph,
                    Opt.AccelerationTimeS,
                    Opt.DecelerationTimeS,
                    Opt.FieldEfficiency);
                HaulCycleChainOptimizer.Result Chain =
                    HaulCycleChainOptimizer.Optimize(
                        Route.OrderedPasses,
                        Grid,
                        Opt.BladeWidthMm / 1000.0,
                        GradeReturn: true,
                        DirectionAware: true,
                        Opt.UnloadedTravelSpeedMph,
                        Opt.TurnSpeedMph,
                        Opt.MinimumTurningDiameterFt,
                        ChainTimeParameters,
                        Opt.Optimization.CycleChainRounds,
                        Opt.Optimization.CycleChainCandidatesPerRoute,
                        Opt.Optimization.CycleChainMinimumImprovementSeconds,
                        Obstacles);
                PassResult CandidatePassResult = new PassResult(
                    Chain.Passes,
                    PassResult.Rounds,
                    PassResult.UnmatchedCutBcy,
                    PassResult.UnmatchedFillBcy);
                Route CandidateRoute = new Route(
                    Enumerable.Range(0, Chain.Passes.Count).ToList(),
                    Chain.Passes,
                    Routing.CalculateDeadhead(
                        Chain.Passes,
                        GradeReturn: true,
                        Obstacles));
                if (ActiveReorderWindow > 0)
                {
                    Routing.RefineRouteOrder(
                        CandidateRoute,
                        ActiveReorderWindow,
                        GradeReturn: true,
                        DirectionAware: true,
                        Opt.UnloadedTravelSpeedMph,
                        Opt.TurnSpeedMph,
                        Opt.MinimumTurningDiameterFt,
                        12,
                        Obstacles);
                }
                Result.HaulCycleChainOptimization = Chain;
                if (Chain.AcceptedSwaps > 0)
                {
                    GuidanceCostOracle.Result BaselineExact =
                        GuidanceCostOracle.EvaluatePlan(
                            BaselineRoute, Grid, Field, HeadingDeg, Opt);
                    GuidanceCostOracle.Result CandidateExact =
                        GuidanceCostOracle.EvaluatePlan(
                            CandidateRoute, Grid, Field, HeadingDeg, Opt);
                    Boolean Promote = CandidateExact.Success
                        && (!BaselineExact.Success
                            || (CandidateExact.DeterministicSeconds
                                < BaselineExact.DeterministicSeconds - 1e-6));
                    if (Promote)
                    {
                        Route = CandidateRoute;
                        PassResult = CandidatePassResult;
                        Result.PassResult = PassResult;
                        Result.Route = Route;
                        PrecomputedGuidancePlan = CandidateExact.Plan;
                        Result.HaulCycleChainPromoted = true;
                        ReplaceOptimizationPassResult(Result, PassResult);
                        Result.Messages.Add(String.Format(
                            CultureInfo.InvariantCulture,
                            "Optional field-wide chain refinement promoted {0} whole-fill exchanges after exact guidance improved from {1:F1} s to {2:F1} s.",
                            Chain.AcceptedSwaps,
                            BaselineExact.DeterministicSeconds,
                            CandidateExact.DeterministicSeconds));
                    }
                    else
                    {
                        Route = BaselineRoute;
                        PassResult = BaselinePassResult;
                        Result.PassResult = PassResult;
                        Result.Route = Route;
                        PrecomputedGuidancePlan = BaselineExact.Plan;
                        Result.Messages.Add(String.Format(
                            CultureInfo.InvariantCulture,
                            "Optional field-wide chain refinement rejected {0} provisional exchanges because exact guidance did not improve ({1:F1} s baseline, {2:F1} s candidate, candidate success={3}).",
                            Chain.AcceptedSwaps,
                            BaselineExact.DeterministicSeconds,
                            CandidateExact.DeterministicSeconds,
                            CandidateExact.Success));
                    }
                }
            }

            BuildZoneSequence(Result, Grid, Field, Route, Opt);
            BuildPairs(Result, Grid, Field, Route, Opt);
            // Pivot positions must exist before guidance so the final lifecycle can
            // route around the held arm for each executed haul route.
            BuildPivot(Result, Grid, Route, Obstacles, Opt);
            BuildGuidance(Result, Grid, Field, Route, Obstacles, HeadingDeg, Opt,
                PrecomputedGuidancePlan);
            BuildTimingAndMetrics(Result, Grid, PassResult, Route, HeadingDeg, Opt);
            BuildAnimations(Result, Grid, Route, Opt);

            return Result;
        }

        /// <summary>Rebinds allocation diagnostics to a transactionally promoted pass set.</summary>
        private static void ReplaceOptimizationPassResult(
            HaulBuilderResult Result,
            PassResult PassResult)
        {
            if (Result.HaulOptimization == null) return;
            HaulOptimizationResult Previous = Result.HaulOptimization;
            Result.HaulOptimization = new HaulOptimizationResult(
                PassResult,
                Previous.Axis,
                Previous.CutLaneCount,
                Previous.FillLaneCount,
                Previous.TotalAssignedBcy,
                Previous.UnmatchedCutBcy,
                Previous.UnmatchedFillBcy,
                Previous.DeferredWork,
                Previous.Validation,
                Previous.PairRefinementSwaps,
                Previous.PairRefinementBeforeM,
                Previous.PairRefinementAfterM,
                Previous.CompactPreflightFailures,
                Previous.CompactOmittedCycles,
                Previous.EdgeCompletionCycles,
                Previous.EdgeOmittedCycles);
        }

        private static ObstacleField ResolveExclusions
            (
            HaulBuilderResult result,
            AgdField field,
            Grid grid,
            HaulBuilderOptions options
            )
        {
            // Exclusion polygon parsing mirrors CLI behavior and enables both planning and routing detours.
            if (options.ExcludePolygonSpecs.Count <= 0)
            {
                return HaulBuilder.Obstacles.EMPTY_OBSTACLES;
            }

            List<List<(Double Lat, Double Lon)>> Rings = ParsePolygonSpecs(options.ExcludePolygonSpecs);
            List<List<(Double X, Double Y)>> RingsXy = new List<List<(Double X, Double Y)>>();
            foreach (List<(Double Lat, Double Lon)> Ring in Rings)
            {
                RingsXy.Add(Agd.ProjectLatlonRing(Ring));
            }

            GridMath.ApplyExclusions(grid, RingsXy);
            Double ClearanceM = ((options.EquipmentWidthFt * Constants.FEET_TO_METERS) / 2.0);
            ObstacleField ObstacleFieldResult = ObstacleField.FromRings(grid.Exclusions, ClearanceM);
            result.Messages.Add(String.Format(CultureInfo.InvariantCulture, "Applied {0} exclusion polygon(s).", Rings.Count));
            return ObstacleFieldResult;
        }

        private static void BuildZoneSequence
            (
            HaulBuilderResult result,
            Grid grid,
            AgdField field,
            Route route,
            HaulBuilderOptions options
            )
        {
            // Zone sequencing condenses hundreds of passes into a coarse operational work order.
            if (options.ZoneCount <= 0)
            {
                return;
            }

            List<ZoneStep> Steps = Sequence.SequenceCutZones(
                grid, field, route, options.ZoneCount, options.RandomSeed);
            result.ZoneSteps.AddRange(Steps);
            if (Steps.Count > 0)
            {
                String Csv = BuildZoneCsv(Steps);
                result.CsvTextByName["zone_sequence"] = Csv;
                Byte[] Overlay = Render.OverlayZoneWaypoints(
                    result.PngBytesByName["main"], grid, Steps,
                    options.PixelsPerCell);
                result.PngBytesByName["main_with_zones"] = Overlay;
            }
        }

        private static void BuildPairs
            (
            HaulBuilderResult result,
            Grid grid,
            AgdField field,
            Route route,
            HaulBuilderOptions options
            )
        {
            // Pair rows are built explicitly so the engine never relies on file-backed CSV helpers.
            TimeParams TimeParams = new TimeParams(
                options.CutSpeedMph,
                options.LoadedTravelSpeedMph,
                options.FillSpeedMph,
                options.UnloadedTravelSpeedMph,
                options.AccelerationTimeS,
                options.DecelerationTimeS,
                options.FieldEfficiency);
            Double TotalCutBcyBefore = 0.0;
            Double TotalFillBcyBefore = 0.0;
            for (Int32 Iter = 0; Iter < route.OrderedPasses.Count; Iter += 1)
            {
                Pass Pass = route.OrderedPasses[Iter];
                Double RouteTimeS = TimeModel.EstimatePassTime(grid, Pass, TimeParams);
                result.CutFillPairs.Add(
                    new CutFillPair(
                        Iter + 1,
                        Pass.LoadBcy,
                        Pass.HaulDistanceM,
                        Pass.CutStartXy,
                        Pass.CutEndXy,
                        Pass.FillStartXy,
                        Pass.FillEndXy,
                        TotalCutBcyBefore,
                        TotalFillBcyBefore,
                        RouteTimeS));
                TotalCutBcyBefore += TimeModel.PassCutBcy(grid, Pass);
                TotalFillBcyBefore += TimeModel.PassFillBcy(grid, Pass);
            }

            if (options.IncludeCutFillPairCsv)
            {
                result.CsvTextByName["cut_fill_pairs"] = BuildPairsCsv(result.CutFillPairs, field);
            }

            if (options.IncludeCutFillPairPng)
            {
                result.PngBytesByName["cut_fill_pairs"] = Render.RenderPassPairsPng(
                    grid,
                    route.OrderedPasses,
                    options.PixelsPerCell);
            }
        }

        private static void BuildGuidance
            (
            HaulBuilderResult result,
            Grid grid,
            AgdField field,
            Route route,
            ObstacleField obstacles,
            Double headingDeg,
            HaulBuilderOptions options,
            GuidancePlanner.BuildResult? PrecomputedPlan = null
            )
        {
            // Exact production guidance is mandatory for every completed haul plan.

            GuidancePlanner.BuildResult Plan = PrecomputedPlan
                ?? GuidancePlanner.Build(
                    route,
                    grid,
                    field,
                    headingDeg,
                    options,
                    Diagnostic: Message => result.Messages.Add(Message));

            if (obstacles.Active)
            {
                (Plan, GuidanceObstacleRouter.Result StaticRouting) =
                    GuidancePlanner.ApplyObstacleRouting(
                        Plan,
                        route,
                        grid,
                        field,
                        headingDeg,
                        options,
                        obstacles);
                result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Guidance routed {0} blade-up move(s) around exclusion polygons; {1} high-risk move(s) require manual control; added distance {2:F1} m.",
                    StaticRouting.StaticDetourCount,
                    StaticRouting.ManualMoveCount,
                    StaticRouting.AddedDistanceM));
            }

            Dictionary<Int32, ObstacleField>? GuidanceArmObstacles =
                ToGuidanceObstacleFields(result.ArmObstacles);
            if ((GuidanceArmObstacles != null)
                && (GuidanceArmObstacles.Count > 0))
            {
                Double BeforePivotS = Plan.Timing.TotalSeconds;
                result.PivotExtraHaulM = CalculatePivotExtraHaulM(
                    route, obstacles, GuidanceArmObstacles);
                (Plan, GuidanceObstacleRouter.Result PivotRouting) =
                    GuidancePlanner.ApplyObstacleRouting(
                        Plan,
                        route,
                        grid,
                        field,
                        headingDeg,
                        options,
                        obstacles,
                        GuidanceArmObstacles);
                result.PivotAddedTimeS = Math.Max(
                    0.0, Plan.Timing.TotalSeconds - BeforePivotS);
                result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Guidance routed {0} blade-up move(s) around the parked pivot arm; {1} high-risk move(s) require manual control; added {2:F1} m and {3:F1} s.",
                    PivotRouting.PivotDetourCount,
                    PivotRouting.ManualMoveCount,
                    PivotRouting.AddedDistanceM,
                    result.PivotAddedTimeS));
            }
            GuidanceResult Guidance = Plan.Guidance;
            result.Guidance = Guidance;
            result.GuidancePoints.AddRange(Guidance.Points);
            result.LifecycleGuidance = Plan.Lifecycle;
            result.DetailedGuidanceTime = Plan.Timing;

            result.CsvTextByName["guidance"] = BuildGuidanceCsv(Guidance.Points);
            if (options.IncludeGuidancePng)
            {
                result.PngBytesByName["guidance"] = Render.RenderGuidancePng(
                    grid, Guidance.Points, options.PixelsPerCell,
                    options.BladeWidthMm / 1000.0);
            }

            BuildPairSamples(result, grid, Guidance.Points, options);
        }

        private static Double CalculatePivotExtraHaulM(
            Route route,
            ObstacleField staticObstacles,
            IReadOnlyDictionary<Int32, ObstacleField> armObstacles)
        {
            Double ExtraM = 0.0;
            for (Int32 Index = 0; Index < route.OrderedPasses.Count; Index += 1)
            {
                Int32 RouteNumber = Index + 1;
                if (!armObstacles.TryGetValue(
                    RouteNumber, out ObstacleField? Field)
                    || (Field == null))
                {
                    continue;
                }
                Pass Pass = route.OrderedPasses[Index];
                Double CombinedM = Field.PathLength(
                    Pass.CutEndXy, Pass.FillStartXy);
                Double StaticM = staticObstacles.Active
                    ? staticObstacles.PathLength(
                        Pass.CutEndXy, Pass.FillStartXy)
                    : SegmentDistance(
                        Pass.CutEndXy, Pass.FillStartXy);
                ExtraM += Math.Max(0.0, CombinedM - StaticM);
            }
            return ExtraM;
        }

        private static Double SegmentDistance(
            (Double X, Double Y) first,
            (Double X, Double Y) second)
        {
            Double Dx = second.X - first.X;
            Double Dy = second.Y - first.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        private static void BuildPairSamples
            (
            HaulBuilderResult result,
            Grid grid,
            IReadOnlyList<GuidancePoint> points,
            HaulBuilderOptions options
            )
        {
            // Random single-pair PNGs mirror make_pair_images.py for operator-facing cut/haul/fill diagrams.
            if (options.RandomRouteImageCount <= 0)
            {
                return;
            }

            Dictionary<Int32, List<GuidancePoint>> ByPass = new Dictionary<Int32, List<GuidancePoint>>();
            for (Int32 Iter = 0; Iter < points.Count; Iter += 1)
            {
                GuidancePoint Point = points[Iter];
                if (!ByPass.TryGetValue(Point.PassNo, out List<GuidancePoint>? Bucket))
                {
                    Bucket = new List<GuidancePoint>();
                    ByPass[Point.PassNo] = Bucket;
                }

                Bucket.Add(Point);
            }

            List<Int32> Usable = new List<Int32>();
            foreach (KeyValuePair<Int32, List<GuidancePoint>> Entry in ByPass)
            {
                Boolean HasCut = Entry.Value.Any(Point => Point.Action == "CUT");
                Boolean HasFill = Entry.Value.Any(Point => Point.Action == "FILL");
                if (HasCut && HasFill)
                {
                    Usable.Add(Entry.Key);
                }
            }

            if (Usable.Count <= 0)
            {
                return;
            }

            Int32 Take = Math.Min(options.RandomRouteImageCount, Usable.Count);
            Random Rng = new Random(options.RandomSeed);
            List<Int32> Pool = new List<Int32>(Usable);
            for (Int32 Iter = 0; Iter < Pool.Count; Iter += 1)
            {
                Int32 Swap = Iter + Rng.Next(Pool.Count - Iter);
                (Pool[Iter], Pool[Swap]) = (Pool[Swap], Pool[Iter]);
            }

            List<Int32> Chosen = Pool.Take(Take).OrderBy(Value => Value).ToList();
            Double BladeM = (options.BladeWidthMm / 1000.0);
            for (Int32 Iter = 0; Iter < Chosen.Count; Iter += 1)
            {
                Int32 PassNo = Chosen[Iter];
                String Key = String.Format(CultureInfo.InvariantCulture, "pair_{0:D4}", PassNo);
                result.PngBytesByName[Key] = Render.RenderSinglePairPng(grid, ByPass[PassNo], BladeM);
            }
        }

        private static void BuildTimingAndMetrics
            (
            HaulBuilderResult result,
            Grid grid,
            PassResult passResult,
            Route route,
            Double headingDeg,
            HaulBuilderOptions options
            )
        {
            // KPI calculations mirror CLI printouts so API and CLI users see identical operational metrics.
            TimeParams Params = new TimeParams(
                options.CutSpeedMph,
                options.LoadedTravelSpeedMph,
                options.FillSpeedMph,
                options.UnloadedTravelSpeedMph,
                options.AccelerationTimeS,
                options.DecelerationTimeS,
                options.FieldEfficiency);
            TimeResult Time = Timemodel.EstimateTime(grid, route, Params);
            result.TimeResult = Time;

            Double MovedBcy = 0.0;
            for (Int32 Iter = 0; Iter < passResult.Passes.Count; Iter += 1)
            {
                Pass Pass = passResult.Passes[Iter];
                MovedBcy += Pass.LoadBcy;
            }

            Int32 Valid = 0;
            for (Int32 By = 0; By < grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < grid.Width; Bx += 1)
                {
                    if (grid.Valid[By, Bx]) Valid += 1;
                }
            }

            Double AreaAc = ((Valid * grid.AreaM2) / M2_PER_ACRE);
            Double DeterministicH = result.DetailedGuidanceTime != null
                ? (result.DetailedGuidanceTime.MotionSeconds
                    + result.DetailedGuidanceTime.EventSeconds) / 3600.0
                : (Time.TotalTimeS - Time.OtherTimeS) / 3600.0;
            Double ScheduledH = result.DetailedGuidanceTime != null
                ? result.DetailedGuidanceTime.TotalSeconds / 3600.0
                : Time.TotalTimeS / 3600.0;
            Double DeterministicMoveRate = DeterministicH > 0.0
                ? MovedBcy / DeterministicH
                : 0.0;
            Double ScheduledMoveRate = ScheduledH > 0.0
                ? MovedBcy / ScheduledH
                : 0.0;
            Double GoldenAchievementPercent =
                100.0 * DeterministicMoveRate
                    / Constants.GOLDEN_MOVE_RATE_BCY_PER_HOUR;
            Double AvgMoveYd = passResult.Passes.Count > 0 ? ((Time.HaulingDistM / passResult.Passes.Count) / METERS_PER_YARD) : 0.0;
            result.Metrics = new HaulAnalysisMetrics
            {
                ValidAreas = Valid,
                FieldAreaAcres = AreaAc,
                MoveVolumeBcy = MovedBcy,
                DeterministicHours = DeterministicH,
                ScheduledHours = ScheduledH,
                DeterministicMoveRateBcyPerHour = DeterministicMoveRate,
                ScheduledMoveRateBcyPerHour = ScheduledMoveRate,
                MoveVolumeBcyPerAcre = AreaAc > 0.0
                    ? MovedBcy / AreaAc
                    : 0.0,
                GoldenMoveRateTargetBcyPerHour =
                    Constants.GOLDEN_MOVE_RATE_BCY_PER_HOUR,
                GoldenMoveRateAchievementPercent = GoldenAchievementPercent,
                MeetsGoldenMoveRateTarget = DeterministicMoveRate
                    >= Constants.GOLDEN_MOVE_RATE_BCY_PER_HOUR,
                AreaRateAcresPerHour = ScheduledH > 0.0
                    ? AreaAc / ScheduledH
                    : 0.0,
                AverageMoveDistanceYards = AvgMoveYd,
                LaneAxisHeadingDeg = headingDeg,
                PivotExtraHaulM = result.PivotExtraHaulM,
                PivotAddedTimeS = result.PivotAddedTimeS,
                ManualCleanupBcy = result.DeferredWorkSummary?.ManualCleanupBcy
                    ?? 0.0,
                DeferredCutBcy = result.DeferredWorkSummary?.DeferredCutBcy
                    ?? passResult.UnmatchedCutBcy,
                DeferredFillBcy = result.DeferredWorkSummary?.DeferredFillBcy
                    ?? passResult.UnmatchedFillBcy,
            };
            result.Messages.Add(String.Format(
                CultureInfo.InvariantCulture,
                "Productivity: {0:F1} BCY/deterministic guidance hour versus {1:F1} golden target ({2:F1}%); scheduled productivity={3:F1} BCY/h; automatic volume={4:F1} BCY/acre.",
                DeterministicMoveRate,
                Constants.GOLDEN_MOVE_RATE_BCY_PER_HOUR,
                GoldenAchievementPercent,
                ScheduledMoveRate,
                result.Metrics.MoveVolumeBcyPerAcre));
        }

        private static void BuildAnimations
            (
            HaulBuilderResult result,
            Grid grid,
            Route route,
            HaulBuilderOptions options
            )
        {
            // Base leveling animation can be emitted as per-frame PNGs and/or one GIF.
            Boolean NeedAnimation = options.IncludeAnimationGif
                || options.IncludeAnimationFrames;
            if (!NeedAnimation) return;
            Animate.AnimationPayload Payload = Animate.AnimateLeveling(
                grid, route.OrderedPasses, options.AnimationStride,
                options.AnimationPixelsPerCell, 80, options.AnimationMode);
            if (options.IncludeAnimationGif && (Payload.GifBytes is not null))
                result.AnimationGifBytes = Payload.GifBytes;
            if (!options.IncludeAnimationFrames) return;
            for (Int32 Iter = 0; Iter < Payload.FramePngBytes.Count; Iter += 1)
            {
                result.PngBytesByName[String.Format(CultureInfo.InvariantCulture, "animation_frame_{0:D5}", Iter)] = Payload.FramePngBytes[Iter];
            }
        }

        private static void BuildPivot
            (
            HaulBuilderResult result,
            Grid grid,
            Route route,
            ObstacleField obstacles,
            HaulBuilderOptions options
            )
        {
            // Pivot stage is optional and includes both static schedule artifacts and optional animation.
            if (!options.EnablePivot)
            {
                return;
            }

            Pivot.Grid PivotGrid = ToPivotGrid(grid);
            List<Pivot.Pass> PivotPasses = ToPivotPasses(route.OrderedPasses);
            Pivot.ObstacleField PivotObstacles = ToPivotObstacleField(obstacles);
            (Double X, Double Y)? PivotCenter = ResolvePivotCenter(
                result, grid, options.PivotCenter);
            Pivot.PivotGeometry Geometry = Pivot.ComputePivotGeometry(
                PivotGrid,
                options.PivotWheelbaseFt,
                options.PivotClearanceFt,
                PivotCenter);
            Pivot.PivotPlan Plan = Pivot.PlanPivot(PivotGrid, PivotPasses, Geometry, 720, options.PivotMoveCost);
            Dictionary<Int32, Pivot.ObstacleField> Arm = Pivot.BuildArmObstacles(Geometry, Plan, PivotObstacles, ((options.EquipmentWidthFt * Constants.FEET_TO_METERS) / 2.0));
            (Int32 Routed, Double ExtraHaulM, Int32 UnderArm, _) =
                Pivot.PivotDetourStats(PivotPasses, Arm);
            result.PivotGeometry = Geometry;
            result.PivotPlan = Plan;
            result.ArmObstacles = Arm;
            result.PivotExtraHaulM = ExtraHaulM;
            result.PivotCsvData = Pivot.WritePivotCsv(Plan);
            result.CsvTextByName["pivot"] = BuildPivotCsv(result.PivotCsvData);
            result.Messages.Add(String.Format(
                CultureInfo.InvariantCulture,
                "Pivot schedule selected {0} position(s), {1} move(s), {2} loaded-haul detour candidate(s), and {3} work-under-arm conflict(s).",
                Plan.Positions.Count,
                Plan.MoveCount,
                Routed,
                UnderArm));

            String PivotDir = Path.Combine(Path.GetTempPath(), "haulbuilder_pivot_png_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(PivotDir);
            try
            {
                List<String> Paths = Pivot.RenderPivotPngs(
                    PivotGrid, PivotPasses, Plan, PivotDir,
                    options.PivotMode, options.PivotPixelsPerCell);
                foreach (String PathItem in Paths)
                {
                    String Name = Path.GetFileNameWithoutExtension(PathItem);
                    result.PivotPngBytesByName[Name] = File.ReadAllBytes(PathItem);
                }
            }
            finally
            {
                try
                {
                    Directory.Delete(PivotDir, true);
                }
                catch (IOException)
                {
                    // Best-effort cleanup only.
                }
                catch (UnauthorizedAccessException)
                {
                    // Best-effort cleanup only.
                }
            }

            if (options.IncludePivotAnimationGif
                || options.IncludePivotAnimationFrames)
            {
                Animate.AnimationPayload Payload = Animate.AnimatePivotLeveling(
                    grid,
                    route.OrderedPasses,
                    Plan,
                    options.PivotAnimationStride,
                    options.PivotPixelsPerCell,
                    options.PivotMode);
                if (options.IncludePivotAnimationGif)
                    result.PivotAnimationGifBytes = Payload.GifBytes;
                if (!options.IncludePivotAnimationFrames) return;
                for (Int32 Iter = 0; Iter < Payload.FramePngBytes.Count; Iter += 1)
                {
                    result.PivotPngBytesByName[String.Format(CultureInfo.InvariantCulture, "pivot_animation_frame_{0:D5}", Iter)] = Payload.FramePngBytes[Iter];
                }
            }
        }

        private static Pivot.Grid ToPivotGrid
            (
            Grid grid
            )
        {
            // Adapt top-level grid into Pivot's nested grid DTO.
            return new Pivot.Grid(grid.GridSizeM, grid.MinX, grid.MinY, grid.Zone, grid.IsNorthern, grid.Existing, grid.Target, grid.Valid);
        }

        private static (Double X, Double Y)? ResolvePivotCenter(
            HaulBuilderResult result,
            Grid grid,
            String? specification)
        {
            if (String.IsNullOrWhiteSpace(specification)) return null;
            String[] Parts = specification.Split(',');
            if ((Parts.Length != 2)
                || !Double.TryParse(
                    Parts[0].Trim(), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out Double Lat)
                || !Double.TryParse(
                    Parts[1].Trim(), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out Double Lon))
            {
                throw new ArgumentException(
                    "PivotCenter must be formatted as lat,lon.",
                    nameof(specification));
            }
            UtmCoord Center = Agd.UtmFromLatlon(Lat, Lon);
            if ((Center.Zone != grid.Zone)
                || (Center.IsNorthern != grid.IsNorthern))
            {
                result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Pivot center projects to UTM zone {0}{1}, while the field uses {2}{3}; continuing with projected coordinates.",
                    Center.Zone,
                    Center.IsNorthern ? "N" : "S",
                    grid.Zone,
                    grid.IsNorthern ? "N" : "S"));
            }
            return (Center.Easting, Center.Northing);
        }

        private static List<Pivot.Pass> ToPivotPasses
            (
            IReadOnlyList<Pass> passes
            )
        {
            // Convert pass tuple cells into Pivot.PassCell sequences.
            List<Pivot.Pass> Out = new List<Pivot.Pass>();
            for (Int32 Iter = 0; Iter < passes.Count; Iter += 1)
            {
                List<Pivot.PassCell> Cut = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in passes[Iter].CutCells)
                {
                    Cut.Add(new Pivot.PassCell(Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM));
                }

                List<Pivot.PassCell> Fill = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in passes[Iter].FillCells)
                {
                    Fill.Add(new Pivot.PassCell(Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM));
                }

                Out.Add(new Pivot.Pass(Cut, Fill, passes[Iter].CutEndXy, passes[Iter].FillStartXy));
            }

            return Out;
        }

        private static Pivot.ObstacleField ToPivotObstacleField
            (
            ObstacleField obstacles
            )
        {
            // Rebuild Pivot obstacle geometry from top-level obstacle zones.
            if (!obstacles.Active)
            {
                return Pivot.ObstacleField.EMPTY_OBSTACLES;
            }

            List<Pivot.ExclusionZone> Zones = new List<Pivot.ExclusionZone>();
            foreach (ExclusionZone Zone in obstacles.Zones)
            {
                Zones.Add(new Pivot.ExclusionZone(Zone.Ring, Zone.BufferRing));
            }

            return Pivot.ObstacleField.FromZones(Zones);
        }

        private static Dictionary<Int32, ObstacleField>?
            ToGuidanceObstacleFields
            (
            Dictionary<Int32, Pivot.ObstacleField>? obstacles
            )
        {
            if ((obstacles == null) || (obstacles.Count == 0)) return null;
            Dictionary<Pivot.ObstacleField, ObstacleField> Cache =
                new Dictionary<Pivot.ObstacleField, ObstacleField>();
            Dictionary<Int32, ObstacleField> Result =
                new Dictionary<Int32, ObstacleField>();
            foreach (KeyValuePair<Int32, Pivot.ObstacleField> Entry in obstacles)
            {
                if (!Cache.TryGetValue(Entry.Value, out ObstacleField? Field))
                {
                    List<ExclusionZone> Zones = new List<ExclusionZone>();
                    foreach (Pivot.ExclusionZone Zone in Entry.Value.Zones)
                    {
                        List<(Double X, Double Y)> Ring =
                            new List<(Double X, Double Y)>(Zone.Ring);
                        List<(Double X, Double Y)> BufferRing =
                            new List<(Double X, Double Y)>(Zone.BufferRing);
                        Zones.Add(new ExclusionZone(
                            Ring,
                            BufferRing,
                            Zone.BBox));
                    }
                    Field = ObstacleField.FromZones(Zones);
                    Cache[Entry.Value] = Field;
                }
                Result[Entry.Key] = Field;
            }
            return Result;
        }

        private static List<List<(Double Lat, Double Lon)>> ParsePolygonSpecs
            (
            IReadOnlyList<String> specs
            )
        {
            // Parsing follows the CLI grammar: repeated flag, each as lat,lon pairs separated by semicolons.
            List<List<(Double Lat, Double Lon)>> Rings = new List<List<(Double Lat, Double Lon)>>();
            for (Int32 Iter = 0; Iter < specs.Count; Iter += 1)
            {
                String Spec = specs[Iter];
                List<(Double Lat, Double Lon)> Ring = new List<(Double Lat, Double Lon)>();
                String[] Chunks = Spec.Replace("\n", ";", StringComparison.Ordinal).Split(';', StringSplitOptions.RemoveEmptyEntries);
                for (Int32 C = 0; C < Chunks.Length; C += 1)
                {
                    String[] Parts = Chunks[C].Split(',', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);
                    if (Parts.Length != 2)
                    {
                        throw new ArgumentException($"Invalid exclusion vertex '{Chunks[C]}'. Expected 'lat,lon'.");
                    }

                    Ring.Add((Double.Parse(Parts[0], CultureInfo.InvariantCulture), Double.Parse(Parts[1], CultureInfo.InvariantCulture)));
                }

                if (Ring.Count < 3) throw new ArgumentException("Exclusion polygon needs at least 3 vertices.");
                Rings.Add(Ring);
            }

            return Rings;
        }

        private static String BuildZoneCsv
            (
            IReadOnlyList<ZoneStep> steps
            )
        {
            // CSV building in-memory replaces filesystem writes while keeping parity with Python column order.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("work_order,zone_id,compass,loads,moved_bcy,easting_m,northing_m,lat,lon,first_pass,last_pass,mean_pass");
            for (Int32 Iter = 0; Iter < steps.Count; Iter += 1)
            {
                dynamic S = steps[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3},{4:F1},{5:F2},{6:F2},{7:F7},{8:F7},{9},{10},{11:F0}", (Int32)S.WorkOrder, (Int32)S.ZoneId, (String)S.Compass, (Int32)S.NLoads, (Double)S.TotalBcy, (Double)S.Easting, (Double)S.Northing, (Double)S.Lat, (Double)S.Lon, (Int32)S.FirstPass, (Int32)S.LastPass, (Double)S.MeanPass);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        private static String BuildPairsCsv
            (
            IReadOnlyList<CutFillPair> pairs,
            AgdField field
            )
        {
            // Pair CSV mirrors the Python writer while remaining pure in-memory.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("order,load_bcy,haul_m,total_cut_bcy_before,total_fill_bcy_before,route_time_s,cut_start_lat,cut_start_lon,cut_end_lat,cut_end_lon,fill_start_lat,fill_start_lon,fill_end_lat,fill_end_lon");
            for (Int32 Iter = 0; Iter < pairs.Count; Iter += 1)
            {
                CutFillPair P = pairs[Iter];
                (Double CsLat, Double CsLon) = Agd.LatLonFromUtm(P.CutStartXy.X, P.CutStartXy.Y, field.Zone, field.IsNorthern);
                (Double CeLat, Double CeLon) = Agd.LatLonFromUtm(P.CutEndXy.X, P.CutEndXy.Y, field.Zone, field.IsNorthern);
                (Double FsLat, Double FsLon) = Agd.LatLonFromUtm(P.FillStartXy.X, P.FillStartXy.Y, field.Zone, field.IsNorthern);
                (Double FeLat, Double FeLon) = Agd.LatLonFromUtm(P.FillEndXy.X, P.FillEndXy.Y, field.Zone, field.IsNorthern);
                Sb.AppendFormat(
                    CultureInfo.InvariantCulture,
                    "{0},{1:F2},{2:F1},{3:F2},{4:F2},{5:F1},{6:F7},{7:F7},{8:F7},{9:F7},{10:F7},{11:F7},{12:F7},{13:F7}",
                    P.Order,
                    P.LoadBcy,
                    P.HaulDistanceM,
                    P.TotalCutBcyBefore,
                    P.TotalFillBcyBefore,
                    P.RouteTimeS,
                    CsLat,
                    CsLon,
                    CeLat,
                    CeLon,
                    FsLat,
                    FsLon,
                    FeLat,
                    FeLon);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        private static String BuildGuidanceCsv
            (
            IReadOnlyList<GuidancePoint> points
            )
        {
            // Minimal guidance export covers the core turn-by-turn fields consumed by operator tools.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("seq,pass_no,action,x,y,lat,lon,depth_mm,heading_deg,blade,instruction");
            for (Int32 Iter = 0; Iter < points.Count; Iter += 1)
            {
                dynamic P = points[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3:F3},{4:F3},{5:F7},{6:F7},{7:F1},{8:F1},{9},{10}", (Int32)P.Seq, (Int32)P.PassNo, (String)P.Action, (Double)P.X, (Double)P.Y, (Double)P.Lat, (Double)P.Lon, (Double)P.DepthMm, (Double)P.HeadingDeg, (String)P.Blade, (String)P.Instruction);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        private static String BuildPivotCsv
            (
            Pivot.PivotCsvData data
            )
        {
            // Pivot schedule is serialized from typed rows so callers can consume either table or object model.
            StringBuilder Sb = new StringBuilder();
            foreach (KeyValuePair<String, String> Pair in data.Metadata)
            {
                Sb.Append('#');
                Sb.Append(Pair.Key);
                Sb.Append('=');
                Sb.AppendLine(Pair.Value);
            }

            Sb.AppendLine("position,first_pass,last_pass,pass_count,bearing_deg,cardinal,move_deg,tip_lat,tip_lon,conflict_count");
            for (Int32 Iter = 0; Iter < data.Rows.Count; Iter += 1)
            {
                Pivot.PivotCsvRow Row = data.Rows[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3},{4:F2},{5},{6:F2},{7:F7},{8:F7},{9}", Row.Position, Row.FirstPass, Row.LastPass, Row.PassCount, Row.BearingDeg, Row.Cardinal, Row.MoveDeg, Row.TipLat, Row.TipLon, Row.ConflictCount);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        private static HaulBuilderOptions CloneOptions
            (
            HaulBuilderOptions options
            )
        {
            // Cloning ensures the result captures immutable run configuration.
            HaulBuilderOptions Copy = new HaulBuilderOptions
            {
                AgdPath = options.AgdPath,
                GridSizeFt = options.GridSizeFt,
                ToleranceMm = options.ToleranceMm,
                ScraperCapacityLcy = options.ScraperCapacityLcy,
                SwellFactor = options.SwellFactor,
                CutFillRatio = options.CutFillRatio,
                MaxDepthMm = options.MaxDepthMm,
                BladeWidthMm = options.BladeWidthMm,
                LaneAxisHeadingDeg = options.LaneAxisHeadingDeg,
                FieldEfficiency = options.FieldEfficiency,
                CutSpeedMph = options.CutSpeedMph,
                LoadedTravelSpeedMph = options.LoadedTravelSpeedMph,
                FillSpeedMph = options.FillSpeedMph,
                UnloadedTravelSpeedMph = options.UnloadedTravelSpeedMph,
                AccelerationTimeS = options.AccelerationTimeS,
                DecelerationTimeS = options.DecelerationTimeS,
                PixelsPerCell = options.PixelsPerCell,
                FlowArrowStrideCells = options.FlowArrowStrideCells,
                ZoneCount = options.ZoneCount,
                IncludeCutFillPairCsv = options.IncludeCutFillPairCsv,
                IncludeCutFillPairPng = options.IncludeCutFillPairPng,
                IncludeGuidancePng = options.IncludeGuidancePng,
                TurnSpeedMph = options.TurnSpeedMph,
                GradeSpeedMph = options.GradeSpeedMph,
                MinimumTurningDiameterFt = options.MinimumTurningDiameterFt,
                EquipmentLengthFt = options.EquipmentLengthFt,
                WaypointTurnThresholdDeg = options.WaypointTurnThresholdDeg,
                IncludeAnimationGif = options.IncludeAnimationGif,
                IncludeAnimationFrames = options.IncludeAnimationFrames,
                AnimationStride = options.AnimationStride,
                AnimationPixelsPerCell = options.AnimationPixelsPerCell,
                AnimationMode = options.AnimationMode,
                EnablePivot = options.EnablePivot,
                PivotCenter = options.PivotCenter,
                PivotWheelbaseFt = options.PivotWheelbaseFt,
                PivotClearanceFt = options.PivotClearanceFt,
                PivotMoveCost = options.PivotMoveCost,
                PivotMode = options.PivotMode,
                PivotPixelsPerCell = options.PivotPixelsPerCell,
                IncludePivotAnimationGif = options.IncludePivotAnimationGif,
                IncludePivotAnimationFrames = options.IncludePivotAnimationFrames,
                PivotAnimationStride = options.PivotAnimationStride,
                EquipmentWidthFt = options.EquipmentWidthFt,
                SkipHaulPlan = options.SkipHaulPlan,
                RandomRouteImageCount = options.RandomRouteImageCount,
                RandomSeed = options.RandomSeed,
            };
            CopyOptimization(options.Optimization, Copy.Optimization);
            Copy.ExcludePolygonSpecs.AddRange(options.ExcludePolygonSpecs);
            return Copy;
        }

        private static void CopyOptimization(
            HaulOptimizationOptions source,
            HaulOptimizationOptions destination)
        {
            destination.MaxLaneSlicesPerCycle = source.MaxLaneSlicesPerCycle;
            destination.FlowPenaltyWeight = source.FlowPenaltyWeight;
            destination.EdgeMaxLaneSlicesPerCycle = source.EdgeMaxLaneSlicesPerCycle;
            destination.EnableRouteConsolidation = source.EnableRouteConsolidation;
            destination.RouteConsolidationGroupSize = source.RouteConsolidationGroupSize;
            destination.RouteConsolidationMaximumCycles = source.RouteConsolidationMaximumCycles;
            destination.PairRefinementRounds = source.PairRefinementRounds;
            destination.EnableMultiRouteExchange = source.EnableMultiRouteExchange;
            destination.MultiRouteExchangeRounds = source.MultiRouteExchangeRounds;
            destination.MultiRouteExchangeGroupSize = source.MultiRouteExchangeGroupSize;
            destination.MultiRouteExchangeMaximumGroups = source.MultiRouteExchangeMaximumGroups;
            destination.MultiRouteExchangeMinimumImprovementM = source.MultiRouteExchangeMinimumImprovementM;
            destination.EnableCycleChainOptimization = source.EnableCycleChainOptimization;
            destination.CycleChainRounds = source.CycleChainRounds;
            destination.CycleChainCandidatesPerRoute = source.CycleChainCandidatesPerRoute;
            destination.CycleChainMinimumImprovementSeconds = source.CycleChainMinimumImprovementSeconds;
            destination.RouteReorderWindow = source.RouteReorderWindow;
        }

        private static void ValidateOptions
            (
            HaulBuilderOptions options
            )
        {
            static void Positive(Double value, String name)
            {
                if (!Double.IsFinite(value) || (value <= 0.0))
                    throw new ArgumentOutOfRangeException(
                        name, "Value must be finite and greater than zero.");
            }

            Positive(options.GridSizeFt, nameof(options.GridSizeFt));
            if (!Double.IsFinite(options.ToleranceMm)
                || (options.ToleranceMm < 0.0))
                throw new ArgumentOutOfRangeException(
                    nameof(options.ToleranceMm));
            Positive(options.ScraperCapacityLcy,
                nameof(options.ScraperCapacityLcy));
            Positive(options.SwellFactor, nameof(options.SwellFactor));
            Positive(options.CutFillRatio, nameof(options.CutFillRatio));
            Positive(options.MaxDepthMm, nameof(options.MaxDepthMm));
            Positive(options.BladeWidthMm, nameof(options.BladeWidthMm));
            if (!Double.IsFinite(options.LaneAxisHeadingDeg))
                throw new ArgumentOutOfRangeException(
                    nameof(options.LaneAxisHeadingDeg));
            if (!Double.IsFinite(options.FieldEfficiency)
                || (options.FieldEfficiency <= 0.0)
                || (options.FieldEfficiency > 1.0))
                throw new ArgumentOutOfRangeException(
                    nameof(options.FieldEfficiency));
            Positive(options.CutSpeedMph, nameof(options.CutSpeedMph));
            Positive(options.LoadedTravelSpeedMph,
                nameof(options.LoadedTravelSpeedMph));
            Positive(options.FillSpeedMph, nameof(options.FillSpeedMph));
            Positive(options.UnloadedTravelSpeedMph,
                nameof(options.UnloadedTravelSpeedMph));
            Positive(options.TurnSpeedMph, nameof(options.TurnSpeedMph));
            Positive(options.GradeSpeedMph, nameof(options.GradeSpeedMph));
            Positive(options.AccelerationTimeS,
                nameof(options.AccelerationTimeS));
            Positive(options.DecelerationTimeS,
                nameof(options.DecelerationTimeS));
            Positive(options.MinimumTurningDiameterFt,
                nameof(options.MinimumTurningDiameterFt));
            Positive(options.EquipmentLengthFt,
                nameof(options.EquipmentLengthFt));
            Positive(options.EquipmentWidthFt,
                nameof(options.EquipmentWidthFt));
            if (!Double.IsFinite(options.WaypointTurnThresholdDeg)
                || (options.WaypointTurnThresholdDeg < 0.0)
                || (options.WaypointTurnThresholdDeg > 180.0))
                throw new ArgumentOutOfRangeException(
                    nameof(options.WaypointTurnThresholdDeg));
            if (options.PixelsPerCell <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(options.PixelsPerCell));
            if (options.FlowArrowStrideCells < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(options.FlowArrowStrideCells));
            if (options.ZoneCount < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(options.ZoneCount));
            if (options.RandomRouteImageCount < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(options.RandomRouteImageCount));
            if (options.AnimationStride <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(options.AnimationStride));
            if (options.AnimationPixelsPerCell <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(options.AnimationPixelsPerCell));
            if (!String.Equals(options.AnimationMode, "cutfill",
                    StringComparison.OrdinalIgnoreCase)
                && !String.Equals(options.AnimationMode, "topology",
                    StringComparison.OrdinalIgnoreCase))
                throw new ArgumentException(
                    "AnimationMode must be 'cutfill' or 'topology'.",
                    nameof(options.AnimationMode));
            if (options.EnablePivot)
            {
                Positive(options.PivotWheelbaseFt,
                    nameof(options.PivotWheelbaseFt));
                if (!Double.IsFinite(options.PivotClearanceFt)
                    || (options.PivotClearanceFt < 0.0))
                    throw new ArgumentOutOfRangeException(
                        nameof(options.PivotClearanceFt));
                if (!Double.IsFinite(options.PivotMoveCost)
                    || (options.PivotMoveCost < 0.0))
                    throw new ArgumentOutOfRangeException(
                        nameof(options.PivotMoveCost));
                if (options.PivotPixelsPerCell <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(options.PivotPixelsPerCell));
                if (options.PivotAnimationStride <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(options.PivotAnimationStride));
                if (!String.Equals(options.PivotMode, "cutfill",
                        StringComparison.OrdinalIgnoreCase)
                    && !String.Equals(options.PivotMode, "topology",
                        StringComparison.OrdinalIgnoreCase))
                    throw new ArgumentException(
                        "PivotMode must be 'cutfill' or 'topology'.",
                        nameof(options.PivotMode));
            }

            ValidateOptimizationOptions(options);
        }

        private static void ValidateOptimizationOptions
            (
            HaulBuilderOptions options
            )
        {
            HaulOptimizationOptions optimization = options.Optimization;
            if (optimization.MaxLaneSlicesPerCycle <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(optimization.MaxLaneSlicesPerCycle));
            if (optimization.EdgeMaxLaneSlicesPerCycle <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(optimization.EdgeMaxLaneSlicesPerCycle));
            if (!Double.IsFinite(optimization.FlowPenaltyWeight)
                || (optimization.FlowPenaltyWeight < 0.0))
                throw new ArgumentOutOfRangeException(
                    nameof(optimization.FlowPenaltyWeight));
            if (optimization.PairRefinementRounds < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(optimization.PairRefinementRounds));
            if (optimization.RouteReorderWindow < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(optimization.RouteReorderWindow));
            if (optimization.EnableRouteConsolidation
                && (optimization.RouteConsolidationGroupSize < 3))
                throw new ArgumentOutOfRangeException(
                    nameof(optimization.RouteConsolidationGroupSize));
            if (optimization.EnableRouteConsolidation
                && (optimization.RouteConsolidationMaximumCycles <= 0))
                throw new ArgumentOutOfRangeException(
                    nameof(optimization.RouteConsolidationMaximumCycles));
            if (optimization.EnableMultiRouteExchange)
            {
                if (optimization.MultiRouteExchangeRounds <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(optimization.MultiRouteExchangeRounds));
                if (optimization.MultiRouteExchangeGroupSize < 3)
                    throw new ArgumentOutOfRangeException(
                        nameof(optimization.MultiRouteExchangeGroupSize));
                if (optimization.MultiRouteExchangeMaximumGroups <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(optimization.MultiRouteExchangeMaximumGroups));
                if (!Double.IsFinite(
                        optimization.MultiRouteExchangeMinimumImprovementM)
                    || (optimization.MultiRouteExchangeMinimumImprovementM < 0.0))
                    throw new ArgumentOutOfRangeException(
                        nameof(optimization.MultiRouteExchangeMinimumImprovementM));
            }
            if (optimization.EnableCycleChainOptimization)
            {
                if (optimization.CycleChainRounds <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(optimization.CycleChainRounds));
                if (optimization.CycleChainCandidatesPerRoute <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(optimization.CycleChainCandidatesPerRoute));
                if (!Double.IsFinite(
                        optimization.CycleChainMinimumImprovementSeconds)
                    || (optimization.CycleChainMinimumImprovementSeconds < 0.0))
                    throw new ArgumentOutOfRangeException(
                        nameof(optimization.CycleChainMinimumImprovementSeconds));
            }
        }

        /// <summary>Retains stable reasons for work deferred before compact preflight.</summary>
        private static IReadOnlyList<HaulDeferredWork>
            PreserveExistingDeferredReasons
            (
            IReadOnlyList<HaulDeferredWork> Current,
            IReadOnlyList<HaulDeferredWork> Existing
            )
        {
            Dictionary<(
                HaulLaneBuilder.WorkKind Kind,
                (Int32 Bx, Int32 By) Cell,
                Int32 LayerIndex), String> Reasons = Existing.ToDictionary(
                    Entry => (Entry.Kind, Entry.Cell, Entry.LayerIndex),
                    Entry => Entry.Reason);
            return Current.Select(Entry => Reasons.TryGetValue(
                    (Entry.Kind, Entry.Cell, Entry.LayerIndex),
                    out String? Reason)
                ? new HaulDeferredWork(
                    Entry.Kind,
                    Entry.Cell,
                    Entry.LayerIndex,
                    Entry.DepthM,
                    Entry.VolumeBcy,
                    Reason)
                : Entry).ToList();
        }
    }
}
