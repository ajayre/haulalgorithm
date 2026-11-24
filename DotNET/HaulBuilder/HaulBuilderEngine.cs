using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading;

namespace HaulBuilder
{
    /// <summary>
    /// Production end-to-end in-memory haul-planning pipeline.
    /// </summary>
    public sealed class HaulBuilderEngine
    {
        private const Double M2_PER_ACRE = 4046.8564224;
        private const Double METERS_PER_YARD = 0.9144;
        private const Double METERS_PER_MILE = Constants.METERS_PER_MILE;

        /// <summary>
        /// Executes a full haul-planning run and returns typed in-memory results.
        /// </summary>
        /// <param name="Options">Validated production planning options.</param>
        /// <param name="GuidanceProgress">Optional guidance-build progress reporter.</param>
        /// <param name="CancellationToken">Cancellation token observed during guidance construction.</param>
        /// <returns>Structured pipeline outputs and render payloads.</returns>
        /// <exception cref="ArgumentNullException">Thrown when options is null.</exception>
        /// <exception cref="ArgumentException">Thrown when required options are invalid.</exception>
        /// <exception cref="OperationCanceledException">Thrown when guidance construction is cancelled.</exception>
        public HaulBuilderResult Run
            (
            HaulBuilderOptions Options,
            IProgress<GuidanceBuildProgress>? GuidanceProgress = null,
            CancellationToken CancellationToken = default
            )
        {
            // Options are copied so later caller mutations cannot alter an already produced result.
            if (Options is null) throw new ArgumentNullException(nameof(Options));
            if (String.IsNullOrWhiteSpace(Options.AgdPath)) throw new ArgumentException("AGD path is required.", nameof(Options));
            if (!File.Exists(Options.AgdPath)) throw new ArgumentException($"AGD file not found: {Options.AgdPath}", nameof(Options));
            ValidateOptions(Options);

            HaulBuilderOptions Opt = CloneOptions(Options);
            HaulBuilderResult Result = new HaulBuilderResult
            {
                Options = Opt,
            };

            Interlocked.Increment(ref _ActiveRunCount);
            try
            {
                // Ambient minimum length aligns compact preflight with guidance.
                using (CoveragePlanner.PushMinimumWorkLength(
                    Opt.ResolveMinimumWorkLengthM()))
                {
                    return RunCore(
                        Opt,
                        Result,
                        GuidanceProgress,
                        CancellationToken);
                }
            }
            finally
            {
                // The connector caches are keyed by grid objects that this run
                // created, so no other run can use an entry. Release them when the
                // last active run ends; a run still in flight only loses memoized
                // results, which it recomputes with the same values.
                if (Interlocked.Decrement(ref _ActiveRunCount) == 0)
                {
                    TransitPlanner.ReleaseCaches();
                }
            }
        }

        /// <summary>Number of <see cref="Run"/> calls executing in this process.</summary>
        static private Int32 _ActiveRunCount;

        /// <summary>
        /// Factor that turns a ratio into a percentage, for warning text.
        /// </summary>
        private const Double PERCENT_PER_UNIT = 100.0;

        /// <summary>
        /// Executes the validated haul-planning pipeline under an ambient work-length scope.
        /// </summary>
        /// <param name="Opt">Cloned production options.</param>
        /// <param name="Result">Result object being populated.</param>
        /// <param name="GuidanceProgress">Optional guidance progress reporter.</param>
        /// <param name="CancellationToken">Cancellation token for guidance.</param>
        /// <returns>Completed pipeline result.</returns>
        private HaulBuilderResult RunCore
            (
            HaulBuilderOptions Opt,
            HaulBuilderResult Result,
            IProgress<GuidanceBuildProgress>? GuidanceProgress,
            CancellationToken CancellationToken
            )
        {
            AgdField Field = Agd.Parse(Opt.AgdPath);
            // Intent: the parser writes this warning to the console only, where a
            // caller with no console never sees it.
            if (Field.Skipped > 0)
            {
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "WARNING: Skipped {0} unusable AGD rows (missing/zero/short).",
                    Field.Skipped));
            }
            Grid Grid = GridMath.BuildGrid(Field, Opt.GridSizeFt, Opt.SwellFactor, Opt.CutFillRatio, Opt.MaxDepthMm);
            Result.Field = Field;
            Result.Grid = Grid;

            ObstacleField Obstacles = ResolveExclusions(Result, Field, Grid, Opt);
            Double BalanceThreshold = Obstacles.Active ? Double.PositiveInfinity : Balance.DEFAULT_IMBALANCE_THRESHOLD;
            BalanceResult BalanceResult = Balance.Compute(Grid, "check", BalanceThreshold);
            Result.Balance = BalanceResult;
            // Intent: same test and same wording as the console warning in
            // Balance.Compute, so both report the same imbalance.
            if (BalanceResult.RelativeImbalance > BalanceThreshold)
            {
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "WARNING: cut/fill imbalance {0:0.0}% exceeds {1:0.0}% (supply={2:0.0} BCY, demand={3:0.0} BCY)",
                    BalanceResult.RelativeImbalance * PERCENT_PER_UNIT,
                    BalanceThreshold * PERCENT_PER_UNIT,
                    BalanceResult.TotalSupplyBcy,
                    BalanceResult.TotalDemandBcy));
            }

            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            Result.Flow = Flow;
            Result.PngBytesByName["main"] = Render.RenderPng(
                Grid, Flow, Opt.PixelsPerCell, Opt.FlowArrowStrideCells);

            if (Opt.SkipHaulPlan)
            {
                Result.Messages.Add("Planning stages skipped because SkipHaulPlan was set.");
                return Result;
            }

            // The ribbon takes each heading from the flow. The reported axis stays north-south.
            Double HeadingDeg = 0.0;
            Double ScraperBcy = (Opt.ScraperCapacityLcy / Opt.SwellFactor);
            // Captured before consolidation can move boundary cells into other loads.
            HashSet<(Int32 Bx, Int32 By)> EdgeCutCells =
                new HashSet<(Int32 Bx, Int32 By)>();
            HashSet<(Int32 Bx, Int32 By)> EdgeFillCells =
                new HashSet<(Int32 Bx, Int32 By)>();

            PassResult PassResult;
            {
                HaulWorkLedger Ledger = HaulWorkLedger.Build(
                    Grid, Opt.ToleranceMm);
                HaulOptimizationResult Optimized =
                    FlowRibbonSearch.Optimize(
                        Grid,
                        Flow,
                        Ledger,
                        HaulLaneBuilder.LaneAxis.UserHeading,
                        ScraperBcy,
                        true,
                        Obstacles,
                        false,
                        Opt.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                        Opt.MinimumTurningDiameterFt,
                        Opt.Optimization.ClusterModeAverageWindow,
                        Opt.Optimization.ClusterModeLoadFraction,
                        Clock: new TimeParams(
                            Opt.CutSpeedMph,
                            Opt.LoadedTravelSpeedMph,
                            Opt.FillSpeedMph,
                            Opt.UnloadedTravelSpeedMph,
                            Opt.AccelerationTimeS,
                            Opt.DecelerationTimeS,
                            Opt.FieldEfficiency));
                List<Pass> OptimizedPasses = Optimized.PassResult.Passes;
                HaulPairDistanceRefiner.Result? PairRefinement = null;
                // A ribbon already hauls along the flow. Swapping its fill for a
                // nearer centroid would send the soil off that streamline.
                Boolean FlowRibbons = OptimizedPasses.Exists(Pass => Pass.RibbonOrder);
                if (!FlowRibbons && (Opt.Optimization.PairRefinementRounds > 0))
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
                // The search has left hauling. The residual flow pass would
                // scrape the dirt clusters or correcting leave.
                if (!Optimized.EnteredClusterMode)
                {
                EdgeCompletion = HaulResidualEdgeCompleter.Complete(
                        OptimizedPasses,
                        Grid,
                        Flow,
                        Ledger,
                        HaulLaneBuilder.LaneAxis.UserHeading,
                        ScraperBcy,
                        Opt.Optimization.PairRefinementRounds,
                        HaulPairDistanceRefiner.PairMetric.Centroid,
                        Opt.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                        GradeReturn: true,
                        Obstacles,
                        Opt.MinimumTurningDiameterFt);
                foreach (Pass EdgePass in EdgeCompletion.Passes)
                {
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                        in EdgePass.CutCells)
                    {
                        EdgeCutCells.Add(Entry.Cell);
                    }
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                        in EdgePass.FillCells)
                    {
                        EdgeFillCells.Add(Entry.Cell);
                    }
                }
                OptimizedPasses = OptimizedPasses
                    .Concat(EdgeCompletion.Passes)
                    .ToList();
                }
                HaulMultiRouteExchangeOptimizer.Result? MultiRouteExchange = null;
                HaulRouteConsolidationOptimizer.Result? RouteConsolidation = null;
                // Consolidation rebuilds passes as unordered cell groups. That
                // throws away the ribbon and the guidance falls back to lanes.
                Boolean FlowRibbonsForConsolidation =
                    OptimizedPasses.Exists(Pass => Pass.RibbonOrder);
                if (Opt.Optimization.EnableRouteConsolidation
                    && !FlowRibbonsForConsolidation)
                {
                    RouteConsolidation =
                        HaulRouteConsolidationOptimizer.Optimize(
                            OptimizedPasses,
                            Grid,
                            ScraperBcy,
                            Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER,
                            Opt.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
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
                        Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER,
                        Opt.Optimization.MultiRouteExchangeRounds,
                        Opt.Optimization.MultiRouteExchangeGroupSize,
                        Opt.Optimization.MultiRouteExchangeMaximumGroups,
                        MinimumImprovementM:
                            Opt.Optimization.MultiRouteExchangeMinimumImprovementM,
                        BladeWidthM: Opt.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                        GradeReturn: true,
                        Obstacles: Obstacles);
                    OptimizedPasses = MultiRouteExchange.Passes;
                    Result.HaulMultiRouteExchange = MultiRouteExchange;
                }
                if ((PairRefinement != null)
                    || (EdgeCompletion != null)
                    || (RouteConsolidation != null)
                    || (MultiRouteExchange != null))
                {
                    IReadOnlyList<HaulDeferredWork> Deferred =
                        PreserveExistingDeferredReasons(
                            Ledger.BuildDeferredWork(
                                OptimizedPasses,
                                "OPTIMIZER_UNASSIGNED"),
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
                        Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER,
                        AllowDeferredWork: Deferred.Count > 0);
                    HaulOptimizationResult PreviousOptimized = Optimized;
                    Optimized = new HaulOptimizationResult(
                        new PassResult(
                            OptimizedPasses,
                            PreviousOptimized.PassResult.Rounds,
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
                        0,
                        0,
                        EdgeCompletion?.Passes.Count ?? 0,
                        EdgeCompletion?.OmittedCycles ?? 0);
                    Optimized.CopyClusterModeFrom(PreviousOptimized);
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

            Int32 ActiveReorderWindow = Opt.Optimization.RouteReorderWindow;
            Route Route;
            if (Opt.Optimization.EnableEarlyFullStripBanding)
            {
                HaulRouteBanding.Result Banded = BandProductionRoute(
                    PassResult.Passes,
                    EdgeCutCells,
                    EdgeFillCells,
                    Opt,
                    Obstacles);
                Route = Banded.Route;
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Early full-strip banding ordered {0} one-strip interior route(s), then {1} other interior route(s), then {2} field-edge route(s), then {3} scrap route(s).",
                    Banded.FullStripCount,
                    Banded.OtherInteriorCount,
                    Banded.EdgeCount,
                    Banded.ScrapCount));
            }
            else
            {
                Route = Routing.SequencePasses(
                    PassResult.Passes,
                    1500,
                    GradeReturn: true,
                    DirectionAware: true,
                    Opt.UnloadedTravelSpeedMph,
                    Opt.TurnSpeedMph,
                    Opt.MinimumTurningDiameterFt,
                    Obstacles,
                    StartCandidateCount: 8,
                    OrderByDistance: Opt.Optimization.OptimizeRouteOrderByDistance);
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
                        Obstacles,
                        OrderByDistance: Opt.Optimization.OptimizeRouteOrderByDistance);
                }
            }
            Result.Messages.Add(Opt.Optimization.OptimizeRouteOrderByDistance
                ? "Route order scored by distance (empty hop plus turn arc)."
                : "Route order scored by transition time.");
            Result.Route = Route;

            GuidancePlanner.BuildResult? PrecomputedGuidancePlan = null;
            // Exact chain promotion requires full guidance; skip when callers want a fast preview.
            if (Opt.Optimization.EnableCycleChainOptimization
                && !Opt.SkipGuidance)
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
                        Opt.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
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
                Route CandidateRoute;
                HaulRouteBanding.Result? BandedCandidate = null;
                if (Opt.Optimization.EnableEarlyFullStripBanding)
                {
                    // Fill exchanges can change which load is one strip. Band again
                    // so the window still cannot pull an edge load forward.
                    BandedCandidate = BandProductionRoute(
                        Chain.Passes,
                        EdgeCutCells,
                        EdgeFillCells,
                        Opt,
                        Obstacles);
                    CandidateRoute = BandedCandidate.Route;
                }
                else
                {
                    CandidateRoute = new Route(
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
                            Obstacles,
                            OrderByDistance: Opt.Optimization.OptimizeRouteOrderByDistance);
                    }
                }
                Result.HaulCycleChainOptimization = Chain;
                if (Chain.AcceptedSwaps > 0)
                {
                    GuidanceCostOracle.Result BaselineExact =
                        GuidanceCostOracle.EvaluatePlan(
                            BaselineRoute, Grid, Field, HeadingDeg, Opt,
                            CancellationToken: CancellationToken);
                    GuidanceCostOracle.Result CandidateExact =
                        GuidanceCostOracle.EvaluatePlan(
                            CandidateRoute, Grid, Field, HeadingDeg, Opt,
                            CancellationToken: CancellationToken);
                    Boolean Promote = CandidateExact.Success
                        && (!BaselineExact.Success
                            || (CandidateExact.DeterministicSeconds
                                < BaselineExact.DeterministicSeconds - Constants.GEOMETRY_COMPARISON_TOLERANCE));
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
                        if (BandedCandidate != null)
                        {
                            Result.Messages.Add(String.Format(
                                CultureInfo.InvariantCulture,
                                "Early full-strip banding reapplied after chain refinement: {0} one-strip interior route(s), then {1} other interior route(s), then {2} field-edge route(s), then {3} scrap route(s).",
                                BandedCandidate.FullStripCount,
                                BandedCandidate.OtherInteriorCount,
                                BandedCandidate.EdgeCount,
                                BandedCandidate.ScrapCount));
                        }
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

            if (Opt.Ponding.Audit || Opt.Ponding.Optimize)
            {
                Result.Ponding = Ponding.Run(
                    Grid,
                    PassResult.Passes,
                    Route,
                    Opt.Ponding,
                    Opt.UnloadedTravelSpeedMph,
                    Opt.TurnSpeedMph,
                    Opt.MinimumTurningDiameterFt,
                    Obstacles,
                    ActiveReorderWindow > 0 ? ActiveReorderWindow : 50);
                PondingRunResult PondingResult = Result.Ponding;
                PondingAuditResult? FinalAudit = PondingResult.FinalAudit;
                if (FinalAudit != null && PondingResult.Outlets != null)
                {
                    Result.CsvTextByName["ponding"] = Ponding.BuildCsv(
                        FinalAudit, PondingResult.Outlets.Count);
                    if (PondingResult.Before != null
                        && PondingResult.After != null
                        && !ReferenceEquals(PondingResult.Before, PondingResult.After))
                        Result.CsvTextByName["ponding_before"] = Ponding.BuildCsv(
                            PondingResult.Before, PondingResult.Outlets.Count);
                    if (Opt.Ponding.IncludeMap)
                        Result.PngBytesByName["ponding"] =
                            Ponding.RenderWorstCheckpointPng(
                                Grid, PondingResult.Outlets,
                                FinalAudit.WorstCheckpoint,
                                Opt.Ponding.MapPixelsPerCell);
                    Result.Messages.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Ponding summary ({0}): penalty={1:F6}->{2:F6}, peak={3:F2} BCY at pass {4}, mean_avg_depth={5:F6} m, max_depth={6:F3} m, outlets={7}, T={8:F1}->{9:F1} s, relocations={10}, analyser_failures={11}, weight={12:F1}.",
                        Opt.Ponding.ObjectiveMode,
                        PondingResult.Before?.Penalty ?? FinalAudit.Penalty,
                        FinalAudit.Penalty,
                        FinalAudit.PeakVolumeBcy,
                        FinalAudit.WorstCheckpoint.PassIndex,
                        FinalAudit.MeanAveragePondDepthM,
                        FinalAudit.WorstCheckpoint.Metrics.MaxPondDepthM,
                        PondingResult.Outlets.Count,
                        PondingResult.TransitionCostBeforeS,
                        PondingResult.TransitionCostAfterS,
                        PondingResult.RelocationsAccepted,
                        FinalAudit.AnalyserFailures,
                        Opt.Ponding.Weight));
                }
                Result.Messages.AddRange(PondingResult.Warnings);
                if (Opt.Ponding.Optimize)
                    PrecomputedGuidancePlan = null;
            }

            BuildZoneSequence(Result, Grid, Field, Route, Opt);
            BuildPairs(Result, Grid, Field, Route, Opt);
            // Pivot positions must exist before guidance so the final lifecycle can
            // route around the held arm for each executed haul route.
            BuildPivot(Result, Grid, Route, Obstacles, Opt);
            if (Opt.SkipGuidance)
            {
                Result.Messages.Add(
                    "Exact production guidance skipped because SkipGuidance was set; "
                    + "metrics use TimeModel estimates until guidance is built.");
            }
            else
            {
                BuildGuidance(
                    Result,
                    Grid,
                    Field,
                    Route,
                    Obstacles,
                    HeadingDeg,
                    Opt,
                    PrecomputedGuidancePlan,
                    GuidanceProgress,
                    CancellationToken);
            }

            BuildTimingAndMetrics(Result, Grid, PassResult, Route, HeadingDeg, Opt);
            BuildAnimations(Result, Grid, Route, Opt);
            // Found order, not the drive tour. The routes are where hauling
            // handed off to clusters, where clusters handed off to thin-cut,
            // and where thin-cut handed off to correcting.
            if (Result.HaulOptimization != null)
            {
                Result.State = Result.HaulOptimization.State;
                Result.ClusterModeStartsAtRoute = Result.HaulOptimization.ClusterModeStartsAtRoute;
                Result.ThinCutStartsAtRoute = Result.HaulOptimization.ThinCutStartsAtRoute;
                Result.CorrectingStartsAtRoute = Result.HaulOptimization.CorrectingStartsAtRoute;
                Result.CeilingCutLeftBcy = Result.HaulOptimization.CeilingCutLeftBcy;
                Result.CeilingFillLeftBcy = Result.HaulOptimization.CeilingFillLeftBcy;
                Result.StoppedOnCeiling = Result.HaulOptimization.StoppedOnCeiling;
            }

            return Result;
        }

        /// <summary>Ensures guidance progress percentages never move backward.</summary>
        private sealed class MonotonicGuidanceProgress
        {
            private readonly IProgress<GuidanceBuildProgress> Target;
            private readonly Object Gate = new Object();
            private Double LastPercent;

            /// <summary>Initializes one monotonic guidance progress adapter.</summary>
            /// <param name="Target">Underlying progress sink.</param>
            public MonotonicGuidanceProgress
                (
                IProgress<GuidanceBuildProgress> Target
                )
            {
                this.Target = Target;
            }

            /// <summary>Reports one planner phase when it advances overall progress.</summary>
            /// <param name="Phase">Planner phase label.</param>
            /// <param name="Completed">Completed units in the active phase.</param>
            /// <param name="Total">Total units in the active phase.</param>
            public void Report
                (
                String Phase,
                Int32 Completed,
                Int32 Total
                )
            {
                GuidanceBuildProgress Snapshot =
                    GuidanceBuildProgress.FromPlannerPhase(
                        Phase, Completed, Total);
                lock (Gate)
                {
                    if (Snapshot.PercentComplete < LastPercent)
                    {
                        return;
                    }
                    LastPercent = Snapshot.PercentComplete;
                }
                Target.Report(Snapshot);
            }
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
            Result.HaulOptimization.CopyClusterModeFrom(Previous);
        }

        /// <summary>
        /// Resolves exclusions.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Options">Configuration controlling the operation.</param>
        /// <returns>The resulting exclusions.</returns>

        private static ObstacleField ResolveExclusions
            (
            HaulBuilderResult Result,
            AgdField Field,
            Grid Grid,
            HaulBuilderOptions Options
            )
        {
            // Exclusion polygon parsing mirrors CLI behavior and enables both planning and routing detours.
            if (Options.ExcludePolygonSpecs.Count <= 0)
            {
                return HaulBuilder.Obstacles.EMPTY_OBSTACLES;
            }

            List<List<(Double Lat, Double Lon)>> Rings = ParsePolygonSpecs(Options.ExcludePolygonSpecs);
            List<List<(Double X, Double Y)>> RingsXy = new List<List<(Double X, Double Y)>>();
            foreach (List<(Double Lat, Double Lon)> Ring in Rings)
            {
                RingsXy.Add(Agd.ProjectLatlonRing(Ring));
            }

            GridMath.ApplyExclusions(Grid, RingsXy);
            Double ClearanceM = ((Options.EquipmentWidthFt * Constants.FEET_TO_METERS) / 2.0);
            ObstacleField ObstacleFieldResult = ObstacleField.FromRings(Grid.Exclusions, ClearanceM);
            Result.Messages.Add(String.Format(CultureInfo.InvariantCulture, "Applied {0} exclusion polygon(s).", Rings.Count));
            return ObstacleFieldResult;
        }

        /// <summary>
        /// Builds zone sequence.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <param name="Route">The route used by the operation.</param>
        /// <param name="Options">Configuration controlling the operation.</param>

        private static void BuildZoneSequence
            (
            HaulBuilderResult Result,
            Grid Grid,
            AgdField Field,
            Route Route,
            HaulBuilderOptions Options
            )
        {
            // Zone sequencing condenses hundreds of passes into a coarse operational work order.
            if (Options.ZoneCount <= 0)
            {
                return;
            }

            List<ZoneStep> Steps = Sequence.SequenceCutZones(
                Grid, Field, Route, Options.ZoneCount, Options.RandomSeed);
            Result.ZoneSteps.AddRange(Steps);
            if (Steps.Count > 0)
            {
                String Csv = BuildZoneCsv(Steps);
                Result.CsvTextByName["zone_sequence"] = Csv;
                Byte[] Overlay = Render.OverlayZoneWaypoints(
                    Result.PngBytesByName["main"], Grid, Steps,
                    Options.PixelsPerCell);
                Result.PngBytesByName["main_with_zones"] = Overlay;
            }
        }

        /// <summary>
        /// Builds pairs.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <param name="Route">The route used by the operation.</param>
        /// <param name="Options">Configuration controlling the operation.</param>

        private static void BuildPairs
            (
            HaulBuilderResult Result,
            Grid Grid,
            AgdField Field,
            Route Route,
            HaulBuilderOptions Options
            )
        {
            // Pair rows are built explicitly so the engine never relies on file-backed CSV helpers.
            TimeParams TimeParams = new TimeParams(
                Options.CutSpeedMph,
                Options.LoadedTravelSpeedMph,
                Options.FillSpeedMph,
                Options.UnloadedTravelSpeedMph,
                Options.AccelerationTimeS,
                Options.DecelerationTimeS,
                Options.FieldEfficiency);
            Double TotalCutBcyBefore = 0.0;
            Double TotalFillBcyBefore = 0.0;
            for (Int32 Iter = 0; Iter < Route.OrderedPasses.Count; Iter += 1)
            {
                Pass Pass = Route.OrderedPasses[Iter];
                Double RouteTimeS = TimeModel.EstimatePassTime(Grid, Pass, TimeParams);
                Result.CutFillPairs.Add(
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
                TotalCutBcyBefore += TimeModel.PassCutBcy(Grid, Pass);
                TotalFillBcyBefore += TimeModel.PassFillBcy(Grid, Pass);
            }

            if (Options.IncludeCutFillPairCsv)
            {
                Result.CsvTextByName["cut_fill_pairs"] = BuildPairsCsv(Result.CutFillPairs, Field);
            }

            if (Options.IncludeCutFillPairPng)
            {
                Result.PngBytesByName["cut_fill_pairs"] = Render.RenderPassPairsPng(
                    Grid,
                    Route.OrderedPasses,
                    Options.PixelsPerCell);
            }
        }

        /// <summary>
        /// Builds guidance.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <param name="Route">The route used by the operation.</param>
        /// <param name="Obstacles">The obstacles used by the operation.</param>
        /// <param name="HeadingDeg">Direction in degrees.</param>
        /// <param name="Options">Configuration controlling the operation.</param>
        /// <param name="PrecomputedPlan">The precomputed plan used by the operation.</param>
        /// <param name="GuidanceProgress">Optional guidance-build progress reporter.</param>
        /// <param name="CancellationToken">Cancellation token observed during guidance construction.</param>

        private static void BuildGuidance
            (
            HaulBuilderResult Result,
            Grid Grid,
            AgdField Field,
            Route Route,
            ObstacleField Obstacles,
            Double HeadingDeg,
            HaulBuilderOptions Options,
            GuidancePlanner.BuildResult? PrecomputedPlan = null,
            IProgress<GuidanceBuildProgress>? GuidanceProgress = null,
            CancellationToken CancellationToken = default
            )
        {
            // Exact production guidance is mandatory for every completed haul plan.
            MonotonicGuidanceProgress? MonotonicProgress =
                GuidanceProgress == null
                    ? null
                    : new MonotonicGuidanceProgress(GuidanceProgress);
            Action<String, Int32, Int32>? PlannerProgress = MonotonicProgress == null
                ? null
                : MonotonicProgress.Report;

            GuidancePlanner.BuildResult Plan = PrecomputedPlan
                ?? GuidancePlanner.Build(
                    Route,
                    Grid,
                    Field,
                    HeadingDeg,
                    Options,
                    Progress: PlannerProgress,
                    Diagnostic: Message => Result.Messages.Add(Message),
                    CancellationToken: CancellationToken);

            if (Obstacles.Active)
            {
                (Plan, GuidanceObstacleRouter.Result StaticRouting) =
                    GuidancePlanner.ApplyObstacleRouting(
                        Plan,
                        Route,
                        Grid,
                        Field,
                        HeadingDeg,
                        Options,
                        Obstacles);
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Guidance routed {0} blade-up move(s) around exclusion polygons; {1} high-risk move(s) require manual control; added distance {2:F1} m.",
                    StaticRouting.StaticDetourCount,
                    StaticRouting.ManualMoveCount,
                    StaticRouting.AddedDistanceM));
            }

            Dictionary<Int32, ObstacleField>? GuidanceArmObstacles =
                ToGuidanceObstacleFields(Result.ArmObstacles);
            if ((GuidanceArmObstacles != null)
                && (GuidanceArmObstacles.Count > 0))
            {
                Double BeforePivotS = Plan.Timing.TotalSeconds;
                Result.PivotExtraHaulM = CalculatePivotExtraHaulM(
                    Route, Obstacles, GuidanceArmObstacles);
                (Plan, GuidanceObstacleRouter.Result PivotRouting) =
                    GuidancePlanner.ApplyObstacleRouting(
                        Plan,
                        Route,
                        Grid,
                        Field,
                        HeadingDeg,
                        Options,
                        Obstacles,
                        GuidanceArmObstacles);
                Result.PivotAddedTimeS = Math.Max(
                    0.0, Plan.Timing.TotalSeconds - BeforePivotS);
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Guidance routed {0} blade-up move(s) around the parked pivot arm; {1} high-risk move(s) require manual control; added {2:F1} m and {3:F1} s.",
                    PivotRouting.PivotDetourCount,
                    PivotRouting.ManualMoveCount,
                    PivotRouting.AddedDistanceM,
                    Result.PivotAddedTimeS));
            }
            Plan = AppendDeferredCleanupRoutes(
                Result,
                Plan,
                Grid,
                Field,
                Route,
                HeadingDeg,
                Options,
                PlannerProgress,
                CancellationToken);
            GuidanceResult Guidance = Plan.Guidance;
            Result.Guidance = Guidance;
            Result.GuidancePoints.Clear();
            Result.GuidancePoints.AddRange(Guidance.Points);
            Result.LifecycleGuidance = Plan.Lifecycle;
            Result.DetailedGuidanceTime = Plan.Timing;

            // Callers that read GuidancePoints do not need the multi-hundred-megabyte CSV.
            if (Options.IncludeGuidanceCsv)
            {
                Result.CsvTextByName["guidance"] = BuildGuidanceCsv(Guidance.Points);
            }
            if (Options.IncludeGuidancePng)
            {
                Result.PngBytesByName["guidance"] = Render.RenderGuidancePng(
                    Grid, Guidance.Points, Options.PixelsPerCell,
                    Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER);
            }

            BuildPairSamples(Result, Grid, Guidance.Points, Options);
            BuildRouteGuidanceImages(Result, Grid, Options);
            BuildRouteTriplets(Result, Grid, Options);
        }

        /// <summary>
        /// Rebuilds guidance-deferred scraps into area-grouped haul routes appended
        /// at end of job, then records any leftovers that still cannot be paired.
        /// </summary>
        /// <param name="Result">Pipeline result to update.</param>
        /// <param name="Plan">Main-job guidance plan.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Field">AGD field model.</param>
        /// <param name="Route">Current routed pass order (mutated when cleanup is appended).</param>
        /// <param name="HeadingDeg">Lane axis heading.</param>
        /// <param name="Options">Production options.</param>
        /// <param name="PlannerProgress">Optional guidance progress sink.</param>
        /// <param name="CancellationToken">Cancellation token.</param>
        /// <returns>Guidance plan including any cleanup tail.</returns>
        private static GuidancePlanner.BuildResult AppendDeferredCleanupRoutes
            (
            HaulBuilderResult Result,
            GuidancePlanner.BuildResult Plan,
            Grid Grid,
            AgdField Field,
            Route Route,
            Double HeadingDeg,
            HaulBuilderOptions Options,
            Action<String, Int32, Int32>? PlannerProgress,
            CancellationToken CancellationToken
            )
        {
            FlowField Flow = Result.Flow
                ?? throw new InvalidOperationException(
                    "Deferred cleanup requires a computed flow field.");
            Double ScraperBcy = Options.ScraperCapacityLcy / Options.SwellFactor;
            // Intent: pack nearby deferred scraps into local area cycles at job end.
            HaulDeferredCleanupCompleter.Result Cleanup =
                HaulDeferredCleanupCompleter.Complete(
                    Plan.Passes,
                    Grid,
                    Flow,
                    ScraperBcy,
                    HaulBuilder.Obstacles.EMPTY_OBSTACLES,
                    Options.ToleranceMm,
                    Options.MinimumTurningDiameterFt);
            // Intent: typed values, so a caller does not parse message text. With
            // no cleanup build, the main build is also the last build.
            Result.GuidanceSequenceFailureCount =
                Plan.Passes.Count(PassPlan => PassPlan.DeferredEntirePass);
            Result.UnsequenceablePassCount = Result.GuidanceSequenceFailureCount;
            // Intent: depth that the main build neither drove nor deferred is
            // still owed, whichever branch follows.
            List<HaulDeferredWork> StillDeferred =
                new List<HaulDeferredWork>(Cleanup.RemainingDeferred);
            StillDeferred.AddRange(BuildUnrecordedDeferred(Grid, Plan.Passes));
            // Intent: cleanup labels every leftover as a sequence failure.
            // Cells the main build omitted only for length keep that reason.
            StillDeferred = RelabelUnpairedMinimumLength(Plan, StillDeferred);
            if (Cleanup.Passes.Count == 0)
            {
                // RemainingDeferred already holds the short lists, so appending
                // them again would count each cell twice.
                RecordGuidanceDeferredWork(
                    Result,
                    Grid,
                    Plan,
                    StillDeferred,
                    CleanupCycleCount: 0,
                    AppendPlanShortLists: false);
                return Plan;
            }

            Int32 HeadPassCount = Route.OrderedPasses.Count;
            List<Pass> StrippedMain = StripAllocatedCellsFromPasses(
                Route.OrderedPasses,
                Cleanup.CutCells,
                Cleanup.FillCells,
                Grid);
            Route CleanupRoute = Routing.SequencePasses(
                Cleanup.Passes,
                1500,
                GradeReturn: true,
                DirectionAware: true,
                Options.UnloadedTravelSpeedMph,
                Options.TurnSpeedMph,
                Options.MinimumTurningDiameterFt,
                HaulBuilder.Obstacles.EMPTY_OBSTACLES,
                StartCandidateCount: 8,
                OrderByDistance: Options.Optimization.OptimizeRouteOrderByDistance);
            List<Pass> CombinedPasses = StrippedMain
                .Concat(CleanupRoute.OrderedPasses)
                .ToList();
            Double LinkDeadheadM = 0.0;
            if ((StrippedMain.Count > 0) && (CleanupRoute.OrderedPasses.Count > 0))
            {
                Pass LastMain = StrippedMain[StrippedMain.Count - 1];
                Pass FirstCleanup = CleanupRoute.OrderedPasses[0];
                Double Dx = FirstCleanup.CutStartXy.X - LastMain.FillEndXy.X;
                Double Dy = FirstCleanup.CutStartXy.Y - LastMain.FillEndXy.Y;
                LinkDeadheadM = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            Route CombinedRoute = new Route(
                Enumerable.Range(0, CombinedPasses.Count).ToList(),
                CombinedPasses,
                Route.TotalDeadheadM + CleanupRoute.TotalDeadheadM + LinkDeadheadM);
            Result.Route = CombinedRoute;
            if (Result.PassResult != null)
            {
                Result.PassResult = new PassResult(
                    CombinedPasses,
                    Result.PassResult.Rounds,
                    Result.PassResult.UnmatchedCutBcy,
                    Result.PassResult.UnmatchedFillBcy);
            }

            // Intent: cleanup routes drive micro scraps, so disable the min-length gate.
            HaulBuilderOptions CleanupOptions = CloneOptions(Options);
            CleanupOptions.MinimumWorkLengthFt = 1e-6;
            GuidancePlanner.BuildResult CleanupPlan = GuidancePlanner.Build(
                CleanupRoute,
                Grid,
                Field,
                HeadingDeg,
                CleanupOptions,
                Progress: PlannerProgress,
                Diagnostic: Message => Result.Messages.Add(Message),
                CancellationToken: CancellationToken);
            GuidancePlanner.BuildResult Merged = GuidancePlanner.Concatenate(
                Plan,
                CleanupPlan,
                CombinedRoute,
                Grid,
                Field,
                HeadingDeg,
                Options,
                HeadPassCount);
            Result.Messages.Add(String.Format(
                CultureInfo.InvariantCulture,
                "Appended {0} end-of-job cleanup haul route(s) grouping deferred scraps by area ({1:F3} BCY).",
                CleanupRoute.OrderedPasses.Count,
                Cleanup.AssignedCutBcy));
            Result.CleanupRouteCount = CleanupRoute.OrderedPasses.Count;
            Result.CleanupRouteBcy = Cleanup.AssignedCutBcy;
            // Intent: the cleanup build is now the last build, so its count is
            // the number of passes that no route drives.
            Result.UnsequenceablePassCount =
                CleanupPlan.Passes.Count(PassPlan => PassPlan.DeferredEntirePass);
            StillDeferred.AddRange(BuildUnrecordedDeferred(Grid, CleanupPlan.Passes));
            RecordGuidanceDeferredWork(
                Result,
                Grid,
                CleanupPlan,
                StillDeferred,
                CleanupRoute.OrderedPasses.Count);
            return Merged;
        }

        /// <summary>
        /// Builds deferred entries for depth that one guidance build neither
        /// drove nor listed as deferred.
        /// </summary>
        /// <param name="Grid">Field grid for volume conversion.</param>
        /// <param name="Passes">Final pass plans of one guidance build.</param>
        /// <returns>One entry per unrecorded cell and side.</returns>
        static private List<HaulDeferredWork> BuildUnrecordedDeferred
            (
            Grid Grid,
            IReadOnlyList<LifecyclePlanner.PassPlan> Passes
            )
        {
            Double CutBcyPerDepthM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillBcyPerDepthM = CutBcyPerDepthM * Grid.Compaction;
            List<HaulDeferredWork> Output = new List<HaulDeferredWork>();
            foreach (LifecyclePlanner.PassPlan PassPlan in Passes)
            {
                AppendShortDeferred(
                    Output,
                    HaulLaneBuilder.WorkKind.Cut,
                    PassPlan.CutUnrecordedCells,
                    CutBcyPerDepthM,
                    HaulDeferredWorkReasons.GUIDANCE_COVERAGE_OMITTED);
                AppendShortDeferred(
                    Output,
                    HaulLaneBuilder.WorkKind.Fill,
                    PassPlan.FillUnrecordedCells,
                    FillBcyPerDepthM,
                    HaulDeferredWorkReasons.GUIDANCE_COVERAGE_OMITTED);
            }
            return Output;
        }

        /// <summary>
        /// Removes deferred cell depths from main-job passes so cleanup cycles
        /// uniquely own that material.
        /// </summary>
        /// <param name="Passes">Main-job passes.</param>
        /// <param name="CutCells">Cut depths to strip.</param>
        /// <param name="FillCells">Fill depths to strip.</param>
        /// <param name="Grid">Field grid for rebuilt endpoints.</param>
        /// <returns>Passes with deferred depths removed; empty cycles dropped.</returns>
        private static List<Pass> StripAllocatedCellsFromPasses
            (
            IReadOnlyList<Pass> Passes,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
            Grid Grid
            )
        {
            Dictionary<(Int32 Bx, Int32 By), Double> CutRemaining =
                AggregateDepthByCell(CutCells);
            Dictionary<(Int32 Bx, Int32 By), Double> FillRemaining =
                AggregateDepthByCell(FillCells);
            Double CutBcyPerDepthM = Grid.AreaM2 * Constants.M3_TO_CY;
            List<Pass> Output = new List<Pass>();
            foreach (Pass Pass in Passes)
            {
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> NextCut =
                    SubtractDepths(Pass.CutCells, CutRemaining);
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> NextFill =
                    SubtractDepths(Pass.FillCells, FillRemaining);
                if ((NextCut.Count == 0) || (NextFill.Count == 0))
                {
                    continue;
                }
                Double LoadBcy = NextCut.Sum(Entry => Entry.DepthM * CutBcyPerDepthM);
                Pass Rebuilt = HaulBuilder.Passes.BuildPass(
                    Grid,
                    NextCut,
                    NextFill,
                    LoadBcy,
                    HaulBuilder.Obstacles.EMPTY_OBSTACLES,
                    Ordered: true);
                Rebuilt.CutHeadingDeg = Pass.CutHeadingDeg;
                Rebuilt.FillHeadingDeg = Pass.FillHeadingDeg;
                Output.Add(Rebuilt);
            }
            return Output;
        }

        /// <summary>Aggregates cell depths for destructive subtraction.</summary>
        /// <param name="Cells">Cell/depth entries.</param>
        /// <returns>Mutable remaining depth by cell.</returns>
        private static Dictionary<(Int32 Bx, Int32 By), Double> AggregateDepthByCell
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            Dictionary<(Int32 Bx, Int32 By), Double> Output =
                new Dictionary<(Int32 Bx, Int32 By), Double>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                if (Entry.DepthM <= 0.0)
                {
                    continue;
                }
                Output.TryGetValue(Entry.Cell, out Double Existing);
                Output[Entry.Cell] = Existing + Entry.DepthM;
            }
            return Output;
        }

        /// <summary>Subtracts remaining depths from a pass cell list.</summary>
        /// <param name="Cells">Original pass cells.</param>
        /// <param name="Remaining">Depth still to strip, mutated in place.</param>
        /// <returns>Cells with stripped depths removed or reduced.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> SubtractDepths
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Dictionary<(Int32 Bx, Int32 By), Double> Remaining
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Output =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                if (!Remaining.TryGetValue(Entry.Cell, out Double RemoveM)
                    || (RemoveM <= Constants.NUMERIC_COMPARISON_TOLERANCE))
                {
                    Output.Add(Entry);
                    continue;
                }
                Double KeepM = Entry.DepthM - RemoveM;
                if (KeepM > Constants.NUMERIC_COMPARISON_TOLERANCE)
                {
                    Output.Add((Entry.Cell, KeepM));
                    Remaining[Entry.Cell] = 0.0;
                }
                else
                {
                    Remaining[Entry.Cell] = Math.Max(0.0, RemoveM - Entry.DepthM);
                }
            }
            return Output;
        }

        /// <summary>
        /// Gives unpaired short cells the reason the short-list append would have used.
        /// </summary>
        /// <param name="Plan">Guidance plan whose short lists name the cells.</param>
        /// <param name="Deferred">Deferred entries already built from those cells.</param>
        /// <returns>The same entries, with minimum-length cells relabeled.</returns>
        static private List<HaulDeferredWork> RelabelUnpairedMinimumLength
            (
            GuidancePlanner.BuildResult Plan,
            IReadOnlyList<HaulDeferredWork> Deferred
            )
        {
            // Intent: the cleanup ledger splits a short cell across layers, so
            // the leftover depth is not the depth stored on the short list.
            HashSet<(HaulLaneBuilder.WorkKind Kind, (Int32 Bx, Int32 By) Cell)>
                MinimumLength =
                new HashSet<(HaulLaneBuilder.WorkKind Kind, (Int32 Bx, Int32 By) Cell)>();
            foreach (LifecyclePlanner.PassPlan PassPlan in Plan.Passes)
            {
                if (PassPlan.DeferredEntirePass)
                {
                    continue;
                }
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                    in PassPlan.CutShortOmittedCells)
                {
                    MinimumLength.Add((HaulLaneBuilder.WorkKind.Cut, Entry.Cell));
                }
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                    in PassPlan.FillShortOmittedCells)
                {
                    MinimumLength.Add((HaulLaneBuilder.WorkKind.Fill, Entry.Cell));
                }
            }

            List<HaulDeferredWork> Output = new List<HaulDeferredWork>(Deferred.Count);
            foreach (HaulDeferredWork Entry in Deferred)
            {
                if ((Entry.Reason == HaulDeferredWorkReasons.GUIDANCE_SEQUENCE_FAILED)
                    && MinimumLength.Contains((Entry.Kind, Entry.Cell)))
                {
                    Output.Add(new HaulDeferredWork(
                        Entry.Kind,
                        Entry.Cell,
                        Entry.LayerIndex,
                        Entry.DepthM,
                        Entry.VolumeBcy,
                        HaulDeferredWorkReasons.BELOW_MINIMUM_WORK_LENGTH));
                    continue;
                }
                Output.Add(Entry);
            }
            return Output;
        }

        /// <summary>
        /// Records leftover guidance-deferred work after cleanup routes are built.
        /// </summary>
        /// <param name="Result">Pipeline result to update.</param>
        /// <param name="Grid">Field grid for volume conversion.</param>
        /// <param name="Plan">Guidance plan whose short omissions still count.</param>
        /// <param name="ExtraDeferred">Additional unpaired cleanup leftovers.</param>
        /// <param name="CleanupCycleCount">Number of appended cleanup routes.</param>
        /// <param name="AppendPlanShortLists">
        /// When false, the plan's short lists are not appended. The empty-cleanup
        /// branch already carries those cells in <paramref name="ExtraDeferred"/>.
        /// </param>
        static internal void RecordGuidanceDeferredWork
            (
            HaulBuilderResult Result,
            Grid Grid,
            GuidancePlanner.BuildResult Plan,
            IReadOnlyList<HaulDeferredWork> ExtraDeferred,
            Int32 CleanupCycleCount,
            Boolean AppendPlanShortLists = true
            )
        {
            Double CutBcyPerDepthM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillBcyPerDepthM = CutBcyPerDepthM * Grid.Compaction;
            List<HaulDeferredWork> ShortDeferred = new List<HaulDeferredWork>();
            Int32 SequenceFailedPasses = 0;
            foreach (LifecyclePlanner.PassPlan PassPlan in Plan.Passes)
            {
                String Reason = PassPlan.DeferredEntirePass
                    ? HaulDeferredWorkReasons.GUIDANCE_SEQUENCE_FAILED
                    : HaulDeferredWorkReasons.BELOW_MINIMUM_WORK_LENGTH;
                if (PassPlan.DeferredEntirePass)
                {
                    SequenceFailedPasses += 1;
                }
                if (!AppendPlanShortLists)
                {
                    continue;
                }
                AppendShortDeferred(
                    ShortDeferred,
                    HaulLaneBuilder.WorkKind.Cut,
                    PassPlan.CutShortOmittedCells,
                    CutBcyPerDepthM,
                    Reason);
                AppendShortDeferred(
                    ShortDeferred,
                    HaulLaneBuilder.WorkKind.Fill,
                    PassPlan.FillShortOmittedCells,
                    FillBcyPerDepthM,
                    Reason);
            }
            ShortDeferred.AddRange(ExtraDeferred);
            if (ShortDeferred.Count == 0)
            {
                return;
            }

            List<HaulDeferredWork> Merged = new List<HaulDeferredWork>();
            if (Result.HaulOptimization != null)
            {
                Merged.AddRange(Result.HaulOptimization.DeferredWork);
            }
            Merged.AddRange(ShortDeferred);
            Double DeferredCutBcy = Merged
                .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
                .Sum(Entry => Entry.VolumeBcy);
            Double DeferredFillBcy = Merged
                .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
                .Sum(Entry => Entry.VolumeBcy);
            Double RequiredCutBcy = Result.DeferredWorkSummary?.RequiredCutBcy
                ?? Result.HaulOptimization?.PassResult.Passes.Sum(Pass => Pass.LoadBcy)
                ?? DeferredCutBcy;
            Double RequiredFillBcy = Result.DeferredWorkSummary?.RequiredFillBcy
                ?? RequiredCutBcy;
            if (Result.HaulOptimization != null)
            {
                HaulOptimizationResult Previous = Result.HaulOptimization;
                Result.HaulOptimization = new HaulOptimizationResult(
                    Previous.PassResult,
                    Previous.Axis,
                    Previous.CutLaneCount,
                    Previous.FillLaneCount,
                    Previous.TotalAssignedBcy,
                    DeferredCutBcy,
                    DeferredFillBcy,
                    Merged,
                    Previous.Validation,
                    Previous.PairRefinementSwaps,
                    Previous.PairRefinementBeforeM,
                    Previous.PairRefinementAfterM,
                    Previous.CompactPreflightFailures,
                    Previous.CompactOmittedCycles,
                    Previous.EdgeCompletionCycles,
                    Previous.EdgeOmittedCycles);
                Result.HaulOptimization.CopyClusterModeFrom(Previous);
            }
            Result.DeferredWorkSummary = HaulDeferredWorkSummary.Create(
                RequiredCutBcy,
                RequiredFillBcy,
                DeferredCutBcy,
                DeferredFillBcy,
                Merged);
            Double ShortCutBcy = ShortDeferred
                .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
                .Sum(Entry => Entry.VolumeBcy);
            Double ShortFillBcy = ShortDeferred
                .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
                .Sum(Entry => Entry.VolumeBcy);
            Result.Messages.Add(String.Format(
                CultureInfo.InvariantCulture,
                "Deferred {0} guidance cell(s) after cleanup routing ({1:F3} BCY cut, {2:F3} BCY fill; {3} unsequenceable pass(es); {4} cleanup route(s)).",
                ShortDeferred.Count,
                ShortCutBcy,
                ShortFillBcy,
                SequenceFailedPasses,
                CleanupCycleCount));
        }

        /// <summary>
        /// Appends deferred entries for cells omitted from automatic guidance.
        /// </summary>
        /// <param name="Output">Deferred-work accumulator.</param>
        /// <param name="Kind">Cut or fill kind.</param>
        /// <param name="Cells">Omitted cell/depth entries.</param>
        /// <param name="BcyPerDepthM">Bank-volume conversion factor.</param>
        /// <param name="Reason">Stable deferral reason string.</param>
        private static void AppendShortDeferred
            (
            List<HaulDeferredWork> Output,
            HaulLaneBuilder.WorkKind Kind,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double BcyPerDepthM,
            String Reason
            )
        {
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Cells[Index];
                if (Entry.DepthM <= 0.0)
                {
                    continue;
                }
                Output.Add(new HaulDeferredWork(
                    Kind,
                    Entry.Cell,
                    Index,
                    Entry.DepthM,
                    Entry.DepthM * BcyPerDepthM,
                    Reason));
            }
        }

        /// <summary>
        /// Builds one cut/fill guidance PNG for each of the first N routes.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Options">Configuration controlling the operation.</param>
        private static void BuildRouteGuidanceImages
            (
            HaulBuilderResult Result,
            Grid Grid,
            HaulBuilderOptions Options
            )
        {
            if (Options.RouteGuidanceImageCount <= 0)
            {
                return;
            }

            if (Result.LifecycleGuidance == null)
            {
                return;
            }

            Int32 PassCount = Result.Route?.OrderedPasses.Count ?? 0;
            if (PassCount <= 0)
            {
                return;
            }

            Int32 Take = Math.Min(Options.RouteGuidanceImageCount, PassCount);
            Double BladeM = Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER;
            for (Int32 RouteNumber = 1; RouteNumber <= Take; RouteNumber += 1)
            {
                String Key = String.Format(
                    CultureInfo.InvariantCulture,
                    "pair_{0:D4}",
                    RouteNumber);
                Result.PngBytesByName[Key] =
                    Render.RenderConsecutiveRouteSequencePng(
                        Result.LifecycleGuidance,
                        Grid,
                        RouteNumber,
                        1,
                        BladeM);
            }
        }

        /// <summary>
        /// Builds random PNGs of three consecutive routes with red cut and blue fill.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Options">Configuration controlling the operation.</param>
        private static void BuildRouteTriplets
            (
            HaulBuilderResult Result,
            Grid Grid,
            HaulBuilderOptions Options
            )
        {
            if (Options.RandomRouteTripletCount <= 0)
            {
                return;
            }

            if (Result.LifecycleGuidance == null)
            {
                return;
            }

            const Int32 ROUTES_PER_TRIPLET = 3;
            Int32 PassCount = Result.Route?.OrderedPasses.Count ?? 0;
            if (PassCount < ROUTES_PER_TRIPLET)
            {
                return;
            }

            Int32 MaximumStart = PassCount - ROUTES_PER_TRIPLET + 1;
            Int32 Take = Math.Min(Options.RandomRouteTripletCount, MaximumStart);
            Random Rng = new Random(Options.RandomSeed);
            HashSet<Int32> Starts = new HashSet<Int32>();
            while (Starts.Count < Take)
            {
                Starts.Add(Rng.Next(1, MaximumStart + 1));
            }

            Double BladeM = Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER;
            foreach (Int32 Start in Starts.OrderBy(Value => Value))
            {
                Int32 End = Start + ROUTES_PER_TRIPLET - 1;
                String Key = String.Format(
                    CultureInfo.InvariantCulture,
                    "triplet_{0:D4}_{1:D4}",
                    Start,
                    End);
                Result.PngBytesByName[Key] =
                    Render.RenderConsecutiveRouteSequencePng(
                        Result.LifecycleGuidance,
                        Grid,
                        Start,
                        ROUTES_PER_TRIPLET,
                        BladeM);
            }
        }

        /// <summary>
        /// Calculates pivot extra haul m.
        /// </summary>
        /// <param name="Route">The route used by the operation.</param>
        /// <param name="StaticObstacles">The static obstacles used by the operation.</param>
        /// <param name="ArmObstacles">The arm obstacles used by the operation.</param>
        /// <returns>The resulting pivot extra haul m.</returns>

        private static Double CalculatePivotExtraHaulM(
            Route Route,
            ObstacleField StaticObstacles,
            IReadOnlyDictionary<Int32, ObstacleField> ArmObstacles)
        {
            Double ExtraM = 0.0;
            for (Int32 Index = 0; Index < Route.OrderedPasses.Count; Index += 1)
            {
                Int32 RouteNumber = Index + 1;
                if (!ArmObstacles.TryGetValue(
                    RouteNumber, out ObstacleField? Field)
                    || (Field == null))
                {
                    continue;
                }
                Pass Pass = Route.OrderedPasses[Index];
                Double CombinedM = Field.PathLength(
                    Pass.CutEndXy, Pass.FillStartXy);
                Double StaticM = StaticObstacles.Active
                    ? StaticObstacles.PathLength(
                        Pass.CutEndXy, Pass.FillStartXy)
                    : SegmentDistance(
                        Pass.CutEndXy, Pass.FillStartXy);
                ExtraM += Math.Max(0.0, CombinedM - StaticM);
            }
            return ExtraM;
        }

        /// <summary>
        /// Executes the segment distance operation.
        /// </summary>
        /// <param name="First">The first used by the operation.</param>
        /// <param name="Second">The second used by the operation.</param>
        /// <returns>The resulting segment distance.</returns>

        private static Double SegmentDistance(
            (Double X, Double Y) First,
            (Double X, Double Y) Second)
        {
            Double Dx = Second.X - First.X;
            Double Dy = Second.Y - First.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Builds pair samples.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Points">The points used by the operation.</param>
        /// <param name="Options">Configuration controlling the operation.</param>

        private static void BuildPairSamples
            (
            HaulBuilderResult Result,
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            HaulBuilderOptions Options
            )
        {
            // Random single-pair PNGs mirror make_pair_images.py for operator-facing cut/haul/fill diagrams.
            if (Options.RandomRouteImageCount <= 0)
            {
                return;
            }

            Dictionary<Int32, List<GuidancePoint>> ByPass = new Dictionary<Int32, List<GuidancePoint>>();
            for (Int32 Iter = 0; Iter < Points.Count; Iter += 1)
            {
                GuidancePoint Point = Points[Iter];
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

            Int32 Take = Math.Min(Options.RandomRouteImageCount, Usable.Count);
            Random Rng = new Random(Options.RandomSeed);
            List<Int32> Pool = new List<Int32>(Usable);
            for (Int32 Iter = 0; Iter < Pool.Count; Iter += 1)
            {
                Int32 Swap = Iter + Rng.Next(Pool.Count - Iter);
                (Pool[Iter], Pool[Swap]) = (Pool[Swap], Pool[Iter]);
            }

            List<Int32> Chosen = Pool.Take(Take).OrderBy(Value => Value).ToList();
            Double BladeM = (Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER);
            for (Int32 Iter = 0; Iter < Chosen.Count; Iter += 1)
            {
                Int32 PassNo = Chosen[Iter];
                String Key = String.Format(CultureInfo.InvariantCulture, "pair_{0:D4}", PassNo);
                Result.PngBytesByName[Key] = Render.RenderSinglePairPng(Grid, ByPass[PassNo], BladeM);
            }
        }

        /// <summary>
        /// Builds timing and metrics.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="PassResult">The pass result used by the operation.</param>
        /// <param name="Route">The route used by the operation.</param>
        /// <param name="HeadingDeg">Direction in degrees.</param>
        /// <param name="Options">Configuration controlling the operation.</param>

        private static void BuildTimingAndMetrics
            (
            HaulBuilderResult Result,
            Grid Grid,
            PassResult PassResult,
            Route Route,
            Double HeadingDeg,
            HaulBuilderOptions Options
            )
        {
            // KPI calculations mirror CLI printouts so API and CLI users see identical operational metrics.
            TimeParams Params = new TimeParams(
                Options.CutSpeedMph,
                Options.LoadedTravelSpeedMph,
                Options.FillSpeedMph,
                Options.UnloadedTravelSpeedMph,
                Options.AccelerationTimeS,
                Options.DecelerationTimeS,
                Options.FieldEfficiency);
            TimeResult Time = Timemodel.EstimateTime(Grid, Route, Params);
            Result.TimeResult = Time;

            Double MovedBcy = 0.0;
            for (Int32 Iter = 0; Iter < PassResult.Passes.Count; Iter += 1)
            {
                Pass Pass = PassResult.Passes[Iter];
                MovedBcy += Pass.LoadBcy;
            }

            Int32 Valid = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx]) Valid += 1;
                }
            }

            Double AreaAc = ((Valid * Grid.AreaM2) / M2_PER_ACRE);
            Double DeterministicH = Result.DetailedGuidanceTime != null
                ? (Result.DetailedGuidanceTime.MotionSeconds
                    + Result.DetailedGuidanceTime.EventSeconds) / Constants.SECONDS_PER_HOUR
                : (Time.TotalTimeS - Time.OtherTimeS) / Constants.SECONDS_PER_HOUR;
            Double ScheduledH = Result.DetailedGuidanceTime != null
                ? Result.DetailedGuidanceTime.TotalSeconds / Constants.SECONDS_PER_HOUR
                : Time.TotalTimeS / Constants.SECONDS_PER_HOUR;
            Double DeterministicMoveRate = DeterministicH > 0.0
                ? MovedBcy / DeterministicH
                : 0.0;
            Double ScheduledMoveRate = ScheduledH > 0.0
                ? MovedBcy / ScheduledH
                : 0.0;
            Double GoldenAchievementPercent =
                100.0 * DeterministicMoveRate
                    / Constants.GOLDEN_MOVE_RATE_BCY_PER_HOUR;
            Double AvgMoveYd = PassResult.Passes.Count > 0 ? ((Time.HaulingDistM / PassResult.Passes.Count) / METERS_PER_YARD) : 0.0;
            Result.Metrics = new HaulAnalysisMetrics
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
                LaneAxisHeadingDeg = HeadingDeg,
                PivotExtraHaulM = Result.PivotExtraHaulM,
                PivotAddedTimeS = Result.PivotAddedTimeS,
                ManualCleanupBcy = Result.DeferredWorkSummary?.ManualCleanupBcy
                    ?? 0.0,
                DeferredCutBcy = Result.DeferredWorkSummary?.DeferredCutBcy
                    ?? PassResult.UnmatchedCutBcy,
                DeferredFillBcy = Result.DeferredWorkSummary?.DeferredFillBcy
                    ?? PassResult.UnmatchedFillBcy,
            };
            Result.Messages.Add(String.Format(
                CultureInfo.InvariantCulture,
                "Productivity: {0:F1} BCY/deterministic guidance hour versus {1:F1} golden target ({2:F1}%); scheduled productivity={3:F1} BCY/h; automatic volume={4:F1} BCY/acre.",
                DeterministicMoveRate,
                Constants.GOLDEN_MOVE_RATE_BCY_PER_HOUR,
                GoldenAchievementPercent,
                ScheduledMoveRate,
                Result.Metrics.MoveVolumeBcyPerAcre));
        }

        /// <summary>
        /// Builds animations.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Route">The route used by the operation.</param>
        /// <param name="Options">Configuration controlling the operation.</param>

        private static void BuildAnimations
            (
            HaulBuilderResult Result,
            Grid Grid,
            Route Route,
            HaulBuilderOptions Options
            )
        {
            // Base leveling animation can be emitted as per-frame PNGs and/or one GIF.
            Boolean NeedAnimation = Options.IncludeAnimationGif
                || Options.IncludeAnimationFrames;
            if (!NeedAnimation) return;
            Animate.AnimationPayload Payload = Animate.AnimateLeveling(
                Grid, Route.OrderedPasses, Options.AnimationStride,
                Options.AnimationPixelsPerCell, 80, Options.AnimationMode);
            if (Options.IncludeAnimationGif && (Payload.GifBytes is not null))
                Result.AnimationGifBytes = Payload.GifBytes;
            if (!Options.IncludeAnimationFrames) return;
            for (Int32 Iter = 0; Iter < Payload.FramePngBytes.Count; Iter += 1)
            {
                Result.PngBytesByName[String.Format(CultureInfo.InvariantCulture, "animation_frame_{0:D5}", Iter)] = Payload.FramePngBytes[Iter];
            }
        }

        /// <summary>
        /// Builds pivot.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Route">The route used by the operation.</param>
        /// <param name="Obstacles">The obstacles used by the operation.</param>
        /// <param name="Options">Configuration controlling the operation.</param>

        private static void BuildPivot
            (
            HaulBuilderResult Result,
            Grid Grid,
            Route Route,
            ObstacleField Obstacles,
            HaulBuilderOptions Options
            )
        {
            // Pivot stage is optional and includes both static schedule artifacts and optional animation.
            if (!Options.EnablePivot)
            {
                return;
            }

            Pivot.Grid PivotGrid = ToPivotGrid(Grid);
            List<Pivot.Pass> PivotPasses = ToPivotPasses(Route.OrderedPasses);
            Pivot.ObstacleField PivotObstacles = ToPivotObstacleField(Obstacles);
            (Double X, Double Y)? PivotCenter = ResolvePivotCenter(
                Result, Grid, Options.PivotCenter);
            Pivot.PivotGeometry Geometry = Pivot.ComputePivotGeometry(
                PivotGrid,
                Options.PivotWheelbaseFt,
                Options.PivotClearanceFt,
                PivotCenter);
            Pivot.PivotPlan Plan = Pivot.PlanPivot(PivotGrid, PivotPasses, Geometry, 720, Options.PivotMoveCost);
            Dictionary<Int32, Pivot.ObstacleField> Arm = Pivot.BuildArmObstacles(Geometry, Plan, PivotObstacles, ((Options.EquipmentWidthFt * Constants.FEET_TO_METERS) / 2.0));
            (Int32 Routed, Double ExtraHaulM, Int32 UnderArm, _) =
                Pivot.PivotDetourStats(PivotPasses, Arm);
            Result.PivotGeometry = Geometry;
            Result.PivotPlan = Plan;
            Result.ArmObstacles = Arm;
            Result.PivotExtraHaulM = ExtraHaulM;
            Result.PivotCsvData = Pivot.WritePivotCsv(Plan);
            Result.CsvTextByName["pivot"] = BuildPivotCsv(Result.PivotCsvData);
            Result.Messages.Add(String.Format(
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
                    Options.PivotMode, Options.PivotPixelsPerCell);
                foreach (String PathItem in Paths)
                {
                    String Name = Path.GetFileNameWithoutExtension(PathItem);
                    Result.PivotPngBytesByName[Name] = File.ReadAllBytes(PathItem);
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

            if (Options.IncludePivotAnimationGif
                || Options.IncludePivotAnimationFrames)
            {
                Animate.AnimationPayload Payload = Animate.AnimatePivotLeveling(
                    Grid,
                    Route.OrderedPasses,
                    Plan,
                    Options.PivotAnimationStride,
                    Options.PivotPixelsPerCell,
                    Options.PivotMode);
                if (Options.IncludePivotAnimationGif)
                    Result.PivotAnimationGifBytes = Payload.GifBytes;
                if (!Options.IncludePivotAnimationFrames) return;
                for (Int32 Iter = 0; Iter < Payload.FramePngBytes.Count; Iter += 1)
                {
                    Result.PivotPngBytesByName[String.Format(CultureInfo.InvariantCulture, "pivot_animation_frame_{0:D5}", Iter)] = Payload.FramePngBytes[Iter];
                }
            }
        }

        /// <summary>
        /// Executes the to pivot grid operation.
        /// </summary>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting to pivot grid.</returns>

        private static Pivot.Grid ToPivotGrid
            (
            Grid Grid
            )
        {
            // Adapt top-level grid into Pivot's nested grid DTO.
            return new Pivot.Grid(Grid.GridSizeM, Grid.MinX, Grid.MinY, Grid.Zone, Grid.IsNorthern, Grid.Existing, Grid.Target, Grid.Valid);
        }

        /// <summary>
        /// Resolves pivot center.
        /// </summary>
        /// <param name="Result">The result used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Specification">The specification used by the operation.</param>
        /// <returns>The resulting pivot center.</returns>

        private static (Double X, Double Y)? ResolvePivotCenter(
            HaulBuilderResult Result,
            Grid Grid,
            String? Specification)
        {
            if (String.IsNullOrWhiteSpace(Specification)) return null;
            String[] Parts = Specification.Split(',');
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
                    nameof(Specification));
            }
            UtmCoord Center = Agd.UtmFromLatlon(Lat, Lon);
            if ((Center.Zone != Grid.Zone)
                || (Center.IsNorthern != Grid.IsNorthern))
            {
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Pivot center projects to UTM zone {0}{1}, while the field uses {2}{3}; continuing with projected coordinates.",
                    Center.Zone,
                    Center.IsNorthern ? "N" : "S",
                    Grid.Zone,
                    Grid.IsNorthern ? "N" : "S"));
            }
            return (Center.Easting, Center.Northing);
        }

        /// <summary>
        /// Executes the to pivot passes operation.
        /// </summary>
        /// <param name="Passes">The passes used by the operation.</param>
        /// <returns>The resulting to pivot passes.</returns>

        private static List<Pivot.Pass> ToPivotPasses
            (
            IReadOnlyList<Pass> Passes
            )
        {
            // Convert pass tuple cells into Pivot.PassCell sequences.
            List<Pivot.Pass> Out = new List<Pivot.Pass>();
            for (Int32 Iter = 0; Iter < Passes.Count; Iter += 1)
            {
                List<Pivot.PassCell> Cut = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Passes[Iter].CutCells)
                {
                    Cut.Add(new Pivot.PassCell(Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM));
                }

                List<Pivot.PassCell> Fill = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Passes[Iter].FillCells)
                {
                    Fill.Add(new Pivot.PassCell(Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM));
                }

                Out.Add(new Pivot.Pass(Cut, Fill, Passes[Iter].CutEndXy, Passes[Iter].FillStartXy));
            }

            return Out;
        }

        /// <summary>
        /// Executes the to pivot obstacle field operation.
        /// </summary>
        /// <param name="Obstacles">The obstacles used by the operation.</param>
        /// <returns>The resulting to pivot obstacle field.</returns>

        private static Pivot.ObstacleField ToPivotObstacleField
            (
            ObstacleField Obstacles
            )
        {
            // Rebuild Pivot obstacle geometry from top-level obstacle zones.
            if (!Obstacles.Active)
            {
                return Pivot.ObstacleField.EMPTY_OBSTACLES;
            }

            List<Pivot.ExclusionZone> Zones = new List<Pivot.ExclusionZone>();
            foreach (ExclusionZone Zone in Obstacles.Zones)
            {
                Zones.Add(new Pivot.ExclusionZone(Zone.Ring, Zone.BufferRing));
            }

            return Pivot.ObstacleField.FromZones(Zones);
        }

        /// <summary>
        /// Executes the to guidance obstacle fields operation.
        /// </summary>
        /// <param name="Obstacles">The obstacles used by the operation.</param>
        /// <returns>The resulting to guidance obstacle fields.</returns>

        private static Dictionary<Int32, ObstacleField>?
            ToGuidanceObstacleFields
            (
            Dictionary<Int32, Pivot.ObstacleField>? Obstacles
            )
        {
            if ((Obstacles == null) || (Obstacles.Count == 0)) return null;
            Dictionary<Pivot.ObstacleField, ObstacleField> Cache =
                new Dictionary<Pivot.ObstacleField, ObstacleField>();
            Dictionary<Int32, ObstacleField> Result =
                new Dictionary<Int32, ObstacleField>();
            foreach (KeyValuePair<Int32, Pivot.ObstacleField> Entry in Obstacles)
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

        /// <summary>
        /// Executes the parse polygon specs operation.
        /// </summary>
        /// <param name="Specs">The specs used by the operation.</param>
        /// <returns>The resulting parse polygon specs.</returns>

        private static List<List<(Double Lat, Double Lon)>> ParsePolygonSpecs
            (
            IReadOnlyList<String> Specs
            )
        {
            // Parsing follows the CLI grammar: repeated flag, each as lat,lon pairs separated by semicolons.
            List<List<(Double Lat, Double Lon)>> Rings = new List<List<(Double Lat, Double Lon)>>();
            for (Int32 Iter = 0; Iter < Specs.Count; Iter += 1)
            {
                String Spec = Specs[Iter];
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

        /// <summary>
        /// Builds zone csv.
        /// </summary>
        /// <param name="Steps">The steps used by the operation.</param>
        /// <returns>The resulting zone csv.</returns>

        private static String BuildZoneCsv
            (
            IReadOnlyList<ZoneStep> Steps
            )
        {
            // CSV building in-memory replaces filesystem writes while keeping parity with Python column order.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("work_order,zone_id,compass,loads,moved_bcy,easting_m,northing_m,lat,lon,first_pass,last_pass,mean_pass");
            for (Int32 Iter = 0; Iter < Steps.Count; Iter += 1)
            {
                dynamic S = Steps[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3},{4:F1},{5:F2},{6:F2},{7:F7},{8:F7},{9},{10},{11:F0}", (Int32)S.WorkOrder, (Int32)S.ZoneId, (String)S.Compass, (Int32)S.NLoads, (Double)S.TotalBcy, (Double)S.Easting, (Double)S.Northing, (Double)S.Lat, (Double)S.Lon, (Int32)S.FirstPass, (Int32)S.LastPass, (Double)S.MeanPass);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        /// <summary>
        /// Builds pairs csv.
        /// </summary>
        /// <param name="Pairs">The pairs used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <returns>The resulting pairs csv.</returns>

        private static String BuildPairsCsv
            (
            IReadOnlyList<CutFillPair> Pairs,
            AgdField Field
            )
        {
            // Pair CSV mirrors the Python writer while remaining pure in-memory.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("order,load_bcy,haul_m,total_cut_bcy_before,total_fill_bcy_before,route_time_s,cut_start_lat,cut_start_lon,cut_end_lat,cut_end_lon,fill_start_lat,fill_start_lon,fill_end_lat,fill_end_lon");
            for (Int32 Iter = 0; Iter < Pairs.Count; Iter += 1)
            {
                CutFillPair P = Pairs[Iter];
                (Double CsLat, Double CsLon) = Agd.LatLonFromUtm(P.CutStartXy.X, P.CutStartXy.Y, Field.Zone, Field.IsNorthern);
                (Double CeLat, Double CeLon) = Agd.LatLonFromUtm(P.CutEndXy.X, P.CutEndXy.Y, Field.Zone, Field.IsNorthern);
                (Double FsLat, Double FsLon) = Agd.LatLonFromUtm(P.FillStartXy.X, P.FillStartXy.Y, Field.Zone, Field.IsNorthern);
                (Double FeLat, Double FeLon) = Agd.LatLonFromUtm(P.FillEndXy.X, P.FillEndXy.Y, Field.Zone, Field.IsNorthern);
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

        /// <summary>
        /// Builds guidance csv.
        /// </summary>
        /// <param name="Points">The points used by the operation.</param>
        /// <returns>The resulting guidance csv.</returns>

        private static String BuildGuidanceCsv
            (
            IReadOnlyList<GuidancePoint> Points
            )
        {
            // Minimal guidance export covers the core turn-by-turn fields consumed by operator tools.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("seq,pass_no,action,x,y,lat,lon,depth_mm,heading_deg,blade,instruction");
            for (Int32 Iter = 0; Iter < Points.Count; Iter += 1)
            {
                dynamic P = Points[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3:F3},{4:F3},{5:F7},{6:F7},{7:F1},{8:F1},{9},{10}", (Int32)P.Seq, (Int32)P.PassNo, (String)P.Action, (Double)P.X, (Double)P.Y, (Double)P.Lat, (Double)P.Lon, (Double)P.DepthMm, (Double)P.HeadingDeg, (String)P.Blade, (String)P.Instruction);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        /// <summary>
        /// Builds pivot csv.
        /// </summary>
        /// <param name="Data">The data used by the operation.</param>
        /// <returns>The resulting pivot csv.</returns>

        private static String BuildPivotCsv
            (
            Pivot.PivotCsvData Data
            )
        {
            // Pivot schedule is serialized from typed rows so callers can consume either table or object model.
            StringBuilder Sb = new StringBuilder();
            foreach (KeyValuePair<String, String> Pair in Data.Metadata)
            {
                Sb.Append('#');
                Sb.Append(Pair.Key);
                Sb.Append('=');
                Sb.AppendLine(Pair.Value);
            }

            Sb.AppendLine("position,first_pass,last_pass,pass_count,bearing_deg,cardinal,move_deg,tip_lat,tip_lon,conflict_count");
            for (Int32 Iter = 0; Iter < Data.Rows.Count; Iter += 1)
            {
                Pivot.PivotCsvRow Row = Data.Rows[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3},{4:F2},{5},{6:F2},{7:F7},{8:F7},{9}", Row.Position, Row.FirstPass, Row.LastPass, Row.PassCount, Row.BearingDeg, Row.Cardinal, Row.MoveDeg, Row.TipLat, Row.TipLon, Row.ConflictCount);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        /// <summary>
        /// Executes the clone options operation.
        /// </summary>
        /// <param name="Options">Configuration controlling the operation.</param>
        /// <returns>The resulting clone options.</returns>

        static internal HaulBuilderOptions CloneOptions
            (
            HaulBuilderOptions Options
            )
        {
            // Cloning ensures the result captures immutable run configuration.
            HaulBuilderOptions Copy = new HaulBuilderOptions
            {
                AgdPath = Options.AgdPath,
                GridSizeFt = Options.GridSizeFt,
                ToleranceMm = Options.ToleranceMm,
                ScraperCapacityLcy = Options.ScraperCapacityLcy,
                SwellFactor = Options.SwellFactor,
                CutFillRatio = Options.CutFillRatio,
                MaxDepthMm = Options.MaxDepthMm,
                BladeWidthMm = Options.BladeWidthMm,
                FieldEfficiency = Options.FieldEfficiency,
                CutSpeedMph = Options.CutSpeedMph,
                LoadedTravelSpeedMph = Options.LoadedTravelSpeedMph,
                FillSpeedMph = Options.FillSpeedMph,
                UnloadedTravelSpeedMph = Options.UnloadedTravelSpeedMph,
                AccelerationTimeS = Options.AccelerationTimeS,
                DecelerationTimeS = Options.DecelerationTimeS,
                PixelsPerCell = Options.PixelsPerCell,
                FlowArrowStrideCells = Options.FlowArrowStrideCells,
                ZoneCount = Options.ZoneCount,
                IncludeCutFillPairCsv = Options.IncludeCutFillPairCsv,
                IncludeCutFillPairPng = Options.IncludeCutFillPairPng,
                IncludeGuidancePng = Options.IncludeGuidancePng,
                IncludeGuidanceCsv = Options.IncludeGuidanceCsv,
                TurnSpeedMph = Options.TurnSpeedMph,
                GradeSpeedMph = Options.GradeSpeedMph,
                MinimumTurningDiameterFt = Options.MinimumTurningDiameterFt,
                EquipmentLengthFt = Options.EquipmentLengthFt,
                MinimumWorkLengthFt = Options.MinimumWorkLengthFt,
                EnableMinimumWorkLength = Options.EnableMinimumWorkLength,
                WaypointTurnThresholdDeg = Options.WaypointTurnThresholdDeg,
                IncludeAnimationGif = Options.IncludeAnimationGif,
                IncludeAnimationFrames = Options.IncludeAnimationFrames,
                AnimationStride = Options.AnimationStride,
                AnimationPixelsPerCell = Options.AnimationPixelsPerCell,
                AnimationMode = Options.AnimationMode,
                EnablePivot = Options.EnablePivot,
                PivotCenter = Options.PivotCenter,
                PivotWheelbaseFt = Options.PivotWheelbaseFt,
                PivotClearanceFt = Options.PivotClearanceFt,
                PivotMoveCost = Options.PivotMoveCost,
                PivotMode = Options.PivotMode,
                PivotPixelsPerCell = Options.PivotPixelsPerCell,
                IncludePivotAnimationGif = Options.IncludePivotAnimationGif,
                IncludePivotAnimationFrames = Options.IncludePivotAnimationFrames,
                PivotAnimationStride = Options.PivotAnimationStride,
                EquipmentWidthFt = Options.EquipmentWidthFt,
                SkipHaulPlan = Options.SkipHaulPlan,
                SkipGuidance = Options.SkipGuidance,
                RandomRouteImageCount = Options.RandomRouteImageCount,
                RouteGuidanceImageCount = Options.RouteGuidanceImageCount,
                RandomRouteTripletCount = Options.RandomRouteTripletCount,
                RandomSeed = Options.RandomSeed,
            };
            CopyOptimization(Options.Optimization, Copy.Optimization);
            CopyGuidance(Options.Guidance, Copy.Guidance);
            CopyPonding(Options.Ponding, Copy.Ponding);
            Copy.ExcludePolygonSpecs.AddRange(Options.ExcludePolygonSpecs);
            return Copy;
        }

        private static void CopyPonding(
            PondingOptions Source,
            PondingOptions Destination)
        {
            Destination.Optimize = Source.Optimize;
            Destination.ObjectiveMode = Source.ObjectiveMode;
            Destination.Audit = Source.Audit;
            Destination.NoImplicitAudit = Source.NoImplicitAudit;
            Destination.TargetSpillOutlets = Source.TargetSpillOutlets;
            Destination.SpillToleranceFt = Source.SpillToleranceFt;
            Destination.Weight = Source.Weight;
            Destination.CheckpointInterval = Source.CheckpointInterval;
            Destination.WhiteBoxToolsPath = Source.WhiteBoxToolsPath;
            Destination.WhiteBoxToolsWorkDirectory = Source.WhiteBoxToolsWorkDirectory;
            Destination.WarnDepthFt = Source.WarnDepthFt;
            Destination.WarnVolumeBcy = Source.WarnVolumeBcy;
            Destination.DesignRainfallIn = Source.DesignRainfallIn;
            Destination.FailOnWarn = Source.FailOnWarn;
            Destination.IncludeMap = Source.IncludeMap;
            Destination.MapPixelsPerCell = Source.MapPixelsPerCell;
            Destination.OutletSpecs.AddRange(Source.OutletSpecs);
            Destination.OutletPolygonSpecs.AddRange(Source.OutletPolygonSpecs);
        }

        /// <summary>
        /// Sequences one-strip interior loads, then other interior loads, then
        /// the field-edge batch. The reorder window stays inside each band.
        /// </summary>
        /// <param name="Passes">Loads to order.</param>
        /// <param name="EdgeCutCells">Cut cells owned by the field-edge batch.</param>
        /// <param name="EdgeFillCells">Fill cells owned by the field-edge batch.</param>
        /// <param name="Opt">Production options, including the reorder window.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <returns>The banded route and the size of each band.</returns>
        private static HaulRouteBanding.Result BandProductionRoute
            (
            IReadOnlyList<Pass> Passes,
            HashSet<(Int32 Bx, Int32 By)> EdgeCutCells,
            HashSet<(Int32 Bx, Int32 By)> EdgeFillCells,
            HaulBuilderOptions Opt,
            ObstacleField Obstacles
            )
        {
            return HaulRouteBanding.Apply(
                Passes,
                EdgeCutCells,
                EdgeFillCells,
                TwoOptCap: 1500,
                GradeReturn: true,
                DirectionAware: true,
                UnloadedTravelSpeedMph: Opt.UnloadedTravelSpeedMph,
                TurnSpeedMph: Opt.TurnSpeedMph,
                TurningCircleFt: Opt.MinimumTurningDiameterFt,
                Obstacles: Obstacles,
                StartCandidateCount: 8,
                ReorderWindow: Opt.Optimization.RouteReorderWindow,
                ReorderMaxRounds: 12,
                OrderByDistance: Opt.Optimization.OptimizeRouteOrderByDistance);
        }

        /// <summary>
        /// Executes the copy optimization operation.
        /// </summary>
        /// <param name="Source">The source used by the operation.</param>
        /// <param name="Destination">The destination used by the operation.</param>

        private static void CopyOptimization(
            HaulOptimizationOptions Source,
            HaulOptimizationOptions Destination)
        {
            Destination.EnableRouteConsolidation = Source.EnableRouteConsolidation;
            Destination.RouteConsolidationGroupSize = Source.RouteConsolidationGroupSize;
            Destination.RouteConsolidationMaximumCycles = Source.RouteConsolidationMaximumCycles;
            Destination.PairRefinementRounds = Source.PairRefinementRounds;
            Destination.EnableMultiRouteExchange = Source.EnableMultiRouteExchange;
            Destination.MultiRouteExchangeRounds = Source.MultiRouteExchangeRounds;
            Destination.MultiRouteExchangeGroupSize = Source.MultiRouteExchangeGroupSize;
            Destination.MultiRouteExchangeMaximumGroups = Source.MultiRouteExchangeMaximumGroups;
            Destination.MultiRouteExchangeMinimumImprovementM = Source.MultiRouteExchangeMinimumImprovementM;
            Destination.EnableCycleChainOptimization = Source.EnableCycleChainOptimization;
            Destination.CycleChainRounds = Source.CycleChainRounds;
            Destination.CycleChainCandidatesPerRoute = Source.CycleChainCandidatesPerRoute;
            Destination.CycleChainMinimumImprovementSeconds = Source.CycleChainMinimumImprovementSeconds;
            Destination.RouteReorderWindow = Source.RouteReorderWindow;
            Destination.EnableEarlyFullStripBanding = Source.EnableEarlyFullStripBanding;
            Destination.OptimizeRouteOrderByDistance = Source.OptimizeRouteOrderByDistance;
            Destination.ClusterModeAverageWindow = Source.ClusterModeAverageWindow;
            Destination.ClusterModeLoadFraction = Source.ClusterModeLoadFraction;
        }

        /// <summary>
        /// Copies Dubins guidance rules onto the clone's own group.
        /// </summary>
        /// <param name="Source">Rules to read.</param>
        /// <param name="Destination">Rules on the cloned options.</param>
        private static void CopyGuidance
            (
            GuidanceOptions Source,
            GuidanceOptions Destination
            )
        {
            Destination.ShortJointRadii = Source.ShortJointRadii;
            Destination.ConnectorTieFraction = Source.ConnectorTieFraction;
            Destination.ConnectorTieRadii = Source.ConnectorTieRadii;
            Destination.SteerHeadingDeg = Source.SteerHeadingDeg;
            Destination.MaxCorrectingLines = Source.MaxCorrectingLines;
            Destination.LeftoverRatio = Source.LeftoverRatio;
            Destination.MergeRouteRadii = Source.MergeRouteRadii;
            Destination.EndFragmentSpanRadii = Source.EndFragmentSpanRadii;
            Destination.EndFragmentGapRadii = Source.EndFragmentGapRadii;
            Destination.NearEmptyRadii = Source.NearEmptyRadii;
            Destination.NearEmptyBowlFraction = Source.NearEmptyBowlFraction;
            Destination.ArcSpeedMph = Source.ArcSpeedMph;
            Destination.GradeLaneSpeedMph = Source.GradeLaneSpeedMph;
        }

        /// <summary>
        /// Executes the validate options operation.
        /// </summary>
        /// <param name="Options">Configuration controlling the operation.</param>

        private static void ValidateOptions
            (
            HaulBuilderOptions Options
            )
        {
            // Rejects non-finite and non-positive option values.

            static void Positive(Double Value, String Name)
            {
                if (!Double.IsFinite(Value) || (Value <= 0.0))
                    throw new ArgumentOutOfRangeException(
                        Name, "Value must be finite and greater than zero.");
            }

            Positive(Options.GridSizeFt, nameof(Options.GridSizeFt));
            if (!Double.IsFinite(Options.ToleranceMm)
                || (Options.ToleranceMm < 0.0))
                throw new ArgumentOutOfRangeException(
                    nameof(Options.ToleranceMm));
            Positive(Options.ScraperCapacityLcy,
                nameof(Options.ScraperCapacityLcy));
            Positive(Options.SwellFactor, nameof(Options.SwellFactor));
            Positive(Options.CutFillRatio, nameof(Options.CutFillRatio));
            Positive(Options.MaxDepthMm, nameof(Options.MaxDepthMm));
            Positive(Options.BladeWidthMm, nameof(Options.BladeWidthMm));
            if (!Double.IsFinite(Options.FieldEfficiency)
                || (Options.FieldEfficiency <= 0.0)
                || (Options.FieldEfficiency > 1.0))
                throw new ArgumentOutOfRangeException(
                    nameof(Options.FieldEfficiency));
            Positive(Options.CutSpeedMph, nameof(Options.CutSpeedMph));
            Positive(Options.LoadedTravelSpeedMph,
                nameof(Options.LoadedTravelSpeedMph));
            Positive(Options.FillSpeedMph, nameof(Options.FillSpeedMph));
            Positive(Options.UnloadedTravelSpeedMph,
                nameof(Options.UnloadedTravelSpeedMph));
            Positive(Options.TurnSpeedMph, nameof(Options.TurnSpeedMph));
            Positive(Options.GradeSpeedMph, nameof(Options.GradeSpeedMph));
            Positive(Options.AccelerationTimeS,
                nameof(Options.AccelerationTimeS));
            Positive(Options.DecelerationTimeS,
                nameof(Options.DecelerationTimeS));
            Positive(Options.MinimumTurningDiameterFt,
                nameof(Options.MinimumTurningDiameterFt));
            Positive(Options.EquipmentLengthFt,
                nameof(Options.EquipmentLengthFt));
            if (!Double.IsFinite(Options.MinimumWorkLengthFt)
                || (Options.MinimumWorkLengthFt < 0.0))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(Options.MinimumWorkLengthFt));
            }
            Positive(Options.EquipmentWidthFt,
                nameof(Options.EquipmentWidthFt));
            if (!Double.IsFinite(Options.WaypointTurnThresholdDeg)
                || (Options.WaypointTurnThresholdDeg < 0.0)
                || (Options.WaypointTurnThresholdDeg > Constants.DEGREES_PER_HALF_TURN))
                throw new ArgumentOutOfRangeException(
                    nameof(Options.WaypointTurnThresholdDeg));
            if (Options.PixelsPerCell <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.PixelsPerCell));
            if (Options.FlowArrowStrideCells < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.FlowArrowStrideCells));
            if (Options.ZoneCount < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.ZoneCount));
            if (Options.RandomRouteImageCount < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.RandomRouteImageCount));
            if (Options.RouteGuidanceImageCount < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.RouteGuidanceImageCount));
            if (Options.RandomRouteTripletCount < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.RandomRouteTripletCount));
            if (Options.AnimationStride <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.AnimationStride));
            if (Options.AnimationPixelsPerCell <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.AnimationPixelsPerCell));
            if (!String.Equals(Options.AnimationMode, "cutfill",
                    StringComparison.OrdinalIgnoreCase)
                && !String.Equals(Options.AnimationMode, "topology",
                    StringComparison.OrdinalIgnoreCase))
                throw new ArgumentException(
                    "AnimationMode must be 'cutfill' or 'topology'.",
                    nameof(Options.AnimationMode));
            if (Options.EnablePivot)
            {
                Positive(Options.PivotWheelbaseFt,
                    nameof(Options.PivotWheelbaseFt));
                if (!Double.IsFinite(Options.PivotClearanceFt)
                    || (Options.PivotClearanceFt < 0.0))
                    throw new ArgumentOutOfRangeException(
                        nameof(Options.PivotClearanceFt));
                if (!Double.IsFinite(Options.PivotMoveCost)
                    || (Options.PivotMoveCost < 0.0))
                    throw new ArgumentOutOfRangeException(
                        nameof(Options.PivotMoveCost));
                if (Options.PivotPixelsPerCell <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(Options.PivotPixelsPerCell));
                if (Options.PivotAnimationStride <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(Options.PivotAnimationStride));
                if (!String.Equals(Options.PivotMode, "cutfill",
                        StringComparison.OrdinalIgnoreCase)
                    && !String.Equals(Options.PivotMode, "topology",
                        StringComparison.OrdinalIgnoreCase))
                    throw new ArgumentException(
                        "PivotMode must be 'cutfill' or 'topology'.",
                        nameof(Options.PivotMode));
            }

            ValidateOptimizationOptions(Options);
            ValidateGuidanceOptions(Options.Guidance);
            ValidatePondingOptions(Options.Ponding);
        }

        /// <summary>
        /// Rejects a Dubins rule that is not a finite limit.
        /// </summary>
        /// <param name="Rules">Guidance rules to check.</param>
        private static void ValidateGuidanceOptions
            (
            GuidanceOptions Rules
            )
        {
            if (Rules == null)
            {
                throw new ArgumentNullException(nameof(Rules));
            }

            // A zero radius or fraction turns that limit off. A negative does not.
            static void NonNegative(Double Value, String Name)
            {
                if (!Double.IsFinite(Value) || (Value < 0.0))
                {
                    throw new ArgumentOutOfRangeException(
                        Name, "Value must be finite and zero or greater.");
                }
            }

            static void Positive(Double Value, String Name)
            {
                if (!Double.IsFinite(Value) || (Value <= 0.0))
                {
                    throw new ArgumentOutOfRangeException(
                        Name, "Value must be finite and greater than zero.");
                }
            }

            NonNegative(Rules.ShortJointRadii, nameof(Rules.ShortJointRadii));
            NonNegative(Rules.ConnectorTieFraction, nameof(Rules.ConnectorTieFraction));
            NonNegative(Rules.ConnectorTieRadii, nameof(Rules.ConnectorTieRadii));
            if (!Double.IsFinite(Rules.SteerHeadingDeg)
                || (Rules.SteerHeadingDeg < 0.0)
                || (Rules.SteerHeadingDeg > Constants.DEGREES_PER_HALF_TURN))
            {
                throw new ArgumentOutOfRangeException(nameof(Rules.SteerHeadingDeg));
            }

            if (Rules.MaxCorrectingLines < 1)
            {
                throw new ArgumentOutOfRangeException(nameof(Rules.MaxCorrectingLines));
            }

            Positive(Rules.LeftoverRatio, nameof(Rules.LeftoverRatio));
            NonNegative(Rules.MergeRouteRadii, nameof(Rules.MergeRouteRadii));
            NonNegative(Rules.EndFragmentSpanRadii, nameof(Rules.EndFragmentSpanRadii));
            NonNegative(Rules.EndFragmentGapRadii, nameof(Rules.EndFragmentGapRadii));
            NonNegative(Rules.NearEmptyRadii, nameof(Rules.NearEmptyRadii));
            if (!Double.IsFinite(Rules.NearEmptyBowlFraction)
                || (Rules.NearEmptyBowlFraction < 0.0)
                || (Rules.NearEmptyBowlFraction > 1.0))
            {
                throw new ArgumentOutOfRangeException(nameof(Rules.NearEmptyBowlFraction));
            }

            Positive(Rules.ArcSpeedMph, nameof(Rules.ArcSpeedMph));
            Positive(Rules.GradeLaneSpeedMph, nameof(Rules.GradeLaneSpeedMph));
        }

        /// <summary>Rejects a ponding setting that is not a finite limit.</summary>
        /// <param name="Options">Ponding settings to check.</param>
        private static void ValidatePondingOptions(PondingOptions Options)
        {
            if (!Double.IsFinite(Options.SpillToleranceFt)
                || Options.SpillToleranceFt < 0.0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.SpillToleranceFt));
            if (!Double.IsFinite(Options.Weight) || Options.Weight < 0.0)
                throw new ArgumentOutOfRangeException(nameof(Options.Weight));
            if (Options.CheckpointInterval < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.CheckpointInterval));
            if (!Double.IsFinite(Options.WarnDepthFt)
                || Options.WarnDepthFt < 0.0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.WarnDepthFt));
            if (!Double.IsFinite(Options.WarnVolumeBcy)
                || Options.WarnVolumeBcy < 0.0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.WarnVolumeBcy));
            if (!Double.IsFinite(Options.DesignRainfallIn)
                || Options.DesignRainfallIn < 0.0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.DesignRainfallIn));
            if (Options.MapPixelsPerCell <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Options.MapPixelsPerCell));
        }

        /// <summary>
        /// Executes the validate optimization options operation.
        /// </summary>
        /// <param name="Options">Configuration controlling the operation.</param>

        private static void ValidateOptimizationOptions
            (
            HaulBuilderOptions Options
            )
        {
            HaulOptimizationOptions Optimization = Options.Optimization;
            if (Optimization.PairRefinementRounds < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Optimization.PairRefinementRounds));
            if (Optimization.RouteReorderWindow < 0)
                throw new ArgumentOutOfRangeException(
                    nameof(Optimization.RouteReorderWindow));
            if (Optimization.ClusterModeAverageWindow < 1)
                throw new ArgumentOutOfRangeException(
                    nameof(Optimization.ClusterModeAverageWindow));
            // Zero keeps the mean from ever leaving hauling. Above one bowl
            // is not a share of capacity.
            if (!Double.IsFinite(Optimization.ClusterModeLoadFraction)
                || (Optimization.ClusterModeLoadFraction < 0.0)
                || (Optimization.ClusterModeLoadFraction > 1.0))
                throw new ArgumentOutOfRangeException(
                    nameof(Optimization.ClusterModeLoadFraction));
            if (Optimization.EnableRouteConsolidation
                && (Optimization.RouteConsolidationGroupSize < 3))
                throw new ArgumentOutOfRangeException(
                    nameof(Optimization.RouteConsolidationGroupSize));
            if (Optimization.EnableRouteConsolidation
                && (Optimization.RouteConsolidationMaximumCycles <= 0))
                throw new ArgumentOutOfRangeException(
                    nameof(Optimization.RouteConsolidationMaximumCycles));
            if (Optimization.EnableMultiRouteExchange)
            {
                if (Optimization.MultiRouteExchangeRounds <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(Optimization.MultiRouteExchangeRounds));
                if (Optimization.MultiRouteExchangeGroupSize < 3)
                    throw new ArgumentOutOfRangeException(
                        nameof(Optimization.MultiRouteExchangeGroupSize));
                if (Optimization.MultiRouteExchangeMaximumGroups <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(Optimization.MultiRouteExchangeMaximumGroups));
                if (!Double.IsFinite(
                        Optimization.MultiRouteExchangeMinimumImprovementM)
                    || (Optimization.MultiRouteExchangeMinimumImprovementM < 0.0))
                    throw new ArgumentOutOfRangeException(
                        nameof(Optimization.MultiRouteExchangeMinimumImprovementM));
            }
            if (Optimization.EnableCycleChainOptimization)
            {
                if (Optimization.CycleChainRounds <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(Optimization.CycleChainRounds));
                if (Optimization.CycleChainCandidatesPerRoute <= 0)
                    throw new ArgumentOutOfRangeException(
                        nameof(Optimization.CycleChainCandidatesPerRoute));
                if (!Double.IsFinite(
                        Optimization.CycleChainMinimumImprovementSeconds)
                    || (Optimization.CycleChainMinimumImprovementSeconds < 0.0))
                    throw new ArgumentOutOfRangeException(
                        nameof(Optimization.CycleChainMinimumImprovementSeconds));
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
