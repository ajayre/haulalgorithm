using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;

using HaulBuilder;

/// <summary>Executable diagnostics for the lane-first haul optimizer.</summary>
internal static class HaulOptimizationBench
{
    /// <summary>Builds and optionally exactly evaluates one optimizer axis.</summary>
    static public Int32 Run
        (
        String[] Args
        )
    {
        String AgdPath = Args.Length > 0
            ? Path.GetFullPath(Args[0])
            : Path.GetFullPath(Path.Combine(
                AppContext.BaseDirectory, "..", "..", "..", "..",
                "TheShop2_2ft.agd"));
        HaulLaneBuilder.LaneAxis Axis = Args.Length > 1
            && String.Equals(Args[1], "adaptive", StringComparison.OrdinalIgnoreCase)
            ? HaulLaneBuilder.LaneAxis.Adaptive
            : Args.Length > 1
                && String.Equals(Args[1], "northsouth", StringComparison.OrdinalIgnoreCase)
                ? HaulLaneBuilder.LaneAxis.NorthSouth
                : HaulLaneBuilder.LaneAxis.EastWest;
        Int32 MaximumRunsPerKind = 2;
        if ((Args.Length > 2)
            && Int32.TryParse(
                Args[2],
                NumberStyles.Integer,
                CultureInfo.InvariantCulture,
                out Int32 ParsedMaximumRuns))
        {
            MaximumRunsPerKind = Math.Max(1, ParsedMaximumRuns);
        }
        Boolean Exact = Args.Any(Arg =>
            String.Equals(Arg, "--exact", StringComparison.OrdinalIgnoreCase));
        Boolean DeferEdgeWork = Args.Any(Arg =>
            String.Equals(Arg, "--defer-edge", StringComparison.OrdinalIgnoreCase));
        Boolean CompleteDeferredEdge = Args.Any(Arg => String.Equals(
            Arg,
            "--complete-deferred-edge",
            StringComparison.OrdinalIgnoreCase));
        Boolean CompactReport = Args.Any(Arg =>
            String.Equals(Arg, "--compact-report", StringComparison.OrdinalIgnoreCase));
        Boolean RepairPairs = Args.Any(Arg =>
            String.Equals(Arg, "--repair-pairs", StringComparison.OrdinalIgnoreCase));
        Boolean SplitFailures = Args.Any(Arg =>
            String.Equals(Arg, "--split-failures", StringComparison.OrdinalIgnoreCase));
        Boolean PairTwoOpt = Args.Any(Arg =>
            String.Equals(Arg, "--pair-2opt", StringComparison.OrdinalIgnoreCase));
        Boolean PairAuction = Args.Any(Arg =>
            String.Equals(Arg, "--pair-auction", StringComparison.OrdinalIgnoreCase));
        Int32 ReorderWindow = ReadIntOption(Args, "--reorder-window=", 200);
        Int32 EdgeMaximumRunsPerKind = Math.Max(
            1,
            ReadIntOption(
                Args,
                "--edge-max-runs=",
                MaximumRunsPerKind));
        Int32 PairTwoOptRounds = ReadIntOption(
            Args, "--pair-2opt-rounds=", 8);
        Int32 PairTwoOptSeed = ReadIntOption(
            Args, "--pair-2opt-seed=", 0);
        Int32 OmitCompactFailures = ReadIntOption(
            Args, "--omit-compact-failures=", 0);
        HaulDifficultCycleFilter.OmissionPriority OmissionPriority =
            Args.Any(Arg => Arg.Equals(
                "--omit-priority=fragmented",
                StringComparison.OrdinalIgnoreCase))
                ? HaulDifficultCycleFilter.OmissionPriority.MostFragmentedFirst
                : HaulDifficultCycleFilter.OmissionPriority.SimplestFirst;
        HaulPairDistanceRefiner.PairMetric PairTwoOptMetric =
            Args.Any(Arg => String.Equals(
                Arg, "--pair-metric=endpoint", StringComparison.OrdinalIgnoreCase))
                ? HaulPairDistanceRefiner.PairMetric.Endpoint
                : Args.Any(Arg => String.Equals(
                    Arg, "--pair-metric=hybrid", StringComparison.OrdinalIgnoreCase))
                    ? HaulPairDistanceRefiner.PairMetric.Hybrid
                    : HaulPairDistanceRefiner.PairMetric.Centroid;
        Double FlowPenaltyWeight = ReadDoubleOption(
            Args, "--flow-weight=", 0.20);
        Double PairAuctionEpsilonM = ReadDoubleOption(
            Args, "--pair-auction-epsilon=", 0.25);
        HashSet<Int32> DiagnosticPasses = ReadIntSetOption(
            Args, "--diagnostic-pass=");
        Int32 RandomRoutePngCount = ReadIntOption(
            Args, "--render-random-routes=", 0);
        Int32 RandomRouteSeed = ReadIntOption(
            Args, "--render-seed=", 20260715);
        String RandomRouteOutputDirectory = ReadStringOption(
            Args,
            "--render-dir=",
            Path.Combine("assets", "optimized_random_routes"));
        String LevelingGifPath = ReadStringOption(
            Args,
            "--render-leveling-gif=",
            String.Empty);
        Int32 LevelingGifPixelsPerBin = Math.Max(
            1,
            ReadIntOption(Args, "--leveling-gif-ppb=", 3));
        Stopwatch Timer = Stopwatch.StartNew();

        HaulBuilderOptions Options = new HaulBuilderOptions
        {
            AgdPath = AgdPath,
            OmitIsolatedEdgePatches = false,
        };
        AgdField Field = Agd.Parse(AgdPath);
        Grid Grid = GridMath.BuildGrid(
            Field,
            Options.GridSizeFt,
            Options.SwellFactor,
            Options.CutFillRatio,
            Options.MaxDepthMm);
        Balance.Compute(Grid, Options.BalanceMode, Double.PositiveInfinity);
        FlowField Flow = FlowFieldSolver.Compute(Grid, Options.TransportMode);
        HaulWorkLedger Ledger = HaulWorkLedger.Build(
            Grid, Options.ToleranceMm);
        Double MaximumLoadBcy = Options.ScraperCapacityLcy
            / Options.SwellFactor;
        HaulOptimizationResult Optimized = HaulCycleOptimizer.Optimize(
            Grid, Flow, Ledger, Axis, MaximumLoadBcy,
            MaximumRunsPerKind,
            DeferEdgeWork,
            FlowPenaltyWeight,
            Obstacles: Obstacles.EMPTY_OBSTACLES);
        List<Pass> CandidatePasses = Optimized.PassResult.Passes;
        HaulPairAuctionOptimizer.Result? PairAuctionResult = null;
        if (PairAuction)
        {
            PairAuctionResult = HaulPairAuctionOptimizer.Optimize(
                CandidatePasses,
                Grid,
                PairAuctionEpsilonM,
                Obstacles: Obstacles.EMPTY_OBSTACLES);
            CandidatePasses = PairAuctionResult.Passes;
        }
        HaulPairDistanceRefiner.Result? PairTwoOptResult = null;
        if (PairTwoOpt)
        {
            PairTwoOptResult = HaulPairDistanceRefiner.Refine(
                CandidatePasses,
                Grid,
                PairTwoOptRounds,
                PairTwoOptMetric,
                PairTwoOptSeed,
                Obstacles: Obstacles.EMPTY_OBSTACLES);
            CandidatePasses = PairTwoOptResult.Passes;
        }
        HaulOptimizationResult? EdgeCompletion = null;
        HaulPairDistanceRefiner.Result? EdgePairTwoOptResult = null;
        HaulPairRepair.Result? EdgePairRepairResult = null;
        HaulDifficultCycleFilter.Result? EdgeOmissionResult = null;
        List<Pass> InteriorCandidatePasses = CandidatePasses;
        List<Pass> EdgeCandidatePasses = new List<Pass>();
        if (CompleteDeferredEdge)
        {
            if (!DeferEdgeWork)
            {
                throw new ArgumentException(
                    "--complete-deferred-edge requires --defer-edge.");
            }
            if (RepairPairs || SplitFailures || (OmitCompactFailures > 0))
            {
                throw new ArgumentException(
                    "Edge completion cannot be combined with whole-plan omission or repair options because its interior route must remain frozen.");
            }
            HaulWorkLedger ResidualLedger =
                Ledger.BuildResidual(CandidatePasses);
            EdgeCompletion = HaulCycleOptimizer.Optimize(
                Grid,
                Flow,
                ResidualLedger,
                Axis,
                MaximumLoadBcy,
                EdgeMaximumRunsPerKind,
                DeferEdgeWork: false,
                FlowPenaltyWeight,
                Obstacles: Obstacles.EMPTY_OBSTACLES);
            EdgeCandidatePasses = EdgeCompletion.PassResult.Passes;
            if (PairTwoOpt)
            {
                EdgePairTwoOptResult = HaulPairDistanceRefiner.Refine(
                    EdgeCandidatePasses,
                    Grid,
                    PairTwoOptRounds,
                    PairTwoOptMetric,
                    PairTwoOptSeed + 1,
                    Obstacles: Obstacles.EMPTY_OBSTACLES);
                EdgeCandidatePasses = EdgePairTwoOptResult.Passes;
            }
            EdgePairRepairResult = HaulPairRepair.Repair(
                EdgeCandidatePasses,
                Grid,
                Options.BladeWidthMm / 1000.0,
                !Options.NoGradeReturn,
                Obstacles: Obstacles.EMPTY_OBSTACLES);
            EdgeCandidatePasses = EdgePairRepairResult.Passes;
            if (EdgePairRepairResult.FinalFailures > 0)
            {
                EdgeOmissionResult = HaulDifficultCycleFilter.Apply(
                    EdgeCandidatePasses,
                    Grid,
                    Options.BladeWidthMm / 1000.0,
                    !Options.NoGradeReturn,
                    EdgePairRepairResult.FinalFailures,
                    HaulDifficultCycleFilter.OmissionPriority.SimplestFirst);
                EdgeCandidatePasses = EdgeOmissionResult.Passes;
            }
            CandidatePasses = CandidatePasses
                .Concat(EdgeCandidatePasses)
                .ToList();
        }
        Int32 OmittedCompactCycles = 0;
        if (OmitCompactFailures > 0)
        {
            HaulDifficultCycleFilter.Result Filter =
                HaulDifficultCycleFilter.Apply(
                    CandidatePasses,
                    Grid,
                    Options.BladeWidthMm / 1000.0,
                    !Options.NoGradeReturn,
                    OmitCompactFailures,
                    OmissionPriority);
            CandidatePasses = Filter.Passes;
            OmittedCompactCycles = Filter.OmittedCycles;
        }
        HaulPairRepair.Result? PairRepair = null;
        if (RepairPairs)
        {
            PairRepair = HaulPairRepair.Repair(
                CandidatePasses,
                Grid,
                Options.BladeWidthMm / 1000.0,
                !Options.NoGradeReturn,
                Obstacles: Obstacles.EMPTY_OBSTACLES);
            CandidatePasses = PairRepair.Passes;
        }
        HaulCycleSplitRepair.Result? SplitRepair = null;
        if (SplitFailures)
        {
            SplitRepair = HaulCycleSplitRepair.Repair(
                CandidatePasses,
                Grid,
                Options.BladeWidthMm / 1000.0,
                !Options.NoGradeReturn,
                Obstacles: Obstacles.EMPTY_OBSTACLES);
            CandidatePasses = SplitRepair.Passes;
        }
        IReadOnlyList<HaulDeferredWork> CandidateDeferred =
            Ledger.BuildDeferredWork(
                CandidatePasses,
                OmittedCompactCycles > 0
                    ? "GUIDANCE_COMPACT_PREFLIGHT"
                    : "OPTIMIZER_UNASSIGNED");
        Double CandidateDeferredCutBcy = CandidateDeferred
            .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
            .Sum(Entry => Entry.VolumeBcy);
        Double CandidateDeferredFillBcy = CandidateDeferred
            .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
            .Sum(Entry => Entry.VolumeBcy);
        GuidancePlanner.CompactCycleResult[] CompactResults =
            Array.Empty<GuidancePlanner.CompactCycleResult>();
        if (CompactReport)
        {
            CompactResults = new GuidancePlanner.CompactCycleResult[
                CandidatePasses.Count];
            Parallel.For(
                0,
                CompactResults.Length,
                Index => CompactResults[Index] =
                    GuidancePlanner.EvaluateCompactCycle(
                        CandidatePasses[Index],
                        Grid,
                        Options.BladeWidthMm / 1000.0,
                        !Options.NoGradeReturn));
        }
        Route Route;
        (Double Before, Double After, Int32 Moves) Refinement;
        if (CompleteDeferredEdge)
        {
            Route InteriorRoute = BuildAndRefineRoute(
                InteriorCandidatePasses,
                ReorderWindow,
                Options,
                out (Double Before, Double After, Int32 Moves)
                    InteriorRefinement);
            Route EdgeRoute = BuildAndRefineRoute(
                EdgeCandidatePasses,
                ReorderWindow,
                Options,
                out (Double Before, Double After, Int32 Moves)
                    EdgeRefinement);
            List<Pass> OrderedPasses = InteriorRoute.OrderedPasses
                .Concat(EdgeRoute.OrderedPasses)
                .ToList();
            List<Int32> Order = InteriorRoute.Order
                .Concat(EdgeRoute.Order.Select(Index =>
                    InteriorCandidatePasses.Count + Index))
                .ToList();
            Route = new Route(
                Order,
                OrderedPasses,
                Routing.CalculateDeadhead(
                    OrderedPasses,
                    !Options.NoGradeReturn,
                    Obstacles.EMPTY_OBSTACLES));
            Refinement = (
                InteriorRefinement.Before + EdgeRefinement.Before,
                InteriorRefinement.After + EdgeRefinement.After,
                InteriorRefinement.Moves + EdgeRefinement.Moves);
        }
        else
        {
            Route = BuildAndRefineRoute(
                CandidatePasses,
                ReorderWindow,
                Options,
                out Refinement);
        }
        TimeParams Parameters = new TimeParams(
            Options.LoadingSpeedMph,
            Options.HaulSpeedMph,
            Options.DumpingSpeedMph,
            Options.ReturnSpeedMph,
            Options.AccelTimeS,
            Options.DecelTimeS,
            Options.FieldEfficiency);
        TimeResult ProxyTime = Timemodel.EstimateTime(
            Grid, Route, Parameters);

        Console.WriteLine("HAUL OPTIMIZATION PROBE");
        Console.WriteLine("agd={0}", AgdPath);
        Console.WriteLine("axis={0}", Axis);
        Console.WriteLine("maximum_runs_per_kind={0}", MaximumRunsPerKind);
        Console.WriteLine("defer_edge_work={0}", DeferEdgeWork);
        Console.WriteLine("complete_deferred_edge={0}", CompleteDeferredEdge);
        Console.WriteLine("edge_maximum_runs_per_kind={0}", EdgeMaximumRunsPerKind);
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "flow_penalty_weight={0:F3}",
            FlowPenaltyWeight));
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "reorder_window={0} reorder_moves={1} reorder_before_s={2:F3} reorder_after_s={3:F3}",
            ReorderWindow,
            Refinement.Moves,
            Refinement.Before,
            Refinement.After));
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "required_cut_bcy={0:F6} required_fill_bcy={1:F6} imbalance_bcy={2:F6}",
            Ledger.TotalCutBcy,
            Ledger.TotalFillBcy,
            Ledger.TotalCutBcy - Ledger.TotalFillBcy));
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "passes={0} cut_lanes={1} fill_lanes={2} assigned_bcy={3:F6} unmatched_cut_bcy={4:F9} unmatched_fill_bcy={5:F9}",
            Optimized.PassResult.Passes.Count,
            Optimized.CutLaneCount,
            Optimized.FillLaneCount,
            Optimized.TotalAssignedBcy,
            Optimized.UnmatchedCutBcy,
            Optimized.UnmatchedFillBcy));
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "deferred_entries={0} deferred_cut_percent={1:F3} deferred_fill_percent={2:F3} deferred_work_lower_bound_hours={3:F3}",
            Optimized.DeferredWork.Count,
            100.0 * Optimized.UnmatchedCutBcy / Ledger.TotalCutBcy,
            100.0 * Optimized.UnmatchedFillBcy / Ledger.TotalFillBcy,
            DeferredWorkLowerBoundHours(
                Optimized.DeferredWork,
                Grid,
                Options)));
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "validation={0} max_cut_error_m={1:R} max_fill_error_m={2:R} issue_count={3}",
            Optimized.Validation.Success,
            Optimized.Validation.MaximumCutErrorM,
            Optimized.Validation.MaximumFillErrorM,
            Optimized.Validation.Issues.Count));
        if (PairRepair != null)
        {
            HaulPlanValidation RepairValidation = Ledger.ValidateComplete(
                CandidatePasses,
                MaximumLoadBcy,
                Grid.MaxDepthMm / 1000.0,
                AllowDeferredWork: true);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "pair_repair_swaps={0} pair_repair_attempts={1} pair_repair_failures_before={2} pair_repair_failures_after={3} pair_repair_validation={4}",
                PairRepair.SwapCount,
                PairRepair.CandidateAttempts,
                PairRepair.InitialFailures,
                PairRepair.FinalFailures,
                RepairValidation.Success));
        }
        if (PairTwoOptResult != null)
        {
            HaulPlanValidation PairTwoOptValidation = Ledger.ValidateComplete(
                CandidatePasses,
                MaximumLoadBcy,
                Grid.MaxDepthMm / 1000.0,
                AllowDeferredWork: true);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "pair_2opt_swaps={0} pair_2opt_before_m={1:F1} pair_2opt_after_m={2:F1} pair_2opt_validation={3}",
                PairTwoOptResult.Swaps,
                PairTwoOptResult.BeforeM,
                PairTwoOptResult.AfterM,
                PairTwoOptValidation.Success));
        }
        if (EdgeCompletion != null)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "edge_completion_passes={0} edge_assigned_bcy={1:F6} edge_unmatched_cut_bcy={2:F9} edge_unmatched_fill_bcy={3:F9}",
                EdgeCompletion.PassResult.Passes.Count,
                EdgeCompletion.TotalAssignedBcy,
                EdgeCompletion.UnmatchedCutBcy,
                EdgeCompletion.UnmatchedFillBcy));
        }
        if (EdgePairTwoOptResult != null)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "edge_pair_2opt_swaps={0} edge_pair_2opt_before_m={1:F1} edge_pair_2opt_after_m={2:F1}",
                EdgePairTwoOptResult.Swaps,
                EdgePairTwoOptResult.BeforeM,
                EdgePairTwoOptResult.AfterM));
        }
        if (EdgePairRepairResult != null)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "edge_pair_repair_swaps={0} edge_pair_repair_attempts={1} edge_pair_repair_failures_before={2} edge_pair_repair_failures_after={3}",
                EdgePairRepairResult.SwapCount,
                EdgePairRepairResult.CandidateAttempts,
                EdgePairRepairResult.InitialFailures,
                EdgePairRepairResult.FinalFailures));
        }
        if (EdgeOmissionResult != null)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "edge_compact_failures={0} edge_omitted_cycles={1}",
                EdgeOmissionResult.FailureCount,
                EdgeOmissionResult.OmittedCycles));
        }
        if (CompleteDeferredEdge)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "final_deferred_cut_bcy={0:F6} final_deferred_fill_bcy={1:F6} final_deferred_cut_percent={2:F4} final_deferred_fill_percent={3:F4}",
                CandidateDeferredCutBcy,
                CandidateDeferredFillBcy,
                100.0 * CandidateDeferredCutBcy / Ledger.TotalCutBcy,
                100.0 * CandidateDeferredFillBcy / Ledger.TotalFillBcy));
        }
        if (PairAuctionResult != null)
        {
            HaulPlanValidation PairAuctionValidation = Ledger.ValidateComplete(
                CandidatePasses,
                MaximumLoadBcy,
                Grid.MaxDepthMm / 1000.0,
                AllowDeferredWork: true);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "pair_auction_bids={0} pair_auction_before_m={1:F1} pair_auction_after_m={2:F1} pair_auction_validation={3}",
                PairAuctionResult.BidCount,
                PairAuctionResult.BeforeM,
                PairAuctionResult.AfterM,
                PairAuctionValidation.Success));
        }
        if (SplitRepair != null)
        {
            HaulPlanValidation SplitValidation = Ledger.ValidateComplete(
                CandidatePasses,
                MaximumLoadBcy,
                Grid.MaxDepthMm / 1000.0,
                AllowDeferredWork: true);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "split_repair_cycles={0} split_repair_added_cycles={1} split_repair_failures_before={2} split_repair_failures_after={3} split_repair_validation={4}",
                SplitRepair.SplitCycles,
                SplitRepair.AddedCycles,
                SplitRepair.InitialFailures,
                SplitRepair.FinalFailures,
                SplitValidation.Success));
        }
        if (OmittedCompactCycles > 0)
        {
            HaulPlanValidation OmissionValidation = Ledger.ValidateComplete(
                CandidatePasses,
                MaximumLoadBcy,
                Grid.MaxDepthMm / 1000.0,
                AllowDeferredWork: true);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "compact_omitted_cycles={0} compact_omission_priority={1} compact_deferred_cut_bcy={2:F3} compact_deferred_fill_bcy={3:F3} compact_deferred_cut_percent={4:F3} compact_deferred_fill_percent={5:F3} compact_deferred_lower_bound_hours={6:F3} compact_omission_validation={7}",
                OmittedCompactCycles,
                OmissionPriority,
                CandidateDeferredCutBcy,
                CandidateDeferredFillBcy,
                100.0 * CandidateDeferredCutBcy / Ledger.TotalCutBcy,
                100.0 * CandidateDeferredFillBcy / Ledger.TotalFillBcy,
                DeferredWorkLowerBoundHours(
                    CandidateDeferred,
                    Grid,
                    Options),
                OmissionValidation.Success));
        }
        foreach (String Issue in Optimized.Validation.Issues.Take(20))
        {
            Console.WriteLine("validation_issue={0}", Issue);
        }
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "proxy_hours={0:F3} proxy_haul_m={1:F1} proxy_deadhead_m={2:F1} elapsed_s={3:F2}",
            ProxyTime.TotalTimeS / 3600.0,
            ProxyTime.HaulingDistM,
            Route.TotalDeadheadM,
            Timer.Elapsed.TotalSeconds));
        if (CompactResults.Length > 0)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "compact_success={0} compact_failure={1} compact_initial_runs={2}",
                CompactResults.Count(Result => Result.Success),
                CompactResults.Count(Result => !Result.Success),
                CompactResults.Sum(Result =>
                    Result.CutRunCount + Result.FillRunCount)));
            foreach (IGrouping<String, GuidancePlanner.CompactCycleResult> Group
                in CompactResults.Where(Result => !Result.Success)
                    .GroupBy(Result => Result.FailureReason)
                    .OrderByDescending(Group => Group.Count()))
            {
                Console.WriteLine(
                    "compact_failure_reason={0} count={1}",
                    Group.Key,
                    Group.Count());
            }
        }

        if (!String.IsNullOrWhiteSpace(LevelingGifPath))
        {
            String GifPath = Path.GetFullPath(LevelingGifPath);
            String? GifDirectory = Path.GetDirectoryName(GifPath);
            if (!String.IsNullOrEmpty(GifDirectory))
            {
                Directory.CreateDirectory(GifDirectory);
            }
            Animate.AnimationPayload Animation =
                Animate.AnimateLevelingSplitView(
                    Grid,
                    Route.OrderedPasses,
                    new Animate.SplitLevelingOptions
                    {
                        PixelsPerBin = LevelingGifPixelsPerBin,
                        TargetFrames = 100,
                        EarlyPerPass = 0,
                        GifMs = 100,
                        Title = "The Shop 2 - optimized leveling progress",
                    });
            if (Animation.GifBytes == null)
            {
                throw new InvalidOperationException(
                    "Leveling animation did not produce GIF bytes.");
            }
            File.WriteAllBytes(GifPath, Animation.GifBytes);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "leveling_gif={0} frames={1} leveling_seconds=10.0 final_hold_seconds=4.0 total_seconds=14.0 bytes={2}",
                GifPath,
                Animation.FramePngBytes.Count,
                Animation.GifBytes.Length));
        }

        if (!Exact || !Optimized.Validation.Success)
        {
            return Optimized.Validation.Success ? 0 : 1;
        }

        GuidanceCostOracle.Result ExactResult =
            GuidanceCostOracle.EvaluatePlan(
                Route,
                Grid,
                Field,
                Passes.EstimateSlopeHeading(Grid),
                Options,
                (Stage, Completed, Total) =>
                {
                    if ((Completed == Total) || (Completed % 100 == 0))
                    {
                        Console.WriteLine(
                            "progress stage={0} completed={1} total={2} elapsed={3}",
                            Stage, Completed, Total, Timer.Elapsed);
                    }
                },
                Message =>
                {
                    if (Message.Contains("sequence_failure", StringComparison.Ordinal)
                        || Message.Contains("pass_timing", StringComparison.Ordinal)
                        || Message.Contains("timing total", StringComparison.Ordinal)
                        || DiagnosticPasses.Any(PassNumber =>
                            ContainsPassToken(Message, PassNumber)))
                    {
                        Console.WriteLine("diagnostic {0}", Message);
                    }
                });
        if (!ExactResult.Success)
        {
            Console.WriteLine("exact_success=False");
            Console.WriteLine("exact_failure={0}", ExactResult.FailureReason);
            return 1;
        }

        GuidanceTiming.Result Timing = ExactResult.Plan!.Timing;
        Console.WriteLine("exact_success=True");
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "exact_deterministic_hours={0:F3} exact_scheduled_hours={1:F3} exact_distance_m={2:F1} exact_segments={3} manual_assists={4} acceleration_overhead_hours={5:F3} motion_blocks={6} curvature_limited_m={7:F1} elapsed_minutes={8:F2}",
            ExactResult.DeterministicSeconds / 3600.0,
            ExactResult.ScheduledSeconds / 3600.0,
            ExactResult.DistanceM,
            ExactResult.Plan.Lifecycle.Segments.Count,
            ExactResult.Plan.Lifecycle.ManualAssistCount,
            Timing.AccelerationConstraintSeconds / 3600.0,
            Timing.MotionBlockCount,
            Timing.CurvatureLimitedDistanceM,
            Timer.Elapsed.TotalMinutes));
        foreach (LifecyclePlanner.Phase Phase in Timing.PhaseSeconds.Keys.OrderBy(Value => Value))
        {
            Timing.PhaseDistanceM.TryGetValue(Phase, out Double DistanceM);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "phase={0} hours={1:F4} distance_m={2:F1}",
                Phase,
                Timing.PhaseSeconds[Phase] / 3600.0,
                DistanceM));
        }
        foreach (KeyValuePair<Int32, Double> RouteTime in Timing.RouteSeconds
            .OrderByDescending(Pair => Pair.Value)
            .Take(20))
        {
            Timing.RouteDistanceM.TryGetValue(
                RouteTime.Key, out Double RouteDistanceM);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "hot_route={0} hours={1:F4} distance_m={2:F1}",
                RouteTime.Key,
                RouteTime.Value / 3600.0,
                RouteDistanceM));
        }
        FieldEdgePolicy ExactEdgePolicy = new FieldEdgePolicy(Grid, Grid.GridSizeM);
        foreach (Int32 PassNumber in DiagnosticPasses
            .Where(PassNumber => PassNumber >= 1
                && PassNumber <= ExactResult.Plan.Passes.Count)
            .OrderBy(PassNumber => PassNumber))
        {
            LifecyclePlanner.PassPlan Pass =
                ExactResult.Plan.Passes[PassNumber - 1];
            Console.WriteLine(
                "exact_route pass={0} cut_runs={1} fill_runs={2}",
                PassNumber,
                Pass.Cut.Runs.Count,
                Pass.Fill.Runs.Count);
            for (Int32 RunIndex = 0;
                RunIndex < Pass.Fill.Runs.Count;
                RunIndex += 1)
            {
                CoveragePlanner.Run Run = Pass.Fill.Runs[RunIndex].Run;
                Int32 MinimumBx = Run.Cells.Min(Entry => Entry.Cell.Bx);
                Int32 MaximumBx = Run.Cells.Max(Entry => Entry.Cell.Bx);
                Int32 MinimumBy = Run.Cells.Min(Entry => Entry.Cell.By);
                Int32 MaximumBy = Run.Cells.Max(Entry => Entry.Cell.By);
                List<Double> EdgeHeadings = Run.Cells
                    .Select(Entry => ExactEdgePolicy.TryGetParallelHeadingDeg(
                        Entry.Cell, out Double EdgeHeadingDeg)
                            ? (Double?)EdgeHeadingDeg
                            : null)
                    .Where(HeadingDeg => HeadingDeg.HasValue)
                    .Select(HeadingDeg => HeadingDeg!.Value)
                    .ToList();
                Console.WriteLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "exact_route_fill_run pass={0} run={1} cells={2} heading={3:F2} length_m={4:F2} bounds=({5},{6})-({7},{8}) edge_headings={9}",
                    PassNumber,
                    RunIndex,
                    Run.Cells.Count,
                    Run.HeadingDeg,
                    Run.LengthM,
                    MinimumBx,
                    MinimumBy,
                    MaximumBx,
                    MaximumBy,
                    EdgeHeadings.Count == 0
                        ? "none"
                        : String.Join(",", EdgeHeadings
                            .Select(HeadingDeg => HeadingDeg.ToString(
                                "F1", CultureInfo.InvariantCulture)))));
            }
        }
        if (RandomRoutePngCount > 0)
        {
            RenderRandomExactRoutes(
                ExactResult.Plan,
                Grid,
                Options.BladeWidthMm / 1000.0,
                Math.Min(RandomRoutePngCount, ExactResult.Plan.Passes.Count),
                RandomRouteSeed,
                Path.GetFullPath(RandomRouteOutputDirectory));
        }
        Double ExactHours = ExactResult.DeterministicSeconds / 3600.0;
        Double SavingsPercent = 100.0 * (433.483 - ExactHours) / 433.483;
        Boolean GatePassed = ExactHours <= 216.742;
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "comparison baseline_h=433.483 best_h={0:F3} savings_percent={1:F3} distance_m={2:F1} passes={3} deferred_cut_bcy={4:F3} exact_status=PASS gate_status={5}",
            ExactHours,
            SavingsPercent,
            ExactResult.DistanceM,
            Route.OrderedPasses.Count,
            CandidateDeferredCutBcy,
            GatePassed ? "PASS" : "FAIL"));
        return GatePassed
            ? 0 : 2;
    }

    /// <summary>Sequences and locally refines one independently frozen pass batch.</summary>
    private static Route BuildAndRefineRoute
        (
        List<Pass> Passes,
        Int32 ReorderWindow,
        HaulBuilderOptions Options,
        out (Double Before, Double After, Int32 Moves) Refinement
        )
    {
        Route Route = Routing.SequencePasses(
            Passes,
            1500,
            !Options.NoGradeReturn,
            !Options.NoDirectionAware,
            Options.ReturnSpeedMph,
            Options.TurnSpeedMph,
            Options.TurningCircleFt,
            Obstacles.EMPTY_OBSTACLES);
        Refinement = Routing.RefineRouteOrder(
            Route,
            ReorderWindow,
            !Options.NoGradeReturn,
            !Options.NoDirectionAware,
            Options.ReturnSpeedMph,
            Options.TurnSpeedMph,
            Options.TurningCircleFt,
            20,
            Obstacles.EMPTY_OBSTACLES);
        return Route;
    }

    /// <summary>Renders a deterministic random sample from the final exact lifecycle.</summary>
    private static void RenderRandomExactRoutes
        (
        GuidancePlanner.BuildResult Plan,
        Grid Grid,
        Double BladeWidthM,
        Int32 Count,
        Int32 Seed,
        String OutputDirectory
        )
    {
        Directory.CreateDirectory(OutputDirectory);
        Random Random = new Random(Seed);
        HashSet<Int32> Selected = new HashSet<Int32>();
        while (Selected.Count < Count)
        {
            Selected.Add(Random.Next(1, Plan.Passes.Count + 1));
        }
        foreach (Int32 RouteNumber in Selected.OrderBy(Value => Value))
        {
            List<LifecyclePlanner.Segment> Segments = Plan.Lifecycle.Segments
                .Where(Segment => Segment.RouteNumber == RouteNumber)
                .ToList();
            List<IReadOnlyList<(Double X, Double Y)>> CutRuns = Segments
                .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.Cut)
                .Select(Segment =>
                    (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
                .ToList();
            List<IReadOnlyList<(Double X, Double Y)>> FillRuns = Segments
                .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.Fill)
                .Select(Segment =>
                    (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
                .ToList();
            List<IReadOnlyList<(Double X, Double Y)>> ManualRuns = Segments
                .Where(Segment =>
                    Segment.Phase == LifecyclePlanner.Phase.ManualAssist)
                .Select(Segment =>
                    (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
                .ToList();
            List<(Double X, Double Y)> RoutePoints = Segments
                .SelectMany(Segment => Segment.Points)
                .ToList();
            if (RoutePoints.Count == 0) continue;
            (Double X, Double Y) MarkerPoint = CutRuns.Count > 0
                && CutRuns[0].Count > 0
                ? CutRuns[0][CutRuns[0].Count / 2]
                : RoutePoints[0];
            Byte[] Png = Render.RenderCoverageRunsPng(
                Grid,
                CutRuns,
                FillRuns,
                RoutePoints,
                BladeWidthM,
                pixelsPerBin: 40,
                padCells: 8,
                manualAssistRuns: ManualRuns,
                routeMarkers: new List<((Double X, Double Y) Point,
                    String Label)>
                {
                    (MarkerPoint, RouteNumber.ToString(
                        CultureInfo.InvariantCulture)),
                },
                drawDirectionArrows: true,
                directionArrowSpacingM: 6.0 * Grid.GridSizeM);
            String OutputPath = Path.Combine(
                OutputDirectory,
                String.Format(
                    CultureInfo.InvariantCulture,
                    "optimized_route_{0:D4}.png",
                    RouteNumber));
            File.WriteAllBytes(OutputPath, Png);
            Console.WriteLine(
                "random_route_png route={0} seed={1} path={2}",
                RouteNumber,
                Seed,
                OutputPath);
        }
    }

    /// <summary>Reads a comma-separated integer set from one command-line option.</summary>
    private static HashSet<Int32> ReadIntSetOption
        (
        IEnumerable<String> Args,
        String Prefix
        )
    {
        String? Value = Args.FirstOrDefault(Arg =>
            Arg.StartsWith(Prefix, StringComparison.OrdinalIgnoreCase));
        if (Value == null) return new HashSet<Int32>();
        return Value.Substring(Prefix.Length)
            .Split(',', StringSplitOptions.RemoveEmptyEntries
                | StringSplitOptions.TrimEntries)
            .Select(Part => Int32.TryParse(
                Part,
                NumberStyles.Integer,
                CultureInfo.InvariantCulture,
                out Int32 Parsed)
                    ? Parsed
                    : 0)
            .Where(Parsed => Parsed > 0)
            .ToHashSet();
    }

    /// <summary>Reads one string option without locale-dependent conversion.</summary>
    private static String ReadStringOption
        (
        IEnumerable<String> Args,
        String Prefix,
        String DefaultValue
        )
    {
        String? Value = Args.FirstOrDefault(Arg =>
            Arg.StartsWith(Prefix, StringComparison.OrdinalIgnoreCase));
        return Value == null || Value.Length == Prefix.Length
            ? DefaultValue
            : Value.Substring(Prefix.Length);
    }

    /// <summary>Tests one diagnostic for an exact whole-plan pass token.</summary>
    private static Boolean ContainsPassToken(String Message, Int32 PassNumber)
    {
        String Token = String.Format(
            CultureInfo.InvariantCulture,
            "pass={0}",
            PassNumber);
        Int32 Start = Message.IndexOf(Token, StringComparison.Ordinal);
        if (Start < 0) return false;
        Int32 End = Start + Token.Length;
        return (End == Message.Length) || !Char.IsDigit(Message[End]);
    }

    private static Double DeferredWorkLowerBoundHours
        (
        IReadOnlyList<HaulDeferredWork> Deferred,
        Grid Grid,
        HaulBuilderOptions Options
        )
    {
        Double CutSeconds = Deferred.Count(Entry =>
                Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
            * Grid.GridSizeM
            / (Options.LoadingSpeedMph * Constants.MPH_TO_MPS);
        Double FillDistanceM = Deferred.Count(Entry =>
                Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
            * Grid.GridSizeM;
        Double FillSeconds = FillDistanceM
            / (Options.DumpingSpeedMph * Constants.MPH_TO_MPS);
        Double GradeSeconds = FillDistanceM
            / (Options.GradeSpeedMph * Constants.MPH_TO_MPS);
        return (CutSeconds + FillSeconds + GradeSeconds) / 3600.0;
    }

    private static Int32 ReadIntOption
        (
        IEnumerable<String> Args,
        String Prefix,
        Int32 DefaultValue
        )
    {
        String? Text = Args.FirstOrDefault(Arg =>
            Arg.StartsWith(Prefix, StringComparison.OrdinalIgnoreCase));
        return (Text != null)
            && Int32.TryParse(
                Text.Substring(Prefix.Length),
                NumberStyles.Integer,
                CultureInfo.InvariantCulture,
                out Int32 Value)
            ? Math.Max(0, Value)
            : DefaultValue;
    }

    private static Double ReadDoubleOption
        (
        IEnumerable<String> Args,
        String Prefix,
        Double DefaultValue
        )
    {
        String? Text = Args.FirstOrDefault(Arg =>
            Arg.StartsWith(Prefix, StringComparison.OrdinalIgnoreCase));
        return (Text != null)
            && Double.TryParse(
                Text.Substring(Prefix.Length),
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out Double Value)
            ? Math.Max(0.0, Value)
            : DefaultValue;
    }
}
