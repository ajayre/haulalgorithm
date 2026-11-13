using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

using HaulBuilder;

/// <summary>
/// Builds one lane-first production haul plan, then audits and optimizes copies
/// of its route so ponding risk and route-time effects can be compared without
/// rebuilding expensive downstream guidance twice.
/// </summary>
internal static class FastPondingProof
{
    /// <summary>Runs the full-field ponding proof.</summary>
    /// <param name="Args">AGD, output directory, interval, window, and weight.</param>
    /// <returns>Zero when the comparison completes.</returns>
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
        String OutputDirectory = Args.Length > 1
            ? Path.GetFullPath(Args[1])
            : Path.GetFullPath(Path.Combine(
                AppContext.BaseDirectory, "..", "..", "..", "..", "..",
                "assets", "ponding_proof"));
        Int32 Interval = Args.Length > 2
            ? Int32.Parse(Args[2], CultureInfo.InvariantCulture)
            : 25;
        Int32 Window = Args.Length > 3
            ? Int32.Parse(Args[3], CultureInfo.InvariantCulture)
            : 5;
        Double Weight = Args.Length > 4
            ? Double.Parse(Args[4], CultureInfo.InvariantCulture)
            : 3600.0;
        PondingObjectiveMode ObjectiveMode = Args.Length > 5
            && String.Equals(Args[5], "average",
                StringComparison.OrdinalIgnoreCase)
            ? PondingObjectiveMode.AverageDepth
            : PondingObjectiveMode.PeakVolume;
        Boolean SkipGuidance = Args.Length > 6
            && String.Equals(Args[6], "no-guidance",
                StringComparison.OrdinalIgnoreCase);
        Directory.CreateDirectory(OutputDirectory);

        // Reuse one planning pass for audit, optimisation, and optional guidance.
        var State = BuildComparisonState(
            AgdPath, Interval, Window, Weight, ObjectiveMode);
        Grid Grid = State.Grid;
        PassResult PassResult = State.PassResult;
        Route AuditRoute = State.AuditRoute;
        Route OptimizedRoute = State.OptimizedRoute;
        PondingRunResult Audit = State.Audit;
        PondingRunResult Optimized = State.Optimized;
        TimeSpan PlanningElapsed = State.PlanningElapsed;
        TimeSpan AuditElapsed = State.AuditElapsed;
        TimeSpan OptimizeElapsed = State.OptimizeElapsed;
        HaulBuilderOptions ProductionOptions = new() { AgdPath = AgdPath };
        ProductionOptions.Optimization.RouteReorderWindow = Window;
        AgdField Field = Agd.Parse(AgdPath);

        Double HeadingDeg = HaulLaneBuilder.NormalizeAxisHeadingDeg(
            ProductionOptions.LaneAxisHeadingDeg);
        Double AuditDeterministicHours = 0.0;
        Double AuditScheduledHours = 0.0;
        Double AuditGuidanceDistanceM = 0.0;
        TimeSpan AuditGuidanceWatch = TimeSpan.Zero;
        if (!SkipGuidance)
        {
            Stopwatch GuidanceWatch = Stopwatch.StartNew();
            GuidanceCostOracle.Result AuditGuidance =
                GuidanceCostOracle.EvaluatePlan(
                    AuditRoute,
                    Grid,
                    Field,
                    HeadingDeg,
                    ProductionOptions);
            GuidanceWatch.Stop();
            if (!AuditGuidance.Success)
                throw new InvalidOperationException(
                    "Audit exact guidance failed: "
                    + AuditGuidance.FailureReason);
            AuditDeterministicHours = AuditGuidance.DeterministicSeconds
                / Constants.SECONDS_PER_HOUR;
            AuditScheduledHours = AuditGuidance.ScheduledSeconds
                / Constants.SECONDS_PER_HOUR;
            AuditGuidanceDistanceM = AuditGuidance.DistanceM;
            AuditGuidanceWatch = GuidanceWatch.Elapsed;
        }

        Double OptimizedDeterministicHours = 0.0;
        Double OptimizedScheduledHours = 0.0;
        Double OptimizedGuidanceDistanceM = 0.0;
        TimeSpan OptimizedGuidanceWatch = TimeSpan.Zero;
        if (!SkipGuidance)
        {
            Stopwatch GuidanceWatch = Stopwatch.StartNew();
            GuidanceCostOracle.Result OptimizedGuidance =
                GuidanceCostOracle.EvaluatePlan(
                    OptimizedRoute,
                    Grid,
                    Field,
                    HeadingDeg,
                    ProductionOptions);
            GuidanceWatch.Stop();
            if (!OptimizedGuidance.Success)
                throw new InvalidOperationException(
                    "Optimized exact guidance failed: "
                    + OptimizedGuidance.FailureReason);
            OptimizedDeterministicHours =
                OptimizedGuidance.DeterministicSeconds
                / Constants.SECONDS_PER_HOUR;
            OptimizedScheduledHours = OptimizedGuidance.ScheduledSeconds
                / Constants.SECONDS_PER_HOUR;
            OptimizedGuidanceDistanceM = OptimizedGuidance.DistanceM;
            OptimizedGuidanceWatch = GuidanceWatch.Elapsed;
        }
        Int32 ChangedPositions = CountChangedPositions(
            AuditRoute, OptimizedRoute);
        Double FinalSurfaceDifference = FinalSurfaceDifferenceM(
            Grid, AuditRoute, OptimizedRoute);
        Double MovedBcy = PassResult.Passes.Sum(Pass => Pass.LoadBcy);

        WriteArtifacts(OutputDirectory, "audit", Grid, Audit);
        WriteArtifacts(OutputDirectory, "optimized", Grid, Optimized);
        WriteComparison(
            OutputDirectory,
            Grid,
            PassResult.Passes.Count,
            MovedBcy,
            PlanningElapsed,
            Audit,
            AuditElapsed,
            AuditDeterministicHours,
            AuditScheduledHours,
            AuditGuidanceDistanceM,
            AuditGuidanceWatch,
            Optimized,
            OptimizeElapsed,
            OptimizedDeterministicHours,
            OptimizedScheduledHours,
            OptimizedGuidanceDistanceM,
            OptimizedGuidanceWatch,
            ChangedPositions,
            FinalSurfaceDifference,
            Interval,
            Window,
            Weight,
            ObjectiveMode);
        PrintSummary(
            PassResult.Passes.Count,
            MovedBcy,
            PlanningElapsed,
            Audit,
            AuditElapsed,
            AuditDeterministicHours,
            AuditScheduledHours,
            AuditGuidanceDistanceM,
            AuditGuidanceWatch,
            Optimized,
            OptimizeElapsed,
            OptimizedDeterministicHours,
            OptimizedScheduledHours,
            OptimizedGuidanceDistanceM,
            OptimizedGuidanceWatch,
            ChangedPositions,
            FinalSurfaceDifference,
            ObjectiveMode);
        return 0;
    }

    /// <summary>Builds baseline and optimized routes for ponding comparisons.</summary>
    /// <param name="AgdPath">AGD input path.</param>
    /// <param name="Interval">Ponding checkpoint interval.</param>
    /// <param name="Window">Relocation window.</param>
    /// <param name="Weight">Combined objective weight.</param>
    /// <param name="ObjectiveMode">Active ponding penalty.</param>
    /// <returns>Planning grid, passes, routes, and ponding audit results.</returns>
    static public (Grid Grid, PassResult PassResult, Route AuditRoute,
        Route OptimizedRoute, PondingRunResult Audit, PondingRunResult Optimized,
        TimeSpan PlanningElapsed, TimeSpan AuditElapsed, TimeSpan OptimizeElapsed)
        BuildComparisonState(
            String AgdPath,
            Int32 Interval,
            Int32 Window,
            Double Weight,
            PondingObjectiveMode ObjectiveMode)
    {
        Stopwatch PlanningWatch = Stopwatch.StartNew();
        HaulBuilderOptions ProductionOptions = new() { AgdPath = AgdPath };
        ProductionOptions.Optimization.RouteReorderWindow = Window;
        AgdField Field = Agd.Parse(AgdPath);
        Grid Grid = GridMath.BuildGrid(
            Field,
            ProductionOptions.GridSizeFt,
            ProductionOptions.SwellFactor,
            ProductionOptions.CutFillRatio,
            ProductionOptions.MaxDepthMm);
        Balance.Compute(Grid, "check", Double.PositiveInfinity);
        FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
        PassResult PassResult = BuildProductionPasses(
            Grid, Flow, ProductionOptions, Obstacles.EMPTY_OBSTACLES);
        Route BaseRoute = Routing.SequencePasses(
            PassResult.Passes,
            1500,
            true,
            true,
            ProductionOptions.UnloadedTravelSpeedMph,
            ProductionOptions.TurnSpeedMph,
            ProductionOptions.MinimumTurningDiameterFt,
            Obstacles.EMPTY_OBSTACLES,
            StartCandidateCount: 8);
        Routing.RefineRouteOrder(
            BaseRoute,
            Window,
            true,
            true,
            ProductionOptions.UnloadedTravelSpeedMph,
            ProductionOptions.TurnSpeedMph,
            ProductionOptions.MinimumTurningDiameterFt,
            12,
            Obstacles.EMPTY_OBSTACLES);
        PlanningWatch.Stop();

        Route AuditRoute = CloneRoute(BaseRoute);
        PondingOptions AuditOptions = BuildOptions(
            Optimize: false, Interval, Weight, ObjectiveMode);
        Stopwatch AuditWatch = Stopwatch.StartNew();
        PondingRunResult Audit = Ponding.Run(
            Grid,
            PassResult.Passes,
            AuditRoute,
            AuditOptions,
            Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
            Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT,
            Obstacles.EMPTY_OBSTACLES,
            Window);
        AuditWatch.Stop();

        Route OptimizedRoute = CloneRoute(BaseRoute);
        PondingOptions OptimizedOptions = BuildOptions(
            Optimize: true, Interval, Weight, ObjectiveMode);
        Stopwatch OptimizeWatch = Stopwatch.StartNew();
        PondingRunResult Optimized = Ponding.Run(
            Grid,
            PassResult.Passes,
            OptimizedRoute,
            OptimizedOptions,
            Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
            Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT,
            Obstacles.EMPTY_OBSTACLES,
            Window);
        OptimizeWatch.Stop();
        return (Grid, PassResult, AuditRoute, OptimizedRoute, Audit, Optimized,
            PlanningWatch.Elapsed, AuditWatch.Elapsed, OptimizeWatch.Elapsed);
    }

    /// <summary>
    /// Runs the production lane-first allocation stages through consolidation.
    /// The optional exact-guidance-gated cycle-chain stage is intentionally not
    /// run because this proof isolates the ponding sequencer before guidance.
    /// </summary>
    /// <param name="Grid">Balanced planning grid.</param>
    /// <param name="Flow">Solved drainage field.</param>
    /// <param name="Options">Production planning options.</param>
    /// <param name="Obstacles">Resolved obstacle field.</param>
    /// <returns>Validated production passes.</returns>
    static private PassResult BuildProductionPasses
        (
        Grid Grid,
        FlowField Flow,
        HaulBuilderOptions Options,
        ObstacleField Obstacles
        )
    {
        Double HeadingDeg = HaulLaneBuilder.NormalizeAxisHeadingDeg(
            Options.LaneAxisHeadingDeg);
        Double ScraperBcy = Options.ScraperCapacityLcy / Options.SwellFactor;
        HaulWorkLedger Ledger = HaulWorkLedger.Build(
            Grid, Options.ToleranceMm);
        HaulOptimizationResult Optimized = HaulCycleOptimizer.Optimize(
            Grid,
            Flow,
            Ledger,
            HaulLaneBuilder.LaneAxis.UserHeading,
            ScraperBcy,
            Options.Optimization.MaxLaneSlicesPerCycle,
            DeferEdgeWork: true,
            Options.Optimization.FlowPenaltyWeight,
            Obstacles,
            HeadingDeg);
        List<Pass> OptimizedPasses = Optimized.PassResult.Passes;
        if (Options.Optimization.PairRefinementRounds > 0)
        {
            HaulPairDistanceRefiner.Result PairRefinement =
                HaulPairDistanceRefiner.Refine(
                    OptimizedPasses,
                    Grid,
                    Options.Optimization.PairRefinementRounds,
                    HaulPairDistanceRefiner.PairMetric.Centroid,
                    Seed: 0,
                    Obstacles: Obstacles);
            OptimizedPasses = PairRefinement.Passes;
        }

        HaulResidualEdgeCompleter.Result EdgeCompletion =
            HaulResidualEdgeCompleter.Complete(
                OptimizedPasses,
                Grid,
                Flow,
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                ScraperBcy,
                Options.Optimization.EdgeMaxLaneSlicesPerCycle,
                Options.Optimization.FlowPenaltyWeight,
                Options.Optimization.PairRefinementRounds,
                HaulPairDistanceRefiner.PairMetric.Centroid,
                Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                GradeReturn: true,
                Obstacles,
                HeadingDeg);
        OptimizedPasses = OptimizedPasses
            .Concat(EdgeCompletion.Passes)
            .ToList();

        if (Options.Optimization.EnableRouteConsolidation)
        {
            HaulRouteConsolidationOptimizer.Result Consolidation =
                HaulRouteConsolidationOptimizer.Optimize(
                    OptimizedPasses,
                    Grid,
                    ScraperBcy,
                    Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER,
                    Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                    GradeReturn: true,
                    Options.Optimization.RouteConsolidationGroupSize,
                    Options.Optimization.RouteConsolidationMaximumCycles,
                    Obstacles);
            OptimizedPasses = Consolidation.Passes;
        }

        if (Options.Optimization.EnableMultiRouteExchange)
        {
            HaulMultiRouteExchangeOptimizer.Result Exchange =
                HaulMultiRouteExchangeOptimizer.Optimize(
                    OptimizedPasses,
                    Grid,
                    Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER,
                    Options.Optimization.MultiRouteExchangeRounds,
                    Options.Optimization.MultiRouteExchangeGroupSize,
                    Options.Optimization.MultiRouteExchangeMaximumGroups,
                    Options.Optimization.MultiRouteExchangeMinimumImprovementM,
                    Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                    GradeReturn: true,
                    Obstacles: Obstacles);
            OptimizedPasses = Exchange.Passes;
        }

        IReadOnlyList<HaulDeferredWork> Deferred = Ledger.BuildDeferredWork(
            OptimizedPasses, "OPTIMIZER_UNASSIGNED");
        Double UnmatchedCutBcy = Deferred
            .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
            .Sum(Entry => Entry.VolumeBcy);
        Double UnmatchedFillBcy = Deferred
            .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
            .Sum(Entry => Entry.VolumeBcy);
        HaulPlanValidation Validation = Ledger.ValidateComplete(
            OptimizedPasses,
            ScraperBcy,
            Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER,
            AllowDeferredWork: Deferred.Count > 0);
        if (!Validation.Success)
        {
            throw new InvalidOperationException(
                "Production haul allocation failed ledger validation: "
                + String.Join("; ", Validation.Issues.Take(10)));
        }

        return new PassResult(
            OptimizedPasses,
            Optimized.PassResult.Rounds,
            UnmatchedCutBcy,
            UnmatchedFillBcy);
    }

    /// <summary>Builds one mode's ponding options.</summary>
    /// <param name="Optimize">Whether ordering may change.</param>
    /// <param name="Interval">Checkpoint interval.</param>
    /// <param name="Weight">Ponding objective weight.</param>
    /// <param name="ObjectiveMode">Active ponding penalty definition.</param>
    /// <returns>Resolved ponding options.</returns>
    static private PondingOptions BuildOptions
        (
        Boolean Optimize,
        Int32 Interval,
        Double Weight,
        PondingObjectiveMode ObjectiveMode
        )
    {
        return new PondingOptions
        {
            Audit = true,
            Optimize = Optimize,
            ObjectiveMode = ObjectiveMode,
            CheckpointInterval = Interval,
            Weight = Weight,
            IncludeMap = true,
        };
    }

    /// <summary>Clones mutable route containers while retaining immutable passes.</summary>
    /// <param name="Route">Source route.</param>
    /// <returns>Independent route container.</returns>
    static private Route CloneRoute
        (
        Route Route
        )
    {
        return new Route(
            Route.Order.ToList(),
            Route.OrderedPasses.ToList(),
            Route.TotalDeadheadM);
    }

    /// <summary>Counts route slots whose original pass identifier changed.</summary>
    /// <param name="Before">Audit order.</param>
    /// <param name="After">Optimized order.</param>
    /// <returns>Changed slot count.</returns>
    static private Int32 CountChangedPositions(Route Before, Route After)
    {
        Int32 Count = 0;
        for (Int32 Index = 0; Index < Before.Order.Count; Index += 1)
            if (Before.Order[Index] != After.Order[Index]) Count += 1;
        return Count;
    }

    /// <summary>Checks that both orders reach the same final surface.</summary>
    /// <param name="Grid">Planning grid.</param>
    /// <param name="Before">Audit order.</param>
    /// <param name="After">Optimized order.</param>
    /// <returns>Maximum absolute final elevation difference.</returns>
    static private Double FinalSurfaceDifferenceM(
        Grid Grid,
        Route Before,
        Route After)
    {
        Double[,] First = Ponding.ReplaySurface(
            Grid, Before.OrderedPasses, Before.OrderedPasses.Count);
        Double[,] Second = Ponding.ReplaySurface(
            Grid, After.OrderedPasses, After.OrderedPasses.Count);
        Double Maximum = 0.0;
        for (Int32 Y = 0; Y < Grid.Height; Y += 1)
        for (Int32 X = 0; X < Grid.Width; X += 1)
            if (Grid.Valid[Y, X])
                Maximum = Math.Max(
                    Maximum, Math.Abs(First[Y, X] - Second[Y, X]));
        return Maximum;
    }

    /// <summary>Writes one audit's CSV and worst-checkpoint map.</summary>
    /// <param name="OutputDirectory">Destination directory.</param>
    /// <param name="Prefix">Artifact prefix.</param>
    /// <param name="Grid">Planning grid.</param>
    /// <param name="Result">Ponding result.</param>
    static private void WriteArtifacts(
        String OutputDirectory,
        String Prefix,
        Grid Grid,
        PondingRunResult Result)
    {
        PondingAuditResult Audit = Result.FinalAudit!;
        File.WriteAllText(
            Path.Combine(OutputDirectory, Prefix + "_ponding.csv"),
            Ponding.BuildCsv(Audit, Result.Outlets!.Count));
        File.WriteAllBytes(
            Path.Combine(OutputDirectory, Prefix + "_ponding.png"),
            Ponding.RenderWorstCheckpointPng(
                Grid, Result.Outlets!, Audit.WorstCheckpoint, 8));
    }

    /// <summary>Writes the complete comparison as a readable text report.</summary>
    static private void WriteComparison(
        String OutputDirectory,
        Grid Grid,
        Int32 PassCount,
        Double MovedBcy,
        TimeSpan PlanningElapsed,
        PondingRunResult Audit,
        TimeSpan AuditElapsed,
        Double AuditDeterministicHours,
        Double AuditScheduledHours,
        Double AuditGuidanceDistanceM,
        TimeSpan AuditGuidanceElapsed,
        PondingRunResult Optimized,
        TimeSpan OptimizeElapsed,
        Double OptimizedDeterministicHours,
        Double OptimizedScheduledHours,
        Double OptimizedGuidanceDistanceM,
        TimeSpan OptimizedGuidanceElapsed,
        Int32 ChangedPositions,
        Double FinalSurfaceDifference,
        Int32 Interval,
        Int32 Window,
        Double Weight,
        PondingObjectiveMode ObjectiveMode)
    {
        PondingAuditResult Before = Audit.FinalAudit!;
        PondingAuditResult After = Optimized.FinalAudit!;
        StringBuilder Text = new StringBuilder();
        Text.Append(
            "pass_pipeline=production_lane_first_through_consolidation\n"
            + "cycle_chain_optimization=not_run\n"
            + "exact_guidance=production_guidance_cost_oracle\n");
        Text.AppendFormat(CultureInfo.InvariantCulture,
            "grid={0}x{1}\npasses={2}\nmoved_bcy={3:F3}\nplanning_seconds={4:F3}\ncheckpoint_interval={5}\nwindow={6}\nweight={7:F1}\nobjective={8}\n",
            Grid.Width, Grid.Height, PassCount, MovedBcy,
            PlanningElapsed.TotalSeconds, Interval, Window, Weight,
            ObjectiveMode);
        Text.AppendFormat(CultureInfo.InvariantCulture,
            "audit_penalty={0:F9}\naudit_peak_bcy={1:F6}\naudit_mean_avg_depth_m={2:F9}\naudit_depth_m={3:F6}\naudit_worst_pass={4}\naudit_transition_s={5:F6}\naudit_exact_deterministic_h={6:F6}\naudit_exact_scheduled_h={7:F6}\naudit_exact_distance_m={8:F3}\naudit_stage_s={9:F3}\naudit_guidance_stage_s={10:F3}\n",
            Before.Penalty,
            Before.PeakVolumeBcy,
            Before.MeanAveragePondDepthM,
            Before.WorstCheckpoint.Metrics.MaxPondDepthM,
            Before.WorstCheckpoint.PassIndex,
            Audit.TransitionCostAfterS,
            AuditDeterministicHours,
            AuditScheduledHours,
            AuditGuidanceDistanceM,
            AuditElapsed.TotalSeconds,
            AuditGuidanceElapsed.TotalSeconds);
        Text.AppendFormat(CultureInfo.InvariantCulture,
            "optimized_penalty={0:F9}\noptimized_peak_bcy={1:F6}\noptimized_mean_avg_depth_m={2:F9}\noptimized_depth_m={3:F6}\noptimized_worst_pass={4}\noptimized_transition_s={5:F6}\noptimized_exact_deterministic_h={6:F6}\noptimized_exact_scheduled_h={7:F6}\noptimized_exact_distance_m={8:F3}\noptimized_stage_s={9:F3}\noptimized_guidance_stage_s={10:F3}\nrelocations={11}\nchanged_positions={12}\nfinal_surface_max_difference_m={13:E9}\n",
            After.Penalty,
            After.PeakVolumeBcy,
            After.MeanAveragePondDepthM,
            After.WorstCheckpoint.Metrics.MaxPondDepthM,
            After.WorstCheckpoint.PassIndex,
            Optimized.TransitionCostAfterS,
            OptimizedDeterministicHours,
            OptimizedScheduledHours,
            OptimizedGuidanceDistanceM,
            OptimizeElapsed.TotalSeconds,
            OptimizedGuidanceElapsed.TotalSeconds,
            Optimized.RelocationsAccepted,
            ChangedPositions,
            FinalSurfaceDifference);
        File.WriteAllText(
            Path.Combine(OutputDirectory, "comparison.txt"),
            Text.ToString());
    }

    /// <summary>Prints the headline proof metrics.</summary>
    static private void PrintSummary(
        Int32 PassCount,
        Double MovedBcy,
        TimeSpan PlanningElapsed,
        PondingRunResult Audit,
        TimeSpan AuditElapsed,
        Double AuditDeterministicHours,
        Double AuditScheduledHours,
        Double AuditGuidanceDistanceM,
        TimeSpan AuditGuidanceElapsed,
        PondingRunResult Optimized,
        TimeSpan OptimizeElapsed,
        Double OptimizedDeterministicHours,
        Double OptimizedScheduledHours,
        Double OptimizedGuidanceDistanceM,
        TimeSpan OptimizedGuidanceElapsed,
        Int32 ChangedPositions,
        Double FinalSurfaceDifference,
        PondingObjectiveMode ObjectiveMode)
    {
        PondingAuditResult Before = Audit.FinalAudit!;
        PondingAuditResult After = Optimized.FinalAudit!;
        Console.WriteLine(
            "plan: passes={0} moved_bcy={1:F3} planning_s={2:F2} objective={3}",
            PassCount, MovedBcy, PlanningElapsed.TotalSeconds, ObjectiveMode);
        Console.WriteLine(
            "audit: penalty={0:F6} peak_bcy={1:F3} mean_avg_depth_m={2:F6} depth_m={3:F4} worst_pass={4} transition_s={5:F2} exact_h={6:F4} scheduled_h={7:F4} distance_m={8:F1} stage_s={9:F2} guidance_s={10:F2}",
            Before.Penalty,
            Before.PeakVolumeBcy,
            Before.MeanAveragePondDepthM,
            Before.WorstCheckpoint.Metrics.MaxPondDepthM,
            Before.WorstCheckpoint.PassIndex,
            Audit.TransitionCostAfterS,
            AuditDeterministicHours,
            AuditScheduledHours,
            AuditGuidanceDistanceM,
            AuditElapsed.TotalSeconds,
            AuditGuidanceElapsed.TotalSeconds);
        Console.WriteLine(
            "optimized: penalty={0:F6} peak_bcy={1:F3} mean_avg_depth_m={2:F6} depth_m={3:F4} worst_pass={4} transition_s={5:F2} exact_h={6:F4} scheduled_h={7:F4} distance_m={8:F1} stage_s={9:F2} guidance_s={10:F2} relocations={11} changed_positions={12}",
            After.Penalty,
            After.PeakVolumeBcy,
            After.MeanAveragePondDepthM,
            After.WorstCheckpoint.Metrics.MaxPondDepthM,
            After.WorstCheckpoint.PassIndex,
            Optimized.TransitionCostAfterS,
            OptimizedDeterministicHours,
            OptimizedScheduledHours,
            OptimizedGuidanceDistanceM,
            OptimizeElapsed.TotalSeconds,
            OptimizedGuidanceElapsed.TotalSeconds,
            Optimized.RelocationsAccepted,
            ChangedPositions);
        Console.WriteLine(
            "proof: penalty_delta={0:F6} peak_delta_bcy={1:F3} mean_avg_depth_delta_m={2:F6} transition_delta_s={3:F2} exact_delta_h={4:F4} scheduled_delta_h={5:F4} distance_delta_m={6:F1} final_surface_max_difference_m={7:E9}",
            After.Penalty - Before.Penalty,
            After.PeakVolumeBcy - Before.PeakVolumeBcy,
            After.MeanAveragePondDepthM - Before.MeanAveragePondDepthM,
            Optimized.TransitionCostAfterS - Audit.TransitionCostAfterS,
            OptimizedDeterministicHours - AuditDeterministicHours,
            OptimizedScheduledHours - AuditScheduledHours,
            OptimizedGuidanceDistanceM - AuditGuidanceDistanceM,
            FinalSurfaceDifference);
    }
}
