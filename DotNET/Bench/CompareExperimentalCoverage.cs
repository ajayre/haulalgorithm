using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading.Tasks;

using HaulBuilder;

/// <summary>
/// Compares restored baseline cell ordering with isolated straight-run coverage plans.
/// </summary>
internal static class CompareExperimentalCoverage
{
    private const Int32 FIRST_ROUTE_INDEX = 7;
    private const Int32 LAST_ROUTE_INDEX = 9;
    private const Double SHARP_TURN_DEG = 45.0;
    private const Double REVERSAL_DEG = 100.0;

    private sealed class DisplayPoint
    {
        /// <summary>
        /// Initializes one diagnostic waypoint before heading and distance calculation.
        /// </summary>
        /// <param name="Action">Inbound diagnostic action.</param>
        /// <param name="Blade">Blade state.</param>
        /// <param name="Xy">World coordinate.</param>
        /// <param name="DepthMm">Displayed blade depth.</param>
        public DisplayPoint
            (
            String Action,
            String Blade,
            (Double X, Double Y) Xy,
            Double DepthMm
            )
        {
            // Keep diagnostic construction separate from production guidance orchestration.
            this.Action = Action;
            this.Blade = Blade;
            this.Xy = Xy;
            this.DepthMm = DepthMm;
        }

        public String Action { get; }
        public String Blade { get; }
        public (Double X, Double Y) Xy { get; }
        public Double DepthMm { get; }
    }

    private sealed class DisplayTrajectory
    {
        /// <summary>
        /// Initializes one diagnostic trajectory and connector summary.
        /// </summary>
        public DisplayTrajectory
            (
            List<GuidancePoint> Points,
            Int32 ConnectorCount,
            Int32 SuccessfulConnectorCount,
            Double ConnectorDistanceM,
            Int32 OperatorAlignmentCount,
            Double ManualAssistDistanceM
            )
        {
            // Keep feasibility visible independently of the illustrative PNG.
            this.Points = Points;
            this.ConnectorCount = ConnectorCount;
            this.SuccessfulConnectorCount = SuccessfulConnectorCount;
            this.ConnectorDistanceM = ConnectorDistanceM;
            this.OperatorAlignmentCount = OperatorAlignmentCount;
            this.ManualAssistDistanceM = ManualAssistDistanceM;
        }

        public List<GuidancePoint> Points { get; }
        public Int32 ConnectorCount { get; }
        public Int32 SuccessfulConnectorCount { get; }
        public Double ConnectorDistanceM { get; }
        public Int32 OperatorAlignmentCount { get; }
        public Double ManualAssistDistanceM { get; }
    }

    /// <summary>
    /// Builds TheShop2 once, reports route 8-10 metrics, and writes experimental-only PNGs.
    /// </summary>
    /// <param name="Args">Optional output directory and AGD path.</param>
    /// <returns>Zero when every experimental plan conserves allocation.</returns>
    static public Int32 Run
        (
        String[] Args
        )
    {
        String RepoRoot = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String OutputDirectory = (Args.Length > 0)
            ? Path.GetFullPath(Args[0])
            : Path.Combine(RepoRoot, "assets", "theshop2_experimental_coverage");
        String AgdPath = (Args.Length > 1)
            ? Path.GetFullPath(Args[1])
            : Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");
        Directory.CreateDirectory(OutputDirectory);

        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg) Plan = BuildRoute(AgdPath);
        Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0;
        Boolean Conserved = true;
        List<ExperimentalLifecyclePlanner.PassPlan> LifecyclePasses =
            new List<ExperimentalLifecyclePlanner.PassPlan>();

        for (Int32 RouteIndex = FIRST_ROUTE_INDEX; RouteIndex <= LAST_ROUTE_INDEX; RouteIndex += 1)
        {
            Pass Pass = Plan.Route.OrderedPasses[RouteIndex];
            ExperimentalCoveragePlanner.PlanResult CutPlan = ExperimentalCoveragePlanner.Plan(
                Pass.CutCells,
                Plan.Grid,
                BladeM,
                null,
                Pass.FillStartXy);
            (Double X, Double Y)? FillStart = CutPlan.Runs.Count == 0
                ? null
                : CutPlan.Runs[CutPlan.Runs.Count - 1].EndXy;
            ExperimentalCoveragePlanner.PlanResult FillPlan = ExperimentalCoveragePlanner.Plan(
                Pass.FillCells,
                Plan.Grid,
                BladeM,
                FillStart,
                null);
            (CutPlan, FillPlan) = OptimizeCutAndFillSequence(CutPlan, FillPlan, Plan.Grid);
            ExperimentalRunSequencer.Result CutSequence = BuildSequence(CutPlan.Runs, Plan.Grid);
            ExperimentalRunSequencer.Result FillSequence = BuildSequence(FillPlan.Runs, Plan.Grid);
            LifecyclePasses.Add(new ExperimentalLifecyclePlanner.PassPlan(
                CutSequence, FillSequence));

            Boolean CutConserved = ExperimentalCoveragePlanner.ConservesAllocation(
                Pass.CutCells, CutPlan.OrderedCells);
            Boolean FillConserved = ExperimentalCoveragePlanner.ConservesAllocation(
                Pass.FillCells, FillPlan.OrderedCells);
            Conserved = Conserved && CutConserved && FillConserved;

            (Int32 Sharp, Int32 Reversals, Double DistanceM) CutBaseline =
                CellPathMetrics(Pass.CutCells, Plan.Grid);
            (Int32 Sharp, Int32 Reversals, Double DistanceM) FillBaseline =
                CellPathMetrics(Pass.FillCells, Plan.Grid);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "route={0} cut cells={1} baseline_sharp={2} baseline_reverse={3} baseline_m={4:F1} experimental_runs={5} connector_m={6:F1} heading={7:F0} max_offset_m={8:F2} conserved={9}",
                RouteIndex + 1,
                Pass.CutCells.Count,
                CutBaseline.Sharp,
                CutBaseline.Reversals,
                CutBaseline.DistanceM,
                CutPlan.Runs.Count,
                CutPlan.ConnectorDistanceM,
                CutPlan.HeadingDeg,
                CutPlan.MaximumLateralErrorM,
                CutConserved));
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "route={0} fill cells={1} baseline_sharp={2} baseline_reverse={3} baseline_m={4:F1} experimental_runs={5} connector_m={6:F1} heading={7:F0} max_offset_m={8:F2} conserved={9}",
                RouteIndex + 1,
                Pass.FillCells.Count,
                FillBaseline.Sharp,
                FillBaseline.Reversals,
                FillBaseline.DistanceM,
                FillPlan.Runs.Count,
                FillPlan.ConnectorDistanceM,
                FillPlan.HeadingDeg,
                FillPlan.MaximumLateralErrorM,
                FillConserved));

            DisplayTrajectory Trajectory = BuildDisplayGuidance(CutPlan, FillPlan, Plan.Grid);
            List<GuidancePoint> Points = Trajectory.Points;
            List<IReadOnlyList<(Double X, Double Y)>> CutRuns = CutPlan.Runs.Select(Run =>
                (IReadOnlyList<(Double X, Double Y)>)new List<(Double X, Double Y)>
                {
                    Run.StartXy,
                    Run.EndXy,
                }).ToList();
            List<IReadOnlyList<(Double X, Double Y)>> FillRuns = FillPlan.Runs.Select(Run =>
                (IReadOnlyList<(Double X, Double Y)>)new List<(Double X, Double Y)>
                {
                    Run.StartXy,
                    Run.EndXy,
                }).ToList();
            List<(Double X, Double Y)> RoutePoints = Points.Select(Point =>
                (Point.X, Point.Y)).ToList();
            List<IReadOnlyList<(Double X, Double Y)>> ManualAssistRuns =
                ExtractActionRuns(Points, "MANUAL_ASSIST");
            Byte[] Png = Render.RenderCoverageRunsPng(
                Plan.Grid,
                CutRuns,
                FillRuns,
                RoutePoints,
                BladeM,
                pixelsPerBin: 40,
                padCells: 6,
                manualAssistRuns: ManualAssistRuns);
            String OutputPath = Path.Combine(
                OutputDirectory,
                String.Format(CultureInfo.InvariantCulture, "route_{0:D2}.png", RouteIndex + 1));
            File.WriteAllBytes(OutputPath, Png);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "route={0} valid_transits={1}/{2} transit_path_m={3:F1} operator_alignment_zones={4} manual_m={5:F1}",
                RouteIndex + 1,
                Trajectory.SuccessfulConnectorCount,
                Trajectory.ConnectorCount,
                Trajectory.ConnectorDistanceM,
                Trajectory.OperatorAlignmentCount,
                Trajectory.ManualAssistDistanceM));
            Console.WriteLine("wrote " + OutputPath);
        }

        ReportLifecycle("routes_08_10", LifecyclePasses, false, Plan.Grid);
        ReportLifecycle("routes_08_10", LifecyclePasses, true, Plan.Grid);
        List<ExperimentalLifecyclePlanner.PassPlan> WholePlanGradeOff =
            BuildWholePlanPasses(Plan.Route, Plan.Grid, BladeM, false);
        ReportLifecycle("whole_plan", WholePlanGradeOff, false, Plan.Grid);
        List<ExperimentalLifecyclePlanner.PassPlan> WholePlanGradeOn =
            BuildWholePlanPasses(Plan.Route, Plan.Grid, BladeM, true);
        ReportLifecycle("whole_plan", WholePlanGradeOn, true, Plan.Grid);

        return Conserved ? 0 : 1;
    }

    /// <summary>
    /// Builds isolated straight coverage and sequences for every ordered TheShop2 pass.
    /// </summary>
    private static List<ExperimentalLifecyclePlanner.PassPlan> BuildWholePlanPasses
        (
        Route Route,
        Grid Grid,
        Double BladeM,
        Boolean GradeReturn
        )
    {
        Int32 PassCount = Route.OrderedPasses.Count;
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "whole_plan building_passes={0}",
            PassCount));
        ExperimentalCoveragePlanner.PlanResult?[] CutPlans =
            new ExperimentalCoveragePlanner.PlanResult?[PassCount];
        ExperimentalCoveragePlanner.PlanResult?[] FillPlans =
            new ExperimentalCoveragePlanner.PlanResult?[PassCount];
        String?[] Failures = new String?[PassCount];
        Parallel.For(0, PassCount, PassIndex =>
        {
            Pass Pass = Route.OrderedPasses[PassIndex];
            ExperimentalCoveragePlanner.PlanResult Cut = ExperimentalCoveragePlanner.Plan(
                Pass.CutCells, Grid, BladeM, null, Pass.FillStartXy);
            (Double X, Double Y)? FillAnchor = Cut.Runs.Count == 0
                ? null
                : Cut.Runs[Cut.Runs.Count - 1].EndXy;
            ExperimentalCoveragePlanner.PlanResult Fill = ExperimentalCoveragePlanner.Plan(
                Pass.FillCells, Grid, BladeM, FillAnchor, null);
            CutPlans[PassIndex] = Cut;
            FillPlans[PassIndex] = Fill;
        });
        List<ExperimentalLifecyclePlanner.PassPlan> Output =
            new List<ExperimentalLifecyclePlanner.PassPlan>(PassCount);
        ExperimentalConnectorPlanner.Pose? Current = null;
        for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
        {
            ExperimentalCoveragePlanner.PlanResult Cut = CutPlans[PassIndex]
                ?? throw new InvalidOperationException("Parallel coverage construction omitted cut work.");
            ExperimentalCoveragePlanner.PlanResult Fill = FillPlans[PassIndex]
                ?? throw new InvalidOperationException("Parallel coverage construction omitted fill work.");
            ExperimentalRunSequencer.Result FillSequence = BuildSequence(Fill.Runs, Grid);
            if (GradeReturn && !SupportsGradeReturn(FillSequence, Grid))
            {
                foreach (ExperimentalCoveragePlanner.PlanResult Candidate in
                    ExperimentalCoveragePlanner.CandidatePlans(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM))
                {
                    ExperimentalRunSequencer.Result CandidateSequence =
                        BuildSequence(Candidate.Runs, Grid);
                    if (!CandidateSequence.Success
                        || !SupportsGradeReturn(CandidateSequence, Grid))
                    {
                        continue;
                    }
                    Fill = Candidate;
                    FillSequence = CandidateSequence;
                    break;
                }
            }
            ExperimentalConnectorPlanner.Pose? FillEntry = FillSequence.Runs.Count == 0
                ? null
                : FillSequence.Runs[0].EntryPose;
            ExperimentalRunSequencer.Result CutSequence = BuildSequence(
                Cut.Runs, Grid, Current, FillEntry);
            if (!CutSequence.Success)
            {
                foreach (ExperimentalCoveragePlanner.PlanResult Candidate in
                    ExperimentalCoveragePlanner.CandidatePlans(
                        Route.OrderedPasses[PassIndex].CutCells,
                        Grid,
                        BladeM,
                        Current.HasValue ? (Current.Value.X, Current.Value.Y) : null,
                        FillEntry.HasValue ? (FillEntry.Value.X, FillEntry.Value.Y) : null))
                {
                    ExperimentalRunSequencer.Result CandidateSequence = BuildSequence(
                        Candidate.Runs, Grid, Current, FillEntry);
                    if (!CandidateSequence.Success) continue;
                    Cut = Candidate;
                    CutSequence = CandidateSequence;
                    break;
                }
            }
            if (!CutSequence.Success || !FillSequence.Success)
            {
                Failures[PassIndex] = String.Format(
                    CultureInfo.InvariantCulture,
                    "whole_plan sequence_failure pass={0} cut_runs={1} fill_runs={2} cut_reason={3} fill_reason={4}",
                    PassIndex + 1,
                    Cut.Runs.Count,
                    Fill.Runs.Count,
                    CutSequence.FailureReason,
                    FillSequence.FailureReason);
            }
            Output.Add(new ExperimentalLifecyclePlanner.PassPlan(CutSequence, FillSequence));
            if (FillSequence.Runs.Count > 0)
            {
                ExperimentalRunSequencer.SequencedRun LastFill =
                    FillSequence.Runs[FillSequence.Runs.Count - 1];
                Current = LastFill.ExitPose;
                if (GradeReturn)
                {
                    ExperimentalRunSequencer.SequencedRun FirstFill = FillSequence.Runs[0];
                    Current = new ExperimentalConnectorPlanner.Pose(
                        FirstFill.EntryPose.X,
                        FirstFill.EntryPose.Y,
                        FirstFill.EntryPose.HeadingDeg + 180.0);
                }
            }
        }
        Int32 FailureCount = Failures.Count(Failure => Failure != null);
        Int32 ReportedFailures = 0;
        for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
        {
            if ((Failures[PassIndex] != null) && (ReportedFailures < 20))
            {
                Console.WriteLine(Failures[PassIndex]);
                ReportedFailures += 1;
            }
        }
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "whole_plan sequence_failures={0} reported={1}",
            FailureCount,
            ReportedFailures));
        return Output;
    }

    /// <summary>Checks fill, turnaround, and reverse grade connectivity for one pass.</summary>
    private static Boolean SupportsGradeReturn
        (
        ExperimentalRunSequencer.Result FillSequence,
        Grid Grid
        )
    {
        ExperimentalRunSequencer.Result EmptyCut = new ExperimentalRunSequencer.Result(
            true,
            new List<ExperimentalRunSequencer.SequencedRun>(),
            new List<ExperimentalTransitPlanner.Result>(),
            0.0,
            0.0,
            String.Empty);
        ExperimentalLifecyclePlanner.Result Lifecycle = ExperimentalLifecyclePlanner.Build(
            new List<ExperimentalLifecyclePlanner.PassPlan>
            {
                new ExperimentalLifecyclePlanner.PassPlan(EmptyCut, FillSequence),
            },
            true,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
        return Lifecycle.Success;
    }

    /// <summary>
    /// Extracts contiguous action-owned polylines including each inbound start point.
    /// </summary>
    private static List<IReadOnlyList<(Double X, Double Y)>> ExtractActionRuns
        (
        List<GuidancePoint> Points,
        String Action
        )
    {
        List<IReadOnlyList<(Double X, Double Y)>> Output =
            new List<IReadOnlyList<(Double X, Double Y)>>();
        List<(Double X, Double Y)>? Current = null;
        for (Int32 Index = 0; Index < Points.Count; Index += 1)
        {
            if (Points[Index].Action == Action)
            {
                if (Current == null)
                {
                    Current = new List<(Double X, Double Y)>();
                    if (Index > 0) Current.Add((Points[Index - 1].X, Points[Index - 1].Y));
                }
                Current.Add((Points[Index].X, Points[Index].Y));
            }
            else if (Current != null)
            {
                Output.Add(Current);
                Current = null;
            }
        }
        if (Current != null) Output.Add(Current);
        return Output;
    }

    /// <summary>
    /// Builds one strict experimental work sequence for lifecycle comparison.
    /// </summary>
    private static ExperimentalRunSequencer.Result BuildSequence
        (
        List<ExperimentalCoveragePlanner.Run> Runs,
        Grid Grid,
        ExperimentalConnectorPlanner.Pose? Start = null,
        ExperimentalConnectorPlanner.Pose? End = null
        )
    {
        return ExperimentalRunSequencer.Optimize(
            Runs,
            Start,
            End,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
    }

    /// <summary>
    /// Reports grade-off or grade-on post-fill lifecycle distance and final-geometry time.
    /// </summary>
    private static void ReportLifecycle
        (
        String Scope,
        List<ExperimentalLifecyclePlanner.PassPlan> Passes,
        Boolean GradeReturn,
        Grid Grid
        )
    {
        ExperimentalLifecyclePlanner.Result Lifecycle = ExperimentalLifecyclePlanner.Build(
            Passes,
            GradeReturn,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
        ExperimentalGuidanceTiming.Result Timing = ExperimentalGuidanceTiming.Estimate(
            Lifecycle,
            new ExperimentalGuidanceTiming.Options
            {
                FieldEfficiency = Constants.DEFAULT_FIELD_EFFICIENCY,
            });
        Double ManualAssistDistanceM = Lifecycle.Segments
            .Where(Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.ManualAssist)
            .Sum(Segment => PolylineDistance(Segment.Points));
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "lifecycle scope={0} grade_return={1} success={2} segments={3} manual_zones={4} manual_m={5:F1} distance_m={6:F1} modeled_s={7:F1} reported_s={8:F1} failure={9}",
            Scope,
            GradeReturn,
            Lifecycle.Success,
            Lifecycle.Segments.Count,
            Lifecycle.ManualAssistCount,
            ManualAssistDistanceM,
            Lifecycle.DistanceM,
            Timing.MotionSeconds + Timing.EventSeconds,
            Timing.TotalSeconds,
            Lifecycle.FailureReason));
    }

    /// <summary>
    /// Computes polyline distance for lifecycle diagnostics.
    /// </summary>
    private static Double PolylineDistance
        (
        List<(Double X, Double Y)> Points
        )
    {
        Double TotalM = 0.0;
        for (Int32 Index = 1; Index < Points.Count; Index += 1)
        {
            TotalM += Distance(Points[Index - 1], Points[Index]);
        }
        return TotalM;
    }

    /// <summary>
    /// Builds the same restored and refined route sequence used by baseline exports.
    /// </summary>
    private static (AgdField Field, Grid Grid, Route Route, Double HeadingDeg) BuildRoute
        (
        String AgdPath
        )
    {
        Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
        AgdField Field = Agd.Parse(AgdPath);
        Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
        Balance.Compute(Grid, "check", Double.PositiveInfinity);
        FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
        List<Strip> Strips = HaulBuilder.Strips.TraceStrips(Grid, Flow, 400);
        PassResult PassResult = Passes.GeneratePasses(
            Grid,
            Strips,
            Constants.DEFAULT_TOLERANCE_MM,
            ScraperBcy,
            0,
            null,
            0.5,
            50.0,
            Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS,
            Flow,
            true,
            0.8,
            Obstacles.EMPTY_OBSTACLES);
        Route Route = Routing.SequencePasses(
            PassResult.Passes,
            1500,
            true,
            true,
            Constants.DEFAULT_RETURN_SPEED_MPH,
            Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT,
            Obstacles.EMPTY_OBSTACLES);
        Double HeadingDeg = Passes.EstimateSlopeHeading(Grid);
        Swaths.ReprofileKnots(
            Route, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, HeadingDeg, 0.0);
        Routing.RefineRouteOrder(
            Route,
            50,
            true,
            true,
            Constants.DEFAULT_RETURN_SPEED_MPH,
            Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT,
            12,
            Obstacles.EMPTY_OBSTACLES);
        return (Field, Grid, Route, HeadingDeg);
    }

    /// <summary>
    /// Alternates exact connector-aware optimization of cut and fill run order and direction.
    /// </summary>
    private static (
        ExperimentalCoveragePlanner.PlanResult CutPlan,
        ExperimentalCoveragePlanner.PlanResult FillPlan) OptimizeCutAndFillSequence
        (
        ExperimentalCoveragePlanner.PlanResult CutPlan,
        ExperimentalCoveragePlanner.PlanResult FillPlan,
        Grid Grid
        )
    {
        List<ExperimentalCoveragePlanner.Run> CutRuns = CutPlan.Runs;
        List<ExperimentalCoveragePlanner.Run> FillRuns = FillPlan.Runs;
        for (Int32 Iteration = 0; Iteration < 2; Iteration += 1)
        {
            ExperimentalConnectorPlanner.Pose? FillEntry = FillRuns.Count == 0
                ? null
                : StartPose(FillRuns[0], Grid);
            CutRuns = OptimizeRunSequence(CutRuns, null, FillEntry, Grid);
            ExperimentalConnectorPlanner.Pose? CutExit = CutRuns.Count == 0
                ? null
                : EndPose(CutRuns[CutRuns.Count - 1], Grid);
            FillRuns = OptimizeRunSequence(FillRuns, CutExit, null, Grid);
        }

        return (
            RebuildPlan(CutPlan, CutRuns),
            RebuildPlan(FillPlan, FillRuns));
    }

    /// <summary>
    /// Solves the shortest strict-connector Hamiltonian path across both orientations of every run.
    /// </summary>
    private static List<ExperimentalCoveragePlanner.Run> OptimizeRunSequence
        (
        List<ExperimentalCoveragePlanner.Run> Runs,
        ExperimentalConnectorPlanner.Pose? Start,
        ExperimentalConnectorPlanner.Pose? End,
        Grid Grid
        )
    {
        ExperimentalRunSequencer.Result Sequence = ExperimentalRunSequencer.Optimize(
            Runs,
            Start,
            End,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
        return Sequence.Success
            ? Sequence.Runs.Select(Sequenced => Sequenced.Run).ToList()
            : Runs;
    }

    /// <summary>
    /// Creates the entry pose of a straight coverage run.
    /// </summary>
    private static ExperimentalConnectorPlanner.Pose StartPose
        (
        ExperimentalCoveragePlanner.Run Run,
        Grid Grid
        )
    {
        ExperimentalRunSequencer.SequencedRun Prepared = ExperimentalRunSequencer.Prepare(
            Run,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
        return Prepared.EntryPose;
    }

    /// <summary>
    /// Creates the exit pose of a straight coverage run.
    /// </summary>
    private static ExperimentalConnectorPlanner.Pose EndPose
        (
        ExperimentalCoveragePlanner.Run Run,
        Grid Grid
        )
    {
        ExperimentalRunSequencer.SequencedRun Prepared = ExperimentalRunSequencer.Prepare(
            Run,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
        return Prepared.ExitPose;
    }

    /// <summary>
    /// Rebuilds comparison metrics after connector-aware ordering.
    /// </summary>
    private static ExperimentalCoveragePlanner.PlanResult RebuildPlan
        (
        ExperimentalCoveragePlanner.PlanResult Original,
        List<ExperimentalCoveragePlanner.Run> Runs
        )
    {
        Double ConnectorDistanceM = 0.0;
        for (Int32 Index = 1; Index < Runs.Count; Index += 1)
        {
            ConnectorDistanceM += Distance(Runs[Index - 1].EndXy, Runs[Index].StartXy);
        }
        return new ExperimentalCoveragePlanner.PlanResult(
            Original.HeadingDeg,
            Runs,
            ConnectorDistanceM,
            Original.AnchorDistanceM,
            Original.Score);
    }

    /// <summary>
    /// Converts straight coverage runs into a diagnostic-only cut/haul/fill polyline.
    /// </summary>
    private static DisplayTrajectory BuildDisplayGuidance
        (
        ExperimentalCoveragePlanner.PlanResult CutPlan,
        ExperimentalCoveragePlanner.PlanResult FillPlan,
        Grid Grid
        )
    {
        List<DisplayPoint> Display = new List<DisplayPoint>();
        Int32 ConnectorCount = 0;
        Int32 SuccessfulConnectorCount = 0;
        Int32 OperatorAlignmentCount = 0;
        Double ConnectorDistanceM = 0.0;
        Double ManualAssistDistanceM = 0.0;
        ExperimentalRunSequencer.SequencedRun? PreviousRun = null;
        AppendRuns(
            Display,
            CutPlan.Runs,
            "CUT",
            "START",
            Grid,
            ref PreviousRun,
            ref ConnectorCount,
            ref SuccessfulConnectorCount,
            ref ConnectorDistanceM,
            ref OperatorAlignmentCount,
            ref ManualAssistDistanceM);
        AppendRuns(
            Display,
            FillPlan.Runs,
            "FILL",
            "HAUL",
            Grid,
            ref PreviousRun,
            ref ConnectorCount,
            ref SuccessfulConnectorCount,
            ref ConnectorDistanceM,
            ref OperatorAlignmentCount,
            ref ManualAssistDistanceM);

        List<GuidancePoint> Output = new List<GuidancePoint>();
        Double CumulativeM = 0.0;
        for (Int32 Index = 0; Index < Display.Count; Index += 1)
        {
            Double SegmentM = (Index == 0) ? 0.0 : Distance(Display[Index - 1].Xy, Display[Index].Xy);
            CumulativeM += SegmentM;
            Double HeadingDeg = (Index + 1 < Display.Count)
                ? BearingDeg(Display[Index].Xy, Display[Index + 1].Xy)
                : ((Index > 0) ? BearingDeg(Display[Index - 1].Xy, Display[Index].Xy) : 0.0);
            DisplayPoint Point = Display[Index];
            Output.Add(new GuidancePoint(
                Index + 1,
                1,
                Point.Action,
                Point.Blade,
                Point.Xy.X,
                Point.Xy.Y,
                0.0,
                0.0,
                Point.DepthMm,
                HeadingDeg,
                SegmentM,
                CumulativeM,
                "Experimental coverage comparison"));
        }
        return new DisplayTrajectory(
            Output,
            ConnectorCount,
            SuccessfulConnectorCount,
            ConnectorDistanceM,
            OperatorAlignmentCount,
            ManualAssistDistanceM);
    }

    /// <summary>
    /// Appends straight blade-down runs separated by explicit blade-up reposition segments.
    /// </summary>
    private static void AppendRuns
        (
        List<DisplayPoint> Output,
        List<ExperimentalCoveragePlanner.Run> Runs,
        String WorkAction,
        String InitialAction,
        Grid Grid,
        ref ExperimentalRunSequencer.SequencedRun? PreviousRun,
        ref Int32 ConnectorCount,
        ref Int32 SuccessfulConnectorCount,
        ref Double ConnectorDistanceM,
        ref Int32 OperatorAlignmentCount,
        ref Double ManualAssistDistanceM
        )
    {
        for (Int32 RunIndex = 0; RunIndex < Runs.Count; RunIndex += 1)
        {
            ExperimentalCoveragePlanner.Run Run = Runs[RunIndex];
            ExperimentalRunSequencer.SequencedRun Prepared = ExperimentalRunSequencer.Prepare(
                Run,
                Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
                Math.Min(0.5, Grid.GridSizeM),
                Grid);
            if (PreviousRun == null)
            {
                Output.Add(new DisplayPoint(
                    InitialAction,
                    "UP",
                    (Prepared.EntryPose.X, Prepared.EntryPose.Y),
                    0.0));
            }
            else
            {
                ConnectorCount += 1;
                String ConnectorAction = ((RunIndex == 0) && (WorkAction == "FILL"))
                    ? "HAUL"
                    : "MOVE";
                ExperimentalTransitPlanner.Result Connector = ExperimentalTransitPlanner.Connect(
                    new ExperimentalConnectorPlanner.Pose(
                        PreviousRun.ExitPose.X,
                        PreviousRun.ExitPose.Y,
                        PreviousRun.ExitPose.HeadingDeg),
                    Prepared.EntryPose,
                    Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
                    Math.Min(0.5, Grid.GridSizeM),
                    Grid);
                if (Connector.Success)
                {
                    SuccessfulConnectorCount += 1;
                    ConnectorDistanceM += Connector.LengthM;
                    ManualAssistDistanceM += Connector.ManualAssistDistanceM;
                    foreach (ExperimentalTransitPlanner.Leg Leg in Connector.Legs)
                    {
                        if (Leg.OperatorAlignmentRequired) OperatorAlignmentCount += 1;
                        String DisplayAction = Leg.OperatorAlignmentRequired
                            ? "MANUAL_ASSIST"
                            : ConnectorAction;
                        for (Int32 PointIndex = 1; PointIndex < Leg.Points.Count; PointIndex += 1)
                        {
                            Output.Add(new DisplayPoint(
                                DisplayAction,
                                "UP",
                                Leg.Points[PointIndex],
                                0.0));
                        }
                    }
                }
                else
                {
                    // Retain a visible chord only to locate the failed connection in the diagnostic image.
                    Output.Add(new DisplayPoint(
                        ConnectorAction,
                        "UP",
                        (Prepared.EntryPose.X, Prepared.EntryPose.Y),
                        0.0));
                }
            }
            Output.Add(new DisplayPoint("LINEUP", "UP", Run.StartXy, 0.0));
            Double DepthMm = Run.Cells.Count == 0
                ? 0.0
                : 1000.0 * Run.Cells.Average(Cell => Math.Abs(Cell.DepthM));
            Output.Add(new DisplayPoint(WorkAction, "DOWN", Run.EndXy, DepthMm));
            Output.Add(new DisplayPoint(
                "LINEUP",
                "UP",
                (Prepared.ExitPose.X, Prepared.ExitPose.Y),
                0.0));
            PreviousRun = Prepared;
        }
    }

    /// <summary>
    /// Counts sharp corners and reversals in the supplied baseline cell order.
    /// </summary>
    private static (Int32 Sharp, Int32 Reversals, Double DistanceM) CellPathMetrics
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid
        )
    {
        Int32 Sharp = 0;
        Int32 Reversals = 0;
        Double DistanceM = 0.0;
        List<(Double X, Double Y)> Points = Cells.Select(Cell =>
            Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By)).ToList();
        for (Int32 Index = 1; Index < Points.Count; Index += 1)
        {
            DistanceM += Distance(Points[Index - 1], Points[Index]);
        }
        for (Int32 Index = 1; Index + 1 < Points.Count; Index += 1)
        {
            Double TurnDeg = HeadingChangeDeg(Points[Index - 1], Points[Index], Points[Index + 1]);
            if (TurnDeg >= SHARP_TURN_DEG) Sharp += 1;
            if (TurnDeg >= REVERSAL_DEG) Reversals += 1;
        }
        return (Sharp, Reversals, DistanceM);
    }

    /// <summary>
    /// Computes absolute heading change at one planar vertex.
    /// </summary>
    private static Double HeadingChangeDeg
        (
        (Double X, Double Y) A,
        (Double X, Double Y) B,
        (Double X, Double Y) C
        )
    {
        Double First = Math.Atan2(B.Y - A.Y, B.X - A.X);
        Double Second = Math.Atan2(C.Y - B.Y, C.X - B.X);
        Double Delta = Math.Abs((Second - First) * 180.0 / Math.PI);
        return Delta > 180.0 ? 360.0 - Delta : Delta;
    }

    /// <summary>
    /// Computes compass bearing in degrees.
    /// </summary>
    private static Double BearingDeg
        (
        (Double X, Double Y) A,
        (Double X, Double Y) B
        )
    {
        Double HeadingDeg = Math.Atan2(B.X - A.X, B.Y - A.Y) * 180.0 / Math.PI;
        return (HeadingDeg + 360.0) % 360.0;
    }

    /// <summary>
    /// Computes planar distance in metres.
    /// </summary>
    private static Double Distance
        (
        (Double X, Double Y) A,
        (Double X, Double Y) B
        )
    {
        Double Dx = B.X - A.X;
        Double Dy = B.Y - A.Y;
        return Math.Sqrt((Dx * Dx) + (Dy * Dy));
    }
}
