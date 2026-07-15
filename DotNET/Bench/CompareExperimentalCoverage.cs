using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using HaulBuilder;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Gif;
using SixLabors.ImageSharp.PixelFormats;

/// <summary>
/// Compares restored baseline cell ordering with isolated straight-run coverage plans.
/// </summary>
internal static class CompareExperimentalCoverage
{
    private const Int32 FIRST_ROUTE_INDEX = 7;
    private const Int32 LAST_ROUTE_INDEX = 9;
    private const Double SHARP_TURN_DEG = 45.0;
    private const Double REVERSAL_DEG = 100.0;
    private const Int32 MAX_PAIRED_CANDIDATE_ATTEMPTS = 72 * 72;
    private const Int32 MAX_BACKTRACK_CANDIDATE_ATTEMPTS = 250000;
    private const Int32 MAX_INITIAL_CUT_CANDIDATE_ATTEMPTS = 6;

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

        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg, EdgePatchOmissionResult EdgeOmission) Plan = BuildRoute(AgdPath);
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
                CutSequence,
                FillSequence,
                CutPlan.OmittedCells.Count,
                FillPlan.OmittedCells.Count));

            Boolean CutConserved = ExperimentalCoveragePlanner.ConservesAllocation(
                Pass.CutCells, CutPlan.AccountedCells);
            Boolean FillConserved = ExperimentalCoveragePlanner.ConservesAllocation(
                Pass.FillCells, FillPlan.AccountedCells);
            Conserved = Conserved && CutConserved && FillConserved;

            (Int32 Sharp, Int32 Reversals, Double DistanceM) CutBaseline =
                CellPathMetrics(Pass.CutCells, Plan.Grid);
            (Int32 Sharp, Int32 Reversals, Double DistanceM) FillBaseline =
                CellPathMetrics(Pass.FillCells, Plan.Grid);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "route={0} cut cells={1} baseline_sharp={2} baseline_reverse={3} baseline_m={4:F1} experimental_runs={5} connector_m={6:F1} heading={7:F0} max_offset_m={8:F2} accounted={9} omitted_cells={10}",
                RouteIndex + 1,
                Pass.CutCells.Count,
                CutBaseline.Sharp,
                CutBaseline.Reversals,
                CutBaseline.DistanceM,
                CutPlan.Runs.Count,
                CutPlan.ConnectorDistanceM,
                CutPlan.HeadingDeg,
                CutPlan.MaximumLateralErrorM,
                CutConserved,
                CutPlan.OmittedCells.Count));
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "route={0} fill cells={1} baseline_sharp={2} baseline_reverse={3} baseline_m={4:F1} experimental_runs={5} connector_m={6:F1} heading={7:F0} max_offset_m={8:F2} accounted={9} omitted_cells={10}",
                RouteIndex + 1,
                Pass.FillCells.Count,
                FillBaseline.Sharp,
                FillBaseline.Reversals,
                FillBaseline.DistanceM,
                FillPlan.Runs.Count,
                FillPlan.ConnectorDistanceM,
                FillPlan.HeadingDeg,
                FillPlan.MaximumLateralErrorM,
                FillConserved,
                FillPlan.OmittedCells.Count));

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
    /// Plans the complete grade-enabled field lifecycle once and writes a concise
    /// manual-intervention and geometry report.
    /// </summary>
    /// <param name="Args">Optional AGD path, report path, and pass limit.</param>
    /// <returns>Zero when the complete lifecycle succeeds and stays within allowance.</returns>
    static public Int32 ReportWholeField
        (
        String[] Args
        )
    {
        String RepoRoot = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String AgdPath = (Args.Length > 0)
            ? Path.GetFullPath(Args[0])
            : Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");
        String ReportPath = (Args.Length > 1)
            ? Path.GetFullPath(Args[1])
            : Path.Combine(
                RepoRoot,
                "assets",
                "theshop2_experimental_coverage",
                "whole_field_guidance_stats.txt");
        Int32? PassLimit = null;
        if ((Args.Length > 2)
            && Int32.TryParse(
                Args[2],
                NumberStyles.Integer,
                CultureInfo.InvariantCulture,
                out Int32 ParsedPassLimit))
        {
            PassLimit = Math.Max(0, ParsedPassLimit);
        }
        String? ReportDirectory = Path.GetDirectoryName(ReportPath);
        if (!String.IsNullOrWhiteSpace(ReportDirectory))
        {
            Directory.CreateDirectory(ReportDirectory);
        }

        ExperimentalTransitPlanner.ResetCache();
        Stopwatch RunTimer = Stopwatch.StartNew();
        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg,
            EdgePatchOmissionResult EdgeOmission) Plan = default;
        List<ExperimentalLifecyclePlanner.PassPlan> Passes;
        ExperimentalLifecyclePlanner.Result Lifecycle;
        List<String> PlanningDiagnostics = new List<String>();
        TextWriter OriginalOutput = Console.Out;
        String ProgressPath = ReportPath + ".progress.txt";
        Object ProgressLock = new Object();
        String ProgressStage = String.Empty;
        Stopwatch StageTimer = Stopwatch.StartNew();
        Stopwatch ProgressWriteTimer = Stopwatch.StartNew();
        void WriteProgress(String Stage, Int32 Completed, Int32 Total)
        {
            lock (ProgressLock)
            {
                String StageKey = Stage.Split(' ', 2)[0];
                if (!String.Equals(ProgressStage, StageKey, StringComparison.Ordinal))
                {
                    ProgressStage = StageKey;
                    StageTimer.Restart();
                    ProgressWriteTimer.Restart();
                }
                Boolean Complete = (Total == 0) || (Completed >= Total);
                Boolean PassStart = Stage.Contains(" pass=", StringComparison.Ordinal);
                if (!Complete && !PassStart && (Completed > 0)
                    && (ProgressWriteTimer.Elapsed < TimeSpan.FromSeconds(5.0)))
                {
                    return;
                }
                Double StageSeconds = Math.Max(StageTimer.Elapsed.TotalSeconds, 0.001);
                Double Rate = Completed / StageSeconds;
                Double EtaSeconds = (Rate > 0.0)
                    ? Math.Max(0, Total - Completed) / Rate
                    : Double.PositiveInfinity;
                String Eta = Double.IsFinite(EtaSeconds)
                    ? TimeSpan.FromSeconds(EtaSeconds).ToString("c", CultureInfo.InvariantCulture)
                    : "unknown";
                String ProgressText = String.Format(
                    CultureInfo.InvariantCulture,
                    "stage={0}{1}completed={2}{1}total={3}{1}percent={4:F2}{1}rate_per_second={5:F4}{1}stage_elapsed={6}{1}stage_eta={7}{1}total_elapsed={8}{1}",
                    Stage,
                    Environment.NewLine,
                    Completed,
                    Total,
                    Total > 0 ? 100.0 * Completed / Total : 100.0,
                    Rate,
                    StageTimer.Elapsed.ToString("c", CultureInfo.InvariantCulture),
                    Eta,
                    RunTimer.Elapsed.ToString("c", CultureInfo.InvariantCulture));
                File.WriteAllText(ProgressPath, ProgressText);
                OriginalOutput.WriteLine(ProgressText.Replace(Environment.NewLine, " ").Trim());
                OriginalOutput.Flush();
                ProgressWriteTimer.Restart();
            }
        }
        try
        {
            Console.SetOut(TextWriter.Null);
            Plan = BuildRoute(AgdPath);
            Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0;
            Passes = BuildWholePlanPasses(
                Plan.Route, Plan.Grid, BladeM, true,
                PassLimit,
                Progress: WriteProgress,
                Diagnostic: Message =>
                {
                    PlanningDiagnostics.Add(Message);
                    OriginalOutput.WriteLine(Message);
                    OriginalOutput.Flush();
                },
                StopOnFailure: true);
            Lifecycle = ExperimentalLifecyclePlanner.Build(
                Passes,
                true,
                Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
                Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
                Math.Min(0.5, Plan.Grid.GridSizeM),
                Plan.Grid,
                (Completed, Total) => WriteProgress(
                    "lifecycle", Completed, Total),
                FillSequencesPrepared: true);
        }
        catch (Exception Error)
        {
            String FailureText = String.Format(
                CultureInfo.InvariantCulture,
                "WHOLE FIELD GUIDANCE REPORT{0}agd={1}{0}lifecycle_success=False{0}elapsed_minutes={2:F2}{0}failure=unhandled_exception{0}exception={3}{0}",
                Environment.NewLine,
                AgdPath,
                RunTimer.Elapsed.TotalMinutes,
                Error);
            File.WriteAllText(ReportPath, FailureText);
            File.WriteAllText(
                ProgressPath,
                String.Format(
                    CultureInfo.InvariantCulture,
                    "stage=failed{0}total_elapsed={1}{0}failure=unhandled_exception{0}",
                    Environment.NewLine,
                    RunTimer.Elapsed.ToString("c", CultureInfo.InvariantCulture)));
            OriginalOutput.WriteLine(FailureText);
            OriginalOutput.WriteLine("report_path={0}", ReportPath);
            OriginalOutput.Flush();
            return 1;
        }
        finally
        {
            Console.SetOut(OriginalOutput);
        }

        ExperimentalGuidanceTiming.Result Timing = ExperimentalGuidanceTiming.Estimate(
            Lifecycle,
            new ExperimentalGuidanceTiming.Options
            {
                FieldEfficiency = Constants.DEFAULT_FIELD_EFFICIENCY,
            });
        ExperimentalFieldEdgePolicy EdgePolicy =
            new ExperimentalFieldEdgePolicy(Plan.Grid, Plan.Grid.GridSizeM);
        List<(Int32 SegmentIndex, ExperimentalLifecyclePlanner.Segment Segment,
            Double DistanceM, Boolean Exterior, Boolean BeyondAllowance,
            ExperimentalLifecyclePlanner.Phase Previous,
            ExperimentalLifecyclePlanner.Phase Next)> Manual =
                new List<(Int32, ExperimentalLifecyclePlanner.Segment, Double,
                    Boolean, Boolean, ExperimentalLifecyclePlanner.Phase,
                    ExperimentalLifecyclePlanner.Phase)>();
        for (Int32 SegmentIndex = 0;
            SegmentIndex < Lifecycle.Segments.Count;
            SegmentIndex += 1)
        {
            ExperimentalLifecyclePlanner.Segment Segment =
                Lifecycle.Segments[SegmentIndex];
            if (Segment.Phase != ExperimentalLifecyclePlanner.Phase.ManualAssist) continue;
            ExperimentalLifecyclePlanner.Phase Previous = SegmentIndex > 0
                ? Lifecycle.Segments[SegmentIndex - 1].Phase
                : ExperimentalLifecyclePlanner.Phase.Start;
            ExperimentalLifecyclePlanner.Phase Next =
                SegmentIndex + 1 < Lifecycle.Segments.Count
                    ? Lifecycle.Segments[SegmentIndex + 1].Phase
                    : ExperimentalLifecyclePlanner.Phase.End;
            Manual.Add((
                SegmentIndex,
                Segment,
                PolylineDistance(Segment.Points),
                Segment.Points.Any(Point =>
                    !Plan.Grid.ContainsWorldXy(Point.X, Point.Y)),
                Segment.Points.Any(Point =>
                    !EdgePolicy.ContainsWithExteriorAllowance(Point.X, Point.Y)),
                Previous,
                Next));
        }

        List<Double> ManualDistancesM = Manual
            .Select(Item => Item.DistanceM)
            .OrderBy(DistanceM => DistanceM)
            .ToList();
        Double ManualDistanceM = ManualDistancesM.Sum();
        Double MaximumGapM = 0.0;
        for (Int32 SegmentIndex = 1;
            SegmentIndex < Lifecycle.Segments.Count;
            SegmentIndex += 1)
        {
            List<(Double X, Double Y)> Previous =
                Lifecycle.Segments[SegmentIndex - 1].Points;
            List<(Double X, Double Y)> Current =
                Lifecycle.Segments[SegmentIndex].Points;
            if ((Previous.Count == 0) || (Current.Count == 0)) continue;
            MaximumGapM = Math.Max(
                MaximumGapM,
                Distance(Previous[Previous.Count - 1], Current[0]));
        }
        Int32 BeyondAllowanceSegments = Lifecycle.Segments.Count(Segment =>
            Segment.Points.Any(Point =>
                !EdgePolicy.ContainsWithExteriorAllowance(Point.X, Point.Y)));
        Int32 OmittedCells = Passes.Sum(Pass =>
            Pass.CutOmittedCellCount + Pass.FillOmittedCellCount);
        Double StraightCoverageShortfallM = Passes
            .SelectMany(Pass => Pass.Cut.Runs.Concat(Pass.Fill.Runs))
            .Sum(Run => Run.StartCoverageShortfallM + Run.EndCoverageShortfallM);
        (Int64 Hits, Int64 Misses, Int32 Entries,
            Int32 RetainedEntries, Int64 Clears) TransitCache =
            ExperimentalTransitPlanner.GetCacheStatistics();
        (Int64 Calls, Int64 Successes, Int64 ExpansionLimitHits,
            Int64 Expansions, Int64 MaximumSuccessExpansions,
            Double ElapsedSeconds) ForwardDetours =
            ExperimentalTransitPlanner.GetForwardDetourStatistics();

        StringBuilder Report = new StringBuilder();
        Report.AppendLine("WHOLE FIELD GUIDANCE REPORT");
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "agd={0}", AgdPath));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "routes={0} grade_return=true lifecycle_success={1}",
            Passes.Count,
            Lifecycle.Success));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "manual_interventions={0} manual_segments={1} manual_distance_m={2:F1} manual_distance_mi={3:F2} manual_percent_distance={4:F3}",
            Lifecycle.ManualAssistCount,
            Manual.Count,
            ManualDistanceM,
            ManualDistanceM / 1609.344,
            Lifecycle.DistanceM > 0.0 ? 100.0 * ManualDistanceM / Lifecycle.DistanceM : 0.0));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "manual_exterior={0} manual_in_field_alignment={1} manual_beyond_allowance={2}",
            Manual.Count(Item => Item.Exterior),
            Manual.Count(Item => !Item.Exterior),
            Manual.Count(Item => Item.BeyondAllowance)));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "manual_length_m_min={0:F1} median={1:F1} p90={2:F1} max={3:F1}",
            Percentile(ManualDistancesM, 0.0),
            Percentile(ManualDistancesM, 0.5),
            Percentile(ManualDistancesM, 0.9),
            Percentile(ManualDistancesM, 1.0)));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "total_segments={0} total_distance_m={1:F1} total_distance_mi={2:F2} modeled_hours={3:F2}",
            Lifecycle.Segments.Count,
            Lifecycle.DistanceM,
            Lifecycle.DistanceM / 1609.344,
            Timing.TotalSeconds / 3600.0));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "max_segment_gap_m={0:F6} segments_beyond_one_cell_allowance={1}",
            MaximumGapM,
            BeyondAllowanceSegments));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "guidance_omitted_cells={0} nominal_straight_coverage_shortfall_m={1:F1}",
            OmittedCells,
            StraightCoverageShortfallM));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "edge_patch_omitted_cut_cells={0} edge_patch_omitted_cut_bcy={1:F2}",
            Plan.EdgeOmission.OmittedCutCells.Count,
            Plan.EdgeOmission.OmittedCutBcy));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "elapsed_minutes={0:F2} failure={1}",
            RunTimer.Elapsed.TotalMinutes,
            Lifecycle.FailureReason));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "transit_cache_hits={0} transit_cache_misses={1} transit_cache_entries={2} retained_entries={3} clears={4} hit_percent={5:F1}",
            TransitCache.Hits,
            TransitCache.Misses,
            TransitCache.Entries,
            TransitCache.RetainedEntries,
            TransitCache.Clears,
            (TransitCache.Hits + TransitCache.Misses) > 0
                ? 100.0 * TransitCache.Hits
                    / (TransitCache.Hits + TransitCache.Misses)
                : 0.0));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "forward_detour_calls={0} successes={1} expansion_limit_hits={2} expansions={3} max_success_expansions={4} elapsed_seconds={5:F2} avg_expansions={6:F1}",
            ForwardDetours.Calls,
            ForwardDetours.Successes,
            ForwardDetours.ExpansionLimitHits,
            ForwardDetours.Expansions,
            ForwardDetours.MaximumSuccessExpansions,
            ForwardDetours.ElapsedSeconds,
            ForwardDetours.Calls > 0
                ? ForwardDetours.Expansions / (Double)ForwardDetours.Calls
                : 0.0));
        Report.AppendLine("planning_diagnostics:");
        foreach (String Diagnostic in PlanningDiagnostics)
        {
            Report.AppendLine("  " + Diagnostic);
        }
        Report.AppendLine("manual_phase_pairs:");
        foreach (IGrouping<(ExperimentalLifecyclePlanner.Phase Previous,
            ExperimentalLifecyclePlanner.Phase Next),
            (Int32 SegmentIndex, ExperimentalLifecyclePlanner.Segment Segment,
                Double DistanceM, Boolean Exterior, Boolean BeyondAllowance,
                ExperimentalLifecyclePlanner.Phase Previous,
                ExperimentalLifecyclePlanner.Phase Next)> Group in Manual
                    .GroupBy(Item => (Item.Previous, Item.Next))
                    .OrderByDescending(Group => Group.Count()))
        {
            Report.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "  {0}->{1}: count={2} distance_m={3:F1}",
                Group.Key.Previous,
                Group.Key.Next,
                Group.Count(),
                Group.Sum(Item => Item.DistanceM)));
        }
        Report.AppendLine("longest_manual_interventions:");
        foreach ((Int32 SegmentIndex, ExperimentalLifecyclePlanner.Segment Segment,
            Double DistanceM, Boolean Exterior, Boolean BeyondAllowance,
            ExperimentalLifecyclePlanner.Phase Previous,
            ExperimentalLifecyclePlanner.Phase Next) Item in Manual
                .OrderByDescending(Item => Item.DistanceM)
                .Take(20))
        {
            (Double X, Double Y) Start = Item.Segment.Points[0];
            (Double X, Double Y) End =
                Item.Segment.Points[Item.Segment.Points.Count - 1];
            Report.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "  segment={0} {1}->{2} length_m={3:F1} exterior={4} beyond_allowance={5} start=({6:F1},{7:F1}) end=({8:F1},{9:F1})",
                Item.SegmentIndex,
                Item.Previous,
                Item.Next,
                Item.DistanceM,
                Item.Exterior,
                Item.BeyondAllowance,
                Start.X,
                Start.Y,
                End.X,
                End.Y));
        }

        File.WriteAllText(ReportPath, Report.ToString());
        Console.Write(Report.ToString());
        Console.WriteLine("report_path=" + ReportPath);
        return Lifecycle.Success && (BeyondAllowanceSegments == 0) ? 0 : 1;
    }

    /// <summary>
    /// Builds only the parallel straight-coverage stage and records the work-run
    /// counts that determine sequencing search complexity.
    /// </summary>
    /// <param name="Args">Optional AGD path followed by CSV path.</param>
    /// <returns>Zero after the complete coverage inventory is written.</returns>
    static public Int32 ProfileGuidanceCoverage
        (
        String[] Args
        )
    {
        String RepoRoot = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String AgdPath = (Args.Length > 0)
            ? Path.GetFullPath(Args[0])
            : Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");
        String CsvPath = (Args.Length > 1)
            ? Path.GetFullPath(Args[1])
            : Path.Combine(
                RepoRoot,
                "assets",
                "theshop2_experimental_coverage",
                "whole_field_coverage_profile.csv");
        String? CsvDirectory = Path.GetDirectoryName(CsvPath);
        if (!String.IsNullOrWhiteSpace(CsvDirectory))
        {
            Directory.CreateDirectory(CsvDirectory);
        }

        Stopwatch Timer = Stopwatch.StartNew();
        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg,
            EdgePatchOmissionResult EdgeOmission) Plan = BuildRoute(AgdPath);
        Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0;
        Int32 PassCount = Plan.Route.OrderedPasses.Count;
        ExperimentalCoveragePlanner.PlanResult?[] Cuts =
            new ExperimentalCoveragePlanner.PlanResult?[PassCount];
        ExperimentalCoveragePlanner.PlanResult?[] Fills =
            new ExperimentalCoveragePlanner.PlanResult?[PassCount];
        Parallel.For(0, PassCount, PassIndex =>
        {
            Pass Pass = Plan.Route.OrderedPasses[PassIndex];
            ExperimentalCoveragePlanner.PlanResult Cut =
                ExperimentalCoveragePlanner.Plan(
                    Pass.CutCells, Plan.Grid, BladeM, null, Pass.FillStartXy);
            (Double X, Double Y)? FillAnchor = Cut.Runs.Count == 0
                ? null
                : Cut.Runs[Cut.Runs.Count - 1].EndXy;
            Cuts[PassIndex] = Cut;
            Fills[PassIndex] = ExperimentalCoveragePlanner.Plan(
                Pass.FillCells, Plan.Grid, BladeM, FillAnchor, null);
        });

        StringBuilder Csv = new StringBuilder();
        Csv.AppendLine("pass,cut_cells,fill_cells,cut_runs,fill_runs,cut_omitted,fill_omitted,max_runs");
        List<Int32> MaximumRunCounts = new List<Int32>(PassCount);
        for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
        {
            Pass Pass = Plan.Route.OrderedPasses[PassIndex];
            ExperimentalCoveragePlanner.PlanResult Cut = Cuts[PassIndex]
                ?? throw new InvalidOperationException("Coverage inventory omitted cut work.");
            ExperimentalCoveragePlanner.PlanResult Fill = Fills[PassIndex]
                ?? throw new InvalidOperationException("Coverage inventory omitted fill work.");
            Int32 MaximumRuns = Math.Max(Cut.Runs.Count, Fill.Runs.Count);
            MaximumRunCounts.Add(MaximumRuns);
            Csv.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "{0},{1},{2},{3},{4},{5},{6},{7}",
                PassIndex + 1,
                Pass.CutCells.Count,
                Pass.FillCells.Count,
                Cut.Runs.Count,
                Fill.Runs.Count,
                Cut.OmittedCells.Count,
                Fill.OmittedCells.Count,
                MaximumRuns));
        }
        File.WriteAllText(CsvPath, Csv.ToString());

        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "coverage_profile passes={0} elapsed_s={1:F2} max_runs={2} passes_over_exact_5={3} passes_over_fallback_7={4} passes_over_10={5}",
            PassCount,
            Timer.Elapsed.TotalSeconds,
            MaximumRunCounts.Max(),
            MaximumRunCounts.Count(Count => Count > 5),
            MaximumRunCounts.Count(Count => Count > 7),
            MaximumRunCounts.Count(Count => Count > 10)));
        Console.WriteLine("coverage_profile_path=" + CsvPath);
        return 0;
    }

    /// <summary>Runs one exact field connector diagnostic without sequencing routes.</summary>
    /// <param name="Args">AGD path and start/end X, Y, heading values.</param>
    /// <returns>Zero when a forward connector succeeds.</returns>
    static public Int32 DiagnoseGuidanceConnector
        (
        String[] Args
        )
    {
        if (Args.Length < 7)
        {
            Console.Error.WriteLine(
                "usage: --diagnose-guidance-connector agd start_x start_y start_heading end_x end_y end_heading");
            return 2;
        }
        String AgdPath = Path.GetFullPath(Args[0]);
        Double[] Values = Args.Skip(1).Take(6)
            .Select(Value => Double.Parse(
                Value, NumberStyles.Float, CultureInfo.InvariantCulture))
            .ToArray();
        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg,
            EdgePatchOmissionResult EdgeOmission) Plan = BuildRoute(AgdPath);
        ExperimentalConnectorPlanner.Pose Start =
            new ExperimentalConnectorPlanner.Pose(
                Values[0], Values[1], Values[2]);
        ExperimentalConnectorPlanner.Pose End =
            new ExperimentalConnectorPlanner.Pose(
                Values[3], Values[4], Values[5]);
        Double RadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
            * Constants.FEET_TO_METERS;
        Double SampleSpacingM = Math.Min(0.5, Plan.Grid.GridSizeM);
        foreach (Boolean AllowForwardDetour in new[] { false, true })
        {
            Stopwatch Timer = Stopwatch.StartNew();
            ExperimentalTransitPlanner.Result Result =
                ExperimentalTransitPlanner.Connect(
                    Start,
                    End,
                    RadiusM,
                    SampleSpacingM,
                    Plan.Grid,
                    EdgeAllowanceM: Plan.Grid.GridSizeM,
                    AllowForwardDetour: AllowForwardDetour);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "connector detour={0} success={1} mode={2} length_m={3:F2} manual_m={4:F2} leaves_field={5} points={6} elapsed_s={7:F3} reason={8}",
                AllowForwardDetour,
                Result.Success,
                Result.Mode,
                Result.LengthM,
                Result.ManualAssistDistanceM,
                Result.LeavesField,
                Result.Points.Count,
                Timer.Elapsed.TotalSeconds,
                Result.FailureReason));
        }
        foreach (Double HeadingDeg in new[] { 0.0, 90.0, 180.0, 270.0 })
        {
            Double HeadingRad = HeadingDeg * Math.PI / 180.0;
            Double ClearanceM = 0.0;
            for (Double DistanceM = SampleSpacingM;
                DistanceM <= 100.0;
                DistanceM += SampleSpacingM)
            {
                if (!Plan.Grid.ContainsWorldXy(
                    Start.X + (Math.Sin(HeadingRad) * DistanceM),
                    Start.Y + (Math.Cos(HeadingRad) * DistanceM)))
                {
                    break;
                }
                ClearanceM = DistanceM;
            }
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "clearance heading={0:F0} distance_m={1:F1}",
                HeadingDeg,
                ClearanceM));
        }
        return ExperimentalTransitPlanner.Connect(
            Start,
            End,
            RadiusM,
            SampleSpacingM,
            Plan.Grid,
            EdgeAllowanceM: Plan.Grid.GridSizeM,
            AllowForwardDetour: true).Success ? 0 : 1;
    }

    /// <summary>
    /// Writes one cut/fill map showing the complete connected lifecycle for routes one to three.
    /// </summary>
    /// <param name="Args">Optional --gif flag, output PNG path, and AGD path.</param>
    /// <returns>Zero when the selected lifecycle is complete.</returns>
    static public Int32 ExportFirstThree
        (
        String[] Args
        )
    {
        return ExportRouteRange(Args, 1, 3);
    }

    /// <summary>Writes one connected lifecycle map for routes three through five.</summary>
    static public Int32 ExportRoutesThreeToFive
        (
        String[] Args
        )
    {
        return ExportRouteRange(Args, 3, 5);
    }

    /// <summary>Writes one connected lifecycle map for an arbitrary ordered route range.</summary>
    static public Int32 ExportRouteRange
        (
        String[] Args
        )
    {
        if ((Args.Length < 2)
            || !Int32.TryParse(Args[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out Int32 FirstRouteNumber)
            || !Int32.TryParse(Args[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out Int32 LastRouteNumber)
            || (FirstRouteNumber < 1)
            || (LastRouteNumber < FirstRouteNumber))
        {
            Console.Error.WriteLine(
                "usage: --export-route-range <first> <last> [--gif] [outputPng] [agdPath]");
            return 2;
        }

        return ExportRouteRange(
            Args.Skip(2).ToArray(), FirstRouteNumber, LastRouteNumber);
    }

    /// <summary>Writes one cut/fill map for an inclusive ordered route range.</summary>
    private static Int32 ExportRouteRange
        (
        String[] Args,
        Int32 FirstRouteNumber,
        Int32 LastRouteNumber
        )
    {
        Boolean WriteGif = Args.Any(Argument => String.Equals(
            Argument, "--gif", StringComparison.OrdinalIgnoreCase));
        List<String> PositionalArgs = Args.Where(Argument => !String.Equals(
            Argument, "--gif", StringComparison.OrdinalIgnoreCase)).ToList();
        String RepoRoot = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String OutputPath = (PositionalArgs.Count > 0)
            ? Path.GetFullPath(PositionalArgs[0])
            : Path.Combine(
                RepoRoot,
                "assets",
                "theshop2_experimental_coverage",
                String.Format(
                    CultureInfo.InvariantCulture,
                    "routes_{0:D2}_{1:D2}_connected.png",
                    FirstRouteNumber,
                    LastRouteNumber));
        String AgdPath = (PositionalArgs.Count > 1)
            ? Path.GetFullPath(PositionalArgs[1])
            : Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");
        String? OutputDirectory = Path.GetDirectoryName(OutputPath);
        if (!String.IsNullOrWhiteSpace(OutputDirectory))
        {
            Directory.CreateDirectory(OutputDirectory);
        }

        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg, EdgePatchOmissionResult EdgeOmission) Plan = BuildRoute(AgdPath);
        Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0;
        List<ExperimentalLifecyclePlanner.PassPlan> AllPasses =
            BuildWholePlanPasses(
                Plan.Route, Plan.Grid, BladeM, true, LastRouteNumber);
        List<ExperimentalLifecyclePlanner.PassPlan> Passes = AllPasses
            .Skip(FirstRouteNumber - 1)
            .Take(LastRouteNumber - FirstRouteNumber + 1)
            .ToList();
        if (Passes.Count != LastRouteNumber - FirstRouteNumber + 1)
        {
            Console.Error.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "requested routes {0}-{1}, but only {2} ordered routes exist",
                FirstRouteNumber,
                LastRouteNumber,
                AllPasses.Count));
            return 1;
        }
        ExperimentalLifecyclePlanner.Result Lifecycle = ExperimentalLifecyclePlanner.Build(
            Passes,
            true,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Plan.Grid.GridSizeM),
            Plan.Grid);
        if (!Lifecycle.Success)
        {
            Int32 FailedPassOffset = Passes.FindIndex(Pass =>
                !Pass.Cut.Success || !Pass.Fill.Success);
            Int32 FailedRouteNumber = FailedPassOffset >= 0
                ? FirstRouteNumber + FailedPassOffset
                : FirstRouteNumber;
            WriteFailedRouteDiagnostic(
                Plan.Route.OrderedPasses[FailedRouteNumber - 1],
                Plan.Grid,
                BladeM,
                FailedRouteNumber,
                OutputPath);
            Console.Error.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "routes {0}-{1} lifecycle failed: {2}; wrote route {3} failure diagnostic to {4}",
                FirstRouteNumber,
                LastRouteNumber,
                Lifecycle.FailureReason,
                FailedRouteNumber,
                OutputPath));
            return 1;
        }
        List<IReadOnlyList<(Double X, Double Y)>> CutRuns = Lifecycle.Segments
            .Where(Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.Cut)
            .Select(Segment => (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
            .ToList();
        List<IReadOnlyList<(Double X, Double Y)>> FillRuns = Lifecycle.Segments
            .Where(Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.Fill)
            .Select(Segment => (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
            .ToList();
        List<(Double X, Double Y)> RoutePoints = Lifecycle.Segments
            .SelectMany(Segment => Segment.Points)
            .ToList();
        ReportLifecycleGeometry(Lifecycle, Plan.Grid, BladeM);
        for (Int32 SegmentIndex = 0; SegmentIndex < Lifecycle.Segments.Count; SegmentIndex += 1)
        {
            ExperimentalLifecyclePlanner.Segment Segment = Lifecycle.Segments[SegmentIndex];
            Double SegmentDistanceM = PolylineDistance(Segment.Points);
            if ((Segment.Phase != ExperimentalLifecyclePlanner.Phase.Fill)
                && (Segment.Phase != ExperimentalLifecyclePlanner.Phase.Grade))
            {
                continue;
            }
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "work_segment={0} phase={1} start=({2:F1},{3:F1}) end=({4:F1},{5:F1}) heading_deg={6:F1} length_m={7:F1}",
                SegmentIndex,
                Segment.Phase,
                Segment.Points[0].X,
                Segment.Points[0].Y,
                Segment.Points[Segment.Points.Count - 1].X,
                Segment.Points[Segment.Points.Count - 1].Y,
                BearingDeg(
                    Segment.Points[0],
                    Segment.Points[Segment.Points.Count - 1]),
                SegmentDistanceM));
        }
        List<IReadOnlyList<(Double X, Double Y)>> ManualAssistRuns = Lifecycle.Segments
            .Where(Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.ManualAssist)
            .Select(Segment => (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
            .ToList();
        Double CoverageShortfallM = Passes
            .SelectMany(Pass => Pass.Cut.Runs.Concat(Pass.Fill.Runs))
            .Sum(Run => Run.StartCoverageShortfallM + Run.EndCoverageShortfallM);
        Int32 OmittedCellCount = Passes.Sum(Pass =>
            Pass.CutOmittedCellCount + Pass.FillOmittedCellCount);
        Int32 ManualZoneIndex = 0;
        for (Int32 SegmentIndex = 0; SegmentIndex < Lifecycle.Segments.Count; SegmentIndex += 1)
        {
            ExperimentalLifecyclePlanner.Segment Segment = Lifecycle.Segments[SegmentIndex];
            if (Segment.Phase != ExperimentalLifecyclePlanner.Phase.ManualAssist) continue;
            ManualZoneIndex += 1;
            Double ManualStartHeading = Segment.Points.Count >= 2
                ? BearingDeg(Segment.Points[0], Segment.Points[1])
                : Double.NaN;
            Double ManualEndHeading = Segment.Points.Count >= 2
                ? BearingDeg(
                    Segment.Points[Segment.Points.Count - 2],
                    Segment.Points[Segment.Points.Count - 1])
                : Double.NaN;
            Double InboundHeading = AdjacentHeading(
                Lifecycle.Segments, SegmentIndex, -1);
            Double OutboundHeading = AdjacentHeading(
                Lifecycle.Segments, SegmentIndex, 1);
            Double MaximumInteriorDeflectionDeg = 0.0;
            for (Int32 PointIndex = 1;
                PointIndex + 1 < Segment.Points.Count;
                PointIndex += 1)
            {
                MaximumInteriorDeflectionDeg = Math.Max(
                    MaximumInteriorDeflectionDeg,
                    HeadingChangeDeg(
                        Segment.Points[PointIndex - 1],
                        Segment.Points[PointIndex],
                        Segment.Points[PointIndex + 1]));
            }
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "manual_zone={0} segment={1} previous={2} next={3} length_m={4:F2} entry_delta_deg={5:F1} exit_delta_deg={6:F1} max_internal_delta_deg={7:F1} start=({8:F1},{9:F1}) end=({10:F1},{11:F1})",
                ManualZoneIndex,
                SegmentIndex,
                SegmentIndex > 0 ? Lifecycle.Segments[SegmentIndex - 1].Phase : ExperimentalLifecyclePlanner.Phase.Start,
                SegmentIndex + 1 < Lifecycle.Segments.Count ? Lifecycle.Segments[SegmentIndex + 1].Phase : ExperimentalLifecyclePlanner.Phase.End,
                PolylineDistance(Segment.Points),
                HeadingDeltaDeg(InboundHeading, ManualStartHeading),
                HeadingDeltaDeg(ManualEndHeading, OutboundHeading),
                MaximumInteriorDeflectionDeg,
                Segment.Points[0].X,
                Segment.Points[0].Y,
                Segment.Points[Segment.Points.Count - 1].X,
                Segment.Points[Segment.Points.Count - 1].Y));
        }
        List<((Double X, Double Y) Point, String Label)> RouteMarkers = Passes
            .Select((Pass, Index) =>
            {
                ExperimentalRunSequencer.SequencedRun First = Pass.Cut.Runs[0];
                return (
                    (
                        0.5 * (First.Run.StartXy.X + First.Run.EndXy.X),
                        0.5 * (First.Run.StartXy.Y + First.Run.EndXy.Y)),
                    (FirstRouteNumber + Index).ToString(CultureInfo.InvariantCulture));
            })
            .ToList();

        Byte[] Png = Render.RenderCoverageRunsPng(
            Plan.Grid,
            CutRuns,
            FillRuns,
            RoutePoints,
            BladeM,
            pixelsPerBin: 40,
            padCells: 8,
            manualAssistRuns: ManualAssistRuns,
            routeMarkers: RouteMarkers,
            drawDirectionArrows: true,
            directionArrowSpacingM: 6.0 * Plan.Grid.GridSizeM);
        File.WriteAllBytes(OutputPath, Png);
        if (WriteGif)
        {
            String GifPath = Path.ChangeExtension(OutputPath, ".gif");
            try
            {
                WriteLifecycleGif(
                    GifPath,
                    Lifecycle,
                    Plan.Grid,
                    CutRuns,
                    FillRuns,
                    BladeM,
                    RouteMarkers);
            }
            catch (Exception Exception)
            {
                Console.Error.WriteLine("GIF export failed: " + Exception.Message);
                return 1;
            }
        }
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "edge_patch_omission eligible_components={0} omitted_components={1} removed_passes={2} cut_cells={3} fill_cells={4} cut_bcy={5:F2} fill_bcy={6:F2} cap_bcy={7:F2}",
            Plan.EdgeOmission.EligibleComponentCount,
            Plan.EdgeOmission.OmittedComponentCount,
            Plan.EdgeOmission.RemovedPassCount,
            Plan.EdgeOmission.OmittedCutCells.Count,
            Plan.EdgeOmission.OmittedFillCells.Count,
            Plan.EdgeOmission.OmittedCutBcy,
            Plan.EdgeOmission.OmittedFillBcy,
            Plan.EdgeOmission.EffectiveProjectCapBcy));
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "wrote {0} routes={1}-{2} segments={3} manual_zones={4} distance_m={5:F1} coverage_shortfall_m={6:F1} guidance_omitted_cells={7} guidance_omitted_area_m2={8:F2} edge_omitted_cut_cells={9} edge_omitted_cut_bcy={10:F2}",
            OutputPath,
            FirstRouteNumber,
            LastRouteNumber,
            Lifecycle.Segments.Count,
            Lifecycle.ManualAssistCount,
            Lifecycle.DistanceM,
            CoverageShortfallM,
            OmittedCellCount,
            OmittedCellCount * Plan.Grid.AreaM2,
            Plan.EdgeOmission.OmittedCutCells.Count,
            Plan.EdgeOmission.OmittedCutBcy));
        return 0;
    }

    /// <summary>Writes raw work runs and attempted connectors when lifecycle sequencing fails.</summary>
    private static void WriteFailedRouteDiagnostic
        (
        Pass Pass,
        Grid Grid,
        Double BladeM,
        Int32 RouteNumber,
        String OutputPath
        )
    {
        ExperimentalCoveragePlanner.PlanResult CutPlan =
            ExperimentalCoveragePlanner.Plan(
                Pass.CutCells, Grid, BladeM, null, Pass.FillStartXy);
        (Double X, Double Y)? FillStart = CutPlan.Runs.Count == 0
            ? null
            : CutPlan.Runs[CutPlan.Runs.Count - 1].EndXy;
        ExperimentalCoveragePlanner.PlanResult FillPlan =
            ExperimentalCoveragePlanner.Plan(
                Pass.FillCells, Grid, BladeM, FillStart, null);
        (CutPlan, FillPlan) = OptimizeCutAndFillSequence(
            CutPlan, FillPlan, Grid);
        DisplayTrajectory Trajectory = BuildDisplayGuidance(
            CutPlan, FillPlan, Grid);
        List<IReadOnlyList<(Double X, Double Y)>> CutRuns = CutPlan.Runs
            .Select(Run => (IReadOnlyList<(Double X, Double Y)>)
                new List<(Double X, Double Y)>
                {
                    Run.StartXy,
                    Run.EndXy,
                })
            .ToList();
        List<IReadOnlyList<(Double X, Double Y)>> FillRuns = FillPlan.Runs
            .Select(Run => (IReadOnlyList<(Double X, Double Y)>)
                new List<(Double X, Double Y)>
                {
                    Run.StartXy,
                    Run.EndXy,
                })
            .ToList();
        List<(Double X, Double Y)> RoutePoints = Trajectory.Points
            .Select(Point => (Point.X, Point.Y))
            .ToList();
        List<IReadOnlyList<(Double X, Double Y)>> ManualAssistRuns =
            ExtractActionRuns(Trajectory.Points, "MANUAL_ASSIST");
        List<((Double X, Double Y) Point, String Label)> RouteMarkers =
            new List<((Double X, Double Y) Point, String Label)>();
        if (CutPlan.Runs.Count > 0)
        {
            ExperimentalCoveragePlanner.Run First = CutPlan.Runs[0];
            RouteMarkers.Add((
                (
                    0.5 * (First.StartXy.X + First.EndXy.X),
                    0.5 * (First.StartXy.Y + First.EndXy.Y)),
                RouteNumber.ToString(CultureInfo.InvariantCulture)));
        }

        Byte[] Png = Render.RenderCoverageRunsPng(
            Grid,
            CutRuns,
            FillRuns,
            RoutePoints,
            BladeM,
            pixelsPerBin: 40,
            padCells: 8,
            manualAssistRuns: ManualAssistRuns,
            routeMarkers: RouteMarkers,
            drawDirectionArrows: true,
            directionArrowSpacingM: 6.0 * Grid.GridSizeM);
        File.WriteAllBytes(OutputPath, Png);
    }

    /// <summary>Writes a progressive animated GIF of one connected experimental lifecycle.</summary>
    private static void WriteLifecycleGif
        (
        String GifPath,
        ExperimentalLifecyclePlanner.Result Lifecycle,
        Grid Grid,
        List<IReadOnlyList<(Double X, Double Y)>> CutRuns,
        List<IReadOnlyList<(Double X, Double Y)>> FillRuns,
        Double BladeM,
        List<((Double X, Double Y) Point, String Label)> RouteMarkers
        )
    {
        const Int32 FRAME_COUNT = 96;
        const Int32 FRAME_DELAY_MS = 100;
        List<Byte[]> Frames = new List<Byte[]>(FRAME_COUNT);
        List<(Double X, Double Y)> FixedBounds = Lifecycle.Segments
            .SelectMany(Segment => Segment.Points)
            .ToList();
        for (Int32 FrameIndex = 0; FrameIndex < FRAME_COUNT; FrameIndex += 1)
        {
            Double TargetM = Lifecycle.DistanceM * FrameIndex / (FRAME_COUNT - 1);
            (List<(Double X, Double Y)> Route,
                List<IReadOnlyList<(Double X, Double Y)>> ManualRuns) =
                BuildLifecyclePrefix(Lifecycle, TargetM);
            List<((Double X, Double Y) Point, String Label)> Markers =
                new List<((Double X, Double Y) Point, String Label)>(RouteMarkers);
            if (Route.Count > 0)
            {
                Markers.Add((Route[Route.Count - 1], String.Empty));
            }
            Frames.Add(Render.RenderCoverageRunsPng(
                Grid,
                CutRuns,
                FillRuns,
                Route,
                BladeM,
                pixelsPerBin: 20,
                padCells: 8,
                manualAssistRuns: ManualRuns,
                routeMarkers: Markers,
                drawDirectionArrows: true,
                directionArrowSpacingM: 6.0 * Grid.GridSizeM,
                boundsPoints: FixedBounds));
        }

        using Image<Rgba32> Gif = Image.Load<Rgba32>(Frames[0]);
        SetGifDelay(Gif.Frames.RootFrame, FRAME_DELAY_MS);
        for (Int32 FrameIndex = 1; FrameIndex < Frames.Count; FrameIndex += 1)
        {
            using Image<Rgba32> Frame = Image.Load<Rgba32>(Frames[FrameIndex]);
            ImageFrame<Rgba32> Added = Gif.Frames.AddFrame(Frame.Frames.RootFrame);
            SetGifDelay(
                Added,
                FrameIndex == Frames.Count - 1 ? 2500 : FRAME_DELAY_MS);
        }
        Gif.Metadata.GetGifMetadata().RepeatCount = 0;
        Gif.Save(GifPath, new GifEncoder());
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "wrote {0} frames -> {1}",
            Frames.Count,
            GifPath));
    }

    /// <summary>Builds the driven and manual portions completed by a target distance.</summary>
    private static (List<(Double X, Double Y)> Route,
        List<IReadOnlyList<(Double X, Double Y)>> ManualRuns) BuildLifecyclePrefix
        (
        ExperimentalLifecyclePlanner.Result Lifecycle,
        Double TargetM
        )
    {
        List<(Double X, Double Y)> Route = new List<(Double X, Double Y)>();
        List<IReadOnlyList<(Double X, Double Y)>> ManualRuns =
            new List<IReadOnlyList<(Double X, Double Y)>>();
        Double RemainingM = Math.Max(0.0, TargetM);
        foreach (ExperimentalLifecyclePlanner.Segment Segment in Lifecycle.Segments)
        {
            if (Segment.Points.Count == 0) continue;
            List<(Double X, Double Y)>? Manual = Segment.Phase
                == ExperimentalLifecyclePlanner.Phase.ManualAssist
                    ? new List<(Double X, Double Y)>()
                    : null;
            AppendDistinct(Route, Segment.Points[0]);
            if (Manual != null) AppendDistinct(Manual, Segment.Points[0]);
            for (Int32 PointIndex = 1;
                PointIndex < Segment.Points.Count;
                PointIndex += 1)
            {
                (Double X, Double Y) Start = Segment.Points[PointIndex - 1];
                (Double X, Double Y) End = Segment.Points[PointIndex];
                Double LegM = Distance(Start, End);
                if (LegM <= RemainingM + 1e-9)
                {
                    AppendDistinct(Route, End);
                    if (Manual != null) AppendDistinct(Manual, End);
                    RemainingM = Math.Max(0.0, RemainingM - LegM);
                    continue;
                }

                if (LegM > 1e-12)
                {
                    Double Fraction = RemainingM / LegM;
                    (Double X, Double Y) Partial = (
                        Start.X + ((End.X - Start.X) * Fraction),
                        Start.Y + ((End.Y - Start.Y) * Fraction));
                    AppendDistinct(Route, Partial);
                    if (Manual != null) AppendDistinct(Manual, Partial);
                }
                if ((Manual != null) && (Manual.Count >= 2)) ManualRuns.Add(Manual);
                return (Route, ManualRuns);
            }
            if ((Manual != null) && (Manual.Count >= 2)) ManualRuns.Add(Manual);
            if (RemainingM <= 1e-9) return (Route, ManualRuns);
        }
        return (Route, ManualRuns);
    }

    /// <summary>Adds a point unless it duplicates the current polyline endpoint.</summary>
    private static void AppendDistinct
        (
        List<(Double X, Double Y)> Points,
        (Double X, Double Y) Point
        )
    {
        if ((Points.Count == 0)
            || (Distance(Points[Points.Count - 1], Point) > 1e-9))
        {
            Points.Add(Point);
        }
    }

    /// <summary>Sets one GIF frame delay from milliseconds.</summary>
    private static void SetGifDelay
        (
        ImageFrame<Rgba32> Frame,
        Int32 DelayMs
        )
    {
        Frame.Metadata.GetGifMetadata().FrameDelay = Math.Max(
            1,
            (Int32)Math.Round(DelayMs / 10.0, MidpointRounding.AwayFromZero));
    }

    /// <summary>
    /// Reports whether the rendered lifecycle is continuous and wholly contained by the field mask.
    /// </summary>
    private static void ReportLifecycleGeometry
        (
        ExperimentalLifecyclePlanner.Result Lifecycle,
        Grid Grid,
        Double EdgeAllowanceM
        )
    {
        Int32 OutsideSampleCount = 0;
        Int32 OutsideInterpolatedCount = 0;
        Int32 BeyondAllowanceSampleCount = 0;
        Int32 BeyondAllowanceInterpolatedCount = 0;
        Int32 ReportedOutsideLegs = 0;
        Double MaximumBoundaryGapM = 0.0;
        Int32 MaximumBoundaryGapAfterSegment = -1;
        (Double X, Double Y) MaximumGapStart = (0.0, 0.0);
        (Double X, Double Y) MaximumGapEnd = (0.0, 0.0);
        List<(Double X, Double Y)> DrivenPoints = new List<(Double X, Double Y)>();
        Double CheckSpacingM = Math.Min(0.10, Grid.GridSizeM / 4.0);
        ExperimentalFieldEdgePolicy EdgePolicy =
            new ExperimentalFieldEdgePolicy(Grid, EdgeAllowanceM);

        for (Int32 SegmentIndex = 0; SegmentIndex < Lifecycle.Segments.Count; SegmentIndex += 1)
        {
            ExperimentalLifecyclePlanner.Segment Segment = Lifecycle.Segments[SegmentIndex];
            foreach ((Double X, Double Y) Point in Segment.Points)
            {
                if ((DrivenPoints.Count == 0)
                    || (Distance(DrivenPoints[DrivenPoints.Count - 1], Point) > 1e-6))
                {
                    DrivenPoints.Add(Point);
                }
            }
            OutsideSampleCount += Segment.Points.Count(Point =>
                !Grid.ContainsWorldXy(Point.X, Point.Y));
            BeyondAllowanceSampleCount += Segment.Points.Count(Point =>
                !EdgePolicy.ContainsWithExteriorAllowance(Point.X, Point.Y));
            for (Int32 PointIndex = 1; PointIndex < Segment.Points.Count; PointIndex += 1)
            {
                Int32 OutsideOnLeg = CountOutsideAlongLine(
                    Segment.Points[PointIndex - 1],
                    Segment.Points[PointIndex],
                    CheckSpacingM,
                    Grid);
                OutsideInterpolatedCount += OutsideOnLeg;
                Int32 BeyondAllowanceOnLeg = CountBeyondAllowanceAlongLine(
                    Segment.Points[PointIndex - 1],
                    Segment.Points[PointIndex],
                    CheckSpacingM,
                    EdgePolicy);
                BeyondAllowanceInterpolatedCount += BeyondAllowanceOnLeg;
                if ((BeyondAllowanceOnLeg > 0) && (ReportedOutsideLegs < 20))
                {
                    Console.WriteLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "outside_leg segment={0} phase={1} leg={2} checks={3} from=({4:F2},{5:F2}) to=({6:F2},{7:F2})",
                        SegmentIndex,
                        Segment.Phase,
                        PointIndex - 1,
                        BeyondAllowanceOnLeg,
                        Segment.Points[PointIndex - 1].X,
                        Segment.Points[PointIndex - 1].Y,
                        Segment.Points[PointIndex].X,
                        Segment.Points[PointIndex].Y));
                    ReportedOutsideLegs += 1;
                }
            }

            if ((SegmentIndex + 1) >= Lifecycle.Segments.Count
                || Segment.Points.Count == 0
                || Lifecycle.Segments[SegmentIndex + 1].Points.Count == 0)
            {
                continue;
            }
            (Double X, Double Y) A = Segment.Points[Segment.Points.Count - 1];
            (Double X, Double Y) B = Lifecycle.Segments[SegmentIndex + 1].Points[0];
            Double GapM = Math.Sqrt(
                ((B.X - A.X) * (B.X - A.X))
                + ((B.Y - A.Y) * (B.Y - A.Y)));
            if (GapM > MaximumBoundaryGapM)
            {
                MaximumBoundaryGapM = GapM;
                MaximumBoundaryGapAfterSegment = SegmentIndex;
                MaximumGapStart = A;
                MaximumGapEnd = B;
            }
            Int32 OutsideAcrossBoundary = CountOutsideAlongLine(A, B, CheckSpacingM, Grid);
            OutsideInterpolatedCount += OutsideAcrossBoundary;
            Int32 BeyondAllowanceAcrossBoundary = CountBeyondAllowanceAlongLine(
                A, B, CheckSpacingM, EdgePolicy);
            BeyondAllowanceInterpolatedCount += BeyondAllowanceAcrossBoundary;
            if ((BeyondAllowanceAcrossBoundary > 0) && (ReportedOutsideLegs < 20))
            {
                Console.WriteLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "outside_boundary after_segment={0} checks={1} from=({2:F2},{3:F2}) to=({4:F2},{5:F2})",
                    SegmentIndex,
                    BeyondAllowanceAcrossBoundary,
                    A.X,
                    A.Y,
                    B.X,
                    B.Y));
                ReportedOutsideLegs += 1;
            }
        }

        ExperimentalLifecyclePlanner.Segment? Start = Lifecycle.Segments
            .FirstOrDefault(Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.Start);
        ExperimentalLifecyclePlanner.Segment? End = Lifecycle.Segments
            .LastOrDefault(Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.End);
        (Double X, Double Y)? StartPoint = Start?.Points.FirstOrDefault();
        (Double X, Double Y)? EndPoint = End?.Points.LastOrDefault();
        Double MaximumHeadingChangeDeg = 0.0;
        Int32 CuspCount = 0;
        for (Int32 Index = 1; Index + 1 < DrivenPoints.Count; Index += 1)
        {
            Double ChangeDeg = HeadingChangeDeg(
                DrivenPoints[Index - 1], DrivenPoints[Index], DrivenPoints[Index + 1]);
            MaximumHeadingChangeDeg = Math.Max(MaximumHeadingChangeDeg, ChangeDeg);
            if (ChangeDeg >= 90.0) CuspCount += 1;
        }
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "geometry outside_field_samples={0} outside_field_interpolated={1} beyond_allowance_samples={2} beyond_allowance_interpolated={3} max_segment_gap_m={4:F3} gap_after_segment={5} max_heading_change_deg={6:F1} cusps={7}",
            OutsideSampleCount,
            OutsideInterpolatedCount,
            BeyondAllowanceSampleCount,
            BeyondAllowanceInterpolatedCount,
            MaximumBoundaryGapM,
            MaximumBoundaryGapAfterSegment,
            MaximumHeadingChangeDeg,
            CuspCount));
        if (MaximumBoundaryGapM > 1e-6)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "geometry_gap after_segment={0} from=({1:F2},{2:F2}) to=({3:F2},{4:F2})",
                MaximumBoundaryGapAfterSegment,
                MaximumGapStart.X,
                MaximumGapStart.Y,
                MaximumGapEnd.X,
                MaximumGapEnd.Y));
        }
        if (StartPoint.HasValue)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "route_start=({0:F1},{1:F1}) in_field={2}",
                StartPoint.Value.X,
                StartPoint.Value.Y,
                Grid.ContainsWorldXy(StartPoint.Value.X, StartPoint.Value.Y)));
        }
        if (EndPoint.HasValue)
        {
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "route_end=({0:F1},{1:F1}) in_field={2}",
                EndPoint.Value.X,
                EndPoint.Value.Y,
                Grid.ContainsWorldXy(EndPoint.Value.X, EndPoint.Value.Y)));
        }
    }

    /// <summary>Counts invalid interpolation checks between two adjacent route samples.</summary>
    private static Int32 CountOutsideAlongLine
        (
        (Double X, Double Y) A,
        (Double X, Double Y) B,
        Double SpacingM,
        Grid Grid
        )
    {
        Double DistanceM = Math.Sqrt(
            ((B.X - A.X) * (B.X - A.X))
            + ((B.Y - A.Y) * (B.Y - A.Y)));
        Int32 StepCount = Math.Max(1, (Int32)Math.Ceiling(DistanceM / SpacingM));
        Int32 Outside = 0;
        for (Int32 Step = 0; Step <= StepCount; Step += 1)
        {
            Double T = (Double)Step / StepCount;
            Double X = A.X + ((B.X - A.X) * T);
            Double Y = A.Y + ((B.Y - A.Y) * T);
            if (!Grid.ContainsWorldXy(X, Y)) Outside += 1;
        }
        return Outside;
    }

    /// <summary>Counts interpolation checks beyond the permitted outer-field strip.</summary>
    private static Int32 CountBeyondAllowanceAlongLine
        (
        (Double X, Double Y) A,
        (Double X, Double Y) B,
        Double SpacingM,
        ExperimentalFieldEdgePolicy EdgePolicy
        )
    {
        Double DistanceM = Math.Sqrt(
            ((B.X - A.X) * (B.X - A.X))
            + ((B.Y - A.Y) * (B.Y - A.Y)));
        Int32 StepCount = Math.Max(1, (Int32)Math.Ceiling(DistanceM / SpacingM));
        Int32 Outside = 0;
        for (Int32 Step = 0; Step <= StepCount; Step += 1)
        {
            Double T = (Double)Step / StepCount;
            Double X = A.X + ((B.X - A.X) * T);
            Double Y = A.Y + ((B.Y - A.Y) * T);
            if (!EdgePolicy.ContainsWithExteriorAllowance(X, Y)) Outside += 1;
        }
        return Outside;
    }

    /// <summary>
    /// Builds isolated straight coverage and sequences for every ordered TheShop2 pass.
    /// </summary>
    private static List<ExperimentalLifecyclePlanner.PassPlan> BuildWholePlanPasses
        (
        Route Route,
        Grid Grid,
        Double BladeM,
        Boolean GradeReturn,
        Int32? PassLimit = null,
        Action<String, Int32, Int32>? Progress = null,
        Action<String>? Diagnostic = null,
        Boolean StopOnFailure = false
        )
    {
        Int32 PassCount = PassLimit.HasValue
            ? Math.Min(Route.OrderedPasses.Count, Math.Max(0, PassLimit.Value))
            : Route.OrderedPasses.Count;
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "whole_plan building_passes={0}",
            PassCount));
        ExperimentalCoveragePlanner.PlanResult?[] CutPlans =
            new ExperimentalCoveragePlanner.PlanResult?[PassCount];
        ExperimentalCoveragePlanner.PlanResult?[] FillPlans =
            new ExperimentalCoveragePlanner.PlanResult?[PassCount];
        ExperimentalRunSequencer.Result?[] PreselectedFillSequences =
            new ExperimentalRunSequencer.Result?[PassCount];
        String?[] Failures = new String?[PassCount];
        Int32 CoverageCompleted = 0;
        Progress?.Invoke("coverage", 0, PassCount);
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
            Progress?.Invoke(
                "coverage",
                Interlocked.Increment(ref CoverageCompleted),
                PassCount);
        });
        List<ExperimentalLifecyclePlanner.PassPlan> Output =
            new List<ExperimentalLifecyclePlanner.PassPlan>(PassCount);
        ExperimentalConnectorPlanner.Pose? Current = null;
        Progress?.Invoke("sequencing", 0, PassCount);
        for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
        {
            Stopwatch PassTimer = Stopwatch.StartNew();
            Stopwatch StageTimer = Stopwatch.StartNew();
            void ReportStage(String Stage)
            {
                if (StageTimer.Elapsed >= TimeSpan.FromSeconds(0.25))
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan pass_stage pass={0} stage={1} elapsed_s={2:F3}",
                        PassIndex + 1,
                        Stage,
                        StageTimer.Elapsed.TotalSeconds));
                }
                StageTimer.Restart();
            }
            ExperimentalCoveragePlanner.PlanResult Cut = CutPlans[PassIndex]
                ?? throw new InvalidOperationException("Parallel coverage construction omitted cut work.");
            ExperimentalCoveragePlanner.PlanResult Fill = FillPlans[PassIndex]
                ?? throw new InvalidOperationException("Parallel coverage construction omitted fill work.");
            Progress?.Invoke(
                String.Format(
                    CultureInfo.InvariantCulture,
                    "sequencing pass={0} cut_runs={1} fill_runs={2}",
                    PassIndex + 1,
                    Cut.Runs.Count,
                    Fill.Runs.Count),
                PassIndex,
                PassCount);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "whole_plan pass={0} cut_runs={1} fill_runs={2} cut_omitted={3} fill_omitted={4}",
                PassIndex + 1,
                Cut.Runs.Count,
                Fill.Runs.Count,
                Cut.OmittedCells.Count,
                Fill.OmittedCells.Count));
            ExperimentalRunSequencer.Result FillSequence =
                PreselectedFillSequences[PassIndex]
                ?? BuildSequence(Fill.Runs, Grid);
            if (GradeReturn && !SupportsGradeReturn(FillSequence, Grid))
            {
                foreach (ExperimentalCoveragePlanner.PlanResult Candidate in
                    MinimalOmissionCandidates(
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
            ReportStage("initial_fill_and_grade_selection");
            ExperimentalConnectorPlanner.Pose? FillEntry = FillSequence.Runs.Count == 0
                ? null
                : FillSequence.Runs[0].EntryPose;
            // The next cut is part of the same continuous forward-only track.
            // Anchor its ordering to the actual post-grade pose so final
            // connectivity is selected, not merely checked after the fact.
            ExperimentalConnectorPlanner.Pose? CutSearchStart = Current;
            ExperimentalRunSequencer.Result CutSequence = BuildSequence(
                Cut.Runs, Grid, CutSearchStart, FillEntry);
            if (!CutSequence.Success || !SequencesConnect(CutSequence, FillSequence, Grid))
            {
                foreach (ExperimentalCoveragePlanner.PlanResult Candidate in
                    MinimalOmissionCandidates(
                        Route.OrderedPasses[PassIndex].CutCells,
                        Grid,
                        BladeM,
                        Current.HasValue ? (Current.Value.X, Current.Value.Y) : null,
                        FillEntry.HasValue ? (FillEntry.Value.X, FillEntry.Value.Y) : null)
                    .Take(MAX_INITIAL_CUT_CANDIDATE_ATTEMPTS))
                {
                    ExperimentalRunSequencer.Result CandidateSequence = BuildSequence(
                        Candidate.Runs, Grid, CutSearchStart, FillEntry);
                    if (!CandidateSequence.Success
                        || !SequencesConnect(CandidateSequence, FillSequence, Grid)) continue;
                    Cut = Candidate;
                    CutSequence = CandidateSequence;
                    break;
                }
            }
            ReportStage("initial_cut_selection");
            if (!CutSequence.Success || !SequencesConnect(CutSequence, FillSequence, Grid))
            {
                List<ExperimentalCoveragePlanner.PlanResult> FillCandidates =
                    MinimalOmissionCandidates(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM);
                List<ExperimentalCoveragePlanner.PlanResult> CutCandidates =
                    MinimalOmissionCandidates(
                        Route.OrderedPasses[PassIndex].CutCells,
                        Grid,
                        BladeM,
                        Current.HasValue ? (Current.Value.X, Current.Value.Y) : null,
                        null);
                Boolean SelectedPair = false;
                Int32 SelectedFillIndex = -1;
                Int32 SelectedCutIndex = -1;
                Int32 PairAttempts = 0;
                Diagnostic?.Invoke(String.Format(
                    CultureInfo.InvariantCulture,
                    "whole_plan pair_candidates pass={0} fills={1} cuts={2}",
                    PassIndex + 1,
                    FillCandidates.Count,
                    CutCandidates.Count));
                ExperimentalRunSequencer.Result?[] ProvisionalFillSequences =
                    new ExperimentalRunSequencer.Result?[FillCandidates.Count];
                List<(Int32 FillIndex, Int32 CutIndex)> CandidatePairs =
                    Enumerable.Range(
                        0,
                        CutCandidates.Count + FillCandidates.Count - 1)
                        .SelectMany(RankSum =>
                            Enumerable.Range(
                                Math.Max(0, RankSum - FillCandidates.Count + 1),
                                Math.Min(CutCandidates.Count - 1, RankSum)
                                    - Math.Max(
                                        0,
                                        RankSum - FillCandidates.Count + 1)
                                    + 1)
                                .Select(CutIndex =>
                                    (FillIndex: RankSum - CutIndex, CutIndex)))
                        .ToList();
                foreach ((Int32 FillIndex, Int32 CutIndex) Pair in CandidatePairs)
                {
                    if (PairAttempts >= MAX_PAIRED_CANDIDATE_ATTEMPTS) break;
                    PairAttempts += 1;
                    ExperimentalCoveragePlanner.PlanResult FillCandidate =
                        FillCandidates[Pair.FillIndex];
                    ExperimentalCoveragePlanner.PlanResult CutCandidate =
                        CutCandidates[Pair.CutIndex];
                    ExperimentalRunSequencer.Result ProvisionalFillSequence =
                        ProvisionalFillSequences[Pair.FillIndex]
                            ?? BuildSequence(FillCandidate.Runs, Grid);
                    ProvisionalFillSequences[Pair.FillIndex] =
                        ProvisionalFillSequence;
                    if (!ProvisionalFillSequence.Success
                        || (ProvisionalFillSequence.Runs.Count == 0))
                    {
                        continue;
                    }
                    ExperimentalConnectorPlanner.Pose CandidateFillEntry =
                        ProvisionalFillSequence.Runs[0].EntryPose;
                    ExperimentalRunSequencer.Result CandidateCutSequence =
                        BuildSequence(
                            CutCandidate.Runs,
                            Grid,
                            CutSearchStart,
                            CandidateFillEntry);
                    if (!CandidateCutSequence.Success
                        || (CandidateCutSequence.Runs.Count == 0))
                    {
                        continue;
                    }
                    ExperimentalConnectorPlanner.Pose CandidateCutExit =
                        CandidateCutSequence.Runs[
                            CandidateCutSequence.Runs.Count - 1].ExitPose;
                    ExperimentalRunSequencer.Result CandidateFillSequence =
                        BuildSequence(
                            FillCandidate.Runs,
                            Grid,
                            CandidateCutExit,
                            null);
                    if (GradeReturn
                        && CandidateFillSequence.Success
                        && (CandidateFillSequence.Runs.Count > 1))
                    {
                        CandidateFillSequence =
                            ExperimentalLifecyclePlanner.BuildContinuousFillSequence(
                                CandidateFillSequence,
                                CandidateCutExit,
                                Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                                    * Constants.FEET_TO_METERS,
                                Math.Min(0.5, Grid.GridSizeM),
                                Grid);
                    }
                    if (!CandidateFillSequence.Success
                        || (GradeReturn
                            && !SupportsGradeReturn(
                                CandidateFillSequence,
                                Grid,
                                FillSequencePrepared: true))
                        || !SequencesConnect(
                            CandidateCutSequence,
                            CandidateFillSequence,
                            Grid))
                    {
                        continue;
                    }
                    Cut = CutCandidate;
                    Fill = FillCandidate;
                    CutSequence = CandidateCutSequence;
                    FillSequence = CandidateFillSequence;
                    SelectedPair = true;
                    SelectedFillIndex = Pair.FillIndex;
                    SelectedCutIndex = Pair.CutIndex;
                    break;
                }
                Diagnostic?.Invoke(String.Format(
                    CultureInfo.InvariantCulture,
                    "whole_plan pair_result pass={0} attempts={1} selected={2} fill_index={3} cut_index={4} truncated={5}",
                    PassIndex + 1,
                    PairAttempts,
                    SelectedPair,
                    SelectedFillIndex,
                    SelectedCutIndex,
                    !SelectedPair && (PairAttempts < CandidatePairs.Count)));
            }
            ReportStage("paired_cut_fill_recovery");
            if (CutSequence.Success
                && (CutSequence.Runs.Count > 0)
                && (Fill.Runs.Count > 0))
            {
                ExperimentalConnectorPlanner.Pose CutExit =
                    CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose;
                ExperimentalConnectorPlanner.Pose? NextWorkEntry = null;
                if ((PassIndex + 1) < PassCount)
                {
                    ExperimentalCoveragePlanner.PlanResult? NextCut = CutPlans[PassIndex + 1];
                    if ((NextCut != null) && (NextCut.Runs.Count > 0))
                    {
                        ExperimentalRunSequencer.Result NextCutSequence =
                            BuildSequence(NextCut.Runs, Grid);
                        if (NextCutSequence.Success && (NextCutSequence.Runs.Count > 0))
                        {
                            NextWorkEntry = NextCutSequence.Runs[0].EntryPose;
                        }
                    }
                    if (!NextWorkEntry.HasValue)
                    {
                        ExperimentalCoveragePlanner.PlanResult? NextFill =
                            FillPlans[PassIndex + 1];
                        if ((NextFill != null) && (NextFill.Runs.Count > 0))
                        {
                            ExperimentalRunSequencer.Result NextFillSequence =
                                BuildSequence(NextFill.Runs, Grid);
                            if (NextFillSequence.Success
                                && (NextFillSequence.Runs.Count > 0))
                            {
                                NextWorkEntry = NextFillSequence.Runs[0].EntryPose;
                            }
                        }
                    }
                }
                ExperimentalRunSequencer.Result ConnectedFillSequence =
                    BuildSequence(Fill.Runs, Grid, CutExit, NextWorkEntry);
                if (ConnectedFillSequence.Success
                    && (!GradeReturn || SupportsGradeReturn(ConnectedFillSequence, Grid))
                    && SequencesConnect(CutSequence, ConnectedFillSequence, Grid)
                    && (!NextWorkEntry.HasValue
                        || SequenceConnectsToPose(
                            ConnectedFillSequence,
                            NextWorkEntry.Value,
                            Grid)))
                {
                    FillSequence = ConnectedFillSequence;
                }
                if (((PassIndex + 1) < PassCount)
                    && ((CutPlans[PassIndex + 1]?.Runs.Count ?? 0) == 0)
                    && (FillPlans[PassIndex + 1]?.Runs.Count ?? 0) > 0
                    && TrySelectAdjacentFills(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Route.OrderedPasses[PassIndex + 1].FillCells,
                        Grid,
                        BladeM,
                        CutExit,
                        GradeReturn,
                        out ExperimentalCoveragePlanner.PlanResult SelectedCurrentFill,
                        out ExperimentalRunSequencer.Result SelectedCurrentFillSequence,
                        out ExperimentalCoveragePlanner.PlanResult SelectedNextFill,
                        out ExperimentalRunSequencer.Result SelectedNextFillSequence))
                {
                    Fill = SelectedCurrentFill;
                    FillSequence = SelectedCurrentFillSequence;
                    FillPlans[PassIndex + 1] = SelectedNextFill;
                    PreselectedFillSequences[PassIndex + 1] = SelectedNextFillSequence;
                }
            }
            ReportStage("connected_and_adjacent_fill_selection");
            if (CutSequence.Success
                && (CutSequence.Runs.Count == 0)
                && Current.HasValue
                && (Fill.Runs.Count > 0)
                && ((PassIndex + 1) < PassCount)
                && ((CutPlans[PassIndex + 1]?.Runs.Count ?? 0) == 0)
                && (FillPlans[PassIndex + 1]?.Runs.Count ?? 0) > 0
                && TrySelectAdjacentFills(
                    Route.OrderedPasses[PassIndex].FillCells,
                    Route.OrderedPasses[PassIndex + 1].FillCells,
                    Grid,
                    BladeM,
                    Current.Value,
                    GradeReturn,
                    out ExperimentalCoveragePlanner.PlanResult ChainedCurrentFill,
                    out ExperimentalRunSequencer.Result ChainedCurrentFillSequence,
                    out ExperimentalCoveragePlanner.PlanResult ChainedNextFill,
                    out ExperimentalRunSequencer.Result ChainedNextFillSequence))
            {
                Fill = ChainedCurrentFill;
                FillSequence = ChainedCurrentFillSequence;
                FillPlans[PassIndex + 1] = ChainedNextFill;
                PreselectedFillSequences[PassIndex + 1] = ChainedNextFillSequence;
            }
            if (CutSequence.Success
                && (CutSequence.Runs.Count == 0)
                && Current.HasValue
                && (Fill.Runs.Count > 0)
                && !PoseConnectsToSequence(Current.Value, FillSequence, Grid))
            {
                foreach (ExperimentalCoveragePlanner.PlanResult FillCandidate in
                    MinimalOmissionCandidates(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM,
                        (Current.Value.X, Current.Value.Y),
                        null))
                {
                    ExperimentalRunSequencer.Result CandidateFillSequence =
                        BuildSequence(FillCandidate.Runs, Grid, Current, null);
                    if (!CandidateFillSequence.Success
                        || !PoseConnectsToSequence(
                            Current.Value,
                            CandidateFillSequence,
                            Grid)
                        || (GradeReturn
                            && !SupportsGradeReturn(CandidateFillSequence, Grid)))
                    {
                        continue;
                    }
                    Fill = FillCandidate;
                    FillSequence = CandidateFillSequence;
                    break;
                }
                if (!PoseConnectsToSequence(Current.Value, FillSequence, Grid)
                    && (PassIndex > 0)
                    && (Output.Count == PassIndex))
                {
                    ExperimentalLifecyclePlanner.PassPlan PreviousPass =
                        Output[PassIndex - 1];
                    if ((PreviousPass.Cut.Runs.Count > 0)
                        && TrySelectAdjacentFills(
                            Route.OrderedPasses[PassIndex - 1].FillCells,
                            Route.OrderedPasses[PassIndex].FillCells,
                            Grid,
                            BladeM,
                            PreviousPass.Cut.Runs[
                                PreviousPass.Cut.Runs.Count - 1].ExitPose,
                            GradeReturn,
                            out ExperimentalCoveragePlanner.PlanResult BacktrackedPreviousFill,
                            out ExperimentalRunSequencer.Result BacktrackedPreviousSequence,
                            out ExperimentalCoveragePlanner.PlanResult BacktrackedCurrentFill,
                            out ExperimentalRunSequencer.Result BacktrackedCurrentSequence))
                    {
                        Output[PassIndex - 1] =
                            new ExperimentalLifecyclePlanner.PassPlan(
                                PreviousPass.Cut,
                                BacktrackedPreviousSequence,
                                PreviousPass.CutOmittedCellCount,
                                BacktrackedPreviousFill.OmittedCells.Count);
                        FillPlans[PassIndex - 1] = BacktrackedPreviousFill;
                        FillPlans[PassIndex] = BacktrackedCurrentFill;
                        PreselectedFillSequences[PassIndex] = BacktrackedCurrentSequence;
                        Fill = BacktrackedCurrentFill;
                        FillSequence = BacktrackedCurrentSequence;
                        Current = GradeReturn
                            ? ReversePose(BacktrackedPreviousSequence.Runs[0].EntryPose)
                            : BacktrackedPreviousSequence.Runs[
                                BacktrackedPreviousSequence.Runs.Count - 1].ExitPose;
                    }
                }
                if (!PoseConnectsToSequence(Current.Value, FillSequence, Grid)
                    && (PassIndex > 0)
                    && (Output.Count == PassIndex))
                {
                    ExperimentalLifecyclePlanner.PassPlan PreviousPass =
                        Output[PassIndex - 1];
                    if ((PreviousPass.Cut.Runs.Count > 0)
                        && TryBacktrackPreviousFillAndCurrentPass(
                            Route.OrderedPasses[PassIndex - 1].FillCells,
                            Route.OrderedPasses[PassIndex].CutCells,
                            Route.OrderedPasses[PassIndex].FillCells,
                            Grid,
                            BladeM,
                            PreviousPass.Cut.Runs[
                                PreviousPass.Cut.Runs.Count - 1].ExitPose,
                            GradeReturn,
                            out ExperimentalCoveragePlanner.PlanResult TriplePreviousFill,
                            out ExperimentalRunSequencer.Result TriplePreviousSequence,
                            out ExperimentalCoveragePlanner.PlanResult TripleCut,
                            out ExperimentalRunSequencer.Result TripleCutSequence,
                            out ExperimentalCoveragePlanner.PlanResult TripleFill,
                            out ExperimentalRunSequencer.Result TripleFillSequence))
                    {
                        Output[PassIndex - 1] =
                            new ExperimentalLifecyclePlanner.PassPlan(
                                PreviousPass.Cut,
                                TriplePreviousSequence,
                                PreviousPass.CutOmittedCellCount,
                                TriplePreviousFill.OmittedCells.Count);
                        FillPlans[PassIndex - 1] = TriplePreviousFill;
                        CutPlans[PassIndex] = TripleCut;
                        FillPlans[PassIndex] = TripleFill;
                        Cut = TripleCut;
                        CutSequence = TripleCutSequence;
                        Fill = TripleFill;
                        FillSequence = TripleFillSequence;
                        Current = GradeReturn
                            ? ReversePose(TriplePreviousSequence.Runs[0].EntryPose)
                            : TriplePreviousSequence.Runs[
                                TriplePreviousSequence.Runs.Count - 1].ExitPose;
                    }
                }
                if (!PoseConnectsToSequence(Current.Value, FillSequence, Grid)
                    && TryFillWithMinimalAdditionalEdgeOmission(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM,
                        Current.Value,
                        GradeReturn,
                        out ExperimentalCoveragePlanner.PlanResult TrimmedFill,
                        out ExperimentalRunSequencer.Result TrimmedFillSequence))
                {
                    Fill = TrimmedFill;
                    FillSequence = TrimmedFillSequence;
                }
            }
            ReportStage("fill_backtracking_and_omission");
            if (GradeReturn && FillSequence.Success && (FillSequence.Runs.Count > 1))
            {
                ExperimentalConnectorPlanner.Pose? FillInbound =
                    CutSequence.Success && (CutSequence.Runs.Count > 0)
                        ? CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose
                        : Current;
                FillSequence = ExperimentalLifecyclePlanner.BuildContinuousFillSequence(
                    FillSequence,
                    FillInbound,
                    Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                        * Constants.FEET_TO_METERS,
                    Math.Min(0.5, Grid.GridSizeM),
                    Grid);
            }
            ReportStage("continuous_fill_compaction");
            if (!CutSequence.Success)
            {
                CutSequence = BuildFlexibleSequence(
                    Cut.Runs,
                    Grid,
                    CutSearchStart,
                    FillSequence.Success && (FillSequence.Runs.Count > 0)
                        ? FillSequence.Runs[0].EntryPose
                        : FillEntry);
            }
            if (!CutSequence.Success)
            {
                ExperimentalRunSequencer.Result UnanchoredCut =
                    BuildFlexibleSequence(Cut.Runs, Grid);
                if (UnanchoredCut.Success && (UnanchoredCut.Runs.Count > 0))
                {
                    ExperimentalRunSequencer.Result ConnectedFill =
                        BuildFlexibleSequence(
                            Fill.Runs,
                            Grid,
                            UnanchoredCut.Runs[
                                UnanchoredCut.Runs.Count - 1].ExitPose,
                            null);
                    if (ConnectedFill.Success
                        && SequencesConnect(UnanchoredCut, ConnectedFill, Grid))
                    {
                        CutSequence = UnanchoredCut;
                        FillSequence = ConnectedFill;
                    }
                }
            }
            if (FillSequence.Success
                && (FillSequence.Runs.Count > 0)
                && (!CutSequence.Success
                    || (CutSequence.Runs.Count == 0)
                    || !SequencesConnect(CutSequence, FillSequence, Grid))
                && TrySelectFeasibleCutCoverageCandidate(
                    Route.OrderedPasses[PassIndex].CutCells,
                    Grid,
                    BladeM,
                    CutSearchStart,
                    FillSequence,
                    out ExperimentalCoveragePlanner.PlanResult AlternativeCut,
                    out ExperimentalRunSequencer.Result AlternativeCutSequence))
            {
                Diagnostic?.Invoke(String.Format(
                    CultureInfo.InvariantCulture,
                    "whole_plan alternative_cut_coverage pass={0} heading={1:F1} runs={2} omitted_cells={3}",
                    PassIndex + 1,
                    AlternativeCut.HeadingDeg,
                    AlternativeCutSequence.Runs.Count,
                    AlternativeCut.OmittedCells.Count));
                Cut = AlternativeCut;
                CutSequence = AlternativeCutSequence;
            }
            ReportStage("alternative_cut_coverage_recovery");
            if (CutSequence.Success
                && (CutSequence.Runs.Count > 0)
                && (!FillSequence.Success
                    || (FillSequence.Runs.Count == 0)
                    || !SequencesConnect(CutSequence, FillSequence, Grid)))
            {
                FillSequence = BuildFlexibleSequence(
                    Fill.Runs,
                    Grid,
                    CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose,
                    null);
            }
            ReportStage("flexible_sequence_recovery");
            if (CutSequence.Success
                && (Fill.Runs.Count > 0)
                && (!FillSequence.Success
                    || (FillSequence.Runs.Count == 0)
                    || !SequencesConnect(CutSequence, FillSequence, Grid)))
            {
                ExperimentalConnectorPlanner.Pose? AlternativeFillStart =
                    CutSequence.Runs.Count > 0
                        ? CutSequence.Runs[
                            CutSequence.Runs.Count - 1].ExitPose
                        : Current;
                if (AlternativeFillStart.HasValue
                    && TrySelectFeasibleFillCoverageCandidate(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM,
                        AlternativeFillStart.Value,
                        GradeReturn,
                        out ExperimentalCoveragePlanner.PlanResult AlternativeFill,
                        out ExperimentalRunSequencer.Result AlternativeFillSequence))
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan alternative_fill_coverage pass={0} heading={1:F1} runs={2} omitted_cells={3}",
                        PassIndex + 1,
                        AlternativeFill.HeadingDeg,
                        AlternativeFillSequence.Runs.Count,
                        AlternativeFill.OmittedCells.Count));
                    Fill = AlternativeFill;
                    FillSequence = AlternativeFillSequence;
                }
            }
            ReportStage("alternative_fill_coverage_recovery");
            if (GradeReturn && FillSequence.Success && (FillSequence.Runs.Count > 1))
            {
                ExperimentalConnectorPlanner.Pose? FillInbound =
                    CutSequence.Success && (CutSequence.Runs.Count > 0)
                        ? CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose
                        : Current;
                FillSequence = ExperimentalLifecyclePlanner.BuildContinuousFillSequence(
                    FillSequence,
                    FillInbound,
                    Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                        * Constants.FEET_TO_METERS,
                    Math.Min(0.5, Grid.GridSizeM),
                    Grid);
            }
            ReportStage("post_flexible_fill_compaction");
            if (GradeReturn
                && FillSequence.Success
                && (FillSequence.Runs.Count > 0)
                && !SupportsGradeReturn(
                    FillSequence,
                    Grid,
                    FillSequencePrepared: true))
            {
                ExperimentalConnectorPlanner.Pose? GradeRecoveryStart =
                    CutSequence.Runs.Count > 0
                        ? CutSequence.Runs[
                            CutSequence.Runs.Count - 1].ExitPose
                        : Current;
                ExperimentalCoveragePlanner.PlanResult GradeFill = Fill;
                ExperimentalRunSequencer.Result GradeFillSequence =
                    FillSequence;
                Boolean GradeRecovered = GradeRecoveryStart.HasValue
                    && TrySelectFeasibleFillCoverageCandidate(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM,
                        GradeRecoveryStart.Value,
                        true,
                        out GradeFill,
                        out GradeFillSequence);
                if (GradeRecovered)
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan alternative_grade_coverage pass={0} heading={1:F1} runs={2} omitted_cells={3}",
                        PassIndex + 1,
                        GradeFill.HeadingDeg,
                        GradeFillSequence.Runs.Count,
                        GradeFill.OmittedCells.Count));
                    Fill = GradeFill;
                    FillSequence = GradeFillSequence;
                }
                else
                {
                    ExperimentalRunSequencer.SequencedRun LastFill =
                        FillSequence.Runs[FillSequence.Runs.Count - 1];
                    ExperimentalConnectorPlanner.Pose GradeEntry =
                        new ExperimentalConnectorPlanner.Pose(
                            LastFill.Run.EndXy.X,
                            LastFill.Run.EndXy.Y,
                            LastFill.Run.HeadingDeg + 180.0);
                    ExperimentalTransitPlanner.Result GradeTurn =
                        ExperimentalTransitPlanner.Connect(
                            LastFill.ExitPose,
                            GradeEntry,
                            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                                * Constants.FEET_TO_METERS,
                            Math.Min(0.5, Grid.GridSizeM),
                            Grid,
                            EdgeAllowanceM: Grid.GridSizeM,
                            AllowForwardDetour: true);
                    ExperimentalFieldEdgePolicy GradeEdgePolicy =
                        new ExperimentalFieldEdgePolicy(
                            Grid, Grid.GridSizeM);
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan grade_turn_failure pass={0} fill_runs={1} work_end=({2:F2},{3:F2}) exit=({4:F2},{5:F2},{6:F1}) grade_entry_heading={7:F1} near_edge={8} reason={9}",
                        PassIndex + 1,
                        FillSequence.Runs.Count,
                        LastFill.Run.EndXy.X,
                        LastFill.Run.EndXy.Y,
                        LastFill.ExitPose.X,
                        LastFill.ExitPose.Y,
                        LastFill.ExitPose.HeadingDeg,
                        GradeEntry.HeadingDeg,
                        GradeEdgePolicy.IsFieldPointNearOuterEdge(
                            LastFill.Run.EndXy.X,
                            LastFill.Run.EndXy.Y),
                        GradeTurn.FailureReason));
                    FillSequence = new ExperimentalRunSequencer.Result(
                        false,
                        FillSequence.Runs,
                        FillSequence.Connectors,
                        FillSequence.ConnectorDistanceM,
                        FillSequence.MinimumLineupM,
                        "Final fill sequence cannot execute its required grade return.");
                }
            }
            ReportStage("final_grade_validation");
            List<IReadOnlyCollection<(Int32 Bx, Int32 By)>>
                InfeasibleCutRunCellSets =
                    new List<IReadOnlyCollection<(Int32, Int32)>>();
            if (Current.HasValue
                && FillSequence.Success
                && (FillSequence.Runs.Count > 0)
                && (!CutSequence.Success
                    || (CutSequence.Runs.Count == 0)
                    || !SequencesConnect(CutSequence, FillSequence, Grid)))
            {
                ExperimentalRunSequencer.Result DetourCutSequence =
                    ExperimentalRunSequencer.OptimizeFlexibleEndpoints(
                        Cut.Runs,
                        Current.Value,
                        FillSequence.Runs[0].EntryPose,
                        Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                            * Constants.FEET_TO_METERS,
                        Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                            * Constants.FEET_TO_METERS,
                        Math.Min(0.5, Grid.GridSizeM),
                        Grid,
                        AllowForwardDetour: true);
                if (DetourCutSequence.Success
                    && PoseConnectsToSequence(
                        Current.Value, DetourCutSequence, Grid)
                    && SequencesConnect(
                        DetourCutSequence, FillSequence, Grid))
                {
                    CutSequence = DetourCutSequence;
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan infeasible_cut_forward_detour pass={0} runs={1} shortfall_m={2:F2}",
                        PassIndex + 1,
                        DetourCutSequence.Runs.Count,
                        DetourCutSequence.Runs.Sum(Run =>
                            Run.StartCoverageShortfallM
                                + Run.EndCoverageShortfallM)));
                }
                else
                {
                    foreach (Int32 RunIndex in
                        DetourCutSequence.InfeasibleRunIndices)
                    {
                        if ((RunIndex < 0) || (RunIndex >= Cut.Runs.Count))
                        {
                            continue;
                        }
                        InfeasibleCutRunCellSets.Add(Cut.Runs[RunIndex].Cells
                            .Select(Entry => Entry.Cell)
                            .Distinct()
                            .ToList());
                    }
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan infeasible_cut_forward_detour_failed pass={0} success={1} runs={2} start_connected={3} fill_connected={4} reason={5}",
                        PassIndex + 1,
                        DetourCutSequence.Success,
                        DetourCutSequence.Runs.Count,
                        DetourCutSequence.Success
                            && PoseConnectsToSequence(
                                Current.Value, DetourCutSequence, Grid),
                        DetourCutSequence.Success
                            && SequencesConnect(
                                DetourCutSequence, FillSequence, Grid),
                        DetourCutSequence.FailureReason));
                }
            }
            if (Current.HasValue
                && FillSequence.Success
                && (FillSequence.Runs.Count > 0)
                && (!CutSequence.Success
                    || (CutSequence.Runs.Count == 0)
                    || !SequencesConnect(CutSequence, FillSequence, Grid))
                && TryCutWithMinimalAdditionalEdgeOmission(
                    Route.OrderedPasses[PassIndex].CutCells,
                    Grid,
                    BladeM,
                    Current.Value,
                    Route.OrderedPasses[PassIndex].FillCells,
                    Fill,
                    FillSequence,
                    GradeReturn,
                    InfeasibleCutRunCellSets,
                    out ExperimentalCoveragePlanner.PlanResult RecoveredCut,
                    out ExperimentalRunSequencer.Result RecoveredCutSequence,
                    out ExperimentalCoveragePlanner.PlanResult RecoveredFill,
                    out ExperimentalRunSequencer.Result RecoveredFillSequence,
                    Diagnostic))
            {
                Cut = RecoveredCut;
                CutSequence = RecoveredCutSequence;
                CutPlans[PassIndex] = RecoveredCut;
                Fill = RecoveredFill;
                FillSequence = RecoveredFillSequence;
                FillPlans[PassIndex] = RecoveredFill;
                Diagnostic?.Invoke(String.Format(
                    CultureInfo.InvariantCulture,
                    "whole_plan infeasible_cut_edge_omission pass={0} heading={1:F1} runs={2} omitted_cells={3}",
                    PassIndex + 1,
                    RecoveredCut.HeadingDeg,
                    RecoveredCutSequence.Runs.Count,
                    RecoveredCut.OmittedCells.Count));
            }
            Boolean PreviousToFillConnected = !Current.HasValue
                || (CutSequence.Runs.Count > 0)
                || (FillSequence.Runs.Count == 0)
                || PoseConnectsToSequence(
                    Current.Value, FillSequence, Grid);
            Boolean PreviousToCutConnected = !Current.HasValue
                || (CutSequence.Runs.Count == 0)
                || PoseConnectsToSequence(
                    Current.Value, CutSequence, Grid);
            if (!PreviousToCutConnected
                && Current.HasValue
                && FillSequence.Success
                && (FillSequence.Runs.Count > 0)
                && TryCutWithMinimalAdditionalEdgeOmission(
                    Route.OrderedPasses[PassIndex].CutCells,
                    Grid,
                    BladeM,
                    Current.Value,
                    Route.OrderedPasses[PassIndex].FillCells,
                    Fill,
                    FillSequence,
                    GradeReturn,
                    Array.Empty<IReadOnlyCollection<(Int32 Bx, Int32 By)>>(),
                    out ExperimentalCoveragePlanner.PlanResult TrimmedCut,
                    out ExperimentalRunSequencer.Result TrimmedCutSequence,
                    out ExperimentalCoveragePlanner.PlanResult RecoveredInboundFill,
                    out ExperimentalRunSequencer.Result RecoveredInboundFillSequence,
                    Diagnostic))
            {
                Cut = TrimmedCut;
                CutSequence = TrimmedCutSequence;
                CutPlans[PassIndex] = TrimmedCut;
                Fill = RecoveredInboundFill;
                FillSequence = RecoveredInboundFillSequence;
                FillPlans[PassIndex] = RecoveredInboundFill;
                PreviousToCutConnected = true;
                PreviousToFillConnected = true;
                Diagnostic?.Invoke(String.Format(
                    CultureInfo.InvariantCulture,
                    "whole_plan inbound_cut_edge_omission pass={0} heading={1:F1} runs={2} omitted_cells={3}",
                    PassIndex + 1,
                    TrimmedCut.HeadingDeg,
                    TrimmedCutSequence.Runs.Count,
                    TrimmedCut.OmittedCells.Count));
            }
            if (!PreviousToCutConnected
                && (PassIndex > 0)
                && (Output.Count == PassIndex))
            {
                ExperimentalLifecyclePlanner.PassPlan PreviousPass =
                    Output[PassIndex - 1];
                if ((PreviousPass.Cut.Runs.Count > 0)
                    && TryBacktrackPreviousFillAndCurrentPass(
                        Route.OrderedPasses[PassIndex - 1].FillCells,
                        Route.OrderedPasses[PassIndex].CutCells,
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM,
                        PreviousPass.Cut.Runs[
                            PreviousPass.Cut.Runs.Count - 1].ExitPose,
                        GradeReturn,
                        out ExperimentalCoveragePlanner.PlanResult BacktrackedPreviousFill,
                        out ExperimentalRunSequencer.Result BacktrackedPreviousSequence,
                        out ExperimentalCoveragePlanner.PlanResult BacktrackedCut,
                        out ExperimentalRunSequencer.Result BacktrackedCutSequence,
                        out ExperimentalCoveragePlanner.PlanResult BacktrackedFill,
                        out ExperimentalRunSequencer.Result BacktrackedFillSequence,
                        Diagnostic))
                {
                    Output[PassIndex - 1] =
                        new ExperimentalLifecyclePlanner.PassPlan(
                            PreviousPass.Cut,
                            BacktrackedPreviousSequence,
                            PreviousPass.CutOmittedCellCount,
                            BacktrackedPreviousFill.OmittedCells.Count);
                    FillPlans[PassIndex - 1] = BacktrackedPreviousFill;
                    CutPlans[PassIndex] = BacktrackedCut;
                    FillPlans[PassIndex] = BacktrackedFill;
                    Cut = BacktrackedCut;
                    CutSequence = BacktrackedCutSequence;
                    Fill = BacktrackedFill;
                    FillSequence = BacktrackedFillSequence;
                    Current = GradeReturn
                        ? ReversePose(
                            BacktrackedPreviousSequence.Runs[0].EntryPose)
                        : BacktrackedPreviousSequence.Runs[
                            BacktrackedPreviousSequence.Runs.Count - 1].ExitPose;
                    PreviousToCutConnected = PoseConnectsToSequence(
                        Current.Value, CutSequence, Grid);
                    PreviousToFillConnected = (CutSequence.Runs.Count > 0)
                        || (FillSequence.Runs.Count == 0)
                        || PoseConnectsToSequence(
                            Current.Value, FillSequence, Grid);
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan inbound_cut_backtrack pass={0} previous_fill_heading={1:F1} cut_heading={2:F1} fill_heading={3:F1} omitted_cells={4}",
                        PassIndex + 1,
                        BacktrackedPreviousFill.HeadingDeg,
                        BacktrackedCut.HeadingDeg,
                        BacktrackedFill.HeadingDeg,
                        BacktrackedPreviousFill.OmittedCells.Count
                            + BacktrackedCut.OmittedCells.Count
                            + BacktrackedFill.OmittedCells.Count));
                }
            }
            if (!PreviousToCutConnected
                && (PassIndex > 0)
                && (Output.Count == PassIndex))
            {
                ExperimentalLifecyclePlanner.PassPlan PreviousPass =
                    Output[PassIndex - 1];
                ExperimentalConnectorPlanner.Pose? PreviousIncoming =
                    (PassIndex > 1)
                        ? PassOutgoingPose(Output[PassIndex - 2], GradeReturn)
                        : null;
                if ((PreviousPass.Cut.Runs.Count > 0)
                    && TryBacktrackPreviousCutAndCurrentPass(
                        PreviousPass.Cut,
                        Route.OrderedPasses[PassIndex - 1].FillCells,
                        Route.OrderedPasses[PassIndex].CutCells,
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM,
                        PreviousIncoming,
                        PreviousPass.Cut.Runs[
                            PreviousPass.Cut.Runs.Count - 1].ExitPose,
                        GradeReturn,
                        out ExperimentalRunSequencer.Result DeeperPreviousCut,
                        out ExperimentalCoveragePlanner.PlanResult DeeperPreviousFill,
                        out ExperimentalRunSequencer.Result DeeperPreviousFillSequence,
                        out ExperimentalCoveragePlanner.PlanResult DeeperCut,
                        out ExperimentalRunSequencer.Result DeeperCutSequence,
                        out ExperimentalCoveragePlanner.PlanResult DeeperFill,
                        out ExperimentalRunSequencer.Result DeeperFillSequence,
                        Diagnostic))
                {
                    Output[PassIndex - 1] =
                        new ExperimentalLifecyclePlanner.PassPlan(
                            DeeperPreviousCut,
                            DeeperPreviousFillSequence,
                            PreviousPass.CutOmittedCellCount,
                            DeeperPreviousFill.OmittedCells.Count);
                    FillPlans[PassIndex - 1] = DeeperPreviousFill;
                    CutPlans[PassIndex] = DeeperCut;
                    FillPlans[PassIndex] = DeeperFill;
                    Cut = DeeperCut;
                    CutSequence = DeeperCutSequence;
                    Fill = DeeperFill;
                    FillSequence = DeeperFillSequence;
                    Current = GradeReturn
                        ? ReversePose(
                            DeeperPreviousFillSequence.Runs[0].EntryPose)
                        : DeeperPreviousFillSequence.Runs[
                            DeeperPreviousFillSequence.Runs.Count - 1].ExitPose;
                    PreviousToCutConnected = PoseConnectsToSequence(
                        Current.Value, CutSequence, Grid);
                    PreviousToFillConnected = (CutSequence.Runs.Count > 0)
                        || (FillSequence.Runs.Count == 0)
                        || PoseConnectsToSequence(
                            Current.Value, FillSequence, Grid);
                    RetainSelectedLifecycleConnections(
                        PreviousIncoming,
                        DeeperPreviousCut,
                        DeeperPreviousFillSequence,
                        GradeReturn,
                        Grid);
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan inbound_cut_deep_backtrack pass={0} previous_cut_runs={1} previous_fill_heading={2:F1} cut_heading={3:F1} fill_heading={4:F1} omitted_cells={5}",
                        PassIndex + 1,
                        DeeperPreviousCut.Runs.Count,
                        DeeperPreviousFill.HeadingDeg,
                        DeeperCut.HeadingDeg,
                        DeeperFill.HeadingDeg,
                        DeeperPreviousFill.OmittedCells.Count
                            + DeeperCut.OmittedCells.Count
                            + DeeperFill.OmittedCells.Count));
                }
            }
            if (!CutSequence.Success
                || !FillSequence.Success
                || !SequencesConnect(CutSequence, FillSequence, Grid)
                || !PreviousToCutConnected
                || !PreviousToFillConnected)
            {
                if (CutSequence.Success
                    && FillSequence.Success
                    && (!SequencesConnect(CutSequence, FillSequence, Grid)
                        || !PreviousToCutConnected
                        || !PreviousToFillConnected))
                {
                    CutSequence = new ExperimentalRunSequencer.Result(
                        false,
                        CutSequence.Runs,
                        CutSequence.Connectors,
                        CutSequence.ConnectorDistanceM,
                        CutSequence.MinimumLineupM,
                        "No complete incoming connector to the pass's first remaining work satisfies the edge policy.");
                }
                Failures[PassIndex] = String.Format(
                    CultureInfo.InvariantCulture,
                        "whole_plan sequence_failure pass={0} cut_runs={1} fill_runs={2} cut_reason={3} fill_reason={4} cut_to_fill={5} previous_to_cut={6} previous_to_fill={7}",
                    PassIndex + 1,
                    Cut.Runs.Count,
                    Fill.Runs.Count,
                    CutSequence.FailureReason,
                    FillSequence.FailureReason,
                    SequencesConnect(CutSequence, FillSequence, Grid),
                    PreviousToCutConnected,
                    PreviousToFillConnected);
            }
            ReportStage("final_connectivity_validation");
            if ((Failures[PassIndex] == null)
                && CutSequence.Success
                && FillSequence.Success)
            {
                RetainSelectedLifecycleConnections(
                    Current,
                    CutSequence,
                    FillSequence,
                    GradeReturn,
                    Grid);
            }
            Output.Add(new ExperimentalLifecyclePlanner.PassPlan(
                CutSequence,
                FillSequence,
                Cut.OmittedCells.Count,
                Fill.OmittedCells.Count));
            Progress?.Invoke("sequencing", PassIndex + 1, PassCount);
            if ((PassTimer.Elapsed >= TimeSpan.FromSeconds(1.0))
                || (Failures[PassIndex] != null))
            {
                Diagnostic?.Invoke(String.Format(
                    CultureInfo.InvariantCulture,
                    "whole_plan pass_timing pass={0} elapsed_s={1:F3} cut_runs={2} fill_runs={3} success={4}",
                    PassIndex + 1,
                    PassTimer.Elapsed.TotalSeconds,
                    Cut.Runs.Count,
                    Fill.Runs.Count,
                    Failures[PassIndex] == null));
            }
            if (Failures[PassIndex] != null)
            {
                ExperimentalFieldEdgePolicy FailedEdgePolicy =
                    new ExperimentalFieldEdgePolicy(Grid, Grid.GridSizeM);
                for (Int32 RunIndex = 0;
                    RunIndex < Cut.Runs.Count;
                    RunIndex += 1)
                {
                    ExperimentalCoveragePlanner.Run FailedRun = Cut.Runs[RunIndex];
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan failed_cut_run pass={0} run={1} cells={2} length_m={3:F2} heading={4:F1} start=({5:F2},{6:F2}) end=({7:F2},{8:F2})",
                        PassIndex + 1,
                        RunIndex,
                        FailedRun.Cells.Count,
                        FailedRun.LengthM,
                        FailedRun.HeadingDeg,
                        FailedRun.StartXy.X,
                        FailedRun.StartXy.Y,
                        FailedRun.EndXy.X,
                        FailedRun.EndXy.Y));
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                        in FailedRun.Cells)
                    {
                        Boolean HasEdgeHeading =
                            FailedEdgePolicy.TryGetParallelHeadingDeg(
                                Entry.Cell,
                                out Double EdgeHeadingDeg);
                        Diagnostic?.Invoke(String.Format(
                            CultureInfo.InvariantCulture,
                            "whole_plan failed_cut_cell pass={0} run={1} cell=({2},{3}) edge={4} edge_heading={5}",
                            PassIndex + 1,
                            RunIndex,
                            Entry.Cell.Bx,
                            Entry.Cell.By,
                            FailedEdgePolicy.IsEdgeWorkCell(Entry.Cell),
                            HasEdgeHeading
                                ? EdgeHeadingDeg.ToString(
                                    "F1", CultureInfo.InvariantCulture)
                                : "none"));
                    }
                }
                for (Int32 RunIndex = 0;
                    RunIndex < Fill.Runs.Count;
                    RunIndex += 1)
                {
                    ExperimentalCoveragePlanner.Run FailedRun =
                        Fill.Runs[RunIndex];
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan failed_fill_run pass={0} run={1} cells={2} length_m={3:F2} heading={4:F1} start=({5:F2},{6:F2}) end=({7:F2},{8:F2})",
                        PassIndex + 1,
                        RunIndex,
                        FailedRun.Cells.Count,
                        FailedRun.LengthM,
                        FailedRun.HeadingDeg,
                        FailedRun.StartXy.X,
                        FailedRun.StartXy.Y,
                        FailedRun.EndXy.X,
                        FailedRun.EndXy.Y));
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                        in FailedRun.Cells)
                    {
                        Boolean HasEdgeHeading =
                            FailedEdgePolicy.TryGetParallelHeadingDeg(
                                Entry.Cell,
                                out Double EdgeHeadingDeg);
                        Diagnostic?.Invoke(String.Format(
                            CultureInfo.InvariantCulture,
                            "whole_plan failed_fill_cell pass={0} run={1} cell=({2},{3}) edge={4} edge_heading={5}",
                            PassIndex + 1,
                            RunIndex,
                            Entry.Cell.Bx,
                            Entry.Cell.By,
                            FailedEdgePolicy.IsEdgeWorkCell(Entry.Cell),
                            HasEdgeHeading
                                ? EdgeHeadingDeg.ToString(
                                    "F1", CultureInfo.InvariantCulture)
                                : "none"));
                    }
                }
                Diagnostic?.Invoke(Failures[PassIndex]!);
                if (StopOnFailure) break;
            }
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "whole_plan finalized_pass={0} cut_runs={1} fill_runs={2} cut_success={3} fill_success={4}",
                PassIndex + 1,
                CutSequence.Runs.Count,
                FillSequence.Runs.Count,
                CutSequence.Success,
                FillSequence.Success));
            if (CutSequence.Success
                && FillSequence.Success
                && (CutSequence.Runs.Count > 0)
                && (FillSequence.Runs.Count > 0))
            {
                ExperimentalTransitPlanner.Result Link = ExperimentalTransitPlanner.Connect(
                    CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose,
                    FillSequence.Runs[0].EntryPose,
                    Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
                    Math.Min(0.5, Grid.GridSizeM),
                    Grid);
                Console.WriteLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "pass_link pass={0} success={1} mode={2} start=({3:F1},{4:F1},{5:F1}) end=({6:F1},{7:F1},{8:F1}) reason={9}",
                    PassIndex + 1,
                    Link.Success,
                    Link.Mode,
                    CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose.X,
                    CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose.Y,
                    CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose.HeadingDeg,
                    FillSequence.Runs[0].EntryPose.X,
                    FillSequence.Runs[0].EntryPose.Y,
                    FillSequence.Runs[0].EntryPose.HeadingDeg,
                    Link.FailureReason));
            }
            if (CutSequence.Success && FillSequence.Success && (FillSequence.Runs.Count > 0))
            {
                ExperimentalRunSequencer.SequencedRun LastFill =
                    FillSequence.Runs[FillSequence.Runs.Count - 1];
                Current = LastFill.ExitPose;
                if (GradeReturn)
                {
                    List<ExperimentalRunSequencer.SequencedRun> GradeRuns =
                        ExperimentalLifecyclePlanner.BuildGradeRuns(
                            FillSequence.Runs,
                            Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                                * Constants.FEET_TO_METERS,
                            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                                * Constants.FEET_TO_METERS,
                            Math.Min(0.5, Grid.GridSizeM),
                            Grid);
                    Current = GradeRuns[GradeRuns.Count - 1].ExitPose;
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
        Grid Grid,
        Boolean FillSequencePrepared = false,
        Boolean RetainResults = false
        )
    {
        ExperimentalRunSequencer.Result Candidate = FillSequencePrepared
            ? FillSequence
            : ExperimentalLifecyclePlanner.BuildContinuousFillSequence(
                FillSequence,
                null,
                Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
                Math.Min(0.5, Grid.GridSizeM),
                Grid);
        return ExperimentalLifecyclePlanner.SupportsPreparedGradeReturn(
            Candidate,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid,
            out _,
            RetainResults);
    }

    /// <summary>
    /// Protects only final selected pass connectors from speculative candidate-cache
    /// churn so lifecycle generation reuses the exact validated guidance.
    /// </summary>
    private static void RetainSelectedLifecycleConnections
        (
        ExperimentalConnectorPlanner.Pose? Previous,
        ExperimentalRunSequencer.Result Cut,
        ExperimentalRunSequencer.Result Fill,
        Boolean GradeReturn,
        Grid Grid
        )
    {
        Double RadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT
            * 0.5 * Constants.FEET_TO_METERS;
        Double SampleM = Math.Min(0.5, Grid.GridSizeM);
        ExperimentalConnectorPlanner.Pose? First = Cut.Runs.Count > 0
            ? Cut.Runs[0].EntryPose
            : Fill.Runs.Count > 0
                ? Fill.Runs[0].EntryPose
                : null;
        if (Previous.HasValue && First.HasValue)
        {
            ExperimentalTransitPlanner.Connect(
                Previous.Value,
                First.Value,
                RadiusM,
                SampleM,
                Grid,
                EdgeAllowanceM: Grid.GridSizeM,
                AllowForwardDetour: true,
                RetainResult: true);
        }
        if ((Cut.Runs.Count > 0) && (Fill.Runs.Count > 0))
        {
            ExperimentalTransitPlanner.Connect(
                Cut.Runs[Cut.Runs.Count - 1].ExitPose,
                Fill.Runs[0].EntryPose,
                RadiusM,
                SampleM,
                Grid,
                EdgeAllowanceM: Grid.GridSizeM,
                AllowForwardDetour: true,
                RetainResult: true);
        }
        if (GradeReturn && (Fill.Runs.Count > 0))
        {
            SupportsGradeReturn(
                Fill,
                Grid,
                FillSequencePrepared: true,
                RetainResults: true);
        }
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
    /// Keeps every heading candidate in the smallest available omission tier.
    /// </summary>
    private static List<ExperimentalCoveragePlanner.PlanResult> MinimalOmissionCandidates
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        (Double X, Double Y)? Start = null,
        (Double X, Double Y)? End = null
        )
    {
        List<ExperimentalCoveragePlanner.PlanResult> Candidates =
            OrderedCoverageCandidates(Cells, Grid, BladeM, Start, End);
        Int32 MinimumOmittedCells = Candidates.Min(
            Candidate => Candidate.OmittedCells.Count);
        return Candidates
            .Where(Candidate => Candidate.OmittedCells.Count == MinimumOmittedCells)
            .ToList();
    }

    /// <summary>
    /// Returns each distinct coverage geometry once, ordered first by omitted work
    /// and then by the planner's deterministic travel score. Opposite packing
    /// directions frequently produce the same unordered run set; the run sequencer
    /// can orient and reorder that geometry, so evaluating it twice adds no choices.
    /// </summary>
    private static List<ExperimentalCoveragePlanner.PlanResult> OrderedCoverageCandidates
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        (Double X, Double Y)? Start = null,
        (Double X, Double Y)? End = null
        )
    {
        return ExperimentalCoveragePlanner.CandidatePlans(
                Cells, Grid, BladeM, Start, End)
            .GroupBy(
                Candidate => CoverageGeometrySignature(Candidate),
                StringComparer.Ordinal)
            .Select(Group => Group
                .OrderBy(Candidate => Candidate.Score)
                .ThenBy(Candidate => Candidate.HeadingDeg)
                .First())
            .OrderBy(Candidate => Candidate.OmittedCells.Count)
            .ThenBy(Candidate => Candidate.Score)
            .ThenBy(Candidate => Candidate.HeadingDeg)
            .ToList();
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

    /// <summary>Builds a bounded flexible-endpoint sequence after strict ordering fails.</summary>
    private static ExperimentalRunSequencer.Result BuildFlexibleSequence
        (
        List<ExperimentalCoveragePlanner.Run> Runs,
        Grid Grid,
        ExperimentalConnectorPlanner.Pose? Start = null,
        ExperimentalConnectorPlanner.Pose? End = null
        )
    {
        return ExperimentalRunSequencer.OptimizeFlexibleEndpoints(
            Runs,
            Start,
            End,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
    }

    /// <summary>
    /// Tries every bounded coverage heading in omission-count/score order after
    /// the minimum-omission geometry proves impossible to sequence. This lets a
    /// compact edge-parallel plan omit the smallest incompatible corner set
    /// instead of retaining multiple short runs with no forward exit.
    /// </summary>
    private static Boolean TrySelectFeasibleFillCoverageCandidate
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out ExperimentalCoveragePlanner.PlanResult SelectedPlan,
        out ExperimentalRunSequencer.Result SelectedSequence
        )
    {
        List<ExperimentalCoveragePlanner.PlanResult> Candidates =
            ExperimentalCoveragePlanner.CandidatePlans(
                Cells,
                Grid,
                BladeM,
                (Start.X, Start.Y),
                null)
            .GroupBy(
                Candidate => CoverageGeometrySignature(Candidate),
                StringComparer.Ordinal)
            .Select(Group => Group
                .OrderBy(Candidate => Candidate.Score)
                .ThenBy(Candidate => Candidate.HeadingDeg)
                .First())
            .OrderBy(Candidate => Candidate.OmittedCells.Count)
            .ThenBy(Candidate => Candidate.Score)
            .ThenBy(Candidate => Candidate.HeadingDeg)
            .ToList();
        foreach (ExperimentalCoveragePlanner.PlanResult Candidate in Candidates)
        {
            if (TryBuildFeasibleFillCandidate(
                Candidate,
                Grid,
                Start,
                GradeReturn,
                out ExperimentalRunSequencer.Result Sequence))
            {
                SelectedPlan = Candidate;
                SelectedSequence = Sequence;
                return true;
            }
        }
        SelectedPlan = ExperimentalCoveragePlanner.Plan(
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
            Grid,
            BladeM);
        SelectedSequence = BuildSequence(SelectedPlan.Runs, Grid);
        return false;
    }

    /// <summary>
    /// Tries alternate bounded cut headings when every minimum-omission cut layout
    /// is individually feasible but cannot exit toward the selected fill.
    /// </summary>
    private static Boolean TrySelectFeasibleCutCoverageCandidate
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose? Start,
        ExperimentalRunSequencer.Result FillSequence,
        out ExperimentalCoveragePlanner.PlanResult SelectedPlan,
        out ExperimentalRunSequencer.Result SelectedSequence
        )
    {
        ExperimentalConnectorPlanner.Pose FillEntry =
            FillSequence.Runs[0].EntryPose;
        List<ExperimentalCoveragePlanner.PlanResult> Candidates =
            ExperimentalCoveragePlanner.CandidatePlans(
                Cells,
                Grid,
                BladeM,
                Start.HasValue ? (Start.Value.X, Start.Value.Y) : null,
                (FillEntry.X, FillEntry.Y))
            .GroupBy(
                Candidate => CoverageGeometrySignature(Candidate),
                StringComparer.Ordinal)
            .Select(Group => Group
                .OrderBy(Candidate => Candidate.Score)
                .ThenBy(Candidate => Candidate.HeadingDeg)
                .First())
            .OrderBy(Candidate => Candidate.OmittedCells.Count)
            .ThenBy(Candidate => Candidate.Score)
            .ThenBy(Candidate => Candidate.HeadingDeg)
            .ToList();
        foreach (ExperimentalCoveragePlanner.PlanResult Candidate in Candidates)
        {
            if (Candidate.Runs.Count == 0) continue;
            ExperimentalRunSequencer.Result Sequence = BuildSequence(
                Candidate.Runs,
                Grid,
                Start,
                FillEntry);
            if (!Sequence.Success)
            {
                Sequence = BuildFlexibleSequence(
                    Candidate.Runs,
                    Grid,
                    Start,
                    FillEntry);
            }
            if (!Sequence.Success
                || (Start.HasValue
                    && !PoseConnectsToSequence(Start.Value, Sequence, Grid))
                || !SequencesConnect(Sequence, FillSequence, Grid))
            {
                continue;
            }
            SelectedPlan = Candidate;
            SelectedSequence = Sequence;
            return true;
        }
        SelectedPlan = ExperimentalCoveragePlanner.Plan(
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
            Grid,
            BladeM);
        SelectedSequence = BuildSequence(SelectedPlan.Runs, Grid);
        return false;
    }

    /// <summary>
    /// Jointly selects reduced cut coverage, its final orientation, and a
    /// grade-capable fill entry. Entry-pose trials are bounded globally so a
    /// difficult pair cannot multiply every coverage candidate indefinitely.
    /// </summary>
    private static Boolean TrySelectFeasibleCutFillPair
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out ExperimentalCoveragePlanner.PlanResult SelectedCutPlan,
        out ExperimentalRunSequencer.Result SelectedCutSequence,
        out ExperimentalCoveragePlanner.PlanResult SelectedFillPlan,
        out ExperimentalRunSequencer.Result SelectedFillSequence
        )
    {
        List<ExperimentalCoveragePlanner.PlanResult> CutCandidates =
            MinimalOmissionCandidates(
                CutCells, Grid, BladeM, (Start.X, Start.Y), null);
        List<ExperimentalCoveragePlanner.PlanResult> FillCandidates =
            MinimalOmissionCandidates(FillCells, Grid, BladeM);
        Int32 Attempts = 0;
        foreach (Int32 RankSum in Enumerable.Range(
            0, CutCandidates.Count + FillCandidates.Count - 1))
        {
            Int32 FirstCutIndex = Math.Max(
                0, RankSum - FillCandidates.Count + 1);
            Int32 LastCutIndex = Math.Min(
                CutCandidates.Count - 1, RankSum);
            for (Int32 CutIndex = FirstCutIndex;
                CutIndex <= LastCutIndex;
                CutIndex += 1)
            {
                Int32 FillIndex = RankSum - CutIndex;
                ExperimentalCoveragePlanner.PlanResult CutCandidate =
                    CutCandidates[CutIndex];
                ExperimentalCoveragePlanner.PlanResult FillCandidate =
                    FillCandidates[FillIndex];
                foreach (ExperimentalConnectorPlanner.Pose FillEntry in
                    PossibleEntryPoses(FillCandidate.Runs, Grid))
                {
                    if (Attempts >= MAX_PAIRED_CANDIDATE_ATTEMPTS)
                    {
                        goto Failure;
                    }
                    Attempts += 1;
                    ExperimentalRunSequencer.Result CutSequence =
                        BuildSequence(
                            CutCandidate.Runs, Grid, Start, FillEntry);
                    if (!CutSequence.Success)
                    {
                        CutSequence = BuildFlexibleSequence(
                            CutCandidate.Runs, Grid, Start, FillEntry);
                    }
                    if (!CutSequence.Success
                        || (CutSequence.Runs.Count == 0)
                        || !PoseConnectsToSequence(Start, CutSequence, Grid))
                    {
                        continue;
                    }
                    ExperimentalConnectorPlanner.Pose CutExit =
                        CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose;
                    if (!TryBuildFeasibleFillCandidate(
                        FillCandidate,
                        Grid,
                        CutExit,
                        GradeReturn,
                        out ExperimentalRunSequencer.Result FillSequence)
                        || !SequencesConnect(
                            CutSequence, FillSequence, Grid))
                    {
                        continue;
                    }
                    SelectedCutPlan = CutCandidate;
                    SelectedCutSequence = CutSequence;
                    SelectedFillPlan = FillCandidate;
                    SelectedFillSequence = FillSequence;
                    return true;
                }
            }
        }

    Failure:
        SelectedCutPlan = ExperimentalCoveragePlanner.Plan(
            CutCells, Grid, BladeM);
        SelectedCutSequence = BuildSequence(
            SelectedCutPlan.Runs, Grid);
        SelectedFillPlan = ExperimentalCoveragePlanner.Plan(
            FillCells, Grid, BladeM);
        SelectedFillSequence = BuildSequence(
            SelectedFillPlan.Runs, Grid);
        return false;
    }

    /// <summary>
    /// Returns an order- and orientation-independent exact signature for one
    /// coverage decomposition. Opposite cross-axis packing often produces the
    /// same run set; sequencing that geometry twice cannot change feasibility.
    /// </summary>
    private static String CoverageGeometrySignature
        (
        ExperimentalCoveragePlanner.PlanResult Candidate
        )
    {
        List<String> Runs = Candidate.Runs.Select(Run =>
        {
            (Double X, Double Y) First = Run.StartXy;
            (Double X, Double Y) Second = Run.EndXy;
            if ((Second.X < First.X)
                || ((Second.X == First.X) && (Second.Y < First.Y)))
            {
                (First, Second) = (Second, First);
            }
            String Cells = String.Join(",", Run.Cells
                .OrderBy(Entry => Entry.Cell.Bx)
                .ThenBy(Entry => Entry.Cell.By)
                .ThenBy(Entry => BitConverter.DoubleToInt64Bits(Entry.DepthM))
                .Select(Entry => String.Format(
                    CultureInfo.InvariantCulture,
                    "{0}:{1}:{2:X16}",
                    Entry.Cell.Bx,
                    Entry.Cell.By,
                    BitConverter.DoubleToInt64Bits(Entry.DepthM))));
            return String.Format(
                CultureInfo.InvariantCulture,
                "{0:X16}:{1:X16}:{2:X16}:{3:X16}:{4}",
                BitConverter.DoubleToInt64Bits(First.X),
                BitConverter.DoubleToInt64Bits(First.Y),
                BitConverter.DoubleToInt64Bits(Second.X),
                BitConverter.DoubleToInt64Bits(Second.Y),
                Cells);
        }).OrderBy(Signature => Signature, StringComparer.Ordinal).ToList();
        String Omitted = String.Join(",", Candidate.OmittedCells
            .OrderBy(Entry => Entry.Cell.Bx)
            .ThenBy(Entry => Entry.Cell.By)
            .ThenBy(Entry => BitConverter.DoubleToInt64Bits(Entry.DepthM))
            .Select(Entry => String.Format(
                CultureInfo.InvariantCulture,
                "{0}:{1}:{2:X16}",
                Entry.Cell.Bx,
                Entry.Cell.By,
                BitConverter.DoubleToInt64Bits(Entry.DepthM))));
        return String.Join("|", Runs) + "#" + Omitted;
    }

    /// <summary>Builds and fully validates one alternate fill heading.</summary>
    private static Boolean TryBuildFeasibleFillCandidate
        (
        ExperimentalCoveragePlanner.PlanResult Candidate,
        Grid Grid,
        ExperimentalConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out ExperimentalRunSequencer.Result Sequence
        )
    {
        Sequence = BuildSequence(Candidate.Runs, Grid, Start, null);
        if (Candidate.Runs.Count == 0) return false;
        if (!Sequence.Success)
        {
            Sequence = BuildFlexibleSequence(
                Candidate.Runs, Grid, Start, null);
        }
        if (!Sequence.Success) return false;
        if (GradeReturn && (Sequence.Runs.Count > 1))
        {
            Sequence = ExperimentalLifecyclePlanner.BuildContinuousFillSequence(
                Sequence,
                Start,
                Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                    * Constants.FEET_TO_METERS,
                Math.Min(0.5, Grid.GridSizeM),
                Grid);
        }
        return Sequence.Success
            && PoseConnectsToSequence(Start, Sequence, Grid)
            && (!GradeReturn || SupportsGradeReturn(Sequence, Grid));
    }

    /// <summary>Validates the explicit lifecycle connector between cut exit and fill entry.</summary>
    private static Boolean SequencesConnect
        (
        ExperimentalRunSequencer.Result Cut,
        ExperimentalRunSequencer.Result Fill,
        Grid Grid
        )
    {
        if (!Cut.Success || !Fill.Success) return false;
        if ((Cut.Runs.Count == 0) || (Fill.Runs.Count == 0)) return true;
        ExperimentalTransitPlanner.Result Connector = ExperimentalTransitPlanner.Connect(
            Cut.Runs[Cut.Runs.Count - 1].ExitPose,
            Fill.Runs[0].EntryPose,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
        return Connector.Success;
    }

    /// <summary>Validates a previous-pass pose against the first remaining run in a cut-omitted pass.</summary>
    private static Boolean PoseConnectsToSequence
        (
        ExperimentalConnectorPlanner.Pose Start,
        ExperimentalRunSequencer.Result Sequence,
        Grid Grid,
        Boolean AllowForwardDetour = false
        )
    {
        if (!Sequence.Success) return false;
        if (Sequence.Runs.Count == 0) return true;
        ExperimentalTransitPlanner.Result Connector = ExperimentalTransitPlanner.Connect(
            Start,
            Sequence.Runs[0].EntryPose,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid,
            AllowForwardDetour: AllowForwardDetour);
        return Connector.Success;
    }

    /// <summary>Validates the final run against an explicitly requested following pose.</summary>
    private static Boolean SequenceConnectsToPose
        (
        ExperimentalRunSequencer.Result Sequence,
        ExperimentalConnectorPlanner.Pose End,
        Grid Grid
        )
    {
        if (!Sequence.Success) return false;
        if (Sequence.Runs.Count == 0) return true;
        ExperimentalTransitPlanner.Result Connector = ExperimentalTransitPlanner.Connect(
            Sequence.Runs[Sequence.Runs.Count - 1].ExitPose,
            End,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
        return Connector.Success;
    }

    /// <summary>
    /// Jointly selects consecutive fill routes when the intervening corner cut was omitted.
    /// The next pass is used as an optimization hint only after both real connectors validate.
    /// </summary>
    private static Boolean TrySelectAdjacentFills
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CurrentCells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> NextCells,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose CutExit,
        Boolean GradeReturn,
        out ExperimentalCoveragePlanner.PlanResult SelectedCurrentPlan,
        out ExperimentalRunSequencer.Result SelectedCurrentSequence,
        out ExperimentalCoveragePlanner.PlanResult SelectedNextPlan,
        out ExperimentalRunSequencer.Result SelectedNextSequence
        )
    {
        List<ExperimentalCoveragePlanner.PlanResult> CurrentCandidates =
            MinimalOmissionCandidates(
                CurrentCells, Grid, BladeM, (CutExit.X, CutExit.Y), null);
        List<ExperimentalCoveragePlanner.PlanResult> NextCandidates =
            MinimalOmissionCandidates(NextCells, Grid, BladeM);
        ExperimentalCoveragePlanner.PlanResult? BestCurrentPlan = null;
        ExperimentalRunSequencer.Result? BestCurrentSequence = null;
        ExperimentalCoveragePlanner.PlanResult? BestNextPlan = null;
        ExperimentalRunSequencer.Result? BestNextSequence = null;
        Int32 BestOmittedCells = Int32.MaxValue;
        Double BestScore = Double.PositiveInfinity;

        foreach (ExperimentalCoveragePlanner.PlanResult NextCandidate in NextCandidates)
        {
            List<ExperimentalConnectorPlanner.Pose?> Targets =
                PossibleEntryPoses(NextCandidate.Runs, Grid)
                    .Select(Pose => (ExperimentalConnectorPlanner.Pose?)Pose)
                    .ToList();
            Targets.Add(null);
            foreach (ExperimentalCoveragePlanner.PlanResult CurrentCandidate in CurrentCandidates)
            {
                foreach (ExperimentalConnectorPlanner.Pose? Target in Targets)
                {
                    ExperimentalRunSequencer.Result CurrentSequence = BuildSequence(
                        CurrentCandidate.Runs, Grid, CutExit, Target);
                    if (!CurrentSequence.Success
                        || (CurrentSequence.Runs.Count == 0)
                        || !PoseConnectsToSequence(CutExit, CurrentSequence, Grid)
                        || (GradeReturn && !SupportsGradeReturn(CurrentSequence, Grid)))
                    {
                        continue;
                    }
                    ExperimentalConnectorPlanner.Pose Outgoing = GradeReturn
                        ? ReversePose(CurrentSequence.Runs[0].EntryPose)
                        : CurrentSequence.Runs[
                            CurrentSequence.Runs.Count - 1].ExitPose;
                    ExperimentalRunSequencer.Result NextSequence = BuildSequence(
                        NextCandidate.Runs, Grid, Outgoing, null);
                    if (!NextSequence.Success
                        || (NextSequence.Runs.Count == 0)
                        || !PoseConnectsToSequence(Outgoing, NextSequence, Grid))
                    {
                        continue;
                    }
                    Int32 OmittedCells = CurrentCandidate.OmittedCells.Count
                        + NextCandidate.OmittedCells.Count;
                    Double Score = CurrentCandidate.Score + NextCandidate.Score;
                    if ((OmittedCells > BestOmittedCells)
                        || ((OmittedCells == BestOmittedCells)
                            && (Score >= BestScore - 1e-9)))
                    {
                        continue;
                    }
                    BestCurrentPlan = CurrentCandidate;
                    BestCurrentSequence = CurrentSequence;
                    BestNextPlan = NextCandidate;
                    BestNextSequence = NextSequence;
                    BestOmittedCells = OmittedCells;
                    BestScore = Score;
                }
            }
        }
        if ((BestCurrentPlan != null)
            && (BestCurrentSequence != null)
            && (BestNextPlan != null)
            && (BestNextSequence != null))
        {
            SelectedCurrentPlan = BestCurrentPlan;
            SelectedCurrentSequence = BestCurrentSequence;
            SelectedNextPlan = BestNextPlan;
            SelectedNextSequence = BestNextSequence;
            return true;
        }
        SelectedCurrentPlan = ExperimentalCoveragePlanner.Plan(CurrentCells, Grid, BladeM);
        SelectedCurrentSequence = BuildSequence(SelectedCurrentPlan.Runs, Grid);
        SelectedNextPlan = ExperimentalCoveragePlanner.Plan(NextCells, Grid, BladeM);
        SelectedNextSequence = BuildSequence(SelectedNextPlan.Runs, Grid);
        return false;
    }

    /// <summary>Returns every prepared orientation that could begin the next coverage plan.</summary>
    private static List<ExperimentalConnectorPlanner.Pose> PossibleEntryPoses
        (
        List<ExperimentalCoveragePlanner.Run> Runs,
        Grid Grid
        )
    {
        List<ExperimentalConnectorPlanner.Pose> Output =
            new List<ExperimentalConnectorPlanner.Pose>();
        Double LineupM = Constants.DEFAULT_EQUIPMENT_LENGTH_FT
            * Constants.FEET_TO_METERS;
        Double MinimumRadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT
            * 0.5 * Constants.FEET_TO_METERS;
        Double SampleSpacingM = Math.Min(0.5, Grid.GridSizeM);
        foreach (ExperimentalCoveragePlanner.Run Run in Runs)
        {
            ExperimentalCoveragePlanner.Run Reverse = ReverseRun(Run);
            Output.Add(ExperimentalRunSequencer.Prepare(
                Run, LineupM, MinimumRadiusM, SampleSpacingM, Grid).EntryPose);
            Output.Add(new ExperimentalConnectorPlanner.Pose(
                Run.StartXy.X, Run.StartXy.Y, Run.HeadingDeg));
            Output.Add(ExperimentalRunSequencer.Prepare(
                Reverse, LineupM, MinimumRadiusM, SampleSpacingM, Grid).EntryPose);
            Output.Add(new ExperimentalConnectorPlanner.Pose(
                Reverse.StartXy.X,
                Reverse.StartXy.Y,
                Reverse.HeadingDeg));
        }
        return Output
            .GroupBy(Pose => String.Format(
                CultureInfo.InvariantCulture,
                "{0:X16}:{1:X16}:{2:X16}",
                BitConverter.DoubleToInt64Bits(Pose.X),
                BitConverter.DoubleToInt64Bits(Pose.Y),
                BitConverter.DoubleToInt64Bits(Pose.HeadingDeg % 360.0)))
            .Select(Group => Group.First())
            .ToList();
    }

    /// <summary>
    /// Reorients the preceding fill together with the current cut and fill when a late
    /// edge omission invalidates the connector anticipated by the preceding pass.
    /// </summary>
    private static Boolean TryBacktrackPreviousFillAndCurrentPass
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> PreviousFillCells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CurrentCutCells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CurrentFillCells,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose PreviousCutExit,
        Boolean GradeReturn,
        out ExperimentalCoveragePlanner.PlanResult SelectedPreviousFillPlan,
        out ExperimentalRunSequencer.Result SelectedPreviousFillSequence,
        out ExperimentalCoveragePlanner.PlanResult SelectedCutPlan,
        out ExperimentalRunSequencer.Result SelectedCutSequence,
        out ExperimentalCoveragePlanner.PlanResult SelectedFillPlan,
        out ExperimentalRunSequencer.Result SelectedFillSequence,
        Action<String>? Diagnostic = null
        )
    {
        List<ExperimentalCoveragePlanner.PlanResult> PreviousFillCandidates =
            MinimalOmissionCandidates(
                PreviousFillCells,
                Grid,
                BladeM,
                (PreviousCutExit.X, PreviousCutExit.Y),
                null);
        // If every complete cut layout is unreachable, retain higher-omission
        // layouts as a controlled final recovery. RankedTriples below guarantees
        // that no such layout is tried before every lower-omission alternative.
        List<ExperimentalCoveragePlanner.PlanResult> CutCandidates =
            OrderedCoverageCandidates(CurrentCutCells, Grid, BladeM);
        List<ExperimentalCoveragePlanner.PlanResult> FillCandidates =
            MinimalOmissionCandidates(
                CurrentFillCells, Grid, BladeM);
        ExperimentalCoveragePlanner.PlanResult? BestPreviousFillPlan = null;
        ExperimentalRunSequencer.Result? BestPreviousFillSequence = null;
        ExperimentalCoveragePlanner.PlanResult? BestCutPlan = null;
        ExperimentalRunSequencer.Result? BestCutSequence = null;
        ExperimentalCoveragePlanner.PlanResult? BestFillPlan = null;
        ExperimentalRunSequencer.Result? BestFillSequence = null;
        Int32 BestOmittedCells = Int32.MaxValue;
        Double BestScore = Double.PositiveInfinity;
        Int32 MinimumOmittedCells = PreviousFillCandidates[0].OmittedCells.Count
            + CutCandidates[0].OmittedCells.Count
            + FillCandidates[0].OmittedCells.Count;
        Int32 CandidateAttempts = 0;
        Diagnostic?.Invoke(String.Format(
            CultureInfo.InvariantCulture,
            "whole_plan backtrack_candidates previous_fills={0} cuts={1} fills={2} minimum_omitted={3} cut_omission_tiers={4} limit={5}",
            PreviousFillCandidates.Count,
            CutCandidates.Count,
            FillCandidates.Count,
            MinimumOmittedCells,
            String.Join(",", CutCandidates
                .GroupBy(Candidate => Candidate.OmittedCells.Count)
                .OrderBy(Group => Group.Key)
                .Select(Group => String.Format(
                    CultureInfo.InvariantCulture,
                    "{0}:{1}",
                    Group.Key,
                    Group.Count()))),
            MAX_BACKTRACK_CANDIDATE_ATTEMPTS));

        ExperimentalRunSequencer.Result?[] ProvisionalFills =
            new ExperimentalRunSequencer.Result?[FillCandidates.Count];
        List<ExperimentalConnectorPlanner.Pose>?[] CutEntryTargetsByCandidate =
            new List<ExperimentalConnectorPlanner.Pose>?[CutCandidates.Count];
        Dictionary<(Int32 PreviousFillIndex, Int32 CutIndex, Int32 TargetIndex),
            (ExperimentalRunSequencer.Result Sequence,
                ExperimentalConnectorPlanner.Pose Outgoing)?> PreviousOptions =
            new Dictionary<(Int32 PreviousFillIndex, Int32 CutIndex,
                Int32 TargetIndex),
                (ExperimentalRunSequencer.Result Sequence,
                    ExperimentalConnectorPlanner.Pose Outgoing)?>();
        List<(Int32 PreviousFillIndex, Int32 CutIndex, Int32 FillIndex)>
            RankedTriples = Enumerable.Range(0, PreviousFillCandidates.Count)
                .SelectMany(PreviousFillIndex =>
                    Enumerable.Range(0, CutCandidates.Count)
                        .SelectMany(CutIndex =>
                            Enumerable.Range(0, FillCandidates.Count)
                                .Select(FillIndex => (
                                    PreviousFillIndex,
                                    CutIndex,
                                    FillIndex))))
                .OrderBy(Triple =>
                    PreviousFillCandidates[Triple.PreviousFillIndex]
                        .OmittedCells.Count
                    + CutCandidates[Triple.CutIndex].OmittedCells.Count
                    + FillCandidates[Triple.FillIndex].OmittedCells.Count)
                .ThenBy(Triple => Triple.PreviousFillIndex
                    + Triple.CutIndex + Triple.FillIndex)
                .ThenBy(Triple => Math.Max(
                    Triple.PreviousFillIndex,
                    Math.Max(Triple.CutIndex, Triple.FillIndex)))
                .ThenBy(Triple => Triple.FillIndex)
                .ThenBy(Triple => Triple.CutIndex)
                .ThenBy(Triple => Triple.PreviousFillIndex)
                .ToList();
        foreach ((Int32 PreviousFillIndex, Int32 CutIndex, Int32 FillIndex) Triple
            in RankedTriples)
        {
            ExperimentalCoveragePlanner.PlanResult PreviousFillCandidate =
                PreviousFillCandidates[Triple.PreviousFillIndex];
            ExperimentalCoveragePlanner.PlanResult CutCandidate =
                CutCandidates[Triple.CutIndex];
            ExperimentalCoveragePlanner.PlanResult FillCandidate =
                FillCandidates[Triple.FillIndex];
            ExperimentalRunSequencer.Result ProvisionalFill =
                ProvisionalFills[Triple.FillIndex]
                ?? BuildSequence(FillCandidate.Runs, Grid);
            ProvisionalFills[Triple.FillIndex] = ProvisionalFill;
            if (!ProvisionalFill.Success || (ProvisionalFill.Runs.Count == 0)) continue;
            if (CutCandidate.Runs.Count == 0) continue;
            ExperimentalConnectorPlanner.Pose FillEntry =
                ProvisionalFill.Runs[0].EntryPose;
            List<ExperimentalConnectorPlanner.Pose> CutEntryTargets =
                CutEntryTargetsByCandidate[Triple.CutIndex]
                ?? PossibleEntryPoses(CutCandidate.Runs, Grid);
            CutEntryTargetsByCandidate[Triple.CutIndex] = CutEntryTargets;
            for (Int32 TargetIndex = 0;
                TargetIndex < CutEntryTargets.Count;
                TargetIndex += 1)
            {
                ExperimentalConnectorPlanner.Pose CutEntryTarget =
                    CutEntryTargets[TargetIndex];
                CandidateAttempts += 1;
                if ((CandidateAttempts % 10000) == 0)
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan backtrack_progress attempts={0} limit={1} cached_previous_options={2}",
                        CandidateAttempts,
                        MAX_BACKTRACK_CANDIDATE_ATTEMPTS,
                        PreviousOptions.Count));
                }
                if (CandidateAttempts > MAX_BACKTRACK_CANDIDATE_ATTEMPTS)
                {
                    goto BacktrackSearchComplete;
                }
                (Int32 PreviousFillIndex, Int32 CutIndex, Int32 TargetIndex)
                    PreviousKey = (
                        Triple.PreviousFillIndex,
                        Triple.CutIndex,
                        TargetIndex);
                if (!PreviousOptions.TryGetValue(
                    PreviousKey,
                    out (ExperimentalRunSequencer.Result Sequence,
                        ExperimentalConnectorPlanner.Pose Outgoing)? PreviousOption))
                {
                    ExperimentalRunSequencer.Result PreviousFillSequenceCandidate =
                            BuildSequence(
                                PreviousFillCandidate.Runs,
                                Grid,
                                PreviousCutExit,
                                CutEntryTarget);
                    if (GradeReturn
                        && PreviousFillSequenceCandidate.Success
                        && (PreviousFillSequenceCandidate.Runs.Count > 1))
                    {
                        PreviousFillSequenceCandidate =
                                ExperimentalLifecyclePlanner.BuildContinuousFillSequence(
                                    PreviousFillSequenceCandidate,
                                    PreviousCutExit,
                                    Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                                        * Constants.FEET_TO_METERS,
                                    Math.Min(0.5, Grid.GridSizeM),
                                    Grid);
                    }
                    if (!PreviousFillSequenceCandidate.Success
                        || (PreviousFillSequenceCandidate.Runs.Count == 0)
                        || !PoseConnectsToSequence(
                            PreviousCutExit,
                            PreviousFillSequenceCandidate,
                            Grid)
                        || (GradeReturn
                            && !SupportsGradeReturn(
                                PreviousFillSequenceCandidate,
                                Grid,
                                FillSequencePrepared: true)))
                    {
                        PreviousOption = null;
                    }
                    else
                    {
                        ExperimentalConnectorPlanner.Pose PreviousOutgoingCandidate =
                            GradeReturn
                                ? ReversePose(
                                    PreviousFillSequenceCandidate.Runs[0].EntryPose)
                                : PreviousFillSequenceCandidate.Runs[
                                    PreviousFillSequenceCandidate.Runs.Count - 1]
                                    .ExitPose;
                        PreviousOption = (
                            PreviousFillSequenceCandidate,
                            PreviousOutgoingCandidate);
                    }
                    PreviousOptions[PreviousKey] = PreviousOption;
                }
                if (!PreviousOption.HasValue) continue;
                ExperimentalRunSequencer.Result PreviousFillSequence =
                    PreviousOption.Value.Sequence;
                ExperimentalConnectorPlanner.Pose PreviousOutgoing =
                    PreviousOption.Value.Outgoing;
                        ExperimentalRunSequencer.Result CutSequence = BuildSequence(
                            CutCandidate.Runs, Grid, PreviousOutgoing, FillEntry);
                        if (!CutSequence.Success
                            || !PoseConnectsToSequence(
                                PreviousOutgoing, CutSequence, Grid))
                        {
                            continue;
                        }
                        ExperimentalConnectorPlanner.Pose CutExit =
                            CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose;
                        ExperimentalRunSequencer.Result FillSequence = BuildSequence(
                            FillCandidate.Runs, Grid, CutExit, null);
                        if (GradeReturn
                            && FillSequence.Success
                            && (FillSequence.Runs.Count > 1))
                        {
                            FillSequence =
                                ExperimentalLifecyclePlanner.BuildContinuousFillSequence(
                                    FillSequence,
                                    CutExit,
                                    Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                                        * Constants.FEET_TO_METERS,
                                    Math.Min(0.5, Grid.GridSizeM),
                                    Grid);
                        }
                        if (!FillSequence.Success
                            || !SequencesConnect(CutSequence, FillSequence, Grid)
                            || (GradeReturn && !SupportsGradeReturn(
                                FillSequence,
                                Grid,
                                FillSequencePrepared: true)))
                        {
                            continue;
                        }
                        Int32 OmittedCells = PreviousFillCandidate.OmittedCells.Count
                            + CutCandidate.OmittedCells.Count
                            + FillCandidate.OmittedCells.Count;
                        Double Score = PreviousFillCandidate.Score
                            + CutCandidate.Score
                            + FillCandidate.Score;
                        if ((OmittedCells > BestOmittedCells)
                            || ((OmittedCells == BestOmittedCells)
                                && (Score >= BestScore - 1e-9)))
                        {
                            continue;
                        }
                        BestPreviousFillPlan = PreviousFillCandidate;
                        BestPreviousFillSequence = PreviousFillSequence;
                        BestCutPlan = CutCandidate;
                        BestCutSequence = CutSequence;
                        BestFillPlan = FillCandidate;
                        BestFillSequence = FillSequence;
                        BestOmittedCells = OmittedCells;
                        BestScore = Score;
                        SelectedPreviousFillPlan = PreviousFillCandidate;
                        SelectedPreviousFillSequence = PreviousFillSequence;
                        SelectedCutPlan = CutCandidate;
                        SelectedCutSequence = CutSequence;
                        SelectedFillPlan = FillCandidate;
                        SelectedFillSequence = FillSequence;
                        Diagnostic?.Invoke(String.Format(
                            CultureInfo.InvariantCulture,
                            "whole_plan backtrack_result attempts={0} selected=True previous_fill_index={1} cut_index={2} fill_index={3} omitted_cells={4} additional_omitted={5}",
                            CandidateAttempts,
                            Triple.PreviousFillIndex,
                            Triple.CutIndex,
                            Triple.FillIndex,
                            OmittedCells,
                            OmittedCells - MinimumOmittedCells));
                        return true;
            }
        }
    BacktrackSearchComplete:
        Diagnostic?.Invoke(String.Format(
            CultureInfo.InvariantCulture,
            "whole_plan backtrack_result attempts={0} selected={1} exhausted={2}",
            CandidateAttempts,
            BestPreviousFillPlan != null,
            CandidateAttempts > MAX_BACKTRACK_CANDIDATE_ATTEMPTS));
        if ((BestPreviousFillPlan != null)
            && (BestPreviousFillSequence != null)
            && (BestCutPlan != null)
            && (BestCutSequence != null)
            && (BestFillPlan != null)
            && (BestFillSequence != null))
        {
            SelectedPreviousFillPlan = BestPreviousFillPlan;
            SelectedPreviousFillSequence = BestPreviousFillSequence;
            SelectedCutPlan = BestCutPlan;
            SelectedCutSequence = BestCutSequence;
            SelectedFillPlan = BestFillPlan;
            SelectedFillSequence = BestFillSequence;
            return true;
        }
        SelectedPreviousFillPlan = ExperimentalCoveragePlanner.Plan(
            PreviousFillCells, Grid, BladeM);
        SelectedPreviousFillSequence = BuildSequence(SelectedPreviousFillPlan.Runs, Grid);
        SelectedCutPlan = ExperimentalCoveragePlanner.Plan(CurrentCutCells, Grid, BladeM);
        SelectedCutSequence = BuildSequence(SelectedCutPlan.Runs, Grid);
        SelectedFillPlan = ExperimentalCoveragePlanner.Plan(CurrentFillCells, Grid, BladeM);
        SelectedFillSequence = BuildSequence(SelectedFillPlan.Runs, Grid);
        return false;
    }

    /// <summary>
    /// Reorients the preceding cut before retrying the bounded three-way recovery.
    /// Only distinct forward exits of the already-selected cut geometry are tried,
    /// so this adds sequencing freedom without changing the preceding cut coverage.
    /// </summary>
    private static Boolean TryBacktrackPreviousCutAndCurrentPass
        (
        ExperimentalRunSequencer.Result ExistingPreviousCut,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> PreviousFillCells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CurrentCutCells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CurrentFillCells,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose? PreviousIncoming,
        ExperimentalConnectorPlanner.Pose ExistingPreviousCutExit,
        Boolean GradeReturn,
        out ExperimentalRunSequencer.Result SelectedPreviousCut,
        out ExperimentalCoveragePlanner.PlanResult SelectedPreviousFillPlan,
        out ExperimentalRunSequencer.Result SelectedPreviousFillSequence,
        out ExperimentalCoveragePlanner.PlanResult SelectedCutPlan,
        out ExperimentalRunSequencer.Result SelectedCutSequence,
        out ExperimentalCoveragePlanner.PlanResult SelectedFillPlan,
        out ExperimentalRunSequencer.Result SelectedFillSequence,
        Action<String>? Diagnostic = null
        )
    {
        List<ExperimentalCoveragePlanner.Run> PreviousCutRuns =
            ExistingPreviousCut.Runs
                .Select(Run => Run.Run)
                .ToList();
        List<ExperimentalConnectorPlanner.Pose> PreviousFillEntries =
            MinimalOmissionCandidates(PreviousFillCells, Grid, BladeM)
                .SelectMany(Candidate => PossibleEntryPoses(Candidate.Runs, Grid))
                .GroupBy(Pose => String.Format(
                    CultureInfo.InvariantCulture,
                    "{0:F2},{1:F2},{2:F1}",
                    Pose.X,
                    Pose.Y,
                    NormalizeHeading(Pose.HeadingDeg)),
                    StringComparer.Ordinal)
                .Select(Group => Group.First())
                .ToList();
        List<ExperimentalRunSequencer.Result> PreviousCutOptions =
            new List<ExperimentalRunSequencer.Result>();
        foreach (ExperimentalConnectorPlanner.Pose FillEntry in PreviousFillEntries)
        {
            ExperimentalRunSequencer.Result Candidate = BuildSequence(
                PreviousCutRuns,
                Grid,
                PreviousIncoming,
                FillEntry);
            if (!Candidate.Success || (Candidate.Runs.Count == 0)) continue;
            if (PreviousIncoming.HasValue
                && !PoseConnectsToSequence(
                    PreviousIncoming.Value,
                    Candidate,
                    Grid))
            {
                continue;
            }
            if (!SequenceConnectsToPose(Candidate, FillEntry, Grid)) continue;
            ExperimentalConnectorPlanner.Pose Exit = Candidate.Runs[
                Candidate.Runs.Count - 1].ExitPose;
            if (PoseApproximatelyEquals(Exit, ExistingPreviousCutExit)) continue;
            if (PreviousCutOptions.Any(Option => PoseApproximatelyEquals(
                Option.Runs[Option.Runs.Count - 1].ExitPose,
                Exit)))
            {
                continue;
            }
            PreviousCutOptions.Add(Candidate);
        }
        Diagnostic?.Invoke(String.Format(
            CultureInfo.InvariantCulture,
            "whole_plan deep_backtrack_candidates previous_fill_entries={0} distinct_previous_cut_exits={1}",
            PreviousFillEntries.Count,
            PreviousCutOptions.Count));
        foreach (ExperimentalRunSequencer.Result PreviousCutOption
            in PreviousCutOptions)
        {
            ExperimentalConnectorPlanner.Pose PreviousCutExit =
                PreviousCutOption.Runs[
                    PreviousCutOption.Runs.Count - 1].ExitPose;
            if (TryBacktrackPreviousFillAndCurrentPass(
                PreviousFillCells,
                CurrentCutCells,
                CurrentFillCells,
                Grid,
                BladeM,
                PreviousCutExit,
                GradeReturn,
                out SelectedPreviousFillPlan,
                out SelectedPreviousFillSequence,
                out SelectedCutPlan,
                out SelectedCutSequence,
                out SelectedFillPlan,
                out SelectedFillSequence,
                Diagnostic))
            {
                SelectedPreviousCut = PreviousCutOption;
                return true;
            }
        }
        SelectedPreviousCut = ExistingPreviousCut;
        SelectedPreviousFillPlan = ExperimentalCoveragePlanner.Plan(
            PreviousFillCells, Grid, BladeM);
        SelectedPreviousFillSequence = BuildSequence(
            SelectedPreviousFillPlan.Runs, Grid);
        SelectedCutPlan = ExperimentalCoveragePlanner.Plan(
            CurrentCutCells, Grid, BladeM);
        SelectedCutSequence = BuildSequence(SelectedCutPlan.Runs, Grid);
        SelectedFillPlan = ExperimentalCoveragePlanner.Plan(
            CurrentFillCells, Grid, BladeM);
        SelectedFillSequence = BuildSequence(SelectedFillPlan.Runs, Grid);
        return false;
    }

    /// <summary>Returns the final forward pose after one completed pass.</summary>
    private static ExperimentalConnectorPlanner.Pose? PassOutgoingPose
        (
        ExperimentalLifecyclePlanner.PassPlan Pass,
        Boolean GradeReturn
        )
    {
        if (Pass.Fill.Runs.Count > 0)
        {
            return GradeReturn
                ? ReversePose(Pass.Fill.Runs[0].EntryPose)
                : Pass.Fill.Runs[Pass.Fill.Runs.Count - 1].ExitPose;
        }
        if (Pass.Cut.Runs.Count > 0)
        {
            return Pass.Cut.Runs[Pass.Cut.Runs.Count - 1].ExitPose;
        }
        return null;
    }

    /// <summary>Compares two poses at the sequencer's practical cache precision.</summary>
    private static Boolean PoseApproximatelyEquals
        (
        ExperimentalConnectorPlanner.Pose First,
        ExperimentalConnectorPlanner.Pose Second
        )
    {
        return (Math.Abs(First.X - Second.X) <= 0.01)
            && (Math.Abs(First.Y - Second.Y) <= 0.01)
            && (Math.Abs(NormalizeHeading(First.HeadingDeg - Second.HeadingDeg))
                <= 0.1
                || Math.Abs(
                    NormalizeHeading(First.HeadingDeg - Second.HeadingDeg) - 360.0)
                    <= 0.1);
    }

    /// <summary>Normalizes a heading to [0, 360).</summary>
    private static Double NormalizeHeading(Double HeadingDeg)
    {
        Double Output = HeadingDeg % 360.0;
        return Output < 0.0 ? Output + 360.0 : Output;
    }

    /// <summary>Reverses a coverage run without changing its assigned work cells.</summary>
    private static ExperimentalCoveragePlanner.Run ReverseRun
        (
        ExperimentalCoveragePlanner.Run Run
        )
    {
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Run.Cells);
        Cells.Reverse();
        return new ExperimentalCoveragePlanner.Run(
            Cells,
            Run.EndXy,
            Run.StartXy,
            Run.HeadingDeg + 180.0,
            Run.MaximumLateralErrorM);
    }

    /// <summary>Returns the same position facing in the opposite direction.</summary>
    private static ExperimentalConnectorPlanner.Pose ReversePose
        (
        ExperimentalConnectorPlanner.Pose Pose
        )
    {
        return new ExperimentalConnectorPlanner.Pose(
            Pose.X, Pose.Y, Pose.HeadingDeg + 180.0);
    }

    /// <summary>
    /// Finds a feasible incoming edge fill using the smallest tested boundary-cell omission set.
    /// </summary>
    private static Boolean TryFillWithMinimalAdditionalEdgeOmission
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out ExperimentalCoveragePlanner.PlanResult SelectedPlan,
        out ExperimentalRunSequencer.Result SelectedSequence
        )
    {
        ExperimentalFieldEdgePolicy EdgePolicy =
            new ExperimentalFieldEdgePolicy(Grid, BladeM);
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> EdgeCells = Cells
            .Where(Entry => EdgePolicy.IsEdgeWorkCell(Entry.Cell))
            .ToList();
        Int32 MaximumAdditionalOmissions = Math.Min(3, EdgeCells.Count);
        for (Int32 OmissionCount = 1;
            OmissionCount <= MaximumAdditionalOmissions;
            OmissionCount += 1)
        {
            ExperimentalCoveragePlanner.PlanResult? BestPlan = null;
            ExperimentalRunSequencer.Result? BestSequence = null;
            Double BestScore = Double.PositiveInfinity;
            for (Int32 First = 0; First < EdgeCells.Count; First += 1)
            {
                Int32 SecondStart = OmissionCount >= 2 ? First + 1 : EdgeCells.Count;
                Int32 SecondEnd = OmissionCount >= 2 ? EdgeCells.Count : EdgeCells.Count + 1;
                for (Int32 Second = SecondStart; Second < SecondEnd; Second += 1)
                {
                    Int32 ThirdStart = OmissionCount >= 3 ? Second + 1 : EdgeCells.Count;
                    Int32 ThirdEnd = OmissionCount >= 3 ? EdgeCells.Count : EdgeCells.Count + 1;
                    for (Int32 Third = ThirdStart; Third < ThirdEnd; Third += 1)
                    {
                        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Removed =
                            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                            {
                                EdgeCells[First],
                            };
                        if (OmissionCount >= 2) Removed.Add(EdgeCells[Second]);
                        if (OmissionCount >= 3) Removed.Add(EdgeCells[Third]);
                        if (!TryBuildFillAfterOmissions(
                            Cells,
                            Removed,
                            Grid,
                            BladeM,
                            Start,
                            GradeReturn,
                            out ExperimentalCoveragePlanner.PlanResult CandidatePlan,
                            out ExperimentalRunSequencer.Result CandidateSequence))
                        {
                            continue;
                        }
                        if (CandidatePlan.Score >= BestScore - 1e-9) continue;
                        BestPlan = CandidatePlan;
                        BestSequence = CandidateSequence;
                        BestScore = CandidatePlan.Score;
                    }
                }
            }
            if ((BestPlan != null) && (BestSequence != null))
            {
                SelectedPlan = BestPlan;
                SelectedSequence = BestSequence;
                return true;
            }
        }
        SelectedPlan = ExperimentalCoveragePlanner.Plan(
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(), Grid, BladeM);
        SelectedSequence = BuildSequence(SelectedPlan.Runs, Grid);
        return false;
    }

    /// <summary>
    /// Removes the smallest bounded set of edge cut cells needed to obtain a
    /// complete forward-only inbound and cut-to-fill connection. Compact runs
    /// whose physical endpoints touch the edge zone are included even when their
    /// cell centers lie just beyond the one-cell classification threshold.
    /// </summary>
    private static Boolean TryCutWithMinimalAdditionalEdgeOmission
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose Start,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
        ExperimentalCoveragePlanner.PlanResult FillPlan,
        ExperimentalRunSequencer.Result FillSequence,
        Boolean GradeReturn,
        IReadOnlyList<IReadOnlyCollection<(Int32 Bx, Int32 By)>>
            PreferredRunCellSets,
        out ExperimentalCoveragePlanner.PlanResult SelectedPlan,
        out ExperimentalRunSequencer.Result SelectedSequence,
        out ExperimentalCoveragePlanner.PlanResult SelectedFillPlan,
        out ExperimentalRunSequencer.Result SelectedFillSequence,
        Action<String>? Diagnostic = null
        )
    {
        SelectedFillPlan = FillPlan;
        SelectedFillSequence = FillSequence;
        ExperimentalFieldEdgePolicy EdgePolicy =
            new ExperimentalFieldEdgePolicy(Grid, Grid.GridSizeM);
        const Int32 MaximumCompactEdgeRunCells = 2;
        List<ExperimentalCoveragePlanner.Run> CompactEndpointEdgeRuns =
            OrderedCoverageCandidates(Cells, Grid, BladeM)
                .SelectMany(Candidate => Candidate.Runs)
                .Where(Run => Run.Cells.Count <= MaximumCompactEdgeRunCells)
                .Where(Run => EdgePolicy.IsFieldPointNearOuterEdge(
                        Run.StartXy.X, Run.StartXy.Y)
                    || EdgePolicy.IsFieldPointNearOuterEdge(
                        Run.EndXy.X, Run.EndXy.Y))
                .ToList();
        HashSet<(Int32 Bx, Int32 By)> CompactEndpointEdgeCells =
            CompactEndpointEdgeRuns
                .SelectMany(Run => Run.Cells.Select(Entry => Entry.Cell))
                .ToHashSet();
        HashSet<(Int32 Bx, Int32 By)> PreferredRunCells = PreferredRunCellSets
            .SelectMany(Set => Set)
            .ToHashSet();
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> EdgeCells = Cells
            .Where(Entry => (PreferredRunCellSets.Count > 0)
                || EdgePolicy.IsEdgeWorkCell(Entry.Cell)
                || CompactEndpointEdgeCells.Contains(Entry.Cell)
                || PreferredRunCells.Contains(Entry.Cell))
            .OrderBy(Entry => Entry.Cell.Bx)
            .ThenBy(Entry => Entry.Cell.By)
            .ToList();
        const Int32 MaximumOmissionSetsPerTier = 128;
        Int32 MaximumAdditionalOmissions = Math.Min(6, EdgeCells.Count);
        ExperimentalConnectorPlanner.Pose FillEntry =
            FillSequence.Runs[0].EntryPose;
        Dictionary<(Int32 Bx, Int32 By), Int32> EdgeCellIndices = EdgeCells
            .Select((Entry, Index) => (Entry.Cell, Index))
            .ToDictionary(Item => Item.Cell, Item => Item.Index);
        HashSet<String> CompactRunRemovalKeys = CompactEndpointEdgeRuns
            .Select(Run => Run.Cells
                .Select(Entry => Entry.Cell)
                .Where(Cell => EdgeCellIndices.ContainsKey(Cell))
                .Select(Cell => EdgeCellIndices[Cell])
                .Distinct()
                .OrderBy(Index => Index)
                .ToArray())
            .Where(Set => (Set.Length > 0)
                && (Set.Length <= MaximumAdditionalOmissions))
            .Select(Set => String.Join(",", Set))
            .ToHashSet(StringComparer.Ordinal);
        List<Int32[]> PreferredRunRemovalSets = PreferredRunCellSets
            .Select(Set => Set
                .Where(Cell => EdgeCellIndices.ContainsKey(Cell))
                .Select(Cell => EdgeCellIndices[Cell])
                .Distinct()
                .OrderBy(Index => Index)
                .ToArray())
            .Where(Set => (Set.Length > 0)
                && (Set.Length <= MaximumAdditionalOmissions))
            .ToList();
        HashSet<String> PreferredRunRemovalKeys = PreferredRunRemovalSets
            .Select(Set => String.Join(",", Set))
            .ToHashSet(StringComparer.Ordinal);
        List<Int32[]> PreferredRunSubsetSets = PreferredRunRemovalSets
            .SelectMany(Set => Enumerable.Range(1, Set.Length)
                .SelectMany(Count => EnumerateIndexCombinations(
                    Set.Length, Count, MaximumOmissionSetsPerTier)
                    .Select(Subset => Subset.Select(Index => Set[Index])
                        .OrderBy(Index => Index)
                        .ToArray())))
            .GroupBy(Set => String.Join(",", Set), StringComparer.Ordinal)
            .Select(Group => Group.First())
            .ToList();
        List<Int32[]> ConflictExpansionSets = new List<Int32[]>();
        List<Int32[]> PriorityRemovalSets = BuildPriorityEdgeRemovalSets(
            Cells,
            EdgeCells,
            EdgeCellIndices,
            Grid,
            BladeM,
            MaximumAdditionalOmissions)
            .OrderBy(Set => Set.Length)
            .ThenBy(Set => PreferredRunRemovalKeys.Contains(
                String.Join(",", Set)) ? 0 : 1)
            .ThenBy(Set => CompactRunRemovalKeys.Contains(
                String.Join(",", Set)) ? 0 : 1)
            .ThenBy(Set => String.Join(",", Set), StringComparer.Ordinal)
            .ToList();
        for (Int32 OmissionCount = 1;
            OmissionCount <= MaximumAdditionalOmissions;
            OmissionCount += 1)
        {
            Int32 OmissionSets = 0;
            HashSet<String> SeenSets = new HashSet<String>(StringComparer.Ordinal);
            IEnumerable<Int32[]> CandidateRemovalSets =
                PreferredRunSubsetSets.Count > 0
                    ? PreferredRunSubsetSets
                        .Where(Set => Set.Length == OmissionCount)
                    : PriorityRemovalSets
                        .Where(Set => Set.Length == OmissionCount)
                        .Concat(EnumerateIndexCombinations(
                            EdgeCells.Count,
                            OmissionCount,
                            MaximumOmissionSetsPerTier));
            foreach (Int32[] RemovedIndices in CandidateRemovalSets)
            {
                String RemovalKey = String.Join(",", RemovedIndices);
                if (!SeenSets.Add(RemovalKey)) continue;
                if (OmissionSets >= MaximumOmissionSetsPerTier) break;
                OmissionSets += 1;
                if (PreferredRunRemovalKeys.Contains(RemovalKey))
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan infeasible_run_omission_candidate removed={0} set={1} cells=[{2}]",
                        OmissionCount,
                        RemovalKey,
                        String.Join(",", RemovedIndices.Select(Index =>
                            String.Format(
                                CultureInfo.InvariantCulture,
                                "{0}:{1}",
                                EdgeCells[Index].Cell.Bx,
                                EdgeCells[Index].Cell.By)))));
                }
                if (CompactRunRemovalKeys.Contains(RemovalKey))
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan compact_edge_run_omission_candidate removed={0} set={1} cells=[{2}]",
                        OmissionCount,
                        RemovalKey,
                        String.Join(",", RemovedIndices.Select(Index =>
                            String.Format(
                                CultureInfo.InvariantCulture,
                                "{0}:{1}",
                                EdgeCells[Index].Cell.Bx,
                                EdgeCells[Index].Cell.By)))));
                }
                if ((OmissionSets % 25) == 0)
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan edge_cut_omission_progress removed={0} sets_tested={1} limit={2}",
                        OmissionCount,
                        OmissionSets,
                        MaximumOmissionSetsPerTier));
                }
                HashSet<(Int32 Bx, Int32 By)> RemovedCells = RemovedIndices
                    .Select(Index => EdgeCells[Index].Cell)
                    .ToHashSet();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Remaining = Cells
                    .Where(Entry => !RemovedCells.Contains(Entry.Cell))
                    .ToList();
                const Int32 MaximumDetourCoverageCandidates = 1;
                Boolean JointFillRecoveryAttempted = false;
                List<ExperimentalCoveragePlanner.PlanResult>
                    DetourCoverageCandidates =
                        new List<ExperimentalCoveragePlanner.PlanResult>();
                foreach (ExperimentalCoveragePlanner.PlanResult Candidate in
                    OrderedCoverageCandidates(
                        Remaining,
                        Grid,
                        BladeM,
                        (Start.X, Start.Y),
                        (FillEntry.X, FillEntry.Y)))
                {
                    if (Candidate.Runs.Count == 0) continue;
                    ExperimentalRunSequencer.Result Sequence = BuildSequence(
                        Candidate.Runs,
                        Grid,
                        Start,
                        FillEntry);
                    if (!Sequence.Success)
                    {
                        Sequence = BuildFlexibleSequence(
                            Candidate.Runs,
                            Grid,
                            Start,
                            FillEntry);
                    }
                    if (!Sequence.Success
                        || !PoseConnectsToSequence(Start, Sequence, Grid)
                        || !SequencesConnect(Sequence, FillSequence, Grid))
                    {
                        if (!JointFillRecoveryAttempted)
                        {
                            JointFillRecoveryAttempted = true;
                            if (TrySelectFeasibleCutFillPair(
                                Remaining,
                                FillCells,
                                Grid,
                                BladeM,
                                Start,
                                GradeReturn,
                                out ExperimentalCoveragePlanner.PlanResult
                                    AlternativeCut,
                                out ExperimentalRunSequencer.Result
                                    AlternativeCutSequence,
                                out ExperimentalCoveragePlanner.PlanResult
                                    AlternativeFill,
                                out ExperimentalRunSequencer.Result
                                    AlternativeFillSequence))
                            {
                                List<((Int32 Bx, Int32 By) Cell,
                                    Double DepthM)> JointOmitted =
                                        new List<((Int32, Int32), Double)>(
                                            AlternativeCut.OmittedCells);
                                JointOmitted.AddRange(RemovedIndices.Select(
                                    Index => EdgeCells[Index]));
                                SelectedPlan =
                                    new ExperimentalCoveragePlanner.PlanResult(
                                        AlternativeCut.HeadingDeg,
                                        AlternativeCut.Runs,
                                        AlternativeCut.ConnectorDistanceM,
                                        AlternativeCut.AnchorDistanceM,
                                        AlternativeCut.Score,
                                        JointOmitted);
                                SelectedSequence = AlternativeCutSequence;
                                SelectedFillPlan = AlternativeFill;
                                SelectedFillSequence =
                                    AlternativeFillSequence;
                                Diagnostic?.Invoke(String.Format(
                                    CultureInfo.InvariantCulture,
                                    "whole_plan edge_cut_omission_joint_fill_result removed={0} sets_tested={1} cut_heading={2:F1} fill_heading={3:F1} total_omitted={4}",
                                    OmissionCount,
                                    OmissionSets,
                                    AlternativeCut.HeadingDeg,
                                    AlternativeFill.HeadingDeg,
                                    JointOmitted.Count));
                                return true;
                            }
                        }
                        if (DetourCoverageCandidates.Count
                            < MaximumDetourCoverageCandidates)
                        {
                            DetourCoverageCandidates.Add(Candidate);
                        }
                        continue;
                    }
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Omitted =
                        new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(
                            Candidate.OmittedCells);
                    Omitted.AddRange(RemovedIndices.Select(Index => EdgeCells[Index]));
                    SelectedPlan = new ExperimentalCoveragePlanner.PlanResult(
                        Candidate.HeadingDeg,
                        Candidate.Runs,
                        Candidate.ConnectorDistanceM,
                        Candidate.AnchorDistanceM,
                        Candidate.Score,
                        Omitted);
                    SelectedSequence = Sequence;
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan edge_cut_omission_result removed={0} sets_tested={1} coverage_omitted={2} total_omitted={3}",
                        OmissionCount,
                        OmissionSets,
                        Candidate.OmittedCells.Count,
                        Omitted.Count));
                    return true;
                }
                foreach (ExperimentalCoveragePlanner.PlanResult Candidate in
                    DetourCoverageCandidates)
                {
                    ExperimentalRunSequencer.Result Sequence =
                        ExperimentalRunSequencer.OptimizeFlexibleEndpoints(
                            Candidate.Runs,
                            Start,
                            FillEntry,
                            Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                                * Constants.FEET_TO_METERS,
                            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5
                                * Constants.FEET_TO_METERS,
                            Math.Min(0.5, Grid.GridSizeM),
                            Grid,
                            AllowForwardDetour: true);
                    if (!Sequence.Success
                        || !PoseConnectsToSequence(Start, Sequence, Grid)
                        || !SequencesConnect(Sequence, FillSequence, Grid))
                    {
                        foreach (Int32 RunIndex in
                            Sequence.InfeasibleRunIndices)
                        {
                            if ((RunIndex < 0)
                                || (RunIndex >= Candidate.Runs.Count))
                            {
                                continue;
                            }
                            Int32[] Expanded = RemovedIndices
                                .Concat(Candidate.Runs[RunIndex].Cells
                                    .Select(Entry => Entry.Cell)
                                    .Where(Cell => EdgeCellIndices
                                        .ContainsKey(Cell))
                                    .Select(Cell => EdgeCellIndices[Cell]))
                                .Distinct()
                                .OrderBy(Index => Index)
                                .ToArray();
                            if ((Expanded.Length <= RemovedIndices.Length)
                                || (Expanded.Length
                                    > MaximumAdditionalOmissions))
                            {
                                continue;
                            }
                            String ExpandedKey = String.Join(",", Expanded);
                            if (PreferredRunRemovalKeys.Add(ExpandedKey))
                            {
                                ConflictExpansionSets.Add(Expanded);
                                Diagnostic?.Invoke(String.Format(
                                    CultureInfo.InvariantCulture,
                                    "whole_plan infeasible_run_omission_expand from={0} to={1} heading={2:F1} run={3} cells=[{4}]",
                                    RemovalKey,
                                    ExpandedKey,
                                    Candidate.HeadingDeg,
                                    RunIndex,
                                    String.Join(",", Expanded.Select(Index =>
                                        String.Format(
                                            CultureInfo.InvariantCulture,
                                            "{0}:{1}",
                                            EdgeCells[Index].Cell.Bx,
                                            EdgeCells[Index].Cell.By)))));
                            }
                        }
                        continue;
                    }
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Omitted =
                        new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(
                            Candidate.OmittedCells);
                    Omitted.AddRange(RemovedIndices.Select(
                        Index => EdgeCells[Index]));
                    SelectedPlan = new ExperimentalCoveragePlanner.PlanResult(
                        Candidate.HeadingDeg,
                        Candidate.Runs,
                        Candidate.ConnectorDistanceM,
                        Candidate.AnchorDistanceM,
                        Candidate.Score,
                        Omitted);
                    SelectedSequence = Sequence;
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan edge_cut_omission_detour_result removed={0} sets_tested={1} coverage_omitted={2} total_omitted={3}",
                        OmissionCount,
                        OmissionSets,
                        Candidate.OmittedCells.Count,
                        Omitted.Count));
                    return true;
                }
            }
            if (ConflictExpansionSets.Count > 0)
            {
                PreferredRunSubsetSets.AddRange(ConflictExpansionSets);
                ConflictExpansionSets.Clear();
            }
            Diagnostic?.Invoke(String.Format(
                CultureInfo.InvariantCulture,
                "whole_plan edge_cut_omission_tier removed={0} sets_tested={1} selected=False truncated={2}",
                OmissionCount,
                OmissionSets,
                OmissionSets >= MaximumOmissionSetsPerTier));
        }
        SelectedPlan = ExperimentalCoveragePlanner.Plan(
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(), Grid, BladeM);
        SelectedSequence = BuildSequence(SelectedPlan.Runs, Grid);
        return false;
    }

    /// <summary>
    /// Prioritizes whole boundary run fragments and connected corner components
    /// before arbitrary cell combinations of the same omission count.
    /// </summary>
    private static List<Int32[]> BuildPriorityEdgeRemovalSets
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> EdgeCells,
        Dictionary<(Int32 Bx, Int32 By), Int32> EdgeCellIndices,
        Grid Grid,
        Double BladeM,
        Int32 MaximumSize
        )
    {
        Dictionary<String, Int32[]> Sets =
            new Dictionary<String, Int32[]>(StringComparer.Ordinal);
        void AddSet(IEnumerable<(Int32 Bx, Int32 By)> CellsToRemove)
        {
            Int32[] Indices = CellsToRemove
                .Where(Cell => EdgeCellIndices.ContainsKey(Cell))
                .Select(Cell => EdgeCellIndices[Cell])
                .Distinct()
                .OrderBy(Index => Index)
                .ToArray();
            if ((Indices.Length == 0) || (Indices.Length > MaximumSize)) return;
            Sets[String.Join(",", Indices)] = Indices;
        }
        foreach (ExperimentalCoveragePlanner.PlanResult Candidate in
            OrderedCoverageCandidates(Cells, Grid, BladeM))
        {
            foreach (ExperimentalCoveragePlanner.Run Run in Candidate.Runs)
            {
                AddSet(Run.Cells.Select(Entry => Entry.Cell));
            }
        }
        HashSet<(Int32 Bx, Int32 By)> Unvisited = EdgeCells
            .Select(Entry => Entry.Cell)
            .ToHashSet();
        while (Unvisited.Count > 0)
        {
            (Int32 Bx, Int32 By) Seed = Unvisited
                .OrderBy(Cell => Cell.Bx)
                .ThenBy(Cell => Cell.By)
                .First();
            Queue<(Int32 Bx, Int32 By)> Queue =
                new Queue<(Int32 Bx, Int32 By)>();
            List<(Int32 Bx, Int32 By)> Component =
                new List<(Int32 Bx, Int32 By)>();
            Queue.Enqueue(Seed);
            Unvisited.Remove(Seed);
            while (Queue.Count > 0)
            {
                (Int32 Bx, Int32 By) Cell = Queue.Dequeue();
                Component.Add(Cell);
                for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                {
                    for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                    {
                        if ((Dx == 0) && (Dy == 0)) continue;
                        (Int32 Bx, Int32 By) Neighbor =
                            (Cell.Bx + Dx, Cell.By + Dy);
                        if (Unvisited.Remove(Neighbor)) Queue.Enqueue(Neighbor);
                    }
                }
            }
            AddSet(Component);
        }
        return Sets.Values
            .OrderBy(Set => Set.Length)
            .ThenBy(Set => String.Join(",", Set), StringComparer.Ordinal)
            .ToList();
    }

    /// <summary>Enumerates lexicographic fixed-size index sets under a hard cap.</summary>
    private static IEnumerable<Int32[]> EnumerateIndexCombinations
        (
        Int32 ItemCount,
        Int32 SelectionCount,
        Int32 Limit
        )
    {
        if ((SelectionCount <= 0)
            || (SelectionCount > ItemCount)
            || (Limit <= 0))
        {
            yield break;
        }
        Int32[] Indices = Enumerable.Range(0, SelectionCount).ToArray();
        Int32 Produced = 0;
        while (Produced < Limit)
        {
            yield return (Int32[])Indices.Clone();
            Produced += 1;
            Int32 Position = SelectionCount - 1;
            while ((Position >= 0)
                && (Indices[Position] == ItemCount - SelectionCount + Position))
            {
                Position -= 1;
            }
            if (Position < 0) yield break;
            Indices[Position] += 1;
            for (Int32 Index = Position + 1;
                Index < SelectionCount;
                Index += 1)
            {
                Indices[Index] = Indices[Index - 1] + 1;
            }
        }
    }

    /// <summary>Plans and validates a fill after removing one fixed edge-cell set.</summary>
    private static Boolean TryBuildFillAfterOmissions
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Removed,
        Grid Grid,
        Double BladeM,
        ExperimentalConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out ExperimentalCoveragePlanner.PlanResult SelectedPlan,
        out ExperimentalRunSequencer.Result SelectedSequence
        )
    {
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Remaining =
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Cells);
        foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Removed)
        {
            if (!Remaining.Remove(Entry))
            {
                SelectedPlan = ExperimentalCoveragePlanner.Plan(
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(), Grid, BladeM);
                SelectedSequence = BuildSequence(SelectedPlan.Runs, Grid);
                return false;
            }
        }
        foreach (ExperimentalCoveragePlanner.PlanResult Candidate in
            MinimalOmissionCandidates(
                Remaining, Grid, BladeM, (Start.X, Start.Y), null))
        {
            ExperimentalRunSequencer.Result Sequence =
                BuildSequence(Candidate.Runs, Grid, Start, null);
            if (!Sequence.Success
                || !PoseConnectsToSequence(Start, Sequence, Grid)
                || (GradeReturn && !SupportsGradeReturn(Sequence, Grid)))
            {
                continue;
            }
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Omitted =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(
                    Candidate.OmittedCells);
            Omitted.AddRange(Removed);
            SelectedPlan = new ExperimentalCoveragePlanner.PlanResult(
                Candidate.HeadingDeg,
                Candidate.Runs,
                Candidate.ConnectorDistanceM,
                Candidate.AnchorDistanceM,
                Candidate.Score,
                Omitted);
            SelectedSequence = Sequence;
            return true;
        }
        SelectedPlan = ExperimentalCoveragePlanner.Plan(
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(), Grid, BladeM);
        SelectedSequence = BuildSequence(SelectedPlan.Runs, Grid);
        return false;
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

    /// <summary>Returns a linearly interpolated percentile from an ascending sample.</summary>
    private static Double Percentile
        (
        List<Double> SortedValues,
        Double Fraction
        )
    {
        if (SortedValues.Count == 0) return 0.0;
        Double Position = Math.Max(0.0, Math.Min(1.0, Fraction))
            * (SortedValues.Count - 1);
        Int32 Lower = (Int32)Math.Floor(Position);
        Int32 Upper = (Int32)Math.Ceiling(Position);
        if (Lower == Upper) return SortedValues[Lower];
        Double Weight = Position - Lower;
        return SortedValues[Lower]
            + (Weight * (SortedValues[Upper] - SortedValues[Lower]));
    }

    /// <summary>Finds the nearest driven heading before or after one lifecycle segment.</summary>
    private static Double AdjacentHeading
        (
        List<ExperimentalLifecyclePlanner.Segment> Segments,
        Int32 SegmentIndex,
        Int32 Direction
        )
    {
        for (Int32 Index = SegmentIndex + Direction;
            (Index >= 0) && (Index < Segments.Count);
            Index += Direction)
        {
            List<(Double X, Double Y)> Points = Segments[Index].Points;
            if (Points.Count < 2) continue;
            if (Direction < 0)
            {
                for (Int32 PointIndex = Points.Count - 1; PointIndex > 0; PointIndex -= 1)
                {
                    if (Distance(Points[PointIndex - 1], Points[PointIndex]) <= 1e-9) continue;
                    return BearingDeg(Points[PointIndex - 1], Points[PointIndex]);
                }
            }
            else
            {
                for (Int32 PointIndex = 1; PointIndex < Points.Count; PointIndex += 1)
                {
                    if (Distance(Points[PointIndex - 1], Points[PointIndex]) <= 1e-9) continue;
                    return BearingDeg(Points[PointIndex - 1], Points[PointIndex]);
                }
            }
        }
        return Double.NaN;
    }

    /// <summary>Computes the smallest absolute difference between two headings.</summary>
    private static Double HeadingDeltaDeg
        (
        Double First,
        Double Second
        )
    {
        if (!Double.IsFinite(First) || !Double.IsFinite(Second)) return Double.NaN;
        Double Difference = ((First - Second + 540.0) % 360.0) - 180.0;
        return Math.Abs(Difference);
    }

    /// <summary>
    /// Builds the same restored and refined route sequence used by baseline exports.
    /// </summary>
    private static (AgdField Field, Grid Grid, Route Route, Double HeadingDeg, EdgePatchOmissionResult EdgeOmission) BuildRoute
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
        HaulBuilderOptions EdgePatchOptions = new HaulBuilderOptions
        {
            OmitIsolatedEdgePatches = true,
            MaxIsolatedEdgePatchCells = 1,
            MaxIsolatedEdgePatchVolumeBcy = 0.6,
            MaxTotalIsolatedEdgePatchOmissionBcy = 2.0,
            MaxTotalIsolatedEdgePatchOmissionPercent = 1.0,
        };
        EdgePatchOmissionResult EdgeOmission = EdgePatchOmission.Apply(
            PassResult,
            Grid,
            EdgePatchOptions,
            Obstacles.EMPTY_OBSTACLES);
        PassResult = EdgeOmission.PassResult;
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
        return (Field, Grid, Route, HeadingDeg, EdgeOmission);
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
            Original.Score,
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Original.OmittedCells));
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
