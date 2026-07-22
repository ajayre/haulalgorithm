using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace HaulBuilder
{
/// <summary>
/// Builds production coverage, forward-only connectors, lifecycle guidance, and diagnostics.
/// </summary>
static public class GuidancePlanner
{
    private const Int32 MAX_PAIRED_CANDIDATE_ATTEMPTS = 72 * 72;
    private const Int32 MAX_BACKTRACK_CANDIDATE_ATTEMPTS = 250000;
    private const Int32 MAX_INITIAL_CUT_CANDIDATE_ATTEMPTS = 6;
    private const Int32 DIAGNOSTIC_ROUTE_LIMIT = 20;
    private const Double ENDPOINT_MATCH_TOLERANCE_M = 0.01;
    private const Int32 CANDIDATE_PROGRESS_INTERVAL = 10000;
    private const Int32 OMISSION_PROGRESS_INTERVAL = 25;
    private static readonly AsyncLocal<GeometrySettings?> CurrentGeometry =
        new AsyncLocal<GeometrySettings?>();

    /// <summary>One invocation's user-configured guidance geometry.</summary>
    private sealed class GeometrySettings
    {
        /// <summary>Initializes one geometry configuration.</summary>
        public GeometrySettings
            (
            Double MinimumRadiusM,
            Double PreferredLineupM
            )
        {
            this.MinimumRadiusM = MinimumRadiusM;
            this.PreferredLineupM = PreferredLineupM;
        }

        /// <summary>Gets the minimum turning radius.</summary>
        public Double MinimumRadiusM { get; }

        /// <summary>Gets the preferred straight lineup distance.</summary>
        public Double PreferredLineupM { get; }
    }

    /// <summary>Gets the active or default minimum turning radius.</summary>
    private static Double PlanningMinimumRadiusM =>
        CurrentGeometry.Value?.MinimumRadiusM
        ?? Constants.DEFAULT_TURNING_CIRCLE_FT * Constants.HALF_SCALE * Constants.FEET_TO_METERS;

    /// <summary>Gets the active or default lineup distance.</summary>
    private static Double PlanningPreferredLineupM =>
        CurrentGeometry.Value?.PreferredLineupM
        ?? Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS;

    /// <summary>Compact isolated-cycle feasibility used by haul allocation refinement.</summary>
    public sealed class CompactCycleResult
    {
        /// <summary>Initializes one compact-cycle feasibility result.</summary>
        public CompactCycleResult
            (
            Boolean Success,
            Int32 CutRunCount,
            Int32 FillRunCount,
            Double DistanceM,
            String FailureReason
            )
        {
            this.Success = Success;
            this.CutRunCount = CutRunCount;
            this.FillRunCount = FillRunCount;
            this.DistanceM = DistanceM;
            this.FailureReason = FailureReason;
        }

        /// <summary>Gets whether the minimum-run cut and fill geometry is feasible.</summary>
        public Boolean Success { get; }

        /// <summary>Gets the initial cut run count.</summary>
        public Int32 CutRunCount { get; }

        /// <summary>Gets the initial fill run count.</summary>
        public Int32 FillRunCount { get; }

        /// <summary>Gets compact work and connector distance.</summary>
        public Double DistanceM { get; }

        /// <summary>Gets the first failed compact-cycle requirement.</summary>
        public String FailureReason { get; }
    }

    /// <summary>Complete production guidance and timing outputs.</summary>
    public sealed class BuildResult
    {
        /// <summary>Initializes one production guidance result.</summary>
        public BuildResult
            (
            LifecyclePlanner.Result Lifecycle,
            GuidanceTiming.Result Timing,
            GuidanceResult Guidance,
            IReadOnlyList<LifecyclePlanner.PassPlan> Passes
            )
        {
            this.Lifecycle = Lifecycle;
            this.Timing = Timing;
            this.Guidance = Guidance;
            this.Passes = Passes;
        }

        /// <summary>Gets the final connected forward-only lifecycle.</summary>
        public LifecyclePlanner.Result Lifecycle { get; }

        /// <summary>Gets the detailed final-geometry timing result.</summary>
        public GuidanceTiming.Result Timing { get; }

        /// <summary>Gets the operator-facing waypoint adapter.</summary>
        public GuidanceResult Guidance { get; }

        /// <summary>Gets the final selected cut/fill sequences used by the lifecycle.</summary>
        public IReadOnlyList<LifecyclePlanner.PassPlan> Passes { get; }
    }

    /// <summary>Builds the default production guidance and final-geometry timing.</summary>
    static public BuildResult Build
        (
        Route Route,
        Grid Grid,
        AgdField Field,
        Double HeadingDeg,
        HaulBuilderOptions Options,
        Action<String, Int32, Int32>? Progress = null,
        Action<String>? Diagnostic = null
        )
    {
        if (Route == null) throw new ArgumentNullException(nameof(Route));
        if (Grid == null) throw new ArgumentNullException(nameof(Grid));
        if (Field == null) throw new ArgumentNullException(nameof(Field));
        if (Options == null) throw new ArgumentNullException(nameof(Options));
        Double BladeM = Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER;
        List<String> PlanningDiagnostics = new List<String>();
        // Records a diagnostic for both the result and the optional live sink.
        // Message: Human-readable planning diagnostic.
        void ReportDiagnostic(String Message)
        {
            PlanningDiagnostics.Add(Message);
            Diagnostic?.Invoke(Message);
        }
        GeometrySettings? PreviousGeometry = CurrentGeometry.Value;
        List<LifecyclePlanner.PassPlan> Passes;
        CurrentGeometry.Value = new GeometrySettings(
            Options.MinimumTurningDiameterFt * Constants.HALF_SCALE * Constants.FEET_TO_METERS,
            Options.EquipmentLengthFt * Constants.FEET_TO_METERS);
        try
        {
            Passes = BuildWholePlanPasses(
                Route,
                Grid,
                BladeM,
                true,
                Progress: Progress,
                Diagnostic: ReportDiagnostic,
                StopOnFailure: true);
        }
        finally
        {
            CurrentGeometry.Value = PreviousGeometry;
        }
        LifecyclePlanner.Result Lifecycle = LifecyclePlanner.Build(
            Passes,
            true,
            Options.EquipmentLengthFt * Constants.FEET_TO_METERS,
            Options.MinimumTurningDiameterFt * Constants.HALF_SCALE * Constants.FEET_TO_METERS,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid,
            FillSequencesPrepared: true);
        if (!Lifecycle.Success)
        {
            String Detail = PlanningDiagnostics.LastOrDefault(Message =>
                Message.Contains("sequence_failure", StringComparison.Ordinal))
                ?? "No sequencing diagnostic was recorded.";
            throw new InvalidOperationException(
                "Guidance lifecycle construction failed: "
                + Lifecycle.FailureReason
                + " "
                + Detail);
        }
        GuidanceTiming.Result Timing = GuidanceTiming.Estimate(
            Lifecycle, new GuidanceTiming.Options(Options));
        GuidanceResult Guidance = BuildGuidanceResult(
            Lifecycle, Timing, Route, Grid, Field, HeadingDeg, Options);
        return new BuildResult(Lifecycle, Timing, Guidance, Passes);
    }

    /// <summary>
    /// Applies buffered static and per-route obstacles to an existing exact
    /// guidance plan, then retimes the resulting final geometry.
    /// </summary>
    static public (BuildResult Plan, GuidanceObstacleRouter.Result Routing)
        ApplyObstacleRouting
        (
        BuildResult Source,
        Route Route,
        Grid Grid,
        AgdField Field,
        Double HeadingDeg,
        HaulBuilderOptions Options,
        ObstacleField? StaticObstacles = null,
        IReadOnlyDictionary<Int32, ObstacleField>? PerRouteObstacles = null
        )
    {
        if (Source == null) throw new ArgumentNullException(nameof(Source));
        GuidanceObstacleRouter.Result Routed = GuidanceObstacleRouter.Apply(
            Source.Lifecycle,
            Grid,
            Options.MinimumTurningDiameterFt * Constants.HALF_SCALE * Constants.FEET_TO_METERS,
            StaticObstacles,
            PerRouteObstacles);
        if (ReferenceEquals(Routed.Lifecycle, Source.Lifecycle))
        {
            return (Source, Routed);
        }
        GuidanceTiming.Result Timing = GuidanceTiming.Estimate(
            Routed.Lifecycle, new GuidanceTiming.Options(Options));
        GuidanceResult Guidance = BuildGuidanceResult(
            Routed.Lifecycle, Timing, Route, Grid, Field, HeadingDeg, Options);
        return (new BuildResult(
            Routed.Lifecycle,
            Timing,
            Guidance,
            Source.Passes), Routed);
    }

    /// <summary>Adapts lifecycle geometry to the established waypoint export model.</summary>
    private static GuidanceResult BuildGuidanceResult
        (
        LifecyclePlanner.Result Lifecycle,
        GuidanceTiming.Result Timing,
        Route Route,
        Grid Grid,
        AgdField Field,
        Double HeadingDeg,
        HaulBuilderOptions Options
        )
    {
        List<GuidancePoint> Points = new List<GuidancePoint>();
        Double[] CutDepthM = Route.OrderedPasses
            .Select(Pass => AverageDepth(Pass.CutCells)).ToArray();
        Double[] FillDepthM = Route.OrderedPasses
            .Select(Pass => AverageDepth(Pass.FillCells)).ToArray();
        Boolean HasPending = false;
        Boolean HasPreviousPoint = false;
        (Double X, Double Y) PreviousPoint = (0.0, 0.0);
        (Double X, Double Y) PendingPoint = (0.0, 0.0);
        Int32 PendingPassNo = 1;
        String PendingAction = "START";
        String PendingBlade = "UP";
        Double PendingDepthMm = 0.0;
        Double PendingSegmentM = 0.0;
        Double PendingCumulativeM = 0.0;
        Double CumulativeM = 0.0;
        Int32 TurnCount = 0;
        Double TurnAngleRad = 0.0;

        // Appends one waypoint while preserving segment and turn metrics.
        // Point, pass, action, blade, and depth describe the waypoint to append.
        void AppendPoint
            (
            (Double X, Double Y) Point,
            Int32 PassNo,
            String Action,
            String Blade,
            Double DepthMm
            )
        {
            if (!HasPending)
            {
                HasPending = true;
                PendingPoint = Point;
                PendingPassNo = PassNo;
                PendingAction = "START";
                PendingBlade = "UP";
                PendingDepthMm = 0.0;
                return;
            }
            Double SegmentM = Distance(PendingPoint, Point);
            if (SegmentM <= Constants.NUMERIC_COMPARISON_TOLERANCE) return;
            if (HasPreviousPoint)
            {
                Double ChangeDeg = HeadingChangeDeg(
                    PreviousPoint, PendingPoint, Point);
                if (ChangeDeg >= Options.WaypointTurnThresholdDeg)
                {
                    TurnCount += 1;
                    TurnAngleRad += ChangeDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
                }
            }
            (Double Lat, Double Lon) LatLon =
                Field.LatLonFromUtm(PendingPoint.X, PendingPoint.Y);
            Points.Add(new GuidancePoint(
                Points.Count + 1,
                PendingPassNo,
                PendingAction,
                PendingBlade,
                PendingPoint.X,
                PendingPoint.Y,
                LatLon.Lat,
                LatLon.Lon,
                PendingDepthMm,
                BearingDeg(PendingPoint, Point),
                PendingSegmentM,
                PendingCumulativeM,
                GuidanceInstruction(PendingAction, PendingDepthMm)));
            PreviousPoint = PendingPoint;
            HasPreviousPoint = true;
            CumulativeM += SegmentM;
            PendingPoint = Point;
            PendingPassNo = PassNo;
            PendingAction = Action;
            PendingBlade = Blade;
            PendingDepthMm = DepthMm;
            PendingSegmentM = SegmentM;
            PendingCumulativeM = CumulativeM;
        }

        Double ReturnDistanceM = 0.0;
        Double HaulDistanceM = 0.0;
        Double CutDistanceM = 0.0;
        Double FillDistanceM = 0.0;
        Double GradeDistanceM = 0.0;
        Double RepositionDistanceM = 0.0;
        foreach (LifecyclePlanner.Segment Segment in Lifecycle.Segments)
        {
            Int32 PassNo = Math.Max(1, Segment.RouteNumber);
            String Action = GuidanceAction(Segment);
            String Blade = Segment.BladeDown ? "DOWN" : "UP";
            Int32 DepthIndex = Math.Min(
                Math.Max(0, PassNo - 1), Math.Max(0, Route.OrderedPasses.Count - 1));
            Double DepthMm = Segment.Phase switch
            {
                LifecyclePlanner.Phase.Cut when CutDepthM.Length > 0 =>
                    Constants.MILLIMETERS_PER_METER * CutDepthM[DepthIndex],
                LifecyclePlanner.Phase.Fill when FillDepthM.Length > 0 =>
                    Constants.MILLIMETERS_PER_METER * FillDepthM[DepthIndex],
                _ => 0.0,
            };
            Double SegmentDistanceM = PolylineDistance(Segment.Points);
            switch (Action)
            {
                case "CUT": CutDistanceM += SegmentDistanceM; break;
                case "FILL": FillDistanceM += SegmentDistanceM; break;
                case "GRADE": GradeDistanceM += SegmentDistanceM; break;
                case "HAUL": HaulDistanceM += SegmentDistanceM; break;
                case "RETURN": ReturnDistanceM += SegmentDistanceM; break;
                case "MOVE": RepositionDistanceM += SegmentDistanceM; break;
                case "MANUAL_MOVE": RepositionDistanceM += SegmentDistanceM; break;
            }
            for (Int32 PointIndex = 0;
                PointIndex < Segment.Points.Count;
                PointIndex += 1)
            {
                AppendPoint(
                    Segment.Points[PointIndex], PassNo, Action, Blade, DepthMm);
            }
        }
        if (HasPending)
        {
            (Double Lat, Double Lon) LatLon =
                Field.LatLonFromUtm(PendingPoint.X, PendingPoint.Y);
            Points.Add(new GuidancePoint(
                Points.Count + 1,
                PendingPassNo,
                PendingAction,
                PendingBlade,
                PendingPoint.X,
                PendingPoint.Y,
                LatLon.Lat,
                LatLon.Lon,
                PendingDepthMm,
                0.0,
                PendingSegmentM,
                PendingCumulativeM,
                GuidanceInstruction(PendingAction, PendingDepthMm)));
        }
        return new GuidanceResult(
            Points,
            Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
            Grid.GridSizeM,
            HeadingDeg,
            Route.OrderedPasses.Count,
            ReturnDistanceM,
            HaulDistanceM,
            CutDistanceM,
            FillDistanceM,
            GradeDistanceM,
            RepositionDistanceM,
            CutDistanceM,
            FillDistanceM,
            Route.OrderedPasses.Sum(Pass => Pass.HaulDistanceM),
            TurnCount,
            TurnAngleRad,
            0);
    }

    /// <summary>Maps a lifecycle segment to an established waypoint action.</summary>
    private static String GuidanceAction
        (
        LifecyclePlanner.Segment Segment
        )
    {
        return Segment.Phase switch
        {
            LifecyclePlanner.Phase.Start => "START",
            LifecyclePlanner.Phase.Cut => "CUT",
            LifecyclePlanner.Phase.Fill => "FILL",
            LifecyclePlanner.Phase.Grade => "GRADE",
            LifecyclePlanner.Phase.ManualAssist => "MANUAL_MOVE",
            _ => Segment.Loaded ? "HAUL" : "RETURN",
        };
    }

    /// <summary>Returns concise operator text for one exported action.</summary>
    private static String GuidanceInstruction
        (
        String Action,
        Double DepthMm
        )
    {
        return Action switch
        {
            "CUT" => String.Format(
                CultureInfo.InvariantCulture, "Cut approximately {0:F0} mm", DepthMm),
            "FILL" => String.Format(
                CultureInfo.InvariantCulture, "Fill approximately {0:F0} mm", DepthMm),
            "GRADE" => "Grade the completed fill",
            "HAUL" => "Travel loaded",
            "RETURN" => "Travel unloaded",
            "MOVE" => "Operator-supervised guidance",
            "MANUAL_MOVE" =>
                "MANUAL MOVE: operator must verify and drive clear of obstruction",
            _ => "Begin guidance",
        };
    }

    /// <summary>Returns mean absolute work depth for one route allocation.</summary>
    private static Double AverageDepth
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
        )
    {
        return Cells.Count == 0
            ? 0.0
            : Cells.Average(Cell => Math.Abs(Cell.DepthM));
    }

    /// <summary>
    /// Tests whether one allocation retains its minimum-run geometry through the
    /// immutable cut-to-fill and grade-return guidance rules.
    /// </summary>
    static public CompactCycleResult EvaluateCompactCycle
        (
        Pass Pass,
        Grid Grid,
        Double BladeM,
        Boolean GradeReturn
        )
    {
        if (Pass == null) throw new ArgumentNullException(nameof(Pass));
        if (Grid == null) throw new ArgumentNullException(nameof(Grid));
        CoveragePlanner.PlanResult Cut = CoveragePlanner.Plan(
            Pass.CutCells, Grid, BladeM, null, Pass.FillStartXy);
        (Double X, Double Y)? FillAnchor = Cut.Runs.Count == 0
            ? null
            : Cut.Runs[Cut.Runs.Count - 1].EndXy;
        CoveragePlanner.PlanResult Fill = CoveragePlanner.Plan(
            Pass.FillCells, Grid, BladeM, FillAnchor, null);
        RunSequencer.Result FillSequence = BuildSequence(Fill.Runs, Grid);
        if (!FillSequence.Success)
        {
            return new CompactCycleResult(
                false,
                Cut.Runs.Count,
                Fill.Runs.Count,
                Double.PositiveInfinity,
                "FILL_SEQUENCE");
        }
        ConnectorPlanner.Pose? FillEntry = FillSequence.Runs.Count == 0
            ? null
            : FillSequence.Runs[0].EntryPose;
        RunSequencer.Result CutSequence = BuildSequence(
            Cut.Runs, Grid, null, FillEntry);
        if (!CutSequence.Success)
        {
            return new CompactCycleResult(
                false,
                Cut.Runs.Count,
                Fill.Runs.Count,
                Double.PositiveInfinity,
                "CUT_SEQUENCE");
        }
        if ((CutSequence.Runs.Count > 0) && (Fill.Runs.Count > 0))
        {
            ConnectorPlanner.Pose CutExit = CutSequence.Runs[
                CutSequence.Runs.Count - 1].ExitPose;
            FillSequence = BuildSequence(
                Fill.Runs, Grid, CutExit, null);
            if (GradeReturn
                && FillSequence.Success
                && (FillSequence.Runs.Count > 1))
            {
                FillSequence = LifecyclePlanner.BuildContinuousFillSequence(
                    FillSequence,
                    CutExit,
                    PlanningMinimumRadiusM,
                    Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                    Grid);
            }
        }
        if (!FillSequence.Success
            || !SequencesConnect(CutSequence, FillSequence, Grid))
        {
            return new CompactCycleResult(
                false,
                Cut.Runs.Count,
                Fill.Runs.Count,
                Double.PositiveInfinity,
                "CUT_TO_FILL");
        }
        if (GradeReturn && !SupportsGradeReturn(
            FillSequence,
            Grid,
            FillSequencePrepared: true))
        {
            return new CompactCycleResult(
                false,
                Cut.Runs.Count,
                Fill.Runs.Count,
                Double.PositiveInfinity,
                "GRADE_RETURN");
        }
        Double DistanceM = Cut.WorkDistanceM
            + Fill.WorkDistanceM
            + CutSequence.ConnectorDistanceM
            + FillSequence.ConnectorDistanceM;
        if ((CutSequence.Runs.Count > 0) && (FillSequence.Runs.Count > 0))
        {
            TransitPlanner.Result Connector = TransitPlanner.Connect(
                CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose,
                FillSequence.Runs[0].EntryPose,
                PlanningMinimumRadiusM,
                Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                Grid);
            if (Connector.Success) DistanceM += Connector.LengthM;
        }
        if (GradeReturn) DistanceM += Fill.WorkDistanceM;
        return new CompactCycleResult(
            true,
            Cut.Runs.Count,
            Fill.Runs.Count,
            DistanceM,
            String.Empty);
    }

    /// <summary>
    /// Builds isolated straight coverage and sequences for every ordered TheShop2 pass.
    /// </summary>
    private static List<LifecyclePlanner.PassPlan> BuildWholePlanPasses
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
        Diagnostic?.Invoke(String.Format(
            CultureInfo.InvariantCulture,
            "whole_plan building_passes={0}",
            PassCount));
        CoveragePlanner.PlanResult?[] CutPlans =
            new CoveragePlanner.PlanResult?[PassCount];
        CoveragePlanner.PlanResult?[] FillPlans =
            new CoveragePlanner.PlanResult?[PassCount];
        RunSequencer.Result?[] PreselectedFillSequences =
            new RunSequencer.Result?[PassCount];
        String?[] Failures = new String?[PassCount];
        Int32 CoverageCompleted = 0;
        Progress?.Invoke("coverage", 0, PassCount);
        Parallel.For(0, PassCount, PassIndex =>
        {
            Pass Pass = Route.OrderedPasses[PassIndex];
            CoveragePlanner.PlanResult Cut = CoveragePlanner.Plan(
                Pass.CutCells, Grid, BladeM, null, Pass.FillStartXy);
            (Double X, Double Y)? FillAnchor = Cut.Runs.Count == 0
                ? null
                : Cut.Runs[Cut.Runs.Count - 1].EndXy;
            CoveragePlanner.PlanResult Fill = CoveragePlanner.Plan(
                Pass.FillCells, Grid, BladeM, FillAnchor, null);
            CutPlans[PassIndex] = Cut;
            FillPlans[PassIndex] = Fill;
            Progress?.Invoke(
                "coverage",
                Interlocked.Increment(ref CoverageCompleted),
                PassCount);
        });
        List<LifecyclePlanner.PassPlan> Output =
            new List<LifecyclePlanner.PassPlan>(PassCount);
        ConnectorPlanner.Pose? Current = null;
        Progress?.Invoke("sequencing", 0, PassCount);
        for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
        {
            Stopwatch PassTimer = Stopwatch.StartNew();
            Stopwatch StageTimer = Stopwatch.StartNew();
            // Reports slow per-pass stages while avoiding noise from fast stages.
            // Stage: Name of the completed planning stage.
            void ReportStage(String Stage)
            {
                if (StageTimer.Elapsed >= TimeSpan.FromSeconds(Constants.QUARTER_SCALE))
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
            CoveragePlanner.PlanResult Cut = CutPlans[PassIndex]
                ?? throw new InvalidOperationException("Parallel coverage construction omitted cut work.");
            CoveragePlanner.PlanResult Fill = FillPlans[PassIndex]
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
            Diagnostic?.Invoke(String.Format(
                CultureInfo.InvariantCulture,
                "whole_plan pass={0} cut_runs={1} fill_runs={2} cut_omitted={3} fill_omitted={4}",
                PassIndex + 1,
                Cut.Runs.Count,
                Fill.Runs.Count,
                Cut.OmittedCells.Count,
                Fill.OmittedCells.Count));
            RunSequencer.Result FillSequence =
                PreselectedFillSequences[PassIndex]
                ?? BuildSequence(Fill.Runs, Grid);
            if (GradeReturn && !SupportsGradeReturn(FillSequence, Grid))
            {
                foreach (CoveragePlanner.PlanResult Candidate in
                    MinimalOmissionCandidates(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM))
                {
                    RunSequencer.Result CandidateSequence =
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
            ConnectorPlanner.Pose? FillEntry = FillSequence.Runs.Count == 0
                ? null
                : FillSequence.Runs[0].EntryPose;
            // The next cut is part of the same continuous forward-only track.
            // Anchor its ordering to the actual post-grade pose so final
            // connectivity is selected, not merely checked after the fact.
            ConnectorPlanner.Pose? CutSearchStart = Current;
            RunSequencer.Result CutSequence = BuildSequence(
                Cut.Runs, Grid, CutSearchStart, FillEntry);
            if (!CutSequence.Success || !SequencesConnect(CutSequence, FillSequence, Grid))
            {
                Int32 InitialCutCandidateIndex = 0;
                foreach (CoveragePlanner.PlanResult Candidate in
                    MinimalOmissionCandidates(
                        Route.OrderedPasses[PassIndex].CutCells,
                        Grid,
                        BladeM,
                        Current.HasValue ? (Current.Value.X, Current.Value.Y) : null,
                        FillEntry.HasValue ? (FillEntry.Value.X, FillEntry.Value.Y) : null)
                    .Take(MAX_INITIAL_CUT_CANDIDATE_ATTEMPTS))
                {
                    RunSequencer.Result CandidateSequence =
                        BuildCutSequenceWithEndpointRecovery(
                            Candidate.Runs,
                            Grid,
                            CutSearchStart,
                            FillEntry);
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan initial_cut_candidate pass={0} index={1} heading={2:F1} runs={3} success={4} connects={5} reason={6}",
                        PassIndex + 1,
                        InitialCutCandidateIndex,
                        Candidate.HeadingDeg,
                        Candidate.Runs.Count,
                        CandidateSequence.Success,
                        CandidateSequence.Success
                            && SequencesConnect(CandidateSequence, FillSequence, Grid),
                        CandidateSequence.FailureReason));
                    InitialCutCandidateIndex += 1;
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
                List<CoveragePlanner.PlanResult> FillCandidates =
                    MinimalOmissionCandidates(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM);
                List<CoveragePlanner.PlanResult> CutCandidates =
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
                RunSequencer.Result?[] ProvisionalFillSequences =
                    new RunSequencer.Result?[FillCandidates.Count];
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
                    CoveragePlanner.PlanResult FillCandidate =
                        FillCandidates[Pair.FillIndex];
                    CoveragePlanner.PlanResult CutCandidate =
                        CutCandidates[Pair.CutIndex];
                    RunSequencer.Result ProvisionalFillSequence =
                        ProvisionalFillSequences[Pair.FillIndex]
                            ?? BuildSequence(FillCandidate.Runs, Grid);
                    ProvisionalFillSequences[Pair.FillIndex] =
                        ProvisionalFillSequence;
                    if (!ProvisionalFillSequence.Success
                        || (ProvisionalFillSequence.Runs.Count == 0))
                    {
                        continue;
                    }
                    ConnectorPlanner.Pose CandidateFillEntry =
                        ProvisionalFillSequence.Runs[0].EntryPose;
                    RunSequencer.Result CandidateCutSequence =
                        BuildCutSequenceWithEndpointRecovery(
                            CutCandidate.Runs,
                            Grid,
                            CutSearchStart,
                            CandidateFillEntry);
                    if (!CandidateCutSequence.Success
                        || (CandidateCutSequence.Runs.Count == 0))
                    {
                        continue;
                    }
                    ConnectorPlanner.Pose CandidateCutExit =
                        CandidateCutSequence.Runs[
                            CandidateCutSequence.Runs.Count - 1].ExitPose;
                    RunSequencer.Result CandidateFillSequence =
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
                            LifecyclePlanner.BuildContinuousFillSequence(
                                CandidateFillSequence,
                                CandidateCutExit,
                                PlanningMinimumRadiusM,
                                Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
                    "whole_plan pair_result pass={0} attempts={1} selected={2} fill_index={3} cut_index={4} cut_heading={5:F1} cut_runs={6} fill_heading={7:F1} fill_runs={8} truncated={9}",
                    PassIndex + 1,
                    PairAttempts,
                    SelectedPair,
                    SelectedFillIndex,
                    SelectedCutIndex,
                    SelectedCutIndex >= 0
                        ? CutCandidates[SelectedCutIndex].HeadingDeg
                        : -1.0,
                    SelectedCutIndex >= 0
                        ? CutCandidates[SelectedCutIndex].Runs.Count
                        : -1,
                    SelectedFillIndex >= 0
                        ? FillCandidates[SelectedFillIndex].HeadingDeg
                        : -1.0,
                    SelectedFillIndex >= 0
                        ? FillCandidates[SelectedFillIndex].Runs.Count
                        : -1,
                    !SelectedPair && (PairAttempts < CandidatePairs.Count)));
            }
            ReportStage("paired_cut_fill_recovery");
            if (CutSequence.Success
                && (CutSequence.Runs.Count > 0)
                && (Fill.Runs.Count > 0))
            {
                ConnectorPlanner.Pose CutExit =
                    CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose;
                ConnectorPlanner.Pose? NextWorkEntry = null;
                if ((PassIndex + 1) < PassCount)
                {
                    CoveragePlanner.PlanResult? NextCut = CutPlans[PassIndex + 1];
                    if ((NextCut != null) && (NextCut.Runs.Count > 0))
                    {
                        RunSequencer.Result NextCutSequence =
                            BuildSequence(NextCut.Runs, Grid);
                        if (NextCutSequence.Success && (NextCutSequence.Runs.Count > 0))
                        {
                            NextWorkEntry = NextCutSequence.Runs[0].EntryPose;
                        }
                    }
                    if (!NextWorkEntry.HasValue)
                    {
                        CoveragePlanner.PlanResult? NextFill =
                            FillPlans[PassIndex + 1];
                        if ((NextFill != null) && (NextFill.Runs.Count > 0))
                        {
                            RunSequencer.Result NextFillSequence =
                                BuildSequence(NextFill.Runs, Grid);
                            if (NextFillSequence.Success
                                && (NextFillSequence.Runs.Count > 0))
                            {
                                NextWorkEntry = NextFillSequence.Runs[0].EntryPose;
                            }
                        }
                    }
                }
                Double BaselineDirectionScoreM = Double.PositiveInfinity;
                Double SelectedDirectionScoreM = Double.PositiveInfinity;
                Int32 DirectionCandidatesEvaluated = 0;
                RunSequencer.Result ConnectedFillSequence = GradeReturn
                    ? SelectGradeAwareFillDirection(
                        Fill.Runs,
                        FillSequence,
                        Grid,
                        CutExit,
                        NextWorkEntry,
                        out BaselineDirectionScoreM,
                        out SelectedDirectionScoreM,
                        out DirectionCandidatesEvaluated)
                    : BuildSequence(
                        Fill.Runs, Grid, CutExit, NextWorkEntry);
                if (ConnectedFillSequence.Success
                    && (!GradeReturn || SupportsGradeReturn(
                        ConnectedFillSequence,
                        Grid,
                        FillSequencePrepared: true))
                    && SequencesConnect(CutSequence, ConnectedFillSequence, Grid)
                    && (GradeReturn
                        || !NextWorkEntry.HasValue
                        || SequenceConnectsToPose(
                            ConnectedFillSequence,
                            NextWorkEntry.Value,
                            Grid)))
                {
                    FillSequence = ConnectedFillSequence;
                }
                if (GradeReturn)
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan grade_aware_fill_direction pass={0} candidates={1} baseline_m={2:F2} selected_m={3:F2} improvement_m={4:F2} entry=({5:F1},{6:F1},{7:F1}) post_grade=({8:F1},{9:F1},{10:F1})",
                        PassIndex + 1,
                        DirectionCandidatesEvaluated,
                        BaselineDirectionScoreM,
                        SelectedDirectionScoreM,
                        Double.IsFinite(BaselineDirectionScoreM)
                            && Double.IsFinite(SelectedDirectionScoreM)
                                ? BaselineDirectionScoreM
                                    - SelectedDirectionScoreM
                                : 0.0,
                        ConnectedFillSequence.Runs.Count == 0
                            ? 0.0
                            : ConnectedFillSequence.Runs[0].EntryPose.X,
                        ConnectedFillSequence.Runs.Count == 0
                            ? 0.0
                            : ConnectedFillSequence.Runs[0].EntryPose.Y,
                        ConnectedFillSequence.Runs.Count == 0
                            ? 0.0
                            : ConnectedFillSequence.Runs[0].EntryPose.HeadingDeg,
                        ConnectedFillSequence.Runs.Count == 0
                            ? 0.0
                            : ReversePose(
                                ConnectedFillSequence.Runs[0].EntryPose).X,
                        ConnectedFillSequence.Runs.Count == 0
                            ? 0.0
                            : ReversePose(
                                ConnectedFillSequence.Runs[0].EntryPose).Y,
                        ConnectedFillSequence.Runs.Count == 0
                            ? 0.0
                            : ReversePose(
                                ConnectedFillSequence.Runs[0].EntryPose)
                                .HeadingDeg));
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
                        out CoveragePlanner.PlanResult SelectedCurrentFill,
                        out RunSequencer.Result SelectedCurrentFillSequence,
                        out CoveragePlanner.PlanResult SelectedNextFill,
                        out RunSequencer.Result SelectedNextFillSequence))
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
                    out CoveragePlanner.PlanResult ChainedCurrentFill,
                    out RunSequencer.Result ChainedCurrentFillSequence,
                    out CoveragePlanner.PlanResult ChainedNextFill,
                    out RunSequencer.Result ChainedNextFillSequence))
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
                foreach (CoveragePlanner.PlanResult FillCandidate in
                    MinimalOmissionCandidates(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM,
                        (Current.Value.X, Current.Value.Y),
                        null))
                {
                    RunSequencer.Result CandidateFillSequence =
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
                    LifecyclePlanner.PassPlan PreviousPass =
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
                            out CoveragePlanner.PlanResult BacktrackedPreviousFill,
                            out RunSequencer.Result BacktrackedPreviousSequence,
                            out CoveragePlanner.PlanResult BacktrackedCurrentFill,
                            out RunSequencer.Result BacktrackedCurrentSequence))
                    {
                        Output[PassIndex - 1] =
                            new LifecyclePlanner.PassPlan(
                                PreviousPass.Cut,
                                BacktrackedPreviousSequence,
                                PreviousPass.CutOmittedCells,
                                BacktrackedPreviousFill.OmittedCells);
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
                    LifecyclePlanner.PassPlan PreviousPass =
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
                            out CoveragePlanner.PlanResult TriplePreviousFill,
                            out RunSequencer.Result TriplePreviousSequence,
                            out CoveragePlanner.PlanResult TripleCut,
                            out RunSequencer.Result TripleCutSequence,
                            out CoveragePlanner.PlanResult TripleFill,
                            out RunSequencer.Result TripleFillSequence))
                    {
                        Output[PassIndex - 1] =
                            new LifecyclePlanner.PassPlan(
                                PreviousPass.Cut,
                                TriplePreviousSequence,
                                PreviousPass.CutOmittedCells,
                                TriplePreviousFill.OmittedCells);
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
                        out CoveragePlanner.PlanResult TrimmedFill,
                        out RunSequencer.Result TrimmedFillSequence))
                {
                    Fill = TrimmedFill;
                    FillSequence = TrimmedFillSequence;
                }
            }
            ReportStage("fill_backtracking_and_omission");
            if (GradeReturn && FillSequence.Success && (FillSequence.Runs.Count > 1))
            {
                ConnectorPlanner.Pose? FillInbound =
                    CutSequence.Success && (CutSequence.Runs.Count > 0)
                        ? CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose
                        : Current;
                FillSequence = LifecyclePlanner.BuildContinuousFillSequence(
                    FillSequence,
                    FillInbound,
                    PlanningMinimumRadiusM,
                    Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
                RunSequencer.Result UnanchoredCut =
                    BuildFlexibleSequence(Cut.Runs, Grid);
                if (UnanchoredCut.Success && (UnanchoredCut.Runs.Count > 0))
                {
                    RunSequencer.Result ConnectedFill =
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
                    out CoveragePlanner.PlanResult AlternativeCut,
                    out RunSequencer.Result AlternativeCutSequence))
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
                ConnectorPlanner.Pose? AlternativeFillStart =
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
                        out CoveragePlanner.PlanResult AlternativeFill,
                        out RunSequencer.Result AlternativeFillSequence))
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
                ConnectorPlanner.Pose? FillInbound =
                    CutSequence.Success && (CutSequence.Runs.Count > 0)
                        ? CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose
                        : Current;
                FillSequence = LifecyclePlanner.BuildContinuousFillSequence(
                    FillSequence,
                    FillInbound,
                    PlanningMinimumRadiusM,
                    Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
                ConnectorPlanner.Pose? GradeRecoveryStart =
                    CutSequence.Runs.Count > 0
                        ? CutSequence.Runs[
                            CutSequence.Runs.Count - 1].ExitPose
                        : Current;
                CoveragePlanner.PlanResult GradeFill = Fill;
                RunSequencer.Result GradeFillSequence =
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
                    RunSequencer.SequencedRun LastFill =
                        FillSequence.Runs[FillSequence.Runs.Count - 1];
                    ConnectorPlanner.Pose GradeEntry =
                        new ConnectorPlanner.Pose(
                            LastFill.Run.EndXy.X,
                            LastFill.Run.EndXy.Y,
                            LastFill.Run.HeadingDeg + Constants.DEGREES_PER_HALF_TURN);
                    TransitPlanner.Result GradeTurn =
                        TransitPlanner.Connect(
                            LastFill.ExitPose,
                            GradeEntry,
                            PlanningMinimumRadiusM,
                            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                            Grid,
                            EdgeAllowanceM: Grid.GridSizeM,
                            AllowForwardDetour: true);
                    FieldEdgePolicy GradeEdgePolicy =
                        new FieldEdgePolicy(
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
                    FillSequence = new RunSequencer.Result(
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
                RunSequencer.Result DetourCutSequence =
                    RunSequencer.OptimizeFlexibleEndpoints(
                        Cut.Runs,
                        Current.Value,
                        FillSequence.Runs[0].EntryPose,
                        PlanningPreferredLineupM,
                        PlanningMinimumRadiusM,
                        Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
                    out CoveragePlanner.PlanResult RecoveredCut,
                    out RunSequencer.Result RecoveredCutSequence,
                    out CoveragePlanner.PlanResult RecoveredFill,
                    out RunSequencer.Result RecoveredFillSequence,
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
                    out CoveragePlanner.PlanResult TrimmedCut,
                    out RunSequencer.Result TrimmedCutSequence,
                    out CoveragePlanner.PlanResult RecoveredInboundFill,
                    out RunSequencer.Result RecoveredInboundFillSequence,
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
                LifecyclePlanner.PassPlan PreviousPass =
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
                        out CoveragePlanner.PlanResult BacktrackedPreviousFill,
                        out RunSequencer.Result BacktrackedPreviousSequence,
                        out CoveragePlanner.PlanResult BacktrackedCut,
                        out RunSequencer.Result BacktrackedCutSequence,
                        out CoveragePlanner.PlanResult BacktrackedFill,
                        out RunSequencer.Result BacktrackedFillSequence,
                        Diagnostic))
                {
                    Output[PassIndex - 1] =
                        new LifecyclePlanner.PassPlan(
                            PreviousPass.Cut,
                            BacktrackedPreviousSequence,
                            PreviousPass.CutOmittedCells,
                            BacktrackedPreviousFill.OmittedCells);
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
                LifecyclePlanner.PassPlan PreviousPass =
                    Output[PassIndex - 1];
                ConnectorPlanner.Pose? PreviousIncoming =
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
                        out RunSequencer.Result DeeperPreviousCut,
                        out CoveragePlanner.PlanResult DeeperPreviousFill,
                        out RunSequencer.Result DeeperPreviousFillSequence,
                        out CoveragePlanner.PlanResult DeeperCut,
                        out RunSequencer.Result DeeperCutSequence,
                        out CoveragePlanner.PlanResult DeeperFill,
                        out RunSequencer.Result DeeperFillSequence,
                        Diagnostic))
                {
                    Output[PassIndex - 1] =
                        new LifecyclePlanner.PassPlan(
                            DeeperPreviousCut,
                            DeeperPreviousFillSequence,
                            PreviousPass.CutOmittedCells,
                            DeeperPreviousFill.OmittedCells);
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
            if (GradeReturn
                && FillSequence.Success
                && (FillSequence.Runs.Count > 0)
                && !SupportsGradeReturn(
                    FillSequence,
                    Grid,
                    FillSequencePrepared: true))
            {
                ConnectorPlanner.Pose? LateGradeRecoveryStart =
                    CutSequence.Success && (CutSequence.Runs.Count > 0)
                        ? CutSequence.Runs[
                            CutSequence.Runs.Count - 1].ExitPose
                        : Current;
                if (LateGradeRecoveryStart.HasValue
                    && TrySelectFeasibleFillCoverageCandidate(
                        Route.OrderedPasses[PassIndex].FillCells,
                        Grid,
                        BladeM,
                        LateGradeRecoveryStart.Value,
                        true,
                        out CoveragePlanner.PlanResult LateGradeFill,
                        out RunSequencer.Result LateGradeFillSequence))
                {
                    RunSequencer.Result PreparedLateGradeFill =
                        LateGradeFillSequence.Runs.Count > 1
                            ? LifecyclePlanner.BuildContinuousFillSequence(
                                LateGradeFillSequence,
                                LateGradeRecoveryStart,
                                PlanningMinimumRadiusM,
                                Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                                Grid)
                            : LateGradeFillSequence;
                    if (PreparedLateGradeFill.Success
                        && SupportsGradeReturn(
                            PreparedLateGradeFill,
                            Grid,
                            FillSequencePrepared: true)
                        && ((CutSequence.Runs.Count == 0)
                            || SequencesConnect(
                                CutSequence,
                                PreparedLateGradeFill,
                                Grid)))
                    {
                        Fill = LateGradeFill;
                        FillSequence = PreparedLateGradeFill;
                        FillPlans[PassIndex] = LateGradeFill;
                        RetainSelectedLifecycleConnections(
                            Current,
                            CutSequence,
                            FillSequence,
                            GradeReturn,
                            Grid);
                        Diagnostic?.Invoke(String.Format(
                            CultureInfo.InvariantCulture,
                            "whole_plan late_grade_recovery pass={0} heading={1:F1} runs={2} omitted_cells={3}",
                            PassIndex + 1,
                            LateGradeFill.HeadingDeg,
                            PreparedLateGradeFill.Runs.Count,
                            LateGradeFill.OmittedCells.Count));
                    }
                }
            }
            Boolean FinalGradeReturnValid = !GradeReturn
                || !FillSequence.Success
                || (FillSequence.Runs.Count == 0)
                || SupportsGradeReturn(
                    FillSequence,
                    Grid,
                    FillSequencePrepared: true);
            if (!CutSequence.Success
                || !FillSequence.Success
                || !FinalGradeReturnValid
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
                    CutSequence = new RunSequencer.Result(
                        false,
                        CutSequence.Runs,
                        CutSequence.Connectors,
                        CutSequence.ConnectorDistanceM,
                        CutSequence.MinimumLineupM,
                        "No complete incoming connector to the pass's first remaining work satisfies the edge policy.");
                }
                else if (!FinalGradeReturnValid)
                {
                    FillSequence = new RunSequencer.Result(
                        false,
                        FillSequence.Runs,
                        FillSequence.Connectors,
                        FillSequence.ConnectorDistanceM,
                        FillSequence.MinimumLineupM,
                        "Final fill sequence cannot execute its required grade return.");
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
            Output.Add(new LifecyclePlanner.PassPlan(
                CutSequence,
                FillSequence,
                Cut.OmittedCells,
                Fill.OmittedCells));
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
                FieldEdgePolicy FailedEdgePolicy =
                    new FieldEdgePolicy(Grid, Grid.GridSizeM);
                for (Int32 RunIndex = 0;
                    RunIndex < Cut.Runs.Count;
                    RunIndex += 1)
                {
                    CoveragePlanner.Run FailedRun = Cut.Runs[RunIndex];
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
                    CoveragePlanner.Run FailedRun =
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
            Diagnostic?.Invoke(String.Format(
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
                TransitPlanner.Result Link = TransitPlanner.Connect(
                    CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose,
                    FillSequence.Runs[0].EntryPose,
                    PlanningMinimumRadiusM,
                    Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                    Grid);
                Diagnostic?.Invoke(String.Format(
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
                RunSequencer.SequencedRun LastFill =
                    FillSequence.Runs[FillSequence.Runs.Count - 1];
                Current = LastFill.ExitPose;
                if (GradeReturn)
                {
                    List<RunSequencer.SequencedRun> GradeRuns =
                        LifecyclePlanner.BuildGradeRuns(
                            FillSequence.Runs,
                            PlanningPreferredLineupM,
                            PlanningMinimumRadiusM,
                            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                            Grid);
                    Current = GradeRuns[GradeRuns.Count - 1].ExitPose;
                }
            }
        }
        Int32 FailureCount = Failures.Count(Failure => Failure != null);
        Int32 ReportedFailures = 0;
        for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
        {
            if ((Failures[PassIndex] != null) && (ReportedFailures < DIAGNOSTIC_ROUTE_LIMIT))
            {
                Diagnostic?.Invoke(Failures[PassIndex]!);
                ReportedFailures += 1;
            }
        }
        Diagnostic?.Invoke(String.Format(
            CultureInfo.InvariantCulture,
            "whole_plan sequence_failures={0} reported={1}",
            FailureCount,
            ReportedFailures));
        return Output;
    }

    /// <summary>Checks fill, turnaround, and reverse grade connectivity for one pass.</summary>
    private static Boolean SupportsGradeReturn
        (
        RunSequencer.Result FillSequence,
        Grid Grid,
        Boolean FillSequencePrepared = false,
        Boolean RetainResults = false
        )
    {
        RunSequencer.Result Candidate = FillSequencePrepared
            ? FillSequence
            : LifecyclePlanner.BuildContinuousFillSequence(
                FillSequence,
                null,
                PlanningMinimumRadiusM,
                Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                Grid);
        return LifecyclePlanner.SupportsPreparedGradeReturn(
            Candidate,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
        ConnectorPlanner.Pose? Previous,
        RunSequencer.Result Cut,
        RunSequencer.Result Fill,
        Boolean GradeReturn,
        Grid Grid
        )
    {
        Double RadiusM = PlanningMinimumRadiusM;
        Double SampleM = Math.Min(Constants.HALF_SCALE, Grid.GridSizeM);
        ConnectorPlanner.Pose? First = Cut.Runs.Count > 0
            ? Cut.Runs[0].EntryPose
            : Fill.Runs.Count > 0
                ? Fill.Runs[0].EntryPose
                : null;
        if (Previous.HasValue && First.HasValue)
        {
            TransitPlanner.Connect(
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
            TransitPlanner.Connect(
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
    /// Keeps every heading candidate in the smallest available omission tier.
    /// </summary>
    private static List<CoveragePlanner.PlanResult> MinimalOmissionCandidates
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        (Double X, Double Y)? Start = null,
        (Double X, Double Y)? End = null
        )
    {
        List<CoveragePlanner.PlanResult> Candidates =
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
    private static List<CoveragePlanner.PlanResult> OrderedCoverageCandidates
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        (Double X, Double Y)? Start = null,
        (Double X, Double Y)? End = null
        )
    {
        return CoveragePlanner.CandidatePlans(
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
    /// Builds one strict production work sequence for lifecycle comparison.
    /// </summary>
    private static RunSequencer.Result BuildSequence
        (
        List<CoveragePlanner.Run> Runs,
        Grid Grid,
        ConnectorPlanner.Pose? Start = null,
        ConnectorPlanner.Pose? End = null
        )
    {
        return RunSequencer.Optimize(
            Runs,
            Start,
            End,
            PlanningPreferredLineupM,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid);
    }

    /// <summary>Builds a bounded flexible-endpoint sequence after strict ordering fails.</summary>
    private static RunSequencer.Result BuildFlexibleSequence
        (
        List<CoveragePlanner.Run> Runs,
        Grid Grid,
        ConnectorPlanner.Pose? Start = null,
        ConnectorPlanner.Pose? End = null
        )
    {
        return RunSequencer.OptimizeFlexibleEndpoints(
            Runs,
            Start,
            End,
            PlanningPreferredLineupM,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid);
    }

    /// <summary>
    /// Compares both complete fill bearings using the actual post-grade exit.
    /// Reverse grading finishes at the fill's entry end, so an ordinary sequence
    /// end anchor scores the wrong route boundary when grade return is enabled.
    /// </summary>
    static internal RunSequencer.Result SelectGradeAwareFillDirection
        (
        List<CoveragePlanner.Run> Runs,
        RunSequencer.Result Existing,
        Grid Grid,
        ConnectorPlanner.Pose CutExit,
        ConnectorPlanner.Pose? NextWorkEntry,
        out Double BaselineScoreM,
        out Double SelectedScoreM,
        out Int32 CandidatesEvaluated
        )
    {
        if (Runs == null) throw new ArgumentNullException(nameof(Runs));
        if (Existing == null) throw new ArgumentNullException(nameof(Existing));
        if (Grid == null) throw new ArgumentNullException(nameof(Grid));
        BaselineScoreM = Double.PositiveInfinity;
        SelectedScoreM = Double.PositiveInfinity;
        CandidatesEvaluated = 0;
        if (!Existing.Success || (Runs.Count == 0)) return Existing;

        List<RunSequencer.Result> Seeds = new List<RunSequencer.Result>
        {
            Existing,
            LifecyclePlanner.ReversePreparedSequence(Existing),
            BuildSequence(Runs, Grid, CutExit, null),
        };
        if (NextWorkEntry.HasValue)
        {
            // Reversing a forward connector from the post-grade exit to the
            // next work pose produces a connector from the reversed next-work
            // pose to the fill entry.  This anchor exposes the other bearing
            // without weakening any connector rule.
            Seeds.Add(BuildSequence(
                Runs,
                Grid,
                ReversePose(NextWorkEntry.Value),
                null));
        }

        RunSequencer.Result Best = Existing;
        Int32 BestManualAssists = Int32.MaxValue;
        HashSet<String> Seen = new HashSet<String>(StringComparer.Ordinal);
        for (Int32 SeedIndex = 0; SeedIndex < Seeds.Count; SeedIndex += 1)
        {
            RunSequencer.Result Seed = Seeds[SeedIndex];
            if (!Seed.Success || (Seed.Runs.Count == 0)) continue;
            RunSequencer.Result Prepared =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Seed,
                    CutExit,
                    PlanningMinimumRadiusM,
                    Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                    Grid);
            if (!Prepared.Success || (Prepared.Runs.Count == 0)) continue;
            String Key = SequenceDirectionSignature(Prepared);
            if (!Seen.Add(Key)) continue;
            if (!TryScoreGradeAwareFillDirection(
                Prepared,
                Grid,
                CutExit,
                NextWorkEntry,
                out Double ScoreM,
                out Int32 ManualAssists))
            {
                continue;
            }
            CandidatesEvaluated += 1;
            if (SeedIndex == 0) BaselineScoreM = ScoreM;
            if ((ManualAssists > BestManualAssists)
                || ((ManualAssists == BestManualAssists)
                    && (ScoreM >= SelectedScoreM - Constants.NUMERIC_COMPARISON_TOLERANCE)))
            {
                continue;
            }
            Best = Prepared;
            BestManualAssists = ManualAssists;
            SelectedScoreM = ScoreM;
        }
        if (!Double.IsFinite(SelectedScoreM)
            && Double.IsFinite(BaselineScoreM))
        {
            SelectedScoreM = BaselineScoreM;
        }
        return Best;
    }

    /// <summary>Scores the true cut-to-fill-to-grade-to-next-work boundary.</summary>
    private static Boolean TryScoreGradeAwareFillDirection
        (
        RunSequencer.Result Fill,
        Grid Grid,
        ConnectorPlanner.Pose CutExit,
        ConnectorPlanner.Pose? NextWorkEntry,
        out Double ScoreM,
        out Int32 ManualAssists
        )
    {
        ScoreM = Double.PositiveInfinity;
        ManualAssists = Int32.MaxValue;
        if (!LifecyclePlanner.SupportsPreparedGradeReturn(
            Fill,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid,
            out _))
        {
            return false;
        }
        TransitPlanner.Result Inbound = TransitPlanner.Connect(
            CutExit,
            Fill.Runs[0].EntryPose,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid,
            EdgeAllowanceM: Grid.GridSizeM,
            AllowForwardDetour: true);
        if (!Inbound.Success) return false;
        List<RunSequencer.SequencedRun> GradeRuns =
            LifecyclePlanner.BuildGradeRuns(
                Fill.Runs,
                PlanningPreferredLineupM,
                PlanningMinimumRadiusM,
                Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                Grid);
        TransitPlanner.Result TurnToGrade = TransitPlanner.Connect(
            Fill.Runs[Fill.Runs.Count - 1].ExitPose,
            GradeRuns[0].EntryPose,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid,
            EdgeAllowanceM: Grid.GridSizeM,
            AllowForwardDetour: true);
        if (!TurnToGrade.Success) return false;
        TransitPlanner.Result? Outbound = null;
        if (NextWorkEntry.HasValue)
        {
            Outbound = TransitPlanner.Connect(
                GradeRuns[GradeRuns.Count - 1].ExitPose,
                NextWorkEntry.Value,
                PlanningMinimumRadiusM,
                Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                Grid,
                EdgeAllowanceM: Grid.GridSizeM,
                AllowForwardDetour: true);
            if (!Outbound.Success) return false;
        }
        ScoreM = Inbound.LengthM
            + TurnToGrade.LengthM
            + (2.0 * Fill.ConnectorDistanceM)
            + (Outbound?.LengthM ?? 0.0);
        ManualAssists = ConnectorManualAssistCount(Inbound)
            + ConnectorManualAssistCount(TurnToGrade)
            + (Outbound == null ? 0 : ConnectorManualAssistCount(Outbound))
            + (2 * Fill.Connectors.Sum(ConnectorManualAssistCount));
        return true;
    }

    /// <summary>Returns a deterministic signature for a prepared drive direction.</summary>
    private static String SequenceDirectionSignature
        (
        RunSequencer.Result Sequence
        ) => String.Join(
            ";",
            Sequence.Runs.Select(Run => String.Format(
                CultureInfo.InvariantCulture,
                "{0:F2},{1:F2}>{2:F2},{3:F2}@{4:F1}",
                Run.Run.StartXy.X,
                Run.Run.StartXy.Y,
                Run.Run.EndXy.X,
                Run.Run.EndXy.Y,
                Run.Run.HeadingDeg)));

    /// <summary>Counts explicit operator-alignment legs in one connector.</summary>
    private static Int32 ConnectorManualAssistCount
        (
        TransitPlanner.Result Connector
        ) => Connector.Legs.Count(Leg => Leg.OperatorAlignmentRequired);

    /// <summary>
    /// Preserves a compact cut layout by trying bounded endpoint recovery before
    /// the coverage search falls through to a more fragmented heading. Flexible
    /// endpoint distance remains explicit coverage shortfall and the lifecycle
    /// performs it blade-down on the adjoining connector.
    /// </summary>
    static internal RunSequencer.Result BuildCutSequenceWithEndpointRecovery
        (
        List<CoveragePlanner.Run> Runs,
        Grid Grid,
        ConnectorPlanner.Pose? Start = null,
        ConnectorPlanner.Pose? End = null
        )
    {
        RunSequencer.Result Strict = BuildSequence(Runs, Grid, Start, End);
        if (Strict.Success) return Strict;
        RunSequencer.Result Recovered =
            BuildFlexibleSequence(Runs, Grid, Start, End);
        if (!Recovered.Success || (Recovered.Runs.Count == 0))
        {
            return Recovered;
        }
        // Internal shortfalls are worked blade-down by AppendSequenceConnector.
        // A shortfall at the outer ends has no adjoining work connector and would
        // therefore change the completed cut surface, so reject it transactionally.
        if ((Recovered.Runs[0].StartCoverageShortfallM > Constants.NUMERIC_COMPARISON_TOLERANCE)
            || (Recovered.Runs[Recovered.Runs.Count - 1]
                .EndCoverageShortfallM > Constants.NUMERIC_COMPARISON_TOLERANCE))
        {
            return new RunSequencer.Result(
                false,
                Recovered.Runs,
                Recovered.Connectors,
                Recovered.ConnectorDistanceM,
                Recovered.MinimumLineupM,
                "Compact cut recovery would leave an external coverage shortfall.");
        }
        return Recovered;
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
        ConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out CoveragePlanner.PlanResult SelectedPlan,
        out RunSequencer.Result SelectedSequence
        )
    {
        List<CoveragePlanner.PlanResult> Candidates =
            CoveragePlanner.CandidatePlans(
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
        foreach (CoveragePlanner.PlanResult Candidate in Candidates)
        {
            if (TryBuildFeasibleFillCandidate(
                Candidate,
                Grid,
                Start,
                GradeReturn,
                out RunSequencer.Result Sequence))
            {
                SelectedPlan = Candidate;
                SelectedSequence = Sequence;
                return true;
            }
        }
        SelectedPlan = CoveragePlanner.Plan(
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
        ConnectorPlanner.Pose? Start,
        RunSequencer.Result FillSequence,
        out CoveragePlanner.PlanResult SelectedPlan,
        out RunSequencer.Result SelectedSequence
        )
    {
        ConnectorPlanner.Pose FillEntry =
            FillSequence.Runs[0].EntryPose;
        List<CoveragePlanner.PlanResult> Candidates =
            CoveragePlanner.CandidatePlans(
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
        foreach (CoveragePlanner.PlanResult Candidate in Candidates)
        {
            if (Candidate.Runs.Count == 0) continue;
            RunSequencer.Result Sequence = BuildSequence(
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
        SelectedPlan = CoveragePlanner.Plan(
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
        ConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out CoveragePlanner.PlanResult SelectedCutPlan,
        out RunSequencer.Result SelectedCutSequence,
        out CoveragePlanner.PlanResult SelectedFillPlan,
        out RunSequencer.Result SelectedFillSequence
        )
    {
        List<CoveragePlanner.PlanResult> CutCandidates =
            MinimalOmissionCandidates(
                CutCells, Grid, BladeM, (Start.X, Start.Y), null);
        List<CoveragePlanner.PlanResult> FillCandidates =
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
                CoveragePlanner.PlanResult CutCandidate =
                    CutCandidates[CutIndex];
                CoveragePlanner.PlanResult FillCandidate =
                    FillCandidates[FillIndex];
                foreach (ConnectorPlanner.Pose FillEntry in
                    PossibleEntryPoses(FillCandidate.Runs, Grid))
                {
                    if (Attempts >= MAX_PAIRED_CANDIDATE_ATTEMPTS)
                    {
                        goto Failure;
                    }
                    Attempts += 1;
                    RunSequencer.Result CutSequence =
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
                    ConnectorPlanner.Pose CutExit =
                        CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose;
                    if (!TryBuildFeasibleFillCandidate(
                        FillCandidate,
                        Grid,
                        CutExit,
                        GradeReturn,
                        out RunSequencer.Result FillSequence)
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
        SelectedCutPlan = CoveragePlanner.Plan(
            CutCells, Grid, BladeM);
        SelectedCutSequence = BuildSequence(
            SelectedCutPlan.Runs, Grid);
        SelectedFillPlan = CoveragePlanner.Plan(
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
        CoveragePlanner.PlanResult Candidate
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
        CoveragePlanner.PlanResult Candidate,
        Grid Grid,
        ConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out RunSequencer.Result Sequence
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
            Sequence = LifecyclePlanner.BuildContinuousFillSequence(
                Sequence,
                Start,
                PlanningMinimumRadiusM,
                Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                Grid);
        }
        return Sequence.Success
            && PoseConnectsToSequence(Start, Sequence, Grid)
            && (!GradeReturn || SupportsGradeReturn(Sequence, Grid));
    }

    /// <summary>Validates the explicit lifecycle connector between cut exit and fill entry.</summary>
    private static Boolean SequencesConnect
        (
        RunSequencer.Result Cut,
        RunSequencer.Result Fill,
        Grid Grid
        )
    {
        if (!Cut.Success || !Fill.Success) return false;
        if ((Cut.Runs.Count == 0) || (Fill.Runs.Count == 0)) return true;
        TransitPlanner.Result Connector = TransitPlanner.Connect(
            Cut.Runs[Cut.Runs.Count - 1].ExitPose,
            Fill.Runs[0].EntryPose,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid);
        return Connector.Success;
    }

    /// <summary>Validates a previous-pass pose against the first remaining run in a cut-omitted pass.</summary>
    private static Boolean PoseConnectsToSequence
        (
        ConnectorPlanner.Pose Start,
        RunSequencer.Result Sequence,
        Grid Grid,
        Boolean AllowForwardDetour = false
        )
    {
        if (!Sequence.Success) return false;
        if (Sequence.Runs.Count == 0) return true;
        TransitPlanner.Result Connector = TransitPlanner.Connect(
            Start,
            Sequence.Runs[0].EntryPose,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid,
            AllowForwardDetour: AllowForwardDetour);
        return Connector.Success;
    }

    /// <summary>Validates the final run against an explicitly requested following pose.</summary>
    private static Boolean SequenceConnectsToPose
        (
        RunSequencer.Result Sequence,
        ConnectorPlanner.Pose End,
        Grid Grid
        )
    {
        if (!Sequence.Success) return false;
        if (Sequence.Runs.Count == 0) return true;
        TransitPlanner.Result Connector = TransitPlanner.Connect(
            Sequence.Runs[Sequence.Runs.Count - 1].ExitPose,
            End,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
        ConnectorPlanner.Pose CutExit,
        Boolean GradeReturn,
        out CoveragePlanner.PlanResult SelectedCurrentPlan,
        out RunSequencer.Result SelectedCurrentSequence,
        out CoveragePlanner.PlanResult SelectedNextPlan,
        out RunSequencer.Result SelectedNextSequence
        )
    {
        List<CoveragePlanner.PlanResult> CurrentCandidates =
            MinimalOmissionCandidates(
                CurrentCells, Grid, BladeM, (CutExit.X, CutExit.Y), null);
        List<CoveragePlanner.PlanResult> NextCandidates =
            MinimalOmissionCandidates(NextCells, Grid, BladeM);
        CoveragePlanner.PlanResult? BestCurrentPlan = null;
        RunSequencer.Result? BestCurrentSequence = null;
        CoveragePlanner.PlanResult? BestNextPlan = null;
        RunSequencer.Result? BestNextSequence = null;
        Int32 BestOmittedCells = Int32.MaxValue;
        Double BestScore = Double.PositiveInfinity;

        foreach (CoveragePlanner.PlanResult NextCandidate in NextCandidates)
        {
            List<ConnectorPlanner.Pose?> Targets =
                PossibleEntryPoses(NextCandidate.Runs, Grid)
                    .Select(Pose => (ConnectorPlanner.Pose?)Pose)
                    .ToList();
            Targets.Add(null);
            foreach (CoveragePlanner.PlanResult CurrentCandidate in CurrentCandidates)
            {
                foreach (ConnectorPlanner.Pose? Target in Targets)
                {
                    RunSequencer.Result CurrentSequence = BuildSequence(
                        CurrentCandidate.Runs, Grid, CutExit, Target);
                    if (!CurrentSequence.Success
                        || (CurrentSequence.Runs.Count == 0)
                        || !PoseConnectsToSequence(CutExit, CurrentSequence, Grid)
                        || (GradeReturn && !SupportsGradeReturn(CurrentSequence, Grid)))
                    {
                        continue;
                    }
                    ConnectorPlanner.Pose Outgoing = GradeReturn
                        ? ReversePose(CurrentSequence.Runs[0].EntryPose)
                        : CurrentSequence.Runs[
                            CurrentSequence.Runs.Count - 1].ExitPose;
                    RunSequencer.Result NextSequence = BuildSequence(
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
                            && (Score >= BestScore - Constants.NUMERIC_COMPARISON_TOLERANCE)))
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
        SelectedCurrentPlan = CoveragePlanner.Plan(CurrentCells, Grid, BladeM);
        SelectedCurrentSequence = BuildSequence(SelectedCurrentPlan.Runs, Grid);
        SelectedNextPlan = CoveragePlanner.Plan(NextCells, Grid, BladeM);
        SelectedNextSequence = BuildSequence(SelectedNextPlan.Runs, Grid);
        return false;
    }

    /// <summary>Returns every prepared orientation that could begin the next coverage plan.</summary>
    private static List<ConnectorPlanner.Pose> PossibleEntryPoses
        (
        List<CoveragePlanner.Run> Runs,
        Grid Grid
        )
    {
        List<ConnectorPlanner.Pose> Output =
            new List<ConnectorPlanner.Pose>();
        Double LineupM = PlanningPreferredLineupM;
        Double MinimumRadiusM = PlanningMinimumRadiusM;
        Double SampleSpacingM = Math.Min(Constants.HALF_SCALE, Grid.GridSizeM);
        foreach (CoveragePlanner.Run Run in Runs)
        {
            CoveragePlanner.Run Reverse = ReverseRun(Run);
            Output.Add(RunSequencer.Prepare(
                Run, LineupM, MinimumRadiusM, SampleSpacingM, Grid).EntryPose);
            Output.Add(new ConnectorPlanner.Pose(
                Run.StartXy.X, Run.StartXy.Y, Run.HeadingDeg));
            Output.Add(RunSequencer.Prepare(
                Reverse, LineupM, MinimumRadiusM, SampleSpacingM, Grid).EntryPose);
            Output.Add(new ConnectorPlanner.Pose(
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
                BitConverter.DoubleToInt64Bits(Pose.HeadingDeg % Constants.DEGREES_PER_TURN)))
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
        ConnectorPlanner.Pose PreviousCutExit,
        Boolean GradeReturn,
        out CoveragePlanner.PlanResult SelectedPreviousFillPlan,
        out RunSequencer.Result SelectedPreviousFillSequence,
        out CoveragePlanner.PlanResult SelectedCutPlan,
        out RunSequencer.Result SelectedCutSequence,
        out CoveragePlanner.PlanResult SelectedFillPlan,
        out RunSequencer.Result SelectedFillSequence,
        Action<String>? Diagnostic = null
        )
    {
        List<CoveragePlanner.PlanResult> PreviousFillCandidates =
            MinimalOmissionCandidates(
                PreviousFillCells,
                Grid,
                BladeM,
                (PreviousCutExit.X, PreviousCutExit.Y),
                null);
        // If every complete cut layout is unreachable, retain higher-omission
        // layouts as a controlled final recovery. RankedTriples below guarantees
        // that no such layout is tried before every lower-omission alternative.
        List<CoveragePlanner.PlanResult> CutCandidates =
            OrderedCoverageCandidates(CurrentCutCells, Grid, BladeM);
        List<CoveragePlanner.PlanResult> FillCandidates =
            MinimalOmissionCandidates(
                CurrentFillCells, Grid, BladeM);
        CoveragePlanner.PlanResult? BestPreviousFillPlan = null;
        RunSequencer.Result? BestPreviousFillSequence = null;
        CoveragePlanner.PlanResult? BestCutPlan = null;
        RunSequencer.Result? BestCutSequence = null;
        CoveragePlanner.PlanResult? BestFillPlan = null;
        RunSequencer.Result? BestFillSequence = null;
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

        RunSequencer.Result?[] ProvisionalFills =
            new RunSequencer.Result?[FillCandidates.Count];
        List<ConnectorPlanner.Pose>?[] CutEntryTargetsByCandidate =
            new List<ConnectorPlanner.Pose>?[CutCandidates.Count];
        Dictionary<(Int32 PreviousFillIndex, Int32 CutIndex, Int32 TargetIndex),
            (RunSequencer.Result Sequence,
                ConnectorPlanner.Pose Outgoing)?> PreviousOptions =
            new Dictionary<(Int32 PreviousFillIndex, Int32 CutIndex,
                Int32 TargetIndex),
                (RunSequencer.Result Sequence,
                    ConnectorPlanner.Pose Outgoing)?>();
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
            CoveragePlanner.PlanResult PreviousFillCandidate =
                PreviousFillCandidates[Triple.PreviousFillIndex];
            CoveragePlanner.PlanResult CutCandidate =
                CutCandidates[Triple.CutIndex];
            CoveragePlanner.PlanResult FillCandidate =
                FillCandidates[Triple.FillIndex];
            RunSequencer.Result ProvisionalFill =
                ProvisionalFills[Triple.FillIndex]
                ?? BuildSequence(FillCandidate.Runs, Grid);
            ProvisionalFills[Triple.FillIndex] = ProvisionalFill;
            if (!ProvisionalFill.Success || (ProvisionalFill.Runs.Count == 0)) continue;
            if (CutCandidate.Runs.Count == 0) continue;
            ConnectorPlanner.Pose FillEntry =
                ProvisionalFill.Runs[0].EntryPose;
            List<ConnectorPlanner.Pose> CutEntryTargets =
                CutEntryTargetsByCandidate[Triple.CutIndex]
                ?? PossibleEntryPoses(CutCandidate.Runs, Grid);
            CutEntryTargetsByCandidate[Triple.CutIndex] = CutEntryTargets;
            for (Int32 TargetIndex = 0;
                TargetIndex < CutEntryTargets.Count;
                TargetIndex += 1)
            {
                ConnectorPlanner.Pose CutEntryTarget =
                    CutEntryTargets[TargetIndex];
                CandidateAttempts += 1;
                if ((CandidateAttempts % CANDIDATE_PROGRESS_INTERVAL) == 0)
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
                    out (RunSequencer.Result Sequence,
                        ConnectorPlanner.Pose Outgoing)? PreviousOption))
                {
                    RunSequencer.Result PreviousFillSequenceCandidate =
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
                                LifecyclePlanner.BuildContinuousFillSequence(
                                    PreviousFillSequenceCandidate,
                                    PreviousCutExit,
                                    PlanningMinimumRadiusM,
                                    Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
                        ConnectorPlanner.Pose PreviousOutgoingCandidate =
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
                RunSequencer.Result PreviousFillSequence =
                    PreviousOption.Value.Sequence;
                ConnectorPlanner.Pose PreviousOutgoing =
                    PreviousOption.Value.Outgoing;
                        RunSequencer.Result CutSequence = BuildSequence(
                            CutCandidate.Runs, Grid, PreviousOutgoing, FillEntry);
                        if (!CutSequence.Success
                            || !PoseConnectsToSequence(
                                PreviousOutgoing, CutSequence, Grid))
                        {
                            continue;
                        }
                        ConnectorPlanner.Pose CutExit =
                            CutSequence.Runs[CutSequence.Runs.Count - 1].ExitPose;
                        RunSequencer.Result FillSequence = BuildSequence(
                            FillCandidate.Runs, Grid, CutExit, null);
                        if (GradeReturn
                            && FillSequence.Success
                            && (FillSequence.Runs.Count > 1))
                        {
                            FillSequence =
                                LifecyclePlanner.BuildContinuousFillSequence(
                                    FillSequence,
                                    CutExit,
                                    PlanningMinimumRadiusM,
                                    Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
                                && (Score >= BestScore - Constants.NUMERIC_COMPARISON_TOLERANCE)))
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
        SelectedPreviousFillPlan = CoveragePlanner.Plan(
            PreviousFillCells, Grid, BladeM);
        SelectedPreviousFillSequence = BuildSequence(SelectedPreviousFillPlan.Runs, Grid);
        SelectedCutPlan = CoveragePlanner.Plan(CurrentCutCells, Grid, BladeM);
        SelectedCutSequence = BuildSequence(SelectedCutPlan.Runs, Grid);
        SelectedFillPlan = CoveragePlanner.Plan(CurrentFillCells, Grid, BladeM);
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
        RunSequencer.Result ExistingPreviousCut,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> PreviousFillCells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CurrentCutCells,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CurrentFillCells,
        Grid Grid,
        Double BladeM,
        ConnectorPlanner.Pose? PreviousIncoming,
        ConnectorPlanner.Pose ExistingPreviousCutExit,
        Boolean GradeReturn,
        out RunSequencer.Result SelectedPreviousCut,
        out CoveragePlanner.PlanResult SelectedPreviousFillPlan,
        out RunSequencer.Result SelectedPreviousFillSequence,
        out CoveragePlanner.PlanResult SelectedCutPlan,
        out RunSequencer.Result SelectedCutSequence,
        out CoveragePlanner.PlanResult SelectedFillPlan,
        out RunSequencer.Result SelectedFillSequence,
        Action<String>? Diagnostic = null
        )
    {
        List<CoveragePlanner.Run> PreviousCutRuns =
            ExistingPreviousCut.Runs
                .Select(Run => Run.Run)
                .ToList();
        List<ConnectorPlanner.Pose> PreviousFillEntries =
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
        List<RunSequencer.Result> PreviousCutOptions =
            new List<RunSequencer.Result>();
        foreach (ConnectorPlanner.Pose FillEntry in PreviousFillEntries)
        {
            RunSequencer.Result Candidate = BuildSequence(
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
            ConnectorPlanner.Pose Exit = Candidate.Runs[
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
        foreach (RunSequencer.Result PreviousCutOption
            in PreviousCutOptions)
        {
            ConnectorPlanner.Pose PreviousCutExit =
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
        SelectedPreviousFillPlan = CoveragePlanner.Plan(
            PreviousFillCells, Grid, BladeM);
        SelectedPreviousFillSequence = BuildSequence(
            SelectedPreviousFillPlan.Runs, Grid);
        SelectedCutPlan = CoveragePlanner.Plan(
            CurrentCutCells, Grid, BladeM);
        SelectedCutSequence = BuildSequence(SelectedCutPlan.Runs, Grid);
        SelectedFillPlan = CoveragePlanner.Plan(
            CurrentFillCells, Grid, BladeM);
        SelectedFillSequence = BuildSequence(SelectedFillPlan.Runs, Grid);
        return false;
    }

    /// <summary>Returns the final forward pose after one completed pass.</summary>
    private static ConnectorPlanner.Pose? PassOutgoingPose
        (
        LifecyclePlanner.PassPlan Pass,
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
        ConnectorPlanner.Pose First,
        ConnectorPlanner.Pose Second
        )
    {
        return (Math.Abs(First.X - Second.X) <= ENDPOINT_MATCH_TOLERANCE_M)
            && (Math.Abs(First.Y - Second.Y) <= ENDPOINT_MATCH_TOLERANCE_M)
            && (Math.Abs(NormalizeHeading(First.HeadingDeg - Second.HeadingDeg))
                <= 0.1
                || Math.Abs(
                    NormalizeHeading(First.HeadingDeg - Second.HeadingDeg) - Constants.DEGREES_PER_TURN)
                    <= 0.1);
    }

    /// <summary>Normalizes a heading to [0, 360).</summary>
    private static Double NormalizeHeading(Double HeadingDeg)
    {
        Double Output = HeadingDeg % Constants.DEGREES_PER_TURN;
        return Output < 0.0 ? Output + Constants.DEGREES_PER_TURN : Output;
    }

    /// <summary>Reverses a coverage run without changing its assigned work cells.</summary>
    private static CoveragePlanner.Run ReverseRun
        (
        CoveragePlanner.Run Run
        )
    {
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Run.Cells);
        Cells.Reverse();
        return new CoveragePlanner.Run(
            Cells,
            Run.EndXy,
            Run.StartXy,
            Run.HeadingDeg + Constants.DEGREES_PER_HALF_TURN,
            Run.MaximumLateralErrorM);
    }

    /// <summary>Returns the same position facing in the opposite direction.</summary>
    private static ConnectorPlanner.Pose ReversePose
        (
        ConnectorPlanner.Pose Pose
        )
    {
        return new ConnectorPlanner.Pose(
            Pose.X, Pose.Y, Pose.HeadingDeg + Constants.DEGREES_PER_HALF_TURN);
    }

    /// <summary>
    /// Finds a feasible incoming edge fill using the smallest tested boundary-cell omission set.
    /// </summary>
    private static Boolean TryFillWithMinimalAdditionalEdgeOmission
        (
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
        Grid Grid,
        Double BladeM,
        ConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out CoveragePlanner.PlanResult SelectedPlan,
        out RunSequencer.Result SelectedSequence
        )
    {
        FieldEdgePolicy EdgePolicy =
            new FieldEdgePolicy(Grid, BladeM);
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> EdgeCells = Cells
            .Where(Entry => EdgePolicy.IsEdgeWorkCell(Entry.Cell))
            .ToList();
        Int32 MaximumAdditionalOmissions = Math.Min(3, EdgeCells.Count);
        for (Int32 OmissionCount = 1;
            OmissionCount <= MaximumAdditionalOmissions;
            OmissionCount += 1)
        {
            CoveragePlanner.PlanResult? BestPlan = null;
            RunSequencer.Result? BestSequence = null;
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
                            out CoveragePlanner.PlanResult CandidatePlan,
                            out RunSequencer.Result CandidateSequence))
                        {
                            continue;
                        }
                        if (CandidatePlan.Score >= BestScore - Constants.NUMERIC_COMPARISON_TOLERANCE) continue;
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
        SelectedPlan = CoveragePlanner.Plan(
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
        ConnectorPlanner.Pose Start,
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
        CoveragePlanner.PlanResult FillPlan,
        RunSequencer.Result FillSequence,
        Boolean GradeReturn,
        IReadOnlyList<IReadOnlyCollection<(Int32 Bx, Int32 By)>>
            PreferredRunCellSets,
        out CoveragePlanner.PlanResult SelectedPlan,
        out RunSequencer.Result SelectedSequence,
        out CoveragePlanner.PlanResult SelectedFillPlan,
        out RunSequencer.Result SelectedFillSequence,
        Action<String>? Diagnostic = null
        )
    {
        SelectedFillPlan = FillPlan;
        SelectedFillSequence = FillSequence;
        FieldEdgePolicy EdgePolicy =
            new FieldEdgePolicy(Grid, Grid.GridSizeM);
        const Int32 MAXIMUM_COMPACT_EDGE_RUN_CELLS = 2;
        List<CoveragePlanner.Run> CompactEndpointEdgeRuns =
            OrderedCoverageCandidates(Cells, Grid, BladeM)
                .SelectMany(Candidate => Candidate.Runs)
                .Where(Run => Run.Cells.Count <= MAXIMUM_COMPACT_EDGE_RUN_CELLS)
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
        const Int32 MAXIMUM_OMISSION_SETS_PER_TIER = 128;
        Int32 MaximumAdditionalOmissions = Math.Min(6, EdgeCells.Count);
        ConnectorPlanner.Pose FillEntry =
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
                    Set.Length, Count, MAXIMUM_OMISSION_SETS_PER_TIER)
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
                            MAXIMUM_OMISSION_SETS_PER_TIER));
            foreach (Int32[] RemovedIndices in CandidateRemovalSets)
            {
                String RemovalKey = String.Join(",", RemovedIndices);
                if (!SeenSets.Add(RemovalKey)) continue;
                if (OmissionSets >= MAXIMUM_OMISSION_SETS_PER_TIER) break;
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
                if ((OmissionSets % OMISSION_PROGRESS_INTERVAL) == 0)
                {
                    Diagnostic?.Invoke(String.Format(
                        CultureInfo.InvariantCulture,
                        "whole_plan edge_cut_omission_progress removed={0} sets_tested={1} limit={2}",
                        OmissionCount,
                        OmissionSets,
                        MAXIMUM_OMISSION_SETS_PER_TIER));
                }
                HashSet<(Int32 Bx, Int32 By)> RemovedCells = RemovedIndices
                    .Select(Index => EdgeCells[Index].Cell)
                    .ToHashSet();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Remaining = Cells
                    .Where(Entry => !RemovedCells.Contains(Entry.Cell))
                    .ToList();
                const Int32 MAXIMUM_DETOUR_COVERAGE_CANDIDATES = 1;
                Boolean JointFillRecoveryAttempted = false;
                List<CoveragePlanner.PlanResult>
                    DetourCoverageCandidates =
                        new List<CoveragePlanner.PlanResult>();
                foreach (CoveragePlanner.PlanResult Candidate in
                    OrderedCoverageCandidates(
                        Remaining,
                        Grid,
                        BladeM,
                        (Start.X, Start.Y),
                        (FillEntry.X, FillEntry.Y)))
                {
                    if (Candidate.Runs.Count == 0) continue;
                    RunSequencer.Result Sequence = BuildSequence(
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
                                out CoveragePlanner.PlanResult
                                    AlternativeCut,
                                out RunSequencer.Result
                                    AlternativeCutSequence,
                                out CoveragePlanner.PlanResult
                                    AlternativeFill,
                                out RunSequencer.Result
                                    AlternativeFillSequence))
                            {
                                List<((Int32 Bx, Int32 By) Cell,
                                    Double DepthM)> JointOmitted =
                                        new List<((Int32, Int32), Double)>(
                                            AlternativeCut.OmittedCells);
                                JointOmitted.AddRange(RemovedIndices.Select(
                                    Index => EdgeCells[Index]));
                                SelectedPlan =
                                    new CoveragePlanner.PlanResult(
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
                            < MAXIMUM_DETOUR_COVERAGE_CANDIDATES)
                        {
                            DetourCoverageCandidates.Add(Candidate);
                        }
                        continue;
                    }
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Omitted =
                        new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(
                            Candidate.OmittedCells);
                    Omitted.AddRange(RemovedIndices.Select(Index => EdgeCells[Index]));
                    SelectedPlan = new CoveragePlanner.PlanResult(
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
                foreach (CoveragePlanner.PlanResult Candidate in
                    DetourCoverageCandidates)
                {
                    RunSequencer.Result Sequence =
                        RunSequencer.OptimizeFlexibleEndpoints(
                            Candidate.Runs,
                            Start,
                            FillEntry,
                            PlanningPreferredLineupM,
                            PlanningMinimumRadiusM,
                            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
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
                    SelectedPlan = new CoveragePlanner.PlanResult(
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
                OmissionSets >= MAXIMUM_OMISSION_SETS_PER_TIER));
        }
        SelectedPlan = CoveragePlanner.Plan(
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
        // Adds one unique, bounded edge-removal candidate to the search set.
        // CellsToRemove: Edge cells proposed for omission.
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
        foreach (CoveragePlanner.PlanResult Candidate in
            OrderedCoverageCandidates(Cells, Grid, BladeM))
        {
            foreach (CoveragePlanner.Run Run in Candidate.Runs)
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
        ConnectorPlanner.Pose Start,
        Boolean GradeReturn,
        out CoveragePlanner.PlanResult SelectedPlan,
        out RunSequencer.Result SelectedSequence
        )
    {
        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Remaining =
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Cells);
        foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Removed)
        {
            if (!Remaining.Remove(Entry))
            {
                SelectedPlan = CoveragePlanner.Plan(
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(), Grid, BladeM);
                SelectedSequence = BuildSequence(SelectedPlan.Runs, Grid);
                return false;
            }
        }
        foreach (CoveragePlanner.PlanResult Candidate in
            MinimalOmissionCandidates(
                Remaining, Grid, BladeM, (Start.X, Start.Y), null))
        {
            RunSequencer.Result Sequence =
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
            SelectedPlan = new CoveragePlanner.PlanResult(
                Candidate.HeadingDeg,
                Candidate.Runs,
                Candidate.ConnectorDistanceM,
                Candidate.AnchorDistanceM,
                Candidate.Score,
                Omitted);
            SelectedSequence = Sequence;
            return true;
        }
        SelectedPlan = CoveragePlanner.Plan(
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(), Grid, BladeM);
        SelectedSequence = BuildSequence(SelectedPlan.Runs, Grid);
        return false;
    }

    /// <summary>Computes polyline distance for lifecycle geometry summaries.</summary>
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
    /// Alternates exact connector-aware optimization of cut and fill run order and direction.
    /// </summary>
    private static (
        CoveragePlanner.PlanResult CutPlan,
        CoveragePlanner.PlanResult FillPlan) OptimizeCutAndFillSequence
        (
        CoveragePlanner.PlanResult CutPlan,
        CoveragePlanner.PlanResult FillPlan,
        Grid Grid
        )
    {
        List<CoveragePlanner.Run> CutRuns = CutPlan.Runs;
        List<CoveragePlanner.Run> FillRuns = FillPlan.Runs;
        for (Int32 Iteration = 0; Iteration < 2; Iteration += 1)
        {
            ConnectorPlanner.Pose? FillEntry = FillRuns.Count == 0
                ? null
                : StartPose(FillRuns[0], Grid);
            CutRuns = OptimizeRunSequence(CutRuns, null, FillEntry, Grid);
            ConnectorPlanner.Pose? CutExit = CutRuns.Count == 0
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
    private static List<CoveragePlanner.Run> OptimizeRunSequence
        (
        List<CoveragePlanner.Run> Runs,
        ConnectorPlanner.Pose? Start,
        ConnectorPlanner.Pose? End,
        Grid Grid
        )
    {
        RunSequencer.Result Sequence = RunSequencer.Optimize(
            Runs,
            Start,
            End,
            PlanningPreferredLineupM,
            PlanningMinimumRadiusM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid);
        return Sequence.Success
            ? Sequence.Runs.Select(Sequenced => Sequenced.Run).ToList()
            : Runs;
    }

    /// <summary>
    /// Creates the entry pose of a straight coverage run.
    /// </summary>
    private static ConnectorPlanner.Pose StartPose
        (
        CoveragePlanner.Run Run,
        Grid Grid
        )
    {
        RunSequencer.SequencedRun Prepared = RunSequencer.Prepare(
            Run,
            PlanningPreferredLineupM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid);
        return Prepared.EntryPose;
    }

    /// <summary>
    /// Creates the exit pose of a straight coverage run.
    /// </summary>
    private static ConnectorPlanner.Pose EndPose
        (
        CoveragePlanner.Run Run,
        Grid Grid
        )
    {
        RunSequencer.SequencedRun Prepared = RunSequencer.Prepare(
            Run,
            PlanningPreferredLineupM,
            Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
            Grid);
        return Prepared.ExitPose;
    }

    /// <summary>
    /// Rebuilds comparison metrics after connector-aware ordering.
    /// </summary>
    private static CoveragePlanner.PlanResult RebuildPlan
        (
        CoveragePlanner.PlanResult Original,
        List<CoveragePlanner.Run> Runs
        )
    {
        Double ConnectorDistanceM = 0.0;
        for (Int32 Index = 1; Index < Runs.Count; Index += 1)
        {
            ConnectorDistanceM += Distance(Runs[Index - 1].EndXy, Runs[Index].StartXy);
        }
        return new CoveragePlanner.PlanResult(
            Original.HeadingDeg,
            Runs,
            ConnectorDistanceM,
            Original.AnchorDistanceM,
            Original.Score,
            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Original.OmittedCells));
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
        Double Delta = Math.Abs((Second - First) * Constants.DEGREES_PER_HALF_TURN / Math.PI);
        return Delta > Constants.DEGREES_PER_HALF_TURN ? Constants.DEGREES_PER_TURN - Delta : Delta;
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
        Double HeadingDeg = Math.Atan2(B.X - A.X, B.Y - A.Y) * Constants.DEGREES_PER_HALF_TURN / Math.PI;
        return (HeadingDeg + Constants.DEGREES_PER_TURN) % Constants.DEGREES_PER_TURN;
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
}
