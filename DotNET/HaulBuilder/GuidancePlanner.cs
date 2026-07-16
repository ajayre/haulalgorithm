using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Gif;
using SixLabors.ImageSharp.PixelFormats;

namespace HaulBuilder
{
/// <summary>
/// Builds production coverage, forward-only connectors, lifecycle guidance, and diagnostics.
/// </summary>
public static class GuidancePlanner
{
    private const Int32 FIRST_ROUTE_INDEX = 7;
    private const Int32 LAST_ROUTE_INDEX = 9;
    private const Double SHARP_TURN_DEG = 45.0;
    private const Double REVERSAL_DEG = 100.0;
    private const Int32 MAX_PAIRED_CANDIDATE_ATTEMPTS = 72 * 72;
    private const Int32 MAX_BACKTRACK_CANDIDATE_ATTEMPTS = 250000;
    private const Int32 MAX_INITIAL_CUT_CANDIDATE_ATTEMPTS = 6;
    private const Double FAST_OPERATING_SPEED_SCALE = 1.1;
    private const Double FAST_ACCELERATION_SCALE = 2.0;
    private const Double HIGH_OPERATIONAL_AVAILABILITY = 0.9;
    private const Double LOW_OPERATIONAL_AVAILABILITY = 0.7;
    private const Double MEDIAN_PERCENTILE = 0.5;
    private const Double PERCENTILE_90 = 0.9;
    private const Double PERCENTILE_99 = 0.99;
    private const Double SECONDS_PER_HOUR = 3600.0;
    private const Double SLOW_OPERATING_SPEED_SCALE = 0.9;
    private const Double SLOW_ACCELERATION_SCALE = 0.5;
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
        ?? Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS;

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
            GuidanceTime SummaryTime,
            IReadOnlyList<LifecyclePlanner.PassPlan> Passes
            )
        {
            this.Lifecycle = Lifecycle;
            this.Timing = Timing;
            this.Guidance = Guidance;
            this.SummaryTime = SummaryTime;
            this.Passes = Passes;
        }

        /// <summary>Gets the final connected forward-only lifecycle.</summary>
        public LifecyclePlanner.Result Lifecycle { get; }

        /// <summary>Gets the detailed final-geometry timing result.</summary>
        public GuidanceTiming.Result Timing { get; }

        /// <summary>Gets the operator-facing waypoint adapter.</summary>
        public GuidanceResult Guidance { get; }

        /// <summary>Gets the legacy-shaped timing summary backed by detailed timing.</summary>
        public GuidanceTime SummaryTime { get; }

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
        Double BladeM = Options.BladeWidthMm / 1000.0;
        List<String> PlanningDiagnostics = new List<String>();
        void ReportDiagnostic(String Message)
        {
            PlanningDiagnostics.Add(Message);
            Diagnostic?.Invoke(Message);
        }
        GeometrySettings? PreviousGeometry = CurrentGeometry.Value;
        List<LifecyclePlanner.PassPlan> Passes;
        CurrentGeometry.Value = new GeometrySettings(
            Options.TurningCircleFt * 0.5 * Constants.FEET_TO_METERS,
            Options.EquipmentLengthFt * Constants.FEET_TO_METERS);
        try
        {
            Passes = BuildWholePlanPasses(
                Route,
                Grid,
                BladeM,
                !Options.NoGradeReturn,
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
            !Options.NoGradeReturn,
            Options.EquipmentLengthFt * Constants.FEET_TO_METERS,
            Options.TurningCircleFt * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Grid.GridSizeM),
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
        GuidanceTime SummaryTime = BuildSummaryTime(Timing);
        return new BuildResult(Lifecycle, Timing, Guidance, SummaryTime, Passes);
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
            if (SegmentM <= 1e-9) return;
            if (HasPreviousPoint)
            {
                Double ChangeDeg = HeadingChangeDeg(
                    PreviousPoint, PendingPoint, Point);
                if (ChangeDeg >= Options.TurnThresholdDeg)
                {
                    TurnCount += 1;
                    TurnAngleRad += ChangeDeg * Math.PI / 180.0;
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
                    1000.0 * CutDepthM[DepthIndex],
                LifecyclePlanner.Phase.Fill when FillDepthM.Length > 0 =>
                    1000.0 * FillDepthM[DepthIndex],
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
            Options.BladeWidthMm / 1000.0,
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

    /// <summary>Maps detailed timing to the established summary object.</summary>
    private static GuidanceTime BuildSummaryTime
        (
        GuidanceTiming.Result Timing
        )
    {
        Double LoadingS = PhaseSeconds(Timing, LifecyclePlanner.Phase.Cut);
        Double DumpingS = PhaseSeconds(Timing, LifecyclePlanner.Phase.Fill);
        Double GradingS = PhaseSeconds(Timing, LifecyclePlanner.Phase.Grade);
        Double HaulingS = Math.Max(
            0.0, Timing.LoadedSeconds - LoadingS - DumpingS);
        Double ReturningS = Math.Max(
            0.0, Timing.UnloadedSeconds - GradingS);
        return new GuidanceTime(
            Timing.TotalSeconds,
            LoadingS,
            DumpingS,
            GradingS,
            HaulingS,
            ReturningS,
            0.0,
            Timing.EventSeconds + Timing.FieldEfficiencyAllowanceSeconds,
            Timing.CurvatureConstraintCount);
    }

    /// <summary>Gets one phase's timing ledger value.</summary>
    private static Double PhaseSeconds
        (
        GuidanceTiming.Result Timing,
        LifecyclePlanner.Phase Phase
        )
    {
        return Timing.PhaseSeconds.TryGetValue(Phase, out Double Seconds)
            ? Seconds
            : 0.0;
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
            LifecyclePlanner.Phase.ManualAssist => "MOVE",
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
    /// Builds TheShop2 once, reports route 8-10 metrics, and writes guidance PNGs.
    /// </summary>
    /// <param name="Args">Optional output directory and AGD path.</param>
    /// <returns>Zero when every guidance plan conserves allocation.</returns>
    static public Int32 Run
        (
        String[] Args
        )
    {
        String RepoRoot = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String OutputDirectory = (Args.Length > 0)
            ? Path.GetFullPath(Args[0])
            : Path.Combine(RepoRoot, "assets", "theshop2_guidance");
        String AgdPath = (Args.Length > 1)
            ? Path.GetFullPath(Args[1])
            : Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");
        Directory.CreateDirectory(OutputDirectory);

        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg, EdgePatchOmissionResult EdgeOmission) Plan = BuildRoute(AgdPath);
        Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0;
        Boolean Conserved = true;
        List<LifecyclePlanner.PassPlan> LifecyclePasses =
            new List<LifecyclePlanner.PassPlan>();

        for (Int32 RouteIndex = FIRST_ROUTE_INDEX; RouteIndex <= LAST_ROUTE_INDEX; RouteIndex += 1)
        {
            Pass Pass = Plan.Route.OrderedPasses[RouteIndex];
            CoveragePlanner.PlanResult CutPlan = CoveragePlanner.Plan(
                Pass.CutCells,
                Plan.Grid,
                BladeM,
                null,
                Pass.FillStartXy);
            (Double X, Double Y)? FillStart = CutPlan.Runs.Count == 0
                ? null
                : CutPlan.Runs[CutPlan.Runs.Count - 1].EndXy;
            CoveragePlanner.PlanResult FillPlan = CoveragePlanner.Plan(
                Pass.FillCells,
                Plan.Grid,
                BladeM,
                FillStart,
                null);
            (CutPlan, FillPlan) = OptimizeCutAndFillSequence(CutPlan, FillPlan, Plan.Grid);
            RunSequencer.Result CutSequence = BuildSequence(CutPlan.Runs, Plan.Grid);
            RunSequencer.Result FillSequence = BuildSequence(FillPlan.Runs, Plan.Grid);
            LifecyclePasses.Add(new LifecyclePlanner.PassPlan(
                CutSequence,
                FillSequence,
                CutPlan.OmittedCells,
                FillPlan.OmittedCells));

            Boolean CutConserved = CoveragePlanner.ConservesAllocation(
                Pass.CutCells, CutPlan.AccountedCells);
            Boolean FillConserved = CoveragePlanner.ConservesAllocation(
                Pass.FillCells, FillPlan.AccountedCells);
            Conserved = Conserved && CutConserved && FillConserved;

            (Int32 Sharp, Int32 Reversals, Double DistanceM) CutBaseline =
                CellPathMetrics(Pass.CutCells, Plan.Grid);
            (Int32 Sharp, Int32 Reversals, Double DistanceM) FillBaseline =
                CellPathMetrics(Pass.FillCells, Plan.Grid);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "route={0} cut cells={1} baseline_sharp={2} baseline_reverse={3} baseline_m={4:F1} guidance_runs={5} connector_m={6:F1} heading={7:F0} max_offset_m={8:F2} accounted={9} omitted_cells={10}",
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
                "route={0} fill cells={1} baseline_sharp={2} baseline_reverse={3} baseline_m={4:F1} guidance_runs={5} connector_m={6:F1} heading={7:F0} max_offset_m={8:F2} accounted={9} omitted_cells={10}",
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
        List<LifecyclePlanner.PassPlan> WholePlanGradeOff =
            BuildWholePlanPasses(Plan.Route, Plan.Grid, BladeM, false);
        ReportLifecycle("whole_plan", WholePlanGradeOff, false, Plan.Grid);
        List<LifecyclePlanner.PassPlan> WholePlanGradeOn =
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
                "theshop2_guidance",
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

        TransitPlanner.ResetCache();
        Stopwatch RunTimer = Stopwatch.StartNew();
        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg,
            EdgePatchOmissionResult EdgeOmission) Plan = default;
        List<LifecyclePlanner.PassPlan> Passes;
        LifecyclePlanner.Result Lifecycle;
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
            Lifecycle = LifecyclePlanner.Build(
                Passes,
                true,
                PlanningPreferredLineupM,
                PlanningMinimumRadiusM,
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

        GuidanceTiming.Options TimingOptions =
            new GuidanceTiming.Options
            {
                FieldEfficiency = Constants.DEFAULT_FIELD_EFFICIENCY,
            };
        GuidanceTiming.Result Timing =
            GuidanceTiming.Estimate(Lifecycle, TimingOptions);
        // Run every sensitivity against identical final geometry so only the
        // named physical or operational assumption can move the result.
        GuidanceTiming.Result SlowerTiming =
            GuidanceTiming.Estimate(
                Lifecycle, ScaleTimingOptions(
                    TimingOptions,
                    SLOW_OPERATING_SPEED_SCALE,
                    SLOW_OPERATING_SPEED_SCALE));
        GuidanceTiming.Result FasterTiming =
            GuidanceTiming.Estimate(
                Lifecycle, ScaleTimingOptions(
                    TimingOptions,
                    FAST_OPERATING_SPEED_SCALE,
                    FAST_OPERATING_SPEED_SCALE));
        GuidanceTiming.Result SlowerAccelerationTiming =
            GuidanceTiming.Estimate(
                Lifecycle, ScaleTimingOptions(
                    TimingOptions, 1.0, SLOW_ACCELERATION_SCALE));
        GuidanceTiming.Result FasterAccelerationTiming =
            GuidanceTiming.Estimate(
                Lifecycle, ScaleTimingOptions(
                    TimingOptions, 1.0, FAST_ACCELERATION_SCALE));
        FieldEdgePolicy EdgePolicy =
            new FieldEdgePolicy(Plan.Grid, Plan.Grid.GridSizeM);
        List<(Int32 SegmentIndex, LifecyclePlanner.Segment Segment,
            Double DistanceM, Boolean Exterior, Boolean BeyondAllowance,
            LifecyclePlanner.Phase Previous,
            LifecyclePlanner.Phase Next)> Manual =
                new List<(Int32, LifecyclePlanner.Segment, Double,
                    Boolean, Boolean, LifecyclePlanner.Phase,
                    LifecyclePlanner.Phase)>();
        for (Int32 SegmentIndex = 0;
            SegmentIndex < Lifecycle.Segments.Count;
            SegmentIndex += 1)
        {
            LifecyclePlanner.Segment Segment =
                Lifecycle.Segments[SegmentIndex];
            if (Segment.Phase != LifecyclePlanner.Phase.ManualAssist) continue;
            LifecyclePlanner.Phase Previous = SegmentIndex > 0
                ? Lifecycle.Segments[SegmentIndex - 1].Phase
                : LifecyclePlanner.Phase.Start;
            LifecyclePlanner.Phase Next =
                SegmentIndex + 1 < Lifecycle.Segments.Count
                    ? Lifecycle.Segments[SegmentIndex + 1].Phase
                    : LifecyclePlanner.Phase.End;
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
        Int32 OmittedAllocations = Passes.Sum(Pass =>
            Pass.CutOmittedCellCount + Pass.FillOmittedCellCount);
        HashSet<(Int32 Bx, Int32 By)> GuidanceOmittedCells = Passes
            .SelectMany(Pass => Pass.CutOmittedCells.Concat(Pass.FillOmittedCells))
            .Select(Entry => Entry.Cell)
            .ToHashSet();
        HashSet<(Int32 Bx, Int32 By)> EdgePatchAffectedCells = Plan.EdgeOmission
            .OmittedCutCells.Concat(Plan.EdgeOmission.OmittedFillCells)
            .Select(Entry => Entry.Cell)
            .ToHashSet();
        HashSet<(Int32 Bx, Int32 By)> AllAffectedCells =
            new HashSet<(Int32 Bx, Int32 By)>(GuidanceOmittedCells);
        AllAffectedCells.UnionWith(EdgePatchAffectedCells);
        Int32 ValidFieldCells = 0;
        Int32 ActiveWorkCells = 0;
        Double[,] Delta = Plan.Grid.Delta();
        Double WorkToleranceM = Constants.DEFAULT_TOLERANCE_MM / 1000.0;
        for (Int32 By = 0; By < Plan.Grid.Height; By += 1)
        {
            for (Int32 Bx = 0; Bx < Plan.Grid.Width; Bx += 1)
            {
                if (!Plan.Grid.Valid[By, Bx]) continue;
                ValidFieldCells += 1;
                if (Math.Abs(Delta[By, Bx]) > WorkToleranceM)
                {
                    ActiveWorkCells += 1;
                }
            }
        }
        Double StraightCoverageShortfallM = Passes
            .SelectMany(Pass => Pass.Cut.Runs.Concat(Pass.Fill.Runs))
            .Sum(Run => Run.StartCoverageShortfallM + Run.EndCoverageShortfallM);
        (Int64 Hits, Int64 Misses, Int32 Entries,
            Int32 RetainedEntries, Int64 Clears) TransitCache =
            TransitPlanner.GetCacheStatistics();
        (Int64 Calls, Int64 Successes, Int64 ExpansionLimitHits,
            Int64 Expansions, Int64 MaximumSuccessExpansions,
            Double ElapsedSeconds) ForwardDetours =
            TransitPlanner.GetForwardDetourStatistics();

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
            "total_segments={0} total_distance_m={1:F1} total_distance_mi={2:F2} deterministic_drive_hours={3:F3} scheduled_hours_at_80_percent_operational_availability={4:F3}",
            Lifecycle.Segments.Count,
            Lifecycle.DistanceM,
            Lifecycle.DistanceM / 1609.344,
            (Timing.MotionSeconds + Timing.EventSeconds) / SECONDS_PER_HOUR,
            Timing.TotalSeconds / SECONDS_PER_HOUR));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "timing_distance_m={0:F3} distance_reconciliation_m={1:F6} constant_limit_hours={2:F3} acceleration_and_speed_transition_overhead_hours={3:F3} explicit_event_hours={4:F3} operational_availability_allowance_hours={5:F3}",
            Timing.DistanceM,
            Timing.DistanceM - Lifecycle.DistanceM,
            Timing.ConstantSpeedSeconds / SECONDS_PER_HOUR,
            Timing.AccelerationConstraintSeconds / SECONDS_PER_HOUR,
            Timing.EventSeconds / SECONDS_PER_HOUR,
            Timing.FieldEfficiencyAllowanceSeconds / SECONDS_PER_HOUR));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "timing_motion_blocks={0} curvature_vertices={1} curvature_limited_m={2:F1} curvature_limited_percent={3:F3} forced_geometry_stops={4} minimum_curve_radius_m={5:F3} operator_rounded_tight_vertices={6}",
            Timing.MotionBlockCount,
            Timing.CurvatureConstraintCount,
            Timing.CurvatureLimitedDistanceM,
            Timing.DistanceM > 0.0
                ? 100.0 * Timing.CurvatureLimitedDistanceM / Timing.DistanceM
                : 0.0,
            Timing.ForcedStopCount,
            Timing.MinimumObservedCurveRadiusM,
            Timing.BelowMinimumRadiusVertexCount));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "timing_assumptions_mph cut={0:F2} fill={1:F2} grade={2:F2} loaded_full={3:F2} unloaded_full={4:F2} maximum_steering={5:F2}",
            TimingOptions.CutSpeedMph,
            TimingOptions.FillSpeedMph,
            TimingOptions.GradeSpeedMph,
            TimingOptions.LoadedTravelSpeedMph,
            TimingOptions.UnloadedTravelSpeedMph,
            TimingOptions.TurnSpeedMph));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "timing_dynamics acceleration_mps2={0:F6} deceleration_mps2={1:F6} turn_speed_rule=linear_heading_change minimum_turn_radius_m={2:F4} curvature_sample_spacing_m={3:F3} radius_tolerance_fraction={4:F3} schedule_operational_availability={5:F3}",
            TimingOptions.AccelerationMps2,
            TimingOptions.DecelerationMps2,
            TimingOptions.MinimumTurningRadiusM,
            TimingOptions.CurvatureEvaluationSpacingM,
            TimingOptions.MinimumRadiusToleranceFraction,
            TimingOptions.FieldEfficiency));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "timing_by_payload loaded_distance_m={0:F1} loaded_hours={1:F3} unloaded_distance_m={2:F1} unloaded_hours={3:F3}",
            Timing.LoadedDistanceM,
            Timing.LoadedSeconds / SECONDS_PER_HOUR,
            Timing.UnloadedDistanceM,
            Timing.UnloadedSeconds / SECONDS_PER_HOUR));
        Report.AppendLine("timing_sensitivity:");
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "  all_operating_speeds_minus_10_percent: drive_hours={0:F3} scheduled_hours_at_baseline_availability={1:F3}",
            (SlowerTiming.MotionSeconds + SlowerTiming.EventSeconds) / SECONDS_PER_HOUR,
            SlowerTiming.TotalSeconds / SECONDS_PER_HOUR));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "  all_operating_speeds_plus_10_percent: drive_hours={0:F3} scheduled_hours_at_baseline_availability={1:F3}",
            (FasterTiming.MotionSeconds + FasterTiming.EventSeconds) / SECONDS_PER_HOUR,
            FasterTiming.TotalSeconds / SECONDS_PER_HOUR));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "  longitudinal_acceleration_half: drive_hours={0:F3} delta_hours={1:F3}",
            (SlowerAccelerationTiming.MotionSeconds
                + SlowerAccelerationTiming.EventSeconds) / SECONDS_PER_HOUR,
            (SlowerAccelerationTiming.TotalSeconds - Timing.TotalSeconds)
                / SECONDS_PER_HOUR));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "  longitudinal_acceleration_double: drive_hours={0:F3} delta_hours={1:F3}",
            (FasterAccelerationTiming.MotionSeconds
                + FasterAccelerationTiming.EventSeconds) / SECONDS_PER_HOUR,
            (FasterAccelerationTiming.TotalSeconds - Timing.TotalSeconds)
                / SECONDS_PER_HOUR));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "  schedule_operational_availability_70_percent_hours={0:F3} schedule_operational_availability_80_percent_hours={1:F3} schedule_operational_availability_90_percent_hours={2:F3}",
            (Timing.MotionSeconds + Timing.EventSeconds)
                / (SECONDS_PER_HOUR * LOW_OPERATIONAL_AVAILABILITY),
            (Timing.MotionSeconds + Timing.EventSeconds)
                / (SECONDS_PER_HOUR * TimingOptions.FieldEfficiency),
            (Timing.MotionSeconds + Timing.EventSeconds)
                / (SECONDS_PER_HOUR * HIGH_OPERATIONAL_AVAILABILITY)));
        Report.AppendLine("timing_by_phase:");
        foreach (LifecyclePlanner.Phase Phase in
            Timing.PhaseDistanceM.Keys.Union(Timing.PhaseSeconds.Keys).OrderBy(Phase => Phase))
        {
            Timing.PhaseDistanceM.TryGetValue(Phase, out Double PhaseDistanceM);
            Timing.PhaseSeconds.TryGetValue(Phase, out Double PhaseSeconds);
            Report.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "  {0}: distance_m={1:F1} distance_mi={2:F3} hours={3:F4} average_mph={4:F3}",
                Phase,
                PhaseDistanceM,
                PhaseDistanceM / 1609.344,
                PhaseSeconds / SECONDS_PER_HOUR,
                PhaseSeconds > 0.0
                    ? PhaseDistanceM / PhaseSeconds / Constants.MPH_TO_MPS
                    : 0.0));
        }
        Report.AppendLine("timing_by_route:");
        // Retain the full route ledger and a compact distribution so expensive
        // outliers remain visible without being hidden inside the field total.
        List<Double> RouteDriveHours = Timing.RouteSeconds
            .Where(Item => Item.Key > 0)
            .Select(Item => Item.Value / SECONDS_PER_HOUR)
            .OrderBy(Hours => Hours)
            .ToList();
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "  summary: min_hours={0:F4} median_hours={1:F4} p90_hours={2:F4} p99_hours={3:F4} max_hours={4:F4}",
            Percentile(RouteDriveHours, 0.0),
            Percentile(RouteDriveHours, MEDIAN_PERCENTILE),
            Percentile(RouteDriveHours, PERCENTILE_90),
            Percentile(RouteDriveHours, PERCENTILE_99),
            Percentile(RouteDriveHours, 1.0)));
        foreach (Int32 RouteNumber in
            Timing.RouteDistanceM.Keys.Union(Timing.RouteSeconds.Keys).OrderBy(Value => Value))
        {
            Timing.RouteDistanceM.TryGetValue(RouteNumber, out Double RouteDistanceM);
            Timing.RouteSeconds.TryGetValue(RouteNumber, out Double RouteSeconds);
            Report.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "  route={0} distance_m={1:F1} distance_mi={2:F3} drive_hours={3:F4} average_mph={4:F3}",
                RouteNumber,
                RouteDistanceM,
                RouteDistanceM / 1609.344,
                RouteSeconds / SECONDS_PER_HOUR,
                RouteSeconds > 0.0
                    ? RouteDistanceM / RouteSeconds / Constants.MPH_TO_MPS
                    : 0.0));
        }
        Report.AppendLine("longest_routes_by_drive_time:");
        foreach (KeyValuePair<Int32, Double> RouteTime in Timing.RouteSeconds
            .Where(Item => Item.Key > 0)
            .OrderByDescending(Item => Item.Value)
            .Take(20))
        {
            Timing.RouteDistanceM.TryGetValue(
                RouteTime.Key, out Double RouteDistanceM);
            Report.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "  route={0} drive_hours={1:F4} distance_m={2:F1} average_mph={3:F3} rounded_tight_vertices={4}",
                RouteTime.Key,
                RouteTime.Value / SECONDS_PER_HOUR,
                RouteDistanceM,
                RouteTime.Value > 0.0
                    ? RouteDistanceM / RouteTime.Value / Constants.MPH_TO_MPS
                    : 0.0,
                Timing.CurveViolations.Count(Violation =>
                    Violation.RouteNumber == RouteTime.Key)));
        }
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "max_segment_gap_m={0:F6} segments_beyond_one_cell_allowance={1}",
            MaximumGapM,
            BeyondAllowanceSegments));
        Report.AppendLine("operator_rounded_tight_vertices:");
        foreach (GuidanceTiming.CurveViolation Violation in
            Timing.CurveViolations.OrderBy(Violation => Violation.RadiusM).Take(50))
        {
            Report.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "  route={0} {1}->{2} radius_m={3:F3} radius_ft={4:F2} previous=({5:F2},{6:F2}) point=({7:F2},{8:F2}) next=({9:F2},{10:F2})",
                Violation.RouteNumber,
                Violation.PreviousPhase,
                Violation.NextPhase,
                Violation.RadiusM,
                Violation.RadiusM / Constants.FEET_TO_METERS,
                Violation.PreviousPoint.X,
                Violation.PreviousPoint.Y,
                Violation.X,
                Violation.Y,
                Violation.NextPoint.X,
                Violation.NextPoint.Y));
        }
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "guidance_omitted_allocations={0} guidance_omitted_unique_cells={1} guidance_affected_area_m2={2:F1} guidance_percent_field={3:F3} guidance_percent_active_work={4:F3}",
            OmittedAllocations,
            GuidanceOmittedCells.Count,
            GuidanceOmittedCells.Count * Plan.Grid.AreaM2,
            ValidFieldCells > 0
                ? 100.0 * GuidanceOmittedCells.Count / ValidFieldCells
                : 0.0,
            ActiveWorkCells > 0
                ? 100.0 * GuidanceOmittedCells.Count / ActiveWorkCells
                : 0.0));
        Report.AppendLine(String.Format(
            CultureInfo.InvariantCulture,
            "valid_field_cells={0} active_work_cells={1} all_omission_affected_unique_cells={2} all_omission_percent_field={3:F3} nominal_straight_coverage_shortfall_m={4:F1}",
            ValidFieldCells,
            ActiveWorkCells,
            AllAffectedCells.Count,
            ValidFieldCells > 0
                ? 100.0 * AllAffectedCells.Count / ValidFieldCells
                : 0.0,
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
        foreach (IGrouping<(LifecyclePlanner.Phase Previous,
            LifecyclePlanner.Phase Next),
            (Int32 SegmentIndex, LifecyclePlanner.Segment Segment,
                Double DistanceM, Boolean Exterior, Boolean BeyondAllowance,
                LifecyclePlanner.Phase Previous,
                LifecyclePlanner.Phase Next)> Group in Manual
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
        foreach ((Int32 SegmentIndex, LifecyclePlanner.Segment Segment,
            Double DistanceM, Boolean Exterior, Boolean BeyondAllowance,
            LifecyclePlanner.Phase Previous,
            LifecyclePlanner.Phase Next) Item in Manual
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
                "theshop2_guidance",
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
        CoveragePlanner.PlanResult?[] Cuts =
            new CoveragePlanner.PlanResult?[PassCount];
        CoveragePlanner.PlanResult?[] Fills =
            new CoveragePlanner.PlanResult?[PassCount];
        Parallel.For(0, PassCount, PassIndex =>
        {
            Pass Pass = Plan.Route.OrderedPasses[PassIndex];
            CoveragePlanner.PlanResult Cut =
                CoveragePlanner.Plan(
                    Pass.CutCells, Plan.Grid, BladeM, null, Pass.FillStartXy);
            (Double X, Double Y)? FillAnchor = Cut.Runs.Count == 0
                ? null
                : Cut.Runs[Cut.Runs.Count - 1].EndXy;
            Cuts[PassIndex] = Cut;
            Fills[PassIndex] = CoveragePlanner.Plan(
                Pass.FillCells, Plan.Grid, BladeM, FillAnchor, null);
        });

        StringBuilder Csv = new StringBuilder();
        Csv.AppendLine("pass,cut_cells,fill_cells,cut_runs,fill_runs,cut_omitted,fill_omitted,max_runs");
        List<Int32> MaximumRunCounts = new List<Int32>(PassCount);
        for (Int32 PassIndex = 0; PassIndex < PassCount; PassIndex += 1)
        {
            Pass Pass = Plan.Route.OrderedPasses[PassIndex];
            CoveragePlanner.PlanResult Cut = Cuts[PassIndex]
                ?? throw new InvalidOperationException("Coverage inventory omitted cut work.");
            CoveragePlanner.PlanResult Fill = Fills[PassIndex]
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
        ConnectorPlanner.Pose Start =
            new ConnectorPlanner.Pose(
                Values[0], Values[1], Values[2]);
        ConnectorPlanner.Pose End =
            new ConnectorPlanner.Pose(
                Values[3], Values[4], Values[5]);
        Double RadiusM = PlanningMinimumRadiusM;
        Double SampleSpacingM = Math.Min(0.5, Plan.Grid.GridSizeM);
        foreach (Boolean AllowForwardDetour in new[] { false, true })
        {
            Stopwatch Timer = Stopwatch.StartNew();
            TransitPlanner.Result Result =
                TransitPlanner.Connect(
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
        return TransitPlanner.Connect(
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
                "theshop2_guidance",
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
        List<LifecyclePlanner.PassPlan> AllPasses =
            BuildWholePlanPasses(
                Plan.Route, Plan.Grid, BladeM, true, LastRouteNumber);
        List<LifecyclePlanner.PassPlan> Passes = AllPasses
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
        LifecyclePlanner.Result Lifecycle = LifecyclePlanner.Build(
            Passes,
            true,
            PlanningPreferredLineupM,
            PlanningMinimumRadiusM,
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
            .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.Cut)
            .Select(Segment => (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
            .ToList();
        List<IReadOnlyList<(Double X, Double Y)>> FillRuns = Lifecycle.Segments
            .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.Fill)
            .Select(Segment => (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
            .ToList();
        List<(Double X, Double Y)> RoutePoints = Lifecycle.Segments
            .SelectMany(Segment => Segment.Points)
            .ToList();
        ReportLifecycleGeometry(Lifecycle, Plan.Grid, BladeM);
        for (Int32 SegmentIndex = 0; SegmentIndex < Lifecycle.Segments.Count; SegmentIndex += 1)
        {
            LifecyclePlanner.Segment Segment = Lifecycle.Segments[SegmentIndex];
            Double SegmentDistanceM = PolylineDistance(Segment.Points);
            if ((Segment.Phase != LifecyclePlanner.Phase.Fill)
                && (Segment.Phase != LifecyclePlanner.Phase.Grade))
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
            .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.ManualAssist)
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
            LifecyclePlanner.Segment Segment = Lifecycle.Segments[SegmentIndex];
            if (Segment.Phase != LifecyclePlanner.Phase.ManualAssist) continue;
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
                SegmentIndex > 0 ? Lifecycle.Segments[SegmentIndex - 1].Phase : LifecyclePlanner.Phase.Start,
                SegmentIndex + 1 < Lifecycle.Segments.Count ? Lifecycle.Segments[SegmentIndex + 1].Phase : LifecyclePlanner.Phase.End,
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
                RunSequencer.SequencedRun First = Pass.Cut.Runs[0];
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
        CoveragePlanner.PlanResult CutPlan =
            CoveragePlanner.Plan(
                Pass.CutCells, Grid, BladeM, null, Pass.FillStartXy);
        (Double X, Double Y)? FillStart = CutPlan.Runs.Count == 0
            ? null
            : CutPlan.Runs[CutPlan.Runs.Count - 1].EndXy;
        CoveragePlanner.PlanResult FillPlan =
            CoveragePlanner.Plan(
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
            CoveragePlanner.Run First = CutPlan.Runs[0];
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

    /// <summary>Writes a progressive animated GIF of one connected production lifecycle.</summary>
    private static void WriteLifecycleGif
        (
        String GifPath,
        LifecyclePlanner.Result Lifecycle,
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
        LifecyclePlanner.Result Lifecycle,
        Double TargetM
        )
    {
        List<(Double X, Double Y)> Route = new List<(Double X, Double Y)>();
        List<IReadOnlyList<(Double X, Double Y)>> ManualRuns =
            new List<IReadOnlyList<(Double X, Double Y)>>();
        Double RemainingM = Math.Max(0.0, TargetM);
        foreach (LifecyclePlanner.Segment Segment in Lifecycle.Segments)
        {
            if (Segment.Points.Count == 0) continue;
            List<(Double X, Double Y)>? Manual = Segment.Phase
                == LifecyclePlanner.Phase.ManualAssist
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
        LifecyclePlanner.Result Lifecycle,
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
        FieldEdgePolicy EdgePolicy =
            new FieldEdgePolicy(Grid, EdgeAllowanceM);

        for (Int32 SegmentIndex = 0; SegmentIndex < Lifecycle.Segments.Count; SegmentIndex += 1)
        {
            LifecyclePlanner.Segment Segment = Lifecycle.Segments[SegmentIndex];
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

        LifecyclePlanner.Segment? Start = Lifecycle.Segments
            .FirstOrDefault(Segment => Segment.Phase == LifecyclePlanner.Phase.Start);
        LifecyclePlanner.Segment? End = Lifecycle.Segments
            .LastOrDefault(Segment => Segment.Phase == LifecyclePlanner.Phase.End);
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
        FieldEdgePolicy EdgePolicy
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
                    Math.Min(0.5, Grid.GridSizeM),
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
                Math.Min(0.5, Grid.GridSizeM),
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
                RunSequencer.Result ConnectedFillSequence =
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
                            LastFill.Run.HeadingDeg + 180.0);
                    TransitPlanner.Result GradeTurn =
                        TransitPlanner.Connect(
                            LastFill.ExitPose,
                            GradeEntry,
                            PlanningMinimumRadiusM,
                            Math.Min(0.5, Grid.GridSizeM),
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
                                Math.Min(0.5, Grid.GridSizeM),
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
                    Math.Min(0.5, Grid.GridSizeM),
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
                Math.Min(0.5, Grid.GridSizeM),
                Grid);
        return LifecyclePlanner.SupportsPreparedGradeReturn(
            Candidate,
            PlanningMinimumRadiusM,
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
        ConnectorPlanner.Pose? Previous,
        RunSequencer.Result Cut,
        RunSequencer.Result Fill,
        Boolean GradeReturn,
        Grid Grid
        )
    {
        Double RadiusM = PlanningMinimumRadiusM;
        Double SampleM = Math.Min(0.5, Grid.GridSizeM);
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
            Math.Min(0.5, Grid.GridSizeM),
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
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
    }

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
        if ((Recovered.Runs[0].StartCoverageShortfallM > 1e-9)
            || (Recovered.Runs[Recovered.Runs.Count - 1]
                .EndCoverageShortfallM > 1e-9))
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
            Math.Min(0.5, Grid.GridSizeM),
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
            Math.Min(0.5, Grid.GridSizeM),
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
        Double SampleSpacingM = Math.Min(0.5, Grid.GridSizeM);
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
            Run.HeadingDeg + 180.0,
            Run.MaximumLateralErrorM);
    }

    /// <summary>Returns the same position facing in the opposite direction.</summary>
    private static ConnectorPlanner.Pose ReversePose
        (
        ConnectorPlanner.Pose Pose
        )
    {
        return new ConnectorPlanner.Pose(
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
        const Int32 MaximumCompactEdgeRunCells = 2;
        List<CoveragePlanner.Run> CompactEndpointEdgeRuns =
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
                OmissionSets >= MaximumOmissionSetsPerTier));
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

    /// <summary>
    /// Reports grade-off or grade-on post-fill lifecycle distance and final-geometry time.
    /// </summary>
    private static void ReportLifecycle
        (
        String Scope,
        List<LifecyclePlanner.PassPlan> Passes,
        Boolean GradeReturn,
        Grid Grid
        )
    {
        LifecyclePlanner.Result Lifecycle = LifecyclePlanner.Build(
            Passes,
            GradeReturn,
            PlanningPreferredLineupM,
            PlanningMinimumRadiusM,
            Math.Min(0.5, Grid.GridSizeM),
            Grid);
        GuidanceTiming.Result Timing = GuidanceTiming.Estimate(
            Lifecycle,
            new GuidanceTiming.Options
            {
                FieldEfficiency = Constants.DEFAULT_FIELD_EFFICIENCY,
            });
        Double ManualAssistDistanceM = Lifecycle.Segments
            .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.ManualAssist)
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

    /// <summary>Copies timing options while applying explicit sensitivity scales.</summary>
    private static GuidanceTiming.Options ScaleTimingOptions
        (
        GuidanceTiming.Options Source,
        Double SpeedScale,
        Double AccelerationScale
        )
    {
        return new GuidanceTiming.Options
        {
            CutSpeedMph = Source.CutSpeedMph * SpeedScale,
            FillSpeedMph = Source.FillSpeedMph * SpeedScale,
            GradeSpeedMph = Source.GradeSpeedMph * SpeedScale,
            LoadedTravelSpeedMph = Source.LoadedTravelSpeedMph * SpeedScale,
            UnloadedTravelSpeedMph = Source.UnloadedTravelSpeedMph * SpeedScale,
            TurnSpeedMph = Source.TurnSpeedMph * SpeedScale,
            MinimumTurningRadiusM = Source.MinimumTurningRadiusM,
            CurvatureEvaluationSpacingM = Source.CurvatureEvaluationSpacingM,
            MinimumRadiusToleranceFraction = Source.MinimumRadiusToleranceFraction,
            AccelerationMps2 = Source.AccelerationMps2 * AccelerationScale,
            DecelerationMps2 = Source.DecelerationMps2 * AccelerationScale,
            ManualAssistOverheadS = Source.ManualAssistOverheadS,
            BladeLowerS = Source.BladeLowerS,
            BladeRaiseS = Source.BladeRaiseS,
            FieldEfficiency = Source.FieldEfficiency,
        };
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
        List<LifecyclePlanner.Segment> Segments,
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
            Math.Min(0.5, Grid.GridSizeM),
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
            Math.Min(0.5, Grid.GridSizeM),
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
            Math.Min(0.5, Grid.GridSizeM),
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
    /// Converts straight coverage runs into a diagnostic-only cut/haul/fill polyline.
    /// </summary>
    private static DisplayTrajectory BuildDisplayGuidance
        (
        CoveragePlanner.PlanResult CutPlan,
        CoveragePlanner.PlanResult FillPlan,
        Grid Grid
        )
    {
        List<DisplayPoint> Display = new List<DisplayPoint>();
        Int32 ConnectorCount = 0;
        Int32 SuccessfulConnectorCount = 0;
        Int32 OperatorAlignmentCount = 0;
        Double ConnectorDistanceM = 0.0;
        Double ManualAssistDistanceM = 0.0;
        RunSequencer.SequencedRun? PreviousRun = null;
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
                " coverage comparison"));
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
        List<CoveragePlanner.Run> Runs,
        String WorkAction,
        String InitialAction,
        Grid Grid,
        ref RunSequencer.SequencedRun? PreviousRun,
        ref Int32 ConnectorCount,
        ref Int32 SuccessfulConnectorCount,
        ref Double ConnectorDistanceM,
        ref Int32 OperatorAlignmentCount,
        ref Double ManualAssistDistanceM
        )
    {
        for (Int32 RunIndex = 0; RunIndex < Runs.Count; RunIndex += 1)
        {
            CoveragePlanner.Run Run = Runs[RunIndex];
            RunSequencer.SequencedRun Prepared = RunSequencer.Prepare(
                Run,
                PlanningPreferredLineupM,
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
                TransitPlanner.Result Connector = TransitPlanner.Connect(
                    new ConnectorPlanner.Pose(
                        PreviousRun.ExitPose.X,
                        PreviousRun.ExitPose.Y,
                        PreviousRun.ExitPose.HeadingDeg),
                    Prepared.EntryPose,
                    PlanningMinimumRadiusM,
                    Math.Min(0.5, Grid.GridSizeM),
                    Grid);
                if (Connector.Success)
                {
                    SuccessfulConnectorCount += 1;
                    ConnectorDistanceM += Connector.LengthM;
                    ManualAssistDistanceM += Connector.ManualAssistDistanceM;
                    foreach (TransitPlanner.Leg Leg in Connector.Legs)
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
}
