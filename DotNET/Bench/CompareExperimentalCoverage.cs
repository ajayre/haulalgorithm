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
    /// Writes one cut/fill map showing the complete connected lifecycle for routes one to three.
    /// </summary>
    /// <param name="Args">Optional output PNG path and AGD path.</param>
    /// <returns>Zero when the selected lifecycle is complete.</returns>
    static public Int32 ExportFirstThree
        (
        String[] Args
        )
    {
        String RepoRoot = Path.GetFullPath(Path.Combine(
            AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String OutputPath = (Args.Length > 0)
            ? Path.GetFullPath(Args[0])
            : Path.Combine(
                RepoRoot,
                "assets",
                "theshop2_experimental_coverage",
                "routes_01_03_connected.png");
        String AgdPath = (Args.Length > 1)
            ? Path.GetFullPath(Args[1])
            : Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");
        String? OutputDirectory = Path.GetDirectoryName(OutputPath);
        if (!String.IsNullOrWhiteSpace(OutputDirectory))
        {
            Directory.CreateDirectory(OutputDirectory);
        }

        (AgdField Field, Grid Grid, Route Route, Double HeadingDeg, EdgePatchOmissionResult EdgeOmission) Plan = BuildRoute(AgdPath);
        Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0;
        List<ExperimentalLifecyclePlanner.PassPlan> AllPasses =
            BuildWholePlanPasses(Plan.Route, Plan.Grid, BladeM, true, 3);
        List<ExperimentalLifecyclePlanner.PassPlan> Passes = AllPasses.Take(3).ToList();
        ExperimentalLifecyclePlanner.Result Lifecycle = ExperimentalLifecyclePlanner.Build(
            Passes,
            true,
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS,
            Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS,
            Math.Min(0.5, Plan.Grid.GridSizeM),
            Plan.Grid);
        if (!Lifecycle.Success)
        {
            Console.Error.WriteLine("routes 1-3 lifecycle failed: " + Lifecycle.FailureReason);
            return 1;
        }
        List<IReadOnlyList<(Double X, Double Y)>> CutRuns = Passes
            .SelectMany(Pass => Pass.Cut.Runs)
            .Select(Run => (IReadOnlyList<(Double X, Double Y)>)new List<(Double X, Double Y)>
            {
                Run.Run.StartXy,
                Run.Run.EndXy,
            })
            .ToList();
        List<IReadOnlyList<(Double X, Double Y)>> FillRuns = Passes
            .SelectMany(Pass => Pass.Fill.Runs)
            .Select(Run => (IReadOnlyList<(Double X, Double Y)>)new List<(Double X, Double Y)>
            {
                Run.Run.StartXy,
                Run.Run.EndXy,
            })
            .ToList();
        List<(Double X, Double Y)> RoutePoints = Lifecycle.Segments
            .SelectMany(Segment => Segment.Points)
            .ToList();
        ReportLifecycleGeometry(Lifecycle, Plan.Grid, BladeM);
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
                    (Index + 1).ToString(CultureInfo.InvariantCulture));
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
            "wrote {0} routes=3 segments={1} manual_zones={2} distance_m={3:F1} coverage_shortfall_m={4:F1} guidance_omitted_cells={5} guidance_omitted_area_m2={6:F2} edge_omitted_cut_cells={7} edge_omitted_cut_bcy={8:F2}",
            OutputPath,
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
        Double CheckSpacingM = Math.Min(0.10, Grid.GridSizeM / 4.0);
        ExperimentalFieldEdgePolicy EdgePolicy =
            new ExperimentalFieldEdgePolicy(Grid, EdgeAllowanceM);

        for (Int32 SegmentIndex = 0; SegmentIndex < Lifecycle.Segments.Count; SegmentIndex += 1)
        {
            ExperimentalLifecyclePlanner.Segment Segment = Lifecycle.Segments[SegmentIndex];
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
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "geometry outside_field_samples={0} outside_field_interpolated={1} beyond_allowance_samples={2} beyond_allowance_interpolated={3} max_segment_gap_m={4:F3} gap_after_segment={5}",
            OutsideSampleCount,
            OutsideInterpolatedCount,
            BeyondAllowanceSampleCount,
            BeyondAllowanceInterpolatedCount,
            MaximumBoundaryGapM,
            MaximumBoundaryGapAfterSegment));
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
        Int32? PassLimit = null
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
            ExperimentalConnectorPlanner.Pose? FillEntry = FillSequence.Runs.Count == 0
                ? null
                : FillSequence.Runs[0].EntryPose;
            ExperimentalConnectorPlanner.Pose? CutSearchStart = GradeReturn
                ? null
                : Current;
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
                        FillEntry.HasValue ? (FillEntry.Value.X, FillEntry.Value.Y) : null))
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
                foreach (ExperimentalCoveragePlanner.PlanResult FillCandidate in FillCandidates)
                {
                    ExperimentalRunSequencer.Result ProvisionalFillSequence =
                        BuildSequence(FillCandidate.Runs, Grid);
                    if (!ProvisionalFillSequence.Success
                        || (ProvisionalFillSequence.Runs.Count == 0))
                    {
                        continue;
                    }
                    ExperimentalConnectorPlanner.Pose CandidateFillEntry =
                        ProvisionalFillSequence.Runs[0].EntryPose;
                    foreach (ExperimentalCoveragePlanner.PlanResult CutCandidate in CutCandidates)
                    {
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
                        if (!CandidateFillSequence.Success
                            || (GradeReturn
                                && !SupportsGradeReturn(CandidateFillSequence, Grid))
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
                        break;
                    }
                    if (SelectedPair) break;
                }
            }
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
            Boolean PreviousToFillConnected = !Current.HasValue
                || (CutSequence.Runs.Count > 0)
                || (FillSequence.Runs.Count == 0)
                || PoseConnectsToSequence(
                    Current.Value, FillSequence, Grid, GradeReturn);
            Boolean PreviousToCutConnected = !Current.HasValue
                || (CutSequence.Runs.Count == 0)
                || PoseConnectsToSequence(
                    Current.Value, CutSequence, Grid, GradeReturn);
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
            Output.Add(new ExperimentalLifecyclePlanner.PassPlan(
                CutSequence,
                FillSequence,
                Cut.OmittedCells.Count,
                Fill.OmittedCells.Count));
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
            ExperimentalCoveragePlanner.CandidatePlans(
                Cells, Grid, BladeM, Start, End);
        Int32 MinimumOmittedCells = Candidates.Min(
            Candidate => Candidate.OmittedCells.Count);
        return Candidates
            .Where(Candidate => Candidate.OmittedCells.Count == MinimumOmittedCells)
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
        Boolean AllowKeyholeHybrid = false
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
            AllowKeyholeHybrid: AllowKeyholeHybrid);
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
            Output.Add(ExperimentalRunSequencer.Prepare(
                Run, LineupM, MinimumRadiusM, SampleSpacingM, Grid).EntryPose);
            Output.Add(ExperimentalRunSequencer.Prepare(
                ReverseRun(Run), LineupM, MinimumRadiusM, SampleSpacingM, Grid).EntryPose);
        }
        return Output;
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
        out ExperimentalRunSequencer.Result SelectedFillSequence
        )
    {
        List<ExperimentalCoveragePlanner.PlanResult> PreviousFillCandidates =
            MinimalOmissionCandidates(
                PreviousFillCells,
                Grid,
                BladeM,
                (PreviousCutExit.X, PreviousCutExit.Y),
                null);
        List<ExperimentalCoveragePlanner.PlanResult> CutCandidates =
            MinimalOmissionCandidates(
                CurrentCutCells, Grid, BladeM);
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

        foreach (ExperimentalCoveragePlanner.PlanResult FillCandidate in FillCandidates)
        {
            ExperimentalRunSequencer.Result ProvisionalFill =
                BuildSequence(FillCandidate.Runs, Grid);
            if (!ProvisionalFill.Success || (ProvisionalFill.Runs.Count == 0)) continue;
            ExperimentalConnectorPlanner.Pose FillEntry =
                ProvisionalFill.Runs[0].EntryPose;
            foreach (ExperimentalCoveragePlanner.PlanResult CutCandidate in CutCandidates)
            {
                if (CutCandidate.Runs.Count == 0) continue;
                List<ExperimentalConnectorPlanner.Pose> CutEntryTargets =
                    PossibleEntryPoses(CutCandidate.Runs, Grid);
                foreach (ExperimentalConnectorPlanner.Pose CutEntryTarget in CutEntryTargets)
                {
                    foreach (ExperimentalCoveragePlanner.PlanResult PreviousFillCandidate
                        in PreviousFillCandidates)
                    {
                        ExperimentalRunSequencer.Result PreviousFillSequence =
                            BuildSequence(
                                PreviousFillCandidate.Runs,
                                Grid,
                                PreviousCutExit,
                                CutEntryTarget);
                        if (!PreviousFillSequence.Success
                            || (PreviousFillSequence.Runs.Count == 0)
                            || !PoseConnectsToSequence(
                                PreviousCutExit, PreviousFillSequence, Grid)
                            || (GradeReturn
                                && !SupportsGradeReturn(PreviousFillSequence, Grid)))
                        {
                            continue;
                        }
                        ExperimentalConnectorPlanner.Pose PreviousOutgoing = GradeReturn
                            ? ReversePose(PreviousFillSequence.Runs[0].EntryPose)
                            : PreviousFillSequence.Runs[
                                PreviousFillSequence.Runs.Count - 1].ExitPose;
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
                        if (!FillSequence.Success
                            || !SequencesConnect(CutSequence, FillSequence, Grid)
                            || (GradeReturn && !SupportsGradeReturn(FillSequence, Grid)))
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
                        if (OmittedCells == MinimumOmittedCells)
                        {
                            SelectedPreviousFillPlan = PreviousFillCandidate;
                            SelectedPreviousFillSequence = PreviousFillSequence;
                            SelectedCutPlan = CutCandidate;
                            SelectedCutSequence = CutSequence;
                            SelectedFillPlan = FillCandidate;
                            SelectedFillSequence = FillSequence;
                            return true;
                        }
                    }
                }
            }
        }
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
