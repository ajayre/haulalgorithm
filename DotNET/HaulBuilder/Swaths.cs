using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// One waypoint in the blade-scale operator guidance track.
    /// </summary>
    public sealed class GuidancePoint
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="GuidancePoint"/> class.
        /// </summary>
        /// <param name="seq">1-based global sequence number.</param>
        /// <param name="passNo">1-based pass index in the optimized route.</param>
        /// <param name="action">Move/action type on the inbound segment.</param>
        /// <param name="blade">Blade state on the inbound segment.</param>
        /// <param name="x">Smoothed steering easting in metres.</param>
        /// <param name="y">Smoothed steering northing in metres.</param>
        /// <param name="lat">Latitude.</param>
        /// <param name="lon">Longitude.</param>
        /// <param name="depthMm">Target depth in millimetres (magnitude).</param>
        /// <param name="headingDeg">Bearing to next waypoint (0=N, 90=E).</param>
        /// <param name="segDistM">Coverage distance from previous waypoint in metres.</param>
        /// <param name="cumDistM">Cumulative coverage distance in metres.</param>
        /// <param name="instruction">Plain-language operator instruction.</param>
        public GuidancePoint
            (
            Int32 seq,
            Int32 passNo,
            String action,
            String blade,
            Double x,
            Double y,
            Double lat,
            Double lon,
            Double depthMm,
            Double headingDeg,
            Double segDistM,
            Double cumDistM,
            String instruction
            )
        {
            // Persist all waypoint fields exactly as computed during guidance assembly.
            Seq = seq;
            PassNo = passNo;
            Action = action;
            Blade = blade;
            X = x;
            Y = y;
            Lat = lat;
            Lon = lon;
            DepthMm = depthMm;
            HeadingDeg = headingDeg;
            SegDistM = segDistM;
            CumDistM = cumDistM;
            Instruction = instruction;
        }

        /// <summary>
        /// Gets the 1-based global waypoint sequence.
        /// </summary>
        public Int32 Seq { get; }

        /// <summary>
        /// Gets the 1-based pass number in route order.
        /// </summary>
        public Int32 PassNo { get; }

        /// <summary>
        /// Gets the inbound action code.
        /// </summary>
        public String Action { get; }

        /// <summary>
        /// Gets the blade state for the inbound leg.
        /// </summary>
        public String Blade { get; }

        /// <summary>
        /// Gets the smoothed steering easting in metres.
        /// </summary>
        public Double X { get; }

        /// <summary>
        /// Gets the smoothed steering northing in metres.
        /// </summary>
        public Double Y { get; }

        /// <summary>
        /// Gets latitude.
        /// </summary>
        public Double Lat { get; }

        /// <summary>
        /// Gets longitude.
        /// </summary>
        public Double Lon { get; }

        /// <summary>
        /// Gets target blade depth in millimetres (magnitude).
        /// </summary>
        public Double DepthMm { get; }

        /// <summary>
        /// Gets heading to the next waypoint in degrees.
        /// </summary>
        public Double HeadingDeg { get; }

        /// <summary>
        /// Gets segment coverage distance from the previous waypoint in metres.
        /// </summary>
        public Double SegDistM { get; }

        /// <summary>
        /// Gets cumulative coverage distance from the start in metres.
        /// </summary>
        public Double CumDistM { get; }

        /// <summary>
        /// Gets plain-language operator guidance text.
        /// </summary>
        public String Instruction { get; }
    }

    /// <summary>
    /// Full blade-scale guidance output and distance accounting.
    /// </summary>
    public sealed class GuidanceResult
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="GuidanceResult"/> class.
        /// </summary>
        /// <param name="points">Ordered guidance waypoints.</param>
        /// <param name="bladeWidthM">Blade width in metres.</param>
        /// <param name="gridSizeM">Grid cell edge length in metres.</param>
        /// <param name="headingDeg">Global fallback heading in degrees.</param>
        /// <param name="nPasses">Total number of sequenced passes.</param>
        /// <param name="returnDistM">Total RETURN distance in metres.</param>
        /// <param name="haulDistM">Total HAUL distance in metres.</param>
        /// <param name="cutDistM">Total CUT distance in metres.</param>
        /// <param name="fillDistM">Total FILL distance in metres.</param>
        /// <param name="gradeDistM">Total GRADE distance in metres.</param>
        /// <param name="repositionDistM">Total MOVE distance in metres.</param>
        /// <param name="plannedCutDistM">Planned centerline cut distance in metres.</param>
        /// <param name="plannedFillDistM">Planned centerline fill distance in metres.</param>
        /// <param name="routeHaulDistM">Optimizer haul objective total in metres.</param>
        /// <param name="nTurns">Count of significant heading changes.</param>
        /// <param name="turnAngleRad">Total significant heading change in radians.</param>
        /// <param name="nRepairedSweeps">Count of sweep repairs applied.</param>
        public GuidanceResult
            (
            List<GuidancePoint> points,
            Double bladeWidthM,
            Double gridSizeM,
            Double headingDeg,
            Int32 nPasses,
            Double returnDistM,
            Double haulDistM,
            Double cutDistM,
            Double fillDistM,
            Double gradeDistM,
            Double repositionDistM,
            Double plannedCutDistM,
            Double plannedFillDistM,
            Double routeHaulDistM,
            Int32 nTurns,
            Double turnAngleRad,
            Int32 nRepairedSweeps
            )
        {
            // Store complete summary metrics so timing and reporting can run independently.
            Points = points;
            BladeWidthM = bladeWidthM;
            GridSizeM = gridSizeM;
            HeadingDeg = headingDeg;
            NPasses = nPasses;
            ReturnDistM = returnDistM;
            HaulDistM = haulDistM;
            CutDistM = cutDistM;
            FillDistM = fillDistM;
            GradeDistM = gradeDistM;
            RepositionDistM = repositionDistM;
            PlannedCutDistM = plannedCutDistM;
            PlannedFillDistM = plannedFillDistM;
            RouteHaulDistM = routeHaulDistM;
            NTurns = nTurns;
            TurnAngleRad = turnAngleRad;
            NRepairedSweeps = nRepairedSweeps;
        }

        /// <summary>
        /// Gets ordered blade-scale guidance waypoints.
        /// </summary>
        public List<GuidancePoint> Points { get; }

        /// <summary>
        /// Gets blade width in metres.
        /// </summary>
        public Double BladeWidthM { get; }

        /// <summary>
        /// Gets grid size in metres.
        /// </summary>
        public Double GridSizeM { get; }

        /// <summary>
        /// Gets global fallback heading in degrees.
        /// </summary>
        public Double HeadingDeg { get; }

        /// <summary>
        /// Gets total pass count.
        /// </summary>
        public Int32 NPasses { get; }

        /// <summary>
        /// Gets total RETURN distance in metres.
        /// </summary>
        public Double ReturnDistM { get; }

        /// <summary>
        /// Gets total HAUL distance in metres.
        /// </summary>
        public Double HaulDistM { get; }

        /// <summary>
        /// Gets total CUT distance in metres.
        /// </summary>
        public Double CutDistM { get; }

        /// <summary>
        /// Gets total FILL distance in metres.
        /// </summary>
        public Double FillDistM { get; }

        /// <summary>
        /// Gets total GRADE distance in metres.
        /// </summary>
        public Double GradeDistM { get; }

        /// <summary>
        /// Gets total MOVE distance in metres.
        /// </summary>
        public Double RepositionDistM { get; }

        /// <summary>
        /// Gets planned centerline cut sweep distance in metres.
        /// </summary>
        public Double PlannedCutDistM { get; }

        /// <summary>
        /// Gets planned centerline fill sweep distance in metres.
        /// </summary>
        public Double PlannedFillDistM { get; }

        /// <summary>
        /// Gets optimizer haul objective distance in metres.
        /// </summary>
        public Double RouteHaulDistM { get; }

        /// <summary>
        /// Gets count of significant heading changes.
        /// </summary>
        public Int32 NTurns { get; }

        /// <summary>
        /// Gets total significant heading change in radians.
        /// </summary>
        public Double TurnAngleRad { get; }

        /// <summary>
        /// Gets the number of sweeps repaired by knot fix-up.
        /// </summary>
        public Int32 NRepairedSweeps { get; }

        /// <summary>
        /// Computes blade-scale sweep length divided by planned centerline sweep length.
        /// </summary>
        /// <returns>Coverage factor, always at least 1 when planned distance is positive.</returns>
        public Double CoverageFactor
            (
            )
        {
            // Keep parity with Python: if no planned sweep exists, return neutral factor.
            Double Planned = PlannedCutDistM + PlannedFillDistM;
            Double Actual = CutDistM + FillDistM;
            return (Planned > 0.0) ? (Actual / Planned) : 1.0;
        }
    }

    /// <summary>
    /// Realistic time estimate built from the drivable guidance track.
    /// </summary>
    public sealed class GuidanceTime
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="GuidanceTime"/> class.
        /// </summary>
        /// <param name="totalS">Total time in seconds.</param>
        /// <param name="loadingS">Loading time in seconds.</param>
        /// <param name="dumpingS">Dumping time in seconds.</param>
        /// <param name="gradingS">Grading time in seconds.</param>
        /// <param name="haulingS">Hauling time in seconds.</param>
        /// <param name="returningS">Returning time in seconds.</param>
        /// <param name="turningS">Turning time in seconds.</param>
        /// <param name="otherS">Non-productive overhead time in seconds.</param>
        /// <param name="nTurns">Count of significant turns.</param>
        public GuidanceTime
            (
            Double totalS,
            Double loadingS,
            Double dumpingS,
            Double gradingS,
            Double haulingS,
            Double returningS,
            Double turningS,
            Double otherS,
            Int32 nTurns
            )
        {
            // Persist timing breakdown by phase for reporting parity with Python.
            TotalS = totalS;
            LoadingS = loadingS;
            DumpingS = dumpingS;
            GradingS = gradingS;
            HaulingS = haulingS;
            ReturningS = returningS;
            TurningS = turningS;
            OtherS = otherS;
            NTurns = nTurns;
        }

        /// <summary>
        /// Gets total guidance time in seconds.
        /// </summary>
        public Double TotalS { get; }

        /// <summary>
        /// Gets loading time in seconds.
        /// </summary>
        public Double LoadingS { get; }

        /// <summary>
        /// Gets dumping time in seconds.
        /// </summary>
        public Double DumpingS { get; }

        /// <summary>
        /// Gets grading time in seconds.
        /// </summary>
        public Double GradingS { get; }

        /// <summary>
        /// Gets hauling time in seconds.
        /// </summary>
        public Double HaulingS { get; }

        /// <summary>
        /// Gets returning time in seconds.
        /// </summary>
        public Double ReturningS { get; }

        /// <summary>
        /// Gets turning time in seconds.
        /// </summary>
        public Double TurningS { get; }

        /// <summary>
        /// Gets non-productive overhead time in seconds.
        /// </summary>
        public Double OtherS { get; }

        /// <summary>
        /// Gets count of significant turns.
        /// </summary>
        public Int32 NTurns { get; }
    }

    /// <summary>
    /// Blade-scale swath and guidance construction utilities ported from Python.
    /// </summary>
    public static class Swaths
    {
        private const String START = "START";
        private const String RETURN = "RETURN";
        private const String HAUL = "HAUL";
        private const String CUT = "CUT";
        private const String FILL = "FILL";
        private const String GRADE = "GRADE";
        private const String MOVE = "MOVE";
        private const String MANUAL_MOVE = "MANUAL_MOVE";
        private const Double EPSILON = 1e-12;
        private const Double DEFAULT_TURNING_CIRCLE_FT = 50.0;

        /// <summary>
        /// Soft-bend length budget in half-turns of R (1 => πR) plus the chord.
        /// </summary>
        private const Double SOFT_BEND_MAX_HALF_TURNS = 1.0;

        /// <summary>
        /// Max forward slide for an in-field keyhole, in multiples of R.
        /// </summary>
        private const Double KEYHOLE_FORWARD_MAX_FRAC_R = 2.5;

        /// <summary>
        /// Extra length budget (multiples of R) beyond 2.75πR for a forward-slid keyhole spur.
        /// </summary>
        private const Double KEYHOLE_SPUR_BUDGET_FRAC_R = 5.0;

        /// <summary>
        /// Minimum local heading change (deg) before coalescing a CCC keyhole via FindUturnExit.
        /// </summary>
        /// <remarks>
        /// Soft pinches below this still try a classic fillet; they must not start a 4R
        /// reverse-window keyhole (that bulb is what a slight bend was incorrectly getting).
        /// </remarks>
        private const Double UTURN_COALESCE_MIN_HEADING_DEG = 60.0;

        /// <summary>
        /// Minimum sustained reverse travel (as a fraction of R) for a single-segment reverse exit.
        /// </summary>
        private const Double UTURN_REVERSE_MIN_FRAC_R = 0.5;

        private sealed class RawPoint
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="RawPoint"/> class.
            /// </summary>
            /// <param name="xr">Raw coverage easting.</param>
            /// <param name="yr">Raw coverage northing.</param>
            /// <param name="depthM">Depth in metres.</param>
            /// <param name="action">Action for inbound leg.</param>
            /// <param name="passNo">Pass number.</param>
            /// <param name="xs">Smoothed steering easting.</param>
            /// <param name="ys">Smoothed steering northing.</param>
            public RawPoint
                (
                Double xr,
                Double yr,
                Double depthM,
                String action,
                Int32 passNo,
                Double xs,
                Double ys
                )
            {
                // Keep both raw and smoothed coordinates so metrics and display stay decoupled.
                Xr = xr;
                Yr = yr;
                DepthM = depthM;
                Action = action;
                PassNo = passNo;
                Xs = xs;
                Ys = ys;
            }

            /// <summary>
            /// Gets raw coverage easting.
            /// </summary>
            public Double Xr { get; }

            /// <summary>
            /// Gets raw coverage northing.
            /// </summary>
            public Double Yr { get; }

            /// <summary>
            /// Gets depth in metres.
            /// </summary>
            public Double DepthM { get; }

            /// <summary>
            /// Gets action code.
            /// </summary>
            public String Action { get; }

            /// <summary>
            /// Gets pass number.
            /// </summary>
            public Int32 PassNo { get; }

            /// <summary>
            /// Gets smoothed steering easting.
            /// </summary>
            public Double Xs { get; }

            /// <summary>
            /// Gets smoothed steering northing.
            /// </summary>
            public Double Ys { get; }
        }

        /// <summary>
        /// Re-profiles tangled pass sweeps into boustrophedon lanes in place.
        /// </summary>
        /// <param name="route">Route whose ordered passes are updated in place.</param>
        /// <param name="grid">Grid geometry and cell-center projection utility.</param>
        /// <param name="bladeWidthMm">Blade width in millimetres.</param>
        /// <param name="hairpinMin">Minimum hairpin count that flags a knot.</param>
        /// <param name="headingDeg">Fallback heading for non-flow passes.</param>
        /// <param name="laneOverlap">Lane overlap fraction [0, 1).</param>
        /// <returns>Number of cut/fill patches that were re-profiled.</returns>
        static public Int32 ReprofileKnots
            (
            Route route,
            Grid grid,
            Double bladeWidthMm,
            Int32 hairpinMin = 3,
            Double headingDeg = 0.0,
            Double laneOverlap = 0.0
            )
        {
            // Walk passes and only replace a sweep when the alternative strictly improves drivability.
            Double BladeM = bladeWidthMm / 1000.0;
            Int32 Count = 0;

            foreach (Pass Pass in route.OrderedPasses)
            {
                Boolean FlowPass = (Pass.CutHeadingDeg.HasValue || Pass.FillHeadingDeg.HasValue);
                Double CutHeading = Pass.CutHeadingDeg.HasValue ? Pass.CutHeadingDeg.Value : headingDeg;
                Double FillHeading = Pass.FillHeadingDeg.HasValue ? Pass.FillHeadingDeg.Value : headingDeg;

                if (Pass.CutCells.Count > 3)
                {
                    List<(Double X, Double Y, Double DepthM)> Rendered = RenderedLane
                        (
                        Pass.CutCells,
                        grid,
                        BladeM,
                        FlowPass,
                        CutHeading,
                        laneOverlap,
                        null,
                        Pass.FillStartXy
                        );
                    Int32 CurrentHairpins = HardTurnCount(Rendered, grid, BladeM);
                    // Intent: compact blade-width knots under-count after RDP; also flag raw reverses.
                    Int32 KnotScore = Math.Max(CurrentHairpins, CellNearReverseCount(Pass.CutCells, grid));

                    if (KnotScore >= hairpinMin)
                    {
                        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> BCells = BoustrophedonCells
                            (
                            Pass.CutCells,
                            grid,
                            BladeM,
                            null,
                            Pass.FillStartXy
                            );
                        List<(Double X, Double Y, Double DepthM)> BLane = LaneDirect(BCells, grid, null, Pass.FillStartXy);
                        Int32 BScore = Math.Max(
                            HardTurnCount(BLane, grid, BladeM),
                            CellNearReverseCount(BCells, grid));
                        // Intent: adopt only when total U-turn burden drops (lanes−1 vs knot score).
                        Int32 CurrentProxy = Math.Max(KnotScore, PatchDrivableLanes(Pass.CutCells, grid, BladeM).Count - 1);
                        Int32 BoustProxy = Math.Max(BScore, PatchDrivableLanes(BCells, grid, BladeM).Count - 1);

                        if (BoustProxy < CurrentProxy)
                        {
                            // Keep data parity by only changing in-patch ordering, not bins or quantities.
                            Pass.CutCells = BCells;
                            Pass.CutKnotReduced = true;
                            Count += 1;
                        }
                    }
                }

                (Double X, Double Y)? CutEnd = null;
                if (Pass.CutCells.Count > 0)
                {
                    (Int32 Bx, Int32 By) LastCut = Pass.CutCells[Pass.CutCells.Count - 1].Cell;
                    CutEnd = grid.CellCenterXY(LastCut.Bx, LastCut.By);
                }

                if (Pass.FillCells.Count > 3)
                {
                    List<(Double X, Double Y, Double DepthM)> Rendered = RenderedLane
                        (
                        Pass.FillCells,
                        grid,
                        BladeM,
                        FlowPass,
                        FillHeading,
                        laneOverlap,
                        CutEnd,
                        null
                        );
                    Int32 CurrentHairpins = HardTurnCount(Rendered, grid, BladeM);
                    Int32 KnotScore = Math.Max(CurrentHairpins, CellNearReverseCount(Pass.FillCells, grid));

                    if (KnotScore >= hairpinMin)
                    {
                        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> BCells = BoustrophedonCells
                            (
                            Pass.FillCells,
                            grid,
                            BladeM,
                            CutEnd,
                            null
                            );
                        List<(Double X, Double Y, Double DepthM)> BLane = LaneDirect(BCells, grid, CutEnd, null);
                        Int32 BScore = Math.Max(
                            HardTurnCount(BLane, grid, BladeM),
                            CellNearReverseCount(BCells, grid));
                        Int32 CurrentProxy = Math.Max(KnotScore, PatchDrivableLanes(Pass.FillCells, grid, BladeM).Count - 1);
                        Int32 BoustProxy = Math.Max(BScore, PatchDrivableLanes(BCells, grid, BladeM).Count - 1);

                        if (BoustProxy < CurrentProxy)
                        {
                            // Keep logic symmetric with cut-side fix-up.
                            Pass.FillCells = BCells;
                            Pass.FillKnotReduced = true;
                            Count += 1;
                        }
                    }
                }
            }

            return Count;
        }

        /// <summary>
        /// Builds the blade-scale guidance track from an optimized route.
        /// </summary>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="field">AGD field metadata for coordinate conversion.</param>
        /// <param name="route">Optimized route sequence.</param>
        /// <param name="bladeWidthMm">Blade width in millimetres.</param>
        /// <param name="headingDeg">Fallback global heading for non-flow passes.</param>
        /// <param name="laneOverlap">Lane overlap fraction [0, 1).</param>
        /// <param name="gradeReturn">Whether to include grade/compact return over fill.</param>
        /// <param name="turnThresholdDeg">Minimum heading change counted as turn.</param>
        /// <param name="smoothRoute">Whether to smooth steering geometry.</param>
        /// <param name="turnRadiusM">Optional explicit turn radius in metres.</param>
        /// <param name="smoothScale">Multiplier applied to smoothing sigma.</param>
        /// <param name="obstacles">Optional static obstacle field for detours.</param>
        /// <param name="armObstacles">Optional per-pass obstacle fields.</param>
        /// <param name="equipmentLengthM">Straight lineup length before/after CUT/FILL (metres); 0 disables.</param>
        /// <returns>Complete guidance output and accounting metrics.</returns>
        static public GuidanceResult BuildGuidance
            (
            Grid grid,
            AgdField field,
            Route route,
            Double bladeWidthMm,
            Double headingDeg,
            Double laneOverlap = 0.0,
            Boolean gradeReturn = true,
            Double turnThresholdDeg = 60.0,
            Boolean smoothRoute = true,
            Double? turnRadiusM = null,
            Double smoothScale = 0.7,
            ObstacleField? obstacles = null,
            Dictionary<Int32, ObstacleField>? armObstacles = null,
            Double equipmentLengthM = -1.0
            )
        {
            // Preserve Python accounting model: metrics use raw coverage path, not smoothed display path.
            Double BladeM = bladeWidthMm / 1000.0;
            ObstacleField Obstacles = obstacles ?? EMPTY_OBSTACLES;
            Double HopThresh = Math.Max(3.0 * grid.GridSizeM, 2.0 * BladeM);
            Double EffectiveTurnRadiusM = turnRadiusM.HasValue ? turnRadiusM.Value : (DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS);
            Double SigmaIdx = smoothRoute ? ((EffectiveTurnRadiusM / grid.GridSizeM) * smoothScale) : 0.0;
            // Intent: default 43 ft lineup when caller leaves equipmentLengthM at the sentinel.
            Double LineupM = (equipmentLengthM < 0.0)
                ? (Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS)
                : equipmentLengthM;

            List<GuidancePoint> Points = new List<GuidancePoint>();
            List<RawPoint> Raw = new List<RawPoint>();

            Double ReturnDist = 0.0;
            Double HaulDist = 0.0;
            Double CutDist = 0.0;
            Double FillDist = 0.0;
            Double GradeDist = 0.0;
            Double RepositionDist = 0.0;
            Double PlannedCutDist = 0.0;
            Double PlannedFillDist = 0.0;
            Double RouteHaulDist = 0.0;
            Int32 RepairedSweeps = 0;

            for (Int32 PassNo = 1; PassNo <= route.OrderedPasses.Count; PassNo += 1)
            {
                Pass Pass = route.OrderedPasses[PassNo - 1];
                RouteHaulDist += Pass.HaulDistanceM;
                PlannedCutDist += SegLengthFromCells(Pass.CutCells, grid);
                PlannedFillDist += SegLengthFromCells(Pass.FillCells, grid);

                Boolean FlowPass = (Pass.CutHeadingDeg.HasValue || Pass.FillHeadingDeg.HasValue);
                Double CutHeading = Pass.CutHeadingDeg.HasValue ? Pass.CutHeadingDeg.Value : headingDeg;
                Double FillHeading = Pass.FillHeadingDeg.HasValue ? Pass.FillHeadingDeg.Value : headingDeg;

                List<(Double X, Double Y, Double DepthM)> CutLane;
                if (Pass.CutKnotReduced)
                {
                    // Respect previously repaired order exactly to avoid re-tangling.
                    CutLane = LaneDirect(Pass.CutCells, grid, null, Pass.FillStartXy);
                }
                else if (FlowPass)
                {
                    CutLane = LanesFlow(Pass.CutCells, grid, null, Pass.FillStartXy);
                    (CutLane, Int32 Fixed) = RepairIfTangled(CutLane, Pass.CutCells, grid, BladeM, null, Pass.FillStartXy);
                    RepairedSweeps += Fixed;
                }
                else
                {
                    CutLane = LanesMulti(Pass.CutCells, grid, BladeM, CutHeading, laneOverlap, null, Pass.FillStartXy);
                    (CutLane, Int32 Fixed) = RepairIfTangled(CutLane, Pass.CutCells, grid, BladeM, null, Pass.FillStartXy);
                    RepairedSweeps += Fixed;
                }

                (Double X, Double Y)? CutEndXy = null;
                if (CutLane.Count > 0)
                {
                    (Double X, Double Y, Double DepthM) Last = CutLane[CutLane.Count - 1];
                    CutEndXy = (Last.X, Last.Y);
                }

                List<(Double X, Double Y, Double DepthM)> FillLane;
                if (Pass.FillKnotReduced)
                {
                    FillLane = LaneDirect(Pass.FillCells, grid, CutEndXy, null);
                }
                else if (FlowPass)
                {
                    FillLane = LanesFlow(Pass.FillCells, grid, CutEndXy, null);
                    (FillLane, Int32 Fixed) = RepairIfTangled(FillLane, Pass.FillCells, grid, BladeM, CutEndXy, null);
                    RepairedSweeps += Fixed;
                }
                else
                {
                    FillLane = LanesMulti(Pass.FillCells, grid, BladeM, FillHeading, laneOverlap, CutEndXy, null);
                    (FillLane, Int32 Fixed) = RepairIfTangled(FillLane, Pass.FillCells, grid, BladeM, CutEndXy, null);
                    RepairedSweeps += Fixed;
                }

                List<(Double X, Double Y, Double DepthM)> CutDisp = SmoothLane(CutLane, SigmaIdx, HopThresh);
                List<(Double X, Double Y, Double DepthM)> FillDisp = SmoothLane(FillLane, SigmaIdx, HopThresh);

                for (Int32 i = 0; i < CutLane.Count; i += 1)
                {
                    (Double X, Double Y, Double DepthM) Step = CutLane[i];
                    String Action;

                    if (i == 0)
                    {
                        Action = (PassNo == 1) ? START : RETURN;
                    }
                    else if (Dist((CutLane[i - 1].X, CutLane[i - 1].Y), (Step.X, Step.Y)) > HopThresh)
                    {
                        Action = MOVE;
                    }
                    else
                    {
                        Action = CUT;
                    }

                    Raw.Add(new RawPoint(Step.X, Step.Y, Step.DepthM, Action, PassNo, CutDisp[i].X, CutDisp[i].Y));
                }

                for (Int32 i = 0; i < FillLane.Count; i += 1)
                {
                    (Double X, Double Y, Double DepthM) Step = FillLane[i];
                    String Action;

                    if (i == 0)
                    {
                        Action = HAUL;
                    }
                    else if (Dist((FillLane[i - 1].X, FillLane[i - 1].Y), (Step.X, Step.Y)) > HopThresh)
                    {
                        Action = MOVE;
                    }
                    else
                    {
                        Action = FILL;
                    }

                    Raw.Add(new RawPoint(Step.X, Step.Y, Step.DepthM, Action, PassNo, FillDisp[i].X, FillDisp[i].Y));
                }

                if (gradeReturn && (FillLane.Count >= 2))
                {
                    List<(Double X, Double Y, Double DepthM)> Rev = new List<(Double X, Double Y, Double DepthM)>(FillLane);
                    List<(Double X, Double Y, Double DepthM)> RevDisp = new List<(Double X, Double Y, Double DepthM)>(FillDisp);
                    Rev.Reverse();
                    RevDisp.Reverse();

                    for (Int32 i = 0; i < Rev.Count; i += 1)
                    {
                        (Double X, Double Y, Double DepthM) Step = Rev[i];

                        if ((i > 0) && (Dist((Rev[i - 1].X, Rev[i - 1].Y), (Step.X, Step.Y)) > HopThresh))
                        {
                            Raw.Add(new RawPoint(Step.X, Step.Y, 0.0, MOVE, PassNo, RevDisp[i].X, RevDisp[i].Y));
                        }
                        else
                        {
                            Raw.Add(new RawPoint(Step.X, Step.Y, 0.0, GRADE, PassNo, RevDisp[i].X, RevDisp[i].Y));
                        }
                    }
                }
            }

            // Hard turning-radius bound on blade-down sweeps before obstacle detours.
            Raw = EnforceMinTurnRadius(
                Raw, EffectiveTurnRadiusM, turnThresholdDeg, Math.Max(grid.GridSizeM, 0.5), grid, LineupM);
            // Intent: straighten the train before and after each CUT/FILL run.
            Double SampleM = Math.Max(grid.GridSizeM, 0.5);
            Raw = EnforceEquipmentLineup(Raw, LineupM, SampleM, EffectiveTurnRadiusM, grid);
            // Intent: exit/entry spurs change HAUL/MOVE chords — re-apply R-bounded transit.
            if (LineupM > 1e-6)
            {
                Raw = EnforceBladeUpTransitTurns(Raw, EffectiveTurnRadiusM, SampleM, grid);
                Raw = RepairSharpSteeringBends(Raw, EffectiveTurnRadiusM, SampleM, grid);
            }

            if ((Raw.Count >= 2) && (Obstacles.Active || (armObstacles != null)))
            {
                // Insert detour vertices only on blade-up legs so blade-down coverage remains untouched.
                List<RawPoint> Detoured = new List<RawPoint> { Raw[0] };

                for (Int32 i = 1; i < Raw.Count; i += 1)
                {
                    RawPoint Current = Raw[i];
                    RawPoint Previous = Detoured[Detoured.Count - 1];
                    ObstacleField FieldForPass = Obstacles;

                    if ((armObstacles != null) && armObstacles.TryGetValue(Current.PassNo, out ObstacleField? Value) && (Value != null))
                    {
                        FieldForPass = Value;
                    }

                    if (IsBladeUp(Current.Action) && (Current.Action != START) && FieldForPass.Active)
                    {
                        List<(Double X, Double Y)> Chain = FieldForPass.Path((Previous.Xr, Previous.Yr), (Current.Xr, Current.Yr));
                        for (Int32 j = 1; j < (Chain.Count - 1); j += 1)
                        {
                            (Double X, Double Y) V = Chain[j];
                            Detoured.Add(new RawPoint(V.X, V.Y, 0.0, Current.Action, Current.PassNo, V.X, V.Y));
                        }
                    }

                    Detoured.Add(Current);
                }

                Raw = Detoured;
            }

            Double Cum = 0.0;
            (Double X, Double Y)? PrevRaw = null;
            (Double X, Double Y)? PrevCoverageRaw = null;
            Int32 Seq = 0;

            for (Int32 Index = 0; Index < Raw.Count; Index += 1)
            {
                RawPoint Entry = Raw[Index];
                (Double X, Double Y) RawPointXy = (Entry.Xr, Entry.Yr);
                Double Seg;
                if (IsBladeDown(Entry.Action))
                {
                    // Skip inserted MOVE headlands so CUT/FILL/GRADE keep the coverage floor.
                    if (PrevCoverageRaw.HasValue)
                    {
                        Seg = Dist(PrevCoverageRaw.Value, RawPointXy);
                    }
                    else if (PrevRaw.HasValue)
                    {
                        Seg = Dist(PrevRaw.Value, RawPointXy);
                    }
                    else
                    {
                        Seg = 0.0;
                    }

                    PrevCoverageRaw = RawPointXy;
                }
                else
                {
                    Seg = PrevRaw.HasValue ? Dist(PrevRaw.Value, RawPointXy) : 0.0;
                    if (Entry.Action != MOVE)
                    {
                        PrevCoverageRaw = null;
                    }
                }

                if (Entry.Action == START)
                {
                    // Keep the first waypoint as zero travel to mirror Python semantics.
                    Seg = 0.0;
                }

                if (Entry.Action == RETURN)
                {
                    ReturnDist += Seg;
                }
                else if (Entry.Action == HAUL)
                {
                    HaulDist += Seg;
                }
                else if (Entry.Action == CUT)
                {
                    CutDist += Seg;
                }
                else if (Entry.Action == FILL)
                {
                    FillDist += Seg;
                }
                else if (Entry.Action == GRADE)
                {
                    GradeDist += Seg;
                }
                else if (Entry.Action == MOVE)
                {
                    RepositionDist += Seg;
                }

                Cum += Seg;

                Double Heading = 0.0;
                if (Index + 1 < Raw.Count)
                {
                    RawPoint Next = Raw[Index + 1];
                    Heading = BearingDeg((Entry.Xs, Entry.Ys), (Next.Xs, Next.Ys));
                }

                String Blade = IsBladeUp(Entry.Action) ? "UP" : "DOWN";
                Double DepthMm = Math.Abs(Entry.DepthM) * 1000.0;
                (Double Lat, Double Lon) LatLon = LatLonFromUtm(field, Entry.Xs, Entry.Ys);

                Seq += 1;
                Points.Add
                    (
                    new GuidancePoint
                        (
                        Seq,
                        Entry.PassNo,
                        Entry.Action,
                        Blade,
                        Entry.Xs,
                        Entry.Ys,
                        LatLon.Lat,
                        LatLon.Lon,
                        DepthMm,
                        Heading,
                        Seg,
                        Cum,
                        Instruction(Entry.Action, DepthMm)
                        )
                    );

                PrevRaw = RawPointXy;
            }

            List<Double> RawXs = new List<Double>();
            List<Double> RawYs = new List<Double>();
            foreach (RawPoint Entry in Raw)
            {
                // Feed turn metrics with raw coverage geometry so smoothing does not alter timing.
                RawXs.Add(Entry.Xr);
                RawYs.Add(Entry.Yr);
            }

            (Int32 TurnCount, Double TurnAngle) = TurnMetrics(RawXs, RawYs, grid.GridSizeM, BladeM, turnThresholdDeg);

            return new GuidanceResult
                (
                Points,
                BladeM,
                grid.GridSizeM,
                headingDeg,
                route.OrderedPasses.Count,
                ReturnDist,
                HaulDist,
                CutDist,
                FillDist,
                GradeDist,
                RepositionDist,
                PlannedCutDist,
                PlannedFillDist,
                RouteHaulDist,
                TurnCount,
                TurnAngle,
                RepairedSweeps
                );
        }

        /// <summary>
        /// Estimates realistic execution time for a guidance result.
        /// </summary>
        /// <param name="result">Guidance result with segment actions and distances.</param>
        /// <param name="loadingSpeedMph">Loading speed in mph.</param>
        /// <param name="haulSpeedMph">Hauling speed in mph.</param>
        /// <param name="dumpingSpeedMph">Dumping speed in mph.</param>
        /// <param name="returnSpeedMph">Returning speed in mph.</param>
        /// <param name="gradeSpeedMph">Grading speed in mph.</param>
        /// <param name="turnSpeedMph">Turning speed in mph.</param>
        /// <param name="turningCircleFt">Turning circle in feet.</param>
        /// <param name="accelTimeS">Acceleration phase time in seconds.</param>
        /// <param name="decelTimeS">Deceleration phase time in seconds.</param>
        /// <param name="fieldEfficiency">Field efficiency fraction in (0, 1].</param>
        /// <returns>Detailed timing breakdown for the track.</returns>
        static public GuidanceTime EstimateGuidanceTime
            (
            GuidanceResult result,
            Double loadingSpeedMph,
            Double haulSpeedMph,
            Double dumpingSpeedMph,
            Double returnSpeedMph,
            Double gradeSpeedMph,
            Double turnSpeedMph,
            Double turningCircleFt,
            Double accelTimeS,
            Double decelTimeS,
            Double fieldEfficiency
            )
        {
            // Match Python phase accounting exactly, including move-time model and efficiency scaling.
            Double LoadV = loadingSpeedMph * Constants.MPH_TO_MPS;
            Double HaulVMax = haulSpeedMph * Constants.MPH_TO_MPS;
            Double DumpV = dumpingSpeedMph * Constants.MPH_TO_MPS;
            Double ReturnVMax = returnSpeedMph * Constants.MPH_TO_MPS;
            Double GradeV = gradeSpeedMph * Constants.MPH_TO_MPS;
            Double TurnV = turnSpeedMph * Constants.MPH_TO_MPS;
            Double Radius = (turningCircleFt * Constants.FEET_TO_METERS) / 2.0;

            Double LoadS = 0.0;
            Double DumpS = 0.0;
            Double GradeS = 0.0;
            Double HaulS = 0.0;
            Double ReturnS = 0.0;

            foreach (GuidancePoint Point in result.Points)
            {
                Double Segment = Point.SegDistM;

                if (Point.Action == CUT)
                {
                    LoadS += (LoadV > 0.0) ? (Segment / LoadV) : 0.0;
                }
                else if (Point.Action == FILL)
                {
                    DumpS += (DumpV > 0.0) ? (Segment / DumpV) : 0.0;
                }
                else if (Point.Action == GRADE)
                {
                    GradeS += (GradeV > 0.0) ? (Segment / GradeV) : 0.0;
                }
                else if ((Point.Action == HAUL) || (Point.Action == MOVE)
                    || (Point.Action == MANUAL_MOVE))
                {
                    HaulS += MoveTime(Segment, HaulVMax, accelTimeS, decelTimeS);
                }
                else if ((Point.Action == RETURN) || (Point.Action == START))
                {
                    ReturnS += MoveTime(Segment, ReturnVMax, accelTimeS, decelTimeS);
                }
            }

            Double TurnS = (TurnV > 0.0) ? ((Radius * result.TurnAngleRad) / TurnV) : 0.0;
            Double Productive = LoadS + DumpS + GradeS + HaulS + ReturnS + TurnS;
            Double Efficiency = ((fieldEfficiency > 0.0) && (fieldEfficiency <= 1.0)) ? fieldEfficiency : 1.0;
            Double Total = Productive / Efficiency;
            Double Other = Total - Productive;

            return new GuidanceTime
                (
                Total,
                LoadS,
                DumpS,
                GradeS,
                HaulS,
                ReturnS,
                TurnS,
                Other,
                result.NTurns
                );
        }

        /// <summary>
        /// Writes guidance output to CSV for in-cab import.
        /// </summary>
        /// <param name="result">Guidance result to write.</param>
        /// <param name="path">Output CSV file path.</param>
        static public void WriteGuidanceCsv
            (
            GuidanceResult result,
            String path
            )
        {
            // Emit deterministic invariant-culture formatting to keep parity with Python CSV.
            using (StreamWriter Writer = new StreamWriter(path, false))
            {
                Writer.WriteLine("seq,pass,action,blade,lat,lon,depth_mm,heading_deg,seg_dist_m,cum_dist_m,instruction");

                foreach (GuidancePoint Point in result.Points)
                {
                    String Line = String.Join
                        (
                        ",",
                        Point.Seq.ToString(CultureInfo.InvariantCulture),
                        Point.PassNo.ToString(CultureInfo.InvariantCulture),
                        CsvEscape(Point.Action),
                        CsvEscape(Point.Blade),
                        Point.Lat.ToString("F7", CultureInfo.InvariantCulture),
                        Point.Lon.ToString("F7", CultureInfo.InvariantCulture),
                        Point.DepthMm.ToString("F0", CultureInfo.InvariantCulture),
                        Point.HeadingDeg.ToString("F1", CultureInfo.InvariantCulture),
                        Point.SegDistM.ToString("F2", CultureInfo.InvariantCulture),
                        Point.CumDistM.ToString("F1", CultureInfo.InvariantCulture),
                        CsvEscape(Point.Instruction)
                        );
                    Writer.WriteLine(Line);
                }
            }
        }

        /// <summary>
        /// Computes compass bearing from point p to point q (0=N, 90=E).
        /// </summary>
        /// <param name="p">Start point.</param>
        /// <param name="q">End point.</param>
        /// <returns>Bearing in degrees, normalized to [0, 360).</returns>
        static private Double BearingDeg
            (
            (Double X, Double Y) p,
            (Double X, Double Y) q
            )
        {
            // Use east/north ordering exactly like the Python atan2(de, dn) formulation.
            Double De = q.X - p.X;
            Double Dn = q.Y - p.Y;
            if ((Math.Abs(De) < EPSILON) && (Math.Abs(Dn) < EPSILON))
            {
                return 0.0;
            }

            Double Deg = RadiansToDegrees(Math.Atan2(De, Dn));
            return Mod360(Deg);
        }

        /// <summary>
        /// Computes squared euclidean distance between two points.
        /// </summary>
        /// <param name="p">First point.</param>
        /// <param name="q">Second point.</param>
        /// <returns>Squared distance.</returns>
        static private Double D2
            (
            (Double X, Double Y) p,
            (Double X, Double Y) q
            )
        {
            // Keep this helper allocation-free because it is called heavily during ordering.
            Double Dx = p.X - q.X;
            Double Dy = p.Y - q.Y;
            return (Dx * Dx) + (Dy * Dy);
        }

        /// <summary>
        /// Computes euclidean distance between two points.
        /// </summary>
        /// <param name="p">First point.</param>
        /// <param name="q">Second point.</param>
        /// <returns>Distance in metres.</returns>
        static private Double Dist
            (
            (Double X, Double Y) p,
            (Double X, Double Y) q
            )
        {
            // Provide a local replacement for Python math.dist used throughout the port.
            return Math.Sqrt(D2(p, q));
        }

        /// <summary>
        /// Computes centroid of a point cloud.
        /// </summary>
        /// <param name="points">Input points.</param>
        /// <returns>Centroid coordinates.</returns>
        static private (Double X, Double Y) Centroid
            (
            List<(Double X, Double Y)> points
            )
        {
            // Preserve exact averaging semantics used by the Python implementation.
            Double SumX = 0.0;
            Double SumY = 0.0;
            foreach ((Double X, Double Y) Point in points)
            {
                SumX += Point.X;
                SumY += Point.Y;
            }

            Double Count = points.Count;
            return (SumX / Count, SumY / Count);
        }

        /// <summary>
        /// Computes polyline length from a list of points.
        /// </summary>
        /// <param name="points">Ordered polyline points.</param>
        /// <returns>Total length in metres.</returns>
        static private Double SegLength
            (
            List<(Double X, Double Y)> points
            )
        {
            // Sum contiguous segment distances without smoothing or simplification.
            Double Total = 0.0;
            for (Int32 k = 0; k < (points.Count - 1); k += 1)
            {
                Total += Dist(points[k], points[k + 1]);
            }
            return Total;
        }

        /// <summary>
        /// Computes polyline length from cell center sequence.
        /// </summary>
        /// <param name="cells">Ordered cell-depth list.</param>
        /// <param name="grid">Grid projection helper.</param>
        /// <returns>Total centerline distance in metres.</returns>
        static private Double SegLengthFromCells
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            Grid grid
            )
        {
            // Map cells to center points then reuse the generic polyline length helper.
            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cells)
            {
                Points.Add(grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By));
            }
            return SegLength(Points);
        }

        /// <summary>
        /// Counts cell-path corners whose inbound/outbound directions are nearly reverse.
        /// </summary>
        /// <param name="cells">Ordered coverage cells.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <returns>Near-reverse corner count on the unsmoothed cell polyline.</returns>
        static private Int32 CellNearReverseCount
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            Grid grid
            )
        {
            // Intent: RDP-smoothed HardTurnCount can under-count compact knots when blade≈grid;
            // raw reverse corners still need boustrophedon repair.
            Int32 Count = 0;
            for (Int32 K = 1; K + 1 < cells.Count; K += 1)
            {
                (Double X, Double Y) A = grid.CellCenterXy(cells[K - 1].Cell.Bx, cells[K - 1].Cell.By);
                (Double X, Double Y) B = grid.CellCenterXy(cells[K].Cell.Bx, cells[K].Cell.By);
                (Double X, Double Y) C = grid.CellCenterXy(cells[K + 1].Cell.Bx, cells[K + 1].Cell.By);
                Double Dx1 = B.X - A.X;
                Double Dy1 = B.Y - A.Y;
                Double Dx2 = C.X - B.X;
                Double Dy2 = C.Y - B.Y;
                Double N1 = Math.Sqrt((Dx1 * Dx1) + (Dy1 * Dy1));
                Double N2 = Math.Sqrt((Dx2 * Dx2) + (Dy2 * Dy2));
                if ((N1 < 1e-6) || (N2 < 1e-6))
                {
                    continue;
                }

                if (((Dx1 * Dx2) + (Dy1 * Dy2)) / (N1 * N2) < -0.5)
                {
                    Count += 1;
                }
            }

            return Count;
        }

        /// <summary>
        /// Counts near-reversal turns on an RDP-simplified lane centerline.
        /// </summary>
        /// <param name="lane">Lane points with depth values.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="degThreshold">Minimum heading change treated as hairpin.</param>
        /// <returns>Hairpin-like turn count.</returns>
        static private Int32 HardTurnCount
            (
            List<(Double X, Double Y, Double DepthM)> lane,
            Grid grid,
            Double bladeM,
            Double degThreshold = 100.0
            )
        {
            // Simplify first so staircase raster artifacts do not inflate hairpin count.
            if (lane.Count < 3)
            {
                return 0;
            }

            List<Double> Xs = new List<Double>();
            List<Double> Ys = new List<Double>();
            foreach ((Double X, Double Y, Double DepthM) Point in lane)
            {
                Xs.Add(Point.X);
                Ys.Add(Point.Y);
            }

            Double Tol = Math.Max(0.5 * bladeM, 0.5 * grid.GridSizeM);
            List<Int32> Keep = RdpKeep(Xs, Ys, Tol);
            if (Keep.Count < 3)
            {
                return 0;
            }

            Int32 Count = 0;
            for (Int32 k = 1; k < (Keep.Count - 1); k += 1)
            {
                (Double X, Double Y) A = (Xs[Keep[k - 1]], Ys[Keep[k - 1]]);
                (Double X, Double Y) B = (Xs[Keep[k]], Ys[Keep[k]]);
                (Double X, Double Y) C = (Xs[Keep[k + 1]], Ys[Keep[k + 1]]);

                Double B0 = BearingDeg(A, B);
                Double B1 = BearingDeg(B, C);
                Double Delta = Math.Abs(((B1 - B0 + 180.0) % 360.0) - 180.0);

                if (Delta >= degThreshold)
                {
                    Count += 1;
                }
            }

            return Count;
        }

        /// <summary>
        /// Computes principal-axis heading of a point cloud in compass degrees.
        /// </summary>
        /// <param name="centers">Point cloud.</param>
        /// <returns>Major-axis heading in [0, 360).</returns>
        static private Double PrincipalHeadingDeg
            (
            List<(Double X, Double Y)> centers
            )
        {
            // Use covariance eigensystem to align lanes with patch long axis.
            Int32 N = centers.Count;
            if (N < 2)
            {
                return 0.0;
            }

            (Double X, Double Y) Origin = Centroid(centers);
            Double Sxx = 0.0;
            Double Sxy = 0.0;
            Double Syy = 0.0;

            foreach ((Double X, Double Y) Point in centers)
            {
                Double Dx = Point.X - Origin.X;
                Double Dy = Point.Y - Origin.Y;
                Sxx += Dx * Dx;
                Sxy += Dx * Dy;
                Syy += Dy * Dy;
            }

            Double Tr = Sxx + Syy;
            Double Det = (Sxx * Syy) - (Sxy * Sxy);
            Double Disc = Math.Max(0.0, ((Tr * 0.5) * (Tr * 0.5)) - Det);
            Double Lambda = (Tr * 0.5) + Math.Sqrt(Disc);
            Double Ex = Sxy;
            Double Ey = Lambda - Sxx;

            if ((Math.Abs(Ex) < EPSILON) && (Math.Abs(Ey) < EPSILON))
            {
                Ex = Lambda - Syy;
                Ey = Sxy;
            }
            if ((Math.Abs(Ex) < EPSILON) && (Math.Abs(Ey) < EPSILON))
            {
                return 0.0;
            }

            return Mod360(RadiansToDegrees(Math.Atan2(Ex, Ey)));
        }

        /// <summary>
        /// Smooths one contiguous run using a gaussian kernel over waypoint index.
        /// </summary>
        /// <param name="xys">Run points.</param>
        /// <param name="sigmaIdx">Sigma in waypoint indices.</param>
        /// <returns>Smoothed run with endpoints preserved.</returns>
        static private List<(Double X, Double Y)> SmoothRunXy
            (
            List<(Double X, Double Y)> xys,
            Double sigmaIdx
            )
        {
            // Smooth curvature while pinning endpoints so route junctions remain untouched.
            Int32 N = xys.Count;
            if ((N < 3) || (sigmaIdx <= 0.0))
            {
                return new List<(Double X, Double Y)>(xys);
            }

            Int32 Rad = Math.Min(Math.Max(1, (Int32)Math.Round(3.0 * sigmaIdx)), N - 1);
            List<Int32> Ks = new List<Int32>();
            for (Int32 k = -Rad; k <= Rad; k += 1)
            {
                Ks.Add(k);
            }

            List<Double> Weights = new List<Double>();
            Double WeightSum = 0.0;
            foreach (Int32 k in Ks)
            {
                Double Value = Math.Exp(-((Double)(k * k)) / (2.0 * sigmaIdx * sigmaIdx));
                Weights.Add(Value);
                WeightSum += Value;
            }
            for (Int32 i = 0; i < Weights.Count; i += 1)
            {
                Weights[i] = Weights[i] / WeightSum;
            }

            List<(Double X, Double Y)> Out = new List<(Double X, Double Y)>();
            for (Int32 i = 0; i < N; i += 1)
            {
                Double SumX = 0.0;
                Double SumY = 0.0;

                for (Int32 t = 0; t < Ks.Count; t += 1)
                {
                    Int32 j = i + Ks[t];
                    if (j < 0)
                    {
                        j = -j;
                    }
                    else if (j >= N)
                    {
                        j = (2 * (N - 1)) - j;
                    }

                    j = Math.Min(Math.Max(j, 0), N - 1);
                    SumX += Weights[t] * xys[j].X;
                    SumY += Weights[t] * xys[j].Y;
                }

                Out.Add((SumX, SumY));
            }

            Out[0] = xys[0];
            Out[Out.Count - 1] = xys[xys.Count - 1];
            return Out;
        }

        /// <summary>
        /// Smooths lane geometry per contiguous run while preserving depth sequence.
        /// </summary>
        /// <param name="lane">Raw lane points.</param>
        /// <param name="sigmaIdx">Smoothing sigma in waypoint indices.</param>
        /// <param name="hopThresh">Threshold that splits runs at reposition jumps.</param>
        /// <returns>Lane with smoothed coordinates and unchanged depth values.</returns>
        static private List<(Double X, Double Y, Double DepthM)> SmoothLane
            (
            List<(Double X, Double Y, Double DepthM)> lane,
            Double sigmaIdx,
            Double hopThresh
            )
        {
            // Never smooth across pocket-to-pocket jumps so steering does not bridge empty ground.
            if ((sigmaIdx <= 0.0) || (lane.Count < 3))
            {
                return lane;
            }

            List<List<Int32>> Runs = new List<List<Int32>>();
            List<Int32> Current = new List<Int32> { 0 };

            for (Int32 i = 1; i < lane.Count; i += 1)
            {
                Double Step = Dist((lane[i - 1].X, lane[i - 1].Y), (lane[i].X, lane[i].Y));
                if (Step > hopThresh)
                {
                    Runs.Add(Current);
                    Current = new List<Int32> { i };
                }
                else
                {
                    Current.Add(i);
                }
            }
            Runs.Add(Current);

            List<(Double X, Double Y, Double DepthM)> Out = new List<(Double X, Double Y, Double DepthM)>(lane);
            foreach (List<Int32> Run in Runs)
            {
                if (Run.Count < 3)
                {
                    continue;
                }

                List<(Double X, Double Y)> RunXy = new List<(Double X, Double Y)>();
                foreach (Int32 Index in Run)
                {
                    RunXy.Add((lane[Index].X, lane[Index].Y));
                }
                List<(Double X, Double Y)> Smoothed = SmoothRunXy(RunXy, sigmaIdx);

                for (Int32 k = 0; k < Run.Count; k += 1)
                {
                    Int32 Index = Run[k];
                    Out[Index] = (Smoothed[k].X, Smoothed[k].Y, lane[Index].DepthM);
                }
            }

            return Out;
        }

        /// <summary>
        /// Circumradius of triangle ABC in metres; infinity when nearly collinear.
        /// </summary>
        /// <param name="a">First vertex.</param>
        /// <param name="b">Second vertex.</param>
        /// <param name="c">Third vertex.</param>
        /// <returns>Circumradius in metres.</returns>
        static private Double CircumRadiusM
            (
            (Double X, Double Y) a,
            (Double X, Double Y) b,
            (Double X, Double Y) c
            )
        {
            Double La = Dist(b, c);
            Double Lb = Dist(a, c);
            Double Lc = Dist(a, b);
            Double Cross = Math.Abs((a.X * (b.Y - c.Y)) + (b.X * (c.Y - a.Y)) + (c.X * (a.Y - b.Y)));
            if (Cross < 1e-9)
            {
                return Double.PositiveInfinity;
            }

            // Shoelace magnitude is 2K; R = abc / (4K) = abc / (2 * Cross).
            return (La * Lb * Lc) / (2.0 * Cross);
        }

        /// <summary>
        /// Absolute heading change at b along a→b→c in degrees.
        /// </summary>
        /// <param name="a">Inbound start.</param>
        /// <param name="b">Corner.</param>
        /// <param name="c">Outbound end.</param>
        /// <returns>Absolute turn angle in degrees.</returns>
        static private Double HeadingChangeDeg
            (
            (Double X, Double Y) a,
            (Double X, Double Y) b,
            (Double X, Double Y) c
            )
        {
            Double Dx1 = b.X - a.X;
            Double Dy1 = b.Y - a.Y;
            Double Dx2 = c.X - b.X;
            Double Dy2 = c.Y - b.Y;
            return Math.Abs(RadiansToDegrees(Math.Atan2((Dx1 * Dy2) - (Dy1 * Dx2), (Dx1 * Dx2) + (Dy1 * Dy2))));
        }

        /// <summary>
        /// Converts compass bearing (0=N, 90=E) to math angle CCW from +X (east).
        /// </summary>
        /// <param name="headingDeg">Compass heading in degrees.</param>
        /// <returns>Math heading in radians.</returns>
        static private Double CompassToMath
            (
            Double headingDeg
            )
        {
            return (Math.PI / 2.0) - DegreesToRadians(headingDeg);
        }

        /// <summary>
        /// Wraps angle into [0, 2π) using floor (Andrew Walker Dubins convention).
        /// </summary>
        /// <param name="theta">Angle in radians.</param>
        /// <returns>Wrapped angle in [0, 2π).</returns>
        static private Double DubinsMod2Pi
            (
            Double theta
            )
        {
            return theta - (2.0 * Math.PI * Math.Floor(theta / (2.0 * Math.PI)));
        }

        /// <summary>
        /// Advances a normalized Dubins segment (L/R/S) from pose <paramref name="qi"/> by length <paramref name="t"/>.
        /// </summary>
        /// <param name="t">Normalized segment length.</param>
        /// <param name="qi">Start pose (x, y, heading) in unit-radius space.</param>
        /// <param name="typ">Segment type: L, R, or S.</param>
        /// <returns>Pose after the segment.</returns>
        static private (Double X, Double Y, Double Th) DubinsSegmentStep
            (
            Double t,
            (Double X, Double Y, Double Th) qi,
            String typ
            )
        {
            Double St = Math.Sin(qi.Th);
            Double Ct = Math.Cos(qi.Th);
            if (typ == "L")
            {
                return (qi.X + Math.Sin(qi.Th + t) - St, qi.Y - Math.Cos(qi.Th + t) + Ct, qi.Th + t);
            }

            if (typ == "R")
            {
                return (qi.X - Math.Sin(qi.Th - t) + St, qi.Y + Math.Cos(qi.Th - t) - Ct, qi.Th - t);
            }

            return (qi.X + (Ct * t), qi.Y + (St * t), qi.Th);
        }

        /// <summary>
        /// Segment types for each Dubins word (LSL … LRL).
        /// </summary>
        static private readonly Dictionary<String, String[]> DubinsDir = new Dictionary<String, String[]>
        {
            { "LSL", new[] { "L", "S", "L" } },
            { "LSR", new[] { "L", "S", "R" } },
            { "RSL", new[] { "R", "S", "L" } },
            { "RSR", new[] { "R", "S", "R" } },
            { "RLR", new[] { "R", "L", "R" } },
            { "LRL", new[] { "L", "R", "L" } },
        };

        /// <summary>
        /// Normalized segment lengths for each Dubins word (Andrew Walker formulas).
        /// </summary>
        /// <param name="alpha">Normalized start heading relative to SE line.</param>
        /// <param name="beta">Normalized end heading relative to SE line.</param>
        /// <param name="d">Normalized start–end distance.</param>
        /// <param name="sa">sin(alpha).</param>
        /// <param name="sb">sin(beta).</param>
        /// <param name="ca">cos(alpha).</param>
        /// <param name="cb">cos(beta).</param>
        /// <param name="cab">cos(alpha - beta).</param>
        /// <param name="dsq">d squared.</param>
        /// <returns>Word name → (t, p, q) normalized lengths for feasible words.</returns>
        static private Dictionary<String, (Double T, Double P, Double Q)> DubinsWords
            (
            Double alpha,
            Double beta,
            Double d,
            Double sa,
            Double sb,
            Double ca,
            Double cb,
            Double cab,
            Double dsq
            )
        {
            Dictionary<String, (Double T, Double P, Double Q)> Out = new Dictionary<String, (Double T, Double P, Double Q)>();
            Double Tmp0 = d + sa - sb;
            Double Psq = 2.0 + dsq - (2.0 * cab) + (2.0 * d * (sa - sb));
            if (Psq >= 0.0)
            {
                Double Tmp1 = Math.Atan2(cb - ca, Tmp0);
                Out["LSL"] = (DubinsMod2Pi(Tmp1 - alpha), Math.Sqrt(Psq), DubinsMod2Pi(beta - Tmp1));
            }

            Tmp0 = d - sa + sb;
            Psq = 2.0 + dsq - (2.0 * cab) + (2.0 * d * (sb - sa));
            if (Psq >= 0.0)
            {
                Double Tmp1 = Math.Atan2(ca - cb, Tmp0);
                Out["RSR"] = (DubinsMod2Pi(alpha - Tmp1), Math.Sqrt(Psq), DubinsMod2Pi(Tmp1 - beta));
            }

            Psq = -2.0 + dsq + (2.0 * cab) + (2.0 * d * (sa + sb));
            if (Psq >= 0.0)
            {
                Double P = Math.Sqrt(Psq);
                Tmp0 = Math.Atan2(-ca - cb, d + sa + sb) - Math.Atan2(-2.0, P);
                Out["LSR"] = (DubinsMod2Pi(Tmp0 - alpha), P, DubinsMod2Pi(Tmp0 - DubinsMod2Pi(beta)));
            }

            Psq = -2.0 + dsq + (2.0 * cab) - (2.0 * d * (sa + sb));
            if (Psq >= 0.0)
            {
                Double P = Math.Sqrt(Psq);
                Tmp0 = Math.Atan2(ca + cb, d - sa - sb) - Math.Atan2(2.0, P);
                Out["RSL"] = (DubinsMod2Pi(alpha - Tmp0), P, DubinsMod2Pi(beta - Tmp0));
            }

            Tmp0 = (6.0 - dsq + (2.0 * cab) + (2.0 * d * (sa - sb))) / 8.0;
            Double Phi = Math.Atan2(ca - cb, d - sa + sb);
            if (Math.Abs(Tmp0) <= 1.0)
            {
                Double P = DubinsMod2Pi((2.0 * Math.PI) - Math.Acos(Tmp0));
                Double T = DubinsMod2Pi(alpha - Phi + DubinsMod2Pi(P / 2.0));
                Out["RLR"] = (T, P, DubinsMod2Pi(alpha - beta - T + DubinsMod2Pi(P)));
            }

            Tmp0 = (6.0 - dsq + (2.0 * cab) + (2.0 * d * (sb - sa))) / 8.0;
            Phi = Math.Atan2(ca - cb, d + sa - sb);
            if (Math.Abs(Tmp0) <= 1.0)
            {
                Double P = DubinsMod2Pi((2.0 * Math.PI) - Math.Acos(Tmp0));
                Double T = DubinsMod2Pi(-alpha - Phi + (P / 2.0));
                Out["LRL"] = (T, P, DubinsMod2Pi(DubinsMod2Pi(beta) - alpha - T + DubinsMod2Pi(P)));
            }

            return Out;
        }

        /// <summary>
        /// Samples a Dubins path; returns intermediate points (excludes exact start).
        /// </summary>
        /// <param name="q0">Start pose (world x, y, math heading).</param>
        /// <param name="rho">Turn radius in metres.</param>
        /// <param name="typ">Dubins word name.</param>
        /// <param name="parameters">Normalized (t, p, q) segment lengths.</param>
        /// <param name="stepM">Sample spacing in metres.</param>
        /// <returns>Sampled waypoints excluding the exact start.</returns>
        static private List<(Double X, Double Y)> DubinsSample
            (
            (Double X, Double Y, Double Th) q0,
            Double rho,
            String typ,
            (Double T, Double P, Double Q) parameters,
            Double stepM
            )
        {
            String[] Types = DubinsDir[typ];
            Double[] Params = { parameters.T, parameters.P, parameters.Q };
            Double Length = (parameters.T + parameters.P + parameters.Q) * rho;
            List<(Double X, Double Y)> Pts = new List<(Double X, Double Y)>();
            if (Length < 1e-9)
            {
                return Pts;
            }

            // Dense enough that sampled triples on arc/straight junctions stay near R.
            Double Step = Math.Min(Math.Max(stepM, 1e-6), Math.Max(rho / 12.0, 0.25));
            Int32 N = Math.Max(24, (Int32)Math.Ceiling(Length / Math.Max(Step, 1e-6)));
            (Double X, Double Y, Double Th) Qi = (0.0, 0.0, q0.Th);
            for (Int32 I = 1; I <= N; I += 1)
            {
                Double Tp = (Length * (I / (Double)N)) / rho;
                (Double X, Double Y, Double Th) Q = Qi;
                Double Rem = Tp;
                for (Int32 Seg = 0; Seg < 3; Seg += 1)
                {
                    Double P = Params[Seg];
                    String Name = Types[Seg];
                    // Consume whole segments until the sample falls inside one.
                    if (Rem <= (P + 1e-12))
                    {
                        Q = DubinsSegmentStep(Rem, Q, Name);
                        break;
                    }

                    Q = DubinsSegmentStep(P, Q, Name);
                    Rem -= P;
                }

                Pts.Add((Q.X * rho + q0.X, Q.Y * rho + q0.Y));
            }

            return Pts;
        }

        /// <summary>
        /// Shortest Dubins path (Walker); optional CCC-only or CSC-only word filters.
        /// </summary>
        /// <param name="sx">Start easting.</param>
        /// <param name="sy">Start northing.</param>
        /// <param name="stheta">Start math heading.</param>
        /// <param name="ex">End easting.</param>
        /// <param name="ey">End northing.</param>
        /// <param name="etheta">End math heading.</param>
        /// <param name="radius">Turn radius in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="cccOnly">When true, only LRL/RLR words are considered.</param>
        /// <param name="cscOnly">When true (and not <paramref name="cccOnly"/>), only same-side CSC (LSL/RSR).</param>
        /// <param name="cscFamilyOnly">When true (and not CCC/same-side-only), all four CSC words — no CCC bulbs.</param>
        /// <param name="field">Optional field mask; when set, prefer paths that stay in-field.</param>
        /// <param name="headingInDeg">Inbound compass heading for CCC forward-excursion tiebreak.</param>
        /// <returns>Intermediate waypoints, empty list when radius is tiny, or null when no word applies.</returns>
        static private List<(Double X, Double Y)>? DubinsShortest
            (
            Double sx,
            Double sy,
            Double stheta,
            Double ex,
            Double ey,
            Double etheta,
            Double radius,
            Double sampleM,
            Boolean cccOnly = false,
            Boolean cscOnly = false,
            Grid? field = null,
            Double? headingInDeg = null,
            Boolean cscFamilyOnly = false
            )
        {
            if (radius <= 1e-9)
            {
                return new List<(Double X, Double Y)>();
            }

            Double Dx = ex - sx;
            Double Dy = ey - sy;
            Double D = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Double DistN = D / radius;
            Double Theta = (DistN > 0.0) ? DubinsMod2Pi(Math.Atan2(Dy, Dx)) : 0.0;
            Double Alpha = DubinsMod2Pi(stheta - Theta);
            Double Beta = DubinsMod2Pi(etheta - Theta);
            Double Sa = Math.Sin(Alpha);
            Double Sb = Math.Sin(Beta);
            Double Ca = Math.Cos(Alpha);
            Double Cb = Math.Cos(Beta);
            Double Cab = Math.Cos(Alpha - Beta);
            Dictionary<String, (Double T, Double P, Double Q)> Words =
                DubinsWords(Alpha, Beta, DistN, Sa, Sb, Ca, Cb, Cab, DistN * DistN);
            if (cccOnly)
            {
                // Intent: agricultural keyhole uses only CCC (LRL/RLR), not CSC side loops.
                Dictionary<String, (Double T, Double P, Double Q)> Filtered =
                    new Dictionary<String, (Double T, Double P, Double Q)>();
                foreach (KeyValuePair<String, (Double T, Double P, Double Q)> Kv in Words)
                {
                    if ((Kv.Key == "LRL") || (Kv.Key == "RLR"))
                    {
                        Filtered[Kv.Key] = Kv.Value;
                    }
                }

                Words = Filtered;
            }
            else if (cscOnly)
            {
                // Intent: soft pinches use same-side CSC (LSL/RSR) only — LSR/RSL can loop ~2πR.
                Dictionary<String, (Double T, Double P, Double Q)> Filtered =
                    new Dictionary<String, (Double T, Double P, Double Q)>();
                foreach (KeyValuePair<String, (Double T, Double P, Double Q)> Kv in Words)
                {
                    if ((Kv.Key == "LSL") || (Kv.Key == "RSR"))
                    {
                        Filtered[Kv.Key] = Kv.Value;
                    }
                }

                Words = Filtered;
            }
            else if (cscFamilyOnly)
            {
                // Intent: lane-change transit needs LSR/RSL but must not pick a CCC bulb.
                Dictionary<String, (Double T, Double P, Double Q)> Filtered =
                    new Dictionary<String, (Double T, Double P, Double Q)>();
                foreach (KeyValuePair<String, (Double T, Double P, Double Q)> Kv in Words)
                {
                    if ((Kv.Key == "LSL") || (Kv.Key == "RSR")
                        || (Kv.Key == "LSR") || (Kv.Key == "RSL"))
                    {
                        Filtered[Kv.Key] = Kv.Value;
                    }
                }

                Words = Filtered;
            }

            if (Words.Count == 0)
            {
                return null;
            }

            // Intent: if the shortest word leaves the field, flip to the other chirality.
            List<(Double X, Double Y)>? BestPts = null;
            Int32 BestOutside = Int32.MaxValue;
            Double BestExcursion = Double.PositiveInfinity;
            Double BestLen = Double.PositiveInfinity;
            (Double X, Double Y)? Fwd = null;
            if (headingInDeg.HasValue)
            {
                Fwd = ForwardEn(headingInDeg.Value);
            }

            foreach (KeyValuePair<String, (Double T, Double P, Double Q)> Kv in Words)
            {
                Double Len = (Kv.Value.T + Kv.Value.P + Kv.Value.Q) * radius;
                List<(Double X, Double Y)> Pts = DubinsSample((sx, sy, stheta), radius, Kv.Key, Kv.Value, sampleM);
                if ((Pts.Count > 0) && (Dist(Pts[Pts.Count - 1], (ex, ey)) < 1e-6))
                {
                    Pts.RemoveAt(Pts.Count - 1);
                }

                Int32 Outside = 0;
                Double Excursion = 0.0;
                if (field != null)
                {
                    Double ClearM = Math.Max(0.4 * field.GridSizeM, 0.5);
                    foreach ((Double X, Double Y) P in Pts)
                    {
                        if (!PointInFieldWithClearance(field, P.X, P.Y, ClearM))
                        {
                            Outside += 1;
                        }

                        if (Fwd.HasValue)
                        {
                            Double Proj = ((P.X - sx) * Fwd.Value.X) + ((P.Y - sy) * Fwd.Value.Y);
                            if (Proj > Excursion)
                            {
                                Excursion = Proj;
                            }
                        }
                    }
                }
                else if (Fwd.HasValue)
                {
                    foreach ((Double X, Double Y) P in Pts)
                    {
                        Double Proj = ((P.X - sx) * Fwd.Value.X) + ((P.Y - sy) * Fwd.Value.Y);
                        if (Proj > Excursion)
                        {
                            Excursion = Proj;
                        }
                    }
                }

                Boolean Better = (Outside < BestOutside)
                    || ((Outside == BestOutside) && (Len < (BestLen - 1e-6)))
                    || ((Outside == BestOutside)
                        && (Math.Abs(Len - BestLen) <= 1e-6)
                        && (Excursion < BestExcursion));
                if (Better)
                {
                    BestOutside = Outside;
                    BestExcursion = Excursion;
                    BestLen = Len;
                    BestPts = Pts;
                }
            }

            return BestPts;
        }

        /// <summary>
        /// Shortest Dubins CSC path for non-keyhole soft bends.
        /// </summary>
        /// <param name="sx">Start easting.</param>
        /// <param name="sy">Start northing.</param>
        /// <param name="stheta">Start math heading.</param>
        /// <param name="ex">End easting.</param>
        /// <param name="ey">End northing.</param>
        /// <param name="etheta">End math heading.</param>
        /// <param name="radius">Turn radius in metres.</param>
        /// <returns>Intermediate waypoints, or null when no path exists.</returns>
        static private List<(Double X, Double Y)>? DubinsCsc
            (
            Double sx,
            Double sy,
            Double stheta,
            Double ex,
            Double ey,
            Double etheta,
            Double radius
            )
        {
            return DubinsShortest(
                sx, sy, stheta, ex, ey, etheta, radius, Math.Max(radius / 8.0, 0.5), false, true);
        }

        /// <summary>
        /// Polyline length in metres (including the start→first leg).
        /// </summary>
        /// <param name="start">Path start.</param>
        /// <param name="pts">Intermediate waypoints.</param>
        /// <returns>Total length in metres.</returns>
        static private Double PolylineLengthM
            (
            (Double X, Double Y) start,
            List<(Double X, Double Y)> pts
            )
        {
            Double Len = 0.0;
            (Double X, Double Y) Prev = start;
            foreach ((Double X, Double Y) P in pts)
            {
                Len += Dist(Prev, P);
                Prev = P;
            }

            return Len;
        }

        /// <summary>
        /// Classic constant-R corner fillet tangent to inbound and outbound headings.
        /// </summary>
        /// <param name="corner">Apex (coverage corner).</param>
        /// <param name="headingInDeg">Inbound compass heading.</param>
        /// <param name="end">Next coverage point (along outbound).</param>
        /// <param name="headingOutDeg">Outbound compass heading.</param>
        /// <param name="turnRadiusM">Turn radius in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="inboundLenM">Length available on the inbound leg (A→corner).</param>
        /// <param name="tangentIn">Inbound tangent point (caller should pin corner steering here).</param>
        /// <returns>MOVE samples from after tangentIn through the arc to end, or null if the fillet will not fit.</returns>
        static private List<(Double X, Double Y)>? ClassicCornerFillet
            (
            (Double X, Double Y) corner,
            Double headingInDeg,
            (Double X, Double Y) end,
            Double headingOutDeg,
            Double turnRadiusM,
            Double sampleM,
            Double inboundLenM,
            out (Double X, Double Y) tangentIn
            )
        {
            tangentIn = corner;
            (Double X, Double Y) U = ForwardEn(headingInDeg);
            (Double X, Double Y) V = ForwardEn(headingOutDeg);
            Double Cross = (U.X * V.Y) - (U.Y * V.X);
            Double Dot = Math.Max(-1.0, Math.Min(1.0, (U.X * V.X) + (U.Y * V.Y)));
            Double Phi = Math.Atan2(Cross, Dot);
            if (Math.Abs(Phi) < 1e-3)
            {
                return null;
            }

            // Intent: tangent length R·tan(φ/2); both legs must be long enough to host it.
            Double Half = Math.Abs(Phi) * 0.5;
            Double TanHalf = Math.Tan(Half);
            if (TanHalf < 1e-9)
            {
                return null;
            }

            Double TangLen = turnRadiusM * TanHalf;
            if ((inboundLenM < (TangLen - 1e-6)) || (Dist(corner, end) < (TangLen - 1e-6)))
            {
                return null;
            }

            tangentIn = (corner.X - (TangLen * U.X), corner.Y - (TangLen * U.Y));
            (Double X, Double Y) TangentOut = (corner.X + (TangLen * V.X), corner.Y + (TangLen * V.Y));
            Boolean Left = Phi > 0.0;
            Double Rx = Left ? -U.Y : U.Y;
            Double Ry = Left ? U.X : -U.X;
            (Double X, Double Y) Center = (
                tangentIn.X + (turnRadiusM * Rx),
                tangentIn.Y + (turnRadiusM * Ry));
            Double A0 = Math.Atan2(tangentIn.Y - Center.Y, tangentIn.X - Center.X);
            Double A1 = Math.Atan2(TangentOut.Y - Center.Y, TangentOut.X - Center.X);
            Double Sweep = A1 - A0;
            if (Left)
            {
                while (Sweep < 0.0)
                {
                    Sweep += 2.0 * Math.PI;
                }
            }
            else
            {
                while (Sweep > 0.0)
                {
                    Sweep -= 2.0 * Math.PI;
                }
            }

            Double ArcLen = Math.Abs(Sweep) * turnRadiusM;
            Double Step = Math.Max(sampleM, 1e-6);
            Int32 N = Math.Max(4, (Int32)Math.Ceiling(ArcLen / Step));
            List<(Double X, Double Y)> Pts = new List<(Double X, Double Y)>();
            for (Int32 I = 1; I <= N; I += 1)
            {
                Double T = I / (Double)N;
                Double A = A0 + (T * Sweep);
                Pts.Add((Center.X + (turnRadiusM * Math.Cos(A)), Center.Y + (turnRadiusM * Math.Sin(A))));
            }

            // TangentOut → end is along outbound heading (no kink).
            if (Dist(TangentOut, end) > 1e-6)
            {
                Pts.AddRange(SampleStraight(TangentOut, end, Step));
            }

            if ((Pts.Count > 0) && (Dist(Pts[Pts.Count - 1], end) < 1e-6))
            {
                Pts.RemoveAt(Pts.Count - 1);
            }

            return Pts;
        }

        /// <summary>
        /// Last-resort same-side CSC when a classic corner fillet will not fit.
        /// </summary>
        /// <param name="start">Start point.</param>
        /// <param name="headingInDeg">Inbound compass heading.</param>
        /// <param name="end">End point.</param>
        /// <param name="headingOutDeg">Outbound compass heading.</param>
        /// <param name="turnRadiusM">Turn radius in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="field">Optional field mask; prefer in-field chirality when set.</param>
        /// <returns>Intermediate MOVE waypoints, or empty when no CSC word applies.</returns>
        static private List<(Double X, Double Y)> SoftBendFillet
            (
            (Double X, Double Y) start,
            Double headingInDeg,
            (Double X, Double Y) end,
            Double headingOutDeg,
            Double turnRadiusM,
            Double sampleM,
            Grid? field = null
            )
        {
            // Intent: never SampleStraight here — a chord from the apex keeps the sharp bend.
            Double St = CompassToMath(headingInDeg);
            Double Et = CompassToMath(headingOutDeg);
            List<(Double X, Double Y)>? Pts = DubinsShortest(
                start.X, start.Y, St, end.X, end.Y, Et, turnRadiusM, sampleM, false, true, field, headingInDeg);
            if ((Pts != null) && (Pts.Count > 0))
            {
                return Pts;
            }

            return new List<(Double X, Double Y)>();
        }

        /// <summary>
        /// Samples a straight segment (excluding the start point).
        /// </summary>
        static private List<(Double X, Double Y)> SampleStraight
            (
            (Double X, Double Y) p0,
            (Double X, Double Y) p1,
            Double step
            )
        {
            Double D = Dist(p0, p1);
            List<(Double X, Double Y)> Out = new List<(Double X, Double Y)>();
            if (D < 1e-9)
            {
                return Out;
            }

            Int32 N = Math.Max(1, (Int32)Math.Ceiling(D / Math.Max(step, 1e-6)));
            for (Int32 i = 1; i <= N; i += 1)
            {
                Double T = i / (Double)N;
                Out.Add((p0.X + ((p1.X - p0.X) * T), p0.Y + ((p1.Y - p0.Y) * T)));
            }

            return Out;
        }

        /// <summary>
        /// Unit forward vector (east, north) for a compass heading.
        /// </summary>
        /// <param name="headingDeg">Compass heading in degrees.</param>
        /// <returns>Unit forward vector.</returns>
        static private (Double X, Double Y) ForwardEn
            (
            Double headingDeg
            )
        {
            Double Rad = DegreesToRadians(headingDeg);
            return (Math.Sin(Rad), Math.Cos(Rad));
        }

        /// <summary>
        /// Appends samples along A→B (excludes A; optionally includes B).
        /// </summary>
        /// <param name="outPts">Destination list.</param>
        /// <param name="a">Segment start (not appended).</param>
        /// <param name="b">Segment end.</param>
        /// <param name="sampleM">Nominal sample spacing in metres.</param>
        /// <param name="includeEnd">When true, appends <paramref name="b"/>.</param>
        static private void AppendLineSamples
            (
            List<(Double X, Double Y)> outPts,
            (Double X, Double Y) a,
            (Double X, Double Y) b,
            Double sampleM,
            Boolean includeEnd
            )
        {
            Double Len = Dist(a, b);
            if (Len < 1e-9)
            {
                if (includeEnd)
                {
                    outPts.Add(b);
                }

                return;
            }

            Double Step = Math.Max(sampleM, 1e-3);
            Int32 N = Math.Max(1, (Int32)Math.Ceiling(Len / Step));
            for (Int32 I = 1; I < N; I += 1)
            {
                Double T = (Double)I / (Double)N;
                outPts.Add((a.X + ((b.X - a.X) * T), a.Y + ((b.Y - a.Y) * T)));
            }

            if (includeEnd)
            {
                outPts.Add(b);
            }
        }

        /// <summary>
        /// Counts how many samples lie outside the field mask.
        /// </summary>
        /// <remarks>
        /// Uses a small clearance ring (~0.4 cell) so accepted headlands stay visibly inside
        /// the painted field, not merely on the last valid cell center.
        /// </remarks>
        /// <param name="pts">Waypoints to test.</param>
        /// <param name="field">Field mask; null means treat all as in-field.</param>
        /// <returns>Outside sample count.</returns>
        static private Int32 CountOutsideField
            (
            List<(Double X, Double Y)> pts,
            Grid? field
            )
        {
            if (field == null)
            {
                return 0;
            }

            // Intent: keep guidance strokes off the black void in rendered PNGs.
            Double ClearM = Math.Max(0.4 * field.GridSizeM, 0.5);
            Int32 Outside = 0;
            foreach ((Double X, Double Y) P in pts)
            {
                if (!PointInFieldWithClearance(field, P.X, P.Y, ClearM))
                {
                    Outside += 1;
                }
            }

            return Outside;
        }

        /// <summary>
        /// True when <paramref name="x"/>,<paramref name="y"/> and a clearance ring stay in-field.
        /// </summary>
        /// <param name="field">Field mask.</param>
        /// <param name="x">Easting in metres.</param>
        /// <param name="y">Northing in metres.</param>
        /// <param name="clearanceM">Ring radius in metres (0 = point-only).</param>
        /// <returns>True when the point is safely inside the mask.</returns>
        static private Boolean PointInFieldWithClearance
            (
            Grid field,
            Double x,
            Double y,
            Double clearanceM
            )
        {
            if (!field.ContainsWorldXy(x, y))
            {
                return false;
            }

            if (clearanceM <= 1e-9)
            {
                return true;
            }

            for (Int32 I = 0; I < 8; I += 1)
            {
                Double Ang = I * (Math.PI / 4.0);
                Double Px = x + (clearanceM * Math.Cos(Ang));
                Double Py = y + (clearanceM * Math.Sin(Ang));
                if (!field.ContainsWorldXy(Px, Py))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Builds a constant-R side-arc keyhole (spur + arc + reconnect) into preferred field.
        /// </summary>
        /// <remarks>
        /// Unlike tip Dubins CCC (which extends ~2–3R past the lane tip), a side arc of radius R
        /// only needs ~R of forward room and can loop left/right into available field — the
        /// shorter purple-style path when a symmetric bulb would leave the mask.
        /// </remarks>
        /// <param name="start">Coverage tip (inbound).</param>
        /// <param name="headingInDeg">Inbound compass heading.</param>
        /// <param name="end">Reverse-lane tip.</param>
        /// <param name="headingOutDeg">Outbound compass heading.</param>
        /// <param name="turnRadiusM">Turn radius in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="field">Optional field mask.</param>
        /// <returns>Best side-arc path, or null when none is feasible.</returns>
        static private List<(Double X, Double Y)>? GeometricSideKeyhole
            (
            (Double X, Double Y) start,
            Double headingInDeg,
            (Double X, Double Y) end,
            Double headingOutDeg,
            Double turnRadiusM,
            Double sampleM,
            Grid? field
            )
        {
            (Double X, Double Y) U = ForwardEn(headingInDeg);
            Double Step = Math.Max(sampleM / turnRadiusM, 0.08);
            Double MaxF = 1.5 * turnRadiusM;
            Double FStep = Math.Max(sampleM, turnRadiusM * 0.2);

            List<(Double X, Double Y)>? BestPts = null;
            Int32 BestOutside = Int32.MaxValue;
            Double BestLen = Double.PositiveInfinity;

            for (Int32 Side = -1; Side <= 1; Side += 2)
            {
                (Double X, Double Y) N = ((-U.Y) * Side, U.X * Side);
                for (Double F = 0.0; F <= (MaxF + 1e-9); F += FStep)
                {
                    (Double X, Double Y) Tip = (start.X + (F * U.X), start.Y + (F * U.Y));
                    Double ClearM = (field != null) ? Math.Max(0.4 * field.GridSizeM, 0.5) : 0.0;
                    if ((field != null) && !PointInFieldWithClearance(field, Tip.X, Tip.Y, ClearM))
                    {
                        break;
                    }

                    (Double X, Double Y) C = (Tip.X + (turnRadiusM * N.X), Tip.Y + (turnRadiusM * N.Y));
                    if ((field != null) && !PointInFieldWithClearance(field, C.X, C.Y, ClearM))
                    {
                        continue;
                    }

                    Double Ang0 = Math.Atan2(Tip.Y - C.Y, Tip.X - C.X);
                    Double Dir = (Side > 0) ? 1.0 : -1.0;
                    List<(Double X, Double Y)> Spur = new List<(Double X, Double Y)>();
                    if (F > 1e-9)
                    {
                        AppendLineSamples(Spur, start, Tip, sampleM, includeEnd: true);
                    }

                    List<(Double X, Double Y)>? BestArc = null;
                    Double BestArcScore = Double.PositiveInfinity;
                    Double BestArcHeading = headingOutDeg;
                    Double SweepMin = 0.85 * Math.PI;
                    Double SweepMax = 1.75 * Math.PI;
                    Int32 MaxK = (Int32)Math.Ceiling(SweepMax / Step) + 1;
                    for (Int32 K = 1; K <= MaxK; K += 1)
                    {
                        Double Sweep = K * Step;
                        if (Sweep < SweepMin)
                        {
                            continue;
                        }

                        if (Sweep > SweepMax)
                        {
                            break;
                        }

                        Double Ang = Ang0 + (Dir * Sweep);
                        (Double X, Double Y) P = (
                            C.X + (turnRadiusM * Math.Cos(Ang)),
                            C.Y + (turnRadiusM * Math.Sin(Ang)));
                        Double Tx = (-Math.Sin(Ang)) * Dir;
                        Double Ty = Math.Cos(Ang) * Dir;
                        // Intent: compass from math tangent (east, north) = atan2(east, north).
                        Double H = Math.Atan2(Tx, Ty) * (180.0 / Math.PI);
                        if (H < 0.0)
                        {
                            H += 360.0;
                        }

                        Double Dh = Math.Abs(((H - headingOutDeg + 180.0) % 360.0) - 180.0);
                        Double Dend = Dist(P, end);
                        if ((Dh > 25.0) || (Dend > (0.75 * turnRadiusM)))
                        {
                            continue;
                        }

                        // Intent: exit when nearly on outbound heading and close to the reverse tip.
                        Double Score = Dh + (8.0 * (Dend / Math.Max(turnRadiusM, 1e-3)));
                        if (Score < BestArcScore)
                        {
                            BestArcScore = Score;
                            BestArcHeading = H;
                            BestArc = new List<(Double X, Double Y)>(Spur);
                            for (Int32 J = 1; J <= K; J += 1)
                            {
                                Double Aj = Ang0 + (Dir * J * Step);
                                BestArc.Add((
                                    C.X + (turnRadiusM * Math.Cos(Aj)),
                                    C.Y + (turnRadiusM * Math.Sin(Aj))));
                            }
                        }
                    }

                    if ((BestArc == null) || (BestArc.Count == 0))
                    {
                        continue;
                    }

                    (Double X, Double Y) Exit = BestArc[BestArc.Count - 1];
                    List<(Double X, Double Y)> Full = new List<(Double X, Double Y)>(BestArc);
                    if (Dist(Exit, end) > 1e-3)
                    {
                        Double StConn = CompassToMath(BestArcHeading);
                        Double EtConn = CompassToMath(headingOutDeg);
                        List<(Double X, Double Y)>? Conn = DubinsShortest(
                            Exit.X, Exit.Y, StConn, end.X, end.Y, EtConn, turnRadiusM, sampleM,
                            false, false, field, BestArcHeading);
                        Double ConnCap = (Math.PI * turnRadiusM) + Dist(Exit, end);
                        if ((Conn == null) || (Conn.Count == 0)
                            || (PolylineLengthM(Exit, Conn) > ConnCap)
                            || (CountOutsideField(Conn, field) > 0)
                            || !PathRespectsMinRadius(Exit, Conn, turnRadiusM))
                        {
                            // Intent: field wins — try same-side CSC only before giving up on this arc.
                            Conn = DubinsShortest(
                                Exit.X, Exit.Y, StConn, end.X, end.Y, EtConn, turnRadiusM, sampleM,
                                false, true, field, BestArcHeading);
                            if ((Conn == null) || (Conn.Count == 0)
                                || (PolylineLengthM(Exit, Conn) > ConnCap)
                                || (CountOutsideField(Conn, field) > 0))
                            {
                                continue;
                            }
                        }

                        Full.AddRange(Conn);
                    }

                    if ((Full.Count > 0) && (Dist(Full[Full.Count - 1], end) < 1e-6))
                    {
                        Full.RemoveAt(Full.Count - 1);
                    }

                    Int32 Outside = CountOutsideField(Full, field);
                    Double Len = PolylineLengthM(start, Full);
                    Boolean Better = (Outside < BestOutside)
                        || ((Outside == BestOutside) && (Len < BestLen));
                    if (Better)
                    {
                        BestOutside = Outside;
                        BestLen = Len;
                        BestPts = Full;
                    }
                }
            }

            // Intent: field always wins — do not return a side arc that leaves the mask.
            if ((field != null) && (BestOutside > 0))
            {
                return null;
            }

            return BestPts;
        }

        /// <summary>
        /// Agricultural keyhole/bulb: prefer in-field side-arc, else forward-slid Dubins CCC.
        /// </summary>
        /// <remarks>
        /// Tip CCC bulbs often spill past the boundary or take the long way around. A constant-R
        /// side arc (with a short forward spur when needed) stays in available field and is usually
        /// shorter. Forward-slid CCC remains as a fallback when the side arc cannot reconnect.
        /// </remarks>
        /// <param name="start">Start point.</param>
        /// <param name="headingInDeg">Inbound compass heading.</param>
        /// <param name="end">End point.</param>
        /// <param name="headingOutDeg">Outbound compass heading.</param>
        /// <param name="turnRadiusM">Turn radius in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="field">Optional field mask; prefer in-field paths when set.</param>
        /// <returns>MOVE waypoints, or null when no CCC word applies.</returns>
        static private List<(Double X, Double Y)>? KeyholeUTurn
            (
            (Double X, Double Y) start,
            Double headingInDeg,
            (Double X, Double Y) end,
            Double headingOutDeg,
            Double turnRadiusM,
            Double sampleM,
            Grid? field = null
            )
        {
            Double St = CompassToMath(headingInDeg);
            Double Et = CompassToMath(headingOutDeg);
            (Double X, Double Y) Fwd = ForwardEn(headingInDeg);
            Double Step = Math.Max(sampleM, turnRadiusM * 0.25);
            Double MaxD = KEYHOLE_FORWARD_MAX_FRAC_R * turnRadiusM;

            List<(Double X, Double Y)>? BestPts = null;
            Int32 BestOutside = Int32.MaxValue;
            Double BestLen = Double.PositiveInfinity;
            Double BestD = Double.PositiveInfinity;

            void ConsiderPath
                (
                Double d,
                List<(Double X, Double Y)> full
                )
            {
                if ((full.Count > 0) && (Dist(full[full.Count - 1], end) < 1e-6))
                {
                    full = new List<(Double X, Double Y)>(full);
                    full.RemoveAt(full.Count - 1);
                }

                Int32 Outside = CountOutsideField(full, field);
                Double Len = PolylineLengthM(start, full);
                Boolean Better = (Outside < BestOutside)
                    || ((Outside == BestOutside) && (Len < (BestLen - 1e-9)))
                    || ((Outside == BestOutside)
                        && (Math.Abs(Len - BestLen) <= 1e-9)
                        && (d < BestD));
                if (Better)
                {
                    BestOutside = Outside;
                    BestD = d;
                    BestLen = Len;
                    BestPts = full;
                }
            }

            void ConsiderCcc
                (
                Double d,
                (Double X, Double Y) s2,
                (Double X, Double Y) e2
                )
            {
                // Intent: never seed a keyhole from an out-of-field tip (lineup clamp may leave d=0 outside).
                Double ClearM = (field != null) ? Math.Max(0.4 * field.GridSizeM, 0.5) : 0.0;
                if ((field != null)
                    && (!PointInFieldWithClearance(field, s2.X, s2.Y, ClearM)
                        || !PointInFieldWithClearance(field, e2.X, e2.Y, ClearM)))
                {
                    return;
                }

                List<(Double X, Double Y)>? Ccc = DubinsShortest(
                    s2.X, s2.Y, St, e2.X, e2.Y, Et, turnRadiusM, sampleM, true, false, field, headingInDeg);
                if ((Ccc == null) || (Ccc.Count == 0))
                {
                    return;
                }

                List<(Double X, Double Y)> Full = new List<(Double X, Double Y)>();
                if (Dist(start, s2) > 1e-9)
                {
                    AppendLineSamples(Full, start, s2, sampleM, includeEnd: true);
                }

                Full.AddRange(Ccc);
                if (Dist(e2, end) > 1e-9)
                {
                    if ((Full.Count == 0) || (Dist(Full[Full.Count - 1], e2) > 1e-6))
                    {
                        Full.Add(e2);
                    }

                    AppendLineSamples(Full, e2, end, sampleM, includeEnd: false);
                }

                ConsiderPath(d, Full);
            }

            // Intent: side-arc into available field first — tip CCC often spills past the boundary.
            List<(Double X, Double Y)>? Side = GeometricSideKeyhole(
                start, headingInDeg, end, headingOutDeg, turnRadiusM, sampleM, field);
            if ((Side != null) && (Side.Count > 0))
            {
                ConsiderPath(0.0, Side);
            }

            if (BestOutside > 0)
            {
                for (Double D = 0.0; D <= (MaxD + 1e-9); D += Step)
                {
                    (Double X, Double Y) S2 = (start.X + (D * Fwd.X), start.Y + (D * Fwd.Y));
                    ConsiderCcc(D, S2, end);
                    if (D > 1e-9)
                    {
                        (Double X, Double Y) E2 = (end.X + (D * Fwd.X), end.Y + (D * Fwd.Y));
                        ConsiderCcc(D, S2, E2);
                    }

                    if (BestOutside == 0)
                    {
                        break;
                    }
                }
            }

            // Intent: field always wins — never emit a keyhole that leaves the mask.
            if ((field != null) && (BestOutside > 0))
            {
                return null;
            }

            return BestPts;
        }

        /// <summary>
        /// True when every meaningful bend on start→pts respects the turn radius.
        /// </summary>
        /// <param name="start">Path start.</param>
        /// <param name="pts">Intermediate waypoints.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <returns>True when the polyline is steerable at R.</returns>
        static private Boolean PathRespectsMinRadius
            (
            (Double X, Double Y) start,
            List<(Double X, Double Y)> pts,
            Double turnRadiusM
            )
        {
            if (pts.Count < 2)
            {
                return true;
            }

            (Double X, Double Y) A = start;
            (Double X, Double Y) B = pts[0];
            for (Int32 I = 1; I < pts.Count; I += 1)
            {
                (Double X, Double Y) C = pts[I];
                if ((Dist(A, B) >= 1e-6) && (Dist(B, C) >= 1e-6)
                    && (HeadingChangeDeg(A, B, C) >= 15.0)
                    && (CircumRadiusM(A, B, C) < (turnRadiusM * 0.99)))
                {
                    return false;
                }

                A = B;
                B = C;
            }

            return true;
        }

        /// <summary>
        /// If corner K starts a tight serpentine U-turn, returns the reverse-lane start index and heading.
        /// </summary>
        /// <param name="run">Contiguous blade-down run.</param>
        /// <param name="cornerK">Interior corner index within the run.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <returns>End index and outbound heading, or null when no sustained reverse within 4R.</returns>
        static private (Int32 EndK, Double HoutDeg)? FindUturnExit
            (
            List<RawPoint> run,
            Int32 cornerK,
            Double turnRadiusM
            )
        {
            if ((cornerK < 1) || (cornerK >= (run.Count - 1)))
            {
                return null;
            }

            (Double X, Double Y) A = (run[cornerK - 1].Xs, run[cornerK - 1].Ys);
            (Double X, Double Y) B = (run[cornerK].Xs, run[cornerK].Ys);
            Double Hin = BearingDeg(A, B);
            (Double X, Double Y) U = ForwardEn(Hin);
            Double Along = 0.0;
            Double MaxAlong = 4.0 * turnRadiusM;
            Double MinRevAlong = UTURN_REVERSE_MIN_FRAC_R * turnRadiusM;
            for (Int32 M = cornerK; M < (run.Count - 1); M += 1)
            {
                (Double X, Double Y) P0 = (run[M].Xs, run[M].Ys);
                (Double X, Double Y) P1 = (run[M + 1].Xs, run[M + 1].Ys);
                if (M > cornerK)
                {
                    (Double X, Double Y) PrevPt = (run[M - 1].Xs, run[M - 1].Ys);
                    Along += Dist(PrevPt, P0);
                    if (Along > MaxAlong)
                    {
                        break;
                    }
                }

                (Double X, Double Y) V = ForwardEn(BearingDeg(P0, P1));
                // Intent: coalesce the whole reverse window (W), not only the next cell.
                if (((U.X * V.X) + (U.Y * V.Y)) >= -0.5)
                {
                    continue;
                }

                // Intent: reject one-cell zig-zags; accept 2+ reverse segments or one long reverse.
                Double RevAlong = Dist(P0, P1);
                Int32 RevSegs = 1;
                for (Int32 N = M + 1; N < (run.Count - 1); N += 1)
                {
                    (Double X, Double Y) Q0 = (run[N].Xs, run[N].Ys);
                    (Double X, Double Y) Q1 = (run[N + 1].Xs, run[N + 1].Ys);
                    (Double X, Double Y) W = ForwardEn(BearingDeg(Q0, Q1));
                    if (((U.X * W.X) + (U.Y * W.Y)) >= -0.5)
                    {
                        break;
                    }

                    RevAlong += Dist(Q0, Q1);
                    RevSegs += 1;
                }

                if ((RevSegs >= 2) || (RevAlong >= MinRevAlong))
                {
                    return (M, BearingDeg(P0, P1));
                }
            }

            return null;
        }

        /// <summary>
        /// Builds an R-bounded blade-up path from a start pose to an end pose.
        /// </summary>
        /// <param name="start">Start point (coverage corner).</param>
        /// <param name="headingInDeg">Inbound compass heading.</param>
        /// <param name="end">End point (next coverage point).</param>
        /// <param name="headingOutDeg">Outbound compass heading.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <param name="sampleM">Nominal sample spacing in metres.</param>
        /// <param name="field">Optional field mask; prefer in-field chirality when set.</param>
        /// <returns>Intermediate MOVE waypoints (excludes start; drops duplicate end).</returns>
        static private List<(Double X, Double Y)> HeadlandMovePoints
            (
            (Double X, Double Y) start,
            Double headingInDeg,
            (Double X, Double Y) end,
            Double headingOutDeg,
            Double turnRadiusM,
            Double sampleM,
            Grid? field = null
            )
        {
            (Double X, Double Y) U = ForwardEn(headingInDeg);
            (Double X, Double Y) V = ForwardEn(headingOutDeg);
            Double St = CompassToMath(headingInDeg);
            Double Et = CompassToMath(headingOutDeg);
            // Near-reverse => CCC keyhole bulb; slide forward when a tip bulb leaves the field.
            if (((U.X * V.X) + (U.Y * V.Y)) < -0.5)
            {
                List<(Double X, Double Y)>? Key = KeyholeUTurn(
                    start, headingInDeg, end, headingOutDeg, turnRadiusM, sampleM, field);
                if ((Key != null) && (Key.Count > 0)
                    && (CountOutsideField(Key, field) == 0))
                {
                    return Key;
                }

                // Intent: never collapse a U-turn to a straight chord — try any in-field Dubins word.
                List<(Double X, Double Y)>? Any = DubinsShortest(
                    start.X, start.Y, St, end.X, end.Y, Et, turnRadiusM, sampleM, false, false, field, headingInDeg);
                if ((Any != null) && (Any.Count > 0)
                    && (CountOutsideField(Any, field) == 0))
                {
                    return Any;
                }
            }

            // Soft bend: same-side CSC (LSL/RSR); SoftBendFillet is last resort.
            List<(Double X, Double Y)>? Pts = DubinsShortest(
                start.X, start.Y, St, end.X, end.Y, Et, turnRadiusM, sampleM, false, true, field, headingInDeg);
            if ((Pts != null) && (Pts.Count > 0))
            {
                Double Chord = Dist(start, end);
                Double Cap = (SOFT_BEND_MAX_HALF_TURNS * Math.PI * turnRadiusM) + Chord;
                if ((PolylineLengthM(start, Pts) <= Cap)
                    && (CountOutsideField(Pts, field) == 0))
                {
                    return Pts;
                }
            }

            List<(Double X, Double Y)> Fillet = SoftBendFillet(
                start, headingInDeg, end, headingOutDeg, turnRadiusM, sampleM, field);
            if (CountOutsideField(Fillet, field) == 0)
            {
                return Fillet;
            }

            // Intent: hard R still wins over clearance — accept point-in-field CSC when the
            // clearance ring rejected the only feasible soft bend (edge lanes).
            if ((Pts != null) && (Pts.Count > 0))
            {
                Double Chord = Dist(start, end);
                Double Cap = (SOFT_BEND_MAX_HALF_TURNS * Math.PI * turnRadiusM) + Chord;
                if ((PolylineLengthM(start, Pts) <= Cap)
                    && (CountOutsideFieldPointsOnly(Pts, field) == 0))
                {
                    return Pts;
                }
            }

            List<(Double X, Double Y)>? AnySoft = DubinsShortest(
                start.X, start.Y, St, end.X, end.Y, Et, turnRadiusM, sampleM,
                false, false, field, headingInDeg, cscFamilyOnly: true);
            if ((AnySoft != null) && (AnySoft.Count > 0)
                && (CountOutsideFieldPointsOnly(AnySoft, field) == 0))
            {
                Double Chord = Dist(start, end);
                Double Cap = (SOFT_BEND_MAX_HALF_TURNS * Math.PI * turnRadiusM) + Chord;
                if (PolylineLengthM(start, AnySoft) <= Cap)
                {
                    return AnySoft;
                }
            }

            // Intent: field wins over an out-of-field headland — caller may peel earlier.
            return new List<(Double X, Double Y)>();
        }

        /// <summary>
        /// Counts samples outside the field mask with no clearance ring (point-in-cell only).
        /// </summary>
        /// <param name="pts">Waypoints to test.</param>
        /// <param name="field">Field mask; null means treat all as in-field.</param>
        /// <returns>Outside sample count.</returns>
        static private Int32 CountOutsideFieldPointsOnly
            (
            List<(Double X, Double Y)> pts,
            Grid? field
            )
        {
            if (field == null)
            {
                return 0;
            }

            Int32 Outside = 0;
            foreach ((Double X, Double Y) P in pts)
            {
                if (!field.ContainsWorldXy(P.X, P.Y))
                {
                    Outside += 1;
                }
            }

            return Outside;
        }

        /// <summary>
        /// Available polyline length along <paramref name="run"/> from <paramref name="cornerK"/>
        /// walking <paramref name="dir"/> (no heading filter — stair-steps still count).
        /// </summary>
        /// <param name="run">Contiguous blade-down run.</param>
        /// <param name="cornerK">Corner index within the run.</param>
        /// <param name="dir">-1 = inbound (toward lower indices), +1 = outbound.</param>
        /// <param name="maxLenM">Stop after this many metres.</param>
        /// <returns>Path length in metres.</returns>
        static private Double RunPathLengthM
            (
            List<RawPoint> run,
            Int32 cornerK,
            Int32 dir,
            Double maxLenM
            )
        {
            Double Len = 0.0;
            if (dir < 0)
            {
                for (Int32 I = cornerK; I > 0; I -= 1)
                {
                    Len += Dist(
                        (run[I - 1].Xs, run[I - 1].Ys),
                        (run[I].Xs, run[I].Ys));
                    if (Len >= maxLenM)
                    {
                        return maxLenM;
                    }
                }
            }
            else
            {
                for (Int32 I = cornerK; I < (run.Count - 1); I += 1)
                {
                    Len += Dist(
                        (run[I].Xs, run[I].Ys),
                        (run[I + 1].Xs, run[I + 1].Ys));
                    if (Len >= maxLenM)
                    {
                        return maxLenM;
                    }
                }
            }

            return Len;
        }

        /// <summary>
        /// Available polyline length along <paramref name="run"/> from <paramref name="cornerK"/>
        /// walking <paramref name="dir"/> while segments stay aligned with <paramref name="headingDeg"/>.
        /// </summary>
        /// <param name="run">Contiguous blade-down run.</param>
        /// <param name="cornerK">Corner index within the run.</param>
        /// <param name="headingDeg">Compass heading that defines the corridor.</param>
        /// <param name="dir">-1 = inbound (toward lower indices), +1 = outbound.</param>
        /// <param name="maxLenM">Stop after this many metres.</param>
        /// <returns>Aligned length in metres.</returns>
        static private Double RunAlignedLengthM
            (
            List<RawPoint> run,
            Int32 cornerK,
            Double headingDeg,
            Int32 dir,
            Double maxLenM
            )
        {
            (Double X, Double Y) U = ForwardEn(headingDeg);
            Double Len = 0.0;
            if (dir < 0)
            {
                for (Int32 I = cornerK; I > 0; I -= 1)
                {
                    (Double X, Double Y) P0 = (run[I - 1].Xs, run[I - 1].Ys);
                    (Double X, Double Y) P1 = (run[I].Xs, run[I].Ys);
                    Double Seg = Dist(P0, P1);
                    if (Seg < 1e-9)
                    {
                        continue;
                    }

                    (Double X, Double Y) V = ((P1.X - P0.X) / Seg, (P1.Y - P0.Y) / Seg);
                    // Intent: stop when the path bends away from the inbound heading.
                    if (((V.X * U.X) + (V.Y * U.Y)) < 0.5)
                    {
                        break;
                    }

                    Len += Seg;
                    if (Len >= maxLenM)
                    {
                        return maxLenM;
                    }
                }
            }
            else
            {
                for (Int32 I = cornerK; I < (run.Count - 1); I += 1)
                {
                    (Double X, Double Y) P0 = (run[I].Xs, run[I].Ys);
                    (Double X, Double Y) P1 = (run[I + 1].Xs, run[I + 1].Ys);
                    Double Seg = Dist(P0, P1);
                    if (Seg < 1e-9)
                    {
                        continue;
                    }

                    (Double X, Double Y) V = ((P1.X - P0.X) / Seg, (P1.Y - P0.Y) / Seg);
                    if (((V.X * U.X) + (V.Y * U.Y)) < 0.5)
                    {
                        break;
                    }

                    Len += Seg;
                    if (Len >= maxLenM)
                    {
                        return maxLenM;
                    }
                }
            }

            return Len;
        }

        /// <summary>
        /// Walks outbound from <paramref name="cornerK"/> along the run for <paramref name="distM"/> metres.
        /// </summary>
        /// <param name="run">Contiguous blade-down run.</param>
        /// <param name="cornerK">Start index.</param>
        /// <param name="distM">Distance to walk.</param>
        /// <param name="endIndex">Last run index reached (inclusive lower bound of remaining coverage).</param>
        /// <returns>World point at that distance, or null when the run is too short.</returns>
        static private (Double X, Double Y)? PointAlongRunOutbound
            (
            List<RawPoint> run,
            Int32 cornerK,
            Double distM,
            out Int32 endIndex
            )
        {
            endIndex = cornerK;
            Double Remain = distM;
            (Double X, Double Y) Cur = (run[cornerK].Xs, run[cornerK].Ys);
            for (Int32 I = cornerK; I < (run.Count - 1); I += 1)
            {
                (Double X, Double Y) Nxt = (run[I + 1].Xs, run[I + 1].Ys);
                Double Seg = Dist(Cur, Nxt);
                if (Seg < 1e-9)
                {
                    Cur = Nxt;
                    endIndex = I + 1;
                    continue;
                }

                if (Remain <= Seg)
                {
                    Double T = Remain / Seg;
                    endIndex = I + 1;
                    return (Cur.X + (T * (Nxt.X - Cur.X)), Cur.Y + (T * (Nxt.Y - Cur.Y)));
                }

                Remain -= Seg;
                Cur = Nxt;
                endIndex = I + 1;
            }

            return null;
        }

        /// <summary>
        /// True when the bend at b is tighter than the turning radius.
        /// </summary>
        /// <remarks>
        /// Hard R bound applies to every real bend; <paramref name="turnThresholdDeg"/>
        /// only sets a noise floor (at least 15°) so micro-wiggles are ignored.
        /// </remarks>
        /// <param name="a">Incoming waypoint.</param>
        /// <param name="b">Apex waypoint.</param>
        /// <param name="c">Outgoing waypoint.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <param name="turnThresholdDeg">Caller turn threshold; noise floor is min(15°, this).</param>
        /// <returns>True when the apex must be softened.</returns>
        static private Boolean SteerViolatesMinRadius
            (
            (Double X, Double Y) a,
            (Double X, Double Y) b,
            (Double X, Double Y) c,
            Double turnRadiusM,
            Double turnThresholdDeg
            )
        {
            // Intent: 40–60° pinches with r≪R must still get headlands (route-7 W).
            Double NoiseFloor = Math.Min(15.0, turnThresholdDeg);
            if (HeadingChangeDeg(a, b, c) < NoiseFloor)
            {
                return false;
            }

            return CircumRadiusM(a, b, c) < (turnRadiusM * 0.99);
        }

        /// <summary>
        /// Inserts a straight blade-up lineup of equipment length before and after each CUT/FILL run.
        /// </summary>
        /// <remarks>
        /// The tractor+scraper train must be traveling straight along the run heading for
        /// approximately <paramref name="equipmentLengthM"/> before the blade engages and again
        /// after it lifts, so cutting/filling starts and finishes aligned. Arrival tips
        /// (START/RETURN/HAUL) near the run start are relocated to the entry lineup; exit
        /// inserts a MOVE spur past the run tip before the next travel/turn.
        /// </remarks>
        /// <param name="raw">Track after turn-radius enforcement.</param>
        /// <param name="equipmentLengthM">Required straight approach/exit length in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="turnRadiusM">Turn radius for reconnect headlands.</param>
        /// <param name="field">Field mask for in-field reconnect turns.</param>
        /// <returns>Track with lineup MOVE approaches and exits inserted.</returns>
        static private List<RawPoint> EnforceEquipmentLineup
            (
            List<RawPoint> raw,
            Double equipmentLengthM,
            Double sampleM,
            Double turnRadiusM,
            Grid field
            )
        {
            if ((equipmentLengthM <= 1e-6) || (raw.Count < 2))
            {
                return raw;
            }

            List<RawPoint> Out = new List<RawPoint>();
            Int32 I = 0;
            while (I < raw.Count)
            {
                RawPoint Cur = raw[I];
                Boolean IsCutOrFill = (Cur.Action == CUT) || (Cur.Action == FILL);
                Boolean PrevSame = (Out.Count > 0) && (Out[Out.Count - 1].Action == Cur.Action);
                Boolean RunStart = IsCutOrFill && !PrevSame && ((I + 1) < raw.Count);
                Boolean RunEnd = IsCutOrFill
                    && (((I + 1) >= raw.Count) || (raw[I + 1].Action != Cur.Action));
                // Intent: in-run MOVE headlands (U-turn keyholes) resume the same action —
                // those exits are owned by coalesce, not a cut→fill splice.
                Boolean InRunHeadland = false;
                if (RunEnd && ((I + 1) < raw.Count) && (raw[I + 1].Action == MOVE))
                {
                    for (Int32 J = I + 1; J < raw.Count; J += 1)
                    {
                        if (raw[J].Action == Cur.Action)
                        {
                            InRunHeadland = true;
                            break;
                        }

                        if (raw[J].Action != MOVE)
                        {
                            break;
                        }
                    }
                }

                if (RunStart)
                {
                    InsertEntryLineup(Out, raw, I, Cur, equipmentLengthM, sampleM, turnRadiusM, field);
                }

                Out.Add(Cur);

                if (RunEnd && !InRunHeadland)
                {
                    InsertExitLineup(Out, raw, I, Cur, equipmentLengthM, sampleM, field, turnRadiusM);
                    // Intent: drop tip-based HAUL loop and rebuild one path from the exit tip.
                    Int32 SkipTo = RebuildHaulAfterExit(
                        Out, raw, I, equipmentLengthM, sampleM, turnRadiusM, field);
                    if (SkipTo > I)
                    {
                        I = SkipTo;
                        continue;
                    }
                }

                I += 1;
            }

            return Out;
        }

        /// <summary>
        /// Rebuilds cut→fill HAUL from the exit tip, skipping the old tip-based transit.
        /// </summary>
        /// <param name="outPts">Track including exit spur (modified in place).</param>
        /// <param name="raw">Input track after turn-radius enforcement.</param>
        /// <param name="cutEndIndex">Index of the CUT tip in <paramref name="raw"/>.</param>
        /// <param name="equipmentLengthM">Equipment length in metres (unused; exit already applied).</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <param name="field">Field mask for in-field turns.</param>
        /// <returns>Next raw index to process (FILL start), or -1 if no HAUL rebuild.</returns>
        static private Int32 RebuildHaulAfterExit
            (
            List<RawPoint> outPts,
            List<RawPoint> raw,
            Int32 cutEndIndex,
            Double equipmentLengthM,
            Double sampleM,
            Double turnRadiusM,
            Grid field
            )
        {
            // Intent: exit length already applied by InsertExitLineup; keep signature parallel.
            if (equipmentLengthM < 0.0)
            {
                return -1;
            }

            Int32 J = cutEndIndex + 1;
            while ((J < raw.Count) && (raw[J].Action == MOVE))
            {
                J += 1;
            }

            if ((J >= raw.Count) || (raw[J].Action != HAUL))
            {
                return -1;
            }

            Int32 HaulStart = J;
            while ((J < raw.Count) && (raw[J].Action == HAUL))
            {
                J += 1;
            }

            if ((J >= raw.Count) || ((raw[J].Action != FILL) && (raw[J].Action != GRADE)))
            {
                return -1;
            }

            if (outPts.Count < 2)
            {
                return -1;
            }

            RawPoint ExitTip = outPts[outPts.Count - 1];
            RawPoint BeforeExit = outPts[outPts.Count - 2];
            (Double X, Double Y) ExitEnd = (ExitTip.Xs, ExitTip.Ys);
            (Double X, Double Y) Before = (BeforeExit.Xs, BeforeExit.Ys);
            if (Dist(Before, ExitEnd) < 1e-6)
            {
                return -1;
            }

            Double Hin = BearingDeg(Before, ExitEnd);
            RawPoint LastHaul = raw[J - 1];
            (Double X, Double Y) Pc = (LastHaul.Xs, LastHaul.Ys);
            RawPoint FillStart = raw[J];
            (Double X, Double Y) Pd = (FillStart.Xs, FillStart.Ys);
            // Intent: arrive aligned with the fill run; HAUL tip often sits on the fill start.
            Double Hout = Hin;
            if (((J + 1) < raw.Count) && (raw[J + 1].Action == FillStart.Action))
            {
                (Double X, Double Y) Pe = (raw[J + 1].Xs, raw[J + 1].Ys);
                if (Dist(Pd, Pe) > 1e-6)
                {
                    Hout = BearingDeg(Pd, Pe);
                }
            }
            else if (Dist(Pc, Pd) > 1e-6)
            {
                Hout = BearingDeg(Pc, Pd);
            }

            Double Chord = Dist(ExitEnd, Pc);
            (Double X, Double Y) U = ForwardEn(Hin);
            (Double X, Double Y) V = ForwardEn(Hout);
            Boolean NearReverse = ((U.X * V.X) + (U.Y * V.Y)) < -0.5;
            // Intent: cut→fill is usually a lane-change CSC; only allow CCC for short reverse hops.
            Boolean AllowKeyhole = NearReverse && (Chord < (4.0 * turnRadiusM));
            List<(Double X, Double Y)> Moves;
            if (AllowKeyhole)
            {
                Moves = HeadlandMovePoints(ExitEnd, Hin, Pc, Hout, turnRadiusM, sampleM, field);
            }
            else
            {
                Double St = CompassToMath(Hin);
                Double Et = CompassToMath(Hout);
                // Intent: short CSC family only (no CCC bulb); length cap rejects long loops.
                List<(Double X, Double Y)>? Soft = DubinsShortest(
                    ExitEnd.X, ExitEnd.Y, St, Pc.X, Pc.Y, Et, turnRadiusM, sampleM,
                    false, false, field, null, cscFamilyOnly: true);
                Moves = Soft ?? new List<(Double X, Double Y)>();
                // Intent: field wins — refuse a CSC that clips the mask; try a short in-field reconnect.
                if ((Moves.Count > 0) && (CountOutsideField(Moves, field) > 0))
                {
                    List<(Double X, Double Y)>? InField = MinCurveInFieldReconnect(
                        ExitEnd, Hin, Pc, Hout, turnRadiusM, sampleM, field);
                    Moves = InField ?? new List<(Double X, Double Y)>();
                }
            }

            // Intent: non-keyhole transit may bend once (~πR), never a 2+ turn CCC bulb.
            Double Cap = AllowKeyhole
                ? ((2.75 * Math.PI * turnRadiusM) + (KEYHOLE_SPUR_BUDGET_FRAC_R * turnRadiusM))
                : ((Math.PI * turnRadiusM) + Chord);
            if ((Moves.Count == 0) || (PolylineLengthM(ExitEnd, Moves) > Cap)
                || (CountOutsideField(Moves, field) > 0))
            {
                // Intent: drop the tip-based loop; leave one HAUL marker for later transit repair.
                outPts.Add(new RawPoint(
                    LastHaul.Xr, LastHaul.Yr, LastHaul.DepthM, HAUL, LastHaul.PassNo, Pc.X, Pc.Y));
                return J;
            }

            foreach ((Double X, Double Y) M in Moves)
            {
                outPts.Add(new RawPoint(M.X, M.Y, 0.0, HAUL, LastHaul.PassNo, M.X, M.Y));
            }

            if ((outPts.Count == 0) || (Dist((outPts[outPts.Count - 1].Xs, outPts[outPts.Count - 1].Ys), Pc) > 1e-3))
            {
                outPts.Add(new RawPoint(
                    LastHaul.Xr, LastHaul.Yr, LastHaul.DepthM, HAUL, LastHaul.PassNo, Pc.X, Pc.Y));
            }

            return J;
        }

        /// <summary>
        /// Longest in-field distance from <paramref name="from"/> along unit <paramref name="u"/> up to <paramref name="maxLenM"/>.
        /// </summary>
        /// <param name="from">Start point (assumed in-field or on the tip).</param>
        /// <param name="u">Unit direction (east, north).</param>
        /// <param name="maxLenM">Maximum length to probe.</param>
        /// <param name="field">Field mask; null treats the full length as free.</param>
        /// <returns>In-field length in metres (≥ 0).</returns>
        static private Double InFieldRayLengthM
            (
            (Double X, Double Y) from,
            (Double X, Double Y) u,
            Double maxLenM,
            Grid? field
            )
        {
            if (maxLenM <= 1e-9)
            {
                return 0.0;
            }

            if (field == null)
            {
                return maxLenM;
            }

            Double Step = Math.Max(0.25, maxLenM / 24.0);
            Double Ok = 0.0;
            for (Double T = Step; T <= (maxLenM + 1e-9); T += Step)
            {
                Double X = from.X + (T * u.X);
                Double Y = from.Y + (T * u.Y);
                if (!field.ContainsWorldXy(X, Y))
                {
                    break;
                }

                Ok = T;
            }

            (Double X, Double Y) Tip = (from.X + (maxLenM * u.X), from.Y + (maxLenM * u.Y));
            if (field.ContainsWorldXy(Tip.X, Tip.Y))
            {
                Ok = Math.Max(Ok, maxLenM);
            }

            return Ok;
        }

        /// <summary>
        /// Shortest in-field CSC reconnect (same-side preferred), capped near one soft bend.
        /// </summary>
        /// <remarks>
        /// Straight lineup is preferred when it stays in-field. When it cannot, this is the
        /// fallback: minimum curve at R that stays in the field — never a full-circle bulb.
        /// </remarks>
        /// <param name="start">Current tip.</param>
        /// <param name="headingInDeg">Inbound compass heading.</param>
        /// <param name="end">Lineup entry / exit target.</param>
        /// <param name="headingOutDeg">Outbound compass heading at <paramref name="end"/>.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="field">Field mask (required for in-field preference).</param>
        /// <returns>Intermediate waypoints, or null when no short in-field path exists.</returns>
        static private List<(Double X, Double Y)>? MinCurveInFieldReconnect
            (
            (Double X, Double Y) start,
            Double headingInDeg,
            (Double X, Double Y) end,
            Double headingOutDeg,
            Double turnRadiusM,
            Double sampleM,
            Grid field
            )
        {
            Double St = CompassToMath(headingInDeg);
            Double Et = CompassToMath(headingOutDeg);
            Double Chord = Dist(start, end);
            Double Cap = (Math.PI * turnRadiusM) + Chord;
            // Intent: same-side CSC first (gentlest); then any short CSC; reject paths that leave the field.
            List<(Double X, Double Y)>? Soft = DubinsShortest(
                start.X, start.Y, St, end.X, end.Y, Et, turnRadiusM, sampleM, false, true, field);
            if ((Soft != null) && (Soft.Count > 0) && (PolylineLengthM(start, Soft) <= Cap)
                && (CountOutsideField(Soft, field) == 0))
            {
                return Soft;
            }

            List<(Double X, Double Y)>? Any = DubinsShortest(
                start.X, start.Y, St, end.X, end.Y, Et, turnRadiusM, sampleM, false, false, field);
            if ((Any != null) && (Any.Count > 0) && (PolylineLengthM(start, Any) <= Cap)
                && (CountOutsideField(Any, field) == 0))
            {
                return Any;
            }

            return null;
        }

        /// <summary>
        /// Inserts the straight entry approach ending at the CUT/FILL run start.
        /// </summary>
        /// <remarks>
        /// Prefer a straight blade-up of length L in-field. If that spur would leave the field,
        /// clip to the longest in-field straight and reconnect with the minimum in-field R-curve —
        /// never leave the mask and never invent a full-circle approach just to force full L.
        /// </remarks>
        /// <param name="outPts">Points already emitted (modified in place).</param>
        /// <param name="raw">Full input track.</param>
        /// <param name="index">Index of the run-start point in <paramref name="raw"/>.</param>
        /// <param name="cur">Run-start raw point.</param>
        /// <param name="equipmentLengthM">Required straight length in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="turnRadiusM">Turn radius for reconnect headlands.</param>
        /// <param name="field">Field mask for in-field reconnect turns.</param>
        static private void InsertEntryLineup
            (
            List<RawPoint> outPts,
            List<RawPoint> raw,
            Int32 index,
            RawPoint cur,
            Double equipmentLengthM,
            Double sampleM,
            Double turnRadiusM,
            Grid field
            )
        {
            (Double X, Double Y) B = (cur.Xs, cur.Ys);
            (Double X, Double Y) C = (raw[index + 1].Xs, raw[index + 1].Ys);
            if (Dist(B, C) < 1e-6)
            {
                return;
            }

            Double Hout = BearingDeg(B, C);
            (Double X, Double Y) U = ForwardEn(Hout);
            // Intent: approach from behind the tip; clip to the longest in-field straight ≤ L.
            (Double X, Double Y) Back = (-U.X, -U.Y);
            Double StraightL = InFieldRayLengthM(B, Back, equipmentLengthM, field);
            if (StraightL <= 1e-6)
            {
                // Intent: no in-field straight room — leave existing approach (may be short).
                return;
            }

            (Double X, Double Y) A = (B.X + (StraightL * Back.X), B.Y + (StraightL * Back.Y));
            if (MeasureStraightApproachM(outPts, B, Hout) >= (StraightL * 0.95))
            {
                return;
            }

            if (outPts.Count > 0)
            {
                RawPoint Prev = outPts[outPts.Count - 1];
                (Double X, Double Y) P = (Prev.Xs, Prev.Ys);
                Double TipTol = Math.Max(2.0, 0.15 * equipmentLengthM);
                if (Dist(P, B) <= TipTol)
                {
                    // Intent: START/RETURN may slide back to the (in-field) lineup entry; a HAUL/MOVE
                    // tip already on the corridor must not jump to A (forces a 360° loop).
                    if ((Prev.Action != START) && (Prev.Action != RETURN))
                    {
                        return;
                    }

                    outPts[outPts.Count - 1] = new RawPoint(
                        Prev.Xr, Prev.Yr, Prev.DepthM, Prev.Action, Prev.PassNo, A.X, A.Y);
                }
                else if (Dist(P, A) > 1.0)
                {
                    Double Hin = Hout;
                    if (outPts.Count >= 2)
                    {
                        RawPoint Prev2 = outPts[outPts.Count - 2];
                        if (Dist((Prev2.Xs, Prev2.Ys), P) > 1e-6)
                        {
                            Hin = BearingDeg((Prev2.Xs, Prev2.Ys), P);
                        }
                    }

                    List<(Double X, Double Y)>? Conn = MinCurveInFieldReconnect(
                        P, Hin, A, Hout, turnRadiusM, sampleM, field);
                    if (Conn == null)
                    {
                        // Intent: cut corridor already feeds the tip — keep the shorter approach.
                        return;
                    }

                    foreach ((Double X, Double Y) M in Conn)
                    {
                        outPts.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, cur.PassNo, M.X, M.Y));
                    }

                    if ((outPts.Count == 0)
                        || (Dist((outPts[outPts.Count - 1].Xs, outPts[outPts.Count - 1].Ys), A) > 1e-3))
                    {
                        outPts.Add(new RawPoint(A.X, A.Y, 0.0, MOVE, cur.PassNo, A.X, A.Y));
                    }
                }
            }
            else
            {
                outPts.Add(new RawPoint(A.X, A.Y, 0.0, MOVE, cur.PassNo, A.X, A.Y));
            }

            List<(Double X, Double Y)> Spur = new List<(Double X, Double Y)>();
            AppendLineSamples(Spur, A, B, sampleM, includeEnd: false);
            foreach ((Double X, Double Y) M in Spur)
            {
                outPts.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, cur.PassNo, M.X, M.Y));
            }
        }

        /// <summary>
        /// Exit spur length: prefer full L, but never overshoot a same-corridor fill or leave the field.
        /// </summary>
        /// <param name="e">Blade-down run tip.</param>
        /// <param name="headingInDeg">Exit compass heading.</param>
        /// <param name="equipmentLengthM">Nominal equipment length in metres.</param>
        /// <param name="raw">Full input track (to locate the next FILL/GRADE).</param>
        /// <param name="index">Index of the run tip in <paramref name="raw"/>.</param>
        /// <param name="field">Field mask; null skips in-field clamping.</param>
        /// <returns>Clamped exit length in metres (≥ 0).</returns>
        static private Double ClampedExitLineupLengthM
            (
            (Double X, Double Y) e,
            Double headingInDeg,
            Double equipmentLengthM,
            List<RawPoint> raw,
            Int32 index,
            Grid? field
            )
        {
            Double Len = equipmentLengthM;
            (Double X, Double Y) U = ForwardEn(headingInDeg);

            // Intent: same-heading cut→fill ahead on the exit ray — stop short of the fill tip
            // so the gap is the approach; overshooting forces a ~360° return loop.
            Int32 J = index + 1;
            while ((J < raw.Count) && (raw[J].Action == MOVE))
            {
                J += 1;
            }

            while ((J < raw.Count) && (raw[J].Action == HAUL))
            {
                J += 1;
            }

            if ((J < raw.Count) && ((raw[J].Action == FILL) || (raw[J].Action == GRADE)))
            {
                (Double X, Double Y) Pd = (raw[J].Xs, raw[J].Ys);
                Double Along = ((Pd.X - e.X) * U.X) + ((Pd.Y - e.Y) * U.Y);
                Double Lateral = Math.Abs(((Pd.X - e.X) * (-U.Y)) + ((Pd.Y - e.Y) * U.X));
                Double Corridor = Math.Max(2.0, 0.5 * equipmentLengthM);
                if ((Along > 1e-3) && (Lateral <= Corridor))
                {
                    Len = Math.Min(Len, Math.Max(0.0, Along - 0.5));
                }
            }

            // Intent: field wins over straight L — clip to the longest in-field ray.
            Len = Math.Min(Len, InFieldRayLengthM(e, U, Len, field));
            return Len;
        }

        /// <summary>
        /// Inserts a blade-up exit past the end of a CUT/FILL run (straight in-field, else min curve).
        /// </summary>
        /// <param name="outPts">Points already emitted including the run tip (modified in place).</param>
        /// <param name="raw">Full input track (for measuring an exit already built by U-turn coalesce).</param>
        /// <param name="index">Index of the run-end point in <paramref name="raw"/>.</param>
        /// <param name="cur">Last CUT/FILL point of the run.</param>
        /// <param name="equipmentLengthM">Required straight exit length in metres.</param>
        /// <param name="sampleM">Sample spacing in metres.</param>
        /// <param name="field">Field mask — exit must stay inside.</param>
        /// <param name="turnRadiusM">Turn radius for a min-curve exit when straight L will not fit.</param>
        static private void InsertExitLineup
            (
            List<RawPoint> outPts,
            List<RawPoint> raw,
            Int32 index,
            RawPoint cur,
            Double equipmentLengthM,
            Double sampleM,
            Grid field,
            Double turnRadiusM = 0.0
            )
        {
            (Double X, Double Y) E = (cur.Xs, cur.Ys);
            // Intent: exit along the heading used to arrive at the tip (last in-run leg).
            Double Hin = 0.0;
            Boolean HaveHin = false;
            for (Int32 J = outPts.Count - 2; J >= 0; J -= 1)
            {
                RawPoint P = outPts[J];
                if (P.Action != cur.Action)
                {
                    break;
                }

                (Double X, Double Y) Q = (P.Xs, P.Ys);
                if (Dist(Q, E) > 1e-6)
                {
                    Hin = BearingDeg(Q, E);
                    HaveHin = true;
                    break;
                }
            }

            if (!HaveHin)
            {
                return;
            }

            Double ExitLen = ClampedExitLineupLengthM(E, Hin, equipmentLengthM, raw, index, field);
            (Double X, Double Y) U = ForwardEn(Hin);

            // Intent: when straight L will not fit in-field (and we are not stopping short of a
            // same-corridor fill), bend into available field with the minimum R-curve.
            if ((ExitLen < (0.5 * equipmentLengthM)) && (turnRadiusM > 1e-6)
                && (InFieldRayLengthM(E, U, equipmentLengthM, field) < (0.5 * equipmentLengthM)))
            {
                // Intent: target a pose ~L ahead along Hin if in-field, else a side offset into field.
                (Double X, Double Y) Ideal = (E.X + (equipmentLengthM * U.X), E.Y + (equipmentLengthM * U.Y));
                (Double X, Double Y) Target = Ideal;
                if (!field.ContainsWorldXy(Ideal.X, Ideal.Y))
                {
                    (Double X, Double Y) Left = (-U.Y, U.X);
                    (Double X, Double Y) Right = (U.Y, -U.X);
                    (Double X, Double Y) Best = E;
                    Double BestAlong = -1.0;
                    foreach ((Double X, Double Y) Side in new[] { Left, Right })
                    {
                        for (Double F = turnRadiusM; F <= (equipmentLengthM + 1e-9); F += Math.Max(sampleM, turnRadiusM * 0.25))
                        {
                            (Double X, Double Y) Cand = (
                                E.X + (F * U.X) + (turnRadiusM * Side.X),
                                E.Y + (F * U.Y) + (turnRadiusM * Side.Y));
                            if (!field.ContainsWorldXy(Cand.X, Cand.Y))
                            {
                                continue;
                            }

                            if (F > BestAlong)
                            {
                                BestAlong = F;
                                Best = Cand;
                            }
                        }
                    }

                    if (BestAlong < 0.0)
                    {
                        // Intent: no in-field curved target — emit whatever short straight remains.
                    }
                    else
                    {
                        Target = Best;
                    }
                }

                if (Dist(E, Target) > 1e-3)
                {
                    List<(Double X, Double Y)>? Curve = MinCurveInFieldReconnect(
                        E, Hin, Target, Hin, turnRadiusM, sampleM, field);
                    if ((Curve != null) && (Curve.Count > 0))
                    {
                        foreach ((Double X, Double Y) M in Curve)
                        {
                            outPts.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, cur.PassNo, M.X, M.Y));
                        }

                        if (Dist((outPts[outPts.Count - 1].Xs, outPts[outPts.Count - 1].Ys), Target) > 1e-3)
                        {
                            outPts.Add(new RawPoint(
                                Target.X, Target.Y, 0.0, MOVE, cur.PassNo, Target.X, Target.Y));
                        }

                        return;
                    }
                }
            }

            if (ExitLen <= 1e-6)
            {
                return;
            }

            (Double X, Double Y) ExitEnd = (E.X + (ExitLen * U.X), E.Y + (ExitLen * U.Y));
            // Intent: U-turn coalesce may already have emitted the exit spur as MOVE after E.
            if (MeasureStraightExitAheadM(raw, index, E, Hin) >= (ExitLen * 0.95))
            {
                // Intent: copy that spur so Out ends at ExitEnd before any HAUL rebuild.
                (Double X, Double Y) CurXy = E;
                Double Along = 0.0;
                for (Int32 I = index + 1; I < raw.Count; I += 1)
                {
                    RawPoint P = raw[I];
                    if (P.Action != MOVE)
                    {
                        break;
                    }

                    (Double X, Double Y) Q = (P.Xs, P.Ys);
                    (Double X, Double Y) Dlt = (Q.X - CurXy.X, Q.Y - CurXy.Y);
                    Double Seg = Math.Sqrt((Dlt.X * Dlt.X) + (Dlt.Y * Dlt.Y));
                    if (Seg < 1e-9)
                    {
                        CurXy = Q;
                        continue;
                    }

                    Double Dot = ((Dlt.X / Seg) * U.X) + ((Dlt.Y / Seg) * U.Y);
                    if (Dot < 0.95)
                    {
                        break;
                    }

                    outPts.Add(P);
                    Along += Seg;
                    CurXy = Q;
                    if (Along >= (ExitLen * 0.95))
                    {
                        break;
                    }
                }

                return;
            }

            List<(Double X, Double Y)> Spur = new List<(Double X, Double Y)>();
            AppendLineSamples(Spur, E, ExitEnd, sampleM, includeEnd: true);
            foreach ((Double X, Double Y) M in Spur)
            {
                outPts.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, cur.PassNo, M.X, M.Y));
            }
        }

        /// <summary>
        /// Length of an existing straight blade-up exit after a run tip in the input track.
        /// </summary>
        /// <param name="raw">Full input track.</param>
        /// <param name="index">Index of the run-end point.</param>
        /// <param name="e">Blade-down run end.</param>
        /// <param name="headingDeg">Exit compass heading.</param>
        /// <returns>Along-track exit length in metres.</returns>
        static private Double MeasureStraightExitAheadM
            (
            List<RawPoint> raw,
            Int32 index,
            (Double X, Double Y) e,
            Double headingDeg
            )
        {
            (Double X, Double Y) U = ForwardEn(headingDeg);
            Double Along = 0.0;
            (Double X, Double Y) Cur = e;
            for (Int32 I = index + 1; I < raw.Count; I += 1)
            {
                RawPoint P = raw[I];
                if (P.Action != MOVE)
                {
                    break;
                }

                (Double X, Double Y) Q = (P.Xs, P.Ys);
                (Double X, Double Y) Dlt = (Q.X - Cur.X, Q.Y - Cur.Y);
                Double Seg = Math.Sqrt((Dlt.X * Dlt.X) + (Dlt.Y * Dlt.Y));
                if (Seg < 1e-9)
                {
                    Cur = Q;
                    continue;
                }

                Double Dot = ((Dlt.X / Seg) * U.X) + ((Dlt.Y / Seg) * U.Y);
                if (Dot < 0.95)
                {
                    break;
                }

                Along += Seg;
                Cur = Q;
            }

            return Along;
        }

        /// <summary>
        /// Length of an existing straight blade-up approach into B along heading H.
        /// </summary>
        /// <param name="outPts">Points already emitted (walked backward).</param>
        /// <param name="b">Blade-down run start.</param>
        /// <param name="headingDeg">Outbound compass heading of the run.</param>
        /// <returns>Along-track approach length in metres.</returns>
        static private Double MeasureStraightApproachM
            (
            List<RawPoint> outPts,
            (Double X, Double Y) b,
            Double headingDeg
            )
        {
            if (outPts.Count < 1)
            {
                return 0.0;
            }

            (Double X, Double Y) U = ForwardEn(headingDeg);
            Double Along = 0.0;
            (Double X, Double Y) Cur = b;
            for (Int32 I = outPts.Count - 1; I >= 0; I -= 1)
            {
                RawPoint P = outPts[I];
                if (!IsBladeUp(P.Action) && (P.Action != START))
                {
                    break;
                }

                (Double X, Double Y) Q = (P.Xs, P.Ys);
                (Double X, Double Y) Dlt = (Cur.X - Q.X, Cur.Y - Q.Y);
                Double Seg = Math.Sqrt((Dlt.X * Dlt.X) + (Dlt.Y * Dlt.Y));
                if (Seg < 1e-9)
                {
                    Cur = Q;
                    continue;
                }

                Double Dot = ((Dlt.X / Seg) * U.X) + ((Dlt.Y / Seg) * U.Y);
                // Intent: only count colinear approach from behind the tip.
                if (Dot < 0.95)
                {
                    break;
                }

                Along += Seg;
                Cur = Q;
            }

            return Along;
        }

        /// <summary>
        /// Inserts R-radius MOVE headlands wherever a blade-down corner is too sharp.
        /// </summary>
        /// <param name="raw">Assembled raw guidance records.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <param name="turnThresholdDeg">Minimum heading change treated as a turn.</param>
        /// <param name="sampleM">Arc sample spacing in metres.</param>
        /// <param name="field">Field mask used to flip out-of-field keyholes/CSC.</param>
        /// <param name="equipmentLengthM">Straight lineup before/after a coalesced U-turn (metres).</param>
        /// <returns>Track with MOVE arcs inserted; coverage raw coords preserved.</returns>
        static private List<RawPoint> EnforceMinTurnRadius
            (
            List<RawPoint> raw,
            Double turnRadiusM,
            Double turnThresholdDeg,
            Double sampleM,
            Grid field,
            Double equipmentLengthM = 0.0
            )
        {
            if ((turnRadiusM <= 0.0) || (raw.Count < 3))
            {
                return raw;
            }

            List<RawPoint> Out = new List<RawPoint>();
            Int32 I = 0;
            while (I < raw.Count)
            {
                RawPoint Cur0 = raw[I];
                if (!IsBladeDown(Cur0.Action))
                {
                    Out.Add(Cur0);
                    I += 1;
                    continue;
                }

                // Contiguous blade-down run of one action (CUT / FILL / GRADE).
                Int32 J = I;
                while ((J + 1 < raw.Count) && (raw[J + 1].Action == Cur0.Action))
                {
                    J += 1;
                }

                List<RawPoint> Run = raw.GetRange(I, (J - I) + 1);
                // Intent: check entry from prior blade-up (RETURN/MOVE/HAUL) into the first CUT.
                if ((Out.Count > 0) && (Run.Count >= 2))
                {
                    RawPoint PrevUp = Out[Out.Count - 1];
                    (Double X, Double Y) Pa0 = (PrevUp.Xs, PrevUp.Ys);
                    (Double X, Double Y) Pb0 = (Run[0].Xs, Run[0].Ys);
                    (Double X, Double Y) Pc0 = (Run[1].Xs, Run[1].Ys);
                    if (SteerViolatesMinRadius(Pa0, Pb0, Pc0, turnRadiusM, turnThresholdDeg))
                    {
                        Double Hin0 = BearingDeg(Pa0, Pb0);
                        Double Hout0 = BearingDeg(Pb0, Pc0);
                        Double St0 = CompassToMath(Hin0);
                        Double Et0 = CompassToMath(Hout0);
                        List<(Double X, Double Y)>? Entry = HeadlandMovePoints(
                            Pb0, Hin0, Pc0, Hout0, turnRadiusM, sampleM, field);
                        Double Cap0 = 2.75 * Math.PI * turnRadiusM;
                        if ((Entry == null) || (Entry.Count == 0) || (PolylineLengthM(Pb0, Entry) > Cap0))
                        {
                            Entry = DubinsShortest(
                                Pb0.X, Pb0.Y, St0, Pc0.X, Pc0.Y, Et0, turnRadiusM, sampleM, false, false, field);
                        }

                        Out.Add(Run[0]);
                        if ((Entry != null) && (Entry.Count > 0) && (PolylineLengthM(Pb0, Entry) <= Cap0))
                        {
                            foreach ((Double X, Double Y) M in Entry)
                            {
                                Out.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, Run[0].PassNo, M.X, M.Y));
                            }
                        }
                    }
                    else
                    {
                        Out.Add(Run[0]);
                    }
                }
                else
                {
                    Out.Add(Run[0]);
                }

                HashSet<Int32> SkipTurn = new HashSet<Int32>();
                HashSet<Int32> CoalesceEmitted = new HashSet<Int32>();
                for (Int32 K = 1; K < (Run.Count - 1); K += 1)
                {
                    RawPoint Prev = Run[K - 1];
                    RawPoint Cur = Run[K];
                    RawPoint Nxt = Run[K + 1];
                    (Double X, Double Y) A = (Prev.Xs, Prev.Ys);
                    (Double X, Double Y) B = (Cur.Xs, Cur.Ys);
                    (Double X, Double Y) C = (Nxt.Xs, Nxt.Ys);
                    if (SkipTurn.Contains(K))
                    {
                        // EndK still needs a real coverage/steering vertex; mid cells were pinned already.
                        if (!CoalesceEmitted.Contains(K))
                        {
                            Out.Add(Cur);
                        }

                        continue;
                    }

                    Out.Add(Cur);
                    if (!SteerViolatesMinRadius(A, B, C, turnRadiusM, turnThresholdDeg))
                    {
                        continue;
                    }

                    Double Hin = BearingDeg(A, B);
                    Double LocalTurnDeg = HeadingChangeDeg(A, B, C);
                    // Intent: soft pinches must not open a 4R reverse-window keyhole — only
                    // real doglegs/U-tips (>= 60°) may coalesce to a CCC bulb.
                    (Int32 EndK, Double HoutDeg)? Uturn = null;
                    if (LocalTurnDeg >= UTURN_COALESCE_MIN_HEADING_DEG)
                    {
                        Uturn = FindUturnExit(Run, K, turnRadiusM);
                    }

                    if (Uturn.HasValue)
                    {
                        // One readable keyhole for the full serpentine reverse.
                        (Double X, Double Y) EndPt = (Run[Uturn.Value.EndK].Xs, Run[Uturn.Value.EndK].Ys);
                        Double HoutU = Uturn.Value.HoutDeg;
                        (Double X, Double Y) Uin = ForwardEn(Hin);
                        (Double X, Double Y) Uout = ForwardEn(HoutU);
                        // Intent: leave the inbound tip straight for L, turn, then enter reverse straight for L.
                        (Double X, Double Y) KeyStart = B;
                        (Double X, Double Y) KeyEnd = EndPt;
                        if (equipmentLengthM > 1e-6)
                        {
                            // Intent: lineup spurs must stay in-field; a full L past the tip often exits the mask.
                            Double ExitL = InFieldRayLengthM(B, Uin, equipmentLengthM, field);
                            Double EntryL = InFieldRayLengthM(
                                EndPt, (-Uout.X, -Uout.Y), equipmentLengthM, field);
                            KeyStart = (B.X + (ExitL * Uin.X), B.Y + (ExitL * Uin.Y));
                            KeyEnd = (EndPt.X - (EntryL * Uout.X), EndPt.Y - (EntryL * Uout.Y));
                        }

                        List<(Double X, Double Y)> KeyMoves = HeadlandMovePoints(
                            KeyStart, Hin, KeyEnd, HoutU, turnRadiusM, sampleM, field);
                        // Intent: if lineup tips blocked an in-field keyhole, retry from the coverage tips.
                        if ((KeyMoves.Count == 0)
                            && ((Dist(KeyStart, B) > 1e-3) || (Dist(KeyEnd, EndPt) > 1e-3)))
                        {
                            KeyStart = B;
                            KeyEnd = EndPt;
                            KeyMoves = HeadlandMovePoints(
                                KeyStart, Hin, KeyEnd, HoutU, turnRadiusM, sampleM, field);
                        }

                        // Intent: never SkipTurn a reverse window if the keyhole builder produced nothing.
                        if ((KeyMoves.Count > 0) || (Dist(KeyStart, KeyEnd) < 1e-3))
                        {
                            if (Dist(B, KeyStart) > 1e-3)
                            {
                                List<(Double X, Double Y)> ExitSpur = new List<(Double X, Double Y)>();
                                AppendLineSamples(ExitSpur, B, KeyStart, sampleM, includeEnd: true);
                                foreach ((Double X, Double Y) M in ExitSpur)
                                {
                                    Out.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, Cur.PassNo, M.X, M.Y));
                                }
                            }

                            foreach ((Double X, Double Y) M in KeyMoves)
                            {
                                Out.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, Cur.PassNo, M.X, M.Y));
                            }

                            if (Dist(KeyEnd, EndPt) > 1e-3)
                            {
                                List<(Double X, Double Y)> Spur = new List<(Double X, Double Y)>();
                                AppendLineSamples(Spur, KeyEnd, EndPt, sampleM, includeEnd: false);
                                foreach ((Double X, Double Y) M in Spur)
                                {
                                    Out.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, Cur.PassNo, M.X, M.Y));
                                }
                            }

                            for (Int32 S = K + 1; S < Uturn.Value.EndK; S += 1)
                            {
                                RawPoint Mid = Run[S];
                                Out.Add(new RawPoint(
                                    Mid.Xr, Mid.Yr, Mid.DepthM, Mid.Action, Mid.PassNo, EndPt.X, EndPt.Y));
                                SkipTurn.Add(S);
                                CoalesceEmitted.Add(S);
                            }

                            SkipTurn.Add(Uturn.Value.EndK);
                            continue;
                        }
                    }

                    Double Hout = BearingDeg(B, C);
                    // Prefer a classic R fillet spanning enough inbound/outbound run (not just
                    // one cell). Short stair-steps otherwise leave undrivable blade-down corners.
                    (Double X, Double Y) UinF = ForwardEn(Hin);
                    (Double X, Double Y) UoutF = ForwardEn(Hout);
                    Double CrossF = (UinF.X * UoutF.Y) - (UinF.Y * UoutF.X);
                    Double DotF = Math.Max(-1.0, Math.Min(1.0, (UinF.X * UoutF.X) + (UinF.Y * UoutF.Y)));
                    Double PhiF = Math.Atan2(CrossF, DotF);
                    Double TangLen = 0.0;
                    if (Math.Abs(PhiF) >= 1e-3)
                    {
                        Double TanHalf = Math.Tan(Math.Abs(PhiF) * 0.5);
                        if (TanHalf > 1e-9)
                        {
                            TangLen = turnRadiusM * TanHalf;
                        }
                    }

                    Double InAvail = Math.Max(
                        RunAlignedLengthM(Run, K, Hin, -1, Math.Max(TangLen, Dist(A, B))),
                        RunPathLengthM(Run, K, -1, Math.Max(TangLen, turnRadiusM)));
                    Double OutAvail = Math.Max(
                        RunAlignedLengthM(Run, K, Hout, 1, Math.Max(TangLen, Dist(B, C))),
                        RunPathLengthM(Run, K, 1, Math.Max(TangLen, turnRadiusM)));
                    (Double X, Double Y) FilletEnd = C;
                    Int32 FilletEndIndex = K + 1;
                    if ((TangLen > 1e-6) && (OutAvail >= (TangLen - 1e-6)))
                    {
                        (Double X, Double Y)? Along = PointAlongRunOutbound(Run, K, TangLen, out FilletEndIndex);
                        if (Along.HasValue)
                        {
                            FilletEnd = Along.Value;
                        }
                    }

                    (Double X, Double Y) TangentIn;
                    List<(Double X, Double Y)>? Fillet = null;
                    if ((TangLen > 1e-6) && (InAvail >= (TangLen - 1e-6)) && (OutAvail >= (TangLen - 1e-6)))
                    {
                        Fillet = ClassicCornerFillet(
                            B, Hin, FilletEnd, Hout, turnRadiusM, sampleM, InAvail, out TangentIn);
                    }
                    else
                    {
                        TangentIn = B;
                        Fillet = ClassicCornerFillet(
                            B, Hin, C, Hout, turnRadiusM, sampleM, Dist(A, B), out TangentIn);
                    }

                    if ((Fillet != null) && (CountOutsideFieldPointsOnly(Fillet, field) == 0))
                    {
                        Int32 Last = Out.Count - 1;
                        RawPoint Apex = Out[Last];
                        Out[Last] = new RawPoint(
                            Apex.Xr, Apex.Yr, Apex.DepthM, Apex.Action, Apex.PassNo,
                            TangentIn.X, TangentIn.Y);
                        foreach ((Double X, Double Y) M in Fillet)
                        {
                            Out.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, Cur.PassNo, M.X, M.Y));
                        }

                        // Intent: skip coverage cells whose steering was replaced by the fillet arc.
                        for (Int32 S = K + 1; S < FilletEndIndex; S += 1)
                        {
                            RawPoint Mid = Run[S];
                            Out.Add(new RawPoint(
                                Mid.Xr, Mid.Yr, Mid.DepthM, Mid.Action, Mid.PassNo,
                                FilletEnd.X, FilletEnd.Y));
                            SkipTurn.Add(S);
                            CoalesceEmitted.Add(S);
                        }

                        SkipTurn.Add(FilletEndIndex);
                        continue;
                    }

                    // Soft / dogleg fallback. Never leave a sharp corner.
                    (Double X, Double Y) Ucap = ForwardEn(Hin);
                    (Double X, Double Y) Vcap = ForwardEn(Hout);
                    Boolean NearReverse = ((Ucap.X * Vcap.X) + (Ucap.Y * Vcap.Y)) < -0.5;
                    List<(Double X, Double Y)> Moves;
                    Double Cap;
                    if (NearReverse)
                    {
                        Moves = HeadlandMovePoints(
                            B, Hin, C, Hout, turnRadiusM, sampleM, field);
                        Cap = (2.75 * Math.PI * turnRadiusM)
                            + (KEYHOLE_SPUR_BUDGET_FRAC_R * turnRadiusM);
                    }
                    else
                    {
                        // Intent: soft 90° stairs stay CSC/fillet-capped — never a keyhole bulb.
                        Double SoftSt = CompassToMath(Hin);
                        Double SoftEt = CompassToMath(Hout);
                        Cap = (1.5 * Math.PI * turnRadiusM) + Dist(B, C) + turnRadiusM;
                        List<(Double X, Double Y)>? SoftCsc = DubinsShortest(
                            B.X, B.Y, SoftSt, C.X, C.Y, SoftEt, turnRadiusM, sampleM,
                            false, true, field, Hin);
                        if ((SoftCsc != null) && (SoftCsc.Count > 0)
                            && (PolylineLengthM(B, SoftCsc) <= Cap)
                            && (CountOutsideFieldPointsOnly(SoftCsc, field) == 0))
                        {
                            Moves = SoftCsc;
                        }
                        else
                        {
                            List<(Double X, Double Y)> FilletSoft0 = SoftBendFillet(
                                B, Hin, C, Hout, turnRadiusM, sampleM, field);
                            Moves = ((FilletSoft0.Count > 0)
                                    && (PolylineLengthM(B, FilletSoft0) <= Cap)
                                    && (CountOutsideFieldPointsOnly(FilletSoft0, field) == 0))
                                ? FilletSoft0
                                : HeadlandMovePoints(B, Hin, C, Hout, turnRadiusM, sampleM, field);
                        }
                    }

                    Int32 PeelEndIndex = K + 1;
                    (Double X, Double Y) PeelStart = B;
                    (Double X, Double Y) PeelEnd = C;
                    // Intent: short cell legs cannot host an R turn at the apex — peel earlier/later.
                    if (((Moves.Count == 0) || (PolylineLengthM(B, Moves) > Cap)
                            || (CountOutsideFieldPointsOnly(Moves, field) > 0))
                        && (Math.Max(TangLen, turnRadiusM) > 1e-6))
                    {
                        Double Need = Math.Max(TangLen, turnRadiusM);
                        Double PeelIn = Math.Min(InAvail, Need);
                        Double PeelOut = Math.Min(OutAvail, Need);
                        // Intent: even one-cell legs can peel along the inbound/outbound heading rays.
                        if ((PeelIn < (0.5 * turnRadiusM)) && (InAvail >= Dist(A, B)))
                        {
                            PeelIn = Math.Max(PeelIn, Math.Min(Need, Dist(A, B) + turnRadiusM));
                        }

                        if ((PeelOut < (0.5 * turnRadiusM)) && (OutAvail >= Dist(B, C)))
                        {
                            PeelOut = Math.Max(PeelOut, Math.Min(Need, Dist(B, C) + turnRadiusM));
                        }

                        if ((PeelIn >= 1e-3) && (PeelOut >= 1e-3))
                        {
                            // Intent: never peel outside the mask — clamp to in-field rays.
                            PeelIn = Math.Min(
                                PeelIn,
                                InFieldRayLengthM(B, (-UinF.X, -UinF.Y), PeelIn, field));
                            PeelOut = Math.Min(
                                PeelOut,
                                InFieldRayLengthM(B, UoutF, PeelOut, field));
                            if ((PeelIn < 1e-3) || (PeelOut < 1e-3))
                            {
                                // leave Moves as-is
                            }
                            else
                            {
                            PeelStart = (
                                B.X - (PeelIn * UinF.X),
                                B.Y - (PeelIn * UinF.Y));
                            PeelEnd = (
                                B.X + (PeelOut * UoutF.X),
                                B.Y + (PeelOut * UoutF.Y));
                            (Double X, Double Y)? AlongOut = PointAlongRunOutbound(
                                Run, K, PeelOut, out PeelEndIndex);
                            if (AlongOut.HasValue)
                            {
                                PeelEnd = AlongOut.Value;
                            }
                            else
                            {
                                PeelEndIndex = Math.Min(Run.Count - 1, K + 1);
                            }

                            if (field.ContainsWorldXy(PeelStart.X, PeelStart.Y)
                                && field.ContainsWorldXy(PeelEnd.X, PeelEnd.Y))
                            {
                                Int32 LastPin = Out.Count - 1;
                                RawPoint ApexPin = Out[LastPin];
                                Out[LastPin] = new RawPoint(
                                    ApexPin.Xr, ApexPin.Yr, ApexPin.DepthM, ApexPin.Action, ApexPin.PassNo,
                                    PeelStart.X, PeelStart.Y);
                                // Intent: peeled soft tips must not escalate into keyhole bulbs.
                                if (NearReverse)
                                {
                                    Moves = HeadlandMovePoints(
                                        PeelStart, Hin, PeelEnd, Hout, turnRadiusM, sampleM, field);
                                }
                                else
                                {
                                    Double SoftSt2 = CompassToMath(Hin);
                                    Double SoftEt2 = CompassToMath(Hout);
                                    List<(Double X, Double Y)>? Soft2 = DubinsShortest(
                                        PeelStart.X, PeelStart.Y, SoftSt2, PeelEnd.X, PeelEnd.Y, SoftEt2,
                                        turnRadiusM, sampleM, false, true, field, Hin);
                                    if ((Soft2 != null) && (Soft2.Count > 0)
                                        && (PolylineLengthM(PeelStart, Soft2) <= Cap)
                                        && (CountOutsideFieldPointsOnly(Soft2, field) == 0))
                                    {
                                        Moves = Soft2;
                                    }
                                    else
                                    {
                                        List<(Double X, Double Y)> FilletSoft = SoftBendFillet(
                                            PeelStart, Hin, PeelEnd, Hout, turnRadiusM, sampleM, field);
                                        // Intent: last resort — HeadlandMovePoints soft path (CSC only).
                                        Moves = ((FilletSoft.Count > 0)
                                                && (PolylineLengthM(PeelStart, FilletSoft) <= Cap)
                                                && (CountOutsideFieldPointsOnly(FilletSoft, field) == 0))
                                            ? FilletSoft
                                            : HeadlandMovePoints(
                                                PeelStart, Hin, PeelEnd, Hout, turnRadiusM, sampleM, field);
                                    }
                                }
                            }
                            }
                        }
                    }

                    if ((Moves.Count > 0) && (PolylineLengthM(PeelStart, Moves) <= Cap)
                        && (CountOutsideFieldPointsOnly(Moves, field) == 0))
                    {
                        foreach ((Double X, Double Y) M in Moves)
                        {
                            Out.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, Cur.PassNo, M.X, M.Y));
                        }

                        for (Int32 S = K + 1; S < PeelEndIndex; S += 1)
                        {
                            RawPoint Mid = Run[S];
                            Out.Add(new RawPoint(
                                Mid.Xr, Mid.Yr, Mid.DepthM, Mid.Action, Mid.PassNo,
                                PeelEnd.X, PeelEnd.Y));
                            SkipTurn.Add(S);
                            CoalesceEmitted.Add(S);
                        }

                        SkipTurn.Add(PeelEndIndex);
                    }
                }

                if (Run.Count > 1)
                {
                    Out.Add(Run[Run.Count - 1]);
                }

                I = J + 1;
            }

            // Intent: cut→fill HAUL and multi-drop MOVE chords need R-bounded transit turns.
            Out = EnforceBladeUpTransitTurns(Out, turnRadiusM, sampleM, field);
            // Intent: catch residual CUT/MOVE/CUT chords and soft pinches that still undercut R.
            return RepairSharpSteeringBends(Out, turnRadiusM, sampleM, field);
        }

        /// <summary>
        /// Replaces sharp blade-up transit chords (HAUL / multi-drop MOVE) with R-bounded arcs.
        /// </summary>
        /// <remarks>
        /// Loaded HAUL and multi-drop MOVE are assembled as straight chords to the next
        /// pocket start. Blade-down enforcement skips those junctions, so this pass inserts
        /// CSC exit/entry (or one CCC keyhole for a short near-reverse). Intermediates keep
        /// the original blade-up action for distance accounting.
        /// </remarks>
        /// <param name="raw">Track after blade-down headland insertion.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <param name="sampleM">Arc sample spacing in metres.</param>
        /// <param name="field">Field mask used to flip out-of-field transit turns.</param>
        /// <returns>Track with curved blade-up transit where needed.</returns>
        static private List<RawPoint> EnforceBladeUpTransitTurns
            (
            List<RawPoint> raw,
            Double turnRadiusM,
            Double sampleM,
            Grid field
            )
        {
            if ((raw.Count < 3) || (turnRadiusM <= 0.0))
            {
                return raw;
            }

            List<RawPoint> Out = new List<RawPoint>();
            for (Int32 I = 0; I < raw.Count; I += 1)
            {
                RawPoint Cur = raw[I];
                Boolean IsHaul = (Cur.Action == HAUL);
                Boolean IsMoveHop = (Cur.Action == MOVE);
                if ((!IsHaul && !IsMoveHop) || (Out.Count < 2) || ((I + 1) >= raw.Count))
                {
                    Out.Add(Cur);
                    continue;
                }

                RawPoint PrevEnd = Out[Out.Count - 1];
                RawPoint PrevPrev = Out[Out.Count - 2];
                RawPoint Next = raw[I + 1];
                // Intent: HAUL = cut→fill; MOVE hop = same blade-down action on both sides.
                Boolean HaulOk = IsHaul
                    && (PrevEnd.Action == CUT)
                    && ((Next.Action == FILL) || (Next.Action == GRADE));
                Boolean MoveHopOk = IsMoveHop
                    && IsBladeDown(PrevEnd.Action)
                    && (Next.Action == PrevEnd.Action);
                if (!HaulOk && !MoveHopOk)
                {
                    Out.Add(Cur);
                    continue;
                }

                (Double X, Double Y) Pa = (PrevPrev.Xs, PrevPrev.Ys);
                (Double X, Double Y) Pb = (PrevEnd.Xs, PrevEnd.Ys);
                (Double X, Double Y) Pc = (Cur.Xs, Cur.Ys);
                (Double X, Double Y) Pd = (Next.Xs, Next.Ys);
                // Intent: long transit legs make circumradius large at a sharp kink; use heading.
                Boolean ExitTurn = HeadingChangeDeg(Pa, Pb, Pc) >= 15.0;
                Boolean EntryTurn = HeadingChangeDeg(Pb, Pc, Pd) >= 15.0;
                if (!ExitTurn && !EntryTurn)
                {
                    Out.Add(Cur);
                    continue;
                }

                Double Hin = BearingDeg(Pa, Pb);
                Double Hout = BearingDeg(Pc, Pd);
                Double Chord = Dist(Pb, Pc);
                (Double X, Double Y) U = ForwardEn(Hin);
                (Double X, Double Y) V = ForwardEn(Hout);
                Boolean NearReverse = ((U.X * V.X) + (U.Y * V.Y)) < -0.5;
                // Intent: parallel-lane U-turn (route 11) → one keyhole; long haul → CSC.
                Boolean AllowKeyhole = NearReverse && (Chord < (4.0 * turnRadiusM));
                List<(Double X, Double Y)> Moves;
                if (AllowKeyhole)
                {
                    Moves = HeadlandMovePoints(Pb, Hin, Pc, Hout, turnRadiusM, sampleM, field);
                }
                else
                {
                    Double St = CompassToMath(Hin);
                    Double Et = CompassToMath(Hout);
                    // Intent: transit needs LSR/RSL; CCC allowed only when CSC cannot connect.
                    List<(Double X, Double Y)>? Soft = DubinsShortest(
                        Pb.X, Pb.Y, St, Pc.X, Pc.Y, Et, turnRadiusM, sampleM,
                        false, false, field, null, cscFamilyOnly: true);
                    if ((Soft == null) || (Soft.Count == 0))
                    {
                        Soft = DubinsShortest(
                            Pb.X, Pb.Y, St, Pc.X, Pc.Y, Et, turnRadiusM, sampleM, false, false, field);
                    }

                    Moves = Soft ?? new List<(Double X, Double Y)>();
                }

                Double Cap = AllowKeyhole
                    ? ((2.75 * Math.PI * turnRadiusM) + (KEYHOLE_SPUR_BUDGET_FRAC_R * turnRadiusM))
                    : ((2.75 * Math.PI * turnRadiusM) + Chord);
                if ((Moves.Count == 0) || (PolylineLengthM(Pb, Moves) > Cap))
                {
                    Out.Add(Cur);
                    continue;
                }

                String TransitAction = IsHaul ? HAUL : MOVE;
                foreach ((Double X, Double Y) M in Moves)
                {
                    Out.Add(new RawPoint(M.X, M.Y, 0.0, TransitAction, Cur.PassNo, M.X, M.Y));
                }

                Out.Add(Cur);
            }

            return Out;
        }

        /// <summary>
        /// Replaces residual sharp splices with short R-bounded arcs (no CCC cascade).
        /// </summary>
        /// <remarks>
        /// Blade-down apexes are kept (coverage floor); only blade-up residual
        /// junctions may drop the apex and splice A→C. Interior MOVE samples of an
        /// already-inserted headland are left alone.
        /// </remarks>
        /// <param name="raw">Track after the primary headland pass.</param>
        /// <param name="turnRadiusM">Minimum turn radius in metres.</param>
        /// <param name="sampleM">Arc sample spacing in metres.</param>
        /// <param name="field">Field mask used to flip out-of-field repair arcs.</param>
        /// <returns>Repaired track.</returns>
        static private List<RawPoint> RepairSharpSteeringBends
            (
            List<RawPoint> raw,
            Double turnRadiusM,
            Double sampleM,
            Grid field
            )
        {
            if ((raw.Count < 3) || (turnRadiusM <= 0.0))
            {
                return raw;
            }

            List<RawPoint> Cur = raw;
            // More sweeps: peel/fillet insertion can create new MOVE↔CUT junctions that need a second pass.
            for (Int32 Sweep = 0; Sweep < 4; Sweep += 1)
            {
                List<RawPoint> Next = new List<RawPoint> { Cur[0] };
                Boolean Changed = false;
                Int32 I = 1;
                while (I < (Cur.Count - 1))
                {
                    RawPoint A = Next[Next.Count - 1];
                    RawPoint B = Cur[I];
                    RawPoint C = Cur[I + 1];
                    (Double X, Double Y) Pa = (A.Xs, A.Ys);
                    (Double X, Double Y) Pb = (B.Xs, B.Ys);
                    (Double X, Double Y) Pc = (C.Xs, C.Ys);

                    // Intent: do not treat samples on an existing R-arc as new soft pinches.
                    if ((A.Action == MOVE) && (B.Action == MOVE) && (C.Action == MOVE))
                    {
                        Next.Add(B);
                        I += 1;
                        continue;
                    }

                    // Intent: same for curved HAUL transit samples (cut→fill).
                    if ((A.Action == HAUL) && (B.Action == HAUL) && (C.Action == HAUL))
                    {
                        Next.Add(B);
                        I += 1;
                        continue;
                    }

                    if (!SteerViolatesMinRadius(Pa, Pb, Pc, turnRadiusM, 15.0))
                    {
                        Next.Add(B);
                        I += 1;
                        continue;
                    }

                    Changed = true;
                    if (IsBladeDown(B.Action) && (A.Action == MOVE))
                    {
                        // Intent: keyhole/MOVE exit into coverage — span the apex with an R turn.
                        Double HinExit = BearingDeg(Pa, Pb);
                        Double HoutExit = BearingDeg(Pb, Pc);
                        (Double X, Double Y) TangentIn;
                        List<(Double X, Double Y)>? Fillet = ClassicCornerFillet(
                            Pb, HinExit, Pc, HoutExit, turnRadiusM, sampleM, Dist(Pa, Pb), out TangentIn);
                        if ((Fillet != null) && (CountOutsideFieldPointsOnly(Fillet, field) == 0))
                        {
                            Next.Add(new RawPoint(
                                B.Xr, B.Yr, B.DepthM, B.Action, B.PassNo, TangentIn.X, TangentIn.Y));
                            foreach ((Double X, Double Y) M in Fillet)
                            {
                                Next.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, B.PassNo, M.X, M.Y));
                            }
                        }
                        else
                        {
                            List<(Double X, Double Y)> Moves = HeadlandMovePoints(
                                Pb, HinExit, Pc, HoutExit, turnRadiusM, sampleM, field);
                            Double Cap = (2.75 * Math.PI * turnRadiusM) + Dist(Pb, Pc);
                            if ((Moves.Count == 0) || (PolylineLengthM(Pb, Moves) > Cap)
                                || (CountOutsideFieldPointsOnly(Moves, field) > 0))
                            {
                                // Intent: peel back along the inbound MOVE so the R-turn has room.
                                (Double X, Double Y) Back = ForwardEn(HinExit);
                                Double Peel = Math.Min(Dist(Pa, Pb), turnRadiusM);
                                (Double X, Double Y) PeelStart = (
                                    Pb.X - (Peel * Back.X),
                                    Pb.Y - (Peel * Back.Y));
                                if (field.ContainsWorldXy(PeelStart.X, PeelStart.Y))
                                {
                                    Moves = HeadlandMovePoints(
                                        PeelStart, HinExit, Pc, HoutExit, turnRadiusM, sampleM, field);
                                    Next.Add(new RawPoint(
                                        B.Xr, B.Yr, B.DepthM, B.Action, B.PassNo,
                                        PeelStart.X, PeelStart.Y));
                                }
                                else
                                {
                                    Next.Add(B);
                                }
                            }
                            else
                            {
                                Next.Add(B);
                            }

                            if ((Moves.Count > 0) && (PolylineLengthM(
                                    (Next[Next.Count - 1].Xs, Next[Next.Count - 1].Ys), Moves) <= Cap)
                                && (CountOutsideFieldPointsOnly(Moves, field) == 0))
                            {
                                foreach ((Double X, Double Y) M in Moves)
                                {
                                    Next.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, B.PassNo, M.X, M.Y));
                                }
                            }
                        }

                        Next.Add(new RawPoint(C.Xr, C.Yr, C.DepthM, C.Action, C.PassNo, C.Xs, C.Ys));
                        I += 2;
                        continue;
                    }

                    if (IsBladeDown(B.Action))
                    {
                        // Soft CUT pinches: classic R fillet when it fits; else any in-field CSC/keyhole.
                        Next.Add(B);
                        Double Hin = BearingDeg(Pa, Pb);
                        Double Hout = BearingDeg(Pb, Pc);
                        (Double X, Double Y) TangentIn;
                        List<(Double X, Double Y)>? Fillet = ClassicCornerFillet(
                            Pb, Hin, Pc, Hout, turnRadiusM, sampleM, Dist(Pa, Pb), out TangentIn);
                        if ((Fillet != null) && (CountOutsideFieldPointsOnly(Fillet, field) == 0))
                        {
                            Int32 Last = Next.Count - 1;
                            RawPoint Apex = Next[Last];
                            Next[Last] = new RawPoint(
                                Apex.Xr, Apex.Yr, Apex.DepthM, Apex.Action, Apex.PassNo,
                                TangentIn.X, TangentIn.Y);
                            foreach ((Double X, Double Y) M in Fillet)
                            {
                                Next.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, B.PassNo, M.X, M.Y));
                            }
                        }
                        else
                        {
                            // Intent: hard R bound — never leave a blade-down corner sharper than R.
                            List<(Double X, Double Y)> Moves = HeadlandMovePoints(
                                Pb, Hin, Pc, Hout, turnRadiusM, sampleM, field);
                            Double Cap = 2.75 * Math.PI * turnRadiusM;
                            if ((Moves.Count > 0) && (PolylineLengthM(Pb, Moves) <= Cap)
                                && (CountOutsideFieldPointsOnly(Moves, field) == 0))
                            {
                                foreach ((Double X, Double Y) M in Moves)
                                {
                                    Next.Add(new RawPoint(M.X, M.Y, 0.0, MOVE, B.PassNo, M.X, M.Y));
                                }
                            }
                        }

                        Next.Add(new RawPoint(C.Xr, C.Yr, C.DepthM, C.Action, C.PassNo, C.Xs, C.Ys));
                        I += 2;
                        continue;
                    }

                    // Blade-up residual (e.g. CUT→MOVE→CUT or leftover HAUL chord): short CSC only.
                    Double HinUp = BearingDeg(Pa, Pb);
                    Double HoutUp = BearingDeg(Pb, Pc);
                    Double StUp = CompassToMath(HinUp);
                    Double EtUp = CompassToMath(HoutUp);
                    List<(Double X, Double Y)>? ShortUp = DubinsShortest(
                        Pa.X, Pa.Y, StUp, Pc.X, Pc.Y, EtUp, turnRadiusM, sampleM, false, true, field);
                    Double CapUp = (SOFT_BEND_MAX_HALF_TURNS * Math.PI * turnRadiusM) + Dist(Pa, Pc);
                    // Intent: preserve HAUL action on loaded transit repairs (haul distance).
                    String UpAction = (B.Action == HAUL) ? HAUL : MOVE;
                    if ((ShortUp != null) && (ShortUp.Count > 0)
                        && (PolylineLengthM(Pa, ShortUp) <= CapUp))
                    {
                        foreach ((Double X, Double Y) M in ShortUp)
                        {
                            Next.Add(new RawPoint(M.X, M.Y, 0.0, UpAction, B.PassNo, M.X, M.Y));
                        }
                    }

                    Next.Add(new RawPoint(C.Xr, C.Yr, C.DepthM, C.Action, C.PassNo, C.Xs, C.Ys));
                    I += 2;
                }

                if (I < Cur.Count)
                {
                    Next.Add(Cur[Cur.Count - 1]);
                }

                Cur = Next;
                if (!Changed)
                {
                    break;
                }
            }

            return Cur;
        }

        /// <summary>
        /// True when the action is blade-down coverage (CUT/FILL/GRADE).
        /// </summary>
        /// <param name="action">Guidance action code.</param>
        /// <returns>True for blade-down actions.</returns>
        static private Boolean IsBladeDown
            (
            String action
            )
        {
            return (action == CUT) || (action == FILL) || (action == GRADE);
        }

        /// <summary>
        /// Tiles one patch into blade-width serpentine lanes.
        /// </summary>
        /// <param name="cellList">Patch cells with depth.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="headingDeg">Lane heading in compass degrees.</param>
        /// <param name="overlap">Lane overlap fraction [0, 0.9].</param>
        /// <param name="orientStartXy">Optional point to orient sweep start toward.</param>
        /// <param name="orientEndXy">Optional point to orient sweep end toward.</param>
        /// <returns>Ordered lane points with depth in metres.</returns>
        static private List<(Double X, Double Y, Double DepthM)> LanesForPatch
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cellList,
            Grid grid,
            Double bladeM,
            Double headingDeg,
            Double overlap = 0.0,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Follow Python lane-band assignment so blade coverage conserves area over wide cells.
            if (cellList.Count == 0)
            {
                return new List<(Double X, Double Y, Double DepthM)>();
            }

            List<(Double X, Double Y)> Centers = new List<(Double X, Double Y)>();
            List<Double> Depths = new List<Double>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cellList)
            {
                Centers.Add(grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By));
                Depths.Add(Entry.DepthM);
            }

            if (cellList.Count == 1)
            {
                return new List<(Double X, Double Y, Double DepthM)> { (Centers[0].X, Centers[0].Y, Depths[0]) };
            }

            Double Theta = DegreesToRadians(headingDeg);
            (Double X, Double Y) Uh = (Math.Sin(Theta), Math.Cos(Theta));
            (Double X, Double Y) Vh = (Math.Cos(Theta), -Math.Sin(Theta));
            (Double X, Double Y) Origin = Centroid(Centers);

            List<Double> Us = new List<Double>();
            List<Double> Vs = new List<Double>();
            for (Int32 i = 0; i < Centers.Count; i += 1)
            {
                Double Dx = Centers[i].X - Origin.X;
                Double Dy = Centers[i].Y - Origin.Y;
                Us.Add((Dx * Uh.X) + (Dy * Uh.Y));
                Vs.Add((Dx * Vh.X) + (Dy * Vh.Y));
            }

            Double G = grid.GridSizeM;
            Double ClampedOverlap = Math.Min(Math.Max(overlap, 0.0), 0.9);
            Double Step = Math.Max(bladeM * (1.0 - ClampedOverlap), 1e-3);
            Double VMin = Min(Vs);
            Double VMax = Max(Vs);
            Int32 NLanes = Math.Max(1, (Int32)Math.Floor(((VMax - VMin) / Step) + 1e-9) + 1);
            List<Double> LaneV = new List<Double>();
            for (Int32 k = 0; k < NLanes; k += 1)
            {
                LaneV.Add(VMin + (Step * k));
            }
            if ((VMax - LaneV[LaneV.Count - 1]) > (0.5 * Step))
            {
                LaneV.Add(VMax);
            }

            Double Half = (G / 2.0) + 1e-6;
            List<(Double Vc, List<(Double U, Double Depth, Int32 Index)> Members)> Lanes = new List<(Double Vc, List<(Double U, Double Depth, Int32 Index)> Members)>();

            foreach (Double Vc in LaneV)
            {
                List<(Double U, Double Depth, Int32 Index)> Members = new List<(Double U, Double Depth, Int32 Index)>();
                for (Int32 i = 0; i < cellList.Count; i += 1)
                {
                    if (Math.Abs(Vs[i] - Vc) <= Half)
                    {
                        Members.Add((Us[i], Depths[i], i));
                    }
                }
                Lanes.Add((Vc, Members));
            }

            HashSet<Int32> Covered = new HashSet<Int32>();
            foreach ((Double Vc, List<(Double U, Double Depth, Int32 Index)> Members) Lane in Lanes)
            {
                foreach ((Double U, Double Depth, Int32 Index) Member in Lane.Members)
                {
                    Covered.Add(Member.Index);
                }
            }

            for (Int32 i = 0; i < cellList.Count; i += 1)
            {
                if (!Covered.Contains(i))
                {
                    Int32 Best = 0;
                    Double BestDist = Double.MaxValue;

                    for (Int32 laneIndex = 0; laneIndex < Lanes.Count; laneIndex += 1)
                    {
                        Double Distance = Math.Abs(Lanes[laneIndex].Vc - Vs[i]);
                        if (Distance < BestDist)
                        {
                            BestDist = Distance;
                            Best = laneIndex;
                        }
                    }

                    Lanes[Best].Members.Add((Us[i], Depths[i], i));
                }
            }

            List<(Double Vc, List<(Double U, Double Depth, Int32 Index)> Members)> NonEmpty = new List<(Double Vc, List<(Double U, Double Depth, Int32 Index)> Members)>();
            foreach ((Double Vc, List<(Double U, Double Depth, Int32 Index)> Members) Lane in Lanes)
            {
                if (Lane.Members.Count > 0)
                {
                    NonEmpty.Add(Lane);
                }
            }
            Lanes = NonEmpty;

            List<(Double X, Double Y, Double DepthM)> Points = new List<(Double X, Double Y, Double DepthM)>();
            for (Int32 k = 0; k < Lanes.Count; k += 1)
            {
                (Double Vc, List<(Double U, Double Depth, Int32 Index)> Members) Lane = Lanes[k];
                Lane.Members.Sort((A, B) => A.U.CompareTo(B.U));

                if ((k % 2) == 1)
                {
                    Lane.Members.Reverse();
                }

                foreach ((Double U, Double Depth, Int32 Index) Member in Lane.Members)
                {
                    Double X = Origin.X + (Member.U * Uh.X) + (Lane.Vc * Vh.X);
                    Double Y = Origin.Y + (Member.U * Uh.Y) + (Lane.Vc * Vh.Y);
                    Points.Add((X, Y, Member.Depth));
                }
            }

            if (orientEndXy.HasValue && (Points.Count > 0))
            {
                if (D2((Points[0].X, Points[0].Y), orientEndXy.Value) < D2((Points[Points.Count - 1].X, Points[Points.Count - 1].Y), orientEndXy.Value))
                {
                    Points.Reverse();
                }
            }
            else if (orientStartXy.HasValue && (Points.Count > 0))
            {
                if (D2((Points[Points.Count - 1].X, Points[Points.Count - 1].Y), orientStartXy.Value) < D2((Points[0].X, Points[0].Y), orientStartXy.Value))
                {
                    Points.Reverse();
                }
            }

            return Points;
        }

        /// <summary>
        /// Splits a patch into 8-connected pockets.
        /// </summary>
        /// <param name="cellList">Patch cells with depth.</param>
        /// <returns>Connected components with original entries preserved.</returns>
        static private List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> ConnectedPockets
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cellList
            )
        {
            // Discover connected bins first, then reattach all original entries for each component.
            HashSet<(Int32 Bx, Int32 By)> Cells = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cellList)
            {
                Cells.Add(Entry.Cell);
            }

            HashSet<(Int32 Bx, Int32 By)> Seen = new HashSet<(Int32 Bx, Int32 By)>();
            List<HashSet<(Int32 Bx, Int32 By)>> Components = new List<HashSet<(Int32 Bx, Int32 By)>>();

            foreach ((Int32 Bx, Int32 By) Start in Cells)
            {
                if (Seen.Contains(Start))
                {
                    continue;
                }

                HashSet<(Int32 Bx, Int32 By)> Component = new HashSet<(Int32 Bx, Int32 By)> { Start };
                Seen.Add(Start);
                Stack<(Int32 Bx, Int32 By)> Stack = new Stack<(Int32 Bx, Int32 By)>();
                Stack.Push(Start);

                while (Stack.Count > 0)
                {
                    (Int32 Bx, Int32 By) Current = Stack.Pop();

                    for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                    {
                        for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                        {
                            if ((Dx == 0) && (Dy == 0))
                            {
                                continue;
                            }

                            (Int32 Bx, Int32 By) Neighbor = (Current.Bx + Dx, Current.By + Dy);
                            if (Cells.Contains(Neighbor) && !Seen.Contains(Neighbor))
                            {
                                Seen.Add(Neighbor);
                                Component.Add(Neighbor);
                                Stack.Push(Neighbor);
                            }
                        }
                    }
                }

                Components.Add(Component);
            }

            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Result = new List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>();
            foreach (HashSet<(Int32 Bx, Int32 By)> Component in Components)
            {
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Bucket = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cellList)
                {
                    if (Component.Contains(Entry.Cell))
                    {
                        Bucket.Add(Entry);
                    }
                }
                Result.Add(Bucket);
            }

            return Result;
        }

        /// <summary>
        /// Computes greedy nearest-neighbour ordering of centroids.
        /// </summary>
        /// <param name="cents">Centroids to chain.</param>
        /// <param name="orientStartXy">Optional anchor for first centroid.</param>
        /// <param name="orientEndXy">Optional anchor for last centroid.</param>
        /// <returns>Ordered centroid indices.</returns>
        static private List<Int32> NnOrder
            (
            List<(Double X, Double Y)> cents,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Keep cut and fill anchoring behavior symmetric with Python chain construction.
            Int32 N = cents.Count;
            if (N <= 1)
            {
                List<Int32> Identity = new List<Int32>();
                for (Int32 i = 0; i < N; i += 1)
                {
                    Identity.Add(i);
                }
                return Identity;
            }

            HashSet<Int32> Remaining = new HashSet<Int32>();
            for (Int32 i = 0; i < N; i += 1)
            {
                Remaining.Add(i);
            }

            if (orientEndXy.HasValue)
            {
                Int32 Last = ArgMinDistanceToPoint(Remaining, cents, orientEndXy.Value);
                List<Int32> Chain = new List<Int32> { Last };
                Remaining.Remove(Last);

                while (Remaining.Count > 0)
                {
                    Int32 Front = Chain[0];
                    Int32 Next = ArgMinDistanceToCentroid(Remaining, cents, cents[Front]);
                    Chain.Insert(0, Next);
                    Remaining.Remove(Next);
                }

                return Chain;
            }

            (Double X, Double Y) Source = orientStartXy.HasValue ? orientStartXy.Value : cents[0];
            Int32 First = ArgMinDistanceToPoint(Remaining, cents, Source);
            List<Int32> Forward = new List<Int32> { First };
            Remaining.Remove(First);

            while (Remaining.Count > 0)
            {
                Int32 Back = Forward[Forward.Count - 1];
                Int32 Next = ArgMinDistanceToCentroid(Remaining, cents, cents[Back]);
                Forward.Add(Next);
                Remaining.Remove(Next);
            }

            return Forward;
        }

        /// <summary>
        /// Splits a flow strip into maximal runs of consecutive 8-adjacent cells.
        /// </summary>
        /// <param name="cellList">Ordered streamline cells with depth.</param>
        /// <returns>Runs preserving original in-run order.</returns>
        static private List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> OrderedRuns
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cellList
            )
        {
            // Preserve flow order and break only at non-adjacent jumps that represent multi-drop hops.
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Runs = new List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Current = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            (Int32 Bx, Int32 By)? Prev = null;

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cellList)
            {
                if (!Prev.HasValue || ((Math.Abs(Entry.Cell.Bx - Prev.Value.Bx) <= 1) && (Math.Abs(Entry.Cell.By - Prev.Value.By) <= 1)))
                {
                    Current.Add(Entry);
                }
                else
                {
                    Runs.Add(Current);
                    Current = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { Entry };
                }
                Prev = Entry.Cell;
            }

            if (Current.Count > 0)
            {
                Runs.Add(Current);
            }

            return Runs;
        }

        /// <summary>
        /// Builds lanes by following streamline order per run.
        /// </summary>
        /// <param name="cellList">Cells in flow order.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="orientStartXy">Optional anchor for first run.</param>
        /// <param name="orientEndXy">Optional anchor for last run.</param>
        /// <returns>Drive-ordered lane points.</returns>
        static private List<(Double X, Double Y, Double DepthM)> LanesFlow
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cellList,
            Grid grid,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Keep each run intact and only flip whole runs to honor haul-side orientation.
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Runs = OrderedRuns(cellList);
            List<List<(Double X, Double Y, Double DepthM)>> Segments = new List<List<(Double X, Double Y, Double DepthM)>>();

            foreach (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Run in Runs)
            {
                if (Run.Count == 0)
                {
                    continue;
                }

                List<(Double X, Double Y, Double DepthM)> Segment = new List<(Double X, Double Y, Double DepthM)>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Run)
                {
                    (Double X, Double Y) Center = grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By);
                    Segment.Add((Center.X, Center.Y, Entry.DepthM));
                }
                Segments.Add(Segment);
            }

            if (Segments.Count == 0)
            {
                return new List<(Double X, Double Y, Double DepthM)>();
            }

            List<(Double X, Double Y)> Cents = new List<(Double X, Double Y)>();
            foreach (List<(Double X, Double Y, Double DepthM)> Segment in Segments)
            {
                List<(Double X, Double Y)> Planar = new List<(Double X, Double Y)>();
                foreach ((Double X, Double Y, Double DepthM) Point in Segment)
                {
                    Planar.Add((Point.X, Point.Y));
                }
                Cents.Add(Centroid(Planar));
            }

            List<(Double X, Double Y, Double DepthM)> Points = new List<(Double X, Double Y, Double DepthM)>();
            if (orientEndXy.HasValue)
            {
                List<Int32> Order = NnOrder(Cents, null, orientEndXy);
                for (Int32 Rank = 0; Rank < Order.Count; Rank += 1)
                {
                    Int32 Index = Order[Rank];
                    (Double X, Double Y) Target = (Rank == (Order.Count - 1)) ? orientEndXy.Value : Cents[Order[Rank + 1]];
                    List<(Double X, Double Y, Double DepthM)> Segment = new List<(Double X, Double Y, Double DepthM)>(Segments[Index]);

                    if ((Segment.Count >= 2) && (D2((Segment[0].X, Segment[0].Y), Target) < D2((Segment[Segment.Count - 1].X, Segment[Segment.Count - 1].Y), Target)))
                    {
                        Segment.Reverse();
                    }

                    Points.AddRange(Segment);
                }
            }
            else
            {
                (Double X, Double Y) Source = orientStartXy.HasValue ? orientStartXy.Value : Cents[0];
                List<Int32> Order = NnOrder(Cents, orientStartXy, null);

                for (Int32 Rank = 0; Rank < Order.Count; Rank += 1)
                {
                    Int32 Index = Order[Rank];
                    (Double X, Double Y) StartRef = (Rank == 0) ? Source : Cents[Order[Rank - 1]];
                    List<(Double X, Double Y, Double DepthM)> Segment = new List<(Double X, Double Y, Double DepthM)>(Segments[Index]);

                    if ((Segment.Count >= 2) && (D2((Segment[Segment.Count - 1].X, Segment[Segment.Count - 1].Y), StartRef) < D2((Segment[0].X, Segment[0].Y), StartRef)))
                    {
                        Segment.Reverse();
                    }

                    Points.AddRange(Segment);
                }
            }

            return Points;
        }

        /// <summary>
        /// Builds a lane directly from existing cell order.
        /// </summary>
        /// <param name="cellList">Cell list already in drivable order.</param>
        /// <param name="grid">Grid projection helper.</param>
        /// <param name="orientStartXy">Optional orientation start point.</param>
        /// <param name="orientEndXy">Optional orientation end point.</param>
        /// <returns>Drive-ordered lane points.</returns>
        static private List<(Double X, Double Y, Double DepthM)> LaneDirect
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cellList,
            Grid grid,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Keep knot-reduced cell order untouched except for whole-lane orientation flip.
            List<(Double X, Double Y, Double DepthM)> Points = new List<(Double X, Double Y, Double DepthM)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cellList)
            {
                (Double X, Double Y) Center = grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By);
                Points.Add((Center.X, Center.Y, Entry.DepthM));
            }

            if (Points.Count >= 2)
            {
                if (orientEndXy.HasValue)
                {
                    if (D2((Points[0].X, Points[0].Y), orientEndXy.Value) < D2((Points[Points.Count - 1].X, Points[Points.Count - 1].Y), orientEndXy.Value))
                    {
                        Points.Reverse();
                    }
                }
                else if (orientStartXy.HasValue)
                {
                    if (D2((Points[Points.Count - 1].X, Points[Points.Count - 1].Y), orientStartXy.Value) < D2((Points[0].X, Points[0].Y), orientStartXy.Value))
                    {
                        Points.Reverse();
                    }
                }
            }

            return Points;
        }

        /// <summary>
        /// Builds pocket-aware lanes for multi-drop patches.
        /// </summary>
        /// <param name="cellList">Patch cells with depth.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="headingDeg">Lane heading in degrees.</param>
        /// <param name="overlap">Lane overlap fraction.</param>
        /// <param name="orientStartXy">Optional sweep start anchor.</param>
        /// <param name="orientEndXy">Optional sweep end anchor.</param>
        /// <returns>Combined lane points across pockets.</returns>
        static private List<(Double X, Double Y, Double DepthM)> LanesMulti
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cellList,
            Grid grid,
            Double bladeM,
            Double headingDeg,
            Double overlap,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Sequence pockets so inter-pocket blade-up hops stay short while honoring haul-side anchors.
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Pockets = ConnectedPockets(cellList);
            if (Pockets.Count <= 1)
            {
                return LanesForPatch(cellList, grid, bladeM, headingDeg, overlap, orientStartXy, orientEndXy);
            }

            List<(Double X, Double Y)> Cents = new List<(Double X, Double Y)>();
            foreach (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Pocket in Pockets)
            {
                List<(Double X, Double Y)> PocketCenters = new List<(Double X, Double Y)>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pocket)
                {
                    PocketCenters.Add(grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By));
                }
                Cents.Add(Centroid(PocketCenters));
            }

            List<(Double X, Double Y, Double DepthM)> Points = new List<(Double X, Double Y, Double DepthM)>();
            if (orientEndXy.HasValue)
            {
                List<Int32> Order = NnOrder(Cents, null, orientEndXy);
                for (Int32 Rank = 0; Rank < Order.Count; Rank += 1)
                {
                    Int32 Index = Order[Rank];
                    (Double X, Double Y) Target = (Rank == (Order.Count - 1)) ? orientEndXy.Value : Cents[Order[Rank + 1]];
                    Points.AddRange(LanesForPatch(Pockets[Index], grid, bladeM, headingDeg, overlap, null, Target));
                }
            }
            else
            {
                (Double X, Double Y) Source = orientStartXy.HasValue ? orientStartXy.Value : Cents[0];
                List<Int32> Order = NnOrder(Cents, orientStartXy, null);
                for (Int32 Rank = 0; Rank < Order.Count; Rank += 1)
                {
                    Int32 Index = Order[Rank];
                    (Double X, Double Y) StartRef = (Rank == 0) ? Source : Cents[Order[Rank - 1]];
                    Points.AddRange(LanesForPatch(Pockets[Index], grid, bladeM, headingDeg, overlap, StartRef, null));
                }
            }

            return Points;
        }

        /// <summary>
        /// Re-tiles a tangled sweep along principal axis when it strictly improves hairpins.
        /// </summary>
        /// <param name="lane">Current lane points.</param>
        /// <param name="cells">Original cells and depths.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="orientStartXy">Optional start anchor.</param>
        /// <param name="orientEndXy">Optional end anchor.</param>
        /// <returns>Tuple of selected lane and repair flag (1 repaired, 0 unchanged).</returns>
        static private (List<(Double X, Double Y, Double DepthM)> Lane, Int32 Repaired) RepairIfTangled
            (
            List<(Double X, Double Y, Double DepthM)> lane,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            Grid grid,
            Double bladeM,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Apply conservative replacement: only keep alternate lane when it strictly lowers hairpin count.
            Int32 Initial = HardTurnCount(lane, grid, bladeM);
            if (Initial == 0)
            {
                return (lane, 0);
            }

            List<(Double X, Double Y)> Centers = new List<(Double X, Double Y)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cells)
            {
                Centers.Add(grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By));
            }

            Double PcaHeading = PrincipalHeadingDeg(Centers);
            List<(Double X, Double Y, Double DepthM)> Alternate = LanesMulti(cells, grid, bladeM, PcaHeading, 0.0, orientStartXy, orientEndXy);

            if ((Alternate.Count > 0) && (HardTurnCount(Alternate, grid, bladeM) < Initial))
            {
                return (Alternate, 1);
            }

            return (lane, 0);
        }

        /// <summary>
        /// Angular step (degrees) when searching for the min-lane sweep heading.
        /// </summary>
        private const Double MIN_LANE_HEADING_STEP_DEG = 10.0;

        /// <summary>
        /// Decomposes a patch into parallel blade-width lanes along the min-lane sweep heading.
        /// </summary>
        /// <param name="cells">Patch cells.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <returns>Lanes in cross-axis order, each sorted along the sweep axis.</returns>
        static private List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> PatchDrivableLanes
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            Grid grid,
            Double bladeM
            )
        {
            // Intent: fewer lanes ⇒ fewer U-turn keyholes; search headings, not PCA alone.
            if (cells.Count <= 1)
            {
                return new List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> { new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(cells) };
            }

            List<(Double X, Double Y)> Centers = new List<(Double X, Double Y)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cells)
            {
                Centers.Add(grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By));
            }

            Double BestHeading = PrincipalHeadingDeg(Centers);
            (Int32 NLanes, Double AlongSpan) BestScore = LaneCountAndSpan(Centers, BestHeading, bladeM);
            for (Double Deg = 0.0; Deg < 180.0; Deg += MIN_LANE_HEADING_STEP_DEG)
            {
                (Int32 NLanes, Double AlongSpan) Score = LaneCountAndSpan(Centers, Deg, bladeM);
                // Intent: only leave PCA when another heading truly needs fewer U-turns.
                if (Score.NLanes < BestScore.NLanes)
                {
                    BestScore = Score;
                    BestHeading = Deg;
                }
            }

            return LanesAtHeading(cells, Centers, BestHeading, bladeM);
        }

        /// <summary>
        /// Counts blade-width lanes and along-axis span for a candidate sweep heading.
        /// </summary>
        /// <param name="centers">Cell centers.</param>
        /// <param name="headingDeg">Sweep heading in compass degrees.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <returns>Lane count and along-axis span in metres.</returns>
        static private (Int32 NLanes, Double AlongSpan) LaneCountAndSpan
            (
            List<(Double X, Double Y)> centers,
            Double headingDeg,
            Double bladeM
            )
        {
            Double Theta = DegreesToRadians(headingDeg);
            (Double X, Double Y) Uh = (Math.Sin(Theta), Math.Cos(Theta));
            (Double X, Double Y) Vh = (Math.Cos(Theta), -Math.Sin(Theta));
            (Double X, Double Y) Origin = Centroid(centers);
            Double UMin = Double.PositiveInfinity;
            Double UMax = Double.NegativeInfinity;
            Double VMin = Double.PositiveInfinity;
            Double VMax = Double.NegativeInfinity;
            foreach ((Double X, Double Y) Point in centers)
            {
                Double Dx = Point.X - Origin.X;
                Double Dy = Point.Y - Origin.Y;
                Double U = (Dx * Uh.X) + (Dy * Uh.Y);
                Double V = (Dx * Vh.X) + (Dy * Vh.Y);
                if (U < UMin) { UMin = U; }
                if (U > UMax) { UMax = U; }
                if (V < VMin) { VMin = V; }
                if (V > VMax) { VMax = V; }
            }

            Double Step = Math.Max(bladeM, 1e-3);
            Int32 NLanes = Math.Max(1, (Int32)Math.Floor(((VMax - VMin) / Step) + 1e-9) + 1);
            if ((VMax - (VMin + (Step * (NLanes - 1)))) > (0.5 * Step))
            {
                NLanes += 1;
            }

            return (NLanes, UMax - UMin);
        }

        /// <summary>
        /// Bins cells into blade-width lanes at a fixed sweep heading.
        /// </summary>
        /// <param name="cells">Patch cells.</param>
        /// <param name="centers">Matching cell centers.</param>
        /// <param name="headingDeg">Sweep heading in compass degrees.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <returns>Non-empty lanes in cross-axis order.</returns>
        static private List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> LanesAtHeading
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            List<(Double X, Double Y)> centers,
            Double headingDeg,
            Double bladeM
            )
        {
            Double Theta = DegreesToRadians(headingDeg);
            (Double X, Double Y) Uh = (Math.Sin(Theta), Math.Cos(Theta));
            (Double X, Double Y) Vh = (Math.Cos(Theta), -Math.Sin(Theta));
            (Double X, Double Y) Origin = Centroid(centers);

            List<Double> Us = new List<Double>();
            List<Double> Vs = new List<Double>();
            foreach ((Double X, Double Y) Point in centers)
            {
                Double Dx = Point.X - Origin.X;
                Double Dy = Point.Y - Origin.Y;
                Us.Add((Dx * Uh.X) + (Dy * Uh.Y));
                Vs.Add((Dx * Vh.X) + (Dy * Vh.Y));
            }

            Double Step = Math.Max(bladeM, 1e-3);
            Double VMin = Min(Vs);
            Double VMax = Max(Vs);
            Int32 NLanes = Math.Max(1, (Int32)Math.Floor(((VMax - VMin) / Step) + 1e-9) + 1);
            List<Double> LaneV = new List<Double>();
            for (Int32 k = 0; k < NLanes; k += 1)
            {
                LaneV.Add(VMin + (Step * k));
            }
            if ((VMax - LaneV[LaneV.Count - 1]) > (0.5 * Step))
            {
                LaneV.Add(VMax);
            }

            List<List<Int32>> Groups = new List<List<Int32>>();
            for (Int32 i = 0; i < LaneV.Count; i += 1)
            {
                Groups.Add(new List<Int32>());
            }

            for (Int32 i = 0; i < cells.Count; i += 1)
            {
                Int32 Best = 0;
                Double BestDist = Double.MaxValue;
                for (Int32 Lane = 0; Lane < LaneV.Count; Lane += 1)
                {
                    Double Distance = Math.Abs(Vs[i] - LaneV[Lane]);
                    if (Distance < BestDist)
                    {
                        BestDist = Distance;
                        Best = Lane;
                    }
                }
                Groups[Best].Add(i);
            }

            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Lanes = new List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>();
            foreach (List<Int32> Members in Groups)
            {
                if (Members.Count == 0)
                {
                    continue;
                }

                Members.Sort((A, B) => Us[A].CompareTo(Us[B]));
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lane = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                foreach (Int32 Index in Members)
                {
                    Lane.Add(cells[Index]);
                }
                Lanes.Add(Lane);
            }

            return Lanes;
        }

        /// <summary>
        /// Builds boustrophedon ordering for one connected pocket.
        /// </summary>
        /// <param name="cells">Connected pocket cells.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="orientStartXy">Optional start anchor.</param>
        /// <param name="orientEndXy">Optional end anchor.</param>
        /// <returns>Cell ordering in serpentine lane walk.</returns>
        static private List<((Int32 Bx, Int32 By) Cell, Double DepthM)> BoustrophedonOne
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            Grid grid,
            Double bladeM,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Alternate lane direction to create one turn per lane transition.
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Lanes = PatchDrivableLanes(cells, grid, bladeM);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Out = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();

            for (Int32 j = 0; j < Lanes.Count; j += 1)
            {
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lane = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Lanes[j]);
                if ((j % 2) == 1)
                {
                    Lane.Reverse();
                }
                Out.AddRange(Lane);
            }

            if (Out.Count >= 2)
            {
                (Double X, Double Y) C0 = grid.CellCenterXY(Out[0].Cell.Bx, Out[0].Cell.By);
                (Double X, Double Y) C1 = grid.CellCenterXY(Out[Out.Count - 1].Cell.Bx, Out[Out.Count - 1].Cell.By);

                if (orientEndXy.HasValue)
                {
                    if (D2(C0, orientEndXy.Value) < D2(C1, orientEndXy.Value))
                    {
                        Out.Reverse();
                    }
                }
                else if (orientStartXy.HasValue)
                {
                    if (D2(C1, orientStartXy.Value) < D2(C0, orientStartXy.Value))
                    {
                        Out.Reverse();
                    }
                }
            }

            return Out;
        }

        /// <summary>
        /// Builds boustrophedon ordering pocket-by-pocket for multi-drop patches.
        /// </summary>
        /// <param name="cells">Patch cells.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="orientStartXy">Optional start anchor.</param>
        /// <param name="orientEndXy">Optional end anchor.</param>
        /// <returns>Boustrophedon cell sequence with all original cells retained.</returns>
        static private List<((Int32 Bx, Int32 By) Cell, Double DepthM)> BoustrophedonCells
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            Grid grid,
            Double bladeM,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Sequence pockets with nearest-neighbour chaining to minimize blade-up reposition gaps.
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Pockets = ConnectedPockets(cells);
            if (Pockets.Count <= 1)
            {
                return BoustrophedonOne(cells, grid, bladeM, orientStartXy, orientEndXy);
            }

            List<(Double X, Double Y)> Cents = new List<(Double X, Double Y)>();
            foreach (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Pocket in Pockets)
            {
                List<(Double X, Double Y)> PocketCenters = new List<(Double X, Double Y)>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pocket)
                {
                    PocketCenters.Add(grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By));
                }
                Cents.Add(Centroid(PocketCenters));
            }

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Out = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();

            if (orientEndXy.HasValue)
            {
                List<Int32> Order = NnOrder(Cents, null, orientEndXy);
                for (Int32 Rank = 0; Rank < Order.Count; Rank += 1)
                {
                    Int32 Index = Order[Rank];
                    (Double X, Double Y) Target = (Rank == (Order.Count - 1)) ? orientEndXy.Value : Cents[Order[Rank + 1]];
                    Out.AddRange(BoustrophedonOne(Pockets[Index], grid, bladeM, null, Target));
                }
            }
            else
            {
                (Double X, Double Y) Source = orientStartXy.HasValue ? orientStartXy.Value : Cents[0];
                List<Int32> Order = NnOrder(Cents, orientStartXy, null);
                for (Int32 Rank = 0; Rank < Order.Count; Rank += 1)
                {
                    Int32 Index = Order[Rank];
                    (Double X, Double Y) StartRef = (Rank == 0) ? Source : Cents[Order[Rank - 1]];
                    Out.AddRange(BoustrophedonOne(Pockets[Index], grid, bladeM, StartRef, null));
                }
            }

            return Out;
        }

        /// <summary>
        /// Renders a pass lane exactly as current guidance logic would draw it.
        /// </summary>
        /// <param name="cells">Pass cells.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="flowPass">Whether pass follows flow ordering.</param>
        /// <param name="headingDeg">Fallback heading for lane tiling.</param>
        /// <param name="overlap">Lane overlap fraction.</param>
        /// <param name="orientStartXy">Optional start anchor.</param>
        /// <param name="orientEndXy">Optional end anchor.</param>
        /// <returns>Rendered lane points after optional knot repair.</returns>
        static private List<(Double X, Double Y, Double DepthM)> RenderedLane
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            Grid grid,
            Double bladeM,
            Boolean flowPass,
            Double headingDeg,
            Double overlap,
            (Double X, Double Y)? orientStartXy = null,
            (Double X, Double Y)? orientEndXy = null
            )
        {
            // Mirror production rendering path so knot detection targets real current behavior.
            List<(Double X, Double Y, Double DepthM)> Lane = flowPass
                ? LanesFlow(cells, grid, orientStartXy, orientEndXy)
                : LanesMulti(cells, grid, bladeM, headingDeg, overlap, orientStartXy, orientEndXy);

            (Lane, Int32 _) = RepairIfTangled(Lane, cells, grid, bladeM, orientStartXy, orientEndXy);
            return Lane;
        }

        /// <summary>
        /// Returns whether the rendered cut or fill sweep for a pass still hairpins.
        /// </summary>
        /// <param name="pass">Pass to test.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="hairpinMin">Minimum hairpin count to flag knot.</param>
        /// <param name="headingDeg">Fallback heading.</param>
        /// <param name="overlap">Lane overlap fraction.</param>
        /// <returns>True if cut or fill sweep appears knotted.</returns>
        static private Boolean PassIsKnot
            (
            Pass pass,
            Grid grid,
            Double bladeM,
            Int32 hairpinMin,
            Double headingDeg = 0.0,
            Double overlap = 0.0
            )
        {
            // Evaluate the rendered lane rather than raw cells so existing repairs are honored.
            Boolean FlowPass = pass.CutHeadingDeg.HasValue || pass.FillHeadingDeg.HasValue;
            Double CutHeading = pass.CutHeadingDeg.HasValue ? pass.CutHeadingDeg.Value : headingDeg;
            Double FillHeading = pass.FillHeadingDeg.HasValue ? pass.FillHeadingDeg.Value : headingDeg;

            if (pass.CutCells.Count > 3)
            {
                List<(Double X, Double Y, Double DepthM)> Lane = RenderedLane(pass.CutCells, grid, bladeM, FlowPass, CutHeading, overlap, null, pass.FillStartXy);
                if (HardTurnCount(Lane, grid, bladeM) >= hairpinMin)
                {
                    return true;
                }
            }

            if (pass.FillCells.Count > 3)
            {
                (Double X, Double Y)? CutEnd = null;
                if (pass.CutCells.Count > 0)
                {
                    (Int32 Bx, Int32 By) Last = pass.CutCells[pass.CutCells.Count - 1].Cell;
                    CutEnd = grid.CellCenterXY(Last.Bx, Last.By);
                }

                List<(Double X, Double Y, Double DepthM)> Lane = RenderedLane(pass.FillCells, grid, bladeM, FlowPass, FillHeading, overlap, CutEnd, null);
                if (HardTurnCount(Lane, grid, bladeM) >= hairpinMin)
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Ramer-Douglas-Peucker simplification retaining indices of kept vertices.
        /// </summary>
        /// <param name="xs">Polyline x coordinates.</param>
        /// <param name="ys">Polyline y coordinates.</param>
        /// <param name="tol">Tolerance in metres.</param>
        /// <returns>Indices retained by simplification.</returns>
        static private List<Int32> RdpKeep
            (
            List<Double> xs,
            List<Double> ys,
            Double tol
            )
        {
            // Use iterative stack form so long guidance tracks avoid recursion depth issues.
            Int32 N = xs.Count;
            if (N < 3)
            {
                List<Int32> All = new List<Int32>();
                for (Int32 i = 0; i < N; i += 1)
                {
                    All.Add(i);
                }
                return All;
            }

            List<Boolean> Keep = new List<Boolean>();
            for (Int32 i = 0; i < N; i += 1)
            {
                Keep.Add(false);
            }
            Keep[0] = true;
            Keep[N - 1] = true;

            Double Tol2 = tol * tol;
            Stack<(Int32 A, Int32 B)> Stack = new Stack<(Int32 A, Int32 B)>();
            Stack.Push((0, N - 1));

            while (Stack.Count > 0)
            {
                (Int32 A, Int32 B) Segment = Stack.Pop();
                Double Ax = xs[Segment.A];
                Double Ay = ys[Segment.A];
                Double Bx = xs[Segment.B];
                Double By = ys[Segment.B];
                Double Dx = Bx - Ax;
                Double Dy = By - Ay;
                Double SegLen2 = (Dx * Dx) + (Dy * Dy);

                Double Dmax = 0.0;
                Int32 Index = -1;

                for (Int32 i = Segment.A + 1; i < Segment.B; i += 1)
                {
                    Double D2Local;
                    if (SegLen2 == 0.0)
                    {
                        D2Local = ((xs[i] - Ax) * (xs[i] - Ax)) + ((ys[i] - Ay) * (ys[i] - Ay));
                    }
                    else
                    {
                        Double T = (((xs[i] - Ax) * Dx) + ((ys[i] - Ay) * Dy)) / SegLen2;
                        Double Px = Ax + (T * Dx);
                        Double Py = Ay + (T * Dy);
                        D2Local = ((xs[i] - Px) * (xs[i] - Px)) + ((ys[i] - Py) * (ys[i] - Py));
                    }

                    if (D2Local > Dmax)
                    {
                        Dmax = D2Local;
                        Index = i;
                    }
                }

                if ((Index != -1) && (Dmax > Tol2))
                {
                    Keep[Index] = true;
                    Stack.Push((Segment.A, Index));
                    Stack.Push((Index, Segment.B));
                }
            }

            List<Int32> Result = new List<Int32>();
            for (Int32 i = 0; i < N; i += 1)
            {
                if (Keep[i])
                {
                    Result.Add(i);
                }
            }
            return Result;
        }

        /// <summary>
        /// Computes total turn count and summed turn angle from raw track geometry.
        /// </summary>
        /// <param name="xs">Raw x coordinates.</param>
        /// <param name="ys">Raw y coordinates.</param>
        /// <param name="gridSizeM">Grid size in metres.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <param name="turnThresholdDeg">Minimum heading change counted as turn.</param>
        /// <returns>Turn count and total turn angle in radians.</returns>
        static private (Int32 NTurns, Double TurnAngleRad) TurnMetrics
            (
            List<Double> xs,
            List<Double> ys,
            Double gridSizeM,
            Double bladeM,
            Double turnThresholdDeg = 60.0
            )
        {
            // Simplify first so raster stair-steps are not mistaken for real slow-down turns.
            if (xs.Count < 3)
            {
                return (0, 0.0);
            }

            Double Tol = Math.Max(bladeM, gridSizeM) * 1.5;
            List<Int32> Idx = RdpKeep(xs, ys, Tol);
            List<Double> Bearings = new List<Double>();

            for (Int32 k = 0; k < (Idx.Count - 1); k += 1)
            {
                Int32 i = Idx[k];
                Int32 j = Idx[k + 1];
                Bearings.Add(RadiansToDegrees(Math.Atan2(xs[j] - xs[i], ys[j] - ys[i])));
            }

            Int32 TurnCount = 0;
            Double Total = 0.0;

            for (Int32 k = 1; k < Bearings.Count; k += 1)
            {
                Double Delta = Math.Abs(((Bearings[k] - Bearings[k - 1] + 180.0) % 360.0) - 180.0);
                if (Delta >= turnThresholdDeg)
                {
                    Total += DegreesToRadians(Delta);
                    TurnCount += 1;
                }
            }

            return (TurnCount, Total);
        }

        /// <summary>
        /// Builds operator instruction text for one action/depth pair.
        /// </summary>
        /// <param name="action">Action code.</param>
        /// <param name="depthMm">Target depth in millimetres.</param>
        /// <returns>Instruction string.</returns>
        static private String Instruction
            (
            String action,
            Double depthMm
            )
        {
            // Keep exact phrase mapping so downstream operator prompts stay consistent.
            Int32 D = (Int32)Math.Round(depthMm);
            if (action == START)
            {
                return $"Start: drive to first cut, lower blade, cut {D} mm";
            }
            if (action == RETURN)
            {
                return $"Raise blade, drive to next cut, lower blade, cut {D} mm";
            }
            if (action == CUT)
            {
                return $"Cut {D} mm";
            }
            if (action == HAUL)
            {
                return $"Raise blade, haul to fill, lower blade, spread {D} mm";
            }
            if (action == FILL)
            {
                return $"Spread {D} mm";
            }
            if (action == GRADE)
            {
                return "Turn back over fill, grade & compact";
            }
            if (action == MOVE)
            {
                return $"Raise blade, drive to next pocket, lower blade, {D} mm";
            }
            if (action == MANUAL_MOVE)
            {
                return "MANUAL MOVE: verify and drive clear of obstruction";
            }
            return String.Empty;
        }

        /// <summary>
        /// Converts UTM point to latitude/longitude.
        /// </summary>
        /// <param name="field">Field metadata and conversion context.</param>
        /// <param name="x">UTM easting.</param>
        /// <param name="y">UTM northing.</param>
        /// <returns>Latitude and longitude tuple.</returns>
        static private (Double Lat, Double Lon) LatLonFromUtm
            (
            AgdField field,
            Double x,
            Double y
            )
        {
            // Delegate to the AGD field conversion utility expected in the C# AGD port.
            return field.LatLonFromUtm(x, y);
        }

        /// <summary>
        /// Computes move time for one trapezoidal accel/decel travel leg.
        /// </summary>
        /// <param name="distanceM">Distance in metres.</param>
        /// <param name="vMaxMps">Maximum speed in metres per second.</param>
        /// <param name="accelTimeS">Acceleration time in seconds.</param>
        /// <param name="decelTimeS">Deceleration time in seconds.</param>
        /// <returns>Move time in seconds.</returns>
        static private Double MoveTime
            (
            Double distanceM,
            Double vMaxMps,
            Double accelTimeS,
            Double decelTimeS
            )
        {
            // Reproduce timemodel._move_time behavior locally so Swaths stays self-contained.
            if ((distanceM <= 0.0) || (vMaxMps <= 0.0))
            {
                return 0.0;
            }

            Double AccelDist = 0.5 * vMaxMps * Math.Max(accelTimeS, 0.0);
            Double DecelDist = 0.5 * vMaxMps * Math.Max(decelTimeS, 0.0);
            Double RampDist = AccelDist + DecelDist;

            if (distanceM > RampDist)
            {
                Double Cruise = (distanceM - RampDist) / vMaxMps;
                return Math.Max(accelTimeS, 0.0) + Cruise + Math.Max(decelTimeS, 0.0);
            }

            Double Scale = (RampDist > 0.0) ? (distanceM / RampDist) : 0.0;
            return (Math.Max(accelTimeS, 0.0) + Math.Max(decelTimeS, 0.0)) * Scale;
        }

        /// <summary>
        /// Determines whether an action is blade-up travel.
        /// </summary>
        /// <param name="action">Action code.</param>
        /// <returns>True for START/RETURN/HAUL/MOVE.</returns>
        static private Boolean IsBladeUp
            (
            String action
            )
        {
            // Keep travel-state categorization centralized for detours and blade labels.
            return (action == START) || (action == RETURN)
                || (action == HAUL) || (action == MOVE)
                || (action == MANUAL_MOVE);
        }

        /// <summary>
        /// Escapes one CSV field with quote doubling as needed.
        /// </summary>
        /// <param name="value">Raw field value.</param>
        /// <returns>CSV-safe field text.</returns>
        static private String CsvEscape
            (
            String value
            )
        {
            // Quote only when required so output matches compact Python csv.writer behavior.
            if ((value.Contains(',')) || (value.Contains('"')) || (value.Contains('\n')) || (value.Contains('\r')))
            {
                return "\"" + value.Replace("\"", "\"\"") + "\"";
            }
            return value;
        }

        /// <summary>
        /// Converts radians to degrees.
        /// </summary>
        /// <param name="radians">Angle in radians.</param>
        /// <returns>Angle in degrees.</returns>
        static private Double RadiansToDegrees
            (
            Double radians
            )
        {
            // Keep explicit conversion helper to avoid scattered magic factors.
            return radians * (180.0 / Math.PI);
        }

        /// <summary>
        /// Converts degrees to radians.
        /// </summary>
        /// <param name="degrees">Angle in degrees.</param>
        /// <returns>Angle in radians.</returns>
        static private Double DegreesToRadians
            (
            Double degrees
            )
        {
            // Keep explicit conversion helper to avoid scattered magic factors.
            return degrees * (Math.PI / 180.0);
        }

        /// <summary>
        /// Normalizes an angle to [0, 360).
        /// </summary>
        /// <param name="degrees">Raw degree value.</param>
        /// <returns>Normalized degree value.</returns>
        static private Double Mod360
            (
            Double degrees
            )
        {
            // Preserve Python modulo normalization semantics for negative bearings.
            Double Mod = degrees % 360.0;
            return (Mod < 0.0) ? (Mod + 360.0) : Mod;
        }

        /// <summary>
        /// Returns minimum value from a non-empty list.
        /// </summary>
        /// <param name="values">Input values.</param>
        /// <returns>Minimum value.</returns>
        static private Double Min
            (
            List<Double> values
            )
        {
            // Avoid LINQ to keep explicit style and allocation behavior.
            Double Best = values[0];
            for (Int32 i = 1; i < values.Count; i += 1)
            {
                if (values[i] < Best)
                {
                    Best = values[i];
                }
            }
            return Best;
        }

        /// <summary>
        /// Returns maximum value from a non-empty list.
        /// </summary>
        /// <param name="values">Input values.</param>
        /// <returns>Maximum value.</returns>
        static private Double Max
            (
            List<Double> values
            )
        {
            // Avoid LINQ to keep explicit style and allocation behavior.
            Double Best = values[0];
            for (Int32 i = 1; i < values.Count; i += 1)
            {
                if (values[i] > Best)
                {
                    Best = values[i];
                }
            }
            return Best;
        }

        /// <summary>
        /// Finds index in candidate set with minimum squared distance to point.
        /// </summary>
        /// <param name="candidates">Candidate indices.</param>
        /// <param name="cents">Centroid list.</param>
        /// <param name="point">Target point.</param>
        /// <returns>Best candidate index.</returns>
        static private Int32 ArgMinDistanceToPoint
            (
            HashSet<Int32> candidates,
            List<(Double X, Double Y)> cents,
            (Double X, Double Y) point
            )
        {
            // Keep deterministic tie behavior by preserving first-seen minimum.
            Int32 BestIndex = -1;
            Double BestDist = Double.MaxValue;
            foreach (Int32 Candidate in candidates)
            {
                Double Distance = D2(cents[Candidate], point);
                if (Distance < BestDist)
                {
                    BestDist = Distance;
                    BestIndex = Candidate;
                }
            }
            return BestIndex;
        }

        /// <summary>
        /// Finds index in candidate set with minimum squared distance to target centroid.
        /// </summary>
        /// <param name="candidates">Candidate indices.</param>
        /// <param name="cents">Centroid list.</param>
        /// <param name="target">Target centroid.</param>
        /// <returns>Best candidate index.</returns>
        static private Int32 ArgMinDistanceToCentroid
            (
            HashSet<Int32> candidates,
            List<(Double X, Double Y)> cents,
            (Double X, Double Y) target
            )
        {
            // Share nearest-neighbour selection behavior with point-anchored variant.
            Int32 BestIndex = -1;
            Double BestDist = Double.MaxValue;
            foreach (Int32 Candidate in candidates)
            {
                Double Distance = D2(cents[Candidate], target);
                if (Distance < BestDist)
                {
                    BestDist = Distance;
                    BestIndex = Candidate;
                }
            }
            return BestIndex;
        }
    }
}
