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
        private const Double EPSILON = 1e-12;
        private const Double DEFAULT_TURNING_CIRCLE_FT = 45.0;

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

                    if (CurrentHairpins >= hairpinMin)
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

                        if (HardTurnCount(BLane, grid, BladeM) < CurrentHairpins)
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

                    if (CurrentHairpins >= hairpinMin)
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

                        if (HardTurnCount(BLane, grid, BladeM) < CurrentHairpins)
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
            Dictionary<Int32, ObstacleField>? armObstacles = null
            )
        {
            // Preserve Python accounting model: metrics use raw coverage path, not smoothed display path.
            Double BladeM = bladeWidthMm / 1000.0;
            ObstacleField Obstacles = obstacles ?? EMPTY_OBSTACLES;
            Double HopThresh = Math.Max(3.0 * grid.GridSizeM, 2.0 * BladeM);
            Double EffectiveTurnRadiusM = turnRadiusM.HasValue ? turnRadiusM.Value : (DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS);
            Double SigmaIdx = smoothRoute ? ((EffectiveTurnRadiusM / grid.GridSizeM) * smoothScale) : 0.0;

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
            Int32 Seq = 0;

            for (Int32 Index = 0; Index < Raw.Count; Index += 1)
            {
                RawPoint Entry = Raw[Index];
                (Double X, Double Y) RawPointXy = (Entry.Xr, Entry.Yr);
                Double Seg = PrevRaw.HasValue ? Dist(PrevRaw.Value, RawPointXy) : 0.0;

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
                else if ((Point.Action == HAUL) || (Point.Action == MOVE))
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
        /// Decomposes a patch into principal-axis blade-wide lane groups.
        /// </summary>
        /// <param name="cells">Patch cells with depth.</param>
        /// <param name="grid">Grid geometry.</param>
        /// <param name="bladeM">Blade width in metres.</param>
        /// <returns>List of lane cell groups sorted by cross-axis order.</returns>
        static private List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> PatchDrivableLanes
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
            Grid grid,
            Double bladeM
            )
        {
            // Build straight drivable lane groups so boustrophedon can avoid knot-like U-turn clustering.
            if (cells.Count <= 1)
            {
                return new List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> { new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(cells) };
            }

            List<(Double X, Double Y)> Centers = new List<(Double X, Double Y)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in cells)
            {
                Centers.Add(grid.CellCenterXY(Entry.Cell.Bx, Entry.Cell.By));
            }

            Double Heading = PrincipalHeadingDeg(Centers);
            Double Theta = DegreesToRadians(Heading);
            (Double X, Double Y) Uh = (Math.Sin(Theta), Math.Cos(Theta));
            (Double X, Double Y) Vh = (Math.Cos(Theta), -Math.Sin(Theta));
            (Double X, Double Y) Origin = Centroid(Centers);

            List<Double> Us = new List<Double>();
            List<Double> Vs = new List<Double>();
            foreach ((Double X, Double Y) Point in Centers)
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
            return (action == START) || (action == RETURN) || (action == HAUL) || (action == MOVE);
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
