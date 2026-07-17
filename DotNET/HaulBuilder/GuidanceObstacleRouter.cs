using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Applies static and per-route obstacle clearance to an already connected
    /// forward-only lifecycle without changing any blade-down work geometry.
    /// </summary>
    static public class GuidanceObstacleRouter
    {
        /// <summary>Obstacle-routing outcome and audit counters.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one routing result.</summary>
            public Result(
                LifecyclePlanner.Result Lifecycle,
                Int32 StaticDetourCount,
                Int32 PivotDetourCount,
                Int32 ManualMoveCount,
                Double AddedDistanceM)
            {
                this.Lifecycle = Lifecycle;
                this.StaticDetourCount = StaticDetourCount;
                this.PivotDetourCount = PivotDetourCount;
                this.ManualMoveCount = ManualMoveCount;
                this.AddedDistanceM = AddedDistanceM;
            }

            /// <summary>Gets the obstacle-aware lifecycle.</summary>
            public LifecyclePlanner.Result Lifecycle { get; }

            /// <summary>Gets blade-up moves rerouted for static exclusions.</summary>
            public Int32 StaticDetourCount { get; }

            /// <summary>Gets blade-up moves rerouted for the parked pivot arm.</summary>
            public Int32 PivotDetourCount { get; }

            /// <summary>Gets risky moves explicitly assigned to the operator.</summary>
            public Int32 ManualMoveCount { get; }

            /// <summary>Gets added centerline distance relative to input guidance.</summary>
            public Double AddedDistanceM { get; }
        }

        /// <summary>
        /// Reroutes blocked blade-up segments. A forward-only, radius-valid path
        /// remains automatic; an unverified visibility fallback is marked manual.
        /// </summary>
        static public Result Apply(
            LifecyclePlanner.Result Source,
            Grid Grid,
            Double MinimumRadiusM,
            ObstacleField? StaticObstacles = null,
            IReadOnlyDictionary<Int32, ObstacleField>? PerRouteObstacles = null)
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            ObstacleField StaticField = StaticObstacles ?? EMPTY_OBSTACLES;
            if (!StaticField.Active && ((PerRouteObstacles == null)
                || (PerRouteObstacles.Count == 0)))
            {
                return new Result(Source, 0, 0, 0, 0.0);
            }

            Dictionary<ObstacleField, Grid> MaskedGrids =
                new Dictionary<ObstacleField, Grid>();
            List<LifecyclePlanner.Segment> Output =
                new List<LifecyclePlanner.Segment>();
            Int32 StaticDetours = 0;
            Int32 PivotDetours = 0;
            Int32 NewManualMoves = 0;

            foreach (LifecyclePlanner.Segment Segment in Source.Segments)
            {
                ObstacleField Field = StaticField;
                ObstacleField? RouteField = null;
                Boolean HasRouteField = PerRouteObstacles != null
                    && PerRouteObstacles.TryGetValue(
                        Segment.RouteNumber, out RouteField)
                    && RouteField != null;
                if (HasRouteField) Field = RouteField!;

                if (Segment.BladeDown || (Segment.Points.Count < 2)
                    || !Field.Active || !PolylineBlocked(Segment.Points, Field))
                {
                    Output.Add(Clone(Segment));
                    continue;
                }

                Boolean StaticBlocked = StaticField.Active
                    && PolylineBlocked(Segment.Points, StaticField);
                if (StaticBlocked) StaticDetours += 1;
                else PivotDetours += 1;

                if (!MaskedGrids.TryGetValue(Field, out Grid? Masked))
                {
                    Masked = BuildMaskedGrid(Grid, Field);
                    MaskedGrids[Field] = Masked;
                }

                ConnectorPlanner.Pose Start = EndpointPose(
                    Segment.Points, AtStart: true);
                ConnectorPlanner.Pose End = EndpointPose(
                    Segment.Points, AtStart: false);
                TransitPlanner.Result Transit = TransitPlanner.Connect(
                    Start,
                    End,
                    MinimumRadiusM,
                    Math.Min(Constants.HALF_SCALE, Grid.GridSizeM),
                    Masked,
                    AllowManualAssist: true,
                    EdgeAllowanceM: Grid.GridSizeM,
                    AllowForwardDetour: true);

                if (Transit.Success && (Transit.Legs.Count > 0)
                    && PolylineClear(Transit.Points, Field))
                {
                    foreach (TransitPlanner.Leg Leg in Transit.Legs)
                    {
                        LifecyclePlanner.Phase Phase =
                            Leg.OperatorAlignmentRequired
                                ? LifecyclePlanner.Phase.ManualAssist
                                : Segment.Phase;
                        if (Leg.OperatorAlignmentRequired) NewManualMoves += 1;
                        Output.Add(CreateSegment(
                            Segment, Phase, Leg.Points));
                    }
                    continue;
                }

                // Visibility routing gives the operator a concrete safe corridor
                // when the automatic forward-only solver cannot certify one.
                // If even that cannot be certified, retain the original endpoints
                // but label the entire move manual instead of issuing false guidance.
                List<(Double X, Double Y)> ManualPath =
                    BuildVisibilityPath(Segment.Points, Field);
                if (!PolylineClear(ManualPath, Field))
                {
                    ManualPath = new List<(Double X, Double Y)>(Segment.Points);
                }
                Output.Add(CreateSegment(
                    Segment, LifecyclePlanner.Phase.ManualAssist, ManualPath));
                NewManualMoves += 1;
            }

            Int32 ManualCount = Output.Count(Segment =>
                Segment.Phase == LifecyclePlanner.Phase.ManualAssist);
            Double DistanceM = TotalDistance(Output);
            LifecyclePlanner.Result Lifecycle = new LifecyclePlanner.Result(
                Source.Success,
                Output,
                ManualCount,
                DistanceM,
                Source.FailureReason);
            return new Result(
                Lifecycle,
                StaticDetours,
                PivotDetours,
                NewManualMoves,
                DistanceM - Source.DistanceM);
        }

        /// <summary>
        /// Executes the clone operation.
        /// </summary>
        /// <param name="Source">The source used by the operation.</param>
        /// <returns>The resulting clone.</returns>

        private static LifecyclePlanner.Segment Clone(
            LifecyclePlanner.Segment Source) => CreateSegment(
                Source, Source.Phase, new List<(Double X, Double Y)>(Source.Points));

        /// <summary>
        /// Builds segment.
        /// </summary>
        /// <param name="Source">The source used by the operation.</param>
        /// <param name="Phase">The phase used by the operation.</param>
        /// <param name="Points">The points used by the operation.</param>
        /// <returns>The resulting segment.</returns>

        private static LifecyclePlanner.Segment CreateSegment(
            LifecyclePlanner.Segment Source,
            LifecyclePlanner.Phase Phase,
            List<(Double X, Double Y)> Points)
        {
            LifecyclePlanner.Segment Result = new LifecyclePlanner.Segment(
                Phase,
                Phase == LifecyclePlanner.Phase.ManualAssist
                    ? false
                    : Source.BladeDown,
                Points,
                Source.Loaded);
            Result.RouteNumber = Source.RouteNumber;
            Result.Loaded = Source.Loaded;
            return Result;
        }

        /// <summary>
        /// Executes the endpoint pose operation.
        /// </summary>
        /// <param name="Points">The points used by the operation.</param>
        /// <param name="AtStart">The at start used by the operation.</param>
        /// <returns>The resulting endpoint pose.</returns>

        private static ConnectorPlanner.Pose EndpointPose(
            IReadOnlyList<(Double X, Double Y)> Points,
            Boolean AtStart)
        {
            Int32 A = AtStart ? 0 : Points.Count - 2;
            Int32 B = AtStart ? 1 : Points.Count - 1;
            return new ConnectorPlanner.Pose(
                AtStart ? Points[0].X : Points[Points.Count - 1].X,
                AtStart ? Points[0].Y : Points[Points.Count - 1].Y,
                BearingDeg(Points[A], Points[B]));
        }

        /// <summary>
        /// Builds visibility path.
        /// </summary>
        /// <param name="Source">The source used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <returns>The resulting visibility path.</returns>

        private static List<(Double X, Double Y)> BuildVisibilityPath(
            IReadOnlyList<(Double X, Double Y)> Source,
            ObstacleField Field)
        {
            List<(Double X, Double Y)> Output =
                new List<(Double X, Double Y)> { Source[0] };
            for (Int32 Index = 1; Index < Source.Count; Index += 1)
            {
                List<(Double X, Double Y)> Leg = Field.Path(
                    Output[Output.Count - 1], Source[Index]);
                for (Int32 PointIndex = 1;
                    PointIndex < Leg.Count;
                    PointIndex += 1)
                {
                    if (Distance(Output[Output.Count - 1], Leg[PointIndex]) > Constants.NUMERIC_COMPARISON_TOLERANCE)
                    {
                        Output.Add(Leg[PointIndex]);
                    }
                }
            }
            return Output;
        }

        /// <summary>
        /// Executes the polyline blocked operation.
        /// </summary>
        /// <param name="Points">The points used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <returns>The resulting polyline blocked.</returns>

        private static Boolean PolylineBlocked(
            IReadOnlyList<(Double X, Double Y)> Points,
            ObstacleField Field)
        {
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                if (Field.Blocked(Points[Index - 1], Points[Index])) return true;
            }
            return false;
        }

        /// <summary>
        /// Executes the polyline clear operation.
        /// </summary>
        /// <param name="Points">The points used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <returns>The resulting polyline clear.</returns>

        private static Boolean PolylineClear(
            IReadOnlyList<(Double X, Double Y)> Points,
            ObstacleField Field) => (Points.Count >= 2)
                && !PolylineBlocked(Points, Field);

        /// <summary>
        /// Builds masked grid.
        /// </summary>
        /// <param name="Source">The source used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <returns>The resulting masked grid.</returns>

        private static Grid BuildMaskedGrid(Grid Source, ObstacleField Field)
        {
            Boolean[,] Valid = (Boolean[,])Source.Valid.Clone();
            for (Int32 By = 0; By < Source.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Source.Width; Bx += 1)
                {
                    if (!Valid[By, Bx]) continue;
                    Double MinX = Source.MinX + (Bx * Source.GridSizeM);
                    Double MinY = Source.MinY + (By * Source.GridSizeM);
                    Double MaxX = MinX + Source.GridSizeM;
                    Double MaxY = MinY + Source.GridSizeM;
                    foreach (ExclusionZone Zone in Field.Zones)
                    {
                        if (CellIntersectsZone(MinX, MinY, MaxX, MaxY, Zone))
                        {
                            Valid[By, Bx] = false;
                            break;
                        }
                    }
                }
            }
            return new Grid(
                Source.GridSizeM,
                Source.Width,
                Source.Height,
                Source.MinX,
                Source.MinY,
                Source.Zone,
                Source.IsNorthern,
                Source.MeanLat,
                Source.MeanLon,
                Source.Existing,
                Source.Target,
                Valid,
                Source.SwellFactor,
                Source.Compaction,
                Source.MaxDepthMm,
                Source.Exclusions.Select(Ring =>
                    new List<(Double X, Double Y)>(Ring)).ToList());
        }

        /// <summary>
        /// Executes the cell intersects zone operation.
        /// </summary>
        /// <param name="MinX">Horizontal coordinate in field units.</param>
        /// <param name="MinY">Vertical coordinate in field units.</param>
        /// <param name="MaxX">Horizontal coordinate in field units.</param>
        /// <param name="MaxY">Vertical coordinate in field units.</param>
        /// <param name="Zone">The zone used by the operation.</param>
        /// <returns>The resulting cell intersects zone.</returns>

        private static Boolean CellIntersectsZone(
            Double MinX,
            Double MinY,
            Double MaxX,
            Double MaxY,
            ExclusionZone Zone)
        {
            if ((MaxX < Zone.Bbox.MinX) || (MinX > Zone.Bbox.MaxX)
                || (MaxY < Zone.Bbox.MinY) || (MinY > Zone.Bbox.MaxY))
            {
                return false;
            }
            (Double X, Double Y)[] Corners =
            {
                (MinX, MinY), (MaxX, MinY),
                (MaxX, MaxY), (MinX, MaxY),
            };
            if (Corners.Any(Point => Obstacles.PointInRing(
                    Point.X, Point.Y, Zone.BufferRing))
                || Obstacles.PointInRing(
                    Constants.HALF_SCALE * (MinX + MaxX),
                    Constants.HALF_SCALE * (MinY + MaxY),
                    Zone.BufferRing))
            {
                return true;
            }
            if (Zone.BufferRing.Any(Point =>
                (Point.X >= MinX) && (Point.X <= MaxX)
                && (Point.Y >= MinY) && (Point.Y <= MaxY)))
            {
                return true;
            }
            for (Int32 Index = 0; Index < Corners.Length; Index += 1)
            {
                if (Zone.Blocks(Corners[Index], Corners[(Index + 1) % Corners.Length]))
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Executes the total distance operation.
        /// </summary>
        /// <param name="Segments">The segments used by the operation.</param>
        /// <returns>The resulting total distance.</returns>

        private static Double TotalDistance(
            IEnumerable<LifecyclePlanner.Segment> Segments) => Segments.Sum(
                Segment => PolylineDistance(Segment.Points));

        /// <summary>
        /// Executes the polyline distance operation.
        /// </summary>
        /// <param name="Points">The points used by the operation.</param>
        /// <returns>The resulting polyline distance.</returns>

        private static Double PolylineDistance(
            IReadOnlyList<(Double X, Double Y)> Points)
        {
            Double TotalM = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                TotalM += Distance(Points[Index - 1], Points[Index]);
            }
            return TotalM;
        }

        /// <summary>
        /// Executes the bearing deg operation.
        /// </summary>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <returns>The resulting bearing deg.</returns>

        private static Double BearingDeg(
            (Double X, Double Y) A,
            (Double X, Double Y) B) =>
            ((Math.Atan2(B.X - A.X, B.Y - A.Y) * Constants.DEGREES_PER_HALF_TURN / Math.PI)
                + Constants.DEGREES_PER_TURN) % Constants.DEGREES_PER_TURN;

        /// <summary>
        /// Executes the distance operation.
        /// </summary>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <returns>The resulting distance.</returns>

        private static Double Distance(
            (Double X, Double Y) A,
            (Double X, Double Y) B)
        {
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }
    }
}
