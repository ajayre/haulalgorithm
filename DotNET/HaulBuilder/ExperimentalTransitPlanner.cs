using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder
{
    /// <summary>
    /// Builds radius-valid experimental transit with explicit operator-alignment fallback.
    /// </summary>
    public static class ExperimentalTransitPlanner
    {
        private const Double AUTOMATIC_ALIGNMENT_TOLERANCE_DEG = 5.0;
        private const Double MAX_MANUAL_HEADING_CORRECTION_DEG = 30.0;
        private static readonly Object AllowanceCacheLock = new Object();
        private static readonly Dictionary<(Grid Field, Int64 AllowanceBits),
            (ExperimentalFieldEdgePolicy Policy, Grid AllowanceGrid)> AllowanceCache =
                new Dictionary<(Grid Field, Int64 AllowanceBits),
                    (ExperimentalFieldEdgePolicy Policy, Grid AllowanceGrid)>();
        /// <summary>
        /// One automatic or operator-controlled portion of a transit.
        /// </summary>
        public sealed class Leg
        {
            /// <summary>Initializes one transit leg.</summary>
            public Leg
                (
                Boolean OperatorAlignmentRequired,
                List<(Double X, Double Y)> Points
                )
            {
                this.OperatorAlignmentRequired = OperatorAlignmentRequired;
                this.Points = Points;
            }

            /// <summary>Gets whether the operator controls this short alignment leg.</summary>
            public Boolean OperatorAlignmentRequired { get; }

            /// <summary>Gets the leg centerline points.</summary>
            public List<(Double X, Double Y)> Points { get; }
        }

        /// <summary>
        /// Outcome of one experimental reposition segment.
        /// </summary>
        public sealed class Result
        {
            /// <summary>Initializes one transit result.</summary>
            public Result
                (
                Boolean Success,
                Boolean OperatorAlignmentRequired,
                List<(Double X, Double Y)> Points,
                Double LengthM,
                List<Leg> Legs,
                Double ManualAssistDistanceM,
                String Mode,
                String FailureReason,
                Boolean LeavesField = false
                )
            {
                this.Success = Success;
                this.OperatorAlignmentRequired = OperatorAlignmentRequired;
                this.Points = Points;
                this.LengthM = LengthM;
                this.Legs = Legs;
                this.ManualAssistDistanceM = ManualAssistDistanceM;
                this.Mode = Mode;
                this.FailureReason = FailureReason;
                this.LeavesField = LeavesField;
            }

            /// <summary>Gets whether a wholly in-field nominal transit exists.</summary>
            public Boolean Success { get; }

            /// <summary>Gets whether the operator must perform endpoint alignment.</summary>
            public Boolean OperatorAlignmentRequired { get; }

            /// <summary>Gets sampled nominal centerline points.</summary>
            public List<(Double X, Double Y)> Points { get; }

            /// <summary>Gets nominal transit distance.</summary>
            public Double LengthM { get; }

            /// <summary>Gets automatic and manual portions in drive order.</summary>
            public List<Leg> Legs { get; }

            /// <summary>Gets total distance assigned to short operator-alignment zones.</summary>
            public Double ManualAssistDistanceM { get; }

            /// <summary>Gets the selected transit mode.</summary>
            public String Mode { get; }

            /// <summary>Gets the explicit failure reason.</summary>
            public String FailureReason { get; }

            /// <summary>Gets whether this connector enters the permitted one-cell exterior strip.</summary>
            public Boolean LeavesField { get; }
        }

        /// <summary>
        /// Uses an in-field automatic curve when possible, otherwise a safe positional transit.
        /// </summary>
        static public Result Connect
            (
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AllowManualAssist = true,
            Double MaxManualAssistM = 45.0 * Constants.FEET_TO_METERS,
            Double EdgeAllowanceM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0,
            Boolean AllowKeyholeHybrid = false
            )
        {
            List<(Double X, Double Y)> Gentle = GentleBend(
                Start, End, MinimumRadiusM, SampleSpacingM, Field);
            Double GentleLengthM = Gentle.Count > 0 ? PolylineLength(Gentle) : Double.PositiveInfinity;
            ExperimentalConnectorPlanner.Result Automatic = ExperimentalConnectorPlanner.Connect(
                Start, End, MinimumRadiusM, SampleSpacingM, Field);
            if ((Gentle.Count > 0)
                && DensePolylineAllowed(Gentle, Field, null)
                && (!Automatic.Success || (GentleLengthM + 1e-9 < Automatic.LengthM)))
            {
                return new Result(true, false, Gentle,
                    GentleLengthM,
                    new List<Leg> { new Leg(false, Gentle) },
                    0.0,
                    "AUTO_BEND", String.Empty);
            }
            if (Automatic.Success && DensePolylineAllowed(Automatic.Points, Field, null))
            {
                return new Result(true, false, Automatic.Points,
                    Automatic.LengthM,
                    new List<Leg> { new Leg(false, Automatic.Points) },
                    0.0,
                    "AUTO_CURVE", String.Empty);
            }

            if (!AllowManualAssist)
            {
                return new Result(false, false, new List<(Double X, Double Y)>(), 0.0,
                    new List<Leg>(), 0.0,
                    "FAILED", "No compact automatic connector exists and manual assist is disabled.");
            }

            Result? InFieldFallback = null;
            List<(Double X, Double Y)> Direct = SampleLine(
                (Start.X, Start.Y), (End.X, End.Y), SampleSpacingM);
            if (DensePolylineAllowed(Direct, Field, null))
            {
                Result DirectResult = AlignmentResult(
                    Direct,
                    Start,
                    End,
                    MinimumRadiusM,
                    MaxManualAssistM,
                    3.0 * MinimumRadiusM,
                    "OPERATOR_ALIGN");
                if (!DirectResult.OperatorAlignmentRequired) return DirectResult;
                InFieldFallback = DirectResult;
            }

            if (InFieldFallback == null)
            {
                List<(Double X, Double Y)> Routed = GridRoute(
                    (Start.X, Start.Y), (End.X, End.Y), SampleSpacingM, Field);
                if (Routed.Count > 0)
                {
                    Routed = SmoothRadiusValidCorners(
                        Routed, MinimumRadiusM, SampleSpacingM, Field);
                    Result RoutedResult = AlignmentResult(
                        Routed, Start, End, MinimumRadiusM, MaxManualAssistM, 0.0,
                        "OPERATOR_ALIGN_ROUTED");
                    if (!RoutedResult.OperatorAlignmentRequired) return RoutedResult;
                    InFieldFallback = RoutedResult;
                }
            }

            Double MinimumKeyholePullM = 0.5
                * Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                * Constants.FEET_TO_METERS;
            Result? Keyhole = TryManualKeyhole(
                Start,
                End,
                SampleSpacingM,
                MinimumKeyholePullM,
                MaxManualAssistM,
                Field,
                null);
            if (Keyhole != null) return Keyhole;

            if (AllowKeyholeHybrid)
            {
                Result? Hybrid = TryEndpointKeyholeHybrid(
                    Start,
                    End,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    MaxManualAssistM,
                    EdgeAllowanceM);
                if (Hybrid != null) return Hybrid;
            }

            Result? Exterior = TryExteriorConnector(
                Start, End, MinimumRadiusM, SampleSpacingM, Field, EdgeAllowanceM);
            if (Exterior != null) return Exterior;
            if (InFieldFallback != null) return InFieldFallback;

            return new Result(false, true, new List<(Double X, Double Y)>(), 0.0,
                new List<Leg>(), 0.0,
                "FAILED", "No nominal in-field positional transit exists.");
        }

        /// <summary>
        /// Builds a local pull-forward/reverse maneuver with endpoint-aligned legs.
        /// Detailed vehicle articulation is deliberately left to the manual operator zone.
        /// </summary>
        private static Result? TryManualKeyhole
            (
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Double SampleSpacingM,
            Double MinimumPullM,
            Double MaximumLengthM,
            Grid Field,
            ExperimentalFieldEdgePolicy? ExteriorPolicy
            )
        {
            Double StartRad = Start.HeadingDeg * Math.PI / 180.0;
            Double EndRad = End.HeadingDeg * Math.PI / 180.0;
            (Double X, Double Y) StartUnit =
                (Math.Sin(StartRad), Math.Cos(StartRad));
            (Double X, Double Y) EndUnit =
                (Math.Sin(EndRad), Math.Cos(EndRad));
            List<(Double X, Double Y)> Best = new List<(Double X, Double Y)>();
            Double BestLengthM = Double.PositiveInfinity;
            String BestMode = "MANUAL_KEYHOLE";

            void ConsiderPoints(List<(Double X, Double Y)> Points, String Mode)
            {
                if (Points.Count < 3) return;
                Double StartErrorDeg = HeadingDifferenceDeg(
                    Start.HeadingDeg, BearingDeg(Points[0], Points[1]));
                Double EndErrorDeg = HeadingDifferenceDeg(
                    BearingDeg(Points[Points.Count - 2], Points[Points.Count - 1]),
                    End.HeadingDeg);
                Double MaximumReversalDeg = 0.0;
                for (Int32 Index = 1; Index + 1 < Points.Count; Index += 1)
                {
                    MaximumReversalDeg = Math.Max(
                        MaximumReversalDeg,
                        CornerDeflectionDeg(
                            Points[Index - 1], Points[Index], Points[Index + 1]));
                }
                if ((StartErrorDeg > MAX_MANUAL_HEADING_CORRECTION_DEG)
                    || (EndErrorDeg > MAX_MANUAL_HEADING_CORRECTION_DEG)
                    || (MaximumReversalDeg < 90.0))
                {
                    return;
                }
                if (!DensePolylineAllowed(Points, Field, ExteriorPolicy)) return;
                Double LengthM = PolylineLength(Points);
                if (LengthM > MaximumLengthM + 1e-9) return;
                if (LengthM >= BestLengthM - 1e-9) return;
                Best = Points;
                BestLengthM = LengthM;
                BestMode = Mode;
            }

            void ConsiderCusp((Double X, Double Y) Cusp)
            {
                List<(Double X, Double Y)> First = SampleLine(
                    (Start.X, Start.Y), Cusp, SampleSpacingM);
                List<(Double X, Double Y)> Second = SampleLine(
                    Cusp, (End.X, End.Y), SampleSpacingM);
                if ((First.Count < 2) || (Second.Count < 2)) return;
                List<(Double X, Double Y)> Points =
                    new List<(Double X, Double Y)>(First);
                Points.AddRange(Second.Skip(1));
                ConsiderPoints(Points, "MANUAL_KEYHOLE");
            }

            void ConsiderThreePoint(Double PullM, Double Side)
            {
                (Double X, Double Y) Perpendicular =
                    (-StartUnit.Y, StartUnit.X);
                Double LateralM = 0.18 * PullM * Side;
                (Double X, Double Y) Forward = (
                    Start.X + (StartUnit.X * PullM),
                    Start.Y + (StartUnit.Y * PullM));
                (Double X, Double Y) Reverse = (
                    Start.X + (StartUnit.X * 0.55 * PullM)
                        + (Perpendicular.X * LateralM),
                    Start.Y + (StartUnit.Y * 0.55 * PullM)
                        + (Perpendicular.Y * LateralM));
                List<(Double X, Double Y)> First = SampleLine(
                    (Start.X, Start.Y), Forward, SampleSpacingM);
                List<(Double X, Double Y)> Second = SampleLine(
                    Forward, Reverse, SampleSpacingM);
                List<(Double X, Double Y)> Third = SampleLine(
                    Reverse, (End.X, End.Y), SampleSpacingM);
                List<(Double X, Double Y)> Points =
                    new List<(Double X, Double Y)>(First);
                Points.AddRange(Second.Skip(1));
                Points.AddRange(Third.Skip(1));
                ConsiderPoints(Points, "MANUAL_THREE_POINT_KEYHOLE");
            }

            Double DeltaX = End.X - Start.X;
            Double DeltaY = End.Y - Start.Y;
            Double Determinant = (StartUnit.X * EndUnit.Y)
                - (StartUnit.Y * EndUnit.X);
            if (Math.Abs(Determinant) > 1e-9)
            {
                Double ForwardM = ((DeltaX * EndUnit.Y) - (DeltaY * EndUnit.X))
                    / Determinant;
                Double ReverseM = ((StartUnit.X * DeltaY) - (StartUnit.Y * DeltaX))
                    / Determinant;
                if ((ForwardM >= MinimumPullM - 1e-9)
                    && (ReverseM >= MinimumPullM - 1e-9))
                {
                    ConsiderCusp((
                        Start.X + (StartUnit.X * ForwardM),
                        Start.Y + (StartUnit.Y * ForwardM)));
                }
            }

            Double StepM = Math.Max(SampleSpacingM, Field.GridSizeM * 0.25);
            Double MaximumM = 2.0 * Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                * Constants.FEET_TO_METERS;
            Double PositionDistanceM = Distance(
                (Start.X, Start.Y), (End.X, End.Y));
            Boolean CoincidentOpposing = PositionDistanceM <= SampleSpacingM
                && HeadingDifferenceDeg(Start.HeadingDeg, End.HeadingDeg) >= 150.0;
            for (Double DistanceM = Math.Max(MinimumPullM, StepM);
                DistanceM <= MaximumM + 1e-9;
                DistanceM += StepM)
            {
                if (CoincidentOpposing)
                {
                    ConsiderThreePoint(DistanceM, -1.0);
                    ConsiderThreePoint(DistanceM, 1.0);
                    continue;
                }
                ConsiderCusp((
                    End.X - (EndUnit.X * DistanceM),
                    End.Y - (EndUnit.Y * DistanceM)));
                ConsiderCusp((
                    Start.X + (StartUnit.X * DistanceM),
                    Start.Y + (StartUnit.Y * DistanceM)));
            }
            if (Best.Count == 0) return null;
            return new Result(
                true,
                true,
                Best,
                BestLengthM,
                new List<Leg> { new Leg(true, Best) },
                BestLengthM,
                BestMode,
                String.Empty);
        }

        /// <summary>
        /// Adds a local three-point reversal before or after a long transit when an
        /// endpoint faces substantially away from the required direction of travel.
        /// </summary>
        private static Result? TryEndpointKeyholeHybrid
            (
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Double MaxManualAssistM,
            Double EdgeAllowanceM
            )
        {
            Double DirectM = Distance((Start.X, Start.Y), (End.X, End.Y));
            if (DirectM <= SampleSpacingM) return null;
            Double DirectHeadingDeg = BearingDeg(
                (Start.X, Start.Y), (End.X, End.Y));
            Boolean ReverseStart = HeadingDifferenceDeg(
                Start.HeadingDeg, DirectHeadingDeg) > 60.0;
            Boolean ReverseEnd = HeadingDifferenceDeg(
                DirectHeadingDeg, End.HeadingDeg) > 60.0;
            if (!ReverseStart && !ReverseEnd) return null;

            ExperimentalConnectorPlanner.Pose FlippedStart =
                new ExperimentalConnectorPlanner.Pose(
                    Start.X, Start.Y, Start.HeadingDeg + 180.0);
            ExperimentalConnectorPlanner.Pose FlippedEnd =
                new ExperimentalConnectorPlanner.Pose(
                    End.X, End.Y, End.HeadingDeg + 180.0);
            Result? StartTurn = null;
            Result? EndTurn = null;
            List<Result> Candidates = new List<Result>();
            if (ReverseStart)
            {
                StartTurn = Connect(
                    Start,
                    FlippedStart,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    true,
                    MaxManualAssistM,
                    EdgeAllowanceM,
                    false);
                if (StartTurn.Success)
                {
                    Result Tail = Connect(
                        FlippedStart,
                        End,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        true,
                        MaxManualAssistM,
                        EdgeAllowanceM,
                        false);
                    if (Tail.Success)
                    {
                        Candidates.Add(Combine(
                            new[] { StartTurn, Tail },
                            "START_KEYHOLE_HYBRID"));
                    }
                }
            }
            if (ReverseEnd)
            {
                EndTurn = Connect(
                    FlippedEnd,
                    End,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    true,
                    MaxManualAssistM,
                    EdgeAllowanceM,
                    false);
                if (EndTurn.Success)
                {
                    Result Head = Connect(
                        Start,
                        FlippedEnd,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        true,
                        MaxManualAssistM,
                        EdgeAllowanceM,
                        false);
                    if (Head.Success)
                    {
                        Candidates.Add(Combine(
                            new[] { Head, EndTurn },
                            "END_KEYHOLE_HYBRID"));
                    }
                }
            }
            if (ReverseStart
                && ReverseEnd
                && (StartTurn?.Success ?? false)
                && (EndTurn?.Success ?? false))
            {
                Result Middle = Connect(
                    FlippedStart,
                    FlippedEnd,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    true,
                    MaxManualAssistM,
                    EdgeAllowanceM,
                    false);
                if (Middle.Success)
                {
                    Candidates.Add(Combine(
                        new[] { StartTurn!, Middle, EndTurn! },
                        "BOTH_KEYHOLE_HYBRID"));
                }
            }
            return Candidates
                .OrderBy(Candidate => Candidate.LengthM)
                .ThenBy(Candidate => Candidate.Mode, StringComparer.Ordinal)
                .FirstOrDefault();
        }

        /// <summary>Combines contiguous validated connectors into one lifecycle transit.</summary>
        private static Result Combine
            (
            IEnumerable<Result> Parts,
            String Mode
            )
        {
            List<Result> Ordered = Parts.ToList();
            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            List<Leg> Legs = new List<Leg>();
            foreach (Result Part in Ordered)
            {
                if (Points.Count == 0)
                {
                    Points.AddRange(Part.Points);
                }
                else
                {
                    Points.AddRange(Part.Points.Skip(1));
                }
                Legs.AddRange(Part.Legs);
            }
            return new Result(
                true,
                Ordered.Any(Part => Part.OperatorAlignmentRequired),
                Points,
                Ordered.Sum(Part => Part.LengthM),
                Legs,
                Ordered.Sum(Part => Part.ManualAssistDistanceM),
                Mode,
                String.Empty,
                Ordered.Any(Part => Part.LeavesField));
        }

        /// <summary>
        /// Proposes a radius-valid curve in the one-cell outer strip after all wholly in-field guidance fails.
        /// </summary>
        private static Result? TryExteriorConnector
            (
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Double EdgeAllowanceM
            )
        {
            (ExperimentalFieldEdgePolicy Policy, Grid AllowanceGrid) =
                GetAllowanceContext(Field, EdgeAllowanceM);
            if (!Policy.IsFieldPointNearOuterEdge(Start.X, Start.Y)
                && !Policy.IsFieldPointNearOuterEdge(End.X, End.Y))
            {
                return null;
            }
            List<(Double X, Double Y)> Gentle = GentleBend(
                Start, End, MinimumRadiusM, SampleSpacingM, AllowanceGrid);
            ExperimentalConnectorPlanner.Result Curve = ExperimentalConnectorPlanner.Connect(
                Start, End, MinimumRadiusM, SampleSpacingM, AllowanceGrid);
            List<(Double X, Double Y)> Selected = new List<(Double X, Double Y)>();
            Double SelectedLengthM = Double.PositiveInfinity;
            if ((Gentle.Count > 0)
                && Gentle.Any(Point => Policy.IsPermittedExterior(Point.X, Point.Y))
                && DensePolylineAllowed(Gentle, Field, Policy))
            {
                Selected = Gentle;
                SelectedLengthM = PolylineLength(Gentle);
            }
            if (Curve.Success
                && Curve.Points.Any(Point => Policy.IsPermittedExterior(Point.X, Point.Y))
                && DensePolylineAllowed(Curve.Points, Field, Policy)
                && (Curve.LengthM < SelectedLengthM - 1e-9))
            {
                Selected = Curve.Points;
                SelectedLengthM = Curve.LengthM;
            }
            if (Selected.Count == 0)
            {
                Result? Keyhole = TryManualKeyhole(
                    Start,
                    End,
                    SampleSpacingM,
                    0.5 * Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                        * Constants.FEET_TO_METERS,
                    Double.PositiveInfinity,
                    Field,
                    Policy);
                if ((Keyhole != null)
                    && Keyhole.Points.Any(
                        Point => Policy.IsPermittedExterior(Point.X, Point.Y)))
                {
                    return new Result(
                        true,
                        true,
                        Keyhole.Points,
                        Keyhole.LengthM,
                        Keyhole.Legs,
                        Keyhole.ManualAssistDistanceM,
                        "EXTERIOR_MANUAL_KEYHOLE",
                        String.Empty,
                        true);
                }
                return null;
            }
            return new Result(
                true,
                true,
                Selected,
                SelectedLengthM,
                new List<Leg> { new Leg(true, Selected) },
                SelectedLengthM,
                "EXTERIOR_MANUAL_CURVE",
                String.Empty,
                true);
        }

        /// <summary>Reuses immutable exterior masks during sequence-cost evaluation.</summary>
        private static (ExperimentalFieldEdgePolicy Policy, Grid AllowanceGrid) GetAllowanceContext
            (
            Grid Field,
            Double EdgeAllowanceM
            )
        {
            (Grid Field, Int64 AllowanceBits) Key =
                (Field, BitConverter.DoubleToInt64Bits(EdgeAllowanceM));
            lock (AllowanceCacheLock)
            {
                if (AllowanceCache.TryGetValue(Key, out
                    (ExperimentalFieldEdgePolicy Policy, Grid AllowanceGrid) Existing))
                {
                    return Existing;
                }
                ExperimentalFieldEdgePolicy Policy =
                    new ExperimentalFieldEdgePolicy(Field, EdgeAllowanceM);
                (ExperimentalFieldEdgePolicy Policy, Grid AllowanceGrid) Created =
                    (Policy, Policy.BuildExteriorAllowanceGrid());
                AllowanceCache[Key] = Created;
                return Created;
            }
        }

        /// <summary>
        /// Fits a smooth endpoint-tangent bend and accepts it only when sampled curvature and field checks pass.
        /// </summary>
        private static List<(Double X, Double Y)> GentleBend
            (
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            Double DirectM = Distance((Start.X, Start.Y), (End.X, End.Y));
            if (DirectM <= SampleSpacingM) return new List<(Double X, Double Y)>();
            Double DirectHeadingDeg = Math.Atan2(
                End.X - Start.X,
                End.Y - Start.Y) * 180.0 / Math.PI;
            if ((HeadingDifferenceDeg(Start.HeadingDeg, DirectHeadingDeg) > 45.0)
                || (HeadingDifferenceDeg(End.HeadingDeg, DirectHeadingDeg) > 45.0)
                || (HeadingDifferenceDeg(Start.HeadingDeg, End.HeadingDeg) > 45.0))
            {
                return new List<(Double X, Double Y)>();
            }

            Double StartRad = Start.HeadingDeg * Math.PI / 180.0;
            Double EndRad = End.HeadingDeg * Math.PI / 180.0;
            (Double X, Double Y) StartUnit = (Math.Sin(StartRad), Math.Cos(StartRad));
            (Double X, Double Y) EndUnit = (Math.Sin(EndRad), Math.Cos(EndRad));
            List<Double> Controls = new[]
                {
                    DirectM / 3.0,
                    DirectM / 2.0,
                    0.75 * DirectM,
                    DirectM,
                    Math.Max(MinimumRadiusM, DirectM / 3.0),
                }
                .Where(Value => Value > 1e-6)
                .Distinct()
                .OrderBy(Value => Value)
                .ToList();
            List<(Double X, Double Y)> Best = new List<(Double X, Double Y)>();
            Double BestLengthM = Double.PositiveInfinity;
            foreach (Double ControlM in Controls)
            {
                (Double X, Double Y) P0 = (Start.X, Start.Y);
                (Double X, Double Y) P1 = (
                    Start.X + (StartUnit.X * ControlM),
                    Start.Y + (StartUnit.Y * ControlM));
                (Double X, Double Y) P2 = (
                    End.X - (EndUnit.X * ControlM),
                    End.Y - (EndUnit.Y * ControlM));
                (Double X, Double Y) P3 = (End.X, End.Y);
                Double ControlPolygonM = Distance(P0, P1)
                    + Distance(P1, P2)
                    + Distance(P2, P3);
                Int32 Samples = Math.Max(
                    8,
                    (Int32)Math.Ceiling(ControlPolygonM / SampleSpacingM));
                List<(Double X, Double Y)> Points =
                    new List<(Double X, Double Y)>(Samples + 1);
                Boolean Valid = true;
                for (Int32 Index = 0; Index <= Samples; Index += 1)
                {
                    Double T = (Double)Index / Samples;
                    Double OneMinusT = 1.0 - T;
                    Double B0 = OneMinusT * OneMinusT * OneMinusT;
                    Double B1 = 3.0 * OneMinusT * OneMinusT * T;
                    Double B2 = 3.0 * OneMinusT * T * T;
                    Double B3 = T * T * T;
                    (Double X, Double Y) Point = (
                        (B0 * P0.X) + (B1 * P1.X) + (B2 * P2.X) + (B3 * P3.X),
                        (B0 * P0.Y) + (B1 * P1.Y) + (B2 * P2.Y) + (B3 * P3.Y));
                    if (!Field.ContainsWorldXy(Point.X, Point.Y)
                        || !CurvatureValid(
                            T, P0, P1, P2, P3, MinimumRadiusM))
                    {
                        Valid = false;
                        break;
                    }
                    Points.Add(Point);
                }
                if (!Valid) continue;
                Double LengthM = PolylineLength(Points);
                if (LengthM + 1e-9 < BestLengthM)
                {
                    Best = Points;
                    BestLengthM = LengthM;
                }
            }
            return Best;
        }

        /// <summary>Returns the smallest absolute compass-heading difference.</summary>
        private static Double HeadingDifferenceDeg
            (
            Double First,
            Double Second
            )
        {
            Double Difference = ((First - Second + 540.0) % 360.0) - 180.0;
            return Math.Abs(Difference);
        }

        /// <summary>Checks cubic Bezier curvature against the configured minimum radius.</summary>
        private static Boolean CurvatureValid
            (
            Double T,
            (Double X, Double Y) P0,
            (Double X, Double Y) P1,
            (Double X, Double Y) P2,
            (Double X, Double Y) P3,
            Double MinimumRadiusM
            )
        {
            Double OneMinusT = 1.0 - T;
            Double Dx = 3.0 * (
                (OneMinusT * OneMinusT * (P1.X - P0.X))
                + (2.0 * OneMinusT * T * (P2.X - P1.X))
                + (T * T * (P3.X - P2.X)));
            Double Dy = 3.0 * (
                (OneMinusT * OneMinusT * (P1.Y - P0.Y))
                + (2.0 * OneMinusT * T * (P2.Y - P1.Y))
                + (T * T * (P3.Y - P2.Y)));
            Double Ddx = 6.0 * (
                (OneMinusT * (P2.X - (2.0 * P1.X) + P0.X))
                + (T * (P3.X - (2.0 * P2.X) + P1.X)));
            Double Ddy = 6.0 * (
                (OneMinusT * (P2.Y - (2.0 * P1.Y) + P0.Y))
                + (T * (P3.Y - (2.0 * P2.Y) + P1.Y)));
            Double SpeedSquared = (Dx * Dx) + (Dy * Dy);
            if (SpeedSquared <= 1e-12) return false;
            Double Curvature = Math.Abs((Dx * Ddy) - (Dy * Ddx))
                / Math.Pow(SpeedSquared, 1.5);
            return Curvature <= (1.0 / MinimumRadiusM) * 1.001;
        }

        /// <summary>
        /// Splits a positional fallback into straight automatic legs and bounded manual corrections.
        /// </summary>
        private static Result AlignmentResult
            (
            List<(Double X, Double Y)> Points,
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Double MinimumRadiusM,
            Double MaxManualAssistM,
            Double MinimumHybridLengthM,
            String Mode
            )
        {
            Double LengthM = PolylineLength(Points);
            if (Points.Count < 2)
            {
                return new Result(false, false, new List<(Double X, Double Y)>(), 0.0,
                    new List<Leg>(), 0.0, "FAILED", "Positional fallback has no driven segment.");
            }

            List<(Double FromM, Double ToM)> ManualIntervals =
                new List<(Double FromM, Double ToM)>();
            Double CorrectionM = Math.Min(MinimumRadiusM * 0.5, MaxManualAssistM * 0.5);
            Double StartErrorDeg = HeadingDifferenceDeg(
                Start.HeadingDeg, BearingDeg(Points[0], Points[1]));
            Double EndErrorDeg = HeadingDifferenceDeg(
                BearingDeg(Points[Points.Count - 2], Points[Points.Count - 1]),
                End.HeadingDeg);
            if ((StartErrorDeg > MAX_MANUAL_HEADING_CORRECTION_DEG)
                || (EndErrorDeg > MAX_MANUAL_HEADING_CORRECTION_DEG))
            {
                return new Result(false, true, new List<(Double X, Double Y)>(), 0.0,
                    new List<Leg>(), 0.0,
                    "FAILED",
                    String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "Required endpoint heading correction ({0:F1}/{1:F1} deg) exceeds the {2:F1} deg operator limit.",
                        StartErrorDeg,
                        EndErrorDeg,
                        MAX_MANUAL_HEADING_CORRECTION_DEG));
            }
            if (StartErrorDeg > AUTOMATIC_ALIGNMENT_TOLERANCE_DEG)
            {
                ManualIntervals.Add((0.0, Math.Min(CorrectionM, LengthM)));
            }
            if (EndErrorDeg > AUTOMATIC_ALIGNMENT_TOLERANCE_DEG)
            {
                ManualIntervals.Add((Math.Max(0.0, LengthM - CorrectionM), LengthM));
            }
            Double CumulativeM = 0.0;
            for (Int32 Index = 1; Index < Points.Count - 1; Index += 1)
            {
                CumulativeM += Distance(Points[Index - 1], Points[Index]);
                Double DeflectionDeg = CornerDeflectionDeg(
                    Points[Index - 1], Points[Index], Points[Index + 1]);
                if (DeflectionDeg <= AUTOMATIC_ALIGNMENT_TOLERANCE_DEG) continue;
                Double ObservedRadiusM = ThreePointRadius(
                    Points[Index - 1], Points[Index], Points[Index + 1]);
                if (ObservedRadiusM >= MinimumRadiusM * 0.999) continue;
                ManualIntervals.Add((
                    Math.Max(0.0, CumulativeM - CorrectionM),
                    Math.Min(LengthM, CumulativeM + CorrectionM)));
            }
            ManualIntervals = MergeIntervals(ManualIntervals);
            if (ManualIntervals.Count == 0)
            {
                return new Result(true, false, Points, LengthM,
                    new List<Leg> { new Leg(false, Points) },
                    0.0, "AUTO_POSITIONAL", String.Empty);
            }
            if (LengthM <= MaxManualAssistM + 1e-9)
            {
                return new Result(true, true, Points, LengthM,
                    new List<Leg> { new Leg(true, Points) },
                    LengthM, Mode, String.Empty);
            }
            if (LengthM < MinimumHybridLengthM - 1e-9)
            {
                return new Result(false, true, new List<(Double X, Double Y)>(), 0.0,
                    new List<Leg>(), 0.0,
                    "FAILED",
                    String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "Manual assist length {0:F2} m exceeds the {1:F2} m limit.",
                        LengthM,
                        MaxManualAssistM));
            }
            if (ManualIntervals.Any(Interval =>
                (Interval.ToM - Interval.FromM) > MaxManualAssistM + 1e-9))
            {
                return new Result(false, true, new List<(Double X, Double Y)>(), 0.0,
                    new List<Leg>(), 0.0,
                    "FAILED", "Required contiguous manual correction exceeds the configured limit.");
            }

            List<Leg> Legs = new List<Leg>();
            Double PositionM = 0.0;
            Double ManualM = 0.0;
            foreach ((Double FromM, Double ToM) Interval in ManualIntervals)
            {
                if (Interval.FromM > PositionM + 1e-9)
                {
                    Legs.Add(new Leg(false, Slice(Points, PositionM, Interval.FromM)));
                }
                Legs.Add(new Leg(true, Slice(Points, Interval.FromM, Interval.ToM)));
                ManualM += Interval.ToM - Interval.FromM;
                PositionM = Interval.ToM;
            }
            if (PositionM < LengthM - 1e-9)
            {
                Legs.Add(new Leg(false, Slice(Points, PositionM, LengthM)));
            }
            Boolean EntirelyManual = (Legs.Count == 1) && Legs[0].OperatorAlignmentRequired;
            return new Result(true, true, Points, LengthM, Legs,
                ManualM, EntirelyManual ? Mode : Mode + "_HYBRID", String.Empty);
        }

        /// <summary>Returns the compass bearing from one point to the next.</summary>
        private static Double BearingDeg
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End
            )
        {
            return Math.Atan2(End.X - Start.X, End.Y - Start.Y) * 180.0 / Math.PI;
        }

        /// <summary>Returns the absolute heading deflection at an interior polyline point.</summary>
        private static Double CornerDeflectionDeg
            (
            (Double X, Double Y) Before,
            (Double X, Double Y) Point,
            (Double X, Double Y) After
            )
        {
            Double InX = Point.X - Before.X;
            Double InY = Point.Y - Before.Y;
            Double OutX = After.X - Point.X;
            Double OutY = After.Y - Point.Y;
            Double InLength = Math.Sqrt((InX * InX) + (InY * InY));
            Double OutLength = Math.Sqrt((OutX * OutX) + (OutY * OutY));
            if ((InLength <= 1e-12) || (OutLength <= 1e-12)) return 0.0;
            Double Cosine = ((InX * OutX) + (InY * OutY)) / (InLength * OutLength);
            Cosine = Math.Max(-1.0, Math.Min(1.0, Cosine));
            return Math.Acos(Cosine) * 180.0 / Math.PI;
        }

        /// <summary>Returns the circumradius through three points, or infinity when collinear.</summary>
        private static Double ThreePointRadius
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            (Double X, Double Y) C
            )
        {
            Double Ab = Distance(A, B);
            Double Bc = Distance(B, C);
            Double Ac = Distance(A, C);
            Double TwiceArea = Math.Abs(
                ((B.X - A.X) * (C.Y - A.Y))
                - ((B.Y - A.Y) * (C.X - A.X)));
            if (TwiceArea <= 1e-12) return Double.PositiveInfinity;
            return (Ab * Bc * Ac) / (2.0 * TwiceArea);
        }

        /// <summary>
        /// Replaces modest grid-route corners with sampled minimum-radius circular fillets when they fit.
        /// </summary>
        private static List<(Double X, Double Y)> SmoothRadiusValidCorners
            (
            List<(Double X, Double Y)> Points,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if (Points.Count < 3) return Points;
            List<(Double X, Double Y)> Output = new List<(Double X, Double Y)>
            {
                Points[0],
            };
            for (Int32 Index = 1; Index < Points.Count - 1; Index += 1)
            {
                List<(Double X, Double Y)> Fillet = BuildFillet(
                    Points[Index - 1],
                    Points[Index],
                    Points[Index + 1],
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field);
                if (Fillet.Count == 0)
                {
                    Output.Add(Points[Index]);
                    continue;
                }
                if (Distance(Output[Output.Count - 1], Fillet[0]) > 1e-9)
                {
                    Output.Add(Fillet[0]);
                }
                for (Int32 FilletIndex = 1; FilletIndex < Fillet.Count; FilletIndex += 1)
                {
                    Output.Add(Fillet[FilletIndex]);
                }
            }
            if (Distance(Output[Output.Count - 1], Points[Points.Count - 1]) > 1e-9)
            {
                Output.Add(Points[Points.Count - 1]);
            }
            return Output;
        }

        /// <summary>Builds one tangent circular fillet for a modest heading change.</summary>
        private static List<(Double X, Double Y)> BuildFillet
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            (Double X, Double Y) C,
            Double RadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            Double InX = B.X - A.X;
            Double InY = B.Y - A.Y;
            Double OutX = C.X - B.X;
            Double OutY = C.Y - B.Y;
            Double InLength = Math.Sqrt((InX * InX) + (InY * InY));
            Double OutLength = Math.Sqrt((OutX * OutX) + (OutY * OutY));
            if ((InLength <= 1e-9) || (OutLength <= 1e-9))
            {
                return new List<(Double X, Double Y)>();
            }
            InX /= InLength;
            InY /= InLength;
            OutX /= OutLength;
            OutY /= OutLength;
            Double Dot = Math.Max(-1.0, Math.Min(1.0, (InX * OutX) + (InY * OutY)));
            Double DeflectionRad = Math.Acos(Dot);
            Double DeflectionDeg = DeflectionRad * 180.0 / Math.PI;
            if ((DeflectionDeg <= AUTOMATIC_ALIGNMENT_TOLERANCE_DEG)
                || (DeflectionDeg > 45.0))
            {
                return new List<(Double X, Double Y)>();
            }
            Double Cross = (InX * OutY) - (InY * OutX);
            if (Math.Abs(Cross) <= 1e-9) return new List<(Double X, Double Y)>();
            Double TangentM = RadiusM * Math.Tan(0.5 * DeflectionRad);
            if ((TangentM >= 0.45 * InLength) || (TangentM >= 0.45 * OutLength))
            {
                return new List<(Double X, Double Y)>();
            }

            (Double X, Double Y) TangentIn = (
                B.X - (InX * TangentM),
                B.Y - (InY * TangentM));
            (Double X, Double Y) TangentOut = (
                B.X + (OutX * TangentM),
                B.Y + (OutY * TangentM));
            Double Direction = Cross > 0.0 ? 1.0 : -1.0;
            Double CenterX = TangentIn.X + (Direction * RadiusM * -InY);
            Double CenterY = TangentIn.Y + (Direction * RadiusM * InX);
            if (Math.Abs(Distance((CenterX, CenterY), TangentOut) - RadiusM) > 1e-5)
            {
                return new List<(Double X, Double Y)>();
            }
            Double StartAngle = Math.Atan2(TangentIn.Y - CenterY, TangentIn.X - CenterX);
            Double Delta = Direction * DeflectionRad;
            Int32 Samples = Math.Max(
                2,
                (Int32)Math.Ceiling(Math.Abs(Delta) * RadiusM / SampleSpacingM));
            List<(Double X, Double Y)> Output =
                new List<(Double X, Double Y)>(Samples + 1);
            for (Int32 SampleIndex = 0; SampleIndex <= Samples; SampleIndex += 1)
            {
                Double Angle = StartAngle + (Delta * SampleIndex / Samples);
                (Double X, Double Y) Point = (
                    CenterX + (RadiusM * Math.Cos(Angle)),
                    CenterY + (RadiusM * Math.Sin(Angle)));
                if (SampleIndex == Samples) Point = TangentOut;
                if (!Field.ContainsWorldXy(Point.X, Point.Y))
                {
                    return new List<(Double X, Double Y)>();
                }
                Output.Add(Point);
            }
            return Output;
        }

        /// <summary>Merges overlapping correction intervals in cumulative-distance order.</summary>
        private static List<(Double FromM, Double ToM)> MergeIntervals
            (
            List<(Double FromM, Double ToM)> Intervals
            )
        {
            List<(Double FromM, Double ToM)> Ordered = Intervals
                .OrderBy(Interval => Interval.FromM)
                .ThenBy(Interval => Interval.ToM)
                .ToList();
            List<(Double FromM, Double ToM)> Output =
                new List<(Double FromM, Double ToM)>();
            foreach ((Double FromM, Double ToM) Interval in Ordered)
            {
                if ((Output.Count == 0)
                    || (Interval.FromM > Output[Output.Count - 1].ToM + 1e-9))
                {
                    Output.Add(Interval);
                    continue;
                }
                (Double FromM, Double ToM) Previous = Output[Output.Count - 1];
                Output[Output.Count - 1] = (
                    Previous.FromM,
                    Math.Max(Previous.ToM, Interval.ToM));
            }
            return Output;
        }

        /// <summary>Extracts an exact cumulative-distance interval from a polyline.</summary>
        private static List<(Double X, Double Y)> Slice
            (
            List<(Double X, Double Y)> Points,
            Double FromM,
            Double ToM
            )
        {
            List<(Double X, Double Y)> Output = new List<(Double X, Double Y)>
            {
                PointAt(Points, FromM),
            };
            Double CumulativeM = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                CumulativeM += Distance(Points[Index - 1], Points[Index]);
                if ((CumulativeM > FromM + 1e-9) && (CumulativeM < ToM - 1e-9))
                {
                    Output.Add(Points[Index]);
                }
            }
            Output.Add(PointAt(Points, ToM));
            return Output;
        }

        /// <summary>Interpolates an exact point at a cumulative polyline distance.</summary>
        private static (Double X, Double Y) PointAt
            (
            List<(Double X, Double Y)> Points,
            Double TargetM
            )
        {
            if (TargetM <= 0.0) return Points[0];
            Double CumulativeM = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double SegmentM = Distance(Points[Index - 1], Points[Index]);
                if (CumulativeM + SegmentM >= TargetM - 1e-9)
                {
                    Double Fraction = SegmentM <= 1e-12
                        ? 0.0
                        : (TargetM - CumulativeM) / SegmentM;
                    return (
                        Points[Index - 1].X + (Fraction * (Points[Index].X - Points[Index - 1].X)),
                        Points[Index - 1].Y + (Fraction * (Points[Index].Y - Points[Index - 1].Y)));
                }
                CumulativeM += SegmentM;
            }
            return Points[Points.Count - 1];
        }

        /// <summary>Finds a shortest valid-cell route and simplifies it by line of sight.</summary>
        private static List<(Double X, Double Y)> GridRoute
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double SampleSpacingM,
            Grid Field
            )
        {
            (Int32 Bx, Int32 By) StartCell = WorldCell(Start, Field);
            (Int32 Bx, Int32 By) EndCell = WorldCell(End, Field);
            Queue<(Int32 Bx, Int32 By)> Open = new Queue<(Int32 Bx, Int32 By)>();
            Dictionary<(Int32 Bx, Int32 By), (Int32 Bx, Int32 By)> Previous =
                new Dictionary<(Int32 Bx, Int32 By), (Int32 Bx, Int32 By)>();
            Open.Enqueue(StartCell);
            Previous[StartCell] = StartCell;
            Int32[] Offsets = { -1, 0, 1 };
            while (Open.Count > 0)
            {
                (Int32 Bx, Int32 By) Current = Open.Dequeue();
                if (Current == EndCell) break;
                foreach (Int32 Dx in Offsets)
                {
                    foreach (Int32 Dy in Offsets)
                    {
                        if ((Dx == 0) && (Dy == 0)) continue;
                        (Int32 Bx, Int32 By) Next = (Current.Bx + Dx, Current.By + Dy);
                        if (!IsValid(Next, Field) || Previous.ContainsKey(Next)) continue;
                        Previous[Next] = Current;
                        Open.Enqueue(Next);
                    }
                }
            }
            if (!Previous.ContainsKey(EndCell)) return new List<(Double X, Double Y)>();

            List<(Double X, Double Y)> Reverse = new List<(Double X, Double Y)> { End };
            (Int32 Bx, Int32 By) Cell = EndCell;
            while (Cell != StartCell)
            {
                Reverse.Add(Field.CellCenterXy(Cell.Bx, Cell.By));
                Cell = Previous[Cell];
            }
            Reverse.Add(Start);
            Reverse.Reverse();
            return Simplify(Reverse, SampleSpacingM, Field);
        }

        /// <summary>Removes intermediate points while retaining sampled in-field chords.</summary>
        private static List<(Double X, Double Y)> Simplify
            (
            List<(Double X, Double Y)> Input,
            Double SampleSpacingM,
            Grid Field
            )
        {
            List<(Double X, Double Y)> Output = new List<(Double X, Double Y)> { Input[0] };
            Int32 Anchor = 0;
            while (Anchor < Input.Count - 1)
            {
                Int32 Furthest = Anchor + 1;
                for (Int32 Candidate = Anchor + 2; Candidate < Input.Count; Candidate += 1)
                {
                    if (!AllInside(SampleLine(Input[Anchor], Input[Candidate], SampleSpacingM), Field))
                    {
                        break;
                    }
                    Furthest = Candidate;
                }
                Output.Add(Input[Furthest]);
                Anchor = Furthest;
            }
            return Output;
        }

        /// <summary>Samples a straight segment at the requested maximum spacing.</summary>
        private static List<(Double X, Double Y)> SampleLine
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double SampleSpacingM
            )
        {
            Double LengthM = Distance(Start, End);
            Int32 Count = Math.Max(1, (Int32)Math.Ceiling(LengthM / SampleSpacingM));
            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            for (Int32 Index = 0; Index <= Count; Index += 1)
            {
                Double Fraction = (Double)Index / Count;
                Points.Add((
                    Start.X + (Fraction * (End.X - Start.X)),
                    Start.Y + (Fraction * (End.Y - Start.Y))));
            }
            return Points;
        }

        /// <summary>Converts a world coordinate to its containing grid cell.</summary>
        private static (Int32 Bx, Int32 By) WorldCell
            (
            (Double X, Double Y) Point,
            Grid Field
            )
        {
            return (
                (Int32)Math.Floor((Point.X - Field.MinX) / Field.GridSizeM),
                (Int32)Math.Floor((Point.Y - Field.MinY) / Field.GridSizeM));
        }

        /// <summary>Checks a cell index against the raw field mask.</summary>
        private static Boolean IsValid
            (
            (Int32 Bx, Int32 By) Cell,
            Grid Field
            )
        {
            return (Cell.Bx >= 0) && (Cell.By >= 0)
                && (Cell.Bx < Field.Width) && (Cell.By < Field.Height)
                && Field.Valid[Cell.By, Cell.Bx];
        }

        /// <summary>Checks every point against the raw field mask.</summary>
        private static Boolean AllInside
            (
            List<(Double X, Double Y)> Points,
            Grid Field
            )
        {
            foreach ((Double X, Double Y) Point in Points)
            {
                if (!Field.ContainsWorldXy(Point.X, Point.Y)) return false;
            }
            return true;
        }

        /// <summary>
        /// Densely validates every chord, preventing two valid stored samples from crossing an invalid cell between them.
        /// </summary>
        private static Boolean DensePolylineAllowed
            (
            List<(Double X, Double Y)> Points,
            Grid Field,
            ExperimentalFieldEdgePolicy? ExteriorPolicy
            )
        {
            if (Points.Count == 0) return false;
            Double SpacingM = Math.Min(0.10, Field.GridSizeM / 4.0);
            for (Int32 Index = 0; Index < Points.Count; Index += 1)
            {
                (Double X, Double Y) Start = Points[Index];
                (Double X, Double Y) End = Index + 1 < Points.Count
                    ? Points[Index + 1]
                    : Start;
                Double LengthM = Distance(Start, End);
                Int32 Count = Math.Max(1, (Int32)Math.Ceiling(LengthM / SpacingM));
                for (Int32 Sample = 0; Sample <= Count; Sample += 1)
                {
                    Double Fraction = (Double)Sample / Count;
                    Double X = Start.X + (Fraction * (End.X - Start.X));
                    Double Y = Start.Y + (Fraction * (End.Y - Start.Y));
                    Boolean Allowed = ExteriorPolicy == null
                        ? Field.ContainsWorldXy(X, Y)
                        : ExteriorPolicy.ContainsWithExteriorAllowance(X, Y);
                    if (!Allowed) return false;
                }
            }
            return true;
        }

        /// <summary>Computes sampled polyline length.</summary>
        private static Double PolylineLength
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

        /// <summary>Computes planar distance.</summary>
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
