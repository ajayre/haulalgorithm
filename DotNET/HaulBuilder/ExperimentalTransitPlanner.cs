using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder
{
    /// <summary>
    /// Builds loop-free experimental transit with explicit operator-alignment fallback.
    /// </summary>
    public static class ExperimentalTransitPlanner
    {
        private const Double MAX_AUTOMATIC_DETOUR_RATIO = 1.75;
        private const Double MAX_AUTOMATIC_TURN_OVERHEAD_RAD = 0.65 * Math.PI;

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
                String FailureReason
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
        }

        /// <summary>
        /// Uses a compact automatic curve when possible, otherwise a safe positional transit.
        /// </summary>
        static public Result Connect
            (
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AllowManualAssist = true,
            Double MaxManualAssistM = 45.0 * Constants.FEET_TO_METERS
            )
        {
            Double DirectM = Distance((Start.X, Start.Y), (End.X, End.Y));
            ExperimentalConnectorPlanner.Result Automatic = ExperimentalConnectorPlanner.Connect(
                Start, End, MinimumRadiusM, SampleSpacingM, Field);
            Double MaximumAutomaticM = Math.Min(
                DirectM * MAX_AUTOMATIC_DETOUR_RATIO,
                DirectM + (MAX_AUTOMATIC_TURN_OVERHEAD_RAD * MinimumRadiusM));
            if (Automatic.Success && (Automatic.LengthM <= MaximumAutomaticM))
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

            List<(Double X, Double Y)> Direct = SampleLine(
                (Start.X, Start.Y), (End.X, End.Y), SampleSpacingM);
            if (AllInside(Direct, Field))
            {
                return AlignmentResult(
                    Direct,
                    MinimumRadiusM,
                    MaxManualAssistM,
                    3.0 * MinimumRadiusM,
                    "OPERATOR_ALIGN");
            }

            List<(Double X, Double Y)> Routed = GridRoute(
                (Start.X, Start.Y), (End.X, End.Y), SampleSpacingM, Field);
            if (Routed.Count > 0)
            {
                return AlignmentResult(Routed, MinimumRadiusM, MaxManualAssistM, 0.0,
                    "OPERATOR_ALIGN_ROUTED");
            }

            return new Result(false, true, new List<(Double X, Double Y)>(), 0.0,
                new List<Leg>(), 0.0,
                "FAILED", "No nominal in-field positional transit exists.");
        }

        /// <summary>
        /// Splits a positional fallback into straight automatic legs and bounded manual corrections.
        /// </summary>
        private static Result AlignmentResult
            (
            List<(Double X, Double Y)> Points,
            Double MinimumRadiusM,
            Double MaxManualAssistM,
            Double MinimumHybridLengthM,
            String Mode
            )
        {
            Double LengthM = PolylineLength(Points);
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

            Double CorrectionM = Math.Min(MinimumRadiusM * 0.5, MaxManualAssistM * 0.5);
            List<(Double FromM, Double ToM)> ManualIntervals =
                new List<(Double FromM, Double ToM)>
                {
                    (0.0, Math.Min(CorrectionM, LengthM)),
                    (Math.Max(0.0, LengthM - CorrectionM), LengthM),
                };
            Double CumulativeM = 0.0;
            for (Int32 Index = 1; Index < Points.Count - 1; Index += 1)
            {
                CumulativeM += Distance(Points[Index - 1], Points[Index]);
                if (!IsCorner(Points[Index - 1], Points[Index], Points[Index + 1])) continue;
                ManualIntervals.Add((
                    Math.Max(0.0, CumulativeM - CorrectionM),
                    Math.Min(LengthM, CumulativeM + CorrectionM)));
            }
            ManualIntervals = MergeIntervals(ManualIntervals);
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
            return new Result(true, true, Points, LengthM, Legs,
                ManualM, Mode + "_HYBRID", String.Empty);
        }

        /// <summary>Checks whether an interior polyline point changes heading materially.</summary>
        private static Boolean IsCorner
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
            if ((InLength <= 1e-12) || (OutLength <= 1e-12)) return false;
            Double Cosine = ((InX * OutX) + (InY * OutY)) / (InLength * OutLength);
            return Cosine < Math.Cos(Math.PI / 180.0);
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
