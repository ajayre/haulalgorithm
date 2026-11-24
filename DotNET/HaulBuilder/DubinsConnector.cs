using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Forward-only connectors of turning radius R. The six Dubins words are the
    /// closed-form paths from Shkel and Lumelsky. The machine never backs up.
    /// </summary>
    static public class DubinsConnector
    {
        /// <summary>Full turn, radians.</summary>
        private const Double TWO_PI = 2.0 * Math.PI;

        /// <summary>Spacing of the field check and of guidance samples, metres.</summary>
        public const Double SAMPLE_M = 0.5;

        /// <summary>Lead-in and lead-out are tried on this step, metres.</summary>
        public const Double LEAD_STEP_M = 0.5;

        /// <summary>Longest lead is two radii. A longer straight is a different path.</summary>
        private const Double MAX_LEAD_RADII = 2.0;

        /// <summary>An arc this close to 0 or 2π is a numerical full turn, not a real one.</summary>
        private const Double ARC_SNAP_RAD = 1e-6;

        /// <summary>A word that misses the end pose by more than this is not that word.</summary>
        private const Double END_MISS_M = 0.01;

        /// <summary>A word that misses the end heading by more than this is not that word.</summary>
        private const Double END_MISS_RAD = 0.01;

        /// <summary>
        /// Poses no farther apart than this are the same place.
        /// A join inside this distance must not grow into a full circle.
        /// </summary>
        private const Double SAME_POINT_M = 0.05;

        /// <summary>Headings closer than one degree are the same direction.</summary>
        private const Double SAME_HEADING_DEG = 1.0;

        /// <summary>Words in the order a tie keeps. Earlier wins when lengths match.</summary>
        private static readonly String[] WordOrder = new String[]
        {
            "LSL",
            "RSR",
            "LSR",
            "RSL",
            "RLR",
            "LRL",
        };

        /// <summary>
        /// A pose is a point and the direction of travel. Heading is radians
        /// anticlockwise from east.
        /// </summary>
        public readonly record struct Pose
        {
            /// <summary>
            /// Initializes a pose.
            /// </summary>
            /// <param name="X">Easting, metres.</param>
            /// <param name="Y">Northing, metres.</param>
            /// <param name="HeadingRad">Travel heading, radians anticlockwise from east.</param>
            public Pose
                (
                Double X,
                Double Y,
                Double HeadingRad
                )
            {
                this.X = X;
                this.Y = Y;
                this.HeadingRad = HeadingRad;
            }

            /// <summary>Gets easting in metres.</summary>
            public Double X { get; }

            /// <summary>Gets northing in metres.</summary>
            public Double Y { get; }

            /// <summary>Gets travel heading, radians anticlockwise from east.</summary>
            public Double HeadingRad { get; }
        }

        /// <summary>
        /// One piece of a Dubins word. An arc has curvature 1/R. A straight has none.
        /// </summary>
        public sealed class Piece
        {
            /// <summary>
            /// Initializes one piece.
            /// </summary>
            /// <param name="Kind">Left arc, right arc, or straight.</param>
            /// <param name="LengthM">Centreline length in metres.</param>
            /// <param name="CurvaturePerM">Absolute curvature. Zero on a straight. 1/R on an arc.</param>
            public Piece
                (
                PieceKind Kind,
                Double LengthM,
                Double CurvaturePerM
                )
            {
                this.Kind = Kind;
                this.LengthM = LengthM;
                this.CurvaturePerM = CurvaturePerM;
            }

            /// <summary>Gets the piece kind.</summary>
            public PieceKind Kind { get; }

            /// <summary>Gets the centreline length in metres.</summary>
            public Double LengthM { get; }

            /// <summary>Gets absolute curvature in 1/metres.</summary>
            public Double CurvaturePerM { get; }
        }

        /// <summary>
        /// Left arc, right arc, or straight. Left increases heading.
        /// </summary>
        public enum PieceKind
        {
            /// <summary>No curvature.</summary>
            Straight = 0,

            /// <summary>Anticlockwise arc. Heading increases.</summary>
            Left = 1,

            /// <summary>Clockwise arc. Heading decreases.</summary>
            Right = 2,
        }

        /// <summary>
        /// One chosen connector, including any lead-in or lead-out.
        /// </summary>
        public sealed class Path
        {
            /// <summary>
            /// Initializes a connector.
            /// </summary>
            /// <param name="Word">Dubins word, or S when the poses already match.</param>
            /// <param name="Pieces">Pieces in drive order. Leads are straight pieces.</param>
            /// <param name="Start">Pose the path leaves.</param>
            /// <param name="End">Pose the path arrives at.</param>
            /// <param name="Outside">True when every in-field candidate was rejected.</param>
            /// <param name="LeadOutM">Straight added along the start heading.</param>
            /// <param name="LeadInM">Straight added into the end pose.</param>
            public Path
                (
                String Word,
                List<Piece> Pieces,
                Pose Start,
                Pose End,
                Boolean Outside,
                Double LeadOutM,
                Double LeadInM
                )
            {
                Double Length = 0.0;
                foreach (Piece Item in Pieces)
                {
                    Length += Item.LengthM;
                }

                this.Word = Word;
                this.Pieces = Pieces;
                this.Start = Start;
                this.End = End;
                this.Outside = Outside;
                this.LeadOutM = LeadOutM;
                this.LeadInM = LeadInM;
                this.LengthM = Length;
            }

            /// <summary>Gets the Dubins word.</summary>
            public String Word { get; }

            /// <summary>Gets the pieces in drive order.</summary>
            public List<Piece> Pieces { get; }

            /// <summary>Gets the pose the path leaves.</summary>
            public Pose Start { get; }

            /// <summary>Gets the pose the path arrives at.</summary>
            public Pose End { get; }

            /// <summary>Gets whether the centreline had to leave the field.</summary>
            public Boolean Outside { get; }

            /// <summary>Gets the lead-out length in metres.</summary>
            public Double LeadOutM { get; }

            /// <summary>Gets the lead-in length in metres.</summary>
            public Double LeadInM { get; }

            /// <summary>Gets the centreline length in metres, leads included.</summary>
            public Double LengthM { get; }

            /// <summary>
            /// Returns the same geometry marked as leaving the field.
            /// </summary>
            /// <returns>A copy with <see cref="Outside"/> set.</returns>
            public Path AsOutside
                (
                )
            {
                return new Path(this.Word, this.Pieces, this.Start, this.End, true, this.LeadOutM, this.LeadInM);
            }
        }

        /// <summary>
        /// Minimum turning radius from the machine's turning circle. The circle
        /// constant is a diameter.
        /// </summary>
        static public Double DefaultRadiusM
        {
            get
            {
                return Constants.DEFAULT_TURNING_CIRCLE_FT
                    * Constants.HALF_SCALE
                    * Constants.FEET_TO_METERS;
            }
        }

        /// <summary>
        /// Shortest forward connector from <paramref name="Start"/> to <paramref name="End"/>.
        /// A null <paramref name="Inside"/> does not test the field. Otherwise a
        /// centreline that leaves the field is dropped, then a lead is tried, and
        /// only then is the shortest path kept and marked outside.
        /// </summary>
        /// <param name="Start">Pose the machine is in.</param>
        /// <param name="End">Pose the machine must reach, still facing its travel direction.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <param name="Inside">True when a centreline point lies in the field mask. Null skips the mask.</param>
        /// <returns>The chosen connector. Never null.</returns>
        static public Path Connect
            (
            Pose Start,
            Pose End,
            Double RadiusM,
            Func<Double, Double, Boolean>? Inside
            )
        {
            if (RadiusM <= 0.0)
            {
                throw new ArgumentOutOfRangeException(nameof(RadiusM));
            }

            // Already there, or a heading that wrapped through a full turn.
            // That join is empty. A 2πR loop is the bug this guard exists to stop.
            if (IsEmptyJoin(Start, End))
            {
                return new Path("S", new List<Piece>(), Start, End, false, 0.0, 0.0);
            }

            Path? Chosen = Select(Start, End, RadiusM, Inside);
            if (Chosen != null)
            {
                return Chosen;
            }

            // The six words all leave the field. A short lead can move the
            // arc off the boundary. The lead itself has to stay in the mask.
            if (Inside != null)
            {
                Path? WithLead = SelectLead(Start, End, RadiusM, Inside);
                if (WithLead != null)
                {
                    return WithLead;
                }
            }

            // Still nothing. Keep the shortest word and say so. The run continues.
            Path? Open = Select(Start, End, RadiusM, null);
            if (Open != null)
            {
                return Open.AsOutside();
            }

            return StraightFallback(Start, End);
        }

        /// <summary>
        /// Length of the shortest word, with no field test and no piece list.
        /// Trip ordering uses this so it does not build a path for every candidate.
        /// </summary>
        /// <param name="Start">Start pose.</param>
        /// <param name="End">End pose.</param>
        /// <param name="RadiusM">Turning radius, metres.</param>
        /// <returns>Centreline length in metres. Zero when the join is empty.</returns>
        static public Double ShortestLengthM
            (
            Pose Start,
            Pose End,
            Double RadiusM
            )
        {
            if (RadiusM <= 0.0)
            {
                throw new ArgumentOutOfRangeException(nameof(RadiusM));
            }

            if (IsEmptyJoin(Start, End))
            {
                return 0.0;
            }

            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double Distance = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Double Normalized = Distance / RadiusM;
            Double Theta = (Normalized > 0.0) ? Mod2Pi(Math.Atan2(Dy, Dx)) : 0.0;
            Double Alpha = Mod2Pi(Start.HeadingRad - Theta);
            Double Beta = Mod2Pi(End.HeadingRad - Theta);
            Double Best = Double.PositiveInfinity;
            for (Int32 Index = 0; Index < WordOrder.Length; Index += 1)
            {
                if (!TryParameters(WordOrder[Index], Alpha, Beta, Normalized, out Double First, out Double Middle, out Double Last))
                {
                    continue;
                }

                Double Length = (First + Middle + Last) * RadiusM;
                if (Length < Best)
                {
                    Best = Length;
                }
            }

            if (Double.IsPositiveInfinity(Best))
            {
                return Distance;
            }

            return Best;
        }

        /// <summary>
        /// Advances one piece from a pose. Curvature is 1/R on an arc and 0 on a straight.
        /// </summary>
        /// <param name="Start">Pose at the start of the step.</param>
        /// <param name="Kind">Which piece is being driven.</param>
        /// <param name="DistanceM">Distance along the piece, metres.</param>
        /// <param name="RadiusM">Turning radius, metres.</param>
        /// <returns>The pose after the step.</returns>
        static public Pose Advance
            (
            Pose Start,
            PieceKind Kind,
            Double DistanceM,
            Double RadiusM
            )
        {
            if (DistanceM <= 0.0)
            {
                return Start;
            }

            Double Heading = Start.HeadingRad;
            if (Kind == PieceKind.Straight)
            {
                return new Pose(
                    Start.X + (Math.Cos(Heading) * DistanceM),
                    Start.Y + (Math.Sin(Heading) * DistanceM),
                    Heading);
            }

            // Left and right sit on opposite circles. A negative left sweep is
            // not a right turn: the centre is on the other side of the blade.
            Double Sweep = DistanceM / RadiusM;
            if (Kind == PieceKind.Left)
            {
                Double Next = Heading + Sweep;
                return new Pose(
                    Start.X + (RadiusM * (Math.Sin(Next) - Math.Sin(Heading))),
                    Start.Y + (RadiusM * (-Math.Cos(Next) + Math.Cos(Heading))),
                    Next);
            }

            Double Right = Heading - Sweep;
            return new Pose(
                Start.X + (RadiusM * (Math.Sin(Heading) - Math.Sin(Right))),
                Start.Y + (RadiusM * (Math.Cos(Right) - Math.Cos(Heading))),
                Right);
        }

        /// <summary>
        /// Pose at the end of a piece list.
        /// </summary>
        /// <param name="Start">Pose at the first piece.</param>
        /// <param name="Pieces">Pieces in drive order.</param>
        /// <param name="RadiusM">Turning radius, metres.</param>
        /// <returns>The arrival pose.</returns>
        static public Pose Integrate
            (
            Pose Start,
            IReadOnlyList<Piece> Pieces,
            Double RadiusM
            )
        {
            Pose Cursor = Start;
            foreach (Piece Item in Pieces)
            {
                Cursor = Advance(Cursor, Item.Kind, Item.LengthM, RadiusM);
            }

            return Cursor;
        }

        /// <summary>
        /// Samples a piece list. The last point is the requested end when the
        /// analytic arrival is already on it, so the guidance list meets the lane.
        /// </summary>
        /// <param name="Start">Path start.</param>
        /// <param name="Pieces">Pieces in drive order.</param>
        /// <param name="SpacingM">Maximum gap between samples, metres.</param>
        /// <param name="RadiusM">Turning radius, metres.</param>
        /// <param name="End">Pose the path is supposed to reach.</param>
        /// <returns>Samples including the start and the end.</returns>
        static public List<Pose> Sample
            (
            Pose Start,
            IReadOnlyList<Piece> Pieces,
            Double SpacingM,
            Double RadiusM,
            Pose End
            )
        {
            List<Pose> Points = new List<Pose> { Start };
            Pose Cursor = Start;
            foreach (Piece Item in Pieces)
            {
                Int32 Count = Math.Max(1, (Int32)Math.Ceiling((Item.LengthM / SpacingM) - 1e-9));
                for (Int32 Step = 1; Step <= Count; Step += 1)
                {
                    Double Distance = Item.LengthM * Step / Count;
                    Points.Add(Advance(Cursor, Item.Kind, Distance, RadiusM));
                }

                Cursor = Advance(Cursor, Item.Kind, Item.LengthM, RadiusM);
            }

            if (Points.Count > 0)
            {
                Pose Last = Points[Points.Count - 1];
                if (Near(Last, End))
                {
                    Points[Points.Count - 1] = End;
                }
            }

            return Points;
        }

        /// <summary>
        /// Wraps a heading into (−π, π]. A jump of 2π is the same direction.
        /// </summary>
        /// <param name="Angle">Heading in radians.</param>
        /// <returns>Wrapped heading.</returns>
        static public Double WrapPi
            (
            Double Angle
            )
        {
            Double Wrapped = Angle % TWO_PI;
            if (Wrapped <= -Math.PI)
            {
                Wrapped += TWO_PI;
            }
            else if (Wrapped > Math.PI)
            {
                Wrapped -= TWO_PI;
            }

            return Wrapped;
        }

        /// <summary>
        /// Picks the shortest word that stays inside, or the shortest word when
        /// <paramref name="Inside"/> is null.
        /// </summary>
        /// <param name="Start">Start pose.</param>
        /// <param name="End">End pose.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Inside">Field test. Null accepts every defined word.</param>
        /// <returns>The chosen path, or null when no word is defined or none stay in.</returns>
        private static Path? Select
            (
            Pose Start,
            Pose End,
            Double RadiusM,
            Func<Double, Double, Boolean>? Inside
            )
        {
            // Tie goes to the earlier word. A later word replaces it only when shorter.
            Path? Best = null;
            for (Int32 Index = 0; Index < WordOrder.Length; Index += 1)
            {
                Path? Candidate = TryWord(WordOrder[Index], Start, End, RadiusM);
                if (Candidate == null)
                {
                    continue;
                }

                if ((Inside != null) && !StaysInside(Candidate, RadiusM, Inside))
                {
                    continue;
                }

                if ((Best == null) || (Candidate.LengthM + 1e-9 < Best.LengthM))
                {
                    Best = Candidate;
                }
            }

            return Best;
        }

        /// <summary>
        /// Tries lead-out and lead-in until a Dubins word stays in the field.
        /// The shortest total, lead included, wins.
        /// </summary>
        /// <param name="Start">Original start pose.</param>
        /// <param name="End">Original end pose.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Inside">Field test.</param>
        /// <returns>The shortest leaded connector, or null.</returns>
        private static Path? SelectLead
            (
            Pose Start,
            Pose End,
            Double RadiusM,
            Func<Double, Double, Boolean> Inside
            )
        {
            Double MaxLead = MAX_LEAD_RADII * RadiusM;
            Int32 Steps = (Int32)Math.Floor((MaxLead + 1e-9) / LEAD_STEP_M);
            Path? Best = null;
            Double BestLength = Double.PositiveInfinity;
            for (Int32 OutStep = 0; OutStep <= Steps; OutStep += 1)
            {
                Double LeadOut = OutStep * LEAD_STEP_M;
                for (Int32 InStep = 0; InStep <= Steps; InStep += 1)
                {
                    Double LeadIn = InStep * LEAD_STEP_M;
                    ConsiderLead(Start, End, RadiusM, Inside, LeadOut, LeadIn, ref Best, ref BestLength);
                }
            }

            // The last partial step reaches the full two-radius lead.
            if ((Steps * LEAD_STEP_M) + 1e-6 < MaxLead)
            {
                for (Int32 InStep = 0; InStep <= Steps; InStep += 1)
                {
                    ConsiderLead(Start, End, RadiusM, Inside, MaxLead, InStep * LEAD_STEP_M, ref Best, ref BestLength);
                }

                for (Int32 OutStep = 0; OutStep <= Steps; OutStep += 1)
                {
                    ConsiderLead(Start, End, RadiusM, Inside, OutStep * LEAD_STEP_M, MaxLead, ref Best, ref BestLength);
                }

                ConsiderLead(Start, End, RadiusM, Inside, MaxLead, MaxLead, ref Best, ref BestLength);
            }

            return Best;
        }

        /// <summary>
        /// Keeps a lead pair when its Dubins middle stays in the field and the
        /// total is shorter than the best so far.
        /// </summary>
        /// <param name="Start">Original start.</param>
        /// <param name="End">Original end.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="LeadOut">Lead-out length, metres.</param>
        /// <param name="LeadIn">Lead-in length, metres.</param>
        /// <param name="Best">Best path so far.</param>
        /// <param name="BestLength">Length of <paramref name="Best"/>.</param>
        private static void ConsiderLead
            (
            Pose Start,
            Pose End,
            Double RadiusM,
            Func<Double, Double, Boolean> Inside,
            Double LeadOut,
            Double LeadIn,
            ref Path? Best,
            ref Double BestLength
            )
        {
            // Zero lead was already tried. A lead longer than the best cannot win.
            if ((LeadOut <= 0.0) && (LeadIn <= 0.0))
            {
                return;
            }

            if (LeadOut + LeadIn >= BestLength)
            {
                return;
            }

            Pose ShiftedStart = Advance(Start, PieceKind.Straight, LeadOut, RadiusM);
            Pose ShiftedEnd = new Pose(
                End.X - (Math.Cos(End.HeadingRad) * LeadIn),
                End.Y - (Math.Sin(End.HeadingRad) * LeadIn),
                End.HeadingRad);
            if (!SegmentInside(Start.X, Start.Y, ShiftedStart.X, ShiftedStart.Y, Inside))
            {
                return;
            }

            if (!SegmentInside(ShiftedEnd.X, ShiftedEnd.Y, End.X, End.Y, Inside))
            {
                return;
            }

            Path? Middle = Select(ShiftedStart, ShiftedEnd, RadiusM, Inside);
            if (Middle == null)
            {
                return;
            }

            Double Total = LeadOut + Middle.LengthM + LeadIn;
            if (Total >= BestLength)
            {
                return;
            }

            List<Piece> Pieces = new List<Piece>();
            if (LeadOut > 0.0)
            {
                Pieces.Add(new Piece(PieceKind.Straight, LeadOut, 0.0));
            }

            Pieces.AddRange(Middle.Pieces);
            if (LeadIn > 0.0)
            {
                Pieces.Add(new Piece(PieceKind.Straight, LeadIn, 0.0));
            }

            Best = new Path(Middle.Word, Pieces, Start, End, false, LeadOut, LeadIn);
            BestLength = Total;
        }

        /// <summary>
        /// Builds one word when the formula is defined and the pieces arrive.
        /// </summary>
        /// <param name="Word">Dubins word name.</param>
        /// <param name="Start">Start pose.</param>
        /// <param name="End">End pose.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>The word, or null when it is undefined or misses the end.</returns>
        private static Path? TryWord
            (
            String Word,
            Pose Start,
            Pose End,
            Double RadiusM
            )
        {
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double Distance = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Double Normalized = Distance / RadiusM;
            // Atan2(0, 0) is undefined. A turn in place has no chord direction.
            Double Theta = (Normalized > 0.0) ? Mod2Pi(Math.Atan2(Dy, Dx)) : 0.0;
            Double Alpha = Mod2Pi(Start.HeadingRad - Theta);
            Double Beta = Mod2Pi(End.HeadingRad - Theta);
            if (!TryParameters(Word, Alpha, Beta, Normalized, out Double First, out Double Middle, out Double Last))
            {
                return null;
            }

            List<Piece> Pieces = PiecesFor(Word, First, Middle, Last, RadiusM);
            Pose Arrival = Integrate(Start, Pieces, RadiusM);
            if (!Near(Arrival, End))
            {
                return null;
            }

            return new Path(Word, Pieces, Start, End, false, 0.0, 0.0);
        }

        /// <summary>
        /// Evaluates one Shkel and Lumelsky word. Arc parameters are normalised
        /// into [0, 2π) and a numerical full turn is dropped.
        /// </summary>
        /// <param name="Word">Word name.</param>
        /// <param name="Alpha">Start heading in the Dubins frame.</param>
        /// <param name="Beta">End heading in the Dubins frame.</param>
        /// <param name="Distance">Chord length divided by the radius.</param>
        /// <param name="First">First segment, normalised.</param>
        /// <param name="Middle">Middle segment, normalised.</param>
        /// <param name="Last">Last segment, normalised.</param>
        /// <returns>True when the word is defined.</returns>
        private static Boolean TryParameters
            (
            String Word,
            Double Alpha,
            Double Beta,
            Double Distance,
            out Double First,
            out Double Middle,
            out Double Last
            )
        {
            First = 0.0;
            Middle = 0.0;
            Last = 0.0;
            Double SinAlpha = Math.Sin(Alpha);
            Double SinBeta = Math.Sin(Beta);
            Double CosAlpha = Math.Cos(Alpha);
            Double CosBeta = Math.Cos(Beta);
            Double CosDifference = Math.Cos(Alpha - Beta);
            Double DistanceSquared = Distance * Distance;
            if (Word == "LSL")
            {
                Double StraightSquared = 2.0 + DistanceSquared - (2.0 * CosDifference)
                    + (2.0 * Distance * (SinAlpha - SinBeta));
                if (StraightSquared < 0.0)
                {
                    return false;
                }

                Double Angle = Math.Atan2(CosBeta - CosAlpha, Distance + SinAlpha - SinBeta);
                First = CanonicalArc(Angle - Alpha);
                Middle = Math.Sqrt(StraightSquared);
                Last = CanonicalArc(Beta - Angle);
                return Finite(First, Middle, Last);
            }

            if (Word == "RSR")
            {
                Double StraightSquared = 2.0 + DistanceSquared - (2.0 * CosDifference)
                    + (2.0 * Distance * (SinBeta - SinAlpha));
                if (StraightSquared < 0.0)
                {
                    return false;
                }

                Double Angle = Math.Atan2(CosAlpha - CosBeta, Distance - SinAlpha + SinBeta);
                First = CanonicalArc(Alpha - Angle);
                Middle = Math.Sqrt(StraightSquared);
                Last = CanonicalArc(Angle - Beta);
                return Finite(First, Middle, Last);
            }

            if (Word == "LSR")
            {
                Double StraightSquared = -2.0 + DistanceSquared + (2.0 * CosDifference)
                    + (2.0 * Distance * (SinAlpha + SinBeta));
                if (StraightSquared < 0.0)
                {
                    return false;
                }

                Double Straight = Math.Sqrt(StraightSquared);
                Double Angle = Math.Atan2(-CosAlpha - CosBeta, Distance + SinAlpha + SinBeta)
                    - Math.Atan2(-2.0, Straight);
                First = CanonicalArc(Angle - Alpha);
                Middle = Straight;
                Last = CanonicalArc(Angle - Mod2Pi(Beta));
                return Finite(First, Middle, Last);
            }

            if (Word == "RSL")
            {
                Double StraightSquared = -2.0 + DistanceSquared + (2.0 * CosDifference)
                    - (2.0 * Distance * (SinAlpha + SinBeta));
                if (StraightSquared < 0.0)
                {
                    return false;
                }

                Double Straight = Math.Sqrt(StraightSquared);
                Double Angle = Math.Atan2(CosAlpha + CosBeta, Distance - SinAlpha - SinBeta)
                    - Math.Atan2(2.0, Straight);
                First = CanonicalArc(Alpha - Angle);
                Middle = Straight;
                Last = CanonicalArc(Beta - Angle);
                return Finite(First, Middle, Last);
            }

            if (Word == "RLR")
            {
                Double Temporary = (6.0 - DistanceSquared + (2.0 * CosDifference)
                    + (2.0 * Distance * (SinAlpha - SinBeta))) / 8.0;
                if (Math.Abs(Temporary) > 1.0)
                {
                    return false;
                }

                Double Phi = Math.Atan2(CosAlpha - CosBeta, Distance - SinAlpha + SinBeta);
                Double Turn = Mod2Pi(TWO_PI - Math.Acos(Temporary));
                Double Arc = Mod2Pi(Alpha - Phi + Mod2Pi(Turn / 2.0));
                First = CanonicalArc(Arc);
                Middle = CanonicalArc(Turn);
                Last = CanonicalArc(Alpha - Beta - Arc + Mod2Pi(Turn));
                return Finite(First, Middle, Last);
            }

            if (Word == "LRL")
            {
                Double Temporary = (6.0 - DistanceSquared + (2.0 * CosDifference)
                    + (2.0 * Distance * (SinBeta - SinAlpha))) / 8.0;
                if (Math.Abs(Temporary) > 1.0)
                {
                    return false;
                }

                Double Phi = Math.Atan2(CosAlpha - CosBeta, Distance + SinAlpha - SinBeta);
                Double Turn = Mod2Pi(TWO_PI - Math.Acos(Temporary));
                Double Arc = Mod2Pi(-Alpha - Phi + (Turn / 2.0));
                First = CanonicalArc(Arc);
                Middle = CanonicalArc(Turn);
                Last = CanonicalArc(Mod2Pi(Beta) - Alpha - Arc + Mod2Pi(Turn));
                return Finite(First, Middle, Last);
            }

            return false;
        }

        /// <summary>
        /// Turns normalised parameters into pieces. A zero arc is omitted so a
        /// right-angle join is one arc and a straight, not a phantom loop.
        /// </summary>
        /// <param name="Word">Word name. The middle of a CSC word is the straight.</param>
        /// <param name="First">First normalised parameter.</param>
        /// <param name="Middle">Middle normalised parameter.</param>
        /// <param name="Last">Last normalised parameter.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>The non-zero pieces.</returns>
        private static List<Piece> PiecesFor
            (
            String Word,
            Double First,
            Double Middle,
            Double Last,
            Double RadiusM
            )
        {
            String Letters = LettersFor(Word);
            Double[] Parameters = new Double[] { First, Middle, Last };
            List<Piece> Pieces = new List<Piece>();
            for (Int32 Index = 0; Index < Letters.Length; Index += 1)
            {
                Char Letter = Letters[Index];
                Double Parameter = Parameters[Index];
                if (Letter == 'S')
                {
                    Double Length = Parameter * RadiusM;
                    if (Length > 1e-6)
                    {
                        Pieces.Add(new Piece(PieceKind.Straight, Length, 0.0));
                    }

                    continue;
                }

                // Arc parameter is already in radians. Length is that angle times R.
                if (Parameter <= ARC_SNAP_RAD)
                {
                    continue;
                }

                PieceKind Kind = (Letter == 'L') ? PieceKind.Left : PieceKind.Right;
                Pieces.Add(new Piece(Kind, Parameter * RadiusM, 1.0 / RadiusM));
            }

            return Pieces;
        }

        /// <summary>
        /// Segment letters for a word. CSC words have a straight in the middle.
        /// </summary>
        /// <param name="Word">Word name.</param>
        /// <returns>Three letters.</returns>
        private static String LettersFor
            (
            String Word
            )
        {
            if (Word == "LSL")
            {
                return "LSL";
            }

            if (Word == "RSR")
            {
                return "RSR";
            }

            if (Word == "LSR")
            {
                return "LSR";
            }

            if (Word == "RSL")
            {
                return "RSL";
            }

            if (Word == "RLR")
            {
                return "RLR";
            }

            return "LRL";
        }

        /// <summary>
        /// True when every sample on the centreline is in the field.
        /// </summary>
        /// <param name="Candidate">Path to test.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Inside">Field test.</param>
        /// <returns>True when the centreline stays in.</returns>
        private static Boolean StaysInside
            (
            Path Candidate,
            Double RadiusM,
            Func<Double, Double, Boolean> Inside
            )
        {
            if (!Inside(Candidate.Start.X, Candidate.Start.Y))
            {
                return false;
            }

            Pose Cursor = Candidate.Start;
            foreach (Piece Item in Candidate.Pieces)
            {
                Int32 Count = Math.Max(1, (Int32)Math.Ceiling((Item.LengthM / SAMPLE_M) - 1e-9));
                for (Int32 Step = 1; Step <= Count; Step += 1)
                {
                    Double Distance = Item.LengthM * Step / Count;
                    Pose At = Advance(Cursor, Item.Kind, Distance, RadiusM);
                    if (!Inside(At.X, At.Y))
                    {
                        return false;
                    }
                }

                Cursor = Advance(Cursor, Item.Kind, Item.LengthM, RadiusM);
            }

            return Inside(Candidate.End.X, Candidate.End.Y);
        }

        /// <summary>
        /// True when a straight lead stays in the field.
        /// </summary>
        /// <param name="X0">Start easting.</param>
        /// <param name="Y0">Start northing.</param>
        /// <param name="X1">End easting.</param>
        /// <param name="Y1">End northing.</param>
        /// <param name="Inside">Field test.</param>
        /// <returns>True when every sample is in.</returns>
        private static Boolean SegmentInside
            (
            Double X0,
            Double Y0,
            Double X1,
            Double Y1,
            Func<Double, Double, Boolean> Inside
            )
        {
            Double Dx = X1 - X0;
            Double Dy = Y1 - Y0;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Int32 Count = Math.Max(1, (Int32)Math.Ceiling((Length / SAMPLE_M) - 1e-9));
            for (Int32 Step = 0; Step <= Count; Step += 1)
            {
                Double Fraction = (Double)Step / Count;
                if (!Inside(X0 + (Dx * Fraction), Y0 + (Dy * Fraction)))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Last resort when no Dubins word is defined. The run still finishes.
        /// </summary>
        /// <param name="Start">Start pose.</param>
        /// <param name="End">End pose.</param>
        /// <returns>A straight chord marked outside.</returns>
        private static Path StraightFallback
            (
            Pose Start,
            Pose End
            )
        {
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            List<Piece> Pieces = new List<Piece>();
            if (Length > 1e-6)
            {
                Pieces.Add(new Piece(PieceKind.Straight, Length, 0.0));
            }

            return new Path("S", Pieces, Start, End, true, 0.0, 0.0);
        }

        /// <summary>
        /// True when two poses are the same place and the same direction.
        /// The heading difference is wrapped into (−π, π] first, so a heading
        /// of 2π is the same direction as 0 and does not earn a full circle.
        /// </summary>
        /// <param name="Start">First pose.</param>
        /// <param name="End">Second pose.</param>
        /// <returns>True when a connector would have no work.</returns>
        static public Boolean IsEmptyJoin
            (
            Pose Start,
            Pose End
            )
        {
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            if (((Dx * Dx) + (Dy * Dy)) > (SAME_POINT_M * SAME_POINT_M))
            {
                return false;
            }

            // Less than one degree. A real turnaround is half a turn, so it stays.
            Double Turn = Math.Abs(WrapPi(End.HeadingRad - Start.HeadingRad));
            Double Limit = SAME_HEADING_DEG * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            return Turn < Limit;
        }

        /// <summary>
        /// True when an integrated pose lands on the requested end.
        /// </summary>
        /// <param name="Arrival">Integrated pose.</param>
        /// <param name="End">Requested pose.</param>
        /// <returns>True when position and heading both match.</returns>
        private static Boolean Near
            (
            Pose Arrival,
            Pose End
            )
        {
            Double Dx = Arrival.X - End.X;
            Double Dy = Arrival.Y - End.Y;
            if (((Dx * Dx) + (Dy * Dy)) > (END_MISS_M * END_MISS_M))
            {
                return false;
            }

            return Math.Abs(WrapPi(Arrival.HeadingRad - End.HeadingRad)) <= END_MISS_RAD;
        }

        /// <summary>
        /// Wraps an arc into [0, 2π) and drops a numerical full turn.
        /// </summary>
        /// <param name="Angle">Arc parameter in radians.</param>
        /// <returns>Zero, or the arc in [0, 2π).</returns>
        private static Double CanonicalArc
            (
            Double Angle
            )
        {
            Double Wrapped = Mod2Pi(Angle);
            if ((Wrapped < ARC_SNAP_RAD) || (Wrapped > (TWO_PI - ARC_SNAP_RAD)))
            {
                return 0.0;
            }

            return Wrapped;
        }

        /// <summary>
        /// Wraps radians into [0, 2π).
        /// </summary>
        /// <param name="Angle">Any radian measure.</param>
        /// <returns>The equivalent angle in [0, 2π).</returns>
        private static Double Mod2Pi
            (
            Double Angle
            )
        {
            return Angle - (TWO_PI * Math.Floor(Angle / TWO_PI));
        }

        /// <summary>
        /// True when every parameter is a real number.
        /// </summary>
        /// <param name="First">First parameter.</param>
        /// <param name="Middle">Middle parameter.</param>
        /// <param name="Last">Last parameter.</param>
        /// <returns>True when none is NaN or infinite.</returns>
        private static Boolean Finite
            (
            Double First,
            Double Middle,
            Double Last
            )
        {
            return !(Double.IsNaN(First) || Double.IsNaN(Middle) || Double.IsNaN(Last)
                || Double.IsInfinity(First) || Double.IsInfinity(Middle) || Double.IsInfinity(Last));
        }
    }
}
