using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace HaulBuilder
{
    /// <summary>
    /// Guidance between the blade-down lanes the planner already chose.
    /// Each trip may reverse a spread lane, and a cut lane when the lane is flat.
    /// A lane end within a turning radius of the boundary may bend on a
    /// blade-down quarter circle so the turnaround stays in the field.
    /// A forward Dubins connector joins the chosen shapes. The loaded connector
    /// from the cut end to the spread start is part of that score. Lane
    /// direction is chosen by time alone. A road-stage trip is the exception:
    /// every lane on the bank faces the same way, and a lane change is a
    /// forward S-bend rather than a reversal. A short joint is a straight steer
    /// across, or a trimmed lane, instead of a loop.
    /// A grade pass then runs back over the fill. A correcting trip does not
    /// grade: the land plane follows. Its lines are one sweep, at most three
    /// cut lines and three spread lines, and the spreads snake. A correcting
    /// line starts at its first fragment and ends at its last, so the connector
    /// owns the turn. A connector is chosen from the six Dubins words and the
    /// short-joint trims. Within two percent of the shortest, or half a radius
    /// when that is more, the path with the least length outside the field wins.
    /// When a path stays in the field it wins, unless
    /// <see cref="GuidanceOptions.AllowLeaveField"/> is set. With that option,
    /// a clearly shorter outside path is kept. Two connectors with no lane between them become
    /// one Dubins path. A short correcting trip may merge into a later
    /// correcting trip whose route passes within two radii. An end fragment
    /// of a correcting line that is shorter than one radius, and more than
    /// four radii past the next fragment, is deferred. After correcting,
    /// sweep-up trips take that pool by nearest neighbour until the bowl is
    /// full, then use the same correcting shape. A cut or spread shorter
    /// than half a radius, or lighter than one percent of the bowl, does not
    /// separate connectors. Its cells are still cut and filled. After every
    /// trip is fixed, trips are reordered inside their own stage to shorten
    /// the empty returns. Two trips that share a cell keep their order.
    /// </summary>
    static public class TripGuidance
    {
        /// <summary>
        /// Default speed on every arc, miles per hour. Straights keep the haul clock.
        /// <see cref="GuidanceOptions.ArcSpeedMph"/> starts at this value.
        /// </summary>
        public const Double TURN_SPEED_MPH = 5.0;

        /// <summary>
        /// Lanes on one side that are tried both ways. Four and four is 256 courses.
        /// </summary>
        private const Int32 MAX_FLIP_LANES = 4;

        /// <summary>
        /// Along-lane grade below this has no downhill. The ribbon will not load
        /// back uphill; a flatter cut may face either way.
        /// </summary>
        private const Double FLAT_LOAD_GRADE = 0.001;

        /// <summary>A cut shorter than this has no along-track grade to trust.</summary>
        private const Double MIN_SLOPE_LENGTH_M = 1.0;

        /// <summary>Open ground measured ahead of a lane end, in turning radii.</summary>
        private const Double CLEARANCE_RADII = 4.0;

        /// <summary>Times closer than this are a tie. The earlier combination wins.</summary>
        private const Double TIME_TIE_S = 1e-6;

        /// <summary>
        /// Direction and bend combinations tried for one trip. Four lanes a side,
        /// each both ways, is 256. Bends use whatever of that budget is left.
        /// </summary>
        private const Int32 MAX_COMBINATIONS = 256;

        /// <summary>Extra metres past the arc and the blade that still count as near the boundary.</summary>
        private const Double BEND_MARGIN_M = 1.0;

        /// <summary>
        /// A lane must head at least this far into the boundary before a bend is tried.
        /// Half is about sixty degrees off straight into the mask edge.
        /// </summary>
        private const Double APPROACH_DOT = 0.5;

        /// <summary>A bend must leave at least this much straight lane behind the arc.</summary>
        private const Double MIN_STRAIGHT_AFTER_BEND_M = 0.5;

        /// <summary>
        /// Headings closer than this are the same direction along the bank.
        /// A larger change is a turn, not a lane change.
        /// </summary>
        private const Double LANE_CHANGE_HEADING_RAD = 20.0 * Math.PI / 180.0;

        /// <summary>Forward progress, in metres, before a sideways shift counts as a lane change.</summary>
        private const Double LANE_CHANGE_FORWARD_M = 0.5;

        /// <summary>One grade lift. A 5 mm residual is inside this, so it reaches target.</summary>
        private const Double GRADE_LIFT_M = Correcting.SPREAD_LIFT_M;

        /// <summary>
        /// Whether a cut lane may be driven both ways.
        /// A downhill lane keeps that direction. A flat lane may face either way.
        /// </summary>
        private enum CutDirection
        {
            /// <summary>The stored direction descends. Reversing it would load uphill.</summary>
            Keep = 0,

            /// <summary>The stored direction climbs. Drive the other way.</summary>
            Reverse = 1,

            /// <summary>The lane is flat. Either direction may load.</summary>
            Either = 2,
        }

        /// <summary>Closer than this to target is already at target, metres.</summary>
        private const Double AT_TARGET_M = 1e-4;

        /// <summary>Two samples this close are the same point and are not repeated.</summary>
        private const Double JOINT_M = 1e-4;

        /// <summary>
        /// Land-plane tolerance, metres. A leftover correcting trip whose cells
        /// are already inside this is dropped. Correcting does not grade.
        /// </summary>
        private const Double GRADE_DEPTH_M = Constants.DEFAULT_TOLERANCE_MM / Constants.MILLIMETERS_PER_METER;

        /// <summary>
        /// Rules for the build running on this thread. A nested course keeps the
        /// outer rules when it does not pass its own.
        /// </summary>
        private static readonly AsyncLocal<GuidanceOptions?> ActiveRules =
            new AsyncLocal<GuidanceOptions?>();

        /// <summary>
        /// Installs <paramref name="Requested"/> for this call.
        /// </summary>
        /// <param name="Requested">Rules for this call. Null keeps the outer rules.</param>
        /// <returns>The rules to restore when the call ends.</returns>
        private static GuidanceOptions? InstallRules
            (
            GuidanceOptions? Requested
            )
        {
            GuidanceOptions? Previous = ActiveRules.Value;
            // A course built inside a job keeps that job's rules.
            ActiveRules.Value = Requested ?? Previous ?? new GuidanceOptions();
            return Previous;
        }

        /// <summary>Gets the rules for this thread, or the production defaults.</summary>
        private static GuidanceOptions CurrentRules
        {
            get
            {
                GuidanceOptions? Current = ActiveRules.Value;
                if (Current == null)
                {
                    return new GuidanceOptions();
                }

                return Current;
            }
        }

        /// <summary>A connector that turns more than this is a reversal.</summary>
        private const Double REVERSAL_DEG = 150.0;

        /// <summary>Reordered and unreordered plans must agree to this, metres.</summary>
        private const Double SURFACE_MATCH_M = 0.001;

        /// <summary>
        /// Half a cell past the first and last fragment. The blade still covers
        /// the cell. Anything further is empty travel and belongs to the connector.
        /// </summary>
        private const Double FRAGMENT_PAD_CELLS = 0.5;

        /// <summary>
        /// Fragments whose centre lines sit inside this band are the same line.
        /// Half a blade would still be the neighbouring pass, so this stays tighter.
        /// </summary>
        private const Double LINE_LATERAL_M = 0.75;

        /// <summary>A fragment heading within this many degrees shares the line's direction.</summary>
        private const Double LINE_ANGLE_DEG = 10.0;

        /// <summary>A lane shorter than this has no chord. Its start heading is the axis.</summary>
        private const Double LINE_POINT_M = 0.05;

        /// <summary>How many of the worst trips each diagnostic lists.</summary>
        private const Int32 WORST_COUNT = 20;

        /// <summary>
        /// Passes of consecutive-pair swaps while shortening empty returns.
        /// One pass walks every adjacent pair once. Stop earlier when a pass changes nothing.
        /// </summary>
        private const Int32 MAX_ORDER_PASSES = 64;

        /// <summary>Empty-return lengths closer than this are a tie. The earlier trip stays first.</summary>
        private const Double LENGTH_TIE_M = 1e-6;

        /// <summary>
        /// What the blade is doing at a guidance point.
        /// </summary>
        public enum GuidanceAction
        {
            /// <summary>Blade down in a cut lane.</summary>
            Cut = 0,

            /// <summary>Loaded connector between blade-down lanes.</summary>
            Carry = 1,

            /// <summary>Blade down in a spread lane.</summary>
            Fill = 2,

            /// <summary>Turnaround at the end of the spread.</summary>
            Turn = 3,

            /// <summary>Blade at target, driving back over the fill.</summary>
            Grade = 4,

            /// <summary>Empty connector to the next trip's first cut.</summary>
            Return = 5,
        }

        /// <summary>
        /// One sample on a trip. Heading is radians anticlockwise from east.
        /// </summary>
        public sealed class GuidancePoint
        {
            /// <summary>
            /// Initializes one sample.
            /// </summary>
            /// <param name="Sequence">1-based order along the trip.</param>
            /// <param name="X">Easting, metres.</param>
            /// <param name="Y">Northing, metres.</param>
            /// <param name="HeadingRad">Travel heading, radians anticlockwise from east.</param>
            /// <param name="Action">What the machine is doing.</param>
            /// <param name="BladeDown">True when the blade is in the ground.</param>
            /// <param name="Outside">True when this sample is on a connector that left the field.</param>
            public GuidancePoint
                (
                Int32 Sequence,
                Double X,
                Double Y,
                Double HeadingRad,
                GuidanceAction Action,
                Boolean BladeDown,
                Boolean Outside
                )
            {
                this.Sequence = Sequence;
                this.X = X;
                this.Y = Y;
                this.HeadingRad = HeadingRad;
                this.Action = Action;
                this.BladeDown = BladeDown;
                this.Outside = Outside;
            }

            /// <summary>Gets the 1-based sequence.</summary>
            public Int32 Sequence { get; }

            /// <summary>Gets easting in metres.</summary>
            public Double X { get; }

            /// <summary>Gets northing in metres.</summary>
            public Double Y { get; }

            /// <summary>Gets travel heading, radians anticlockwise from east.</summary>
            public Double HeadingRad { get; }

            /// <summary>Gets the action.</summary>
            public GuidanceAction Action { get; }

            /// <summary>Gets whether the blade is down.</summary>
            public Boolean BladeDown { get; }

            /// <summary>Gets whether the connector left the field.</summary>
            public Boolean Outside { get; }
        }

        /// <summary>
        /// One blade-down lane, kept as the planner drew it.
        /// </summary>
        public sealed class WorkLane
        {
            /// <summary>
            /// Initializes a lane.
            /// </summary>
            /// <param name="Centerline">Centerline points in drive order.</param>
            /// <param name="Start">Pose at the first point.</param>
            /// <param name="End">Pose at the last point.</param>
            /// <param name="Cells">Material cells the blade still sweeps, in drive order.</param>
            /// <param name="LengthM">Timed length, straight plus the exact arc.</param>
            /// <param name="StraightLengthM">Straight centreline that is driven at cut or fill speed.</param>
            /// <param name="ArcLengthM">Quarter-circle length R times the angle. Zero when the lane is straight.</param>
            /// <param name="Deferred">
            /// Cells the straight lane reached that this shape does not sweep.
            /// They stay for a later trip or the correcting headland lap.
            /// </param>
            public WorkLane
                (
                List<(Double X, Double Y)> Centerline,
                DubinsConnector.Pose Start,
                DubinsConnector.Pose End,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
                Double LengthM,
                Double StraightLengthM,
                Double ArcLengthM,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Deferred
                )
            {
                this.Centerline = Centerline;
                this.Start = Start;
                this.End = End;
                this.Cells = Cells;
                this.LengthM = LengthM;
                this.StraightLengthM = StraightLengthM;
                this.ArcLengthM = ArcLengthM;
                this.Deferred = Deferred;
            }

            /// <summary>Gets the centerline in drive order.</summary>
            public List<(Double X, Double Y)> Centerline { get; }

            /// <summary>Gets the start pose.</summary>
            public DubinsConnector.Pose Start { get; }

            /// <summary>Gets the end pose.</summary>
            public DubinsConnector.Pose End { get; }

            /// <summary>Gets the material cells in drive order.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells { get; }

            /// <summary>Gets the timed length in metres, straight plus the exact arc.</summary>
            public Double LengthM { get; }

            /// <summary>Gets the straight length driven at cut or fill speed.</summary>
            public Double StraightLengthM { get; }

            /// <summary>Gets the quarter-circle length. Zero when the lane does not bend.</summary>
            public Double ArcLengthM { get; }

            /// <summary>
            /// Gets cells the straight lane reached that this shape leaves behind.
            /// </summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Deferred { get; }

            /// <summary>
            /// Builds a lane from a centerline. Headings come from the first and
            /// last segments. A single point keeps <paramref name="FallbackHeadingRad"/>.
            /// </summary>
            /// <param name="Centerline">Points in drive order.</param>
            /// <param name="Cells">Material cells in drive order.</param>
            /// <param name="FallbackHeadingRad">Heading when the line has no length.</param>
            /// <returns>The lane.</returns>
            static public WorkLane FromCenterline
                (
                IReadOnlyList<(Double X, Double Y)> Centerline,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
                Double FallbackHeadingRad
                )
            {
                List<(Double X, Double Y)> Copy = new List<(Double X, Double Y)>(Centerline);
                if (Copy.Count == 0)
                {
                    Copy.Add((0.0, 0.0));
                }

                Double StartHeading = FallbackHeadingRad;
                Double EndHeading = FallbackHeadingRad;
                Double Length = 0.0;
                Boolean HaveStart = false;
                for (Int32 Index = 1; Index < Copy.Count; Index += 1)
                {
                    Double Dx = Copy[Index].X - Copy[Index - 1].X;
                    Double Dy = Copy[Index].Y - Copy[Index - 1].Y;
                    Double Step = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                    if (Step <= 1e-6)
                    {
                        continue;
                    }

                    Double Heading = Math.Atan2(Dy, Dx);
                    if (!HaveStart)
                    {
                        StartHeading = Heading;
                        HaveStart = true;
                    }

                    EndHeading = Heading;
                    Length += Step;
                }

                DubinsConnector.Pose Start = new DubinsConnector.Pose(Copy[0].X, Copy[0].Y, StartHeading);
                (Double X, Double Y) Last = Copy[Copy.Count - 1];
                DubinsConnector.Pose End = new DubinsConnector.Pose(Last.X, Last.Y, EndHeading);
                return new WorkLane(
                    Copy,
                    Start,
                    End,
                    Cells,
                    Length,
                    Length,
                    0.0,
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>());
            }

            /// <summary>
            /// The same lane driven the other way. The grade pass uses this.
            /// </summary>
            /// <returns>The reversed lane.</returns>
            public WorkLane Reversed
                (
                )
            {
                List<(Double X, Double Y)> Line = new List<(Double X, Double Y)>(this.Centerline);
                Line.Reverse();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Flipped =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(this.Cells);
                Flipped.Reverse();
                // A point lane has no segment to read. Face the other way.
                WorkLane Turned = FromCenterline(Line, Flipped, DubinsConnector.WrapPi(this.Start.HeadingRad + Math.PI));
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Left =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(this.Deferred);
                return new WorkLane(
                    Turned.Centerline,
                    Turned.Start,
                    Turned.End,
                    Turned.Cells,
                    this.StraightLengthM + this.ArcLengthM,
                    this.StraightLengthM,
                    this.ArcLengthM,
                    Left);
            }

            /// <summary>
            /// Drops the start of the lane and keeps the rest. Cells that fall
            /// in the dropped part stay on <see cref="Deferred"/> for a later pass.
            /// </summary>
            /// <param name="DropM">Metres to slide the start forward along the lane.</param>
            /// <returns>The shortened lane, or null when the remainder would be too short.</returns>
            public WorkLane? TrimFromStart
                (
                Double DropM
                )
            {
                return TrimSpan(DropM, true);
            }

            /// <summary>
            /// Drops the end of the lane and keeps the rest. Cells that fall
            /// in the dropped part stay on <see cref="Deferred"/> for a later pass.
            /// </summary>
            /// <param name="DropM">Metres to trim off the end.</param>
            /// <returns>The shortened lane, or null when the remainder would be too short.</returns>
            public WorkLane? TrimFromEnd
                (
                Double DropM
                )
            {
                return TrimSpan(DropM, false);
            }

            /// <summary>
            /// Cuts one end off the centreline. The kept cells are the same
            /// fraction of the cell list as the kept length, in drive order.
            /// </summary>
            /// <param name="DropM">Metres removed.</param>
            /// <param name="FromStart">True drops the start. False drops the end.</param>
            /// <returns>The shortened lane, or null when it cannot be shortened.</returns>
            private WorkLane? TrimSpan
                (
                Double DropM,
                Boolean FromStart
                )
            {
                if (DropM <= 0.05)
                {
                    return this;
                }

                Double Poly = PolylineLength(this.Centerline);
                if (Poly <= (DropM + MIN_STRAIGHT_AFTER_BEND_M))
                {
                    // The lane would vanish. Leave it for another option.
                    return null;
                }

                Double StartM = FromStart ? DropM : 0.0;
                Double EndM = FromStart ? Poly : (Poly - DropM);
                List<(Double X, Double Y)> Line = SlicePolyline(this.Centerline, StartM, EndM);
                if (Line.Count < 2)
                {
                    return null;
                }

                Double Keep = EndM - StartM;
                Int32 DropCount = 0;
                if ((this.Cells.Count > 0) && (Poly > 1e-6))
                {
                    DropCount = (Int32)Math.Round(this.Cells.Count * (DropM / Poly));
                    if (DropCount < 0)
                    {
                        DropCount = 0;
                    }

                    if (DropCount > this.Cells.Count)
                    {
                        DropCount = this.Cells.Count;
                    }
                }

                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Kept =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Left =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(this.Deferred);
                if (FromStart)
                {
                    for (Int32 Index = 0; Index < DropCount; Index += 1)
                    {
                        Left.Add(this.Cells[Index]);
                    }

                    for (Int32 Index = DropCount; Index < this.Cells.Count; Index += 1)
                    {
                        Kept.Add(this.Cells[Index]);
                    }
                }
                else
                {
                    Int32 KeepCount = this.Cells.Count - DropCount;
                    for (Int32 Index = 0; Index < KeepCount; Index += 1)
                    {
                        Kept.Add(this.Cells[Index]);
                    }

                    for (Int32 Index = KeepCount; Index < this.Cells.Count; Index += 1)
                    {
                        Left.Add(this.Cells[Index]);
                    }
                }

                Double Fallback = FromStart ? this.End.HeadingRad : this.Start.HeadingRad;
                WorkLane Fresh = FromCenterline(Line, Kept, Fallback);
                return new WorkLane(
                    Fresh.Centerline,
                    Fresh.Start,
                    Fresh.End,
                    Fresh.Cells,
                    Keep,
                    Keep,
                    0.0,
                    Left);
            }
        }

        /// <summary>
        /// One drawn piece. A connector arrow sits on each of these.
        /// </summary>
        public sealed class DrawnPiece
        {
            /// <summary>
            /// Initializes a drawn piece.
            /// </summary>
            /// <param name="Kind">Left, right, or straight.</param>
            /// <param name="LengthM">Length in metres.</param>
            /// <param name="CurvaturePerM">Absolute curvature.</param>
            /// <param name="Samples">Centerline samples including both ends.</param>
            /// <param name="Arrow">Midpoint pose for the arrowhead. Null draws none.</param>
            public DrawnPiece
                (
                DubinsConnector.PieceKind Kind,
                Double LengthM,
                Double CurvaturePerM,
                List<DubinsConnector.Pose> Samples,
                DubinsConnector.Pose? Arrow
                )
            {
                this.Kind = Kind;
                this.LengthM = LengthM;
                this.CurvaturePerM = CurvaturePerM;
                this.Samples = Samples;
                this.Arrow = Arrow;
            }

            /// <summary>Gets the piece kind.</summary>
            public DubinsConnector.PieceKind Kind { get; }

            /// <summary>Gets the length in metres.</summary>
            public Double LengthM { get; }

            /// <summary>Gets absolute curvature.</summary>
            public Double CurvaturePerM { get; }

            /// <summary>Gets the samples.</summary>
            public List<DubinsConnector.Pose> Samples { get; }

            /// <summary>Gets the arrow pose, or null.</summary>
            public DubinsConnector.Pose? Arrow { get; }
        }

        /// <summary>
        /// One lane or one connector on the trip.
        /// </summary>
        public sealed class CourseSegment
        {
            /// <summary>
            /// Initializes a segment.
            /// </summary>
            /// <param name="Action">What this stretch is.</param>
            /// <param name="IsLane">True for a blade-down lane. False for a connector.</param>
            /// <param name="Outside">True when the connector left the field.</param>
            /// <param name="Word">Dubins word. Empty on a lane.</param>
            /// <param name="LeadOutM">Lead-out length.</param>
            /// <param name="LeadInM">Lead-in length.</param>
            /// <param name="LengthM">Centerline length.</param>
            /// <param name="Seconds">Productive seconds, before field efficiency.</param>
            /// <param name="Pieces">Drawn pieces.</param>
            public CourseSegment
                (
                GuidanceAction Action,
                Boolean IsLane,
                Boolean Outside,
                String Word,
                Double LeadOutM,
                Double LeadInM,
                Double LengthM,
                Double Seconds,
                List<DrawnPiece> Pieces
                )
            {
                this.Action = Action;
                this.IsLane = IsLane;
                this.Outside = Outside;
                this.Word = Word;
                this.LeadOutM = LeadOutM;
                this.LeadInM = LeadInM;
                this.LengthM = LengthM;
                this.Seconds = Seconds;
                this.Pieces = Pieces;
            }

            /// <summary>Gets the action.</summary>
            public GuidanceAction Action { get; }

            /// <summary>Gets whether this is a blade-down lane.</summary>
            public Boolean IsLane { get; }

            /// <summary>Gets whether the connector left the field.</summary>
            public Boolean Outside { get; }

            /// <summary>Gets the Dubins word.</summary>
            public String Word { get; }

            /// <summary>Gets the lead-out length in metres.</summary>
            public Double LeadOutM { get; }

            /// <summary>Gets the lead-in length in metres.</summary>
            public Double LeadInM { get; }

            /// <summary>Gets the length in metres.</summary>
            public Double LengthM { get; }

            /// <summary>Gets productive seconds before field efficiency.</summary>
            public Double Seconds { get; }

            /// <summary>Gets the drawn pieces.</summary>
            public List<DrawnPiece> Pieces { get; }
        }

        /// <summary>
        /// One connector the report lists.
        /// </summary>
        public sealed class ConnectorNote
        {
            /// <summary>
            /// Initializes a note.
            /// </summary>
            /// <param name="TripNumber">1-based trip number.</param>
            /// <param name="Action">Which stretch it joins.</param>
            /// <param name="Word">Dubins word.</param>
            /// <param name="LengthM">Length including leads.</param>
            /// <param name="LeadOutM">Lead-out length.</param>
            /// <param name="LeadInM">Lead-in length.</param>
            /// <param name="Outside">True when it left the field.</param>
            public ConnectorNote
                (
                Int32 TripNumber,
                GuidanceAction Action,
                String Word,
                Double LengthM,
                Double LeadOutM,
                Double LeadInM,
                Boolean Outside
                )
            {
                this.TripNumber = TripNumber;
                this.Action = Action;
                this.Word = Word;
                this.LengthM = LengthM;
                this.LeadOutM = LeadOutM;
                this.LeadInM = LeadInM;
                this.Outside = Outside;
            }

            /// <summary>Gets the 1-based trip number.</summary>
            public Int32 TripNumber { get; }

            /// <summary>Gets the action.</summary>
            public GuidanceAction Action { get; }

            /// <summary>Gets the Dubins word.</summary>
            public String Word { get; }

            /// <summary>Gets the length in metres.</summary>
            public Double LengthM { get; }

            /// <summary>Gets the lead-out length.</summary>
            public Double LeadOutM { get; }

            /// <summary>Gets the lead-in length.</summary>
            public Double LeadInM { get; }

            /// <summary>Gets whether it left the field.</summary>
            public Boolean Outside { get; }
        }

        /// <summary>
        /// Soil the grade pass moved. Cubic metres of earth, conserved as
        /// cut = placed + still in the bowl.
        /// </summary>
        public sealed class GradeLedger
        {
            /// <summary>
            /// Initializes a ledger.
            /// </summary>
            /// <param name="CutM3">Soil cut down to target.</param>
            /// <param name="PlacedM3">Soil placed up to target.</param>
            /// <param name="BowlM3">Soil still in the bowl.</param>
            public GradeLedger
                (
                Double CutM3,
                Double PlacedM3,
                Double BowlM3
                )
            {
                this.CutM3 = CutM3;
                this.PlacedM3 = PlacedM3;
                this.BowlM3 = BowlM3;
            }

            /// <summary>Gets soil cut, cubic metres.</summary>
            public Double CutM3 { get; set; }

            /// <summary>Gets soil placed, cubic metres.</summary>
            public Double PlacedM3 { get; set; }

            /// <summary>Gets soil still in the bowl, cubic metres.</summary>
            public Double BowlM3 { get; set; }
        }

        /// <summary>
        /// One trip's path, clock, and grade ledger.
        /// </summary>
        public sealed class TripCourse
        {
            /// <summary>
            /// Initializes a trip.
            /// </summary>
            /// <param name="TripNumber">1-based trip number.</param>
            /// <param name="Stage">Which builder made the trip.</param>
            /// <param name="Segments">Lanes and connectors in drive order.</param>
            /// <param name="Points">Ordered guidance samples.</param>
            /// <param name="Notes">Connectors on this trip.</param>
            /// <param name="CutSeconds">Productive cut seconds.</param>
            /// <param name="LoadedSeconds">Productive loaded-connector seconds.</param>
            /// <param name="SpreadSeconds">Productive spread seconds.</param>
            /// <param name="TurnaroundSeconds">Productive turnaround seconds.</param>
            /// <param name="GradeSeconds">Productive grade-pass seconds.</param>
            /// <param name="EmptySeconds">Productive empty-connector seconds.</param>
            /// <param name="Grade">Grade-pass soil.</param>
            public TripCourse
                (
                Int32 TripNumber,
                HaulStage Stage,
                List<CourseSegment> Segments,
                List<GuidancePoint> Points,
                List<ConnectorNote> Notes,
                Double CutSeconds,
                Double LoadedSeconds,
                Double SpreadSeconds,
                Double TurnaroundSeconds,
                Double GradeSeconds,
                Double EmptySeconds,
                GradeLedger Grade
                )
            {
                this.TripNumber = TripNumber;
                this.Stage = Stage;
                this.Segments = Segments;
                this.Points = Points;
                this.Notes = Notes;
                this.CutSeconds = CutSeconds;
                this.LoadedSeconds = LoadedSeconds;
                this.SpreadSeconds = SpreadSeconds;
                this.TurnaroundSeconds = TurnaroundSeconds;
                this.GradeSeconds = GradeSeconds;
                this.EmptySeconds = EmptySeconds;
                this.Grade = Grade;
            }

            /// <summary>Gets the 1-based trip number.</summary>
            public Int32 TripNumber { get; }

            /// <summary>Gets the stage.</summary>
            public HaulStage Stage { get; }

            /// <summary>Gets the segments.</summary>
            public List<CourseSegment> Segments { get; }

            /// <summary>Gets the guidance points.</summary>
            public List<GuidancePoint> Points { get; }

            /// <summary>Gets the connector notes.</summary>
            public List<ConnectorNote> Notes { get; }

            /// <summary>Gets productive cut seconds.</summary>
            public Double CutSeconds { get; }

            /// <summary>Gets productive loaded-connector seconds.</summary>
            public Double LoadedSeconds { get; }

            /// <summary>Gets productive spread seconds.</summary>
            public Double SpreadSeconds { get; }

            /// <summary>Gets productive turnaround seconds.</summary>
            public Double TurnaroundSeconds { get; }

            /// <summary>Gets productive grade-pass seconds.</summary>
            public Double GradeSeconds { get; }

            /// <summary>Gets productive empty-connector seconds.</summary>
            public Double EmptySeconds { get; }

            /// <summary>Gets the grade-pass soil.</summary>
            public GradeLedger Grade { get; }

            /// <summary>
            /// Gets or sets the 1-based position in the order the trips are driven.
            /// Zero until the empty-return reorder assigns it.
            /// </summary>
            public Int32 ExecutionIndex { get; set; }

            /// <summary>
            /// Gets or sets whether this trip was built from deferred end fragments.
            /// Sweep-up trips follow the correcting pass.
            /// </summary>
            public Boolean SweepUp { get; set; }

            /// <summary>
            /// Gets or sets whether this trip is finishing work.
            /// Finishing follows sweep-up.
            /// </summary>
            public Boolean Finishing { get; set; }

            /// <summary>
            /// Gets or sets whether the cells were already a flow ribbon.
            /// A flow trip that is not a ribbon is a band.
            /// </summary>
            public Boolean RibbonOrder { get; set; }

            /// <summary>Gets or sets the cut load in bank cubic yards.</summary>
            public Double CutBcy { get; set; }

            /// <summary>
            /// Gets or sets whether the loaded connector crosses a fill cell
            /// other than the cell where the spread starts.
            /// </summary>
            public Boolean LoadedCrossesFill { get; set; }

            /// <summary>
            /// Gets or sets the stamp of the plan this trip was saved with.
            /// The viewer compares it with the stamp in the plan header.
            /// </summary>
            public String PlanStamp { get; set; } = "";

            /// <summary>
            /// Productive seconds of the six components, before field efficiency.
            /// </summary>
            public Double ProductiveSeconds
            {
                get
                {
                    return this.CutSeconds
                        + this.LoadedSeconds
                        + this.SpreadSeconds
                        + this.TurnaroundSeconds
                        + this.GradeSeconds
                        + this.EmptySeconds;
                }
            }
        }

        /// <summary>
        /// The directions chosen for one trip, and the course those directions drive.
        /// </summary>
        public sealed class DirectionChoice
        {
            /// <summary>
            /// Initializes a choice.
            /// </summary>
            /// <param name="Course">Connectors, turnaround, and grade pass for these directions.</param>
            /// <param name="Cuts">Cut lanes in the chosen drive order.</param>
            /// <param name="Fills">Spread lanes in the chosen drive order.</param>
            /// <param name="CutLines">Line id of each cut lane. Equal ids share a straight blade-up gap.</param>
            /// <param name="FillLines">Line id of each spread lane.</param>
            /// <param name="HeldCuts">Cut cells past the three-line cap.</param>
            /// <param name="HeldFills">Spread cells past the three-line cap.</param>
            /// <param name="EndCutFragments">Short end fragments deferred off the cut lines.</param>
            /// <param name="EndFillFragments">Short end fragments deferred off the spread lines.</param>
            public DirectionChoice
                (
                TripCourse Course,
                List<WorkLane> Cuts,
                List<WorkLane> Fills,
                List<Int32> CutLines,
                List<Int32> FillLines,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> HeldCuts,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> HeldFills,
                List<WorkLane> EndCutFragments,
                List<WorkLane> EndFillFragments
                )
            {
                this.Course = Course;
                this.Cuts = Cuts;
                this.Fills = Fills;
                this.CutLines = CutLines;
                this.FillLines = FillLines;
                this.HeldCuts = HeldCuts;
                this.HeldFills = HeldFills;
                this.EndCutFragments = EndCutFragments;
                this.EndFillFragments = EndFillFragments;
            }

            /// <summary>Gets the course for the chosen directions.</summary>
            public TripCourse Course { get; }

            /// <summary>Gets the cut lanes in the chosen drive order.</summary>
            public List<WorkLane> Cuts { get; }

            /// <summary>Gets the spread lanes in the chosen drive order.</summary>
            public List<WorkLane> Fills { get; }

            /// <summary>Gets the line id of each cut lane.</summary>
            public List<Int32> CutLines { get; }

            /// <summary>Gets the line id of each spread lane.</summary>
            public List<Int32> FillLines { get; }

            /// <summary>
            /// Gets cut cells past the three-line cap. They are not driven on this trip.
            /// </summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> HeldCuts { get; }

            /// <summary>
            /// Gets spread cells past the three-line cap. They are not driven on this trip.
            /// </summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> HeldFills { get; }

            /// <summary>
            /// Gets cut fragments left off the line. A sweep-up trip drives them later.
            /// </summary>
            public List<WorkLane> EndCutFragments { get; }

            /// <summary>
            /// Gets spread fragments left off the line. A sweep-up trip drives them later.
            /// </summary>
            public List<WorkLane> EndFillFragments { get; }
        }

        /// <summary>
        /// Every trip, the grade soil, and the ground the snapshot trip was about to touch.
        /// </summary>
        public sealed class JobGuidance
        {
            /// <summary>
            /// Initializes a job.
            /// </summary>
            /// <param name="Trips">Trips in haul order.</param>
            /// <param name="Surface">Ground after every trip, including grade passes.</param>
            /// <param name="Snapshot">Ground at the start of the requested trip. Null when none was asked for.</param>
            /// <param name="Efficiency">Field efficiency used to turn productive seconds into hours.</param>
            /// <param name="Snapshots">Ground at the start of each requested trip, keyed by 0-based index.</param>
            /// <param name="EmptyReturnHoursBefore">Charged empty-return hours in the planner's trip order.</param>
            /// <param name="EmptyReturnHoursAfter">Charged empty-return hours after the empty-travel reorder.</param>
            /// <param name="DriveOrder">Indexes into <paramref name="Trips"/> in the order they are driven.</param>
            /// <param name="MergedCount">Correcting trips folded into an adjacent-line trip.</param>
            /// <param name="DroppedCount">Correcting trips dropped because their cells were already within tolerance.</param>
            /// <param name="DeferredFragmentCount">End fragments taken off correcting lines for a sweep-up trip.</param>
            public JobGuidance
                (
                List<TripCourse> Trips,
                Double[,] Surface,
                Double[,]? Snapshot,
                Double Efficiency,
                Dictionary<Int32, Double[,]> Snapshots,
                Double EmptyReturnHoursBefore,
                Double EmptyReturnHoursAfter,
                List<Int32> DriveOrder,
                Int32 MergedCount,
                Int32 DroppedCount,
                Int32 DeferredFragmentCount
                )
            {
                this.Trips = Trips;
                this.Surface = Surface;
                this.Snapshot = Snapshot;
                this.Efficiency = Efficiency;
                this.Snapshots = Snapshots;
                this.EmptyReturnHoursBefore = EmptyReturnHoursBefore;
                this.EmptyReturnHoursAfter = EmptyReturnHoursAfter;
                this.DriveOrder = DriveOrder;
                this.MergedCount = MergedCount;
                this.DroppedCount = DroppedCount;
                this.DeferredFragmentCount = DeferredFragmentCount;
            }

            /// <summary>Gets the trips.</summary>
            public List<TripCourse> Trips { get; }

            /// <summary>Gets the ground after the last grade pass.</summary>
            public Double[,] Surface { get; }

            /// <summary>Gets the ground at the start of the snapshot trip.</summary>
            public Double[,]? Snapshot { get; }

            /// <summary>Gets the field efficiency.</summary>
            public Double Efficiency { get; }

            /// <summary>Gets the ground at the start of each requested trip.</summary>
            public Dictionary<Int32, Double[,]> Snapshots { get; }

            /// <summary>Gets charged empty-return hours before the trips were reordered.</summary>
            public Double EmptyReturnHoursBefore { get; }

            /// <summary>Gets charged empty-return hours after the trips were reordered.</summary>
            public Double EmptyReturnHoursAfter { get; }

            /// <summary>Gets indexes into <see cref="Trips"/> in the order they are driven.</summary>
            public List<Int32> DriveOrder { get; }

            /// <summary>Gets how many correcting trips were merged into a neighbour.</summary>
            public Int32 MergedCount { get; }

            /// <summary>Gets how many correcting trips were dropped.</summary>
            public Int32 DroppedCount { get; }

            /// <summary>Gets how many end fragments were deferred onto sweep-up trips.</summary>
            public Int32 DeferredFragmentCount { get; }

            /// <summary>
            /// Gets or sets blade-shaping hours for road slopes, crown, and pad exclusion zones.
            /// Zero when the plan has no roads. This allowance is not divided by field efficiency.
            /// </summary>
            public Double BladeShapingHours { get; set; }

            /// <summary>
            /// Hours by component and by stage, Dubins counts, leads, and outside connectors.
            /// </summary>
            /// <param name="PreviousHours">Hours of the previous build, for the comparison line.</param>
            /// <returns>The report text.</returns>
            public String FormatReport
                (
                Double PreviousHours
                )
            {
                Double Factor = ((this.Efficiency > 0.0) && (this.Efficiency <= 1.0)) ? this.Efficiency : 1.0;
                Double Cut = 0.0;
                Double Loaded = 0.0;
                Double Spread = 0.0;
                Double Turn = 0.0;
                Double Grade = 0.0;
                Double Empty = 0.0;
                Double GradeCut = 0.0;
                Double GradePlaced = 0.0;
                Double GradeBowl = 0.0;
                Dictionary<String, Int32> Words = new Dictionary<String, Int32>(StringComparer.Ordinal);
                List<ConnectorNote> Leads = new List<ConnectorNote>();
                List<ConnectorNote> Outside = new List<ConnectorNote>();
                Dictionary<HaulStage, Double[]> Stages = new Dictionary<HaulStage, Double[]>();
                foreach (TripCourse Trip in this.Trips)
                {
                    Cut += Trip.CutSeconds;
                    Loaded += Trip.LoadedSeconds;
                    Spread += Trip.SpreadSeconds;
                    Turn += Trip.TurnaroundSeconds;
                    Grade += Trip.GradeSeconds;
                    Empty += Trip.EmptySeconds;
                    GradeCut += Trip.Grade.CutM3;
                    GradePlaced += Trip.Grade.PlacedM3;
                    GradeBowl += Trip.Grade.BowlM3;
                    if (!Stages.ContainsKey(Trip.Stage))
                    {
                        Stages[Trip.Stage] = new Double[6];
                    }

                    Double[] Bucket = Stages[Trip.Stage];
                    Bucket[0] += Trip.CutSeconds;
                    Bucket[1] += Trip.LoadedSeconds;
                    Bucket[2] += Trip.SpreadSeconds;
                    Bucket[3] += Trip.TurnaroundSeconds;
                    Bucket[4] += Trip.GradeSeconds;
                    Bucket[5] += Trip.EmptySeconds;
                    foreach (ConnectorNote Note in Trip.Notes)
                    {
                        if (!Words.ContainsKey(Note.Word))
                        {
                            Words[Note.Word] = 0;
                        }

                        Words[Note.Word] += 1;
                        if ((Note.LeadOutM > 0.0) || (Note.LeadInM > 0.0))
                        {
                            Leads.Add(Note);
                        }

                        if (Note.Outside)
                        {
                            Outside.Add(Note);
                        }
                    }
                }

                Double Productive = Cut + Loaded + Spread + Turn + Grade + Empty;
                Double Charged = Productive / Factor;
                // Hours, trip count, and the stamp all come from this trip list.
                PlanOrder.Require(this.Trips, this.DriveOrder);
                Double PlanHours = PlanOrder.ChargedHours(this.Trips, Factor);
                if (Math.Abs(PlanHours - (Charged / Constants.SECONDS_PER_HOUR)) > 1e-6)
                {
                    throw new InvalidOperationException("Plan hours do not match the trips.");
                }

                String Stamp = PlanOrder.Stamp(this.Trips, this.DriveOrder, PlanHours);
                String Warning = PlanOrder.ViewerWarning(Stamp, this.Trips, this.DriveOrder, PlanHours);
                StringBuilder Text = new StringBuilder();
                Text.AppendLine("Guidance with Dubins connectors and a grade pass");
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Trips {0}",
                    this.Trips.Count));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Hours {0:F3}",
                    PlanHours));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Stamp {0}",
                    Stamp));
                if (Warning.Length > 0)
                {
                    Text.AppendLine(Warning);
                }

                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Previous build {0:F3} h    this build {1:F3} h",
                    PreviousHours,
                    PlanHours));
                PlanOrder.AppendDriveSample(Text, this.Trips, this.DriveOrder);
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Field efficiency {0:F2}. Hours below include it.",
                    Factor));
                Text.AppendLine();
                Text.AppendLine("Hours by component");
                AppendComponent(Text, "cut", Cut, Factor);
                AppendComponent(Text, "loaded connectors", Loaded, Factor);
                AppendComponent(Text, "spread", Spread, Factor);
                AppendComponent(Text, "turnaround", Turn, Factor);
                AppendComponent(Text, "grade pass", Grade, Factor);
                AppendComponent(Text, "empty connectors", Empty, Factor);
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Empty return before reorder  {0:F3} h",
                    this.EmptyReturnHoursBefore));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Empty return after reorder   {0:F3} h",
                    this.EmptyReturnHoursAfter));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Correcting trips merged  {0}",
                    this.MergedCount));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Correcting trips dropped {0}",
                    this.DroppedCount));
                Int32 SweepTrips = 0;
                Double SweepSeconds = 0.0;
                foreach (TripCourse Trip in this.Trips)
                {
                    if (!Trip.SweepUp)
                    {
                        continue;
                    }

                    SweepTrips += 1;
                    SweepSeconds += Trip.ProductiveSeconds;
                }

                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Fragments deferred {0}",
                    this.DeferredFragmentCount));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Sweep-up trips {0}",
                    SweepTrips));
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Sweep-up hours {0:F3} h",
                    (SweepSeconds / Factor) / Constants.SECONDS_PER_HOUR));
                Int32 RoadTrips = 0;
                Double RoadSeconds = 0.0;
                foreach (TripCourse Trip in this.Trips)
                {
                    if (Trip.Stage != HaulStage.Road)
                    {
                        continue;
                    }

                    RoadTrips += 1;
                    RoadSeconds += Trip.ProductiveSeconds;
                }

                if (RoadTrips > 0)
                {
                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "Road stage hours {0:F3} h",
                        (RoadSeconds / Factor) / Constants.SECONDS_PER_HOUR));
                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "Blade shaping {0:F3} h",
                        this.BladeShapingHours));
                }

                Text.AppendLine();
                Text.AppendLine("Reversals (connector turn over 150 deg)");
                foreach (HaulStage Stage in new HaulStage[]
                {
                    HaulStage.Flow,
                    HaulStage.Clusters,
                    HaulStage.ThinCut,
                    HaulStage.Correcting,
                })
                {
                    Int32 Reversals = 0;
                    foreach (TripCourse Trip in this.Trips)
                    {
                        if (Trip.Stage == Stage)
                        {
                            Reversals += ReversalCount(Trip);
                        }
                    }

                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "{0}  {1}",
                        Stage,
                        Reversals));
                }

                Text.AppendLine();
                Text.AppendLine("Hours by stage");
                foreach (HaulStage Stage in new HaulStage[]
                {
                    HaulStage.Flow,
                    HaulStage.Clusters,
                    HaulStage.ThinCut,
                    HaulStage.Correcting,
                    HaulStage.Road,
                })
                {
                    if (!Stages.ContainsKey(Stage))
                    {
                        continue;
                    }

                    Double[] Bucket = Stages[Stage];
                    Double Sum = Bucket[0] + Bucket[1] + Bucket[2] + Bucket[3] + Bucket[4] + Bucket[5];
                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "{0}  {1:F3} h",
                        Stage,
                        (Sum / Factor) / Constants.SECONDS_PER_HOUR));
                    AppendComponent(Text, "  cut", Bucket[0], Factor);
                    AppendComponent(Text, "  loaded connectors", Bucket[1], Factor);
                    AppendComponent(Text, "  spread", Bucket[2], Factor);
                    AppendComponent(Text, "  turnaround", Bucket[3], Factor);
                    AppendComponent(Text, "  grade pass", Bucket[4], Factor);
                    AppendComponent(Text, "  empty connectors", Bucket[5], Factor);
                }

                Text.AppendLine();
                Text.AppendLine("Connectors by Dubins type");
                foreach (String Word in new String[] { "LSL", "RSR", "LSR", "RSL", "RLR", "LRL", "S" })
                {
                    Int32 Count = Words.ContainsKey(Word) ? Words[Word] : 0;
                    Text.AppendLine(String.Format(CultureInfo.InvariantCulture, "{0}  {1}", Word, Count));
                }

                Text.AppendLine();
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Leads {0}",
                    Leads.Count));
                foreach (ConnectorNote Note in Leads)
                {
                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "trip {0} {1} {2} lead-out {3:F2} m lead-in {4:F2} m",
                        Note.TripNumber,
                        Note.Action,
                        Note.Word,
                        Note.LeadOutM,
                        Note.LeadInM));
                }

                Text.AppendLine();
                Text.AppendLine("Outside connectors by type");
                foreach (GuidanceAction Action in new GuidanceAction[]
                {
                    GuidanceAction.Carry,
                    GuidanceAction.Turn,
                    GuidanceAction.Grade,
                    GuidanceAction.Return,
                })
                {
                    Int32 Count = 0;
                    foreach (ConnectorNote Note in Outside)
                    {
                        if (Note.Action == Action)
                        {
                            Count += 1;
                        }
                    }

                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "{0}  {1}",
                        Action,
                        Count));
                }

                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "OUTSIDE {0}",
                    Outside.Count));
                foreach (ConnectorNote Note in Outside)
                {
                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "trip {0} {1} {2} {3:F1} m",
                        Note.TripNumber,
                        Note.Action,
                        Note.Word,
                        Note.LengthM));
                }

                Text.AppendLine();
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Grade pass cut {0:F3} m3 ({1:F3} BCY), placed {2:F3} m3, still in the bowl {3:F3} m3",
                    GradeCut,
                    GradeCut * Constants.M3_TO_CY,
                    GradePlaced,
                    GradeBowl));
                Text.AppendLine();
                Text.AppendLine("Hours by trip");
                foreach (TripCourse Trip in this.Trips)
                {
                    Text.AppendLine(String.Format(
                        CultureInfo.InvariantCulture,
                        "trip {0} {1} cut {2:F4} loaded {3:F4} spread {4:F4} turn {5:F4} grade {6:F4} empty {7:F4}",
                        Trip.TripNumber,
                        Trip.Stage,
                        (Trip.CutSeconds / Factor) / Constants.SECONDS_PER_HOUR,
                        (Trip.LoadedSeconds / Factor) / Constants.SECONDS_PER_HOUR,
                        (Trip.SpreadSeconds / Factor) / Constants.SECONDS_PER_HOUR,
                        (Trip.TurnaroundSeconds / Factor) / Constants.SECONDS_PER_HOUR,
                        (Trip.GradeSeconds / Factor) / Constants.SECONDS_PER_HOUR,
                        (Trip.EmptySeconds / Factor) / Constants.SECONDS_PER_HOUR));
                }

                AppendDiagnostics(Text, this.Trips);
                return Text.ToString();
            }
        }

        /// <summary>
        /// Along-track room a forward S-bend needs to shift sideways by
        /// <paramref name="LateralM"/> at turning radius <paramref name="RadiusM"/>.
        /// For a shift of at most two radii this is sqrt(d * (4R - d)):
        /// about 26 ft when the shift is one 8.5 ft lane and the radius is 22.5 ft.
        /// A wider shift uses two quarter turns, so the room is two radii.
        /// </summary>
        /// <param name="LateralM">Sideways shift, metres. The sign is ignored.</param>
        /// <param name="RadiusM">Turning radius, metres.</param>
        /// <returns>Metres along the departure heading. Zero when there is nothing to shift.</returns>
        static public Double ForwardSBendM
            (
            Double LateralM,
            Double RadiusM
            )
        {
            Double Lateral = Math.Abs(LateralM);
            if ((Lateral <= 1e-9) || (RadiusM <= 1e-9))
            {
                return 0.0;
            }

            // Past two radii the tight S would start turning back. Two quarter circles stay forward.
            if (Lateral > (2.0 * RadiusM))
            {
                return 2.0 * RadiusM;
            }

            return Math.Sqrt(Lateral * ((4.0 * RadiusM) - Lateral));
        }

        /// <summary>
        /// Builds guidance for every trip. Cell choice is the pass list's.
        /// Lane direction is chosen by time. The grade pass is the only extra
        /// earthwork, and a correcting trip does not grade.
        /// </summary>
        /// <param name="Grid">Field grid. Its mask is the field test.</param>
        /// <param name="Passes">Trips in haul order.</param>
        /// <param name="Clock">Haul, return, cut, and spread speeds.</param>
        /// <param name="BowlCapacityM3">Scraper bowl, cubic metres.</param>
        /// <param name="BladeWidthM">Blade width. Used only to keep the existing lanes.</param>
        /// <param name="RadiusM">Turning radius, metres.</param>
        /// <param name="SnapshotTripIndex">0-based trip whose start ground is kept. Negative keeps none.</param>
        /// <param name="Rules">Dubins rules. Null uses the production defaults.</param>
        /// <returns>The job.</returns>
        static public JobGuidance Build
            (
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            TimeParams Clock,
            Double BowlCapacityM3,
            Double BladeWidthM,
            Double RadiusM,
            Int32 SnapshotTripIndex,
            GuidanceOptions? Rules = null
            )
        {
            return Build(
                Grid,
                Passes,
                Clock,
                BowlCapacityM3,
                BladeWidthM,
                RadiusM,
                SnapshotTripIndex,
                null,
                Rules);
        }

        /// <summary>
        /// Builds guidance for every trip and keeps the start ground of each listed trip.
        /// </summary>
        /// <param name="Grid">Field grid. Its mask is the field test.</param>
        /// <param name="Passes">Trips in haul order.</param>
        /// <param name="Clock">Haul, return, cut, and spread speeds.</param>
        /// <param name="BowlCapacityM3">Scraper bowl, cubic metres.</param>
        /// <param name="BladeWidthM">Blade width. Used only to keep the existing lanes.</param>
        /// <param name="RadiusM">Turning radius, metres.</param>
        /// <param name="SnapshotTripIndex">0-based trip whose start ground is kept on <see cref="JobGuidance.Snapshot"/>. Negative keeps none.</param>
        /// <param name="SnapshotIndexes">Other 0-based trips whose start ground is kept. Null keeps only the single snapshot.</param>
        /// <param name="Rules">Dubins rules. Null uses the production defaults, or the rules of the build that called this.</param>
        /// <returns>The job.</returns>
        static public JobGuidance Build
            (
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            TimeParams Clock,
            Double BowlCapacityM3,
            Double BladeWidthM,
            Double RadiusM,
            Int32 SnapshotTripIndex,
            IReadOnlyCollection<Int32>? SnapshotIndexes,
            GuidanceOptions? Rules = null
            )
        {
            GuidanceOptions? PreviousRules = InstallRules(Rules);
            try
            {
            if (Grid == null)
            {
                throw new ArgumentNullException(nameof(Grid));
            }

            if (Passes == null)
            {
                throw new ArgumentNullException(nameof(Passes));
            }

            Double CircleFt = RadiusM / (Constants.FEET_TO_METERS * Constants.HALF_SCALE);
            List<Pass> Active = new List<Pass>(Passes.Count);
            Int32[] Origin = new Int32[Passes.Count];
            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                Active.Add(Passes[Index]);
                Origin[Index] = Index;
            }

            TripCourse[] Courses;
            List<WorkLane>[] ChosenCuts;
            List<WorkLane>[] ChosenFills;
            List<Int32>[] CutLines;
            List<Int32>[] FillLines;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] HeldCuts;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] HeldFills;
            List<WorkLane>[] EndCutFragments;
            List<WorkLane>[] EndFillFragments;
            PlanCourses(
                Grid, Active, Origin, Clock, BladeWidthM, RadiusM, CircleFt, BowlCapacityM3, true,
                out Courses, out ChosenCuts, out ChosenFills, out CutLines, out FillLines,
                out HeldCuts, out HeldFills, out EndCutFragments, out EndFillFragments);
            Int32 Merged = 0;
            Int32 Dropped = 0;
            List<Pass> MergedPasses;
            Int32[] MergedOrigin;
            Int32 MergedNow;
            Int32 DroppedNow;
            if (MergeLeftovers(
                Active,
                Origin,
                Courses,
                ChosenCuts,
                ChosenFills,
                BowlCapacityM3,
                RadiusM,
                out MergedPasses,
                out MergedOrigin,
                out MergedNow,
                out DroppedNow))
            {
                // The merged trips are new passes. Plan them once, then stop.
                Active = MergedPasses;
                Origin = MergedOrigin;
                Merged = MergedNow;
                Dropped = DroppedNow;
                PlanCourses(
                    Grid, Active, Origin, Clock, BladeWidthM, RadiusM, CircleFt, BowlCapacityM3, true,
                    out Courses, out ChosenCuts, out ChosenFills, out CutLines, out FillLines,
                    out HeldCuts, out HeldFills, out EndCutFragments, out EndFillFragments);
            }

            // End fragments were left off the correcting lines. Sweep-up trips
            // take them after that pass, so the source trip must not apply them.
            Int32 DeferredFragments = 0;
            List<WorkLane> PoolCuts = new List<WorkLane>();
            List<WorkLane> PoolFills = new List<WorkLane>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] EndSkipCuts =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[Active.Count];
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] EndSkipFills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[Active.Count];
            for (Int32 Index = 0; Index < Active.Count; Index += 1)
            {
                EndSkipCuts[Index] = CellsOfLanes(EndCutFragments[Index]);
                EndSkipFills[Index] = CellsOfLanes(EndFillFragments[Index]);
                DeferredFragments += EndCutFragments[Index].Count + EndFillFragments[Index].Count;
                PoolCuts.AddRange(EndCutFragments[Index]);
                PoolFills.AddRange(EndFillFragments[Index]);
            }

            if ((PoolCuts.Count > 0) || (PoolFills.Count > 0))
            {
                DubinsConnector.Pose? From = null;
                for (Int32 Index = Active.Count - 1; Index >= 0; Index -= 1)
                {
                    if (Active[Index].Stage == HaulStage.Correcting)
                    {
                        From = DepartureOf(Courses[Index]);
                        break;
                    }
                }

                List<Pass> SweepPasses = BuildSweepPasses(
                    PoolCuts, PoolFills, Grid, BowlCapacityM3, From);
                if (SweepPasses.Count > 0)
                {
                    Int32[] SweepOrigin = new Int32[SweepPasses.Count];
                    for (Int32 Index = 0; Index < SweepPasses.Count; Index += 1)
                    {
                        Active.Add(SweepPasses[Index]);
                        // Past every original trip number. Snapshots stay on the source trips.
                        SweepOrigin[Index] = Passes.Count + Index;
                    }

                    Origin = ConcatArrays(Origin, SweepOrigin);
                    TripCourse[] SweepCourses;
                    List<WorkLane>[] SweepCuts;
                    List<WorkLane>[] SweepFills;
                    List<Int32>[] SweepCutLines;
                    List<Int32>[] SweepFillLines;
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] SweepHeldCuts;
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] SweepHeldFills;
                    List<WorkLane>[] SweepEndCuts;
                    List<WorkLane>[] SweepEndFills;
                    // The pool is already the deferred work. Do not defer it again.
                    PlanCourses(
                        Grid, SweepPasses, SweepOrigin, Clock, BladeWidthM, RadiusM, CircleFt,
                        BowlCapacityM3, false,
                        out SweepCourses, out SweepCuts, out SweepFills, out SweepCutLines, out SweepFillLines,
                        out SweepHeldCuts, out SweepHeldFills, out SweepEndCuts, out SweepEndFills);
                    for (Int32 Index = 0; Index < SweepCourses.Length; Index += 1)
                    {
                        SweepCourses[Index].SweepUp = true;
                    }

                    Courses = ConcatArrays(Courses, SweepCourses);
                    ChosenCuts = ConcatArrays(ChosenCuts, SweepCuts);
                    ChosenFills = ConcatArrays(ChosenFills, SweepFills);
                    CutLines = ConcatArrays(CutLines, SweepCutLines);
                    FillLines = ConcatArrays(FillLines, SweepFillLines);
                    HeldCuts = ConcatArrays(HeldCuts, SweepHeldCuts);
                    HeldFills = ConcatArrays(HeldFills, SweepHeldFills);
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] ExtraCuts =
                        new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[SweepPasses.Count];
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] ExtraFills =
                        new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[SweepPasses.Count];
                    for (Int32 Index = 0; Index < SweepPasses.Count; Index += 1)
                    {
                        ExtraCuts[Index] = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                        ExtraFills[Index] = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                    }

                    EndSkipCuts = ConcatArrays(EndSkipCuts, ExtraCuts);
                    EndSkipFills = ConcatArrays(EndSkipFills, ExtraFills);
                }
            }

            Int32 Count = Active.Count;
            Double EmptyBeforeS = 0.0;
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                EmptyBeforeS += Courses[Index].EmptySeconds;
            }

            Int32[] DriveIndexes = OrderTrips(Courses, ChosenCuts, ChosenFills, Active, RadiusM);
            RequireSameSurface(
                Grid, Active, DriveIndexes, ChosenCuts, ChosenFills, HeldCuts, HeldFills, EndSkipCuts, EndSkipFills);
            TripCourse[] Rebuilt = new TripCourse[Count];
            for (Int32 Step = 0; Step < Count; Step += 1)
            {
                Int32 Index = DriveIndexes[Step];
                DubinsConnector.Pose? NextCut = null;
                if ((Step + 1) < Count)
                {
                    List<WorkLane> NextLanes = ChosenCuts[DriveIndexes[Step + 1]];
                    if ((NextLanes != null) && (NextLanes.Count > 0))
                    {
                        NextCut = NextLanes[0].Start;
                    }
                }

                // Same lanes. Only the empty return changes, because the next cut moved.
                // Joints were already trimmed while the directions were chosen.
                Rebuilt[Index] = BuildCourse(
                    ChosenCuts[Index],
                    ChosenFills[Index],
                    NextCut,
                    Grid.ContainsWorldXy,
                    Clock,
                    RadiusM,
                    Grid.GridSizeM,
                    Origin[Index] + 1,
                    Active[Index].Stage,
                    CutLines[Index],
                    FillLines[Index],
                    BladeWidthM,
                    false,
                    Grid.AreaM2,
                    BowlCapacityM3);
                Rebuilt[Index].SweepUp = Courses[Index].SweepUp;
                Rebuilt[Index].Finishing = Courses[Index].Finishing;
                Rebuilt[Index].RibbonOrder = Courses[Index].RibbonOrder;
                Rebuilt[Index].CutBcy = Courses[Index].CutBcy;
                // The loaded path is the one that is driven, after the empty return is rebuilt.
                Rebuilt[Index].LoadedCrossesFill = PlanOrder.LoadedCrossesFill(
                    Rebuilt[Index],
                    Grid,
                    Active[Index].FillCells);
                Rebuilt[Index].ExecutionIndex = Step + 1;
            }

            Double EmptyAfterS = 0.0;
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                EmptyAfterS += Rebuilt[Index].EmptySeconds;
            }

            Double[,] Surface = Grid.Delta();
            Double[,]? Snapshot = null;
            HashSet<Int32> KeepGround = new HashSet<Int32>();
            if ((SnapshotTripIndex >= 0) && (SnapshotTripIndex < Passes.Count))
            {
                KeepGround.Add(SnapshotTripIndex);
            }

            if (SnapshotIndexes != null)
            {
                foreach (Int32 Index in SnapshotIndexes)
                {
                    if ((Index >= 0) && (Index < Passes.Count))
                    {
                        KeepGround.Add(Index);
                    }
                }
            }

            Dictionary<Int32, Double[,]> KeptGround = new Dictionary<Int32, Double[,]>(KeepGround.Count);
            Double CarryM3 = 0.0;
            TripCourse? Previous = null;
            // Earthwork follows the shortened empty-return order. The trip list
            // stays in planner order so a trip number still names the same haul.
            for (Int32 Step = 0; Step < Count; Step += 1)
            {
                Int32 Index = DriveIndexes[Step];
                Int32 Original = Origin[Index];
                if (KeepGround.Contains(Original))
                {
                    // Clone before this trip's cut and fill, in the order it is driven.
                    Double[,] Copy = (Double[,])Surface.Clone();
                    KeptGround[Original] = Copy;
                    if (Original == SnapshotTripIndex)
                    {
                        Snapshot = Copy;
                    }
                }

                Pass Current = Active[Index];
                TripCourse Trip = Rebuilt[Index];
                HashSet<(Int32 Bx, Int32 By)> CutSkip = DeferredCells(ChosenCuts[Index]);
                HashSet<(Int32 Bx, Int32 By)> FillSkip = DeferredCells(ChosenFills[Index]);
                AddHeld(CutSkip, HeldCuts[Index]);
                AddHeld(FillSkip, HeldFills[Index]);
                AddHeld(CutSkip, EndSkipCuts[Index]);
                AddHeld(FillSkip, EndSkipFills[Index]);
                // A bend leaves the cells it no longer sweeps. Apply them on a later trip.
                ApplyPass(Surface, Current, CutSkip, FillSkip);
                // Leftover from the previous grade rides the next spread.
                if ((CarryM3 > 0.0) && (Previous != null))
                {
                    CarryM3 = PlaceOnCells(Surface, Grid, Current.FillCells, CarryM3, Previous.Grade);
                }

                if (HasGradeLane(Trip))
                {
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> GradeOrder =
                        GradeOrderOf(ChosenFills[Index], Current.FillCells, FillSkip);
                    List<DubinsConnector.Pose>? ReturnSamples = ReturnSamplesOf(Trip);
                    GradeLedger Moved = GradeCells(Surface, Grid, GradeOrder, BowlCapacityM3, ReturnSamples);
                    Trip.Grade.CutM3 = Moved.CutM3;
                    Trip.Grade.PlacedM3 = Moved.PlacedM3;
                    Trip.Grade.BowlM3 = Moved.BowlM3;
                    CarryM3 = Moved.BowlM3;
                }

                Previous = Trip;
            }

            List<TripCourse> Trips = new List<TripCourse>(Count);
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                Trips.Add(Rebuilt[Index]);
            }

            // Indexes into this trip list. Origin still names the pass the trip
            // came from, which is not a position in the list after a merge.
            List<Int32> DriveOrder = new List<Int32>(Count);
            for (Int32 Step = 0; Step < Count; Step += 1)
            {
                DriveOrder.Add(DriveIndexes[Step]);
            }

            Double Factor = ((Clock.FieldEfficiency > 0.0) && (Clock.FieldEfficiency <= 1.0))
                ? Clock.FieldEfficiency
                : 1.0;
            Double BeforeHours = (EmptyBeforeS / Factor) / Constants.SECONDS_PER_HOUR;
            Double AfterHours = (EmptyAfterS / Factor) / Constants.SECONDS_PER_HOUR;
            JobGuidance Job = new JobGuidance(
                Trips,
                Surface,
                Snapshot,
                Clock.FieldEfficiency,
                KeptGround,
                BeforeHours,
                AfterHours,
                DriveOrder,
                Merged,
                Dropped,
                DeferredFragments);
            // A stage that goes backwards fails the run before the plan is returned.
            PlanOrder.Require(Job.Trips, Job.DriveOrder);
            String Stamp = PlanOrder.Stamp(
                Job.Trips,
                Job.DriveOrder,
                PlanOrder.ChargedHours(Job.Trips, Factor));
            foreach (TripCourse Trip in Job.Trips)
            {
                Trip.PlanStamp = Stamp;
            }

            return Job;
            }
            finally
            {
                ActiveRules.Value = PreviousRules;
            }
        }

        /// <summary>
        /// Picks lane directions and boundary bends for one trip. Every spread
        /// lane may face either way. A cut lane may face either way only when it
        /// is flat; a slope keeps the downhill direction. An end within a radius
        /// of the boundary may also bend left or right. At most four lanes on
        /// each side are tried, and at most 256 combinations. The winner is the
        /// least time. That time
        /// includes the loaded connector from the last cut to the first spread,
        /// at the connector's real length. An equal time keeps the earlier
        /// combination, which is the planned lane. Outside connectors are
        /// counted in the report and do not decide the direction. A correcting
        /// trip gathers fragments that share a line, visits each line once in
        /// offset order, and keeps at most three cut lines and three spread
        /// lines. Spread lines snake. Those lines are clipped to their fragments
        /// and are not bent out to the field edge. A short end fragment more
        /// than four radii past the next fragment is deferred for a sweep-up
        /// trip. A flat solo cut may still reverse.
        /// </summary>
        /// <param name="CutLanes">Cut lanes in the planner's drive order.</param>
        /// <param name="FillLanes">Spread lanes in the planner's drive order.</param>
        /// <param name="NextCutStart">Next trip's first cut. Null after the last trip.</param>
        /// <param name="Grid">Field grid. Existing elevation is the loading slope.</param>
        /// <param name="Inside">Field test. Null does not test the field.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width. The bend keeps the band's outer edge on the boundary.</param>
        /// <param name="LaneSampleM">Point spacing on a lane.</param>
            /// <param name="TripNumber">1-based trip number.</param>
            /// <param name="Stage">Stage label.</param>
            /// <param name="CellAreaM2">Cell area. Zero skips the light-lane volume test.</param>
            /// <param name="BowlCapacityM3">Bowl capacity. Zero skips the light-lane volume test.</param>
            /// <param name="DeferEnds">True defers a short isolated end fragment. Sweep-up trips pass false.</param>
            /// <param name="Rules">Dubins rules. Null uses the production defaults, or the rules of the build that called this.</param>
            /// <returns>The chosen lanes and the course they drive.</returns>
            static public DirectionChoice ChooseDirections
            (
            IReadOnlyList<WorkLane> CutLanes,
            IReadOnlyList<WorkLane> FillLanes,
            DubinsConnector.Pose? NextCutStart,
            Grid Grid,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM,
            Double BladeWidthM,
            Double LaneSampleM,
            Int32 TripNumber,
            HaulStage Stage,
            Double CellAreaM2 = 0.0,
            Double BowlCapacityM3 = 0.0,
            Boolean DeferEnds = true,
            GuidanceOptions? Rules = null
            )
        {
            GuidanceOptions? PreviousRules = InstallRules(Rules);
            try
            {
            if (CutLanes == null)
            {
                throw new ArgumentNullException(nameof(CutLanes));
            }

            if (FillLanes == null)
            {
                throw new ArgumentNullException(nameof(FillLanes));
            }

            if (Grid == null)
            {
                throw new ArgumentNullException(nameof(Grid));
            }

            if (Stage == HaulStage.Road)
            {
                // The bank is one direction. The tie-break only chooses which end turns around.
                return ChooseRoadDirections(
                    CutLanes,
                    FillLanes,
                    NextCutStart,
                    Grid,
                    Inside,
                    Clock,
                    RadiusM,
                    BladeWidthM,
                    LaneSampleM,
                    TripNumber,
                    CellAreaM2,
                    BowlCapacityM3);
            }

            List<WorkLane> PreparedCuts;
            List<WorkLane> FillSource;
            List<Int32> CutLines;
            List<Int32> FillLines;
            List<Int32> FlatCuts;
            List<Int32> CutVariable;
            List<Int32> FillVariable;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> HeldCuts =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> HeldFills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            List<WorkLane> EndCutFragments = new List<WorkLane>();
            List<WorkLane> EndFillFragments = new List<WorkLane>();
            if (Stage == HaulStage.Correcting)
            {
                // One direction per line, then one sweep by offset. Extra lines are not driven.
                List<WorkLane> GroupedCuts = ArrangeOnLines(CutLanes, out CutLines);
                FillSource = ArrangeOnLines(FillLanes, out FillLines);
                if (DeferEnds)
                {
                    // A short end sitting more than four radii past the next fragment
                    // is cheaper as its own later trip than as a line extension.
                    GroupedCuts = TrimEndFragments(
                        GroupedCuts, CutLines, Grid, RadiusM, out CutLines, out EndCutFragments);
                    FillSource = TrimEndFragments(
                        FillSource, FillLines, Grid, RadiusM, out FillLines, out EndFillFragments);
                }

                GroupedCuts = SweepLines(GroupedCuts, CutLines, false, out CutLines, out HeldCuts);
                FillSource = SweepLines(FillSource, FillLines, true, out FillLines, out HeldFills);
                // The centreline stops on the fragments. The field edge is not part of the line.
                GroupedCuts = ClipToFragments(GroupedCuts, Grid);
                FillSource = ClipToFragments(FillSource, Grid);
                CutVariable = SoloIndexes(CutLines);
                // The serpentine sets every spread direction. The search must not flip it back.
                FillVariable = new List<Int32>();
                HashSet<Int32> CutSolo = new HashSet<Int32>(CutVariable);
                PreparedCuts = new List<WorkLane>(GroupedCuts.Count);
                FlatCuts = new List<Int32>();
                for (Int32 Index = 0; Index < GroupedCuts.Count; Index += 1)
                {
                    if (!CutSolo.Contains(Index))
                    {
                        // A shared line stays in the direction ArrangeOnLines picked.
                        PreparedCuts.Add(GroupedCuts[Index]);
                        continue;
                    }

                    List<WorkLane> One = PrepareCuts(
                        new List<WorkLane> { GroupedCuts[Index] },
                        Grid,
                        out List<Int32> OneFlat);
                    PreparedCuts.Add(One[0]);
                    if (OneFlat.Count > 0)
                    {
                        FlatCuts.Add(Index);
                    }
                }
            }
            else
            {
                PreparedCuts = PrepareCuts(CutLanes, Grid, out FlatCuts);
                FillSource = new List<WorkLane>(FillLanes);
                CutLines = IdentityLines(PreparedCuts.Count);
                FillLines = IdentityLines(FillSource.Count);
                CutVariable = IdentityLines(PreparedCuts.Count);
                FillVariable = IdentityLines(FillSource.Count);
            }

            // Straight is variant 0, so a tie keeps the planned direction.
            // A correcting line is not bent out to the field edge. A flat solo cut may still reverse.
            Boolean AllowBend = Stage != HaulStage.Correcting;
            List<LaneVariants> CutVariants = VariantsFor(
                PreparedCuts, CutVariable, FlatCuts, Grid, Inside, RadiusM, BladeWidthM, AllowBend);
            List<LaneVariants> FillVariants = VariantsFor(
                FillSource, FillVariable, FillVariable, Grid, Inside, RadiusM, BladeWidthM, AllowBend);
            List<LaneVariants> Sides = new List<LaneVariants>(CutVariants.Count + FillVariants.Count);
            Sides.AddRange(CutVariants);
            Sides.AddRange(FillVariants);
            TrimToBudget(Sides);
            Int32[] Choice = new Int32[Sides.Count];
            DirectionChoice? Best = null;
            Double BestTime = 0.0;
            Boolean More = true;
            while (More)
            {
                List<WorkLane> Cuts = Assemble(PreparedCuts, Sides, Choice, 0, CutVariants.Count);
                List<WorkLane> Fills = Assemble(FillSource, Sides, Choice, CutVariants.Count, FillVariants.Count);
                TripCourse Course = BuildCourse(
                    Cuts,
                    Fills,
                    NextCutStart,
                    Inside,
                    Clock,
                    RadiusM,
                    LaneSampleM,
                    TripNumber,
                    Stage,
                    CutLines,
                    FillLines,
                    BladeWidthM,
                    true,
                    CellAreaM2,
                    BowlCapacityM3);
                // Productive seconds include the loaded cut-to-spread connector.
                // A loop is charged at that connector's real length, not as zero.
                Double Time = Course.ProductiveSeconds;
                // Time alone. The first variant is the planned lane, so a tie leaves it alone.
                if ((Best == null) || ((Time + TIME_TIE_S) < BestTime))
                {
                    Best = new DirectionChoice(
                        Course, Cuts, Fills, CutLines, FillLines, HeldCuts, HeldFills,
                        EndCutFragments, EndFillFragments);
                    BestTime = Time;
                }

                More = AdvanceChoice(Sides, Choice);
            }

            if (Best == null)
            {
                throw new InvalidOperationException("A trip has no direction combination.");
            }

            return Best;
            }
            finally
            {
                ActiveRules.Value = PreviousRules;
            }
        }

        /// <summary>
        /// Direction choice for a road-stage trip. Every lane faces one way along
        /// the bank. The other way is the whole trip turned around, so the
        /// turnaround can sit at either end. Time breaks the tie, and an equal
        /// time keeps the planned direction. The turnaround word itself still
        /// prefers the path that stays in the field.
        /// </summary>
        /// <param name="CutLanes">Cut lanes in the planner's order.</param>
        /// <param name="FillLanes">Spread lanes in the planner's order.</param>
        /// <param name="NextCutStart">Next trip's first cut. Null on the last trip.</param>
        /// <param name="Grid">Field grid. Existing elevation locks a sloped cut.</param>
        /// <param name="Inside">Field test. Null ignores the boundary.</param>
        /// <param name="Clock">Speeds and field efficiency.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="LaneSampleM">Lane sample spacing.</param>
        /// <param name="TripNumber">1-based trip number.</param>
        /// <param name="CellAreaM2">Cell area.</param>
        /// <param name="BowlCapacityM3">Bowl capacity.</param>
        /// <returns>The chosen lanes and the course they drive.</returns>
        private static DirectionChoice ChooseRoadDirections
            (
            IReadOnlyList<WorkLane> CutLanes,
            IReadOnlyList<WorkLane> FillLanes,
            DubinsConnector.Pose? NextCutStart,
            Grid Grid,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM,
            Double BladeWidthM,
            Double LaneSampleM,
            Int32 TripNumber,
            Double CellAreaM2,
            Double BowlCapacityM3
            )
        {
            List<WorkLane> Cuts = PrepareCuts(CutLanes, Grid, out List<Int32> _);
            List<WorkLane> Fills = new List<WorkLane>(FillLanes);
            // A lane that came in facing the other way is turned with the trip, not on its own.
            AlignBank(Cuts, Fills);
            DirectionChoice Forward = DriveRoad(
                CopyWorkLanes(Cuts),
                CopyWorkLanes(Fills),
                NextCutStart,
                Inside,
                Clock,
                RadiusM,
                BladeWidthM,
                LaneSampleM,
                TripNumber,
                CellAreaM2,
                BowlCapacityM3);
            DirectionChoice Backward = DriveRoad(
                ReverseAll(Cuts),
                ReverseAll(Fills),
                NextCutStart,
                Inside,
                Clock,
                RadiusM,
                BladeWidthM,
                LaneSampleM,
                TripNumber,
                CellAreaM2,
                BowlCapacityM3);
            if ((Backward.Course.ProductiveSeconds + TIME_TIE_S) < Forward.Course.ProductiveSeconds)
            {
                return Backward;
            }

            return Forward;
        }

        /// <summary>
        /// Builds one road course. The lanes are already facing one way.
        /// </summary>
        /// <param name="Cuts">Cut lanes. Short joints may trim this list.</param>
        /// <param name="Fills">Spread lanes. Short joints may trim this list.</param>
        /// <param name="NextCutStart">Next trip's first cut. Null on the last trip.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="Clock">Speeds and field efficiency.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="LaneSampleM">Lane sample spacing.</param>
        /// <param name="TripNumber">1-based trip number.</param>
        /// <param name="CellAreaM2">Cell area.</param>
        /// <param name="BowlCapacityM3">Bowl capacity.</param>
        /// <returns>The course and the lanes it drove.</returns>
        private static DirectionChoice DriveRoad
            (
            List<WorkLane> Cuts,
            List<WorkLane> Fills,
            DubinsConnector.Pose? NextCutStart,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM,
            Double BladeWidthM,
            Double LaneSampleM,
            Int32 TripNumber,
            Double CellAreaM2,
            Double BowlCapacityM3
            )
        {
            List<Int32> CutLines = IdentityLines(Cuts.Count);
            List<Int32> FillLines = IdentityLines(Fills.Count);
            TripCourse Course = BuildCourse(
                Cuts,
                Fills,
                NextCutStart,
                Inside,
                Clock,
                RadiusM,
                LaneSampleM,
                TripNumber,
                HaulStage.Road,
                CutLines,
                FillLines,
                BladeWidthM,
                true,
                CellAreaM2,
                BowlCapacityM3);
            return new DirectionChoice(
                Course,
                Cuts,
                Fills,
                CutLines,
                FillLines,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                new List<WorkLane>(),
                new List<WorkLane>());
        }

        /// <summary>
        /// Turns every bank lane that faces backwards onto the fill heading.
        /// A cut that is square to the road, such as a field borrow, is left alone.
        /// </summary>
        /// <param name="Cuts">Cut lanes. Replaced in place when one is reversed.</param>
        /// <param name="Fills">Spread lanes. Replaced in place when one is reversed.</param>
        private static void AlignBank
            (
            List<WorkLane> Cuts,
            List<WorkLane> Fills
            )
        {
            Double Axis = 0.0;
            Boolean HaveAxis = false;
            if (Fills.Count > 0)
            {
                Axis = Fills[0].Start.HeadingRad;
                HaveAxis = true;
            }
            else if (Cuts.Count > 0)
            {
                Axis = Cuts[0].Start.HeadingRad;
                HaveAxis = true;
            }

            if (!HaveAxis)
            {
                return;
            }

            for (Int32 Index = 0; Index < Fills.Count; Index += 1)
            {
                if (FacingBack(Fills[Index], Axis))
                {
                    Fills[Index] = Fills[Index].Reversed();
                }
            }

            for (Int32 Index = 0; Index < Cuts.Count; Index += 1)
            {
                // Only an anti-parallel cut is a bank lane facing the wrong way.
                // A borrow that comes in from the side keeps the heading PrepareCuts chose.
                Double Turn = Math.Abs(DubinsConnector.WrapPi(Cuts[Index].Start.HeadingRad - Axis));
                if (Turn >= (Math.PI - LANE_CHANGE_HEADING_RAD))
                {
                    Cuts[Index] = Cuts[Index].Reversed();
                }
            }
        }

        /// <summary>
        /// True when a lane's heading is more than a right angle off the bank.
        /// </summary>
        /// <param name="Lane">Lane to test.</param>
        /// <param name="Axis">Bank heading, radians anticlockwise from east.</param>
        /// <returns>True when the lane should be reversed to match the bank.</returns>
        private static Boolean FacingBack
            (
            WorkLane Lane,
            Double Axis
            )
        {
            return Math.Abs(DubinsConnector.WrapPi(Lane.Start.HeadingRad - Axis)) > (Math.PI * 0.5);
        }

        /// <summary>
        /// Copies lanes so one direction search can trim them without moving the other.
        /// </summary>
        /// <param name="Lanes">Lanes to copy.</param>
        /// <returns>New lanes on the same centreline.</returns>
        private static List<WorkLane> CopyWorkLanes
            (
            IReadOnlyList<WorkLane> Lanes
            )
        {
            List<WorkLane> Copy = new List<WorkLane>(Lanes.Count);
            foreach (WorkLane Lane in Lanes)
            {
                Copy.Add(WorkLane.FromCenterline(
                    Lane.Centerline,
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Lane.Cells),
                    Lane.Start.HeadingRad));
            }

            return Copy;
        }

        /// <summary>
        /// Turns the trip around. The last lane is driven first, and each lane
        /// faces the other way, so a forward S-bend stays a forward S-bend.
        /// </summary>
        /// <param name="Lanes">Lanes in the planned direction.</param>
        /// <returns>The same lanes, driven from the other end.</returns>
        private static List<WorkLane> ReverseAll
            (
            IReadOnlyList<WorkLane> Lanes
            )
        {
            List<WorkLane> Reversed = new List<WorkLane>(Lanes.Count);
            for (Int32 Index = Lanes.Count - 1; Index >= 0; Index -= 1)
            {
                Reversed.Add(Lanes[Index].Reversed());
            }

            return Reversed;
        }

        /// <summary>
        /// Locks each cut to its downhill direction. A flat cut stays as planned
        /// and is returned in <paramref name="Flat"/> so the search may reverse it.
        /// </summary>
        /// <param name="Lanes">Cut lanes in the planner's order.</param>
        /// <param name="Grid">Existing elevation is the loading slope.</param>
        /// <param name="Flat">Indexes of lanes the search may reverse.</param>
        /// <returns>Lanes with uphill cuts already reversed.</returns>
        private static List<WorkLane> PrepareCuts
            (
            IReadOnlyList<WorkLane> Lanes,
            Grid Grid,
            out List<Int32> Flat
            )
        {
            List<WorkLane> Prepared = new List<WorkLane>(Lanes.Count);
            Flat = new List<Int32>();
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                WorkLane Lane = Lanes[Index];
                CutDirection Direction = LoadingDirection(Lane, Grid);
                if (Direction == CutDirection.Reverse)
                {
                    // The planner faced this cut uphill. Only the reverse may load.
                    Lane = Lane.Reversed();
                }

                Prepared.Add(Lane);
                if (Direction == CutDirection.Either)
                {
                    Flat.Add(Index);
                }
            }

            return Prepared;
        }

        /// <summary>
        /// The lanes the search may reverse. When more than four are free, the
        /// four whose forward end has the least open ground are the ones tried.
        /// Those are the ends a turnaround would leave the field from.
        /// </summary>
        /// <param name="Lanes">Lanes in drive order.</param>
        /// <param name="Free">Indexes that may be varied. A locked cut is still included when it can bend.</param>
        /// <param name="Inside">Field test. Null treats every end as open.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>At most four indexes, in lane order.</returns>
        private static List<Int32> FlipCandidates
            (
            IReadOnlyList<WorkLane> Lanes,
            IReadOnlyList<Int32> Free,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM
            )
        {
            List<(Double Clearance, Int32 Index)> Ranked =
                new List<(Double Clearance, Int32 Index)>(Free.Count);
            foreach (Int32 Index in Free)
            {
                Double Ahead = EndClearance(Lanes[Index], Inside, RadiusM);
                Double Edge = BoundaryDistance(Lanes[Index], Inside, RadiusM);
                Double Room = Ahead < Edge ? Ahead : Edge;
                Ranked.Add((Room, Index));
            }

            Ranked.Sort(CompareClearance);
            List<Int32> Picked = new List<Int32>();
            Int32 Take = Math.Min(MAX_FLIP_LANES, Ranked.Count);
            for (Int32 Index = 0; Index < Take; Index += 1)
            {
                Picked.Add(Ranked[Index].Index);
            }

            // Bit 0 is the earliest lane, so mask 0 stays the planned order.
            Picked.Sort();
            return Picked;
        }

        /// <summary>
        /// Orders tight ends first. Equal room keeps the earlier lane.
        /// </summary>
        /// <param name="Left">One lane's room and index.</param>
        /// <param name="Right">The other lane's room and index.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareClearance
            (
            (Double Clearance, Int32 Index) Left,
            (Double Clearance, Int32 Index) Right
            )
        {
            Int32 ByRoom = Left.Clearance.CompareTo(Right.Clearance);
            if (ByRoom != 0)
            {
                return ByRoom;
            }

            return Left.Index.CompareTo(Right.Index);
        }

        /// <summary>
        /// Open metres straight ahead of a lane end, before the mask runs out.
        /// </summary>
        /// <param name="Lane">Lane whose end is measured.</param>
        /// <param name="Inside">Field test. Null means the end is open.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>Metres of open ground, capped at four radii.</returns>
        private static Double EndClearance
            (
            WorkLane Lane,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM
            )
        {
            if (Inside == null)
            {
                return Double.PositiveInfinity;
            }

            Double Heading = Lane.End.HeadingRad;
            Double Cos = Math.Cos(Heading);
            Double Sin = Math.Sin(Heading);
            Double Limit = CLEARANCE_RADII * RadiusM;
            Double Clear = 0.0;
            // The turnaround bulb leaves forward of the end. Room along that ray
            // is what decides which lanes are worth reversing.
            while ((Clear + DubinsConnector.SAMPLE_M) <= Limit)
            {
                Double Ahead = Clear + DubinsConnector.SAMPLE_M;
                Double X = Lane.End.X + (Cos * Ahead);
                Double Y = Lane.End.Y + (Sin * Ahead);
                if (!Inside(X, Y))
                {
                    return Clear;
                }

                Clear = Ahead;
            }

            return Clear;
        }

        /// <summary>
        /// One lane's straight, bend, and reverse shapes, tightest room first.
        /// </summary>
        private sealed class LaneVariants
        {
            /// <summary>
            /// Initializes the shapes for one lane.
            /// </summary>
            /// <param name="Index">Index in that side's lane list.</param>
            /// <param name="Room">Metres of room at the tighter end. Smaller is tried first when trimming.</param>
            /// <param name="BaseCount">How many shapes are directions only. Trimming stops here.</param>
            /// <param name="Options">Shapes. The first is the planned lane.</param>
            public LaneVariants
                (
                Int32 Index,
                Double Room,
                Int32 BaseCount,
                List<WorkLane> Options
                )
            {
                this.Index = Index;
                this.Room = Room;
                this.BaseCount = BaseCount;
                this.Options = Options;
            }

            /// <summary>Gets the lane index on its side.</summary>
            public Int32 Index { get; }

            /// <summary>Gets the room at the tighter end.</summary>
            public Double Room { get; }

            /// <summary>Gets how many leading shapes are directions, not bends.</summary>
            public Int32 BaseCount { get; }

            /// <summary>Gets the shapes in search order.</summary>
            public List<WorkLane> Options { get; }
        }

        /// <summary>
        /// Builds the shapes for the lanes this side may vary.
        /// </summary>
        /// <param name="Lanes">Lanes in the legal drive order.</param>
        /// <param name="Variable">Indexes that may bend or, when also reversible, turn around.</param>
        /// <param name="Reversible">Indexes that may be driven the other way.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="AllowBend">False on a correcting line, which ends on its last fragment.</param>
        /// <returns>Lanes that have more than one shape.</returns>
        private static List<LaneVariants> VariantsFor
            (
            IReadOnlyList<WorkLane> Lanes,
            IReadOnlyList<Int32> Variable,
            IReadOnlyList<Int32> Reversible,
            Grid Grid,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM,
            Double BladeWidthM,
            Boolean AllowBend
            )
        {
            HashSet<Int32> MayReverse = new HashSet<Int32>(Reversible);
            List<Int32> Picked = FlipCandidates(Lanes, Variable, Inside, RadiusM);
            List<LaneVariants> Variants = new List<LaneVariants>();
            foreach (Int32 Index in Picked)
            {
                Boolean Reverse = MayReverse.Contains(Index);
                List<WorkLane> Options = ShapesFor(
                    Lanes[Index], Reverse, Grid, Inside, RadiusM, BladeWidthM, AllowBend);
                if (Options.Count < 2)
                {
                    continue;
                }

                Double Ahead = EndClearance(Lanes[Index], Inside, RadiusM);
                Double Edge = BoundaryDistance(Lanes[Index], Inside, RadiusM);
                Double Room = Ahead < Edge ? Ahead : Edge;
                // Directions are listed before bends, so trimming drops bends first.
                Int32 Base = Reverse ? 2 : 1;
                if (Base > Options.Count)
                {
                    Base = Options.Count;
                }

                Variants.Add(new LaneVariants(Index, Room, Base, Options));
            }

            return Variants;
        }

        /// <summary>
        /// Straight, then each bend, then the reverse and its bends.
        /// </summary>
        /// <param name="Lane">Lane in its legal direction.</param>
        /// <param name="AllowReverse">False when loading downhill forbids the other way.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="AllowBend">False on a correcting line, which ends on its last fragment.</param>
        /// <returns>At least the planned lane.</returns>
        private static List<WorkLane> ShapesFor
            (
            WorkLane Lane,
            Boolean AllowReverse,
            Grid Grid,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM,
            Double BladeWidthM,
            Boolean AllowBend
            )
        {
            List<WorkLane> Options = new List<WorkLane> { Lane };
            if (AllowReverse)
            {
                Options.Add(Lane.Reversed());
            }

            // A bend would carry the lane out to the field edge. Correcting lines stay on the fragments.
            if (!AllowBend)
            {
                return Options;
            }

            // Bends follow the directions so trimming can drop them first.
            AddBends(Options, Lane, Grid, Inside, RadiusM, BladeWidthM);
            if (AllowReverse)
            {
                AddBends(Options, Lane.Reversed(), Grid, Inside, RadiusM, BladeWidthM);
            }

            return Options;
        }

        /// <summary>
        /// Appends a left bend and a right bend at each end that is near the boundary.
        /// A bend whose arc would leave the field is left out.
        /// </summary>
        /// <param name="Options">Shapes gathered so far.</param>
        /// <param name="Lane">Lane the bend is attached to.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        private static void AddBends
            (
            List<WorkLane> Options,
            WorkLane Lane,
            Grid Grid,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM,
            Double BladeWidthM
            )
        {
            for (Int32 Sign = 1; Sign >= -1; Sign -= 2)
            {
                WorkLane? EndBend = BendEnd(Lane, Sign, Grid, Inside, RadiusM, BladeWidthM);
                if (EndBend != null)
                {
                    Options.Add(EndBend);
                }

                // The start is the mirror: the arc leaves the boundary, then the straight.
                WorkLane? StartBend = BendEnd(Lane.Reversed(), Sign, Grid, Inside, RadiusM, BladeWidthM);
                if (StartBend != null)
                {
                    Options.Add(StartBend.Reversed());
                }
            }
        }

        /// <summary>
        /// Shortens the lane and adds a blade-down quarter circle at the end.
        /// The arc ends with the band's outer edge on the boundary and the
        /// heading parallel to that boundary. Null when the arc would leave
        /// the field or the end is not up against the mask.
        /// </summary>
        /// <param name="Lane">Lane to bend. The bend is at its end.</param>
        /// <param name="TurnSign">+1 turns left, -1 turns right.</param>
        /// <param name="Grid">Field grid. Cell centres decide the swept band.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <returns>The bent lane, or null when this turn is not legal.</returns>
        private static WorkLane? BendEnd
            (
            WorkLane Lane,
            Int32 TurnSign,
            Grid Grid,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM,
            Double BladeWidthM
            )
        {
            if ((Inside == null) || (BladeWidthM <= 0.0) || (RadiusM <= 0.0) || (Lane.LengthM < MIN_STRAIGHT_AFTER_BEND_M))
            {
                return null;
            }

            if (!NearestBoundary(
                Lane.End.X,
                Lane.End.Y,
                Inside,
                RadiusM + (BladeWidthM * Constants.HALF_SCALE) + BEND_MARGIN_M,
                out Double Distance,
                out Double NormalX,
                out Double NormalY))
            {
                return null;
            }

            Double Heading = Lane.End.HeadingRad;
            Double ForwardX = Math.Cos(Heading);
            Double ForwardY = Math.Sin(Heading);
            Double Approach = (ForwardX * NormalX) + (ForwardY * NormalY);
            // A lane that merely runs along the boundary is not shortened into it.
            if (Approach < APPROACH_DOT)
            {
                return null;
            }

            DubinsConnector.PieceKind Kind = TurnSign > 0
                ? DubinsConnector.PieceKind.Left
                : DubinsConnector.PieceKind.Right;
            Double ArcLength = RadiusM * (Math.PI * Constants.HALF_SCALE);
            DubinsConnector.Pose Moved = DubinsConnector.Advance(
                new DubinsConnector.Pose(0.0, 0.0, Heading),
                Kind,
                ArcLength,
                RadiusM);
            Double DeltaNormal = (Moved.X * NormalX) + (Moved.Y * NormalY);
            // How much straight to drop so the arc finishes one blade-half inside the mask.
            Double Shorten = (DeltaNormal - Distance + (BladeWidthM * Constants.HALF_SCALE)) / Approach;
            if ((Shorten <= 0.05) || (Shorten > (Lane.LengthM - MIN_STRAIGHT_AFTER_BEND_M)))
            {
                return null;
            }

            if (!CutBack(Lane.Centerline, Shorten, out List<(Double X, Double Y)> Prefix, out Double StraightLength, out Double CutHeading))
            {
                return null;
            }

            (Double X, Double Y) Joint = Prefix[Prefix.Count - 1];
            DubinsConnector.Pose Cursor = new DubinsConnector.Pose(Joint.X, Joint.Y, CutHeading);
            if (!ArcStaysIn(Cursor, Kind, ArcLength, RadiusM, BladeWidthM, Inside))
            {
                return null;
            }

            DubinsConnector.Pose ArcEnd = DubinsConnector.Advance(Cursor, Kind, ArcLength, RadiusM);
            Double OuterX = ArcEnd.X + (NormalX * BladeWidthM * Constants.HALF_SCALE);
            Double OuterY = ArcEnd.Y + (NormalY * BladeWidthM * Constants.HALF_SCALE);
            // The outer edge has to land on the boundary, not a metre short of it.
            if (DistanceToBoundary(OuterX, OuterY, Inside, BEND_MARGIN_M) > 0.2)
            {
                return null;
            }

            List<(Double X, Double Y)> Line = new List<(Double X, Double Y)>(Prefix);
            Int32 Steps = Math.Max(1, (Int32)Math.Ceiling((ArcLength / DubinsConnector.SAMPLE_M) - 1e-9));
            for (Int32 Step = 1; Step <= Steps; Step += 1)
            {
                DubinsConnector.Pose At = DubinsConnector.Advance(Cursor, Kind, ArcLength * Step / Steps, RadiusM);
                Line.Add((At.X, At.Y));
            }

            // A short stub locks the end heading to the arc tangent.
            Line.Add((
                ArcEnd.X + (Math.Cos(ArcEnd.HeadingRad) * 0.01),
                ArcEnd.Y + (Math.Sin(ArcEnd.HeadingRad) * 0.01)));
            PartitionCells(Lane, Line, Grid, BladeWidthM, out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Swept, out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Left);
            WorkLane Shaped = WorkLane.FromCenterline(Line, Swept, CutHeading);
            return new WorkLane(
                Shaped.Centerline,
                Shaped.Start,
                Shaped.End,
                Shaped.Cells,
                StraightLength + ArcLength,
                StraightLength,
                ArcLength,
                Left);
        }

        /// <summary>
        /// True when the centreline and both blade edges stay in the field.
        /// A point on the boundary still counts.
        /// </summary>
        /// <param name="Start">Pose where the arc begins.</param>
        /// <param name="Kind">Left or right.</param>
        /// <param name="ArcLength">Arc length in metres.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="Inside">Field test.</param>
        /// <returns>False when any sample leaves the mask.</returns>
        private static Boolean ArcStaysIn
            (
            DubinsConnector.Pose Start,
            DubinsConnector.PieceKind Kind,
            Double ArcLength,
            Double RadiusM,
            Double BladeWidthM,
            Func<Double, Double, Boolean> Inside
            )
        {
            Int32 Steps = Math.Max(1, (Int32)Math.Ceiling((ArcLength / DubinsConnector.SAMPLE_M) - 1e-9));
            Double Half = BladeWidthM * Constants.HALF_SCALE;
            for (Int32 Step = 0; Step <= Steps; Step += 1)
            {
                DubinsConnector.Pose At = DubinsConnector.Advance(Start, Kind, ArcLength * Step / Steps, RadiusM);
                if (!NearlyInside(Inside, At.X, At.Y))
                {
                    return false;
                }

                Double Left = At.HeadingRad + (Math.PI * Constants.HALF_SCALE);
                Double Right = At.HeadingRad - (Math.PI * Constants.HALF_SCALE);
                if (!NearlyInside(Inside, At.X + (Math.Cos(Left) * Half), At.Y + (Math.Sin(Left) * Half)))
                {
                    return false;
                }

                if (!NearlyInside(Inside, At.X + (Math.Cos(Right) * Half), At.Y + (Math.Sin(Right) * Half)))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// True on the mask, and true on the boundary line itself.
        /// </summary>
        /// <param name="Inside">Field test.</param>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <returns>False when the point is clearly outside.</returns>
        private static Boolean NearlyInside
            (
            Func<Double, Double, Boolean> Inside,
            Double X,
            Double Y
            )
        {
            if (Inside(X, Y))
            {
                return true;
            }

            const Double NudgeM = 0.03;
            return Inside(X + NudgeM, Y)
                || Inside(X - NudgeM, Y)
                || Inside(X, Y + NudgeM)
                || Inside(X, Y - NudgeM);
        }

        /// <summary>
        /// Walks back from the end of a centreline.
        /// </summary>
        /// <param name="Line">Centreline in drive order.</param>
        /// <param name="DropFromEnd">Metres to remove from the end.</param>
        /// <param name="Prefix">The centreline that remains, including the cut point.</param>
        /// <param name="StraightLength">Length of <paramref name="Prefix"/>.</param>
        /// <param name="Heading">Heading of the last remaining segment.</param>
        /// <returns>False when the drop consumes the whole line.</returns>
        private static Boolean CutBack
            (
            IReadOnlyList<(Double X, Double Y)> Line,
            Double DropFromEnd,
            out List<(Double X, Double Y)> Prefix,
            out Double StraightLength,
            out Double Heading
            )
        {
            Prefix = new List<(Double X, Double Y)>();
            StraightLength = 0.0;
            Heading = 0.0;
            if (Line.Count < 2)
            {
                return false;
            }

            Double Total = 0.0;
            Double[] Segment = new Double[Line.Count];
            for (Int32 Index = 1; Index < Line.Count; Index += 1)
            {
                Double Dx = Line[Index].X - Line[Index - 1].X;
                Double Dy = Line[Index].Y - Line[Index - 1].Y;
                Segment[Index] = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                Total += Segment[Index];
            }

            Double Keep = Total - DropFromEnd;
            if (Keep < MIN_STRAIGHT_AFTER_BEND_M)
            {
                return false;
            }

            Prefix.Add(Line[0]);
            Double Walked = 0.0;
            for (Int32 Index = 1; Index < Line.Count; Index += 1)
            {
                Double Dx = Line[Index].X - Line[Index - 1].X;
                Double Dy = Line[Index].Y - Line[Index - 1].Y;
                if ((Walked + Segment[Index]) >= (Keep - 1e-9))
                {
                    Double Fraction = Segment[Index] <= 1e-9 ? 0.0 : (Keep - Walked) / Segment[Index];
                    Prefix.Add((Line[Index - 1].X + (Dx * Fraction), Line[Index - 1].Y + (Dy * Fraction)));
                    if (Segment[Index] > 1e-6)
                    {
                        Heading = Math.Atan2(Dy, Dx);
                    }

                    StraightLength = Keep;
                    return true;
                }

                Prefix.Add(Line[Index]);
                Walked += Segment[Index];
                if (Segment[Index] > 1e-6)
                {
                    Heading = Math.Atan2(Dy, Dx);
                }
            }

            return false;
        }

        /// <summary>
        /// Splits the straight lane's cells into those the bent band still covers
        /// and those left for a later trip. Nothing is added and nothing is shared.
        /// </summary>
        /// <param name="Source">Straight lane the cells came from.</param>
        /// <param name="Line">Bent centreline.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="Swept">Cells under the bent band, in drive order.</param>
        /// <param name="Left">Cells the straight reached and the bend does not.</param>
        private static void PartitionCells
            (
            WorkLane Source,
            IReadOnlyList<(Double X, Double Y)> Line,
            Grid Grid,
            Double BladeWidthM,
            out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Swept,
            out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Left
            )
        {
            Swept = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Left = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double Half = BladeWidthM * Constants.HALF_SCALE;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Kept =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            HashSet<(Int32 Bx, Int32 By)> Seen = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Source.Cells)
            {
                if (!Seen.Add(Entry.Cell))
                {
                    continue;
                }

                (Double X, Double Y) Center = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                if (DistanceToLine(Center.X, Center.Y, Line) <= (Half + 1e-6))
                {
                    Kept.Add(Entry);
                }
                else
                {
                    Left.Add(Entry);
                }
            }

            Kept.Sort((LeftCell, RightCell) =>
            {
                (Double X, Double Y) A = Grid.CellCenterXy(LeftCell.Cell.Bx, LeftCell.Cell.By);
                (Double X, Double Y) B = Grid.CellCenterXy(RightCell.Cell.Bx, RightCell.Cell.By);
                return StationOf(A.X, A.Y, Line).CompareTo(StationOf(B.X, B.Y, Line));
            });
            Swept.AddRange(Kept);
        }

        /// <summary>
        /// Distance from a point to a polyline.
        /// </summary>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <param name="Line">Polyline.</param>
        /// <returns>Metres.</returns>
        private static Double DistanceToLine
            (
            Double X,
            Double Y,
            IReadOnlyList<(Double X, Double Y)> Line
            )
        {
            if (Line.Count == 0)
            {
                return Double.PositiveInfinity;
            }

            if (Line.Count == 1)
            {
                Double Dx = X - Line[0].X;
                Double Dy = Y - Line[0].Y;
                return Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }

            Double Best = Double.PositiveInfinity;
            for (Int32 Index = 1; Index < Line.Count; Index += 1)
            {
                Double Distance = DistanceToSegment(X, Y, Line[Index - 1].X, Line[Index - 1].Y, Line[Index].X, Line[Index].Y);
                if (Distance < Best)
                {
                    Best = Distance;
                }
            }

            return Best;
        }

        /// <summary>
        /// Distance along a polyline to the closest point.
        /// </summary>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <param name="Line">Polyline.</param>
        /// <returns>Metres from the start of the line.</returns>
        private static Double StationOf
            (
            Double X,
            Double Y,
            IReadOnlyList<(Double X, Double Y)> Line
            )
        {
            Double Best = Double.PositiveInfinity;
            Double BestStation = 0.0;
            Double Walked = 0.0;
            for (Int32 Index = 1; Index < Line.Count; Index += 1)
            {
                Double X0 = Line[Index - 1].X;
                Double Y0 = Line[Index - 1].Y;
                Double X1 = Line[Index].X;
                Double Y1 = Line[Index].Y;
                Double Dx = X1 - X0;
                Double Dy = Y1 - Y0;
                Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                Double Fraction = 0.0;
                if (Length > 1e-9)
                {
                    Fraction = (((X - X0) * Dx) + ((Y - Y0) * Dy)) / (Length * Length);
                    if (Fraction < 0.0)
                    {
                        Fraction = 0.0;
                    }
                    else if (Fraction > 1.0)
                    {
                        Fraction = 1.0;
                    }
                }

                Double Px = X0 + (Dx * Fraction);
                Double Py = Y0 + (Dy * Fraction);
                Double Ex = X - Px;
                Double Ey = Y - Py;
                Double Distance = (Ex * Ex) + (Ey * Ey);
                if (Distance < Best)
                {
                    Best = Distance;
                    BestStation = Walked + (Length * Fraction);
                }

                Walked += Length;
            }

            return BestStation;
        }

        /// <summary>
        /// Distance from a point to a segment.
        /// </summary>
        /// <param name="X">Easting of the point.</param>
        /// <param name="Y">Northing of the point.</param>
        /// <param name="X0">Easting of the segment start.</param>
        /// <param name="Y0">Northing of the segment start.</param>
        /// <param name="X1">Easting of the segment end.</param>
        /// <param name="Y1">Northing of the segment end.</param>
        /// <returns>Metres.</returns>
        private static Double DistanceToSegment
            (
            Double X,
            Double Y,
            Double X0,
            Double Y0,
            Double X1,
            Double Y1
            )
        {
            Double Dx = X1 - X0;
            Double Dy = Y1 - Y0;
            Double Length = (Dx * Dx) + (Dy * Dy);
            Double Fraction = 0.0;
            if (Length > 1e-12)
            {
                Fraction = (((X - X0) * Dx) + ((Y - Y0) * Dy)) / Length;
                if (Fraction < 0.0)
                {
                    Fraction = 0.0;
                }
                else if (Fraction > 1.0)
                {
                    Fraction = 1.0;
                }
            }

            Double Ex = X - (X0 + (Dx * Fraction));
            Double Ey = Y - (Y0 + (Dy * Fraction));
            return Math.Sqrt((Ex * Ex) + (Ey * Ey));
        }

        /// <summary>
        /// Distance from a lane's nearer end to the field boundary.
        /// </summary>
        /// <param name="Lane">Lane to measure.</param>
        /// <param name="Inside">Field test. Null means the lane is not near a boundary.</param>
        /// <param name="RadiusM">Turning radius. The search stops after four radii.</param>
        /// <returns>Metres. Large when neither end is near the mask.</returns>
        private static Double BoundaryDistance
            (
            WorkLane Lane,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM
            )
        {
            if (Inside == null)
            {
                return Double.PositiveInfinity;
            }

            // Rank on the real gap, not only the bend threshold, so a lane whose
            // turnaround would leave is still varied when it is too far to bend.
            Double Limit = CLEARANCE_RADII * RadiusM;
            Double Start = DistanceToBoundary(Lane.Start.X, Lane.Start.Y, Inside, Limit);
            Double End = DistanceToBoundary(Lane.End.X, Lane.End.Y, Inside, Limit);
            return Start < End ? Start : End;
        }

        /// <summary>
        /// Outward normal and distance of the nearest mask edge.
        /// </summary>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="Limit">Stop looking past this many metres.</param>
        /// <param name="Distance">Metres to the boundary.</param>
        /// <param name="NormalX">Outward unit normal, east.</param>
        /// <param name="NormalY">Outward unit normal, north.</param>
        /// <returns>False when no boundary lies inside the limit.</returns>
        private static Boolean NearestBoundary
            (
            Double X,
            Double Y,
            Func<Double, Double, Boolean> Inside,
            Double Limit,
            out Double Distance,
            out Double NormalX,
            out Double NormalY
            )
        {
            Distance = DistanceToBoundary(X, Y, Inside, Limit);
            NormalX = 0.0;
            NormalY = 0.0;
            if (Distance > Limit)
            {
                return false;
            }

            // The nearest ray is the outward normal of that boundary segment.
            const Int32 Rays = 72;
            Double Best = Double.PositiveInfinity;
            for (Int32 Ray = 0; Ray < Rays; Ray += 1)
            {
                Double Angle = (Math.PI * 2.0 * Ray) / Rays;
                Double Hit = RayDistance(X, Y, Math.Cos(Angle), Math.Sin(Angle), Inside, Limit + 0.5);
                if (Hit < Best)
                {
                    Best = Hit;
                    NormalX = Math.Cos(Angle);
                    NormalY = Math.Sin(Angle);
                }
            }

            if (Best > Limit)
            {
                return false;
            }

            Distance = Best;
            return true;
        }

        /// <summary>
        /// Metres from a point to the nearest place outside the mask.
        /// </summary>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="Limit">Stop looking past this many metres.</param>
        /// <returns>The distance, or one past the limit when the neighbourhood is open.</returns>
        private static Double DistanceToBoundary
            (
            Double X,
            Double Y,
            Func<Double, Double, Boolean> Inside,
            Double Limit
            )
        {
            if (!Inside(X, Y))
            {
                return 0.0;
            }

            const Int32 Rays = 72;
            Double Best = Limit + 1.0;
            for (Int32 Ray = 0; Ray < Rays; Ray += 1)
            {
                Double Angle = (Math.PI * 2.0 * Ray) / Rays;
                Double Hit = RayDistance(X, Y, Math.Cos(Angle), Math.Sin(Angle), Inside, Limit + 0.5);
                if (Hit < Best)
                {
                    Best = Hit;
                }
            }

            return Best;
        }

        /// <summary>
        /// Metres along one ray until the mask runs out.
        /// </summary>
        /// <param name="X">Easting of the start.</param>
        /// <param name="Y">Northing of the start.</param>
        /// <param name="DirX">Unit east component.</param>
        /// <param name="DirY">Unit north component.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="Limit">Give up after this many metres.</param>
        /// <returns>The distance, or one past the limit when the ray stays inside.</returns>
        private static Double RayDistance
            (
            Double X,
            Double Y,
            Double DirX,
            Double DirY,
            Func<Double, Double, Boolean> Inside,
            Double Limit
            )
        {
            const Double StepM = 0.25;
            Double Traveled = 0.0;
            while ((Traveled + StepM) <= Limit)
            {
                Traveled += StepM;
                if (!Inside(X + (DirX * Traveled), Y + (DirY * Traveled)))
                {
                    Double Lo = Traveled - StepM;
                    Double Hi = Traveled;
                    // Enough steps that a cell edge and the opposite cell edge
                    // measure the same gap. The last inside point is the boundary.
                    for (Int32 Refine = 0; Refine < 24; Refine += 1)
                    {
                        Double Mid = (Lo + Hi) * Constants.HALF_SCALE;
                        if (Inside(X + (DirX * Mid), Y + (DirY * Mid)))
                        {
                            Lo = Mid;
                        }
                        else
                        {
                            Hi = Mid;
                        }
                    }

                    return Lo;
                }
            }

            return Limit + 1.0;
        }

        /// <summary>
        /// Cells a bend left behind, across every lane on one side.
        /// </summary>
        /// <param name="Lanes">Chosen lanes.</param>
        /// <returns>The deferred cell indexes.</returns>
        private static HashSet<(Int32 Bx, Int32 By)> DeferredCells
            (
            IReadOnlyList<WorkLane> Lanes
            )
        {
            HashSet<(Int32 Bx, Int32 By)> Skip = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (WorkLane Lane in Lanes)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Lane.Deferred)
                {
                    Skip.Add(Entry.Cell);
                }
            }

            return Skip;
        }

        /// <summary>
        /// Drops bend shapes from the loosest lanes until the search fits the budget.
        /// </summary>
        /// <param name="Sides">Every varied lane, cuts then spreads.</param>
        private static void TrimToBudget
            (
            List<LaneVariants> Sides
            )
        {
            while (CombinationCount(Sides) > MAX_COMBINATIONS)
            {
                LaneVariants? Loosest = null;
                foreach (LaneVariants Side in Sides)
                {
                    if (Side.Options.Count <= Side.BaseCount)
                    {
                        continue;
                    }

                    if ((Loosest == null) || (Side.Room > Loosest.Room))
                    {
                        Loosest = Side;
                    }
                }

                if (Loosest == null)
                {
                    return;
                }

                // The tail is a bend. Directions stay at the front.
                Loosest.Options.RemoveAt(Loosest.Options.Count - 1);
            }
        }

        /// <summary>
        /// Product of the shape counts.
        /// </summary>
        /// <param name="Sides">Varied lanes.</param>
        /// <returns>How many courses the search would build.</returns>
        private static Int64 CombinationCount
            (
            List<LaneVariants> Sides
            )
        {
            Int64 Count = 1;
            foreach (LaneVariants Side in Sides)
            {
                Count *= Side.Options.Count;
                if (Count > MAX_COMBINATIONS)
                {
                    return Count;
                }
            }

            return Count;
        }

        /// <summary>
        /// Copies the base lanes and writes the chosen shape into each varied slot.
        /// </summary>
        /// <param name="Base">Lanes before this combination.</param>
        /// <param name="Sides">All varied lanes.</param>
        /// <param name="Choice">Shape index per varied lane.</param>
        /// <param name="Offset">First varied lane that belongs to this side.</param>
        /// <param name="Count">How many varied lanes are on this side.</param>
        /// <returns>The lanes for one course.</returns>
        private static List<WorkLane> Assemble
            (
            IReadOnlyList<WorkLane> Base,
            List<LaneVariants> Sides,
            Int32[] Choice,
            Int32 Offset,
            Int32 Count
            )
        {
            List<WorkLane> Result = new List<WorkLane>(Base.Count);
            for (Int32 Index = 0; Index < Base.Count; Index += 1)
            {
                Result.Add(Base[Index]);
            }

            for (Int32 Slot = 0; Slot < Count; Slot += 1)
            {
                LaneVariants Side = Sides[Offset + Slot];
                Result[Side.Index] = Side.Options[Choice[Offset + Slot]];
            }

            return Result;
        }

        /// <summary>
        /// Steps the mixed-radix shape index. The first shape is tried first.
        /// </summary>
        /// <param name="Sides">Varied lanes.</param>
        /// <param name="Choice">Shape index per varied lane. Updated.</param>
        /// <returns>False when every combination has been visited.</returns>
        private static Boolean AdvanceChoice
            (
            List<LaneVariants> Sides,
            Int32[] Choice
            )
        {
            for (Int32 Slot = 0; Slot < Choice.Length; Slot += 1)
            {
                Choice[Slot] += 1;
                if (Choice[Slot] < Sides[Slot].Options.Count)
                {
                    return true;
                }

                Choice[Slot] = 0;
            }

            return false;
        }

        /// <summary>
        /// How many connectors on the course left the field.
        /// </summary>
        /// <param name="Course">One combination's course.</param>
        /// <returns>The outside-connector count.</returns>
        private static Int32 OutsideCount
            (
            TripCourse Course
            )
        {
            Int32 Count = 0;
            foreach (ConnectorNote Note in Course.Notes)
            {
                if (Note.Outside)
                {
                    Count += 1;
                }
            }

            return Count;
        }

        /// <summary>
        /// Downhill is the direction that loses elevation. A grade under a tenth
        /// of a percent is flat, so either direction may load. Steeper than that,
        /// only the descending direction is legal.
        /// </summary>
        /// <param name="Lane">Cut lane in the planner's order.</param>
        /// <param name="Grid">Existing elevation.</param>
        /// <returns>Keep, reverse, or either.</returns>
        private static CutDirection LoadingDirection
            (
            WorkLane Lane,
            Grid Grid
            )
        {
            if (Lane.LengthM < MIN_SLOPE_LENGTH_M)
            {
                return CutDirection.Either;
            }

            if (!TryEnds(Grid, Lane, out Double StartZ, out Double EndZ))
            {
                return CutDirection.Either;
            }

            Double Grade = (StartZ - EndZ) / Lane.LengthM;
            if (Grade > FLAT_LOAD_GRADE)
            {
                return CutDirection.Keep;
            }

            if (Grade < -FLAT_LOAD_GRADE)
            {
                return CutDirection.Reverse;
            }

            return CutDirection.Either;
        }

        /// <summary>
        /// Elevation at the two ends of a cut. Cells win when the lane has them.
        /// </summary>
        /// <param name="Grid">Existing elevation.</param>
        /// <param name="Lane">Cut lane.</param>
        /// <param name="StartZ">Elevation at the start.</param>
        /// <param name="EndZ">Elevation at the end.</param>
        /// <returns>False when either end is off the mask.</returns>
        private static Boolean TryEnds
            (
            Grid Grid,
            WorkLane Lane,
            out Double StartZ,
            out Double EndZ
            )
        {
            StartZ = 0.0;
            EndZ = 0.0;
            if (Lane.Cells.Count >= 2)
            {
                (Int32 Bx, Int32 By) StartCell = Lane.Cells[0].Cell;
                (Int32 Bx, Int32 By) EndCell = Lane.Cells[Lane.Cells.Count - 1].Cell;
                if (TryCell(Grid, StartCell, out StartZ) && TryCell(Grid, EndCell, out EndZ))
                {
                    return true;
                }
            }

            return TryWorld(Grid, Lane.Start.X, Lane.Start.Y, out StartZ)
                && TryWorld(Grid, Lane.End.X, Lane.End.Y, out EndZ);
        }

        /// <summary>
        /// Existing elevation of one cell.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cell">Cell index.</param>
        /// <param name="Elevation">Elevation when the cell is on the mask.</param>
        /// <returns>False when the cell is outside the grid or the mask.</returns>
        private static Boolean TryCell
            (
            Grid Grid,
            (Int32 Bx, Int32 By) Cell,
            out Double Elevation
            )
        {
            Elevation = 0.0;
            if ((Cell.Bx < 0) || (Cell.By < 0) || (Cell.Bx >= Grid.Width) || (Cell.By >= Grid.Height))
            {
                return false;
            }

            if (!Grid.Valid[Cell.By, Cell.Bx])
            {
                return false;
            }

            Elevation = Grid.Existing[Cell.By, Cell.Bx];
            return true;
        }

        /// <summary>
        /// Existing elevation of the cell that contains a world point.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="X">Easting, metres.</param>
        /// <param name="Y">Northing, metres.</param>
        /// <param name="Elevation">Elevation when the point is on the mask.</param>
        /// <returns>False when the point is outside the mask.</returns>
        private static Boolean TryWorld
            (
            Grid Grid,
            Double X,
            Double Y,
            out Double Elevation
            )
        {
            Int32 Bx = (Int32)Math.Floor((X - Grid.MinX) / Grid.GridSizeM);
            Int32 By = (Int32)Math.Floor((Y - Grid.MinY) / Grid.GridSizeM);
            return TryCell(Grid, (Bx, By), out Elevation);
        }

        /// <summary>
        /// Builds one trip from lanes that are already known. No earthwork.
        /// </summary>
        /// <param name="CutLanes">Cut lanes in drive order.</param>
        /// <param name="FillLanes">Spread lanes in drive order.</param>
        /// <param name="NextCutStart">Next trip's first cut. Null after the last trip.</param>
        /// <param name="Inside">Field test. Null does not test the field.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="LaneSampleM">Point spacing on a lane.</param>
        /// <param name="TripNumber">1-based trip number.</param>
        /// <param name="Stage">Stage label.</param>
        /// <param name="CutLineIds">Line id of each cut lane. Null gives every lane its own line.</param>
        /// <param name="FillLineIds">Line id of each spread lane. Null gives every lane its own line.</param>
        /// <param name="BladeWidthM">Blade width. Zero leaves short joints as Dubins paths.</param>
        /// <param name="FitJoints">True trims a short joint. False draws the lanes as they already are.</param>
        /// <param name="CellAreaM2">Cell area. Zero skips the light-lane volume test.</param>
        /// <param name="BowlCapacityM3">Bowl capacity. Zero skips the light-lane volume test.</param>
        /// <param name="Rules">Dubins rules. Null uses the production defaults, or the rules of the build that called this.</param>
        /// <returns>The trip path and clock.</returns>
        static public TripCourse BuildCourse
            (
            IReadOnlyList<WorkLane> CutLanes,
            IReadOnlyList<WorkLane> FillLanes,
            DubinsConnector.Pose? NextCutStart,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM,
            Double LaneSampleM,
            Int32 TripNumber,
            HaulStage Stage,
            IReadOnlyList<Int32>? CutLineIds = null,
            IReadOnlyList<Int32>? FillLineIds = null,
            Double BladeWidthM = 0.0,
            Boolean FitJoints = true,
            Double CellAreaM2 = 0.0,
            Double BowlCapacityM3 = 0.0,
            GuidanceOptions? Rules = null
            )
        {
            GuidanceOptions? PreviousRules = InstallRules(Rules);
            try
            {
            List<WorkLane> CutWork = new List<WorkLane>(CutLanes);
            List<WorkLane> FillWork = new List<WorkLane>(FillLanes);
            // Judge the lane before a short joint trims it. A trim is not a light load.
            Boolean[] CutKeep = KeepForDrive(CutWork, RadiusM, CellAreaM2, BowlCapacityM3);
            Boolean[] FillKeep = KeepForDrive(FillWork, RadiusM, CellAreaM2, BowlCapacityM3);
            if (FitJoints && (BladeWidthM > 0.0) && (RadiusM > 0.0))
            {
                // Short joints are trimmed on copies, then written back so the cells they skip stay deferred.
                FitShortJoints(CutWork, FillWork, RadiusM, BladeWidthM, Inside, Stage == HaulStage.Road);
                CopyLanes(CutLanes, CutWork);
                CopyLanes(FillLanes, FillWork);
            }

            // A near-empty lane is not a waypoint. Connectors meet across it.
            // The cells stay on the pass, so the cut and the fill still happen.
            List<WorkLane> CutDrive = DrivingLanes(CutWork, CutLineIds, CutKeep, out List<Int32>? CutDriveIds);
            List<WorkLane> FillDrive = DrivingLanes(FillWork, FillLineIds, FillKeep, out List<Int32>? FillDriveIds);

            List<CourseSegment> Segments = new List<CourseSegment>();
            List<GuidancePoint> Points = new List<GuidancePoint>();
            List<ConnectorNote> Notes = new List<ConnectorNote>();
            Double CutSeconds = 0.0;
            Double LoadedSeconds = 0.0;
            Double SpreadSeconds = 0.0;
            Double TurnSeconds = 0.0;
            Double GradeSeconds = 0.0;
            Double EmptySeconds = 0.0;
            Int32 Sequence = 1;
            // Correcting fragments on one line get a straight blade-up gap, not a Dubins bulb.
            Boolean StraightOnLine = Stage == HaulStage.Correcting;
            DubinsConnector.Pose? Cursor = DriveSide(
                Segments, Points, Notes, ref Sequence, ref CutSeconds, ref LoadedSeconds, ref SpreadSeconds,
                ref TurnSeconds, ref GradeSeconds, ref EmptySeconds, null, CutDrive, CutDriveIds,
                GuidanceAction.Cut, GuidanceAction.Carry, StraightOnLine, Inside, Clock, RadiusM, LaneSampleM, TripNumber, BladeWidthM, Stage == HaulStage.Road);
            Cursor = DriveSide(
                Segments, Points, Notes, ref Sequence, ref CutSeconds, ref LoadedSeconds, ref SpreadSeconds,
                ref TurnSeconds, ref GradeSeconds, ref EmptySeconds, Cursor, FillDrive, FillDriveIds,
                GuidanceAction.Fill, GuidanceAction.Carry, StraightOnLine, Inside, Clock, RadiusM, LaneSampleM, TripNumber, BladeWidthM, Stage == HaulStage.Road);

            if (FillDrive.Count > 0)
            {
                List<WorkLane> GradeLanes = new List<WorkLane>();
                List<Int32> GradeLineIds = new List<Int32>();
                for (Int32 Index = FillDrive.Count - 1; Index >= 0; Index -= 1)
                {
                    GradeLanes.Add(FillDrive[Index].Reversed());
                    if (FillDriveIds != null)
                    {
                        GradeLineIds.Add(FillDriveIds[Index]);
                    }
                }

                // One turnaround, onto the reversed last spread. Later lines do not each turn around.
                if (Cursor.HasValue)
                {
                    AddConnector(
                        Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                        ref GradeSeconds, ref EmptySeconds, Cursor.Value, GradeLanes[0].Start,
                        GuidanceAction.Turn, Inside, Clock, RadiusM, TripNumber);
                }

                // Correcting leaves the grade to the land plane, whatever the depth.
                Boolean DriveGrade = Stage != HaulStage.Correcting;
                if (DriveGrade)
                {
                    IReadOnlyList<Int32>? GradeIds = (FillDriveIds == null) ? null : GradeLineIds;
                    Cursor = DriveSide(
                        Segments, Points, Notes, ref Sequence, ref CutSeconds, ref LoadedSeconds, ref SpreadSeconds,
                        ref TurnSeconds, ref GradeSeconds, ref EmptySeconds, null, GradeLanes, GradeIds,
                        GuidanceAction.Grade, GuidanceAction.Grade, StraightOnLine, Inside, Clock, RadiusM, LaneSampleM, TripNumber, BladeWidthM, Stage == HaulStage.Road);
                }
                else
                {
                    Cursor = GradeLanes[0].Start;
                }
            }

            if (NextCutStart.HasValue && Cursor.HasValue)
            {
                AddConnector(
                    Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                    ref GradeSeconds, ref EmptySeconds, Cursor.Value, NextCutStart.Value,
                    GuidanceAction.Return, Inside, Clock, RadiusM, TripNumber);
            }

            // A turnaround and the empty return meet with no grade between them.
            // One Dubins path replaces that stack. Short joints have already been fitted.
            List<CourseSegment> Collapsed = CollapseStacked(
                Segments, Inside, Clock, RadiusM);
            if (!Object.ReferenceEquals(Collapsed, Segments))
            {
                Segments = Collapsed;
                RetotalCourse(
                    Segments,
                    Points,
                    Notes,
                    TripNumber,
                    ref Sequence,
                    ref CutSeconds,
                    ref LoadedSeconds,
                    ref SpreadSeconds,
                    ref TurnSeconds,
                    ref GradeSeconds,
                    ref EmptySeconds);
            }

            return new TripCourse(
                TripNumber,
                Stage,
                Segments,
                Points,
                Notes,
                CutSeconds,
                LoadedSeconds,
                SpreadSeconds,
                TurnSeconds,
                GradeSeconds,
                EmptySeconds,
                new GradeLedger(0.0, 0.0, 0.0));
            }
            finally
            {
                ActiveRules.Value = PreviousRules;
            }
        }

        /// <summary>
        /// Cuts cells that are high and fills cells that are low, one lift each.
        /// The bowl starts empty. Whatever is left is placed on the return, one
        /// lift per cell, and the remainder stays in the ledger.
        /// </summary>
        /// <param name="Surface">Existing-minus-target, metres. Updated in place.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cells">Cells in grade-drive order.</param>
        /// <param name="BowlCapacityM3">Bowl capacity, cubic metres.</param>
        /// <param name="ReturnSamples">Return centerline. Null leaves the bowl full.</param>
        /// <returns>The soil moved.</returns>
        static public GradeLedger GradeCells
            (
            Double[,] Surface,
            Grid Grid,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double BowlCapacityM3,
            IReadOnlyList<DubinsConnector.Pose>? ReturnSamples
            )
        {
            GradeLedger Ledger = new GradeLedger(0.0, 0.0, 0.0);
            HashSet<(Int32 Bx, Int32 By)> Seen = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                if (!Seen.Add(Entry.Cell))
                {
                    continue;
                }

                LiftCell(Surface, Grid, Entry.Cell.Bx, Entry.Cell.By, BowlCapacityM3, Ledger, true);
            }

            // The return is still empty of a plan. Drop on the first low cell it crosses.
            if ((ReturnSamples != null) && (Ledger.BowlM3 > 0.0))
            {
                HashSet<(Int32 Bx, Int32 By)> Dropped = new HashSet<(Int32 Bx, Int32 By)>();
                foreach (DubinsConnector.Pose Sample in ReturnSamples)
                {
                    if (Ledger.BowlM3 <= 0.0)
                    {
                        break;
                    }

                    Int32 Bx = (Int32)Math.Floor((Sample.X - Grid.MinX) / Grid.GridSizeM);
                    Int32 By = (Int32)Math.Floor((Sample.Y - Grid.MinY) / Grid.GridSizeM);
                    if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height))
                    {
                        continue;
                    }

                    if (!Grid.Valid[By, Bx] || !Dropped.Add((Bx, By)))
                    {
                        continue;
                    }

                    LiftCell(Surface, Grid, Bx, By, BowlCapacityM3, Ledger, false);
                }
            }

            return Ledger;
        }

        /// <summary>
        /// Appends one component line in charged hours.
        /// </summary>
        /// <param name="Text">Report being built.</param>
        /// <param name="Name">Component name.</param>
        /// <param name="ProductiveSeconds">Seconds before efficiency.</param>
        /// <param name="Efficiency">Field efficiency.</param>
        private static void AppendComponent
            (
            StringBuilder Text,
            String Name,
            Double ProductiveSeconds,
            Double Efficiency
            )
        {
            Text.AppendLine(String.Format(
                CultureInfo.InvariantCulture,
                "{0}  {1:F3} h",
                Name,
                (ProductiveSeconds / Efficiency) / Constants.SECONDS_PER_HOUR));
        }

        /// <summary>
        /// Lanes for one side of a trip. Ribbon passes keep ribbon order.
        /// </summary>
        /// <param name="Cells">Pass cells.</param>
        /// <param name="RibbonOrder">True when the cells are already in drive order.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="CircleFt">Turning circle diameter, feet. Splits only a real reversal.</param>
        /// <returns>Lanes in drive order.</returns>
        private static List<WorkLane> LanesOf
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Boolean RibbonOrder,
            Grid Grid,
            Double BladeWidthM,
            Double CircleFt
            )
        {
            List<WorkLane> Lanes = new List<WorkLane>();
            if ((Cells == null) || (Cells.Count == 0))
            {
                return Lanes;
            }

            // The lane builder is the one the guidance already used. Connectors
            // change after this returns. The cells on the lane do not.
            CoveragePlanner.PlanResult Plan = RibbonOrder
                ? CoveragePlanner.PlanRibbon(Cells, Grid, BladeWidthM, CircleFt)
                : CoveragePlanner.Plan(Cells, Grid, BladeWidthM, null, null, 0.0);
            foreach (CoveragePlanner.Run Run in Plan.Runs)
            {
                List<(Double X, Double Y)> Line = new List<(Double X, Double Y)>();
                if ((Run.DrivePoints != null) && (Run.DrivePoints.Count >= 2))
                {
                    Line.AddRange(Run.DrivePoints);
                }
                else
                {
                    Line.Add(Run.StartXy);
                    Line.Add(Run.EndXy);
                }

                Double Fallback = CompassToMath(Run.HeadingDeg);
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Owned =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Run.Cells);
                Lanes.Add(WorkLane.FromCenterline(Line, Owned, Fallback));
            }

            return Lanes;
        }

        /// <summary>
        /// Splits road-stage cells into blade-width lanes that all face the trip's long direction.
        /// The axis is the long axis of the cells, pointed from the first cell toward the
        /// farthest, so a lane stays one strip and a later lane is ahead.
        /// </summary>
        /// <param name="Cells">Cells in the order the road planner will drive them.</param>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="BladeWidthM">Lane width. One lane is one blade.</param>
        /// <returns>One lane per blade strip, in the order the strips first appear.</returns>
        private static List<WorkLane> RoadLanesOf
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double BladeWidthM
            )
        {
            List<WorkLane> Lanes = new List<WorkLane>();
            if ((Cells == null) || (Cells.Count == 0) || (BladeWidthM <= 0.0))
            {
                return Lanes;
            }

            (Double X, Double Y) Origin = Grid.CellCenterXy(Cells[0].Cell.Bx, Cells[0].Cell.By);
            (Double X, Double Y) Far = Origin;
            Double Far2 = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                Double Dx = Point.X - Origin.X;
                Double Dy = Point.Y - Origin.Y;
                Double Dist2 = (Dx * Dx) + (Dy * Dy);
                if (Dist2 > Far2)
                {
                    Far2 = Dist2;
                    Far = Point;
                }
            }

            // Along-lane steps, not the diagonal from the first cell to the farthest.
            // That diagonal leans across the bank and would split one lane into two.
            Double AxisX = 1.0;
            Double AxisY = 0.0;
            LaneAxis(Cells, Grid, Origin, Far, out AxisX, out AxisY);

            // Left of the axis, so a positive offset is one side of the bank.
            Double SideX = -AxisY;
            Double SideY = AxisX;
            Dictionary<Int32, List<(Double Along, ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry, Double X, Double Y)>> Bins =
                new Dictionary<Int32, List<(Double Along, ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry, Double X, Double Y)>>();
            List<Int32> BinOrder = new List<Int32>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                Double Dx = Point.X - Origin.X;
                Double Dy = Point.Y - Origin.Y;
                Double Along = (Dx * AxisX) + (Dy * AxisY);
                Double Side = (Dx * SideX) + (Dy * SideY);
                Int32 Bin = (Int32)Math.Round(Side / BladeWidthM, MidpointRounding.AwayFromZero);
                if (!Bins.TryGetValue(Bin, out List<(Double Along, ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry, Double X, Double Y)>? Group))
                {
                    Group = new List<(Double Along, ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry, Double X, Double Y)>();
                    Bins[Bin] = Group;
                    BinOrder.Add(Bin);
                }

                Group.Add((Along, Entry, Point.X, Point.Y));
            }

            foreach (Int32 Bin in BinOrder)
            {
                List<(Double Along, ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry, Double X, Double Y)> Group = Bins[Bin];
                Group.Sort((Left, Right) => Left.Along.CompareTo(Right.Along));
                List<(Double X, Double Y)> Line = new List<(Double X, Double Y)>();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Owned =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                foreach ((Double Along, ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry, Double X, Double Y) Item in Group)
                {
                    Line.Add((Item.X, Item.Y));
                    Owned.Add(Item.Entry);
                }

                if (Line.Count == 1)
                {
                    // A single cell has no heading. One metre along the trip keeps the lane on the bank.
                    Line.Add((Line[0].X + AxisX, Line[0].Y + AxisY));
                }

                Lanes.Add(WorkLane.FromCenterline(Line, Owned, Math.Atan2(AxisY, AxisX)));
            }

            return Lanes;
        }

        /// <summary>
        /// Direction the trip travels along the bank. Along-lane steps decide it.
        /// A step that is mostly sideways is a lane change and does not tilt the axis.
        /// The result points from the first cell toward the farthest.
        /// </summary>
        /// <param name="Cells">Cells in drive order.</param>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Origin">First cell centre.</param>
        /// <param name="Far">Cell centre farthest from the first.</param>
        /// <param name="AxisX">Unit axis, east.</param>
        /// <param name="AxisY">Unit axis, north.</param>
        private static void LaneAxis
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            (Double X, Double Y) Origin,
            (Double X, Double Y) Far,
            out Double AxisX,
            out Double AxisY
            )
        {
            BankAxis(Cells, Grid, out AxisX, out AxisY);
            Double Face = ((Far.X - Origin.X) * AxisX) + ((Far.Y - Origin.Y) * AxisY);
            if (Face < 0.0)
            {
                AxisX = -AxisX;
                AxisY = -AxisY;
            }

            Double Reference = Math.Atan2(AxisY, AxisX);
            List<Double> Offsets = new List<Double>();
            (Double X, Double Y) Previous = Origin;
            for (Int32 Index = 1; Index < Cells.Count; Index += 1)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(Cells[Index].Cell.Bx, Cells[Index].Cell.By);
                Double Dx = Point.X - Previous.X;
                Double Dy = Point.Y - Previous.Y;
                Previous = Point;
                Double Forward = (Dx * AxisX) + (Dy * AxisY);
                Double Side = (Dx * -AxisY) + (Dy * AxisX);
                // A lane change is mostly sideways. It must not pull the axis off the bank.
                if ((Math.Abs(Forward) < 0.2) || (Math.Abs(Forward) < Math.Abs(Side)))
                {
                    continue;
                }

                Offsets.Add(DubinsConnector.WrapPi(Math.Atan2(Dy, Dx) - Reference));
            }

            if (Offsets.Count == 0)
            {
                return;
            }

            Offsets.Sort();
            Double Heading = DubinsConnector.WrapPi(Reference + Offsets[Offsets.Count / 2]);
            AxisX = Math.Cos(Heading);
            AxisY = Math.Sin(Heading);
        }

        /// <summary>
        /// Long axis of a set of cells, before it is pointed along the trip.
        /// </summary>
        /// <param name="Cells">Cells on the trip.</param>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="AxisX">Unit axis, east. Not oriented to the trip.</param>
        /// <param name="AxisY">Unit axis, north. Not oriented to the trip.</param>
        private static void BankAxis
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            out Double AxisX,
            out Double AxisY
            )
        {
            AxisX = 1.0;
            AxisY = 0.0;
            Double MeanX = 0.0;
            Double MeanY = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                MeanX += Point.X;
                MeanY += Point.Y;
            }

            MeanX /= Cells.Count;
            MeanY /= Cells.Count;
            Double Sxx = 0.0;
            Double Syy = 0.0;
            Double Sxy = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                Double Dx = Point.X - MeanX;
                Double Dy = Point.Y - MeanY;
                Sxx += Dx * Dx;
                Syy += Dy * Dy;
                Sxy += Dx * Dy;
            }

            Double Shift = Sxx - Syy;
            Double Disc = Math.Sqrt((Shift * Shift) + (4.0 * Sxy * Sxy));
            Double Eigen = ((Sxx + Syy) + Disc) * 0.5;
            Double Vx = Sxy;
            Double Vy = Eigen - Sxx;
            Double Length = Math.Sqrt((Vx * Vx) + (Vy * Vy));
            if (Length <= 1e-9)
            {
                // The other component of the same eigenvector. East-west roads land here.
                Vx = Eigen - Syy;
                Vy = Sxy;
                Length = Math.Sqrt((Vx * Vx) + (Vy * Vy));
            }

            if (Length > 1e-9)
            {
                AxisX = Vx / Length;
                AxisY = Vy / Length;
            }
            else if (Syy > Sxx)
            {
                AxisX = 0.0;
                AxisY = 1.0;
            }
        }

        /// <summary>
        /// Grade-drive cell order: each spread lane reversed, last lane first.
        /// Any pass cell the lane builder did not keep is still graded, once.
        /// </summary>
        /// <param name="FillLanes">Spread lanes in forward order.</param>
        /// <param name="FillCells">Pass fill cells.</param>
        /// <param name="Skip">Cells a bend left for a later trip. They are not graded now.</param>
        /// <returns>Cells in grade order, each at most once.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> GradeOrderOf
            (
            List<WorkLane> FillLanes,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
            HashSet<(Int32 Bx, Int32 By)> Skip
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Order =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            HashSet<(Int32 Bx, Int32 By)> Seen = new HashSet<(Int32 Bx, Int32 By)>();
            for (Int32 Index = FillLanes.Count - 1; Index >= 0; Index -= 1)
            {
                WorkLane Lane = FillLanes[Index];
                for (Int32 CellIndex = Lane.Cells.Count - 1; CellIndex >= 0; CellIndex -= 1)
                {
                    ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Lane.Cells[CellIndex];
                    if (Seen.Add(Entry.Cell))
                    {
                        Order.Add(Entry);
                    }
                }
            }

            if (FillCells != null)
            {
                for (Int32 Index = FillCells.Count - 1; Index >= 0; Index -= 1)
                {
                    if (Skip.Contains(FillCells[Index].Cell))
                    {
                        continue;
                    }

                    if (Seen.Add(FillCells[Index].Cell))
                    {
                        Order.Add(FillCells[Index]);
                    }
                }
            }

            return Order;
        }

        /// <summary>
        /// Samples of the return connector, for the leftover drop.
        /// </summary>
        /// <param name="Trip">Trip whose return may cross a low cell.</param>
        /// <returns>The samples, or null when this trip does not return.</returns>
        private static List<DubinsConnector.Pose>? ReturnSamplesOf
            (
            TripCourse Trip
            )
        {
            List<DubinsConnector.Pose> Samples = new List<DubinsConnector.Pose>();
            foreach (CourseSegment Segment in Trip.Segments)
            {
                if (Segment.Action != GuidanceAction.Return)
                {
                    continue;
                }

                foreach (DrawnPiece Piece in Segment.Pieces)
                {
                    Samples.AddRange(Piece.Samples);
                }
            }

            return Samples.Count == 0 ? null : Samples;
        }

        /// <summary>
        /// Applies the pass the planner already recorded. This does not choose cells.
        /// </summary>
        /// <param name="Surface">Ground being replayed.</param>
        /// <param name="Pass">The recorded trip.</param>
        /// <param name="CutSkip">Cut cells a bend left for a later trip.</param>
        /// <param name="FillSkip">Fill cells a bend left for a later trip.</param>
        private static void ApplyPass
            (
            Double[,] Surface,
            Pass Pass,
            HashSet<(Int32 Bx, Int32 By)> CutSkip,
            HashSet<(Int32 Bx, Int32 By)> FillSkip
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.CutCells)
            {
                if (CutSkip.Contains(Entry.Cell))
                {
                    continue;
                }

                if (InSurface(Surface, Entry.Cell.Bx, Entry.Cell.By) && !Double.IsNaN(Surface[Entry.Cell.By, Entry.Cell.Bx]))
                {
                    Surface[Entry.Cell.By, Entry.Cell.Bx] -= Entry.DepthM;
                }
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.FillCells)
            {
                if (FillSkip.Contains(Entry.Cell))
                {
                    continue;
                }

                if (InSurface(Surface, Entry.Cell.Bx, Entry.Cell.By) && !Double.IsNaN(Surface[Entry.Cell.By, Entry.Cell.Bx]))
                {
                    Surface[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
                }
            }
        }

        /// <summary>
        /// Drops carried soil on the first low cells of a spread. One lift each.
        /// </summary>
        /// <param name="Surface">Ground.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cells">Spread cells in drive order.</param>
        /// <param name="CarryM3">Soil still in the bowl.</param>
        /// <param name="Ledger">Ledger of the trip that cut the soil.</param>
        /// <returns>Soil still not placed.</returns>
        private static Double PlaceOnCells
            (
            Double[,] Surface,
            Grid Grid,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double CarryM3,
            GradeLedger Ledger
            )
        {
            if ((Cells == null) || (CarryM3 <= 0.0))
            {
                return CarryM3;
            }

            Ledger.BowlM3 = CarryM3;
            HashSet<(Int32 Bx, Int32 By)> Seen = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                if (Ledger.BowlM3 <= 0.0)
                {
                    break;
                }

                if (!Seen.Add(Entry.Cell))
                {
                    continue;
                }

                LiftCell(Surface, Grid, Entry.Cell.Bx, Entry.Cell.By, Double.PositiveInfinity, Ledger, false);
            }

            return Ledger.BowlM3;
        }

        /// <summary>
        /// One lift on one cell. A cut only happens while the bowl has room and
        /// <paramref name="AllowCut"/> is set. A fill only happens while the bowl
        /// has soil. Neither crosses target.
        /// </summary>
        /// <param name="Surface">Ground.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="BowlCapacityM3">Bowl capacity.</param>
        /// <param name="Ledger">Soil accounting.</param>
        /// <param name="AllowCut">True on the grade pass. The return only places.</param>
        private static void LiftCell
            (
            Double[,] Surface,
            Grid Grid,
            Int32 Bx,
            Int32 By,
            Double BowlCapacityM3,
            GradeLedger Ledger,
            Boolean AllowCut
            )
        {
            if (!InSurface(Surface, Bx, By) || !Grid.Valid[By, Bx])
            {
                return;
            }

            Double Residual = Surface[By, Bx];
            if (Double.IsNaN(Residual))
            {
                return;
            }

            Double Area = Grid.AreaM2;
            if (AllowCut && (Residual > AT_TARGET_M))
            {
                Double RoomM = (BowlCapacityM3 - Ledger.BowlM3) / Area;
                if (RoomM <= 0.0)
                {
                    return;
                }

                Double Take = Math.Min(Residual, Math.Min(GRADE_LIFT_M, RoomM));
                if (Residual - Take < AT_TARGET_M)
                {
                    Take = Residual;
                }

                Surface[By, Bx] = Residual - Take;
                Double Volume = Take * Area;
                Ledger.CutM3 += Volume;
                Ledger.BowlM3 += Volume;
                return;
            }

            if ((Residual < -AT_TARGET_M) && (Ledger.BowlM3 > 0.0))
            {
                Double HaveM = Ledger.BowlM3 / Area;
                Double Place = Math.Min(-Residual, Math.Min(GRADE_LIFT_M, HaveM));
                if ((-Residual) - Place < AT_TARGET_M)
                {
                    Place = -Residual;
                }

                Surface[By, Bx] = Residual + Place;
                Double Volume = Place * Area;
                Ledger.PlacedM3 += Volume;
                Ledger.BowlM3 -= Volume;
            }
        }

        /// <summary>
        /// True when a column and row address the surface.
        /// </summary>
        /// <param name="Surface">Ground.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <returns>True when the indexes are inside.</returns>
        private static Boolean InSurface
            (
            Double[,] Surface,
            Int32 Bx,
            Int32 By
            )
        {
            return (By >= 0) && (Bx >= 0) && (By < Surface.GetLength(0)) && (Bx < Surface.GetLength(1));
        }

        /// <summary>
        /// Adds a lane and its points. Lane time is length over the lane speed.
        /// The grade pass uses <see cref="GuidanceOptions.GradeLaneSpeedMph"/>.
        /// </summary>
        private static void AddLane
            (
            List<CourseSegment> Segments,
            List<GuidancePoint> Points,
            ref Int32 Sequence,
            ref Double CutSeconds,
            ref Double SpreadSeconds,
            ref Double GradeSeconds,
            WorkLane Lane,
            GuidanceAction Action,
            Double LaneSampleM,
            TimeParams Clock
            )
        {
            List<DubinsConnector.Pose> Samples = SampleLine(Lane, LaneSampleM);
            DubinsConnector.Pose? Arrow = null;
            if (Samples.Count > 0)
            {
                Arrow = Samples[Samples.Count / 2];
            }

            List<DrawnPiece> Pieces = new List<DrawnPiece>
            {
                new DrawnPiece(DubinsConnector.PieceKind.Straight, Lane.LengthM, 0.0, Samples, Arrow),
            };
            Double SpeedMph = Clock.CutSpeedMph;
            if (Action == GuidanceAction.Fill)
            {
                SpeedMph = Clock.FillSpeedMph;
            }
            else if (Action == GuidanceAction.Grade)
            {
                SpeedMph = CurrentRules.GradeLaneSpeedMph;
            }

            Double Seconds = 0.0;
            if (SpeedMph > 0.0)
            {
                // The straight runs at cut or fill speed. The quarter circle is
                // the same speed, and never faster than the arc speed.
                Double ArcMph = Math.Min(SpeedMph, CurrentRules.ArcSpeedMph);
                Seconds = Lane.StraightLengthM / (SpeedMph * Constants.MPH_TO_MPS);
                if (ArcMph > 0.0)
                {
                    Seconds += Lane.ArcLengthM / (ArcMph * Constants.MPH_TO_MPS);
                }
            }

            if (Action == GuidanceAction.Cut)
            {
                CutSeconds += Seconds;
            }
            else if (Action == GuidanceAction.Fill)
            {
                SpreadSeconds += Seconds;
            }
            else
            {
                GradeSeconds += Seconds;
            }

            CourseSegment Segment = new CourseSegment(
                Action, true, false, String.Empty, 0.0, 0.0, Lane.LengthM, Seconds, Pieces);
            Segments.Add(Segment);
            AppendPoints(Points, ref Sequence, Segment);
        }

        /// <summary>
        /// Adds a Dubins connector. Arcs use the guidance arc speed. Straights use the
        /// loaded speed on a carry and the return speed everywhere else.
        /// </summary>
        /// <param name="Segments">Segments being built.</param>
        /// <param name="Points">Guidance samples.</param>
        /// <param name="Notes">Connector notes.</param>
        /// <param name="Sequence">Next sample number.</param>
        /// <param name="LoadedSeconds">Loaded-connector seconds so far.</param>
        /// <param name="TurnSeconds">Turnaround seconds so far.</param>
        /// <param name="GradeSeconds">Grade seconds so far.</param>
        /// <param name="EmptySeconds">Empty-connector seconds so far.</param>
        /// <param name="Start">Pose the connector leaves.</param>
        /// <param name="End">Pose the connector arrives at.</param>
        /// <param name="Action">What the connector is.</param>
        /// <param name="Inside">Field test. Null does not test the field.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="TripNumber">1-based trip number.</param>
        private static void AddConnector
            (
            List<CourseSegment> Segments,
            List<GuidancePoint> Points,
            List<ConnectorNote> Notes,
            ref Int32 Sequence,
            ref Double LoadedSeconds,
            ref Double TurnSeconds,
            ref Double GradeSeconds,
            ref Double EmptySeconds,
            DubinsConnector.Pose Start,
            DubinsConnector.Pose End,
            GuidanceAction Action,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM,
            Int32 TripNumber
            )
        {
            CourseSegment? Built = MakeConnector(Start, End, Action, Inside, Clock, RadiusM);
            if (Built == null)
            {
                return;
            }

            if (Action == GuidanceAction.Carry)
            {
                LoadedSeconds += Built.Seconds;
            }
            else if (Action == GuidanceAction.Turn)
            {
                TurnSeconds += Built.Seconds;
            }
            else if (Action == GuidanceAction.Grade)
            {
                GradeSeconds += Built.Seconds;
            }
            else
            {
                EmptySeconds += Built.Seconds;
            }

            Segments.Add(Built);
            Notes.Add(new ConnectorNote(
                TripNumber,
                Action,
                Built.Word,
                Built.LengthM,
                Built.LeadOutM,
                Built.LeadInM,
                Built.Outside));
            AppendPoints(Points, ref Sequence, Built);
        }

        /// <summary>
        /// Picks the connector for one pair of poses. The six Dubins words are
        /// the candidates. Within the tie of the shortest, the least length
        /// outside the field wins, then the shortest. With
        /// <see cref="GuidanceOptions.AllowLeaveField"/> off, a path that stays
        /// in the field wins over any path that leaves. With it on, a clearly
        /// shorter path stays chosen, including one that leaves the field.
        /// </summary>
        /// <param name="Start">Pose the connector leaves.</param>
        /// <param name="End">Pose the connector arrives at.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Inside">Field test. Null treats every path as inside.</param>
        /// <returns>The chosen path. Never null.</returns>
        private static DubinsConnector.Path SelectConnector
            (
            DubinsConnector.Pose Start,
            DubinsConnector.Pose End,
            Double RadiusM,
            Func<Double, Double, Boolean>? Inside
            )
        {
            List<DubinsConnector.Path> Words = DubinsConnector.AllWords(Start, End, RadiusM);
            if (Words.Count == 0)
            {
                return DubinsConnector.Connect(Start, End, RadiusM, null);
            }

            Int32 Chosen = PreferField(Words, RadiusM, Inside);
            if (Chosen < 0)
            {
                Chosen = 0;
            }

            return MarkOutside(Words[Chosen], RadiusM, Inside);
        }

        /// <summary>
        /// Index of the candidate that stays in the field when it is nearly the
        /// shortest. Equal outside length keeps the shorter path. An equal
        /// length keeps the earlier candidate. With leaving forbidden, the
        /// shortest path that stays inside wins whenever one exists.
        /// </summary>
        /// <param name="Candidates">Paths between the same kind of join. At least one.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Inside">Field test. Null treats every path as inside.</param>
        /// <returns>Index into <paramref name="Candidates"/>.</returns>
        private static Int32 PreferField
            (
            IReadOnlyList<DubinsConnector.Path> Candidates,
            Double RadiusM,
            Func<Double, Double, Boolean>? Inside
            )
        {
            if (!CurrentRules.AllowLeaveField)
            {
                Int32 InsideBest = -1;
                for (Int32 Index = 0; Index < Candidates.Count; Index += 1)
                {
                    if (OutsideLengthM(Candidates[Index], RadiusM, Inside) > LENGTH_TIE_M)
                    {
                        continue;
                    }

                    // Equal length keeps the earlier word.
                    if ((InsideBest < 0)
                        || ((Candidates[Index].LengthM + LENGTH_TIE_M) < Candidates[InsideBest].LengthM))
                    {
                        InsideBest = Index;
                    }
                }

                if (InsideBest >= 0)
                {
                    return InsideBest;
                }

                // No word stays in the field. The shorter leaving path is still drawn.
            }

            Double Shortest = Double.PositiveInfinity;
            for (Int32 Index = 0; Index < Candidates.Count; Index += 1)
            {
                if (Candidates[Index].LengthM < Shortest)
                {
                    Shortest = Candidates[Index].LengthM;
                }
            }

            // The tie band is a fraction of the shortest length, or a number of
            // radii when that is wider. Past the band, the shorter path wins.
            Double Allowance = Math.Max(CurrentRules.ConnectorTieFraction * Shortest, CurrentRules.ConnectorTieRadii * RadiusM);
            Double Limit = Shortest + Allowance;
            Int32 Best = 0;
            Double BestOutside = Double.PositiveInfinity;
            for (Int32 Index = 0; Index < Candidates.Count; Index += 1)
            {
                DubinsConnector.Path Candidate = Candidates[Index];
                if (Candidate.LengthM > (Limit + LENGTH_TIE_M))
                {
                    continue;
                }

                Double Outside = OutsideLengthM(Candidate, RadiusM, Inside);
                Boolean LessOutside = (Outside + LENGTH_TIE_M) < BestOutside;
                Boolean SameOutside = Math.Abs(Outside - BestOutside) <= LENGTH_TIE_M;
                Boolean Shorter = (Candidate.LengthM + LENGTH_TIE_M) < Candidates[Best].LengthM;
                if ((BestOutside == Double.PositiveInfinity) || LessOutside || (SameOutside && Shorter))
                {
                    Best = Index;
                    BestOutside = Outside;
                }
            }

            return Best;
        }

        /// <summary>
        /// Marks a path that spends any length outside the field.
        /// </summary>
        /// <param name="Path">Chosen connector.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Inside">Field test. Null leaves the path unmarked.</param>
        /// <returns>The same path, or a copy flagged outside.</returns>
        private static DubinsConnector.Path MarkOutside
            (
            DubinsConnector.Path Path,
            Double RadiusM,
            Func<Double, Double, Boolean>? Inside
            )
        {
            if ((Inside != null) && (OutsideLengthM(Path, RadiusM, Inside) > LENGTH_TIE_M))
            {
                return Path.AsOutside();
            }

            return Path;
        }

        /// <summary>
        /// Centreline length whose samples fall outside the field.
        /// </summary>
        /// <param name="Path">Connector to measure.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Inside">Field test. Null reports none.</param>
        /// <returns>Metres outside the field.</returns>
        private static Double OutsideLengthM
            (
            DubinsConnector.Path Path,
            Double RadiusM,
            Func<Double, Double, Boolean>? Inside
            )
        {
            if (Inside == null)
            {
                return 0.0;
            }

            Double Outside = 0.0;
            DubinsConnector.Pose Cursor = Path.Start;
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                if (Piece.LengthM <= 0.0)
                {
                    continue;
                }

                Int32 Count = Math.Max(1, (Int32)Math.Ceiling((Piece.LengthM / DubinsConnector.SAMPLE_M) - 1e-9));
                Double Step = Piece.LengthM / Count;
                for (Int32 Sample = 1; Sample <= Count; Sample += 1)
                {
                    DubinsConnector.Pose At = DubinsConnector.Advance(
                        Cursor, Piece.Kind, Step * Sample, RadiusM);
                    if (!Inside(At.X, At.Y))
                    {
                        Outside += Step;
                    }
                }

                Cursor = DubinsConnector.Advance(Cursor, Piece.Kind, Piece.LengthM, RadiusM);
            }

            return Outside;
        }

        /// <summary>
        /// Adds the forward S-bend between two bank lanes. Only LSR and RSL are
        /// offered, and only when the poses leave the Dubins room for that offset.
        /// A pose pair that cannot do that still gets a connector, so the trip draws.
        /// </summary>
        /// <param name="Segments">Segments being built.</param>
        /// <param name="Points">Guidance samples.</param>
        /// <param name="Notes">Connector notes.</param>
        /// <param name="Sequence">Next sample number.</param>
        /// <param name="LoadedSeconds">Loaded-connector seconds so far.</param>
        /// <param name="TurnSeconds">Turnaround seconds so far.</param>
        /// <param name="GradeSeconds">Grade seconds so far.</param>
        /// <param name="EmptySeconds">Empty-connector seconds so far.</param>
        /// <param name="Start">Pose the previous lane ended on.</param>
        /// <param name="End">Pose the next lane starts on.</param>
        /// <param name="Action">Carry between cuts or spreads. Grade on the way back.</param>
        /// <param name="Inside">Field test. Null does not test the field.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="TripNumber">1-based trip number.</param>
        /// <param name="BladeWidthM">Blade width, used to recognise the lane change.</param>
        private static void AddForwardBend
            (
            List<CourseSegment> Segments,
            List<GuidancePoint> Points,
            List<ConnectorNote> Notes,
            ref Int32 Sequence,
            ref Double LoadedSeconds,
            ref Double TurnSeconds,
            ref Double GradeSeconds,
            ref Double EmptySeconds,
            DubinsConnector.Pose Start,
            DubinsConnector.Pose End,
            GuidanceAction Action,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM,
            Int32 TripNumber,
            Double BladeWidthM
            )
        {
            if (!IsForwardLaneChange(Start, End, BladeWidthM, out Double LateralM))
            {
                AddConnector(
                    Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                    ref GradeSeconds, ref EmptySeconds, Start, End, Action, Inside, Clock, RadiusM, TripNumber);
                return;
            }

            Double Room = ForwardSBendM(LateralM, RadiusM);
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double Forward = (Dx * Math.Cos(Start.HeadingRad)) + (Dy * Math.Sin(Start.HeadingRad));
            List<DubinsConnector.Path> Words = DubinsConnector.AllWords(Start, End, RadiusM);
            List<DubinsConnector.Path> Bends = new List<DubinsConnector.Path>();
            foreach (DubinsConnector.Path Word in Words)
            {
                // LSR and RSL are the forward S. A U-turn word is not a lane change.
                if ((Word.Word != "LSR") && (Word.Word != "RSL"))
                {
                    continue;
                }

                if (Forward + 0.05 < Room)
                {
                    continue;
                }

                Bends.Add(Word);
            }

            Int32 Bend = (Bends.Count == 0) ? -1 : PreferField(Bends, RadiusM, Inside);
            // No in-field S-bend falls through to the ordinary connector.
            DubinsConnector.Path Path = (Bend < 0)
                ? SelectConnector(Start, End, RadiusM, Inside)
                : MarkOutside(Bends[Bend], RadiusM, Inside);
            CourseSegment? Built = SegmentFromPath(Start, End, Action, Path, Clock, RadiusM);
            if (Built == null)
            {
                return;
            }

            if (Action == GuidanceAction.Carry)
            {
                LoadedSeconds += Built.Seconds;
            }
            else if (Action == GuidanceAction.Turn)
            {
                TurnSeconds += Built.Seconds;
            }
            else if (Action == GuidanceAction.Grade)
            {
                GradeSeconds += Built.Seconds;
            }
            else
            {
                EmptySeconds += Built.Seconds;
            }

            Segments.Add(Built);
            Notes.Add(new ConnectorNote(
                TripNumber,
                Action,
                Built.Word,
                Built.LengthM,
                Built.LeadOutM,
                Built.LeadInM,
                Built.Outside));
            AppendPoints(Points, ref Sequence, Built);
        }

        /// <summary>
        /// Builds one connector and does not add it to a course. A wrapped full
        /// turn at the same point is an empty join, not a 2πR loop. The word is
        /// the near-shortest path with the least length outside the field.
        /// </summary>
        /// <param name="Start">Pose the connector leaves.</param>
        /// <param name="End">Pose the connector arrives at.</param>
        /// <param name="Action">What the connector is.</param>
        /// <param name="Inside">Field test. Null does not test the field.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>The connector, or null when the poses already meet.</returns>
        private static CourseSegment? MakeConnector
            (
            DubinsConnector.Pose Start,
            DubinsConnector.Pose End,
            GuidanceAction Action,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM
            )
        {
            if (DubinsConnector.IsEmptyJoin(Start, End))
            {
                return null;
            }

            // Loaded, empty, turnaround, and line-change connectors share this choice.
            DubinsConnector.Path Path = SelectConnector(Start, End, RadiusM, Inside);
            return SegmentFromPath(Start, End, Action, Path, Clock, RadiusM);
        }

        /// <summary>
        /// Draws a connector that has already been chosen.
        /// </summary>
        /// <param name="Start">Pose the connector leaves.</param>
        /// <param name="End">Pose the connector arrives at.</param>
        /// <param name="Action">What the connector is.</param>
        /// <param name="Path">The chosen Dubins path.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="RadiusM">Turning radius used to sample arcs.</param>
        /// <returns>The connector, or null when the path has no length.</returns>
        private static CourseSegment? SegmentFromPath
            (
            DubinsConnector.Pose Start,
            DubinsConnector.Pose End,
            GuidanceAction Action,
            DubinsConnector.Path Path,
            TimeParams Clock,
            Double RadiusM
            )
        {
            if (Path.LengthM <= 0.0)
            {
                return null;
            }

            List<DrawnPiece> Drawn = new List<DrawnPiece>();
            DubinsConnector.Pose Cursor = Start;
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                List<DubinsConnector.Pose> Samples = new List<DubinsConnector.Pose> { Cursor };
                Int32 Count = Math.Max(1, (Int32)Math.Ceiling((Piece.LengthM / DubinsConnector.SAMPLE_M) - 1e-9));
                for (Int32 Step = 1; Step <= Count; Step += 1)
                {
                    Double Distance = Piece.LengthM * Step / Count;
                    Samples.Add(DubinsConnector.Advance(Cursor, Piece.Kind, Distance, RadiusM));
                }

                DubinsConnector.Pose Arrow = DubinsConnector.Advance(
                    Cursor, Piece.Kind, Piece.LengthM * Constants.HALF_SCALE, RadiusM);
                Drawn.Add(new DrawnPiece(Piece.Kind, Piece.LengthM, Piece.CurvaturePerM, Samples, Arrow));
                Cursor = DubinsConnector.Advance(Cursor, Piece.Kind, Piece.LengthM, RadiusM);
            }

            if (Drawn.Count > 0)
            {
                List<DubinsConnector.Pose> LastSamples = Drawn[Drawn.Count - 1].Samples;
                DubinsConnector.Pose Arrival = LastSamples[LastSamples.Count - 1];
                if (NearPose(Arrival, End))
                {
                    LastSamples[LastSamples.Count - 1] = End;
                }
            }

            return new CourseSegment(
                Action,
                false,
                Path.Outside,
                Path.Word,
                Path.LeadOutM,
                Path.LeadInM,
                Path.LengthM,
                ConnectorSeconds(Path, Action, Clock),
                Drawn);
        }

        /// <summary>
        /// Seconds for a connector. Arcs use the turn speed. Straights use the
        /// loaded speed on a carry and the return speed everywhere else.
        /// </summary>
        /// <param name="Path">Connector to time.</param>
        /// <param name="Action">What the connector is.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <returns>Productive seconds, before field efficiency.</returns>
        private static Double ConnectorSeconds
            (
            DubinsConnector.Path Path,
            GuidanceAction Action,
            TimeParams Clock
            )
        {
            Double StraightMph = (Action == GuidanceAction.Carry)
                ? Clock.LoadedTravelSpeedMph
                : Clock.UnloadedTravelSpeedMph;
            Double Seconds = 0.0;
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                if (Piece.Kind == DubinsConnector.PieceKind.Straight)
                {
                    Seconds += TimeModel.MoveTime(
                        Piece.LengthM,
                        StraightMph * Constants.MPH_TO_MPS,
                        Clock.AccelerationTimeS,
                        Clock.DecelerationTimeS);
                }
                else
                {
                    Seconds += Piece.LengthM / (CurrentRules.ArcSpeedMph * Constants.MPH_TO_MPS);
                }
            }

            return Seconds;
        }

        /// <summary>
        /// Replaces a run of connectors that have no lane between them with one
        /// Dubins path from the first start pose to the last end pose.
        /// </summary>
        /// <param name="Segments">Segments in drive order.</param>
        /// <param name="Inside">Field test. Null does not test the field.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>The same list when nothing was stacked. Otherwise the collapsed list.</returns>
        private static List<CourseSegment> CollapseStacked
            (
            List<CourseSegment> Segments,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM
            )
        {
            Boolean Stacked = false;
            for (Int32 Index = 0; Index < (Segments.Count - 1); Index += 1)
            {
                if (!Segments[Index].IsLane && !Segments[Index + 1].IsLane)
                {
                    Stacked = true;
                    break;
                }
            }

            if (!Stacked)
            {
                return Segments;
            }

            List<CourseSegment> Result = new List<CourseSegment>(Segments.Count);
            Int32 Cursor = 0;
            while (Cursor < Segments.Count)
            {
                if (Segments[Cursor].IsLane)
                {
                    Result.Add(Segments[Cursor]);
                    Cursor += 1;
                    continue;
                }

                Int32 End = Cursor;
                while ((End < Segments.Count) && !Segments[End].IsLane)
                {
                    End += 1;
                }

                if ((End - Cursor) < 2)
                {
                    Result.Add(Segments[Cursor]);
                    Cursor = End;
                    continue;
                }

                // The whole run is one departure. Pairwise joins would keep the waypoint the stack is removing.
                if (!TrySegmentPose(Segments[Cursor], true, out DubinsConnector.Pose Start)
                    || !TrySegmentPose(Segments[End - 1], false, out DubinsConnector.Pose Finish))
                {
                    for (Int32 Keep = Cursor; Keep < End; Keep += 1)
                    {
                        Result.Add(Segments[Keep]);
                    }

                    Cursor = End;
                    continue;
                }

                CourseSegment? Built = MakeConnector(
                    Start,
                    Finish,
                    StackAction(Segments, Cursor, End),
                    Inside,
                    Clock,
                    RadiusM);
                if (Built != null)
                {
                    Result.Add(Built);
                }

                Cursor = End;
            }

            return Result;
        }

        /// <summary>
        /// Action charged for a collapsed stack. A return makes the whole path empty.
        /// A turnaround with no return stays a turnaround.
        /// </summary>
        /// <param name="Segments">Segments in drive order.</param>
        /// <param name="Start">First connector in the stack.</param>
        /// <param name="End">Index just past the stack.</param>
        /// <returns>The action to charge.</returns>
        private static GuidanceAction StackAction
            (
            List<CourseSegment> Segments,
            Int32 Start,
            Int32 End
            )
        {
            Boolean SawReturn = false;
            Boolean SawTurn = false;
            for (Int32 Index = Start; Index < End; Index += 1)
            {
                if (Segments[Index].Action == GuidanceAction.Return)
                {
                    SawReturn = true;
                }
                else if (Segments[Index].Action == GuidanceAction.Turn)
                {
                    SawTurn = true;
                }
            }

            if (SawReturn)
            {
                return GuidanceAction.Return;
            }

            if (SawTurn)
            {
                return GuidanceAction.Turn;
            }

            return Segments[Start].Action;
        }

        /// <summary>
        /// Start or end pose of a segment, taken from its samples.
        /// </summary>
        /// <param name="Segment">Segment to read.</param>
        /// <param name="AtStart">True for the first sample. False for the last.</param>
        /// <param name="Pose">The sample when one exists.</param>
        /// <returns>False when the segment has no samples.</returns>
        private static Boolean TrySegmentPose
            (
            CourseSegment Segment,
            Boolean AtStart,
            out DubinsConnector.Pose Pose
            )
        {
            Pose = new DubinsConnector.Pose(0.0, 0.0, 0.0);
            if (Segment.Pieces.Count == 0)
            {
                return false;
            }

            DrawnPiece Piece = AtStart ? Segment.Pieces[0] : Segment.Pieces[Segment.Pieces.Count - 1];
            if (Piece.Samples.Count == 0)
            {
                return false;
            }

            Pose = AtStart ? Piece.Samples[0] : Piece.Samples[Piece.Samples.Count - 1];
            return true;
        }

        /// <summary>
        /// Rebuilds sample numbers, notes, and the six clocks from the segments.
        /// Used after stacked connectors are replaced.
        /// </summary>
        /// <param name="Segments">Segments in drive order.</param>
        /// <param name="Points">Guidance samples. Cleared and rebuilt.</param>
        /// <param name="Notes">Connector notes. Cleared and rebuilt.</param>
        /// <param name="TripNumber">1-based trip number.</param>
        /// <param name="Sequence">Next sample number. Reset to 1.</param>
        /// <param name="CutSeconds">Cut seconds. Replaced.</param>
        /// <param name="LoadedSeconds">Loaded-connector seconds. Replaced.</param>
        /// <param name="SpreadSeconds">Spread seconds. Replaced.</param>
        /// <param name="TurnSeconds">Turnaround seconds. Replaced.</param>
        /// <param name="GradeSeconds">Grade seconds. Replaced.</param>
        /// <param name="EmptySeconds">Empty-connector seconds. Replaced.</param>
        private static void RetotalCourse
            (
            List<CourseSegment> Segments,
            List<GuidancePoint> Points,
            List<ConnectorNote> Notes,
            Int32 TripNumber,
            ref Int32 Sequence,
            ref Double CutSeconds,
            ref Double LoadedSeconds,
            ref Double SpreadSeconds,
            ref Double TurnSeconds,
            ref Double GradeSeconds,
            ref Double EmptySeconds
            )
        {
            Points.Clear();
            Notes.Clear();
            Sequence = 1;
            CutSeconds = 0.0;
            LoadedSeconds = 0.0;
            SpreadSeconds = 0.0;
            TurnSeconds = 0.0;
            GradeSeconds = 0.0;
            EmptySeconds = 0.0;
            foreach (CourseSegment Segment in Segments)
            {
                if (Segment.IsLane)
                {
                    if (Segment.Action == GuidanceAction.Cut)
                    {
                        CutSeconds += Segment.Seconds;
                    }
                    else if (Segment.Action == GuidanceAction.Fill)
                    {
                        SpreadSeconds += Segment.Seconds;
                    }
                    else
                    {
                        GradeSeconds += Segment.Seconds;
                    }
                }
                else
                {
                    if (Segment.Action == GuidanceAction.Carry)
                    {
                        LoadedSeconds += Segment.Seconds;
                    }
                    else if (Segment.Action == GuidanceAction.Turn)
                    {
                        TurnSeconds += Segment.Seconds;
                    }
                    else if (Segment.Action == GuidanceAction.Grade)
                    {
                        GradeSeconds += Segment.Seconds;
                    }
                    else
                    {
                        EmptySeconds += Segment.Seconds;
                    }

                    Notes.Add(new ConnectorNote(
                        TripNumber,
                        Segment.Action,
                        Segment.Word,
                        Segment.LengthM,
                        Segment.LeadOutM,
                        Segment.LeadInM,
                        Segment.Outside));
                }

                AppendPoints(Points, ref Sequence, Segment);
            }
        }

        /// <summary>
        /// Samples a lane at cell spacing. Heading follows the segment being driven.
        /// </summary>
        /// <param name="Lane">Lane to sample.</param>
        /// <param name="SpacingM">Maximum gap, metres.</param>
        /// <returns>Samples including both ends.</returns>
        private static List<DubinsConnector.Pose> SampleLine
            (
            WorkLane Lane,
            Double SpacingM
            )
        {
            List<DubinsConnector.Pose> Points = new List<DubinsConnector.Pose>();
            List<(Double X, Double Y)> Line = Lane.Centerline;
            if (Line.Count == 0)
            {
                return Points;
            }

            if (Line.Count == 1)
            {
                Points.Add(Lane.Start);
                return Points;
            }

            Double Heading = Lane.Start.HeadingRad;
            Points.Add(new DubinsConnector.Pose(Line[0].X, Line[0].Y, Heading));
            for (Int32 Index = 1; Index < Line.Count; Index += 1)
            {
                Double Dx = Line[Index].X - Line[Index - 1].X;
                Double Dy = Line[Index].Y - Line[Index - 1].Y;
                Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Length > 1e-6)
                {
                    Heading = Math.Atan2(Dy, Dx);
                }

                Int32 Count = Math.Max(1, (Int32)Math.Ceiling((Length / SpacingM) - 1e-9));
                for (Int32 Step = 1; Step <= Count; Step += 1)
                {
                    Double Fraction = (Double)Step / Count;
                    Points.Add(new DubinsConnector.Pose(
                        Line[Index - 1].X + (Dx * Fraction),
                        Line[Index - 1].Y + (Dy * Fraction),
                        Heading));
                }
            }

            return Points;
        }

        /// <summary>
        /// Appends a segment's samples. The joint with the previous segment is
        /// already in the list, so it is not repeated.
        /// </summary>
        private static void AppendPoints
            (
            List<GuidancePoint> Points,
            ref Int32 Sequence,
            CourseSegment Segment
            )
        {
            // A grade connector between fragments is blade-up. Only the lane itself grades.
            Boolean Blade = Segment.IsLane
                && ((Segment.Action == GuidanceAction.Cut)
                    || (Segment.Action == GuidanceAction.Fill)
                    || (Segment.Action == GuidanceAction.Grade));
            foreach (DrawnPiece Piece in Segment.Pieces)
            {
                foreach (DubinsConnector.Pose Sample in Piece.Samples)
                {
                    if (Points.Count > 0)
                    {
                        GuidancePoint Previous = Points[Points.Count - 1];
                        Double Dx = Sample.X - Previous.X;
                        Double Dy = Sample.Y - Previous.Y;
                        if (((Dx * Dx) + (Dy * Dy)) <= (JOINT_M * JOINT_M))
                        {
                            continue;
                        }
                    }

                    Points.Add(new GuidancePoint(
                        Sequence,
                        Sample.X,
                        Sample.Y,
                        DubinsConnector.WrapPi(Sample.HeadingRad),
                        Segment.Action,
                        Blade,
                        Segment.Outside));
                    Sequence += 1;
                }
            }
        }

        /// <summary>
        /// Drives a side of the trip. A correcting gap on the same line is a
        /// straight with the blade up. Every other gap is a Dubins connector.
        /// A correcting line change takes the faster of the in-field path and the open path.
        /// </summary>
        /// <param name="Segments">Segments being built.</param>
        /// <param name="Points">Guidance samples.</param>
        /// <param name="Notes">Connector notes.</param>
        /// <param name="Sequence">Next sample number.</param>
        /// <param name="CutSeconds">Cut seconds so far.</param>
        /// <param name="LoadedSeconds">Loaded-connector seconds so far.</param>
        /// <param name="SpreadSeconds">Spread seconds so far.</param>
        /// <param name="TurnSeconds">Turnaround seconds so far.</param>
        /// <param name="GradeSeconds">Grade seconds so far.</param>
        /// <param name="EmptySeconds">Empty-connector seconds so far.</param>
        /// <param name="Cursor">Pose the previous lane ended on. Null before the first lane.</param>
        /// <param name="Lanes">Lanes in drive order.</param>
        /// <param name="LineIds">Line id of each lane. Null means each lane is its own line.</param>
        /// <param name="LaneAction">Action while the blade is down on the lane.</param>
        /// <param name="JoinAction">Action on the gap before a lane.</param>
        /// <param name="StraightOnLine">True when a shared line uses a straight gap.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="LaneSampleM">Lane sample spacing.</param>
        /// <param name="TripNumber">1-based trip number.</param>
        /// <param name="BladeWidthM">Blade width. Zero does not steer a short joint straight.</param>
        /// <param name="RoadBank">True on the road stage. A lane change is an S-bend, not a straight steer.</param>
        /// <returns>The pose at the end of the last lane, or <paramref name="Cursor"/> when there are none.</returns>
        private static DubinsConnector.Pose? DriveSide
            (
            List<CourseSegment> Segments,
            List<GuidancePoint> Points,
            List<ConnectorNote> Notes,
            ref Int32 Sequence,
            ref Double CutSeconds,
            ref Double LoadedSeconds,
            ref Double SpreadSeconds,
            ref Double TurnSeconds,
            ref Double GradeSeconds,
            ref Double EmptySeconds,
            DubinsConnector.Pose? Cursor,
            IReadOnlyList<WorkLane> Lanes,
            IReadOnlyList<Int32>? LineIds,
            GuidanceAction LaneAction,
            GuidanceAction JoinAction,
            Boolean StraightOnLine,
            Func<Double, Double, Boolean>? Inside,
            TimeParams Clock,
            Double RadiusM,
            Double LaneSampleM,
            Int32 TripNumber,
            Double BladeWidthM,
            Boolean RoadBank
            )
        {
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                WorkLane Lane = Lanes[Index];
                if (Cursor.HasValue)
                {
                    Boolean SameLine = StraightOnLine
                        && (LineIds != null)
                        && (Index > 0)
                        && (LineIds[Index] == LineIds[Index - 1]);
                    // A road lane change keeps the S-bend. A steer would cut the corner across the bank.
                    Boolean LaneChange = RoadBank
                        && IsForwardLaneChange(Cursor.Value, Lane.Start, BladeWidthM, out Double _);
                    Boolean Steer = (!SameLine) && !LaneChange && SteerAcross(Cursor.Value, Lane.Start, RadiusM, BladeWidthM);
                    if (LaneChange)
                    {
                        AddForwardBend(
                            Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                            ref GradeSeconds, ref EmptySeconds, Cursor.Value, Lane.Start,
                            JoinAction, Inside, Clock, RadiusM, TripNumber, BladeWidthM);
                    }
                    else if (SameLine || Steer)
                    {
                        AddStraight(
                            Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                            ref GradeSeconds, ref EmptySeconds, Cursor.Value, Lane.Start,
                            JoinAction, Clock, TripNumber);
                    }
                    else
                    {
                        AddConnector(
                            Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                            ref GradeSeconds, ref EmptySeconds, Cursor.Value, Lane.Start,
                            JoinAction, Inside, Clock, RadiusM, TripNumber);
                    }
                }

                AddLane(
                    Segments, Points, ref Sequence, ref CutSeconds, ref SpreadSeconds, ref GradeSeconds,
                    Lane, LaneAction, LaneSampleM, Clock);
                Cursor = Lane.End;
            }

            return Cursor;
        }

        /// <summary>
        /// Adds a straight blade-up gap. Used between correcting fragments that
        /// already share a line, so the machine does not turn around in the gap.
        /// </summary>
        /// <param name="Segments">Segments being built.</param>
        /// <param name="Points">Guidance samples.</param>
        /// <param name="Notes">Connector notes.</param>
        /// <param name="Sequence">Next sample number.</param>
        /// <param name="LoadedSeconds">Loaded-connector seconds so far.</param>
        /// <param name="TurnSeconds">Turnaround seconds so far.</param>
        /// <param name="GradeSeconds">Grade seconds so far.</param>
        /// <param name="EmptySeconds">Empty-connector seconds so far.</param>
        /// <param name="Start">Pose the previous fragment ended on.</param>
        /// <param name="End">Pose the next fragment starts on.</param>
        /// <param name="Action">Carry between cuts or spreads. Grade on the way back.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="TripNumber">1-based trip number.</param>
        private static void AddStraight
            (
            List<CourseSegment> Segments,
            List<GuidancePoint> Points,
            List<ConnectorNote> Notes,
            ref Int32 Sequence,
            ref Double LoadedSeconds,
            ref Double TurnSeconds,
            ref Double GradeSeconds,
            ref Double EmptySeconds,
            DubinsConnector.Pose Start,
            DubinsConnector.Pose End,
            GuidanceAction Action,
            TimeParams Clock,
            Int32 TripNumber
            )
        {
            if (DubinsConnector.IsEmptyJoin(Start, End))
            {
                return;
            }

            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Length <= 1e-6)
            {
                return;
            }

            // The chord is the path. Heading stays on that chord, so the gap is not a turn.
            Double Heading = Math.Atan2(Dy, Dx);
            List<DubinsConnector.Pose> Samples = new List<DubinsConnector.Pose>
            {
                new DubinsConnector.Pose(Start.X, Start.Y, Heading),
            };
            Int32 Count = Math.Max(1, (Int32)Math.Ceiling((Length / DubinsConnector.SAMPLE_M) - 1e-9));
            for (Int32 Step = 1; Step <= Count; Step += 1)
            {
                Double Fraction = (Double)Step / Count;
                Samples.Add(new DubinsConnector.Pose(
                    Start.X + (Dx * Fraction),
                    Start.Y + (Dy * Fraction),
                    Heading));
            }

            DubinsConnector.Pose Arrow = new DubinsConnector.Pose(
                Start.X + (Dx * Constants.HALF_SCALE),
                Start.Y + (Dy * Constants.HALF_SCALE),
                Heading);
            List<DrawnPiece> Pieces = new List<DrawnPiece>
            {
                new DrawnPiece(DubinsConnector.PieceKind.Straight, Length, 0.0, Samples, Arrow),
            };
            Double Seconds = 0.0;
            if (Action == GuidanceAction.Grade)
            {
                Double Speed = Constants.DEFAULT_FILL_SPEED_MPH * Constants.MPH_TO_MPS;
                if (Speed > 0.0)
                {
                    Seconds = Length / Speed;
                }
            }
            else
            {
                Double StraightMph = (Action == GuidanceAction.Carry)
                    ? Clock.LoadedTravelSpeedMph
                    : Clock.UnloadedTravelSpeedMph;
                Seconds = TimeModel.MoveTime(
                    Length,
                    StraightMph * Constants.MPH_TO_MPS,
                    Clock.AccelerationTimeS,
                    Clock.DecelerationTimeS);
            }

            if (Action == GuidanceAction.Carry)
            {
                LoadedSeconds += Seconds;
            }
            else if (Action == GuidanceAction.Turn)
            {
                TurnSeconds += Seconds;
            }
            else if (Action == GuidanceAction.Grade)
            {
                GradeSeconds += Seconds;
            }
            else
            {
                EmptySeconds += Seconds;
            }

            CourseSegment Segment = new CourseSegment(
                Action, false, false, "S", 0.0, 0.0, Length, Seconds, Pieces);
            Segments.Add(Segment);
            Notes.Add(new ConnectorNote(TripNumber, Action, "S", Length, 0.0, 0.0, false));
            AppendPoints(Points, ref Sequence, Segment);
        }

        /// <summary>
        /// True when the course drives a grade lane. A skipped grade pass has none.
        /// </summary>
        /// <param name="Trip">One trip.</param>
        /// <returns>True when a grade lane is present.</returns>
        private static Boolean HasGradeLane
            (
            TripCourse Trip
            )
        {
            foreach (CourseSegment Segment in Trip.Segments)
            {
                if (Segment.IsLane && (Segment.Action == GuidanceAction.Grade))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Deepest spread cell on the lanes, including cells a bend set aside.
        /// </summary>
        /// <param name="Lanes">Spread lanes.</param>
        /// <returns>Depth in metres. Zero when the lanes carry no cells.</returns>
        private static Double MaxSpreadDepth
            (
            IReadOnlyList<WorkLane> Lanes
            )
        {
            Double Max = 0.0;
            foreach (WorkLane Lane in Lanes)
            {
                Max = Deeper(Max, Lane.Cells);
                Max = Deeper(Max, Lane.Deferred);
            }

            return Max;
        }

        /// <summary>
        /// Keeps the larger of the running depth and the listed cells.
        /// </summary>
        /// <param name="Max">Deepest so far, metres.</param>
        /// <param name="Cells">Cells to inspect.</param>
        /// <returns>The deeper depth.</returns>
        private static Double Deeper
            (
            Double Max,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            if (Cells == null)
            {
                return Max;
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                if (Entry.DepthM > Max)
                {
                    Max = Entry.DepthM;
                }
            }

            return Max;
        }

        /// <summary>
        /// Groups fragments that share a line, sorts each line by position, and
        /// drives every fragment on that line in the same direction.
        /// </summary>
        /// <param name="Lanes">Fragments in the planner's order.</param>
        /// <param name="LineIds">Line id of each returned lane. Equal ids are one line.</param>
        /// <returns>Lanes in drive order.</returns>
        private static List<WorkLane> ArrangeOnLines
            (
            IReadOnlyList<WorkLane> Lanes,
            out List<Int32> LineIds
            )
        {
            List<List<Int32>> Groups = new List<List<Int32>>();
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                Int32 Found = -1;
                for (Int32 Group = 0; Group < Groups.Count; Group += 1)
                {
                    if (SharesLine(Lanes[Index], Lanes[Groups[Group][0]]))
                    {
                        Found = Group;
                        break;
                    }
                }

                if (Found < 0)
                {
                    Groups.Add(new List<Int32> { Index });
                }
                else
                {
                    Groups[Found].Add(Index);
                }
            }

            List<WorkLane> Ordered = new List<WorkLane>();
            LineIds = new List<Int32>();
            for (Int32 Group = 0; Group < Groups.Count; Group += 1)
            {
                // Finish one line before the next, so the only turn is at the line's end.
                List<WorkLane> Oriented = OrientLine(Lanes, Groups[Group]);
                for (Int32 Index = 0; Index < Oriented.Count; Index += 1)
                {
                    Ordered.Add(Oriented[Index]);
                    LineIds.Add(Group);
                }
            }

            return Ordered;
        }

        /// <summary>
        /// Points every fragment along the line and orders them from the low end.
        /// </summary>
        /// <param name="Lanes">All fragments.</param>
        /// <param name="Indexes">Indexes that belong to this line, in discovery order.</param>
        /// <returns>The line in drive order.</returns>
        private static List<WorkLane> OrientLine
            (
            IReadOnlyList<WorkLane> Lanes,
            IReadOnlyList<Int32> Indexes
            )
        {
            WorkLane Reference = Lanes[Indexes[0]];
            LineAxis(Reference, out Double OriginX, out Double OriginY, out Double Cos, out Double Sin, out Double _);
            List<(Double Station, Int32 Order, WorkLane Lane)> Items =
                new List<(Double Station, Int32 Order, WorkLane Lane)>(Indexes.Count);
            for (Int32 Order = 0; Order < Indexes.Count; Order += 1)
            {
                WorkLane Lane = Lanes[Indexes[Order]];
                Double StartS = ((Lane.Start.X - OriginX) * Cos) + ((Lane.Start.Y - OriginY) * Sin);
                Double EndS = ((Lane.End.X - OriginX) * Cos) + ((Lane.End.Y - OriginY) * Sin);
                if ((EndS + LINE_POINT_M) < StartS)
                {
                    // This fragment faces the other way down the same line. Turn it around once.
                    Lane = Lane.Reversed();
                    StartS = ((Lane.Start.X - OriginX) * Cos) + ((Lane.Start.Y - OriginY) * Sin);
                }

                Items.Add((StartS, Order, Lane));
            }

            Items.Sort(CompareStation);
            List<WorkLane> Ordered = new List<WorkLane>(Items.Count);
            foreach ((Double Station, Int32 Order, WorkLane Lane) Item in Items)
            {
                Ordered.Add(Item.Lane);
            }

            return Ordered;
        }

        /// <summary>
        /// Orders a line from the low station. Equal stations keep discovery order.
        /// </summary>
        /// <param name="Left">One fragment.</param>
        /// <param name="Right">The other fragment.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareStation
            (
            (Double Station, Int32 Order, WorkLane Lane) Left,
            (Double Station, Int32 Order, WorkLane Lane) Right
            )
        {
            Int32 ByStation = Left.Station.CompareTo(Right.Station);
            if (ByStation != 0)
            {
                return ByStation;
            }

            return Left.Order.CompareTo(Right.Order);
        }

        /// <summary>
        /// True when <paramref name="Candidate"/> lies on the same centre line as
        /// <paramref name="AxisLane"/>, facing either way along it.
        /// </summary>
        /// <param name="Candidate">Fragment being grouped.</param>
        /// <param name="AxisLane">The line's first fragment.</param>
        /// <returns>True when both ends sit in the band and the headings are parallel.</returns>
        private static Boolean SharesLine
            (
            WorkLane Candidate,
            WorkLane AxisLane
            )
        {
            LineAxis(AxisLane, out Double OriginX, out Double OriginY, out Double Cos, out Double Sin, out Double AxisHeading);
            LineAxis(Candidate, out Double _, out Double _, out Double _, out Double _, out Double CandidateHeading);
            Double Delta = Math.Abs(DubinsConnector.WrapPi(CandidateHeading - AxisHeading));
            Double Limit = LINE_ANGLE_DEG * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            Boolean Parallel = (Delta <= Limit) || ((Math.PI - Delta) <= Limit);
            if (!Parallel)
            {
                return false;
            }

            Double StartLateral = Math.Abs(((Cos * (Candidate.Start.Y - OriginY)) - (Sin * (Candidate.Start.X - OriginX))));
            Double EndLateral = Math.Abs(((Cos * (Candidate.End.Y - OriginY)) - (Sin * (Candidate.End.X - OriginX))));
            return (StartLateral <= LINE_LATERAL_M) && (EndLateral <= LINE_LATERAL_M);
        }

        /// <summary>
        /// Chord of a lane, or its start heading when the lane is only a point.
        /// </summary>
        /// <param name="Lane">Lane being measured.</param>
        /// <param name="OriginX">Easting of the start.</param>
        /// <param name="OriginY">Northing of the start.</param>
        /// <param name="Cos">East component of the unit axis.</param>
        /// <param name="Sin">North component of the unit axis.</param>
        /// <param name="Heading">Axis heading, radians.</param>
        private static void LineAxis
            (
            WorkLane Lane,
            out Double OriginX,
            out Double OriginY,
            out Double Cos,
            out Double Sin,
            out Double Heading
            )
        {
            OriginX = Lane.Start.X;
            OriginY = Lane.Start.Y;
            Double Dx = Lane.End.X - Lane.Start.X;
            Double Dy = Lane.End.Y - Lane.Start.Y;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Length < LINE_POINT_M)
            {
                Heading = Lane.Start.HeadingRad;
            }
            else
            {
                Heading = Math.Atan2(Dy, Dx);
            }

            Cos = Math.Cos(Heading);
            Sin = Math.Sin(Heading);
        }

        /// <summary>
        /// Indexes whose line id appears once. Those lanes may still reverse or bend.
        /// </summary>
        /// <param name="LineIds">Line id of each lane.</param>
        /// <returns>Indexes of lines that have a single fragment.</returns>
        private static List<Int32> SoloIndexes
            (
            IReadOnlyList<Int32> LineIds
            )
        {
            Dictionary<Int32, Int32> Count = new Dictionary<Int32, Int32>();
            for (Int32 Index = 0; Index < LineIds.Count; Index += 1)
            {
                Int32 Id = LineIds[Index];
                if (!Count.ContainsKey(Id))
                {
                    Count[Id] = 0;
                }

                Count[Id] += 1;
            }

            List<Int32> Solo = new List<Int32>();
            for (Int32 Index = 0; Index < LineIds.Count; Index += 1)
            {
                if (Count[LineIds[Index]] == 1)
                {
                    Solo.Add(Index);
                }
            }

            return Solo;
        }

        /// <summary>
        /// Line ids that give every lane a line of its own.
        /// </summary>
        /// <param name="Count">How many lanes.</param>
        /// <returns>0, 1, 2, ...</returns>
        private static List<Int32> IdentityLines
            (
            Int32 Count
            )
        {
            List<Int32> Ids = new List<Int32>(Count);
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                Ids.Add(Index);
            }

            return Ids;
        }

        /// <summary>
        /// Orders trips to shorten empty returns, inside each phase. Flow,
        /// clusters, and thin-cut stay before the road stage. Correcting follows
        /// the road, sweep-up follows correcting, and finishing stays last. Two trips that share a cut or
        /// fill cell keep the planner's relative order. The length is the
        /// Dubins length with no field mask. The driven return still uses the
        /// mask.
        /// </summary>
        /// <param name="Courses">Courses in planner order, used for the departure pose.</param>
        /// <param name="Cuts">Chosen cut lanes, for each trip's cut start.</param>
        /// <param name="Fills">Chosen spread lanes, used when a trip has no cut.</param>
        /// <param name="Passes">Trips whose cells decide which pairs may swap.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>Indexes into <paramref name="Passes"/> in drive order.</returns>
        private static Int32[] OrderTrips
            (
            TripCourse[] Courses,
            List<WorkLane>[] Cuts,
            List<WorkLane>[] Fills,
            IReadOnlyList<Pass> Passes,
            Double RadiusM
            )
        {
            Int32 Count = Courses.Length;
            Int32[] Order = new Int32[Count];
            if (Count == 0)
            {
                return Order;
            }

            DubinsConnector.Pose[] Depart = new DubinsConnector.Pose[Count];
            DubinsConnector.Pose[] Arrive = new DubinsConnector.Pose[Count];
            Boolean[] HasArrive = new Boolean[Count];
            HashSet<(Int32 Bx, Int32 By)>[] Cells = new HashSet<(Int32 Bx, Int32 By)>[Count];
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                Depart[Index] = DepartureOf(Courses[Index]);
                HasArrive[Index] = TryArrival(Cuts[Index], Fills[Index], out Arrive[Index]);
                Cells[Index] = CellsOf(Passes[Index]);
            }

            // Flow, clusters, and thin-cut share one phase and may interleave.
            // The road stage follows, then correcting, then sweep-up, then finishing.
            HaulStage[] Stages = new HaulStage[]
            {
                HaulStage.Flow,
                HaulStage.Clusters,
                HaulStage.ThinCut,
                HaulStage.Correcting,
                HaulStage.Road,
            };
            Boolean[] Used = new Boolean[Count];
            Int32 Slot = 0;
            DubinsConnector.Pose? Incoming = null;
            // Band 0 is flow. Band 1 is the road stage. Band 2 is correcting.
            // Band 3 is sweep-up. Band 4 is finishing.
            for (Int32 Band = 0; Band < 5; Band += 1)
            {
            foreach (HaulStage Stage in Stages)
            {
                if ((Band == 0)
                    && (Stage != HaulStage.Flow)
                    && (Stage != HaulStage.Clusters)
                    && (Stage != HaulStage.ThinCut))
                {
                    continue;
                }

                if ((Band == 1) && (Stage != HaulStage.Road))
                {
                    continue;
                }

                if ((Band == 2) && (Stage != HaulStage.Correcting))
                {
                    continue;
                }

                if ((Band == 3) && (Stage != HaulStage.Correcting))
                {
                    continue;
                }

                List<Int32> Members = new List<Int32>();
                for (Int32 Index = 0; Index < Count; Index += 1)
                {
                    if (Courses[Index].Stage != Stage)
                    {
                        continue;
                    }

                    if (Band == 4)
                    {
                        if (!Courses[Index].Finishing)
                        {
                            continue;
                        }
                    }
                    else if (Courses[Index].Finishing || (Courses[Index].SweepUp != (Band == 3)))
                    {
                        // Finishing waits until sweep-up is done.
                        // Sweep-up stays after the correcting pass that deferred it.
                        // Road trips are neither sweep-up nor finishing.
                        continue;
                    }

                    Members.Add(Index);
                }

                if (Members.Count == 0)
                {
                    continue;
                }

                List<Int32>[] Blockers = BlockersFor(Members, Cells);
                // The job starts on the planner's first trip. Later stages may open on any ready trip.
                Boolean HoldFirst = !Incoming.HasValue;
                Int32 Placed = 0;
                while (Placed < Members.Count)
                {
                    Int32 Best = -1;
                    Double BestLength = 0.0;
                    foreach (Int32 Candidate in Members)
                    {
                        if (Used[Candidate] || !Ready(Candidate, Used, Blockers))
                        {
                            continue;
                        }

                        if (HoldFirst && (Candidate != Members[0]))
                        {
                            continue;
                        }

                        Double Length = 0.0;
                        if (Incoming.HasValue && HasArrive[Candidate])
                        {
                            Length = EmptyLinkM(Incoming.Value, Arrive[Candidate], RadiusM);
                        }

                        Boolean Closer = (Best >= 0) && ((Length + LENGTH_TIE_M) < BestLength);
                        Boolean EarlierTie = (Best >= 0)
                            && (Math.Abs(Length - BestLength) <= LENGTH_TIE_M)
                            && (Candidate < Best);
                        if ((Best < 0) || Closer || EarlierTie)
                        {
                            Best = Candidate;
                            BestLength = Length;
                        }
                    }

                    if (Best < 0)
                    {
                        // A blocker cycle would stall. Take the earliest remaining trip.
                        foreach (Int32 Candidate in Members)
                        {
                            if (!Used[Candidate])
                            {
                                Best = Candidate;
                                break;
                            }
                        }
                    }

                    Order[Slot] = Best;
                    Used[Best] = true;
                    Slot += 1;
                    Placed += 1;
                    HoldFirst = false;
                    Incoming = Depart[Best];
                }
            }
            }

            ImproveOrder(Order, Courses, Depart, Arrive, HasArrive, Cells, RadiusM);
            return Order;
        }

        /// <summary>
        /// Swaps consecutive trips while the swap shortens the empty returns.
        /// A pair in different stages, or a pair that shares a cell, stays put.
        /// </summary>
        /// <param name="Order">Drive order. Updated in place.</param>
        /// <param name="Courses">Courses, for the stage of each trip.</param>
        /// <param name="Depart">Pose each trip leaves from.</param>
        /// <param name="Arrive">Cut start of each trip.</param>
        /// <param name="HasArrive">False when a trip has no cut or spread start.</param>
        /// <param name="Cells">Cut and fill cells of each trip.</param>
        /// <param name="RadiusM">Turning radius.</param>
        private static void ImproveOrder
            (
            Int32[] Order,
            TripCourse[] Courses,
            DubinsConnector.Pose[] Depart,
            DubinsConnector.Pose[] Arrive,
            Boolean[] HasArrive,
            HashSet<(Int32 Bx, Int32 By)>[] Cells,
            Double RadiusM
            )
        {
            Boolean Improved = true;
            Int32 Pass = 0;
            while (Improved && (Pass < MAX_ORDER_PASSES))
            {
                Improved = false;
                Pass += 1;
                for (Int32 Slot = 0; Slot < (Order.Length - 1); Slot += 1)
                {
                    if (Slot == 0)
                    {
                        // The machine is already at the planner's first trip.
                        continue;
                    }

                    Int32 Left = Order[Slot];
                    Int32 Right = Order[Slot + 1];
                    if (Courses[Left].Stage != Courses[Right].Stage)
                    {
                        continue;
                    }

                    if (Courses[Left].SweepUp != Courses[Right].SweepUp)
                    {
                        // A sweep-up trip stays after the correcting pass.
                        continue;
                    }

                    if (Courses[Left].Finishing != Courses[Right].Finishing)
                    {
                        // Finishing stays after sweep-up.
                        continue;
                    }

                    if (SharesCells(Cells[Left], Cells[Right]))
                    {
                        // Shared cells keep the order the greedy pass already respected.
                        continue;
                    }

                    Double Before = TouchingLength(Order, Slot, Depart, Arrive, HasArrive, RadiusM);
                    Order[Slot] = Right;
                    Order[Slot + 1] = Left;
                    Double After = TouchingLength(Order, Slot, Depart, Arrive, HasArrive, RadiusM);
                    if ((After + LENGTH_TIE_M) < Before)
                    {
                        Improved = true;
                    }
                    else
                    {
                        Order[Slot] = Left;
                        Order[Slot + 1] = Right;
                    }
                }
            }
        }

        /// <summary>
        /// Empty-return length of the links that a consecutive swap can change.
        /// </summary>
        /// <param name="Order">Drive order.</param>
        /// <param name="Slot">First trip of the pair.</param>
        /// <param name="Depart">Departure poses.</param>
        /// <param name="Arrive">Cut starts.</param>
        /// <param name="HasArrive">Which trips have a cut start.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>Sum of the neighbouring empty-return lengths.</returns>
        private static Double TouchingLength
            (
            Int32[] Order,
            Int32 Slot,
            DubinsConnector.Pose[] Depart,
            DubinsConnector.Pose[] Arrive,
            Boolean[] HasArrive,
            Double RadiusM
            )
        {
            Double Sum = EdgeLength(Order, Slot - 1, Slot, Depart, Arrive, HasArrive, RadiusM);
            Sum += EdgeLength(Order, Slot, Slot + 1, Depart, Arrive, HasArrive, RadiusM);
            Sum += EdgeLength(Order, Slot + 1, Slot + 2, Depart, Arrive, HasArrive, RadiusM);
            return Sum;
        }

        /// <summary>
        /// Dubins length from one trip's departure to the next trip's cut start.
        /// </summary>
        /// <param name="Order">Drive order.</param>
        /// <param name="FromSlot">Slot of the trip being left. Negative is the start of the job.</param>
        /// <param name="ToSlot">Slot of the trip being entered.</param>
        /// <param name="Depart">Departure poses.</param>
        /// <param name="Arrive">Cut starts.</param>
        /// <param name="HasArrive">Which trips have a cut start.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>Length in metres. Zero when the slot is off the ends of the route.</returns>
        private static Double EdgeLength
            (
            Int32[] Order,
            Int32 FromSlot,
            Int32 ToSlot,
            DubinsConnector.Pose[] Depart,
            DubinsConnector.Pose[] Arrive,
            Boolean[] HasArrive,
            Double RadiusM
            )
        {
            if ((FromSlot < 0) || (ToSlot >= Order.Length))
            {
                return 0.0;
            }

            Int32 To = Order[ToSlot];
            if (!HasArrive[To])
            {
                return 0.0;
            }

            return EmptyLinkM(Depart[Order[FromSlot]], Arrive[To], RadiusM);
        }

        /// <summary>
        /// Closed-form empty connector. The field mask is not applied.
        /// </summary>
        /// <param name="From">Departure pose.</param>
        /// <param name="To">Next cut start.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>Centreline length in metres.</returns>
        private static Double EmptyLinkM
            (
            DubinsConnector.Pose From,
            DubinsConnector.Pose To,
            Double RadiusM
            )
        {
            // Length only. The driven return is built later, with the field mask.
            return DubinsConnector.ShortestLengthM(From, To, RadiusM);
        }

        /// <summary>
        /// Pose the empty return leaves from: the end of the trip, before the return.
        /// </summary>
        /// <param name="Course">One trip.</param>
        /// <returns>The departure pose.</returns>
        private static DubinsConnector.Pose DepartureOf
            (
            TripCourse Course
            )
        {
            for (Int32 Index = Course.Segments.Count - 1; Index >= 0; Index -= 1)
            {
                CourseSegment Segment = Course.Segments[Index];
                if (Segment.Action == GuidanceAction.Return)
                {
                    continue;
                }

                if (Segment.Pieces.Count == 0)
                {
                    continue;
                }

                List<DubinsConnector.Pose> Samples = Segment.Pieces[Segment.Pieces.Count - 1].Samples;
                if (Samples.Count == 0)
                {
                    continue;
                }

                return Samples[Samples.Count - 1];
            }

            if (Course.Points.Count > 0)
            {
                GuidancePoint Last = Course.Points[Course.Points.Count - 1];
                return new DubinsConnector.Pose(Last.X, Last.Y, Last.HeadingRad);
            }

            return new DubinsConnector.Pose(0.0, 0.0, 0.0);
        }

        /// <summary>
        /// Cut start the empty return aims at. A trip with no cut uses its spread start.
        /// </summary>
        /// <param name="Cuts">Chosen cut lanes.</param>
        /// <param name="Fills">Chosen spread lanes.</param>
        /// <param name="Arrive">The start pose when one exists.</param>
        /// <returns>False when the trip has neither.</returns>
        private static Boolean TryArrival
            (
            List<WorkLane> Cuts,
            List<WorkLane> Fills,
            out DubinsConnector.Pose Arrive
            )
        {
            if ((Cuts != null) && (Cuts.Count > 0))
            {
                Arrive = Cuts[0].Start;
                return true;
            }

            if ((Fills != null) && (Fills.Count > 0))
            {
                Arrive = Fills[0].Start;
                return true;
            }

            Arrive = new DubinsConnector.Pose(0.0, 0.0, 0.0);
            return false;
        }

        /// <summary>
        /// Lists the 20 worst trips by connector ratio, loaded heading change, and turnarounds.
        /// </summary>
        /// <param name="Text">Report being built.</param>
        /// <param name="Trips">Trips in planner order.</param>
        private static void AppendDiagnostics
            (
            StringBuilder Text,
            IReadOnlyList<TripCourse> Trips
            )
        {
            Double[] Ratio = new Double[Trips.Count];
            Double[] Heading = new Double[Trips.Count];
            Double[] Turns = new Double[Trips.Count];
            Double[] ConnectorM = new Double[Trips.Count];
            Double[] ProductiveM = new Double[Trips.Count];
            for (Int32 Index = 0; Index < Trips.Count; Index += 1)
            {
                TripCourse Trip = Trips[Index];
                ConnectorM[Index] = ConnectorLengthM(Trip);
                ProductiveM[Index] = ProductiveLengthM(Trip);
                if (ProductiveM[Index] < 1e-6)
                {
                    Ratio[Index] = Double.PositiveInfinity;
                }
                else
                {
                    Ratio[Index] = ConnectorM[Index] / ProductiveM[Index];
                }

                Heading[Index] = LoadedHeadingChange(Trip);
                Turns[Index] = TurnaroundCount(Trip);
            }

            Text.AppendLine();
            Text.AppendLine("Worst trips by connector length / productive length");
            foreach (Int32 Index in WorstTrips(Trips, Ratio, WORST_COUNT))
            {
                String RatioText = Double.IsPositiveInfinity(Ratio[Index])
                    ? "inf"
                    : Ratio[Index].ToString("F3", CultureInfo.InvariantCulture);
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "trip {0} {1}  {2}  connectors {3:F1} m  productive {4:F1} m",
                    Trips[Index].TripNumber,
                    Trips[Index].Stage,
                    RatioText,
                    ConnectorM[Index],
                    ProductiveM[Index]));
            }

            Text.AppendLine();
            Text.AppendLine("Worst trips by loaded-connector heading change");
            foreach (Int32 Index in WorstTrips(Trips, Heading, WORST_COUNT))
            {
                String Flag = (Heading[Index] > (2.0 * Math.PI)) ? " FLAG" : "";
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "trip {0} {1}  {2:F3} rad{3}",
                    Trips[Index].TripNumber,
                    Trips[Index].Stage,
                    Heading[Index],
                    Flag));
            }

            Text.AppendLine();
            Text.AppendLine("Worst trips by turnaround count");
            foreach (Int32 Index in WorstTrips(Trips, Turns, WORST_COUNT))
            {
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "trip {0} {1}  {2:F0}",
                    Trips[Index].TripNumber,
                    Trips[Index].Stage,
                    Turns[Index]));
            }
        }

        /// <summary>
        /// Highest scores first. An equal score keeps the earlier trip number.
        /// </summary>
        /// <param name="Trips">Trips the scores belong to.</param>
        /// <param name="Scores">One score per trip.</param>
        /// <param name="Count">How many to keep.</param>
        /// <returns>Indexes into <paramref name="Trips"/>.</returns>
        private static List<Int32> WorstTrips
            (
            IReadOnlyList<TripCourse> Trips,
            Double[] Scores,
            Int32 Count
            )
        {
            List<Int32> Picked = new List<Int32>();
            Boolean[] Used = new Boolean[Trips.Count];
            Int32 Take = Math.Min(Count, Trips.Count);
            for (Int32 Rank = 0; Rank < Take; Rank += 1)
            {
                Int32 Best = -1;
                for (Int32 Index = 0; Index < Trips.Count; Index += 1)
                {
                    if (Used[Index])
                    {
                        continue;
                    }

                    if (Best < 0)
                    {
                        Best = Index;
                        continue;
                    }

                    if (Scores[Index] > Scores[Best])
                    {
                        Best = Index;
                        continue;
                    }

                    if ((Scores[Index] == Scores[Best]) && (Trips[Index].TripNumber < Trips[Best].TripNumber))
                    {
                        Best = Index;
                    }
                }

                if (Best < 0)
                {
                    break;
                }

                Used[Best] = true;
                Picked.Add(Best);
            }

            return Picked;
        }

        /// <summary>
        /// Length of every connector on the trip, lanes excluded.
        /// </summary>
        /// <param name="Trip">One trip.</param>
        /// <returns>Metres.</returns>
        private static Double ConnectorLengthM
            (
            TripCourse Trip
            )
        {
            Double Length = 0.0;
            foreach (CourseSegment Segment in Trip.Segments)
            {
                if (!Segment.IsLane)
                {
                    Length += Segment.LengthM;
                }
            }

            return Length;
        }

        /// <summary>
        /// Cut length plus spread length. Grade lanes are not included.
        /// </summary>
        /// <param name="Trip">One trip.</param>
        /// <returns>Metres.</returns>
        private static Double ProductiveLengthM
            (
            TripCourse Trip
            )
        {
            Double Length = 0.0;
            foreach (CourseSegment Segment in Trip.Segments)
            {
                if (!Segment.IsLane)
                {
                    continue;
                }

                if ((Segment.Action == GuidanceAction.Cut) || (Segment.Action == GuidanceAction.Fill))
                {
                    Length += Segment.LengthM;
                }
            }

            return Length;
        }

        /// <summary>
        /// Absolute heading change along loaded connectors, radians.
        /// </summary>
        /// <param name="Trip">One trip.</param>
        /// <returns>Radians. A straight adds nothing.</returns>
        private static Double LoadedHeadingChange
            (
            TripCourse Trip
            )
        {
            Double Change = 0.0;
            foreach (CourseSegment Segment in Trip.Segments)
            {
                if (Segment.IsLane || (Segment.Action != GuidanceAction.Carry))
                {
                    continue;
                }

                foreach (DrawnPiece Piece in Segment.Pieces)
                {
                    if (Piece.Kind != DubinsConnector.PieceKind.Straight)
                    {
                        Change += Piece.LengthM * Piece.CurvaturePerM;
                    }
                }
            }

            return Change;
        }

        /// <summary>
        /// How many turnaround segments the trip drives.
        /// </summary>
        /// <param name="Trip">One trip.</param>
        /// <returns>The count.</returns>
        private static Double TurnaroundCount
            (
            TripCourse Trip
            )
        {
            Double Count = 0.0;
            foreach (CourseSegment Segment in Trip.Segments)
            {
                if (Segment.Action == GuidanceAction.Turn)
                {
                    Count += 1.0;
                }
            }

            return Count;
        }

        /// <summary>
        /// Plans every trip in planner order. Later trips are planned first so
        /// each return aims at the next trip's chosen cut.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Passes">Trips in planner order.</param>
        /// <param name="Origin">Original 0-based index of each trip.</param>
        /// <param name="Clock">Speeds and acceleration.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="CircleFt">Turning diameter in feet, for the lane builder.</param>
        /// <param name="BowlCapacityM3">Bowl capacity. Used to ignore a light lane.</param>
        /// <param name="DeferEnds">True defers a short isolated end fragment.</param>
        /// <param name="Courses">Courses aimed at the planner's next trip.</param>
        /// <param name="ChosenCuts">Cut lanes after the direction search.</param>
        /// <param name="ChosenFills">Spread lanes after the direction search.</param>
        /// <param name="CutLines">Line id of each cut lane.</param>
        /// <param name="FillLines">Line id of each spread lane.</param>
        /// <param name="HeldCuts">Cut cells past the three-line cap.</param>
        /// <param name="HeldFills">Spread cells past the three-line cap.</param>
        /// <param name="EndCutFragments">Deferred cut end fragments, one list per trip.</param>
        /// <param name="EndFillFragments">Deferred spread end fragments, one list per trip.</param>
        private static void PlanCourses
            (
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            Int32[] Origin,
            TimeParams Clock,
            Double BladeWidthM,
            Double RadiusM,
            Double CircleFt,
            Double BowlCapacityM3,
            Boolean DeferEnds,
            out TripCourse[] Courses,
            out List<WorkLane>[] ChosenCuts,
            out List<WorkLane>[] ChosenFills,
            out List<Int32>[] CutLines,
            out List<Int32>[] FillLines,
            out List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] HeldCuts,
            out List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] HeldFills,
            out List<WorkLane>[] EndCutFragments,
            out List<WorkLane>[] EndFillFragments
            )
        {
            Int32 Count = Passes.Count;
            List<WorkLane>[] PlannedCuts = new List<WorkLane>[Count];
            List<WorkLane>[] PlannedFills = new List<WorkLane>[Count];
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                Pass Current = Passes[Index];
                if (Current.Stage == HaulStage.Road)
                {
                    // Ribbon order would fold a lane change into one run. The bank needs one lane each.
                    PlannedCuts[Index] = RoadLanesOf(Current.CutCells, Grid, BladeWidthM);
                    PlannedFills[Index] = RoadLanesOf(Current.FillCells, Grid, BladeWidthM);
                }
                else
                {
                    PlannedCuts[Index] = LanesOf(Current.CutCells, Current.RibbonOrder, Grid, BladeWidthM, CircleFt);
                    PlannedFills[Index] = LanesOf(Current.FillCells, Current.RibbonOrder, Grid, BladeWidthM, CircleFt);
                }
            }

            Courses = new TripCourse[Count];
            ChosenCuts = new List<WorkLane>[Count];
            ChosenFills = new List<WorkLane>[Count];
            CutLines = new List<Int32>[Count];
            FillLines = new List<Int32>[Count];
            HeldCuts = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[Count];
            HeldFills = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[Count];
            EndCutFragments = new List<WorkLane>[Count];
            EndFillFragments = new List<WorkLane>[Count];
            for (Int32 Index = Count - 1; Index >= 0; Index -= 1)
            {
                DubinsConnector.Pose? NextCut = null;
                if (Index + 1 < Count)
                {
                    List<WorkLane>? NextLanes = ChosenCuts[Index + 1];
                    if ((NextLanes != null) && (NextLanes.Count > 0))
                    {
                        NextCut = NextLanes[0].Start;
                    }
                }

                DirectionChoice Choice = ChooseDirections(
                    PlannedCuts[Index],
                    PlannedFills[Index],
                    NextCut,
                    Grid,
                    Grid.ContainsWorldXy,
                    Clock,
                    RadiusM,
                    BladeWidthM,
                    Grid.GridSizeM,
                    Origin[Index] + 1,
                    Passes[Index].Stage,
                    Grid.AreaM2,
                    BowlCapacityM3,
                    DeferEnds);
                Courses[Index] = Choice.Course;
                Courses[Index].Finishing = Passes[Index].Finishing;
                Courses[Index].RibbonOrder = Passes[Index].RibbonOrder;
                Courses[Index].CutBcy = Passes[Index].LoadBcy;
                ChosenCuts[Index] = Choice.Cuts;
                ChosenFills[Index] = Choice.Fills;
                CutLines[Index] = Choice.CutLines;
                FillLines[Index] = Choice.FillLines;
                HeldCuts[Index] = Choice.HeldCuts;
                HeldFills[Index] = Choice.HeldFills;
                EndCutFragments[Index] = Choice.EndCutFragments;
                EndFillFragments[Index] = Choice.EndFillFragments;
            }
        }

        /// <summary>
        /// Folds a short or connector-heavy correcting trip into a later
        /// correcting trip whose route passes within two radii, when the bowl
        /// can take the load. Otherwise drops it when every cell is already
        /// within the land-plane tolerance.
        /// </summary>
        /// <param name="Passes">Trips in planner order. Not mutated.</param>
        /// <param name="Origin">Original index of each trip.</param>
        /// <param name="Courses">Courses used for the leftover test.</param>
        /// <param name="Cuts">Chosen cut lanes. The fragment the route must pass.</param>
        /// <param name="Fills">Chosen spread lanes. The fragment the route must pass.</param>
        /// <param name="BowlCapacityM3">Scraper bowl, cubic metres.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="NextPasses">Trips after the merge and the drops.</param>
        /// <param name="NextOrigin">Original index of each remaining trip.</param>
        /// <param name="Merged">How many trips were merged.</param>
        /// <param name="Dropped">How many trips were dropped.</param>
        /// <returns>True when the pass list changed.</returns>
        private static Boolean MergeLeftovers
            (
            List<Pass> Passes,
            Int32[] Origin,
            TripCourse[] Courses,
            List<WorkLane>[] Cuts,
            List<WorkLane>[] Fills,
            Double BowlCapacityM3,
            Double RadiusM,
            out List<Pass> NextPasses,
            out Int32[] NextOrigin,
            out Int32 Merged,
            out Int32 Dropped
            )
        {
            NextPasses = Passes;
            NextOrigin = Origin;
            Merged = 0;
            Dropped = 0;
            Int32 Count = Passes.Count;
            Double[] Productive = new Double[Count];
            List<Int32> Pending = new List<Int32>();
            Double Limit = CurrentRules.ShortJointRadii * RadiusM;
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                if (Passes[Index].Stage != HaulStage.Correcting)
                {
                    continue;
                }

                Productive[Index] = ProductiveLengthM(Courses[Index]);
                Double Connectors = ConnectorLengthM(Courses[Index]);
                Boolean Short = Productive[Index] < Limit;
                Boolean Heavy = (Productive[Index] > 1e-6)
                    ? ((Connectors / Productive[Index]) > CurrentRules.LeftoverRatio)
                    : (Connectors > 1e-6);
                if (Short || Heavy)
                {
                    Pending.Add(Index);
                }
            }

            if (Pending.Count == 0)
            {
                return false;
            }

            Pending.Sort(delegate (Int32 Left, Int32 Right)
            {
                Int32 ByLength = Productive[Left].CompareTo(Productive[Right]);
                if (ByLength != 0)
                {
                    return ByLength;
                }

                return Left.CompareTo(Right);
            });
            List<Pass> Working = new List<Pass>(Passes);
            Boolean[] Removed = new Boolean[Count];
            Double BowlBcy = BowlCapacityM3 * Constants.M3_TO_CY;
            foreach (Int32 Source in Pending)
            {
                if (Removed[Source])
                {
                    continue;
                }

                Int32 Target = LaterRoute(
                    Source, Working, Removed, Courses, Cuts, Fills, RadiusM, BowlBcy);
                if (Target >= 0)
                {
                    Working[Target] = CombinePasses(Working[Target], Working[Source]);
                    Removed[Source] = true;
                    Merged += 1;
                }
                else if (WithinTolerance(Working[Source]))
                {
                    Removed[Source] = true;
                    Dropped += 1;
                }
            }

            if ((Merged == 0) && (Dropped == 0))
            {
                return false;
            }

            NextPasses = new List<Pass>();
            List<Int32> Kept = new List<Int32>();
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                if (!Removed[Index])
                {
                    NextPasses.Add(Working[Index]);
                    Kept.Add(Origin[Index]);
                }
            }

            NextOrigin = Kept.ToArray();
            return true;
        }

        /// <summary>
        /// One deferred fragment waiting to be packed into a sweep-up trip.
        /// </summary>
        private sealed class PoolPiece
        {
            /// <summary>
            /// Initializes a fragment.
            /// </summary>
            /// <param name="Cells">Cells in the fragment.</param>
            /// <param name="X">Centroid easting, metres.</param>
            /// <param name="Y">Centroid northing, metres.</param>
            /// <param name="VolumeM3">Bank volume, cubic metres.</param>
            public PoolPiece
                (
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
                Double X,
                Double Y,
                Double VolumeM3
                )
            {
                this.Cells = Cells;
                this.X = X;
                this.Y = Y;
                this.VolumeM3 = VolumeM3;
            }

            /// <summary>Gets the cells.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells { get; }

            /// <summary>Gets the centroid easting.</summary>
            public Double X { get; }

            /// <summary>Gets the centroid northing.</summary>
            public Double Y { get; }

            /// <summary>Gets the bank volume in cubic metres.</summary>
            public Double VolumeM3 { get; }
        }

        /// <summary>
        /// Copies two arrays into one, the left array first.
        /// </summary>
        /// <typeparam name="TItem">Element type.</typeparam>
        /// <param name="Left">The earlier part.</param>
        /// <param name="Right">The later part.</param>
        /// <returns>The joined array.</returns>
        private static TItem[] ConcatArrays<TItem>
            (
            TItem[] Left,
            TItem[] Right
            )
        {
            TItem[] Both = new TItem[Left.Length + Right.Length];
            Array.Copy(Left, Both, Left.Length);
            Array.Copy(Right, 0, Both, Left.Length, Right.Length);
            return Both;
        }

        /// <summary>
        /// Cells on these lanes, in lane order.
        /// </summary>
        /// <param name="Lanes">Lanes whose cells are skipped on the source trip.</param>
        /// <returns>The cells. Empty when <paramref name="Lanes"/> is null.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CellsOfLanes
            (
            List<WorkLane> Lanes
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            if (Lanes == null)
            {
                return Cells;
            }

            foreach (WorkLane Lane in Lanes)
            {
                Cells.AddRange(Lane.Cells);
            }

            return Cells;
        }

        /// <summary>
        /// Lanes the course actually drives. A near-empty lane is left out so the
        /// connectors on either side become one path. The pass still owns the cells.
        /// </summary>
        /// <param name="Source">Lanes in drive order. Near-empty ones may already have been trimmed.</param>
        /// <param name="SourceIds">Line id of each lane. Null when each lane is its own line.</param>
        /// <param name="Keep">True for each lane that still separates connectors.</param>
        /// <param name="KeptIds">Line ids of the lanes that remain. Null when <paramref name="SourceIds"/> is null.</param>
        /// <returns>The lanes that separate connectors.</returns>
        private static List<WorkLane> DrivingLanes
            (
            IReadOnlyList<WorkLane> Source,
            IReadOnlyList<Int32>? SourceIds,
            Boolean[] Keep,
            out List<Int32>? KeptIds
            )
        {
            List<WorkLane> Kept = new List<WorkLane>();
            KeptIds = (SourceIds == null) ? null : new List<Int32>();
            for (Int32 Index = 0; Index < Source.Count; Index += 1)
            {
                if ((Index < Keep.Length) && !Keep[Index])
                {
                    continue;
                }

                Kept.Add(Source[Index]);
                if ((KeptIds != null) && (SourceIds != null) && (Index < SourceIds.Count))
                {
                    KeptIds.Add(SourceIds[Index]);
                }
            }

            return Kept;
        }

        /// <summary>
        /// Which lanes separate connectors. A side that is entirely near-empty
        /// still drives, because there is no other lane for the connectors to join.
        /// </summary>
        /// <param name="Source">Lanes before any short-joint trim.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="CellAreaM2">Cell area. Zero skips the volume test.</param>
        /// <param name="BowlCapacityM3">Bowl capacity. Zero skips the volume test.</param>
        /// <returns>True for each lane the course should drive.</returns>
        private static Boolean[] KeepForDrive
            (
            IReadOnlyList<WorkLane> Source,
            Double RadiusM,
            Double CellAreaM2,
            Double BowlCapacityM3
            )
        {
            Boolean[] Keep = new Boolean[Source.Count];
            Boolean AnySubstantial = false;
            for (Int32 Index = 0; Index < Source.Count; Index += 1)
            {
                Boolean Near = IsNearEmpty(Source[Index], RadiusM, CellAreaM2, BowlCapacityM3);
                Keep[Index] = !Near;
                if (!Near)
                {
                    AnySubstantial = true;
                }
            }

            if (!AnySubstantial)
            {
                for (Int32 Index = 0; Index < Keep.Length; Index += 1)
                {
                    Keep[Index] = true;
                }
            }

            return Keep;
        }

        /// <summary>
        /// True when a lane is too small to be a waypoint between connectors.
        /// A lane with no recorded depth is not a light load; the length test still applies.
        /// </summary>
        /// <param name="Lane">Lane being considered.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="CellAreaM2">Cell area. Zero skips the volume test.</param>
        /// <param name="BowlCapacityM3">Bowl capacity. Zero skips the volume test.</param>
        /// <returns>True when the connectors should meet across this lane.</returns>
        private static Boolean IsNearEmpty
            (
            WorkLane Lane,
            Double RadiusM,
            Double CellAreaM2,
            Double BowlCapacityM3
            )
        {
            if ((RadiusM > 0.0) && (Lane.LengthM < (CurrentRules.NearEmptyRadii * RadiusM)))
            {
                return true;
            }

            if ((CellAreaM2 <= 0.0) || (BowlCapacityM3 <= 0.0))
            {
                return false;
            }

            Double Volume = LaneVolumeM3(Lane, CellAreaM2);
            if (Volume <= 0.0)
            {
                return false;
            }

            return Volume < (CurrentRules.NearEmptyBowlFraction * BowlCapacityM3);
        }

        /// <summary>
        /// Recorded volume of a lane, cubic metres.
        /// </summary>
        /// <param name="Lane">Lane whose cells carry a depth.</param>
        /// <param name="CellAreaM2">Cell area.</param>
        /// <returns>The sum of depth times area.</returns>
        private static Double LaneVolumeM3
            (
            WorkLane Lane,
            Double CellAreaM2
            )
        {
            Double Volume = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Lane.Cells)
            {
                Volume += Math.Abs(Entry.DepthM) * CellAreaM2;
            }

            return Volume;
        }

        /// <summary>
        /// Drops a short end fragment that sits more than four radii past the
        /// next fragment on the same line. The kept lanes are clipped to the
        /// cells that remain.
        /// </summary>
        /// <param name="Lanes">Lanes already grouped onto lines, low station first.</param>
        /// <param name="LineIds">Line id of each lane.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="KeptIds">Line id of each lane that stays.</param>
        /// <param name="Deferred">Fragments taken off the lines.</param>
        /// <returns>The lanes the correcting trip still drives.</returns>
        private static List<WorkLane> TrimEndFragments
            (
            List<WorkLane> Lanes,
            List<Int32> LineIds,
            Grid Grid,
            Double RadiusM,
            out List<Int32> KeptIds,
            out List<WorkLane> Deferred
            )
        {
            Deferred = new List<WorkLane>();
            List<WorkLane> Current = new List<WorkLane>();
            KeptIds = new List<Int32>();
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                // Cells on one centreline can still be two fragments when a hole is wider than four radii.
                WorkLane? Split = SplitLaneEnds(Lanes[Index], Grid, RadiusM, Deferred);
                if (Split == null)
                {
                    continue;
                }

                Current.Add(ClipLaneToCells(Split, Grid));
                KeptIds.Add((Index < LineIds.Count) ? LineIds[Index] : Index);
            }

            Boolean Removed = true;
            while (Removed)
            {
                Removed = false;
                List<Int32> Drop = EndIndexes(Current, KeptIds, RadiusM);
                if (Drop.Count > 0)
                {
                    RemoveAtIndexes(Current, KeptIds, Drop, Deferred);
                    Removed = true;
                }
            }

            // A one-cell run can miss the line group when its fallback heading is off the line.
            // Its cells still sit on the axis, so it is the same kind of end fragment.
            Removed = true;
            while (Removed)
            {
                Removed = false;
                List<Int32> Drop = StrayIndexes(Current, KeptIds, Grid, RadiusM);
                if (Drop.Count > 0)
                {
                    RemoveAtIndexes(Current, KeptIds, Drop, Deferred);
                    Removed = true;
                }
            }

            return Current;
        }

        /// <summary>
        /// Indexes of end lanes that are shorter than one radius and more than
        /// four radii from the next lane on the line.
        /// </summary>
        /// <param name="Lanes">Lanes in drive order.</param>
        /// <param name="LineIds">Line id of each lane.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>Indexes to defer. Empty when every end stays.</returns>
        private static List<Int32> EndIndexes
            (
            List<WorkLane> Lanes,
            List<Int32> LineIds,
            Double RadiusM
            )
        {
            List<Int32> Drop = new List<Int32>();
            Dictionary<Int32, List<Int32>> Groups = IndexesByLine(LineIds);
            foreach (List<Int32> Group in Groups.Values)
            {
                if (Group.Count < 2)
                {
                    continue;
                }

                Int32 Low = Group[0];
                Int32 High = Group[Group.Count - 1];
                if (IsIsolatedEnd(Lanes[Low], Lanes[Group[1]], false, RadiusM))
                {
                    Drop.Add(Low);
                }

                if ((High != Low) && IsIsolatedEnd(Lanes[High], Lanes[Group[Group.Count - 2]], true, RadiusM))
                {
                    Drop.Add(High);
                }
            }

            return Drop;
        }

        /// <summary>
        /// Indexes of solo lanes that are a short end of some other line.
        /// </summary>
        /// <param name="Lanes">Lanes in drive order.</param>
        /// <param name="LineIds">Line id of each lane.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>Indexes to defer.</returns>
        private static List<Int32> StrayIndexes
            (
            List<WorkLane> Lanes,
            List<Int32> LineIds,
            Grid Grid,
            Double RadiusM
            )
        {
            List<Int32> Drop = new List<Int32>();
            Dictionary<Int32, List<Int32>> Groups = IndexesByLine(LineIds);
            Double SpanLimit = CurrentRules.EndFragmentSpanRadii * RadiusM;
            foreach (List<Int32> Group in Groups.Values)
            {
                // A multi-lane line already peeled its own ends.
                if (Group.Count != 1)
                {
                    continue;
                }

                Int32 Index = Group[0];
                if (Lanes[Index].LengthM >= SpanLimit)
                {
                    continue;
                }

                if (ExtendsSomeLine(Index, Lanes, LineIds, Groups, Grid, RadiusM))
                {
                    Drop.Add(Index);
                }
            }

            return Drop;
        }

        /// <summary>
        /// True when <paramref name="Index"/> is a short fragment past the end of another line.
        /// </summary>
        /// <param name="Index">Lane being tested.</param>
        /// <param name="Lanes">All lanes.</param>
        /// <param name="LineIds">Line id of each lane.</param>
        /// <param name="Groups">Lane indexes by line id.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>True when the lane should leave its own line and be deferred.</returns>
        private static Boolean ExtendsSomeLine
            (
            Int32 Index,
            List<WorkLane> Lanes,
            List<Int32> LineIds,
            Dictionary<Int32, List<Int32>> Groups,
            Grid Grid,
            Double RadiusM
            )
        {
            Int32 Own = LineIds[Index];
            foreach (KeyValuePair<Int32, List<Int32>> Pair in Groups)
            {
                if (Pair.Key == Own)
                {
                    continue;
                }

                List<WorkLane> Line = new List<WorkLane>();
                foreach (Int32 LaneIndex in Pair.Value)
                {
                    Line.Add(Lanes[LaneIndex]);
                }

                if (ExtendsLine(Lanes[Index], Line, Grid, RadiusM))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// True when every point of <paramref name="Fragment"/> lies on
        /// <paramref name="Line"/> and the fragment sits more than four radii
        /// past that line's cells.
        /// </summary>
        /// <param name="Fragment">Short lane that may have missed the line group.</param>
        /// <param name="Line">Lanes already on the line.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>True when the fragment is an isolated end of this line.</returns>
        private static Boolean ExtendsLine
            (
            WorkLane Fragment,
            List<WorkLane> Line,
            Grid Grid,
            Double RadiusM
            )
        {
            if ((Line.Count == 0) || (Fragment.LengthM >= (CurrentRules.EndFragmentSpanRadii * RadiusM)))
            {
                return false;
            }

            WorkLane AxisLane = Line[0];
            for (Int32 Index = 1; Index < Line.Count; Index += 1)
            {
                if (Line[Index].LengthM > AxisLane.LengthM)
                {
                    AxisLane = Line[Index];
                }
            }

            LineAxis(AxisLane, out Double OriginX, out Double OriginY, out Double Cos, out Double Sin, out Double AxisHeading);
            if (!OnAxis(Fragment, OriginX, OriginY, Cos, Sin, Grid))
            {
                return false;
            }

            StationSpan(Line, Grid, OriginX, OriginY, Cos, Sin, out Double LineLo, out Double LineHi);
            List<WorkLane> One = new List<WorkLane> { Fragment };
            StationSpan(One, Grid, OriginX, OriginY, Cos, Sin, out Double FragLo, out Double FragHi);
            Double Gap;
            if (FragHi < LineLo)
            {
                Gap = LineLo - FragHi;
            }
            else if (FragLo > LineHi)
            {
                Gap = FragLo - LineHi;
            }
            else
            {
                // The fragment sits inside the line, so it is not an end.
                return false;
            }

            // AxisHeading is only carried so the axis call stays complete.
            _ = AxisHeading;
            return Gap > (CurrentRules.EndFragmentGapRadii * RadiusM);
        }

        /// <summary>
        /// True when both ends of a lane sit in the line's lateral band.
        /// </summary>
        /// <param name="Lane">Lane being tested.</param>
        /// <param name="OriginX">Easting of the axis origin.</param>
        /// <param name="OriginY">Northing of the axis origin.</param>
        /// <param name="Cos">East component of the unit axis.</param>
        /// <param name="Sin">North component of the unit axis.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>True when every sample is inside the band.</returns>
        private static Boolean OnAxis
            (
            WorkLane Lane,
            Double OriginX,
            Double OriginY,
            Double Cos,
            Double Sin,
            Grid Grid
            )
        {
            List<(Double X, Double Y)> Points = LanePoints(Lane, Grid);
            if (Points.Count == 0)
            {
                return false;
            }

            foreach ((Double X, Double Y) Point in Points)
            {
                Double Lateral = Math.Abs(((Cos * (Point.Y - OriginY)) - (Sin * (Point.X - OriginX))));
                if (Lateral > LINE_LATERAL_M)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Station extent of these lanes on an axis.
        /// </summary>
        /// <param name="Lanes">Lanes to measure.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="OriginX">Easting of the axis origin.</param>
        /// <param name="OriginY">Northing of the axis origin.</param>
        /// <param name="Cos">East component of the unit axis.</param>
        /// <param name="Sin">North component of the unit axis.</param>
        /// <param name="Lo">Lowest station.</param>
        /// <param name="Hi">Highest station.</param>
        private static void StationSpan
            (
            IReadOnlyList<WorkLane> Lanes,
            Grid Grid,
            Double OriginX,
            Double OriginY,
            Double Cos,
            Double Sin,
            out Double Lo,
            out Double Hi
            )
        {
            Lo = 0.0;
            Hi = 0.0;
            Boolean Any = false;
            foreach (WorkLane Lane in Lanes)
            {
                foreach ((Double X, Double Y) Point in LanePoints(Lane, Grid))
                {
                    Double Station = ((Point.X - OriginX) * Cos) + ((Point.Y - OriginY) * Sin);
                    if (!Any || (Station < Lo))
                    {
                        Lo = Station;
                    }

                    if (!Any || (Station > Hi))
                    {
                        Hi = Station;
                    }

                    Any = true;
                }
            }
        }

        /// <summary>
        /// Cell centres of a lane, or the two poses when the lane has no cells.
        /// </summary>
        /// <param name="Lane">Lane being measured.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The points.</returns>
        private static List<(Double X, Double Y)> LanePoints
            (
            WorkLane Lane,
            Grid Grid
            )
        {
            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            if (Lane.Cells.Count == 0)
            {
                Points.Add((Lane.Start.X, Lane.Start.Y));
                Points.Add((Lane.End.X, Lane.End.Y));
                return Points;
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Lane.Cells)
            {
                Points.Add(Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By));
            }

            return Points;
        }

        /// <summary>
        /// True when an end lane is shorter than one radius and the gap to the
        /// neighbour is more than four radii.
        /// </summary>
        /// <param name="End">The lane at the end of the line.</param>
        /// <param name="Neighbor">The next lane toward the middle.</param>
        /// <param name="EndIsHigh">True when <paramref name="End"/> is the high-station end.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <returns>True when the end should be deferred.</returns>
        private static Boolean IsIsolatedEnd
            (
            WorkLane End,
            WorkLane Neighbor,
            Boolean EndIsHigh,
            Double RadiusM
            )
        {
            if (End.LengthM >= (CurrentRules.EndFragmentSpanRadii * RadiusM))
            {
                return false;
            }

            // The facing ends are the gap the connector would otherwise drive.
            Double Gap = EndIsHigh
                ? PoseGap(Neighbor.End, End.Start)
                : PoseGap(End.End, Neighbor.Start);
            return Gap > (CurrentRules.EndFragmentGapRadii * RadiusM);
        }

        /// <summary>
        /// Lane indexes grouped by line id, in drive order inside each line.
        /// </summary>
        /// <param name="LineIds">Line id of each lane.</param>
        /// <returns>The groups.</returns>
        private static Dictionary<Int32, List<Int32>> IndexesByLine
            (
            List<Int32> LineIds
            )
        {
            Dictionary<Int32, List<Int32>> Groups = new Dictionary<Int32, List<Int32>>();
            for (Int32 Index = 0; Index < LineIds.Count; Index += 1)
            {
                Int32 Id = LineIds[Index];
                if (!Groups.ContainsKey(Id))
                {
                    Groups[Id] = new List<Int32>();
                }

                Groups[Id].Add(Index);
            }

            return Groups;
        }

        /// <summary>
        /// Removes lanes by index and keeps them as deferred fragments.
        /// </summary>
        /// <param name="Lanes">Lanes. Updated.</param>
        /// <param name="Ids">Line ids. Updated in step with <paramref name="Lanes"/>.</param>
        /// <param name="Drop">Indexes to remove.</param>
        /// <param name="Deferred">Fragments already deferred. The removed lanes are appended.</param>
        private static void RemoveAtIndexes
            (
            List<WorkLane> Lanes,
            List<Int32> Ids,
            List<Int32> Drop,
            List<WorkLane> Deferred
            )
        {
            Drop.Sort();
            List<Int32> Unique = new List<Int32>();
            Int32 Previous = -1;
            foreach (Int32 Index in Drop)
            {
                if (Index != Previous)
                {
                    Unique.Add(Index);
                    Previous = Index;
                }
            }

            for (Int32 Slot = Unique.Count - 1; Slot >= 0; Slot -= 1)
            {
                Int32 Index = Unique[Slot];
                Deferred.Add(Lanes[Index]);
                Lanes.RemoveAt(Index);
                Ids.RemoveAt(Index);
            }
        }

        /// <summary>
        /// Splits a far cell cluster off either end of one lane.
        /// </summary>
        /// <param name="Lane">Lane that may contain more than one fragment.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="Deferred">Fragments already deferred. Peeled clusters are appended.</param>
        /// <returns>The lane without those clusters, or null when every cell was peeled.</returns>
        private static WorkLane? SplitLaneEnds
            (
            WorkLane Lane,
            Grid Grid,
            Double RadiusM,
            List<WorkLane> Deferred
            )
        {
            if ((Lane.Cells.Count < 2) || (Lane.Centerline.Count < 2))
            {
                return Lane;
            }

            Double GapLimit = CurrentRules.EndFragmentGapRadii * RadiusM;
            Double SpanLimit = CurrentRules.EndFragmentSpanRadii * RadiusM;
            List<(Double Station, Int32 Index)> Placed = new List<(Double Station, Int32 Index)>();
            List<Int32> Unplaced = new List<Int32>();
            for (Int32 Index = 0; Index < Lane.Cells.Count; Index += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Lane.Cells[Index];
                (Double X, Double Y) Center = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                if (!TryNearestStation(Lane.Centerline, Center.X, Center.Y, out Double Station))
                {
                    // A cell the centreline does not cover stays with the lane.
                    Unplaced.Add(Index);
                    continue;
                }

                Placed.Add((Station, Index));
            }

            if (Placed.Count < 2)
            {
                return Lane;
            }

            Placed.Sort(ComparePlaced);
            List<List<Int32>> Clusters = new List<List<Int32>>();
            List<Double> ClusterLo = new List<Double>();
            List<Double> ClusterHi = new List<Double>();
            Clusters.Add(new List<Int32> { Placed[0].Index });
            ClusterLo.Add(Placed[0].Station);
            ClusterHi.Add(Placed[0].Station);
            for (Int32 Index = 1; Index < Placed.Count; Index += 1)
            {
                if ((Placed[Index].Station - Placed[Index - 1].Station) > GapLimit)
                {
                    Clusters.Add(new List<Int32>());
                    ClusterLo.Add(Placed[Index].Station);
                    ClusterHi.Add(Placed[Index].Station);
                }

                Int32 Last = Clusters.Count - 1;
                Clusters[Last].Add(Placed[Index].Index);
                if (Placed[Index].Station < ClusterLo[Last])
                {
                    ClusterLo[Last] = Placed[Index].Station;
                }

                if (Placed[Index].Station > ClusterHi[Last])
                {
                    ClusterHi[Last] = Placed[Index].Station;
                }
            }

            if (Clusters.Count < 2)
            {
                return Lane;
            }

            Boolean Peeled = true;
            while (Peeled && (Clusters.Count >= 2))
            {
                Peeled = false;
                if ((ClusterHi[0] - ClusterLo[0]) < SpanLimit)
                {
                    AddFragment(Deferred, Lane, Clusters[0], Grid);
                    Clusters.RemoveAt(0);
                    ClusterLo.RemoveAt(0);
                    ClusterHi.RemoveAt(0);
                    Peeled = true;
                }

                Int32 Tail = Clusters.Count - 1;
                if ((Clusters.Count >= 2) && ((ClusterHi[Tail] - ClusterLo[Tail]) < SpanLimit))
                {
                    AddFragment(Deferred, Lane, Clusters[Tail], Grid);
                    Clusters.RemoveAt(Tail);
                    ClusterLo.RemoveAt(Tail);
                    ClusterHi.RemoveAt(Tail);
                    Peeled = true;
                }
            }

            HashSet<Int32> Keep = new HashSet<Int32>(Unplaced);
            foreach (List<Int32> Cluster in Clusters)
            {
                foreach (Int32 Index in Cluster)
                {
                    Keep.Add(Index);
                }
            }

            if (Keep.Count == 0)
            {
                return null;
            }

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Index = 0; Index < Lane.Cells.Count; Index += 1)
            {
                if (Keep.Contains(Index))
                {
                    Cells.Add(Lane.Cells[Index]);
                }
            }

            return new WorkLane(
                Lane.Centerline,
                Lane.Start,
                Lane.End,
                Cells,
                Lane.LengthM,
                Lane.StraightLengthM,
                Lane.ArcLengthM,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Lane.Deferred));
        }

        /// <summary>
        /// Orders cell stations. An equal station keeps the original cell index.
        /// </summary>
        /// <param name="Left">One placed cell.</param>
        /// <param name="Right">The other placed cell.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 ComparePlaced
            (
            (Double Station, Int32 Index) Left,
            (Double Station, Int32 Index) Right
            )
        {
            Int32 ByStation = Left.Station.CompareTo(Right.Station);
            if (ByStation != 0)
            {
                return ByStation;
            }

            return Left.Index.CompareTo(Right.Index);
        }

        /// <summary>
        /// Records one peeled cell cluster as its own fragment.
        /// </summary>
        /// <param name="Deferred">Fragments already deferred.</param>
        /// <param name="Parent">Lane the cells came from. Its heading is the fallback.</param>
        /// <param name="CellIndexes">Indexes into <paramref name="Parent"/>'s cells, in station order.</param>
        /// <param name="Grid">Field grid.</param>
        private static void AddFragment
            (
            List<WorkLane> Deferred,
            WorkLane Parent,
            List<Int32> CellIndexes,
            Grid Grid
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            List<(Double X, Double Y)> Line = new List<(Double X, Double Y)>();
            foreach (Int32 Index in CellIndexes)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Parent.Cells[Index];
                Cells.Add(Entry);
                Line.Add(Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By));
            }

            if (Cells.Count == 0)
            {
                return;
            }

            Deferred.Add(WorkLane.FromCenterline(Line, Cells, Parent.Start.HeadingRad));
        }

        /// <summary>
        /// Packs deferred fragments into correcting trips. Each trip takes the
        /// nearest remaining cut until the bowl is full, then the nearest fills
        /// for that load. The caller plans those trips with the usual shape rules.
        /// </summary>
        /// <param name="Cuts">Deferred cut fragments.</param>
        /// <param name="Fills">Deferred spread fragments.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BowlM3">Bowl capacity, cubic metres.</param>
        /// <param name="Start">Pose the correcting pass ended on. Null starts at the first fragment.</param>
        /// <returns>The sweep-up trips, in nearest-neighbour order.</returns>
        private static List<Pass> BuildSweepPasses
            (
            List<WorkLane> Cuts,
            List<WorkLane> Fills,
            Grid Grid,
            Double BowlM3,
            DubinsConnector.Pose? Start
            )
        {
            List<PoolPiece> CutPieces = ToPieces(Cuts, Grid);
            List<PoolPiece> FillPieces = ToPieces(Fills, Grid);
            List<Pass> Trips = new List<Pass>();
            Double CursorX = 0.0;
            Double CursorY = 0.0;
            Boolean HaveCursor = false;
            if (Start.HasValue)
            {
                CursorX = Start.Value.X;
                CursorY = Start.Value.Y;
                HaveCursor = true;
            }
            else if (CutPieces.Count > 0)
            {
                CursorX = CutPieces[0].X;
                CursorY = CutPieces[0].Y;
                HaveCursor = true;
            }
            else if (FillPieces.Count > 0)
            {
                CursorX = FillPieces[0].X;
                CursorY = FillPieces[0].Y;
                HaveCursor = true;
            }

            Int32 Guard = CutPieces.Count + FillPieces.Count + 1;
            while (((CutPieces.Count > 0) || (FillPieces.Count > 0)) && (Guard > 0))
            {
                Guard -= 1;
                Int32 Before = CutPieces.Count + FillPieces.Count;
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> TripCuts =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> TripFills =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                Double Load = 0.0;
                Double LastX = CursorX;
                Double LastY = CursorY;
                if (CutPieces.Count > 0)
                {
                    // The first fragment is taken even when it is heavier than the bowl.
                    Int32 Seed = HaveCursor ? NearestPiece(CutPieces, LastX, LastY) : 0;
                    TakePiece(CutPieces, Seed, TripCuts, ref Load, ref LastX, ref LastY);
                    while ((CutPieces.Count > 0) && (Load < (BowlM3 - 1e-6)))
                    {
                        Int32 Next = NearestPiece(CutPieces, LastX, LastY);
                        if ((Load + CutPieces[Next].VolumeM3) > (BowlM3 + 1e-6))
                        {
                            break;
                        }

                        TakePiece(CutPieces, Next, TripCuts, ref Load, ref LastX, ref LastY);
                    }
                }

                Double Placed = 0.0;
                while (FillPieces.Count > 0)
                {
                    Int32 Next = NearestPiece(FillPieces, LastX, LastY);
                    Boolean HaveFill = TripFills.Count > 0;
                    if ((TripCuts.Count > 0) && HaveFill && (Placed >= (Load - 1e-9)))
                    {
                        break;
                    }

                    if ((TripCuts.Count > 0) && HaveFill
                        && ((Placed + FillPieces[Next].VolumeM3) > (Load + 1e-6)))
                    {
                        // Leave an oversized fill for a later load that can use it.
                        break;
                    }

                    if ((TripCuts.Count == 0) && HaveFill
                        && ((Placed + FillPieces[Next].VolumeM3) > (BowlM3 + 1e-6)))
                    {
                        break;
                    }

                    TakePiece(FillPieces, Next, TripFills, ref Placed, ref LastX, ref LastY);
                    if ((TripCuts.Count == 0) && (Placed >= (BowlM3 - 1e-6)))
                    {
                        break;
                    }
                }

                if ((TripCuts.Count == 0) && (TripFills.Count == 0))
                {
                    break;
                }

                if ((CutPieces.Count + FillPieces.Count) >= Before)
                {
                    break;
                }

                Double Moved = (TripCuts.Count > 0) ? Load : Placed;
                (Double X, Double Y) CutStart = (0.0, 0.0);
                (Double X, Double Y) CutEnd = (0.0, 0.0);
                (Double X, Double Y) FillStart = (0.0, 0.0);
                (Double X, Double Y) FillEnd = (0.0, 0.0);
                if (TripCuts.Count > 0)
                {
                    CutStart = Grid.CellCenterXy(TripCuts[0].Cell.Bx, TripCuts[0].Cell.By);
                    ((Int32 Bx, Int32 By) Cell, Double DepthM) LastCut = TripCuts[TripCuts.Count - 1];
                    CutEnd = Grid.CellCenterXy(LastCut.Cell.Bx, LastCut.Cell.By);
                }

                if (TripFills.Count > 0)
                {
                    FillStart = Grid.CellCenterXy(TripFills[0].Cell.Bx, TripFills[0].Cell.By);
                    ((Int32 Bx, Int32 By) Cell, Double DepthM) LastFill = TripFills[TripFills.Count - 1];
                    FillEnd = Grid.CellCenterXy(LastFill.Cell.Bx, LastFill.Cell.By);
                }

                Pass Trip = new Pass(
                    TripCuts,
                    TripFills,
                    Moved * Constants.M3_TO_CY,
                    0.0,
                    CutStart,
                    CutEnd,
                    FillStart,
                    FillEnd);
                Trip.Stage = HaulStage.Correcting;
                Trip.StackedLifts = true;
                // The lane builder retitles the cells, so a jump is not blade-down.
                Trip.RibbonOrder = false;
                Trips.Add(Trip);
                CursorX = LastX;
                CursorY = LastY;
                HaveCursor = true;
            }

            return Trips;
        }

        /// <summary>
        /// Turns fragment lanes into pieces the packer can measure.
        /// </summary>
        /// <param name="Lanes">Deferred fragments.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>One piece per lane that has cells.</returns>
        private static List<PoolPiece> ToPieces
            (
            List<WorkLane> Lanes,
            Grid Grid
            )
        {
            List<PoolPiece> Pieces = new List<PoolPiece>();
            foreach (WorkLane Lane in Lanes)
            {
                if (Lane.Cells.Count == 0)
                {
                    continue;
                }

                Double SumX = 0.0;
                Double SumY = 0.0;
                Double Volume = 0.0;
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Lane.Cells);
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
                {
                    (Double X, Double Y) Center = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                    SumX += Center.X;
                    SumY += Center.Y;
                    Volume += Math.Abs(Entry.DepthM) * Grid.AreaM2;
                }

                Double Count = Cells.Count;
                Pieces.Add(new PoolPiece(Cells, SumX / Count, SumY / Count, Volume));
            }

            return Pieces;
        }

        /// <summary>
        /// Index of the piece closest to a point. An equal distance keeps the earlier piece.
        /// </summary>
        /// <param name="Pieces">Remaining pieces.</param>
        /// <param name="X">Easting, metres.</param>
        /// <param name="Y">Northing, metres.</param>
        /// <returns>The index.</returns>
        private static Int32 NearestPiece
            (
            List<PoolPiece> Pieces,
            Double X,
            Double Y
            )
        {
            Int32 Best = 0;
            Double BestDistance = Double.MaxValue;
            for (Int32 Index = 0; Index < Pieces.Count; Index += 1)
            {
                Double Dx = Pieces[Index].X - X;
                Double Dy = Pieces[Index].Y - Y;
                Double Distance = (Dx * Dx) + (Dy * Dy);
                if (Distance < BestDistance)
                {
                    BestDistance = Distance;
                    Best = Index;
                }
            }

            return Best;
        }

        /// <summary>
        /// Moves one piece onto the trip being packed.
        /// </summary>
        /// <param name="Pieces">Remaining pieces. The chosen index is removed.</param>
        /// <param name="Index">Piece to take.</param>
        /// <param name="Cells">Trip cells. The piece's cells are appended.</param>
        /// <param name="Volume">Running volume. Increased by the piece.</param>
        /// <param name="X">Cursor easting. Set to the piece centroid.</param>
        /// <param name="Y">Cursor northing. Set to the piece centroid.</param>
        private static void TakePiece
            (
            List<PoolPiece> Pieces,
            Int32 Index,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            ref Double Volume,
            ref Double X,
            ref Double Y
            )
        {
            PoolPiece Piece = Pieces[Index];
            Cells.AddRange(Piece.Cells);
            Volume += Piece.VolumeM3;
            X = Piece.X;
            Y = Piece.Y;
            Pieces.RemoveAt(Index);
        }

        /// <summary>
        /// Clips every correcting lane to the station span of its cells.
        /// </summary>
        /// <param name="Lanes">Lanes in drive order.</param>
        /// <param name="Grid">Field grid. Cell centres are the fragment positions.</param>
        /// <returns>Lanes that start and end on their fragments.</returns>
        private static List<WorkLane> ClipToFragments
            (
            List<WorkLane> Lanes,
            Grid Grid
            )
        {
            List<WorkLane> Clipped = new List<WorkLane>(Lanes.Count);
            foreach (WorkLane Lane in Lanes)
            {
                Clipped.Add(ClipLaneToCells(Lane, Grid));
            }

            return Clipped;
        }

        /// <summary>
        /// Drops centreline that runs past the first and last cell. The cells stay.
        /// A bend is not involved: this runs before a direction is chosen.
        /// </summary>
        /// <param name="Lane">Lane whose centreline may run out to the field edge.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The same lane when it already sits on its cells.</returns>
        private static WorkLane ClipLaneToCells
            (
            WorkLane Lane,
            Grid Grid
            )
        {
            if ((Lane.Cells.Count == 0) || (Lane.Centerline.Count < 2))
            {
                return Lane;
            }

            Double Poly = PolylineLength(Lane.Centerline);
            if (Poly <= LINE_POINT_M)
            {
                return Lane;
            }

            Double MinS = Poly;
            Double MaxS = 0.0;
            Boolean Any = false;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Lane.Cells)
            {
                (Double X, Double Y) Center = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                if (TryNearestStation(Lane.Centerline, Center.X, Center.Y, out Double Station))
                {
                    Any = true;
                    if (Station < MinS)
                    {
                        MinS = Station;
                    }

                    if (Station > MaxS)
                    {
                        MaxS = Station;
                    }
                }
            }

            if (!Any)
            {
                return Lane;
            }

            Double Pad = Grid.GridSizeM * FRAGMENT_PAD_CELLS;
            Double KeepStart = Math.Max(0.0, MinS - Pad);
            Double KeepEnd = Math.Min(Poly, MaxS + Pad);
            if ((KeepStart <= LINE_POINT_M) && (KeepEnd >= (Poly - LINE_POINT_M)))
            {
                return Lane;
            }

            List<(Double X, Double Y)> Kept = SlicePolyline(Lane.Centerline, KeepStart, KeepEnd);
            if (Kept.Count < 2)
            {
                return Lane;
            }

            WorkLane Fresh = WorkLane.FromCenterline(Kept, Lane.Cells, Lane.Start.HeadingRad);
            if (Lane.Deferred.Count == 0)
            {
                return Fresh;
            }

            return new WorkLane(
                Fresh.Centerline,
                Fresh.Start,
                Fresh.End,
                Lane.Cells,
                Fresh.LengthM,
                Fresh.StraightLengthM,
                0.0,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Lane.Deferred));
        }

        /// <summary>
        /// Station of the closest point on a polyline.
        /// </summary>
        /// <param name="Line">Polyline in drive order.</param>
        /// <param name="X">Easting, metres.</param>
        /// <param name="Y">Northing, metres.</param>
        /// <param name="Station">Distance from the start of the polyline.</param>
        /// <returns>False when the polyline has no segment.</returns>
        private static Boolean TryNearestStation
            (
            IReadOnlyList<(Double X, Double Y)> Line,
            Double X,
            Double Y,
            out Double Station
            )
        {
            Station = 0.0;
            Double Best = Double.MaxValue;
            Double Cursor = 0.0;
            Boolean Found = false;
            for (Int32 Index = 1; Index < Line.Count; Index += 1)
            {
                Double Dx = Line[Index].X - Line[Index - 1].X;
                Double Dy = Line[Index].Y - Line[Index - 1].Y;
                Double Segment = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Segment > 1e-9)
                {
                    Double T = (((X - Line[Index - 1].X) * Dx) + ((Y - Line[Index - 1].Y) * Dy)) / (Segment * Segment);
                    if (T < 0.0)
                    {
                        T = 0.0;
                    }

                    if (T > 1.0)
                    {
                        T = 1.0;
                    }

                    Double Px = Line[Index - 1].X + (T * Dx);
                    Double Py = Line[Index - 1].Y + (T * Dy);
                    Double Dist = ((X - Px) * (X - Px)) + ((Y - Py) * (Y - Py));
                    if (Dist < Best)
                    {
                        Best = Dist;
                        Station = Cursor + (T * Segment);
                        Found = true;
                    }
                }

                Cursor += Segment;
            }

            return Found;
        }

        /// <summary>
        /// Closest later correcting trip whose route passes within two radii of
        /// this fragment and whose bowl still has room.
        /// </summary>
        /// <param name="Source">Leftover trip index.</param>
        /// <param name="Passes">Current passes, including earlier merges.</param>
        /// <param name="Removed">Trips already merged or dropped.</param>
        /// <param name="Courses">Routes from the first plan.</param>
        /// <param name="Cuts">Cut lanes of each trip.</param>
        /// <param name="Fills">Spread lanes of each trip.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BowlBcy">Bowl capacity in bank cubic yards.</param>
        /// <returns>The trip index, or -1 when none can take the load.</returns>
        private static Int32 LaterRoute
            (
            Int32 Source,
            List<Pass> Passes,
            Boolean[] Removed,
            TripCourse[] Courses,
            List<WorkLane>[] Cuts,
            List<WorkLane>[] Fills,
            Double RadiusM,
            Double BowlBcy
            )
        {
            Double Reach = CurrentRules.MergeRouteRadii * RadiusM;
            Int32 Best = -1;
            Double BestDist = 0.0;
            for (Int32 Index = Source + 1; Index < Passes.Count; Index += 1)
            {
                if (Removed[Index] || (Passes[Index].Stage != HaulStage.Correcting))
                {
                    continue;
                }

                if ((Passes[Index].LoadBcy + Passes[Source].LoadBcy) > (BowlBcy + 1e-6))
                {
                    continue;
                }

                if (!RoutePassesNear(Cuts[Source], Fills[Source], Courses[Index].Points, Reach, out Double Dist))
                {
                    continue;
                }

                // The closest route takes the fragment. An equal distance keeps the earlier trip.
                if ((Best < 0) || (Dist < BestDist))
                {
                    Best = Index;
                    BestDist = Dist;
                }
            }

            return Best;
        }

        /// <summary>
        /// True when any sample of a route comes within <paramref name="Reach"/> of a fragment lane.
        /// </summary>
        /// <param name="Cuts">Fragment cut lanes.</param>
        /// <param name="Fills">Fragment spread lanes.</param>
        /// <param name="Route">Later trip's guidance samples.</param>
        /// <param name="Reach">Metres.</param>
        /// <param name="Best">Closest approach, when one is inside the reach.</param>
        /// <returns>False when every sample is farther than the reach.</returns>
        private static Boolean RoutePassesNear
            (
            List<WorkLane> Cuts,
            List<WorkLane> Fills,
            IReadOnlyList<GuidancePoint> Route,
            Double Reach,
            out Double Best
            )
        {
            Best = Double.MaxValue;
            if ((Route == null) || (Route.Count == 0))
            {
                return false;
            }

            Double Reach2 = Reach * Reach;
            Boolean Hit = false;
            foreach (GuidancePoint Point in Route)
            {
                Double Dist2 = DistanceToLanesSquared(Cuts, Fills, Point.X, Point.Y);
                if (Dist2 < Best)
                {
                    Best = Dist2;
                }

                if (Dist2 <= Reach2)
                {
                    Hit = true;
                }
            }

            if (!Hit)
            {
                return false;
            }

            Best = Math.Sqrt(Best);
            return true;
        }

        /// <summary>
        /// Squared distance from a point to the nearest fragment centreline.
        /// Lanes are not joined, so the gap between a cut and a spread is not a shortcut.
        /// </summary>
        /// <param name="Cuts">Cut lanes.</param>
        /// <param name="Fills">Spread lanes.</param>
        /// <param name="X">Easting, metres.</param>
        /// <param name="Y">Northing, metres.</param>
        /// <returns>Squared metres. A huge value when the fragment has no centreline.</returns>
        private static Double DistanceToLanesSquared
            (
            List<WorkLane> Cuts,
            List<WorkLane> Fills,
            Double X,
            Double Y
            )
        {
            Double Best = Double.MaxValue;
            Best = CloserLane(Cuts, X, Y, Best);
            Best = CloserLane(Fills, X, Y, Best);
            return Best;
        }

        /// <summary>
        /// Keeps the closer of <paramref name="Best"/> and these lanes.
        /// </summary>
        /// <param name="Lanes">Lanes to measure.</param>
        /// <param name="X">Easting, metres.</param>
        /// <param name="Y">Northing, metres.</param>
        /// <param name="Best">Squared distance so far.</param>
        /// <returns>The smaller squared distance.</returns>
        private static Double CloserLane
            (
            List<WorkLane> Lanes,
            Double X,
            Double Y,
            Double Best
            )
        {
            if (Lanes == null)
            {
                return Best;
            }

            foreach (WorkLane Lane in Lanes)
            {
                Double Dist2 = PointToPolylineSquared(Lane.Centerline, X, Y);
                if (Dist2 < Best)
                {
                    Best = Dist2;
                }
            }

            return Best;
        }

        /// <summary>
        /// Squared distance from a point to a polyline.
        /// </summary>
        /// <param name="Line">Polyline. A single point is that point.</param>
        /// <param name="X">Easting, metres.</param>
        /// <param name="Y">Northing, metres.</param>
        /// <returns>Squared metres.</returns>
        private static Double PointToPolylineSquared
            (
            IReadOnlyList<(Double X, Double Y)> Line,
            Double X,
            Double Y
            )
        {
            if ((Line == null) || (Line.Count == 0))
            {
                return Double.MaxValue;
            }

            if (Line.Count == 1)
            {
                Double Dx = X - Line[0].X;
                Double Dy = Y - Line[0].Y;
                return (Dx * Dx) + (Dy * Dy);
            }

            Double Best = Double.MaxValue;
            for (Int32 Index = 1; Index < Line.Count; Index += 1)
            {
                Double Sx = Line[Index].X - Line[Index - 1].X;
                Double Sy = Line[Index].Y - Line[Index - 1].Y;
                Double Segment = (Sx * Sx) + (Sy * Sy);
                Double T = 0.0;
                if (Segment > 1e-18)
                {
                    T = (((X - Line[Index - 1].X) * Sx) + ((Y - Line[Index - 1].Y) * Sy)) / Segment;
                    if (T < 0.0)
                    {
                        T = 0.0;
                    }

                    if (T > 1.0)
                    {
                        T = 1.0;
                    }
                }

                Double Px = Line[Index - 1].X + (T * Sx);
                Double Py = Line[Index - 1].Y + (T * Sy);
                Double Dist = ((X - Px) * (X - Px)) + ((Y - Py) * (Y - Py));
                if (Dist < Best)
                {
                    Best = Dist;
                }
            }

            return Best;
        }

        /// <summary>
        /// One correcting pass that carries both trips' cells. The lane builder
        /// retitles them, so the two lines stay separate.
        /// </summary>
        /// <param name="Target">The trip that keeps its place in the list.</param>
        /// <param name="Source">The trip whose cells move.</param>
        /// <returns>The combined pass.</returns>
        private static Pass CombinePasses
            (
            Pass Target,
            Pass Source
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Target.CutCells);
            Cuts.AddRange(Source.CutCells);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Target.FillCells);
            Fills.AddRange(Source.FillCells);
            Pass Combined = new Pass(
                Cuts,
                Fills,
                Target.LoadBcy + Source.LoadBcy,
                Target.HaulDistanceM + Source.HaulDistanceM,
                Target.CutStartXy,
                Target.CutEndXy,
                Target.FillStartXy,
                Target.FillEndXy);
            Combined.Stage = HaulStage.Correcting;
            Combined.StackedLifts = true;
            // Ribbon order would draw the jump between the two lines with the blade down.
            Combined.RibbonOrder = false;
            return Combined;
        }

        /// <summary>
        /// True when every cut and fill cell is already inside the land-plane tolerance.
        /// </summary>
        /// <param name="Trip">One pass.</param>
        /// <returns>True when the trip can be dropped.</returns>
        private static Boolean WithinTolerance
            (
            Pass Trip
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Trip.CutCells)
            {
                if (Entry.DepthM > GRADE_DEPTH_M)
                {
                    return false;
                }
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Trip.FillCells)
            {
                if (Entry.DepthM > GRADE_DEPTH_M)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Replays the recorded cut and fill in planner order and in drive order.
        /// The two surfaces must agree to 1 mm. Grade returns follow the empty
        /// travel, so they are not part of this check.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Passes">Trips after leftover merge.</param>
        /// <param name="DriveOrder">Indexes into <paramref name="Passes"/>.</param>
        /// <param name="Cuts">Chosen cut lanes.</param>
        /// <param name="Fills">Chosen spread lanes.</param>
        /// <param name="HeldCuts">Cut cells the path does not drive.</param>
        /// <param name="HeldFills">Spread cells the path does not drive.</param>
        /// <param name="EndSkipCuts">Cut cells deferred onto a sweep-up trip.</param>
        /// <param name="EndSkipFills">Spread cells deferred onto a sweep-up trip.</param>
        private static void RequireSameSurface
            (
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            Int32[] DriveOrder,
            List<WorkLane>[] Cuts,
            List<WorkLane>[] Fills,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] HeldCuts,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] HeldFills,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] EndSkipCuts,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] EndSkipFills
            )
        {
            Int32 Count = Passes.Count;
            HashSet<(Int32 Bx, Int32 By)>[] CutSkip = new HashSet<(Int32 Bx, Int32 By)>[Count];
            HashSet<(Int32 Bx, Int32 By)>[] FillSkip = new HashSet<(Int32 Bx, Int32 By)>[Count];
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                CutSkip[Index] = DeferredCells(Cuts[Index]);
                FillSkip[Index] = DeferredCells(Fills[Index]);
                AddHeld(CutSkip[Index], HeldCuts[Index]);
                AddHeld(FillSkip[Index], HeldFills[Index]);
                AddHeld(CutSkip[Index], EndSkipCuts[Index]);
                AddHeld(FillSkip[Index], EndSkipFills[Index]);
            }

            Double[,] Planned = Grid.Delta();
            Double[,] Driven = Grid.Delta();
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                ApplyPass(Planned, Passes[Index], CutSkip[Index], FillSkip[Index]);
            }

            for (Int32 Step = 0; Step < DriveOrder.Length; Step += 1)
            {
                Int32 Index = DriveOrder[Step];
                ApplyPass(Driven, Passes[Index], CutSkip[Index], FillSkip[Index]);
            }

            Double Worst = 0.0;
            Int32 WorstX = 0;
            Int32 WorstY = 0;
            Int32 Height = Planned.GetLength(0);
            Int32 Width = Planned.GetLength(1);
            for (Int32 Row = 0; Row < Height; Row += 1)
            {
                for (Int32 Column = 0; Column < Width; Column += 1)
                {
                    if ((Grid.Valid != null) && !Grid.Valid[Row, Column])
                    {
                        continue;
                    }

                    Double Left = Planned[Row, Column];
                    Double Right = Driven[Row, Column];
                    if (Double.IsNaN(Left) || Double.IsNaN(Right))
                    {
                        continue;
                    }

                    Double Gap = Math.Abs(Left - Right);
                    if (Gap > Worst)
                    {
                        Worst = Gap;
                        WorstX = Column;
                        WorstY = Row;
                    }
                }
            }

            if (Worst > SURFACE_MATCH_M)
            {
                throw new InvalidOperationException(String.Format(
                    CultureInfo.InvariantCulture,
                    "Reordered plan differs by {0:F4} m at cell {1},{2}. The limit is 1 mm.",
                    Worst,
                    WorstX,
                    WorstY));
            }
        }

        /// <summary>
        /// Adds cells the path did not drive to a skip set.
        /// </summary>
        /// <param name="Skip">Cells already left behind by a bend or a trim.</param>
        /// <param name="Held">Cells past the line cap.</param>
        private static void AddHeld
            (
            HashSet<(Int32 Bx, Int32 By)> Skip,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Held
            )
        {
            if (Held == null)
            {
                return;
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Held)
            {
                Skip.Add(Entry.Cell);
            }
        }

        /// <summary>
        /// Orders lines by lateral offset and keeps the first three. Odd spread
        /// lines are reversed so the spreads snake. Dropped cells are returned
        /// for a later pass.
        /// </summary>
        /// <param name="Lanes">Lanes already grouped onto lines.</param>
        /// <param name="LineIds">Line id of each lane.</param>
        /// <param name="Serpentine">True reverses every other line.</param>
        /// <param name="SweptIds">Line id of each kept lane, in sweep order.</param>
        /// <param name="Held">Cells on the lines that were not kept.</param>
        /// <returns>The lanes the trip drives.</returns>
        private static List<WorkLane> SweepLines
            (
            List<WorkLane> Lanes,
            List<Int32> LineIds,
            Boolean Serpentine,
            out List<Int32> SweptIds,
            out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Held
            )
        {
            SweptIds = new List<Int32>();
            Held = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            if ((Lanes == null) || (Lanes.Count == 0))
            {
                return new List<WorkLane>();
            }

            List<Int32> Seen = new List<Int32>();
            Dictionary<Int32, List<WorkLane>> Groups = new Dictionary<Int32, List<WorkLane>>();
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                Int32 Id = (Index < LineIds.Count) ? LineIds[Index] : Index;
                if (!Groups.ContainsKey(Id))
                {
                    Groups[Id] = new List<WorkLane>();
                    Seen.Add(Id);
                }

                Groups[Id].Add(Lanes[Index]);
            }

            WorkLane OriginLane = Groups[Seen[0]][0];
            Double Heading = OriginLane.Start.HeadingRad;
            Double OriginX = OriginLane.Start.X;
            Double OriginY = OriginLane.Start.Y;
            List<Int32> Ranked = new List<Int32>(Seen);
            Ranked.Sort(delegate (Int32 Left, Int32 Right)
            {
                Double LeftOffset = LineOffset(Groups[Left][0], OriginX, OriginY, Heading);
                Double RightOffset = LineOffset(Groups[Right][0], OriginX, OriginY, Heading);
                Int32 ByOffset = LeftOffset.CompareTo(RightOffset);
                if (ByOffset != 0)
                {
                    return ByOffset;
                }

                return Seen.IndexOf(Left).CompareTo(Seen.IndexOf(Right));
            });
            Int32 Keep = Ranked.Count;
            if (Keep > CurrentRules.MaxCorrectingLines)
            {
                Keep = CurrentRules.MaxCorrectingLines;
            }

            for (Int32 Index = Keep; Index < Ranked.Count; Index += 1)
            {
                foreach (WorkLane Lane in Groups[Ranked[Index]])
                {
                    Held.AddRange(Lane.Cells);
                    Held.AddRange(Lane.Deferred);
                }
            }

            List<WorkLane> Result = new List<WorkLane>();
            for (Int32 Index = 0; Index < Keep; Index += 1)
            {
                List<WorkLane> Group = Groups[Ranked[Index]];
                if (Serpentine && ((Index % 2) == 1))
                {
                    // Start this line at the end the previous line just reached.
                    for (Int32 LaneIndex = Group.Count - 1; LaneIndex >= 0; LaneIndex -= 1)
                    {
                        Result.Add(Group[LaneIndex].Reversed());
                        SweptIds.Add(Index);
                    }
                }
                else
                {
                    foreach (WorkLane Lane in Group)
                    {
                        Result.Add(Lane);
                        SweptIds.Add(Index);
                    }
                }
            }

            return Result;
        }

        /// <summary>
        /// Signed sideways position of a lane start, metres.
        /// </summary>
        /// <param name="Lane">One lane on the line.</param>
        /// <param name="OriginX">Easting of the first line.</param>
        /// <param name="OriginY">Northing of the first line.</param>
        /// <param name="Heading">Heading of the first line.</param>
        /// <returns>Metres to the left of the first line.</returns>
        private static Double LineOffset
            (
            WorkLane Lane,
            Double OriginX,
            Double OriginY,
            Double Heading
            )
        {
            Double Dx = Lane.Start.X - OriginX;
            Double Dy = Lane.Start.Y - OriginY;
            return (Dx * -Math.Sin(Heading)) + (Dy * Math.Cos(Heading));
        }

        /// <summary>
        /// Tries the start slides and the end trims of a short joint. Each trim
        /// adds its six Dubins words, or a straight steer, to the same pool as
        /// the untrimmed words. The near-shortest path with the least length
        /// outside the field keeps its trim.
        /// </summary>
        /// <param name="Cuts">Cut lanes. Updated in place.</param>
        /// <param name="Fills">Spread lanes. Updated in place.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <param name="Inside">Field test. Null ignores the boundary.</param>
        /// <param name="RoadBank">True on the road stage. A forward lane change is not trimmed into a steer.</param>
        private static void FitShortJoints
            (
            List<WorkLane> Cuts,
            List<WorkLane> Fills,
            Double RadiusM,
            Double BladeWidthM,
            Func<Double, Double, Boolean>? Inside,
            Boolean RoadBank
            )
        {
            List<WorkLane> All = new List<WorkLane>(Cuts.Count + Fills.Count);
            All.AddRange(Cuts);
            All.AddRange(Fills);
            Double[] Slides = new Double[] { 0.0, 0.5, 1.0, 1.5, 2.0 };
            for (Int32 Index = 0; Index < (All.Count - 1); Index += 1)
            {
                WorkLane Left = All[Index];
                WorkLane Right = All[Index + 1];
                if (RoadBank && IsForwardLaneChange(Left.End, Right.Start, BladeWidthM, out Double _))
                {
                    // The cells already leave room for the S-bend. Sliding them would shorten it.
                    continue;
                }

                Double Gap = PoseGap(Left.End, Right.Start);
                if (Gap >= (CurrentRules.ShortJointRadii * RadiusM))
                {
                    continue;
                }

                List<DubinsConnector.Path> Pool = new List<DubinsConnector.Path>();
                List<WorkLane> PoolLeft = new List<WorkLane>();
                List<WorkLane> PoolRight = new List<WorkLane>();
                foreach (Double EndSlide in Slides)
                {
                    foreach (Double StartSlide in Slides)
                    {
                        // A bend is the end of the lane. Trimming it would straighten the boundary.
                        WorkLane? TrimmedLeft = (EndSlide == 0.0)
                            ? Left
                            : ((Left.ArcLengthM > 0.0) ? null : Left.TrimFromEnd(EndSlide * RadiusM));
                        WorkLane? TrimmedRight = (StartSlide == 0.0) ? Right : Right.TrimFromStart(StartSlide * RadiusM);
                        if ((TrimmedLeft == null) || (TrimmedRight == null))
                        {
                            continue;
                        }

                        AddJointCandidates(
                            Pool, PoolLeft, PoolRight, TrimmedLeft, TrimmedRight, RadiusM, BladeWidthM);
                    }
                }

                if (Pool.Count == 0)
                {
                    continue;
                }

                Int32 Chosen = PreferField(Pool, RadiusM, Inside);
                if (Chosen < 0)
                {
                    // Every trim leaves the field. The lanes stay, and the connector decides.
                    continue;
                }

                All[Index] = PoolLeft[Chosen];
                All[Index + 1] = PoolRight[Chosen];
            }

            for (Int32 Index = 0; Index < Cuts.Count; Index += 1)
            {
                Cuts[Index] = All[Index];
            }

            for (Int32 Index = 0; Index < Fills.Count; Index += 1)
            {
                Fills[Index] = All[Cuts.Count + Index];
            }
        }

        /// <summary>
        /// Writes trimmed lanes back into the list the caller owns.
        /// </summary>
        /// <param name="Target">The caller's list.</param>
        /// <param name="Source">The trimmed lanes.</param>
        private static void CopyLanes
            (
            IReadOnlyList<WorkLane> Target,
            List<WorkLane> Source
            )
        {
            List<WorkLane>? List = Target as List<WorkLane>;
            if ((List == null) || (List.Count != Source.Count))
            {
                return;
            }

            for (Int32 Index = 0; Index < List.Count; Index += 1)
            {
                List[Index] = Source[Index];
            }
        }

        /// <summary>
        /// True when the next pose is a forward lane change: same heading, ahead,
        /// and more than half a blade to the side.
        /// </summary>
        /// <param name="From">Pose the previous lane ended on.</param>
        /// <param name="To">Pose the next lane starts on.</param>
        /// <param name="BladeWidthM">Blade width. Half of this is still one lane.</param>
        /// <param name="LateralM">Sideways shift, metres. Zero when this is not a lane change.</param>
        /// <returns>True when the join should be a forward S-bend.</returns>
        private static Boolean IsForwardLaneChange
            (
            DubinsConnector.Pose From,
            DubinsConnector.Pose To,
            Double BladeWidthM,
            out Double LateralM
            )
        {
            LateralM = 0.0;
            if (BladeWidthM <= 0.0)
            {
                return false;
            }

            Double Turn = Math.Abs(DubinsConnector.WrapPi(To.HeadingRad - From.HeadingRad));
            if (Turn > LANE_CHANGE_HEADING_RAD)
            {
                return false;
            }

            Double Dx = To.X - From.X;
            Double Dy = To.Y - From.Y;
            Double Forward = (Dx * Math.Cos(From.HeadingRad)) + (Dy * Math.Sin(From.HeadingRad));
            Double Side = (Dx * -Math.Sin(From.HeadingRad)) + (Dy * Math.Cos(From.HeadingRad));
            LateralM = Math.Abs(Side);
            return (Forward > LANE_CHANGE_FORWARD_M) && (LateralM > (BladeWidthM * 0.5));
        }

        /// <summary>
        /// True when a short gap is a small heading change and at most one blade
        /// of sideways offset, so the operator steers across it in a straight line.
        /// </summary>
        /// <param name="From">Pose the previous lane ended on.</param>
        /// <param name="To">Pose the next lane starts on.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width. Zero never steers.</param>
        /// <returns>True when the connector should be a straight line.</returns>
        private static Boolean SteerAcross
            (
            DubinsConnector.Pose From,
            DubinsConnector.Pose To,
            Double RadiusM,
            Double BladeWidthM
            )
        {
            if ((BladeWidthM <= 0.0) || (RadiusM <= 0.0))
            {
                return false;
            }

            Double Gap = PoseGap(From, To);
            if (Gap >= (CurrentRules.ShortJointRadii * RadiusM))
            {
                return false;
            }

            Double Turn = Math.Abs(DubinsConnector.WrapPi(To.HeadingRad - From.HeadingRad));
            if (Turn >= (CurrentRules.SteerHeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN))
            {
                return false;
            }

            Double Dx = To.X - From.X;
            Double Dy = To.Y - From.Y;
            Double Side = (Dx * -Math.Sin(From.HeadingRad)) + (Dy * Math.Cos(From.HeadingRad));
            return Math.Abs(Side) <= (BladeWidthM + 1e-6);
        }

        /// <summary>
        /// Adds one trim's connector candidates: the straight steer, when the
        /// gap is gentle, and every Dubins word between the trimmed poses.
        /// </summary>
        /// <param name="Pool">Candidates gathered for this joint.</param>
        /// <param name="PoolLeft">Lane that produced each candidate.</param>
        /// <param name="PoolRight">Following lane that produced each candidate.</param>
        /// <param name="Left">Previous lane, already trimmed.</param>
        /// <param name="Right">Next lane, already trimmed.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        private static void AddJointCandidates
            (
            List<DubinsConnector.Path> Pool,
            List<WorkLane> PoolLeft,
            List<WorkLane> PoolRight,
            WorkLane Left,
            WorkLane Right,
            Double RadiusM,
            Double BladeWidthM
            )
        {
            if (SteerAcross(Left.End, Right.Start, RadiusM, BladeWidthM))
            {
                Pool.Add(StraightJoin(Left.End, Right.Start));
                PoolLeft.Add(Left);
                PoolRight.Add(Right);
            }

            List<DubinsConnector.Path> Words = DubinsConnector.AllWords(Left.End, Right.Start, RadiusM);
            for (Int32 Index = 0; Index < Words.Count; Index += 1)
            {
                Pool.Add(Words[Index]);
                PoolLeft.Add(Left);
                PoolRight.Add(Right);
            }
        }

        /// <summary>
        /// A straight connector between two poses. Used when a short joint is
        /// steered across.
        /// </summary>
        /// <param name="From">Departure pose.</param>
        /// <param name="To">Arrival pose.</param>
        /// <returns>One straight piece, or an empty path when the poses meet.</returns>
        private static DubinsConnector.Path StraightJoin
            (
            DubinsConnector.Pose From,
            DubinsConnector.Pose To
            )
        {
            Double Dx = To.X - From.X;
            Double Dy = To.Y - From.Y;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            List<DubinsConnector.Piece> Pieces = new List<DubinsConnector.Piece>();
            if (Length > LENGTH_TIE_M)
            {
                Pieces.Add(new DubinsConnector.Piece(DubinsConnector.PieceKind.Straight, Length, 0.0));
            }

            return new DubinsConnector.Path("S", Pieces, From, To, false, 0.0, 0.0);
        }

        /// <summary>
        /// Straight-line distance between two poses.
        /// </summary>
        /// <param name="From">Start pose.</param>
        /// <param name="To">End pose.</param>
        /// <returns>Metres.</returns>
        private static Double PoseGap
            (
            DubinsConnector.Pose From,
            DubinsConnector.Pose To
            )
        {
            Double Dx = To.X - From.X;
            Double Dy = To.Y - From.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Length of a polyline.
        /// </summary>
        /// <param name="Points">Points in order.</param>
        /// <returns>Metres.</returns>
        private static Double PolylineLength
            (
            IReadOnlyList<(Double X, Double Y)> Points
            )
        {
            Double Length = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double Dx = Points[Index].X - Points[Index - 1].X;
                Double Dy = Points[Index].Y - Points[Index - 1].Y;
                Length += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }

            return Length;
        }

        /// <summary>
        /// The part of a polyline between two stations, with the cut ends interpolated.
        /// </summary>
        /// <param name="Points">Points in order.</param>
        /// <param name="StartM">Station where the kept part starts.</param>
        /// <param name="EndM">Station where the kept part ends.</param>
        /// <returns>The kept points. Empty when the range is empty.</returns>
        private static List<(Double X, Double Y)> SlicePolyline
            (
            IReadOnlyList<(Double X, Double Y)> Points,
            Double StartM,
            Double EndM
            )
        {
            List<(Double X, Double Y)> Result = new List<(Double X, Double Y)>();
            if ((Points.Count == 0) || (EndM <= StartM))
            {
                return Result;
            }

            Double Cursor = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double Dx = Points[Index].X - Points[Index - 1].X;
                Double Dy = Points[Index].Y - Points[Index - 1].Y;
                Double Segment = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                Double Next = Cursor + Segment;
                if ((Result.Count == 0) && (Next >= StartM))
                {
                    Double T = (Segment <= 1e-9) ? 0.0 : ((StartM - Cursor) / Segment);
                    Result.Add((Points[Index - 1].X + (T * Dx), Points[Index - 1].Y + (T * Dy)));
                }

                if (Result.Count > 0)
                {
                    if (Next >= EndM)
                    {
                        Double T = (Segment <= 1e-9) ? 0.0 : ((EndM - Cursor) / Segment);
                        Result.Add((Points[Index - 1].X + (T * Dx), Points[Index - 1].Y + (T * Dy)));
                        break;
                    }

                    Result.Add(Points[Index]);
                }

                Cursor = Next;
            }

            return Result;
        }

        /// <summary>
        /// Cut and fill cells of one trip.
        /// </summary>
        /// <param name="Trip">One pass.</param>
        /// <returns>The cell indexes.</returns>
        private static HashSet<(Int32 Bx, Int32 By)> CellsOf
            (
            Pass Trip
            )
        {
            HashSet<(Int32 Bx, Int32 By)> Cells = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Trip.CutCells)
            {
                Cells.Add(Entry.Cell);
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Trip.FillCells)
            {
                Cells.Add(Entry.Cell);
            }

            return Cells;
        }

        /// <summary>
        /// For each trip, the earlier trips that share a cell and must stay ahead of it.
        /// </summary>
        /// <param name="Members">Trip indexes in planner order, one stage.</param>
        /// <param name="Cells">Cells of every trip.</param>
        /// <returns>Blockers indexed by trip.</returns>
        private static List<Int32>[] BlockersFor
            (
            List<Int32> Members,
            HashSet<(Int32 Bx, Int32 By)>[] Cells
            )
        {
            List<Int32>[] Blockers = new List<Int32>[Cells.Length];
            for (Int32 Index = 0; Index < Blockers.Length; Index += 1)
            {
                Blockers[Index] = new List<Int32>();
            }

            Dictionary<(Int32 Bx, Int32 By), Int32> Last = new Dictionary<(Int32 Bx, Int32 By), Int32>();
            foreach (Int32 Member in Members)
            {
                HashSet<Int32> Wait = new HashSet<Int32>();
                foreach ((Int32 Bx, Int32 By) Cell in Cells[Member])
                {
                    Int32 Previous;
                    if (Last.TryGetValue(Cell, out Previous) && (Previous != Member))
                    {
                        Wait.Add(Previous);
                    }

                    Last[Cell] = Member;
                }

                Blockers[Member].AddRange(Wait);
            }

            return Blockers;
        }

        /// <summary>
        /// True when every earlier overlapping trip is already in the drive order.
        /// </summary>
        /// <param name="Candidate">Trip being considered.</param>
        /// <param name="Used">Trips already placed.</param>
        /// <param name="Blockers">Earlier trips that share a cell.</param>
        /// <returns>True when the candidate may be next.</returns>
        private static Boolean Ready
            (
            Int32 Candidate,
            Boolean[] Used,
            List<Int32>[] Blockers
            )
        {
            foreach (Int32 Blocker in Blockers[Candidate])
            {
                if (!Used[Blocker])
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// True when the two trips share a cut or fill cell.
        /// </summary>
        /// <param name="Left">Cells of one trip.</param>
        /// <param name="Right">Cells of the other trip.</param>
        /// <returns>True when they overlap.</returns>
        private static Boolean SharesCells
            (
            HashSet<(Int32 Bx, Int32 By)> Left,
            HashSet<(Int32 Bx, Int32 By)> Right
            )
        {
            HashSet<(Int32 Bx, Int32 By)> Smaller = (Left.Count <= Right.Count) ? Left : Right;
            HashSet<(Int32 Bx, Int32 By)> Larger = (Left.Count <= Right.Count) ? Right : Left;
            foreach ((Int32 Bx, Int32 By) Cell in Smaller)
            {
                if (Larger.Contains(Cell))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Connectors whose path turns more than 150 degrees.
        /// </summary>
        /// <param name="Trip">One trip.</param>
        /// <returns>The count.</returns>
        private static Int32 ReversalCount
            (
            TripCourse Trip
            )
        {
            Int32 Count = 0;
            Double Limit = REVERSAL_DEG * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            foreach (CourseSegment Segment in Trip.Segments)
            {
                if (Segment.IsLane)
                {
                    continue;
                }

                Double Turn = 0.0;
                foreach (DrawnPiece Piece in Segment.Pieces)
                {
                    if (Piece.Kind != DubinsConnector.PieceKind.Straight)
                    {
                        Turn += Piece.LengthM * Piece.CurvaturePerM;
                    }
                }

                if (Turn > Limit)
                {
                    Count += 1;
                }
            }

            return Count;
        }

        /// <summary>
        /// Compass degrees to radians anticlockwise from east.
        /// </summary>
        /// <param name="HeadingDeg">Compass heading. 0 is north, 90 is east.</param>
        /// <returns>Mathematical heading.</returns>
        private static Double CompassToMath
            (
            Double HeadingDeg
            )
        {
            Double Radians = HeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            return DubinsConnector.WrapPi((Math.PI / 2.0) - Radians);
        }

        /// <summary>
        /// True when a sample is close enough to pin to the lane pose.
        /// </summary>
        private static Boolean NearPose
            (
            DubinsConnector.Pose Arrival,
            DubinsConnector.Pose End
            )
        {
            Double Dx = Arrival.X - End.X;
            Double Dy = Arrival.Y - End.Y;
            if (((Dx * Dx) + (Dy * Dy)) > 0.02 * 0.02)
            {
                return false;
            }

            return Math.Abs(DubinsConnector.WrapPi(Arrival.HeadingRad - End.HeadingRad)) <= 0.02;
        }
    }
}
