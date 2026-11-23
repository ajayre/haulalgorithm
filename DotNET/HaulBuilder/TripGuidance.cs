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
    /// A forward Dubins connector joins the chosen shapes, then a grade pass
    /// runs back over the fill.
    /// </summary>
    static public class TripGuidance
    {
        /// <summary>Speed on every arc, miles per hour. Straights keep the haul clock.</summary>
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
            public DirectionChoice
                (
                TripCourse Course,
                List<WorkLane> Cuts,
                List<WorkLane> Fills
                )
            {
                this.Course = Course;
                this.Cuts = Cuts;
                this.Fills = Fills;
            }

            /// <summary>Gets the course for the chosen directions.</summary>
            public TripCourse Course { get; }

            /// <summary>Gets the cut lanes in the chosen drive order.</summary>
            public List<WorkLane> Cuts { get; }

            /// <summary>Gets the spread lanes in the chosen drive order.</summary>
            public List<WorkLane> Fills { get; }
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
        public JobGuidance
            (
            List<TripCourse> Trips,
            Double[,] Surface,
            Double[,]? Snapshot,
            Double Efficiency,
            Dictionary<Int32, Double[,]> Snapshots
            )
        {
            this.Trips = Trips;
            this.Surface = Surface;
            this.Snapshot = Snapshot;
            this.Efficiency = Efficiency;
            this.Snapshots = Snapshots;
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
                StringBuilder Text = new StringBuilder();
                Text.AppendLine("Guidance with Dubins connectors and a grade pass");
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "Previous build {0:F3} h    this build {1:F3} h",
                    PreviousHours,
                    Charged / Constants.SECONDS_PER_HOUR));
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
                Text.AppendLine();
                Text.AppendLine("Hours by stage");
                foreach (HaulStage Stage in new HaulStage[]
                {
                    HaulStage.Flow,
                    HaulStage.Clusters,
                    HaulStage.ThinCut,
                    HaulStage.Correcting,
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

                return Text.ToString();
            }
        }

        /// <summary>
        /// Builds guidance for every trip. Cell choice is the pass list's.
        /// Lane direction is chosen so the connectors stay in the field.
        /// The grade pass is the only extra earthwork.
        /// </summary>
        /// <param name="Grid">Field grid. Its mask is the field test.</param>
        /// <param name="Passes">Trips in haul order.</param>
        /// <param name="Clock">Haul, return, cut, and spread speeds.</param>
        /// <param name="BowlCapacityM3">Scraper bowl, cubic metres.</param>
        /// <param name="BladeWidthM">Blade width. Used only to keep the existing lanes.</param>
        /// <param name="RadiusM">Turning radius, metres.</param>
        /// <param name="SnapshotTripIndex">0-based trip whose start ground is kept. Negative keeps none.</param>
        /// <returns>The job.</returns>
        static public JobGuidance Build
            (
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            TimeParams Clock,
            Double BowlCapacityM3,
            Double BladeWidthM,
            Double RadiusM,
            Int32 SnapshotTripIndex
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
                null);
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
            IReadOnlyCollection<Int32>? SnapshotIndexes
            )
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
            Int32 Count = Passes.Count;
            List<WorkLane>[] PlannedCuts = new List<WorkLane>[Count];
            List<WorkLane>[] PlannedFills = new List<WorkLane>[Count];
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                Pass Current = Passes[Index];
                PlannedCuts[Index] = LanesOf(Current.CutCells, Current.RibbonOrder, Grid, BladeWidthM, CircleFt);
                PlannedFills[Index] = LanesOf(Current.FillCells, Current.RibbonOrder, Grid, BladeWidthM, CircleFt);
            }

            TripCourse[] Courses = new TripCourse[Count];
            List<WorkLane>[] ChosenCuts = new List<WorkLane>[Count];
            List<WorkLane>[] ChosenFills = new List<WorkLane>[Count];
            // The return aims at the next trip's first cut. Choose the later
            // trip first so that pose is the direction it will actually drive.
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
                    Index + 1,
                    Passes[Index].Stage);
                Courses[Index] = Choice.Course;
                ChosenCuts[Index] = Choice.Cuts;
                ChosenFills[Index] = Choice.Fills;
            }

            Double[,] Surface = Grid.Delta();
            Double[,]? Snapshot = null;
            HashSet<Int32> KeepGround = new HashSet<Int32>();
            if ((SnapshotTripIndex >= 0) && (SnapshotTripIndex < Count))
            {
                KeepGround.Add(SnapshotTripIndex);
            }

            if (SnapshotIndexes != null)
            {
                foreach (Int32 Index in SnapshotIndexes)
                {
                    if ((Index >= 0) && (Index < Count))
                    {
                        KeepGround.Add(Index);
                    }
                }
            }

            Dictionary<Int32, Double[,]> KeptGround = new Dictionary<Int32, Double[,]>(KeepGround.Count);
            Double CarryM3 = 0.0;
            TripCourse? Previous = null;
            List<TripCourse> Trips = new List<TripCourse>(Count);
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                if (KeepGround.Contains(Index))
                {
                    // Clone before this trip's cut and fill, so the sheet shows the ground it starts on.
                    Double[,] Copy = (Double[,])Surface.Clone();
                    KeptGround[Index] = Copy;
                    if (Index == SnapshotTripIndex)
                    {
                        Snapshot = Copy;
                    }
                }

                Pass Current = Passes[Index];
                TripCourse Trip = Courses[Index];
                HashSet<(Int32 Bx, Int32 By)> CutSkip = DeferredCells(ChosenCuts[Index]);
                HashSet<(Int32 Bx, Int32 By)> FillSkip = DeferredCells(ChosenFills[Index]);
                // A bend leaves the cells it no longer sweeps. Apply them on a later trip.
                ApplyPass(Surface, Current, CutSkip, FillSkip);
                // Leftover from the previous grade rides the next spread.
                if ((CarryM3 > 0.0) && (Previous != null))
                {
                    CarryM3 = PlaceOnCells(Surface, Grid, Current.FillCells, CarryM3, Previous.Grade);
                }
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> GradeOrder =
                    GradeOrderOf(ChosenFills[Index], Current.FillCells, FillSkip);
                List<DubinsConnector.Pose>? ReturnSamples = ReturnSamplesOf(Trip);
                GradeLedger Moved = GradeCells(Surface, Grid, GradeOrder, BowlCapacityM3, ReturnSamples);
                Trip.Grade.CutM3 = Moved.CutM3;
                Trip.Grade.PlacedM3 = Moved.PlacedM3;
                Trip.Grade.BowlM3 = Moved.BowlM3;
                CarryM3 = Moved.BowlM3;
                Previous = Trip;
                Trips.Add(Trip);
            }

            Double Efficiency = Clock.FieldEfficiency;
            return new JobGuidance(Trips, Surface, Snapshot, Efficiency, KeptGround);
        }

        /// <summary>
        /// Picks lane directions and boundary bends for one trip. Every spread
        /// lane may face either way. A cut lane may face either way only when it
        /// is flat; a slope keeps the downhill direction. An end within a radius
        /// of the boundary may also bend left or right. At most four lanes on
        /// each side are tried, and at most 256 combinations. The winner has the
        /// fewest connectors outside the field, then the least time. An equal
        /// score keeps the earlier combination, which is the planned lane.
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
            HaulStage Stage
            )
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

            List<WorkLane> PreparedCuts = PrepareCuts(CutLanes, Grid, out List<Int32> FlatCuts);
            List<Int32> AllFills = new List<Int32>(FillLanes.Count);
            for (Int32 Index = 0; Index < FillLanes.Count; Index += 1)
            {
                AllFills.Add(Index);
            }

            // Straight is variant 0, so a tie keeps the planned direction.
            List<LaneVariants> CutVariants = VariantsFor(
                PreparedCuts, FlatCuts, Grid, Inside, RadiusM, BladeWidthM);
            List<LaneVariants> FillVariants = VariantsFor(
                FillLanes, AllFills, Grid, Inside, RadiusM, BladeWidthM);
            List<LaneVariants> Sides = new List<LaneVariants>(CutVariants.Count + FillVariants.Count);
            Sides.AddRange(CutVariants);
            Sides.AddRange(FillVariants);
            TrimToBudget(Sides);
            Int32[] Choice = new Int32[Sides.Count];
            DirectionChoice? Best = null;
            Int32 BestOutside = 0;
            Double BestTime = 0.0;
            Boolean More = true;
            while (More)
            {
                List<WorkLane> Cuts = Assemble(PreparedCuts, Sides, Choice, 0, CutVariants.Count);
                List<WorkLane> Fills = Assemble(FillLanes, Sides, Choice, CutVariants.Count, FillVariants.Count);
                TripCourse Course = BuildCourse(
                    Cuts,
                    Fills,
                    NextCutStart,
                    Inside,
                    Clock,
                    RadiusM,
                    LaneSampleM,
                    TripNumber,
                    Stage);
                Int32 Outside = OutsideCount(Course);
                Double Time = Course.ProductiveSeconds;
                // Fewer outside connectors wins. Equal counts take the faster course.
                // The first variant is the planned lane, so a tie leaves it alone.
                if ((Best == null)
                    || (Outside < BestOutside)
                    || ((Outside == BestOutside) && ((Time + TIME_TIE_S) < BestTime)))
                {
                    Best = new DirectionChoice(Course, Cuts, Fills);
                    BestOutside = Outside;
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
        /// <param name="Reversible">Indexes that may be driven the other way.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Inside">Field test.</param>
        /// <param name="RadiusM">Turning radius.</param>
        /// <param name="BladeWidthM">Blade width.</param>
        /// <returns>Lanes that have more than one shape.</returns>
        private static List<LaneVariants> VariantsFor
            (
            IReadOnlyList<WorkLane> Lanes,
            IReadOnlyList<Int32> Reversible,
            Grid Grid,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM,
            Double BladeWidthM
            )
        {
            List<Int32> Every = new List<Int32>(Lanes.Count);
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                Every.Add(Index);
            }

            HashSet<Int32> MayReverse = new HashSet<Int32>(Reversible);
            List<Int32> Picked = FlipCandidates(Lanes, Every, Inside, RadiusM);
            List<LaneVariants> Variants = new List<LaneVariants>();
            foreach (Int32 Index in Picked)
            {
                Boolean Reverse = MayReverse.Contains(Index);
                List<WorkLane> Options = ShapesFor(Lanes[Index], Reverse, Grid, Inside, RadiusM, BladeWidthM);
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
        /// <returns>At least the planned lane.</returns>
        private static List<WorkLane> ShapesFor
            (
            WorkLane Lane,
            Boolean AllowReverse,
            Grid Grid,
            Func<Double, Double, Boolean>? Inside,
            Double RadiusM,
            Double BladeWidthM
            )
        {
            List<WorkLane> Options = new List<WorkLane> { Lane };
            if (AllowReverse)
            {
                Options.Add(Lane.Reversed());
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
            HaulStage Stage
            )
        {
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
            DubinsConnector.Pose? Cursor = null;
            foreach (WorkLane Lane in CutLanes)
            {
                if (Cursor.HasValue)
                {
                    AddConnector(
                        Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                        ref GradeSeconds, ref EmptySeconds, Cursor.Value, Lane.Start,
                        GuidanceAction.Carry, Inside, Clock, RadiusM, TripNumber);
                }

                AddLane(Segments, Points, ref Sequence, ref CutSeconds, ref SpreadSeconds, ref GradeSeconds, Lane, GuidanceAction.Cut, LaneSampleM, Clock);
                Cursor = Lane.End;
            }

            foreach (WorkLane Lane in FillLanes)
            {
                if (Cursor.HasValue)
                {
                    AddConnector(
                        Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                        ref GradeSeconds, ref EmptySeconds, Cursor.Value, Lane.Start,
                        GuidanceAction.Carry, Inside, Clock, RadiusM, TripNumber);
                }

                AddLane(Segments, Points, ref Sequence, ref CutSeconds, ref SpreadSeconds, ref GradeSeconds, Lane, GuidanceAction.Fill, LaneSampleM, Clock);
                Cursor = Lane.End;
            }

            // Grade drives the spread lanes backwards, last one first.
            if (FillLanes.Count > 0)
            {
                List<WorkLane> GradeLanes = new List<WorkLane>();
                for (Int32 Index = FillLanes.Count - 1; Index >= 0; Index -= 1)
                {
                    GradeLanes.Add(FillLanes[Index].Reversed());
                }

                if (Cursor.HasValue)
                {
                    AddConnector(
                        Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                        ref GradeSeconds, ref EmptySeconds, Cursor.Value, GradeLanes[0].Start,
                        GuidanceAction.Turn, Inside, Clock, RadiusM, TripNumber);
                }

                for (Int32 Index = 0; Index < GradeLanes.Count; Index += 1)
                {
                    if (Index > 0)
                    {
                        AddConnector(
                            Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                            ref GradeSeconds, ref EmptySeconds, GradeLanes[Index - 1].End, GradeLanes[Index].Start,
                            GuidanceAction.Grade, Inside, Clock, RadiusM, TripNumber);
                    }

                    AddLane(
                        Segments, Points, ref Sequence, ref CutSeconds, ref SpreadSeconds, ref GradeSeconds,
                        GradeLanes[Index], GuidanceAction.Grade, LaneSampleM, Clock);
                }

                Cursor = GradeLanes[GradeLanes.Count - 1].End;
            }

            if (NextCutStart.HasValue && Cursor.HasValue)
            {
                AddConnector(
                    Segments, Points, Notes, ref Sequence, ref LoadedSeconds, ref TurnSeconds,
                    ref GradeSeconds, ref EmptySeconds, Cursor.Value, NextCutStart.Value,
                    GuidanceAction.Return, Inside, Clock, RadiusM, TripNumber);
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
        /// The grade pass uses the fill speed constant, not the haul clock.
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
                SpeedMph = Constants.DEFAULT_FILL_SPEED_MPH;
            }

            Double Seconds = 0.0;
            if (SpeedMph > 0.0)
            {
                // The straight runs at cut or fill speed. The quarter circle is
                // the same speed, and never faster than the turn speed.
                Double ArcMph = Math.Min(SpeedMph, TURN_SPEED_MPH);
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
        /// Adds a Dubins connector. Arcs use the turn speed. Straights use the
        /// loaded speed on a carry and the return speed everywhere else.
        /// </summary>
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
            if (SamePose(Start, End))
            {
                return;
            }

            DubinsConnector.Path Path = DubinsConnector.Connect(Start, End, RadiusM, Inside);
            List<DrawnPiece> Drawn = new List<DrawnPiece>();
            Double Seconds = 0.0;
            DubinsConnector.Pose Cursor = Start;
            Double StraightMph = (Action == GuidanceAction.Carry)
                ? Clock.LoadedTravelSpeedMph
                : Clock.UnloadedTravelSpeedMph;
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                List<DubinsConnector.Pose> Samples = new List<DubinsConnector.Pose> { Cursor };
                Int32 Count = Math.Max(1, (Int32)Math.Ceiling((Piece.LengthM / DubinsConnector.SAMPLE_M) - 1e-9));
                for (Int32 Step = 1; Step <= Count; Step += 1)
                {
                    Double Distance = Piece.LengthM * Step / Count;
                    Samples.Add(DubinsConnector.Advance(Cursor, Piece.Kind, Distance, RadiusM));
                }

                DubinsConnector.Pose Arrow = DubinsConnector.Advance(Cursor, Piece.Kind, Piece.LengthM * Constants.HALF_SCALE, RadiusM);
                Drawn.Add(new DrawnPiece(Piece.Kind, Piece.LengthM, Piece.CurvaturePerM, Samples, Arrow));
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
                    Seconds += Piece.LengthM / (TURN_SPEED_MPH * Constants.MPH_TO_MPS);
                }

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
                Action,
                false,
                Path.Outside,
                Path.Word,
                Path.LeadOutM,
                Path.LeadInM,
                Path.LengthM,
                Seconds,
                Drawn);
            Segments.Add(Segment);
            Notes.Add(new ConnectorNote(
                TripNumber,
                Action,
                Path.Word,
                Path.LengthM,
                Path.LeadOutM,
                Path.LeadInM,
                Path.Outside));
            AppendPoints(Points, ref Sequence, Segment);
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
            Boolean Blade = (Segment.Action == GuidanceAction.Cut)
                || (Segment.Action == GuidanceAction.Fill)
                || (Segment.Action == GuidanceAction.Grade);
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
        /// True when two poses are the same place and the same direction.
        /// </summary>
        private static Boolean SamePose
            (
            DubinsConnector.Pose Start,
            DubinsConnector.Pose End
            )
        {
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            if (((Dx * Dx) + (Dy * Dy)) > (JOINT_M * JOINT_M))
            {
                return false;
            }

            return Math.Abs(DubinsConnector.WrapPi(End.HeadingRad - Start.HeadingRad)) <= 1e-4;
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
