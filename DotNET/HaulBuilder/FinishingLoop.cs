using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// One finishing loop. Every decision prices the next flow trip, the cheapest
    /// cluster pair, the cheapest tile pair, each coverage line, and a straight
    /// line through each out-of-tolerance group. The score is seconds per bank
    /// cubic yard of error removed. Surplus and deficit use cells at the target,
    /// and only after the real lows or highs that this move can reach.
    /// </summary>
    public static class FinishingLoop
    {
        /// <summary>The loop stopped because haul error is gone, or the rest is left for planing.</summary>
        public const String PLANING = "planing";

        /// <summary>A low remains and the deficit source is at its cap.</summary>
        public const String DEFICIT_EXCEEDS_CAP = "deficit exceeds cap";

        /// <summary>A high remains and the surplus sink is at its cap.</summary>
        public const String SURPLUS_EXCEEDS_CAP = "surplus exceeds cap";

        /// <summary>A group remains and no priced move removes its haul error.</summary>
        public const String NO_CANDIDATE = "no candidate";

        /// <summary>The cheapest move is more than three times the opening median.</summary>
        public const String COST_LIMIT = "cost limit";

        /// <summary>One plane pass is assumed to shave or fill this many millimetres.</summary>
        public const Double PLANE_MAX_MM = 50.0;

        /// <summary>A move after the opening sample may cost at most this many times the median.</summary>
        public const Double MAX_RATIO = 3.0;

        /// <summary>Scores kept before the cost limit starts.</summary>
        public const Int32 MEDIAN_MOVES = 500;

        /// <summary>One visit may cut this deep. The machine max can only make it shallower.</summary>
        private const Double CUT_LIFT_M = 0.061;

        /// <summary>One visit may spread this deep. The machine max can only make it shallower.</summary>
        private const Double SPREAD_LIFT_M = 0.10;

        /// <summary>Depths below this are dust, not a lift.</summary>
        private const Double DEPTH_EPS_M = 1e-9;

        /// <summary>Error below this has left the field. The same bar rejects an empty move.</summary>
        private const Double ERROR_EPS_BCY = 1e-8;

        /// <summary>Two scores this close are the same price. The earlier kind wins.</summary>
        private const Double SCORE_EPS = 1e-9;

        /// <summary>
        /// Runs the loop on the search residual and appends the moves it keeps.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Flow">Poisson field. The mouth search reads it.</param>
        /// <param name="Ledger">Yard factors and the tolerance.</param>
        /// <param name="CutLeft">Remaining cut. Real cuts are subtracted. Deficit cuts are not.</param>
        /// <param name="FillLeft">Remaining fill. Real fills are subtracted. Surplus spread is not.</param>
        /// <param name="Fx">Smoothed flow east.</param>
        /// <param name="Fy">Smoothed flow north.</param>
        /// <param name="MaximumLoadBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="BladeWidthM">Blade width in metres. Lines are one blade apart.</param>
        /// <param name="MinimumRadiusM">Turning radius passed to the mouth search.</param>
        /// <param name="MaxDepthM">Deepest single entry the machine accepts.</param>
        /// <param name="Active">Obstacles for the mouth search.</param>
        /// <param name="FinishLeftovers">True when the mouth search may keep a fold.</param>
        /// <param name="FieldEdge">Headland cells for the mouth search.</param>
        /// <param name="GuardLimit">Mouth attempts before the ribbon gives up.</param>
        /// <param name="Blocked">Mouths that grew no cut.</param>
        /// <param name="ShortMouths">Mouths that could not fill a bowl.</param>
        /// <param name="FoldedMouths">Mouths that looped.</param>
        /// <param name="Phase">Mouth phase. The ribbon keeps it across decisions.</param>
        /// <param name="FlowSlack">How far the blade may leave the arrow.</param>
        /// <param name="CrossRidges">True when a leftover ridge may be crossed.</param>
        /// <param name="Guard">Mouth attempts so far.</param>
        /// <param name="Clock">Speeds for every candidate. The same clock prices all of them.</param>
        /// <param name="Passes">Found-order routes. Kept moves are appended.</param>
        /// <returns>The per-type totals, the final error, and the stop reason.</returns>
        internal static Report Run
            (
            Grid Grid,
            FlowField Flow,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double[,] Fx,
            Double[,] Fy,
            Double MaximumLoadBcy,
            Double BladeWidthM,
            Double MinimumRadiusM,
            Double MaxDepthM,
            ObstacleField Active,
            Boolean FinishLeftovers,
            Boolean[,] FieldEdge,
            Int32 GuardLimit,
            HashSet<(Int32 Bx, Int32 By)> Blocked,
            HashSet<(Int32 Bx, Int32 By)> ShortMouths,
            HashSet<(Int32 Bx, Int32 By)> FoldedMouths,
            ref Int32 Phase,
            ref Double FlowSlack,
            ref Boolean CrossRidges,
            ref Int32 Guard,
            TimeParams Clock,
            List<Pass> Passes
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Flow == null) throw new ArgumentNullException(nameof(Flow));
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            if (CutLeft == null) throw new ArgumentNullException(nameof(CutLeft));
            if (FillLeft == null) throw new ArgumentNullException(nameof(FillLeft));
            if (Clock == null) throw new ArgumentNullException(nameof(Clock));
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            Search Search = new Search(
                Grid,
                Flow,
                Ledger,
                CutLeft,
                FillLeft,
                Fx,
                Fy,
                MaximumLoadBcy,
                BladeWidthM,
                MinimumRadiusM,
                MaxDepthM,
                Active,
                FinishLeftovers,
                FieldEdge,
                GuardLimit,
                Blocked,
                ShortMouths,
                FoldedMouths,
                Clock,
                Passes);
            return Search.Go(ref Phase, ref FlowSlack, ref CrossRidges, ref Guard);
        }

        /// <summary>Totals for one candidate type, and the field when the loop stops.</summary>
        public sealed class Report
        {
            /// <summary>
            /// Initializes the report the loop returns.
            /// </summary>
            /// <param name="Types">One row per candidate type, in enum order.</param>
            /// <param name="Groups">Out-of-tolerance groups still on the field. Empty at planing.</param>
            /// <param name="FinalErrorBcy">Error E when the loop stopped, in bank cubic yards.</param>
            /// <param name="FinalHaulErrorBcy">Haul error when the loop stopped. Plane-fixable groups are not in it.</param>
            /// <param name="StopReason">Why the loop stopped.</param>
            /// <param name="State">Planing when haul error is done or left for the plane. Stopped on the trip backstop.</param>
            /// <param name="SurfaceM">Existing minus target after the loop, in metres. Positive is high.</param>
            /// <param name="Moves">Moves kept, in order. The score is seconds per bank cubic yard of haul error.</param>
            public Report
                (
                IReadOnlyList<TypeTotal> Types,
                IReadOnlyList<Group> Groups,
                Double FinalErrorBcy,
                Double FinalHaulErrorBcy,
                String StopReason,
                HaulSearchState State,
                Double[,] SurfaceM,
                IReadOnlyList<Taken> Moves
                )
            {
                this.Types = Types;
                this.Groups = Groups;
                this.FinalErrorBcy = FinalErrorBcy;
                this.FinalHaulErrorBcy = FinalHaulErrorBcy;
                this.StopReason = StopReason;
                this.State = State;
                this.SurfaceM = SurfaceM;
                this.Moves = Moves;
            }

            /// <summary>Gets the per-type totals.</summary>
            public IReadOnlyList<TypeTotal> Types { get; }

            /// <summary>Gets the groups that still sit outside the tolerance.</summary>
            public IReadOnlyList<Group> Groups { get; }

            /// <summary>Gets the error still on the field, in bank cubic yards.</summary>
            public Double FinalErrorBcy { get; }

            /// <summary>Gets the haul error still on the field, in bank cubic yards.</summary>
            public Double FinalHaulErrorBcy { get; }

            /// <summary>Gets the stop reason.</summary>
            public String StopReason { get; }

            /// <summary>Gets the search state the loop reached.</summary>
            public HaulSearchState State { get; }

            /// <summary>Gets the residual in metres. Invalid cells are NaN.</summary>
            public Double[,] SurfaceM { get; }

            /// <summary>Gets the moves that were kept.</summary>
            public IReadOnlyList<Taken> Moves { get; }
        }

        /// <summary>One kept move and the haul error it took off.</summary>
        public sealed class Taken
        {
            /// <summary>
            /// Initializes one kept move.
            /// </summary>
            /// <param name="Score">Seconds per bank cubic yard of haul error.</param>
            /// <param name="HaulErrorBcy">Haul error removed, in bank cubic yards.</param>
            public Taken
                (
                Double Score,
                Double HaulErrorBcy
                )
            {
                this.Score = Score;
                this.HaulErrorBcy = HaulErrorBcy;
            }

            /// <summary>Gets the score.</summary>
            public Double Score { get; }

            /// <summary>Gets the haul error removed.</summary>
            public Double HaulErrorBcy { get; }
        }

        /// <summary>Moves, yards, error, and hours for one candidate type.</summary>
        public sealed class TypeTotal
        {
            /// <summary>
            /// Initializes one type total.
            /// </summary>
            /// <param name="Kind">Which candidate this row counts.</param>
            /// <param name="Moves">Moves kept from this type.</param>
            /// <param name="BcyMoved">Bank cubic yards those moves carried.</param>
            /// <param name="ErrorRemovedBcy">Error those moves took off the field.</param>
            /// <param name="SecondsPerErrorBcy">Seconds per bank cubic yard of error. Zero when no error was removed.</param>
            /// <param name="Hours">Hours of those moves, field efficiency included.</param>
            public TypeTotal
                (
                HaulStage Kind,
                Int32 Moves,
                Double BcyMoved,
                Double ErrorRemovedBcy,
                Double SecondsPerErrorBcy,
                Double Hours
                )
            {
                this.Kind = Kind;
                this.Moves = Moves;
                this.BcyMoved = BcyMoved;
                this.ErrorRemovedBcy = ErrorRemovedBcy;
                this.SecondsPerErrorBcy = SecondsPerErrorBcy;
                this.Hours = Hours;
            }

            /// <summary>Gets the candidate type.</summary>
            public HaulStage Kind { get; }

            /// <summary>Gets how many moves of this type were kept.</summary>
            public Int32 Moves { get; }

            /// <summary>Gets the bank cubic yards moved.</summary>
            public Double BcyMoved { get; }

            /// <summary>Gets the error removed, in bank cubic yards.</summary>
            public Double ErrorRemovedBcy { get; }

            /// <summary>Gets seconds per bank cubic yard of error removed.</summary>
            public Double SecondsPerErrorBcy { get; }

            /// <summary>Gets the hours.</summary>
            public Double Hours { get; }
        }

        /// <summary>One out-of-tolerance group left when the loop stops.</summary>
        public sealed class Group
        {
            /// <summary>
            /// Initializes one remaining group.
            /// </summary>
            /// <param name="Cells">Cells in the group.</param>
            /// <param name="PeakMm">The residual furthest from the target, in millimetres. Negative is low.</param>
            /// <param name="VolumeBcy">Bank cubic yards still off the target.</param>
            /// <param name="NearestM">Metres to the nearest source or destination.</param>
            /// <param name="CheapestScore">Seconds per bank cubic yard of the cheapest move that touches the group. Infinite when none exists.</param>
            /// <param name="Reason">Why no move finished the group.</param>
            public Group
                (
                Int32 Cells,
                Double PeakMm,
                Double VolumeBcy,
                Double NearestM,
                Double CheapestScore,
                String Reason
                )
            {
                this.Cells = Cells;
                this.PeakMm = PeakMm;
                this.VolumeBcy = VolumeBcy;
                this.NearestM = NearestM;
                this.CheapestScore = CheapestScore;
                this.Reason = Reason;
            }

            /// <summary>Gets the cell count.</summary>
            public Int32 Cells { get; }

            /// <summary>Gets the peak residual in millimetres.</summary>
            public Double PeakMm { get; }

            /// <summary>Gets the bank cubic yards still off the target.</summary>
            public Double VolumeBcy { get; }

            /// <summary>Gets the metres to the nearest source or destination.</summary>
            public Double NearestM { get; }

            /// <summary>Gets the cheapest score that still touches this group.</summary>
            public Double CheapestScore { get; }

            /// <summary>Gets the reason this group remains.</summary>
            public String Reason { get; }
        }

        /// <summary>The field, the balance, and the candidate builders for one solve.</summary>
        private sealed class Search
        {
            private readonly Grid _Grid;
            private readonly FlowField _Flow;
            private readonly HaulWorkLedger _Ledger;
            private readonly Double[,] _CutLeft;
            private readonly Double[,] _FillLeft;
            private readonly Double[,] _Fx;
            private readonly Double[,] _Fy;
            private readonly Double _Bowl;
            private readonly Double _BladeM;
            private readonly Double _RadiusM;
            private readonly Double _MaxDepthM;
            private readonly Double _CutLiftM;
            private readonly Double _SpreadLiftM;
            private readonly ObstacleField _Active;
            private readonly Boolean _FinishLeftovers;
            private readonly Boolean[,] _FieldEdge;
            private readonly Int32 _GuardLimit;
            private readonly HashSet<(Int32 Bx, Int32 By)> _Blocked;
            private readonly HashSet<(Int32 Bx, Int32 By)> _ShortMouths;
            private readonly HashSet<(Int32 Bx, Int32 By)> _FoldedMouths;
            private readonly TimeParams _Clock;
            private readonly List<Pass> _Passes;
            private readonly Double[,] _R;
            private readonly Boolean[,] _Raised;
            private readonly Boolean[,] _Lowered;
            private readonly Double _TolM;
            private readonly Double _CapM;
            private readonly Double _CutPerM;
            private readonly Double _FillPerM;
            private readonly Double _CellM;
            private readonly Int32 _Rows;
            private readonly Int32 _Cols;
            private readonly Double _FlowX;
            private readonly Double _FlowY;
            private readonly Tally[] _Totals;
            private readonly Int32[,] _Stamp;
            private readonly List<Candidate> _Queue = new List<Candidate>();
            private readonly List<Taken> _Taken = new List<Taken>();
            private Double _SurplusLeft;
            private Double _DeficitLeft;
            private Double _Error;
            private Double _Haul;
            private Double _RibbonBound;
            private Double _PairBound;
            private Double _LockedMedian;
            private Int32 _Epoch;
            private Int32 _Moves;
            private Int32 _PairBuilt;
            private Int32 _TileBuilt;
            private Int32 _LineBuilt;
            private Int32 _LineEpoch;
            private Boolean _Lazy;
            private Candidate _BestRejected;
            private List<List<(Int32 Row, Int32 Col)>> _Bands;
            private Boolean _CoverageBuilt;
            private Boolean _ClusterQueued;
            private Boolean _TileQueued;
            private Boolean _LineQueued;

            /// <summary>
            /// Copies the residual and the balance. S is fixed here and is not recomputed.
            /// </summary>
            /// <param name="Grid">Field grid.</param>
            /// <param name="Flow">Poisson field.</param>
            /// <param name="Ledger">Yard factors.</param>
            /// <param name="CutLeft">Remaining cut.</param>
            /// <param name="FillLeft">Remaining fill.</param>
            /// <param name="Fx">Smoothed flow east.</param>
            /// <param name="Fy">Smoothed flow north.</param>
            /// <param name="BowlBcy">Pan capacity.</param>
            /// <param name="BladeWidthM">Blade width.</param>
            /// <param name="MinimumRadiusM">Turning radius.</param>
            /// <param name="MaxDepthM">Machine max depth.</param>
            /// <param name="Active">Obstacles.</param>
            /// <param name="FinishLeftovers">Mouth-search fold flag.</param>
            /// <param name="FieldEdge">Headland cells.</param>
            /// <param name="GuardLimit">Mouth attempt cap.</param>
            /// <param name="Blocked">Blocked mouths.</param>
            /// <param name="ShortMouths">Short mouths.</param>
            /// <param name="FoldedMouths">Folded mouths.</param>
            /// <param name="Clock">Shared clock.</param>
            /// <param name="Passes">Route list to append to.</param>
            public Search
                (
                Grid Grid,
                FlowField Flow,
                HaulWorkLedger Ledger,
                Double[,] CutLeft,
                Double[,] FillLeft,
                Double[,] Fx,
                Double[,] Fy,
                Double BowlBcy,
                Double BladeWidthM,
                Double MinimumRadiusM,
                Double MaxDepthM,
                ObstacleField Active,
                Boolean FinishLeftovers,
                Boolean[,] FieldEdge,
                Int32 GuardLimit,
                HashSet<(Int32 Bx, Int32 By)> Blocked,
                HashSet<(Int32 Bx, Int32 By)> ShortMouths,
                HashSet<(Int32 Bx, Int32 By)> FoldedMouths,
                TimeParams Clock,
                List<Pass> Passes
                )
            {
                _Grid = Grid;
                _Flow = Flow;
                _Ledger = Ledger;
                _CutLeft = CutLeft;
                _FillLeft = FillLeft;
                _Fx = Fx;
                _Fy = Fy;
                _Bowl = BowlBcy;
                _BladeM = BladeWidthM;
                _RadiusM = MinimumRadiusM;
                _MaxDepthM = MaxDepthM;
                // A visit stops at the lift, and also at the machine's own max.
                _CutLiftM = Math.Min(MaxDepthM, CUT_LIFT_M);
                _SpreadLiftM = Math.Min(MaxDepthM, SPREAD_LIFT_M);
                _Active = Active;
                _FinishLeftovers = FinishLeftovers;
                _FieldEdge = FieldEdge;
                _GuardLimit = GuardLimit;
                _Blocked = Blocked;
                _ShortMouths = ShortMouths;
                _FoldedMouths = FoldedMouths;
                _Clock = Clock;
                _Passes = Passes;
                _Rows = Grid.Height;
                _Cols = Grid.Width;
                _CellM = Grid.GridSizeM;
                _TolM = Ledger.ToleranceM;
                _CapM = Ledger.ToleranceM / 3.0;
                _CutPerM = Ledger.CutBcyPerDepthM;
                _FillPerM = Ledger.FillBcyPerDepthM;
                _R = new Double[_Rows, _Cols];
                _Raised = new Boolean[_Rows, _Cols];
                _Lowered = new Boolean[_Rows, _Cols];
                _Totals = new Tally[6];
                _Stamp = new Int32[_Rows, _Cols];
                _PairBuilt = -50;
                _TileBuilt = -50;
                _LineBuilt = -50;
                _LockedMedian = -1.0;
                Double CutBank = 0.0;
                Double FillBank = 0.0;
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (!Grid.Valid[Row, Col])
                        {
                            _R[Row, Col] = Double.NaN;
                            continue;
                        }
                        Double Residual = CutLeft[Row, Col] - FillLeft[Row, Col];
                        _R[Row, Col] = Residual;
                        if (Residual > 0.0)
                        {
                            CutBank += Residual * _CutPerM;
                        }
                        else if (Residual < 0.0)
                        {
                            FillBank += -Residual * _FillPerM;
                        }
                    }
                }
                Double Balance = CutBank - FillBank;
                // The cap is a volume, not a per-trip allowance. It is spent as moves use it.
                _SurplusLeft = Balance > 0.0 ? Balance : 0.0;
                _DeficitLeft = Balance < 0.0 ? -Balance : 0.0;
                (Double X, Double Y) Heading = MeanFlow();
                _FlowX = Heading.X;
                _FlowY = Heading.Y;
            }

            /// <summary>
            /// Takes the cheapest move until haul error is gone or nothing under the cost limit removes any.
            /// </summary>
            /// <param name="Phase">Mouth phase.</param>
            /// <param name="FlowSlack">Arrow slack.</param>
            /// <param name="CrossRidges">Ridge-crossing flag.</param>
            /// <param name="Guard">Mouth attempts.</param>
            /// <returns>The report.</returns>
            public Report Go
                (
                ref Int32 Phase,
                ref Double FlowSlack,
                ref Boolean CrossRidges,
                ref Int32 Guard
                )
            {
                _Error = Error();
                _Haul = MeasureHaul();
                _Epoch = 1;
                _Lazy = !String.Equals(
                    Environment.GetEnvironmentVariable("FINISHING_LOOP_EAGER"),
                    "1",
                    StringComparison.Ordinal);
                BoundScores();
                // A cell can take several lifts. Past this, the loop is not making progress.
                Int32 TripLimit = (_Rows * _Cols * 8) + 1;
                Int32 Trips = 0;
                while (_Haul > ERROR_EPS_BCY)
                {
                    if (Trips >= TripLimit)
                    {
                        return Finish(HaulSearchState.Stopped, "trip limit");
                    }
                    NoteProgress(Trips);
                    Candidate Best = Choose(ref Phase, ref FlowSlack, ref CrossRidges, ref Guard);
                    if (Best == null)
                    {
                        String Reason = _BestRejected != null ? COST_LIMIT : GroupReason();
                        return Finish(HaulSearchState.Planing, Reason);
                    }
                    Apply(Best);
                    _Taken.Add(new Taken(Best.Score, Best.ErrorRemoved));
                    _Passes.Add(Best.Pass);
                    Tally Total = _Totals[(Int32)Best.Kind];
                    Total.Moves += 1;
                    Total.Bcy += Best.Pass.LoadBcy;
                    Total.Error += Best.ErrorRemoved;
                    Total.Seconds += Best.Seconds;
                    _Totals[(Int32)Best.Kind] = Total;
                    _Moves += 1;
                    Trips += 1;
                }
                return Finish(HaulSearchState.Planing, PLANING);
            }

            /// <summary>
            /// Prices every candidate against the current residual and keeps the lowest score.
            /// </summary>
            /// <param name="Phase">Mouth phase.</param>
            /// <param name="FlowSlack">Arrow slack.</param>
            /// <param name="CrossRidges">Ridge-crossing flag.</param>
            /// <param name="Guard">Mouth attempts.</param>
            /// <returns>The move to apply, or null when none removes error.</returns>
            private Candidate Choose
                (
                ref Int32 Phase,
                ref Double FlowSlack,
                ref Boolean CrossRidges,
                ref Int32 Guard
                )
            {
                _BestRejected = null;
                if (_Lazy)
                {
                    return ChooseLazy(ref Phase, ref FlowSlack, ref CrossRidges, ref Guard);
                }
                Candidate Best = null;
                Consider(ref Best, FlowCandidate(ref Phase, ref FlowSlack, ref CrossRidges, ref Guard));
                Double BestScore = Best == null ? Double.PositiveInfinity : Best.Score;
                if (_RibbonBound <= BestScore + SCORE_EPS)
                {
                    EnsureBands();
                    for (Int32 CoverageIndex = 0; CoverageIndex < _Bands.Count; CoverageIndex += 1)
                    {
                        List<(Int32 Row, Int32 Col)> Line = _Bands[CoverageIndex];
                        (Double X, Double Y) Anchor = Centroid(Line);
                        Consider(ref Best, TryMove(
                            RealCuts(Line),
                            RealFills(Line),
                            Hints(Line, Sinks: true),
                            Hints(Line, Sinks: false),
                            _FlowX,
                            _FlowY,
                            Anchor,
                            HaulStage.Coverage,
                            CoverageIndex));
                    }
                    Int32 LineIndex = 0;
                    foreach (List<(Int32 Row, Int32 Col)> Members in ErrorGroups())
                    {
                        Consider(ref Best, LineCandidate(Members, LineIndex));
                        LineIndex += 1;
                    }
                }
                BestScore = Best == null ? Double.PositiveInfinity : Best.Score;
                if (_PairBound <= BestScore + SCORE_EPS)
                {
                    Consider(ref Best, PairCandidate(Clusters: true));
                    Consider(ref Best, PairCandidate(Clusters: false));
                }
                return Best;
            }

            /// <summary>
            /// Keeps the new move when it is strictly cheaper. A tie keeps the earlier kind.
            /// </summary>
            /// <param name="Best">The move kept so far.</param>
            /// <param name="Next">The move just priced.</param>
            private void Consider
                (
                ref Candidate Best,
                Candidate Next
                )
            {
                if (Next == null)
                {
                    return;
                }
                if (OverLimit(Next))
                {
                    NoteRejected(Next);
                    return;
                }
                if (Best == null)
                {
                    Best = Next;
                    return;
                }
                Double Diff = Next.Score - Best.Score;
                if (Diff < -SCORE_EPS)
                {
                    Best = Next;
                    return;
                }
                if (Diff > SCORE_EPS)
                {
                    return;
                }
                if ((Int32)Next.Kind < (Int32)Best.Kind)
                {
                    Best = Next;
                    return;
                }
                if (((Int32)Next.Kind == (Int32)Best.Kind) && (Next.Index < Best.Index))
                {
                    Best = Next;
                }
            }

            /// <summary>
            /// Peeks the next mouth trip, puts its dirt back, and prices those cells under the shared rules.
            /// The mouth itself stays advanced, the same way a refused mouth is not offered twice.
            /// </summary>
            /// <param name="Phase">Mouth phase.</param>
            /// <param name="FlowSlack">Arrow slack.</param>
            /// <param name="CrossRidges">Ridge-crossing flag.</param>
            /// <param name="Guard">Mouth attempts.</param>
            /// <returns>The priced flow move, or null.</returns>
            private Candidate FlowCandidate
                (
                ref Int32 Phase,
                ref Double FlowSlack,
                ref Boolean CrossRidges,
                ref Int32 Guard
                )
            {
                Pass Peeked = FlowRibbonSearch.TryNextFlowPass(
                    _Grid,
                    _Flow,
                    _Ledger,
                    _Bowl,
                    _BladeM,
                    _RadiusM,
                    _MaxDepthM,
                    _CutLeft,
                    _FillLeft,
                    _Fx,
                    _Fy,
                    _Active,
                    _FinishLeftovers,
                    _FieldEdge,
                    _GuardLimit,
                    _Blocked,
                    _ShortMouths,
                    _FoldedMouths,
                    ref Phase,
                    ref FlowSlack,
                    ref CrossRidges,
                    ref Guard);
                if (Peeked == null)
                {
                    return null;
                }
                List<(Int32 Row, Int32 Col)> Cuts = CellsOf(Peeked.CutCells);
                List<(Int32 Row, Int32 Col)> Fills = CellsOf(Peeked.FillCells);
                Refund(Peeked.CutCells, _CutLeft);
                Refund(Peeked.FillCells, _FillLeft);
                (Double X, Double Y) Anchor = Cuts.Count > 0 ? Centroid(Cuts) : Centroid(Fills);
                (Double X, Double Y) Direction = DirectionOf(Cuts, Fills);
                return TryMove(
                    Cuts,
                    Fills,
                    Hints(Fills, Sinks: true),
                    Hints(Cuts, Sinks: false),
                    Direction.X,
                    Direction.Y,
                    Anchor,
                    HaulStage.Flow,
                    0);
            }

            /// <summary>
            /// Prices the cheapest cluster pair, or the cheapest tile pair.
            /// A field with nothing to pair contributes no move.
            /// </summary>
            /// <param name="Clusters">True for cluster pieces. False for thin-cut tiles.</param>
            /// <returns>The priced move, or null.</returns>
            private Candidate PairCandidate
                (
                Boolean Clusters
                )
            {
                Double[,] Millimetres = new Double[_Rows, _Cols];
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        Millimetres[Row, Col] = _Grid.Valid[Row, Col] ? _R[Row, Col] * 1000.0 : 0.0;
                    }
                }
                ClusterSchedule Schedule = Clusters
                    ? ClusterHauling.CandidateField(Millimetres, _Grid, _Bowl, null, null)
                    : ThinCut.CandidateField(Millimetres, _Grid, _Bowl, null, null);
                if ((Schedule.Plan.Trips == null) || (Schedule.Plan.Trips.Count == 0))
                {
                    return null;
                }
                ResidualPairTrip Trip = Schedule.Plan.Trips[0];
                Dictionary<String, ResidualPairPiece> ById = new Dictionary<String, ResidualPairPiece>();
                for (Int32 Index = 0; Index < Schedule.Pieces.Count; Index += 1)
                {
                    ResidualPairPiece Piece = Schedule.Pieces[Index];
                    if ((Piece != null) && (Piece.Id != null))
                    {
                        ById[Piece.Id] = Piece;
                    }
                }
                List<(Int32 Row, Int32 Col)> Cuts = PieceCells(Trip.Cuts, ById);
                List<(Int32 Row, Int32 Col)> Fills = PieceCells(Trip.Fills, ById);
                (Double X, Double Y) Direction = DirectionOf(Cuts, Fills);
                return TryMove(
                    Cuts,
                    Fills,
                    Hints(Fills, Sinks: true),
                    Hints(Cuts, Sinks: false),
                    Direction.X,
                    Direction.Y,
                    Centroid(Cuts.Count > 0 ? Cuts : Fills),
                    Clusters ? HaulStage.Clusters : HaulStage.ThinCut,
                    0);
            }

            /// <summary>
            /// Prices a straight line from the group to its nearest source or destination.
            /// </summary>
            /// <param name="Members">Cells of one out-of-tolerance group.</param>
            /// <param name="Index">Stable index. An equal score keeps the earlier group.</param>
            /// <returns>The priced move, or null.</returns>
            private Candidate LineCandidate
                (
                List<(Int32 Row, Int32 Col)> Members,
                Int32 Index
                )
            {
                Boolean High = _R[Members[0].Row, Members[0].Col] > 0.0;
                (Int32 Row, Int32 Col) Seed = CentroidCell(Members);
                (Int32 Row, Int32 Col) Target = NearestComplement(Members, High);
                List<(Int32 Row, Int32 Col)> Line = Bresenham(Seed.Row, Seed.Col, Target.Row, Target.Col);
                for (Int32 IndexCell = 0; IndexCell < Members.Count; IndexCell += 1)
                {
                    if (!Line.Contains(Members[IndexCell]))
                    {
                        Line.Add(Members[IndexCell]);
                    }
                }
                (Double X, Double Y) Direction = (
                    (Target.Col - Seed.Col) * _CellM,
                    (Target.Row - Seed.Row) * _CellM);
                Double Length = Math.Sqrt((Direction.X * Direction.X) + (Direction.Y * Direction.Y));
                if (Length < 1e-9)
                {
                    Direction = (_FlowX, _FlowY);
                }
                else
                {
                    Direction = (Direction.X / Length, Direction.Y / Length);
                }
                return TryMove(
                    RealCuts(Line),
                    RealFills(Line),
                    Hints(Line, Sinks: true),
                    Hints(Line, Sinks: false),
                    Direction.X,
                    Direction.Y,
                    Centroid(Members),
                    HaulStage.Line,
                    Index);
            }

            /// <summary>
            /// Simulates one move. Real lows are filled before any surplus sink.
            /// Real highs are cut before any deficit source. A move that removes no error is dropped.
            /// </summary>
            /// <param name="CutPrefs">Cells the move may cut, real highs first in this list.</param>
            /// <param name="FillPrefs">Cells the move may fill.</param>
            /// <param name="SinkHints">At-target cells this move reaches, used before the rest of the field.</param>
            /// <param name="SourceHints">At-target cells this move may lower, used before the rest of the field.</param>
            /// <param name="DirX">Travel east. The pass is ordered along it.</param>
            /// <param name="DirY">Travel north.</param>
            /// <param name="Anchor">Where surplus and deficit measure "nearest".</param>
            /// <param name="Kind">Candidate type.</param>
            /// <param name="Index">Stable index inside the type.</param>
            /// <returns>The priced move, or null.</returns>
            private Candidate TryMove
                (
                List<(Int32 Row, Int32 Col)> CutPrefs,
                List<(Int32 Row, Int32 Col)> FillPrefs,
                List<(Int32 Row, Int32 Col)> SinkHints,
                List<(Int32 Row, Int32 Col)> SourceHints,
                Double DirX,
                Double DirY,
                (Double X, Double Y) Anchor,
                HaulStage Kind,
                Int32 Index
                )
            {
                List<Slot> Cuts = new List<Slot>();
                List<Slot> Fills = new List<Slot>();
                HashSet<(Int32 Row, Int32 Col)> Used = new HashSet<(Int32 Row, Int32 Col)>();
                Double BowlLeft = _Bowl;
                List<(Int32 Row, Int32 Col)> CutOrder = Order(CutPrefs, HighFirst: true);
                for (Int32 IndexCell = 0; IndexCell < CutOrder.Count; IndexCell += 1)
                {
                    (Int32 Row, Int32 Col) Cell = CutOrder[IndexCell];
                    if (!IsRealCut(Cell.Row, Cell.Col) || !Used.Add(Cell))
                    {
                        continue;
                    }
                    Double Depth = Math.Min(_R[Cell.Row, Cell.Col], _CutLiftM);
                    AddCut(Cuts, Cell.Row, Cell.Col, Depth, Extra: false, ref BowlLeft);
                }
                Boolean CutsExhausted = !HasRealCutLeft(CutPrefs, Cuts);
                Double CutBank = BankOf(Cuts);
                Double Place = CutBank;
                List<(Int32 Row, Int32 Col)> FillOrder = Order(FillPrefs, HighFirst: false);
                for (Int32 IndexCell = 0; IndexCell < FillOrder.Count; IndexCell += 1)
                {
                    (Int32 Row, Int32 Col) Cell = FillOrder[IndexCell];
                    if (!IsRealFill(Cell.Row, Cell.Col) || Used.Contains(Cell))
                    {
                        continue;
                    }
                    Double Room = Math.Min(-_R[Cell.Row, Cell.Col], _SpreadLiftM);
                    Double Bank = Math.Min(Room * _FillPerM, Place);
                    if (Bank <= ERROR_EPS_BCY)
                    {
                        continue;
                    }
                    Used.Add(Cell);
                    Fills.Add(new Slot(Cell.Row, Cell.Col, Bank, Extra: false));
                    Place -= Bank;
                }
                // Leftover bowl dirt goes to sinks only after the lows this move can reach.
                if ((Place > ERROR_EPS_BCY) && (_SurplusLeft > ERROR_EPS_BCY))
                {
                    Double Want = Math.Min(Place, _SurplusLeft);
                    foreach ((Int32 Row, Int32 Col) Cell in Pool(Sinks: true, Anchor, SinkHints, Used))
                    {
                        if (Want <= ERROR_EPS_BCY)
                        {
                            break;
                        }
                        Double Room = Math.Min(_CapM - _R[Cell.Row, Cell.Col], _SpreadLiftM);
                        Double Bank = Math.Min(Room * _FillPerM, Want);
                        if (Bank <= ERROR_EPS_BCY)
                        {
                            continue;
                        }
                        Used.Add(Cell);
                        Fills.Add(new Slot(Cell.Row, Cell.Col, Bank, Extra: true));
                        Want -= Bank;
                        Place -= Bank;
                    }
                }
                if (Place > ERROR_EPS_BCY)
                {
                    Trim(Cuts, Place);
                }
                // A short cut can still feed a hole from the deficit source, once no real high remains on this move.
                if (CutsExhausted && (BowlLeft > ERROR_EPS_BCY) && (_DeficitLeft > ERROR_EPS_BCY))
                {
                    Double FillWant = 0.0;
                    List<Slot> MoreFills = new List<Slot>();
                    for (Int32 IndexCell = 0; IndexCell < FillOrder.Count; IndexCell += 1)
                    {
                        (Int32 Row, Int32 Col) Cell = FillOrder[IndexCell];
                        if (!IsRealFill(Cell.Row, Cell.Col) || Used.Contains(Cell))
                        {
                            continue;
                        }
                        Double Already = 0.0;
                        for (Int32 FillIndex = 0; FillIndex < Fills.Count; FillIndex += 1)
                        {
                            if ((Fills[FillIndex].Row == Cell.Row) && (Fills[FillIndex].Col == Cell.Col))
                            {
                                Already = Fills[FillIndex].Bank;
                            }
                        }
                        Double Room = Math.Min(-_R[Cell.Row, Cell.Col], _SpreadLiftM);
                        Double Bank = (Room * _FillPerM) - Already;
                        Bank = Math.Min(Bank, BowlLeft - FillWant);
                        Bank = Math.Min(Bank, _DeficitLeft - FillWant);
                        if (Bank <= ERROR_EPS_BCY)
                        {
                            continue;
                        }
                        MoreFills.Add(new Slot(Cell.Row, Cell.Col, Bank, Extra: false));
                        FillWant += Bank;
                    }
                    if (FillWant > ERROR_EPS_BCY)
                    {
                        Double Got = 0.0;
                        List<Slot> Sources = new List<Slot>();
                        foreach ((Int32 Row, Int32 Col) Cell in Pool(Sinks: false, Anchor, SourceHints, Used))
                        {
                            if (Got >= FillWant - ERROR_EPS_BCY)
                            {
                                break;
                            }
                            Double Room = Math.Min(_R[Cell.Row, Cell.Col] + _CapM, _CutLiftM);
                            Double Bank = Math.Min(Room * _CutPerM, FillWant - Got);
                            if (Bank <= ERROR_EPS_BCY)
                            {
                                continue;
                            }
                            Used.Add(Cell);
                            Sources.Add(new Slot(Cell.Row, Cell.Col, Bank, Extra: true));
                            Got += Bank;
                        }
                        if (Got > ERROR_EPS_BCY)
                        {
                            // The hole only takes as much as the deficit source could give.
                            Double Left = Got;
                            for (Int32 FillIndex = 0; FillIndex < MoreFills.Count; FillIndex += 1)
                            {
                                Slot Slot = MoreFills[FillIndex];
                                Double Bank = Math.Min(Slot.Bank, Left);
                                if (Bank <= ERROR_EPS_BCY)
                                {
                                    continue;
                                }
                                Fills.Add(new Slot(Slot.Row, Slot.Col, Bank, Extra: false));
                                Used.Add((Slot.Row, Slot.Col));
                                Left -= Bank;
                            }
                            Cuts.AddRange(Sources);
                            if (Left > ERROR_EPS_BCY)
                            {
                                Trim(Cuts, Left);
                            }
                        }
                    }
                }
                Double FinalCut = BankOf(Cuts);
                Double FinalFill = BankOf(Fills);
                if (FinalCut > FinalFill + ERROR_EPS_BCY)
                {
                    Trim(Cuts, FinalCut - FinalFill);
                }
                else if (FinalFill > FinalCut + ERROR_EPS_BCY)
                {
                    Trim(Fills, FinalFill - FinalCut);
                }
                FinalCut = BankOf(Cuts);
                if (FinalCut <= ERROR_EPS_BCY)
                {
                    return null;
                }
                Double Removed = HaulRemoved(Cuts, Fills);
                if (Removed <= ERROR_EPS_BCY)
                {
                    return null;
                }
                Pass Built = BuildPass(Cuts, Fills, FinalCut, DirX, DirY, Kind);
                if (Built == null)
                {
                    return null;
                }
                Double Seconds = TimeModel.EstimatePassTime(_Grid, Built, _Clock);
                if (!Double.IsFinite(Seconds) || (Seconds < 0.0))
                {
                    return null;
                }
                Candidate Priced = new Candidate(
                    Built,
                    Removed,
                    Seconds,
                    Seconds / Removed,
                    Kind,
                    Index,
                    CutPrefs,
                    FillPrefs,
                    SinkHints,
                    SourceHints,
                    DirX,
                    DirY,
                    Anchor);
                Priced.Epoch = _Epoch;
                return Priced;
            }

            /// <summary>
            /// Writes the move onto the residual and the ledgers.
            /// Surplus does not become required fill, and a deficit cut does not become required cut.
            /// </summary>
            /// <param name="Move">The move that won.</param>
            private void Apply
                (
                Candidate Move
                )
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Move.Pass.CutCells)
                {
                    Int32 Row = Entry.Cell.By;
                    Int32 Col = Entry.Cell.Bx;
                    Double Before = CellError(Row, Col, _R[Row, Col], _Raised[Row, Col], _Lowered[Row, Col]);
                    Boolean Source = _Lowered[Row, Col] || !IsRealCut(Row, Col);
                    _R[Row, Col] -= Entry.DepthM;
                    if (Source)
                    {
                        _Lowered[Row, Col] = true;
                        _DeficitLeft -= Entry.DepthM * _CutPerM;
                    }
                    else
                    {
                        _CutLeft[Row, Col] = Math.Max(0.0, _CutLeft[Row, Col] - Entry.DepthM);
                    }
                    Double AfterCut = CellError(Row, Col, _R[Row, Col], _Raised[Row, Col], _Lowered[Row, Col]);
                    _Error += AfterCut - Before;
                    _Stamp[Row, Col] = _Epoch + 1;
                }
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Move.Pass.FillCells)
                {
                    Int32 Row = Entry.Cell.By;
                    Int32 Col = Entry.Cell.Bx;
                    Double Before = CellError(Row, Col, _R[Row, Col], _Raised[Row, Col], _Lowered[Row, Col]);
                    Boolean Sink = _Raised[Row, Col] || !IsRealFill(Row, Col);
                    _R[Row, Col] += Entry.DepthM;
                    if (Sink)
                    {
                        _Raised[Row, Col] = true;
                        _SurplusLeft -= Entry.DepthM * _FillPerM;
                    }
                    else
                    {
                        _FillLeft[Row, Col] = Math.Max(0.0, _FillLeft[Row, Col] - Entry.DepthM);
                    }
                    Double AfterFill = CellError(Row, Col, _R[Row, Col], _Raised[Row, Col], _Lowered[Row, Col]);
                    _Error += AfterFill - Before;
                    _Stamp[Row, Col] = _Epoch + 1;
                }
                StampCorridor(Move.Pass);
                if (_SurplusLeft < 0.0)
                {
                    _SurplusLeft = 0.0;
                }
                if (_DeficitLeft < 0.0)
                {
                    _DeficitLeft = 0.0;
                }
                _Epoch += 1;
                // Group membership can change around the cells just touched.
                _Haul = MeasureHaul();
            }

            /// <summary>
            /// Builds the pass. Cells are ordered along the travel so the clock sees one line, not a jump.
            /// </summary>
            /// <param name="Cuts">Cut slots, bank cubic yards.</param>
            /// <param name="Fills">Fill slots, bank cubic yards.</param>
            /// <param name="LoadBcy">Balanced bank cubic yards.</param>
            /// <param name="DirX">Travel east.</param>
            /// <param name="DirY">Travel north.</param>
            /// <param name="Kind">Candidate type.</param>
            /// <returns>The pass, or null when a side is empty.</returns>
            private Pass BuildPass
                (
                List<Slot> Cuts,
                List<Slot> Fills,
                Double LoadBcy,
                Double DirX,
                Double DirY,
                HaulStage Kind
                )
            {
                List<Slot> CutOrder = OrderSlots(Cuts, DirX, DirY);
                List<Slot> FillOrder = OrderSlots(Fills, DirX, DirY);
                if ((CutOrder.Count == 0) || (FillOrder.Count == 0))
                {
                    return null;
                }
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                for (Int32 Index = 0; Index < CutOrder.Count; Index += 1)
                {
                    Slot Slot = CutOrder[Index];
                    Double Depth = Slot.Bank / _CutPerM;
                    if (Depth > DEPTH_EPS_M)
                    {
                        CutCells.Add(((Slot.Col, Slot.Row), Depth));
                    }
                }
                for (Int32 Index = 0; Index < FillOrder.Count; Index += 1)
                {
                    Slot Slot = FillOrder[Index];
                    Double Depth = Slot.Bank / _FillPerM;
                    if (Depth > DEPTH_EPS_M)
                    {
                        FillCells.Add(((Slot.Col, Slot.Row), Depth));
                    }
                }
                if ((CutCells.Count == 0) || (FillCells.Count == 0))
                {
                    return null;
                }
                (Double X, Double Y) CutStart = _Grid.CellCenterXy(CutCells[0].Cell.Bx, CutCells[0].Cell.By);
                (Double X, Double Y) CutEnd = _Grid.CellCenterXy(CutCells[CutCells.Count - 1].Cell.Bx, CutCells[CutCells.Count - 1].Cell.By);
                (Double X, Double Y) FillStart = _Grid.CellCenterXy(FillCells[0].Cell.Bx, FillCells[0].Cell.By);
                (Double X, Double Y) FillEnd = _Grid.CellCenterXy(FillCells[FillCells.Count - 1].Cell.Bx, FillCells[FillCells.Count - 1].Cell.By);
                Double HaulM = Math.Sqrt(((FillStart.X - CutEnd.X) * (FillStart.X - CutEnd.X)) + ((FillStart.Y - CutEnd.Y) * (FillStart.Y - CutEnd.Y)));
                Double CutLen = Polyline(CutCells);
                Double FillLen = Polyline(FillCells);
                Double LoadV = _Clock.CutSpeedMph * Constants.MPH_TO_MPS;
                Double DumpV = _Clock.FillSpeedMph * Constants.MPH_TO_MPS;
                Pass Built = new Pass(CutCells, FillCells, LoadBcy, HaulM, CutStart, CutEnd, FillStart, FillEnd);
                Built.Stage = Kind;
                Built.StripLoadDistanceM = CutLen;
                Built.StripSpreadDistanceM = FillLen;
                Built.StripLoadTimeS = LoadV > 0.0 ? CutLen / LoadV : 0.0;
                Built.StripSpreadTimeS = DumpV > 0.0 ? FillLen / DumpV : 0.0;
                return Built;
            }

            /// <summary>Adds a cut slot when the bank still fits in the bowl.</summary>
            /// <param name="Cuts">Slots already taken.</param>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <param name="DepthM">Depth available this lift.</param>
            /// <param name="Extra">True for a deficit source.</param>
            /// <param name="BowlLeft">Bowl still open, in bank cubic yards.</param>
            private void AddCut
                (
                List<Slot> Cuts,
                Int32 Row,
                Int32 Col,
                Double DepthM,
                Boolean Extra,
                ref Double BowlLeft
                )
            {
                if ((DepthM <= DEPTH_EPS_M) || (BowlLeft <= ERROR_EPS_BCY))
                {
                    return;
                }
                Double Bank = Math.Min(DepthM * _CutPerM, BowlLeft);
                if (Bank <= ERROR_EPS_BCY)
                {
                    return;
                }
                Cuts.Add(new Slot(Row, Col, Bank, Extra));
                BowlLeft -= Bank;
            }

            /// <summary>True when a preference cell still has real cut this lift did not take.</summary>
            /// <param name="Prefs">Cut cells the move was allowed to reach.</param>
            /// <param name="Taken">Cut already in the bowl.</param>
            /// <returns>True when more real cut remains on the move.</returns>
            private Boolean HasRealCutLeft
                (
                List<(Int32 Row, Int32 Col)> Prefs,
                List<Slot> Taken
                )
            {
                for (Int32 Index = 0; Index < Prefs.Count; Index += 1)
                {
                    (Int32 Row, Int32 Col) Cell = Prefs[Index];
                    if (!IsRealCut(Cell.Row, Cell.Col))
                    {
                        continue;
                    }
                    Double Left = _R[Cell.Row, Cell.Col];
                    for (Int32 TakenIndex = 0; TakenIndex < Taken.Count; TakenIndex += 1)
                    {
                        if ((Taken[TakenIndex].Row == Cell.Row) && (Taken[TakenIndex].Col == Cell.Col))
                        {
                            Left -= Taken[TakenIndex].Bank / _CutPerM;
                        }
                    }
                    if (Left > DEPTH_EPS_M)
                    {
                        return true;
                    }
                }
                return false;
            }

            /// <summary>
            /// Haul error this move takes off. A group that falls inside the plane gate
            /// drops out whole, so the delta is not just the cells the bowl touches.
            /// </summary>
            /// <param name="Cuts">Cut slots.</param>
            /// <param name="Fills">Fill slots.</param>
            /// <returns>Bank cubic yards of haul error removed.</returns>
            private Double HaulRemoved
                (
                List<Slot> Cuts,
                List<Slot> Fills
                )
            {
                List<Snap> Snaps = new List<Snap>();
                Paint(Cuts, Fills, Snaps);
                Double After = MeasureHaul();
                Restore(Snaps);
                return _Haul - After;
            }

            /// <summary>Error of one cell. The tolerance band is not error.</summary>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <param name="ResidualM">Existing minus target, in metres.</param>
            /// <param name="Raised">True when the cell is a surplus sink, not a cut.</param>
            /// <param name="Lowered">True when the cell is a deficit source, not a hole.</param>
            /// <returns>Bank cubic yards outside the tolerance.</returns>
            private Double CellError
                (
                Int32 Row,
                Int32 Col,
                Double ResidualM,
                Boolean Raised,
                Boolean Lowered
                )
            {
                if (!_Grid.Valid[Row, Col] || Double.IsNaN(ResidualM))
                {
                    return 0.0;
                }
                if (!Raised && (ResidualM > _TolM))
                {
                    return (ResidualM - _TolM) * _CutPerM;
                }
                if (!Lowered && (ResidualM < -_TolM))
                {
                    return (-ResidualM - _TolM) * _FillPerM;
                }
                return 0.0;
            }

            /// <summary>Field error E.</summary>
            /// <returns>Bank cubic yards outside the tolerance.</returns>
            private Double Error()
            {
                Double Total = 0.0;
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        Total += CellError(Row, Col, _R[Row, Col], _Raised[Row, Col], _Lowered[Row, Col]);
                    }
                }
                return Total;
            }

            /// <summary>True when the cell is still a cut down to the target.</summary>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <returns>True when dirt can be cut here.</returns>
            private Boolean IsRealCut
                (
                Int32 Row,
                Int32 Col
                )
            {
                return _Grid.Valid[Row, Col]
                    && !_Raised[Row, Col]
                    && !Double.IsNaN(_R[Row, Col])
                    && (_R[Row, Col] > DEPTH_EPS_M);
            }

            /// <summary>True when the cell is still a hole up to the target.</summary>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <returns>True when dirt can be placed here.</returns>
            private Boolean IsRealFill
                (
                Int32 Row,
                Int32 Col
                )
            {
                return _Grid.Valid[Row, Col]
                    && !_Lowered[Row, Col]
                    && !Double.IsNaN(_R[Row, Col])
                    && (_R[Row, Col] < -DEPTH_EPS_M);
            }

            /// <summary>True when an at-target cell can still be raised by the cap.</summary>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <returns>True when surplus can land here.</returns>
            private Boolean IsSink
                (
                Int32 Row,
                Int32 Col
                )
            {
                // A cell the design called high is not a sink after it is cut to target.
                // Raising it would record fill on required cut.
                if (!AtGrade(Row, Col) || _Lowered[Row, Col] || IsRealCut(Row, Col))
                {
                    return false;
                }
                return _R[Row, Col] < (_CapM - DEPTH_EPS_M);
            }

            /// <summary>True when an at-target cell can still be lowered by the cap.</summary>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <returns>True when deficit can be taken here.</returns>
            private Boolean IsSource
                (
                Int32 Row,
                Int32 Col
                )
            {
                // Only design-grade ground. Cutting a finished high past the target breaks the ledger.
                if (!AtGrade(Row, Col) || _Raised[Row, Col] || IsRealFill(Row, Col))
                {
                    return false;
                }
                return (_R[Row, Col] > (-_CapM + DEPTH_EPS_M)) && (_R[Row, Col] <= DEPTH_EPS_M);
            }

            /// <summary>True when the design gave this cell neither cut nor fill.</summary>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <returns>True when the cell started at the target.</returns>
            private Boolean AtGrade
                (
                Int32 Row,
                Int32 Col
                )
            {
                if (!_Grid.Valid[Row, Col] || Double.IsNaN(_R[Row, Col]))
                {
                    return false;
                }
                return (_Ledger.RequiredCutDepthM[Row, Col] <= DEPTH_EPS_M)
                    && (_Ledger.RequiredFillDepthM[Row, Col] <= DEPTH_EPS_M);
            }

            /// <summary>
            /// Sink or source cells, hints first, then the nearest remaining cell.
            /// </summary>
            /// <param name="Sinks">True for surplus sinks. False for deficit sources.</param>
            /// <param name="Anchor">Point the distance is measured from.</param>
            /// <param name="Hints">Cells the move already reaches.</param>
            /// <param name="Used">Cells already in this move.</param>
            /// <returns>Cells in the order they should be used.</returns>
            private List<(Int32 Row, Int32 Col)> Pool
                (
                Boolean Sinks,
                (Double X, Double Y) Anchor,
                List<(Int32 Row, Int32 Col)> Hints,
                HashSet<(Int32 Row, Int32 Col)> Used
                )
            {
                List<(Int32 Row, Int32 Col)> Ordered = new List<(Int32 Row, Int32 Col)>();
                HashSet<(Int32 Row, Int32 Col)> Seen = new HashSet<(Int32 Row, Int32 Col)>();
                for (Int32 Index = 0; Index < Hints.Count; Index += 1)
                {
                    (Int32 Row, Int32 Col) Cell = Hints[Index];
                    if (Used.Contains(Cell) || !Seen.Add(Cell))
                    {
                        continue;
                    }
                    if (Sinks ? IsSink(Cell.Row, Cell.Col) : IsSource(Cell.Row, Cell.Col))
                    {
                        Ordered.Add(Cell);
                    }
                }
                List<(Int32 Row, Int32 Col)> Rest = new List<(Int32 Row, Int32 Col)>();
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        (Int32 Row, Int32 Col) Cell = (Row, Col);
                        if (Used.Contains(Cell) || Seen.Contains(Cell))
                        {
                            continue;
                        }
                        if (Sinks ? IsSink(Row, Col) : IsSource(Row, Col))
                        {
                            Rest.Add(Cell);
                        }
                    }
                }
                Rest.Sort(delegate ((Int32 Row, Int32 Col) Left, (Int32 Row, Int32 Col) Right)
                {
                    Int32 Distance = Distance2(Left, Anchor).CompareTo(Distance2(Right, Anchor));
                    if (Distance != 0)
                    {
                        return Distance;
                    }
                    Int32 RowOrder = Left.Row.CompareTo(Right.Row);
                    if (RowOrder != 0)
                    {
                        return RowOrder;
                    }
                    return Left.Col.CompareTo(Right.Col);
                });
                Ordered.AddRange(Rest);
                return Ordered;
            }

            /// <summary>Coverage lines along the mean flow, one blade wide, that still hold error.</summary>
            /// <returns>Each line's cells, ordered along the flow.</returns>
            private List<List<(Int32 Row, Int32 Col)>> CoverageBands()
            {
                Dictionary<Int32, List<(Int32 Row, Int32 Col)>> Bands = new Dictionary<Int32, List<(Int32 Row, Int32 Col)>>();
                List<Int32> Keys = new List<Int32>();
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (!_Grid.Valid[Row, Col])
                        {
                            continue;
                        }
                        Double Across = ((Col + 0.5) * -_FlowY) + ((Row + 0.5) * _FlowX);
                        // The blade is one cell. Flooring the across-track cell index is one pass.
                        Int32 Band = (Int32)Math.Floor(Across);
                        if (!Bands.TryGetValue(Band, out List<(Int32 Row, Int32 Col)> Cells))
                        {
                            Cells = new List<(Int32 Row, Int32 Col)>();
                            Bands[Band] = Cells;
                            Keys.Add(Band);
                        }
                        Cells.Add((Row, Col));
                    }
                }
                Keys.Sort();
                List<List<(Int32 Row, Int32 Col)>> Lines = new List<List<(Int32 Row, Int32 Col)>>();
                for (Int32 Index = 0; Index < Keys.Count; Index += 1)
                {
                    List<(Int32 Row, Int32 Col)> Cells = Bands[Keys[Index]];
                    Boolean Outside = false;
                    for (Int32 CellIndex = 0; CellIndex < Cells.Count; CellIndex += 1)
                    {
                        Double Residual = _R[Cells[CellIndex].Row, Cells[CellIndex].Col];
                        if (!Double.IsNaN(Residual) && (Math.Abs(Residual) > _TolM))
                        {
                            Outside = true;
                            break;
                        }
                    }
                    if (!Outside)
                    {
                        continue;
                    }
                    Cells.Sort(delegate ((Int32 Row, Int32 Col) Left, (Int32 Row, Int32 Col) Right)
                    {
                        Double AlongLeft = ((Left.Col + 0.5) * _FlowX) + ((Left.Row + 0.5) * _FlowY);
                        Double AlongRight = ((Right.Col + 0.5) * _FlowX) + ((Right.Row + 0.5) * _FlowY);
                        Int32 Along = AlongLeft.CompareTo(AlongRight);
                        if (Along != 0)
                        {
                            return Along;
                        }
                        Int32 RowOrder = Left.Row.CompareTo(Right.Row);
                        if (RowOrder != 0)
                        {
                            return RowOrder;
                        }
                        return Left.Col.CompareTo(Right.Col);
                    });
                    Lines.Add(Cells);
                }
                return Lines;
            }

            /// <summary>Out-of-tolerance groups, high and low kept apart. Scan order is the stable index.</summary>
            /// <returns>Each group's cells.</returns>
            private List<List<(Int32 Row, Int32 Col)>> ErrorGroups()
            {
                Boolean[,] Seen = new Boolean[_Rows, _Cols];
                List<List<(Int32 Row, Int32 Col)>> Found = new List<List<(Int32 Row, Int32 Col)>>();
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (Seen[Row, Col] || !Outside(Row, Col))
                        {
                            continue;
                        }
                        Boolean High = _R[Row, Col] > 0.0;
                        List<(Int32 Row, Int32 Col)> Members = new List<(Int32 Row, Int32 Col)>();
                        Queue<(Int32 Row, Int32 Col)> Queue = new Queue<(Int32 Row, Int32 Col)>();
                        Queue.Enqueue((Row, Col));
                        Seen[Row, Col] = true;
                        while (Queue.Count > 0)
                        {
                            (Int32 Row, Int32 Col) Cell = Queue.Dequeue();
                            Members.Add(Cell);
                            Push(Queue, Seen, Cell.Row - 1, Cell.Col, High);
                            Push(Queue, Seen, Cell.Row + 1, Cell.Col, High);
                            Push(Queue, Seen, Cell.Row, Cell.Col - 1, High);
                            Push(Queue, Seen, Cell.Row, Cell.Col + 1, High);
                        }
                        Found.Add(Members);
                    }
                }
                return Found;
            }

            /// <summary>Adds a same-sign neighbour that is still outside the tolerance.</summary>
            /// <param name="Queue">Flood queue.</param>
            /// <param name="Seen">Cells already claimed.</param>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <param name="High">True for a high group.</param>
            private void Push
                (
                Queue<(Int32 Row, Int32 Col)> Queue,
                Boolean[,] Seen,
                Int32 Row,
                Int32 Col,
                Boolean High
                )
            {
                if ((Row < 0) || (Col < 0) || (Row >= _Rows) || (Col >= _Cols) || Seen[Row, Col] || !Outside(Row, Col))
                {
                    return;
                }
                if ((_R[Row, Col] > 0.0) != High)
                {
                    return;
                }
                Seen[Row, Col] = true;
                Queue.Enqueue((Row, Col));
            }

            /// <summary>True when the cell still contributes to E.</summary>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <returns>True when the residual is outside the tolerance and is not a cap cell.</returns>
            private Boolean Outside
                (
                Int32 Row,
                Int32 Col
                )
            {
                return CellError(Row, Col, _R[Row, Col], _Raised[Row, Col], _Lowered[Row, Col]) > ERROR_EPS_BCY;
            }

            /// <summary>
            /// Nearest real complement. A high group looks for a hole, then a sink.
            /// A low group looks for a high, then a deficit source.
            /// </summary>
            /// <param name="Members">Group cells. They are not their own complement.</param>
            /// <param name="High">True when the group is above the target.</param>
            /// <returns>The complement cell, or the group centroid when the field has none.</returns>
            private (Int32 Row, Int32 Col) NearestComplement
                (
                List<(Int32 Row, Int32 Col)> Members,
                Boolean High
                )
            {
                HashSet<(Int32 Row, Int32 Col)> Inside = new HashSet<(Int32 Row, Int32 Col)>(Members);
                (Double X, Double Y) Anchor = Centroid(Members);
                (Int32 Row, Int32 Col) Best = CentroidCell(Members);
                Double BestDistance = Double.PositiveInfinity;
                Boolean Found = false;
                for (Int32 Pass = 0; Pass < 2; Pass += 1)
                {
                    for (Int32 Row = 0; Row < _Rows; Row += 1)
                    {
                        for (Int32 Col = 0; Col < _Cols; Col += 1)
                        {
                            if (Inside.Contains((Row, Col)))
                            {
                                continue;
                            }
                            Boolean Match = High
                                ? ((Pass == 0) ? IsRealFill(Row, Col) : IsSink(Row, Col))
                                : ((Pass == 0) ? IsRealCut(Row, Col) : IsSource(Row, Col));
                            if (!Match)
                            {
                                continue;
                            }
                            Double Distance = Distance2((Row, Col), Anchor);
                            if (!Found || (Distance < BestDistance - 1e-9)
                                || ((Math.Abs(Distance - BestDistance) <= 1e-9) && ((Row < Best.Row) || ((Row == Best.Row) && (Col < Best.Col)))))
                            {
                                Best = (Row, Col);
                                BestDistance = Distance;
                                Found = true;
                            }
                        }
                    }
                    if (Found)
                    {
                        return Best;
                    }
                }
                return Best;
            }

            /// <summary>The reason shared by every remaining group, or the generic stop when they differ.</summary>
            /// <returns>The stop reason.</returns>
            private String GroupReason()
            {
                List<Group> Groups = Remaining();
                if (Groups.Count == 0)
                {
                    return NO_CANDIDATE;
                }
                String Reason = Groups[0].Reason;
                for (Int32 Index = 1; Index < Groups.Count; Index += 1)
                {
                    if (Groups[Index].Reason != Reason)
                    {
                        return NO_CANDIDATE;
                    }
                }
                return Reason;
            }

            /// <summary>Builds the report, including a surface the picture can draw.</summary>
            /// <param name="State">Planing or stopped.</param>
            /// <param name="Reason">Stop reason.</param>
            /// <returns>The report.</returns>
            private Report Finish
                (
                HaulSearchState State,
                String Reason
                )
            {
                Double[,] Surface = new Double[_Rows, _Cols];
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        Surface[Row, Col] = _Grid.Valid[Row, Col] ? _R[Row, Col] : Double.NaN;
                    }
                }
                HaulStage[] Kinds = new HaulStage[]
                {
                    HaulStage.Flow,
                    HaulStage.Clusters,
                    HaulStage.ThinCut,
                    HaulStage.Coverage,
                    HaulStage.Line,
                };
                List<TypeTotal> Types = new List<TypeTotal>(Kinds.Length);
                for (Int32 Index = 0; Index < Kinds.Length; Index += 1)
                {
                    Tally Total = _Totals[(Int32)Kinds[Index]];
                    Double PerBcy = Total.Error > ERROR_EPS_BCY ? Total.Seconds / Total.Error : 0.0;
                    Types.Add(new TypeTotal(
                        Kinds[Index],
                        Total.Moves,
                        Total.Bcy,
                        Total.Error,
                        PerBcy,
                        Total.Seconds / 3600.0));
                }
                IReadOnlyList<Group> Groups = _Haul <= ERROR_EPS_BCY
                    ? (IReadOnlyList<Group>)new List<Group>()
                    : Remaining();
                return new Report(Types, Groups, _Error, _Haul, Reason, State, Surface, _Taken);
            }

            /// <summary>Every group still outside the tolerance, with why it could not be moved.</summary>
            /// <returns>The groups.</returns>
            private List<Group> Remaining()
            {
                Boolean AnyHigh = false;
                Boolean AnyLow = false;
                Boolean SinkRoom = _SurplusLeft > ERROR_EPS_BCY;
                Boolean SourceRoom = _DeficitLeft > ERROR_EPS_BCY;
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (IsRealCut(Row, Col) && (_R[Row, Col] > _TolM))
                        {
                            AnyHigh = true;
                        }
                        if (IsRealFill(Row, Col) && (_R[Row, Col] < -_TolM))
                        {
                            AnyLow = true;
                        }
                        if (IsSink(Row, Col) && ((_CapM - _R[Row, Col]) > DEPTH_EPS_M))
                        {
                            SinkRoom = SinkRoom && true;
                        }
                        if (IsSource(Row, Col) && ((_R[Row, Col] + _CapM) > DEPTH_EPS_M))
                        {
                            SourceRoom = SourceRoom && true;
                        }
                    }
                }
                // Room that no cell can accept is not room.
                Boolean AnySink = false;
                Boolean AnySource = false;
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (IsSink(Row, Col) && ((_CapM - _R[Row, Col]) > DEPTH_EPS_M))
                        {
                            AnySink = true;
                        }
                        if (IsSource(Row, Col) && ((_R[Row, Col] + _CapM) > DEPTH_EPS_M))
                        {
                            AnySource = true;
                        }
                    }
                }
                SinkRoom = (_SurplusLeft > ERROR_EPS_BCY) && AnySink;
                SourceRoom = (_DeficitLeft > ERROR_EPS_BCY) && AnySource;
                List<Group> Groups = new List<Group>();
                foreach (List<(Int32 Row, Int32 Col)> Members in CountingGroups())
                {
                    Boolean High = _R[Members[0].Row, Members[0].Col] > 0.0;
                    Double Peak = High ? Double.NegativeInfinity : Double.PositiveInfinity;
                    Double Volume = 0.0;
                    for (Int32 Index = 0; Index < Members.Count; Index += 1)
                    {
                        Double Residual = _R[Members[Index].Row, Members[Index].Col];
                        if (High)
                        {
                            Peak = Math.Max(Peak, Residual);
                            Volume += Residual * _CutPerM;
                        }
                        else
                        {
                            Peak = Math.Min(Peak, Residual);
                            Volume += -Residual * _FillPerM;
                        }
                    }
                    (Int32 Row, Int32 Col) Near = NearestComplement(Members, High);
                    (Double X, Double Y) From = Centroid(Members);
                    (Double X, Double Y) To = _Grid.CellCenterXy(Near.Col, Near.Row);
                    Double Metres = Math.Sqrt(((To.X - From.X) * (To.X - From.X)) + ((To.Y - From.Y) * (To.Y - From.Y)));
                    String Reason = NO_CANDIDATE;
                    if (High && !AnyLow && !SinkRoom)
                    {
                        Reason = SURPLUS_EXCEEDS_CAP;
                    }
                    else if (!High && !AnyHigh && !SourceRoom)
                    {
                        Reason = DEFICIT_EXCEEDS_CAP;
                    }
                    Double Cheapest = CheapestFor(Members);
                    if ((Cheapest < Double.PositiveInfinity) && OverScore(Cheapest))
                    {
                        Reason = COST_LIMIT;
                    }
                    Groups.Add(new Group(Members.Count, Peak * 1000.0, Volume, Metres, Cheapest, Reason));
                }
                return Groups;
            }

            /// <summary>Unit mean of the smoothed arrows. A flat field points east.</summary>
            /// <returns>The unit heading.</returns>
            private (Double X, Double Y) MeanFlow()
            {
                Double SumX = 0.0;
                Double SumY = 0.0;
                Int32 Count = 0;
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (!_Grid.Valid[Row, Col])
                        {
                            continue;
                        }
                        SumX += _Fx[Row, Col];
                        SumY += _Fy[Row, Col];
                        Count += 1;
                    }
                }
                Double Length = Math.Sqrt((SumX * SumX) + (SumY * SumY));
                if ((Count == 0) || (Length < 1e-9))
                {
                    return (1.0, 0.0);
                }
                return (SumX / Length, SumY / Length);
            }

            /// <summary>Real cuts in the list, highest residual first.</summary>
            /// <param name="Cells">Candidate cells.</param>
            /// <returns>The cuts.</returns>
            private List<(Int32 Row, Int32 Col)> RealCuts
                (
                List<(Int32 Row, Int32 Col)> Cells
                )
            {
                return Order(Cells, HighFirst: true);
            }

            /// <summary>Real holes in the list, deepest first.</summary>
            /// <param name="Cells">Candidate cells.</param>
            /// <returns>The holes.</returns>
            private List<(Int32 Row, Int32 Col)> RealFills
                (
                List<(Int32 Row, Int32 Col)> Cells
                )
            {
                return Order(Cells, HighFirst: false);
            }

            /// <summary>Cells in the list that can take surplus, or that can give deficit.</summary>
            /// <param name="Cells">Candidate cells.</param>
            /// <param name="Sinks">True for sinks.</param>
            /// <returns>The hints, in list order.</returns>
            private List<(Int32 Row, Int32 Col)> Hints
                (
                List<(Int32 Row, Int32 Col)> Cells,
                Boolean Sinks
                )
            {
                List<(Int32 Row, Int32 Col)> Found = new List<(Int32 Row, Int32 Col)>();
                for (Int32 Index = 0; Index < Cells.Count; Index += 1)
                {
                    (Int32 Row, Int32 Col) Cell = Cells[Index];
                    if (Sinks ? IsSink(Cell.Row, Cell.Col) : IsSource(Cell.Row, Cell.Col))
                    {
                        Found.Add(Cell);
                    }
                }
                return Found;
            }

            /// <summary>Orders cells by residual. High-first is a cut. Low-first is a hole. Ties break by row, then column.</summary>
            /// <param name="Cells">Cells to order.</param>
            /// <param name="HighFirst">True to put the highest residual first.</param>
            /// <returns>A new list.</returns>
            private List<(Int32 Row, Int32 Col)> Order
                (
                List<(Int32 Row, Int32 Col)> Cells,
                Boolean HighFirst
                )
            {
                List<(Int32 Row, Int32 Col)> Copy = new List<(Int32 Row, Int32 Col)>(Cells);
                Copy.Sort(delegate ((Int32 Row, Int32 Col) Left, (Int32 Row, Int32 Col) Right)
                {
                    Double LeftResidual = _R[Left.Row, Left.Col];
                    Double RightResidual = _R[Right.Row, Right.Col];
                    Int32 Residual = HighFirst
                        ? RightResidual.CompareTo(LeftResidual)
                        : LeftResidual.CompareTo(RightResidual);
                    if (Residual != 0)
                    {
                        return Residual;
                    }
                    Int32 RowOrder = Left.Row.CompareTo(Right.Row);
                    if (RowOrder != 0)
                    {
                        return RowOrder;
                    }
                    return Left.Col.CompareTo(Right.Col);
                });
                return Copy;
            }

            /// <summary>Orders slots along the travel.</summary>
            /// <param name="Slots">Slots to order.</param>
            /// <param name="DirX">Travel east.</param>
            /// <param name="DirY">Travel north.</param>
            /// <returns>A new list.</returns>
            private List<Slot> OrderSlots
                (
                List<Slot> Slots,
                Double DirX,
                Double DirY
                )
            {
                List<Slot> Copy = new List<Slot>(Slots);
                Copy.Sort(delegate (Slot Left, Slot Right)
                {
                    Double AlongLeft = ((Left.Col + 0.5) * DirX) + ((Left.Row + 0.5) * DirY);
                    Double AlongRight = ((Right.Col + 0.5) * DirX) + ((Right.Row + 0.5) * DirY);
                    Int32 Along = AlongLeft.CompareTo(AlongRight);
                    if (Along != 0)
                    {
                        return Along;
                    }
                    Int32 RowOrder = Left.Row.CompareTo(Right.Row);
                    if (RowOrder != 0)
                    {
                        return RowOrder;
                    }
                    return Left.Col.CompareTo(Right.Col);
                });
                return Copy;
            }

            /// <summary>Length of the cell polyline. One cell is still a blade-width of travel.</summary>
            /// <param name="Cells">Ordered cells.</param>
            /// <returns>Metres.</returns>
            private Double Polyline
                (
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
                )
            {
                if (Cells.Count == 0)
                {
                    return 0.0;
                }
                if (Cells.Count == 1)
                {
                    return _CellM;
                }
                Double Length = 0.0;
                for (Int32 Index = 1; Index < Cells.Count; Index += 1)
                {
                    (Double X, Double Y) From = _Grid.CellCenterXy(Cells[Index - 1].Cell.Bx, Cells[Index - 1].Cell.By);
                    (Double X, Double Y) To = _Grid.CellCenterXy(Cells[Index].Cell.Bx, Cells[Index].Cell.By);
                    Length += Math.Sqrt(((To.X - From.X) * (To.X - From.X)) + ((To.Y - From.Y) * (To.Y - From.Y)));
                }
                return Math.Max(Length, _CellM);
            }

            /// <summary>Mean of the cell centres.</summary>
            /// <param name="Cells">Cells.</param>
            /// <returns>The centroid, or the origin when the list is empty.</returns>
            private (Double X, Double Y) Centroid
                (
                List<(Int32 Row, Int32 Col)> Cells
                )
            {
                if (Cells.Count == 0)
                {
                    return (0.0, 0.0);
                }
                Double SumX = 0.0;
                Double SumY = 0.0;
                for (Int32 Index = 0; Index < Cells.Count; Index += 1)
                {
                    (Double X, Double Y) Center = _Grid.CellCenterXy(Cells[Index].Col, Cells[Index].Row);
                    SumX += Center.X;
                    SumY += Center.Y;
                }
                return (SumX / Cells.Count, SumY / Cells.Count);
            }

            /// <summary>The cell nearest the centroid.</summary>
            /// <param name="Cells">Cells.</param>
            /// <returns>One member cell.</returns>
            private (Int32 Row, Int32 Col) CentroidCell
                (
                List<(Int32 Row, Int32 Col)> Cells
                )
            {
                (Double X, Double Y) Center = Centroid(Cells);
                (Int32 Row, Int32 Col) Best = Cells[0];
                Double BestDistance = Double.PositiveInfinity;
                for (Int32 Index = 0; Index < Cells.Count; Index += 1)
                {
                    Double Distance = Distance2(Cells[Index], Center);
                    if (Distance < BestDistance)
                    {
                        Best = Cells[Index];
                        BestDistance = Distance;
                    }
                }
                return Best;
            }

            /// <summary>Unit direction from the cuts toward the fills. A one-sided list uses the mean flow.</summary>
            /// <param name="Cuts">Cut cells.</param>
            /// <param name="Fills">Fill cells.</param>
            /// <returns>A unit vector.</returns>
            private (Double X, Double Y) DirectionOf
                (
                List<(Int32 Row, Int32 Col)> Cuts,
                List<(Int32 Row, Int32 Col)> Fills
                )
            {
                if ((Cuts.Count == 0) || (Fills.Count == 0))
                {
                    return (_FlowX, _FlowY);
                }
                (Double X, Double Y) From = Centroid(Cuts);
                (Double X, Double Y) To = Centroid(Fills);
                Double Dx = To.X - From.X;
                Double Dy = To.Y - From.Y;
                Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Length < 1e-9)
                {
                    return (_FlowX, _FlowY);
                }
                return (Dx / Length, Dy / Length);
            }

            /// <summary>Squared metres from a cell to a point.</summary>
            /// <param name="Cell">Cell.</param>
            /// <param name="Point">Point.</param>
            /// <returns>Squared metres.</returns>
            private Double Distance2
                (
                (Int32 Row, Int32 Col) Cell,
                (Double X, Double Y) Point
                )
            {
                (Double X, Double Y) Center = _Grid.CellCenterXy(Cell.Col, Cell.Row);
                Double Dx = Center.X - Point.X;
                Double Dy = Center.Y - Point.Y;
                return (Dx * Dx) + (Dy * Dy);
            }

            /// <summary>Grid cells on the straight line, including both ends.</summary>
            /// <param name="Row0">Start row.</param>
            /// <param name="Col0">Start column.</param>
            /// <param name="Row1">End row.</param>
            /// <param name="Col1">End column.</param>
            /// <returns>The cells, in line order.</returns>
            private List<(Int32 Row, Int32 Col)> Bresenham
                (
                Int32 Row0,
                Int32 Col0,
                Int32 Row1,
                Int32 Col1
                )
            {
                List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>();
                Int32 DeltaCol = Math.Abs(Col1 - Col0);
                Int32 DeltaRow = Math.Abs(Row1 - Row0);
                Int32 StepCol = Col0 < Col1 ? 1 : -1;
                Int32 StepRow = Row0 < Row1 ? 1 : -1;
                Int32 Error = DeltaCol - DeltaRow;
                Int32 Row = Row0;
                Int32 Col = Col0;
                Int32 Guard = (_Rows + _Cols) * 4;
                while (Guard > 0)
                {
                    Guard -= 1;
                    if ((Row >= 0) && (Col >= 0) && (Row < _Rows) && (Col < _Cols) && _Grid.Valid[Row, Col])
                    {
                        Cells.Add((Row, Col));
                    }
                    if ((Row == Row1) && (Col == Col1))
                    {
                        break;
                    }
                    Int32 Twice = Error * 2;
                    if (Twice > -DeltaRow)
                    {
                        Error -= DeltaRow;
                        Col += StepCol;
                    }
                    if (Twice < DeltaCol)
                    {
                        Error += DeltaCol;
                        Row += StepRow;
                    }
                }
                return Cells;
            }

            /// <summary>Piece cells for the legs of one pair, in leg order.</summary>
            /// <param name="Legs">Cut or fill legs.</param>
            /// <param name="ById">Pieces by id.</param>
            /// <returns>The cells.</returns>
            private static List<(Int32 Row, Int32 Col)> PieceCells
                (
                IReadOnlyList<ResidualPairLeg> Legs,
                Dictionary<String, ResidualPairPiece> ById
                )
            {
                List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>();
                if (Legs == null)
                {
                    return Cells;
                }
                for (Int32 Index = 0; Index < Legs.Count; Index += 1)
                {
                    if (!ById.TryGetValue(Legs[Index].Id, out ResidualPairPiece Piece) || (Piece.Cells == null))
                    {
                        continue;
                    }
                    for (Int32 CellIndex = 0; CellIndex < Piece.Cells.Count; CellIndex += 1)
                    {
                        (Int32 Row, Int32 Col) Cell = Piece.Cells[CellIndex];
                        Cells.Add((Cell.Row, Cell.Col));
                    }
                }
                return Cells;
            }

            /// <summary>Pass cells as row and column. Depth is ignored. The simulator cuts to the target itself.</summary>
            /// <param name="Cells">Pass cells.</param>
            /// <returns>The coordinates.</returns>
            private static List<(Int32 Row, Int32 Col)> CellsOf
                (
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
                )
            {
                List<(Int32 Row, Int32 Col)> Found = new List<(Int32 Row, Int32 Col)>();
                for (Int32 Index = 0; Index < Cells.Count; Index += 1)
                {
                    Found.Add((Cells[Index].Cell.By, Cells[Index].Cell.Bx));
                }
                return Found;
            }

            /// <summary>Puts a peeked mouth's dirt back. The finishing move, if it wins, subtracts its own depths.</summary>
            /// <param name="Cells">Cells the mouth took.</param>
            /// <param name="Remaining">Cut or fill ledger.</param>
            private static void Refund
                (
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
                Double[,] Remaining
                )
            {
                for (Int32 Index = 0; Index < Cells.Count; Index += 1)
                {
                    Remaining[Cells[Index].Cell.By, Cells[Index].Cell.Bx] += Cells[Index].DepthM;
                }
            }

            /// <summary>Drops bank off the tail until the two sides match.</summary>
            /// <param name="Slots">The heavy side.</param>
            /// <param name="Bank">Bank cubic yards to drop.</param>
            private static void Trim
                (
                List<Slot> Slots,
                Double Bank
                )
            {
                Double Left = Bank;
                for (Int32 Index = Slots.Count - 1; (Index >= 0) && (Left > ERROR_EPS_BCY); Index -= 1)
                {
                    Slot Slot = Slots[Index];
                    Double Take = Math.Min(Slot.Bank, Left);
                    Slot.Bank -= Take;
                    Left -= Take;
                    if (Slot.Bank <= ERROR_EPS_BCY)
                    {
                        Slots.RemoveAt(Index);
                    }
                    else
                    {
                        Slots[Index] = Slot;
                    }
                }
            }

            /// <summary>Sums slot bank.</summary>
            /// <param name="Slots">Slots.</param>
            /// <returns>Bank cubic yards.</returns>
            private static Double BankOf
                (
                List<Slot> Slots
                )
            {
                Double Total = 0.0;
                for (Int32 Index = 0; Index < Slots.Count; Index += 1)
                {
                    Total += Slots[Index].Bank;
                }
                return Total;
            }

            /// <summary>One cell's bank on this move. Extra marks a cap cell rather than real cut or fill.</summary>
            private struct Slot
            {
                /// <summary>Initializes a slot.</summary>
                /// <param name="Row">Row.</param>
                /// <param name="Col">Column.</param>
                /// <param name="Bank">Bank cubic yards.</param>
                /// <param name="Extra">True for a surplus sink or a deficit source.</param>
                public Slot
                    (
                    Int32 Row,
                    Int32 Col,
                    Double Bank,
                    Boolean Extra
                    )
                {
                    this.Row = Row;
                    this.Col = Col;
                    this.Bank = Bank;
                    this.Extra = Extra;
                }

                /// <summary>Gets or sets the row.</summary>
                public Int32 Row;

                /// <summary>Gets or sets the column.</summary>
                public Int32 Col;

                /// <summary>Gets or sets the bank cubic yards.</summary>
                public Double Bank;

                /// <summary>Gets or sets whether this slot is cap ground.</summary>
                public Boolean Extra;
            }

            /// <summary>Prices the queue and keeps the clean cheapest move.</summary>
            /// <param name="Phase">Mouth phase.</param>
            /// <param name="FlowSlack">Arrow slack.</param>
            /// <param name="CrossRidges">Ridge-crossing flag.</param>
            /// <param name="Guard">Mouth attempts.</param>
            /// <returns>The move to apply, or null.</returns>
            private Candidate ChooseLazy
                (
                ref Int32 Phase,
                ref Double FlowSlack,
                ref Boolean CrossRidges,
                ref Int32 Guard
                )
            {
                RefreshFlow(ref Phase, ref FlowSlack, ref CrossRidges, ref Guard);
                RefreshCoverage();
                RefreshLines();
                Double Best = BestQueued();
                RefreshPair(Clusters: true, ref _PairBuilt, Best);
                Best = BestQueued();
                RefreshPair(Clusters: false, ref _TileBuilt, Best);
                return PopClean();
            }

            /// <summary>Appends a progress line while FINISHING_LOOP_PROGRESS is 1.</summary>
            /// <param name="Trips">Moves already applied.</param>
            private void NoteProgress
                (
                Int32 Trips
                )
            {
                if ((Trips % 25) != 0)
                {
                    return;
                }
                if (!String.Equals(Environment.GetEnvironmentVariable("FINISHING_LOOP_PROGRESS"), "1", StringComparison.Ordinal))
                {
                    return;
                }
                String Line = DateTime.UtcNow.ToString("o", System.Globalization.CultureInfo.InvariantCulture)
                    + " trip " + Trips.ToString(System.Globalization.CultureInfo.InvariantCulture)
                    + " haul " + _Haul.ToString("R", System.Globalization.CultureInfo.InvariantCulture)
                    + Environment.NewLine;
                File.AppendAllText(Path.Combine(Path.GetTempPath(), "finishing-loop-shop2-progress.txt"), Line);
            }

            /// <summary>Fixed cycle at zero haul, divided by a full bowl and by a quarter bowl.</summary>
            private void BoundScores()
            {
                Double LoadV = _Clock.CutSpeedMph * Constants.MPH_TO_MPS;
                Double DumpV = _Clock.FillSpeedMph * Constants.MPH_TO_MPS;
                Double Cell = Math.Max(_CellM, 1e-6);
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((0, 0), 0.01),
                };
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((Math.Min(1, _Cols - 1), 0), 0.01),
                };
                (Double X, Double Y) CutAt = _Grid.CellCenterXy(Cuts[0].Cell.Bx, Cuts[0].Cell.By);
                (Double X, Double Y) FillAt = _Grid.CellCenterXy(Fills[0].Cell.Bx, Fills[0].Cell.By);
                Pass Sample = new Pass(Cuts, Fills, _Bowl, 0.0, CutAt, CutAt, FillAt, FillAt);
                Sample.StripLoadDistanceM = Cell;
                Sample.StripSpreadDistanceM = Cell;
                Sample.StripLoadTimeS = LoadV > 0.0 ? Cell / LoadV : 0.0;
                Sample.StripSpreadTimeS = DumpV > 0.0 ? Cell / DumpV : 0.0;
                Double Seconds = TimeModel.EstimatePassTime(_Grid, Sample, _Clock);
                if (!Double.IsFinite(Seconds) || (Seconds < 0.0))
                {
                    Seconds = 0.0;
                }
                _RibbonBound = _Bowl > 0.0 ? Seconds / _Bowl : 0.0;
                // Pairs under a quarter bowl are not offered, so their floor is that payload.
                Double Floor = ResidualPairing.PARTIAL_BOWL_FRACTION * _Bowl;
                _PairBound = Floor > 0.0 ? Seconds / Floor : _RibbonBound;
            }

            /// <summary>True when this score is above the opening median times <see cref="MAX_RATIO"/>.</summary>
            /// <param name="Move">Priced move.</param>
            /// <returns>True when the cost limit rejects it.</returns>
            private Boolean OverLimit
                (
                Candidate Move
                )
            {
                return (Move != null) && OverScore(Move.Score);
            }

            /// <summary>True when a score is above the cost limit. The first 500 moves have no limit.</summary>
            /// <param name="Score">Seconds per bank cubic yard.</param>
            /// <returns>True when the limit is active and the score is over it.</returns>
            private Boolean OverScore
                (
                Double Score
                )
            {
                if (_Taken.Count < MEDIAN_MOVES)
                {
                    return false;
                }
                if (_LockedMedian < 0.0)
                {
                    Double[] Copy = new Double[MEDIAN_MOVES];
                    for (Int32 Index = 0; Index < MEDIAN_MOVES; Index += 1)
                    {
                        Copy[Index] = _Taken[Index].Score;
                    }
                    Array.Sort(Copy);
                    _LockedMedian = 0.5 * (Copy[249] + Copy[250]);
                }
                return Score > (_LockedMedian * MAX_RATIO) + SCORE_EPS;
            }

            /// <summary>Keeps the cheapest move the cost limit rejected.</summary>
            /// <param name="Move">Rejected move.</param>
            private void NoteRejected
                (
                Candidate Move
                )
            {
                if (Move == null)
                {
                    return;
                }
                if ((_BestRejected == null) || (Move.Score < _BestRejected.Score - SCORE_EPS))
                {
                    _BestRejected = Move;
                }
            }

            /// <summary>Puts one flow trip on the queue. The mouth advances, matching a full reprice.</summary>
            /// <param name="Phase">Mouth phase.</param>
            /// <param name="FlowSlack">Arrow slack.</param>
            /// <param name="CrossRidges">Ridge-crossing flag.</param>
            /// <param name="Guard">Mouth attempts.</param>
            private void RefreshFlow
                (
                ref Int32 Phase,
                ref Double FlowSlack,
                ref Boolean CrossRidges,
                ref Int32 Guard
                )
            {
                RemoveKind(HaulStage.Flow);
                Candidate Next = FlowCandidate(ref Phase, ref FlowSlack, ref CrossRidges, ref Guard);
                if (Next == null)
                {
                    return;
                }
                Next.Watch = Union(Next.CutPrefs, Next.FillPrefs);
                Push(Next);
            }

            /// <summary>Builds each coverage line once. A later pop reprices a line that crossed changed dirt.</summary>
            private void RefreshCoverage()
            {
                if (_CoverageBuilt)
                {
                    return;
                }
                EnsureBands();
                if (_RibbonBound > BestQueued() + SCORE_EPS)
                {
                    return;
                }
                for (Int32 Index = 0; Index < _Bands.Count; Index += 1)
                {
                    List<(Int32 Row, Int32 Col)> Line = _Bands[Index];
                    (Double X, Double Y) Anchor = Centroid(Line);
                    Candidate Next = TryMove(
                        RealCuts(Line),
                        RealFills(Line),
                        Hints(Line, Sinks: true),
                        Hints(Line, Sinks: false),
                        _FlowX,
                        _FlowY,
                        Anchor,
                        HaulStage.Coverage,
                        Index);
                    if (Next == null)
                    {
                        continue;
                    }
                    Next.Watch = Line;
                    Push(Next);
                }
                _CoverageBuilt = true;
            }

            /// <summary>Rebuilds group lines when the queue is empty, 50 moves have passed, or a tenth of the cells are dirty.</summary>
            private void RefreshLines()
            {
                Boolean StillThere = HasKind(HaulStage.Line);
                Boolean Consumed = _LineQueued && !StillThere;
                Boolean Due = ((_Moves - _LineBuilt) >= 50)
                    || Consumed
                    || (DirtyOutsideFraction() > 0.10);
                if (!Due)
                {
                    return;
                }
                if (_RibbonBound > BestQueued() + SCORE_EPS)
                {
                    return;
                }
                RemoveKind(HaulStage.Line);
                Int32 Index = 0;
                foreach (List<(Int32 Row, Int32 Col)> Members in ErrorGroups())
                {
                    Candidate Next = LineCandidate(Members, Index);
                    if (Next != null)
                    {
                        Next.Watch = Members;
                        Push(Next);
                    }
                    Index += 1;
                }
                _LineBuilt = _Moves;
                _LineEpoch = _Epoch;
                _LineQueued = HasKind(HaulStage.Line);
            }

            /// <summary>Rebuilds one pairing type on the same schedule as the group lines.</summary>
            /// <param name="Clusters">True for clusters. False for tiles.</param>
            /// <param name="Built">Move index of the last rebuild.</param>
            /// <param name="Best">Best score already on the queue.</param>
            private void RefreshPair
                (
                Boolean Clusters,
                ref Int32 Built,
                Double Best
                )
            {
                if (_PairBound > Best + SCORE_EPS)
                {
                    return;
                }
                HaulStage Kind = Clusters ? HaulStage.Clusters : HaulStage.ThinCut;
                Candidate Held = FindKind(Kind);
                Boolean WasQueued = Clusters ? _ClusterQueued : _TileQueued;
                Boolean Consumed = WasQueued && (Held == null);
                Boolean Due = ((_Moves - Built) >= 50)
                    || Consumed
                    || ((Held != null) && (DirtyFraction(Held.Watch, Held.Epoch) > 0.10));
                if (!Due)
                {
                    return;
                }
                RemoveKind(Kind);
                Candidate Next = PairCandidate(Clusters);
                if (Next != null)
                {
                    Next.Watch = Union(Next.CutPrefs, Next.FillPrefs);
                    Push(Next);
                }
                Built = _Moves;
                if (Clusters)
                {
                    _ClusterQueued = FindKind(Kind) != null;
                }
                else
                {
                    _TileQueued = FindKind(Kind) != null;
                }
            }

            /// <summary>Pops until the best clean affordable move is on top.</summary>
            /// <returns>That move, or null when none is under the limit.</returns>
            private Candidate PopClean()
            {
                while (_Queue.Count > 0)
                {
                    Candidate Top = _Queue[0];
                    if (Stale(Top))
                    {
                        _Queue.RemoveAt(0);
                        Candidate Fresh = TryMove(
                            Top.CutPrefs,
                            Top.FillPrefs,
                            Top.SinkHints,
                            Top.SourceHints,
                            Top.DirX,
                            Top.DirY,
                            Top.Anchor,
                            Top.Kind,
                            Top.Index);
                        if (Fresh != null)
                        {
                            Fresh.Watch = Top.Watch;
                            Push(Fresh);
                        }
                        continue;
                    }
                    if (OverLimit(Top))
                    {
                        NoteRejected(Top);
                        return null;
                    }
                    _Queue.RemoveAt(0);
                    return Top;
                }
                return null;
            }

            /// <summary>Inserts by score, then kind, then index.</summary>
            /// <param name="Move">Priced move.</param>
            private void Push
                (
                Candidate Move
                )
            {
                if (Move == null)
                {
                    return;
                }
                if (OverLimit(Move))
                {
                    NoteRejected(Move);
                    return;
                }
                Move.Epoch = _Epoch;
                Int32 At = _Queue.Count;
                for (Int32 Index = 0; Index < _Queue.Count; Index += 1)
                {
                    if (Better(Move, _Queue[Index]))
                    {
                        At = Index;
                        break;
                    }
                }
                _Queue.Insert(At, Move);
            }

            /// <summary>True when Left should be taken before Right.</summary>
            /// <param name="Left">New move.</param>
            /// <param name="Right">Queued move.</param>
            /// <returns>True when Left sorts first.</returns>
            private static Boolean Better
                (
                Candidate Left,
                Candidate Right
                )
            {
                Double Diff = Left.Score - Right.Score;
                if (Diff < -SCORE_EPS)
                {
                    return true;
                }
                if (Diff > SCORE_EPS)
                {
                    return false;
                }
                if ((Int32)Left.Kind != (Int32)Right.Kind)
                {
                    return (Int32)Left.Kind < (Int32)Right.Kind;
                }
                return Left.Index < Right.Index;
            }

            /// <summary>Best score on the queue, or infinity when it is empty.</summary>
            /// <returns>The score.</returns>
            private Double BestQueued()
            {
                if (_Queue.Count == 0)
                {
                    return Double.PositiveInfinity;
                }
                return _Queue[0].Score;
            }

            /// <summary>True when any watched cell changed after this price.</summary>
            /// <param name="Move">Queued move.</param>
            /// <returns>True when it must be priced again.</returns>
            private Boolean Stale
                (
                Candidate Move
                )
            {
                if (Move == null)
                {
                    return false;
                }
                if (WatchTouches(Move.Watch, Move.Epoch))
                {
                    return true;
                }
                if (WatchTouches(Move.CutPrefs, Move.Epoch))
                {
                    return true;
                }
                if (WatchTouches(Move.FillPrefs, Move.Epoch))
                {
                    return true;
                }
                if ((Move.Pass.CutCells.Count == 0) || (Move.Pass.FillCells.Count == 0))
                {
                    return false;
                }
                (Int32 Bx, Int32 By) Cut = Move.Pass.CutCells[Move.Pass.CutCells.Count - 1].Cell;
                (Int32 Bx, Int32 By) Fill = Move.Pass.FillCells[0].Cell;
                List<(Int32 Row, Int32 Col)> Corridor = Bresenham(Cut.By, Cut.Bx, Fill.By, Fill.Bx);
                return WatchTouches(Corridor, Move.Epoch);
            }

            /// <summary>True when a listed cell was stamped after the price.</summary>
            /// <param name="Cells">Cells to test.</param>
            /// <param name="Epoch">Epoch of the price.</param>
            /// <returns>True when one of them is dirty.</returns>
            private Boolean WatchTouches
                (
                List<(Int32 Row, Int32 Col)> Cells,
                Int32 Epoch
                )
            {
                if (Cells == null)
                {
                    return false;
                }
                for (Int32 Index = 0; Index < Cells.Count; Index += 1)
                {
                    (Int32 Row, Int32 Col) Cell = Cells[Index];
                    if ((Cell.Row < 0) || (Cell.Col < 0) || (Cell.Row >= _Rows) || (Cell.Col >= _Cols))
                    {
                        continue;
                    }
                    if (_Stamp[Cell.Row, Cell.Col] > Epoch)
                    {
                        return true;
                    }
                }
                return false;
            }

            /// <summary>Drops every queued move of one type.</summary>
            /// <param name="Kind">Candidate type.</param>
            private void RemoveKind
                (
                HaulStage Kind
                )
            {
                for (Int32 Index = _Queue.Count - 1; Index >= 0; Index -= 1)
                {
                    if (_Queue[Index].Kind == Kind)
                    {
                        _Queue.RemoveAt(Index);
                    }
                }
            }

            /// <summary>Drops one indexed move of a type.</summary>
            /// <param name="Kind">Candidate type.</param>
            /// <param name="Which">Stable index.</param>
            private void RemoveKindIndex
                (
                HaulStage Kind,
                Int32 Which
                )
            {
                for (Int32 Index = _Queue.Count - 1; Index >= 0; Index -= 1)
                {
                    if ((_Queue[Index].Kind == Kind) && (_Queue[Index].Index == Which))
                    {
                        _Queue.RemoveAt(Index);
                    }
                }
            }

            /// <summary>True when the queue holds this type.</summary>
            /// <param name="Kind">Candidate type.</param>
            /// <returns>True when at least one is queued.</returns>
            private Boolean HasKind
                (
                HaulStage Kind
                )
            {
                return FindKind(Kind) != null;
            }

            /// <summary>The first queued move of a type.</summary>
            /// <param name="Kind">Candidate type.</param>
            /// <returns>The move, or null.</returns>
            private Candidate FindKind
                (
                HaulStage Kind
                )
            {
                for (Int32 Index = 0; Index < _Queue.Count; Index += 1)
                {
                    if (_Queue[Index].Kind == Kind)
                    {
                        return _Queue[Index];
                    }
                }
                return null;
            }

            /// <summary>The queued move of a type and index.</summary>
            /// <param name="Kind">Candidate type.</param>
            /// <param name="Which">Stable index.</param>
            /// <returns>The move, or null.</returns>
            private Candidate Find
                (
                HaulStage Kind,
                Int32 Which
                )
            {
                for (Int32 Index = 0; Index < _Queue.Count; Index += 1)
                {
                    if ((_Queue[Index].Kind == Kind) && (_Queue[Index].Index == Which))
                    {
                        return _Queue[Index];
                    }
                }
                return null;
            }

            /// <summary>Builds the coverage geometry once. Later prices reuse these cells.</summary>
            private void EnsureBands()
            {
                if (_Bands != null)
                {
                    return;
                }
                Dictionary<Int32, List<(Int32 Row, Int32 Col)>> Bands = new Dictionary<Int32, List<(Int32 Row, Int32 Col)>>();
                List<Int32> Keys = new List<Int32>();
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (!_Grid.Valid[Row, Col])
                        {
                            continue;
                        }
                        Double Across = ((Col + 0.5) * -_FlowY) + ((Row + 0.5) * _FlowX);
                        Int32 Band = (Int32)Math.Floor(Across);
                        if (!Bands.TryGetValue(Band, out List<(Int32 Row, Int32 Col)> Cells))
                        {
                            Cells = new List<(Int32 Row, Int32 Col)>();
                            Bands[Band] = Cells;
                            Keys.Add(Band);
                        }
                        Cells.Add((Row, Col));
                    }
                }
                Keys.Sort();
                _Bands = new List<List<(Int32 Row, Int32 Col)>>();
                for (Int32 Index = 0; Index < Keys.Count; Index += 1)
                {
                    List<(Int32 Row, Int32 Col)> Cells = Bands[Keys[Index]];
                    Cells.Sort(delegate ((Int32 Row, Int32 Col) Left, (Int32 Row, Int32 Col) Right)
                    {
                        Double AlongLeft = ((Left.Col + 0.5) * _FlowX) + ((Left.Row + 0.5) * _FlowY);
                        Double AlongRight = ((Right.Col + 0.5) * _FlowX) + ((Right.Row + 0.5) * _FlowY);
                        Int32 Along = AlongLeft.CompareTo(AlongRight);
                        if (Along != 0)
                        {
                            return Along;
                        }
                        Int32 RowOrder = Left.Row.CompareTo(Right.Row);
                        if (RowOrder != 0)
                        {
                            return RowOrder;
                        }
                        return Left.Col.CompareTo(Right.Col);
                    });
                    _Bands.Add(Cells);
                }
            }

            /// <summary>Share of out-of-tolerance cells stamped in the current epoch.</summary>
            /// <returns>A fraction from 0 to 1.</returns>
            private Double DirtyOutsideFraction()
            {
                Int32 OutsideCount = 0;
                Int32 Dirty = 0;
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (!Outside(Row, Col))
                        {
                            continue;
                        }
                        OutsideCount += 1;
                        if (_Stamp[Row, Col] > _LineEpoch)
                        {
                            Dirty += 1;
                        }
                    }
                }
                if (OutsideCount == 0)
                {
                    return 0.0;
                }
                return (Double)Dirty / OutsideCount;
            }

            /// <summary>Share of a watch list stamped after it was priced.</summary>
            /// <param name="Cells">Cells the generator owns.</param>
            /// <param name="PricedEpoch">Epoch of that price.</param>
            /// <returns>A fraction from 0 to 1.</returns>
            private Double DirtyFraction
                (
                List<(Int32 Row, Int32 Col)> Cells,
                Int32 PricedEpoch
                )
            {
                if ((Cells == null) || (Cells.Count == 0))
                {
                    return 0.0;
                }
                Int32 Dirty = 0;
                for (Int32 Index = 0; Index < Cells.Count; Index += 1)
                {
                    (Int32 Row, Int32 Col) Cell = Cells[Index];
                    if ((Cell.Row < 0) || (Cell.Col < 0) || (Cell.Row >= _Rows) || (Cell.Col >= _Cols))
                    {
                        continue;
                    }
                    if (_Stamp[Cell.Row, Cell.Col] > PricedEpoch)
                    {
                        Dirty += 1;
                    }
                }
                return (Double)Dirty / Cells.Count;
            }

            /// <summary>Stamps the straight haul from the last cut to the first fill.</summary>
            /// <param name="Built">The pass just applied.</param>
            private void StampCorridor
                (
                Pass Built
                )
            {
                if ((Built.CutCells.Count == 0) || (Built.FillCells.Count == 0))
                {
                    return;
                }
                (Int32 Bx, Int32 By) Cut = Built.CutCells[Built.CutCells.Count - 1].Cell;
                (Int32 Bx, Int32 By) Fill = Built.FillCells[0].Cell;
                List<(Int32 Row, Int32 Col)> Corridor = Bresenham(Cut.By, Cut.Bx, Fill.By, Fill.Bx);
                for (Int32 Index = 0; Index < Corridor.Count; Index += 1)
                {
                    _Stamp[Corridor[Index].Row, Corridor[Index].Col] = _Epoch + 1;
                }
            }

            /// <summary>Cheapest score among the rejected move and a fresh line through the group.</summary>
            /// <param name="Members">Group cells.</param>
            /// <returns>The score, or infinity when nothing removes haul error.</returns>
            private Double CheapestFor
                (
                List<(Int32 Row, Int32 Col)> Members
                )
            {
                Double Best = Double.PositiveInfinity;
                Candidate Line = LineCandidate(Members, 0);
                if (Line != null)
                {
                    Best = Line.Score;
                }
                if ((_BestRejected != null) && Intersects(_BestRejected, Members) && (_BestRejected.Score < Best))
                {
                    Best = _BestRejected.Score;
                }
                return Best;
            }

            /// <summary>True when the move's cells meet the group.</summary>
            /// <param name="Move">Priced move.</param>
            /// <param name="Members">Group cells.</param>
            /// <returns>True when they share a cell.</returns>
            private static Boolean Intersects
                (
                Candidate Move,
                List<(Int32 Row, Int32 Col)> Members
                )
            {
                HashSet<(Int32 Row, Int32 Col)> Inside = new HashSet<(Int32 Row, Int32 Col)>(Members);
                for (Int32 Index = 0; Index < Move.Pass.CutCells.Count; Index += 1)
                {
                    (Int32 Bx, Int32 By) Cell = Move.Pass.CutCells[Index].Cell;
                    if (Inside.Contains((Cell.By, Cell.Bx)))
                    {
                        return true;
                    }
                }
                for (Int32 Index = 0; Index < Move.Pass.FillCells.Count; Index += 1)
                {
                    (Int32 Bx, Int32 By) Cell = Move.Pass.FillCells[Index].Cell;
                    if (Inside.Contains((Cell.By, Cell.Bx)))
                    {
                        return true;
                    }
                }
                return false;
            }

            /// <summary>Joins two cell lists.</summary>
            /// <param name="Left">First list.</param>
            /// <param name="Right">Second list.</param>
            /// <returns>The union, in first-seen order.</returns>
            private static List<(Int32 Row, Int32 Col)> Union
                (
                List<(Int32 Row, Int32 Col)> Left,
                List<(Int32 Row, Int32 Col)> Right
                )
            {
                List<(Int32 Row, Int32 Col)> Both = new List<(Int32 Row, Int32 Col)>();
                HashSet<(Int32 Row, Int32 Col)> Seen = new HashSet<(Int32 Row, Int32 Col)>();
                if (Left != null)
                {
                    for (Int32 Index = 0; Index < Left.Count; Index += 1)
                    {
                        if (Seen.Add(Left[Index]))
                        {
                            Both.Add(Left[Index]);
                        }
                    }
                }
                if (Right != null)
                {
                    for (Int32 Index = 0; Index < Right.Count; Index += 1)
                    {
                        if (Seen.Add(Right[Index]))
                        {
                            Both.Add(Right[Index]);
                        }
                    }
                }
                return Both;
            }

            /// <summary>One cell put back after a haul-error trial.</summary>
            private struct Snap
            {
                /// <summary>Gets or sets the row.</summary>
                public Int32 Row;

                /// <summary>Gets or sets the column.</summary>
                public Int32 Col;

                /// <summary>Gets or sets the residual in metres.</summary>
                public Double Residual;

                /// <summary>Gets or sets the surplus flag.</summary>
                public Boolean Raised;

                /// <summary>Gets or sets the deficit flag.</summary>
                public Boolean Lowered;
            }

            /// <summary>Applies slots onto the residual without touching the ledgers.</summary>
            /// <param name="Cuts">Cut slots.</param>
            /// <param name="Fills">Fill slots.</param>
            /// <param name="Snaps">Cells to restore.</param>
            private void Paint
                (
                List<Slot> Cuts,
                List<Slot> Fills,
                List<Snap> Snaps
                )
            {
                for (Int32 Index = 0; Index < Cuts.Count; Index += 1)
                {
                    Slot Slot = Cuts[Index];
                    Snaps.Add(new Snap
                    {
                        Row = Slot.Row,
                        Col = Slot.Col,
                        Residual = _R[Slot.Row, Slot.Col],
                        Raised = _Raised[Slot.Row, Slot.Col],
                        Lowered = _Lowered[Slot.Row, Slot.Col],
                    });
                    Boolean Source = _Lowered[Slot.Row, Slot.Col] || !IsRealCut(Slot.Row, Slot.Col);
                    _R[Slot.Row, Slot.Col] -= Slot.Bank / _CutPerM;
                    if (Source)
                    {
                        _Lowered[Slot.Row, Slot.Col] = true;
                    }
                }
                for (Int32 Index = 0; Index < Fills.Count; Index += 1)
                {
                    Slot Slot = Fills[Index];
                    Snaps.Add(new Snap
                    {
                        Row = Slot.Row,
                        Col = Slot.Col,
                        Residual = _R[Slot.Row, Slot.Col],
                        Raised = _Raised[Slot.Row, Slot.Col],
                        Lowered = _Lowered[Slot.Row, Slot.Col],
                    });
                    Boolean Sink = _Raised[Slot.Row, Slot.Col] || !IsRealFill(Slot.Row, Slot.Col);
                    _R[Slot.Row, Slot.Col] += Slot.Bank / _FillPerM;
                    if (Sink)
                    {
                        _Raised[Slot.Row, Slot.Col] = true;
                    }
                }
            }

            /// <summary>Puts a trial residual back.</summary>
            /// <param name="Snaps">Saved cells, last change first.</param>
            private void Restore
                (
                List<Snap> Snaps
                )
            {
                for (Int32 Index = Snaps.Count - 1; Index >= 0; Index -= 1)
                {
                    Snap Snap = Snaps[Index];
                    _R[Snap.Row, Snap.Col] = Snap.Residual;
                    _Raised[Snap.Row, Snap.Col] = Snap.Raised;
                    _Lowered[Snap.Row, Snap.Col] = Snap.Lowered;
                }
            }

            /// <summary>Haul error: 8-connected groups the plane cannot fix.</summary>
            /// <returns>Bank cubic yards.</returns>
            private Double MeasureHaul()
            {
                Double Total = 0.0;
                foreach (List<(Int32 Row, Int32 Col)> Members in CountingGroups())
                {
                    for (Int32 Index = 0; Index < Members.Count; Index += 1)
                    {
                        (Int32 Row, Int32 Col) Cell = Members[Index];
                        Total += CellError(Cell.Row, Cell.Col, _R[Cell.Row, Cell.Col], _Raised[Cell.Row, Cell.Col], _Lowered[Cell.Row, Cell.Col]);
                    }
                }
                return Total;
            }

            /// <summary>8-connected groups beyond the tolerance whose length or peak the plane cannot take.</summary>
            /// <returns>The groups that count as haul error.</returns>
            private List<List<(Int32 Row, Int32 Col)>> CountingGroups()
            {
                Boolean[,] Seen = new Boolean[_Rows, _Cols];
                List<List<(Int32 Row, Int32 Col)>> Found = new List<List<(Int32 Row, Int32 Col)>>();
                for (Int32 Row = 0; Row < _Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < _Cols; Col += 1)
                    {
                        if (Seen[Row, Col] || !Outside(Row, Col))
                        {
                            continue;
                        }
                        Boolean High = _R[Row, Col] > 0.0;
                        List<(Int32 Row, Int32 Col)> Members = new List<(Int32 Row, Int32 Col)>();
                        Queue<(Int32 Row, Int32 Col)> Waiting = new Queue<(Int32 Row, Int32 Col)>();
                        Waiting.Enqueue((Row, Col));
                        Seen[Row, Col] = true;
                        Int32 MinRow = Row;
                        Int32 MaxRow = Row;
                        Int32 MinCol = Col;
                        Int32 MaxCol = Col;
                        Double Peak = Math.Abs(_R[Row, Col]);
                        while (Waiting.Count > 0)
                        {
                            (Int32 Row, Int32 Col) Cell = Waiting.Dequeue();
                            Members.Add(Cell);
                            MinRow = Math.Min(MinRow, Cell.Row);
                            MaxRow = Math.Max(MaxRow, Cell.Row);
                            MinCol = Math.Min(MinCol, Cell.Col);
                            MaxCol = Math.Max(MaxCol, Cell.Col);
                            Peak = Math.Max(Peak, Math.Abs(_R[Cell.Row, Cell.Col]));
                            for (Int32 DRow = -1; DRow <= 1; DRow += 1)
                            {
                                for (Int32 DCol = -1; DCol <= 1; DCol += 1)
                                {
                                    if ((DRow == 0) && (DCol == 0))
                                    {
                                        continue;
                                    }
                                    Push8(Waiting, Seen, Cell.Row + DRow, Cell.Col + DCol, High);
                                }
                            }
                        }
                        Double LengthM = Math.Max(MaxRow - MinRow + 1, MaxCol - MinCol + 1) * _CellM;
                        Double PeakMm = Peak * 1000.0;
                        if ((LengthM > Correcting.PLANE_LEN_M) || (PeakMm > PLANE_MAX_MM))
                        {
                            Found.Add(Members);
                        }
                    }
                }
                return Found;
            }

            /// <summary>Adds an 8-connected same-sign neighbour that is still outside the tolerance.</summary>
            /// <param name="Waiting">Flood queue.</param>
            /// <param name="Seen">Cells already claimed.</param>
            /// <param name="Row">Row.</param>
            /// <param name="Col">Column.</param>
            /// <param name="High">True for a high group.</param>
            private void Push8
                (
                Queue<(Int32 Row, Int32 Col)> Waiting,
                Boolean[,] Seen,
                Int32 Row,
                Int32 Col,
                Boolean High
                )
            {
                if ((Row < 0) || (Col < 0) || (Row >= _Rows) || (Col >= _Cols) || Seen[Row, Col] || !Outside(Row, Col))
                {
                    return;
                }
                if ((_R[Row, Col] > 0.0) != High)
                {
                    return;
                }
                Seen[Row, Col] = true;
                Waiting.Enqueue((Row, Col));
            }

            /// <summary>A priced move that removes haul error.</summary>
            private sealed class Candidate
            {
                /// <summary>Initializes a priced move.</summary>
                /// <param name="Pass">The pass to keep if this move wins.</param>
                /// <param name="ErrorRemoved">Haul error it removes.</param>
                /// <param name="Seconds">Predicted seconds.</param>
                /// <param name="Score">Seconds per bank cubic yard of haul error.</param>
                /// <param name="Kind">Candidate type.</param>
                /// <param name="Index">Stable index.</param>
                /// <param name="CutPrefs">Cells the move may cut.</param>
                /// <param name="FillPrefs">Cells the move may fill.</param>
                /// <param name="SinkHints">Surplus hints.</param>
                /// <param name="SourceHints">Deficit hints.</param>
                /// <param name="DirX">Travel east.</param>
                /// <param name="DirY">Travel north.</param>
                /// <param name="Anchor">Nearest-cell anchor.</param>
                public Candidate
                    (
                    Pass Pass,
                    Double ErrorRemoved,
                    Double Seconds,
                    Double Score,
                    HaulStage Kind,
                    Int32 Index,
                    List<(Int32 Row, Int32 Col)> CutPrefs,
                    List<(Int32 Row, Int32 Col)> FillPrefs,
                    List<(Int32 Row, Int32 Col)> SinkHints,
                    List<(Int32 Row, Int32 Col)> SourceHints,
                    Double DirX,
                    Double DirY,
                    (Double X, Double Y) Anchor
                    )
                {
                    this.Pass = Pass;
                    this.ErrorRemoved = ErrorRemoved;
                    this.Seconds = Seconds;
                    this.Score = Score;
                    this.Kind = Kind;
                    this.Index = Index;
                    this.CutPrefs = CutPrefs;
                    this.FillPrefs = FillPrefs;
                    this.SinkHints = SinkHints;
                    this.SourceHints = SourceHints;
                    this.DirX = DirX;
                    this.DirY = DirY;
                    this.Anchor = Anchor;
                }

                /// <summary>Gets the pass.</summary>
                public Pass Pass { get; }

                /// <summary>Gets the haul error removed.</summary>
                public Double ErrorRemoved { get; }

                /// <summary>Gets the seconds.</summary>
                public Double Seconds { get; }

                /// <summary>Gets the score.</summary>
                public Double Score { get; }

                /// <summary>Gets the type.</summary>
                public HaulStage Kind { get; }

                /// <summary>Gets the stable index.</summary>
                public Int32 Index { get; }

                /// <summary>Gets the cut cells the simulation started from.</summary>
                public List<(Int32 Row, Int32 Col)> CutPrefs { get; }

                /// <summary>Gets the fill cells the simulation started from.</summary>
                public List<(Int32 Row, Int32 Col)> FillPrefs { get; }

                /// <summary>Gets the surplus hints.</summary>
                public List<(Int32 Row, Int32 Col)> SinkHints { get; }

                /// <summary>Gets the deficit hints.</summary>
                public List<(Int32 Row, Int32 Col)> SourceHints { get; }

                /// <summary>Gets the travel east.</summary>
                public Double DirX { get; }

                /// <summary>Gets the travel north.</summary>
                public Double DirY { get; }

                /// <summary>Gets the anchor.</summary>
                public (Double X, Double Y) Anchor { get; }

                /// <summary>Gets or sets the cells whose dirt invalidates this price. Null watches the prefs.</summary>
                public List<(Int32 Row, Int32 Col)> Watch { get; set; }

                /// <summary>Gets or sets the epoch this price was made in.</summary>
                public Int32 Epoch { get; set; }
            }

            /// <summary>Running totals for one candidate type.</summary>
            private struct Tally
            {
                /// <summary>Gets or sets the moves kept.</summary>
                public Int32 Moves;

                /// <summary>Gets or sets the bank cubic yards moved.</summary>
                public Double Bcy;

                /// <summary>Gets or sets the error removed.</summary>
                public Double Error;

                /// <summary>Gets or sets the seconds.</summary>
                public Double Seconds;
            }
        }
    }
}
