using System;
using System.Collections.Generic;
using System.Globalization;

namespace HaulBuilder
{
    /// <summary>
    /// At each haul, prices the next flow trip against the best trip in the
    /// current cluster plan and the best trip in the current thin-cut plan.
    /// The cheapest seconds per bank cubic yard is kept. A plan refreshes after
    /// ten trips from that stage. A batch that no longer fits is replaced from
    /// the rest of that plan, and a used-up plan refreshes on the same ten-trip cadence.
    /// </summary>
    internal sealed class HaulCostSearch
    {
        /// <summary>What one decision did.</summary>
        internal enum CostStep
        {
            /// <summary>A trip was appended.</summary>
            TookTrip = 0,

            /// <summary>Nothing under the ceiling. Correcting should start.</summary>
            Stop     = 1,

            /// <summary>The picture asked to stop after its cluster trips.</summary>
            Picture  = 2,
        }

        /// <summary>Cycle clock used for every offer.</summary>
        private readonly TimeParams _Clock;

        /// <summary>Flow trips required before the ceiling locks.</summary>
        private readonly Int32 _BaselineTrips;

        /// <summary>Smoothed flow east, shared by cluster and thin-cut plans.</summary>
        private readonly Double[,] _ArrowX;

        /// <summary>Smoothed flow north.</summary>
        private readonly Double[,] _ArrowY;

        /// <summary>Seconds per bank cubic yard of each flow trip already kept, in order.</summary>
        private readonly List<Double> _FlowCosts = new List<Double>();

        /// <summary>The cluster plan the current batch was taken from. Null before the first plan.</summary>
        private ClusterSchedule? _ClusterSchedule;

        /// <summary>Cluster trips still available from the last plan. At most ten.</summary>
        private List<ResidualPairTrip> _ClusterLeft = new List<ResidualPairTrip>();

        /// <summary>Cluster trips kept since the last plan.</summary>
        private Int32 _ClusterSincePlan;

        /// <summary>Cluster trips kept on this field.</summary>
        private Int32 _ClusterTrips;

        /// <summary>The thin-cut plan the current batch was taken from. Null before the first plan.</summary>
        private ClusterSchedule? _ThinSchedule;

        /// <summary>Thin-cut trips still available from the last plan. At most ten.</summary>
        private List<ResidualPairTrip> _ThinLeft = new List<ResidualPairTrip>();

        /// <summary>Thin-cut trips kept since the last plan.</summary>
        private Int32 _ThinSincePlan;

        /// <summary>Thin-cut trips kept on this field.</summary>
        private Int32 _ThinTrips;

        /// <summary>
        /// Cluster trips already priced against a fingerprint of their piece depths.
        /// The same depths build the same route, so the next decision reuses it.
        /// </summary>
        private List<ScoredTrip?> _ClusterScores = new List<ScoredTrip?>();

        /// <summary>Thin-cut trips already priced against their piece depths.</summary>
        private List<ScoredTrip?> _ThinScores = new List<ScoredTrip?>();

        /// <summary>Cluster trips after the current batch. The plan is not rebuilt to reach them.</summary>
        private List<ResidualPairTrip> _ClusterQueue = new List<ResidualPairTrip>();

        /// <summary>Thin-cut trips after the current batch.</summary>
        private List<ResidualPairTrip> _ThinQueue = new List<ResidualPairTrip>();

        /// <summary>
        /// Decisions in a row whose cluster plan built nothing.
        /// A fresh plan waits until this reaches ten, so a dead plan is not rebuilt every haul.
        /// </summary>
        private Int32 _ClusterIdle;

        /// <summary>Decisions in a row whose thin-cut plan built nothing.</summary>
        private Int32 _ThinIdle;

        /// <summary>
        /// Creates the search.
        /// </summary>
        /// <param name="Clock">Speeds and efficiency for the cycle clock.</param>
        /// <param name="BaselineTrips">Flow trips that fix the median. Below one uses the production count.</param>
        /// <param name="ArrowX">Smoothed flow east.</param>
        /// <param name="ArrowY">Smoothed flow north.</param>
        public HaulCostSearch
            (
            TimeParams Clock,
            Int32 BaselineTrips,
            Double[,] ArrowX,
            Double[,] ArrowY
            )
        {
            if (Clock == null) throw new ArgumentNullException(nameof(Clock));
            if (ArrowX == null) throw new ArgumentNullException(nameof(ArrowX));
            if (ArrowY == null) throw new ArgumentNullException(nameof(ArrowY));
            _Clock = Clock;
            _BaselineTrips = BaselineTrips < 1 ? StageCost.BASELINE_FLOW_TRIPS : BaselineTrips;
            _ArrowX = ArrowX;
            _ArrowY = ArrowY;
        }

        /// <summary>Gets the cut still on the field when hauling stopped, in bank cubic yards.</summary>
        public Double CutLeftBcy { get; private set; }

        /// <summary>Gets the fill still on the field when hauling stopped, in bank cubic yards.</summary>
        public Double FillLeftBcy { get; private set; }

        /// <summary>Gets whether the stop was the ceiling rather than an empty field.</summary>
        public Boolean StoppedOnCeiling { get; private set; }

        /// <summary>Gets whether the leftover yards have been stored.</summary>
        public Boolean HasLeft { get; private set; }

        /// <summary>
        /// Prices the flow trip against the other stages and keeps the winner.
        /// The flow trip's dirt is already off the field. It is put back unless that trip wins.
        /// </summary>
        /// <param name="FlowPass">The next mouth-search trip, or null when the ribbon has none.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="CutLeft">Remaining cut. The winner is subtracted here.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="Passes">Found-order routes. The winner is appended.</param>
        /// <param name="StopAfterClusterTrips">
        /// When greater than zero, return a picture stop after this many cluster trips.
        /// </param>
        /// <returns>Whether a trip was kept, hauling should stop, or the picture is full.</returns>
        public CostStep Take
            (
            Pass FlowPass,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            List<Pass> Passes,
            Int32 StopAfterClusterTrips
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            if (CutLeft == null) throw new ArgumentNullException(nameof(CutLeft));
            if (FillLeft == null) throw new ArgumentNullException(nameof(FillLeft));
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            Double? Ceiling = StageCost.CeilingSecondsPerBcy(_FlowCosts, _BaselineTrips);
            StageCost.Offer? FlowOffer = PriceFlow(FlowPass, Grid);
            // The other stages have to see the ground without a trip that may be refused.
            if (FlowPass != null)
            {
                AddDepth(FlowPass.CutCells, CutLeft);
                AddDepth(FlowPass.FillCells, FillLeft);
            }
            Boolean Rejected = false;
            Priced Cluster = PricePlan(
                Clusters: true,
                Grid,
                Ledger,
                CutLeft,
                FillLeft,
                BowlBcy,
                Ceiling,
                ref Rejected);
            Priced Thin = PricePlan(
                Clusters: false,
                Grid,
                Ledger,
                CutLeft,
                FillLeft,
                BowlBcy,
                Ceiling,
                ref Rejected);
            List<StageCost.Offer> Offers = new List<StageCost.Offer>();
            if (FlowOffer.HasValue)
            {
                if (!Ceiling.HasValue || (FlowOffer.Value.SecondsPerBcy <= Ceiling.Value))
                {
                    Offers.Add(FlowOffer.Value);
                }
                else
                {
                    Rejected = true;
                }
            }
            if (Cluster.Offer.HasValue)
            {
                Offers.Add(Cluster.Offer.Value);
            }
            if (Thin.Offer.HasValue)
            {
                Offers.Add(Thin.Offer.Value);
            }
            StageCost.Offer? Choice = StageCost.Choose(Offers, Ceiling);
            if (!Choice.HasValue)
            {
                // A candidate that lost the ceiling is dirt left on purpose.
                // No candidate means the field has nothing these stages can haul.
                StoppedOnCeiling = Rejected;
                RememberLeft(CutLeft, FillLeft, Grid, Ledger);
                return CostStep.Stop;
            }
            if (Choice.Value.Stage == HaulStage.Flow)
            {
                RemoveDepth(FlowPass.CutCells, CutLeft);
                RemoveDepth(FlowPass.FillCells, FillLeft);
                FlowPass.Stage = HaulStage.Flow;
                Passes.Add(FlowPass);
                _FlowCosts.Add(Choice.Value.SecondsPerBcy);
                return CostStep.TookTrip;
            }
            if (Choice.Value.Stage == HaulStage.Clusters)
            {
                return KeepPlanned(
                    Cluster,
                    HaulStage.Clusters,
                    ref _ClusterLeft,
                    ref _ClusterSincePlan,
                    ref _ClusterTrips,
                    CutLeft,
                    FillLeft,
                    Passes,
                    StopAfterClusterTrips,
                    "cluster hauling");
            }
            return KeepPlanned(
                Thin,
                HaulStage.ThinCut,
                ref _ThinLeft,
                ref _ThinSincePlan,
                ref _ThinTrips,
                CutLeft,
                FillLeft,
                Passes,
                StopAfterTrips: 0,
                "thin-cut");
        }

        /// <summary>
        /// Stores the cut and fill still on the field, in bank cubic yards.
        /// </summary>
        /// <param name="CutLeft">Remaining cut depth in metres.</param>
        /// <param name="FillLeft">Remaining fill depth in metres.</param>
        /// <param name="Grid">Field grid. Cells outside it are skipped.</param>
        /// <param name="Ledger">Yard factors.</param>
        public void RememberLeft
            (
            Double[,] CutLeft,
            Double[,] FillLeft,
            Grid Grid,
            HaulWorkLedger Ledger
            )
        {
            Double Cut = 0.0;
            Double Fill = 0.0;
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    if (!Grid.Valid[Row, Col])
                    {
                        continue;
                    }
                    Cut += CutLeft[Row, Col] * Ledger.CutBcyPerDepthM;
                    Fill += FillLeft[Row, Col] * Ledger.FillBcyPerDepthM;
                }
            }
            CutLeftBcy = Cut;
            FillLeftBcy = Fill;
            HasLeft = true;
        }

        /// <summary>
        /// Prices the flow trip. A missing or empty trip is not an offer.
        /// </summary>
        /// <param name="FlowPass">The mouth-search trip, or null.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The offer, or null.</returns>
        private StageCost.Offer? PriceFlow
            (
            Pass FlowPass,
            Grid Grid
            )
        {
            if ((FlowPass == null) || (FlowPass.LoadBcy <= 1e-12))
            {
                return null;
            }
            Double Seconds = TimeModel.EstimatePassTime(Grid, FlowPass, _Clock);
            return new StageCost.Offer(HaulStage.Flow, Seconds, FlowPass.LoadBcy);
        }

        /// <summary>
        /// The cheapest buildable trip in the current batch. A batch that builds
        /// nothing is replaced from the rest of the plan before the field is reclustered.
        /// </summary>
        /// <param name="Clusters">True for the cluster plan. False for thin-cut.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="CutLeft">Remaining cut.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <param name="BowlBcy">Pan capacity.</param>
        /// <param name="Ceiling">Maximum seconds per yard. Null while the baseline is open.</param>
        /// <param name="Rejected">Set when a built trip loses only because of the ceiling.</param>
        /// <returns>The affordable trip, or an empty price when the stage has none.</returns>
        private Priced PricePlan
            (
            Boolean Clusters,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            Double? Ceiling,
            ref Boolean Rejected
            )
        {
            EnsurePlan(Clusters, Force: false, Grid, Ledger, CutLeft, FillLeft, BowlBcy);
            Priced Priced = ScoreAhead(Clusters, Grid, Ledger, CutLeft, FillLeft, BowlBcy, Ceiling, ref Rejected);
            if (Priced.Kind != PriceKind.None)
            {
                if (Clusters)
                {
                    _ClusterIdle = 0;
                }
                else
                {
                    _ThinIdle = 0;
                }
                return Priced;
            }
            // Nothing left in this plan fits the ground. Rebuild on the ten-trip cadence.
            Int32 Idle = (Clusters ? _ClusterIdle : _ThinIdle) + 1;
            if (Clusters)
            {
                _ClusterIdle = Idle;
            }
            else
            {
                _ThinIdle = Idle;
            }
            if (Idle < ClusterHauling.TRIPS_PER_PLAN)
            {
                return Priced;
            }
            EnsurePlan(Clusters, Force: true, Grid, Ledger, CutLeft, FillLeft, BowlBcy);
            if (Clusters)
            {
                _ClusterIdle = 0;
            }
            else
            {
                _ThinIdle = 0;
            }
            return ScoreAhead(Clusters, Grid, Ledger, CutLeft, FillLeft, BowlBcy, Ceiling, ref Rejected);
        }

        /// <summary>
        /// Scores the current batch. When that batch builds nothing, the next
        /// ten trips of the same plan become the batch. The field is not reclustered here.
        /// </summary>
        /// <param name="Clusters">True for the cluster plan. False for thin-cut.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="CutLeft">Remaining cut.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <param name="BowlBcy">Pan capacity.</param>
        /// <param name="Ceiling">Maximum seconds per yard. Null while the baseline is open.</param>
        /// <param name="Rejected">Set when a built trip loses only because of the ceiling.</param>
        /// <returns>The first batch that builds a trip, or nothing when the plan is used up.</returns>
        private Priced ScoreAhead
            (
            Boolean Clusters,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            Double? Ceiling,
            ref Boolean Rejected
            )
        {
            Priced Priced = ScoreBatch(Clusters, Grid, Ledger, CutLeft, FillLeft, BowlBcy, Ceiling, ref Rejected);
            while ((Priced.Kind == PriceKind.None) && AdvanceWindow(Clusters))
            {
                Priced = ScoreBatch(Clusters, Grid, Ledger, CutLeft, FillLeft, BowlBcy, Ceiling, ref Rejected);
            }
            return Priced;
        }

        /// <summary>
        /// Replaces the current batch with the next ten trips already in the plan.
        /// </summary>
        /// <param name="Clusters">True for the cluster queue. False for thin-cut.</param>
        /// <returns>True when a new batch was loaded.</returns>
        private Boolean AdvanceWindow
            (
            Boolean Clusters
            )
        {
            List<ResidualPairTrip> Queue = Clusters ? _ClusterQueue : _ThinQueue;
            if (Queue.Count == 0)
            {
                return false;
            }
            Int32 Take = Math.Min(ClusterHauling.TRIPS_PER_PLAN, Queue.Count);
            List<ResidualPairTrip> Left = new List<ResidualPairTrip>(Take);
            for (Int32 Index = 0; Index < Take; Index += 1)
            {
                Left.Add(Queue[Index]);
            }
            Queue.RemoveRange(0, Take);
            List<ScoredTrip?> Scores = new List<ScoredTrip?>(Take);
            for (Int32 Index = 0; Index < Take; Index += 1)
            {
                Scores.Add(null);
            }
            if (Clusters)
            {
                _ClusterLeft = Left;
                _ClusterScores = Scores;
            }
            else
            {
                _ThinLeft = Left;
                _ThinScores = Scores;
            }
            return true;
        }

        /// <summary>
        /// Loads a batch of at most ten trips. A live batch is kept until ten
        /// of its trips have been hauled, unless the caller forces a new plan.
        /// </summary>
        /// <param name="Clusters">True for clusters. False for thin-cut.</param>
        /// <param name="Force">True replaces the batch even before ten trips.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors. Unused. The plan uses the grid compaction.</param>
        /// <param name="CutLeft">Remaining cut.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <param name="BowlBcy">Pan capacity.</param>
        private void EnsurePlan
            (
            Boolean Clusters,
            Boolean Force,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy
            )
        {
            _ = Ledger;
            Int32 Since = Clusters ? _ClusterSincePlan : _ThinSincePlan;
            ClusterSchedule Held = Clusters ? _ClusterSchedule : _ThinSchedule;
            if (!Force && (Held != null) && (Since < ClusterHauling.TRIPS_PER_PLAN))
            {
                return;
            }
            Double[,] ResidualMm = ClusterHauling.ResidualMillimetres(CutLeft, FillLeft, Grid);
            // The full pairing schedules every yard. The next haul only needs the cheapest pairs.
            ClusterSchedule Schedule = Clusters
                ? ClusterHauling.CandidateField(ResidualMm, Grid, BowlBcy, _ArrowX, _ArrowY)
                : ThinCut.CandidateField(ResidualMm, Grid, BowlBcy, _ArrowX, _ArrowY);
            List<ResidualPairTrip> Left = new List<ResidualPairTrip>();
            List<ResidualPairTrip> Queue = new List<ResidualPairTrip>();
            IReadOnlyList<ResidualPairTrip> Trips = Schedule.Plan.Trips;
            Int32 Take = Math.Min(ClusterHauling.TRIPS_PER_PLAN, Trips.Count);
            for (Int32 Index = 0; Index < Take; Index += 1)
            {
                Left.Add(Trips[Index]);
            }
            for (Int32 Index = Take; Index < Trips.Count; Index += 1)
            {
                Queue.Add(Trips[Index]);
            }
            List<ScoredTrip?> Scores = new List<ScoredTrip?>(Left.Count);
            for (Int32 Index = 0; Index < Left.Count; Index += 1)
            {
                Scores.Add(null);
            }
            if (Clusters)
            {
                _ClusterSchedule = Schedule;
                _ClusterLeft = Left;
                _ClusterQueue = Queue;
                _ClusterSincePlan = 0;
                _ClusterScores = Scores;
            }
            else
            {
                _ThinSchedule = Schedule;
                _ThinLeft = Left;
                _ThinQueue = Queue;
                _ThinSincePlan = 0;
                _ThinScores = Scores;
            }
        }

        /// <summary>
        /// Builds each trip in the batch on a copy of the field and keeps the cheapest.
        /// A trip over the ceiling is remembered so hauling can stop, and it is not offered.
        /// </summary>
        /// <param name="Clusters">True for the cluster batch.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="CutLeft">Remaining cut. The copy is what the build changes.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <param name="BowlBcy">Pan capacity.</param>
        /// <param name="Ceiling">Maximum seconds per yard.</param>
        /// <param name="Rejected">Set when the cheapest built trip is over the ceiling.</param>
        /// <returns>The affordable winner, a ceiling rejection, or nothing buildable.</returns>
        private Priced ScoreBatch
            (
            Boolean Clusters,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            Double? Ceiling,
            ref Boolean Rejected
            )
        {
            List<ResidualPairTrip> Left = Clusters ? _ClusterLeft : _ThinLeft;
            ClusterSchedule Schedule = Clusters ? _ClusterSchedule : _ThinSchedule;
            List<ScoredTrip?> Scores = Clusters ? _ClusterScores : _ThinScores;
            Priced Best = new Priced();
            Double BestCost = Double.PositiveInfinity;
            if ((Schedule == null) || (Left.Count == 0))
            {
                return Best;
            }
            Dictionary<String, ResidualPairPiece> ById = PieceIndex(Schedule);
            for (Int32 Index = 0; Index < Left.Count; Index += 1)
            {
                UInt64 Print = Fingerprint(Left[Index], ById, CutLeft, FillLeft);
                Pass? Built = null;
                Double Seconds = 0.0;
                Double Cost = Double.PositiveInfinity;
                ScoredTrip? Cached = Index < Scores.Count ? Scores[Index] : null;
                if (Cached.HasValue && (Cached.Value.Fingerprint == Print))
                {
                    Built = Cached.Value.Pass;
                    Seconds = Cached.Value.Seconds;
                    Cost = Cached.Value.Cost;
                }
                else
                {
                    Built = BuildOnCopy(Left[Index], Schedule, Grid, Ledger, CutLeft, FillLeft, BowlBcy);
                    if ((Built != null) && (Built.LoadBcy > 1e-12))
                    {
                        Seconds = TimeModel.EstimatePassTime(Grid, Built, _Clock);
                        Cost = Seconds / Built.LoadBcy;
                    }
                    ScoredTrip Scored = new ScoredTrip
                    {
                        Fingerprint = Print,
                        Pass = Built,
                        Seconds = Seconds,
                        Cost = Cost,
                    };
                    if (Index < Scores.Count)
                    {
                        Scores[Index] = Scored;
                    }
                }
                if ((Built == null) || (Built.LoadBcy <= 1e-12))
                {
                    continue;
                }
                // The same cost keeps the earlier trip in the plan, so the sequence does not drift.
                if (Cost > BestCost - 1e-9)
                {
                    continue;
                }
                Pass Kept = Built;
                BestCost = Cost;
                Best = new Priced
                {
                    Kind = PriceKind.Affordable,
                    Offer = new StageCost.Offer(
                        Clusters ? HaulStage.Clusters : HaulStage.ThinCut,
                        Seconds,
                        Kept.LoadBcy),
                    Pass = Kept,
                    Index = Index,
                };
            }
            if ((Best.Kind == PriceKind.Affordable)
                && Ceiling.HasValue
                && (Best.Offer.Value.SecondsPerBcy > Ceiling.Value))
            {
                Rejected = true;
                Best.Kind = PriceKind.TooCostly;
                Best.Offer = null;
            }
            return Best;
        }

        /// <summary>
        /// Builds one planned trip against a copy of the remaining depths.
        /// </summary>
        /// <param name="Trip">The planned trip.</param>
        /// <param name="Schedule">The plan and its pieces.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="CutLeft">Remaining cut. Not changed.</param>
        /// <param name="FillLeft">Remaining fill. Not changed.</param>
        /// <param name="BowlBcy">Pan capacity.</param>
        /// <returns>The route, or null when the trip moved nothing.</returns>
        private Pass BuildOnCopy
            (
            ResidualPairTrip Trip,
            ClusterSchedule Schedule,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy
            )
        {
            Double[,] CutCopy = (Double[,])CutLeft.Clone();
            Double[,] FillCopy = (Double[,])FillLeft.Clone();
            Double[,] ResidualMm = ClusterHauling.ResidualMillimetres(CutCopy, FillCopy, Grid);
            List<Pass> Built = new List<Pass>();
            List<ResidualPairTrip> Batch = new List<ResidualPairTrip>();
            Batch.Add(Trip);
            ClusterHauling.ExecuteBatch(
                ResidualMm,
                Schedule,
                Batch,
                Grid,
                Ledger,
                CutCopy,
                FillCopy,
                BowlBcy,
                Built,
                _ArrowX,
                _ArrowY);
            if (Built.Count == 0)
            {
                return null;
            }
            return Built[0];
        }

        /// <summary>
        /// Writes a planned trip onto the field and drops it from the batch.
        /// The five-hundredth trip of a stage is the last. The next one throws.
        /// </summary>
        /// <param name="Priced">The trip to keep. Its pass was built on a copy.</param>
        /// <param name="Stage">The label stored on the route.</param>
        /// <param name="Left">The batch the trip came from.</param>
        /// <param name="SincePlan">Trips kept since the last plan.</param>
        /// <param name="TripsDone">Trips kept on this field for the stage.</param>
        /// <param name="CutLeft">Remaining cut.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <param name="Passes">Found-order routes.</param>
        /// <param name="StopAfterTrips">Picture limit. Zero does not stop early. Used for clusters only.</param>
        /// <param name="LimitName">Who stopped. The thrown text starts with this name.</param>
        /// <returns>A kept trip, or a picture stop after the cluster limit.</returns>
        private CostStep KeepPlanned
            (
            Priced Priced,
            HaulStage Stage,
            ref List<ResidualPairTrip> Left,
            ref Int32 SincePlan,
            ref Int32 TripsDone,
            Double[,] CutLeft,
            Double[,] FillLeft,
            List<Pass> Passes,
            Int32 StopAfterTrips,
            String LimitName
            )
        {
            if (TripsDone >= ClusterHauling.MAX_CLUSTER_TRIPS)
            {
                Double RemainingBcy = 0.0;
                for (Int32 Index = 0; Index < Left.Count; Index += 1)
                {
                    RemainingBcy += Left[Index].PayloadBcy;
                }
                throw new InvalidOperationException(String.Format(
                    CultureInfo.InvariantCulture,
                    "{0} stopped: trip limit. remaining trips={1} bcy={2:R}",
                    LimitName,
                    Left.Count,
                    RemainingBcy));
            }
            ClusterHauling.ApplyBuiltPass(Priced.Pass, CutLeft, FillLeft);
            Priced.Pass.Stage = Stage;
            Passes.Add(Priced.Pass);
            Left.RemoveAt(Priced.Index);
            // The cached route was built before this trip took its cells. Drop it with the trip.
            List<ScoredTrip?> Scores = Stage == HaulStage.Clusters ? _ClusterScores : _ThinScores;
            if (Priced.Index < Scores.Count)
            {
                Scores.RemoveAt(Priced.Index);
            }
            SincePlan += 1;
            TripsDone += 1;
            if ((StopAfterTrips > 0) && (TripsDone >= StopAfterTrips))
            {
                return CostStep.Picture;
            }
            return CostStep.TookTrip;
        }

        /// <summary>
        /// Puts depth back without clearing the cell list.
        /// The flow trip may still be the one that gets kept.
        /// </summary>
        /// <param name="Cells">Cells and depths to restore.</param>
        /// <param name="Remaining">Depth ledger.</param>
        private static void AddDepth
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double[,] Remaining
            )
        {
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Cells[Index];
                Remaining[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
            }
        }

        /// <summary>
        /// Takes the same depth off again after the flow trip wins.
        /// </summary>
        /// <param name="Cells">Cells and depths to remove.</param>
        /// <param name="Remaining">Depth ledger.</param>
        private static void RemoveDepth
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double[,] Remaining
            )
        {
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Cells[Index];
                Remaining[Entry.Cell.By, Entry.Cell.Bx] -= Entry.DepthM;
            }
        }

        /// <summary>
        /// Fingerprint of the depths a trip would cut and fill.
        /// Strip placement reads only those piece cells, so an equal print is the same route.
        /// </summary>
        /// <param name="Trip">The planned trip.</param>
        /// <param name="ById">Pieces by id.</param>
        /// <param name="CutLeft">Remaining cut.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <returns>The fingerprint.</returns>
        private static UInt64 Fingerprint
            (
            ResidualPairTrip Trip,
            Dictionary<String, ResidualPairPiece> ById,
            Double[,] CutLeft,
            Double[,] FillLeft
            )
        {
            UInt64 Hash = 14695981039346656037UL;
            Hash = Mix(Hash, (UInt64)(Trip.TripNo + 1));
            Hash = Mix(Hash, (UInt64)BitConverter.DoubleToInt64Bits(Trip.PayloadBcy));
            Hash = MixLegs(Hash, Trip.Cuts, ById, CutLeft, FillLeft);
            return MixLegs(Hash, Trip.Fills, ById, CutLeft, FillLeft);
        }

        /// <summary>
        /// Mixes one side of a trip. Missing pieces still change the print, so a
        /// dropped id cannot look like an empty piece.
        /// </summary>
        /// <param name="Hash">The print so far.</param>
        /// <param name="Legs">Cut or fill legs.</param>
        /// <param name="ById">Pieces by id.</param>
        /// <param name="CutLeft">Remaining cut.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <returns>The print with these legs mixed in.</returns>
        private static UInt64 MixLegs
            (
            UInt64 Hash,
            IReadOnlyList<ResidualPairLeg> Legs,
            Dictionary<String, ResidualPairPiece> ById,
            Double[,] CutLeft,
            Double[,] FillLeft
            )
        {
            if (Legs == null)
            {
                return Mix(Hash, 0UL);
            }
            for (Int32 LegIndex = 0; LegIndex < Legs.Count; LegIndex += 1)
            {
                ResidualPairLeg Leg = Legs[LegIndex];
                Hash = Mix(Hash, (UInt64)(LegIndex + 1));
                if ((Leg == null) || (Leg.Id == null) || !ById.TryGetValue(Leg.Id, out ResidualPairPiece? Piece) || (Piece == null))
                {
                    Hash = Mix(Hash, 0UL);
                    continue;
                }
                IReadOnlyList<(Int32 Row, Int32 Col)> Cells = Piece.Cells;
                for (Int32 CellIndex = 0; CellIndex < Cells.Count; CellIndex += 1)
                {
                    (Int32 Row, Int32 Col) Cell = Cells[CellIndex];
                    Hash = Mix(Hash, (UInt64)(Cell.Row + 1));
                    Hash = Mix(Hash, (UInt64)(Cell.Col + 1));
                    Hash = Mix(Hash, (UInt64)BitConverter.DoubleToInt64Bits(CutLeft[Cell.Row, Cell.Col]));
                    Hash = Mix(Hash, (UInt64)BitConverter.DoubleToInt64Bits(FillLeft[Cell.Row, Cell.Col]));
                }
            }
            return Hash;
        }

        /// <summary>
        /// FNV-1a step. The offset basis lives with the first caller.
        /// </summary>
        /// <param name="Hash">The print so far.</param>
        /// <param name="Value">The next word.</param>
        /// <returns>The mixed print.</returns>
        private static UInt64 Mix
            (
            UInt64 Hash,
            UInt64 Value
            )
        {
            return (Hash ^ Value) * 1099511628211UL;
        }

        /// <summary>
        /// Indexes the plan's pieces by id. A repeated id keeps the later piece.
        /// </summary>
        /// <param name="Schedule">The plan.</param>
        /// <returns>The index.</returns>
        private static Dictionary<String, ResidualPairPiece> PieceIndex
            (
            ClusterSchedule Schedule
            )
        {
            Dictionary<String, ResidualPairPiece> ById =
                new Dictionary<String, ResidualPairPiece>(StringComparer.Ordinal);
            IReadOnlyList<ResidualPairPiece> Pieces = Schedule.Pieces;
            for (Int32 Index = 0; Index < Pieces.Count; Index += 1)
            {
                ResidualPairPiece Piece = Pieces[Index];
                if ((Piece != null) && (Piece.Id != null))
                {
                    ById[Piece.Id] = Piece;
                }
            }
            return ById;
        }

        /// <summary>How a batch scored.</summary>
        private enum PriceKind
        {
            /// <summary>No trip in the batch could be built.</summary>
            None       = 0,

            /// <summary>The cheapest trip is at or under the ceiling.</summary>
            Affordable = 1,

            /// <summary>The cheapest trip was built and it is over the ceiling.</summary>
            TooCostly  = 2,
        }

        /// <summary>One stage's priced trip.</summary>
        private struct Priced
        {
            /// <summary>Gets or sets whether a trip was built and whether it is affordable.</summary>
            public PriceKind Kind;

            /// <summary>Gets or sets the offer. Null when the trip is over the ceiling or missing.</summary>
            public StageCost.Offer? Offer;

            /// <summary>Gets or sets the route built on a copy of the field.</summary>
            public Pass Pass;

            /// <summary>Gets or sets the batch index, so the kept trip can be dropped.</summary>
            public Int32 Index;
        }

        /// <summary>
        /// A trip already built against one fingerprint of its piece depths.
        /// A null route means those depths built nothing.
        /// </summary>
        private struct ScoredTrip
        {
            /// <summary>Gets or sets the depth fingerprint the route was built from.</summary>
            public UInt64 Fingerprint;

            /// <summary>Gets or sets the route. Null when the trip moved nothing.</summary>
            public Pass? Pass;

            /// <summary>Gets or sets the cycle seconds. Zero when the trip moved nothing.</summary>
            public Double Seconds;

            /// <summary>Gets or sets seconds per bank cubic yard.</summary>
            public Double Cost;
        }
    }
}
