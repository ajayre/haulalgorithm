using System.Globalization;

namespace HaulBuilder
{
    /// <summary>
    /// Cluster hauling after the flow ribbon hands off.
    /// Each pass clusters the current residual, pairs it, and runs at most
    /// ten trips. The old schedule is then dropped and the field is clustered
    /// again. Hauling stops when a plan has no trips, which hands off to
    /// thin-cut. Five hundred trips is a planner error: the search stops
    /// and does not enter thin-cut. The 50 mm band is not the switch.
    /// Cells are cut and filled to the target.
    /// </summary>
    static public class ClusterHauling
    {
        /// <summary>Normal end. The current residual scheduled nothing.</summary>
        public const String NO_HAULABLE_WORK = "no haulable work";

        /// <summary>Error end. The same work kept coming back. Correcting does not start.</summary>
        public const String TRIP_LIMIT = "trip limit";

        /// <summary>
        /// Early stop for a picture of the first routes. Thin-cut does not start.
        /// The production loop does not use this reason.
        /// </summary>
        public const String PICTURE_STOP = "picture";

        /// <summary>Executed cluster trips before the search treats the planner as stuck.</summary>
        public const Int32 MAX_CLUSTER_TRIPS = 500;

        /// <summary>Trips driven from one schedule before the field is clustered again.</summary>
        public const Int32 TRIPS_PER_PLAN = 10;

        /// <summary>
        /// Cheap pairs kept for the cost search. The first ten are priced,
        /// and the rest are the next batches when those no longer fit the ground.
        /// </summary>
        private const Int32 CANDIDATE_PAIRS = 40;

        /// <summary>Cut cells are taken to this many millimetres above the target.</summary>
        public const Double CUT_TO_MM = 0.0;

        /// <summary>Fill cells are brought to this many millimetres below the target.</summary>
        public const Double FILL_TO_MM = 0.0;

        /// <summary>Report band. Cells outside it are listed. They do not stop the loop.</summary>
        private const Double REPORT_BAND_MM = 50.0;

        /// <summary>Keeps two reports from writing the same temp file at once.</summary>
        private static readonly Object ReportGate = new Object();

        /// <summary>
        /// Runs cluster hauling, appends the trips to <paramref name="Passes"/>,
        /// and subtracts them from the remaining cut and fill.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Work ledger. Its yard factors balance each trip.</param>
        /// <param name="CutLeft">Remaining cut depth in metres. Each trip reduces it.</param>
        /// <param name="FillLeft">Remaining fill depth in metres. Each trip reduces it.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="Passes">Found-order routes. Cluster trips are appended.</param>
        /// <param name="ArrowX">Smoothed flow east. Null leaves lanes on the axis.</param>
        /// <param name="ArrowY">Smoothed flow north.</param>
        /// <param name="StopAfterTrips">
        /// When greater than zero, the loop returns after this many trips so a picture
        /// can be drawn. Zero runs until the field is clear or the trip limit.
        /// </param>
        /// <returns>Why the loop stopped. Correcting follows only when the reason is no haulable work.</returns>
        public static ClusterHaulStop Run
            (
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            List<Pass> Passes,
            Double[,]? ArrowX = null,
            Double[,]? ArrowY = null,
            Int32 StopAfterTrips = 0
            )
        {
            return Run(Grid, Ledger, CutLeft, FillLeft, BowlBcy, Passes, null, null, ArrowX, ArrowY, StopAfterTrips);
        }

        /// <summary>
        /// The state after a stop. No haulable work enters thin-cut.
        /// A trip limit throws, so thin-cut does not start.
        /// </summary>
        /// <param name="Stop">The stop from the cluster loop.</param>
        /// <returns>Thin-cut, when the field has no haulable cluster work.</returns>
        public static HaulSearchState Handoff
            (
            ClusterHaulStop Stop
            )
        {
            if (Stop == null) throw new ArgumentNullException(nameof(Stop));
            if (Stop.Reason == TRIP_LIMIT)
            {
                throw new InvalidOperationException(String.Format(
                    CultureInfo.InvariantCulture,
                    "cluster hauling stopped: trip limit. remaining trips={0} bcy={1:R}",
                    Stop.RemainingTrips,
                    Stop.RemainingBcy));
            }
            return HaulSearchState.ThinCut;
        }

        /// <summary>
        /// Same loop as the public Run, with a stand-in planner and executor
        /// for the end-condition tests.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Work ledger.</param>
        /// <param name="CutLeft">Remaining cut depth in metres.</param>
        /// <param name="FillLeft">Remaining fill depth in metres.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="Passes">Found-order routes.</param>
        /// <param name="Plan">Plans the current residual. Null uses clustering and pairing.</param>
        /// <param name="Execute">Drives one batch. Null builds the haul geometry.</param>
        /// <param name="ArrowX">Smoothed flow east. Null leaves lanes on the axis.</param>
        /// <param name="ArrowY">Smoothed flow north.</param>
        /// <param name="StopAfterTrips">
        /// When greater than zero, the loop returns after this many trips.
        /// Zero runs until the field is clear or the trip limit.
        /// </param>
        /// <returns>Why the loop stopped.</returns>
        internal static ClusterHaulStop Run
            (
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            List<Pass> Passes,
            Func<Double[,], ClusterSchedule>? Plan,
            Action<IReadOnlyList<ResidualPairTrip>>? Execute,
            Double[,]? ArrowX = null,
            Double[,]? ArrowY = null,
            Int32 StopAfterTrips = 0
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            if (CutLeft == null) throw new ArgumentNullException(nameof(CutLeft));
            if (FillLeft == null) throw new ArgumentNullException(nameof(FillLeft));
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            if (BowlBcy <= 0.0) throw new ArgumentOutOfRangeException(nameof(BowlBcy));
            String Reason = NO_HAULABLE_WORK;
            ResidualPairPlan Last = EmptyPlan();
            Int32 TripsDone = 0;
            Int32 RemainingTrips = 0;
            Double RemainingBcy = 0.0;
            while (true)
            {
                Double[,] ResidualMm = ResidualMillimetres(CutLeft, FillLeft, Grid);
                ClusterSchedule Schedule = Plan == null
                    ? PlanField(ResidualMm, Grid, BowlBcy, ArrowX, ArrowY)
                    : Plan(ResidualMm);
                if (Schedule == null) throw new InvalidOperationException("The cluster plan is missing.");
                Last = Schedule.Plan;
                // Nothing left to schedule is the normal handoff to thin-cut.
                if (Last.Trips.Count == 0)
                {
                    Reason = NO_HAULABLE_WORK;
                    break;
                }
                Int32 Take = Math.Min(TRIPS_PER_PLAN, Last.Trips.Count);
                List<ResidualPairTrip> Batch = new List<ResidualPairTrip>(Take);
                for (Int32 Index = 0; Index < Take; Index += 1)
                {
                    Batch.Add(Last.Trips[Index]);
                }
                if (Execute == null)
                {
                    Int32 Mark = Passes.Count;
                    ExecuteBatch(ResidualMm, Schedule, Batch, Grid, Ledger, CutLeft, FillLeft, BowlBcy, Passes, ArrowX, ArrowY);
                    // The cost search labels its own trips. This path is the sequential loop.
                    for (Int32 Index = Mark; Index < Passes.Count; Index += 1)
                    {
                        Passes[Index].Stage = HaulStage.Clusters;
                    }
                }
                else
                {
                    Execute(Batch);
                }
                TripsDone += Take;
                // A picture asks for the first routes only. The rest of the field stays put.
                if ((StopAfterTrips > 0) && (TripsDone >= StopAfterTrips))
                {
                    Reason = PICTURE_STOP;
                    break;
                }
                // The batch is finished before the check, so a stuck planner stops on this plan.
                if (TripsDone >= MAX_CLUSTER_TRIPS)
                {
                    Reason = TRIP_LIMIT;
                    RemainingTrips = Last.Trips.Count - Take;
                    RemainingBcy = 0.0;
                    for (Int32 Index = Take; Index < Last.Trips.Count; Index += 1)
                    {
                        RemainingBcy += Last.Trips[Index].PayloadBcy;
                    }
                    break;
                }
            }
            ClusterHaulStop Stop = new ClusterHaulStop(Reason, TripsDone, RemainingTrips, RemainingBcy);
            Report(Stop, Last, CutLeft, FillLeft, Grid);
            return Stop;
        }

        /// <summary>
        /// Clusters the residual on mask E and pairs it down to the target.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Grid">Field grid. Clustering erodes its valid mask itself.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="ArrowX">Smoothed flow east.</param>
        /// <param name="ArrowY">Smoothed flow north.</param>
        /// <returns>The schedule for this residual.</returns>
        internal static ClusterSchedule PlanField
            (
            Double[,] ResidualMm,
            Grid Grid,
            Double BowlBcy,
            Double[,]? ArrowX,
            Double[,]? ArrowY
            )
        {
            ResidualClusterReport Clusters = ResidualClusters.Build(
                ResidualMm,
                Grid.Valid,
                Grid.GridSizeM,
                BowlBcy,
                Grid.Compaction);
            List<ResidualPairPiece> Pieces = PiecesToTarget(Clusters, ResidualMm, Grid);
            // The bowl seconds are computed once per cut piece. Pairing still chooses the fills.
            StripLanes.Ground Ground = new StripLanes.Ground(
                Grid,
                ResidualMm,
                Grid.Valid,
                Grid.Compaction,
                CUT_TO_MM,
                ArrowX,
                ArrowY);
            ResidualPairPlan Plan = ResidualPairing.Plan(
                Pieces,
                Grid.GridSizeM,
                BowlBcy,
                Constants.DEFAULT_BLADE_WIDTH_MM / Constants.MILLIMETERS_PER_METER,
                Ground);
            return new ClusterSchedule(Plan, Pieces);
        }

        /// <summary>
        /// Clusters the residual and keeps the cheapest cut and fill pairs.
        /// The cost search prices these. The full pairing still belongs to
        /// <see cref="PlanField"/>, which schedules every yard.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="ArrowX">Smoothed flow east. Unused. The heading is the pair.</param>
        /// <param name="ArrowY">Smoothed flow north. Unused.</param>
        /// <returns>The cheap pairs, shortest seconds per yard first.</returns>
        internal static ClusterSchedule CandidateField
            (
            Double[,] ResidualMm,
            Grid Grid,
            Double BowlBcy,
            Double[,]? ArrowX,
            Double[,]? ArrowY
            )
        {
            _ = ArrowX;
            _ = ArrowY;
            ResidualClusterReport Clusters = ResidualClusters.Build(
                ResidualMm,
                Grid.Valid,
                Grid.GridSizeM,
                BowlBcy,
                Grid.Compaction);
            List<ResidualPairPiece> Pieces = PiecesToTarget(Clusters, ResidualMm, Grid);
            return CheapSchedule(Pieces, Grid.GridSizeM, BowlBcy);
        }

        /// <summary>
        /// Ranks cut and fill pairs by haul seconds per yard and keeps the cheapest.
        /// A pair under a quarter bowl is left for correcting, the same floor the pairing uses.
        /// </summary>
        /// <param name="Pieces">Cut and fill pieces.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <returns>A schedule whose trips are those pairs.</returns>
        internal static ClusterSchedule CheapSchedule
            (
            IReadOnlyList<ResidualPairPiece> Pieces,
            Double CellSizeM,
            Double BowlBcy
            )
        {
            if (Pieces == null) throw new ArgumentNullException(nameof(Pieces));
            if (CellSizeM <= 0.0) throw new ArgumentOutOfRangeException(nameof(CellSizeM));
            if (BowlBcy <= 0.0) throw new ArgumentOutOfRangeException(nameof(BowlBcy));
            List<ResidualPairPiece> Cuts = new List<ResidualPairPiece>();
            List<ResidualPairPiece> Fills = new List<ResidualPairPiece>();
            for (Int32 Index = 0; Index < Pieces.Count; Index += 1)
            {
                ResidualPairPiece Piece = Pieces[Index];
                if ((Piece == null) || (Piece.Id == null) || (Piece.HaulVolumeBcy <= 0.0))
                {
                    continue;
                }
                if (Piece.Sign == "cut")
                {
                    Cuts.Add(Piece);
                }
                else if (Piece.Sign == "fill")
                {
                    Fills.Add(Piece);
                }
            }
            Double FloorBcy = ResidualPairing.PARTIAL_BOWL_FRACTION * BowlBcy;
            List<RankedPair> Ranked = new List<RankedPair>();
            for (Int32 CutIndex = 0; CutIndex < Cuts.Count; CutIndex += 1)
            {
                ResidualPairPiece Cut = Cuts[CutIndex];
                for (Int32 FillIndex = 0; FillIndex < Fills.Count; FillIndex += 1)
                {
                    ResidualPairPiece Fill = Fills[FillIndex];
                    Double Payload = BowlBcy;
                    if (Cut.HaulVolumeBcy < Payload)
                    {
                        Payload = Cut.HaulVolumeBcy;
                    }
                    if (Fill.HaulVolumeBcy < Payload)
                    {
                        Payload = Fill.HaulVolumeBcy;
                    }
                    if (Payload < FloorBcy)
                    {
                        continue;
                    }
                    Double DCol = (Cut.CentroidCol - Fill.CentroidCol) * CellSizeM;
                    Double DRow = (Cut.CentroidRow - Fill.CentroidRow) * CellSizeM;
                    Double HaulM = Math.Sqrt((DCol * DCol) + (DRow * DRow));
                    // Fixed seconds for the speed change, then the haul both ways. A larger load wins a tie.
                    Double HaulS = (HaulM / ResidualPairing.V_LOADED_MS)
                        + (HaulM / ResidualPairing.V_EMPTY_MS)
                        + Constants.DEFAULT_ACCELERATION_TIME_S
                        + Constants.DEFAULT_DECELERATION_TIME_S;
                    Ranked.Add(new RankedPair
                    {
                        Cost = HaulS / Payload,
                        Payload = Payload,
                        Distance = HaulM,
                        Cut = Cut,
                        Fill = Fill,
                    });
                }
            }
            Ranked.Sort(CompareRanked);
            Int32 Keep = Math.Min(CANDIDATE_PAIRS, Ranked.Count);
            List<ResidualPairTrip> Trips = new List<ResidualPairTrip>(Keep);
            Double PayloadSum = 0.0;
            Double HaulSum = 0.0;
            Int32 Partials = 0;
            for (Int32 Index = 0; Index < Keep; Index += 1)
            {
                RankedPair Pair = Ranked[Index];
                if (Pair.Payload < BowlBcy - 1e-6)
                {
                    Partials += 1;
                }
                PayloadSum += Pair.Payload;
                HaulSum += Pair.Distance * Pair.Payload;
                Trips.Add(new ResidualPairTrip(
                    Index + 1,
                    new ResidualPairLeg[] { new ResidualPairLeg(Pair.Cut.Id, Pair.Payload) },
                    new ResidualPairLeg[] { new ResidualPairLeg(Pair.Fill.Id, Pair.Payload) },
                    Pair.Payload,
                    Pair.Distance,
                    Pair.Cost * Pair.Payload));
            }
            Double MeanPayload = Keep == 0 ? 0.0 : PayloadSum / Keep;
            Double MeanHaul = PayloadSum <= 0.0 ? 0.0 : HaulSum / PayloadSum;
            ResidualPairPlan Plan = new ResidualPairPlan(
                Array.Empty<ResidualPairFlow>(),
                Trips,
                Array.Empty<ResidualPairRemainder>(),
                0.0,
                0.0,
                "balanced",
                0.0,
                Partials,
                MeanPayload,
                0.0,
                MeanHaul);
            return new ClusterSchedule(Plan, Pieces);
        }

        /// <summary>
        /// Cheapest estimated pair first. An equal estimate keeps the earlier piece id.
        /// </summary>
        /// <param name="Left">One pair.</param>
        /// <param name="Right">The other pair.</param>
        /// <returns>The sort order.</returns>
        private static Int32 CompareRanked
            (
            RankedPair Left,
            RankedPair Right
            )
        {
            Int32 Cost = Left.Cost.CompareTo(Right.Cost);
            if (Cost != 0)
            {
                return Cost;
            }
            Int32 Cut = String.CompareOrdinal(Left.Cut.Id, Right.Cut.Id);
            if (Cut != 0)
            {
                return Cut;
            }
            return String.CompareOrdinal(Left.Fill.Id, Right.Fill.Id);
        }

        /// <summary>One cut and fill pair before it becomes a trip.</summary>
        private struct RankedPair
        {
            /// <summary>Gets or sets haul seconds per bank cubic yard.</summary>
            public Double Cost;

            /// <summary>Gets or sets the yards the pair can move, at most one bowl.</summary>
            public Double Payload;

            /// <summary>Gets or sets the centroid haul in metres.</summary>
            public Double Distance;

            /// <summary>Gets or sets the cut piece.</summary>
            public ResidualPairPiece Cut;

            /// <summary>Gets or sets the fill piece.</summary>
            public ResidualPairPiece Fill;
        }

        /// <summary>
        /// Drives one batch and subtracts it from the residual.
        /// The rest of the schedule is not kept. The next loop plans again.
        /// </summary>
        /// <param name="ResidualMm">Residual the plan was built from.</param>
        /// <param name="Schedule">The plan and the pieces it paired.</param>
        /// <param name="Batch">The trips to drive, at most ten.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="CutLeft">Remaining cut.</param>
        /// <param name="FillLeft">Remaining fill.</param>
        /// <param name="BowlBcy">Pan capacity.</param>
        /// <param name="Passes">Routes to append.</param>
        /// <param name="ArrowX">Smoothed flow east.</param>
        /// <param name="ArrowY">Smoothed flow north.</param>
        internal static void ExecuteBatch
            (
            Double[,] ResidualMm,
            ClusterSchedule Schedule,
            IReadOnlyList<ResidualPairTrip> Batch,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            List<Pass> Passes,
            Double[,]? ArrowX = null,
            Double[,]? ArrowY = null
            )
        {
            List<ResidualHaul> Hauls = ResidualHauls.Build(
                ResidualMm,
                Grid.Valid,
                Schedule.Pieces,
                Batch,
                Grid,
                Grid.Compaction,
                CUT_TO_MM,
                ArrowX,
                ArrowY,
                BowlBcy);
            foreach (ResidualHaul Haul in Hauls)
            {
                Pass? Trip = ToPass(Haul, Grid, Ledger, CutLeft, FillLeft, BowlBcy);
                if (Trip != null)
                {
                    Passes.Add(Trip);
                }
            }
        }

        /// <summary>
        /// An empty plan used before the first round, so the report has a pairing.
        /// </summary>
        /// <returns>A balanced plan with no trips.</returns>
        private static ResidualPairPlan EmptyPlan()
        {
            return ResidualPairing.Plan(
                new List<ResidualPairPiece>(),
                1.0,
                1.0,
                1.0);
        }

        /// <summary>
        /// Signed residual in millimetres. Positive is cut still to move.
        /// </summary>
        /// <param name="CutLeft">Remaining cut depth in metres.</param>
        /// <param name="FillLeft">Remaining fill depth in metres.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The residual, indexed [row, col].</returns>
        internal static Double[,] ResidualMillimetres
            (
            Double[,] CutLeft,
            Double[,] FillLeft,
            Grid Grid
            )
        {
            Double[,] ResidualMm = new Double[Grid.Height, Grid.Width];
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    if (!Grid.Valid[Row, Col])
                    {
                        continue;
                    }
                    ResidualMm[Row, Col] = (CutLeft[Row, Col] - FillLeft[Row, Col]) * 1000.0;
                }
            }
            return ResidualMm;
        }

        /// <summary>
        /// Pieces whose volume is the depth down to the target, not the 50 mm floor.
        /// The floor still decided which cells joined the cluster.
        /// </summary>
        /// <param name="Clusters">The clustering of this round.</param>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The pieces pairing can schedule.</returns>
        internal static List<ResidualPairPiece> PiecesToTarget
            (
            ResidualClusterReport Clusters,
            Double[,] ResidualMm,
            Grid Grid
            )
        {
            List<ResidualPairPiece> Pieces = new List<ResidualPairPiece>();
            foreach (ResidualCluster Cluster in Clusters.Pieces)
            {
                Double Volume = ResidualHauls.LevelVolumeBcy(
                    Cluster.Cells,
                    Cluster.Sign,
                    ResidualMm,
                    Grid,
                    Grid.Compaction,
                    CUT_TO_MM);
                Double Mean = MeanDepthMm(Cluster, ResidualMm);
                if ((Volume <= 0.0) || (Mean <= 0.0))
                {
                    continue;
                }
                Pieces.Add(new ResidualPairPiece(
                    Cluster.Id,
                    Cluster.Sign,
                    Volume,
                    Mean,
                    Cluster.CentroidRow,
                    Cluster.CentroidCol,
                    Cluster.LongAxisLenM,
                    Cluster.Cells));
            }
            return Pieces;
        }

        /// <summary>
        /// Mean of the depth still outside the target. Bridged cells on grade add nothing.
        /// </summary>
        /// <param name="Piece">The piece.</param>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <returns>Millimetres. Zero when every cell is on the target.</returns>
        private static Double MeanDepthMm
            (
            ResidualCluster Piece,
            Double[,] ResidualMm
            )
        {
            if (Piece.Cells.Count == 0)
            {
                return 0.0;
            }
            Double Sum = 0.0;
            foreach ((Int32 Row, Int32 Col) Cell in Piece.Cells)
            {
                if ((Cell.Row < 0) || (Cell.Col < 0)
                    || (Cell.Row >= ResidualMm.GetLength(0))
                    || (Cell.Col >= ResidualMm.GetLength(1)))
                {
                    continue;
                }
                Double Depth = ResidualMm[Cell.Row, Cell.Col];
                if (Piece.Sign == "fill")
                {
                    Depth = -Depth;
                }
                if (Depth > CUT_TO_MM)
                {
                    Sum += Depth - CUT_TO_MM;
                }
            }
            return Sum / Piece.Cells.Count;
        }

        /// <summary>
        /// One route for a cluster trip. Lifts stay separate. The ledger sees
        /// equal cut and fill yards, and no cell is taken past the depth still left.
        /// </summary>
        /// <param name="Haul">The trip.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="CutLeft">Remaining cut. Reduced by the trip.</param>
        /// <param name="FillLeft">Remaining fill. Reduced by the trip.</param>
        /// <param name="BowlBcy">Pan capacity.</param>
        /// <returns>The route, or null when the trip moved nothing.</returns>
        private static Pass? ToPass
            (
            ResidualHaul Haul,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy
            )
        {
            Double MaxDepthM = Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts = FitCells(Haul.CutCells, CutLeft);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills = FitFill(
                Haul.FillCells, FillLeft, CutLeft, Ledger);
            if ((Cuts.Count == 0) || (Fills.Count == 0))
            {
                return null;
            }
            Double CutBcy = SumBcy(Cuts, Ledger.CutBcyPerDepthM);
            // The cluster yards use a slightly shorter metre-to-yard factor than the ledger.
            // Scale the trip down so a full bowl still fits the ledger capacity.
            if (CutBcy > (BowlBcy + 1e-8))
            {
                Double Scale = BowlBcy / CutBcy;
                ScaleDepths(Cuts, Scale);
                ScaleDepths(Fills, Scale);
                CutBcy = SumBcy(Cuts, Ledger.CutBcyPerDepthM);
            }
            Double FillBcy = SumBcy(Fills, Ledger.FillBcyPerDepthM);
            if ((FillBcy > 0.0) && (CutBcy > FillBcy))
            {
                // Growing a fill past the dirt still there would fail the ledger.
                // Shrink the cut to the fill when the last lift has no room left.
                Double ExtraM = (CutBcy - FillBcy) / Ledger.FillBcyPerDepthM;
                if (CanGrow(Fills, FillLeft, MaxDepthM, ExtraM))
                {
                    Int32 Last = Fills.Count - 1;
                    Fills[Last] = (Fills[Last].Cell, Fills[Last].DepthM + ExtraM);
                }
                else
                {
                    ScaleDepths(Cuts, FillBcy / CutBcy);
                }
            }
            else if ((CutBcy > 0.0) && (FillBcy > CutBcy))
            {
                ScaleDepths(Fills, CutBcy / FillBcy);
            }
            CutBcy = SumBcy(Cuts, Ledger.CutBcyPerDepthM);
            FillBcy = SumBcy(Fills, Ledger.FillBcyPerDepthM);
            // A scaled sum can sit a few ulps off the other side. Shave the larger
            // side so the ledger's balance check sees the same yards.
            Double GapBcy = CutBcy - FillBcy;
            if ((GapBcy > 1e-12) && (Cuts.Count > 0))
            {
                Int32 Last = Cuts.Count - 1;
                Double Trim = Math.Min(Cuts[Last].DepthM, GapBcy / Ledger.CutBcyPerDepthM);
                Cuts[Last] = (Cuts[Last].Cell, Cuts[Last].DepthM - Trim);
                CutBcy = SumBcy(Cuts, Ledger.CutBcyPerDepthM);
            }
            else if ((GapBcy < -1e-12) && (Fills.Count > 0))
            {
                Int32 Last = Fills.Count - 1;
                Double Trim = Math.Min(Fills[Last].DepthM, (-GapBcy) / Ledger.FillBcyPerDepthM);
                Fills[Last] = (Fills[Last].Cell, Fills[Last].DepthM - Trim);
            }
            Commit(Cuts, CutLeft);
            CommitFill(Fills, FillLeft, CutLeft);
            (Double X, Double Y) CutStart;
            (Double X, Double Y) CutEnd;
            (Double X, Double Y) FillStart;
            (Double X, Double Y) FillEnd;
            if (Haul.AreaBased)
            {
                // The haul runs from the cut piece's exit face to the fill piece's entry face.
                CutStart = (Haul.CutEntryX, Haul.CutEntryY);
                CutEnd = (Haul.CutExitX, Haul.CutExitY);
                FillStart = (Haul.FillEntryX, Haul.FillEntryY);
                FillEnd = (Haul.FillExitX, Haul.FillExitY);
            }
            else
            {
                CutStart = Grid.CellCenterXy(Cuts[0].Cell.Bx, Cuts[0].Cell.By);
                CutEnd = Grid.CellCenterXy(Cuts[Cuts.Count - 1].Cell.Bx, Cuts[Cuts.Count - 1].Cell.By);
                FillStart = Grid.CellCenterXy(Fills[0].Cell.Bx, Fills[0].Cell.By);
                FillEnd = Grid.CellCenterXy(Fills[Fills.Count - 1].Cell.Bx, Fills[Fills.Count - 1].Cell.By);
            }
            Pass Trip = new Pass(Cuts, Fills, CutBcy, Haul.HaulM, CutStart, CutEnd, FillStart, FillEnd);
            Trip.StackedLifts = true;
            // The cells are an area, not a ribbon. A ribbon mark would collapse them into one stroke.
            Trip.RibbonOrder = false;
            Trip.StripLoadTimeS = Haul.LoadSeconds;
            Trip.StripSpreadTimeS = Haul.SpreadSeconds;
            Trip.StripLoadDistanceM = Haul.LoadDistanceM;
            Trip.StripSpreadDistanceM = Haul.SpreadDistanceM;
            if (Haul.AreaBased)
            {
                Trip.AreaCutPasses = Haul.CutRuns;
                Trip.AreaFillPasses = Haul.FillRuns;
                Trip.AreaDirX = Haul.DirX;
                Trip.AreaDirY = Haul.DirY;
                Trip.AreaCutOutline = Haul.CutOutline;
                Trip.AreaFillOutline = Haul.FillOutline;
                Trip.AreaCutPiece = Haul.CutPieceId;
                Trip.AreaFillPiece = Haul.FillPieceId;
            }
            return Trip;
        }

        /// <summary>
        /// Subtracts a trip that was built on a copy of the field.
        /// The depths on the pass are the ones the ledger already accepted.
        /// </summary>
        /// <param name="Trip">The route to commit.</param>
        /// <param name="CutLeft">Remaining cut.</param>
        /// <param name="FillLeft">Remaining fill. Surplus fill is added to the cut.</param>
        internal static void ApplyBuiltPass
            (
            Pass Trip,
            Double[,] CutLeft,
            Double[,] FillLeft
            )
        {
            if (Trip == null) throw new ArgumentNullException(nameof(Trip));
            Commit(Trip.CutCells, CutLeft);
            CommitFill(Trip.FillCells, FillLeft, CutLeft);
        }

        /// <summary>
        /// Copies lifts that still fit in the remaining depth. The field is not changed yet.
        /// </summary>
        /// <param name="Lifts">Lifts from the haul, in drive order.</param>
        /// <param name="Left">Remaining depth for this sign.</param>
        /// <returns>The lifts that fit.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FitCells
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lifts,
            Double[,] Left
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Applied =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Dictionary<(Int32 Bx, Int32 By), Double> Used =
                new Dictionary<(Int32 Bx, Int32 By), Double>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Lifts)
            {
                if ((Lift.Cell.By < 0) || (Lift.Cell.Bx < 0)
                    || (Lift.Cell.By >= Left.GetLength(0))
                    || (Lift.Cell.Bx >= Left.GetLength(1)))
                {
                    continue;
                }
                Used.TryGetValue(Lift.Cell, out Double Already);
                Double Room = Left[Lift.Cell.By, Lift.Cell.Bx] - Already;
                Double Take = Math.Min(Lift.DepthM, Math.Max(0.0, Room));
                if (Take <= 1e-12)
                {
                    continue;
                }
                Used[Lift.Cell] = Already + Take;
                Applied.Add((Lift.Cell, Take));
            }
            return Applied;
        }

        /// <summary>
        /// True when the last lift can grow by <paramref name="ExtraM"/> and stay
        /// inside the cell's remaining depth and the pass depth cap.
        /// </summary>
        /// <param name="Lifts">Lifts already fitted.</param>
        /// <param name="Left">Remaining depth.</param>
        /// <param name="MaxDepthM">Deepest one lift may be.</param>
        /// <param name="ExtraM">Metres to add to the last lift.</param>
        /// <returns>True when the growth fits.</returns>
        private static Boolean CanGrow
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lifts,
            Double[,] Left,
            Double MaxDepthM,
            Double ExtraM
            )
        {
            if ((Lifts.Count == 0) || (ExtraM < 0.0))
            {
                return false;
            }
            (Int32 Bx, Int32 By) Cell = Lifts[Lifts.Count - 1].Cell;
            Double OnCell = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Lifts)
            {
                if (Lift.Cell == Cell)
                {
                    OnCell += Lift.DepthM;
                }
            }
            Double Room = Left[Cell.By, Cell.Bx] - OnCell;
            Double Last = Lifts[Lifts.Count - 1].DepthM;
            // Strict. A depth past the ledger, even by a nanometre, fails the plan.
            return (ExtraM <= Room) && ((Last + ExtraM) <= MaxDepthM);
        }

        /// <summary>
        /// Keeps fill that fits the hole, plus up to 10 mm on a cell the design left at grade.
        /// A cell that was a cut or a fill does not take surplus. That would exceed the ledger.
        /// </summary>
        /// <param name="Lifts">Spread lifts from the haul, in drive order.</param>
        /// <param name="FillLeft">Remaining hole, in metres.</param>
        /// <param name="CutLeft">Height already recorded above target, in metres.</param>
        /// <param name="Ledger">Original required depths. Surplus is only legal where both are zero.</param>
        /// <returns>The lifts that fit.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FitFill
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lifts,
            Double[,] FillLeft,
            Double[,] CutLeft,
            HaulWorkLedger Ledger
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Applied =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Dictionary<(Int32 Bx, Int32 By), Double> Used =
                new Dictionary<(Int32 Bx, Int32 By), Double>();
            Double CeilingM = StripLanes.SURPLUS_MAX_MM / 1000.0;
            if (Ledger.ToleranceM < CeilingM)
            {
                // The ledger only accepts finish-band fill up to the grade tolerance.
                CeilingM = Ledger.ToleranceM;
            }
            if (CeilingM < 0.0)
            {
                CeilingM = 0.0;
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Lifts)
            {
                if ((Lift.Cell.By < 0) || (Lift.Cell.Bx < 0)
                    || (Lift.Cell.By >= FillLeft.GetLength(0))
                    || (Lift.Cell.Bx >= FillLeft.GetLength(1)))
                {
                    continue;
                }
                Used.TryGetValue(Lift.Cell, out Double Already);
                Double Extra = 0.0;
                if ((Ledger.RequiredCutDepthM[Lift.Cell.By, Lift.Cell.Bx] <= 1e-9)
                    && (Ledger.RequiredFillDepthM[Lift.Cell.By, Lift.Cell.Bx] <= 1e-9))
                {
                    Double High = CutLeft[Lift.Cell.By, Lift.Cell.Bx];
                    if (High < 0.0)
                    {
                        High = 0.0;
                    }
                    Extra = CeilingM - High;
                    if (Extra < 0.0)
                    {
                        Extra = 0.0;
                    }
                }
                Double Room = FillLeft[Lift.Cell.By, Lift.Cell.Bx] - Already + Extra;
                Double Take = Math.Min(Lift.DepthM, Math.Max(0.0, Room));
                if (Take <= 1e-12)
                {
                    continue;
                }
                Used[Lift.Cell] = Already + Take;
                Applied.Add((Lift.Cell, Take));
            }
            return Applied;
        }

        /// <summary>
        /// Removes the fitted depths from the field. Called once the trip balances.
        /// </summary>
        /// <param name="Lifts">Final lifts.</param>
        /// <param name="Left">Remaining depth.</param>
        private static void Commit
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lifts,
            Double[,] Left
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Lifts)
            {
                Double Next = Left[Lift.Cell.By, Lift.Cell.Bx] - Lift.DepthM;
                // A rounding crumb must not flip a finished cell into the other sign.
                Left[Lift.Cell.By, Lift.Cell.Bx] = Next < 0.0 ? 0.0 : Next;
            }
        }

        /// <summary>
        /// Takes the hole out of the fill that is left, and records surplus as height above target.
        /// The next plan then sees the cell high and does not spread the same 10 mm again.
        /// </summary>
        /// <param name="Lifts">Final fill lifts.</param>
        /// <param name="FillLeft">Remaining hole.</param>
        /// <param name="CutLeft">Height above target. Surplus is added here.</param>
        private static void CommitFill
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lifts,
            Double[,] FillLeft,
            Double[,] CutLeft
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Lifts)
            {
                Double Hole = FillLeft[Lift.Cell.By, Lift.Cell.Bx];
                if (Hole < 0.0)
                {
                    Hole = 0.0;
                }
                Double IntoHole = Lift.DepthM;
                if (IntoHole > Hole)
                {
                    IntoHole = Hole;
                }
                FillLeft[Lift.Cell.By, Lift.Cell.Bx] = Hole - IntoHole;
                Double Extra = Lift.DepthM - IntoHole;
                if (Extra > 1e-12)
                {
                    CutLeft[Lift.Cell.By, Lift.Cell.Bx] += Extra;
                }
            }
        }

        /// <summary>
        /// Scales every lift. Used when the ledger's yard factor would put the trip over the bowl.
        /// </summary>
        /// <param name="Lifts">Lifts to scale in place.</param>
        /// <param name="Scale">Fraction of the depth to keep.</param>
        private static void ScaleDepths
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lifts,
            Double Scale
            )
        {
            for (Int32 Index = 0; Index < Lifts.Count; Index += 1)
            {
                Lifts[Index] = (Lifts[Index].Cell, Lifts[Index].DepthM * Scale);
            }
        }

        /// <summary>
        /// Bank yards in a list of lifts.
        /// </summary>
        /// <param name="Lifts">Lifts.</param>
        /// <param name="BcyPerDepthM">Ledger yards per metre of depth.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double SumBcy
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lifts,
            Double BcyPerDepthM
            )
        {
            Double Sum = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Lifts)
            {
                Sum += Lift.DepthM * BcyPerDepthM;
            }
            return Sum;
        }

        /// <summary>
        /// Prints the handoff. Nothing in the report decides whether to switch.
        /// </summary>
        /// <param name="Stop">Why the loop stopped, and the unexecuted tail on a trip limit.</param>
        /// <param name="Plan">The last pairing.</param>
        /// <param name="CutLeft">Cut still remaining.</param>
        /// <param name="FillLeft">Fill still remaining.</param>
        /// <param name="Grid">Field grid.</param>
        private static void Report
            (
            ClusterHaulStop Stop,
            ResidualPairPlan Plan,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Grid Grid
            )
        {
            Double[,] ResidualMm = ResidualMillimetres(CutLeft, FillLeft, Grid);
            // The work mask is every valid cell. There is no boundary ring left out.
            Boolean[,] Interior = Grid.Valid;
            Double LocalCut = Plan.BalanceKind == ResidualPairing.LOCAL_ID ? Plan.BalanceBcy : 0.0;
            Double ShortFill = Plan.BalanceKind == ResidualPairing.SHORT_ID ? Plan.BalanceBcy : 0.0;
            List<String> Lines = new List<String>();
            // The reason is first. A trip limit then names the plan that was not finished.
            Say(Lines, String.Format(
                CultureInfo.InvariantCulture,
                "cluster hauling stopped: {0}",
                Stop.Reason));
            if (Stop.Reason == TRIP_LIMIT)
            {
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "remaining plan: {0} trips, {1:R} bcy",
                    Stop.RemainingTrips,
                    Stop.RemainingBcy));
            }
            Say(Lines, String.Format(CultureInfo.InvariantCulture, "trips: {0}", Stop.TripsDone));
            Say(Lines, String.Format(CultureInfo.InvariantCulture, "LOCAL cut: {0:R} bcy", LocalCut));
            Say(Lines, String.Format(CultureInfo.InvariantCulture, "SHORT fill: {0:R} bcy", ShortFill));
            if (Plan.Remainders.Count == 0)
            {
                Say(Lines, "dropped under 0.25 bowl: none");
            }
            foreach (ResidualPairRemainder Remainder in Plan.Remainders)
            {
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "dropped under 0.25 bowl: {0} {1} {2:R} bcy",
                    Remainder.CutPiece,
                    Remainder.FillPiece,
                    Remainder.VolumeBcy));
            }
            AppendFieldGroups(Lines, ResidualMm, Grid, Interior);
            // A test host can hide the console. The same lines stay in the temp file.
            // Parallel tests must not fail the haul because the file is briefly open.
            lock (ReportGate)
            {
                String ReportPath = Path.Combine(Path.GetTempPath(), "cluster-haul-report.txt");
                try
                {
                    File.WriteAllLines(ReportPath, Lines);
                }
                catch (IOException)
                {
                    File.WriteAllLines(
                        Path.Combine(Path.GetTempPath(), "cluster-haul-report-" + Guid.NewGuid().ToString("N") + ".txt"),
                        Lines);
                }
            }
        }

        /// <summary>
        /// Writes the cells past 50 mm and the edge groups. Thin-cut uses the same list.
        /// </summary>
        /// <param name="Lines">Lines already printed.</param>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Interior">Cells counted in the beyond-50 groups. The edge is valid and outside this mask.</param>
        internal static void AppendFieldGroups
            (
            List<String> Lines,
            Double[,] ResidualMm,
            Grid Grid,
            Boolean[,] Interior
            )
        {
            List<Patch> Deep = GroupsBeyondBand(ResidualMm, Interior, Grid, true);
            Double DeepBcy = 0.0;
            Int32 DeepCells = 0;
            foreach (Patch Group in Deep)
            {
                DeepBcy += Group.Bcy;
                DeepCells += Group.Cells;
            }
            Say(Lines, String.Format(
                CultureInfo.InvariantCulture,
                "beyond +/-50 mm on E: {0} cells, {1:R} bcy",
                DeepCells,
                DeepBcy));
            Int32 Listed = Math.Min(10, Deep.Count);
            for (Int32 Index = 0; Index < Listed; Index += 1)
            {
                Patch Group = Deep[Index];
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "  group {0}: {1} cells, {2:R} bcy, peak {3:F1} mm",
                    Index + 1,
                    Group.Cells,
                    Group.Bcy,
                    Group.PeakMm));
            }
            Boolean[,] Edge = new Boolean[Grid.Height, Grid.Width];
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    Edge[Row, Col] = Grid.Valid[Row, Col] && !Interior[Row, Col];
                }
            }
            List<Patch> Edges = GroupsBeyondBand(ResidualMm, Edge, Grid, false);
            Boolean Pending = false;
            for (Int32 Index = 0; Index < Edges.Count; Index += 1)
            {
                Patch Group = Edges[Index];
                String Status = Group.Pending ? "pending" : "clear";
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "edge group {0}: {1}, peak {2:F1} mm, {3:R} bcy",
                    Index + 1,
                    Status,
                    Group.PeakMm,
                    Group.Bcy));
                if (Group.Pending)
                {
                    Pending = true;
                    Say(Lines, String.Format(
                        CultureInfo.InvariantCulture,
                        "warning: edge group {0} is pending",
                        Index + 1));
                }
            }
            if (!Pending)
            {
                Say(Lines, "edge groups: none pending");
            }
        }

        /// <summary>
        /// Prints one handoff line and keeps it for the temp copy.
        /// </summary>
        /// <param name="Lines">Lines already printed.</param>
        /// <param name="Text">The next line.</param>
        private static void Say
            (
            List<String> Lines,
            String Text
            )
        {
            Lines.Add(Text);
            Console.WriteLine(Text);
        }

        /// <summary>
        /// Connected groups on a mask. Interior groups are only the cells past 50 mm.
        /// Edge groups are the whole component, and pending means one cell is past 50 mm.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Cells allowed in the groups.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="OnlyBeyondBand">True for the interior report. False for edge groups.</param>
        /// <returns>Groups, largest bank yards first.</returns>
        private static List<Patch> GroupsBeyondBand
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Grid Grid,
            Boolean OnlyBeyondBand
            )
        {
            Int32 Rows = ResidualMm.GetLength(0);
            Int32 Cols = ResidualMm.GetLength(1);
            Boolean[,] Seen = new Boolean[Rows, Cols];
            List<Patch> Groups = new List<Patch>();
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (Seen[Row, Col] || !Mask[Row, Col])
                    {
                        continue;
                    }
                    if (OnlyBeyondBand && (Math.Abs(ResidualMm[Row, Col]) <= REPORT_BAND_MM))
                    {
                        continue;
                    }
                    Patch Group = Flood(ResidualMm, Mask, Seen, Grid, Row, Col, OnlyBeyondBand);
                    if (Group.Cells > 0)
                    {
                        Groups.Add(Group);
                    }
                }
            }
            Groups.Sort((Patch Left, Patch Right) => Right.Bcy.CompareTo(Left.Bcy));
            return Groups;
        }

        /// <summary>
        /// One 8-connected component.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Cells allowed in the group.</param>
        /// <param name="Seen">Cells already given to a group.</param>
        /// <param name="Grid">Field grid. The yard conversion uses its area and compaction.</param>
        /// <param name="StartRow">Seed row.</param>
        /// <param name="StartCol">Seed column.</param>
        /// <param name="OnlyBeyondBand">True to skip cells inside the 50 mm band.</param>
        /// <returns>The component.</returns>
        private static Patch Flood
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Boolean[,] Seen,
            Grid Grid,
            Int32 StartRow,
            Int32 StartCol,
            Boolean OnlyBeyondBand
            )
        {
            Int32 Rows = ResidualMm.GetLength(0);
            Int32 Cols = ResidualMm.GetLength(1);
            Patch Group = new Patch();
            Queue<(Int32 Row, Int32 Col)> Waiting = new Queue<(Int32 Row, Int32 Col)>();
            Waiting.Enqueue((StartRow, StartCol));
            Seen[StartRow, StartCol] = true;
            while (Waiting.Count > 0)
            {
                (Int32 Row, Int32 Col) Cell = Waiting.Dequeue();
                Double Mm = ResidualMm[Cell.Row, Cell.Col];
                Double AbsMm = Math.Abs(Mm);
                if (!OnlyBeyondBand || (AbsMm > REPORT_BAND_MM))
                {
                    Group.Cells += 1;
                    if (AbsMm > Group.PeakMm)
                    {
                        Group.PeakMm = AbsMm;
                    }
                    if (AbsMm > REPORT_BAND_MM)
                    {
                        Group.Pending = true;
                    }
                    Double OutsideMm = OnlyBeyondBand ? (AbsMm - REPORT_BAND_MM) : AbsMm;
                    Double Metres = OutsideMm / 1000.0;
                    Double Bcy = Metres * Grid.AreaM2 * ResidualClusters.M3_TO_CY;
                    if (Mm < 0.0)
                    {
                        Bcy *= Grid.Compaction;
                    }
                    Group.Bcy += Bcy;
                }
                for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                {
                    for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                    {
                        if ((Dx == 0) && (Dy == 0))
                        {
                            continue;
                        }
                        Int32 NextRow = Cell.Row + Dy;
                        Int32 NextCol = Cell.Col + Dx;
                        if ((NextRow < 0) || (NextCol < 0) || (NextRow >= Rows) || (NextCol >= Cols))
                        {
                            continue;
                        }
                        if (Seen[NextRow, NextCol] || !Mask[NextRow, NextCol])
                        {
                            continue;
                        }
                        if (OnlyBeyondBand && (Math.Abs(ResidualMm[NextRow, NextCol]) <= REPORT_BAND_MM))
                        {
                            continue;
                        }
                        Seen[NextRow, NextCol] = true;
                        Waiting.Enqueue((NextRow, NextCol));
                    }
                }
            }
            return Group;
        }

        /// <summary>
        /// One connected patch in the handoff report.
        /// </summary>
        private sealed class Patch
        {
            /// <summary>Gets or sets the cell count.</summary>
            public Int32 Cells { get; set; }

            /// <summary>Gets or sets the bank yards in the patch.</summary>
            public Double Bcy { get; set; }

            /// <summary>Gets or sets the furthest cell from the target, in millimetres.</summary>
            public Double PeakMm { get; set; }

            /// <summary>Gets or sets whether any cell is more than 50 mm from the target.</summary>
            public Boolean Pending { get; set; }
        }
    }

    /// <summary>
    /// Why cluster hauling stopped, and the trips left on the plan when the trip limit fired.
    /// </summary>
    public sealed class ClusterHaulStop
    {
        /// <summary>
        /// Creates a stop.
        /// </summary>
        /// <param name="Reason"><see cref="ClusterHauling.NO_HAULABLE_WORK"/> or <see cref="ClusterHauling.TRIP_LIMIT"/>.</param>
        /// <param name="TripsDone">Trips executed before the stop.</param>
        /// <param name="RemainingTrips">Trips on the last plan that were not executed. Zero unless the trip limit fired.</param>
        /// <param name="RemainingBcy">Bank yards on those remaining trips.</param>
        public ClusterHaulStop
            (
            String Reason,
            Int32 TripsDone,
            Int32 RemainingTrips,
            Double RemainingBcy
            )
        {
            this.Reason = Reason;
            this.TripsDone = TripsDone;
            this.RemainingTrips = RemainingTrips;
            this.RemainingBcy = RemainingBcy;
        }

        /// <summary>Gets the stop reason.</summary>
        public String Reason { get; }

        /// <summary>Gets how many trips were executed.</summary>
        public Int32 TripsDone { get; }

        /// <summary>Gets the unexecuted trips on the plan that hit the trip limit.</summary>
        public Int32 RemainingTrips { get; }

        /// <summary>Gets the bank yards on the unexecuted trips.</summary>
        public Double RemainingBcy { get; }
    }

    /// <summary>
    /// One cluster plan and the pieces it was paired from.
    /// </summary>
    internal sealed class ClusterSchedule
    {
        /// <summary>
        /// Creates a schedule.
        /// </summary>
        /// <param name="Plan">The pairing.</param>
        /// <param name="Pieces">Pieces the trips name. Empty when a test supplies the plan.</param>
        public ClusterSchedule
            (
            ResidualPairPlan Plan,
            IReadOnlyList<ResidualPairPiece> Pieces
            )
        {
            if (Plan == null) throw new ArgumentNullException(nameof(Plan));
            if (Pieces == null) throw new ArgumentNullException(nameof(Pieces));
            this.Plan = Plan;
            this.Pieces = Pieces;
        }

        /// <summary>Gets the pairing.</summary>
        public ResidualPairPlan Plan { get; }

        /// <summary>Gets the pieces.</summary>
        public IReadOnlyList<ResidualPairPiece> Pieces { get; }
    }
}
