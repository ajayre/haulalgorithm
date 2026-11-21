using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Routed pass order and deadhead summary.
    /// </summary>
    public sealed class Route
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="Route"/> class.
        /// </summary>
        /// <param name="Order">Indices into original pass list.</param>
        /// <param name="OrderedPasses">Passes in routed order.</param>
        /// <param name="TotalDeadheadM">Total deadhead distance.</param>
        public Route
            (
            List<Int32> Order,
            List<Pass> OrderedPasses,
            Double TotalDeadheadM
            )
        {
            // Persist route order plus extracted pass sequence.
            this.Order = Order;
            this.OrderedPasses = OrderedPasses;
            this.TotalDeadheadM = TotalDeadheadM;
        }

        /// <summary>
        /// Gets or sets original pass indices in routed order.
        /// </summary>
        public List<Int32> Order { get; set; }

        /// <summary>
        /// Gets or sets ordered pass list.
        /// </summary>
        public List<Pass> OrderedPasses { get; set; }

        /// <summary>
        /// Gets or sets total deadhead distance in metres.
        /// </summary>
        public Double TotalDeadheadM { get; set; }
    }

    /// <summary>
    /// Pass sequencing and local route refinement.
    /// </summary>
    static public class Routing
    {
        private const Double DEFAULT_UNLOADED_TRAVEL_SPEED_MPH = 13.0;
        private const Double DEFAULT_TURN_SPEED_MPH = 4.0;
        private const Double DEFAULT_TURNING_CIRCLE_FT = Constants.DEFAULT_TURNING_CIRCLE_FT;

        /// <summary>
        /// Reusable direction-aware transition evaluator for field-wide audits
        /// and optimizers.  It uses the same proxy model as route sequencing.
        /// </summary>
        public sealed class TransitionCostEvaluator
        {
            private readonly _TransitionModel Model;

            /// <summary>Initializes one immutable transition evaluator.</summary>
            public TransitionCostEvaluator
                (
                Boolean GradeReturn = true,
                Boolean DirectionAware = true,
                Double UnloadedTravelSpeedMph = DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
                Double TurnSpeedMph = DEFAULT_TURN_SPEED_MPH,
                Double TurningCircleFt = DEFAULT_TURNING_CIRCLE_FT,
                ObstacleField? Obstacles = null,
                Boolean OrderByDistance = false
                )
            {
                Model = new _TransitionModel(
                    GradeReturn,
                    DirectionAware,
                    UnloadedTravelSpeedMph * Constants.MPH_TO_MPS,
                    TurnSpeedMph * Constants.MPH_TO_MPS,
                    (TurningCircleFt * Constants.FEET_TO_METERS) / 2.0,
                    Obstacles ?? EMPTY_OBSTACLES,
                    OrderByDistance);
            }

            /// <summary>
            /// Gets the transition cost. Seconds, unless this evaluator was
            /// constructed to score metres.
            /// </summary>
            public Double CostSeconds(Pass First, Pass Second) =>
                Model.Cost(First, Second);
        }

        /// <summary>
        /// Sequences passes using nearest-neighbor then optional 2-opt.
        /// </summary>
        /// <param name="Passes">Pass list.</param>
        /// <param name="TwoOptCap">Maximum pass count for 2-opt.</param>
        /// <param name="GradeReturn">Whether grade-return is enabled.</param>
        /// <param name="DirectionAware">Whether transition cost is heading-aware.</param>
        /// <param name="UnloadedTravelSpeedMph">Unloaded travel speed in mph.</param>
        /// <param name="TurnSpeedMph">Turn speed mph.</param>
        /// <param name="TurningCircleFt">Turning circle diameter feet.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <param name="StartCandidateCount">
        /// Maximum deterministic open-route start candidates.
        /// </param>
        /// <param name="OrderByDistance">
        /// When true, rank the next load by metres of hop plus turn arc.
        /// When false, rank it by drive time plus turn time.
        /// </param>
        /// <returns>Routed order and deadhead total.</returns>
        static public Route SequencePasses
            (
            List<Pass> Passes,
            Int32 TwoOptCap = 1500,
            Boolean GradeReturn = true,
            Boolean DirectionAware = true,
            Double UnloadedTravelSpeedMph = DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
            Double TurnSpeedMph = DEFAULT_TURN_SPEED_MPH,
            Double TurningCircleFt = DEFAULT_TURNING_CIRCLE_FT,
            ObstacleField? Obstacles = null,
            Int32 StartCandidateCount = 1,
            Boolean OrderByDistance = false
            )
        {
            // Handle degenerate list sizes up front.
            Int32 N = Passes.Count;
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            if (N == 0)
            {
                return new Route(new List<Int32>(), new List<Pass>(), 0.0);
            }

            if (N == 1)
            {
                return new Route(new List<Int32> { 0 }, new List<Pass>(Passes), 0.0);
            }

            _TransitionModel Model = new _TransitionModel
                (
                GradeReturn,
                DirectionAware,
                UnloadedTravelSpeedMph * Constants.MPH_TO_MPS,
                TurnSpeedMph * Constants.MPH_TO_MPS,
                (TurningCircleFt * Constants.FEET_TO_METERS) / 2.0,
                ActiveObstacles,
                OrderByDistance
                );

            // Open routes have no fixed depot, so evaluate deterministic spatial
            // extrema rather than forcing allocation item zero to be first.
            List<Int32> Starts = StartCandidates(
                Passes, Math.Max(1, StartCandidateCount));
            List<Int32>? Order = null;
            Double BestObjective = Double.PositiveInfinity;
            foreach (Int32 Start in Starts)
            {
                List<Int32> CandidateOrder = NearestNeighborOrder(
                    Passes, Model, Start, StartCandidateCount <= 1);
                Double Objective = RouteObjective(
                    Model,
                    CandidateOrder.Select(Index => Passes[Index]).ToList());
                if ((Objective < BestObjective - Constants.NUMERIC_COMPARISON_TOLERANCE)
                    || ((Math.Abs(Objective - BestObjective) <= Constants.NUMERIC_COMPARISON_TOLERANCE)
                        && ((Order == null)
                            || LexicographicallyLess(CandidateOrder, Order))))
                {
                    Order = CandidateOrder;
                    BestObjective = Objective;
                }
            }
            Order ??= NearestNeighborOrder(
                Passes, Model, 0, StartCandidateCount <= 1);

            if (!DirectionAware && (N <= TwoOptCap))
            {
                Order = TwoOpt(Passes, Order, 30, GradeReturn);
            }

            Double Total = 0.0;
            for (Int32 Index = 0; Index < (Order.Count - 1); Index += 1)
            {
                Total += Deadhead(Passes[Order[Index]], Passes[Order[Index + 1]], GradeReturn, ActiveObstacles);
            }

            List<Pass> Ordered = new List<Pass>();
            foreach (Int32 Index in Order)
            {
                Ordered.Add(Passes[Index]);
            }

            return new Route(Order, Ordered, Total);
        }

        /// <summary>
        /// Executes the start candidates operation.
        /// </summary>
        /// <param name="Passes">The passes used by the operation.</param>
        /// <param name="MaximumCount">Count limit applied by the operation.</param>
        /// <returns>The resulting start candidates.</returns>

        private static List<Int32> StartCandidates
            (
            IReadOnlyList<Pass> Passes,
            Int32 MaximumCount
            )
        {
            IEnumerable<(Int32 Index, Double X, Double Y)> Points = Passes
                .Select((Pass, Index) =>
                    (Index, Pass.CutStartXy.X, Pass.CutStartXy.Y));
            List<Int32> Candidates = new List<Int32> { 0 };
            if (MaximumCount <= 1) return Candidates;
            // Adds a unique candidate index while respecting the search limit.

            void Add(Int32 Index)
            {
                if (!Candidates.Contains(Index)
                    && (Candidates.Count < MaximumCount))
                {
                    Candidates.Add(Index);
                }
            }
            Add(Points.OrderBy(Point => Point.X)
                .ThenBy(Point => Point.Index).First().Index);
            Add(Points.OrderByDescending(Point => Point.X)
                .ThenBy(Point => Point.Index).First().Index);
            Add(Points.OrderBy(Point => Point.Y)
                .ThenBy(Point => Point.Index).First().Index);
            Add(Points.OrderByDescending(Point => Point.Y)
                .ThenBy(Point => Point.Index).First().Index);
            Add(Points.OrderBy(Point => Point.X + Point.Y)
                .ThenBy(Point => Point.Index).First().Index);
            Add(Points.OrderByDescending(Point => Point.X + Point.Y)
                .ThenBy(Point => Point.Index).First().Index);
            Add(Points.OrderBy(Point => Point.X - Point.Y)
                .ThenBy(Point => Point.Index).First().Index);
            return Candidates;
        }

        /// <summary>
        /// Orders the remaining loads by the cheapest hop from the load just driven.
        /// A straight hop longer than the best full cost cannot win, so those
        /// loads skip the turn-arc cost. The load that wins is the same one a full scan picks.
        /// </summary>
        /// <param name="Passes">Loads still to order.</param>
        /// <param name="Model">Hop cost, including the turn arc.</param>
        /// <param name="Start">Index of the first load.</param>
        /// <param name="PreserveLegacyTies">When true, an equal cost keeps the earlier index already chosen.</param>
        /// <returns>Load indexes in drive order.</returns>
        private static List<Int32> NearestNeighborOrder
            (
            IReadOnlyList<Pass> Passes,
            _TransitionModel Model,
            Int32 Start,
            Boolean PreserveLegacyTies
            )
        {
            return NeighborOrder(Passes, Model, Start, PreserveLegacyTies, SkipFarHops: true);
        }

        /// <summary>
        /// Checks the far-hop skip against a full cost scan.
        /// A miss here would drive the loads in a different order.
        /// </summary>
        /// <returns>A description of the first order that disagrees, or null when every case matches.</returns>
        internal static String? NeighborOrderMismatch
            (
            )
        {
            List<Pass> Passes = new List<Pass>();
            for (Int32 Index = 0; Index < 36; Index += 1)
            {
                Double X = (Index % 6) * 40.0;
                Double Y = (Index / 6) * 35.0;
                // Two loads share a start so an equal hop has to keep the same index.
                if (Index == 8)
                {
                    X = 0.0;
                    Y = 0.0;
                }
                Double Heading = (Index % 5) * 0.4;
                Passes.Add(new Pass(
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                    1.0,
                    0.0,
                    (X, Y),
                    (X + (10.0 * Math.Cos(Heading)), Y + (10.0 * Math.Sin(Heading))),
                    (X + 12.0, Y + 4.0),
                    (X + 18.0, Y + (6.0 * Math.Cos(Heading)))));
            }
            _TransitionModel[] Models =
            {
                new _TransitionModel(
                    GradeReturn: true,
                    DirectionAware: true,
                    ReturnV: 4.0,
                    TurnV: 1.5,
                    R: 6.0,
                    Obstacles: EMPTY_OBSTACLES,
                    OrderByDistance: true),
                new _TransitionModel(
                    GradeReturn: true,
                    DirectionAware: true,
                    ReturnV: 4.0,
                    TurnV: 1.5,
                    R: 6.0,
                    Obstacles: EMPTY_OBSTACLES,
                    OrderByDistance: false),
            };
            Boolean[] TieModes = { true, false };
            foreach (_TransitionModel Model in Models)
            {
                foreach (Boolean PreserveLegacyTies in TieModes)
                {
                    for (Int32 Start = 0; Start < Passes.Count; Start += 1)
                    {
                        List<Int32> Fast = NeighborOrder(
                            Passes, Model, Start, PreserveLegacyTies, SkipFarHops: true);
                        List<Int32> Full = NeighborOrder(
                            Passes, Model, Start, PreserveLegacyTies, SkipFarHops: false);
                        for (Int32 Index = 0; Index < Fast.Count; Index += 1)
                        {
                            if (Fast[Index] == Full[Index]) continue;
                            return String.Format(
                                System.Globalization.CultureInfo.InvariantCulture,
                                "distance={0} legacy={1} start={2} index={3} fast={4} full={5}",
                                Model.OrderByDistance,
                                PreserveLegacyTies,
                                Start,
                                Index,
                                Fast[Index],
                                Full[Index]);
                        }
                    }
                }
            }
            return null;
        }

        /// <summary>
        /// Builds one nearest-neighbor order. The far-hop skip is the picture-run
        /// path. The full scan is the check that the skip did not change the winner.
        /// </summary>
        /// <param name="Passes">Loads still to order.</param>
        /// <param name="Model">Hop cost, including the turn arc.</param>
        /// <param name="Start">Index of the first load.</param>
        /// <param name="PreserveLegacyTies">When true, an equal cost keeps the earlier index already chosen.</param>
        /// <param name="SkipFarHops">When true, a straight hop past the best full cost is not priced.</param>
        /// <returns>Load indexes in drive order.</returns>
        private static List<Int32> NeighborOrder
            (
            IReadOnlyList<Pass> Passes,
            _TransitionModel Model,
            Int32 Start,
            Boolean PreserveLegacyTies,
            Boolean SkipFarHops
            )
        {
            Boolean[] Unvisited = Enumerable.Repeat(true, Passes.Count)
                .ToArray();
            List<Int32> Order = new List<Int32>(Passes.Count) { Start };
            Unvisited[Start] = false;
            Int32 Current = Start;
            // Extra metres kept in the hop test so a rounding error cannot
            // drop the load a full scan would have picked.
            const Double HopMarginM = 1e-4;
            for (Int32 Step = 1; Step < Passes.Count; Step += 1)
            {
                Double BestCost = Double.PositiveInfinity;
                Int32 Best = -1;
                Double Ceiling = Double.PositiveInfinity;
                (Double X, Double Y) Exit = default;
                if (SkipFarHops)
                {
                    // The closest straight hop has a full cost. Any load farther
                    // than that cost is a longer drive even before its turn arc.
                    Exit = ExitXy(Passes[Current], Model.GradeReturn);
                    Int32 Near = -1;
                    Double NearSq = Double.PositiveInfinity;
                    for (Int32 Candidate = 0; Candidate < Passes.Count; Candidate += 1)
                    {
                        if (!Unvisited[Candidate]) continue;
                        (Double X, Double Y) Cut = Passes[Candidate].CutStartXy;
                        Double Dx = Cut.X - Exit.X;
                        Double Dy = Cut.Y - Exit.Y;
                        Double DistSq = (Dx * Dx) + (Dy * Dy);
                        if (DistSq < NearSq)
                        {
                            NearSq = DistSq;
                            Near = Candidate;
                        }
                    }
                    Ceiling = Model.Cost(Passes[Current], Passes[Near]);
                }
                // A time cost is seconds. The straight hop is metres, so the
                // bound is how long that hop takes to drive. A distance cost
                // is already metres, and the turn arc only adds to it.
                Double MetresPerCost = 1.0;
                if (SkipFarHops
                    && Model.DirectionAware
                    && !Model.OrderByDistance
                    && (Model.ReturnV > 0.0))
                {
                    MetresPerCost = Model.ReturnV;
                }
                for (Int32 Candidate = 0; Candidate < Passes.Count; Candidate += 1)
                {
                    if (!Unvisited[Candidate]) continue;
                    if (SkipFarHops)
                    {
                        Double Cap = (Best < 0) ? Ceiling : Math.Min(Ceiling, BestCost);
                        Double Limit = Cap
                            + Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE
                            + HopMarginM;
                        (Double X, Double Y) Cut = Passes[Candidate].CutStartXy;
                        Double Dx = Cut.X - Exit.X;
                        Double Dy = Cut.Y - Exit.Y;
                        Double LimitMetres = Limit * MetresPerCost;
                        if (((Dx * Dx) + (Dy * Dy)) > (LimitMetres * LimitMetres)) continue;
                    }
                    Double Cost = Model.Cost(
                        Passes[Current], Passes[Candidate]);
                    Boolean Better = PreserveLegacyTies
                        ? Cost < BestCost
                        : (Cost < BestCost - Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                            || ((Math.Abs(Cost - BestCost) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                                && ((Best < 0) || (Candidate < Best)));
                    if (Better)
                    {
                        BestCost = Cost;
                        Best = Candidate;
                    }
                }
                Order.Add(Best);
                Unvisited[Best] = false;
                Current = Best;
            }
            return Order;
        }

        /// <summary>
        /// Executes the lexicographically less operation.
        /// </summary>
        /// <param name="First">The first used by the operation.</param>
        /// <param name="Second">The second used by the operation.</param>
        /// <returns>The resulting lexicographically less.</returns>

        private static Boolean LexicographicallyLess
            (
            IReadOnlyList<Int32> First,
            IReadOnlyList<Int32> Second
            )
        {
            for (Int32 Index = 0; Index < First.Count; Index += 1)
            {
                if (First[Index] == Second[Index]) continue;
                return First[Index] < Second[Index];
            }
            return false;
        }

        /// <summary>Calculates deadhead for a fixed pass order without changing it.</summary>
        /// <param name="Passes">Passes in their final execution order.</param>
        /// <param name="GradeReturn">Whether each pass exits after reverse grading.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <returns>Total inter-pass deadhead distance in metres.</returns>
        static public Double CalculateDeadhead
            (
            IReadOnlyList<Pass> Passes,
            Boolean GradeReturn = true,
            ObstacleField? Obstacles = null
            )
        {
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            Double TotalM = 0.0;
            for (Int32 Index = 0; Index < (Passes.Count - 1); Index += 1)
            {
                TotalM += DeadheadDistance(
                    Passes[Index],
                    Passes[Index + 1],
                    GradeReturn,
                    ActiveObstacles);
            }
            return TotalM;
        }

        /// <summary>
        /// Refines route with local relocate moves.
        /// </summary>
        /// <param name="Route">Route to mutate.</param>
        /// <param name="Window">Search window.</param>
        /// <param name="GradeReturn">Grade-return flag.</param>
        /// <param name="DirectionAware">Direction-aware cost flag.</param>
        /// <param name="UnloadedTravelSpeedMph">Unloaded travel speed in mph.</param>
        /// <param name="TurnSpeedMph">Turn speed mph.</param>
        /// <param name="TurningCircleFt">Turning circle diameter.</param>
        /// <param name="MaxRounds">Max refinement rounds.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <param name="OrderByDistance">
        /// When true, relocate moves are scored in metres. When false, in seconds.
        /// </param>
        /// <returns>Before objective, after objective, and move count.</returns>
        static public (Double Before, Double After, Int32 Moves) RefineRouteOrder
            (
            Route Route,
            Int32 Window = 50,
            Boolean GradeReturn = true,
            Boolean DirectionAware = true,
            Double UnloadedTravelSpeedMph = DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
            Double TurnSpeedMph = DEFAULT_TURN_SPEED_MPH,
            Double TurningCircleFt = DEFAULT_TURNING_CIRCLE_FT,
            Int32 MaxRounds = 12,
            ObstacleField? Obstacles = null,
            Boolean OrderByDistance = false
            )
        {
            // Use relocate (or-opt) moves which preserve segment orientation.
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            _TransitionModel Model = new _TransitionModel
                (
                GradeReturn,
                DirectionAware,
                UnloadedTravelSpeedMph * Constants.MPH_TO_MPS,
                TurnSpeedMph * Constants.MPH_TO_MPS,
                (TurningCircleFt * Constants.FEET_TO_METERS) / 2.0,
                ActiveObstacles,
                OrderByDistance
                );

            List<Pass> Seq = Route.OrderedPasses;
            Double Before = RouteObjective(Model, Seq);
            if ((Seq.Count < 3) || (Window < 2))
            {
                return (Before, Before, 0);
            }

            List<(Pass Pass, Int32 Id)> Tagged = new List<(Pass Pass, Int32 Id)>();
            for (Int32 Index = 0; Index < Seq.Count; Index += 1)
            {
                Tagged.Add((Seq[Index], Route.Order[Index]));
            }

            Int32 Moves = 0;
            // One sequence for the whole refine. Rebuilding it at every stop
            // copied every load once per load, and that copy was the wait
            // before a picture run could draw. A move is written here too,
            // so the next stop still sees the same order.
            List<Pass> Cur = new List<Pass>(Tagged.Count);
            for (Int32 Index = 0; Index < Tagged.Count; Index += 1)
            {
                Cur.Add(Tagged[Index].Pass);
            }
            for (Int32 Round = 0; Round < MaxRounds; Round += 1)
            {
                Boolean Improved = false;
                Int32 K = 0;
                while (K < Tagged.Count)
                {
                    Int32 Lo = Math.Max(0, K - Window);
                    Int32 Hi = Math.Min(Tagged.Count - 1, K + Window);
                    Int32 BestD = K;
                    Double BestDelta = -1e-7;
                    for (Int32 D = Lo; D <= Hi; D += 1)
                    {
                        if (D == K)
                        {
                            continue;
                        }

                        Double Delta = RelocateDelta(Model, Cur, K, D);
                        if (Delta < BestDelta)
                        {
                            BestDelta = Delta;
                            BestD = D;
                        }
                    }

                    if (BestD != K)
                    {
                        (Pass Pass, Int32 Id) Item = Tagged[K];
                        Tagged.RemoveAt(K);
                        Tagged.Insert(BestD, Item);
                        // Same shift as the tagged route, so the next delta
                        // reads the order a rebuilt list would have had.
                        Pass Moved = Cur[K];
                        Cur.RemoveAt(K);
                        Cur.Insert(BestD, Moved);
                        Moves += 1;
                        Improved = true;
                    }

                    K += 1;
                }

                if (!Improved)
                {
                    break;
                }
            }

            Route.OrderedPasses = new List<Pass>();
            Route.Order = new List<Int32>();
            foreach ((Pass Pass, Int32 Id) Item in Tagged)
            {
                Route.OrderedPasses.Add(Item.Pass);
                Route.Order.Add(Item.Id);
            }

            Double Deadhead = 0.0;
            for (Int32 Index = 0; Index < (Route.OrderedPasses.Count - 1); Index += 1)
            {
                Deadhead += DeadheadDistance(Route.OrderedPasses[Index], Route.OrderedPasses[Index + 1], GradeReturn, ActiveObstacles);
            }

            Route.TotalDeadheadM = Deadhead;
            Double After = RouteObjective(Model, Route.OrderedPasses);
            return (Before, After, Moves);
        }

        /// <summary>
        /// Executes the exit xy operation.
        /// </summary>
        /// <param name="P">The p used by the operation.</param>
        /// <param name="GradeReturn">The grade return used by the operation.</param>
        /// <returns>The resulting exit xy.</returns>

        static private (Double X, Double Y) ExitXy
            (
            Pass P,
            Boolean GradeReturn
            )
        {
            // Return endpoint depends on whether final grade return is enabled.
            return GradeReturn ? P.FillStartXy : P.FillEndXy;
        }

        /// <summary>
        /// Executes the deadhead operation.
        /// </summary>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <param name="GradeReturn">The grade return used by the operation.</param>
        /// <param name="Obstacles">The obstacles used by the operation.</param>
        /// <returns>The resulting deadhead.</returns>

        static private Double Deadhead
            (
            Pass A,
            Pass B,
            Boolean GradeReturn,
            ObstacleField Obstacles
            )
        {
            // Measure direct deadhead between pass exit and next cut start.
            return DeadheadDistance(A, B, GradeReturn, Obstacles);
        }

        /// <summary>
        /// Executes the deadhead distance operation.
        /// </summary>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <param name="GradeReturn">The grade return used by the operation.</param>
        /// <param name="Obstacles">The obstacles used by the operation.</param>
        /// <returns>The resulting deadhead distance.</returns>

        static private Double DeadheadDistance
            (
            Pass A,
            Pass B,
            Boolean GradeReturn,
            ObstacleField Obstacles
            )
        {
            // Route around obstacles when active.
            (Double X, Double Y) Exit = ExitXy(A, GradeReturn);
            return Obstacles.Active ? Obstacles.PathLength(Exit, B.CutStartXy) : Distance(Exit, B.CutStartXy);
        }

        /// <summary>
        /// Executes the unit operation.
        /// </summary>
        /// <param name="P0">The p0 used by the operation.</param>
        /// <param name="P1">The p1 used by the operation.</param>
        /// <returns>The resulting unit.</returns>

        static private (Double X, Double Y)? Unit
            (
            (Double X, Double Y) P0,
            (Double X, Double Y) P1
            )
        {
            // Return normalized direction, or null for zero-length segment.
            Double Dx = P1.X - P0.X;
            Double Dy = P1.Y - P0.Y;
            Double Norm = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Norm < Constants.NUMERIC_COMPARISON_TOLERANCE)
            {
                return null;
            }

            return (Dx / Norm, Dy / Norm);
        }

        /// <summary>
        /// Executes the cut dir operation.
        /// </summary>
        /// <param name="P">The p used by the operation.</param>
        /// <returns>The resulting cut dir.</returns>

        static private (Double X, Double Y)? CutDir
            (
            Pass P
            )
        {
            // Cut heading is along cut_start to cut_end.
            return Unit(P.CutStartXy, P.CutEndXy);
        }

        /// <summary>
        /// Executes the exit dir operation.
        /// </summary>
        /// <param name="P">The p used by the operation.</param>
        /// <param name="GradeReturn">The grade return used by the operation.</param>
        /// <returns>The resulting exit dir.</returns>

        static private (Double X, Double Y)? ExitDir
            (
            Pass P,
            Boolean GradeReturn
            )
        {
            // Exit direction changes when grade-return reverses the final fill traverse.
            return GradeReturn ? Unit(P.FillEndXy, P.FillStartXy) : Unit(P.FillStartXy, P.FillEndXy);
        }

        /// <summary>
        /// Executes the turn angle operation.
        /// </summary>
        /// <param name="U">The u used by the operation.</param>
        /// <param name="V">The v used by the operation.</param>
        /// <returns>The resulting turn angle.</returns>

        static private Double TurnAngle
            (
            (Double X, Double Y)? U,
            (Double X, Double Y)? V
            )
        {
            // Use acos of clamped dot product.
            if (!U.HasValue || !V.HasValue)
            {
                return 0.0;
            }

            Double Dot = Math.Max(-1.0, Math.Min(1.0, (U.Value.X * V.Value.X) + (U.Value.Y * V.Value.Y)));
            return Math.Acos(Dot);
        }

        /// <summary>
        /// Executes the cost or zero operation.
        /// </summary>
        /// <param name="Model">The model used by the operation.</param>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <returns>The resulting cost or zero.</returns>

        static private Double CostOrZero
            (
            _TransitionModel Model,
            Pass? A,
            Pass? B
            )
        {
            // Missing endpoint means route boundary with no transition.
            if ((A == null) || (B == null))
            {
                return 0.0;
            }

            return Model.Cost(A, B);
        }

        /// <summary>
        /// Executes the route objective operation.
        /// </summary>
        /// <param name="Model">The model used by the operation.</param>
        /// <param name="Passes">The passes used by the operation.</param>
        /// <returns>The resulting route objective.</returns>

        static private Double RouteObjective
            (
            _TransitionModel Model,
            List<Pass> Passes
            )
        {
            // Objective is sum of consecutive transition costs.
            Double Sum = 0.0;
            for (Int32 Index = 0; Index < (Passes.Count - 1); Index += 1)
            {
                Sum += Model.Cost(Passes[Index], Passes[Index + 1]);
            }

            return Sum;
        }

        /// <summary>
        /// Executes the relocate delta operation.
        /// </summary>
        /// <param name="Model">The model used by the operation.</param>
        /// <param name="Seq">The seq used by the operation.</param>
        /// <param name="S">The s used by the operation.</param>
        /// <param name="D">The d used by the operation.</param>
        /// <returns>The resulting relocate delta.</returns>

        static private Double RelocateDelta
            (
            _TransitionModel Model,
            List<Pass> Seq,
            Int32 S,
            Int32 D
            )
        {
            // O(1) delta for removing item at S and inserting at D in reduced sequence.
            Int32 N = Seq.Count;
            Pass X = Seq[S];
            Pass? PrevS = (S > 0) ? Seq[S - 1] : null;
            Pass? NextS = (S < (N - 1)) ? Seq[S + 1] : null;
            Double Removed = CostOrZero(Model, PrevS, X) + CostOrZero(Model, X, NextS);
            Double Bridge = CostOrZero(Model, PrevS, NextS);

            // Resolves the original work pass represented by a routed pass.

            Pass? Orig
                (
                Int32 P
                )
            {
                if ((P < 0) || (P > (N - 2)))
                {
                    return null;
                }

                return (P < S) ? Seq[P] : Seq[P + 1];
            }

            Pass? Left = Orig(D - 1);
            Pass? Right = Orig(D);
            Double RemovedIns = CostOrZero(Model, Left, Right);
            Double AddedIns = CostOrZero(Model, Left, X) + CostOrZero(Model, X, Right);
            return (Bridge - Removed) + (AddedIns - RemovedIns);
        }

        /// <summary>
        /// Executes the two opt operation.
        /// </summary>
        /// <param name="Passes">The passes used by the operation.</param>
        /// <param name="Order">The order used by the operation.</param>
        /// <param name="MaxIter">The max iter used by the operation.</param>
        /// <param name="GradeReturn">The grade return used by the operation.</param>
        /// <returns>The resulting two opt.</returns>

        static private List<Int32> TwoOpt
            (
            List<Pass> Passes,
            List<Int32> Order,
            Int32 MaxIter,
            Boolean GradeReturn
            )
        {
            // Distance-only two-opt refinement (safe for symmetric cost).
            Int32 N = Order.Count;
            Boolean Improved = true;
            Int32 Iter = 0;
            while (Improved && (Iter < MaxIter))
            {
                Improved = false;
                Iter += 1;
                for (Int32 I = 0; I < (N - 1); I += 1)
                {
                    (Double X, Double Y) AEnd = ExitXy(Passes[Order[I]], GradeReturn);
                    (Double X, Double Y) BStart = Passes[Order[I + 1]].CutStartXy;
                    Double DAb = Distance(AEnd, BStart);
                    for (Int32 K = I + 2; K < N; K += 1)
                    {
                        Pass C = Passes[Order[K]];
                        Double NewDA = Distance(AEnd, Passes[Order[K]].CutStartXy);
                        Double OldTail;
                        Double NewTail;
                        if ((K + 1) < N)
                        {
                            OldTail = Distance(ExitXy(C, GradeReturn), Passes[Order[K + 1]].CutStartXy);
                            NewTail = Distance(ExitXy(Passes[Order[I + 1]], GradeReturn), Passes[Order[K + 1]].CutStartXy);
                        }
                        else
                        {
                            OldTail = 0.0;
                            NewTail = 0.0;
                        }

                        Double Delta = (NewDA + NewTail) - (DAb + OldTail);
                        if (Delta < -Constants.GEOMETRY_COMPARISON_TOLERANCE)
                        {
                            Order.Reverse(I + 1, K - I);
                            Improved = true;
                            break;
                        }
                    }

                    if (Improved)
                    {
                        break;
                    }
                }
            }

            return Order;
        }

        /// <summary>
        /// Executes the distance operation.
        /// </summary>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <returns>The resulting distance.</returns>

        static private Double Distance
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            // Shared Euclidean distance helper.
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Transition-cost model between passes.
        /// </summary>
        private sealed class _TransitionModel
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="_TransitionModel"/> class.
            /// </summary>
            /// <param name="GradeReturn">Grade-return flag.</param>
            /// <param name="DirectionAware">Direction-aware flag.</param>
            /// <param name="ReturnV">Return speed m/s.</param>
            /// <param name="TurnV">Turn speed m/s.</param>
            /// <param name="R">Turning radius.</param>
            /// <param name="Obstacles">Obstacle field.</param>
            /// <param name="OrderByDistance">Whether the cost is metres instead of seconds.</param>
            public _TransitionModel
                (
                Boolean GradeReturn,
                Boolean DirectionAware,
                Double ReturnV,
                Double TurnV,
                Double R,
                ObstacleField Obstacles,
                Boolean OrderByDistance
                )
            {
                // Cache motion parameters for repeated transition evaluations.
                this.GradeReturn = GradeReturn;
                this.DirectionAware = DirectionAware;
                this.ReturnV = ReturnV;
                this.TurnV = TurnV;
                this.R = R;
                this.Obstacles = Obstacles;
                this.OrderByDistance = OrderByDistance;
            }

            /// <summary>
            /// Gets grade-return flag.
            /// </summary>
            public Boolean GradeReturn { get; }

            /// <summary>
            /// Gets direction-aware flag.
            /// </summary>
            public Boolean DirectionAware { get; }

            /// <summary>
            /// Gets return speed.
            /// </summary>
            public Double ReturnV { get; }

            /// <summary>
            /// Gets turn speed.
            /// </summary>
            public Double TurnV { get; }

            /// <summary>
            /// Gets turning radius.
            /// </summary>
            public Double R { get; }

            /// <summary>
            /// Gets obstacle field.
            /// </summary>
            public ObstacleField Obstacles { get; }

            /// <summary>Gets whether the cost is metres instead of seconds.</summary>
            public Boolean OrderByDistance { get; }

            /// <summary>
            /// Computes transition cost from one pass to the next.
            /// </summary>
            /// <param name="A">Current pass.</param>
            /// <param name="B">Next pass.</param>
            /// <returns>
            /// Deadhead plus turn arc, in metres, or deadhead time plus turn
            /// time, in seconds, when <see cref="OrderByDistance"/> is false.
            /// </returns>
            public Double Cost
                (
                Pass A,
                Pass B
                )
            {
                (Double X, Double Y) Exit = ExitXy(A, this.GradeReturn);
                (Double X, Double Y) CutStart = B.CutStartXy;
                Double Dist = this.Obstacles.Active ? this.Obstacles.PathLength(Exit, CutStart) : Distance(Exit, CutStart);
                if (!DirectionAware)
                {
                    return Dist;
                }

                // A half-turn is about 24 m of arc. Dividing that arc by turn
                // speed makes it cost the same time as about 80 m of driving.
                Double Angle = 0.0;
                if ((Dist >= Constants.GEOMETRY_COMPARISON_TOLERANCE) && (TurnV > 0.0))
                {
                    (Double X, Double Y)? DeadDir = Unit(Exit, CutStart);
                    Angle = TurnAngle(ExitDir(A, this.GradeReturn), DeadDir)
                        + TurnAngle(DeadDir, CutDir(B));
                }
                if (OrderByDistance)
                {
                    return Dist + (R * Angle);
                }

                Double DriveT = (ReturnV > 0.0) ? (Dist / ReturnV) : Dist;
                if ((Dist < Constants.GEOMETRY_COMPARISON_TOLERANCE) || (TurnV <= 0.0))
                {
                    return DriveT;
                }
                Double TurnT = (R * Angle) / TurnV;
                return DriveT + TurnT;
            }

        }
    }
}
