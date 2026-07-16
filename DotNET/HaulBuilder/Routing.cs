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
    public static class Routing
    {
        private const Double DEFAULT_RETURN_SPEED_MPH = 13.0;
        private const Double DEFAULT_TURN_SPEED_MPH = 4.0;
        private const Double DEFAULT_TURNING_CIRCLE_FT = 50.0;

        /// <summary>
        /// Sequences passes using nearest-neighbor then optional 2-opt.
        /// </summary>
        /// <param name="Passes">Pass list.</param>
        /// <param name="TwoOptCap">Maximum pass count for 2-opt.</param>
        /// <param name="GradeReturn">Whether grade-return is enabled.</param>
        /// <param name="DirectionAware">Whether transition cost is heading-aware.</param>
        /// <param name="ReturnSpeedMph">Return speed mph.</param>
        /// <param name="TurnSpeedMph">Turn speed mph.</param>
        /// <param name="TurningCircleFt">Turning circle diameter feet.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <returns>Routed order and deadhead total.</returns>
        static public Route SequencePasses
            (
            List<Pass> Passes,
            Int32 TwoOptCap = 1500,
            Boolean GradeReturn = true,
            Boolean DirectionAware = true,
            Double ReturnSpeedMph = DEFAULT_RETURN_SPEED_MPH,
            Double TurnSpeedMph = DEFAULT_TURN_SPEED_MPH,
            Double TurningCircleFt = DEFAULT_TURNING_CIRCLE_FT,
            ObstacleField? Obstacles = null
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
                ReturnSpeedMph * Constants.MPH_TO_MPS,
                TurnSpeedMph * Constants.MPH_TO_MPS,
                (TurningCircleFt * Constants.FEET_TO_METERS) / 2.0,
                ActiveObstacles
                );

            // Deterministic nearest-neighbor construction from pass 0.
            HashSet<Int32> Unvisited = new HashSet<Int32>();
            for (Int32 Index = 0; Index < N; Index += 1)
            {
                Unvisited.Add(Index);
            }

            List<Int32> Order = new List<Int32> { 0 };
            Unvisited.Remove(0);
            Int32 Current = 0;
            while (Unvisited.Count > 0)
            {
                Double BestCost = Double.PositiveInfinity;
                Int32 Best = -1;
                // Ascending index order matches CPython set(range(n)) iteration so
                // equal transition costs pick the same successor as Python.
                foreach (Int32 Candidate in Unvisited.OrderBy(Index => Index))
                {
                    Double Cost = Model.Cost(Passes[Current], Passes[Candidate]);
                    if (Cost < BestCost)
                    {
                        BestCost = Cost;
                        Best = Candidate;
                    }
                }

                Order.Add(Best);
                Unvisited.Remove(Best);
                Current = Best;
            }

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
        /// <param name="ReturnSpeedMph">Return speed mph.</param>
        /// <param name="TurnSpeedMph">Turn speed mph.</param>
        /// <param name="TurningCircleFt">Turning circle diameter.</param>
        /// <param name="MaxRounds">Max refinement rounds.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <returns>Before objective, after objective, and move count.</returns>
        static public (Double Before, Double After, Int32 Moves) RefineRouteOrder
            (
            Route Route,
            Int32 Window = 50,
            Boolean GradeReturn = true,
            Boolean DirectionAware = true,
            Double ReturnSpeedMph = DEFAULT_RETURN_SPEED_MPH,
            Double TurnSpeedMph = DEFAULT_TURN_SPEED_MPH,
            Double TurningCircleFt = DEFAULT_TURNING_CIRCLE_FT,
            Int32 MaxRounds = 12,
            ObstacleField? Obstacles = null
            )
        {
            // Use relocate (or-opt) moves which preserve segment orientation.
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            _TransitionModel Model = new _TransitionModel
                (
                GradeReturn,
                DirectionAware,
                ReturnSpeedMph * Constants.MPH_TO_MPS,
                TurnSpeedMph * Constants.MPH_TO_MPS,
                (TurningCircleFt * Constants.FEET_TO_METERS) / 2.0,
                ActiveObstacles
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
            for (Int32 Round = 0; Round < MaxRounds; Round += 1)
            {
                Boolean Improved = false;
                Int32 K = 0;
                while (K < Tagged.Count)
                {
                    List<Pass> Cur = new List<Pass>();
                    foreach ((Pass Pass, Int32 Id) Item in Tagged)
                    {
                        Cur.Add(Item.Pass);
                    }

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

        static private (Double X, Double Y) ExitXy
            (
            Pass P,
            Boolean GradeReturn
            )
        {
            // Return endpoint depends on whether final grade return is enabled.
            return GradeReturn ? P.FillStartXy : P.FillEndXy;
        }

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
            if (Norm < 1e-9)
            {
                return null;
            }

            return (Dx / Norm, Dy / Norm);
        }

        static private (Double X, Double Y)? CutDir
            (
            Pass P
            )
        {
            // Cut heading is along cut_start to cut_end.
            return Unit(P.CutStartXy, P.CutEndXy);
        }

        static private (Double X, Double Y)? ExitDir
            (
            Pass P,
            Boolean GradeReturn
            )
        {
            // Exit direction changes when grade-return reverses the final fill traverse.
            return GradeReturn ? Unit(P.FillEndXy, P.FillStartXy) : Unit(P.FillStartXy, P.FillEndXy);
        }

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
                        if (Delta < -1e-6)
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
            public _TransitionModel
                (
                Boolean GradeReturn,
                Boolean DirectionAware,
                Double ReturnV,
                Double TurnV,
                Double R,
                ObstacleField Obstacles
                )
            {
                // Cache motion parameters for repeated transition evaluations.
                this.GradeReturn = GradeReturn;
                this.DirectionAware = DirectionAware;
                this.ReturnV = ReturnV;
                this.TurnV = TurnV;
                this.R = R;
                this.Obstacles = Obstacles;
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

            /// <summary>
            /// Computes transition cost from one pass to the next.
            /// </summary>
            /// <param name="A">Current pass.</param>
            /// <param name="B">Next pass.</param>
            /// <returns>Transition cost.</returns>
            public Double Cost
                (
                Pass A,
                Pass B
                )
            {
                // Blend deadhead drive time plus turn-arc time when enabled.
                (Double X, Double Y) Exit = ExitXy(A, GradeReturn);
                (Double X, Double Y) CutStart = B.CutStartXy;
                Double Dist = Obstacles.Active ? Obstacles.PathLength(Exit, CutStart) : Distance(Exit, CutStart);
                if (!DirectionAware)
                {
                    return Dist;
                }

                Double DriveT = (ReturnV > 0.0) ? (Dist / ReturnV) : Dist;
                if ((Dist < 1e-6) || (TurnV <= 0.0))
                {
                    return DriveT;
                }

                (Double X, Double Y)? DeadDir = Unit(Exit, CutStart);
                Double Angle = TurnAngle(ExitDir(A, GradeReturn), DeadDir) + TurnAngle(DeadDir, CutDir(B));
                Double TurnT = (R * Angle) / TurnV;
                return DriveT + TurnT;
            }
        }
    }
}
