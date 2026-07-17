using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Computes optimistic field-wide lower bounds without changing allocation.
    /// Bounds intentionally relax unique ownership and subtour constraints; a
    /// small gap therefore proves a plateau, while a large gap only identifies
    /// opportunity for a stronger exact-constrained optimizer.
    /// </summary>
    static public class HaulFieldWideAudit
    {
        private const Double LOAD_GROUP_PRECISION_BCY = Constants.GEOMETRY_COMPARISON_TOLERANCE;

        /// <summary>Field-wide opportunity diagnostics.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one audit result.</summary>
            public Result(
                Double CurrentTransitionSeconds,
                Double TransitionLowerBoundSeconds,
                Double CurrentCutFillCentroidM,
                Double CutFillLowerBoundM,
                Double CurrentJointEquivalentM,
                Double JointLowerBoundEquivalentM)
            {
                this.CurrentTransitionSeconds = CurrentTransitionSeconds;
                this.TransitionLowerBoundSeconds = TransitionLowerBoundSeconds;
                this.CurrentCutFillCentroidM = CurrentCutFillCentroidM;
                this.CutFillLowerBoundM = CutFillLowerBoundM;
                this.CurrentJointEquivalentM = CurrentJointEquivalentM;
                this.JointLowerBoundEquivalentM = JointLowerBoundEquivalentM;
            }

            /// <summary>Gets current ordered next-cut transition proxy seconds.</summary>
            public Double CurrentTransitionSeconds { get; }

            /// <summary>Gets the optimistic open-path transition lower bound.</summary>
            public Double TransitionLowerBoundSeconds { get; }

            /// <summary>Gets current cut/fill centroid assignment distance.</summary>
            public Double CurrentCutFillCentroidM { get; }

            /// <summary>Gets independent non-unique cut/fill assignment lower bound.</summary>
            public Double CutFillLowerBoundM { get; }

            /// <summary>Gets current cut/fill plus next-cut equivalent distance.</summary>
            public Double CurrentJointEquivalentM { get; }

            /// <summary>Gets optimistic non-unique joint equivalent-distance bound.</summary>
            public Double JointLowerBoundEquivalentM { get; }
        }

        /// <summary>Audits one fixed ordered cycle portfolio.</summary>
        static public Result Analyze(
            IReadOnlyList<Pass> OrderedPasses,
            Grid Grid,
            Boolean GradeReturn,
            Boolean DirectionAware,
            Double UnloadedTravelSpeedMph,
            Double TurnSpeedMph,
            Double TurningCircleFt,
            ObstacleField? Obstacles = null)
        {
            if (OrderedPasses == null)
            {
                throw new ArgumentNullException(nameof(OrderedPasses));
            }
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            Int32 Count = OrderedPasses.Count;
            if (Count == 0)
            {
                return new Result(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            }
            if (Count == 1)
            {
                Double OnlyCutFillM = Distance(
                    Centroid(OrderedPasses[0].CutCells, Grid),
                    Centroid(OrderedPasses[0].FillCells, Grid));
                return new Result(
                    0.0,
                    0.0,
                    OnlyCutFillM,
                    OnlyCutFillM,
                    OnlyCutFillM,
                    OnlyCutFillM);
            }
            Routing.TransitionCostEvaluator Evaluator =
                new Routing.TransitionCostEvaluator(
                    GradeReturn,
                    DirectionAware,
                    UnloadedTravelSpeedMph,
                    TurnSpeedMph,
                    TurningCircleFt,
                    Obstacles ?? EMPTY_OBSTACLES);
            Double CurrentTransitionSeconds = 0.0;
            for (Int32 Index = 0; Index < Count - 1; Index += 1)
            {
                CurrentTransitionSeconds += Evaluator.CostSeconds(
                    OrderedPasses[Index], OrderedPasses[Index + 1]);
            }
            Double[] MinimumOutgoing = Enumerable.Repeat(
                Double.PositiveInfinity, Count).ToArray();
            Double[] MinimumIncoming = Enumerable.Repeat(
                Double.PositiveInfinity, Count).ToArray();
            Double[] MinimumFromFill = Enumerable.Repeat(
                Double.PositiveInfinity, Count).ToArray();
            for (Int32 First = 0; First < Count; First += 1)
            {
                for (Int32 Second = 0; Second < Count; Second += 1)
                {
                    if (First == Second) continue;
                    Double Seconds = Evaluator.CostSeconds(
                        OrderedPasses[First], OrderedPasses[Second]);
                    MinimumOutgoing[First] = Math.Min(
                        MinimumOutgoing[First], Seconds);
                    MinimumIncoming[Second] = Math.Min(
                        MinimumIncoming[Second], Seconds);
                    MinimumFromFill[First] = Math.Min(
                        MinimumFromFill[First], Seconds);
                }
            }
            Double OutgoingBound = MinimumOutgoing
                .OrderBy(Value => Value).Take(Count - 1).Sum();
            Double IncomingBound = MinimumIncoming
                .OrderBy(Value => Value).Take(Count - 1).Sum();
            Double TransitionLowerBoundSeconds = Math.Max(
                OutgoingBound, IncomingBound);

            (Double X, Double Y)[] Cuts = OrderedPasses
                .Select(Pass => Centroid(Pass.CutCells, Grid)).ToArray();
            (Double X, Double Y)[] Fills = OrderedPasses
                .Select(Pass => Centroid(Pass.FillCells, Grid)).ToArray();
            Dictionary<Int64, List<Int32>> Groups = Enumerable
                .Range(0, Count)
                .GroupBy(Index => LoadKey(OrderedPasses[Index].LoadBcy))
                .ToDictionary(Group => Group.Key, Group => Group.ToList());
            Double CurrentCutFillM = 0.0;
            Double CutFillLowerBoundM = 0.0;
            Double JointLowerBoundM = 0.0;
            Double LargestTerminalSavingM = 0.0;
            Double ReturnSpeedMps = UnloadedTravelSpeedMph * Constants.MPH_TO_MPS;
            for (Int32 CutIndex = 0; CutIndex < Count; CutIndex += 1)
            {
                CurrentCutFillM += Distance(Cuts[CutIndex], Fills[CutIndex]);
                Double BestCutFillM = Double.PositiveInfinity;
                Double BestJointM = Double.PositiveInfinity;
                foreach (Int32 FillIndex in Groups[
                    LoadKey(OrderedPasses[CutIndex].LoadBcy)])
                {
                    Double CutFillM = Distance(
                        Cuts[CutIndex], Fills[FillIndex]);
                    BestCutFillM = Math.Min(BestCutFillM, CutFillM);
                    BestJointM = Math.Min(
                        BestJointM,
                        CutFillM
                            + (ReturnSpeedMps * MinimumFromFill[FillIndex]));
                }
                CutFillLowerBoundM += BestCutFillM;
                JointLowerBoundM += BestJointM;
                // An open route has one final cycle with no next-cut
                // transition. Relax terminal identity independently too.
                LargestTerminalSavingM = Math.Max(
                    LargestTerminalSavingM,
                    BestJointM - BestCutFillM);
            }
            JointLowerBoundM -= LargestTerminalSavingM;
            Double CurrentJointM = CurrentCutFillM
                + (ReturnSpeedMps * CurrentTransitionSeconds);
            return new Result(
                CurrentTransitionSeconds,
                TransitionLowerBoundSeconds,
                CurrentCutFillM,
                CutFillLowerBoundM,
                CurrentJointM,
                JointLowerBoundM);
        }

        /// <summary>
        /// Reads key.
        /// </summary>
        /// <param name="LoadBcy">The load bcy used by the operation.</param>
        /// <returns>The resulting key.</returns>

        private static Int64 LoadKey(Double LoadBcy) =>
            checked((Int64)Math.Round(
                LoadBcy / LOAD_GROUP_PRECISION_BCY));

        /// <summary>
        /// Executes the centroid operation.
        /// </summary>
        /// <param name="Cells">The cells used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting centroid.</returns>

        private static (Double X, Double Y) Centroid(
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid)
        {
            Double Weight = Cells.Sum(Entry => Entry.DepthM);
            if (Weight <= 0.0) return (0.0, 0.0);
            Double X = 0.0;
            Double Y = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double Px, Double Py) = Grid.CellCenterXy(
                    Entry.Cell.Bx, Entry.Cell.By);
                X += Px * Entry.DepthM;
                Y += Py * Entry.DepthM;
            }
            return (X / Weight, Y / Weight);
        }

        /// <summary>
        /// Executes the distance operation.
        /// </summary>
        /// <param name="First">The first used by the operation.</param>
        /// <param name="Second">The second used by the operation.</param>
        /// <returns>The resulting distance.</returns>

        private static Double Distance(
            (Double X, Double Y) First,
            (Double X, Double Y) Second)
        {
            Double Dx = Second.X - First.X;
            Double Dy = Second.Y - First.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }
    }
}
