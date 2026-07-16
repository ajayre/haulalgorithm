using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Couples cut-to-fill ownership with the following next-cut transition.
    /// It searches the complete equal-load portfolio, swaps whole fill ledgers,
    /// and scores candidates with the production time model plus the actual
    /// routing transition model. Only compact-guidance-feasible, exactly
    /// conserved changes are accepted. Route count and every cut ownership
    /// remain fixed.
    /// </summary>
    public static class HaulCycleChainOptimizer
    {
        private const Double LOAD_TOLERANCE_BCY = 1e-9;
        private const Double DEPTH_TOLERANCE_M = 1e-9;

        /// <summary>Joint field-wide optimization diagnostics.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one optimization result.</summary>
            public Result(
                List<Pass> Passes,
                Int32 AcceptedSwaps,
                Int32 ExactCandidateEvaluations,
                Double BeforeObjectiveSeconds,
                Double AfterObjectiveSeconds,
                Double BeforeProxySeconds,
                Double AfterProxySeconds,
                Double BeforeCompactM,
                Double AfterCompactM,
                Double BeforeTransitionSeconds,
                Double AfterTransitionSeconds)
            {
                this.Passes = Passes;
                this.AcceptedSwaps = AcceptedSwaps;
                this.ExactCandidateEvaluations = ExactCandidateEvaluations;
                this.BeforeObjectiveSeconds = BeforeObjectiveSeconds;
                this.AfterObjectiveSeconds = AfterObjectiveSeconds;
                this.BeforeProxySeconds = BeforeProxySeconds;
                this.AfterProxySeconds = AfterProxySeconds;
                this.BeforeCompactM = BeforeCompactM;
                this.AfterCompactM = AfterCompactM;
                this.BeforeTransitionSeconds = BeforeTransitionSeconds;
                this.AfterTransitionSeconds = AfterTransitionSeconds;
            }

            /// <summary>Gets optimized passes in their input route order.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets accepted whole-fill exchanges.</summary>
            public Int32 AcceptedSwaps { get; }

            /// <summary>Gets compact-guidance candidate evaluations.</summary>
            public Int32 ExactCandidateEvaluations { get; }

            /// <summary>Gets initial production-time plus transition objective.</summary>
            public Double BeforeObjectiveSeconds { get; }

            /// <summary>Gets final production-time plus transition objective.</summary>
            public Double AfterObjectiveSeconds { get; }

            /// <summary>Gets initial production proxy time.</summary>
            public Double BeforeProxySeconds { get; }

            /// <summary>Gets final production proxy time.</summary>
            public Double AfterProxySeconds { get; }

            /// <summary>Gets initial compact-guidance distance.</summary>
            public Double BeforeCompactM { get; }

            /// <summary>Gets final compact-guidance distance.</summary>
            public Double AfterCompactM { get; }

            /// <summary>Gets initial ordered next-cut transition seconds.</summary>
            public Double BeforeTransitionSeconds { get; }

            /// <summary>Gets final ordered next-cut transition seconds.</summary>
            public Double AfterTransitionSeconds { get; }
        }

        /// <summary>Runs deterministic field-wide chain refinement.</summary>
        static public Result Optimize(
            IReadOnlyList<Pass> OrderedSource,
            Grid Grid,
            Double BladeWidthM,
            Boolean GradeReturn,
            Boolean DirectionAware,
            Double ReturnSpeedMph,
            Double TurnSpeedMph,
            Double TurningCircleFt,
            TimeParams TimeParameters,
            Int32 MaximumRounds = 2,
            Int32 ExactCandidatesPerRoute = 4,
            Double MinimumImprovementSeconds = 0.25,
            ObstacleField? Obstacles = null)
        {
            if (OrderedSource == null)
            {
                throw new ArgumentNullException(nameof(OrderedSource));
            }
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (TimeParameters == null)
            {
                throw new ArgumentNullException(nameof(TimeParameters));
            }
            if (MaximumRounds <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumRounds));
            }
            if (ExactCandidatesPerRoute <= 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(ExactCandidatesPerRoute));
            }
            List<Pass> Original = OrderedSource.ToList();
            List<Pass> Passes = OrderedSource.ToList();
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            Routing.TransitionCostEvaluator Evaluator =
                new Routing.TransitionCostEvaluator(
                    GradeReturn,
                    DirectionAware,
                    ReturnSpeedMph,
                    TurnSpeedMph,
                    TurningCircleFt,
                    ActiveObstacles);
            GuidancePlanner.CompactCycleResult[] Compact = Passes
                .Select(Pass => GuidancePlanner.EvaluateCompactCycle(
                    Pass, Grid, BladeWidthM, GradeReturn))
                .ToArray();
            (Double X, Double Y)[] CutCentroids = Passes
                .Select(Pass => Centroid(Pass.CutCells, Grid)).ToArray();
            (Double X, Double Y)[] FillCentroids = Passes
                .Select(Pass => Centroid(Pass.FillCells, Grid)).ToArray();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] FillCells =
                Passes.Select(Pass => Pass.FillCells.ToList()).ToArray();
            Double?[] FillHeadings = Passes
                .Select(Pass => Pass.FillHeadingDeg).ToArray();
            Double BeforeCompactM = Compact
                .Where(Result => Result.Success)
                .Sum(Result => Result.DistanceM);
            Double BeforeTransitionSeconds = TransitionSeconds(
                Passes, Evaluator);
            Double BeforeProxySeconds = Passes.Sum(Pass =>
                TimeModel.EstimatePassTime(Grid, Pass, TimeParameters));
            Double BeforeObjectiveSeconds = BeforeProxySeconds
                + BeforeTransitionSeconds;
            Int32 Accepted = 0;
            Int32 Evaluations = 0;

            for (Int32 Round = 0; Round < MaximumRounds; Round += 1)
            {
                Boolean[] Used = new Boolean[Passes.Count];
                Int32 RoundAccepted = 0;
                IEnumerable<Int32> Visit = Enumerable.Range(0, Passes.Count)
                    .OrderByDescending(Index => LocalObjectiveSeconds(
                        Index,
                        Passes,
                        Evaluator,
                        Grid,
                        TimeParameters))
                    .ThenBy(Index => Index);
                foreach (Int32 First in Visit)
                {
                    // Full guidance has flexible recovery modes not represented
                    // by compact preflight. Freeze those accepted routes rather
                    // than using a weaker feasibility test for reassignment.
                    if (Used[First] || !Compact[First].Success) continue;
                    List<(Int32 Second, Double DeltaM)> Screened =
                        new List<(Int32 Second, Double DeltaM)>();
                    for (Int32 Second = 0;
                        Second < Passes.Count;
                        Second += 1)
                    {
                        if ((Second == First)
                            || Used[Second]
                            || !Compact[Second].Success
                            || (Math.Abs(Passes[First].LoadBcy
                                - Passes[Second].LoadBcy)
                                    > LOAD_TOLERANCE_BCY))
                        {
                            continue;
                        }
                        Double DeltaM = CheapPairDeltaM(
                            First,
                            Second,
                            Passes,
                            CutCentroids,
                            FillCentroids);
                        InsertCandidate(
                            Screened,
                            (Second, DeltaM),
                            ExactCandidatesPerRoute);
                    }
                    Pass? BestFirst = null;
                    Pass? BestSecond = null;
                    Int32 BestSecondIndex = -1;
                    GuidancePlanner.CompactCycleResult? BestFirstCompact = null;
                    GuidancePlanner.CompactCycleResult? BestSecondCompact = null;
                    Double BestDeltaSeconds = -MinimumImprovementSeconds;
                    foreach ((Int32 Second, Double _) in Screened)
                    {
                        Pass CandidateFirst = RebuildWithFill(
                            Passes[First],
                            FillCells[Second],
                            FillHeadings[Second],
                            Grid,
                            ActiveObstacles);
                        Pass CandidateSecond = RebuildWithFill(
                            Passes[Second],
                            FillCells[First],
                            FillHeadings[First],
                            Grid,
                            ActiveObstacles);
                        GuidancePlanner.CompactCycleResult FirstCompact =
                            GuidancePlanner.EvaluateCompactCycle(
                                CandidateFirst,
                                Grid,
                                BladeWidthM,
                                GradeReturn);
                        GuidancePlanner.CompactCycleResult SecondCompact =
                            GuidancePlanner.EvaluateCompactCycle(
                                CandidateSecond,
                                Grid,
                                BladeWidthM,
                                GradeReturn);
                        Evaluations += 2;
                        if (!FirstCompact.Success || !SecondCompact.Success)
                        {
                            continue;
                        }
                        Double BeforeLocalSeconds = LocalPairObjectiveSeconds(
                            First,
                            Second,
                            Passes[First],
                            Passes[Second],
                            Passes,
                            Evaluator,
                            Grid,
                            TimeParameters);
                        Double BeforeLocalProxySeconds =
                            TimeModel.EstimatePassTime(
                                Grid, Passes[First], TimeParameters)
                            + TimeModel.EstimatePassTime(
                                Grid, Passes[Second], TimeParameters);
                        Double AfterLocalProxySeconds =
                            TimeModel.EstimatePassTime(
                                Grid, CandidateFirst, TimeParameters)
                            + TimeModel.EstimatePassTime(
                                Grid, CandidateSecond, TimeParameters);
                        Double BeforeLocalCompactM = Compact[First].DistanceM
                            + Compact[Second].DistanceM;
                        Double AfterLocalCompactM = FirstCompact.DistanceM
                            + SecondCompact.DistanceM;
                        // Production proxy and compact guidance describe
                        // different real costs. Never trade a regression in
                        // either one for an apparent gain in the other.
                        if ((AfterLocalProxySeconds
                                > BeforeLocalProxySeconds + 1e-9)
                            || (AfterLocalCompactM
                                > BeforeLocalCompactM + 1e-9))
                        {
                            continue;
                        }
                        Double AfterLocalSeconds = LocalPairObjectiveSeconds(
                            First,
                            Second,
                            CandidateFirst,
                            CandidateSecond,
                            Passes,
                            Evaluator,
                            Grid,
                            TimeParameters);
                        Double DeltaSeconds = AfterLocalSeconds
                            - BeforeLocalSeconds;
                        if ((DeltaSeconds < BestDeltaSeconds)
                            || ((Math.Abs(DeltaSeconds - BestDeltaSeconds) <= 1e-9)
                                && (Second < BestSecondIndex)))
                        {
                            BestDeltaSeconds = DeltaSeconds;
                            BestSecondIndex = Second;
                            BestFirst = CandidateFirst;
                            BestSecond = CandidateSecond;
                            BestFirstCompact = FirstCompact;
                            BestSecondCompact = SecondCompact;
                        }
                    }
                    if ((BestSecondIndex < 0)
                        || (BestFirst == null)
                        || (BestSecond == null))
                    {
                        continue;
                    }
                    Int32 AcceptedSecond = BestSecondIndex;
                    (FillCells[First], FillCells[AcceptedSecond]) =
                        (FillCells[AcceptedSecond], FillCells[First]);
                    (FillHeadings[First], FillHeadings[AcceptedSecond]) =
                        (FillHeadings[AcceptedSecond], FillHeadings[First]);
                    (FillCentroids[First], FillCentroids[AcceptedSecond]) =
                        (FillCentroids[AcceptedSecond], FillCentroids[First]);
                    Passes[First] = BestFirst;
                    Passes[AcceptedSecond] = BestSecond;
                    Compact[First] = BestFirstCompact!;
                    Compact[AcceptedSecond] = BestSecondCompact!;
                    Used[First] = true;
                    Used[AcceptedSecond] = true;
                    Accepted += 1;
                    RoundAccepted += 1;
                }
                if (RoundAccepted == 0) break;
            }
            if (!SameWork(Original, Passes, FillKind: false)
                || !SameWork(Original, Passes, FillKind: true)
                || Math.Abs(Original.Sum(Pass => Pass.LoadBcy)
                    - Passes.Sum(Pass => Pass.LoadBcy)) > LOAD_TOLERANCE_BCY)
            {
                throw new InvalidOperationException(
                    "Joint chain optimizer failed exact ledger conservation.");
            }
            Double AfterCompactM = Compact
                .Where(Result => Result.Success)
                .Sum(Result => Result.DistanceM);
            Double AfterTransitionSeconds = TransitionSeconds(
                Passes, Evaluator);
            Double AfterProxySeconds = Passes.Sum(Pass =>
                TimeModel.EstimatePassTime(Grid, Pass, TimeParameters));
            Double AfterObjectiveSeconds = AfterProxySeconds
                + AfterTransitionSeconds;
            if (AfterObjectiveSeconds > BeforeObjectiveSeconds + 1e-7)
            {
                throw new InvalidOperationException(
                    "Joint chain optimizer increased its production objective.");
            }
            if ((AfterProxySeconds > BeforeProxySeconds + 1e-7)
                || (AfterCompactM > BeforeCompactM + 1e-7))
            {
                throw new InvalidOperationException(
                    "Joint chain optimizer violated its Pareto acceptance rule.");
            }
            return new Result(
                Passes,
                Accepted,
                Evaluations,
                BeforeObjectiveSeconds,
                AfterObjectiveSeconds,
                BeforeProxySeconds,
                AfterProxySeconds,
                BeforeCompactM,
                AfterCompactM,
                BeforeTransitionSeconds,
                AfterTransitionSeconds);
        }

        private static Double LocalObjectiveSeconds(
            Int32 Index,
            IReadOnlyList<Pass> Passes,
            Routing.TransitionCostEvaluator Evaluator,
            Grid Grid,
            TimeParams TimeParameters)
        {
            Double TransitionSeconds = Index < Passes.Count - 1
                ? Evaluator.CostSeconds(
                    Passes[Index], Passes[Index + 1])
                : 0.0;
            return TimeModel.EstimatePassTime(
                Grid, Passes[Index], TimeParameters) + TransitionSeconds;
        }

        private static Double LocalPairObjectiveSeconds(
            Int32 FirstIndex,
            Int32 SecondIndex,
            Pass First,
            Pass Second,
            IReadOnlyList<Pass> Current,
            Routing.TransitionCostEvaluator Evaluator,
            Grid Grid,
            TimeParams TimeParameters)
        {
            Pass At(Int32 Index)
            {
                if (Index == FirstIndex) return First;
                if (Index == SecondIndex) return Second;
                return Current[Index];
            }
            Double TransitionSeconds = 0.0;
            if (FirstIndex < Current.Count - 1)
            {
                TransitionSeconds += Evaluator.CostSeconds(
                    First, At(FirstIndex + 1));
            }
            if ((SecondIndex < Current.Count - 1)
                && (SecondIndex != FirstIndex))
            {
                TransitionSeconds += Evaluator.CostSeconds(
                    Second, At(SecondIndex + 1));
            }
            return TimeModel.EstimatePassTime(Grid, First, TimeParameters)
                + TimeModel.EstimatePassTime(Grid, Second, TimeParameters)
                + TransitionSeconds;
        }

        private static Double CheapPairDeltaM(
            Int32 First,
            Int32 Second,
            IReadOnlyList<Pass> Passes,
            IReadOnlyList<(Double X, Double Y)> Cuts,
            IReadOnlyList<(Double X, Double Y)> Fills)
        {
            Double Cost(Int32 CutIndex, Int32 FillIndex)
            {
                Double Value = Distance(Cuts[CutIndex], Fills[FillIndex]);
                if (CutIndex < Passes.Count - 1)
                {
                    Value += Distance(
                        Passes[FillIndex].FillStartXy,
                        Passes[CutIndex + 1].CutStartXy);
                }
                return Value;
            }
            return Cost(First, Second)
                + Cost(Second, First)
                - Cost(First, First)
                - Cost(Second, Second);
        }

        private static void InsertCandidate(
            List<(Int32 Second, Double DeltaM)> Candidates,
            (Int32 Second, Double DeltaM) Candidate,
            Int32 MaximumCount)
        {
            Candidates.Add(Candidate);
            Candidates.Sort((First, Second) =>
            {
                Int32 Cost = First.DeltaM.CompareTo(Second.DeltaM);
                return Cost != 0 ? Cost : First.Second.CompareTo(Second.Second);
            });
            if (Candidates.Count > MaximumCount)
            {
                Candidates.RemoveAt(Candidates.Count - 1);
            }
        }

        private static Pass RebuildWithFill(
            Pass CutSource,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
            Double? FillHeadingDeg,
            Grid Grid,
            ObstacleField Obstacles)
        {
            Pass Rebuilt = HaulBuilder.Passes.BuildPass(
                Grid,
                CutSource.CutCells.ToList(),
                FillCells.ToList(),
                CutSource.LoadBcy,
                Obstacles,
                Ordered: true);
            Rebuilt.CutHeadingDeg = CutSource.CutHeadingDeg;
            Rebuilt.FillHeadingDeg = FillHeadingDeg;
            return Rebuilt;
        }

        private static Double TransitionSeconds(
            IReadOnlyList<Pass> Passes,
            Routing.TransitionCostEvaluator Evaluator)
        {
            Double Seconds = 0.0;
            for (Int32 Index = 0; Index < Passes.Count - 1; Index += 1)
            {
                Seconds += Evaluator.CostSeconds(
                    Passes[Index], Passes[Index + 1]);
            }
            return Seconds;
        }

        private static Boolean SameWork(
            IEnumerable<Pass> First,
            IEnumerable<Pass> Second,
            Boolean FillKind)
        {
            Dictionary<(Int32 Bx, Int32 By), Double> A = First
                .SelectMany(Pass => FillKind ? Pass.FillCells : Pass.CutCells)
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            Dictionary<(Int32 Bx, Int32 By), Double> B = Second
                .SelectMany(Pass => FillKind ? Pass.FillCells : Pass.CutCells)
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            return A.Keys.Union(B.Keys).All(Cell => Math.Abs(
                A.GetValueOrDefault(Cell) - B.GetValueOrDefault(Cell))
                    <= DEPTH_TOLERANCE_M);
        }

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
