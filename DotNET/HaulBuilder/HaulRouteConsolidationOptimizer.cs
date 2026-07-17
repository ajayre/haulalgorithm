using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Removes underfilled cycles by transactionally pooling both work ledgers
    /// into nearby routes with spare scraper capacity.  Cut and fill are rebuilt
    /// to identical BCY targets, so balance cannot ripple outside the group.
    /// </summary>
    static public class HaulRouteConsolidationOptimizer
    {
        private const Double TOLERANCE_BCY = 1e-7;
        private const Double TOLERANCE_DEPTH_M = Constants.NUMERIC_COMPARISON_TOLERANCE;

        /// <summary>Consolidation diagnostics.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one consolidation result.</summary>
            public Result
                (
                List<Pass> Passes,
                Int32 RemovedCycles,
                Int32 CandidateAttempts,
                Double BeforeGuidanceM,
                Double AfterGuidanceM
                )
            {
                this.Passes = Passes;
                this.RemovedCycles = RemovedCycles;
                this.CandidateAttempts = CandidateAttempts;
                this.BeforeGuidanceM = BeforeGuidanceM;
                this.AfterGuidanceM = AfterGuidanceM;
            }

            /// <summary>Gets the consolidated passes.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets committed cycle removals.</summary>
            public Int32 RemovedCycles { get; }

            /// <summary>Gets attempted local consolidation groups.</summary>
            public Int32 CandidateAttempts { get; }

            /// <summary>Gets compact-guidance distance before committed moves.</summary>
            public Double BeforeGuidanceM { get; }

            /// <summary>Gets compact-guidance distance after committed moves.</summary>
            public Double AfterGuidanceM { get; }
        }

        /// <summary>Runs bounded deterministic cycle consolidation.</summary>
        static public Result Optimize
            (
            IReadOnlyList<Pass> Source,
            Grid Grid,
            Double MaximumLoadBcy,
            Double MaximumDepthM,
            Double BladeWidthM,
            Boolean GradeReturn,
            Int32 MaximumGroupRoutes = 8,
            Int32 MaximumRemovedCycles = 50,
            ObstacleField? Obstacles = null
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (MaximumGroupRoutes < 3)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumGroupRoutes));
            }
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            List<Pass> Original = Source.ToList();
            List<Pass> Passes = Source.ToList();
            Int32 Removed = 0;
            Int32 Attempts = 0;
            Double BeforeM = 0.0;
            Double AfterM = 0.0;
            while ((Removed < MaximumRemovedCycles) && (Passes.Count >= 3))
            {
                Boolean Accepted = false;
                (Double X, Double Y)[] Centers = Passes
                    .Select(Pass => CombinedCentroid(Pass, Grid)).ToArray();
                foreach (Int32 Seed in Enumerable.Range(0, Passes.Count)
                    .OrderBy(Index => Passes[Index].LoadBcy)
                    .ThenBy(Index => Index))
                {
                    List<Int32> Nearest = Enumerable.Range(0, Passes.Count)
                        .Where(Index => Index != Seed)
                        .OrderBy(Index => Distance(Centers[Seed], Centers[Index]))
                        .ThenBy(Index => Index)
                        .Take(MaximumGroupRoutes - 1)
                        .ToList();
                    for (Int32 Count = 2; Count <= Nearest.Count; Count += 1)
                    {
                        List<Int32> Group = new List<Int32> { Seed };
                        Group.AddRange(Nearest.Take(Count));
                        if (Group.Sum(Index => Passes[Index].LoadBcy)
                            > (Group.Count - 1) * MaximumLoadBcy
                                + TOLERANCE_BCY)
                        {
                            continue;
                        }
                        Attempts += 1;
                        if (!TryConsolidate(
                            Passes,
                            Group,
                            Seed,
                            Grid,
                            MaximumLoadBcy,
                            MaximumDepthM,
                            BladeWidthM,
                            GradeReturn,
                            ActiveObstacles,
                            out List<Pass>? Candidate,
                            out Double CandidateBeforeM,
                            out Double CandidateAfterM))
                        {
                            continue;
                        }
                        Passes = Candidate!;
                        BeforeM += CandidateBeforeM;
                        AfterM += CandidateAfterM;
                        Removed += 1;
                        Accepted = true;
                        break;
                    }
                    if (Accepted) break;
                }
                if (!Accepted) break;
            }
            if (!SameWork(Original, Passes, FillKind: false)
                || !SameWork(Original, Passes, FillKind: true)
                || Math.Abs(
                    Original.Sum(Pass => Pass.LoadBcy)
                    - Passes.Sum(Pass => Pass.LoadBcy)) > TOLERANCE_BCY)
            {
                throw new InvalidOperationException(
                    "Route consolidation failed exact pooled-ledger conservation.");
            }
            return new Result(Passes, Removed, Attempts, BeforeM, AfterM);
        }

        /// <summary>
        /// Executes the try consolidate operation.
        /// </summary>
        /// <param name="Source">The source used by the operation.</param>
        /// <param name="Group">The group used by the operation.</param>
        /// <param name="RemovedIndex">Zero-based item index.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="MaximumLoadBcy">Count limit applied by the operation.</param>
        /// <param name="MaximumDepthM">Count limit applied by the operation.</param>
        /// <param name="BladeWidthM">The blade width m used by the operation.</param>
        /// <param name="GradeReturn">The grade return used by the operation.</param>
        /// <param name="Obstacles">The obstacles used by the operation.</param>
        /// <param name="Candidate">The candidate used by the operation.</param>
        /// <param name="BeforeM">The before m used by the operation.</param>
        /// <param name="AfterM">The after m used by the operation.</param>
        /// <returns>The resulting try consolidate.</returns>

        private static Boolean TryConsolidate
            (
            IReadOnlyList<Pass> Source,
            IReadOnlyList<Int32> Group,
            Int32 RemovedIndex,
            Grid Grid,
            Double MaximumLoadBcy,
            Double MaximumDepthM,
            Double BladeWidthM,
            Boolean GradeReturn,
            ObstacleField Obstacles,
            out List<Pass>? Candidate,
            out Double BeforeM,
            out Double AfterM
            )
        {
            Candidate = null;
            BeforeM = 0.0;
            AfterM = 0.0;
            List<Int32> Retained = Group
                .Where(Index => Index != RemovedIndex).ToList();
            Double[] Targets = Retained
                .Select(Index => Source[Index].LoadBcy).ToArray();
            Double RemainingBcy = Source[RemovedIndex].LoadBcy;
            (Double X, Double Y) RemovedCenter = CombinedCentroid(
                Source[RemovedIndex], Grid);
            foreach (Int32 LocalIndex in Enumerable.Range(0, Retained.Count)
                .OrderBy(Index => Distance(
                    RemovedCenter,
                    CombinedCentroid(Source[Retained[Index]], Grid)))
                .ThenBy(Index => Retained[Index]))
            {
                Double AddedBcy = Math.Min(
                    RemainingBcy, MaximumLoadBcy - Targets[LocalIndex]);
                if (AddedBcy <= 0.0) continue;
                Targets[LocalIndex] += AddedBcy;
                RemainingBcy -= AddedBcy;
                if (RemainingBcy <= TOLERANCE_BCY) break;
            }
            if (RemainingBcy > TOLERANCE_BCY) return false;

            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>? Cuts =
                AllocateKind(Source, Group, Retained, Targets, Grid,
                    MaximumDepthM, FillKind: false);
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>? Fills =
                AllocateKind(Source, Group, Retained, Targets, Grid,
                    MaximumDepthM, FillKind: true);
            if ((Cuts == null) || (Fills == null)) return false;

            Dictionary<Int32, Pass> Replacements = new Dictionary<Int32, Pass>();
            Int32 BeforeRuns = 0;
            Int32 AfterRuns = 0;
            foreach ((Int32 RouteIndex, Int32 LocalIndex) Item in Retained
                .Select((RouteIndex, LocalIndex) => (RouteIndex, LocalIndex)))
            {
                Pass Existing = Source[Item.RouteIndex];
                Pass Rebuilt = Passes.BuildPass(
                    Grid,
                    Cuts[Item.LocalIndex],
                    Fills[Item.LocalIndex],
                    Targets[Item.LocalIndex],
                    Obstacles,
                    Ordered: false);
                Rebuilt.CutHeadingDeg = Existing.CutHeadingDeg;
                Rebuilt.FillHeadingDeg = Existing.FillHeadingDeg;
                GuidancePlanner.CompactCycleResult After =
                    GuidancePlanner.EvaluateCompactCycle(
                        Rebuilt, Grid, BladeWidthM, GradeReturn);
                if (!After.Success) return false;
                AfterM += After.DistanceM;
                AfterRuns += After.CutRunCount + After.FillRunCount;
                Replacements[Item.RouteIndex] = Rebuilt;
            }
            foreach (Int32 RouteIndex in Group)
            {
                GuidancePlanner.CompactCycleResult Before =
                    GuidancePlanner.EvaluateCompactCycle(
                        Source[RouteIndex], Grid, BladeWidthM, GradeReturn);
                if (!Before.Success) return false;
                BeforeM += Before.DistanceM;
                BeforeRuns += Before.CutRunCount + Before.FillRunCount;
            }
            if ((AfterM >= BeforeM - Constants.GEOMETRY_COMPARISON_TOLERANCE) || (AfterRuns > BeforeRuns))
            {
                return false;
            }
            List<Pass> Output = new List<Pass>(Source.Count - 1);
            for (Int32 Index = 0; Index < Source.Count; Index += 1)
            {
                if (Index == RemovedIndex) continue;
                Output.Add(Replacements.GetValueOrDefault(Index, Source[Index]));
            }
            Candidate = Output;
            return true;
        }

        /// <summary>
        /// Executes the allocate kind operation.
        /// </summary>
        /// <param name="Source">The source used by the operation.</param>
        /// <param name="Group">The group used by the operation.</param>
        /// <param name="Retained">The retained used by the operation.</param>
        /// <param name="TargetsBcy">The targets bcy used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="MaximumDepthM">Count limit applied by the operation.</param>
        /// <param name="FillKind">The fill kind used by the operation.</param>
        /// <returns>The resulting allocate kind.</returns>

        private static List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>?
            AllocateKind
            (
            IReadOnlyList<Pass> Source,
            IReadOnlyList<Int32> Group,
            IReadOnlyList<Int32> Retained,
            IReadOnlyList<Double> TargetsBcy,
            Grid Grid,
            Double MaximumDepthM,
            Boolean FillKind
            )
        {
            Double BcyPerDepthM = Grid.AreaM2 * Constants.M3_TO_CY
                * (FillKind ? Grid.Compaction : 1.0);
            Dictionary<(Int32 Bx, Int32 By), Double> Pool = Group
                .SelectMany(Index => Cells(Source[Index], FillKind))
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Grouping => Grouping.Key,
                    Grouping => Grouping.Sum(Entry => Entry.DepthM));
            Double[] Quotas = TargetsBcy
                .Select(Target => Target / BcyPerDepthM).ToArray();
            (Double X, Double Y)[] Anchors = Retained
                .Select(Index => Centroid(Cells(Source[Index], FillKind), Grid))
                .ToArray();
            List<Dictionary<(Int32 Bx, Int32 By), Double>> Allocations =
                Retained.Select(_ =>
                    new Dictionary<(Int32 Bx, Int32 By), Double>()).ToList();
            foreach (KeyValuePair<(Int32 Bx, Int32 By), Double> Pair in Pool
                .OrderBy(Pair => Pair.Key.By).ThenBy(Pair => Pair.Key.Bx))
            {
                Double RemainingDepthM = Pair.Value;
                while (RemainingDepthM > TOLERANCE_DEPTH_M)
                {
                    Int32 Best = Enumerable.Range(0, Retained.Count)
                        .Where(Index => Quotas[Index] > TOLERANCE_DEPTH_M
                            && Allocations[Index].GetValueOrDefault(Pair.Key)
                                < MaximumDepthM - TOLERANCE_DEPTH_M)
                        .OrderBy(Index => Distance(
                            Grid.CellCenterXy(Pair.Key.Bx, Pair.Key.By),
                            Anchors[Index]))
                        .ThenBy(Index => Retained[Index])
                        .FirstOrDefault(-1);
                    if (Best < 0) return null;
                    Double Take = Math.Min(
                        RemainingDepthM,
                        Math.Min(
                            Quotas[Best],
                            MaximumDepthM
                                - Allocations[Best].GetValueOrDefault(Pair.Key)));
                    if (Take <= TOLERANCE_DEPTH_M) return null;
                    Allocations[Best][Pair.Key] =
                        Allocations[Best].GetValueOrDefault(Pair.Key) + Take;
                    Quotas[Best] -= Take;
                    RemainingDepthM -= Take;
                }
            }
            if (Quotas.Any(Quota => Math.Abs(Quota) > TOLERANCE_DEPTH_M))
            {
                return null;
            }
            return Allocations.Select(Allocation => Allocation
                .Where(Pair => Pair.Value > TOLERANCE_DEPTH_M)
                .OrderBy(Pair => Pair.Key.By)
                .ThenBy(Pair => Pair.Key.Bx)
                .Select(Pair => (Pair.Key, Pair.Value))
                .ToList()).ToList();
        }

        /// <summary>
        /// Executes the cells operation.
        /// </summary>
        /// <param name="Pass">The pass used by the operation.</param>
        /// <param name="FillKind">The fill kind used by the operation.</param>
        /// <returns>The resulting cells.</returns>

        private static IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            Cells(Pass Pass, Boolean FillKind) =>
                FillKind ? Pass.FillCells : Pass.CutCells;

        /// <summary>
        /// Executes the combined centroid operation.
        /// </summary>
        /// <param name="Pass">The pass used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting combined centroid.</returns>

        private static (Double X, Double Y) CombinedCentroid(Pass Pass, Grid Grid)
        {
            (Double X, Double Y) Cut = Centroid(Pass.CutCells, Grid);
            (Double X, Double Y) Fill = Centroid(Pass.FillCells, Grid);
            return ((Cut.X + Fill.X) * Constants.HALF_SCALE, (Cut.Y + Fill.Y) * Constants.HALF_SCALE);
        }

        /// <summary>
        /// Executes the centroid operation.
        /// </summary>
        /// <param name="Cells">The cells used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting centroid.</returns>

        private static (Double X, Double Y) Centroid
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
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
        /// Executes the same work operation.
        /// </summary>
        /// <param name="First">The first used by the operation.</param>
        /// <param name="Second">The second used by the operation.</param>
        /// <param name="FillKind">The fill kind used by the operation.</param>
        /// <returns>The resulting same work.</returns>

        private static Boolean SameWork
            (
            IEnumerable<Pass> First,
            IEnumerable<Pass> Second,
            Boolean FillKind
            )
        {
            Dictionary<(Int32 Bx, Int32 By), Double> A = First
                .SelectMany(Pass => Cells(Pass, FillKind))
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            Dictionary<(Int32 Bx, Int32 By), Double> B = Second
                .SelectMany(Pass => Cells(Pass, FillKind))
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            return A.Keys.Union(B.Keys).All(Cell => Math.Abs(
                A.GetValueOrDefault(Cell) - B.GetValueOrDefault(Cell))
                    <= TOLERANCE_DEPTH_M);
        }

        /// <summary>
        /// Executes the distance operation.
        /// </summary>
        /// <param name="First">The first used by the operation.</param>
        /// <param name="Second">The second used by the operation.</param>
        /// <returns>The resulting distance.</returns>

        private static Double Distance
            (
            (Double X, Double Y) First,
            (Double X, Double Y) Second
            )
        {
            Double Dx = Second.X - First.X;
            Double Dy = Second.Y - First.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }
    }
}
