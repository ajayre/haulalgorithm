using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Optionally removes remote cut/fill fragments by transactionally pooling
    /// one work kind across a bounded group of nearby routes. Each route keeps
    /// its original work volume, so balance and scraper capacity cannot ripple
    /// outside the group; the pooled cell-depth ledger is conserved exactly.
    /// </summary>
    public static class HaulMultiRouteExchangeOptimizer
    {
        private const Double DEPTH_TOLERANCE_M = 1e-9;

        /// <summary>Diagnostics from bounded multi-route exchanges.</summary>
        public sealed class Result
        {
            /// <summary>Initializes exchange diagnostics.</summary>
            public Result
                (
                List<Pass> Passes,
                Int32 AcceptedGroups,
                Int32 RejectedGroups,
                Int32 RoutesChanged,
                Double BeforeM,
                Double AfterM,
                Boolean ConservationValidated
                )
            {
                this.Passes = Passes;
                this.AcceptedGroups = AcceptedGroups;
                this.RejectedGroups = RejectedGroups;
                this.RoutesChanged = RoutesChanged;
                this.BeforeM = BeforeM;
                this.AfterM = AfterM;
                this.ConservationValidated = ConservationValidated;
            }

            /// <summary>Gets the accepted route allocation.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets the number of committed route-group transactions.</summary>
            public Int32 AcceptedGroups { get; }

            /// <summary>Gets candidate groups rejected by objective or validation.</summary>
            public Int32 RejectedGroups { get; }

            /// <summary>Gets the number of distinct routes changed.</summary>
            public Int32 RoutesChanged { get; }

            /// <summary>Gets the initial compactness/connection objective in metres.</summary>
            public Double BeforeM { get; }

            /// <summary>Gets the final compactness/connection objective in metres.</summary>
            public Double AfterM { get; }

            /// <summary>Gets whether the final exact work ledger matched the input.</summary>
            public Boolean ConservationValidated { get; }
        }

        private sealed class Geometry
        {
            public Geometry
                (
                Int32 RouteIndex,
                (Double X, Double Y) Anchor,
                (Double X, Double Y) Outlier,
                Double OutlierDistanceM
                )
            {
                this.RouteIndex = RouteIndex;
                this.Anchor = Anchor;
                this.Outlier = Outlier;
                this.OutlierDistanceM = OutlierDistanceM;
            }

            public Int32 RouteIndex { get; }
            public (Double X, Double Y) Anchor { get; }
            public (Double X, Double Y) Outlier { get; }
            public Double OutlierDistanceM { get; }
        }

        private sealed class PooledCell
        {
            public PooledCell
                (
                (Int32 Bx, Int32 By) Cell,
                Double RemainingDepthM
                )
            {
                this.Cell = Cell;
                this.RemainingDepthM = RemainingDepthM;
            }

            public (Int32 Bx, Int32 By) Cell { get; }
            public Double RemainingDepthM { get; set; }
        }

        /// <summary>
        /// Runs deterministic bounded route-group exchanges. This is deliberately
        /// optional because each round evaluates many local transportation pools.
        /// </summary>
        static public Result Optimize
            (
            IReadOnlyList<Pass> Source,
            Grid Grid,
            Double MaximumDepthM,
            Int32 MaximumRounds = 2,
            Int32 MaximumGroupRoutes = 8,
                Int32 MaximumAcceptedGroups = 100,
                Double MinimumOutlierDistanceM = 0.0,
                Double MinimumImprovementM = 1.0,
                Double BladeWidthM = 0.0,
                Boolean GradeReturn = true,
                ObstacleField? Obstacles = null
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (!Double.IsFinite(MaximumDepthM) || (MaximumDepthM <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumDepthM));
            }
            if (MaximumRounds <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumRounds));
            }
            if (MaximumGroupRoutes < 3)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumGroupRoutes));
            }
            if (MaximumAcceptedGroups <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumAcceptedGroups));
            }
            if (!Double.IsFinite(MinimumOutlierDistanceM)
                || (MinimumOutlierDistanceM < 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(MinimumOutlierDistanceM));
            }
            if (!Double.IsFinite(MinimumImprovementM)
                || (MinimumImprovementM < 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(MinimumImprovementM));
            }
            if (!Double.IsFinite(BladeWidthM) || (BladeWidthM < 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(BladeWidthM));
            }

            List<Pass> Original = Source.ToList();
            List<Pass> Passes = Source.ToList();
            if (Passes.Count < 3)
            {
                Double ObjectiveM = Objective(Passes, Grid);
                return new Result(
                    Passes, 0, 0, 0, ObjectiveM, ObjectiveM, true);
            }

            Double ActiveMinimumOutlierM = MinimumOutlierDistanceM > 0.0
                ? MinimumOutlierDistanceM
                : 4.0 * Grid.GridSizeM;
            Double BeforeM = Objective(Passes, Grid);
            Int32 Accepted = 0;
            Int32 Rejected = 0;
            HashSet<Int32> ChangedRoutes = new HashSet<Int32>();
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;

            for (Int32 Round = 0;
                (Round < MaximumRounds)
                    && (Accepted < MaximumAcceptedGroups);
                Round += 1)
            {
                Int32 RoundAccepted = 0;
                foreach (Boolean FillKind in new[] { true, false })
                {
                    Geometry?[] GeometryByRoute = Passes
                        .Select((Pass, Index) => BuildGeometry(
                            Index,
                            Cells(Pass, FillKind),
                            Grid))
                        .ToArray();
                    HashSet<Int32> UsedRoutes = new HashSet<Int32>();
                    IEnumerable<Geometry> Seeds = GeometryByRoute
                        .Where(Geometry => Geometry != null)
                        .Select(Geometry => Geometry!)
                        .Where(Geometry =>
                            Geometry.OutlierDistanceM
                                >= ActiveMinimumOutlierM)
                        .OrderByDescending(Geometry =>
                            Geometry.OutlierDistanceM)
                        .ThenBy(Geometry => Geometry.RouteIndex);
                    foreach (Geometry Seed in Seeds)
                    {
                        if (Accepted >= MaximumAcceptedGroups) break;
                        if (UsedRoutes.Contains(Seed.RouteIndex)) continue;
                        List<Int32> Group = BuildGroup(
                            Seed,
                            GeometryByRoute,
                            UsedRoutes,
                            MaximumGroupRoutes);
                        if (Group.Count < 3) continue;
                        if (TryReassignGroup(
                            Passes,
                            Group,
                            FillKind,
                            GeometryByRoute,
                            Grid,
                            MaximumDepthM,
                            MinimumImprovementM,
                            BladeWidthM,
                            GradeReturn,
                            ActiveObstacles,
                            out List<Pass>? Candidate))
                        {
                            Passes = Candidate!;
                            Accepted += 1;
                            RoundAccepted += 1;
                            foreach (Int32 RouteIndex in Group)
                            {
                                UsedRoutes.Add(RouteIndex);
                                ChangedRoutes.Add(RouteIndex);
                            }
                        }
                        else
                        {
                            Rejected += 1;
                        }
                    }
                }
                if (RoundAccepted == 0) break;
            }

            Boolean Conserved = SameWork(Original, Passes, FillKind: false)
                && SameWork(Original, Passes, FillKind: true)
                && SameRouteVolumes(Original, Passes);
            if (!Conserved)
            {
                // The pass is transactional: no partially valid exchange escapes.
                Passes = Original;
                Accepted = 0;
                ChangedRoutes.Clear();
                Rejected += 1;
            }
            return new Result(
                Passes,
                Accepted,
                Rejected,
                ChangedRoutes.Count,
                BeforeM,
                Objective(Passes, Grid),
                Conserved);
        }

        private static List<Int32> BuildGroup
            (
            Geometry Seed,
            IReadOnlyList<Geometry?> GeometryByRoute,
            HashSet<Int32> UsedRoutes,
            Int32 MaximumGroupRoutes
            )
        {
            List<Geometry> Available = GeometryByRoute
                .Where(Geometry => Geometry != null)
                .Select(Geometry => Geometry!)
                .Where(Geometry =>
                    (Geometry.RouteIndex == Seed.RouteIndex)
                    || !UsedRoutes.Contains(Geometry.RouteIndex))
                .ToList();
            List<Int32> Group = new List<Int32> { Seed.RouteIndex };

            AddNearest(Seed.Outlier);
            AddNearest(Seed.Anchor);
            foreach (Geometry Candidate in Available
                .Where(Geometry => !Group.Contains(Geometry.RouteIndex))
                .OrderBy(Geometry => Math.Min(
                    Distance(Geometry.Anchor, Seed.Outlier),
                    Distance(Geometry.Anchor, Seed.Anchor)))
                .ThenBy(Geometry => Geometry.RouteIndex))
            {
                if (Group.Count >= MaximumGroupRoutes) break;
                Group.Add(Candidate.RouteIndex);
            }
            return Group;

            void AddNearest((Double X, Double Y) Point)
            {
                Geometry? Nearest = Available
                    .Where(Geometry => !Group.Contains(Geometry.RouteIndex))
                    .OrderBy(Geometry => Distance(Geometry.Anchor, Point))
                    .ThenBy(Geometry => Geometry.RouteIndex)
                    .FirstOrDefault();
                if ((Nearest != null) && (Group.Count < MaximumGroupRoutes))
                {
                    Group.Add(Nearest.RouteIndex);
                }
            }
        }

        private static Boolean TryReassignGroup
            (
            IReadOnlyList<Pass> Source,
            IReadOnlyList<Int32> Group,
            Boolean FillKind,
            IReadOnlyList<Geometry?> GeometryByRoute,
            Grid Grid,
            Double MaximumDepthM,
            Double MinimumImprovementM,
            Double BladeWidthM,
            Boolean GradeReturn,
            ObstacleField Obstacles,
            out List<Pass>? Candidate
            )
        {
            Candidate = null;
            Dictionary<(Int32 Bx, Int32 By), Double> Pool = Group
                .SelectMany(Index => Cells(Source[Index], FillKind))
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(
                    Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            Double[] Quotas = Group
                .Select(Index => Cells(Source[Index], FillKind)
                    .Sum(Entry => Entry.DepthM))
                .ToArray();
            (Double X, Double Y)[] Anchors = Group
                .Select(Index => GeometryByRoute[Index]!.Anchor)
                .ToArray();
            List<Dictionary<(Int32 Bx, Int32 By), Double>> Allocations = Group
                .Select(_ => new Dictionary<(Int32 Bx, Int32 By), Double>())
                .ToList();
            List<PooledCell> RemainingCells = Pool
                .OrderBy(Pair => Pair.Key.By)
                .ThenBy(Pair => Pair.Key.Bx)
                .Select(Pair => new PooledCell(Pair.Key, Pair.Value))
                .ToList();

            while (RemainingCells.Any(Cell =>
                Cell.RemainingDepthM > DEPTH_TOLERANCE_M))
            {
                PooledCell? SelectedCell = null;
                Int32 SelectedRoute = -1;
                Double SelectedRegret = Double.NegativeInfinity;
                foreach (PooledCell Cell in RemainingCells.Where(Cell =>
                    Cell.RemainingDepthM > DEPTH_TOLERANCE_M))
                {
                    List<(Int32 Route, Double Cost)> Feasible = Enumerable
                        .Range(0, Group.Count)
                        .Where(Route => AvailableDepth(
                            Allocations[Route],
                            Cell.Cell,
                            Quotas[Route],
                            MaximumDepthM) > DEPTH_TOLERANCE_M)
                        .Select(Route => (
                            Route,
                            Cost: Distance(
                                Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By),
                                Anchors[Route])))
                        .OrderBy(Item => Item.Cost)
                        .ThenBy(Item => Group[Item.Route])
                        .ToList();
                    if (Feasible.Count == 0) return false;
                    Double Regret = Feasible.Count == 1
                        ? Double.PositiveInfinity
                        : Feasible[1].Cost - Feasible[0].Cost;
                    if ((Regret > SelectedRegret)
                        || ((Regret == SelectedRegret)
                            && ((SelectedCell == null)
                                || (Cell.Cell.By < SelectedCell.Cell.By)
                                || ((Cell.Cell.By == SelectedCell.Cell.By)
                                    && (Cell.Cell.Bx < SelectedCell.Cell.Bx)))))
                    {
                        SelectedCell = Cell;
                        SelectedRoute = Feasible[0].Route;
                        SelectedRegret = Regret;
                    }
                }
                if ((SelectedCell == null) || (SelectedRoute < 0)) return false;
                Double AssignDepthM = Math.Min(
                    SelectedCell.RemainingDepthM,
                    AvailableDepth(
                        Allocations[SelectedRoute],
                        SelectedCell.Cell,
                        Quotas[SelectedRoute],
                        MaximumDepthM));
                if (AssignDepthM <= DEPTH_TOLERANCE_M) return false;
                Allocations[SelectedRoute][SelectedCell.Cell] =
                    Allocations[SelectedRoute].GetValueOrDefault(
                        SelectedCell.Cell) + AssignDepthM;
                Quotas[SelectedRoute] -= AssignDepthM;
                SelectedCell.RemainingDepthM -= AssignDepthM;
                if (Math.Abs(Quotas[SelectedRoute]) <= DEPTH_TOLERANCE_M)
                {
                    Quotas[SelectedRoute] = 0.0;
                }
                if (Math.Abs(SelectedCell.RemainingDepthM)
                    <= DEPTH_TOLERANCE_M)
                {
                    SelectedCell.RemainingDepthM = 0.0;
                }
            }
            if (Quotas.Any(Quota => Math.Abs(Quota) > DEPTH_TOLERANCE_M))
            {
                return false;
            }

            List<Pass> Output = Source.ToList();
            foreach ((Int32 RouteIndex, Int32 LocalIndex) Pair in Group
                .Select((RouteIndex, LocalIndex) => (RouteIndex, LocalIndex)))
            {
                Pass Existing = Source[Pair.RouteIndex];
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> NewCells =
                    Allocations[Pair.LocalIndex]
                        .Where(Item => Item.Value > DEPTH_TOLERANCE_M)
                        .OrderBy(Item => Item.Key.By)
                        .ThenBy(Item => Item.Key.Bx)
                        .Select(Item => (Item.Key, Item.Value))
                        .ToList();
                Pass Rebuilt = Passes.BuildPass(
                    Grid,
                    FillKind ? Existing.CutCells.ToList() : NewCells,
                    FillKind ? NewCells : Existing.FillCells.ToList(),
                    Existing.LoadBcy,
                    Obstacles,
                    Ordered: false);
                Rebuilt.CutHeadingDeg = Existing.CutHeadingDeg;
                Rebuilt.FillHeadingDeg = Existing.FillHeadingDeg;
                Output[Pair.RouteIndex] = Rebuilt;
            }

            if (!SameWork(
                    Group.Select(Index => Source[Index]),
                    Group.Select(Index => Output[Index]),
                    FillKind)
                || !SameRouteVolumes(
                    Group.Select(Index => Source[Index]),
                    Group.Select(Index => Output[Index])))
            {
                return false;
            }
            Double BeforeM = Group.Sum(Index => RouteObjective(
                Source[Index], Grid));
            Double AfterM = Group.Sum(Index => RouteObjective(
                Output[Index], Grid));
            if ((BeforeM - AfterM) + 1e-9 < MinimumImprovementM)
            {
                return false;
            }
            if (BladeWidthM > 0.0)
            {
                // Do not trade an apparently compact allocation for a route that
                // violates the immutable cut/fill/grade-return sequence rules.
                // Existing preflight failures are left untouched for their
                // dedicated repair path rather than being mutated here.
                Int32 BeforeRunCount = 0;
                Int32 AfterRunCount = 0;
                Double BeforeGuidanceDistanceM = 0.0;
                Double AfterGuidanceDistanceM = 0.0;
                foreach (Int32 RouteIndex in Group)
                {
                    GuidancePlanner.CompactCycleResult BeforeGuidance =
                        GuidancePlanner.EvaluateCompactCycle(
                            Source[RouteIndex],
                            Grid,
                            BladeWidthM,
                            GradeReturn);
                    if (!BeforeGuidance.Success) return false;
                    BeforeRunCount += BeforeGuidance.CutRunCount
                        + BeforeGuidance.FillRunCount;
                    BeforeGuidanceDistanceM += BeforeGuidance.DistanceM;
                    GuidancePlanner.CompactCycleResult AfterGuidance =
                        GuidancePlanner.EvaluateCompactCycle(
                            Output[RouteIndex],
                            Grid,
                            BladeWidthM,
                            GradeReturn);
                    if (!AfterGuidance.Success) return false;
                    AfterRunCount += AfterGuidance.CutRunCount
                        + AfterGuidance.FillRunCount;
                    AfterGuidanceDistanceM += AfterGuidance.DistanceM;
                }
                if ((AfterRunCount > BeforeRunCount)
                    || ((BeforeGuidanceDistanceM - AfterGuidanceDistanceM)
                        + 1e-9 < MinimumImprovementM))
                {
                    return false;
                }
            }
            Candidate = Output;
            return true;
        }

        private static Double AvailableDepth
            (
            IReadOnlyDictionary<(Int32 Bx, Int32 By), Double> Allocation,
            (Int32 Bx, Int32 By) Cell,
            Double RemainingQuotaM,
            Double MaximumDepthM
            ) => Math.Min(
                RemainingQuotaM,
                MaximumDepthM - Allocation.GetValueOrDefault(Cell));

        private static Geometry? BuildGeometry
            (
            Int32 RouteIndex,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            if (Cells.Count == 0) return null;
            List<((Double X, Double Y) Point, Double Weight)> Points = Cells
                .Select(Entry => (
                    Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By),
                    Entry.DepthM))
                .ToList();
            (Double X, Double Y) Anchor = Points
                .Select(Candidate => (
                    Candidate.Point,
                    Cost: Points.Sum(Point =>
                        Point.Weight * Distance(
                            Candidate.Point, Point.Point))))
                .OrderBy(Item => Item.Cost)
                .ThenBy(Item => Item.Point.Y)
                .ThenBy(Item => Item.Point.X)
                .First().Point;
            (Double X, Double Y) Outlier = Points
                .OrderByDescending(Point => Distance(Point.Point, Anchor))
                .ThenBy(Point => Point.Point.Y)
                .ThenBy(Point => Point.Point.X)
                .First().Point;
            return new Geometry(
                RouteIndex,
                Anchor,
                Outlier,
                Distance(Anchor, Outlier));
        }

        private static IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            Cells
            (
            Pass Pass,
            Boolean FillKind
            ) => FillKind ? Pass.FillCells : Pass.CutCells;

        private static Double Objective
            (
            IEnumerable<Pass> Passes,
            Grid Grid
            ) => Passes.Sum(Pass => RouteObjective(Pass, Grid));

        private static Double RouteObjective
            (
            Pass Pass,
            Grid Grid
            ) => PatchTreeLength(Pass.CutCells, Grid)
                + PatchTreeLength(Pass.FillCells, Grid)
                + NearestDistance(Pass.CutCells, Pass.FillCells, Grid);

        private static Double PatchTreeLength
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            (Double X, Double Y)[] Points = Cells
                .Select(Entry => Entry.Cell)
                .Distinct()
                .Select(Cell => Grid.CellCenterXy(Cell.Bx, Cell.By))
                .ToArray();
            if (Points.Length <= 1) return 0.0;
            Boolean[] Used = new Boolean[Points.Length];
            Double[] Best = Enumerable.Repeat(
                Double.PositiveInfinity, Points.Length).ToArray();
            Best[0] = 0.0;
            Double TotalM = 0.0;
            for (Int32 Step = 0; Step < Points.Length; Step += 1)
            {
                Int32 Next = -1;
                for (Int32 Index = 0; Index < Points.Length; Index += 1)
                {
                    if (!Used[Index]
                        && ((Next < 0) || (Best[Index] < Best[Next])))
                    {
                        Next = Index;
                    }
                }
                Used[Next] = true;
                TotalM += Best[Next];
                for (Int32 Index = 0; Index < Points.Length; Index += 1)
                {
                    if (!Used[Index])
                    {
                        Best[Index] = Math.Min(
                            Best[Index], Distance(Points[Next], Points[Index]));
                    }
                }
            }
            return TotalM;
        }

        private static Double NearestDistance
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> First,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Second,
            Grid Grid
            )
        {
            Double BestM = Double.PositiveInfinity;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) A in First)
            {
                (Double X, Double Y) FirstPoint = Grid.CellCenterXy(
                    A.Cell.Bx, A.Cell.By);
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) B in Second)
                {
                    BestM = Math.Min(
                        BestM,
                        Distance(
                            FirstPoint,
                            Grid.CellCenterXy(B.Cell.Bx, B.Cell.By)));
                }
            }
            return Double.IsFinite(BestM) ? BestM : 0.0;
        }

        private static Boolean SameWork
            (
            IEnumerable<Pass> First,
            IEnumerable<Pass> Second,
            Boolean FillKind
            )
        {
            Dictionary<(Int32 Bx, Int32 By), Double> FirstDepth = First
                .SelectMany(Pass => Cells(Pass, FillKind))
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(
                    Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            Dictionary<(Int32 Bx, Int32 By), Double> SecondDepth = Second
                .SelectMany(Pass => Cells(Pass, FillKind))
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(
                    Group => Group.Key,
                    Group => Group.Sum(Entry => Entry.DepthM));
            return FirstDepth.Keys.Union(SecondDepth.Keys).All(Cell =>
                Math.Abs(
                    FirstDepth.GetValueOrDefault(Cell)
                    - SecondDepth.GetValueOrDefault(Cell))
                        <= DEPTH_TOLERANCE_M);
        }

        private static Boolean SameRouteVolumes
            (
            IEnumerable<Pass> First,
            IEnumerable<Pass> Second
            )
        {
            Pass[] FirstPasses = First.ToArray();
            Pass[] SecondPasses = Second.ToArray();
            if (FirstPasses.Length != SecondPasses.Length) return false;
            for (Int32 Index = 0; Index < FirstPasses.Length; Index += 1)
            {
                if (Math.Abs(
                        FirstPasses[Index].CutCells.Sum(Entry => Entry.DepthM)
                        - SecondPasses[Index].CutCells.Sum(Entry => Entry.DepthM))
                            > DEPTH_TOLERANCE_M
                    || Math.Abs(
                        FirstPasses[Index].FillCells.Sum(Entry => Entry.DepthM)
                        - SecondPasses[Index].FillCells.Sum(Entry => Entry.DepthM))
                            > DEPTH_TOLERANCE_M
                    || Math.Abs(
                        FirstPasses[Index].LoadBcy
                        - SecondPasses[Index].LoadBcy) > 1e-8)
                {
                    return false;
                }
            }
            return true;
        }

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
