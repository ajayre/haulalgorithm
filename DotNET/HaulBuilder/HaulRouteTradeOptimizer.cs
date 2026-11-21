using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Trades fragmented cut or fill cells between two overlapping loads so each
    /// load receives one solid run. A trade is kept only when both loads stay
    /// full and the pair is no longer. Passes repeat until one accepts nothing.
    /// </summary>
    static public class HaulRouteTradeOptimizer
    {
        private const Double DEPTH_TOLERANCE_M = Constants.NUMERIC_COMPARISON_TOLERANCE;
        /// <summary>Overlapping loads checked per route. The rest of the field is not paired.</summary>
        private const Int32 MAXIMUM_NEIGHBORS = 4;

        /// <summary>Diagnostics from one settle of pairwise route trades.</summary>
        public sealed class Result
        {
            /// <summary>Initializes trade diagnostics.</summary>
            /// <param name="Passes">Passes after the trades that were kept.</param>
            /// <param name="PassesRun">Passes that actually ran, including the empty last one.</param>
            /// <param name="AcceptedTrades">Trades kept.</param>
            /// <param name="RejectedTrades">Interleaved pairs whose trade was longer or undrivable.</param>
            public Result
                (
                List<Pass> Passes,
                Int32 PassesRun,
                Int32 AcceptedTrades,
                Int32 RejectedTrades
                )
            {
                this.Passes = Passes;
                this.PassesRun = PassesRun;
                this.AcceptedTrades = AcceptedTrades;
                this.RejectedTrades = RejectedTrades;
            }

            /// <summary>Gets the passes after settling.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets how many field passes ran.</summary>
            public Int32 PassesRun { get; }

            /// <summary>Gets trades that shortened the pair.</summary>
            public Int32 AcceptedTrades { get; }

            /// <summary>Gets interleaved pairs that were not kept.</summary>
            public Int32 RejectedTrades { get; }
        }

        /// <summary>One load's cells on the side being traded.</summary>
        private sealed class Side
        {
            /// <summary>Initializes one side snapshot.</summary>
            /// <param name="RouteIndex">Index into the pass list.</param>
            /// <param name="Cells">Cells and depths on this side.</param>
            /// <param name="Grid">Field grid.</param>
            public Side
                (
                Int32 RouteIndex,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
                Grid Grid
                )
            {
                this.RouteIndex = RouteIndex;
                this.Cells = Cells;
                DepthSumM = Cells.Sum(Entry => Entry.DepthM);
                if (Cells.Count == 0)
                {
                    Centroid = (0.0, 0.0);
                    return;
                }
                Double SumX = 0.0;
                Double SumY = 0.0;
                MinX = Double.PositiveInfinity;
                MinY = Double.PositiveInfinity;
                MaxX = Double.NegativeInfinity;
                MaxY = Double.NegativeInfinity;
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
                {
                    (Double X, Double Y) Point = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                    SumX += Point.X;
                    SumY += Point.Y;
                    if (Point.X < MinX) MinX = Point.X;
                    if (Point.Y < MinY) MinY = Point.Y;
                    if (Point.X > MaxX) MaxX = Point.X;
                    if (Point.Y > MaxY) MaxY = Point.Y;
                }
                Centroid = (SumX / Cells.Count, SumY / Cells.Count);
            }

            /// <summary>Gets the pass index.</summary>
            public Int32 RouteIndex { get; }

            /// <summary>Gets the cells on this side.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells { get; }

            /// <summary>Gets the total depth, the volume quota for the trade.</summary>
            public Double DepthSumM { get; }

            /// <summary>Gets the mean cell centre.</summary>
            public (Double X, Double Y) Centroid { get; }

            /// <summary>Gets the western cell centre.</summary>
            public Double MinX { get; }

            /// <summary>Gets the southern cell centre.</summary>
            public Double MinY { get; }

            /// <summary>Gets the eastern cell centre.</summary>
            public Double MaxX { get; }

            /// <summary>Gets the northern cell centre.</summary>
            public Double MaxY { get; }
        }

        /// <summary>One pooled cell projected along the shared run.</summary>
        private readonly struct ProjectedCell
        {
            /// <summary>Initializes one projected cell.</summary>
            /// <param name="Cell">Grid index.</param>
            /// <param name="DepthM">Pooled depth.</param>
            /// <param name="Along">Position along the run.</param>
            public ProjectedCell
                (
                (Int32 Bx, Int32 By) Cell,
                Double DepthM,
                Double Along
                )
            {
                this.Cell = Cell;
                this.DepthM = DepthM;
                this.Along = Along;
            }

            /// <summary>Gets the grid index.</summary>
            public (Int32 Bx, Int32 By) Cell { get; }

            /// <summary>Gets the pooled depth.</summary>
            public Double DepthM { get; }

            /// <summary>Gets the position along the run.</summary>
            public Double Along { get; }
        }

        /// <summary>
        /// Runs pairwise trades until a pass accepts nothing or <paramref name="MaximumPasses"/> is reached.
        /// </summary>
        /// <param name="Source">Loads in their current order.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="MaximumDepthM">Deepest cut or fill one pass may take from one cell.</param>
        /// <param name="GradeReturn">Whether the grade pass repeats the fill.</param>
        /// <param name="MaximumPasses">Cap on field passes. The run stops earlier when a pass accepts nothing.</param>
        /// <param name="Obstacles">Optional exclusion obstacles.</param>
        /// <returns>The settled loads and how many trades were kept.</returns>
        static public Result Optimize
            (
            IReadOnlyList<Pass> Source,
            Grid Grid,
            Double BladeWidthM,
            Double MaximumDepthM,
            Boolean GradeReturn = true,
            Int32 MaximumPasses = 4,
            ObstacleField? Obstacles = null
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (!Double.IsFinite(BladeWidthM) || (BladeWidthM <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(BladeWidthM));
            }
            if (!Double.IsFinite(MaximumDepthM) || (MaximumDepthM <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumDepthM));
            }
            if (MaximumPasses <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumPasses));
            }
            List<Pass> Passes = Source.ToList();
            if (Passes.Count < 2)
            {
                return new Result(Passes, 0, 0, 0);
            }
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            Int32 Accepted = 0;
            Int32 Rejected = 0;
            Int32 PassesRun = 0;
            for (Int32 PassIndex = 0; PassIndex < MaximumPasses; PassIndex += 1)
            {
                PassesRun += 1;
                Int32 PassAccepted = 0;
                foreach (Boolean FillKind in new[] { true, false })
                {
                    GuidancePlanner.CompactCycleResult?[] Cached =
                        new GuidancePlanner.CompactCycleResult?[Passes.Count];
                    HashSet<Int32> Traded = new HashSet<Int32>();
                    List<Side> Sides = new List<Side>(Passes.Count);
                    for (Int32 Index = 0; Index < Passes.Count; Index += 1)
                    {
                        Sides.Add(new Side(Index, CellsOf(Passes[Index], FillKind), Grid));
                    }
                    for (Int32 Index = 0; Index < Sides.Count; Index += 1)
                    {
                        if (Traded.Contains(Index)) continue;
                        if (Sides[Index].Cells.Count == 0) continue;
                        foreach (Int32 Neighbor in Neighbors(Sides, Index))
                        {
                            if (Traded.Contains(Neighbor)) continue;
                            Boolean? Kept = TryTrade(
                                Passes,
                                Sides[Index],
                                Sides[Neighbor],
                                FillKind,
                                Grid,
                                BladeWidthM,
                                MaximumDepthM,
                                GradeReturn,
                                ActiveObstacles,
                                Cached);
                            if (!Kept.HasValue) continue;
                            if (Kept.Value)
                            {
                                Accepted += 1;
                                PassAccepted += 1;
                                Traded.Add(Index);
                                Traded.Add(Neighbor);
                                Cached[Index] = null;
                                Cached[Neighbor] = null;
                                break;
                            }
                            Rejected += 1;
                        }
                    }
                }
                // A pass that keeps nothing has no overlap left to unpick.
                if (PassAccepted == 0) break;
            }
            return new Result(Passes, PassesRun, Accepted, Rejected);
        }

        /// <summary>
        /// Overlapping loads, nearest first. A load whose cells sit outside this
        /// gap is not a candidate.
        /// </summary>
        /// <param name="Sides">Every load's cells on the side being traded.</param>
        /// <param name="Index">Load looking for a partner.</param>
        /// <returns>Up to <see cref="MAXIMUM_NEIGHBORS"/> partner indexes.</returns>
        private static IEnumerable<Int32> Neighbors
            (
            List<Side> Sides,
            Int32 Index
            )
        {
            Side Seed = Sides[Index];
            return Sides
                .Where(Other => (Other.RouteIndex != Index)
                    && (Other.Cells.Count > 0)
                    && Overlaps(Seed, Other))
                .OrderBy(Other => Distance(Seed.Centroid, Other.Centroid))
                .ThenBy(Other => Other.RouteIndex)
                .Take(MAXIMUM_NEIGHBORS)
                .Select(Other => Other.RouteIndex);
        }

        /// <summary>True when the two cell boxes meet or cross.</summary>
        /// <param name="First">First load.</param>
        /// <param name="Second">Second load.</param>
        /// <returns>True when a trade might unpick an interleave.</returns>
        private static Boolean Overlaps
            (
            Side First,
            Side Second
            )
        {
            return (First.MinX <= Second.MaxX)
                && (Second.MinX <= First.MaxX)
                && (First.MinY <= Second.MaxY)
                && (Second.MinY <= First.MaxY);
        }

        /// <summary>
        /// Gives each load one solid block of the pooled cells when the cells
        /// are interleaved. Keeps the trade only when both loads stay drivable
        /// and the pair's compact distance does not grow.
        /// </summary>
        /// <param name="Passes">Loads, updated in place when the trade is kept.</param>
        /// <param name="First">First load's cells.</param>
        /// <param name="Second">Second load's cells.</param>
        /// <param name="FillKind">True to trade fill cells. False trades cut cells.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="MaximumDepthM">Deepest cut or fill one pass may take from one cell.</param>
        /// <param name="GradeReturn">Whether the grade pass repeats the fill.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Cached">Compact results for the current pass. Null means not yet measured.</param>
        /// <returns>True when kept, false when refused, null when the cells are already two solid runs.</returns>
        private static Boolean? TryTrade
            (
            List<Pass> Passes,
            Side First,
            Side Second,
            Boolean FillKind,
            Grid Grid,
            Double BladeWidthM,
            Double MaximumDepthM,
            Boolean GradeReturn,
            ObstacleField Obstacles,
            GuidancePlanner.CompactCycleResult?[] Cached
            )
        {
            List<ProjectedCell> Ordered = Project(First, Second, Grid);
            if (Ordered.Count < 2) return null;
            if (!Interleaved(First, Second, Grid)) return null;
            GuidancePlanner.CompactCycleResult BeforeFirst = Measure(
                Passes, First.RouteIndex, Grid, BladeWidthM, GradeReturn, Cached);
            GuidancePlanner.CompactCycleResult BeforeSecond = Measure(
                Passes, Second.RouteIndex, Grid, BladeWidthM, GradeReturn, Cached);
            // A load that cannot be driven yet is left for its own repair.
            if (!BeforeFirst.Success || !BeforeSecond.Success) return false;
            Double BeforeDistanceM = BeforeFirst.DistanceM + BeforeSecond.DistanceM;
            Int32 BeforeRuns = BeforeFirst.CutRunCount + BeforeFirst.FillRunCount
                + BeforeSecond.CutRunCount + BeforeSecond.FillRunCount;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[]? BestCells = null;
            Double BestDistanceM = BeforeDistanceM;
            Int32 BestRuns = BeforeRuns;
            Boolean Improved = false;
            foreach (Boolean LowBlockToFirst in new[] { true, false })
            {
                Double LowQuotaM = LowBlockToFirst ? First.DepthSumM : Second.DepthSumM;
                if (!TrySplit(
                    Ordered,
                    LowQuotaM,
                    MaximumDepthM,
                    out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Low,
                    out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> High))
                {
                    continue;
                }
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FirstCells =
                    LowBlockToFirst ? Low : High;
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> SecondCells =
                    LowBlockToFirst ? High : Low;
                if (SameCells(First.Cells, FirstCells) && SameCells(Second.Cells, SecondCells))
                {
                    continue;
                }
                Pass RebuiltFirst = Rebuild(
                    Passes[First.RouteIndex], FirstCells, FillKind, Grid, Obstacles);
                Pass RebuiltSecond = Rebuild(
                    Passes[Second.RouteIndex], SecondCells, FillKind, Grid, Obstacles);
                GuidancePlanner.CompactCycleResult AfterFirst = GuidancePlanner.EvaluateCompactCycle(
                    RebuiltFirst, Grid, BladeWidthM, GradeReturn);
                GuidancePlanner.CompactCycleResult AfterSecond = GuidancePlanner.EvaluateCompactCycle(
                    RebuiltSecond, Grid, BladeWidthM, GradeReturn);
                if (!AfterFirst.Success || !AfterSecond.Success) continue;
                Double AfterDistanceM = AfterFirst.DistanceM + AfterSecond.DistanceM;
                Int32 AfterRuns = AfterFirst.CutRunCount + AfterFirst.FillRunCount
                    + AfterSecond.CutRunCount + AfterSecond.FillRunCount;
                Boolean DistanceOk = AfterDistanceM <= BeforeDistanceM + Constants.NUMERIC_COMPARISON_TOLERANCE;
                Boolean RunsOk = AfterRuns <= BeforeRuns;
                Boolean DistanceImproved = AfterDistanceM + Constants.NUMERIC_COMPARISON_TOLERANCE < BeforeDistanceM;
                Boolean RunsImproved = AfterRuns < BeforeRuns;
                // A wash is refused so two passes cannot hand the same cells back and forth.
                if (!DistanceOk || !RunsOk || (!DistanceImproved && !RunsImproved)) continue;
                if ((BestCells != null)
                    && (AfterDistanceM >= BestDistanceM - Constants.NUMERIC_COMPARISON_TOLERANCE)
                    && (AfterRuns >= BestRuns))
                {
                    continue;
                }
                BestCells = new[] { FirstCells, SecondCells };
                BestDistanceM = AfterDistanceM;
                BestRuns = AfterRuns;
                Improved = true;
            }
            if (!Improved || (BestCells == null)) return false;
            Passes[First.RouteIndex] = Rebuild(
                Passes[First.RouteIndex], BestCells[0], FillKind, Grid, Obstacles);
            Passes[Second.RouteIndex] = Rebuild(
                Passes[Second.RouteIndex], BestCells[1], FillKind, Grid, Obstacles);
            return true;
        }

        /// <summary>Projects the pooled cells onto the long direction of the pair.</summary>
        /// <param name="First">First load.</param>
        /// <param name="Second">Second load.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>Cells sorted from the low end of the run. Depths of a shared cell are summed.</returns>
        private static List<ProjectedCell> Project
            (
            Side First,
            Side Second,
            Grid Grid
            )
        {
            Dictionary<(Int32 Bx, Int32 By), Double> Depths =
                new Dictionary<(Int32 Bx, Int32 By), Double>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in First.Cells.Concat(Second.Cells))
            {
                Depths[Entry.Cell] = Depths.GetValueOrDefault(Entry.Cell) + Entry.DepthM;
            }
            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            foreach ((Int32 Bx, Int32 By) Cell in Depths.Keys)
            {
                Points.Add(Grid.CellCenterXy(Cell.Bx, Cell.By));
            }
            (Double X, Double Y) Axis = MajorAxis(Points);
            return Depths
                .Select(Pair =>
                {
                    (Double X, Double Y) Point = Grid.CellCenterXy(Pair.Key.Bx, Pair.Key.By);
                    Double Along = (Point.X * Axis.X) + (Point.Y * Axis.Y);
                    return new ProjectedCell(Pair.Key, Pair.Value, Along);
                })
                .OrderBy(Cell => Cell.Along)
                .ThenBy(Cell => Cell.Cell.By)
                .ThenBy(Cell => Cell.Cell.Bx)
                .ToList();
        }

        /// <summary>
        /// True when the two loads' cells alternate along the run. Two solid
        /// blocks already sitting end to end are left alone.
        /// </summary>
        /// <param name="First">First load.</param>
        /// <param name="Second">Second load.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>
        /// True when exclusive ownership along the run breaks more than once.
        /// A cell owned by both loads is a second layer, not a spatial interleave.
        /// </returns>
        private static Boolean Interleaved
            (
            Side First,
            Side Second,
            Grid Grid
            )
        {
            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in First.Cells.Concat(Second.Cells))
            {
                Points.Add(Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By));
            }
            (Double X, Double Y) Axis = MajorAxis(Points);
            HashSet<(Int32 Bx, Int32 By)> FirstCells = First.Cells
                .Select(Entry => Entry.Cell)
                .ToHashSet();
            HashSet<(Int32 Bx, Int32 By)> SecondCells = Second.Cells
                .Select(Entry => Entry.Cell)
                .ToHashSet();
            List<(Double Along, Int32 Owner)> Tags = new List<(Double Along, Int32 Owner)>();
            // A cell on both loads is the next 61 mm layer. It stays split so one
            // pass is not asked to cut or fill deeper than the machine allows.
            void Tag(Side Side, HashSet<(Int32 Bx, Int32 By)> OtherCells)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Side.Cells)
                {
                    if (OtherCells.Contains(Entry.Cell)) continue;
                    (Double X, Double Y) Point = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                    Tags.Add(((Point.X * Axis.X) + (Point.Y * Axis.Y), Side.RouteIndex));
                }
            }
            Tag(First, SecondCells);
            Tag(Second, FirstCells);
            Tags.Sort((Left, Right) =>
            {
                Int32 Along = Left.Along.CompareTo(Right.Along);
                return Along != 0 ? Along : Left.Owner.CompareTo(Right.Owner);
            });
            if (Tags.Count < 2) return false;
            Int32 Runs = 1;
            for (Int32 Index = 1; Index < Tags.Count; Index += 1)
            {
                if (Tags[Index].Owner != Tags[Index - 1].Owner) Runs += 1;
            }
            // One break is two solid blocks. More than one break is an interleave.
            return Runs > 2;
        }

        /// <summary>Long direction of a set of points, as a unit vector.</summary>
        /// <param name="Points">Cell centres.</param>
        /// <returns>A unit vector. East when the points are a single cell.</returns>
        private static (Double X, Double Y) MajorAxis
            (
            List<(Double X, Double Y)> Points
            )
        {
            if (Points.Count == 0) return (1.0, 0.0);
            Double MeanX = Points.Average(Point => Point.X);
            Double MeanY = Points.Average(Point => Point.Y);
            Double CovarianceXx = 0.0;
            Double CovarianceXy = 0.0;
            Double CovarianceYy = 0.0;
            foreach ((Double X, Double Y) Point in Points)
            {
                Double Dx = Point.X - MeanX;
                Double Dy = Point.Y - MeanY;
                CovarianceXx += Dx * Dx;
                CovarianceXy += Dx * Dy;
                CovarianceYy += Dy * Dy;
            }
            if ((CovarianceXx + CovarianceYy) <= Constants.NUMERIC_COMPARISON_TOLERANCE)
            {
                return (1.0, 0.0);
            }
            Double Angle = Constants.HALF_SCALE * Math.Atan2(
                2.0 * CovarianceXy,
                CovarianceXx - CovarianceYy);
            return (Math.Cos(Angle), Math.Sin(Angle));
        }

        /// <summary>
        /// Cuts the run into a low block of <paramref name="LowQuotaM"/> and the high remainder.
        /// A cell that straddles the quota is split. A cell already split into two
        /// layers stays split, because one pass cannot hold more than
        /// <paramref name="MaximumDepthM"/> of that cell.
        /// </summary>
        /// <param name="Ordered">Cells from the low end.</param>
        /// <param name="LowQuotaM">Depth the low block must hold.</param>
        /// <param name="MaximumDepthM">Deepest cut or fill one pass may take from one cell.</param>
        /// <param name="Low">The low block when the split is legal.</param>
        /// <param name="High">The high block when the split is legal.</param>
        /// <returns>False when the quota cannot be met without a cell deeper than one pass.</returns>
        private static Boolean TrySplit
            (
            List<ProjectedCell> Ordered,
            Double LowQuotaM,
            Double MaximumDepthM,
            out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Low,
            out List<((Int32 Bx, Int32 By) Cell, Double DepthM)> High
            )
        {
            Low = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            High = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double RemainingM = LowQuotaM;
            foreach (ProjectedCell Cell in Ordered)
            {
                // Two layers of one cell sum past the pass limit. Each side can
                // take at most one layer, and the overflow cannot sit on the side
                // that has no quota left.
                Boolean FitsOnOnePass =
                    Cell.DepthM <= MaximumDepthM + DEPTH_TOLERANCE_M;
                Double FloorM = FitsOnOnePass
                    ? 0.0
                    : Cell.DepthM - MaximumDepthM;
                Double CeilingM = Math.Min(Cell.DepthM, MaximumDepthM);
                if (FloorM > CeilingM + DEPTH_TOLERANCE_M) return false;
                Double TakeM = Math.Min(CeilingM, Math.Max(0.0, RemainingM));
                if (TakeM + DEPTH_TOLERANCE_M < FloorM) return false;
                if (TakeM < FloorM) TakeM = FloorM;
                if (TakeM > DEPTH_TOLERANCE_M)
                {
                    Low.Add((Cell.Cell, TakeM));
                }
                Double RestM = Cell.DepthM - TakeM;
                if (RestM > MaximumDepthM + DEPTH_TOLERANCE_M) return false;
                if (RestM > DEPTH_TOLERANCE_M)
                {
                    High.Add((Cell.Cell, RestM));
                }
                RemainingM -= TakeM;
            }
            return RemainingM <= DEPTH_TOLERANCE_M;
        }

        /// <summary>Rebuilds one load with a new cut or fill list. The other side is unchanged.</summary>
        /// <param name="Existing">Load being rebuilt.</param>
        /// <param name="NewCells">Cells for the side being traded.</param>
        /// <param name="FillKind">True when <paramref name="NewCells"/> are fill.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <returns>The rebuilt load, with the previous headings kept.</returns>
        private static Pass Rebuild
            (
            Pass Existing,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> NewCells,
            Boolean FillKind,
            Grid Grid,
            ObstacleField Obstacles
            )
        {
            Pass Rebuilt = Passes.BuildPass(
                Grid,
                FillKind ? Existing.CutCells.ToList() : NewCells,
                FillKind ? NewCells : Existing.FillCells.ToList(),
                Existing.LoadBcy,
                Obstacles,
                Ordered: false);
            Rebuilt.CutHeadingDeg = Existing.CutHeadingDeg;
            Rebuilt.FillHeadingDeg = Existing.FillHeadingDeg;
            return Rebuilt;
        }

        /// <summary>Measures one load, reusing a value already computed in this pass.</summary>
        /// <param name="Passes">Current loads.</param>
        /// <param name="Index">Load to measure.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="GradeReturn">Whether the grade pass repeats the fill.</param>
        /// <param name="Cached">Slot per load. A null slot is measured and stored.</param>
        /// <returns>The compact cycle.</returns>
        private static GuidancePlanner.CompactCycleResult Measure
            (
            List<Pass> Passes,
            Int32 Index,
            Grid Grid,
            Double BladeWidthM,
            Boolean GradeReturn,
            GuidancePlanner.CompactCycleResult?[] Cached
            )
        {
            if (Cached[Index] != null) return Cached[Index]!;
            GuidancePlanner.CompactCycleResult Measured = GuidancePlanner.EvaluateCompactCycle(
                Passes[Index], Grid, BladeWidthM, GradeReturn);
            Cached[Index] = Measured;
            return Measured;
        }

        /// <summary>Cells on the side being traded.</summary>
        /// <param name="Pass">Load.</param>
        /// <param name="FillKind">True for fill cells.</param>
        /// <returns>A copy of that side.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CellsOf
            (
            Pass Pass,
            Boolean FillKind
            )
        {
            return (FillKind ? Pass.FillCells : Pass.CutCells).ToList();
        }

        /// <summary>True when two cell lists hold the same cell depths.</summary>
        /// <param name="Left">First list.</param>
        /// <param name="Right">Second list.</param>
        /// <returns>True when each cell's depth matches.</returns>
        private static Boolean SameCells
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Left,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Right
            )
        {
            Dictionary<(Int32 Bx, Int32 By), Double> Depths =
                new Dictionary<(Int32 Bx, Int32 By), Double>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Left)
            {
                Depths[Entry.Cell] = Depths.GetValueOrDefault(Entry.Cell) + Entry.DepthM;
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Right)
            {
                if (!Depths.TryGetValue(Entry.Cell, out Double DepthM)) return false;
                Depths[Entry.Cell] = DepthM - Entry.DepthM;
            }
            return Depths.Values.All(DepthM => Math.Abs(DepthM) <= DEPTH_TOLERANCE_M);
        }

        /// <summary>Straight distance between two points.</summary>
        /// <param name="A">First point.</param>
        /// <param name="B">Second point.</param>
        /// <returns>Metres.</returns>
        private static Double Distance
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }
    }
}
