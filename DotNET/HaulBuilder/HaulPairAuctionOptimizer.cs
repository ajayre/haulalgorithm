using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Solves a global equal-volume cut/fill assignment with a deterministic
    /// epsilon-auction algorithm over depth-weighted centroids.
    /// </summary>
    public static class HaulPairAuctionOptimizer
    {
        private const Double LOAD_GROUP_PRECISION_BCY = 1e-6;

        /// <summary>Global assignment output and audit metrics.</summary>
        public sealed class Result
        {
            /// <summary>Initializes global pair-assignment diagnostics.</summary>
            public Result
                (
                List<Pass> Passes,
                Double BeforeM,
                Double AfterM,
                Int32 BidCount
                )
            {
                this.Passes = Passes;
                this.BeforeM = BeforeM;
                this.AfterM = AfterM;
                this.BidCount = BidCount;
            }

            /// <summary>Gets globally reassigned passes.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets original centroid assignment distance.</summary>
            public Double BeforeM { get; }

            /// <summary>Gets auction assignment centroid distance.</summary>
            public Double AfterM { get; }

            /// <summary>Gets processed auction bids.</summary>
            public Int32 BidCount { get; }
        }

        /// <summary>Assigns every fill batch exactly once within its load group.</summary>
        static public Result Optimize
            (
            IReadOnlyList<Pass> Source,
            Grid Grid,
            Double EpsilonM = 0.25,
            ObstacleField? Obstacles = null
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (!Double.IsFinite(EpsilonM) || (EpsilonM <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(EpsilonM));
            }
            List<Pass> Passes = Source.ToList();
            (Double X, Double Y)[] Cuts = Passes
                .Select(Pass => Centroid(Pass.CutCells, Grid)).ToArray();
            (Double X, Double Y)[] Fills = Passes
                .Select(Pass => Centroid(Pass.FillCells, Grid)).ToArray();
            Int32[] FillAssignment = Enumerable.Range(0, Passes.Count).ToArray();
            Int32 BidCount = 0;
            IEnumerable<List<Int32>> Groups = Enumerable.Range(0, Passes.Count)
                .GroupBy(Index => Math.Round(
                    Passes[Index].LoadBcy / LOAD_GROUP_PRECISION_BCY))
                .Select(Group => Group.ToList());
            foreach (List<Int32> Group in Groups)
            {
                if (Group.Count <= 1) continue;
                Int32 Count = Group.Count;
                Double[] Prices = new Double[Count];
                Int32[] Owners = Enumerable.Repeat(-1, Count).ToArray();
                Int32[] Assignments = Enumerable.Repeat(-1, Count).ToArray();
                Queue<Int32> Unassigned = new Queue<Int32>(
                    Enumerable.Range(0, Count));
                Int64 GroupBidCount = 0;
                Int64 Guard = Math.Max(
                    100000L,
                    (Int64)Count * Count * 100L);
                while ((Unassigned.Count > 0) && (GroupBidCount < Guard))
                {
                    Int32 Bidder = Unassigned.Dequeue();
                    Double Best = Double.PositiveInfinity;
                    Double Second = Double.PositiveInfinity;
                    Int32 BestItem = -1;
                    for (Int32 Item = 0; Item < Count; Item += 1)
                    {
                        Double Value = Distance(
                            Cuts[Group[Bidder]],
                            Fills[Group[Item]]) + Prices[Item];
                        if (Value < Best)
                        {
                            Second = Best;
                            Best = Value;
                            BestItem = Item;
                        }
                        else if (Value < Second)
                        {
                            Second = Value;
                        }
                    }
                    if (BestItem < 0)
                    {
                        throw new InvalidOperationException(
                            "Global pair auction did not find a fill item.");
                    }
                    if (!Double.IsFinite(Second)) Second = Best;
                    Prices[BestItem] += (Second - Best) + EpsilonM;
                    Int32 PreviousOwner = Owners[BestItem];
                    Owners[BestItem] = Bidder;
                    Assignments[Bidder] = BestItem;
                    if (PreviousOwner >= 0)
                    {
                        Assignments[PreviousOwner] = -1;
                        Unassigned.Enqueue(PreviousOwner);
                    }
                    BidCount += 1;
                    GroupBidCount += 1;
                }
                if (Unassigned.Count > 0)
                {
                    throw new InvalidOperationException(
                        "Global pair auction exceeded its deterministic guard.");
                }
                for (Int32 Bidder = 0; Bidder < Count; Bidder += 1)
                {
                    FillAssignment[Group[Bidder]] = Group[Assignments[Bidder]];
                }
            }

            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            List<Pass> Output = new List<Pass>(Passes.Count);
            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                Pass FillSource = Passes[FillAssignment[Index]];
                Pass Rebuilt = HaulBuilder.Passes.BuildPass(
                    Grid,
                    Passes[Index].CutCells.ToList(),
                    FillSource.FillCells.ToList(),
                    Passes[Index].LoadBcy,
                    ActiveObstacles,
                    Ordered: true);
                Rebuilt.CutHeadingDeg = Passes[Index].CutHeadingDeg;
                Rebuilt.FillHeadingDeg = FillSource.FillHeadingDeg;
                Output.Add(Rebuilt);
            }
            Double BeforeM = Enumerable.Range(0, Passes.Count)
                .Sum(Index => Distance(Cuts[Index], Fills[Index]));
            Double AfterM = Enumerable.Range(0, Passes.Count)
                .Sum(Index => Distance(
                    Cuts[Index],
                    Fills[FillAssignment[Index]]));
            return new Result(Output, BeforeM, AfterM, BidCount);
        }

        private static (Double X, Double Y) Centroid
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            Double SumX = 0.0;
            Double SumY = 0.0;
            Double Weight = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(
                    Entry.Cell.Bx, Entry.Cell.By);
                SumX += Point.X * Entry.DepthM;
                SumY += Point.Y * Entry.DepthM;
                Weight += Entry.DepthM;
            }
            return Weight <= 0.0
                ? (0.0, 0.0)
                : (SumX / Weight, SumY / Weight);
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
