using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Applies deterministic two-exchanges to equal-volume fill allocations so
    /// loaded cut-to-fill centroid travel cannot increase.
    /// </summary>
    static public class HaulPairDistanceRefiner
    {
        private const Double LOAD_TOLERANCE_BCY = 1e-7;
        private const Double IMPROVEMENT_TOLERANCE_M = 0.01;

        /// <summary>Geometry proxy minimized by equal-volume exchanges.</summary>
        public enum PairMetric
        {
            /// <summary>Depth-weighted cut/fill centroid distance.</summary>
            Centroid,

            /// <summary>Shortest distance between cut and fill lane endpoints.</summary>
            Endpoint,

            /// <summary>Endpoint distance plus a bounded centroid regularizer.</summary>
            Hybrid,
        }

        /// <summary>Diagnostics from a bounded pair-distance refinement.</summary>
        public sealed class Result
        {
            /// <summary>Initializes pair refinement diagnostics.</summary>
            public Result
                (
                List<Pass> Passes,
                Int32 Swaps,
                Double BeforeM,
                Double AfterM
                )
            {
                this.Passes = Passes;
                this.Swaps = Swaps;
                this.BeforeM = BeforeM;
                this.AfterM = AfterM;
            }

            /// <summary>Gets refined passes.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets accepted equal-volume fill swaps.</summary>
            public Int32 Swaps { get; }

            /// <summary>Gets loaded centroid distance before refinement.</summary>
            public Double BeforeM { get; }

            /// <summary>Gets loaded centroid distance after refinement.</summary>
            public Double AfterM { get; }
        }

        /// <summary>Runs bounded best-partner two-exchange rounds.</summary>
        static public Result Refine
            (
            IReadOnlyList<Pass> Source,
            Grid Grid,
            Int32 MaximumRounds = 8,
            PairMetric Metric = PairMetric.Centroid,
            Int32 Seed = 0,
            ObstacleField? Obstacles = null
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (MaximumRounds <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumRounds));
            }
            List<Pass> Passes = Source.ToList();
            (Double X, Double Y)[] Cuts = Passes
                .Select(Pass => Centroid(Pass.CutCells, Grid)).ToArray();
            (Double X, Double Y)[] Fills = Passes
                .Select(Pass => Centroid(Pass.FillCells, Grid)).ToArray();
            ((Double X, Double Y) First, (Double X, Double Y) Last)[] CutEnds =
                Passes.Select(Pass => Endpoints(Pass.CutCells, Grid)).ToArray();
            ((Double X, Double Y) First, (Double X, Double Y) Last)[] FillEnds =
                Passes.Select(Pass => Endpoints(Pass.FillCells, Grid)).ToArray();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>[] FillCells =
                Passes.Select(Pass => Pass.FillCells).ToArray();
            Double?[] FillHeadings = Passes
                .Select(Pass => Pass.FillHeadingDeg).ToArray();
            Double BeforeM = Enumerable.Range(0, Passes.Count)
                .Sum(Index => Cost(
                    Cuts[Index], Fills[Index],
                    CutEnds[Index], FillEnds[Index], Metric));
            Int32 Swaps = 0;
            UInt32 ShuffleState = unchecked((UInt32)Seed);
            for (Int32 Round = 0; Round < MaximumRounds; Round += 1)
            {
                Boolean[] Used = new Boolean[Passes.Count];
                Int32 RoundSwaps = 0;
                Int32[] VisitOrder = Enumerable.Range(0, Passes.Count).ToArray();
                if (Seed != 0)
                {
                    for (Int32 Index = VisitOrder.Length - 1; Index > 0; Index -= 1)
                    {
                        ShuffleState = unchecked(
                            (1664525U * ShuffleState) + 1013904223U);
                        Int32 Other = (Int32)(ShuffleState % (UInt32)(Index + 1));
                        (VisitOrder[Index], VisitOrder[Other]) =
                            (VisitOrder[Other], VisitOrder[Index]);
                    }
                }
                foreach (Int32 First in VisitOrder)
                {
                    if (Used[First]) continue;
                    Double CurrentFirstM = Cost(
                        Cuts[First], Fills[First],
                        CutEnds[First], FillEnds[First], Metric);
                    Int32 BestSecond = -1;
                    Double BestDeltaM = -IMPROVEMENT_TOLERANCE_M;
                    for (Int32 Second = 0;
                        Second < Passes.Count;
                        Second += 1)
                    {
                        if ((Second == First)
                            || Used[Second]
                            || Math.Abs(Passes[First].LoadBcy
                                - Passes[Second].LoadBcy) > LOAD_TOLERANCE_BCY)
                        {
                            continue;
                        }
                        Double DeltaM = Cost(
                                Cuts[First], Fills[Second],
                                CutEnds[First], FillEnds[Second], Metric)
                            + Cost(
                                Cuts[Second], Fills[First],
                                CutEnds[Second], FillEnds[First], Metric)
                            - CurrentFirstM
                            - Cost(
                                Cuts[Second], Fills[Second],
                                CutEnds[Second], FillEnds[Second], Metric);
                        if (DeltaM < BestDeltaM)
                        {
                            BestDeltaM = DeltaM;
                            BestSecond = Second;
                        }
                    }
                    if (BestSecond < 0) continue;
                    (Fills[First], Fills[BestSecond]) =
                        (Fills[BestSecond], Fills[First]);
                    (FillEnds[First], FillEnds[BestSecond]) =
                        (FillEnds[BestSecond], FillEnds[First]);
                    (FillCells[First], FillCells[BestSecond]) =
                        (FillCells[BestSecond], FillCells[First]);
                    (FillHeadings[First], FillHeadings[BestSecond]) =
                        (FillHeadings[BestSecond], FillHeadings[First]);
                    Used[First] = true;
                    Used[BestSecond] = true;
                    RoundSwaps += 1;
                    Swaps += 1;
                }
                if (RoundSwaps == 0) break;
            }
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            List<Pass> Output = new List<Pass>(Passes.Count);
            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                Pass Rebuilt = HaulBuilder.Passes.BuildPass(
                    Grid,
                    Passes[Index].CutCells.ToList(),
                    FillCells[Index].ToList(),
                    Passes[Index].LoadBcy,
                    ActiveObstacles,
                    Ordered: true);
                Rebuilt.CutHeadingDeg = Passes[Index].CutHeadingDeg;
                Rebuilt.FillHeadingDeg = FillHeadings[Index];
                Output.Add(Rebuilt);
            }
            Double AfterM = Enumerable.Range(0, Passes.Count)
                .Sum(Index => Cost(
                    Cuts[Index], Fills[Index],
                    CutEnds[Index], FillEnds[Index], Metric));
            return new Result(Output, Swaps, BeforeM, AfterM);
        }

        /// <summary>
        /// Executes the endpoints operation.
        /// </summary>
        /// <param name="Cells">The cells used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting endpoints.</returns>

        private static ((Double X, Double Y) First, (Double X, Double Y) Last)
            Endpoints
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            (Int32 Bx, Int32 By) First = Cells[0].Cell;
            (Int32 Bx, Int32 By) Last = Cells[Cells.Count - 1].Cell;
            return (
                Grid.CellCenterXy(First.Bx, First.By),
                Grid.CellCenterXy(Last.Bx, Last.By));
        }

        /// <summary>
        /// Executes the cost operation.
        /// </summary>
        /// <param name="CutCentroid">The cut centroid used by the operation.</param>
        /// <param name="FillCentroid">The fill centroid used by the operation.</param>
        /// <param name="CutEnds">The cut ends used by the operation.</param>
        /// <param name="FillEnds">The fill ends used by the operation.</param>
        /// <param name="Metric">The metric used by the operation.</param>
        /// <returns>The resulting cost.</returns>

        private static Double Cost
            (
            (Double X, Double Y) CutCentroid,
            (Double X, Double Y) FillCentroid,
            ((Double X, Double Y) First, (Double X, Double Y) Last) CutEnds,
            ((Double X, Double Y) First, (Double X, Double Y) Last) FillEnds,
            PairMetric Metric
            )
        {
            Double CentroidM = Distance(CutCentroid, FillCentroid);
            if (Metric == PairMetric.Centroid) return CentroidM;
            Double EndpointM = Math.Min(
                Math.Min(
                    Distance(CutEnds.First, FillEnds.First),
                    Distance(CutEnds.First, FillEnds.Last)),
                Math.Min(
                    Distance(CutEnds.Last, FillEnds.First),
                    Distance(CutEnds.Last, FillEnds.Last)));
            return Metric == PairMetric.Endpoint
                ? EndpointM
                : EndpointM + (Constants.QUARTER_SCALE * CentroidM);
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
