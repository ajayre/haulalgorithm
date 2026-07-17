using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Reassigns equal-volume fill loads when the nominal compact cut geometry
    /// cannot reach its fill under immutable production guidance constraints.
    /// </summary>
    static public class HaulPairRepair
    {
        private const Double LOAD_TOLERANCE_BCY = 1e-7;

        /// <summary>Diagnostics from one bounded equal-volume pair repair.</summary>
        public sealed class Result
        {
            /// <summary>Initializes repair diagnostics.</summary>
            public Result
                (
                List<Pass> Passes,
                Int32 InitialFailures,
                Int32 FinalFailures,
                Int32 SwapCount,
                Int32 CandidateAttempts
                )
            {
                this.Passes = Passes;
                this.InitialFailures = InitialFailures;
                this.FinalFailures = FinalFailures;
                this.SwapCount = SwapCount;
                this.CandidateAttempts = CandidateAttempts;
            }

            /// <summary>Gets repaired passes.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets compact-cycle failures before repair.</summary>
            public Int32 InitialFailures { get; }

            /// <summary>Gets compact-cycle failures after repair.</summary>
            public Int32 FinalFailures { get; }

            /// <summary>Gets accepted two-cycle fill swaps.</summary>
            public Int32 SwapCount { get; }

            /// <summary>Gets tested swap candidates.</summary>
            public Int32 CandidateAttempts { get; }
        }

        /// <summary>
        /// Swaps complete equal-volume fill allocations between two cycles only
        /// when both resulting cycles retain their compact guidance geometry.
        /// </summary>
        static public Result Repair
            (
            IReadOnlyList<Pass> Source,
            Grid Grid,
            Double BladeM,
            Boolean GradeReturn,
            Int32 CandidateLimit = 64,
            Double MaximumCentroidIncreaseM = 125.0,
            ObstacleField? Obstacles = null
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (CandidateLimit <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(CandidateLimit));
            }
            List<Pass> Passes = Source.ToList();
            GuidancePlanner.CompactCycleResult[] Compact =
                new GuidancePlanner.CompactCycleResult[Passes.Count];
            Parallel.For(
                0,
                Passes.Count,
                Index => Compact[Index] = GuidancePlanner.EvaluateCompactCycle(
                    Passes[Index], Grid, BladeM, GradeReturn));
            Int32 InitialFailures = Compact.Count(Result => !Result.Success);
            Boolean[] Locked = new Boolean[Passes.Count];
            Int32 SwapCount = 0;
            Int32 CandidateAttempts = 0;
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;

            foreach (Int32 FirstIndex in Enumerable.Range(0, Passes.Count)
                .Where(Index => !Compact[Index].Success)
                .OrderByDescending(Index =>
                    Compact[Index].CutRunCount + Compact[Index].FillRunCount))
            {
                if (Locked[FirstIndex] || Compact[FirstIndex].Success) continue;
                Pass First = Passes[FirstIndex];
                Double CurrentCentroidM = PairCentroidDistance(First, Grid);
                IEnumerable<Int32> Candidates = Enumerable.Range(0, Passes.Count)
                    .Where(Index => Index != FirstIndex
                        && !Locked[Index]
                        && Math.Abs(Passes[Index].LoadBcy - First.LoadBcy)
                            <= LOAD_TOLERANCE_BCY)
                    .Select(Index => new
                    {
                        Index,
                        NewDistanceM = CrossPairCentroidDistance(
                            First, Passes[Index], Grid),
                        CurrentDistanceM = CurrentCentroidM
                            + PairCentroidDistance(Passes[Index], Grid),
                    })
                    .Where(Candidate => Candidate.NewDistanceM
                        <= Candidate.CurrentDistanceM + MaximumCentroidIncreaseM)
                    .OrderBy(Candidate =>
                        Candidate.NewDistanceM - Candidate.CurrentDistanceM)
                    .ThenBy(Candidate => Candidate.Index)
                    .Take(CandidateLimit)
                    .Select(Candidate => Candidate.Index);
                foreach (Int32 SecondIndex in Candidates)
                {
                    CandidateAttempts += 1;
                    Pass Second = Passes[SecondIndex];
                    Pass NewFirst = BuildPass(
                        First,
                        Second.FillCells,
                        Second.FillHeadingDeg,
                        Grid,
                        ActiveObstacles);
                    GuidancePlanner.CompactCycleResult FirstResult =
                        GuidancePlanner.EvaluateCompactCycle(
                            NewFirst, Grid, BladeM, GradeReturn);
                    if (!FirstResult.Success) continue;
                    Pass NewSecond = BuildPass(
                        Second,
                        First.FillCells,
                        First.FillHeadingDeg,
                        Grid,
                        ActiveObstacles);
                    GuidancePlanner.CompactCycleResult SecondResult =
                        GuidancePlanner.EvaluateCompactCycle(
                            NewSecond, Grid, BladeM, GradeReturn);
                    if (!SecondResult.Success) continue;

                    Passes[FirstIndex] = NewFirst;
                    Passes[SecondIndex] = NewSecond;
                    Compact[FirstIndex] = FirstResult;
                    Compact[SecondIndex] = SecondResult;
                    Locked[FirstIndex] = true;
                    Locked[SecondIndex] = true;
                    SwapCount += 1;
                    break;
                }
            }
            return new Result(
                Passes,
                InitialFailures,
                Compact.Count(Result => !Result.Success),
                SwapCount,
                CandidateAttempts);
        }

        /// <summary>
        /// Builds pass.
        /// </summary>
        /// <param name="CutSource">The cut source used by the operation.</param>
        /// <param name="FillCells">The fill cells used by the operation.</param>
        /// <param name="FillHeadingDeg">Direction in degrees.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Obstacles">The obstacles used by the operation.</param>
        /// <returns>The resulting pass.</returns>

        private static Pass BuildPass
            (
            Pass CutSource,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
            Double? FillHeadingDeg,
            Grid Grid,
            ObstacleField Obstacles
            )
        {
            Pass Output = HaulBuilder.Passes.BuildPass(
                Grid,
                CutSource.CutCells.ToList(),
                FillCells.ToList(),
                CutSource.LoadBcy,
                Obstacles,
                Ordered: true);
            Output.CutHeadingDeg = CutSource.CutHeadingDeg;
            Output.FillHeadingDeg = FillHeadingDeg;
            return Output;
        }

        /// <summary>
        /// Executes the cross pair centroid distance operation.
        /// </summary>
        /// <param name="First">The first used by the operation.</param>
        /// <param name="Second">The second used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting cross pair centroid distance.</returns>

        private static Double CrossPairCentroidDistance
            (
            Pass First,
            Pass Second,
            Grid Grid
            )
        {
            return CellDistance(First.CutCells, Second.FillCells, Grid)
                + CellDistance(Second.CutCells, First.FillCells, Grid);
        }

        /// <summary>
        /// Executes the pair centroid distance operation.
        /// </summary>
        /// <param name="Pass">The pass used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting pair centroid distance.</returns>

        private static Double PairCentroidDistance(Pass Pass, Grid Grid)
        {
            return CellDistance(Pass.CutCells, Pass.FillCells, Grid);
        }

        /// <summary>
        /// Executes the cell distance operation.
        /// </summary>
        /// <param name="First">The first used by the operation.</param>
        /// <param name="Second">The second used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting cell distance.</returns>

        private static Double CellDistance
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> First,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Second,
            Grid Grid
            )
        {
            (Double X, Double Y) A = Centroid(First, Grid);
            (Double X, Double Y) B = Centroid(Second, Grid);
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
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
                (Double X, Double Y) Xy = Grid.CellCenterXy(
                    Entry.Cell.Bx, Entry.Cell.By);
                SumX += Xy.X * Entry.DepthM;
                SumY += Xy.Y * Entry.DepthM;
                Weight += Entry.DepthM;
            }
            return Weight <= 0.0
                ? (0.0, 0.0)
                : (SumX / Weight, SumY / Weight);
        }
    }
}
