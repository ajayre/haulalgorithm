using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Splits only compact-infeasible allocations into smaller exactly balanced
    /// cycles, retaining every cut and fill depth entry without approximation.
    /// </summary>
    static public class HaulCycleSplitRepair
    {
        private const Double DEPTH_TOLERANCE_M = 1e-10;

        /// <summary>Diagnostics from one bounded split repair.</summary>
        public sealed class Result
        {
            /// <summary>Initializes split diagnostics.</summary>
            public Result
                (
                List<Pass> Passes,
                Int32 InitialFailures,
                Int32 FinalFailures,
                Int32 SplitCycles,
                Int32 AddedCycles
                )
            {
                this.Passes = Passes;
                this.InitialFailures = InitialFailures;
                this.FinalFailures = FinalFailures;
                this.SplitCycles = SplitCycles;
                this.AddedCycles = AddedCycles;
            }

            /// <summary>Gets repaired passes.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets compact failures before repair.</summary>
            public Int32 InitialFailures { get; }

            /// <summary>Gets unsplit compact failures after repair.</summary>
            public Int32 FinalFailures { get; }

            /// <summary>Gets source cycles replaced by smaller cycles.</summary>
            public Int32 SplitCycles { get; }

            /// <summary>Gets the net cycle-count increase.</summary>
            public Int32 AddedCycles { get; }
        }

        /// <summary>Attempts deterministic two-, three-, then four-way splits.</summary>
        static public Result Repair
            (
            IReadOnlyList<Pass> Source,
            Grid Grid,
            Double BladeM,
            Boolean GradeReturn,
            Int32 MaximumParts = 4,
            ObstacleField? Obstacles = null
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (MaximumParts < 2)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumParts));
            }
            GuidancePlanner.CompactCycleResult[] Compact =
                new GuidancePlanner.CompactCycleResult[Source.Count];
            Parallel.For(
                0,
                Source.Count,
                Index => Compact[Index] = GuidancePlanner.EvaluateCompactCycle(
                    Source[Index], Grid, BladeM, GradeReturn));
            Int32 InitialFailures = Compact.Count(Result => !Result.Success);
            List<Pass>?[] Replacements = new List<Pass>?[Source.Count];
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            Parallel.For(0, Source.Count, Index =>
            {
                if (Compact[Index].Success) return;
                Pass Pass = Source[Index];
                for (Int32 PartCount = 2;
                    PartCount <= MaximumParts;
                    PartCount += 1)
                {
                    List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Cuts =
                        SplitEntries(Pass.CutCells, PartCount);
                    List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Fills =
                        SplitEntries(Pass.FillCells, PartCount);
                    if ((Cuts.Count != PartCount) || (Fills.Count != PartCount))
                    {
                        continue;
                    }
                    List<Pass> Parts = new List<Pass>(PartCount);
                    Boolean AllCompact = true;
                    for (Int32 PartIndex = 0;
                        PartIndex < PartCount;
                        PartIndex += 1)
                    {
                        Pass Part = HaulBuilder.Passes.BuildPass(
                            Grid,
                            Cuts[PartIndex],
                            Fills[PartIndex],
                            Pass.LoadBcy / PartCount,
                            ActiveObstacles,
                            Ordered: true);
                        Part.CutHeadingDeg = Pass.CutHeadingDeg;
                        Part.FillHeadingDeg = Pass.FillHeadingDeg;
                        GuidancePlanner.CompactCycleResult Result =
                            GuidancePlanner.EvaluateCompactCycle(
                                Part, Grid, BladeM, GradeReturn);
                        if (!Result.Success)
                        {
                            AllCompact = false;
                            break;
                        }
                        Parts.Add(Part);
                    }
                    if (!AllCompact) continue;
                    Replacements[Index] = Parts;
                    break;
                }
            });

            List<Pass> Output = new List<Pass>();
            Int32 SplitCycles = 0;
            Int32 FinalFailures = 0;
            for (Int32 Index = 0; Index < Source.Count; Index += 1)
            {
                if (Replacements[Index] == null)
                {
                    Output.Add(Source[Index]);
                    if (!Compact[Index].Success) FinalFailures += 1;
                    continue;
                }
                SplitCycles += 1;
                Output.AddRange(Replacements[Index]!);
            }
            return new Result(
                Output,
                InitialFailures,
                FinalFailures,
                SplitCycles,
                Output.Count - Source.Count);
        }

        /// <summary>
        /// Executes the split entries operation.
        /// </summary>
        /// <param name="Entries">The entries used by the operation.</param>
        /// <param name="PartCount">Count limit applied by the operation.</param>
        /// <returns>The resulting split entries.</returns>

        private static List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>
            SplitEntries
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Entries,
            Int32 PartCount
            )
        {
            Double TotalDepthM = Entries.Sum(Entry => Entry.DepthM);
            Double PartDepthM = TotalDepthM / PartCount;
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Output =
                Enumerable.Range(0, PartCount)
                    .Select(_ => new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>())
                    .ToList();
            Int32 PartIndex = 0;
            Double RemainingPartDepthM = PartDepthM;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Entries)
            {
                Double RemainingEntryDepthM = Entry.DepthM;
                while ((RemainingEntryDepthM > DEPTH_TOLERANCE_M)
                    && (PartIndex < PartCount))
                {
                    Double TakeDepthM = PartIndex == (PartCount - 1)
                        ? RemainingEntryDepthM
                        : Math.Min(RemainingEntryDepthM, RemainingPartDepthM);
                    Output[PartIndex].Add((Entry.Cell, TakeDepthM));
                    RemainingEntryDepthM -= TakeDepthM;
                    RemainingPartDepthM -= TakeDepthM;
                    if ((RemainingPartDepthM <= DEPTH_TOLERANCE_M)
                        && (PartIndex < (PartCount - 1)))
                    {
                        PartIndex += 1;
                        RemainingPartDepthM = PartDepthM;
                    }
                }
            }
            return Output.Where(Part => Part.Count > 0).ToList();
        }
    }
}
