using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace HaulBuilder
{
    /// <summary>
    /// Deterministically defers a bounded number of cycles that fail the exact
    /// compact guidance preflight. Deferred work remains in the canonical ledger.
    /// </summary>
    static public class HaulDifficultCycleFilter
    {
        /// <summary>Deterministic ranking used when the omission bound is active.</summary>
        public enum OmissionPriority
        {
            /// <summary>Prefer failures with the fewest initial work runs.</summary>
            SimplestFirst,

            /// <summary>Prefer failures with the most fragmented initial work.</summary>
            MostFragmentedFirst,
        }

        /// <summary>Result of one bounded compact-guidance preflight.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one filter result.</summary>
            public Result
                (
                List<Pass> Passes,
                Int32 FailureCount,
                Int32 OmittedCycles,
                IReadOnlyList<Int32> OmittedSourceIndices
                )
            {
                this.Passes = Passes;
                this.FailureCount = FailureCount;
                this.OmittedCycles = OmittedCycles;
                this.OmittedSourceIndices = OmittedSourceIndices;
            }

            /// <summary>Gets cycles retained for full guidance construction.</summary>
            public List<Pass> Passes { get; }

            /// <summary>Gets compact-preflight failures before omission.</summary>
            public Int32 FailureCount { get; }

            /// <summary>Gets cycles explicitly deferred.</summary>
            public Int32 OmittedCycles { get; }

            /// <summary>Gets omitted indices in the source pass list.</summary>
            public IReadOnlyList<Int32> OmittedSourceIndices { get; }
        }

        /// <summary>
        /// Removes at most <paramref name="MaximumOmissions"/> failing cycles,
        /// using the configured deterministic priority and preserving source order
        /// for retained work.
        /// </summary>
        static public Result Apply
            (
            IReadOnlyList<Pass> Source,
            Grid Grid,
            Double BladeWidthM,
            Boolean GradeReturn,
            Int32 MaximumOmissions,
            OmissionPriority Priority = OmissionPriority.SimplestFirst
            )
        {
            if (Source == null) throw new ArgumentNullException(nameof(Source));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (!Double.IsFinite(BladeWidthM) || (BladeWidthM <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(BladeWidthM));
            }
            if (MaximumOmissions < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumOmissions));
            }
            if ((MaximumOmissions == 0) || (Source.Count == 0))
            {
                return new Result(Source.ToList(), 0, 0, Array.Empty<Int32>());
            }

            GuidancePlanner.CompactCycleResult[] Preflight =
                new GuidancePlanner.CompactCycleResult[Source.Count];
            Parallel.For(
                0,
                Source.Count,
                Index => Preflight[Index] =
                    GuidancePlanner.EvaluateCompactCycle(
                        Source[Index], Grid, BladeWidthM, GradeReturn));

            Int32[] Omitted = SelectOmittedIndices(
                Preflight, MaximumOmissions, Priority);
            HashSet<Int32> OmittedSet = Omitted.ToHashSet();
            List<Pass> Retained = Enumerable.Range(0, Source.Count)
                .Where(Index => !OmittedSet.Contains(Index))
                .Select(Index => Source[Index])
                .ToList();
            return new Result(
                Retained,
                Preflight.Count(Result => !Result.Success),
                Omitted.Length,
                Omitted);
        }

        /// <summary>Ranks compact failures for deterministic bounded omission.</summary>
        static internal Int32[] SelectOmittedIndices
            (
            IReadOnlyList<GuidancePlanner.CompactCycleResult> Preflight,
            Int32 MaximumOmissions,
            OmissionPriority Priority = OmissionPriority.SimplestFirst
            )
        {
            IEnumerable<Int32> Failures = Enumerable.Range(0, Preflight.Count)
                .Where(Index => !Preflight[Index].Success);
            IOrderedEnumerable<Int32> Ranked =
                Priority == OmissionPriority.MostFragmentedFirst
                    ? Failures.OrderByDescending(Index =>
                        Preflight[Index].CutRunCount
                        + Preflight[Index].FillRunCount)
                        .ThenBy(Index => Preflight[Index].FailureReason)
                        .ThenBy(Index => Index)
                    : Failures.OrderBy(Index => Preflight[Index].FailureReason)
                        .ThenBy(Index =>
                            Preflight[Index].CutRunCount
                            + Preflight[Index].FillRunCount)
                        .ThenBy(Index => Index);
            return Ranked.Take(MaximumOmissions).ToArray();
        }
    }
}
