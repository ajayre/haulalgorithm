using System;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Contains automated checks for haul difficult cycle filter.
    /// </summary>

    public sealed class HaulDifficultCycleFilterTests
    {
        /// <summary>
        /// Resolves selection is bounded and deterministic.
        /// </summary>

        [Fact]
        public void SelectionIsBoundedAndDeterministic
            (
            )
        {
            GuidancePlanner.CompactCycleResult[] Results =
            {
                Compact(false, 3, 3, "GRADE_RETURN"),
                Compact(true, 1, 1, String.Empty),
                Compact(false, 5, 4, "CUT_SEQUENCE"),
                Compact(false, 1, 2, "CUT_SEQUENCE"),
                Compact(false, 2, 1, "CUT_SEQUENCE"),
            };

            Int32[] Omitted =
                HaulDifficultCycleFilter.SelectOmittedIndices(Results, 3);

            Assert.Equal(new[] { 3, 4, 2 }, Omitted);
        }

        /// <summary>
        /// Resolves selection never omits successful cycles.
        /// </summary>

        [Fact]
        public void SelectionNeverOmitsSuccessfulCycles
            (
            )
        {
            GuidancePlanner.CompactCycleResult[] Results =
            {
                Compact(true, 20, 20, String.Empty),
                Compact(false, 1, 1, "CUT_SEQUENCE"),
            };

            Assert.Equal(
                new[] { 1 },
                HaulDifficultCycleFilter.SelectOmittedIndices(Results, 20));
        }

        /// <summary>
        /// Verifies fragmented priority selects largest run count first.
        /// </summary>

        [Fact]
        public void FragmentedPrioritySelectsLargestRunCountFirst
            (
            )
        {
            GuidancePlanner.CompactCycleResult[] Results =
            {
                Compact(false, 1, 2, "CUT_SEQUENCE"),
                Compact(false, 8, 7, "CUT_SEQUENCE"),
                Compact(false, 6, 4, "GRADE_RETURN"),
            };

            Assert.Equal(
                new[] { 1, 2 },
                HaulDifficultCycleFilter.SelectOmittedIndices(
                    Results,
                    2,
                    HaulDifficultCycleFilter.OmissionPriority
                        .MostFragmentedFirst));
        }

        /// <summary>
        /// Verifies compact.
        /// </summary>
        /// <param name="Success">The success used by the operation.</param>
        /// <param name="CutRuns">The cut runs used by the operation.</param>
        /// <param name="FillRuns">The fill runs used by the operation.</param>
        /// <param name="Failure">The failure used by the operation.</param>
        /// <returns>The resulting compact.</returns>

        private static GuidancePlanner.CompactCycleResult Compact
            (
            Boolean Success,
            Int32 CutRuns,
            Int32 FillRuns,
            String Failure
            ) => new GuidancePlanner.CompactCycleResult(
                Success, CutRuns, FillRuns, 0.0, Failure);
    }
}
