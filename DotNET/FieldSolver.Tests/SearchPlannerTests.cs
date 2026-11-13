using System;
using System.Collections.Generic;
using System.Linq;

using Xunit;

using HaulBuilder;

namespace FieldSolver.Tests
{
    /// <summary>Verifies the order and content of the runs the search plans, without running anything.</summary>
    public sealed class SearchPlannerTests
    {
        /// <summary>Verifies the standard search on engine defaults: baseline, three single changes, three pairs, the triple.</summary>
        [Fact]
        public void BuildCombinations_EngineDefaults_OrdersByNumberOfChanges
            (
            )
        {
            FieldSolverSettings Baseline = FieldSolverSettings.FromOptions(new HaulBuilderOptions());

            List<FieldSolverSettings> Combinations = SearchPlanner.BuildCombinations(
                Baseline,
                FieldSolverSearchSpace.CreateStandard());

            FieldSolverSettings[] Expected =
            {
                SolverFixtures.Primary(SolverFixtures.EDGE_DEFAULT, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_NORTH_SOUTH),
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_NORTH_SOUTH),
                SolverFixtures.Primary(SolverFixtures.EDGE_DEFAULT, SolverFixtures.WORK_OFF, SolverFixtures.AXIS_NORTH_SOUTH),
                SolverFixtures.Primary(SolverFixtures.EDGE_DEFAULT, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_EAST_WEST),
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_OFF, SolverFixtures.AXIS_NORTH_SOUTH),
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_EAST_WEST),
                SolverFixtures.Primary(SolverFixtures.EDGE_DEFAULT, SolverFixtures.WORK_OFF, SolverFixtures.AXIS_EAST_WEST),
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_OFF, SolverFixtures.AXIS_EAST_WEST),
            };
            Assert.Equal(Expected, Combinations);
        }

        /// <summary>Verifies a caller value outside the standard lists becomes a third value of its list.</summary>
        [Fact]
        public void BuildCombinations_CallerValueOutsideTheLists_IsKeptAsTheBaseline
            (
            )
        {
            const Int32 CALLER_EDGE = 20;
            const Int32 EXPECTED_COMBINATIONS = 12;
            FieldSolverSettings Baseline = SolverFixtures.Primary(
                CALLER_EDGE,
                SolverFixtures.WORK_DEFAULT,
                SolverFixtures.AXIS_NORTH_SOUTH);

            List<FieldSolverSettings> Combinations = SearchPlanner.BuildCombinations(
                Baseline,
                FieldSolverSearchSpace.CreateStandard());

            // Edge 20, 12 and 40, times two work lengths, times two axes.
            Assert.Equal(EXPECTED_COMBINATIONS, Combinations.Count);
            Assert.Equal(Baseline, Combinations[0]);
            Assert.Equal(EXPECTED_COMBINATIONS, Combinations.Distinct().Count());
            // The number of changed settings never falls from one run to the next.
            for (Int32 Index = 1; Index < Combinations.Count; Index += 1)
            {
                Assert.True(
                    Combinations[Index].CountDifferences(Baseline) >= Combinations[Index - 1].CountDifferences(Baseline),
                    "Run " + (Index + 1) + " has fewer changes than the run before it.");
            }
        }

        /// <summary>Verifies lane axes that differ by a half turn are tried once.</summary>
        [Fact]
        public void BuildCombinations_AxesHalfATurnApart_AreOneAxis
            (
            )
        {
            const Double HALF_TURN_DEG = 180.0;
            FieldSolverSearchSpace Space = new FieldSolverSearchSpace();
            Space.LaneAxisHeadingDegValues.Add(SolverFixtures.AXIS_NORTH_SOUTH);
            Space.LaneAxisHeadingDegValues.Add(HALF_TURN_DEG);
            Space.LaneAxisHeadingDegValues.Add(SolverFixtures.AXIS_EAST_WEST + HALF_TURN_DEG);
            FieldSolverSettings Baseline = FieldSolverSettings.FromOptions(new HaulBuilderOptions());

            List<FieldSolverSettings> Combinations = SearchPlanner.BuildCombinations(Baseline, Space);

            FieldSolverSettings[] Expected =
            {
                Baseline,
                Baseline with { LaneAxisHeadingDeg = SolverFixtures.AXIS_EAST_WEST },
            };
            Assert.Equal(Expected, Combinations);
        }

        /// <summary>Verifies an empty search space plans the baseline alone.</summary>
        [Fact]
        public void BuildCombinations_EmptySpace_IsTheBaselineAlone
            (
            )
        {
            FieldSolverSettings Baseline = FieldSolverSettings.FromOptions(new HaulBuilderOptions());

            List<FieldSolverSettings> Combinations = SearchPlanner.BuildCombinations(Baseline, new FieldSolverSearchSpace());

            Assert.Equal(new FieldSolverSettings[] { Baseline }, Combinations);
        }

        /// <summary>Verifies the thorough search screens seven values in five groups, each one change from the leader.</summary>
        [Fact]
        public void BuildScreeningGroups_ThoroughSpace_ChangesOneValuePerRun
            (
            )
        {
            const Int32 GROUPS = 5;
            const Int32 SCREENING_VALUES = 7;
            FieldSolverSearchSpace Space = FieldSolverSearchSpace.CreateThorough();
            FieldSolverSettings Leader = SolverFixtures.Primary(
                SolverFixtures.EDGE_LOOSE,
                SolverFixtures.WORK_OFF,
                SolverFixtures.AXIS_EAST_WEST);

            List<List<FieldSolverSettings>> Groups = SearchPlanner.BuildScreeningGroups(Leader, Space);

            Assert.Equal(GROUPS, Groups.Count);
            Assert.Equal(SCREENING_VALUES, Groups.Sum(Group => Group.Count));
            Assert.Equal(SCREENING_VALUES, SearchPlanner.CountScreeningValues(Space));
            foreach (FieldSolverSettings Settings in Groups.SelectMany(Group => Group))
            {
                Assert.Equal(1, Settings.CountDifferences(Leader));
            }
        }

        /// <summary>Verifies a screening value the leader already holds is not run again.</summary>
        [Fact]
        public void BuildScreeningGroups_ValueEqualToTheLeader_IsSkipped
            (
            )
        {
            FieldSolverSearchSpace Space = new FieldSolverSearchSpace();
            Space.EnableCycleChainOptimizationValues.Add(true);
            Space.EnableCycleChainOptimizationValues.Add(false);
            FieldSolverSettings Leader = FieldSolverSettings.FromOptions(new HaulBuilderOptions());

            List<List<FieldSolverSettings>> Groups = SearchPlanner.BuildScreeningGroups(Leader, Space);

            // The engine default is "off", so only "on" is a new plan.
            List<FieldSolverSettings> Runs = Groups.SelectMany(Group => Group).ToList();
            Assert.Equal(new FieldSolverSettings[] { Leader with { EnableCycleChainOptimization = true } }, Runs);
        }

        /// <summary>Verifies two screening winners, each one change from the leader, merge into one settings object.</summary>
        [Fact]
        public void MergeChanges_TwoWinners_HoldsBothChanges
            (
            )
        {
            const Double LOW_FLOW = 0.5;
            const Int32 FEWER_SLICES = 8;
            FieldSolverSettings Leader = SolverFixtures.Primary(
                SolverFixtures.EDGE_LOOSE,
                SolverFixtures.WORK_OFF,
                SolverFixtures.AXIS_EAST_WEST);

            FieldSolverSettings Merged = SearchPlanner.MergeChanges(Leader, Leader, Leader with { FlowPenaltyWeight = LOW_FLOW });
            Merged = SearchPlanner.MergeChanges(Merged, Leader, Leader with { MaxLaneSlicesPerCycle = FEWER_SLICES });

            Assert.Equal(Leader with { FlowPenaltyWeight = LOW_FLOW, MaxLaneSlicesPerCycle = FEWER_SLICES }, Merged);
        }
    }
}
