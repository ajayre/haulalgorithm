using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

using Xunit;

using HaulBuilder;

namespace FieldSolver.Tests
{
    /// <summary>
    /// Verifies the search, the scoring, and the stopping rules against a table of
    /// results, so no test in this class runs the engine.
    /// </summary>
    public sealed class FieldSolverEngineTests
    {
        /// <summary>
        /// Tolerance for scores printed to one decimal and recomputed from a
        /// distance that was also stored to one decimal.
        /// </summary>
        private const Double SCORE_TOLERANCE_M = 0.5;

        /// <summary>Verifies the standard search runs the eight combinations in order and picks the measured winner.</summary>
        [Fact]
        public void Solve_StandardSearch_RunsEightCombinationsAndPicksTheBest
            (
            )
        {
            TableRunner Table = SolverFixtures.ShopTable();
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();

            FieldSolverResult Result = new FieldSolverEngine(Table).Solve(Request);

            const Int32 BEST_RUN = 6;
            FieldSolverSettings BestSettings = SolverFixtures.Primary(
                SolverFixtures.EDGE_LOOSE,
                SolverFixtures.WORK_DEFAULT,
                SolverFixtures.AXIS_EAST_WEST);
            FieldSolverSettings LastRun = SolverFixtures.Primary(
                SolverFixtures.EDGE_LOOSE,
                SolverFixtures.WORK_OFF,
                SolverFixtures.AXIS_EAST_WEST);
            Assert.Equal(FieldSolverStatus.Completed, Result.Status);
            Assert.Equal(SolverFixtures.STANDARD_RUNS, Result.PlannedRunCount);
            Assert.Equal(SolverFixtures.STANDARD_RUNS, Result.Candidates.Count);
            Assert.Equal(1, Result.Baseline.RunIndex);
            Assert.Equal(0, Result.Baseline.ChangeCount);
            Assert.Equal(BestSettings, Result.Best.Settings);
            Assert.Equal(BEST_RUN, Result.Best.RunIndex);
            Assert.Equal(LastRun, Table.Calls[SolverFixtures.STANDARD_RUNS - 1]);
            // Every search run had the optional outputs switched off.
            Assert.DoesNotContain(true, Table.SeenOutputSwitches);
        }

        /// <summary>Verifies the scores of the eight measured runs against the hand-checked table of the handoff.</summary>
        [Fact]
        public void Solve_StandardSearch_ScoresMatchTheHandCheckedTable
            (
            )
        {
            const Double BASELINE_UNFINISHED_BCY = 341.916;
            const Double BEST_SAVING_M = 75600.3;
            // Run order: baseline, edge 40, off, axis 90, edge 40 + off, edge 40 + axis 90, off + axis 90, all three.
            Double[] ExpectedScores =
            {
                2100520.4, 2060466.6, 2098900.3, 2073693.6, 2075925.1, 2032178.8, 2081224.1, 2038755.6,
            };

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(SolverFixtures.DefaultRequest());

            for (Int32 Index = 0; Index < ExpectedScores.Length; Index += 1)
            {
                Assert.InRange(
                    Result.Candidates[Index].ScoreM,
                    ExpectedScores[Index] - SCORE_TOLERANCE_M,
                    ExpectedScores[Index] + SCORE_TOLERANCE_M);
            }
            // After Task L the cut left beyond the surplus exceeds the deferred fill.
            Assert.Equal(BASELINE_UNFINISHED_BCY, Result.Baseline.UnfinishedBcy, 9);
            Assert.InRange(Result.SavingM, BEST_SAVING_M - SCORE_TOLERANCE_M, BEST_SAVING_M + SCORE_TOLERANCE_M);
            Assert.Equal(
                Result.Best.DistanceM + Result.Best.UnfinishedChargeM,
                Result.Best.ScoreM,
                9);
        }

        /// <summary>Verifies the returned options carry the best settings and the caller's own outputs, and that the caller's object is untouched.</summary>
        [Fact]
        public void Solve_StandardSearch_ReturnsOptionsReadyToRun
            (
            )
        {
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(Request);

            Assert.NotSame(Request.BaseOptions, Result.BestOptions);
            Assert.Equal(SolverFixtures.EDGE_LOOSE, Result.BestOptions.Optimization.EdgeMaxLaneSlicesPerCycle);
            Assert.Equal(SolverFixtures.WORK_DEFAULT, Result.BestOptions.MinimumWorkLengthFt);
            Assert.Equal(SolverFixtures.AXIS_EAST_WEST, Result.BestOptions.LaneAxisHeadingDeg);
            Assert.True(Result.BestOptions.IncludeGuidancePng);
            Assert.Equal(SolverFixtures.CALLER_IMAGE_COUNT, Result.BestOptions.RouteGuidanceImageCount);
            Assert.Equal("table-only.agd", Result.BestOptions.AgdPath);
            // The caller's object may be bound to a user interface: it must not change.
            Assert.Equal(SolverFixtures.EDGE_DEFAULT, Request.BaseOptions.Optimization.EdgeMaxLaneSlicesPerCycle);
            Assert.Equal(SolverFixtures.WORK_DEFAULT, Request.BaseOptions.MinimumWorkLengthFt);
            Assert.Equal(SolverFixtures.AXIS_NORTH_SOUTH, Request.BaseOptions.LaneAxisHeadingDeg);
            Assert.True(Request.BaseOptions.IncludeGuidancePng);
        }

        /// <summary>Verifies a table runner yields two progress reports per run, ending at 100 percent with the best run.</summary>
        [Fact]
        public void Solve_WithProgress_ReportsEveryRunStartAndFinish
            (
            )
        {
            const Double FULL_PERCENT = 100.0;
            List<FieldSolverProgress> Reports = new List<FieldSolverProgress>();

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(
                SolverFixtures.DefaultRequest(),
                new ListProgress(Reports));

            Assert.Equal(SolverFixtures.STANDARD_RUNS + SolverFixtures.STANDARD_RUNS, Reports.Count);
            Assert.Null(Reports[0].BestSoFar);
            Assert.Null(Reports[0].Completed);
            Assert.Equal(1, Reports[0].RunIndex);
            Assert.Same(Result.Baseline, Reports[1].Completed);
            Assert.Equal(FULL_PERCENT, Reports[Reports.Count - 1].OverallPercentComplete);
            Assert.Same(Result.Best, Reports[Reports.Count - 1].BestSoFar);
            // Overall progress never runs backwards.
            for (Int32 Index = 1; Index < Reports.Count; Index += 1)
            {
                Assert.True(Reports[Index].OverallPercentComplete >= Reports[Index - 1].OverallPercentComplete);
            }
        }

        /// <summary>Verifies a factor of zero ranks the plans on driven distance alone.</summary>
        [Fact]
        public void Solve_FactorZero_ScoreIsTheDistance
            (
            )
        {
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.UnfinishedSoilFactor = 0.0;

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(Request);

            Assert.All(Result.Candidates, Candidate => Assert.Equal(Candidate.DistanceM, Candidate.ScoreM));
            Assert.Equal(
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_EAST_WEST),
                Result.Best.Settings);
        }

        /// <summary>
        /// Verifies where the winner changes as unfinished soil is charged more.
        /// Edge 40 with the rule on and east-west lanes leads through factor 10.
        /// The same edge with north-south lanes leads from 11 to 15, and the rule
        /// off leads from 16.
        /// </summary>
        /// <param name="Factor">Unfinished soil factor.</param>
        /// <param name="WorkLengthFt">Minimum work length of the expected winner.</param>
        /// <param name="ExpectedAxisDeg">Lane axis of the expected winner. Every winner uses edge 40.</param>
        [Theory]
        [InlineData(1.0, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_EAST_WEST)]
        [InlineData(5.0, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_EAST_WEST)]
        [InlineData(10.0, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_EAST_WEST)]
        [InlineData(11.0, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_NORTH_SOUTH)]
        [InlineData(15.0, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_NORTH_SOUTH)]
        [InlineData(16.0, SolverFixtures.WORK_OFF, SolverFixtures.AXIS_NORTH_SOUTH)]
        public void Solve_FactorSweep_WinnerChangesBetween10And16
            (
            Double Factor,
            Double WorkLengthFt,
            Double ExpectedAxisDeg
            )
        {
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.UnfinishedSoilFactor = Factor;

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(Request);

            Assert.Equal(
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, WorkLengthFt, ExpectedAxisDeg),
                Result.Best.Settings);
        }

        /// <summary>Verifies the unfinished-soil cap removes plans over it and never removes the baseline.</summary>
        [Fact]
        public void Solve_UnfinishedCap_PicksTheBestPlanUnderTheCap
            (
            )
        {
            const Double CAP_PERCENT = 0.1;
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.MaximumUnfinishedPercent = CAP_PERCENT;

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(Request);

            // Every measured plan leaves more than 0.1 percent, so the cap keeps the baseline.
            Assert.Equal(Result.Baseline.Settings, Result.Best.Settings);
            Assert.True(Result.Baseline.WithinLimits);
            Assert.True(Result.Baseline.UnfinishedPercent > CAP_PERCENT);
            Assert.Equal(0, Result.Candidates.Count(Candidate => Candidate.WithinLimits && (Candidate.RunIndex > 1)));
        }

        /// <summary>Verifies the manual-assist cap removes plans over it.</summary>
        [Fact]
        public void Solve_AssistCap_PicksTheBestPlanUnderTheCap
            (
            )
        {
            const Int32 CAP_ASSISTS = 111;
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.MaximumManualAssists = CAP_ASSISTS;

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(Request);

            Assert.Equal(
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_EAST_WEST),
                Result.Best.Settings);
        }

        /// <summary>Verifies equal plans resolve to the baseline, which has no changed setting.</summary>
        [Fact]
        public void Solve_EveryPlanEqual_KeepsTheBaseline
            (
            )
        {
            TableRunner Table = SolverFixtures.ShopTable();
            CandidateMeasurement Same = SolverFixtures.ShopRow(2116737.1, 228.749, 150.932, 116.522, 111);
            foreach (FieldSolverSettings Settings in SearchPlanner.BuildCombinations(
                FieldSolverSettings.FromOptions(new HaulBuilderOptions()),
                FieldSolverSearchSpace.CreateStandard()))
            {
                Table.Add(Settings, Same);
            }

            FieldSolverResult Result = new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest());

            Assert.Same(Result.Baseline, Result.Best);
            Assert.Equal(0.0, Result.SavingM);
        }

        /// <summary>Verifies that among equal better plans the one with the fewest changes wins, and then the earliest run.</summary>
        [Fact]
        public void Solve_EqualBetterPlans_PicksFewestChangesThenEarliestRun
            (
            )
        {
            const Int32 FIRST_SINGLE_CHANGE_RUN = 2;
            TableRunner Table = SolverFixtures.ShopTable();
            FieldSolverSettings Baseline = FieldSolverSettings.FromOptions(new HaulBuilderOptions());
            CandidateMeasurement Better = SolverFixtures.ShopRow(2001217.3, 216.598, 150.932, 116.522, 111);
            foreach (FieldSolverSettings Settings in SearchPlanner.BuildCombinations(Baseline, FieldSolverSearchSpace.CreateStandard()))
            {
                // Every plan except the baseline gets the same, better, result.
                if (!Settings.Equals(Baseline)) Table.Add(Settings, Better);
            }

            FieldSolverResult Result = new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest());

            Assert.Equal(FIRST_SINGLE_CHANGE_RUN, Result.Best.RunIndex);
            Assert.Equal(1, Result.Best.ChangeCount);
        }

        /// <summary>Verifies the run limit ends the search and returns the best of the runs made.</summary>
        [Fact]
        public void Solve_RunLimit_ReturnsTheBestOfTheRunsMade
            (
            )
        {
            const Int32 RUN_LIMIT = 3;
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.MaximumRuns = RUN_LIMIT;

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(Request);

            Assert.Equal(FieldSolverStatus.RunLimitReached, Result.Status);
            Assert.Equal(RUN_LIMIT, Result.Candidates.Count);
            Assert.Equal(SolverFixtures.STANDARD_RUNS, Result.PlannedRunCount);
            // Of baseline, edge 40, and rule off, edge 40 with the rule on scores lowest.
            Assert.Equal(
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_NORTH_SOUTH),
                Result.Best.Settings);
        }

        /// <summary>Verifies a cancel after the baseline returns the runs finished so far.</summary>
        [Fact]
        public void Solve_CancelAfterTheBaseline_ReturnsAPartialResult
            (
            )
        {
            const Int32 CANCEL_AT_RUN = 3;
            using (CancellationTokenSource Source = new CancellationTokenSource())
            {
                TableRunner Table = SolverFixtures.ShopTable();
                // The caller cancels while run 3 is in progress.
                Table.BeforeRun = RunIndex =>
                {
                    if (RunIndex == CANCEL_AT_RUN) Source.Cancel();
                };

                FieldSolverResult Result = new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest(), null, Source.Token);

                Assert.Equal(FieldSolverStatus.Cancelled, Result.Status);
                Assert.Equal(CANCEL_AT_RUN - 1, Result.Candidates.Count);
                Assert.Equal(
                    SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_NORTH_SOUTH),
                    Result.Best.Settings);
            }
        }

        /// <summary>Verifies a cancel during the baseline throws, because there is no plan to return.</summary>
        [Fact]
        public void Solve_CancelDuringTheBaseline_Throws
            (
            )
        {
            using (CancellationTokenSource Source = new CancellationTokenSource())
            {
                TableRunner Table = SolverFixtures.ShopTable();
                Table.BeforeRun = RunIndex => Source.Cancel();

                Assert.Throws<OperationCanceledException>(
                    () => new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest(), null, Source.Token));
            }
        }

        /// <summary>Verifies a token cancelled before the call throws without running anything.</summary>
        [Fact]
        public void Solve_TokenAlreadyCancelled_ThrowsBeforeAnyRun
            (
            )
        {
            using (CancellationTokenSource Source = new CancellationTokenSource())
            {
                TableRunner Table = SolverFixtures.ShopTable();
                Source.Cancel();

                Assert.Throws<OperationCanceledException>(
                    () => new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest(), null, Source.Token));
                Assert.Empty(Table.Calls);
            }
        }

        /// <summary>Verifies the time limit after the baseline returns the runs finished so far.</summary>
        [Fact]
        public void Solve_TimeLimitAfterTheBaseline_ReturnsAPartialResult
            (
            )
        {
            const Int32 SLOW_RUN = 2;
            TableRunner Table = SolverFixtures.ShopTable();
            // Run 2 does not end until the time limit fires.
            Table.WaitForStopAtRun = SLOW_RUN;
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            // Long enough for the baseline's table lookup on a busy machine.
            Request.TimeLimit = TimeSpan.FromSeconds(1.0);

            FieldSolverResult Result = new FieldSolverEngine(Table).Solve(Request);

            Assert.Equal(FieldSolverStatus.TimeLimitReached, Result.Status);
            Assert.Single(Result.Candidates);
            Assert.Same(Result.Baseline, Result.Best);
        }

        /// <summary>Verifies the time limit during the baseline throws, because there is no plan to return.</summary>
        [Fact]
        public void Solve_TimeLimitDuringTheBaseline_Throws
            (
            )
        {
            const Double TIME_LIMIT_MS = 50.0;
            TableRunner Table = SolverFixtures.ShopTable();
            Table.WaitForStopAtRun = 1;
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.TimeLimit = TimeSpan.FromMilliseconds(TIME_LIMIT_MS);

            Assert.Throws<TimeoutException>(() => new FieldSolverEngine(Table).Solve(Request));
        }

        /// <summary>Verifies a candidate the engine cannot plan is recorded as failed and the search goes on.</summary>
        [Fact]
        public void Solve_CandidateFails_IsRecordedAndTheSearchContinues
            (
            )
        {
            const Int32 FAILING_RUN = 3;
            TableRunner Table = SolverFixtures.ShopTable();
            Table.BeforeRun = RunIndex =>
            {
                if (RunIndex == FAILING_RUN) throw new InvalidOperationException("ledger validation failed");
            };

            FieldSolverResult Result = new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest());

            FieldSolverCandidate Failed = Result.Candidates[FAILING_RUN - 1];
            Assert.Equal(FieldSolverStatus.Completed, Result.Status);
            Assert.Equal(SolverFixtures.STANDARD_RUNS, Result.Candidates.Count);
            Assert.False(Failed.Succeeded);
            Assert.Equal("InvalidOperationException: ledger validation failed", Failed.FailureMessage);
            // A failed run has no score and must never be the best run.
            Assert.True(Result.Best.Succeeded);
            Assert.Equal(
                SolverFixtures.Primary(SolverFixtures.EDGE_LOOSE, SolverFixtures.WORK_DEFAULT, SolverFixtures.AXIS_EAST_WEST),
                Result.Best.Settings);
        }

        /// <summary>Verifies a baseline the engine cannot plan is not swallowed.</summary>
        [Fact]
        public void Solve_BaselineFails_ThrowsTheEngineError
            (
            )
        {
            TableRunner Table = SolverFixtures.ShopTable();
            Table.BeforeRun = RunIndex => throw new InvalidOperationException("baseline does not plan");

            InvalidOperationException Error = Assert.Throws<InvalidOperationException>(
                () => new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest()));

            Assert.Equal("baseline does not plan", Error.Message);
        }

        /// <summary>Verifies running out of memory ends the solve instead of being recorded as a failed candidate.</summary>
        [Fact]
        public void Solve_OutOfMemoryInACandidate_Throws
            (
            )
        {
            const Int32 FAILING_RUN = 3;
            TableRunner Table = SolverFixtures.ShopTable();
            Table.BeforeRun = RunIndex =>
            {
                if (RunIndex == FAILING_RUN) throw new OutOfMemoryException();
            };

            Assert.Throws<OutOfMemoryException>(() => new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest()));
        }

        /// <summary>Verifies a caller already on edge 40 still gets eight runs, with edge 12 as the change.</summary>
        [Fact]
        public void Solve_CallerAlreadyOnEdge40_TriesEdge12AsTheChange
            (
            )
        {
            TableRunner Table = SolverFixtures.ShopTable();
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.BaseOptions.Optimization.EdgeMaxLaneSlicesPerCycle = SolverFixtures.EDGE_LOOSE;

            FieldSolverResult Result = new FieldSolverEngine(Table).Solve(Request);

            Assert.Equal(SolverFixtures.STANDARD_RUNS, Result.Candidates.Count);
            Assert.Equal(SolverFixtures.EDGE_LOOSE, Result.Baseline.Settings.EdgeMaxLaneSlicesPerCycle);
            Assert.Equal(SolverFixtures.EDGE_DEFAULT, Table.Calls[1].EdgeMaxLaneSlicesPerCycle);
            Assert.Equal(1, Result.Candidates[1].ChangeCount);
        }

        /// <summary>Verifies a caller lane axis of 180 degrees is the north-south axis and adds no run.</summary>
        [Fact]
        public void Solve_CallerAxisOf180_IsTheNorthSouthAxis
            (
            )
        {
            const Double HALF_TURN_DEG = 180.0;
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.BaseOptions.LaneAxisHeadingDeg = HALF_TURN_DEG;

            FieldSolverResult Result = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(Request);

            Assert.Equal(SolverFixtures.STANDARD_RUNS, Result.Candidates.Count);
            Assert.Equal(SolverFixtures.AXIS_NORTH_SOUTH, Result.Baseline.Settings.LaneAxisHeadingDeg);
        }

        /// <summary>
        /// Verifies the thorough search: seven screening runs on the best combination,
        /// then one run that combines the two settings that helped. The screening rows
        /// are made up; they exercise the search, they are not measurements.
        /// </summary>
        [Fact]
        public void Solve_ThoroughSearch_CombinesTheScreeningWinners
            (
            )
        {
            const Int32 EXPECTED_RUNS = 16;
            const Double LOW_FLOW = 0.5;
            const Int32 FEWER_SLICES = 8;
            const Double BETTER_FLOW_M = 1990000.0;
            const Double BETTER_SLICES_M = 1995000.0;
            const Double COMBINED_M = 1985000.0;
            FieldSolverSettings Leader = SolverFixtures.Primary(
                SolverFixtures.EDGE_LOOSE,
                SolverFixtures.WORK_DEFAULT,
                SolverFixtures.AXIS_EAST_WEST);
            TableRunner Table = ThoroughTable(Leader);
            Table.Add(Leader with { FlowPenaltyWeight = LOW_FLOW }, LeaderRow(BETTER_FLOW_M));
            Table.Add(Leader with { MaxLaneSlicesPerCycle = FEWER_SLICES }, LeaderRow(BETTER_SLICES_M));
            Table.Add(Leader with { FlowPenaltyWeight = LOW_FLOW, MaxLaneSlicesPerCycle = FEWER_SLICES }, LeaderRow(COMBINED_M));
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.SearchSpace = FieldSolverSearchSpace.CreateThorough();

            FieldSolverResult Result = new FieldSolverEngine(Table).Solve(Request);

            Assert.Equal(EXPECTED_RUNS, Result.PlannedRunCount);
            Assert.Equal(EXPECTED_RUNS, Result.Candidates.Count);
            Assert.Equal(
                Leader with { FlowPenaltyWeight = LOW_FLOW, MaxLaneSlicesPerCycle = FEWER_SLICES },
                Result.Best.Settings);
            Assert.Equal(EXPECTED_RUNS, Result.Best.RunIndex);
        }

        /// <summary>Verifies that when one screened setting helps, it is the answer and no combining run is made.</summary>
        [Fact]
        public void Solve_ThoroughSearch_OneWinner_MakesNoCombiningRun
            (
            )
        {
            const Int32 EXPECTED_RUNS = 15;
            const Int32 PLANNED_RUNS = 16;
            const Double LOW_FLOW = 0.5;
            const Double BETTER_FLOW_M = 1990000.0;
            FieldSolverSettings Leader = SolverFixtures.Primary(
                SolverFixtures.EDGE_LOOSE,
                SolverFixtures.WORK_DEFAULT,
                SolverFixtures.AXIS_EAST_WEST);
            TableRunner Table = ThoroughTable(Leader);
            Table.Add(Leader with { FlowPenaltyWeight = LOW_FLOW }, LeaderRow(BETTER_FLOW_M));
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.SearchSpace = FieldSolverSearchSpace.CreateThorough();

            FieldSolverResult Result = new FieldSolverEngine(Table).Solve(Request);

            // The planned count is an upper bound; the combining run was not needed.
            Assert.Equal(PLANNED_RUNS, Result.PlannedRunCount);
            Assert.Equal(EXPECTED_RUNS, Result.Candidates.Count);
            Assert.Equal(FieldSolverStatus.Completed, Result.Status);
            Assert.Equal(Leader with { FlowPenaltyWeight = LOW_FLOW }, Result.Best.Settings);
        }

        /// <summary>Verifies options that skip planning or guidance are refused, because no distance would exist.</summary>
        /// <param name="SkipHaulPlan">Value for the caller's SkipHaulPlan.</param>
        /// <param name="SkipGuidance">Value for the caller's SkipGuidance.</param>
        [Theory]
        [InlineData(true, false)]
        [InlineData(false, true)]
        public void Solve_OptionsThatSkipThePlan_AreRefused
            (
            Boolean SkipHaulPlan,
            Boolean SkipGuidance
            )
        {
            TableRunner Table = SolverFixtures.ShopTable();
            FieldSolverRequest Request = SolverFixtures.DefaultRequest();
            Request.BaseOptions.SkipHaulPlan = SkipHaulPlan;
            Request.BaseOptions.SkipGuidance = SkipGuidance;

            Assert.Throws<ArgumentException>(() => new FieldSolverEngine(Table).Solve(Request));
            Assert.Empty(Table.Calls);
        }

        /// <summary>Verifies bad limits and bad search values are refused before any run is made.</summary>
        [Fact]
        public void Solve_ValuesOutOfRange_AreRefusedBeforeAnyRun
            (
            )
        {
            TableRunner Table = SolverFixtures.ShopTable();
            FieldSolverEngine Solver = new FieldSolverEngine(Table);

            FieldSolverRequest BadFactor = SolverFixtures.DefaultRequest();
            BadFactor.UnfinishedSoilFactor = -1.0;
            Assert.Throws<ArgumentOutOfRangeException>(() => Solver.Solve(BadFactor));

            FieldSolverRequest BadCap = SolverFixtures.DefaultRequest();
            BadCap.MaximumUnfinishedPercent = Double.NaN;
            Assert.Throws<ArgumentOutOfRangeException>(() => Solver.Solve(BadCap));

            FieldSolverRequest BadAssists = SolverFixtures.DefaultRequest();
            BadAssists.MaximumManualAssists = -1;
            Assert.Throws<ArgumentOutOfRangeException>(() => Solver.Solve(BadAssists));

            FieldSolverRequest BadRuns = SolverFixtures.DefaultRequest();
            BadRuns.MaximumRuns = -1;
            Assert.Throws<ArgumentOutOfRangeException>(() => Solver.Solve(BadRuns));

            FieldSolverRequest BadTime = SolverFixtures.DefaultRequest();
            BadTime.TimeLimit = TimeSpan.Zero;
            Assert.Throws<ArgumentOutOfRangeException>(() => Solver.Solve(BadTime));

            // The engine itself would refuse this value, but only when its run came up.
            FieldSolverRequest BadEdge = SolverFixtures.DefaultRequest();
            BadEdge.SearchSpace.EdgeMaxLaneSlicesPerCycleValues.Add(0);
            Assert.Throws<ArgumentOutOfRangeException>(() => Solver.Solve(BadEdge));

            FieldSolverRequest BadAxis = SolverFixtures.DefaultRequest();
            BadAxis.SearchSpace.LaneAxisHeadingDegValues.Add(Double.PositiveInfinity);
            Assert.Throws<ArgumentOutOfRangeException>(() => Solver.Solve(BadAxis));

            Assert.Throws<ArgumentNullException>(() => Solver.Solve(null!));
            Assert.Empty(Table.Calls);
        }

        /// <summary>Verifies a second solve is refused while one runs, and allowed again afterwards.</summary>
        [Fact]
        public void Solve_WhileAnotherSolveRuns_IsRefused
            (
            )
        {
            TableRunner Table = SolverFixtures.ShopTable();
            Boolean InnerRefused = false;
            // A solve started from inside the baseline run of another solve.
            Table.BeforeRun = RunIndex =>
            {
                if (RunIndex != 1) return;
                try
                {
                    new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(SolverFixtures.DefaultRequest());
                }
                catch (InvalidOperationException)
                {
                    InnerRefused = true;
                }
            };

            new FieldSolverEngine(Table).Solve(SolverFixtures.DefaultRequest());

            Assert.True(InnerRefused, "A second solve was allowed while one was running.");
            // The guard must be released when the first solve ends.
            FieldSolverResult Next = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(SolverFixtures.DefaultRequest());
            Assert.Equal(SolverFixtures.STANDARD_RUNS, Next.Candidates.Count);
        }

        /// <summary>Verifies the guard is released when a solve ends with an exception.</summary>
        [Fact]
        public void Solve_AfterAFailedSolve_IsAllowedAgain
            (
            )
        {
            TableRunner Broken = SolverFixtures.ShopTable();
            Broken.BeforeRun = RunIndex => throw new InvalidOperationException("baseline does not plan");
            Assert.Throws<InvalidOperationException>(() => new FieldSolverEngine(Broken).Solve(SolverFixtures.DefaultRequest()));

            FieldSolverResult Next = new FieldSolverEngine(SolverFixtures.ShopTable()).Solve(SolverFixtures.DefaultRequest());

            Assert.Equal(FieldSolverStatus.Completed, Next.Status);
        }

        /// <summary>
        /// Builds the measured table plus one made-up row for each of the seven
        /// screening values, every one worse than the leader.
        /// </summary>
        /// <param name="Leader">The best of the eight measured combinations.</param>
        /// <returns>A table the thorough search can run on.</returns>
        static private TableRunner ThoroughTable
            (
            FieldSolverSettings Leader
            )
        {
            const Double WORSE_M = 2100000.0;
            const Double LOW_FLOW = 0.5;
            const Double HIGH_FLOW = 2.0;
            const Int32 FEWER_SLICES = 8;
            const Int32 MORE_SLICES = 14;
            const Int32 WIDE_WINDOW = 1000;
            TableRunner Table = SolverFixtures.ShopTable();
            // SYNTHETIC rows: the screened settings were never measured on the leader.
            Table.Add(Leader with { FlowPenaltyWeight = LOW_FLOW }, LeaderRow(WORSE_M));
            Table.Add(Leader with { FlowPenaltyWeight = HIGH_FLOW }, LeaderRow(WORSE_M));
            Table.Add(Leader with { MaxLaneSlicesPerCycle = FEWER_SLICES }, LeaderRow(WORSE_M));
            Table.Add(Leader with { MaxLaneSlicesPerCycle = MORE_SLICES }, LeaderRow(WORSE_M));
            Table.Add(Leader with { RouteReorderWindow = WIDE_WINDOW }, LeaderRow(WORSE_M));
            Table.Add(Leader with { EnableMultiRouteExchange = true }, LeaderRow(WORSE_M));
            Table.Add(Leader with { EnableCycleChainOptimization = true }, LeaderRow(WORSE_M));
            return Table;
        }

        /// <summary>Builds a made-up row that differs from the measured leader only in its distance.</summary>
        /// <param name="DistanceM">Pretended whole-job driven metres.</param>
        /// <returns>The row.</returns>
        static private CandidateMeasurement LeaderRow
            (
            Double DistanceM
            )
        {
            const Double LEADER_HOURS = 216.598;
            const Double LEADER_DEFERRED_CUT = 207.249;
            const Double LEADER_DEFERRED_FILL = 87.020;
            const Int32 LEADER_ASSISTS = 141;
            return SolverFixtures.ShopRow(DistanceM, LEADER_HOURS, LEADER_DEFERRED_CUT, LEADER_DEFERRED_FILL, LEADER_ASSISTS);
        }

        /// <summary>Collects progress snapshots in a list on the reporting thread.</summary>
        private sealed class ListProgress : IProgress<FieldSolverProgress>
        {
            /// <summary>List that receives every snapshot.</summary>
            private readonly List<FieldSolverProgress> _Reports;

            /// <summary>Initializes the collector.</summary>
            /// <param name="Reports">List that receives every snapshot.</param>
            public ListProgress
                (
                List<FieldSolverProgress> Reports
                )
            {
                _Reports = Reports;
            }

            /// <summary>Stores one snapshot.</summary>
            /// <param name="Value">Snapshot to store.</param>
            public void Report
                (
                FieldSolverProgress Value
                )
            {
                _Reports.Add(Value);
            }
        }
    }
}
