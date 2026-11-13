using System;
using System.IO;
using System.Linq;

using Xunit;

using HaulBuilder;

namespace FieldSolver.Tests
{
    /// <summary>
    /// Runs the solver with the real HaulBuilder engine on a tiny field. The test
    /// checks what must hold on any engine version, not particular distances, so
    /// an improvement to the engine does not break it.
    /// </summary>
    public sealed class FieldSolverIntegrationTests
    {
        /// <summary>
        /// Verifies a four-run solve end to end: the baseline is the caller's plan,
        /// the best run has the lowest score, the returned options reproduce the
        /// best run exactly, and the engine's cache is empty afterwards.
        /// </summary>
        [Fact]
        [Trait("Category", "Integration")]
        public void Solve_TinyField_ReturnsOptionsThatReproduceTheBestRun
            (
            )
        {
            const Int32 EXPECTED_RUNS = 4;
            String AgdPath = SolverFixtures.CreateTinyAgd();
            try
            {
                HaulBuilderOptions Options = new HaulBuilderOptions
                {
                    AgdPath = AgdPath,
                    // East-west lanes follow this field's haul direction, which keeps
                    // every run of the test to a few seconds.
                    LaneAxisHeadingDeg = SolverFixtures.AXIS_EAST_WEST,
                };
                FieldSolverRequest Request = new FieldSolverRequest(Options);
                // Two values for two settings: four engine runs of a few seconds each.
                Request.SearchSpace = new FieldSolverSearchSpace();
                Request.SearchSpace.EdgeMaxLaneSlicesPerCycleValues.Add(SolverFixtures.EDGE_DEFAULT);
                Request.SearchSpace.EdgeMaxLaneSlicesPerCycleValues.Add(SolverFixtures.EDGE_LOOSE);
                Request.SearchSpace.MinimumWorkLengthFtValues.Add(SolverFixtures.WORK_DEFAULT);
                Request.SearchSpace.MinimumWorkLengthFtValues.Add(SolverFixtures.WORK_OFF);

                FieldSolverResult Result = new FieldSolverEngine().Solve(Request);

                Assert.Equal(FieldSolverStatus.Completed, Result.Status);
                Assert.Equal(EXPECTED_RUNS, Result.Candidates.Count);
                Assert.All(Result.Candidates, Candidate => Assert.True(Candidate.Succeeded, Candidate.FailureMessage));
                Assert.All(Result.Candidates, Candidate => Assert.True(Candidate.DistanceM > 0.0));
                Assert.Equal(FieldSolverSettings.FromOptions(Options), Result.Baseline.Settings);
                Assert.Equal(Result.Candidates.Min(Candidate => Candidate.ScoreM), Result.Best.ScoreM);
                Assert.True(Result.Best.ScoreM <= Result.Baseline.ScoreM);
                Assert.Equal(Result.Baseline.DistanceM - Result.Best.DistanceM, Result.SavingM);
                // The solver left nothing behind in the engine's process-wide cache.
                Assert.Equal(0, TransitPlanner.GetCacheStatistics().Entries);
                Assert.Equal(0, TransitPlanner.GetCacheStatistics().RetainedEntries);

                // The returned options must give the plan the solver measured.
                HaulBuilderResult Direct = new HaulBuilderEngine().Run(Result.BestOptions);
                TransitPlanner.ResetCache();
                Assert.Equal(Result.Best.DistanceM, Direct.DetailedGuidanceTime!.DistanceM);
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }
    }
}
