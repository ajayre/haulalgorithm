using System;
using System.Collections.Generic;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>Verifies fill direction is scored from the post-grade exit.</summary>
    public sealed class GradeAwareFillDirectionTests
    {
        /// <summary>
        /// Resolves grade aware fill direction reverses toward next cut after grade.
        /// </summary>

        [Fact]
        public void SelectGradeAwareFillDirection_ReversesTowardNextCutAfterGrade()
        {
            Grid Grid = MakeGrid();
            CoveragePlanner.Run NorthToSouth = MakeRun(
                (40.5, 85.5),
                (40.5, 25.5),
                Constants.DEGREES_PER_HALF_TURN,
                (40, 55));
            ConnectorPlanner.Pose ArtificialNorthAnchor =
                new ConnectorPlanner.Pose(65.5, 90.5, 270.0);
            RunSequencer.Result Existing = RunSequencer.Optimize(
                new List<CoveragePlanner.Run> { NorthToSouth },
                ArtificialNorthAnchor,
                null,
                9.0,
                8.0,
                Constants.HALF_SCALE,
                Grid);
            Assert.True(Existing.Success, Existing.FailureReason);
            Assert.True(Existing.Runs[0].Run.StartXy.Y
                > Existing.Runs[0].Run.EndXy.Y);

            ConnectorPlanner.Pose CutExit =
                new ConnectorPlanner.Pose(70.5, 30.5, 270.0);
            ConnectorPlanner.Pose NextCutEntry =
                new ConnectorPlanner.Pose(75.5, 25.5, 90.0);
            RunSequencer.Result Selected =
                GuidancePlanner.SelectGradeAwareFillDirection(
                    new List<CoveragePlanner.Run> { NorthToSouth },
                    Existing,
                    Grid,
                    CutExit,
                    NextCutEntry,
                    out Double BaselineScoreM,
                    out Double SelectedScoreM,
                    out Int32 CandidatesEvaluated);

            Assert.True(Selected.Success, Selected.FailureReason);
            Assert.True(CandidatesEvaluated >= 2);
            Assert.True(SelectedScoreM < BaselineScoreM);
            Assert.True(Selected.Runs[0].Run.StartXy.Y
                < Selected.Runs[0].Run.EndXy.Y);
            Assert.Equal(NorthToSouth.Cells, Selected.Runs[0].Run.Cells);
        }

        /// <summary>
        /// Verifies reverse prepared sequence preserves allocation and reverses endpoints.
        /// </summary>

        [Fact]
        public void ReversePreparedSequence_PreservesAllocationAndReversesEndpoints()
        {
            Grid Grid = MakeGrid();
            CoveragePlanner.Run Run = MakeRun(
                (40.5, 85.5),
                (40.5, 25.5),
                Constants.DEGREES_PER_HALF_TURN,
                (40, 55));
            RunSequencer.Result Forward = RunSequencer.Optimize(
                new List<CoveragePlanner.Run> { Run },
                null,
                null,
                9.0,
                8.0,
                Constants.HALF_SCALE,
                Grid);

            RunSequencer.Result Reversed =
                LifecyclePlanner.ReversePreparedSequence(Forward);

            Assert.True(Reversed.Success);
            Assert.Equal(Forward.Runs[0].Run.StartXy, Reversed.Runs[0].Run.EndXy);
            Assert.Equal(Forward.Runs[0].Run.EndXy, Reversed.Runs[0].Run.StartXy);
            Assert.Equal(Forward.Runs[0].Run.Cells, Reversed.Runs[0].Run.Cells);
        }

        /// <summary>
        /// Builds run.
        /// </summary>
        /// <param name="Start">The start used by the operation.</param>
        /// <param name="End">The end used by the operation.</param>
        /// <param name="HeadingDeg">Direction in degrees.</param>
        /// <param name="Cell">The cell used by the operation.</param>
        /// <returns>The resulting run.</returns>

        private static CoveragePlanner.Run MakeRun
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double HeadingDeg,
            (Int32 Bx, Int32 By) Cell
            ) => new CoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (Cell, 0.02),
                },
                Start,
                End,
                HeadingDeg,
                0.0);

        /// <summary>
        /// Builds grid.
        /// </summary>
        /// <returns>The resulting grid.</returns>

        private static Grid MakeGrid()
        {
            const Int32 GRID_SIZE = 120;
            Boolean[,] Valid = new Boolean[GRID_SIZE, GRID_SIZE];
            for (Int32 By = 0; By < GRID_SIZE; By += 1)
            {
                for (Int32 Bx = 0; Bx < GRID_SIZE; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                GRID_SIZE,
                GRID_SIZE,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[GRID_SIZE, GRID_SIZE],
                new Double[GRID_SIZE, GRID_SIZE],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
