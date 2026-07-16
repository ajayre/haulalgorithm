using System;
using System.Collections.Generic;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>Verifies fill direction is scored from the post-grade exit.</summary>
    public sealed class GradeAwareFillDirectionTests
    {
        [Fact]
        public void SelectGradeAwareFillDirection_ReversesTowardNextCutAfterGrade()
        {
            Grid Grid = MakeGrid();
            CoveragePlanner.Run NorthToSouth = MakeRun(
                (40.5, 85.5),
                (40.5, 25.5),
                180.0,
                (40, 55));
            ConnectorPlanner.Pose ArtificialNorthAnchor =
                new ConnectorPlanner.Pose(65.5, 90.5, 270.0);
            RunSequencer.Result Existing = RunSequencer.Optimize(
                new List<CoveragePlanner.Run> { NorthToSouth },
                ArtificialNorthAnchor,
                null,
                9.0,
                8.0,
                0.5,
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

        [Fact]
        public void ReversePreparedSequence_PreservesAllocationAndReversesEndpoints()
        {
            Grid Grid = MakeGrid();
            CoveragePlanner.Run Run = MakeRun(
                (40.5, 85.5),
                (40.5, 25.5),
                180.0,
                (40, 55));
            RunSequencer.Result Forward = RunSequencer.Optimize(
                new List<CoveragePlanner.Run> { Run },
                null,
                null,
                9.0,
                8.0,
                0.5,
                Grid);

            RunSequencer.Result Reversed =
                LifecyclePlanner.ReversePreparedSequence(Forward);

            Assert.True(Reversed.Success);
            Assert.Equal(Forward.Runs[0].Run.StartXy, Reversed.Runs[0].Run.EndXy);
            Assert.Equal(Forward.Runs[0].Run.EndXy, Reversed.Runs[0].Run.StartXy);
            Assert.Equal(Forward.Runs[0].Run.Cells, Reversed.Runs[0].Run.Cells);
        }

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

        private static Grid MakeGrid()
        {
            const Int32 Size = 120;
            Boolean[,] Valid = new Boolean[Size, Size];
            for (Int32 By = 0; By < Size; By += 1)
            {
                for (Int32 Bx = 0; Bx < Size; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                Size,
                Size,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[Size, Size],
                new Double[Size, Size],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
