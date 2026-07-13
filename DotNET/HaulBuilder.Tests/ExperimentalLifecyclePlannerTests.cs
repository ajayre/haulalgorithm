using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies explicit post-fill routing with grade return disabled and enabled.
    /// </summary>
    public sealed class ExperimentalLifecyclePlannerTests
    {
        private const Double LINEUP_M = 3.0;
        private const Double RADIUS_M = 4.0;
        private const Double SAMPLE_M = 0.5;

        /// <summary>
        /// Verifies fill completion routes to the next cut and never emits grade phases.
        /// </summary>
        [Fact]
        public void Build_GradeOff_RoutesFillExitToNextCut
            (
            )
        {
            Grid Grid = MakeGrid();
            List<ExperimentalLifecyclePlanner.PassPlan> Passes = MakePasses(Grid);

            ExperimentalLifecyclePlanner.Result Result = ExperimentalLifecyclePlanner.Build(
                Passes, false, LINEUP_M, RADIUS_M, SAMPLE_M, Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.DoesNotContain(Result.Segments,
                Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.Grade);
            Int32 FirstFillExit = Result.Segments.FindIndex(Segment =>
                Segment.Phase == ExperimentalLifecyclePlanner.Phase.ExitFill);
            Int32 SecondCutLineup = Result.Segments.FindIndex(FirstFillExit + 1, Segment =>
                Segment.Phase == ExperimentalLifecyclePlanner.Phase.LineupCut);
            Assert.True(SecondCutLineup > FirstFillExit);
            Assert.Contains(Result.Segments.Skip(FirstFillExit + 1).Take(SecondCutLineup - FirstFillExit - 1),
                Segment => (Segment.Phase == ExperimentalLifecyclePlanner.Phase.TransitToNextCut)
                    || (Segment.Phase == ExperimentalLifecyclePlanner.Phase.ManualAssist));
        }

        /// <summary>
        /// Verifies grade return traverses the completed fill in reverse before the next cut.
        /// </summary>
        [Fact]
        public void Build_GradeOn_GradesThenRoutesToNextCut
            (
            )
        {
            Grid Grid = MakeGrid();
            List<ExperimentalLifecyclePlanner.PassPlan> Passes = MakePasses(Grid);

            ExperimentalLifecyclePlanner.Result Result = ExperimentalLifecyclePlanner.Build(
                Passes, true, LINEUP_M, RADIUS_M, SAMPLE_M, Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Int32 FirstFill = Result.Segments.FindIndex(Segment =>
                Segment.Phase == ExperimentalLifecyclePlanner.Phase.Fill);
            Int32 FirstGrade = Result.Segments.FindIndex(Segment =>
                Segment.Phase == ExperimentalLifecyclePlanner.Phase.Grade);
            Int32 SecondCut = Result.Segments.FindIndex(FirstGrade + 1, Segment =>
                Segment.Phase == ExperimentalLifecyclePlanner.Phase.Cut);
            Assert.True(FirstFill >= 0);
            Assert.True(FirstGrade > FirstFill);
            Assert.True(SecondCut > FirstGrade);
            ExperimentalLifecyclePlanner.Segment Fill = Result.Segments[FirstFill];
            ExperimentalLifecyclePlanner.Segment Grade = Result.Segments[FirstGrade];
            Assert.Equal(Fill.Points[0], Grade.Points[Grade.Points.Count - 1]);
            Assert.Equal(Fill.Points[Fill.Points.Count - 1], Grade.Points[0]);
            Assert.All(Result.Segments.Where(Segment => Segment.Phase ==
                ExperimentalLifecyclePlanner.Phase.Grade), Segment => Assert.True(Segment.BladeDown));
        }

        /// <summary>Creates two simple cut/fill pass plans.</summary>
        private static List<ExperimentalLifecyclePlanner.PassPlan> MakePasses
            (
            Grid Grid
            )
        {
            return new List<ExperimentalLifecyclePlanner.PassPlan>
            {
                MakePass(Grid, 5.5, 15.5, 8.5),
                MakePass(Grid, 22.5, 32.5, 12.5),
            };
        }

        /// <summary>Creates and sequences one cut and fill pair.</summary>
        private static ExperimentalLifecyclePlanner.PassPlan MakePass
            (
            Grid Grid,
            Double CutX,
            Double FillX,
            Double Y
            )
        {
            ExperimentalCoveragePlanner.Run Cut = MakeRun(CutX, CutX + 6.0, Y, 90.0);
            ExperimentalCoveragePlanner.Run Fill = MakeRun(FillX, FillX + 6.0, Y, 90.0);
            ExperimentalRunSequencer.Result CutResult = ExperimentalRunSequencer.Optimize(
                new List<ExperimentalCoveragePlanner.Run> { Cut },
                null, null, LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            ExperimentalRunSequencer.Result FillResult = ExperimentalRunSequencer.Optimize(
                new List<ExperimentalCoveragePlanner.Run> { Fill },
                null, null, LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            return new ExperimentalLifecyclePlanner.PassPlan(CutResult, FillResult);
        }

        /// <summary>Creates one allocated eastbound run.</summary>
        private static ExperimentalCoveragePlanner.Run MakeRun
            (
            Double StartX,
            Double EndX,
            Double Y,
            Double HeadingDeg
            )
        {
            return new ExperimentalCoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (((Int32)StartX, (Int32)Y), 0.02),
                },
                (StartX, Y),
                (EndX, Y),
                HeadingDeg,
                0.0);
        }

        /// <summary>Creates a large all-valid raw field.</summary>
        private static Grid MakeGrid
            (
            )
        {
            const Int32 WIDTH = 50;
            const Int32 HEIGHT = 20;
            Boolean[,] Valid = new Boolean[HEIGHT, WIDTH];
            for (Int32 By = 0; By < HEIGHT; By += 1)
            {
                for (Int32 Bx = 0; Bx < WIDTH; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                WIDTH,
                HEIGHT,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[HEIGHT, WIDTH],
                new Double[HEIGHT, WIDTH],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
