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

        /// <summary>
        /// Verifies collinear fragments from one route are filled continuously,
        /// turned once, and graded continuously in reverse order.
        /// </summary>
        [Fact]
        public void Build_GradeOn_CollinearFillFragmentsUseOneTurnaround
            (
            )
        {
            const Double HAUL_GRID_M = 8.5 * Constants.FEET_TO_METERS;
            const Double EQUIPMENT_M = Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                * Constants.FEET_TO_METERS;
            const Double TURN_RADIUS_M = 0.5 * Constants.DEFAULT_TURNING_CIRCLE_FT
                * Constants.FEET_TO_METERS;
            Grid Grid = MakeGrid(HAUL_GRID_M);
            ExperimentalRunSequencer.SequencedRun Main =
                ExperimentalRunSequencer.Prepare(
                    MakeRun(21.1, 29.4, 48.2, 120.0, 43.4),
                    EQUIPMENT_M,
                    TURN_RADIUS_M,
                    SAMPLE_M,
                    Grid);
            ExperimentalRunSequencer.SequencedRun Fragment =
                ExperimentalRunSequencer.Prepare(
                    MakeRun(18.6, 16.4, 50.4, 300.0, 51.6),
                    EQUIPMENT_M,
                    TURN_RADIUS_M,
                    SAMPLE_M,
                    Grid);
            ExperimentalTransitPlanner.Result LegacyConnector =
                ExperimentalTransitPlanner.Connect(
                    Main.ExitPose,
                    Fragment.EntryPose,
                    TURN_RADIUS_M,
                    SAMPLE_M,
                    Grid);
            ExperimentalRunSequencer.Result Fill = new ExperimentalRunSequencer.Result(
                true,
                new List<ExperimentalRunSequencer.SequencedRun> { Main, Fragment },
                new List<ExperimentalTransitPlanner.Result> { LegacyConnector },
                LegacyConnector.LengthM,
                Math.Min(Main.EntryLineupM, Fragment.EntryLineupM),
                String.Empty);
            ExperimentalRunSequencer.Result Empty = new ExperimentalRunSequencer.Result(
                true,
                new List<ExperimentalRunSequencer.SequencedRun>(),
                new List<ExperimentalTransitPlanner.Result>(),
                0.0,
                0.0,
                String.Empty);

            ExperimentalLifecyclePlanner.Result Result = ExperimentalLifecyclePlanner.Build(
                new List<ExperimentalLifecyclePlanner.PassPlan>
                {
                    new ExperimentalLifecyclePlanner.PassPlan(Empty, Fill),
                },
                true,
                EQUIPMENT_M,
                TURN_RADIUS_M,
                SAMPLE_M,
                Grid);

            Assert.True(Result.Success, Result.FailureReason);
            List<ExperimentalLifecyclePlanner.Segment> Fills = Result.Segments.Where(
                Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.Fill).ToList();
            List<ExperimentalLifecyclePlanner.Segment> Grades = Result.Segments.Where(
                Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.Grade).ToList();
            Assert.Equal(2, Fills.Count);
            Assert.Equal(2, Grades.Count);
            Assert.Equal((16.4, 51.6), Fills[0].Points[0]);
            Assert.Equal((18.6, 50.4), Fills[0].Points[Fills[0].Points.Count - 1]);
            Assert.Equal((21.1, 48.2), Fills[1].Points[0]);
            Assert.Equal((29.4, 43.4), Fills[1].Points[Fills[1].Points.Count - 1]);
            Assert.Equal((29.4, 43.4), Grades[0].Points[0]);
            Assert.Equal((21.1, 48.2), Grades[0].Points[Grades[0].Points.Count - 1]);
            Assert.Equal((18.6, 50.4), Grades[1].Points[0]);
            Assert.Equal((16.4, 51.6), Grades[1].Points[Grades[1].Points.Count - 1]);
            Int32 LastFillIndex = Result.Segments.FindLastIndex(
                Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.Fill);
            Int32 FirstGradeIndex = Result.Segments.FindIndex(
                Segment => Segment.Phase == ExperimentalLifecyclePlanner.Phase.Grade);
            List<ExperimentalLifecyclePlanner.Segment> Turnaround = Result.Segments
                .Skip(LastFillIndex + 1)
                .Take(FirstGradeIndex - LastFillIndex - 1)
                .Where(Segment => SegmentLength(Segment) > 1e-9)
                .ToList();
            Assert.Single(Turnaround);
            Assert.True(
                (Turnaround[0].Phase == ExperimentalLifecyclePlanner.Phase.TurnToGrade)
                    || (Turnaround[0].Phase
                        == ExperimentalLifecyclePlanner.Phase.ManualAssist));
            Assert.All(Result.Segments.Where(Segment => Segment.Phase
                == ExperimentalLifecyclePlanner.Phase.Transit),
                Segment => Assert.InRange(SegmentLength(Segment), 3.0, 4.0));
        }

        /// <summary>
        /// Verifies nearby aligned cut fragments retain only the outer lineups and
        /// use one short blade-up jog instead of a minimum-radius internal turn.
        /// </summary>
        [Fact]
        public void ContinuousCutSequence_UsesShortJogBetweenAlignedFragments
            (
            )
        {
            Grid Grid = MakeGrid();
            ExperimentalRunSequencer.SequencedRun First =
                ExperimentalRunSequencer.Prepare(
                    MakeRun(10.0, 14.0, 15.0, 90.0),
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            ExperimentalRunSequencer.SequencedRun Second =
                ExperimentalRunSequencer.Prepare(
                    MakeRun(15.0, 19.0, 15.3, 90.0, 15.3),
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            ExperimentalTransitPlanner.Result LegacyConnector =
                ExperimentalTransitPlanner.Connect(
                    First.ExitPose,
                    Second.EntryPose,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            Assert.True(LegacyConnector.Success, LegacyConnector.FailureReason);
            ExperimentalRunSequencer.Result Original =
                new ExperimentalRunSequencer.Result(
                    true,
                    new List<ExperimentalRunSequencer.SequencedRun> { First, Second },
                    new List<ExperimentalTransitPlanner.Result> { LegacyConnector },
                    LegacyConnector.LengthM,
                    Math.Min(First.EntryLineupM, Second.EntryLineupM),
                    String.Empty);

            ExperimentalRunSequencer.Result Result =
                ExperimentalLifecyclePlanner.BuildContinuousCutSequence(
                    Original,
                    null,
                    null,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.Equal("AUTO_CORRIDOR_BRIDGE", Result.Connectors[0].Mode);
            Assert.True(Result.Runs[0].EntryLineupM > 0.0);
            Assert.Equal(Result.Runs[0].Run.EndXy.X, Result.Runs[0].ExitPose.X);
            Assert.Equal(Result.Runs[0].Run.EndXy.Y, Result.Runs[0].ExitPose.Y);
            Assert.Equal(Result.Runs[1].Run.StartXy.X, Result.Runs[1].EntryPose.X);
            Assert.Equal(Result.Runs[1].Run.StartXy.Y, Result.Runs[1].EntryPose.Y);
            Assert.True(Result.Runs[1].ExitLineupM > 0.0);
            Assert.Equal(0.0, Result.Runs[0].ExitLineupM);
            Assert.Equal(0.0, Result.Runs[1].EntryLineupM);
            Assert.InRange(Result.Connectors[0].LengthM, 1.0, 1.1);
            Assert.True(Result.Connectors[0].LengthM < LegacyConnector.LengthM);
        }

        /// <summary>Verifies compatible fragments can be made adjacent before jogging.</summary>
        [Fact]
        public void ContinuousCutSequence_ReordersSeparatedCompatibleFragments
            (
            )
        {
            Grid Grid = MakeGrid();
            ExperimentalRunSequencer.SequencedRun First =
                ExperimentalRunSequencer.Prepare(
                    MakeRun(20.0, 16.0, 10.0, 270.0),
                    LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            ExperimentalRunSequencer.SequencedRun Middle =
                ExperimentalRunSequencer.Prepare(
                    MakeRun(10.0, 20.0, 15.0, 90.0),
                    LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            ExperimentalRunSequencer.SequencedRun Last =
                ExperimentalRunSequencer.Prepare(
                    MakeRun(23.0, 21.0, 10.5, 270.0, 10.5),
                    LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            ExperimentalTransitPlanner.Result FirstConnector =
                ExperimentalTransitPlanner.Connect(
                    First.ExitPose, Middle.EntryPose,
                    RADIUS_M, SAMPLE_M, Grid);
            ExperimentalTransitPlanner.Result SecondConnector =
                ExperimentalTransitPlanner.Connect(
                    Middle.ExitPose, Last.EntryPose,
                    RADIUS_M, SAMPLE_M, Grid);
            Assert.True(FirstConnector.Success, FirstConnector.FailureReason);
            Assert.True(SecondConnector.Success, SecondConnector.FailureReason);
            ExperimentalRunSequencer.Result Original = new ExperimentalRunSequencer.Result(
                true,
                new List<ExperimentalRunSequencer.SequencedRun> { First, Middle, Last },
                new List<ExperimentalTransitPlanner.Result>
                {
                    FirstConnector,
                    SecondConnector,
                },
                FirstConnector.LengthM + SecondConnector.LengthM,
                LINEUP_M,
                String.Empty);

            ExperimentalRunSequencer.Result Result =
                ExperimentalLifecyclePlanner.BuildContinuousCutSequence(
                    Original,
                    null,
                    null,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.Contains(Result.Connectors,
                Connector => Connector.Mode == "AUTO_CORRIDOR_BRIDGE");
            Int32 FirstIndex = Result.Runs.FindIndex(Run =>
                Run.Run.Cells[0].Cell == (20, 10));
            Int32 LastIndex = Result.Runs.FindIndex(Run =>
                Run.Run.Cells[0].Cell == (23, 10));
            Assert.Equal(1, Math.Abs(FirstIndex - LastIndex));
        }

        /// <summary>Returns one lifecycle segment's polyline length.</summary>
        private static Double SegmentLength
            (
            ExperimentalLifecyclePlanner.Segment Segment
            )
        {
            Double LengthM = 0.0;
            for (Int32 Index = 1; Index < Segment.Points.Count; Index += 1)
            {
                Double Dx = Segment.Points[Index].X - Segment.Points[Index - 1].X;
                Double Dy = Segment.Points[Index].Y - Segment.Points[Index - 1].Y;
                LengthM += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            return LengthM;
        }

        /// <summary>Creates two simple cut/fill pass plans.</summary>
        private static List<ExperimentalLifecyclePlanner.PassPlan> MakePasses
            (
            Grid Grid
            )
        {
            return new List<ExperimentalLifecyclePlanner.PassPlan>
            {
                MakePass(Grid, 5.5, 15.5, 15.5),
                MakePass(Grid, 22.5, 32.5, 22.5),
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
            return MakeRun(StartX, EndX, Y, HeadingDeg, Y);
        }

        /// <summary>Creates one allocated run with explicit endpoint ordinates.</summary>
        private static ExperimentalCoveragePlanner.Run MakeRun
            (
            Double StartX,
            Double EndX,
            Double StartY,
            Double HeadingDeg,
            Double EndY
            )
        {
            return new ExperimentalCoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (((Int32)StartX, (Int32)StartY), 0.02),
                },
                (StartX, StartY),
                (EndX, EndY),
                HeadingDeg,
                0.0);
        }

        /// <summary>Creates a large all-valid raw field.</summary>
        private static Grid MakeGrid
            (
            )
        {
            return MakeGrid(1.0);
        }

        /// <summary>Creates a large all-valid raw field at a requested grid scale.</summary>
        private static Grid MakeGrid
            (
            Double GridSizeM
            )
        {
            const Int32 WIDTH = 50;
            const Int32 HEIGHT = 40;
            Boolean[,] Valid = new Boolean[HEIGHT, WIDTH];
            for (Int32 By = 0; By < HEIGHT; By += 1)
            {
                for (Int32 Bx = 0; Bx < WIDTH; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                GridSizeM,
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
