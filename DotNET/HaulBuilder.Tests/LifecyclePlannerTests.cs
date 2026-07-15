using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies explicit post-fill routing with grade return disabled and enabled.
    /// </summary>
    public sealed class LifecyclePlannerTests
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
            List<LifecyclePlanner.PassPlan> Passes = MakePasses(Grid);

            LifecyclePlanner.Result Result = LifecyclePlanner.Build(
                Passes, false, LINEUP_M, RADIUS_M, SAMPLE_M, Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.DoesNotContain(Result.Segments,
                Segment => Segment.Phase == LifecyclePlanner.Phase.Grade);
            Int32 FirstFillExit = Result.Segments.FindIndex(Segment =>
                Segment.Phase == LifecyclePlanner.Phase.ExitFill);
            Int32 SecondCutLineup = Result.Segments.FindIndex(FirstFillExit + 1, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.LineupCut);
            Assert.True(SecondCutLineup > FirstFillExit);
            Assert.Contains(Result.Segments.Skip(FirstFillExit + 1).Take(SecondCutLineup - FirstFillExit - 1),
                Segment => (Segment.Phase == LifecyclePlanner.Phase.TransitToNextCut)
                    || (Segment.Phase == LifecyclePlanner.Phase.ManualAssist));
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
            List<LifecyclePlanner.PassPlan> Passes = MakePasses(Grid);

            LifecyclePlanner.Result Result = LifecyclePlanner.Build(
                Passes, true, LINEUP_M, RADIUS_M, SAMPLE_M, Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Int32 FirstFill = Result.Segments.FindIndex(Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Fill);
            Int32 FirstGrade = Result.Segments.FindIndex(Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Grade);
            Int32 SecondCut = Result.Segments.FindIndex(FirstGrade + 1, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Cut);
            Assert.True(FirstFill >= 0);
            Assert.True(FirstGrade > FirstFill);
            Assert.True(SecondCut > FirstGrade);
            LifecyclePlanner.Segment Fill = Result.Segments[FirstFill];
            LifecyclePlanner.Segment Grade = Result.Segments[FirstGrade];
            LifecyclePlanner.Segment FirstCut = Result.Segments[Result.Segments
                .FindIndex(Segment => Segment.Phase == LifecyclePlanner.Phase.Cut)];
            Assert.Equal(Fill.Points[0], Grade.Points[Grade.Points.Count - 1]);
            Assert.Equal(Fill.Points[Fill.Points.Count - 1], Grade.Points[0]);
            Assert.False(FirstCut.Loaded);
            Assert.True(Fill.Loaded);
            Assert.False(Grade.Loaded);
            Assert.All(Result.Segments.Where(Segment => Segment.Phase ==
                LifecyclePlanner.Phase.TransitToNextCut),
                Segment => Assert.False(Segment.Loaded));
            Assert.All(Result.Segments.Where(Segment => Segment.Phase ==
                LifecyclePlanner.Phase.Grade), Segment => Assert.True(Segment.BladeDown));
            for (Int32 Index = 1; Index < Result.Segments.Count; Index += 1)
            {
                LifecyclePlanner.Segment Previous = Result.Segments[Index - 1];
                LifecyclePlanner.Segment Current = Result.Segments[Index];
                Assert.Equal(
                    Previous.Points[Previous.Points.Count - 1],
                    Current.Points[0]);
            }
        }

        /// <summary>
        /// Verifies endpoint distance borrowed from straight work is performed along
        /// the adjacent automatic curve instead of becoming blade-up deadheading.
        /// </summary>
        [Fact]
        public void Build_FlexibleFillEndpoints_KeepBladeDownOnConnectorTails
            (
            )
        {
            Grid Grid = MakeGrid();
            CoveragePlanner.Run FirstRun = MakeRun(4.0, 8.0, 10.0, 90.0);
            CoveragePlanner.Run SecondRun = MakeRun(16.0, 20.0, 12.0, 90.0);
            RunSequencer.SequencedRun First =
                new RunSequencer.SequencedRun(
                    FirstRun,
                    new ConnectorPlanner.Pose(4.0, 10.0, 90.0),
                    new ConnectorPlanner.Pose(8.0, 10.0, 90.0),
                    0.0,
                    0.0,
                    0.0,
                    2.0);
            RunSequencer.SequencedRun Second =
                new RunSequencer.SequencedRun(
                    SecondRun,
                    new ConnectorPlanner.Pose(16.0, 12.0, 90.0),
                    new ConnectorPlanner.Pose(20.0, 12.0, 90.0),
                    0.0,
                    0.0,
                    3.0,
                    0.0);
            List<(Double X, Double Y)> ConnectorPoints =
                new List<(Double X, Double Y)>
                {
                    (8.0, 10.0),
                    (10.0, 10.0),
                    (12.0, 11.0),
                    (14.0, 12.0),
                    (16.0, 12.0),
                };
            Double ConnectorLengthM = 0.0;
            for (Int32 Index = 1; Index < ConnectorPoints.Count; Index += 1)
            {
                Double Dx = ConnectorPoints[Index].X - ConnectorPoints[Index - 1].X;
                Double Dy = ConnectorPoints[Index].Y - ConnectorPoints[Index - 1].Y;
                ConnectorLengthM += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            TransitPlanner.Result Connector =
                new TransitPlanner.Result(
                    true,
                    false,
                    ConnectorPoints,
                    ConnectorLengthM,
                    new List<TransitPlanner.Leg>
                    {
                        new TransitPlanner.Leg(false, ConnectorPoints),
                    },
                    0.0,
                    "AUTO_CURVE",
                    String.Empty);
            RunSequencer.Result Fill = new RunSequencer.Result(
                true,
                new List<RunSequencer.SequencedRun> { First, Second },
                new List<TransitPlanner.Result> { Connector },
                ConnectorLengthM,
                0.0,
                String.Empty);
            RunSequencer.Result Empty = new RunSequencer.Result(
                true,
                new List<RunSequencer.SequencedRun>(),
                new List<TransitPlanner.Result>(),
                0.0,
                0.0,
                String.Empty);

            LifecyclePlanner.Result Result = LifecyclePlanner.Build(
                new List<LifecyclePlanner.PassPlan>
                {
                    new LifecyclePlanner.PassPlan(Empty, Fill),
                },
                false,
                LINEUP_M,
                RADIUS_M,
                SAMPLE_M,
                Grid);

            Assert.True(Result.Success, Result.FailureReason);
            List<LifecyclePlanner.Segment> FillSegments = Result.Segments
                .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.Fill)
                .ToList();
            Assert.Equal(4, FillSegments.Count);
            Assert.All(FillSegments, Segment => Assert.True(Segment.BladeDown));
            Assert.Equal(13.0, FillSegments.Sum(SegmentLength), 6);
            Assert.Equal(
                ConnectorLengthM - 5.0,
                Result.Segments
                    .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.Transit)
                    .Sum(SegmentLength),
                6);
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
            RunSequencer.SequencedRun Main =
                RunSequencer.Prepare(
                    MakeRun(21.1, 29.4, 48.2, 120.0, 43.4),
                    EQUIPMENT_M,
                    TURN_RADIUS_M,
                    SAMPLE_M,
                    Grid);
            RunSequencer.SequencedRun Fragment =
                RunSequencer.Prepare(
                    MakeRun(18.6, 16.4, 50.4, 300.0, 51.6),
                    EQUIPMENT_M,
                    TURN_RADIUS_M,
                    SAMPLE_M,
                    Grid);
            TransitPlanner.Result LegacyConnector =
                TransitPlanner.Connect(
                    Main.ExitPose,
                    Fragment.EntryPose,
                    TURN_RADIUS_M,
                    SAMPLE_M,
                    Grid);
            RunSequencer.Result Fill = new RunSequencer.Result(
                true,
                new List<RunSequencer.SequencedRun> { Main, Fragment },
                new List<TransitPlanner.Result> { LegacyConnector },
                LegacyConnector.LengthM,
                Math.Min(Main.EntryLineupM, Fragment.EntryLineupM),
                String.Empty);
            RunSequencer.Result Empty = new RunSequencer.Result(
                true,
                new List<RunSequencer.SequencedRun>(),
                new List<TransitPlanner.Result>(),
                0.0,
                0.0,
                String.Empty);

            LifecyclePlanner.Result Result = LifecyclePlanner.Build(
                new List<LifecyclePlanner.PassPlan>
                {
                    new LifecyclePlanner.PassPlan(Empty, Fill),
                },
                true,
                EQUIPMENT_M,
                TURN_RADIUS_M,
                SAMPLE_M,
                Grid);

            Assert.True(Result.Success, Result.FailureReason);
            List<LifecyclePlanner.Segment> Fills = Result.Segments.Where(
                Segment => Segment.Phase == LifecyclePlanner.Phase.Fill).ToList();
            List<LifecyclePlanner.Segment> Grades = Result.Segments.Where(
                Segment => Segment.Phase == LifecyclePlanner.Phase.Grade).ToList();
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
                Segment => Segment.Phase == LifecyclePlanner.Phase.Fill);
            Int32 FirstGradeIndex = Result.Segments.FindIndex(
                Segment => Segment.Phase == LifecyclePlanner.Phase.Grade);
            List<LifecyclePlanner.Segment> Turnaround = Result.Segments
                .Skip(LastFillIndex + 1)
                .Take(FirstGradeIndex - LastFillIndex - 1)
                .Where(Segment => SegmentLength(Segment) > 1e-9)
                .ToList();
            Assert.Single(Turnaround);
            Assert.Equal(
                LifecyclePlanner.Phase.TurnToGrade,
                Turnaround[0].Phase);
            Assert.True(MaximumInteriorDeflectionDeg(Turnaround[0].Points) < 90.0);
            Assert.All(Result.Segments.Where(Segment => Segment.Phase
                == LifecyclePlanner.Phase.Transit),
                Segment => Assert.InRange(SegmentLength(Segment), 3.0, 4.0));
        }

        /// <summary>
        /// Verifies the allocation-free grade feasibility check agrees with a full
        /// lifecycle build when the supplied fill sequence is already prepared.
        /// </summary>
        [Fact]
        public void SupportsPreparedGradeReturn_AgreesWithPreparedLifecycleBuild
            (
            )
        {
            Grid Grid = MakeGrid();
            RunSequencer.Result Fill = MakePass(
                Grid, 5.5, 15.5, 15.5).Fill;
            RunSequencer.Result Empty =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun>(),
                    new List<TransitPlanner.Result>(),
                    0.0,
                    0.0,
                    String.Empty);

            Boolean Supported =
                LifecyclePlanner.SupportsPreparedGradeReturn(
                    Fill,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid,
                    out String FailureReason);
            LifecyclePlanner.Result Lifecycle =
                LifecyclePlanner.Build(
                    new List<LifecyclePlanner.PassPlan>
                    {
                        new LifecyclePlanner.PassPlan(Empty, Fill),
                    },
                    true,
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid,
                    FillSequencesPrepared: true);

            Assert.Equal(Lifecycle.Success, Supported);
            Assert.True(Supported, FailureReason);
        }

        /// <summary>
        /// Verifies a loaded westbound approach fills an aligned corridor immediately
        /// instead of deadheading west across it and returning to fill eastbound.
        /// </summary>
        [Fact]
        public void ContinuousFillSequence_LoadedAlignedApproachChoosesInboundDirection
            (
            )
        {
            Grid Grid = MakeGrid();
            RunSequencer.SequencedRun WestFragment =
                RunSequencer.Prepare(
                    MakeRun(12.0, 20.0, 15.0, 90.0),
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            RunSequencer.SequencedRun EastFragment =
                RunSequencer.Prepare(
                    MakeRun(21.0, 29.0, 15.0, 90.0),
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            TransitPlanner.Result LegacyConnector =
                TransitPlanner.Connect(
                    WestFragment.ExitPose,
                    EastFragment.EntryPose,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            Assert.True(LegacyConnector.Success, LegacyConnector.FailureReason);
            RunSequencer.Result Original =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun>
                    {
                        WestFragment,
                        EastFragment,
                    },
                    new List<TransitPlanner.Result> { LegacyConnector },
                    LegacyConnector.LengthM,
                    LINEUP_M,
                    String.Empty);
            ConnectorPlanner.Pose LoadedInbound =
                new ConnectorPlanner.Pose(35.0, 15.0, 270.0);

            RunSequencer.Result Result =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Original,
                    LoadedInbound,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.Equal(2, Result.Runs.Count);
            Assert.All(Result.Runs, Run => Assert.InRange(
                Run.Run.HeadingDeg, 269.999, 270.001));
            Assert.Equal((29.0, 15.0), Result.Runs[0].Run.StartXy);
            Assert.Equal((12.0, 15.0), Result.Runs[1].Run.EndXy);
            Assert.Single(Result.Connectors);
            Assert.Equal("AUTO_CORRIDOR_BRIDGE", Result.Connectors[0].Mode);
        }

        /// <summary>
        /// Verifies a fill aimed toward a field edge is stopped early enough to
        /// make its forward-only grade turnaround. The uncovered tail is recorded
        /// as coverage shortfall instead of creating an impossible endpoint turn.
        /// </summary>
        [Fact]
        public void ContinuousFillSequence_FieldEdgeRetreatsForGradeTurn
            (
            )
        {
            Grid Grid = MakeGrid();
            CoveragePlanner.Run EdgeFill =
                MakeRun(20.0, 0.5, 12.0, 270.0);
            RunSequencer.SequencedRun Prepared =
                RunSequencer.Prepare(
                    EdgeFill,
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            RunSequencer.Result Original =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun>
                    {
                        Prepared,
                    },
                    new List<TransitPlanner.Result>(),
                    0.0,
                    LINEUP_M,
                    String.Empty);

            RunSequencer.Result Result =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Original,
                    null,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.Single(Result.Runs);
            Assert.True(Result.Runs[0].Run.EndXy.X > EdgeFill.EndXy.X);
            Assert.True(Result.Runs[0].EndCoverageShortfallM > 0.0);

            RunSequencer.Result Empty =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun>(),
                    new List<TransitPlanner.Result>(),
                    0.0,
                    0.0,
                    String.Empty);
            LifecyclePlanner.Result Lifecycle =
                LifecyclePlanner.Build(
                    new List<LifecyclePlanner.PassPlan>
                    {
                        new LifecyclePlanner.PassPlan(Empty, Result),
                    },
                    true,
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Lifecycle.Success, Lifecycle.FailureReason);
            Assert.Contains(Lifecycle.Segments, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.TurnToGrade);
        }

        /// <summary>
        /// Verifies a nearby parallel fill fragment borrows work distance for a
        /// compact blade-down lane shift and retraces it while grading.
        /// </summary>
        [Fact]
        public void ContinuousFillSequence_ParallelFragmentUsesReversibleWorkLaneShift
            (
            )
        {
            Grid Grid = MakeGrid();
            RunSequencer.SequencedRun LongRun =
                RunSequencer.Prepare(
                    MakeRun(30.0, 10.0, 15.0, 270.0),
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            RunSequencer.SequencedRun ShortRun =
                RunSequencer.Prepare(
                    MakeRun(10.0, 8.0, 16.5, 270.0),
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            TransitPlanner.Result LegacyConnector =
                TransitPlanner.Connect(
                    LongRun.ExitPose,
                    ShortRun.EntryPose,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            Assert.True(LegacyConnector.Success, LegacyConnector.FailureReason);
            RunSequencer.Result Original =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun>
                    {
                        LongRun,
                        ShortRun,
                    },
                    new List<TransitPlanner.Result> { LegacyConnector },
                    LegacyConnector.LengthM,
                    LINEUP_M,
                    String.Empty);

            RunSequencer.Result Continuous =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Original,
                    new ConnectorPlanner.Pose(35.0, 15.0, 270.0),
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Continuous.Success, Continuous.FailureReason);
            Assert.Single(Continuous.Connectors);
            Assert.Equal(
                "AUTO_WORK_CORRIDOR_BRIDGE",
                Continuous.Connectors[0].Mode);
            Assert.True(Continuous.Connectors[0].LengthM < 4.0 * RADIUS_M);
            Assert.True(Continuous.Runs[0].Run.EndXy.X > 10.0);

            RunSequencer.Result Empty =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun>(),
                    new List<TransitPlanner.Result>(),
                    0.0,
                    0.0,
                    String.Empty);
            LifecyclePlanner.Result Lifecycle =
                LifecyclePlanner.Build(
                    new List<LifecyclePlanner.PassPlan>
                    {
                        new LifecyclePlanner.PassPlan(Empty, Continuous),
                    },
                    true,
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Lifecycle.Success, Lifecycle.FailureReason);
            Assert.Equal(3, Lifecycle.Segments.Count(Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Fill));
            Assert.Equal(3, Lifecycle.Segments.Count(Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Grade));
            Assert.Single(Lifecycle.Segments, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.TurnToGrade);
            Assert.DoesNotContain(Lifecycle.Segments, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Transit
                && SegmentLength(Segment) > 4.0 * RADIUS_M);
        }

        /// <summary>
        /// Verifies a one-cell fragment embedded beside a longer fill is absorbed
        /// into that blade-width swath instead of forcing fill and grade loops.
        /// </summary>
        [Fact]
        public void ContinuousFillSequence_NestedCellMergesBeforeLaneShift
            (
            )
        {
            const Double HAUL_GRID_M = 8.5 * Constants.FEET_TO_METERS;
            const Double EQUIPMENT_M = Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                * Constants.FEET_TO_METERS;
            const Double TURN_RADIUS_M = 0.5 * Constants.DEFAULT_TURNING_CIRCLE_FT
                * Constants.FEET_TO_METERS;
            Grid Grid = MakeGrid(HAUL_GRID_M);
            RunSequencer.SequencedRun First =
                RunSequencer.Prepare(
                    MakeRun(74.3, 58.9, 24.4, 275.0, 25.8),
                    EQUIPMENT_M, TURN_RADIUS_M, SAMPLE_M, Grid);
            RunSequencer.SequencedRun Long =
                RunSequencer.Prepare(
                    MakeRun(56.3, 19.4, 22.6, 275.0, 25.8),
                    EQUIPMENT_M, TURN_RADIUS_M, SAMPLE_M, Grid);
            RunSequencer.SequencedRun Nested =
                RunSequencer.Prepare(
                    MakeRun(43.3, 45.9, 25.2, 95.0, 25.0),
                    EQUIPMENT_M, TURN_RADIUS_M, SAMPLE_M, Grid);
            TransitPlanner.Result FirstConnector =
                TransitPlanner.Connect(
                    First.ExitPose, Long.EntryPose,
                    TURN_RADIUS_M, SAMPLE_M, Grid);
            TransitPlanner.Result SecondConnector =
                TransitPlanner.Connect(
                    Long.ExitPose, Nested.EntryPose,
                    TURN_RADIUS_M, SAMPLE_M, Grid);
            Assert.True(FirstConnector.Success, FirstConnector.FailureReason);
            Assert.True(SecondConnector.Success, SecondConnector.FailureReason);
            RunSequencer.Result Original =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun>
                    {
                        First,
                        Long,
                        Nested,
                    },
                    new List<TransitPlanner.Result>
                    {
                        FirstConnector,
                        SecondConnector,
                    },
                    FirstConnector.LengthM + SecondConnector.LengthM,
                    EQUIPMENT_M,
                    String.Empty);

            RunSequencer.Result Continuous =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Original,
                    new ConnectorPlanner.Pose(76.9, 23.8, 270.0),
                    TURN_RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Continuous.Success, Continuous.FailureReason);
            Assert.Equal(2, Continuous.Runs.Count);
            Assert.Single(Continuous.Connectors);
            Assert.Equal(
                "AUTO_WORK_CORRIDOR_BRIDGE",
                Continuous.Connectors[0].Mode);
            Assert.Contains(Continuous.Runs, Run => Run.Run.Cells.Count == 2);

            RunSequencer.Result Empty =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun>(),
                    new List<TransitPlanner.Result>(),
                    0.0,
                    0.0,
                    String.Empty);
            LifecyclePlanner.Result Lifecycle =
                LifecyclePlanner.Build(
                    new List<LifecyclePlanner.PassPlan>
                    {
                        new LifecyclePlanner.PassPlan(Empty, Continuous),
                    },
                    true,
                    EQUIPMENT_M,
                    TURN_RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Lifecycle.Success, Lifecycle.FailureReason);
            Assert.Single(Lifecycle.Segments, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.TurnToGrade);
            Assert.DoesNotContain(Lifecycle.Segments, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Transit);
        }

        /// <summary>
        /// Verifies a close comb of fill cells becomes one blade-down envelope
        /// corridor while retaining the exact allocated cell/depth multiset for
        /// redistribution by the reverse grading run.
        /// </summary>
        [Fact]
        public void ContinuousFillSequence_CloseCombUsesFillEnvelope
            (
            )
        {
            Grid Grid = MakeCoordinateGrid();
            List<CoveragePlanner.Run> Fragments = Enumerable.Range(0, 6)
                .Select(Index => MakeCellRun(
                    Grid,
                    6 + Index,
                    10 + (Index % 2),
                    90.0))
                .ToList();
            List<RunSequencer.SequencedRun> Prepared = Fragments
                .Select(Run => RunSequencer.Prepare(
                    Run, LINEUP_M, RADIUS_M, SAMPLE_M, Grid))
                .ToList();
            RunSequencer.Result Original = new RunSequencer.Result(
                true,
                Prepared,
                new List<TransitPlanner.Result>(),
                0.0,
                LINEUP_M,
                String.Empty);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Expected =
                Fragments.SelectMany(Run => Run.Cells).ToList();

            RunSequencer.Result Continuous =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Original,
                    new ConnectorPlanner.Pose(2.0, 10.5, 90.0),
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Continuous.Success, Continuous.FailureReason);
            Assert.Single(Continuous.Runs);
            Assert.True(CoveragePlanner.ConservesAllocation(
                Expected,
                Continuous.Runs.SelectMany(Run => Run.Run.Cells).ToList()));
            Assert.True(
                Continuous.Runs[0].Run.LengthM > Grid.GridSizeM,
                Continuous.Runs[0].Run.LengthM.ToString());

            RunSequencer.Result Empty = new RunSequencer.Result(
                true,
                new List<RunSequencer.SequencedRun>(),
                new List<TransitPlanner.Result>(),
                0.0,
                0.0,
                String.Empty);
            LifecyclePlanner.Result Lifecycle = LifecyclePlanner.Build(
                new List<LifecyclePlanner.PassPlan>
                {
                    new LifecyclePlanner.PassPlan(Empty, Continuous),
                },
                true,
                LINEUP_M,
                RADIUS_M,
                SAMPLE_M,
                Grid,
                FillSequencesPrepared: true);

            Assert.True(Lifecycle.Success, Lifecycle.FailureReason);
            Assert.Single(Lifecycle.Segments, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Fill);
            Assert.Single(Lifecycle.Segments, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.Grade);
        }

        /// <summary>
        /// Verifies an oblique close-proximity cluster selects its geometric axis
        /// instead of inheriting an unrelated fragment or cardinal heading.
        /// </summary>
        [Fact]
        public void ContinuousFillSequence_DiagonalClusterUsesAdaptiveHeading
            (
            )
        {
            Grid Grid = MakeCoordinateGrid();
            List<CoveragePlanner.Run> Fragments = Enumerable.Range(0, 6)
                .Select(Index => MakeCellRun(
                    Grid,
                    6 + Index,
                    6 + Index,
                    90.0))
                .ToList();
            RunSequencer.Result Original = new RunSequencer.Result(
                true,
                Fragments.Select(Run => RunSequencer.Prepare(
                    Run, LINEUP_M, RADIUS_M, SAMPLE_M, Grid)).ToList(),
                new List<TransitPlanner.Result>(),
                0.0,
                LINEUP_M,
                String.Empty);

            RunSequencer.Result Continuous =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Original,
                    null,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Continuous.Success, Continuous.FailureReason);
            Assert.Single(Continuous.Runs);
            Assert.InRange(
                FieldEdgePolicy.AxialHeadingDifferenceDeg(
                    Continuous.Runs[0].Run.HeadingDeg, 45.0),
                0.0,
                0.25);
            Assert.True(CoveragePlanner.ConservesAllocation(
                Fragments.SelectMany(Run => Run.Cells).ToList(),
                Continuous.Runs.SelectMany(Run => Run.Run.Cells).ToList()));
        }

        /// <summary>
        /// Verifies one distant difficult cell remains separate while a nearby
        /// fragmented component is compacted at its own geometric heading.
        /// </summary>
        [Fact]
        public void ContinuousFillSequence_DistantOutlierDoesNotSetClusterHeading
            (
            )
        {
            Grid Grid = MakeCoordinateGrid();
            List<CoveragePlanner.Run> Fragments = Enumerable.Range(0, 10)
                .Select(Index => MakeCellRun(
                    Grid,
                    25,
                    20 + Index,
                    145.0))
                .Append(MakeCellRun(Grid, 4, 4, 145.0))
                .ToList();
            RunSequencer.Result Original = new RunSequencer.Result(
                true,
                Fragments.Select(Run => RunSequencer.Prepare(
                    Run, LINEUP_M, RADIUS_M, SAMPLE_M, Grid)).ToList(),
                new List<TransitPlanner.Result>(),
                0.0,
                LINEUP_M,
                String.Empty);

            RunSequencer.Result Continuous =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Original,
                    null,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.True(Continuous.Success, Continuous.FailureReason);
            Assert.Equal(2, Continuous.Runs.Count);
            CoveragePlanner.Run Cluster = Continuous.Runs
                .Select(Run => Run.Run)
                .Single(Run => Run.Cells.Count == 10);
            Assert.InRange(
                FieldEdgePolicy.AxialHeadingDifferenceDeg(
                    Cluster.HeadingDeg, 0.0),
                0.0,
                0.25);
            Assert.True(CoveragePlanner.ConservesAllocation(
                Fragments.SelectMany(Run => Run.Cells).ToList(),
                Continuous.Runs.SelectMany(Run => Run.Run.Cells).ToList()));
        }

        /// <summary>
        /// Verifies sparse fragments are not joined when blade-down work distance
        /// would grow beyond the bounded redistribution envelope.
        /// </summary>
        [Fact]
        public void ContinuousFillSequence_SparseCellsRejectFillEnvelope
            (
            )
        {
            Grid Grid = MakeCoordinateGrid();
            List<CoveragePlanner.Run> Fragments = new List<CoveragePlanner.Run>
            {
                MakeCellRun(Grid, 5, 12, 90.0),
                MakeCellRun(Grid, 9, 12, 90.0),
                MakeCellRun(Grid, 13, 12, 90.0),
                MakeCellRun(Grid, 17, 12, 90.0),
            };
            RunSequencer.Result Original = new RunSequencer.Result(
                true,
                Fragments.Select(Run => RunSequencer.Prepare(
                    Run, LINEUP_M, RADIUS_M, SAMPLE_M, Grid)).ToList(),
                new List<TransitPlanner.Result>(),
                0.0,
                LINEUP_M,
                String.Empty);

            RunSequencer.Result Continuous =
                LifecyclePlanner.BuildContinuousFillSequence(
                    Original,
                    null,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);

            Assert.Equal(4, Continuous.Runs.Count);
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
            RunSequencer.SequencedRun First =
                RunSequencer.Prepare(
                    MakeRun(10.0, 14.0, 15.0, 90.0),
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            RunSequencer.SequencedRun Second =
                RunSequencer.Prepare(
                    MakeRun(15.0, 19.0, 15.3, 90.0, 15.3),
                    LINEUP_M,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            TransitPlanner.Result LegacyConnector =
                TransitPlanner.Connect(
                    First.ExitPose,
                    Second.EntryPose,
                    RADIUS_M,
                    SAMPLE_M,
                    Grid);
            Assert.True(LegacyConnector.Success, LegacyConnector.FailureReason);
            RunSequencer.Result Original =
                new RunSequencer.Result(
                    true,
                    new List<RunSequencer.SequencedRun> { First, Second },
                    new List<TransitPlanner.Result> { LegacyConnector },
                    LegacyConnector.LengthM,
                    Math.Min(First.EntryLineupM, Second.EntryLineupM),
                    String.Empty);

            RunSequencer.Result Result =
                LifecyclePlanner.BuildContinuousCutSequence(
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
            RunSequencer.SequencedRun First =
                RunSequencer.Prepare(
                    MakeRun(20.0, 16.0, 10.0, 270.0),
                    LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            RunSequencer.SequencedRun Middle =
                RunSequencer.Prepare(
                    MakeRun(10.0, 20.0, 15.0, 90.0),
                    LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            RunSequencer.SequencedRun Last =
                RunSequencer.Prepare(
                    MakeRun(23.0, 21.0, 10.5, 270.0, 10.5),
                    LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            TransitPlanner.Result FirstConnector =
                TransitPlanner.Connect(
                    First.ExitPose, Middle.EntryPose,
                    RADIUS_M, SAMPLE_M, Grid);
            TransitPlanner.Result SecondConnector =
                TransitPlanner.Connect(
                    Middle.ExitPose, Last.EntryPose,
                    RADIUS_M, SAMPLE_M, Grid);
            Assert.True(FirstConnector.Success, FirstConnector.FailureReason);
            Assert.True(SecondConnector.Success, SecondConnector.FailureReason);
            RunSequencer.Result Original = new RunSequencer.Result(
                true,
                new List<RunSequencer.SequencedRun> { First, Middle, Last },
                new List<TransitPlanner.Result>
                {
                    FirstConnector,
                    SecondConnector,
                },
                FirstConnector.LengthM + SecondConnector.LengthM,
                LINEUP_M,
                String.Empty);

            RunSequencer.Result Result =
                LifecyclePlanner.BuildContinuousCutSequence(
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
            LifecyclePlanner.Segment Segment
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

        /// <summary>Returns the largest sampled heading change within a polyline.</summary>
        private static Double MaximumInteriorDeflectionDeg
            (
            List<(Double X, Double Y)> Points
            )
        {
            Double MaximumDeg = 0.0;
            for (Int32 Index = 1; Index + 1 < Points.Count; Index += 1)
            {
                (Double X, Double Y) Before = Points[Index - 1];
                (Double X, Double Y) Point = Points[Index];
                (Double X, Double Y) After = Points[Index + 1];
                Double InX = Point.X - Before.X;
                Double InY = Point.Y - Before.Y;
                Double OutX = After.X - Point.X;
                Double OutY = After.Y - Point.Y;
                Double Denominator = Math.Sqrt((InX * InX) + (InY * InY))
                    * Math.Sqrt((OutX * OutX) + (OutY * OutY));
                if (Denominator <= 1e-12) continue;
                Double Cos = Math.Clamp(
                    ((InX * OutX) + (InY * OutY)) / Denominator,
                    -1.0,
                    1.0);
                MaximumDeg = Math.Max(MaximumDeg, Math.Acos(Cos) * 180.0 / Math.PI);
            }
            return MaximumDeg;
        }

        /// <summary>Creates two simple cut/fill pass plans.</summary>
        private static List<LifecyclePlanner.PassPlan> MakePasses
            (
            Grid Grid
            )
        {
            return new List<LifecyclePlanner.PassPlan>
            {
                MakePass(Grid, 5.5, 15.5, 15.5),
                MakePass(Grid, 22.5, 32.5, 22.5),
            };
        }

        /// <summary>Creates and sequences one cut and fill pair.</summary>
        private static LifecyclePlanner.PassPlan MakePass
            (
            Grid Grid,
            Double CutX,
            Double FillX,
            Double Y
            )
        {
            CoveragePlanner.Run Cut = MakeRun(CutX, CutX + 6.0, Y, 90.0);
            CoveragePlanner.Run Fill = MakeRun(FillX, FillX + 6.0, Y, 90.0);
            RunSequencer.Result CutResult = RunSequencer.Optimize(
                new List<CoveragePlanner.Run> { Cut },
                null, null, LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            RunSequencer.Result FillResult = RunSequencer.Optimize(
                new List<CoveragePlanner.Run> { Fill },
                null, null, LINEUP_M, RADIUS_M, SAMPLE_M, Grid);
            return new LifecyclePlanner.PassPlan(CutResult, FillResult);
        }

        /// <summary>Creates one allocated eastbound run.</summary>
        private static CoveragePlanner.Run MakeRun
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
        private static CoveragePlanner.Run MakeRun
            (
            Double StartX,
            Double EndX,
            Double StartY,
            Double HeadingDeg,
            Double EndY
            )
        {
            return new CoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (((Int32)StartX, (Int32)StartY), 0.02),
                },
                (StartX, StartY),
                (EndX, EndY),
                HeadingDeg,
                0.0);
        }

        /// <summary>Creates one cell-centered blade-width work fragment.</summary>
        private static CoveragePlanner.Run MakeCellRun
            (
            Grid Grid,
            Int32 Bx,
            Int32 By,
            Double HeadingDeg
            )
        {
            (Double X, Double Y) Center = Grid.CellCenterXy(Bx, By);
            Double HeadingRad = HeadingDeg * Math.PI / 180.0;
            Double HalfM = 0.49 * Grid.GridSizeM;
            Double Dx = Math.Sin(HeadingRad) * HalfM;
            Double Dy = Math.Cos(HeadingRad) * HalfM;
            return new CoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((Bx, By), 0.02),
                },
                (Center.X - Dx, Center.Y - Dy),
                (Center.X + Dx, Center.Y + Dy),
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

        /// <summary>Creates an all-valid grid with physical cell centers.</summary>
        private static Grid MakeCoordinateGrid
            (
            )
        {
            const Int32 WIDTH = 50;
            const Int32 HEIGHT = 40;
            Boolean[,] Valid = new Boolean[HEIGHT, WIDTH];
            Double[,] CenterX = new Double[HEIGHT, WIDTH];
            Double[,] CenterY = new Double[HEIGHT, WIDTH];
            for (Int32 By = 0; By < HEIGHT; By += 1)
            {
                for (Int32 Bx = 0; Bx < WIDTH; Bx += 1)
                {
                    Valid[By, Bx] = true;
                    CenterX[By, Bx] = Bx + 0.5;
                    CenterY[By, Bx] = By + 0.5;
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
                CenterX,
                CenterY,
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
