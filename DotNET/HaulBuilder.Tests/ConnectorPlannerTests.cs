using System;
using System.Collections.Generic;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies strict field-bounded minimum-radius connectors.
    /// </summary>
    public sealed class ConnectorPlannerTests
    {
        [Fact]
        public void UnconstrainedDistanceIsExactForStraightAlignedPoses
            (
            )
        {
            Double LengthM = ConnectorPlanner.ShortestPathLengthM(
                new ConnectorPlanner.Pose(5.0, 5.0, 0.0),
                new ConnectorPlanner.Pose(5.0, 25.0, 0.0),
                7.62);

            Assert.Equal(20.0, LengthM, 9);
        }

        private const Double TURN_RADIUS_M = 10.0;
        private const Double SAMPLE_SPACING_M = 0.5;

        /// <summary>
        /// Verifies a feasible connector is sampled inside the field with exact endpoints.
        /// </summary>
        [Fact]
        public void Connect_BuildsStrictInFieldPath
            (
            )
        {
            Grid Grid = MakeGrid(100, 100);
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(20.5, 20.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(60.5, 40.5, 90.0);

            ConnectorPlanner.Result Result = ConnectorPlanner.Connect(
                Start, End, TURN_RADIUS_M, SAMPLE_SPACING_M, Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.NotEmpty(Result.Word);
            Assert.Equal((Start.X, Start.Y), Result.Points[0]);
            Assert.Equal((End.X, End.Y), Result.Points[Result.Points.Count - 1]);
            Assert.All(Result.Points, Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
            AssertMaximumStep(Result.Points, SAMPLE_SPACING_M + 1e-6);
        }

        /// <summary>
        /// Verifies a narrow field rejects a reverse-heading turn instead of emitting a chord.
        /// </summary>
        [Fact]
        public void Connect_FailsWhenEveryRadiusValidTurnLeavesField
            (
            )
        {
            Grid Grid = MakeGrid(20, 5);
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(2.5, 2.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(17.5, 2.5, 270.0);

            ConnectorPlanner.Result Result = ConnectorPlanner.Connect(
                Start, End, 5.0, SAMPLE_SPACING_M, Grid);

            Assert.False(Result.Success);
            Assert.Empty(Result.Points);
            Assert.Contains("leaves the raw field", Result.FailureReason, StringComparison.Ordinal);
        }

        /// <summary>
        /// Verifies sampling density does not create oversized gaps on a straight connector.
        /// </summary>
        [Fact]
        public void Connect_HonorsSampleSpacingOnStraightPosePair
            (
            )
        {
            Grid Grid = MakeGrid(100, 100);
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(20.5, 20.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(60.5, 20.5, 90.0);

            ConnectorPlanner.Result Result = ConnectorPlanner.Connect(
                Start, End, TURN_RADIUS_M, SAMPLE_SPACING_M, Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.Equal(40.0, Result.LengthM, 2);
            AssertMaximumStep(Result.Points, SAMPLE_SPACING_M + 1e-6);
        }

        /// <summary>Verifies a coincident opposite-heading pose produces a complete keyhole-style turn.</summary>
        [Fact]
        public void Connect_CoincidentOppositeHeading_BuildsRadiusValidTurn
            (
            )
        {
            Grid Grid = MakeGrid(50, 30);
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(24.5, 12.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(24.5, 12.5, 270.0);

            ConnectorPlanner.Result Result = ConnectorPlanner.Connect(
                Start, End, 4.0, SAMPLE_SPACING_M, Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.True(Result.LengthM > 0.0);
            Assert.Equal((Start.X, Start.Y), Result.Points[0]);
            Assert.Equal((End.X, End.Y), Result.Points[Result.Points.Count - 1]);
            AssertMaximumStep(Result.Points, SAMPLE_SPACING_M + 1e-6);
        }

        /// <summary>
        /// Verifies consecutive samples stay within the requested interval.
        /// </summary>
        private static void AssertMaximumStep
            (
            List<(Double X, Double Y)> Points,
            Double MaximumStepM
            )
        {
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double Dx = Points[Index].X - Points[Index - 1].X;
                Double Dy = Points[Index].Y - Points[Index - 1].Y;
                Assert.True(Math.Sqrt((Dx * Dx) + (Dy * Dy)) <= MaximumStepM);
            }
        }

        /// <summary>
        /// Creates an all-valid raw field grid.
        /// </summary>
        private static Grid MakeGrid
            (
            Int32 Width,
            Int32 Height
            )
        {
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                Width,
                Height,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[Height, Width],
                new Double[Height, Width],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
