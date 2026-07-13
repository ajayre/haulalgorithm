using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies loop rejection and explicit operator-alignment transit fallback.
    /// </summary>
    public sealed class ExperimentalTransitPlannerTests
    {
        /// <summary>
        /// Verifies opposing nearby headings produce a short explicit assist instead of a circle.
        /// </summary>
        [Fact]
        public void Connect_RejectsLoopAndMarksOperatorAlignment
            (
            )
        {
            Grid Grid = MakeGrid(30, 10, null);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(5.5, 5.5, 90.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(20.5, 5.5, 270.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 5.0, 0.5, Grid, true, 20.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.True(Result.OperatorAlignmentRequired);
            Assert.Equal("OPERATOR_ALIGN", Result.Mode);
            Assert.Equal(15.0, Result.LengthM, 6);
            Assert.Equal(15.0, Result.ManualAssistDistanceM, 6);
            Assert.Single(Result.Legs);
            Assert.True(Result.Legs[0].OperatorAlignmentRequired);
            Assert.All(Result.Points, Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>
        /// Verifies a blocked chord is routed through valid cells and remains explicitly assisted.
        /// </summary>
        [Fact]
        public void Connect_RoutesAroundInvalidCellsWithoutLeavingField
            (
            )
        {
            List<(Int32 Bx, Int32 By)> Invalid = new List<(Int32 Bx, Int32 By)>();
            for (Int32 By = 0; By < 8; By += 1)
            {
                if (By != 6) Invalid.Add((10, By));
            }
            Grid Grid = MakeGrid(25, 10, Invalid);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(3.5, 3.5, 90.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(20.5, 3.5, 90.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 3.0, 0.25, Grid, true, 8.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.True(Result.OperatorAlignmentRequired);
            Assert.Equal("OPERATOR_ALIGN_ROUTED_HYBRID", Result.Mode);
            Assert.True(Result.LengthM > 17.0);
            Assert.InRange(Result.ManualAssistDistanceM, 0.0, Result.LengthM);
            Assert.Contains(Result.Legs, Leg => Leg.OperatorAlignmentRequired);
            Assert.Contains(Result.Legs, Leg => !Leg.OperatorAlignmentRequired);
            Assert.All(Result.Legs.Where(Leg => Leg.OperatorAlignmentRequired), Leg =>
                Assert.True(PolylineLength(Leg.Points) <= 8.0 + 1e-9));
            Assert.All(Result.Points, Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>Computes polyline length for manual-assist assertions.</summary>
        private static Double PolylineLength
            (
            List<(Double X, Double Y)> Points
            )
        {
            Double TotalM = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double Dx = Points[Index].X - Points[Index - 1].X;
                Double Dy = Points[Index].Y - Points[Index - 1].Y;
                TotalM += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            return TotalM;
        }

        /// <summary>
        /// Verifies a positional fallback fails rather than exceeding its manual-assist limit.
        /// </summary>
        [Fact]
        public void Connect_ManualAssistBeyondLimit_FailsExplicitly
            (
            )
        {
            Grid Grid = MakeGrid(30, 10, null);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(5.5, 5.5, 90.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(15.5, 5.5, 270.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 5.0, 0.5, Grid, true, 8.0);

            Assert.False(Result.Success);
            Assert.Contains("exceeds", Result.FailureReason, StringComparison.OrdinalIgnoreCase);
            Assert.Empty(Result.Legs);
        }

        /// <summary>
        /// Creates a raw field with optional invalid cells.
        /// </summary>
        private static Grid MakeGrid
            (
            Int32 Width,
            Int32 Height,
            List<(Int32 Bx, Int32 By)>? Invalid
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
            if (Invalid != null)
            {
                foreach ((Int32 Bx, Int32 By) Cell in Invalid)
                {
                    Valid[Cell.By, Cell.Bx] = false;
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
