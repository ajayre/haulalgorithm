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
        /// Verifies an opposing endpoint heading becomes an explicit bounded manual keyhole.
        /// </summary>
        [Fact]
        public void Connect_OpposingEndpointHeading_UsesBoundedManualKeyhole
            (
            )
        {
            Grid Grid = MakeGrid(30, 10, null);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(5.5, 5.5, 90.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(20.5, 5.5, 270.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 5.0, 0.5, Grid, true, 30.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.True(Result.OperatorAlignmentRequired);
            Assert.Equal("MANUAL_KEYHOLE", Result.Mode);
            Assert.InRange(Result.ManualAssistDistanceM, 0.0, 30.0);
            Assert.Single(Result.Legs);
            Assert.True(Result.Legs[0].OperatorAlignmentRequired);
            Assert.True(Result.Points.Count > 2);
            Assert.All(Result.Points,
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>
        /// Verifies a coincident grade reversal is a visible equipment-scale three-point
        /// maneuver rather than a sub-cell out-and-back with an instantaneous heading flip.
        /// </summary>
        [Fact]
        public void Connect_CoincidentOpposingHeading_UsesVisibleThreePointKeyhole
            (
            )
        {
            Grid Grid = MakeGrid(40, 4, null);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(12.5, 1.5, 90.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(12.5, 1.5, 270.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 5.0, 0.25, Grid, true, 14.0);

            Double MinimumPullM = 0.5
                * Constants.DEFAULT_EQUIPMENT_LENGTH_FT
                * Constants.FEET_TO_METERS;
            Double MaximumExcursionM = Result.Points.Max(Point =>
            {
                Double Dx = Point.X - Start.X;
                Double Dy = Point.Y - Start.Y;
                return Math.Sqrt((Dx * Dx) + (Dy * Dy));
            });
            Double LateralSpanM = Result.Points.Max(Point => Point.Y)
                - Result.Points.Min(Point => Point.Y);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.True(Result.OperatorAlignmentRequired);
            Assert.Equal("MANUAL_THREE_POINT_KEYHOLE", Result.Mode);
            Assert.InRange(Result.LengthM, 13.0, 14.0);
            Assert.True(MaximumExcursionM >= MinimumPullM - 1e-9);
            Assert.True(LateralSpanM >= 1.0);
            Assert.All(Result.Points,
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>
        /// Verifies one aligned endpoint does not excuse an opposing heading at the other endpoint.
        /// </summary>
        [Fact]
        public void Connect_AlignedStartWithOpposingEnd_FailsSmallCorrectionLimit
            (
            )
        {
            Grid Grid = MakeGrid(50, 10, null);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(5.5, 5.5, 90.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(35.5, 5.5, 270.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 5.0, 0.5, Grid, true, 20.0);

            Assert.False(Result.Success);
            Assert.Contains("heading correction", Result.FailureReason, StringComparison.OrdinalIgnoreCase);
            Assert.Empty(Result.Legs);
        }

        /// <summary>
        /// Verifies a small heading correction is emitted as a smooth automatic bend.
        /// </summary>
        [Fact]
        public void Connect_SlightHeadingChange_UsesAutomaticBend
            (
            )
        {
            Grid Grid = MakeGrid(40, 20, null);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(5.5, 8.5, 90.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(30.5, 10.5, 85.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 5.0, 0.25, Grid, true, 8.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.False(Result.OperatorAlignmentRequired);
            Assert.StartsWith("AUTO_", Result.Mode, StringComparison.Ordinal);
            Assert.Equal(0.0, Result.ManualAssistDistanceM, 9);
            Assert.All(Result.Points,
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
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

        /// <summary>
        /// Verifies a shallow obstacle detour is rounded and remains automatic.
        /// </summary>
        [Fact]
        public void Connect_ShallowRoutedBend_DoesNotRequireManualAssist
            (
            )
        {
            List<(Int32 Bx, Int32 By)> Invalid = new List<(Int32 Bx, Int32 By)>();
            for (Int32 By = 0; By < 10; By += 1)
            {
                if ((By < 3) || (By > 5)) Invalid.Add((10, By));
            }
            Grid Grid = MakeGrid(25, 10, Invalid);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(3.5, 3.5, 90.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(20.5, 3.5, 90.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 3.0, 0.25, Grid, true, 8.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.False(Result.OperatorAlignmentRequired);
            Assert.StartsWith("AUTO_", Result.Mode, StringComparison.Ordinal);
            Assert.Equal(0.0, Result.ManualAssistDistanceM, 9);
            Assert.All(Result.Points,
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
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
        /// Verifies an edge turnaround may use no more than one cell of outer-field space and is wholly manual.
        /// </summary>
        [Fact]
        public void Connect_EdgeTurn_UsesOneCellExteriorAllowanceAndFlagsManual
            (
            )
        {
            List<(Int32 Bx, Int32 By)> Invalid = new List<(Int32 Bx, Int32 By)>();
            for (Int32 By = 1; By < 5; By += 1)
            {
                for (Int32 Bx = 0; Bx < 30; Bx += 1)
                {
                    Invalid.Add((Bx, By));
                }
            }
            Grid Grid = MakeGrid(30, 5, Invalid);
            ExperimentalConnectorPlanner.Pose Start =
                new ExperimentalConnectorPlanner.Pose(5.5, 0.5, 270.0);
            ExperimentalConnectorPlanner.Pose End =
                new ExperimentalConnectorPlanner.Pose(15.5, 0.5, 90.0);

            ExperimentalTransitPlanner.Result Result = ExperimentalTransitPlanner.Connect(
                Start, End, 0.75, 0.10, Grid, true, 2.0, 1.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.True(Result.LeavesField);
            Assert.True(Result.OperatorAlignmentRequired);
            Assert.Equal("EXTERIOR_MANUAL_CURVE", Result.Mode);
            Assert.Single(Result.Legs);
            Assert.True(Result.Legs[0].OperatorAlignmentRequired);
            ExperimentalFieldEdgePolicy Policy = new ExperimentalFieldEdgePolicy(Grid, 1.0);
            Assert.Contains(Result.Points,
                Point => Policy.IsPermittedExterior(Point.X, Point.Y));
            Assert.All(Result.Points, Point => Assert.True(
                Policy.ContainsWithExteriorAllowance(Point.X, Point.Y)));
            Assert.All(Result.Points.Where(Point => !Grid.ContainsWorldXy(Point.X, Point.Y)),
                Point => Assert.True(
                    Policy.DistanceToNearestValidCellM(Point.X, Point.Y)
                    <= Grid.GridSizeM + 1e-9));
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
