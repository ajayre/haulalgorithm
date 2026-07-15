using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies forward-only detours and explicit operator-alignment transit fallback.
    /// </summary>
    public sealed class TransitPlannerTests
    {
        /// <summary>
        /// Verifies an opposing endpoint heading is reached without a cusp or reverse maneuver.
        /// </summary>
        [Fact]
        public void Connect_OpposingEndpointHeading_UsesForwardOnlyCurve
            (
            )
        {
            Grid Grid = MakeGrid(30, 20, null);
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(5.5, 10.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(20.5, 10.5, 270.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
                Start, End, 5.0, 0.5, Grid, true, 30.0, 1.0, true);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.False(Result.OperatorAlignmentRequired);
            Assert.StartsWith("AUTO_", Result.Mode, StringComparison.Ordinal);
            Assert.Equal(0.0, Result.ManualAssistDistanceM, 9);
            Assert.Single(Result.Legs);
            Assert.False(Result.Legs[0].OperatorAlignmentRequired);
            Assert.True(Result.Points.Count > 2);
            Assert.True(MaximumInteriorDeflectionDeg(Result.Points) < 90.0);
            Assert.All(Result.Points,
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>
        /// Verifies a coincident grade turnaround near an edge takes a longer forward loop
        /// through open field space instead of using a three-point reversal.
        /// </summary>
        [Fact]
        public void Connect_CoincidentOpposingHeading_UsesForwardOnlyDetour
            (
            )
        {
            Grid Grid = MakeGrid(50, 30, null);
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(12.5, 1.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(12.5, 1.5, 270.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
                Start, End, 5.0, 0.25, Grid, true, 14.0, 1.0, true);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.False(Result.OperatorAlignmentRequired);
            Assert.Equal("AUTO_FORWARD_DETOUR", Result.Mode);
            Assert.True(Result.LengthM > 2.0 * Math.PI * 5.0);
            Assert.True(MaximumInteriorDeflectionDeg(Result.Points) < 90.0);
            Assert.True(MinimumObservedRadiusM(Result.Points) >= 5.0 * 0.99);
            Assert.Equal((Start.X, Start.Y), Result.Points[0]);
            Assert.Equal((End.X, End.Y), Result.Points[Result.Points.Count - 1]);
            Assert.All(Result.Points,
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>
        /// Verifies identical field-scoped connector queries reuse the immutable
        /// result instead of repeating geometric search.
        /// </summary>
        [Fact]
        public void Connect_IdenticalQuery_ReusesCachedResult
            (
            )
        {
            Grid Grid = MakeGrid(30, 20, null);
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(5.5, 10.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(20.5, 10.5, 90.0);

            TransitPlanner.Result First =
                TransitPlanner.Connect(
                    Start, End, 5.0, 0.5, Grid, true, 30.0, 1.0, true);
            TransitPlanner.Result Second =
                TransitPlanner.Connect(
                    Start, End, 5.0, 0.5, Grid, true, 30.0, 1.0, true);

            // A reference match proves the second call came from the field cache.
            Assert.Same(First, Second);
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

        /// <summary>Returns the smallest non-collinear circumradius in a polyline.</summary>
        private static Double MinimumObservedRadiusM
            (
            List<(Double X, Double Y)> Points
            )
        {
            Double MinimumM = Double.PositiveInfinity;
            for (Int32 Index = 1; Index + 1 < Points.Count; Index += 1)
            {
                Double A = Distance(Points[Index - 1], Points[Index]);
                Double B = Distance(Points[Index], Points[Index + 1]);
                Double C = Distance(Points[Index - 1], Points[Index + 1]);
                Double Cross = Math.Abs(
                    ((Points[Index].X - Points[Index - 1].X)
                        * (Points[Index + 1].Y - Points[Index - 1].Y))
                    - ((Points[Index].Y - Points[Index - 1].Y)
                        * (Points[Index + 1].X - Points[Index - 1].X)));
                if (Cross <= 1e-9) continue;
                MinimumM = Math.Min(MinimumM, (A * B * C) / (2.0 * Cross));
            }
            return MinimumM;
        }

        /// <summary>Returns Euclidean distance between two points.</summary>
        private static Double Distance
            (
            (Double X, Double Y) First,
            (Double X, Double Y) Second
            )
        {
            Double Dx = Second.X - First.X;
            Double Dy = Second.Y - First.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
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
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(5.5, 5.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(35.5, 5.5, 270.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
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
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(5.5, 8.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(30.5, 10.5, 85.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
                Start, End, 5.0, 0.25, Grid, true, 8.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.False(Result.OperatorAlignmentRequired);
            Assert.StartsWith("AUTO_", Result.Mode, StringComparison.Ordinal);
            Assert.Equal(0.0, Result.ManualAssistDistanceM, 9);
            Assert.All(Result.Points,
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>
        /// Verifies nearby aligned poses use the permitted short tracking correction
        /// instead of an exact-pose loop many times longer than their separation.
        /// </summary>
        [Fact]
        public void Connect_NearbyAlignedPoses_UseShortAlignmentJog
            (
            )
        {
            Grid Grid = MakeGrid(40, 30, null);
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(20.0, 15.0, 270.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(19.0, 15.23, 275.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
                Start, End, 3.4, 0.1, Grid, true, 8.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.False(Result.OperatorAlignmentRequired);
            Assert.Equal("AUTO_SHORT_ALIGNMENT_JOG", Result.Mode);
            Assert.InRange(Result.LengthM, 1.0, 1.1);
            Assert.Equal((Start.X, Start.Y), Result.Points[0]);
            Assert.Equal((End.X, End.Y), Result.Points[Result.Points.Count - 1]);
            Assert.All(Result.Points,
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>
        /// Sweeps more than three thousand nearby pose combinations and verifies the
        /// global connector quality gate never returns a disproportionate loop.
        /// </summary>
        [Fact]
        public void Connect_ShortAlignedPoseSweep_NeverReturnsGrossLoop
            (
            )
        {
            const Double RADIUS_M = 3.4;
            Grid Grid = MakeGrid(100, 100, null);
            Double[] DistancesM = { 0.6, 1.0, 1.4 };
            Double[] HeadingOffsetsDeg = { -14.0, -10.0, -5.0, 0.0, 5.0, 10.0, 14.0 };
            Int32 Cases = 0;
            for (Double TravelHeadingDeg = 0.0;
                TravelHeadingDeg < 360.0;
                TravelHeadingDeg += 15.0)
            {
                Double TravelRad = TravelHeadingDeg * Math.PI / 180.0;
                foreach (Double DistanceM in DistancesM)
                {
                    ConnectorPlanner.Pose StartPosition =
                        new ConnectorPlanner.Pose(50.0, 50.0, 0.0);
                    Double EndX = StartPosition.X + (Math.Sin(TravelRad) * DistanceM);
                    Double EndY = StartPosition.Y + (Math.Cos(TravelRad) * DistanceM);
                    foreach (Double StartOffsetDeg in HeadingOffsetsDeg)
                    {
                        foreach (Double EndOffsetDeg in HeadingOffsetsDeg)
                        {
                            ConnectorPlanner.Pose Start =
                                new ConnectorPlanner.Pose(
                                    StartPosition.X,
                                    StartPosition.Y,
                                    TravelHeadingDeg + StartOffsetDeg);
                            ConnectorPlanner.Pose End =
                                new ConnectorPlanner.Pose(
                                    EndX,
                                    EndY,
                                    TravelHeadingDeg + EndOffsetDeg);
                            TransitPlanner.Result Result =
                                TransitPlanner.Connect(
                                    Start, End, RADIUS_M, 0.1, Grid, true, 8.0);
                            Double MaximumReasonableM = Math.Max(
                                4.0 * DistanceM,
                                2.0 * RADIUS_M);
                            Assert.True(Result.Success, Result.FailureReason);
                            Assert.True(
                                Result.LengthM <= MaximumReasonableM + 1e-9,
                                String.Format(
                                    System.Globalization.CultureInfo.InvariantCulture,
                                    "heading={0:F0} start_offset={1:F0} end_offset={2:F0} distance={3:F1} mode={4} length={5:F2} limit={6:F2}",
                                    TravelHeadingDeg,
                                    StartOffsetDeg,
                                    EndOffsetDeg,
                                    DistanceM,
                                    Result.Mode,
                                    Result.LengthM,
                                    MaximumReasonableM));
                            Cases += 1;
                        }
                    }
                }
            }
            Assert.Equal(3528, Cases);
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
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(3.5, 3.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(20.5, 3.5, 90.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
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
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(3.5, 3.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(20.5, 3.5, 90.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
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
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(5.5, 5.5, 90.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(15.5, 5.5, 270.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
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
            ConnectorPlanner.Pose Start =
                new ConnectorPlanner.Pose(5.5, 0.5, 270.0);
            ConnectorPlanner.Pose End =
                new ConnectorPlanner.Pose(15.5, 0.5, 90.0);

            TransitPlanner.Result Result = TransitPlanner.Connect(
                Start, End, 0.75, 0.10, Grid, true, 2.0, 1.0);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.True(Result.LeavesField);
            Assert.True(Result.OperatorAlignmentRequired);
            Assert.Equal("EXTERIOR_MANUAL_CURVE", Result.Mode);
            Assert.Single(Result.Legs);
            Assert.True(Result.Legs[0].OperatorAlignmentRequired);
            FieldEdgePolicy Policy = new FieldEdgePolicy(Grid, 1.0);
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
