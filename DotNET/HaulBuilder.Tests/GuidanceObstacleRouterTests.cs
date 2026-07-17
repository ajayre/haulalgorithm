using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;
using static HaulBuilder.Obstacles;

namespace HaulBuilder.Tests
{
    /// <summary>Verifies production guidance never silently crosses an obstruction.</summary>
    public sealed class GuidanceObstacleRouterTests
    {
        /// <summary>A small obstruction on a very long leg cannot fall between samples.</summary>
        [Fact]
        public void Blocks_DetectsSmallPolygonOnLongSegment()
        {
            ExclusionZone Zone = ExclusionZone.FromRing(new[]
            {
                (49.9, -0.1), (50.1, -0.1),
                (50.1, 0.1), (49.9, 0.1),
            }, 0.0);

            Assert.True(Zone.Blocks((0.0, 0.0), (100.0, 0.0)));
        }

        /// <summary>Every automatically labelled output leg is clear of the buffer.</summary>
        [Fact]
        public void Apply_LeavesNoBlockedAutomaticLeg()
        {
            Grid Grid = MakeGrid(100, 100);
            ObstacleField Field = ObstacleField.FromRings(new[]
            {
                (IReadOnlyList<(Double X, Double Y)>)new List<(Double X, Double Y)>
                {
                    (45.0, 45.0), (55.0, 45.0),
                    (55.0, 55.0), (45.0, 55.0),
                },
            }, 2.0);
            LifecyclePlanner.Result Source = MakeLifecycle(
                (10.0, 50.0), (90.0, 50.0));

            GuidanceObstacleRouter.Result Routed =
                GuidanceObstacleRouter.Apply(Source, Grid, 5.0, Field);

            Assert.True(Routed.AddedDistanceM > 0.0);
            foreach (LifecyclePlanner.Segment Segment in Routed.Lifecycle.Segments
                .Where(Segment => Segment.Phase
                    != LifecyclePlanner.Phase.ManualAssist))
            {
                for (Int32 Index = 1; Index < Segment.Points.Count; Index += 1)
                {
                    Assert.False(Field.Blocked(
                        Segment.Points[Index - 1], Segment.Points[Index]));
                }
            }
        }

        /// <summary>An endpoint under an obstruction is explicitly operator-controlled.</summary>
        [Fact]
        public void Apply_MarksUncertifiableMoveManual()
        {
            Grid Grid = MakeGrid(100, 100);
            ObstacleField Field = ObstacleField.FromRings(new[]
            {
                (IReadOnlyList<(Double X, Double Y)>)new List<(Double X, Double Y)>
                {
                    (45.0, 45.0), (55.0, 45.0),
                    (55.0, 55.0), (45.0, 55.0),
                },
            }, 2.0);
            LifecyclePlanner.Result Source = MakeLifecycle(
                (50.0, 50.0), (90.0, 50.0));

            GuidanceObstacleRouter.Result Routed =
                GuidanceObstacleRouter.Apply(Source, Grid, 5.0, Field);

            Assert.True(Routed.ManualMoveCount > 0);
            Assert.Contains(Routed.Lifecycle.Segments, Segment =>
                Segment.Phase == LifecyclePlanner.Phase.ManualAssist);
        }

        /// <summary>Per-route arm fields are used in preference to the static field.</summary>
        [Fact]
        public void Apply_UsesPerRoutePivotArmObstacle()
        {
            Grid Grid = MakeGrid(100, 100);
            ObstacleField Arm = ObstacleField.FromRings(new[]
            {
                (IReadOnlyList<(Double X, Double Y)>)new List<(Double X, Double Y)>
                {
                    (49.0, 20.0), (51.0, 20.0),
                    (51.0, 80.0), (49.0, 80.0),
                },
            }, 2.0);
            LifecyclePlanner.Result Source = MakeLifecycle(
                (10.0, 50.0), (90.0, 50.0));

            GuidanceObstacleRouter.Result Routed =
                GuidanceObstacleRouter.Apply(
                    Source,
                    Grid,
                    5.0,
                    EMPTY_OBSTACLES,
                    new Dictionary<Int32, ObstacleField> { { 0, Arm } });

            Assert.Equal(1, Routed.PivotDetourCount);
            Assert.True(Routed.AddedDistanceM > 0.0);
        }

        /// <summary>Manual obstruction moves have an unambiguous exported action.</summary>
        [Fact]
        public void GuidanceExport_LabelsManualAssistAsManualMove()
        {
            LifecyclePlanner.Segment Segment = new LifecyclePlanner.Segment(
                LifecyclePlanner.Phase.ManualAssist,
                false,
                new List<(Double X, Double Y)> { (0.0, 0.0), (1.0, 0.0) });
            System.Reflection.MethodInfo? Method = typeof(GuidancePlanner)
                .GetMethod(
                    "GuidanceAction",
                    System.Reflection.BindingFlags.NonPublic
                        | System.Reflection.BindingFlags.Static);

            Assert.NotNull(Method);
            Assert.Equal("MANUAL_MOVE", Method!.Invoke(null, new Object[] { Segment }));
        }

        /// <summary>Pivot moves are preferred over avoidable scraper conflicts.</summary>
        [Fact]
        public void PlanPivot_PrioritizesClearGuidanceOverMoveCost()
        {
            const Int32 GRID_SIZE = 101;
            Double[,] Existing = new Double[GRID_SIZE, GRID_SIZE];
            Double[,] Target = new Double[GRID_SIZE, GRID_SIZE];
            Boolean[,] Valid = new Boolean[GRID_SIZE, GRID_SIZE];
            for (Int32 By = 0; By < GRID_SIZE; By += 1)
            {
                for (Int32 Bx = 0; Bx < GRID_SIZE; Bx += 1) Valid[By, Bx] = true;
            }
            Pivot.Grid Grid = new Pivot.Grid(
                1.0, 0.0, 0.0, 12, true, Existing, Target, Valid);
            Pivot.PivotGeometry Geometry = new Pivot.PivotGeometry(
                50.5, 50.5, 40.0, 2.0, 2.0, 12, true);
            Pivot.Pass First = MakeAngularPass(90.0);
            Pivot.Pass Second = MakeAngularPass(270.0);

            Pivot.PivotPlan Plan = Pivot.PlanPivot(
                Grid, new[] { First, Second }, Geometry, 72, 1_000_000.0);

            Assert.Empty(Plan.ConflictPasses);
            Assert.True(Plan.MoveCount >= 1);
        }

        /// <summary>
        /// Builds angular pass.
        /// </summary>
        /// <param name="OpenBearingDeg">Direction in degrees.</param>
        /// <returns>The resulting angular pass.</returns>

        private static Pivot.Pass MakeAngularPass(Double OpenBearingDeg)
        {
            List<Pivot.PassCell> Cells = new List<Pivot.PassCell>();
            for (Int32 Bearing = 0; Bearing < 360; Bearing += 30)
            {
                if (Math.Abs(Bearing - OpenBearingDeg) < Constants.NUMERIC_COMPARISON_TOLERANCE) continue;
                Double Radians = Bearing * Math.PI / Constants.DEGREES_PER_HALF_TURN;
                Double X = 50.5 + (20.0 * Math.Sin(Radians));
                Double Y = 50.5 + (20.0 * Math.Cos(Radians));
                Cells.Add(new Pivot.PassCell(
                    (Int32)Math.Floor(X), (Int32)Math.Floor(Y), 0.05));
            }
            return new Pivot.Pass(
                Cells,
                Array.Empty<Pivot.PassCell>(),
                (50.5, 70.5),
                (50.5, 70.5));
        }

        /// <summary>
        /// Builds lifecycle.
        /// </summary>
        /// <param name="Start">The start used by the operation.</param>
        /// <param name="End">The end used by the operation.</param>
        /// <returns>The resulting lifecycle.</returns>

        private static LifecyclePlanner.Result MakeLifecycle(
            (Double X, Double Y) Start,
            (Double X, Double Y) End)
        {
            LifecyclePlanner.Segment Segment = new LifecyclePlanner.Segment(
                LifecyclePlanner.Phase.Transit,
                false,
                new List<(Double X, Double Y)> { Start, End },
                true);
            return new LifecyclePlanner.Result(
                true,
                new List<LifecyclePlanner.Segment> { Segment },
                0,
                Distance(Start, End),
                String.Empty);
        }

        /// <summary>
        /// Builds grid.
        /// </summary>
        /// <param name="Width">The width used by the operation.</param>
        /// <param name="Height">The height used by the operation.</param>
        /// <returns>The resulting grid.</returns>

        private static Grid MakeGrid(Int32 Width, Int32 Height)
        {
            Double[,] Existing = new Double[Height, Width];
            Double[,] Target = new Double[Height, Width];
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1) Valid[By, Bx] = true;
            }
            return new Grid(
                1.0, Width, Height, 0.0, 0.0, 12, true,
                0.0, 0.0, Existing, Target, Valid,
                1.3, 1.2, 61.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>
        /// Verifies distance.
        /// </summary>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <returns>The resulting distance.</returns>

        private static Double Distance(
            (Double X, Double Y) A,
            (Double X, Double Y) B)
        {
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }
    }
}
