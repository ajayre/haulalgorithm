using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies field-clamped lineups and deterministic production sequencing.
    /// </summary>
    public sealed class RunSequencerTests
    {
        /// <summary>
        /// Verifies every selected lineup and nominal transit remains inside the raw field.
        /// </summary>
        [Fact]
        public void Optimize_BuildsInFieldLineupsAndTransits
            (
            )
        {
            Grid Grid = MakeGrid();
            List<CoveragePlanner.Run> Runs = new List<CoveragePlanner.Run>
            {
                MakeRun((5.5, 5.5), (15.5, 5.5), 90.0, (5, 5)),
                MakeRun((15.5, 10.5), (5.5, 10.5), 270.0, (5, 10)),
            };

            RunSequencer.Result Result = RunSequencer.Optimize(
                Runs, null, null, 4.0, 5.0, Constants.HALF_SCALE, Grid);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.Equal(2, Result.Runs.Count);
            Assert.Single(Result.Connectors);
            Assert.All(Result.Runs, Run =>
            {
                Assert.True(Grid.ContainsWorldXy(Run.EntryPose.X, Run.EntryPose.Y));
                Assert.True(Grid.ContainsWorldXy(Run.ExitPose.X, Run.ExitPose.Y));
                Assert.InRange(Run.EntryLineupM, 0.0, 4.0);
                Assert.InRange(Run.ExitLineupM, 0.0, 4.0);
            });
            Assert.All(Result.Connectors.SelectMany(Connector => Connector.Points),
                Point => Assert.True(Grid.ContainsWorldXy(Point.X, Point.Y)));
        }

        /// <summary>
        /// Verifies larger run groups use a deterministic validated sequence without subset explosion.
        /// </summary>
        [Fact]
        public void Optimize_LargeRunGroup_IsDeterministicAndComplete
            (
            )
        {
            Grid Grid = MakeGrid();
            List<CoveragePlanner.Run> Runs = new List<CoveragePlanner.Run>();
            for (Int32 Index = 0; Index < 8; Index += 1)
            {
                Runs.Add(MakeRun(
                    (4.5, 3.5 + (2.0 * Index)),
                    (18.5, 3.5 + (2.0 * Index)),
                    90.0,
                    (4, 3 + (2 * Index))));
            }

            RunSequencer.Result First = RunSequencer.Optimize(
                Runs, null, null, 2.0, 3.0, Constants.HALF_SCALE, Grid);
            RunSequencer.Result Second = RunSequencer.Optimize(
                Runs, null, null, 2.0, 3.0, Constants.HALF_SCALE, Grid);

            Assert.True(First.Success, First.FailureReason);
            Assert.Equal(8, First.Runs.Count);
            Assert.Equal(7, First.Connectors.Count);
            Assert.Equal(
                First.Runs.Select(Run => (Run.Run.StartXy, Run.Run.EndXy)),
                Second.Runs.Select(Run => (Run.Run.StartXy, Run.Run.EndXy)));
        }

        /// <summary>
        /// Verifies flexible endpoint sequencing remains bounded for a run group
        /// larger than the exact subset-search limit.
        /// </summary>
        [Fact]
        public void OptimizeFlexibleEndpoints_LargeRunGroup_IsDeterministicAndComplete
            (
            )
        {
            Grid Grid = MakeGrid();
            List<CoveragePlanner.Run> Runs =
                new List<CoveragePlanner.Run>();
            for (Int32 Index = 0; Index < 12; Index += 1)
            {
                Runs.Add(MakeRun(
                    (4.5, 2.5 + (1.5 * Index)),
                    (18.5, 2.5 + (1.5 * Index)),
                    90.0,
                    (4, 2 + Index)));
            }

            RunSequencer.Result First =
                RunSequencer.OptimizeFlexibleEndpoints(
                    Runs, null, null, 2.0, 3.0, Constants.HALF_SCALE, Grid);
            RunSequencer.Result Second =
                RunSequencer.OptimizeFlexibleEndpoints(
                    Runs, null, null, 2.0, 3.0, Constants.HALF_SCALE, Grid);

            Assert.True(First.Success, First.FailureReason);
            Assert.Equal(12, First.Runs.Count);
            Assert.Equal(11, First.Connectors.Count);
            Assert.Equal(
                First.Runs.Select(Run => (Run.Run.StartXy, Run.Run.EndXy)),
                Second.Runs.Select(Run => (Run.Run.StartXy, Run.Run.EndXy)));
        }

        /// <summary>
        /// Verifies a loaded inbound path starts the overlapping long fill instead
        /// of crossing it blade-up to reach a small interior fragment first.
        /// </summary>
        [Fact]
        public void Optimize_LoadedApproach_DoesNotDeadheadAcrossUnfilledRun
            (
            )
        {
            Grid Grid = MakeGrid();
            CoveragePlanner.Run LongRun = MakeRun(
                (5.5, 10.5), (18.5, 10.5), 90.0, (5, 10));
            CoveragePlanner.Run InteriorFragment = MakeRun(
                (14.5, 11.0), (12.5, 11.0), 270.0, (12, 11));
            ConnectorPlanner.Pose LoadedInbound =
                new ConnectorPlanner.Pose(22.5, 10.5, 270.0);

            RunSequencer.Result Result =
                RunSequencer.Optimize(
                    new List<CoveragePlanner.Run>
                    {
                        InteriorFragment,
                        LongRun,
                    },
                    LoadedInbound,
                    null,
                    4.0,
                    4.0,
                    Constants.HALF_SCALE,
                    Grid,
                    AvoidLoadedRunOverlap: true);

            Assert.True(Result.Success, Result.FailureReason);
            Assert.Equal((5, 10), Result.Runs[0].Run.Cells[0].Cell);
            Assert.True(Result.Runs[0].Run.StartXy.X
                > Result.Runs[0].Run.EndXy.X);
            Assert.InRange(Result.Runs[0].Run.HeadingDeg, 269.999, 270.001);
        }

        /// <summary>
        /// Verifies a boundary-facing work run ends early enough to begin a real in-field turn.
        /// </summary>
        [Fact]
        public void Prepare_FieldEdge_RetreatsSoftWorkEndpointForTurnClearance
            (
            )
        {
            Grid Grid = MakeGrid();
            CoveragePlanner.Run Run = MakeRun(
                (5.5, 12.5), (24.5, 12.5), 90.0, (24, 12));

            RunSequencer.SequencedRun Prepared =
                RunSequencer.Prepare(
                    Run, 0.0, 4.0, Constants.HALF_SCALE, Grid);

            Assert.Equal(Run.StartXy, Prepared.Run.StartXy);
            Assert.True(Prepared.Run.EndXy.X < Run.EndXy.X - 3.0);
            Assert.Equal(Run.EndXy.Y, Prepared.Run.EndXy.Y, 9);
            Assert.Same(Run.Cells, Prepared.Run.Cells);
            Assert.True(Grid.ContainsWorldXy(
                Prepared.ExitPose.X, Prepared.ExitPose.Y));
        }

        /// <summary>Creates one minimal allocated straight run.</summary>
        private static CoveragePlanner.Run MakeRun
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double HeadingDeg,
            (Int32 Bx, Int32 By) Cell
            )
        {
            return new CoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { (Cell, 0.02) },
                Start,
                End,
                HeadingDeg,
                0.0);
        }

        /// <summary>Creates an all-valid raw field.</summary>
        private static Grid MakeGrid
            (
            )
        {
            const Int32 SIZE = 25;
            Boolean[,] Valid = new Boolean[SIZE, SIZE];
            for (Int32 By = 0; By < SIZE; By += 1)
            {
                for (Int32 Bx = 0; Bx < SIZE; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                SIZE,
                SIZE,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[SIZE, SIZE],
                new Double[SIZE, SIZE],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
