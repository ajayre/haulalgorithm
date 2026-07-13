using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies field-clamped lineups and deterministic experimental sequencing.
    /// </summary>
    public sealed class ExperimentalRunSequencerTests
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
            List<ExperimentalCoveragePlanner.Run> Runs = new List<ExperimentalCoveragePlanner.Run>
            {
                MakeRun((5.5, 5.5), (15.5, 5.5), 90.0, (5, 5)),
                MakeRun((15.5, 10.5), (5.5, 10.5), 270.0, (5, 10)),
            };

            ExperimentalRunSequencer.Result Result = ExperimentalRunSequencer.Optimize(
                Runs, null, null, 4.0, 5.0, 0.5, Grid);

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
            List<ExperimentalCoveragePlanner.Run> Runs = new List<ExperimentalCoveragePlanner.Run>();
            for (Int32 Index = 0; Index < 8; Index += 1)
            {
                Runs.Add(MakeRun(
                    (4.5, 3.5 + (2.0 * Index)),
                    (18.5, 3.5 + (2.0 * Index)),
                    90.0,
                    (4, 3 + (2 * Index))));
            }

            ExperimentalRunSequencer.Result First = ExperimentalRunSequencer.Optimize(
                Runs, null, null, 2.0, 3.0, 0.5, Grid);
            ExperimentalRunSequencer.Result Second = ExperimentalRunSequencer.Optimize(
                Runs, null, null, 2.0, 3.0, 0.5, Grid);

            Assert.True(First.Success, First.FailureReason);
            Assert.Equal(8, First.Runs.Count);
            Assert.Equal(7, First.Connectors.Count);
            Assert.Equal(
                First.Runs.Select(Run => (Run.Run.StartXy, Run.Run.EndXy)),
                Second.Runs.Select(Run => (Run.Run.StartXy, Run.Run.EndXy)));
        }

        /// <summary>Creates one minimal allocated straight run.</summary>
        private static ExperimentalCoveragePlanner.Run MakeRun
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double HeadingDeg,
            (Int32 Bx, Int32 By) Cell
            )
        {
            return new ExperimentalCoveragePlanner.Run(
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
