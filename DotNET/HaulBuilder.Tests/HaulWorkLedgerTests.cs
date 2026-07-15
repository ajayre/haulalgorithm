using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    public sealed class HaulWorkLedgerTests
    {
        [Fact]
        public void LayersRecombineToTheExactRequiredDepth
            (
            )
        {
            Grid Grid = MakeGrid();
            Grid.Existing[2, 2] = 0.140;
            Grid.Existing[7, 7] = -0.140;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);

            HaulLaneBuilder.Inventory Inventory = HaulLaneBuilder.Build(
                Ledger, HaulLaneBuilder.LaneAxis.EastWest);

            Assert.Equal(3, Inventory.CutLanes.Count);
            Assert.Equal(3, Inventory.FillLanes.Count);
            Assert.Equal(
                0.140,
                Inventory.CutLanes.SelectMany(Lane => Lane.Entries)
                    .Sum(Entry => Entry.DepthM),
                12);
            Assert.All(
                Inventory.CutLanes.SelectMany(Lane => Lane.Entries),
                Entry => Assert.InRange(Entry.DepthM, 0.0, 0.061000000001));
        }

        [Fact]
        public void DeferredLedgerExactlyReconcilesUnassignedLayers
            (
            )
        {
            Grid Grid = MakeGrid();
            Grid.Existing[2, 2] = 0.140;
            Grid.Existing[7, 7] = -0.140;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);
            Double LoadBcy = 0.061 * Ledger.CutBcyPerDepthM;
            Pass Pass = Passes.BuildPass(
                Grid,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((2, 2), 0.061),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((7, 7), 0.061),
                },
                LoadBcy,
                Obstacles.EMPTY_OBSTACLES,
                true);

            IReadOnlyList<HaulDeferredWork> Deferred =
                Ledger.BuildDeferredWork(
                    new List<Pass> { Pass }, "TEST_DEFERRED");

            Assert.Equal(4, Deferred.Count);
            Assert.Equal(
                0.079,
                Deferred.Where(Entry =>
                        Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
                    .Sum(Entry => Entry.DepthM),
                12);
            Assert.Equal(
                0.079,
                Deferred.Where(Entry =>
                        Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
                    .Sum(Entry => Entry.DepthM),
                12);
            Assert.All(Deferred, Entry => Assert.Equal("TEST_DEFERRED", Entry.Reason));
        }

        private static Grid MakeGrid
            (
            )
        {
            const Int32 Size = 10;
            Boolean[,] Valid = new Boolean[Size, Size];
            for (Int32 By = 0; By < Size; By += 1)
            {
                for (Int32 Bx = 0; Bx < Size; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                Size,
                Size,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[Size, Size],
                new Double[Size, Size],
                Valid,
                1.3,
                1.0,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
