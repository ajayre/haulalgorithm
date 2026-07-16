using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    public sealed class HaulWorkLedgerTests
    {
        [Theory]
        [InlineData(0.0, 0.0)]
        [InlineData(180.0, 0.0)]
        [InlineData(270.0, 90.0)]
        [InlineData(-90.0, 90.0)]
        [InlineData(217.5, 37.5)]
        public void HeadingInputNormalizesToAnUndirectedAxis
            (
            Double InputDeg,
            Double ExpectedDeg
            )
        {
            Assert.Equal(
                ExpectedDeg,
                HaulLaneBuilder.NormalizeAxisHeadingDeg(InputDeg),
                12);
        }

        [Fact]
        public void ArbitraryAxisConservesLayersAndOppositeBearingIsIdentical
            (
            )
        {
            Grid Grid = MakeGrid();
            Grid.Existing[2, 2] = 0.140;
            Grid.Existing[3, 3] = 0.061;
            Grid.Existing[6, 6] = -0.061;
            Grid.Existing[7, 7] = -0.140;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);

            HaulLaneBuilder.Inventory First = HaulLaneBuilder.Build(
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                HeadingAxisDeg: 37.5);
            HaulLaneBuilder.Inventory Opposite = HaulLaneBuilder.Build(
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                HeadingAxisDeg: 217.5);

            Assert.Equal(LaneSignature(First), LaneSignature(Opposite));
            Assert.All(
                First.CutLanes.Concat(First.FillLanes),
                Lane => Assert.Equal(37.5, Lane.AxisHeadingDeg, 12));
            Assert.Equal(
                Ledger.RequiredCutDepthM.Cast<Double>().Sum(),
                First.CutLanes.SelectMany(Lane => Lane.Entries)
                    .Sum(Entry => Entry.DepthM),
                12);
            Assert.Equal(
                Ledger.RequiredFillDepthM.Cast<Double>().Sum(),
                First.FillLanes.SelectMany(Lane => Lane.Entries)
                    .Sum(Entry => Entry.DepthM),
                12);
        }

        [Fact]
        public void CardinalUserAxisReusesTheExistingLaneCover
            (
            )
        {
            Grid Grid = MakeGrid();
            Grid.Existing[2, 2] = 0.140;
            Grid.Existing[3, 2] = 0.061;
            Grid.Existing[6, 7] = -0.061;
            Grid.Existing[7, 7] = -0.140;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);

            HaulLaneBuilder.Inventory Existing = HaulLaneBuilder.Build(
                Ledger, HaulLaneBuilder.LaneAxis.NorthSouth);
            HaulLaneBuilder.Inventory User = HaulLaneBuilder.Build(
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                HeadingAxisDeg: 180.0);

            Assert.Equal(LaneSignature(Existing), LaneSignature(User));
        }

        [Fact]
        public void ArbitraryAxisCoversEveryLayerOnARectangularField
            (
            )
        {
            Grid Grid = MakeGrid(14, 8);
            Grid.Existing[1, 2] = 0.140;
            Grid.Existing[5, 11] = 0.061;
            Grid.Existing[2, 10] = -0.061;
            Grid.Existing[6, 3] = -0.140;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);

            HaulLaneBuilder.Inventory Inventory = HaulLaneBuilder.Build(
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                HeadingAxisDeg: 37.5);

            Assert.Equal(
                Ledger.RequiredCutDepthM.Cast<Double>().Sum(),
                Inventory.CutLanes.SelectMany(Lane => Lane.Entries)
                    .Sum(Entry => Entry.DepthM),
                12);
            Assert.Equal(
                Ledger.RequiredFillDepthM.Cast<Double>().Sum(),
                Inventory.FillLanes.SelectMany(Lane => Lane.Entries)
                    .Sum(Entry => Entry.DepthM),
                12);
            Assert.All(
                Inventory.CutLanes.Concat(Inventory.FillLanes),
                Lane => Assert.Equal(37.5, Lane.AxisHeadingDeg, 12));
        }

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

        [Fact]
        public void ResidualLedgerContainsOnlyUnassignedWork
            (
            )
        {
            Grid Grid = MakeGrid();
            Grid.Existing[2, 2] = 0.140;
            Grid.Existing[7, 7] = -0.140;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);
            Double AssignedDepthM = 0.061;
            Double LoadBcy = AssignedDepthM * Ledger.CutBcyPerDepthM;
            Pass Pass = Passes.BuildPass(
                Grid,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((2, 2), AssignedDepthM),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((7, 7), AssignedDepthM),
                },
                LoadBcy,
                Obstacles.EMPTY_OBSTACLES,
                true);

            HaulWorkLedger Residual = Ledger.BuildResidual(
                new List<Pass> { Pass });

            Assert.Equal(0.079, Residual.RequiredCutDepthM[2, 2], 12);
            Assert.Equal(0.079, Residual.RequiredFillDepthM[7, 7], 12);
            Assert.Equal(
                0.079 * Ledger.CutBcyPerDepthM,
                Residual.TotalCutBcy,
                12);
            Assert.Equal(
                0.079 * Ledger.FillBcyPerDepthM,
                Residual.TotalFillBcy,
                12);
        }

        private static Grid MakeGrid
            (
            ) => MakeGrid(10, 10);

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
                1.0,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        private static String LaneSignature
            (
            HaulLaneBuilder.Inventory Inventory
            ) => String.Join(
                "|",
                Inventory.CutLanes.Concat(Inventory.FillLanes)
                    .Select(Lane => String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "{0}:{1}:{2}:{3}",
                        Lane.Kind,
                        Lane.LayerIndex,
                        Lane.AxisHeadingDeg,
                        String.Join(",", Lane.Entries.Select(Entry =>
                            String.Format(
                                System.Globalization.CultureInfo.InvariantCulture,
                                "{0}/{1}/{2:R}",
                                Entry.Cell.Bx,
                                Entry.Cell.By,
                                Entry.DepthM))))));
    }
}
