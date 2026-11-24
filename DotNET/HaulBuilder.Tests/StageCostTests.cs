using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// The offer chooser still prefers the cheaper seconds per yard.
    /// The same field through cost selection repeats exactly.
    /// </summary>
    public sealed class StageCostTests
    {
        /// <summary>Bowl used by the production scraper, in bank cubic yards.</summary>
        private const Double CAPACITY_BCY = 20.0 / 1.3;

        /// <summary>
        /// A 6 yard flow trip at 120 seconds is 20 seconds per yard.
        /// A 12 yard cluster trip at 600 seconds is 50. The flow trip is taken.
        /// </summary>
        [Fact]
        public void FlowTripBeatsASlowerClusterTrip()
        {
            StageCost.Offer[] Offers = new[]
            {
                new StageCost.Offer(HaulStage.Flow, 120.0, 6.0),
                new StageCost.Offer(HaulStage.Clusters, 600.0, 12.0),
            };
            StageCost.Offer Choice = Assert.NotNull(StageCost.Choose(Offers, null));
            Assert.Equal(HaulStage.Flow, Choice.Stage);
            Assert.Equal(20.0, Choice.Seconds / Choice.PayloadBcy, 6);
        }

        /// <summary>
        /// The split field has cut, fill, and a little surplus. Cost selection
        /// hands the remainder to correcting, and the ledger accepts the result.
        /// </summary>
        [Fact]
        public void SplitFieldFinishesInCorrecting()
        {
            HaulOptimizationResult Result = RunSplitField(BaselineTrips: 1);
            Assert.Equal(HaulSearchState.Correcting, Result.State);
            Assert.True(
                Result.Validation.Success,
                String.Join(" | ", Result.Validation.Issues));
        }

        /// <summary>
        /// The same field twice yields the same stage and the same loads, in order.
        /// </summary>
        [Fact]
        public void SameFieldProducesTheSameTripSequence()
        {
            HaulOptimizationResult First = RunSplitField(BaselineTrips: 500);
            HaulOptimizationResult Second = RunSplitField(BaselineTrips: 500);
            Assert.Equal(First.PassResult.Passes.Count, Second.PassResult.Passes.Count);
            for (Int32 Index = 0; Index < First.PassResult.Passes.Count; Index += 1)
            {
                Pass Left = First.PassResult.Passes[Index];
                Pass Right = Second.PassResult.Passes[Index];
                Assert.Equal(Left.Stage, Right.Stage);
                Assert.Equal(Left.LoadBcy, Right.LoadBcy, 9);
                Assert.Equal(Left.CutCells.Count, Right.CutCells.Count);
                Assert.Equal(Left.CutCells[0].Cell, Right.CutCells[0].Cell);
            }
        }

        /// <summary>
        /// A near cut and a far shallow cell, so the second trip can miss the ceiling.
        /// </summary>
        /// <param name="BaselineTrips">Flow trips that fix the median.</param>
        /// <returns>The search result.</returns>
        private static HaulOptimizationResult RunSplitField
            (
            Int32 BaselineTrips
            )
        {
            Grid Grid = MakeGrid(8, 24);
            Double[,] Existing = Grid.Existing;
            for (Int32 Col = 1; Col <= 8; Col += 1)
            {
                Existing[2, Col] = 0.061;
            }
            for (Int32 Col = 10; Col <= 16; Col += 1)
            {
                Existing[2, Col] = -0.061;
            }
            // A shallow cut at the far side, filled back at the near side.
            // The haul is long and the payload is small, so the seconds per yard are high.
            Existing[6, 22] = 0.015;
            Existing[6, 1] = -0.040;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);
            return FlowRibbonSearch.Optimize(
                Grid,
                EastFlow(Grid),
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                CAPACITY_BCY,
                true,
                null,
                false,
                2.5908,
                45.0,
                1,
                0.5,
                0,
                null,
                BaselineTrips);
        }

        /// <summary>Builds an empty valid grid.</summary>
        /// <param name="Height">Rows.</param>
        /// <param name="Width">Columns.</param>
        /// <returns>The grid.</returns>
        private static Grid MakeGrid
            (
            Int32 Height,
            Int32 Width
            )
        {
            Double[,] Existing = new Double[Height, Width];
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                2.5908,
                Width,
                Height,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                Existing,
                new Double[Height, Width],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>A flow that points east on every cell.</summary>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The flow.</returns>
        private static FlowField EastFlow
            (
            Grid Grid
            )
        {
            Double[,] Phi = new Double[Grid.Height, Grid.Width];
            Double[,] Fx = new Double[Grid.Height, Grid.Width];
            Double[,] Fy = new Double[Grid.Height, Grid.Width];
            Double[,] Magnitude = new Double[Grid.Height, Grid.Width];
            Boolean[,] HasArrow = new Boolean[Grid.Height, Grid.Width];
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Fx[By, Bx] = 1.0;
                    Magnitude[By, Bx] = 1.0;
                    HasArrow[By, Bx] = true;
                }
            }
            return new FlowField(Phi, Fx, Fy, Magnitude, HasArrow, "test");
        }
    }
}
