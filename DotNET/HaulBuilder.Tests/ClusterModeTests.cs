using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Checks the hauling, clusters, thin-cut, and correcting transitions.
    /// </summary>
    public sealed class ClusterModeTests
    {
        /// <summary>Bowl used by the production scraper, in bank cubic yards.</summary>
        private const Double CAPACITY_BCY = 20.0 / 1.3;

        /// <summary>
        /// Fifty millimetres is haulable. Cost selection takes what it can,
        /// then correcting runs. The ledger accepts the plan.
        /// </summary>
        [Fact]
        public void FieldOnTheBandMovesToCorrecting()
        {
            HaulOptimizationResult Result = Run(0.050);
            Assert.Equal(HaulSearchState.Correcting, Result.State);
            Assert.True(Result.EnteredClusterMode);
            Assert.True(Result.CorrectingStartsAtRoute > 0);
            Assert.True(
                Result.Validation.Success,
                String.Join(" | ", Result.Validation.Issues));
        }

        /// <summary>
        /// Eighty millimetres is hauled by cost, then correcting runs.
        /// The search stops at correcting.
        /// </summary>
        [Fact]
        public void FieldPastTheBandWithNoTripMovesToCorrecting()
        {
            HaulOptimizationResult Result = Run(0.080);
            Assert.Equal(HaulSearchState.Correcting, Result.State);
            Assert.True(Result.EnteredClusterMode);
            Assert.True(Result.CorrectingStartsAtRoute > 0);
            Assert.True(
                Result.Validation.Success,
                String.Join(" | ", Result.Validation.Issues));
        }

        /// <summary>
        /// Runs the ribbon on two cut cells and two fill cells of one depth.
        /// The window is on, so the trip is chosen by seconds per yard.
        /// </summary>
        /// <param name="DepthM">Cut and fill depth in metres.</param>
        /// <returns>The search result.</returns>
        private static HaulOptimizationResult Run
            (
            Double DepthM
            )
        {
            Grid Grid = MakeGrid(8, 8);
            Double[,] Existing = Grid.Existing;
            Existing[3, 1] = DepthM;
            Existing[3, 2] = DepthM;
            Existing[3, 5] = -DepthM;
            Existing[3, 6] = -DepthM;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);
            FlowField Flow = EastFlow(Grid);
            return FlowRibbonSearch.Optimize(
                Grid,
                Flow,
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                CAPACITY_BCY,
                true,
                null,
                false,
                2.5908,
                45.0,
                1,
                1.0);
        }

        /// <summary>Builds an empty valid grid.</summary>
        /// <param name="Width">Columns.</param>
        /// <param name="Height">Rows.</param>
        /// <returns>The grid.</returns>
        private static Grid MakeGrid
            (
            Int32 Width,
            Int32 Height
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
