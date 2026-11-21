using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// The cluster-hauling stop. No haulable work enters thin-cut.
    /// A planner that keeps returning the same trip stops at 500 and does not.
    /// </summary>
    public sealed class ClusterHaulingTests
    {
        /// <summary>Yards on each synthetic trip.</summary>
        private const Double TRIP_BCY = 10.0;

        /// <summary>
        /// Twenty-five trips are driven ten at a time. The next plan is empty,
        /// so the stop is no haulable work after those 25 trips.
        /// </summary>
        [Fact]
        public void TwentyFiveTripsReplanToNoHaulableWork()
        {
            ClusterHaulStop Stop = DriveCountdown(25);
            Assert.Equal(ClusterHauling.NO_HAULABLE_WORK, Stop.Reason);
            Assert.Equal(25, Stop.TripsDone);
            Assert.Equal(HaulSearchState.ThinCut, ClusterHauling.Handoff(Stop));
        }

        /// <summary>
        /// A field whose plans add up to 107 trips runs all 107.
        /// The stop is no haulable work, not the trip limit.
        /// </summary>
        [Fact]
        public void OneHundredSevenTripsEndWithNoHaulableWork()
        {
            ClusterHaulStop Stop = DriveCountdown(107);
            Assert.Equal(ClusterHauling.NO_HAULABLE_WORK, Stop.Reason);
            Assert.NotEqual(ClusterHauling.TRIP_LIMIT, Stop.Reason);
            Assert.Equal(107, Stop.TripsDone);
            Assert.Equal(HaulSearchState.ThinCut, ClusterHauling.Handoff(Stop));
        }

        /// <summary>
        /// A planner that returns the same trip forever is an error at 500 trips.
        /// Thin-cut does not start.
        /// </summary>
        [Fact]
        public void RepeatedTripStopsAtTheTripLimit()
        {
            Grid Grid = MakeGrid();
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);
            ResidualPairTrip Stuck = OneTrip();
            ClusterHaulStop Stop = ClusterHauling.Run(
                Grid,
                Ledger,
                new Double[4, 4],
                new Double[4, 4],
                20.0 / 1.3,
                new List<Pass>(),
                Residual => new ClusterSchedule(PlanOf(new[] { Stuck }), new List<ResidualPairPiece>()),
                Batch => { });
            Assert.Equal(ClusterHauling.TRIP_LIMIT, Stop.Reason);
            Assert.Equal(ClusterHauling.MAX_CLUSTER_TRIPS, Stop.TripsDone);
            Assert.Equal(0, Stop.RemainingTrips);
            Assert.Equal(0.0, Stop.RemainingBcy);
            InvalidOperationException Halt = Assert.Throws<InvalidOperationException>(
                () => ClusterHauling.Handoff(Stop));
            Assert.Contains("trip limit", Halt.Message, StringComparison.Ordinal);
        }

        /// <summary>
        /// Drives a countdown. Each plan returns the trips still left, and
        /// executing a batch of at most ten reduces that count.
        /// </summary>
        /// <param name="TripCount">Trips the field needs.</param>
        /// <returns>The stop.</returns>
        private static ClusterHaulStop DriveCountdown
            (
            Int32 TripCount
            )
        {
            Grid Grid = MakeGrid();
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, 10.0);
            Int32 Left = TripCount;
            return ClusterHauling.Run(
                Grid,
                Ledger,
                new Double[4, 4],
                new Double[4, 4],
                20.0 / 1.3,
                new List<Pass>(),
                Residual =>
                {
                    // The next plan sees the residual after the previous batch, modelled as the count still left.
                    if (Left <= 0)
                    {
                        return new ClusterSchedule(
                            ResidualPairing.Plan(new List<ResidualPairPiece>(), 1.0, 1.0, 1.0),
                            new List<ResidualPairPiece>());
                    }
                    ResidualPairTrip[] Trips = new ResidualPairTrip[Left];
                    for (Int32 Index = 0; Index < Left; Index += 1)
                    {
                        Trips[Index] = OneTrip();
                    }
                    return new ClusterSchedule(PlanOf(Trips), new List<ResidualPairPiece>());
                },
                Batch =>
                {
                    Left -= Batch.Count;
                });
        }

        /// <summary>
        /// One trip of a fixed payload. The stuck planner returns this same trip.
        /// </summary>
        /// <returns>The trip.</returns>
        private static ResidualPairTrip OneTrip()
        {
            return new ResidualPairTrip(
                1,
                new[] { new ResidualPairLeg("C1", TRIP_BCY) },
                new[] { new ResidualPairLeg("F1", TRIP_BCY) },
                TRIP_BCY,
                10.0,
                1.0);
        }

        /// <summary>
        /// A balanced plan whose only content is the trips.
        /// </summary>
        /// <param name="Trips">The schedule.</param>
        /// <returns>The plan.</returns>
        private static ResidualPairPlan PlanOf
            (
            IReadOnlyList<ResidualPairTrip> Trips
            )
        {
            Double Volume = Trips.Count * TRIP_BCY;
            return new ResidualPairPlan(
                new List<ResidualPairFlow>(),
                Trips,
                new List<ResidualPairRemainder>(),
                Volume,
                Volume,
                "balanced",
                0.0,
                0,
                Trips.Count == 0 ? 0.0 : TRIP_BCY,
                0.0,
                10.0);
        }

        /// <summary>
        /// A small valid grid. The countdown tests do not move its dirt.
        /// </summary>
        /// <returns>The grid.</returns>
        private static Grid MakeGrid()
        {
            Boolean[,] Valid = new Boolean[4, 4];
            for (Int32 Row = 0; Row < 4; Row += 1)
            {
                for (Int32 Col = 0; Col < 4; Col += 1)
                {
                    Valid[Row, Col] = true;
                }
            }
            return new Grid(
                2.5908,
                4,
                4,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[4, 4],
                new Double[4, 4],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
