using System;
using System.Collections.Generic;

using Xunit;

namespace HaulBuilder.Tests
{
    /// <summary>Tests fixed-order route accounting used to join frozen route batches.</summary>
    public sealed class RoutingTests
    {
        /// <summary>Deadhead includes each boundary between consecutive passes.</summary>
        [Fact]
        public void CalculateDeadhead_PreservesFixedOrderAndGradeExit()
        {
            List<Pass> Passes = new List<Pass>
            {
                MakePass((0.0, 0.0), (10.0, 0.0), (100.0, 100.0)),
                MakePass((13.0, 4.0), (20.0, 0.0), (200.0, 200.0)),
                MakePass((20.0, 12.0), (30.0, 0.0), (300.0, 300.0)),
            };

            Double DistanceM = Routing.CalculateDeadhead(Passes, GradeReturn: true);
            Double NoGradeDistanceM = Routing.CalculateDeadhead(
                Passes, GradeReturn: false);

            Assert.Equal(17.0, DistanceM, 9);
            Assert.Equal(
                Math.Sqrt((87.0 * 87.0) + (96.0 * 96.0))
                    + Math.Sqrt((Constants.DEGREES_PER_HALF_TURN * Constants.DEGREES_PER_HALF_TURN) + (188.0 * 188.0)),
                NoGradeDistanceM,
                9);
        }

        /// <summary>Legacy/default sequencing retains pass zero as its anchor.</summary>
        [Fact]
        public void SequencePasses_DefaultStartPreservesLegacyAnchor()
        {
            List<Pass> Passes = new List<Pass>
            {
                MakePass((100.0, 100.0), (110.0, 100.0), (120.0, 100.0)),
                MakePass((0.0, 0.0), (10.0, 0.0), (20.0, 0.0)),
                MakePass((20.0, 0.0), (30.0, 0.0), (40.0, 0.0)),
            };

            Route Route = Routing.SequencePasses(Passes);

            Assert.Equal(0, Route.Order[0]);
        }

        /// <summary>
        /// Verifies direction-aware order follows transition seconds, so a long
        /// straight deadhead beats a short half-turn.
        /// </summary>
        [Fact]
        public void SequencePasses_DirectionAware_PrefersFasterStraightOverShortTurn
            (
            )
        {
            // Grade return exits at the fill start, heading from fill end back to fill start (south).
            Pass Anchor = MakePass((0.0, 0.0), (0.0, 0.0), (0.0, 10.0));
            // Ten metres north: a half-turn, short in metres and slow in seconds.
            Pass ShortTurn = MakePass((0.0, 10.0), (0.0, 20.0), (0.0, 30.0));
            // Eighty metres south: straight, long in metres and fast in seconds.
            Pass LongStraight = MakePass((0.0, -80.0), (0.0, -70.0), (0.0, -60.0));
            Routing.TransitionCostEvaluator Evaluator = new Routing.TransitionCostEvaluator();
            Double ShortSeconds = Evaluator.CostSeconds(Anchor, ShortTurn);
            Double LongSeconds = Evaluator.CostSeconds(Anchor, LongStraight);
            Assert.True(LongSeconds < ShortSeconds);

            Route Route = Routing.SequencePasses(
                new List<Pass> { Anchor, ShortTurn, LongStraight });
            Assert.Equal(0, Route.Order[0]);
            // Index 2 is the long straight. Ranking by metres was measured and reverted.
            Assert.Equal(2, Route.Order[1]);
        }

        /// <summary>
        /// Builds pass.
        /// </summary>
        /// <param name="CutStart">The cut start used by the operation.</param>
        /// <param name="FillStart">The fill start used by the operation.</param>
        /// <param name="FillEnd">The fill end used by the operation.</param>
        /// <returns>The resulting pass.</returns>

        private static Pass MakePass
            (
            (Double X, Double Y) CutStart,
            (Double X, Double Y) FillStart,
            (Double X, Double Y) FillEnd
            )
        {
            return new Pass(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                1.0,
                1.0,
                CutStart,
                CutStart,
                FillStart,
                FillEnd);
        }
    }
}
