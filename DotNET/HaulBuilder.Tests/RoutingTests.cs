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
                    + Math.Sqrt((180.0 * 180.0) + (188.0 * 188.0)),
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
