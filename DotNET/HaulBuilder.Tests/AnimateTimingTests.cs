using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    public sealed class AnimateTimingTests
    {
        [Fact]
        public void ExactProgressDurationIncludesFourSecondFinalHold()
        {
            IReadOnlyList<Int32> Delays = Animate.ComputeGifFrameDelaysCs(
                102,
                100,
                10000,
                4000);

            Assert.Equal(102, Delays.Count);
            Assert.Equal(1400, Delays.Sum());
            Assert.Equal(400, Delays[^1]);
            Assert.All(Delays.Take(101), Delay => Assert.InRange(Delay, 9, 10));
        }

        [Fact]
        public void UnspecifiedProgressDurationRetainsUniformCadence()
        {
            IReadOnlyList<Int32> Delays = Animate.ComputeGifFrameDelaysCs(
                3,
                100,
                null,
                4000);

            Assert.Equal(new[] { 10, 10, 400 }, Delays);
        }
    }
}
