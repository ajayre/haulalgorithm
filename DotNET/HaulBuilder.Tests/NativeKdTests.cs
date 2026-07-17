namespace HaulBuilder.Tests
{
    /// <summary>
    /// Tests for the native nanoflann KD-tree wrapper.
    /// </summary>
    public sealed class NativeKdTests
    {
        /// <summary>
        /// Verifies native library loads and batch anchor matches brute force.
        /// </summary>
        [Fact]
        public void NativeKd_BatchK1_Matches_BruteForce
            (
            )
        {
            Assert.True(NativeKd.Available, "haul_kdtree native library must be present for tests.");

            Double[] FillCoords =
            {
                0.0, 0.0,
                10.0, 0.0,
                0.0, 10.0,
                10.0, 10.0,
            };
            IntPtr Tree = NativeKd.haul_kdtree_build(4, FillCoords, 1);
            try
            {
                Double[] CutCoords =
                {
                    1.0, 1.0,
                    9.0, 9.0,
                    5.0, 5.0,
                };
                Int32[] BatchIndices = new Int32[3];
                Double[] BatchDist = new Double[3];
                NativeKd.haul_kdtree_query_batch_k1(Tree, 3, CutCoords, BatchIndices, BatchDist);

                for (Int32 Query = 0; Query < 3; Query += 1)
                {
                    Double X = CutCoords[Query * 2];
                    Double Y = CutCoords[(Query * 2) + 1];
                    Int32 Best = 0;
                    Double BestDist = Double.PositiveInfinity;
                    for (Int32 Fill = 0; Fill < 4; Fill += 1)
                    {
                        Double Dx = X - FillCoords[Fill * 2];
                        Double Dy = Y - FillCoords[(Fill * 2) + 1];
                        Double Dist = (Dx * Dx) + (Dy * Dy);
                        if (Dist < BestDist)
                        {
                            BestDist = Dist;
                            Best = Fill;
                        }
                    }

                    Assert.Equal(Best, BatchIndices[Query]);
                    Assert.Equal(BestDist, BatchDist[Query], Constants.NUMERIC_COMPARISON_TOLERANCE);
                }
            }
            finally
            {
                NativeKd.haul_kdtree_free(Tree);
            }
        }
    }
}
