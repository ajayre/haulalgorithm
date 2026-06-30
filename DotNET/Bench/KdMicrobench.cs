using System;
using System.Diagnostics;
using HaulBuilder;

internal static class KdMicrobench
{
    internal static void Run
        (
        Int32 cutCount,
        Int32 fillCount,
        Int32 iterations
        )
    {
        if (!NativeKd.Available)
        {
            Console.WriteLine("Native haul_kdtree is not loaded.");
            return;
        }

        Random Rng = new Random(0);
        Double[] FillCoords = new Double[fillCount * 2];
        for (Int32 Iter = 0; Iter < fillCount; Iter += 1)
        {
            FillCoords[(Iter * 2)] = Rng.NextDouble();
            FillCoords[(Iter * 2) + 1] = Rng.NextDouble();
        }

        Double[][] CutPoints = new Double[cutCount][];
        Double[] CutCoords = new Double[cutCount * 2];
        for (Int32 Iter = 0; Iter < cutCount; Iter += 1)
        {
            CutCoords[(Iter * 2)] = Rng.NextDouble();
            CutCoords[(Iter * 2) + 1] = Rng.NextDouble();
            CutPoints[Iter] = new Double[] { CutCoords[(Iter * 2)], CutCoords[(Iter * 2) + 1] };
        }

        Stopwatch Sw = Stopwatch.StartNew();
        for (Int32 Iter = 0; Iter < iterations; Iter += 1)
        {
            IntPtr Tree = NativeKd.haul_kdtree_build(fillCount, FillCoords, 1);
            NativeKd.haul_kdtree_free(Tree);
        }

        Double BuildMs = Sw.Elapsed.TotalMilliseconds / iterations;
        IntPtr Built = NativeKd.haul_kdtree_build(fillCount, FillCoords, 1);
        Int32[] Indices = new Int32[1];
        Double[] Dists = new Double[1];
        Sw.Restart();
        for (Int32 Iter = 0; Iter < iterations; Iter += 1)
        {
            NativeKd.haul_kdtree_query_batch_k1(Built, cutCount, CutCoords, new Int32[cutCount], new Double[cutCount]);
        }

        Double BatchMs = Sw.Elapsed.TotalMilliseconds / iterations;
        Sw.Restart();
        for (Int32 Iter = 0; Iter < iterations; Iter += 1)
        {
            for (Int32 Cut = 0; Cut < cutCount; Cut += 1)
            {
                NativeKd.haul_kdtree_query_knn(Built, CutPoints[Cut][0], CutPoints[Cut][1], 1, Indices, Dists);
            }
        }

        Double PerPointMs = Sw.Elapsed.TotalMilliseconds / iterations;
        NativeKd.haul_kdtree_free(Built);
        Console.WriteLine(String.Format("--- kd microbench (native nanoflann, n_cut={0} n_fill={1} x{2}) ---", cutCount, fillCount, iterations));
        Console.WriteLine(String.Format("build_ms={0:F2} batch_k1_ms={1:F2} per_point_query_ms={2:F2}", BuildMs, BatchMs, PerPointMs));
    }
}
