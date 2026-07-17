using System;
using System.Diagnostics;
using HaulBuilder;

/// <summary>
/// Provides kd microbench behavior.
/// </summary>
internal static class KdMicrobench
{
    /// <summary>
    /// Runs the requested workflow.
    /// </summary>
    /// <param name="CutCount">Count limit applied by the operation.</param>
    /// <param name="FillCount">Count limit applied by the operation.</param>
    /// <param name="Iterations">The iterations used by the operation.</param>

    internal static void Run
        (
        Int32 CutCount,
        Int32 FillCount,
        Int32 Iterations
        )
    {
        if (!NativeKd.Available)
        {
            Console.WriteLine("Native haul_kdtree is not loaded.");
            return;
        }

        Random Rng = new Random(0);
        Double[] FillCoords = new Double[FillCount * 2];
        for (Int32 Iter = 0; Iter < FillCount; Iter += 1)
        {
            FillCoords[(Iter * 2)] = Rng.NextDouble();
            FillCoords[(Iter * 2) + 1] = Rng.NextDouble();
        }

        Double[][] CutPoints = new Double[CutCount][];
        Double[] CutCoords = new Double[CutCount * 2];
        for (Int32 Iter = 0; Iter < CutCount; Iter += 1)
        {
            CutCoords[(Iter * 2)] = Rng.NextDouble();
            CutCoords[(Iter * 2) + 1] = Rng.NextDouble();
            CutPoints[Iter] = new Double[] { CutCoords[(Iter * 2)], CutCoords[(Iter * 2) + 1] };
        }

        Stopwatch Sw = Stopwatch.StartNew();
        for (Int32 Iter = 0; Iter < Iterations; Iter += 1)
        {
            IntPtr Tree = NativeKd.haul_kdtree_build(FillCount, FillCoords, 1);
            NativeKd.haul_kdtree_free(Tree);
        }

        Double BuildMs = Sw.Elapsed.TotalMilliseconds / Iterations;
        IntPtr Built = NativeKd.haul_kdtree_build(FillCount, FillCoords, 1);
        Int32[] Indices = new Int32[1];
        Double[] Dists = new Double[1];
        Sw.Restart();
        for (Int32 Iter = 0; Iter < Iterations; Iter += 1)
        {
            NativeKd.haul_kdtree_query_batch_k1(Built, CutCount, CutCoords, new Int32[CutCount], new Double[CutCount]);
        }

        Double BatchMs = Sw.Elapsed.TotalMilliseconds / Iterations;
        Sw.Restart();
        for (Int32 Iter = 0; Iter < Iterations; Iter += 1)
        {
            for (Int32 Cut = 0; Cut < CutCount; Cut += 1)
            {
                NativeKd.haul_kdtree_query_knn(Built, CutPoints[Cut][0], CutPoints[Cut][1], 1, Indices, Dists);
            }
        }

        Double PerPointMs = Sw.Elapsed.TotalMilliseconds / Iterations;
        NativeKd.haul_kdtree_free(Built);
        Console.WriteLine(String.Format("--- kd microbench (native nanoflann, n_cut={0} n_fill={1} x{2}) ---", CutCount, FillCount, Iterations));
        Console.WriteLine(String.Format("build_ms={0:F2} batch_k1_ms={1:F2} per_point_query_ms={2:F2}", BuildMs, BatchMs, PerPointMs));
    }
}
