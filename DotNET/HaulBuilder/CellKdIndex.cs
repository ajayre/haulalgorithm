using System.Runtime.InteropServices;

namespace HaulBuilder
{
    /// <summary>
    /// KD-tree index over active grid cells for fast nearest-neighbor queries.
    /// Uses native nanoflann when <see cref="NativeKd"/> is available.
    /// </summary>
    internal sealed class CellKdIndex : IDisposable
    {
        private IntPtr _Handle;
        private readonly List<(Int32 Bx, Int32 By)> _Cells;
        private readonly Int32[] _IndexScratch;
        private readonly Double[] _DistScratch;

        private CellKdIndex
            (
            IntPtr handle,
            List<(Int32 Bx, Int32 By)> cells
            )
        {
            _Handle = handle;
            _Cells = cells;
            _IndexScratch = new Int32[256];
            _DistScratch = new Double[256];
        }

        public Int32 Count => _Cells.Count;

        public void Dispose
            (
            )
        {
            if (_Handle != IntPtr.Zero)
            {
                NativeKd.haul_kdtree_free(_Handle);
                _Handle = IntPtr.Zero;
            }
        }

        static public CellKdIndex? FromCells
            (
            IReadOnlyList<(Int32 Bx, Int32 By)> cells,
            Grid grid
            )
        {
            if (!NativeKd.Available)
            {
                throw new InvalidOperationException("Native haul_kdtree library is not loaded.");
            }

            if (cells.Count <= 0)
            {
                return null;
            }

            Int64 BuildStart = PassProfiler.Enabled ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
            Int32 Count = cells.Count;
            Double[] Coords = new Double[Count * 2];
            for (Int32 Iter = 0; Iter < Count; Iter += 1)
            {
                (Int32 Bx, Int32 By) Cell = cells[Iter];
                Int32 Offset = Iter * 2;
                Coords[Offset] = grid.CenterX[Cell.By, Cell.Bx];
                Coords[Offset + 1] = grid.CenterY[Cell.By, Cell.Bx];
            }

            IntPtr Handle = NativeKd.haul_kdtree_build(Count, Coords, 1);
            if (Handle == IntPtr.Zero)
            {
                throw new InvalidOperationException("haul_kdtree_build failed.");
            }

            PassProfiler.AddTicks(ref PassProfiler.KdBuildTicks, BuildStart);
            if (PassProfiler.Enabled)
            {
                PassProfiler.KdBuilds += 1;
            }

            return new CellKdIndex(Handle, new List<(Int32 Bx, Int32 By)>(cells));
        }

        public (Int32 Bx, Int32 By) CellAt
            (
            Int32 nodeIndex
            )
        {
            return _Cells[nodeIndex];
        }

        public Int32 QueryKnn
            (
            Double queryX,
            Double queryY,
            Int32 k,
            Int32[] outIndices,
            Double[] outDistSq
            )
        {
            return NativeKd.haul_kdtree_query_knn(_Handle, queryX, queryY, k, outIndices, outDistSq);
        }

        public IEnumerable<(Int32 NodeIndex, Double DistSq)> Nearest
            (
            Double queryX,
            Double queryY,
            Int32 k
            )
        {
            Int32 Take = Math.Min(k, _Cells.Count);
            Int32[] Indices = (Take <= _IndexScratch.Length) ? _IndexScratch : new Int32[Take];
            Double[] Dists = (Take <= _DistScratch.Length) ? _DistScratch : new Double[Take];
            Int32 Found = QueryKnn(queryX, queryY, Take, Indices, Dists);
            for (Int32 Iter = 0; Iter < Found; Iter += 1)
            {
                yield return (Indices[Iter], Dists[Iter]);
            }
        }

        static public (Int32 Bx, Int32 By) ChooseAnchor
            (
            IReadOnlyList<(Int32 Bx, Int32 By)> cutCells,
            CellKdIndex fillIndex,
            Grid grid
            )
        {
            if (cutCells.Count <= 0)
            {
                throw new ArgumentException("cutCells must not be empty.", nameof(cutCells));
            }

            Int32 CutCount = cutCells.Count;
            Double[] CutCoords = new Double[CutCount * 2];
            for (Int32 Iter = 0; Iter < CutCount; Iter += 1)
            {
                (Int32 Bx, Int32 By) Cell = cutCells[Iter];
                CutCoords[(Iter * 2)] = grid.CenterX[Cell.By, Cell.Bx];
                CutCoords[(Iter * 2) + 1] = grid.CenterY[Cell.By, Cell.Bx];
            }

            Int32[] NearestIndices = new Int32[CutCount];
            Double[] NearestDistSq = new Double[CutCount];
            NativeKd.haul_kdtree_query_batch_k1(fillIndex._Handle, CutCount, CutCoords, NearestIndices, NearestDistSq);
            if (PassProfiler.Enabled)
            {
                PassProfiler.AnchorBatchQueries += 1;
            }

            Int32 BestCut = 0;
            Double BestDistSq = NearestDistSq[0];
            for (Int32 Iter = 1; Iter < CutCount; Iter += 1)
            {
                // Strict less-than keeps the first minimum; with cutCells sorted
                // (by, bx) that matches Python np.argmin on argwhere order.
                if (NearestDistSq[Iter] < BestDistSq)
                {
                    BestDistSq = NearestDistSq[Iter];
                    BestCut = Iter;
                }
            }

            return cutCells[BestCut];
        }
    }
}
