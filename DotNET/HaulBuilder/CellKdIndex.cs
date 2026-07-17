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

        /// <summary>
        /// Initializes a new Cell Kd Index instance.
        /// </summary>
        /// <param name="Handle">The handle used by the operation.</param>
        /// <param name="Cells">The cells used by the operation.</param>

        private CellKdIndex
            (
            IntPtr Handle,
            List<(Int32 Bx, Int32 By)> Cells
            )
        {
            _Handle = Handle;
            _Cells = Cells;
            _IndexScratch = new Int32[256];
            _DistScratch = new Double[256];
        }

        public Int32 Count => _Cells.Count;

        /// <summary>
        /// Executes the dispose operation.
        /// </summary>

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

        /// <summary>
        /// Executes the from cells operation.
        /// </summary>
        /// <param name="Cells">The cells used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting from cells.</returns>

        static public CellKdIndex? FromCells
            (
            IReadOnlyList<(Int32 Bx, Int32 By)> Cells,
            Grid Grid
            )
        {
            if (!NativeKd.Available)
            {
                throw new InvalidOperationException("Native haul_kdtree library is not loaded.");
            }

            if (Cells.Count <= 0)
            {
                return null;
            }

            Int64 BuildStart = PassProfiler.Enabled ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;
            Int32 Count = Cells.Count;
            Double[] Coords = new Double[Count * 2];
            for (Int32 Iter = 0; Iter < Count; Iter += 1)
            {
                (Int32 Bx, Int32 By) Cell = Cells[Iter];
                Int32 Offset = Iter * 2;
                Coords[Offset] = Grid.CenterX[Cell.By, Cell.Bx];
                Coords[Offset + 1] = Grid.CenterY[Cell.By, Cell.Bx];
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

            return new CellKdIndex(Handle, new List<(Int32 Bx, Int32 By)>(Cells));
        }

        /// <summary>
        /// Executes the cell at operation.
        /// </summary>
        /// <param name="NodeIndex">Zero-based item index.</param>
        /// <returns>The resulting cell at.</returns>

        public (Int32 Bx, Int32 By) CellAt
            (
            Int32 NodeIndex
            )
        {
            return _Cells[NodeIndex];
        }

        /// <summary>
        /// Executes the query knn operation.
        /// </summary>
        /// <param name="QueryX">Horizontal coordinate in field units.</param>
        /// <param name="QueryY">Vertical coordinate in field units.</param>
        /// <param name="k">The k used by the operation.</param>
        /// <param name="OutIndices">The out indices used by the operation.</param>
        /// <param name="OutDistSq">The out dist sq used by the operation.</param>
        /// <returns>The resulting query knn.</returns>

        public Int32 QueryKnn
            (
            Double QueryX,
            Double QueryY,
            Int32 k,
            Int32[] OutIndices,
            Double[] OutDistSq
            )
        {
            return NativeKd.haul_kdtree_query_knn(_Handle, QueryX, QueryY, k, OutIndices, OutDistSq);
        }

        /// <summary>
        /// Executes the nearest operation.
        /// </summary>
        /// <param name="QueryX">Horizontal coordinate in field units.</param>
        /// <param name="QueryY">Vertical coordinate in field units.</param>
        /// <param name="k">The k used by the operation.</param>
        /// <returns>The resulting nearest.</returns>

        public IEnumerable<(Int32 NodeIndex, Double DistSq)> Nearest
            (
            Double QueryX,
            Double QueryY,
            Int32 k
            )
        {
            Int32 Take = Math.Min(k, _Cells.Count);
            Int32[] Indices = (Take <= _IndexScratch.Length) ? _IndexScratch : new Int32[Take];
            Double[] Dists = (Take <= _DistScratch.Length) ? _DistScratch : new Double[Take];
            Int32 Found = QueryKnn(QueryX, QueryY, Take, Indices, Dists);
            for (Int32 Iter = 0; Iter < Found; Iter += 1)
            {
                yield return (Indices[Iter], Dists[Iter]);
            }
        }

        /// <summary>
        /// Executes the choose anchor operation.
        /// </summary>
        /// <param name="CutCells">The cut cells used by the operation.</param>
        /// <param name="FillIndex">Zero-based item index.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting choose anchor.</returns>

        static public (Int32 Bx, Int32 By) ChooseAnchor
            (
            IReadOnlyList<(Int32 Bx, Int32 By)> CutCells,
            CellKdIndex FillIndex,
            Grid Grid
            )
        {
            if (CutCells.Count <= 0)
            {
                throw new ArgumentException("cutCells must not be empty.", nameof(CutCells));
            }

            Int32 CutCount = CutCells.Count;
            Double[] CutCoords = new Double[CutCount * 2];
            for (Int32 Iter = 0; Iter < CutCount; Iter += 1)
            {
                (Int32 Bx, Int32 By) Cell = CutCells[Iter];
                CutCoords[(Iter * 2)] = Grid.CenterX[Cell.By, Cell.Bx];
                CutCoords[(Iter * 2) + 1] = Grid.CenterY[Cell.By, Cell.Bx];
            }

            Int32[] NearestIndices = new Int32[CutCount];
            Double[] NearestDistSq = new Double[CutCount];
            NativeKd.haul_kdtree_query_batch_k1(FillIndex._Handle, CutCount, CutCoords, NearestIndices, NearestDistSq);
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

            return CutCells[BestCut];
        }
    }
}
