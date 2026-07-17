using System.Reflection;
using System.Runtime.InteropServices;

namespace HaulBuilder
{
    /// <summary>
    /// P/Invoke wrapper for the native nanoflann-backed KD-tree library.
    /// </summary>
    internal static class NativeKd
    {
        private static readonly Boolean _Available = TryLoad();

        /// <summary>
        /// Executes the try load operation.
        /// </summary>
        /// <returns>The resulting try load.</returns>

        static private Boolean TryLoad
            (
            )
        {
            try
            {
                NativeLibrary.SetDllImportResolver(typeof(NativeKd).Assembly, ResolveLibrary);
                if (!NativeLibrary.TryLoad("haul_kdtree", typeof(NativeKd).Assembly, null, out IntPtr _))
                {
                    return false;
                }

                return true;
            }
            catch (DllNotFoundException)
            {
                return false;
            }
            catch (BadImageFormatException)
            {
                return false;
            }
        }

        static public Boolean Available => _Available;

        /// <summary>
        /// Resolves library.
        /// </summary>
        /// <param name="LibraryName">The library name used by the operation.</param>
        /// <param name="Assembly">The assembly used by the operation.</param>
        /// <param name="SearchPath">Path to the required input or output.</param>
        /// <returns>The resulting library.</returns>

        static private IntPtr ResolveLibrary
            (
            String LibraryName,
            Assembly Assembly,
            DllImportSearchPath? SearchPath
            )
        {
            if (!String.Equals(LibraryName, "haul_kdtree", StringComparison.Ordinal))
            {
                return IntPtr.Zero;
            }

            if (NativeLibrary.TryLoad("haul_kdtree", Assembly, SearchPath, out IntPtr Handle))
            {
                return Handle;
            }

            String BaseDir = AppContext.BaseDirectory;
            String[] Candidates = OperatingSystem.IsWindows()
                ? new[] { Path.Combine(BaseDir, "haul_kdtree.dll") }
                : OperatingSystem.IsMacOS()
                    ? new[] { Path.Combine(BaseDir, "libhaul_kdtree.dylib"), Path.Combine(BaseDir, "haul_kdtree.dylib") }
                    : new[] { Path.Combine(BaseDir, "libhaul_kdtree.so"), Path.Combine(BaseDir, "haul_kdtree.so") };

            foreach (String Candidate in Candidates)
            {
                if (File.Exists(Candidate) && NativeLibrary.TryLoad(Candidate, out Handle))
                {
                    return Handle;
                }
            }

            return IntPtr.Zero;
        }

        /// <summary>
        /// Executes the haul kdtree build operation.
        /// </summary>
        /// <param name="N_points">The n points used by the operation.</param>
        /// <param name="Coords">The coords used by the operation.</param>
        /// <param name="Copy_coords">The copy coords used by the operation.</param>
        /// <returns>The resulting haul kdtree build.</returns>

        [DllImport("haul_kdtree", CallingConvention = CallingConvention.Cdecl)]
        static public extern IntPtr haul_kdtree_build(Int32 N_points, Double[] Coords, Int32 Copy_coords);

        /// <summary>
        /// Executes the haul kdtree free operation.
        /// </summary>
        /// <param name="Tree">The tree used by the operation.</param>

        [DllImport("haul_kdtree", CallingConvention = CallingConvention.Cdecl)]
        static public extern void haul_kdtree_free(IntPtr Tree);

        /// <summary>
        /// Executes the haul kdtree query knn operation.
        /// </summary>
        /// <param name="Tree">The tree used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="k">The k used by the operation.</param>
        /// <param name="Out_indices">The out indices used by the operation.</param>
        /// <param name="Out_dist_sq">The out dist sq used by the operation.</param>
        /// <returns>The resulting haul kdtree query knn.</returns>

        [DllImport("haul_kdtree", CallingConvention = CallingConvention.Cdecl)]
        static public extern Int32 haul_kdtree_query_knn(
            IntPtr Tree,
            Double X,
            Double Y,
            Int32 k,
            Int32[] Out_indices,
            Double[] Out_dist_sq);

        /// <summary>
        /// Executes the haul kdtree query batch k1 operation.
        /// </summary>
        /// <param name="Tree">The tree used by the operation.</param>
        /// <param name="N_query">The n query used by the operation.</param>
        /// <param name="Query_coords">The query coords used by the operation.</param>
        /// <param name="Out_indices">The out indices used by the operation.</param>
        /// <param name="Out_dist_sq">The out dist sq used by the operation.</param>

        [DllImport("haul_kdtree", CallingConvention = CallingConvention.Cdecl)]
        static public extern void haul_kdtree_query_batch_k1(
            IntPtr Tree,
            Int32 N_query,
            Double[] Query_coords,
            Int32[] Out_indices,
            Double[] Out_dist_sq);
    }
}
