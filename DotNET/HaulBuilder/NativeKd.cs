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

        public static Boolean Available => _Available;

        static private IntPtr ResolveLibrary
            (
            String libraryName,
            Assembly assembly,
            DllImportSearchPath? searchPath
            )
        {
            if (!String.Equals(libraryName, "haul_kdtree", StringComparison.Ordinal))
            {
                return IntPtr.Zero;
            }

            if (NativeLibrary.TryLoad("haul_kdtree", assembly, searchPath, out IntPtr Handle))
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

        [DllImport("haul_kdtree", CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr haul_kdtree_build(Int32 n_points, Double[] coords, Int32 copy_coords);

        [DllImport("haul_kdtree", CallingConvention = CallingConvention.Cdecl)]
        public static extern void haul_kdtree_free(IntPtr tree);

        [DllImport("haul_kdtree", CallingConvention = CallingConvention.Cdecl)]
        public static extern Int32 haul_kdtree_query_knn(
            IntPtr tree,
            Double x,
            Double y,
            Int32 k,
            Int32[] out_indices,
            Double[] out_dist_sq);

        [DllImport("haul_kdtree", CallingConvention = CallingConvention.Cdecl)]
        public static extern void haul_kdtree_query_batch_k1(
            IntPtr tree,
            Int32 n_query,
            Double[] query_coords,
            Int32[] out_indices,
            Double[] out_dist_sq);
    }
}
