using System.Diagnostics;
using System.Globalization;
using System.Text;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Shared helpers for cross-language parity tests against the Python <c>landlevel</c> package.
    /// </summary>
    internal static class PythonParityHelper
    {
        /// <summary>
        /// Requires a Python interpreter on PATH; fails the test when unavailable.
        /// </summary>
        internal static String RequirePython
            (
            )
        {
            if (TryFindPython(out String PythonExe))
            {
                return PythonExe;
            }

            throw new InvalidOperationException(
                "Python 3 with the landlevel package is required for parity tests. " +
                "Install Python and ensure Python/landlevel is on PYTHONPATH via the repo root.");
        }

        internal static Boolean TryFindPython
            (
            out String pythonExe
            )
        {
            foreach (String Candidate in new String[] { "python", "python3", "py" })
            {
                try
                {
                    ProcessStartInfo Start = new ProcessStartInfo
                    {
                        FileName = Candidate,
                        Arguments = Candidate == "py" ? "-3 --version" : "--version",
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        UseShellExecute = false,
                        CreateNoWindow = true,
                    };
                    using Process Process = Process.Start(Start)!;
                    Process.WaitForExit(5000);
                    if (Process.ExitCode == 0)
                    {
                        pythonExe = Candidate;
                        return true;
                    }
                }
                catch (Exception)
                {
                    // Try the next interpreter name.
                }
            }

            pythonExe = String.Empty;
            return false;
        }

        internal static String FindRepoRoot
            (
            )
        {
            String? Dir = AppContext.BaseDirectory;
            while (Dir is not null)
            {
                if (Directory.Exists(Path.Combine(Dir, "Python", "landlevel")))
                {
                    return Path.GetDirectoryName(Path.Combine(Dir, "Python", "landlevel"))!;
                }

                if (Directory.Exists(Path.Combine(Dir, "landlevel")))
                {
                    return Dir;
                }

                Dir = Directory.GetParent(Dir)?.FullName;
            }

            throw new InvalidOperationException("Could not locate Python landlevel package root.");
        }

        internal static String FindPythonPackageRoot
            (
            )
        {
            return FindRepoRoot();
        }

        internal static String ParityRenderScriptPath
            (
            )
        {
            return Path.Combine(AppContext.BaseDirectory, "parity_render.py");
        }

        internal static (Int32 ExitCode, String StdOut, String StdErr) RunPython
            (
            String pythonExe,
            String workingDirectory,
            String arguments
            )
        {
            ProcessStartInfo Start = new ProcessStartInfo
            {
                FileName = pythonExe,
                Arguments = arguments,
                WorkingDirectory = workingDirectory,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true,
            };
            Start.Environment["PYTHONPATH"] = workingDirectory;
            using Process Process = Process.Start(Start)!;
            String Output = Process.StandardOutput.ReadToEnd();
            String Error = Process.StandardError.ReadToEnd();
            Process.WaitForExit(300000);
            return (Process.ExitCode, Output, Error);
        }

        internal static String QuoteForShell
            (
            String value
            )
        {
            return "\"" + value.Replace("\"", "\\\"") + "\"";
        }

        internal static String WriteRampAgd
            (
            )
        {
            String AgdPath = Path.Combine(Path.GetTempPath(), "haulbuilder_parity_" + Guid.NewGuid().ToString("N", CultureInfo.InvariantCulture) + ".agd");
            WriteRampAgd(AgdPath);
            return AgdPath;
        }

        internal static void WriteRampAgd
            (
            String path
            )
        {
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments");
            Sb.AppendLine("40.0, -93.0, 100.0,,,0MB,");
            Double Lat0 = 40.0;
            Double Lon0 = -93.0;
            Double DLat = (1.0 / 111000.0);
            Double DLon = (1.0 / (111000.0 * Math.Cos((Math.PI / 180.0) * Lat0)));
            Int32 N = 24;
            for (Int32 R = 0; R < N; R += 1)
            {
                for (Int32 C = 0; C < N; C += 1)
                {
                    Double Lat = (Lat0 + (R * 3 * DLat));
                    Double Lon = (Lon0 + (C * 3 * DLon));
                    Double Existing = (100.0 + ((C - (N / 2.0)) * 0.02));
                    Double Proposed = 100.0;
                    Double CutFill = (Proposed - Existing);
                    Sb.AppendFormat(
                        CultureInfo.InvariantCulture,
                        "{0:F8}, {1:F8}, {2:F3}, {3:F3}, {4:F3}, 3GRD, ",
                        Lat,
                        Lon,
                        Existing,
                        Proposed,
                        CutFill);
                    Sb.AppendLine();
                }
            }

            File.WriteAllText(path, Sb.ToString(), Encoding.UTF8);
        }

        internal static HaulBuilderOptions RampOptions
            (
            String agdPath,
            Int32 pixelsPerBin = 8
            )
        {
            return new HaulBuilderOptions
            {
                AgdPath = agdPath,
                GridSizeFt = 15.0,
                ToleranceMm = 10.0,
                NoFlowAligned = true,
                HeadingDeg = 90.0,
                HeadingStrength = 0.5,
                Zones = 0,
                PixelsPerBin = pixelsPerBin,
                ArrowStride = 2,
                PairsPngPath = "__IN_MEMORY__",
                GuidancePngPath = "__IN_MEMORY__",
                PairSampleCount = 0,
            };
        }
    }
}
