using System.Diagnostics;
using System.Globalization;
using System.Text;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Cross-language parity checks against the Python <c>landlevel</c> package on synthetic fields.
    /// </summary>
    public sealed class PythonParityTests
    {
        /// <summary>
        /// Verifies pass count and moved BCY match Python on the synthetic east-up ramp.
        /// </summary>
        [Fact]
        public void Synthetic_Ramp_Matches_Python_Metrics
            (
            )
        {
            String PythonExe = PythonParityHelper.RequirePython();
            String RepoRoot = PythonParityHelper.FindPythonPackageRoot();
            String AgdPath = PythonParityHelper.WriteRampAgd();
            try
            {
                HaulBuilderResult CsResult = RunCSharp(AgdPath);
                (Int32 PyPasses, Double PyMovedBcy) = RunPythonMetrics(PythonExe, RepoRoot, AgdPath);
                Assert.Equal(PyPasses, CsResult.PassResult!.Passes.Count);
                Assert.Equal(PyMovedBcy, CsResult.Metrics!.MoveVolumeBcy, 0.5);
            }
            finally
            {
                if (File.Exists(AgdPath))
                {
                    File.Delete(AgdPath);
                }
            }
        }

        private static HaulBuilderResult RunCSharp
            (
            String agdPath
            )
        {
            HaulBuilderEngine Engine = new HaulBuilderEngine();
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = agdPath,
                GridSizeFt = 15.0,
                ToleranceMm = 10.0,
                NoFlowAligned = true,
                HeadingDeg = 90.0,
                HeadingStrength = 0.5,
                NoPlan = false,
                Zones = 0,
            };
            return Engine.Run(Options);
        }

        private static (Int32 Passes, Double MovedBcy) RunPythonMetrics
            (
            String pythonExe,
            String repoRoot,
            String agdPath
            )
        {
            String Script = """
import sys
from landlevel.agd import parse_agd_file
from landlevel.balance import balance
from landlevel.flowfield import compute_flow_field
from landlevel.grid import build_grid
from landlevel.passes import generate_passes
from landlevel.routing import sequence_passes
from landlevel.strips import trace_strips

field = parse_agd_file(sys.argv[1])
grid = build_grid(field, grid_size_ft=15.0)
balance(grid, mode='check')
flow = compute_flow_field(grid, mode='poisson')
strips = trace_strips(grid, flow)
pr = generate_passes(grid, strips, tolerance_mm=10.0, max_load_bcy=15.38, heading_deg=90.0, heading_strength=0.5)
route = sequence_passes(pr.passes)
moved = sum(p.load_bcy for p in pr.passes)
print(f'{len(pr.passes)},{moved:.6f}')
""";
            ProcessStartInfo Start = new ProcessStartInfo
            {
                FileName = pythonExe,
                Arguments = "-c " + QuoteForShell(Script) + " " + QuoteForShell(agdPath),
                WorkingDirectory = repoRoot,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true,
            };
            using Process Process = Process.Start(Start)!;
            String Output = Process.StandardOutput.ReadToEnd();
            String Error = Process.StandardError.ReadToEnd();
            Process.WaitForExit(120000);
            Assert.True(Process.ExitCode == 0, $"Python failed: {Error}");
            String[] Parts = Output.Trim().Split(',');
            return (Int32.Parse(Parts[0], CultureInfo.InvariantCulture), Double.Parse(Parts[1], CultureInfo.InvariantCulture));
        }

        private static String QuoteForShell
            (
            String value
            )
        {
            return PythonParityHelper.QuoteForShell(value);
        }
    }
}
