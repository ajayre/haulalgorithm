using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Cross-language render parity: algorithm data must match; PNGs must carry the same visual information.
    /// </summary>
    public sealed class PngParityTests
    {
        private const Int32 PixelsPerBin = 8;

        /// <summary>
        /// Main cut/fill map conveys the same cut vs fill information as Python on the synthetic ramp.
        /// </summary>
        [Fact]
        public void Main_Png_Conveys_Same_Information_As_Python
            (
            )
        {
            RunPngParity("main");
        }

        /// <summary>
        /// Cut/fill haul-density pairs overlay conveys the same information as Python on the synthetic ramp.
        /// </summary>
        [Fact]
        public void PassPairs_Png_Conveys_Same_Information_As_Python
            (
            )
        {
            RunPngParity("pass_pairs");
        }

        /// <summary>
        /// Blade-scale guidance overlay conveys the same action colors as Python on the synthetic ramp.
        /// </summary>
        [Fact]
        public void Guidance_Png_Conveys_Same_Information_As_Python
            (
            )
        {
            RunPngParity("guidance");
        }

        /// <summary>
        /// Single-pass cut/haul/fill diagram conveys the same information as Python on the synthetic ramp.
        /// </summary>
        [Fact]
        public void SinglePair_Png_Conveys_Same_Information_As_Python
            (
            )
        {
            RunPngParity("single_pair");
        }

        private static void RunPngParity
            (
            String pngKey
            )
        {
            String PythonExe = PythonParityHelper.RequirePython();
            String PythonRoot = PythonParityHelper.FindPythonPackageRoot();
            String AgdPath = PythonParityHelper.WriteRampAgd();
            String OutDir = Path.Combine(Path.GetTempPath(), "haulbuilder_parity_png_" + Guid.NewGuid().ToString("N", CultureInfo.InvariantCulture));
            Directory.CreateDirectory(OutDir);

            try
            {
                String ScriptPath = PythonParityHelper.ParityRenderScriptPath();
                String Args = String.Format(
                    CultureInfo.InvariantCulture,
                    "{0} {1} {2} {3} {4}",
                    PythonParityHelper.QuoteForShell(ScriptPath),
                    PythonParityHelper.QuoteForShell(AgdPath),
                    PythonParityHelper.QuoteForShell(OutDir),
                    PixelsPerBin,
                    2);
                (Int32 ExitCode, String StdOut, String StdErr) = PythonParityHelper.RunPython(PythonExe, PythonRoot, Args);
                Assert.True(ExitCode == 0, $"Python parity_render failed: {StdErr}");

                Dictionary<String, String> Manifest = ParseManifest(StdOut);
                Assert.True(Manifest.ContainsKey(pngKey), $"Python manifest missing {pngKey}: {StdOut}");
                HaulBuilderResult? CsResult = null;
                if (Manifest.ContainsKey("guidance_sig"))
                {
                    HaulBuilderOptions Options = PythonParityHelper.RampOptions(AgdPath, PixelsPerBin);
                    CsResult = new HaulBuilderEngine().Run(Options);
                    String CsGuidanceSig = String.Join(",", CsResult.GuidancePoints.Select(Point => Point.Action));
                    Assert.Equal(Manifest["guidance_sig"], CsGuidanceSig);
                }

                if ((pngKey == "single_pair") && Manifest.TryGetValue("single_pair_sig", out String? PyPairSig)
                    && Manifest.TryGetValue("single_pair_pass", out String? PassText)
                    && Int32.TryParse(PassText, NumberStyles.Integer, CultureInfo.InvariantCulture, out Int32 PassNo))
                {
                    CsResult ??= new HaulBuilderEngine().Run(PythonParityHelper.RampOptions(AgdPath, PixelsPerBin));
                    String CsPairSig = String.Join(
                        ",",
                        CsResult.GuidancePoints.Where(Point => Point.PassNo == PassNo).Select(Point => Point.Action));
                    Assert.Equal(PyPairSig, CsPairSig);
                }

                if ((pngKey == "single_pair") && Manifest.TryGetValue("haul_seg", out String? HaulSeg))
                {
                    CsResult ??= new HaulBuilderEngine().Run(PythonParityHelper.RampOptions(AgdPath, PixelsPerBin));
                    Assert.Equal(HaulSeg, ComputeHaulSegment(CsResult, Manifest["single_pair_pass"]));
                }

                Byte[] Expected = File.ReadAllBytes(Manifest[pngKey]);
                Byte[] Actual = RenderPngFromCSharp(AgdPath, pngKey, Manifest, CsResult);
                ImageParity.AssertVisualInformationMatch(Expected, Actual, pngKey);
            }
            finally
            {
                TryDelete(AgdPath);
                TryDeleteDirectory(OutDir);
            }
        }

        private static Byte[] RenderPngFromCSharp
            (
            String agdPath,
            String pngKey,
            Dictionary<String, String> manifest,
            HaulBuilderResult? existingResult
            )
        {
            HaulBuilderOptions Options = PythonParityHelper.RampOptions(agdPath, PixelsPerBin);
            HaulBuilderResult Result = existingResult ?? new HaulBuilderEngine().Run(Options);

            if (pngKey == "main")
            {
                Assert.True(Result.PngBytesByName.ContainsKey("main"));
                return Result.PngBytesByName["main"];
            }

            if (pngKey == "pass_pairs")
            {
                Assert.True(Result.PngBytesByName.ContainsKey("cut_fill_pairs"));
                return Result.PngBytesByName["cut_fill_pairs"];
            }

            if (pngKey == "guidance")
            {
                Assert.True(Result.PngBytesByName.ContainsKey("guidance"));
                return Result.PngBytesByName["guidance"];
            }

            if (pngKey == "single_pair")
            {
                Assert.True(manifest.TryGetValue("single_pair_pass", out String? PassText));
                Assert.True(Int32.TryParse(PassText, NumberStyles.Integer, CultureInfo.InvariantCulture, out Int32 PassNo));
                Dictionary<Int32, List<GuidancePoint>> ByPass = new Dictionary<Int32, List<GuidancePoint>>();
                for (Int32 Iter = 0; Iter < Result.GuidancePoints.Count; Iter += 1)
                {
                    GuidancePoint Point = Result.GuidancePoints[Iter];
                    if (!ByPass.TryGetValue(Point.PassNo, out List<GuidancePoint>? Bucket))
                    {
                        Bucket = new List<GuidancePoint>();
                        ByPass[Point.PassNo] = Bucket;
                    }

                    Bucket.Add(Point);
                }

                Assert.True(ByPass.ContainsKey(PassNo));
                return Render.RenderSinglePairPng(
                    Result.Grid!,
                    ByPass[PassNo],
                    Options.BladeWidthMm / 1000.0,
                    PixelsPerBin,
                    showRoute: false);
            }

            throw new ArgumentException($"Unknown png key: {pngKey}", nameof(pngKey));
        }

        private static String ComputeHaulSegment
            (
            HaulBuilderResult result,
            String passText
            )
        {
            Assert.True(Int32.TryParse(passText, NumberStyles.Integer, CultureInfo.InvariantCulture, out Int32 PassNo));
            List<GuidancePoint> Points = result.GuidancePoints.Where(Point => Point.PassNo == PassNo).ToList();
            List<GuidancePoint> CutPts = Points.Where(Point => Point.Action is "START" or "RETURN" or "CUT").ToList();
            List<GuidancePoint> FillPts = Points.Where(Point => Point.Action is "HAUL" or "FILL").ToList();
            Assert.True(CutPts.Count > 0 && FillPts.Count > 0);
            Grid Grid = result.Grid!;
            Int32 Ppb = PixelsPerBin;
            (Double Ax, Double Ay) = WorldToPixel(Grid, CutPts[^1].X, CutPts[^1].Y, Ppb);
            (Double Bx, Double By) = WorldToPixel(Grid, FillPts[0].X, FillPts[0].Y, Ppb);
            return String.Format(
                CultureInfo.InvariantCulture,
                "{0},{1},{2},{3}",
                (Int32)Math.Round(Ax, MidpointRounding.AwayFromZero),
                (Int32)Math.Round(Ay, MidpointRounding.AwayFromZero),
                (Int32)Math.Round(Bx, MidpointRounding.AwayFromZero),
                (Int32)Math.Round(By, MidpointRounding.AwayFromZero));
        }

        private static (Double X, Double Y) WorldToPixel
            (
            Grid grid,
            Double x,
            Double y,
            Int32 ppb
            )
        {
            Double Px = ((x - grid.MinX) / grid.GridSizeM) * ppb;
            Double Py = (grid.Height - ((y - grid.MinY) / grid.GridSizeM)) * ppb;
            return (Px, Py);
        }

        private static Dictionary<String, String> ParseManifest
            (
            String stdout
            )
        {
            Dictionary<String, String> Manifest = new Dictionary<String, String>(StringComparer.Ordinal);
            using StringReader Reader = new StringReader(stdout);
            String? Line;
            while ((Line = Reader.ReadLine()) is not null)
            {
                Int32 Eq = Line.IndexOf('=');
                if (Eq <= 0)
                {
                    continue;
                }

                String Key = Line.Substring(0, Eq);
                String Value = Line.Substring(Eq + 1);
                Manifest[Key] = Value;
            }

            return Manifest;
        }

        private static void TryDelete
            (
            String path
            )
        {
            try
            {
                if (File.Exists(path))
                {
                    File.Delete(path);
                }
            }
            catch (Exception)
            {
                // Best-effort temp cleanup.
            }
        }

        private static void TryDeleteDirectory
            (
            String path
            )
        {
            try
            {
                if (Directory.Exists(path))
                {
                    Directory.Delete(path, true);
                }
            }
            catch (Exception)
            {
                // Best-effort temp cleanup.
            }
        }
    }
}
