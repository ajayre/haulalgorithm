using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// PNG comparison helpers: semantic visual checks for cross-language render parity.
    /// </summary>
    internal static class ImageParity
    {
        /// <summary>
        /// Provides visual profile behavior.
        /// </summary>

        private sealed class VisualProfile
        {
            public Int32 Width { get; init; }
            public Int32 Height { get; init; }
            public Int32 OpaquePixels { get; init; }
            public Int32 CutStripPixels { get; init; }
            public Int32 FillStripPixels { get; init; }
            public Int32 HaulPixels { get; init; }
            public Int32 HeatOverlayPixels { get; init; }
            public Int32 CutFillMapCutPixels { get; init; }
            public Int32 CutFillMapFillPixels { get; init; }
            public Int32 CutStartMarkerPixels { get; init; }
            public Int32 CutEndMarkerPixels { get; init; }
            public Int32 FillStartMarkerPixels { get; init; }
            public Int32 FillEndMarkerPixels { get; init; }
        }

        /// <summary>
        /// Both PNGs decode, have similar size, and carry the same visual roles for algorithm review.
        /// </summary>
        internal static void AssertVisualInformationMatch
            (
            Byte[] ExpectedPng,
            Byte[] ActualPng,
            String PngKey
            )
        {
            Assert.True(ExpectedPng.Length > 100, $"{PngKey}: Python PNG too small");
            Assert.True(ActualPng.Length > 100, $"{PngKey}: C# PNG too small");

            VisualProfile Expected = Analyze(ExpectedPng);
            VisualProfile Actual = Analyze(ActualPng);
            AssertDimensionsSimilar(PngKey, Expected, Actual);

            Int32 MinOpaque = PngKey == "single_pair" ? 200 : 500;
            Assert.True(Expected.OpaquePixels >= MinOpaque, $"{PngKey}: Python PNG has too few opaque pixels");
            Assert.True(Actual.OpaquePixels >= MinOpaque, $"{PngKey}: C# PNG has too few opaque pixels");
            AssertSimilarCount(PngKey, "opaque", Expected.OpaquePixels, Actual.OpaquePixels, 0.35);

            switch (PngKey)
            {
                case "main":
                    AssertRole(PngKey, "cut-fill map (cut)", Expected.CutFillMapCutPixels, Actual.CutFillMapCutPixels, 100);
                    AssertRole(PngKey, "cut-fill map (fill)", Expected.CutFillMapFillPixels, Actual.CutFillMapFillPixels, 100);
                    break;
                case "pass_pairs":
                    AssertRole(PngKey, "heat-map overlay", Expected.HeatOverlayPixels, Actual.HeatOverlayPixels, 20);
                    break;
                case "guidance":
                    AssertRole(PngKey, "cut strip", Expected.CutStripPixels, Actual.CutStripPixels, 50);
                    AssertRole(PngKey, "fill strip", Expected.FillStripPixels, Actual.FillStripPixels, 50);
                    break;
                case "single_pair":
                    AssertRole(PngKey, "cut strip", Expected.CutStripPixels, Actual.CutStripPixels, 30);
                    AssertRole(PngKey, "fill strip", Expected.FillStripPixels, Actual.FillStripPixels, 30);
                    AssertRole(PngKey, "haul leg", Expected.HaulPixels, Actual.HaulPixels, 5);
                    AssertRole(PngKey, "cut-start marker", Expected.CutStartMarkerPixels, Actual.CutStartMarkerPixels, 5);
                    AssertRole(PngKey, "cut-end marker", Expected.CutEndMarkerPixels, Actual.CutEndMarkerPixels, 5);
                    AssertRole(PngKey, "fill-start marker", Expected.FillStartMarkerPixels, Actual.FillStartMarkerPixels, 5);
                    AssertRole(PngKey, "fill-end marker", Expected.FillEndMarkerPixels, Actual.FillEndMarkerPixels, 5);
                    break;
                default:
                    throw new ArgumentException($"Unknown png key: {PngKey}", nameof(PngKey));
            }
        }

        /// <summary>
        /// Executes the analyze operation.
        /// </summary>
        /// <param name="PngBytes">The png bytes used by the operation.</param>
        /// <returns>The resulting analyze.</returns>

        private static VisualProfile Analyze
            (
            Byte[] PngBytes
            )
        {
            using Image<Rgba32> Decoded = SixLabors.ImageSharp.Image.Load<Rgba32>(PngBytes);
            Int32 Opaque = 0;
            Int32 CutStrip = 0;
            Int32 FillStrip = 0;
            Int32 Haul = 0;
            Int32 MapCut = 0;
            Int32 MapFill = 0;
            Int32 Heat = 0;
            Int32 CutStart = 0;
            Int32 CutEnd = 0;
            Int32 FillStart = 0;
            Int32 FillEnd = 0;

            for (Int32 Y = 0; Y < Decoded.Height; Y += 1)
            {
                for (Int32 X = 0; X < Decoded.Width; X += 1)
                {
                    Rgba32 Pixel = Decoded[X, Y];
                    if (Pixel.A < 128)
                    {
                        continue;
                    }

                    Opaque += 1;
                    if (IsCutStrip(Pixel))
                    {
                        CutStrip += 1;
                    }

                    if (IsFillStrip(Pixel))
                    {
                        FillStrip += 1;
                    }

                    if (IsHaulLeg(Pixel) || IsGuidanceHaul(Pixel))
                    {
                        Haul += 1;
                    }

                    if (IsHeatOverlay(Pixel))
                    {
                        Heat += 1;
                    }

                    if (IsCutFillMapCut(Pixel))
                    {
                        MapCut += 1;
                    }

                    if (IsCutFillMapFill(Pixel))
                    {
                        MapFill += 1;
                    }

                    if (IsCutStartMarker(Pixel))
                    {
                        CutStart += 1;
                    }

                    if (IsCutEndMarker(Pixel))
                    {
                        CutEnd += 1;
                    }

                    if (IsFillStartMarker(Pixel))
                    {
                        FillStart += 1;
                    }

                    if (IsFillEndMarker(Pixel))
                    {
                        FillEnd += 1;
                    }
                }
            }

            return new VisualProfile
            {
                Width = Decoded.Width,
                Height = Decoded.Height,
                OpaquePixels = Opaque,
                CutStripPixels = CutStrip,
                FillStripPixels = FillStrip,
                HaulPixels = Haul,
                CutFillMapCutPixels = MapCut,
                CutFillMapFillPixels = MapFill,
                HeatOverlayPixels = Heat,
                CutStartMarkerPixels = CutStart,
                CutEndMarkerPixels = CutEnd,
                FillStartMarkerPixels = FillStart,
                FillEndMarkerPixels = FillEnd,
            };
        }

        /// <summary>
        /// Executes the assert dimensions similar operation.
        /// </summary>
        /// <param name="PngKey">The png key used by the operation.</param>
        /// <param name="Expected">The expected used by the operation.</param>
        /// <param name="Actual">The actual used by the operation.</param>

        private static void AssertDimensionsSimilar
            (
            String PngKey,
            VisualProfile Expected,
            VisualProfile Actual
            )
        {
            Int32 WidthTol = Math.Max(4, (Int32)Math.Round(Expected.Width * 0.08));
            Int32 HeightTol = Math.Max(4, (Int32)Math.Round(Expected.Height * 0.08));
            Assert.True(
                Math.Abs(Expected.Width - Actual.Width) <= WidthTol,
                $"{PngKey}: width Python={Expected.Width} C#={Actual.Width} (tol={WidthTol})");
            Assert.True(
                Math.Abs(Expected.Height - Actual.Height) <= HeightTol,
                $"{PngKey}: height Python={Expected.Height} C#={Actual.Height} (tol={HeightTol})");
        }

        /// <summary>
        /// Executes the assert role operation.
        /// </summary>
        /// <param name="PngKey">The png key used by the operation.</param>
        /// <param name="Role">The role used by the operation.</param>
        /// <param name="ExpectedCount">Count limit applied by the operation.</param>
        /// <param name="ActualCount">Count limit applied by the operation.</param>
        /// <param name="MinimumEach">The minimum each used by the operation.</param>

        private static void AssertRole
            (
            String PngKey,
            String Role,
            Int32 ExpectedCount,
            Int32 ActualCount,
            Int32 MinimumEach
            )
        {
            Assert.True(ExpectedCount >= MinimumEach, $"{PngKey}: Python missing {Role} ({ExpectedCount} px)");
            Assert.True(ActualCount >= MinimumEach, $"{PngKey}: C# missing {Role} ({ActualCount} px)");
            AssertSimilarCount(PngKey, Role, ExpectedCount, ActualCount, 0.40);
        }

        /// <summary>
        /// Executes the assert similar count operation.
        /// </summary>
        /// <param name="PngKey">The png key used by the operation.</param>
        /// <param name="Role">The role used by the operation.</param>
        /// <param name="ExpectedCount">Count limit applied by the operation.</param>
        /// <param name="ActualCount">Count limit applied by the operation.</param>
        /// <param name="MaxRelativeDiff">The max relative diff used by the operation.</param>

        private static void AssertSimilarCount
            (
            String PngKey,
            String Role,
            Int32 ExpectedCount,
            Int32 ActualCount,
            Double MaxRelativeDiff
            )
        {
            Int32 Baseline = Math.Max(ExpectedCount, ActualCount);
            if (Baseline == 0)
            {
                return;
            }

            Double RelativeDiff = Math.Abs(ExpectedCount - ActualCount) / (Double)Baseline;
            Assert.True(
                RelativeDiff <= MaxRelativeDiff,
                $"{PngKey}: {Role} pixel counts differ too much (Python={ExpectedCount}, C#={ActualCount}, diff={RelativeDiff:P0})");
        }

        /// <summary>
        /// Determines whether cut strip.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when cut strip; otherwise false.</returns>

        private static Boolean IsCutStrip
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 200 && Pixel.R > 170 && Pixel.G < 60 && Pixel.B < 60;
        }

        /// <summary>
        /// Determines whether fill strip.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when fill strip; otherwise false.</returns>

        private static Boolean IsFillStrip
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 200 && Pixel.R < 60 && Pixel.G > 60 && Pixel.B > 150;
        }

        /// <summary>
        /// Determines whether haul leg.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when haul leg; otherwise false.</returns>

        private static Boolean IsHaulLeg
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 200 && Pixel.R < 40 && Pixel.G < 40 && Pixel.B < 40;
        }

        /// <summary>
        /// Determines whether guidance haul.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when guidance haul; otherwise false.</returns>

        private static Boolean IsGuidanceHaul
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 60 && Pixel.R > 150 && Pixel.G < 60 && Pixel.B > 150;
        }

        /// <summary>
        /// Determines whether heat overlay.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when heat overlay; otherwise false.</returns>

        private static Boolean IsHeatOverlay
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 100 && Pixel.R > 170 && Pixel.G > 70 && Pixel.B < 130;
        }

        /// <summary>
        /// Determines whether cut fill map cut.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when cut fill map cut; otherwise false.</returns>

        private static Boolean IsCutFillMapCut
            (
            Rgba32 Pixel
            )
        {
            // Negative delta bands: violet through cyan (material to cut).
            return Pixel.A > 200 && Pixel.B > 100 && Pixel.R < 160;
        }

        /// <summary>
        /// Determines whether cut fill map fill.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when cut fill map fill; otherwise false.</returns>

        private static Boolean IsCutFillMapFill
            (
            Rgba32 Pixel
            )
        {
            // Positive delta bands: yellow through dark red (material to fill).
            return Pixel.A > 200 && Pixel.R > 150 && Pixel.B < 140 && Pixel.G < 220;
        }

        /// <summary>
        /// Determines whether cut start marker.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when cut start marker; otherwise false.</returns>

        private static Boolean IsCutStartMarker
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 200 && Pixel.G > 150 && Pixel.R < 80 && Pixel.B < 80;
        }

        /// <summary>
        /// Determines whether cut end marker.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when cut end marker; otherwise false.</returns>

        private static Boolean IsCutEndMarker
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 200 && Pixel.R > 200 && Pixel.G > 100 && Pixel.G < 180 && Pixel.B < 80;
        }

        /// <summary>
        /// Determines whether fill start marker.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when fill start marker; otherwise false.</returns>

        private static Boolean IsFillStartMarker
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 200 && Pixel.G > 180 && Pixel.B > 180 && Pixel.R < 80;
        }

        /// <summary>
        /// Determines whether fill end marker.
        /// </summary>
        /// <param name="Pixel">The pixel used by the operation.</param>
        /// <returns>True when fill end marker; otherwise false.</returns>

        private static Boolean IsFillEndMarker
            (
            Rgba32 Pixel
            )
        {
            return Pixel.A > 200 && Pixel.R > 120 && Pixel.R < 200 && Pixel.G < 60 && Pixel.B > 150;
        }
    }
}
