using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// PNG comparison helpers: semantic visual checks for cross-language render parity.
    /// </summary>
    internal static class ImageParity
    {
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
            Byte[] expected,
            Byte[] actual,
            String pngKey
            )
        {
            Assert.True(expected.Length > 100, $"{pngKey}: Python PNG too small");
            Assert.True(actual.Length > 100, $"{pngKey}: C# PNG too small");

            VisualProfile Expected = Analyze(expected);
            VisualProfile Actual = Analyze(actual);
            AssertDimensionsSimilar(pngKey, Expected, Actual);

            Int32 MinOpaque = pngKey == "single_pair" ? 200 : 500;
            Assert.True(Expected.OpaquePixels >= MinOpaque, $"{pngKey}: Python PNG has too few opaque pixels");
            Assert.True(Actual.OpaquePixels >= MinOpaque, $"{pngKey}: C# PNG has too few opaque pixels");
            AssertSimilarCount(pngKey, "opaque", Expected.OpaquePixels, Actual.OpaquePixels, 0.35);

            switch (pngKey)
            {
                case "main":
                    AssertRole(pngKey, "cut-fill map (cut)", Expected.CutFillMapCutPixels, Actual.CutFillMapCutPixels, 100);
                    AssertRole(pngKey, "cut-fill map (fill)", Expected.CutFillMapFillPixels, Actual.CutFillMapFillPixels, 100);
                    break;
                case "pass_pairs":
                    AssertRole(pngKey, "heat-map overlay", Expected.HeatOverlayPixels, Actual.HeatOverlayPixels, 20);
                    break;
                case "guidance":
                    AssertRole(pngKey, "cut strip", Expected.CutStripPixels, Actual.CutStripPixels, 50);
                    AssertRole(pngKey, "fill strip", Expected.FillStripPixels, Actual.FillStripPixels, 50);
                    break;
                case "single_pair":
                    AssertRole(pngKey, "cut strip", Expected.CutStripPixels, Actual.CutStripPixels, 30);
                    AssertRole(pngKey, "fill strip", Expected.FillStripPixels, Actual.FillStripPixels, 30);
                    AssertRole(pngKey, "haul leg", Expected.HaulPixels, Actual.HaulPixels, 5);
                    AssertRole(pngKey, "cut-start marker", Expected.CutStartMarkerPixels, Actual.CutStartMarkerPixels, 5);
                    AssertRole(pngKey, "cut-end marker", Expected.CutEndMarkerPixels, Actual.CutEndMarkerPixels, 5);
                    AssertRole(pngKey, "fill-start marker", Expected.FillStartMarkerPixels, Actual.FillStartMarkerPixels, 5);
                    AssertRole(pngKey, "fill-end marker", Expected.FillEndMarkerPixels, Actual.FillEndMarkerPixels, 5);
                    break;
                default:
                    throw new ArgumentException($"Unknown png key: {pngKey}", nameof(pngKey));
            }
        }

        private static VisualProfile Analyze
            (
            Byte[] pngBytes
            )
        {
            using Image<Rgba32> Decoded = SixLabors.ImageSharp.Image.Load<Rgba32>(pngBytes);
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

        private static void AssertDimensionsSimilar
            (
            String pngKey,
            VisualProfile expected,
            VisualProfile actual
            )
        {
            Int32 WidthTol = Math.Max(4, (Int32)Math.Round(expected.Width * 0.08));
            Int32 HeightTol = Math.Max(4, (Int32)Math.Round(expected.Height * 0.08));
            Assert.True(
                Math.Abs(expected.Width - actual.Width) <= WidthTol,
                $"{pngKey}: width Python={expected.Width} C#={actual.Width} (tol={WidthTol})");
            Assert.True(
                Math.Abs(expected.Height - actual.Height) <= HeightTol,
                $"{pngKey}: height Python={expected.Height} C#={actual.Height} (tol={HeightTol})");
        }

        private static void AssertRole
            (
            String pngKey,
            String role,
            Int32 expectedCount,
            Int32 actualCount,
            Int32 minimumEach
            )
        {
            Assert.True(expectedCount >= minimumEach, $"{pngKey}: Python missing {role} ({expectedCount} px)");
            Assert.True(actualCount >= minimumEach, $"{pngKey}: C# missing {role} ({actualCount} px)");
            AssertSimilarCount(pngKey, role, expectedCount, actualCount, 0.40);
        }

        private static void AssertSimilarCount
            (
            String pngKey,
            String role,
            Int32 expectedCount,
            Int32 actualCount,
            Double maxRelativeDiff
            )
        {
            Int32 Baseline = Math.Max(expectedCount, actualCount);
            if (Baseline == 0)
            {
                return;
            }

            Double RelativeDiff = Math.Abs(expectedCount - actualCount) / (Double)Baseline;
            Assert.True(
                RelativeDiff <= maxRelativeDiff,
                $"{pngKey}: {role} pixel counts differ too much (Python={expectedCount}, C#={actualCount}, diff={RelativeDiff:P0})");
        }

        private static Boolean IsCutStrip
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 200 && pixel.R > 170 && pixel.G < 60 && pixel.B < 60;
        }

        private static Boolean IsFillStrip
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 200 && pixel.R < 60 && pixel.G > 60 && pixel.B > 150;
        }

        private static Boolean IsHaulLeg
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 200 && pixel.R < 40 && pixel.G < 40 && pixel.B < 40;
        }

        private static Boolean IsGuidanceHaul
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 60 && pixel.R > 150 && pixel.G < 60 && pixel.B > 150;
        }

        private static Boolean IsHeatOverlay
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 100 && pixel.R > 170 && pixel.G > 70 && pixel.B < 130;
        }

        private static Boolean IsCutFillMapCut
            (
            Rgba32 pixel
            )
        {
            // Negative delta bands: violet through cyan (material to cut).
            return pixel.A > 200 && pixel.B > 100 && pixel.R < 160;
        }

        private static Boolean IsCutFillMapFill
            (
            Rgba32 pixel
            )
        {
            // Positive delta bands: yellow through dark red (material to fill).
            return pixel.A > 200 && pixel.R > 150 && pixel.B < 140 && pixel.G < 220;
        }

        private static Boolean IsCutStartMarker
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 200 && pixel.G > 150 && pixel.R < 80 && pixel.B < 80;
        }

        private static Boolean IsCutEndMarker
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 200 && pixel.R > 200 && pixel.G > 100 && pixel.G < 180 && pixel.B < 80;
        }

        private static Boolean IsFillStartMarker
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 200 && pixel.G > 180 && pixel.B > 180 && pixel.R < 80;
        }

        private static Boolean IsFillEndMarker
            (
            Rgba32 pixel
            )
        {
            return pixel.A > 200 && pixel.R > 120 && pixel.R < 200 && pixel.G < 60 && pixel.B > 150;
        }
    }
}
