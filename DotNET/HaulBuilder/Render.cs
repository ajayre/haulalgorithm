using System.Globalization;
using System.Linq;

using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Png;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

namespace HaulBuilder
{
    /// <summary>
    /// PNG render helpers ported from <c>landlevel.render</c> using ImageSharp.
    /// </summary>
    public static class Render
    {
        private static readonly Rgba32 VIOLET = new Rgba32(0x80, 0x00, 0x80, 0xFF);
        private static readonly Rgba32 INDIGO = new Rgba32(0x9A, 0x31, 0xFF, 0xFF);
        private static readonly Rgba32 BLUE = new Rgba32(0x00, 0x66, 0xFF, 0xFF);
        private static readonly Rgba32 CYAN = new Rgba32(0x36, 0xF5, 0xF5, 0xFF);
        private static readonly Rgba32 GREEN = new Rgba32(0x00, 0xCD, 0x00, 0xFF);
        private static readonly Rgba32 YELLOW = new Rgba32(0xFF, 0xFF, 0x00, 0xFF);
        private static readonly Rgba32 ORANGE = new Rgba32(0xFF, 0x80, 0x00, 0xFF);
        private static readonly Rgba32 RED = new Rgba32(0xFF, 0x00, 0x00, 0xFF);
        private static readonly Rgba32 DARKRED = new Rgba32(0xB4, 0x00, 0x00, 0xFF);

        /// <summary>
        /// Maps <c>existing-target</c> delta to the nine-band cut/fill palette.
        /// </summary>
        /// <param name="diff">Delta in metres.</param>
        /// <returns>Palette colour.</returns>
        static public Rgba32 CutFillColor
            (
            Double diff
            )
        {
            // Thresholds are copied exactly from Python so visualization is parity-safe.
            if (diff < -0.80) return VIOLET;
            if (diff < -0.50) return INDIGO;
            if (diff < -0.05) return BLUE;
            if (diff < -0.01) return CYAN;
            if (diff <= 0.01) return GREEN;
            if (diff <= 0.05) return YELLOW;
            if (diff <= 0.50) return ORANGE;
            if (diff <= 0.80) return RED;
            return DARKRED;
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Rgba32 cutfill_color
            (
            Double diff
            ) => CutFillColor(diff);

        private static readonly Rgba32[] CutFillPalette =
        {
            VIOLET, INDIGO, BLUE, CYAN, GREEN, YELLOW, ORANGE, RED, DARKRED,
        };

        /// <summary>
        /// Vectorized cut/fill image render from a diff grid.
        /// </summary>
        /// <param name="diff">Diff grid (<c>existing-target</c>).</param>
        /// <param name="valid">Validity mask.</param>
        /// <param name="pixelsPerBin">Pixels per cell edge.</param>
        /// <returns>Rendered image with Y flipped to image space.</returns>
        static public Image<Rgba32> DiffToImage
            (
            Double[,] diff,
            Boolean[,] valid,
            Int32 pixelsPerBin
            )
        {
            // Rendering the base at one pixel per cell keeps the raster logic simple, then upscale nearest-neighbor.
            Int32 Height = diff.GetLength(0);
            Int32 Width = diff.GetLength(1);
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);

            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Int32 Y = (Height - 1 - By);
                    if (!valid[By, Bx] || Double.IsNaN(diff[By, Bx]) || Double.IsInfinity(diff[By, Bx]))
                    {
                        Image[Bx, Y] = new Rgba32(0, 0, 0, 0);
                        continue;
                    }

                    Image[Bx, Y] = CutFillColor(diff[By, Bx]);
                }
            }

            if (pixelsPerBin > 1)
            {
                Image.Mutate(Action => Action.Resize(Width * pixelsPerBin, Height * pixelsPerBin, KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        /// <summary>
        /// Cut/fill image using numpy-style <c>digitize</c> bands (Python <c>diff_to_image</c>).
        /// </summary>
        static public Image<Rgba32> DiffToImageDigitize
            (
            Double[,] diff,
            Boolean[,] valid,
            Int32 pixelsPerBin
            )
        {
            Int32 Height = diff.GetLength(0);
            Int32 Width = diff.GetLength(1);
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);

            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Int32 Y = (Height - 1 - By);
                    Double Value = valid[By, Bx] && !Double.IsNaN(diff[By, Bx]) && !Double.IsInfinity(diff[By, Bx])
                        ? diff[By, Bx]
                        : 1e9;
                    Rgba32 Color = CutFillPalette[DigitizeBand(Value)];
                    Byte Alpha = (valid[By, Bx] && !Double.IsNaN(diff[By, Bx]) && !Double.IsInfinity(diff[By, Bx]))
                        ? (Byte)255
                        : (Byte)0;
                    Image[Bx, Y] = new Rgba32(Color.R, Color.G, Color.B, Alpha);
                }
            }

            if (pixelsPerBin > 1)
            {
                Image.Mutate(Action => Action.Resize(Width * pixelsPerBin, Height * pixelsPerBin, KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        private static Int32 DigitizeBand
            (
            Double diff
            )
        {
            // Matches np.digitize(..., _BAND_EDGES) with default right=True.
            if (diff <= -0.80) return 0;
            if (diff <= -0.50) return 1;
            if (diff <= -0.05) return 2;
            if (diff <= -0.01) return 3;
            if (diff <= 0.01) return 4;
            if (diff <= 0.05) return 5;
            if (diff <= 0.50) return 6;
            if (diff <= 0.80) return 7;
            return 8;
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Image<Rgba32> diff_to_image
            (
            Double[,] diff,
            Boolean[,] valid,
            Int32 ppb
            ) => DiffToImageDigitize(diff, valid, ppb);

        /// <summary>
        /// Terrain-ramp elevation render for topology animations.
        /// </summary>
        /// <param name="elev">Elevation grid in metres.</param>
        /// <param name="valid">Validity mask.</param>
        /// <param name="pixelsPerBin">Pixels per cell edge.</param>
        /// <param name="vmin">Fixed scale low value.</param>
        /// <param name="vmax">Fixed scale high value.</param>
        /// <returns>Rendered terrain image with Y flipped to image space.</returns>
        static public Image<Rgba32> ElevationToImage
            (
            Double[,] elev,
            Boolean[,] valid,
            Int32 pixelsPerBin,
            Double vmin,
            Double vmax
            )
        {
            // A fixed range across frames shows flattening over time instead of re-normalizing each image.
            Int32 Height = elev.GetLength(0);
            Int32 Width = elev.GetLength(1);
            Double Range = Math.Abs(vmax - vmin) > 1e-12 ? (vmax - vmin) : 1.0;
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);

            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Int32 Y = (Height - 1 - By);
                    if (!valid[By, Bx])
                    {
                        Image[Bx, Y] = new Rgba32(0, 0, 0, 0);
                        continue;
                    }

                    Double Elev = elev[By, Bx];
                    if (Double.IsNaN(Elev) || Double.IsInfinity(Elev))
                    {
                        Elev = vmin;
                    }

                    Double T = Math.Max(0.0, Math.Min(1.0, (Elev - vmin) / Range));
                    Rgba32 C = TerrainColor(T);
                    Image[Bx, Y] = C;
                }
            }

            if (pixelsPerBin > 1)
            {
                Image.Mutate(Action => Action.Resize(Width * pixelsPerBin, Height * pixelsPerBin, KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Image<Rgba32> elevation_to_image
            (
            Double[,] elev,
            Boolean[,] valid,
            Int32 ppb,
            Double vmin,
            Double vmax
            ) => ElevationToImage(elev, valid, ppb, vmin, vmax);

        /// <summary>
        /// Renders the cut/fill map with optional flow arrows and returns PNG bytes.
        /// </summary>
        /// <param name="grid">Grid source model.</param>
        /// <param name="flow">Optional flow field.</param>
        /// <param name="pixelsPerBin">Pixels per grid cell edge.</param>
        /// <param name="arrowStride">Arrow stride in cells.</param>
        /// <returns>PNG bytes.</returns>
        static public Byte[] RenderPng
            (
            Grid grid,
            FlowField? flow,
            Int32 pixelsPerBin = 50,
            Int32 arrowStride = 2
            )
        {
            // Render starts from the same diff source used by Python to keep palette parity.
            using Image<Rgba32> Image = DiffToImage(grid.Delta(), grid.Valid, Math.Max(1, pixelsPerBin));
            if ((flow is not null) && (arrowStride > 0))
            {
                DrawFlowArrows(Image, grid, flow, Math.Max(1, pixelsPerBin), arrowStride);
            }

            DrawExclusions(Image, grid, Math.Max(1, pixelsPerBin));
            return ToPng(Image);
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Byte[] render_png
            (
            Grid grid,
            FlowField? flow,
            Int32 pixels_per_bin = 50,
            Int32 arrow_stride = 2
            ) => RenderPng(grid, flow, pixels_per_bin, arrow_stride);

        /// <summary>
        /// Renders cut/fill base plus one weighted line per cut/fill pair.
        /// </summary>
        static public Byte[] RenderPassPairsPng
            (
            Grid grid,
            IReadOnlyList<Pass> passes,
            Int32 pixelsPerBin = 50,
            Double baseDim = 0.28
            )
        {
            // Heat accumulation highlights repeated haul corridors (Python render_pass_pairs_png).
            Int32 Ppb = Math.Max(1, Math.Min(pixelsPerBin, 16));
            using Image<Rgba32> Base = DiffToImageDigitize(grid.Delta(), grid.Valid, Ppb);
            DimTowardsWhite(Base, baseDim);
            Int32 W = Base.Width;
            Int32 H = Base.Height;
            Single[,] Acc = new Single[H, W];

            Int32 StampRadius = Math.Max(1, (Int32)Math.Round(Ppb / 6.0, MidpointRounding.AwayFromZero));
            List<(Int32 Dx, Int32 Dy)> Offsets = new List<(Int32 Dx, Int32 Dy)>();
            for (Int32 Dy = -StampRadius; Dy <= StampRadius; Dy += 1)
            {
                for (Int32 Dx = -StampRadius; Dx <= StampRadius; Dx += 1)
                {
                    if (((Dx * Dx) + (Dy * Dy)) <= (StampRadius * StampRadius))
                    {
                        Offsets.Add((Dx, Dy));
                    }
                }
            }

            for (Int32 Iter = 0; Iter < passes.Count; Iter += 1)
            {
                Pass Pass = passes[Iter];
                (Double X0, Double Y0) = WorldToPixel(grid, Pass.CutEndXy.X, Pass.CutEndXy.Y, Ppb);
                (Double X1, Double Y1) = WorldToPixel(grid, Pass.FillStartXy.X, Pass.FillStartXy.Y, Ppb);
                Double Dist = Math.Sqrt(((X1 - X0) * (X1 - X0)) + ((Y1 - Y0) * (Y1 - Y0)));
                Int32 Steps = Math.Max(2, (Int32)Math.Floor(Dist) + 1);
                HashSet<Int32> LinePixels = new HashSet<Int32>();
                for (Int32 Step = 0; Step < Steps; Step += 1)
                {
                    Double T = (Steps <= 1) ? 0.0 : ((Double)Step / (Steps - 1));
                    Int32 Xs = (Int32)Math.Round(X0 + ((X1 - X0) * T), MidpointRounding.AwayFromZero);
                    Int32 Ys = (Int32)Math.Round(Y0 + ((Y1 - Y0) * T), MidpointRounding.AwayFromZero);
                    for (Int32 Off = 0; Off < Offsets.Count; Off += 1)
                    {
                        (Int32 Dx, Int32 Dy) = Offsets[Off];
                        Int32 Px = Xs + Dx;
                        Int32 Py = Ys + Dy;
                        if ((Px < 0) || (Py < 0) || (Px >= W) || (Py >= H))
                        {
                            continue;
                        }

                        LinePixels.Add((Py * W) + Px);
                    }
                }

                foreach (Int32 Lin in LinePixels)
                {
                    Acc[Lin / W, Lin % W] += 1.0f;
                }
            }

            List<Single> Positive = new List<Single>();
            for (Int32 Y = 0; Y < H; Y += 1)
            {
                for (Int32 X = 0; X < W; X += 1)
                {
                    if (Acc[Y, X] > 0.0f)
                    {
                        Positive.Add(Acc[Y, X]);
                    }
                }
            }

            Single Scale = 1.0f;
            if (Positive.Count > 0)
            {
                Positive.Sort();
                Scale = Math.Max(PercentileLinear(Positive, 0.9), 1.0f);
            }

            for (Int32 Y = 0; Y < H; Y += 1)
            {
                for (Int32 X = 0; X < W; X += 1)
                {
                    if (Acc[Y, X] <= 0.0f)
                    {
                        continue;
                    }

                    Single T = (Single)Math.Sqrt(Math.Max(0.0, Math.Min(1.0, Acc[Y, X] / Scale)));
                    Rgba32 Heat = HeatMapColor(T);
                    Byte Alpha = TruncByte(Math.Max(0.0, Math.Min(255.0, 150.0 + (T * 105.0))));
                    AlphaCompositePixel(Base, X, Y, Heat, Alpha);
                }
            }

            DrawExclusions(Base, grid, Ppb);
            return ToPng(Base);
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Byte[] render_pass_pairs_png
            (
            Grid grid,
            IReadOnlyList<Pass> passes,
            Int32 pixels_per_bin = 50
            ) => RenderPassPairsPng(grid, passes, pixels_per_bin);

        /// <summary>
        /// Renders guidance track over a faded cut/fill map.
        /// </summary>
        static public Byte[] RenderGuidancePng
            (
            Grid grid,
            IReadOnlyList<GuidancePoint> points,
            Int32 pixelsPerBin = 50,
            Double bladeWidthM = 2.271
            )
        {
            // Action-colored guidance confirms cut/fill/haul sequencing at blade scale.
            Int32 Ppb = Math.Max(1, Math.Min(pixelsPerBin, 16));
            using Image<Rgba32> Base = DiffToImageDigitize(grid.Delta(), grid.Valid, Ppb);
            DimTowardsWhite(Base, 0.30);
            Int32 BladePx = Math.Max(1, (Int32)Math.Round((bladeWidthM / grid.GridSizeM) * Ppb, MidpointRounding.AwayFromZero));

            for (Int32 Iter = 1; Iter < points.Count; Iter += 1)
            {
                GuidancePoint Prev = points[Iter - 1];
                GuidancePoint Cur = points[Iter];
                (Double X0, Double Y0) = WorldToPixel(grid, Prev.X, Prev.Y, Ppb);
                (Double X1, Double Y1) = WorldToPixel(grid, Cur.X, Cur.Y, Ppb);
                String Action = Cur.Action;
                Rgba32 Color = GuidanceColor(Action);
                Int32 Width = (Action == "CUT" || Action == "FILL" || Action == "GRADE") ? BladePx : 1;
                DrawLine(
                    Base,
                    Math.Round(X0, MidpointRounding.AwayFromZero),
                    Math.Round(Y0, MidpointRounding.AwayFromZero),
                    Math.Round(X1, MidpointRounding.AwayFromZero),
                    Math.Round(Y1, MidpointRounding.AwayFromZero),
                    Color,
                    Width);
            }

            DrawExclusions(Base, grid, Ppb);
            return ToPng(Base);
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Byte[] render_guidance_png
            (
            Grid grid,
            IReadOnlyList<GuidancePoint> points,
            Int32 pixels_per_bin = 50,
            Double blade_width_m = 2.271
            ) => RenderGuidancePng(grid, points, pixels_per_bin, blade_width_m);

        /// <summary>
        /// Renders one pass guidance pair and returns cropped PNG bytes.
        /// Thick red = blade-down CUT only; thick blue = blade-down FILL only.
        /// </summary>
        static public Byte[] RenderSinglePairPng
            (
            Grid grid,
            IReadOnlyList<GuidancePoint> points,
            Double bladeWidthM = 2.271,
            Int32 pixelsPerBin = 26,
            Int32 padCells = 4,
            Boolean showRoute = true
            )
        {
            // Single-pair render: thick red = blade-down CUT only, thick blue = blade-down FILL only;
            // HAUL/MOVE/START/RETURN stay on the thin GPS route so cut/fill overlays match the grade map.
            Int32 Ppb = Math.Max(1, pixelsPerBin);
            using Image<Rgba32> Base = DiffToImageDigitize(grid.Delta(), grid.Valid, Ppb);
            DimTowardsWhite(Base, 0.45);

            Int32 BladePx = Math.Max(3, (Int32)Math.Round((bladeWidthM / grid.GridSizeM) * Ppb, MidpointRounding.AwayFromZero));
            List<GuidancePoint> CutPts = new List<GuidancePoint>();
            List<GuidancePoint> FillPts = new List<GuidancePoint>();
            for (Int32 Iter = 0; Iter < points.Count; Iter += 1)
            {
                GuidancePoint Point = points[Iter];
                if (Point.Action == "CUT")
                {
                    CutPts.Add(Point);
                }

                if (Point.Action == "FILL")
                {
                    FillPts.Add(Point);
                }
            }

            if (CutPts.Count >= 2)
            {
                DrawPolyline(Base, grid, CutPts, Ppb, new Rgba32(200, 0, 0, 255), BladePx);
            }

            if (FillPts.Count >= 2)
            {
                DrawPolyline(Base, grid, FillPts, Ppb, new Rgba32(0, 90, 220, 255), BladePx);
            }

            if ((CutPts.Count > 0) && (FillPts.Count > 0))
            {
                (Double Ax, Double Ay) = WorldToPixel(grid, CutPts[^1].X, CutPts[^1].Y, Ppb);
                (Double Bx, Double By) = WorldToPixel(grid, FillPts[0].X, FillPts[0].Y, Ppb);
                Int32 HaulWidth = Math.Max(2, BladePx / 2);
                DrawLine(Base, Ax, Ay, Bx, By, new Rgba32(0, 0, 0, 255), HaulWidth);
                Double Ang = Math.Atan2(By - Ay, Bx - Ax);
                Double Ah = Math.Max(8.0, BladePx * 1.6);
                foreach (Double Da in new Double[] { (150.0 * Math.PI / 180.0), (-150.0 * Math.PI / 180.0) })
                {
                    DrawLine(
                        Base,
                        Bx,
                        By,
                        (Bx + (Ah * Math.Cos(Ang + Da))),
                        (By + (Ah * Math.Sin(Ang + Da))),
                        new Rgba32(0, 0, 0, 255),
                        HaulWidth);
                }
            }

            if (showRoute && points.Count >= 2)
            {
                List<(Double X, Double Y)> RouteXy = new List<(Double X, Double Y)>();
                for (Int32 Iter = 0; Iter < points.Count; Iter += 1)
                {
                    RouteXy.Add(WorldToPixel(grid, points[Iter].X, points[Iter].Y, Ppb));
                }

                Int32 RouteWidth = Math.Max(1, BladePx / 8);
                DrawPolylinePixels(Base, RouteXy, new Rgba32(35, 35, 35, 255), RouteWidth);
                Int32 DotRadius = Math.Max(2, BladePx / 6);
                Double StepPx = Math.Max(1.0, 2.0 * Ppb);
                Double Acc = StepPx;
                (Double X, Double Y) Prev = RouteXy[0];
                DrawCircle(Base, Prev.X, Prev.Y, DotRadius, new Rgba32(255, 235, 0, 255), new Rgba32(35, 35, 35, 255));
                for (Int32 Iter = 1; Iter < RouteXy.Count; Iter += 1)
                {
                    (Double CurX, Double CurY) = RouteXy[Iter];
                    Double Dist = Math.Sqrt(((CurX - Prev.X) * (CurX - Prev.X)) + ((CurY - Prev.Y) * (CurY - Prev.Y)));
                    Acc += Dist;
                    if (Acc >= StepPx)
                    {
                        Acc = 0.0;
                        DrawCircle(Base, CurX, CurY, DotRadius, new Rgba32(255, 235, 0, 255), new Rgba32(35, 35, 35, 255));
                    }

                    Prev = (CurX, CurY);
                }
            }

            Int32 MarkerRadius = Math.Max(5, BladePx);
            // Green/orange mark true CUT start/end (blade-down only).
            if (CutPts.Count > 0)
            {
                DotGuidancePoint(Base, grid, CutPts[0], Ppb, new Rgba32(0, 200, 0, 255), MarkerRadius);
                DotGuidancePoint(Base, grid, CutPts[^1], Ppb, new Rgba32(255, 140, 0, 255), MarkerRadius);
            }

            // Cyan/purple mark true FILL start/end (not HAUL transit).
            if (FillPts.Count > 0)
            {
                DotGuidancePoint(Base, grid, FillPts[0], Ppb, new Rgba32(0, 220, 255, 255), MarkerRadius);
                DotGuidancePoint(Base, grid, FillPts[^1], Ppb, new Rgba32(160, 0, 200, 255), MarkerRadius);
            }

            List<Double> Xs = new List<Double>();
            List<Double> Ys = new List<Double>();
            for (Int32 Iter = 0; Iter < points.Count; Iter += 1)
            {
                (Double Px, Double Py) = WorldToPixel(grid, points[Iter].X, points[Iter].Y, Ppb);
                Xs.Add(Px);
                Ys.Add(Py);
            }

            Int32 Pad = padCells * Ppb;
            Int32 X0 = Math.Max(0, TruncTowardZero(Xs.Min() - Pad));
            Int32 Y0 = Math.Max(0, TruncTowardZero(Ys.Min() - Pad));
            Int32 X1 = Math.Min(Base.Width, TruncTowardZero(Xs.Max() + Pad));
            Int32 Y1 = Math.Min(Base.Height, TruncTowardZero(Ys.Max() + Pad));
            if ((X1 <= X0) || (Y1 <= Y0))
            {
                return ToPng(Base);
            }

            Base.Mutate(Context => Context.Crop(new Rectangle(X0, Y0, X1 - X0, Y1 - Y0)));
            return ToPng(Base);
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Byte[] render_single_pair_png
            (
            Grid grid,
            IReadOnlyList<GuidancePoint> pts,
            Int32 pixels_per_bin = 26
            ) => RenderSinglePairPng(grid, pts, 2.271, pixels_per_bin);

        /// <summary>
        /// Renders explicitly separated coverage runs for diagnostic comparison.
        /// </summary>
        /// <param name="grid">Grid geometry and grade surface.</param>
        /// <param name="cutRuns">Independent straight cut polylines.</param>
        /// <param name="fillRuns">Independent straight fill polylines.</param>
        /// <param name="routePoints">Complete thin diagnostic route.</param>
        /// <param name="bladeWidthM">Displayed blade width in metres.</param>
        /// <param name="pixelsPerBin">Pixels per grid bin.</param>
        /// <param name="padCells">Crop padding in grid cells.</param>
        /// <param name="manualAssistRuns">Explicit operator-alignment polylines.</param>
        /// <param name="routeMarkers">Optional numbered route marker locations.</param>
        /// <param name="drawDirectionArrows">Whether to draw arrowheads along the complete route.</param>
        /// <param name="directionArrowSpacingM">Distance between route arrowheads in metres.</param>
        /// <param name="boundsPoints">Optional non-drawn points used to keep animation crops stable.</param>
        /// <returns>Cropped PNG bytes.</returns>
        static public Byte[] RenderCoverageRunsPng
            (
            Grid grid,
            IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> cutRuns,
            IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> fillRuns,
            IReadOnlyList<(Double X, Double Y)> routePoints,
            Double bladeWidthM = 2.271,
            Int32 pixelsPerBin = 26,
            Int32 padCells = 4,
            IReadOnlyList<IReadOnlyList<(Double X, Double Y)>>? manualAssistRuns = null,
            IReadOnlyList<((Double X, Double Y) Point, String Label)>? routeMarkers = null,
            Boolean drawDirectionArrows = false,
            Double directionArrowSpacingM = 0.0,
            IReadOnlyList<(Double X, Double Y)>? boundsPoints = null
            )
        {
            // Draw each work run independently so blade-up repositioning never becomes a thick work overlay.
            Int32 Ppb = Math.Max(1, pixelsPerBin);
            using Image<Rgba32> Base = DiffToImageDigitize(grid.Delta(), grid.Valid, Ppb);
            for (Int32 Y = 0; Y < Base.Height; Y += 1)
            {
                for (Int32 X = 0; X < Base.Width; X += 1)
                {
                    if (Base[X, Y].A == 0)
                    {
                        Base[X, Y] = new Rgba32(238, 238, 238, 255);
                    }
                }
            }
            DimTowardsWhite(Base, 0.45);
            Int32 BladePx = Math.Max(
                3,
                (Int32)Math.Round(
                    (bladeWidthM / grid.GridSizeM) * Ppb,
                    MidpointRounding.AwayFromZero));

            foreach (IReadOnlyList<(Double X, Double Y)> Run in cutRuns)
            {
                DrawWorldPolyline(
                    Base, grid, Run, Ppb, new Rgba32(200, 0, 0, 255), BladePx);
            }
            foreach (IReadOnlyList<(Double X, Double Y)> Run in fillRuns)
            {
                DrawWorldPolyline(
                    Base, grid, Run, Ppb, new Rgba32(0, 90, 220, 255), BladePx);
            }
            if (routePoints.Count >= 2)
            {
                Int32 RouteWidth = Math.Max(1, BladePx / 8);
                DrawWorldPolyline(
                    Base, grid, routePoints, Ppb, new Rgba32(35, 35, 35, 255), RouteWidth);
            }
            if (manualAssistRuns != null)
            {
                Int32 AssistWidth = Math.Max(2, BladePx / 5);
                foreach (IReadOnlyList<(Double X, Double Y)> Run in manualAssistRuns)
                {
                    DrawWorldPolyline(
                        Base, grid, Run, Ppb, new Rgba32(255, 140, 0, 255), AssistWidth);
                }
            }
            if (drawDirectionArrows && (routePoints.Count >= 2))
            {
                Double SpacingM = directionArrowSpacingM > 0.0
                    ? directionArrowSpacingM
                    : 6.0 * grid.GridSizeM;
                DrawRouteDirectionArrows(Base, grid, routePoints, Ppb, SpacingM);
            }
            if (routeMarkers != null)
            {
                Int32 MarkerRadius = Math.Max(8, BladePx / 2);
                foreach (((Double X, Double Y) Point, String Label) Marker in routeMarkers)
                {
                    (Double Px, Double Py) = WorldToPixel(
                        grid, Marker.Point.X, Marker.Point.Y, Ppb);
                    DrawCircle(
                        Base, Px, Py, MarkerRadius,
                        new Rgba32(255, 255, 255, 235),
                        new Rgba32(0, 0, 0, 255));
                    DrawTextDigit(
                        Base,
                        (Int32)Math.Round(Px),
                        (Int32)Math.Round(Py),
                        Marker.Label);
                }
            }

            List<(Double X, Double Y)> Bounds = new List<(Double X, Double Y)>(routePoints);
            foreach (IReadOnlyList<(Double X, Double Y)> Run in cutRuns)
            {
                Bounds.AddRange(Run);
            }
            foreach (IReadOnlyList<(Double X, Double Y)> Run in fillRuns)
            {
                Bounds.AddRange(Run);
            }
            if (manualAssistRuns != null)
            {
                foreach (IReadOnlyList<(Double X, Double Y)> Run in manualAssistRuns)
                {
                    Bounds.AddRange(Run);
                }
            }
            if (routeMarkers != null)
            {
                Bounds.AddRange(routeMarkers.Select(Marker => Marker.Point));
            }
            if (boundsPoints != null)
            {
                Bounds.AddRange(boundsPoints);
            }
            if (Bounds.Count == 0) return ToPng(Base);

            List<(Double X, Double Y)> Pixels = Bounds.Select(Point =>
                WorldToPixel(grid, Point.X, Point.Y, Ppb)).ToList();
            Int32 Pad = padCells * Ppb;
            Int32 X0 = Math.Max(0, TruncTowardZero(Pixels.Min(Point => Point.X) - Pad));
            Int32 Y0 = Math.Max(0, TruncTowardZero(Pixels.Min(Point => Point.Y) - Pad));
            Int32 X1 = Math.Min(Base.Width, TruncTowardZero(Pixels.Max(Point => Point.X) + Pad));
            Int32 Y1 = Math.Min(Base.Height, TruncTowardZero(Pixels.Max(Point => Point.Y) + Pad));
            if ((X1 > X0) && (Y1 > Y0))
            {
                Base.Mutate(Context => Context.Crop(new Rectangle(X0, Y0, X1 - X0, Y1 - Y0)));
            }
            return ToPng(Base);
        }

        /// <summary>
        /// Converts a world polyline to pixels and draws it at a fixed width.
        /// </summary>
        /// <param name="image">Destination image.</param>
        /// <param name="grid">Grid projection.</param>
        /// <param name="points">World polyline.</param>
        /// <param name="pixelsPerBin">Pixels per grid bin.</param>
        /// <param name="color">Line color.</param>
        /// <param name="width">Line width in pixels.</param>
        private static void DrawWorldPolyline
            (
            Image<Rgba32> image,
            Grid grid,
            IReadOnlyList<(Double X, Double Y)> points,
            Int32 pixelsPerBin,
            Rgba32 color,
            Int32 width
            )
        {
            if (points.Count < 2) return;

            // Reuse the established raster primitive after one coordinate conversion pass.
            List<(Double X, Double Y)> Pixels = points.Select(Point =>
                WorldToPixel(grid, Point.X, Point.Y, pixelsPerBin)).ToList();
            DrawPolylinePixels(image, Pixels, color, width);
        }

        /// <summary>
        /// Renders several consecutive pass pairs as one operator-track figure.
        /// </summary>
        static public Byte[] RenderPairSequencePng
            (
            Grid grid,
            IReadOnlyList<GuidancePoint> points,
            Double bladeWidthM = 2.271,
            Int32 pixelsPerBin = 26
            )
        {
            // Multi-pair sequence uses the same action coloring to show transitions across adjacent passes.
            return RenderGuidancePng(grid, points, pixelsPerBin, bladeWidthM);
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Byte[] render_pair_sequence_png
            (
            Grid grid,
            IReadOnlyList<GuidancePoint> pts,
            Int32 pixels_per_bin = 26
            ) => RenderPairSequencePng(grid, pts, 2.271, pixels_per_bin);

        /// <summary>
        /// Overlays numbered zone waypoints onto an existing PNG buffer.
        /// </summary>
        /// <param name="pngBytes">Base PNG bytes.</param>
        /// <param name="grid">Grid geometry for world->pixel projection.</param>
        /// <param name="steps">Ordered zone steps.</param>
        /// <param name="pixelsPerBin">Pixels per bin for projection math.</param>
        /// <returns>Updated PNG bytes.</returns>
        static public Byte[] OverlayZoneWaypoints
            (
            Byte[] pngBytes,
            Grid grid,
            IReadOnlyList<ZoneStep> steps,
            Int32 pixelsPerBin = 50
            )
        {
            // Overlay is done in-place on decoded PNG bytes so engine can stay fully in-memory.
            using Image<Rgba32> OverlayImage = SixLabors.ImageSharp.Image.Load<Rgba32>(pngBytes);
            if (steps.Count <= 0)
            {
                return ToPng(OverlayImage);
            }

            Int32 Ppb = Math.Max(1, pixelsPerBin);
            for (Int32 Iter = 0; Iter < steps.Count; Iter += 1)
            {
                ZoneStep Step = steps[Iter];
                (Double Px, Double Py) = WorldToPixel(grid, Step.Easting, Step.Northing, Ppb);
                DrawCircle(OverlayImage, Px, Py, Math.Max(4, Ppb / 2), new Rgba32(255, 255, 255, 235), new Rgba32(0, 0, 0, 255));
                DrawTextDigit(OverlayImage, (Int32)Math.Round(Px), (Int32)Math.Round(Py), ((Int32)Step.WorkOrder).ToString(CultureInfo.InvariantCulture));
            }

            for (Int32 Iter = 1; Iter < steps.Count; Iter += 1)
            {
                ZoneStep Prev = steps[Iter - 1];
                ZoneStep Cur = steps[Iter];
                (Double X0, Double Y0) = WorldToPixel(grid, Prev.Easting, Prev.Northing, Ppb);
                (Double X1, Double Y1) = WorldToPixel(grid, Cur.Easting, Cur.Northing, Ppb);
                DrawLine(OverlayImage, X0, Y0, X1, Y1, new Rgba32(0, 0, 0, 220), Math.Max(2, Ppb / 4));
            }

            return ToPng(OverlayImage);
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public Byte[] overlay_zone_waypoints
            (
            Byte[] in_out_png,
            Grid grid,
            IReadOnlyList<ZoneStep> steps,
            Int32 pixels_per_bin = 50
            ) => OverlayZoneWaypoints(in_out_png, grid, steps, pixels_per_bin);

        private static Rgba32 TerrainColor
            (
            Double t
            )
        {
            // Piecewise interpolation reproduces Python numpy.interp behavior for elevation ramps.
            if (t <= 0.25)
            {
                return Lerp(new Rgba32(30, 90, 40, 255), new Rgba32(120, 170, 70, 255), (t / 0.25));
            }

            if (t <= 0.50)
            {
                return Lerp(new Rgba32(120, 170, 70, 255), new Rgba32(210, 200, 120, 255), ((t - 0.25) / 0.25));
            }

            if (t <= 0.75)
            {
                return Lerp(new Rgba32(210, 200, 120, 255), new Rgba32(150, 110, 70, 255), ((t - 0.50) / 0.25));
            }

            return Lerp(new Rgba32(150, 110, 70, 255), new Rgba32(245, 245, 245, 255), ((t - 0.75) / 0.25));
        }

        private static Rgba32 Lerp
            (
            Rgba32 a,
            Rgba32 b,
            Double t
            )
        {
            // Truncate like numpy astype(uint8) after float interpolation.
            Byte R = TruncByte(a.R + ((b.R - a.R) * t));
            Byte G = TruncByte(a.G + ((b.G - a.G) * t));
            Byte B = TruncByte(a.B + ((b.B - a.B) * t));
            return new Rgba32(R, G, B, 0xFF);
        }

        private static Byte TruncByte
            (
            Double value
            )
        {
            if (value <= 0.0)
            {
                return 0;
            }

            if (value >= 255.0)
            {
                return 255;
            }

            return (Byte)value;
        }

        private static (Double X, Double Y) WorldToPixel
            (
            Grid grid,
            Double x,
            Double y,
            Int32 ppb
            )
        {
            // Coordinate transform intentionally mirrors Python render._world_to_pixel.
            Double Px = ((x - grid.MinX) / grid.GridSizeM) * ppb;
            Double Py = (grid.Height - ((y - grid.MinY) / grid.GridSizeM)) * ppb;
            return (Px, Py);
        }

        private static void DrawFlowArrows
            (
            Image<Rgba32> image,
            Grid grid,
            FlowField flow,
            Int32 ppb,
            Int32 arrowStride
            )
        {
            // Arrow glyphs are hollow triangles so the map remains visible beneath direction marks.
            Double Length = (ppb * 0.45);
            Double HalfBase = (ppb * 0.16);
            for (Int32 By = 0; By < grid.Height; By += arrowStride)
            {
                for (Int32 Bx = 0; Bx < grid.Width; Bx += arrowStride)
                {
                    if (!grid.Valid[By, Bx]) continue;
                    if (!FlowHasArrow(flow, Bx, By)) continue;
                    Double DirectionDeg = FlowDirectionDeg(flow, Bx, By);
                    Double Rad = (Math.PI / 180.0) * DirectionDeg;
                    Double Dx = Math.Sin(Rad);
                    Double Dy = -Math.Cos(Rad);
                    Double Px = ((Bx + 0.5) * ppb);
                    Double Py = ((grid.Height - 0.5 - By) * ppb);
                    Double EndX = (Px + (Length * Dx));
                    Double EndY = (Py + (Length * Dy));
                    Double BaseX = (Px - (0.35 * Length * Dx));
                    Double BaseY = (Py - (0.35 * Length * Dy));
                    Double PerpX = -Dy;
                    Double PerpY = Dx;
                    Int32 TipX = (Int32)Math.Round(EndX, MidpointRounding.AwayFromZero);
                    Int32 TipY = (Int32)Math.Round(EndY, MidpointRounding.AwayFromZero);
                    Int32 LeftX = (Int32)Math.Round(BaseX + (HalfBase * PerpX), MidpointRounding.AwayFromZero);
                    Int32 LeftY = (Int32)Math.Round(BaseY + (HalfBase * PerpY), MidpointRounding.AwayFromZero);
                    Int32 RightX = (Int32)Math.Round(BaseX - (HalfBase * PerpX), MidpointRounding.AwayFromZero);
                    Int32 RightY = (Int32)Math.Round(BaseY - (HalfBase * PerpY), MidpointRounding.AwayFromZero);

                    DrawThinLineBresenham(image, TipX, TipY, LeftX, LeftY, new Rgba32(0, 0, 0, 255));
                    DrawThinLineBresenham(image, TipX, TipY, RightX, RightY, new Rgba32(0, 0, 0, 255));
                    DrawThinLineBresenham(image, LeftX, LeftY, RightX, RightY, new Rgba32(0, 0, 0, 255));
                }
            }
        }

        /// <summary>Draws high-contrast arrowheads at regular travelled distances along a route.</summary>
        private static void DrawRouteDirectionArrows
            (
            Image<Rgba32> image,
            Grid grid,
            IReadOnlyList<(Double X, Double Y)> routePoints,
            Int32 ppb,
            Double spacingM
            )
        {
            if ((routePoints.Count < 2) || (spacingM <= 0.0)) return;
            Double UntilArrowM = 0.5 * spacingM;
            Double ArrowLengthPx = Math.Max(8.0, ppb * 0.42);
            Double HalfWidthPx = Math.Max(4.0, ppb * 0.18);
            Int32 OutlineWidth = Math.Max(3, ppb / 8);
            Int32 InnerWidth = Math.Max(1, OutlineWidth / 2);
            Rgba32 Outline = new Rgba32(0, 0, 0, 255);
            Rgba32 Inner = new Rgba32(255, 255, 255, 255);

            for (Int32 Index = 1; Index < routePoints.Count; Index += 1)
            {
                (Double X, Double Y) A = routePoints[Index - 1];
                (Double X, Double Y) B = routePoints[Index];
                Double DxM = B.X - A.X;
                Double DyM = B.Y - A.Y;
                Double SegmentM = Math.Sqrt((DxM * DxM) + (DyM * DyM));
                if (SegmentM <= 1e-9) continue;
                while (UntilArrowM <= SegmentM + 1e-9)
                {
                    Double Fraction = Math.Min(1.0, UntilArrowM / SegmentM);
                    Double X = A.X + (Fraction * DxM);
                    Double Y = A.Y + (Fraction * DyM);
                    (Double TipX, Double TipY) = WorldToPixel(grid, X, Y, ppb);
                    Double UnitX = DxM / SegmentM;
                    Double UnitY = -DyM / SegmentM;
                    Double BaseX = TipX - (ArrowLengthPx * UnitX);
                    Double BaseY = TipY - (ArrowLengthPx * UnitY);
                    Double PerpendicularX = -UnitY;
                    Double PerpendicularY = UnitX;
                    Double LeftX = BaseX + (HalfWidthPx * PerpendicularX);
                    Double LeftY = BaseY + (HalfWidthPx * PerpendicularY);
                    Double RightX = BaseX - (HalfWidthPx * PerpendicularX);
                    Double RightY = BaseY - (HalfWidthPx * PerpendicularY);
                    DrawLine(image, TipX, TipY, LeftX, LeftY, Outline, OutlineWidth);
                    DrawLine(image, TipX, TipY, RightX, RightY, Outline, OutlineWidth);
                    DrawLine(image, TipX, TipY, LeftX, LeftY, Inner, InnerWidth);
                    DrawLine(image, TipX, TipY, RightX, RightY, Inner, InnerWidth);
                    UntilArrowM += spacingM;
                }
                UntilArrowM -= SegmentM;
            }
        }

        private static void DrawExclusions
            (
            Image<Rgba32> image,
            Grid grid,
            Int32 ppb
            )
        {
            // Exclusion overlays keep no-go zones visually explicit without hiding the underlying grade map.
            IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> Rings = GridExclusions(grid);
            if (Rings.Count <= 0)
            {
                return;
            }

            foreach (IReadOnlyList<(Double X, Double Y)> Ring in Rings)
            {
                for (Int32 Iter = 1; Iter < Ring.Count; Iter += 1)
                {
                    (Double Ax, Double Ay) = WorldToPixel(grid, Ring[Iter - 1].X, Ring[Iter - 1].Y, ppb);
                    (Double Bx, Double By) = WorldToPixel(grid, Ring[Iter].X, Ring[Iter].Y, ppb);
                    DrawLine(image, Ax, Ay, Bx, By, new Rgba32(0, 0, 0, 255), Math.Max(2, ppb / 4));
                }
            }
        }

        private static void DimTowardsWhite
            (
            Image<Rgba32> image,
            Double baseDim
            )
        {
            // Fading base color increases contrast for overlays while preserving context.
            if ((baseDim < 0.0) || (baseDim >= 1.0))
            {
                return;
            }

            for (Int32 Y = 0; Y < image.Height; Y += 1)
            {
                for (Int32 X = 0; X < image.Width; X += 1)
                {
                    Rgba32 P = image[X, Y];
                    // Pillow dims RGB for every pixel, including alpha=0 cells (render_pass_pairs_png).
                    P.R = TruncByte((P.R * baseDim) + (255.0 * (1.0 - baseDim)));
                    P.G = TruncByte((P.G * baseDim) + (255.0 * (1.0 - baseDim)));
                    P.B = TruncByte((P.B * baseDim) + (255.0 * (1.0 - baseDim)));
                    image[X, Y] = P;
                }
            }
        }

        private static Rgba32 GuidanceColor
            (
            String action
            )
        {
            // Action palette mirrors Python guidance rendering semantics.
            if (action == "CUT") return new Rgba32(200, 0, 0, 255);
            if (action == "FILL") return new Rgba32(0, 90, 220, 255);
            if (action == "GRADE") return new Rgba32(40, 160, 40, 200);
            if (action == "HAUL") return new Rgba32(200, 0, 200, 90);
            if (action == "MOVE") return new Rgba32(255, 140, 0, 120);
            if (action == "MANUAL_MOVE") return new Rgba32(255, 80, 0, 220);
            return new Rgba32(120, 120, 120, 70);
        }

        private static void DrawPolyline
            (
            Image<Rgba32> image,
            Grid grid,
            IReadOnlyList<GuidancePoint> points,
            Int32 ppb,
            Rgba32 color,
            Int32 width
            )
        {
            // Guidance polylines are drawn segment-by-segment in world space.
            for (Int32 Iter = 1; Iter < points.Count; Iter += 1)
            {
                (Double X0, Double Y0) = WorldToPixel(grid, points[Iter - 1].X, points[Iter - 1].Y, ppb);
                (Double X1, Double Y1) = WorldToPixel(grid, points[Iter].X, points[Iter].Y, ppb);
                DrawLine(
                    image,
                    X0,
                    Y0,
                    X1,
                    Y1,
                    color,
                    width);
            }
        }

        private static void DrawPolylinePixels
            (
            Image<Rgba32> image,
            IReadOnlyList<(Double X, Double Y)> points,
            Rgba32 color,
            Int32 width
            )
        {
            // Pixel-space polylines are used for route overlays already projected to image coordinates.
            for (Int32 Iter = 1; Iter < points.Count; Iter += 1)
            {
                (Double X0, Double Y0) = points[Iter - 1];
                (Double X1, Double Y1) = points[Iter];
                DrawLine(image, X0, Y0, X1, Y1, color, width);
            }
        }

        private static void DotGuidancePoint
            (
            Image<Rgba32> image,
            Grid grid,
            GuidancePoint point,
            Int32 ppb,
            Rgba32 color,
            Int32 radius
            )
        {
            (Double Px, Double Py) = WorldToPixel(grid, point.X, point.Y, ppb);
            DrawEllipsePil(
                image,
                Px,
                Py,
                radius,
                color,
                new Rgba32(0, 0, 0, 255),
                2);
        }

        private static void DrawEllipsePil
            (
            Image<Rgba32> image,
            Double cx,
            Double cy,
            Int32 radius,
            Rgba32 fill,
            Rgba32 outline,
            Int32 outlineWidth
            )
        {
            // Matches Pillow ImageDraw.ellipse((x-r,y-r,x+r,y+r), fill=..., outline=..., width=...).
            Int32 X0 = TruncTowardZero(cx - radius);
            Int32 Y0 = TruncTowardZero(cy - radius);
            Int32 X1 = TruncTowardZero(cx + radius);
            Int32 Y1 = TruncTowardZero(cy + radius);
            if (outlineWidth > 0)
            {
                DrawFilledEllipse(image, X0, Y0, X1, Y1, fill);
                DrawEllipseOutlineRing(image, X0, Y0, X1, Y1, outline, outlineWidth);
                return;
            }

            DrawFilledEllipse(image, X0, Y0, X1, Y1, fill);
        }

        private static void DrawEllipseOutlineRing
            (
            Image<Rgba32> image,
            Int32 x0,
            Int32 y0,
            Int32 x1,
            Int32 y1,
            Rgba32 color,
            Int32 width
            )
        {
            Int32 InnerPad = width / 2;
            Int32 OuterPad = width - InnerPad;
            Int32 Ox0 = x0 - OuterPad;
            Int32 Oy0 = y0 - OuterPad;
            Int32 Ox1 = x1 + OuterPad;
            Int32 Oy1 = y1 + OuterPad;
            Int32 Ix0 = x0 + InnerPad;
            Int32 Iy0 = y0 + InnerPad;
            Int32 Ix1 = x1 - InnerPad;
            Int32 Iy1 = y1 - InnerPad;
            Double Ocx = (Ox0 + Ox1) / 2.0;
            Double Ocy = (Oy0 + Oy1) / 2.0;
            Double Orx = Math.Max(0.5, (Ox1 - Ox0) / 2.0);
            Double Ory = Math.Max(0.5, (Oy1 - Oy0) / 2.0);
            Double Icx = (Ix0 + Ix1) / 2.0;
            Double Icy = (Iy0 + Iy1) / 2.0;
            Double Irx = Math.Max(0.5, (Ix1 - Ix0) / 2.0);
            Double Iry = Math.Max(0.5, (Iy1 - Iy0) / 2.0);
            Int32 MinX = Math.Max(0, Ox0);
            Int32 MaxX = Math.Min(image.Width - 1, Ox1);
            Int32 MinY = Math.Max(0, Oy0);
            Int32 MaxY = Math.Min(image.Height - 1, Oy1);
            for (Int32 Y = MinY; Y <= MaxY; Y += 1)
            {
                for (Int32 X = MinX; X <= MaxX; X += 1)
                {
                    Double Onx = (X - Ocx) / Orx;
                    Double Ony = (Y - Ocy) / Ory;
                    if (((Onx * Onx) + (Ony * Ony)) > 1.0)
                    {
                        continue;
                    }

                    Double Inx = (X - Icx) / Irx;
                    Double Iny = (Y - Icy) / Iry;
                    if (((Inx * Inx) + (Iny * Iny)) <= 1.0)
                    {
                        continue;
                    }

                    Plot(image, X, Y, color);
                }
            }
        }

        private static Int32 TruncTowardZero
            (
            Double value
            )
        {
            return value >= 0.0 ? (Int32)Math.Floor(value) : (Int32)Math.Ceiling(value);
        }

        private static void DrawFilledEllipse
            (
            Image<Rgba32> image,
            Int32 x0,
            Int32 y0,
            Int32 x1,
            Int32 y1,
            Rgba32 color
            )
        {
            Double Cx = (x0 + x1) / 2.0;
            Double Cy = (y0 + y1) / 2.0;
            Double Rx = Math.Max(0.5, (x1 - x0) / 2.0);
            Double Ry = Math.Max(0.5, (y1 - y0) / 2.0);
            Int32 MinX = Math.Max(0, x0);
            Int32 MaxX = Math.Min(image.Width - 1, x1);
            Int32 MinY = Math.Max(0, y0);
            Int32 MaxY = Math.Min(image.Height - 1, y1);
            for (Int32 Y = MinY; Y <= MaxY; Y += 1)
            {
                for (Int32 X = MinX; X <= MaxX; X += 1)
                {
                    Double Nx = (X - Cx) / Rx;
                    Double Ny = (Y - Cy) / Ry;
                    if (((Nx * Nx) + (Ny * Ny)) <= 1.0)
                    {
                        Plot(image, X, Y, color);
                    }
                }
            }
        }

        private static void DrawLine
            (
            Image<Rgba32> image,
            Double x0,
            Double y0,
            Double x1,
            Double y1,
            Rgba32 color,
            Int32 width
            )
        {
            // Pillow rounds endpoints and strokes width via parallel 1-px lines.
            Int32 X0 = (Int32)Math.Round(x0, MidpointRounding.AwayFromZero);
            Int32 Y0 = (Int32)Math.Round(y0, MidpointRounding.AwayFromZero);
            Int32 X1 = (Int32)Math.Round(x1, MidpointRounding.AwayFromZero);
            Int32 Y1 = (Int32)Math.Round(y1, MidpointRounding.AwayFromZero);
            if (width <= 1)
            {
                DrawThinLineBresenham(image, X0, Y0, X1, Y1, color);
                return;
            }

            DrawWideLinePillow(image, X0, Y0, X1, Y1, color, width);
        }

        private static void DrawWideLinePillow
            (
            Image<Rgba32> image,
            Int32 x0,
            Int32 y0,
            Int32 x1,
            Int32 y1,
            Rgba32 color,
            Int32 width
            )
        {
            // Port of Pillow ImagingDrawWideLine (src/libImaging/Draw.c).
            Int32 Dx = (x1 - x0);
            Int32 Dy = (y1 - y0);
            if ((Dx == 0) && (Dy == 0))
            {
                Plot(image, x0, y0, color);
                return;
            }

            Double BigHyp = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Double SmallHyp = (width - 1) / 2.0;
            Double RatioMax = RoundUp(SmallHyp) / BigHyp;
            Double RatioMin = RoundDown(SmallHyp) / BigHyp;
            Int32 DxMin = RoundDown(RatioMin * Dy);
            Int32 DxMax = RoundDown(RatioMax * Dy);
            Int32 DyMin = RoundDown(RatioMin * Dx);
            Int32 DyMax = RoundDown(RatioMax * Dx);
            Int32[] Xs = { x0 - DxMin, x1 - DxMin, x1 + DxMax, x0 + DxMax };
            Int32[] Ys = { y0 + DyMax, y1 + DyMax, y1 - DyMin, y0 - DyMin };
            FillPolygonScanline(image, Xs, Ys, color);
            for (Int32 Iter = 0; Iter < Xs.Length; Iter += 1)
            {
                Plot(image, Xs[Iter], Ys[Iter], color);
            }
        }

        private static Int32 RoundUp
            (
            Double value
            )
        {
            return value >= 0.0
                ? (Int32)Math.Floor(value + 0.5)
                : -(Int32)Math.Floor(Math.Abs(value) + 0.5);
        }

        private static Int32 RoundDown
            (
            Double value
            )
        {
            return value >= 0.0
                ? (Int32)Math.Ceiling(value - 0.5)
                : -(Int32)Math.Ceiling(Math.Abs(value) - 0.5);
        }

        private static void FillPolygonScanline
            (
            Image<Rgba32> image,
            Int32[] xs,
            Int32[] ys,
            Rgba32 color
            )
        {
            Int32 Count = xs.Length;
            Int32 MinY = ys.Min();
            Int32 MaxY = ys.Max();
            for (Int32 Y = MinY; Y <= MaxY; Y += 1)
            {
                List<Double> Intersections = new List<Double>();
                for (Int32 Iter = 0; Iter < Count; Iter += 1)
                {
                    Int32 Y0 = ys[Iter];
                    Int32 Y1 = ys[(Iter + 1) % Count];
                    Int32 X0 = xs[Iter];
                    Int32 X1 = xs[(Iter + 1) % Count];
                    if (Y0 == Y1)
                    {
                        continue;
                    }

                    if (Y >= Math.Min(Y0, Y1) && Y < Math.Max(Y0, Y1))
                    {
                        Double X = X0 + (((Double)(Y - Y0) / (Y1 - Y0)) * (X1 - X0));
                        Intersections.Add(X);
                    }
                }

                Intersections.Sort();
                for (Int32 Iter = 1; Iter < Intersections.Count; Iter += 2)
                {
                    Int32 XStart = RoundUp(Intersections[Iter - 1]);
                    Int32 XEnd = RoundDown(Intersections[Iter]);
                    for (Int32 X = XStart; X <= XEnd; X += 1)
                    {
                        Plot(image, X, Y, color);
                    }
                }
            }
        }

        private static void DrawThinLineBresenham
            (
            Image<Rgba32> image,
            Int32 x0,
            Int32 y0,
            Int32 x1,
            Int32 y1,
            Rgba32 color
            )
        {
            Int32 Dx = Math.Abs(x1 - x0);
            Int32 Dy = Math.Abs(y1 - y0);
            Int32 Sx = (x0 < x1) ? 1 : -1;
            Int32 Sy = (y0 < y1) ? 1 : -1;
            Int32 Err = (Dx - Dy);
            Int32 X = x0;
            Int32 Y = y0;
            while (true)
            {
                Plot(image, X, Y, color);
                if ((X == x1) && (Y == y1))
                {
                    break;
                }

                Int32 E2 = (2 * Err);
                if (E2 > -Dy)
                {
                    Err -= Dy;
                    X += Sx;
                }

                if (E2 < Dx)
                {
                    Err += Dx;
                    Y += Sy;
                }
            }
        }

        private static void DrawFilledSquare
            (
            Image<Rgba32> image,
            Int32 cx,
            Int32 cy,
            Int32 width,
            Rgba32 color
            )
        {
            Int32 Half = width / 2;
            for (Int32 Dy = -Half; Dy <= (width - Half - 1); Dy += 1)
            {
                for (Int32 Dx = -Half; Dx <= (width - Half - 1); Dx += 1)
                {
                    Plot(image, cx + Dx, cy + Dy, color);
                }
            }
        }

        private static void DrawCircle
            (
            Image<Rgba32> image,
            Double x,
            Double y,
            Int32 radius,
            Rgba32 fill,
            Rgba32 stroke
            )
        {
            // Marker circles are rasterized manually to avoid optional drawing package dependencies.
            DrawCircleFilled(image, (Int32)Math.Round(x), (Int32)Math.Round(y), radius, fill);
            Int32 Outline = Math.Max(1, radius / 4);
            for (Int32 Iter = 0; Iter < Outline; Iter += 1)
            {
                DrawCircleRing(image, (Int32)Math.Round(x), (Int32)Math.Round(y), Math.Max(1, radius - Iter), stroke);
            }
        }

        private static void DrawCircleFilled
            (
            Image<Rgba32> image,
            Int32 cx,
            Int32 cy,
            Int32 radius,
            Rgba32 color
            )
        {
            // Filled disk stamp is reused by markers and thick polyline drawing.
            for (Int32 Dy = -radius; Dy <= radius; Dy += 1)
            {
                for (Int32 Dx = -radius; Dx <= radius; Dx += 1)
                {
                    if (((Dx * Dx) + (Dy * Dy)) > (radius * radius)) continue;
                    Int32 X = (cx + Dx);
                    Int32 Y = (cy + Dy);
                    if ((X < 0) || (Y < 0) || (X >= image.Width) || (Y >= image.Height)) continue;
                    image[X, Y] = color;
                }
            }
        }

        private static void DrawCircleRing
            (
            Image<Rgba32> image,
            Int32 cx,
            Int32 cy,
            Int32 radius,
            Rgba32 color
            )
        {
            // Ring midpoint stepping draws a crisp border around waypoint markers.
            if (radius <= 0) return;
            Int32 X = radius;
            Int32 Y = 0;
            Int32 Err = 0;
            while (X >= Y)
            {
                Plot8(image, cx, cy, X, Y, color);
                Y += 1;
                if (Err <= 0)
                {
                    Err += ((2 * Y) + 1);
                }
                else
                {
                    X -= 1;
                    Err -= ((2 * X) + 1);
                }
            }
        }

        private static void Plot8
            (
            Image<Rgba32> image,
            Int32 cx,
            Int32 cy,
            Int32 x,
            Int32 y,
            Rgba32 color
            )
        {
            // Eight-way symmetry keeps circle plotting branch-free at call sites.
            Plot(image, (cx + x), (cy + y), color);
            Plot(image, (cx + y), (cy + x), color);
            Plot(image, (cx - y), (cy + x), color);
            Plot(image, (cx - x), (cy + y), color);
            Plot(image, (cx - x), (cy - y), color);
            Plot(image, (cx - y), (cy - x), color);
            Plot(image, (cx + y), (cy - x), color);
            Plot(image, (cx + x), (cy - y), color);
        }

        private static void Plot
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            Rgba32 color
            )
        {
            // Bounds checks are centralized so primitive routines remain simple.
            if ((x < 0) || (y < 0) || (x >= image.Width) || (y >= image.Height)) return;
            image[x, y] = color;
        }

        private static void DrawTextDigit
            (
            Image<Rgba32> image,
            Int32 centerX,
            Int32 centerY,
            String text
            )
        {
            // Small bitmap digits avoid adding a font rasterizer dependency just for waypoint labels.
            if (String.IsNullOrWhiteSpace(text)) return;
            String Value = text.Trim();
            Int32 OffsetX = centerX - (Value.Length * 4);
            for (Int32 Iter = 0; Iter < Value.Length; Iter += 1)
            {
                DrawGlyph(image, Value[Iter], (OffsetX + (Iter * 8)), (centerY - 4), new Rgba32(0, 0, 0, 255));
            }
        }

        private static IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> GridExclusions
            (
            Grid grid
            )
        {
            // Reflection keeps this renderer compatible with grid variants that may omit exclusions.
            Object? Value = grid.GetType().GetProperty("Exclusions")?.GetValue(grid);
            if (Value is IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> Typed)
            {
                return Typed;
            }

            return Array.Empty<IReadOnlyList<(Double X, Double Y)>>();
        }

        private static Boolean FlowHasArrow
            (
            FlowField flow,
            Int32 bx,
            Int32 by
            )
        {
            // Both array-backed and method-backed flow ports are supported.
            Object? ArrowArray = flow.GetType().GetProperty("HasArrow")?.GetValue(flow);
            if (ArrowArray is Boolean[,] Matrix)
            {
                return Matrix[by, bx];
            }

            System.Reflection.MethodInfo? HasArrowMethod = flow.GetType().GetMethod("HasArrow");
            if (HasArrowMethod is not null)
            {
                Object? Result = HasArrowMethod.Invoke(flow, new Object[] { bx, by });
                if (Result is Boolean Flag) return Flag;
            }

            return false;
        }

        private static Double FlowDirectionDeg
            (
            FlowField flow,
            Int32 bx,
            Int32 by
            )
        {
            // Direction lookup falls back to fx/fy vector conversion when a direct helper is absent.
            System.Reflection.MethodInfo? DirectionMethod = flow.GetType().GetMethod("DirectionDeg");
            if (DirectionMethod is not null)
            {
                Object? Value = DirectionMethod.Invoke(flow, new Object[] { bx, by });
                if (Value is Double Degrees) return Degrees;
            }

            Object? FxValue = flow.GetType().GetProperty("Fx")?.GetValue(flow);
            Object? FyValue = flow.GetType().GetProperty("Fy")?.GetValue(flow);
            if ((FxValue is Double[,] Fx) && (FyValue is Double[,] Fy))
            {
                Double Rad = Math.Atan2(Fx[by, bx], Fy[by, bx]);
                Double Deg = (Rad * (180.0 / Math.PI));
                if (Deg < 0.0) Deg += 360.0;
                return Deg;
            }

            return 0.0;
        }

        private static void DrawGlyph
            (
            Image<Rgba32> image,
            Char ch,
            Int32 x,
            Int32 y,
            Rgba32 color
            )
        {
            // Minimal 3x5 digit glyphs keep labels legible without external drawing libraries.
            String[] Glyph = ch switch
            {
                '0' => new[] { "111", "101", "101", "101", "111" },
                '1' => new[] { "010", "110", "010", "010", "111" },
                '2' => new[] { "111", "001", "111", "100", "111" },
                '3' => new[] { "111", "001", "111", "001", "111" },
                '4' => new[] { "101", "101", "111", "001", "001" },
                '5' => new[] { "111", "100", "111", "001", "111" },
                '6' => new[] { "111", "100", "111", "101", "111" },
                '7' => new[] { "111", "001", "001", "001", "001" },
                '8' => new[] { "111", "101", "111", "101", "111" },
                '9' => new[] { "111", "101", "111", "001", "111" },
                _ => Array.Empty<String>(),
            };
            for (Int32 Row = 0; Row < Glyph.Length; Row += 1)
            {
                String Line = Glyph[Row];
                for (Int32 Col = 0; Col < Line.Length; Col += 1)
                {
                    if (Line[Col] != '1') continue;
                    Plot(image, (x + Col), (y + Row), color);
                }
            }
        }

        private static Byte[] ToPng
            (
            Image<Rgba32> image
            )
        {
            // Memory-stream encoding keeps the render API file-system independent.
            using MemoryStream Stream = new MemoryStream();
            image.Save(Stream, new PngEncoder());
            return Stream.ToArray();
        }

        private static Single PercentileLinear
            (
            IReadOnlyList<Single> sortedValues,
            Double percentile
            )
        {
            // Matches numpy.percentile(..., method="linear") used by Python render_pass_pairs_png.
            if (sortedValues.Count == 0)
            {
                return 1.0f;
            }

            if (sortedValues.Count == 1)
            {
                return sortedValues[0];
            }

            Double Index = percentile * (sortedValues.Count - 1);
            Int32 Lo = (Int32)Math.Floor(Index);
            Int32 Hi = (Int32)Math.Ceiling(Index);
            if (Lo == Hi)
            {
                return sortedValues[Lo];
            }

            Double Weight = Index - Lo;
            return (Single)((sortedValues[Lo] * (1.0 - Weight)) + (sortedValues[Hi] * Weight));
        }

        private static Rgba32 HeatMapColor
            (
            Single t
            )
        {
            // Yellow -> orange -> red ramp (_HEAT_STOPS / _HEAT_COLORS in Python render.py).
            Double[] Stops = { 0.0, 0.5, 1.0 };
            Double[] Rs = { 255.0, 255.0, 200.0 };
            Double[] Gs = { 225.0, 110.0, 0.0 };
            Double[] Bs = { 0.0, 0.0, 0.0 };
            return new Rgba32(
                TruncByte(NumpyInterp(t, Stops, Rs)),
                TruncByte(NumpyInterp(t, Stops, Gs)),
                TruncByte(NumpyInterp(t, Stops, Bs)),
                255);
        }

        private static Double NumpyInterp
            (
            Double x,
            Double[] xp,
            Double[] fp
            )
        {
            if (x <= xp[0])
            {
                return fp[0];
            }

            Int32 Last = xp.Length - 1;
            if (x >= xp[Last])
            {
                return fp[Last];
            }

            for (Int32 Iter = 0; Iter < Last; Iter += 1)
            {
                if (x < xp[Iter + 1])
                {
                    Double T = (x - xp[Iter]) / (xp[Iter + 1] - xp[Iter]);
                    return fp[Iter] + (T * (fp[Iter + 1] - fp[Iter]));
                }
            }

            return fp[Last];
        }

        private static void AlphaCompositePixel
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            Rgba32 source,
            Byte sourceAlpha
            )
        {
            // PIL Image.alpha_composite over the existing base pixel.
            Rgba32 Dest = image[x, y];
            Double Sa = sourceAlpha / 255.0;
            Double Da = Dest.A / 255.0;
            Double OutA = Sa + (Da * (1.0 - Sa));
            if (OutA <= 1e-12)
            {
                image[x, y] = new Rgba32(0, 0, 0, 0);
                return;
            }

            Byte R = (Byte)Math.Round(((source.R * Sa) + (Dest.R * Da * (1.0 - Sa))) / OutA, MidpointRounding.AwayFromZero);
            Byte G = (Byte)Math.Round(((source.G * Sa) + (Dest.G * Da * (1.0 - Sa))) / OutA, MidpointRounding.AwayFromZero);
            Byte B = (Byte)Math.Round(((source.B * Sa) + (Dest.B * Da * (1.0 - Sa))) / OutA, MidpointRounding.AwayFromZero);
            Byte A = (Byte)Math.Round(OutA * 255.0, MidpointRounding.AwayFromZero);
            image[x, y] = new Rgba32(R, G, B, A);
        }
    }
}
