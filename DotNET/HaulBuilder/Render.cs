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
    static public class Render
    {
        private const Byte EMPTY_COLOR_CHANNEL = 0x00;
        private const Byte VIOLET_CHANNEL = 0x80;
        private const Byte INDIGO_RED_CHANNEL = 0x9A;
        private const Byte INDIGO_GREEN_CHANNEL = 0x31;
        private const Byte BLUE_GREEN_CHANNEL = 0x66;
        private const Byte CYAN_RED_CHANNEL = 0x36;
        private const Byte CYAN_GREEN_BLUE_CHANNEL = 0xF5;
        private const Byte GREEN_CHANNEL = 0xCD;
        private const Byte ORANGE_GREEN_CHANNEL = 0x80;
        private const Byte DARK_RED_CHANNEL = 0xB4;
        private const Double EXTREME_CUT_THRESHOLD_M = -0.80;
        private const Double LARGE_CUT_THRESHOLD_M = -Constants.HALF_SCALE;
        private const Double MEDIUM_CUT_THRESHOLD_M = -0.05;
        private const Double SMALL_CUT_THRESHOLD_M = -0.01;
        private const Double ON_GRADE_THRESHOLD_M = 0.01;
        private const Double SMALL_FILL_THRESHOLD_M = 0.05;
        private const Double MEDIUM_FILL_THRESHOLD_M = Constants.HALF_SCALE;
        private const Double LARGE_FILL_THRESHOLD_M = 0.80;
        private static readonly Rgba32 VIOLET = new Rgba32(VIOLET_CHANNEL, EMPTY_COLOR_CHANNEL, VIOLET_CHANNEL, Constants.BYTE_MASK);
        private static readonly Rgba32 INDIGO = new Rgba32(INDIGO_RED_CHANNEL, INDIGO_GREEN_CHANNEL, Constants.BYTE_MASK, Constants.BYTE_MASK);
        private static readonly Rgba32 BLUE = new Rgba32(EMPTY_COLOR_CHANNEL, BLUE_GREEN_CHANNEL, Constants.BYTE_MASK, Constants.BYTE_MASK);
        private static readonly Rgba32 CYAN = new Rgba32(CYAN_RED_CHANNEL, CYAN_GREEN_BLUE_CHANNEL, CYAN_GREEN_BLUE_CHANNEL, Constants.BYTE_MASK);
        private static readonly Rgba32 GREEN = new Rgba32(EMPTY_COLOR_CHANNEL, GREEN_CHANNEL, EMPTY_COLOR_CHANNEL, Constants.BYTE_MASK);
        private static readonly Rgba32 YELLOW = new Rgba32(Constants.BYTE_MASK, Constants.BYTE_MASK, EMPTY_COLOR_CHANNEL, Constants.BYTE_MASK);
        private static readonly Rgba32 ORANGE = new Rgba32(Constants.BYTE_MASK, ORANGE_GREEN_CHANNEL, EMPTY_COLOR_CHANNEL, Constants.BYTE_MASK);
        private static readonly Rgba32 RED = new Rgba32(Constants.BYTE_MASK, EMPTY_COLOR_CHANNEL, EMPTY_COLOR_CHANNEL, Constants.BYTE_MASK);
        private static readonly Rgba32 DARKRED = new Rgba32(DARK_RED_CHANNEL, EMPTY_COLOR_CHANNEL, EMPTY_COLOR_CHANNEL, Constants.BYTE_MASK);

        /// <summary>
        /// Maps <c>existing-target</c> delta to the nine-band cut/fill palette.
        /// </summary>
        /// <param name="Diff">Delta in metres.</param>
        /// <returns>Palette colour.</returns>
        static public Rgba32 CutFillColor
            (
            Double Diff
            )
        {
            // Thresholds are copied exactly from Python so visualization is parity-safe.
            if (Diff < EXTREME_CUT_THRESHOLD_M) return VIOLET;
            if (Diff < LARGE_CUT_THRESHOLD_M) return INDIGO;
            if (Diff < MEDIUM_CUT_THRESHOLD_M) return BLUE;
            if (Diff < SMALL_CUT_THRESHOLD_M) return CYAN;
            if (Diff <= ON_GRADE_THRESHOLD_M) return GREEN;
            if (Diff <= SMALL_FILL_THRESHOLD_M) return YELLOW;
            if (Diff <= MEDIUM_FILL_THRESHOLD_M) return ORANGE;
            if (Diff <= LARGE_FILL_THRESHOLD_M) return RED;
            return DARKRED;
        }


        private static readonly Rgba32[] CutFillPalette =
        {
            VIOLET, INDIGO, BLUE, CYAN, GREEN, YELLOW, ORANGE, RED, DARKRED,
        };

        /// <summary>
        /// Vectorized cut/fill image render from a diff grid.
        /// </summary>
        /// <param name="Diff">Diff grid (<c>existing-target</c>).</param>
        /// <param name="Valid">Validity mask.</param>
        /// <param name="PixelsPerBin">Pixels per cell edge.</param>
        /// <returns>Rendered image with Y flipped to image space.</returns>
        static public Image<Rgba32> DiffToImage
            (
            Double[,] Diff,
            Boolean[,] Valid,
            Int32 PixelsPerBin
            )
        {
            // Rendering the base at one pixel per cell keeps the raster logic simple, then upscale nearest-neighbor.
            Int32 Height = Diff.GetLength(0);
            Int32 Width = Diff.GetLength(1);
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);

            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Int32 Y = (Height - 1 - By);
                    if (!Valid[By, Bx] || Double.IsNaN(Diff[By, Bx]) || Double.IsInfinity(Diff[By, Bx]))
                    {
                        Image[Bx, Y] = new Rgba32(0, 0, 0, 0);
                        continue;
                    }

                    Image[Bx, Y] = CutFillColor(Diff[By, Bx]);
                }
            }

            if (PixelsPerBin > 1)
            {
                Image.Mutate(Action => Action.Resize(Width * PixelsPerBin, Height * PixelsPerBin, KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        /// <summary>
        /// Cut/fill image using numpy-style <c>digitize</c> bands (Python <c>diff_to_image</c>).
        /// </summary>
        static public Image<Rgba32> DiffToImageDigitize
            (
            Double[,] Diff,
            Boolean[,] Valid,
            Int32 PixelsPerBin
            )
        {
            Int32 Height = Diff.GetLength(0);
            Int32 Width = Diff.GetLength(1);
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);

            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Int32 Y = (Height - 1 - By);
                    Double Value = Valid[By, Bx] && !Double.IsNaN(Diff[By, Bx]) && !Double.IsInfinity(Diff[By, Bx])
                        ? Diff[By, Bx]
                        : 1e9;
                    Rgba32 Color = CutFillPalette[DigitizeBand(Value)];
                    Byte Alpha = (Valid[By, Bx] && !Double.IsNaN(Diff[By, Bx]) && !Double.IsInfinity(Diff[By, Bx]))
                        ? (Byte)Constants.MAX_COLOR_CHANNEL
                        : (Byte)0;
                    Image[Bx, Y] = new Rgba32(Color.R, Color.G, Color.B, Alpha);
                }
            }

            if (PixelsPerBin > 1)
            {
                Image.Mutate(Action => Action.Resize(Width * PixelsPerBin, Height * PixelsPerBin, KnownResamplers.NearestNeighbor));
            }

            return Image;
        }

        /// <summary>
        /// Executes the digitize band operation.
        /// </summary>
        /// <param name="Diff">The diff used by the operation.</param>
        /// <returns>The resulting digitize band.</returns>

        private static Int32 DigitizeBand
            (
            Double Diff
            )
        {
            // Matches np.digitize(..., _BAND_EDGES) with default right=True.
            if (Diff <= -0.80) return 0;
            if (Diff <= -Constants.HALF_SCALE) return 1;
            if (Diff <= -0.05) return 2;
            if (Diff <= -0.01) return 3;
            if (Diff <= 0.01) return 4;
            if (Diff <= 0.05) return 5;
            if (Diff <= Constants.HALF_SCALE) return 6;
            if (Diff <= 0.80) return 7;
            return 8;
        }


        /// <summary>
        /// Terrain-ramp elevation render for topology animations.
        /// </summary>
        /// <param name="Elev">Elevation grid in metres.</param>
        /// <param name="Valid">Validity mask.</param>
        /// <param name="PixelsPerBin">Pixels per cell edge.</param>
        /// <param name="Vmin">Fixed scale low value.</param>
        /// <param name="Vmax">Fixed scale high value.</param>
        /// <returns>Rendered terrain image with Y flipped to image space.</returns>
        static public Image<Rgba32> ElevationToImage
            (
            Double[,] Elev,
            Boolean[,] Valid,
            Int32 PixelsPerBin,
            Double Vmin,
            Double Vmax
            )
        {
            // A fixed range across frames shows flattening over time instead of re-normalizing each image.
            Int32 Height = Elev.GetLength(0);
            Int32 Width = Elev.GetLength(1);
            Double Range = Math.Abs(Vmax - Vmin) > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE ? (Vmax - Vmin) : 1.0;
            Image<Rgba32> Image = new Image<Rgba32>(Width, Height);

            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Int32 Y = (Height - 1 - By);
                    if (!Valid[By, Bx])
                    {
                        Image[Bx, Y] = new Rgba32(0, 0, 0, 0);
                        continue;
                    }

                    Double Elevation = Elev[By, Bx];
                    if (Double.IsNaN(Elevation) || Double.IsInfinity(Elevation))
                    {
                        Elevation = Vmin;
                    }

                    Double T = Math.Max(0.0, Math.Min(1.0, (Elevation - Vmin) / Range));
                    Rgba32 C = TerrainColor(T);
                    Image[Bx, Y] = C;
                }
            }

            if (PixelsPerBin > 1)
            {
                Image.Mutate(Action => Action.Resize(Width * PixelsPerBin, Height * PixelsPerBin, KnownResamplers.NearestNeighbor));
            }

            return Image;
        }


        /// <summary>
        /// Renders the cut/fill map with optional flow arrows and returns PNG bytes.
        /// </summary>
        /// <param name="Grid">Grid source model.</param>
        /// <param name="Flow">Optional flow field.</param>
        /// <param name="PixelsPerBin">Pixels per grid cell edge.</param>
        /// <param name="ArrowStride">Arrow stride in cells.</param>
        /// <returns>PNG bytes.</returns>
        static public Byte[] RenderPng
            (
            Grid Grid,
            FlowField? Flow,
            Int32 PixelsPerBin = 50,
            Int32 ArrowStride = 2
            )
        {
            // Render starts from the same diff source used by Python to keep palette parity.
            using Image<Rgba32> Image = DiffToImage(Grid.Delta(), Grid.Valid, Math.Max(1, PixelsPerBin));
            if ((Flow is not null) && (ArrowStride > 0))
            {
                DrawFlowArrows(Image, Grid, Flow, Math.Max(1, PixelsPerBin), ArrowStride);
            }

            DrawExclusions(Image, Grid, Math.Max(1, PixelsPerBin));
            return ToPng(Image);
        }


        /// <summary>
        /// Renders cut/fill base plus one weighted line per cut/fill pair.
        /// </summary>
        static public Byte[] RenderPassPairsPng
            (
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            Int32 PixelsPerBin = 50,
            Double BaseDim = 0.28
            )
        {
            // Heat accumulation highlights repeated haul corridors (Python render_pass_pairs_png).
            Int32 Ppb = Math.Max(1, Math.Min(PixelsPerBin, 16));
            using Image<Rgba32> Base = DiffToImageDigitize(Grid.Delta(), Grid.Valid, Ppb);
            DimTowardsWhite(Base, BaseDim);
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

            for (Int32 Iter = 0; Iter < Passes.Count; Iter += 1)
            {
                Pass Pass = Passes[Iter];
                (Double X0, Double Y0) = WorldToPixel(Grid, Pass.CutEndXy.X, Pass.CutEndXy.Y, Ppb);
                (Double X1, Double Y1) = WorldToPixel(Grid, Pass.FillStartXy.X, Pass.FillStartXy.Y, Ppb);
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
                    Byte Alpha = TruncByte(Math.Max(0.0, Math.Min(Constants.MAX_COLOR_CHANNEL, 150.0 + (T * 105.0))));
                    AlphaCompositePixel(Base, X, Y, Heat, Alpha);
                }
            }

            DrawExclusions(Base, Grid, Ppb);
            return ToPng(Base);
        }


        /// <summary>
        /// Renders guidance track over a faded cut/fill map.
        /// </summary>
        static public Byte[] RenderGuidancePng
            (
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            Int32 PixelsPerBin = 50,
            Double BladeWidthM = 2.271
            )
        {
            // Action-colored guidance confirms cut/fill/haul sequencing at blade scale.
            Int32 Ppb = Math.Max(1, Math.Min(PixelsPerBin, 16));
            using Image<Rgba32> Base = DiffToImageDigitize(Grid.Delta(), Grid.Valid, Ppb);
            DimTowardsWhite(Base, 0.30);
            Int32 BladePx = Math.Max(1, (Int32)Math.Round((BladeWidthM / Grid.GridSizeM) * Ppb, MidpointRounding.AwayFromZero));

            for (Int32 Iter = 1; Iter < Points.Count; Iter += 1)
            {
                GuidancePoint Prev = Points[Iter - 1];
                GuidancePoint Cur = Points[Iter];
                (Double X0, Double Y0) = WorldToPixel(Grid, Prev.X, Prev.Y, Ppb);
                (Double X1, Double Y1) = WorldToPixel(Grid, Cur.X, Cur.Y, Ppb);
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

            DrawExclusions(Base, Grid, Ppb);
            return ToPng(Base);
        }


        /// <summary>
        /// Renders one pass guidance pair and returns PNG bytes.
        /// Every cut cell is painted red and every fill cell blue. The white
        /// line on those cells is the blade centerline. The picture is cropped
        /// to that haul unless the whole field is requested.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Points">Guidance points, including the smoothed blade path.</param>
        /// <param name="BladeWidthM">Displayed blade width in metres.</param>
        /// <param name="PixelsPerBin">Pixels per grid cell.</param>
        /// <param name="PadCells">Crop padding in grid cells.</param>
        /// <param name="ShowRoute">Whether to draw the thin GPS trace.</param>
        /// <param name="CutCells">Cut cells to paint. Null leaves the cells unmarked.</param>
        /// <param name="FillCells">Fill cells to paint. Null leaves the cells unmarked.</param>
        /// <param name="ArrowX">Smoothed east component of the Poisson arrow. Null draws no arrows.</param>
        /// <param name="ArrowY">Smoothed north component of the Poisson arrow.</param>
        /// <param name="ArrowStride">Cells between arrows. One puts an arrow on every cell.</param>
        /// <param name="Surface">
        /// Existing-minus-target depth still in the ground, in metres, indexed
        /// [row, column]. Null paints the original survey. A route sheet passes
        /// the ground left after the loads already found.
        /// </param>
        /// <param name="EntireField">
        /// When true, arrows cover every cell and the picture keeps the whole
        /// grid. When false, the sheet is cropped to the haul.
        /// </param>
        /// <returns>PNG bytes.</returns>
        static public Byte[] RenderSinglePairPng
            (
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            Double BladeWidthM = 2.271,
            Int32 PixelsPerBin = 26,
            Int32 PadCells = 4,
            Boolean ShowRoute = true,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? CutCells = null,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? FillCells = null,
            Double[,]? ArrowX = null,
            Double[,]? ArrowY = null,
            Int32 ArrowStride = 1,
            Double[,]? Surface = null,
            Boolean EntireField = false
            )
        {
            // Cells are the work. The thick line is only the smoothed path the machine drives.
            Int32 Ppb = Math.Max(1, PixelsPerBin);
            // The sheet shows the ground this load is about to touch. Later loads
            // have not been taken off yet.
            using Image<Rgba32> Base = DiffToImageDigitize(Surface ?? Grid.Delta(), Grid.Valid, Ppb);
            DimTowardsWhite(Base, 0.45);
            PaintWorkCells(Base, Grid, CutCells, Ppb, new Rgba32(200, 0, 0, Constants.MAX_COLOR_CHANNEL));
            PaintWorkCells(Base, Grid, FillCells, Ppb, new Rgba32(0, 90, 220, Constants.MAX_COLOR_CHANNEL));
            // Arrows sit on the grade and on the work cells. The centerline is drawn
            // afterwards so the blade path stays visible on top of them.
            // Only the crop is stroked. An arrow on every cell of the whole field
            // is the same direction data and a much larger picture.
            if ((ArrowX != null) && (ArrowY != null) && (ArrowStride > 0))
            {
                // A haul crop would throw away the arrows on the rest of the field.
                (Int32 X0, Int32 Y0, Int32 X1, Int32 Y1) Window = EntireField
                    ? (0, 0, Grid.Width - 1, Grid.Height - 1)
                    : ArrowWindow(Grid, Points, CutCells, FillCells, PadCells);
                DrawSmoothedArrows(Base, Grid, ArrowX, ArrowY, Ppb, ArrowStride, Window);
            }

            Int32 BladePx = Math.Max(3, (Int32)Math.Round((BladeWidthM / Grid.GridSizeM) * Ppb, MidpointRounding.AwayFromZero));
            // The cells are the work. Once they are painted, the line is the
            // blade centerline, not a second copy of the blade width.
            Boolean CellsPainted = (CutCells != null) || (FillCells != null);
            Int32 PathPx = CellsPainted
                ? Math.Max(2, BladePx / 4)
                : BladePx;
            List<GuidancePoint> CutPts = new List<GuidancePoint>();
            List<GuidancePoint> FillPts = new List<GuidancePoint>();
            for (Int32 Iter = 0; Iter < Points.Count; Iter += 1)
            {
                GuidancePoint Point = Points[Iter];
                if (Point.Action == "CUT")
                {
                    CutPts.Add(Point);
                }

                if (Point.Action == "FILL")
                {
                    FillPts.Add(Point);
                }
            }

            // Separate blade-down runs stay separate. Joining them across a
            // blade-up turn draws the V the smoother just removed.
            // White sits on both the red cut and the blue fill.
            Rgba32 CutPath = CellsPainted
                ? new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL)
                : new Rgba32(200, 0, 0, Constants.MAX_COLOR_CHANNEL);
            Rgba32 FillPath = CellsPainted
                ? new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL)
                : new Rgba32(0, 90, 220, Constants.MAX_COLOR_CHANNEL);
            DrawWorkStrokes(Base, Grid, Points, "CUT", Ppb, CutPath, PathPx);
            DrawWorkStrokes(Base, Grid, Points, "FILL", Ppb, FillPath, PathPx);

            if ((CutPts.Count > 0) && (FillPts.Count > 0))
            {
                (Double Ax, Double Ay) = WorldToPixel(Grid, CutPts[^1].X, CutPts[^1].Y, Ppb);
                (Double Bx, Double By) = WorldToPixel(Grid, FillPts[0].X, FillPts[0].Y, Ppb);
                Int32 HaulWidth = Math.Max(2, BladePx / 2);
                DrawLine(Base, Ax, Ay, Bx, By, new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL), HaulWidth);
                Double Ang = Math.Atan2(By - Ay, Bx - Ax);
                Double Ah = Math.Max(8.0, BladePx * 1.6);
                foreach (Double Da in new Double[] { (150.0 * Math.PI / Constants.DEGREES_PER_HALF_TURN), (-150.0 * Math.PI / Constants.DEGREES_PER_HALF_TURN) })
                {
                    DrawLine(
                        Base,
                        Bx,
                        By,
                        (Bx + (Ah * Math.Cos(Ang + Da))),
                        (By + (Ah * Math.Sin(Ang + Da))),
                        new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL),
                        HaulWidth);
                }
            }

            if (ShowRoute && Points.Count >= 2)
            {
                List<(Double X, Double Y)> RouteXy = new List<(Double X, Double Y)>();
                for (Int32 Iter = 0; Iter < Points.Count; Iter += 1)
                {
                    RouteXy.Add(WorldToPixel(Grid, Points[Iter].X, Points[Iter].Y, Ppb));
                }

                Int32 RouteWidth = Math.Max(1, BladePx / 8);
                DrawPolylinePixels(Base, RouteXy, new Rgba32(35, 35, 35, Constants.MAX_COLOR_CHANNEL), RouteWidth);
                Int32 DotRadius = Math.Max(2, BladePx / 6);
                Double StepPx = Math.Max(1.0, 2.0 * Ppb);
                Double Acc = StepPx;
                (Double X, Double Y) Prev = RouteXy[0];
                DrawCircle(Base, Prev.X, Prev.Y, DotRadius, new Rgba32(Constants.MAX_COLOR_CHANNEL, 235, 0, Constants.MAX_COLOR_CHANNEL), new Rgba32(35, 35, 35, Constants.MAX_COLOR_CHANNEL));
                for (Int32 Iter = 1; Iter < RouteXy.Count; Iter += 1)
                {
                    (Double CurX, Double CurY) = RouteXy[Iter];
                    Double Dist = Math.Sqrt(((CurX - Prev.X) * (CurX - Prev.X)) + ((CurY - Prev.Y) * (CurY - Prev.Y)));
                    Acc += Dist;
                    if (Acc >= StepPx)
                    {
                        Acc = 0.0;
                        DrawCircle(Base, CurX, CurY, DotRadius, new Rgba32(Constants.MAX_COLOR_CHANNEL, 235, 0, Constants.MAX_COLOR_CHANNEL), new Rgba32(35, 35, 35, Constants.MAX_COLOR_CHANNEL));
                    }

                    Prev = (CurX, CurY);
                }
            }

            Int32 MarkerRadius = Math.Max(5, BladePx);
            // Green/orange mark true CUT start/end (blade-down only).
            if (CutPts.Count > 0)
            {
                DotGuidancePoint(Base, Grid, CutPts[0], Ppb, new Rgba32(0, 200, 0, Constants.MAX_COLOR_CHANNEL), MarkerRadius);
                DotGuidancePoint(Base, Grid, CutPts[^1], Ppb, new Rgba32(Constants.MAX_COLOR_CHANNEL, 140, 0, Constants.MAX_COLOR_CHANNEL), MarkerRadius);
            }

            // Cyan/purple mark true FILL start/end (not HAUL transit).
            if (FillPts.Count > 0)
            {
                DotGuidancePoint(Base, Grid, FillPts[0], Ppb, new Rgba32(0, 220, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL), MarkerRadius);
                DotGuidancePoint(Base, Grid, FillPts[^1], Ppb, new Rgba32(160, 0, 200, Constants.MAX_COLOR_CHANNEL), MarkerRadius);
            }

            List<Double> Xs = new List<Double>();
            List<Double> Ys = new List<Double>();
            for (Int32 Iter = 0; Iter < Points.Count; Iter += 1)
            {
                (Double Px, Double Py) = WorldToPixel(Grid, Points[Iter].X, Points[Iter].Y, Ppb);
                Xs.Add(Px);
                Ys.Add(Py);
            }
            IncludeWorkCells(Xs, Ys, Grid, CutCells, Ppb);
            IncludeWorkCells(Xs, Ys, Grid, FillCells, Ppb);

            if (EntireField)
            {
                return ToPng(Base);
            }

            Int32 Pad = PadCells * Ppb;
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
        /// Close-up of one area trip. The piece is outlined, the heading is an arrow,
        /// and each chosen band has one centreline over the cells that moved.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="WorkCells">Cells this trip moves. Null paints none.</param>
        /// <param name="Outline">Every cell of the piece. Null uses the work cells.</param>
        /// <param name="DirX">Haul heading east.</param>
        /// <param name="DirY">Haul heading north.</param>
        /// <param name="PixelsPerBin">Pixels per grid cell.</param>
        /// <param name="PadCells">Cells of ground kept around the piece.</param>
        /// <param name="Fill">True paints the work blue. False paints it red.</param>
        /// <param name="Surface">Ground still left, in metres. Null uses the grid delta.</param>
        /// <returns>PNG bytes.</returns>
        static public Byte[] RenderAreaPassPng
            (
            Grid Grid,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? WorkCells,
            IReadOnlyList<(Int32 Bx, Int32 By)>? Outline,
            Double DirX,
            Double DirY,
            Int32 PixelsPerBin,
            Int32 PadCells,
            Boolean Fill,
            Double[,]? Surface
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            Int32 Ppb = Math.Max(1, PixelsPerBin);
            using Image<Rgba32> Base = DiffToImageDigitize(Surface ?? Grid.Delta(), Grid.Valid, Ppb);
            DimTowardsWhite(Base, 0.45);
            Rgba32 Work = Fill
                ? new Rgba32(0, 90, 220, Constants.MAX_COLOR_CHANNEL)
                : new Rgba32(200, 0, 0, Constants.MAX_COLOR_CHANNEL);
            PaintWorkCells(Base, Grid, WorkCells, Ppb, Work);
            List<(Int32 Bx, Int32 By)> Piece = new List<(Int32 Bx, Int32 By)>();
            if ((Outline != null) && (Outline.Count > 0))
            {
                Piece.AddRange(Outline);
            }
            else if (WorkCells != null)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in WorkCells)
                {
                    Piece.Add(Lift.Cell);
                }
            }
            DrawPieceOutline(Base, Grid, Piece, Ppb);
            List<(Int32 Row, Int32 Col)> Rows = new List<(Int32 Row, Int32 Col)>(Piece.Count);
            foreach ((Int32 Bx, Int32 By) Cell in Piece)
            {
                Rows.Add((Cell.By, Cell.Bx));
            }
            Rgba32 Line = new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL);
            Rgba32 Arrow = new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL);
            List<Double> Xs = new List<Double>();
            List<Double> Ys = new List<Double>();
            foreach (StripLanes.PassLine Pass in StripLanes.BandLines(Grid, WorkCells, DirX, DirY))
            {
                (Double Px1, Double Py1) = WorldToPixel(Grid, Pass.X1, Pass.Y1, Ppb);
                (Double Px2, Double Py2) = WorldToPixel(Grid, Pass.X2, Pass.Y2, Ppb);
                DrawLine(Base, Px1, Py1, Px2, Py2, Line, 4);
                Xs.Add(Px1);
                Ys.Add(Py1);
                Xs.Add(Px2);
                Ys.Add(Py2);
            }
            StripLanes.PassLine Heading = StripLanes.HeadingArrow(Grid, Rows, DirX, DirY);
            if (Heading.HasLength)
            {
                (Double Ax1, Double Ay1) = WorldToPixel(Grid, Heading.X1, Heading.Y1, Ppb);
                (Double Ax2, Double Ay2) = WorldToPixel(Grid, Heading.X2, Heading.Y2, Ppb);
                DrawLine(Base, Ax1, Ay1, Ax2, Ay2, Arrow, 5);
                DrawArrowHead(Base, Ax1, Ay1, Ax2, Ay2, Arrow);
                Xs.Add(Ax1);
                Ys.Add(Ay1);
                Xs.Add(Ax2);
                Ys.Add(Ay2);
            }
            IncludeWorkCells(Xs, Ys, Grid, WorkCells, Ppb);
            foreach ((Int32 Bx, Int32 By) Cell in Piece)
            {
                if ((Cell.Bx < 0) || (Cell.By < 0) || (Cell.Bx >= Grid.Width) || (Cell.By >= Grid.Height))
                {
                    continue;
                }
                (Double X, Double Y) Center = Grid.CellCenterXy(Cell.Bx, Cell.By);
                (Double Px, Double Py) = WorldToPixel(Grid, Center.X, Center.Y, Ppb);
                Xs.Add(Px);
                Ys.Add(Py);
            }
            if ((Xs.Count == 0) || (Ys.Count == 0))
            {
                return ToPng(Base);
            }
            Int32 Pad = PadCells * Ppb;
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
        /// Strokes the outer edges of a piece. An edge with a neighbour in the piece is left open.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cells">The piece.</param>
        /// <param name="Ppb">Pixels per grid cell.</param>
        private static void DrawPieceOutline
            (
            Image<Rgba32> Image,
            Grid Grid,
            IReadOnlyList<(Int32 Bx, Int32 By)> Cells,
            Int32 Ppb
            )
        {
            HashSet<(Int32 Bx, Int32 By)> Inside = new HashSet<(Int32 Bx, Int32 By)>();
            foreach ((Int32 Bx, Int32 By) Cell in Cells)
            {
                Inside.Add(Cell);
            }
            Rgba32 Ink = new Rgba32(20, 20, 20, Constants.MAX_COLOR_CHANNEL);
            foreach ((Int32 Bx, Int32 By) Cell in Cells)
            {
                if ((Cell.Bx < 0) || (Cell.By < 0) || (Cell.Bx >= Grid.Width) || (Cell.By >= Grid.Height))
                {
                    continue;
                }
                Int32 Left = Cell.Bx * Ppb;
                Int32 Top = (Grid.Height - 1 - Cell.By) * Ppb;
                Int32 Right = Left + Ppb;
                Int32 Bottom = Top + Ppb;
                if (!Inside.Contains((Cell.Bx, Cell.By + 1)))
                {
                    DrawLine(Image, Left, Top, Right, Top, Ink, 3);
                }
                if (!Inside.Contains((Cell.Bx, Cell.By - 1)))
                {
                    DrawLine(Image, Left, Bottom, Right, Bottom, Ink, 3);
                }
                if (!Inside.Contains((Cell.Bx - 1, Cell.By)))
                {
                    DrawLine(Image, Left, Top, Left, Bottom, Ink, 3);
                }
                if (!Inside.Contains((Cell.Bx + 1, Cell.By)))
                {
                    DrawLine(Image, Right, Top, Right, Bottom, Ink, 3);
                }
            }
        }

        /// <summary>
        /// Two short strokes at the downstream end, so the heading reads as an arrow.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="X1">Upstream pixel easting.</param>
        /// <param name="Y1">Upstream pixel northing.</param>
        /// <param name="X2">Downstream pixel easting.</param>
        /// <param name="Y2">Downstream pixel northing.</param>
        /// <param name="Color">Arrow colour.</param>
        private static void DrawArrowHead
            (
            Image<Rgba32> Image,
            Double X1,
            Double Y1,
            Double X2,
            Double Y2,
            Rgba32 Color
            )
        {
            Double Dx = X2 - X1;
            Double Dy = Y2 - Y1;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Length < 1.0)
            {
                return;
            }
            Double Ux = Dx / Length;
            Double Uy = Dy / Length;
            Double Head = Math.Min(28.0, Length * 0.25);
            Double Wing = Head * 0.55;
            Double BackX = X2 - (Ux * Head);
            Double BackY = Y2 - (Uy * Head);
            DrawLine(Image, X2, Y2, BackX - (Uy * Wing), BackY + (Ux * Wing), Color, 4);
            DrawLine(Image, X2, Y2, BackX + (Uy * Wing), BackY - (Ux * Wing), Color, 4);
        }

        /// <summary>
        /// Close-up of one cut or one fill, with the blade width and the turns between lanes.
        /// Each straight run is a band one blade wide. The orange path is the turn into the next run.
        /// Numbers are the drive order. The green dot is the start and the orange dot is the finish.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Points">Guidance for this side. A MOVE breaks one lane from the next.</param>
        /// <param name="Action">CUT or FILL. The other actions are the turns.</param>
        /// <param name="BladeWidthM">Blade width in metres. The band is this wide.</param>
        /// <param name="TurnRadiusM">How far the turn rolls past a lane end before it can change direction.</param>
        /// <param name="PixelsPerBin">Pixels per grid cell.</param>
        /// <param name="PadCells">Cells of ground kept around the work and the turns.</param>
        /// <param name="CutCells">Cut cells. Null on a fill sheet.</param>
        /// <param name="FillCells">Fill cells. Null on a cut sheet.</param>
        /// <param name="ArrowX">Smoothed flow east. Null skips the arrows.</param>
        /// <param name="ArrowY">Smoothed flow north.</param>
        /// <param name="Surface">Ground still left, in metres. Null uses the grid delta.</param>
        /// <returns>PNG bytes.</returns>
        static public Byte[] RenderLaneDrivePng
            (
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            String Action,
            Double BladeWidthM,
            Double TurnRadiusM,
            Int32 PixelsPerBin,
            Int32 PadCells,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? CutCells,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? FillCells,
            Double[,]? ArrowX,
            Double[,]? ArrowY,
            Double[,]? Surface
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Points == null) throw new ArgumentNullException(nameof(Points));
            Int32 Ppb = Math.Max(1, PixelsPerBin);
            using Image<Rgba32> Base = DiffToImageDigitize(Surface ?? Grid.Delta(), Grid.Valid, Ppb);
            DimTowardsWhite(Base, 0.45);
            PaintWorkCells(Base, Grid, CutCells, Ppb, new Rgba32(200, 0, 0, Constants.MAX_COLOR_CHANNEL));
            PaintWorkCells(Base, Grid, FillCells, Ppb, new Rgba32(0, 90, 220, Constants.MAX_COLOR_CHANNEL));
            if ((ArrowX != null) && (ArrowY != null))
            {
                (Int32 X0, Int32 Y0, Int32 X1, Int32 Y1) Window = ArrowWindow(Grid, Points, CutCells, FillCells, PadCells);
                DrawSmoothedArrows(Base, Grid, ArrowX, ArrowY, Ppb, 1, Window);
            }

            // The band is the steel. A centerline alone does not show how wide the cut is.
            Int32 BladePx = Math.Max(3, (Int32)Math.Round((BladeWidthM / Grid.GridSizeM) * Ppb, MidpointRounding.AwayFromZero));
            List<List<GuidancePoint>> Lanes = LaneRuns(Points, Action);
            List<(Double X, Double Y)> TurnWorld = new List<(Double X, Double Y)>();
            Rgba32 Band = new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, 150);
            Rgba32 Edge = new Rgba32(20, 20, 20, Constants.MAX_COLOR_CHANNEL);
            Rgba32 Turn = new Rgba32(210, 90, 0, Constants.MAX_COLOR_CHANNEL);
            (Double X, Double Y) Centroid = LaneCentroid(Lanes);
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                DrawBladeBand(Base, Grid, Lanes[Index], Ppb, BladePx, Band, Edge);
            }
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                DrawLaneArrow(Base, Grid, Lanes[Index], Ppb, BladeWidthM);
                if (Index + 1 >= Lanes.Count) continue;
                // A short headland return. An 8 m loop on a lane only a few metres long covers the cut.
                List<(Double X, Double Y)> Path = TurnPath(
                    Lanes, Index, BladeWidthM, TurnRadiusM);
                TurnWorld.AddRange(Path);
                DrawWorldPolyline(Base, Grid, Path, Ppb, Turn, Math.Max(3, BladePx / 10));
                DrawOneArrow(Base, Grid, Path, Ppb, BladeWidthM * 0.7);
            }
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                DrawLaneNumber(Base, Grid, Lanes[Index], Ppb, BladePx, Index + 1, Centroid);
            }
            if (Lanes.Count > 0)
            {
                GuidancePoint First = Lanes[0][0];
                GuidancePoint Last = Lanes[Lanes.Count - 1][Lanes[Lanes.Count - 1].Count - 1];
                Int32 Marker = Math.Max(8, BladePx / 5);
                DotGuidancePoint(Base, Grid, First, Ppb, new Rgba32(0, 170, 0, Constants.MAX_COLOR_CHANNEL), Marker);
                DotGuidancePoint(Base, Grid, Last, Ppb, new Rgba32(Constants.MAX_COLOR_CHANNEL, 120, 0, Constants.MAX_COLOR_CHANNEL), Marker);
            }

            List<Double> Xs = new List<Double>();
            List<Double> Ys = new List<Double>();
            for (Int32 Iter = 0; Iter < Points.Count; Iter += 1)
            {
                (Double Px, Double Py) = WorldToPixel(Grid, Points[Iter].X, Points[Iter].Y, Ppb);
                Xs.Add(Px);
                Ys.Add(Py);
            }
            foreach ((Double X, Double Y) Bend in TurnWorld)
            {
                (Double Px, Double Py) = WorldToPixel(Grid, Bend.X, Bend.Y, Ppb);
                Xs.Add(Px);
                Ys.Add(Py);
            }
            IncludeWorkCells(Xs, Ys, Grid, CutCells, Ppb);
            IncludeWorkCells(Xs, Ys, Grid, FillCells, Ppb);
            if (Xs.Count == 0)
            {
                return ToPng(Base);
            }
            Int32 Pad = PadCells * Ppb;
            Int32 CropX0 = Math.Max(0, TruncTowardZero(Xs.Min() - Pad));
            Int32 CropY0 = Math.Max(0, TruncTowardZero(Ys.Min() - Pad));
            Int32 CropX1 = Math.Min(Base.Width, TruncTowardZero(Xs.Max() + Pad));
            Int32 CropY1 = Math.Min(Base.Height, TruncTowardZero(Ys.Max() + Pad));
            if ((CropX1 <= CropX0) || (CropY1 <= CropY0))
            {
                return ToPng(Base);
            }
            Base.Mutate(Context => Context.Crop(new Rectangle(CropX0, CropY0, CropX1 - CropX0, CropY1 - CropY0)));
            return ToPng(Base);
        }

        /// <summary>
        /// Straight blade-down runs. A MOVE, or any other action, ends the run.
        /// </summary>
        /// <param name="Points">Guidance in drive order.</param>
        /// <param name="Action">CUT or FILL.</param>
        /// <returns>One list per lane. A one-cell lane is a single point.</returns>
        private static List<List<GuidancePoint>> LaneRuns
            (
            IReadOnlyList<GuidancePoint> Points,
            String Action
            )
        {
            List<List<GuidancePoint>> Lanes = new List<List<GuidancePoint>>();
            List<GuidancePoint> Run = new List<GuidancePoint>();
            for (Int32 Index = 0; Index < Points.Count; Index += 1)
            {
                if (Points[Index].Action != Action)
                {
                    if (Run.Count > 0)
                    {
                        Lanes.Add(Run);
                        Run = new List<GuidancePoint>();
                    }
                    continue;
                }
                Run.Add(Points[Index]);
            }
            if (Run.Count > 0)
            {
                Lanes.Add(Run);
            }
            return Lanes;
        }

        /// <summary>
        /// Mean of the lane points. Turns bulge away from this so they stay off the cut.
        /// </summary>
        /// <param name="Lanes">The runs.</param>
        /// <returns>World position. The origin when there are no points.</returns>
        private static (Double X, Double Y) LaneCentroid
            (
            List<List<GuidancePoint>> Lanes
            )
        {
            Double X = 0.0;
            Double Y = 0.0;
            Int32 Count = 0;
            foreach (List<GuidancePoint> Lane in Lanes)
            {
                foreach (GuidancePoint Point in Lane)
                {
                    X += Point.X;
                    Y += Point.Y;
                    Count += 1;
                }
            }
            if (Count == 0) return (0.0, 0.0);
            return (X / Count, Y / Count);
        }

        /// <summary>
        /// Paints one lane as a band the width of the blade, with a dark edge on each side.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Lane">Cells of one straight run, in drive order.</param>
        /// <param name="Ppb">Pixels per cell.</param>
        /// <param name="BladePx">Blade width in pixels.</param>
        /// <param name="Band">See-through fill of the swath.</param>
        /// <param name="Edge">The two sides of the blade.</param>
        private static void DrawBladeBand
            (
            Image<Rgba32> Image,
            Grid Grid,
            List<GuidancePoint> Lane,
            Int32 Ppb,
            Int32 BladePx,
            Rgba32 Band,
            Rgba32 Edge
            )
        {
            if (Lane.Count == 0) return;
            (Double X, Double Y) Start = WorldToPixel(Grid, Lane[0].X, Lane[0].Y, Ppb);
            GuidancePoint EndPoint = Lane[Lane.Count - 1];
            (Double X, Double Y) End = WorldToPixel(Grid, EndPoint.X, EndPoint.Y, Ppb);
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            // A one-cell lane is a disk the width of the blade. It still has a directionless cut.
            if (Length < 1.0)
            {
                BlendDisk(Image, Start.X, Start.Y, BladePx / 2.0, Band);
                DrawCircle(Image, Start.X, Start.Y, Math.Max(2, BladePx / 2), Band, Edge);
                return;
            }
            Double Ux = Dx / Length;
            Double Uy = Dy / Length;
            Double Px = -Uy;
            Double Py = Ux;
            Double Half = BladePx / 2.0;
            // Step across the band. The red or blue cell stays visible through the white.
            for (Double Along = -Half; Along <= (Length + Half); Along += 1.0)
            {
                for (Double Across = -Half; Across <= Half; Across += 1.0)
                {
                    BlendPlot(
                        Image,
                        (Int32)Math.Round(Start.X + (Ux * Along) + (Px * Across)),
                        (Int32)Math.Round(Start.Y + (Uy * Along) + (Py * Across)),
                        Band);
                }
            }
            DrawLine(
                Image,
                Start.X + (Px * Half),
                Start.Y + (Py * Half),
                End.X + (Px * Half),
                End.Y + (Py * Half),
                Edge,
                3);
            DrawLine(
                Image,
                Start.X - (Px * Half),
                Start.Y - (Py * Half),
                End.X - (Px * Half),
                End.Y - (Py * Half),
                Edge,
                3);
        }

        /// <summary>
        /// One arrow on the lane, pointing the way the blade travels.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Lane">The run.</param>
        /// <param name="Ppb">Pixels per cell.</param>
        /// <param name="BladeM">Blade width. The arrow is about that long.</param>
        private static void DrawLaneArrow
            (
            Image<Rgba32> Image,
            Grid Grid,
            List<GuidancePoint> Lane,
            Int32 Ppb,
            Double BladeM
            )
        {
            if (Lane.Count < 2) return;
            GuidancePoint A = Lane[0];
            GuidancePoint B = Lane[Lane.Count - 1];
            (Double X, Double Y) Dir = Travel(Lane);
            Double Run = Distance((A.X, A.Y), (B.X, B.Y));
            // Sit the arrow in the middle of a long lane, and near the end of a short one.
            Double Reach = Math.Min(Run * 0.7, Math.Max(BladeM * 0.8, Run * 0.45));
            (Double X, Double Y) Tip = (A.X + (Dir.X * Reach), A.Y + (Dir.Y * Reach));
            (Double X, Double Y) Tail = (A.X + (Dir.X * Reach * 0.35), A.Y + (Dir.Y * Reach * 0.35));
            DrawOneArrow(Image, Grid, new[] { Tail, Tip }, Ppb, BladeM * 0.55);
        }

        /// <summary>
        /// The path from the end of one lane to the start of the next.
        /// Same-direction lanes return through the headland, each turn a little further out.
        /// A U-turn is a short hook just past the two ends.
        /// </summary>
        /// <param name="Lanes">Every run, so the return can stay outside them.</param>
        /// <param name="Index">The lane just finished.</param>
        /// <param name="BladeM">Blade width. The headland offset steps by a fraction of this.</param>
        /// <param name="RadiusM">Not used to size the picture. The machine's radius is larger than these lanes.</param>
        /// <returns>World points of the turn.</returns>
        private static List<(Double X, Double Y)> TurnPath
            (
            List<List<GuidancePoint>> Lanes,
            Int32 Index,
            Double BladeM,
            Double RadiusM
            )
        {
            List<GuidancePoint> Done = Lanes[Index];
            List<GuidancePoint> Next = Lanes[Index + 1];
            (Double X, Double Y) End = (Done[Done.Count - 1].X, Done[Done.Count - 1].Y);
            (Double X, Double Y) Start = (Next[0].X, Next[0].Y);
            (Double X, Double Y) Leave = Travel(Done);
            (Double X, Double Y) Enter = Travel(Next);
            Double Out = Math.Max(BladeM * 0.8, 1.0) + (Index * BladeM * 0.45);
            Boolean SameWay = ((Leave.X * Enter.X) + (Leave.Y * Enter.Y)) > 0.2;
            Double Ahead = ((Start.X - End.X) * Leave.X) + ((Start.Y - End.Y) * Leave.Y);
            // Returning to the other end of the patch. The path stays past the ends and down one side.
            if (SameWay && (Ahead < 0.0))
            {
                (Double X, Double Y) Side = HeadlandSide(Leave, End, Start, LaneBox(Lanes));
                Double FarLeave = Extreme(Lanes, Leave);
                Double FarSide = Extreme(Lanes, Side);
                Double EndSide = (End.X * Side.X) + (End.Y * Side.Y);
                Double StartLeave = (Start.X * Leave.X) + (Start.Y * Leave.Y);
                Double StartSide = (Start.X * Side.X) + (Start.Y * Side.Y);
                return new List<(Double X, Double Y)>
                {
                    End,
                    At(Leave, Side, FarLeave + Out, EndSide),
                    At(Leave, Side, FarLeave + Out, FarSide + Out),
                    At(Leave, Side, StartLeave - (BladeM * 0.6), FarSide + Out),
                    At(Leave, Side, StartLeave - (BladeM * 0.6), StartSide),
                    Start,
                };
            }
            // The next lane starts at this same end. A hook just clears the two blades.
            _ = RadiusM;
            (Double X, Double Y) Perp = (-Leave.Y, Leave.X);
            Double ToStart = ((Start.X - End.X) * Perp.X) + ((Start.Y - End.Y) * Perp.Y);
            if (ToStart < 0.0)
            {
                Perp = (Leave.Y, -Leave.X);
            }
            Double Hook = Math.Max(BladeM, Distance(End, Start) * 0.5);
            return new List<(Double X, Double Y)>
            {
                End,
                (End.X + (Leave.X * Hook * 0.6) + (Perp.X * Hook), End.Y + (Leave.Y * Hook * 0.6) + (Perp.Y * Hook)),
                (Start.X - (Enter.X * Hook * 0.6) + (Perp.X * Hook), Start.Y - (Enter.Y * Hook * 0.6) + (Perp.Y * Hook)),
                Start,
            };
        }

        /// <summary>
        /// A world point from coordinates along travel and across it.
        /// The two axes are unit length and perpendicular.
        /// </summary>
        /// <param name="Leave">Travel axis.</param>
        /// <param name="Side">Cross-track axis.</param>
        /// <param name="Along">Coordinate on the travel axis.</param>
        /// <param name="Across">Coordinate on the cross-track axis.</param>
        /// <returns>The world point.</returns>
        private static (Double X, Double Y) At
            (
            (Double X, Double Y) Leave,
            (Double X, Double Y) Side,
            Double Along,
            Double Across
            )
        {
            return ((Leave.X * Along) + (Side.X * Across), (Leave.Y * Along) + (Side.Y * Across));
        }

        /// <summary>
        /// The furthest lane point along an axis.
        /// </summary>
        /// <param name="Lanes">The runs.</param>
        /// <param name="Axis">A unit axis.</param>
        /// <returns>The largest projection.</returns>
        private static Double Extreme
            (
            List<List<GuidancePoint>> Lanes,
            (Double X, Double Y) Axis
            )
        {
            Double Far = Double.MinValue;
            foreach (List<GuidancePoint> Lane in Lanes)
            {
                foreach (GuidancePoint Point in Lane)
                {
                    Double Projected = (Point.X * Axis.X) + (Point.Y * Axis.Y);
                    if (Projected > Far) Far = Projected;
                }
            }
            return Far;
        }

        /// <summary>
        /// Which way is outside the patch, perpendicular to travel.
        /// The return drops down that side instead of across the lanes.
        /// </summary>
        /// <param name="Leave">Direction the finished lane was driven.</param>
        /// <param name="End">Where that lane stopped.</param>
        /// <param name="Start">Where the next lane starts.</param>
        /// <param name="Box">Extent of every lane.</param>
        /// <returns>A unit step toward the outside.</returns>
        private static (Double X, Double Y) HeadlandSide
            (
            (Double X, Double Y) Leave,
            (Double X, Double Y) End,
            (Double X, Double Y) Start,
            (Double MinX, Double MinY, Double MaxX, Double MaxY) Box
            )
        {
            (Double X, Double Y) Left = (-Leave.Y, Leave.X);
            (Double X, Double Y) Mid = ((Box.MinX + Box.MaxX) * 0.5, (Box.MinY + Box.MaxY) * 0.5);
            Double LeftOfCenter = ((End.X - Mid.X) * Left.X) + ((End.Y - Mid.Y) * Left.Y);
            Double StartOfCenter = ((Start.X - Mid.X) * Left.X) + ((Start.Y - Mid.Y) * Left.Y);
            // Prefer the side the next lane already sits on, so the drop is short.
            if ((StartOfCenter * LeftOfCenter) < 0.0)
            {
                return StartOfCenter >= 0.0 ? Left : (-Left.X, -Left.Y);
            }
            return LeftOfCenter >= 0.0 ? Left : (-Left.X, -Left.Y);
        }

        /// <summary>
        /// The extent of every lane, in world metres.
        /// </summary>
        /// <param name="Lanes">The runs.</param>
        /// <returns>Minimum and maximum east and north.</returns>
        private static (Double MinX, Double MinY, Double MaxX, Double MaxY) LaneBox
            (
            List<List<GuidancePoint>> Lanes
            )
        {
            Double MinX = Double.MaxValue;
            Double MinY = Double.MaxValue;
            Double MaxX = Double.MinValue;
            Double MaxY = Double.MinValue;
            foreach (List<GuidancePoint> Lane in Lanes)
            {
                foreach (GuidancePoint Point in Lane)
                {
                    if (Point.X < MinX) MinX = Point.X;
                    if (Point.Y < MinY) MinY = Point.Y;
                    if (Point.X > MaxX) MaxX = Point.X;
                    if (Point.Y > MaxY) MaxY = Point.Y;
                }
            }
            return (MinX, MinY, MaxX, MaxY);
        }

        /// <summary>
        /// One arrowhead at the end of a path.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Path">World points. The head sits on the last point.</param>
        /// <param name="Ppb">Pixels per cell.</param>
        /// <param name="LengthM">How long the head is, in metres.</param>
        private static void DrawOneArrow
            (
            Image<Rgba32> Image,
            Grid Grid,
            IReadOnlyList<(Double X, Double Y)> Path,
            Int32 Ppb,
            Double LengthM
            )
        {
            if (Path.Count < 2) return;
            (Double X, Double Y) Tip = Path[Path.Count - 1];
            (Double X, Double Y) From = Path[Path.Count - 2];
            Double Dx = Tip.X - From.X;
            Double Dy = Tip.Y - From.Y;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Length < 1e-6) return;
            (Double X, Double Y) TipPx = WorldToPixel(Grid, Tip.X, Tip.Y, Ppb);
            Double UnitX = (Dx / Length);
            Double UnitY = -(Dy / Length);
            Double Head = Math.Max(10.0, (LengthM / Grid.GridSizeM) * Ppb);
            Double Half = Head * 0.45;
            Double BaseX = TipPx.X - (UnitX * Head);
            Double BaseY = TipPx.Y - (UnitY * Head);
            DrawLine(Image, TipPx.X, TipPx.Y, BaseX + (-UnitY * Half), BaseY + (UnitX * Half), new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL), 4);
            DrawLine(Image, TipPx.X, TipPx.Y, BaseX - (-UnitY * Half), BaseY - (UnitX * Half), new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL), 4);
        }

        /// <summary>
        /// Unit step from the first cell of a lane toward the last. A one-cell lane faces east.
        /// </summary>
        /// <param name="Lane">The run.</param>
        /// <returns>A unit vector in world metres.</returns>
        private static (Double X, Double Y) Travel
            (
            List<GuidancePoint> Lane
            )
        {
            if (Lane.Count < 2) return (1.0, 0.0);
            Double Dx = Lane[Lane.Count - 1].X - Lane[0].X;
            Double Dy = Lane[Lane.Count - 1].Y - Lane[0].Y;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Length < 1e-6) return (1.0, 0.0);
            return (Dx / Length, Dy / Length);
        }

        /// <summary>
        /// The lane number, sitting just outside the blade so it does not cover the cut.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Lane">The run.</param>
        /// <param name="Ppb">Pixels per cell.</param>
        /// <param name="BladePx">Blade width in pixels.</param>
        /// <param name="Number">1-based drive order.</param>
        /// <param name="Centroid">Middle of the work. The badge sits on the far side of the lane.</param>
        private static void DrawLaneNumber
            (
            Image<Rgba32> Image,
            Grid Grid,
            List<GuidancePoint> Lane,
            Int32 Ppb,
            Int32 BladePx,
            Int32 Number,
            (Double X, Double Y) Centroid
            )
        {
            // The number sits just before the blade starts, on the left of travel.
            (Double X, Double Y) Dir = Travel(Lane);
            (Double X, Double Y) Left = (-Dir.Y, Dir.X);
            Double Back = Math.Max(Grid.GridSizeM * 0.85, 1.0);
            (Double X, Double Y) Place = (
                Lane[0].X - (Dir.X * Back) + (Left.X * Grid.GridSizeM * 0.15),
                Lane[0].Y - (Dir.Y * Back) + (Left.Y * Grid.GridSizeM * 0.15));
            (Double X, Double Y) At = WorldToPixel(Grid, Place.X, Place.Y, Ppb);
            Double X = At.X;
            Double Y = At.Y;
            _ = Centroid;
            _ = BladePx;
            DrawCircle(
                Image,
                X,
                Y,
                16,
                new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL),
                new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL));
            DrawScaledText(Image, (Int32)Math.Round(X), (Int32)Math.Round(Y), Number.ToString(CultureInfo.InvariantCulture), 3);
        }

        /// <summary>
        /// Bitmap text drawn several pixels per glyph bit, so a lane number is readable on a close-up.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="CenterX">Horizontal centre.</param>
        /// <param name="CenterY">Vertical centre.</param>
        /// <param name="Text">Digits.</param>
        /// <param name="Scale">Pixels per glyph bit.</param>
        private static void DrawScaledText
            (
            Image<Rgba32> Image,
            Int32 CenterX,
            Int32 CenterY,
            String Text,
            Int32 Scale
            )
        {
            if (String.IsNullOrWhiteSpace(Text)) return;
            Int32 Step = 4 * Scale;
            Int32 OriginX = CenterX - ((Text.Length * Step) / 2);
            for (Int32 Index = 0; Index < Text.Length; Index += 1)
            {
                DrawScaledGlyph(
                    Image,
                    Text[Index],
                    OriginX + (Index * Step),
                    CenterY - ((5 * Scale) / 2),
                    Scale);
            }
        }

        /// <summary>
        /// One digit, scaled up from the 3 by 5 glyph.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Ch">The digit.</param>
        /// <param name="X">Left of the glyph.</param>
        /// <param name="Y">Top of the glyph.</param>
        /// <param name="Scale">Pixels per glyph bit.</param>
        private static void DrawScaledGlyph
            (
            Image<Rgba32> Image,
            Char Ch,
            Int32 X,
            Int32 Y,
            Int32 Scale
            )
        {
            String[] Glyph = Ch switch
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
            Rgba32 Ink = new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL);
            for (Int32 Row = 0; Row < Glyph.Length; Row += 1)
            {
                String Line = Glyph[Row];
                for (Int32 Col = 0; Col < Line.Length; Col += 1)
                {
                    if (Line[Col] != '1') continue;
                    for (Int32 Sy = 0; Sy < Scale; Sy += 1)
                    {
                        for (Int32 Sx = 0; Sx < Scale; Sx += 1)
                        {
                            Plot(Image, X + (Col * Scale) + Sx, Y + (Row * Scale) + Sy, Ink);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Writes a pixel over the colour already there. The work cells stay visible inside the blade band.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="X">Pixel column.</param>
        /// <param name="Y">Pixel row.</param>
        /// <param name="Color">Colour to lay on. Its alpha is the mix.</param>
        private static void BlendPlot
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Rgba32 Color
            )
        {
            if ((X < 0) || (Y < 0) || (X >= Image.Width) || (Y >= Image.Height)) return;
            Rgba32 Under = Image[X, Y];
            Double Mix = Color.A / (Double)Constants.MAX_COLOR_CHANNEL;
            Image[X, Y] = new Rgba32(
                TruncByte((Under.R * (1.0 - Mix)) + (Color.R * Mix)),
                TruncByte((Under.G * (1.0 - Mix)) + (Color.G * Mix)),
                TruncByte((Under.B * (1.0 - Mix)) + (Color.B * Mix)),
                Constants.MAX_COLOR_CHANNEL);
        }

        /// <summary>
        /// A filled disk blended over the ground. Used for a one-cell lane.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Cx">Centre pixel.</param>
        /// <param name="Cy">Centre pixel.</param>
        /// <param name="Radius">Radius in pixels.</param>
        /// <param name="Color">Band colour.</param>
        private static void BlendDisk
            (
            Image<Rgba32> Image,
            Double Cx,
            Double Cy,
            Double Radius,
            Rgba32 Color
            )
        {
            Int32 MinX = (Int32)Math.Floor(Cx - Radius);
            Int32 MaxX = (Int32)Math.Ceiling(Cx + Radius);
            Int32 MinY = (Int32)Math.Floor(Cy - Radius);
            Int32 MaxY = (Int32)Math.Ceiling(Cy + Radius);
            Double Limit = Radius * Radius;
            for (Int32 Y = MinY; Y <= MaxY; Y += 1)
            {
                for (Int32 X = MinX; X <= MaxX; X += 1)
                {
                    Double Dx = X - Cx;
                    Double Dy = Y - Cy;
                    if (((Dx * Dx) + (Dy * Dy)) <= Limit)
                    {
                        BlendPlot(Image, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Distance between two world points.
        /// </summary>
        /// <param name="A">First point.</param>
        /// <param name="B">Second point.</param>
        /// <returns>Metres.</returns>
        private static Double Distance
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Renders explicitly separated coverage runs for diagnostic comparison.
        /// </summary>
        /// <param name="Grid">Grid geometry and grade surface.</param>
        /// <param name="CutRuns">Independent straight cut polylines.</param>
        /// <param name="FillRuns">Independent straight fill polylines.</param>
        /// <param name="RoutePoints">Complete thin diagnostic route.</param>
        /// <param name="BladeWidthM">Displayed blade width in metres.</param>
        /// <param name="PixelsPerBin">Pixels per grid bin.</param>
        /// <param name="PadCells">Crop padding in grid cells.</param>
        /// <param name="ManualAssistRuns">Explicit operator-alignment polylines.</param>
        /// <param name="RouteMarkers">Optional numbered route marker locations.</param>
        /// <param name="DrawDirectionArrows">Whether to draw arrowheads along the complete route.</param>
        /// <param name="DirectionArrowSpacingM">Distance between route arrowheads in metres.</param>
        /// <param name="BoundsPoints">Optional non-drawn points used to keep animation crops stable.</param>
        /// <param name="Surface">
        /// Existing-minus-target still in the ground, in metres, indexed [row, column].
        /// Null paints the original survey.
        /// </param>
        /// <returns>Cropped PNG bytes.</returns>
        static public Byte[] RenderCoverageRunsPng
            (
            Grid Grid,
            IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> CutRuns,
            IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> FillRuns,
            IReadOnlyList<(Double X, Double Y)> RoutePoints,
            Double BladeWidthM = 2.271,
            Int32 PixelsPerBin = 26,
            Int32 PadCells = 4,
            IReadOnlyList<IReadOnlyList<(Double X, Double Y)>>? ManualAssistRuns = null,
            IReadOnlyList<((Double X, Double Y) Point, String Label)>? RouteMarkers = null,
            Boolean DrawDirectionArrows = false,
            Double DirectionArrowSpacingM = 0.0,
            IReadOnlyList<(Double X, Double Y)>? BoundsPoints = null,
            Double[,]? Surface = null
            )
        {
            // Draw each work run independently so blade-up repositioning never becomes a thick work overlay.
            Int32 Ppb = Math.Max(1, PixelsPerBin);
            // A haul sheet shows the ground this load is about to touch.
            using Image<Rgba32> Base = DiffToImageDigitize(Surface ?? Grid.Delta(), Grid.Valid, Ppb);
            for (Int32 Y = 0; Y < Base.Height; Y += 1)
            {
                for (Int32 X = 0; X < Base.Width; X += 1)
                {
                    if (Base[X, Y].A == 0)
                    {
                        Base[X, Y] = new Rgba32(238, 238, 238, Constants.MAX_COLOR_CHANNEL);
                    }
                }
            }
            DimTowardsWhite(Base, 0.45);
            Int32 BladePx = Math.Max(
                3,
                (Int32)Math.Round(
                    (BladeWidthM / Grid.GridSizeM) * Ppb,
                    MidpointRounding.AwayFromZero));

            foreach (IReadOnlyList<(Double X, Double Y)> Run in CutRuns)
            {
                DrawWorldPolyline(
                    Base, Grid, Run, Ppb, new Rgba32(200, 0, 0, Constants.MAX_COLOR_CHANNEL), BladePx);
            }
            foreach (IReadOnlyList<(Double X, Double Y)> Run in FillRuns)
            {
                DrawWorldPolyline(
                    Base, Grid, Run, Ppb, new Rgba32(0, 90, 220, Constants.MAX_COLOR_CHANNEL), BladePx);
            }
            if (RoutePoints.Count >= 2)
            {
                Int32 RouteWidth = Math.Max(1, BladePx / 8);
                DrawWorldPolyline(
                    Base, Grid, RoutePoints, Ppb, new Rgba32(35, 35, 35, Constants.MAX_COLOR_CHANNEL), RouteWidth);
            }
            if (ManualAssistRuns != null)
            {
                Int32 AssistWidth = Math.Max(2, BladePx / 5);
                foreach (IReadOnlyList<(Double X, Double Y)> Run in ManualAssistRuns)
                {
                    DrawWorldPolyline(
                        Base, Grid, Run, Ppb, new Rgba32(Constants.MAX_COLOR_CHANNEL, 140, 0, Constants.MAX_COLOR_CHANNEL), AssistWidth);
                }
            }
            if (DrawDirectionArrows && (RoutePoints.Count >= 2))
            {
                Double SpacingM = DirectionArrowSpacingM > 0.0
                    ? DirectionArrowSpacingM
                    : 6.0 * Grid.GridSizeM;
                DrawRouteDirectionArrows(Base, Grid, RoutePoints, Ppb, SpacingM);
            }
            if (RouteMarkers != null)
            {
                Int32 MarkerRadius = Math.Max(8, BladePx / 2);
                foreach (((Double X, Double Y) Point, String Label) Marker in RouteMarkers)
                {
                    (Double Px, Double Py) = WorldToPixel(
                        Grid, Marker.Point.X, Marker.Point.Y, Ppb);
                    DrawCircle(
                        Base, Px, Py, MarkerRadius,
                        new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, 235),
                        new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL));
                    DrawTextDigit(
                        Base,
                        (Int32)Math.Round(Px),
                        (Int32)Math.Round(Py),
                        Marker.Label);
                }
            }

            List<(Double X, Double Y)> Bounds = new List<(Double X, Double Y)>(RoutePoints);
            foreach (IReadOnlyList<(Double X, Double Y)> Run in CutRuns)
            {
                Bounds.AddRange(Run);
            }
            foreach (IReadOnlyList<(Double X, Double Y)> Run in FillRuns)
            {
                Bounds.AddRange(Run);
            }
            if (ManualAssistRuns != null)
            {
                foreach (IReadOnlyList<(Double X, Double Y)> Run in ManualAssistRuns)
                {
                    Bounds.AddRange(Run);
                }
            }
            if (RouteMarkers != null)
            {
                Bounds.AddRange(RouteMarkers.Select(Marker => Marker.Point));
            }
            if (BoundsPoints != null)
            {
                Bounds.AddRange(BoundsPoints);
            }
            if (Bounds.Count == 0) return ToPng(Base);

            List<(Double X, Double Y)> Pixels = Bounds.Select(Point =>
                WorldToPixel(Grid, Point.X, Point.Y, Ppb)).ToList();
            Int32 Pad = PadCells * Ppb;
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
        /// Renders a consecutive route range from one connected lifecycle with
        /// cut runs in red, fill runs in blue, and numbered route markers.
        /// </summary>
        /// <param name="Lifecycle">Final connected guidance lifecycle.</param>
        /// <param name="Grid">Field grid used for projection.</param>
        /// <param name="FirstRouteNumber">First 1-based route index in the sequence.</param>
        /// <param name="RouteCount">Number of consecutive routes to include.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="PixelsPerBin">Render scale in pixels per grid cell.</param>
        /// <param name="PadCells">Crop padding in cells.</param>
        /// <param name="Surface">
        /// Existing-minus-target still in the ground, in metres, indexed [row, column].
        /// Null paints the original survey.
        /// </param>
        /// <returns>Cropped PNG bytes.</returns>
        static public Byte[] RenderConsecutiveRouteSequencePng
            (
            LifecyclePlanner.Result Lifecycle,
            Grid Grid,
            Int32 FirstRouteNumber,
            Int32 RouteCount,
            Double BladeWidthM,
            Int32 PixelsPerBin = 24,
            Int32 PadCells = 8,
            Double[,]? Surface = null
            )
        {
            if (Lifecycle == null) throw new ArgumentNullException(nameof(Lifecycle));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (RouteCount < 1)
            {
                throw new ArgumentOutOfRangeException(nameof(RouteCount));
            }

            Int32 LastRouteNumber = FirstRouteNumber + RouteCount - 1;
            HashSet<Int32> RouteNumbers = Enumerable.Range(
                FirstRouteNumber,
                LastRouteNumber - FirstRouteNumber + 1).ToHashSet();
            List<LifecyclePlanner.Segment> Segments = Lifecycle.Segments
                .Where(Segment => RouteNumbers.Contains(Segment.RouteNumber))
                .ToList();
            List<IReadOnlyList<(Double X, Double Y)>> CutRuns = Segments
                .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.Cut)
                .Select(Segment =>
                    (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
                .ToList();
            List<IReadOnlyList<(Double X, Double Y)>> FillRuns = Segments
                .Where(Segment => Segment.Phase == LifecyclePlanner.Phase.Fill)
                .Select(Segment =>
                    (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
                .ToList();
            List<IReadOnlyList<(Double X, Double Y)>> ManualRuns = Segments
                .Where(Segment =>
                    Segment.Phase == LifecyclePlanner.Phase.ManualAssist)
                .Select(Segment =>
                    (IReadOnlyList<(Double X, Double Y)>)Segment.Points)
                .ToList();
            List<(Double X, Double Y)> RoutePoints = Segments
                .SelectMany(Segment => Segment.Points)
                .ToList();
            List<((Double X, Double Y) Point, String Label)> Markers =
                new List<((Double X, Double Y) Point, String Label)>();
            foreach (Int32 RouteNumber in RouteNumbers.OrderBy(Value => Value))
            {
                LifecyclePlanner.Segment? Cut = Segments.FirstOrDefault(
                    Segment => Segment.RouteNumber == RouteNumber
                        && Segment.Phase == LifecyclePlanner.Phase.Cut
                        && Segment.Points.Count > 0);
                LifecyclePlanner.Segment? Any = Segments.FirstOrDefault(
                    Segment => Segment.RouteNumber == RouteNumber
                        && Segment.Points.Count > 0);
                LifecyclePlanner.Segment? MarkerSegment = Cut ?? Any;
                if (MarkerSegment == null)
                {
                    continue;
                }

                Markers.Add((
                    MarkerSegment.Points[MarkerSegment.Points.Count / 2],
                    RouteNumber.ToString(CultureInfo.InvariantCulture)));
            }

            return RenderCoverageRunsPng(
                Grid,
                CutRuns,
                FillRuns,
                RoutePoints,
                BladeWidthM,
                PixelsPerBin,
                PadCells,
                ManualRuns,
                Markers,
                DrawDirectionArrows: true,
                DirectionArrowSpacingM: 6.0 * Grid.GridSizeM,
                Surface: Surface);
        }

        /// <summary>
        /// Converts a world polyline to pixels and draws it at a fixed width.
        /// </summary>
        /// <param name="Image">Destination image.</param>
        /// <param name="Grid">Grid projection.</param>
        /// <param name="Points">World polyline.</param>
        /// <param name="PixelsPerBin">Pixels per grid bin.</param>
        /// <param name="Color">Line color.</param>
        /// <param name="Width">Line width in pixels.</param>
        private static void DrawWorldPolyline
            (
            Image<Rgba32> Image,
            Grid Grid,
            IReadOnlyList<(Double X, Double Y)> Points,
            Int32 PixelsPerBin,
            Rgba32 Color,
            Int32 Width
            )
        {
            if (Points.Count < 2) return;

            // Reuse the established raster primitive after one coordinate conversion pass.
            List<(Double X, Double Y)> Pixels = Points.Select(Point =>
                WorldToPixel(Grid, Point.X, Point.Y, PixelsPerBin)).ToList();
            DrawPolylinePixels(Image, Pixels, Color, Width);
        }

        /// <summary>
        /// Renders several consecutive pass pairs as one operator-track figure.
        /// </summary>
        static public Byte[] RenderPairSequencePng
            (
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            Double BladeWidthM = 2.271,
            Int32 PixelsPerBin = 26
            )
        {
            // Multi-pair sequence uses the same action coloring to show transitions across adjacent passes.
            return RenderGuidancePng(Grid, Points, PixelsPerBin, BladeWidthM);
        }


        /// <summary>
        /// Overlays numbered zone waypoints onto an existing PNG buffer.
        /// </summary>
        /// <param name="PngBytes">Base PNG bytes.</param>
        /// <param name="Grid">Grid geometry for world->pixel projection.</param>
        /// <param name="Steps">Ordered zone steps.</param>
        /// <param name="PixelsPerBin">Pixels per bin for projection math.</param>
        /// <returns>Updated PNG bytes.</returns>
        static public Byte[] OverlayZoneWaypoints
            (
            Byte[] PngBytes,
            Grid Grid,
            IReadOnlyList<ZoneStep> Steps,
            Int32 PixelsPerBin = 50
            )
        {
            // Overlay is done in-place on decoded PNG bytes so engine can stay fully in-memory.
            using Image<Rgba32> OverlayImage = SixLabors.ImageSharp.Image.Load<Rgba32>(PngBytes);
            if (Steps.Count <= 0)
            {
                return ToPng(OverlayImage);
            }

            Int32 Ppb = Math.Max(1, PixelsPerBin);
            for (Int32 Iter = 0; Iter < Steps.Count; Iter += 1)
            {
                ZoneStep Step = Steps[Iter];
                (Double Px, Double Py) = WorldToPixel(Grid, Step.Easting, Step.Northing, Ppb);
                DrawCircle(OverlayImage, Px, Py, Math.Max(4, Ppb / 2), new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, 235), new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL));
                DrawTextDigit(OverlayImage, (Int32)Math.Round(Px), (Int32)Math.Round(Py), ((Int32)Step.WorkOrder).ToString(CultureInfo.InvariantCulture));
            }

            for (Int32 Iter = 1; Iter < Steps.Count; Iter += 1)
            {
                ZoneStep Prev = Steps[Iter - 1];
                ZoneStep Cur = Steps[Iter];
                (Double X0, Double Y0) = WorldToPixel(Grid, Prev.Easting, Prev.Northing, Ppb);
                (Double X1, Double Y1) = WorldToPixel(Grid, Cur.Easting, Cur.Northing, Ppb);
                DrawLine(OverlayImage, X0, Y0, X1, Y1, new Rgba32(0, 0, 0, 220), Math.Max(2, Ppb / 4));
            }

            return ToPng(OverlayImage);
        }


        /// <summary>
        /// Executes the terrain color operation.
        /// </summary>
        /// <param name="T">The t used by the operation.</param>
        /// <returns>The resulting terrain color.</returns>

        private static Rgba32 TerrainColor
            (
            Double T
            )
        {
            // Piecewise interpolation reproduces Python numpy.interp behavior for elevation ramps.
            if (T <= Constants.QUARTER_SCALE)
            {
                return Lerp(new Rgba32(30, 90, 40, Constants.MAX_COLOR_CHANNEL), new Rgba32(120, 170, 70, Constants.MAX_COLOR_CHANNEL), (T / Constants.QUARTER_SCALE));
            }

            if (T <= Constants.HALF_SCALE)
            {
                return Lerp(new Rgba32(120, 170, 70, Constants.MAX_COLOR_CHANNEL), new Rgba32(210, 200, 120, Constants.MAX_COLOR_CHANNEL), ((T - Constants.QUARTER_SCALE) / Constants.QUARTER_SCALE));
            }

            if (T <= 0.75)
            {
                return Lerp(new Rgba32(210, 200, 120, Constants.MAX_COLOR_CHANNEL), new Rgba32(150, 110, 70, Constants.MAX_COLOR_CHANNEL), ((T - Constants.HALF_SCALE) / Constants.QUARTER_SCALE));
            }

            return Lerp(new Rgba32(150, 110, 70, Constants.MAX_COLOR_CHANNEL), new Rgba32(245, 245, 245, Constants.MAX_COLOR_CHANNEL), ((T - 0.75) / Constants.QUARTER_SCALE));
        }

        /// <summary>
        /// Executes the lerp operation.
        /// </summary>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <param name="T">The t used by the operation.</param>
        /// <returns>The resulting lerp.</returns>

        private static Rgba32 Lerp
            (
            Rgba32 A,
            Rgba32 B,
            Double T
            )
        {
            // Truncate like numpy astype(uint8) after float interpolation.
            Byte R = TruncByte(A.R + ((B.R - A.R) * T));
            Byte G = TruncByte(A.G + ((B.G - A.G) * T));
            Byte Blue = TruncByte(A.B + ((B.B - A.B) * T));
            return new Rgba32(R, G, Blue, Constants.BYTE_MASK);
        }

        /// <summary>
        /// Executes the trunc byte operation.
        /// </summary>
        /// <param name="Value">The value used by the operation.</param>
        /// <returns>The resulting trunc byte.</returns>

        private static Byte TruncByte
            (
            Double Value
            )
        {
            if (Value <= 0.0)
            {
                return 0;
            }

            if (Value >= Constants.MAX_COLOR_CHANNEL)
            {
                return Constants.MAX_COLOR_CHANNEL;
            }

            return (Byte)Value;
        }

        /// <summary>
        /// Paints each allocated cell as its own block. Nothing here is smoothed.
        /// The equipment path is drawn afterwards from the guidance points.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cells">Cells to paint. Null paints nothing.</param>
        /// <param name="Ppb">Pixels per grid cell.</param>
        /// <param name="Color">Solid colour for every cell.</param>
        private static void PaintWorkCells
            (
            Image<Rgba32> Image,
            Grid Grid,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? Cells,
            Int32 Ppb,
            Rgba32 Color
            )
        {
            if (Cells == null) return;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                Int32 Bx = Entry.Cell.Bx;
                Int32 By = Entry.Cell.By;
                if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height)) continue;
                // Same bin the terrain image uses, so the block is that cell and no other.
                Int32 X0 = Bx * Ppb;
                Int32 Y0 = (Grid.Height - 1 - By) * Ppb;
                for (Int32 Y = Y0; Y < Y0 + Ppb; Y += 1)
                {
                    if ((Y < 0) || (Y >= Image.Height)) continue;
                    for (Int32 X = X0; X < X0 + Ppb; X += 1)
                    {
                        if ((X < 0) || (X >= Image.Width)) continue;
                        Image[X, Y] = Color;
                    }
                }
            }
        }

        /// <summary>
        /// Adds cell centers to the crop so a cell off the smoothed path is not clipped.
        /// </summary>
        /// <param name="Xs">Pixel columns already included.</param>
        /// <param name="Ys">Pixel rows already included.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cells">Cells to keep in frame. Null adds nothing.</param>
        /// <param name="Ppb">Pixels per grid cell.</param>
        private static void IncludeWorkCells
            (
            List<Double> Xs,
            List<Double> Ys,
            Grid Grid,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? Cells,
            Int32 Ppb
            )
        {
            if (Cells == null) return;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Center = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                (Double Px, Double Py) = WorldToPixel(Grid, Center.X, Center.Y, Ppb);
                Xs.Add(Px);
                Ys.Add(Py);
            }
        }

        /// <summary>
        /// Executes the world to pixel operation.
        /// </summary>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Ppb">The ppb used by the operation.</param>
        /// <returns>The resulting world to pixel.</returns>

        private static (Double X, Double Y) WorldToPixel
            (
            Grid Grid,
            Double X,
            Double Y,
            Int32 Ppb
            )
        {
            // Coordinate transform intentionally mirrors Python render._world_to_pixel.
            Double Px = ((X - Grid.MinX) / Grid.GridSizeM) * Ppb;
            Double Py = (Grid.Height - ((Y - Grid.MinY) / Grid.GridSizeM)) * Ppb;
            return (Px, Py);
        }

        /// <summary>
        /// Cell window of one route, padded the same way the picture is cropped.
        /// Arrows outside this window would be cut off.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Points">Blade path, in world metres.</param>
        /// <param name="CutCells">Cut cells. Null when the cut is not painted.</param>
        /// <param name="FillCells">Fill cells. Null when the fill is not painted.</param>
        /// <param name="PadCells">Extra cells kept around the work.</param>
        /// <returns>Inclusive cell bounds.</returns>
        private static (Int32 X0, Int32 Y0, Int32 X1, Int32 Y1) ArrowWindow
            (
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? CutCells,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? FillCells,
            Int32 PadCells
            )
        {
            Int32 MinX = Grid.Width;
            Int32 MinY = Grid.Height;
            Int32 MaxX = 0;
            Int32 MaxY = 0;
            Boolean Any = false;
            if (CutCells != null)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in CutCells)
                {
                    IncludeArrowCell(Entry.Cell.Bx, Entry.Cell.By, ref MinX, ref MinY, ref MaxX, ref MaxY, ref Any);
                }
            }
            if (FillCells != null)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in FillCells)
                {
                    IncludeArrowCell(Entry.Cell.Bx, Entry.Cell.By, ref MinX, ref MinY, ref MaxX, ref MaxY, ref Any);
                }
            }
            foreach (GuidancePoint Point in Points)
            {
                Int32 Bx = (Int32)Math.Floor((Point.X - Grid.MinX) / Grid.GridSizeM);
                Int32 By = (Int32)Math.Floor((Point.Y - Grid.MinY) / Grid.GridSizeM);
                IncludeArrowCell(Bx, By, ref MinX, ref MinY, ref MaxX, ref MaxY, ref Any);
            }
            if (!Any)
            {
                return (0, 0, Grid.Width - 1, Grid.Height - 1);
            }
            Int32 Pad = Math.Max(0, PadCells);
            return (
                Math.Max(0, MinX - Pad),
                Math.Max(0, MinY - Pad),
                Math.Min(Grid.Width - 1, MaxX + Pad),
                Math.Min(Grid.Height - 1, MaxY + Pad));
        }

        /// <summary>Grows a cell window to include one cell.</summary>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="MinX">Smallest column so far.</param>
        /// <param name="MinY">Smallest row so far.</param>
        /// <param name="MaxX">Largest column so far.</param>
        /// <param name="MaxY">Largest row so far.</param>
        /// <param name="Any">True once one cell has been included.</param>
        private static void IncludeArrowCell
            (
            Int32 Bx,
            Int32 By,
            ref Int32 MinX,
            ref Int32 MinY,
            ref Int32 MaxX,
            ref Int32 MaxY,
            ref Boolean Any
            )
        {
            if (Bx < MinX) MinX = Bx;
            if (By < MinY) MinY = By;
            if (Bx > MaxX) MaxX = Bx;
            if (By > MaxY) MaxY = By;
            Any = true;
        }

        /// <summary>
        /// Draws one downhill arrow on each cell. The shaft is shorter than a
        /// cell so neighboring arrows stay apart. A white edge keeps the mark
        /// visible on both the red cut and the pale grade bands.
        /// </summary>
        /// <param name="Image">Image the arrows are drawn onto.</param>
        /// <param name="Grid">Field grid. Arrow rows match this grid.</param>
        /// <param name="ArrowX">East component of the arrow.</param>
        /// <param name="ArrowY">North component of the arrow.</param>
        /// <param name="Ppb">Pixels per cell.</param>
        /// <param name="ArrowStride">Cells between arrows.</param>
        /// <param name="Window">Inclusive cell bounds to stroke. Outside this the crop will discard them.</param>
        private static void DrawSmoothedArrows
            (
            Image<Rgba32> Image,
            Grid Grid,
            Double[,] ArrowX,
            Double[,] ArrowY,
            Int32 Ppb,
            Int32 ArrowStride,
            (Int32 X0, Int32 Y0, Int32 X1, Int32 Y1) Window
            )
        {
            // Downstream is the Poisson direction: east to the right, north up the image.
            // The halo stays two pixels. A wider stroke at this scale paints over the grade band.
            Double Length = Ppb * 0.55;
            Double HalfBase = Ppb * 0.14;
            Int32 Outline = 2;
            Int32 Inner = 1;
            Rgba32 Halo = new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL);
            Rgba32 Ink = new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL);
            Int32 Height = Grid.Height;
            Int32 Width = Grid.Width;
            Int32 By0 = Math.Max(0, Window.Y0);
            Int32 By1 = Math.Min(Height - 1, Window.Y1);
            Int32 Bx0 = Math.Max(0, Window.X0);
            Int32 Bx1 = Math.Min(Width - 1, Window.X1);
            for (Int32 By = By0; By <= By1; By += ArrowStride)
            {
                for (Int32 Bx = Bx0; Bx <= Bx1; Bx += ArrowStride)
                {
                    if (!Grid.Valid[By, Bx]) continue;
                    if ((By >= ArrowY.GetLength(0)) || (Bx >= ArrowX.GetLength(1))) continue;
                    Double Ux = ArrowX[By, Bx];
                    Double Uy = ArrowY[By, Bx];
                    Double Mag = Math.Sqrt((Ux * Ux) + (Uy * Uy));
                    if (Mag <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) continue;
                    Ux /= Mag;
                    Uy /= Mag;
                    Double Dx = Ux;
                    Double Dy = -Uy;
                    Double Px = (Bx + Constants.HALF_SCALE) * Ppb;
                    Double Py = (Height - Constants.HALF_SCALE - By) * Ppb;
                    Double TipX = Px + (Length * Dx);
                    Double TipY = Py + (Length * Dy);
                    Double TailX = Px - (0.35 * Length * Dx);
                    Double TailY = Py - (0.35 * Length * Dy);
                    Double BaseX = Px + (0.05 * Length * Dx);
                    Double BaseY = Py + (0.05 * Length * Dy);
                    Double PerpX = -Dy;
                    Double PerpY = Dx;
                    Double LeftX = BaseX + (HalfBase * PerpX);
                    Double LeftY = BaseY + (HalfBase * PerpY);
                    Double RightX = BaseX - (HalfBase * PerpX);
                    Double RightY = BaseY - (HalfBase * PerpY);
                    DrawLine(Image, TailX, TailY, BaseX, BaseY, Halo, Outline);
                    DrawLine(Image, TipX, TipY, LeftX, LeftY, Halo, Outline);
                    DrawLine(Image, TipX, TipY, RightX, RightY, Halo, Outline);
                    DrawLine(Image, LeftX, LeftY, RightX, RightY, Halo, Outline);
                    DrawLine(Image, TailX, TailY, BaseX, BaseY, Ink, Inner);
                    DrawLine(Image, TipX, TipY, LeftX, LeftY, Ink, Inner);
                    DrawLine(Image, TipX, TipY, RightX, RightY, Ink, Inner);
                    DrawLine(Image, LeftX, LeftY, RightX, RightY, Ink, Inner);
                }
            }
        }

        /// <summary>
        /// Renders flow arrows.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Flow">The flow used by the operation.</param>
        /// <param name="Ppb">The ppb used by the operation.</param>
        /// <param name="ArrowStride">The arrow stride used by the operation.</param>

        private static void DrawFlowArrows
            (
            Image<Rgba32> Image,
            Grid Grid,
            FlowField Flow,
            Int32 Ppb,
            Int32 ArrowStride
            )
        {
            // Arrow glyphs are hollow triangles so the map remains visible beneath direction marks.
            Double Length = (Ppb * 0.45);
            Double HalfBase = (Ppb * 0.16);
            for (Int32 By = 0; By < Grid.Height; By += ArrowStride)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += ArrowStride)
                {
                    if (!Grid.Valid[By, Bx]) continue;
                    if (!FlowHasArrow(Flow, Bx, By)) continue;
                    Double DirectionDeg = FlowDirectionDeg(Flow, Bx, By);
                    Double Rad = (Math.PI / Constants.DEGREES_PER_HALF_TURN) * DirectionDeg;
                    Double Dx = Math.Sin(Rad);
                    Double Dy = -Math.Cos(Rad);
                    Double Px = ((Bx + Constants.HALF_SCALE) * Ppb);
                    Double Py = ((Grid.Height - Constants.HALF_SCALE - By) * Ppb);
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

                    DrawThinLineBresenham(Image, TipX, TipY, LeftX, LeftY, new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL));
                    DrawThinLineBresenham(Image, TipX, TipY, RightX, RightY, new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL));
                    DrawThinLineBresenham(Image, LeftX, LeftY, RightX, RightY, new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL));
                }
            }
        }

        /// <summary>Draws high-contrast arrowheads at regular travelled distances along a route.</summary>
        private static void DrawRouteDirectionArrows
            (
            Image<Rgba32> Image,
            Grid Grid,
            IReadOnlyList<(Double X, Double Y)> RoutePoints,
            Int32 Ppb,
            Double SpacingM
            )
        {
            if ((RoutePoints.Count < 2) || (SpacingM <= 0.0)) return;
            Double UntilArrowM = Constants.HALF_SCALE * SpacingM;
            Double ArrowLengthPx = Math.Max(8.0, Ppb * 0.42);
            Double HalfWidthPx = Math.Max(4.0, Ppb * 0.18);
            Int32 OutlineWidth = Math.Max(3, Ppb / 8);
            Int32 InnerWidth = Math.Max(1, OutlineWidth / 2);
            Rgba32 Outline = new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL);
            Rgba32 Inner = new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL);

            for (Int32 Index = 1; Index < RoutePoints.Count; Index += 1)
            {
                (Double X, Double Y) A = RoutePoints[Index - 1];
                (Double X, Double Y) B = RoutePoints[Index];
                Double DxM = B.X - A.X;
                Double DyM = B.Y - A.Y;
                Double SegmentM = Math.Sqrt((DxM * DxM) + (DyM * DyM));
                if (SegmentM <= Constants.NUMERIC_COMPARISON_TOLERANCE) continue;
                while (UntilArrowM <= SegmentM + Constants.NUMERIC_COMPARISON_TOLERANCE)
                {
                    Double Fraction = Math.Min(1.0, UntilArrowM / SegmentM);
                    Double X = A.X + (Fraction * DxM);
                    Double Y = A.Y + (Fraction * DyM);
                    (Double TipX, Double TipY) = WorldToPixel(Grid, X, Y, Ppb);
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
                    DrawLine(Image, TipX, TipY, LeftX, LeftY, Outline, OutlineWidth);
                    DrawLine(Image, TipX, TipY, RightX, RightY, Outline, OutlineWidth);
                    DrawLine(Image, TipX, TipY, LeftX, LeftY, Inner, InnerWidth);
                    DrawLine(Image, TipX, TipY, RightX, RightY, Inner, InnerWidth);
                    UntilArrowM += SpacingM;
                }
                UntilArrowM -= SegmentM;
            }
        }

        /// <summary>
        /// Renders exclusions.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Ppb">The ppb used by the operation.</param>

        private static void DrawExclusions
            (
            Image<Rgba32> Image,
            Grid Grid,
            Int32 Ppb
            )
        {
            // Exclusion overlays keep no-go zones visually explicit without hiding the underlying grade map.
            IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> Rings = GridExclusions(Grid);
            if (Rings.Count <= 0)
            {
                return;
            }

            foreach (IReadOnlyList<(Double X, Double Y)> Ring in Rings)
            {
                for (Int32 Iter = 1; Iter < Ring.Count; Iter += 1)
                {
                    (Double Ax, Double Ay) = WorldToPixel(Grid, Ring[Iter - 1].X, Ring[Iter - 1].Y, Ppb);
                    (Double Bx, Double By) = WorldToPixel(Grid, Ring[Iter].X, Ring[Iter].Y, Ppb);
                    DrawLine(Image, Ax, Ay, Bx, By, new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL), Math.Max(2, Ppb / 4));
                }
            }
        }

        /// <summary>
        /// Executes the dim towards white operation.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="BaseDim">The base dim used by the operation.</param>

        private static void DimTowardsWhite
            (
            Image<Rgba32> Image,
            Double BaseDim
            )
        {
            // Fading base color increases contrast for overlays while preserving context.
            if ((BaseDim < 0.0) || (BaseDim >= 1.0))
            {
                return;
            }

            for (Int32 Y = 0; Y < Image.Height; Y += 1)
            {
                for (Int32 X = 0; X < Image.Width; X += 1)
                {
                    Rgba32 P = Image[X, Y];
                    // Pillow dims RGB for every pixel, including alpha=0 cells (render_pass_pairs_png).
                    P.R = TruncByte((P.R * BaseDim) + (Constants.MAX_COLOR_CHANNEL * (1.0 - BaseDim)));
                    P.G = TruncByte((P.G * BaseDim) + (Constants.MAX_COLOR_CHANNEL * (1.0 - BaseDim)));
                    P.B = TruncByte((P.B * BaseDim) + (Constants.MAX_COLOR_CHANNEL * (1.0 - BaseDim)));
                    Image[X, Y] = P;
                }
            }
        }

        /// <summary>
        /// Executes the guidance color operation.
        /// </summary>
        /// <param name="Action">The action used by the operation.</param>
        /// <returns>The resulting guidance color.</returns>

        private static Rgba32 GuidanceColor
            (
            String Action
            )
        {
            // Action palette mirrors Python guidance rendering semantics.
            if (Action == "CUT") return new Rgba32(200, 0, 0, Constants.MAX_COLOR_CHANNEL);
            if (Action == "FILL") return new Rgba32(0, 90, 220, Constants.MAX_COLOR_CHANNEL);
            if (Action == "GRADE") return new Rgba32(40, 160, 40, 200);
            if (Action == "HAUL") return new Rgba32(200, 0, 200, 90);
            if (Action == "MOVE") return new Rgba32(Constants.MAX_COLOR_CHANNEL, 140, 0, 120);
            if (Action == "MANUAL_MOVE") return new Rgba32(Constants.MAX_COLOR_CHANNEL, 80, 0, 220);
            return new Rgba32(120, 120, 120, 70);
        }

        /// <summary>
        /// Draws one blade-down action as separate strokes.
        /// A blade-up join between two strips is not part of the cut, so the
        /// stroke breaks there instead of cutting a corner across the gap.
        /// </summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Points">Guidance points in drive order.</param>
        /// <param name="Action">Action to draw, CUT or FILL.</param>
        /// <param name="Ppb">Pixels per grid cell.</param>
        /// <param name="Color">Stroke color.</param>
        /// <param name="Width">Stroke width in pixels.</param>
        private static void DrawWorkStrokes
            (
            Image<Rgba32> Image,
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            String Action,
            Int32 Ppb,
            Rgba32 Color,
            Int32 Width
            )
        {
            List<GuidancePoint> Stroke = new List<GuidancePoint>();
            Int32 Previous = -2;
            for (Int32 Index = 0; Index < Points.Count; Index += 1)
            {
                if (Points[Index].Action != Action)
                {
                    if (Stroke.Count >= 2)
                    {
                        DrawPolyline(Image, Grid, Stroke, Ppb, Color, Width);
                    }
                    Stroke = new List<GuidancePoint>();
                    continue;
                }
                // A gap in the guidance list is a blade-up turn between strips.
                if ((Stroke.Count > 0) && (Index != (Previous + 1)))
                {
                    if (Stroke.Count >= 2)
                    {
                        DrawPolyline(Image, Grid, Stroke, Ppb, Color, Width);
                    }
                    Stroke = new List<GuidancePoint>();
                }
                Stroke.Add(Points[Index]);
                Previous = Index;
            }
            if (Stroke.Count >= 2)
            {
                DrawPolyline(Image, Grid, Stroke, Ppb, Color, Width);
            }
        }

        /// <summary>Draws a guidance polyline in world space.</summary>
        /// <param name="Image">Image being drawn.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Points">Points of one stroke.</param>
        /// <param name="Ppb">Pixels per grid cell.</param>
        /// <param name="Color">Stroke color.</param>
        /// <param name="Width">Stroke width in pixels.</param>
        private static void DrawPolyline
            (
            Image<Rgba32> Image,
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            Int32 Ppb,
            Rgba32 Color,
            Int32 Width
            )
        {
            // Guidance polylines are drawn segment-by-segment in world space.
            for (Int32 Iter = 1; Iter < Points.Count; Iter += 1)
            {
                (Double X0, Double Y0) = WorldToPixel(Grid, Points[Iter - 1].X, Points[Iter - 1].Y, Ppb);
                (Double X1, Double Y1) = WorldToPixel(Grid, Points[Iter].X, Points[Iter].Y, Ppb);
                DrawLine(
                    Image,
                    X0,
                    Y0,
                    X1,
                    Y1,
                    Color,
                    Width);
            }
        }

        /// <summary>
        /// Renders polyline pixels.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Points">The points used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>
        /// <param name="Width">The width used by the operation.</param>

        private static void DrawPolylinePixels
            (
            Image<Rgba32> Image,
            IReadOnlyList<(Double X, Double Y)> Points,
            Rgba32 Color,
            Int32 Width
            )
        {
            // Pixel-space polylines are used for route overlays already projected to image coordinates.
            for (Int32 Iter = 1; Iter < Points.Count; Iter += 1)
            {
                (Double X0, Double Y0) = Points[Iter - 1];
                (Double X1, Double Y1) = Points[Iter];
                DrawLine(Image, X0, Y0, X1, Y1, Color, Width);
            }
        }

        /// <summary>
        /// Executes the dot guidance point operation.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="Point">The point used by the operation.</param>
        /// <param name="Ppb">The ppb used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>
        /// <param name="Radius">The radius used by the operation.</param>

        private static void DotGuidancePoint
            (
            Image<Rgba32> Image,
            Grid Grid,
            GuidancePoint Point,
            Int32 Ppb,
            Rgba32 Color,
            Int32 Radius
            )
        {
            (Double Px, Double Py) = WorldToPixel(Grid, Point.X, Point.Y, Ppb);
            DrawEllipsePil(
                Image,
                Px,
                Py,
                Radius,
                Color,
                new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL),
                2);
        }

        /// <summary>
        /// Renders ellipse pil.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Cx">The cx used by the operation.</param>
        /// <param name="Cy">The cy used by the operation.</param>
        /// <param name="Radius">The radius used by the operation.</param>
        /// <param name="Fill">The fill used by the operation.</param>
        /// <param name="Outline">The outline used by the operation.</param>
        /// <param name="OutlineWidth">The outline width used by the operation.</param>

        private static void DrawEllipsePil
            (
            Image<Rgba32> Image,
            Double Cx,
            Double Cy,
            Int32 Radius,
            Rgba32 Fill,
            Rgba32 Outline,
            Int32 OutlineWidth
            )
        {
            // Matches Pillow ImageDraw.ellipse((x-r,y-r,x+r,y+r), fill=..., outline=..., width=...).
            Int32 X0 = TruncTowardZero(Cx - Radius);
            Int32 Y0 = TruncTowardZero(Cy - Radius);
            Int32 X1 = TruncTowardZero(Cx + Radius);
            Int32 Y1 = TruncTowardZero(Cy + Radius);
            if (OutlineWidth > 0)
            {
                DrawFilledEllipse(Image, X0, Y0, X1, Y1, Fill);
                DrawEllipseOutlineRing(Image, X0, Y0, X1, Y1, Outline, OutlineWidth);
                return;
            }

            DrawFilledEllipse(Image, X0, Y0, X1, Y1, Fill);
        }

        /// <summary>
        /// Renders ellipse outline ring.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X0">The x0 used by the operation.</param>
        /// <param name="Y0">The y0 used by the operation.</param>
        /// <param name="X1">The x1 used by the operation.</param>
        /// <param name="Y1">The y1 used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>
        /// <param name="Width">The width used by the operation.</param>

        private static void DrawEllipseOutlineRing
            (
            Image<Rgba32> Image,
            Int32 X0,
            Int32 Y0,
            Int32 X1,
            Int32 Y1,
            Rgba32 Color,
            Int32 Width
            )
        {
            Int32 InnerPad = Width / 2;
            Int32 OuterPad = Width - InnerPad;
            Int32 Ox0 = X0 - OuterPad;
            Int32 Oy0 = Y0 - OuterPad;
            Int32 Ox1 = X1 + OuterPad;
            Int32 Oy1 = Y1 + OuterPad;
            Int32 Ix0 = X0 + InnerPad;
            Int32 Iy0 = Y0 + InnerPad;
            Int32 Ix1 = X1 - InnerPad;
            Int32 Iy1 = Y1 - InnerPad;
            Double Ocx = (Ox0 + Ox1) / 2.0;
            Double Ocy = (Oy0 + Oy1) / 2.0;
            Double Orx = Math.Max(Constants.HALF_SCALE, (Ox1 - Ox0) / 2.0);
            Double Ory = Math.Max(Constants.HALF_SCALE, (Oy1 - Oy0) / 2.0);
            Double Icx = (Ix0 + Ix1) / 2.0;
            Double Icy = (Iy0 + Iy1) / 2.0;
            Double Irx = Math.Max(Constants.HALF_SCALE, (Ix1 - Ix0) / 2.0);
            Double Iry = Math.Max(Constants.HALF_SCALE, (Iy1 - Iy0) / 2.0);
            Int32 MinX = Math.Max(0, Ox0);
            Int32 MaxX = Math.Min(Image.Width - 1, Ox1);
            Int32 MinY = Math.Max(0, Oy0);
            Int32 MaxY = Math.Min(Image.Height - 1, Oy1);
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

                    Plot(Image, X, Y, Color);
                }
            }
        }

        /// <summary>
        /// Executes the trunc toward zero operation.
        /// </summary>
        /// <param name="Value">The value used by the operation.</param>
        /// <returns>The resulting trunc toward zero.</returns>

        private static Int32 TruncTowardZero
            (
            Double Value
            )
        {
            return Value >= 0.0 ? (Int32)Math.Floor(Value) : (Int32)Math.Ceiling(Value);
        }

        /// <summary>
        /// Renders filled ellipse.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X0">The x0 used by the operation.</param>
        /// <param name="Y0">The y0 used by the operation.</param>
        /// <param name="X1">The x1 used by the operation.</param>
        /// <param name="Y1">The y1 used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void DrawFilledEllipse
            (
            Image<Rgba32> Image,
            Int32 X0,
            Int32 Y0,
            Int32 X1,
            Int32 Y1,
            Rgba32 Color
            )
        {
            Double Cx = (X0 + X1) / 2.0;
            Double Cy = (Y0 + Y1) / 2.0;
            Double Rx = Math.Max(Constants.HALF_SCALE, (X1 - X0) / 2.0);
            Double Ry = Math.Max(Constants.HALF_SCALE, (Y1 - Y0) / 2.0);
            Int32 MinX = Math.Max(0, X0);
            Int32 MaxX = Math.Min(Image.Width - 1, X1);
            Int32 MinY = Math.Max(0, Y0);
            Int32 MaxY = Math.Min(Image.Height - 1, Y1);
            for (Int32 Y = MinY; Y <= MaxY; Y += 1)
            {
                for (Int32 X = MinX; X <= MaxX; X += 1)
                {
                    Double Nx = (X - Cx) / Rx;
                    Double Ny = (Y - Cy) / Ry;
                    if (((Nx * Nx) + (Ny * Ny)) <= 1.0)
                    {
                        Plot(Image, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Renders line.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X0">The x0 used by the operation.</param>
        /// <param name="Y0">The y0 used by the operation.</param>
        /// <param name="X1">The x1 used by the operation.</param>
        /// <param name="Y1">The y1 used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>
        /// <param name="Width">The width used by the operation.</param>

        private static void DrawLine
            (
            Image<Rgba32> Image,
            Double X0,
            Double Y0,
            Double X1,
            Double Y1,
            Rgba32 Color,
            Int32 Width
            )
        {
            // Pillow rounds endpoints and strokes width via parallel 1-px lines.
            Int32 PixelX0 = (Int32)Math.Round(X0, MidpointRounding.AwayFromZero);
            Int32 PixelY0 = (Int32)Math.Round(Y0, MidpointRounding.AwayFromZero);
            Int32 PixelX1 = (Int32)Math.Round(X1, MidpointRounding.AwayFromZero);
            Int32 PixelY1 = (Int32)Math.Round(Y1, MidpointRounding.AwayFromZero);
            if (Width <= 1)
            {
                DrawThinLineBresenham(Image, PixelX0, PixelY0, PixelX1, PixelY1, Color);
                return;
            }

            DrawWideLinePillow(Image, PixelX0, PixelY0, PixelX1, PixelY1, Color, Width);
        }

        /// <summary>
        /// Renders wide line pillow.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X0">The x0 used by the operation.</param>
        /// <param name="Y0">The y0 used by the operation.</param>
        /// <param name="X1">The x1 used by the operation.</param>
        /// <param name="Y1">The y1 used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>
        /// <param name="Width">The width used by the operation.</param>

        private static void DrawWideLinePillow
            (
            Image<Rgba32> Image,
            Int32 X0,
            Int32 Y0,
            Int32 X1,
            Int32 Y1,
            Rgba32 Color,
            Int32 Width
            )
        {
            // Port of Pillow ImagingDrawWideLine (src/libImaging/Draw.c).
            Int32 Dx = (X1 - X0);
            Int32 Dy = (Y1 - Y0);
            if ((Dx == 0) && (Dy == 0))
            {
                Plot(Image, X0, Y0, Color);
                return;
            }

            Double BigHyp = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Double SmallHyp = (Width - 1) / 2.0;
            Double RatioMax = RoundUp(SmallHyp) / BigHyp;
            Double RatioMin = RoundDown(SmallHyp) / BigHyp;
            Int32 DxMin = RoundDown(RatioMin * Dy);
            Int32 DxMax = RoundDown(RatioMax * Dy);
            Int32 DyMin = RoundDown(RatioMin * Dx);
            Int32 DyMax = RoundDown(RatioMax * Dx);
            Int32[] Xs = { X0 - DxMin, X1 - DxMin, X1 + DxMax, X0 + DxMax };
            Int32[] Ys = { Y0 + DyMax, Y1 + DyMax, Y1 - DyMin, Y0 - DyMin };
            FillPolygonScanline(Image, Xs, Ys, Color);
            for (Int32 Iter = 0; Iter < Xs.Length; Iter += 1)
            {
                Plot(Image, Xs[Iter], Ys[Iter], Color);
            }
        }

        /// <summary>
        /// Executes the round up operation.
        /// </summary>
        /// <param name="Value">The value used by the operation.</param>
        /// <returns>The resulting round up.</returns>

        private static Int32 RoundUp
            (
            Double Value
            )
        {
            return Value >= 0.0
                ? (Int32)Math.Floor(Value + Constants.HALF_SCALE)
                : -(Int32)Math.Floor(Math.Abs(Value) + Constants.HALF_SCALE);
        }

        /// <summary>
        /// Executes the round down operation.
        /// </summary>
        /// <param name="Value">The value used by the operation.</param>
        /// <returns>The resulting round down.</returns>

        private static Int32 RoundDown
            (
            Double Value
            )
        {
            return Value >= 0.0
                ? (Int32)Math.Ceiling(Value - Constants.HALF_SCALE)
                : -(Int32)Math.Ceiling(Math.Abs(Value) - Constants.HALF_SCALE);
        }

        /// <summary>
        /// Executes the fill polygon scanline operation.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Xs">The xs used by the operation.</param>
        /// <param name="Ys">The ys used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void FillPolygonScanline
            (
            Image<Rgba32> Image,
            Int32[] Xs,
            Int32[] Ys,
            Rgba32 Color
            )
        {
            Int32 Count = Xs.Length;
            Int32 MinY = Ys.Min();
            Int32 MaxY = Ys.Max();
            for (Int32 Y = MinY; Y <= MaxY; Y += 1)
            {
                List<Double> Intersections = new List<Double>();
                for (Int32 Iter = 0; Iter < Count; Iter += 1)
                {
                    Int32 Y0 = Ys[Iter];
                    Int32 Y1 = Ys[(Iter + 1) % Count];
                    Int32 X0 = Xs[Iter];
                    Int32 X1 = Xs[(Iter + 1) % Count];
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
                        Plot(Image, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Renders thin line bresenham.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X0">The x0 used by the operation.</param>
        /// <param name="Y0">The y0 used by the operation.</param>
        /// <param name="X1">The x1 used by the operation.</param>
        /// <param name="Y1">The y1 used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void DrawThinLineBresenham
            (
            Image<Rgba32> Image,
            Int32 X0,
            Int32 Y0,
            Int32 X1,
            Int32 Y1,
            Rgba32 Color
            )
        {
            Int32 Dx = Math.Abs(X1 - X0);
            Int32 Dy = Math.Abs(Y1 - Y0);
            Int32 Sx = (X0 < X1) ? 1 : -1;
            Int32 Sy = (Y0 < Y1) ? 1 : -1;
            Int32 Err = (Dx - Dy);
            Int32 X = X0;
            Int32 Y = Y0;
            while (true)
            {
                Plot(Image, X, Y, Color);
                if ((X == X1) && (Y == Y1))
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

        /// <summary>
        /// Renders filled square.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Cx">The cx used by the operation.</param>
        /// <param name="Cy">The cy used by the operation.</param>
        /// <param name="Width">The width used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void DrawFilledSquare
            (
            Image<Rgba32> Image,
            Int32 Cx,
            Int32 Cy,
            Int32 Width,
            Rgba32 Color
            )
        {
            Int32 Half = Width / 2;
            for (Int32 Dy = -Half; Dy <= (Width - Half - 1); Dy += 1)
            {
                for (Int32 Dx = -Half; Dx <= (Width - Half - 1); Dx += 1)
                {
                    Plot(Image, Cx + Dx, Cy + Dy, Color);
                }
            }
        }

        /// <summary>
        /// Renders circle.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Radius">The radius used by the operation.</param>
        /// <param name="Fill">The fill used by the operation.</param>
        /// <param name="Stroke">The stroke used by the operation.</param>

        private static void DrawCircle
            (
            Image<Rgba32> Image,
            Double X,
            Double Y,
            Int32 Radius,
            Rgba32 Fill,
            Rgba32 Stroke
            )
        {
            // Marker circles are rasterized manually to avoid optional drawing package dependencies.
            DrawCircleFilled(Image, (Int32)Math.Round(X), (Int32)Math.Round(Y), Radius, Fill);
            Int32 Outline = Math.Max(1, Radius / 4);
            for (Int32 Iter = 0; Iter < Outline; Iter += 1)
            {
                DrawCircleRing(Image, (Int32)Math.Round(X), (Int32)Math.Round(Y), Math.Max(1, Radius - Iter), Stroke);
            }
        }

        /// <summary>
        /// Renders circle filled.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Cx">The cx used by the operation.</param>
        /// <param name="Cy">The cy used by the operation.</param>
        /// <param name="Radius">The radius used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void DrawCircleFilled
            (
            Image<Rgba32> Image,
            Int32 Cx,
            Int32 Cy,
            Int32 Radius,
            Rgba32 Color
            )
        {
            // Filled disk stamp is reused by markers and thick polyline drawing.
            for (Int32 Dy = -Radius; Dy <= Radius; Dy += 1)
            {
                for (Int32 Dx = -Radius; Dx <= Radius; Dx += 1)
                {
                    if (((Dx * Dx) + (Dy * Dy)) > (Radius * Radius)) continue;
                    Int32 X = (Cx + Dx);
                    Int32 Y = (Cy + Dy);
                    if ((X < 0) || (Y < 0) || (X >= Image.Width) || (Y >= Image.Height)) continue;
                    Image[X, Y] = Color;
                }
            }
        }

        /// <summary>
        /// Renders circle ring.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Cx">The cx used by the operation.</param>
        /// <param name="Cy">The cy used by the operation.</param>
        /// <param name="Radius">The radius used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void DrawCircleRing
            (
            Image<Rgba32> Image,
            Int32 Cx,
            Int32 Cy,
            Int32 Radius,
            Rgba32 Color
            )
        {
            // Ring midpoint stepping draws a crisp border around waypoint markers.
            if (Radius <= 0) return;
            Int32 X = Radius;
            Int32 Y = 0;
            Int32 Err = 0;
            while (X >= Y)
            {
                Plot8(Image, Cx, Cy, X, Y, Color);
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

        /// <summary>
        /// Renders plot8.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Cx">The cx used by the operation.</param>
        /// <param name="Cy">The cy used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void Plot8
            (
            Image<Rgba32> Image,
            Int32 Cx,
            Int32 Cy,
            Int32 X,
            Int32 Y,
            Rgba32 Color
            )
        {
            // Eight-way symmetry keeps circle plotting branch-free at call sites.
            Plot(Image, (Cx + X), (Cy + Y), Color);
            Plot(Image, (Cx + Y), (Cy + X), Color);
            Plot(Image, (Cx - Y), (Cy + X), Color);
            Plot(Image, (Cx - X), (Cy + Y), Color);
            Plot(Image, (Cx - X), (Cy - Y), Color);
            Plot(Image, (Cx - Y), (Cy - X), Color);
            Plot(Image, (Cx + Y), (Cy - X), Color);
            Plot(Image, (Cx + X), (Cy - Y), Color);
        }

        /// <summary>
        /// Renders plot.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void Plot
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Rgba32 Color
            )
        {
            // Bounds checks are centralized so primitive routines remain simple.
            if ((X < 0) || (Y < 0) || (X >= Image.Width) || (Y >= Image.Height)) return;
            Image[X, Y] = Color;
        }

        /// <summary>
        /// Draws small bitmap text for map banners and labels.
        /// </summary>
        /// <param name="Image">Destination image.</param>
        /// <param name="CenterX">Horizontal centre of the text block.</param>
        /// <param name="CenterY">Vertical centre of the text block.</param>
        /// <param name="Text">Text to render.</param>
        /// <param name="Color">Optional glyph colour; defaults to white.</param>
        static public void DrawBannerText
            (
            Image<Rgba32> Image,
            Int32 CenterX,
            Int32 CenterY,
            String Text,
            Rgba32? Color = null
            )
        {
            DrawTextDigit(Image, CenterX, CenterY, Text, Color ?? new Rgba32(255, 255, 255, 255));
        }

        /// <summary>Renders one bitmap glyph for banner text.</summary>
        /// <param name="Image">Destination image.</param>
        /// <param name="CenterX">Horizontal centre of the text block.</param>
        /// <param name="CenterY">Vertical centre of the text block.</param>
        /// <param name="Text">Text to render.</param>
        private static void DrawTextDigit
            (
            Image<Rgba32> Image,
            Int32 CenterX,
            Int32 CenterY,
            String Text
            )
        {
            DrawTextDigit(Image, CenterX, CenterY, Text,
                new Rgba32(0, 0, 0, Constants.MAX_COLOR_CHANNEL));
        }

        /// <summary>Renders one bitmap glyph for banner text.</summary>
        /// <param name="Image">Destination image.</param>
        /// <param name="CenterX">Horizontal centre of the text block.</param>
        /// <param name="CenterY">Vertical centre of the text block.</param>
        /// <param name="Text">Text to render.</param>
        /// <param name="Color">Glyph colour.</param>
        private static void DrawTextDigit
            (
            Image<Rgba32> Image,
            Int32 CenterX,
            Int32 CenterY,
            String Text,
            Rgba32 Color
            )
        {
            // Small bitmap digits avoid adding a font rasterizer dependency just for waypoint labels.
            if (String.IsNullOrWhiteSpace(Text)) return;
            String Value = Text.Trim();
            Int32 OffsetX = CenterX - (Value.Length * 4);
            for (Int32 Iter = 0; Iter < Value.Length; Iter += 1)
            {
                DrawGlyph(Image, Value[Iter], (OffsetX + (Iter * 8)), (CenterY - 4), Color);
            }
        }

        /// <summary>
        /// Executes the grid exclusions operation.
        /// </summary>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting grid exclusions.</returns>

        private static IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> GridExclusions
            (
            Grid Grid
            )
        {
            // Reflection keeps this renderer compatible with grid variants that may omit exclusions.
            Object? Value = Grid.GetType().GetProperty("Exclusions")?.GetValue(Grid);
            if (Value is IReadOnlyList<IReadOnlyList<(Double X, Double Y)>> Typed)
            {
                return Typed;
            }

            return Array.Empty<IReadOnlyList<(Double X, Double Y)>>();
        }

        /// <summary>
        /// Executes the flow has arrow operation.
        /// </summary>
        /// <param name="Flow">The flow used by the operation.</param>
        /// <param name="Bx">The bx used by the operation.</param>
        /// <param name="By">The by used by the operation.</param>
        /// <returns>The resulting flow has arrow.</returns>

        private static Boolean FlowHasArrow
            (
            FlowField Flow,
            Int32 Bx,
            Int32 By
            )
        {
            // Both array-backed and method-backed flow ports are supported.
            Object? ArrowArray = Flow.GetType().GetProperty("HasArrow")?.GetValue(Flow);
            if (ArrowArray is Boolean[,] Matrix)
            {
                return Matrix[By, Bx];
            }

            System.Reflection.MethodInfo? HasArrowMethod = Flow.GetType().GetMethod("HasArrow");
            if (HasArrowMethod is not null)
            {
                Object? Result = HasArrowMethod.Invoke(Flow, new Object[] { Bx, By });
                if (Result is Boolean Flag) return Flag;
            }

            return false;
        }

        /// <summary>
        /// Executes the flow direction deg operation.
        /// </summary>
        /// <param name="Flow">The flow used by the operation.</param>
        /// <param name="Bx">The bx used by the operation.</param>
        /// <param name="By">The by used by the operation.</param>
        /// <returns>The resulting flow direction deg.</returns>

        private static Double FlowDirectionDeg
            (
            FlowField Flow,
            Int32 Bx,
            Int32 By
            )
        {
            // Direction lookup falls back to fx/fy vector conversion when a direct helper is absent.
            System.Reflection.MethodInfo? DirectionMethod = Flow.GetType().GetMethod("DirectionDeg");
            if (DirectionMethod is not null)
            {
                Object? Value = DirectionMethod.Invoke(Flow, new Object[] { Bx, By });
                if (Value is Double Degrees) return Degrees;
            }

            Object? FxValue = Flow.GetType().GetProperty("Fx")?.GetValue(Flow);
            Object? FyValue = Flow.GetType().GetProperty("Fy")?.GetValue(Flow);
            if ((FxValue is Double[,] Fx) && (FyValue is Double[,] Fy))
            {
                Double Rad = Math.Atan2(Fx[By, Bx], Fy[By, Bx]);
                Double Deg = (Rad * (Constants.DEGREES_PER_HALF_TURN / Math.PI));
                if (Deg < 0.0) Deg += Constants.DEGREES_PER_TURN;
                return Deg;
            }

            return 0.0;
        }

        /// <summary>
        /// Renders glyph.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Ch">The ch used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void DrawGlyph
            (
            Image<Rgba32> Image,
            Char Ch,
            Int32 X,
            Int32 Y,
            Rgba32 Color
            )
        {
            // Minimal 3x5 digit glyphs keep labels legible without external drawing libraries.
            String[] Glyph = Ch switch
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
                    Plot(Image, (X + Col), (Y + Row), Color);
                }
            }
        }

        /// <summary>
        /// Executes the to png operation.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <returns>The resulting to png.</returns>

        private static Byte[] ToPng
            (
            Image<Rgba32> Image
            )
        {
            // Memory-stream encoding keeps the render API file-system independent.
            using MemoryStream Stream = new MemoryStream();
            Image.Save(Stream, new PngEncoder());
            return Stream.ToArray();
        }

        /// <summary>
        /// Executes the percentile linear operation.
        /// </summary>
        /// <param name="SortedValues">The sorted values used by the operation.</param>
        /// <param name="Percentile">The percentile used by the operation.</param>
        /// <returns>The resulting percentile linear.</returns>

        private static Single PercentileLinear
            (
            IReadOnlyList<Single> SortedValues,
            Double Percentile
            )
        {
            // Matches numpy.percentile(..., method="linear") used by Python render_pass_pairs_png.
            if (SortedValues.Count == 0)
            {
                return 1.0f;
            }

            if (SortedValues.Count == 1)
            {
                return SortedValues[0];
            }

            Double Index = Percentile * (SortedValues.Count - 1);
            Int32 Lo = (Int32)Math.Floor(Index);
            Int32 Hi = (Int32)Math.Ceiling(Index);
            if (Lo == Hi)
            {
                return SortedValues[Lo];
            }

            Double Weight = Index - Lo;
            return (Single)((SortedValues[Lo] * (1.0 - Weight)) + (SortedValues[Hi] * Weight));
        }

        /// <summary>
        /// Executes the heat map color operation.
        /// </summary>
        /// <param name="T">The t used by the operation.</param>
        /// <returns>The resulting heat map color.</returns>

        private static Rgba32 HeatMapColor
            (
            Single T
            )
        {
            // Yellow -> orange -> red ramp (_HEAT_STOPS / _HEAT_COLORS in Python render.py).
            Double[] Stops = { 0.0, Constants.HALF_SCALE, 1.0 };
            Double[] Rs = { Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, 200.0 };
            Double[] Gs = { 225.0, 110.0, 0.0 };
            Double[] Bs = { 0.0, 0.0, 0.0 };
            return new Rgba32(
                TruncByte(NumpyInterp(T, Stops, Rs)),
                TruncByte(NumpyInterp(T, Stops, Gs)),
                TruncByte(NumpyInterp(T, Stops, Bs)),
                Constants.MAX_COLOR_CHANNEL);
        }

        /// <summary>
        /// Executes the numpy interp operation.
        /// </summary>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Xp">The xp used by the operation.</param>
        /// <param name="Fp">The fp used by the operation.</param>
        /// <returns>The resulting numpy interp.</returns>

        private static Double NumpyInterp
            (
            Double X,
            Double[] Xp,
            Double[] Fp
            )
        {
            if (X <= Xp[0])
            {
                return Fp[0];
            }

            Int32 Last = Xp.Length - 1;
            if (X >= Xp[Last])
            {
                return Fp[Last];
            }

            for (Int32 Iter = 0; Iter < Last; Iter += 1)
            {
                if (X < Xp[Iter + 1])
                {
                    Double T = (X - Xp[Iter]) / (Xp[Iter + 1] - Xp[Iter]);
                    return Fp[Iter] + (T * (Fp[Iter + 1] - Fp[Iter]));
                }
            }

            return Fp[Last];
        }

        /// <summary>
        /// Executes the alpha composite pixel operation.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Source">The source used by the operation.</param>
        /// <param name="SourceAlpha">The source alpha used by the operation.</param>

        private static void AlphaCompositePixel
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Rgba32 Source,
            Byte SourceAlpha
            )
        {
            // PIL Image.alpha_composite over the existing base pixel.
            Rgba32 Dest = Image[X, Y];
            Double Sa = SourceAlpha / Constants.MAX_COLOR_CHANNEL;
            Double Da = Dest.A / Constants.MAX_COLOR_CHANNEL;
            Double OutA = Sa + (Da * (1.0 - Sa));
            if (OutA <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                Image[X, Y] = new Rgba32(0, 0, 0, 0);
                return;
            }

            Byte R = (Byte)Math.Round(((Source.R * Sa) + (Dest.R * Da * (1.0 - Sa))) / OutA, MidpointRounding.AwayFromZero);
            Byte G = (Byte)Math.Round(((Source.G * Sa) + (Dest.G * Da * (1.0 - Sa))) / OutA, MidpointRounding.AwayFromZero);
            Byte B = (Byte)Math.Round(((Source.B * Sa) + (Dest.B * Da * (1.0 - Sa))) / OutA, MidpointRounding.AwayFromZero);
            Byte A = (Byte)Math.Round(OutA * Constants.MAX_COLOR_CHANNEL, MidpointRounding.AwayFromZero);
            Image[X, Y] = new Rgba32(R, G, B, A);
        }

        /// <summary>
        /// Copies a whole-field sheet and rings two cells.
        /// Rows match the terrain image: column across, and row 0 at the bottom.
        /// </summary>
        /// <param name="SourcePath">Sheet to copy.</param>
        /// <param name="DestPath">Sheet to write.</param>
        /// <param name="PixelsPerCell">Pixels along one grid cell.</param>
        /// <param name="HighBx">Column of the highest cell.</param>
        /// <param name="HighBy">Row of the highest cell.</param>
        /// <param name="HighLabel">Text beside the highest cell.</param>
        /// <param name="LowBx">Column of the lowest cell.</param>
        /// <param name="LowBy">Row of the lowest cell.</param>
        /// <param name="LowLabel">Text beside the lowest cell.</param>
        internal static void MarkSpotsOnSheet
            (
            String SourcePath,
            String DestPath,
            Int32 PixelsPerCell,
            Int32 HighBx,
            Int32 HighBy,
            String HighLabel,
            Int32 LowBx,
            Int32 LowBy,
            String LowLabel
            )
        {
            if (String.IsNullOrEmpty(SourcePath)) throw new ArgumentNullException(nameof(SourcePath));
            if (String.IsNullOrEmpty(DestPath)) throw new ArgumentNullException(nameof(DestPath));
            if (PixelsPerCell < 1) throw new ArgumentOutOfRangeException(nameof(PixelsPerCell));
            using (Image<Rgba32> Sheet = Image.Load<Rgba32>(SourcePath))
            {
                Int32 GridHeight = Sheet.Height / PixelsPerCell;
                // The ring is several cells wide so it survives a look at the whole field.
                MarkOneSpot(Sheet, PixelsPerCell, GridHeight, HighBx, HighBy, HighLabel);
                MarkOneSpot(Sheet, PixelsPerCell, GridHeight, LowBx, LowBy, LowLabel);
                Sheet.Save(DestPath);
            }
        }

        /// <summary>
        /// Rings one cell and writes its label beside the ring.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="PixelsPerCell">Pixels along one grid cell.</param>
        /// <param name="GridHeight">Rows in the sheet.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row, with row 0 at the bottom of the sheet.</param>
        /// <param name="Label">Text beside the ring.</param>
        private static void MarkOneSpot
            (
            Image<Rgba32> Sheet,
            Int32 PixelsPerCell,
            Int32 GridHeight,
            Int32 Bx,
            Int32 By,
            String Label
            )
        {
            Int32 X0 = Bx * PixelsPerCell;
            Int32 Y0 = (GridHeight - 1 - By) * PixelsPerCell;
            Int32 Cx = X0 + (PixelsPerCell / 2);
            Int32 Cy = Y0 + (PixelsPerCell / 2);
            Rgba32 White = new Rgba32(255, 255, 255, 255);
            Rgba32 Black = new Rgba32(0, 0, 0, 255);
            // A ring this size stays on the sheet even when the cell is one row off the south edge.
            const Int32 RingOuter = 36;
            Int32 Scale = 4;
            Int32 PlateW = (Label.Length * 6 * Scale) + 16;
            Int32 PlateH = (7 * Scale) + 16;
            // Keep the plate clear of the ring. Spots near the south edge take the plate to the north.
            Int32 PlateX = Cx + RingOuter + 12;
            Int32 PlateY = Cy - RingOuter - PlateH - 8;
            if (PlateX + PlateW > Sheet.Width - 8)
            {
                PlateX = Cx - RingOuter - PlateW - 12;
            }
            if (PlateX < 8)
            {
                PlateX = 8;
            }
            if (PlateY < 8)
            {
                PlateY = Cy + RingOuter + 8;
            }
            if (PlateY + PlateH > Sheet.Height - 8)
            {
                PlateY = Sheet.Height - 8 - PlateH;
            }
            DrawLine(Sheet, Cx, Cy, PlateX, PlateY + (PlateH / 2), White, 5);
            DrawLine(Sheet, Cx, Cy, PlateX, PlateY + (PlateH / 2), Black, 2);
            // Halo first, then the black ring, so the mark reads on red cut and blue fill.
            DrawRing(Sheet, Cx, Cy, RingOuter, 5, White);
            DrawRing(Sheet, Cx, Cy, 30, 3, Black);
            DrawCellFrame(Sheet, X0, Y0, PixelsPerCell, White, 3);
            DrawCellFrame(Sheet, X0 + 3, Y0 + 3, PixelsPerCell - 6, Black, 2);
            FillRect(Sheet, PlateX, PlateY, PlateW, PlateH, Black);
            DrawRect(Sheet, PlateX, PlateY, PlateW, PlateH, White, 2);
            DrawLabel(Sheet, PlateX + 8, PlateY + 8, Label, Scale, White);
        }

        /// <summary>
        /// Paints a ring. The hole in the middle leaves the cell colour visible.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="Cx">Centre column.</param>
        /// <param name="Cy">Centre row.</param>
        /// <param name="Radius">Outer radius in pixels.</param>
        /// <param name="Thickness">Ring width in pixels.</param>
        /// <param name="Color">Ring colour.</param>
        private static void DrawRing
            (
            Image<Rgba32> Sheet,
            Int32 Cx,
            Int32 Cy,
            Int32 Radius,
            Int32 Thickness,
            Rgba32 Color
            )
        {
            Int32 Outer = Radius * Radius;
            Int32 Inner = Math.Max(0, Radius - Thickness);
            Int32 InnerSq = Inner * Inner;
            for (Int32 Y = Cy - Radius; Y <= Cy + Radius; Y += 1)
            {
                for (Int32 X = Cx - Radius; X <= Cx + Radius; X += 1)
                {
                    Int32 Dx = X - Cx;
                    Int32 Dy = Y - Cy;
                    Int32 Dist = (Dx * Dx) + (Dy * Dy);
                    if ((Dist <= Outer) && (Dist >= InnerSq))
                    {
                        Plot(Sheet, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Outlines one grid cell.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X0">Left pixel of the cell.</param>
        /// <param name="Y0">Top pixel of the cell.</param>
        /// <param name="Size">Cell size in pixels.</param>
        /// <param name="Color">Frame colour.</param>
        /// <param name="Thickness">Frame width in pixels.</param>
        private static void DrawCellFrame
            (
            Image<Rgba32> Sheet,
            Int32 X0,
            Int32 Y0,
            Int32 Size,
            Rgba32 Color,
            Int32 Thickness
            )
        {
            if (Size <= 0)
            {
                return;
            }
            FillRect(Sheet, X0, Y0, Size, Thickness, Color);
            FillRect(Sheet, X0, Y0 + Size - Thickness, Size, Thickness, Color);
            FillRect(Sheet, X0, Y0, Thickness, Size, Color);
            FillRect(Sheet, X0 + Size - Thickness, Y0, Thickness, Size, Color);
        }

        /// <summary>
        /// Fills a rectangle, clipped to the sheet.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X0">Left pixel.</param>
        /// <param name="Y0">Top pixel.</param>
        /// <param name="Width">Width in pixels.</param>
        /// <param name="Height">Height in pixels.</param>
        /// <param name="Color">Fill colour.</param>
        private static void FillRect
            (
            Image<Rgba32> Sheet,
            Int32 X0,
            Int32 Y0,
            Int32 Width,
            Int32 Height,
            Rgba32 Color
            )
        {
            for (Int32 Y = Y0; Y < Y0 + Height; Y += 1)
            {
                for (Int32 X = X0; X < X0 + Width; X += 1)
                {
                    Plot(Sheet, X, Y, Color);
                }
            }
        }

        /// <summary>
        /// Strokes the edge of a rectangle.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X0">Left pixel.</param>
        /// <param name="Y0">Top pixel.</param>
        /// <param name="Width">Width in pixels.</param>
        /// <param name="Height">Height in pixels.</param>
        /// <param name="Color">Stroke colour.</param>
        /// <param name="Thickness">Stroke width in pixels.</param>
        private static void DrawRect
            (
            Image<Rgba32> Sheet,
            Int32 X0,
            Int32 Y0,
            Int32 Width,
            Int32 Height,
            Rgba32 Color,
            Int32 Thickness
            )
        {
            FillRect(Sheet, X0, Y0, Width, Thickness, Color);
            FillRect(Sheet, X0, Y0 + Height - Thickness, Width, Thickness, Color);
            FillRect(Sheet, X0, Y0, Thickness, Height, Color);
            FillRect(Sheet, X0 + Width - Thickness, Y0, Thickness, Height, Color);
        }

        /// <summary>
        /// Writes a label in a 5 by 7 pixel face, scaled up.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X0">Left pixel of the first character.</param>
        /// <param name="Y0">Top pixel of the first character.</param>
        /// <param name="Label">Text to write. Unknown letters are skipped as blanks.</param>
        /// <param name="Scale">Pixels per font pixel.</param>
        /// <param name="Color">Ink colour.</param>
        private static void DrawLabel
            (
            Image<Rgba32> Sheet,
            Int32 X0,
            Int32 Y0,
            String Label,
            Int32 Scale,
            Rgba32 Color
            )
        {
            Int32 Cursor = X0;
            foreach (Char Letter in Label)
            {
                String[] Rows = SpotGlyph(Letter);
                for (Int32 Row = 0; Row < Rows.Length; Row += 1)
                {
                    for (Int32 Col = 0; Col < Rows[Row].Length; Col += 1)
                    {
                        if (Rows[Row][Col] != '1')
                        {
                            continue;
                        }
                        FillRect(Sheet, Cursor + (Col * Scale), Y0 + (Row * Scale), Scale, Scale, Color);
                    }
                }
                Cursor += 6 * Scale;
            }
        }

        /// <summary>
        /// Five-pixel rows of one character. A 1 is ink.
        /// </summary>
        /// <param name="Letter">Character to look up.</param>
        /// <returns>Seven rows. A blank face is used for a character this face does not have.</returns>
        private static String[] SpotGlyph
            (
            Char Letter
            )
        {
            switch (Letter)
            {
                case 'H': return new[] { "10001", "10001", "10001", "11111", "10001", "10001", "10001" };
                case 'I': return new[] { "11111", "00100", "00100", "00100", "00100", "00100", "11111" };
                case 'G': return new[] { "01110", "10001", "10000", "10111", "10001", "10001", "01110" };
                case 'L': return new[] { "10000", "10000", "10000", "10000", "10000", "10000", "11111" };
                case 'O': return new[] { "01110", "10001", "10001", "10001", "10001", "10001", "01110" };
                case 'W': return new[] { "10001", "10001", "10001", "10101", "10101", "10101", "01010" };
                case 'M': return new[] { "10001", "11011", "10101", "10101", "10001", "10001", "10001" };
                case 'm': return new[] { "00000", "00000", "11011", "10101", "10101", "10101", "10101" };
                case '+': return new[] { "00000", "00100", "00100", "11111", "00100", "00100", "00000" };
                case '-': return new[] { "00000", "00000", "00000", "11111", "00000", "00000", "00000" };
                case '4': return new[] { "00100", "01100", "10100", "10100", "11111", "00100", "00100" };
                case '5': return new[] { "11111", "10000", "11110", "00001", "00001", "10001", "01110" };
                case '8': return new[] { "01110", "10001", "10001", "01110", "10001", "10001", "01110" };
                case '9': return new[] { "01110", "10001", "10001", "01111", "00001", "00001", "01110" };
                case ' ': return new[] { "00000", "00000", "00000", "00000", "00000", "00000", "00000" };
                default: return new[] { "00000", "00000", "00000", "00000", "00000", "00000", "00000" };
            }
        }
    }
}
