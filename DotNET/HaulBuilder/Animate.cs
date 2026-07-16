using System.Globalization;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Gif;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

namespace HaulBuilder
{
    /// <summary>
    /// Frame-by-frame leveling animation utilities ported from <c>landlevel.animate</c>.
    /// </summary>
    public static class Animate
    {
        /// <summary>
        /// One in-memory animation payload.
        /// </summary>
        public sealed class AnimationPayload
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="AnimationPayload"/> class.
            /// </summary>
            public AnimationPayload
                (
                )
            {
                // Frames are kept in route order so callers can stream or persist them deterministically.
                FramePngBytes = new List<Byte[]>();
            }

            /// <summary>
            /// Gets route-ordered frame PNG buffers.
            /// </summary>
            public List<Byte[]> FramePngBytes { get; }

            /// <summary>
            /// Gets or sets assembled GIF bytes.
            /// </summary>
            public Byte[]? GifBytes { get; set; }
        }

        /// <summary>
        /// Animates the field as ordered passes are applied.
        /// </summary>
        /// <param name="grid">Grid model.</param>
        /// <param name="orderedPasses">Passes in route order.</param>
        /// <param name="stride">One rendered frame per N passes.</param>
        /// <param name="pixelsPerBin">Pixels per area edge for frames.</param>
        /// <param name="gifMs">Default GIF frame delay in milliseconds.</param>
        /// <param name="mode">Render mode <c>cutfill</c> or <c>topology</c>.</param>
        /// <returns>In-memory frame PNGs and optional GIF bytes.</returns>
        static public AnimationPayload AnimateLeveling
            (
            Grid grid,
            IReadOnlyList<Pass> orderedPasses,
            Int32 stride = 10,
            Int32 pixelsPerBin = 8,
            Int32 gifMs = 80,
            String mode = "cutfill"
            )
        {
            // Existing is cloned so animation never mutates caller-owned state.
            AnimationPayload Payload = new AnimationPayload();
            Double[,] Existing = Clone(grid.Existing);
            Double[,] Target = grid.Target;
            Boolean[,] Valid = grid.Valid;
            Int32 Ppb = Math.Max(1, pixelsPerBin);
            Int32 Stride = Math.Max(1, stride);
            (Double VMin, Double VMax) = ComputeRange(Existing, Target, Valid, mode);

            Action Emit = () =>
            {
                // Frame rendering keeps topology color scale fixed to initial bounds.
                using Image<Rgba32> Frame = BuildFrame(Existing, Target, Valid, Ppb, mode, VMin, VMax);
                Payload.FramePngBytes.Add(ToPng(Frame));
            };

            Emit();
            Int32 Count = orderedPasses.Count;
            for (Int32 PassNo = 1; PassNo <= Count; PassNo += 1)
            {
                dynamic Pass = orderedPasses[PassNo - 1];
                foreach (dynamic Cell in Pass.CutCells)
                {
                    Existing[(Int32)Cell.Cell.By, (Int32)Cell.Cell.Bx] -= (Double)Cell.DepthM;
                }

                foreach (dynamic Cell in Pass.FillCells)
                {
                    Existing[(Int32)Cell.Cell.By, (Int32)Cell.Cell.Bx] += (Double)Cell.DepthM;
                }

                if (((PassNo % Stride) == 0) || (PassNo == Count))
                {
                    Emit();
                }
            }

            if (Payload.FramePngBytes.Count > 0)
            {
                Payload.GifBytes = BuildGif(Payload.FramePngBytes, gifMs);
            }

            return Payload;
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public AnimationPayload animate_leveling
            (
            Grid grid,
            IReadOnlyList<Pass> ordered_passes,
            Int32 stride = 10,
            Int32 pixels_per_bin = 8,
            Int32 gif_ms = 80,
            String mode = "cutfill"
            ) => AnimateLeveling(grid, ordered_passes, stride, pixels_per_bin, gif_ms, mode);

        /// <summary>
        /// Options for the side-by-side cut/fill + topology leveling animation.
        /// </summary>
        public sealed class SplitLevelingOptions
        {
            /// <summary>Gets or sets pixels per grid cell edge.</summary>
            public Int32 PixelsPerBin { get; set; } = 4;

            /// <summary>Gets or sets target frame count after the early pass-by-pass section.</summary>
            public Int32 TargetFrames { get; set; } = 500;

            /// <summary>Gets or sets how many initial passes emit one frame each.</summary>
            public Int32 EarlyPerPass { get; set; } = 40;

            /// <summary>Gets or sets GIF frame delay in milliseconds.</summary>
            public Int32 GifMs { get; set; } = 90;

            /// <summary>
            /// Gets or sets the optional exact duration of the progressing
            /// portion of the GIF.  The final completed frame hold is not
            /// included.  When omitted, <see cref="GifMs"/> is applied to
            /// every progressing frame.
            /// </summary>
            public Int32? ProgressDurationMs { get; set; }

            /// <summary>Gets or sets the completed-frame hold duration.</summary>
            public Int32 FinalHoldMs { get; set; } = 4000;

            /// <summary>Gets or sets on-grade tolerance in metres.</summary>
            public Double ToleranceM { get; set; } = 0.010;

            /// <summary>Gets or sets header band height in pixels.</summary>
            public Int32 HeaderHeight { get; set; } = 64;

            /// <summary>Gets or sets gap between panels and margins.</summary>
            public Int32 Gap { get; set; } = 14;

            /// <summary>Gets or sets optional title shown in console output.</summary>
            public String Title { get; set; } = "Land leveling by pass (route order)";
        }

        /// <summary>
        /// Animates leveling with cut/fill on the left and topology on the right.
        /// </summary>
        /// <param name="grid">Grid model.</param>
        /// <param name="orderedPasses">Passes in route order.</param>
        /// <param name="options">Optional render cadence and layout settings.</param>
        /// <returns>Composite frame PNGs and assembled GIF bytes.</returns>
        static public AnimationPayload AnimateLevelingSplitView
            (
            Grid grid,
            IReadOnlyList<Pass> orderedPasses,
            SplitLevelingOptions? options = null
            )
        {
            SplitLevelingOptions Opt = options ?? new SplitLevelingOptions();
            AnimationPayload Payload = new AnimationPayload();
            Double[,] Existing = Clone(grid.Existing);
            Double[,] Target = grid.Target;
            Boolean[,] Valid = grid.Valid;
            Int32 Ppb = Math.Max(1, Opt.PixelsPerBin);
            Int32 PassCount = orderedPasses.Count;
            (Double VMin, Double VMax) = ComputeRange(Existing, Target, Valid, "topology");
            Double TotalCutBcy = ComputeTotalCutBcy(grid);
            HashSet<Int32> EmitAt = ComputeEmitPasses(PassCount, Opt.TargetFrames, Opt.EarlyPerPass);
            Int32 PanelW = grid.Width * Ppb;
            Int32 PanelH = grid.Height * Ppb;
            Int32 CanvasW = (PanelW * 2) + (Opt.Gap * 3);
            Int32 CanvasH = Opt.HeaderHeight + PanelH + Opt.Gap;
            Double MovedBcy = 0.0;
            Int32 FrameNo = 0;

            Action<Int32> Emit = (Int32 PassDone) =>
            {
                using Image<Rgba32> Canvas = BuildSplitFrame(
                    Existing,
                    Target,
                    Valid,
                    Ppb,
                    VMin,
                    VMax,
                    Opt,
                    PanelW,
                    PanelH,
                    CanvasW,
                    CanvasH,
                    PassDone,
                    PassCount,
                    MovedBcy,
                    TotalCutBcy,
                    Opt.ToleranceM);
                Payload.FramePngBytes.Add(ToPng(Canvas));
                Double OnGrade = ComputeOnGradeFraction(Existing, Target, Valid, Opt.ToleranceM) * 100.0;
                Console.WriteLine(
                    String.Format(
                        CultureInfo.InvariantCulture,
                        "  frame {0}: pass {1}/{2}  on-grade {3:F1}%  moved {4:F0}/{5:F0} BCY",
                        FrameNo,
                        PassDone,
                        PassCount,
                        OnGrade,
                        MovedBcy,
                        TotalCutBcy));
                FrameNo += 1;
            };

            Emit(0);
            for (Int32 PassNo = 1; PassNo <= PassCount; PassNo += 1)
            {
                Pass Pass = orderedPasses[PassNo - 1];
                ApplyPass(Existing, Pass, grid.AreaM2, ref MovedBcy);
                if (EmitAt.Contains(PassNo))
                {
                    Emit(PassNo);
                }
            }

            if (Payload.FramePngBytes.Count > 0)
            {
                Payload.GifBytes = BuildGif(
                    Payload.FramePngBytes,
                    Opt.GifMs,
                    Opt.ProgressDurationMs,
                    Opt.FinalHoldMs);
            }

            return Payload;
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public AnimationPayload animate_leveling_split_view
            (
            Grid grid,
            IReadOnlyList<Pass> ordered_passes,
            SplitLevelingOptions? options = null
            ) => AnimateLevelingSplitView(grid, ordered_passes, options);

        /// <summary>
        /// Pivot-animation hook that forwards to the dedicated pivot module.
        /// </summary>
        /// <param name="grid">Grid model.</param>
        /// <param name="orderedPasses">Passes in route order.</param>
        /// <param name="plan">Pivot plan.</param>
        /// <param name="stride">Render stride in passes.</param>
        /// <param name="pixelsPerBin">Pixels per grid cell.</param>
        /// <param name="mode">Render mode <c>cutfill</c> or <c>topology</c>.</param>
        /// <returns>In-memory frames and GIF bytes.</returns>
        static public AnimationPayload AnimatePivotLeveling
            (
            Grid grid,
            IReadOnlyList<Pass> orderedPasses,
            Pivot.PivotPlan plan,
            Int32 stride = 15,
            Int32 pixelsPerBin = 6,
            String mode = "cutfill"
            )
        {
            // Pivot module already owns move-timing semantics, so this wrapper only marshals temporary files to bytes.
            String Root = Path.Combine(Path.GetTempPath(), "haulbuilder_pivot_anim_" + Guid.NewGuid().ToString("N"));
            String FramesDir = Path.Combine(Root, "frames");
            String GifPath = Path.Combine(Root, "pivot.gif");
            Directory.CreateDirectory(FramesDir);
            Pivot.Grid PivotGrid = ToPivotGrid(grid);
            List<Pivot.Pass> PivotPasses = ToPivotPasses(orderedPasses);

            Int32 FrameCount = Pivot.AnimatePivotLeveling
                (
                PivotGrid,
                PivotPasses,
                plan,
                GifPath,
                FramesDir,
                stride,
                pixelsPerBin,
                mode,
                90
                );

            AnimationPayload Payload = new AnimationPayload();
            if (File.Exists(GifPath))
            {
                Payload.GifBytes = File.ReadAllBytes(GifPath);
            }

            for (Int32 Iter = 0; Iter < FrameCount; Iter += 1)
            {
                String FramePath = Path.Combine(FramesDir, String.Format(System.Globalization.CultureInfo.InvariantCulture, "frame_{0:D5}.png", Iter));
                if (File.Exists(FramePath))
                {
                    Payload.FramePngBytes.Add(File.ReadAllBytes(FramePath));
                }
            }

            try
            {
                Directory.Delete(Root, true);
            }
            catch (IOException)
            {
                // Best-effort cleanup only; caller data is already materialized.
            }
            catch (UnauthorizedAccessException)
            {
                // Best-effort cleanup only; caller data is already materialized.
            }

            return Payload;
        }

        /// <summary>
        /// Snake-case alias for Python naming parity.
        /// </summary>
        static public AnimationPayload animate_pivot_leveling
            (
            Grid grid,
            IReadOnlyList<Pass> ordered_passes,
            Pivot.PivotPlan plan,
            Int32 stride = 15,
            Int32 pixels_per_bin = 6,
            String mode = "cutfill"
            ) => AnimatePivotLeveling(grid, ordered_passes, plan, stride, pixels_per_bin, mode);

        private static HashSet<Int32> ComputeEmitPasses
            (
            Int32 passCount,
            Int32 targetFrames,
            Int32 earlyPerPass
            )
        {
            HashSet<Int32> EmitAt = new HashSet<Int32>();
            if (passCount <= 0)
            {
                return EmitAt;
            }

            Int32 Early = Math.Min(earlyPerPass, passCount);
            for (Int32 PassNo = 1; PassNo <= Early; PassNo += 1)
            {
                EmitAt.Add(PassNo);
            }

            Int32 Rest = Math.Max(1, (Int32)Math.Round(((Double)(passCount - Early) / Math.Max(1, targetFrames - Early)), MidpointRounding.AwayFromZero));
            for (Int32 PassNo = Early; PassNo <= passCount; PassNo += Rest)
            {
                EmitAt.Add(PassNo);
            }

            EmitAt.Add(passCount);
            return EmitAt;
        }

        private static Double ComputeTotalCutBcy
            (
            Grid grid
            )
        {
            Double[,] Delta = grid.Delta();
            Double Total = 0.0;
            Int32 Height = Delta.GetLength(0);
            Int32 Width = Delta.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!grid.Valid[By, Bx]) continue;
                    Double Value = Delta[By, Bx];
                    if (Value > 0.0)
                    {
                        Total += Value;
                    }
                }
            }

            return Total * grid.AreaM2 * Constants.M3_TO_CY;
        }

        private static void ApplyPass
            (
            Double[,] existing,
            Pass pass,
            Double areaM2,
            ref Double movedBcy
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in pass.CutCells)
            {
                existing[Entry.Cell.By, Entry.Cell.Bx] -= Entry.DepthM;
                movedBcy += Entry.DepthM * areaM2 * Constants.M3_TO_CY;
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in pass.FillCells)
            {
                existing[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
            }
        }

        private static Double ComputeOnGradeFraction
            (
            Double[,] existing,
            Double[,] target,
            Boolean[,] valid,
            Double toleranceM
            )
        {
            Int32 OnGrade = 0;
            Int32 Count = 0;
            Int32 Height = existing.GetLength(0);
            Int32 Width = existing.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!valid[By, Bx]) continue;
                    Count += 1;
                    if (Math.Abs(existing[By, Bx] - target[By, Bx]) <= toleranceM)
                    {
                        OnGrade += 1;
                    }
                }
            }

            return Count > 0 ? ((Double)OnGrade / Count) : 0.0;
        }

        private static Image<Rgba32> BuildSplitFrame
            (
            Double[,] existing,
            Double[,] target,
            Boolean[,] valid,
            Int32 ppb,
            Double vmin,
            Double vmax,
            SplitLevelingOptions opt,
            Int32 panelW,
            Int32 panelH,
            Int32 canvasW,
            Int32 canvasH,
            Int32 passDone,
            Int32 passCount,
            Double movedBcy,
            Double totalCutBcy,
            Double toleranceM
            )
        {
            Int32 Height = existing.GetLength(0);
            Int32 Width = existing.GetLength(1);
            Double[,] Diff = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Diff[By, Bx] = existing[By, Bx] - target[By, Bx];
                }
            }

            using Image<Rgba32> CutFill = Render.DiffToImageDigitize(Diff, valid, ppb);
            using Image<Rgba32> Topo = Render.ElevationToImage(existing, valid, ppb, vmin, vmax);
            Image<Rgba32> Canvas = new Image<Rgba32>(canvasW, canvasH);
            Canvas.Mutate(Context => Context.BackgroundColor(new Rgba32(255, 255, 255, 255)));
            Canvas.Mutate(Context => Context.DrawImage(CutFill, new Point(opt.Gap, opt.HeaderHeight), 1.0f));
            Canvas.Mutate(Context => Context.DrawImage(Topo, new Point((opt.Gap * 2) + panelW, opt.HeaderHeight), 1.0f));

            Double Progress = passCount > 0 ? ((Double)passDone / passCount) : 1.0;
            Int32 BarY = opt.HeaderHeight - 16;
            Int32 BarMaxW = canvasW - (opt.Gap * 2);
            Int32 BarW = Math.Max(0, (Int32)Math.Round(BarMaxW * Progress, MidpointRounding.AwayFromZero));
            FillRect(Canvas, opt.Gap, BarY, BarMaxW, 8, new Rgba32(230, 230, 230, 255));
            FillRect(Canvas, opt.Gap, BarY, BarW, 8, new Rgba32(40, 120, 40, 255));

            DrawCaptionBar(Canvas, opt.Gap, opt.HeaderHeight + 2, panelW, "CUT / FILL");
            DrawCaptionBar(Canvas, (opt.Gap * 2) + panelW, opt.HeaderHeight + 2, panelW, "TOPOLOGY");

            Double OnGradePct = ComputeOnGradeFraction(existing, target, valid, toleranceM) * 100.0;
            String Stats = String.Format(
                CultureInfo.InvariantCulture,
                "pass {0}/{1}  on-grade {2:F1}%  moved {3:F0}/{4:F0} BCY",
                passDone,
                passCount,
                OnGradePct,
                movedBcy,
                totalCutBcy);
            DrawStatsLine(Canvas, opt.Gap, 8, Stats);

            return Canvas;
        }

        private static void DrawCaptionBar
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            Int32 width,
            String label
            )
        {
            FillRect(image, x, y, width, 18, new Rgba32(20, 20, 20, 170));
            DrawSimpleText(image, x + 6, y + 6, label, new Rgba32(255, 255, 255, 255));
        }

        private static void DrawStatsLine
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            String text
            )
        {
            DrawSimpleText(image, x, y, text, new Rgba32(20, 20, 20, 255));
        }

        private static void DrawSimpleText
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            String text,
            Rgba32 color
            )
        {
            Int32 CursorX = x;
            foreach (Char Ch in text)
            {
                if (Ch == ' ')
                {
                    CursorX += 4;
                    continue;
                }

                DrawMiniGlyph(image, Ch, CursorX, y, color);
                CursorX += 6;
            }
        }

        private static void DrawMiniGlyph
            (
            Image<Rgba32> image,
            Char ch,
            Int32 x,
            Int32 y,
            Rgba32 color
            )
        {
            String Pattern = ch switch
            {
                'A' => "010,111,101,101,101",
                'B' => "110,101,110,101,110",
                'C' => "011,100,100,100,011",
                'D' => "110,101,101,101,110",
                'E' => "111,100,110,100,111",
                'F' => "111,100,110,100,100",
                'G' => "011,100,101,101,011",
                'H' => "101,101,111,101,101",
                'I' => "111,010,010,010,111",
                'L' => "100,100,100,100,111",
                'M' => "101,111,111,101,101",
                'N' => "101,111,111,111,101",
                'O' => "010,101,101,101,010",
                'P' => "110,101,110,100,100",
                'R' => "110,101,110,101,101",
                'S' => "011,100,010,001,110",
                'T' => "111,010,010,010,010",
                'U' => "101,101,101,101,111",
                'Y' => "101,101,010,010,010",
                '0' => "111,101,101,101,111",
                '1' => "010,110,010,010,111",
                '2' => "111,001,111,100,111",
                '3' => "111,001,111,001,111",
                '4' => "101,101,111,001,001",
                '5' => "111,100,111,001,111",
                '6' => "111,100,111,101,111",
                '7' => "111,001,001,001,001",
                '8' => "111,101,111,101,111",
                '9' => "111,101,111,001,111",
                '/' => "001,001,010,100,100",
                '%' => "100,001,010,100,001",
                '.' => "000,000,000,000,010",
                ':' => "010,010,000,010,010",
                '-' => "000,000,111,000,000",
                _ => String.Empty,
            };

            if (Pattern.Length == 0)
            {
                return;
            }

            String[] Rows = Pattern.Split(',');
            for (Int32 Row = 0; Row < Rows.Length; Row += 1)
            {
                String Line = Rows[Row];
                for (Int32 Col = 0; Col < Line.Length; Col += 1)
                {
                    if (Line[Col] != '1') continue;
                    PlotPixel(image, x + Col, y + Row, color);
                }
            }
        }

        private static void FillRect
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            Int32 width,
            Int32 height,
            Rgba32 color
            )
        {
            Int32 X1 = Math.Min(image.Width, x + width);
            Int32 Y1 = Math.Min(image.Height, y + height);
            for (Int32 Py = Math.Max(0, y); Py < Y1; Py += 1)
            {
                for (Int32 Px = Math.Max(0, x); Px < X1; Px += 1)
                {
                    image[Px, Py] = color;
                }
            }
        }

        private static void PlotPixel
            (
            Image<Rgba32> image,
            Int32 x,
            Int32 y,
            Rgba32 color
            )
        {
            if ((x < 0) || (y < 0) || (x >= image.Width) || (y >= image.Height))
            {
                return;
            }

            image[x, y] = color;
        }

        private static Pivot.Grid ToPivotGrid
            (
            Grid grid
            )
        {
            // Adapt top-level grid model for Pivot module APIs.
            return new Pivot.Grid(grid.GridSizeM, grid.MinX, grid.MinY, grid.Zone, grid.IsNorthern, grid.Existing, grid.Target, grid.Valid);
        }

        private static List<Pivot.Pass> ToPivotPasses
            (
            IReadOnlyList<Pass> passes
            )
        {
            // Convert pass tuple payloads into Pivot pass DTOs.
            List<Pivot.Pass> Out = new List<Pivot.Pass>();
            for (Int32 Iter = 0; Iter < passes.Count; Iter += 1)
            {
                List<Pivot.PassCell> Cut = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in passes[Iter].CutCells)
                {
                    Cut.Add(new Pivot.PassCell(Cell.Cell.Bx, Cell.Cell.By, Cell.DepthM));
                }

                List<Pivot.PassCell> Fill = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in passes[Iter].FillCells)
                {
                    Fill.Add(new Pivot.PassCell(Cell.Cell.Bx, Cell.Cell.By, Cell.DepthM));
                }

                Out.Add(new Pivot.Pass(Cut, Fill, passes[Iter].CutEndXy, passes[Iter].FillStartXy));
            }

            return Out;
        }

        private static (Double VMin, Double VMax) ComputeRange
            (
            Double[,] existing,
            Double[,] target,
            Boolean[,] valid,
            String mode
            )
        {
            // Topology mode needs a fixed range across all frames; cutfill mode ignores this range.
            if (!String.Equals(mode, "topology", StringComparison.OrdinalIgnoreCase))
            {
                return (0.0, 0.0);
            }

            Double Min = Double.PositiveInfinity;
            Double Max = Double.NegativeInfinity;
            Int32 Height = existing.GetLength(0);
            Int32 Width = existing.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!valid[By, Bx]) continue;
                    Min = Math.Min(Min, Math.Min(existing[By, Bx], target[By, Bx]));
                    Max = Math.Max(Max, Math.Max(existing[By, Bx], target[By, Bx]));
                }
            }

            if (!Double.IsFinite(Min) || !Double.IsFinite(Max))
            {
                return (0.0, 1.0);
            }

            return (Min, Max);
        }

        private static Image<Rgba32> BuildFrame
            (
            Double[,] existing,
            Double[,] target,
            Boolean[,] valid,
            Int32 ppb,
            String mode,
            Double vmin,
            Double vmax
            )
        {
            // Render module owns the exact palette and coordinate conventions.
            if (String.Equals(mode, "topology", StringComparison.OrdinalIgnoreCase))
            {
                return Render.ElevationToImage(existing, valid, ppb, vmin, vmax);
            }

            Int32 Height = existing.GetLength(0);
            Int32 Width = existing.GetLength(1);
            Double[,] Diff = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Diff[By, Bx] = (existing[By, Bx] - target[By, Bx]);
                }
            }

            return Render.DiffToImage(Diff, valid, ppb);
        }

        private static Double[,] Clone
            (
            Double[,] src
            )
        {
            // Explicit deep copy prevents accidental aliasing between pipeline stages.
            Int32 Height = src.GetLength(0);
            Int32 Width = src.GetLength(1);
            Double[,] Dst = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Dst[By, Bx] = src[By, Bx];
                }
            }

            return Dst;
        }

        private static Byte[] ToPng
            (
            Image<Rgba32> image
            )
        {
            // Memory-stream PNG encoding keeps animation consumers file-agnostic.
            using MemoryStream Stream = new MemoryStream();
            image.SaveAsPng(Stream);
            return Stream.ToArray();
        }

        private static Byte[] BuildGif
            (
            IReadOnlyList<Byte[]> frames,
            Int32 gifMs,
            Int32? progressDurationMs = null,
            Int32 finalHoldMs = 4000
            )
        {
            // Last-frame hold mirrors Python UX so the completed grade remains visible.
            IReadOnlyList<Int32> DelaysCs = ComputeGifFrameDelaysCs(
                frames.Count,
                gifMs,
                progressDurationMs,
                finalHoldMs);
            using Image<Rgba32> First = Image.Load<Rgba32>(frames[0]);
            SetGifDelayCentiseconds(First.Frames.RootFrame, DelaysCs[0]);
            for (Int32 Iter = 1; Iter < frames.Count; Iter += 1)
            {
                using Image<Rgba32> Next = Image.Load<Rgba32>(frames[Iter]);
                ImageFrame<Rgba32> Added = First.Frames.AddFrame(Next.Frames.RootFrame);
                SetGifDelayCentiseconds(Added, DelaysCs[Iter]);
            }

            First.Metadata.GetGifMetadata().RepeatCount = 0;

            using MemoryStream Stream = new MemoryStream();
            First.Save(Stream, new GifEncoder());
            return Stream.ToArray();
        }

        /// <summary>Computes GIF frame delays in centiseconds.</summary>
        internal static IReadOnlyList<Int32> ComputeGifFrameDelaysCs
            (
            Int32 frameCount,
            Int32 gifMs,
            Int32? progressDurationMs,
            Int32 finalHoldMs
            )
        {
            if (frameCount <= 0)
            {
                return Array.Empty<Int32>();
            }

            Int32[] DelaysCs = new Int32[frameCount];
            Int32 ProgressFrameCount = frameCount - 1;
            if (ProgressFrameCount > 0)
            {
                if (progressDurationMs.HasValue)
                {
                    Int32 TotalProgressCs = Math.Max(
                        ProgressFrameCount,
                        (Int32)Math.Round(
                            progressDurationMs.Value / 10.0,
                            MidpointRounding.AwayFromZero));
                    for (Int32 Iter = 0; Iter < ProgressFrameCount; Iter += 1)
                    {
                        // Integer interval differences spread the rounding
                        // evenly while preserving the exact requested total.
                        Int64 Start = (Int64)Iter * TotalProgressCs
                            / ProgressFrameCount;
                        Int64 End = (Int64)(Iter + 1) * TotalProgressCs
                            / ProgressFrameCount;
                        DelaysCs[Iter] = Math.Max(1, (Int32)(End - Start));
                    }
                }
                else
                {
                    Int32 UniformDelayCs = Math.Max(
                        1,
                        (Int32)Math.Round(
                            gifMs / 10.0,
                            MidpointRounding.AwayFromZero));
                    Array.Fill(
                        DelaysCs,
                        UniformDelayCs,
                        0,
                        ProgressFrameCount);
                }
            }

            DelaysCs[frameCount - 1] = Math.Max(
                1,
                (Int32)Math.Round(
                    finalHoldMs / 10.0,
                    MidpointRounding.AwayFromZero));
            return DelaysCs;
        }

        private static void SetGifDelayCentiseconds
            (
            ImageFrame<Rgba32> frame,
            Int32 delayCs
            )
        {
            frame.Metadata.GetGifMetadata().FrameDelay = Math.Max(1, delayCs);
        }
    }
}
