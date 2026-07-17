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
    static public class Animate
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
        /// <param name="Grid">Grid model.</param>
        /// <param name="OrderedPasses">Passes in route order.</param>
        /// <param name="Stride">One rendered frame per N passes.</param>
        /// <param name="PixelsPerBin">Pixels per area edge for frames.</param>
        /// <param name="GifMs">Default GIF frame delay in milliseconds.</param>
        /// <param name="Mode">Render mode <c>cutfill</c> or <c>topology</c>.</param>
        /// <returns>In-memory frame PNGs and optional GIF bytes.</returns>
        static public AnimationPayload AnimateLeveling
            (
            Grid Grid,
            IReadOnlyList<Pass> OrderedPasses,
            Int32 Stride = 10,
            Int32 PixelsPerBin = 8,
            Int32 GifMs = 80,
            String Mode = "cutfill"
            )
        {
            // Existing is cloned so animation never mutates caller-owned state.
            AnimationPayload Payload = new AnimationPayload();
            Double[,] Existing = Clone(Grid.Existing);
            Double[,] Target = Grid.Target;
            Boolean[,] Valid = Grid.Valid;
            Int32 Ppb = Math.Max(1, PixelsPerBin);
            Int32 EffectiveStride = Math.Max(1, Stride);
            (Double VMin, Double VMax) = ComputeRange(Existing, Target, Valid, Mode);

            Action Emit = () =>
            {
                // Frame rendering keeps topology color scale fixed to initial bounds.
                using Image<Rgba32> Frame = BuildFrame(Existing, Target, Valid, Ppb, Mode, VMin, VMax);
                Payload.FramePngBytes.Add(ToPng(Frame));
            };

            Emit();
            Int32 Count = OrderedPasses.Count;
            for (Int32 PassNo = 1; PassNo <= Count; PassNo += 1)
            {
                dynamic Pass = OrderedPasses[PassNo - 1];
                foreach (dynamic Cell in Pass.CutCells)
                {
                    Existing[(Int32)Cell.Cell.By, (Int32)Cell.Cell.Bx] -= (Double)Cell.DepthM;
                }

                foreach (dynamic Cell in Pass.FillCells)
                {
                    Existing[(Int32)Cell.Cell.By, (Int32)Cell.Cell.Bx] += (Double)Cell.DepthM;
                }

                if (((PassNo % EffectiveStride) == 0) || (PassNo == Count))
                {
                    Emit();
                }
            }

            if (Payload.FramePngBytes.Count > 0)
            {
                Payload.GifBytes = BuildGif(Payload.FramePngBytes, GifMs);
            }

            return Payload;
        }


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
        /// <param name="Grid">Grid model.</param>
        /// <param name="OrderedPasses">Passes in route order.</param>
        /// <param name="Options">Optional render cadence and layout settings.</param>
        /// <returns>Composite frame PNGs and assembled GIF bytes.</returns>
        static public AnimationPayload AnimateLevelingSplitView
            (
            Grid Grid,
            IReadOnlyList<Pass> OrderedPasses,
            SplitLevelingOptions? Options = null
            )
        {
            SplitLevelingOptions Opt = Options ?? new SplitLevelingOptions();
            AnimationPayload Payload = new AnimationPayload();
            Double[,] Existing = Clone(Grid.Existing);
            Double[,] Target = Grid.Target;
            Boolean[,] Valid = Grid.Valid;
            Int32 Ppb = Math.Max(1, Opt.PixelsPerBin);
            Int32 PassCount = OrderedPasses.Count;
            (Double VMin, Double VMax) = ComputeRange(Existing, Target, Valid, "topology");
            Double TotalCutBcy = ComputeTotalCutBcy(Grid);
            HashSet<Int32> EmitAt = ComputeEmitPasses(PassCount, Opt.TargetFrames, Opt.EarlyPerPass);
            Int32 PanelW = Grid.Width * Ppb;
            Int32 PanelH = Grid.Height * Ppb;
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
                Pass Pass = OrderedPasses[PassNo - 1];
                ApplyPass(Existing, Pass, Grid.AreaM2, ref MovedBcy);
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
        /// Pivot-animation hook that forwards to the dedicated pivot module.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="OrderedPasses">Passes in route order.</param>
        /// <param name="Plan">Pivot plan.</param>
        /// <param name="Stride">Render stride in passes.</param>
        /// <param name="PixelsPerBin">Pixels per grid cell.</param>
        /// <param name="Mode">Render mode <c>cutfill</c> or <c>topology</c>.</param>
        /// <returns>In-memory frames and GIF bytes.</returns>
        static public AnimationPayload AnimatePivotLeveling
            (
            Grid Grid,
            IReadOnlyList<Pass> OrderedPasses,
            Pivot.PivotPlan Plan,
            Int32 Stride = 15,
            Int32 PixelsPerBin = 6,
            String Mode = "cutfill"
            )
        {
            // Pivot module already owns move-timing semantics, so this wrapper only marshals temporary files to bytes.
            String Root = Path.Combine(Path.GetTempPath(), "haulbuilder_pivot_anim_" + Guid.NewGuid().ToString("N"));
            String FramesDir = Path.Combine(Root, "frames");
            String GifPath = Path.Combine(Root, "pivot.gif");
            Directory.CreateDirectory(FramesDir);
            Pivot.Grid PivotGrid = ToPivotGrid(Grid);
            List<Pivot.Pass> PivotPasses = ToPivotPasses(OrderedPasses);

            Int32 FrameCount = Pivot.AnimatePivotLeveling
                (
                PivotGrid,
                PivotPasses,
                Plan,
                GifPath,
                FramesDir,
                Stride,
                PixelsPerBin,
                Mode,
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
        /// Calculates emit passes.
        /// </summary>
        /// <param name="PassCount">Count limit applied by the operation.</param>
        /// <param name="TargetFrames">The target frames used by the operation.</param>
        /// <param name="EarlyPerPass">The early per pass used by the operation.</param>
        /// <returns>The resulting emit passes.</returns>

        private static HashSet<Int32> ComputeEmitPasses
            (
            Int32 PassCount,
            Int32 TargetFrames,
            Int32 EarlyPerPass
            )
        {
            HashSet<Int32> EmitAt = new HashSet<Int32>();
            if (PassCount <= 0)
            {
                return EmitAt;
            }

            Int32 Early = Math.Min(EarlyPerPass, PassCount);
            for (Int32 PassNo = 1; PassNo <= Early; PassNo += 1)
            {
                EmitAt.Add(PassNo);
            }

            Int32 Rest = Math.Max(1, (Int32)Math.Round(((Double)(PassCount - Early) / Math.Max(1, TargetFrames - Early)), MidpointRounding.AwayFromZero));
            for (Int32 PassNo = Early; PassNo <= PassCount; PassNo += Rest)
            {
                EmitAt.Add(PassNo);
            }

            EmitAt.Add(PassCount);
            return EmitAt;
        }

        /// <summary>
        /// Calculates total cut bcy.
        /// </summary>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting total cut bcy.</returns>

        private static Double ComputeTotalCutBcy
            (
            Grid Grid
            )
        {
            Double[,] Delta = Grid.Delta();
            Double Total = 0.0;
            Int32 Height = Delta.GetLength(0);
            Int32 Width = Delta.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx]) continue;
                    Double Value = Delta[By, Bx];
                    if (Value > 0.0)
                    {
                        Total += Value;
                    }
                }
            }

            return Total * Grid.AreaM2 * Constants.M3_TO_CY;
        }

        /// <summary>
        /// Executes the apply pass operation.
        /// </summary>
        /// <param name="Existing">The existing used by the operation.</param>
        /// <param name="Pass">The pass used by the operation.</param>
        /// <param name="AreaM2">The area m2 used by the operation.</param>
        /// <param name="MovedBcy">The moved bcy used by the operation.</param>

        private static void ApplyPass
            (
            Double[,] Existing,
            Pass Pass,
            Double AreaM2,
            ref Double MovedBcy
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.CutCells)
            {
                Existing[Entry.Cell.By, Entry.Cell.Bx] -= Entry.DepthM;
                MovedBcy += Entry.DepthM * AreaM2 * Constants.M3_TO_CY;
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.FillCells)
            {
                Existing[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
            }
        }

        /// <summary>
        /// Calculates on grade fraction.
        /// </summary>
        /// <param name="Existing">The existing used by the operation.</param>
        /// <param name="Target">The target used by the operation.</param>
        /// <param name="Valid">The valid used by the operation.</param>
        /// <param name="ToleranceM">The tolerance m used by the operation.</param>
        /// <returns>The resulting on grade fraction.</returns>

        private static Double ComputeOnGradeFraction
            (
            Double[,] Existing,
            Double[,] Target,
            Boolean[,] Valid,
            Double ToleranceM
            )
        {
            Int32 OnGrade = 0;
            Int32 Count = 0;
            Int32 Height = Existing.GetLength(0);
            Int32 Width = Existing.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Valid[By, Bx]) continue;
                    Count += 1;
                    if (Math.Abs(Existing[By, Bx] - Target[By, Bx]) <= ToleranceM)
                    {
                        OnGrade += 1;
                    }
                }
            }

            return Count > 0 ? ((Double)OnGrade / Count) : 0.0;
        }

        /// <summary>
        /// Builds split frame.
        /// </summary>
        /// <param name="Existing">The existing used by the operation.</param>
        /// <param name="Target">The target used by the operation.</param>
        /// <param name="Valid">The valid used by the operation.</param>
        /// <param name="Ppb">The ppb used by the operation.</param>
        /// <param name="Vmin">The vmin used by the operation.</param>
        /// <param name="Vmax">The vmax used by the operation.</param>
        /// <param name="Opt">The opt used by the operation.</param>
        /// <param name="PanelW">The panel w used by the operation.</param>
        /// <param name="PanelH">The panel h used by the operation.</param>
        /// <param name="CanvasW">The canvas w used by the operation.</param>
        /// <param name="CanvasH">The canvas h used by the operation.</param>
        /// <param name="PassDone">The pass done used by the operation.</param>
        /// <param name="PassCount">Count limit applied by the operation.</param>
        /// <param name="MovedBcy">The moved bcy used by the operation.</param>
        /// <param name="TotalCutBcy">The total cut bcy used by the operation.</param>
        /// <param name="ToleranceM">The tolerance m used by the operation.</param>
        /// <returns>The resulting split frame.</returns>

        private static Image<Rgba32> BuildSplitFrame
            (
            Double[,] Existing,
            Double[,] Target,
            Boolean[,] Valid,
            Int32 Ppb,
            Double Vmin,
            Double Vmax,
            SplitLevelingOptions Opt,
            Int32 PanelW,
            Int32 PanelH,
            Int32 CanvasW,
            Int32 CanvasH,
            Int32 PassDone,
            Int32 PassCount,
            Double MovedBcy,
            Double TotalCutBcy,
            Double ToleranceM
            )
        {
            Int32 Height = Existing.GetLength(0);
            Int32 Width = Existing.GetLength(1);
            Double[,] Diff = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Diff[By, Bx] = Existing[By, Bx] - Target[By, Bx];
                }
            }

            using Image<Rgba32> CutFill = Render.DiffToImageDigitize(Diff, Valid, Ppb);
            using Image<Rgba32> Topo = Render.ElevationToImage(Existing, Valid, Ppb, Vmin, Vmax);
            Image<Rgba32> Canvas = new Image<Rgba32>(CanvasW, CanvasH);
            Canvas.Mutate(Context => Context.BackgroundColor(new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL)));
            Canvas.Mutate(Context => Context.DrawImage(CutFill, new Point(Opt.Gap, Opt.HeaderHeight), 1.0f));
            Canvas.Mutate(Context => Context.DrawImage(Topo, new Point((Opt.Gap * 2) + PanelW, Opt.HeaderHeight), 1.0f));

            Double Progress = PassCount > 0 ? ((Double)PassDone / PassCount) : 1.0;
            Int32 BarY = Opt.HeaderHeight - 16;
            Int32 BarMaxW = CanvasW - (Opt.Gap * 2);
            Int32 BarW = Math.Max(0, (Int32)Math.Round(BarMaxW * Progress, MidpointRounding.AwayFromZero));
            FillRect(Canvas, Opt.Gap, BarY, BarMaxW, 8, new Rgba32(230, 230, 230, Constants.MAX_COLOR_CHANNEL));
            FillRect(Canvas, Opt.Gap, BarY, BarW, 8, new Rgba32(40, 120, 40, Constants.MAX_COLOR_CHANNEL));

            DrawCaptionBar(Canvas, Opt.Gap, Opt.HeaderHeight + 2, PanelW, "CUT / FILL");
            DrawCaptionBar(Canvas, (Opt.Gap * 2) + PanelW, Opt.HeaderHeight + 2, PanelW, "TOPOLOGY");

            Double OnGradePct = ComputeOnGradeFraction(Existing, Target, Valid, ToleranceM) * 100.0;
            String Stats = String.Format(
                CultureInfo.InvariantCulture,
                "pass {0}/{1}  on-grade {2:F1}%  moved {3:F0}/{4:F0} BCY",
                PassDone,
                PassCount,
                OnGradePct,
                MovedBcy,
                TotalCutBcy);
            DrawStatsLine(Canvas, Opt.Gap, 8, Stats);

            return Canvas;
        }

        /// <summary>
        /// Renders caption bar.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Width">The width used by the operation.</param>
        /// <param name="Label">The label used by the operation.</param>

        private static void DrawCaptionBar
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Int32 Width,
            String Label
            )
        {
            FillRect(Image, X, Y, Width, 18, new Rgba32(20, 20, 20, 170));
            DrawSimpleText(Image, X + 6, Y + 6, Label, new Rgba32(Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL, Constants.MAX_COLOR_CHANNEL));
        }

        /// <summary>
        /// Renders stats line.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Text">The text used by the operation.</param>

        private static void DrawStatsLine
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            String Text
            )
        {
            DrawSimpleText(Image, X, Y, Text, new Rgba32(20, 20, 20, Constants.MAX_COLOR_CHANNEL));
        }

        /// <summary>
        /// Renders simple text.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Text">The text used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void DrawSimpleText
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            String Text,
            Rgba32 Color
            )
        {
            Int32 CursorX = X;
            foreach (Char Ch in Text)
            {
                if (Ch == ' ')
                {
                    CursorX += 4;
                    continue;
                }

                DrawMiniGlyph(Image, Ch, CursorX, Y, Color);
                CursorX += 6;
            }
        }

        /// <summary>
        /// Renders mini glyph.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="Ch">The ch used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void DrawMiniGlyph
            (
            Image<Rgba32> Image,
            Char Ch,
            Int32 X,
            Int32 Y,
            Rgba32 Color
            )
        {
            String Pattern = Ch switch
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
                    PlotPixel(Image, X + Col, Y + Row, Color);
                }
            }
        }

        /// <summary>
        /// Executes the fill rect operation.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Width">The width used by the operation.</param>
        /// <param name="Height">The height used by the operation.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void FillRect
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Int32 Width,
            Int32 Height,
            Rgba32 Color
            )
        {
            Int32 X1 = Math.Min(Image.Width, X + Width);
            Int32 Y1 = Math.Min(Image.Height, Y + Height);
            for (Int32 Py = Math.Max(0, Y); Py < Y1; Py += 1)
            {
                for (Int32 Px = Math.Max(0, X); Px < X1; Px += 1)
                {
                    Image[Px, Py] = Color;
                }
            }
        }

        /// <summary>
        /// Renders pixel.
        /// </summary>
        /// <param name="Image">The image used by the operation.</param>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Color">The color used by the operation.</param>

        private static void PlotPixel
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Rgba32 Color
            )
        {
            if ((X < 0) || (Y < 0) || (X >= Image.Width) || (Y >= Image.Height))
            {
                return;
            }

            Image[X, Y] = Color;
        }

        /// <summary>
        /// Executes the to pivot grid operation.
        /// </summary>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting to pivot grid.</returns>

        private static Pivot.Grid ToPivotGrid
            (
            Grid Grid
            )
        {
            // Adapt top-level grid model for Pivot module APIs.
            return new Pivot.Grid(Grid.GridSizeM, Grid.MinX, Grid.MinY, Grid.Zone, Grid.IsNorthern, Grid.Existing, Grid.Target, Grid.Valid);
        }

        /// <summary>
        /// Executes the to pivot passes operation.
        /// </summary>
        /// <param name="Passes">The passes used by the operation.</param>
        /// <returns>The resulting to pivot passes.</returns>

        private static List<Pivot.Pass> ToPivotPasses
            (
            IReadOnlyList<Pass> Passes
            )
        {
            // Convert pass tuple payloads into Pivot pass DTOs.
            List<Pivot.Pass> Out = new List<Pivot.Pass>();
            for (Int32 Iter = 0; Iter < Passes.Count; Iter += 1)
            {
                List<Pivot.PassCell> Cut = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in Passes[Iter].CutCells)
                {
                    Cut.Add(new Pivot.PassCell(Cell.Cell.Bx, Cell.Cell.By, Cell.DepthM));
                }

                List<Pivot.PassCell> Fill = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in Passes[Iter].FillCells)
                {
                    Fill.Add(new Pivot.PassCell(Cell.Cell.Bx, Cell.Cell.By, Cell.DepthM));
                }

                Out.Add(new Pivot.Pass(Cut, Fill, Passes[Iter].CutEndXy, Passes[Iter].FillStartXy));
            }

            return Out;
        }

        /// <summary>
        /// Calculates range.
        /// </summary>
        /// <param name="Existing">The existing used by the operation.</param>
        /// <param name="Target">The target used by the operation.</param>
        /// <param name="Valid">The valid used by the operation.</param>
        /// <param name="Mode">The mode used by the operation.</param>
        /// <returns>The resulting range.</returns>

        private static (Double VMin, Double VMax) ComputeRange
            (
            Double[,] Existing,
            Double[,] Target,
            Boolean[,] Valid,
            String Mode
            )
        {
            // Topology mode needs a fixed range across all frames; cutfill mode ignores this range.
            if (!String.Equals(Mode, "topology", StringComparison.OrdinalIgnoreCase))
            {
                return (0.0, 0.0);
            }

            Double Min = Double.PositiveInfinity;
            Double Max = Double.NegativeInfinity;
            Int32 Height = Existing.GetLength(0);
            Int32 Width = Existing.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Valid[By, Bx]) continue;
                    Min = Math.Min(Min, Math.Min(Existing[By, Bx], Target[By, Bx]));
                    Max = Math.Max(Max, Math.Max(Existing[By, Bx], Target[By, Bx]));
                }
            }

            if (!Double.IsFinite(Min) || !Double.IsFinite(Max))
            {
                return (0.0, 1.0);
            }

            return (Min, Max);
        }

        /// <summary>
        /// Builds frame.
        /// </summary>
        /// <param name="Existing">The existing used by the operation.</param>
        /// <param name="Target">The target used by the operation.</param>
        /// <param name="Valid">The valid used by the operation.</param>
        /// <param name="Ppb">The ppb used by the operation.</param>
        /// <param name="Mode">The mode used by the operation.</param>
        /// <param name="Vmin">The vmin used by the operation.</param>
        /// <param name="Vmax">The vmax used by the operation.</param>
        /// <returns>The resulting frame.</returns>

        private static Image<Rgba32> BuildFrame
            (
            Double[,] Existing,
            Double[,] Target,
            Boolean[,] Valid,
            Int32 Ppb,
            String Mode,
            Double Vmin,
            Double Vmax
            )
        {
            // Render module owns the exact palette and coordinate conventions.
            if (String.Equals(Mode, "topology", StringComparison.OrdinalIgnoreCase))
            {
                return Render.ElevationToImage(Existing, Valid, Ppb, Vmin, Vmax);
            }

            Int32 Height = Existing.GetLength(0);
            Int32 Width = Existing.GetLength(1);
            Double[,] Diff = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Diff[By, Bx] = (Existing[By, Bx] - Target[By, Bx]);
                }
            }

            return Render.DiffToImage(Diff, Valid, Ppb);
        }

        /// <summary>
        /// Executes the clone operation.
        /// </summary>
        /// <param name="Src">The src used by the operation.</param>
        /// <returns>The resulting clone.</returns>

        private static Double[,] Clone
            (
            Double[,] Src
            )
        {
            // Explicit deep copy prevents accidental aliasing between pipeline stages.
            Int32 Height = Src.GetLength(0);
            Int32 Width = Src.GetLength(1);
            Double[,] Dst = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Dst[By, Bx] = Src[By, Bx];
                }
            }

            return Dst;
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
            // Memory-stream PNG encoding keeps animation consumers file-agnostic.
            using MemoryStream Stream = new MemoryStream();
            Image.SaveAsPng(Stream);
            return Stream.ToArray();
        }

        /// <summary>
        /// Builds gif.
        /// </summary>
        /// <param name="Frames">The frames used by the operation.</param>
        /// <param name="GifMs">The gif ms used by the operation.</param>
        /// <param name="ProgressDurationMs">The progress duration ms used by the operation.</param>
        /// <param name="FinalHoldMs">The final hold ms used by the operation.</param>
        /// <returns>The resulting gif.</returns>

        private static Byte[] BuildGif
            (
            IReadOnlyList<Byte[]> Frames,
            Int32 GifMs,
            Int32? ProgressDurationMs = null,
            Int32 FinalHoldMs = 4000
            )
        {
            // Last-frame hold mirrors Python UX so the completed grade remains visible.
            IReadOnlyList<Int32> DelaysCs = ComputeGifFrameDelaysCs(
                Frames.Count,
                GifMs,
                ProgressDurationMs,
                FinalHoldMs);
            using Image<Rgba32> First = Image.Load<Rgba32>(Frames[0]);
            SetGifDelayCentiseconds(First.Frames.RootFrame, DelaysCs[0]);
            for (Int32 Iter = 1; Iter < Frames.Count; Iter += 1)
            {
                using Image<Rgba32> Next = Image.Load<Rgba32>(Frames[Iter]);
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
            Int32 FrameCount,
            Int32 GifMs,
            Int32? ProgressDurationMs,
            Int32 FinalHoldMs
            )
        {
            if (FrameCount <= 0)
            {
                return Array.Empty<Int32>();
            }

            Int32[] DelaysCs = new Int32[FrameCount];
            Int32 ProgressFrameCount = FrameCount - 1;
            if (ProgressFrameCount > 0)
            {
                if (ProgressDurationMs.HasValue)
                {
                    Int32 TotalProgressCs = Math.Max(
                        ProgressFrameCount,
                        (Int32)Math.Round(
                            ProgressDurationMs.Value / 10.0,
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
                            GifMs / 10.0,
                            MidpointRounding.AwayFromZero));
                    Array.Fill(
                        DelaysCs,
                        UniformDelayCs,
                        0,
                        ProgressFrameCount);
                }
            }

            DelaysCs[FrameCount - 1] = Math.Max(
                1,
                (Int32)Math.Round(
                    FinalHoldMs / 10.0,
                    MidpointRounding.AwayFromZero));
            return DelaysCs;
        }

        /// <summary>
        /// Executes the set gif delay centiseconds operation.
        /// </summary>
        /// <param name="Frame">The frame used by the operation.</param>
        /// <param name="DelayCs">The delay cs used by the operation.</param>

        private static void SetGifDelayCentiseconds
            (
            ImageFrame<Rgba32> Frame,
            Int32 DelayCs
            )
        {
            Frame.Metadata.GetGifMetadata().FrameDelay = Math.Max(1, DelayCs);
        }
    }
}
