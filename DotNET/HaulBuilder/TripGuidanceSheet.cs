using System;
using System.Collections.Generic;

using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;

namespace HaulBuilder
{
    /// <summary>
    /// Draws one trip the way the guidance is driven. Lanes are blade-wide bands.
    /// Connectors are the arcs and straights themselves, one arrowhead each.
    /// Nothing else is drawn: a loop appears only when the path contains one.
    /// </summary>
    static public class TripGuidanceSheet
    {
        /// <summary>How far the field colour is pulled toward white so the path reads.</summary>
        private const Double DIM_TOWARD_WHITE = 0.45;

        /// <summary>Dash length for a turnaround, metres.</summary>
        private const Double DASH_M = 2.0;

        /// <summary>Off-field grey, matching the other field sheets.</summary>
        private const Byte OFF_FIELD = 238;

        /// <summary>
        /// Writes the trip over the ground it is about to touch.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Surface">Existing-minus-target at the start of the trip.</param>
        /// <param name="Trip">The trip to draw.</param>
        /// <param name="Path">PNG path.</param>
        /// <param name="BladeM">Blade width, metres.</param>
        /// <param name="PixelsPerCell">Pixels across one cell.</param>
        /// <param name="ShownStamp">Stamp from the plan header. Empty draws the trip's own stamp.</param>
        static public void Save
            (
            Grid Grid,
            Double[,] Surface,
            TripGuidance.TripCourse Trip,
            String Path,
            Double BladeM,
            Int32 PixelsPerCell,
            String ShownStamp = ""
            )
        {
            Image<Rgba32> Field = Render.DiffToImageDigitize(Surface, Grid.Valid, PixelsPerCell);
            Dim(Field);
            // A turnaround that leaves the mask still has to be on the sheet.
            Image<Rgba32> Image = CanvasForPath(Field, Grid, Trip, PixelsPerCell, out Int32 OriginX, out Int32 OriginY);
            if (!Object.ReferenceEquals(Image, Field))
            {
                Field.Dispose();
            }

            using (Image)
            {
                Int32 Band = Math.Max(2, (Int32)Math.Round((BladeM * Constants.HALF_SCALE / Grid.GridSizeM) * PixelsPerCell));
                foreach (TripGuidance.CourseSegment Segment in Trip.Segments)
                {
                    if (!Segment.IsLane)
                    {
                        continue;
                    }

                    if (Segment.Action == TripGuidance.GuidanceAction.Cut)
                    {
                        DrawSegment(Image, Grid, PixelsPerCell, OriginX, OriginY, Segment, new Rgba32(220, 30, 30), Band, false);
                    }
                    else if (Segment.Action == TripGuidance.GuidanceAction.Fill)
                    {
                        DrawSegment(Image, Grid, PixelsPerCell, OriginX, OriginY, Segment, new Rgba32(30, 90, 220), Band, false);
                    }
                }

                foreach (TripGuidance.CourseSegment Segment in Trip.Segments)
                {
                    if (Segment.IsLane || (Segment.Action == TripGuidance.GuidanceAction.Grade))
                    {
                        continue;
                    }

                    Rgba32 Color = ColorOf(Segment);
                    Boolean Dashed = Segment.Action == TripGuidance.GuidanceAction.Turn;
                    DrawSegment(Image, Grid, PixelsPerCell, OriginX, OriginY, Segment, Color, 1, Dashed);
                }

                // The grade line sits on the fill band, thin and white.
                foreach (TripGuidance.CourseSegment Segment in Trip.Segments)
                {
                    if (Segment.Action != TripGuidance.GuidanceAction.Grade)
                    {
                        continue;
                    }

                    DrawSegment(Image, Grid, PixelsPerCell, OriginX, OriginY, Segment, new Rgba32(255, 255, 255), 1, false);
                }

                foreach (TripGuidance.CourseSegment Segment in Trip.Segments)
                {
                    DrawArrows(Image, Grid, PixelsPerCell, OriginX, OriginY, Segment);
                }

                CropToPath(Image, Grid, PixelsPerCell, OriginX, OriginY, Trip);
                Boolean Mismatch = (ShownStamp != null)
                    && (ShownStamp.Length > 0)
                    && !String.Equals(ShownStamp, Trip.PlanStamp, StringComparison.Ordinal);
                DrawLabel(Image, Trip.TripNumber, Trip.ExecutionIndex, Trip.PlanStamp, Mismatch);
                Image.SaveAsPng(Path);
            }
        }

        /// <summary>
        /// Gives the path room past the field mask. The field image is copied
        /// onto a grey sheet when a sample would otherwise fall off the bitmap.
        /// </summary>
        /// <param name="Field">Dimmed field image. Disposed by the caller when a new sheet is returned.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Trip">Trip whose samples set the canvas.</param>
        /// <param name="PixelsPerCell">Pixels across one cell.</param>
        /// <param name="OriginX">Added to a field pixel to land on the sheet.</param>
        /// <param name="OriginY">Added to a field pixel to land on the sheet.</param>
        /// <returns>The sheet to draw on. It may be <paramref name="Field"/> itself.</returns>
        private static Image<Rgba32> CanvasForPath
            (
            Image<Rgba32> Field,
            Grid Grid,
            TripGuidance.TripCourse Trip,
            Int32 PixelsPerCell,
            out Int32 OriginX,
            out Int32 OriginY
            )
        {
            OriginX = 0;
            OriginY = 0;
            if (!PathBounds(Grid, PixelsPerCell, Trip, out Int32 MinX, out Int32 MinY, out Int32 MaxX, out Int32 MaxY))
            {
                return Field;
            }

            // Eight cells around the path, including the part that left the mask.
            Int32 Pad = 8 * PixelsPerCell;
            OriginX = Math.Max(0, Pad - MinX);
            OriginY = Math.Max(0, Pad - MinY);
            Int32 Width = Math.Max(Field.Width + OriginX, MaxX + OriginX + Pad + 1);
            Int32 Height = Math.Max(Field.Height + OriginY, MaxY + OriginY + Pad + 1);
            if ((OriginX == 0) && (OriginY == 0) && (Width == Field.Width) && (Height == Field.Height))
            {
                return Field;
            }

            Image<Rgba32> Sheet = new Image<Rgba32>(Width, Height, new Rgba32(OFF_FIELD, OFF_FIELD, OFF_FIELD));
            // The draw lambda cannot capture the out parameters.
            Int32 PasteX = OriginX;
            Int32 PasteY = OriginY;
            Sheet.Mutate(Context => Context.DrawImage(Field, new Point(PasteX, PasteY), 1.0f));
            return Sheet;
        }

        /// <summary>
        /// Pixel bounds of every sample, in field-image coordinates.
        /// Those coordinates may fall outside the field bitmap.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="PixelsPerCell">Pixels across one cell.</param>
        /// <param name="Trip">Trip to measure.</param>
        /// <param name="MinX">Leftmost sample.</param>
        /// <param name="MinY">Topmost sample.</param>
        /// <param name="MaxX">Rightmost sample.</param>
        /// <param name="MaxY">Bottommost sample.</param>
        /// <returns>False when the trip has no samples.</returns>
        private static Boolean PathBounds
            (
            Grid Grid,
            Int32 PixelsPerCell,
            TripGuidance.TripCourse Trip,
            out Int32 MinX,
            out Int32 MinY,
            out Int32 MaxX,
            out Int32 MaxY
            )
        {
            MinX = 0;
            MinY = 0;
            MaxX = 0;
            MaxY = 0;
            Boolean Any = false;
            foreach (TripGuidance.CourseSegment Segment in Trip.Segments)
            {
                foreach (TripGuidance.DrawnPiece Piece in Segment.Pieces)
                {
                    foreach (DubinsConnector.Pose Sample in Piece.Samples)
                    {
                        (Int32 X, Int32 Y) At = Pixel(Grid, PixelsPerCell, 0, 0, Sample.X, Sample.Y);
                        if (!Any)
                        {
                            MinX = At.X;
                            MinY = At.Y;
                            MaxX = At.X;
                            MaxY = At.Y;
                            Any = true;
                            continue;
                        }

                        if (At.X < MinX)
                        {
                            MinX = At.X;
                        }

                        if (At.Y < MinY)
                        {
                            MinY = At.Y;
                        }

                        if (At.X > MaxX)
                        {
                            MaxX = At.X;
                        }

                        if (At.Y > MaxY)
                        {
                            MaxY = At.Y;
                        }
                    }
                }
            }

            return Any;
        }

        /// <summary>
        /// Crops the sheet to the path, with a margin of field around it.
        /// </summary>
        /// <param name="Image">Sheet the path was drawn on.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="PixelsPerCell">Pixels across one cell.</param>
        /// <param name="OriginX">Added to a field pixel to land on the sheet.</param>
        /// <param name="OriginY">Added to a field pixel to land on the sheet.</param>
        /// <param name="Trip">Trip whose samples set the window.</param>
        private static void CropToPath
            (
            Image<Rgba32> Image,
            Grid Grid,
            Int32 PixelsPerCell,
            Int32 OriginX,
            Int32 OriginY,
            TripGuidance.TripCourse Trip
            )
        {
            Int32 MinX = Image.Width;
            Int32 MinY = Image.Height;
            Int32 MaxX = 0;
            Int32 MaxY = 0;
            Boolean Any = false;
            foreach (TripGuidance.CourseSegment Segment in Trip.Segments)
            {
                foreach (TripGuidance.DrawnPiece Piece in Segment.Pieces)
                {
                    foreach (DubinsConnector.Pose Sample in Piece.Samples)
                    {
                        (Int32 X, Int32 Y) At = Pixel(Grid, PixelsPerCell, OriginX, OriginY, Sample.X, Sample.Y);
                        Any = true;
                        if (At.X < MinX)
                        {
                            MinX = At.X;
                        }

                        if (At.Y < MinY)
                        {
                            MinY = At.Y;
                        }

                        if (At.X > MaxX)
                        {
                            MaxX = At.X;
                        }

                        if (At.Y > MaxY)
                        {
                            MaxY = At.Y;
                        }
                    }
                }
            }

            if (!Any)
            {
                return;
            }

            // Eight cells of ground around the path, so the cut and the fill stay in frame.
            Int32 Pad = 8 * PixelsPerCell;
            MinX = Math.Max(0, MinX - Pad);
            MinY = Math.Max(0, MinY - Pad);
            MaxX = Math.Min(Image.Width, MaxX + Pad);
            MaxY = Math.Min(Image.Height, MaxY + Pad);
            if ((MaxX - MinX) < 2 || (MaxY - MinY) < 2)
            {
                return;
            }

            Image.Mutate(Context => Context.Crop(new Rectangle(MinX, MinY, MaxX - MinX, MaxY - MinY)));
        }

        /// <summary>
        /// Pulls field colours toward white. Off-field cells become the sheet grey.
        /// </summary>
        /// <param name="Image">Field image.</param>
        private static void Dim
            (
            Image<Rgba32> Image
            )
        {
            Double Kept = 1.0 - DIM_TOWARD_WHITE;
            for (Int32 Y = 0; Y < Image.Height; Y += 1)
            {
                for (Int32 X = 0; X < Image.Width; X += 1)
                {
                    Rgba32 Pixel = Image[X, Y];
                    if (Pixel.A == 0)
                    {
                        Image[X, Y] = new Rgba32(OFF_FIELD, OFF_FIELD, OFF_FIELD);
                        continue;
                    }

                    Byte Red = (Byte)Math.Round((Pixel.R * Kept) + (255.0 * DIM_TOWARD_WHITE));
                    Byte Green = (Byte)Math.Round((Pixel.G * Kept) + (255.0 * DIM_TOWARD_WHITE));
                    Byte Blue = (Byte)Math.Round((Pixel.B * Kept) + (255.0 * DIM_TOWARD_WHITE));
                    Image[X, Y] = new Rgba32(Red, Green, Blue);
                }
            }
        }

        /// <summary>
        /// Colour of a connector. A path that left the field is magenta.
        /// </summary>
        /// <param name="Segment">Connector segment.</param>
        /// <returns>The stroke colour.</returns>
        private static Rgba32 ColorOf
            (
            TripGuidance.CourseSegment Segment
            )
        {
            if (Segment.Outside)
            {
                return new Rgba32(255, 0, 255);
            }

            if (Segment.Action == TripGuidance.GuidanceAction.Return)
            {
                return new Rgba32(255, 140, 0);
            }

            return new Rgba32(0, 0, 0);
        }

        /// <summary>
        /// Strokes every piece. A lane is a band. A connector is a line.
        /// </summary>
        /// <param name="Image">Sheet to draw on.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="PixelsPerCell">Pixels across one cell.</param>
        /// <param name="OriginX">Added to a field pixel to land on the sheet.</param>
        /// <param name="OriginY">Added to a field pixel to land on the sheet.</param>
        /// <param name="Segment">Lane or connector.</param>
        /// <param name="Color">Stroke colour.</param>
        /// <param name="Thickness">Stamp radius in pixels.</param>
        /// <param name="Dashed">True for a turnaround.</param>
        private static void DrawSegment
            (
            Image<Rgba32> Image,
            Grid Grid,
            Int32 PixelsPerCell,
            Int32 OriginX,
            Int32 OriginY,
            TripGuidance.CourseSegment Segment,
            Rgba32 Color,
            Int32 Thickness,
            Boolean Dashed
            )
        {
            foreach (TripGuidance.DrawnPiece Piece in Segment.Pieces)
            {
                Double Covered = 0.0;
                for (Int32 Index = 1; Index < Piece.Samples.Count; Index += 1)
                {
                    DubinsConnector.Pose From = Piece.Samples[Index - 1];
                    DubinsConnector.Pose To = Piece.Samples[Index];
                    Double Dx = To.X - From.X;
                    Double Dy = To.Y - From.Y;
                    Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                    Boolean Draw = true;
                    if (Dashed)
                    {
                        // On and off alternate every DASH_M so the turnaround reads as dashed.
                        Double Mid = Covered + (Length * Constants.HALF_SCALE);
                        Draw = (((Int32)Math.Floor(Mid / DASH_M)) % 2) == 0;
                    }

                    if (Draw && (Length > 0.0))
                    {
                        (Int32 X, Int32 Y) A = Pixel(Grid, PixelsPerCell, OriginX, OriginY, From.X, From.Y);
                        (Int32 X, Int32 Y) B = Pixel(Grid, PixelsPerCell, OriginX, OriginY, To.X, To.Y);
                        Stroke(Image, A.X, A.Y, B.X, B.Y, Color, Thickness);
                    }

                    Covered += Length;
                }
            }
        }

        /// <summary>
        /// One arrowhead on each piece that has one. The heading is the tangent, not the chord.
        /// </summary>
        /// <param name="Image">Sheet to draw on.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="PixelsPerCell">Pixels across one cell.</param>
        /// <param name="OriginX">Added to a field pixel to land on the sheet.</param>
        /// <param name="OriginY">Added to a field pixel to land on the sheet.</param>
        /// <param name="Segment">Lane or connector.</param>
        private static void DrawArrows
            (
            Image<Rgba32> Image,
            Grid Grid,
            Int32 PixelsPerCell,
            Int32 OriginX,
            Int32 OriginY,
            TripGuidance.CourseSegment Segment
            )
        {
            // The arrow has to sit on top of its own stroke, so it takes the opposite tone.
            Rgba32 Color = new Rgba32(255, 255, 255);
            if (Segment.Action == TripGuidance.GuidanceAction.Grade)
            {
                Color = new Rgba32(0, 0, 0);
            }
            else if (!Segment.IsLane && !Segment.Outside
                && (Segment.Action != TripGuidance.GuidanceAction.Return))
            {
                Color = new Rgba32(255, 255, 255);
            }
            else if (Segment.Action == TripGuidance.GuidanceAction.Return)
            {
                Color = new Rgba32(0, 0, 0);
            }
            else if (Segment.Outside)
            {
                Color = new Rgba32(255, 255, 255);
            }

            foreach (TripGuidance.DrawnPiece Piece in Segment.Pieces)
            {
                if (!Piece.Arrow.HasValue)
                {
                    continue;
                }

                DubinsConnector.Pose At = Piece.Arrow.Value;
                (Int32 X, Int32 Y) Tip = Pixel(Grid, PixelsPerCell, OriginX, OriginY, At.X, At.Y);
                // Pixel y grows south. World north is negative pixel y.
                Double ForwardX = Math.Cos(At.HeadingRad);
                Double ForwardY = -Math.Sin(At.HeadingRad);
                Double SideX = -ForwardY;
                Double SideY = ForwardX;
                Int32 Length = 14;
                Int32 Width = 6;
                (Int32 X, Int32 Y) Nose = (Tip.X + (Int32)Math.Round(ForwardX * Length), Tip.Y + (Int32)Math.Round(ForwardY * Length));
                (Int32 X, Int32 Y) Left = (
                    Tip.X + (Int32)Math.Round((-ForwardX * 2.0) + (SideX * Width)),
                    Tip.Y + (Int32)Math.Round((-ForwardY * 2.0) + (SideY * Width)));
                (Int32 X, Int32 Y) Right = (
                    Tip.X + (Int32)Math.Round((-ForwardX * 2.0) - (SideX * Width)),
                    Tip.Y + (Int32)Math.Round((-ForwardY * 2.0) - (SideY * Width)));
                FillTriangle(Image, Nose, Left, Right, Color);
            }
        }

        /// <summary>
        /// World metres to a pixel. North is up. The origin shifts the field
        /// onto a sheet that also holds path samples outside the mask.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="PixelsPerCell">Pixels across one cell.</param>
        /// <param name="OriginX">Added to the field pixel.</param>
        /// <param name="OriginY">Added to the field pixel.</param>
        /// <param name="X">Easting, metres.</param>
        /// <param name="Y">Northing, metres.</param>
        /// <returns>The pixel on the sheet.</returns>
        private static (Int32 X, Int32 Y) Pixel
            (
            Grid Grid,
            Int32 PixelsPerCell,
            Int32 OriginX,
            Int32 OriginY,
            Double X,
            Double Y
            )
        {
            Double East = (X - Grid.MinX) / Grid.GridSizeM;
            Double North = (Y - Grid.MinY) / Grid.GridSizeM;
            Int32 Px = (Int32)Math.Round(East * PixelsPerCell) + OriginX;
            Int32 Py = (Int32)Math.Round((Grid.Height - North) * PixelsPerCell) + OriginY;
            return (Px, Py);
        }

        /// <summary>
        /// Thick straight stroke in pixel space.
        /// </summary>
        private static void Stroke
            (
            Image<Rgba32> Image,
            Int32 X0,
            Int32 Y0,
            Int32 X1,
            Int32 Y1,
            Rgba32 Color,
            Int32 Thickness
            )
        {
            Int32 Steps = Math.Max(Math.Abs(X1 - X0), Math.Abs(Y1 - Y0));
            if (Steps < 1)
            {
                Stamp(Image, X0, Y0, Color, Thickness);
                return;
            }

            for (Int32 Step = 0; Step <= Steps; Step += 1)
            {
                Int32 X = X0 + ((X1 - X0) * Step / Steps);
                Int32 Y = Y0 + ((Y1 - Y0) * Step / Steps);
                Stamp(Image, X, Y, Color, Thickness);
            }
        }

        /// <summary>
        /// Fills a small disk. Used for the blade band and for line thickness.
        /// </summary>
        private static void Stamp
            (
            Image<Rgba32> Image,
            Int32 X,
            Int32 Y,
            Rgba32 Color,
            Int32 Radius
            )
        {
            Int32 Limit = Radius * Radius;
            for (Int32 Dy = -Radius; Dy <= Radius; Dy += 1)
            {
                for (Int32 Dx = -Radius; Dx <= Radius; Dx += 1)
                {
                    if (((Dx * Dx) + (Dy * Dy)) > Limit)
                    {
                        continue;
                    }

                    Plot(Image, X + Dx, Y + Dy, Color);
                }
            }
        }

        /// <summary>
        /// Fills a triangle. Arrowheads are small, so the box is walked directly.
        /// </summary>
        private static void FillTriangle
            (
            Image<Rgba32> Image,
            (Int32 X, Int32 Y) A,
            (Int32 X, Int32 Y) B,
            (Int32 X, Int32 Y) C,
            Rgba32 Color
            )
        {
            Int32 MinX = Math.Min(A.X, Math.Min(B.X, C.X));
            Int32 MaxX = Math.Max(A.X, Math.Max(B.X, C.X));
            Int32 MinY = Math.Min(A.Y, Math.Min(B.Y, C.Y));
            Int32 MaxY = Math.Max(A.Y, Math.Max(B.Y, C.Y));
            for (Int32 Y = MinY; Y <= MaxY; Y += 1)
            {
                for (Int32 X = MinX; X <= MaxX; X += 1)
                {
                    if (Inside(X, Y, A, B, C))
                    {
                        Plot(Image, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Barycentric inside test. The edge counts as inside so the tip is not lost.
        /// </summary>
        private static Boolean Inside
            (
            Int32 X,
            Int32 Y,
            (Int32 X, Int32 Y) A,
            (Int32 X, Int32 Y) B,
            (Int32 X, Int32 Y) C
            )
        {
            // Same side of every edge. A point on an edge has a zero and still counts.
            (Int32 X, Int32 Y) Point = (X, Y);
            Int32 First = Edge(Point, A, B);
            Int32 Second = Edge(Point, B, C);
            Int32 Third = Edge(Point, C, A);
            Boolean Negative = (First < 0) || (Second < 0) || (Third < 0);
            Boolean Positive = (First > 0) || (Second > 0) || (Third > 0);
            return !(Negative && Positive);
        }

        /// <summary>
        /// Twice the signed area of a triangle.
        /// </summary>
        private static Int32 Edge
            (
            (Int32 X, Int32 Y) A,
            (Int32 X, Int32 Y) B,
            (Int32 X, Int32 Y) C
            )
        {
            return ((B.X - A.X) * (C.Y - A.Y)) - ((B.Y - A.Y) * (C.X - A.X));
        }

        /// <summary>
        /// Paints the trip number, the drive position, and the plan stamp.
        /// A red label means the header stamp does not match this trip.
        /// </summary>
        /// <param name="Image">The sheet.</param>
        /// <param name="TripNumber">Original 1-based trip number.</param>
        /// <param name="ExecutionIndex">1-based position in drive order. Zero prints the trip number only.</param>
        /// <param name="PlanStamp">Stamp copied from the saved plan. Empty draws none.</param>
        /// <param name="Mismatch">True when the header stamp is not <paramref name="PlanStamp"/>.</param>
        private static void DrawLabel
            (
            Image<Rgba32> Image,
            Int32 TripNumber,
            Int32 ExecutionIndex,
            String PlanStamp,
            Boolean Mismatch
            )
        {
            String Text = (ExecutionIndex > 0)
                ? String.Format(System.Globalization.CultureInfo.InvariantCulture, "{0}/{1}", TripNumber, ExecutionIndex)
                : TripNumber.ToString(System.Globalization.CultureInfo.InvariantCulture);
            if ((PlanStamp != null) && (PlanStamp.Length > 0))
            {
                Text = String.Format(
                    System.Globalization.CultureInfo.InvariantCulture,
                    "{0}/{1}",
                    Text,
                    PlanStamp);
            }

            const Int32 Scale = 2;
            const Int32 Pad = 3;
            Int32 Width = (Text.Length * 6 * Scale) + (Pad * 2);
            Int32 Height = (7 * Scale) + (Pad * 2);
            if (Width > Image.Width)
            {
                Width = Image.Width;
            }

            if (Height > Image.Height)
            {
                Height = Image.Height;
            }

            // A red label is the warning that the header stamp is not this trip's plan.
            Rgba32 Paper = Mismatch ? new Rgba32(255, 80, 80) : new Rgba32(255, 255, 255);
            for (Int32 Y = 0; Y < Height; Y += 1)
            {
                for (Int32 X = 0; X < Width; X += 1)
                {
                    Image[X, Y] = Paper;
                }
            }

            Int32 Cursor = Pad;
            foreach (Char GlyphChar in Text)
            {
                String Rows = GlyphRows(GlyphChar);
                String[] Split = Rows.Split(';');
                for (Int32 Row = 0; Row < Split.Length; Row += 1)
                {
                    for (Int32 Column = 0; Column < Split[Row].Length; Column += 1)
                    {
                        if (Split[Row][Column] != '1')
                        {
                            continue;
                        }

                        for (Int32 Y = 0; Y < Scale; Y += 1)
                        {
                            for (Int32 X = 0; X < Scale; X += 1)
                            {
                                Int32 PixelX = Cursor + (Column * Scale) + X;
                                Int32 PixelY = Pad + (Row * Scale) + Y;
                                if ((PixelX >= 0) && (PixelX < Image.Width) && (PixelY >= 0) && (PixelY < Image.Height))
                                {
                                    Image[PixelX, PixelY] = new Rgba32(0, 0, 0);
                                }
                            }
                        }
                    }
                }

                Cursor += 6 * Scale;
            }
        }

        /// <summary>
        /// Seven rows of a 5-wide glyph, ones for ink, separated by semicolons.
        /// </summary>
        /// <param name="GlyphChar">A digit or a slash.</param>
        /// <returns>The rows.</returns>
        private static String GlyphRows
            (
            Char GlyphChar
            )
        {
            switch (GlyphChar)
            {
                case '0':
                    return "11111;10001;10001;10001;10001;10001;11111";
                case '1':
                    return "00100;01100;00100;00100;00100;00100;01110";
                case '2':
                    return "11111;00001;00001;11111;10000;10000;11111";
                case '3':
                    return "11111;00001;00001;11111;00001;00001;11111";
                case '4':
                    return "10001;10001;10001;11111;00001;00001;00001";
                case '5':
                    return "11111;10000;10000;11111;00001;00001;11111";
                case '6':
                    return "11111;10000;10000;11111;10001;10001;11111";
                case '7':
                    return "11111;00001;00001;00010;00100;00100;00100";
                case '8':
                    return "11111;10001;10001;11111;10001;10001;11111";
                case '9':
                    return "11111;10001;10001;11111;00001;00001;11111";
                case '/':
                    return "00001;00010;00010;00100;01000;01000;10000";
                default:
                    return "00000;00000;00000;00000;00000;00000;00000";
            }
        }

        /// <summary>
        /// Writes one pixel when it is on the image.
        /// </summary>
        private static void Plot
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
    }
}
