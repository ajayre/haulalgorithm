using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Describes the outer field boundary, edge-parallel work headings, and the permitted one-cell exterior strip.
    /// </summary>
    public sealed class FieldEdgePolicy
    {
        private const Double NUMERIC_TOLERANCE = 1e-9;
        private readonly Grid Field;
        private readonly Boolean[,] ExteriorInvalid;
        private readonly Double HaulGridSizeM;

        /// <summary>Builds one immutable boundary classification for a field grid.</summary>
        public FieldEdgePolicy
            (
            Grid Field,
            Double HaulGridSizeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0
            )
        {
            this.Field = Field ?? throw new ArgumentNullException(nameof(Field));
            if (HaulGridSizeM <= 0.0) throw new ArgumentOutOfRangeException(nameof(HaulGridSizeM));
            this.HaulGridSizeM = HaulGridSizeM;
            ExteriorInvalid = BuildExteriorInvalid(Field);
        }

        /// <summary>Gets the maximum permitted centerline excursion beyond the outer field edge.</summary>
        public Double ExteriorAllowanceM => HaulGridSizeM;

        /// <summary>
        /// Returns true when a valid work cell lies within one cell width of the outer field boundary.
        /// </summary>
        public Boolean IsEdgeWorkCell
            (
            (Int32 Bx, Int32 By) Cell
            )
        {
            if (!IsValidCell(Cell.Bx, Cell.By)) return false;
            return DistanceFromCellCenterToExteriorM(Cell)
                <= HaulGridSizeM + NUMERIC_TOLERANCE;
        }

        /// <summary>Returns true when an in-field world point occupies an edge-adjacent valid cell.</summary>
        public Boolean IsFieldPointNearOuterEdge
            (
            Double X,
            Double Y
            )
        {
            if (!Field.ContainsWorldXy(X, Y)) return false;
            return IsEdgeWorkCell((
                (Int32)Math.Floor((X - Field.MinX) / Field.GridSizeM),
                (Int32)Math.Floor((Y - Field.MinY) / Field.GridSizeM)));
        }

        /// <summary>
        /// Estimates the local outer-edge tangent for an edge-adjacent valid cell.
        /// </summary>
        public Boolean TryGetParallelHeadingDeg
            (
            (Int32 Bx, Int32 By) Cell,
            out Double HeadingDeg
            )
        {
            HeadingDeg = 0.0;
            if (!IsEdgeWorkCell(Cell)) return false;

            Double CenterX = Field.MinX + ((Cell.Bx + 0.5) * Field.GridSizeM);
            Double CenterY = Field.MinY + ((Cell.By + 0.5) * Field.GridSizeM);
            Int32 RadiusCells = (Int32)Math.Ceiling(HaulGridSizeM / Field.GridSizeM) + 2;
            Double BestDistanceM = DistanceFromCellCenterToExteriorM(Cell);
            Double NormalX = 0.0;
            Double NormalY = 0.0;
            for (Int32 Dy = -RadiusCells; Dy <= RadiusCells; Dy += 1)
            {
                for (Int32 Dx = -RadiusCells; Dx <= RadiusCells; Dx += 1)
                {
                    if ((Dx == 0) && (Dy == 0)) continue;
                    if (!IsExterior(Cell.Bx + Dx, Cell.By + Dy)) continue;
                    (Double NearestX, Double NearestY, Double DistanceM) =
                        NearestPointOnCellRectangle(
                            CenterX,
                            CenterY,
                            Cell.Bx + Dx,
                            Cell.By + Dy);
                    if (DistanceM > BestDistanceM + (0.25 * Field.GridSizeM)) continue;
                    if (DistanceM <= NUMERIC_TOLERANCE) continue;
                    NormalX += (NearestX - CenterX) / DistanceM;
                    NormalY += (NearestY - CenterY) / DistanceM;
                }
            }
            if (Math.Sqrt((NormalX * NormalX) + (NormalY * NormalY)) <= NUMERIC_TOLERANCE)
            {
                return false;
            }

            // A tangent is perpendicular to the outward mask gradient. Direction is axial, so normalize to [0,180).
            Double TangentX = -NormalY;
            Double TangentY = NormalX;
            HeadingDeg = NormalizeAxialHeading(
                Math.Atan2(TangentX, TangentY) * 180.0 / Math.PI);
            return true;
        }

        /// <summary>Returns true when a work heading is parallel to the local edge within the supplied tolerance.</summary>
        public Boolean IsParallelToEdge
            (
            (Int32 Bx, Int32 By) Cell,
            Double HeadingDeg,
            Double ToleranceDeg
            )
        {
            if (!TryGetParallelHeadingDeg(Cell, out Double EdgeHeadingDeg)) return true;
            return AxialHeadingDifferenceDeg(HeadingDeg, EdgeHeadingDeg) <= ToleranceDeg + NUMERIC_TOLERANCE;
        }

        /// <summary>
        /// Returns true for points in the field or in the outer connected exterior no farther than one cell away.
        /// Internal holes and exclusions are never permitted.
        /// </summary>
        public Boolean ContainsWithExteriorAllowance
            (
            Double X,
            Double Y
            )
        {
            if (Field.ContainsWorldXy(X, Y)) return true;
            Int32 Bx = (Int32)Math.Floor((X - Field.MinX) / Field.GridSizeM);
            Int32 By = (Int32)Math.Floor((Y - Field.MinY) / Field.GridSizeM);
            if (!IsExterior(Bx, By)) return false;
            return DistanceToNearestValidCellM(X, Y) <= ExteriorAllowanceM + NUMERIC_TOLERANCE;
        }

        /// <summary>Returns true when a point is in the permitted exterior strip rather than in the field.</summary>
        public Boolean IsPermittedExterior
            (
            Double X,
            Double Y
            ) => !Field.ContainsWorldXy(X, Y) && ContainsWithExteriorAllowance(X, Y);

        /// <summary>
        /// Builds a one-cell-padded planning mask used only to propose exterior-strip connectors.
        /// Every accepted connector must still be densely checked against this policy afterward.
        /// </summary>
        public Grid BuildExteriorAllowanceGrid
            (
            )
        {
            Int32 PaddingCells = (Int32)Math.Ceiling(HaulGridSizeM / Field.GridSizeM) + 1;
            Int32 Width = Field.Width + (2 * PaddingCells);
            Int32 Height = Field.Height + (2 * PaddingCells);
            Double MinX = Field.MinX - (PaddingCells * Field.GridSizeM);
            Double MinY = Field.MinY - (PaddingCells * Field.GridSizeM);
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                Double Y = MinY + ((By + 0.5) * Field.GridSizeM);
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Double X = MinX + ((Bx + 0.5) * Field.GridSizeM);
                    Valid[By, Bx] = ContainsWithExteriorAllowance(X, Y);
                }
            }
            return new Grid(
                Field.GridSizeM,
                Width,
                Height,
                MinX,
                MinY,
                Field.Zone,
                Field.IsNorthern,
                Field.MeanLat,
                Field.MeanLon,
                new Double[Height, Width],
                new Double[Height, Width],
                Valid,
                Field.SwellFactor,
                Field.Compaction,
                Field.MaxDepthMm,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>Returns the shortest distance from a point to any nearby valid-cell rectangle.</summary>
        public Double DistanceToNearestValidCellM
            (
            Double X,
            Double Y
            )
        {
            Int32 CenterBx = (Int32)Math.Floor((X - Field.MinX) / Field.GridSizeM);
            Int32 CenterBy = (Int32)Math.Floor((Y - Field.MinY) / Field.GridSizeM);
            Double BestM = Double.PositiveInfinity;
            for (Int32 By = CenterBy - 2; By <= CenterBy + 2; By += 1)
            {
                for (Int32 Bx = CenterBx - 2; Bx <= CenterBx + 2; Bx += 1)
                {
                    if (!IsValidCell(Bx, By)) continue;
                    Double MinX = Field.MinX + (Bx * Field.GridSizeM);
                    Double MinY = Field.MinY + (By * Field.GridSizeM);
                    Double MaxX = MinX + Field.GridSizeM;
                    Double MaxY = MinY + Field.GridSizeM;
                    Double Dx = X < MinX ? MinX - X : (X > MaxX ? X - MaxX : 0.0);
                    Double Dy = Y < MinY ? MinY - Y : (Y > MaxY ? Y - MaxY : 0.0);
                    BestM = Math.Min(BestM, Math.Sqrt((Dx * Dx) + (Dy * Dy)));
                }
            }
            return BestM;
        }

        /// <summary>Computes the smallest difference between two undirected headings.</summary>
        public static Double AxialHeadingDifferenceDeg
            (
            Double FirstDeg,
            Double SecondDeg
            )
        {
            Double First = NormalizeAxialHeading(FirstDeg);
            Double Second = NormalizeAxialHeading(SecondDeg);
            Double Difference = Math.Abs(First - Second);
            return Math.Min(Difference, 180.0 - Difference);
        }

        private Boolean IsValidCell
            (
            Int32 Bx,
            Int32 By
            ) => (Bx >= 0) && (By >= 0) && (Bx < Field.Width) && (By < Field.Height)
                && Field.Valid[By, Bx];

        private Boolean IsExterior
            (
            Int32 Bx,
            Int32 By
            )
        {
            if ((Bx < 0) || (By < 0) || (Bx >= Field.Width) || (By >= Field.Height)) return true;
            return ExteriorInvalid[By, Bx];
        }

        private Double DistanceFromCellCenterToExteriorM
            (
            (Int32 Bx, Int32 By) Cell
            )
        {
            Double X = Field.MinX + ((Cell.Bx + 0.5) * Field.GridSizeM);
            Double Y = Field.MinY + ((Cell.By + 0.5) * Field.GridSizeM);
            Int32 RadiusCells = (Int32)Math.Ceiling(HaulGridSizeM / Field.GridSizeM) + 2;
            Double BestM = Double.PositiveInfinity;
            for (Int32 Dy = -RadiusCells; Dy <= RadiusCells; Dy += 1)
            {
                for (Int32 Dx = -RadiusCells; Dx <= RadiusCells; Dx += 1)
                {
                    if (!IsExterior(Cell.Bx + Dx, Cell.By + Dy)) continue;
                    BestM = Math.Min(
                        BestM,
                        NearestPointOnCellRectangle(
                            X,
                            Y,
                            Cell.Bx + Dx,
                            Cell.By + Dy).DistanceM);
                }
            }
            return BestM;
        }

        private (Double NearestX, Double NearestY, Double DistanceM) NearestPointOnCellRectangle
            (
            Double X,
            Double Y,
            Int32 Bx,
            Int32 By
            )
        {
            Double MinX = Field.MinX + (Bx * Field.GridSizeM);
            Double MinY = Field.MinY + (By * Field.GridSizeM);
            Double MaxX = MinX + Field.GridSizeM;
            Double MaxY = MinY + Field.GridSizeM;
            Double NearestX = Math.Max(MinX, Math.Min(MaxX, X));
            Double NearestY = Math.Max(MinY, Math.Min(MaxY, Y));
            Double Dx = NearestX - X;
            Double Dy = NearestY - Y;
            return (NearestX, NearestY, Math.Sqrt((Dx * Dx) + (Dy * Dy)));
        }

        private static Double NormalizeAxialHeading
            (
            Double HeadingDeg
            )
        {
            Double Output = HeadingDeg % 180.0;
            if (Output < 0.0) Output += 180.0;
            return Output;
        }

        private static Boolean[,] BuildExteriorInvalid
            (
            Grid Field
            )
        {
            Boolean[,] Exterior = new Boolean[Field.Height, Field.Width];
            Queue<(Int32 Bx, Int32 By)> Open = new Queue<(Int32 Bx, Int32 By)>();
            for (Int32 Bx = 0; Bx < Field.Width; Bx += 1)
            {
                EnqueueIfInvalid(Bx, 0, Field, Exterior, Open);
                EnqueueIfInvalid(Bx, Field.Height - 1, Field, Exterior, Open);
            }
            for (Int32 By = 0; By < Field.Height; By += 1)
            {
                EnqueueIfInvalid(0, By, Field, Exterior, Open);
                EnqueueIfInvalid(Field.Width - 1, By, Field, Exterior, Open);
            }
            Int32[] Offsets = { -1, 0, 1 };
            while (Open.Count > 0)
            {
                (Int32 Bx, Int32 By) Current = Open.Dequeue();
                foreach (Int32 Dy in Offsets)
                {
                    foreach (Int32 Dx in Offsets)
                    {
                        if ((Dx == 0) && (Dy == 0)) continue;
                        EnqueueIfInvalid(
                            Current.Bx + Dx,
                            Current.By + Dy,
                            Field,
                            Exterior,
                            Open);
                    }
                }
            }
            return Exterior;
        }

        private static void EnqueueIfInvalid
            (
            Int32 Bx,
            Int32 By,
            Grid Field,
            Boolean[,] Exterior,
            Queue<(Int32 Bx, Int32 By)> Open
            )
        {
            if ((Bx < 0) || (By < 0) || (Bx >= Field.Width) || (By >= Field.Height)) return;
            if (Field.Valid[By, Bx] || Exterior[By, Bx]) return;
            Exterior[By, Bx] = true;
            Open.Enqueue((Bx, By));
        }
    }
}
