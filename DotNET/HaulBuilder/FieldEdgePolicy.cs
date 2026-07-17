using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Describes the outer field boundary, edge-parallel work headings, and the permitted one-cell exterior strip.
    /// </summary>
    public sealed class FieldEdgePolicy
    {
        private const Double NUMERIC_TOLERANCE = Constants.NUMERIC_COMPARISON_TOLERANCE;
        private readonly Grid Field;
        private readonly Boolean[,] ExteriorInvalid;
        private readonly Double HaulGridSizeM;

        /// <summary>Builds one immutable boundary classification for a field grid.</summary>
        public FieldEdgePolicy
            (
            Grid Field,
            Double HaulGridSizeM = Constants.DEFAULT_BLADE_WIDTH_MM / Constants.MILLIMETERS_PER_METER
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
            if (!this.Field.ContainsWorldXy(X, Y)) return false;
            return IsEdgeWorkCell((
                (Int32)Math.Floor((X - this.Field.MinX) / this.Field.GridSizeM),
                (Int32)Math.Floor((Y - this.Field.MinY) / this.Field.GridSizeM)));
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

            Double CenterX = this.Field.MinX + ((Cell.Bx + Constants.HALF_SCALE) * this.Field.GridSizeM);
            Double CenterY = this.Field.MinY + ((Cell.By + Constants.HALF_SCALE) * this.Field.GridSizeM);
            Int32 RadiusCells = (Int32)Math.Ceiling(HaulGridSizeM / this.Field.GridSizeM) + 2;
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
                    if (DistanceM > BestDistanceM + (Constants.QUARTER_SCALE * this.Field.GridSizeM)) continue;
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
                Math.Atan2(TangentX, TangentY) * Constants.DEGREES_PER_HALF_TURN / Math.PI);
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
            if (this.Field.ContainsWorldXy(X, Y)) return true;
            Int32 Bx = (Int32)Math.Floor((X - this.Field.MinX) / this.Field.GridSizeM);
            Int32 By = (Int32)Math.Floor((Y - this.Field.MinY) / this.Field.GridSizeM);
            if (!IsExterior(Bx, By)) return false;
            return DistanceToNearestValidCellM(X, Y) <= ExteriorAllowanceM + NUMERIC_TOLERANCE;
        }

        /// <summary>Returns true when a point is in the permitted exterior strip rather than in the field.</summary>
        public Boolean IsPermittedExterior
            (
            Double X,
            Double Y
            ) => !this.Field.ContainsWorldXy(X, Y) && ContainsWithExteriorAllowance(X, Y);

        /// <summary>
        /// Builds a one-cell-padded planning mask used only to propose exterior-strip connectors.
        /// Every accepted connector must still be densely checked against this policy afterward.
        /// </summary>
        public Grid BuildExteriorAllowanceGrid
            (
            )
        {
            Int32 PaddingCells = (Int32)Math.Ceiling(HaulGridSizeM / this.Field.GridSizeM) + 1;
            Int32 Width = this.Field.Width + (2 * PaddingCells);
            Int32 Height = this.Field.Height + (2 * PaddingCells);
            Double MinX = this.Field.MinX - (PaddingCells * this.Field.GridSizeM);
            Double MinY = this.Field.MinY - (PaddingCells * this.Field.GridSizeM);
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                Double Y = MinY + ((By + Constants.HALF_SCALE) * this.Field.GridSizeM);
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Double X = MinX + ((Bx + Constants.HALF_SCALE) * this.Field.GridSizeM);
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
            Int32 CenterBx = (Int32)Math.Floor((X - this.Field.MinX) / this.Field.GridSizeM);
            Int32 CenterBy = (Int32)Math.Floor((Y - this.Field.MinY) / this.Field.GridSizeM);
            Double BestM = Double.PositiveInfinity;
            for (Int32 By = CenterBy - 2; By <= CenterBy + 2; By += 1)
            {
                for (Int32 Bx = CenterBx - 2; Bx <= CenterBx + 2; Bx += 1)
                {
                    if (!IsValidCell(Bx, By)) continue;
                    Double MinX = this.Field.MinX + (Bx * this.Field.GridSizeM);
                    Double MinY = this.Field.MinY + (By * this.Field.GridSizeM);
                    Double MaxX = MinX + this.Field.GridSizeM;
                    Double MaxY = MinY + this.Field.GridSizeM;
                    Double Dx = X < MinX ? MinX - X : (X > MaxX ? X - MaxX : 0.0);
                    Double Dy = Y < MinY ? MinY - Y : (Y > MaxY ? Y - MaxY : 0.0);
                    BestM = Math.Min(BestM, Math.Sqrt((Dx * Dx) + (Dy * Dy)));
                }
            }
            return BestM;
        }

        /// <summary>Computes the smallest difference between two undirected headings.</summary>
        static public Double AxialHeadingDifferenceDeg
            (
            Double FirstDeg,
            Double SecondDeg
            )
        {
            Double First = NormalizeAxialHeading(FirstDeg);
            Double Second = NormalizeAxialHeading(SecondDeg);
            Double Difference = Math.Abs(First - Second);
            return Math.Min(Difference, Constants.DEGREES_PER_HALF_TURN - Difference);
        }

        /// <summary>
        /// Determines whether valid cell.
        /// </summary>
        /// <param name="Bx">The bx used by the operation.</param>
        /// <param name="By">The by used by the operation.</param>
        /// <returns>True when valid cell; otherwise false.</returns>

        private Boolean IsValidCell
            (
            Int32 Bx,
            Int32 By
            ) => (Bx >= 0) && (By >= 0) && (Bx < this.Field.Width) && (By < this.Field.Height)
                && this.Field.Valid[By, Bx];

        /// <summary>
        /// Determines whether exterior.
        /// </summary>
        /// <param name="Bx">The bx used by the operation.</param>
        /// <param name="By">The by used by the operation.</param>
        /// <returns>True when exterior; otherwise false.</returns>

        private Boolean IsExterior
            (
            Int32 Bx,
            Int32 By
            )
        {
            if ((Bx < 0) || (By < 0) || (Bx >= this.Field.Width) || (By >= this.Field.Height)) return true;
            return ExteriorInvalid[By, Bx];
        }

        /// <summary>
        /// Executes the distance from cell center to exterior m operation.
        /// </summary>
        /// <param name="Cell">The cell used by the operation.</param>
        /// <returns>The resulting distance from cell center to exterior m.</returns>

        private Double DistanceFromCellCenterToExteriorM
            (
            (Int32 Bx, Int32 By) Cell
            )
        {
            Double X = this.Field.MinX + ((Cell.Bx + Constants.HALF_SCALE) * this.Field.GridSizeM);
            Double Y = this.Field.MinY + ((Cell.By + Constants.HALF_SCALE) * this.Field.GridSizeM);
            Int32 RadiusCells = (Int32)Math.Ceiling(HaulGridSizeM / this.Field.GridSizeM) + 2;
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

        /// <summary>
        /// Executes the nearest point on cell rectangle operation.
        /// </summary>
        /// <param name="X">Horizontal coordinate in field units.</param>
        /// <param name="Y">Vertical coordinate in field units.</param>
        /// <param name="Bx">The bx used by the operation.</param>
        /// <param name="By">The by used by the operation.</param>
        /// <returns>The resulting nearest point on cell rectangle.</returns>

        private (Double NearestX, Double NearestY, Double DistanceM) NearestPointOnCellRectangle
            (
            Double X,
            Double Y,
            Int32 Bx,
            Int32 By
            )
        {
            Double MinX = this.Field.MinX + (Bx * this.Field.GridSizeM);
            Double MinY = this.Field.MinY + (By * this.Field.GridSizeM);
            Double MaxX = MinX + this.Field.GridSizeM;
            Double MaxY = MinY + this.Field.GridSizeM;
            Double NearestX = Math.Max(MinX, Math.Min(MaxX, X));
            Double NearestY = Math.Max(MinY, Math.Min(MaxY, Y));
            Double Dx = NearestX - X;
            Double Dy = NearestY - Y;
            return (NearestX, NearestY, Math.Sqrt((Dx * Dx) + (Dy * Dy)));
        }

        /// <summary>
        /// Executes the normalize axial heading operation.
        /// </summary>
        /// <param name="HeadingDeg">Direction in degrees.</param>
        /// <returns>The resulting axial heading.</returns>

        private static Double NormalizeAxialHeading
            (
            Double HeadingDeg
            )
        {
            Double Output = HeadingDeg % Constants.DEGREES_PER_HALF_TURN;
            if (Output < 0.0) Output += Constants.DEGREES_PER_HALF_TURN;
            return Output;
        }

        /// <summary>
        /// Builds exterior invalid.
        /// </summary>
        /// <param name="Field">The field used by the operation.</param>
        /// <returns>The resulting exterior invalid.</returns>

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

        /// <summary>
        /// Executes the enqueue if invalid operation.
        /// </summary>
        /// <param name="Bx">The bx used by the operation.</param>
        /// <param name="By">The by used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <param name="Exterior">The exterior used by the operation.</param>
        /// <param name="Open">The open used by the operation.</param>

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
