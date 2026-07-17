using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Grid construction and BCY conversion utilities ported from landlevel.grid.
    /// </summary>
    static public class GridMath
    {
        private const Double DEFAULT_COMPACTION = 1.2;
        private const Double DEFAULT_GRID_SIZE_FT = 8.5;
        private const Double DEFAULT_MAX_DEPTH_MM = 61.0;
        private const Double DEFAULT_SWELL_FACTOR = 1.3;

        /// <summary>
        /// Computes available cut volume in BCY for a positive delta.
        /// </summary>
        /// <param name="DeltaM">Existing-target delta in metres.</param>
        /// <param name="AreaM2">Cell area in square metres.</param>
        /// <returns>Bank cut supply in BCY.</returns>
        static public Double SupplyBcy
            (
            Double DeltaM,
            Double AreaM2
            )
        {
            // Cut contributes supply; fill contributes none.
            if (DeltaM <= 0.0)
            {
                return 0.0;
            }

            return DeltaM * AreaM2 * Constants.M3_TO_CY;
        }

        /// <summary>
        /// Computes required cut BCY to satisfy a fill delta.
        /// </summary>
        /// <param name="DeltaM">Existing-target delta in metres.</param>
        /// <param name="AreaM2">Cell area in square metres.</param>
        /// <param name="CutfillRatio">Compaction/cut-fill ratio.</param>
        /// <returns>Bank cut demand in BCY.</returns>
        static public Double DemandBcy
            (
            Double DeltaM,
            Double AreaM2,
            Double CutfillRatio = DEFAULT_COMPACTION
            )
        {
            // Fill space must be multiplied by compaction to get required bank cut.
            if (DeltaM >= 0.0)
            {
                return 0.0;
            }

            Double FillSpaceM3 = (-DeltaM) * AreaM2;
            return FillSpaceM3 * Constants.M3_TO_CY * CutfillRatio;
        }

        /// <summary>
        /// Converts loose cubic yards to bank cubic yards.
        /// </summary>
        /// <param name="Lcy">Loose cubic yards.</param>
        /// <param name="SwellFactor">Swell factor.</param>
        /// <returns>Equivalent BCY.</returns>
        static public Double LcyToBcy
            (
            Double Lcy,
            Double SwellFactor = DEFAULT_SWELL_FACTOR
            )
        {
            // LCY = BCY * swell, so BCY = LCY / swell.
            return Lcy / SwellFactor;
        }

        /// <summary>
        /// Computes number of depth-limited layers needed for a depth.
        /// </summary>
        /// <param name="AbsDeltaM">Absolute depth in metres.</param>
        /// <param name="MaxDepthMm">Maximum per-pass depth in millimetres.</param>
        /// <returns>Layer count.</returns>
        static public Int32 LayersForDepth
            (
            Double AbsDeltaM,
            Double MaxDepthMm = DEFAULT_MAX_DEPTH_MM
            )
        {
            // Mirror Python ceil-based layer count.
            if (AbsDeltaM <= 0.0)
            {
                return 0;
            }

            return (Int32)Math.Ceiling(AbsDeltaM / (MaxDepthMm / Constants.MILLIMETERS_PER_METER));
        }

        /// <summary>
        /// Builds the processing grid from parsed AGD points.
        /// </summary>
        /// <param name="Agd">Parsed AGD field.</param>
        /// <param name="GridSizeFt">Grid size in feet.</param>
        /// <param name="SwellFactor">Swell factor.</param>
        /// <param name="Compaction">Compaction factor.</param>
        /// <param name="MaxDepthMm">Maximum pass depth in millimetres.</param>
        /// <returns>Constructed grid model.</returns>
        static public Grid BuildGrid
            (
            AgdField Agd,
            Double GridSizeFt = DEFAULT_GRID_SIZE_FT,
            Double SwellFactor = DEFAULT_SWELL_FACTOR,
            Double Compaction = DEFAULT_COMPACTION,
            Double MaxDepthMm = DEFAULT_MAX_DEPTH_MM
            )
        {
            // Convert requested bin size to metres so all geometry stays in UTM units.
            Double GridSizeM = GridSizeFt * Constants.FEET_TO_METERS;
            List<AgdPoint> Points = Agd.Points;

            Double MinX = Double.PositiveInfinity;
            Double MinY = Double.PositiveInfinity;
            Double MaxX = Double.NegativeInfinity;
            Double MaxY = Double.NegativeInfinity;
            foreach (AgdPoint Point in Points)
            {
                MinX = Math.Min(MinX, Point.Easting);
                MinY = Math.Min(MinY, Point.Northing);
                MaxX = Math.Max(MaxX, Point.Easting);
                MaxY = Math.Max(MaxY, Point.Northing);
            }

            Int32 Width = Math.Max(1, (Int32)Math.Ceiling((MaxX - MinX) / GridSizeM) + 1);
            Int32 Height = Math.Max(1, (Int32)Math.Ceiling((MaxY - MinY) / GridSizeM) + 1);

            Double[,] SumExisting = new Double[Height, Width];
            Double[,] SumTarget = new Double[Height, Width];
            Int32[,] Counts = new Int32[Height, Width];
            Boolean[,] Occupied = new Boolean[Height, Width];

            // Bin all input points into [by, bx] indices.
            foreach (AgdPoint Point in Points)
            {
                Int32 Bx = (Int32)Math.Floor((Point.Easting - MinX) / GridSizeM);
                Int32 By = (Int32)Math.Floor((Point.Northing - MinY) / GridSizeM);
                Bx = Math.Max(0, Math.Min(Width - 1, Bx));
                By = Math.Max(0, Math.Min(Height - 1, By));
                SumExisting[By, Bx] += Point.Existing;
                SumTarget[By, Bx] += Point.Proposed;
                Counts[By, Bx] += 1;
                Occupied[By, Bx] = true;
            }

            Double[,] Existing = new Double[Height, Width];
            Double[,] Target = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    // Unoccupied cells remain NaN until interpolation.
                    if (Counts[By, Bx] > 0)
                    {
                        Existing[By, Bx] = SumExisting[By, Bx] / Counts[By, Bx];
                        Target[By, Bx] = SumTarget[By, Bx] / Counts[By, Bx];
                    }
                    else
                    {
                        Existing[By, Bx] = Double.NaN;
                        Target[By, Bx] = Double.NaN;
                    }
                }
            }

            Boolean[,] Mask;
            if (Agd.Boundary.Count > 0)
            {
                List<(Double X, Double Y)> Ring = OrderBoundaryRing(Agd.Boundary);
                Mask = new Boolean[Height, Width];
                for (Int32 By = 0; By < Height; By += 1)
                {
                    Double Cy = MinY + ((By + Constants.HALF_SCALE) * GridSizeM);
                    for (Int32 Bx = 0; Bx < Width; Bx += 1)
                    {
                        Double Cx = MinX + ((Bx + Constants.HALF_SCALE) * GridSizeM);
                        Mask[By, Bx] = PointInPolygon(Cx, Cy, Ring);
                        if (Occupied[By, Bx])
                        {
                            Mask[By, Bx] = true;
                        }
                    }
                }
            }
            else
            {
                Mask = MaskFromObserved(Occupied);
            }

            FillMissing(Existing, Target, Occupied, Mask);

            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    // Cells outside mask are invalid and forced to NaN.
                    if (!Mask[By, Bx])
                    {
                        Existing[By, Bx] = Double.NaN;
                        Target[By, Bx] = Double.NaN;
                        Valid[By, Bx] = false;
                    }
                    else
                    {
                        Valid[By, Bx] = !(Double.IsNaN(Existing[By, Bx]) || Double.IsNaN(Target[By, Bx]));
                    }
                }
            }

            return new Grid
                (
                GridSizeM,
                Width,
                Height,
                MinX,
                MinY,
                Agd.Zone,
                Agd.IsNorthern,
                Agd.MeanLat,
                Agd.MeanLon,
                Existing,
                Target,
                Valid,
                SwellFactor,
                Compaction,
                MaxDepthMm,
                new List<List<(Double X, Double Y)>>()
                );
        }

        /// <summary>
        /// Applies exclusion rings by removing affected bins from the grid mask.
        /// </summary>
        /// <param name="Grid">Grid to mutate.</param>
        /// <param name="Rings">Projected exclusion rings.</param>
        /// <returns>Summary of removed bins and volumes.</returns>
        static public ExclusionResult ApplyExclusions
            (
            Grid Grid,
            List<List<(Double X, Double Y)>> Rings
            )
        {
            // A blank ring list is a no-op.
            if ((Rings == null) || (Rings.Count == 0))
            {
                return new ExclusionResult(0, 0.0, 0.0);
            }

            Double[,] SupplyBefore = Grid.SupplyGrid();
            Double[,] DemandBefore = Grid.DemandGrid();
            Boolean[,] Removed = new Boolean[Grid.Height, Grid.Width];

            foreach (List<(Double X, Double Y)> Ring in Rings)
            {
                if (Ring.Count < 3)
                {
                    continue;
                }

                Double MinX = Double.PositiveInfinity;
                Double MinY = Double.PositiveInfinity;
                Double MaxX = Double.NegativeInfinity;
                Double MaxY = Double.NegativeInfinity;
                foreach ((Double X, Double Y) Point in Ring)
                {
                    MinX = Math.Min(MinX, Point.X);
                    MinY = Math.Min(MinY, Point.Y);
                    MaxX = Math.Max(MaxX, Point.X);
                    MaxY = Math.Max(MaxY, Point.Y);
                }

                Int32 BxLo = (Int32)Math.Floor((MinX - Grid.MinX) / Grid.GridSizeM);
                Int32 BxHi = (Int32)Math.Floor((MaxX - Grid.MinX) / Grid.GridSizeM);
                Int32 ByLo = (Int32)Math.Floor((MinY - Grid.MinY) / Grid.GridSizeM);
                Int32 ByHi = (Int32)Math.Floor((MaxY - Grid.MinY) / Grid.GridSizeM);

                BxLo = Math.Max(0, Math.Min(Grid.Width - 1, BxLo));
                BxHi = Math.Max(0, Math.Min(Grid.Width - 1, BxHi));
                ByLo = Math.Max(0, Math.Min(Grid.Height - 1, ByLo));
                ByHi = Math.Max(0, Math.Min(Grid.Height - 1, ByHi));

                for (Int32 By = ByLo; By <= ByHi; By += 1)
                {
                    for (Int32 Bx = BxLo; Bx <= BxHi; Bx += 1)
                    {
                        Removed[By, Bx] = true;
                    }
                }
            }

            Int32 NRemoved = 0;
            Double RemovedSupply = 0.0;
            Double RemovedDemand = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    // Only previously valid bins count as removed.
                    if (Removed[By, Bx] && Grid.Valid[By, Bx])
                    {
                        NRemoved += 1;
                        RemovedSupply += SupplyBefore[By, Bx];
                        RemovedDemand += DemandBefore[By, Bx];
                        Grid.Valid[By, Bx] = false;
                    }
                }
            }

            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        Grid.Existing[By, Bx] = Double.NaN;
                        Grid.Target[By, Bx] = Double.NaN;
                    }
                }
            }

            List<List<(Double X, Double Y)>> OutRings = new List<List<(Double X, Double Y)>>();
            foreach (List<(Double X, Double Y)> Ring in Rings)
            {
                if (Ring.Count >= 3)
                {
                    OutRings.Add(new List<(Double X, Double Y)>(Ring));
                }
            }

            Grid.Exclusions = OutRings;
            return new ExclusionResult(NRemoved, RemovedSupply, RemovedDemand);
        }

        /// <summary>
        /// Generates a mask from occupied cells using row/column span fill and cleanup.
        /// </summary>
        /// <param name="Occupied">Observed occupancy mask.</param>
        /// <returns>Field mask.</returns>
        static public Boolean[,] MaskFromObserved
            (
            Boolean[,] Occupied
            )
        {
            Int32 Height = Occupied.GetLength(0);
            Int32 Width = Occupied.GetLength(1);
            Boolean Any = false;
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Any |= Occupied[By, Bx];
                }
            }

            if (!Any)
            {
                Boolean[,] All = new Boolean[Height, Width];
                for (Int32 By = 0; By < Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Width; Bx += 1)
                    {
                        All[By, Bx] = true;
                    }
                }

                return All;
            }

            Boolean[,] RowFill = (Boolean[,])Occupied.Clone();
            for (Int32 By = 0; By < Height; By += 1)
            {
                Int32 First = -1;
                Int32 Last = -1;
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (Occupied[By, Bx])
                    {
                        if (First < 0)
                        {
                            First = Bx;
                        }

                        Last = Bx;
                    }
                }

                if ((First >= 0) && (Last > First))
                {
                    for (Int32 Bx = First; Bx <= Last; Bx += 1)
                    {
                        RowFill[By, Bx] = true;
                    }
                }
            }

            Boolean[,] ColFill = (Boolean[,])Occupied.Clone();
            for (Int32 Bx = 0; Bx < Width; Bx += 1)
            {
                Int32 First = -1;
                Int32 Last = -1;
                for (Int32 By = 0; By < Height; By += 1)
                {
                    if (Occupied[By, Bx])
                    {
                        if (First < 0)
                        {
                            First = By;
                        }

                        Last = By;
                    }
                }

                if ((First >= 0) && (Last > First))
                {
                    for (Int32 By = First; By <= Last; By += 1)
                    {
                        ColFill[By, Bx] = true;
                    }
                }
            }

            Boolean[,] Envelope = OrMask(RowFill, ColFill);
            Boolean[,] Closed = BinaryErode(BinaryDilate(Envelope, 2), 2);
            Boolean[,] Filled = FillHoles(Closed);
            Boolean[,] Main = LargestComponent(Filled);
            return OrMask(Main, Occupied);
        }

        /// <summary>
        /// Binary dilate helper used for mask closure.
        /// </summary>
        /// <param name="Mask">Input mask.</param>
        /// <param name="Passes">Iteration count.</param>
        /// <returns>Dilated mask.</returns>
        static public Boolean[,] BinaryDilate
            (
            Boolean[,] Mask,
            Int32 Passes = 1
            )
        {
            Int32 Height = Mask.GetLength(0);
            Int32 Width = Mask.GetLength(1);
            Boolean[,] Out = (Boolean[,])Mask.Clone();
            for (Int32 Pass = 0; Pass < Math.Max(0, Passes); Pass += 1)
            {
                Boolean[,] Next = new Boolean[Height, Width];
                for (Int32 By = 0; By < Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Width; Bx += 1)
                    {
                        Boolean Value = false;
                        for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                        {
                            for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                            {
                                Int32 Ny = By + Dy;
                                Int32 Nx = Bx + Dx;
                                if ((Ny >= 0) && (Ny < Height) && (Nx >= 0) && (Nx < Width) && Out[Ny, Nx])
                                {
                                    Value = true;
                                }
                            }
                        }

                        Next[By, Bx] = Value;
                    }
                }

                Out = Next;
            }

            return Out;
        }

        /// <summary>
        /// Binary erode helper used for mask closure.
        /// </summary>
        /// <param name="Mask">Input mask.</param>
        /// <param name="Passes">Iteration count.</param>
        /// <returns>Eroded mask.</returns>
        static public Boolean[,] BinaryErode
            (
            Boolean[,] Mask,
            Int32 Passes = 1
            )
        {
            Int32 Height = Mask.GetLength(0);
            Int32 Width = Mask.GetLength(1);
            Boolean[,] Out = (Boolean[,])Mask.Clone();
            for (Int32 Pass = 0; Pass < Math.Max(0, Passes); Pass += 1)
            {
                Boolean[,] Next = new Boolean[Height, Width];
                for (Int32 By = 0; By < Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Width; Bx += 1)
                    {
                        Boolean Value = true;
                        for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                        {
                            for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                            {
                                Int32 Ny = By + Dy;
                                Int32 Nx = Bx + Dx;
                                if ((Ny < 0) || (Ny >= Height) || (Nx < 0) || (Nx >= Width) || !Out[Ny, Nx])
                                {
                                    Value = false;
                                }
                            }
                        }

                        Next[By, Bx] = Value;
                    }
                }

                Out = Next;
            }

            return Out;
        }

        /// <summary>
        /// Fills internal holes in a binary mask.
        /// </summary>
        /// <param name="Mask">Input mask.</param>
        /// <returns>Mask with enclosed holes filled.</returns>
        static public Boolean[,] FillHoles
            (
            Boolean[,] Mask
            )
        {
            Int32 Height = Mask.GetLength(0);
            Int32 Width = Mask.GetLength(1);
            Boolean[,] Inv = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Inv[By, Bx] = !Mask[By, Bx];
                }
            }

            Boolean[,] Outside = new Boolean[Height, Width];
            Stack<(Int32 X, Int32 Y)> Stack = new Stack<(Int32 X, Int32 Y)>();
            for (Int32 Bx = 0; Bx < Width; Bx += 1)
            {
                if (Inv[0, Bx]) Stack.Push((Bx, 0));
                if (Inv[Height - 1, Bx]) Stack.Push((Bx, Height - 1));
            }

            for (Int32 By = 0; By < Height; By += 1)
            {
                if (Inv[By, 0]) Stack.Push((0, By));
                if (Inv[By, Width - 1]) Stack.Push((Width - 1, By));
            }

            while (Stack.Count > 0)
            {
                (Int32 X, Int32 Y) Cell = Stack.Pop();
                if ((Cell.X < 0) || (Cell.X >= Width) || (Cell.Y < 0) || (Cell.Y >= Height))
                {
                    continue;
                }

                if (Outside[Cell.Y, Cell.X] || !Inv[Cell.Y, Cell.X])
                {
                    continue;
                }

                Outside[Cell.Y, Cell.X] = true;
                Stack.Push((Cell.X - 1, Cell.Y));
                Stack.Push((Cell.X + 1, Cell.Y));
                Stack.Push((Cell.X, Cell.Y - 1));
                Stack.Push((Cell.X, Cell.Y + 1));
            }

            Boolean[,] Out = (Boolean[,])Mask.Clone();
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (Inv[By, Bx] && !Outside[By, Bx])
                    {
                        Out[By, Bx] = true;
                    }
                }
            }

            return Out;
        }

        /// <summary>
        /// Keeps only the largest 8-connected component.
        /// </summary>
        /// <param name="Mask">Input mask.</param>
        /// <returns>Largest component mask.</returns>
        static public Boolean[,] LargestComponent
            (
            Boolean[,] Mask
            )
        {
            Int32 Height = Mask.GetLength(0);
            Int32 Width = Mask.GetLength(1);
            Boolean[,] Seen = new Boolean[Height, Width];
            List<(Int32 X, Int32 Y)> Best = new List<(Int32 X, Int32 Y)>();
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (Seen[By, Bx] || !Mask[By, Bx])
                    {
                        continue;
                    }

                    List<(Int32 X, Int32 Y)> Comp = new List<(Int32 X, Int32 Y)>();
                    Stack<(Int32 X, Int32 Y)> Stack = new Stack<(Int32 X, Int32 Y)>();
                    Stack.Push((Bx, By));
                    Seen[By, Bx] = true;
                    while (Stack.Count > 0)
                    {
                        (Int32 X, Int32 Y) Cell = Stack.Pop();
                        Comp.Add(Cell);
                        for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                        {
                            for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                            {
                                if ((Dx == 0) && (Dy == 0))
                                {
                                    continue;
                                }

                                Int32 Nx = Cell.X + Dx;
                                Int32 Ny = Cell.Y + Dy;
                                if ((Nx >= 0) && (Nx < Width) && (Ny >= 0) && (Ny < Height) && !Seen[Ny, Nx] && Mask[Ny, Nx])
                                {
                                    Seen[Ny, Nx] = true;
                                    Stack.Push((Nx, Ny));
                                }
                            }
                        }
                    }

                    if (Comp.Count > Best.Count)
                    {
                        Best = Comp;
                    }
                }
            }

            Boolean[,] Out = new Boolean[Height, Width];
            foreach ((Int32 X, Int32 Y) Cell in Best)
            {
                Out[Cell.Y, Cell.X] = true;
            }

            return Out;
        }

        /// <summary>
        /// Point-in-ring helper.
        /// </summary>
        /// <param name="X">Point X.</param>
        /// <param name="Y">Point Y.</param>
        /// <param name="Ring">Polygon ring.</param>
        /// <returns>Whether point lies in polygon.</returns>
        static public Boolean PointInPolygon
            (
            Double X,
            Double Y,
            List<(Double X, Double Y)> Ring
            )
        {
            // Use the same ray-cast toggle rule as the Python code.
            Boolean Inside = false;
            Int32 N = Ring.Count;
            Int32 J = N - 1;
            for (Int32 I = 0; I < N; I += 1)
            {
                (Double Xi, Double Yi) = Ring[I];
                (Double Xj, Double Yj) = Ring[J];
                if ((Yi > Y) != (Yj > Y))
                {
                    Double XAtY = ((Xj - Xi) * (Y - Yi) / ((Yj - Yi) + Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)) + Xi;
                    if (X <= XAtY)
                    {
                        Inside = !Inside;
                    }
                }

                J = I;
            }

            return Inside;
        }

        /// <summary>
        /// Orders boundary points by angle around centroid.
        /// </summary>
        /// <param name="Boundary">Boundary AGD points.</param>
        /// <returns>Ordered polygon ring.</returns>
        static public List<(Double X, Double Y)> OrderBoundaryRing
            (
            List<AgdPoint> Boundary
            )
        {
            List<(Double X, Double Y)> Out = new List<(Double X, Double Y)>();
            foreach (AgdPoint Point in Boundary)
            {
                Out.Add((Point.Easting, Point.Northing));
            }

            if (Out.Count < 3)
            {
                return Out;
            }

            Double Cx = 0.0;
            Double Cy = 0.0;
            foreach ((Double X, Double Y) Point in Out)
            {
                Cx += Point.X;
                Cy += Point.Y;
            }

            Cx /= Out.Count;
            Cy /= Out.Count;
            Out.Sort
                (
                (A, B) =>
                {
                    // Sort by polar angle around centroid.
                    Double AngleA = Math.Atan2(A.Y - Cy, A.X - Cx);
                    Double AngleB = Math.Atan2(B.Y - Cy, B.X - Cx);
                    return AngleA.CompareTo(AngleB);
                }
                );
            return Out;
        }

        /// <summary>
        /// Fills masked-but-empty cells by neighbor averaging.
        /// </summary>
        /// <param name="Existing">Existing elevation matrix to mutate.</param>
        /// <param name="Target">Target elevation matrix to mutate.</param>
        /// <param name="Occupied">Observed occupancy mask.</param>
        /// <param name="Mask">Field mask.</param>
        static public void FillMissing
            (
            Double[,] Existing,
            Double[,] Target,
            Boolean[,] Occupied,
            Boolean[,] Mask
            )
        {
            Int32 Height = Existing.GetLength(0);
            Int32 Width = Existing.GetLength(1);
            Boolean[,] Pending = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Pending[By, Bx] = Mask[By, Bx] && !Occupied[By, Bx];
                }
            }

            for (Int32 Iter = 0; Iter < (Math.Max(Height, Width) * 2); Iter += 1)
            {
                Boolean Any = false;
                for (Int32 By = 0; By < Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Width; Bx += 1)
                    {
                        Any |= Pending[By, Bx];
                    }
                }

                if (!Any)
                {
                    break;
                }

                List<(Int32 Bx, Int32 By, Double Ex, Double Tg)> Updates = new List<(Int32 Bx, Int32 By, Double Ex, Double Tg)>();
                for (Int32 By = 0; By < Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Width; Bx += 1)
                    {
                        if (!Pending[By, Bx])
                        {
                            continue;
                        }

                        Double SumEx = 0.0;
                        Double SumTg = 0.0;
                        Int32 Cnt = 0;
                        for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                        {
                            for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                            {
                                if ((Dx == 0) && (Dy == 0))
                                {
                                    continue;
                                }

                                Int32 Ny = By + Dy;
                                Int32 Nx = Bx + Dx;
                                if ((Ny < 0) || (Ny >= Height) || (Nx < 0) || (Nx >= Width))
                                {
                                    continue;
                                }

                                if (!Double.IsNaN(Existing[Ny, Nx]))
                                {
                                    SumEx += Existing[Ny, Nx];
                                    SumTg += Target[Ny, Nx];
                                    Cnt += 1;
                                }
                            }
                        }

                        if (Cnt > 0)
                        {
                            Updates.Add((Bx, By, SumEx / Cnt, SumTg / Cnt));
                        }
                    }
                }

                foreach ((Int32 Bx, Int32 By, Double Ex, Double Tg) Update in Updates)
                {
                    // Apply deferred updates so each iteration uses prior-state neighbors.
                    Existing[Update.By, Update.Bx] = Update.Ex;
                    Target[Update.By, Update.Bx] = Update.Tg;
                    Pending[Update.By, Update.Bx] = false;
                }
            }
        }

        /// <summary>
        /// Executes the or mask operation.
        /// </summary>
        /// <param name="A">The a used by the operation.</param>
        /// <param name="B">The b used by the operation.</param>
        /// <returns>The resulting or mask.</returns>

        static private Boolean[,] OrMask
            (
            Boolean[,] A,
            Boolean[,] B
            )
        {
            // Consolidate mask OR to keep closure pipeline readable.
            Int32 Height = A.GetLength(0);
            Int32 Width = A.GetLength(1);
            Boolean[,] Out = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Out[By, Bx] = A[By, Bx] || B[By, Bx];
                }
            }

            return Out;
        }
    }

    /// <summary>
    /// Core grid model storing elevations and validity mask.
    /// </summary>
    public sealed class Grid
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="Grid"/> class.
        /// </summary>
        /// <param name="GridSizeM">Grid size in metres.</param>
        /// <param name="Width">Grid width in bins.</param>
        /// <param name="Height">Grid height in bins.</param>
        /// <param name="MinX">Minimum easting.</param>
        /// <param name="MinY">Minimum northing.</param>
        /// <param name="Zone">UTM zone.</param>
        /// <param name="IsNorthern">Hemisphere flag.</param>
        /// <param name="MeanLat">Mean latitude.</param>
        /// <param name="MeanLon">Mean longitude.</param>
        /// <param name="Existing">Existing elevations [by,bx].</param>
        /// <param name="Target">Target elevations [by,bx].</param>
        /// <param name="Valid">Valid mask [by,bx].</param>
        /// <param name="SwellFactor">Swell factor.</param>
        /// <param name="Compaction">Compaction factor.</param>
        /// <param name="MaxDepthMm">Max depth mm.</param>
        /// <param name="Exclusions">Exclusion rings.</param>
        public Grid
            (
            Double GridSizeM,
            Int32 Width,
            Int32 Height,
            Double MinX,
            Double MinY,
            Int32 Zone,
            Boolean IsNorthern,
            Double MeanLat,
            Double MeanLon,
            Double[,] Existing,
            Double[,] Target,
            Boolean[,] Valid,
            Double SwellFactor,
            Double Compaction,
            Double MaxDepthMm,
            List<List<(Double X, Double Y)>> Exclusions
            )
        {
            // Store geometry and cell-state matrices used throughout planning.
            this.GridSizeM = GridSizeM;
            this.Width = Width;
            this.Height = Height;
            this.MinX = MinX;
            this.MinY = MinY;
            this.Zone = Zone;
            this.IsNorthern = IsNorthern;
            this.MeanLat = MeanLat;
            this.MeanLon = MeanLon;
            this.Existing = Existing;
            this.Target = Target;
            this.Valid = Valid;
            this.SwellFactor = SwellFactor;
            this.Compaction = Compaction;
            this.MaxDepthMm = MaxDepthMm;
            this.Exclusions = Exclusions;
            CenterX = new Double[Height, Width];
            CenterY = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                Double Cy = MinY + ((By + Constants.HALF_SCALE) * GridSizeM);
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    CenterX[By, Bx] = MinX + ((Bx + Constants.HALF_SCALE) * GridSizeM);
                    CenterY[By, Bx] = Cy;
                }
            }
        }

        /// <summary>
        /// Gets grid size in metres.
        /// </summary>
        public Double GridSizeM { get; }

        /// <summary>
        /// Gets width in bins.
        /// </summary>
        public Int32 Width { get; }

        /// <summary>
        /// Gets height in bins.
        /// </summary>
        public Int32 Height { get; }

        /// <summary>
        /// Gets minimum easting.
        /// </summary>
        public Double MinX { get; }

        /// <summary>
        /// Gets minimum northing.
        /// </summary>
        public Double MinY { get; }

        /// <summary>
        /// Gets UTM zone.
        /// </summary>
        public Int32 Zone { get; }

        /// <summary>
        /// Gets a value indicating whether UTM is northern hemisphere.
        /// </summary>
        public Boolean IsNorthern { get; }

        /// <summary>
        /// Gets mean latitude.
        /// </summary>
        public Double MeanLat { get; }

        /// <summary>
        /// Gets mean longitude.
        /// </summary>
        public Double MeanLon { get; }

        /// <summary>
        /// Gets existing elevations [by,bx].
        /// </summary>
        public Double[,] Existing { get; set; }

        /// <summary>
        /// Gets target elevations [by,bx].
        /// </summary>
        public Double[,] Target { get; set; }

        /// <summary>
        /// Gets valid field mask [by,bx].
        /// </summary>
        public Boolean[,] Valid { get; set; }

        /// <summary>
        /// Gets swell factor.
        /// </summary>
        public Double SwellFactor { get; }

        /// <summary>
        /// Gets compaction ratio.
        /// </summary>
        public Double Compaction { get; }

        /// <summary>
        /// Gets max depth per pass in millimetres.
        /// </summary>
        public Double MaxDepthMm { get; }

        /// <summary>
        /// Gets precomputed cell-center eastings [by,bx] in metres.
        /// </summary>
        public Double[,] CenterX { get; }

        /// <summary>
        /// Gets precomputed cell-center northings [by,bx] in metres.
        /// </summary>
        public Double[,] CenterY { get; }

        /// <summary>
        /// Gets or sets exclusion rings.
        /// </summary>
        public List<List<(Double X, Double Y)>> Exclusions { get; set; }

        /// <summary>
        /// Gets cell area in square metres.
        /// </summary>
        public Double AreaM2
        {
            get
            {
                // Uniform square cells share one area value.
                return this.GridSizeM * this.GridSizeM;
            }
        }

        /// <summary>
        /// Computes existing-target delta matrix with NaN outside valid mask.
        /// </summary>
        /// <returns>Delta matrix [by,bx].</returns>
        public Double[,] Delta
            (
            )
        {
            // Preserve NaN outside mask exactly like the Python property.
            Double[,] Out = new Double[this.Height, this.Width];
            for (Int32 By = 0; By < this.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < this.Width; Bx += 1)
                {
                    Out[By, Bx] = this.Valid[By, Bx] ? (this.Existing[By, Bx] - this.Target[By, Bx]) : Double.NaN;
                }
            }

            return Out;
        }

        /// <summary>
        /// Computes per-cell cut supply in BCY.
        /// </summary>
        /// <returns>Supply matrix [by,bx].</returns>
        public Double[,] SupplyGrid
            (
            )
        {
            // Convert positive deltas into bank cut supply.
            Double[,] Out = new Double[this.Height, this.Width];
            for (Int32 By = 0; By < this.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < this.Width; Bx += 1)
                {
                    if (!this.Valid[By, Bx])
                    {
                        Out[By, Bx] = 0.0;
                        continue;
                    }

                    Double Delta = this.Existing[By, Bx] - this.Target[By, Bx];
                    Double Positive = Math.Max(Delta, 0.0);
                    Out[By, Bx] = Positive * AreaM2 * Constants.M3_TO_CY;
                }
            }

            return Out;
        }

        /// <summary>
        /// Computes per-cell fill demand in BCY of required cut soil.
        /// </summary>
        /// <returns>Demand matrix [by,bx].</returns>
        public Double[,] DemandGrid
            (
            )
        {
            // Convert negative deltas into required compacted fill demand.
            Double[,] Out = new Double[this.Height, this.Width];
            for (Int32 By = 0; By < this.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < this.Width; Bx += 1)
                {
                    if (!this.Valid[By, Bx])
                    {
                        Out[By, Bx] = 0.0;
                        continue;
                    }

                    Double Delta = this.Existing[By, Bx] - this.Target[By, Bx];
                    Double Fill = Math.Max(-Delta, 0.0);
                    Out[By, Bx] = Fill * AreaM2 * Constants.M3_TO_CY * Compaction;
                }
            }

            return Out;
        }

        /// <summary>
        /// Returns world centre coordinates for a cell.
        /// </summary>
        /// <param name="Bx">Cell x index.</param>
        /// <param name="By">Cell y index.</param>
        /// <returns>Cell centre (x,y) in metres.</returns>
        public (Double X, Double Y) CellCenterXy
            (
            Int32 Bx,
            Int32 By
            )
        {
            return (this.CenterX[By, Bx], this.CenterY[By, Bx]);
        }

        /// <summary>
        /// Backward-compatible alias with upper-case XY suffix.
        /// </summary>
        /// <param name="Bx">Cell x index.</param>
        /// <param name="By">Cell y index.</param>
        /// <returns>Cell centre coordinate.</returns>
        public (Double X, Double Y) CellCenterXY
            (
            Int32 Bx,
            Int32 By
            )
        {
            // Preserve existing call sites that use XY casing.
            return CellCenterXy(Bx, By);
        }

        /// <summary>
        /// True when a world point lies in a valid field cell (not outside the mask).
        /// </summary>
        /// <param name="X">Easting in metres.</param>
        /// <param name="Y">Northing in metres.</param>
        /// <returns>True when the containing bin is valid.</returns>
        public Boolean ContainsWorldXy
            (
            Double X,
            Double Y
            )
        {
            Int32 Bx = (Int32)Math.Floor((X - this.MinX) / this.GridSizeM);
            Int32 By = (Int32)Math.Floor((Y - this.MinY) / this.GridSizeM);
            if ((Bx < 0) || (By < 0) || (Bx >= this.Width) || (By >= this.Height))
            {
                return false;
            }

            return this.Valid[By, Bx];
        }
    }

    /// <summary>
    /// Outcome of applying exclusion zones to a grid.
    /// </summary>
    public sealed class ExclusionResult
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="ExclusionResult"/> class.
        /// </summary>
        /// <param name="NBinsRemoved">Number of removed bins.</param>
        /// <param name="RemovedSupplyBcy">Removed supply in BCY.</param>
        /// <param name="RemovedDemandBcy">Removed demand in BCY.</param>
        public ExclusionResult
            (
            Int32 NBinsRemoved,
            Double RemovedSupplyBcy,
            Double RemovedDemandBcy
            )
        {
            // Persist removal stats for reporting.
            this.NBinsRemoved = NBinsRemoved;
            this.RemovedSupplyBcy = RemovedSupplyBcy;
            this.RemovedDemandBcy = RemovedDemandBcy;
        }

        /// <summary>
        /// Gets number of bins removed by exclusions.
        /// </summary>
        public Int32 NBinsRemoved { get; }

        /// <summary>
        /// Gets removed cut supply in BCY.
        /// </summary>
        public Double RemovedSupplyBcy { get; }

        /// <summary>
        /// Gets removed fill demand in BCY.
        /// </summary>
        public Double RemovedDemandBcy { get; }
    }
}
