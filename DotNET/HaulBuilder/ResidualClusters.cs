using System.Globalization;
using System.Text.Json;
using System.Text.Json.Serialization;

using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace HaulBuilder
{
    /// <summary>
    /// Groups residual cut and fill into clusters. It does not pair them or plan a route.
    /// A cell more than <see cref="DEEP_MM"/> seeds a cluster. The cluster grows through
    /// cells more than <see cref="FLOOR_MM"/>, then across a gap of <see cref="GAP_CELLS"/>.
    /// Volume above the floor must reach half a bowl or the component is dropped.
    /// A cluster bigger than <see cref="MAX_PIECE_BOWLS"/> bowls is bisected along its
    /// long axis until every piece is within that limit. Clusters are the reporting
    /// regions. Pieces are the parts a later step can pair.
    /// </summary>
    static public class ResidualClusters
    {
        /// <summary>Depth, in millimetres, that can seed a cluster.</summary>
        public const Double DEEP_MM = 150;

        /// <summary>Depth, in millimetres, through which a seed may grow. Volume is counted above this.</summary>
        public const Double FLOOR_MM = 50;

        /// <summary>Gaps this many cells wide are bridged into the cluster. Bridged cells add no volume at or below the floor.</summary>
        public const Int32 GAP_CELLS = 1;

        /// <summary>Neighbourhood used for the seed grow, the gap bridge, and the component labels.</summary>
        public const Int32 CONNECTIVITY = 8;

        /// <summary>Share of the bowl a cluster must hold, measured above the floor.</summary>
        public const Double MIN_CLUSTER_BOWL_FRACTION = 0.5;

        /// <summary>A piece larger than this many bowls is bisected along its long axis.</summary>
        public const Double MAX_PIECE_BOWLS = 2;

        /// <summary>
        /// Relative slack when the running haul meets half a piece.
        /// A rounding error must not pull another cell across the median.
        /// </summary>
        private const Double MEDIAN_SLACK = 1e-9;

        /// <summary>Cubic metres to cubic yards. Specified as 1.30795.</summary>
        public const Double M3_TO_CY = 1.30795;

        /// <summary>
        /// Clusters the residual grid.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres. Positive is cut. Negative is fill. Indexed [row, col].</param>
        /// <param name="Valid">True on field cells.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="FillToBank">Fill volume is multiplied by this after the yard conversion.</param>
        /// <returns>The kept clusters, the pieces cut from them, and the diagnostic counts.</returns>
        public static ResidualClusterReport Build
            (
            Double[,] ResidualMm,
            Boolean[,] Valid,
            Double CellSizeM,
            Double BowlBcy,
            Double FillToBank
            )
        {
            if (ResidualMm == null) throw new ArgumentNullException(nameof(ResidualMm));
            if (Valid == null) throw new ArgumentNullException(nameof(Valid));
            Int32 Rows = ResidualMm.GetLength(0);
            Int32 Cols = ResidualMm.GetLength(1);
            if ((Valid.GetLength(0) != Rows) || (Valid.GetLength(1) != Cols))
            {
                throw new ArgumentException("The valid mask must match the residual grid.", nameof(Valid));
            }
            if ((CellSizeM <= 0.0) || (BowlBcy <= 0.0) || (FillToBank <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(CellSizeM));
            }

            // Every valid cell can join a cluster, including the outer ring of the field.
            Boolean[,] Work = Valid;
            Double MinimumBcy = MIN_CLUSTER_BOWL_FRACTION * BowlBcy;
            Double CellAreaM2 = CellSizeM * CellSizeM;
            Int32 DeepCutBefore = 0;
            Int32 DeepFillBefore = 0;
            Int32 DeepCutAfter = 0;
            Int32 DeepFillAfter = 0;
            Double[,] CutDepth = new Double[Rows, Cols];
            Double[,] FillDepth = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (!Valid[Row, Col])
                    {
                        continue;
                    }
                    Double Residual = ResidualMm[Row, Col];
                    if (Residual > DEEP_MM)
                    {
                        DeepCutBefore += 1;
                    }
                    if (-Residual > DEEP_MM)
                    {
                        DeepFillBefore += 1;
                    }
                    if (!Work[Row, Col])
                    {
                        continue;
                    }
                    if (Residual > 0.0)
                    {
                        CutDepth[Row, Col] = Residual;
                        if (Residual > DEEP_MM)
                        {
                            DeepCutAfter += 1;
                        }
                    }
                    else if (Residual < 0.0)
                    {
                        FillDepth[Row, Col] = -Residual;
                        if (-Residual > DEEP_MM)
                        {
                            DeepFillAfter += 1;
                        }
                    }
                }
            }

            List<ResidualCluster> CutClusters = BuildSign(
                CutDepth, Work, Rows, Cols, CellSizeM, CellAreaM2, MinimumBcy, 1.0, "cut", "C");
            List<ResidualCluster> FillClusters = BuildSign(
                FillDepth, Work, Rows, Cols, CellSizeM, CellAreaM2, MinimumBcy, FillToBank, "fill", "F");
            List<ResidualCluster> Clusters = new List<ResidualCluster>(CutClusters.Count + FillClusters.Count);
            Clusters.AddRange(CutClusters);
            Clusters.AddRange(FillClusters);
            Double CutIn = SumHaul(CutClusters);
            Double FillIn = SumHaul(FillClusters);
            Double CutOut = OutsideHaul(CutDepth, CutClusters, Rows, Cols, CellAreaM2, 1.0);
            Double FillOut = OutsideHaul(FillDepth, FillClusters, Rows, Cols, CellAreaM2, FillToBank);
            ResidualClusterDiagnostics Diagnostics = new ResidualClusterDiagnostics(
                DeepCutBefore,
                DeepCutAfter,
                DeepFillBefore,
                DeepFillAfter,
                Clusters.Count,
                CutIn,
                FillIn,
                CutOut,
                FillOut);
            // Pieces are a second level. A cluster within two bowls is already one piece.
            Double PieceLimit = MAX_PIECE_BOWLS * BowlBcy;
            List<ResidualCluster> Pieces = new List<ResidualCluster>();
            foreach (ResidualCluster Cluster in Clusters)
            {
                Double[,] Depth = Cluster.Sign == "fill" ? FillDepth : CutDepth;
                Double Bank = Cluster.Sign == "fill" ? FillToBank : 1.0;
                Pieces.AddRange(SplitCluster(
                    Cluster, Depth, CellSizeM, CellAreaM2, Bank, PieceLimit));
            }
            return new ResidualClusterReport(Clusters, Pieces, Diagnostics);
        }

        /// <summary>
        /// Writes the residual in the field palette and rings each cluster.
        /// North is up. Each label is the cluster id and its haul volume.
        /// </summary>
        /// <param name="Path">PNG path to write.</param>
        /// <param name="ResidualMm">Residual in millimetres, indexed [row, col].</param>
        /// <param name="Valid">True on field cells.</param>
        /// <param name="Clusters">Clusters to outline.</param>
        /// <param name="PixelsPerCell">Pixels along one cell.</param>
        public static void SavePng
            (
            String Path,
            Double[,] ResidualMm,
            Boolean[,] Valid,
            IReadOnlyList<ResidualCluster> Clusters,
            Int32 PixelsPerCell
            )
        {
            if (String.IsNullOrEmpty(Path)) throw new ArgumentNullException(nameof(Path));
            if (ResidualMm == null) throw new ArgumentNullException(nameof(ResidualMm));
            if (Valid == null) throw new ArgumentNullException(nameof(Valid));
            if (Clusters == null) throw new ArgumentNullException(nameof(Clusters));
            if (PixelsPerCell < 1) throw new ArgumentOutOfRangeException(nameof(PixelsPerCell));
            Int32 Rows = ResidualMm.GetLength(0);
            Int32 Cols = ResidualMm.GetLength(1);
            Double[,] Metres = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    Metres[Row, Col] = Valid[Row, Col] ? (ResidualMm[Row, Col] / 1000.0) : Double.NaN;
                }
            }
            using Image<Rgba32> Sheet = Render.DiffToImageDigitize(Metres, Valid, PixelsPerCell);
            Int32[,] Owner = new Int32[Rows, Cols];
            for (Int32 Index = 0; Index < Clusters.Count; Index += 1)
            {
                foreach ((Int32 Row, Int32 Col) Cell in Clusters[Index].Cells)
                {
                    Owner[Cell.Row, Cell.Col] = Index + 1;
                }
            }
            Rgba32 Black = new Rgba32(0, 0, 0, 255);
            Rgba32 White = new Rgba32(255, 255, 255, 255);
            Rgba32 Cyan = new Rgba32(0, 220, 220, 255);
            for (Int32 Index = 0; Index < Clusters.Count; Index += 1)
            {
                Rgba32 Ink = Clusters[Index].Sign == "fill" ? Cyan : White;
                foreach ((Int32 Row, Int32 Col) Cell in Clusters[Index].Cells)
                {
                    Int32 X0 = Cell.Col * PixelsPerCell;
                    Int32 Y0 = (Rows - 1 - Cell.Row) * PixelsPerCell;
                    // An edge is drawn only where the next cell is a different cluster, so the interior stays readable.
                    if (OwnerOf(Owner, Cell.Row, Cell.Col - 1) != Index + 1)
                    {
                        FillRect(Sheet, X0, Y0, 3, PixelsPerCell, Black);
                        FillRect(Sheet, X0, Y0, 1, PixelsPerCell, Ink);
                    }
                    if (OwnerOf(Owner, Cell.Row, Cell.Col + 1) != Index + 1)
                    {
                        FillRect(Sheet, X0 + PixelsPerCell - 3, Y0, 3, PixelsPerCell, Black);
                        FillRect(Sheet, X0 + PixelsPerCell - 1, Y0, 1, PixelsPerCell, Ink);
                    }
                    if (OwnerOf(Owner, Cell.Row - 1, Cell.Col) != Index + 1)
                    {
                        FillRect(Sheet, X0, Y0 + PixelsPerCell - 3, PixelsPerCell, 3, Black);
                        FillRect(Sheet, X0, Y0 + PixelsPerCell - 1, PixelsPerCell, 1, Ink);
                    }
                    if (OwnerOf(Owner, Cell.Row + 1, Cell.Col) != Index + 1)
                    {
                        FillRect(Sheet, X0, Y0, PixelsPerCell, 3, Black);
                        FillRect(Sheet, X0, Y0, PixelsPerCell, 1, Ink);
                    }
                }
            }
            List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)> Plates =
                new List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)>();
            for (Int32 Index = 0; Index < Clusters.Count; Index += 1)
            {
                ResidualCluster Cluster = Clusters[Index];
                String Label = String.Format(
                    CultureInfo.InvariantCulture,
                    "{0} {1:0.0}",
                    Cluster.Id,
                    Cluster.HaulVolumeBcy);
                // Piece ids are longer than cluster ids, so the face stays at two pixels per font pixel.
                Int32 Scale = 2;
                Int32 PlateW = (Label.Length * 6 * Scale) + 8;
                Int32 PlateH = (7 * Scale) + 8;
                Int32 Cx = (Int32)Math.Round((Cluster.CentroidCol + 0.5) * PixelsPerCell, MidpointRounding.AwayFromZero);
                Int32 Cy = (Int32)Math.Round((Rows - Cluster.CentroidRow - 0.5) * PixelsPerCell, MidpointRounding.AwayFromZero);
                Int32 PlateX = Cx - (PlateW / 2);
                Int32 PreferredY = Cy - PlateH - 4;
                if (PlateX < 2)
                {
                    PlateX = 2;
                }
                if (PlateX + PlateW > Sheet.Width - 2)
                {
                    PlateX = Sheet.Width - 2 - PlateW;
                }
                // Dense pieces cannot all sit on their centroids. Walk outward until a free plate is found.
                (Int32 PlateXPlaced, Int32 PlateY) = PlacePlate(
                    Plates, PlateX, PreferredY, PlateW, PlateH, Sheet.Width, Sheet.Height);
                PlateX = PlateXPlaced;
                FillRect(Sheet, PlateX, PlateY, PlateW, PlateH, Black);
                DrawLabel(Sheet, PlateX + 4, PlateY + 4, Label, Scale, White);
                Plates.Add((PlateX, PlateY, PlateW, PlateH));
            }
            Sheet.Save(Path);
        }

        /// <summary>
        /// Writes the clusters and the pieces as JSON.
        /// </summary>
        /// <param name="Path">JSON path to write.</param>
        /// <param name="Report">Report to write. Diagnostics are not included.</param>
        public static void SaveJson
            (
            String Path,
            ResidualClusterReport Report
            )
        {
            if (String.IsNullOrEmpty(Path)) throw new ArgumentNullException(nameof(Path));
            if (Report == null) throw new ArgumentNullException(nameof(Report));
            JsonSerializerOptions Options = new JsonSerializerOptions
            {
                WriteIndented = true,
            };
            String Json = JsonSerializer.Serialize(Report, Options);
            File.WriteAllText(Path, Json);
        }

        /// <summary>
        /// Builds one sign, drops components under half a bowl, and numbers the rest.
        /// </summary>
        /// <param name="DepthMm">Positive depth for this sign. Zero outside the work mask.</param>
        /// <param name="Work">Valid cells. The outer ring is included.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="CellAreaM2">Cell area in square metres.</param>
        /// <param name="MinimumBcy">Half a bowl, in bank cubic yards.</param>
        /// <param name="BankFactor">1 for cut. The fill-to-bank ratio for fill.</param>
        /// <param name="Sign">"cut" or "fill".</param>
        /// <param name="IdPrefix">C for cut, F for fill.</param>
        /// <returns>Clusters sorted by haul volume, then by the smallest cell.</returns>
        private static List<ResidualCluster> BuildSign
            (
            Double[,] DepthMm,
            Boolean[,] Work,
            Int32 Rows,
            Int32 Cols,
            Double CellSizeM,
            Double CellAreaM2,
            Double MinimumBcy,
            Double BankFactor,
            String Sign,
            String IdPrefix
            )
        {
            Boolean[,] Grown = Hysteresis(DepthMm, Rows, Cols);
            Boolean[,] Bridged = Close(Grown, GAP_CELLS);
            // The closing erosion treats the outside of the grid as empty, so it
            // deletes cells on the field edge. Those cells were already in the cluster.
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (Grown[Row, Col])
                    {
                        Bridged[Row, Col] = true;
                    }
                }
            }
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (!Work[Row, Col])
                    {
                        Bridged[Row, Col] = false;
                    }
                }
            }
            List<List<(Int32 Row, Int32 Col)>> Components = Label(Bridged, Rows, Cols);
            List<ResidualCluster> Kept = new List<ResidualCluster>();
            foreach (List<(Int32 Row, Int32 Col)> Cells in Components)
            {
                ResidualCluster Cluster = Measure(
                    Cells, DepthMm, CellSizeM, CellAreaM2, BankFactor, Sign);
                if (Cluster.HaulVolumeBcy < MinimumBcy)
                {
                    continue;
                }
                Kept.Add(Cluster);
            }
            // Equal haul volumes keep the cluster whose lowest cell is first in row, then column.
            Kept.Sort(CompareClusters);
            for (Int32 Index = 0; Index < Kept.Count; Index += 1)
            {
                Kept[Index].AssignId(IdPrefix + (Index + 1).ToString(CultureInfo.InvariantCulture));
            }
            return Kept;
        }

        /// <summary>
        /// Orders by haul volume descending, then by the smallest cell.
        /// The smallest cell is the lowest row, and the lowest column on that row.
        /// </summary>
        /// <param name="Left">First cluster.</param>
        /// <param name="Right">Second cluster.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareClusters
            (
            ResidualCluster Left,
            ResidualCluster Right
            )
        {
            Int32 ByVolume = Right.HaulVolumeBcy.CompareTo(Left.HaulVolumeBcy);
            if (ByVolume != 0)
            {
                return ByVolume;
            }
            (Int32 Row, Int32 Col) LeftCell = SmallestCell(Left);
            (Int32 Row, Int32 Col) RightCell = SmallestCell(Right);
            Int32 ByRow = LeftCell.Row.CompareTo(RightCell.Row);
            if (ByRow != 0)
            {
                return ByRow;
            }
            return LeftCell.Col.CompareTo(RightCell.Col);
        }

        /// <summary>
        /// The member cell with the lowest row, then the lowest column.
        /// </summary>
        /// <param name="Cluster">Cluster to scan.</param>
        /// <returns>That cell.</returns>
        private static (Int32 Row, Int32 Col) SmallestCell
            (
            ResidualCluster Cluster
            )
        {
            Int32 Row = Int32.MaxValue;
            Int32 Col = Int32.MaxValue;
            foreach ((Int32 CellRow, Int32 CellCol) Cell in Cluster.Cells)
            {
                if ((Cell.CellRow < Row) || ((Cell.CellRow == Row) && (Cell.CellCol < Col)))
                {
                    Row = Cell.CellRow;
                    Col = Cell.CellCol;
                }
            }
            return (Row, Col);
        }

        /// <summary>
        /// Measures one component. Cells at or below the floor add nothing to the haul volume.
        /// </summary>
        /// <param name="Cells">Member cells, including bridged gaps.</param>
        /// <param name="DepthMm">Positive depth grid.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="CellAreaM2">Cell area in square metres.</param>
        /// <param name="BankFactor">Yard conversion factor for this sign.</param>
        /// <param name="Sign">"cut" or "fill".</param>
        /// <returns>The measured cluster. The id is assigned after the sort.</returns>
        private static ResidualCluster Measure
            (
            List<(Int32 Row, Int32 Col)> Cells,
            Double[,] DepthMm,
            Double CellSizeM,
            Double CellAreaM2,
            Double BankFactor,
            String Sign
            )
        {
            Double HaulM3 = 0.0;
            Double TotalM3 = 0.0;
            Double Peak = 0.0;
            Double DepthSum = 0.0;
            Double Weight = 0.0;
            Double WeightRow = 0.0;
            Double WeightCol = 0.0;
            Int32 MinRow = Int32.MaxValue;
            Int32 MinCol = Int32.MaxValue;
            Int32 MaxRow = 0;
            Int32 MaxCol = 0;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                Double Depth = DepthMm[Cell.Row, Cell.Col];
                if (Depth < 0.0)
                {
                    Depth = 0.0;
                }
                HaulM3 += Math.Max(Depth - FLOOR_MM, 0.0) / 1000.0 * CellAreaM2;
                TotalM3 += Depth / 1000.0 * CellAreaM2;
                DepthSum += Depth;
                if (Depth > Peak)
                {
                    Peak = Depth;
                }
                Weight += Depth;
                WeightRow += Depth * Cell.Row;
                WeightCol += Depth * Cell.Col;
                if (Cell.Row < MinRow)
                {
                    MinRow = Cell.Row;
                }
                if (Cell.Col < MinCol)
                {
                    MinCol = Cell.Col;
                }
                if (Cell.Row > MaxRow)
                {
                    MaxRow = Cell.Row;
                }
                if (Cell.Col > MaxCol)
                {
                    MaxCol = Cell.Col;
                }
            }
            Double CentroidRow;
            Double CentroidCol;
            if (Weight > 0.0)
            {
                CentroidRow = WeightRow / Weight;
                CentroidCol = WeightCol / Weight;
            }
            else
            {
                CentroidRow = Cells.Average(Cell => (Double)Cell.Row);
                CentroidCol = Cells.Average(Cell => (Double)Cell.Col);
            }
            (Double Degrees, Double LongM, Double ShortM) = Axes(Cells, DepthMm, CellSizeM);
            return new ResidualCluster(
                Sign,
                Cells,
                HaulM3,
                HaulM3 * M3_TO_CY * BankFactor,
                TotalM3 * M3_TO_CY * BankFactor,
                Peak,
                DepthSum / Cells.Count,
                CentroidRow,
                CentroidCol,
                Degrees,
                LongM,
                ShortM,
                MinRow,
                MinCol,
                MaxRow,
                MaxCol);
        }

        /// <summary>
        /// Principal axis of the cells, weighted by depth. The angle is clockwise
        /// from north and folded into a half turn because the axis has no direction.
        /// Lengths are the outer span of the cells along that axis.
        /// </summary>
        /// <param name="Cells">Member cells.</param>
        /// <param name="DepthMm">Positive depth grid.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <returns>Degrees, long-axis metres, and short-axis metres.</returns>
        private static (Double Degrees, Double LongM, Double ShortM) Axes
            (
            List<(Int32 Row, Int32 Col)> Cells,
            Double[,] DepthMm,
            Double CellSizeM
            )
        {
            (Double East, Double South) = LongAxis(Cells, DepthMm, CellSizeM);
            Double North = -South;
            Double Degrees = Math.Atan2(East, North) * (180.0 / Math.PI);
            if (Degrees < 0.0)
            {
                Degrees += 180.0;
            }
            if (Degrees >= 180.0)
            {
                Degrees -= 180.0;
            }
            Double LongSpan = Span(Cells, CellSizeM, East, South);
            Double ShortSpan = Span(Cells, CellSizeM, -South, East);
            return (Degrees, LongSpan, ShortSpan);
        }

        /// <summary>
        /// Unit vector of the depth-weighted long axis. East and south are the
        /// positive components. The opposite vector is the same undirected axis.
        /// </summary>
        /// <param name="Cells">Member cells.</param>
        /// <param name="DepthMm">Positive depth grid.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <returns>East and south components.</returns>
        private static (Double East, Double South) LongAxis
            (
            List<(Int32 Row, Int32 Col)> Cells,
            Double[,] DepthMm,
            Double CellSizeM
            )
        {
            Double Weight = 0.0;
            Double MeanX = 0.0;
            Double MeanY = 0.0;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                Double Depth = Math.Max(DepthMm[Cell.Row, Cell.Col], 0.0);
                Weight += Depth;
                MeanX += Depth * ((Cell.Col + 0.5) * CellSizeM);
                MeanY += Depth * ((Cell.Row + 0.5) * CellSizeM);
            }
            // A bridge of empty cells has no depth. The axis then uses each cell once.
            Boolean Unweighted = Weight <= 0.0;
            if (Unweighted)
            {
                Weight = Cells.Count;
                MeanX = 0.0;
                MeanY = 0.0;
                foreach ((Int32 Row, Int32 Col) Cell in Cells)
                {
                    MeanX += (Cell.Col + 0.5) * CellSizeM;
                    MeanY += (Cell.Row + 0.5) * CellSizeM;
                }
            }
            MeanX /= Weight;
            MeanY /= Weight;
            Double Cxx = 0.0;
            Double Cyy = 0.0;
            Double Cxy = 0.0;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                Double Depth = Unweighted ? 1.0 : Math.Max(DepthMm[Cell.Row, Cell.Col], 0.0);
                Double X = ((Cell.Col + 0.5) * CellSizeM) - MeanX;
                Double Y = ((Cell.Row + 0.5) * CellSizeM) - MeanY;
                Cxx += Depth * X * X;
                Cyy += Depth * Y * Y;
                Cxy += Depth * X * Y;
            }
            Cxx /= Weight;
            Cyy /= Weight;
            Cxy /= Weight;
            // The larger eigenvalue is the long axis. A square has no preferred axis, so east is used.
            Double HalfTrace = (Cxx + Cyy) * 0.5;
            Double Det = (Cxx * Cyy) - (Cxy * Cxy);
            Double Disc = Math.Sqrt(Math.Max(0.0, (HalfTrace * HalfTrace) - Det));
            Double Long = HalfTrace + Disc;
            Double East;
            Double South;
            if (Math.Abs(Cxy) > 1e-12)
            {
                East = Long - Cyy;
                South = Cxy;
            }
            else if (Cxx >= Cyy)
            {
                East = 1.0;
                South = 0.0;
            }
            else
            {
                East = 0.0;
                South = 1.0;
            }
            Double Norm = Math.Sqrt((East * East) + (South * South));
            East /= Norm;
            South /= Norm;
            return (East, South);
        }

        /// <summary>
        /// Splits one cluster into pieces of at most two bowls. A cluster already
        /// within the limit is returned as one piece under the parent id.
        /// </summary>
        /// <param name="Cluster">Parent cluster. Its cells are not changed.</param>
        /// <param name="DepthMm">Positive depth grid for this sign.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="CellAreaM2">Cell area in square metres.</param>
        /// <param name="BankFactor">1 for cut. The fill-to-bank ratio for fill.</param>
        /// <param name="LimitBcy">Two bowls, in bank cubic yards.</param>
        /// <returns>Pieces ordered along the parent axis from the parent's first cell.</returns>
        private static List<ResidualCluster> SplitCluster
            (
            ResidualCluster Cluster,
            Double[,] DepthMm,
            Double CellSizeM,
            Double CellAreaM2,
            Double BankFactor,
            Double LimitBcy
            )
        {
            List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>(Cluster.Cells);
            List<List<(Int32 Row, Int32 Col)>> Leaves = new List<List<(Int32 Row, Int32 Col)>>();
            Bisect(Cells, DepthMm, CellSizeM, CellAreaM2, BankFactor, Cluster.Sign, LimitBcy, Leaves);
            return NumberPieces(Cluster, Cells, Leaves, DepthMm, CellSizeM, CellAreaM2, BankFactor);
        }

        /// <summary>
        /// Recursively cuts a piece at its volume-weighted median until the haul
        /// is within two bowls. Each cut uses that piece's own long axis.
        /// </summary>
        /// <param name="Cells">Cells of the piece being cut.</param>
        /// <param name="DepthMm">Positive depth grid.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="CellAreaM2">Cell area in square metres.</param>
        /// <param name="BankFactor">Yard conversion factor for this sign.</param>
        /// <param name="Sign">"cut" or "fill".</param>
        /// <param name="LimitBcy">Two bowls, in bank cubic yards.</param>
        /// <param name="Leaves">Pieces that are within the limit are appended here.</param>
        private static void Bisect
            (
            List<(Int32 Row, Int32 Col)> Cells,
            Double[,] DepthMm,
            Double CellSizeM,
            Double CellAreaM2,
            Double BankFactor,
            String Sign,
            Double LimitBcy,
            List<List<(Int32 Row, Int32 Col)>> Leaves
            )
        {
            ResidualCluster Measured = Measure(Cells, DepthMm, CellSizeM, CellAreaM2, BankFactor, Sign);
            if ((Measured.HaulVolumeBcy <= LimitBcy) || (Cells.Count < 2))
            {
                Leaves.Add(Cells);
                return;
            }
            (List<(Int32 Row, Int32 Col)> Low, List<(Int32 Row, Int32 Col)> High) = CutAtMedian(
                Cells, DepthMm, CellSizeM, CellAreaM2);
            // A cell cannot be divided. Stop when the median does not make two smaller pieces.
            if ((Low.Count == 0) || (High.Count == 0) || (Low.Count >= Cells.Count) || (High.Count >= Cells.Count))
            {
                Leaves.Add(Cells);
                return;
            }
            Bisect(Low, DepthMm, CellSizeM, CellAreaM2, BankFactor, Sign, LimitBcy, Leaves);
            Bisect(High, DepthMm, CellSizeM, CellAreaM2, BankFactor, Sign, LimitBcy, Leaves);
        }

        /// <summary>
        /// Cuts cells into the two sides of the volume-weighted median.
        /// Position along the axis is the depth-weighted principal axis.
        /// The weight of the median is the haul above the floor, which is the volume the bowl limit uses.
        /// </summary>
        /// <param name="Cells">Cells to cut.</param>
        /// <param name="DepthMm">Positive depth grid.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="CellAreaM2">Cell area in square metres.</param>
        /// <returns>The low-projection side, then the high-projection side.</returns>
        private static (List<(Int32 Row, Int32 Col)> Low, List<(Int32 Row, Int32 Col)> High) CutAtMedian
            (
            List<(Int32 Row, Int32 Col)> Cells,
            Double[,] DepthMm,
            Double CellSizeM,
            Double CellAreaM2
            )
        {
            (Double East, Double South) = LongAxis(Cells, DepthMm, CellSizeM);
            List<(Int32 Row, Int32 Col, Double Projection, Double Haul)> Ordered =
                new List<(Int32 Row, Int32 Col, Double Projection, Double Haul)>(Cells.Count);
            Double Total = 0.0;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                Double Depth = Math.Max(DepthMm[Cell.Row, Cell.Col], 0.0);
                Double Haul = Math.Max(Depth - FLOOR_MM, 0.0) / 1000.0 * CellAreaM2;
                Double Projection = Project(Cell.Row, Cell.Col, East, South, CellSizeM);
                Total += Haul;
                Ordered.Add((Cell.Row, Cell.Col, Projection, Haul));
            }
            Ordered.Sort(CompareMedian);
            Double Half = Total * 0.5;
            Double Cumulative = 0.0;
            Int32 Cut = 0;
            for (Int32 Index = 0; Index < Ordered.Count; Index += 1)
            {
                Cumulative += Ordered[Index].Haul;
                Cut = Index;
                if ((Cumulative + (Total * MEDIAN_SLACK)) >= Half)
                {
                    break;
                }
            }
            List<(Int32 Row, Int32 Col)> Low = new List<(Int32 Row, Int32 Col)>();
            List<(Int32 Row, Int32 Col)> High = new List<(Int32 Row, Int32 Col)>();
            for (Int32 Index = 0; Index < Ordered.Count; Index += 1)
            {
                (Int32 Row, Int32 Col) Cell = (Ordered[Index].Row, Ordered[Index].Col);
                if (Index <= Cut)
                {
                    Low.Add(Cell);
                }
                else
                {
                    High.Add(Cell);
                }
            }
            return (Low, High);
        }

        /// <summary>
        /// Orders median cells by projection, then by row, then by column.
        /// </summary>
        /// <param name="Left">First cell.</param>
        /// <param name="Right">Second cell.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareMedian
            (
            (Int32 Row, Int32 Col, Double Projection, Double Haul) Left,
            (Int32 Row, Int32 Col, Double Projection, Double Haul) Right
            )
        {
            Int32 ByProjection = Left.Projection.CompareTo(Right.Projection);
            if (ByProjection != 0)
            {
                return ByProjection;
            }
            Int32 ByRow = Left.Row.CompareTo(Right.Row);
            if (ByRow != 0)
            {
                return ByRow;
            }
            return Left.Col.CompareTo(Right.Col);
        }

        /// <summary>
        /// Measures the leaves and numbers them from the parent end nearest the
        /// parent's first cell. The first cell is the lowest row, then the lowest column.
        /// One leaf keeps the parent id. Several leaves are parent.1, parent.2, and so on.
        /// </summary>
        /// <param name="Parent">Cluster the leaves were cut from.</param>
        /// <param name="ParentCells">Parent cells, used for the ordering axis.</param>
        /// <param name="Leaves">Pieces produced by the bisection.</param>
        /// <param name="DepthMm">Positive depth grid.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="CellAreaM2">Cell area in square metres.</param>
        /// <param name="BankFactor">Yard conversion factor for this sign.</param>
        /// <returns>The numbered pieces.</returns>
        private static List<ResidualCluster> NumberPieces
            (
            ResidualCluster Parent,
            List<(Int32 Row, Int32 Col)> ParentCells,
            List<List<(Int32 Row, Int32 Col)>> Leaves,
            Double[,] DepthMm,
            Double CellSizeM,
            Double CellAreaM2,
            Double BankFactor
            )
        {
            List<ResidualCluster> Pieces = new List<ResidualCluster>(Leaves.Count);
            foreach (List<(Int32 Row, Int32 Col)> Leaf in Leaves)
            {
                Pieces.Add(Measure(Leaf, DepthMm, CellSizeM, CellAreaM2, BankFactor, Parent.Sign));
            }
            if (Pieces.Count == 1)
            {
                Pieces[0].AssignId(Parent.Id);
                return Pieces;
            }
            (Double East, Double South) = LongAxis(ParentCells, DepthMm, CellSizeM);
            (Int32 Row, Int32 Col) First = SmallestCell(Parent);
            Double FirstProjection = Project(First.Row, First.Col, East, South, CellSizeM);
            Double Low = Double.PositiveInfinity;
            Double High = Double.NegativeInfinity;
            foreach ((Int32 Row, Int32 Col) Cell in ParentCells)
            {
                Double Projection = Project(Cell.Row, Cell.Col, East, South, CellSizeM);
                if (Projection < Low)
                {
                    Low = Projection;
                }
                if (Projection > High)
                {
                    High = Projection;
                }
            }
            // The nearer end of the parent axis is where piece 1 starts.
            Boolean FromLowEnd = (FirstProjection - Low) <= (High - FirstProjection);
            Pieces.Sort((ResidualCluster Left, ResidualCluster Right) =>
                ComparePieces(Left, Right, East, South, CellSizeM, FromLowEnd));
            for (Int32 Index = 0; Index < Pieces.Count; Index += 1)
            {
                String Number = (Index + 1).ToString(CultureInfo.InvariantCulture);
                Pieces[Index].AssignId(Parent.Id + "." + Number);
            }
            return Pieces;
        }

        /// <summary>
        /// Orders two pieces along the parent axis. Ties use the lowest cell.
        /// </summary>
        /// <param name="Left">First piece.</param>
        /// <param name="Right">Second piece.</param>
        /// <param name="East">East component of the parent axis.</param>
        /// <param name="South">South component of the parent axis.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="FromLowEnd">True when piece 1 is at the low-projection end.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 ComparePieces
            (
            ResidualCluster Left,
            ResidualCluster Right,
            Double East,
            Double South,
            Double CellSizeM,
            Boolean FromLowEnd
            )
        {
            Double LeftProjection = Project(Left.CentroidRow, Left.CentroidCol, East, South, CellSizeM);
            Double RightProjection = Project(Right.CentroidRow, Right.CentroidCol, East, South, CellSizeM);
            Int32 ByAxis = FromLowEnd
                ? LeftProjection.CompareTo(RightProjection)
                : RightProjection.CompareTo(LeftProjection);
            if (ByAxis != 0)
            {
                return ByAxis;
            }
            (Int32 Row, Int32 Col) LeftCell = SmallestCell(Left);
            (Int32 Row, Int32 Col) RightCell = SmallestCell(Right);
            Int32 ByRow = LeftCell.Row.CompareTo(RightCell.Row);
            if (ByRow != 0)
            {
                return ByRow;
            }
            return LeftCell.Col.CompareTo(RightCell.Col);
        }

        /// <summary>
        /// Projects a cell-index point onto an axis. The half-cell shift uses the cell centre.
        /// </summary>
        /// <param name="Row">Row index. A centroid may be fractional.</param>
        /// <param name="Col">Column index. A centroid may be fractional.</param>
        /// <param name="East">East component of the unit axis.</param>
        /// <param name="South">South component of the unit axis.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <returns>The projection in metres.</returns>
        private static Double Project
            (
            Double Row,
            Double Col,
            Double East,
            Double South,
            Double CellSizeM
            )
        {
            Double X = (Col + 0.5) * CellSizeM;
            Double Y = (Row + 0.5) * CellSizeM;
            return (X * East) + (Y * South);
        }

        /// <summary>
        /// Outer length of the cells along a unit direction, in metres.
        /// One cell is one cell wide.
        /// </summary>
        /// <param name="Cells">Member cells.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="East">East component of the unit direction.</param>
        /// <param name="South">South component of the unit direction. South is increasing row.</param>
        /// <returns>The span in metres.</returns>
        private static Double Span
            (
            List<(Int32 Row, Int32 Col)> Cells,
            Double CellSizeM,
            Double East,
            Double South
            )
        {
            Double Low = Double.PositiveInfinity;
            Double High = Double.NegativeInfinity;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                Double X = (Cell.Col + 0.5) * CellSizeM;
                Double Y = (Cell.Row + 0.5) * CellSizeM;
                Double Projected = (X * East) + (Y * South);
                if (Projected < Low)
                {
                    Low = Projected;
                }
                if (Projected > High)
                {
                    High = Projected;
                }
            }
            return (High - Low) + CellSizeM;
        }

        /// <summary>
        /// Haul volume of cells that are not in a kept cluster.
        /// </summary>
        /// <param name="DepthMm">Positive depth grid.</param>
        /// <param name="Kept">Clusters that were kept for this sign.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="CellAreaM2">Cell area in square metres.</param>
        /// <param name="BankFactor">Yard conversion factor for this sign.</param>
        /// <returns>Bank cubic yards above the floor.</returns>
        private static Double OutsideHaul
            (
            Double[,] DepthMm,
            List<ResidualCluster> Kept,
            Int32 Rows,
            Int32 Cols,
            Double CellAreaM2,
            Double BankFactor
            )
        {
            Boolean[,] Inside = new Boolean[Rows, Cols];
            foreach (ResidualCluster Cluster in Kept)
            {
                foreach ((Int32 Row, Int32 Col) Cell in Cluster.Cells)
                {
                    Inside[Cell.Row, Cell.Col] = true;
                }
            }
            Double M3 = 0.0;
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (Inside[Row, Col])
                    {
                        continue;
                    }
                    M3 += Math.Max(DepthMm[Row, Col] - FLOOR_MM, 0.0) / 1000.0 * CellAreaM2;
                }
            }
            return M3 * M3_TO_CY * BankFactor;
        }

        /// <summary>
        /// Sums haul volume over a cluster list.
        /// </summary>
        /// <param name="Clusters">Clusters of one sign.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double SumHaul
            (
            List<ResidualCluster> Clusters
            )
        {
            Double Sum = 0.0;
            foreach (ResidualCluster Cluster in Clusters)
            {
                Sum += Cluster.HaulVolumeBcy;
            }
            return Sum;
        }

        /// <summary>
        /// Keeps cells above the floor that touch a cell above the deep threshold
        /// through other cells above the floor. The neighbourhood is 8-connected.
        /// </summary>
        /// <param name="DepthMm">Positive depth grid.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <returns>The kept mask.</returns>
        private static Boolean[,] Hysteresis
            (
            Double[,] DepthMm,
            Int32 Rows,
            Int32 Cols
            )
        {
            Boolean[,] Low = new Boolean[Rows, Cols];
            Boolean[,] High = new Boolean[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    Double Depth = DepthMm[Row, Col];
                    if (Depth > FLOOR_MM)
                    {
                        Low[Row, Col] = true;
                    }
                    if (Depth > DEEP_MM)
                    {
                        High[Row, Col] = true;
                    }
                }
            }
            List<List<(Int32 Row, Int32 Col)>> Components = Label(Low, Rows, Cols);
            Boolean[,] Kept = new Boolean[Rows, Cols];
            foreach (List<(Int32 Row, Int32 Col)> Cells in Components)
            {
                Boolean Seeded = false;
                foreach ((Int32 Row, Int32 Col) Cell in Cells)
                {
                    if (High[Cell.Row, Cell.Col])
                    {
                        Seeded = true;
                        break;
                    }
                }
                if (!Seeded)
                {
                    continue;
                }
                foreach ((Int32 Row, Int32 Col) Cell in Cells)
                {
                    Kept[Cell.Row, Cell.Col] = true;
                }
            }
            return Kept;
        }

        /// <summary>
        /// Closes a mask with a 3 by 3 square. One iteration bridges a one-cell gap.
        /// </summary>
        /// <param name="Mask">Input mask.</param>
        /// <param name="Iterations">Dilations, followed by the same number of erosions.</param>
        /// <returns>The closed mask.</returns>
        private static Boolean[,] Close
            (
            Boolean[,] Mask,
            Int32 Iterations
            )
        {
            Boolean[,] Current = Mask;
            for (Int32 Step = 0; Step < Iterations; Step += 1)
            {
                Current = Dilate(Current);
            }
            for (Int32 Step = 0; Step < Iterations; Step += 1)
            {
                Current = Erode(Current, 1);
            }
            return Current;
        }

        /// <summary>
        /// Erodes a mask with a 3 by 3 square. Cells outside the grid are empty.
        /// </summary>
        /// <param name="Mask">Input mask.</param>
        /// <param name="Iterations">How many times to erode.</param>
        /// <returns>The eroded mask.</returns>
        private static Boolean[,] Erode
            (
            Boolean[,] Mask,
            Int32 Iterations
            )
        {
            Boolean[,] Current = Mask;
            Int32 Rows = Mask.GetLength(0);
            Int32 Cols = Mask.GetLength(1);
            for (Int32 Step = 0; Step < Iterations; Step += 1)
            {
                Boolean[,] Next = new Boolean[Rows, Cols];
                for (Int32 Row = 0; Row < Rows; Row += 1)
                {
                    for (Int32 Col = 0; Col < Cols; Col += 1)
                    {
                        Boolean Keep = true;
                        for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                        {
                            for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                            {
                                Int32 Ny = Row + Dy;
                                Int32 Nx = Col + Dx;
                                if ((Ny < 0) || (Nx < 0) || (Ny >= Rows) || (Nx >= Cols) || !Current[Ny, Nx])
                                {
                                    Keep = false;
                                }
                            }
                        }
                        Next[Row, Col] = Keep;
                    }
                }
                Current = Next;
            }
            return Current;
        }

        /// <summary>
        /// Dilates a mask with a 3 by 3 square. Cells outside the grid are empty.
        /// </summary>
        /// <param name="Mask">Input mask.</param>
        /// <returns>The dilated mask.</returns>
        private static Boolean[,] Dilate
            (
            Boolean[,] Mask
            )
        {
            Int32 Rows = Mask.GetLength(0);
            Int32 Cols = Mask.GetLength(1);
            Boolean[,] Next = new Boolean[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (!Mask[Row, Col])
                    {
                        continue;
                    }
                    for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                    {
                        for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                        {
                            Int32 Ny = Row + Dy;
                            Int32 Nx = Col + Dx;
                            if ((Ny >= 0) && (Nx >= 0) && (Ny < Rows) && (Nx < Cols))
                            {
                                Next[Ny, Nx] = true;
                            }
                        }
                    }
                }
            }
            return Next;
        }

        /// <summary>
        /// Labels 8-connected true cells. Each returned list is one component.
        /// </summary>
        /// <param name="Mask">Input mask.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <returns>The components. Labels are not reused across calls.</returns>
        private static List<List<(Int32 Row, Int32 Col)>> Label
            (
            Boolean[,] Mask,
            Int32 Rows,
            Int32 Cols
            )
        {
            Boolean[,] Seen = new Boolean[Rows, Cols];
            List<List<(Int32 Row, Int32 Col)>> Components = new List<List<(Int32 Row, Int32 Col)>>();
            Int32[] Dy = { -1, -1, -1, 0, 0, 1, 1, 1 };
            Int32[] Dx = { -1, 0, 1, -1, 1, -1, 0, 1 };
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (!Mask[Row, Col] || Seen[Row, Col])
                    {
                        continue;
                    }
                    List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>();
                    Queue<(Int32 Row, Int32 Col)> Queue = new Queue<(Int32 Row, Int32 Col)>();
                    Queue.Enqueue((Row, Col));
                    Seen[Row, Col] = true;
                    while (Queue.Count > 0)
                    {
                        (Int32 Row, Int32 Col) Cell = Queue.Dequeue();
                        Cells.Add(Cell);
                        for (Int32 Neighbour = 0; Neighbour < CONNECTIVITY; Neighbour += 1)
                        {
                            Int32 Ny = Cell.Row + Dy[Neighbour];
                            Int32 Nx = Cell.Col + Dx[Neighbour];
                            if ((Ny < 0) || (Nx < 0) || (Ny >= Rows) || (Nx >= Cols))
                            {
                                continue;
                            }
                            if (!Mask[Ny, Nx] || Seen[Ny, Nx])
                            {
                                continue;
                            }
                            Seen[Ny, Nx] = true;
                            Queue.Enqueue((Ny, Nx));
                        }
                    }
                    Components.Add(Cells);
                }
            }
            return Components;
        }

        /// <summary>
        /// Puts a label near its centroid without covering a label already drawn.
        /// The walk starts on the preferred point and moves outward by one step at a time.
        /// </summary>
        /// <param name="Plates">Plates already drawn.</param>
        /// <param name="PreferredX">Left edge wanted for this plate.</param>
        /// <param name="PreferredY">Top edge wanted for this plate.</param>
        /// <param name="Width">Plate width.</param>
        /// <param name="Height">Plate height.</param>
        /// <param name="SheetWidth">Image width.</param>
        /// <param name="SheetHeight">Image height.</param>
        /// <returns>The left and top of a free plate, or the preferred point clamped to the sheet.</returns>
        private static (Int32 X, Int32 Y) PlacePlate
            (
            List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)> Plates,
            Int32 PreferredX,
            Int32 PreferredY,
            Int32 Width,
            Int32 Height,
            Int32 SheetWidth,
            Int32 SheetHeight
            )
        {
            const Int32 Step = 8;
            const Int32 MaxRadius = 480;
            for (Int32 Radius = 0; Radius <= MaxRadius; Radius += Step)
            {
                if (Radius == 0)
                {
                    if (PlateFits(Plates, PreferredX, PreferredY, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX, PreferredY);
                    }
                    continue;
                }
                for (Int32 Along = -Radius; Along <= Radius; Along += Step)
                {
                    if (PlateFits(Plates, PreferredX + Along, PreferredY - Radius, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX + Along, PreferredY - Radius);
                    }
                    if (PlateFits(Plates, PreferredX + Along, PreferredY + Radius, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX + Along, PreferredY + Radius);
                    }
                    if (PlateFits(Plates, PreferredX - Radius, PreferredY + Along, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX - Radius, PreferredY + Along);
                    }
                    if (PlateFits(Plates, PreferredX + Radius, PreferredY + Along, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX + Radius, PreferredY + Along);
                    }
                }
            }
            Int32 X = Math.Clamp(PreferredX, 2, Math.Max(2, SheetWidth - Width - 2));
            Int32 Y = Math.Clamp(PreferredY, 2, Math.Max(2, SheetHeight - Height - 2));
            return (X, Y);
        }

        /// <summary>
        /// True when the plate lies on the sheet and misses every plate already drawn.
        /// </summary>
        /// <param name="Plates">Plates already drawn.</param>
        /// <param name="X">Left of the candidate.</param>
        /// <param name="Y">Top of the candidate.</param>
        /// <param name="Width">Candidate width.</param>
        /// <param name="Height">Candidate height.</param>
        /// <param name="SheetWidth">Image width.</param>
        /// <param name="SheetHeight">Image height.</param>
        /// <returns>True when the candidate can be drawn.</returns>
        private static Boolean PlateFits
            (
            List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)> Plates,
            Int32 X,
            Int32 Y,
            Int32 Width,
            Int32 Height,
            Int32 SheetWidth,
            Int32 SheetHeight
            )
        {
            if ((X < 2) || (Y < 2) || ((X + Width) > (SheetWidth - 2)) || ((Y + Height) > (SheetHeight - 2)))
            {
                return false;
            }
            return !Overlaps(Plates, X, Y, Width, Height);
        }

        /// <summary>
        /// True when a new label plate would cover one that is already drawn.
        /// </summary>
        /// <param name="Plates">Plates already drawn.</param>
        /// <param name="X">Left of the new plate.</param>
        /// <param name="Y">Top of the new plate.</param>
        /// <param name="Width">Width of the new plate.</param>
        /// <param name="Height">Height of the new plate.</param>
        /// <returns>True when the rectangles meet.</returns>
        private static Boolean Overlaps
            (
            List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)> Plates,
            Int32 X,
            Int32 Y,
            Int32 Width,
            Int32 Height
            )
        {
            foreach ((Int32 X, Int32 Y, Int32 Width, Int32 Height) Plate in Plates)
            {
                // Two pixels of air keeps neighbouring plates from touching.
                const Int32 Gap = 2;
                Boolean Clear = ((X + Width + Gap) <= Plate.X) || ((Plate.X + Plate.Width + Gap) <= X)
                    || ((Y + Height + Gap) <= Plate.Y) || ((Plate.Y + Plate.Height + Gap) <= Y);
                if (!Clear)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Reads a label grid. Cells outside the grid belong to no cluster.
        /// </summary>
        /// <param name="Owner">Cluster index plus one. Zero is unclustered.</param>
        /// <param name="Row">Row.</param>
        /// <param name="Col">Column.</param>
        /// <returns>The owner, or zero outside the grid.</returns>
        private static Int32 OwnerOf
            (
            Int32[,] Owner,
            Int32 Row,
            Int32 Col
            )
        {
            if ((Row < 0) || (Col < 0) || (Row >= Owner.GetLength(0)) || (Col >= Owner.GetLength(1)))
            {
                return 0;
            }
            return Owner[Row, Col];
        }

        /// <summary>
        /// Fills a rectangle, clipped to the image.
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
            Int32 X1 = Math.Min(Sheet.Width, X0 + Width);
            Int32 Y1 = Math.Min(Sheet.Height, Y0 + Height);
            Int32 XStart = Math.Max(0, X0);
            Int32 YStart = Math.Max(0, Y0);
            for (Int32 Y = YStart; Y < Y1; Y += 1)
            {
                for (Int32 X = XStart; X < X1; X += 1)
                {
                    Sheet[X, Y] = Color;
                }
            }
        }

        /// <summary>
        /// Writes a label in a 5 by 7 face.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X0">Left pixel of the first character.</param>
        /// <param name="Y0">Top pixel of the first character.</param>
        /// <param name="Label">Text. Unknown characters are blank.</param>
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
                String[] Rows = Glyph(Letter);
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
        /// <returns>Seven rows.</returns>
        private static String[] Glyph
            (
            Char Letter
            )
        {
            switch (Letter)
            {
                case 'C': return new[] { "01110", "10001", "10000", "10000", "10000", "10001", "01110" };
                case 'F': return new[] { "11111", "10000", "10000", "11110", "10000", "10000", "10000" };
                case '0': return new[] { "01110", "10001", "10011", "10101", "11001", "10001", "01110" };
                case '1': return new[] { "00100", "01100", "00100", "00100", "00100", "00100", "01110" };
                case '2': return new[] { "01110", "10001", "00001", "00010", "00100", "01000", "11111" };
                case '3': return new[] { "11110", "00001", "00001", "01110", "00001", "00001", "11110" };
                case '4': return new[] { "00010", "00110", "01010", "10010", "11111", "00010", "00010" };
                case '5': return new[] { "11111", "10000", "11110", "00001", "00001", "10001", "01110" };
                case '6': return new[] { "01110", "10000", "10000", "11110", "10001", "10001", "01110" };
                case '7': return new[] { "11111", "00001", "00010", "00100", "01000", "01000", "01000" };
                case '8': return new[] { "01110", "10001", "10001", "01110", "10001", "10001", "01110" };
                case '9': return new[] { "01110", "10001", "10001", "01111", "00001", "00001", "01110" };
                case '.': return new[] { "00000", "00000", "00000", "00000", "00000", "01100", "01100" };
                case ' ': return new[] { "00000", "00000", "00000", "00000", "00000", "00000", "00000" };
                default: return new[] { "00000", "00000", "00000", "00000", "00000", "00000", "00000" };
            }
        }
    }

    /// <summary>
    /// The clusters found on one residual grid, the pieces cut from them, and the counts that explain them.
    /// </summary>
    public sealed class ResidualClusterReport
    {
        /// <summary>
        /// Creates a report.
        /// </summary>
        /// <param name="Clusters">Kept clusters, cuts first, then fills.</param>
        /// <param name="Pieces">Pieces of those clusters, in cluster order, then along each parent axis.</param>
        /// <param name="Diagnostics">The required counts.</param>
        public ResidualClusterReport
            (
            IReadOnlyList<ResidualCluster> Clusters,
            IReadOnlyList<ResidualCluster> Pieces,
            ResidualClusterDiagnostics Diagnostics
            )
        {
            this.Clusters = Clusters;
            this.Pieces = Pieces;
            this.Diagnostics = Diagnostics;
        }

        /// <summary>Gets the kept clusters. Cut ids come before fill ids.</summary>
        [JsonPropertyName("clusters")]
        public IReadOnlyList<ResidualCluster> Clusters { get; }

        /// <summary>Gets the pieces. A cluster within two bowls contributes one piece under the parent id.</summary>
        [JsonPropertyName("pieces")]
        public IReadOnlyList<ResidualCluster> Pieces { get; }

        /// <summary>Gets the diagnostic counts.</summary>
        [JsonIgnore]
        public ResidualClusterDiagnostics Diagnostics { get; }

        /// <summary>
        /// Writes the deep-cell counts, the cluster count, and the haul totals.
        /// </summary>
        /// <param name="Writer">Where to print.</param>
        public void Print
            (
            TextWriter Writer
            )
        {
            if (Writer == null) throw new ArgumentNullException(nameof(Writer));
            Writer.WriteLine(
                "cut deep cells: {0}",
                this.Diagnostics.DeepCutAfterBuffer);
            Writer.WriteLine(
                "fill deep cells: {0}",
                this.Diagnostics.DeepFillAfterBuffer);
            Writer.WriteLine("clusters: {0}", this.Diagnostics.ClusterCount);
            Writer.WriteLine("pieces: {0}", this.Pieces.Count);
            Writer.WriteLine(
                "cut haul in clusters: {0:R} bcy",
                this.Diagnostics.CutHaulBcy);
            Writer.WriteLine(
                "fill haul in clusters: {0:R} bcy",
                this.Diagnostics.FillHaulBcy);
            Writer.WriteLine(
                "cut haul not in a cluster: {0:R} bcy",
                this.Diagnostics.CutHaulOutsideBcy);
            Writer.WriteLine(
                "fill haul not in a cluster: {0:R} bcy",
                this.Diagnostics.FillHaulOutsideBcy);
        }
    }

    /// <summary>
    /// Counts required beside the cluster list.
    /// </summary>
    public sealed class ResidualClusterDiagnostics
    {
        /// <summary>
        /// Creates the counts.
        /// </summary>
        /// <param name="DeepCutBeforeBuffer">Cut cells deeper than the seed threshold on the whole field.</param>
        /// <param name="DeepCutAfterBuffer">Cut cells deeper than the seed threshold that clustering can use.</param>
        /// <param name="DeepFillBeforeBuffer">Fill cells deeper than the seed threshold on the whole field.</param>
        /// <param name="DeepFillAfterBuffer">Fill cells deeper than the seed threshold that clustering can use.</param>
        /// <param name="ClusterCount">Kept clusters.</param>
        /// <param name="CutHaulBcy">Cut haul volume in kept clusters.</param>
        /// <param name="FillHaulBcy">Fill haul volume in kept clusters.</param>
        /// <param name="CutHaulOutsideBcy">Cut haul volume in cells that are not in a kept cluster.</param>
        /// <param name="FillHaulOutsideBcy">Fill haul volume in cells that are not in a kept cluster.</param>
        public ResidualClusterDiagnostics
            (
            Int32 DeepCutBeforeBuffer,
            Int32 DeepCutAfterBuffer,
            Int32 DeepFillBeforeBuffer,
            Int32 DeepFillAfterBuffer,
            Int32 ClusterCount,
            Double CutHaulBcy,
            Double FillHaulBcy,
            Double CutHaulOutsideBcy,
            Double FillHaulOutsideBcy
            )
        {
            this.DeepCutBeforeBuffer = DeepCutBeforeBuffer;
            this.DeepCutAfterBuffer = DeepCutAfterBuffer;
            this.DeepFillBeforeBuffer = DeepFillBeforeBuffer;
            this.DeepFillAfterBuffer = DeepFillAfterBuffer;
            this.ClusterCount = ClusterCount;
            this.CutHaulBcy = CutHaulBcy;
            this.FillHaulBcy = FillHaulBcy;
            this.CutHaulOutsideBcy = CutHaulOutsideBcy;
            this.FillHaulOutsideBcy = FillHaulOutsideBcy;
        }

        /// <summary>Gets valid cut cells deeper than 150 mm.</summary>
        public Int32 DeepCutBeforeBuffer { get; }

        /// <summary>Gets valid cut cells deeper than 150 mm that clustering can use.</summary>
        public Int32 DeepCutAfterBuffer { get; }

        /// <summary>Gets valid fill cells deeper than 150 mm.</summary>
        public Int32 DeepFillBeforeBuffer { get; }

        /// <summary>Gets valid fill cells deeper than 150 mm that clustering can use.</summary>
        public Int32 DeepFillAfterBuffer { get; }

        /// <summary>Gets the number of kept clusters.</summary>
        public Int32 ClusterCount { get; }

        /// <summary>Gets cut haul volume in kept clusters, in bank cubic yards.</summary>
        public Double CutHaulBcy { get; }

        /// <summary>Gets fill haul volume in kept clusters, in bank cubic yards.</summary>
        public Double FillHaulBcy { get; }

        /// <summary>Gets cut haul volume that is not in a kept cluster, in bank cubic yards.</summary>
        public Double CutHaulOutsideBcy { get; }

        /// <summary>Gets fill haul volume that is not in a kept cluster, in bank cubic yards.</summary>
        public Double FillHaulOutsideBcy { get; }
    }

    /// <summary>
    /// One kept cut or fill cluster.
    /// </summary>
    public sealed class ResidualCluster
    {
        /// <summary>
        /// Creates a cluster. The id is filled in after the volume sort.
        /// </summary>
        /// <param name="Sign">"cut" or "fill".</param>
        /// <param name="Cells">Member cells, row then column.</param>
        /// <param name="HaulVolumeM3">Volume above the floor, in cubic metres.</param>
        /// <param name="HaulVolumeBcy">Haul volume in bank cubic yards.</param>
        /// <param name="TotalVolumeBcy">Full depth volume in bank cubic yards.</param>
        /// <param name="PeakMm">Deepest cell, in millimetres.</param>
        /// <param name="MeanMm">Mean depth of the member cells, in millimetres.</param>
        /// <param name="CentroidRow">Volume-weighted row.</param>
        /// <param name="CentroidCol">Volume-weighted column.</param>
        /// <param name="LongAxisDeg">Long-axis bearing, clockwise from north, in [0, 180).</param>
        /// <param name="LongAxisLenM">Long-axis length in metres.</param>
        /// <param name="ShortAxisLenM">Short-axis length in metres.</param>
        /// <param name="MinRow">Smallest member row.</param>
        /// <param name="MinCol">Smallest member column on the smallest row's cluster. The minimum column of the component.</param>
        /// <param name="MaxRow">Largest member row.</param>
        /// <param name="MaxCol">Largest member column.</param>
        public ResidualCluster
            (
            String Sign,
            List<(Int32 Row, Int32 Col)> Cells,
            Double HaulVolumeM3,
            Double HaulVolumeBcy,
            Double TotalVolumeBcy,
            Double PeakMm,
            Double MeanMm,
            Double CentroidRow,
            Double CentroidCol,
            Double LongAxisDeg,
            Double LongAxisLenM,
            Double ShortAxisLenM,
            Int32 MinRow,
            Int32 MinCol,
            Int32 MaxRow,
            Int32 MaxCol
            )
        {
            this.Sign = Sign;
            this.Cells = Cells;
            this.HaulVolumeM3 = HaulVolumeM3;
            this.HaulVolumeBcy = HaulVolumeBcy;
            this.TotalVolumeBcy = TotalVolumeBcy;
            this.PeakMm = PeakMm;
            this.MeanMm = MeanMm;
            this.CentroidRow = CentroidRow;
            this.CentroidCol = CentroidCol;
            this.LongAxisDeg = LongAxisDeg;
            this.LongAxisLenM = LongAxisLenM;
            this.ShortAxisLenM = ShortAxisLenM;
            this.MinRow = MinRow;
            this.MinCol = MinCol;
            this.MaxRow = MaxRow;
            this.MaxCol = MaxCol;
            this.Id = "";
        }

        /// <summary>Gets the id. C1, C2 for cut. F1, F2 for fill.</summary>
        [JsonPropertyName("id")]
        public String Id { get; private set; }

        /// <summary>Gets "cut" or "fill".</summary>
        [JsonPropertyName("sign")]
        public String Sign { get; }

        /// <summary>Gets the member cells.</summary>
        [JsonIgnore]
        public IReadOnlyList<(Int32 Row, Int32 Col)> Cells { get; }

        /// <summary>Gets the cell count.</summary>
        [JsonPropertyName("n_cells")]
        public Int32 CellCount
        {
            get { return this.Cells.Count; }
        }

        /// <summary>Gets the haul volume in cubic metres, before the yard conversion.</summary>
        [JsonIgnore]
        public Double HaulVolumeM3 { get; }

        /// <summary>Gets the haul volume in bank cubic yards.</summary>
        [JsonPropertyName("haul_volume_bcy")]
        public Double HaulVolumeBcy { get; }

        /// <summary>Gets the full depth volume in bank cubic yards.</summary>
        [JsonPropertyName("total_volume_bcy")]
        public Double TotalVolumeBcy { get; }

        /// <summary>Gets the deepest cell in millimetres.</summary>
        [JsonPropertyName("peak_mm")]
        public Double PeakMm { get; }

        /// <summary>Gets the mean depth of the member cells in millimetres.</summary>
        [JsonPropertyName("mean_mm")]
        public Double MeanMm { get; }

        /// <summary>Gets the volume-weighted row.</summary>
        [JsonIgnore]
        public Double CentroidRow { get; }

        /// <summary>Gets the volume-weighted column.</summary>
        [JsonIgnore]
        public Double CentroidCol { get; }

        /// <summary>Gets the centroid as row and column.</summary>
        [JsonPropertyName("centroid")]
        public ResidualClusterPoint Centroid
        {
            get { return new ResidualClusterPoint(this.CentroidRow, this.CentroidCol); }
        }

        /// <summary>Gets the long-axis bearing in degrees, clockwise from north.</summary>
        [JsonPropertyName("long_axis_deg")]
        public Double LongAxisDeg { get; }

        /// <summary>Gets the long-axis length in metres.</summary>
        [JsonPropertyName("long_axis_len_m")]
        public Double LongAxisLenM { get; }

        /// <summary>Gets the short-axis length in metres.</summary>
        [JsonPropertyName("short_axis_len_m")]
        public Double ShortAxisLenM { get; }

        /// <summary>Gets the smallest member row. Used to break volume ties.</summary>
        [JsonIgnore]
        public Int32 MinRow { get; }

        /// <summary>Gets the smallest member column.</summary>
        [JsonIgnore]
        public Int32 MinCol { get; }

        /// <summary>Gets the largest member row.</summary>
        [JsonIgnore]
        public Int32 MaxRow { get; }

        /// <summary>Gets the largest member column.</summary>
        [JsonIgnore]
        public Int32 MaxCol { get; }

        /// <summary>Gets the inclusive bounding box.</summary>
        [JsonPropertyName("bbox")]
        public ResidualClusterBox Bbox
        {
            get { return new ResidualClusterBox(this.MinRow, this.MinCol, this.MaxRow, this.MaxCol); }
        }

        /// <summary>Gets the member cells as row, column pairs.</summary>
        [JsonPropertyName("cells")]
        public Int32[][] CellList
        {
            get
            {
                Int32[][] List = new Int32[this.Cells.Count][];
                for (Int32 Index = 0; Index < this.Cells.Count; Index += 1)
                {
                    List[Index] = new[] { this.Cells[Index].Row, this.Cells[Index].Col };
                }
                return List;
            }
        }

        /// <summary>
        /// Sets the id after the clusters have been ordered.
        /// </summary>
        /// <param name="Id">C1 or F1 and so on.</param>
        public void AssignId
            (
            String Id
            )
        {
            this.Id = Id;
        }
    }

    /// <summary>
    /// A row and column. The centroid uses fractional indexes.
    /// </summary>
    public sealed class ResidualClusterPoint
    {
        /// <summary>
        /// Creates a point.
        /// </summary>
        /// <param name="Row">Row.</param>
        /// <param name="Col">Column.</param>
        public ResidualClusterPoint
            (
            Double Row,
            Double Col
            )
        {
            this.Row = Row;
            this.Col = Col;
        }

        /// <summary>Gets the row.</summary>
        [JsonPropertyName("row")]
        public Double Row { get; }

        /// <summary>Gets the column.</summary>
        [JsonPropertyName("col")]
        public Double Col { get; }
    }

    /// <summary>
    /// Inclusive cell bounds of a cluster.
    /// </summary>
    public sealed class ResidualClusterBox
    {
        /// <summary>
        /// Creates a box.
        /// </summary>
        /// <param name="Row0">Smallest row.</param>
        /// <param name="Col0">Smallest column.</param>
        /// <param name="Row1">Largest row.</param>
        /// <param name="Col1">Largest column.</param>
        public ResidualClusterBox
            (
            Int32 Row0,
            Int32 Col0,
            Int32 Row1,
            Int32 Col1
            )
        {
            this.Row0 = Row0;
            this.Col0 = Col0;
            this.Row1 = Row1;
            this.Col1 = Col1;
        }

        /// <summary>Gets the smallest row.</summary>
        [JsonPropertyName("row0")]
        public Int32 Row0 { get; }

        /// <summary>Gets the smallest column.</summary>
        [JsonPropertyName("col0")]
        public Int32 Col0 { get; }

        /// <summary>Gets the largest row.</summary>
        [JsonPropertyName("row1")]
        public Int32 Row1 { get; }

        /// <summary>Gets the largest column.</summary>
        [JsonPropertyName("col1")]
        public Int32 Col1 { get; }
    }
}
