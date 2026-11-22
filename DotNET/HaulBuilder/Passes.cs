using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// One cut-haul-fill cycle.
    /// </summary>
    public sealed class Pass
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="Pass"/> class.
        /// </summary>
        /// <param name="CutCells">Cut cells and depths.</param>
        /// <param name="FillCells">Fill cells and depths.</param>
        /// <param name="LoadBcy">Moved load in BCY.</param>
        /// <param name="HaulDistanceM">Haul distance in metres.</param>
        /// <param name="CutStartXy">Cut start coordinate.</param>
        /// <param name="CutEndXy">Cut end coordinate.</param>
        /// <param name="FillStartXy">Fill start coordinate.</param>
        /// <param name="FillEndXy">Fill end coordinate.</param>
        public Pass
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
            Double LoadBcy,
            Double HaulDistanceM,
            (Double X, Double Y) CutStartXy,
            (Double X, Double Y) CutEndXy,
            (Double X, Double Y) FillStartXy,
            (Double X, Double Y) FillEndXy
            )
        {
            // Persist patch geometry and travel metrics for guidance/routing.
            this.CutCells = CutCells;
            this.FillCells = FillCells;
            this.LoadBcy = LoadBcy;
            this.HaulDistanceM = HaulDistanceM;
            this.CutStartXy = CutStartXy;
            this.CutEndXy = CutEndXy;
            this.FillStartXy = FillStartXy;
            this.FillEndXy = FillEndXy;
        }

        /// <summary>
        /// Gets or sets cut cells and depths.
        /// </summary>
        public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells { get; set; }

        /// <summary>
        /// Gets or sets fill cells and depths.
        /// </summary>
        public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells { get; set; }

        /// <summary>
        /// Gets moved load BCY.
        /// </summary>
        public Double LoadBcy { get; }

        /// <summary>
        /// Gets haul distance in metres.
        /// </summary>
        public Double HaulDistanceM { get; }

        /// <summary>
        /// Gets cut sweep start.
        /// </summary>
        public (Double X, Double Y) CutStartXy { get; }

        /// <summary>
        /// Gets cut sweep end.
        /// </summary>
        public (Double X, Double Y) CutEndXy { get; }

        /// <summary>
        /// Gets fill sweep start.
        /// </summary>
        public (Double X, Double Y) FillStartXy { get; }

        /// <summary>
        /// Gets fill sweep end.
        /// </summary>
        public (Double X, Double Y) FillEndXy { get; }

        /// <summary>
        /// Gets or sets local cut heading.
        /// </summary>
        public Double? CutHeadingDeg { get; set; }

        /// <summary>
        /// Gets or sets local fill heading.
        /// </summary>
        public Double? FillHeadingDeg { get; set; }

        /// <summary>
        /// Gets or sets the cut seconds from the strip builder, including turns.
        /// Null uses the cell-polyline time.
        /// </summary>
        public Double? StripLoadTimeS { get; set; }

        /// <summary>
        /// Gets or sets the spread seconds from the strip builder, including turns.
        /// Null uses the cell-polyline time.
        /// </summary>
        public Double? StripSpreadTimeS { get; set; }

        /// <summary>
        /// Gets or sets the blade-down cut distance in metres.
        /// Null uses the cell polyline, which counts the jump from one lane to the next.
        /// </summary>
        public Double? StripLoadDistanceM { get; set; }

        /// <summary>
        /// Gets or sets the blade-down fill distance in metres.
        /// </summary>
        public Double? StripSpreadDistanceM { get; set; }

        /// <summary>Gets or sets how many cut passes the area estimate used. Zero on a flow haul.</summary>
        public Int32 AreaCutPasses { get; set; }

        /// <summary>Gets or sets how many fill passes the area estimate used. Zero on a flow haul.</summary>
        public Int32 AreaFillPasses { get; set; }

        /// <summary>Gets or sets the haul heading east. Zero on a flow haul.</summary>
        public Double AreaDirX { get; set; }

        /// <summary>Gets or sets the haul heading north. Zero on a flow haul.</summary>
        public Double AreaDirY { get; set; }

        /// <summary>Gets or sets the cut piece outline. Null on a flow haul.</summary>
        public List<(Int32 Bx, Int32 By)>? AreaCutOutline { get; set; }

        /// <summary>Gets or sets the fill piece outline. Null on a flow haul.</summary>
        public List<(Int32 Bx, Int32 By)>? AreaFillOutline { get; set; }

        /// <summary>Gets or sets the cut piece id. Empty on a flow haul.</summary>
        public String AreaCutPiece { get; set; } = "";

        /// <summary>Gets or sets the fill piece ids. Empty on a flow haul.</summary>
        public String AreaFillPiece { get; set; } = "";

        /// <summary>
        /// Gets or sets whether the cell lists are already a blade-wide flow ribbon.
        /// Guidance then follows that order instead of retileing the cells into lanes.
        /// </summary>
        public Boolean RibbonOrder { get; set; }

        /// <summary>
        /// Gets or sets which builder produced this trip.
        /// Flow, clusters, and thin-cut can interleave, so the label is not a route range.
        /// </summary>
        public HaulStage Stage { get; set; } = HaulStage.Flow;

        /// <summary>
        /// Gets or sets whether one cell may appear once per lift.
        /// A cluster haul records each lift, and a cell deeper than one pass has several.
        /// </summary>
        public Boolean StackedLifts { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether cut knot was reduced.
        /// </summary>
        public Boolean CutKnotReduced { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether fill knot was reduced.
        /// </summary>
        public Boolean FillKnotReduced { get; set; }

        /// <summary>Gets or sets the shallowest ponding layer touched by this pass.</summary>
        public Int32? PondingLayerMin { get; set; }

        /// <summary>Gets or sets the deepest ponding layer touched by this pass.</summary>
        public Int32? PondingLayerMax { get; set; }

        /// <summary>Gets or sets audited pond depth immediately after this pass.</summary>
        public Double? MaxPondDepthM { get; set; }

        /// <summary>Gets or sets audited closed-depression volume immediately after this pass.</summary>
        public Double? ClosedDepressionVolumeBcy { get; set; }
    }

    /// <summary>
    /// Result of pass generation.
    /// </summary>
    public sealed class PassResult
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="PassResult"/> class.
        /// </summary>
        /// <param name="Passes">Generated passes.</param>
        /// <param name="Rounds">Round count.</param>
        /// <param name="UnmatchedCutBcy">Unmatched cut BCY.</param>
        /// <param name="UnmatchedFillBcy">Unmatched fill BCY.</param>
        public PassResult
            (
            List<Pass> Passes,
            Int32 Rounds,
            Double UnmatchedCutBcy,
            Double UnmatchedFillBcy
            )
        {
            // Carry generated passes and remaining imbalance.
            this.Passes = Passes;
            this.Rounds = Rounds;
            this.UnmatchedCutBcy = UnmatchedCutBcy;
            this.UnmatchedFillBcy = UnmatchedFillBcy;
        }

        /// <summary>
        /// Gets passes.
        /// </summary>
        public List<Pass> Passes { get; }

        /// <summary>
        /// Gets round count.
        /// </summary>
        public Int32 Rounds { get; }

        /// <summary>
        /// Gets unmatched cut BCY.
        /// </summary>
        public Double UnmatchedCutBcy { get; }

        /// <summary>
        /// Gets unmatched fill BCY.
        /// </summary>
        public Double UnmatchedFillBcy { get; }
    }

    /// <summary>
    /// Pass construction utilities used by the lane-first haul optimizer.
    /// </summary>
    internal static class Passes
    {
        /// <summary>
        /// Estimates down-slope heading from target grid.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="SnapTolDeg">Axis snap tolerance.</param>
        /// <returns>Compass heading in degrees.</returns>
        static public Double EstimateSlopeHeading
            (
            Grid Grid,
            Double SnapTolDeg = 12.0
            )
        {
            // Approximate mean negative gradient of target to find down-slope direction.
            Double SumE = 0.0;
            Double SumN = 0.0;
            Int32 Count = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double? Gx = CentralDiff(Grid.Target, Grid.Valid, By, Bx, true);
                    Double? Gy = CentralDiff(Grid.Target, Grid.Valid, By, Bx, false);
                    if (Gx.HasValue && Gy.HasValue)
                    {
                        SumE += -Gx.Value;
                        SumN += -Gy.Value;
                        Count += 1;
                    }
                }
            }

            if ((Count == 0) || ((Math.Abs(SumE) < Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) && (Math.Abs(SumN) < Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)))
            {
                return 0.0;
            }

            Double Bearing = (Math.Atan2(SumE, SumN) * Constants.DEGREES_PER_HALF_TURN / Math.PI + Constants.DEGREES_PER_TURN) % Constants.DEGREES_PER_TURN;
            Double Nearest = Math.Round(Bearing / 90.0) * 90.0;
            Double Diff = Math.Abs((((Bearing - Nearest + Constants.DEGREES_PER_HALF_TURN) % Constants.DEGREES_PER_TURN) - Constants.DEGREES_PER_HALF_TURN));
            if (Diff <= SnapTolDeg)
            {
                return (Nearest + Constants.DEGREES_PER_TURN) % Constants.DEGREES_PER_TURN;
            }

            return Bearing;
        }

        /// <summary>
        /// Builds final pass object with ordered cut/fill sweeps.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="CutCells">Cut cells.</param>
        /// <param name="FillCells">Fill cells.</param>
        /// <param name="LoadBcy">Load BCY.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <param name="Ordered">When true, preserve streamline order and only flip sweeps.</param>
        /// <returns>Constructed pass.</returns>
        static public Pass BuildPass
            (
            Grid Grid,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
            Double LoadBcy,
            ObstacleField Obstacles,
            Boolean Ordered = false
            )
        {
            // Order cut to end near fill and fill to start near cut end.
            (Double X, Double Y) FillCentroid = CentroidCells(FillCells, Grid);
            if (Ordered)
            {
                if (CutCells.Count >= 2)
                {
                    (Double X, Double Y) CutFirst = Grid.CellCenterXy(CutCells[0].Cell.Bx, CutCells[0].Cell.By);
                    (Double X, Double Y) CutLast = Grid.CellCenterXy(CutCells[CutCells.Count - 1].Cell.Bx, CutCells[CutCells.Count - 1].Cell.By);
                    if (SqDist(CutFirst, FillCentroid) < SqDist(CutLast, FillCentroid))
                    {
                        CutCells.Reverse();
                    }
                }

                (Double X, Double Y) CutEnd = Grid.CellCenterXy(CutCells[CutCells.Count - 1].Cell.Bx, CutCells[CutCells.Count - 1].Cell.By);
                if (FillCells.Count >= 2)
                {
                    (Double X, Double Y) FillFirst = Grid.CellCenterXy(FillCells[0].Cell.Bx, FillCells[0].Cell.By);
                    (Double X, Double Y) FillLast = Grid.CellCenterXy(FillCells[FillCells.Count - 1].Cell.Bx, FillCells[FillCells.Count - 1].Cell.By);
                    if (SqDist(FillLast, CutEnd) < SqDist(FillFirst, CutEnd))
                    {
                        FillCells.Reverse();
                    }
                }
            }
            else
            {
                Int32 CutStartIndex = FarthestIndex(CutCells, FillCentroid, Grid);
                CutCells = NnChain(CutCells, Grid, CutStartIndex);
                (Double X, Double Y) CutEnd = Grid.CellCenterXy(CutCells[CutCells.Count - 1].Cell.Bx, CutCells[CutCells.Count - 1].Cell.By);
                Int32 FillStartIndex = NearestIndex(FillCells, CutEnd, Grid);
                FillCells = NnChain(FillCells, Grid, FillStartIndex);
            }

            (Double X, Double Y) CutStart = Grid.CellCenterXy(CutCells[0].Cell.Bx, CutCells[0].Cell.By);
            (Double X, Double Y) FillStart = Grid.CellCenterXy(FillCells[0].Cell.Bx, FillCells[0].Cell.By);
            (Double X, Double Y) CutEndFinal = Grid.CellCenterXy(CutCells[CutCells.Count - 1].Cell.Bx, CutCells[CutCells.Count - 1].Cell.By);
            (Double X, Double Y) FillEnd = Grid.CellCenterXy(FillCells[FillCells.Count - 1].Cell.Bx, FillCells[FillCells.Count - 1].Cell.By);
            Double Haul = Obstacles.Active ? Obstacles.PathLength(CutEndFinal, FillStart) : Distance(CutEndFinal, FillStart);
            return new Pass(CutCells, FillCells, LoadBcy, Haul, CutStart, CutEndFinal, FillStart, FillEnd);
        }

        /// <summary>
        /// Squared distance between two points.
        /// </summary>
        static private Double SqDist
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            Double Dx = (A.X - B.X);
            Double Dy = (A.Y - B.Y);
            return ((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Central difference derivative on the valid grid mask.
        /// </summary>
        static private Double? CentralDiff
            (
            Double[,] Field,
            Boolean[,] Valid,
            Int32 By,
            Int32 Bx,
            Boolean XAxis
            )
        {
            Int32 Height = Field.GetLength(0);
            Int32 Width = Field.GetLength(1);
            Int32 MinusBy = By;
            Int32 MinusBx = Bx;
            Int32 PlusBy = By;
            Int32 PlusBx = Bx;
            if (XAxis)
            {
                MinusBx -= 1;
                PlusBx += 1;
            }
            else
            {
                MinusBy -= 1;
                PlusBy += 1;
            }

            if ((MinusBy < 0) || (PlusBy >= Height) || (MinusBx < 0) || (PlusBx >= Width))
            {
                return null;
            }

            if (!Valid[MinusBy, MinusBx] || !Valid[PlusBy, PlusBx])
            {
                return null;
            }

            return (Field[PlusBy, PlusBx] - Field[MinusBy, MinusBx]) / 2.0;
        }

        /// <summary>
        /// Centroid of patch cell centres.
        /// </summary>
        static private (Double X, Double Y) CentroidCells
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            Double SumX = 0.0;
            Double SumY = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in Cells)
            {
                (Double X, Double Y) P = Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By);
                SumX += P.X;
                SumY += P.Y;
            }

            return (SumX / Cells.Count, SumY / Cells.Count);
        }

        /// <summary>
        /// Index of the cell farthest from a reference point.
        /// </summary>
        static private Int32 FarthestIndex
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            (Double X, Double Y) Ref,
            Grid Grid
            )
        {
            Double Best = Double.NegativeInfinity;
            Int32 BestIndex = 0;
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                Double Dist = Distance(Grid.CellCenterXy(Cells[Index].Cell.Bx, Cells[Index].Cell.By), Ref);
                if (Dist > Best)
                {
                    Best = Dist;
                    BestIndex = Index;
                }
            }

            return BestIndex;
        }

        /// <summary>
        /// Index of the cell nearest a reference point.
        /// </summary>
        static private Int32 NearestIndex
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            (Double X, Double Y) Ref,
            Grid Grid
            )
        {
            Double Best = Double.PositiveInfinity;
            Int32 BestIndex = 0;
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                Double Dist = Distance(Grid.CellCenterXy(Cells[Index].Cell.Bx, Cells[Index].Cell.By), Ref);
                if (Dist < Best)
                {
                    Best = Dist;
                    BestIndex = Index;
                }
            }

            return BestIndex;
        }

        /// <summary>
        /// Reorders patch cells as a nearest-neighbor chain.
        /// </summary>
        static private List<((Int32 Bx, Int32 By) Cell, Double DepthM)> NnChain
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Int32 StartIdx
            )
        {
            if (Cells.Count <= 2)
            {
                return new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Cells);
            }

            List<(Double X, Double Y)> Points = new List<(Double X, Double Y)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in Cells)
            {
                Points.Add(Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By));
            }

            List<Int32> Remaining = new List<Int32>();
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                if (Index != StartIdx)
                {
                    Remaining.Add(Index);
                }
            }

            List<Int32> Order = new List<Int32> { StartIdx };
            while (Remaining.Count > 0)
            {
                (Double X, Double Y) Last = Points[Order[Order.Count - 1]];
                Int32 BestPos = 0;
                Double Best = Double.PositiveInfinity;
                for (Int32 Pos = 0; Pos < Remaining.Count; Pos += 1)
                {
                    Int32 Candidate = Remaining[Pos];
                    Double Dx = Points[Candidate].X - Last.X;
                    Double Dy = Points[Candidate].Y - Last.Y;
                    Double Dist2 = (Dx * Dx) + (Dy * Dy);
                    if (Dist2 < Best)
                    {
                        Best = Dist2;
                        BestPos = Pos;
                    }
                }

                Order.Add(Remaining[BestPos]);
                Remaining.RemoveAt(BestPos);
            }

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Out = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            foreach (Int32 Index in Order)
            {
                Out.Add(Cells[Index]);
            }

            return Out;
        }

        /// <summary>
        /// Euclidean distance between two points.
        /// </summary>
        static private Double Distance
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }
    }
}
