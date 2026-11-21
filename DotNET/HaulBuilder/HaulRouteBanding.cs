using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Drives full bowls first, then other interior loads, then the field
    /// edge. Scraps are the last band. Each band keeps its own travel sort.
    /// </summary>
    static public class HaulRouteBanding
    {
        /// <summary>
        /// Bank cubic yards a load may sit below the fullest bowl and still
        /// count as full. Less than that is a partial and waits until the end.
        /// </summary>
        private const Double SHORT_OF_FULL_BCY = 0.05;

        /// <summary>Cross-track neighbor limit, matching one lane step.</summary>
        private const Int32 NEIGHBOR_CELL_LIMIT = 1;

        /// <summary>Band counts and the stitched route.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one banded route.</summary>
            /// <param name="Route">Passes in band order.</param>
            /// <param name="FullStripCount">One-strip interior loads.</param>
            /// <param name="OtherInteriorCount">Interior loads that join more than one strip.</param>
            /// <param name="EdgeCount">Full loads whose cells all belong to the field-edge batch.</param>
            /// <param name="ScrapCount">One-cell loads and short bowls, driven after the field.</param>
            public Result
                (
                Route Route,
                Int32 FullStripCount,
                Int32 OtherInteriorCount,
                Int32 EdgeCount,
                Int32 ScrapCount
                )
            {
                this.Route = Route;
                this.FullStripCount = FullStripCount;
                this.OtherInteriorCount = OtherInteriorCount;
                this.EdgeCount = EdgeCount;
                this.ScrapCount = ScrapCount;
            }

            /// <summary>Gets the stitched route.</summary>
            public Route Route { get; }

            /// <summary>Gets how many one-strip interior loads were placed first.</summary>
            public Int32 FullStripCount { get; }

            /// <summary>Gets how many other interior loads follow the full strips.</summary>
            public Int32 OtherInteriorCount { get; }

            /// <summary>Gets how many field-edge loads follow the interior.</summary>
            public Int32 EdgeCount { get; }

            /// <summary>Gets how many scraps were left until the end of the field.</summary>
            public Int32 ScrapCount { get; }
        }

        /// <summary>
        /// Sequences each progress band on its own and keeps the reorder window
        /// from moving a load into another band.
        /// </summary>
        /// <param name="Passes">Loads to order. Not modified.</param>
        /// <param name="EdgeCutCells">Cut cells owned by the field-edge batch.</param>
        /// <param name="EdgeFillCells">Fill cells owned by the field-edge batch.</param>
        /// <param name="TwoOptCap">Maximum pass count for 2-opt inside one band.</param>
        /// <param name="GradeReturn">Whether each load exits after reverse grading.</param>
        /// <param name="DirectionAware">Whether transition cost is heading-aware.</param>
        /// <param name="UnloadedTravelSpeedMph">Unloaded travel speed in mph.</param>
        /// <param name="TurnSpeedMph">Turn speed in mph.</param>
        /// <param name="TurningCircleFt">Turning circle diameter in feet.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <param name="StartCandidateCount">Start candidates tried inside each band.</param>
        /// <param name="ReorderWindow">Relocation window. Zero leaves the band sequence as sorted.</param>
        /// <param name="ReorderMaxRounds">Maximum relocation rounds inside one band.</param>
        /// <param name="OrderByDistance">
        /// When true, each band is ordered by metres. When false, by seconds.
        /// </param>
        /// <returns>The banded route and the size of each band.</returns>
        /// <exception cref="ArgumentNullException">Thrown when a required argument is null.</exception>
        static public Result Apply
            (
            IReadOnlyList<Pass> Passes,
            HashSet<(Int32 Bx, Int32 By)> EdgeCutCells,
            HashSet<(Int32 Bx, Int32 By)> EdgeFillCells,
            Int32 TwoOptCap = 1500,
            Boolean GradeReturn = true,
            Boolean DirectionAware = true,
            Double UnloadedTravelSpeedMph = 13.0,
            Double TurnSpeedMph = 4.0,
            Double TurningCircleFt = Constants.DEFAULT_TURNING_CIRCLE_FT,
            ObstacleField? Obstacles = null,
            Int32 StartCandidateCount = 8,
            Int32 ReorderWindow = 0,
            Int32 ReorderMaxRounds = 12,
            Boolean OrderByDistance = false
            )
        {
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            if (EdgeCutCells == null) throw new ArgumentNullException(nameof(EdgeCutCells));
            if (EdgeFillCells == null) throw new ArgumentNullException(nameof(EdgeFillCells));

            Double FullestBcy = 0.0;
            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                if (Passes[Index].LoadBcy > FullestBcy)
                {
                    FullestBcy = Passes[Index].LoadBcy;
                }
            }
            List<Pass> FullPasses = new List<Pass>();
            List<Int32> FullIndices = new List<Int32>();
            List<Pass> OtherPasses = new List<Pass>();
            List<Int32> OtherIndices = new List<Int32>();
            List<Pass> EdgePasses = new List<Pass>();
            List<Int32> EdgeIndices = new List<Int32>();
            List<Pass> ScrapPasses = new List<Pass>();
            List<Int32> ScrapIndices = new List<Int32>();
            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                Pass Pass = Passes[Index];
                // A one-cell load and a short bowl are scraps. They used to share
                // the edge band, and the travel sort then drove them before the
                // full loads that were still in that band.
                if (IsScrap(Pass, FullestBcy))
                {
                    ScrapPasses.Add(Pass);
                    ScrapIndices.Add(Index);
                }
                else if (IsEdgeLoad(Pass, EdgeCutCells, EdgeFillCells))
                {
                    EdgePasses.Add(Pass);
                    EdgeIndices.Add(Index);
                }
                else if (IsFullStrip(Pass))
                {
                    FullPasses.Add(Pass);
                    FullIndices.Add(Index);
                }
                else
                {
                    OtherPasses.Add(Pass);
                    OtherIndices.Add(Index);
                }
            }

            List<Route> Bands = new List<Route>();
            AddBand(
                Bands, FullPasses, FullIndices, TwoOptCap, GradeReturn,
                DirectionAware, UnloadedTravelSpeedMph, TurnSpeedMph,
                TurningCircleFt, Obstacles, StartCandidateCount,
                ReorderWindow, ReorderMaxRounds, OrderByDistance);
            AddBand(
                Bands, OtherPasses, OtherIndices, TwoOptCap, GradeReturn,
                DirectionAware, UnloadedTravelSpeedMph, TurnSpeedMph,
                TurningCircleFt, Obstacles, StartCandidateCount,
                ReorderWindow, ReorderMaxRounds, OrderByDistance);
            AddBand(
                Bands, EdgePasses, EdgeIndices, TwoOptCap, GradeReturn,
                DirectionAware, UnloadedTravelSpeedMph, TurnSpeedMph,
                TurningCircleFt, Obstacles, StartCandidateCount,
                ReorderWindow, ReorderMaxRounds, OrderByDistance);
            AddBand(
                Bands, ScrapPasses, ScrapIndices, TwoOptCap, GradeReturn,
                DirectionAware, UnloadedTravelSpeedMph, TurnSpeedMph,
                TurningCircleFt, Obstacles, StartCandidateCount,
                ReorderWindow, ReorderMaxRounds, OrderByDistance);

            List<Pass> Ordered = new List<Pass>();
            List<Int32> Order = new List<Int32>();
            foreach (Route Band in Bands)
            {
                Ordered.AddRange(Band.OrderedPasses);
                Order.AddRange(Band.Order);
            }
            // Band sorts omit the empty trip from one band to the next.
            Double DeadheadM = Routing.CalculateDeadhead(
                Ordered, GradeReturn, Obstacles);
            return new Result(
                new Route(Order, Ordered, DeadheadM),
                FullPasses.Count,
                OtherPasses.Count,
                EdgePasses.Count,
                ScrapPasses.Count);
        }

        /// <summary>Sequences one band and remaps its order back to the caller's list.</summary>
        private static void AddBand
            (
            List<Route> Bands,
            List<Pass> BandPasses,
            List<Int32> OriginalIndices,
            Int32 TwoOptCap,
            Boolean GradeReturn,
            Boolean DirectionAware,
            Double UnloadedTravelSpeedMph,
            Double TurnSpeedMph,
            Double TurningCircleFt,
            ObstacleField? Obstacles,
            Int32 StartCandidateCount,
            Int32 ReorderWindow,
            Int32 ReorderMaxRounds,
            Boolean OrderByDistance
            )
        {
            if (BandPasses.Count == 0)
            {
                return;
            }
            Route BandRoute = Routing.SequencePasses(
                BandPasses,
                TwoOptCap,
                GradeReturn,
                DirectionAware,
                UnloadedTravelSpeedMph,
                TurnSpeedMph,
                TurningCircleFt,
                Obstacles,
                StartCandidateCount,
                OrderByDistance);
            // SequencePasses numbers the band. Callers still want the original index.
            for (Int32 Index = 0; Index < BandRoute.Order.Count; Index += 1)
            {
                BandRoute.Order[Index] = OriginalIndices[BandRoute.Order[Index]];
            }
            if (ReorderWindow > 0)
            {
                // The window is applied to this band only, so it cannot cross.
                Routing.RefineRouteOrder(
                    BandRoute,
                    ReorderWindow,
                    GradeReturn,
                    DirectionAware,
                    UnloadedTravelSpeedMph,
                    TurnSpeedMph,
                    TurningCircleFt,
                    ReorderMaxRounds,
                    Obstacles,
                    OrderByDistance);
            }
            Bands.Add(BandRoute);
        }

        /// <summary>
        /// True when this load is a scrap. A one-cell side, or a bowl short of
        /// the fullest load, stays until the rest of the field is done.
        /// </summary>
        /// <param name="Pass">Load to classify.</param>
        /// <param name="FullestBcy">Largest load in the job, the full bowl.</param>
        /// <returns>True when the load belongs in the last band.</returns>
        private static Boolean IsScrap
            (
            Pass Pass,
            Double FullestBcy
            )
        {
            return IsShortLoad(Pass) || IsShortBowl(Pass, FullestBcy);
        }

        /// <summary>
        /// True when either side is too short to show as blade-down work.
        /// Those loads wait until the real strips are done.
        /// </summary>
        /// <param name="Pass">Load to classify.</param>
        /// <returns>True when the cut or the fill is only a scrap.</returns>
        private static Boolean IsShortLoad
            (
            Pass Pass
            )
        {
            return (Pass.CutCells.Count < 2) || (Pass.FillCells.Count < 2);
        }

        /// <summary>
        /// True when this load did not fill the scraper. Those loads follow
        /// the full bowls so the start of the job is not a short haul.
        /// </summary>
        /// <param name="Pass">Load to classify.</param>
        /// <param name="FullestBcy">Largest load in the job, the full bowl.</param>
        /// <returns>True when the load is short of that bowl.</returns>
        private static Boolean IsShortBowl
            (
            Pass Pass,
            Double FullestBcy
            )
        {
            return Pass.LoadBcy < FullestBcy - SHORT_OF_FULL_BCY;
        }

        /// <summary>True when every cut and fill cell belongs to the field-edge batch.</summary>
        private static Boolean IsEdgeLoad
            (
            Pass Pass,
            HashSet<(Int32 Bx, Int32 By)> EdgeCutCells,
            HashSet<(Int32 Bx, Int32 By)> EdgeFillCells
            )
        {
            if ((Pass.CutCells.Count == 0) || (Pass.FillCells.Count == 0))
            {
                return false;
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.CutCells)
            {
                if (!EdgeCutCells.Contains(Entry.Cell))
                {
                    return false;
                }
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.FillCells)
            {
                if (!EdgeFillCells.Contains(Entry.Cell))
                {
                    return false;
                }
            }
            return true;
        }

        /// <summary>True when the cut is one strip and the fill is one strip.</summary>
        private static Boolean IsFullStrip
            (
            Pass Pass
            )
        {
            return IsSingleStrip(Pass.CutCells, Pass.CutHeadingDeg)
                && IsSingleStrip(Pass.FillCells, Pass.FillHeadingDeg);
        }

        /// <summary>
        /// True when the cells are one contiguous lane along the given axis.
        /// A missing heading is accepted only for a single row or column.
        /// </summary>
        private static Boolean IsSingleStrip
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double? HeadingDeg
            )
        {
            List<(Int32 Bx, Int32 By)> Unique = UniqueCells(Cells);
            if (Unique.Count == 0)
            {
                return false;
            }
            Double AxisDeg;
            if (HeadingDeg.HasValue)
            {
                if (!Double.IsFinite(HeadingDeg.Value))
                {
                    return false;
                }
                AxisDeg = HaulLaneBuilder.NormalizeAxisHeadingDeg(HeadingDeg.Value);
            }
            else if (!TryInferCardinalAxis(Unique, out AxisDeg))
            {
                return false;
            }

            // Cardinal lanes are exact rows or columns. Any other axis uses the
            // same cross-track band the lane builder uses, so the next parallel
            // lane is a different strip.
            if (Math.Abs(AxisDeg) <= Constants.NUMERIC_COMPARISON_TOLERANCE)
            {
                return IsCardinalStrip(Unique, NorthSouth: true);
            }
            if (Math.Abs(AxisDeg - 90.0) <= Constants.NUMERIC_COMPARISON_TOLERANCE)
            {
                return IsCardinalStrip(Unique, NorthSouth: false);
            }
            return IsHeadingBandStrip(Unique, AxisDeg);
        }

        /// <summary>True when every cell shares one row or column and has no gap.</summary>
        private static Boolean IsCardinalStrip
            (
            List<(Int32 Bx, Int32 By)> Cells,
            Boolean NorthSouth
            )
        {
            if (NorthSouth)
            {
                Int32 Column = Cells[0].Bx;
                for (Int32 Index = 1; Index < Cells.Count; Index += 1)
                {
                    if (Cells[Index].Bx != Column)
                    {
                        return false;
                    }
                }
                Cells.Sort((Left, Right) => Left.By.CompareTo(Right.By));
                return StepsByOne(Cells, UseX: false);
            }

            Int32 Row = Cells[0].By;
            for (Int32 Index = 1; Index < Cells.Count; Index += 1)
            {
                if (Cells[Index].By != Row)
                {
                    return false;
                }
            }
            Cells.Sort((Left, Right) => Left.Bx.CompareTo(Right.Bx));
            return StepsByOne(Cells, UseX: true);
        }

        /// <summary>True when sorted cardinal coordinates advance by one cell.</summary>
        private static Boolean StepsByOne
            (
            List<(Int32 Bx, Int32 By)> Sorted,
            Boolean UseX
            )
        {
            for (Int32 Index = 1; Index < Sorted.Count; Index += 1)
            {
                Int32 Previous = UseX ? Sorted[Index - 1].Bx : Sorted[Index - 1].By;
                Int32 Current = UseX ? Sorted[Index].Bx : Sorted[Index].By;
                if (Current != (Previous + 1))
                {
                    return false;
                }
            }
            return true;
        }

        /// <summary>True when every cell shares one heading band and the band has no gap.</summary>
        private static Boolean IsHeadingBandStrip
            (
            List<(Int32 Bx, Int32 By)> Cells,
            Double AxisDeg
            )
        {
            Double HeadingRad = AxisDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            Double AlongX = Math.Sin(HeadingRad);
            Double AlongY = Math.Cos(HeadingRad);
            Double CrossX = Math.Cos(HeadingRad);
            Double CrossY = -Math.Sin(HeadingRad);
            Int32 Band = (Int32)Math.Round(
                (Cells[0].Bx * CrossX) + (Cells[0].By * CrossY),
                MidpointRounding.AwayFromZero);
            List<(Int32 Bx, Int32 By, Double Along)> Ordered =
                new List<(Int32 Bx, Int32 By, Double Along)>();
            foreach ((Int32 Bx, Int32 By) Cell in Cells)
            {
                Int32 CellBand = (Int32)Math.Round(
                    (Cell.Bx * CrossX) + (Cell.By * CrossY),
                    MidpointRounding.AwayFromZero);
                if (CellBand != Band)
                {
                    return false;
                }
                Double Along = (Cell.Bx * AlongX) + (Cell.By * AlongY);
                Ordered.Add((Cell.Bx, Cell.By, Along));
            }
            Ordered.Sort((Left, Right) =>
            {
                Int32 AlongCompare = Left.Along.CompareTo(Right.Along);
                if (AlongCompare != 0)
                {
                    return AlongCompare;
                }
                Int32 XCompare = Left.Bx.CompareTo(Right.Bx);
                return XCompare != 0 ? XCompare : Left.By.CompareTo(Right.By);
            });
            for (Int32 Index = 1; Index < Ordered.Count; Index += 1)
            {
                Int32 DeltaX = Math.Abs(Ordered[Index].Bx - Ordered[Index - 1].Bx);
                Int32 DeltaY = Math.Abs(Ordered[Index].By - Ordered[Index - 1].By);
                if ((DeltaX > NEIGHBOR_CELL_LIMIT) || (DeltaY > NEIGHBOR_CELL_LIMIT))
                {
                    return false;
                }
            }
            return true;
        }

        /// <summary>Infers a north-south or east-west axis when the cells already share one.</summary>
        private static Boolean TryInferCardinalAxis
            (
            List<(Int32 Bx, Int32 By)> Cells,
            out Double AxisDeg
            )
        {
            Boolean SameColumn = true;
            Boolean SameRow = true;
            for (Int32 Index = 1; Index < Cells.Count; Index += 1)
            {
                if (Cells[Index].Bx != Cells[0].Bx)
                {
                    SameColumn = false;
                }
                if (Cells[Index].By != Cells[0].By)
                {
                    SameRow = false;
                }
            }
            if (SameColumn)
            {
                AxisDeg = 0.0;
                return true;
            }
            if (SameRow)
            {
                AxisDeg = 90.0;
                return true;
            }
            AxisDeg = 0.0;
            return false;
        }

        /// <summary>Drops repeated depth slices of the same cell.</summary>
        private static List<(Int32 Bx, Int32 By)> UniqueCells
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            List<(Int32 Bx, Int32 By)> Unique = new List<(Int32 Bx, Int32 By)>();
            HashSet<(Int32 Bx, Int32 By)> Seen = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                if (Seen.Add(Entry.Cell))
                {
                    Unique.Add(Entry.Cell);
                }
            }
            return Unique;
        }
    }
}
