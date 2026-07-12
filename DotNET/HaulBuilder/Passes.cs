using System;
using System.Collections.Generic;
using System.Diagnostics;
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
        /// Gets or sets a value indicating whether cut knot was reduced.
        /// </summary>
        public Boolean CutKnotReduced { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether fill knot was reduced.
        /// </summary>
        public Boolean FillKnotReduced { get; set; }
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
    /// Pass generation utilities ported from landlevel.passes.
    /// </summary>
    public static class Passes
    {
        private const Double DEFAULT_MAX_DROP_HOP_FT = 100.0;

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

            if ((Count == 0) || ((Math.Abs(SumE) < 1e-12) && (Math.Abs(SumN) < 1e-12)))
            {
                return 0.0;
            }

            Double Bearing = (Math.Atan2(SumE, SumN) * 180.0 / Math.PI + 360.0) % 360.0;
            Double Nearest = Math.Round(Bearing / 90.0) * 90.0;
            Double Diff = Math.Abs((((Bearing - Nearest + 180.0) % 360.0) - 180.0));
            if (Diff <= SnapTolDeg)
            {
                return (Nearest + 360.0) % 360.0;
            }

            return Bearing;
        }

        /// <summary>
        /// Generates cut-haul-fill passes up to scraper load limits.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="StripList">Stream strips.</param>
        /// <param name="ToleranceMm">Tolerance in millimetres.</param>
        /// <param name="MaxLoadBcy">Load capacity in BCY.</param>
        /// <param name="MaxRounds">Reserved compatibility parameter.</param>
        /// <param name="HeadingDeg">Optional global heading.</param>
        /// <param name="HeadingStrength">Heading strength.</param>
        /// <param name="StripBias">Strip bias.</param>
        /// <param name="MaxDropHopM">Max drop-hop distance.</param>
        /// <param name="Flow">Flow field for flow-aligned mode.</param>
        /// <param name="FlowAligned">Whether to use flow-aligned mode.</param>
        /// <param name="MinLoadFrac">Minimum load fraction before stopping multi-drop.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <returns>Generated passes and unmatched totals.</returns>
        static public PassResult GeneratePasses
            (
            Grid Grid,
            List<Strip> StripList,
            Double ToleranceMm,
            Double MaxLoadBcy,
            Int32 MaxRounds = 0,
            Double? HeadingDeg = null,
            Double HeadingStrength = 0.5,
            Double StripBias = 50.0,
            Double MaxDropHopM = DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS,
            FlowField? Flow = null,
            Boolean FlowAligned = false,
            Double MinLoadFrac = 1.0,
            ObstacleField? Obstacles = null
            )
        {
            // Mirror ledger-based depletion of cut and fill depths.
            _Ledger Ledger = new _Ledger(Grid, ToleranceMm / 1000.0);
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            MinLoadFrac = Math.Min(Math.Max(MinLoadFrac, 0.0), 1.0);
            Double CutPerM = Ledger.CutBcyPerM();
            Double FillPerM = Ledger.FillBcyPerM();
            _ActiveTracker Tracker = new _ActiveTracker(Ledger);
            _PassContext Context = new _PassContext(Grid);
            (Double Ux, Double Uy)? StripUv = null;
            if (HeadingDeg.HasValue && (StripBias > 1.0))
            {
                Double Radians = HeadingDeg.Value * Math.PI / 180.0;
                StripUv = (Math.Sin(Radians), Math.Cos(Radians));
            }

            (Double[,] FxS, Double[,] FyS)? FlowDir = null;
            if (FlowAligned && (Flow != null))
            {
                FlowDir = Strips.FlowDirectionField(Grid, Flow);
            }

            Double PruneGapCells = Math.Max(MaxDropHopM / Grid.GridSizeM, 1.5);

            List<Pass> PassesOut = new List<Pass>();
            Int32 Guard = 0;
            Int32 MaxGuard = (Grid.Width * Grid.Height * 64) + 10000;
            while (Guard < MaxGuard)
            {
                Guard += 1;
                if (PassProfiler.Enabled)
                {
                    PassProfiler.PassIterations += 1;
                }

                IReadOnlyList<(Int32 Bx, Int32 By)> CutCellsAvail = Tracker.CutCells();
                IReadOnlyList<(Int32 Bx, Int32 By)> FillCellsAvail = Tracker.FillCells();
                if (PassProfiler.Enabled)
                {
                    PassProfiler.CutCellsListed += CutCellsAvail.Count;
                    PassProfiler.FillCellsListed += FillCellsAvail.Count;
                }
                if ((CutCellsAvail.Count == 0) || (FillCellsAvail.Count == 0))
                {
                    break;
                }

                Double TotalFillBcy = Tracker.FillDepthSum * FillPerM;
                Double LoadCap = Math.Min(MaxLoadBcy, TotalFillBcy);
                if (LoadCap <= 1e-9)
                {
                    break;
                }

                CellKdIndex? FillIndexRaw = CellKdIndex.FromCells(FillCellsAvail, Grid);
                CellKdIndex? CutIndexRaw = CellKdIndex.FromCells(CutCellsAvail, Grid);
                if ((FillIndexRaw is null) || (CutIndexRaw is null))
                {
                    break;
                }

                using CellKdIndex FillIndex = FillIndexRaw;
                using CellKdIndex CutIndex = CutIndexRaw;

                (Int32 Bx, Int32 By) Anchor = CellKdIndex.ChooseAnchor(CutCellsAvail, FillIndex, Grid);
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutPatch = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                Double Loaded = 0.0;
                (Int32 Bx, Int32 By) Seed = Anchor;
                // One consume stamp for the whole cut multi-drop: never re-enter a
                // cell already bitten this load (deep remainder waits for a later pass).
                Context.BeginPhase();
                while (Loaded < (LoadCap - 1e-9))
                {
                    (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells, Double Got) Result;
                    if (FlowDir.HasValue)
                    {
                        Result = GrowFlowTube(
                            Ledger.RemainingCut,
                            Ledger.MaxDepthM,
                            Ledger.Tol,
                            Seed,
                            LoadCap - Loaded,
                            CutPerM,
                            Grid.Width,
                            Grid.Height,
                            FlowDir.Value.FxS,
                            FlowDir.Value.FyS,
                            Grid.Valid,
                            TraceDown: false,
                            TraceUp: true,
                            PruneGapCells,
                            Tracker,
                            Context,
                            IsFill: false);
                    }
                    else
                    {
                        Result = GrowRegion(
                            Ledger.RemainingCut,
                            Ledger.MaxDepthM,
                            Ledger.Tol,
                            Seed,
                            LoadCap - Loaded,
                            CutPerM,
                            Grid.Width,
                            Grid.Height,
                            StripUv,
                            StripBias,
                            Tracker,
                            IsFill: false,
                            Context);
                    }

                    if (Result.Cells.Count > 0)
                    {
                        CutPatch.AddRange(Result.Cells);
                        Loaded += Result.Got;
                    }

                    if (Loaded >= (LoadCap - 1e-9))
                    {
                        break;
                    }

                    if (Loaded >= ((MinLoadFrac * LoadCap) - 1e-9))
                    {
                        break;
                    }

                    (Double X, Double Y) LastXy = (Result.Cells.Count > 0) ? Grid.CellCenterXy(Result.Cells[Result.Cells.Count - 1].Cell.Bx, Result.Cells[Result.Cells.Count - 1].Cell.By) : Grid.CellCenterXy(Seed.Bx, Seed.By);
                    ((Int32 Bx, Int32 By) Cell, Double Dist)? Next = NearestRemaining(LastXy, CutIndex, Ledger.RemainingCut, Ledger.Tol, Context);
                    if (!Next.HasValue || (Next.Value.Dist > MaxDropHopM))
                    {
                        break;
                    }

                    Seed = Next.Value.Cell;
                }

                if (CutPatch.Count == 0)
                {
                    break;
                }

                (Double X, Double Y) CutCentroid = CentroidCells(CutPatch, Grid);
                (Int32 Bx, Int32 By)? FillSeed = null;
                if (FlowDir.HasValue)
                {
                    FillSeed = Strips.StreamlineTo
                        (
                        FlowDir.Value.FxS,
                        FlowDir.Value.FyS,
                        Grid.Valid,
                        (CutCentroid.X - Grid.MinX) / Grid.GridSizeM,
                        (CutCentroid.Y - Grid.MinY) / Grid.GridSizeM,
                        (Int32 Bx, Int32 By) => Ledger.RemainingFill[By, Bx] > Ledger.Tol
                        );
                }

                if (!FillSeed.HasValue)
                {
                    if (PassProfiler.Enabled)
                    {
                        PassProfiler.ChooseFillSeedCalls += 1;
                    }

                    FillSeed = ChooseFillSeed(FillIndex, CutCentroid, Grid, HeadingDeg, HeadingStrength);
                }

                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillPatch = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                Double Placed = 0.0;
                (Int32 Bx, Int32 By) FillSeedValue = FillSeed.Value;
                // Fresh stamp for fill multi-drop so cut-phase marks do not block dump cells.
                Context.BeginPhase();
                while (Placed < (Loaded - 1e-9))
                {
                    (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells, Double Got) Result;
                    if (FlowDir.HasValue)
                    {
                        Result = GrowFlowTube(
                            Ledger.RemainingFill,
                            Ledger.MaxDepthM,
                            Ledger.Tol,
                            FillSeedValue,
                            Loaded - Placed,
                            FillPerM,
                            Grid.Width,
                            Grid.Height,
                            FlowDir.Value.FxS,
                            FlowDir.Value.FyS,
                            Grid.Valid,
                            TraceDown: true,
                            TraceUp: false,
                            PruneGapCells,
                            Tracker,
                            Context,
                            IsFill: true);
                    }
                    else
                    {
                        Result = GrowRegion(
                            Ledger.RemainingFill,
                            Ledger.MaxDepthM,
                            Ledger.Tol,
                            FillSeedValue,
                            Loaded - Placed,
                            FillPerM,
                            Grid.Width,
                            Grid.Height,
                            StripUv,
                            StripBias,
                            Tracker,
                            IsFill: true,
                            Context);
                    }

                    if (Result.Cells.Count > 0)
                    {
                        FillPatch.AddRange(Result.Cells);
                        Placed += Result.Got;
                    }

                    if (Placed >= (Loaded - 1e-9))
                    {
                        break;
                    }

                    (Double X, Double Y) LastXy = (Result.Cells.Count > 0) ? Grid.CellCenterXy(Result.Cells[Result.Cells.Count - 1].Cell.Bx, Result.Cells[Result.Cells.Count - 1].Cell.By) : Grid.CellCenterXy(FillSeedValue.Bx, FillSeedValue.By);
                    ((Int32 Bx, Int32 By) Cell, Double Dist)? Next = NearestRemaining(LastXy, FillIndex, Ledger.RemainingFill, Ledger.Tol, Context);
                    if (!Next.HasValue || (Next.Value.Dist > MaxDropHopM))
                    {
                        break;
                    }

                    FillSeedValue = Next.Value.Cell;
                }

                if (FillPatch.Count == 0)
                {
                    RefundCells(Ledger.RemainingCut, CutPatch, Tracker, IsFill: false);
                    break;
                }

                Double Leftover = Loaded - Placed;
                if (Leftover > 1e-9)
                {
                    // Refund unplaced cut from tail first so kept cut stays nearest served fill.
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> KeptRev = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                    for (Int32 Index = CutPatch.Count - 1; Index >= 0; Index -= 1)
                    {
                        ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = CutPatch[Index];
                        if (Leftover <= 1e-9)
                        {
                            KeptRev.Add(Entry);
                            continue;
                        }

                        Double CellBcy = Entry.DepthM * CutPerM;
                        Double Give = Math.Min(Leftover, CellBcy);
                        Double Old = Ledger.RemainingCut[Entry.Cell.By, Entry.Cell.Bx];
                        Ledger.RemainingCut[Entry.Cell.By, Entry.Cell.Bx] += Give / CutPerM;
                        Tracker.NoteCutDepth(Entry.Cell.Bx, Entry.Cell.By, Old, Ledger.RemainingCut[Entry.Cell.By, Entry.Cell.Bx]);
                        Leftover -= Give;
                        Double KeepDepth = Entry.DepthM - (Give / CutPerM);
                        if (KeepDepth > 1e-12)
                        {
                            KeptRev.Add((Entry.Cell, KeepDepth));
                        }
                    }

                    KeptRev.Reverse();
                    CutPatch = KeptRev;
                }

                if (CutPatch.Count == 0)
                {
                    continue;
                }

                Int64 BuildStart = PassProfiler.Enabled ? Stopwatch.GetTimestamp() : 0;
                Pass Built = BuildPass(Grid, CutPatch, FillPatch, Placed, ActiveObstacles, FlowDir.HasValue);
                PassProfiler.AddTicks(ref PassProfiler.BuildPassTicks, BuildStart);
                if (PassProfiler.Enabled)
                {
                    PassProfiler.BuildPassCalls += 1;
                }
                if (FlowDir.HasValue)
                {
                    Built.CutHeadingDeg = FlowHeading(FlowDir.Value.FxS, FlowDir.Value.FyS, CutPatch);
                    Built.FillHeadingDeg = FlowHeading(FlowDir.Value.FxS, FlowDir.Value.FyS, FillPatch);
                }

                PassesOut.Add(Built);
            }

            Double UnmatchedCut = SumAbove(Ledger.RemainingCut, Ledger.Tol) * CutPerM;
            Double UnmatchedFill = SumAbove(Ledger.RemainingFill, Ledger.Tol) * FillPerM;
            PassProfiler.Dump();
            return new PassResult(PassesOut, 1, UnmatchedCut, UnmatchedFill);
        }

        /// <summary>
        /// Grows contiguous region around a seed until target BCY is met.
        /// </summary>
        /// <param name="Remaining">Remaining-depth matrix.</param>
        /// <param name="MaxDepth">Max depth per cell.</param>
        /// <param name="Tol">Tolerance threshold.</param>
        /// <param name="SeedCell">Seed cell.</param>
        /// <param name="TargetBcy">Target load in BCY.</param>
        /// <param name="PerM">BCY per metre depth.</param>
        /// <param name="Width">Grid width.</param>
        /// <param name="Height">Grid height.</param>
        /// <param name="HeadingUv">Optional strip heading unit vector.</param>
        /// <param name="StripBias">Perpendicular penalty.</param>
        /// <param name="Tracker">Optional active-cell tracker.</param>
        /// <param name="IsFill">True when consuming fill ledger cells.</param>
        /// <param name="Context">Pass-phase consume stamp (blocks re-entry this load).</param>
        /// <returns>Consumed cells and BCY.</returns>
        static private (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells, Double Got) GrowRegion
            (
            Double[,] Remaining,
            Double MaxDepth,
            Double Tol,
            (Int32 Bx, Int32 By) SeedCell,
            Double TargetBcy,
            Double PerM,
            Int32 Width,
            Int32 Height,
            (Double Ux, Double Uy)? HeadingUv,
            Double StripBias,
            _ActiveTracker? Tracker = null,
            Boolean IsFill = false,
            _PassContext? Context = null
            )
        {
            // Best-first contiguous expansion matches Python region grow behavior.
            PriorityQueue<(Double Key, Int32 T, Int32 X, Int32 Y), Double> Heap = new PriorityQueue<(Double Key, Int32 T, Int32 X, Int32 Y), Double>();
            HashSet<(Int32 X, Int32 Y)> Visited = new HashSet<(Int32 X, Int32 Y)>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Used = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double Acc = 0.0;
            Int32 Counter = 0;
            Heap.Enqueue((0.0, 0, SeedCell.Bx, SeedCell.By), 0.0);
            Boolean Anisotropic = HeadingUv.HasValue && (StripBias != 1.0);
            Double Ux = HeadingUv.HasValue ? HeadingUv.Value.Ux : 0.0;
            Double Uy = HeadingUv.HasValue ? HeadingUv.Value.Uy : 0.0;
            Double Vx = Uy;
            Double Vy = -Ux;

            while ((Heap.Count > 0) && (Acc < (TargetBcy - 1e-9)))
            {
                (Double Key, Int32 T, Int32 X, Int32 Y) Entry = Heap.Dequeue();
                if (Visited.Contains((Entry.X, Entry.Y)))
                {
                    continue;
                }

                Visited.Add((Entry.X, Entry.Y));
                // Skip cells already bitten earlier in this cut/fill multi-drop phase.
                if ((Context is not null) && Context.AlreadyConsumed(Entry.X, Entry.Y))
                {
                    continue;
                }

                Double Rem = Remaining[Entry.Y, Entry.X];
                if (Rem <= Tol)
                {
                    continue;
                }

                Double Depth = Math.Min(MaxDepth, Rem);
                Double Bcy = Depth * PerM;
                if ((Acc + Bcy) > TargetBcy)
                {
                    Bcy = TargetBcy - Acc;
                    Depth = Bcy / PerM;
                }

                Remaining[Entry.Y, Entry.X] -= Depth;
                if (Tracker is not null)
                {
                    if (IsFill)
                    {
                        Tracker.NoteFillDepth(Entry.X, Entry.Y, Rem, Remaining[Entry.Y, Entry.X]);
                    }
                    else
                    {
                        Tracker.NoteCutDepth(Entry.X, Entry.Y, Rem, Remaining[Entry.Y, Entry.X]);
                    }
                }

                Acc += Bcy;
                Used.Add(((Entry.X, Entry.Y), Depth));
                Context?.MarkConsumed(Entry.X, Entry.Y);

                for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                {
                    for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                    {
                        if ((Dx == 0) && (Dy == 0))
                        {
                            continue;
                        }

                        Int32 Nx = Entry.X + Dx;
                        Int32 Ny = Entry.Y + Dy;
                        if ((Nx < 0) || (Nx >= Width) || (Ny < 0) || (Ny >= Height) || Visited.Contains((Nx, Ny)))
                        {
                            continue;
                        }

                        Int32 Ddx = Nx - SeedCell.Bx;
                        Int32 Ddy = Ny - SeedCell.By;
                        Double Key;
                        if (Anisotropic)
                        {
                            Double Along = (Ddx * Ux) + (Ddy * Uy);
                            Double Perp = (Ddx * Vx) + (Ddy * Vy);
                            Key = Math.Abs(Along) + (StripBias * Perp * Perp);
                        }
                        else
                        {
                            Key = (Ddx * Ddx) + (Ddy * Ddy);
                        }

                        Counter += 1;
                        Heap.Enqueue((Key, Counter, Nx, Ny), Key);
                    }
                }
            }

            return (Used, Acc);
        }

        /// <summary>
        /// Consumes one load along a curving streamline through the seed cell.
        /// </summary>
        static private (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells, Double Got) GrowFlowTube
            (
            Double[,] Remaining,
            Double MaxDepth,
            Double Tol,
            (Int32 Bx, Int32 By) SeedCell,
            Double TargetBcy,
            Double PerM,
            Int32 Width,
            Int32 Height,
            Double[,] Fxs,
            Double[,] Fys,
            Boolean[,] Valid,
            Boolean TraceDown,
            Boolean TraceUp,
            Double? MaxGapCells,
            _ActiveTracker? Tracker,
            _PassContext? Context,
            Boolean IsFill,
            Double Step = 0.5,
            Int32 MaxSteps = 4000,
            Int32 MissGap = 4
            )
        {
            if (PassProfiler.Enabled)
            {
                PassProfiler.FlowTubeCalls += 1;
            }

            // Streamline integration keeps flow-aligned passes as one continuous lane.
            // Consume stamps persist across multi-drop tubes in this phase (BeginPhase
            // was already called); do not reset here or deep cells get re-bitten undrivably.
            Int32 Sx = SeedCell.Bx;
            Int32 Sy = SeedCell.By;
            Double Acc = 0.0;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> SeedEntry = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();

            Boolean Consume
                (
                Int32 Bx,
                Int32 By,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Bucket
                )
            {
                if ((Context is not null) && Context.AlreadyConsumed(Bx, By))
                {
                    return false;
                }

                Double Rem = Remaining[By, Bx];
                if (Rem <= Tol)
                {
                    return false;
                }

                Double Depth = Math.Min(MaxDepth, Rem);
                Double Bcy = Depth * PerM;
                if ((Acc + Bcy) > TargetBcy)
                {
                    Bcy = TargetBcy - Acc;
                    Depth = (PerM > 0.0) ? (Bcy / PerM) : 0.0;
                    if (Depth <= 1e-12)
                    {
                        return false;
                    }
                }

                Double Old = Remaining[By, Bx];
                Remaining[By, Bx] -= Depth;
                if (Tracker is not null)
                {
                    if (IsFill)
                    {
                        Tracker.NoteFillDepth(Bx, By, Old, Remaining[By, Bx]);
                    }
                    else
                    {
                        Tracker.NoteCutDepth(Bx, By, Old, Remaining[By, Bx]);
                    }
                }

                Acc += Bcy;
                Bucket.Add(((Bx, By), Depth));
                Context?.MarkConsumed(Bx, By);
                if (PassProfiler.Enabled)
                {
                    PassProfiler.FlowConsumes += 1;
                }

                return true;
            }

            Consume(Sx, Sy, SeedEntry);

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Trace
                (
                Double Sign
                )
            {
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Out = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                Double X = Sx + 0.5;
                Double Y = Sy + 0.5;
                Int32 Misses = 0;
                for (Int32 Iter = 0; Iter < MaxSteps; Iter += 1)
                {
                    if (PassProfiler.Enabled)
                    {
                        PassProfiler.FlowTubeSteps += 1;
                    }

                    if (Acc >= (TargetBcy - 1e-9))
                    {
                        break;
                    }

                    (Double Ux, Double Uy) Flow = Strips.SampleFlow(Fxs, Fys, Valid, X, Y);
                    Double Norm = Math.Sqrt((Flow.Ux * Flow.Ux) + (Flow.Uy * Flow.Uy));
                    if (Norm < 1e-6)
                    {
                        break;
                    }

                    X += Sign * Step * Flow.Ux / Norm;
                    Y += Sign * Step * Flow.Uy / Norm;
                    Int32 Bx = (Int32)Math.Floor(X);
                    Int32 By = (Int32)Math.Floor(Y);
                    if ((Bx < 0) || (Bx >= Width) || (By < 0) || (By >= Height) || !Valid[By, Bx])
                    {
                        break;
                    }

                    if (Consume(Bx, By, Out))
                    {
                        Misses = 0;
                    }
                    else if ((Context is null) || !Context.AlreadyConsumed(Bx, By))
                    {
                        Misses += 1;
                        if ((Misses >= MissGap) && (Out.Count > 0))
                        {
                            break;
                        }
                    }
                }

                return Out;
            }

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Down = TraceDown ? Trace(1.0) : new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Up = TraceUp ? Trace(-1.0) : new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Up.Reverse();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ordered = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Ordered.AddRange(Up);
            Ordered.AddRange(SeedEntry);
            Ordered.AddRange(Down);
            return PruneFarFragments(Ordered, Acc, Remaining, PerM, MaxGapCells, SeedCell, Tracker, IsFill);
        }

        /// <summary>
        /// Keeps one spatially compact fragment of a flow tube and refunds the rest.
        /// </summary>
        static private (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells, Double Got) PruneFarFragments
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ordered,
            Double Acc,
            Double[,] Remaining,
            Double PerM,
            Double? MaxGapCells,
            (Int32 Bx, Int32 By)? SeedCell,
            _ActiveTracker? Tracker,
            Boolean IsFill
            )
        {
            Int64 PruneStart = PassProfiler.Enabled ? Stopwatch.GetTimestamp() : 0;
            if (PassProfiler.Enabled)
            {
                PassProfiler.PruneCalls += 1;
            }

            Int32 N = Ordered.Count;
            if ((N <= 1) || !MaxGapCells.HasValue)
            {
                PassProfiler.AddTicks(ref PassProfiler.PruneTicks, PruneStart);
                return (Ordered, Acc);
            }

            Int32[] Parent = new Int32[N];
            for (Int32 I = 0; I < N; I += 1)
            {
                Parent[I] = I;
            }

            Int32 Find
                (
                Int32 Index
                )
            {
                while (Parent[Index] != Index)
                {
                    Parent[Index] = Parent[Parent[Index]];
                    Index = Parent[Index];
                }

                return Index;
            }

            Double G2 = MaxGapCells.Value * MaxGapCells.Value;
            for (Int32 I = 0; I < N; I += 1)
            {
                (Int32 Xi, Int32 Yi) = Ordered[I].Cell;
                for (Int32 J = (I + 1); J < N; J += 1)
                {
                    Int32 Dx = Xi - Ordered[J].Cell.Bx;
                    Int32 Dy = Yi - Ordered[J].Cell.By;
                    if (((Dx * Dx) + (Dy * Dy)) <= G2)
                    {
                        Int32 Ri = Find(I);
                        Int32 Rj = Find(J);
                        if (Ri != Rj)
                        {
                            Parent[Ri] = Rj;
                        }
                    }
                }
            }

            Dictionary<Int32, List<Int32>> Groups = new Dictionary<Int32, List<Int32>>();
            for (Int32 I = 0; I < N; I += 1)
            {
                Int32 Root = Find(I);
                if (!Groups.TryGetValue(Root, out List<Int32>? Bucket))
                {
                    Bucket = new List<Int32>();
                    Groups[Root] = Bucket;
                }

                Bucket.Add(I);
            }

            if (Groups.Count <= 1)
            {
                PassProfiler.AddTicks(ref PassProfiler.PruneTicks, PruneStart);
                return (Ordered, Acc);
            }

            Int32? KeepRoot = null;
            if (SeedCell.HasValue)
            {
                (Int32 Sx, Int32 Sy) = SeedCell.Value;
                for (Int32 I = 0; I < N; I += 1)
                {
                    if ((Ordered[I].Cell.Bx == Sx) && (Ordered[I].Cell.By == Sy))
                    {
                        KeepRoot = Find(I);
                        break;
                    }
                }
            }

            if (!KeepRoot.HasValue)
            {
                Double BestDepth = Double.NegativeInfinity;
                foreach (KeyValuePair<Int32, List<Int32>> Pair in Groups)
                {
                    Double DepthSum = 0.0;
                    foreach (Int32 Index in Pair.Value)
                    {
                        DepthSum += Ordered[Index].DepthM;
                    }

                    if (DepthSum > BestDepth)
                    {
                        BestDepth = DepthSum;
                        KeepRoot = Pair.Key;
                    }
                }
            }

            HashSet<Int32> Keep = new HashSet<Int32>(Groups[KeepRoot!.Value]);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Kept = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double RemovedBcy = 0.0;
            for (Int32 I = 0; I < Ordered.Count; I += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Ordered[I];
                if (Keep.Contains(I))
                {
                    Kept.Add(Entry);
                }
                else
                {
                    Double Old = Remaining[Entry.Cell.By, Entry.Cell.Bx];
                    Remaining[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
                    if (Tracker is not null)
                    {
                        if (IsFill)
                        {
                            Tracker.NoteFillDepth(Entry.Cell.Bx, Entry.Cell.By, Old, Remaining[Entry.Cell.By, Entry.Cell.Bx]);
                        }
                        else
                        {
                            Tracker.NoteCutDepth(Entry.Cell.Bx, Entry.Cell.By, Old, Remaining[Entry.Cell.By, Entry.Cell.Bx]);
                        }
                    }

                    RemovedBcy += Entry.DepthM * PerM;
                }
            }

            PassProfiler.AddTicks(ref PassProfiler.PruneTicks, PruneStart);
            return (Kept, Acc - RemovedBcy);
        }

        /// <summary>
        /// Builds final pass object with ordered cut/fill sweeps.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="CutCells">Cut cells.</param>
        /// <param name="FillCells">Fill cells.</param>
        /// <param name="LoadBcy">Load BCY.</param>
        /// <param name="Obstacles">Obstacle field.</param>
        /// <param name="Ordered">When true, preserve streamline order and only flip sweeps (flow-aligned passes).</param>
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
                // Flow-tube cells are already in streamline order; flip whole sweeps only.
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

        static private Double? CentralDiff
            (
            Double[,] Field,
            Boolean[,] Valid,
            Int32 By,
            Int32 Bx,
            Boolean XAxis
            )
        {
            // Estimate local derivative only where neighbors exist in valid mask.
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

        static private List<(Int32 Bx, Int32 By)> CellsAbove
            (
            Double[,] Matrix,
            Double Tol
            )
        {
            // Collect currently active material cells above tolerance.
            List<(Int32 Bx, Int32 By)> Out = new List<(Int32 Bx, Int32 By)>();
            Int32 Height = Matrix.GetLength(0);
            Int32 Width = Matrix.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (Matrix[By, Bx] > Tol)
                    {
                        Out.Add((Bx, By));
                    }
                }
            }

            return Out;
        }

        static private Double SumAbove
            (
            Double[,] Matrix,
            Double Tol
            )
        {
            // Sum active depth remaining.
            Double Sum = 0.0;
            Int32 Height = Matrix.GetLength(0);
            Int32 Width = Matrix.GetLength(1);
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (Matrix[By, Bx] > Tol)
                    {
                        Sum += Matrix[By, Bx];
                    }
                }
            }

            return Sum;
        }

        /// <summary>
        /// Tracks active cut/fill cells incrementally to avoid full-grid rescans each pass.
        /// </summary>
        private sealed class _ActiveTracker
        {
            private readonly Double Tol;
            private readonly List<(Int32 Bx, Int32 By)> CutList = new List<(Int32 Bx, Int32 By)>();
            private readonly List<(Int32 Bx, Int32 By)> FillList = new List<(Int32 Bx, Int32 By)>();
            private Boolean CutListDirty = true;
            private Boolean FillListDirty = true;

            public _ActiveTracker
                (
                _Ledger ledger
                )
            {
                Tol = ledger.Tol;
                for (Int32 By = 0; By < ledger.Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < ledger.Grid.Width; Bx += 1)
                    {
                        Double CutDepth = ledger.RemainingCut[By, Bx];
                        if (CutDepth > Tol)
                        {
                            Cut.Add((Bx, By));
                            CutListDirty = true;
                        }

                        Double FillDepth = ledger.RemainingFill[By, Bx];
                        if (FillDepth > Tol)
                        {
                            Fill.Add((Bx, By));
                            FillListDirty = true;
                            FillDepthSum += FillDepth;
                        }
                    }
                }
            }

            public HashSet<(Int32 Bx, Int32 By)> Cut { get; } = new HashSet<(Int32 Bx, Int32 By)>();

            public HashSet<(Int32 Bx, Int32 By)> Fill { get; } = new HashSet<(Int32 Bx, Int32 By)>();

            public Double FillDepthSum { get; private set; }

            public IReadOnlyList<(Int32 Bx, Int32 By)> CutCells
                (
                )
            {
                if (CutListDirty)
                {
                    // Match Python np.argwhere row-major order (by, then bx) so
                    // anchor ties and KD batch queries stay deterministic.
                    CutList.Clear();
                    CutList.AddRange(Cut.OrderBy(Cell => Cell.By).ThenBy(Cell => Cell.Bx));
                    CutListDirty = false;
                }

                return CutList;
            }

            public IReadOnlyList<(Int32 Bx, Int32 By)> FillCells
                (
                )
            {
                if (FillListDirty)
                {
                    FillList.Clear();
                    FillList.AddRange(Fill.OrderBy(Cell => Cell.By).ThenBy(Cell => Cell.Bx));
                    FillListDirty = false;
                }

                return FillList;
            }

            public void NoteCutDepth
                (
                Int32 bx,
                Int32 by,
                Double oldDepth,
                Double newDepth
                )
            {
                UpdateSet(Cut, ref CutListDirty, bx, by, oldDepth, newDepth);
            }

            public void NoteFillDepth
                (
                Int32 bx,
                Int32 by,
                Double oldDepth,
                Double newDepth
                )
            {
                if (oldDepth > Tol)
                {
                    FillDepthSum -= oldDepth;
                }

                if (newDepth > Tol)
                {
                    FillDepthSum += newDepth;
                }

                UpdateSet(Fill, ref FillListDirty, bx, by, oldDepth, newDepth);
            }

            private void UpdateSet
                (
                HashSet<(Int32 Bx, Int32 By)> set,
                ref Boolean listDirty,
                Int32 bx,
                Int32 by,
                Double oldDepth,
                Double newDepth
                )
            {
                (Int32 Bx, Int32 By) Key = (bx, by);
                if ((oldDepth > Tol) && (newDepth <= Tol))
                {
                    set.Remove(Key);
                    listDirty = true;
                }
                else if ((oldDepth <= Tol) && (newDepth > Tol))
                {
                    set.Add(Key);
                    listDirty = true;
                }
            }
        }

        /// <summary>
        /// Reusable per-pass scratch state for flow-tube and region growth.
        /// </summary>
        private sealed class _PassContext
        {
            private readonly Int32[,] _ConsumeStamp;

            /// <summary>
            /// Allocates a consume-stamp grid matching the plan grid.
            /// </summary>
            /// <param name="grid">Grid whose dimensions size the stamp array.</param>
            public _PassContext
                (
                Grid grid
                )
            {
                _ConsumeStamp = new Int32[grid.Height, grid.Width];
            }

            /// <summary>
            /// Gets the active phase stamp id used by <see cref="AlreadyConsumed"/>.
            /// </summary>
            public Int32 ConsumeStampId { get; private set; }

            /// <summary>
            /// Starts a new cut or fill multi-drop phase so prior marks do not block the next.
            /// </summary>
            public void BeginPhase
                (
                )
            {
                // Intent: one stamp covers every tube/region hop in the phase so a cell
                // can contribute at most one max-depth bite per load (no hop-back restack).
                ConsumeStampId += 1;
                if (ConsumeStampId == Int32.MaxValue)
                {
                    Array.Clear(_ConsumeStamp, 0, _ConsumeStamp.Length);
                    ConsumeStampId = 1;
                }
            }

            /// <summary>
            /// Returns whether <paramref name="bx"/>,<paramref name="by"/> was already bitten this phase.
            /// </summary>
            /// <param name="bx">Cell column.</param>
            /// <param name="by">Cell row.</param>
            /// <returns>True when already consumed under the current phase stamp.</returns>
            public Boolean AlreadyConsumed
                (
                Int32 bx,
                Int32 by
                )
            {
                return _ConsumeStamp[by, bx] == ConsumeStampId;
            }

            /// <summary>
            /// Marks <paramref name="bx"/>,<paramref name="by"/> as consumed for the current phase.
            /// </summary>
            /// <param name="bx">Cell column.</param>
            /// <param name="by">Cell row.</param>
            public void MarkConsumed
                (
                Int32 bx,
                Int32 by
                )
            {
                _ConsumeStamp[by, bx] = ConsumeStampId;
            }
        }

        static private (Int32 Bx, Int32 By) ChooseFillSeed
            (
            CellKdIndex FillIndex,
            (Double X, Double Y) CutCentroid,
            Grid Grid,
            Double? HeadingDeg,
            Double HeadingStrength
            )
        {
            // Prefer nearest fill, optionally biased down-heading (Python _choose_fill_seed).
            if (!HeadingDeg.HasValue || (HeadingStrength <= 0.0))
            {
                foreach ((Int32 NodeIndex, Double _) in FillIndex.Nearest(CutCentroid.X, CutCentroid.Y, 1))
                {
                    return FillIndex.CellAt(NodeIndex);
                }

                return FillIndex.CellAt(0);
            }

            Double Radians = HeadingDeg.Value * Math.PI / 180.0;
            (Double Ux, Double Uy) Hvec = (Math.Sin(Radians), Math.Cos(Radians));
            Double K = Math.Max(0.0, Math.Min(0.95, HeadingStrength));
            Int32 CandidateCount = Math.Min(200, FillIndex.Count);
            Double BestCost = Double.PositiveInfinity;
            (Int32 Bx, Int32 By) BestPick = FillIndex.CellAt(0);
            foreach ((Int32 NodeIndex, Double DistSq) in FillIndex.Nearest(CutCentroid.X, CutCentroid.Y, CandidateCount))
            {
                (Int32 Bx, Int32 By) Cell = FillIndex.CellAt(NodeIndex);
                Double Dx = Grid.CenterX[Cell.By, Cell.Bx] - CutCentroid.X;
                Double Dy = Grid.CenterY[Cell.By, Cell.Bx] - CutCentroid.Y;
                Double Dist = Math.Sqrt(DistSq);
                Double Safe = (Dist > 1e-9) ? Dist : 1e-9;
                Double Cos = ((Dx * Hvec.Ux) + (Dy * Hvec.Uy)) / Safe;
                Double Cost = Dist * (1.0 - (K * Cos));
                if (Cost < BestCost)
                {
                    BestCost = Cost;
                    BestPick = Cell;
                }
            }

            return BestPick;
        }

        /// <summary>
        /// Finds the nearest remaining material cell to a query point.
        /// </summary>
        /// <param name="Query">Query coordinate.</param>
        /// <param name="Index">KD index of candidate cells.</param>
        /// <param name="Remaining">Remaining depth matrix.</param>
        /// <param name="Tol">Depth tolerance.</param>
        /// <param name="Context">Optional phase stamp; skips cells already bitten this load.</param>
        /// <param name="K">Max KD neighbors to inspect.</param>
        /// <returns>Nearest remaining cell and distance, or null.</returns>
        static private ((Int32 Bx, Int32 By) Cell, Double Dist)? NearestRemaining
            (
            (Double X, Double Y) Query,
            CellKdIndex Index,
            Double[,] Remaining,
            Double Tol,
            _PassContext? Context = null,
            Int32 K = 24
            )
        {
            Int64 QueryStart = PassProfiler.Enabled ? Stopwatch.GetTimestamp() : 0;
            if (PassProfiler.Enabled)
            {
                PassProfiler.NearestRemainingCalls += 1;
            }

            // Inspect the k nearest active pockets; skip cells already consumed this
            // phase so multi-drop hops to a *new* pocket, not a hop-back restack.
            // On equal distance prefer (by, bx) so scipy vs nanoflann tie order agrees.
            const Double DistTieEps = 1e-12;
            Boolean HaveBest = false;
            (Int32 Bx, Int32 By) BestCell = default;
            Double BestDist = 0.0;
            foreach ((Int32 NodeIndex, Double DistSq) in Index.Nearest(Query.X, Query.Y, K))
            {
                (Int32 Bx, Int32 By) Cell = Index.CellAt(NodeIndex);
                if (Remaining[Cell.By, Cell.Bx] <= Tol)
                {
                    continue;
                }

                if ((Context is not null) && Context.AlreadyConsumed(Cell.Bx, Cell.By))
                {
                    continue;
                }

                Double Dist = Math.Sqrt(DistSq);
                if (!HaveBest
                    || (Dist < (BestDist - DistTieEps))
                    || ((Math.Abs(Dist - BestDist) <= DistTieEps)
                        && ((Cell.By < BestCell.By)
                            || ((Cell.By == BestCell.By) && (Cell.Bx < BestCell.Bx)))))
                {
                    HaveBest = true;
                    BestCell = Cell;
                    BestDist = Dist;
                }
            }

            PassProfiler.AddTicks(ref PassProfiler.NearestRemainingTicks, QueryStart);
            return HaveBest ? (BestCell, BestDist) : null;
        }

        static private Double FlowHeading
            (
            Double[,] FxS,
            Double[,] FyS,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            // Aggregate local flow vectors over patch to obtain dominant heading.
            Double SumX = 0.0;
            Double SumY = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in Cells)
            {
                SumX += FxS[Cell.Cell.By, Cell.Cell.Bx];
                SumY += FyS[Cell.Cell.By, Cell.Cell.Bx];
            }

            if (((SumX * SumX) + (SumY * SumY)) < 1e-12)
            {
                return Double.NaN;
            }

            return (Math.Atan2(SumX, SumY) * 180.0 / Math.PI + 360.0) % 360.0;
        }

        static private void RefundCells
            (
            Double[,] Remaining,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            _ActiveTracker? Tracker,
            Boolean IsFill
            )
        {
            // Undo tentative cut consumption when paired fill cannot be created.
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                Double Old = Remaining[Entry.Cell.By, Entry.Cell.Bx];
                Remaining[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
                if (Tracker is null)
                {
                    continue;
                }

                if (IsFill)
                {
                    Tracker.NoteFillDepth(Entry.Cell.Bx, Entry.Cell.By, Old, Remaining[Entry.Cell.By, Entry.Cell.Bx]);
                }
                else
                {
                    Tracker.NoteCutDepth(Entry.Cell.Bx, Entry.Cell.By, Old, Remaining[Entry.Cell.By, Entry.Cell.Bx]);
                }
            }
        }

        static private (Double X, Double Y) CentroidCells
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            // Compute centroid of cell centres.
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

        static private Int32 FarthestIndex
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            (Double X, Double Y) Ref,
            Grid Grid
            )
        {
            // Pick cell farthest from reference.
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

        static private Int32 NearestIndex
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            (Double X, Double Y) Ref,
            Grid Grid
            )
        {
            // Pick cell nearest to reference.
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

        static private List<((Int32 Bx, Int32 By) Cell, Double DepthM)> NnChain
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Int32 StartIdx
            )
        {
            // Reorder patch as nearest-neighbor chain.
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

        static private Double Distance
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            // Shared Euclidean helper.
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Internal depth ledger.
        /// </summary>
        private sealed class _Ledger
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="_Ledger"/> class.
            /// </summary>
            /// <param name="Grid">Grid model.</param>
            /// <param name="ToleranceM">Tolerance in metres.</param>
            public _Ledger
                (
                Grid Grid,
                Double ToleranceM
                )
            {
                // Split signed delta into independent cut/fill depth ledgers.
                this.Grid = Grid;
                Tol = ToleranceM;
                RemainingCut = new Double[Grid.Height, Grid.Width];
                RemainingFill = new Double[Grid.Height, Grid.Width];
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        if (!Grid.Valid[By, Bx])
                        {
                            continue;
                        }

                        Double Delta = Grid.Existing[By, Bx] - Grid.Target[By, Bx];
                        if (Delta > 0.0)
                        {
                            RemainingCut[By, Bx] = Delta;
                        }
                        else if (Delta < 0.0)
                        {
                            RemainingFill[By, Bx] = -Delta;
                        }
                    }
                }

                Area = Grid.AreaM2;
                Swell = Grid.SwellFactor;
                Compaction = Grid.Compaction;
                MaxDepthM = Grid.MaxDepthMm / 1000.0;
            }

            /// <summary>
            /// Gets grid.
            /// </summary>
            public Grid Grid { get; }

            /// <summary>
            /// Gets tolerance.
            /// </summary>
            public Double Tol { get; }

            /// <summary>
            /// Gets remaining cut depth.
            /// </summary>
            public Double[,] RemainingCut { get; }

            /// <summary>
            /// Gets remaining fill depth.
            /// </summary>
            public Double[,] RemainingFill { get; }

            /// <summary>
            /// Gets area.
            /// </summary>
            public Double Area { get; }

            /// <summary>
            /// Gets swell.
            /// </summary>
            public Double Swell { get; }

            /// <summary>
            /// Gets compaction.
            /// </summary>
            public Double Compaction { get; }

            /// <summary>
            /// Gets max depth.
            /// </summary>
            public Double MaxDepthM { get; }

            /// <summary>
            /// Cut BCY per metre of depth.
            /// </summary>
            /// <returns>BCY per metre.</returns>
            public Double CutBcyPerM
                (
                )
            {
                // Convert cut depth to bank volume.
                return Area * Constants.M3_TO_CY;
            }

            /// <summary>
            /// Fill-required BCY per metre depth.
            /// </summary>
            /// <returns>BCY per metre.</returns>
            public Double FillBcyPerM
                (
                )
            {
                // Include compaction multiplier for fill demand.
                return Area * Constants.M3_TO_CY * Compaction;
            }
        }
    }
}
