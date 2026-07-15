using System;
using System.Collections.Generic;
using System.Linq;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// One physical grid cell omitted from the cut or fill allocation.
    /// </summary>
    public sealed class EdgePatchOmissionCell
    {
        /// <summary>Initializes an omitted-cell summary.</summary>
        public EdgePatchOmissionCell
            (
            (Int32 Bx, Int32 By) Cell,
            Double DepthM,
            Double VolumeBcy
            )
        {
            this.Cell = Cell;
            this.DepthM = DepthM;
            this.VolumeBcy = VolumeBcy;
        }

        /// <summary>Gets the grid coordinate.</summary>
        public (Int32 Bx, Int32 By) Cell { get; }

        /// <summary>Gets the total omitted depth at this coordinate.</summary>
        public Double DepthM { get; }

        /// <summary>Gets the omitted bank volume represented by this coordinate.</summary>
        public Double VolumeBcy { get; }
    }

    /// <summary>
    /// Result of applying the configurable isolated-edge-patch policy.
    /// </summary>
    public sealed class EdgePatchOmissionResult
    {
        /// <summary>Initializes an omission result.</summary>
        public EdgePatchOmissionResult
            (
            PassResult PassResult,
            Int32 OriginalPassCount,
            Int32 EligibleComponentCount,
            Int32 OmittedComponentCount,
            Double EffectiveProjectCapBcy,
            IReadOnlyList<EdgePatchOmissionCell> OmittedCutCells,
            IReadOnlyList<EdgePatchOmissionCell> OmittedFillCells
            )
        {
            this.PassResult = PassResult;
            this.OriginalPassCount = OriginalPassCount;
            this.EligibleComponentCount = EligibleComponentCount;
            this.OmittedComponentCount = OmittedComponentCount;
            this.EffectiveProjectCapBcy = EffectiveProjectCapBcy;
            this.OmittedCutCells = OmittedCutCells;
            this.OmittedFillCells = OmittedFillCells;
        }

        /// <summary>Gets the mass-balanced pass result used by routing.</summary>
        public PassResult PassResult { get; }

        /// <summary>Gets the pass count before applying the omission policy.</summary>
        public Int32 OriginalPassCount { get; }

        /// <summary>Gets the number of passes eliminated completely by omission.</summary>
        public Int32 RemovedPassCount => OriginalPassCount - PassResult.Passes.Count;

        /// <summary>Gets the number of components satisfying the per-patch limits.</summary>
        public Int32 EligibleComponentCount { get; }

        /// <summary>Gets the number of whole components actually omitted.</summary>
        public Int32 OmittedComponentCount { get; }

        /// <summary>Gets the smaller of the configured absolute and percentage caps.</summary>
        public Double EffectiveProjectCapBcy { get; }

        /// <summary>Gets omitted cut cells, aggregated by physical grid coordinate.</summary>
        public IReadOnlyList<EdgePatchOmissionCell> OmittedCutCells { get; }

        /// <summary>Gets reduced fill cells, aggregated by physical grid coordinate.</summary>
        public IReadOnlyList<EdgePatchOmissionCell> OmittedFillCells { get; }

        /// <summary>Gets omitted cut volume in bank cubic yards.</summary>
        public Double OmittedCutBcy => OmittedCutCells.Sum(Cell => Cell.VolumeBcy);

        /// <summary>Gets matching reduced fill demand in bank cubic yards.</summary>
        public Double OmittedFillBcy => OmittedFillCells.Sum(Cell => Cell.VolumeBcy);
    }

    /// <summary>
    /// Removes isolated, low-volume edge cut components before route sequencing and
    /// reduces the paired fill demand by exactly the same bank volume.
    /// </summary>
    public static class EdgePatchOmission
    {
        private const Double VOLUME_TOLERANCE_BCY = 1e-8;

        /// <summary>
        /// Applies the edge-patch policy without mutating the supplied passes.
        /// </summary>
        /// <param name="Source">Original generated passes.</param>
        /// <param name="Grid">Haul grid whose cell size defines the edge band.</param>
        /// <param name="Options">User-configurable omission limits.</param>
        /// <param name="Obstacles">Optional exclusion geometry used to rebuild changed passes.</param>
        /// <returns>A replacement pass result and an audit of every omitted cell.</returns>
        static public EdgePatchOmissionResult Apply
            (
            PassResult Source,
            Grid Grid,
            HaulBuilderOptions Options,
            ObstacleField? Obstacles = null
            )
        {
            if (Source is null) throw new ArgumentNullException(nameof(Source));
            if (Grid is null) throw new ArgumentNullException(nameof(Grid));
            if (Options is null) throw new ArgumentNullException(nameof(Options));

            if (!Options.OmitIsolatedEdgePatches || (Source.Passes.Count == 0))
            {
                return EmptyResult(Source);
            }

            Double CutPerM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPerM = CutPerM * Grid.Compaction;
            Double TotalCutBcy = Source.Passes.Sum(Pass =>
                Pass.CutCells.Sum(Entry => Entry.DepthM * CutPerM));

            Double PercentageCapBcy = TotalCutBcy
                * Options.MaxTotalIsolatedEdgePatchOmissionPercent / 100.0;
            Double EffectiveCapBcy = Math.Min(
                Options.MaxTotalIsolatedEdgePatchOmissionBcy,
                PercentageCapBcy);
            if (EffectiveCapBcy <= VOLUME_TOLERANCE_BCY)
            {
                return new EdgePatchOmissionResult(
                    Source, Source.Passes.Count, 0, 0, Math.Max(0.0, EffectiveCapBcy),
                    Array.Empty<EdgePatchOmissionCell>(),
                    Array.Empty<EdgePatchOmissionCell>());
            }

            FieldEdgePolicy EdgePolicy =
                new FieldEdgePolicy(Grid, Grid.GridSizeM);
            List<Candidate> Candidates = BuildCandidates(
                Source.Passes,
                CutPerM,
                EdgePolicy,
                Options);
            List<Candidate> Selected = SelectCandidates(Candidates, EffectiveCapBcy);
            if (Selected.Count == 0)
            {
                return new EdgePatchOmissionResult(
                    Source, Source.Passes.Count, Candidates.Count, 0, EffectiveCapBcy,
                    Array.Empty<EdgePatchOmissionCell>(),
                    Array.Empty<EdgePatchOmissionCell>());
            }

            Dictionary<Int32, HashSet<(Int32 Bx, Int32 By)>> SelectedCellsByPass = Selected
                .GroupBy(Candidate => Candidate.PassIndex)
                .ToDictionary(
                    Group => Group.Key,
                    Group => Group.SelectMany(Candidate => Candidate.Cells).ToHashSet());
            Dictionary<(Int32 Bx, Int32 By), Double> OmittedCutDepths =
                new Dictionary<(Int32 Bx, Int32 By), Double>();
            Dictionary<(Int32 Bx, Int32 By), Double> OmittedFillDepths =
                new Dictionary<(Int32 Bx, Int32 By), Double>();
            List<Pass> NewPasses = new List<Pass>();
            ObstacleField ActiveObstacles = Obstacles ?? EMPTY_OBSTACLES;
            Int32 OmittedComponentCount = 0;

            for (Int32 PassIndex = 0; PassIndex < Source.Passes.Count; PassIndex += 1)
            {
                Pass Pass = Source.Passes[PassIndex];
                if (!SelectedCellsByPass.TryGetValue(
                    PassIndex,
                    out HashSet<(Int32 Bx, Int32 By)>? SelectedCells))
                {
                    NewPasses.Add(Pass);
                    continue;
                }
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> RemainingCut = Pass.CutCells
                    .Where(Entry => !SelectedCells.Contains(Entry.Cell))
                    .Select(Entry => (Entry.Cell, Entry.DepthM))
                    .ToList();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> RemovedCut = Pass.CutCells
                    .Where(Entry => SelectedCells.Contains(Entry.Cell))
                    .ToList();
                if (RemovedCut.Count == 0)
                {
                    NewPasses.Add(Pass);
                    continue;
                }

                Double RemovedCutBcy = RemovedCut.Sum(Entry => Entry.DepthM * CutPerM);
                Double AvailableFillBcy = Pass.FillCells.Sum(Entry => Entry.DepthM * FillPerM);
                if (AvailableFillBcy + VOLUME_TOLERANCE_BCY < RemovedCutBcy)
                {
                    // A malformed, unbalanced pass is safer left unchanged than partly omitted.
                    NewPasses.Add(Pass);
                    continue;
                }

                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> RemainingFill = Pass.FillCells
                    .Select(Entry => (Entry.Cell, Entry.DepthM))
                    .ToList();
                Dictionary<(Int32 Bx, Int32 By), Double> RemovedFillDepths =
                    new Dictionary<(Int32 Bx, Int32 By), Double>();
                Double RemainingFillFraction = Math.Max(
                    0.0,
                    (AvailableFillBcy - RemovedCutBcy) / AvailableFillBcy);
                for (Int32 Index = 0; Index < RemainingFill.Count; Index += 1)
                {
                    ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = RemainingFill[Index];
                    Double RemainingDepthM = Entry.DepthM * RemainingFillFraction;
                    Double RemovedDepthM = Entry.DepthM - RemainingDepthM;
                    RemainingFill[Index] = (Entry.Cell, RemainingDepthM);
                    AddDepth(RemovedFillDepths, Entry.Cell, RemovedDepthM);
                }
                Double RemovedFillBcy = RemovedFillDepths.Sum(Pair =>
                    Pair.Value * FillPerM);
                if (Math.Abs(RemovedFillBcy - RemovedCutBcy) > VOLUME_TOLERANCE_BCY)
                {
                    // Avoid an imbalanced output if a malformed pass or floating-point
                    // accumulation prevents an exact proportional reduction.
                    NewPasses.Add(Pass);
                    continue;
                }

                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in RemovedCut)
                {
                    AddDepth(OmittedCutDepths, Entry.Cell, Entry.DepthM);
                }
                foreach (KeyValuePair<(Int32 Bx, Int32 By), Double> Pair in RemovedFillDepths)
                {
                    AddDepth(OmittedFillDepths, Pair.Key, Pair.Value);
                }
                RemainingFill = RemainingFill
                    .Where(Entry => Entry.DepthM > 1e-12)
                    .ToList();
                OmittedComponentCount += Selected.Count(Candidate =>
                    Candidate.PassIndex == PassIndex);

                if ((RemainingCut.Count == 0) && (RemainingFill.Count == 0))
                {
                    continue;
                }
                if ((RemainingCut.Count == 0) || (RemainingFill.Count == 0))
                {
                    throw new InvalidOperationException(
                        "Edge patch omission produced a one-sided pass allocation.");
                }

                Double RemainingLoadBcy = RemainingCut.Sum(Entry => Entry.DepthM * CutPerM);
                Pass Rebuilt = Passes.BuildPass(
                    Grid,
                    RemainingCut,
                    RemainingFill,
                    RemainingLoadBcy,
                    ActiveObstacles,
                    true);
                Rebuilt.CutHeadingDeg = Pass.CutHeadingDeg;
                Rebuilt.FillHeadingDeg = Pass.FillHeadingDeg;
                Rebuilt.CutKnotReduced = Pass.CutKnotReduced;
                Rebuilt.FillKnotReduced = Pass.FillKnotReduced;
                NewPasses.Add(Rebuilt);
            }

            List<EdgePatchOmissionCell> OmittedCutCells = ToAuditCells(
                OmittedCutDepths, CutPerM);
            List<EdgePatchOmissionCell> OmittedFillCells = ToAuditCells(
                OmittedFillDepths, FillPerM);
            Double CutBcy = OmittedCutCells.Sum(Cell => Cell.VolumeBcy);
            Double FillBcy = OmittedFillCells.Sum(Cell => Cell.VolumeBcy);
            if (Math.Abs(CutBcy - FillBcy) > VOLUME_TOLERANCE_BCY)
            {
                throw new InvalidOperationException(
                    "Edge patch omission did not conserve cut/fill bank volume.");
            }

            PassResult NewResult = new PassResult(
                NewPasses,
                Source.Rounds,
                Source.UnmatchedCutBcy,
                Source.UnmatchedFillBcy);
            return new EdgePatchOmissionResult(
                NewResult,
                Source.Passes.Count,
                Candidates.Count,
                OmittedComponentCount,
                EffectiveCapBcy,
                OmittedCutCells,
                OmittedFillCells);
        }

        private static List<Candidate> BuildCandidates
            (
            IReadOnlyList<Pass> Passes,
            Double CutPerM,
            FieldEdgePolicy EdgePolicy,
            HaulBuilderOptions Options
            )
        {
            List<Candidate> Output = new List<Candidate>();
            for (Int32 PassIndex = 0; PassIndex < Passes.Count; PassIndex += 1)
            {
                Pass Pass = Passes[PassIndex];
                Dictionary<(Int32 Bx, Int32 By), Double> CutDepthByCell = Pass.CutCells
                    .GroupBy(Entry => Entry.Cell)
                    .ToDictionary(
                        Group => Group.Key,
                        Group => Group.Sum(Entry => Entry.DepthM));
                HashSet<(Int32 Bx, Int32 By)> Unvisited = CutDepthByCell.Keys.ToHashSet();
                while (Unvisited.Count > 0)
                {
                    (Int32 Bx, Int32 By) Seed = Unvisited.First();
                    Queue<(Int32 Bx, Int32 By)> Open = new Queue<(Int32 Bx, Int32 By)>();
                    List<(Int32 Bx, Int32 By)> Component = new List<(Int32 Bx, Int32 By)>();
                    Unvisited.Remove(Seed);
                    Open.Enqueue(Seed);
                    while (Open.Count > 0)
                    {
                        (Int32 Bx, Int32 By) Current = Open.Dequeue();
                        Component.Add(Current);
                        for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                        {
                            for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                            {
                                if ((Dx == 0) && (Dy == 0)) continue;
                                (Int32 Bx, Int32 By) Neighbor =
                                    (Current.Bx + Dx, Current.By + Dy);
                                if (Unvisited.Remove(Neighbor)) Open.Enqueue(Neighbor);
                            }
                        }
                    }

                    Double VolumeBcy = Component.Sum(Cell => CutDepthByCell[Cell] * CutPerM);
                    if ((Component.Count > Options.MaxIsolatedEdgePatchCells)
                        || (VolumeBcy > Options.MaxIsolatedEdgePatchVolumeBcy + VOLUME_TOLERANCE_BCY)
                        || Component.Any(Cell => !EdgePolicy.IsEdgeWorkCell(Cell)))
                    {
                        continue;
                    }

                    HashSet<(Int32 Bx, Int32 By)> ComponentSet = Component.ToHashSet();
                    Boolean EliminatesPass = Pass.CutCells.All(Entry =>
                        ComponentSet.Contains(Entry.Cell));
                    Output.Add(new Candidate(
                        PassIndex,
                        Component,
                        VolumeBcy,
                        EliminatesPass));
                }
            }
            return Output;
        }

        private static List<Candidate> SelectCandidates
            (
            IReadOnlyList<Candidate> Candidates,
            Double EffectiveCapBcy
            )
        {
            List<Candidate> Selected = new List<Candidate>();
            Double UsedBcy = 0.0;
            foreach (Candidate Candidate in Candidates
                .OrderByDescending(Item => Item.EliminatesPass)
                .ThenBy(Item => Item.VolumeBcy)
                .ThenBy(Item => Item.Cells.Min(Cell => Cell.By))
                .ThenBy(Item => Item.Cells.Min(Cell => Cell.Bx)))
            {
                if ((UsedBcy + Candidate.VolumeBcy)
                    > EffectiveCapBcy + VOLUME_TOLERANCE_BCY)
                {
                    continue;
                }
                Selected.Add(Candidate);
                UsedBcy += Candidate.VolumeBcy;
            }
            return Selected;
        }

        private static void AddDepth
            (
            IDictionary<(Int32 Bx, Int32 By), Double> Depths,
            (Int32 Bx, Int32 By) Cell,
            Double DepthM
            )
        {
            Depths.TryGetValue(Cell, out Double ExistingDepthM);
            Depths[Cell] = ExistingDepthM + DepthM;
        }

        private static List<EdgePatchOmissionCell> ToAuditCells
            (
            IReadOnlyDictionary<(Int32 Bx, Int32 By), Double> Depths,
            Double BcyPerM
            ) => Depths
                .OrderBy(Pair => Pair.Key.By)
                .ThenBy(Pair => Pair.Key.Bx)
                .Select(Pair => new EdgePatchOmissionCell(
                    Pair.Key,
                    Pair.Value,
                    Pair.Value * BcyPerM))
                .ToList();

        private static EdgePatchOmissionResult EmptyResult
            (
            PassResult Source
            ) => new EdgePatchOmissionResult(
                Source,
                Source.Passes.Count,
                0,
                0,
                0.0,
                Array.Empty<EdgePatchOmissionCell>(),
                Array.Empty<EdgePatchOmissionCell>());

        private sealed class Candidate
        {
            public Candidate
                (
                Int32 PassIndex,
                List<(Int32 Bx, Int32 By)> Cells,
                Double VolumeBcy,
                Boolean EliminatesPass
                )
            {
                this.PassIndex = PassIndex;
                this.Cells = Cells;
                this.VolumeBcy = VolumeBcy;
                this.EliminatesPass = EliminatesPass;
            }

            public Int32 PassIndex { get; }
            public List<(Int32 Bx, Int32 By)> Cells { get; }
            public Double VolumeBcy { get; }
            public Boolean EliminatesPass { get; }
        }
    }
}
