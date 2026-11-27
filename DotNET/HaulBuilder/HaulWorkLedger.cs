using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder
{
    /// <summary>
    /// Canonical per-cell cut and fill work required by the unchanged target surface.
    /// </summary>
    public sealed class HaulWorkLedger
    {
        private const Double DEPTH_TOLERANCE_M = Constants.NUMERIC_COMPARISON_TOLERANCE;

        /// <summary>
        /// Initializes a new Haul Work Ledger instance.
        /// </summary>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="ToleranceM">The tolerance m used by the operation.</param>
        /// <param name="RequiredCutDepthM">The required cut depth m used by the operation.</param>
        /// <param name="RequiredFillDepthM">The required fill depth m used by the operation.</param>

        private HaulWorkLedger
            (
            Grid Grid,
            Double ToleranceM,
            Double[,] RequiredCutDepthM,
            Double[,] RequiredFillDepthM
            )
        {
            this.Grid = Grid;
            this.ToleranceM = ToleranceM;
            this.RequiredCutDepthM = RequiredCutDepthM;
            this.RequiredFillDepthM = RequiredFillDepthM;
            CutBcyPerDepthM = Grid.AreaM2 * Constants.M3_TO_CY;
            FillBcyPerDepthM = CutBcyPerDepthM * Grid.Compaction;
            TotalCutBcy = Sum(RequiredCutDepthM) * CutBcyPerDepthM;
            TotalFillBcy = Sum(RequiredFillDepthM) * FillBcyPerDepthM;
        }

        /// <summary>Gets the source grid.</summary>
        public Grid Grid { get; }

        /// <summary>Gets the depth tolerance used to classify work.</summary>
        public Double ToleranceM { get; }

        /// <summary>Gets required cut depth by cell.</summary>
        public Double[,] RequiredCutDepthM { get; }

        /// <summary>Gets required fill depth by cell.</summary>
        public Double[,] RequiredFillDepthM { get; }

        /// <summary>Gets cut BCY represented by one metre of cell depth.</summary>
        public Double CutBcyPerDepthM { get; }

        /// <summary>Gets fill-required BCY represented by one metre of cell depth.</summary>
        public Double FillBcyPerDepthM { get; }

        /// <summary>Gets total required cut BCY.</summary>
        public Double TotalCutBcy { get; private set; }

        /// <summary>Gets total required fill BCY.</summary>
        public Double TotalFillBcy { get; private set; }

        /// <summary>Builds the canonical ledger from the unchanged grid.</summary>
        static public HaulWorkLedger Build
            (
            Grid Grid,
            Double ToleranceMm
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            Double ToleranceM = ToleranceMm / Constants.MILLIMETERS_PER_METER;
            Double[,] Cut = new Double[Grid.Height, Grid.Width];
            Double[,] Fill = new Double[Grid.Height, Grid.Width];
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx]) continue;
                    Double DeltaM = Grid.Existing[By, Bx] - Grid.Target[By, Bx];
                    if (DeltaM > ToleranceM)
                    {
                        Cut[By, Bx] = DeltaM;
                    }
                    else if (DeltaM < -ToleranceM)
                    {
                        Fill[By, Bx] = -DeltaM;
                    }
                }
            }

            return new HaulWorkLedger(Grid, ToleranceM, Cut, Fill);
        }

        /// <summary>
        /// Removes pan fill from a field ledger, and any pan trim still reserved for the road stage.
        /// Surplus above the pan stays ordinary cut. The pan is not a fill target.
        /// </summary>
        /// <param name="PanM">Pan height. Cells at or below zero are left unchanged.</param>
        /// <param name="KeptCutM">Trim depth the road stage keeps, in metres.</param>
        /// <returns>A ledger without that road-stage work.</returns>
        public HaulWorkLedger ReleaseRoadStage
            (
            Double[,] PanM,
            Double[,] KeptCutM
            )
        {
            if (PanM == null)
            {
                throw new ArgumentNullException(nameof(PanM));
            }

            if (KeptCutM == null)
            {
                throw new ArgumentNullException(nameof(KeptCutM));
            }

            Double[,] Cut = (Double[,])RequiredCutDepthM.Clone();
            Double[,] Fill = (Double[,])RequiredFillDepthM.Clone();
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (PanM[By, Bx] <= 1e-5)
                    {
                        continue;
                    }

                    // The main stages cut surplus down to the pan and never fill the cell.
                    // Kept depth is zero at planning time. Leftover surplus is measured after those passes.
                    Fill[By, Bx] = 0.0;
                    Cut[By, Bx] = Math.Max(0.0, Cut[By, Bx] - KeptCutM[By, Bx]);
                }
            }

            return new HaulWorkLedger(Grid, ToleranceM, Cut, Fill);
        }

        /// <summary>
        /// Builds a synthetic ledger from explicit cut/fill cell depths, used for
        /// end-of-job cleanup of guidance-deferred scraps.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="CutCells">Deferred cut cells and depths.</param>
        /// <param name="FillCells">Deferred fill cells and depths.</param>
        /// <param name="ToleranceMm">Depth tolerance in millimetres.</param>
        /// <returns>Ledger containing only the supplied allocated depths.</returns>
        static public HaulWorkLedger FromAllocatedCells
            (
            Grid Grid,
            IEnumerable<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            IEnumerable<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
            Double ToleranceMm = Constants.DEFAULT_TOLERANCE_MM
            )
        {
            if (Grid == null)
            {
                throw new ArgumentNullException(nameof(Grid));
            }
            if (CutCells == null)
            {
                throw new ArgumentNullException(nameof(CutCells));
            }
            if (FillCells == null)
            {
                throw new ArgumentNullException(nameof(FillCells));
            }

            Double ToleranceM = ToleranceMm / Constants.MILLIMETERS_PER_METER;
            Double[,] Cut = new Double[Grid.Height, Grid.Width];
            Double[,] Fill = new Double[Grid.Height, Grid.Width];
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in CutCells)
            {
                if (Entry.DepthM <= 0.0)
                {
                    continue;
                }
                // Intent: accumulate multi-pass scraps that share a cell.
                Cut[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in FillCells)
            {
                if (Entry.DepthM <= 0.0)
                {
                    continue;
                }
                Fill[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
            }
            return new HaulWorkLedger(Grid, ToleranceM, Cut, Fill);
        }

        /// <summary>
        /// Adds cut depth the road stage left in the reserve, so correcting can take it.
        /// </summary>
        /// <param name="By">Row.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="DepthM">Depth to add, in metres. Zero and negative are ignored.</param>
        internal void AddRequiredCut
            (
            Int32 By,
            Int32 Bx,
            Double DepthM
            )
        {
            if (DepthM <= 0.0)
            {
                return;
            }

            RequiredCutDepthM[By, Bx] += DepthM;
            TotalCutBcy += DepthM * CutBcyPerDepthM;
        }

        /// <summary>
        /// Adds fill depth that pairs with cut the road stage returned to correcting.
        /// </summary>
        /// <param name="By">Row.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="DepthM">Depth to add, in metres. Zero and negative are ignored.</param>
        internal void AddRequiredFill
            (
            Int32 By,
            Int32 Bx,
            Double DepthM
            )
        {
            if (DepthM <= 0.0)
            {
                return;
            }

            RequiredFillDepthM[By, Bx] += DepthM;
            TotalFillBcy += DepthM * FillBcyPerDepthM;
        }

        /// <summary>
        /// Removes cut the road stage already took, so correcting does not take it again.
        /// </summary>
        /// <param name="By">Row.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="DepthM">Depth to remove, in metres. Limited to the depth still required.</param>
        internal void ReleaseRequiredCut
            (
            Int32 By,
            Int32 Bx,
            Double DepthM
            )
        {
            if (DepthM <= 0.0)
            {
                return;
            }

            Double Take = Math.Min(DepthM, RequiredCutDepthM[By, Bx]);
            RequiredCutDepthM[By, Bx] -= Take;
            TotalCutBcy -= Take * CutBcyPerDepthM;
            if (TotalCutBcy < 0.0)
            {
                TotalCutBcy = 0.0;
            }
        }

        /// <summary>
        /// Builds a ledger containing only work not owned by the supplied passes.
        /// The source target and material conversions are unchanged; this supports
        /// incremental completion without reallocating an already accepted plan.
        /// </summary>
        public HaulWorkLedger BuildResidual
            (
            IEnumerable<Pass> AssignedPasses
            )
        {
            if (AssignedPasses == null)
            {
                throw new ArgumentNullException(nameof(AssignedPasses));
            }
            (Double[,] AssignedCut, Double[,] AssignedFill) =
                Accumulate(AssignedPasses);
            Double[,] Cut = new Double[this.Grid.Height, this.Grid.Width];
            Double[,] Fill = new Double[this.Grid.Height, this.Grid.Width];
            for (Int32 By = 0; By < this.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < this.Grid.Width; Bx += 1)
                {
                    Double CutRemainingM = RequiredCutDepthM[By, Bx]
                        - AssignedCut[By, Bx];
                    Double FillRemainingM = RequiredFillDepthM[By, Bx]
                        - AssignedFill[By, Bx];
                    if (CutRemainingM < -DEPTH_TOLERANCE_M)
                    {
                        throw new InvalidOperationException(String.Format(
                            System.Globalization.CultureInfo.InvariantCulture,
                            "Assigned work exceeds the source ledger. kind=Cut cell=({0},{1}) required={2:R} assigned={3:R}",
                            Bx,
                            By,
                            RequiredCutDepthM[By, Bx],
                            AssignedCut[By, Bx]));
                    }
                    // Finish-band spread on at-grade ground is recorded fill.
                    // It is not required work, and it stays inside the tolerance.
                    if ((FillRemainingM < -DEPTH_TOLERANCE_M)
                        && !WithinGradeBand(By, Bx, AssignedFill[By, Bx]))
                    {
                        throw new InvalidOperationException(String.Format(
                            System.Globalization.CultureInfo.InvariantCulture,
                            "Assigned work exceeds the source ledger. kind=Fill cell=({0},{1}) required={2:R} assigned={3:R}",
                            Bx,
                            By,
                            RequiredFillDepthM[By, Bx],
                            AssignedFill[By, Bx]));
                    }
                    Cut[By, Bx] = Math.Max(0.0, CutRemainingM);
                    Fill[By, Bx] = Math.Max(0.0, FillRemainingM);
                }
            }
            return new HaulWorkLedger(this.Grid, ToleranceM, Cut, Fill);
        }

        /// <summary>Returns assigned cut and fill depths accumulated by physical cell.</summary>
        public (Double[,] CutDepthM, Double[,] FillDepthM) Accumulate
            (
            IEnumerable<Pass> Passes
            )
        {
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            Double[,] Cut = new Double[this.Grid.Height, this.Grid.Width];
            Double[,] Fill = new Double[this.Grid.Height, this.Grid.Width];
            foreach (Pass Pass in Passes)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.CutCells)
                {
                    ValidateEntry(Entry.Cell, Entry.DepthM);
                    Cut[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
                }
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Pass.FillCells)
                {
                    ValidateEntry(Entry.Cell, Entry.DepthM);
                    Fill[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
                }
            }

            return (Cut, Fill);
        }

        /// <summary>Builds an explicit exact ledger for every unassigned depth layer.</summary>
        public IReadOnlyList<HaulDeferredWork> BuildDeferredWork
            (
            IEnumerable<Pass> Passes,
            String Reason
            )
        {
            if (Reason == null) throw new ArgumentNullException(nameof(Reason));
            (Double[,] AssignedCut, Double[,] AssignedFill) = Accumulate(Passes);
            List<HaulDeferredWork> Output = new List<HaulDeferredWork>();
            AddKind(
                HaulLaneBuilder.WorkKind.Cut,
                RequiredCutDepthM,
                AssignedCut,
                CutBcyPerDepthM);
            AddKind(
                HaulLaneBuilder.WorkKind.Fill,
                RequiredFillDepthM,
                AssignedFill,
                FillBcyPerDepthM);
            return Output;

            // Adds the cut or fill contribution for every valid field cell to
            // the ledger totals.

            void AddKind
                (
                HaulLaneBuilder.WorkKind Kind,
                Double[,] Required,
                Double[,] Assigned,
                Double BcyPerDepthM
                )
            {
                Double MaximumDepthM = this.Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
                for (Int32 By = 0; By < this.Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < this.Grid.Width; Bx += 1)
                    {
                        Double RemainingM = Required[By, Bx] - Assigned[By, Bx];
                        if (RemainingM < -DEPTH_TOLERANCE_M)
                        {
                            // The spread is already inside the finish band, so the
                            // deferred ledger treats that cell as finished.
                            if ((Kind == HaulLaneBuilder.WorkKind.Fill)
                                && WithinGradeBand(By, Bx, Assigned[By, Bx]))
                            {
                                RemainingM = 0.0;
                            }
                            else
                            {
                                throw new InvalidOperationException(String.Format(
                                    System.Globalization.CultureInfo.InvariantCulture,
                                    "Assigned work exceeds the required work ledger. kind={0} cell=({1},{2}) required={3:R} assigned={4:R}",
                                    Kind,
                                    Bx,
                                    By,
                                    Required[By, Bx],
                                    Assigned[By, Bx]));
                            }
                        }
                        RemainingM = Math.Max(0.0, RemainingM);
                        Int32 LayerIndex = (Int32)Math.Floor(
                            (Assigned[By, Bx] + DEPTH_TOLERANCE_M)
                            / MaximumDepthM);
                        while (RemainingM > DEPTH_TOLERANCE_M)
                        {
                            Double OffsetM = Assigned[By, Bx]
                                - (LayerIndex * MaximumDepthM);
                            Double AvailableM = Math.Max(
                                DEPTH_TOLERANCE_M,
                                MaximumDepthM - Math.Max(0.0, OffsetM));
                            Double DepthM = Math.Min(RemainingM, AvailableM);
                            Output.Add(new HaulDeferredWork(
                                Kind,
                                (Bx, By),
                                LayerIndex,
                                DepthM,
                                DepthM * BcyPerDepthM,
                                Reason));
                            RemainingM -= DepthM;
                            LayerIndex += 1;
                        }
                    }
                }
            }
        }

        /// <summary>Validates exact per-cell ownership for a complete automatic-work plan.</summary>
        public HaulPlanValidation ValidateComplete
            (
            IEnumerable<Pass> Passes,
            Double MaximumLoadBcy,
            Double MaximumDepthM,
            Boolean AllowDeferredWork = false
            )
        {
            List<String> Issues = new List<String>();
            List<Pass> PassList = Passes.ToList();
            (Double[,] AssignedCut, Double[,] AssignedFill) = Accumulate(PassList);
            Double MaximumCutErrorM = 0.0;
            Double MaximumFillErrorM = 0.0;
            for (Int32 By = 0; By < this.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < this.Grid.Width; Bx += 1)
                {
                    Double CutErrorM = Math.Abs(AssignedCut[By, Bx] - RequiredCutDepthM[By, Bx]);
                    Double FillErrorM = Math.Abs(AssignedFill[By, Bx] - RequiredFillDepthM[By, Bx]);
                    MaximumCutErrorM = Math.Max(MaximumCutErrorM, CutErrorM);
                    MaximumFillErrorM = Math.Max(MaximumFillErrorM, FillErrorM);
                    Boolean CutExcess = AssignedCut[By, Bx]
                        > RequiredCutDepthM[By, Bx] + DEPTH_TOLERANCE_M;
                    Boolean FillExcess = (AssignedFill[By, Bx]
                        > RequiredFillDepthM[By, Bx] + DEPTH_TOLERANCE_M)
                        && !WithinGradeBand(By, Bx, AssignedFill[By, Bx]);
                    if (CutExcess
                        || (!AllowDeferredWork
                            && (CutErrorM > DEPTH_TOLERANCE_M)))
                    {
                        Issues.Add(String.Format(
                            System.Globalization.CultureInfo.InvariantCulture,
                            "CUT_DEPTH cell=({0},{1}) required={2:R} assigned={3:R}",
                            Bx, By, RequiredCutDepthM[By, Bx], AssignedCut[By, Bx]));
                    }
                    if (FillExcess
                        || (!AllowDeferredWork
                            && (FillErrorM > DEPTH_TOLERANCE_M)))
                    {
                        Issues.Add(String.Format(
                            System.Globalization.CultureInfo.InvariantCulture,
                            "FILL_DEPTH cell=({0},{1}) required={2:R} assigned={3:R}",
                            Bx, By, RequiredFillDepthM[By, Bx], AssignedFill[By, Bx]));
                    }
                }
            }

            for (Int32 PassIndex = 0; PassIndex < PassList.Count; PassIndex += 1)
            {
                Pass Pass = PassList[PassIndex];
                Double CutBcy = Pass.CutCells.Sum(Entry => Entry.DepthM * CutBcyPerDepthM);
                Double FillBcy = Pass.FillCells.Sum(Entry => Entry.DepthM * FillBcyPerDepthM);
                if (CutBcy > MaximumLoadBcy + 1e-8)
                {
                    Issues.Add(String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "CAPACITY pass={0} load={1:R} limit={2:R}",
                        PassIndex, CutBcy, MaximumLoadBcy));
                }
                if (Math.Abs(CutBcy - FillBcy) > 1e-8)
                {
                    Issues.Add(String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "BALANCE pass={0} cut={1:R} fill={2:R}",
                        PassIndex, CutBcy, FillBcy));
                }
                if (Pass.CutCells.Any(Entry => Entry.DepthM > MaximumDepthM + DEPTH_TOLERANCE_M)
                    || Pass.FillCells.Any(Entry => Entry.DepthM > MaximumDepthM + DEPTH_TOLERANCE_M))
                {
                    Issues.Add(String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "MAX_DEPTH pass={0}", PassIndex));
                }
                // A cluster haul keeps one entry per lift. The same cell may
                // appear again for the next lift. Each entry is still depth-capped.
                if (!Pass.StackedLifts
                    && (Pass.CutCells.GroupBy(Entry => Entry.Cell).Any(Group => Group.Count() > 1)
                        || Pass.FillCells.GroupBy(Entry => Entry.Cell).Any(Group => Group.Count() > 1)))
                {
                    Issues.Add(String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "DUPLICATE_CELL pass={0}", PassIndex));
                }
            }

            return new HaulPlanValidation(
                Issues.Count == 0,
                Issues.Take(1000).ToList(),
                MaximumCutErrorM,
                MaximumFillErrorM);
        }

        /// <summary>
        /// True when assigned fill is finish-band spread on a cell the design called at grade.
        /// The room is from that surface up to target plus the finish tolerance.
        /// </summary>
        /// <param name="By">Row.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="AssignedM">Fill depth recorded on the cell.</param>
        /// <returns>True when the depth stays inside the band and the cell had no required work.</returns>
        private Boolean WithinGradeBand
            (
            Int32 By,
            Int32 Bx,
            Double AssignedM
            )
        {
            if (this.RequiredCutDepthM[By, Bx] > DEPTH_TOLERANCE_M)
            {
                return false;
            }
            if (this.RequiredFillDepthM[By, Bx] > DEPTH_TOLERANCE_M)
            {
                return false;
            }
            Double DeltaM = this.Grid.Existing[By, Bx] - this.Grid.Target[By, Bx];
            Double Room = this.ToleranceM - DeltaM;
            if (Room < 0.0)
            {
                Room = 0.0;
            }
            return AssignedM <= (Room + DEPTH_TOLERANCE_M);
        }

        /// <summary>
        /// Executes the validate entry operation.
        /// </summary>
        /// <param name="Cell">The cell used by the operation.</param>
        /// <param name="DepthM">The depth m used by the operation.</param>

        private void ValidateEntry
            (
            (Int32 Bx, Int32 By) Cell,
            Double DepthM
            )
        {
            if ((Cell.Bx < 0) || (Cell.Bx >= this.Grid.Width)
                || (Cell.By < 0) || (Cell.By >= this.Grid.Height)
                || !this.Grid.Valid[Cell.By, Cell.Bx])
            {
                throw new InvalidOperationException("Haul plan contains an invalid work cell.");
            }
            if (!Double.IsFinite(DepthM) || (DepthM <= 0.0))
            {
                throw new InvalidOperationException("Haul plan contains an invalid work depth.");
            }
        }

        /// <summary>
        /// Executes the sum operation.
        /// </summary>
        /// <param name="Values">The values used by the operation.</param>
        /// <returns>The resulting sum.</returns>

        private static Double Sum
            (
            Double[,] Values
            )
        {
            Double Total = 0.0;
            for (Int32 By = 0; By < Values.GetLength(0); By += 1)
            {
                for (Int32 Bx = 0; Bx < Values.GetLength(1); Bx += 1)
                {
                    Total += Values[By, Bx];
                }
            }
            return Total;
        }
    }

}
