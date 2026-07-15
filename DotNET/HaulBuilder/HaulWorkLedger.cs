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
        private const Double DEPTH_TOLERANCE_M = 1e-9;

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
        public Double TotalCutBcy { get; }

        /// <summary>Gets total required fill BCY.</summary>
        public Double TotalFillBcy { get; }

        /// <summary>Builds the canonical ledger from the unchanged grid.</summary>
        static public HaulWorkLedger Build
            (
            Grid Grid,
            Double ToleranceMm
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            Double ToleranceM = ToleranceMm / 1000.0;
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

        /// <summary>Returns assigned cut and fill depths accumulated by physical cell.</summary>
        public (Double[,] CutDepthM, Double[,] FillDepthM) Accumulate
            (
            IEnumerable<Pass> Passes
            )
        {
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            Double[,] Cut = new Double[Grid.Height, Grid.Width];
            Double[,] Fill = new Double[Grid.Height, Grid.Width];
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

            void AddKind
                (
                HaulLaneBuilder.WorkKind Kind,
                Double[,] Required,
                Double[,] Assigned,
                Double BcyPerDepthM
                )
            {
                Double MaximumDepthM = Grid.MaxDepthMm / 1000.0;
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        Double RemainingM = Required[By, Bx] - Assigned[By, Bx];
                        if (RemainingM < -DEPTH_TOLERANCE_M)
                        {
                            throw new InvalidOperationException(
                                "Assigned work exceeds the required work ledger.");
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
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Double CutErrorM = Math.Abs(AssignedCut[By, Bx] - RequiredCutDepthM[By, Bx]);
                    Double FillErrorM = Math.Abs(AssignedFill[By, Bx] - RequiredFillDepthM[By, Bx]);
                    MaximumCutErrorM = Math.Max(MaximumCutErrorM, CutErrorM);
                    MaximumFillErrorM = Math.Max(MaximumFillErrorM, FillErrorM);
                    Boolean CutExcess = AssignedCut[By, Bx]
                        > RequiredCutDepthM[By, Bx] + DEPTH_TOLERANCE_M;
                    Boolean FillExcess = AssignedFill[By, Bx]
                        > RequiredFillDepthM[By, Bx] + DEPTH_TOLERANCE_M;
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
                if (Pass.CutCells.GroupBy(Entry => Entry.Cell).Any(Group => Group.Count() > 1)
                    || Pass.FillCells.GroupBy(Entry => Entry.Cell).Any(Group => Group.Count() > 1))
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

        private void ValidateEntry
            (
            (Int32 Bx, Int32 By) Cell,
            Double DepthM
            )
        {
            if ((Cell.Bx < 0) || (Cell.Bx >= Grid.Width)
                || (Cell.By < 0) || (Cell.By >= Grid.Height)
                || !Grid.Valid[Cell.By, Cell.Bx])
            {
                throw new InvalidOperationException("Haul plan contains an invalid work cell.");
            }
            if (!Double.IsFinite(DepthM) || (DepthM <= 0.0))
            {
                throw new InvalidOperationException("Haul plan contains an invalid work depth.");
            }
        }

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
