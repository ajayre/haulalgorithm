using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder
{
    /// <summary>
    /// Builds experimental straight coverage runs without modifying passes or production guidance.
    /// </summary>
    public static class ExperimentalCoveragePlanner
    {
        private const Double HEADING_STEP_DEG = 5.0;
        private const Double MAX_ALONG_GAP_GRID_MULTIPLIER = 1.75;
        private const Double RUN_COUNT_WEIGHT = 1000000.0;
        private const Double CONNECTOR_WEIGHT = 100.0;
        private const Double WORK_DISTANCE_WEIGHT = 0.01;
        private const Double OMITTED_CELL_WEIGHT = 1000000000000.0;
        private const Double EDGE_PARALLEL_TOLERANCE_DEG = 7.5;
        private const Double NUMERIC_TOLERANCE = 1e-9;
        private const Double ENDPOINT_EXTENSION_GRID_FRACTION = 0.49;

        /// <summary>
        /// One straight blade-down run and the material cells assigned to it.
        /// </summary>
        public sealed class Run
        {
            /// <summary>
            /// Initializes a straight experimental coverage run.
            /// </summary>
            /// <param name="Cells">Allocated material cells covered by the run.</param>
            /// <param name="StartXy">Centerline start coordinate.</param>
            /// <param name="EndXy">Centerline end coordinate.</param>
            /// <param name="HeadingDeg">Compass heading in degrees.</param>
            /// <param name="MaximumLateralErrorM">Maximum assigned-cell distance from the centerline.</param>
            public Run
                (
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
                (Double X, Double Y) StartXy,
                (Double X, Double Y) EndXy,
                Double HeadingDeg,
                Double MaximumLateralErrorM
                )
            {
                // Keep cell ownership explicit so later connector work cannot alter material allocation.
                this.Cells = Cells;
                this.StartXy = StartXy;
                this.EndXy = EndXy;
                this.HeadingDeg = HeadingDeg;
                this.MaximumLateralErrorM = MaximumLateralErrorM;
            }

            /// <summary>Gets the material cells assigned to this run.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells { get; }

            /// <summary>Gets the straight centerline start.</summary>
            public (Double X, Double Y) StartXy { get; }

            /// <summary>Gets the straight centerline end.</summary>
            public (Double X, Double Y) EndXy { get; }

            /// <summary>Gets the run compass heading.</summary>
            public Double HeadingDeg { get; }

            /// <summary>Gets the farthest assigned-cell offset from the centerline.</summary>
            public Double MaximumLateralErrorM { get; }

            /// <summary>Gets straight blade-down centerline length in metres.</summary>
            public Double LengthM
            {
                get
                {
                    Double Dx = EndXy.X - StartXy.X;
                    Double Dy = EndXy.Y - StartXy.Y;
                    return Math.Sqrt((Dx * Dx) + (Dy * Dy));
                }
            }
        }

        /// <summary>
        /// Selected straight-run coverage candidate and its comparison metrics.
        /// </summary>
        public sealed class PlanResult
        {
            /// <summary>
            /// Initializes an experimental coverage result.
            /// </summary>
            /// <param name="HeadingDeg">Selected common sweep heading.</param>
            /// <param name="Runs">Drive-ordered straight coverage runs.</param>
            /// <param name="ConnectorDistanceM">Straight-line distance between coverage runs.</param>
            /// <param name="AnchorDistanceM">Distance to requested start/end anchors.</param>
            /// <param name="Score">Deterministic candidate score.</param>
            /// <param name="OmittedCells">Edge/corner cells that cannot be worked at the required boundary heading.</param>
            public PlanResult
                (
                Double HeadingDeg,
                List<Run> Runs,
                Double ConnectorDistanceM,
                Double AnchorDistanceM,
                Double Score,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> OmittedCells
                )
            {
                // Store candidate metrics separately from production timing and turn construction.
                this.HeadingDeg = HeadingDeg;
                this.Runs = Runs;
                this.ConnectorDistanceM = ConnectorDistanceM;
                this.AnchorDistanceM = AnchorDistanceM;
                this.Score = Score;
                this.OmittedCells = OmittedCells;
            }

            /// <summary>Gets the selected common sweep heading.</summary>
            public Double HeadingDeg { get; }

            /// <summary>Gets drive-ordered straight coverage runs.</summary>
            public List<Run> Runs { get; }

            /// <summary>Gets straight-line distance between successive run endpoints.</summary>
            public Double ConnectorDistanceM { get; }

            /// <summary>Gets approach/exit distance to requested anchors.</summary>
            public Double AnchorDistanceM { get; }

            /// <summary>Gets deterministic candidate score.</summary>
            public Double Score { get; }

            /// <summary>Gets allocated corner cells deliberately omitted from work guidance.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> OmittedCells { get; }

            /// <summary>Gets total straight blade-down centerline distance.</summary>
            public Double WorkDistanceM => Runs.Sum(Run => Run.LengthM);

            /// <summary>Gets maximum assigned-cell distance from any selected centerline.</summary>
            public Double MaximumLateralErrorM => Runs.Count == 0
                ? 0.0
                : Runs.Max(Run => Run.MaximumLateralErrorM);

            /// <summary>Gets all allocated cells in drive order.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> OrderedCells
            {
                get
                {
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Output =
                        new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                    foreach (Run CoverageRun in Runs)
                    {
                        Output.AddRange(CoverageRun.Cells);
                    }
                    return Output;
                }
            }

            /// <summary>Gets driven and deliberately omitted allocation entries together.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> AccountedCells
            {
                get
                {
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Output = OrderedCells;
                    Output.AddRange(OmittedCells);
                    return Output;
                }
            }
        }

        private sealed class ProjectedCell
        {
            /// <summary>
            /// Initializes projected geometry for one allocated cell.
            /// </summary>
            public ProjectedCell
                (
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry,
                Double X,
                Double Y,
                Double Along,
                Double Cross
                )
            {
                // Projection is retained only while comparing one candidate heading.
                this.Entry = Entry;
                this.X = X;
                this.Y = Y;
                this.Along = Along;
                this.Cross = Cross;
            }

            public ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry { get; }
            public Double X { get; }
            public Double Y { get; }
            public Double Along { get; }
            public Double Cross { get; }
        }

        /// <summary>
        /// Selects a deterministic straight-run plan while preserving the exact input cell/depth multiset.
        /// </summary>
        /// <param name="Cells">Allocated cells and depths.</param>
        /// <param name="Grid">Grid geometry.</param>
        /// <param name="BladeWidthM">Effective blade coverage width in metres.</param>
        /// <param name="OrientStartXy">Optional approach anchor.</param>
        /// <param name="OrientEndXy">Optional exit anchor.</param>
        /// <returns>Lowest-scoring straight-run candidate.</returns>
        /// <exception cref="ArgumentNullException">Cells or Grid is null.</exception>
        /// <exception cref="ArgumentOutOfRangeException">BladeWidthM is not positive.</exception>
        /// <exception cref="InvalidOperationException">A candidate changes material allocation.</exception>
        static public PlanResult Plan
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double BladeWidthM,
            (Double X, Double Y)? OrientStartXy = null,
            (Double X, Double Y)? OrientEndXy = null
            )
        {
            List<PlanResult> Candidates = CandidatePlans(
                Cells, Grid, BladeWidthM, OrientStartXy, OrientEndXy);
            return Candidates[0];
        }

        /// <summary>
        /// Builds every deterministic five-degree coverage candidate in score order.
        /// </summary>
        static public List<PlanResult> CandidatePlans
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double BladeWidthM,
            (Double X, Double Y)? OrientStartXy = null,
            (Double X, Double Y)? OrientEndXy = null
            )
        {
            if (Cells == null) throw new ArgumentNullException(nameof(Cells));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (BladeWidthM <= 0.0) throw new ArgumentOutOfRangeException(nameof(BladeWidthM));
            if (Cells.Count == 0)
            {
                return new List<PlanResult>
                {
                    new PlanResult(
                        0.0,
                        new List<Run>(),
                        0.0,
                        0.0,
                        0.0,
                        new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>()),
                };
            }

            ExperimentalFieldEdgePolicy EdgePolicy =
                new ExperimentalFieldEdgePolicy(Grid, Grid.GridSizeM);
            List<PlanResult> Candidates = new List<PlanResult>();
            for (Double HeadingDeg = 0.0; HeadingDeg < 180.0; HeadingDeg += HEADING_STEP_DEG)
            {
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> WorkCells = Cells
                    .Where(Entry => EdgePolicy.IsParallelToEdge(
                        Entry.Cell,
                        HeadingDeg,
                        EDGE_PARALLEL_TOLERANCE_DEG))
                    .ToList();
                HashSet<((Int32 Bx, Int32 By) Cell, Double DepthM)> WorkSet =
                    new HashSet<((Int32 Bx, Int32 By) Cell, Double DepthM)>(WorkCells);
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> OmittedCells = Cells
                    .Where(Entry => !WorkSet.Contains(Entry))
                    .ToList();
                foreach (Boolean DescendingCross in new Boolean[] { false, true })
                {
                    PlanResult Candidate = BuildCandidate(
                        WorkCells,
                        Grid,
                        BladeWidthM,
                        HeadingDeg,
                        DescendingCross,
                        OrientStartXy,
                        OrientEndXy,
                        OmittedCells);
                    if (!ConservesAllocation(Cells, Candidate.AccountedCells))
                        throw new InvalidOperationException(
                            "Experimental coverage planning failed to account for the cell/depth multiset.");
                    Candidates.Add(Candidate);
                }
            }
            return Candidates
                .OrderBy(Candidate => Candidate.Score)
                .ThenBy(Candidate => Candidate.HeadingDeg)
                .ToList();
        }

        /// <summary>
        /// Tests exact cell/depth multiset conservation.
        /// </summary>
        /// <param name="Expected">Expected allocation.</param>
        /// <param name="Actual">Candidate allocation.</param>
        /// <returns>True when every cell/depth entry occurs the same number of times.</returns>
        static public Boolean ConservesAllocation
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Expected,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Actual
            )
        {
            if (Expected.Count != Actual.Count) return false;

            // Count exact depth bit patterns so conservation does not hide numeric drift.
            Dictionary<(Int32 Bx, Int32 By, Int64 DepthBits), Int32> Counts =
                new Dictionary<(Int32 Bx, Int32 By, Int64 DepthBits), Int32>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Expected)
            {
                (Int32 Bx, Int32 By, Int64 DepthBits) Key =
                    (Entry.Cell.Bx, Entry.Cell.By, BitConverter.DoubleToInt64Bits(Entry.DepthM));
                Counts.TryGetValue(Key, out Int32 Count);
                Counts[Key] = Count + 1;
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Actual)
            {
                (Int32 Bx, Int32 By, Int64 DepthBits) Key =
                    (Entry.Cell.Bx, Entry.Cell.By, BitConverter.DoubleToInt64Bits(Entry.DepthM));
                if (!Counts.TryGetValue(Key, out Int32 Count) || (Count <= 0)) return false;
                Counts[Key] = Count - 1;
            }
            return Counts.Values.All(Count => Count == 0);
        }

        /// <summary>
        /// Builds and scores one heading and cross-axis packing direction.
        /// </summary>
        private static PlanResult BuildCandidate
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double BladeWidthM,
            Double HeadingDeg,
            Boolean DescendingCross,
            (Double X, Double Y)? OrientStartXy,
            (Double X, Double Y)? OrientEndXy,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> OmittedCells
            )
        {
            if (Cells.Count == 0)
            {
                return new PlanResult(
                    HeadingDeg,
                    new List<Run>(),
                    0.0,
                    0.0,
                    OmittedCells.Count * OMITTED_CELL_WEIGHT,
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(OmittedCells));
            }
            Double Theta = HeadingDeg * Math.PI / 180.0;
            (Double X, Double Y) AlongUnit = (Math.Sin(Theta), Math.Cos(Theta));
            (Double X, Double Y) CrossUnit = (Math.Cos(Theta), -Math.Sin(Theta));
            (Double X, Double Y) Origin = CellCentroid(Cells, Grid);
            List<ProjectedCell> Projected = Project(Cells, Grid, Origin, AlongUnit, CrossUnit);
            List<List<ProjectedCell>> Groups = PackCrossAxis(Projected, BladeWidthM, DescendingCross);
            List<Run> Runs = new List<Run>();
            Double MaximumGapM = MAX_ALONG_GAP_GRID_MULTIPLIER * Grid.GridSizeM;
            foreach (List<ProjectedCell> Group in Groups)
            {
                Runs.AddRange(BuildGroupRuns(
                    Group,
                    Origin,
                    AlongUnit,
                    CrossUnit,
                    HeadingDeg,
                    Grid.GridSizeM,
                    MaximumGapM));
            }
            Runs = EnsureInFieldRuns(Runs, Grid);
            Runs = OrderRuns(Runs, OrientStartXy, OrientEndXy);

            Double ConnectorDistanceM = ConnectorDistance(Runs);
            Double AnchorDistanceM = AnchorDistance(Runs, OrientStartXy, OrientEndXy);
            Double WorkDistanceM = Runs.Sum(Run => Run.LengthM);
            Double Score = (Runs.Count * RUN_COUNT_WEIGHT)
                + (ConnectorDistanceM * CONNECTOR_WEIGHT)
                + AnchorDistanceM
                + (WorkDistanceM * WORK_DISTANCE_WEIGHT)
                + (OmittedCells.Count * OMITTED_CELL_WEIGHT);
            return new PlanResult(
                HeadingDeg,
                Runs,
                ConnectorDistanceM,
                AnchorDistanceM,
                Score,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(OmittedCells));
        }

        /// <summary>
        /// Retains valid straight runs and splits boundary-crossing runs into maximal valid sub-runs.
        /// </summary>
        private static List<Run> EnsureInFieldRuns
            (
            List<Run> Runs,
            Grid Grid
            )
        {
            List<Run> Output = new List<Run>();
            foreach (Run Run in Runs)
            {
                if (RunInsideField(Run, Grid))
                {
                    Output.Add(Run);
                    continue;
                }

                // Preserve the longest possible straight portions instead of creating one turn per cell.
                Int32 FirstCell = 0;
                while (FirstCell < Run.Cells.Count)
                {
                    Run? Selected = null;
                    Int32 SelectedLastCell = FirstCell;
                    for (Int32 LastCell = Run.Cells.Count - 1;
                        LastCell >= FirstCell;
                        LastCell -= 1)
                    {
                        Run Candidate = BuildSubRun(
                            Run.Cells.GetRange(FirstCell, LastCell - FirstCell + 1),
                            Run.HeadingDeg,
                            Grid);
                        if (!RunInsideField(Candidate, Grid)) continue;
                        Selected = Candidate;
                        SelectedLastCell = LastCell;
                        break;
                    }
                    if (Selected == null)
                    {
                        throw new InvalidOperationException(
                            "A cell-centered coverage run is outside its allocated valid cell.");
                    }
                    Output.Add(Selected);
                    FirstCell = SelectedLastCell + 1;
                }
            }
            return Output;
        }

        /// <summary>
        /// Rebuilds one straight run over a consecutive subset at the parent heading.
        /// </summary>
        private static Run BuildSubRun
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double HeadingDeg,
            Grid Grid
            )
        {
            Double HeadingRad = HeadingDeg * Math.PI / 180.0;
            (Double X, Double Y) AlongUnit = (Math.Sin(HeadingRad), Math.Cos(HeadingRad));
            (Double X, Double Y) CrossUnit = (Math.Cos(HeadingRad), -Math.Sin(HeadingRad));
            List<(Double Along, Double Cross)> Projected =
                new List<(Double Along, Double Cross)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Center = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                Projected.Add((
                    (Center.X * AlongUnit.X) + (Center.Y * AlongUnit.Y),
                    (Center.X * CrossUnit.X) + (Center.Y * CrossUnit.Y)));
            }
            Double MinimumAlong = Projected.Min(Point => Point.Along)
                - (ENDPOINT_EXTENSION_GRID_FRACTION * Grid.GridSizeM);
            Double MaximumAlong = Projected.Max(Point => Point.Along)
                + (ENDPOINT_EXTENSION_GRID_FRACTION * Grid.GridSizeM);
            Double CenterCross = 0.5 * (
                Projected.Min(Point => Point.Cross) + Projected.Max(Point => Point.Cross));
            (Double X, Double Y) Start = (
                (MinimumAlong * AlongUnit.X) + (CenterCross * CrossUnit.X),
                (MinimumAlong * AlongUnit.Y) + (CenterCross * CrossUnit.Y));
            (Double X, Double Y) End = (
                (MaximumAlong * AlongUnit.X) + (CenterCross * CrossUnit.X),
                (MaximumAlong * AlongUnit.Y) + (CenterCross * CrossUnit.Y));
            Double MaximumErrorM = Projected.Max(Point => Math.Abs(Point.Cross - CenterCross));
            return new Run(Cells, Start, End, HeadingDeg, MaximumErrorM);
        }

        /// <summary>
        /// Samples a work centerline against the raw field mask.
        /// </summary>
        private static Boolean RunInsideField
            (
            Run Run,
            Grid Grid
            )
        {
            Int32 Count = Math.Max(
                1,
                (Int32)Math.Ceiling(Run.LengthM / Math.Min(0.5, Grid.GridSizeM * 0.5)));
            for (Int32 Index = 0; Index <= Count; Index += 1)
            {
                Double Fraction = (Double)Index / Count;
                Double X = Run.StartXy.X + (Fraction * (Run.EndXy.X - Run.StartXy.X));
                Double Y = Run.StartXy.Y + (Fraction * (Run.EndXy.Y - Run.StartXy.Y));
                if (!Grid.ContainsWorldXy(X, Y)) return false;
            }
            return true;
        }

        /// <summary>
        /// Projects allocated cell centers into one candidate heading frame.
        /// </summary>
        private static List<ProjectedCell> Project
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            (Double X, Double Y) Origin,
            (Double X, Double Y) AlongUnit,
            (Double X, Double Y) CrossUnit
            )
        {
            List<ProjectedCell> Output = new List<ProjectedCell>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Center = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                Double Dx = Center.X - Origin.X;
                Double Dy = Center.Y - Origin.Y;
                Output.Add(new ProjectedCell(
                    Entry,
                    Center.X,
                    Center.Y,
                    (Dx * AlongUnit.X) + (Dy * AlongUnit.Y),
                    (Dx * CrossUnit.X) + (Dy * CrossUnit.Y)));
            }
            return Output;
        }

        /// <summary>
        /// Greedily packs projected cells into minimum-width blade corridors.
        /// </summary>
        private static List<List<ProjectedCell>> PackCrossAxis
            (
            List<ProjectedCell> Cells,
            Double BladeWidthM,
            Boolean Descending
            )
        {
            List<ProjectedCell> Remaining = Descending
                ? Cells.OrderByDescending(Cell => Cell.Cross).ThenBy(Cell => Cell.Along).ToList()
                : Cells.OrderBy(Cell => Cell.Cross).ThenBy(Cell => Cell.Along).ToList();
            List<List<ProjectedCell>> Output = new List<List<ProjectedCell>>();
            while (Remaining.Count > 0)
            {
                Double Edge = Remaining[0].Cross;
                List<ProjectedCell> Group = Remaining.Where(Cell => Descending
                    ? (Cell.Cross >= Edge - BladeWidthM - NUMERIC_TOLERANCE)
                    : (Cell.Cross <= Edge + BladeWidthM + NUMERIC_TOLERANCE)).ToList();
                Output.Add(Group);
                HashSet<ProjectedCell> Selected = new HashSet<ProjectedCell>(Group);
                Remaining = Remaining.Where(Cell => !Selected.Contains(Cell)).ToList();
            }
            return Output;
        }

        /// <summary>
        /// Splits one blade corridor at large along-axis gaps and creates straight centerlines.
        /// </summary>
        private static List<Run> BuildGroupRuns
            (
            List<ProjectedCell> Cells,
            (Double X, Double Y) Origin,
            (Double X, Double Y) AlongUnit,
            (Double X, Double Y) CrossUnit,
            Double HeadingDeg,
            Double GridSizeM,
            Double MaximumGapM
            )
        {
            List<ProjectedCell> Ordered = Cells.OrderBy(Cell => Cell.Along).ThenBy(Cell => Cell.Cross).ToList();
            List<List<ProjectedCell>> Parts = new List<List<ProjectedCell>>();
            List<ProjectedCell> Current = new List<ProjectedCell>();
            foreach (ProjectedCell Cell in Ordered)
            {
                if ((Current.Count > 0) && ((Cell.Along - Current[Current.Count - 1].Along) > MaximumGapM))
                {
                    Parts.Add(Current);
                    Current = new List<ProjectedCell>();
                }
                Current.Add(Cell);
            }
            if (Current.Count > 0) Parts.Add(Current);

            List<Run> Output = new List<Run>();
            foreach (List<ProjectedCell> Part in Parts)
            {
                Double MinimumAlong = Part.Min(Cell => Cell.Along)
                    - (ENDPOINT_EXTENSION_GRID_FRACTION * GridSizeM);
                Double MaximumAlong = Part.Max(Cell => Cell.Along)
                    + (ENDPOINT_EXTENSION_GRID_FRACTION * GridSizeM);
                Double MinimumCross = Part.Min(Cell => Cell.Cross);
                Double MaximumCross = Part.Max(Cell => Cell.Cross);
                Double CenterCross = 0.5 * (MinimumCross + MaximumCross);
                (Double X, Double Y) Start = ToWorld(Origin, AlongUnit, CrossUnit, MinimumAlong, CenterCross);
                (Double X, Double Y) End = ToWorld(Origin, AlongUnit, CrossUnit, MaximumAlong, CenterCross);
                Double MaximumErrorM = Part.Max(Cell => Math.Abs(Cell.Cross - CenterCross));
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> RunCells =
                    Part.Select(Cell => Cell.Entry).ToList();
                Output.Add(new Run(RunCells, Start, End, HeadingDeg, MaximumErrorM));
            }
            return Output;
        }

        /// <summary>
        /// Orders and orients runs from a start anchor or backward from an end anchor.
        /// </summary>
        private static List<Run> OrderRuns
            (
            List<Run> Runs,
            (Double X, Double Y)? OrientStartXy,
            (Double X, Double Y)? OrientEndXy
            )
        {
            if (Runs.Count <= 1) return Runs;
            List<Run> Remaining = new List<Run>(Runs);
            List<Run> Output = new List<Run>();

            if (OrientStartXy.HasValue)
            {
                // Build forward so each selected run starts at its endpoint nearest current travel.
                (Double X, Double Y) Current = OrientStartXy.Value;
                while (Remaining.Count > 0)
                {
                    (Int32 Index, Boolean Reverse) Choice = NearestRun(Remaining, Current, false);
                    Run Selected = Choice.Reverse ? ReverseRun(Remaining[Choice.Index]) : Remaining[Choice.Index];
                    Output.Add(Selected);
                    Current = Selected.EndXy;
                    Remaining.RemoveAt(Choice.Index);
                }
                return Output;
            }

            if (OrientEndXy.HasValue)
            {
                // Build backward so the final selected run exits nearest the haul target.
                (Double X, Double Y) Current = OrientEndXy.Value;
                while (Remaining.Count > 0)
                {
                    (Int32 Index, Boolean Reverse) Choice = NearestRun(Remaining, Current, true);
                    Run Selected = Choice.Reverse ? ReverseRun(Remaining[Choice.Index]) : Remaining[Choice.Index];
                    Output.Add(Selected);
                    Current = Selected.StartXy;
                    Remaining.RemoveAt(Choice.Index);
                }
                Output.Reverse();
                return Output;
            }

            return Runs;
        }

        /// <summary>
        /// Finds the run endpoint nearest the current forward or backward anchor.
        /// </summary>
        private static (Int32 Index, Boolean Reverse) NearestRun
            (
            List<Run> Runs,
            (Double X, Double Y) Current,
            Boolean MatchEnd
            )
        {
            Int32 BestIndex = 0;
            Boolean BestReverse = false;
            Double BestDistance = Double.PositiveInfinity;
            for (Int32 Index = 0; Index < Runs.Count; Index += 1)
            {
                Run Candidate = Runs[Index];
                Double KeepDistance = MatchEnd
                    ? Distance(Candidate.EndXy, Current)
                    : Distance(Candidate.StartXy, Current);
                Double ReverseDistance = MatchEnd
                    ? Distance(Candidate.StartXy, Current)
                    : Distance(Candidate.EndXy, Current);
                Boolean Reverse = ReverseDistance < KeepDistance;
                Double CandidateDistance = Reverse ? ReverseDistance : KeepDistance;
                if (CandidateDistance < BestDistance - NUMERIC_TOLERANCE)
                {
                    BestIndex = Index;
                    BestReverse = Reverse;
                    BestDistance = CandidateDistance;
                }
            }
            return (BestIndex, BestReverse);
        }

        /// <summary>
        /// Reverses drive direction while preserving assigned material cells.
        /// </summary>
        private static Run ReverseRun
            (
            Run Input
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Input.Cells);
            Cells.Reverse();
            Double HeadingDeg = (Input.HeadingDeg + 180.0) % 360.0;
            return new Run(Cells, Input.EndXy, Input.StartXy, HeadingDeg, Input.MaximumLateralErrorM);
        }

        /// <summary>
        /// Computes connector distance between already ordered runs.
        /// </summary>
        private static Double ConnectorDistance
            (
            List<Run> Runs
            )
        {
            Double Total = 0.0;
            for (Int32 Index = 1; Index < Runs.Count; Index += 1)
            {
                Total += Distance(Runs[Index - 1].EndXy, Runs[Index].StartXy);
            }
            return Total;
        }

        /// <summary>
        /// Computes travel from optional anchors to the selected coverage endpoints.
        /// </summary>
        private static Double AnchorDistance
            (
            List<Run> Runs,
            (Double X, Double Y)? OrientStartXy,
            (Double X, Double Y)? OrientEndXy
            )
        {
            if (Runs.Count == 0) return 0.0;
            Double Total = 0.0;
            if (OrientStartXy.HasValue) Total += Distance(OrientStartXy.Value, Runs[0].StartXy);
            if (OrientEndXy.HasValue) Total += Distance(Runs[Runs.Count - 1].EndXy, OrientEndXy.Value);
            return Total;
        }

        /// <summary>
        /// Computes the world centroid of allocated cell centers.
        /// </summary>
        private static (Double X, Double Y) CellCentroid
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            Double X = 0.0;
            Double Y = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Center = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                X += Center.X;
                Y += Center.Y;
            }
            return (X / Cells.Count, Y / Cells.Count);
        }

        /// <summary>
        /// Converts candidate-frame coordinates back to world coordinates.
        /// </summary>
        private static (Double X, Double Y) ToWorld
            (
            (Double X, Double Y) Origin,
            (Double X, Double Y) AlongUnit,
            (Double X, Double Y) CrossUnit,
            Double Along,
            Double Cross
            )
        {
            return (
                Origin.X + (Along * AlongUnit.X) + (Cross * CrossUnit.X),
                Origin.Y + (Along * AlongUnit.Y) + (Cross * CrossUnit.Y));
        }

        /// <summary>
        /// Computes planar distance in metres.
        /// </summary>
        private static Double Distance
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
