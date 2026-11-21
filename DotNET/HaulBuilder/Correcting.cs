using System.Globalization;

namespace HaulBuilder
{
    /// <summary>
    /// Coverage passes after thin-cut. This is the last stage. Lines run the
    /// length of the field along the mean flow,
    /// a pan width apart, as a serpentine. The blade is at the target. High cells
    /// fill the bowl and low cells take a spread lift. While the field still needs
    /// at least a quarter bowl of fill, load left at the end of a line goes to the
    /// nearest cell below the target. Once that fill need is under a quarter bowl,
    /// surplus mode cuts the highs and spreads the load along the line, at most
    /// 3 mm on a cell already between the target and +10 mm. Load still in the
    /// bowl at the end of a line is carried to the next line.
    /// At most two coverages run, and the search stops when correcting finishes.
    /// </summary>
    static public class Correcting
    {
        /// <summary>Metres of overlap between neighbouring passes.</summary>
        public const Double OVERLAP_M = 0.3;

        /// <summary>Deepest compacted lift one visit may place.</summary>
        public const Double SPREAD_LIFT_M = 0.10;

        /// <summary>Surplus may raise a cell up to this many millimetres above the target.</summary>
        public const Double SURPLUS_MAX_MM = 10.0;

        /// <summary>
        /// In surplus mode, one pass may add at most this many millimetres to a cell
        /// that is already between the target and <see cref="SURPLUS_MAX_MM"/>.
        /// </summary>
        public const Double SURPLUS_STEP_MM = 3.0;

        /// <summary>
        /// Surplus mode is on when the bank fill still below the target is under
        /// this share of the bowl.
        /// </summary>
        public const Double SURPLUS_FILL_FRACTION = 0.25;

        /// <summary>Pass speed in metres per second. Used only for the hour estimate.</summary>
        public const Double V_PASS_MS = 2.2;

        /// <summary>Seconds charged for one serpentine turn.</summary>
        public const Double T_TURN_S = 20.0;

        /// <summary>Coverages of the whole field. The search does not run a third.</summary>
        public const Int32 MAX_COVERAGES = 2;

        /// <summary>A group longer than this, in metres, is not ready for planing.</summary>
        public const Double PLANE_LEN_M = 12.0;

        /// <summary>Cells outside this band are the coverage groups.</summary>
        public const Double GROUP_BAND_MM = 30.0;

        /// <summary>A group deeper than this needs another coverage, until the coverage cap.</summary>
        public const Double DEEP_MM = 50.0;

        /// <summary>Cells outside this band are counted in the coverage report.</summary>
        public const Double FAR_MM = 100.0;

        /// <summary>Keeps two reports from writing the same temp file at once.</summary>
        private static readonly Object ReportGate = new Object();

        /// <summary>
        /// Drives the coverages on a signed residual. Positive is high.
        /// The array is changed in place. The bowl starts empty and is carried
        /// from line to line, and into the next coverage, so a load is not dropped.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres, indexed [row, col].</param>
        /// <param name="Mask">True on field cells. This is mask E.</param>
        /// <param name="FlowEast">Unit east component of the flow arrow.</param>
        /// <param name="FlowNorth">Unit north component of the flow arrow.</param>
        /// <param name="HasArrow">True where the arrow is real.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="PanWidthM">Pan width in metres.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="FillToBank">Bank volume consumed by one compacted volume of fill.</param>
        /// <param name="InsideFraction">
        /// Share of each cell inside the field. Null means every masked cell is fully inside.
        /// </param>
        /// <param name="MaxCoverages">Coverages to allow. The production cap is 2.</param>
        /// <returns>The coverage reports.</returns>
        public static CorrectingResult Run
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Double[,] FlowEast,
            Double[,] FlowNorth,
            Boolean[,] HasArrow,
            Double CellSizeM,
            Double PanWidthM,
            Double BowlBcy,
            Double FillToBank,
            Double[,]? InsideFraction = null,
            Int32 MaxCoverages = MAX_COVERAGES
            )
        {
            if (ResidualMm == null) throw new ArgumentNullException(nameof(ResidualMm));
            if (Mask == null) throw new ArgumentNullException(nameof(Mask));
            if (FlowEast == null) throw new ArgumentNullException(nameof(FlowEast));
            if (FlowNorth == null) throw new ArgumentNullException(nameof(FlowNorth));
            if (HasArrow == null) throw new ArgumentNullException(nameof(HasArrow));
            if (CellSizeM <= 0.0) throw new ArgumentOutOfRangeException(nameof(CellSizeM));
            if (PanWidthM <= OVERLAP_M) throw new ArgumentOutOfRangeException(nameof(PanWidthM));
            if (BowlBcy <= 0.0) throw new ArgumentOutOfRangeException(nameof(BowlBcy));
            if (FillToBank <= 0.0) throw new ArgumentOutOfRangeException(nameof(FillToBank));
            if (MaxCoverages < 1) throw new ArgumentOutOfRangeException(nameof(MaxCoverages));
            Int32 Rows = ResidualMm.GetLength(0);
            Int32 Cols = ResidualMm.GetLength(1);
            // The bowl is bank cubic metres. Yards are the caller's unit.
            Double BowlM3 = BowlBcy / ResidualClusters.M3_TO_CY;
            Double Area = CellSizeM * CellSizeM;
            Double Bowl = 0.0;
            List<CorrectingCoverage> Coverages = new List<CorrectingCoverage>();
            Double CutM3 = 0.0;
            Double FillM3 = 0.0;
            Double SurplusM3 = 0.0;
            Int32 Done = 0;
            for (Int32 Number = 1; Number <= MaxCoverages; Number += 1)
            {
                Tally Tally = new Tally();
                DriveCoverage(
                    ResidualMm,
                    Mask,
                    FlowEast,
                    FlowNorth,
                    HasArrow,
                    InsideFraction,
                    Rows,
                    Cols,
                    CellSizeM,
                    PanWidthM,
                    Area,
                    BowlM3,
                    FillToBank,
                    ref Bowl,
                    Tally);
                CutM3 += Tally.CutM3;
                FillM3 += Tally.FillM3;
                SurplusM3 += Tally.SurplusM3;
                List<CorrectingGroup> Groups = GroupsBeyondBand(
                    ResidualMm, Mask, InsideFraction, Rows, Cols, Area, FillToBank, CellSizeM);
                CorrectingCoverage Coverage = new CorrectingCoverage(
                    Number,
                    PercentWithin(ResidualMm, Mask, Rows, Cols, GROUP_BAND_MM),
                    PercentWithin(ResidualMm, Mask, Rows, Cols, DEEP_MM),
                    CountBeyond(ResidualMm, Mask, Rows, Cols, FAR_MM),
                    Groups,
                    Tally.SurplusM3 * ResidualClusters.M3_TO_CY,
                    Tally.DistanceM,
                    Tally.Turns,
                    Hours(Tally.DistanceM, Tally.Turns),
                    Clone(ResidualMm));
                Coverages.Add(Coverage);
                Done = Number;
                // Another coverage only while a group is long or deep, and only up to the cap.
                if ((Number == MaxCoverages) || !NeedsAnotherCoverage(Groups))
                {
                    break;
                }
            }
            List<CorrectingGroup> Failed = new List<CorrectingGroup>();
            if (Coverages.Count > 0)
            {
                foreach (CorrectingGroup Group in Coverages[Coverages.Count - 1].Groups)
                {
                    if (FailsPlaningGate(Group))
                    {
                        Failed.Add(Group);
                    }
                }
            }
            CorrectingResult Result = new CorrectingResult(
                Coverages,
                Failed.Count == 0,
                Failed,
                CutM3,
                FillM3,
                SurplusM3,
                Bowl,
                Done);
            Report(Result);
            return Result;
        }

        /// <summary>
        /// Runs correcting on the search residual and appends balanced passes.
        /// A pass records the net cut and the fill that was placed on low cells.
        /// Surplus left on a high cell is the cut that pass does not record.
        /// </summary>
        /// <param name="Grid">Field grid. Mask E is its valid cells.</param>
        /// <param name="Flow">Flow arrows.</param>
        /// <param name="Ledger">Yard factors and the required depths.</param>
        /// <param name="CutLeft">Remaining cut. Replaced by the residual after coverage.</param>
        /// <param name="FillLeft">Remaining fill. Replaced by the residual after coverage.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="PanWidthM">Pan width in metres.</param>
        /// <param name="Passes">Found-order routes. Coverage trips are appended.</param>
        /// <returns>The coverage reports.</returns>
        internal static CorrectingResult Apply
            (
            Grid Grid,
            FlowField Flow,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            Double PanWidthM,
            List<Pass> Passes
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Flow == null) throw new ArgumentNullException(nameof(Flow));
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            if (CutLeft == null) throw new ArgumentNullException(nameof(CutLeft));
            if (FillLeft == null) throw new ArgumentNullException(nameof(FillLeft));
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            Int32 Rows = Grid.Height;
            Int32 Cols = Grid.Width;
            Double[,] Start = new Double[Rows, Cols];
            Double[,] Residual = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (!Grid.Valid[Row, Col])
                    {
                        continue;
                    }
                    Double Mm = (CutLeft[Row, Col] - FillLeft[Row, Col]) * 1000.0;
                    Start[Row, Col] = Mm;
                    Residual[Row, Col] = Mm;
                }
            }
            CorrectingResult Coverage = Run(
                Residual,
                Grid.Valid,
                Flow.Fx,
                Flow.Fy,
                Flow.HasArrow,
                Grid.GridSizeM,
                PanWidthM,
                BowlBcy,
                Grid.Compaction,
                null,
                MAX_COVERAGES);
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (!Grid.Valid[Row, Col])
                    {
                        continue;
                    }
                    Double Mm = Residual[Row, Col];
                    if (Mm >= 0.0)
                    {
                        CutLeft[Row, Col] = Mm / 1000.0;
                        FillLeft[Row, Col] = 0.0;
                    }
                    else
                    {
                        CutLeft[Row, Col] = 0.0;
                        FillLeft[Row, Col] = -Mm / 1000.0;
                    }
                }
            }
            EmitPasses(Start, Residual, Grid, Ledger, BowlBcy, Passes);
            return Coverage;
        }

        /// <summary>
        /// One coverage: parallel lines the length of the field, along the mean arrow.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="FlowEast">Unit east arrows.</param>
        /// <param name="FlowNorth">Unit north arrows.</param>
        /// <param name="HasArrow">True where the arrow is real.</param>
        /// <param name="InsideFraction">Cell fraction, or null for 1.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="PanWidthM">Pan width in metres.</param>
        /// <param name="Area">Cell area in square metres.</param>
        /// <param name="BowlM3">Bowl capacity in bank cubic metres.</param>
        /// <param name="FillToBank">Bank per compacted volume.</param>
        /// <param name="Bowl">Load in the bowl, in bank cubic metres.</param>
        /// <param name="Tally">Distance, turns, and volumes for this coverage.</param>
        private static void DriveCoverage
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Double[,] FlowEast,
            Double[,] FlowNorth,
            Boolean[,] HasArrow,
            Double[,]? InsideFraction,
            Int32 Rows,
            Int32 Cols,
            Double CellSizeM,
            Double PanWidthM,
            Double Area,
            Double BowlM3,
            Double FillToBank,
            ref Double Bowl,
            Tally Tally
            )
        {
            Boolean HavePrevious = false;
            Int32 PreviousRow = 0;
            Int32 PreviousCol = 0;
            List<List<(Int32 Row, Int32 Col)>> Lines = LinesAcrossField(
                Mask,
                FlowEast,
                FlowNorth,
                HasArrow,
                Rows,
                Cols,
                CellSizeM,
                PanWidthM);
            foreach (List<(Int32 Row, Int32 Col)> Line in Lines)
            {
                if (Line.Count == 0)
                {
                    continue;
                }
                if (HavePrevious)
                {
                    // The turn joins two cells that are both on the field.
                    Tally.Turns += 1;
                    Tally.DistanceM += StepMetres(
                        PreviousRow, PreviousCol, Line[0].Row, Line[0].Col, CellSizeM);
                }
                for (Int32 Index = 1; Index < Line.Count; Index += 1)
                {
                    Tally.DistanceM += StepMetres(
                        Line[Index - 1].Row,
                        Line[Index - 1].Col,
                        Line[Index].Row,
                        Line[Index].Col,
                        CellSizeM);
                }
                DriveLine(
                    Line,
                    ResidualMm,
                    Mask,
                    InsideFraction,
                    Rows,
                    Cols,
                    Area,
                    BowlM3,
                    FillToBank,
                    ref Bowl,
                    Tally);
                PreviousRow = Line[Line.Count - 1].Row;
                PreviousCol = Line[Line.Count - 1].Col;
                HavePrevious = true;
            }
        }

        /// <summary>
        /// Parallel lines the length of the field. The direction is the mean arrow.
        /// Odd lines run back along that direction. A line is not cut into blocks,
        /// so a load can travel from a high to a low on the same pass.
        /// </summary>
        /// <param name="Mask">Mask E.</param>
        /// <param name="FlowEast">Unit east arrows.</param>
        /// <param name="FlowNorth">Unit north arrows.</param>
        /// <param name="HasArrow">True where the arrow is real.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="PanWidthM">Pan width in metres.</param>
        /// <returns>The lines, serpentine order, masked cells only.</returns>
        private static List<List<(Int32 Row, Int32 Col)>> LinesAcrossField
            (
            Boolean[,] Mask,
            Double[,] FlowEast,
            Double[,] FlowNorth,
            Boolean[,] HasArrow,
            Int32 Rows,
            Int32 Cols,
            Double CellSizeM,
            Double PanWidthM
            )
        {
            Int32 Row0 = 0;
            Int32 Col0 = 0;
            Int32 Row1 = Rows;
            Int32 Col1 = Cols;
            Double East = 0.0;
            Double North = 0.0;
            for (Int32 Row = Row0; Row < Row1; Row += 1)
            {
                for (Int32 Col = Col0; Col < Col1; Col += 1)
                {
                    if (!Mask[Row, Col] || !HasArrow[Row, Col])
                    {
                        continue;
                    }
                    East += FlowEast[Row, Col];
                    North += FlowNorth[Row, Col];
                }
            }
            Double DirCol;
            Double DirRow;
            Double Magnitude = Math.Sqrt((East * East) + (North * North));
            if (Magnitude < 1e-9)
            {
                // No arrow. North matches a zero atan2 heading.
                DirCol = 0.0;
                DirRow = 1.0;
            }
            else
            {
                DirCol = East / Magnitude;
                DirRow = North / Magnitude;
            }
            Double PerpCol = -DirRow;
            Double PerpRow = DirCol;
            Double SpacingCells = (PanWidthM - OVERLAP_M) / CellSizeM;
            Dictionary<Int32, List<(Int32 Row, Int32 Col, Double Along)>> Bins =
                new Dictionary<Int32, List<(Int32 Row, Int32 Col, Double Along)>>();
            for (Int32 Row = Row0; Row < Row1; Row += 1)
            {
                for (Int32 Col = Col0; Col < Col1; Col += 1)
                {
                    if (!Mask[Row, Col])
                    {
                        continue;
                    }
                    Double Perp = ((Col + 0.5) * PerpCol) + ((Row + 0.5) * PerpRow);
                    Int32 Key = (Int32)Math.Floor(Perp / SpacingCells);
                    Double Along = ((Col + 0.5) * DirCol) + ((Row + 0.5) * DirRow);
                    if (!Bins.TryGetValue(Key, out List<(Int32 Row, Int32 Col, Double Along)>? Bin) || (Bin == null))
                    {
                        Bin = new List<(Int32 Row, Int32 Col, Double Along)>();
                        Bins[Key] = Bin;
                    }
                    Bin.Add((Row, Col, Along));
                }
            }
            List<Int32> Keys = new List<Int32>(Bins.Keys);
            Keys.Sort();
            List<List<(Int32 Row, Int32 Col)>> Lines = new List<List<(Int32 Row, Int32 Col)>>();
            Boolean Reverse = false;
            foreach (Int32 Key in Keys)
            {
                List<(Int32 Row, Int32 Col, Double Along)> Bin = Bins[Key];
                Bin.Sort((Left, Right) => Left.Along.CompareTo(Right.Along));
                if (Reverse)
                {
                    Bin.Reverse();
                }
                Reverse = !Reverse;
                List<(Int32 Row, Int32 Col)> Line = new List<(Int32 Row, Int32 Col)>(Bin.Count);
                foreach ((Int32 Row, Int32 Col, Double Along) Cell in Bin)
                {
                    Line.Add((Cell.Row, Cell.Col));
                }
                Lines.Add(Line);
            }
            return Lines;
        }

        /// <summary>
        /// Bank volume of every masked cell still below the target.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres. Negative is fill.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="InsideFraction">Cell fraction, or null for 1.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="Area">Cell area in square metres.</param>
        /// <param name="FillToBank">Bank per compacted volume.</param>
        /// <returns>Fill need in bank cubic metres.</returns>
        private static Double FillNeedBankM3
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Double[,]? InsideFraction,
            Int32 Rows,
            Int32 Cols,
            Double Area,
            Double FillToBank
            )
        {
            Double Need = 0.0;
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (!Mask[Row, Col])
                    {
                        continue;
                    }
                    Double Residual = ResidualMm[Row, Col];
                    if (Residual >= -1e-6)
                    {
                        continue;
                    }
                    Double Fraction = FractionOf(InsideFraction, Row, Col);
                    if (Fraction <= 0.0)
                    {
                        continue;
                    }
                    Need += (-Residual / 1000.0) * Area * Fraction * FillToBank;
                }
            }
            return Need;
        }

        /// <summary>
        /// One line. High cells fill the bowl. Low cells take one spread lift.
        /// A full bowl passes the remaining highs. While the field still needs a
        /// quarter bowl of fill, load left at the end goes to the nearest cell
        /// below the target. In surplus mode the load is spread along the line
        /// and whatever remains is carried to the next line.
        /// </summary>
        /// <param name="Line">Cells in drive order.</param>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="InsideFraction">Cell fraction, or null for 1.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="Area">Cell area in square metres.</param>
        /// <param name="BowlM3">Bowl capacity in bank cubic metres.</param>
        /// <param name="FillToBank">Bank per compacted volume.</param>
        /// <param name="Bowl">Load in the bowl.</param>
        /// <param name="Tally">Volumes for this coverage.</param>
        private static void DriveLine
            (
            List<(Int32 Row, Int32 Col)> Line,
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Double[,]? InsideFraction,
            Int32 Rows,
            Int32 Cols,
            Double Area,
            Double BowlM3,
            Double FillToBank,
            ref Double Bowl,
            Tally Tally
            )
        {
            Double FillNeed = FillNeedBankM3(
                ResidualMm, Mask, InsideFraction, Rows, Cols, Area, FillToBank);
            Double Quarter = SURPLUS_FILL_FRACTION * BowlM3;
            foreach ((Int32 Row, Int32 Col) Cell in Line)
            {
                Double Fraction = FractionOf(InsideFraction, Cell.Row, Cell.Col);
                if (Fraction <= 0.0)
                {
                    continue;
                }
                Double Residual = ResidualMm[Cell.Row, Cell.Col];
                // Under a quarter bowl of fill, the cap is a thin spread, not a cut.
                Boolean Surplus = FillNeed < Quarter;
                Boolean AboveBand = Residual > (SURPLUS_MAX_MM + 1e-6);
                if ((Residual > 1e-6) && (Bowl < (BowlM3 - 1e-12)) && (!Surplus || AboveBand))
                {
                    Double CutMm = Residual;
                    if (Surplus)
                    {
                        Double RoomM = BowlM3 - Bowl;
                        Double FullM = (Residual / 1000.0) * Area * Fraction;
                        // A partial cut stops at +10 so the cell stays a high for the next pass.
                        if (FullM > (RoomM + 1e-12))
                        {
                            CutMm = Residual - SURPLUS_MAX_MM;
                        }
                    }
                    Double Bank = (CutMm / 1000.0) * Area * Fraction;
                    Double Take = Math.Min(Bank, BowlM3 - Bowl);
                    if (Take > 0.0)
                    {
                        Double Depth = Take / (Area * Fraction);
                        ResidualMm[Cell.Row, Cell.Col] = Residual - (Depth * 1000.0);
                        Bowl += Take;
                        Tally.CutM3 += Take;
                    }
                }
                else if ((Residual < -1e-6) && (Bowl > 1e-12))
                {
                    // One lift while the machine is on the cell. Deeper holes wait for another visit.
                    Double Placed = DropOnCell(
                        ResidualMm,
                        InsideFraction,
                        Cell.Row,
                        Cell.Col,
                        Area,
                        FillToBank,
                        ref Bowl,
                        Tally);
                    FillNeed -= Placed;
                }
                else if (Surplus && (Residual >= 0.0) && (Residual < (SURPLUS_MAX_MM - 1e-9)) && (Bowl > 1e-12))
                {
                    SpreadStep(
                        ResidualMm,
                        InsideFraction,
                        Cell.Row,
                        Cell.Col,
                        Area,
                        FillToBank,
                        ref Bowl,
                        Tally);
                }
            }
            // Surplus mode keeps the load for the next line. A real fill still takes the leftover.
            if (FillNeed >= Quarter)
            {
                (Int32 Row, Int32 Col) End = Line[Line.Count - 1];
                PlaceLeftover(
                    ResidualMm,
                    Mask,
                    InsideFraction,
                    Rows,
                    Cols,
                    End.Row,
                    End.Col,
                    Area,
                    FillToBank,
                    ref Bowl,
                    Tally);
            }
        }

        /// <summary>
        /// Adds at most one surplus step to a cell already between the target and +10 mm.
        /// The cell is not raised past +10 mm.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="InsideFraction">Cell fraction, or null for 1.</param>
        /// <param name="Row">Row.</param>
        /// <param name="Col">Column.</param>
        /// <param name="Area">Cell area in square metres.</param>
        /// <param name="FillToBank">Bank per compacted volume.</param>
        /// <param name="Bowl">Load in the bowl.</param>
        /// <param name="Tally">Volumes for this coverage.</param>
        private static void SpreadStep
            (
            Double[,] ResidualMm,
            Double[,]? InsideFraction,
            Int32 Row,
            Int32 Col,
            Double Area,
            Double FillToBank,
            ref Double Bowl,
            Tally Tally
            )
        {
            Double Fraction = FractionOf(InsideFraction, Row, Col);
            if ((Fraction <= 0.0) || (Bowl <= 1e-12))
            {
                return;
            }
            Double Residual = ResidualMm[Row, Col];
            if ((Residual < 0.0) || (Residual >= SURPLUS_MAX_MM))
            {
                return;
            }
            Double RoomMm = Math.Min(SURPLUS_STEP_MM, SURPLUS_MAX_MM - Residual);
            if (RoomMm <= 1e-9)
            {
                return;
            }
            Double BankRoom = (RoomMm / 1000.0) * Area * Fraction * FillToBank;
            Double Take = Math.Min(BankRoom, Bowl);
            if (Take <= 0.0)
            {
                return;
            }
            Double Depth = Take / (Area * Fraction * FillToBank);
            ResidualMm[Row, Col] = Residual + (Depth * 1000.0);
            Bowl -= Take;
            Tally.SurplusM3 += Take;
        }

        /// <summary>
        /// Puts a leftover load on the nearest cell below the target.
        /// Used only while the field still needs at least a quarter bowl of fill.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="InsideFraction">Cell fraction, or null for 1.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="FromRow">The cell the machine just finished.</param>
        /// <param name="FromCol">The column the machine just finished.</param>
        /// <param name="Area">Cell area in square metres.</param>
        /// <param name="FillToBank">Bank per compacted volume.</param>
        /// <param name="Bowl">Load in the bowl.</param>
        /// <param name="Tally">Volumes for this coverage.</param>
        private static void PlaceLeftover
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Double[,]? InsideFraction,
            Int32 Rows,
            Int32 Cols,
            Int32 FromRow,
            Int32 FromCol,
            Double Area,
            Double FillToBank,
            ref Double Bowl,
            Tally Tally
            )
        {
            while (Bowl > 1e-12)
            {
                if (!NearestLow(ResidualMm, Mask, Rows, Cols, FromRow, FromCol, out Int32 Row, out Int32 Col))
                {
                    break;
                }
                Double Before = Bowl;
                DropOnCell(
                    ResidualMm,
                    InsideFraction,
                    Row,
                    Col,
                    Area,
                    FillToBank,
                    ref Bowl,
                    Tally);
                // A cell that cannot take soil would spin this loop.
                if (Bowl >= (Before - 1e-12))
                {
                    break;
                }
            }
        }

        /// <summary>
        /// Drops one lift on a cell below the target. The drop stops at the target.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="InsideFraction">Cell fraction, or null for 1.</param>
        /// <param name="Row">Row.</param>
        /// <param name="Col">Column.</param>
        /// <param name="Area">Cell area in square metres.</param>
        /// <param name="FillToBank">Bank per compacted volume.</param>
        /// <param name="Bowl">Load in the bowl.</param>
        /// <param name="Tally">Volumes for this coverage.</param>
        /// <returns>Bank cubic metres placed. Zero when the cell cannot take any.</returns>
        private static Double DropOnCell
            (
            Double[,] ResidualMm,
            Double[,]? InsideFraction,
            Int32 Row,
            Int32 Col,
            Double Area,
            Double FillToBank,
            ref Double Bowl,
            Tally Tally
            )
        {
            Double Fraction = FractionOf(InsideFraction, Row, Col);
            if ((Fraction <= 0.0) || (Bowl <= 1e-12))
            {
                return 0.0;
            }
            Double Residual = ResidualMm[Row, Col];
            if (Residual >= -1e-6)
            {
                return 0.0;
            }
            Double RoomM = Math.Min(-Residual / 1000.0, SPREAD_LIFT_M);
            Double BankRoom = RoomM * Area * Fraction * FillToBank;
            Double Take = Math.Min(BankRoom, Bowl);
            if (Take <= 0.0)
            {
                return 0.0;
            }
            Double Depth = Take / (Area * Fraction * FillToBank);
            ResidualMm[Row, Col] = Residual + (Depth * 1000.0);
            Bowl -= Take;
            Tally.FillM3 += Take;
            return Take;
        }

        /// <summary>
        /// The nearest masked cell below the target.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="FromRow">Origin row.</param>
        /// <param name="FromCol">Origin column.</param>
        /// <param name="Row">The chosen row.</param>
        /// <param name="Col">The chosen column.</param>
        /// <returns>True when a cell was found.</returns>
        private static Boolean NearestLow
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Int32 Rows,
            Int32 Cols,
            Int32 FromRow,
            Int32 FromCol,
            out Int32 Row,
            out Int32 Col
            )
        {
            Row = -1;
            Col = -1;
            Double Best = Double.PositiveInfinity;
            for (Int32 NextRow = 0; NextRow < Rows; NextRow += 1)
            {
                for (Int32 NextCol = 0; NextCol < Cols; NextCol += 1)
                {
                    if (!Mask[NextRow, NextCol])
                    {
                        continue;
                    }
                    if (ResidualMm[NextRow, NextCol] >= -1e-6)
                    {
                        continue;
                    }
                    Double Dy = NextRow - FromRow;
                    Double Dx = NextCol - FromCol;
                    Double Distance = (Dy * Dy) + (Dx * Dx);
                    if ((Distance < Best)
                        || ((Distance == Best) && ((NextRow < Row) || ((NextRow == Row) && (NextCol < Col)))))
                    {
                        Best = Distance;
                        Row = NextRow;
                        Col = NextCol;
                    }
                }
            }
            return Row >= 0;
        }

        /// <summary>
        /// True when another coverage can still help. A long group or a deep group qualifies.
        /// </summary>
        /// <param name="Groups">Groups outside +/-30 mm.</param>
        /// <returns>True when the next coverage should run.</returns>
        private static Boolean NeedsAnotherCoverage
            (
            List<CorrectingGroup> Groups
            )
        {
            foreach (CorrectingGroup Group in Groups)
            {
                if ((Group.LengthM > PLANE_LEN_M) || (Group.PeakMm > DEEP_MM))
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// True when a group is not shorter than 12 m or is outside +/-50 mm.
        /// Another coverage can still help, until the coverage cap.
        /// </summary>
        /// <param name="Group">One group outside +/-30 mm.</param>
        /// <returns>True when the group is still long or deep.</returns>
        private static Boolean FailsPlaningGate
            (
            CorrectingGroup Group
            )
        {
            return (Group.LengthM >= PLANE_LEN_M) || (Group.PeakMm > DEEP_MM);
        }

        /// <summary>
        /// Share of masked cells whose residual is inside the band, as a percent.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="BandMm">Half-width of the band.</param>
        /// <returns>Percent from 0 to 100.</returns>
        private static Double PercentWithin
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Int32 Rows,
            Int32 Cols,
            Double BandMm
            )
        {
            Int32 Inside = 0;
            Int32 Total = 0;
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (!Mask[Row, Col])
                    {
                        continue;
                    }
                    Total += 1;
                    if (Math.Abs(ResidualMm[Row, Col]) <= BandMm)
                    {
                        Inside += 1;
                    }
                }
            }
            if (Total == 0)
            {
                return 0.0;
            }
            return (100.0 * Inside) / Total;
        }

        /// <summary>
        /// Masked cells whose residual is outside the band.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="BandMm">Half-width of the band.</param>
        /// <returns>The cell count.</returns>
        private static Int32 CountBeyond
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Int32 Rows,
            Int32 Cols,
            Double BandMm
            )
        {
            Int32 Count = 0;
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (Mask[Row, Col] && (Math.Abs(ResidualMm[Row, Col]) > BandMm))
                    {
                        Count += 1;
                    }
                }
            }
            return Count;
        }

        /// <summary>
        /// Eight-connected groups outside +/-30 mm, largest bank yards first.
        /// Length is the longer side of the block of cells.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="InsideFraction">Cell fraction, or null for 1.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="Area">Cell area in square metres.</param>
        /// <param name="FillToBank">Bank per compacted volume.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <returns>The groups.</returns>
        private static List<CorrectingGroup> GroupsBeyondBand
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Double[,]? InsideFraction,
            Int32 Rows,
            Int32 Cols,
            Double Area,
            Double FillToBank,
            Double CellSizeM
            )
        {
            Boolean[,] Seen = new Boolean[Rows, Cols];
            List<CorrectingGroup> Groups = new List<CorrectingGroup>();
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    if (Seen[Row, Col] || !Mask[Row, Col])
                    {
                        continue;
                    }
                    if (Math.Abs(ResidualMm[Row, Col]) <= GROUP_BAND_MM)
                    {
                        continue;
                    }
                    Groups.Add(Flood(
                        ResidualMm,
                        Mask,
                        InsideFraction,
                        Seen,
                        Rows,
                        Cols,
                        Area,
                        FillToBank,
                        CellSizeM,
                        Row,
                        Col));
                }
            }
            Groups.Sort((CorrectingGroup Left, CorrectingGroup Right) => Right.Bcy.CompareTo(Left.Bcy));
            return Groups;
        }

        /// <summary>
        /// One group outside +/-30 mm.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Mask">Mask E.</param>
        /// <param name="InsideFraction">Cell fraction, or null for 1.</param>
        /// <param name="Seen">Cells already given to a group.</param>
        /// <param name="Rows">Row count.</param>
        /// <param name="Cols">Column count.</param>
        /// <param name="Area">Cell area in square metres.</param>
        /// <param name="FillToBank">Bank per compacted volume.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="StartRow">Seed row.</param>
        /// <param name="StartCol">Seed column.</param>
        /// <returns>The group.</returns>
        private static CorrectingGroup Flood
            (
            Double[,] ResidualMm,
            Boolean[,] Mask,
            Double[,]? InsideFraction,
            Boolean[,] Seen,
            Int32 Rows,
            Int32 Cols,
            Double Area,
            Double FillToBank,
            Double CellSizeM,
            Int32 StartRow,
            Int32 StartCol
            )
        {
            Int32 Cells = 0;
            Double Bcy = 0.0;
            Double Peak = 0.0;
            Int32 MinRow = StartRow;
            Int32 MaxRow = StartRow;
            Int32 MinCol = StartCol;
            Int32 MaxCol = StartCol;
            Queue<(Int32 Row, Int32 Col)> Waiting = new Queue<(Int32 Row, Int32 Col)>();
            Waiting.Enqueue((StartRow, StartCol));
            Seen[StartRow, StartCol] = true;
            while (Waiting.Count > 0)
            {
                (Int32 Row, Int32 Col) Cell = Waiting.Dequeue();
                Double Mm = ResidualMm[Cell.Row, Cell.Col];
                Double AbsMm = Math.Abs(Mm);
                Cells += 1;
                if (AbsMm > Peak)
                {
                    Peak = AbsMm;
                }
                Double Fraction = FractionOf(InsideFraction, Cell.Row, Cell.Col);
                Double Bank = (AbsMm / 1000.0) * Area * Fraction;
                if (Mm < 0.0)
                {
                    Bank *= FillToBank;
                }
                Bcy += Bank * ResidualClusters.M3_TO_CY;
                if (Cell.Row < MinRow)
                {
                    MinRow = Cell.Row;
                }
                if (Cell.Row > MaxRow)
                {
                    MaxRow = Cell.Row;
                }
                if (Cell.Col < MinCol)
                {
                    MinCol = Cell.Col;
                }
                if (Cell.Col > MaxCol)
                {
                    MaxCol = Cell.Col;
                }
                for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                {
                    for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                    {
                        if ((Dx == 0) && (Dy == 0))
                        {
                            continue;
                        }
                        Int32 NextRow = Cell.Row + Dy;
                        Int32 NextCol = Cell.Col + Dx;
                        if ((NextRow < 0) || (NextCol < 0) || (NextRow >= Rows) || (NextCol >= Cols))
                        {
                            continue;
                        }
                        if (Seen[NextRow, NextCol] || !Mask[NextRow, NextCol])
                        {
                            continue;
                        }
                        if (Math.Abs(ResidualMm[NextRow, NextCol]) <= GROUP_BAND_MM)
                        {
                            continue;
                        }
                        Seen[NextRow, NextCol] = true;
                        Waiting.Enqueue((NextRow, NextCol));
                    }
                }
            }
            Double LengthM = Math.Max(MaxRow - MinRow + 1, MaxCol - MinCol + 1) * CellSizeM;
            return new CorrectingGroup(Cells, Bcy, Peak, LengthM);
        }

        /// <summary>
        /// Appends bowl-sized passes for the net cut and the fill placed on lows.
        /// The two sides are trimmed to the same ledger yards.
        /// </summary>
        /// <param name="StartMm">Residual before correcting.</param>
        /// <param name="EndMm">Residual after correcting.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="Passes">Routes to append.</param>
        private static void EmitPasses
            (
            Double[,] StartMm,
            Double[,] EndMm,
            Grid Grid,
            HaulWorkLedger Ledger,
            Double BowlBcy,
            List<Pass> Passes
            )
        {
            Double MaxDepthM = Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            if (MaxDepthM <= 0.0)
            {
                return;
            }
            List<(Int32 Col, Int32 Row, Double DepthM)> Cuts =
                new List<(Int32 Col, Int32 Row, Double DepthM)>();
            List<(Int32 Col, Int32 Row, Double DepthM)> Fills =
                new List<(Int32 Col, Int32 Row, Double DepthM)>();
            Double CutBcy = 0.0;
            Double FillBcy = 0.0;
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    if (!Grid.Valid[Row, Col])
                    {
                        continue;
                    }
                    Double Start = StartMm[Row, Col];
                    Double End = EndMm[Row, Col];
                    // Net drop only. Surplus put back on a high cell is not a second fill.
                    if ((Start > 1e-4) && (End < (Start - 1e-4)))
                    {
                        Double Depth = (Start - Math.Max(End, 0.0)) / 1000.0;
                        if (Depth > 1e-9)
                        {
                            Cuts.Add((Col, Row, Depth));
                            CutBcy += Depth * Ledger.CutBcyPerDepthM;
                        }
                    }
                    else if ((Start < -1e-4) && (End > (Start + 1e-4)))
                    {
                        // Design fill stops at the target. A lift past 0 is surplus.
                        Double Depth = (Math.Min(End, 0.0) - Start) / 1000.0;
                        if (Depth > 1e-9)
                        {
                            Fills.Add((Col, Row, Depth));
                            FillBcy += Depth * Ledger.FillBcyPerDepthM;
                        }
                    }
                }
            }
            // Surplus spread on a high cell is omitted on both sides, so the pass stays balanced.
            Double Budget = Math.Min(CutBcy, FillBcy);
            Int32 CutIndex = 0;
            Int32 FillIndex = 0;
            Double CutUsed = 0.0;
            Double FillUsed = 0.0;
            while ((CutUsed < (Budget - 1e-8)) && (FillUsed < (Budget - 1e-8)))
            {
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> PassCuts =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> PassFills =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                Double PassCut = 0.0;
                Double Room = Math.Min(BowlBcy, Budget - CutUsed);
                while ((CutIndex < Cuts.Count) && (PassCut < (Room - 1e-10)))
                {
                    (Int32 Col, Int32 Row, Double DepthM) Item = Cuts[CutIndex];
                    Double Slice = Math.Min(Item.DepthM, MaxDepthM);
                    Double SliceBcy = Slice * Ledger.CutBcyPerDepthM;
                    if (SliceBcy > (Room - PassCut))
                    {
                        Slice = (Room - PassCut) / Ledger.CutBcyPerDepthM;
                        SliceBcy = Room - PassCut;
                    }
                    if (Slice <= 1e-12)
                    {
                        break;
                    }
                    PassCuts.Add(((Item.Col, Item.Row), Slice));
                    PassCut += SliceBcy;
                    Double Left = Item.DepthM - Slice;
                    if (Left <= 1e-12)
                    {
                        CutIndex += 1;
                    }
                    else
                    {
                        Cuts[CutIndex] = (Item.Col, Item.Row, Left);
                    }
                }
                Double PassFill = 0.0;
                while ((FillIndex < Fills.Count) && (PassFill < (PassCut - 1e-10)))
                {
                    (Int32 Col, Int32 Row, Double DepthM) Item = Fills[FillIndex];
                    Double Slice = Math.Min(Item.DepthM, MaxDepthM);
                    Double SliceBcy = Slice * Ledger.FillBcyPerDepthM;
                    if (SliceBcy > (PassCut - PassFill))
                    {
                        Slice = (PassCut - PassFill) / Ledger.FillBcyPerDepthM;
                        SliceBcy = PassCut - PassFill;
                    }
                    if (Slice <= 1e-12)
                    {
                        break;
                    }
                    PassFills.Add(((Item.Col, Item.Row), Slice));
                    PassFill += SliceBcy;
                    Double Left = Item.DepthM - Slice;
                    if (Left <= 1e-12)
                    {
                        FillIndex += 1;
                    }
                    else
                    {
                        Fills[FillIndex] = (Item.Col, Item.Row, Left);
                    }
                }
                if ((PassCuts.Count == 0) || (PassFills.Count == 0))
                {
                    break;
                }
                Shave(PassCuts, PassFills, Ledger);
                DropDust(PassCuts);
                DropDust(PassFills);
                if ((PassCuts.Count == 0) || (PassFills.Count == 0))
                {
                    break;
                }
                Double Balanced = 0.0;
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in PassCuts)
                {
                    Balanced += Lift.DepthM * Ledger.CutBcyPerDepthM;
                }
                (Double X, Double Y) CutStart = Grid.CellCenterXy(PassCuts[0].Cell.Bx, PassCuts[0].Cell.By);
                (Double X, Double Y) CutEnd = Grid.CellCenterXy(
                    PassCuts[PassCuts.Count - 1].Cell.Bx, PassCuts[PassCuts.Count - 1].Cell.By);
                (Double X, Double Y) FillStart = Grid.CellCenterXy(PassFills[0].Cell.Bx, PassFills[0].Cell.By);
                (Double X, Double Y) FillEnd = Grid.CellCenterXy(
                    PassFills[PassFills.Count - 1].Cell.Bx, PassFills[PassFills.Count - 1].Cell.By);
                Double HaulM = Math.Sqrt(
                    Math.Pow(FillStart.X - CutEnd.X, 2.0) + Math.Pow(FillStart.Y - CutEnd.Y, 2.0));
                Pass Trip = new Pass(PassCuts, PassFills, Balanced, HaulM, CutStart, CutEnd, FillStart, FillEnd);
                Trip.StackedLifts = true;
                Trip.RibbonOrder = true;
                Passes.Add(Trip);
                CutUsed += Balanced;
                FillUsed += Balanced;
            }
        }

        /// <summary>
        /// Drops lifts that shaving reduced to nothing.
        /// </summary>
        /// <param name="Lifts">Cut or fill lifts.</param>
        private static void DropDust
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Lifts
            )
        {
            for (Int32 Index = Lifts.Count - 1; Index >= 0; Index -= 1)
            {
                if (Lifts[Index].DepthM <= 0.0)
                {
                    Lifts.RemoveAt(Index);
                }
            }
        }

        /// <summary>
        /// Trims the last lift so the ledger sees the same yards on both sides.
        /// </summary>
        /// <param name="Cuts">Cut lifts.</param>
        /// <param name="Fills">Fill lifts.</param>
        /// <param name="Ledger">Yard factors.</param>
        private static void Shave
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills,
            HaulWorkLedger Ledger
            )
        {
            Double CutBcy = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Cuts)
            {
                CutBcy += Lift.DepthM * Ledger.CutBcyPerDepthM;
            }
            Double FillBcy = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Fills)
            {
                FillBcy += Lift.DepthM * Ledger.FillBcyPerDepthM;
            }
            Double Gap = CutBcy - FillBcy;
            if ((Gap > 1e-12) && (Cuts.Count > 0))
            {
                Int32 Last = Cuts.Count - 1;
                Double Trim = Math.Min(Cuts[Last].DepthM, Gap / Ledger.CutBcyPerDepthM);
                Cuts[Last] = (Cuts[Last].Cell, Cuts[Last].DepthM - Trim);
            }
            else if ((Gap < -1e-12) && (Fills.Count > 0))
            {
                Int32 Last = Fills.Count - 1;
                Double Trim = Math.Min(Fills[Last].DepthM, (-Gap) / Ledger.FillBcyPerDepthM);
                Fills[Last] = (Fills[Last].Cell, Fills[Last].DepthM - Trim);
            }
        }

        /// <summary>
        /// Straight distance between two cell centres, in metres.
        /// </summary>
        /// <param name="FromRow">Start row.</param>
        /// <param name="FromCol">Start column.</param>
        /// <param name="ToRow">End row.</param>
        /// <param name="ToCol">End column.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <returns>Metres.</returns>
        private static Double StepMetres
            (
            Int32 FromRow,
            Int32 FromCol,
            Int32 ToRow,
            Int32 ToCol,
            Double CellSizeM
            )
        {
            Double Row = ToRow - FromRow;
            Double Col = ToCol - FromCol;
            return Math.Sqrt((Row * Row) + (Col * Col)) * CellSizeM;
        }

        /// <summary>
        /// Hours at the pass speed, plus the turn charge.
        /// </summary>
        /// <param name="DistanceM">Metres driven, including the connections.</param>
        /// <param name="Turns">Serpentine turns.</param>
        /// <returns>Hours.</returns>
        private static Double Hours
            (
            Double DistanceM,
            Int32 Turns
            )
        {
            return ((DistanceM / V_PASS_MS) + (Turns * T_TURN_S)) / 3600.0;
        }

        /// <summary>
        /// Inside fraction of one cell. A missing grid means the cell is fully inside.
        /// </summary>
        /// <param name="InsideFraction">Fractions, or null.</param>
        /// <param name="Row">Row.</param>
        /// <param name="Col">Column.</param>
        /// <returns>The fraction from 0 to 1.</returns>
        private static Double FractionOf
            (
            Double[,]? InsideFraction,
            Int32 Row,
            Int32 Col
            )
        {
            if (InsideFraction == null)
            {
                return 1.0;
            }
            Double Fraction = InsideFraction[Row, Col];
            if (Fraction < 0.0)
            {
                return 0.0;
            }
            if (Fraction > 1.0)
            {
                return 1.0;
            }
            return Fraction;
        }

        /// <summary>
        /// A copy of the residual so a later coverage cannot change the snapshot.
        /// </summary>
        /// <param name="Source">Residual in millimetres.</param>
        /// <returns>The copy.</returns>
        private static Double[,] Clone
            (
            Double[,] Source
            )
        {
            Int32 Rows = Source.GetLength(0);
            Int32 Cols = Source.GetLength(1);
            Double[,] Copy = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    Copy[Row, Col] = Source[Row, Col];
                }
            }
            return Copy;
        }

        /// <summary>
        /// Prints each coverage, then any groups that remain. Correcting is the last stage.
        /// </summary>
        /// <param name="Result">The run.</param>
        private static void Report
            (
            CorrectingResult Result
            )
        {
            List<String> Lines = new List<String>();
            foreach (CorrectingCoverage Coverage in Result.Coverages)
            {
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "correcting coverage {0}",
                    Coverage.Number));
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "within +/-30 mm: {0:F1}%",
                    Coverage.Within30Percent));
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "within +/-50 mm: {0:F1}%",
                    Coverage.Within50Percent));
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "beyond +/-100 mm: {0} cells",
                    Coverage.Beyond100Cells));
                Int32 Listed = Math.Min(10, Coverage.Groups.Count);
                if (Listed == 0)
                {
                    Say(Lines, "groups beyond +/-30 mm: none");
                }
                for (Int32 Index = 0; Index < Listed; Index += 1)
                {
                    CorrectingGroup Group = Coverage.Groups[Index];
                    Say(Lines, String.Format(
                        CultureInfo.InvariantCulture,
                        "  group {0}: {1} cells, {2:R} bcy, peak {3:F1} mm, length {4:F1} m",
                        Index + 1,
                        Group.Cells,
                        Group.Bcy,
                        Group.PeakMm,
                        Group.LengthM));
                }
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "surplus spread: {0:R} bcy",
                    Coverage.SurplusBcy));
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "distance: {0:R} m",
                    Coverage.DistanceM));
                Say(Lines, String.Format(CultureInfo.InvariantCulture, "turns: {0}", Coverage.Turns));
                Say(Lines, String.Format(CultureInfo.InvariantCulture, "hours: {0:R}", Coverage.Hours));
            }
            if (Result.ReadyForPlaning)
            {
                Say(Lines, "correcting finished");
            }
            else
            {
                Say(Lines, "groups remain after correcting");
                foreach (CorrectingGroup Group in Result.FailedGroups)
                {
                    Say(Lines, String.Format(
                        CultureInfo.InvariantCulture,
                        "  failing group: {0} cells, {1:R} bcy, peak {2:F1} mm, length {3:F1} m",
                        Group.Cells,
                        Group.Bcy,
                        Group.PeakMm,
                        Group.LengthM));
                }
            }
            lock (ReportGate)
            {
                String ReportPath = Path.Combine(Path.GetTempPath(), "correcting-report.txt");
                try
                {
                    File.WriteAllLines(ReportPath, Lines);
                }
                catch (IOException)
                {
                    File.WriteAllLines(
                        Path.Combine(Path.GetTempPath(), "correcting-report-" + Guid.NewGuid().ToString("N") + ".txt"),
                        Lines);
                }
            }
        }

        /// <summary>
        /// Prints one line and keeps it for the temp copy.
        /// </summary>
        /// <param name="Lines">Lines already printed.</param>
        /// <param name="Text">The next line.</param>
        private static void Say
            (
            List<String> Lines,
            String Text
            )
        {
            Lines.Add(Text);
            Console.WriteLine(Text);
        }

        /// <summary>
        /// Volumes and travel for one coverage.
        /// </summary>
        private sealed class Tally
        {
            /// <summary>Gets or sets bank cubic metres cut.</summary>
            public Double CutM3 { get; set; }

            /// <summary>Gets or sets bank cubic metres placed on low cells.</summary>
            public Double FillM3 { get; set; }

            /// <summary>Gets or sets bank cubic metres spread as surplus.</summary>
            public Double SurplusM3 { get; set; }

            /// <summary>Gets or sets metres driven.</summary>
            public Double DistanceM { get; set; }

            /// <summary>Gets or sets serpentine turns.</summary>
            public Int32 Turns { get; set; }
        }
    }

    /// <summary>
    /// One group of cells outside +/-30 mm after a coverage.
    /// </summary>
    public sealed class CorrectingGroup
    {
        /// <summary>
        /// Creates a group.
        /// </summary>
        /// <param name="Cells">Cell count.</param>
        /// <param name="Bcy">Bank yards in the group.</param>
        /// <param name="PeakMm">Furthest cell from the target, in millimetres.</param>
        /// <param name="LengthM">Longer side of the group, in metres.</param>
        public CorrectingGroup
            (
            Int32 Cells,
            Double Bcy,
            Double PeakMm,
            Double LengthM
            )
        {
            this.Cells = Cells;
            this.Bcy = Bcy;
            this.PeakMm = PeakMm;
            this.LengthM = LengthM;
        }

        /// <summary>Gets the cell count.</summary>
        public Int32 Cells { get; }

        /// <summary>Gets the bank yards.</summary>
        public Double Bcy { get; }

        /// <summary>Gets the peak residual in millimetres.</summary>
        public Double PeakMm { get; }

        /// <summary>Gets the longer side in metres.</summary>
        public Double LengthM { get; }
    }

    /// <summary>
    /// The report for one coverage, including a snapshot of the residual.
    /// </summary>
    public sealed class CorrectingCoverage
    {
        /// <summary>
        /// Creates a coverage report.
        /// </summary>
        /// <param name="Number">Coverage number, starting at 1.</param>
        /// <param name="Within30Percent">Percent of cells inside +/-30 mm.</param>
        /// <param name="Within50Percent">Percent of cells inside +/-50 mm.</param>
        /// <param name="Beyond100Cells">Cells outside +/-100 mm.</param>
        /// <param name="Groups">Groups outside +/-30 mm, largest first.</param>
        /// <param name="SurplusBcy">Load spread as surplus, in bank cubic yards.</param>
        /// <param name="DistanceM">Metres driven.</param>
        /// <param name="Turns">Serpentine turns.</param>
        /// <param name="Hours">Estimated hours.</param>
        /// <param name="ResidualMm">Residual after this coverage.</param>
        public CorrectingCoverage
            (
            Int32 Number,
            Double Within30Percent,
            Double Within50Percent,
            Int32 Beyond100Cells,
            IReadOnlyList<CorrectingGroup> Groups,
            Double SurplusBcy,
            Double DistanceM,
            Int32 Turns,
            Double Hours,
            Double[,] ResidualMm
            )
        {
            this.Number = Number;
            this.Within30Percent = Within30Percent;
            this.Within50Percent = Within50Percent;
            this.Beyond100Cells = Beyond100Cells;
            this.Groups = Groups;
            this.SurplusBcy = SurplusBcy;
            this.DistanceM = DistanceM;
            this.Turns = Turns;
            this.Hours = Hours;
            this.ResidualMm = ResidualMm;
        }

        /// <summary>Gets the coverage number.</summary>
        public Int32 Number { get; }

        /// <summary>Gets the percent of cells inside +/-30 mm.</summary>
        public Double Within30Percent { get; }

        /// <summary>Gets the percent of cells inside +/-50 mm.</summary>
        public Double Within50Percent { get; }

        /// <summary>Gets the cells outside +/-100 mm.</summary>
        public Int32 Beyond100Cells { get; }

        /// <summary>Gets the groups outside +/-30 mm, largest first.</summary>
        public IReadOnlyList<CorrectingGroup> Groups { get; }

        /// <summary>Gets the surplus spread, in bank cubic yards.</summary>
        public Double SurplusBcy { get; }

        /// <summary>Gets the metres driven.</summary>
        public Double DistanceM { get; }

        /// <summary>Gets the serpentine turns.</summary>
        public Int32 Turns { get; }

        /// <summary>Gets the estimated hours.</summary>
        public Double Hours { get; }

        /// <summary>Gets the residual after this coverage.</summary>
        public Double[,] ResidualMm { get; }
    }

    /// <summary>
    /// The coverages and the bank volumes moved. The search stops after this result.
    /// </summary>
    public sealed class CorrectingResult
    {
        /// <summary>
        /// Creates a result.
        /// </summary>
        /// <param name="Coverages">One entry per coverage that ran.</param>
        /// <param name="ReadyForPlaning">True when no remaining group is long or deep.</param>
        /// <param name="FailedGroups">Groups that are still long or deep. The search does not continue.</param>
        /// <param name="CutRemovedM3">Bank cubic metres cut.</param>
        /// <param name="FillPlacedM3">Bank cubic metres placed on low cells.</param>
        /// <param name="SurplusSpreadM3">Bank cubic metres spread as surplus.</param>
        /// <param name="BowlLeftM3">Bank cubic metres still in the bowl.</param>
        /// <param name="CoveragesRun">How many coverages ran.</param>
        public CorrectingResult
            (
            IReadOnlyList<CorrectingCoverage> Coverages,
            Boolean ReadyForPlaning,
            IReadOnlyList<CorrectingGroup> FailedGroups,
            Double CutRemovedM3,
            Double FillPlacedM3,
            Double SurplusSpreadM3,
            Double BowlLeftM3,
            Int32 CoveragesRun
            )
        {
            this.Coverages = Coverages;
            this.ReadyForPlaning = ReadyForPlaning;
            this.FailedGroups = FailedGroups;
            this.CutRemovedM3 = CutRemovedM3;
            this.FillPlacedM3 = FillPlacedM3;
            this.SurplusSpreadM3 = SurplusSpreadM3;
            this.BowlLeftM3 = BowlLeftM3;
            this.CoveragesRun = CoveragesRun;
        }

        /// <summary>Gets the coverage reports.</summary>
        public IReadOnlyList<CorrectingCoverage> Coverages { get; }

        /// <summary>Gets whether no remaining group is long or deep.</summary>
        public Boolean ReadyForPlaning { get; }

        /// <summary>Gets the groups that are still long or deep after the last coverage.</summary>
        public IReadOnlyList<CorrectingGroup> FailedGroups { get; }

        /// <summary>Gets the bank cubic metres cut.</summary>
        public Double CutRemovedM3 { get; }

        /// <summary>Gets the bank cubic metres placed on low cells.</summary>
        public Double FillPlacedM3 { get; }

        /// <summary>Gets the bank cubic metres spread as surplus.</summary>
        public Double SurplusSpreadM3 { get; }

        /// <summary>Gets the bank cubic metres still in the bowl.</summary>
        public Double BowlLeftM3 { get; }

        /// <summary>Gets how many coverages ran.</summary>
        public Int32 CoveragesRun { get; }
    }
}
