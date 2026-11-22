namespace HaulBuilder
{
    /// <summary>
    /// Area placement for one cluster or thin-cut trip.
    /// The piece is split into bands parallel to the haul heading, one blade wide.
    /// The fullest band is taken first. Inside a cut band the highest cells go first, and inside
    /// a fill band the lowest cells go first. A cell moves only when its band is chosen.
    /// The time is the sum of the chosen band lengths over the ground that moved, plus a loop
    /// between bands. The haul is from the cut piece's exit face to the fill piece's entry face.
    /// </summary>
    static public class StripLanes
    {
        /// <summary>Deepest cut in one trip, in millimetres. The machine limit on Shop 2 is the same number.</summary>
        public const Double CUT_DEPTH_CAP_MM = 61.0;

        /// <summary>Deepest fill placed on one cell in one trip, in metres, before the machine limit.</summary>
        public const Double SPREAD_LIFT_M = 0.10;

        /// <summary>Old lane-sweep turn radius, in metres. Area trips do not turn while they load or spread.</summary>
        public const Double MIN_TURN_RADIUS_M = 8.0;

        /// <summary>Most the ledger may finish a grade cell above target, in millimetres. Area placement does not use it.</summary>
        public const Double SURPLUS_MAX_MM = 10.0;

        /// <summary>Cut speed. Four miles an hour.</summary>
        public const Double V_CUT_MPS = Constants.DEFAULT_CUT_SPEED_MPH * Constants.MPH_TO_MPS;

        /// <summary>Fill speed. Five miles an hour.</summary>
        public const Double V_FILL_MPS = Constants.DEFAULT_FILL_SPEED_MPH * Constants.MPH_TO_MPS;

        /// <summary>Seconds charged for each extra pass. The first pass has no loop.</summary>
        public const Double T_LOOP_S = 35.0;

        /// <summary>Depths and yards below this are treated as nothing.</summary>
        private const Double EPS = 1e-9;

        /// <summary>Blade width. One shop cell is one blade.</summary>
        private static readonly Double BLADE_M = Constants.DEFAULT_BLADE_WIDTH_MM / Constants.MILLIMETERS_PER_METER;

        /// <summary>
        /// One illustrated pass. It is a straight line along the heading, not a path the machine drives.
        /// </summary>
        public readonly struct PassLine
        {
            /// <summary>
            /// Creates a line.
            /// </summary>
            /// <param name="X1">Easting of the upstream end.</param>
            /// <param name="Y1">Northing of the upstream end.</param>
            /// <param name="X2">Easting of the downstream end.</param>
            /// <param name="Y2">Northing of the downstream end.</param>
            public PassLine
                (
                Double X1,
                Double Y1,
                Double X2,
                Double Y2
                )
            {
                this.X1 = X1;
                this.Y1 = Y1;
                this.X2 = X2;
                this.Y2 = Y2;
            }

            /// <summary>Gets the upstream easting.</summary>
            public Double X1 { get; }

            /// <summary>Gets the upstream northing.</summary>
            public Double Y1 { get; }

            /// <summary>Gets the downstream easting.</summary>
            public Double X2 { get; }

            /// <summary>Gets the downstream northing.</summary>
            public Double Y2 { get; }

            /// <summary>Gets whether the line has a visible length.</summary>
            public Boolean HasLength
            {
                get
                {
                    Double Dx = this.X2 - this.X1;
                    Double Dy = this.Y2 - this.Y1;
                    return ((Dx * Dx) + (Dy * Dy)) > 1e-6;
                }
            }
        }

        /// <summary>
        /// The cells a trip moves on one side, and the pass time for that side.
        /// </summary>
        public sealed class Result
        {
            /// <summary>Piece the length is measured on. The cells actually moved may be a subset.</summary>
            private readonly List<(Int32 Row, Int32 Col)> _Piece;

            /// <summary>Speed for this side, in metres per second.</summary>
            private readonly Double _Speed;

            /// <summary>True on a fill. The bank factor is the compaction. A cut uses 1.</summary>
            private readonly Boolean _Fill;

            /// <summary>
            /// Creates an empty side. The cells and the time are filled in after the take.
            /// </summary>
            /// <param name="DirX">Heading east, unit length.</param>
            /// <param name="DirY">Heading north, unit length.</param>
            /// <param name="Speed">Metres per second on this side.</param>
            /// <param name="Fill">True when the yards include compaction.</param>
            /// <param name="Piece">Every cell of the piece, for the pass length.</param>
            internal Result
                (
                Double DirX,
                Double DirY,
                Double Speed,
                Boolean Fill,
                List<(Int32 Row, Int32 Col)> Piece
                )
            {
                this.DirX = DirX;
                this.DirY = DirY;
                this._Speed = Speed;
                this._Fill = Fill;
                this._Piece = Piece;
                this.Cells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                this.Lines = new List<PassLine>();
            }

            /// <summary>Gets the cells where dirt moves. Fullest band first, then highest cut or lowest fill inside it.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells { get; }

            /// <summary>Gets one centreline per chosen band, over the ground that moved.</summary>
            public List<PassLine> Lines { get; }

            /// <summary>Gets the seconds for this side, including a loop between passes.</summary>
            public Double Seconds { get; private set; }

            /// <summary>Gets the sum of the chosen band lengths, in metres.</summary>
            public Double DistanceM { get; private set; }

            /// <summary>Gets the bank yards moved.</summary>
            public Double Bcy { get; private set; }

            /// <summary>Gets how many bands this side moved.</summary>
            public Int32 PassCount { get; private set; }

            /// <summary>Gets the loops between passes. The first pass has none.</summary>
            public Int32 Loops
            {
                get
                {
                    return this.PassCount > 0 ? this.PassCount - 1 : 0;
                }
            }

            /// <summary>Gets the heading east.</summary>
            public Double DirX { get; }

            /// <summary>Gets the heading north.</summary>
            public Double DirY { get; }

            /// <summary>Gets the easting of the upstream face.</summary>
            public Double StartX { get; private set; }

            /// <summary>Gets the northing of the upstream face.</summary>
            public Double StartY { get; private set; }

            /// <summary>Gets the easting of the downstream face. A cut hauls away from here.</summary>
            public Double EndX { get; private set; }

            /// <summary>Gets the northing of the downstream face.</summary>
            public Double EndY { get; private set; }

            /// <summary>
            /// Drops cut from the tail until the yards match what the fill could take.
            /// The tail is the last cells chosen, which are the lowest of the high ones.
            /// </summary>
            /// <param name="TargetBcy">Bank yards the fill actually took.</param>
            /// <param name="Ground">Cell size for the yard factor, and the piece length for the new time.</param>
            internal void TrimTo
                (
                Double TargetBcy,
                Ground Ground
                )
            {
                Double Area = Ground.Grid.GridSizeM * Ground.Grid.GridSizeM;
                Double Factor = Area * ResidualClusters.M3_TO_CY;
                Double Excess = this.Bcy - TargetBcy;
                if ((Excess <= 1e-8) || (Factor <= EPS))
                {
                    return;
                }
                for (Int32 Index = this.Cells.Count - 1; (Index >= 0) && (Excess > 1e-12); Index -= 1)
                {
                    ((Int32 Bx, Int32 By) Cell, Double DepthM) Lift = this.Cells[Index];
                    Double CellBcy = Lift.DepthM * Factor;
                    if (CellBcy <= Excess + 1e-12)
                    {
                        Excess -= CellBcy;
                        this.Cells.RemoveAt(Index);
                    }
                    else
                    {
                        Double Depth = (CellBcy - Excess) / Factor;
                        this.Cells[Index] = (Lift.Cell, Depth);
                        Excess = 0.0;
                    }
                }
                this.Retiming(Ground);
            }

            /// <summary>
            /// Puts the exit on the downstream face and the entry on the upstream face.
            /// The haul leaves the cut at the exit and meets the fill at the entry.
            /// </summary>
            /// <param name="Ground">Cell size and origin.</param>
            internal void PlaceEnds
                (
                Ground Ground
                )
            {
                Double CellAlong = Ground.Grid.GridSizeM * (Math.Abs(this.DirX) + Math.Abs(this.DirY));
                Double Half = 0.5 * CellAlong;
                Boolean Any = false;
                Double MinAlong = 0.0;
                Double MaxAlong = 0.0;
                (Double X, Double Y) MinAt = (0.0, 0.0);
                (Double X, Double Y) MaxAt = (0.0, 0.0);
                foreach ((Int32 Row, Int32 Col) Cell in this._Piece)
                {
                    if (!OnField(Ground, Cell.Col, Cell.Row))
                    {
                        continue;
                    }
                    (Double X, Double Y) At = Center(Ground.Grid, Cell.Col, Cell.Row);
                    Double Along = (At.X * this.DirX) + (At.Y * this.DirY);
                    if (!Any || (Along < MinAlong))
                    {
                        MinAlong = Along;
                        MinAt = At;
                    }
                    if (!Any || (Along > MaxAlong))
                    {
                        MaxAlong = Along;
                        MaxAt = At;
                    }
                    Any = true;
                }
                if (!Any)
                {
                    return;
                }
                this.StartX = MinAt.X - (this.DirX * Half);
                this.StartY = MinAt.Y - (this.DirY * Half);
                this.EndX = MaxAt.X + (this.DirX * Half);
                this.EndY = MaxAt.Y + (this.DirY * Half);
            }

            /// <summary>
            /// Rebuilds the yards, the band lines, and the time from the cells that still have dirt.
            /// A trimmed cut can drop a band, so the pass count can fall.
            /// </summary>
            /// <param name="Ground">Cell size and bank factor.</param>
            internal void Retiming
                (
                Ground Ground
                )
            {
                Double Area = Ground.Grid.GridSizeM * Ground.Grid.GridSizeM;
                Double Bank = this._Fill ? Ground.FillToBank : 1.0;
                Double YardsPerMetre = Area * ResidualClusters.M3_TO_CY * Bank;
                Double Bcy = 0.0;
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in this.Cells)
                {
                    if (Lift.DepthM <= EPS)
                    {
                        continue;
                    }
                    Bcy += Lift.DepthM * YardsPerMetre;
                }
                this.Bcy = Bcy;
                // The length is only the ground this trip actually moved, one band at a time.
                List<(Double LengthM, PassLine Line)> Runs = MeasureBands(
                    Ground.Grid, this.Cells, this.DirX, this.DirY);
                this.Lines.Clear();
                Double Run = 0.0;
                foreach ((Double LengthM, PassLine Line) Span in Runs)
                {
                    Run += Span.LengthM;
                    this.Lines.Add(Span.Line);
                }
                this.DistanceM = Run;
                this.PassCount = Runs.Count;
                if ((this.PassCount == 0) || (Run <= EPS))
                {
                    this.Seconds = 0.0;
                    return;
                }
                Double Speed = this._Speed > EPS ? this._Speed : V_CUT_MPS;
                this.Seconds = (Run / Speed) + ((this.PassCount - 1) * T_LOOP_S);
            }
        }

        /// <summary>
        /// The residual a trip reads and writes.
        /// </summary>
        public sealed class Ground
        {
            /// <summary>
            /// Creates the ground.
            /// </summary>
            /// <param name="Grid">Field grid.</param>
            /// <param name="ResidualMm">Residual in millimetres, indexed [row, col]. Positive is cut.</param>
            /// <param name="Mask">True on cells the machine may work.</param>
            /// <param name="FillToBank">Bank yards of fill per loose yard.</param>
            /// <param name="ToleranceMm">Grade line. Cut stops at this. Fill stops at its negative.</param>
            /// <param name="ArrowX">Smoothed flow east. The area trip does not follow it.</param>
            /// <param name="ArrowY">Smoothed flow north. The area trip does not follow it.</param>
            public Ground
                (
                Grid Grid,
                Double[,] ResidualMm,
                Boolean[,] Mask,
                Double FillToBank,
                Double ToleranceMm,
                Double[,]? ArrowX = null,
                Double[,]? ArrowY = null
                )
            {
                this.Grid = Grid;
                this.ResidualMm = ResidualMm;
                this.Mask = Mask;
                this.FillToBank = FillToBank;
                this.ToleranceMm = ToleranceMm;
                this.ArrowX = ArrowX;
                this.ArrowY = ArrowY;
            }

            /// <summary>Gets the grid.</summary>
            public Grid Grid { get; }

            /// <summary>Gets the residual in millimetres.</summary>
            public Double[,] ResidualMm { get; }

            /// <summary>Gets the drive mask.</summary>
            public Boolean[,] Mask { get; }

            /// <summary>Gets the fill bank factor.</summary>
            public Double FillToBank { get; }

            /// <summary>Gets the grade line in millimetres.</summary>
            public Double ToleranceMm { get; }

            /// <summary>Gets the flow east component. The area trip does not follow it.</summary>
            public Double[,]? ArrowX { get; }

            /// <summary>Gets the flow north component. The area trip does not follow it.</summary>
            public Double[,]? ArrowY { get; }
        }

        /// <summary>
        /// Seconds to cut one bowl as a single straight run. Pairing has not chosen the fill yet,
        /// so the cost has no loop and does not depend on a heading.
        /// </summary>
        /// <param name="Ground">The residual this plan was built from.</param>
        /// <param name="Piece">One cut piece.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <returns>Seconds. Zero when the piece has nothing above the grade line.</returns>
        public static Double FullBowlCutSeconds
            (
            Ground Ground,
            ResidualPairPiece Piece,
            Double BowlBcy
            )
        {
            if ((Ground == null) || (Piece == null) || (BowlBcy <= 0.0) || (Piece.Cells.Count == 0))
            {
                return 0.0;
            }
            Double Sum = 0.0;
            Int32 Count = 0;
            foreach ((Int32 Row, Int32 Col) Cell in Piece.Cells)
            {
                if (!OnField(Ground, Cell.Col, Cell.Row))
                {
                    continue;
                }
                Double Depth = AvailableCutM(Ground, Cell.Col, Cell.Row);
                if (Depth <= EPS)
                {
                    continue;
                }
                Sum += Depth;
                Count += 1;
            }
            if ((Count == 0) || (Sum <= EPS))
            {
                return 0.0;
            }
            // One blade, one depth, no loop. The length is the bowl spread along that blade.
            Double Mean = Sum / Count;
            Double RunM = (BowlBcy / ResidualClusters.M3_TO_CY) / (BLADE_M * Mean);
            return RunM / V_CUT_MPS;
        }

        /// <summary>
        /// True when the piece still has a cell above the grade line.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Cells">The piece.</param>
        /// <returns>True when a trip can still cut here.</returns>
        public static Boolean HasCut
            (
            Ground Ground,
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells
            )
        {
            if ((Ground == null) || (Cells == null))
            {
                return false;
            }
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                if (OnField(Ground, Cell.Col, Cell.Row) && (AvailableCutM(Ground, Cell.Col, Cell.Row) > EPS))
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Unit heading from the cut piece toward the fill. A zero length falls back to east.
        /// </summary>
        /// <param name="FromX">Cut easting.</param>
        /// <param name="FromY">Cut northing.</param>
        /// <param name="ToX">Fill easting.</param>
        /// <param name="ToY">Fill northing.</param>
        /// <returns>The unit heading.</returns>
        public static (Double X, Double Y) Heading
            (
            Double FromX,
            Double FromY,
            Double ToX,
            Double ToY
            )
        {
            return Unit(ToX - FromX, ToY - FromY);
        }

        /// <summary>
        /// Removes up to one lift from the cells of a cut piece that are above target.
        /// The highest cells go first. Nothing is cut below the grade line.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Cells">The cut piece.</param>
        /// <param name="DirX">Haul heading east.</param>
        /// <param name="DirY">Haul heading north.</param>
        /// <param name="PayloadBcy">Bank yards this trip may load.</param>
        /// <returns>The cut, or null when the piece has nothing above target.</returns>
        public static Result? Cut
            (
            Ground Ground,
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells,
            Double DirX,
            Double DirY,
            Double PayloadBcy
            )
        {
            return Take(Ground, Cells, DirX, DirY, PayloadBcy, false);
        }

        /// <summary>
        /// Raises the cells of a fill piece that are below target, lowest first.
        /// One cell takes at most one spread, and nothing is placed above the grade line.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Cells">The fill piece.</param>
        /// <param name="DirX">Haul heading east.</param>
        /// <param name="DirY">Haul heading north.</param>
        /// <param name="PayloadBcy">Bank yards to place. This is the load the cut actually took.</param>
        /// <returns>The fill, or null when the piece cannot take any of the load.</returns>
        public static Result? Fill
            (
            Ground Ground,
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells,
            Double DirX,
            Double DirY,
            Double PayloadBcy
            )
        {
            return Take(Ground, Cells, DirX, DirY, PayloadBcy, true);
        }

        /// <summary>
        /// Trims the cut when the fill cannot hold it, then rebuilds the cut time.
        /// </summary>
        /// <param name="Cut">The load.</param>
        /// <param name="Spread">What the fill could take. Null leaves the cut alone.</param>
        /// <param name="Ground">Cell size for the trim.</param>
        public static void Match
            (
            Result Cut,
            Result? Spread,
            Ground Ground
            )
        {
            if ((Cut == null) || (Spread == null) || (Ground == null))
            {
                return;
            }
            if (Cut.Bcy <= Spread.Bcy + 1e-6)
            {
                return;
            }
            Cut.TrimTo(Spread.Bcy, Ground);
        }

        /// <summary>
        /// Writes the depths back onto the working residual.
        /// Cut lowers a cell. Fill raises it.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Cells">Cells just moved.</param>
        /// <param name="Sign">1 lowers the residual. -1 raises it.</param>
        public static void Apply
            (
            Ground Ground,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Int32 Sign
            )
        {
            if ((Ground == null) || (Cells == null))
            {
                return;
            }
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Cells)
            {
                Ground.ResidualMm[Lift.Cell.By, Lift.Cell.Bx] -= Sign * Lift.DepthM * 1000.0;
            }
        }

        /// <summary>
        /// Length of a piece along a heading, from the upstream face to the downstream face.
        /// </summary>
        /// <param name="Ground">Cell size and mask.</param>
        /// <param name="Cells">The piece.</param>
        /// <param name="DirX">Heading east, unit length.</param>
        /// <param name="DirY">Heading north, unit length.</param>
        /// <returns>Metres. At least one cell when the piece is on the field.</returns>
        public static Double LengthAlong
            (
            Ground Ground,
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells,
            Double DirX,
            Double DirY
            )
        {
            (Double X, Double Y) Dir = Unit(DirX, DirY);
            Double CellAlong = Ground.Grid.GridSizeM * (Math.Abs(Dir.X) + Math.Abs(Dir.Y));
            if ((Cells == null) || (Cells.Count == 0))
            {
                return CellAlong;
            }
            Boolean Any = false;
            Double MinAlong = 0.0;
            Double MaxAlong = 0.0;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                if (!OnField(Ground, Cell.Col, Cell.Row))
                {
                    continue;
                }
                (Double X, Double Y) At = Center(Ground.Grid, Cell.Col, Cell.Row);
                Double Along = (At.X * Dir.X) + (At.Y * Dir.Y);
                if (!Any || (Along < MinAlong))
                {
                    MinAlong = Along;
                }
                if (!Any || (Along > MaxAlong))
                {
                    MaxAlong = Along;
                }
                Any = true;
            }
            if (!Any)
            {
                return CellAlong;
            }
            Double Length = (MaxAlong - MinAlong) + CellAlong;
            if (Length < CellAlong)
            {
                Length = CellAlong;
            }
            return Length;
        }

        /// <summary>
        /// One centreline per band that still has dirt in <paramref name="Cells"/>.
        /// The line runs along the heading and spans the cells that moved in that band.
        /// </summary>
        /// <param name="Grid">Cell size and origin.</param>
        /// <param name="Cells">Cells that lost or gained volume. Zero depth is ignored.</param>
        /// <param name="DirX">Heading east.</param>
        /// <param name="DirY">Heading north.</param>
        /// <returns>The centrelines. Empty when nothing moved.</returns>
        public static List<PassLine> BandLines
            (
            Grid Grid,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? Cells,
            Double DirX,
            Double DirY
            )
        {
            List<PassLine> Lines = new List<PassLine>();
            foreach ((Double LengthM, PassLine Line) Span in MeasureBands(Grid, Cells, DirX, DirY))
            {
                Lines.Add(Span.Line);
            }
            return Lines;
        }

        /// <summary>
        /// One arrow through the middle of the piece, pointing along the haul heading.
        /// </summary>
        /// <param name="Grid">Cell size and origin.</param>
        /// <param name="Cells">The piece outline, as row and column.</param>
        /// <param name="DirX">Heading east.</param>
        /// <param name="DirY">Heading north.</param>
        /// <returns>The arrow. It has no length when the piece is empty.</returns>
        public static PassLine HeadingArrow
            (
            Grid Grid,
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells,
            Double DirX,
            Double DirY
            )
        {
            if ((Grid == null) || (Cells == null) || (Cells.Count == 0))
            {
                return new PassLine(0.0, 0.0, 0.0, 0.0);
            }
            (Double X, Double Y) Dir = Unit(DirX, DirY);
            (Double Cx, Double Cy, Double MinAlong, Double MaxAlong) Span = SpanOf(Grid, Cells, Dir.X, Dir.Y);
            if (Double.IsNaN(Span.Cx))
            {
                return new PassLine(0.0, 0.0, 0.0, 0.0);
            }
            Double Half = 0.5 * Grid.GridSizeM * (Math.Abs(Dir.X) + Math.Abs(Dir.Y));
            Double X1 = Span.Cx + (Dir.X * (Span.MinAlong - Half));
            Double Y1 = Span.Cy + (Dir.Y * (Span.MinAlong - Half));
            Double X2 = Span.Cx + (Dir.X * (Span.MaxAlong + Half));
            Double Y2 = Span.Cy + (Dir.Y * (Span.MaxAlong + Half));
            return new PassLine(X1, Y1, X2, Y2);
        }

        /// <summary>
        /// One blade-wide band. The volume is what this lift can still move there.
        /// </summary>
        private sealed class Band
        {
            /// <summary>
            /// Creates an empty band.
            /// </summary>
            /// <param name="Index">Across-track bin. A smaller index is the tie-break.</param>
            public Band
                (
                Int32 Index
                )
            {
                this.Index = Index;
                this.Cells = new List<(Int32 Row, Int32 Col, Double Residual)>();
            }

            /// <summary>Gets the across-track bin.</summary>
            public Int32 Index { get; }

            /// <summary>Gets or sets the bank yards this lift can move in the band.</summary>
            public Double VolumeBcy { get; set; }

            /// <summary>Gets the cells that can still move.</summary>
            public List<(Int32 Row, Int32 Col, Double Residual)> Cells { get; }
        }

        /// <summary>
        /// Takes dirt until the payload is reached or the chosen bands are exhausted.
        /// A later band stays shut while an earlier band can still fill the payload,
        /// so a thin arm across the heading is left until its own band is opened.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Cells">The piece.</param>
        /// <param name="DirX">Haul heading east.</param>
        /// <param name="DirY">Haul heading north.</param>
        /// <param name="PayloadBcy">Bank yards to move.</param>
        /// <param name="Fill">True to raise the lowest holes. False to cut the highest cells.</param>
        /// <returns>The side, or null when nothing moves.</returns>
        private static Result? Take
            (
            Ground Ground,
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells,
            Double DirX,
            Double DirY,
            Double PayloadBcy,
            Boolean Fill
            )
        {
            if ((Ground == null) || (Cells == null) || (Cells.Count == 0) || (PayloadBcy <= EPS))
            {
                return null;
            }
            (Double X, Double Y) Dir = Unit(DirX, DirY);
            Double Bank = Fill ? Ground.FillToBank : 1.0;
            if (Bank <= EPS)
            {
                return null;
            }
            Double Area = Ground.Grid.GridSizeM * Ground.Grid.GridSizeM;
            Double YardsPerMetre = Area * ResidualClusters.M3_TO_CY * Bank;
            if (YardsPerMetre <= EPS)
            {
                return null;
            }
            Dictionary<Int32, Band> ByBand = new Dictionary<Int32, Band>();
            HashSet<(Int32 Row, Int32 Col)> Seen = new HashSet<(Int32 Row, Int32 Col)>();
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                if (!Seen.Add(Cell))
                {
                    continue;
                }
                if (!OnField(Ground, Cell.Col, Cell.Row))
                {
                    continue;
                }
                Double Room = Fill
                    ? AvailableFillM(Ground, Cell.Col, Cell.Row)
                    : AvailableCutM(Ground, Cell.Col, Cell.Row);
                if (Room <= EPS)
                {
                    continue;
                }
                Int32 Index = BandIndex(Ground.Grid, Cell.Col, Cell.Row, Dir.X, Dir.Y);
                if (!ByBand.TryGetValue(Index, out Band? Group) || (Group == null))
                {
                    Group = new Band(Index);
                    ByBand[Index] = Group;
                }
                Group.Cells.Add((Cell.Row, Cell.Col, Ground.ResidualMm[Cell.Row, Cell.Col]));
                Group.VolumeBcy += Room * YardsPerMetre;
            }
            if (ByBand.Count == 0)
            {
                return null;
            }
            // The fullest band goes first. An equal band breaks toward the lower across index.
            List<Band> Order = new List<Band>(ByBand.Values);
            Order.Sort(CompareBand);
            List<(Int32 Row, Int32 Col)> Piece = new List<(Int32 Row, Int32 Col)>(Cells);
            Double Speed = Fill ? V_FILL_MPS : V_CUT_MPS;
            Result Plan = new Result(Dir.X, Dir.Y, Speed, Fill, Piece);
            Double Left = PayloadBcy;
            foreach (Band Group in Order)
            {
                if (Left <= 1e-8)
                {
                    break;
                }
                if (Fill)
                {
                    Group.Cells.Sort(CompareFill);
                }
                else
                {
                    Group.Cells.Sort(CompareCut);
                }
                foreach ((Int32 Row, Int32 Col, Double Residual) Cell in Group.Cells)
                {
                    if (Left <= 1e-8)
                    {
                        break;
                    }
                    Double Room = Fill
                        ? AvailableFillM(Ground, Cell.Col, Cell.Row)
                        : AvailableCutM(Ground, Cell.Col, Cell.Row);
                    if (Room <= EPS)
                    {
                        continue;
                    }
                    Double Full = Room * YardsPerMetre;
                    Double TakeBcy = Full < Left ? Full : Left;
                    Double Depth = TakeBcy / YardsPerMetre;
                    if (Depth <= EPS)
                    {
                        continue;
                    }
                    Plan.Cells.Add(((Cell.Col, Cell.Row), Depth));
                    Left -= TakeBcy;
                }
            }
            if (Plan.Cells.Count == 0)
            {
                return null;
            }
            Plan.PlaceEnds(Ground);
            Plan.Retiming(Ground);
            return Plan;
        }

        /// <summary>
        /// Fullest band first. Equal volume breaks toward the smaller across index.
        /// </summary>
        /// <param name="Left">One band.</param>
        /// <param name="Right">The other band.</param>
        /// <returns>The sort order.</returns>
        private static Int32 CompareBand
            (
            Band Left,
            Band Right
            )
        {
            Int32 ByVolume = Right.VolumeBcy.CompareTo(Left.VolumeBcy);
            if (ByVolume != 0)
            {
                return ByVolume;
            }
            return Left.Index.CompareTo(Right.Index);
        }

        /// <summary>
        /// Across-track bin of a cell. Bands are one blade wide and parallel to the heading.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Col">Column.</param>
        /// <param name="Row">Row.</param>
        /// <param name="DirX">Heading east, unit length.</param>
        /// <param name="DirY">Heading north, unit length.</param>
        /// <returns>The bin.</returns>
        private static Int32 BandIndex
            (
            Grid Grid,
            Int32 Col,
            Int32 Row,
            Double DirX,
            Double DirY
            )
        {
            (Double X, Double Y) At = Center(Grid, Col, Row);
            Double Across = ((-DirY) * At.X) + (DirX * At.Y);
            return (Int32)Math.Floor(Across / BLADE_M);
        }

        /// <summary>
        /// Length and centreline of each band that has cells with depth.
        /// The length is the span of those cells along the heading.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cells">Cells that moved. Null or zero depth adds nothing.</param>
        /// <param name="DirX">Heading east.</param>
        /// <param name="DirY">Heading north.</param>
        /// <returns>One entry per band, in across order.</returns>
        private static List<(Double LengthM, PassLine Line)> MeasureBands
            (
            Grid Grid,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>? Cells,
            Double DirX,
            Double DirY
            )
        {
            List<(Double LengthM, PassLine Line)> Runs = new List<(Double LengthM, PassLine Line)>();
            if ((Grid == null) || (Cells == null) || (Cells.Count == 0) || (BLADE_M <= EPS))
            {
                return Runs;
            }
            (Double X, Double Y) Dir = Unit(DirX, DirY);
            Dictionary<Int32, List<(Int32 Col, Int32 Row)>> Groups =
                new Dictionary<Int32, List<(Int32 Col, Int32 Row)>>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Cells)
            {
                if (Lift.DepthM <= EPS)
                {
                    continue;
                }
                Int32 Col = Lift.Cell.Bx;
                Int32 Row = Lift.Cell.By;
                if ((Col < 0) || (Row < 0) || (Col >= Grid.Width) || (Row >= Grid.Height))
                {
                    continue;
                }
                Int32 Index = BandIndex(Grid, Col, Row, Dir.X, Dir.Y);
                if (!Groups.TryGetValue(Index, out List<(Int32 Col, Int32 Row)>? Group) || (Group == null))
                {
                    Group = new List<(Int32 Col, Int32 Row)>();
                    Groups[Index] = Group;
                }
                Group.Add((Col, Row));
            }
            List<Int32> Keys = new List<Int32>(Groups.Keys);
            Keys.Sort();
            Double CellAlong = Grid.GridSizeM * (Math.Abs(Dir.X) + Math.Abs(Dir.Y));
            Double Half = 0.5 * CellAlong;
            foreach (Int32 Key in Keys)
            {
                List<(Int32 Col, Int32 Row)> Group = Groups[Key];
                Boolean Any = false;
                Double MinAlong = 0.0;
                Double MaxAlong = 0.0;
                foreach ((Int32 Col, Int32 Row) Cell in Group)
                {
                    (Double X, Double Y) At = Center(Grid, Cell.Col, Cell.Row);
                    Double Along = (At.X * Dir.X) + (At.Y * Dir.Y);
                    if (!Any || (Along < MinAlong))
                    {
                        MinAlong = Along;
                    }
                    if (!Any || (Along > MaxAlong))
                    {
                        MaxAlong = Along;
                    }
                    Any = true;
                }
                if (!Any)
                {
                    continue;
                }
                Double Length = (MaxAlong - MinAlong) + CellAlong;
                if (Length < CellAlong)
                {
                    Length = CellAlong;
                }
                // The centre of the bin, so the line sits on the cells of this band.
                Double Across = (Key + 0.5) * BLADE_M;
                Double StartAlong = MinAlong - Half;
                Double EndAlong = MaxAlong + Half;
                Double X1 = (StartAlong * Dir.X) - (Across * Dir.Y);
                Double Y1 = (StartAlong * Dir.Y) + (Across * Dir.X);
                Double X2 = (EndAlong * Dir.X) - (Across * Dir.Y);
                Double Y2 = (EndAlong * Dir.Y) + (Across * Dir.X);
                Runs.Add((Length, new PassLine(X1, Y1, X2, Y2)));
            }
            return Runs;
        }

        /// <summary>
        /// Higher residual first. A tie goes to the smaller row, then the smaller column.
        /// </summary>
        /// <param name="Left">One cell.</param>
        /// <param name="Right">The other cell.</param>
        /// <returns>The sort order.</returns>
        private static Int32 CompareCut
            (
            (Int32 Row, Int32 Col, Double Residual) Left,
            (Int32 Row, Int32 Col, Double Residual) Right
            )
        {
            Int32 ByHeight = Right.Residual.CompareTo(Left.Residual);
            if (ByHeight != 0)
            {
                return ByHeight;
            }
            if (Left.Row != Right.Row)
            {
                return Left.Row.CompareTo(Right.Row);
            }
            return Left.Col.CompareTo(Right.Col);
        }

        /// <summary>
        /// Lower residual first, so the deepest hole is filled before a shallower one.
        /// A tie goes to the smaller row, then the smaller column.
        /// </summary>
        /// <param name="Left">One cell.</param>
        /// <param name="Right">The other cell.</param>
        /// <returns>The sort order.</returns>
        private static Int32 CompareFill
            (
            (Int32 Row, Int32 Col, Double Residual) Left,
            (Int32 Row, Int32 Col, Double Residual) Right
            )
        {
            Int32 ByHole = Left.Residual.CompareTo(Right.Residual);
            if (ByHole != 0)
            {
                return ByHole;
            }
            if (Left.Row != Right.Row)
            {
                return Left.Row.CompareTo(Right.Row);
            }
            return Left.Col.CompareTo(Right.Col);
        }

        /// <summary>
        /// Metres of cut available on a cell, capped at one lift and at the machine.
        /// The grade line is the stop, so a cell is never cut below target.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Col">Column.</param>
        /// <param name="Row">Row.</param>
        /// <returns>Metres. Zero when the cell is already at the grade line.</returns>
        private static Double AvailableCutM
            (
            Ground Ground,
            Int32 Col,
            Int32 Row
            )
        {
            Double Mm = Ground.ResidualMm[Row, Col] - Ground.ToleranceMm;
            if (Mm <= 1e-6)
            {
                return 0.0;
            }
            return CapDepth(Mm / 1000.0, CutCapM(Ground));
        }

        /// <summary>
        /// Metres of fill available on a cell, capped at one spread and at the machine.
        /// The grade line is the stop, so a cell is never raised above target.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Col">Column.</param>
        /// <param name="Row">Row.</param>
        /// <returns>Metres. Zero when the cell is already at the grade line.</returns>
        private static Double AvailableFillM
            (
            Ground Ground,
            Int32 Col,
            Int32 Row
            )
        {
            Double Mm = -Ground.ResidualMm[Row, Col] - Ground.ToleranceMm;
            if (Mm <= 1e-6)
            {
                return 0.0;
            }
            return CapDepth(Mm / 1000.0, SpreadCapM(Ground));
        }

        /// <summary>
        /// The smaller of the cut rule and the machine's deepest pass.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <returns>Metres.</returns>
        private static Double CutCapM
            (
            Ground Ground
            )
        {
            return CapDepth(CUT_DEPTH_CAP_MM / 1000.0, Ground.Grid.MaxDepthMm / 1000.0);
        }

        /// <summary>
        /// The smaller of the spread rule and the machine's deepest pass.
        /// A 100 mm rule on a 61 mm machine is rejected by the ledger, so the machine wins.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <returns>Metres.</returns>
        private static Double SpreadCapM
            (
            Ground Ground
            )
        {
            return CapDepth(SPREAD_LIFT_M, Ground.Grid.MaxDepthMm / 1000.0);
        }

        /// <summary>
        /// The smaller of two positive depths.
        /// </summary>
        /// <param name="DepthM">One depth.</param>
        /// <param name="CapM">The other depth.</param>
        /// <returns>The smaller. Zero when either is not positive.</returns>
        private static Double CapDepth
            (
            Double DepthM,
            Double CapM
            )
        {
            if ((DepthM <= 0.0) || (CapM <= 0.0))
            {
                return 0.0;
            }
            return DepthM < CapM ? DepthM : CapM;
        }

        /// <summary>
        /// True when the column and row are on the mask.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Col">Column.</param>
        /// <param name="Row">Row.</param>
        /// <returns>True when the machine may work this cell.</returns>
        private static Boolean OnField
            (
            Ground Ground,
            Int32 Col,
            Int32 Row
            )
        {
            if ((Col < 0) || (Row < 0) || (Col >= Ground.Grid.Width) || (Row >= Ground.Grid.Height))
            {
                return false;
            }
            if ((Row >= Ground.Mask.GetLength(0)) || (Col >= Ground.Mask.GetLength(1)))
            {
                return false;
            }
            return Ground.Mask[Row, Col];
        }

        /// <summary>
        /// Centre of a cell in world metres.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Col">Column.</param>
        /// <param name="Row">Row.</param>
        /// <returns>The centre.</returns>
        private static (Double X, Double Y) Center
            (
            Grid Grid,
            Int32 Col,
            Int32 Row
            )
        {
            Double X = Grid.MinX + ((Col + 0.5) * Grid.GridSizeM);
            Double Y = Grid.MinY + ((Row + 0.5) * Grid.GridSizeM);
            return (X, Y);
        }

        /// <summary>
        /// Unit vector. A zero vector becomes east so a piece sitting on its fill still has a heading.
        /// </summary>
        /// <param name="X">Easting component.</param>
        /// <param name="Y">Northing component.</param>
        /// <returns>The unit vector.</returns>
        private static (Double X, Double Y) Unit
            (
            Double X,
            Double Y
            )
        {
            Double Length = Math.Sqrt((X * X) + (Y * Y));
            if (Length < 1e-9)
            {
                return (1.0, 0.0);
            }
            return (X / Length, Y / Length);
        }

        /// <summary>
        /// Centroid and the along-track extent of the cell centres, relative to that centroid.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Cells">The piece.</param>
        /// <param name="DirX">Heading east, unit length.</param>
        /// <param name="DirY">Heading north, unit length.</param>
        /// <returns>The centroid and the extent. The easting is NaN when no cell is on the grid.</returns>
        private static (Double Cx, Double Cy, Double MinAlong, Double MaxAlong) SpanOf
            (
            Grid Grid,
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells,
            Double DirX,
            Double DirY
            )
        {
            Double SumX = 0.0;
            Double SumY = 0.0;
            Int32 Count = 0;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                if ((Cell.Col < 0) || (Cell.Row < 0) || (Cell.Col >= Grid.Width) || (Cell.Row >= Grid.Height))
                {
                    continue;
                }
                (Double X, Double Y) At = Center(Grid, Cell.Col, Cell.Row);
                SumX += At.X;
                SumY += At.Y;
                Count += 1;
            }
            if (Count == 0)
            {
                return (Double.NaN, 0.0, 0.0, 0.0);
            }
            Double Cx = SumX / Count;
            Double Cy = SumY / Count;
            Double MinAlong = 0.0;
            Double MaxAlong = 0.0;
            Boolean Any = false;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                if ((Cell.Col < 0) || (Cell.Row < 0) || (Cell.Col >= Grid.Width) || (Cell.Row >= Grid.Height))
                {
                    continue;
                }
                (Double X, Double Y) At = Center(Grid, Cell.Col, Cell.Row);
                Double Along = ((At.X - Cx) * DirX) + ((At.Y - Cy) * DirY);
                if (!Any || (Along < MinAlong))
                {
                    MinAlong = Along;
                }
                if (!Any || (Along > MaxAlong))
                {
                    MaxAlong = Along;
                }
                Any = true;
            }
            return (Cx, Cy, MinAlong, MaxAlong);
        }
    }
}
