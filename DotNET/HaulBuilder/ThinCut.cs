using System.Globalization;

namespace HaulBuilder
{
    /// <summary>
    /// Thin-cut after cluster hauling stops. Cut and fill both come from square
    /// tiles on mask E. A cut tile is the soil above 10 mm. A fill tile is the
    /// hole below −30 mm, taken all the way to the target. Pairing, merging, and
    /// the trip schedule are the cluster steps. The loop replans every ten trips
    /// and stops when a fresh plan has no trips.
    /// </summary>
    static public class ThinCut
    {
        /// <summary>The loop ran and the last plan scheduled nothing.</summary>
        public const String NO_HAULABLE_WORK = "no haulable work";

        /// <summary>Cluster hauling left less than half a bowl of SHORT fill.</summary>
        public const String DID_NOT_RUN = "did not run";

        /// <summary>Error end. The same work kept coming back. Correcting does not start.</summary>
        public const String TRIP_LIMIT = "trip limit";

        /// <summary>Cells on one side of a tile. Eight cells is about 20 m on the shop grid.</summary>
        public const Int32 TILE_CELLS = 8;

        /// <summary>Tile cut volume ignores residual at or below this many millimetres.</summary>
        public const Double MEASURE_ABOVE_MM = 10.0;

        /// <summary>
        /// A fill tile counts a cell only when it is below this many millimetres.
        /// The volume is the full hole up to the target, not the part past this line.
        /// </summary>
        public const Double FILL_BELOW_MM = 30.0;

        /// <summary>Kept so older reports can still name the retired half-bowl gate.</summary>
        public const Double SHORT_BOWL_FRACTION = 0.5;

        /// <summary>Keeps two reports from writing the same temp file at once.</summary>
        private static readonly Object ReportGate = new Object();

        /// <summary>
        /// Runs thin-cut, appends the trips to <paramref name="Passes"/>,
        /// and subtracts them from the remaining cut and fill.
        /// </summary>
        /// <param name="Grid">Field grid. Mask E is its valid cells.</param>
        /// <param name="Ledger">Work ledger. Its yard factors balance each trip.</param>
        /// <param name="CutLeft">Remaining cut depth in metres. Each trip reduces it.</param>
        /// <param name="FillLeft">Remaining fill depth in metres. Each trip reduces it.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="Passes">Found-order routes. Thin-cut trips are appended.</param>
        /// <returns>Why the loop stopped. Correcting follows when the reason is not the trip limit.</returns>
        public static ThinCutStop Run
            (
            Grid Grid,
            HaulWorkLedger Ledger,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double BowlBcy,
            List<Pass> Passes
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            if (CutLeft == null) throw new ArgumentNullException(nameof(CutLeft));
            if (FillLeft == null) throw new ArgumentNullException(nameof(FillLeft));
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            if (BowlBcy <= 0.0) throw new ArgumentOutOfRangeException(nameof(BowlBcy));
            String Reason = NO_HAULABLE_WORK;
            // The loop assigns this before it stops. The empty plan keeps the compiler quiet.
            ResidualPairPlan Last = ResidualPairing.Plan(
                new List<ResidualPairPiece>(),
                Grid.GridSizeM,
                BowlBcy,
                Constants.DEFAULT_BLADE_WIDTH_MM / Constants.MILLIMETERS_PER_METER);
            Int32 TripsDone = 0;
            Int32 RemainingTrips = 0;
            Double RemainingBcy = 0.0;
            Double PayloadSum = 0.0;
            while (true)
            {
                Double[,] ResidualMm = ClusterHauling.ResidualMillimetres(CutLeft, FillLeft, Grid);
                ClusterSchedule Schedule = PlanField(ResidualMm, Grid, BowlBcy);
                Last = Schedule.Plan;
                if (Last.Trips.Count == 0)
                {
                    Reason = NO_HAULABLE_WORK;
                    break;
                }
                Int32 Take = Math.Min(ClusterHauling.TRIPS_PER_PLAN, Last.Trips.Count);
                List<ResidualPairTrip> Batch = new List<ResidualPairTrip>(Take);
                for (Int32 Index = 0; Index < Take; Index += 1)
                {
                    Batch.Add(Last.Trips[Index]);
                    PayloadSum += Last.Trips[Index].PayloadBcy;
                }
                ClusterHauling.ExecuteBatch(
                    ResidualMm, Schedule, Batch, Grid, Ledger, CutLeft, FillLeft, BowlBcy, Passes);
                TripsDone += Take;
                // The same stuck-planner cap as cluster hauling. The batch is already done.
                if (TripsDone >= ClusterHauling.MAX_CLUSTER_TRIPS)
                {
                    Reason = TRIP_LIMIT;
                    RemainingTrips = Last.Trips.Count - Take;
                    RemainingBcy = 0.0;
                    for (Int32 Index = Take; Index < Last.Trips.Count; Index += 1)
                    {
                        RemainingBcy += Last.Trips[Index].PayloadBcy;
                    }
                    break;
                }
            }
            Double Mean = TripsDone == 0 ? 0.0 : PayloadSum / TripsDone;
            ThinCutStop Stop = new ThinCutStop(
                Reason,
                TripsDone,
                RemainingTrips,
                RemainingBcy,
                ShortOf(Last),
                LocalOf(Last),
                Mean);
            Report(Stop, CutLeft, FillLeft, Grid);
            return Stop;
        }

        /// <summary>
        /// The state after a stop. A finished thin-cut enters correcting.
        /// A trip limit throws, so correcting does not start.
        /// Did-not-run is a finish: the field was already short of the half-bowl gate.
        /// </summary>
        /// <param name="Stop">The stop from the thin-cut loop.</param>
        /// <returns>Correcting, when thin-cut finished or did not run.</returns>
        public static HaulSearchState Handoff
            (
            ThinCutStop Stop
            )
        {
            if (Stop == null) throw new ArgumentNullException(nameof(Stop));
            if (Stop.Reason == TRIP_LIMIT)
            {
                throw new InvalidOperationException(String.Format(
                    CultureInfo.InvariantCulture,
                    "thin-cut stopped: trip limit. remaining trips={0} bcy={1:R}",
                    Stop.RemainingTrips,
                    Stop.RemainingBcy));
            }
            return HaulSearchState.Correcting;
        }

        /// <summary>
        /// Tiles the cut and the fill, then pairs those tiles.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres. Positive is cut.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <returns>The schedule for this residual.</returns>
        private static ClusterSchedule PlanField
            (
            Double[,] ResidualMm,
            Grid Grid,
            Double BowlBcy
            )
        {
            List<ResidualPairPiece> Pieces = CutTiles(ResidualMm, Grid, BowlBcy);
            Pieces.AddRange(FillTiles(ResidualMm, Grid, BowlBcy));
            ResidualPairPlan Plan = ResidualPairing.Plan(
                Pieces,
                Grid.GridSizeM,
                BowlBcy,
                Constants.DEFAULT_BLADE_WIDTH_MM / Constants.MILLIMETERS_PER_METER);
            return new ClusterSchedule(Plan, Pieces);
        }

        /// <summary>
        /// Square tiles of mask E. A tile is kept when its cut above 10 mm
        /// is at least a quarter bowl. The piece mean is that volume divided
        /// by the area of the cells that contributed it. The long axis is the tile side.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres. Positive is cut.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <returns>The cut pieces, one per kept tile.</returns>
        private static List<ResidualPairPiece> CutTiles
            (
            Double[,] ResidualMm,
            Grid Grid,
            Double BowlBcy
            )
        {
            List<ResidualPairPiece> Tiles = new List<ResidualPairPiece>();
            Double KeepBcy = ResidualPairing.PARTIAL_BOWL_FRACTION * BowlBcy;
            Double CellArea = Grid.GridSizeM * Grid.GridSizeM;
            Double SideM = TILE_CELLS * Grid.GridSizeM;
            Int32 TileRows = (Grid.Height + TILE_CELLS - 1) / TILE_CELLS;
            Int32 TileCols = (Grid.Width + TILE_CELLS - 1) / TILE_CELLS;
            for (Int32 TileRow = 0; TileRow < TileRows; TileRow += 1)
            {
                for (Int32 TileCol = 0; TileCol < TileCols; TileCol += 1)
                {
                    ResidualPairPiece? Tile = MeasureTile(
                        ResidualMm,
                        Grid,
                        TileRow,
                        TileCol,
                        CellArea,
                        SideM,
                        KeepBcy);
                    if (Tile != null)
                    {
                        Tiles.Add(Tile);
                    }
                }
            }
            return Tiles;
        }

        /// <summary>
        /// Measures one tile. A grid cell sits wholly inside one tile, so its
        /// inside fraction is 1 on mask E and 0 off the field.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="TileRow">Tile row, from the north-west origin of the grid.</param>
        /// <param name="TileCol">Tile column.</param>
        /// <param name="CellArea">One cell in square metres.</param>
        /// <param name="SideM">Full tile side in metres.</param>
        /// <param name="KeepBcy">Minimum cut volume, in bank cubic yards.</param>
        /// <returns>The piece, or null when the tile is under a quarter bowl.</returns>
        private static ResidualPairPiece? MeasureTile
            (
            Double[,] ResidualMm,
            Grid Grid,
            Int32 TileRow,
            Int32 TileCol,
            Double CellArea,
            Double SideM,
            Double KeepBcy
            )
        {
            Int32 Row0 = TileRow * TILE_CELLS;
            Int32 Col0 = TileCol * TILE_CELLS;
            Int32 Row1 = Math.Min(Row0 + TILE_CELLS, Grid.Height);
            Int32 Col1 = Math.Min(Col0 + TILE_CELLS, Grid.Width);
            Double VolumeM3 = 0.0;
            Double CutArea = 0.0;
            Double WeightRow = 0.0;
            Double WeightCol = 0.0;
            List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>();
            for (Int32 Row = Row0; Row < Row1; Row += 1)
            {
                for (Int32 Col = Col0; Col < Col1; Col += 1)
                {
                    if (!Grid.Valid[Row, Col])
                    {
                        continue;
                    }
                    Double Residual = ResidualMm[Row, Col];
                    // A trip may take any cell above the target. The tile volume does not count the last 10 mm.
                    if (Residual > ClusterHauling.CUT_TO_MM)
                    {
                        Cells.Add((Row, Col));
                    }
                    Double AboveMm = Residual - MEASURE_ABOVE_MM;
                    if (AboveMm <= 0.0)
                    {
                        continue;
                    }
                    // The cell is wholly inside this tile and on the field.
                    Double InsideFraction = 1.0;
                    Double DepthM = AboveMm / 1000.0;
                    VolumeM3 += DepthM * CellArea * InsideFraction;
                    CutArea += CellArea * InsideFraction;
                    WeightRow += DepthM * InsideFraction * Row;
                    WeightCol += DepthM * InsideFraction * Col;
                }
            }
            if ((CutArea <= 0.0) || (VolumeM3 <= 0.0))
            {
                return null;
            }
            Double VolumeBcy = VolumeM3 * ResidualClusters.M3_TO_CY;
            if (VolumeBcy < KeepBcy)
            {
                return null;
            }
            // mean_mm is the tile cut volume divided by the area of the cells that produced it.
            Double MeanMm = (VolumeM3 / CutArea) * 1000.0;
            if ((MeanMm <= 0.0) || (SideM <= 0.0) || (Cells.Count == 0))
            {
                return null;
            }
            Double DepthWeight = VolumeM3 / CellArea;
            Double CentroidRow = WeightRow / DepthWeight;
            Double CentroidCol = WeightCol / DepthWeight;
            return new ResidualPairPiece(
                "T" + TileRow.ToString(CultureInfo.InvariantCulture)
                    + "."
                    + TileCol.ToString(CultureInfo.InvariantCulture),
                "cut",
                VolumeBcy,
                MeanMm,
                CentroidRow,
                CentroidCol,
                SideM,
                Cells);
        }

        /// <summary>
        /// Square tiles of mask E. A tile is kept when the hole below −30 mm,
        /// converted with the bank factor, is at least a quarter bowl.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres. Negative is fill.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <returns>The fill pieces, one per kept tile.</returns>
        private static List<ResidualPairPiece> FillTiles
            (
            Double[,] ResidualMm,
            Grid Grid,
            Double BowlBcy
            )
        {
            List<ResidualPairPiece> Tiles = new List<ResidualPairPiece>();
            Double KeepBcy = ResidualPairing.PARTIAL_BOWL_FRACTION * BowlBcy;
            Double CellArea = Grid.GridSizeM * Grid.GridSizeM;
            Double SideM = TILE_CELLS * Grid.GridSizeM;
            Int32 TileRows = (Grid.Height + TILE_CELLS - 1) / TILE_CELLS;
            Int32 TileCols = (Grid.Width + TILE_CELLS - 1) / TILE_CELLS;
            for (Int32 TileRow = 0; TileRow < TileRows; TileRow += 1)
            {
                for (Int32 TileCol = 0; TileCol < TileCols; TileCol += 1)
                {
                    ResidualPairPiece? Tile = MeasureFillTile(
                        ResidualMm,
                        Grid,
                        TileRow,
                        TileCol,
                        CellArea,
                        SideM,
                        KeepBcy);
                    if (Tile != null)
                    {
                        Tiles.Add(Tile);
                    }
                }
            }
            return Tiles;
        }

        /// <summary>
        /// Measures one fill tile. Only cells below −30 mm count. The depth is
        /// the whole hole up to the target, then multiplied by the bank factor.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="TileRow">Tile row.</param>
        /// <param name="TileCol">Tile column.</param>
        /// <param name="CellArea">One cell in square metres.</param>
        /// <param name="SideM">Full tile side in metres.</param>
        /// <param name="KeepBcy">Minimum fill volume, in bank cubic yards.</param>
        /// <returns>The piece, or null when the tile is under a quarter bowl.</returns>
        private static ResidualPairPiece? MeasureFillTile
            (
            Double[,] ResidualMm,
            Grid Grid,
            Int32 TileRow,
            Int32 TileCol,
            Double CellArea,
            Double SideM,
            Double KeepBcy
            )
        {
            Int32 Row0 = TileRow * TILE_CELLS;
            Int32 Col0 = TileCol * TILE_CELLS;
            Int32 Row1 = Math.Min(Row0 + TILE_CELLS, Grid.Height);
            Int32 Col1 = Math.Min(Col0 + TILE_CELLS, Grid.Width);
            Double CompactedM3 = 0.0;
            Double FillArea = 0.0;
            Double WeightRow = 0.0;
            Double WeightCol = 0.0;
            List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>();
            for (Int32 Row = Row0; Row < Row1; Row += 1)
            {
                for (Int32 Col = Col0; Col < Col1; Col += 1)
                {
                    if (!Grid.Valid[Row, Col])
                    {
                        continue;
                    }
                    Double Residual = ResidualMm[Row, Col];
                    // Outside the −30 mm line. The volume is max(-r, 0), the full hole.
                    if (Residual >= -FILL_BELOW_MM)
                    {
                        continue;
                    }
                    Double DepthM = Math.Max(-Residual, 0.0) / 1000.0;
                    if (DepthM <= 0.0)
                    {
                        continue;
                    }
                    Double InsideFraction = 1.0;
                    CompactedM3 += DepthM * CellArea * InsideFraction;
                    FillArea += CellArea * InsideFraction;
                    WeightRow += DepthM * InsideFraction * Row;
                    WeightCol += DepthM * InsideFraction * Col;
                    Cells.Add((Row, Col));
                }
            }
            if ((FillArea <= 0.0) || (CompactedM3 <= 0.0))
            {
                return null;
            }
            Double BankM3 = CompactedM3 * Grid.Compaction;
            Double VolumeBcy = BankM3 * ResidualClusters.M3_TO_CY;
            if (VolumeBcy < KeepBcy)
            {
                return null;
            }
            Double MeanMm = (CompactedM3 / FillArea) * 1000.0;
            if ((MeanMm <= 0.0) || (SideM <= 0.0) || (Cells.Count == 0))
            {
                return null;
            }
            Double DepthWeight = CompactedM3 / CellArea;
            Double CentroidRow = WeightRow / DepthWeight;
            Double CentroidCol = WeightCol / DepthWeight;
            return new ResidualPairPiece(
                "F" + TileRow.ToString(CultureInfo.InvariantCulture)
                    + "."
                    + TileCol.ToString(CultureInfo.InvariantCulture),
                "fill",
                VolumeBcy,
                MeanMm,
                CentroidRow,
                CentroidCol,
                SideM,
                Cells);
        }

        /// <summary>
        /// SHORT fill on a plan. Zero when the plan is balanced or long on cut.
        /// </summary>
        /// <param name="Plan">A pairing.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double ShortOf
            (
            ResidualPairPlan Plan
            )
        {
            if (Plan.BalanceKind == ResidualPairing.SHORT_ID)
            {
                return Plan.BalanceBcy;
            }
            return 0.0;
        }

        /// <summary>
        /// LOCAL cut on a plan. Zero when the plan is balanced or short of cut.
        /// </summary>
        /// <param name="Plan">A pairing.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double LocalOf
            (
            ResidualPairPlan Plan
            )
        {
            if (Plan.BalanceKind == ResidualPairing.LOCAL_ID)
            {
                return Plan.BalanceBcy;
            }
            return 0.0;
        }

        /// <summary>
        /// Prints the handoff. The group list is the cluster report's list.
        /// </summary>
        /// <param name="Stop">Why the loop stopped.</param>
        /// <param name="CutLeft">Cut still remaining.</param>
        /// <param name="FillLeft">Fill still remaining.</param>
        /// <param name="Grid">Field grid.</param>
        private static void Report
            (
            ThinCutStop Stop,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Grid Grid
            )
        {
            List<String> Lines = new List<String>();
            if (Stop.Reason == DID_NOT_RUN)
            {
                Say(Lines, "thin-cut did not run");
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "SHORT fill: {0:R} bcy",
                    Stop.ShortFillBcy));
                Say(Lines, String.Format(CultureInfo.InvariantCulture, "trips: {0}", Stop.TripsDone));
            }
            else
            {
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "thin-cut stopped: {0}",
                    Stop.Reason));
                if (Stop.Reason == TRIP_LIMIT)
                {
                    Say(Lines, String.Format(
                        CultureInfo.InvariantCulture,
                        "remaining plan: {0} trips, {1:R} bcy",
                        Stop.RemainingTrips,
                        Stop.RemainingBcy));
                }
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "SHORT fill: {0:R} bcy",
                    Stop.ShortFillBcy));
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "LOCAL cut: {0:R} bcy",
                    Stop.LocalCutBcy));
                Say(Lines, String.Format(CultureInfo.InvariantCulture, "trips: {0}", Stop.TripsDone));
                Say(Lines, String.Format(
                    CultureInfo.InvariantCulture,
                    "mean payload: {0:R} bcy",
                    Stop.MeanPayloadBcy));
                Double[,] ResidualMm = ClusterHauling.ResidualMillimetres(CutLeft, FillLeft, Grid);
                ClusterHauling.AppendFieldGroups(Lines, ResidualMm, Grid, Grid.Valid);
            }
            lock (ReportGate)
            {
                String ReportPath = Path.Combine(Path.GetTempPath(), "thin-cut-report.txt");
                try
                {
                    File.WriteAllLines(ReportPath, Lines);
                }
                catch (IOException)
                {
                    File.WriteAllLines(
                        Path.Combine(Path.GetTempPath(), "thin-cut-report-" + Guid.NewGuid().ToString("N") + ".txt"),
                        Lines);
                }
            }
        }

        /// <summary>
        /// Prints one handoff line and keeps it for the temp copy.
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
    }

    /// <summary>
    /// Why thin-cut stopped, and the SHORT fill, LOCAL cut, and mean payload left to report.
    /// </summary>
    public sealed class ThinCutStop
    {
        /// <summary>
        /// Creates a stop.
        /// </summary>
        /// <param name="Reason">Did not run, no haulable work, or the trip limit.</param>
        /// <param name="TripsDone">Trips executed.</param>
        /// <param name="RemainingTrips">Trips on the last plan that were not executed.</param>
        /// <param name="RemainingBcy">Bank yards on those remaining trips.</param>
        /// <param name="ShortFillBcy">SHORT fill still unpaid, in bank cubic yards.</param>
        /// <param name="LocalCutBcy">LOCAL cut still unused, in bank cubic yards.</param>
        /// <param name="MeanPayloadBcy">Mean payload of the executed trips. Zero when none ran.</param>
        public ThinCutStop
            (
            String Reason,
            Int32 TripsDone,
            Int32 RemainingTrips,
            Double RemainingBcy,
            Double ShortFillBcy,
            Double LocalCutBcy,
            Double MeanPayloadBcy
            )
        {
            this.Reason = Reason;
            this.TripsDone = TripsDone;
            this.RemainingTrips = RemainingTrips;
            this.RemainingBcy = RemainingBcy;
            this.ShortFillBcy = ShortFillBcy;
            this.LocalCutBcy = LocalCutBcy;
            this.MeanPayloadBcy = MeanPayloadBcy;
        }

        /// <summary>Gets the stop reason.</summary>
        public String Reason { get; }

        /// <summary>Gets how many trips were executed.</summary>
        public Int32 TripsDone { get; }

        /// <summary>Gets the unexecuted trips when the trip limit fired.</summary>
        public Int32 RemainingTrips { get; }

        /// <summary>Gets the bank yards on the unexecuted trips.</summary>
        public Double RemainingBcy { get; }

        /// <summary>Gets the SHORT fill still unpaid, in bank cubic yards.</summary>
        public Double ShortFillBcy { get; }

        /// <summary>Gets the LOCAL cut still unused, in bank cubic yards.</summary>
        public Double LocalCutBcy { get; }

        /// <summary>Gets the mean payload of the executed trips, in bank cubic yards.</summary>
        public Double MeanPayloadBcy { get; }
    }
}
