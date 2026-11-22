using System.Text.Json;
using System.Text.Json.Serialization;

using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace HaulBuilder
{
    /// <summary>
    /// Builds one loaded haul for each scheduled cluster or thin-cut trip.
    /// A trip cuts the highest cells of one piece and fills the lowest cells of the paired piece.
    /// One cut takes at most 61 mm and stops at the target. One fill cell takes at most one spread and stops at the target.
    /// The next piece is used only when the larger one has nothing left above target.
    /// The haul is the straight line from the cut piece's exit face to the fill piece's entry face.
    /// </summary>
    static public class ResidualHauls
    {
        /// <summary>Blade width used to stripe a piece. One shop cell is one blade.</summary>
        private static readonly Double BLADE_M = Constants.DEFAULT_BLADE_WIDTH_MM / Constants.MILLIMETERS_PER_METER;

        /// <summary>
        /// Allocates cells and plans a haul for every trip, in schedule order.
        /// A later trip can take the next lift of a cell this trip did not finish.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres, indexed [row, col]. Positive is cut.</param>
        /// <param name="Valid">True on field cells.</param>
        /// <param name="Pieces">Pieces that were paired, including their cells.</param>
        /// <param name="Trips">Scheduled trips.</param>
        /// <param name="Grid">Field grid. Cell centres and the picture use this.</param>
        /// <param name="FillToBank">Bank yards of fill per loose yard. Clustering's compaction.</param>
        /// <param name="ToleranceMm">
        /// Grade tolerance in millimetres. A hauled cell is left within this of the target.
        /// </param>
        /// <param name="ArrowX">Smoothed flow east. The heading is the paired fill, not this arrow.</param>
        /// <param name="ArrowY">Smoothed flow north. The heading is the paired fill, not this arrow.</param>
        /// <param name="BowlBcy">Pan capacity. A trip payload above this is cut down to one bowl.</param>
        /// <returns>One haul per trip, in the same order.</returns>
        public static List<ResidualHaul> Build
            (
            Double[,] ResidualMm,
            Boolean[,] Valid,
            IReadOnlyList<ResidualPairPiece> Pieces,
            IReadOnlyList<ResidualPairTrip> Trips,
            Grid Grid,
            Double FillToBank,
            Double ToleranceMm = Constants.DEFAULT_TOLERANCE_MM,
            Double[,]? ArrowX = null,
            Double[,]? ArrowY = null,
            Double BowlBcy = 0.0
            )
        {
            if (ResidualMm == null) throw new ArgumentNullException(nameof(ResidualMm));
            if (Valid == null) throw new ArgumentNullException(nameof(Valid));
            if (Pieces == null) throw new ArgumentNullException(nameof(Pieces));
            if (Trips == null) throw new ArgumentNullException(nameof(Trips));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (FillToBank <= 0.0) throw new ArgumentOutOfRangeException(nameof(FillToBank));
            if (Grid.MaxDepthMm <= 0.0) throw new ArgumentOutOfRangeException(nameof(Grid));
            if (!Double.IsFinite(ToleranceMm) || (ToleranceMm < 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(ToleranceMm));
            }
            Dictionary<String, ResidualPairPiece> ById =
                new Dictionary<String, ResidualPairPiece>(StringComparer.Ordinal);
            foreach (ResidualPairPiece Piece in Pieces)
            {
                ById[Piece.Id] = Piece;
            }
            // Later trips in this batch must see what the earlier trips took.
            Double[,] Working = (Double[,])ResidualMm.Clone();
            StripLanes.Ground Ground = new StripLanes.Ground(
                Grid, Working, Valid, FillToBank, ToleranceMm, ArrowX, ArrowY);
            List<ResidualHaul> Hauls = new List<ResidualHaul>(Trips.Count);
            foreach (ResidualPairTrip Trip in Trips)
            {
                (Double FillX, Double FillY) = Anchor(Trip.Fills, ById, Grid);
                ResidualPairPiece? Piece = LargestCut(Ground, Trip, ById);
                List<(Int32 Row, Int32 Col)> FillPiece = PieceCells(Trip.Fills, ById);
                StripLanes.Result? CutPlan = null;
                StripLanes.Result? FillPlan = null;
                if (Piece != null)
                {
                    (Double CutX, Double CutY) = World(Grid, Piece.CentroidCol, Piece.CentroidRow);
                    (Double DirX, Double DirY) = StripLanes.Heading(CutX, CutY, FillX, FillY);
                    Double Payload = Trip.PayloadBcy;
                    if (Payload <= 0.0)
                    {
                        Payload = BowlBcy;
                    }
                    // The schedule is one bowl. A larger ask would fail the ledger.
                    if ((BowlBcy > 0.0) && (Payload > BowlBcy))
                    {
                        Payload = BowlBcy;
                    }
                    CutPlan = StripLanes.Cut(Ground, Piece.Cells, DirX, DirY, Payload);
                    if ((CutPlan != null) && (CutPlan.Bcy > 1e-9))
                    {
                        FillPlan = StripLanes.Fill(Ground, FillPiece, DirX, DirY, CutPlan.Bcy);
                        if (FillPlan != null)
                        {
                            // A fill that cannot hold the load trims the tail of the cut, then the time.
                            StripLanes.Match(CutPlan, FillPlan, Ground);
                        }
                    }
                }
                Boolean Kept = (CutPlan != null) && (FillPlan != null) && (CutPlan.Bcy > 1e-9) && (FillPlan.Bcy > 1e-9);
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts = Kept
                    ? new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(CutPlan!.Cells)
                    : new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills = Kept
                    ? new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(FillPlan!.Cells)
                    : new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                if (Kept)
                {
                    StripLanes.Apply(Ground, Cuts, 1);
                    // Fill raises the residual. Apply's positive sign lowers it, so the fill sign is negative.
                    StripLanes.Apply(Ground, Fills, -1);
                }
                List<GuidancePoint> Points = new List<GuidancePoint>();
                Double HaulM = 0.0;
                if (Kept)
                {
                    // The picture's haul arrow is the exit face to the entry face. There is no cell path.
                    Points.Add(MakePoint(1, "CUT", CutPlan!.EndX, CutPlan.EndY));
                    Points.Add(MakePoint(2, "FILL", FillPlan!.StartX, FillPlan.StartY));
                    HaulM = Metres(CutPlan.EndX, CutPlan.EndY, FillPlan.StartX, FillPlan.StartY);
                }
                Double CutWorkM = Kept ? CutPlan!.DistanceM : 0.0;
                Double FillWorkM = Kept ? FillPlan!.DistanceM : 0.0;
                Double Loaded = Kept ? CutPlan!.Bcy : 0.0;
                ResidualHaul Haul = new ResidualHaul(
                    Trip.TripNo,
                    Trip.PayloadBcy,
                    Loaded,
                    HaulM,
                    CutWorkM + HaulM + FillWorkM,
                    Kept ? CutPlan!.PassCount : 0,
                    Kept ? FillPlan!.PassCount : 0,
                    Cuts,
                    Fills,
                    Points);
                Haul.LoadSeconds = Kept ? CutPlan!.Seconds : 0.0;
                Haul.SpreadSeconds = Kept ? FillPlan!.Seconds : 0.0;
                Haul.LoadDistanceM = CutWorkM;
                Haul.SpreadDistanceM = FillWorkM;
                // The loop time is already inside the cut and fill seconds.
                Haul.LoadedTurns = 0;
                Haul.AreaBased = Kept;
                if (Kept && (Piece != null))
                {
                    Haul.DirX = CutPlan!.DirX;
                    Haul.DirY = CutPlan.DirY;
                    Haul.CutEntryX = CutPlan.StartX;
                    Haul.CutEntryY = CutPlan.StartY;
                    Haul.CutExitX = CutPlan.EndX;
                    Haul.CutExitY = CutPlan.EndY;
                    Haul.FillEntryX = FillPlan!.StartX;
                    Haul.FillEntryY = FillPlan.StartY;
                    Haul.FillExitX = FillPlan.EndX;
                    Haul.FillExitY = FillPlan.EndY;
                    Haul.CutOutline = OutlineOf(Piece.Cells);
                    Haul.FillOutline = OutlineOf(FillPiece);
                    Haul.CutPieceId = Piece.Id;
                    Haul.FillPieceId = SweepKey(Trip.Fills);
                }
                Hauls.Add(Haul);
            }
            return Hauls;
        }

        /// <summary>
        /// Writes one cropped haul picture. Cut cells are red, fill cells are blue,
        /// the white line is the blade, and the black arrow is the loaded haul.
        /// </summary>
        /// <param name="Path">PNG path to write.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Haul">The haul to draw.</param>
        /// <param name="SurfaceM">
        /// Existing-minus-target still in the ground, in metres, indexed [row, col].
        /// </param>
        /// <param name="PixelsPerCell">Pixels along one cell.</param>
        public static void SavePng
            (
            String Path,
            Grid Grid,
            ResidualHaul Haul,
            Double[,] SurfaceM,
            Int32 PixelsPerCell
            )
        {
            if (String.IsNullOrEmpty(Path)) throw new ArgumentNullException(nameof(Path));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Haul == null) throw new ArgumentNullException(nameof(Haul));
            if (SurfaceM == null) throw new ArgumentNullException(nameof(SurfaceM));
            if (PixelsPerCell < 1) throw new ArgumentOutOfRangeException(nameof(PixelsPerCell));
            Byte[] Png = Render.RenderSinglePairPng(
                Grid,
                Haul.Points,
                BLADE_M,
                PixelsPerCell,
                4,
                false,
                Haul.CutCells,
                Haul.FillCells,
                null,
                null,
                1,
                SurfaceM,
                false);
            File.WriteAllBytes(Path, Png);
        }

        /// <summary>
        /// Writes the whole field in the cut and fill palette. North is up.
        /// </summary>
        /// <param name="Path">PNG path to write.</param>
        /// <param name="SurfaceM">Existing-minus-target in metres, indexed [row, col]. Positive is high.</param>
        /// <param name="Valid">True on field cells. Other cells are left clear.</param>
        /// <param name="PixelsPerCell">Pixels along one cell.</param>
        public static void SaveFieldPng
            (
            String Path,
            Double[,] SurfaceM,
            Boolean[,] Valid,
            Int32 PixelsPerCell
            )
        {
            if (String.IsNullOrEmpty(Path)) throw new ArgumentNullException(nameof(Path));
            if (SurfaceM == null) throw new ArgumentNullException(nameof(SurfaceM));
            if (Valid == null) throw new ArgumentNullException(nameof(Valid));
            if (PixelsPerCell < 1) throw new ArgumentOutOfRangeException(nameof(PixelsPerCell));
            using Image<Rgba32> Sheet = Render.DiffToImageDigitize(SurfaceM, Valid, PixelsPerCell);
            Sheet.Save(Path);
        }

        /// <summary>
        /// Writes the haul lengths. Cell lists and guidance points are not included.
        /// </summary>
        /// <param name="Path">JSON path to write.</param>
        /// <param name="Hauls">Hauls in schedule order.</param>
        public static void SaveSummary
            (
            String Path,
            IReadOnlyList<ResidualHaul> Hauls
            )
        {
            if (String.IsNullOrEmpty(Path)) throw new ArgumentNullException(nameof(Path));
            if (Hauls == null) throw new ArgumentNullException(nameof(Hauls));
            List<ResidualHaulSummary> Rows = new List<ResidualHaulSummary>(Hauls.Count);
            foreach (ResidualHaul Haul in Hauls)
            {
                Rows.Add(new ResidualHaulSummary(
                    Haul.TripNo,
                    Haul.PayloadBcy,
                    Haul.LoadedBcy,
                    Haul.HaulM,
                    Haul.DrivenM,
                    Haul.CutRuns,
                    Haul.FillRuns));
            }
            JsonSerializerOptions Options = new JsonSerializerOptions
            {
                WriteIndented = true,
            };
            File.WriteAllText(Path, JsonSerializer.Serialize(Rows, Options));
        }

        /// <summary>
        /// Bank yards of one piece that sit outside the grade tolerance.
        /// Pairing must schedule this, not the volume above the 50 mm cluster floor,
        /// or the haul stops while the cell is still outside the tolerance.
        /// </summary>
        /// <param name="Cells">Piece cells, row then column.</param>
        /// <param name="Sign">"cut" or "fill".</param>
        /// <param name="ResidualMm">Residual in millimetres. Positive is cut.</param>
        /// <param name="Grid">Field grid. The cell area comes from this.</param>
        /// <param name="FillToBank">Fill compaction. Cut uses one.</param>
        /// <param name="ToleranceMm">Grade tolerance in millimetres.</param>
        /// <returns>Bank cubic yards to move to bring the cells inside the tolerance.</returns>
        public static Double LevelVolumeBcy
            (
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells,
            String Sign,
            Double[,] ResidualMm,
            Grid Grid,
            Double FillToBank,
            Double ToleranceMm = Constants.DEFAULT_TOLERANCE_MM
            )
        {
            if (Cells == null) throw new ArgumentNullException(nameof(Cells));
            if (Sign == null) throw new ArgumentNullException(nameof(Sign));
            if (ResidualMm == null) throw new ArgumentNullException(nameof(ResidualMm));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (FillToBank <= 0.0) throw new ArgumentOutOfRangeException(nameof(FillToBank));
            if (!Double.IsFinite(ToleranceMm) || (ToleranceMm < 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(ToleranceMm));
            }
            Double Bank = Sign == "fill" ? FillToBank : 1.0;
            Int32 Rows = ResidualMm.GetLength(0);
            Int32 Cols = ResidualMm.GetLength(1);
            Double Sum = 0.0;
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                if ((Cell.Row < 0) || (Cell.Col < 0) || (Cell.Row >= Rows) || (Cell.Col >= Cols))
                {
                    continue;
                }
                if ((Cell.Row >= Grid.Valid.GetLength(0)) || (Cell.Col >= Grid.Valid.GetLength(1))
                    || !Grid.Valid[Cell.Row, Cell.Col])
                {
                    continue;
                }
                Double Signed = Sign == "fill" ? -ResidualMm[Cell.Row, Cell.Col] : ResidualMm[Cell.Row, Cell.Col];
                Double DepthM = Math.Max(Signed - ToleranceMm, 0.0) / 1000.0;
                if (DepthM <= 0.0)
                {
                    continue;
                }
                Sum += BankBcy(DepthM, Grid, Bank);
            }
            return Sum;
        }

        /// <summary>
        /// Positive haul yards still in one piece. Bridged cells and cells already
        /// listed on another piece add nothing. Depth inside the grade tolerance
        /// is left in the ground.
        /// </summary>
        /// <param name="Piece">The piece.</param>
        /// <param name="ResidualMm">Residual in millimetres.</param>
        /// <param name="Valid">True on field cells.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="FillToBank">Fill compaction. Cut uses one.</param>
        /// <param name="ToleranceMm">Grade tolerance in millimetres. Depth inside it is left.</param>
        /// <param name="Seen">Cells already given to a piece.</param>
        /// <returns>The stock, nearest-pick order is applied later.</returns>
        private static List<Stock> StockFor
            (
            ResidualPairPiece Piece,
            Double[,] ResidualMm,
            Boolean[,] Valid,
            Grid Grid,
            Double FillToBank,
            Double ToleranceMm,
            HashSet<(Int32 Bx, Int32 By)> Seen
            )
        {
            List<Stock> Stock = new List<Stock>();
            Int32 Rows = ResidualMm.GetLength(0);
            Int32 Cols = ResidualMm.GetLength(1);
            Double Bank = Piece.Sign == "fill" ? FillToBank : 1.0;
            foreach ((Int32 Row, Int32 Col) Cell in Piece.Cells)
            {
                if ((Cell.Row < 0) || (Cell.Col < 0) || (Cell.Row >= Rows) || (Cell.Col >= Cols))
                {
                    continue;
                }
                if ((Cell.Row >= Valid.GetLength(0)) || (Cell.Col >= Valid.GetLength(1)) || !Valid[Cell.Row, Cell.Col])
                {
                    continue;
                }
                (Int32 Bx, Int32 By) Key = (Cell.Col, Cell.Row);
                if (!Seen.Add(Key))
                {
                    continue;
                }
                // Fill residual is negative. The 50 mm floor only formed the cluster.
                // Keep taking this cell until it is inside the grade tolerance.
                Double Signed = Piece.Sign == "fill" ? -ResidualMm[Cell.Row, Cell.Col] : ResidualMm[Cell.Row, Cell.Col];
                Double DepthM = Math.Max(Signed - ToleranceMm, 0.0) / 1000.0;
                if (DepthM <= 0.0)
                {
                    continue;
                }
                Stock.Add(new Stock(Cell.Col, Cell.Row, DepthM, BankBcy(DepthM, Grid, Bank), Bank));
            }
            return Stock;
        }

        /// <summary>
        /// Takes each leg's yards from that piece, closest to the anchor first.
        /// Each returned pass is one lift, no deeper than the grid maximum.
        /// </summary>
        /// <param name="Legs">Cut loads or fill spreads.</param>
        /// <param name="Pools">Remaining stock by piece id.</param>
        /// <param name="Grid">Field grid. Its maximum depth caps one lift.</param>
        /// <param name="AnchorX">World easting the haul is aimed at.</param>
        /// <param name="AnchorY">World northing the haul is aimed at.</param>
        /// <returns>One list of cells per lift, in the order the lifts are driven.</returns>
        private static List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> TakeLegs
            (
            IReadOnlyList<ResidualPairLeg> Legs,
            Dictionary<String, List<Stock>> Pools,
            Grid Grid,
            Double AnchorX,
            Double AnchorY
            )
        {
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Passes =
                new List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>>();
            foreach (ResidualPairLeg Leg in Legs)
            {
                if (!Pools.TryGetValue(Leg.Id, out List<Stock>? Pool) || (Pool == null))
                {
                    continue;
                }
                Take(Pool, Leg.Bcy, Grid, AnchorX, AnchorY, Passes);
            }
            return Passes;
        }

        /// <summary>
        /// Moves lifts onto the haul until the leg's yards are covered.
        /// A nearer cell is taken before a farther one. One lift never exceeds
        /// the maximum depth, and the last lift is shortened to the yards still needed.
        /// </summary>
        /// <param name="Pool">Stock still available on this piece.</param>
        /// <param name="NeedBcy">Yards this leg still has to move.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="AnchorX">World easting to prefer.</param>
        /// <param name="AnchorY">World northing to prefer.</param>
        /// <param name="Passes">Lifts already chosen for this haul. One lift is appended at a time.</param>
        private static void Take
            (
            List<Stock> Pool,
            Double NeedBcy,
            Grid Grid,
            Double AnchorX,
            Double AnchorY,
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Passes
            )
        {
            // One pass may cut or spread only this deep. A deeper cell waits for the next lift.
            Double MaxDepthM = Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            Double Got = 0.0;
            while (Got < (NeedBcy - 1e-6))
            {
                List<Stock> Open = new List<Stock>();
                foreach (Stock Cell in Pool)
                {
                    if (!Cell.Used && (Cell.DepthM > 1e-9))
                    {
                        Open.Add(Cell);
                    }
                }
                if (Open.Count == 0)
                {
                    break;
                }
                Open.Sort((Stock Left, Stock Right) => CompareStock(Left, Right, Grid, AnchorX, AnchorY));
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Pass =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                foreach (Stock Cell in Open)
                {
                    if (Got >= (NeedBcy - 1e-6))
                    {
                        break;
                    }
                    Double PerMetre = BankBcy(1.0, Grid, Cell.Bank);
                    if (PerMetre <= 0.0)
                    {
                        break;
                    }
                    Double DepthM = Math.Min(Cell.DepthM, MaxDepthM);
                    Double Bcy = DepthM * PerMetre;
                    Double Room = NeedBcy - Got;
                    if (Bcy > Room)
                    {
                        // The bowl is full before this cell's lift is, so the cut stops early.
                        DepthM = Room / PerMetre;
                        Bcy = Room;
                    }
                    if (DepthM <= 1e-9)
                    {
                        break;
                    }
                    Cell.DepthM -= DepthM;
                    if (Cell.DepthM <= 1e-9)
                    {
                        Cell.DepthM = 0.0;
                        Cell.Used = true;
                    }
                    Cell.Bcy = Cell.DepthM * PerMetre;
                    Pass.Add(((Cell.Bx, Cell.By), DepthM));
                    Got += Bcy;
                }
                if (Pass.Count == 0)
                {
                    break;
                }
                Passes.Add(Pass);
            }
        }

        /// <summary>
        /// One list of every lift, in drive order.
        /// </summary>
        /// <param name="Passes">Lifts.</param>
        /// <returns>The cells, with the metres moved on that lift.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Flatten
            (
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Passes
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            foreach (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Pass in Passes)
            {
                Cells.AddRange(Pass);
            }
            return Cells;
        }

        /// <summary>
        /// Nearer stock first. Equal distances use the lower row, then the lower column.
        /// </summary>
        /// <param name="Left">First cell.</param>
        /// <param name="Right">Second cell.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="AnchorX">World easting.</param>
        /// <param name="AnchorY">World northing.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareStock
            (
            Stock Left,
            Stock Right,
            Grid Grid,
            Double AnchorX,
            Double AnchorY
            )
        {
            Double LeftD = DistanceTo(Grid, Left.Bx, Left.By, AnchorX, AnchorY);
            Double RightD = DistanceTo(Grid, Right.Bx, Right.By, AnchorX, AnchorY);
            Int32 ByDistance = LeftD.CompareTo(RightD);
            if (ByDistance != 0)
            {
                return ByDistance;
            }
            Int32 ByRow = Left.By.CompareTo(Right.By);
            if (ByRow != 0)
            {
                return ByRow;
            }
            return Left.Bx.CompareTo(Right.Bx);
        }

        /// <summary>
        /// World point of a trip's pieces, weighted by the yards on each leg.
        /// </summary>
        /// <param name="Legs">Loads or spreads.</param>
        /// <param name="ById">Pieces by id.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The easting and northing.</returns>
        private static (Double X, Double Y) Anchor
            (
            IReadOnlyList<ResidualPairLeg> Legs,
            Dictionary<String, ResidualPairPiece> ById,
            Grid Grid
            )
        {
            Double Weight = 0.0;
            Double Row = 0.0;
            Double Col = 0.0;
            foreach (ResidualPairLeg Leg in Legs)
            {
                if (!ById.TryGetValue(Leg.Id, out ResidualPairPiece? Piece) || (Piece == null))
                {
                    continue;
                }
                Weight += Leg.Bcy;
                Row += Leg.Bcy * Piece.CentroidRow;
                Col += Leg.Bcy * Piece.CentroidCol;
            }
            if (Weight <= 1e-9)
            {
                return (Grid.MinX, Grid.MinY);
            }
            return World(Grid, Col / Weight, Row / Weight);
        }

        /// <summary>
        /// Centre of the cells actually taken. The piece anchor is used when nothing was taken.
        /// </summary>
        /// <param name="Cells">Taken cells.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Fallback">Point used when the list is empty.</param>
        /// <returns>The easting and northing.</returns>
        private static (Double X, Double Y) CellsAnchor
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            (Double X, Double Y) Fallback
            )
        {
            if (Cells.Count == 0)
            {
                return Fallback;
            }
            Double X = 0.0;
            Double Y = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double Cx, Double Cy) = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                X += Cx;
                Y += Cy;
            }
            return (X / Cells.Count, Y / Cells.Count);
        }

        /// <summary>
        /// True when the plan's first point is nearer the target than its last point.
        /// The cut wants the opposite, so it can leave toward the fill.
        /// </summary>
        /// <param name="Plan">Coverage plan. An empty plan is not closer at the start.</param>
        /// <param name="TargetX">World easting.</param>
        /// <param name="TargetY">World northing.</param>
        /// <returns>True when the start faces the target.</returns>
        private static Boolean CloserStart
            (
            CoveragePlanner.PlanResult Plan,
            Double TargetX,
            Double TargetY
            )
        {
            if (Plan.Runs.Count == 0)
            {
                return false;
            }
            (Double StartX, Double StartY) = EndPoint(Plan, true);
            (Double EndX, Double EndY) = EndPoint(Plan, false);
            return Metres(StartX, StartY, TargetX, TargetY) < Metres(EndX, EndY, TargetX, TargetY);
        }

        /// <summary>
        /// The end of the plan nearer the target.
        /// </summary>
        /// <param name="Plan">Coverage plan.</param>
        /// <param name="TargetX">World easting.</param>
        /// <param name="TargetY">World northing.</param>
        /// <returns>The nearer point, or the target when the plan has no runs.</returns>
        private static (Double X, Double Y) NearEnd
            (
            CoveragePlanner.PlanResult Plan,
            Double TargetX,
            Double TargetY
            )
        {
            if (Plan.Runs.Count == 0)
            {
                return (TargetX, TargetY);
            }
            return EndPoint(Plan, CloserStart(Plan, TargetX, TargetY));
        }

        /// <summary>
        /// First or last point of a plan.
        /// </summary>
        /// <param name="Plan">Coverage plan. It has at least one run.</param>
        /// <param name="Start">True for the first point.</param>
        /// <returns>The point.</returns>
        private static (Double X, Double Y) EndPoint
            (
            CoveragePlanner.PlanResult Plan,
            Boolean Start
            )
        {
            CoveragePlanner.Run Run = Start ? Plan.Runs[0] : Plan.Runs[Plan.Runs.Count - 1];
            List<(Double X, Double Y)> Points = RunPoints(Run);
            return Start ? Points[0] : Points[Points.Count - 1];
        }

        /// <summary>
        /// Plans each lift and appends it. The blade lifts between lifts so two
        /// passes are not drawn as one cut deeper than the maximum.
        /// </summary>
        /// <param name="Points">Points already emitted.</param>
        /// <param name="Passes">Lifts, in drive order. Each cell's depth is one pass.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Near">World point the cut should leave toward, or the fill should start from.</param>
        /// <param name="Action">CUT or FILL.</param>
        /// <param name="ExitNear">True when the lift should finish at <paramref name="Near"/>.</param>
        /// <returns>How many blade-down strips were added, and their length including connectors.</returns>
        private static (Int32 Runs, Double WorkM) EmitPasses
            (
            List<GuidancePoint> Points,
            List<List<((Int32 Bx, Int32 By) Cell, Double DepthM)>> Passes,
            Grid Grid,
            (Double X, Double Y) Near,
            String Action,
            Boolean ExitNear
            )
        {
            Int32 Runs = 0;
            Double WorkM = 0.0;
            Boolean Started = false;
            foreach (List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Pass in Passes)
            {
                if (Pass.Count == 0)
                {
                    continue;
                }
                // Zero keeps a short piece. The cluster pieces are often under one blade length.
                CoveragePlanner.PlanResult Plan = ExitNear
                    ? CoveragePlanner.Plan(Pass, Grid, BLADE_M, null, Near, 0.0)
                    : CoveragePlanner.Plan(Pass, Grid, BLADE_M, Near, null, 0.0);
                Boolean Flip = ExitNear
                    ? CloserStart(Plan, Near.X, Near.Y)
                    : !CloserStart(Plan, Near.X, Near.Y);
                if (Started && (Plan.Runs.Count > 0))
                {
                    (Double EndX, Double EndY) = LastOf(Points, Action, Near);
                    Points.Add(MakePoint(Points.Count + 1, "MOVE", EndX, EndY));
                }
                Emit(Points, Plan, Action, Flip);
                Runs += Plan.Runs.Count;
                WorkM += WorkMetres(Plan);
                if (Plan.Runs.Count > 0)
                {
                    Started = true;
                }
            }
            return (Runs, WorkM);
        }

        /// <summary>
        /// Appends one side of the haul. A move point between strips keeps the blade path from bridging them.
        /// </summary>
        /// <param name="Points">Points already emitted.</param>
        /// <param name="Plan">Coverage plan.</param>
        /// <param name="Action">CUT or FILL.</param>
        /// <param name="Reverse">True when the far end of the plan should be driven first.</param>
        private static void Emit
            (
            List<GuidancePoint> Points,
            CoveragePlanner.PlanResult Plan,
            String Action,
            Boolean Reverse
            )
        {
            if (Plan.Runs.Count == 0)
            {
                return;
            }
            Int32 First = Reverse ? Plan.Runs.Count - 1 : 0;
            Int32 Step = Reverse ? -1 : 1;
            for (Int32 Index = First; (Index >= 0) && (Index < Plan.Runs.Count); Index += Step)
            {
                List<(Double X, Double Y)> Path = RunPoints(Plan.Runs[Index]);
                if (Reverse)
                {
                    Path.Reverse();
                }
                foreach ((Double X, Double Y) Point in Path)
                {
                    Points.Add(MakePoint(Points.Count + 1, Action, Point.X, Point.Y));
                }
                Boolean Last = Reverse ? (Index == 0) : (Index == (Plan.Runs.Count - 1));
                if (!Last)
                {
                    (Double X, Double Y) End = Path[Path.Count - 1];
                    Points.Add(MakePoint(Points.Count + 1, "MOVE", End.X, End.Y));
                }
            }
        }

        /// <summary>
        /// Blade points of one run. A run without a curve is the straight chord.
        /// </summary>
        /// <param name="Run">The run.</param>
        /// <returns>At least two points.</returns>
        private static List<(Double X, Double Y)> RunPoints
            (
            CoveragePlanner.Run Run
            )
        {
            if ((Run.DrivePoints != null) && (Run.DrivePoints.Count >= 2))
            {
                return new List<(Double X, Double Y)>(Run.DrivePoints);
            }
            List<(Double X, Double Y)> Chord = new List<(Double X, Double Y)>
            {
                Run.StartXy,
                Run.EndXy,
            };
            return Chord;
        }

        /// <summary>
        /// One guidance point. Latitude and the timing fields are unused on these pictures.
        /// </summary>
        /// <param name="Seq">1-based index.</param>
        /// <param name="Action">CUT, FILL, or MOVE.</param>
        /// <param name="X">World easting.</param>
        /// <param name="Y">World northing.</param>
        /// <returns>The point.</returns>
        private static GuidancePoint MakePoint
            (
            Int32 Seq,
            String Action,
            Double X,
            Double Y
            )
        {
            String Blade = (Action == "CUT") || (Action == "FILL") ? "DOWN" : "UP";
            return new GuidancePoint(Seq, 1, Action, Blade, X, Y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "");
        }

        /// <summary>
        /// Identity of the fill pieces on a trip. Several pieces on one trip are one fill area.
        /// </summary>
        /// <param name="Legs">Fill spreads on the trip.</param>
        /// <returns>Sorted piece ids.</returns>
        private static String SweepKey
            (
            IReadOnlyList<ResidualPairLeg> Legs
            )
        {
            List<String> Ids = new List<String>(Legs.Count);
            foreach (ResidualPairLeg Leg in Legs)
            {
                Ids.Add(Leg.Id);
            }
            Ids.Sort(StringComparer.Ordinal);
            return String.Join("|", Ids);
        }

        /// <summary>
        /// Cells of the legs' pieces, in piece order.
        /// </summary>
        /// <param name="Legs">Cut loads or fill spreads.</param>
        /// <param name="ById">Pieces by id.</param>
        /// <returns>The cells. Missing pieces are skipped.</returns>
        private static List<(Int32 Row, Int32 Col)> PieceCells
            (
            IReadOnlyList<ResidualPairLeg> Legs,
            Dictionary<String, ResidualPairPiece> ById
            )
        {
            List<(Int32 Row, Int32 Col)> Cells = new List<(Int32 Row, Int32 Col)>();
            foreach (ResidualPairLeg Leg in Legs)
            {
                if (!ById.TryGetValue(Leg.Id, out ResidualPairPiece? Piece) || (Piece == null))
                {
                    continue;
                }
                foreach ((Int32 Row, Int32 Col) Cell in Piece.Cells)
                {
                    Cells.Add(Cell);
                }
            }
            return Cells;
        }

        /// <summary>
        /// The trip's largest cut piece that still has dirt above target.
        /// The next piece is used only when that one is exhausted, so one trip stays on one piece.
        /// </summary>
        /// <param name="Ground">Working residual.</param>
        /// <param name="Trip">The scheduled trip.</param>
        /// <param name="ById">Pieces by id.</param>
        /// <returns>The piece, or null when no cut piece has anything above target.</returns>
        private static ResidualPairPiece? LargestCut
            (
            StripLanes.Ground Ground,
            ResidualPairTrip Trip,
            Dictionary<String, ResidualPairPiece> ById
            )
        {
            List<ResidualPairLeg> Order = new List<ResidualPairLeg>(Trip.Cuts);
            Order.Sort(LargerLeg);
            foreach (ResidualPairLeg Leg in Order)
            {
                if (!ById.TryGetValue(Leg.Id, out ResidualPairPiece? Piece) || (Piece == null))
                {
                    continue;
                }
                if (!StripLanes.HasCut(Ground, Piece.Cells))
                {
                    continue;
                }
                return Piece;
            }
            return null;
        }

        /// <summary>
        /// Puts the larger leg first so a top-up piece waits until the main piece is exhausted.
        /// </summary>
        /// <param name="Left">One leg.</param>
        /// <param name="Right">The other leg.</param>
        /// <returns>The sort order.</returns>
        private static Int32 LargerLeg
            (
            ResidualPairLeg Left,
            ResidualPairLeg Right
            )
        {
            return Right.Bcy.CompareTo(Left.Bcy);
        }

        /// <summary>
        /// Cell boxes for the picture outline. Column is X, row is Y.
        /// </summary>
        /// <param name="Cells">The piece.</param>
        /// <returns>The boxes.</returns>
        private static List<(Int32 Bx, Int32 By)> OutlineOf
            (
            IReadOnlyList<(Int32 Row, Int32 Col)> Cells
            )
        {
            List<(Int32 Bx, Int32 By)> Outline = new List<(Int32 Bx, Int32 By)>(Cells.Count);
            foreach ((Int32 Row, Int32 Col) Cell in Cells)
            {
                Outline.Add((Cell.Col, Cell.Row));
            }
            return Outline;
        }

        /// <summary>
        /// Last point of an action, or the fallback when that action was not emitted.
        /// </summary>
        /// <param name="Points">Emitted points.</param>
        /// <param name="Action">CUT or FILL.</param>
        /// <param name="Fallback">Point used when the action is missing.</param>
        /// <returns>The point.</returns>
        private static (Double X, Double Y) LastOf
            (
            List<GuidancePoint> Points,
            String Action,
            (Double X, Double Y) Fallback
            )
        {
            for (Int32 Index = Points.Count - 1; Index >= 0; Index -= 1)
            {
                if (Points[Index].Action == Action)
                {
                    return (Points[Index].X, Points[Index].Y);
                }
            }
            return Fallback;
        }

        /// <summary>
        /// First point of an action, or the fallback when that action was not emitted.
        /// </summary>
        /// <param name="Points">Emitted points.</param>
        /// <param name="Action">CUT or FILL.</param>
        /// <param name="Fallback">Point used when the action is missing.</param>
        /// <returns>The point.</returns>
        private static (Double X, Double Y) FirstOf
            (
            List<GuidancePoint> Points,
            String Action,
            (Double X, Double Y) Fallback
            )
        {
            foreach (GuidancePoint Point in Points)
            {
                if (Point.Action == Action)
                {
                    return (Point.X, Point.Y);
                }
            }
            return Fallback;
        }

        /// <summary>
        /// Blade-down length plus the straight connectors between strips.
        /// </summary>
        /// <param name="Plan">Coverage plan.</param>
        /// <returns>Metres.</returns>
        private static Double WorkMetres
            (
            CoveragePlanner.PlanResult Plan
            )
        {
            Double Sum = Plan.ConnectorDistanceM;
            foreach (CoveragePlanner.Run Run in Plan.Runs)
            {
                Sum += Run.LengthM;
            }
            return Sum;
        }

        /// <summary>
        /// Bank yards in a depth over one cell.
        /// </summary>
        /// <param name="DepthM">Metres to move on this cell.</param>
        /// <param name="Grid">Field grid. The cell is square.</param>
        /// <param name="Bank">One for cut. Fill compaction for fill.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double BankBcy
            (
            Double DepthM,
            Grid Grid,
            Double Bank
            )
        {
            Double Area = Grid.GridSizeM * Grid.GridSizeM;
            return DepthM * Area * ResidualClusters.M3_TO_CY * Bank;
        }

        /// <summary>
        /// World point of a fractional cell index.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Col">Column. Integers are cell centres when 0.5 is added by the caller.</param>
        /// <param name="Row">Row.</param>
        /// <returns>The easting and northing.</returns>
        private static (Double X, Double Y) World
            (
            Grid Grid,
            Double Col,
            Double Row
            )
        {
            Double X = Grid.MinX + ((Col + 0.5) * Grid.GridSizeM);
            Double Y = Grid.MinY + ((Row + 0.5) * Grid.GridSizeM);
            return (X, Y);
        }

        /// <summary>
        /// Metres from a cell centre to a world point.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="X">World easting.</param>
        /// <param name="Y">World northing.</param>
        /// <returns>The distance.</returns>
        private static Double DistanceTo
            (
            Grid Grid,
            Int32 Bx,
            Int32 By,
            Double X,
            Double Y
            )
        {
            (Double Cx, Double Cy) = Grid.CellCenterXy(Bx, By);
            return Metres(Cx, Cy, X, Y);
        }

        /// <summary>
        /// Straight metres between two world points.
        /// </summary>
        /// <param name="X1">First easting.</param>
        /// <param name="Y1">First northing.</param>
        /// <param name="X2">Second easting.</param>
        /// <param name="Y2">Second northing.</param>
        /// <returns>The distance.</returns>
        private static Double Metres
            (
            Double X1,
            Double Y1,
            Double X2,
            Double Y2
            )
        {
            Double Dx = X1 - X2;
            Double Dy = Y1 - Y2;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// One cell of a piece. Depth left above the floor shrinks as lifts are taken.
        /// </summary>
        private sealed class Stock
        {
            /// <summary>
            /// Creates a stock cell.
            /// </summary>
            /// <param name="Bx">Column.</param>
            /// <param name="By">Row.</param>
            /// <param name="DepthM">Metres still to move on this cell.</param>
            /// <param name="Bcy">Bank yards in that depth.</param>
            /// <param name="Bank">One for cut. Fill compaction for fill.</param>
            public Stock
                (
                Int32 Bx,
                Int32 By,
                Double DepthM,
                Double Bcy,
                Double Bank
                )
            {
                this.Bx = Bx;
                this.By = By;
                this.DepthM = DepthM;
                this.Bcy = Bcy;
                this.Bank = Bank;
            }

            /// <summary>Gets the column.</summary>
            public Int32 Bx { get; }

            /// <summary>Gets the row.</summary>
            public Int32 By { get; }

            /// <summary>Gets or sets the metres still above the floor.</summary>
            public Double DepthM { get; set; }

            /// <summary>Gets or sets the bank yards still above the floor.</summary>
            public Double Bcy { get; set; }

            /// <summary>Gets the bank factor. One for cut, compaction for fill.</summary>
            public Double Bank { get; }

            /// <summary>Gets or sets whether the cell has no depth left above the floor.</summary>
            public Boolean Used { get; set; }
        }
    }

    /// <summary>
    /// One loaded trip: the cells it moves, the blade path, and the haul length.
    /// </summary>
    public sealed class ResidualHaul
    {
        /// <summary>
        /// Creates a haul.
        /// </summary>
        /// <param name="TripNo">Schedule position, starting at 1.</param>
        /// <param name="PayloadBcy">Yards the trip asked for.</param>
        /// <param name="LoadedBcy">Yards in the cells that were taken.</param>
        /// <param name="HaulM">Loaded travel from the cut exit to the fill entry.</param>
        /// <param name="DrivenM">Load, connectors, haul, and spread. The empty return is not included.</param>
        /// <param name="CutRuns">Cut passes along the piece.</param>
        /// <param name="FillRuns">Fill passes along the piece.</param>
        /// <param name="CutCells">Cut cells and the metres taken from each.</param>
        /// <param name="FillCells">Fill cells and the metres placed on each.</param>
        /// <param name="Points">Haul arrow, from the cut exit to the fill entry. There is no cell path.</param>
        public ResidualHaul
            (
            Int32 TripNo,
            Double PayloadBcy,
            Double LoadedBcy,
            Double HaulM,
            Double DrivenM,
            Int32 CutRuns,
            Int32 FillRuns,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells,
            List<GuidancePoint> Points
            )
        {
            this.TripNo = TripNo;
            this.PayloadBcy = PayloadBcy;
            this.LoadedBcy = LoadedBcy;
            this.HaulM = HaulM;
            this.DrivenM = DrivenM;
            this.CutRuns = CutRuns;
            this.FillRuns = FillRuns;
            this.CutCells = CutCells;
            this.FillCells = FillCells;
            this.Points = Points;
        }

        /// <summary>Gets or sets the cut seconds, including a loop between passes.</summary>
        public Double LoadSeconds { get; set; }

        /// <summary>Gets or sets the spread seconds, including a loop between passes.</summary>
        public Double SpreadSeconds { get; set; }

        /// <summary>Gets or sets the blade-down cut distance in metres.</summary>
        public Double LoadDistanceM { get; set; }

        /// <summary>Gets or sets the blade-down fill distance in metres.</summary>
        public Double SpreadDistanceM { get; set; }

        /// <summary>Gets or sets the turns made while the cut was loaded. Area trips keep the loops in the seconds, so this stays zero.</summary>
        public Int32 LoadedTurns { get; set; }

        /// <summary>Gets or sets whether the exit, the entry, and the outlines are the area trip.</summary>
        public Boolean AreaBased { get; set; }

        /// <summary>Gets or sets the haul heading east.</summary>
        public Double DirX { get; set; }

        /// <summary>Gets or sets the haul heading north.</summary>
        public Double DirY { get; set; }

        /// <summary>Gets or sets the cut piece's upstream easting.</summary>
        public Double CutEntryX { get; set; }

        /// <summary>Gets or sets the cut piece's upstream northing.</summary>
        public Double CutEntryY { get; set; }

        /// <summary>Gets or sets the cut piece's downstream easting. The haul leaves from here.</summary>
        public Double CutExitX { get; set; }

        /// <summary>Gets or sets the cut piece's downstream northing.</summary>
        public Double CutExitY { get; set; }

        /// <summary>Gets or sets the fill piece's upstream easting. The haul arrives here.</summary>
        public Double FillEntryX { get; set; }

        /// <summary>Gets or sets the fill piece's upstream northing.</summary>
        public Double FillEntryY { get; set; }

        /// <summary>Gets or sets the fill piece's downstream easting.</summary>
        public Double FillExitX { get; set; }

        /// <summary>Gets or sets the fill piece's downstream northing.</summary>
        public Double FillExitY { get; set; }

        /// <summary>Gets or sets the cut piece outline. Null when the trip moved nothing.</summary>
        public List<(Int32 Bx, Int32 By)>? CutOutline { get; set; }

        /// <summary>Gets or sets the fill piece outline. Null when the trip moved nothing.</summary>
        public List<(Int32 Bx, Int32 By)>? FillOutline { get; set; }

        /// <summary>Gets or sets the cut piece id.</summary>
        public String CutPieceId { get; set; } = "";

        /// <summary>Gets or sets the fill piece ids.</summary>
        public String FillPieceId { get; set; } = "";

        /// <summary>Gets the schedule position, starting at 1.</summary>
        public Int32 TripNo { get; }

        /// <summary>Gets the yards the trip asked for.</summary>
        public Double PayloadBcy { get; }

        /// <summary>Gets the yards in the cells that were taken.</summary>
        public Double LoadedBcy { get; }

        /// <summary>Gets the loaded travel in metres.</summary>
        public Double HaulM { get; }

        /// <summary>Gets load, haul, and spread metres.</summary>
        public Double DrivenM { get; }

        /// <summary>Gets how many cut passes the time estimate used.</summary>
        public Int32 CutRuns { get; }

        /// <summary>Gets how many fill passes the time estimate used.</summary>
        public Int32 FillRuns { get; }

        /// <summary>Gets the cut cells.</summary>
        public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells { get; }

        /// <summary>Gets the fill cells.</summary>
        public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells { get; }

        /// <summary>Gets the blade path.</summary>
        public List<GuidancePoint> Points { get; }
    }

    /// <summary>
    /// Haul lengths without the cell lists.
    /// </summary>
    public sealed class ResidualHaulSummary
    {
        /// <summary>
        /// Creates a summary row.
        /// </summary>
        /// <param name="TripNo">Schedule position.</param>
        /// <param name="PayloadBcy">Yards the trip asked for.</param>
        /// <param name="LoadedBcy">Yards taken from the cells.</param>
        /// <param name="HaulM">Loaded travel in metres.</param>
        /// <param name="DrivenM">Load, haul, and spread metres.</param>
        /// <param name="CutRuns">Cut strips.</param>
        /// <param name="FillRuns">Fill strips.</param>
        public ResidualHaulSummary
            (
            Int32 TripNo,
            Double PayloadBcy,
            Double LoadedBcy,
            Double HaulM,
            Double DrivenM,
            Int32 CutRuns,
            Int32 FillRuns
            )
        {
            this.TripNo = TripNo;
            this.PayloadBcy = PayloadBcy;
            this.LoadedBcy = LoadedBcy;
            this.HaulM = HaulM;
            this.DrivenM = DrivenM;
            this.CutRuns = CutRuns;
            this.FillRuns = FillRuns;
        }

        /// <summary>Gets the schedule position.</summary>
        [JsonPropertyName("trip_no")]
        public Int32 TripNo { get; }

        /// <summary>Gets the yards the trip asked for.</summary>
        [JsonPropertyName("payload_bcy")]
        public Double PayloadBcy { get; }

        /// <summary>Gets the yards taken from the cells.</summary>
        [JsonPropertyName("loaded_bcy")]
        public Double LoadedBcy { get; }

        /// <summary>Gets the loaded travel in metres.</summary>
        [JsonPropertyName("haul_m")]
        public Double HaulM { get; }

        /// <summary>Gets load, haul, and spread metres.</summary>
        [JsonPropertyName("driven_m")]
        public Double DrivenM { get; }

        /// <summary>Gets the cut strips.</summary>
        [JsonPropertyName("cut_runs")]
        public Int32 CutRuns { get; }

        /// <summary>Gets the fill strips.</summary>
        [JsonPropertyName("fill_runs")]
        public Int32 FillRuns { get; }
    }
}
