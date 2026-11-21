using System.Globalization;
using System.Text.Json;
using System.Text.Json.Serialization;

using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace HaulBuilder
{
    /// <summary>
    /// Pairs cut pieces with fill pieces and schedules the bowl trips.
    /// It does not change clusters and it does not build strip or route geometry.
    /// The caller runs <see cref="Plan"/> again from re-clustered pieces after every
    /// <see cref="REPLAN_EVERY_TRIPS"/> executed trips. The previous schedule is not kept.
    /// </summary>
    static public class ResidualPairing
    {
        /// <summary>Loading speed in metres per second. The machine cut speed is 4 mph.</summary>
        public const Double V_LOAD_MS = Constants.DEFAULT_CUT_SPEED_MPH * Constants.MPH_TO_MPS;

        /// <summary>Hauling speed when full, in metres per second. The machine loaded speed is 10 mph.</summary>
        public const Double V_LOADED_MS = Constants.DEFAULT_LOADED_TRAVEL_SPEED_MPH * Constants.MPH_TO_MPS;

        /// <summary>Return speed when empty, in metres per second. The machine unloaded speed is 13 mph.</summary>
        public const Double V_EMPTY_MS = Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH * Constants.MPH_TO_MPS;

        /// <summary>Spreading speed in metres per second. The machine fill speed is 5 mph.</summary>
        public const Double V_SPREAD_MS = Constants.DEFAULT_FILL_SPEED_MPH * Constants.MPH_TO_MPS;

        /// <summary>
        /// Seconds for one 180 degree turn on the edge turning circle at the turn speed.
        /// The circle diameter is <see cref="Constants.DEFAULT_TURNING_CIRCLE_FT"/> feet
        /// and the turn speed is <see cref="Constants.DEFAULT_TURN_SPEED_MPH"/> mph.
        /// </summary>
        public static readonly Double T_TURN_S = TurnSeconds();

        /// <summary>Deepest fill placed in one spreading pass, in metres.</summary>
        public const Double SPREAD_LIFT_M = 0.10;

        /// <summary>Seconds per bank cubic yard charged for borrow that the field does not have.</summary>
        public const Double PENALTY_S_PER_BCY = 10000;

        /// <summary>Solver quantum. Volumes become integer multiples of this many bank cubic yards.</summary>
        public const Double FLOW_QUANTUM_BCY = 0.1;

        /// <summary>A trip still under this share of a bowl, after remainder packing, is left for correcting.</summary>
        public const Double PARTIAL_BOWL_FRACTION = 0.25;

        /// <summary>
        /// How many executed trips the caller hauls before it clusters the updated field and plans again.
        /// </summary>
        public const Int32 REPLAN_EVERY_TRIPS = 10;

        /// <summary>Fill id for cut volume the correcting phase will finish. Its cost is zero.</summary>
        public const String LOCAL_ID = "LOCAL";

        /// <summary>Cut id for fill volume the field cannot supply. Its cost is the penalty.</summary>
        public const String SHORT_ID = "SHORT";

        /// <summary>Yards in one cubic metre. The same factor the cluster volumes use.</summary>
        private const Double M3_TO_CY = ResidualClusters.M3_TO_CY;

        /// <summary>
        /// Pairs the pieces and schedules the trips. A later call replaces this schedule.
        /// </summary>
        /// <param name="Pieces">Cut and fill pieces from one clustering pass.</param>
        /// <param name="CellSizeM">Cell edge in metres. Centroid indexes are multiplied by this.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="PanWidthM">Pan width in metres.</param>
        /// <returns>The flows, the trips, and the diagnostic totals.</returns>
        public static ResidualPairPlan Plan
            (
            IReadOnlyList<ResidualPairPiece> Pieces,
            Double CellSizeM,
            Double BowlBcy,
            Double PanWidthM
            )
        {
            if (Pieces == null) throw new ArgumentNullException(nameof(Pieces));
            if ((CellSizeM <= 0.0) || (BowlBcy <= 0.0) || (PanWidthM <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(CellSizeM));
            }
            List<ResidualPairPiece> Cuts = new List<ResidualPairPiece>();
            List<ResidualPairPiece> Fills = new List<ResidualPairPiece>();
            HashSet<String> Seen = new HashSet<String>(StringComparer.Ordinal);
            foreach (ResidualPairPiece Piece in Pieces)
            {
                if (Piece == null) throw new ArgumentException("A piece is missing.", nameof(Pieces));
                if (!Seen.Add(Piece.Id))
                {
                    throw new ArgumentException("Piece ids must be unique.", nameof(Pieces));
                }
                if ((Piece.Id == LOCAL_ID) || (Piece.Id == SHORT_ID))
                {
                    throw new ArgumentException("LOCAL and SHORT are reserved for the balance nodes.", nameof(Pieces));
                }
                if (Piece.HaulVolumeBcy < 0.0)
                {
                    throw new ArgumentOutOfRangeException(nameof(Pieces));
                }
                if ((Piece.Sign != "cut") && (Piece.Sign != "fill"))
                {
                    throw new ArgumentException("A piece sign must be cut or fill.", nameof(Pieces));
                }
                if (Piece.Sign == "cut")
                {
                    Cuts.Add(Piece);
                }
                else
                {
                    Fills.Add(Piece);
                }
            }
            Double TotalCut = 0.0;
            Double TotalFill = 0.0;
            foreach (ResidualPairPiece Cut in Cuts)
            {
                TotalCut += Cut.HaulVolumeBcy;
            }
            foreach (ResidualPairPiece Fill in Fills)
            {
                TotalFill += Fill.HaulVolumeBcy;
            }
            // The dummy absorbs the imbalance. LOCAL is free cut left behind. SHORT is unpaid fill.
            String BalanceKind = "balanced";
            Double BalanceBcy = 0.0;
            if (TotalCut > (TotalFill + 1e-9))
            {
                BalanceKind = LOCAL_ID;
                BalanceBcy = TotalCut - TotalFill;
            }
            else if (TotalFill > (TotalCut + 1e-9))
            {
                BalanceKind = SHORT_ID;
                BalanceBcy = TotalFill - TotalCut;
            }
            List<String> CutIds = new List<String>();
            List<String> FillIds = new List<String>();
            List<Int32> CutTenths = new List<Int32>();
            List<Int32> FillTenths = new List<Int32>();
            Double[] LoadSeconds = new Double[Cuts.Count];
            Double[] SpreadSeconds = new Double[Fills.Count];
            Dictionary<String, ResidualPairPiece> PieceById = new Dictionary<String, ResidualPairPiece>(StringComparer.Ordinal);
            Dictionary<String, Double> LoadById = new Dictionary<String, Double>(StringComparer.Ordinal);
            Dictionary<String, Double> SpreadById = new Dictionary<String, Double>(StringComparer.Ordinal);
            for (Int32 Index = 0; Index < Cuts.Count; Index += 1)
            {
                CutIds.Add(Cuts[Index].Id);
                CutTenths.Add(Tenths(Cuts[Index].HaulVolumeBcy));
                LoadSeconds[Index] = WorkSeconds(Cuts[Index], PanWidthM, BowlBcy, V_LOAD_MS, false);
                PieceById[Cuts[Index].Id] = Cuts[Index];
                LoadById[Cuts[Index].Id] = LoadSeconds[Index];
            }
            for (Int32 Index = 0; Index < Fills.Count; Index += 1)
            {
                FillIds.Add(Fills[Index].Id);
                FillTenths.Add(Tenths(Fills[Index].HaulVolumeBcy));
                SpreadSeconds[Index] = WorkSeconds(Fills[Index], PanWidthM, BowlBcy, V_SPREAD_MS, true);
                PieceById[Fills[Index].Id] = Fills[Index];
                SpreadById[Fills[Index].Id] = SpreadSeconds[Index];
            }
            Int32 CutSum = 0;
            Int32 FillSum = 0;
            foreach (Int32 Volume in CutTenths)
            {
                CutSum += Volume;
            }
            foreach (Int32 Volume in FillTenths)
            {
                FillSum += Volume;
            }
            Int32 Imbalance = CutSum - FillSum;
            if ((BalanceKind == "balanced") && (Imbalance != 0))
            {
                // Rounding to 0.1 BCY must not invent a dummy when the pieces already match.
                NudgeTenths(Imbalance > 0 ? CutTenths : FillTenths, Math.Abs(Imbalance));
                Imbalance = 0;
            }
            if (Imbalance > 0)
            {
                FillIds.Add(LOCAL_ID);
                FillTenths.Add(Imbalance);
            }
            else if (Imbalance < 0)
            {
                CutIds.Add(SHORT_ID);
                CutTenths.Add(-Imbalance);
            }
            Int32 CutCount = CutIds.Count;
            Int32 FillCount = FillIds.Count;
            Double[,] Distance = new Double[CutCount, FillCount];
            Double[,] Cost = new Double[CutCount, FillCount];
            for (Int32 Cut = 0; Cut < CutCount; Cut += 1)
            {
                for (Int32 Fill = 0; Fill < FillCount; Fill += 1)
                {
                    Boolean Short = CutIds[Cut] == SHORT_ID;
                    Boolean Local = FillIds[Fill] == LOCAL_ID;
                    Double Metres = 0.0;
                    Double SecondsPerBcy = 0.0;
                    if (Short)
                    {
                        SecondsPerBcy = PENALTY_S_PER_BCY;
                    }
                    else if (!Local)
                    {
                        Metres = CentroidMetres(Cuts[Cut], Fills[Fill], CellSizeM);
                        Double Cycle = BowlCycle(LoadSeconds[Cut], SpreadSeconds[Fill], Metres);
                        SecondsPerBcy = Cycle / BowlBcy;
                    }
                    Distance[Cut, Fill] = Metres;
                    Cost[Cut, Fill] = SecondsPerBcy;
                }
            }
            Int32[,] FlowTenths = Solve(CutTenths, FillTenths, Cost);
            List<ResidualPairFlow> Flows = new List<ResidualPairFlow>();
            for (Int32 Cut = 0; Cut < CutCount; Cut += 1)
            {
                for (Int32 Fill = 0; Fill < FillCount; Fill += 1)
                {
                    Double Bcy = FlowTenths[Cut, Fill] * FLOW_QUANTUM_BCY;
                    if (Bcy < FLOW_QUANTUM_BCY)
                    {
                        continue;
                    }
                    Flows.Add(new ResidualPairFlow(
                        CutIds[Cut],
                        FillIds[Fill],
                        Bcy,
                        Distance[Cut, Fill],
                        Cost[Cut, Fill]));
                }
            }
            Flows.Sort(CompareFlows);
            (List<TripDraft> Drafts, List<ResidualPairRemainder> Remainders) = Schedule(
                Flows,
                PieceById,
                LoadById,
                SpreadById,
                CellSizeM,
                BowlBcy);
            List<ResidualPairTrip> Trips = new List<ResidualPairTrip>(Drafts.Count);
            Int32 Partials = 0;
            Double PayloadSum = 0.0;
            Double Seconds = 0.0;
            Double HaulMoment = 0.0;
            for (Int32 Index = 0; Index < Drafts.Count; Index += 1)
            {
                TripDraft Draft = Drafts[Index];
                if (Draft.Payload < (BowlBcy - 1e-6))
                {
                    Partials += 1;
                }
                PayloadSum += Draft.Payload;
                Seconds += Draft.Cycle;
                HaulMoment += Draft.Distance * Draft.Payload;
                Trips.Add(new ResidualPairTrip(
                    Index + 1,
                    Draft.Cuts,
                    Draft.Fills,
                    Draft.Payload,
                    Draft.Distance,
                    Draft.Cycle));
            }
            Double MeanPayload = Drafts.Count == 0 ? 0.0 : (PayloadSum / Drafts.Count);
            Double MeanHaul = PayloadSum <= 0.0 ? 0.0 : (HaulMoment / PayloadSum);
            return new ResidualPairPlan(
                Flows,
                Trips,
                Remainders,
                TotalCut,
                TotalFill,
                BalanceKind,
                BalanceBcy,
                Partials,
                MeanPayload,
                Seconds / 3600.0,
                MeanHaul);
        }

        /// <summary>
        /// Writes the scheduled trips as JSON. LOCAL and SHORT are not trips.
        /// </summary>
        /// <param name="Path">JSON path to write.</param>
        /// <param name="Trips">Trips in schedule order.</param>
        public static void SaveTrips
            (
            String Path,
            IReadOnlyList<ResidualPairTrip> Trips
            )
        {
            if (String.IsNullOrEmpty(Path)) throw new ArgumentNullException(nameof(Path));
            if (Trips == null) throw new ArgumentNullException(nameof(Trips));
            File.WriteAllText(Path, TripsJson(Trips));
        }

        /// <summary>
        /// Serializes the trips with a stable layout, so the same plan writes the same file.
        /// </summary>
        /// <param name="Trips">Trips in schedule order.</param>
        /// <returns>The JSON text.</returns>
        public static String TripsJson
            (
            IReadOnlyList<ResidualPairTrip> Trips
            )
        {
            if (Trips == null) throw new ArgumentNullException(nameof(Trips));
            JsonSerializerOptions Options = new JsonSerializerOptions
            {
                WriteIndented = true,
            };
            return JsonSerializer.Serialize(Trips, Options);
        }

        /// <summary>
        /// Draws one arrow per real flow on the residual. Width follows the bank yards.
        /// North is up. LOCAL and SHORT have no second centroid, so they are not drawn.
        /// </summary>
        /// <param name="Path">PNG path to write.</param>
        /// <param name="ResidualMm">Residual in millimetres, indexed [row, col].</param>
        /// <param name="Valid">True on field cells.</param>
        /// <param name="Plan">The pairing result.</param>
        /// <param name="Pieces">The same pieces that were paired. Centroids are read from here.</param>
        /// <param name="PixelsPerCell">Pixels along one cell.</param>
        public static void SavePng
            (
            String Path,
            Double[,] ResidualMm,
            Boolean[,] Valid,
            ResidualPairPlan Plan,
            IReadOnlyList<ResidualPairPiece> Pieces,
            Int32 PixelsPerCell
            )
        {
            if (String.IsNullOrEmpty(Path)) throw new ArgumentNullException(nameof(Path));
            if (ResidualMm == null) throw new ArgumentNullException(nameof(ResidualMm));
            if (Valid == null) throw new ArgumentNullException(nameof(Valid));
            if (Plan == null) throw new ArgumentNullException(nameof(Plan));
            if (Pieces == null) throw new ArgumentNullException(nameof(Pieces));
            if (PixelsPerCell < 1) throw new ArgumentOutOfRangeException(nameof(PixelsPerCell));
            Int32 Rows = ResidualMm.GetLength(0);
            using Image<Rgba32> Sheet = ResidualSheet(ResidualMm, Valid, PixelsPerCell);
            Dictionary<String, ResidualPairPiece> ById = new Dictionary<String, ResidualPairPiece>(StringComparer.Ordinal);
            foreach (ResidualPairPiece Piece in Pieces)
            {
                ById[Piece.Id] = Piece;
            }
            List<ResidualPairFlow> Drawn = new List<ResidualPairFlow>();
            Double MaxBcy = 0.0;
            foreach (ResidualPairFlow Flow in Plan.Flows)
            {
                if ((Flow.CutPiece == SHORT_ID) || (Flow.FillPiece == LOCAL_ID))
                {
                    continue;
                }
                if (!ById.ContainsKey(Flow.CutPiece) || !ById.ContainsKey(Flow.FillPiece))
                {
                    continue;
                }
                Drawn.Add(Flow);
                if (Flow.VolumeBcy > MaxBcy)
                {
                    MaxBcy = Flow.VolumeBcy;
                }
            }
            Rgba32 Black = new Rgba32(0, 0, 0, 255);
            Rgba32 Yellow = new Rgba32(255, 220, 0, 255);
            Rgba32 White = new Rgba32(255, 255, 255, 255);
            List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)> Plates =
                new List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)>();
            foreach (ResidualPairFlow Flow in Drawn)
            {
                ResidualPairPiece Cut = ById[Flow.CutPiece];
                ResidualPairPiece Fill = ById[Flow.FillPiece];
                Double X1 = (Cut.CentroidCol + 0.5) * PixelsPerCell;
                Double Y1 = (Rows - Cut.CentroidRow - 0.5) * PixelsPerCell;
                Double X2 = (Fill.CentroidCol + 0.5) * PixelsPerCell;
                Double Y2 = (Rows - Fill.CentroidRow - 0.5) * PixelsPerCell;
                Double Thickness = 2.0;
                if (MaxBcy > 0.0)
                {
                    Thickness = 2.0 + (8.0 * (Flow.VolumeBcy / MaxBcy));
                }
                DrawArrow(Sheet, X1, Y1, X2, Y2, Thickness, Black, Yellow, false);
                String Label = Flow.VolumeBcy.ToString("0.0", CultureInfo.InvariantCulture);
                Int32 Scale = 2;
                Int32 PlateW = (Label.Length * 6 * Scale) + 8;
                Int32 PlateH = (7 * Scale) + 8;
                Int32 PlateX = (Int32)Math.Round(((X1 + X2) * 0.5) - (PlateW * 0.5), MidpointRounding.AwayFromZero);
                Int32 PlateY = (Int32)Math.Round(((Y1 + Y2) * 0.5) - (PlateH * 0.5), MidpointRounding.AwayFromZero);
                (Int32 PlacedX, Int32 PlacedY) = PlacePlate(Plates, PlateX, PlateY, PlateW, PlateH, Sheet.Width, Sheet.Height);
                FillRect(Sheet, PlacedX, PlacedY, PlateW, PlateH, Black);
                DrawLabel(Sheet, PlacedX + 4, PlacedY + 4, Label, Scale, White);
                Plates.Add((PlacedX, PlacedY, PlateW, PlateH));
            }
            Sheet.Save(Path);
        }

        /// <summary>
        /// Draws one arrow per scheduled trip on the residual.
        /// The arrow runs from the volume-weighted cut centroid to the volume-weighted fill centroid.
        /// A trip at least 90 percent of a bowl is dark, from half a bowl up to that is medium, and under half a bowl is light.
        /// A trip that loads from more than one cut piece is dashed.
        /// </summary>
        /// <param name="Path">PNG path to write.</param>
        /// <param name="ResidualMm">Residual in millimetres, indexed [row, col].</param>
        /// <param name="Valid">True on field cells.</param>
        /// <param name="Trips">Scheduled trips. Each one becomes one arrow.</param>
        /// <param name="Pieces">The pieces that were paired. Centroids are read from here.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards. Colour is payload divided by this.</param>
        /// <param name="PixelsPerCell">Pixels along one cell.</param>
        public static void SaveTripPng
            (
            String Path,
            Double[,] ResidualMm,
            Boolean[,] Valid,
            IReadOnlyList<ResidualPairTrip> Trips,
            IReadOnlyList<ResidualPairPiece> Pieces,
            Double BowlBcy,
            Int32 PixelsPerCell
            )
        {
            if (String.IsNullOrEmpty(Path)) throw new ArgumentNullException(nameof(Path));
            if (ResidualMm == null) throw new ArgumentNullException(nameof(ResidualMm));
            if (Valid == null) throw new ArgumentNullException(nameof(Valid));
            if (Trips == null) throw new ArgumentNullException(nameof(Trips));
            if (Pieces == null) throw new ArgumentNullException(nameof(Pieces));
            if (BowlBcy <= 0.0) throw new ArgumentOutOfRangeException(nameof(BowlBcy));
            if (PixelsPerCell < 1) throw new ArgumentOutOfRangeException(nameof(PixelsPerCell));
            Int32 Rows = ResidualMm.GetLength(0);
            using Image<Rgba32> Sheet = ResidualSheet(ResidualMm, Valid, PixelsPerCell);
            Dictionary<String, ResidualPairPiece> ById = new Dictionary<String, ResidualPairPiece>(StringComparer.Ordinal);
            foreach (ResidualPairPiece Piece in Pieces)
            {
                ById[Piece.Id] = Piece;
            }
            Rgba32 Black = new Rgba32(0, 0, 0, 255);
            Rgba32 White = new Rgba32(255, 255, 255, 255);
            List<(Double X1, Double Y1, Double X2, Double Y2, String Label)> Labels =
                new List<(Double X1, Double Y1, Double X2, Double Y2, String Label)>();
            foreach (ResidualPairTrip Trip in Trips)
            {
                if (!LegCentroid(Trip.Cuts, ById, out Double CutRow, out Double CutCol))
                {
                    continue;
                }
                if (!LegCentroid(Trip.Fills, ById, out Double FillRow, out Double FillCol))
                {
                    continue;
                }
                Double X1 = (CutCol + 0.5) * PixelsPerCell;
                Double Y1 = (Rows - CutRow - 0.5) * PixelsPerCell;
                Double X2 = (FillCol + 0.5) * PixelsPerCell;
                Double Y2 = (Rows - FillRow - 0.5) * PixelsPerCell;
                Double Fraction = Trip.PayloadBcy / BowlBcy;
                Rgba32 Ink = TripInk(Fraction);
                // More than one cut piece shares the load, so the shaft is broken. The head stays solid.
                Boolean Shared = Trip.Cuts.Count >= 2;
                DrawArrow(Sheet, X1, Y1, X2, Y2, 4.0, Black, Ink, Shared);
                String Label = Trip.TripNo.ToString(CultureInfo.InvariantCulture)
                    + ": "
                    + Trip.PayloadBcy.ToString("0.0", CultureInfo.InvariantCulture);
                Labels.Add((X1, Y1, X2, Y2, Label));
            }
            // Labels go on after every shaft, so a later trip does not cover a trip number.
            List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)> Plates =
                new List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)>();
            foreach ((Double X1, Double Y1, Double X2, Double Y2, String Label) Mark in Labels)
            {
                Int32 Scale = 2;
                Int32 PlateW = (Mark.Label.Length * 6 * Scale) + 8;
                Int32 PlateH = (7 * Scale) + 8;
                Int32 PlateX = (Int32)Math.Round((((Mark.X1 + Mark.X2) * 0.5) - (PlateW * 0.5)), MidpointRounding.AwayFromZero);
                Int32 PlateY = (Int32)Math.Round((((Mark.Y1 + Mark.Y2) * 0.5) - (PlateH * 0.5)), MidpointRounding.AwayFromZero);
                (Int32 PlacedX, Int32 PlacedY) = PlacePlate(Plates, PlateX, PlateY, PlateW, PlateH, Sheet.Width, Sheet.Height);
                FillRect(Sheet, PlacedX, PlacedY, PlateW, PlateH, Black);
                DrawLabel(Sheet, PlacedX + 4, PlacedY + 4, Mark.Label, Scale, White);
                Plates.Add((PlacedX, PlacedY, PlateW, PlateH));
            }
            Sheet.Save(Path);
        }

        /// <summary>
        /// The residual as a north-up sheet, on the same colour scale as the cluster map.
        /// </summary>
        /// <param name="ResidualMm">Residual in millimetres, indexed [row, col].</param>
        /// <param name="Valid">True on field cells.</param>
        /// <param name="PixelsPerCell">Pixels along one cell.</param>
        /// <returns>The sheet. The caller disposes it.</returns>
        private static Image<Rgba32> ResidualSheet
            (
            Double[,] ResidualMm,
            Boolean[,] Valid,
            Int32 PixelsPerCell
            )
        {
            Int32 Rows = ResidualMm.GetLength(0);
            Int32 Cols = ResidualMm.GetLength(1);
            Double[,] Metres = new Double[Rows, Cols];
            for (Int32 Row = 0; Row < Rows; Row += 1)
            {
                for (Int32 Col = 0; Col < Cols; Col += 1)
                {
                    Metres[Row, Col] = Valid[Row, Col] ? (ResidualMm[Row, Col] / 1000.0) : Double.NaN;
                }
            }
            return Render.DiffToImageDigitize(Metres, Valid, PixelsPerCell);
        }

        /// <summary>
        /// Volume-weighted centroid of a trip's loads, in cell indexes.
        /// </summary>
        /// <param name="Legs">Cut loads or fill spreads.</param>
        /// <param name="ById">Pieces by id.</param>
        /// <param name="Row">Weighted row.</param>
        /// <param name="Col">Weighted column.</param>
        /// <returns>False when a piece is missing or the yards are zero.</returns>
        private static Boolean LegCentroid
            (
            IReadOnlyList<ResidualPairLeg> Legs,
            Dictionary<String, ResidualPairPiece> ById,
            out Double Row,
            out Double Col
            )
        {
            Double Weight = 0.0;
            Double RowSum = 0.0;
            Double ColSum = 0.0;
            foreach (ResidualPairLeg Leg in Legs)
            {
                if (!ById.TryGetValue(Leg.Id, out ResidualPairPiece? Piece) || (Piece == null))
                {
                    Row = 0.0;
                    Col = 0.0;
                    return false;
                }
                Weight += Leg.Bcy;
                RowSum += Leg.Bcy * Piece.CentroidRow;
                ColSum += Leg.Bcy * Piece.CentroidCol;
            }
            if (Weight <= 1e-9)
            {
                Row = 0.0;
                Col = 0.0;
                return false;
            }
            Row = RowSum / Weight;
            Col = ColSum / Weight;
            return true;
        }

        /// <summary>
        /// Shaft colour for one trip. Darker means a fuller bowl.
        /// </summary>
        /// <param name="Fraction">Payload divided by one bowl.</param>
        /// <returns>The ink.</returns>
        private static Rgba32 TripInk
            (
            Double Fraction
            )
        {
            // 90 percent of a bowl is the dark band. Half a bowl up to that line is medium.
            if (Fraction >= 0.9)
            {
                return new Rgba32(92, 48, 0, 255);
            }
            if (Fraction >= 0.5)
            {
                return new Rgba32(204, 128, 0, 255);
            }
            return new Rgba32(255, 214, 130, 255);
        }

        /// <summary>
        /// Seconds to load or spread one bowl. Spreading cannot place more than <see cref="SPREAD_LIFT_M"/> per pass.
        /// </summary>
        /// <param name="Piece">The piece being worked.</param>
        /// <param name="PanWidthM">Pan width in metres.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="SpeedMs">Travel speed while the pan is working.</param>
        /// <param name="CapDepth">True for spreading, where the lift is capped.</param>
        /// <returns>Seconds for one bowl.</returns>
        private static Double WorkSeconds
            (
            ResidualPairPiece Piece,
            Double PanWidthM,
            Double BowlBcy,
            Double SpeedMs,
            Boolean CapDepth
            )
        {
            if ((Piece.MeanMm <= 0.0) || (Piece.LongAxisLenM <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(Piece));
            }
            Double Depth = Piece.MeanMm / 1000.0;
            if (CapDepth && (Depth > SPREAD_LIFT_M))
            {
                Depth = SPREAD_LIFT_M;
            }
            Double BowlM3 = BowlBcy / M3_TO_CY;
            Double RunM = BowlM3 / (PanWidthM * Depth);
            Double Strips = Math.Ceiling(RunM / Piece.LongAxisLenM);
            if (Strips < 1.0)
            {
                Strips = 1.0;
            }
            return (RunM / SpeedMs) + ((Strips - 1.0) * T_TURN_S);
        }

        /// <summary>
        /// One full bowl: load, haul, turn, spread, return, turn.
        /// </summary>
        /// <param name="LoadSeconds">Loading time for one bowl.</param>
        /// <param name="SpreadSeconds">Spreading time for one bowl.</param>
        /// <param name="DistanceM">Straight-line haul in metres.</param>
        /// <returns>Seconds.</returns>
        private static Double BowlCycle
            (
            Double LoadSeconds,
            Double SpreadSeconds,
            Double DistanceM
            )
        {
            return LoadSeconds
                + (DistanceM / V_LOADED_MS)
                + T_TURN_S
                + SpreadSeconds
                + (DistanceM / V_EMPTY_MS)
                + T_TURN_S;
        }

        /// <summary>
        /// Straight-line metres between two centroids. Row and column are cell indexes.
        /// </summary>
        /// <param name="Cut">Cut piece.</param>
        /// <param name="Fill">Fill piece.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <returns>The distance.</returns>
        private static Double CentroidMetres
            (
            ResidualPairPiece Cut,
            ResidualPairPiece Fill,
            Double CellSizeM
            )
        {
            Double Row = Cut.CentroidRow - Fill.CentroidRow;
            Double Col = Cut.CentroidCol - Fill.CentroidCol;
            return Math.Sqrt((Row * Row) + (Col * Col)) * CellSizeM;
        }

        /// <summary>
        /// Rounds a volume to the solver quantum.
        /// </summary>
        /// <param name="Bcy">Bank cubic yards.</param>
        /// <returns>How many quanta, rounded to the nearest integer.</returns>
        private static Int32 Tenths
            (
            Double Bcy
            )
        {
            return (Int32)Math.Round(Bcy / FLOW_QUANTUM_BCY, MidpointRounding.AwayFromZero);
        }

        /// <summary>
        /// Removes a rounding imbalance from the largest piece so a balanced field stays balanced.
        /// </summary>
        /// <param name="Tenths">Piece volumes in quanta. The largest entry is changed.</param>
        /// <param name="Amount">Quanta to remove. The largest piece must be able to spare them.</param>
        private static void NudgeTenths
            (
            List<Int32> Tenths,
            Int32 Amount
            )
        {
            Int32 Largest = 0;
            for (Int32 Index = 1; Index < Tenths.Count; Index += 1)
            {
                if (Tenths[Index] > Tenths[Largest])
                {
                    Largest = Index;
                }
            }
            Tenths[Largest] -= Amount;
            if (Tenths[Largest] < 0)
            {
                Tenths[Largest] = 0;
            }
        }

        /// <summary>
        /// Minimum-cost transportation. Volumes are quanta. Costs are seconds per bank cubic yard.
        /// Reverse edges let a later path undo a cheap dummy that took the wrong cut.
        /// </summary>
        /// <param name="CutSupply">Cut quanta, including SHORT when the field is short of cut.</param>
        /// <param name="FillDemand">Fill quanta, including LOCAL when cut is left over.</param>
        /// <param name="Cost">Seconds per bank cubic yard, indexed [cut, fill].</param>
        /// <returns>Quanta sent on each pair.</returns>
        private static Int32[,] Solve
            (
            List<Int32> CutSupply,
            List<Int32> FillDemand,
            Double[,] Cost
            )
        {
            Int32 CutCount = CutSupply.Count;
            Int32 FillCount = FillDemand.Count;
            Int32[,] Flow = new Int32[CutCount, FillCount];
            if ((CutCount == 0) || (FillCount == 0))
            {
                return Flow;
            }
            Int32 Source = 0;
            Int32 Cut0 = 1;
            Int32 Fill0 = 1 + CutCount;
            Int32 Sink = 1 + CutCount + FillCount;
            Int32 NodeCount = Sink + 1;
            List<FlowEdge>[] Graph = new List<FlowEdge>[NodeCount];
            for (Int32 Node = 0; Node < NodeCount; Node += 1)
            {
                Graph[Node] = new List<FlowEdge>();
            }
            Int32 Total = 0;
            for (Int32 Cut = 0; Cut < CutCount; Cut += 1)
            {
                Total += CutSupply[Cut];
                AddEdge(Graph, Source, Cut0 + Cut, CutSupply[Cut], 0.0);
            }
            for (Int32 Fill = 0; Fill < FillCount; Fill += 1)
            {
                AddEdge(Graph, Fill0 + Fill, Sink, FillDemand[Fill], 0.0);
            }
            for (Int32 Cut = 0; Cut < CutCount; Cut += 1)
            {
                for (Int32 Fill = 0; Fill < FillCount; Fill += 1)
                {
                    AddEdge(Graph, Cut0 + Cut, Fill0 + Fill, Total, Cost[Cut, Fill]);
                }
            }
            Double[] Potential = new Double[NodeCount];
            Int32 Sent = 0;
            while (Sent < Total)
            {
                Int32 Pushed = Augment(Graph, Potential, Source, Sink);
                if (Pushed <= 0)
                {
                    throw new InvalidOperationException("The pairing could not place every cubic yard.");
                }
                Sent += Pushed;
            }
            for (Int32 Cut = 0; Cut < CutCount; Cut += 1)
            {
                foreach (FlowEdge Edge in Graph[Cut0 + Cut])
                {
                    if ((Edge.To < Fill0) || (Edge.To >= Sink))
                    {
                        continue;
                    }
                    Int32 Fill = Edge.To - Fill0;
                    Flow[Cut, Fill] = Graph[Edge.To][Edge.Rev].Cap;
                }
            }
            return Flow;
        }

        /// <summary>
        /// Adds a capacitated edge and its reverse. The reverse starts empty.
        /// </summary>
        /// <param name="Graph">Adjacency lists.</param>
        /// <param name="From">Tail node.</param>
        /// <param name="To">Head node.</param>
        /// <param name="Cap">Forward capacity in quanta.</param>
        /// <param name="Cost">Forward cost.</param>
        private static void AddEdge
            (
            List<FlowEdge>[] Graph,
            Int32 From,
            Int32 To,
            Int32 Cap,
            Double Cost
            )
        {
            Graph[From].Add(new FlowEdge(To, Graph[To].Count, Cap, Cost));
            Graph[To].Add(new FlowEdge(From, Graph[From].Count - 1, 0, -Cost));
        }

        /// <summary>
        /// Sends as much as possible along the cheapest residual path from the source to the sink.
        /// Potentials keep the reduced costs non-negative so the search can use Dijkstra.
        /// </summary>
        /// <param name="Graph">Residual graph. Capacities are updated.</param>
        /// <param name="Potential">Node potentials. Updated for the nodes the search reached.</param>
        /// <param name="Source">Source node.</param>
        /// <param name="Sink">Sink node.</param>
        /// <returns>Quanta sent. Zero when the sink cannot be reached.</returns>
        private static Int32 Augment
            (
            List<FlowEdge>[] Graph,
            Double[] Potential,
            Int32 Source,
            Int32 Sink
            )
        {
            Int32 NodeCount = Graph.Length;
            Double[] Distance = new Double[NodeCount];
            Boolean[] Used = new Boolean[NodeCount];
            Int32[] Previous = new Int32[NodeCount];
            Int32[] PreviousEdge = new Int32[NodeCount];
            for (Int32 Node = 0; Node < NodeCount; Node += 1)
            {
                Distance[Node] = Double.PositiveInfinity;
                Previous[Node] = -1;
                PreviousEdge[Node] = -1;
            }
            Distance[Source] = 0.0;
            for (Int32 Step = 0; Step < NodeCount; Step += 1)
            {
                Int32 Best = -1;
                Double BestDistance = Double.PositiveInfinity;
                for (Int32 Node = 0; Node < NodeCount; Node += 1)
                {
                    if (!Used[Node] && (Distance[Node] < BestDistance))
                    {
                        BestDistance = Distance[Node];
                        Best = Node;
                    }
                }
                if ((Best < 0) || Double.IsPositiveInfinity(Distance[Best]))
                {
                    break;
                }
                Used[Best] = true;
                List<FlowEdge> Edges = Graph[Best];
                for (Int32 Index = 0; Index < Edges.Count; Index += 1)
                {
                    FlowEdge Edge = Edges[Index];
                    if (Edge.Cap <= 0)
                    {
                        continue;
                    }
                    Double Reduced = Edge.Cost + Potential[Best] - Potential[Edge.To];
                    if ((Reduced < 0.0) && (Reduced > -1e-8))
                    {
                        Reduced = 0.0;
                    }
                    Double Next = Distance[Best] + Reduced;
                    if (Next < Distance[Edge.To])
                    {
                        Distance[Edge.To] = Next;
                        Previous[Edge.To] = Best;
                        PreviousEdge[Edge.To] = Index;
                    }
                }
            }
            if (Double.IsPositiveInfinity(Distance[Sink]))
            {
                return 0;
            }
            for (Int32 Node = 0; Node < NodeCount; Node += 1)
            {
                if (!Double.IsPositiveInfinity(Distance[Node]))
                {
                    Potential[Node] += Distance[Node];
                }
            }
            Int32 Push = Int32.MaxValue;
            Int32 At = Sink;
            while (At != Source)
            {
                FlowEdge Edge = Graph[Previous[At]][PreviousEdge[At]];
                if (Edge.Cap < Push)
                {
                    Push = Edge.Cap;
                }
                At = Previous[At];
            }
            At = Sink;
            while (At != Source)
            {
                Int32 From = Previous[At];
                FlowEdge Edge = Graph[From][PreviousEdge[At]];
                Edge.Cap -= Push;
                Graph[Edge.To][Edge.Rev].Cap += Push;
                At = From;
            }
            return Push;
        }

        /// <summary>
        /// Turns flows into trips. Fill pieces of one cluster are spread nearest-first.
        /// A partial trip is topped up from edge-adjacent remainders before a short trip is rejected.
        /// </summary>
        /// <param name="Flows">Solved flows, including LOCAL and SHORT.</param>
        /// <param name="PieceById">Pieces by id.</param>
        /// <param name="LoadById">Seconds to load one bowl from a cut piece.</param>
        /// <param name="SpreadById">Seconds to spread one bowl on a fill piece.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <returns>The kept trips, longest haul first, and the volumes sent to correcting.</returns>
        private static (List<TripDraft> Trips, List<ResidualPairRemainder> Correcting) Schedule
            (
            List<ResidualPairFlow> Flows,
            Dictionary<String, ResidualPairPiece> PieceById,
            Dictionary<String, Double> LoadById,
            Dictionary<String, Double> SpreadById,
            Double CellSizeM,
            Double BowlBcy
            )
        {
            Dictionary<(String Cut, String FillCluster), List<FillSlice>> Groups =
                new Dictionary<(String Cut, String FillCluster), List<FillSlice>>();
            foreach (ResidualPairFlow Flow in Flows)
            {
                if ((Flow.CutPiece == SHORT_ID) || (Flow.FillPiece == LOCAL_ID))
                {
                    continue;
                }
                (String Cut, String FillCluster) Key = (Flow.CutPiece, ClusterId(Flow.FillPiece));
                if (!Groups.TryGetValue(Key, out List<FillSlice>? Slices))
                {
                    Slices = new List<FillSlice>();
                    Groups[Key] = Slices;
                }
                ResidualPairPiece Fill = PieceById[Flow.FillPiece];
                Slices.Add(new FillSlice(Flow.FillPiece, Flow.VolumeBcy, Flow.DistanceM, Fill.CentroidRow, Fill.CentroidCol));
            }
            List<(String Cut, String FillCluster)> Keys = new List<(String Cut, String FillCluster)>(Groups.Keys);
            Keys.Sort(CompareGroupKeys);
            List<CutRemainder> Remainders = new List<CutRemainder>();
            List<TripDraft> Drafts = new List<TripDraft>();
            Int32 Sequence = 0;
            foreach ((String Cut, String FillCluster) Key in Keys)
            {
                List<FillSlice> Slices = Groups[Key];
                // Nearest fill is spread first, so a bowl finishes the close piece before the far one.
                Slices.Sort(CompareSlices);
                ResidualPairPiece CutPiece = PieceById[Key.Cut];
                Double Volume = 0.0;
                foreach (FillSlice Slice in Slices)
                {
                    Volume += Slice.Bcy;
                }
                Int32 FullTrips = (Int32)Math.Floor((Volume / BowlBcy) + 1e-9);
                for (Int32 Trip = 0; Trip < FullTrips; Trip += 1)
                {
                    List<FillSlice> Taken = Take(Slices, BowlBcy);
                    Drafts.Add(MakeTrip(
                        new[] { new FillSlice(CutPiece.Id, SumBcy(Taken), 0.0, CutPiece.CentroidRow, CutPiece.CentroidCol) },
                        Taken,
                        LoadById,
                        SpreadById,
                        CellSizeM,
                        BowlBcy,
                        Sequence));
                    Sequence += 1;
                }
                List<FillSlice> Left = new List<FillSlice>();
                foreach (FillSlice Slice in Slices)
                {
                    if (Slice.Bcy > 1e-6)
                    {
                        Left.Add(Slice);
                    }
                }
                Double LeftBcy = SumBcy(Left);
                if (LeftBcy > 1e-6)
                {
                    Remainders.Add(new CutRemainder(CutPiece, Left, LeftBcy, Key.FillCluster));
                }
            }
            // Each remainder starts as its own partial trip. Top-up may move yards onto an earlier trip.
            List<OpenLoad> Loads = new List<OpenLoad>(Remainders.Count);
            foreach (CutRemainder Remainder in Remainders)
            {
                List<FillSlice> Cuts = new List<FillSlice>
                {
                    new FillSlice(Remainder.Cut.Id, Remainder.Bcy, 0.0, Remainder.Cut.CentroidRow, Remainder.Cut.CentroidCol),
                };
                Loads.Add(new OpenLoad(Cuts, Remainder.Fills, Remainder.FillCluster, Sequence));
                Sequence += 1;
            }
            Dictionary<String, HashSet<(Int32 Row, Int32 Col)>> CellsById = CellSets(PieceById);
            List<TripDraft> PartialKeys = new List<TripDraft>(Loads.Count);
            foreach (OpenLoad Load in Loads)
            {
                PartialKeys.Add(MakeTrip(Load.Cuts, Load.Fills, LoadById, SpreadById, CellSizeM, BowlBcy, Load.Sequence));
            }
            List<Int32> Order = new List<Int32>(Loads.Count);
            for (Int32 Index = 0; Index < Loads.Count; Index += 1)
            {
                Order.Add(Index);
            }
            // Schedule order is the haul order: longest distance first, then the usual ties.
            Order.Sort((Int32 Left, Int32 Right) => CompareDrafts(PartialKeys[Left], PartialKeys[Right]));
            foreach (Int32 Index in Order)
            {
                OpenLoad Trip = Loads[Index];
                if (Trip.Payload <= 1e-6)
                {
                    continue;
                }
                // A full bowl is finished. Three cut pieces is the limit.
                while ((Trip.Cuts.Count < 3) && (Trip.Payload < (BowlBcy - 1e-9)))
                {
                    OpenLoad? Donor = LargestDonor(Trip, Loads, CellsById);
                    if (Donor == null)
                    {
                        break;
                    }
                    Double Room = BowlBcy - Trip.Payload;
                    Double Amount = Math.Min(Donor.Payload, Room);
                    // A donor smaller than the room is taken whole, including the last fraction of a yard.
                    if ((Donor.Payload - Amount) <= 1e-6)
                    {
                        Amount = Donor.Payload;
                    }
                    List<FillSlice> Moved = Take(Donor.Fills, Amount);
                    Double MovedBcy = SumBcy(Moved);
                    if (MovedBcy <= 1e-9)
                    {
                        break;
                    }
                    Donor.Cuts[0].Bcy = Math.Max(0.0, Donor.Cuts[0].Bcy - MovedBcy);
                    FillSlice DonorCut = Donor.Cuts[0];
                    Trip.Cuts.Add(new FillSlice(DonorCut.Id, MovedBcy, 0.0, DonorCut.Row, DonorCut.Col));
                    Trip.Fills = MergeFills(Trip.Fills, Moved, Trip.Cuts);
                }
            }
            foreach (OpenLoad Load in Loads)
            {
                if (Load.Payload <= 1e-6)
                {
                    continue;
                }
                List<FillSlice> Cuts = Positive(Load.Cuts);
                List<FillSlice> Fills = Positive(Load.Fills);
                if ((Cuts.Count == 0) || (Fills.Count == 0))
                {
                    continue;
                }
                Drafts.Add(MakeTrip(Cuts, Fills, LoadById, SpreadById, CellSizeM, BowlBcy, Load.Sequence));
            }
            Double Quarter = PARTIAL_BOWL_FRACTION * BowlBcy;
            List<TripDraft> Kept = new List<TripDraft>();
            List<ResidualPairRemainder> Correcting = new List<ResidualPairRemainder>();
            foreach (TripDraft Draft in Drafts)
            {
                // Top-up happens first. Only a trip that is still under a quarter bowl is left behind.
                if (Draft.Payload < (Quarter - 1e-9))
                {
                    Correcting.Add(new ResidualPairRemainder(Draft.TieCut, Draft.TieFill, Draft.Payload));
                    continue;
                }
                Kept.Add(Draft);
            }
            Kept.Sort(CompareDrafts);
            return (Kept, Correcting);
        }

        /// <summary>
        /// Takes volume from fill slices in their current order.
        /// </summary>
        /// <param name="Slices">Slices nearest-first. Their remaining volume is reduced.</param>
        /// <param name="Amount">Bank cubic yards to take.</param>
        /// <returns>The slices that supplied the amount.</returns>
        private static List<FillSlice> Take
            (
            List<FillSlice> Slices,
            Double Amount
            )
        {
            List<FillSlice> Taken = new List<FillSlice>();
            Double Left = Amount;
            foreach (FillSlice Slice in Slices)
            {
                if (Left <= 1e-9)
                {
                    break;
                }
                if (Slice.Bcy <= 1e-9)
                {
                    continue;
                }
                Double Use = Math.Min(Slice.Bcy, Left);
                Taken.Add(new FillSlice(Slice.Id, Use, Slice.Distance, Slice.Row, Slice.Col));
                Slice.Bcy -= Use;
                Left -= Use;
            }
            return Taken;
        }

        /// <summary>
        /// Adds the second remainder's fill onto the first, then orders fills from the cuts' centroid.
        /// </summary>
        /// <param name="First">Fill slices of the first cut piece.</param>
        /// <param name="Second">Fill slices of the neighbouring cut piece.</param>
        /// <param name="Cuts">Both cut loads, used for the shared centroid.</param>
        /// <returns>The combined fill slices, nearest first.</returns>
        private static List<FillSlice> MergeFills
            (
            List<FillSlice> First,
            List<FillSlice> Second,
            List<FillSlice> Cuts
            )
        {
            Dictionary<String, FillSlice> ById = new Dictionary<String, FillSlice>(StringComparer.Ordinal);
            foreach (FillSlice Slice in First)
            {
                ById[Slice.Id] = new FillSlice(Slice.Id, Slice.Bcy, Slice.Distance, Slice.Row, Slice.Col);
            }
            foreach (FillSlice Slice in Second)
            {
                if (ById.TryGetValue(Slice.Id, out FillSlice? Have) && (Have != null))
                {
                    Have.Bcy += Slice.Bcy;
                }
                else
                {
                    ById[Slice.Id] = new FillSlice(Slice.Id, Slice.Bcy, Slice.Distance, Slice.Row, Slice.Col);
                }
            }
            (Double Row, Double Col) = Weighted(Cuts);
            List<FillSlice> Merged = new List<FillSlice>(ById.Values);
            Merged.Sort((FillSlice Left, FillSlice Right) => CompareByCentroid(Left, Right, Row, Col));
            return Merged;
        }

        /// <summary>
        /// Builds one trip from cut loads and fill spreads.
        /// The haul is the distance between the two volume-weighted centroids.
        /// </summary>
        /// <param name="Cuts">Cut pieces and the yards loaded from each.</param>
        /// <param name="Fills">Fill pieces and the yards spread on each.</param>
        /// <param name="LoadById">Seconds to load one bowl from a cut piece.</param>
        /// <param name="SpreadById">Seconds to spread one bowl on a fill piece.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <param name="BowlBcy">Pan capacity in bank cubic yards.</param>
        /// <param name="Sequence">Creation order, used when every sort key ties.</param>
        /// <returns>The draft trip.</returns>
        private static TripDraft MakeTrip
            (
            IReadOnlyList<FillSlice> Cuts,
            IReadOnlyList<FillSlice> Fills,
            Dictionary<String, Double> LoadById,
            Dictionary<String, Double> SpreadById,
            Double CellSizeM,
            Double BowlBcy,
            Int32 Sequence
            )
        {
            Double Payload = SumBcy(Cuts);
            (Double CutRow, Double CutCol) = Weighted(Cuts);
            (Double FillRow, Double FillCol) = Weighted(Fills);
            Double Distance = IndexMetres(CutRow, CutCol, FillRow, FillCol, CellSizeM);
            Double Load = 0.0;
            foreach (FillSlice Cut in Cuts)
            {
                Load += (Cut.Bcy / BowlBcy) * LoadById[Cut.Id];
            }
            Double Spread = 0.0;
            foreach (FillSlice Fill in Fills)
            {
                Spread += (Fill.Bcy / BowlBcy) * SpreadById[Fill.Id];
            }
            Double Cycle = Load
                + (Distance / V_LOADED_MS)
                + T_TURN_S
                + Spread
                + (Distance / V_EMPTY_MS)
                + T_TURN_S;
            return new TripDraft(Cuts, Fills, Payload, Distance, Cycle, Sequence);
        }

        /// <summary>
        /// Volume-weighted centroid of a set of loads.
        /// </summary>
        /// <param name="Slices">Loads. Their yards must be positive.</param>
        /// <returns>The row and column.</returns>
        private static (Double Row, Double Col) Weighted
            (
            IReadOnlyList<FillSlice> Slices
            )
        {
            Double Weight = 0.0;
            Double Row = 0.0;
            Double Col = 0.0;
            foreach (FillSlice Slice in Slices)
            {
                Weight += Slice.Bcy;
                Row += Slice.Bcy * Slice.Row;
                Col += Slice.Bcy * Slice.Col;
            }
            return (Row / Weight, Col / Weight);
        }

        /// <summary>
        /// Sums the yards on a set of loads.
        /// </summary>
        /// <param name="Slices">Loads.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double SumBcy
            (
            IReadOnlyList<FillSlice> Slices
            )
        {
            Double Sum = 0.0;
            foreach (FillSlice Slice in Slices)
            {
                Sum += Slice.Bcy;
            }
            return Sum;
        }

        /// <summary>
        /// Straight-line metres between two cell-index points.
        /// </summary>
        /// <param name="Row1">First row.</param>
        /// <param name="Col1">First column.</param>
        /// <param name="Row2">Second row.</param>
        /// <param name="Col2">Second column.</param>
        /// <param name="CellSizeM">Cell edge in metres.</param>
        /// <returns>The distance.</returns>
        private static Double IndexMetres
            (
            Double Row1,
            Double Col1,
            Double Row2,
            Double Col2,
            Double CellSizeM
            )
        {
            Double Row = Row1 - Row2;
            Double Col = Col1 - Col2;
            return Math.Sqrt((Row * Row) + (Col * Col)) * CellSizeM;
        }

        /// <summary>
        /// Parent cluster of a piece id. F1.3 belongs to F1. A piece with no dot is its own cluster.
        /// </summary>
        /// <param name="Id">Piece id.</param>
        /// <returns>The cluster id.</returns>
        private static String ClusterId
            (
            String Id
            )
        {
            Int32 Dot = Id.IndexOf('.');
            if (Dot < 0)
            {
                return Id;
            }
            return Id.Substring(0, Dot);
        }

        /// <summary>
        /// Orders group keys by cut id, then fill cluster.
        /// </summary>
        /// <param name="Left">First key.</param>
        /// <param name="Right">Second key.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareGroupKeys
            (
            (String Cut, String FillCluster) Left,
            (String Cut, String FillCluster) Right
            )
        {
            Int32 ByCut = String.Compare(Left.Cut, Right.Cut, StringComparison.Ordinal);
            if (ByCut != 0)
            {
                return ByCut;
            }
            return String.Compare(Left.FillCluster, Right.FillCluster, StringComparison.Ordinal);
        }

        /// <summary>
        /// Nearest fill first. Equal distances use the piece id.
        /// </summary>
        /// <param name="Left">First slice.</param>
        /// <param name="Right">Second slice.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareSlices
            (
            FillSlice Left,
            FillSlice Right
            )
        {
            Int32 ByDistance = Left.Distance.CompareTo(Right.Distance);
            if (ByDistance != 0)
            {
                return ByDistance;
            }
            return String.Compare(Left.Id, Right.Id, StringComparison.Ordinal);
        }

        /// <summary>
        /// The largest remainder that can still be loaded onto a partial trip.
        /// It must share the fill cluster and the cut cluster, and share an 8-connected edge
        /// with a cut piece already on the trip.
        /// </summary>
        /// <param name="Trip">The partial trip being filled.</param>
        /// <param name="Loads">Every remainder, including ones already moved.</param>
        /// <param name="CellsById">Cells of each piece.</param>
        /// <returns>The donor, or null when nothing adjacent is left.</returns>
        private static OpenLoad? LargestDonor
            (
            OpenLoad Trip,
            List<OpenLoad> Loads,
            Dictionary<String, HashSet<(Int32 Row, Int32 Col)>> CellsById
            )
        {
            String CutCluster = ClusterId(Trip.Cuts[0].Id);
            OpenLoad? Best = null;
            foreach (OpenLoad Candidate in Loads)
            {
                if (Object.ReferenceEquals(Candidate, Trip))
                {
                    continue;
                }
                // A topped-up trip is no longer a remainder. A full bowl is not either.
                if ((Candidate.Payload <= 1e-6) || (Candidate.Cuts.Count != 1))
                {
                    continue;
                }
                if (Candidate.FillCluster != Trip.FillCluster)
                {
                    continue;
                }
                String DonorId = Candidate.Cuts[0].Id;
                if (ClusterId(DonorId) != CutCluster)
                {
                    continue;
                }
                if (OnTrip(Trip, DonorId))
                {
                    continue;
                }
                if (!SharesEdge(Trip, DonorId, CellsById))
                {
                    continue;
                }
                if (Best == null)
                {
                    Best = Candidate;
                    continue;
                }
                Int32 ByVolume = Candidate.Payload.CompareTo(Best.Payload);
                if (ByVolume > 0)
                {
                    Best = Candidate;
                    continue;
                }
                // Equal yards keep the lower piece id, so the choice does not depend on list order.
                if ((ByVolume == 0) && (String.Compare(DonorId, Best.Cuts[0].Id, StringComparison.Ordinal) < 0))
                {
                    Best = Candidate;
                }
            }
            return Best;
        }

        /// <summary>
        /// True when one cut piece on the trip touches the donor, including a corner.
        /// </summary>
        /// <param name="Trip">The trip being filled.</param>
        /// <param name="DonorId">The remainder's cut piece.</param>
        /// <param name="CellsById">Cells of each piece.</param>
        /// <returns>True when the pieces are 8-connected.</returns>
        private static Boolean SharesEdge
            (
            OpenLoad Trip,
            String DonorId,
            Dictionary<String, HashSet<(Int32 Row, Int32 Col)>> CellsById
            )
        {
            if (!CellsById.TryGetValue(DonorId, out HashSet<(Int32 Row, Int32 Col)>? DonorCells) || (DonorCells == null))
            {
                return false;
            }
            foreach (FillSlice Cut in Trip.Cuts)
            {
                if (!CellsById.TryGetValue(Cut.Id, out HashSet<(Int32 Row, Int32 Col)>? Have) || (Have == null))
                {
                    continue;
                }
                foreach ((Int32 Row, Int32 Col) Cell in Have)
                {
                    for (Int32 RowStep = -1; RowStep <= 1; RowStep += 1)
                    {
                        for (Int32 ColStep = -1; ColStep <= 1; ColStep += 1)
                        {
                            if ((RowStep == 0) && (ColStep == 0))
                            {
                                continue;
                            }
                            if (DonorCells.Contains((Cell.Row + RowStep, Cell.Col + ColStep)))
                            {
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        /// <summary>
        /// True when the trip already loads from this cut piece.
        /// </summary>
        /// <param name="Trip">The trip being filled.</param>
        /// <param name="Id">Cut piece id.</param>
        /// <returns>True when the id is already a cut leg.</returns>
        private static Boolean OnTrip
            (
            OpenLoad Trip,
            String Id
            )
        {
            foreach (FillSlice Cut in Trip.Cuts)
            {
                if (Cut.Id == Id)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Cells of every piece that has a grid, keyed by piece id.
        /// </summary>
        /// <param name="PieceById">Pieces by id.</param>
        /// <returns>The sets. Pieces with no cells are absent.</returns>
        private static Dictionary<String, HashSet<(Int32 Row, Int32 Col)>> CellSets
            (
            Dictionary<String, ResidualPairPiece> PieceById
            )
        {
            Dictionary<String, HashSet<(Int32 Row, Int32 Col)>> Sets =
                new Dictionary<String, HashSet<(Int32 Row, Int32 Col)>>(StringComparer.Ordinal);
            foreach (ResidualPairPiece Piece in PieceById.Values)
            {
                if (Piece.Cells.Count == 0)
                {
                    continue;
                }
                HashSet<(Int32 Row, Int32 Col)> Cells = new HashSet<(Int32 Row, Int32 Col)>();
                foreach ((Int32 Row, Int32 Col) Cell in Piece.Cells)
                {
                    Cells.Add(Cell);
                }
                Sets[Piece.Id] = Cells;
            }
            return Sets;
        }

        /// <summary>
        /// Drops slices whose yards were taken by another trip.
        /// </summary>
        /// <param name="Slices">Loads, some of which may now be empty.</param>
        /// <returns>The slices that still hold yards.</returns>
        private static List<FillSlice> Positive
            (
            List<FillSlice> Slices
            )
        {
            List<FillSlice> Kept = new List<FillSlice>();
            foreach (FillSlice Slice in Slices)
            {
                if (Slice.Bcy > 1e-6)
                {
                    Kept.Add(Slice);
                }
            }
            return Kept;
        }

        /// <summary>
        /// Orders fill slices by distance from a centroid, then by id.
        /// </summary>
        /// <param name="Left">First slice.</param>
        /// <param name="Right">Second slice.</param>
        /// <param name="Row">Centroid row.</param>
        /// <param name="Col">Centroid column.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareByCentroid
            (
            FillSlice Left,
            FillSlice Right,
            Double Row,
            Double Col
            )
        {
            Double LeftD = ((Left.Row - Row) * (Left.Row - Row)) + ((Left.Col - Col) * (Left.Col - Col));
            Double RightD = ((Right.Row - Row) * (Right.Row - Row)) + ((Right.Col - Col) * (Right.Col - Col));
            Int32 ByDistance = LeftD.CompareTo(RightD);
            if (ByDistance != 0)
            {
                return ByDistance;
            }
            return String.Compare(Left.Id, Right.Id, StringComparison.Ordinal);
        }

        /// <summary>
        /// Orders the flow table by cut id, then fill id.
        /// </summary>
        /// <param name="Left">First flow.</param>
        /// <param name="Right">Second flow.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareFlows
            (
            ResidualPairFlow Left,
            ResidualPairFlow Right
            )
        {
            Int32 ByCut = String.Compare(Left.CutPiece, Right.CutPiece, StringComparison.Ordinal);
            if (ByCut != 0)
            {
                return ByCut;
            }
            return String.Compare(Left.FillPiece, Right.FillPiece, StringComparison.Ordinal);
        }

        /// <summary>
        /// Longest haul first. Ties take the larger flow, then the cut id, then the fill id.
        /// </summary>
        /// <param name="Left">First trip.</param>
        /// <param name="Right">Second trip.</param>
        /// <returns>A sort comparison.</returns>
        private static Int32 CompareDrafts
            (
            TripDraft Left,
            TripDraft Right
            )
        {
            Int32 ByDistance = Right.Distance.CompareTo(Left.Distance);
            if (ByDistance != 0)
            {
                return ByDistance;
            }
            Int32 ByFlow = Right.Payload.CompareTo(Left.Payload);
            if (ByFlow != 0)
            {
                return ByFlow;
            }
            Int32 ByCut = String.Compare(Left.TieCut, Right.TieCut, StringComparison.Ordinal);
            if (ByCut != 0)
            {
                return ByCut;
            }
            Int32 ByFill = String.Compare(Left.TieFill, Right.TieFill, StringComparison.Ordinal);
            if (ByFill != 0)
            {
                return ByFill;
            }
            return Left.Sequence.CompareTo(Right.Sequence);
        }

        /// <summary>
        /// Seconds for a half turn around the edge turning circle.
        /// </summary>
        /// <returns>The turn time in seconds.</returns>
        private static Double TurnSeconds
            (
            )
        {
            Double RadiusM = (Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5) * Constants.FEET_TO_METERS;
            Double ArcM = Math.PI * RadiusM;
            Double Speed = Constants.DEFAULT_TURN_SPEED_MPH * Constants.MPH_TO_MPS;
            return ArcM / Speed;
        }

        /// <summary>
        /// Draws a shaft and a head. A black casing is drawn first so the arrow shows on every colour.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X1">Start x.</param>
        /// <param name="Y1">Start y.</param>
        /// <param name="X2">End x, at the fill centroid.</param>
        /// <param name="Y2">End y.</param>
        /// <param name="Thickness">Shaft thickness in pixels.</param>
        /// <param name="Casing">Outline colour.</param>
        /// <param name="Ink">Shaft colour.</param>
        /// <param name="Dashed">True when the shaft is broken into dashes. The head stays solid.</param>
        private static void DrawArrow
            (
            Image<Rgba32> Sheet,
            Double X1,
            Double Y1,
            Double X2,
            Double Y2,
            Double Thickness,
            Rgba32 Casing,
            Rgba32 Ink,
            Boolean Dashed
            )
        {
            Double Dx = X2 - X1;
            Double Dy = Y2 - Y1;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Length < 1.0)
            {
                return;
            }
            Double Ux = Dx / Length;
            Double Uy = Dy / Length;
            Double Head = 14.0 + Thickness;
            if (Head > (Length * 0.45))
            {
                Head = Length * 0.45;
            }
            Double Shaft = Length - Head;
            Double ShaftX = X1 + (Ux * Shaft);
            Double ShaftY = Y1 + (Uy * Shaft);
            if (Dashed)
            {
                // The same gaps are used for the casing and the ink, so the field shows through.
                StrokeDashed(Sheet, X1, Y1, ShaftX, ShaftY, Thickness + 2.0, Casing);
                StrokeDashed(Sheet, X1, Y1, ShaftX, ShaftY, Thickness, Ink);
            }
            else
            {
                Stroke(Sheet, X1, Y1, ShaftX, ShaftY, Thickness + 2.0, Casing);
                Stroke(Sheet, X1, Y1, ShaftX, ShaftY, Thickness, Ink);
            }
            Double Px = -Uy;
            Double Py = Ux;
            Double Half = 5.0 + (Thickness * 0.6);
            Double BaseX = X2 - (Ux * Head);
            Double BaseY = Y2 - (Uy * Head);
            FillTriangle(
                Sheet,
                X2, Y2,
                BaseX + (Px * Half), BaseY + (Py * Half),
                BaseX - (Px * Half), BaseY - (Py * Half),
                Casing);
            Double Inset = 2.0;
            FillTriangle(
                Sheet,
                X2 - (Ux * Inset), Y2 - (Uy * Inset),
                BaseX + (Px * (Half - Inset)) + (Ux * Inset), BaseY + (Py * (Half - Inset)) + (Uy * Inset),
                BaseX - (Px * (Half - Inset)) + (Ux * Inset), BaseY - (Py * (Half - Inset)) + (Uy * Inset),
                Ink);
        }

        /// <summary>
        /// Draws a thick segment by stamping a disc along it.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X1">Start x.</param>
        /// <param name="Y1">Start y.</param>
        /// <param name="X2">End x.</param>
        /// <param name="Y2">End y.</param>
        /// <param name="Thickness">Disc diameter in pixels.</param>
        /// <param name="Color">Ink.</param>
        private static void Stroke
            (
            Image<Rgba32> Sheet,
            Double X1,
            Double Y1,
            Double X2,
            Double Y2,
            Double Thickness,
            Rgba32 Color
            )
        {
            Double Dx = X2 - X1;
            Double Dy = Y2 - Y1;
            Int32 Steps = (Int32)Math.Ceiling(Math.Sqrt((Dx * Dx) + (Dy * Dy)));
            if (Steps < 1)
            {
                Steps = 1;
            }
            Double Radius = Thickness * 0.5;
            for (Int32 Step = 0; Step <= Steps; Step += 1)
            {
                Double T = Step / (Double)Steps;
                Stamp(Sheet, X1 + (Dx * T), Y1 + (Dy * T), Radius, Color);
            }
        }

        /// <summary>
        /// Draws a thick dashed segment. The dash and the gap are the same on every shared-load trip.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X1">Start x.</param>
        /// <param name="Y1">Start y.</param>
        /// <param name="X2">End x.</param>
        /// <param name="Y2">End y.</param>
        /// <param name="Thickness">Disc diameter in pixels.</param>
        /// <param name="Color">Ink.</param>
        private static void StrokeDashed
            (
            Image<Rgba32> Sheet,
            Double X1,
            Double Y1,
            Double X2,
            Double Y2,
            Double Thickness,
            Rgba32 Color
            )
        {
            Double Dx = X2 - X1;
            Double Dy = Y2 - Y1;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Int32 Steps = (Int32)Math.Ceiling(Length);
            if (Steps < 1)
            {
                Steps = 1;
            }
            // The disc rounds each end, so the gap is longer than the dash. A shared load stays readable.
            const Double On = 22.0;
            const Double Off = 20.0;
            Double Period = On + Off;
            Double Radius = Thickness * 0.5;
            for (Int32 Step = 0; Step <= Steps; Step += 1)
            {
                Double Dist = Length * Step / (Double)Steps;
                if ((Dist % Period) > On)
                {
                    continue;
                }
                Double T = Step / (Double)Steps;
                Stamp(Sheet, X1 + (Dx * T), Y1 + (Dy * T), Radius, Color);
            }
        }

        /// <summary>
        /// Fills a disc. Pixels outside the image are skipped.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X">Centre x.</param>
        /// <param name="Y">Centre y.</param>
        /// <param name="Radius">Radius in pixels.</param>
        /// <param name="Color">Ink.</param>
        private static void Stamp
            (
            Image<Rgba32> Sheet,
            Double X,
            Double Y,
            Double Radius,
            Rgba32 Color
            )
        {
            Int32 X0 = (Int32)Math.Floor(X - Radius);
            Int32 Y0 = (Int32)Math.Floor(Y - Radius);
            Int32 X1 = (Int32)Math.Ceiling(X + Radius);
            Int32 Y1 = (Int32)Math.Ceiling(Y + Radius);
            Double Radius2 = Radius * Radius;
            for (Int32 Py = Y0; Py <= Y1; Py += 1)
            {
                for (Int32 Px = X0; Px <= X1; Px += 1)
                {
                    Double Dx = Px - X;
                    Double Dy = Py - Y;
                    if (((Dx * Dx) + (Dy * Dy)) <= Radius2)
                    {
                        Plot(Sheet, Px, Py, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Fills a triangle by testing the bounding box.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X1">First vertex x.</param>
        /// <param name="Y1">First vertex y.</param>
        /// <param name="X2">Second vertex x.</param>
        /// <param name="Y2">Second vertex y.</param>
        /// <param name="X3">Third vertex x.</param>
        /// <param name="Y3">Third vertex y.</param>
        /// <param name="Color">Ink.</param>
        private static void FillTriangle
            (
            Image<Rgba32> Sheet,
            Double X1,
            Double Y1,
            Double X2,
            Double Y2,
            Double X3,
            Double Y3,
            Rgba32 Color
            )
        {
            Double Denom = ((Y2 - Y3) * (X1 - X3)) + ((X3 - X2) * (Y1 - Y3));
            if (Math.Abs(Denom) < 1e-6)
            {
                return;
            }
            Int32 MinX = (Int32)Math.Floor(Math.Min(X1, Math.Min(X2, X3)));
            Int32 MaxX = (Int32)Math.Ceiling(Math.Max(X1, Math.Max(X2, X3)));
            Int32 MinY = (Int32)Math.Floor(Math.Min(Y1, Math.Min(Y2, Y3)));
            Int32 MaxY = (Int32)Math.Ceiling(Math.Max(Y1, Math.Max(Y2, Y3)));
            for (Int32 Y = MinY; Y <= MaxY; Y += 1)
            {
                for (Int32 X = MinX; X <= MaxX; X += 1)
                {
                    Double A = (((Y2 - Y3) * (X - X3)) + ((X3 - X2) * (Y - Y3))) / Denom;
                    Double B = (((Y3 - Y1) * (X - X3)) + ((X1 - X3) * (Y - Y3))) / Denom;
                    Double C = 1.0 - A - B;
                    if ((A >= 0.0) && (B >= 0.0) && (C >= 0.0))
                    {
                        Plot(Sheet, X, Y, Color);
                    }
                }
            }
        }

        /// <summary>
        /// Writes one pixel when it lies on the image.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X">Pixel column.</param>
        /// <param name="Y">Pixel row.</param>
        /// <param name="Color">Ink.</param>
        private static void Plot
            (
            Image<Rgba32> Sheet,
            Int32 X,
            Int32 Y,
            Rgba32 Color
            )
        {
            if ((X < 0) || (Y < 0) || (X >= Sheet.Width) || (Y >= Sheet.Height))
            {
                return;
            }
            Sheet[X, Y] = Color;
        }

        /// <summary>
        /// Finds a free label plate near the wanted point.
        /// </summary>
        /// <param name="Plates">Plates already drawn.</param>
        /// <param name="PreferredX">Wanted left edge.</param>
        /// <param name="PreferredY">Wanted top edge.</param>
        /// <param name="Width">Plate width.</param>
        /// <param name="Height">Plate height.</param>
        /// <param name="SheetWidth">Image width.</param>
        /// <param name="SheetHeight">Image height.</param>
        /// <returns>The left and top to draw.</returns>
        private static (Int32 X, Int32 Y) PlacePlate
            (
            List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)> Plates,
            Int32 PreferredX,
            Int32 PreferredY,
            Int32 Width,
            Int32 Height,
            Int32 SheetWidth,
            Int32 SheetHeight
            )
        {
            const Int32 Step = 8;
            const Int32 MaxRadius = 240;
            for (Int32 Radius = 0; Radius <= MaxRadius; Radius += Step)
            {
                if (Radius == 0)
                {
                    if (PlateFits(Plates, PreferredX, PreferredY, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX, PreferredY);
                    }
                    continue;
                }
                for (Int32 Along = -Radius; Along <= Radius; Along += Step)
                {
                    if (PlateFits(Plates, PreferredX + Along, PreferredY - Radius, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX + Along, PreferredY - Radius);
                    }
                    if (PlateFits(Plates, PreferredX + Along, PreferredY + Radius, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX + Along, PreferredY + Radius);
                    }
                    if (PlateFits(Plates, PreferredX - Radius, PreferredY + Along, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX - Radius, PreferredY + Along);
                    }
                    if (PlateFits(Plates, PreferredX + Radius, PreferredY + Along, Width, Height, SheetWidth, SheetHeight))
                    {
                        return (PreferredX + Radius, PreferredY + Along);
                    }
                }
            }
            Int32 X = Math.Clamp(PreferredX, 2, Math.Max(2, SheetWidth - Width - 2));
            Int32 Y = Math.Clamp(PreferredY, 2, Math.Max(2, SheetHeight - Height - 2));
            return (X, Y);
        }

        /// <summary>
        /// True when the plate is on the sheet and misses every plate already drawn.
        /// </summary>
        /// <param name="Plates">Plates already drawn.</param>
        /// <param name="X">Left of the candidate.</param>
        /// <param name="Y">Top of the candidate.</param>
        /// <param name="Width">Candidate width.</param>
        /// <param name="Height">Candidate height.</param>
        /// <param name="SheetWidth">Image width.</param>
        /// <param name="SheetHeight">Image height.</param>
        /// <returns>True when the candidate can be drawn.</returns>
        private static Boolean PlateFits
            (
            List<(Int32 X, Int32 Y, Int32 Width, Int32 Height)> Plates,
            Int32 X,
            Int32 Y,
            Int32 Width,
            Int32 Height,
            Int32 SheetWidth,
            Int32 SheetHeight
            )
        {
            if ((X < 2) || (Y < 2) || ((X + Width) > (SheetWidth - 2)) || ((Y + Height) > (SheetHeight - 2)))
            {
                return false;
            }
            foreach ((Int32 X, Int32 Y, Int32 Width, Int32 Height) Plate in Plates)
            {
                const Int32 Gap = 2;
                Boolean Clear = ((X + Width + Gap) <= Plate.X) || ((Plate.X + Plate.Width + Gap) <= X)
                    || ((Y + Height + Gap) <= Plate.Y) || ((Plate.Y + Plate.Height + Gap) <= Y);
                if (!Clear)
                {
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Fills a rectangle, clipped to the image.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X0">Left pixel.</param>
        /// <param name="Y0">Top pixel.</param>
        /// <param name="Width">Width in pixels.</param>
        /// <param name="Height">Height in pixels.</param>
        /// <param name="Color">Fill colour.</param>
        private static void FillRect
            (
            Image<Rgba32> Sheet,
            Int32 X0,
            Int32 Y0,
            Int32 Width,
            Int32 Height,
            Rgba32 Color
            )
        {
            Int32 X1 = Math.Min(Sheet.Width, X0 + Width);
            Int32 Y1 = Math.Min(Sheet.Height, Y0 + Height);
            Int32 XStart = Math.Max(0, X0);
            Int32 YStart = Math.Max(0, Y0);
            for (Int32 Y = YStart; Y < Y1; Y += 1)
            {
                for (Int32 X = XStart; X < X1; X += 1)
                {
                    Sheet[X, Y] = Color;
                }
            }
        }

        /// <summary>
        /// Writes a label in a 5 by 7 face.
        /// </summary>
        /// <param name="Sheet">Image being marked.</param>
        /// <param name="X0">Left pixel of the first character.</param>
        /// <param name="Y0">Top pixel of the first character.</param>
        /// <param name="Label">Text. Unknown characters are blank.</param>
        /// <param name="Scale">Pixels per font pixel.</param>
        /// <param name="Color">Ink colour.</param>
        private static void DrawLabel
            (
            Image<Rgba32> Sheet,
            Int32 X0,
            Int32 Y0,
            String Label,
            Int32 Scale,
            Rgba32 Color
            )
        {
            Int32 Cursor = X0;
            foreach (Char Letter in Label)
            {
                String[] Rows = Glyph(Letter);
                for (Int32 Row = 0; Row < Rows.Length; Row += 1)
                {
                    for (Int32 Col = 0; Col < Rows[Row].Length; Col += 1)
                    {
                        if (Rows[Row][Col] != '1')
                        {
                            continue;
                        }
                        FillRect(Sheet, Cursor + (Col * Scale), Y0 + (Row * Scale), Scale, Scale, Color);
                    }
                }
                Cursor += 6 * Scale;
            }
        }

        /// <summary>
        /// Five-pixel rows of one character. A 1 is ink.
        /// </summary>
        /// <param name="Letter">Character to look up.</param>
        /// <returns>Seven rows.</returns>
        private static String[] Glyph
            (
            Char Letter
            )
        {
            switch (Letter)
            {
                case '0': return new[] { "01110", "10001", "10011", "10101", "11001", "10001", "01110" };
                case '1': return new[] { "00100", "01100", "00100", "00100", "00100", "00100", "01110" };
                case '2': return new[] { "01110", "10001", "00001", "00010", "00100", "01000", "11111" };
                case '3': return new[] { "11110", "00001", "00001", "01110", "00001", "00001", "11110" };
                case '4': return new[] { "00010", "00110", "01010", "10010", "11111", "00010", "00010" };
                case '5': return new[] { "11111", "10000", "11110", "00001", "00001", "10001", "01110" };
                case '6': return new[] { "01110", "10000", "10000", "11110", "10001", "10001", "01110" };
                case '7': return new[] { "11111", "00001", "00010", "00100", "01000", "01000", "01000" };
                case '8': return new[] { "01110", "10001", "10001", "01110", "10001", "10001", "01110" };
                case '9': return new[] { "01110", "10001", "10001", "01111", "00001", "00001", "01110" };
                case '.': return new[] { "00000", "00000", "00000", "00000", "00000", "01100", "01100" };
                case ':': return new[] { "00000", "01100", "01100", "00000", "01100", "01100", "00000" };
                default: return new[] { "00000", "00000", "00000", "00000", "00000", "00000", "00000" };
            }
        }

        /// <summary>
        /// One scheduled trip before the sort assigns its number.
        /// </summary>
        private sealed class TripDraft
        {
            /// <summary>
            /// Creates a draft trip.
            /// </summary>
            /// <param name="Cuts">Cut loads.</param>
            /// <param name="Fills">Fill spreads, nearest first.</param>
            /// <param name="Payload">Bank cubic yards on this trip.</param>
            /// <param name="Distance">Haul metres between the volume-weighted centroids.</param>
            /// <param name="Cycle">Predicted seconds.</param>
            /// <param name="Sequence">Emission order, so equal keys stay deterministic.</param>
            public TripDraft
                (
                IReadOnlyList<FillSlice> Cuts,
                IReadOnlyList<FillSlice> Fills,
                Double Payload,
                Double Distance,
                Double Cycle,
                Int32 Sequence
                )
            {
                this.Cuts = Legs(Cuts);
                this.Fills = Legs(Fills);
                this.Payload = Payload;
                this.Distance = Distance;
                this.Cycle = Cycle;
                this.TieCut = this.Cuts[0].Id;
                this.TieFill = this.Fills[0].Id;
                this.Sequence = Sequence;
            }

            /// <summary>Gets the cut loads.</summary>
            public IReadOnlyList<ResidualPairLeg> Cuts { get; }

            /// <summary>Gets the fill spreads.</summary>
            public IReadOnlyList<ResidualPairLeg> Fills { get; }

            /// <summary>Gets the payload in bank cubic yards.</summary>
            public Double Payload { get; }

            /// <summary>Gets the haul in metres.</summary>
            public Double Distance { get; }

            /// <summary>Gets the predicted cycle in seconds.</summary>
            public Double Cycle { get; }

            /// <summary>Gets the first cut id, used to break schedule ties.</summary>
            public String TieCut { get; }

            /// <summary>Gets the first fill id, used to break schedule ties.</summary>
            public String TieFill { get; }

            /// <summary>Gets the order the trip was created in.</summary>
            public Int32 Sequence { get; }

            /// <summary>
            /// Copies slices into the public leg list.
            /// </summary>
            /// <param name="Slices">Loads to copy.</param>
            /// <returns>The legs.</returns>
            private static List<ResidualPairLeg> Legs
                (
                IReadOnlyList<FillSlice> Slices
                )
            {
                List<ResidualPairLeg> Copied = new List<ResidualPairLeg>(Slices.Count);
                foreach (FillSlice Slice in Slices)
                {
                    Copied.Add(new ResidualPairLeg(Slice.Id, Slice.Bcy));
                }
                return Copied;
            }
        }

        /// <summary>
        /// Yards still to place on one fill piece while a cut piece is poured into bowls.
        /// </summary>
        private sealed class FillSlice
        {
            /// <summary>
            /// Creates a slice.
            /// </summary>
            /// <param name="Id">Fill or cut piece id.</param>
            /// <param name="Bcy">Bank cubic yards still on this slice.</param>
            /// <param name="Distance">Metres from the cut that owns the group. Zero for a cut load.</param>
            /// <param name="Row">Centroid row.</param>
            /// <param name="Col">Centroid column.</param>
            public FillSlice
                (
                String Id,
                Double Bcy,
                Double Distance,
                Double Row,
                Double Col
                )
            {
                this.Id = Id;
                this.Bcy = Bcy;
                this.Distance = Distance;
                this.Row = Row;
                this.Col = Col;
            }

            /// <summary>Gets the piece id.</summary>
            public String Id { get; }

            /// <summary>Gets or sets the yards still on this slice.</summary>
            public Double Bcy { get; set; }

            /// <summary>Gets the haul used to order fill pieces.</summary>
            public Double Distance { get; }

            /// <summary>Gets the centroid row.</summary>
            public Double Row { get; }

            /// <summary>Gets the centroid column.</summary>
            public Double Col { get; }
        }

        /// <summary>
        /// The part of one cut piece that did not fill a bowl, aimed at one fill cluster.
        /// </summary>
        private sealed class CutRemainder
        {
            /// <summary>
            /// Creates a remainder.
            /// </summary>
            /// <param name="Cut">The cut piece.</param>
            /// <param name="Fills">Fill slices still to be spread, nearest first.</param>
            /// <param name="Bcy">Yards in those slices.</param>
            /// <param name="FillCluster">Parent id of the fill pieces.</param>
            public CutRemainder
                (
                ResidualPairPiece Cut,
                List<FillSlice> Fills,
                Double Bcy,
                String FillCluster
                )
            {
                this.Cut = Cut;
                this.Fills = Fills;
                this.Bcy = Bcy;
                this.FillCluster = FillCluster;
            }

            /// <summary>Gets the cut piece.</summary>
            public ResidualPairPiece Cut { get; }

            /// <summary>Gets the fill slices.</summary>
            public List<FillSlice> Fills { get; }

            /// <summary>Gets the yards.</summary>
            public Double Bcy { get; }

            /// <summary>Gets the fill cluster id.</summary>
            public String FillCluster { get; }
        }

        /// <summary>
        /// One partial trip while neighbouring remainders are still being moved onto it.
        /// </summary>
        private sealed class OpenLoad
        {
            /// <summary>
            /// Creates a load from one remainder.
            /// </summary>
            /// <param name="Cuts">Cut pieces and the yards still loaded from each.</param>
            /// <param name="Fills">Fill slices still to be spread.</param>
            /// <param name="FillCluster">Parent id of those fill pieces.</param>
            /// <param name="Sequence">Creation order, used when the haul sort ties.</param>
            public OpenLoad
                (
                List<FillSlice> Cuts,
                List<FillSlice> Fills,
                String FillCluster,
                Int32 Sequence
                )
            {
                this.Cuts = Cuts;
                this.Fills = Fills;
                this.FillCluster = FillCluster;
                this.Sequence = Sequence;
            }

            /// <summary>Gets the cut loads. Yards are reduced when another trip takes them.</summary>
            public List<FillSlice> Cuts { get; }

            /// <summary>Gets or sets the fill spreads.</summary>
            public List<FillSlice> Fills { get; set; }

            /// <summary>Gets the fill cluster id.</summary>
            public String FillCluster { get; }

            /// <summary>Gets the creation order.</summary>
            public Int32 Sequence { get; }

            /// <summary>Gets the yards still on this load.</summary>
            public Double Payload
            {
                get { return SumBcy(this.Cuts); }
            }
        }

        /// <summary>
        /// One residual edge in the transportation graph.
        /// </summary>
        private sealed class FlowEdge
        {
            /// <summary>
            /// Creates an edge.
            /// </summary>
            /// <param name="To">Head node.</param>
            /// <param name="Rev">Index of the reverse edge in the head's list.</param>
            /// <param name="Cap">Residual capacity in quanta.</param>
            /// <param name="Cost">Cost of sending one quantum's worth at the per-yard rate.</param>
            public FlowEdge
                (
                Int32 To,
                Int32 Rev,
                Int32 Cap,
                Double Cost
                )
            {
                this.To = To;
                this.Rev = Rev;
                this.Cap = Cap;
                this.Cost = Cost;
            }

            /// <summary>Gets the head node.</summary>
            public Int32 To { get; }

            /// <summary>Gets the reverse edge index.</summary>
            public Int32 Rev { get; }

            /// <summary>Gets or sets the residual capacity.</summary>
            public Int32 Cap { get; set; }

            /// <summary>Gets the edge cost.</summary>
            public Double Cost { get; }
        }
    }

    /// <summary>
    /// One cut or fill piece handed to the pairing. Clustering builds these. Pairing does not.
    /// </summary>
    public sealed class ResidualPairPiece
    {
        /// <summary>
        /// Creates a piece.
        /// </summary>
        /// <param name="Id">Piece id, such as C1.1 or F2.</param>
        /// <param name="Sign">"cut" or "fill".</param>
        /// <param name="HaulVolumeBcy">Haul volume in bank cubic yards.</param>
        /// <param name="MeanMm">Mean depth being moved, in millimetres. The cycle time uses this.</param>
        /// <param name="CentroidRow">Volume-weighted row.</param>
        /// <param name="CentroidCol">Volume-weighted column.</param>
        /// <param name="LongAxisLenM">Long-axis length in metres.</param>
        /// <param name="Cells">Member cells. Empty when the caller has no grid. Used to test an 8-connected edge.</param>
        public ResidualPairPiece
            (
            String Id,
            String Sign,
            Double HaulVolumeBcy,
            Double MeanMm,
            Double CentroidRow,
            Double CentroidCol,
            Double LongAxisLenM,
            IReadOnlyList<(Int32 Row, Int32 Col)>? Cells = null
            )
        {
            this.Id = Id;
            this.Sign = Sign;
            this.HaulVolumeBcy = HaulVolumeBcy;
            this.MeanMm = MeanMm;
            this.CentroidRow = CentroidRow;
            this.CentroidCol = CentroidCol;
            this.LongAxisLenM = LongAxisLenM;
            this.Cells = Cells ?? Array.Empty<(Int32 Row, Int32 Col)>();
        }

        /// <summary>Gets the piece id.</summary>
        public String Id { get; }

        /// <summary>Gets "cut" or "fill".</summary>
        public String Sign { get; }

        /// <summary>Gets the haul volume in bank cubic yards.</summary>
        public Double HaulVolumeBcy { get; }

        /// <summary>Gets the mean depth being moved, in millimetres.</summary>
        public Double MeanMm { get; }

        /// <summary>Gets the volume-weighted row.</summary>
        public Double CentroidRow { get; }

        /// <summary>Gets the volume-weighted column.</summary>
        public Double CentroidCol { get; }

        /// <summary>Gets the long-axis length in metres.</summary>
        public Double LongAxisLenM { get; }

        /// <summary>Gets the member cells. Empty when the piece was built without a grid.</summary>
        public IReadOnlyList<(Int32 Row, Int32 Col)> Cells { get; }
    }

    /// <summary>
    /// The flows and the schedule for one clustering of the field.
    /// </summary>
    public sealed class ResidualPairPlan
    {
        /// <summary>
        /// Creates a plan.
        /// </summary>
        /// <param name="Flows">Kept flows, including LOCAL and SHORT.</param>
        /// <param name="Trips">Scheduled trips. LOCAL and SHORT are absent.</param>
        /// <param name="Remainders">Trips that stayed under a quarter bowl after remainder packing.</param>
        /// <param name="TotalCutBcy">Sum of the cut piece volumes.</param>
        /// <param name="TotalFillBcy">Sum of the fill piece volumes.</param>
        /// <param name="BalanceKind">LOCAL, SHORT, or balanced.</param>
        /// <param name="BalanceBcy">The unmatched volume. Zero when the two sides match.</param>
        /// <param name="PartialTrips">Trips that are not a full bowl.</param>
        /// <param name="MeanPayloadBcy">Mean trip payload.</param>
        /// <param name="PredictedHours">Sum of the trip cycles, in hours.</param>
        /// <param name="MeanHaulM">Payload-weighted mean haul of the trips.</param>
        public ResidualPairPlan
            (
            IReadOnlyList<ResidualPairFlow> Flows,
            IReadOnlyList<ResidualPairTrip> Trips,
            IReadOnlyList<ResidualPairRemainder> Remainders,
            Double TotalCutBcy,
            Double TotalFillBcy,
            String BalanceKind,
            Double BalanceBcy,
            Int32 PartialTrips,
            Double MeanPayloadBcy,
            Double PredictedHours,
            Double MeanHaulM
            )
        {
            this.Flows = Flows;
            this.Trips = Trips;
            this.Remainders = Remainders;
            this.TotalCutBcy = TotalCutBcy;
            this.TotalFillBcy = TotalFillBcy;
            this.BalanceKind = BalanceKind;
            this.BalanceBcy = BalanceBcy;
            this.PartialTrips = PartialTrips;
            this.MeanPayloadBcy = MeanPayloadBcy;
            this.PredictedHours = PredictedHours;
            this.MeanHaulM = MeanHaulM;
        }

        /// <summary>Gets the flows.</summary>
        public IReadOnlyList<ResidualPairFlow> Flows { get; }

        /// <summary>Gets the scheduled trips.</summary>
        public IReadOnlyList<ResidualPairTrip> Trips { get; }

        /// <summary>Gets the volumes left for correcting after remainder packing.</summary>
        public IReadOnlyList<ResidualPairRemainder> Remainders { get; }

        /// <summary>Gets the cut volume in bank cubic yards.</summary>
        public Double TotalCutBcy { get; }

        /// <summary>Gets the fill volume in bank cubic yards.</summary>
        public Double TotalFillBcy { get; }

        /// <summary>Gets LOCAL, SHORT, or balanced.</summary>
        public String BalanceKind { get; }

        /// <summary>Gets the unmatched volume in bank cubic yards.</summary>
        public Double BalanceBcy { get; }

        /// <summary>Gets how many trips are not a full bowl.</summary>
        public Int32 PartialTrips { get; }

        /// <summary>Gets the mean trip payload in bank cubic yards.</summary>
        public Double MeanPayloadBcy { get; }

        /// <summary>Gets the sum of the predicted cycles, in hours.</summary>
        public Double PredictedHours { get; }

        /// <summary>Gets the payload-weighted mean haul in metres.</summary>
        public Double MeanHaulM { get; }

        /// <summary>
        /// Writes the totals, then every flow.
        /// </summary>
        /// <param name="Writer">Where to print.</param>
        public void Print
            (
            TextWriter Writer
            )
        {
            if (Writer == null) throw new ArgumentNullException(nameof(Writer));
            Writer.WriteLine("total cut: {0:R} bcy", this.TotalCutBcy);
            Writer.WriteLine("total fill: {0:R} bcy", this.TotalFillBcy);
            Writer.WriteLine("{0}: {1:R} bcy", this.BalanceKind, this.BalanceBcy);
            Writer.WriteLine("trips: {0}", this.Trips.Count);
            Writer.WriteLine("partial trips: {0}", this.PartialTrips);
            Writer.WriteLine("mean payload: {0:R} bcy", this.MeanPayloadBcy);
            Writer.WriteLine("predicted hours: {0:R}", this.PredictedHours);
            Writer.WriteLine("volume-weighted mean haul: {0:R} m", this.MeanHaulM);
            Writer.WriteLine("{0,-12} {1,-12} {2,12} {3,12} {4,12}", "cut", "fill", "bcy", "d_m", "c_ij");
            foreach (ResidualPairFlow Flow in this.Flows)
            {
                Writer.WriteLine(
                    "{0,-12} {1,-12} {2,12:F2} {3,12:F1} {4,12:F2}",
                    Flow.CutPiece,
                    Flow.FillPiece,
                    Flow.VolumeBcy,
                    Flow.DistanceM,
                    Flow.SecondsPerBcy);
            }
            foreach (ResidualPairRemainder Remainder in this.Remainders)
            {
                Writer.WriteLine(
                    "correcting: {0} {1} {2:R} bcy",
                    Remainder.CutPiece,
                    Remainder.FillPiece,
                    Remainder.VolumeBcy);
            }
        }
    }

    /// <summary>
    /// Volume sent from one cut piece to one fill piece.
    /// </summary>
    public sealed class ResidualPairFlow
    {
        /// <summary>
        /// Creates a flow.
        /// </summary>
        /// <param name="CutPiece">Cut id, or SHORT.</param>
        /// <param name="FillPiece">Fill id, or LOCAL.</param>
        /// <param name="VolumeBcy">Bank cubic yards. At least one quantum.</param>
        /// <param name="DistanceM">Haul metres. Zero for LOCAL and SHORT.</param>
        /// <param name="SecondsPerBcy">Cost in seconds per bank cubic yard.</param>
        public ResidualPairFlow
            (
            String CutPiece,
            String FillPiece,
            Double VolumeBcy,
            Double DistanceM,
            Double SecondsPerBcy
            )
        {
            this.CutPiece = CutPiece;
            this.FillPiece = FillPiece;
            this.VolumeBcy = VolumeBcy;
            this.DistanceM = DistanceM;
            this.SecondsPerBcy = SecondsPerBcy;
        }

        /// <summary>Gets the cut id.</summary>
        public String CutPiece { get; }

        /// <summary>Gets the fill id.</summary>
        public String FillPiece { get; }

        /// <summary>Gets the volume in bank cubic yards.</summary>
        public Double VolumeBcy { get; }

        /// <summary>Gets the haul in metres.</summary>
        public Double DistanceM { get; }

        /// <summary>Gets the cost in seconds per bank cubic yard.</summary>
        public Double SecondsPerBcy { get; }
    }

    /// <summary>
    /// Yards loaded from one piece or spread on one piece during a trip.
    /// </summary>
    public sealed class ResidualPairLeg
    {
        /// <summary>
        /// Creates a leg.
        /// </summary>
        /// <param name="Id">Piece id.</param>
        /// <param name="Bcy">Bank cubic yards on this leg.</param>
        public ResidualPairLeg
            (
            String Id,
            Double Bcy
            )
        {
            this.Id = Id;
            this.Bcy = Bcy;
        }

        /// <summary>Gets the piece id.</summary>
        [JsonPropertyName("id")]
        public String Id { get; }

        /// <summary>Gets the yards on this leg.</summary>
        [JsonPropertyName("bcy")]
        public Double Bcy { get; }
    }

    /// <summary>
    /// One scheduled trip. It can load from two adjacent cut pieces and spread across one fill cluster.
    /// </summary>
    public sealed class ResidualPairTrip
    {
        /// <summary>
        /// Creates a trip.
        /// </summary>
        /// <param name="TripNo">Schedule position, starting at 1.</param>
        /// <param name="Cuts">Cut pieces and the yards loaded from each, along the cut axis.</param>
        /// <param name="Fills">Fill pieces and the yards spread on each, nearest first.</param>
        /// <param name="PayloadBcy">Bank cubic yards on this trip.</param>
        /// <param name="DistanceM">Metres between the volume-weighted cut and fill centroids.</param>
        /// <param name="PredictedCycleS">Predicted seconds for this trip.</param>
        public ResidualPairTrip
            (
            Int32 TripNo,
            IReadOnlyList<ResidualPairLeg> Cuts,
            IReadOnlyList<ResidualPairLeg> Fills,
            Double PayloadBcy,
            Double DistanceM,
            Double PredictedCycleS
            )
        {
            this.TripNo = TripNo;
            this.Cuts = Cuts;
            this.Fills = Fills;
            this.PayloadBcy = PayloadBcy;
            this.DistanceM = DistanceM;
            this.PredictedCycleS = PredictedCycleS;
        }

        /// <summary>Gets the schedule position, starting at 1.</summary>
        [JsonPropertyName("trip_no")]
        public Int32 TripNo { get; }

        /// <summary>Gets the cut loads.</summary>
        [JsonPropertyName("cuts")]
        public IReadOnlyList<ResidualPairLeg> Cuts { get; }

        /// <summary>Gets the fill spreads.</summary>
        [JsonPropertyName("fills")]
        public IReadOnlyList<ResidualPairLeg> Fills { get; }

        /// <summary>Gets the payload in bank cubic yards.</summary>
        [JsonPropertyName("payload_bcy")]
        public Double PayloadBcy { get; }

        /// <summary>Gets the haul in metres.</summary>
        [JsonPropertyName("d_m")]
        public Double DistanceM { get; }

        /// <summary>Gets the predicted cycle in seconds.</summary>
        [JsonPropertyName("predicted_cycle_s")]
        public Double PredictedCycleS { get; }
    }

    /// <summary>
    /// A packed trip under a quarter bowl. The correcting phase takes its volume.
    /// </summary>
    public sealed class ResidualPairRemainder
    {
        /// <summary>
        /// Creates a remainder.
        /// </summary>
        /// <param name="CutPiece">Cut piece id.</param>
        /// <param name="FillPiece">Fill piece id.</param>
        /// <param name="VolumeBcy">Bank cubic yards left behind.</param>
        public ResidualPairRemainder
            (
            String CutPiece,
            String FillPiece,
            Double VolumeBcy
            )
        {
            this.CutPiece = CutPiece;
            this.FillPiece = FillPiece;
            this.VolumeBcy = VolumeBcy;
        }

        /// <summary>Gets the cut piece id.</summary>
        public String CutPiece { get; }

        /// <summary>Gets the fill piece id.</summary>
        public String FillPiece { get; }

        /// <summary>Gets the volume in bank cubic yards.</summary>
        public Double VolumeBcy { get; }
    }
}
