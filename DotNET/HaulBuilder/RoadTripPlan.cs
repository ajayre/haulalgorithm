using System.Globalization;

namespace HaulBuilder
{
    /// <summary>
    /// Forward road trips: several cut runs, then several spread runs, all one way.
    /// </summary>
    static public partial class RoadPadBuild
    {
        /// <summary>
        /// One cell visited while a road trip is still a candidate.
        /// </summary>
        private sealed class WorkStep
        {
            /// <summary>
            /// Initializes a step.
            /// </summary>
            /// <param name="Bx">Column.</param>
            /// <param name="By">Row.</param>
            /// <param name="Key">Signed lane index.</param>
            /// <param name="Order">Station along the feature.</param>
            /// <param name="DepthM">Depth taken, in metres.</param>
            public WorkStep(Int32 Bx, Int32 By, Int32 Key, Double Order, Double DepthM)
            {
                this.Bx = Bx;
                this.By = By;
                this.Key = Key;
                this.Order = Order;
                this.DepthM = DepthM;
            }

            /// <summary>Gets the column.</summary>
            public Int32 Bx { get; }

            /// <summary>Gets the row.</summary>
            public Int32 By { get; }

            /// <summary>Gets the signed lane index.</summary>
            public Int32 Key { get; }

            /// <summary>Gets the station along the feature.</summary>
            public Double Order { get; }

            /// <summary>Gets the depth taken, in metres.</summary>
            public Double DepthM { get; }
        }

        /// <summary>
        /// A road trip that has not yet been committed to the pan, the blade, or another trip.
        /// </summary>
        private sealed class RoadTripDraft
        {
            /// <summary>Gets or sets the cut cells, in travel order.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts { get; set; }

            /// <summary>Gets or sets the fill cells, in travel order.</summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills { get; set; }

            /// <summary>Gets or sets the matched load in BCY.</summary>
            public Double LoadBcy { get; set; }

            /// <summary>Gets or sets +1 for increasing station and -1 for decreasing station.</summary>
            public Int32 Sign { get; set; }

            /// <summary>Gets or sets the along-track span of the cells, in metres.</summary>
            public Double SpanM { get; set; }
        }

        /// <summary>
        /// Plans pan trips that cut, then spread, in one direction.
        /// A lane change is a forward S-bend. A trip under the minimum load is merged
        /// into a same-heading trip with bowl room, or handed to the blade when the
        /// shove stays within the blade limit.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Surface">Pan mask and the blade total.</param>
        /// <param name="Item">Road or pad the station is measured along.</param>
        /// <param name="Lanes">Fill lanes. Cells are in station order. Depths are consumed.</param>
        /// <param name="LaneKeys">Signed lane index of each fill lane.</param>
        /// <param name="Stocks">Remaining reserved cut. Depths are consumed.</param>
        /// <param name="BowlBcy">Scraper capacity in BCY.</param>
        /// <param name="MaxDepthM">Maximum depth per lift, in metres.</param>
        /// <param name="FillPerM">Bank BCY of one metre of fill on one cell.</param>
        /// <param name="CutPerM">Bank BCY of one metre of cut on one cell.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <param name="MinLoadPct">Smallest fraction of the bowl that is still a pan trip.</param>
        /// <param name="BladeMaxM">Longest along-track blade shove, in metres.</param>
        /// <param name="BladeCuts">Cut cells handed to the blade.</param>
        /// <param name="BladeBcy">Bank cubic yards handed to the blade.</param>
        /// <returns>The pan trips.</returns>
        private static List<Pass> EmitForward
            (
            Grid Grid,
            RoadPadSurface Surface,
            Feature Item,
            List<List<LaneCell>> Lanes,
            List<Int32> LaneKeys,
            List<Stock> Stocks,
            Double BowlBcy,
            Double MaxDepthM,
            Double FillPerM,
            Double CutPerM,
            Double LaneM,
            Double RadiusM,
            Double MinLoadPct,
            Double BladeMaxM,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> BladeCuts,
            ref Double BladeBcy
            )
        {
            List<Pass> Passes = new List<Pass>();
            List<Int32> Signs = new List<Int32>();
            HashSet<(Int32 Bx, Int32 By)> Skip = new HashSet<(Int32 Bx, Int32 By)>();
            Double Floor = MinLoadPct * BowlBcy;
            Int32 Guard = 0;
            Int32 Dry = 0;
            while (HasFill(Lanes) && (Guard < 10000))
            {
                Guard += 1;
                Double Before = RemainingFillBcy(Lanes, FillPerM);
                // Both headings are scored. The fuller load is kept. A tie stays on increasing station.
                RoadTripDraft Forward = PlanDirection(
                    Grid, Surface, Item, Lanes, LaneKeys, Stocks, Skip, BowlBcy, MaxDepthM, FillPerM, CutPerM, LaneM, RadiusM, 1);
                RoadTripDraft Back = PlanDirection(
                    Grid, Surface, Item, Lanes, LaneKeys, Stocks, Skip, BowlBcy, MaxDepthM, FillPerM, CutPerM, LaneM, RadiusM, -1);
                RoadTripDraft Best = PreferDraft(Forward, Back);
                if ((Best == null) || (Best.LoadBcy <= 1e-4))
                {
                    break;
                }

                if (Best.LoadBcy + 1e-6 >= Floor)
                {
                    Dry = 0;
                    CommitPan(Grid, Item, Lanes, Stocks, Passes, Signs, Best);
                }
                else if (TryMerge(Grid, Item, Passes, Signs, Best, BowlBcy, LaneM))
                {
                    Dry = 0;
                    ApplyDraft(Lanes, Stocks, Best);
                }
                else if (Best.SpanM <= (BladeMaxM + 1e-6))
                {
                    // A light load the pan will not make. The cells already sit within one blade shove.
                    Dry += 1;
                    ApplyDraft(Lanes, Stocks, Best);
                    BladeBcy += Best.LoadBcy;
                    BladeCuts.AddRange(Best.Cuts);
                    if (Dry >= 80)
                    {
                        break;
                    }
                }
                else
                {
                    // Skipping this pocket lets a later plan see the soil beyond it.
                    // A pad can offer the same light pocket forever, so the retries are capped.
                    Int32 Marked = Skip.Count;
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Best.Fills)
                    {
                        Skip.Add(Fill.Cell);
                    }

                    Dry += 1;
                    if ((Skip.Count == Marked) || (Dry >= 80))
                    {
                        break;
                    }

                    continue;
                }

                // A draft that does not consume soil would be planned again forever.
                if (RemainingFillBcy(Lanes, FillPerM) > (Before - 1e-4))
                {
                    break;
                }
            }

            BladeLocalRemainder(Grid, Item, Lanes, Stocks, BladeCuts, ref BladeBcy, FillPerM, CutPerM, BladeMaxM);
            WarnUnmoved(Surface, Lanes, FillPerM, MinLoadPct, BladeMaxM);
            return Passes;
        }

        /// <summary>
        /// Picks the fuller draft. Equal loads keep increasing station.
        /// </summary>
        /// <param name="Forward">Draft for increasing station. Null when that heading cannot move soil.</param>
        /// <param name="Back">Draft for decreasing station.</param>
        /// <returns>The draft to commit, or null.</returns>
        private static RoadTripDraft PreferDraft
            (
            RoadTripDraft Forward,
            RoadTripDraft Back
            )
        {
            if ((Forward == null) || (Forward.LoadBcy <= 1e-4))
            {
                return Back;
            }

            if ((Back == null) || (Back.LoadBcy <= 1e-4))
            {
                return Forward;
            }

            if (Back.LoadBcy > (Forward.LoadBcy + 1e-4))
            {
                return Back;
            }

            return Forward;
        }

        /// <summary>
        /// Finds the forward cut-then-spread split that places the most soil.
        /// Nothing is consumed. The caller commits the draft it keeps.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Surface">Pan mask. Another feature's trim is left alone.</param>
        /// <param name="Item">Road or pad the station is measured along.</param>
        /// <param name="FillLanes">Fill lanes.</param>
        /// <param name="FillKeys">Signed lane index of each fill lane.</param>
        /// <param name="Stocks">Remaining cut.</param>
        /// <param name="Skip">Fill cells already refused as a pan trip.</param>
        /// <param name="BowlBcy">Scraper capacity in BCY.</param>
        /// <param name="MaxDepthM">Maximum depth per lift, in metres.</param>
        /// <param name="FillPerM">Bank BCY of one metre of fill on one cell.</param>
        /// <param name="CutPerM">Bank BCY of one metre of cut on one cell.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <returns>The best draft, or null when this heading has nothing to move.</returns>
        private static RoadTripDraft PlanDirection
            (
            Grid Grid,
            RoadPadSurface Surface,
            Feature Item,
            List<List<LaneCell>> FillLanes,
            List<Int32> FillKeys,
            List<Stock> Stocks,
            HashSet<(Int32 Bx, Int32 By)> Skip,
            Double BowlBcy,
            Double MaxDepthM,
            Double FillPerM,
            Double CutPerM,
            Double LaneM,
            Double RadiusM,
            Int32 Sign
            )
        {
            List<List<LaneCell>> CutLanes = new List<List<LaneCell>>();
            List<Int32> CutKeys = new List<Int32>();
            BuildCutLanes(Grid, Surface, Item, Stocks, LaneM, CutLanes, CutKeys);
            List<WorkStep> Path = EnumerateCuts(
                CutLanes, CutKeys, FillLanes, FillKeys, Skip, BowlBcy, MaxDepthM, CutPerM, LaneM, RadiusM, Sign);
            if (Path.Count == 0)
            {
                return null;
            }

            // Each prefix is a place to stop cutting and start spreading.
            // A long bank is sampled. Scoring every cell is what made a pad stall.
            Double[] Saved = SaveFill(FillLanes);
            RoadTripDraft Best = null;
            foreach (Int32 Count in SplitCounts(Path.Count))
            {
                RoadTripDraft Draft = ScorePrefix(
                    Grid, Item, FillLanes, FillKeys, Skip, Path, Count, Sign, MaxDepthM, FillPerM, CutPerM, LaneM, RadiusM, Saved);
                if (Draft == null)
                {
                    continue;
                }

                Boolean Fuller = (Best == null) || (Draft.LoadBcy > (Best.LoadBcy + 1e-4));
                Boolean SameLoad = (Best != null)
                    && (Math.Abs(Draft.LoadBcy - Best.LoadBcy) <= 1e-4)
                    && ((Draft.Fills.Count > Best.Fills.Count)
                        || ((Draft.Fills.Count == Best.Fills.Count) && (Draft.Cuts.Count > Best.Cuts.Count)));
                if (Fuller || SameLoad)
                {
                    Best = Draft;
                }
            }

            RestoreFill(FillLanes, Saved);
            return Best;
        }

        /// <summary>
        /// Prefix lengths to score. Short paths are scored in full. Long paths are sampled.
        /// </summary>
        /// <param name="Count">Cut steps available.</param>
        /// <returns>The prefix lengths, each at least 1.</returns>
        private static List<Int32> SplitCounts
            (
            Int32 Count
            )
        {
            List<Int32> Counts = new List<Int32>();
            if (Count <= 48)
            {
                for (Int32 Index = 1; Index <= Count; Index += 1)
                {
                    Counts.Add(Index);
                }

                return Counts;
            }

            Counts.Add(1);
            Counts.Add(Count);
            for (Int32 Sample = 1; Sample < 24; Sample += 1)
            {
                Int32 Index = 1 + ((Sample * (Count - 1)) / 24);
                if (!Counts.Contains(Index))
                {
                    Counts.Add(Index);
                }
            }

            return Counts;
        }

        /// <summary>
        /// Scores one place to stop cutting. Restores the fill lanes before it returns.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="FillLanes">Fill lanes.</param>
        /// <param name="FillKeys">Signed lane index of each fill lane.</param>
        /// <param name="Skip">Fill cells that are not available.</param>
        /// <param name="Path">Cut steps in travel order.</param>
        /// <param name="Count">How many cut steps to keep before spreading.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="MaxDepthM">Maximum depth per lift, in metres.</param>
        /// <param name="FillPerM">Bank BCY of one metre of fill on one cell.</param>
        /// <param name="CutPerM">Bank BCY of one metre of cut on one cell.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <param name="Saved">Fill depths to restore.</param>
        /// <returns>The draft, or null when that split moves no soil.</returns>
        private static RoadTripDraft ScorePrefix
            (
            Grid Grid,
            Feature Item,
            List<List<LaneCell>> FillLanes,
            List<Int32> FillKeys,
            HashSet<(Int32 Bx, Int32 By)> Skip,
            List<WorkStep> Path,
            Int32 Count,
            Int32 Sign,
            Double MaxDepthM,
            Double FillPerM,
            Double CutPerM,
            Double LaneM,
            Double RadiusM,
            Double[] Saved
            )
        {
            RestoreFill(FillLanes, Saved);
            HashSet<(Int32 Bx, Int32 By)> Blocked = new HashSet<(Int32 Bx, Int32 By)>(Skip);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double CutBcy = 0.0;
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                WorkStep Step = Path[Index];
                AddFill(Cuts, Step.Bx, Step.By, Step.DepthM);
                CutBcy += Step.DepthM * CutPerM;
                Blocked.Add((Step.Bx, Step.By));
            }

            ZeroBlocked(FillLanes, Blocked);
            WorkStep End = Path[Count - 1];
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double FillBcy = WalkFills(
                FillLanes, FillKeys, End.Key, End.Order, Sign, CutBcy, MaxDepthM, FillPerM, LaneM, RadiusM, Fills);
            BalanceLoads(Cuts, Fills, ref CutBcy, ref FillBcy, CutPerM, FillPerM);
            RestoreFill(FillLanes, Saved);
            if ((Cuts.Count == 0) || (Fills.Count == 0) || (FillBcy <= 1e-4))
            {
                return null;
            }

            return new RoadTripDraft
            {
                Cuts = Cuts,
                Fills = Fills,
                LoadBcy = FillBcy,
                Sign = Sign,
                SpanM = AlongSpanM(Grid, Item, Cuts, Fills),
            };
        }

        /// <summary>
        /// Groups remaining cut into blade-width lanes along this feature.
        /// Pan trim that belongs to another road or pad is left for that feature.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Surface">Pan mask and owners.</param>
        /// <param name="Item">Road or pad the lanes are measured on.</param>
        /// <param name="Stocks">Remaining cut.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="Lanes">Receives one list per lane, cells in station order.</param>
        /// <param name="Keys">Receives the signed lane index of each lane.</param>
        private static void BuildCutLanes
            (
            Grid Grid,
            RoadPadSurface Surface,
            Feature Item,
            List<Stock> Stocks,
            Double LaneM,
            List<List<LaneCell>> Lanes,
            List<Int32> Keys
            )
        {
            Dictionary<Int32, List<LaneCell>> Bins = new Dictionary<Int32, List<LaneCell>>();
            foreach (Stock Cell in Stocks)
            {
                if (Cell.LeftM <= LIFT_EPSILON_M)
                {
                    continue;
                }

                if ((Surface.PanM[Cell.By, Cell.Bx] > LIFT_EPSILON_M) && !OwnedBy(Surface, Item, Cell.Bx, Cell.By))
                {
                    continue;
                }

                LanePose(Grid, Item, Cell.Bx, Cell.By, out Double Order, out Double Signed);
                Int32 Key = LaneIndex(Signed, LaneM);
                if (!Bins.TryGetValue(Key, out List<LaneCell> Cells))
                {
                    Cells = new List<LaneCell>();
                    Bins[Key] = Cells;
                }

                Cells.Add(new LaneCell(Cell.Bx, Cell.By, Order, Cell.LeftM));
            }

            List<Int32> Ordered = new List<Int32>(Bins.Keys);
            Ordered.Sort(CompareLanes);
            foreach (Int32 Key in Ordered)
            {
                List<LaneCell> Cells = Bins[Key];
                Cells.Sort((Left, Right) => Left.Order.CompareTo(Right.Order));
                Lanes.Add(Cells);
                Keys.Add(Key);
            }
        }

        /// <summary>
        /// True when this feature owns the pan cell.
        /// </summary>
        /// <param name="Surface">Owners.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <returns>False when another feature owns the cell, or nobody does.</returns>
        private static Boolean OwnedBy
            (
            RoadPadSurface Surface,
            Feature Item,
            Int32 Bx,
            Int32 By
            )
        {
            if ((Surface.OwnerIndex[By, Bx] < 0) || (Surface.OwnerIsPad[By, Bx] != Item.IsPad))
            {
                return false;
            }

            Int32 Index = Item.IsPad ? Item.PadIndex : Item.RoadIndex;
            return Surface.OwnerIndex[By, Bx] == Index;
        }

        /// <summary>
        /// Station and signed offset of a cell on a road or pad.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="Order">Station along the feature.</param>
        /// <param name="Signed">Offset left of travel.</param>
        private static void LanePose
            (
            Grid Grid,
            Feature Item,
            Int32 Bx,
            Int32 By,
            out Double Order,
            out Double Signed
            )
        {
            Double X = Grid.CenterX[By, Bx];
            Double Y = Grid.CenterY[By, Bx];
            if (Item.IsPad)
            {
                Signed = ((X - Item.X) * Item.PerpX) + ((Y - Item.Y) * Item.PerpY);
                Order = ((X - Item.X) * Item.DirX) + ((Y - Item.Y) * Item.DirY);
                return;
            }

            Hit Closest = Item.Closest(X, Y);
            Signed = Closest.Signed;
            Order = Closest.Station;
        }

        /// <summary>
        /// Walks cut cells forward from the earliest legal start until the bowl is full
        /// or the next cell has no fill still ahead of it. Depths are put back.
        /// </summary>
        /// <param name="Lanes">Cut lanes. Restored before the method returns.</param>
        /// <param name="Keys">Signed lane index of each cut lane.</param>
        /// <param name="FillLanes">Fill lanes used to see whether a cut still has somewhere to spread.</param>
        /// <param name="FillKeys">Signed lane index of each fill lane.</param>
        /// <param name="Skip">Fill cells that are not available.</param>
        /// <param name="BowlBcy">Scraper capacity in BCY.</param>
        /// <param name="MaxDepthM">Maximum depth per lift, in metres.</param>
        /// <param name="CutPerM">Bank BCY of one metre of cut on one cell.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <returns>The cut steps in travel order. Empty when nothing can be cut.</returns>
        private static List<WorkStep> EnumerateCuts
            (
            List<List<LaneCell>> Lanes,
            List<Int32> Keys,
            List<List<LaneCell>> FillLanes,
            List<Int32> FillKeys,
            HashSet<(Int32 Bx, Int32 By)> Skip,
            Double BowlBcy,
            Double MaxDepthM,
            Double CutPerM,
            Double LaneM,
            Double RadiusM,
            Int32 Sign
            )
        {
            List<WorkStep> Path = new List<WorkStep>();
            if (!TryFindCutStart(Lanes, Keys, FillLanes, FillKeys, Skip, LaneM, RadiusM, Sign, out Int32 Lane, out Int32 Index))
            {
                return Path;
            }

            Double[] Saved = SaveFill(Lanes);
            Double CutBcy = 0.0;
            Double Cursor = Lanes[Lane][Index].Order;
            Int32 Key = Keys[Lane];
            Boolean Have = false;
            Int32 Guard = 0;
            while ((CutBcy < (BowlBcy - 1e-6)) && (Guard < 100000))
            {
                Guard += 1;
                if (!TryNextWork(Lanes, Keys, Key, Cursor, Sign, Have, LaneM, RadiusM, out Int32 NextLane, out Int32 NextCell))
                {
                    break;
                }

                LaneCell Cell = Lanes[NextLane][NextCell];
                // Past the last fill this heading can reach. Spreading it would mean turning back.
                if (!FillReachable(FillLanes, FillKeys, Skip, Keys[NextLane], Cell.Order, Sign, LaneM, RadiusM))
                {
                    break;
                }

                Double Room = BowlBcy - CutBcy;
                Double Depth = Math.Min(Cell.LeftM, Math.Min(MaxDepthM, Room / Math.Max(CutPerM, 1e-9)));
                if (Depth <= LIFT_EPSILON_M)
                {
                    break;
                }

                Path.Add(new WorkStep(Cell.Bx, Cell.By, Keys[NextLane], Cell.Order, Depth));
                Cell.LeftM -= Depth;
                CutBcy += Depth * CutPerM;
                Cursor = Cell.Order;
                Key = Keys[NextLane];
                Have = true;
            }

            RestoreFill(Lanes, Saved);
            return Path;
        }

        /// <summary>
        /// Earliest cut in this direction that still has fill ahead by a forward join.
        /// A shorter lane wins when two cuts start together, so the next lane can be an S-bend.
        /// </summary>
        /// <param name="Lanes">Cut lanes.</param>
        /// <param name="Keys">Signed lane index of each cut lane.</param>
        /// <param name="FillLanes">Fill lanes.</param>
        /// <param name="FillKeys">Signed lane index of each fill lane.</param>
        /// <param name="Skip">Fill cells that are not available.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="Lane">Lane index of the cell.</param>
        /// <param name="Cell">Index of the cell in that lane.</param>
        /// <returns>False when no cut can reach a fill.</returns>
        private static Boolean TryFindCutStart
            (
            List<List<LaneCell>> Lanes,
            List<Int32> Keys,
            List<List<LaneCell>> FillLanes,
            List<Int32> FillKeys,
            HashSet<(Int32 Bx, Int32 By)> Skip,
            Double LaneM,
            Double RadiusM,
            Int32 Sign,
            out Int32 Lane,
            out Int32 Cell
            )
        {
            Lane = -1;
            Cell = -1;
            Double Best = 0.0;
            Double BestLeft = 0.0;
            for (Int32 LaneIndex = 0; LaneIndex < Lanes.Count; LaneIndex += 1)
            {
                List<LaneCell> Cells = Lanes[LaneIndex];
                Int32 StartIndex = -1;
                Double Start = 0.0;
                for (Int32 Index = 0; Index < Cells.Count; Index += 1)
                {
                    if (Cells[Index].LeftM <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    if (!FillReachable(FillLanes, FillKeys, Skip, Keys[LaneIndex], Cells[Index].Order, Sign, LaneM, RadiusM))
                    {
                        continue;
                    }

                    Double Key = Cells[Index].Order * Sign;
                    if ((StartIndex < 0) || (Key < Start))
                    {
                        Start = Key;
                        StartIndex = Index;
                    }
                }

                if (StartIndex < 0)
                {
                    continue;
                }

                Double Left = RunLeft(Cells, Sign, Cells[StartIndex].Order);
                Boolean Earlier = (Lane < 0) || (Start < (Best - 1e-6));
                Boolean SameStation = (Lane >= 0) && (Math.Abs(Start - Best) <= 1e-6) && (Left < (BestLeft - 1e-6));
                if (Earlier || SameStation)
                {
                    Best = Start;
                    BestLeft = Left;
                    Lane = LaneIndex;
                    Cell = StartIndex;
                }
            }

            return Lane >= 0;
        }

        /// <summary>
        /// True when some fill is still ahead of this station on the same lane,
        /// or at least one forward S-bend ahead on another lane.
        /// </summary>
        /// <param name="Lanes">Fill lanes.</param>
        /// <param name="Keys">Signed lane index of each fill lane.</param>
        /// <param name="Skip">Fill cells that are not available.</param>
        /// <param name="FromKey">Lane the machine is on.</param>
        /// <param name="FromStation">Station the machine has reached.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <returns>False when spreading would turn back.</returns>
        private static Boolean FillReachable
            (
            List<List<LaneCell>> Lanes,
            List<Int32> Keys,
            HashSet<(Int32 Bx, Int32 By)> Skip,
            Int32 FromKey,
            Double FromStation,
            Int32 Sign,
            Double LaneM,
            Double RadiusM
            )
        {
            for (Int32 Lane = 0; Lane < Lanes.Count; Lane += 1)
            {
                Boolean Same = Keys[Lane] == FromKey;
                Double Room = Same
                    ? (1e-4 + 1e-8)
                    : (TripGuidance.ForwardSBendM(Math.Abs(Keys[Lane] - FromKey) * LaneM, RadiusM) - 1e-6);
                if (FirstWithDepth(Lanes[Lane], FromStation, Sign, Room, Skip, out Int32 _))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// How far a lane still runs past a station, in the travel direction.
        /// </summary>
        /// <param name="Cells">Lane cells.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="Order">Station already reached.</param>
        /// <returns>Metres of lane still ahead. Zero at the last cell.</returns>
        private static Double RunLeft
            (
            List<LaneCell> Cells,
            Int32 Sign,
            Double Order
            )
        {
            Double End = Order * Sign;
            foreach (LaneCell Cell in Cells)
            {
                if (Cell.LeftM <= LIFT_EPSILON_M)
                {
                    continue;
                }

                Double Key = Cell.Order * Sign;
                if (Key > End)
                {
                    End = Key;
                }
            }

            return End - (Order * Sign);
        }

        /// <summary>
        /// Next cell in this direction. The same lane is taken while it still has cells.
        /// Another lane is taken only when its next cell is at least one S-bend ahead.
        /// A shorter remaining run wins when two lane changes start together.
        /// </summary>
        /// <param name="Lanes">Lanes to scan.</param>
        /// <param name="Keys">Signed lane index of each lane.</param>
        /// <param name="FromKey">Lane the trip is on. It need not be in <paramref name="Keys"/>.</param>
        /// <param name="Cursor">Station the trip has reached.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="Have">True after the first cell, so the same cell is not taken twice.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <param name="NextLane">Lane of the chosen cell.</param>
        /// <param name="NextCell">Index of the chosen cell.</param>
        /// <returns>False when nothing legal remains ahead.</returns>
        private static Boolean TryNextWork
            (
            List<List<LaneCell>> Lanes,
            List<Int32> Keys,
            Int32 FromKey,
            Double Cursor,
            Int32 Sign,
            Boolean Have,
            Double LaneM,
            Double RadiusM,
            out Int32 NextLane,
            out Int32 NextCell
            )
        {
            Int32 Same = -1;
            for (Int32 Index = 0; Index < Keys.Count; Index += 1)
            {
                if (Keys[Index] == FromKey)
                {
                    Same = Index;
                    break;
                }
            }

            if ((Same >= 0) && TrySameLane(Lanes[Same], Cursor, Sign, Have, out NextCell))
            {
                NextLane = Same;
                return true;
            }

            NextLane = -1;
            NextCell = -1;
            Double BestAhead = 0.0;
            Double BestLateral = 0.0;
            Double BestLeft = 0.0;
            for (Int32 Index = 0; Index < Lanes.Count; Index += 1)
            {
                if (Index == Same)
                {
                    continue;
                }

                Double Lateral = Math.Abs(Keys[Index] - FromKey) * LaneM;
                Double Room = TripGuidance.ForwardSBendM(Lateral, RadiusM);
                if (!FirstWithDepth(Lanes[Index], Cursor, Sign, Room - 1e-6, null, out Int32 Cell))
                {
                    continue;
                }

                LaneCell Chosen = Lanes[Index][Cell];
                Double Ahead = (Chosen.Order - Cursor) * Sign;
                Double Left = RunLeft(Lanes[Index], Sign, Chosen.Order);
                Boolean Closer = (NextLane < 0) || (Ahead < (BestAhead - 1e-6));
                Boolean SameSpot = (NextLane >= 0)
                    && (Math.Abs(Ahead - BestAhead) <= 1e-6)
                    && (Lateral < (BestLateral - 1e-6));
                Boolean SameSide = (NextLane >= 0)
                    && (Math.Abs(Ahead - BestAhead) <= 1e-6)
                    && (Math.Abs(Lateral - BestLateral) <= 1e-6)
                    && (Left < (BestLeft - 1e-6));
                if (Closer || SameSpot || SameSide)
                {
                    NextLane = Index;
                    NextCell = Cell;
                    BestAhead = Ahead;
                    BestLateral = Lateral;
                    BestLeft = Left;
                }
            }

            return NextLane >= 0;
        }

        /// <summary>
        /// Spreads forward from the end of the cuts until the load is placed or the lane runs out.
        /// One blade depth is taken from a cell, then the trip moves on.
        /// </summary>
        /// <param name="Lanes">Fill lanes. Depths are reduced.</param>
        /// <param name="Keys">Signed lane index of each fill lane.</param>
        /// <param name="FromKey">Lane the last cut was on.</param>
        /// <param name="Cursor">Station of the last cut.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="NeedBcy">Bank volume still to place.</param>
        /// <param name="MaxDepthM">Maximum depth per lift, in metres.</param>
        /// <param name="FillPerM">Bank BCY of one metre of fill on one cell.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="RadiusM">Turning radius in metres.</param>
        /// <param name="Fills">Receives the fill cells.</param>
        /// <returns>Fill bank BCY placed.</returns>
        private static Double WalkFills
            (
            List<List<LaneCell>> Lanes,
            List<Int32> Keys,
            Int32 FromKey,
            Double Cursor,
            Int32 Sign,
            Double NeedBcy,
            Double MaxDepthM,
            Double FillPerM,
            Double LaneM,
            Double RadiusM,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills
            )
        {
            Double Load = 0.0;
            Int32 Key = FromKey;
            Boolean Have = true;
            Int32 Guard = 0;
            while ((Load < (NeedBcy - 1e-6)) && (Guard < 100000))
            {
                Guard += 1;
                if (!TryNextWork(Lanes, Keys, Key, Cursor, Sign, Have, LaneM, RadiusM, out Int32 NextLane, out Int32 NextCell))
                {
                    break;
                }

                LaneCell Cell = Lanes[NextLane][NextCell];
                Double Room = NeedBcy - Load;
                Double Depth = Math.Min(Cell.LeftM, Math.Min(MaxDepthM, Room / Math.Max(FillPerM, 1e-9)));
                if (Depth <= LIFT_EPSILON_M)
                {
                    break;
                }

                AddFill(Fills, Cell.Bx, Cell.By, Depth);
                Cell.LeftM -= Depth;
                Load += Depth * FillPerM;
                Cursor = Cell.Order;
                Key = Keys[NextLane];
            }

            return Load;
        }

        /// <summary>
        /// Makes the cut and the fill the same bank volume.
        /// Extra cut is removed from the start of the list, which is the soil farthest from the spread.
        /// Extra fill is removed from the end, which the bowl did not have room to place.
        /// </summary>
        /// <param name="Cuts">Cut cells in travel order.</param>
        /// <param name="Fills">Fill cells in travel order.</param>
        /// <param name="CutBcy">Cut bank volume. Updated.</param>
        /// <param name="FillBcy">Fill bank volume. Updated.</param>
        /// <param name="CutPerM">Bank BCY of one metre of cut on one cell.</param>
        /// <param name="FillPerM">Bank BCY of one metre of fill on one cell.</param>
        private static void BalanceLoads
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills,
            ref Double CutBcy,
            ref Double FillBcy,
            Double CutPerM,
            Double FillPerM
            )
        {
            Int32 Index = 0;
            while ((CutBcy > (FillBcy + 1e-4)) && (Index < Cuts.Count))
            {
                Double Bank = Cuts[Index].DepthM * CutPerM;
                Double Extra = CutBcy - FillBcy;
                if (Bank <= (Extra + 1e-9))
                {
                    CutBcy -= Bank;
                    Cuts.RemoveAt(Index);
                }
                else
                {
                    Double Give = Extra / Math.Max(CutPerM, 1e-9);
                    Cuts[Index] = (Cuts[Index].Cell, Cuts[Index].DepthM - Give);
                    CutBcy = FillBcy;
                }
            }

            while ((FillBcy > (CutBcy + 1e-4)) && (Fills.Count > 0))
            {
                Int32 Last = Fills.Count - 1;
                Double Bank = Fills[Last].DepthM * FillPerM;
                Double Extra = FillBcy - CutBcy;
                if (Bank <= (Extra + 1e-9))
                {
                    FillBcy -= Bank;
                    Fills.RemoveAt(Last);
                }
                else
                {
                    Double Give = Extra / Math.Max(FillPerM, 1e-9);
                    Fills[Last] = (Fills[Last].Cell, Fills[Last].DepthM - Give);
                    FillBcy = CutBcy;
                }
            }
        }

        /// <summary>
        /// Sets blocked fill cells aside for one trial. The caller restores the saved depths.
        /// </summary>
        /// <param name="Lanes">Fill lanes.</param>
        /// <param name="Blocked">Cells that must not be spread on this trial.</param>
        private static void ZeroBlocked
            (
            List<List<LaneCell>> Lanes,
            HashSet<(Int32 Bx, Int32 By)> Blocked
            )
        {
            foreach (List<LaneCell> Cells in Lanes)
            {
                foreach (LaneCell Cell in Cells)
                {
                    if (Blocked.Contains((Cell.Bx, Cell.By)))
                    {
                        Cell.LeftM = 0.0;
                    }
                }
            }
        }

        /// <summary>
        /// Along-track distance from the first cell to the last, in metres.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Cuts">Cut cells.</param>
        /// <param name="Fills">Fill cells.</param>
        /// <returns>The span. Zero when there are no cells.</returns>
        private static Double AlongSpanM
            (
            Grid Grid,
            Feature Item,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills
            )
        {
            Double Low = Double.PositiveInfinity;
            Double High = Double.NegativeInfinity;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cuts)
            {
                Double Station = StationOf(Grid, Item, Entry.Cell.Bx, Entry.Cell.By);
                Low = Math.Min(Low, Station);
                High = Math.Max(High, Station);
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Fills)
            {
                Double Station = StationOf(Grid, Item, Entry.Cell.Bx, Entry.Cell.By);
                Low = Math.Min(Low, Station);
                High = Math.Max(High, Station);
            }

            if (Low > High)
            {
                return 0.0;
            }

            return High - Low;
        }

        /// <summary>
        /// Consumes a draft and records it as a pan trip.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Lanes">Fill lanes.</param>
        /// <param name="Stocks">Cut stocks.</param>
        /// <param name="Passes">Pan trips.</param>
        /// <param name="Signs">Travel sign of each pan trip.</param>
        /// <param name="Draft">Trip to commit.</param>
        private static void CommitPan
            (
            Grid Grid,
            Feature Item,
            List<List<LaneCell>> Lanes,
            List<Stock> Stocks,
            List<Pass> Passes,
            List<Int32> Signs,
            RoadTripDraft Draft
            )
        {
            ApplyDraft(Lanes, Stocks, Draft);
            Passes.Add(MakePass(Grid, Item, Draft.Cuts, Draft.Fills, Draft.LoadBcy, Draft.Sign));
            Signs.Add(Draft.Sign);
        }

        /// <summary>
        /// Builds a road-stage pass. Cell order is the order the trip drives.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad the join is measured along.</param>
        /// <param name="Cuts">Cut cells in travel order.</param>
        /// <param name="Fills">Fill cells in travel order.</param>
        /// <param name="LoadBcy">Matched load in BCY.</param>
        /// <param name="Sign">Travel sign. The lists are already in this order.</param>
        /// <returns>The pass.</returns>
        private static Pass MakePass
            (
            Grid Grid,
            Feature Item,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills,
            Double LoadBcy,
            Int32 Sign
            )
        {
            (Double X, Double Y) CutJoin = JoinPoint(Grid, Item, Cuts, Sign, true);
            (Double X, Double Y) FillJoin = JoinPoint(Grid, Item, Fills, Sign, false);
            (Double X, Double Y) CutStart = (Grid.CenterX[Cuts[0].Cell.By, Cuts[0].Cell.Bx], Grid.CenterY[Cuts[0].Cell.By, Cuts[0].Cell.Bx]);
            (Double X, Double Y) FillEnd = (Grid.CenterX[Fills[Fills.Count - 1].Cell.By, Fills[Fills.Count - 1].Cell.Bx], Grid.CenterY[Fills[Fills.Count - 1].Cell.By, Fills[Fills.Count - 1].Cell.Bx]);
            Double Haul = Distance(CutJoin.X, CutJoin.Y, FillJoin.X, FillJoin.Y);
            Pass Trip = new Pass(Cuts, Fills, LoadBcy, Haul, CutStart, CutJoin, FillJoin, FillEnd);
            Trip.Stage = HaulStage.Road;
            Trip.RibbonOrder = true;
            Trip.StackedLifts = true;
            return Trip;
        }

        /// <summary>
        /// The cut cell nearest the spread, or the fill cell the spread starts on.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Cells">Cells to scan.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="End">True for the last cell in the travel direction.</param>
        /// <returns>The cell centre.</returns>
        private static (Double X, Double Y) JoinPoint
            (
            Grid Grid,
            Feature Item,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Int32 Sign,
            Boolean End
            )
        {
            Int32 Chosen = 0;
            Double Best = StationOf(Grid, Item, Cells[0].Cell.Bx, Cells[0].Cell.By) * Sign;
            for (Int32 Index = 1; Index < Cells.Count; Index += 1)
            {
                Double Key = StationOf(Grid, Item, Cells[Index].Cell.Bx, Cells[Index].Cell.By) * Sign;
                if (End ? (Key > Best) : (Key < Best))
                {
                    Best = Key;
                    Chosen = Index;
                }
            }

            return (Grid.CenterX[Cells[Chosen].Cell.By, Cells[Chosen].Cell.Bx], Grid.CenterY[Cells[Chosen].Cell.By, Cells[Chosen].Cell.Bx]);
        }

        /// <summary>
        /// Subtracts a draft from the fill lanes and the cut stocks.
        /// </summary>
        /// <param name="Lanes">Fill lanes.</param>
        /// <param name="Stocks">Cut stocks.</param>
        /// <param name="Draft">Cells to consume.</param>
        private static void ApplyDraft
            (
            List<List<LaneCell>> Lanes,
            List<Stock> Stocks,
            RoadTripDraft Draft
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cut in Draft.Cuts)
            {
                Stock Source = FindStock(Stocks, Cut.Cell.Bx, Cut.Cell.By);
                if (Source != null)
                {
                    Source.LeftM = Math.Max(0.0, Source.LeftM - Cut.DepthM);
                }
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Draft.Fills)
            {
                LaneCell Target = FindLaneCell(Lanes, Fill.Cell.Bx, Fill.Cell.By);
                if (Target != null)
                {
                    Target.LeftM = Math.Max(0.0, Target.LeftM - Fill.DepthM);
                }
            }
        }

        /// <summary>
        /// Finds a stock cell.
        /// </summary>
        /// <param name="Stocks">Cut stocks.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <returns>The stock, or null.</returns>
        private static Stock FindStock
            (
            List<Stock> Stocks,
            Int32 Bx,
            Int32 By
            )
        {
            foreach (Stock Cell in Stocks)
            {
                if ((Cell.Bx == Bx) && (Cell.By == By))
                {
                    return Cell;
                }
            }

            return null;
        }

        /// <summary>
        /// Finds a fill cell.
        /// </summary>
        /// <param name="Lanes">Fill lanes.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <returns>The cell, or null.</returns>
        private static LaneCell FindLaneCell
            (
            List<List<LaneCell>> Lanes,
            Int32 Bx,
            Int32 By
            )
        {
            foreach (List<LaneCell> Cells in Lanes)
            {
                foreach (LaneCell Cell in Cells)
                {
                    if ((Cell.Bx == Bx) && (Cell.By == By))
                    {
                        return Cell;
                    }
                }
            }

            return null;
        }

        /// <summary>
        /// Folds a light trip into a same-heading pan trip that already drives those lanes
        /// and still has bowl room. A new lane is not inserted, because the join would turn back.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Passes">Pan trips.</param>
        /// <param name="Signs">Travel sign of each pan trip.</param>
        /// <param name="Draft">Light trip.</param>
        /// <param name="BowlBcy">Scraper capacity in BCY.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <returns>True when the cells were added to a pan trip.</returns>
        private static Boolean TryMerge
            (
            Grid Grid,
            Feature Item,
            List<Pass> Passes,
            List<Int32> Signs,
            RoadTripDraft Draft,
            Double BowlBcy,
            Double LaneM
            )
        {
            Double Half = Grid.GridSizeM * Constants.HALF_SCALE;
            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                if (Signs[Index] != Draft.Sign)
                {
                    continue;
                }

                if ((Passes[Index].LoadBcy + Draft.LoadBcy) > (BowlBcy + 1e-3))
                {
                    continue;
                }

                if (!HostCovers(Grid, Item, Passes[Index], Draft, LaneM, Half))
                {
                    continue;
                }

                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Passes[Index].CutCells);
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Passes[Index].FillCells);
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cut in Draft.Cuts)
                {
                    AddFill(Cuts, Cut.Cell.Bx, Cut.Cell.By, Cut.DepthM);
                }

                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Draft.Fills)
                {
                    AddFill(Fills, Fill.Cell.Bx, Fill.Cell.By, Fill.DepthM);
                }

                Passes[Index] = MakePass(
                    Grid,
                    Item,
                    Cuts,
                    Fills,
                    Passes[Index].LoadBcy + Draft.LoadBcy,
                    Draft.Sign);
                return true;
            }

            return false;
        }

        /// <summary>
        /// True when every cell of the light trip lies on a lane the host already drives,
        /// inside the host's along-track span.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Host">Pan trip that might absorb the cells.</param>
        /// <param name="Draft">Light trip.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="HalfM">Half a cell, so a cell on the span edge still counts.</param>
        /// <returns>False when the host would have to turn back to collect the cells.</returns>
        private static Boolean HostCovers
            (
            Grid Grid,
            Feature Item,
            Pass Host,
            RoadTripDraft Draft,
            Double LaneM,
            Double HalfM
            )
        {
            HashSet<Int32> CutKeys = new HashSet<Int32>();
            HashSet<Int32> FillKeys = new HashSet<Int32>();
            Double Low = Double.PositiveInfinity;
            Double High = Double.NegativeInfinity;
            CollectHost(Grid, Item, Host.CutCells, Draft.Sign, CutKeys, LaneM, ref Low, ref High);
            CollectHost(Grid, Item, Host.FillCells, Draft.Sign, FillKeys, LaneM, ref Low, ref High);
            if (Low > High)
            {
                return false;
            }

            return CellsFit(Grid, Item, Draft.Cuts, CutKeys, Draft.Sign, LaneM, Low, High, HalfM)
                && CellsFit(Grid, Item, Draft.Fills, FillKeys, Draft.Sign, LaneM, Low, High, HalfM);
        }

        /// <summary>
        /// Records the lane keys and the station span of one side of a host trip.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Cells">Host cells.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="Keys">Receives the lane keys.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="Low">Lowest station times the sign. Updated.</param>
        /// <param name="High">Highest station times the sign. Updated.</param>
        private static void CollectHost
            (
            Grid Grid,
            Feature Item,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Int32 Sign,
            HashSet<Int32> Keys,
            Double LaneM,
            ref Double Low,
            ref Double High
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                LanePose(Grid, Item, Entry.Cell.Bx, Entry.Cell.By, out Double Order, out Double Signed);
                Keys.Add(LaneIndex(Signed, LaneM));
                Double Key = Order * Sign;
                Low = Math.Min(Low, Key);
                High = Math.Max(High, Key);
            }
        }

        /// <summary>
        /// True when every cell is on one of the host lanes and inside the host span.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Cells">Cells to test.</param>
        /// <param name="Keys">Lane keys the host already drives.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <param name="Low">Host span start, station times the sign.</param>
        /// <param name="High">Host span end, station times the sign.</param>
        /// <param name="HalfM">Half a cell of slack.</param>
        /// <returns>False when a cell is on a new lane or outside the span.</returns>
        private static Boolean CellsFit
            (
            Grid Grid,
            Feature Item,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            HashSet<Int32> Keys,
            Int32 Sign,
            Double LaneM,
            Double Low,
            Double High,
            Double HalfM
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                LanePose(Grid, Item, Entry.Cell.Bx, Entry.Cell.By, out Double Order, out Double Signed);
                if (!Keys.Contains(LaneIndex(Signed, LaneM)))
                {
                    return false;
                }

                Double Key = Order * Sign;
                if ((Key < (Low - HalfM)) || (Key > (High + HalfM)))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Bank fill still sitting in the lanes.
        /// </summary>
        /// <param name="Lanes">Fill lanes.</param>
        /// <param name="FillPerM">Bank BCY of one metre of fill on one cell.</param>
        /// <returns>The remaining bank cubic yards.</returns>
        private static Double RemainingFillBcy
            (
            List<List<LaneCell>> Lanes,
            Double FillPerM
            )
        {
            Double Left = 0.0;
            foreach (List<LaneCell> Cells in Lanes)
            {
                foreach (LaneCell Cell in Cells)
                {
                    if (Cell.LeftM > LIFT_EPSILON_M)
                    {
                        Left += Cell.LeftM * FillPerM;
                    }
                }
            }

            return Left;
        }

        /// <summary>
        /// Hands leftover fill to the blade when a cut cell is within the shove limit along the bank.
        /// The pan could not make a forward trip out of these cells.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Lanes">Fill lanes still holding depth.</param>
        /// <param name="Stocks">Cut still available.</param>
        /// <param name="BladeCuts">Cut cells handed to the blade.</param>
        /// <param name="BladeBcy">Bank cubic yards handed to the blade.</param>
        /// <param name="FillPerM">Bank BCY of one metre of fill on one cell.</param>
        /// <param name="CutPerM">Bank BCY of one metre of cut on one cell.</param>
        /// <param name="BladeMaxM">Longest along-track blade shove, in metres.</param>
        private static void BladeLocalRemainder
            (
            Grid Grid,
            Feature Item,
            List<List<LaneCell>> Lanes,
            List<Stock> Stocks,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> BladeCuts,
            ref Double BladeBcy,
            Double FillPerM,
            Double CutPerM,
            Double BladeMaxM
            )
        {
            List<(Double Station, Stock Cell)> Cuts = new List<(Double Station, Stock Cell)>();
            foreach (Stock Source in Stocks)
            {
                if (Source.LeftM > LIFT_EPSILON_M)
                {
                    Cuts.Add((StationOf(Grid, Item, Source.Bx, Source.By), Source));
                }
            }

            Cuts.Sort((Left, Right) => Left.Station.CompareTo(Right.Station));
            // One shove can move the whole remaining depth. The nearest cut inside the limit is used.
            for (Int32 Pass = 0; Pass < 8; Pass += 1)
            {
                Boolean Moved = false;
                foreach (List<LaneCell> Cells in Lanes)
                {
                    foreach (LaneCell Fill in Cells)
                    {
                        if (Fill.LeftM <= LIFT_EPSILON_M)
                        {
                            continue;
                        }

                        Double FillStation = StationOf(Grid, Item, Fill.Bx, Fill.By);
                        if (!NearestCut(Cuts, Fill.Bx, Fill.By, FillStation, BladeMaxM, out Stock Cut, out Double Apart))
                        {
                            continue;
                        }

                        Double FillDepth = Fill.LeftM;
                        Double CutDepth = Math.Min(Cut.LeftM, (FillDepth * FillPerM) / Math.Max(CutPerM, 1e-9));
                        FillDepth = Math.Min(FillDepth, (CutDepth * CutPerM) / Math.Max(FillPerM, 1e-9));
                        if ((FillDepth <= LIFT_EPSILON_M) || (CutDepth <= LIFT_EPSILON_M))
                        {
                            continue;
                        }

                        // Apart is only a guard. The search already rejected a shove past the limit.
                        if (Apart > (BladeMaxM + 1e-6))
                        {
                            continue;
                        }

                        Fill.LeftM -= FillDepth;
                        Cut.LeftM -= CutDepth;
                        BladeBcy += FillDepth * FillPerM;
                        BladeCuts.Add(((Cut.Bx, Cut.By), CutDepth));
                        Moved = true;
                    }
                }

                if (!Moved)
                {
                    return;
                }
            }
        }

        /// <summary>
        /// Closest cut that still has depth and sits within the shove limit.
        /// The list is sorted by station, so the search starts at the fill and walks outward.
        /// </summary>
        /// <param name="Cuts">Cut cells sorted by station.</param>
        /// <param name="Bx">Fill column. The same cell is not a source.</param>
        /// <param name="By">Fill row.</param>
        /// <param name="FillStation">Fill station in metres.</param>
        /// <param name="BladeMaxM">Longest shove, in metres.</param>
        /// <param name="Cut">The chosen stock.</param>
        /// <param name="Apart">Along-track distance in metres.</param>
        /// <returns>False when no cut is in range.</returns>
        private static Boolean NearestCut
            (
            List<(Double Station, Stock Cell)> Cuts,
            Int32 Bx,
            Int32 By,
            Double FillStation,
            Double BladeMaxM,
            out Stock Cut,
            out Double Apart
            )
        {
            Cut = null;
            Apart = 0.0;
            Int32 Right = LowerStation(Cuts, FillStation);
            Int32 Left = Right - 1;
            while ((Left >= 0) || (Right < Cuts.Count))
            {
                Double LeftApart = (Left >= 0) ? (FillStation - Cuts[Left].Station) : Double.PositiveInfinity;
                Double RightApart = (Right < Cuts.Count) ? (Cuts[Right].Station - FillStation) : Double.PositiveInfinity;
                Boolean UseLeft = LeftApart <= RightApart;
                Double Gap = UseLeft ? LeftApart : RightApart;
                if (Gap > (BladeMaxM + 1e-6))
                {
                    return false;
                }

                Stock Source = UseLeft ? Cuts[Left].Cell : Cuts[Right].Cell;
                if (UseLeft)
                {
                    Left -= 1;
                }
                else
                {
                    Right += 1;
                }

                if ((Source.LeftM <= LIFT_EPSILON_M) || ((Source.Bx == Bx) && (Source.By == By)))
                {
                    continue;
                }

                Cut = Source;
                Apart = Gap;
                return true;
            }

            return false;
        }

        /// <summary>
        /// First index whose station is at least the bound.
        /// </summary>
        /// <param name="Cuts">Cut cells sorted by station.</param>
        /// <param name="Bound">Station bound in metres.</param>
        /// <returns>The insertion index.</returns>
        private static Int32 LowerStation
            (
            List<(Double Station, Stock Cell)> Cuts,
            Double Bound
            )
        {
            Int32 Low = 0;
            Int32 High = Cuts.Count;
            while (Low < High)
            {
                Int32 Mid = (Low + High) / 2;
                if (Cuts[Mid].Station < Bound)
                {
                    Low = Mid + 1;
                }
                else
                {
                    High = Mid;
                }
            }

            return Low;
        }

        /// <summary>
        /// Reports fill that was neither hauled nor handed to the blade.
        /// </summary>
        /// <param name="Surface">Surface that receives the warning.</param>
        /// <param name="Lanes">Fill lanes still holding depth.</param>
        /// <param name="FillPerM">Bank BCY of one metre of fill on one cell.</param>
        /// <param name="MinLoadPct">Pan-trip threshold.</param>
        /// <param name="BladeMaxM">Blade shove limit, in metres.</param>
        private static void WarnUnmoved
            (
            RoadPadSurface Surface,
            List<List<LaneCell>> Lanes,
            Double FillPerM,
            Double MinLoadPct,
            Double BladeMaxM
            )
        {
            Double Left = 0.0;
            foreach (List<LaneCell> Cells in Lanes)
            {
                foreach (LaneCell Cell in Cells)
                {
                    if (Cell.LeftM > LIFT_EPSILON_M)
                    {
                        Left += Cell.LeftM * FillPerM;
                    }
                }
            }

            if (Left <= 0.05)
            {
                return;
            }

            Surface.Warnings.Add(String.Format(
                CultureInfo.InvariantCulture,
                "Road stage left {0:F2} BCY. A pan trip would be under {1:F0}% of the bowl, the shove is over {2:F0} ft, and no same-heading trip has room.",
                Left,
                MinLoadPct * 100.0,
                BladeMaxM / Constants.FEET_TO_METERS));
        }
    }
}
