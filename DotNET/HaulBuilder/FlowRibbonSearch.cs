using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

using static HaulBuilder.Obstacles;

namespace HaulBuilder
{
    /// <summary>
    /// Builds each load from the crossover, where remaining cut meets fill.
    /// The search works backwards along the Poisson flow. That line is the
    /// priority. Once no arrow still fills a bowl, the blade may leave the
    /// arrow by 10 percent, and that cone opens in the same steps while a
    /// full bowl can still be made. Where a flow line cannot be followed, the ground is a ridge,
    /// and the cut follows the ridge only while that soil still drains back
    /// through this mouth. Soil that runs to another fill is another load.
    /// The fill follows the flow on into the basin. Cells already at grade
    /// are driven across
    /// with the blade grading, and the dirt stays in the bowl until the next
    /// cell that is still low. Once that still cannot fill a bowl, the fill
    /// may cross a small leftover ridge and place the rest of the bowl in
    /// the basin the arrow reaches. The haul is still only the gap to the first
    /// of those low cells. As the field is levelled the crossover
    /// moves and the finished band between cut and fill gets wider.
    /// Turning-circle guidance then drives the path. The search is a state
    /// machine. When the cluster window is on, each trip is the cheapest
    /// seconds per yard among the next flow trip, the cluster plan, and the
    /// thin-cut plan. Hauling stops when none of those is under the ceiling,
    /// and correcting starts. A window of zero keeps the flow ribbon.
    /// </summary>
    static public class FlowRibbonSearch
    {
        private const Double VOLUME_TOLERANCE_BCY = 1e-6;
        private const Double DEPTH_TOLERANCE_M = 1e-6;
        /// <summary>Step along the streamline, in cells. Short enough to visit every cell.</summary>
        private const Double TRACE_STEP_CELLS = 0.45;
        /// <summary>Empty steps in a row that end a ribbon.</summary>
        private const Int32 MISS_GAP = 8;
        /// <summary>Steps before a trace gives up. Stops a loop on a flat arrow.</summary>
        private const Int32 MAX_TRACE_STEPS = 4000;
        /// <summary>Neighbor averages applied to the arrow field so the ribbon steers smoothly.</summary>
        private const Int32 SMOOTH_PASSES = 4;
        /// <summary>Cut and fill on one load must match tighter than the ledger check.</summary>
        private const Double MATCH_TOLERANCE_BCY = 1e-12;
        /// <summary>
        /// Strips one load may take. A full bowl can be several trips back
        /// to dirt that was not beside the first strip.
        /// </summary>
        private const Int32 MAX_STRIPS = 40;
        /// <summary>Seeds tried while filling one bowl. A seed that grows nothing is skipped.</summary>
        private const Int32 MAX_STRIP_TRIES = 80;
        /// <summary>
        /// How far, in cells, the next cut strip may sit from dirt already on this load.
        /// A turnaround across this gap is shorter than hauling a half-empty bowl.
        /// </summary>
        private const Double ADJACENT_CELLS = 12.0;
        /// <summary>
        /// Fill may be gathered anywhere still open. The machine is already going
        /// there to dump, and a full bowl saves the return trip.
        /// </summary>
        private const Double FILL_REACH_CELLS = 12.0;
        /// <summary>
        /// Fraction of scraper capacity each load takes before it hauls.
        /// The bowl is filled even when the next strip means driving back.
        /// </summary>
        private const Double FULL_BOWL = 1.0;
        /// <summary>
        /// Minimum share of a cell the blade must cover before the cell is cut or filled.
        /// The blade is one cell wide. On a diagonal it covers about a quarter of each
        /// cell beside the corner, and those cells are the continuous strip. A cell
        /// below this share was only nicked, and taking it paints a splotch.
        /// </summary>
        private const Double MIN_CELL_COVERAGE = 0.25;
        /// <summary>Samples along each cell edge when measuring how much of it the blade covers.</summary>
        private const Int32 COVERAGE_SAMPLES = 8;
        /// <summary>
        /// Distance, in cells, from a cell center to the farthest coverage sample.
        /// Samples sit half a step in from the edge, so the true corner is
        /// (0.5 - 0.5/8) * sqrt(2), about 0.619. A hair past that corner keeps
        /// a borderline cell on the sample grid. A center closer than the blade
        /// minus this reach is fully covered, and a center farther than the
        /// blade plus this reach covers nothing.
        /// </summary>
        private const Double COVERAGE_SAMPLE_REACH_CELLS = 0.619;
        /// <summary>
        /// Minimum alignment of a step with the flow. About 32 degrees.
        /// A wider step leaves the streamline and fills the pocket as a blob.
        /// </summary>
        private const Double FORWARD_DOT = 0.85;
        /// <summary>
        /// Minimum alignment with the step just taken. About 45 degrees.
        /// A harder turn folds the ribbon back into ground it already passed.
        /// </summary>
        private const Double TURN_DOT = 0.70;
        /// <summary>
        /// Widest a load may grow, in cells, once a second strip is added.
        /// One blade plus the pass beside it. Wider than this is a splotch.
        /// </summary>
        private const Double MAX_RIBBON_WIDTH_CELLS = 3.0;

        /// <summary>
        /// How far a pass must travel, in cells, before a step back toward
        /// its start counts as turning around. A shorter wiggle is still the
        /// same strip.
        /// </summary>
        private const Double RETURN_CLEAR_CELLS = 1.5;

        /// <summary>
        /// Reversing steps allowed on the field edge. That is a headland corner.
        /// More than this is a second pass back along the boundary.
        /// </summary>
        private const Int32 HEADLAND_REVERSE_STEPS = 6;

        /// <summary>
        /// How close, in cells, downhill flow must pass a mouth before that
        /// soil belongs to the load. Farther than this is a different haul.
        /// </summary>
        private const Int32 SAME_MOUTH_CELLS = 2;

        /// <summary>
        /// How far, in cells, a short bowl may reach beside its upstream end
        /// for more cut. Farther than this is a separate haul.
        /// </summary>
        private const Double TOPUP_REACH_CELLS = 8.0;

        /// <summary>
        /// Extra upstream strips a short bowl may add. Each one still drains
        /// through the same mouth.
        /// </summary>
        private const Int32 TOPUP_STRIPS = 4;

        /// <summary>
        /// How far the blade may first leave the arrow once no strict ribbon
        /// fills the bowl. The step keeps at least 90 percent of its alignment
        /// with the flow, about 26 degrees.
        /// </summary>
        private const Double FLOW_SLACK_STEP = 0.10;

        /// <summary>
        /// Widest the arrow may be left. The step can turn until it is square
        /// to the flow. A wider turn would drive back against the arrow.
        /// </summary>
        private const Double FLOW_SLACK_LIMIT = 1.0;

        /// <summary>
        /// Trace steps of still-high ground a fill may cross once no cleaner
        /// ribbon fills the bowl. Sixteen steps is about seven cells. That is
        /// the leftover ridge between a pocket and the basin the arrow drains
        /// into. A longer high run still stops the fill. An obstacle does not
        /// count as a ridge.
        /// </summary>
        private const Int32 RIDGE_CROSS_STEPS = 16;

        /// <summary>
        /// Deferred-work drops depth at or below a nanometre. On a large field
        /// that dust sums past the geometry tolerance and is still not a load.
        /// </summary>
        private const Double IMBALANCE_TOLERANCE_BCY = 1e-3;

        /// <summary>
        /// Allocates the ledger as flow ribbons.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Flow">Transport flow field. Downstream points from cut toward fill.</param>
        /// <param name="Ledger">Work to allocate.</param>
        /// <param name="Axis">Recorded on the result. The ribbon does not use lane axes.</param>
        /// <param name="MaximumLoadBcy">Scraper capacity in bank cubic yards.</param>
        /// <param name="DeferEdgeWork">When true, leftover edge soil may stay deferred.</param>
        /// <param name="Obstacles">Optional exclusion obstacles.</param>
        /// <param name="AllowAnyDeferredRemainder">
        /// When true, unpaired leftovers are allowed beyond the material imbalance.
        /// </param>
        /// <param name="BladeWidthM">Blade width in metres. Zero uses the default blade.</param>
        /// <param name="TurningCircleFt">
        /// Turning circle diameter in feet. Zero uses the default circle.
        /// The field edge may turn this tight. A bend inside a cut or a fill
        /// uses a wider radius, because a tight turn is slow.
        /// </param>
        /// <param name="ClusterModeAverageWindow">
        /// A positive window, with a positive fraction, prices each trip
        /// against clusters and thin-cut. Zero keeps the flow ribbon for the
        /// whole field. The count is no longer a trailing mean.
        /// </param>
        /// <param name="ClusterModeLoadFraction">
        /// Must be positive before the cost selection runs. Zero never leaves hauling.
        /// </param>
        /// <param name="StopAfterClusterTrips">
        /// When greater than zero, cluster hauling stops after this many trips
        /// and thin-cut does not start. Zero runs the normal cluster loop.
        /// </param>
        /// <param name="Clock">
        /// Speeds and efficiency for seconds per yard. Null uses the machine defaults.
        /// </param>
        /// <param name="CostBaselineTrips">
        /// Flow trips that fix the median. Zero uses the production count of 500.
        /// </param>
        /// <returns>The ribbon loads and the ledger check.</returns>
        static public HaulOptimizationResult Optimize
            (
            Grid Grid,
            FlowField Flow,
            HaulWorkLedger Ledger,
            HaulLaneBuilder.LaneAxis Axis,
            Double MaximumLoadBcy,
            Boolean DeferEdgeWork = false,
            ObstacleField? Obstacles = null,
            Boolean AllowAnyDeferredRemainder = false,
            Double BladeWidthM = 0.0,
            Double TurningCircleFt = Constants.DEFAULT_TURNING_CIRCLE_FT,
            Int32 ClusterModeAverageWindow = 0,
            Double ClusterModeLoadFraction = 0.0,
            Int32 StopAfterClusterTrips = 0,
            TimeParams? Clock = null,
            Int32 CostBaselineTrips = 0
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (Flow == null) throw new ArgumentNullException(nameof(Flow));
            if (Ledger == null) throw new ArgumentNullException(nameof(Ledger));
            if (MaximumLoadBcy <= 0.0) throw new ArgumentOutOfRangeException(nameof(MaximumLoadBcy));
            if (BladeWidthM <= 0.0)
            {
                BladeWidthM = Constants.DEFAULT_BLADE_WIDTH_MM / Constants.MILLIMETERS_PER_METER;
            }
            // A zero circle is the machine default. The radius is half the diameter.
            if (TurningCircleFt <= 0.0)
            {
                TurningCircleFt = Constants.DEFAULT_TURNING_CIRCLE_FT;
            }
            Double MinimumRadiusM = TurningCircleFt
                * Constants.FEET_TO_METERS
                * Constants.HALF_SCALE;
            Double MaxDepthM = Grid.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            Double[,] CutLeft = (Double[,])Ledger.RequiredCutDepthM.Clone();
            Double[,] FillLeft = (Double[,])Ledger.RequiredFillDepthM.Clone();
            (Double[,] Fx, Double[,] Fy) = Smooth(Flow, Grid);
            ObstacleField Active = Obstacles ?? EMPTY_OBSTACLES;
            List<Pass> Passes = new List<Pass>();
            HashSet<(Int32 Bx, Int32 By)> Blocked = new HashSet<(Int32 Bx, Int32 By)>();
            // One downhill line that runs out of soil before the bowl is full.
            HashSet<(Int32 Bx, Int32 By)> ShortMouths = new HashSet<(Int32 Bx, Int32 By)>();
            // A line that comes back on itself. It waits until no straight strip remains.
            HashSet<(Int32 Bx, Int32 By)> FoldedMouths = new HashSet<(Int32 Bx, Int32 By)>();
            // 0 takes a full bowl on one strip. 1 takes a short strip. 2 is the
            // end of the field, where a fold is accepted so the dirt is not abandoned.
            Int32 Phase = 0;
            // 0 follows the arrow. It opens only after that cone cannot fill a bowl.
            Double FlowSlack = 0.0;
            // A leftover ridge on the arrow is crossed only after that cone
            // cannot fill a bowl either.
            Boolean CrossRidges = false;
            Boolean FinishLeftovers = !DeferEdgeWork;
            Double AssignedBcy = 0.0;
            // Hauling until the trailing mean falls. Thin-cut and correcting build
            // no flow loads, so the residual flow pass must not scrape the dirt they leave.
            // Cluster hauling runs after this loop, once hauling has handed off.
            HaulSearchState State = HaulSearchState.Hauling;
            Int32 ClusterModeStartsAtRoute = 0;
            Int32 ThinCutStartsAtRoute = 0;
            Int32 CorrectingStartsAtRoute = 0;
            // A sharp turn on this ring is a headland. The same turn inside the field is a double-back.
            Boolean[,] FieldEdge = FieldEdgeCells(Grid);
            Int32 Guard = 0;
            // A refused mouth does not take a cell, and the end of the field
            // walks the leftovers a second time. One pass per cell is not enough.
            Int32 GuardLimit = ((Grid.Width * Grid.Height) + 2) * 3;
            // A positive window prices every trip. Zero stays on the ribbon.
            Boolean SelectByCost = (ClusterModeAverageWindow >= 1)
                && (ClusterModeLoadFraction > 0.0);
            TimeParams CycleClock = Clock ?? DefaultClock();
            if (CostBaselineTrips < 1)
            {
                CostBaselineTrips = StageCost.BASELINE_FLOW_TRIPS;
            }
            HaulCostSearch Cost = null;
            if (SelectByCost)
            {
                (Double[,] ArrowX, Double[,] ArrowY) CostArrows = SmoothedArrows(Flow, Grid);
                Cost = new HaulCostSearch(CycleClock, CostBaselineTrips, CostArrows.ArrowX, CostArrows.ArrowY);
            }
            Int32 Rounds = 0;
            while ((State == HaulSearchState.Hauling)
                && (Rounds < GuardLimit)
                && (Sum(CutLeft, Grid) > DEPTH_TOLERANCE_M)
                && (Sum(FillLeft, Grid) > DEPTH_TOLERANCE_M))
            {
                Rounds += 1;
                Pass Next = TryNextFlowPass(
                    Grid,
                    Flow,
                    Ledger,
                    MaximumLoadBcy,
                    BladeWidthM,
                    MinimumRadiusM,
                    MaxDepthM,
                    CutLeft,
                    FillLeft,
                    Fx,
                    Fy,
                    Active,
                    FinishLeftovers,
                    FieldEdge,
                    GuardLimit,
                    Blocked,
                    ShortMouths,
                    FoldedMouths,
                    ref Phase,
                    ref FlowSlack,
                    ref CrossRidges,
                    ref Guard);
                if (!SelectByCost)
                {
                    if (Next == null)
                    {
                        break;
                    }
                    Next.Stage = HaulStage.Flow;
                    Passes.Add(Next);
                    AssignedBcy += Next.LoadBcy;
                    // A zero window never enters this branch's handoff. The check stays
                    // so a direct caller that turns the fraction off still refunds a short mean.
                    if (ClusterMode.TrailingMeanIsUnder(
                        Passes,
                        MaximumLoadBcy,
                        ClusterModeAverageWindow,
                        ClusterModeLoadFraction))
                    {
                        Refund(Next.CutCells, CutLeft);
                        Refund(Next.FillCells, FillLeft);
                        Passes.RemoveAt(Passes.Count - 1);
                        AssignedBcy -= Next.LoadBcy;
                        State = HaulSearchState.Clusters;
                        ClusterModeStartsAtRoute = Passes.Count + 1;
                        break;
                    }
                    continue;
                }
                Int32 Before = Passes.Count;
                HaulCostSearch.CostStep Step = Cost.Take(
                    Next,
                    Grid,
                    Ledger,
                    CutLeft,
                    FillLeft,
                    MaximumLoadBcy,
                    Passes,
                    StopAfterClusterTrips);
                if (Step == HaulCostSearch.CostStep.Stop)
                {
                    State = HaulSearchState.Correcting;
                    CorrectingStartsAtRoute = Passes.Count + 1;
                    break;
                }
                if (Passes.Count > Before)
                {
                    Pass Taken = Passes[Passes.Count - 1];
                    AssignedBcy += Taken.LoadBcy;
                    if ((Taken.Stage == HaulStage.Clusters) && (ClusterModeStartsAtRoute == 0))
                    {
                        ClusterModeStartsAtRoute = Passes.Count;
                    }
                    if ((Taken.Stage == HaulStage.ThinCut) && (ThinCutStartsAtRoute == 0))
                    {
                        ThinCutStartsAtRoute = Passes.Count;
                    }
                }
                if (Step == HaulCostSearch.CostStep.Picture)
                {
                    State = HaulSearchState.Clusters;
                    break;
                }
                if (Passes.Count == Before)
                {
                    State = HaulSearchState.Correcting;
                    CorrectingStartsAtRoute = Passes.Count + 1;
                    Cost.RememberLeft(CutLeft, FillLeft, Grid, Ledger);
                    break;
                }
            }
            if (SelectByCost && (State == HaulSearchState.Hauling))
            {
                // The cut or the fill is gone. Correcting still covers what remains.
                State = HaulSearchState.Correcting;
                CorrectingStartsAtRoute = Passes.Count + 1;
                Cost.RememberLeft(CutLeft, FillLeft, Grid, Ledger);
            }
            if (!SelectByCost && (State == HaulSearchState.Clusters))
            {
                (Double[,] ArrowX, Double[,] ArrowY) Arrows = SmoothedArrows(Flow, Grid);
                ClusterHaulStop Stop = ClusterHauling.Run(
                    Grid,
                    Ledger,
                    CutLeft,
                    FillLeft,
                    MaximumLoadBcy,
                    Passes,
                    Arrows.ArrowX,
                    Arrows.ArrowY,
                    StopAfterClusterTrips);
                // The first routes are already on the list. Thin-cut stays closed.
                if (Stop.Reason == ClusterHauling.PICTURE_STOP)
                {
                    State = HaulSearchState.Clusters;
                }
                else
                {
                    State = ClusterHauling.Handoff(Stop);
                }
                if (State == HaulSearchState.ThinCut)
                {
                    ThinCutStartsAtRoute = Passes.Count + 1;
                    ThinCutStop ThinStop = ThinCut.Run(
                        Grid, Ledger, CutLeft, FillLeft, MaximumLoadBcy, Passes, Arrows.ArrowX, Arrows.ArrowY);
                    State = ThinCut.Handoff(ThinStop);
                    if (State == HaulSearchState.Correcting)
                    {
                        // Correcting is the last stage. A short field still gets the coverage.
                        CorrectingStartsAtRoute = Passes.Count + 1;
                        Int32 Mark = Passes.Count;
                        Correcting.Apply(
                            Grid,
                            Flow,
                            Ledger,
                            CutLeft,
                            FillLeft,
                            MaximumLoadBcy,
                            BladeWidthM,
                            Passes);
                        for (Int32 Index = Mark; Index < Passes.Count; Index += 1)
                        {
                            Passes[Index].Stage = HaulStage.Correcting;
                        }
                    }
                }
            }
            else if (SelectByCost && (State == HaulSearchState.Correcting))
            {
                if (!Cost.HasLeft)
                {
                    Cost.RememberLeft(CutLeft, FillLeft, Grid, Ledger);
                }
                (Double[,] ArrowX, Double[,] ArrowY) Arrows = SmoothedArrows(Flow, Grid);
                Int32 Mark = Passes.Count;
                Correcting.Apply(
                    Grid,
                    Flow,
                    Ledger,
                    CutLeft,
                    FillLeft,
                    MaximumLoadBcy,
                    BladeWidthM,
                    Passes);
                for (Int32 Index = Mark; Index < Passes.Count; Index += 1)
                {
                    Passes[Index].Stage = HaulStage.Correcting;
                }
            }
            IReadOnlyList<HaulDeferredWork> Deferred = Ledger.BuildDeferredWork(
                Passes,
                DeferEdgeWork ? "FIELD_EDGE_HEADING" : "MATERIAL_IMBALANCE");
            Double UnmatchedCutBcy = Deferred
                .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
                .Sum(Entry => Entry.VolumeBcy);
            Double UnmatchedFillBcy = Deferred
                .Where(Entry => Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
                .Sum(Entry => Entry.VolumeBcy);
            if (!DeferEdgeWork && !AllowAnyDeferredRemainder)
            {
                Double ExpectedCutBcy = Math.Max(0.0, Ledger.TotalCutBcy - Ledger.TotalFillBcy);
                Double ExpectedFillBcy = Math.Max(0.0, Ledger.TotalFillBcy - Ledger.TotalCutBcy);
                if ((Math.Abs(UnmatchedCutBcy - ExpectedCutBcy) > IMBALANCE_TOLERANCE_BCY)
                    || (Math.Abs(UnmatchedFillBcy - ExpectedFillBcy) > IMBALANCE_TOLERANCE_BCY))
                {
                    throw new InvalidOperationException(
                        String.Format(
                            System.Globalization.CultureInfo.InvariantCulture,
                            "The flow ribbon left work deferred beyond the exact material imbalance. cut={0:F3} fill={1:F3} expectedCut={2:F3} expectedFill={3:F3} passes={4}",
                            UnmatchedCutBcy,
                            UnmatchedFillBcy,
                            ExpectedCutBcy,
                            ExpectedFillBcy,
                            Passes.Count));
                }
            }
            PassResult PassResult = new PassResult(Passes, 1, UnmatchedCutBcy, UnmatchedFillBcy);
            HaulPlanValidation Validation = Ledger.ValidateComplete(
                Passes,
                MaximumLoadBcy,
                MaxDepthM,
                AllowDeferredWork: true);
            HaulOptimizationResult Result = new HaulOptimizationResult(
                PassResult,
                Axis,
                Passes.Count,
                Passes.Count,
                AssignedBcy,
                UnmatchedCutBcy,
                UnmatchedFillBcy,
                Deferred,
                Validation);
            Result.State = State;
            Result.EnteredClusterMode = State != HaulSearchState.Hauling;
            Result.ClusterModeStartsAtRoute = ClusterModeStartsAtRoute;
            if (CorrectingStartsAtRoute > 0)
            {
                // The coverage residual. Cap spread is not a haul, so the picture uses this surface.
                Double[,] Surface = new Double[Grid.Height, Grid.Width];
                for (Int32 Row = 0; Row < Grid.Height; Row += 1)
                {
                    for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                    {
                        Surface[Row, Col] = Grid.Valid[Row, Col]
                            ? CutLeft[Row, Col] - FillLeft[Row, Col]
                            : Double.NaN;
                    }
                }
                Result.CorrectingSurfaceM = Surface;
            }
            Result.ThinCutStartsAtRoute = ThinCutStartsAtRoute;
            Result.CorrectingStartsAtRoute = CorrectingStartsAtRoute;
            if (Cost != null)
            {
                Result.CeilingCutLeftBcy = Cost.CutLeftBcy;
                Result.CeilingFillLeftBcy = Cost.FillLeftBcy;
                Result.StoppedOnCeiling = Cost.StoppedOnCeiling;
            }
            return Result;
        }

        /// <summary>
        /// The next trip the mouth search would take, with its dirt already off the field.
        /// Null when the ribbon has no trip on this residual. Mouths and phase stay put.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Flow">Poisson field. Headings are read from it.</param>
        /// <param name="Ledger">Yard factors.</param>
        /// <param name="MaximumLoadBcy">Pan capacity.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="MinimumRadiusM">Turning radius in metres.</param>
        /// <param name="MaxDepthM">Deepest lift in metres.</param>
        /// <param name="CutLeft">Remaining cut. The returned trip has been subtracted.</param>
        /// <param name="FillLeft">Remaining fill. The returned trip has been subtracted.</param>
        /// <param name="Fx">Smoothed flow east.</param>
        /// <param name="Fy">Smoothed flow north.</param>
        /// <param name="Active">Obstacles.</param>
        /// <param name="FinishLeftovers">True when a fold may be kept at the end of the field.</param>
        /// <param name="FieldEdge">Headland cells.</param>
        /// <param name="GuardLimit">Failed mouth attempts before the search gives up.</param>
        /// <param name="Blocked">Mouths that grew no cut.</param>
        /// <param name="ShortMouths">Mouths that could not fill a bowl.</param>
        /// <param name="FoldedMouths">Mouths that looped.</param>
        /// <param name="Phase">0 full strips, 1 short strips, 2 folds.</param>
        /// <param name="FlowSlack">How far the blade may leave the arrow.</param>
        /// <param name="CrossRidges">True when a leftover ridge may be crossed.</param>
        /// <param name="Guard">Mouth attempts so far.</param>
        /// <returns>The trip, or null when the ribbon is finished for now.</returns>
        private static Pass TryNextFlowPass
            (
            Grid Grid,
            FlowField Flow,
            HaulWorkLedger Ledger,
            Double MaximumLoadBcy,
            Double BladeWidthM,
            Double MinimumRadiusM,
            Double MaxDepthM,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double[,] Fx,
            Double[,] Fy,
            ObstacleField Active,
            Boolean FinishLeftovers,
            Boolean[,] FieldEdge,
            Int32 GuardLimit,
            HashSet<(Int32 Bx, Int32 By)> Blocked,
            HashSet<(Int32 Bx, Int32 By)> ShortMouths,
            HashSet<(Int32 Bx, Int32 By)> FoldedMouths,
            ref Int32 Phase,
            ref Double FlowSlack,
            ref Boolean CrossRidges,
            ref Int32 Guard
            )
        {
            while ((Guard < GuardLimit)
                && (Sum(CutLeft, Grid) > DEPTH_TOLERANCE_M)
                && (Sum(FillLeft, Grid) > DEPTH_TOLERANCE_M))
            {
                Guard += 1;
                // The fill still waiting is the transport target. Recomputed
                // each load because the basin shrinks as dirt arrives.
                (Double X, Double Y)? Centroid = FillCentroid(FillLeft, Grid);
                if (!Centroid.HasValue) return null;
                // Full strips first. A short line waits. A fold waits until the end.
                HashSet<(Int32 Bx, Int32 By)> SkipMouths = new HashSet<(Int32 Bx, Int32 By)>(Blocked);
                if (Phase < 2)
                {
                    foreach ((Int32 Bx, Int32 By) Mouth in FoldedMouths)
                    {
                        SkipMouths.Add(Mouth);
                    }
                }
                if (Phase < 1)
                {
                    foreach ((Int32 Bx, Int32 By) Mouth in ShortMouths)
                    {
                        SkipMouths.Add(Mouth);
                    }
                }
                (Int32 Bx, Int32 By)? Seed = DownstreamMouth(
                    CutLeft,
                    FillLeft,
                    Fx,
                    Fy,
                    Grid,
                    Active,
                    SkipMouths,
                    Centroid.Value,
                    Ledger.CutBcyPerDepthM,
                    MaximumLoadBcy,
                    MaxDepthM,
                    BladeWidthM,
                    Phase == 0,
                    FieldEdge,
                    MinimumRadiusM,
                    FlowSlack);
                if (!Seed.HasValue)
                {
                    // No full bowl at this allowance. Widen the cone before
                    // spending dirt on a short load. A ridge is not crossed
                    // until that cone has already failed once.
                    if ((Phase == 0) && (FlowSlack < FLOW_SLACK_LIMIT))
                    {
                        FlowSlack = Math.Min(FLOW_SLACK_LIMIT, FlowSlack + FLOW_SLACK_STEP);
                        ShortMouths.Clear();
                        FoldedMouths.Clear();
                        Blocked.Clear();
                        continue;
                    }
                    // The cone is as wide as it may go and still cannot fill
                    // a bowl. A small leftover ridge may now be crossed, and
                    // the search starts again on the arrow.
                    if ((Phase == 0) && !CrossRidges)
                    {
                        CrossRidges = true;
                        FlowSlack = 0.0;
                        ShortMouths.Clear();
                        FoldedMouths.Clear();
                        Blocked.Clear();
                        continue;
                    }
                    // No full strip is left. Short lines are still one strip,
                    // and they stop on the first ridge again.
                    if (Phase == 0)
                    {
                        Phase = 1;
                        FlowSlack = 0.0;
                        CrossRidges = false;
                        continue;
                    }
                    // Nothing straight is left. The end of the field keeps a fold.
                    if ((Phase == 1) && FinishLeftovers && (FoldedMouths.Count > 0))
                    {
                        Phase = 2;
                        continue;
                    }
                    return null;
                }
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cut = Grow(
                    Seed.Value,
                    Sign: -1.0,
                    CutLeft,
                    Ledger.CutBcyPerDepthM,
                    MaximumLoadBcy,
                    MaxDepthM,
                    BladeWidthM,
                    Fx,
                    Fy,
                    Grid,
                    Active,
                    Centroid.Value,
                    FieldEdge,
                    DragAcrossGrade: false,
                    CutRemaining: null,
                    Already: null,
                    MinimumRadiusM,
                    FlowSlack,
                    CrossRidges: false);
                if (Cut.Count == 0)
                {
                    Blocked.Add(Seed.Value);
                    continue;
                }
                // The cut worked backwards from the crossover. A short bowl
                // picks up soil beside its upstream end when that soil still
                // drains through this same mouth.
                Double CutBcy = Cut.Sum(Entry => Entry.DepthM * Ledger.CutBcyPerDepthM);
                if (CutBcy < MaximumLoadBcy - 0.05)
                {
                    CutBcy = TopUpCut(
                        Cut,
                        CutLeft,
                        Ledger.CutBcyPerDepthM,
                        MaximumLoadBcy,
                        MaxDepthM,
                        BladeWidthM,
                        Fx,
                        Fy,
                        Grid,
                        Active,
                        Centroid.Value,
                        FieldEdge,
                        MinimumRadiusM,
                        FlowSlack);
                }
                if ((Phase == 0) && (CutBcy < MaximumLoadBcy - 0.05))
                {
                    Refund(Cut, CutLeft);
                    ShortMouths.Add(Seed.Value);
                    continue;
                }
                (Int32 Bx, Int32 By) CutEnd = Cut[Cut.Count - 1].Cell;
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fill = GrowFill(
                    CutEnd,
                    FillLeft,
                    Ledger.FillBcyPerDepthM,
                    CutBcy,
                    MaxDepthM,
                    BladeWidthM,
                    Fx,
                    Fy,
                    Grid,
                    Active,
                    Centroid.Value,
                    Phase == 2,
                    FieldEdge,
                    CutLeft,
                    MinimumRadiusM,
                    FlowSlack,
                    CrossRidges);
                if (Fill.Count == 0)
                {
                    Refund(Cut, CutLeft);
                    // The basin is still there, but every arrival folded back.
                    // That load waits for the end of the field.
                    if (Phase < 2)
                    {
                        FoldedMouths.Add(Seed.Value);
                    }
                    else
                    {
                        Blocked.Add(Seed.Value);
                    }
                    continue;
                }
                CutBcy = MatchVolumes(
                    Cut,
                    Fill,
                    CutLeft,
                    FillLeft,
                    Ledger.CutBcyPerDepthM,
                    Ledger.FillBcyPerDepthM);
                // The fill strip could not take a full bowl. This line waits.
                // Emitting it now would spend the dirt a later full strip still needs.
                if ((Phase == 0) && (CutBcy < MaximumLoadBcy - 0.05))
                {
                    Refund(Cut, CutLeft);
                    Refund(Fill, FillLeft);
                    ShortMouths.Add(Seed.Value);
                    continue;
                }
                if ((Cut.Count == 0) || (Fill.Count == 0) || (CutBcy <= VOLUME_TOLERANCE_BCY))
                {
                    Refund(Cut, CutLeft);
                    Refund(Fill, FillLeft);
                    if (Phase < 2)
                    {
                        FoldedMouths.Add(Seed.Value);
                    }
                    else
                    {
                        Blocked.Add(Seed.Value);
                    }
                    continue;
                }
                // A cut or fill that finishes beside its start circled the pocket.
                // A corner on the field edge is a headland turn and stays.
                // Only the end of the field keeps an interior fold.
                if ((Phase < 2) && (FoldsBack(Cut, FieldEdge) || FoldsBack(Fill, FieldEdge)))
                {
                    Refund(Cut, CutLeft);
                    Refund(Fill, FillLeft);
                    FoldedMouths.Add(Seed.Value);
                    continue;
                }
                (Double X, Double Y) CutStartXy = Grid.CellCenterXy(Cut[0].Cell.Bx, Cut[0].Cell.By);
                (Double X, Double Y) CutEndXy = Grid.CellCenterXy(Cut[Cut.Count - 1].Cell.Bx, Cut[Cut.Count - 1].Cell.By);
                (Double X, Double Y) FillStartXy = Grid.CellCenterXy(Fill[0].Cell.Bx, Fill[0].Cell.By);
                (Double X, Double Y) FillEndXy = Grid.CellCenterXy(
                    Fill[Fill.Count - 1].Cell.Bx,
                    Fill[Fill.Count - 1].Cell.By);
                Double HaulM = Distance(CutEndXy, FillStartXy);
                if (Active.Active) HaulM = Active.PathLength(CutEndXy, FillStartXy);
                Pass Pass = new Pass(
                    Cut,
                    Fill,
                    CutBcy,
                    HaulM,
                    CutStartXy,
                    CutEndXy,
                    FillStartXy,
                    FillEndXy);
                Pass.RibbonOrder = true;
                Pass.CutHeadingDeg = Flow.DirectionDeg(CutEnd.Bx, CutEnd.By);
                Pass.FillHeadingDeg = Flow.DirectionDeg(Fill[0].Cell.Bx, Fill[0].Cell.By);
                return Pass;
            }
            return null;
        }

        /// <summary>
        /// The machine speeds the cycle clock uses when a caller does not pass its own.
        /// </summary>
        /// <returns>The default time parameters.</returns>
        private static TimeParams DefaultClock()
        {
            return new TimeParams(
                Constants.DEFAULT_CUT_SPEED_MPH,
                Constants.DEFAULT_LOADED_TRAVEL_SPEED_MPH,
                Constants.DEFAULT_FILL_SPEED_MPH,
                Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
                Constants.DEFAULT_ACCELERATION_TIME_S,
                Constants.DEFAULT_DECELERATION_TIME_S,
                Constants.DEFAULT_FIELD_EFFICIENCY);
        }

        /// <summary>
        /// Poisson arrows after the neighbor smoothing the ribbon walk uses.
        /// A picture of a load should use these, because the cut follows them.
        /// </summary>
        /// <param name="Flow">Poisson field from the solver.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>Smoothed east and north components. A vanished arrow is zero.</returns>
        internal static (Double[,] Fx, Double[,] Fy) SmoothedArrows
            (
            FlowField Flow,
            Grid Grid
            )
        {
            return Smooth(Flow, Grid);
        }

        /// <summary>
        /// Averages the unit arrows so a ribbon follows the grain instead of
        /// steering at every noisy cell.
        /// </summary>
        /// <param name="Flow">Solved flow field.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>Smoothed unit components.</returns>
        private static (Double[,] Fx, Double[,] Fy) Smooth
            (
            FlowField Flow,
            Grid Grid
            )
        {
            Double[,] Fx = (Double[,])Flow.Fx.Clone();
            Double[,] Fy = (Double[,])Flow.Fy.Clone();
            for (Int32 Pass = 0; Pass < SMOOTH_PASSES; Pass += 1)
            {
                Double[,] NextX = (Double[,])Fx.Clone();
                Double[,] NextY = (Double[,])Fy.Clone();
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        if (!Grid.Valid[By, Bx]) continue;
                        Double SumX = Fx[By, Bx];
                        Double SumY = Fy[By, Bx];
                        Int32 Count = 1;
                        foreach ((Int32 Dx, Int32 Dy) Step in new[] { (-1, 0), (1, 0), (0, -1), (0, 1) })
                        {
                            Int32 Nx = Bx + Step.Dx;
                            Int32 Ny = By + Step.Dy;
                            if ((Nx < 0) || (Ny < 0) || (Nx >= Grid.Width) || (Ny >= Grid.Height)) continue;
                            if (!Grid.Valid[Ny, Nx]) continue;
                            SumX += Fx[Ny, Nx];
                            SumY += Fy[Ny, Nx];
                            Count += 1;
                        }
                        Double Mag = Math.Sqrt((SumX * SumX) + (SumY * SumY));
                        if (Mag <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                        {
                            NextX[By, Bx] = 0.0;
                            NextY[By, Bx] = 0.0;
                        }
                        else
                        {
                            NextX[By, Bx] = SumX / Mag;
                            NextY[By, Bx] = SumY / Mag;
                        }
                    }
                }
                Fx = NextX;
                Fy = NextY;
            }
            return (Fx, Fy);
        }

        /// <summary>
        /// Centre of the remaining fill, in cell coordinates. The ribbon steers
        /// here when the local arrow points away from the basin.
        /// </summary>
        /// <param name="FillLeft">Remaining fill depth.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The centroid, or null when no fill remains.</returns>
        private static (Double X, Double Y)? FillCentroid
            (
            Double[,] FillLeft,
            Grid Grid
            )
        {
            Double SumW = 0.0;
            Double SumX = 0.0;
            Double SumY = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Double Depth = FillLeft[By, Bx];
                    if (Depth <= DEPTH_TOLERANCE_M) continue;
                    if (!Grid.Valid[By, Bx]) continue;
                    SumW += Depth;
                    SumX += Depth * (Bx + 0.5);
                    SumY += Depth * (By + 0.5);
                }
            }
            if (SumW <= DEPTH_TOLERANCE_M) return null;
            return (SumX / SumW, SumY / SumW);
        }

        /// <summary>
        /// Steps near the flow toward soil that is still open.
        /// The pure arrow is kept when it holds as much soil as any step
        /// inside the cone, so a slack of zero is that arrow.
        /// </summary>
        /// <param name="FlowX">East component of the strict step.</param>
        /// <param name="FlowY">North component of the strict step.</param>
        /// <param name="Slack">
        /// Fraction of the arrow the step may leave. The step keeps at least
        /// one minus this as its alignment with the arrow.
        /// </param>
        /// <param name="X">Column, in cell coordinates.</param>
        /// <param name="Y">Row, in cell coordinates.</param>
        /// <param name="Taken">Cells already on this load. Null when none are.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <returns>A unit step. It is the arrow when the cone has no better soil.</returns>
        private static (Double X, Double Y) DirectionWithinSlack
            (
            Double FlowX,
            Double FlowY,
            Double Slack,
            Double X,
            Double Y,
            HashSet<(Int32 Bx, Int32 By)>? Taken,
            Double[,] Remaining,
            Grid Grid,
            ObstacleField Obstacles,
            Double BladeWidthM
            )
        {
            Double Length = Math.Sqrt((FlowX * FlowX) + (FlowY * FlowY));
            if (Length <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return (FlowX, FlowY);
            }
            FlowX /= Length;
            FlowY /= Length;
            if (Slack <= 0.0) return (FlowX, FlowY);
            // 10 percent leaves about 26 degrees. A full opening stops at 90,
            // square to the arrow, so the step cannot run back uphill.
            Double MinDot = Math.Max(0.0, 1.0 - Slack);
            Double Limit = Math.Acos(Math.Min(1.0, MinDot));
            Double BestScore = SoilAhead(
                X, Y, FlowX, FlowY, Taken, Remaining, Grid, Obstacles, BladeWidthM);
            Double BestX = FlowX;
            Double BestY = FlowY;
            Int32 Wings = Math.Max(1, (Int32)Math.Ceiling(Limit / (10.0 * Math.PI / 180.0)));
            for (Int32 Wing = 1; Wing <= Wings; Wing += 1)
            {
                Double Angle = Limit * Wing / Wings;
                for (Int32 Side = -1; Side <= 1; Side += 2)
                {
                    Double Turn = Side * Angle;
                    Double Cos = Math.Cos(Turn);
                    Double Sin = Math.Sin(Turn);
                    Double DirX = (FlowX * Cos) - (FlowY * Sin);
                    Double DirY = (FlowX * Sin) + (FlowY * Cos);
                    Double Score = SoilAhead(
                        X, Y, DirX, DirY, Taken, Remaining, Grid, Obstacles, BladeWidthM);
                    // The arrow wins a tie. Leaving it is only for more soil.
                    if (Score > BestScore)
                    {
                        BestScore = Score;
                        BestX = DirX;
                        BestY = DirY;
                    }
                }
            }
            return (BestX, BestY);
        }

        /// <summary>
        /// Remaining depth a short look along one heading would still cross.
        /// The look is the same gap that ends a ribbon, so the step can see
        /// the soil it is about to lose without searching the whole field.
        /// </summary>
        /// <param name="X">Column, in cell coordinates.</param>
        /// <param name="Y">Row, in cell coordinates.</param>
        /// <param name="DirX">East component of the unit heading.</param>
        /// <param name="DirY">North component of the unit heading.</param>
        /// <param name="Taken">Cells already on this load. Null when none are.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <returns>Sum of open depth, in metres, under the blade along the look.</returns>
        private static Double SoilAhead
            (
            Double X,
            Double Y,
            Double DirX,
            Double DirY,
            HashSet<(Int32 Bx, Int32 By)>? Taken,
            Double[,] Remaining,
            Grid Grid,
            ObstacleField Obstacles,
            Double BladeWidthM
            )
        {
            Double Half = (BladeWidthM / Grid.GridSizeM) * Constants.HALF_SCALE;
            Double SideX = -DirY;
            Double SideY = DirX;
            Double Score = 0.0;
            for (Int32 Step = 1; Step <= MISS_GAP; Step += 1)
            {
                Double Sx = X + (Step * TRACE_STEP_CELLS * DirX);
                Double Sy = Y + (Step * TRACE_STEP_CELLS * DirY);
                Score += OpenDepth(Sx, Sy, Taken, Remaining, Grid, Obstacles);
                Score += OpenDepth(Sx + (Half * SideX), Sy + (Half * SideY), Taken, Remaining, Grid, Obstacles);
                Score += OpenDepth(Sx - (Half * SideX), Sy - (Half * SideY), Taken, Remaining, Grid, Obstacles);
            }
            return Score;
        }

        /// <summary>
        /// Depth still available at one point, or zero when that cell is closed.
        /// </summary>
        /// <param name="X">Column, in cell coordinates.</param>
        /// <param name="Y">Row, in cell coordinates.</param>
        /// <param name="Taken">Cells already on this load. Null when none are.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <returns>Open depth in metres.</returns>
        private static Double OpenDepth
            (
            Double X,
            Double Y,
            HashSet<(Int32 Bx, Int32 By)>? Taken,
            Double[,] Remaining,
            Grid Grid,
            ObstacleField Obstacles
            )
        {
            Int32 Bx = (Int32)Math.Floor(X);
            Int32 By = (Int32)Math.Floor(Y);
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height)) return 0.0;
            if (!Grid.Valid[By, Bx]) return 0.0;
            if ((Taken != null) && Taken.Contains((Bx, By))) return 0.0;
            if (InsideObstacle(Grid, Obstacles, Bx, By)) return 0.0;
            Double Depth = Remaining[By, Bx];
            return Depth > DEPTH_TOLERANCE_M ? Depth : 0.0;
        }

        /// <summary>
        /// Poisson arrow at this point. The upslope search follows it.
        /// When the arrow vanishes, the heading faces the fill.
        /// </summary>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="X">Column coordinate.</param>
        /// <param name="Y">Row coordinate.</param>
        /// <param name="Fill">Fill centroid in cell coordinates.</param>
        /// <returns>A unit heading, downhill along the flow.</returns>
        private static (Double Hx, Double Hy) PoissonAhead
            (
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            Double X,
            Double Y,
            (Double X, Double Y) Fill
            )
        {
            (Double Ux, Double Uy) = Sample(Fx, Fy, Grid, X, Y);
            if ((Ux * Ux) + (Uy * Uy) > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return (Ux, Uy);
            }
            return Steer(Fx, Fy, Grid, X, Y, Fill);
        }

        /// <summary>
        /// Heading the machine should face. When the Poisson arrow points
        /// toward the fill, that arrow is the whole heading. Blending in the
        /// basin centroid pulls the mouth off the crossover, and the haul
        /// then drives across cut that is closer to the fill. The centroid
        /// is used only when the arrow points away, so the back of a mound
        /// is cut toward the basin instead of off the field.
        /// </summary>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="X">Column coordinate.</param>
        /// <param name="Y">Row coordinate.</param>
        /// <param name="Fill">Fill centroid in cell coordinates.</param>
        /// <returns>A unit heading.</returns>
        private static (Double Hx, Double Hy) Steer
            (
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            Double X,
            Double Y,
            (Double X, Double Y) Fill
            )
        {
            (Double Ux, Double Uy) = Sample(Fx, Fy, Grid, X, Y);
            Double Dx = Fill.X - X;
            Double Dy = Fill.Y - Y;
            Double Len = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Len <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return (Ux, Uy);
            }
            Double Tx = Dx / Len;
            Double Ty = Dy / Len;
            // The arrow already runs downhill toward the fill. Keep it.
            // Adding the centroid here is what made the haul cross the orange.
            if ((Ux * Tx) + (Uy * Ty) > 0.0)
            {
                return (Ux, Uy);
            }
            return (Tx, Ty);
        }

        /// <summary>
        /// Adds cut beside the upstream end of a short bowl. The added soil
        /// drains back through this mouth, so the haul stays the gap from
        /// that mouth to the fill. The pan finishes the added strip and steps
        /// onto the ribbon it already had.
        /// </summary>
        /// <param name="Cut">Cut cells, ridge first. The mouth is the last cell.</param>
        /// <param name="CutLeft">Remaining cut depth. Reduced by what this adds.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of cut.</param>
        /// <param name="MaximumLoadBcy">Scraper capacity in bank cubic yards.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Fill">Fill centroid.</param>
        /// <param name="FieldEdge">Cells on the outer boundary.</param>
        /// <param name="MinimumRadiusM">Tightest blade-down radius, in metres.</param>
        /// <param name="FlowSlack">
        /// Fraction of the arrow a step may leave. Zero follows the arrow.
        /// </param>
        /// <returns>The cut volume after the extra strips.</returns>
        private static Double TopUpCut
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cut,
            Double[,] CutLeft,
            Double BcyPerDepthM,
            Double MaximumLoadBcy,
            Double MaxDepthM,
            Double BladeWidthM,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            ObstacleField Obstacles,
            (Double X, Double Y) Fill,
            Boolean[,] FieldEdge,
            Double MinimumRadiusM,
            Double FlowSlack
            )
        {
            if (Cut.Count == 0) return 0.0;
            (Int32 Bx, Int32 By) Mouth = Cut[Cut.Count - 1].Cell;
            HashSet<(Int32 Bx, Int32 By)> Taken = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cut)
            {
                Taken.Add(Entry.Cell);
            }
            Double Volume = Cut.Sum(Entry => Entry.DepthM * BcyPerDepthM);
            Int32 Added = 0;
            Int32 AddedCells = 0;
            Int32 Attempts = 0;
            while ((Added < TOPUP_STRIPS)
                && (Attempts < 24)
                && (Volume < MaximumLoadBcy - 0.05))
            {
                Attempts += 1;
                // Only the upstream end. Soil beside the mouth would be a
                // trip back up the hill after the pan had already arrived.
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Anchor =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                    {
                        Cut[0],
                    };
                (Int32 Bx, Int32 By)? Seed = NearestOpen(
                    Anchor, CutLeft, Grid, Taken, TOPUP_REACH_CELLS);
                if (!Seed.HasValue) break;
                if (!DrainsToMouth(
                    Seed.Value.Bx + 0.5, Seed.Value.By + 0.5,
                    Mouth.Bx, Mouth.By, Fx, Fy, Grid, Fill))
                {
                    Taken.Add(Seed.Value);
                    continue;
                }
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Extra = Grow(
                    Seed.Value,
                    Sign: -1.0,
                    CutLeft,
                    BcyPerDepthM,
                    MaximumLoadBcy - Volume,
                    MaxDepthM,
                    BladeWidthM,
                    Fx,
                    Fy,
                    Grid,
                    Obstacles,
                    Fill,
                    FieldEdge,
                    DragAcrossGrade: false,
                    CutRemaining: null,
                    Already: Taken,
                    MinimumRadiusM,
                    FlowSlack,
                    CrossRidges: false);
                if (Extra.Count == 0)
                {
                    Taken.Add(Seed.Value);
                    continue;
                }
                (Int32 Bx, Int32 By) ExtraEnd = Extra[Extra.Count - 1].Cell;
                (Int32 Bx, Int32 By) Start = Cut[0].Cell;
                Int32 Gap = Math.Max(
                    Math.Abs(ExtraEnd.Bx - Start.Bx),
                    Math.Abs(ExtraEnd.By - Start.By));
                // The extra strip has to finish on the ribbon. A longer hop
                // is a second haul, not the rest of this bowl.
                if (Gap > (Int32)TOPUP_REACH_CELLS)
                {
                    Refund(Extra, CutLeft);
                    Taken.Add(Seed.Value);
                    continue;
                }
                Cut.InsertRange(0, Extra);
                AddedCells += Extra.Count;
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Extra)
                {
                    Taken.Add(Entry.Cell);
                    Volume += Entry.DepthM * BcyPerDepthM;
                }
                Added += 1;
            }
            // A partial extra strip is a second haul. Keep the soil on this
            // load only when it finishes the bowl.
            if ((AddedCells > 0) && (Volume < MaximumLoadBcy - 0.05))
            {
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Extra =
                    Cut.GetRange(0, AddedCells);
                Refund(Extra, CutLeft);
                Cut.RemoveRange(0, AddedCells);
                Volume = Cut.Sum(Entry => Entry.DepthM * BcyPerDepthM);
            }
            return Volume;
        }

        /// <summary>
        /// Nearest remaining cut within a few blade widths of the current mouth.
        /// Farther than that, a new load is shorter than the turnaround.
        /// </summary>
        /// <param name="Ribbon">Cells already on this load, drive order.</param>
        /// <param name="CutLeft">Remaining cut depth.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="OnRibbon">Cells already taken by this load.</param>
        /// <returns>The neighbor, or null when the next dirt is not adjacent.</returns>
        private static (Int32 Bx, Int32 By)? AdjacentCut
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ribbon,
            Double[,] CutLeft,
            Grid Grid,
            HashSet<(Int32 Bx, Int32 By)> OnRibbon
            )
        {
            return NearestOpen(Ribbon, CutLeft, Grid, OnRibbon, ADJACENT_CELLS);
        }

        /// <summary>
        /// Nearest open cell anywhere, measured from where the scraper just finished.
        /// The bowl is still short, so the machine drives back instead of hauling.
        /// </summary>
        /// <param name="Ribbon">Cells already on this load, in drive order.</param>
        /// <param name="Depth">Remaining depth of the side being extended.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Skip">Cells already tried or taken.</param>
        /// <returns>The cell, or null when nothing of this side remains.</returns>
        private static (Int32 Bx, Int32 By)? NearestRemaining
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ribbon,
            Double[,] Depth,
            Grid Grid,
            HashSet<(Int32 Bx, Int32 By)> Skip
            )
        {
            if (Ribbon.Count == 0) return null;
            (Int32 Bx, Int32 By) From = Ribbon[Ribbon.Count - 1].Cell;
            (Int32 Bx, Int32 By)? Best = null;
            Double BestD = Double.PositiveInfinity;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Depth[By, Bx] <= DEPTH_TOLERANCE_M) continue;
                    if (Skip.Contains((Bx, By))) continue;
                    if (!Grid.Valid[By, Bx]) continue;
                    Double Dx = Bx - From.Bx;
                    Double Dy = By - From.By;
                    Double D = (Dx * Dx) + (Dy * Dy);
                    if (D < BestD)
                    {
                        BestD = D;
                        Best = (Bx, By);
                    }
                }
            }
            return Best;
        }

        /// <summary>
        /// Nearest open cell within reach of the ribbon. The window stays on the
        /// ribbon so a large field is not scanned from scratch for every strip.
        /// </summary>
        /// <param name="Ribbon">Cells already on this load.</param>
        /// <param name="Depth">Remaining depth of the side being extended.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="OnRibbon">Cells already taken, which are skipped.</param>
        /// <param name="ReachCells">Farthest a new strip may start, in cells.</param>
        /// <returns>The neighbor, or null when nothing open is in reach.</returns>
        private static (Int32 Bx, Int32 By)? NearestOpen
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ribbon,
            Double[,] Depth,
            Grid Grid,
            HashSet<(Int32 Bx, Int32 By)> OnRibbon,
            Double ReachCells
            )
        {
            if (Ribbon.Count == 0) return null;
            Int32 Reach = (Int32)Math.Ceiling(ReachCells);
            (Int32 Bx, Int32 By)? Best = null;
            Double BestD = ReachCells;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Ribbon)
            {
                Int32 X0 = Math.Max(0, Entry.Cell.Bx - Reach);
                Int32 X1 = Math.Min(Grid.Width - 1, Entry.Cell.Bx + Reach);
                Int32 Y0 = Math.Max(0, Entry.Cell.By - Reach);
                Int32 Y1 = Math.Min(Grid.Height - 1, Entry.Cell.By + Reach);
                for (Int32 By = Y0; By <= Y1; By += 1)
                {
                    for (Int32 Bx = X0; Bx <= X1; Bx += 1)
                    {
                        if (Depth[By, Bx] <= DEPTH_TOLERANCE_M) continue;
                        if (OnRibbon.Contains((Bx, By))) continue;
                        if (!Grid.Valid[By, Bx]) continue;
                        Double Dx = Bx - Entry.Cell.Bx;
                        Double Dy = By - Entry.Cell.By;
                        Double D = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                        if (D < BestD)
                        {
                            BestD = D;
                            Best = (Bx, By);
                        }
                    }
                }
            }
            return Best;
        }

        /// <summary>
        /// Walks from a side cell downstream until the cut ends.
        /// The ribbon for that strip finishes at that mouth.
        /// </summary>
        /// <param name="Start">A remaining cut cell beside the current ribbon.</param>
        /// <param name="CutLeft">Remaining cut depth.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Fill">Fill centroid.</param>
        /// <returns>The downstream mouth of that strip.</returns>
        private static (Int32 Bx, Int32 By) WalkToMouth
            (
            (Int32 Bx, Int32 By) Start,
            Double[,] CutLeft,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            (Double X, Double Y) Fill
            )
        {
            Double X = Start.Bx + 0.5;
            Double Y = Start.By + 0.5;
            (Int32 Bx, Int32 By) Mouth = Start;
            for (Int32 Step = 0; Step < MAX_TRACE_STEPS; Step += 1)
            {
                (Double Hx, Double Hy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
                if ((Hx * Hx) + (Hy * Hy) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) break;
                X += TRACE_STEP_CELLS * Hx;
                Y += TRACE_STEP_CELLS * Hy;
                Int32 Cx = (Int32)Math.Floor(X);
                Int32 Cy = (Int32)Math.Floor(Y);
                if ((Cx < 0) || (Cy < 0) || (Cx >= Grid.Width) || (Cy >= Grid.Height) || !Grid.Valid[Cy, Cx])
                {
                    break;
                }
                if (CutLeft[Cy, Cx] <= DEPTH_TOLERANCE_M) break;
                Mouth = (Cx, Cy);
            }
            return Mouth;
        }

        /// <summary>
        /// Trims the longer side so the load balances. Cut is shortened from
        /// the ridge. Fill is shortened from the downstream tail. The haul
        /// between them stays.
        /// </summary>
        /// <param name="Cut">Cut cells, ridge first.</param>
        /// <param name="Fill">Fill cells, nearest the cut first.</param>
        /// <param name="CutLeft">Cut depth ledger.</param>
        /// <param name="FillLeft">Fill depth ledger.</param>
        /// <param name="CutBcyPerDepthM">Bank cubic yards per metre of cut.</param>
        /// <param name="FillBcyPerDepthM">Bank cubic yards per metre of fill.</param>
        /// <returns>The matched volume.</returns>
        private static Double MatchVolumes
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cut,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fill,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double CutBcyPerDepthM,
            Double FillBcyPerDepthM
            )
        {
            Double CutBcy = Cut.Sum(Entry => Entry.DepthM * CutBcyPerDepthM);
            Double FillBcy = Fill.Sum(Entry => Entry.DepthM * FillBcyPerDepthM);
            if (CutBcy > FillBcy + MATCH_TOLERANCE_BCY)
            {
                RefundExcess(Cut, CutLeft, CutBcyPerDepthM, FillBcy, FromStart: true);
            }
            else if (FillBcy > CutBcy + MATCH_TOLERANCE_BCY)
            {
                RefundExcess(Fill, FillLeft, FillBcyPerDepthM, CutBcy, FromStart: false);
            }
            // Leaf trimming can stop a hair short. The ledger rejects that.
            return ForceMatch(Cut, Fill, CutLeft, FillLeft, CutBcyPerDepthM, FillBcyPerDepthM);
        }

        /// <summary>
        /// Trims the heavier side until cut and fill are the same volume.
        /// The last adjustment is a depth change on one cell, so a bridge stays.
        /// </summary>
        /// <param name="Cut">Cut cells.</param>
        /// <param name="Fill">Fill cells.</param>
        /// <param name="CutLeft">Cut depth ledger.</param>
        /// <param name="FillLeft">Fill depth ledger.</param>
        /// <param name="CutBcyPerDepthM">Bank cubic yards per metre of cut.</param>
        /// <param name="FillBcyPerDepthM">Bank cubic yards per metre of fill.</param>
        /// <returns>The matched volume.</returns>
        private static Double ForceMatch
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cut,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fill,
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double CutBcyPerDepthM,
            Double FillBcyPerDepthM
            )
        {
            for (Int32 Guard = 0; Guard < 8; Guard += 1)
            {
                Double CutBcy = Cut.Sum(Entry => Entry.DepthM * CutBcyPerDepthM);
                Double FillBcy = Fill.Sum(Entry => Entry.DepthM * FillBcyPerDepthM);
                Double Gap = CutBcy - FillBcy;
                if (Math.Abs(Gap) <= MATCH_TOLERANCE_BCY) return CutBcy;
                // The last pass may shave below the tip floor. That floor is what
                // left a speck the ledger still calls unbalanced.
                Double FloorM = Guard < 7 ? DEPTH_TOLERANCE_M : 1e-12;
                if (Gap > 0.0)
                {
                    ShrinkBy(Cut, CutLeft, CutBcyPerDepthM, Gap, FloorM);
                }
                else
                {
                    ShrinkBy(Fill, FillLeft, FillBcyPerDepthM, -Gap, FloorM);
                }
            }
            return Cut.Sum(Entry => Entry.DepthM * CutBcyPerDepthM);
        }

        /// <summary>Returns one side's volume to the ledger until the gap is gone.</summary>
        /// <param name="Cells">Cells to thin. The far end goes first.</param>
        /// <param name="Remaining">Depth ledger.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="ExcessBcy">Volume to give back.</param>
        /// <param name="FloorM">Shallowest depth a cell may keep. Zero is rejected by the ledger.</param>
        private static void ShrinkBy
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double ExcessBcy,
            Double FloorM
            )
        {
            // Depth only. Removing the cell here is how a corner gap comes back.
            for (Int32 Step = 0; (Step < Cells.Count) && (ExcessBcy > MATCH_TOLERANCE_BCY); Step += 1)
            {
                Int32 Index = Cells.Count - 1 - Step;
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Cells[Index];
                if (Entry.DepthM <= FloorM) continue;
                Double DropM = Math.Min(
                    Entry.DepthM - FloorM,
                    ExcessBcy / BcyPerDepthM);
                Remaining[Entry.Cell.By, Entry.Cell.Bx] += DropM;
                Cells[Index] = (Entry.Cell, Entry.DepthM - DropM);
                ExcessBcy -= DropM * BcyPerDepthM;
            }
        }

        /// <summary>One downstream edge cell and how far that cell sits from the fill.</summary>
        private struct MouthCandidate
        {
            /// <summary>Column of the mouth.</summary>
            public Int32 Bx;

            /// <summary>Row of the mouth.</summary>
            public Int32 By;

            /// <summary>Steps downstream from this mouth until remaining fill.</summary>
            public Double Haul;
        }

        /// <summary>
        /// Picks the downstream edge of a cut long enough to fill the bowl.
        /// The edge is where the Poisson arrow leaves the cut, so the haul
        /// is only the gap from that crossover to the fill. The nearest
        /// full ribbon wins. A thin edge that cannot fill the pan waits.
        /// </summary>
        /// <param name="CutLeft">Remaining cut depth.</param>
        /// <param name="FillLeft">Remaining fill depth.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Blocked">Seeds that could not reach a fill.</param>
        /// <param name="Fill">Fill centroid the ribbon steers toward.</param>
        /// <param name="CutBcyPerDepthM">Bank cubic yards per metre of cut.</param>
        /// <param name="MaximumLoadBcy">Scraper capacity in bank cubic yards.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="FullOnly">
        /// When true, a line that cannot fill the bowl is left for later.
        /// The thickest scrap is not a mouth while a full strip may still exist.
        /// </param>
        /// <param name="FieldEdge">Cells on the outer boundary, where a sharp turn is a headland.</param>
        /// <param name="MinimumRadiusM">
        /// Tightest blade-down radius, in metres. The field edge may use it.
        /// A bend inside the cut uses a wider radius.
        /// </param>
        /// <param name="FlowSlack">
        /// Fraction of the arrow the upstream walk may leave while it looks
        /// for a full bowl. Zero follows the arrow.
        /// </param>
        /// <returns>The mouth cell, or null when no cut remains that can haul.</returns>
        private static (Int32 Bx, Int32 By)? DownstreamMouth
            (
            Double[,] CutLeft,
            Double[,] FillLeft,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            ObstacleField Obstacles,
            HashSet<(Int32 Bx, Int32 By)> Blocked,
            (Double X, Double Y) Fill,
            Double CutBcyPerDepthM,
            Double MaximumLoadBcy,
            Double MaxDepthM,
            Double BladeWidthM,
            Boolean FullOnly,
            Boolean[,] FieldEdge,
            Double MinimumRadiusM,
            Double FlowSlack
            )
        {
            List<MouthCandidate> Mouths = new List<MouthCandidate>();
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (CutLeft[By, Bx] <= DEPTH_TOLERANCE_M) continue;
                    if (Blocked.Contains((Bx, By))) continue;
                    if (InsideObstacle(Grid, Obstacles, Bx, By)) continue;
                    (Double Hx, Double Hy) = PoissonAhead(Fx, Fy, Grid, Bx + 0.5, By + 0.5, Fill);
                    if ((Hx * Hx) + (Hy * Hy) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                    {
                        continue;
                    }
                    // The mouth is the last cut cell before the heading leaves the high ground.
                    (Int32 Nx, Int32 Ny) = StepCell(Bx + 0.5, By + 0.5, (Hx, Hy), 1.0, Grid);
                    Boolean LeavesCut = (Nx < 0)
                        || (CutLeft[Ny, Nx] <= DEPTH_TOLERANCE_M);
                    if (!LeavesCut) continue;
                    // A lone cell has no strip behind it. Starting there hauls
                    // one scrap across the field and the route shows no blade-down work.
                    (Int32 Px, Int32 Py) = StepCell(Bx + 0.5, By + 0.5, (Hx, Hy), -1.0, Grid);
                    Boolean HasUpstream = (Px >= 0)
                        && (CutLeft[Py, Px] > DEPTH_TOLERANCE_M);
                    if (!HasUpstream) continue;
                    Mouths.Add(new MouthCandidate { Bx = Bx, By = By, Haul = 0.0 });
                }
            }
            // The haul walk only reads the field. Measuring every mouth at once
            // keeps the nearest full strip, and a load no longer waits on one core.
            MouthCandidate[] Found = Mouths.ToArray();
            MeasureHauls(Found, FillLeft, Fx, Fy, Grid, Fill);
            Int32 Kept = 0;
            for (Int32 Index = 0; Index < Found.Length; Index += 1)
            {
                if (Found[Index].Haul >= 100000.0) continue;
                Found[Kept] = Found[Index];
                Kept += 1;
            }
            if (Kept < Found.Length) Array.Resize(ref Found, Kept);
            // Shortest haul first. The first ribbon that fills the bowl is the
            // near one, so a thin notch does not beat a full cut behind it.
            Array.Sort(Found, (A, B) => A.Haul.CompareTo(B.Haul));
            (Int32 Bx, Int32 By)? BestPartial = null;
            Double BestPartialVolume = -1.0;
            Double[] Volume = new Double[Found.Length];
            // A full bowl ends the search at the nearest mouth that holds one.
            // Probes in one wave are the same read, so they share the cores.
            // One wave is wide enough to use every core, and short enough that a
            // nearby full bowl still stops the search before the far mouths.
            Int32 Wave = Math.Max(32, Environment.ProcessorCount);
            for (Int32 Start = 0; Start < Found.Length; Start += Wave)
            {
                Int32 Count = Math.Min(Wave, Found.Length - Start);
                // While a full strip may still exist, later mouths are farther.
                // Their volume is not needed once this wave already filled the bowl.
                if (!FullOnly)
                {
                    Count = Found.Length - Start;
                }
                ProbeMouths(
                    Found, Volume, Start, Count,
                    CutLeft, CutBcyPerDepthM, MaximumLoadBcy, MaxDepthM, BladeWidthM,
                    Fx, Fy, Grid, Obstacles, Fill, FieldEdge, MinimumRadiusM,
                    FlowSlack);
                for (Int32 Offset = 0; Offset < Count; Offset += 1)
                {
                    Double UpstreamBcy = Volume[Start + Offset];
                    if (UpstreamBcy >= MaximumLoadBcy - VOLUME_TOLERANCE_BCY)
                    {
                        return (Found[Start + Offset].Bx, Found[Start + Offset].By);
                    }
                    if (UpstreamBcy > BestPartialVolume)
                    {
                        BestPartialVolume = UpstreamBcy;
                        BestPartial = (Found[Start + Offset].Bx, Found[Start + Offset].By);
                    }
                }
                if (!FullOnly) break;
            }
            if (FullOnly) return null;
            if (BestPartial.HasValue) return BestPartial;
            // No clean edge. Take the thickest remaining cut and let the trace find the fill.
            (Int32 Bx, Int32 By)? Thickest = null;
            Double BestDepth = -1.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (CutLeft[By, Bx] <= BestDepth) continue;
                    if (Blocked.Contains((Bx, By))) continue;
                    if (InsideObstacle(Grid, Obstacles, Bx, By)) continue;
                    BestDepth = CutLeft[By, Bx];
                    Thickest = (Bx, By);
                }
            }
            return Thickest;
        }

        /// <summary>
        /// Measures the haul of each mouth. The walks only read the field, so
        /// they run together. The mouth that wins is still the nearest full strip.
        /// </summary>
        /// <param name="Mouths">Mouths still missing a haul.</param>
        /// <param name="FillLeft">Remaining fill depth.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Fill">Fill centroid the ribbon steers toward.</param>
        private static void MeasureHauls
            (
            MouthCandidate[] Mouths,
            Double[,] FillLeft,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            (Double X, Double Y) Fill
            )
        {
            if (Mouths.Length < 8)
            {
                for (Int32 Index = 0; Index < Mouths.Length; Index += 1)
                {
                    MouthCandidate Mouth = Mouths[Index];
                    Mouth.Haul = DownstreamDistance(Mouth.Bx, Mouth.By, FillLeft, Fx, Fy, Grid, Fill);
                    Mouths[Index] = Mouth;
                }
                return;
            }
            Parallel.For(
                0,
                Mouths.Length,
                Index =>
                {
                    MouthCandidate Mouth = Mouths[Index];
                    Mouth.Haul = DownstreamDistance(Mouth.Bx, Mouth.By, FillLeft, Fx, Fy, Grid, Fill);
                    Mouths[Index] = Mouth;
                });
        }

        /// <summary>
        /// Probes a contiguous run of mouths. A wave is one read of the cut
        /// that is still there, so the volumes match a one-at-a-time probe.
        /// </summary>
        /// <param name="Mouths">Mouths in haul order.</param>
        /// <param name="Volume">Probe volume, written for this wave.</param>
        /// <param name="Start">First mouth in the wave.</param>
        /// <param name="Count">How many mouths are in the wave.</param>
        /// <param name="CutLeft">Remaining cut depth.</param>
        /// <param name="CutBcyPerDepthM">Bank cubic yards per metre of cut.</param>
        /// <param name="MaximumLoadBcy">Scraper capacity in bank cubic yards.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Fill">Fill centroid the ribbon steers toward.</param>
        /// <param name="FieldEdge">Cells on the outer boundary.</param>
        /// <param name="MinimumRadiusM">Tightest blade-down radius, in metres.</param>
        /// <param name="FlowSlack">Fraction of the arrow the upstream walk may leave.</param>
        private static void ProbeMouths
            (
            MouthCandidate[] Mouths,
            Double[] Volume,
            Int32 Start,
            Int32 Count,
            Double[,] CutLeft,
            Double CutBcyPerDepthM,
            Double MaximumLoadBcy,
            Double MaxDepthM,
            Double BladeWidthM,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            ObstacleField Obstacles,
            (Double X, Double Y) Fill,
            Boolean[,] FieldEdge,
            Double MinimumRadiusM,
            Double FlowSlack
            )
        {
            if (Count < 8)
            {
                for (Int32 Offset = 0; Offset < Count; Offset += 1)
                {
                    Volume[Start + Offset] = ProbeUpstream(
                        (Mouths[Start + Offset].Bx, Mouths[Start + Offset].By),
                        CutLeft,
                        CutBcyPerDepthM,
                        MaximumLoadBcy,
                        MaxDepthM,
                        BladeWidthM,
                        Fx,
                        Fy,
                        Grid,
                        Obstacles,
                        Fill,
                        FieldEdge,
                        MinimumRadiusM,
                        FlowSlack);
                }
                return;
            }
            Parallel.For(
                0,
                Count,
                Offset =>
                {
                    Volume[Start + Offset] = ProbeUpstream(
                        (Mouths[Start + Offset].Bx, Mouths[Start + Offset].By),
                        CutLeft,
                        CutBcyPerDepthM,
                        MaximumLoadBcy,
                        MaxDepthM,
                        BladeWidthM,
                        Fx,
                        Fy,
                        Grid,
                        Obstacles,
                        Fill,
                        FieldEdge,
                        MinimumRadiusM,
                        FlowSlack);
                });
        }

        /// <summary>
        /// Volume one upstream ribbon can take from this mouth, without writing the ledger.
        /// The walk matches <see cref="Grow"/> so the chosen mouth is the cut that will be driven.
        /// </summary>
        /// <param name="Seed">Downstream end of the ribbon.</param>
        /// <param name="Remaining">Cut depth still available.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="QuotaBcy">Volume the ribbon may still take.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Fill">Fill centroid the ribbon steers toward.</param>
        /// <param name="FieldEdge">Cells on the outer boundary, where a sharp turn is a headland.</param>
        /// <param name="MinimumRadiusM">Tightest blade-down radius, in metres.</param>
        /// <param name="FlowSlack">Fraction of the arrow the walk may leave. Zero follows the arrow.</param>
        /// <returns>Bank cubic yards the ribbon would cut.</returns>
        private static Double ProbeUpstream
            (
            (Int32 Bx, Int32 By) Seed,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double QuotaBcy,
            Double MaxDepthM,
            Double BladeWidthM,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            ObstacleField Obstacles,
            (Double X, Double Y) Fill,
            Boolean[,] FieldEdge,
            Double MinimumRadiusM,
            Double FlowSlack
            )
        {
            HashSet<(Int32 Bx, Int32 By)> Taken = new HashSet<(Int32 Bx, Int32 By)>();
            List<((Int32 Bx, Int32 By) Cell, Double Bcy)> Counted =
                new List<((Int32 Bx, Int32 By) Cell, Double Bcy)>();
            Double LeftBcy = QuotaBcy;
            ProbeTake(Seed.Bx, Seed.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Obstacles, Grid, Counted);
            Double X = Seed.Bx + 0.5;
            Double Y = Seed.By + 0.5;
            foreach ((Int32 Bx, Int32 By) Cell in BladeFootprint(X, Y, X, Y, BladeWidthM, Grid))
            {
                ProbeTake(Cell.Bx, Cell.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Obstacles, Grid, Counted);
            }
            (Double StartUx, Double StartUy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
            // Working backwards: up the slope, opposite the Poisson flow.
            Double HeadingX = -StartUx;
            Double HeadingY = -StartUy;
            // The backwards search restarts where soil was last taken, not
            // several steps past it into ground that is already at grade.
            Double SoilX = X;
            Double SoilY = Y;
            Double SoilHx = HeadingX;
            Double SoilHy = HeadingY;
            Int32 Misses = 0;
            Int32 ReverseSteps = 0;
            for (Int32 Step = 0; Step < MAX_TRACE_STEPS; Step += 1)
            {
                if (LeftBcy <= VOLUME_TOLERANCE_BCY) break;
                (Double Ux, Double Uy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
                if ((Ux * Ux) + (Uy * Uy) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) break;
                // The path is the Poisson flow. A full bowl that the arrow misses
                // may leave it, and only far enough to stay on the soil.
                Double DirX = -Ux;
                Double DirY = -Uy;
                if (FlowSlack > 0.0)
                {
                    (DirX, DirY) = DirectionWithinSlack(
                        DirX, DirY, FlowSlack, X, Y, Taken, Remaining, Grid, Obstacles, BladeWidthM);
                }
                Double NextX = X + (TRACE_STEP_CELLS * DirX);
                Double NextY = Y + (TRACE_STEP_CELLS * DirY);
                Int32 Cx = (Int32)Math.Floor(NextX);
                Int32 Cy = (Int32)Math.Floor(NextY);
                if ((Cx < 0) || (Cy < 0) || (Cx >= Grid.Width) || (Cy >= Grid.Height) || !Grid.Valid[Cy, Cx])
                {
                    break;
                }
                // Stop when the trace turns back toward the mouth. A few of
                // those steps on the field edge are the headland corner.
                if (ReturnsToStart(Seed.Bx + 0.5, Seed.By + 0.5, X, Y, NextX, NextY))
                {
                    Boolean Headland = OnFieldEdge(FieldEdge, X, Y) || OnFieldEdge(FieldEdge, NextX, NextY);
                    ReverseSteps += 1;
                    if (!Headland || (ReverseSteps > HEADLAND_REVERSE_STEPS))
                    {
                        break;
                    }
                }
                else
                {
                    ReverseSteps = 0;
                }
                Int32 Before = Taken.Count;
                foreach ((Int32 Bx, Int32 By) Cell in BladeFootprint(X, Y, NextX, NextY, BladeWidthM, Grid))
                {
                    ProbeTake(Cell.Bx, Cell.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Obstacles, Grid, Counted);
                }
                if (Taken.Count > Before)
                {
                    Misses = 0;
                    SoilX = NextX;
                    SoilY = NextY;
                    SoilHx = HeadingX;
                    SoilHy = HeadingY;
                }
                else
                {
                    Misses += 1;
                    if (Misses >= MISS_GAP) break;
                }
                Double StepX = NextX - X;
                Double StepY = NextY - Y;
                Double StepLength = Math.Sqrt((StepX * StepX) + (StepY * StepY));
                if (StepLength > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                {
                    HeadingX = StepX / StepLength;
                    HeadingY = StepY / StepLength;
                    if (Taken.Count > Before)
                    {
                        SoilHx = HeadingX;
                        SoilHy = HeadingY;
                    }
                }
                X = NextX;
                Y = NextY;
            }
            // The flow line has ended. That is the ridge. Follow it only while
            // the soil still drains back through this mouth.
            ExtendFromEnd(
                ref SoilX,
                ref SoilY,
                ref SoilHx,
                ref SoilHy,
                ref LeftBcy,
                Taken,
                FieldEdge,
                Remaining,
                BcyPerDepthM,
                MaxDepthM,
                BladeWidthM,
                Grid,
                Obstacles,
                null,
                Counted,
                -1.0,
                Fx,
                Fy,
                Fill,
                DragAcrossGrade: false,
                CutRemaining: null,
                MinimumRadiusM,
                Seed.Bx,
                Seed.By,
                FlowSlack,
                CrossRidges: false);
            // Loose cells that only meet the strip at a corner are not driven.
            // Counting them is what made a short line look like a full bowl.
            Double Kept = TouchingVolume(Counted);
            return Math.Min(QuotaBcy, Kept);
        }

        /// <summary>
        /// Unit direction of the step just taken. The blade is measured across it.
        /// Before the first step, the flow heading stands in.
        /// </summary>
        /// <param name="Previous">Last step in cell coordinates.</param>
        /// <param name="Sign">-1 upstream, +1 downstream.</param>
        /// <param name="Ux">Unit east heading of the flow.</param>
        /// <param name="Uy">Unit north heading of the flow.</param>
        /// <returns>A unit travel heading.</returns>
        private static (Double Tx, Double Ty) TravelHeading
            (
            (Double Px, Double Py) Previous,
            Double Sign,
            Double Ux,
            Double Uy
            )
        {
            Double Len = Math.Sqrt((Previous.Px * Previous.Px) + (Previous.Py * Previous.Py));
            if (Len <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return (Sign * Ux, Sign * Uy);
            }
            return (Previous.Px / Len, Previous.Py / Len);
        }

        /// <summary>
        /// Steps to the next cell along the flow. A step off that line is
        /// refused, so the ribbon stays one blade wide instead of eating the pocket.
        /// A step behind the heading is refused, so the ribbon cannot reverse.
        /// </summary>
        /// <param name="X">Current column, in cell coordinates. Updated on success.</param>
        /// <param name="Y">Current row, in cell coordinates. Updated on success.</param>
        /// <param name="Sign">-1 walks upstream, +1 walks downstream.</param>
        /// <param name="Ux">Unit east heading of the flow.</param>
        /// <param name="Uy">Unit north heading of the flow.</param>
        /// <param name="Previous">Last step, so the curve does not flick side to side.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Taken">Cells already on this ribbon.</param>
        /// <param name="Spine">Center cells of this ribbon, in walk order. The new cell is appended.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <returns>True when the trace moved to another cut cell.</returns>
        private static Boolean AdvanceInCut
            (
            ref Double X,
            ref Double Y,
            Double Sign,
            Double Ux,
            Double Uy,
            ref (Double Px, Double Py) Previous,
            Double[,] Remaining,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            List<(Int32 Bx, Int32 By)> Spine,
            Grid Grid,
            ObstacleField Obstacles
            )
        {
            Double Dx = Sign * Ux;
            Double Dy = Sign * Uy;
            Int32 HereX = (Int32)Math.Floor(X);
            Int32 HereY = (Int32)Math.Floor(Y);
            (Int32 Bx, Int32 By)? Best = null;
            Double BestScore = Double.NegativeInfinity;
            for (Int32 Ny = HereY - 1; Ny <= HereY + 1; Ny += 1)
            {
                for (Int32 Nx = HereX - 1; Nx <= HereX + 1; Nx += 1)
                {
                    if ((Nx == HereX) && (Ny == HereY)) continue;
                    if ((Nx < 0) || (Ny < 0) || (Nx >= Grid.Width) || (Ny >= Grid.Height)) continue;
                    if (!Grid.Valid[Ny, Nx]) continue;
                    if (Taken.Contains((Nx, Ny))) continue;
                    if (Remaining[Ny, Nx] <= DEPTH_TOLERANCE_M) continue;
                    if (InsideObstacle(Grid, Obstacles, Nx, Ny)) continue;
                    Double StepX = Nx + 0.5 - X;
                    Double StepY = Ny + 0.5 - Y;
                    Double Len = Math.Sqrt((StepX * StepX) + (StepY * StepY));
                    if (Len <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) continue;
                    Double Dot = ((StepX / Len) * Dx) + ((StepY / Len) * Dy);
                    // Behind the heading is a turnaround, which splits the cut in two.
                    if (Dot < 0.0) continue;
                    // Off the flow line. That step is how a pocket becomes a splotch.
                    if (Dot < FORWARD_DOT) continue;
                    if ((Previous.Px * Previous.Px) + (Previous.Py * Previous.Py) > 0.0)
                    {
                        Double PrevLen = Math.Sqrt((Previous.Px * Previous.Px) + (Previous.Py * Previous.Py));
                        Double Turn = ((StepX / Len) * (Previous.Px / PrevLen))
                            + ((StepY / Len) * (Previous.Py / PrevLen));
                        // The flow can swing around a peak. Following that swing
                        // drives the blade back into the patch it just cut.
                        if (Turn < TURN_DOT) continue;
                    }
                    // Beside an earlier part of this same ribbon. The path has looped.
                    if (FoldsIntoSpine(Nx, Ny, Spine)) continue;
                    Double Score = Dot;
                    if ((Previous.Px * Previous.Px) + (Previous.Py * Previous.Py) > 0.0)
                    {
                        Double PrevLen = Math.Sqrt((Previous.Px * Previous.Px) + (Previous.Py * Previous.Py));
                        Score += 0.35 * (((StepX / Len) * (Previous.Px / PrevLen))
                            + ((StepY / Len) * (Previous.Py / PrevLen)));
                    }
                    if (Score > BestScore)
                    {
                        BestScore = Score;
                        Best = (Nx, Ny);
                    }
                }
            }
            if (!Best.HasValue) return false;
            Previous = (Best.Value.Bx + 0.5 - X, Best.Value.By + 0.5 - Y);
            X = Best.Value.Bx + 0.5;
            Y = Best.Value.By + 0.5;
            Spine.Add(Best.Value);
            return true;
        }

        /// <summary>
        /// True when the step sits beside the ribbon already driven.
        /// The cell we are standing on is the last spine cell and does not count.
        /// Coming back beside an earlier cell is the path folding into a blob.
        /// </summary>
        /// <param name="Nx">Candidate column.</param>
        /// <param name="Ny">Candidate row.</param>
        /// <param name="Spine">Center cells already walked, the current cell last.</param>
        /// <returns>True when the candidate folds into the ribbon.</returns>
        private static Boolean FoldsIntoSpine
            (
            Int32 Nx,
            Int32 Ny,
            List<(Int32 Bx, Int32 By)> Spine
            )
        {
            Int32 Last = Spine.Count - 1;
            for (Int32 Index = 0; Index < Last; Index += 1)
            {
                (Int32 Bx, Int32 By) Cell = Spine[Index];
                if (Math.Max(Math.Abs(Cell.Bx - Nx), Math.Abs(Cell.By - Ny)) <= 1)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>Counts the blade footprint the way <see cref="TakeBlade"/> would, without cutting.</summary>
        /// <param name="X">Station column, in cell coordinates.</param>
        /// <param name="Y">Station row, in cell coordinates.</param>
        /// <param name="Ux">Unit east arrow.</param>
        /// <param name="Uy">Unit north arrow.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="LeftBcy">Volume still allowed on this ribbon.</param>
        /// <param name="Taken">Cells already counted.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Grid">Field grid.</param>
        private static void ProbeBlade
            (
            Double X,
            Double Y,
            Double Ux,
            Double Uy,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double MaxDepthM,
            Double BladeWidthM,
            ref Double LeftBcy,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            ObstacleField Obstacles,
            Grid Grid
            )
        {
            Double Px = -Uy;
            Double Py = Ux;
            Int32 Reach = Math.Max(0, (Int32)Math.Ceiling((BladeWidthM * 0.5) / Grid.GridSizeM));
            for (Int32 Offset = -Reach; Offset <= Reach; Offset += 1)
            {
                if (LeftBcy <= VOLUME_TOLERANCE_BCY) return;
                Int32 Bx = (Int32)Math.Floor(X + (Px * Offset));
                Int32 By = (Int32)Math.Floor(Y + (Py * Offset));
                ProbeTake(Bx, By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Obstacles, Grid);
            }
        }

        /// <summary>Counts one layer of one cell the way <see cref="TakeAt"/> would, without cutting.</summary>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="LeftBcy">Volume still allowed on this ribbon.</param>
        /// <param name="Taken">Cells already counted.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Counted">Cells counted, in the order the blade met them.</param>
        private static void ProbeTake
            (
            Int32 Bx,
            Int32 By,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double MaxDepthM,
            ref Double LeftBcy,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            ObstacleField Obstacles,
            Grid Grid,
            List<((Int32 Bx, Int32 By) Cell, Double Bcy)>? Counted = null
            )
        {
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height)) return;
            if (!Grid.Valid[By, Bx]) return;
            if (Taken.Contains((Bx, By))) return;
            if (InsideObstacle(Grid, Obstacles, Bx, By)) return;
            Double Have = Remaining[By, Bx];
            if (Have <= DEPTH_TOLERANCE_M) return;
            Double DepthM = Math.Min(Have, MaxDepthM);
            Double Bcy = DepthM * BcyPerDepthM;
            if (Bcy > LeftBcy)
            {
                DepthM = LeftBcy / BcyPerDepthM;
                Bcy = LeftBcy;
            }
            if (DepthM <= DEPTH_TOLERANCE_M) return;
            LeftBcy -= Bcy;
            Taken.Add((Bx, By));
            if (Counted != null) Counted.Add(((Bx, By), Bcy));
        }

        /// <summary>
        /// Volume of the piece <see cref="KeepTouching"/> would keep.
        /// A corner chain is left whole, the same way the driven strip is.
        /// </summary>
        /// <param name="Cells">Cells the probe counted, seed first.</param>
        /// <returns>Bank cubic yards on the piece that would be driven.</returns>
        private static Double TouchingVolume
            (
            List<((Int32 Bx, Int32 By) Cell, Double Bcy)> Cells
            )
        {
            if (Cells.Count <= 1)
            {
                return Cells.Count == 0 ? 0.0 : Cells[0].Bcy;
            }
            Dictionary<(Int32 Bx, Int32 By), Int32> At =
                new Dictionary<(Int32 Bx, Int32 By), Int32>();
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                At[Cells[Index].Cell] = Index;
            }
            Boolean[] Seen = new Boolean[Cells.Count];
            List<Int32>? Best = null;
            Boolean BestHasSeed = false;
            (Int32 Dx, Int32 Dy)[] Sides = { (1, 0), (-1, 0), (0, 1), (0, -1) };
            for (Int32 Start = 0; Start < Cells.Count; Start += 1)
            {
                if (Seen[Start]) continue;
                List<Int32> Component = new List<Int32>();
                Stack<Int32> Pending = new Stack<Int32>();
                Pending.Push(Start);
                Seen[Start] = true;
                while (Pending.Count > 0)
                {
                    Int32 Current = Pending.Pop();
                    Component.Add(Current);
                    (Int32 Bx, Int32 By) Cell = Cells[Current].Cell;
                    foreach ((Int32 Dx, Int32 Dy) Step in Sides)
                    {
                        (Int32 Bx, Int32 By) Neighbor = (Cell.Bx + Step.Dx, Cell.By + Step.Dy);
                        if (!At.TryGetValue(Neighbor, out Int32 NeighborIndex)) continue;
                        if (Seen[NeighborIndex]) continue;
                        Seen[NeighborIndex] = true;
                        Pending.Push(NeighborIndex);
                    }
                }
                Boolean HasSeed = Component.Contains(0);
                if ((Best == null)
                    || (Component.Count > Best.Count)
                    || ((Component.Count == Best.Count) && HasSeed && !BestHasSeed))
                {
                    Best = Component;
                    BestHasSeed = HasSeed;
                }
            }
            if ((Best == null) || (Best.Count == Cells.Count))
            {
                return Cells.Sum(Entry => Entry.Bcy);
            }
            // Same refusal as KeepTouching: do not shatter a corner chain.
            if ((Best.Count < 2) || (Best.Count * 2 < Cells.Count))
            {
                return Cells.Sum(Entry => Entry.Bcy);
            }
            Double Volume = 0.0;
            foreach (Int32 Index in Best)
            {
                Volume += Cells[Index].Bcy;
            }
            return Volume;
        }

        /// <summary>Cell steps between two cells. A diagonal counts as one step.</summary>
        /// <param name="A">First cell.</param>
        /// <param name="B">Second cell.</param>
        /// <returns>The Chebyshev distance in cells.</returns>
        private static Int32 Chebyshev
            (
            (Int32 Bx, Int32 By) A,
            (Int32 Bx, Int32 By) B
            )
        {
            return Math.Max(Math.Abs(A.Bx - B.Bx), Math.Abs(A.By - B.By));
        }

        /// <summary>
        /// Closest approach of two ribbons, counting only shared edges.
        /// One means the strips share a side. Two is a corner or a gap.
        /// </summary>
        /// <param name="Left">Cells already on this load.</param>
        /// <param name="Right">Cells of the strip just grown.</param>
        /// <returns>The smallest Manhattan gap. One is a shared edge.</returns>
        private static Int32 MinManhattan
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Left,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Right
            )
        {
            Int32 Best = Int32.MaxValue;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) A in Left)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) B in Right)
                {
                    Int32 Gap = Math.Abs(A.Cell.Bx - B.Cell.Bx) + Math.Abs(A.Cell.By - B.Cell.By);
                    if (Gap < Best) Best = Gap;
                    if (Best <= 1) return Best;
                }
            }
            return Best;
        }

        /// <summary>
        /// Length and width of a ribbon in cells. Length is the drive from
        /// the first cell to the last. Width is how far the cells sit off that line.
        /// A folded patch has a short length and a large width.
        /// </summary>
        /// <param name="Cells">Cells in drive order.</param>
        /// <returns>The length and the cross-track width.</returns>
        private static (Double Length, Double Width) RibbonExtent
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            if (Cells.Count <= 1) return (Cells.Count, Cells.Count);
            Double Ax = Cells[0].Cell.Bx;
            Double Ay = Cells[0].Cell.By;
            Double EndX = Cells[Cells.Count - 1].Cell.Bx;
            Double EndY = Cells[Cells.Count - 1].Cell.By;
            Double Dx = EndX - Ax;
            Double Dy = EndY - Ay;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            // Folded back onto the start. Treat it as a patch.
            if (Length < 1.0) return (1.0, Math.Sqrt(Cells.Count));
            Double Ux = Dx / Length;
            Double Uy = Dy / Length;
            Double Min = 0.0;
            Double Max = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                Double Cross = ((Entry.Cell.Bx - Ax) * -Uy) + ((Entry.Cell.By - Ay) * Ux);
                if (Cross < Min) Min = Cross;
                if (Cross > Max) Max = Cross;
            }
            return (Length, Max - Min);
        }

        /// <summary>
        /// Takes the open cells beside the ribbon, one blade over, in drive order.
        /// A new streamline diverges across the pocket and the two passes become a blob.
        /// </summary>
        /// <param name="Ribbon">Cells already on this load, in drive order.</param>
        /// <param name="Remaining">Depth still available. Reduced by what this pass takes.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="QuotaBcy">Volume this pass may still take.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="OnRibbon">Cells already on this load. Not modified.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The return pass. Empty when nothing beside the ribbon is still open.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> ParallelPass
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ribbon,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double QuotaBcy,
            Double MaxDepthM,
            HashSet<(Int32 Bx, Int32 By)> OnRibbon,
            ObstacleField Obstacles,
            Grid Grid
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> More =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            // A copy, so a rejected pass can be refunded without hiding those cells.
            HashSet<(Int32 Bx, Int32 By)> Taken = new HashSet<(Int32 Bx, Int32 By)>(OnRibbon);
            Double LeftBcy = QuotaBcy;
            (Int32 Sx, Int32 Sy)? Side = null;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Ribbon)
            {
                if (LeftBcy <= VOLUME_TOLERANCE_BCY) break;
                (Int32 Bx, Int32 By)? Pick = null;
                if (Side.HasValue)
                {
                    Int32 Nx = Entry.Cell.Bx + Side.Value.Sx;
                    Int32 Ny = Entry.Cell.By + Side.Value.Sy;
                    // Stay on the side already chosen. Switching sides wraps the patch.
                    if (BesideOpen(Nx, Ny, Remaining, Taken, Obstacles, Grid))
                    {
                        Pick = (Nx, Ny);
                    }
                }
                if (!Pick.HasValue)
                {
                    Double BestDepth = -1.0;
                    for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                    {
                        for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                        {
                            // Corners do not share an edge. The blade cannot cross them.
                            if ((Dx != 0) && (Dy != 0)) continue;
                            if ((Dx == 0) && (Dy == 0)) continue;
                            Int32 Nx = Entry.Cell.Bx + Dx;
                            Int32 Ny = Entry.Cell.By + Dy;
                            if (!BesideOpen(Nx, Ny, Remaining, Taken, Obstacles, Grid)) continue;
                            if (Remaining[Ny, Nx] <= BestDepth) continue;
                            BestDepth = Remaining[Ny, Nx];
                            Pick = (Nx, Ny);
                        }
                    }
                }
                if (!Pick.HasValue) continue;
                // A hole in the pass. The cells after it are a different strip.
                // A corner still counts. The cell between the corners is taken below.
                if ((More.Count > 0) && (Chebyshev(More[More.Count - 1].Cell, Pick.Value) > 1))
                {
                    continue;
                }
                Int32 Before = More.Count;
                TakeAt(
                    Pick.Value.Bx,
                    Pick.Value.By,
                    Remaining,
                    BcyPerDepthM,
                    MaxDepthM,
                    ref LeftBcy,
                    Taken,
                    More,
                    Obstacles,
                    Grid);
                if (More.Count == Before) continue;
                Side = (Pick.Value.Bx - Entry.Cell.Bx, Pick.Value.By - Entry.Cell.By);
            }
            // Two cells that only meet at a corner. Take the cell beside both
            // so this pass shares edges the way the blade actually cuts.
            Int32 Laid = More.Count;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ordered =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Laid);
            for (Int32 Index = 0; Index < Laid; Index += 1)
            {
                if ((Index > 0) && (LeftBcy > VOLUME_TOLERANCE_BCY))
                {
                    (Int32 Bx, Int32 By) A = More[Index - 1].Cell;
                    (Int32 Bx, Int32 By) B = More[Index].Cell;
                    Boolean Diagonal = (Math.Max(Math.Abs(A.Bx - B.Bx), Math.Abs(A.By - B.By)) == 1)
                        && (A.Bx != B.Bx)
                        && (A.By != B.By);
                    if (Diagonal)
                    {
                        List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Bridges =
                            new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                        TakeAt(A.Bx, B.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Bridges, Obstacles, Grid);
                        TakeAt(B.Bx, A.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Bridges, Obstacles, Grid);
                        Ordered.AddRange(Bridges);
                    }
                }
                Ordered.Add(More[Index]);
            }
            return Ordered;
        }

        /// <summary>True when a neighboring cell can still be cut or filled.</summary>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Taken">Cells already on this load.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>True when the cell is open.</returns>
        private static Boolean BesideOpen
            (
            Int32 Bx,
            Int32 By,
            Double[,] Remaining,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            ObstacleField Obstacles,
            Grid Grid
            )
        {
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height)) return false;
            if (!Grid.Valid[By, Bx]) return false;
            if (Taken.Contains((Bx, By))) return false;
            if (InsideObstacle(Grid, Obstacles, Bx, By)) return false;
            return Remaining[By, Bx] > DEPTH_TOLERANCE_M;
        }

        /// <summary>
        /// True when adding the next strip still leaves one blade-wide band.
        /// The axis is the longer strip. A return pass ends beside where it
        /// started, so measuring first-to-last on the pair calls the length a width.
        /// </summary>
        /// <param name="Existing">Cells already on this load.</param>
        /// <param name="More">The strip just grown.</param>
        /// <returns>True when the union is still a strip.</returns>
        private static Boolean StaysAStrip
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Existing,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> More
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Axis =
                (Existing.Count >= More.Count) ? Existing : More;
            (Double Length, Double Width) Extent = RibbonExtent(Axis);
            Double Band = Math.Max(Extent.Width, CrossSpread(Axis, Existing, More));
            if (Band > MAX_RIBBON_WIDTH_CELLS) return false;
            // A patch that is nearly square is not a cut the blade can drive.
            if ((Extent.Length > 0.0) && (Band > (Extent.Length * 0.40))) return false;
            return true;
        }

        /// <summary>
        /// How far the cells sit off the axis strip, in cells.
        /// A pass beside the ribbon stays within a few blade widths.
        /// </summary>
        /// <param name="Axis">Strip that defines the drive direction.</param>
        /// <param name="Left">Cells already on this load.</param>
        /// <param name="Right">Cells of the strip just grown.</param>
        /// <returns>The cross-track spread of both sets.</returns>
        private static Double CrossSpread
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Axis,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Left,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Right
            )
        {
            if (Axis.Count <= 1) return Math.Sqrt(Left.Count + Right.Count);
            Double Ax = Axis[0].Cell.Bx;
            Double Ay = Axis[0].Cell.By;
            Double Dx = Axis[Axis.Count - 1].Cell.Bx - Ax;
            Double Dy = Axis[Axis.Count - 1].Cell.By - Ay;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            if (Length < 1.0) return Math.Sqrt(Left.Count + Right.Count);
            Double Ux = Dx / Length;
            Double Uy = Dy / Length;
            Double Min = 0.0;
            Double Max = 0.0;
            // Both sets. The axis cells define the band the new strip has to stay inside.
            AccumulateCross(Left, Ax, Ay, Ux, Uy, ref Min, ref Max);
            AccumulateCross(Right, Ax, Ay, Ux, Uy, ref Min, ref Max);
            return Max - Min;
        }

        /// <summary>Extends the cross-track range with one set of cells.</summary>
        /// <param name="Cells">Cells to measure.</param>
        /// <param name="Ax">Axis start column.</param>
        /// <param name="Ay">Axis start row.</param>
        /// <param name="Ux">Unit east of the axis.</param>
        /// <param name="Uy">Unit north of the axis.</param>
        /// <param name="Min">Lowest cross-track offset. Updated.</param>
        /// <param name="Max">Highest cross-track offset. Updated.</param>
        private static void AccumulateCross
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double Ax,
            Double Ay,
            Double Ux,
            Double Uy,
            ref Double Min,
            ref Double Max
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                Double Cross = ((Entry.Cell.Bx - Ax) * -Uy) + ((Entry.Cell.By - Ay) * Ux);
                if (Cross < Min) Min = Cross;
                if (Cross > Max) Max = Cross;
            }
        }

        /// <summary>Steps downstream until fill or the field ends.</summary>
        /// <param name="Bx">Start column.</param>
        /// <param name="By">Start row.</param>
        /// <param name="FillLeft">Remaining fill depth.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Fill">Fill centroid the ribbon steers toward.</param>
        /// <returns>Cell steps, or a large number when no fill is reached.</returns>
        private static Double DownstreamDistance
            (
            Int32 Bx,
            Int32 By,
            Double[,] FillLeft,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            (Double X, Double Y) Fill
            )
        {
            Double X = Bx + 0.5;
            Double Y = By + 0.5;
            for (Int32 Step = 0; Step < 400; Step += 1)
            {
                (Double Ux, Double Uy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
                if ((Ux * Ux) + (Uy * Uy) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                {
                    return 100000.0;
                }
                X += TRACE_STEP_CELLS * Ux;
                Y += TRACE_STEP_CELLS * Uy;
                Int32 Cx = (Int32)Math.Floor(X);
                Int32 Cy = (Int32)Math.Floor(Y);
                if ((Cx < 0) || (Cy < 0) || (Cx >= Grid.Width) || (Cy >= Grid.Height) || !Grid.Valid[Cy, Cx])
                {
                    return 100000.0;
                }
                if (FillLeft[Cy, Cx] > DEPTH_TOLERANCE_M) return Step;
            }
            return 100000.0;
        }

        /// <summary>
        /// Cells that touch the outer field boundary. A sharp turn on one of
        /// these is the headland. The same turn inside the field doubles back.
        /// </summary>
        /// <param name="Grid">Field grid.</param>
        /// <returns>True for each valid cell within one cell of the outside.</returns>
        private static Boolean[,] FieldEdgeCells
            (
            Grid Grid
            )
        {
            FieldEdgePolicy Policy = new FieldEdgePolicy(Grid, Grid.GridSizeM);
            Boolean[,] Edge = new Boolean[Grid.Height, Grid.Width];
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx]) continue;
                    Edge[By, Bx] = Policy.IsEdgeWorkCell((Bx, By));
                }
            }
            return Edge;
        }

        /// <summary>
        /// True when a trace point, in cell coordinates, is on the outer boundary.
        /// </summary>
        /// <param name="FieldEdge">Edge cells from <see cref="FieldEdgeCells"/>.</param>
        /// <param name="X">Column coordinate.</param>
        /// <param name="Y">Row coordinate.</param>
        /// <returns>True when the point is a headland cell, or already off the grid.</returns>
        private static Boolean OnFieldEdge
            (
            Boolean[,] FieldEdge,
            Double X,
            Double Y
            )
        {
            Int32 Bx = (Int32)Math.Floor(X);
            Int32 By = (Int32)Math.Floor(Y);
            if ((Bx < 0) || (By < 0) || (By >= FieldEdge.GetLength(0)) || (Bx >= FieldEdge.GetLength(1)))
            {
                return true;
            }
            return FieldEdge[By, Bx];
        }

        /// <summary>
        /// True when the next step comes back toward where this pass started.
        /// The pass has to keep moving away, so it stays one strip from the
        /// high ground down to the haul instead of circling the pocket.
        /// </summary>
        /// <param name="OriginX">Start column, in cell coordinates.</param>
        /// <param name="OriginY">Start row, in cell coordinates.</param>
        /// <param name="X">Current column.</param>
        /// <param name="Y">Current row.</param>
        /// <param name="NextX">Column the step would move to.</param>
        /// <param name="NextY">Row the step would move to.</param>
        /// <returns>True when the step should not be taken.</returns>
        private static Boolean ReturnsToStart
            (
            Double OriginX,
            Double OriginY,
            Double X,
            Double Y,
            Double NextX,
            Double NextY
            )
        {
            Double AwayX = X - OriginX;
            Double AwayY = Y - OriginY;
            Double Away2 = (AwayX * AwayX) + (AwayY * AwayY);
            // The first cells are still the start. A turn there is the strip leaving the edge.
            if (Away2 <= (RETURN_CLEAR_CELLS * RETURN_CLEAR_CELLS)) return false;
            Double StepX = NextX - X;
            Double StepY = NextY - Y;
            return ((StepX * AwayX) + (StepY * AwayY)) < 0.0;
        }

        /// <summary>
        /// True when a pass finishes beside where it started after reaching much farther.
        /// That is an interior loop. The same corner on the field edge is a headland turn.
        /// </summary>
        /// <param name="Cells">Cut or fill cells in drive order.</param>
        /// <param name="FieldEdge">Cells on the outer boundary, where a sharp turn is a headland.</param>
        /// <returns>True when this pass should be refused and another start tried.</returns>
        private static Boolean FoldsBack
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Boolean[,] FieldEdge
            )
        {
            if (Cells.Count < 4) return false;
            Double Ax = Cells[0].Cell.Bx;
            Double Ay = Cells[0].Cell.By;
            Double Dx = Cells[Cells.Count - 1].Cell.Bx - Ax;
            Double Dy = Cells[Cells.Count - 1].Cell.By - Ay;
            Double Chord = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Double Far = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                Double Ex = Entry.Cell.Bx - Ax;
                Double Ey = Entry.Cell.By - Ay;
                Double Dist = Math.Sqrt((Ex * Ex) + (Ey * Ey));
                if (Dist > Far) Far = Dist;
            }
            // The finish came back beside the start. A curve whose far end
            // is still the far end is a strip, and it stays.
            if (!((Far > (Chord + RETURN_CLEAR_CELLS)) && (Chord < (Far * 0.45))))
            {
                return false;
            }
            // The corner is the far cell. The leg after it may follow the field
            // edge. Coming back through the field is the double-back.
            Int32 ApexIndex = 0;
            Double ApexDist = -1.0;
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                Double Ex = Cells[Index].Cell.Bx - Ax;
                Double Ey = Cells[Index].Cell.By - Ay;
                Double Dist = (Ex * Ex) + (Ey * Ey);
                if (Dist > ApexDist)
                {
                    ApexDist = Dist;
                    ApexIndex = Index;
                }
            }
            for (Int32 Index = ApexIndex; Index < Cells.Count; Index += 1)
            {
                (Int32 Bx, Int32 By) Cell = Cells[Index].Cell;
                if (!FieldEdge[Cell.By, Cell.Bx]) return true;
            }
            return false;
        }

        /// <summary>
        /// True when untaken soil continues along a heading for the given distance.
        /// A short gap is allowed. Soil only at the start does not count.
        /// </summary>
        /// <param name="X">Start column, in cell coordinates.</param>
        /// <param name="Y">Start row, in cell coordinates.</param>
        /// <param name="Hx">East heading. It does not have to be a unit vector.</param>
        /// <param name="Hy">North heading. It does not have to be a unit vector.</param>
        /// <param name="Taken">Cells already on this load.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="DistanceCells">How far the soil must still reach, in cells.</param>
        /// <returns>True when the heading still runs through soil at that distance.</returns>
        private static Boolean FlowContinues
            (
            Double X,
            Double Y,
            Double Hx,
            Double Hy,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            Double[,] Remaining,
            Grid Grid,
            ObstacleField Obstacles,
            Double DistanceCells
            )
        {
            Double Length = Math.Sqrt((Hx * Hx) + (Hy * Hy));
            if (Length <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) return false;
            Hx /= Length;
            Hy /= Length;
            Int32 Steps = Math.Max(1, (Int32)Math.Round(DistanceCells / TRACE_STEP_CELLS));
            Int32 LastSoil = 0;
            for (Int32 Step = 1; Step <= Steps; Step += 1)
            {
                Int32 Bx = (Int32)Math.Floor(X + (Step * TRACE_STEP_CELLS * Hx));
                Int32 By = (Int32)Math.Floor(Y + (Step * TRACE_STEP_CELLS * Hy));
                if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height)) return false;
                if (!Grid.Valid[By, Bx]) return false;
                Boolean Soil = !Taken.Contains((Bx, By))
                    && !InsideObstacle(Grid, Obstacles, Bx, By)
                    && (Remaining[By, Bx] > DEPTH_TOLERANCE_M);
                if (Soil)
                {
                    LastSoil = Step;
                }
                else if ((Step - LastSoil) >= MISS_GAP)
                {
                    return false;
                }
            }
            // Soil has to reach the far end of the look, not merely the first cell.
            return LastSoil >= Math.Max(1, Steps - 1);
        }

        /// <summary>
        /// True when soil lies along the crest, perpendicular to the downhill flow.
        /// </summary>
        /// <param name="DownX">East component of the downhill Poisson arrow.</param>
        /// <param name="DownY">North component of the downhill Poisson arrow.</param>
        /// <param name="X">Column, in cell coordinates.</param>
        /// <param name="Y">Row, in cell coordinates.</param>
        /// <param name="HeadingX">East component of the current heading.</param>
        /// <param name="HeadingY">North component of the current heading.</param>
        /// <param name="Taken">Cells already on this load.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <returns>True when either side of the crest still has soil.</returns>
        private static Boolean CrestHasSoil
            (
            Double DownX,
            Double DownY,
            Double X,
            Double Y,
            Double HeadingX,
            Double HeadingY,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            Double[,] Remaining,
            Grid Grid,
            ObstacleField Obstacles
            )
        {
            return TryCrestHeading(
                DownX, DownY, HeadingX, HeadingY, X, Y,
                Taken, Remaining, Grid, Obstacles,
                out _, out _);
        }

        /// <summary>
        /// Crest heading perpendicular to the downhill flow. The side that still
        /// has soil is used, and when both do, the one closer to the way the
        /// machine is already heading.
        /// </summary>
        /// <param name="DownX">East component of the downhill Poisson arrow.</param>
        /// <param name="DownY">North component of the downhill Poisson arrow.</param>
        /// <param name="HeadingX">East component of the current heading.</param>
        /// <param name="HeadingY">North component of the current heading.</param>
        /// <param name="X">Column, in cell coordinates.</param>
        /// <param name="Y">Row, in cell coordinates.</param>
        /// <param name="Taken">Cells already on this load.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <returns>Unit heading along the crest.</returns>
        private static (Double X, Double Y) CrestHeading
            (
            Double DownX,
            Double DownY,
            Double HeadingX,
            Double HeadingY,
            Double X,
            Double Y,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            Double[,] Remaining,
            Grid Grid,
            ObstacleField Obstacles
            )
        {
            TryCrestHeading(
                DownX, DownY, HeadingX, HeadingY, X, Y,
                Taken, Remaining, Grid, Obstacles,
                out Double CrestX, out Double CrestY);
            return (CrestX, CrestY);
        }

        /// <summary>
        /// Picks the crest direction that still has soil.
        /// </summary>
        /// <param name="DownX">East component of the downhill Poisson arrow.</param>
        /// <param name="DownY">North component of the downhill Poisson arrow.</param>
        /// <param name="HeadingX">East component of the current heading.</param>
        /// <param name="HeadingY">North component of the current heading.</param>
        /// <param name="X">Column, in cell coordinates.</param>
        /// <param name="Y">Row, in cell coordinates.</param>
        /// <param name="Taken">Cells already on this load.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="CrestX">East component of the crest heading when one exists.</param>
        /// <param name="CrestY">North component of the crest heading when one exists.</param>
        /// <returns>True when a crest side still has soil.</returns>
        private static Boolean TryCrestHeading
            (
            Double DownX,
            Double DownY,
            Double HeadingX,
            Double HeadingY,
            Double X,
            Double Y,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            Double[,] Remaining,
            Grid Grid,
            ObstacleField Obstacles,
            out Double CrestX,
            out Double CrestY
            )
        {
            CrestX = HeadingX;
            CrestY = HeadingY;
            Double Length = Math.Sqrt((DownX * DownX) + (DownY * DownY));
            if (Length <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) return false;
            Double Ax = -DownY / Length;
            Double Ay = DownX / Length;
            Double Bx = -Ax;
            Double By = -Ay;
            // The crest cell beside this one is enough to start the curve.
            Boolean ASoil = FlowContinues(
                X, Y, Ax, Ay, Taken, Remaining, Grid, Obstacles, 1.0);
            Boolean BSoil = FlowContinues(
                X, Y, Bx, By, Taken, Remaining, Grid, Obstacles, 1.0);
            if (!ASoil && !BSoil) return false;
            if (ASoil && !BSoil)
            {
                CrestX = Ax;
                CrestY = Ay;
                return true;
            }
            if (BSoil && !ASoil)
            {
                CrestX = Bx;
                CrestY = By;
                return true;
            }
            Double DotA = (HeadingX * Ax) + (HeadingY * Ay);
            Double DotB = (HeadingX * Bx) + (HeadingY * By);
            if (DotA >= DotB)
            {
                CrestX = Ax;
                CrestY = Ay;
            }
            else
            {
                CrestX = Bx;
                CrestY = By;
            }
            return true;
        }

        /// <summary>
        /// Largest heading change one trace step may make while the blade is down.
        /// The field edge may use the turning circle. A turn inside a cut or a
        /// fill uses a wider radius, because a tight turn is slow.
        /// </summary>
        /// <param name="Grid">Field grid. The step length is a fraction of one cell.</param>
        /// <param name="OnEdge">True when this step is on the outer field boundary.</param>
        /// <param name="MinimumRadiusM">Turning-circle radius in metres.</param>
        /// <returns>Degrees. A tiny radius reports no turn.</returns>
        private static Double BladeTurnLimitDeg
            (
            Grid Grid,
            Boolean OnEdge,
            Double MinimumRadiusM
            )
        {
            Double RadiusM = OnEdge
                ? MinimumRadiusM
                : MinimumRadiusM * Constants.INTERIOR_BLADE_TURN_SCALE;
            Double StepM = TRACE_STEP_CELLS * Grid.GridSizeM;
            if (RadiusM <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return 0.0;
            }
            return (StepM / RadiusM) * (180.0 / Math.PI);
        }

        /// <summary>
        /// Turns a heading toward a target, by at most the given number of degrees.
        /// </summary>
        /// <param name="Hx">East component of the current heading.</param>
        /// <param name="Hy">North component of the current heading.</param>
        /// <param name="Tx">East component of the target heading.</param>
        /// <param name="Ty">North component of the target heading.</param>
        /// <param name="MaxDegrees">Largest turn to apply.</param>
        /// <returns>The turned unit heading.</returns>
        private static (Double X, Double Y) TurnToward
            (
            Double Hx,
            Double Hy,
            Double Tx,
            Double Ty,
            Double MaxDegrees
            )
        {
            Double Cross = (Hx * Ty) - (Hy * Tx);
            Double Dot = (Hx * Tx) + (Hy * Ty);
            Double Degrees = Math.Atan2(Cross, Dot) * (180.0 / Math.PI);
            if (Degrees > MaxDegrees) Degrees = MaxDegrees;
            if (Degrees < -MaxDegrees) Degrees = -MaxDegrees;
            return Rotate(Hx, Hy, Degrees);
        }

        /// <summary>
        /// True when downhill flow from a point passes through the mouth.
        /// Soil that misses the mouth runs out to a different fill.
        /// </summary>
        /// <param name="X">Column of the point, in cell coordinates.</param>
        /// <param name="Y">Row of the point, in cell coordinates.</param>
        /// <param name="MouthBx">Column of the mouth.</param>
        /// <param name="MouthBy">Row of the mouth.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Fill">Fill centroid, used only where the arrow vanishes.</param>
        /// <returns>
        /// True when the walk comes within <see cref="SAME_MOUTH_CELLS"/> of the mouth.
        /// </returns>
        private static Boolean DrainsToMouth
            (
            Double X,
            Double Y,
            Int32 MouthBx,
            Int32 MouthBy,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            (Double X, Double Y) Fill
            )
        {
            Double MouthX = MouthBx + 0.5;
            Double MouthY = MouthBy + 0.5;
            Double Apart = Math.Max(Math.Abs(X - MouthX), Math.Abs(Y - MouthY));
            // A curved line is longer than the straight gap. Past this budget
            // the walk has left for another basin.
            Int32 Budget = (Int32)Math.Ceiling((3.0 * Apart / TRACE_STEP_CELLS) + 8.0);
            if (Budget < 12) Budget = 12;
            if (Budget > 160) Budget = 160;
            for (Int32 Step = 0; Step < Budget; Step += 1)
            {
                Int32 Cx = (Int32)Math.Floor(X);
                Int32 Cy = (Int32)Math.Floor(Y);
                if ((Cx < 0) || (Cy < 0) || (Cx >= Grid.Width) || (Cy >= Grid.Height) || !Grid.Valid[Cy, Cx])
                {
                    return false;
                }
                if (Math.Max(Math.Abs(Cx - MouthBx), Math.Abs(Cy - MouthBy)) <= SAME_MOUTH_CELLS)
                {
                    return true;
                }
                (Double Ux, Double Uy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
                Double Length = Math.Sqrt((Ux * Ux) + (Uy * Uy));
                if (Length <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                {
                    return false;
                }
                X += TRACE_STEP_CELLS * Ux / Length;
                Y += TRACE_STEP_CELLS * Uy / Length;
            }
            return false;
        }

        /// <summary>
        /// Continues a load after its Poisson line ends. While the uphill
        /// direction still has soil, the pan turns toward that line at the
        /// radius it may use. Otherwise the end of the line is a ridge. A cut
        /// follows the crest only while the soil drains back through the mouth.
        /// A fill eases along the valley. Cells are appended at the far end,
        /// so the drive runs back to the crossover.
        /// </summary>
        /// <param name="X">Column of the far end, in cell coordinates. Updated as the search moves.</param>
        /// <param name="Y">Row of the far end, in cell coordinates. Updated as the search moves.</param>
        /// <param name="HeadingX">East component of the heading that reached this end. Updated when the search turns.</param>
        /// <param name="HeadingY">North component of the heading that reached this end. Updated when the search turns.</param>
        /// <param name="LeftBcy">Volume the pan can still take.</param>
        /// <param name="Taken">Cells already on this load.</param>
        /// <param name="FieldEdge">Cells on the outer boundary, where a sharp turn is a headland.</param>
        /// <param name="Remaining">Depth still available. Reduced when <paramref name="Output"/> is set.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Output">Drive-order cells, written when this search is the load itself.</param>
        /// <param name="Counted">Probe cells, written when this search is only measuring the pan.</param>
        /// <param name="Sign">-1 up the flow from a cut. +1 follows the flow into a fill.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Fill">Fill centroid the ribbon steers toward.</param>
        /// <param name="DragAcrossGrade">
        /// When true, a stretch of finished ground does not end the curve.
        /// The blade stays down until the next low cell.
        /// </param>
        /// <param name="CutRemaining">
        /// Cut still to take. A fill curve stops on ground that still needs cutting.
        /// </param>
        /// <param name="MinimumRadiusM">
        /// Tightest blade-down radius, in metres. The field edge may use it.
        /// A bend inside the cut or the fill uses a wider radius.
        /// </param>
        /// <param name="MouthBx">Column of the mouth this ribbon started from.</param>
        /// <param name="MouthBy">Row of the mouth this ribbon started from.</param>
        /// <param name="FlowSlack">
        /// Fraction of the arrow an on-flow step may leave. Zero follows the arrow.
        /// A crest step stays on the heading that still drains through the mouth.
        /// </param>
        /// <param name="CrossRidges">
        /// When true, a short run of still-high ground does not end a fill.
        /// The dirt stays in the bowl until the basin past that ridge.
        /// </param>
        private static void ExtendFromEnd
            (
            ref Double X,
            ref Double Y,
            ref Double HeadingX,
            ref Double HeadingY,
            ref Double LeftBcy,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            Boolean[,] FieldEdge,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double MaxDepthM,
            Double BladeWidthM,
            Grid Grid,
            ObstacleField Obstacles,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>? Output,
            List<((Int32 Bx, Int32 By) Cell, Double Bcy)>? Counted,
            Double Sign,
            Double[,] Fx,
            Double[,] Fy,
            (Double X, Double Y) Fill,
            Boolean DragAcrossGrade,
            Double[,]? CutRemaining,
            Double MinimumRadiusM,
            Int32 MouthBx,
            Int32 MouthBy,
            Double FlowSlack,
            Boolean CrossRidges
            )
        {
            Double HeadingLength = Math.Sqrt((HeadingX * HeadingX) + (HeadingY * HeadingY));
            if (HeadingLength <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return;
            }
            HeadingX /= HeadingLength;
            HeadingY /= HeadingLength;
            Int32 Misses = 0;
            Int32 RidgeSteps = 0;
            for (Int32 Step = 0; Step < MAX_TRACE_STEPS; Step += 1)
            {
                if (LeftBcy <= VOLUME_TOLERANCE_BCY) return;
                (Double Ux, Double Uy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
                Double DownLen = Math.Sqrt((Ux * Ux) + (Uy * Uy));
                Double TravelX = 0.0;
                Double TravelY = 0.0;
                Boolean HaveArrow = DownLen > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE;
                if (HaveArrow)
                {
                    TravelX = Sign * Ux / DownLen;
                    TravelY = Sign * Uy / DownLen;
                }
                Double FlowDot = (HeadingX * TravelX) + (HeadingY * TravelY);
                Boolean FlowHasSoil = HaveArrow
                    && FlowContinues(
                        X, Y, TravelX, TravelY, Taken, Remaining, Grid, Obstacles, 1.0);
                // Uphill of the local arrow is still this streamline. The pan
                // turns toward it at the radius it may use.
                Boolean OnStream = FlowHasSoil && (FlowDot > 0.0);
                // The edge may use the turning circle. Inside a cut or a fill
                // the same bend is wider, because a tight turn is slow.
                Boolean OnEdge = OnFieldEdge(FieldEdge, X, Y);
                Double MaxTurn = BladeTurnLimitDeg(Grid, OnEdge, MinimumRadiusM);
                if (OnStream)
                {
                    (HeadingX, HeadingY) = TurnToward(
                        HeadingX, HeadingY, TravelX, TravelY, MaxTurn);
                }
                else if (!HaveArrow || !CrestHasSoil(
                    Ux, Uy, X, Y, HeadingX, HeadingY, Taken, Remaining, Grid, Obstacles))
                {
                    return;
                }
                else if (Sign < 0.0)
                {
                    (Double CrestX, Double CrestY) = CrestHeading(
                        Ux, Uy, HeadingX, HeadingY, X, Y, Taken, Remaining, Grid, Obstacles);
                    // A ridge that does not come back through this mouth dumps
                    // in a different fill. That soil is the next load.
                    if (!DrainsToMouth(
                        X + CrestX, Y + CrestY, MouthBx, MouthBy, Fx, Fy, Grid, Fill))
                    {
                        return;
                    }
                    (HeadingX, HeadingY) = TurnToward(HeadingX, HeadingY, CrestX, CrestY, MaxTurn);
                }
                else
                {
                    // A fill whose flow line has ended eases along the valley.
                    (Double CrestX, Double CrestY) = CrestHeading(
                        Ux, Uy, HeadingX, HeadingY, X, Y, Taken, Remaining, Grid, Obstacles);
                    (HeadingX, HeadingY) = TurnToward(HeadingX, HeadingY, CrestX, CrestY, MaxTurn);
                }
                // While the ridge is beside the blade, turn toward it before
                // stepping off the end of the flow line into ground with no soil.
                if (!OnStream)
                {
                    for (Int32 Bend = 0; Bend < 6; Bend += 1)
                    {
                        if (FlowContinues(
                            X, Y, HeadingX, HeadingY, Taken, Remaining, Grid, Obstacles, 1.0))
                        {
                            break;
                        }
                        if (!CrestHasSoil(
                            Ux, Uy, X, Y, HeadingX, HeadingY, Taken, Remaining, Grid, Obstacles))
                        {
                            return;
                        }
                        (Double CrestX, Double CrestY) = CrestHeading(
                            Ux, Uy, HeadingX, HeadingY, X, Y, Taken, Remaining, Grid, Obstacles);
                        if ((Sign < 0.0) && !DrainsToMouth(
                            X + CrestX, Y + CrestY, MouthBx, MouthBy, Fx, Fy, Grid, Fill))
                        {
                            return;
                        }
                        (HeadingX, HeadingY) = TurnToward(HeadingX, HeadingY, CrestX, CrestY, MaxTurn);
                    }
                }
                // On the flow, the step is the arrow. The heading catches up at
                // the radius the blade may use, and that lagged heading is
                // what walks a diagonal off its corner cells.
                Double StepX = OnStream ? TravelX : HeadingX;
                Double StepY = OnStream ? TravelY : HeadingY;
                // On the arrow, a full bowl that the line misses may lean toward
                // the soil. The crest stays on the heading that drains here.
                if (OnStream && (FlowSlack > 0.0))
                {
                    (StepX, StepY) = DirectionWithinSlack(
                        StepX, StepY, FlowSlack, X, Y, Taken, Remaining, Grid, Obstacles, BladeWidthM);
                }
                Double NextX = X + (TRACE_STEP_CELLS * StepX);
                Double NextY = Y + (TRACE_STEP_CELLS * StepY);
                // A crest step that has turned away from this mouth would
                // start the pan on soil whose downhill fill is a different one.
                if ((Sign < 0.0) && !OnStream && !DrainsToMouth(
                    NextX, NextY, MouthBx, MouthBy, Fx, Fy, Grid, Fill))
                {
                    return;
                }
                Int32 Cx = (Int32)Math.Floor(NextX);
                Int32 Cy = (Int32)Math.Floor(NextY);
                if ((Cx < 0) || (Cy < 0) || (Cx >= Grid.Width) || (Cy >= Grid.Height) || !Grid.Valid[Cy, Cx])
                {
                    return;
                }
                // Finished grade is driven across. A short leftover ridge is
                // crossed only after cleaner ribbons can no longer fill a bowl.
                // The high cells are not filled and not cut.
                if (DragAcrossGrade && BlocksTheGrade(NextX, NextY, CutRemaining, Grid, Obstacles))
                {
                    RidgeSteps += 1;
                    if (!CrossRidges
                        || !IsCutRidge(NextX, NextY, CutRemaining, Grid, Obstacles)
                        || (RidgeSteps > RIDGE_CROSS_STEPS))
                    {
                        return;
                    }
                }
                else
                {
                    RidgeSteps = 0;
                }
                // Copy: the footprint buffer is reused by the next step.
                List<(Int32 Bx, Int32 By)> Foot = new List<(Int32 Bx, Int32 By)>(
                    BladeFootprint(X, Y, NextX, NextY, BladeWidthM, Grid));
                Boolean TookForward = false;
                foreach ((Int32 Bx, Int32 By) Cell in Foot)
                {
                    if (Taken.Contains(Cell)) continue;
                    if (InsideObstacle(Grid, Obstacles, Cell.Bx, Cell.By)) continue;
                    if (Remaining[Cell.By, Cell.Bx] <= DEPTH_TOLERANCE_M) continue;
                    Double Forward =
                        ((Cell.Bx + 0.5 - X) * StepX)
                        + ((Cell.By + 0.5 - Y) * StepY);
                    if (Forward > 0.5) TookForward = true;
                }
                if (TookForward)
                {
                    RidgeSteps = 0;
                    foreach ((Int32 Bx, Int32 By) Cell in Foot)
                    {
                        if (Output != null)
                        {
                            TakeAt(
                                Cell.Bx, Cell.By, Remaining, BcyPerDepthM, MaxDepthM,
                                ref LeftBcy, Taken, Output, Obstacles, Grid);
                        }
                        else
                        {
                            ProbeTake(
                                Cell.Bx, Cell.By, Remaining, BcyPerDepthM, MaxDepthM,
                                ref LeftBcy, Taken, Obstacles, Grid, Counted);
                        }
                    }
                    Misses = 0;
                }
                else
                {
                    Misses += 1;
                    // A cut ends when the soil stops. A fill keeps grading
                    // across cells that are already at target.
                    if (!DragAcrossGrade && (Misses >= MISS_GAP)) return;
                }
                X = NextX;
                Y = NextY;
            }
        }

        /// <summary>Rotates a unit heading by an angle in degrees.</summary>
        /// <param name="Hx">East component.</param>
        /// <param name="Hy">North component.</param>
        /// <param name="Degrees">Angle to turn. Positive is left of the heading.</param>
        /// <returns>The rotated heading.</returns>
        private static (Double X, Double Y) Rotate
            (
            Double Hx,
            Double Hy,
            Double Degrees
            )
        {
            Double Radians = Degrees * (Math.PI / 180.0);
            Double Cos = Math.Cos(Radians);
            Double Sin = Math.Sin(Radians);
            return ((Hx * Cos) - (Hy * Sin), (Hx * Sin) + (Hy * Cos));
        }

        /// <summary>
        /// True when a fill must not drag across this point. An obstacle is
        /// closed, and a cell that still needs cutting is not finished grade.
        /// </summary>
        /// <param name="X">Column, in cell coordinates.</param>
        /// <param name="Y">Row, in cell coordinates.</param>
        /// <param name="CutRemaining">Cut still to take. Null when this walk is a cut.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <returns>True when the blade should lift and stop.</returns>
        private static Boolean BlocksTheGrade
            (
            Double X,
            Double Y,
            Double[,]? CutRemaining,
            Grid Grid,
            ObstacleField Obstacles
            )
        {
            Int32 Bx = (Int32)Math.Floor(X);
            Int32 By = (Int32)Math.Floor(Y);
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height)) return true;
            if (!Grid.Valid[By, Bx]) return true;
            if (InsideObstacle(Grid, Obstacles, Bx, By)) return true;
            // Still-high ground is the next cut, not a finished pad to drag across.
            return (CutRemaining != null)
                && (CutRemaining[By, Bx] > DEPTH_TOLERANCE_M);
        }

        /// <summary>
        /// True when the point is leftover cut a fill may cross. The field
        /// edge, a hole in the grid, and an obstacle are not a ridge.
        /// </summary>
        /// <param name="X">Column, in cell coordinates.</param>
        /// <param name="Y">Row, in cell coordinates.</param>
        /// <param name="CutRemaining">Cut still to take.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <returns>True when the cell still needs cutting and the blade may pass it.</returns>
        private static Boolean IsCutRidge
            (
            Double X,
            Double Y,
            Double[,]? CutRemaining,
            Grid Grid,
            ObstacleField Obstacles
            )
        {
            Int32 Bx = (Int32)Math.Floor(X);
            Int32 By = (Int32)Math.Floor(Y);
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height)) return false;
            if (!Grid.Valid[By, Bx]) return false;
            if (InsideObstacle(Grid, Obstacles, Bx, By)) return false;
            return (CutRemaining != null)
                && (CutRemaining[By, Bx] > DEPTH_TOLERANCE_M);
        }

        /// <summary>
        /// Gathers one blade-wide ribbon from the crossover. Sign -1 follows
        /// the Poisson flow up the slope. Sign +1 follows it into the fill.
        /// A fill keeps going across cells already at grade. Where that line
        /// ends, a cut follows the ridge only while the soil drains back
        /// through the mouth. A fill eases along the valley.
        /// </summary>
        /// <param name="Seed">Cell the ribbon must pass through.</param>
        /// <param name="Sign">-1 upstream, +1 downstream.</param>
        /// <param name="Remaining">Depth still available. Reduced by what this ribbon takes.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="QuotaBcy">Volume this ribbon may still take.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Fill">Fill centroid the ribbon steers toward.</param>
        /// <param name="FieldEdge">Cells on the outer boundary, where a sharp turn is a headland.</param>
        /// <param name="DragAcrossGrade">
        /// When true, finished cells do not end the ribbon. The blade stays
        /// down and the dirt stays in the bowl until the next low cell.
        /// A cut passes false, so running out of soil still ends that strip.
        /// </param>
        /// <param name="CutRemaining">
        /// Cut still to take. While dragging, a cell that still needs cutting
        /// is not finished grade, and the fill stops there.
        /// </param>
        /// <param name="Already">Cells already on this load. A second layer is not taken.</param>
        /// <param name="MinimumRadiusM">
        /// Tightest blade-down radius, in metres. The field edge may use it.
        /// A bend inside the cut or the fill uses a wider radius.
        /// </param>
        /// <param name="FlowSlack">
        /// Fraction of the arrow a step may leave. Zero follows the arrow.
        /// </param>
        /// <param name="CrossRidges">
        /// When true, a fill keeps going across a short leftover ridge.
        /// The high cells are left for a later cut.
        /// </param>
        /// <returns>Cells in drive order. Upstream growth is reversed so the seed is last.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Grow
            (
            (Int32 Bx, Int32 By) Seed,
            Double Sign,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double QuotaBcy,
            Double MaxDepthM,
            Double BladeWidthM,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            ObstacleField Obstacles,
            (Double X, Double Y) Fill,
            Boolean[,] FieldEdge,
            Boolean DragAcrossGrade,
            Double[,]? CutRemaining,
            HashSet<(Int32 Bx, Int32 By)>? Already,
            Double MinimumRadiusM,
            Double FlowSlack,
            Boolean CrossRidges
            )
        {
            HashSet<(Int32 Bx, Int32 By)> Taken = new HashSet<(Int32 Bx, Int32 By)>();
            if (Already != null)
            {
                foreach ((Int32 Bx, Int32 By) Cell in Already) Taken.Add(Cell);
            }
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Away =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double LeftBcy = QuotaBcy;
            TakeAt(Seed.Bx, Seed.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Away, Obstacles, Grid);
            Double X = Seed.Bx + 0.5;
            Double Y = Seed.By + 0.5;
            foreach ((Int32 Bx, Int32 By) Cell in BladeFootprint(X, Y, X, Y, BladeWidthM, Grid))
            {
                TakeAt(Cell.Bx, Cell.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Away, Obstacles, Grid);
            }
            (Double StartUx, Double StartUy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
            // Sign -1 works back up the Poisson flow. Sign +1 follows it on.
            Double HeadingX = Sign * StartUx;
            Double HeadingY = Sign * StartUy;
            // The backwards search restarts where soil was last taken, not
            // several steps past it into ground that is already at grade.
            Double SoilX = X;
            Double SoilY = Y;
            Double SoilHx = HeadingX;
            Double SoilHy = HeadingY;
            Int32 Misses = 0;
            Int32 ReverseSteps = 0;
            // Still-high steps in a row. A finished cell clears the count,
            // so each leftover ridge is judged on its own.
            Int32 RidgeSteps = 0;
            for (Int32 Step = 0; Step < MAX_TRACE_STEPS; Step += 1)
            {
                if (LeftBcy <= VOLUME_TOLERANCE_BCY) break;
                (Double Ux, Double Uy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
                if ((Ux * Ux) + (Uy * Uy) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) break;
                // Follow the Poisson flow. The blade, not a neighbor search, decides the cells.
                // Once strict ribbons no longer fill a bowl, the step may lean
                // toward soil that is still inside the open cone.
                Double DirX = Sign * Ux;
                Double DirY = Sign * Uy;
                if (FlowSlack > 0.0)
                {
                    (DirX, DirY) = DirectionWithinSlack(
                        DirX, DirY, FlowSlack, X, Y, Taken, Remaining, Grid, Obstacles, BladeWidthM);
                }
                Double NextX = X + (TRACE_STEP_CELLS * DirX);
                Double NextY = Y + (TRACE_STEP_CELLS * DirY);
                Int32 Cx = (Int32)Math.Floor(NextX);
                Int32 Cy = (Int32)Math.Floor(NextY);
                if ((Cx < 0) || (Cy < 0) || (Cx >= Grid.Width) || (Cy >= Grid.Height) || !Grid.Valid[Cy, Cx])
                {
                    break;
                }
                // An interior turn back toward the start is a second pass.
                // A few reversing steps on the field edge are the headland corner.
                if (ReturnsToStart(Seed.Bx + 0.5, Seed.By + 0.5, X, Y, NextX, NextY))
                {
                    Boolean Headland = OnFieldEdge(FieldEdge, X, Y) || OnFieldEdge(FieldEdge, NextX, NextY);
                    ReverseSteps += 1;
                    if (!Headland || (ReverseSteps > HEADLAND_REVERSE_STEPS))
                    {
                        break;
                    }
                }
                else
                {
                    ReverseSteps = 0;
                }
                Int32 Before = Away.Count;
                foreach ((Int32 Bx, Int32 By) Cell in BladeFootprint(X, Y, NextX, NextY, BladeWidthM, Grid))
                {
                    TakeAt(Cell.Bx, Cell.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Away, Obstacles, Grid);
                }
                if (Away.Count > Before)
                {
                    Misses = 0;
                    RidgeSteps = 0;
                    SoilX = NextX;
                    SoilY = NextY;
                    SoilHx = HeadingX;
                    SoilHy = HeadingY;
                }
                else
                {
                    Misses += 1;
                    // The haul already ended at the first low cell. Finished
                    // ground between low cells is graded with the blade down,
                    // so the same haul places the rest of the bowl. A short
                    // leftover ridge is crossed the same way once no cleaner
                    // line can fill the bowl. Nothing is deposited on it.
                    if (DragAcrossGrade)
                    {
                        if (BlocksTheGrade(NextX, NextY, CutRemaining, Grid, Obstacles))
                        {
                            RidgeSteps += 1;
                            if (!CrossRidges
                                || !IsCutRidge(NextX, NextY, CutRemaining, Grid, Obstacles)
                                || (RidgeSteps > RIDGE_CROSS_STEPS))
                            {
                                break;
                            }
                        }
                        else
                        {
                            RidgeSteps = 0;
                        }
                    }
                    else if (Misses >= MISS_GAP)
                    {
                        break;
                    }
                }
                Double StepX = NextX - X;
                Double StepY = NextY - Y;
                Double StepLength = Math.Sqrt((StepX * StepX) + (StepY * StepY));
                if (StepLength > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                {
                    HeadingX = StepX / StepLength;
                    HeadingY = StepY / StepLength;
                    if (Away.Count > Before)
                    {
                        SoilHx = HeadingX;
                        SoilHy = HeadingY;
                    }
                }
                X = NextX;
                Y = NextY;
            }
            // The flow line has ended. Follow the ridge, or the valley on a
            // fill, with a gentle curve. A cut ridge that drains to another
            // mouth is left there. A flow line that can be followed again takes over.
            ExtendFromEnd(
                ref SoilX,
                ref SoilY,
                ref SoilHx,
                ref SoilHy,
                ref LeftBcy,
                Taken,
                FieldEdge,
                Remaining,
                BcyPerDepthM,
                MaxDepthM,
                BladeWidthM,
                Grid,
                Obstacles,
                Away,
                null,
                Sign,
                Fx,
                Fy,
                Fill,
                DragAcrossGrade,
                CutRemaining,
                MinimumRadiusM,
                Seed.Bx,
                Seed.By,
                FlowSlack,
                CrossRidges);
            // A nick beside a cut is not part of the strip. A fill that
            // dragged across finished ground is one pass, and the later low
            // cells are the rest of the bowl, so they stay.
            if (!DragAcrossGrade)
            {
                KeepTouching(Away, Remaining);
            }
            FitToQuota(Away, Remaining, BcyPerDepthM, QuotaBcy);
            // Upstream was collected away from the haul. Drive it back toward the seed.
            if (Sign < 0.0) Away.Reverse();
            return Away;
        }

        /// <summary>
        /// Walks downstream from the cut mouth along the flow until fill
        /// remains, then takes a blade-wide ribbon of that fill up to the cut
        /// volume. The fill is the basin that flow reaches. Cells already at
        /// grade are graded across, so a later low cell on the same flow
        /// still receives dirt. The haul stays the gap to the first low cell.
        /// </summary>
        /// <param name="CutEnd">Downstream end of the cut.</param>
        /// <param name="FillLeft">Remaining fill depth.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of fill depth.</param>
        /// <param name="QuotaBcy">Cut volume that must be matched.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="Fx">Smoothed east arrow.</param>
        /// <param name="Fy">Smoothed north arrow.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Fill">Fill centroid the ribbon steers toward.</param>
        /// <param name="AcceptFold">
        /// When true, a fill that comes back beside its start is kept.
        /// That is only the end of the field, after every straight strip is gone.
        /// </param>
        /// <param name="FieldEdge">Cells on the outer boundary, where a sharp turn is a headland.</param>
        /// <param name="CutLeft">Cut still to take. The fill stops on that ground.</param>
        /// <param name="MinimumRadiusM">
        /// Tightest blade-down radius, in metres. The field edge may use it.
        /// A bend inside the fill uses a wider radius.
        /// </param>
        /// <param name="FlowSlack">
        /// Fraction of the arrow the walk may leave. Zero follows the arrow.
        /// </param>
        /// <param name="CrossRidges">
        /// When true, the fill keeps going across a short leftover ridge
        /// and places the rest of the bowl in the basin beyond it.
        /// </param>
        /// <returns>Fill cells in drive order, nearest the cut first.</returns>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> GrowFill
            (
            (Int32 Bx, Int32 By) CutEnd,
            Double[,] FillLeft,
            Double BcyPerDepthM,
            Double QuotaBcy,
            Double MaxDepthM,
            Double BladeWidthM,
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            ObstacleField Obstacles,
            (Double X, Double Y) Fill,
            Boolean AcceptFold,
            Boolean[,] FieldEdge,
            Double[,] CutLeft,
            Double MinimumRadiusM,
            Double FlowSlack,
            Boolean CrossRidges
            )
        {
            Double X = CutEnd.Bx + 0.5;
            Double Y = CutEnd.By + 0.5;
            (Int32 Bx, Int32 By)? Arrival = null;
            for (Int32 Step = 0; Step < MAX_TRACE_STEPS; Step += 1)
            {
                (Double Ux, Double Uy) = PoissonAhead(Fx, Fy, Grid, X, Y, Fill);
                if ((Ux * Ux) + (Uy * Uy) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) break;
                Double DirX = Ux;
                Double DirY = Uy;
                if (FlowSlack > 0.0)
                {
                    (DirX, DirY) = DirectionWithinSlack(
                        DirX, DirY, FlowSlack, X, Y, null, FillLeft, Grid, Obstacles, BladeWidthM);
                }
                X += TRACE_STEP_CELLS * DirX;
                Y += TRACE_STEP_CELLS * DirY;
                Int32 Cx = (Int32)Math.Floor(X);
                Int32 Cy = (Int32)Math.Floor(Y);
                if ((Cx < 0) || (Cy < 0) || (Cx >= Grid.Width) || (Cy >= Grid.Height) || !Grid.Valid[Cy, Cx])
                {
                    break;
                }
                if (FillLeft[Cy, Cx] > DEPTH_TOLERANCE_M)
                {
                    Arrival = (Cx, Cy);
                    break;
                }
            }
            // The streamline missed the basin. Dump at the nearest fill still open.
            if (!Arrival.HasValue) Arrival = NearestFill(CutEnd, FillLeft, Grid);
            if (!Arrival.HasValue) return new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            // A fill that comes back beside its start is a V. Try the next
            // hole near the cut before giving this load up.
            HashSet<(Int32 Bx, Int32 By)> Tried = new HashSet<(Int32 Bx, Int32 By)>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FromCut =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (CutEnd, 0.0),
                };
            (Int32 Bx, Int32 By)? FoldedAt = null;
            for (Int32 Attempt = 0; Attempt < 8; Attempt += 1)
            {
                if (!Arrival.HasValue) break;
                if (!Tried.Add(Arrival.Value)) break;
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ribbon = Grow(
                    Arrival.Value,
                    Sign: 1.0,
                    FillLeft,
                    BcyPerDepthM,
                    QuotaBcy,
                    MaxDepthM,
                    BladeWidthM,
                    Fx,
                    Fy,
                    Grid,
                    Obstacles,
                    Fill,
                    FieldEdge,
                    DragAcrossGrade: true,
                    CutRemaining: CutLeft,
                    Already: null,
                    MinimumRadiusM,
                    FlowSlack,
                    CrossRidges);
                if (Ribbon.Count == 0)
                {
                    Arrival = AdjacentFill(FromCut, FillLeft, Grid, Tried);
                    continue;
                }
                // One downhill dump. A second strip doubles back, the same
                // way a cut turns around when the soil ahead gets shallow.
                if (!FoldsBack(Ribbon, FieldEdge))
                {
                    return Ribbon;
                }
                Refund(Ribbon, FillLeft);
                if (!FoldedAt.HasValue) FoldedAt = Arrival.Value;
                Arrival = AdjacentFill(FromCut, FillLeft, Grid, Tried);
            }
            // Every straight dump folded. The end of the field keeps the first one.
            if (AcceptFold && FoldedAt.HasValue)
            {
                return Grow(
                    FoldedAt.Value,
                    Sign: 1.0,
                    FillLeft,
                    BcyPerDepthM,
                    QuotaBcy,
                    MaxDepthM,
                    BladeWidthM,
                    Fx,
                    Fy,
                    Grid,
                    Obstacles,
                    Fill,
                    FieldEdge,
                    DragAcrossGrade: true,
                    CutRemaining: CutLeft,
                    Already: null,
                    MinimumRadiusM,
                    FlowSlack,
                    CrossRidges);
            }
            return new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
        }

        /// <summary>
        /// Nearest remaining fill within a few blade widths of the current dump end.
        /// </summary>
        /// <param name="Ribbon">Fill cells already on this load.</param>
        /// <param name="FillLeft">Remaining fill depth.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="OnRibbon">Cells already taken by this load.</param>
        /// <returns>The neighbor, or null when the next hole is not adjacent.</returns>
        private static (Int32 Bx, Int32 By)? AdjacentFill
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Ribbon,
            Double[,] FillLeft,
            Grid Grid,
            HashSet<(Int32 Bx, Int32 By)> OnRibbon
            )
        {
            return NearestOpen(Ribbon, FillLeft, Grid, OnRibbon, FILL_REACH_CELLS);
        }

        /// <summary>Nearest remaining fill cell, so a scrap load can still finish.</summary>
        /// <param name="From">Cut mouth.</param>
        /// <param name="FillLeft">Remaining fill depth.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The cell, or null when no fill remains.</returns>
        private static (Int32 Bx, Int32 By)? NearestFill
            (
            (Int32 Bx, Int32 By) From,
            Double[,] FillLeft,
            Grid Grid
            )
        {
            (Int32 Bx, Int32 By)? Best = null;
            Double BestD = Double.PositiveInfinity;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (FillLeft[By, Bx] <= DEPTH_TOLERANCE_M) continue;
                    if (!Grid.Valid[By, Bx]) continue;
                    Double Dx = Bx - From.Bx;
                    Double Dy = By - From.By;
                    Double D = (Dx * Dx) + (Dy * Dy);
                    if (D < BestD)
                    {
                        BestD = D;
                        Best = (Bx, By);
                    }
                }
            }
            return Best;
        }

        /// <summary>
        /// Fills a corner-only join. The two cells stay, and the cell that shares
        /// an edge with both is taken when it still holds dirt.
        /// </summary>
        /// <param name="Taken">Cells already on this ribbon.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="LeftBcy">Volume still allowed on this ribbon.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Output">Drive-order list. Null counts the volume without storing cells.</param>
        private static void CloseCornerGaps
            (
            HashSet<(Int32 Bx, Int32 By)> Taken,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double MaxDepthM,
            ref Double LeftBcy,
            ObstacleField Obstacles,
            Grid Grid,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>? Output
            )
        {
            Boolean Grew = true;
            while (Grew)
            {
                if (LeftBcy <= VOLUME_TOLERANCE_BCY) return;
                Grew = false;
                List<(Int32 Bx, Int32 By)> Snapshot = new List<(Int32 Bx, Int32 By)>(Taken);
                foreach ((Int32 Bx, Int32 By) Cell in Snapshot)
                {
                    (Int32 Dx, Int32 Dy)[] Diagonals =
                    {
                        (1, 1), (1, -1), (-1, 1), (-1, -1),
                    };
                    foreach ((Int32 Dx, Int32 Dy) Step in Diagonals)
                    {
                        (Int32 Bx, Int32 By) Diag = (Cell.Bx + Step.Dx, Cell.By + Step.Dy);
                        if (!Taken.Contains(Diag)) continue;
                        (Int32 Bx, Int32 By) BridgeA = (Cell.Bx + Step.Dx, Cell.By);
                        (Int32 Bx, Int32 By) BridgeB = (Cell.Bx, Cell.By + Step.Dy);
                        if (Taken.Contains(BridgeA) || Taken.Contains(BridgeB)) continue;
                        if (TakeBridge(BridgeA, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Obstacles, Grid, Output)
                            || TakeBridge(BridgeB, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Obstacles, Grid, Output))
                        {
                            Grew = true;
                        }
                    }
                }
            }
        }

        /// <summary>Takes one bridge cell when it still holds dirt.</summary>
        /// <param name="Cell">Cell that would join two corner-touching cells.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="LeftBcy">Volume still allowed on this ribbon.</param>
        /// <param name="Taken">Cells already on this ribbon.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Output">Drive-order list. Null counts the volume without storing cells.</param>
        /// <returns>True when the cell was added.</returns>
        private static Boolean TakeBridge
            (
            (Int32 Bx, Int32 By) Cell,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double MaxDepthM,
            ref Double LeftBcy,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            ObstacleField Obstacles,
            Grid Grid,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)>? Output
            )
        {
            if (Taken.Contains(Cell)) return false;
            if (Output == null)
            {
                ProbeTake(Cell.Bx, Cell.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Obstacles, Grid);
            }
            else
            {
                TakeAt(Cell.Bx, Cell.By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Output, Obstacles, Grid);
            }
            return Taken.Contains(Cell);
        }

        /// <summary>
        /// Cells covered by the blade on this thread. Reused so a ribbon walk
        /// does not allocate a list at every step. The list is only valid until
        /// the next footprint on the same thread, and the caller finishes
        /// reading it before that call.
        /// </summary>
        [ThreadStatic]
        private static List<(Int32 Bx, Int32 By)>? FootprintCells;

        /// <summary>
        /// Every grid cell the blade overlaps while it moves from one point to the next.
        /// A diagonal sample of three cells leaves gaps. The blade is a solid width.
        /// Cells the centerline clearly covers or clearly misses skip the sample grid.
        /// A cell in between still uses that grid, so the same cells are kept.
        /// </summary>
        /// <param name="FromX">Start column, in cell coordinates.</param>
        /// <param name="FromY">Start row, in cell coordinates.</param>
        /// <param name="ToX">End column, in cell coordinates.</param>
        /// <param name="ToY">End row, in cell coordinates.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>Cells the blade covers on that move.</returns>
        private static List<(Int32 Bx, Int32 By)> BladeFootprint
            (
            Double FromX,
            Double FromY,
            Double ToX,
            Double ToY,
            Double BladeWidthM,
            Grid Grid
            )
        {
            Double Half = (BladeWidthM / Grid.GridSizeM) * 0.5;
            Double Pad = Half + 1.0;
            Int32 X0 = (Int32)Math.Floor(Math.Min(FromX, ToX) - Pad);
            Int32 X1 = (Int32)Math.Floor(Math.Max(FromX, ToX) + Pad);
            Int32 Y0 = (Int32)Math.Floor(Math.Min(FromY, ToY) - Pad);
            Int32 Y1 = (Int32)Math.Floor(Math.Max(FromY, ToY) + Pad);
            X0 = Math.Max(0, X0);
            Y0 = Math.Max(0, Y0);
            X1 = Math.Min(Grid.Width - 1, X1);
            Y1 = Math.Min(Grid.Height - 1, Y1);
            // One list per thread. A new list at every step was the allocation
            // the ribbon walk spent its time on, and the cells are read immediately.
            List<(Int32 Bx, Int32 By)> Cells = FootprintCells ??= new List<(Int32 Bx, Int32 By)>(32);
            Cells.Clear();
            // Squared reach of the sample grid around the cell center. Inside the
            // inner radius every sample is under the blade. Outside the outer
            // radius none is. The band between them is the sample grid.
            Double Inner = Half - COVERAGE_SAMPLE_REACH_CELLS;
            Double InnerSq = (Inner > 0.0) ? (Inner * Inner) : -1.0;
            Double Outer = Half + COVERAGE_SAMPLE_REACH_CELLS;
            Double OuterSq = Outer * Outer;
            for (Int32 By = Y0; By <= Y1; By += 1)
            {
                for (Int32 Bx = X0; Bx <= X1; Bx += 1)
                {
                    // Same rule as the lane builder: the blade has to pass over
                    // the cell, not merely touch a corner.
                    // Sampling sits a hair under a quarter on a diagonal bridge.
                    // That bridge is the cell between two corner-touching centers.
                    if (BladeCoversCell(FromX, FromY, ToX, ToY, Bx, By, Half, InnerSq, OuterSq))
                    {
                        Cells.Add((Bx, By));
                    }
                }
            }
            return Cells;
        }

        /// <summary>
        /// True when the blade covers enough of one cell to take it.
        /// A cell whose center is inside the inner radius is fully covered.
        /// A cell whose center is outside the outer radius is missed.
        /// Anything between those radii uses the sample grid.
        /// </summary>
        /// <param name="FromX">Start column, in cell coordinates.</param>
        /// <param name="FromY">Start row, in cell coordinates.</param>
        /// <param name="ToX">End column, in cell coordinates.</param>
        /// <param name="ToY">End row, in cell coordinates.</param>
        /// <param name="Bx">Cell column.</param>
        /// <param name="By">Cell row.</param>
        /// <param name="Half">Blade half-width in cells.</param>
        /// <param name="InnerSq">Squared radius inside which every sample is covered. Negative when the blade is narrower than the sample reach.</param>
        /// <param name="OuterSq">Squared radius outside which no sample is covered.</param>
        /// <returns>True when this cell belongs on the ribbon.</returns>
        private static Boolean BladeCoversCell
            (
            Double FromX,
            Double FromY,
            Double ToX,
            Double ToY,
            Int32 Bx,
            Int32 By,
            Double Half,
            Double InnerSq,
            Double OuterSq
            )
        {
            Double CenterSq = DistanceSqPointToSegment(
                Bx + 0.5,
                By + 0.5,
                FromX,
                FromY,
                ToX,
                ToY);
            // The sample grid cannot reach this far from the center.
            if (CenterSq > OuterSq) return false;
            // Every sample sits under the blade, so the share is the whole cell.
            if ((InnerSq >= 0.0) && (CenterSq <= InnerSq)) return true;
            return CellCoverage(FromX, FromY, ToX, ToY, Bx, By, Half) >= (MIN_CELL_COVERAGE - 0.02);
        }

        /// <summary>
        /// Checks the footprint shortcut against a full sample grid.
        /// A mismatch would add or drop a cell, and the ribbon would change.
        /// </summary>
        /// <returns>A description of the first cell that disagrees, or null when every case matches.</returns>
        internal static String? CoverageShortcutMismatch
            (
            )
        {
            Double[] Halves = { 0.25, 0.5, 1.0, 2.125, 4.0, 8.0 };
            (Double X, Double Y)[] Offsets =
            {
                (0.0, 0.0),
                (0.45, 0.0),
                (0.45, 0.45),
                (2.0, 0.2),
                (5.0, -3.0),
                (-1.2, 4.0),
            };
            (Double X, Double Y)[] Origins =
            {
                (10.0, 10.0),
                (10.2, 10.7),
                (3.01, 8.4),
            };
            foreach (Double Half in Halves)
            {
                Double Inner = Half - COVERAGE_SAMPLE_REACH_CELLS;
                Double InnerSq = (Inner > 0.0) ? (Inner * Inner) : -1.0;
                Double Outer = Half + COVERAGE_SAMPLE_REACH_CELLS;
                Double OuterSq = Outer * Outer;
                Int32 Pad = (Int32)Math.Ceiling(Half) + 3;
                foreach ((Double X, Double Y) Origin in Origins)
                {
                    foreach ((Double X, Double Y) Offset in Offsets)
                    {
                        Double ToX = Origin.X + Offset.X;
                        Double ToY = Origin.Y + Offset.Y;
                        Int32 X0 = (Int32)Math.Floor(Math.Min(Origin.X, ToX)) - Pad;
                        Int32 X1 = (Int32)Math.Floor(Math.Max(Origin.X, ToX)) + Pad;
                        Int32 Y0 = (Int32)Math.Floor(Math.Min(Origin.Y, ToY)) - Pad;
                        Int32 Y1 = (Int32)Math.Floor(Math.Max(Origin.Y, ToY)) + Pad;
                        for (Int32 By = Y0; By <= Y1; By += 1)
                        {
                            for (Int32 Bx = X0; Bx <= X1; Bx += 1)
                            {
                                Boolean Fast = BladeCoversCell(
                                    Origin.X, Origin.Y, ToX, ToY, Bx, By, Half, InnerSq, OuterSq);
                                Boolean Slow = FullSampleCoverage(
                                    Origin.X, Origin.Y, ToX, ToY, Bx, By, Half)
                                    >= (MIN_CELL_COVERAGE - 0.02);
                                if (Fast == Slow) continue;
                                return String.Format(
                                    System.Globalization.CultureInfo.InvariantCulture,
                                    "half={0} cell={1},{2} from={3},{4} to={5},{6} fast={7} slow={8}",
                                    Half,
                                    Bx,
                                    By,
                                    Origin.X,
                                    Origin.Y,
                                    ToX,
                                    ToY,
                                    Fast,
                                    Slow);
                            }
                        }
                    }
                }
            }
            return null;
        }

        /// <summary>
        /// Full 8 by 8 sample share. This is the coverage the shortcut has to match.
        /// </summary>
        /// <param name="FromX">Start column, in cell coordinates.</param>
        /// <param name="FromY">Start row, in cell coordinates.</param>
        /// <param name="ToX">End column, in cell coordinates.</param>
        /// <param name="ToY">End row, in cell coordinates.</param>
        /// <param name="Bx">Cell column.</param>
        /// <param name="By">Cell row.</param>
        /// <param name="Half">Blade half-width in cells.</param>
        /// <returns>Covered fraction from 0 to 1.</returns>
        private static Double FullSampleCoverage
            (
            Double FromX,
            Double FromY,
            Double ToX,
            Double ToY,
            Int32 Bx,
            Int32 By,
            Double Half
            )
        {
            Double HalfSq = Half * Half;
            Int32 Hits = 0;
            Int32 Samples = COVERAGE_SAMPLES * COVERAGE_SAMPLES;
            for (Int32 Jy = 0; Jy < COVERAGE_SAMPLES; Jy += 1)
            {
                Double Y = By + ((Jy + 0.5) / COVERAGE_SAMPLES);
                for (Int32 Ix = 0; Ix < COVERAGE_SAMPLES; Ix += 1)
                {
                    Double X = Bx + ((Ix + 0.5) / COVERAGE_SAMPLES);
                    if (DistanceSqPointToSegment(X, Y, FromX, FromY, ToX, ToY) <= HalfSq)
                    {
                        Hits += 1;
                    }
                }
            }
            return (Double)Hits / Samples;
        }

        /// <summary>
        /// Share of a cell that lies under the blade while it moves from one point to the next.
        /// The blade is a strip of the given half-width about that segment.
        /// The walk stops once the share has passed the cell test, or once the
        /// remaining samples cannot reach it. Both decisions match a full grid.
        /// </summary>
        /// <param name="FromX">Start column, in cell coordinates.</param>
        /// <param name="FromY">Start row, in cell coordinates.</param>
        /// <param name="ToX">End column, in cell coordinates.</param>
        /// <param name="ToY">End row, in cell coordinates.</param>
        /// <param name="Bx">Cell column.</param>
        /// <param name="By">Cell row.</param>
        /// <param name="Half">Blade half-width in cells.</param>
        /// <returns>Covered fraction from 0 to 1. An early result is on the same side of the cell test as the full grid.</returns>
        private static Double CellCoverage
            (
            Double FromX,
            Double FromY,
            Double ToX,
            Double ToY,
            Int32 Bx,
            Int32 By,
            Double Half
            )
        {
            Double HalfSq = Half * Half;
            Int32 Hits = 0;
            Int32 Examined = 0;
            Int32 Samples = COVERAGE_SAMPLES * COVERAGE_SAMPLES;
            // The footprint keeps a cell at this share. Stopping on either side
            // of it does not add or drop a cell.
            Double Accept = MIN_CELL_COVERAGE - 0.02;
            for (Int32 Jy = 0; Jy < COVERAGE_SAMPLES; Jy += 1)
            {
                Double Y = By + ((Jy + 0.5) / COVERAGE_SAMPLES);
                for (Int32 Ix = 0; Ix < COVERAGE_SAMPLES; Ix += 1)
                {
                    Double X = Bx + ((Ix + 0.5) / COVERAGE_SAMPLES);
                    Examined += 1;
                    if (DistanceSqPointToSegment(X, Y, FromX, FromY, ToX, ToY) <= HalfSq)
                    {
                        Hits += 1;
                    }
                    Double Covered = (Double)Hits / Samples;
                    if (Covered >= Accept) return Covered;
                    // Even if every sample still waiting landed under the blade.
                    if (((Double)(Hits + (Samples - Examined)) / Samples) < Accept)
                    {
                        return Covered;
                    }
                }
            }
            return (Double)Hits / Samples;
        }

        /// <summary>Squared distance from a point to a segment, in cell coordinates.</summary>
        /// <param name="Px">Point column.</param>
        /// <param name="Py">Point row.</param>
        /// <param name="Ax">Segment start column.</param>
        /// <param name="Ay">Segment start row.</param>
        /// <param name="Bx">Segment end column.</param>
        /// <param name="By">Segment end row.</param>
        /// <returns>Squared distance.</returns>
        private static Double DistanceSqPointToSegment
            (
            Double Px,
            Double Py,
            Double Ax,
            Double Ay,
            Double Bx,
            Double By
            )
        {
            Double Dx = Bx - Ax;
            Double Dy = By - Ay;
            Double LengthSq = (Dx * Dx) + (Dy * Dy);
            if (LengthSq <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                Double Ex = Px - Ax;
                Double Ey = Py - Ay;
                return (Ex * Ex) + (Ey * Ey);
            }
            Double T = (((Px - Ax) * Dx) + ((Py - Ay) * Dy)) / LengthSq;
            if (T < 0.0) T = 0.0;
            if (T > 1.0) T = 1.0;
            Double Cx = Ax + (T * Dx) - Px;
            Double Cy = Ay + (T * Dy) - Py;
            return (Cx * Cx) + (Cy * Cy);
        }

        /// <summary>
        /// Squared distance from a segment to a cell square. Zero when the blade touches the cell.
        /// </summary>
        /// <param name="Ax">Segment start column.</param>
        /// <param name="Ay">Segment start row.</param>
        /// <param name="Bx">Segment end column.</param>
        /// <param name="By">Segment end row.</param>
        /// <param name="MinX">Cell west edge.</param>
        /// <param name="MinY">Cell south edge.</param>
        /// <param name="MaxX">Cell east edge.</param>
        /// <param name="MaxY">Cell north edge.</param>
        /// <returns>Squared distance in cell units.</returns>
        private static Double DistanceSqSegmentToSquare
            (
            Double Ax,
            Double Ay,
            Double Bx,
            Double By,
            Double MinX,
            Double MinY,
            Double MaxX,
            Double MaxY
            )
        {
            if (((Ax - Bx) * (Ax - Bx)) + ((Ay - By) * (Ay - By))
                <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return DistanceSqPointToSquare(Ax, Ay, MinX, MinY, MaxX, MaxY);
            }
            if (SegmentHitsSquare(Ax, Ay, Bx, By, MinX, MinY, MaxX, MaxY))
            {
                return 0.0;
            }
            Double Best = Math.Min(
                DistanceSqPointToSquare(Ax, Ay, MinX, MinY, MaxX, MaxY),
                DistanceSqPointToSquare(Bx, By, MinX, MinY, MaxX, MaxY));
            Best = Math.Min(Best, DistanceSqSegmentToSegment(Ax, Ay, Bx, By, MinX, MinY, MaxX, MinY));
            Best = Math.Min(Best, DistanceSqSegmentToSegment(Ax, Ay, Bx, By, MaxX, MinY, MaxX, MaxY));
            Best = Math.Min(Best, DistanceSqSegmentToSegment(Ax, Ay, Bx, By, MaxX, MaxY, MinX, MaxY));
            Best = Math.Min(Best, DistanceSqSegmentToSegment(Ax, Ay, Bx, By, MinX, MaxY, MinX, MinY));
            return Best;
        }

        /// <summary>True when the segment crosses the cell square, including its boundary.</summary>
        /// <param name="Ax">Segment start column.</param>
        /// <param name="Ay">Segment start row.</param>
        /// <param name="Bx">Segment end column.</param>
        /// <param name="By">Segment end row.</param>
        /// <param name="MinX">Cell west edge.</param>
        /// <param name="MinY">Cell south edge.</param>
        /// <param name="MaxX">Cell east edge.</param>
        /// <param name="MaxY">Cell north edge.</param>
        /// <returns>True when the blade centerline enters the cell.</returns>
        private static Boolean SegmentHitsSquare
            (
            Double Ax,
            Double Ay,
            Double Bx,
            Double By,
            Double MinX,
            Double MinY,
            Double MaxX,
            Double MaxY
            )
        {
            // Liang-Barsky. A miss means the centerline stays outside this cell.
            Double T0 = 0.0;
            Double T1 = 1.0;
            Double Dx = Bx - Ax;
            Double Dy = By - Ay;
            if (!ClipSegment(-Dx, Ax - MinX, ref T0, ref T1)) return false;
            if (!ClipSegment(Dx, MaxX - Ax, ref T0, ref T1)) return false;
            if (!ClipSegment(-Dy, Ay - MinY, ref T0, ref T1)) return false;
            if (!ClipSegment(Dy, MaxY - Ay, ref T0, ref T1)) return false;
            return true;
        }

        /// <summary>Clips one axis of a segment against a slab. False when the slab misses the segment.</summary>
        /// <param name="P">Negative of the segment component on this axis.</param>
        /// <param name="Q">Distance from the start to the slab edge.</param>
        /// <param name="T0">Current near clip, updated when the edge cuts the segment.</param>
        /// <param name="T1">Current far clip, updated when the edge cuts the segment.</param>
        /// <returns>False when this slab excludes the segment.</returns>
        private static Boolean ClipSegment
            (
            Double P,
            Double Q,
            ref Double T0,
            ref Double T1
            )
        {
            if (Math.Abs(P) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return Q >= 0.0;
            }
            Double R = Q / P;
            if (P < 0.0)
            {
                if (R > T1) return false;
                if (R > T0) T0 = R;
            }
            else
            {
                if (R < T0) return false;
                if (R < T1) T1 = R;
            }
            return true;
        }

        /// <summary>Squared distance from a point to a cell square.</summary>
        /// <param name="Px">Point column.</param>
        /// <param name="Py">Point row.</param>
        /// <param name="MinX">Cell west edge.</param>
        /// <param name="MinY">Cell south edge.</param>
        /// <param name="MaxX">Cell east edge.</param>
        /// <param name="MaxY">Cell north edge.</param>
        /// <returns>Squared distance in cell units.</returns>
        private static Double DistanceSqPointToSquare
            (
            Double Px,
            Double Py,
            Double MinX,
            Double MinY,
            Double MaxX,
            Double MaxY
            )
        {
            Double Dx = 0.0;
            if (Px < MinX) Dx = MinX - Px;
            else if (Px > MaxX) Dx = Px - MaxX;
            Double Dy = 0.0;
            if (Py < MinY) Dy = MinY - Py;
            else if (Py > MaxY) Dy = Py - MaxY;
            return (Dx * Dx) + (Dy * Dy);
        }

        /// <summary>Squared distance between two segments.</summary>
        /// <param name="Ax">First segment start column.</param>
        /// <param name="Ay">First segment start row.</param>
        /// <param name="Bx">First segment end column.</param>
        /// <param name="By">First segment end row.</param>
        /// <param name="Cx">Second segment start column.</param>
        /// <param name="Cy">Second segment start row.</param>
        /// <param name="Dx">Second segment end column.</param>
        /// <param name="Dy">Second segment end row.</param>
        /// <returns>Squared distance in cell units.</returns>
        private static Double DistanceSqSegmentToSegment
            (
            Double Ax,
            Double Ay,
            Double Bx,
            Double By,
            Double Cx,
            Double Cy,
            Double Dx,
            Double Dy
            )
        {
            Double Ux = Bx - Ax;
            Double Uy = By - Ay;
            Double Vx = Dx - Cx;
            Double Vy = Dy - Cy;
            Double Wx = Ax - Cx;
            Double Wy = Ay - Cy;
            Double A = (Ux * Ux) + (Uy * Uy);
            Double B = (Ux * Vx) + (Uy * Vy);
            Double C = (Vx * Vx) + (Vy * Vy);
            Double D = (Ux * Wx) + (Uy * Wy);
            Double E = (Vx * Wx) + (Vy * Wy);
            Double Denom = (A * C) - (B * B);
            Double SN;
            Double SD = Denom;
            Double TN;
            Double TD = Denom;
            if (Denom <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                SN = 0.0;
                SD = 1.0;
                TN = E;
                TD = C;
            }
            else
            {
                SN = (B * E) - (C * D);
                TN = (A * E) - (B * D);
                if (SN < 0.0)
                {
                    SN = 0.0;
                    TN = E;
                    TD = C;
                }
                else if (SN > SD)
                {
                    SN = SD;
                    TN = E + B;
                    TD = C;
                }
            }
            if (TN < 0.0)
            {
                TN = 0.0;
                if (-D < 0.0) SN = 0.0;
                else if (-D > A)
                {
                    SN = SD;
                }
                else
                {
                    SN = -D;
                    SD = A;
                }
            }
            else if (TN > TD)
            {
                TN = TD;
                if ((-D + B) < 0.0) SN = 0.0;
                else if ((-D + B) > A)
                {
                    SN = SD;
                }
                else
                {
                    SN = (-D + B);
                    SD = A;
                }
            }
            Double SC = Math.Abs(SN) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE ? 0.0 : SN / SD;
            Double TC = Math.Abs(TN) <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE ? 0.0 : TN / TD;
            Double Ex = Wx + (SC * Ux) - (TC * Vx);
            Double Ey = Wy + (SC * Uy) - (TC * Vy);
            return (Ex * Ex) + (Ey * Ey);
        }

        /// <summary>Takes every cell the blade covers at one station, across the flow.</summary>
        /// <param name="X">Station column, in cell coordinates.</param>
        /// <param name="Y">Station row, in cell coordinates.</param>
        /// <param name="Ux">Unit east arrow.</param>
        /// <param name="Uy">Unit north arrow.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="LeftBcy">Volume still allowed on this ribbon.</param>
        /// <param name="Taken">Cells already on this ribbon.</param>
        /// <param name="Output">Drive-order list.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Grid">Field grid.</param>
        private static void TakeBlade
            (
            Double X,
            Double Y,
            Double Ux,
            Double Uy,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double MaxDepthM,
            Double BladeWidthM,
            ref Double LeftBcy,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Output,
            ObstacleField Obstacles,
            Grid Grid
            )
        {
            Double HalfCells = (BladeWidthM * 0.5) / Grid.GridSizeM;
            Double Px = -Uy;
            Double Py = Ux;
            Int32 Reach = Math.Max(0, (Int32)Math.Ceiling(HalfCells));
            for (Int32 Offset = -Reach; Offset <= Reach; Offset += 1)
            {
                if (LeftBcy <= VOLUME_TOLERANCE_BCY) return;
                Int32 Bx = (Int32)Math.Floor(X + (Px * Offset));
                Int32 By = (Int32)Math.Floor(Y + (Py * Offset));
                TakeAt(Bx, By, Remaining, BcyPerDepthM, MaxDepthM, ref LeftBcy, Taken, Output, Obstacles, Grid);
            }
        }

        /// <summary>Takes one layer from one cell when it still holds material.</summary>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="Remaining">Depth still available.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="MaxDepthM">Deepest one pass may take from one cell.</param>
        /// <param name="LeftBcy">Volume still allowed on this ribbon.</param>
        /// <param name="Taken">Cells already on this ribbon.</param>
        /// <param name="Output">Drive-order list.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Grid">Field grid.</param>
        private static void TakeAt
            (
            Int32 Bx,
            Int32 By,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double MaxDepthM,
            ref Double LeftBcy,
            HashSet<(Int32 Bx, Int32 By)> Taken,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Output,
            ObstacleField Obstacles,
            Grid Grid
            )
        {
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height)) return;
            if (!Grid.Valid[By, Bx]) return;
            if (Taken.Contains((Bx, By))) return;
            if (InsideObstacle(Grid, Obstacles, Bx, By)) return;
            Double Have = Remaining[By, Bx];
            if (Have <= DEPTH_TOLERANCE_M) return;
            Double DepthM = Math.Min(Have, MaxDepthM);
            Double Bcy = DepthM * BcyPerDepthM;
            if (Bcy > LeftBcy)
            {
                DepthM = LeftBcy / BcyPerDepthM;
                Bcy = LeftBcy;
            }
            if (DepthM <= DEPTH_TOLERANCE_M) return;
            Remaining[By, Bx] = Have - DepthM;
            LeftBcy -= Bcy;
            Taken.Add((Bx, By));
            Output.Add(((Bx, By), DepthM));
        }

        /// <summary>Puts a whole ribbon back. Used when the other side cannot take it.</summary>
        /// <param name="Cells">Cells to restore.</param>
        /// <param name="Remaining">Depth ledger being restored.</param>
        private static void Refund
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double[,] Remaining
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                Remaining[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
            }
            Cells.Clear();
        }

        /// <summary>
        /// Returns volume until the ribbon matches the other side.
        /// From the start, the ridge is dropped and the haul end stays.
        /// From the end, the downstream tail is dropped and the arrival stays.
        /// </summary>
        /// <param name="Cells">Cells in drive order.</param>
        /// <param name="Remaining">Depth ledger.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="KeepBcy">Volume to keep.</param>
        /// <param name="FromStart">True drops the ridge. False drops the tail.</param>
        /// <returns>Volume still on the ribbon.</returns>
        private static Double RefundExcess
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double KeepBcy,
            Boolean FromStart
            )
        {
            // Keep every cell. Giving one back leaves the next load with a
            // square that only meets its neighbors at a corner.
            ShedDepth(Cells, Remaining, BcyPerDepthM, KeepBcy, FromStart);
            return Cells.Sum(Entry => Entry.DepthM * BcyPerDepthM);
        }

        /// <summary>
        /// Gives extra dirt back until the ribbon fits the bowl.
        /// Cells that keep the strip in one piece stay. The tip is what goes.
        /// </summary>
        /// <param name="Cells">Ribbon in walk order. The far end is last.</param>
        /// <param name="Remaining">Depth ledger.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="QuotaBcy">Most this ribbon may hold.</param>
        private static void FitToQuota
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double QuotaBcy
            )
        {
            RefundExcess(Cells, Remaining, BcyPerDepthM, QuotaBcy, FromStart: false);
        }

        /// <summary>
        /// Keeps the one piece that shares edges and gives the loose cells back.
        /// A square that only touches the strip at a corner is not on this pass.
        /// </summary>
        /// <param name="Cells">Ribbon cells. Loose cells are removed.</param>
        /// <param name="Remaining">Depth ledger the loose cells are returned to.</param>
        private static void KeepTouching
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double[,] Remaining
            )
        {
            if (Cells.Count <= 1) return;
            Dictionary<(Int32 Bx, Int32 By), Int32> At =
                new Dictionary<(Int32 Bx, Int32 By), Int32>();
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                At[Cells[Index].Cell] = Index;
            }
            Boolean[] Seen = new Boolean[Cells.Count];
            List<Int32>? Best = null;
            Boolean BestHasSeed = false;
            (Int32 Dx, Int32 Dy)[] Sides = { (1, 0), (-1, 0), (0, 1), (0, -1) };
            for (Int32 Start = 0; Start < Cells.Count; Start += 1)
            {
                if (Seen[Start]) continue;
                List<Int32> Component = new List<Int32>();
                Stack<Int32> Pending = new Stack<Int32>();
                Pending.Push(Start);
                Seen[Start] = true;
                while (Pending.Count > 0)
                {
                    Int32 Current = Pending.Pop();
                    Component.Add(Current);
                    (Int32 Bx, Int32 By) Cell = Cells[Current].Cell;
                    foreach ((Int32 Dx, Int32 Dy) Step in Sides)
                    {
                        (Int32 Bx, Int32 By) Neighbor = (Cell.Bx + Step.Dx, Cell.By + Step.Dy);
                        if (!At.TryGetValue(Neighbor, out Int32 NeighborIndex)) continue;
                        if (Seen[NeighborIndex]) continue;
                        Seen[NeighborIndex] = true;
                        Pending.Push(NeighborIndex);
                    }
                }
                Boolean HasSeed = Component.Contains(0);
                if ((Best == null)
                    || (Component.Count > Best.Count)
                    || ((Component.Count == Best.Count) && HasSeed && !BestHasSeed))
                {
                    Best = Component;
                    BestHasSeed = HasSeed;
                }
            }
            if ((Best == null) || (Best.Count == Cells.Count)) return;
            // A chain that only meets at corners has no piece bigger than one
            // cell. Splitting it makes one load per cell and the bowl never fills.
            // A small loose end on a real strip is given back.
            if ((Best.Count < 2) || (Best.Count * 2 < Cells.Count)) return;
            HashSet<Int32> Keep = new HashSet<Int32>(Best);
            for (Int32 Index = Cells.Count - 1; Index >= 0; Index -= 1)
            {
                if (Keep.Contains(Index)) continue;
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Cells[Index];
                Remaining[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
                Cells.RemoveAt(Index);
            }
        }

        /// <summary>
        /// Lowers depth on the tip until the ribbon matches the other side.
        /// The cell stays on the ribbon. Deleting it is what left the next
        /// load with squares that only meet at a corner.
        /// </summary>
        /// <param name="Cells">Ribbon cells.</param>
        /// <param name="Remaining">Depth ledger.</param>
        /// <param name="BcyPerDepthM">Bank cubic yards per metre of depth.</param>
        /// <param name="KeepBcy">Volume to keep.</param>
        /// <param name="FromStart">True thins the ridge end first.</param>
        private static void ShedDepth
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double[,] Remaining,
            Double BcyPerDepthM,
            Double KeepBcy,
            Boolean FromStart
            )
        {
            Double Have = Cells.Sum(Entry => Entry.DepthM * BcyPerDepthM);
            // Thin from the tip inward. The cell stays, so the strip stays one piece.
            for (Int32 Step = 0; Step < Cells.Count; Step += 1)
            {
                if (Have <= KeepBcy + MATCH_TOLERANCE_BCY) return;
                Int32 Index = FromStart ? Step : Cells.Count - 1 - Step;
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Cells[Index];
                // A zero depth is rejected by the ledger. Leave a speck so the
                // cell still counts as part of the strip.
                if (Entry.DepthM <= DEPTH_TOLERANCE_M) continue;
                Double DropM = Math.Min(
                    Entry.DepthM - DEPTH_TOLERANCE_M,
                    (Have - KeepBcy) / BcyPerDepthM);
                Remaining[Entry.Cell.By, Entry.Cell.Bx] += DropM;
                Cells[Index] = (Entry.Cell, Entry.DepthM - DropM);
                Have -= DropM * BcyPerDepthM;
            }
        }

        /// <summary>Samples the smoothed arrow at a fractional cell.</summary>
        /// <param name="Fx">East component.</param>
        /// <param name="Fy">North component.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="X">Column coordinate.</param>
        /// <param name="Y">Row coordinate.</param>
        /// <returns>A unit arrow, or zero off the field.</returns>
        private static (Double Ux, Double Uy) Sample
            (
            Double[,] Fx,
            Double[,] Fy,
            Grid Grid,
            Double X,
            Double Y
            )
        {
            Int32 X0 = (Int32)Math.Floor(X);
            Int32 Y0 = (Int32)Math.Floor(Y);
            if ((X0 < 0) || (Y0 < 0) || (X0 >= Grid.Width) || (Y0 >= Grid.Height)) return (0.0, 0.0);
            if (!Grid.Valid[Y0, X0]) return (0.0, 0.0);
            return (Fx[Y0, X0], Fy[Y0, X0]);
        }

        /// <summary>Cell reached by one downstream step from a cell center.</summary>
        /// <param name="X">Column coordinate.</param>
        /// <param name="Y">Row coordinate.</param>
        /// <param name="Arrow">Unit arrow.</param>
        /// <param name="Sign">+1 downstream.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>The cell, or (-1, -1) off the field.</returns>
        private static (Int32 Nx, Int32 Ny) StepCell
            (
            Double X,
            Double Y,
            (Double X, Double Y) Arrow,
            Double Sign,
            Grid Grid
            )
        {
            Double Nx = X + (Sign * Arrow.X);
            Double Ny = Y + (Sign * Arrow.Y);
            Int32 Bx = (Int32)Math.Floor(Nx);
            Int32 By = (Int32)Math.Floor(Ny);
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height) || !Grid.Valid[By, Bx])
            {
                return (-1, -1);
            }
            return (Bx, By);
        }

        /// <summary>True when the cell center sits inside an exclusion.</summary>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Obstacles">Exclusion obstacles.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <returns>True when the ribbon must not enter the cell.</returns>
        private static Boolean InsideObstacle
            (
            Grid Grid,
            ObstacleField Obstacles,
            Int32 Bx,
            Int32 By
            )
        {
            if (!Obstacles.Active) return false;
            (Double X, Double Y) Center = Grid.CellCenterXy(Bx, By);
            return Obstacles.Blocked(Center, Center);
        }

        /// <summary>Sums remaining depth on valid cells.</summary>
        /// <param name="Depth">Depth grid.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>Metres of depth.</returns>
        private static Double Sum
            (
            Double[,] Depth,
            Grid Grid
            )
        {
            Double Total = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx]) Total += Depth[By, Bx];
                }
            }
            return Total;
        }

        /// <summary>Straight distance.</summary>
        /// <param name="A">First point.</param>
        /// <param name="B">Second point.</param>
        /// <returns>Metres.</returns>
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
