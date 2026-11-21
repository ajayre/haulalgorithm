using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

namespace HaulBuilder
{
    /// <summary>
    /// Builds straight coverage runs without modifying passes or production guidance.
    /// </summary>
    static public class CoveragePlanner
    {
        private const Double HEADING_STEP_DEG = 5.0;
        private const Double MAX_ALONG_GAP_GRID_MULTIPLIER = 1.75;
        private const Double RUN_COUNT_WEIGHT = 1000000.0;
        private const Double CONNECTOR_WEIGHT = 100.0;
        private const Double WORK_DISTANCE_WEIGHT = 0.01;
        /// <summary>Heading gap above which the next run is treated as a reverse.</summary>
        private const Double REVERSE_HEADING_SPLIT_DEG = 90.0;
        /// <summary>Degrees in one full circle, for wrapping a heading difference.</summary>
        private const Double FULL_CIRCLE_DEG = 360.0;
        /// <summary>Straight lineup before a coverage run, at the production equipment length.</summary>
        private const Double COVERAGE_LINEUP_M =
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS;
        /// <summary>Straight exit after a coverage run, at the production equipment length.</summary>
        private const Double COVERAGE_EXIT_M =
            Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS;
        /// <summary>Minimum turn radius used to price a reverse between coverage runs.</summary>
        private const Double COVERAGE_MINIMUM_RADIUS_M =
            Constants.DEFAULT_TURNING_CIRCLE_FT
            * Constants.HALF_SCALE
            * Constants.FEET_TO_METERS;
        private const Double OMITTED_CELL_WEIGHT = 1000000000000.0;
        private const Double EDGE_PARALLEL_TOLERANCE_DEG = 7.5;
        private const Double NUMERIC_TOLERANCE = Constants.NUMERIC_COMPARISON_TOLERANCE;
        private const Double ENDPOINT_EXTENSION_GRID_FRACTION = 0.49;
        /// <summary>
        /// Blade-down corner, in degrees, that starts another run.
        /// A wave or a staircase stays one path and is filleted. Only a
        /// fold that turns back is a reversal the scraper must not cut through.
        /// Splitting a milder bend left a gap in the driving line.
        /// </summary>
        private const Double FLOW_BEND_DEG = 120.0;
        /// <summary>
        /// Neighbor-average passes applied to the blade spine.
        /// Two ease a short kink, such as the jog at the start of a cut.
        /// More than that pulls the line off a staircase of cells.
        /// </summary>
        private const Int32 FLOW_SMOOTH_PASSES = 2;
        /// <summary>
        /// A bend harder than this is opened into a short arc. Gentler
        /// bends stay on the cells. The arc itself never steps harder
        /// than <see cref="FLOW_MAX_TURN_DEG"/>.
        /// </summary>
        private const Double FLOW_FILLET_DEG = 35.0;
        /// <summary>
        /// Sharpest single step of a blade-down arc, in degrees.
        /// </summary>
        private const Double FLOW_MAX_TURN_DEG = 60.0;
        /// <summary>
        /// Shortest piece that may be split off at a reversal.
        /// A shorter piece has no room for the turning circle, so the corner
        /// stays on one path and is rounded instead.
        /// </summary>
        private const Double FLOW_MIN_PIECE_M = 20.0;
        /// <summary>
        /// How many grid sizes a cell may sit off the current spine heading
        /// and still belong to that blade step. The footprint is a little
        /// over two cells across. A turn runs past this, and that cell
        /// starts the next step so the line follows the cut.
        /// </summary>
        private const Double FLOW_STATION_SIDE_GRIDS = 2.5;
        private static readonly AsyncLocal<Double?> AmbientMinimumWorkLengthM =
            new AsyncLocal<Double?>();

        /// <summary>
        /// Sets the ambient minimum blade-down length used when callers omit an
        /// explicit gate, typically for one full <see cref="HaulBuilderEngine.Run"/>.
        /// </summary>
        /// <param name="MinimumWorkLengthM">Minimum kept run length in metres.</param>
        /// <returns>Scope that restores the previous ambient value when disposed.</returns>
        static public IDisposable PushMinimumWorkLength
            (
            Double MinimumWorkLengthM
            )
        {
            Double? Previous = AmbientMinimumWorkLengthM.Value;
            AmbientMinimumWorkLengthM.Value = MinimumWorkLengthM;
            return new AmbientMinimumWorkLengthScope(Previous);
        }

        /// <summary>Restores the previous ambient minimum work length.</summary>
        private sealed class AmbientMinimumWorkLengthScope : IDisposable
        {
            private readonly Double? Previous;
            private Boolean Disposed;

            /// <summary>Initializes one ambient restore scope.</summary>
            /// <param name="Previous">Prior ambient value.</param>
            public AmbientMinimumWorkLengthScope
                (
                Double? Previous
                )
            {
                this.Previous = Previous;
            }

            /// <summary>Restores the prior ambient minimum work length.</summary>
            public void Dispose
                (
                )
            {
                if (Disposed)
                {
                    return;
                }
                AmbientMinimumWorkLengthM.Value = Previous;
                Disposed = true;
            }
        }

        /// <summary>
        /// Resolves the effective minimum work length from an explicit argument or ambient scope.
        /// </summary>
        /// <param name="MinimumWorkLengthM">Explicit gate, or null to use ambient/default.</param>
        /// <returns>Effective minimum length in metres; zero disables the gate.</returns>
        private static Double ResolveMinimumWorkLengthM
            (
            Double? MinimumWorkLengthM
            )
        {
            if (MinimumWorkLengthM.HasValue)
            {
                return MinimumWorkLengthM.Value;
            }
            return AmbientMinimumWorkLengthM.Value ?? 0.0;
        }

        /// <summary>
        /// One straight blade-down run and the material cells assigned to it.
        /// </summary>
        public sealed class Run
        {
            /// <summary>
            /// Initializes a straight guidance coverage run.
            /// </summary>
            /// <param name="Cells">Allocated material cells covered by the run.</param>
            /// <param name="StartXy">Centerline start coordinate.</param>
            /// <param name="EndXy">Centerline end coordinate.</param>
            /// <param name="HeadingDeg">Compass heading in degrees.</param>
            /// <param name="MaximumLateralErrorM">Maximum assigned-cell distance from the centerline.</param>
            /// <param name="DrivePoints">
            /// Blade-down centerline. Null keeps the straight chord from
            /// <paramref name="StartXy"/> to <paramref name="EndXy"/>.
            /// </param>
            public Run
                (
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
                (Double X, Double Y) StartXy,
                (Double X, Double Y) EndXy,
                Double HeadingDeg,
                Double MaximumLateralErrorM,
                IReadOnlyList<(Double X, Double Y)>? DrivePoints = null
                )
            {
                // Keep cell ownership explicit so later connector work cannot alter material allocation.
                this.Cells = Cells;
                this.StartXy = StartXy;
                this.EndXy = EndXy;
                this.HeadingDeg = HeadingDeg;
                this.MaximumLateralErrorM = MaximumLateralErrorM;
                this.DrivePoints = DrivePoints;
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

            /// <summary>
            /// Gets the blade-down centerline. Null means drive the straight chord.
            /// </summary>
            public IReadOnlyList<(Double X, Double Y)>? DrivePoints { get; }

            /// <summary>Drive polyline in the opposite direction.</summary>
            /// <returns>Reversed points, or null when this run is a chord.</returns>
            public IReadOnlyList<(Double X, Double Y)>? ReversedDrivePoints
                (
                )
            {
                if (DrivePoints == null) return null;
                List<(Double X, Double Y)> Copy =
                    new List<(Double X, Double Y)>(DrivePoints);
                Copy.Reverse();
                return Copy;
            }

            /// <summary>Gets straight blade-down centerline length in metres.</summary>
            public Double LengthM
            {
                get
                {
                    // Derive run length from endpoints so cached geometry cannot drift.
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
            /// Initializes an guidance coverage result.
            /// </summary>
            /// <param name="HeadingDeg">Selected common sweep heading.</param>
            /// <param name="Runs">Drive-ordered straight coverage runs.</param>
            /// <param name="ConnectorDistanceM">Straight-line distance between coverage runs.</param>
            /// <param name="AnchorDistanceM">Distance to requested start/end anchors.</param>
            /// <param name="Score">Deterministic candidate score.</param>
            /// <param name="OmittedCells">Edge/corner cells that cannot be worked at the required boundary heading.</param>
            /// <param name="ShortRunOmittedCells">Cells omitted because their run was shorter than the minimum work length.</param>
            public PlanResult
                (
                Double HeadingDeg,
                List<Run> Runs,
                Double ConnectorDistanceM,
                Double AnchorDistanceM,
                Double Score,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> OmittedCells,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)>? ShortRunOmittedCells = null
                )
            {
                // Store candidate metrics separately from production timing and turn construction.
                this.HeadingDeg = HeadingDeg;
                this.Runs = Runs;
                this.ConnectorDistanceM = ConnectorDistanceM;
                this.AnchorDistanceM = AnchorDistanceM;
                this.Score = Score;
                this.OmittedCells = OmittedCells;
                this.ShortRunOmittedCells = ShortRunOmittedCells
                    ?? new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
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

            /// <summary>
            /// Gets cells omitted solely because their coverage run was shorter than
            /// the configured minimum blade-down work length.
            /// </summary>
            public List<((Int32 Bx, Int32 By) Cell, Double DepthM)> ShortRunOmittedCells { get; }

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
                    // Rebuild drive order from runs so callers see current allocation order.
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
                    // Include omissions so conservation checks account for every allocation.
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Output = OrderedCells;
                    Output.AddRange(OmittedCells);
                    return Output;
                }
            }
        }

        /// <summary>
        /// Provides projected cell behavior.
        /// </summary>

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
        /// <param name="MinimumWorkLengthM">
        /// Minimum kept blade-down run length in metres; null uses the ambient
        /// engine scope, and zero disables the gate.
        /// </param>
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
            (Double X, Double Y)? OrientEndXy = null,
            Double? MinimumWorkLengthM = null
            )
        {
            List<PlanResult> Candidates = CandidatePlans(
                Cells,
                Grid,
                BladeWidthM,
                OrientStartXy,
                OrientEndXy,
                MinimumWorkLengthM);
            return Candidates[0];
        }

        /// <summary>
        /// Walks cells in the order the flow ribbon stored them. A hop starts a
        /// new run, and so does a corner the scraper cannot cut through. Each
        /// piece is smoothed so the blade-down path bends instead of kinking.
        /// Splitting every gentle bend produced chords the turning circle could
        /// not enter, so only a real reversal is split.
        /// </summary>
        /// <param name="Cells">Ribbon cells, ridge end first.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="TurningCircleFt">
        /// Turning circle diameter in feet. Zero uses the default circle.
        /// The field edge may turn this tight. A bend inside the cut or the
        /// fill uses a wider radius, because a tight turn is slow.
        /// </param>
        /// <returns>One plan whose runs follow the ribbon.</returns>
        static public PlanResult PlanRibbon
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double BladeWidthM,
            Double TurningCircleFt = Constants.DEFAULT_TURNING_CIRCLE_FT
            )
        {
            if (Cells == null) throw new ArgumentNullException(nameof(Cells));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (BladeWidthM <= 0.0) throw new ArgumentOutOfRangeException(nameof(BladeWidthM));
            if (TurningCircleFt <= 0.0) TurningCircleFt = Constants.DEFAULT_TURNING_CIRCLE_FT;
            // Half the circle. The edge may use this. The interior uses more.
            Double MinimumRadiusM = TurningCircleFt
                * Constants.FEET_TO_METERS
                * Constants.HALF_SCALE;
            if (Cells.Count == 0)
            {
                return new PlanResult(
                    0.0,
                    new List<Run>(),
                    0.0,
                    0.0,
                    0.0,
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>());
            }

            List<Run> Runs = new List<Run>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Current =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double HeadingDeg = 0.0;
            Boolean HasHeading = false;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(Entry.Cell.Bx, Entry.Cell.By);
                if (Current.Count == 0)
                {
                    Current.Add(Entry);
                    HasHeading = false;
                    continue;
                }
                (Double X, Double Y) Previous = Grid.CellCenterXy(
                    Current[Current.Count - 1].Cell.Bx,
                    Current[Current.Count - 1].Cell.By);
                Double Dx = Point.X - Previous.X;
                Double Dy = Point.Y - Previous.Y;
                Double StepM = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                // A neighbor strip stays on this run. Only a real hop lifts the blade.
                Boolean Gap = StepM > (Grid.GridSizeM * 6.0);
                if (Gap)
                {
                    AddFlowingRuns(Runs, Current, Grid, HeadingDeg, MinimumRadiusM);
                    Current = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { Entry };
                    HasHeading = false;
                    continue;
                }
                if (!HasHeading && (StepM > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE))
                {
                    HeadingDeg = HeadingOf(Dx, Dy);
                    HasHeading = true;
                }
                Current.Add(Entry);
            }
            if (Current.Count > 0)
            {
                AddFlowingRuns(Runs, Current, Grid, HeadingDeg, MinimumRadiusM);
            }
            // A hop that keeps the same heading is still one cut. Lifting the
            // blade there inserts a turn in the middle of a straight strip.
            MergeAlignedRuns(Runs);
            Double ConnectorM = 0.0;
            for (Int32 Index = 1; Index < Runs.Count; Index += 1)
            {
                Double Dx = Runs[Index].StartXy.X - Runs[Index - 1].EndXy.X;
                Double Dy = Runs[Index].StartXy.Y - Runs[Index - 1].EndXy.Y;
                ConnectorM += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            Double WorkM = Runs.Sum(Run => Run.LengthM);
            return new PlanResult(
                Runs.Count == 0 ? 0.0 : Runs[0].HeadingDeg,
                Runs,
                ConnectorM,
                0.0,
                WorkM + ConnectorM,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>());
        }

        /// <summary>One blade footprint on a flow ribbon.</summary>
        private struct FlowStation
        {
            /// <summary>Footprint center, easting.</summary>
            public Double X;

            /// <summary>Footprint center, northing.</summary>
            public Double Y;

            /// <summary>First ribbon cell in this footprint.</summary>
            public Int32 FirstCell;

            /// <summary>Cell index just past this footprint.</summary>
            public Int32 LastCell;
        }

        /// <summary>
        /// Splits a ribbon where it folds back, then smooths each piece.
        /// The cell list records the whole blade, so a raw join from the mouth
        /// of one strip to the ridge of the next is a V the scraper cannot cut.
        /// A corner on the field edge stays in the same run and stays sharp.
        /// </summary>
        /// <param name="Runs">Runs already accepted for this ribbon.</param>
        /// <param name="Cells">Cells of one continuous visit, in drive order.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="HeadingDeg">Fallback heading when the piece is a single cell.</param>
        /// <param name="MinimumRadiusM">Turning-circle radius in metres.</param>
        private static void AddFlowingRuns
            (
            List<Run> Runs,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double HeadingDeg,
            Double MinimumRadiusM
            )
        {
            List<FlowStation> Stations = DropSideSteps(BladeStations(Cells, Grid), Grid.GridSizeM);
            if (Stations.Count == 0) return;
            // Built once. A corner on this boundary stays a sharp headland turn.
            FieldEdgePolicy Edge = new FieldEdgePolicy(Grid, Grid.GridSizeM);
            Int32 Piece = 0;
            for (Int32 Index = 1; Index < Stations.Count - 1; Index += 1)
            {
                // A fold that turns back is a new run only when both pieces are
                // long enough for the turning circle. A wave is rounded in
                // place. Splitting that wave left a gap through the cells.
                // A corner on the field edge is left sharp, in this same run.
                if (Edge.IsFieldPointNearOuterEdge(Stations[Index].X, Stations[Index].Y))
                {
                    continue;
                }
                if (StationTurnDeg(Stations[Index - 1], Stations[Index], Stations[Index + 1])
                    <= FLOW_BEND_DEG)
                {
                    continue;
                }
                if ((StationLengthM(Stations, Piece, Index) < FLOW_MIN_PIECE_M)
                    || (StationLengthM(Stations, Index, Stations.Count - 1) < FLOW_MIN_PIECE_M))
                {
                    continue;
                }
                EmitFlowRun(Runs, Cells, Grid, Stations, Piece, Index, HeadingDeg, Edge, MinimumRadiusM);
                Piece = Index;
            }
            EmitFlowRun(Runs, Cells, Grid, Stations, Piece, Stations.Count - 1, HeadingDeg, Edge, MinimumRadiusM);
        }

        /// <summary>Adds one smoothed blade-down run for a station range.</summary>
        /// <param name="Runs">Runs already accepted for this ribbon.</param>
        /// <param name="Cells">Cells of the visit.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Stations">Blade footprints in drive order.</param>
        /// <param name="From">First station of this piece.</param>
        /// <param name="To">Last station of this piece.</param>
        /// <param name="HeadingDeg">Fallback heading when the piece is a single cell.</param>
        /// <param name="Edge">Outer boundary. A corner on it may use the turning circle.</param>
        /// <param name="MinimumRadiusM">Turning-circle radius in metres.</param>
        private static void EmitFlowRun
            (
            List<Run> Runs,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            List<FlowStation> Stations,
            Int32 From,
            Int32 To,
            Double HeadingDeg,
            FieldEdgePolicy Edge,
            Double MinimumRadiusM
            )
        {
            // The corner station already belongs to the previous piece.
            // Starting again on it would redraw the kink.
            Int32 DrawFrom = (From == 0) ? From : From + 1;
            if (DrawFrom > To) return;
            Int32 CellFrom = Stations[DrawFrom].FirstCell;
            Int32 CellTo = Stations[To].LastCell;
            if ((CellFrom < 0) || (CellTo > Cells.Count) || (CellFrom >= CellTo)) return;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Slice =
                Cells.GetRange(CellFrom, CellTo - CellFrom);
            List<(Double X, Double Y)> Spine = new List<(Double X, Double Y)>();
            for (Int32 Index = DrawFrom; Index <= To; Index += 1)
            {
                Spine.Add((Stations[Index].X, Stations[Index].Y));
            }
            // Round the spine, then replace any leftover corner with an arc
            // the turning circle can actually drive.
            Double SpacingM = Math.Max(0.5, Grid.GridSizeM * 0.5);
            List<(Double X, Double Y)> Flowing = ResampleFlow(
                FilletFlow(SmoothFlow(Spine, Edge), MinimumRadiusM, SpacingM, Edge),
                SpacingM);
            Runs.Add(MakeRibbonRun(Slice, Grid, HeadingDeg, Flowing));
        }

        /// <summary>Builds one run from a ribbon segment.</summary>
        /// <param name="Cells">Cells in drive order.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="HeadingDeg">Segment heading. Zero when the segment is a single cell.</param>
        /// <param name="Centerline">
        /// Smoothed blade-down path. Null rebuilds a spine from the cells.
        /// </param>
        /// <returns>The run.</returns>
        private static Run MakeRibbonRun
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double HeadingDeg,
            List<(Double X, Double Y)>? Centerline = null
            )
        {
            (Double X, Double Y) Start = Grid.CellCenterXy(Cells[0].Cell.Bx, Cells[0].Cell.By);
            (Double X, Double Y) End = Grid.CellCenterXy(
                Cells[Cells.Count - 1].Cell.Bx,
                Cells[Cells.Count - 1].Cell.By);
            List<(Double X, Double Y)> Points = Centerline
                ?? RibbonCenterline(Cells, Grid, Start, End);
            if (Points.Count >= 1) Start = Points[0];
            if (Points.Count >= 2)
            {
                End = Points[Points.Count - 1];
            }
            else
            {
                // A single cell still needs a direction so the turning circle can enter it.
                End = (Start.X + Math.Sin(HeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN),
                    Start.Y + Math.Cos(HeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN));
                Points.Add(End);
            }
            (Double X, Double Y) Tail = Points[Points.Count - 1];
            (Double X, Double Y) Before = Points[Points.Count - 2];
            Double StepX = Tail.X - Before.X;
            Double StepY = Tail.Y - Before.Y;
            if (((StepX * StepX) + (StepY * StepY))
                > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                // The haul leaves the strip at this end, so the stored
                // heading has to be the arrival heading, not the first step.
                HeadingDeg = HeadingOf(StepX, StepY);
            }
            return new Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Cells),
                Start,
                End,
                HeadingDeg,
                0.0,
                Points);
        }

        /// <summary>
        /// Joins two ribbon runs when the step between them continues.
        /// The check uses where the next run starts, not where it finishes,
        /// so a later curve does not leave a gap through the cells.
        /// The blade stays down instead of lifting for a few metres.
        /// </summary>
        /// <param name="Runs">Runs in drive order. Updated in place.</param>
        private static void MergeAlignedRuns
            (
            List<Run> Runs
            )
        {
            Int32 Index = 1;
            while (Index < Runs.Count)
            {
                Run Previous = Runs[Index - 1];
                Run Next = Runs[Index];
                if ((Previous.DrivePoints == null) || (Next.DrivePoints == null)
                    || (Previous.DrivePoints.Count < 2) || (Next.DrivePoints.Count < 2))
                {
                    Index += 1;
                    continue;
                }
                Double Dx = Next.StartXy.X - Previous.EndXy.X;
                Double Dy = Next.StartXy.Y - Previous.EndXy.Y;
                Double GapM = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (GapM <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                {
                    Index += 1;
                    continue;
                }
                // The last drawn step can be a hook aimed off the strip. The
                // two pieces still travel the same way, and the hole between
                // them is the break in the cut. A fold that turns back stays
                // two runs.
                Double ArrivalDeg = PathHeadingDeg(Previous.DrivePoints, FromEnd: true);
                Double DepartDeg = PathHeadingDeg(Next.DrivePoints, FromEnd: false);
                if ((GapM > 12.0)
                    || (HeadingGapDeg(ArrivalDeg, DepartDeg) > 120.0))
                {
                    Index += 1;
                    continue;
                }
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Previous.Cells);
                Cells.AddRange(Next.Cells);
                List<(Double X, Double Y)> Points =
                    new List<(Double X, Double Y)>(Previous.DrivePoints);
                Points.AddRange(Next.DrivePoints);
                Runs[Index - 1] = new Run(
                    Cells,
                    Points[0],
                    Points[Points.Count - 1],
                    Next.HeadingDeg,
                    0.0,
                    Points);
                Runs.RemoveAt(Index);
            }
        }

        /// <summary>
        /// Heading of a drive path, measured over a few metres so one short
        /// fillet sample does not decide it.
        /// </summary>
        /// <param name="Points">Path in drive order.</param>
        /// <param name="FromEnd">True reads back from the last point. False reads forward from the first.</param>
        /// <returns>Compass heading in degrees.</returns>
        private static Double PathHeadingDeg
            (
            IReadOnlyList<(Double X, Double Y)> Points,
            Boolean FromEnd
            )
        {
            // One cell of a normal grid. Far enough to get off a fillet sample.
            const Double LookM = 3.0;
            if (FromEnd)
            {
                (Double X, Double Y) End = Points[Points.Count - 1];
                for (Int32 Index = Points.Count - 2; Index >= 0; Index -= 1)
                {
                    Double StepX = End.X - Points[Index].X;
                    Double StepY = End.Y - Points[Index].Y;
                    if (((StepX * StepX) + (StepY * StepY)) >= (LookM * LookM))
                    {
                        return HeadingOf(StepX, StepY);
                    }
                }
                (Double X, Double Y) Start = Points[0];
                return HeadingOf(End.X - Start.X, End.Y - Start.Y);
            }
            (Double X, Double Y) Begin = Points[0];
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double StepX = Points[Index].X - Begin.X;
                Double StepY = Points[Index].Y - Begin.Y;
                if (((StepX * StepX) + (StepY * StepY)) >= (LookM * LookM))
                {
                    return HeadingOf(StepX, StepY);
                }
            }
            (Double X, Double Y) Last = Points[Points.Count - 1];
            return HeadingOf(Last.X - Begin.X, Last.Y - Begin.Y);
        }

        /// <summary>Smallest angle between two compass headings.</summary>
        /// <param name="FirstDeg">First heading.</param>
        /// <param name="SecondDeg">Second heading.</param>
        /// <returns>Degrees from 0 to 180.</returns>
        private static Double HeadingGapDeg
            (
            Double FirstDeg,
            Double SecondDeg
            )
        {
            Double Gap = Math.Abs(FirstDeg - SecondDeg) % FULL_CIRCLE_DEG;
            if (Gap > Constants.DEGREES_PER_HALF_TURN) Gap = FULL_CIRCLE_DEG - Gap;
            return Gap;
        }

        /// <summary>Compass heading of a step, with 0 at north and 90 at east.</summary>
        /// <param name="Dx">East component.</param>
        /// <param name="Dy">North component.</param>
        /// <returns>Heading in degrees.</returns>
        private static Double HeadingOf
            (
            Double Dx,
            Double Dy
            )
        {
            return (Math.Atan2(Dx, Dy) * Constants.DEGREES_PER_HALF_TURN / Math.PI
                + Constants.DEGREES_PER_TURN) % Constants.DEGREES_PER_TURN;
        }

        /// <summary>
        /// Collapses a blade-wide ribbon to the spine the scraper actually drives.
        /// Each step records the whole blade, so drawing every cell scribbles sideways.
        /// The ends stay on the spine. Pinning them to the outer blade cells forks the cut.
        /// </summary>
        /// <param name="Cells">Ribbon cells in drive order.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Start">Unused. Kept so callers that have no spine still compile.</param>
        /// <param name="End">Unused. The spine end is the last footprint, not an edge cell.</param>
        /// <returns>Smoothed centerline.</returns>
        private static List<(Double X, Double Y)> RibbonCenterline
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            (Double X, Double Y) Start,
            (Double X, Double Y) End
            )
        {
            _ = Start;
            _ = End;
            List<FlowStation> Stations = DropSideSteps(BladeStations(Cells, Grid), Grid.GridSizeM);
            List<(Double X, Double Y)> Spine = new List<(Double X, Double Y)>();
            foreach (FlowStation Station in Stations)
            {
                Spine.Add((Station.X, Station.Y));
            }
            if (Spine.Count == 0) Spine.Add(Start);
            Double SpacingM = Math.Max(0.5, Grid.GridSizeM * 0.5);
            FieldEdgePolicy Edge = new FieldEdgePolicy(Grid, Grid.GridSizeM);
            return ResampleFlow(
                FilletFlow(SmoothFlow(Spine, Edge), COVERAGE_MINIMUM_RADIUS_M, SpacingM, Edge),
                SpacingM);
        }

        /// <summary>
        /// Groups the blade's left, center, and right cells into one station.
        /// A station ends when the next cell has moved about one cell forward,
        /// or has left the blade to the side. Cells beside the blade stay in
        /// the same station, so the line is the middle of the cut. A turn
        /// starts a new step, so the line follows the cut instead of keeping
        /// the opening heading.
        /// </summary>
        /// <param name="Cells">Ribbon cells in drive order.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>Footprints in drive order.</returns>
        private static List<FlowStation> BladeStations
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            // One cell of travel is a new step. Averaging several steps
            // together is what pulled the line inside a staircase.
            Double StepM = Grid.GridSizeM * 0.85;
            // Wider than the blade, narrower than the next leg of a turn.
            Double SideM = Grid.GridSizeM * FLOW_STATION_SIDE_GRIDS;
            List<FlowStation> Stations = new List<FlowStation>();
            Int32 StartIndex = 0;
            (Double X, Double Y) Origin = (0.0, 0.0);
            (Double X, Double Y) Forward = RibbonForward(Cells, Grid);
            Double SumX = 0.0;
            Double SumY = 0.0;
            Int32 Count = 0;
            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(
                    Cells[Index].Cell.Bx,
                    Cells[Index].Cell.By);
                if (Count > 0)
                {
                    Double Along = ((Point.X - Origin.X) * Forward.X)
                        + ((Point.Y - Origin.Y) * Forward.Y);
                    // Perpendicular to the heading. The other edge of the
                    // blade stays inside SideM. A turn does not, and the
                    // opening heading must not keep averaging that leg
                    // into one footprint.
                    Double Across = Math.Abs(
                        ((Point.X - Origin.X) * Forward.Y)
                        - ((Point.Y - Origin.Y) * Forward.X));
                    if ((Along > StepM) || (Across > SideM))
                    {
                        Stations.Add(new FlowStation
                        {
                            X = SumX / Count,
                            Y = SumY / Count,
                            FirstCell = StartIndex,
                            LastCell = Index,
                        });
                        // The next step faces from this station toward the cell
                        // that just left it, so a curve is not stuck on the
                        // opening heading.
                        Double StepX = Point.X - Origin.X;
                        Double StepY = Point.Y - Origin.Y;
                        Double StepLen = Math.Sqrt((StepX * StepX) + (StepY * StepY));
                        if (StepLen > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                        {
                            Forward = (StepX / StepLen, StepY / StepLen);
                        }
                        StartIndex = Index;
                        Origin = Point;
                        SumX = 0.0;
                        SumY = 0.0;
                        Count = 0;
                    }
                }
                else
                {
                    Origin = Point;
                }
                SumX += Point.X;
                SumY += Point.Y;
                Count += 1;
            }
            if (Count > 0)
            {
                Stations.Add(new FlowStation
                {
                    X = SumX / Count,
                    Y = SumY / Count,
                    FirstCell = StartIndex,
                    LastCell = Cells.Count,
                });
            }
            return Stations;
        }

        /// <summary>
        /// Opening direction of a ribbon. A cell a couple of widths ahead is
        /// down the cut. The first neighbour can be the other edge of the blade.
        /// </summary>
        /// <param name="Cells">Ribbon cells in drive order.</param>
        /// <param name="Grid">Field grid.</param>
        /// <returns>A unit step along the cut.</returns>
        private static (Double X, Double Y) RibbonForward
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid
            )
        {
            if (Cells.Count < 2) return (1.0, 0.0);
            (Double X, Double Y) Start = Grid.CellCenterXy(Cells[0].Cell.Bx, Cells[0].Cell.By);
            Double ReachM = Grid.GridSizeM * 2.0;
            for (Int32 Index = 1; Index < Cells.Count; Index += 1)
            {
                (Double X, Double Y) Point = Grid.CellCenterXy(
                    Cells[Index].Cell.Bx,
                    Cells[Index].Cell.By);
                Double Dx = Point.X - Start.X;
                Double Dy = Point.Y - Start.Y;
                Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Length > ReachM)
                {
                    return (Dx / Length, Dy / Length);
                }
            }
            (Double X, Double Y) End = Grid.CellCenterXy(
                Cells[Cells.Count - 1].Cell.Bx,
                Cells[Cells.Count - 1].Cell.By);
            Double EndX = End.X - Start.X;
            Double EndY = End.Y - Start.Y;
            Double EndLen = Math.Sqrt((EndX * EndX) + (EndY * EndY));
            if (EndLen <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                return (1.0, 0.0);
            }
            return (EndX / EndLen, EndY / EndLen);
        }

        /// <summary>
        /// Drops a footprint that steps sideways and comes straight back.
        /// That step is the blade edge, and drawing it makes the cut zigzag.
        /// A staircase corner turns through a right angle and is kept, so the
        /// line still visits that cell.
        /// </summary>
        /// <param name="Stations">Footprints in drive order.</param>
        /// <param name="GridM">Grid size in metres.</param>
        /// <returns>Spine with the sideways steps removed.</returns>
        private static List<FlowStation> DropSideSteps
            (
            List<FlowStation> Stations,
            Double GridM
            )
        {
            if (Stations.Count < 3) return Stations;
            Double MaxLegM = GridM * 2.5;
            List<FlowStation> Kept = new List<FlowStation> { Stations[0] };
            for (Int32 Index = 1; Index < Stations.Count - 1; Index += 1)
            {
                FlowStation Prev = Kept[Kept.Count - 1];
                Double InX = Stations[Index].X - Prev.X;
                Double InY = Stations[Index].Y - Prev.Y;
                Double OutX = Stations[Index + 1].X - Stations[Index].X;
                Double OutY = Stations[Index + 1].Y - Stations[Index].Y;
                Double InM = Math.Sqrt((InX * InX) + (InY * InY));
                Double OutM = Math.Sqrt((OutX * OutX) + (OutY * OutY));
                Double Turn = StationTurnDeg(Prev, Stations[Index], Stations[Index + 1]);
                // A turn past a right angle on a short leg came back on itself.
                // That is the blade edge. The staircase of the cut is a
                // right angle or less, and the line has to follow it.
                if ((Turn > 90.0) && (InM < MaxLegM) && (OutM < MaxLegM))
                {
                    continue;
                }
                Kept.Add(Stations[Index]);
            }
            Kept.Add(Stations[Stations.Count - 1]);
            return Kept;
        }

        /// <summary>Length of a station range, measured along the footprints.</summary>
        /// <param name="Stations">Footprints in drive order.</param>
        /// <param name="From">First station index.</param>
        /// <param name="To">Last station index.</param>
        /// <returns>Metres along the spine.</returns>
        private static Double StationLengthM
            (
            List<FlowStation> Stations,
            Int32 From,
            Int32 To
            )
        {
            Double Length = 0.0;
            for (Int32 Index = From + 1; Index <= To; Index += 1)
            {
                Double Dx = Stations[Index].X - Stations[Index - 1].X;
                Double Dy = Stations[Index].Y - Stations[Index - 1].Y;
                Length += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            return Length;
        }

        /// <summary>Turn at the middle point, in degrees. Zero is straight.</summary>
        /// <param name="Before">Previous point.</param>
        /// <param name="At">Corner point.</param>
        /// <param name="After">Next point.</param>
        /// <returns>Heading change in degrees, from 0 to 180.</returns>
        private static Double PointTurnDeg
            (
            (Double X, Double Y) Before,
            (Double X, Double Y) At,
            (Double X, Double Y) After
            )
        {
            Double Ux = At.X - Before.X;
            Double Uy = At.Y - Before.Y;
            Double Vx = After.X - At.X;
            Double Vy = After.Y - At.Y;
            Double ULen = Math.Sqrt((Ux * Ux) + (Uy * Uy));
            Double VLen = Math.Sqrt((Vx * Vx) + (Vy * Vy));
            if ((ULen <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                || (VLen <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE))
            {
                return 0.0;
            }
            Double Dot = ((Ux * Vx) + (Uy * Vy)) / (ULen * VLen);
            Dot = Math.Max(-1.0, Math.Min(1.0, Dot));
            return Math.Acos(Dot) * Constants.DEGREES_PER_HALF_TURN / Math.PI;
        }

        /// <summary>Turn at the middle station, in degrees. Zero is straight.</summary>
        /// <param name="Before">Previous footprint.</param>
        /// <param name="At">Corner footprint.</param>
        /// <param name="After">Next footprint.</param>
        /// <returns>Heading change in degrees, from 0 to 180.</returns>
        private static Double StationTurnDeg
            (
            FlowStation Before,
            FlowStation At,
            FlowStation After
            )
        {
            Double Ux = At.X - Before.X;
            Double Uy = At.Y - Before.Y;
            Double Vx = After.X - At.X;
            Double Vy = After.Y - At.Y;
            Double ULen = Math.Sqrt((Ux * Ux) + (Uy * Uy));
            Double VLen = Math.Sqrt((Vx * Vx) + (Vy * Vy));
            if ((ULen <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                || (VLen <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE))
            {
                return 0.0;
            }
            Double Dot = ((Ux * Vx) + (Uy * Vy)) / (ULen * VLen);
            Dot = Math.Max(-1.0, Math.Min(1.0, Dot));
            return Math.Acos(Dot) * Constants.DEGREES_PER_HALF_TURN / Math.PI;
        }

        /// <summary>
        /// Pulls each interior point toward the chord of its neighbors.
        /// Endpoints stay so the haul still leaves the end of the strip.
        /// Two passes take the point off a short kink. The cells still
        /// hold the line.
        /// </summary>
        /// <param name="Points">Spine in drive order.</param>
        /// <param name="Edge">Outer boundary. A point on it is left where it is.</param>
        /// <returns>A less kinked copy. Fewer than three points are unchanged.</returns>
        private static List<(Double X, Double Y)> SmoothFlow
            (
            List<(Double X, Double Y)> Points,
            FieldEdgePolicy? Edge = null
            )
        {
            if ((Points.Count < 3) || (FLOW_SMOOTH_PASSES <= 0))
            {
                return new List<(Double X, Double Y)>(Points);
            }
            List<(Double X, Double Y)> Current = Points;
            for (Int32 Pass = 0; Pass < FLOW_SMOOTH_PASSES; Pass += 1)
            {
                List<(Double X, Double Y)> Next =
                    new List<(Double X, Double Y)> { Current[0] };
                for (Int32 Index = 1; Index < Current.Count - 1; Index += 1)
                {
                    // The headland corner stays sharp. Smoothing it pulls the blade inside the field.
                    if ((Edge != null) && Edge.IsFieldPointNearOuterEdge(Current[Index].X, Current[Index].Y))
                    {
                        Next.Add(Current[Index]);
                        continue;
                    }
                    Next.Add((
                        (Current[Index - 1].X * 0.25)
                            + (Current[Index].X * 0.5)
                            + (Current[Index + 1].X * 0.25),
                        (Current[Index - 1].Y * 0.25)
                            + (Current[Index].Y * 0.5)
                            + (Current[Index + 1].Y * 0.25)));
                }
                Next.Add(Current[Current.Count - 1]);
                Current = Next;
            }
            return Current;
        }

        /// <summary>
        /// Opens a corner sharper than <see cref="FLOW_FILLET_DEG"/> into an arc.
        /// The field edge may use the turning circle. A bend inside a cut or a
        /// fill uses a wider radius, because a tight turn is slow. An arc that
        /// would have to be tighter than that is left on the cells.
        /// </summary>
        /// <param name="Points">Spine in drive order.</param>
        /// <param name="RadiusM">Turning-circle radius in metres. The edge may use this.</param>
        /// <param name="SpacingM">Distance between arc samples.</param>
        /// <param name="Edge">Outer boundary. A corner on it may use <paramref name="RadiusM"/>.</param>
        /// <returns>Spine whose bends are no tighter than the radius allowed there.</returns>
        private static List<(Double X, Double Y)> FilletFlow
            (
            List<(Double X, Double Y)> Points,
            Double RadiusM,
            Double SpacingM,
            FieldEdgePolicy? Edge = null
            )
        {
            if (Points.Count < 3) return new List<(Double X, Double Y)>(Points);
            // A short kink, including the jog where a cut starts, is opened.
            // Bends gentler than this stay on the cells.
            Double MinTurn = FLOW_FILLET_DEG * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            List<(Double X, Double Y)> Output =
                new List<(Double X, Double Y)> { Points[0] };
            for (Int32 Index = 1; Index < Points.Count - 1; Index += 1)
            {
                (Double X, Double Y) A = Points[Index - 1];
                (Double X, Double Y) B = Points[Index];
                (Double X, Double Y) C = Points[Index + 1];
                Double Ux = B.X - A.X;
                Double Uy = B.Y - A.Y;
                Double Vx = C.X - B.X;
                Double Vy = C.Y - B.Y;
                Double ULen = Math.Sqrt((Ux * Ux) + (Uy * Uy));
                Double VLen = Math.Sqrt((Vx * Vx) + (Vy * Vy));
                if ((ULen < (SpacingM * 0.25)) || (VLen < (SpacingM * 0.25)))
                {
                    Output.Add(B);
                    continue;
                }
                Ux /= ULen;
                Uy /= ULen;
                Vx /= VLen;
                Vy /= VLen;
                Double Dot = Math.Max(-1.0, Math.Min(1.0, (Ux * Vx) + (Uy * Vy)));
                Double Turn = Math.Acos(Dot);
                if (Turn < MinTurn)
                {
                    Output.Add(B);
                    continue;
                }
                Double TanHalf = Math.Tan(Turn * 0.5);
                if (TanHalf <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                {
                    Output.Add(B);
                    continue;
                }
                // The edge may use the turning circle. Inside the work the
                // bend is wider, because a tight turn is slow.
                Boolean OnEdge = (Edge != null) && Edge.IsFieldPointNearOuterEdge(B.X, B.Y);
                Double Allowed = OnEdge
                    ? RadiusM
                    : RadiusM * Constants.INTERIOR_BLADE_TURN_SCALE;
                Double Tangent = Allowed * TanHalf;
                Double MaxTangent = 0.45 * Math.Min(ULen, VLen);
                if (Tangent > MaxTangent)
                {
                    // The allowed radius does not fit on these legs. A shorter
                    // arc would be tighter than the pan may turn here.
                    Output.Add(B);
                    continue;
                }
                Double UsedRadius = Allowed;
                // A corner with no leg left is already a point. Skip it.
                if (Tangent < 0.3)
                {
                    Output.Add(B);
                    continue;
                }
                (Double X, Double Y) T1 = (B.X - (Ux * Tangent), B.Y - (Uy * Tangent));
                (Double X, Double Y) T2 = (B.X + (Vx * Tangent), B.Y + (Vy * Tangent));
                // Left of the incoming step is the center of a left turn.
                Double Cross = (Ux * Vy) - (Uy * Vx);
                Double Side = Cross >= 0.0 ? 1.0 : -1.0;
                Double Nx = -Uy * Side;
                Double Ny = Ux * Side;
                (Double X, Double Y) Center = (T1.X + (Nx * UsedRadius), T1.Y + (Ny * UsedRadius));
                Double StartAng = Math.Atan2(T1.Y - Center.Y, T1.X - Center.X);
                // The arc sweep is the heading change. Unwrapping the end
                // angle can take the long way around the circle.
                Double Sweep = Side * Turn;
                // Each sample turns by less than the cap. A 60 degree sample
                // is the sharpest bend the radius check still has to reject.
                Double MaxStep = (FLOW_MAX_TURN_DEG - 0.5) * Math.PI / Constants.DEGREES_PER_HALF_TURN;
                Int32 Steps = Math.Max(
                    2,
                    (Int32)Math.Ceiling(Math.Abs(Sweep) / MaxStep));
                Output.Add(T1);
                for (Int32 Step = 1; Step < Steps; Step += 1)
                {
                    Double Ang = StartAng + (Sweep * Step / Steps);
                    Output.Add((
                        Center.X + (Math.Cos(Ang) * UsedRadius),
                        Center.Y + (Math.Sin(Ang) * UsedRadius)));
                }
                Output.Add(T2);
            }
            Output.Add(Points[Points.Count - 1]);
            return Output;
        }

        /// <summary>Spaces a path so a curve is drawn as small steps, not one chord.</summary>
        /// <param name="Points">Path in drive order.</param>
        /// <param name="SpacingM">Distance between kept points.</param>
        /// <returns>Resampled path. The ends are kept.</returns>
        private static List<(Double X, Double Y)> ResampleFlow
            (
            List<(Double X, Double Y)> Points,
            Double SpacingM
            )
        {
            if (Points.Count < 2) return new List<(Double X, Double Y)>(Points);
            List<(Double X, Double Y)> Output =
                new List<(Double X, Double Y)> { Points[0] };
            Double Acc = 0.0;
            (Double X, Double Y) Previous = Points[0];
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                (Double X, Double Y) Point = Points[Index];
                Double Dx = Point.X - Previous.X;
                Double Dy = Point.Y - Previous.Y;
                Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Length <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
                {
                    continue;
                }
                Acc += Length;
                // Keep a point once the step is long enough to see the bend.
                // Also keep it when skipping it would leave a corner sharper
                // than 60 degrees. The fillet samples are often shorter than
                // the spacing, and dropping them restores the kink.
                Boolean KeepsTheCorner = (Index == (Points.Count - 1));
                if (!KeepsTheCorner && (Index + 1 < Points.Count))
                {
                    (Double X, Double Y) Held = Output[Output.Count - 1];
                    (Double X, Double Y) Next = Points[Index + 1];
                    // This sample is itself a corner the cap does not allow.
                    Double AtPoint = PointTurnDeg(Held, Point, Next);
                    Double Skip = 0.0;
                    if (Output.Count >= 2)
                    {
                        // Jumping past this sample would sharpen the corner already kept.
                        (Double X, Double Y) Incoming = Output[Output.Count - 2];
                        Skip = PointTurnDeg(Incoming, Held, Next);
                    }
                    if ((Skip > (FLOW_MAX_TURN_DEG - 0.5))
                        || (AtPoint > (FLOW_MAX_TURN_DEG - 0.5)))
                    {
                        KeepsTheCorner = true;
                    }
                }
                if ((Acc >= SpacingM) || KeepsTheCorner)
                {
                    Output.Add(Point);
                    Acc = 0.0;
                }
                Previous = Point;
            }
            (Double X, Double Y) Last = Points[Points.Count - 1];
            (Double X, Double Y) Kept = Output[Output.Count - 1];
            Double EndX = Last.X - Kept.X;
            Double EndY = Last.Y - Kept.Y;
            if (((EndX * EndX) + (EndY * EndY))
                > Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE)
            {
                Output.Add(Last);
            }
            return Output;
        }

        /// <summary>
        /// Builds every deterministic five-degree coverage candidate in score order.
        /// </summary>
        /// <param name="Cells">Allocated cells and depths.</param>
        /// <param name="Grid">Grid geometry.</param>
        /// <param name="BladeWidthM">Effective blade coverage width in metres.</param>
        /// <param name="OrientStartXy">Optional approach anchor.</param>
        /// <param name="OrientEndXy">Optional exit anchor.</param>
        /// <param name="MinimumWorkLengthM">
        /// Minimum kept blade-down run length in metres; null uses the ambient
        /// engine scope, and zero disables the gate.
        /// </param>
        /// <returns>Candidates ordered by ascending score.</returns>
        static public List<PlanResult> CandidatePlans
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double BladeWidthM,
            (Double X, Double Y)? OrientStartXy = null,
            (Double X, Double Y)? OrientEndXy = null,
            Double? MinimumWorkLengthM = null
            )
        {
            if (Cells == null) throw new ArgumentNullException(nameof(Cells));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (BladeWidthM <= 0.0) throw new ArgumentOutOfRangeException(nameof(BladeWidthM));
            Double ResolvedMinimumWorkLengthM =
                ResolveMinimumWorkLengthM(MinimumWorkLengthM);
            if (ResolvedMinimumWorkLengthM < 0.0)
            {
                throw new ArgumentOutOfRangeException(nameof(MinimumWorkLengthM));
            }
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

            FieldEdgePolicy EdgePolicy =
                new FieldEdgePolicy(Grid, Grid.GridSizeM);
            List<PlanResult> Candidates = new List<PlanResult>();
            for (Double HeadingDeg = 0.0; HeadingDeg < Constants.DEGREES_PER_HALF_TURN; HeadingDeg += HEADING_STEP_DEG)
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
                        OmittedCells,
                        MAX_ALONG_GAP_GRID_MULTIPLIER,
                        ResolvedMinimumWorkLengthM);
                    if (!ConservesAllocation(Cells, Candidate.AccountedCells))
                        throw new InvalidOperationException(
                            " coverage planning failed to account for the cell/depth multiset.");
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
        /// Re-packs one fill allocation at an established heading while allowing
        /// nearby along-lane gaps to remain inside a continuous work envelope.
        /// The returned runs retain the exact input cell/depth multiset; the extra
        /// centerline is temporary placement redistributed by the grade return.
        /// </summary>
        static internal PlanResult BuildFillEnvelopeCandidate
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double BladeWidthM,
            Double HeadingDeg,
            Double MaximumAlongGapGridMultiplier
            )
        {
            if (Cells == null) throw new ArgumentNullException(nameof(Cells));
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            if (BladeWidthM <= 0.0)
            {
                throw new ArgumentOutOfRangeException(nameof(BladeWidthM));
            }
            if (MaximumAlongGapGridMultiplier
                < MAX_ALONG_GAP_GRID_MULTIPLIER)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(MaximumAlongGapGridMultiplier));
            }
            Double NormalizedHeadingDeg = ((HeadingDeg % Constants.DEGREES_PER_HALF_TURN) + Constants.DEGREES_PER_HALF_TURN)
                % Constants.DEGREES_PER_HALF_TURN;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Omitted =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            // Fill envelopes must keep every allocated cell on a driven run; length
            // omission is intentionally disabled for this temporary grade geometry.
            PlanResult Selected = new Boolean[] { false, true }
                .Select(DescendingCross => BuildCandidate(
                    Cells,
                    Grid,
                    BladeWidthM,
                    NormalizedHeadingDeg,
                    DescendingCross,
                    null,
                    null,
                    Omitted,
                    MaximumAlongGapGridMultiplier,
                    MinimumWorkLengthM: 0.0))
                .OrderBy(Candidate => Candidate.Score)
                .ThenBy(Candidate => Candidate.HeadingDeg)
                .First();
            if (!ConservesAllocation(Cells, Selected.OrderedCells))
            {
                throw new InvalidOperationException(
                    "Fill-envelope planning changed the cell/depth multiset.");
            }
            return Selected;
        }

        /// <summary>
        /// Builds and scores one heading and cross-axis packing direction.
        /// </summary>
        /// <param name="Cells">Work cells for this candidate.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="BladeWidthM">Blade width in metres.</param>
        /// <param name="HeadingDeg">Candidate heading.</param>
        /// <param name="DescendingCross">Cross-axis packing direction.</param>
        /// <param name="OrientStartXy">Optional approach anchor.</param>
        /// <param name="OrientEndXy">Optional exit anchor.</param>
        /// <param name="OmittedCells">Already omitted edge/corner cells.</param>
        /// <param name="MaximumAlongGapGridMultiplier">Along-lane gap split threshold.</param>
        /// <param name="MinimumWorkLengthM">Minimum kept blade-down length; zero disables.</param>
        /// <returns>One scored coverage candidate.</returns>
        private static PlanResult BuildCandidate
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Grid Grid,
            Double BladeWidthM,
            Double HeadingDeg,
            Boolean DescendingCross,
            (Double X, Double Y)? OrientStartXy,
            (Double X, Double Y)? OrientEndXy,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> OmittedCells,
            Double MaximumAlongGapGridMultiplier =
                MAX_ALONG_GAP_GRID_MULTIPLIER,
            Double MinimumWorkLengthM = 0.0
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
            Double Theta = HeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            (Double X, Double Y) AlongUnit = (Math.Sin(Theta), Math.Cos(Theta));
            (Double X, Double Y) CrossUnit = (Math.Cos(Theta), -Math.Sin(Theta));
            (Double X, Double Y) Origin = CellCentroid(Cells, Grid);
            List<ProjectedCell> Projected = Project(Cells, Grid, Origin, AlongUnit, CrossUnit);
            List<List<ProjectedCell>> Groups = PackCrossAxis(Projected, BladeWidthM, DescendingCross);
            List<Run> Runs = new List<Run>();
            Double MaximumGapM = MaximumAlongGapGridMultiplier
                * Grid.GridSizeM;
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
            (List<Run> KeptRuns,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> ShortOmitted) =
                OmitShortRuns(Runs, MinimumWorkLengthM);
            Runs = OrderRuns(KeptRuns, OrientStartXy, OrientEndXy);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CombinedOmitted =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(OmittedCells);
            CombinedOmitted.AddRange(ShortOmitted);

            Double ConnectorDistanceM = ConnectorDistance(Runs);
            Double AnchorDistanceM = AnchorDistance(Runs, OrientStartXy, OrientEndXy);
            Double WorkDistanceM = Runs.Sum(Run => Run.LengthM);
            Double Score = (Runs.Count * RUN_COUNT_WEIGHT)
                + (ConnectorDistanceM * CONNECTOR_WEIGHT)
                + AnchorDistanceM
                + (WorkDistanceM * WORK_DISTANCE_WEIGHT)
                + (CombinedOmitted.Count * OMITTED_CELL_WEIGHT);
            return new PlanResult(
                HeadingDeg,
                Runs,
                ConnectorDistanceM,
                AnchorDistanceM,
                Score,
                CombinedOmitted,
                ShortOmitted);
        }

        /// <summary>
        /// Estimates the blade-up metres one extra coverage run costs: lineup,
        /// exit, and a half-turn when the next run reverses.
        /// </summary>
        /// <param name="Runs">Ordered coverage runs.</param>
        /// <returns>Estimated driven overhead in metres.</returns>
        static internal Double EstimateRunOverheadM
            (
            IReadOnlyList<Run> Runs
            )
        {
            Double TotalM = 0.0;
            for (Int32 Index = 0; Index < Runs.Count; Index += 1)
            {
                TotalM += COVERAGE_LINEUP_M + COVERAGE_EXIT_M;
                if (((Index + 1) < Runs.Count)
                    && NextRunReverses(Runs[Index].HeadingDeg, Runs[Index + 1].HeadingDeg))
                {
                    // A reverse between runs is a half-turn at the minimum radius.
                    TotalM += COVERAGE_MINIMUM_RADIUS_M * Math.PI;
                }
            }
            return TotalM;
        }

        /// <summary>
        /// Reports whether the next run is driven back toward the previous one.
        /// </summary>
        /// <param name="HeadingDeg">Heading of the current run, in degrees.</param>
        /// <param name="NextHeadingDeg">Heading of the following run, in degrees.</param>
        /// <returns>True when the smaller heading gap exceeds a right angle.</returns>
        static private Boolean NextRunReverses
            (
            Double HeadingDeg,
            Double NextHeadingDeg
            )
        {
            Double Delta = Math.Abs(HeadingDeg - NextHeadingDeg) % FULL_CIRCLE_DEG;
            if (Delta > Constants.DEGREES_PER_HALF_TURN)
            {
                Delta = FULL_CIRCLE_DEG - Delta;
            }
            return Delta > REVERSE_HEADING_SPLIT_DEG;
        }

        /// <summary>
        /// Drops blade-down runs shorter than the configured minimum and returns
        /// their cells as short-run omissions.
        /// </summary>
        /// <param name="Runs">Candidate coverage runs.</param>
        /// <param name="MinimumWorkLengthM">Minimum kept length; zero keeps all runs.</param>
        /// <returns>Kept runs and cells from omitted short runs.</returns>
        private static (
            List<Run> KeptRuns,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> ShortOmitted)
            OmitShortRuns
            (
            List<Run> Runs,
            Double MinimumWorkLengthM
            )
        {
            if ((MinimumWorkLengthM <= 0.0) || (Runs.Count == 0))
            {
                return (
                    Runs,
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>());
            }

            List<Run> Kept = new List<Run>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> ShortOmitted =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            foreach (Run Run in Runs)
            {
                // Intent: scrap runs are never worth a full equipment lineup/turn.
                if (Run.LengthM + Constants.NUMERIC_COMPARISON_TOLERANCE
                    < MinimumWorkLengthM)
                {
                    ShortOmitted.AddRange(Run.Cells);
                }
                else
                {
                    Kept.Add(Run);
                }
            }
            return (Kept, ShortOmitted);
        }

        /// <summary>
        /// After independent short-run gates on cut and fill, omits matching bank
        /// volume from the surplus side so the remaining haul stays balanced.
        /// </summary>
        /// <param name="Cut">Cut coverage after the minimum-length gate.</param>
        /// <param name="Fill">Fill coverage after the minimum-length gate.</param>
        /// <param name="Grid">Field grid (area and compaction).</param>
        /// <param name="MinimumWorkLengthM">
        /// Explicit gate for cascaded short-run drops, or null for ambient/default.
        /// </param>
        /// <returns>Cut and fill plans with equalized short-run deferred volume.</returns>
        static public (PlanResult Cut, PlanResult Fill) BalanceShortRunVolumes
            (
            PlanResult Cut,
            PlanResult Fill,
            Grid Grid,
            Double? MinimumWorkLengthM = null
            )
        {
            if (Cut == null)
            {
                throw new ArgumentNullException(nameof(Cut));
            }
            if (Fill == null)
            {
                throw new ArgumentNullException(nameof(Fill));
            }
            if (Grid == null)
            {
                throw new ArgumentNullException(nameof(Grid));
            }

            Double ResolvedMinimumWorkLengthM =
                ResolveMinimumWorkLengthM(MinimumWorkLengthM);
            Double CutBcyPerDepthM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillBcyPerDepthM = CutBcyPerDepthM * Grid.Compaction;
            PlanResult BalancedCut = Cut;
            PlanResult BalancedFill = Fill;

            // Intent: peel the surplus side until short-run deferred cut and fill
            // bank volumes match; cascading min-length drops may need a few passes.
            for (Int32 Guard = 0; Guard < 32; Guard += 1)
            {
                Double CutShortBcy = ShortRunVolumeBcy(
                    BalancedCut.ShortRunOmittedCells, CutBcyPerDepthM);
                Double FillShortBcy = ShortRunVolumeBcy(
                    BalancedFill.ShortRunOmittedCells, FillBcyPerDepthM);
                Double DeltaBcy = FillShortBcy - CutShortBcy;
                if (Math.Abs(DeltaBcy) <= NUMERIC_TOLERANCE)
                {
                    break;
                }

                if (DeltaBcy > 0.0)
                {
                    // Fill scraps exceed cut scraps: trim matching cut volume.
                    (PlanResult NextCut, Double RemovedBcy) = OmitVolumeFromPlan(
                        BalancedCut,
                        Grid,
                        DeltaBcy,
                        CutBcyPerDepthM,
                        ResolvedMinimumWorkLengthM);
                    if (RemovedBcy <= NUMERIC_TOLERANCE)
                    {
                        break;
                    }
                    BalancedCut = NextCut;
                }
                else
                {
                    // Cut scraps exceed fill scraps: trim matching fill volume.
                    (PlanResult NextFill, Double RemovedBcy) = OmitVolumeFromPlan(
                        BalancedFill,
                        Grid,
                        -DeltaBcy,
                        FillBcyPerDepthM,
                        ResolvedMinimumWorkLengthM);
                    if (RemovedBcy <= NUMERIC_TOLERANCE)
                    {
                        break;
                    }
                    BalancedFill = NextFill;
                }
            }

            return (BalancedCut, BalancedFill);
        }

        /// <summary>
        /// Sums short-run omitted bank volume using the cut or fill depth factor.
        /// </summary>
        /// <param name="Cells">Short-run omitted cells.</param>
        /// <param name="BcyPerDepthM">Bank-volume conversion for one metre of depth.</param>
        /// <returns>Total deferred bank volume in BCY.</returns>
        private static Double ShortRunVolumeBcy
            (
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double BcyPerDepthM
            )
        {
            Double Total = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                if (Entry.DepthM > 0.0)
                {
                    Total += Entry.DepthM * BcyPerDepthM;
                }
            }
            return Total;
        }

        /// <summary>
        /// Peels tip cells from the shortest kept runs until TargetBcy is reached,
        /// splitting the last tip by depth when it holds more than the remainder,
        /// and cascading any remainder that falls below the minimum work length.
        /// </summary>
        /// <param name="Plan">Coverage plan to trim.</param>
        /// <param name="Grid">Field grid for run rebuilds.</param>
        /// <param name="TargetBcy">
        /// Bank volume to move into short-run omission. The last tip is split by
        /// depth so the omitted volume can match this target exactly.
        /// </param>
        /// <param name="BcyPerDepthM">Bank-volume conversion for one metre of depth.</param>
        /// <param name="MinimumWorkLengthM">Minimum kept blade-down length.</param>
        /// <returns>Updated plan and bank volume newly omitted.</returns>
        private static (PlanResult Plan, Double RemovedBcy) OmitVolumeFromPlan
            (
            PlanResult Plan,
            Grid Grid,
            Double TargetBcy,
            Double BcyPerDepthM,
            Double MinimumWorkLengthM
            )
        {
            if ((TargetBcy <= NUMERIC_TOLERANCE) || (Plan.Runs.Count == 0))
            {
                return (Plan, 0.0);
            }

            List<Run> Runs = new List<Run>(Plan.Runs);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> ExtraShort =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double RemovedBcy = 0.0;

            while ((RemovedBcy + NUMERIC_TOLERANCE < TargetBcy) && (Runs.Count > 0))
            {
                // Prefer the shortest kept run so balance trims scrap tips first.
                // Taking the longest run instead was 950 m longer on TheShop2.
                Int32 RunIndex = 0;
                for (Int32 Index = 1; Index < Runs.Count; Index += 1)
                {
                    if (Runs[Index].LengthM < Runs[RunIndex].LengthM
                        - NUMERIC_TOLERANCE)
                    {
                        RunIndex = Index;
                    }
                    else if (Math.Abs(Runs[Index].LengthM - Runs[RunIndex].LengthM)
                        <= NUMERIC_TOLERANCE
                        && (Runs[Index].Cells.Count < Runs[RunIndex].Cells.Count))
                    {
                        RunIndex = Index;
                    }
                }

                Run Selected = Runs[RunIndex];
                if (Selected.Cells.Count == 0)
                {
                    Runs.RemoveAt(RunIndex);
                    continue;
                }

                // Peel the drive-end tip; that scrap is least useful for lineup.
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Tip =
                    Selected.Cells[Selected.Cells.Count - 1];
                Double NeededBcy = TargetBcy - RemovedBcy;
                Double TipBcy = Tip.DepthM * BcyPerDepthM;
                // Intent: a whole cell overshoots the balance target, and the caller
                // then trims the other side again. Defer only the needed depth so
                // both sides match after one round.
                if ((Tip.DepthM > 0.0) && (TipBcy > NeededBcy + NUMERIC_TOLERANCE))
                {
                    Double DeferredDepthM = NeededBcy / BcyPerDepthM;
                    // Intent: the run object is shared with the input plan, so build
                    // a new cell list instead of editing Selected.Cells in place.
                    List<((Int32 Bx, Int32 By) Cell, Double DepthM)> SplitCells =
                        Selected.Cells.GetRange(0, Selected.Cells.Count - 1);
                    SplitCells.Add((Tip.Cell, Tip.DepthM - DeferredDepthM));
                    // The run keeps every cell, so its centerline does not change.
                    Runs[RunIndex] = new Run(
                        SplitCells,
                        Selected.StartXy,
                        Selected.EndXy,
                        Selected.HeadingDeg,
                        Selected.MaximumLateralErrorM);
                    ExtraShort.Add((Tip.Cell, DeferredDepthM));
                    RemovedBcy += NeededBcy;
                    break;
                }
                ExtraShort.Add(Tip);
                if (Tip.DepthM > 0.0)
                {
                    RemovedBcy += Tip.DepthM * BcyPerDepthM;
                }

                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Remaining =
                    Selected.Cells.GetRange(0, Selected.Cells.Count - 1);
                if (Remaining.Count == 0)
                {
                    Runs.RemoveAt(RunIndex);
                    continue;
                }

                Run Rebuilt = BuildSubRun(Remaining, Selected.HeadingDeg, Grid);
                // Intent: do not leave a new sub-minimum stub after tip peeling.
                if ((MinimumWorkLengthM > 0.0)
                    && (Rebuilt.LengthM + NUMERIC_TOLERANCE < MinimumWorkLengthM))
                {
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                        in Remaining)
                    {
                        ExtraShort.Add(Entry);
                        if (Entry.DepthM > 0.0)
                        {
                            RemovedBcy += Entry.DepthM * BcyPerDepthM;
                        }
                    }
                    Runs.RemoveAt(RunIndex);
                }
                else
                {
                    Runs[RunIndex] = Rebuilt;
                }
            }

            if (ExtraShort.Count == 0)
            {
                return (Plan, 0.0);
            }

            (List<Run> KeptRuns,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CascadedShort) =
                OmitShortRuns(Runs, MinimumWorkLengthM);
            ExtraShort.AddRange(CascadedShort);
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in CascadedShort)
            {
                if (Entry.DepthM > 0.0)
                {
                    RemovedBcy += Entry.DepthM * BcyPerDepthM;
                }
            }

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> ShortOmitted =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(
                    Plan.ShortRunOmittedCells);
            ShortOmitted.AddRange(ExtraShort);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CombinedOmitted =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Plan.OmittedCells);
            CombinedOmitted.AddRange(ExtraShort);
            Double ConnectorDistanceM = ConnectorDistance(KeptRuns);
            Double WorkDistanceM = KeptRuns.Sum(Run => Run.LengthM);
            Double Score = (KeptRuns.Count * RUN_COUNT_WEIGHT)
                + (ConnectorDistanceM * CONNECTOR_WEIGHT)
                + Plan.AnchorDistanceM
                + (WorkDistanceM * WORK_DISTANCE_WEIGHT)
                + (CombinedOmitted.Count * OMITTED_CELL_WEIGHT);
            PlanResult Updated = new PlanResult(
                Plan.HeadingDeg,
                KeptRuns,
                ConnectorDistanceM,
                Plan.AnchorDistanceM,
                Score,
                CombinedOmitted,
                ShortOmitted);
            return (Updated, RemovedBcy);
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
            Double HeadingRad = HeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
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
            Double CenterCross = Constants.HALF_SCALE * (
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
                (Int32)Math.Ceiling(Run.LengthM / Math.Min(Constants.HALF_SCALE, Grid.GridSizeM * Constants.HALF_SCALE)));
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
                Double CenterCross = Constants.HALF_SCALE * (MinimumCross + MaximumCross);
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
            Double HeadingDeg = (Input.HeadingDeg + Constants.DEGREES_PER_HALF_TURN) % Constants.DEGREES_PER_TURN;
            return new Run(
                Cells,
                Input.EndXy,
                Input.StartXy,
                HeadingDeg,
                Input.MaximumLateralErrorM,
                Input.ReversedDrivePoints());
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
