using System.Globalization;

namespace HaulBuilder
{
    /// <summary>
    /// Builds access roads and well pads to a fixed height above the fitted field
    /// design. Surplus on the pan is ordinary cut for the main stages. The deficit
    /// is reserved as blade-width lanes on the nearest field cut and placed by the
    /// road stage, after flow and before correcting.
    /// </summary>
    static public partial class RoadPadBuild
    {
        /// <summary>Sub-points along one cell edge. The cell is a 4 by 4 sample.</summary>
        public const Int32 SUBPOINT_COUNT = 4;

        /// <summary>Duplicate centreline vertices closer than this are the same road.</summary>
        private const Double DUPLICATE_VERTEX_FT = 0.1;

        /// <summary>Overlapping roads whose heights differ by more than this are warned.</summary>
        private const Double HEIGHT_CONFLICT_FT = 0.1;

        /// <summary>Road-top samples must match the crown formula within one millimetre.</summary>
        private const Double ROAD_TOP_TOLERANCE_M = 0.001;

        /// <summary>A bend smaller than this is treated as straight.</summary>
        private const Double BEND_STRAIGHT_DEG = 1.0;

        /// <summary>A segment this close to a grid axis is snapped onto whole cells.</summary>
        private const Double PAN_AXIS_DEGREES = 1.0;

        /// <summary>Grade samples and shared-length steps, in feet.</summary>
        private const Double SAMPLE_STEP_FT = 1.0;

        /// <summary>Depths below this are not a pan lift.</summary>
        private const Double LIFT_EPSILON_M = 1e-5;

        /// <summary>
        /// Cut left above a reserved layer so the cell still has a flow arrow.
        /// The flow field draws an arrow only when existing and target differ by more than this.
        /// </summary>
        private const Double FLOW_SOURCE_KEEP_M = 0.001;

        /// <summary>Bank cubic yards the road-stage balance may miss and still pass.</summary>
        private const Double BALANCE_CUSHION_BCY = 0.05;

        /// <summary>A reserve lane is this many turning radii long. A shorter run is not reserved.</summary>
        private const Double RESERVE_LANE_RADII = 4.0;

        /// <summary>A slope flatter than this, in metres per metre, has no downhill cut direction.</summary>
        private const Double FLAT_SLOPE = 0.001;

        /// <summary>
        /// Extra metres allowed past the finish tolerance so a cell that landed on the tolerance is not a miss.
        /// </summary>
        private const Double FINISH_COMPARE_SLACK_M = 1e-9;

        /// <summary>Off-grid embankment below this is not reported as an open end.</summary>
        private const Double OPEN_END_BCY = 0.05;

        /// <summary>
        /// Rejects road and pad options that cannot build a surface.
        /// </summary>
        /// <param name="Options">Options to check.</param>
        public static void Validate
            (
            HaulBuilderOptions Options
            )
        {
            if (Options == null)
            {
                throw new ArgumentNullException(nameof(Options));
            }

            Positive(Options.RoadNrcsMinRoadbedFt, nameof(Options.RoadNrcsMinRoadbedFt));
            if (Options.RoadMinLanes < 1)
            {
                throw new ArgumentOutOfRangeException(nameof(Options.RoadMinLanes));
            }

            Positive(Options.RoadHeightFt, nameof(Options.RoadHeightFt));
            Positive(Options.RoadSideSlope, nameof(Options.RoadSideSlope));
            NonNegative(Options.RoadCrossSlopePct, nameof(Options.RoadCrossSlopePct));
            Positive(Options.RoadMaxGradePct, nameof(Options.RoadMaxGradePct));
            Positive(Options.PadDesignTurnRadiusFt, nameof(Options.PadDesignTurnRadiusFt));
            Positive(Options.PadClearanceFt, nameof(Options.PadClearanceFt));
            NonNegative(Options.PadExclusionRadiusFt, nameof(Options.PadExclusionRadiusFt));
            NonNegative(Options.PadSlopeAwayPct, nameof(Options.PadSlopeAwayPct));
            Positive(Options.PadSideSlope, nameof(Options.PadSideSlope));
            NonNegative(Options.BladeShapingHoursPerFt, nameof(Options.BladeShapingHoursPerFt));
            NonNegative(Options.BladeShapingHoursPerPad, nameof(Options.BladeShapingHoursPerPad));
            if ((Options.RoadMinTripLoadPct <= 0.0) || (Options.RoadMinTripLoadPct > 1.0))
            {
                throw new ArgumentOutOfRangeException(nameof(Options.RoadMinTripLoadPct));
            }

            NonNegative(Options.RoadBladeMoveMaxFt, nameof(Options.RoadBladeMoveMaxFt));
            NonNegative(Options.ReserveMaxDepthFt, nameof(Options.ReserveMaxDepthFt));
            if (Options.Roads == null)
            {
                throw new ArgumentNullException(nameof(Options.Roads));
            }

            if (Options.WellPads == null)
            {
                throw new ArgumentNullException(nameof(Options.WellPads));
            }

            for (Int32 Index = 0; Index < Options.Roads.Count; Index += 1)
            {
                RoadCentreline Road = Options.Roads[Index];
                if (Road == null)
                {
                    throw new ArgumentException("A road centreline is null.", nameof(Options));
                }

                if (Road.Vertices.Count < 2)
                {
                    throw new ArgumentException(
                        "A road centreline needs at least two vertices.",
                        nameof(Options));
                }

                foreach ((Double X, Double Y) Vertex in Road.Vertices)
                {
                    if (!Double.IsFinite(Vertex.X) || !Double.IsFinite(Vertex.Y))
                    {
                        throw new ArgumentException("A road vertex is not finite.", nameof(Options));
                    }
                }

                if (Road.HeightFt.HasValue)
                {
                    Positive(Road.HeightFt.Value, nameof(Road.HeightFt));
                }

                if (Road.SideSlope.HasValue)
                {
                    Positive(Road.SideSlope.Value, nameof(Road.SideSlope));
                }

                if (Road.CrossSlopePct.HasValue)
                {
                    NonNegative(Road.CrossSlopePct.Value, nameof(Road.CrossSlopePct));
                }

                if (Road.MinRoadbedFt.HasValue)
                {
                    Positive(Road.MinRoadbedFt.Value, nameof(Road.MinRoadbedFt));
                }

                if (Road.MinLanes.HasValue && (Road.MinLanes.Value < 1))
                {
                    throw new ArgumentOutOfRangeException(nameof(Road.MinLanes));
                }

                if (Road.MaxGradePct.HasValue)
                {
                    Positive(Road.MaxGradePct.Value, nameof(Road.MaxGradePct));
                }
            }

            for (Int32 Index = 0; Index < Options.WellPads.Count; Index += 1)
            {
                WellPad Pad = Options.WellPads[Index];
                if (Pad == null)
                {
                    throw new ArgumentException("A well pad is null.", nameof(Options));
                }

                if (!Double.IsFinite(Pad.X) || !Double.IsFinite(Pad.Y))
                {
                    throw new ArgumentException("A well pad centre is not finite.", nameof(Options));
                }

                if (Pad.HeightFt.HasValue)
                {
                    Positive(Pad.HeightFt.Value, nameof(Pad.HeightFt));
                }

                if (Pad.SlopeAwayPct.HasValue)
                {
                    NonNegative(Pad.SlopeAwayPct.Value, nameof(Pad.SlopeAwayPct));
                }

                if (Pad.SideSlope.HasValue)
                {
                    Positive(Pad.SideSlope.Value, nameof(Pad.SideSlope));
                }

                if (Pad.DesignTurnRadiusFt.HasValue)
                {
                    Positive(Pad.DesignTurnRadiusFt.Value, nameof(Pad.DesignTurnRadiusFt));
                }

                if (Pad.ClearanceFt.HasValue)
                {
                    Positive(Pad.ClearanceFt.Value, nameof(Pad.ClearanceFt));
                }

                if (Pad.ExclusionRadiusFt.HasValue)
                {
                    NonNegative(Pad.ExclusionRadiusFt.Value, nameof(Pad.ExclusionRadiusFt));
                }
            }
        }

        /// <summary>
        /// Blade width in feet. One lane is one blade width.
        /// </summary>
        /// <param name="Options">Options carrying the blade width in millimetres.</param>
        /// <returns>Lane width in feet.</returns>
        static public Double LaneWidthFt
            (
            HaulBuilderOptions Options
            )
        {
            return (Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER) / Constants.FEET_TO_METERS;
        }

        /// <summary>
        /// Smallest whole lane count whose finished top meets the minimum roadbed.
        /// The finished top is the pan width minus one side-slope run.
        /// </summary>
        /// <param name="LaneWidthFt">One lane, in feet.</param>
        /// <param name="MinRoadbedFt">Minimum finished roadbed, in feet.</param>
        /// <param name="MinLanes">Minimum lane count. The formula may use more.</param>
        /// <param name="SideSlope">Horizontal run per unit of shoulder height.</param>
        /// <param name="HeightFt">Shoulder height in feet.</param>
        /// <returns>The lane count.</returns>
        static public Int32 PanLaneCount
            (
            Double LaneWidthFt,
            Double MinRoadbedFt,
            Int32 MinLanes,
            Double SideSlope,
            Double HeightFt
            )
        {
            Double Lane = Math.Max(LaneWidthFt, 1e-6);
            // n × lane − side × height ≥ minimum roadbed.
            Double Need = MinRoadbedFt + (SideSlope * HeightFt);
            Int32 Count = (Int32)Math.Ceiling((Need / Lane) - 1e-9);
            if (Count < 1)
            {
                Count = 1;
            }

            if (MinLanes > Count)
            {
                Count = MinLanes;
            }

            return Count;
        }

        /// <summary>
        /// Finished top width. Defaults are two lanes and a 14 ft top.
        /// </summary>
        /// <param name="LaneWidthFt">One lane, in feet.</param>
        /// <param name="MinRoadbedFt">Minimum finished roadbed, in feet.</param>
        /// <param name="MinLanes">Minimum lane count.</param>
        /// <param name="SideSlope">Horizontal run per unit of shoulder height.</param>
        /// <param name="HeightFt">Shoulder height in feet.</param>
        /// <returns>Finished top width in feet.</returns>
        static public Double TopWidthFt
            (
            Double LaneWidthFt,
            Double MinRoadbedFt,
            Int32 MinLanes,
            Double SideSlope,
            Double HeightFt
            )
        {
            Int32 Lanes = PanLaneCount(LaneWidthFt, MinRoadbedFt, MinLanes, SideSlope, HeightFt);
            Double Top = (Lanes * LaneWidthFt) - (SideSlope * HeightFt);
            return Math.Max(0.0, Top);
        }

        /// <summary>
        /// Finished cross-section area, crown and both side slopes included.
        /// </summary>
        /// <param name="LaneWidthFt">One lane, in feet.</param>
        /// <param name="MinRoadbedFt">Minimum finished roadbed, in feet.</param>
        /// <param name="MinLanes">Minimum lane count.</param>
        /// <param name="SideSlope">Horizontal run per unit of shoulder height.</param>
        /// <param name="HeightFt">Shoulder height in feet.</param>
        /// <param name="CrossSlopePct">Crown slope in percent.</param>
        /// <returns>Area in square feet.</returns>
        static public Double FinishedSectionSqFt
            (
            Double LaneWidthFt,
            Double MinRoadbedFt,
            Int32 MinLanes,
            Double SideSlope,
            Double HeightFt,
            Double CrossSlopePct
            )
        {
            Double Top = TopWidthFt(LaneWidthFt, MinRoadbedFt, MinLanes, SideSlope, HeightFt);
            Double Half = Top * Constants.HALF_SCALE;
            Double Cross = CrossSlopePct / 100.0;
            Double Centre = HeightFt + (Cross * Half);
            // Trapezoid top, plus one triangle on each side at the shoulder height.
            Double TopArea = (Centre + HeightFt) * Half;
            Double SideArea = SideSlope * HeightFt * HeightFt;
            return TopArea + SideArea;
        }

        /// <summary>
        /// Flat height of a pad pan, equal in volume to the finished pad.
        /// The finished pad is the shoulder height at the top radius, a slope up to the well, and a 3:1 edge.
        /// </summary>
        /// <param name="ShoulderM">Shoulder height at the top radius, in metres.</param>
        /// <param name="Cross">Slope away, rise over run.</param>
        /// <param name="TopM">Finished top radius in metres.</param>
        /// <param name="ReachM">Side-slope run in metres.</param>
        /// <param name="PanRadiusM">Pan radius in metres.</param>
        /// <returns>Height in metres.</returns>
        private static Double PadBuildHeightM
            (
            Double ShoulderM,
            Double Cross,
            Double TopM,
            Double ReachM,
            Double PanRadiusM
            )
        {
            if (PanRadiusM <= 1e-9)
            {
                return 0.0;
            }

            Double Toe = TopM + ReachM;
            Int32 Steps = 2000;
            Double Dr = Toe / Steps;
            Double Volume = 0.0;
            for (Int32 Index = 0; Index < Steps; Index += 1)
            {
                Double Radius = (Index + Constants.HALF_SCALE) * Dr;
                Double Profile;
                if (Radius <= TopM)
                {
                    // The shoulder is ShoulderM. The top rises toward the well.
                    Profile = ShoulderM + (Cross * (TopM - Radius));
                }
                else if (ReachM <= 1e-9)
                {
                    Profile = 0.0;
                }
                else
                {
                    Profile = ShoulderM * (1.0 - ((Radius - TopM) / ReachM));
                }

                Volume += 2.0 * Math.PI * Radius * Math.Max(0.0, Profile) * Dr;
            }

            return Volume / (Math.PI * PanRadiusM * PanRadiusM);
        }

        /// <summary>
        /// Pad flat-top radius: design-vehicle turn radius plus clearance.
        /// </summary>
        /// <param name="TurnRadiusFt">Design-vehicle turn radius in feet.</param>
        /// <param name="ClearanceFt">Clearance outside that radius, in feet.</param>
        /// <returns>Top radius in feet.</returns>
        static public Double PadTopRadiusFt
            (
            Double TurnRadiusFt,
            Double ClearanceFt
            )
        {
            return TurnRadiusFt + ClearanceFt;
        }

        /// <summary>
        /// How far inside the boundary an edge-road centreline must sit.
        /// </summary>
        /// <param name="Options">Road defaults.</param>
        /// <param name="Road">Road whose overrides apply.</param>
        /// <returns>Inset in feet. Defaults are 10 ft.</returns>
        static public Double RequiredCentrelineInsetFt
            (
            HaulBuilderOptions Options,
            RoadCentreline Road
            )
        {
            Double Lane = LaneWidthFt(Options);
            Double Bed = Road.MinRoadbedFt ?? Options.RoadNrcsMinRoadbedFt;
            Int32 Lanes = Road.MinLanes ?? Options.RoadMinLanes;
            Double Height = Road.HeightFt ?? Options.RoadHeightFt;
            Double Side = Road.SideSlope ?? Options.RoadSideSlope;
            Double Top = TopWidthFt(Lane, Bed, Lanes, Side, Height);
            return (Top * Constants.HALF_SCALE) + (Side * Height);
        }

        /// <summary>
        /// Height of one road above the field design, in metres.
        /// </summary>
        /// <param name="X">Easting in metres.</param>
        /// <param name="Y">Northing in metres.</param>
        /// <param name="Road">Centreline.</param>
        /// <param name="Options">Defaults and blade width.</param>
        /// <returns>Height in metres. Zero outside the toe.</returns>
        static public Double RoadHeightM
            (
            Double X,
            Double Y,
            RoadCentreline Road,
            HaulBuilderOptions Options
            )
        {
            Feature Built = Feature.FromRoad(Road, 0, Options);
            return Built.HeightAt(X, Y);
        }

        /// <summary>
        /// Height of one pad above the field design, in metres.
        /// </summary>
        /// <param name="X">Easting in metres.</param>
        /// <param name="Y">Northing in metres.</param>
        /// <param name="Pad">Well pad.</param>
        /// <param name="Options">Defaults.</param>
        /// <returns>Height in metres. Zero outside the toe.</returns>
        static public Double PadHeightM
            (
            Double X,
            Double Y,
            WellPad Pad,
            HaulBuilderOptions Options
            )
        {
            Feature Built = Feature.FromPad(Pad, 0, Options, null);
            return Built.HeightAt(X, Y);
        }

        /// <summary>
        /// One of the 4 by 4 sample points in a cell.
        /// </summary>
        /// <param name="Grid">Grid that owns the cell.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="Column">Sample column, 0 through 3.</param>
        /// <param name="Row">Sample row, 0 through 3.</param>
        /// <returns>Sample easting and northing in metres.</returns>
        static public (Double X, Double Y) Subpoint
            (
            Grid Grid,
            Int32 Bx,
            Int32 By,
            Int32 Column,
            Int32 Row
            )
        {
            Double OriginX = Grid.MinX + (Bx * Grid.GridSizeM);
            Double OriginY = Grid.MinY + (By * Grid.GridSizeM);
            Double Step = Grid.GridSizeM / SUBPOINT_COUNT;
            return (
                OriginX + ((Column + Constants.HALF_SCALE) * Step),
                OriginY + ((Row + Constants.HALF_SCALE) * Step));
        }

        /// <summary>
        /// Fits the field design, writes the early surface onto the grid, and
        /// returns the roads and pads. The grid target becomes the field design
        /// plus reserved cut, so field stages do not build the embankment.
        /// </summary>
        /// <param name="Grid">Planning grid. Its target is replaced.</param>
        /// <param name="Options">Roads, pads, and defaults.</param>
        /// <param name="Boundary">Field boundary ring, or null when the valid mask is the field.</param>
        /// <returns>The surface, the report, and the warnings.</returns>
        static public RoadPadSurface Prepare
            (
            Grid Grid,
            HaulBuilderOptions Options,
            IReadOnlyList<(Double X, Double Y)>? Boundary
            )
        {
            if (Grid == null)
            {
                throw new ArgumentNullException(nameof(Grid));
            }

            Validate(Options);
            RoadPadSurface Surface = new RoadPadSurface(Grid.Width, Grid.Height);
            List<RoadCentreline> Kept = DropDuplicates(Options.Roads, Surface.Warnings);
            Surface.KeptRoads.AddRange(Kept);
            Surface.KeptPads.AddRange(Options.WellPads);
            List<Feature> Features = new List<Feature>();
            for (Int32 Index = 0; Index < Kept.Count; Index += 1)
            {
                Feature RoadFeature = Feature.FromRoad(Kept[Index], Index, Options);
                // Axis-aligned segments move onto whole cells before the heights are sampled.
                RoadFeature.SnapPan(Grid);
                Features.Add(RoadFeature);
            }

            for (Int32 Index = 0; Index < Options.WellPads.Count; Index += 1)
            {
                Features.Add(Feature.FromPad(Options.WellPads[Index], Index, Options, Kept));
            }

            List<(Double X, Double Y)>? Ring = null;
            if ((Boundary != null) && (Boundary.Count >= 3))
            {
                Ring = new List<(Double X, Double Y)>(Boundary);
            }

            HashSet<(Int32 A, Int32 B)> Conflicts = new HashSet<(Int32 A, Int32 B)>();
            Double Area = Grid.AreaM2;
            Double BankPerFullCellPerMetre = Area * Constants.M3_TO_CY * Grid.Compaction;
            Double[] RoadBank = new Double[Kept.Count];
            Double[] PadBank = new Double[Options.WellPads.Count];
            Double[] PadExcluded = new Double[Options.WellPads.Count];
            Int32[] Wins = new Int32[Features.Count];
            Double[] SampleHeight = new Double[Features.Count];
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Int32 Inside = 0;
                    Double Sum = 0.0;
                    Array.Clear(Wins, 0, Wins.Length);
                    for (Int32 Row = 0; Row < SUBPOINT_COUNT; Row += 1)
                    {
                        for (Int32 Column = 0; Column < SUBPOINT_COUNT; Column += 1)
                        {
                            (Double X, Double Y) Point = Subpoint(Grid, Bx, By, Column, Row);
                            if ((Ring != null) && !GridMath.PointInPolygon(Point.X, Point.Y, Ring))
                            {
                                continue;
                            }

                            Inside += 1;
                            Double Best = 0.0;
                            Int32 BestIndex = -1;
                            for (Int32 FeatureIndex = 0; FeatureIndex < Features.Count; FeatureIndex += 1)
                            {
                                Double Height = Features[FeatureIndex].HeightAt(Point.X, Point.Y);
                                SampleHeight[FeatureIndex] = Height;
                                // A later feature wins only when it is strictly higher,
                                // so an equal height keeps the lower input index.
                                if (Height > (Best + 1e-9))
                                {
                                    Best = Height;
                                    BestIndex = FeatureIndex;
                                }
                            }

                            for (Int32 Left = 0; Left < Features.Count; Left += 1)
                            {
                                if (Features[Left].IsPad || (SampleHeight[Left] <= 0.0))
                                {
                                    continue;
                                }

                                for (Int32 Right = Left + 1; Right < Features.Count; Right += 1)
                                {
                                    if (Features[Right].IsPad || (SampleHeight[Right] <= 0.0))
                                    {
                                        continue;
                                    }

                                    Double DifferenceFt = Math.Abs(SampleHeight[Left] - SampleHeight[Right]) / Constants.FEET_TO_METERS;
                                    if (DifferenceFt > HEIGHT_CONFLICT_FT)
                                    {
                                        Conflicts.Add((Features[Left].RoadIndex, Features[Right].RoadIndex));
                                    }
                                }
                            }

                            Sum += Best;
                            if (BestIndex >= 0)
                            {
                                Wins[BestIndex] += 1;
                            }
                        }
                    }

                    if (Inside == 0)
                    {
                        continue;
                    }

                    // Mean height on the inside portion. Volume uses the inside fraction.
                    Double Mean = Sum / Inside;
                    Double Weight = Inside / (Double)(SUBPOINT_COUNT * SUBPOINT_COUNT);
                    Surface.RelM[By, Bx] = Mean;
                    Surface.Weight[By, Bx] = Weight;
                    Int32 Owner = -1;
                    Int32 OwnerWins = 0;
                    for (Int32 FeatureIndex = 0; FeatureIndex < Features.Count; FeatureIndex += 1)
                    {
                        if (Wins[FeatureIndex] > OwnerWins)
                        {
                            OwnerWins = Wins[FeatureIndex];
                            Owner = FeatureIndex;
                        }
                    }

                    if ((Owner < 0) || (Mean <= 0.0))
                    {
                        continue;
                    }

                    Feature Winner = Features[Owner];
                    Surface.OwnerIsPad[By, Bx] = Winner.IsPad;
                    Surface.OwnerIndex[By, Bx] = Winner.IsPad ? Winner.PadIndex : Winner.RoadIndex;
                    Double Bank = Mean * Weight * BankPerFullCellPerMetre;
                    if (Winner.IsPad)
                    {
                        PadBank[Winner.PadIndex] += Bank;
                        Double CentreDistance = Distance(
                            Grid.CenterX[By, Bx],
                            Grid.CenterY[By, Bx],
                            Winner.X,
                            Winner.Y);
                        if (CentreDistance <= Winner.ExclusionM)
                        {
                            // The casing keeps the pad target. The pan does not enter it; the blade places that fill.
                            Surface.Excluded[By, Bx] = true;
                            PadExcluded[Winner.PadIndex] += Bank;
                        }
                    }
                    else
                    {
                        RoadBank[Winner.RoadIndex] += Bank;
                    }
                }
            }

            Double RoadFill = 0.0;
            for (Int32 Index = 0; Index < RoadBank.Length; Index += 1)
            {
                RoadFill += RoadBank[Index];
            }

            for (Int32 Index = 0; Index < PadBank.Length; Index += 1)
            {
                RoadFill += PadBank[Index];
            }

            Surface.RoadFillBcy = RoadFill;
            WritePan(Grid, Surface, Features);
            Double[,] Incoming = (Double[,])Grid.Target.Clone();
            Double Shift = 0.0;
            if (RoadFill > 1e-6)
            {
                // The shift pays for the pan. Blade shaping does not change that volume.
                Shift = SolveShift(Grid, Incoming, Surface.PanM);
            }

            Surface.FieldShiftM = Shift;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        Surface.FieldDesignM[By, Bx] = Double.NaN;
                        Surface.FinalDesignM[By, Bx] = Double.NaN;
                        continue;
                    }

                    Double Field = Incoming[By, Bx] + Shift;
                    Surface.FieldDesignM[By, Bx] = Field;
                    Surface.FinalDesignM[By, Bx] = Field + Surface.RelM[By, Bx];
                }
            }

            CheckRoadTop(Grid, Surface, Features);
            WriteReports(Surface, Features, RoadBank, PadBank, PadExcluded);
            WarnConflicts(Surface, Conflicts);
            WarnGrades(Grid, Surface, Features);
            WarnBends(Surface, Features, Options);
            WarnBoundary(Surface, Features, Ring);
            WarnUnreachedPads(Surface, Features);
            WarnOpenEnds(Grid, Surface, Features, Ring);
            AssignStageCut(Grid, Surface);
            ReserveCut(Grid, Surface, Options);
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    if (Surface.PanM[By, Bx] > LIFT_EPSILON_M)
                    {
                        // The pan is the floor. Main stages may cut surplus down to it, and they do not fill it.
                        Grid.Target[By, Bx] = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    }
                    else
                    {
                        Grid.Target[By, Bx] = Surface.FieldDesignM[By, Bx] + Surface.ReservedDepthM[By, Bx];
                    }
                }
            }

            Surface.BladeShapingHours =
                (Options.BladeShapingHoursPerFt * Surface.CentrelineLengthFt)
                + (Options.BladeShapingHoursPerPad * Surface.Pads.Count);
            return Surface;
        }

        /// <summary>
        /// Plans the pan trips that cut the reserved soil and fill the roads and pads.
        /// Excluded casing cells are left out.
        /// </summary>
        /// <param name="Grid">Grid used for cell centres and the time estimate.</param>
        /// <param name="Surface">Surface from <see cref="Prepare"/>.</param>
        /// <param name="Options">Bowl, depth limit, and speeds.</param>
        /// <returns>Road-stage passes in lane order.</returns>
        static public List<Pass> BuildPasses
            (
            Grid Grid,
            RoadPadSurface Surface,
            HaulBuilderOptions Options
            )
        {
            if (Grid == null)
            {
                throw new ArgumentNullException(nameof(Grid));
            }

            if (Surface == null)
            {
                throw new ArgumentNullException(nameof(Surface));
            }

            List<Feature> Features = new List<Feature>();
            for (Int32 Index = 0; Index < Surface.KeptRoads.Count; Index += 1)
            {
                Feature RoadFeature = Feature.FromRoad(Surface.KeptRoads[Index], Index, Options);
                RoadFeature.SnapPan(Grid);
                Features.Add(RoadFeature);
            }

            for (Int32 Index = 0; Index < Surface.KeptPads.Count; Index += 1)
            {
                Features.Add(Feature.FromPad(Surface.KeptPads[Index], Index, Options, Surface.KeptRoads));
            }

            Double LaneM = LaneWidthFt(Options) * Constants.FEET_TO_METERS;
            Boolean[,] Claimed = new Boolean[Grid.Height, Grid.Width];
            // The reserve fills the pan first. Surplus on the bank is used only after that reserve is gone.
            RefreshReserveLeft(Surface);
            List<Stock> SurplusStocks = BuildStocks(Grid, Surface, Features, false);
            List<Pass> Passes = new List<Pass>();
            Int32 PadTrips = 0;
            Surface.BladeCuts.Clear();
            Surface.BladeFills.Clear();
            Double BladeBcy = 0.0;
            Double MinLoad = Options.RoadMinTripLoadPct;
            Double BladeMaxM = Options.RoadBladeMoveMaxFt * Constants.FEET_TO_METERS;
            Double Bowl = Options.ScraperCapacityLcy / Options.SwellFactor;
            Double MaxDepth = Options.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            Double FillPerM = Grid.AreaM2 * Constants.M3_TO_CY * Grid.Compaction;
            Double CutPerM = Grid.AreaM2 * Constants.M3_TO_CY;
            // The pad is planned first. A road planned ahead of it takes the nearby trim,
            // and the pad is then left with hauls that are too light and too long.
            List<Feature> BuildOrder = new List<Feature>();
            foreach (Feature Item in Features)
            {
                if (Item.IsPad)
                {
                    BuildOrder.Add(Item);
                }
            }

            foreach (Feature Item in Features)
            {
                if (!Item.IsPad)
                {
                    BuildOrder.Add(Item);
                }
            }

            foreach (Feature Item in BuildOrder)
            {
                Dictionary<Int32, List<LaneCell>> Lanes = new Dictionary<Int32, List<LaneCell>>();
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        if (Claimed[By, Bx] || Surface.Excluded[By, Bx] || (Surface.PanM[By, Bx] <= LIFT_EPSILON_M))
                        {
                            continue;
                        }

                        // Fill only the gap up to the pan. Cells already at or above it are not filled.
                        Double Gap = (Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx]) - Grid.Existing[By, Bx];
                        if (Gap <= LIFT_EPSILON_M)
                        {
                            continue;
                        }

                        if ((Surface.OwnerIndex[By, Bx] < 0)
                            || (Surface.OwnerIsPad[By, Bx] != Item.IsPad)
                            || (Surface.OwnerIndex[By, Bx] != (Item.IsPad ? Item.PadIndex : Item.RoadIndex)))
                        {
                            continue;
                        }

                        Double X = Grid.CenterX[By, Bx];
                        Double Y = Grid.CenterY[By, Bx];
                        Int32 Lane;
                        Double Order;
                        if (Item.IsPad)
                        {
                            Double Offset = ((X - Item.X) * Item.PerpX) + ((Y - Item.Y) * Item.PerpY);
                            Order = ((X - Item.X) * Item.DirX) + ((Y - Item.Y) * Item.DirY);
                            Lane = LaneIndex(Offset, LaneM);
                        }
                        else
                        {
                            Hit Closest = Item.Closest(X, Y);
                            Lane = LaneIndex(Closest.Signed, LaneM);
                            Order = Closest.Station;
                        }

                        if (!Lanes.TryGetValue(Lane, out List<LaneCell>? Cells))
                        {
                            Cells = new List<LaneCell>();
                            Lanes[Lane] = Cells;
                        }

                        Cells.Add(new LaneCell(Bx, By, Order, Gap));
                        Claimed[By, Bx] = true;
                    }
                }

                List<Int32> Keys = new List<Int32>(Lanes.Keys);
                Keys.Sort(CompareLanes);
                List<List<LaneCell>> Ordered = new List<List<LaneCell>>();
                foreach (Int32 Key in Keys)
                {
                    List<LaneCell> Cells = Lanes[Key];
                    Cells.Sort((Left, Right) => Left.Order.CompareTo(Right.Order));
                    Ordered.Add(Cells);
                }

                // One direction on the bank. A lane change waits until the next lane is an S-bend ahead.
                Double RadiusM = Options.MinimumTurningDiameterFt * Constants.FEET_TO_METERS * Constants.HALF_SCALE;
                Int32 Before = Passes.Count;
                // The cut is a ribbon along the reserve lane, any heading. The spread stays on the bank lanes.
                Passes.AddRange(EmitReserveRibbons(
                    Grid,
                    Item,
                    Ordered,
                    Keys,
                    Surface,
                    Bowl,
                    MaxDepth,
                    FillPerM,
                    CutPerM,
                    LaneM,
                    RadiusM,
                    MinLoad,
                    Surface.BladeCuts,
                    Surface.BladeFills,
                    ref BladeBcy));
                if (ReserveLeft(Surface) <= 1e-4)
                {
                    Passes.AddRange(EmitForward(
                        Grid,
                        Surface,
                        Item,
                        Ordered,
                        Keys,
                        SurplusStocks,
                        Bowl,
                        MaxDepth,
                        FillPerM,
                        CutPerM,
                        LaneM,
                        RadiusM,
                        MinLoad,
                        BladeMaxM,
                        Surface.BladeCuts,
                        Surface.BladeFills,
                        ref BladeBcy,
                        true));
                }

                if (Item.IsPad)
                {
                    PadTrips += Passes.Count - Before;
                }
            }

            // Light drafts plus the casing the pan was told to leave.
            Surface.BladeMoveBcy = BladeBcy + ExcludedFillBcy(Grid, Surface);
            Surface.PadTripCount = PadTrips;

            // Hours, turnaround, and the return come from the same guidance every other stage uses.
            GuideRoadStage(Grid, Surface, Passes, Options, Bowl);
            WriteStageDiagnostics(Surface, Passes, Bowl);
            Double RoadCut = 0.0;
            Double FieldCut = 0.0;
            foreach (Pass Trip in Passes)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cut in Trip.CutCells)
                {
                    Double Bank = Cut.DepthM * CutPerM;
                    if (Surface.PanM[Cut.Cell.By, Cut.Cell.Bx] > LIFT_EPSILON_M)
                    {
                        RoadCut += Bank;
                    }
                    else
                    {
                        FieldCut += Bank;
                    }
                }
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cut in Surface.BladeCuts)
            {
                Double Bank = Cut.DepthM * CutPerM;
                if (Surface.PanM[Cut.Cell.By, Cut.Cell.Bx] > LIFT_EPSILON_M)
                {
                    RoadCut += Bank;
                }
                else
                {
                    FieldCut += Bank;
                }
            }

            Surface.RoadCellCutBcy = RoadCut;
            Surface.FieldCutBcy = FieldCut;
            return Passes;
        }

        /// <summary>
        /// Bank fill on excluded casing cells. The pan does not place it, so the blade does.
        /// </summary>
        /// <param name="Grid">Existing ground and cell area.</param>
        /// <param name="Surface">Pad target and the exclusion mask.</param>
        /// <returns>Bank cubic yards above the existing ground.</returns>
        private static Double ExcludedFillBcy
            (
            Grid Grid,
            RoadPadSurface Surface
            )
        {
            Double FillPerM = Grid.AreaM2 * Constants.M3_TO_CY * Grid.Compaction;
            Double Sum = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Surface.Excluded[By, Bx])
                    {
                        continue;
                    }

                    Double Gap = (Surface.FieldDesignM[By, Bx] + Surface.RelM[By, Bx]) - Grid.Existing[By, Bx];
                    if (Gap <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    // Weight is the fraction of the cell that sits on the pad.
                    Sum += Gap * Surface.Weight[By, Bx] * FillPerM;
                }
            }

            return Sum;
        }

        /// <summary>
        /// Hours per bank cubic yard of the field passes, using the same time estimate as the road stage.
        /// </summary>
        /// <param name="Grid">Grid used for the time estimate.</param>
        /// <param name="Passes">Field passes. Road-stage passes are ignored.</param>
        /// <param name="Options">Speeds and field efficiency.</param>
        /// <returns>Hours per BCY, or null when the field passes move no soil.</returns>
        static public Double? FieldHoursPerBcy
            (
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            HaulBuilderOptions Options
            )
        {
            if ((Grid == null) || (Passes == null) || (Options == null))
            {
                return null;
            }

            TimeParams Clock = new TimeParams(
                Options.CutSpeedMph,
                Options.LoadedTravelSpeedMph,
                Options.FillSpeedMph,
                Options.UnloadedTravelSpeedMph,
                Options.AccelerationTimeS,
                Options.DecelerationTimeS,
                Options.FieldEfficiency);
            Double Seconds = 0.0;
            Double Load = 0.0;
            foreach (Pass Pass in Passes)
            {
                if (Pass.Stage == HaulStage.Road)
                {
                    continue;
                }

                Seconds += TimeModel.EstimatePassTime(Grid, Pass, Clock);
                Load += Pass.LoadBcy;
            }

            if (Load <= 1e-6)
            {
                return null;
            }

            return (Seconds / Constants.SECONDS_PER_HOUR) / Load;
        }

        /// <summary>
        /// Counts cells that are cut below the surface their stage aims at and also filled.
        /// Pan cells aim at the pan. Every other cell aims at the field design.
        /// </summary>
        /// <param name="Grid">Existing ground.</param>
        /// <param name="Surface">Pan and field design.</param>
        /// <param name="Passes">Every haul in every stage.</param>
        /// <returns>The number of double-handled cells.</returns>
        static public Int32 CountCutBelowAndFilled
            (
            Grid Grid,
            RoadPadSurface Surface,
            IReadOnlyList<Pass> Passes
            )
        {
            if ((Grid == null) || (Surface == null) || (Passes == null))
            {
                return 0;
            }

            Double[,] Cut = new Double[Grid.Height, Grid.Width];
            Double[,] Fill = new Double[Grid.Height, Grid.Width];
            foreach (Pass Trip in Passes)
            {
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Trip.CutCells)
                {
                    Cut[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
                }

                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Trip.FillCells)
                {
                    Fill[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
                }
            }

            Int32 Count = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx] || (Fill[By, Bx] <= 1e-4))
                    {
                        continue;
                    }

                    Double Floor = (Surface.PanM[By, Bx] > LIFT_EPSILON_M)
                        ? (Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx])
                        : Surface.FieldDesignM[By, Bx];
                    Double Allowance = Math.Max(0.0, Grid.Existing[By, Bx] - Floor);
                    if (Cut[By, Bx] > (Allowance + 1e-4))
                    {
                        Count += 1;
                    }
                }
            }

            return Count;
        }

        /// <summary>
        /// Writes the finished design onto the grid target.
        /// </summary>
        /// <param name="Grid">Grid to update.</param>
        /// <param name="Surface">Surface whose final design is copied.</param>
        static public void PublishFinal
            (
            Grid Grid,
            RoadPadSurface Surface
            )
        {
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx])
                    {
                        Grid.Target[By, Bx] = Surface.FinalDesignM[By, Bx];
                    }
                }
            }
        }

        /// <summary>
        /// Bank volume of the pointwise minimum of two roads, using the same
        /// 4 by 4 samples as <see cref="Prepare"/>.
        /// </summary>
        /// <param name="Grid">Grid to sample.</param>
        /// <param name="First">First road.</param>
        /// <param name="Second">Second road.</param>
        /// <param name="Options">Defaults and blade width.</param>
        /// <param name="Boundary">Field boundary, or null.</param>
        /// <returns>Overlap fill in BCY.</returns>
        static internal Double OverlapBankBcy
            (
            Grid Grid,
            RoadCentreline First,
            RoadCentreline Second,
            HaulBuilderOptions Options,
            IReadOnlyList<(Double X, Double Y)>? Boundary
            )
        {
            Feature Left = Feature.FromRoad(First, 0, Options);
            Feature Right = Feature.FromRoad(Second, 1, Options);
            List<(Double X, Double Y)>? Ring = null;
            if ((Boundary != null) && (Boundary.Count >= 3))
            {
                Ring = new List<(Double X, Double Y)>(Boundary);
            }

            Double Sum = 0.0;
            Double Scale = Grid.AreaM2 * Constants.M3_TO_CY * Grid.Compaction / (SUBPOINT_COUNT * SUBPOINT_COUNT);
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    for (Int32 Row = 0; Row < SUBPOINT_COUNT; Row += 1)
                    {
                        for (Int32 Column = 0; Column < SUBPOINT_COUNT; Column += 1)
                        {
                            (Double X, Double Y) Point = Subpoint(Grid, Bx, By, Column, Row);
                            if ((Ring != null) && !GridMath.PointInPolygon(Point.X, Point.Y, Ring))
                            {
                                continue;
                            }

                            Double Height = Math.Min(Left.HeightAt(Point.X, Point.Y), Right.HeightAt(Point.X, Point.Y));
                            Sum += Height * Scale;
                        }
                    }
                }
            }

            return Sum;
        }

        /// <summary>
        /// Rejects a non-positive finite value.
        /// </summary>
        /// <param name="Value">Value to check.</param>
        /// <param name="Name">Option name.</param>
        private static void Positive
            (
            Double Value,
            String Name
            )
        {
            if (!Double.IsFinite(Value) || (Value <= 0.0))
            {
                throw new ArgumentOutOfRangeException(Name, "Value must be finite and greater than zero.");
            }
        }

        /// <summary>
        /// Rejects a negative or non-finite value.
        /// </summary>
        /// <param name="Value">Value to check.</param>
        /// <param name="Name">Option name.</param>
        private static void NonNegative
            (
            Double Value,
            String Name
            )
        {
            if (!Double.IsFinite(Value) || (Value < 0.0))
            {
                throw new ArgumentOutOfRangeException(Name, "Value must be finite and at least zero.");
            }
        }

        /// <summary>
        /// Drops a later centreline whose vertices all lie within 0.1 ft of an earlier one.
        /// </summary>
        /// <param name="Roads">Roads in input order.</param>
        /// <param name="Warnings">Warning list.</param>
        /// <returns>Roads that were kept.</returns>
        private static List<RoadCentreline> DropDuplicates
            (
            IReadOnlyList<RoadCentreline> Roads,
            List<String> Warnings
            )
        {
            List<RoadCentreline> Kept = new List<RoadCentreline>();
            Double ToleranceM = DUPLICATE_VERTEX_FT * Constants.FEET_TO_METERS;
            for (Int32 Index = 0; Index < Roads.Count; Index += 1)
            {
                RoadCentreline Road = Roads[Index];
                Int32 Match = -1;
                for (Int32 Earlier = 0; Earlier < Kept.Count; Earlier += 1)
                {
                    if (SamePolyline(Road, Kept[Earlier], ToleranceM))
                    {
                        Match = Earlier;
                        break;
                    }
                }

                if (Match >= 0)
                {
                    Warnings.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Dropped duplicate road {0} (all vertices within 0.1 ft of road {1}).",
                        Index + 1,
                        Match + 1));
                    continue;
                }

                Kept.Add(Road);
            }

            return Kept;
        }

        /// <summary>
        /// True when every vertex of one road lies within the tolerance of the
        /// other, in the same order or reversed.
        /// </summary>
        /// <param name="Left">First road.</param>
        /// <param name="Right">Second road.</param>
        /// <param name="ToleranceM">Match tolerance in metres.</param>
        /// <returns>True when the polylines are the same.</returns>
        private static Boolean SamePolyline
            (
            RoadCentreline Left,
            RoadCentreline Right,
            Double ToleranceM
            )
        {
            if (Left.Vertices.Count != Right.Vertices.Count)
            {
                return false;
            }

            Boolean Forward = true;
            Boolean Reverse = true;
            Int32 Count = Left.Vertices.Count;
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                if (Distance(Left.Vertices[Index].X, Left.Vertices[Index].Y, Right.Vertices[Index].X, Right.Vertices[Index].Y) > ToleranceM)
                {
                    Forward = false;
                }

                (Double X, Double Y) Tail = Right.Vertices[Count - 1 - Index];
                if (Distance(Left.Vertices[Index].X, Left.Vertices[Index].Y, Tail.X, Tail.Y) > ToleranceM)
                {
                    Reverse = false;
                }
            }

            return Forward || Reverse;
        }

        /// <summary>
        /// Solves the uniform design shift that balances the field design with the pan on top.
        /// </summary>
        /// <param name="Grid">Grid areas and compaction.</param>
        /// <param name="Incoming">Incoming target elevations.</param>
        /// <param name="PanM">Pan height above the field design. Zero off the pan.</param>
        /// <returns>Shift in metres added to the incoming target. Negative lowers it.</returns>
        private static Double SolveShift
            (
            Grid Grid,
            Double[,] Incoming,
            Double[,] PanM
            )
        {
            Double SurplusAt(Double Shift)
            {
                Double Cut = 0.0;
                Double Fill = 0.0;
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        if (!Grid.Valid[By, Bx])
                        {
                            continue;
                        }

                        Double Delta = Grid.Existing[By, Bx] - (Incoming[By, Bx] + Shift + PanM[By, Bx]);
                        if (Delta > 0.0)
                        {
                            Cut += Delta;
                        }
                        else
                        {
                            Fill += -Delta;
                        }
                    }
                }

                Double Supply = Cut * Grid.AreaM2 * Constants.M3_TO_CY;
                Double Demand = Fill * Grid.AreaM2 * Constants.M3_TO_CY * Grid.Compaction;
                return Supply - Demand;
            }

            // Raising the design reduces surplus. Bracket a root from both sides.
            Double Lo = -1.0;
            Double Hi = 1.0;
            for (Int32 Step = 0; (Step < 40) && (SurplusAt(Lo) < 0.0); Step += 1)
            {
                Lo *= 2.0;
            }

            for (Int32 Step = 0; (Step < 40) && (SurplusAt(Hi) > 0.0); Step += 1)
            {
                Hi *= 2.0;
            }

            if ((SurplusAt(Lo) < 0.0) || (SurplusAt(Hi) > 0.0))
            {
                return 0.0;
            }

            for (Int32 Step = 0; Step < 80; Step += 1)
            {
                Double Mid = Constants.HALF_SCALE * (Lo + Hi);
                Double Value = SurplusAt(Mid);
                if ((Math.Abs(Value) < 1e-4) || ((Hi - Lo) < 1e-7))
                {
                    return Mid;
                }

                if (Value > 0.0)
                {
                    Lo = Mid;
                }
                else
                {
                    Hi = Mid;
                }
            }

            return Constants.HALF_SCALE * (Lo + Hi);
        }

        /// <summary>
        /// Fails the surface when a road-top sample is not the crown formula,
        /// or when the final design is not the field design plus the cell height.
        /// </summary>
        /// <param name="Grid">Grid geometry.</param>
        /// <param name="Surface">Surface being checked.</param>
        /// <param name="Features">Resolved roads and pads.</param>
        private static void CheckRoadTop
            (
            Grid Grid,
            RoadPadSurface Surface,
            List<Feature> Features
            )
        {
            Double LaneM = 0.0;
            foreach (Feature Item in Features)
            {
                if (!Item.IsPad)
                {
                    LaneM = Item.LaneM;
                    break;
                }
            }

            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double Applied = Surface.FinalDesignM[By, Bx] - Surface.FieldDesignM[By, Bx];
                    if (Math.Abs(Applied - Surface.RelM[By, Bx]) > ROAD_TOP_TOLERANCE_M)
                    {
                        Surface.RoadTopOk = false;
                        Surface.RoadTopFailure = String.Format(
                            CultureInfo.InvariantCulture,
                            "Road top at cell {0},{1} is not the relative height above the field design.",
                            Bx,
                            By);
                        return;
                    }
                }
            }

            foreach (Feature Item in Features)
            {
                if (Item.IsPad)
                {
                    continue;
                }

                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        if (!Grid.Valid[By, Bx])
                        {
                            continue;
                        }

                        Double CentreDistance = Item.Closest(Grid.CenterX[By, Bx], Grid.CenterY[By, Bx]).Distance;
                        if (CentreDistance > (LaneM * Constants.HALF_SCALE))
                        {
                            continue;
                        }

                        Double Actual = Item.HeightAt(Grid.CenterX[By, Bx], Grid.CenterY[By, Bx]);
                        // The centre of a road-top cell is on the crown. Shoulder height
                        // plus the rise to the centreline, not the 4 by 4 cell mean.
                        Double Expected = Item.HeightM + (Item.Cross * (Item.HalfTopM - CentreDistance));
                        if (Math.Abs(Actual - Expected) > ROAD_TOP_TOLERANCE_M)
                        {
                            Surface.RoadTopOk = false;
                            Surface.RoadTopFailure = String.Format(
                                CultureInfo.InvariantCulture,
                                "Road top at cell {0},{1} is not the shoulder height plus the crown.",
                                Bx,
                                By);
                            return;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Fills the per-road and per-pad report rows.
        /// </summary>
        /// <param name="Surface">Surface being reported.</param>
        /// <param name="Features">Resolved features.</param>
        /// <param name="RoadBank">Attributed road fill in BCY.</param>
        /// <param name="PadBank">Attributed pad fill in BCY.</param>
        /// <param name="PadExcluded">Excluded pad fill in BCY.</param>
        private static void WriteReports
            (
            RoadPadSurface Surface,
            List<Feature> Features,
            Double[] RoadBank,
            Double[] PadBank,
            Double[] PadExcluded
            )
        {
            Double LengthFt = 0.0;
            foreach (Feature Item in Features)
            {
                if (Item.IsPad)
                {
                    Surface.Pads.Add(new RoadPadSurface.PadReport
                    {
                        TopRadiusFt = Item.TopRadiusFt,
                        BankBcy = PadBank[Item.PadIndex],
                        ExcludedBcy = PadExcluded[Item.PadIndex],
                    });
                    continue;
                }

                Double SharedM = SharedLengthM(Item, Features);
                LengthFt += Item.LengthM / Constants.FEET_TO_METERS;
                Surface.Roads.Add(new RoadPadSurface.RoadReport
                {
                    LengthFt = Item.LengthM / Constants.FEET_TO_METERS,
                    TopWidthFt = Item.TopWidthFt,
                    LaneCount = Item.LaneCount,
                    PanWidthFt = Item.PanWidthFt,
                    BuildHeightFt = Item.BuildHeightFt,
                    PanShiftFt = Item.PanShiftFt,
                    PartialPanFraction = Item.PartialPanFraction,
                    NrcsMinRoadbedFt = Item.NrcsFt,
                    HasRoadbedOverride = Item.HasRoadbedOverride,
                    RoadbedOverrideFt = Item.RoadbedOverrideFt,
                    HeightFt = Item.HeightFt,
                    CentrelineFt = Item.CentrelineFt,
                    BankBcy = RoadBank[Item.RoadIndex],
                    SharedLengthFt = SharedM / Constants.FEET_TO_METERS,
                });
            }

            Surface.CentrelineLengthFt = LengthFt;
        }

        /// <summary>
        /// Length of this centreline that lies on another road's top.
        /// </summary>
        /// <param name="Road">Road being measured.</param>
        /// <param name="Features">All features.</param>
        /// <returns>Shared length in metres.</returns>
        private static Double SharedLengthM
            (
            Feature Road,
            List<Feature> Features
            )
        {
            Double Step = SAMPLE_STEP_FT * Constants.FEET_TO_METERS;
            Double Shared = 0.0;
            Double Walked = 0.0;
            while (Walked < (Road.LengthM - 1e-9))
            {
                Double Next = Math.Min(Road.LengthM, Walked + Step);
                (Double X, Double Y) Point = Road.PointAt((Walked + Next) * Constants.HALF_SCALE);
                Boolean OnAnother = false;
                foreach (Feature Other in Features)
                {
                    if (Other.IsPad || ReferenceEquals(Other, Road))
                    {
                        continue;
                    }

                    if (Other.Closest(Point.X, Point.Y).Distance <= Other.HalfTopM)
                    {
                        OnAnother = true;
                        break;
                    }
                }

                if (OnAnother)
                {
                    Shared += Next - Walked;
                }

                Walked = Next;
            }

            return Shared;
        }

        /// <summary>
        /// Warns once for each pair of overlapping roads whose heights differ.
        /// </summary>
        /// <param name="Surface">Surface receiving the warning.</param>
        /// <param name="Conflicts">Pairs of road indexes.</param>
        private static void WarnConflicts
            (
            RoadPadSurface Surface,
            HashSet<(Int32 A, Int32 B)> Conflicts
            )
        {
            foreach ((Int32 A, Int32 B) Pair in Conflicts)
            {
                Surface.Warnings.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Overlapping roads {0} and {1} have RoadHeight overrides that differ by more than 0.1 ft.",
                    Pair.A + 1,
                    Pair.B + 1));
            }
        }

        /// <summary>
        /// Warns when the field-design grade along a centreline exceeds the limit.
        /// </summary>
        /// <param name="Grid">Grid used to sample the field design.</param>
        /// <param name="Surface">Field design and warning list.</param>
        /// <param name="Features">Roads to sample.</param>
        private static void WarnGrades
            (
            Grid Grid,
            RoadPadSurface Surface,
            List<Feature> Features
            )
        {
            Double Step = Math.Max(Grid.GridSizeM, SAMPLE_STEP_FT * Constants.FEET_TO_METERS);
            foreach (Feature Item in Features)
            {
                if (Item.IsPad || (Item.LengthM <= 0.0))
                {
                    continue;
                }

                Double Worst = 0.0;
                Double Station = 0.0;
                (Double X, Double Y) Previous = Item.PointAt(0.0);
                Double PreviousZ = SampleDesign(Grid, Surface, Previous.X, Previous.Y);
                while (Station < Item.LengthM)
                {
                    Station = Math.Min(Item.LengthM, Station + Step);
                    (Double X, Double Y) Point = Item.PointAt(Station);
                    Double Z = SampleDesign(Grid, Surface, Point.X, Point.Y);
                    Double Run = Distance(Previous.X, Previous.Y, Point.X, Point.Y);
                    if ((Run > 1e-6) && Double.IsFinite(PreviousZ) && Double.IsFinite(Z))
                    {
                        Worst = Math.Max(Worst, (Math.Abs(Z - PreviousZ) / Run) * 100.0);
                    }

                    Previous = Point;
                    PreviousZ = Z;
                }

                if (Worst > Item.MaxGradePct)
                {
                    Surface.Warnings.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Road {0} grade {1:F1}% exceeds {2:F1}% on the final field design.",
                        Item.RoadIndex + 1,
                        Worst,
                        Item.MaxGradePct));
                }
            }
        }

        /// <summary>
        /// Warns when the inside edge of a bend is tighter than the design vehicle.
        /// </summary>
        /// <param name="Surface">Warning list.</param>
        /// <param name="Features">Roads to inspect.</param>
        /// <param name="Options">Design-vehicle turn radius.</param>
        private static void WarnBends
            (
            RoadPadSurface Surface,
            List<Feature> Features,
            HaulBuilderOptions Options
            )
        {
            Double LimitM = (Options.PadDesignTurnRadiusFt * Constants.FEET_TO_METERS) - 0.0;
            foreach (Feature Item in Features)
            {
                if (Item.IsPad || (Item.Vertices.Count < 3))
                {
                    continue;
                }

                Double InsideLimit = LimitM - Item.HalfTopM;
                for (Int32 Index = 1; Index < (Item.Vertices.Count - 1); Index += 1)
                {
                    (Double X, Double Y) A = Item.Vertices[Index - 1];
                    (Double X, Double Y) B = Item.Vertices[Index];
                    (Double X, Double Y) C = Item.Vertices[Index + 1];
                    Double Abx = B.X - A.X;
                    Double Aby = B.Y - A.Y;
                    Double Bcx = C.X - B.X;
                    Double Bcy = C.Y - B.Y;
                    Double Ab = Math.Sqrt((Abx * Abx) + (Aby * Aby));
                    Double Bc = Math.Sqrt((Bcx * Bcx) + (Bcy * Bcy));
                    if ((Ab < 1e-6) || (Bc < 1e-6))
                    {
                        continue;
                    }

                    Double Turn = Math.Atan2((Abx * Bcy) - (Aby * Bcx), (Abx * Bcx) + (Aby * Bcy));
                    Double TurnDeg = Math.Abs(Turn) * (180.0 / Math.PI);
                    if (TurnDeg < BEND_STRAIGHT_DEG)
                    {
                        continue;
                    }

                    Double Ac = Distance(A.X, A.Y, C.X, C.Y);
                    Double Area = Math.Abs((Abx * (C.Y - A.Y)) - (Aby * (C.X - A.X))) * Constants.HALF_SCALE;
                    Double Circum = (Area > 1e-8) ? ((Ab * Bc * Ac) / (4.0 * Area)) : Double.PositiveInfinity;
                    Double Half = Math.Min(Ab, Bc) * Constants.HALF_SCALE;
                    Double Tangent = Math.Tan(Math.Abs(Turn) * Constants.HALF_SCALE);
                    Double Fitted = (Tangent > 1e-6) ? (Half / Tangent) : Double.PositiveInfinity;
                    Double Radius = Math.Min(Circum, Fitted);
                    Double Inside = Radius - Item.HalfTopM;
                    if (Inside < InsideLimit)
                    {
                        Surface.Warnings.Add(String.Format(
                            CultureInfo.InvariantCulture,
                            "Road {0} bend at vertex {1} has inside edge radius {2:F1} ft, below {3:F1} ft.",
                            Item.RoadIndex + 1,
                            Index + 1,
                            Inside / Constants.FEET_TO_METERS,
                            InsideLimit / Constants.FEET_TO_METERS));
                    }
                }
            }
        }

        /// <summary>
        /// Warns when a toe falls outside the boundary and states the required inset.
        /// </summary>
        /// <param name="Surface">Warning list.</param>
        /// <param name="Features">Roads and pads.</param>
        /// <param name="Ring">Boundary ring, or null.</param>
        private static void WarnBoundary
            (
            RoadPadSurface Surface,
            List<Feature> Features,
            List<(Double X, Double Y)>? Ring
            )
        {
            if (Ring == null)
            {
                return;
            }

            foreach (Feature Item in Features)
            {
                Boolean Outside = false;
                if (Item.IsPad)
                {
                    if (!Covers(Ring, Item.X, Item.Y, Item.ToeM))
                    {
                        Outside = true;
                    }

                    if (Outside)
                    {
                        Surface.Warnings.Add(String.Format(
                            CultureInfo.InvariantCulture,
                            "Pad {0} toe is outside the field boundary. Required offset {1:F1} ft.",
                            Item.PadIndex + 1,
                            Item.ToeM / Constants.FEET_TO_METERS));
                    }

                    continue;
                }

                Double Step = SAMPLE_STEP_FT * Constants.FEET_TO_METERS;
                for (Double Station = 0.0; Station <= Item.LengthM; Station += Step)
                {
                    (Double X, Double Y) Point = Item.PointAt(Math.Min(Station, Item.LengthM));
                    if (!Covers(Ring, Point.X, Point.Y, Item.ToeM))
                    {
                        Outside = true;
                        break;
                    }
                }

                if (Outside)
                {
                    Surface.Warnings.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Road {0} toe is outside the field boundary. Required offset {1:F1} ft.",
                        Item.RoadIndex + 1,
                        Item.ToeM / Constants.FEET_TO_METERS));
                }
            }
        }

        /// <summary>
        /// True when the point is inside the ring and at least the inset from its edge.
        /// </summary>
        /// <param name="Ring">Boundary ring.</param>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <param name="InsetM">Required distance to the boundary, in metres.</param>
        /// <returns>True when the toe around the point stays inside.</returns>
        private static Boolean Covers
            (
            List<(Double X, Double Y)> Ring,
            Double X,
            Double Y,
            Double InsetM
            )
        {
            if (!GridMath.PointInPolygon(X, Y, Ring))
            {
                return false;
            }

            return DistanceToRing(X, Y, Ring) + 1e-6 >= InsetM;
        }

        /// <summary>
        /// Warns when no road centreline comes within the pad top of the well.
        /// </summary>
        /// <param name="Surface">Warning list.</param>
        /// <param name="Features">Roads and pads.</param>
        private static void WarnUnreachedPads
            (
            RoadPadSurface Surface,
            List<Feature> Features
            )
        {
            foreach (Feature Pad in Features)
            {
                if (!Pad.IsPad)
                {
                    continue;
                }

                Double Best = Double.PositiveInfinity;
                foreach (Feature Road in Features)
                {
                    if (Road.IsPad)
                    {
                        continue;
                    }

                    Best = Math.Min(Best, Road.Closest(Pad.X, Pad.Y).Distance);
                }

                if (Best > Pad.TopRadiusM)
                {
                    Surface.Warnings.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Pad {0} is not reached by a road (no centreline within {1:F1} ft).",
                        Pad.PadIndex + 1,
                        Pad.TopRadiusFt));
                }
            }
        }

        /// <summary>
        /// Reports each road end whose embankment leaves the valid grid, and the bank volume that is not built.
        /// </summary>
        /// <param name="Grid">Valid mask.</param>
        /// <param name="Surface">Road reports that receive the note.</param>
        /// <param name="Features">Roads.</param>
        /// <param name="Ring">Boundary ring, or null when the valid mask is the field.</param>
        private static void WarnOpenEnds
            (
            Grid Grid,
            RoadPadSurface Surface,
            List<Feature> Features,
            List<(Double X, Double Y)>? Ring
            )
        {
            Double Step = Grid.GridSizeM / SUBPOINT_COUNT;
            Double Scale = Step * Step * Constants.M3_TO_CY * Grid.Compaction;
            foreach (Feature Road in Features)
            {
                if (Road.IsPad || (Road.Vertices.Count < 2) || (Road.RoadIndex >= Surface.Roads.Count))
                {
                    continue;
                }

                (Double X, Double Y) Start = Road.Vertices[0];
                (Double X, Double Y) End = Road.Vertices[Road.Vertices.Count - 1];
                Double Pad = Road.ToeM + Step;
                Double MinX = Start.X;
                Double MaxX = Start.X;
                Double MinY = Start.Y;
                Double MaxY = Start.Y;
                foreach ((Double X, Double Y) Vertex in Road.Vertices)
                {
                    MinX = Math.Min(MinX, Vertex.X);
                    MaxX = Math.Max(MaxX, Vertex.X);
                    MinY = Math.Min(MinY, Vertex.Y);
                    MaxY = Math.Max(MaxY, Vertex.Y);
                }

                Int32 I0 = (Int32)Math.Floor(((MinX - Pad) - Grid.MinX) / Step);
                Int32 I1 = (Int32)Math.Ceiling(((MaxX + Pad) - Grid.MinX) / Step);
                Int32 J0 = (Int32)Math.Floor(((MinY - Pad) - Grid.MinY) / Step);
                Int32 J1 = (Int32)Math.Ceiling(((MaxY + Pad) - Grid.MinY) / Step);
                Double[] Off = new Double[2];
                for (Int32 J = J0; J < J1; J += 1)
                {
                    Double Y = Grid.MinY + ((J + Constants.HALF_SCALE) * Step);
                    for (Int32 I = I0; I < I1; I += 1)
                    {
                        Double X = Grid.MinX + ((I + Constants.HALF_SCALE) * Step);
                        Double Height = Road.HeightAt(X, Y);
                        if (Height <= ROAD_TOP_TOLERANCE_M)
                        {
                            continue;
                        }

                        if (SampleBuilt(Grid, Ring, X, Y))
                        {
                            continue;
                        }

                        Double ToStart = Distance(X, Y, Start.X, Start.Y);
                        Double ToEnd = Distance(X, Y, End.X, End.Y);
                        Off[ToStart <= ToEnd ? 0 : 1] += Height * Scale;
                    }
                }

                (Double X, Double Y)[] Ends = new (Double X, Double Y)[] { Start, End };
                for (Int32 Index = 0; Index < Ends.Length; Index += 1)
                {
                    if (Off[Index] <= OPEN_END_BCY)
                    {
                        continue;
                    }

                    String Text = String.Format(
                        CultureInfo.InvariantCulture,
                        "Road {0} open end at {1:F1}, {2:F1}; {3:F2} BCY not built.",
                        Road.RoadIndex + 1,
                        Ends[Index].X,
                        Ends[Index].Y,
                        Off[Index]);
                    Surface.Warnings.Add(Text);
                    Surface.Roads[Road.RoadIndex].OpenEndLines.Add(Text);
                    // The clipped end is not a finish failure. The in-grid cells beside it still are.
                    MarkOpenEnd(Grid, Surface, Road, Ends[Index]);
                }
            }
        }

        /// <summary>
        /// Marks pan cells of one road that sit on an open end.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Surface">Receives the open-end mask.</param>
        /// <param name="Road">Road whose end is open.</param>
        /// <param name="End">The open endpoint.</param>
        private static void MarkOpenEnd
            (
            Grid Grid,
            RoadPadSurface Surface,
            Feature Road,
            (Double X, Double Y) End
            )
        {
            Double Reach = Road.ToeM + Grid.GridSizeM;
            Double Reach2 = Reach * Reach;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx] || (Surface.PanM[By, Bx] <= LIFT_EPSILON_M))
                    {
                        continue;
                    }

                    if (Surface.OwnerIsPad[By, Bx] || (Surface.OwnerIndex[By, Bx] != Road.RoadIndex))
                    {
                        continue;
                    }

                    Double Dx = Grid.CenterX[By, Bx] - End.X;
                    Double Dy = Grid.CenterY[By, Bx] - End.Y;
                    if (((Dx * Dx) + (Dy * Dy)) <= Reach2)
                    {
                        Surface.OpenEnd[By, Bx] = true;
                    }
                }
            }
        }

        /// <summary>
        /// True when a sample is on a valid cell and inside the boundary ring.
        /// </summary>
        /// <param name="Grid">Valid mask.</param>
        /// <param name="Ring">Boundary ring, or null.</param>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <returns>True when the embankment at this sample is built.</returns>
        private static Boolean SampleBuilt
            (
            Grid Grid,
            List<(Double X, Double Y)>? Ring,
            Double X,
            Double Y
            )
        {
            Int32 Bx = (Int32)Math.Floor((X - Grid.MinX) / Grid.GridSizeM);
            Int32 By = (Int32)Math.Floor((Y - Grid.MinY) / Grid.GridSizeM);
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height) || !Grid.Valid[By, Bx])
            {
                return false;
            }

            if ((Ring != null) && !GridMath.PointInPolygon(X, Y, Ring))
            {
                return false;
            }

            return true;
        }

        /// <summary>
        /// Writes the flat pan. Snapped cells are whole. Diagonal cells are weighted by coverage.
        /// </summary>
        /// <param name="Grid">Cell corners.</param>
        /// <param name="Surface">Receives the pan height and the coverage.</param>
        /// <param name="Features">Roads and pads. Roads receive their partial-volume fraction.</param>
        private static void WritePan
            (
            Grid Grid,
            RoadPadSurface Surface,
            List<Feature> Features
            )
        {
            Int32 RoadCount = 0;
            foreach (Feature Item in Features)
            {
                if (!Item.IsPad && (Item.RoadIndex >= RoadCount))
                {
                    RoadCount = Item.RoadIndex + 1;
                }
            }

            Double[] Total = new Double[RoadCount];
            Double[] Partial = new Double[RoadCount];
            Double Half = Grid.GridSizeM * Constants.HALF_SCALE;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx] || Surface.Excluded[By, Bx])
                    {
                        continue;
                    }

                    Double X0 = Grid.MinX + (Bx * Grid.GridSizeM);
                    Double Y0 = Grid.MinY + (By * Grid.GridSizeM);
                    Double X1 = X0 + Grid.GridSizeM;
                    Double Y1 = Y0 + Grid.GridSizeM;
                    Double Best = 0.0;
                    Double BestCoverage = 0.0;
                    Feature? Winner = null;
                    for (Int32 Index = 0; Index < Features.Count; Index += 1)
                    {
                        Feature Item = Features[Index];
                        Double Coverage = Item.IsPad
                            ? (CellInPan(Item, Grid.CenterX[By, Bx], Grid.CenterY[By, Bx], Half) ? 1.0 : 0.0)
                            : Item.Coverage(X0, Y0, X1, Y1);
                        Double Height = Item.BuildHeightM * Coverage;
                        // The earlier feature keeps the cell when the heights are equal.
                        if (Height > (Best + 1e-12))
                        {
                            Best = Height;
                            BestCoverage = Coverage;
                            Winner = Item;
                        }
                    }

                    Surface.PanM[By, Bx] = Best;
                    Surface.PanCoverage[By, Bx] = BestCoverage;
                    if ((Winner == null) || (Best <= 0.0))
                    {
                        continue;
                    }

                    // The haul fills the pan, so the pan owner replaces the finished-surface owner.
                    Surface.OwnerIsPad[By, Bx] = Winner.IsPad;
                    Surface.OwnerIndex[By, Bx] = Winner.IsPad ? Winner.PadIndex : Winner.RoadIndex;
                    if (Winner.IsPad)
                    {
                        continue;
                    }

                    Double Volume = Best * Grid.AreaM2;
                    Total[Winner.RoadIndex] += Volume;
                    if (BestCoverage < (1.0 - 1e-8))
                    {
                        Partial[Winner.RoadIndex] += Volume;
                    }
                }
            }

            foreach (Feature Item in Features)
            {
                if (Item.IsPad)
                {
                    continue;
                }

                Double RoadTotal = Total[Item.RoadIndex];
                Item.PartialPanFraction = (RoadTotal > 1e-12) ? (Partial[Item.RoadIndex] / RoadTotal) : 0.0;
            }
        }

        /// <summary>
        /// True when every corner of the cell lies in a pad disc.
        /// </summary>
        /// <param name="Item">Pad.</param>
        /// <param name="X">Cell centre easting.</param>
        /// <param name="Y">Cell centre northing.</param>
        /// <param name="Half">Half the cell size, in metres.</param>
        /// <returns>True for a whole pad cell.</returns>
        private static Boolean CellInPan
            (
            Feature Item,
            Double X,
            Double Y,
            Double Half
            )
        {
            return Item.InPan(X - Half, Y - Half)
                && Item.InPan(X - Half, Y + Half)
                && Item.InPan(X + Half, Y - Half)
                && Item.InPan(X + Half, Y + Half);
        }

        /// <summary>
        /// Area of a pan rectangle clipped to an axis-aligned cell.
        /// </summary>
        /// <param name="Corners">Pan corners, in order.</param>
        /// <param name="MinX">Cell west.</param>
        /// <param name="MinY">Cell south.</param>
        /// <param name="MaxX">Cell east.</param>
        /// <param name="MaxY">Cell north.</param>
        /// <returns>Area in square metres.</returns>
        private static Double ClippedArea
            (
            (Double X, Double Y)[] Corners,
            Double MinX,
            Double MinY,
            Double MaxX,
            Double MaxY
            )
        {
            List<(Double X, Double Y)> Polygon = new List<(Double X, Double Y)>(Corners);
            Polygon = ClipAxis(Polygon, true, true, MinX);
            Polygon = ClipAxis(Polygon, true, false, MaxX);
            Polygon = ClipAxis(Polygon, false, true, MinY);
            Polygon = ClipAxis(Polygon, false, false, MaxY);
            if (Polygon.Count < 3)
            {
                return 0.0;
            }

            Double Sum = 0.0;
            for (Int32 Index = 0; Index < Polygon.Count; Index += 1)
            {
                (Double X, Double Y) A = Polygon[Index];
                (Double X, Double Y) B = Polygon[(Index + 1) % Polygon.Count];
                Sum += (A.X * B.Y) - (B.X * A.Y);
            }

            return Math.Abs(Sum) * Constants.HALF_SCALE;
        }

        /// <summary>
        /// Clips a polygon to one side of an axis-aligned line. Boundary points stay inside.
        /// </summary>
        /// <param name="Input">Polygon in order.</param>
        /// <param name="Vertical">True to clip on X. False clips on Y.</param>
        /// <param name="KeepGreater">True keeps coordinates at or above the limit.</param>
        /// <param name="Limit">The clip coordinate.</param>
        /// <returns>The clipped polygon. Empty when the input is outside.</returns>
        private static List<(Double X, Double Y)> ClipAxis
            (
            List<(Double X, Double Y)> Input,
            Boolean Vertical,
            Boolean KeepGreater,
            Double Limit
            )
        {
            List<(Double X, Double Y)> Output = new List<(Double X, Double Y)>();
            if (Input.Count == 0)
            {
                return Output;
            }

            for (Int32 Index = 0; Index < Input.Count; Index += 1)
            {
                (Double X, Double Y) Current = Input[Index];
                (Double X, Double Y) Previous = Input[(Index + Input.Count - 1) % Input.Count];
                Boolean CurrentIn = ClipInside(Current, Vertical, KeepGreater, Limit);
                Boolean PreviousIn = ClipInside(Previous, Vertical, KeepGreater, Limit);
                if (CurrentIn)
                {
                    if (!PreviousIn)
                    {
                        Output.Add(ClipCross(Previous, Current, Vertical, Limit));
                    }

                    Output.Add(Current);
                }
                else if (PreviousIn)
                {
                    Output.Add(ClipCross(Previous, Current, Vertical, Limit));
                }
            }

            return Output;
        }

        /// <summary>
        /// True when a point is on the kept side of a clip line.
        /// </summary>
        /// <param name="Point">Point to test.</param>
        /// <param name="Vertical">True when the limit is an easting.</param>
        /// <param name="KeepGreater">True keeps the greater side.</param>
        /// <param name="Limit">Clip coordinate.</param>
        /// <returns>True inside or on the boundary.</returns>
        private static Boolean ClipInside
            (
            (Double X, Double Y) Point,
            Boolean Vertical,
            Boolean KeepGreater,
            Double Limit
            )
        {
            Double Value = Vertical ? Point.X : Point.Y;
            // A point on the line belongs to the covered cell, so the shared edge stays inside.
            if (KeepGreater)
            {
                return Value >= (Limit - 1e-9);
            }

            return Value <= (Limit + 1e-9);
        }

        /// <summary>
        /// Intersection of a segment with an axis-aligned clip line.
        /// </summary>
        /// <param name="A">Segment start.</param>
        /// <param name="B">Segment end.</param>
        /// <param name="Vertical">True when the limit is an easting.</param>
        /// <param name="Limit">Clip coordinate.</param>
        /// <returns>The intersection.</returns>
        private static (Double X, Double Y) ClipCross
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            Boolean Vertical,
            Double Limit
            )
        {
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            if (Vertical)
            {
                Double T = (Math.Abs(Dx) < 1e-18) ? 0.0 : ((Limit - A.X) / Dx);
                return (Limit, A.Y + (T * Dy));
            }

            Double Ty = (Math.Abs(Dy) < 1e-18) ? 0.0 : ((Limit - A.Y) / Dy);
            return (A.X + (Ty * Dx), Limit);
        }

        /// <summary>
        /// Nearest cell boundary or cell centre, never more than half a cell away.
        /// </summary>
        /// <param name="Coord">Coordinate to snap, in metres.</param>
        /// <param name="Origin">Grid origin on this axis.</param>
        /// <param name="Cell">Cell size in metres.</param>
        /// <param name="OnBoundary">True for a cell boundary. False for a cell centre.</param>
        /// <returns>The snapped coordinate.</returns>
        private static Double SnapCoordinate
            (
            Double Coord,
            Double Origin,
            Double Cell,
            Boolean OnBoundary
            )
        {
            if (Cell <= 1e-9)
            {
                return Coord;
            }

            Double Local = (Coord - Origin) / Cell;
            Double Index = OnBoundary
                ? Math.Round(Local, MidpointRounding.AwayFromZero)
                : (Math.Round(Local - Constants.HALF_SCALE, MidpointRounding.AwayFromZero) + Constants.HALF_SCALE);
            Double Target = Origin + (Index * Cell);
            Double Limit = Cell * Constants.HALF_SCALE;
            Double Shift = Target - Coord;
            if (Shift > Limit)
            {
                return Coord + Limit;
            }

            if (Shift < -Limit)
            {
                return Coord - Limit;
            }

            return Target;
        }

        /// <summary>
        /// Records pan surplus and pan deficit before haul planning.
        /// Surplus stays with the main stages. The borrow is the pan deficit times fill-to-bank, not the crown volume.
        /// </summary>
        /// <param name="Grid">Existing ground.</param>
        /// <param name="Surface">Pan and field design. Receives the surplus, the deficit, and the borrow.</param>
        private static void AssignStageCut
            (
            Grid Grid,
            RoadPadSurface Surface
            )
        {
            Double CutPer = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPer = CutPer * Grid.Compaction;
            Double SurplusBank = 0.0;
            Double DeficitBank = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx] || (Surface.PanM[By, Bx] <= LIFT_EPSILON_M))
                    {
                        continue;
                    }

                    Double Target = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    Double Surplus = Math.Max(0.0, Grid.Existing[By, Bx] - Target);
                    Double Deficit = Math.Max(0.0, Target - Grid.Existing[By, Bx]);
                    Surface.SurplusM[By, Bx] = Surplus;
                    Surface.DeficitM[By, Bx] = Deficit;
                    // None of the surplus is kept for the road stage. The main stages haul it to any fill.
                    if (Surplus > LIFT_EPSILON_M)
                    {
                        SurplusBank += Surplus * CutPer;
                    }

                    if (Deficit > LIFT_EPSILON_M)
                    {
                        // Fill-to-bank: a metre of pan fill takes compaction times a metre of cut.
                        DeficitBank += Deficit * FillPer;
                    }
                }
            }

            Surface.RoadSurplusBcy = SurplusBank;
            Surface.RoadDeficitBcy = DeficitBank;
            Surface.PanImportBcy = DeficitBank;
        }

        /// <summary>
        /// Distance from a cell centre to the nearest road or pad.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Features">Roads and pads.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <returns>Distance in metres.</returns>
        private static Double NearestFeatureM
            (
            Grid Grid,
            List<Feature> Features,
            Int32 Bx,
            Int32 By
            )
        {
            Double X = Grid.CenterX[By, Bx];
            Double Y = Grid.CenterY[By, Bx];
            Double Nearest = Double.PositiveInfinity;
            foreach (Feature Item in Features)
            {
                Double DistanceM = Item.IsPad
                    ? Distance(X, Y, Item.X, Item.Y)
                    : Item.Closest(X, Y).Distance;
                Nearest = Math.Min(Nearest, DistanceM);
            }

            return Nearest;
        }

        /// <summary>
        /// Distance from a field cell to the nearest pan cell that still needs fill.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Deficits">Pan cells below the pan target.</param>
        /// <param name="Bx">Column of the field cell.</param>
        /// <param name="By">Row of the field cell.</param>
        /// <returns>Distance in metres. Infinite when there is no deficit.</returns>
        private static Double NearestDeficitM
            (
            Grid Grid,
            List<(Int32 Bx, Int32 By)> Deficits,
            Int32 Bx,
            Int32 By
            )
        {
            Double X = Grid.CenterX[By, Bx];
            Double Y = Grid.CenterY[By, Bx];
            Double Best = Double.PositiveInfinity;
            for (Int32 Index = 0; Index < Deficits.Count; Index += 1)
            {
                (Int32 Bx, Int32 By) Cell = Deficits[Index];
                Double Dx = Grid.CenterX[Cell.By, Cell.Bx] - X;
                Double Dy = Grid.CenterY[Cell.By, Cell.Bx] - Y;
                Double Dist = (Dx * Dx) + (Dy * Dy);
                if (Dist < Best)
                {
                    Best = Dist;
                }
            }

            return Math.Sqrt(Best);
        }

        /// <summary>
        /// The line reported with the road stage: reserve size, the pan deficit, what the road used,
        /// what correcting left, and how many field cells missed the design.
        /// </summary>
        /// <param name="Surface">Surface whose reserve figures are printed.</param>
        /// <returns>The report line.</returns>
        internal static String FormatReserveLine
            (
            RoadPadSurface Surface
            )
        {
            return String.Format(
                CultureInfo.InvariantCulture,
                "Road reserve: size {0:F3} BCY, road deficit {1:F3} BCY, reserve used {2:F3} BCY, unused after correcting {3:F3} BCY, field cells off design {4}.",
                Surface.PanImportBcy,
                Surface.RoadDeficitBcy,
                Surface.ReserveUsedBcy,
                Surface.UnusedReserveBcy,
                Surface.FieldCellsOffDesign);
        }

        /// <summary>
        /// The line reported before the road stage: surplus left, deficit left, and reserve left.
        /// </summary>
        /// <param name="Surface">Surface whose before-stage figures are printed.</param>
        /// <returns>The report line.</returns>
        internal static String FormatBalanceLine
            (
            RoadPadSurface Surface
            )
        {
            return String.Format(
                CultureInfo.InvariantCulture,
                "Road balance: surplus remaining {0:F3} BCY, road deficit {1:F3} BCY, reserve available {2:F3} BCY.",
                Surface.SurplusRemainingBcy,
                Surface.DeficitRemainingBcy,
                Surface.ReserveAvailableBcy);
        }

        /// <summary>
        /// Applies the main-stage passes and measures what the road stage still has to do.
        /// The grid's existing ground becomes the ground those passes leave. The caller restores the survey.
        /// Leftover pan surplus is offered to the road stage. Reserved depth is what the main stages did not take.
        /// </summary>
        /// <param name="Grid">Existing ground. The passes are applied to it.</param>
        /// <param name="Surface">Pan, reserve, and the before-stage figures.</param>
        /// <param name="FieldPasses">Passes driven before the road stage. Road passes are not included.</param>
        internal static void AccountBeforeRoadStage
            (
            Grid Grid,
            RoadPadSurface Surface,
            IReadOnlyList<Pass> FieldPasses
            )
        {
            ApplyPasses(Grid.Existing, FieldPasses);
            Double CutPer = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPer = CutPer * Grid.Compaction;
            Double SurplusBank = 0.0;
            Double DeficitBank = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx] || (Surface.PanM[By, Bx] <= LIFT_EPSILON_M))
                    {
                        continue;
                    }

                    Double Target = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    Double Above = Math.Max(0.0, Grid.Existing[By, Bx] - Target);
                    Double Below = Math.Max(0.0, Target - Grid.Existing[By, Bx]);
                    // The road stage may cut only the surplus the main stages left.
                    Surface.StageCutM[By, Bx] = Above;
                    if (Above > LIFT_EPSILON_M)
                    {
                        SurplusBank += Above * CutPer;
                    }

                    if (Below > LIFT_EPSILON_M)
                    {
                        DeficitBank += Below * FillPer;
                    }
                }
            }

            Double ReserveBank = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Surface.ReservedDepthM[By, Bx] <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    // The reserved layer is the bottom of the cut. Soil still above the field design, up to that layer, is what remains.
                    Double AboveDesign = Grid.Existing[By, Bx] - Surface.FieldDesignM[By, Bx];
                    Double Left = Math.Min(Surface.ReservedDepthM[By, Bx], Math.Max(0.0, AboveDesign));
                    Surface.ReservedDepthM[By, Bx] = Left;
                    ReserveBank += Left * CutPer;
                }
            }

            Surface.SurplusRemainingBcy = SurplusBank;
            Surface.DeficitRemainingBcy = DeficitBank;
            Surface.ReserveAvailableBcy = ReserveBank;
        }

        /// <summary>
        /// Gives correcting the reserve the road stage did not use, and the pan gap the road stage left.
        /// Correcting may trim a pan cell down to its pan target, and it may fill a remaining deficit up to that target.
        /// </summary>
        /// <param name="Grid">Cell size and the valid mask. Existing ground is not changed.</param>
        /// <param name="Surface">Reserve, pan, and the road-stage passes. Receives the used and unused figures.</param>
        /// <param name="RoadPasses">Road-stage passes. Blade cuts on the surface are included.</param>
        /// <param name="CutLeft">Cut still to assign. Pan cut the road took is removed. Unused reserve is left for the correcting close.</param>
        /// <param name="FillLeft">Fill still to assign. Pan fill stays empty until the correcting close.</param>
        /// <param name="Ledger">Required depths. Pan cut the road took is released.</param>
        internal static void HandReserveToCorrecting
            (
            Grid Grid,
            RoadPadSurface Surface,
            IReadOnlyList<Pass> RoadPasses,
            Double[,] CutLeft,
            Double[,] FillLeft,
            HaulWorkLedger Ledger
            )
        {
            Double[,] RoadCut = new Double[Grid.Height, Grid.Width];
            Double[,] RoadFill = new Double[Grid.Height, Grid.Width];
            AddDepths(RoadCut, Surface.BladeCuts);
            AddDepths(RoadFill, Surface.BladeFills);
            if (RoadPasses != null)
            {
                for (Int32 Index = 0; Index < RoadPasses.Count; Index += 1)
                {
                    AddDepths(RoadCut, RoadPasses[Index].CutCells);
                    AddDepths(RoadFill, RoadPasses[Index].FillCells);
                }
            }

            Double CutPer = Grid.AreaM2 * Constants.M3_TO_CY;
            Double UsedBank = 0.0;
            Double UnusedBank = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Surface.UnusedDepthM[By, Bx] = 0.0;
                    Surface.PanGapM[By, Bx] = 0.0;
                    if (Surface.PanM[By, Bx] > LIFT_EPSILON_M)
                    {
                        // The coverage must not fill the pan. The correcting close fills whatever the road left.
                        FillLeft[By, Bx] = 0.0;
                        Double Taken = Math.Min(CutLeft[By, Bx], RoadCut[By, Bx]);
                        if (Taken > LIFT_EPSILON_M)
                        {
                            CutLeft[By, Bx] -= Taken;
                            Ledger.ReleaseRequiredCut(By, Bx, Taken);
                        }

                        if (!Surface.Excluded[By, Bx])
                        {
                            Double Gap = Surface.DeficitM[By, Bx] - RoadFill[By, Bx];
                            if (Gap > LIFT_EPSILON_M)
                            {
                                Surface.PanGapM[By, Bx] = Gap;
                            }
                        }

                        continue;
                    }

                    Double Offered = Surface.ReservedDepthM[By, Bx];
                    if (Offered <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    Double Used = Math.Min(Offered, RoadCut[By, Bx]);
                    Double Unused = Offered - Used;
                    UsedBank += Used * CutPer;
                    if (Unused <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    // Correcting cuts this after the coverage, paired with the pan gap.
                    Surface.UnusedDepthM[By, Bx] = Unused;
                    UnusedBank += Unused * CutPer;
                }
            }

            Surface.ReserveUsedBcy = UsedBank;
            Surface.UnusedReserveBcy = UnusedBank;
        }

        /// <summary>
        /// Cuts the reserve the road stage left and fills the pan gap it left.
        /// Field cells still outside the finish tolerance are then levelled against each other.
        /// The trips are correcting trips. A cell is cut or filled at most one lift deep.
        /// </summary>
        /// <param name="Grid">Cell centres and the yard factors.</param>
        /// <param name="Surface">Unused reserve and the pan gap. The unused figure is updated.</param>
        /// <param name="Passes">Correcting trips are appended.</param>
        /// <param name="Ledger">Required depths for the soil these trips move.</param>
        /// <param name="Options">Bowl and the depth limit.</param>
        internal static void CloseUnusedReserve
            (
            Grid Grid,
            RoadPadSurface Surface,
            List<Pass> Passes,
            HaulWorkLedger Ledger,
            HaulBuilderOptions Options
            )
        {
            if ((Grid == null) || (Surface == null) || (Passes == null) || (Ledger == null) || (Options == null))
            {
                return;
            }

            Double CutPer = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPer = CutPer * Grid.Compaction;
            Double Bowl = Options.ScraperCapacityLcy / Options.SwellFactor;
            Double MaxDepth = Options.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            Int32 Guard = 0;
            while (Guard < 10000)
            {
                Guard += 1;
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                Double Room = Bowl;
                for (Int32 By = 0; (By < Grid.Height) && (Room > 1e-4); By += 1)
                {
                    for (Int32 Bx = 0; (Bx < Grid.Width) && (Room > 1e-4); Bx += 1)
                    {
                        if (Surface.UnusedDepthM[By, Bx] <= LIFT_EPSILON_M)
                        {
                            continue;
                        }

                        Double Depth = Math.Min(Surface.UnusedDepthM[By, Bx], Math.Min(MaxDepth, Room / Math.Max(CutPer, 1e-9)));
                        if (Depth <= LIFT_EPSILON_M)
                        {
                            continue;
                        }

                        Cuts.Add(((Bx, By), Depth));
                        Surface.UnusedDepthM[By, Bx] -= Depth;
                        Room -= Depth * CutPer;
                    }
                }

                if (Cuts.Count == 0)
                {
                    break;
                }

                Double Need = (Bowl - Room);
                for (Int32 By = 0; (By < Grid.Height) && (Need > 1e-4); By += 1)
                {
                    for (Int32 Bx = 0; (Bx < Grid.Width) && (Need > 1e-4); Bx += 1)
                    {
                        if ((Surface.PanGapM[By, Bx] <= LIFT_EPSILON_M) || Surface.Excluded[By, Bx])
                        {
                            continue;
                        }

                        Double Depth = Math.Min(Surface.PanGapM[By, Bx], Math.Min(MaxDepth, Need / Math.Max(FillPer, 1e-9)));
                        if (Depth <= LIFT_EPSILON_M)
                        {
                            continue;
                        }

                        Fills.Add(((Bx, By), Depth));
                        Surface.PanGapM[By, Bx] -= Depth;
                        Need -= Depth * FillPer;
                    }
                }

                Double Placed = Bowl - Room - Need;
                if ((Fills.Count == 0) || (Placed <= 1e-4))
                {
                    // No pan gap left for this cut. Put the depth back and stop.
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cut in Cuts)
                    {
                        Surface.UnusedDepthM[Cut.Cell.By, Cut.Cell.Bx] += Cut.DepthM;
                    }

                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Fills)
                    {
                        Surface.PanGapM[Fill.Cell.By, Fill.Cell.Bx] += Fill.DepthM;
                    }

                    break;
                }

                // The fill may be short of the cut. Put the extra cut back, from the last cells.
                Double CutLeftBank = (Bowl - Room) - (Bowl - Room - Need);
                while ((CutLeftBank > 1e-4) && (Cuts.Count > 0))
                {
                    Int32 Last = Cuts.Count - 1;
                    ((Int32 Bx, Int32 By) Cell, Double DepthM) Cut = Cuts[Last];
                    Double Bank = Cut.DepthM * CutPer;
                    if (Bank <= (CutLeftBank + 1e-9))
                    {
                        Surface.UnusedDepthM[Cut.Cell.By, Cut.Cell.Bx] += Cut.DepthM;
                        Cuts.RemoveAt(Last);
                        CutLeftBank -= Bank;
                    }
                    else
                    {
                        Double Give = CutLeftBank / Math.Max(CutPer, 1e-9);
                        Surface.UnusedDepthM[Cut.Cell.By, Cut.Cell.Bx] += Give;
                        Cuts[Last] = (Cut.Cell, Cut.DepthM - Give);
                        CutLeftBank = 0.0;
                    }
                }

                if (Cuts.Count == 0)
                {
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Fills)
                    {
                        Surface.PanGapM[Fill.Cell.By, Fill.Cell.Bx] += Fill.DepthM;
                    }

                    break;
                }

                Double Load = 0.0;
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cut in Cuts)
                {
                    Load += Cut.DepthM * CutPer;
                    Ledger.AddRequiredCut(Cut.Cell.By, Cut.Cell.Bx, Cut.DepthM);
                }

                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Fills)
                {
                    Ledger.AddRequiredFill(Fill.Cell.By, Fill.Cell.Bx, Fill.DepthM);
                }

                (Double X, Double Y) CutStart = (Grid.CenterX[Cuts[0].Cell.By, Cuts[0].Cell.Bx], Grid.CenterY[Cuts[0].Cell.By, Cuts[0].Cell.Bx]);
                (Double X, Double Y) CutEnd = (Grid.CenterX[Cuts[Cuts.Count - 1].Cell.By, Cuts[Cuts.Count - 1].Cell.Bx], Grid.CenterY[Cuts[Cuts.Count - 1].Cell.By, Cuts[Cuts.Count - 1].Cell.Bx]);
                (Double X, Double Y) FillStart = (Grid.CenterX[Fills[0].Cell.By, Fills[0].Cell.Bx], Grid.CenterY[Fills[0].Cell.By, Fills[0].Cell.Bx]);
                (Double X, Double Y) FillEnd = (Grid.CenterX[Fills[Fills.Count - 1].Cell.By, Fills[Fills.Count - 1].Cell.Bx], Grid.CenterY[Fills[Fills.Count - 1].Cell.By, Fills[Fills.Count - 1].Cell.Bx]);
                Double Haul = Distance(CutEnd.X, CutEnd.Y, FillStart.X, FillStart.Y);
                Pass Trip = new Pass(Cuts, Fills, Load, Haul, CutStart, CutEnd, FillStart, FillEnd);
                Trip.Stage = HaulStage.Correcting;
                Trip.RibbonOrder = true;
                Passes.Add(Trip);
            }

            Double Left = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Surface.UnusedDepthM[By, Bx] > LIFT_EPSILON_M)
                    {
                        Left += Surface.UnusedDepthM[By, Bx] * CutPer;
                    }
                }
            }

            Surface.UnusedReserveBcy = Left;
            LevelFieldResidual(Grid, Surface, Passes, Ledger, Options);
        }

        /// <summary>
        /// Moves soil from field cells that finished high to field cells that finished low.
        /// The trips are correcting trips. A cell is not cut below its design or filled above it.
        /// </summary>
        /// <param name="Grid">Survey in <see cref="Grid.Existing"/>, cell centres, and the yard factors.</param>
        /// <param name="Surface">Field design, the road passes, and the blade moves.</param>
        /// <param name="Passes">Passes already planned. Correcting trips are appended.</param>
        /// <param name="Ledger">Required depths for the soil these trips move.</param>
        /// <param name="Options">Bowl, depth limit, and the finish tolerance.</param>
        private static void LevelFieldResidual
            (
            Grid Grid,
            RoadPadSurface Surface,
            List<Pass> Passes,
            HaulWorkLedger Ledger,
            HaulBuilderOptions Options
            )
        {
            Double ToleranceM = Options.ToleranceMm / Constants.MILLIMETERS_PER_METER;
            Double[,] Elevation = (Double[,])Grid.Existing.Clone();
            ApplyPasses(Elevation, Passes);
            if (Surface.StagePasses != null)
            {
                ApplyPasses(Elevation, Surface.StagePasses);
            }

            ApplyCells(Elevation, Surface.BladeCuts, -1.0);
            ApplyCells(Elevation, Surface.BladeFills, 1.0);
            Double CutPer = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPer = CutPer * Grid.Compaction;
            Double Bowl = Options.ScraperCapacityLcy / Options.SwellFactor;
            Double MaxDepth = Options.MaxDepthMm / Constants.MILLIMETERS_PER_METER;
            Int32 Guard = 0;
            while (Guard < 1000)
            {
                Guard += 1;
                Boolean OffDesign = false;
                List<(Int32 Bx, Int32 By, Double ExcessM)> Donors = new List<(Int32 Bx, Int32 By, Double ExcessM)>();
                List<(Int32 Bx, Int32 By, Double HoleM)> Holes = new List<(Int32 Bx, Int32 By, Double HoleM)>();
                for (Int32 By = 0; By < Grid.Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                    {
                        if (!Grid.Valid[By, Bx] || Surface.Excluded[By, Bx] || (Surface.PanM[By, Bx] > LIFT_EPSILON_M))
                        {
                            continue;
                        }

                        Double Off = Elevation[By, Bx] - Surface.FieldDesignM[By, Bx];
                        if (Math.Abs(Off) > (ToleranceM + FINISH_COMPARE_SLACK_M))
                        {
                            OffDesign = true;
                        }

                        if (Off > LIFT_EPSILON_M)
                        {
                            Donors.Add((Bx, By, Off));
                        }
                        else if (Off < -LIFT_EPSILON_M)
                        {
                            Holes.Add((Bx, By, -Off));
                        }
                    }
                }

                if (!OffDesign || (Donors.Count == 0) || (Holes.Count == 0))
                {
                    break;
                }

                Donors.Sort((Left, Right) => Right.ExcessM.CompareTo(Left.ExcessM));
                Holes.Sort((Left, Right) => Right.HoleM.CompareTo(Left.HoleM));
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                Double Room = Bowl;
                for (Int32 Index = 0; (Index < Donors.Count) && (Room > 1e-4); Index += 1)
                {
                    (Int32 Bx, Int32 By, Double ExcessM) Donor = Donors[Index];
                    Double Depth = Math.Min(Donor.ExcessM, Math.Min(MaxDepth, Room / Math.Max(CutPer, 1e-9)));
                    if (Depth <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    Cuts.Add(((Donor.Bx, Donor.By), Depth));
                    Elevation[Donor.By, Donor.Bx] -= Depth;
                    Room -= Depth * CutPer;
                }

                Double Need = Bowl - Room;
                for (Int32 Index = 0; (Index < Holes.Count) && (Need > 1e-4); Index += 1)
                {
                    (Int32 Bx, Int32 By, Double HoleM) Hole = Holes[Index];
                    Double Depth = Math.Min(Hole.HoleM, Math.Min(MaxDepth, Need / Math.Max(FillPer, 1e-9)));
                    if (Depth <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    Fills.Add(((Hole.Bx, Hole.By), Depth));
                    Elevation[Hole.By, Hole.Bx] += Depth;
                    Need -= Depth * FillPer;
                }

                Double Placed = (Bowl - Room) - Need;
                if ((Fills.Count == 0) || (Placed <= 1e-4))
                {
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cut in Cuts)
                    {
                        Elevation[Cut.Cell.By, Cut.Cell.Bx] += Cut.DepthM;
                    }

                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Fills)
                    {
                        Elevation[Fill.Cell.By, Fill.Cell.Bx] -= Fill.DepthM;
                    }

                    break;
                }

                // The fill may be short of the cut. Put the extra cut back, from the last cells.
                Double Spare = Need;
                while ((Spare > 1e-4) && (Cuts.Count > 0))
                {
                    Int32 Last = Cuts.Count - 1;
                    ((Int32 Bx, Int32 By) Cell, Double DepthM) Cut = Cuts[Last];
                    Double Bank = Cut.DepthM * CutPer;
                    if (Bank <= (Spare + 1e-9))
                    {
                        Elevation[Cut.Cell.By, Cut.Cell.Bx] += Cut.DepthM;
                        Cuts.RemoveAt(Last);
                        Spare -= Bank;
                    }
                    else
                    {
                        Double Give = Spare / Math.Max(CutPer, 1e-9);
                        Elevation[Cut.Cell.By, Cut.Cell.Bx] += Give;
                        Cuts[Last] = (Cut.Cell, Cut.DepthM - Give);
                        Spare = 0.0;
                    }
                }

                if (Cuts.Count == 0)
                {
                    foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Fills)
                    {
                        Elevation[Fill.Cell.By, Fill.Cell.Bx] -= Fill.DepthM;
                    }

                    break;
                }

                Double Load = 0.0;
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cut in Cuts)
                {
                    Load += Cut.DepthM * CutPer;
                    Ledger.AddRequiredCut(Cut.Cell.By, Cut.Cell.Bx, Cut.DepthM);
                }

                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Fills)
                {
                    Ledger.AddRequiredFill(Fill.Cell.By, Fill.Cell.Bx, Fill.DepthM);
                }

                (Double X, Double Y) CutStart = (Grid.CenterX[Cuts[0].Cell.By, Cuts[0].Cell.Bx], Grid.CenterY[Cuts[0].Cell.By, Cuts[0].Cell.Bx]);
                (Double X, Double Y) CutEnd = (Grid.CenterX[Cuts[Cuts.Count - 1].Cell.By, Cuts[Cuts.Count - 1].Cell.Bx], Grid.CenterY[Cuts[Cuts.Count - 1].Cell.By, Cuts[Cuts.Count - 1].Cell.Bx]);
                (Double X, Double Y) FillStart = (Grid.CenterX[Fills[0].Cell.By, Fills[0].Cell.Bx], Grid.CenterY[Fills[0].Cell.By, Fills[0].Cell.Bx]);
                (Double X, Double Y) FillEnd = (Grid.CenterX[Fills[Fills.Count - 1].Cell.By, Fills[Fills.Count - 1].Cell.Bx], Grid.CenterY[Fills[Fills.Count - 1].Cell.By, Fills[Fills.Count - 1].Cell.Bx]);
                Pass Trip = new Pass(Cuts, Fills, Load, Distance(CutEnd.X, CutEnd.Y, FillStart.X, FillStart.Y), CutStart, CutEnd, FillStart, FillEnd);
                Trip.Stage = HaulStage.Correcting;
                Trip.RibbonOrder = true;
                Passes.Add(Trip);
            }
        }

        /// <summary>
        /// Fails when a field cell finished off the design or a pan cell missed its pan target.
        /// Open ends and the well casing are not required to meet the pan.
        /// </summary>
        /// <param name="Survey">Existing ground before any pass.</param>
        /// <param name="Grid">Grid size, the valid mask, and the cell area.</param>
        /// <param name="Surface">Pan target, field design, open ends, casing, and the blade moves.</param>
        /// <param name="Passes">Every pass, including the road stage and correcting.</param>
        /// <param name="ToleranceMm">Finish tolerance in millimetres.</param>
        /// <exception cref="InvalidOperationException">A checked cell finished outside the tolerance.</exception>
        internal static void RequireFinish
            (
            Double[,] Survey,
            Grid Grid,
            RoadPadSurface Surface,
            IReadOnlyList<Pass> Passes,
            Double ToleranceMm
            )
        {
            Double ToleranceM = ToleranceMm / Constants.MILLIMETERS_PER_METER;
            Double[,] Elevation = (Double[,])Survey.Clone();
            ApplyPasses(Elevation, Passes);
            ApplyCells(Elevation, Surface.BladeCuts, -1.0);
            ApplyCells(Elevation, Surface.BladeFills, 1.0);
            Int32 HighCount = 0;
            Double WorstHigh = 0.0;
            Int32 WorstHighBx = 0;
            Int32 WorstHighBy = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx] || Surface.Excluded[By, Bx] || (Surface.PanM[By, Bx] > LIFT_EPSILON_M))
                    {
                        continue;
                    }

                    Double Off = Elevation[By, Bx] - Surface.FieldDesignM[By, Bx];
                    if (Math.Abs(Off) <= (ToleranceM + FINISH_COMPARE_SLACK_M))
                    {
                        continue;
                    }

                    HighCount += 1;
                    if (Math.Abs(Off) > Math.Abs(WorstHigh))
                    {
                        WorstHigh = Off;
                        WorstHighBx = Bx;
                        WorstHighBy = By;
                    }
                }
            }

            Surface.FieldCellsOffDesign = HighCount;
            if (HighCount > 0)
            {
                throw new InvalidOperationException(String.Format(
                    CultureInfo.InvariantCulture,
                    "A field cell is more than the finish tolerance from the design. {0} cells, worst {1:F1} mm at ({2}, {3}).",
                    HighCount,
                    WorstHigh * Constants.MILLIMETERS_PER_METER,
                    WorstHighBx,
                    WorstHighBy));
            }

            PanTargetMiss? Miss = MeasurePanTarget(
                Survey,
                Grid,
                Surface,
                Passes,
                new List<Pass>(),
                ToleranceMm);
            if (Miss != null)
            {
                throw new InvalidOperationException(Miss.FormatMessage());
            }
        }

        /// <summary>
        /// Fails when the reserve is not the pan deficit, or the road stage cut more than that deficit.
        /// </summary>
        /// <param name="Surface">Reserve size, deficit, and the cut the road stage took.</param>
        /// <exception cref="InvalidOperationException">The road balance does not close.</exception>
        internal static void RequireRoadBalance
            (
            RoadPadSurface Surface
            )
        {
            if (Math.Abs(Surface.PanImportBcy - Surface.RoadDeficitBcy) > BALANCE_CUSHION_BCY)
            {
                throw new InvalidOperationException(String.Format(
                    CultureInfo.InvariantCulture,
                    "Reserve size {0:F3} BCY is not the road deficit {1:F3} BCY.",
                    Surface.PanImportBcy,
                    Surface.RoadDeficitBcy));
            }

            if (Surface.ReserveUsedBcy > (Surface.ReserveAvailableBcy + BALANCE_CUSHION_BCY))
            {
                throw new InvalidOperationException(String.Format(
                    CultureInfo.InvariantCulture,
                    "Reserve used {0:F3} BCY is more than the reserve available {1:F3} BCY.",
                    Surface.ReserveUsedBcy,
                    Surface.ReserveAvailableBcy));
            }

            Double RoadCut = Surface.RoadCellCutBcy + Surface.FieldCutBcy;
            if (RoadCut > (Surface.DeficitRemainingBcy + BALANCE_CUSHION_BCY))
            {
                throw new InvalidOperationException(String.Format(
                    CultureInfo.InvariantCulture,
                    "Road-stage cut {0:F3} BCY is more than the road deficit remaining {1:F3} BCY.",
                    RoadCut,
                    Surface.DeficitRemainingBcy));
            }
        }

        /// <summary>
        /// Measures pan cells that finished outside the finish tolerance.
        /// The haul plan continues. Open ends and the well casing are not required to meet it.
        /// The survey is the ground before any pass.
        /// </summary>
        /// <param name="Survey">Existing ground before the main stages.</param>
        /// <param name="Grid">Grid size and the valid mask.</param>
        /// <param name="Surface">Pan target, open ends, casing, and the blade moves.</param>
        /// <param name="FieldPasses">Main-stage passes.</param>
        /// <param name="RoadPasses">Road-stage passes.</param>
        /// <param name="ToleranceMm">Finish tolerance in millimetres.</param>
        /// <returns>The miss, or null when every checked cell is inside the tolerance.</returns>
        internal static PanTargetMiss? MeasurePanTarget
            (
            Double[,] Survey,
            Grid Grid,
            RoadPadSurface Surface,
            IReadOnlyList<Pass> FieldPasses,
            IReadOnlyList<Pass> RoadPasses,
            Double ToleranceMm
            )
        {
            Double ToleranceM = ToleranceMm / Constants.MILLIMETERS_PER_METER;
            Double[,] Elevation = (Double[,])Survey.Clone();
            ApplyPasses(Elevation, FieldPasses);
            ApplyPasses(Elevation, RoadPasses);
            ApplyCells(Elevation, Surface.BladeCuts, -1.0);
            ApplyCells(Elevation, Surface.BladeFills, 1.0);
            // Above is bank cut still on the pan. Below is bank fill still missing, with the same compaction as the road stage.
            Double CutPer = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPer = CutPer * Grid.Compaction;
            List<PanTargetMiss.Cell> Cells = new List<PanTargetMiss.Cell>();
            Double AboveBcy = 0.0;
            Double BelowBcy = 0.0;
            Double Worst = 0.0;
            Double WorstSigned = 0.0;
            Int32 WorstBx = 0;
            Int32 WorstBy = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx] || (Surface.PanM[By, Bx] <= LIFT_EPSILON_M))
                    {
                        continue;
                    }

                    if (Surface.Excluded[By, Bx] || Surface.OpenEnd[By, Bx])
                    {
                        continue;
                    }

                    Double Target = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    // Positive means the finished ground is still high.
                    Double Signed = Elevation[By, Bx] - Target;
                    Double Error = Math.Abs(Signed);
                    if (Error <= (ToleranceM + FINISH_COMPARE_SLACK_M))
                    {
                        continue;
                    }

                    Cells.Add(new PanTargetMiss.Cell(Bx, By));
                    if (Signed > 0.0)
                    {
                        AboveBcy += Signed * CutPer;
                    }
                    else
                    {
                        BelowBcy += (-Signed) * FillPer;
                    }

                    if (Error > Worst)
                    {
                        Worst = Error;
                        WorstSigned = Signed;
                        WorstBx = Bx;
                        WorstBy = By;
                    }
                }
            }

            if (Cells.Count == 0)
            {
                return null;
            }

            return new PanTargetMiss(
                WorstBx,
                WorstBy,
                WorstSigned * Constants.MILLIMETERS_PER_METER,
                AboveBcy,
                BelowBcy,
                Cells);
        }

        /// <summary>
        /// Copies the survey back onto the grid after the road stage has read the worked ground.
        /// </summary>
        /// <param name="Grid">Grid whose existing ground is replaced.</param>
        /// <param name="Survey">Elevations taken before the main stages were applied.</param>
        internal static void RestoreSurvey
            (
            Grid Grid,
            Double[,] Survey
            )
        {
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Grid.Existing[By, Bx] = Survey[By, Bx];
                }
            }
        }

        /// <summary>
        /// Applies cut and fill from passes. Cut lowers the ground. Fill raises it.
        /// </summary>
        /// <param name="Elevation">Ground to update.</param>
        /// <param name="Passes">Passes to apply. Null is ignored.</param>
        private static void ApplyPasses
            (
            Double[,] Elevation,
            IReadOnlyList<Pass> Passes
            )
        {
            if (Passes == null)
            {
                return;
            }

            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                Pass Trip = Passes[Index];
                ApplyCells(Elevation, Trip.CutCells, -1.0);
                ApplyCells(Elevation, Trip.FillCells, 1.0);
            }
        }

        /// <summary>
        /// Adds depth onto a grid. Null cells are ignored.
        /// </summary>
        /// <param name="Depth">Depth grid to update.</param>
        /// <param name="Cells">Cells and depths.</param>
        private static void AddDepths
            (
            Double[,] Depth,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            if (Cells == null)
            {
                return;
            }

            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Cells[Index];
                Depth[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
            }
        }

        /// <summary>
        /// Adds a signed depth on each cell.
        /// </summary>
        /// <param name="Elevation">Ground to update.</param>
        /// <param name="Cells">Cells and depths.</param>
        /// <param name="Sign">-1 for cut, +1 for fill.</param>
        private static void ApplyCells
            (
            Double[,] Elevation,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            Double Sign
            )
        {
            if (Cells == null)
            {
                return;
            }

            for (Int32 Index = 0; Index < Cells.Count; Index += 1)
            {
                ((Int32 Bx, Int32 By) Cell, Double DepthM) Entry = Cells[Index];
                Elevation[Entry.Cell.By, Entry.Cell.Bx] += Sign * Entry.DepthM;
            }
        }

        /// <summary>
        /// Holds the pan deficit back as depth on blade-width lanes.
        /// A lane runs with the local cut, is at least four turning radii long, and an isolated cell is never reserved.
        /// Each cell gives at most <see cref="HaulBuilderOptions.ReserveMaxDepthFt"/> and keeps enough cut to stay a flow source.
        /// </summary>
        /// <param name="Grid">Existing elevations.</param>
        /// <param name="Surface">Field design that receives the reserved depth and the lanes.</param>
        /// <param name="Options">The depth cap and the turning circle.</param>
        private static void ReserveCut
            (
            Grid Grid,
            RoadPadSurface Surface,
            HaulBuilderOptions Options
            )
        {
            Surface.ReserveLanes.Clear();
            if (Surface.PanImportBcy <= 1e-6)
            {
                return;
            }

            List<(Int32 Bx, Int32 By)> Deficits = new List<(Int32 Bx, Int32 By)>();
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Surface.DeficitM[By, Bx] > LIFT_EPSILON_M)
                    {
                        Deficits.Add((Bx, By));
                    }
                }
            }

            Double[,] DepthM = new Double[Grid.Height, Grid.Width];
            Double[,] DistanceM = new Double[Grid.Height, Grid.Width];
            List<(Int32 Bx, Int32 By)> Seeds = new List<(Int32 Bx, Int32 By)>();
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx] || Surface.Excluded[By, Bx] || (Surface.PanM[By, Bx] > LIFT_EPSILON_M))
                    {
                        // The footprint is not borrow. Field cells outside it are.
                        continue;
                    }

                    Double Depth = Grid.Existing[By, Bx] - Surface.FieldDesignM[By, Bx];
                    if (Depth <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    DepthM[By, Bx] = Depth;
                    DistanceM[By, Bx] = NearestDeficitM(Grid, Deficits, Bx, By);
                    Seeds.Add((Bx, By));
                }
            }

            Seeds.Sort((Left, Right) =>
            {
                Int32 ByDistance = DistanceM[Left.By, Left.Bx].CompareTo(DistanceM[Right.By, Right.Bx]);
                if (ByDistance != 0)
                {
                    return ByDistance;
                }

                Int32 ByRow = Left.By.CompareTo(Right.By);
                return (ByRow != 0) ? ByRow : Left.Bx.CompareTo(Right.Bx);
            });
            (Double[,] Dx, Double[,] Dy) Heading = CutHeadings(Grid, DepthM, Deficits);
            Double BladeM = Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER;
            Double HalfM = Math.Max(BladeM, Grid.GridSizeM) * Constants.HALF_SCALE * 1.01;
            Double MinLengthM = RESERVE_LANE_RADII * Options.MinimumTurningDiameterFt * Constants.FEET_TO_METERS * Constants.HALF_SCALE;
            Boolean[,] Used = new Boolean[Grid.Height, Grid.Width];
            // Cut-direction lanes are claimed first. The long axis is only for cells those lanes cannot cover.
            for (Int32 Pass = 0; Pass < 2; Pass += 1)
            {
                foreach ((Int32 Bx, Int32 By) Seed in Seeds)
                {
                    if (Used[Seed.By, Seed.Bx])
                    {
                        continue;
                    }

                    // Snap onto the raster so a straight lane stays on cell centres.
                    Double DirX = Heading.Dx[Seed.By, Seed.Bx];
                    Double DirY = Heading.Dy[Seed.By, Seed.Bx];
                    SnapLaneHeading(ref DirX, ref DirY);
                    if (Pass == 0)
                    {
                        TryTakeLane(Grid, Surface, DepthM, DistanceM, Used, Seed.Bx, Seed.By, DirX, DirY, HalfM, MinLengthM);
                    }
                    else
                    {
                        TryTakeLane(Grid, Surface, DepthM, DistanceM, Used, Seed.Bx, Seed.By, -DirY, DirX, HalfM, MinLengthM);
                    }
                }
            }

            Surface.ReserveLanes.Sort((Left, Right) =>
            {
                Int32 ByDistance = Left.DistanceM.CompareTo(Right.DistanceM);
                if (ByDistance != 0)
                {
                    return ByDistance;
                }

                return Left.Cells[0].By.CompareTo(Right.Cells[0].By);
            });
            // A full-depth hold would drop the target onto the existing ground and erase the arrow.
            // The kept sliver is taken only when the capped layer cannot cover the borrow.
            Double CapM = Options.ReserveMaxDepthFt * Constants.FEET_TO_METERS;
            Double BankPerM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double Need = Surface.PanImportBcy;
            Need = ApplyLaneDepth(Surface, DepthM, BankPerM, CapM, Need, true);
            if (Need > 1e-6)
            {
                Need = ApplyLaneDepth(Surface, DepthM, BankPerM, CapM, Need, false);
            }

            if (Need > 0.05)
            {
                Surface.Warnings.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Reserved cut is short by {0:F1} BCY.",
                    Need));
            }
        }

        /// <summary>
        /// Downhill of the existing ground, smoothed so a lane has one heading.
        /// Flat ground points at the nearest pan deficit.
        /// </summary>
        /// <param name="Grid">Existing elevations.</param>
        /// <param name="DepthM">Cut depth. Zero off the borrow pool.</param>
        /// <param name="Deficits">Pan cells below the pan target.</param>
        /// <returns>Unit east and north headings. Zero where the cell is not cut.</returns>
        private static (Double[,] Dx, Double[,] Dy) CutHeadings
            (
            Grid Grid,
            Double[,] DepthM,
            List<(Int32 Bx, Int32 By)> Deficits
            )
        {
            Double[,] Dx = new Double[Grid.Height, Grid.Width];
            Double[,] Dy = new Double[Grid.Height, Grid.Width];
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (DepthM[By, Bx] <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    Double East = SlopeAcross(Grid, Bx, By, 1, 0);
                    Double North = SlopeAcross(Grid, Bx, By, 0, 1);
                    Double Gx = -East;
                    Double Gy = -North;
                    Double Magnitude = Math.Sqrt((Gx * Gx) + (Gy * Gy));
                    if (Magnitude < FLAT_SLOPE)
                    {
                        // No downhill. The haul toward the deficit is the cut direction.
                        AimAtDeficit(Grid, Deficits, Bx, By, out Gx, out Gy);
                        Magnitude = Math.Sqrt((Gx * Gx) + (Gy * Gy));
                    }

                    if (Magnitude > 1e-9)
                    {
                        Dx[By, Bx] = Gx / Magnitude;
                        Dy[By, Bx] = Gy / Magnitude;
                    }
                }
            }

            Double[,] Sx = (Double[,])Dx.Clone();
            Double[,] Sy = (Double[,])Dy.Clone();
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (DepthM[By, Bx] <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    Double SumX = Dx[By, Bx];
                    Double SumY = Dy[By, Bx];
                    for (Int32 StepY = -1; StepY <= 1; StepY += 1)
                    {
                        for (Int32 StepX = -1; StepX <= 1; StepX += 1)
                        {
                            Int32 Nx = Bx + StepX;
                            Int32 Ny = By + StepY;
                            if ((Nx < 0) || (Ny < 0) || (Nx >= Grid.Width) || (Ny >= Grid.Height))
                            {
                                continue;
                            }

                            if (DepthM[Ny, Nx] <= LIFT_EPSILON_M)
                            {
                                continue;
                            }

                            SumX += Dx[Ny, Nx];
                            SumY += Dy[Ny, Nx];
                        }
                    }

                    Double Magnitude = Math.Sqrt((SumX * SumX) + (SumY * SumY));
                    if (Magnitude > 1e-9)
                    {
                        Sx[By, Bx] = SumX / Magnitude;
                        Sy[By, Bx] = SumY / Magnitude;
                    }
                }
            }

            return (Sx, Sy);
        }

        /// <summary>
        /// Snaps a heading to the nearest eighth of a circle so a lane follows cell centres.
        /// </summary>
        /// <param name="DirX">East component. Replaced with the snapped heading.</param>
        /// <param name="DirY">North component. Replaced with the snapped heading.</param>
        private static void SnapLaneHeading
            (
            ref Double DirX,
            ref Double DirY
            )
        {
            Double Magnitude = Math.Sqrt((DirX * DirX) + (DirY * DirY));
            if (Magnitude < 1e-9)
            {
                return;
            }

            // Eight headings: the axis and the diagonal. A finer split leaves the cell centres off the strip.
            Double Angle = Math.Atan2(DirY, DirX);
            Double Snapped = Math.Round(Angle / (Math.PI / 4.0)) * (Math.PI / 4.0);
            DirX = Math.Cos(Snapped);
            DirY = Math.Sin(Snapped);
        }

        /// <summary>
        /// Rise of the existing ground over run, in one grid step.
        /// </summary>
        /// <param name="Grid">Existing elevations.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="StepX">Column step, -1, 0, or 1.</param>
        /// <param name="StepY">Row step, -1, 0, or 1.</param>
        /// <returns>Slope in metres per metre. Zero when a neighbour is missing.</returns>
        private static Double SlopeAcross
            (
            Grid Grid,
            Int32 Bx,
            Int32 By,
            Int32 StepX,
            Int32 StepY
            )
        {
            Int32 AheadX = Bx + StepX;
            Int32 AheadY = By + StepY;
            Int32 BehindX = Bx - StepX;
            Int32 BehindY = By - StepY;
            Boolean Ahead = (AheadX >= 0) && (AheadY >= 0) && (AheadX < Grid.Width) && (AheadY < Grid.Height) && Grid.Valid[AheadY, AheadX];
            Boolean Behind = (BehindX >= 0) && (BehindY >= 0) && (BehindX < Grid.Width) && (BehindY < Grid.Height) && Grid.Valid[BehindY, BehindX];
            if (Ahead && Behind)
            {
                return (Grid.Existing[AheadY, AheadX] - Grid.Existing[BehindY, BehindX]) / (2.0 * Grid.GridSizeM);
            }

            if (Ahead)
            {
                return (Grid.Existing[AheadY, AheadX] - Grid.Existing[By, Bx]) / Grid.GridSizeM;
            }

            if (Behind)
            {
                return (Grid.Existing[By, Bx] - Grid.Existing[BehindY, BehindX]) / Grid.GridSizeM;
            }

            return 0.0;
        }

        /// <summary>
        /// Unit vector from a cell toward the nearest pan deficit.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Deficits">Pan cells below the pan target.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="DirX">East component. Zero when there is no deficit.</param>
        /// <param name="DirY">North component. Zero when there is no deficit.</param>
        private static void AimAtDeficit
            (
            Grid Grid,
            List<(Int32 Bx, Int32 By)> Deficits,
            Int32 Bx,
            Int32 By,
            out Double DirX,
            out Double DirY
            )
        {
            DirX = 1.0;
            DirY = 0.0;
            Double Best = Double.PositiveInfinity;
            Double X = Grid.CenterX[By, Bx];
            Double Y = Grid.CenterY[By, Bx];
            for (Int32 Index = 0; Index < Deficits.Count; Index += 1)
            {
                (Int32 Bx, Int32 By) Cell = Deficits[Index];
                Double Dx = Grid.CenterX[Cell.By, Cell.Bx] - X;
                Double Dy = Grid.CenterY[Cell.By, Cell.Bx] - Y;
                Double Dist = (Dx * Dx) + (Dy * Dy);
                if (Dist < Best)
                {
                    Best = Dist;
                    DirX = Dx;
                    DirY = Dy;
                }
            }
        }

        /// <summary>
        /// Accepts one blade-width lane through a seed when the run is long enough.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Surface">Receives the lane.</param>
        /// <param name="DepthM">Cut depth. Zero off the borrow pool.</param>
        /// <param name="DistanceM">Distance to the nearest pan deficit.</param>
        /// <param name="Used">Cells already in a lane. The accepted run is marked.</param>
        /// <param name="Bx">Seed column.</param>
        /// <param name="By">Seed row.</param>
        /// <param name="DirX">Lane direction east.</param>
        /// <param name="DirY">Lane direction north.</param>
        /// <param name="HalfM">Half the lane width, in metres.</param>
        /// <param name="MinLengthM">Shortest lane, in metres.</param>
        /// <returns>False when this heading has no lane of the required length.</returns>
        private static Boolean TryTakeLane
            (
            Grid Grid,
            RoadPadSurface Surface,
            Double[,] DepthM,
            Double[,] DistanceM,
            Boolean[,] Used,
            Int32 Bx,
            Int32 By,
            Double DirX,
            Double DirY,
            Double HalfM,
            Double MinLengthM
            )
        {
            Double Magnitude = Math.Sqrt((DirX * DirX) + (DirY * DirY));
            if (Magnitude < 1e-9)
            {
                return false;
            }

            DirX /= Magnitude;
            DirY /= Magnitude;
            Double OriginX = Grid.CenterX[By, Bx];
            Double OriginY = Grid.CenterY[By, Bx];
            List<(Int32 Bx, Int32 By, Double Along)> Corridor = new List<(Int32 Bx, Int32 By, Double Along)>();
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    if (Used[Row, Col] || (DepthM[Row, Col] <= LIFT_EPSILON_M))
                    {
                        continue;
                    }

                    Double RelX = Grid.CenterX[Row, Col] - OriginX;
                    Double RelY = Grid.CenterY[Row, Col] - OriginY;
                    Double Perp = (RelX * -DirY) + (RelY * DirX);
                    if (Math.Abs(Perp) > HalfM)
                    {
                        continue;
                    }

                    Double Along = (RelX * DirX) + (RelY * DirY);
                    Corridor.Add((Col, Row, Along));
                }
            }

            Corridor.Sort((Left, Right) => Left.Along.CompareTo(Right.Along));
            Int32 Start = 0;
            while (Start < Corridor.Count)
            {
                Int32 End = Start;
                while ((End + 1) < Corridor.Count)
                {
                    Double Gap = Corridor[End + 1].Along - Corridor[End].Along;
                    if (Gap > (Grid.GridSizeM * 1.75))
                    {
                        break;
                    }

                    End += 1;
                }

                Boolean HoldsSeed = false;
                for (Int32 Index = Start; Index <= End; Index += 1)
                {
                    if ((Corridor[Index].Bx == Bx) && (Corridor[Index].By == By))
                    {
                        HoldsSeed = true;
                        break;
                    }
                }

                Double LengthM = (Corridor[End].Along - Corridor[Start].Along) + Grid.GridSizeM;
                if (HoldsSeed && (LengthM + 1e-6 >= MinLengthM))
                {
                    List<RoadPadSurface.ReserveCell> Cells = new List<RoadPadSurface.ReserveCell>();
                    Double Nearest = Double.PositiveInfinity;
                    for (Int32 Index = Start; Index <= End; Index += 1)
                    {
                        (Int32 CellX, Int32 CellY, Double Along) Cell = Corridor[Index];
                        Used[Cell.CellY, Cell.CellX] = true;
                        Double Distance = DistanceM[Cell.CellY, Cell.CellX];
                        Nearest = Math.Min(Nearest, Distance);
                        Cells.Add(new RoadPadSurface.ReserveCell(Cell.CellX, Cell.CellY, Distance));
                    }

                    if ((Cells.Count > 1) && (Cells[Cells.Count - 1].DistanceM < Cells[0].DistanceM))
                    {
                        // The deficit end is the start of the strip, so the nearest soil is cut first.
                        Cells.Reverse();
                    }

                    Surface.ReserveLanes.Add(new RoadPadSurface.ReserveLane(DirX, DirY, LengthM, Nearest, Cells));
                    return true;
                }

                Start = End + 1;
            }

            return false;
        }

        /// <summary>
        /// Puts one depth on every cell of the nearest lanes until the borrow is covered.
        /// A lane is taken whole, so the reserved part stays a strip rather than a single cell.
        /// </summary>
        /// <param name="Surface">Lanes and the reserved depth.</param>
        /// <param name="DepthM">Cut available before this reserve, in metres.</param>
        /// <param name="BankPerM">Bank cubic yards of one metre of cut on one cell.</param>
        /// <param name="CapM">Deepest reserve on one cell, in metres.</param>
        /// <param name="Need">Bank cubic yards still to reserve.</param>
        /// <param name="KeepSource">True to leave enough cut that the cell stays a flow source.</param>
        /// <returns>Bank cubic yards still short.</returns>
        private static Double ApplyLaneDepth
            (
            RoadPadSurface Surface,
            Double[,] DepthM,
            Double BankPerM,
            Double CapM,
            Double Need,
            Boolean KeepSource
            )
        {
            foreach (RoadPadSurface.ReserveLane Lane in Surface.ReserveLanes)
            {
                if (Need <= 1e-6)
                {
                    break;
                }

                Double Bank = 0.0;
                foreach (RoadPadSurface.ReserveCell Cell in Lane.Cells)
                {
                    Double Room = LaneRoom(Surface, DepthM, Cell, CapM, KeepSource);
                    if (Room > LIFT_EPSILON_M)
                    {
                        Bank += Room * BankPerM;
                    }
                }

                if (Bank <= 1e-9)
                {
                    continue;
                }

                // The whole lane takes the same share, down to whatever this lane can still give.
                Double Fraction = Math.Min(1.0, Need / Bank);
                foreach (RoadPadSurface.ReserveCell Cell in Lane.Cells)
                {
                    Double Room = LaneRoom(Surface, DepthM, Cell, CapM, KeepSource);
                    Double Take = Fraction * Room;
                    if (Take <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    Surface.ReservedDepthM[Cell.By, Cell.Bx] += Take;
                    Need -= Take * BankPerM;
                }
            }

            return Need;
        }

        /// <summary>
        /// Depth one lane cell can still give under the cap.
        /// </summary>
        /// <param name="Surface">Depth already reserved.</param>
        /// <param name="DepthM">Cut available before the reserve.</param>
        /// <param name="Cell">Lane cell.</param>
        /// <param name="CapM">Deepest reserve on one cell, in metres.</param>
        /// <param name="KeepSource">True to leave a millimetre so the flow arrow stays.</param>
        /// <returns>Metres still available. Zero when the cell is at the cap.</returns>
        private static Double LaneRoom
            (
            RoadPadSurface Surface,
            Double[,] DepthM,
            RoadPadSurface.ReserveCell Cell,
            Double CapM,
            Boolean KeepSource
            )
        {
            Double Already = Surface.ReservedDepthM[Cell.By, Cell.Bx];
            Double Room = DepthM[Cell.By, Cell.Bx] - Already;
            if (KeepSource)
            {
                Room -= FLOW_SOURCE_KEEP_M;
            }

            Room = Math.Min(Room, CapM - Already);
            if (Room <= LIFT_EPSILON_M)
            {
                return 0.0;
            }

            return Room;
        }

        /// <summary>
        /// Copies the reserved depth onto the lane cells the trip planner consumes.
        /// Accounting after the main stages may have reduced that depth.
        /// </summary>
        /// <param name="Surface">Lanes and the reserved depth.</param>
        private static void RefreshReserveLeft
            (
            RoadPadSurface Surface
            )
        {
            foreach (RoadPadSurface.ReserveLane Lane in Surface.ReserveLanes)
            {
                foreach (RoadPadSurface.ReserveCell Cell in Lane.Cells)
                {
                    Cell.LeftM = Surface.ReservedDepthM[Cell.By, Cell.Bx];
                }
            }
        }

        /// <summary>
        /// Reserved depth the road stage has not yet cut, in metres.
        /// </summary>
        /// <param name="Surface">Lane cells.</param>
        /// <returns>The deepest cell still holding reserve. Zero when the reserve is gone.</returns>
        private static Double ReserveLeft
            (
            RoadPadSurface Surface
            )
        {
            Double Left = 0.0;
            foreach (RoadPadSurface.ReserveLane Lane in Surface.ReserveLanes)
            {
                foreach (RoadPadSurface.ReserveCell Cell in Lane.Cells)
                {
                    if (Cell.LeftM > Left)
                    {
                        Left = Cell.LeftM;
                    }
                }
            }

            return Left;
        }

        /// <summary>
        /// Samples the fitted field design at a point.
        /// </summary>
        /// <param name="Grid">Grid geometry.</param>
        /// <param name="Surface">Field design.</param>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <returns>Elevation in metres, or NaN outside the field.</returns>
        private static Double SampleDesign
            (
            Grid Grid,
            RoadPadSurface Surface,
            Double X,
            Double Y
            )
        {
            Double Column = ((X - Grid.MinX) / Grid.GridSizeM) - Constants.HALF_SCALE;
            Double Row = ((Y - Grid.MinY) / Grid.GridSizeM) - Constants.HALF_SCALE;
            Int32 C0 = (Int32)Math.Floor(Column);
            Int32 R0 = (Int32)Math.Floor(Row);
            Double Tc = Column - C0;
            Double Tr = Row - R0;
            Double Z00 = DesignAt(Grid, Surface, C0, R0);
            Double Z10 = DesignAt(Grid, Surface, C0 + 1, R0);
            Double Z01 = DesignAt(Grid, Surface, C0, R0 + 1);
            Double Z11 = DesignAt(Grid, Surface, C0 + 1, R0 + 1);
            if (Double.IsFinite(Z00) && Double.IsFinite(Z10) && Double.IsFinite(Z01) && Double.IsFinite(Z11))
            {
                Double Top = (Z00 * (1.0 - Tc)) + (Z10 * Tc);
                Double Bottom = (Z01 * (1.0 - Tc)) + (Z11 * Tc);
                return (Top * (1.0 - Tr)) + (Bottom * Tr);
            }

            Int32 NearC = Math.Clamp((Int32)Math.Round(Column), 0, Grid.Width - 1);
            Int32 NearR = Math.Clamp((Int32)Math.Round(Row), 0, Grid.Height - 1);
            return DesignAt(Grid, Surface, NearC, NearR);
        }

        /// <summary>
        /// Reads one field-design cell, or NaN when it is outside the grid.
        /// </summary>
        /// <param name="Grid">Grid size.</param>
        /// <param name="Surface">Field design.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <returns>Elevation, or NaN.</returns>
        private static Double DesignAt
            (
            Grid Grid,
            RoadPadSurface Surface,
            Int32 Bx,
            Int32 By
            )
        {
            if ((Bx < 0) || (By < 0) || (Bx >= Grid.Width) || (By >= Grid.Height) || !Grid.Valid[By, Bx])
            {
                return Double.NaN;
            }

            return Surface.FieldDesignM[By, Bx];
        }

        /// <summary>
        /// Builds one road-stage cut pool. The reserve pool is field cells. The surplus pool is pan trim.
        /// The reserve is planned first so the road does not spend the surplus and leave the reserve behind.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Surface">Reserved depth and road-stage trim.</param>
        /// <param name="Features">Distance is measured to the nearest one.</param>
        /// <param name="ReservedLayer">True for the reserved field layer. False for leftover pan surplus.</param>
        /// <returns>Cut cells with remaining depth, nearest first.</returns>
        private static List<Stock> BuildStocks
            (
            Grid Grid,
            RoadPadSurface Surface,
            List<Feature> Features,
            Boolean ReservedLayer
            )
        {
            List<Stock> Stocks = new List<Stock>();
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Double Depth = ReservedLayer
                        ? Surface.ReservedDepthM[By, Bx]
                        : Surface.StageCutM[By, Bx];
                    if (Depth <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    Stocks.Add(new Stock(Bx, By, Depth, NearestFeatureM(Grid, Features, Bx, By)));
                }
            }

            Stocks.Sort((Left, Right) => Left.DistanceM.CompareTo(Right.DistanceM));
            return Stocks;
        }

        /// <summary>
        /// Lane order: centre, then the right lane, then the left, then outward.
        /// </summary>
        /// <param name="Left">First lane index.</param>
        /// <param name="Right">Second lane index.</param>
        /// <returns>Sort order.</returns>
        private static Int32 CompareLanes
            (
            Int32 Left,
            Int32 Right
            )
        {
            Int32 LeftRank = (Left == 0) ? 0 : ((Math.Abs(Left) * 2) - (Left > 0 ? 1 : 0));
            Int32 RightRank = (Right == 0) ? 0 : ((Math.Abs(Right) * 2) - (Right > 0 ? 1 : 0));
            return LeftRank.CompareTo(RightRank);
        }

        /// <summary>
        /// Whole lane index. A positive offset is left of the direction of travel.
        /// </summary>
        /// <param name="SignedM">Signed offset in metres.</param>
        /// <param name="LaneM">Lane width in metres.</param>
        /// <returns>Lane index.</returns>
        private static Int32 LaneIndex
            (
            Double SignedM,
            Double LaneM
            )
        {
            if (LaneM <= 0.0)
            {
                return 0;
            }

            return (Int32)Math.Round(SignedM / LaneM, MidpointRounding.AwayFromZero);
        }

        /// <summary>
        /// Mean load as a fraction of the bowl. Zero when there are no trips.
        /// </summary>
        /// <param name="Passes">Trips to average.</param>
        /// <param name="BowlBcy">Scraper capacity in BCY.</param>
        /// <returns>The fraction, from 0 to about 1.</returns>
        private static Double MeanLoadFraction
            (
            List<Pass> Passes,
            Double BowlBcy
            )
        {
            if ((Passes.Count == 0) || (BowlBcy <= 0.0))
            {
                return 0.0;
            }

            Double Sum = 0.0;
            foreach (Pass Pass in Passes)
            {
                Sum += Pass.LoadBcy / BowlBcy;
            }

            return Sum / Passes.Count;
        }

        /// <summary>
        /// Records haul count, mean bowl load, pan volume, mean loaded carry, and road hours per BCY.
        /// Blade volume is already on the surface.
        /// </summary>
        /// <param name="Surface">Surface that receives the figures.</param>
        /// <param name="Passes">Road-stage pan trips.</param>
        /// <param name="BowlBcy">Scraper capacity in BCY.</param>
        private static void WriteStageDiagnostics
            (
            RoadPadSurface Surface,
            List<Pass> Passes,
            Double BowlBcy
            )
        {
            Surface.RoadTripCount = Passes.Count;
            Surface.MeanLoadFraction = MeanLoadFraction(Passes, BowlBcy);
            Double Pan = 0.0;
            Double Carry = 0.0;
            foreach (Pass Pass in Passes)
            {
                Pan += Pass.LoadBcy;
                Carry += Pass.HaulDistanceM;
            }

            Surface.PanHaulBcy = Pan;

            Surface.MeanLoadedCarryFt = (Passes.Count == 0)
                ? 0.0
                : (Carry / Passes.Count) / Constants.FEET_TO_METERS;
            Surface.RoadHoursPerBcy = (Surface.RoadFillBcy > 1e-6)
                ? Surface.RoadStageHours / Surface.RoadFillBcy
                : 0.0;
        }

        /// <summary>
        /// True when any lane cell still has fill.
        /// </summary>
        /// <param name="Lanes">Lanes to scan.</param>
        /// <returns>True when a trip can still spread.</returns>
        private static Boolean HasFill
            (
            List<List<LaneCell>> Lanes
            )
        {
            foreach (List<LaneCell> Cells in Lanes)
            {
                foreach (LaneCell Cell in Cells)
                {
                    if (Cell.LeftM > LIFT_EPSILON_M)
                    {
                        return true;
                    }
                }
            }

            return false;
        }
        /// <summary>
        /// Adds one fill lift, combining depth when the cell is already on the trip.
        /// </summary>
        /// <param name="Fills">Fill cells of this trip.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <param name="DepthM">Depth of this lift, in metres.</param>
        private static void AddFill
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills,
            Int32 Bx,
            Int32 By,
            Double DepthM
            )
        {
            for (Int32 Index = 0; Index < Fills.Count; Index += 1)
            {
                if ((Fills[Index].Cell.Bx == Bx) && (Fills[Index].Cell.By == By))
                {
                    Fills[Index] = (Fills[Index].Cell, Fills[Index].DepthM + DepthM);
                    return;
                }
            }

            Fills.Add(((Bx, By), DepthM));
        }
      /// <summary>
        /// Next remaining cell on one lane, at or ahead of the cursor.
        /// </summary>
        /// <param name="Cells">Lane cells in station order.</param>
        /// <param name="Cursor">Station already reached.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="Have">True after a cell has been taken, so that cell is skipped.</param>
        /// <param name="Index">Index of the cell.</param>
        /// <returns>False when this lane has nothing ahead.</returns>
        private static Boolean TrySameLane
            (
            List<LaneCell> Cells,
            Double Cursor,
            Int32 Sign,
            Boolean Have,
            out Int32 Index
            )
        {
            Index = -1;
            // Lanes are sorted by station. The next cell is the first one still ahead that has depth.
            Double MinAhead = Have ? (1e-4 + 1e-8) : -1e-4;
            return FirstWithDepth(Cells, Cursor, Sign, MinAhead, null, out Index);
        }

        /// <summary>
        /// Closest cell at or beyond a forward distance. Empty cells are skipped.
        /// </summary>
        /// <param name="Cells">Lane cells sorted by station.</param>
        /// <param name="Cursor">Station already reached.</param>
        /// <param name="Sign">+1 follows increasing station.</param>
        /// <param name="MinAhead">Minimum forward distance in metres.</param>
        /// <param name="Skip">Cells to ignore. Null when every cell with depth is usable.</param>
        /// <param name="Index">Index of the cell.</param>
        /// <returns>False when nothing qualifies.</returns>
        private static Boolean FirstWithDepth
            (
            List<LaneCell> Cells,
            Double Cursor,
            Int32 Sign,
            Double MinAhead,
            HashSet<(Int32 Bx, Int32 By)> Skip,
            out Int32 Index
            )
        {
            Index = -1;
            if (Cells.Count == 0)
            {
                return false;
            }

            if (Sign > 0)
            {
                Double Need = Cursor + MinAhead;
                Int32 Low = 0;
                Int32 High = Cells.Count;
                while (Low < High)
                {
                    Int32 Mid = (Low + High) / 2;
                    if (Cells[Mid].Order < Need)
                    {
                        Low = Mid + 1;
                    }
                    else
                    {
                        High = Mid;
                    }
                }

                for (Int32 Cell = Low; Cell < Cells.Count; Cell += 1)
                {
                    if (Cells[Cell].LeftM <= LIFT_EPSILON_M)
                    {
                        continue;
                    }

                    if ((Skip != null) && Skip.Contains((Cells[Cell].Bx, Cells[Cell].By)))
                    {
                        continue;
                    }

                    Index = Cell;
                    return true;
                }

                return false;
            }

            Double Limit = Cursor - MinAhead;
            Int32 Start = 0;
            Int32 End = Cells.Count;
            while (Start < End)
            {
                Int32 Mid = (Start + End) / 2;
                if (Cells[Mid].Order <= Limit)
                {
                    Start = Mid + 1;
                }
                else
                {
                    End = Mid;
                }
            }

            for (Int32 Cell = Start - 1; Cell >= 0; Cell -= 1)
            {
                if (Cells[Cell].LeftM <= LIFT_EPSILON_M)
                {
                    continue;
                }

                if ((Skip != null) && Skip.Contains((Cells[Cell].Bx, Cells[Cell].By)))
                {
                    continue;
                }

                Index = Cell;
                return true;
            }

            return false;
        }

        /// <summary>
        /// Station of a cell along the road or pad.
        /// </summary>
        /// <param name="Grid">Cell centres.</param>
        /// <param name="Item">Road or pad.</param>
        /// <param name="Bx">Column.</param>
        /// <param name="By">Row.</param>
        /// <returns>Metres along the feature.</returns>
        private static Double StationOf
            (
            Grid Grid,
            Feature Item,
            Int32 Bx,
            Int32 By
            )
        {
            Double X = Grid.CenterX[By, Bx];
            Double Y = Grid.CenterY[By, Bx];
            if (Item.IsPad)
            {
                return ((X - Item.X) * Item.DirX) + ((Y - Item.Y) * Item.DirY);
            }

            return Item.Closest(X, Y).Station;
        }

        /// <summary>
        /// Copies remaining fill so a failed direction can be put back.
        /// </summary>
        /// <param name="Lanes">Lanes to copy.</param>
        /// <returns>Depth of every cell, lane by lane.</returns>
        private static Double[] SaveFill
            (
            List<List<LaneCell>> Lanes
            )
        {
            Int32 Count = 0;
            foreach (List<LaneCell> Cells in Lanes)
            {
                Count += Cells.Count;
            }

            Double[] Saved = new Double[Count];
            Int32 Index = 0;
            foreach (List<LaneCell> Cells in Lanes)
            {
                foreach (LaneCell Cell in Cells)
                {
                    Saved[Index] = Cell.LeftM;
                    Index += 1;
                }
            }

            return Saved;
        }

        /// <summary>
        /// Puts fill depths back after a direction that found no cut.
        /// </summary>
        /// <param name="Lanes">Lanes to restore.</param>
        /// <param name="Saved">Depths from <see cref="SaveFill"/>.</param>
        private static void RestoreFill
            (
            List<List<LaneCell>> Lanes,
            Double[] Saved
            )
        {
            Int32 Index = 0;
            foreach (List<LaneCell> Cells in Lanes)
            {
                foreach (LaneCell Cell in Cells)
                {
                    Cell.LeftM = Saved[Index];
                    Index += 1;
                }
            }
        }

        /// <summary>
        /// Copies remaining cut so a failed direction can be put back.
        /// </summary>
        /// <param name="Stocks">Cut cells.</param>
        /// <returns>Remaining depth of each stock.</returns>
        private static Double[] SaveCut
            (
            List<Stock> Stocks
            )
        {
            Double[] Saved = new Double[Stocks.Count];
            for (Int32 Index = 0; Index < Stocks.Count; Index += 1)
            {
                Saved[Index] = Stocks[Index].LeftM;
            }

            return Saved;
        }

        /// <summary>
        /// Puts cut depths back after a direction that found no cut.
        /// </summary>
        /// <param name="Stocks">Cut cells.</param>
        /// <param name="Saved">Depths from <see cref="SaveCut"/>.</param>
        private static void RestoreCut
            (
            List<Stock> Stocks,
            Double[] Saved
            )
        {
            for (Int32 Index = 0; Index < Stocks.Count; Index += 1)
            {
                Stocks[Index].LeftM = Saved[Index];
            }
        }

        /// <summary>
        /// Runs every road trip through guidance and stores the hours and the courses.
        /// No road trip is timed from a straight cell-to-cell arrow.
        /// </summary>
        /// <param name="Grid">Field grid. Its mask is the field test.</param>
        /// <param name="Surface">Surface that receives the hours and the courses.</param>
        /// <param name="Passes">Road-stage trips in planner order.</param>
        /// <param name="Options">Speeds, radius, blade, and Dubins rules.</param>
        /// <param name="BowlBcy">Scraper capacity in bank cubic yards.</param>
        private static void GuideRoadStage
            (
            Grid Grid,
            RoadPadSurface Surface,
            List<Pass> Passes,
            HaulBuilderOptions Options,
            Double BowlBcy
            )
        {
            if (Passes.Count == 0)
            {
                Surface.RoadCourses = new List<TripGuidance.TripCourse>();
                Surface.RoadStageHours = 0.0;
                return;
            }

            TimeParams Clock = new TimeParams(
                Options.CutSpeedMph,
                Options.LoadedTravelSpeedMph,
                Options.FillSpeedMph,
                Options.UnloadedTravelSpeedMph,
                Options.AccelerationTimeS,
                Options.DecelerationTimeS,
                Options.FieldEfficiency);
            Double RadiusM = Options.MinimumTurningDiameterFt * Constants.FEET_TO_METERS * Constants.HALF_SCALE;
            Double BladeM = Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER;
            Double BowlM3 = (Constants.M3_TO_CY > 0.0) ? (BowlBcy / Constants.M3_TO_CY) : 0.0;
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Grid,
                Passes,
                Clock,
                BowlM3,
                BladeM,
                RadiusM,
                -1,
                Options.Guidance);
            Surface.RoadCourses = Job.Trips;
            Double Factor = ((Options.FieldEfficiency > 0.0) && (Options.FieldEfficiency <= 1.0))
                ? Options.FieldEfficiency
                : 1.0;
            Double Seconds = 0.0;
            foreach (TripGuidance.TripCourse Trip in Job.Trips)
            {
                Seconds += Trip.ProductiveSeconds;
            }

            Surface.RoadStageHours = (Seconds / Factor) / Constants.SECONDS_PER_HOUR;
        }

        /// <summary>
        /// Distance from a point to the nearest boundary edge.
        /// </summary>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <param name="Ring">Boundary ring.</param>
        /// <returns>Distance in metres.</returns>
        private static Double DistanceToRing
            (
            Double X,
            Double Y,
            List<(Double X, Double Y)> Ring
            )
        {
            Double Best = Double.PositiveInfinity;
            for (Int32 Index = 0; Index < Ring.Count; Index += 1)
            {
                (Double X, Double Y) A = Ring[Index];
                (Double X, Double Y) B = Ring[(Index + 1) % Ring.Count];
                Best = Math.Min(Best, SegmentDistance(X, Y, A.X, A.Y, B.X, B.Y));
            }

            return Best;
        }

        /// <summary>
        /// Distance from a point to a segment.
        /// </summary>
        /// <param name="X">Point easting.</param>
        /// <param name="Y">Point northing.</param>
        /// <param name="Ax">Segment start easting.</param>
        /// <param name="Ay">Segment start northing.</param>
        /// <param name="Bx">Segment end easting.</param>
        /// <param name="By">Segment end northing.</param>
        /// <returns>Distance in metres.</returns>
        private static Double SegmentDistance
            (
            Double X,
            Double Y,
            Double Ax,
            Double Ay,
            Double Bx,
            Double By
            )
        {
            Double Dx = Bx - Ax;
            Double Dy = By - Ay;
            Double Length2 = (Dx * Dx) + (Dy * Dy);
            Double T = 0.0;
            if (Length2 > 0.0)
            {
                T = Math.Clamp((((X - Ax) * Dx) + ((Y - Ay) * Dy)) / Length2, 0.0, 1.0);
            }

            return Distance(X, Y, Ax + (T * Dx), Ay + (T * Dy));
        }

        /// <summary>
        /// Plan distance between two points.
        /// </summary>
        /// <param name="X0">First easting.</param>
        /// <param name="Y0">First northing.</param>
        /// <param name="X1">Second easting.</param>
        /// <param name="Y1">Second northing.</param>
        /// <returns>Distance in metres.</returns>
        private static Double Distance
            (
            Double X0,
            Double Y0,
            Double X1,
            Double Y1
            )
        {
            Double Dx = X1 - X0;
            Double Dy = Y1 - Y0;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>One resolved road or pad.</summary>
        /// <summary>One pan segment after axis snapping. A diagonal segment keeps the design line.</summary>
        private readonly struct BuiltSegment
        {
            /// <summary>
            /// Initializes one built segment.
            /// </summary>
            /// <param name="Ax">Start easting.</param>
            /// <param name="Ay">Start northing.</param>
            /// <param name="Bx">End easting.</param>
            /// <param name="By">End northing.</param>
            /// <param name="ShiftM">Lateral snap in metres. Zero when the segment was not snapped.</param>
            /// <param name="Snapped">True when the segment was straightened onto a grid axis.</param>
            public BuiltSegment(Double Ax, Double Ay, Double Bx, Double By, Double ShiftM, Boolean Snapped)
            {
                this.Ax = Ax;
                this.Ay = Ay;
                this.Bx = Bx;
                this.By = By;
                this.ShiftM = ShiftM;
                this.Snapped = Snapped;
            }

            /// <summary>Gets the start easting.</summary>
            public Double Ax { get; }

            /// <summary>Gets the start northing.</summary>
            public Double Ay { get; }

            /// <summary>Gets the end easting.</summary>
            public Double Bx { get; }

            /// <summary>Gets the end northing.</summary>
            public Double By { get; }

            /// <summary>Gets the lateral snap in metres. Zero for a diagonal segment.</summary>
            public Double ShiftM { get; }

            /// <summary>Gets a value indicating whether this segment was straightened onto a grid axis.</summary>
            public Boolean Snapped { get; }
        }

        private sealed class Feature
        {
            /// <summary>Gets a value indicating whether this feature is a pad.</summary>
            public Boolean IsPad { get; init; }

            /// <summary>Gets the road index, or -1 for a pad.</summary>
            public Int32 RoadIndex { get; init; }

            /// <summary>Gets the pad index, or -1 for a road.</summary>
            public Int32 PadIndex { get; init; }

            /// <summary>Gets the shoulder height in metres. A pad rises from this height toward the well.</summary>
            public Double HeightM { get; init; }

            /// <summary>Gets the shoulder height in feet. A pad rises from this height toward the well.</summary>
            public Double HeightFt { get; init; }

            /// <summary>Gets the road centreline height in feet. Zero for a pad.</summary>
            public Double CentrelineFt { get; init; }

            /// <summary>Gets the cross slope as a rise over run.</summary>
            public Double Cross { get; init; }

            /// <summary>Gets half the top width, or the pad top radius, in metres.</summary>
            public Double HalfTopM { get; init; }

            /// <summary>Gets the horizontal side-slope run in metres.</summary>
            public Double SideReachM { get; init; }

            /// <summary>Gets the distance from the centre to the toe, in metres.</summary>
            public Double ToeM { get; init; }

            /// <summary>Gets the pad exclusion radius in metres.</summary>
            public Double ExclusionM { get; init; }

            /// <summary>Gets the pad top radius in metres.</summary>
            public Double TopRadiusM { get; init; }

            /// <summary>Gets the pad top radius in feet.</summary>
            public Double TopRadiusFt { get; init; }

            /// <summary>Gets the finished top width in feet.</summary>
            public Double TopWidthFt { get; init; }

            /// <summary>Gets the pan lane count.</summary>
            public Int32 LaneCount { get; init; }

            /// <summary>Gets the pan width in feet.</summary>
            public Double PanWidthFt { get; init; }

            /// <summary>Gets half the pan width, or the pad pan radius, in metres.</summary>
            public Double PanHalfM { get; init; }

            /// <summary>Gets the pad pan radius in metres. Zero for a road.</summary>
            public Double PanRadiusM { get; init; }

            /// <summary>Gets the flat pan height in metres.</summary>
            public Double BuildHeightM { get; init; }

            /// <summary>Gets the flat pan height in feet.</summary>
            public Double BuildHeightFt { get; init; }

            /// <summary>Gets or sets the signed pan snap of the largest move, in feet. Positive is east or north.</summary>
            public Double PanShiftFt { get; set; }

            /// <summary>Gets or sets the fraction of pan volume in cells that are not fully covered.</summary>
            public Double PartialPanFraction { get; set; }

            /// <summary>Gets the NRCS minimum roadbed in feet. This is the standard, not a local override.</summary>
            public Double NrcsFt { get; init; }

            /// <summary>Gets a value indicating whether this road sets <see cref="RoadCentreline.MinRoadbedFt"/>.</summary>
            public Boolean HasRoadbedOverride { get; init; }

            /// <summary>Gets the local roadbed override in feet. Zero when <see cref="HasRoadbedOverride"/> is false.</summary>
            public Double RoadbedOverrideFt { get; init; }

            /// <summary>Gets the grade warning limit in percent.</summary>
            public Double MaxGradePct { get; init; }

            /// <summary>Gets the lane width in metres.</summary>
            public Double LaneM { get; init; }

            /// <summary>Gets the centreline length in metres.</summary>
            public Double LengthM { get; init; }

            /// <summary>Gets the pad or unused road anchor easting.</summary>
            public Double X { get; init; }

            /// <summary>Gets the pad or unused road anchor northing.</summary>
            public Double Y { get; init; }

            /// <summary>Gets the pad lane direction east.</summary>
            public Double DirX { get; init; }

            /// <summary>Gets the pad lane direction north.</summary>
            public Double DirY { get; init; }

            /// <summary>Gets the pad lane perpendicular east.</summary>
            public Double PerpX { get; init; }

            /// <summary>Gets the pad lane perpendicular north.</summary>
            public Double PerpY { get; init; }

            /// <summary>Gets the road vertices.</summary>
            public List<(Double X, Double Y)> Vertices { get; init; } = new List<(Double X, Double Y)>();

            /// <summary>Gets the segments the pan and the finished section are built on.</summary>
            public List<BuiltSegment> Built { get; } = new List<BuiltSegment>();

            /// <summary>
            /// Builds a road feature.
            /// </summary>
            /// <param name="Road">Centreline.</param>
            /// <param name="Index">Index among kept roads.</param>
            /// <param name="Options">Defaults and blade width.</param>
            /// <returns>The resolved road.</returns>
            static public Feature FromRoad
                (
                RoadCentreline Road,
                Int32 Index,
                HaulBuilderOptions Options
                )
            {
                Double LaneFt = LaneWidthFt(Options);
                Double Bed = Road.MinRoadbedFt ?? Options.RoadNrcsMinRoadbedFt;
                Int32 MinLanes = Road.MinLanes ?? Options.RoadMinLanes;
                Double HeightFt = Road.HeightFt ?? Options.RoadHeightFt;
                Double Side = Road.SideSlope ?? Options.RoadSideSlope;
                Double CrossPct = Road.CrossSlopePct ?? Options.RoadCrossSlopePct;
                Int32 Lanes = PanLaneCount(LaneFt, Bed, MinLanes, Side, HeightFt);
                Double TopFt = TopWidthFt(LaneFt, Bed, MinLanes, Side, HeightFt);
                Double PanFt = Lanes * LaneFt;
                Double Section = FinishedSectionSqFt(LaneFt, Bed, MinLanes, Side, HeightFt, CrossPct);
                // Same volume per foot as the crowned section, spread flat across the pan lanes.
                Double BuildFt = (PanFt > 1e-9) ? (Section / PanFt) : 0.0;
                Double HeightM = HeightFt * Constants.FEET_TO_METERS;
                Double Half = (TopFt * Constants.HALF_SCALE) * Constants.FEET_TO_METERS;
                // The side slope starts at the shoulder. The crown is above the shoulder and does not move the toe.
                Double Reach = Side * HeightM;
                Double Length = 0.0;
                for (Int32 Vertex = 1; Vertex < Road.Vertices.Count; Vertex += 1)
                {
                    Length += Distance(
                        Road.Vertices[Vertex - 1].X,
                        Road.Vertices[Vertex - 1].Y,
                        Road.Vertices[Vertex].X,
                        Road.Vertices[Vertex].Y);
                }

                Feature RoadFeature = new Feature
                {
                    IsPad = false,
                    RoadIndex = Index,
                    PadIndex = -1,
                    HeightM = HeightM,
                    HeightFt = HeightFt,
                    CentrelineFt = HeightFt + ((CrossPct / 100.0) * (TopFt * Constants.HALF_SCALE)),
                    Cross = CrossPct / 100.0,
                    HalfTopM = Half,
                    SideReachM = Reach,
                    ToeM = Half + Reach,
                    TopWidthFt = TopFt,
                    LaneCount = Lanes,
                    PanWidthFt = PanFt,
                    PanHalfM = (PanFt * Constants.HALF_SCALE) * Constants.FEET_TO_METERS,
                    BuildHeightM = BuildFt * Constants.FEET_TO_METERS,
                    BuildHeightFt = BuildFt,
                    NrcsFt = Options.RoadNrcsMinRoadbedFt,
                    HasRoadbedOverride = Road.MinRoadbedFt.HasValue,
                    RoadbedOverrideFt = Road.MinRoadbedFt ?? 0.0,
                    MaxGradePct = Road.MaxGradePct ?? Options.RoadMaxGradePct,
                    LaneM = LaneFt * Constants.FEET_TO_METERS,
                    LengthM = Length,
                    Vertices = Road.Vertices,
                };
                for (Int32 Vertex = 1; Vertex < Road.Vertices.Count; Vertex += 1)
                {
                    (Double X, Double Y) A = Road.Vertices[Vertex - 1];
                    (Double X, Double Y) B = Road.Vertices[Vertex];
                    RoadFeature.Built.Add(new BuiltSegment(A.X, A.Y, B.X, B.Y, 0.0, false));
                }

                return RoadFeature;
            }

            /// <summary>
            /// Builds a pad feature. Lanes follow the road that arrives at the well.
            /// </summary>
            /// <param name="Pad">Well pad.</param>
            /// <param name="Index">Index among pads.</param>
            /// <param name="Options">Defaults.</param>
            /// <param name="Roads">Kept roads, used for the arrival heading.</param>
            /// <returns>The resolved pad.</returns>
            static public Feature FromPad
                (
                WellPad Pad,
                Int32 Index,
                HaulBuilderOptions Options,
                List<RoadCentreline>? Roads
                )
            {
                Double HeightFt = Pad.HeightFt ?? Options.RoadHeightFt;
                Double Side = Pad.SideSlope ?? Options.PadSideSlope;
                Double Slope = Pad.SlopeAwayPct ?? Options.PadSlopeAwayPct;
                Double Turn = Pad.DesignTurnRadiusFt ?? Options.PadDesignTurnRadiusFt;
                Double Clearance = Pad.ClearanceFt ?? Options.PadClearanceFt;
                Double ExclusionFt = Pad.ExclusionRadiusFt ?? Options.PadExclusionRadiusFt;
                Double TopFt = PadTopRadiusFt(Turn, Clearance);
                Double HeightM = HeightFt * Constants.FEET_TO_METERS;
                Double TopM = TopFt * Constants.FEET_TO_METERS;
                Double Reach = Side * HeightM;
                Double Cross = Slope / 100.0;
                // The pan is a flat disc. Its radius sits halfway down the side slope, and its
                // height carries the same volume as the sloped top and the 3:1 edge.
                Double PanR = TopM + (Reach * Constants.HALF_SCALE);
                Double BuildM = PadBuildHeightM(HeightM, Cross, TopM, Reach, PanR);
                Double DirX = 1.0;
                Double DirY = 0.0;
                if (Roads != null)
                {
                    Double Best = Double.PositiveInfinity;
                    foreach (RoadCentreline Road in Roads)
                    {
                        Feature Temporary = FromRoad(Road, 0, Options);
                        Hit Closest = Temporary.Closest(Pad.X, Pad.Y);
                        if (Closest.Found && (Closest.Distance < Best))
                        {
                            Best = Closest.Distance;
                            DirX = Closest.DirX;
                            DirY = Closest.DirY;
                        }
                    }
                }

                return new Feature
                {
                    IsPad = true,
                    RoadIndex = -1,
                    PadIndex = Index,
                    HeightM = HeightM,
                    HeightFt = HeightFt,
                    Cross = Cross,
                    HalfTopM = TopM,
                    SideReachM = Reach,
                    ToeM = TopM + Reach,
                    ExclusionM = ExclusionFt * Constants.FEET_TO_METERS,
                    TopRadiusM = TopM,
                    TopRadiusFt = TopFt,
                    PanRadiusM = PanR,
                    PanHalfM = PanR,
                    BuildHeightM = BuildM,
                    BuildHeightFt = BuildM / Constants.FEET_TO_METERS,
                    X = Pad.X,
                    Y = Pad.Y,
                    DirX = DirX,
                    DirY = DirY,
                    PerpX = -DirY,
                    PerpY = DirX,
                };
            }

            /// <summary>
            /// Moves each segment that lies within 1° of a grid axis so the pan covers exactly
            /// <see cref="LaneCount"/> cells. Even lane counts centre on a cell boundary.
            /// Odd lane counts centre on a cell centre. The finished section uses the same line.
            /// </summary>
            /// <param name="Grid">Cell origin and size.</param>
            public void SnapPan
                (
                Grid Grid
                )
            {
                Built.Clear();
                Double Limit = PAN_AXIS_DEGREES * Math.PI / 180.0;
                Double MaxShift = 0.0;
                Boolean OnBoundary = (LaneCount % 2) == 0;
                for (Int32 Index = 1; Index < Vertices.Count; Index += 1)
                {
                    (Double X, Double Y) A = Vertices[Index - 1];
                    (Double X, Double Y) B = Vertices[Index];
                    Double Dx = B.X - A.X;
                    Double Dy = B.Y - A.Y;
                    Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                    if (Length < 1e-9)
                    {
                        Built.Add(new BuiltSegment(A.X, A.Y, B.X, B.Y, 0.0, false));
                        continue;
                    }

                    Double Angle = Math.Abs(Math.Atan2(Dy, Dx));
                    if (Angle > (Math.PI * Constants.HALF_SCALE))
                    {
                        Angle = Math.PI - Angle;
                    }

                    Double Shift = 0.0;
                    Boolean Snapped = false;
                    Double Ax = A.X;
                    Double Ay = A.Y;
                    Double Bx = B.X;
                    Double By = B.Y;
                    if (Angle <= Limit)
                    {
                        // Nearly east-west. Straighten onto the axis, then slide onto the lane lattice.
                        Double Target = SnapCoordinate((A.Y + B.Y) * Constants.HALF_SCALE, Grid.MinY, Grid.GridSizeM, OnBoundary);
                        Shift = Target - ((A.Y + B.Y) * Constants.HALF_SCALE);
                        Ay = Target;
                        By = Target;
                        Snapped = true;
                    }
                    else if (((Math.PI * Constants.HALF_SCALE) - Angle) <= Limit)
                    {
                        Double Target = SnapCoordinate((A.X + B.X) * Constants.HALF_SCALE, Grid.MinX, Grid.GridSizeM, OnBoundary);
                        Shift = Target - ((A.X + B.X) * Constants.HALF_SCALE);
                        Ax = Target;
                        Bx = Target;
                        Snapped = true;
                    }

                    if (Math.Abs(Shift) > Math.Abs(MaxShift))
                    {
                        MaxShift = Shift;
                    }

                    Built.Add(new BuiltSegment(Ax, Ay, Bx, By, Shift, Snapped));
                }

                PanShiftFt = MaxShift / Constants.FEET_TO_METERS;
            }

            /// <summary>
            /// Fraction of a cell covered by this road's pan. One segment uses an exact clip.
            /// Several segments use a sample of their union.
            /// </summary>
            /// <param name="X0">Cell west.</param>
            /// <param name="Y0">Cell south.</param>
            /// <param name="X1">Cell east.</param>
            /// <param name="Y1">Cell north.</param>
            /// <returns>Coverage from 0 to 1.</returns>
            public Double Coverage
                (
                Double X0,
                Double Y0,
                Double X1,
                Double Y1
                )
            {
                Double CellArea = (X1 - X0) * (Y1 - Y0);
                if ((CellArea <= 0.0) || (PanHalfM <= 0.0) || (Built.Count == 0))
                {
                    return 0.0;
                }

                Double Best = 0.0;
                Int32 Hits = 0;
                (Double X, Double Y)[] Corners = new (Double X, Double Y)[4];
                (Double X, Double Y)[] Local = new (Double X, Double Y)[4];
                foreach (BuiltSegment Segment in Built)
                {
                    PanCorners(Segment, PanHalfM, Corners);
                    // Clip in cell-local metres. World coordinates are large enough that the
                    // shoelace cancellation marks a full cell as a fraction short of full.
                    for (Int32 Corner = 0; Corner < 4; Corner += 1)
                    {
                        Local[Corner] = (Corners[Corner].X - X0, Corners[Corner].Y - Y0);
                    }

                    Double Area = ClippedArea(Local, 0.0, 0.0, X1 - X0, Y1 - Y0);
                    if (Area > Best)
                    {
                        Best = Area;
                    }

                    if (Area > 1e-12)
                    {
                        Hits += 1;
                    }
                }

                Double Fraction;
                if (Hits <= 1)
                {
                    Fraction = Best / CellArea;
                }
                else
                {
                    // A bend can cover one cell from two rectangles. Sample the union.
                    Int32 Count = 16;
                    Int32 Inside = 0;
                    Double StepX = (X1 - X0) / Count;
                    Double StepY = (Y1 - Y0) / Count;
                    for (Int32 Row = 0; Row < Count; Row += 1)
                    {
                        for (Int32 Column = 0; Column < Count; Column += 1)
                        {
                            if (InPan(X0 + ((Column + Constants.HALF_SCALE) * StepX), Y0 + ((Row + Constants.HALF_SCALE) * StepY)))
                            {
                                Inside += 1;
                            }
                        }
                    }

                    Fraction = Inside / (Double)(Count * Count);
                }

                if (Fraction >= (1.0 - 1e-8))
                {
                    return 1.0;
                }

                if (Fraction <= 1e-8)
                {
                    return 0.0;
                }

                return Math.Min(1.0, Fraction);
            }

            /// <summary>
            /// Four corners of the pan rectangle for one built segment.
            /// </summary>
            /// <param name="Segment">Built segment.</param>
            /// <param name="Half">Half the pan width, in metres.</param>
            /// <param name="Corners">Receives the corners in order.</param>
            private static void PanCorners
                (
                BuiltSegment Segment,
                Double Half,
                (Double X, Double Y)[] Corners
                )
            {
                Double Dx = Segment.Bx - Segment.Ax;
                Double Dy = Segment.By - Segment.Ay;
                Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Length < 1e-9)
                {
                    Corners[0] = (Segment.Ax, Segment.Ay);
                    Corners[1] = (Segment.Ax, Segment.Ay);
                    Corners[2] = (Segment.Ax, Segment.Ay);
                    Corners[3] = (Segment.Ax, Segment.Ay);
                    return;
                }

                Double Px = (-Dy / Length) * Half;
                Double Py = (Dx / Length) * Half;
                Corners[0] = (Segment.Ax + Px, Segment.Ay + Py);
                Corners[1] = (Segment.Bx + Px, Segment.By + Py);
                Corners[2] = (Segment.Bx - Px, Segment.By - Py);
                Corners[3] = (Segment.Ax - Px, Segment.Ay - Py);
            }

            /// <summary>
            /// True when the point lies in the pan rectangle or the pan disc.
            /// Past a road end the rectangle stops. It does not wrap.
            /// A snapped segment uses the shifted line, so the pan stays on the whole cells.
            /// </summary>
            /// <param name="X">Easting.</param>
            /// <param name="Y">Northing.</param>
            /// <returns>True inside the pan.</returns>
            public Boolean InPan
                (
                Double X,
                Double Y
                )
            {
                if (IsPad)
                {
                    return Distance(X, Y, this.X, this.Y) <= (PanRadiusM + 1e-4);
                }

                if ((Built.Count == 0) || (PanHalfM <= 0.0))
                {
                    return false;
                }

                foreach (BuiltSegment Segment in Built)
                {
                    Double Dx = Segment.Bx - Segment.Ax;
                    Double Dy = Segment.By - Segment.Ay;
                    Double Length2 = (Dx * Dx) + (Dy * Dy);
                    if (Length2 < 1e-18)
                    {
                        continue;
                    }

                    Double T = (((X - Segment.Ax) * Dx) + ((Y - Segment.Ay) * Dy)) / Length2;
                    // A point past the end is outside this segment's rectangle.
                    if ((T < -1e-8) || (T > (1.0 + 1e-8)))
                    {
                        continue;
                    }

                    T = Math.Clamp(T, 0.0, 1.0);
                    Double Px = Segment.Ax + (T * Dx);
                    Double Py = Segment.Ay + (T * Dy);
                    if (Distance(X, Y, Px, Py) <= (PanHalfM + 1e-4))
                    {
                        return true;
                    }
                }

                return false;
            }

            /// <summary>
            /// Height above the field design at a point, in metres.
            /// </summary>
            /// <param name="X">Easting.</param>
            /// <param name="Y">Northing.</param>
            /// <returns>Height in metres.</returns>
            public Double HeightAt
                (
                Double X,
                Double Y
                )
            {
                Double Radial = IsPad
                    ? Distance(X, Y, this.X, this.Y)
                    : Closest(X, Y).Distance;
                if (Radial <= HalfTopM)
                {
                    // The shoulder is HeightM. Both a road and a pad rise from there toward the middle.
                    return HeightM + (Cross * (HalfTopM - Radial));
                }

                if ((Radial >= ToeM) || (SideReachM <= 0.0))
                {
                    return 0.0;
                }

                // The 3:1 edge leaves the shoulder and reaches the toe.
                Double Fraction = (Radial - HalfTopM) / SideReachM;
                return Math.Max(0.0, HeightM * (1.0 - Fraction));
            }

            /// <summary>
            /// Closest point on the built centreline. A snapped road uses the shifted line.
            /// </summary>
            /// <param name="X">Easting.</param>
            /// <param name="Y">Northing.</param>
            /// <returns>Distance, signed offset, and station.</returns>
            public Hit Closest
                (
                Double X,
                Double Y
                )
            {
                Hit Best = new Hit
                {
                    Distance = Double.PositiveInfinity,
                };
                Double Station = 0.0;
                foreach (BuiltSegment Segment in Built)
                {
                    Double Dx = Segment.Bx - Segment.Ax;
                    Double Dy = Segment.By - Segment.Ay;
                    Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                    if (Length < 1e-9)
                    {
                        continue;
                    }

                    Double T = Math.Clamp((((X - Segment.Ax) * Dx) + ((Y - Segment.Ay) * Dy)) / (Length * Length), 0.0, 1.0);
                    Double Px = Segment.Ax + (T * Dx);
                    Double Py = Segment.Ay + (T * Dy);
                    Double OffsetX = X - Px;
                    Double OffsetY = Y - Py;
                    Double Dist = Math.Sqrt((OffsetX * OffsetX) + (OffsetY * OffsetY));
                    if (Dist < Best.Distance)
                    {
                        Double Ux = Dx / Length;
                        Double Uy = Dy / Length;
                        Best.Distance = Dist;
                        Best.Signed = (Ux * OffsetY) - (Uy * OffsetX);
                        Best.Station = Station + (T * Length);
                        Best.DirX = Ux;
                        Best.DirY = Uy;
                        Best.Found = true;
                    }

                    Station += Length;
                }

                return Best;
            }

            /// <summary>
            /// Point at a distance along the centreline.
            /// </summary>
            /// <param name="StationM">Distance from the first vertex, in metres.</param>
            /// <returns>The point.</returns>
            public (Double X, Double Y) PointAt
                (
                Double StationM
                )
            {
                Double Left = Math.Max(0.0, StationM);
                for (Int32 Index = 1; Index < Vertices.Count; Index += 1)
                {
                    (Double X, Double Y) A = Vertices[Index - 1];
                    (Double X, Double Y) B = Vertices[Index];
                    Double Length = Distance(A.X, A.Y, B.X, B.Y);
                    if (Left <= Length)
                    {
                        Double T = (Length <= 1e-9) ? 0.0 : (Left / Length);
                        return (A.X + ((B.X - A.X) * T), A.Y + ((B.Y - A.Y) * T));
                    }

                    Left -= Length;
                }

                return Vertices[Vertices.Count - 1];
            }
        }

        /// <summary>Closest-point result on a centreline.</summary>
        private struct Hit
        {
            /// <summary>Gets or sets the distance to the centreline in metres.</summary>
            public Double Distance;

            /// <summary>Gets or sets the signed offset. Positive is left of travel.</summary>
            public Double Signed;

            /// <summary>Gets or sets the station along the centreline in metres.</summary>
            public Double Station;

            /// <summary>Gets or sets the travel direction east.</summary>
            public Double DirX;

            /// <summary>Gets or sets the travel direction north.</summary>
            public Double DirY;

            /// <summary>Gets or sets a value indicating whether a segment was found.</summary>
            public Boolean Found;
        }

        /// <summary>A cut cell that can still supply the road stage.</summary>
        private sealed class Stock
        {
            /// <summary>
            /// Initializes one stock cell.
            /// </summary>
            /// <param name="Bx">Column.</param>
            /// <param name="By">Row.</param>
            /// <param name="LeftM">Remaining depth in metres.</param>
            /// <param name="DistanceM">Distance to the nearest road or pad.</param>
            public Stock(Int32 Bx, Int32 By, Double LeftM, Double DistanceM)
            {
                this.Bx = Bx;
                this.By = By;
                this.LeftM = LeftM;
                this.DistanceM = DistanceM;
            }

            /// <summary>Gets the column.</summary>
            public Int32 Bx { get; }

            /// <summary>Gets the row.</summary>
            public Int32 By { get; }

            /// <summary>Gets or sets the remaining reserved depth in metres.</summary>
            public Double LeftM { get; set; }

            /// <summary>Gets the distance to the nearest road or pad.</summary>
            public Double DistanceM { get; }
        }

        /// <summary>A fill cell on one lane.</summary>
        private sealed class LaneCell
        {
            /// <summary>
            /// Initializes one lane cell.
            /// </summary>
            /// <param name="Bx">Column.</param>
            /// <param name="By">Row.</param>
            /// <param name="Order">Position along the lane.</param>
            /// <param name="LeftM">Remaining fill depth in metres.</param>
            public LaneCell(Int32 Bx, Int32 By, Double Order, Double LeftM)
            {
                this.Bx = Bx;
                this.By = By;
                this.Order = Order;
                this.LeftM = LeftM;
            }

            /// <summary>Gets the column.</summary>
            public Int32 Bx { get; }

            /// <summary>Gets the row.</summary>
            public Int32 By { get; }

            /// <summary>Gets the position along the lane.</summary>
            public Double Order { get; }

            /// <summary>Gets or sets the remaining fill depth in metres.</summary>
            public Double LeftM { get; set; }
        }
    }
}
