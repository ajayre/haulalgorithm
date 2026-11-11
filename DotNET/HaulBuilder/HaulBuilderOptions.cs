namespace HaulBuilder
{
    /// <summary>
    /// User-facing inputs for the production haul planner. The current lane-first
    /// optimizer and mandatory production guidance are always used unless
    /// <see cref="SkipGuidance"/> requests a fast plan-only preview; this type does
    /// not expose the retired greedy allocation mode.
    /// </summary>
    public sealed class HaulBuilderOptions
    {
        /// <summary>Initializes production options with measured defaults.</summary>
        public HaulBuilderOptions()
        {
            AgdPath = String.Empty;
            GridSizeFt = Constants.DEFAULT_GRID_SIZE_FT;
            ToleranceMm = Constants.DEFAULT_TOLERANCE_MM;
            ScraperCapacityLcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY;
            SwellFactor = Constants.DEFAULT_SWELL_FACTOR;
            CutFillRatio = Constants.DEFAULT_COMPACTION;
            MaxDepthMm = Constants.DEFAULT_MAX_DEPTH_MM;
            BladeWidthMm = Constants.DEFAULT_BLADE_WIDTH_MM;
            LaneAxisHeadingDeg = 0.0;
            FieldEfficiency = Constants.DEFAULT_FIELD_EFFICIENCY;
            CutSpeedMph = Constants.DEFAULT_CUT_SPEED_MPH;
            LoadedTravelSpeedMph = Constants.DEFAULT_LOADED_TRAVEL_SPEED_MPH;
            FillSpeedMph = Constants.DEFAULT_FILL_SPEED_MPH;
            UnloadedTravelSpeedMph = Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH;
            AccelerationTimeS = Constants.DEFAULT_ACCELERATION_TIME_S;
            DecelerationTimeS = Constants.DEFAULT_DECELERATION_TIME_S;
            PixelsPerCell = Constants.DEFAULT_PIXELS_PER_BIN;
            FlowArrowStrideCells = 2;
            ZoneCount = 8;
            TurnSpeedMph = Constants.DEFAULT_TURN_SPEED_MPH;
            GradeSpeedMph = Constants.DEFAULT_GRADE_SPEED_MPH;
            MinimumTurningDiameterFt = Constants.DEFAULT_TURNING_CIRCLE_FT;
            EquipmentLengthFt = Constants.DEFAULT_EQUIPMENT_LENGTH_FT;
            WaypointTurnThresholdDeg = Constants.DEFAULT_TURN_THRESHOLD_DEG;
            AnimationStride = 10;
            AnimationPixelsPerCell = 8;
            AnimationMode = "cutfill";
            PivotCenter = null;
            PivotWheelbaseFt = Constants.DEFAULT_PIVOT_WHEELBASE_FT;
            PivotClearanceFt = Constants.DEFAULT_PIVOT_CLEARANCE_FT;
            PivotMoveCost = 25.0;
            PivotMode = "cutfill";
            PivotPixelsPerCell = 6;
            PivotAnimationStride = 15;
            EquipmentWidthFt = 14.0;
            Optimization = new HaulOptimizationOptions();
            Ponding = new PondingOptions();
            ExcludePolygonSpecs = new List<String>();
            RandomSeed = Constants.DEFAULT_SEED;
        }

        /// <summary>Gets or sets the required AGD input file path.</summary>
        public String AgdPath { get; set; }

        /// <summary>Gets or sets grid spacing in feet.</summary>
        public Double GridSizeFt { get; set; }

        /// <summary>Gets or sets finishing tolerance in millimetres.</summary>
        public Double ToleranceMm { get; set; }

        /// <summary>Gets or sets scraper capacity in loose cubic yards.</summary>
        public Double ScraperCapacityLcy { get; set; }

        /// <summary>Gets or sets the LCY/BCY swell factor.</summary>
        public Double SwellFactor { get; set; }

        /// <summary>Gets or sets bank cut required per bank fill.</summary>
        public Double CutFillRatio { get; set; }

        /// <summary>Gets or sets maximum cut or fill depth per cycle in millimetres.</summary>
        public Double MaxDepthMm { get; set; }

        /// <summary>Gets or sets blade width in millimetres.</summary>
        public Double BladeWidthMm { get; set; }

        /// <summary>
        /// Gets or sets the undirected work-lane axis in compass degrees. Values
        /// normalize into [0, 180), so 0 and 180 describe the same north-south
        /// axis. Production guidance independently chooses either bearing for
        /// every cut and fill.
        /// </summary>
        public Double LaneAxisHeadingDeg { get; set; }

        /// <summary>Gets or sets operational availability in [0, 1].</summary>
        public Double FieldEfficiency { get; set; }

        /// <summary>Gets or sets cut speed in miles per hour.</summary>
        public Double CutSpeedMph { get; set; }

        /// <summary>Gets or sets loaded travel speed in miles per hour.</summary>
        public Double LoadedTravelSpeedMph { get; set; }

        /// <summary>Gets or sets fill speed in miles per hour.</summary>
        public Double FillSpeedMph { get; set; }

        /// <summary>Gets or sets unloaded travel speed in miles per hour.</summary>
        public Double UnloadedTravelSpeedMph { get; set; }

        /// <summary>Gets or sets acceleration time in seconds.</summary>
        public Double AccelerationTimeS { get; set; }

        /// <summary>Gets or sets deceleration time in seconds.</summary>
        public Double DecelerationTimeS { get; set; }

        /// <summary>Gets or sets pixels per grid cell for static maps.</summary>
        public Int32 PixelsPerCell { get; set; }

        /// <summary>Gets or sets flow-arrow spacing in cells; zero disables arrows.</summary>
        public Int32 FlowArrowStrideCells { get; set; }

        /// <summary>Gets or sets coarse operational zone count; zero disables zones.</summary>
        public Int32 ZoneCount { get; set; }

        /// <summary>Gets or sets whether cut/fill pair CSV text is included.</summary>
        public Boolean IncludeCutFillPairCsv { get; set; }

        /// <summary>Gets or sets whether a cut/fill pair PNG is included.</summary>
        public Boolean IncludeCutFillPairPng { get; set; }

        /// <summary>Gets or sets whether a full guidance PNG is included.</summary>
        public Boolean IncludeGuidancePng { get; set; }

        /// <summary>Gets or sets turn speed in miles per hour.</summary>
        public Double TurnSpeedMph { get; set; }

        /// <summary>Gets or sets mandatory grade-return speed in miles per hour.</summary>
        public Double GradeSpeedMph { get; set; }

        /// <summary>Gets or sets minimum tractor turning diameter in feet.</summary>
        public Double MinimumTurningDiameterFt { get; set; }

        /// <summary>Gets or sets tractor and scraper overall length in feet.</summary>
        public Double EquipmentLengthFt { get; set; }

        /// <summary>
        /// Gets or sets the minimum blade-down cut/fill run length in feet.
        /// Zero means half of <see cref="EquipmentLengthFt"/>.
        /// Shorter coverage runs are omitted from automatic guidance and deferred.
        /// </summary>
        public Double MinimumWorkLengthFt { get; set; }

        /// <summary>Gets or sets the waypoint heading-change threshold counted as a turn.</summary>
        public Double WaypointTurnThresholdDeg { get; set; }

        /// <summary>
        /// Resolves the effective minimum blade-down work length in feet.
        /// </summary>
        /// <returns>Configured override, or half the equipment length when unset.</returns>
        public Double ResolveMinimumWorkLengthFt
            (
            )
        {
            if (MinimumWorkLengthFt > 0.0)
            {
                return MinimumWorkLengthFt;
            }
            return EquipmentLengthFt * Constants.HALF_SCALE;
        }

        /// <summary>
        /// Resolves the effective minimum blade-down work length in metres.
        /// </summary>
        /// <returns>Minimum work length used by coverage planning.</returns>
        public Double ResolveMinimumWorkLengthM
            (
            )
        {
            return ResolveMinimumWorkLengthFt() * Constants.FEET_TO_METERS;
        }

        /// <summary>Gets or sets whether leveling animation GIF bytes are included.</summary>
        public Boolean IncludeAnimationGif { get; set; }

        /// <summary>Gets or sets whether leveling animation frame PNGs are included.</summary>
        public Boolean IncludeAnimationFrames { get; set; }

        /// <summary>Gets or sets the number of completed routes between animation frames.</summary>
        public Int32 AnimationStride { get; set; }

        /// <summary>Gets or sets animation pixels per grid cell.</summary>
        public Int32 AnimationPixelsPerCell { get; set; }

        /// <summary>Gets or sets animation mode: <c>cutfill</c> or <c>topology</c>.</summary>
        public String AnimationMode { get; set; }

        /// <summary>Gets or sets whether center-pivot scheduling is enabled.</summary>
        public Boolean EnablePivot { get; set; }

        /// <summary>Gets or sets an optional explicit pivot center as <c>lat,lon</c>.</summary>
        public String? PivotCenter { get; set; }

        /// <summary>Gets or sets pivot arm wheelbase in feet.</summary>
        public Double PivotWheelbaseFt { get; set; }

        /// <summary>Gets or sets extra pivot arm clearance per side in feet.</summary>
        public Double PivotClearanceFt { get; set; }

        /// <summary>
        /// Gets or sets pivot reposition cost after guidance conflicts have been
        /// minimized. Guidance clearance always has priority.
        /// </summary>
        public Double PivotMoveCost { get; set; }

        /// <summary>Gets or sets pivot render mode: <c>cutfill</c> or <c>topology</c>.</summary>
        public String PivotMode { get; set; }

        /// <summary>Gets or sets pivot render pixels per grid cell.</summary>
        public Int32 PivotPixelsPerCell { get; set; }

        /// <summary>Gets or sets whether pivot animation GIF bytes are included.</summary>
        public Boolean IncludePivotAnimationGif { get; set; }

        /// <summary>Gets or sets whether pivot animation frame PNGs are included.</summary>
        public Boolean IncludePivotAnimationFrames { get; set; }

        /// <summary>Gets or sets pass stride for pivot animation.</summary>
        public Int32 PivotAnimationStride { get; set; }

        /// <summary>Gets or sets machine width in feet for obstacle buffering.</summary>
        public Double EquipmentWidthFt { get; set; }

        /// <summary>Gets current optimizer tuning. Production defaults are measured.</summary>
        public HaulOptimizationOptions Optimization { get; }

        /// <summary>Gets ponding-aware audit and sequencing options.</summary>
        public PondingOptions Ponding { get; }

        /// <summary>Gets exclusion polygons as <c>lat,lon;lat,lon;...</c>.</summary>
        public List<String> ExcludePolygonSpecs { get; }

        /// <summary>Gets or sets whether only field parsing, balance, flow, and map rendering run.</summary>
        public Boolean SkipHaulPlan { get; set; }

        /// <summary>
        /// Gets or sets whether exact production guidance is omitted after haul
        /// planning, routing, and optional ponding. Pair maps, zone sequencing,
        /// and productivity metrics still run; metrics fall back to
        /// <see cref="TimeModel"/> estimates until guidance is built.
        /// </summary>
        public Boolean SkipGuidance { get; set; }

        /// <summary>Gets or sets random individual route image count; zero disables it.</summary>
        public Int32 RandomRouteImageCount { get; set; }

        /// <summary>
        /// Gets or sets how many consecutive route PNGs to emit from route 1
        /// through route N, each showing one route's cut/fill guidance geometry.
        /// Zero disables sequential route renders.
        /// </summary>
        public Int32 RouteGuidanceImageCount { get; set; }

        /// <summary>
        /// Gets or sets how many random PNGs to emit, each showing three
        /// consecutive routes with cut runs in red and fill runs in blue.
        /// Zero disables triplet renders.
        /// </summary>
        public Int32 RandomRouteTripletCount { get; set; }

        /// <summary>Gets or sets the deterministic seed for zones and random renders.</summary>
        public Int32 RandomSeed { get; set; }
    }
}
