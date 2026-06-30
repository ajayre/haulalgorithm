namespace HaulBuilder
{
    /// <summary>
    /// In-memory equivalent of all <c>landlevel.cli</c> arguments.
    /// </summary>
    public sealed class HaulBuilderOptions
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="HaulBuilderOptions"/> class.
        /// </summary>
        public HaulBuilderOptions
            (
            )
        {
            // Defaults mirror Python argparse defaults so CLI parity remains deterministic.
            AgdPath = String.Empty;
            OutputPngPath = "haulbuilder.png";
            GridSizeFt = Constants.DEFAULT_GRID_SIZE_FT;
            ToleranceMm = Constants.DEFAULT_TOLERANCE_MM;
            ScraperCapacityLcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY;
            SwellFactor = Constants.DEFAULT_SWELL_FACTOR;
            CutFillRatio = Constants.DEFAULT_COMPACTION;
            MaxDepthMm = Constants.DEFAULT_MAX_DEPTH_MM;
            BladeWidthMm = Constants.DEFAULT_BLADE_WIDTH_MM;
            HeadingStrength = 0.5;
            StripBias = 50.0;
            MaxDropHopFt = Constants.DEFAULT_MAX_DROP_HOP_FT;
            MinLoadFillPct = 90.0;
            FieldEfficiency = Constants.DEFAULT_FIELD_EFFICIENCY;
            LoadingSpeedMph = Constants.DEFAULT_LOADING_SPEED_MPH;
            HaulSpeedMph = Constants.DEFAULT_HAUL_SPEED_MPH;
            DumpingSpeedMph = Constants.DEFAULT_DUMPING_SPEED_MPH;
            ReturnSpeedMph = Constants.DEFAULT_RETURN_SPEED_MPH;
            AccelTimeS = Constants.DEFAULT_ACCEL_TIME_S;
            DecelTimeS = Constants.DEFAULT_DECEL_TIME_S;
            TransportMode = "poisson";
            BalanceMode = "check";
            PixelsPerBin = Constants.DEFAULT_PIXELS_PER_BIN;
            ArrowStride = 2;
            Zones = 8;
            LaneOverlap = 0.0;
            TurnSpeedMph = Constants.DEFAULT_TURN_SPEED_MPH;
            GradeSpeedMph = Constants.DEFAULT_GRADE_SPEED_MPH;
            TurningCircleFt = Constants.DEFAULT_TURNING_CIRCLE_FT;
            TurnThresholdDeg = Constants.DEFAULT_TURN_THRESHOLD_DEG;
            ReorderWindow = 0;
            AnimateStride = 10;
            AnimatePixelsPerBin = 8;
            AnimateMode = "cutfill";
            PivotCenter = null;
            PivotWheelbaseFt = Constants.DEFAULT_PIVOT_WHEELBASE_FT;
            PivotClearanceFt = Constants.DEFAULT_PIVOT_CLEARANCE_FT;
            PivotMoveCost = 25.0;
            PivotMode = "cutfill";
            PivotPixelsPerBin = 6;
            PivotAnimationStride = 15;
            EquipmentWidthFt = 14.0;
            ExcludePolygonSpecs = new List<String>();
            Seed = Constants.DEFAULT_SEED;
        }

        /// <summary>
        /// Gets or sets AGD input file path.
        /// </summary>
        public String AgdPath { get; set; }

        /// <summary>
        /// Gets or sets primary cut/fill PNG output path.
        /// </summary>
        public String OutputPngPath { get; set; }

        /// <summary>
        /// Gets or sets grid spacing in feet.
        /// </summary>
        public Double GridSizeFt { get; set; }

        /// <summary>
        /// Gets or sets finishing tolerance in millimetres.
        /// </summary>
        public Double ToleranceMm { get; set; }

        /// <summary>
        /// Gets or sets scraper capacity in loose cubic yards.
        /// </summary>
        public Double ScraperCapacityLcy { get; set; }

        /// <summary>
        /// Gets or sets swell factor used for LCY/BCY conversion.
        /// </summary>
        public Double SwellFactor { get; set; }

        /// <summary>
        /// Gets or sets cut/fill compaction ratio.
        /// </summary>
        public Double CutFillRatio { get; set; }

        /// <summary>
        /// Gets or sets maximum cut or fill depth per pass in millimetres.
        /// </summary>
        public Double MaxDepthMm { get; set; }

        /// <summary>
        /// Gets or sets blade width in millimetres.
        /// </summary>
        public Double BladeWidthMm { get; set; }

        /// <summary>
        /// Gets or sets optional global heading in compass degrees.
        /// </summary>
        public Double? HeadingDeg { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether flow-aligned strip growth is disabled.
        /// </summary>
        public Boolean NoFlowAligned { get; set; }

        /// <summary>
        /// Gets or sets heading-following strength for non-flow strip growth.
        /// </summary>
        public Double HeadingStrength { get; set; }

        /// <summary>
        /// Gets or sets strip-bias length in grid cells.
        /// </summary>
        public Double StripBias { get; set; }

        /// <summary>
        /// Gets or sets maximum multi-drop hop distance in feet.
        /// </summary>
        public Double MaxDropHopFt { get; set; }

        /// <summary>
        /// Gets or sets minimum load fill percentage before stopping extra hops.
        /// </summary>
        public Double MinLoadFillPct { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether knot re-profiling is disabled.
        /// </summary>
        public Boolean NoKnotRepair { get; set; }

        /// <summary>
        /// Gets or sets field efficiency fraction in [0, 1].
        /// </summary>
        public Double FieldEfficiency { get; set; }

        /// <summary>
        /// Gets or sets loading speed in miles per hour.
        /// </summary>
        public Double LoadingSpeedMph { get; set; }

        /// <summary>
        /// Gets or sets haul speed in miles per hour.
        /// </summary>
        public Double HaulSpeedMph { get; set; }

        /// <summary>
        /// Gets or sets dumping speed in miles per hour.
        /// </summary>
        public Double DumpingSpeedMph { get; set; }

        /// <summary>
        /// Gets or sets return speed in miles per hour.
        /// </summary>
        public Double ReturnSpeedMph { get; set; }

        /// <summary>
        /// Gets or sets acceleration phase duration in seconds.
        /// </summary>
        public Double AccelTimeS { get; set; }

        /// <summary>
        /// Gets or sets deceleration phase duration in seconds.
        /// </summary>
        public Double DecelTimeS { get; set; }

        /// <summary>
        /// Gets or sets transport mode (<c>poisson</c> or <c>mincostflow</c>).
        /// </summary>
        public String TransportMode { get; set; }

        /// <summary>
        /// Gets or sets balance mode (<c>check</c> or <c>adjust</c>).
        /// </summary>
        public String BalanceMode { get; set; }

        /// <summary>
        /// Gets or sets rendered pixels per grid cell.
        /// </summary>
        public Int32 PixelsPerBin { get; set; }

        /// <summary>
        /// Gets or sets arrow subsampling stride in grid cells.
        /// </summary>
        public Int32 ArrowStride { get; set; }

        /// <summary>
        /// Gets or sets number of rough cut zones for sequencing.
        /// </summary>
        public Int32 Zones { get; set; }

        /// <summary>
        /// Gets or sets optional zone-sequence CSV output path.
        /// </summary>
        public String? SequenceCsvPath { get; set; }

        /// <summary>
        /// Gets or sets optional cut/fill pair CSV output path.
        /// </summary>
        public String? PairsCsvPath { get; set; }

        /// <summary>
        /// Gets or sets optional cut/fill pair PNG output path.
        /// </summary>
        public String? PairsPngPath { get; set; }

        /// <summary>
        /// Gets or sets optional blade-scale guidance CSV output path.
        /// </summary>
        public String? GuidanceCsvPath { get; set; }

        /// <summary>
        /// Gets or sets optional blade-scale guidance PNG output path.
        /// </summary>
        public String? GuidancePngPath { get; set; }

        /// <summary>
        /// Gets or sets lane overlap fraction in [0, 0.9].
        /// </summary>
        public Double LaneOverlap { get; set; }

        /// <summary>
        /// Gets or sets turn speed in miles per hour.
        /// </summary>
        public Double TurnSpeedMph { get; set; }

        /// <summary>
        /// Gets or sets grade-return speed in miles per hour.
        /// </summary>
        public Double GradeSpeedMph { get; set; }

        /// <summary>
        /// Gets or sets turning circle diameter in feet.
        /// </summary>
        public Double TurningCircleFt { get; set; }

        /// <summary>
        /// Gets or sets turn threshold in degrees.
        /// </summary>
        public Double TurnThresholdDeg { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether grade-return passes are disabled.
        /// </summary>
        public Boolean NoGradeReturn { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether route smoothing is disabled.
        /// </summary>
        public Boolean NoRouteSmoothing { get; set; }

        /// <summary>
        /// Gets or sets local reorder search window (0 disables).
        /// </summary>
        public Int32 ReorderWindow { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether direction-aware sequencing is disabled.
        /// </summary>
        public Boolean NoDirectionAware { get; set; }

        /// <summary>
        /// Gets or sets optional animation frame output directory.
        /// </summary>
        public String? AnimateDirPath { get; set; }

        /// <summary>
        /// Gets or sets optional leveling animation GIF output path.
        /// </summary>
        public String? AnimateGifPath { get; set; }

        /// <summary>
        /// Gets or sets pass stride for leveling animation frames.
        /// </summary>
        public Int32 AnimateStride { get; set; }

        /// <summary>
        /// Gets or sets pixels per bin for animation rendering.
        /// </summary>
        public Int32 AnimatePixelsPerBin { get; set; }

        /// <summary>
        /// Gets or sets animation render mode (<c>cutfill</c> or <c>topology</c>).
        /// </summary>
        public String AnimateMode { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether center-pivot analysis is enabled.
        /// </summary>
        public Boolean Pivot { get; set; }

        /// <summary>
        /// Gets or sets optional explicit pivot center in <c>lat,lon</c>.
        /// </summary>
        public String? PivotCenter { get; set; }

        /// <summary>
        /// Gets or sets pivot arm wheelbase in feet.
        /// </summary>
        public Double PivotWheelbaseFt { get; set; }

        /// <summary>
        /// Gets or sets extra pivot arm clearance per side in feet.
        /// </summary>
        public Double PivotClearanceFt { get; set; }

        /// <summary>
        /// Gets or sets pivot move cost in detour-equivalent passes.
        /// </summary>
        public Double PivotMoveCost { get; set; }

        /// <summary>
        /// Gets or sets optional pivot PNG output directory.
        /// </summary>
        public String? PivotDirPath { get; set; }

        /// <summary>
        /// Gets or sets optional pivot schedule CSV output path.
        /// </summary>
        public String? PivotCsvPath { get; set; }

        /// <summary>
        /// Gets or sets pivot render mode (<c>cutfill</c> or <c>topology</c>).
        /// </summary>
        public String PivotMode { get; set; }

        /// <summary>
        /// Gets or sets pivot render pixels per bin.
        /// </summary>
        public Int32 PivotPixelsPerBin { get; set; }

        /// <summary>
        /// Gets or sets optional pivot leveling GIF output path.
        /// </summary>
        public String? PivotGifPath { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether pivot GIF generation is disabled.
        /// </summary>
        public Boolean NoPivotGif { get; set; }

        /// <summary>
        /// Gets or sets optional pivot frame output directory.
        /// </summary>
        public String? PivotFramesPath { get; set; }

        /// <summary>
        /// Gets or sets pass stride for pivot animation.
        /// </summary>
        public Int32 PivotAnimationStride { get; set; }

        /// <summary>
        /// Gets or sets machine width in feet for obstacle buffering.
        /// </summary>
        public Double EquipmentWidthFt { get; set; }

        /// <summary>
        /// Gets exclusion polygon specs as <c>lat,lon;lat,lon;...</c>.
        /// </summary>
        public List<String> ExcludePolygonSpecs { get; }

        /// <summary>
        /// Gets or sets a value indicating whether planning stages are skipped.
        /// </summary>
        public Boolean NoPlan { get; set; }

        /// <summary>
        /// Gets or sets number of random single-pair PNGs to render (0 disables).
        /// </summary>
        public Int32 PairSampleCount { get; set; }

        /// <summary>
        /// Gets or sets deterministic random seed for zone clustering.
        /// </summary>
        public Int32 Seed { get; set; }
    }
}
