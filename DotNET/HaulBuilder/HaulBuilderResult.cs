namespace HaulBuilder
{
    /// <summary>
    /// Aggregated in-memory outputs from a full haul-builder pipeline run.
    /// </summary>
    public sealed class HaulBuilderResult
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="HaulBuilderResult"/> class.
        /// </summary>
        public HaulBuilderResult
            (
            )
        {
            // Containers are allocated eagerly so callers can append/inspect without null checks.
            Messages = new List<String>();
            ZoneSteps = new List<ZoneStep>();
            CutFillPairs = new List<CutFillPair>();
            GuidancePoints = new List<GuidancePoint>();
            PivotPngBytesByName = new Dictionary<String, Byte[]>();
            PngBytesByName = new Dictionary<String, Byte[]>();
            CsvTextByName = new Dictionary<String, String>();
        }

        /// <summary>
        /// Gets informational or warning messages emitted during processing.
        /// </summary>
        public List<String> Messages { get; }

        /// <summary>
        /// Gets or sets resolved engine options used for this run.
        /// </summary>
        public HaulBuilderOptions? Options { get; set; }

        /// <summary>
        /// Gets or sets loaded AGD field model.
        /// </summary>
        public AgdField? Field { get; set; }

        /// <summary>
        /// Gets or sets grid used for planning and rendering.
        /// </summary>
        public Grid? Grid { get; set; }

        /// <summary>
        /// Gets or sets optional transport flow field.
        /// </summary>
        public FlowField? Flow { get; set; }

        /// <summary>
        /// Gets or sets mass-balance report.
        /// </summary>
        public BalanceResult? Balance { get; set; }

        /// <summary>
        /// Gets or sets pass-generation result.
        /// </summary>
        public PassResult? PassResult { get; set; }

        /// <summary>
        /// Gets or sets route sequencing result.
        /// </summary>
        public Route? Route { get; set; }

        /// <summary>
        /// Gets zone sequencing summary rows.
        /// </summary>
        public List<ZoneStep> ZoneSteps { get; }

        /// <summary>
        /// Gets cut/fill pair rows in route order.
        /// </summary>
        public List<CutFillPair> CutFillPairs { get; }

        /// <summary>
        /// Gets operator guidance waypoints in drive order.
        /// </summary>
        public List<GuidancePoint> GuidancePoints { get; }

        /// <summary>
        /// Gets or sets full guidance result.
        /// </summary>
        public GuidanceResult? Guidance { get; set; }

        /// <summary>
        /// Gets or sets guidance time estimate.
        /// </summary>
        public GuidanceTime? GuidanceTime { get; set; }

        /// <summary>
        /// Gets or sets base time-model result.
        /// </summary>
        public TimeResult? TimeResult { get; set; }

        /// <summary>
        /// Gets or sets top-level analysis metrics summary.
        /// </summary>
        public HaulAnalysisMetrics? Metrics { get; set; }

        /// <summary>
        /// Gets or sets optional pivot geometry.
        /// </summary>
        public Pivot.PivotGeometry? PivotGeometry { get; set; }

        /// <summary>
        /// Gets or sets optional pivot schedule.
        /// </summary>
        public Pivot.PivotPlan? PivotPlan { get; set; }

        /// <summary>
        /// Gets or sets optional per-pass arm obstacle set.
        /// </summary>
        public Dictionary<Int32, Pivot.ObstacleField>? ArmObstacles { get; set; }

        /// <summary>
        /// Gets or sets optional pivot CSV payload.
        /// </summary>
        public Pivot.PivotCsvData? PivotCsvData { get; set; }

        /// <summary>
        /// Gets rendered PNG bytes by logical artifact key.
        /// </summary>
        public Dictionary<String, Byte[]> PngBytesByName { get; }

        /// <summary>
        /// Gets rendered pivot PNG bytes by logical artifact key.
        /// </summary>
        public Dictionary<String, Byte[]> PivotPngBytesByName { get; }

        /// <summary>
        /// Gets generated CSV text payloads by logical artifact key.
        /// </summary>
        public Dictionary<String, String> CsvTextByName { get; }

        /// <summary>
        /// Gets or sets optional leveling animation GIF bytes.
        /// </summary>
        public Byte[]? AnimationGifBytes { get; set; }

        /// <summary>
        /// Gets or sets optional pivot leveling animation GIF bytes.
        /// </summary>
        public Byte[]? PivotAnimationGifBytes { get; set; }
    }

    /// <summary>
    /// One route-ordered cut/fill pair summary row.
    /// </summary>
    public sealed class CutFillPair
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="CutFillPair"/> class.
        /// </summary>
        /// <param name="order">1-based pass order.</param>
        /// <param name="loadBcy">Carried bank volume in BCY.</param>
        /// <param name="haulDistanceM">Haul distance in metres.</param>
        /// <param name="cutStartXy">Cut-start world coordinates.</param>
        /// <param name="cutEndXy">Cut-end world coordinates.</param>
        /// <param name="fillStartXy">Fill-start world coordinates.</param>
        /// <param name="fillEndXy">Fill-end world coordinates.</param>
        /// <param name="totalCutBcyBefore">Cumulative cut BCY completed before this route.</param>
        /// <param name="totalFillBcyBefore">Cumulative fill BCY completed before this route.</param>
        /// <param name="routeTimeS">Estimated time to complete this route in seconds.</param>
        public CutFillPair
            (
            Int32 order,
            Double loadBcy,
            Double haulDistanceM,
            (Double X, Double Y) cutStartXy,
            (Double X, Double Y) cutEndXy,
            (Double X, Double Y) fillStartXy,
            (Double X, Double Y) fillEndXy,
            Double totalCutBcyBefore,
            Double totalFillBcyBefore,
            Double routeTimeS
            )
        {
            // Pair rows keep both scalar metrics and world geometry for exporters and UIs.
            Order = order;
            LoadBcy = loadBcy;
            HaulDistanceM = haulDistanceM;
            CutStartXy = cutStartXy;
            CutEndXy = cutEndXy;
            FillStartXy = fillStartXy;
            FillEndXy = fillEndXy;
            TotalCutBcyBefore = totalCutBcyBefore;
            TotalFillBcyBefore = totalFillBcyBefore;
            RouteTimeS = routeTimeS;
        }

        /// <summary>
        /// Gets route order index (1-based).
        /// </summary>
        public Int32 Order { get; }

        /// <summary>
        /// Gets carried load in BCY.
        /// </summary>
        public Double LoadBcy { get; }

        /// <summary>
        /// Gets haul distance in metres.
        /// </summary>
        public Double HaulDistanceM { get; }

        /// <summary>
        /// Gets cut-segment start world point.
        /// </summary>
        public (Double X, Double Y) CutStartXy { get; }

        /// <summary>
        /// Gets cut-segment end world point.
        /// </summary>
        public (Double X, Double Y) CutEndXy { get; }

        /// <summary>
        /// Gets fill-segment start world point.
        /// </summary>
        public (Double X, Double Y) FillStartXy { get; }

        /// <summary>
        /// Gets fill-segment end world point.
        /// </summary>
        public (Double X, Double Y) FillEndXy { get; }

        /// <summary>
        /// Gets cumulative cut BCY completed before this route.
        /// </summary>
        public Double TotalCutBcyBefore { get; }

        /// <summary>
        /// Gets cumulative fill BCY completed before this route.
        /// </summary>
        public Double TotalFillBcyBefore { get; }

        /// <summary>
        /// Gets estimated time to complete this route in seconds.
        /// </summary>
        public Double RouteTimeS { get; }
    }

    /// <summary>
    /// High-level analysis metrics equivalent to CLI printed KPIs.
    /// </summary>
    public sealed class HaulAnalysisMetrics
    {
        /// <summary>
        /// Gets or sets number of valid grid areas.
        /// </summary>
        public Int32 ValidAreas { get; set; }

        /// <summary>
        /// Gets or sets moved volume in BCY.
        /// </summary>
        public Double MoveVolumeBcy { get; set; }

        /// <summary>
        /// Gets or sets field area in acres.
        /// </summary>
        public Double FieldAreaAcres { get; set; }

        /// <summary>
        /// Gets or sets total run time in hours.
        /// </summary>
        public Double TotalHours { get; set; }

        /// <summary>
        /// Gets or sets move rate in BCY per hour.
        /// </summary>
        public Double MoveRateBcyPerHour { get; set; }

        /// <summary>
        /// Gets or sets area rate in acres per hour.
        /// </summary>
        public Double AreaRateAcresPerHour { get; set; }

        /// <summary>
        /// Gets or sets average move distance in yards.
        /// </summary>
        public Double AverageMoveDistanceYards { get; set; }

        /// <summary>
        /// Gets or sets applied heading in degrees when not flow-aligned.
        /// </summary>
        public Double? HeadingDeg { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether flow-aligned passes were enabled.
        /// </summary>
        public Boolean FlowAligned { get; set; }

        /// <summary>
        /// Gets or sets pivot detour extra haul in metres.
        /// </summary>
        public Double PivotExtraHaulM { get; set; }

        /// <summary>
        /// Gets or sets pivot detour added time in seconds.
        /// </summary>
        public Double PivotAddedTimeS { get; set; }
    }
}
