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

        /// <summary>Gets or sets the global optimizer allocation and ledger diagnostics.</summary>
        public HaulOptimizationResult? HaulOptimization { get; set; }

        /// <summary>
        /// Gets or sets the explicit automatic-work omission summary.  This
        /// separates unavoidable material surplus from balanced work left for
        /// pragmatic manual cleanup.
        /// </summary>
        public HaulDeferredWorkSummary? DeferredWorkSummary { get; set; }

        /// <summary>Gets or sets optional multi-route fragment-exchange diagnostics.</summary>
        public HaulMultiRouteExchangeOptimizer.Result? HaulMultiRouteExchange { get; set; }

        /// <summary>Gets or sets capacity-flexible route-consolidation diagnostics.</summary>
        public HaulRouteConsolidationOptimizer.Result? HaulRouteConsolidation { get; set; }

        /// <summary>Gets or sets optional field-wide cut/fill-chain diagnostics.</summary>
        public HaulCycleChainOptimizer.Result? HaulCycleChainOptimization { get; set; }

        /// <summary>
        /// Gets or sets whether the provisional cycle-chain result strictly
        /// improved full production guidance and became the final route plan.
        /// </summary>
        public Boolean HaulCycleChainPromoted { get; set; }

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
        /// Gets or sets the final connected forward-only guidance lifecycle.
        /// </summary>
        public LifecyclePlanner.Result? LifecycleGuidance { get; set; }

        /// <summary>
        /// Gets or sets detailed timing integrated over final guidance geometry.
        /// </summary>
        public GuidanceTiming.Result? DetailedGuidanceTime { get; set; }

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

        /// <summary>Gets or sets loaded-haul distance added by the pivot arm.</summary>
        public Double PivotExtraHaulM { get; set; }

        /// <summary>Gets or sets final-guidance time added by pivot avoidance.</summary>
        public Double PivotAddedTimeS { get; set; }

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
        /// <param name="Order">1-based pass order.</param>
        /// <param name="LoadBcy">Carried bank volume in BCY.</param>
        /// <param name="HaulDistanceM">Haul distance in metres.</param>
        /// <param name="CutStartXy">Cut-start world coordinates.</param>
        /// <param name="CutEndXy">Cut-end world coordinates.</param>
        /// <param name="FillStartXy">Fill-start world coordinates.</param>
        /// <param name="FillEndXy">Fill-end world coordinates.</param>
        /// <param name="TotalCutBcyBefore">Cumulative cut BCY completed before this route.</param>
        /// <param name="TotalFillBcyBefore">Cumulative fill BCY completed before this route.</param>
        /// <param name="RouteTimeS">Estimated time to complete this route in seconds.</param>
        public CutFillPair
            (
            Int32 Order,
            Double LoadBcy,
            Double HaulDistanceM,
            (Double X, Double Y) CutStartXy,
            (Double X, Double Y) CutEndXy,
            (Double X, Double Y) FillStartXy,
            (Double X, Double Y) FillEndXy,
            Double TotalCutBcyBefore,
            Double TotalFillBcyBefore,
            Double RouteTimeS
            )
        {
            // Pair rows keep both scalar metrics and world geometry for exporters and UIs.
            this.Order = Order;
            this.LoadBcy = LoadBcy;
            this.HaulDistanceM = HaulDistanceM;
            this.CutStartXy = CutStartXy;
            this.CutEndXy = CutEndXy;
            this.FillStartXy = FillStartXy;
            this.FillEndXy = FillEndXy;
            this.TotalCutBcyBefore = TotalCutBcyBefore;
            this.TotalFillBcyBefore = TotalFillBcyBefore;
            this.RouteTimeS = RouteTimeS;
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

        /// <summary>Gets or sets deterministic production-guidance hours.</summary>
        public Double DeterministicHours { get; set; }

        /// <summary>Gets or sets availability-adjusted scheduled hours.</summary>
        public Double ScheduledHours { get; set; }

        /// <summary>Gets or sets BCY moved per deterministic guidance hour.</summary>
        public Double DeterministicMoveRateBcyPerHour { get; set; }

        /// <summary>Gets or sets BCY moved per scheduled hour.</summary>
        public Double ScheduledMoveRateBcyPerHour { get; set; }

        /// <summary>Gets or sets automatic BCY moved per valid field acre.</summary>
        public Double MoveVolumeBcyPerAcre { get; set; }

        /// <summary>Gets or sets the cross-field deterministic golden target.</summary>
        public Double GoldenMoveRateTargetBcyPerHour { get; set; }

        /// <summary>Gets or sets achieved percentage of the golden target.</summary>
        public Double GoldenMoveRateAchievementPercent { get; set; }

        /// <summary>Gets or sets whether deterministic productivity meets the target.</summary>
        public Boolean MeetsGoldenMoveRateTarget { get; set; }

        /// <summary>
        /// Gets or sets area rate in acres per hour.
        /// </summary>
        public Double AreaRateAcresPerHour { get; set; }

        /// <summary>
        /// Gets or sets average move distance in yards.
        /// </summary>
        public Double AverageMoveDistanceYards { get; set; }

        /// <summary>
        /// Gets or sets the normalized undirected work-lane axis in degrees.
        /// </summary>
        public Double LaneAxisHeadingDeg { get; set; }

        /// <summary>
        /// Gets or sets pivot detour extra haul in metres.
        /// </summary>
        public Double PivotExtraHaulM { get; set; }

        /// <summary>
        /// Gets or sets pivot detour added time in seconds.
        /// </summary>
        public Double PivotAddedTimeS { get; set; }

        /// <summary>Gets or sets balanced BCY left for manual cleanup.</summary>
        public Double ManualCleanupBcy { get; set; }

        /// <summary>Gets or sets deferred cut BCY, including material surplus.</summary>
        public Double DeferredCutBcy { get; set; }

        /// <summary>Gets or sets deferred fill BCY.</summary>
        public Double DeferredFillBcy { get; set; }
    }

    /// <summary>
    /// User-facing accounting for work excluded from the automatic haul plan.
    /// </summary>
    public sealed class HaulDeferredWorkSummary
    {
        /// <summary>Builds a user-facing summary from final ledger volumes.</summary>
        static public HaulDeferredWorkSummary Create
            (
            Double RequiredCutBcy,
            Double RequiredFillBcy,
            Double DeferredCutBcy,
            Double DeferredFillBcy,
            IReadOnlyList<HaulDeferredWork> DeferredWork
            )
        {
            Double MaterialSurplusCutBcy = Math.Max(
                0.0, RequiredCutBcy - RequiredFillBcy);
            Double CleanupCutBcy = Math.Max(
                0.0, DeferredCutBcy - MaterialSurplusCutBcy);
            Double ManualCleanupBcy = Math.Min(
                CleanupCutBcy, DeferredFillBcy);
            return new HaulDeferredWorkSummary
            {
                RequiredCutBcy = RequiredCutBcy,
                RequiredFillBcy = RequiredFillBcy,
                DeferredCutBcy = DeferredCutBcy,
                DeferredFillBcy = DeferredFillBcy,
                MaterialSurplusCutBcy = MaterialSurplusCutBcy,
                ManualCleanupBcy = ManualCleanupBcy,
                DeferredCutPercent = RequiredCutBcy > 0.0
                    ? 100.0 * DeferredCutBcy / RequiredCutBcy
                    : 0.0,
                DeferredFillPercent = RequiredFillBcy > 0.0
                    ? 100.0 * DeferredFillBcy / RequiredFillBcy
                    : 0.0,
                ManualCleanupPercentOfRequiredFill = RequiredFillBcy > 0.0
                    ? 100.0 * ManualCleanupBcy / RequiredFillBcy
                    : 0.0,
                DeferredEntryCount = DeferredWork.Count,
                DeferredCellCount = DeferredWork
                    .Select(Entry => Entry.Cell)
                    .Distinct()
                    .Count(),
            };
        }

        /// <summary>Gets required cut volume before automatic deferral.</summary>
        public Double RequiredCutBcy { get; init; }

        /// <summary>Gets required fill volume before automatic deferral.</summary>
        public Double RequiredFillBcy { get; init; }

        /// <summary>Gets all deferred cut, including immutable surplus.</summary>
        public Double DeferredCutBcy { get; init; }

        /// <summary>Gets all deferred fill.</summary>
        public Double DeferredFillBcy { get; init; }

        /// <summary>Gets cut which cannot be paired because the field has surplus cut.</summary>
        public Double MaterialSurplusCutBcy { get; init; }

        /// <summary>
        /// Gets balanced cut/fill BCY intentionally left for manual cleanup.
        /// One BCY represents one matched bank-volume unit, not the sum of both
        /// the cut and fill sides.
        /// </summary>
        public Double ManualCleanupBcy { get; init; }

        /// <summary>Gets deferred cut as a percentage of required cut.</summary>
        public Double DeferredCutPercent { get; init; }

        /// <summary>Gets deferred fill as a percentage of required fill.</summary>
        public Double DeferredFillPercent { get; init; }

        /// <summary>Gets balanced manual cleanup as a percentage of required fill.</summary>
        public Double ManualCleanupPercentOfRequiredFill { get; init; }

        /// <summary>Gets the number of deferred depth-layer entries.</summary>
        public Int32 DeferredEntryCount { get; init; }

        /// <summary>Gets the number of distinct cells affected by deferral.</summary>
        public Int32 DeferredCellCount { get; init; }
    }
}
