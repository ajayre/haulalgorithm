namespace HaulBuilder
{
    /// <summary>
    /// Advanced tuning for the production lane-first haul optimizer. Fixed
    /// correctness policy—edge ledger completion, grade return, direction-aware
    /// guidance, Poisson flow, and exact validation—is intentionally not optional.
    /// </summary>
    public sealed class HaulOptimizationOptions
    {
        /// <summary>Gets or sets the maximum source-lane slices per work kind and cycle.</summary>
        public Int32 MaxLaneSlicesPerCycle { get; set; } = 10;

        /// <summary>Gets or sets the unchanged Poisson-flow deviation weight.</summary>
        public Double FlowPenaltyWeight { get; set; } = 1.0;

        /// <summary>Gets or sets the residual-edge lane-slice bound.</summary>
        public Int32 EdgeMaxLaneSlicesPerCycle { get; set; } = 12;

        /// <summary>Gets or sets whether capacity-flexible route consolidation runs.</summary>
        public Boolean EnableRouteConsolidation { get; set; } = true;

        /// <summary>Gets or sets the maximum routes pooled in one consolidation transaction.</summary>
        public Int32 RouteConsolidationGroupSize { get; set; } = 8;

        /// <summary>Gets or sets the maximum cycles removed by consolidation.</summary>
        public Int32 RouteConsolidationMaximumCycles { get; set; } = 50;

        /// <summary>Gets or sets deterministic equal-load fill-pair refinement rounds.</summary>
        public Int32 PairRefinementRounds { get; set; } = 8;

        /// <summary>Gets or sets whether the slower closed multi-route exchange runs.</summary>
        public Boolean EnableMultiRouteExchange { get; set; }

        /// <summary>Gets or sets bounded multi-route exchange rounds.</summary>
        public Int32 MultiRouteExchangeRounds { get; set; } = 2;

        /// <summary>Gets or sets maximum routes in one multi-route exchange.</summary>
        public Int32 MultiRouteExchangeGroupSize { get; set; } = 8;

        /// <summary>Gets or sets maximum committed multi-route exchanges.</summary>
        public Int32 MultiRouteExchangeMaximumGroups { get; set; } = 100;

        /// <summary>Gets or sets minimum compact-guidance improvement per exchange.</summary>
        public Double MultiRouteExchangeMinimumImprovementM { get; set; } = 1.0;

        /// <summary>
        /// Gets or sets whether exact-gated field-wide fill-chain refinement runs.
        /// Disabled by default because promotion requires two full guidance builds;
        /// enable when trading build time for lower deterministic drive time.
        /// </summary>
        public Boolean EnableCycleChainOptimization { get; set; }

        /// <summary>Gets or sets field-wide fill-chain rounds.</summary>
        public Int32 CycleChainRounds { get; set; } = 1;

        /// <summary>Gets or sets exact candidates checked per route.</summary>
        public Int32 CycleChainCandidatesPerRoute { get; set; } = 1;

        /// <summary>Gets or sets minimum production-objective saving per chain exchange.</summary>
        public Double CycleChainMinimumImprovementSeconds { get; set; } = Constants.QUARTER_SCALE;

        /// <summary>Gets or sets route relocation search window; zero disables relocation.</summary>
        public Int32 RouteReorderWindow { get; set; } = 200;

        /// <summary>
        /// Gets or sets whether routes are driven as one-strip interior loads,
        /// then other interior loads, then the field-edge batch. The reorder
        /// window stays inside a band. On by default. Off is one travel sort
        /// of every load.
        /// </summary>
        public Boolean EnableEarlyFullStripBanding { get; set; } = true;

        /// <summary>
        /// Gets or sets whether the next load is chosen by metres. The score is
        /// the empty hop plus the turn arc. On by default. Off scores the same
        /// hop in seconds, with the turn at turn speed.
        /// </summary>
        public Boolean OptimizeRouteOrderByDistance { get; set; } = true;

        /// <summary>
        /// Gets or sets how many recent haul loads form the mean that moves
        /// hauling to clusters. The default is the last 20 loads. Clusters
        /// starts on the first found-order haul where that mean is below
        /// <see cref="ClusterModeLoadFraction"/> of bowl capacity.
        /// </summary>
        public Int32 ClusterModeAverageWindow { get; set; } = 20;

        /// <summary>
        /// Gets or sets the share of bowl capacity below which the trailing
        /// mean moves hauling to clusters. The default is one half.
        /// </summary>
        public Double ClusterModeLoadFraction { get; set; } = 0.5;

        /// <summary>Copies all resolved tuning values.</summary>
        internal HaulOptimizationOptions Clone() => new HaulOptimizationOptions
        {
            MaxLaneSlicesPerCycle = MaxLaneSlicesPerCycle,
            FlowPenaltyWeight = FlowPenaltyWeight,
            EdgeMaxLaneSlicesPerCycle = EdgeMaxLaneSlicesPerCycle,
            EnableRouteConsolidation = EnableRouteConsolidation,
            RouteConsolidationGroupSize = RouteConsolidationGroupSize,
            RouteConsolidationMaximumCycles = RouteConsolidationMaximumCycles,
            PairRefinementRounds = PairRefinementRounds,
            EnableMultiRouteExchange = EnableMultiRouteExchange,
            MultiRouteExchangeRounds = MultiRouteExchangeRounds,
            MultiRouteExchangeGroupSize = MultiRouteExchangeGroupSize,
            MultiRouteExchangeMaximumGroups = MultiRouteExchangeMaximumGroups,
            MultiRouteExchangeMinimumImprovementM = MultiRouteExchangeMinimumImprovementM,
            EnableCycleChainOptimization = EnableCycleChainOptimization,
            CycleChainRounds = CycleChainRounds,
            CycleChainCandidatesPerRoute = CycleChainCandidatesPerRoute,
            CycleChainMinimumImprovementSeconds = CycleChainMinimumImprovementSeconds,
            RouteReorderWindow = RouteReorderWindow,
            EnableEarlyFullStripBanding = EnableEarlyFullStripBanding,
            OptimizeRouteOrderByDistance = OptimizeRouteOrderByDistance,
            ClusterModeAverageWindow = ClusterModeAverageWindow,
            ClusterModeLoadFraction = ClusterModeLoadFraction,
        };
    }
}
