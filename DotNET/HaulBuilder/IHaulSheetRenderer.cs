namespace HaulBuilder
{
    /// <summary>
    /// Draws the pictures a haul run can attach to its result.
    /// A data-processing caller leaves this unset, and the PNG and GIF fields stay empty.
    /// </summary>
    public interface IHaulSheetRenderer
    {
        /// <summary>
        /// Writes the cut-and-fill map of the grid as it stands, under the key <c>main</c>.
        /// </summary>
        /// <param name="Result">Run that receives the PNG bytes.</param>
        /// <param name="Grid">Field grid to draw.</param>
        /// <param name="Flow">Flow field drawn as arrows.</param>
        /// <param name="Options">Pixel size and arrow stride.</param>
        void WriteFlowMap
            (
            HaulBuilderResult Result,
            Grid Grid,
            FlowField Flow,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes the worst ponding checkpoint, under the key <c>ponding</c>.
        /// </summary>
        /// <param name="Result">Run that receives the PNG bytes.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="PondingResult">Audit whose worst checkpoint is drawn.</param>
        /// <param name="Options">Map scale.</param>
        void WritePondingMap
            (
            HaulBuilderResult Result,
            Grid Grid,
            PondingRunResult PondingResult,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes <c>main</c> again with the zone walk drawn on it, under <c>main_with_zones</c>.
        /// </summary>
        /// <param name="Result">Run whose <c>main</c> image is the base.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Steps">Zone sequence to overlay.</param>
        /// <param name="Options">Pixel size.</param>
        void WriteZoneOverlay
            (
            HaulBuilderResult Result,
            Grid Grid,
            IReadOnlyList<ZoneStep> Steps,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes the pass-pair map, under the key <c>cut_fill_pairs</c>.
        /// </summary>
        /// <param name="Result">Run that receives the PNG bytes.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Passes">Passes to draw, in route order.</param>
        /// <param name="Options">Pixel size.</param>
        void WriteCutFillPairMap
            (
            HaulBuilderResult Result,
            Grid Grid,
            IReadOnlyList<Pass> Passes,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes the guidance map, under the key <c>guidance</c>.
        /// </summary>
        /// <param name="Result">Run that receives the PNG bytes.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Points">Waypoints to draw.</param>
        /// <param name="Options">Pixel size and blade width.</param>
        void WriteGuidanceMap
            (
            HaulBuilderResult Result,
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes one guidance PNG for each of the first requested routes.
        /// </summary>
        /// <param name="Result">Run whose lifecycle and route supply the pictures.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Options">How many routes to draw, and the blade width.</param>
        void WriteRouteGuidanceImages
            (
            HaulBuilderResult Result,
            Grid Grid,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes randomly chosen three-route guidance PNGs.
        /// </summary>
        /// <param name="Result">Run whose lifecycle and route supply the pictures.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Options">How many triplets to draw, and the random seed.</param>
        void WriteRouteTriplets
            (
            HaulBuilderResult Result,
            Grid Grid,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes randomly chosen single-route guidance PNGs.
        /// </summary>
        /// <param name="Result">Run that receives the PNG bytes.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Points">Guidance waypoints grouped by pass.</param>
        /// <param name="Options">How many routes to draw, and the random seed.</param>
        void WritePairSamples
            (
            HaulBuilderResult Result,
            Grid Grid,
            IReadOnlyList<GuidancePoint> Points,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes the leveling GIF and, when asked, one PNG per frame.
        /// </summary>
        /// <param name="Result">Run that receives the GIF and frame bytes.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Route">Route whose passes are replayed.</param>
        /// <param name="Options">Animation flags, stride, scale, and mode.</param>
        void WriteAnimations
            (
            HaulBuilderResult Result,
            Grid Grid,
            Route Route,
            HaulBuilderOptions Options
            );

        /// <summary>
        /// Writes the pivot position PNGs and, when asked, the pivot leveling GIF.
        /// </summary>
        /// <param name="Result">Run that receives the pivot pictures.</param>
        /// <param name="Grid">Field grid used by the leveling animation.</param>
        /// <param name="Route">Route whose passes are replayed.</param>
        /// <param name="PivotGrid">Pivot grid used by the position sheets.</param>
        /// <param name="PivotPasses">Passes in the pivot module's own cell type.</param>
        /// <param name="Plan">Pivot schedule to draw.</param>
        /// <param name="Options">Pivot picture flags, scale, and mode.</param>
        void WritePivotImages
            (
            HaulBuilderResult Result,
            Grid Grid,
            Route Route,
            Pivot.Grid PivotGrid,
            IReadOnlyList<Pivot.Pass> PivotPasses,
            Pivot.PivotPlan Plan,
            HaulBuilderOptions Options
            );
    }
}
