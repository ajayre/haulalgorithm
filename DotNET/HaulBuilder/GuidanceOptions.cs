namespace HaulBuilder
{
    /// <summary>
    /// Tuning for Dubins trip guidance. Each default is the rule the measured
    /// guidance was built with. <see cref="TripGuidance"/> reads the values
    /// passed to it. Zero on a radius or fraction limit turns that limit off.
    /// Speeds stay positive, and a correcting trip keeps at least one line.
    /// </summary>
    public sealed class GuidanceOptions
    {
        /// <summary>
        /// Gets or sets the gap, in turning radii, that is still a short joint.
        /// A longer gap is a full Dubins connector and is not trimmed.
        /// </summary>
        public Double ShortJointRadii { get; set; } = 4.0;

        /// <summary>
        /// Gets or sets how much longer than the shortest connector still counts
        /// as a tie, as a fraction of that shortest length.
        /// </summary>
        public Double ConnectorTieFraction { get; set; } = 0.02;

        /// <summary>
        /// Gets or sets how much longer than the shortest connector still counts
        /// as a tie, in turning radii. The wider of this and
        /// <see cref="ConnectorTieFraction"/> is the band. Inside the band, the
        /// path with the least length outside the field wins.
        /// </summary>
        public Double ConnectorTieRadii { get; set; } = 0.5;

        /// <summary>
        /// Gets or sets the heading change, in degrees, below which a short
        /// joint may be a straight steer across.
        /// </summary>
        public Double SteerHeadingDeg { get; set; } = 30.0;

        /// <summary>
        /// Gets or sets how many cut lines, and how many spread lines, one
        /// correcting trip keeps. Further lines wait for a later trip.
        /// </summary>
        public Int32 MaxCorrectingLines { get; set; } = 3;

        /// <summary>
        /// Gets or sets the connector length, divided by productive length,
        /// above which a correcting trip is a leftover and may merge.
        /// </summary>
        public Double LeftoverRatio { get; set; } = 10.0;

        /// <summary>
        /// Gets or sets how close, in turning radii, a later correcting route
        /// must pass to take a leftover fragment.
        /// </summary>
        public Double MergeRouteRadii { get; set; } = 2.0;

        /// <summary>
        /// Gets or sets the end-fragment length, in turning radii, that is
        /// short enough to defer.
        /// </summary>
        public Double EndFragmentSpanRadii { get; set; } = 1.0;

        /// <summary>
        /// Gets or sets the gap, in turning radii, past the next fragment that
        /// defers a short end for a sweep-up trip.
        /// </summary>
        public Double EndFragmentGapRadii { get; set; } = 4.0;

        /// <summary>
        /// Gets or sets the lane length, in turning radii, below which a cut
        /// or spread does not separate connectors. Its cells are still worked.
        /// </summary>
        public Double NearEmptyRadii { get; set; } = 0.5;

        /// <summary>
        /// Gets or sets the lane volume, as a fraction of the bowl, below which
        /// a cut or spread does not separate connectors.
        /// </summary>
        public Double NearEmptyBowlFraction { get; set; } = 0.01;

        /// <summary>
        /// Gets or sets arc speed in miles per hour. Straights keep the haul
        /// clock. This starts at <see cref="TripGuidance.TURN_SPEED_MPH"/> and
        /// is separate from <see cref="HaulBuilderOptions.TurnSpeedMph"/>,
        /// which route order uses.
        /// </summary>
        public Double ArcSpeedMph { get; set; } = TripGuidance.TURN_SPEED_MPH;

        /// <summary>
        /// Gets or sets the speed of a grade lane in miles per hour. This
        /// starts at the fill speed. <see cref="HaulBuilderOptions.GradeSpeedMph"/>
        /// remains the grade-return speed of the production guidance planner.
        /// </summary>
        public Double GradeLaneSpeedMph { get; set; } = Constants.DEFAULT_FILL_SPEED_MPH;
    }
}
