namespace HaulBuilder
{
    /// <summary>
    /// One well-pad centre in the project coordinate system (metres).
    /// Null overrides inherit the pad defaults on <see cref="HaulBuilderOptions"/>.
    /// The pad crown uses the road height unless <see cref="HeightFt"/> is set.
    /// </summary>
    public sealed class WellPad
    {
        /// <summary>
        /// Initializes a pad at a well centre.
        /// </summary>
        /// <param name="X">Easting in project metres.</param>
        /// <param name="Y">Northing in project metres.</param>
        public WellPad
            (
            Double X,
            Double Y
            )
        {
            this.X = X;
            this.Y = Y;
        }

        /// <summary>Gets the well-centre easting in project metres.</summary>
        public Double X { get; }

        /// <summary>Gets the well-centre northing in project metres.</summary>
        public Double Y { get; }

        /// <summary>Gets or sets an override for the pad height above the field design, in feet.</summary>
        public Double? HeightFt { get; set; }

        /// <summary>Gets or sets an override for the slope away from the well, in percent.</summary>
        public Double? SlopeAwayPct { get; set; }

        /// <summary>Gets or sets an override for the side slope, horizontal over vertical.</summary>
        public Double? SideSlope { get; set; }

        /// <summary>Gets or sets an override for the design-vehicle turn radius, in feet.</summary>
        public Double? DesignTurnRadiusFt { get; set; }

        /// <summary>Gets or sets an override for the clearance added to the turn radius, in feet.</summary>
        public Double? ClearanceFt { get; set; }

        /// <summary>Gets or sets an override for the casing exclusion radius, in feet.</summary>
        public Double? ExclusionRadiusFt { get; set; }

        /// <summary>
        /// Copies the pad and its overrides.
        /// </summary>
        /// <returns>A new pad with the same centre and overrides.</returns>
        public WellPad Clone
            (
            )
        {
            WellPad Copy = new WellPad(X, Y);
            Copy.HeightFt = HeightFt;
            Copy.SlopeAwayPct = SlopeAwayPct;
            Copy.SideSlope = SideSlope;
            Copy.DesignTurnRadiusFt = DesignTurnRadiusFt;
            Copy.ClearanceFt = ClearanceFt;
            Copy.ExclusionRadiusFt = ExclusionRadiusFt;
            return Copy;
        }
    }
}
