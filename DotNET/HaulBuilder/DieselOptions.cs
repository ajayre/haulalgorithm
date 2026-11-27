namespace HaulBuilder
{
    /// <summary>
    /// Tractor and price inputs for the field diesel estimate.
    /// The load-factor defaults live on <see cref="DieselCost"/> and are not settings.
    /// Leave the three tractor inputs unset to skip the estimate.
    /// </summary>
    public sealed class DieselOptions
    {
        /// <summary>
        /// Gets or sets full-load fuel use in gallons per hour.
        /// Null skips the estimate unless another diesel input is set.
        /// </summary>
        public Double? MaxFuelGalPerHour { get; set; }

        /// <summary>
        /// Gets or sets maximum drawbar horsepower from a Nebraska test or the manufacturer.
        /// Null skips the estimate unless another diesel input is set.
        /// </summary>
        public Double? MaxDrawbarHp { get; set; }

        /// <summary>
        /// Gets or sets the diesel price in dollars per US gallon.
        /// Null skips the estimate unless another diesel input is set.
        /// </summary>
        public Double? PricePerGallon { get; set; }

        /// <summary>
        /// Gets or sets the empty tractor-plus-scraper weight in pounds.
        /// When set with haul geometry, haul and return load factors are calculated.
        /// </summary>
        public Double? EmptyRigWeightLb { get; set; }

        /// <summary>Gets or sets a load-factor override for cutting. Null keeps full load.</summary>
        public Double? LoadFactorLoad { get; set; }

        /// <summary>Gets or sets a load-factor override for loaded hauling.</summary>
        public Double? LoadFactorHaul { get; set; }

        /// <summary>Gets or sets a load-factor override for spreading.</summary>
        public Double? LoadFactorDump { get; set; }

        /// <summary>Gets or sets a load-factor override for grading.</summary>
        public Double? LoadFactorGrade { get; set; }

        /// <summary>Gets or sets a load-factor override for the empty return.</summary>
        public Double? LoadFactorReturn { get; set; }

        /// <summary>
        /// Gets or sets the soil surface used to pick rolling resistance.
        /// Null uses the tilled-field default.
        /// </summary>
        public DieselCost.SoilCondition? SoilCondition { get; set; }

        /// <summary>
        /// Gets or sets a rolling-resistance ratio that replaces the soil-surface value.
        /// </summary>
        public Double? RollingResistance { get; set; }

        /// <summary>
        /// Gets or sets bank soil weight in pounds per bank cubic yard.
        /// Null uses the loam default.
        /// </summary>
        public Double? SoilDensityLbPerBcy { get; set; }

        /// <summary>Gets or sets whether cutting is run at part throttle.</summary>
        public Boolean ReducedThrottleLoad { get; set; }

        /// <summary>Gets or sets whether loaded hauling is run at part throttle.</summary>
        public Boolean ReducedThrottleHaul { get; set; }

        /// <summary>Gets or sets whether spreading is run at part throttle.</summary>
        public Boolean ReducedThrottleDump { get; set; }

        /// <summary>Gets or sets whether grading is run at part throttle.</summary>
        public Boolean ReducedThrottleGrade { get; set; }

        /// <summary>Gets or sets whether the empty return is run at part throttle.</summary>
        public Boolean ReducedThrottleReturn { get; set; }

        /// <summary>
        /// Gets or sets engine-on idle hours with no work.
        /// The default is zero: the engine is off whenever the tractor is not driving.
        /// </summary>
        public Double IdleHours { get; set; }
    }
}
