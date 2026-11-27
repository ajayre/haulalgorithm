using System.Globalization;

namespace HaulBuilder
{
    /// <summary>
    /// Planning-grade diesel cost for one field.
    /// Phase hours come from the haul plan. The caller supplies the tractor's
    /// full-load fuel rate, drawbar horsepower, and the price per gallon.
    /// Load factors and the fuel curve are the hard-coded Section 5 assumptions.
    /// </summary>
    public static class DieselCost
    {
        /// <summary>Share of full-load fuel burned at zero load, full throttle.</summary>
        public const Double BASE_FRACTION = 0.30;

        /// <summary>Cutting load factor. Cutting is taken at full drawbar load.</summary>
        public const Double X_LOAD = 1.00;

        /// <summary>Default loaded-haul load factor.</summary>
        public const Double X_HAUL = 0.60;

        /// <summary>Default spreading load factor.</summary>
        public const Double X_DUMP = 0.55;

        /// <summary>Default grading load factor.</summary>
        public const Double X_GRADE = 0.60;

        /// <summary>Default empty-return load factor.</summary>
        public const Double X_RETURN = 0.35;

        /// <summary>Low-idle fuel as a fraction of the full-load rate.</summary>
        public const Double IDLE_FRACTION = 0.10;

        /// <summary>Rolling resistance for a typical worked field, as a weight ratio.</summary>
        public const Double ROLLING_RESISTANCE = 0.06;

        /// <summary>Rolling resistance on firm, packed ground.</summary>
        public const Double ROLLING_RESISTANCE_FIRM = 0.04;

        /// <summary>Rolling resistance on tilled ground. Same value as the default.</summary>
        public const Double ROLLING_RESISTANCE_TILLED = 0.06;

        /// <summary>Rolling resistance on loose, wet, or fresh fill.</summary>
        public const Double ROLLING_RESISTANCE_SOFT = 0.10;

        /// <summary>Bank weight of common loam, in pounds per bank cubic yard.</summary>
        public const Double SOIL_DENSITY_LB_PER_BCY = 2700.0;

        /// <summary>
        /// Drawbar power available in the field divided by the concrete-rated drawbar power.
        /// </summary>
        public const Double TRACTION_DERATE = 0.85;

        /// <summary>Fuel reduction when a light load is run at part throttle.</summary>
        public const Double THROTTLE_BACK_SAVINGS = 0.15;

        /// <summary>Pounds times miles per hour, per horsepower.</summary>
        public const Double HP_CONVERSION = 375.0;

        /// <summary>Feet per minute in one mile per hour.</summary>
        public const Double FT_PER_MIN_PER_MPH = 88.0;

        /// <summary>Minutes in one hour. The speed formula uses minutes.</summary>
        public const Double MINUTES_PER_HOUR = 60.0;

        /// <summary>Lowest calculated load factor.</summary>
        public const Double X_MIN = 0.10;

        /// <summary>Highest calculated load factor.</summary>
        public const Double X_MAX = 1.00;

        /// <summary>Part throttle applies only below this load factor.</summary>
        public const Double THROTTLE_BACK_MAX_LOAD = 0.70;

        /// <summary>Lowest plausible full-load gallons per drawbar horsepower-hour.</summary>
        public const Double MIN_FUEL_GAL_PER_HP_HOUR = 0.04;

        /// <summary>Highest plausible full-load gallons per drawbar horsepower-hour.</summary>
        public const Double MAX_FUEL_GAL_PER_HP_HOUR = 0.10;

        /// <summary>Low end of the expected average fuel rate, as a percent of full load.</summary>
        public const Double MIN_AVERAGE_LOAD_PERCENT = 55.0;

        /// <summary>High end of the expected average fuel rate, as a percent of full load.</summary>
        public const Double MAX_AVERAGE_LOAD_PERCENT = 90.0;

        /// <summary>How hard the tractor is working, as a fraction of maximum drawbar power.</summary>
        public enum Phase
        {
            /// <summary>Blade-down cutting and loading.</summary>
            Load,

            /// <summary>Loaded travel from the cut to the fill.</summary>
            Haul,

            /// <summary>Blade-down spreading.</summary>
            Dump,

            /// <summary>Blade-down finish pass over the fill.</summary>
            Grade,

            /// <summary>Empty travel back to the next cut.</summary>
            Return,
        }

        /// <summary>Where a phase load factor came from.</summary>
        public enum LoadFactorSource
        {
            /// <summary>The hard-coded factor for that phase.</summary>
            Default,

            /// <summary>Pull from rig weight, soil, grade, and speed.</summary>
            Calculated,

            /// <summary>A value supplied for that phase.</summary>
            Override,
        }

        /// <summary>Ground used to choose a rolling-resistance ratio.</summary>
        public enum SoilCondition
        {
            /// <summary>Firm, dry, packed surface.</summary>
            Firm,

            /// <summary>Typical worked field.</summary>
            Tilled,

            /// <summary>Loose, wet, or freshly placed fill.</summary>
            Soft,
        }

        /// <summary>
        /// Driving hours in the five diesel phases.
        /// Idle and the field-efficiency gap are not included.
        /// </summary>
        public sealed class PhaseHours
        {
            /// <summary>Initializes one phase-hour total.</summary>
            /// <param name="LoadingHours">Cutting and loading hours.</param>
            /// <param name="HaulingHours">Loaded travel hours.</param>
            /// <param name="DumpingHours">Spreading hours.</param>
            /// <param name="GradingHours">Finish-pass hours.</param>
            /// <param name="ReturningHours">Empty-return hours.</param>
            public PhaseHours
                (
                Double LoadingHours,
                Double HaulingHours,
                Double DumpingHours,
                Double GradingHours,
                Double ReturningHours
                )
            {
                this.LoadingHours = LoadingHours;
                this.HaulingHours = HaulingHours;
                this.DumpingHours = DumpingHours;
                this.GradingHours = GradingHours;
                this.ReturningHours = ReturningHours;
            }

            /// <summary>Gets cutting and loading hours.</summary>
            public Double LoadingHours { get; }

            /// <summary>Gets loaded travel hours.</summary>
            public Double HaulingHours { get; }

            /// <summary>Gets spreading hours.</summary>
            public Double DumpingHours { get; }

            /// <summary>Gets finish-pass hours.</summary>
            public Double GradingHours { get; }

            /// <summary>Gets empty-return hours.</summary>
            public Double ReturningHours { get; }
        }

        /// <summary>
        /// One cut-to-fill cycle used to calculate haul and return load.
        /// Distance is one way, in feet. Times are in hours.
        /// </summary>
        public sealed class Cycle
        {
            /// <summary>Initializes one cycle.</summary>
            /// <param name="DistanceFt">One-way haul distance in feet.</param>
            /// <param name="HaulTimeHours">Loaded travel time in hours.</param>
            /// <param name="ReturnTimeHours">Empty return time in hours.</param>
            /// <param name="SoilVolumeBcy">Bank cubic yards in the bowl.</param>
            /// <param name="CutElevationFt">Cut elevation in feet. Null leaves the grade at zero.</param>
            /// <param name="FillElevationFt">Fill elevation in feet. Null leaves the grade at zero.</param>
            public Cycle
                (
                Double DistanceFt,
                Double HaulTimeHours,
                Double ReturnTimeHours,
                Double SoilVolumeBcy,
                Double? CutElevationFt,
                Double? FillElevationFt
                )
            {
                this.DistanceFt = DistanceFt;
                this.HaulTimeHours = HaulTimeHours;
                this.ReturnTimeHours = ReturnTimeHours;
                this.SoilVolumeBcy = SoilVolumeBcy;
                this.CutElevationFt = CutElevationFt;
                this.FillElevationFt = FillElevationFt;
            }

            /// <summary>Gets the one-way distance in feet.</summary>
            public Double DistanceFt { get; }

            /// <summary>Gets loaded travel time in hours.</summary>
            public Double HaulTimeHours { get; }

            /// <summary>Gets empty return time in hours.</summary>
            public Double ReturnTimeHours { get; }

            /// <summary>Gets the bowl load in bank cubic yards.</summary>
            public Double SoilVolumeBcy { get; }

            /// <summary>Gets the cut elevation in feet, when the plan has one.</summary>
            public Double? CutElevationFt { get; }

            /// <summary>Gets the fill elevation in feet, when the plan has one.</summary>
            public Double? FillElevationFt { get; }
        }

        /// <summary>
        /// Inputs for one estimate. Phase hours are the plan. Tractor numbers are the caller.
        /// </summary>
        public sealed class Request
        {
            /// <summary>Gets or sets full-load fuel use in gallons per hour.</summary>
            public Double MaxFuelGalPerHour { get; set; }

            /// <summary>Gets or sets rated drawbar horsepower.</summary>
            public Double MaxDrawbarHp { get; set; }

            /// <summary>Gets or sets the price in dollars per gallon.</summary>
            public Double PricePerGallon { get; set; }

            /// <summary>Gets or sets cutting hours.</summary>
            public Double LoadingHours { get; set; }

            /// <summary>Gets or sets loaded-haul hours.</summary>
            public Double HaulingHours { get; set; }

            /// <summary>Gets or sets spreading hours.</summary>
            public Double DumpingHours { get; set; }

            /// <summary>Gets or sets grading hours.</summary>
            public Double GradingHours { get; set; }

            /// <summary>Gets or sets empty-return hours.</summary>
            public Double ReturningHours { get; set; }

            /// <summary>Gets or sets engine-on idle hours. Zero means the engine is off.</summary>
            public Double IdleHours { get; set; }

            /// <summary>Gets or sets empty rig weight in pounds. Null keeps the default haul factors.</summary>
            public Double? EmptyRigWeightLb { get; set; }

            /// <summary>Gets or sets a cutting load-factor override.</summary>
            public Double? LoadFactorLoad { get; set; }

            /// <summary>Gets or sets a loaded-haul load-factor override.</summary>
            public Double? LoadFactorHaul { get; set; }

            /// <summary>Gets or sets a spreading load-factor override.</summary>
            public Double? LoadFactorDump { get; set; }

            /// <summary>Gets or sets a grading load-factor override.</summary>
            public Double? LoadFactorGrade { get; set; }

            /// <summary>Gets or sets an empty-return load-factor override.</summary>
            public Double? LoadFactorReturn { get; set; }

            /// <summary>Gets or sets the soil surface. Null uses the default rolling resistance.</summary>
            public SoilCondition? SoilCondition { get; set; }

            /// <summary>Gets or sets a rolling-resistance override.</summary>
            public Double? RollingResistanceOverride { get; set; }

            /// <summary>Gets or sets a soil-density override in pounds per bank cubic yard.</summary>
            public Double? SoilDensityLbPerBcy { get; set; }

            /// <summary>Gets or sets part throttle while cutting.</summary>
            public Boolean ReducedThrottleLoad { get; set; }

            /// <summary>Gets or sets part throttle while hauling loaded.</summary>
            public Boolean ReducedThrottleHaul { get; set; }

            /// <summary>Gets or sets part throttle while spreading.</summary>
            public Boolean ReducedThrottleDump { get; set; }

            /// <summary>Gets or sets part throttle while grading.</summary>
            public Boolean ReducedThrottleGrade { get; set; }

            /// <summary>Gets or sets part throttle on the empty return.</summary>
            public Boolean ReducedThrottleReturn { get; set; }

            /// <summary>
            /// Gets or sets haul cycles. Null or empty keeps the default haul and return factors
            /// unless a phase override is set.
            /// </summary>
            public IReadOnlyList<Cycle>? Cycles { get; set; }
        }

        /// <summary>Fuel and cost for one phase.</summary>
        public sealed class PhaseEstimate
        {
            /// <summary>Initializes one phase line.</summary>
            /// <param name="Phase">Which duty.</param>
            /// <param name="Hours">Driving hours.</param>
            /// <param name="LoadFactor">Fraction of maximum drawbar power.</param>
            /// <param name="Source">Where the load factor came from.</param>
            /// <param name="ReducedThrottle">True when part-throttle savings were applied.</param>
            /// <param name="FuelFraction">Gallons per hour divided by the full-load rate.</param>
            /// <param name="Gallons">Diesel for this phase.</param>
            /// <param name="Cost">Dollars for this phase.</param>
            public PhaseEstimate
                (
                Phase Phase,
                Double Hours,
                Double LoadFactor,
                LoadFactorSource Source,
                Boolean ReducedThrottle,
                Double FuelFraction,
                Double Gallons,
                Double Cost
                )
            {
                this.Phase = Phase;
                this.Hours = Hours;
                this.LoadFactor = LoadFactor;
                this.Source = Source;
                this.ReducedThrottle = ReducedThrottle;
                this.FuelFraction = FuelFraction;
                this.Gallons = Gallons;
                this.Cost = Cost;
            }

            /// <summary>Gets the duty.</summary>
            public Phase Phase { get; }

            /// <summary>Gets driving hours.</summary>
            public Double Hours { get; }

            /// <summary>Gets the load factor before any part-throttle reduction.</summary>
            public Double LoadFactor { get; }

            /// <summary>Gets where the load factor came from.</summary>
            public LoadFactorSource Source { get; }

            /// <summary>Gets whether part-throttle savings were applied.</summary>
            public Boolean ReducedThrottle { get; }

            /// <summary>Gets gallons per hour as a fraction of the full-load rate.</summary>
            public Double FuelFraction { get; }

            /// <summary>Gets diesel gallons.</summary>
            public Double Gallons { get; }

            /// <summary>Gets cost in dollars.</summary>
            public Double Cost { get; }
        }

        /// <summary>Diesel gallons, cost, and the assumptions behind them.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one estimate.</summary>
            /// <param name="Blocked">True when the plan had no work or the tractor inputs cannot be priced.</param>
            /// <param name="Phases">One line per duty. Empty when blocked.</param>
            /// <param name="IdleHours">Engine-on idle hours.</param>
            /// <param name="IdleGallons">Idle diesel.</param>
            /// <param name="IdleCost">Idle cost in dollars.</param>
            /// <param name="TotalGallons">Phase gallons plus idle.</param>
            /// <param name="TotalCost">Dollars.</param>
            /// <param name="TotalHours">Driving hours plus idle.</param>
            /// <param name="AverageFuelGalPerHour">Total gallons divided by total hours.</param>
            /// <param name="AverageLoadPercent">Average fuel rate as a percent of the full-load rate.</param>
            /// <param name="Assumptions">Section 5 values in effect.</param>
            /// <param name="Warnings">Sanity-check messages.</param>
            public Result
                (
                Boolean Blocked,
                IReadOnlyList<PhaseEstimate> Phases,
                Double IdleHours,
                Double IdleGallons,
                Double IdleCost,
                Double TotalGallons,
                Double TotalCost,
                Double TotalHours,
                Double AverageFuelGalPerHour,
                Double AverageLoadPercent,
                IReadOnlyList<String> Assumptions,
                IReadOnlyList<String> Warnings
                )
            {
                this.Blocked = Blocked;
                this.Phases = Phases;
                this.IdleHours = IdleHours;
                this.IdleGallons = IdleGallons;
                this.IdleCost = IdleCost;
                this.TotalGallons = TotalGallons;
                this.TotalCost = TotalCost;
                this.TotalHours = TotalHours;
                this.AverageFuelGalPerHour = AverageFuelGalPerHour;
                this.AverageLoadPercent = AverageLoadPercent;
                this.Assumptions = Assumptions;
                this.Warnings = Warnings;
            }

            /// <summary>Gets whether pricing was refused.</summary>
            public Boolean Blocked { get; }

            /// <summary>Gets one line per duty.</summary>
            public IReadOnlyList<PhaseEstimate> Phases { get; }

            /// <summary>Gets engine-on idle hours.</summary>
            public Double IdleHours { get; }

            /// <summary>Gets idle gallons.</summary>
            public Double IdleGallons { get; }

            /// <summary>Gets idle cost in dollars.</summary>
            public Double IdleCost { get; }

            /// <summary>Gets total gallons.</summary>
            public Double TotalGallons { get; }

            /// <summary>Gets total cost in dollars.</summary>
            public Double TotalCost { get; }

            /// <summary>Gets driving hours plus idle hours.</summary>
            public Double TotalHours { get; }

            /// <summary>Gets total gallons divided by total hours.</summary>
            public Double AverageFuelGalPerHour { get; }

            /// <summary>
            /// Gets the average fuel rate as a percent of the full-load rate.
            /// This follows the fuel curve, so it is not the average load factor.
            /// </summary>
            public Double AverageLoadPercent { get; }

            /// <summary>Gets the hard-coded assumptions that were in effect.</summary>
            public IReadOnlyList<String> Assumptions { get; }

            /// <summary>Gets sanity-check messages. Empty when none fired.</summary>
            public IReadOnlyList<String> Warnings { get; }

            /// <summary>Finds one phase line.</summary>
            /// <param name="Phase">The duty to find.</param>
            /// <returns>The line, or null when the estimate was blocked.</returns>
            public PhaseEstimate? Find
                (
                Phase Phase
                )
            {
                for (Int32 Index = 0; Index < Phases.Count; Index += 1)
                {
                    if (Phases[Index].Phase == Phase)
                    {
                        return Phases[Index];
                    }
                }

                return null;
            }
        }

        /// <summary>
        /// Rejects a partial or impossible diesel setup.
        /// An options object with no diesel fields set is left alone.
        /// </summary>
        /// <param name="Options">Tractor and price inputs.</param>
        /// <exception cref="ArgumentNullException">Thrown when options is null.</exception>
        /// <exception cref="ArgumentException">Thrown when some diesel inputs are set and the three tractor inputs are not.</exception>
        /// <exception cref="ArgumentOutOfRangeException">Thrown when a supplied number is outside its range.</exception>
        public static void Validate
            (
            DieselOptions Options
            )
        {
            if (Options == null)
            {
                throw new ArgumentNullException(nameof(Options));
            }

            if (!IsConfigured(Options))
            {
                return;
            }

            if (!Options.MaxFuelGalPerHour.HasValue
                || !Options.MaxDrawbarHp.HasValue
                || !Options.PricePerGallon.HasValue)
            {
                throw new ArgumentException(
                    "Diesel cost needs a max-load fuel rate, max drawbar horsepower, and a diesel price.");
            }

            Positive(Options.MaxFuelGalPerHour.Value, nameof(Options.MaxFuelGalPerHour));
            Positive(Options.MaxDrawbarHp.Value, nameof(Options.MaxDrawbarHp));
            NonNegative(Options.PricePerGallon.Value, nameof(Options.PricePerGallon));
            NonNegative(Options.IdleHours, nameof(Options.IdleHours));
            if (Options.EmptyRigWeightLb.HasValue)
            {
                Positive(Options.EmptyRigWeightLb.Value, nameof(Options.EmptyRigWeightLb));
            }

            UnitInterval(Options.LoadFactorLoad, nameof(Options.LoadFactorLoad));
            UnitInterval(Options.LoadFactorHaul, nameof(Options.LoadFactorHaul));
            UnitInterval(Options.LoadFactorDump, nameof(Options.LoadFactorDump));
            UnitInterval(Options.LoadFactorGrade, nameof(Options.LoadFactorGrade));
            UnitInterval(Options.LoadFactorReturn, nameof(Options.LoadFactorReturn));
            if (Options.RollingResistance.HasValue)
            {
                NonNegative(Options.RollingResistance.Value, nameof(Options.RollingResistance));
            }

            if (Options.SoilDensityLbPerBcy.HasValue)
            {
                Positive(Options.SoilDensityLbPerBcy.Value, nameof(Options.SoilDensityLbPerBcy));
            }
        }

        /// <summary>
        /// True when the three tractor inputs are all present.
        /// Call <see cref="Validate"/> first.
        /// </summary>
        /// <param name="Options">Tractor and price inputs.</param>
        /// <returns>True when an estimate should be attached to the plan.</returns>
        public static Boolean RequestsEstimate
            (
            DieselOptions Options
            )
        {
            if (Options == null)
            {
                return false;
            }

            return Options.MaxFuelGalPerHour.HasValue
                && Options.MaxDrawbarHp.HasValue
                && Options.PricePerGallon.HasValue;
        }

        /// <summary>
        /// Sums guidance phase seconds into the five diesel duties.
        /// In-pass blade-up transit is loaded haul. The trip to the next cut is the empty return.
        /// The field-efficiency allowance is not in these seconds.
        /// </summary>
        /// <param name="PhaseSeconds">Motion and event seconds by lifecycle phase.</param>
        /// <returns>Driving hours. Missing phases count as zero.</returns>
        public static PhaseHours HoursFromGuidance
            (
            IReadOnlyDictionary<LifecyclePlanner.Phase, Double> PhaseSeconds
            )
        {
            if (PhaseSeconds == null)
            {
                throw new ArgumentNullException(nameof(PhaseSeconds));
            }

            // ExitFill stays with the haul: the lifecycle still treats that leg as loaded.
            Double HaulSeconds = Seconds(PhaseSeconds, LifecyclePlanner.Phase.ExitCut)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.LineupFill)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.Transit)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.ExitFill);
            Double GradeSeconds = Seconds(PhaseSeconds, LifecyclePlanner.Phase.Grade)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.LineupGrade)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.ExitGrade)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.TurnToGrade);
            Double ReturnSeconds = Seconds(PhaseSeconds, LifecyclePlanner.Phase.TransitToNextCut)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.LineupCut)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.ManualAssist)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.Start)
                + Seconds(PhaseSeconds, LifecyclePlanner.Phase.End);
            return new PhaseHours(
                Hours(Seconds(PhaseSeconds, LifecyclePlanner.Phase.Cut)),
                Hours(HaulSeconds),
                Hours(Seconds(PhaseSeconds, LifecyclePlanner.Phase.Fill)),
                Hours(GradeSeconds),
                Hours(ReturnSeconds));
        }

        /// <summary>
        /// Sums the speed-model times into diesel duties.
        /// That model has no grade pass, and its other-time bucket is the
        /// field-efficiency gap, which burns no fuel while the engine is off.
        /// </summary>
        /// <param name="Time">Speed-model breakdown. Null yields zeros.</param>
        /// <returns>Driving hours with grading at zero.</returns>
        public static PhaseHours HoursFromTimeModel
            (
            TimeResult? Time
            )
        {
            if (Time == null)
            {
                return new PhaseHours(0.0, 0.0, 0.0, 0.0, 0.0);
            }

            return new PhaseHours(
                Hours(Time.LoadingTimeS),
                Hours(Time.HaulingTimeS),
                Hours(Time.DumpingTimeS),
                0.0,
                Hours(Time.ReturningTimeS));
        }

        /// <summary>
        /// Builds one plan-average cycle from total haul and return distance and time.
        /// The fuel model uses a single one-way distance, so the return time is
        /// the time that reproduces the plan's return speed over the haul distance.
        /// </summary>
        /// <param name="HaulDistanceFt">Total loaded-haul distance in feet.</param>
        /// <param name="HaulTimeHours">Total loaded-haul time in hours.</param>
        /// <param name="ReturnDistanceFt">Total empty-return distance in feet.</param>
        /// <param name="ReturnTimeHours">Total empty-return time in hours.</param>
        /// <param name="MeanVolumeBcy">Mean bowl load in bank cubic yards.</param>
        /// <param name="Grade">Rise over run on the loaded haul. Null means level.</param>
        /// <returns>The average cycle, or null when neither leg has distance and time.</returns>
        internal static Cycle? AverageCycle
            (
            Double HaulDistanceFt,
            Double HaulTimeHours,
            Double ReturnDistanceFt,
            Double ReturnTimeHours,
            Double MeanVolumeBcy,
            Double? Grade
            )
        {
            if (!Double.IsFinite(MeanVolumeBcy) || (MeanVolumeBcy < 0.0))
            {
                return null;
            }

            Boolean HaulUsable = UsableLeg(HaulDistanceFt, HaulTimeHours);
            Boolean ReturnUsable = UsableLeg(ReturnDistanceFt, ReturnTimeHours);
            if (!HaulUsable && !ReturnUsable)
            {
                return null;
            }

            // Prefer the loaded-haul distance. A return-only average still needs a length.
            Double DistanceFt = HaulUsable ? HaulDistanceFt : ReturnDistanceFt;
            Double HaulHours = HaulUsable ? HaulTimeHours : 0.0;
            Double ReturnHours = 0.0;
            if (ReturnUsable)
            {
                Double ReturnMph = ReturnDistanceFt
                    / (ReturnTimeHours * MINUTES_PER_HOUR * FT_PER_MIN_PER_MPH);
                if (ReturnMph > 0.0)
                {
                    ReturnHours = DistanceFt
                        / (ReturnMph * MINUTES_PER_HOUR * FT_PER_MIN_PER_MPH);
                }
            }

            Double? CutFt = null;
            Double? FillFt = null;
            if (Grade.HasValue && Double.IsFinite(Grade.Value) && (DistanceFt > 0.0))
            {
                // Store the grade as a rise over this same distance.
                CutFt = 0.0;
                FillFt = Grade.Value * DistanceFt;
            }

            return new Cycle(
                DistanceFt,
                HaulHours,
                ReturnHours,
                MeanVolumeBcy,
                CutFt,
                FillFt);
        }

        /// <summary>
        /// Prices one plan.
        /// </summary>
        /// <param name="Request">Tractor inputs and phase hours.</param>
        /// <returns>Gallons, dollars, and any warnings. Blocked when the plan has no driving.</returns>
        public static Result Estimate
            (
            Request Request
            )
        {
            if (Request == null)
            {
                throw new ArgumentNullException(nameof(Request));
            }

            String ResistanceNote;
            Double Resistance = ResolveRollingResistance(Request, out ResistanceNote);
            String DensityNote;
            Double Density = ResolveDensity(Request, out DensityNote);
            IReadOnlyList<String> Assumptions = BuildAssumptions(
                Resistance, ResistanceNote, Density, DensityNote);
            List<String> Warnings = new List<String>();

            if (!HoursAreUsable(Request))
            {
                Warnings.Add("Phase hours must be finite and zero or greater.");
                return Block(Assumptions, Warnings);
            }

            if (!Double.IsFinite(Request.IdleHours) || (Request.IdleHours < 0.0))
            {
                Warnings.Add("Idle hours must be finite and zero or greater.");
                return Block(Assumptions, Warnings);
            }

            if (!Double.IsFinite(Request.MaxFuelGalPerHour)
                || (Request.MaxFuelGalPerHour <= 0.0)
                || !Double.IsFinite(Request.MaxDrawbarHp)
                || (Request.MaxDrawbarHp <= 0.0))
            {
                Warnings.Add("Max-load fuel rate and drawbar horsepower must be greater than zero.");
                return Block(Assumptions, Warnings);
            }

            if (!Double.IsFinite(Request.PricePerGallon) || (Request.PricePerGallon < 0.0))
            {
                Warnings.Add("Diesel price must be zero or greater.");
                return Block(Assumptions, Warnings);
            }

            // No driving means there is nothing to price. Idle alone is not a plan.
            if ((Request.LoadingHours == 0.0)
                && (Request.HaulingHours == 0.0)
                && (Request.DumpingHours == 0.0)
                && (Request.GradingHours == 0.0)
                && (Request.ReturningHours == 0.0))
            {
                Warnings.Add("The plan produced no work, so diesel cost was not calculated.");
                return Block(Assumptions, Warnings);
            }

            Double FuelPerHp = Request.MaxFuelGalPerHour / Request.MaxDrawbarHp;
            if ((FuelPerHp < MIN_FUEL_GAL_PER_HP_HOUR) || (FuelPerHp > MAX_FUEL_GAL_PER_HP_HOUR))
            {
                Warnings.Add(
                    "Fuel rate and drawbar HP look inconsistent. Check the tractor inputs.");
            }

            SolvedLoad Load = Solve(
                Request.LoadFactorLoad, X_LOAD, false, Request, Resistance, Density);
            SolvedLoad Haul = Solve(
                Request.LoadFactorHaul, X_HAUL, true, Request, Resistance, Density);
            SolvedLoad Dump = Solve(
                Request.LoadFactorDump, X_DUMP, false, Request, Resistance, Density);
            SolvedLoad Grade = Solve(
                Request.LoadFactorGrade, X_GRADE, false, Request, Resistance, Density);
            // Return uses the opposite grade and the empty weight, so it is not the haul solve.
            SolvedLoad Return = SolveReturn(Request, Resistance, Density);
            // Haul and return share one pass over the cycles so a steep cycle is counted once.
            Int32 OverPower = 0;
            if ((Haul.Source != LoadFactorSource.Override)
                || (Return.Source != LoadFactorSource.Override))
            {
                OverPower = CountOverPowerCycles(Request, Resistance, Density);
            }

            if ((Haul.Source == LoadFactorSource.Default)
                && (Return.Source == LoadFactorSource.Default)
                && CyclesWereSupplied(Request)
                && (Request.EmptyRigWeightLb.GetValueOrDefault() > 0.0)
                && !Request.LoadFactorHaul.HasValue
                && !Request.LoadFactorReturn.HasValue)
            {
                Warnings.Add(
                    "Haul cycles could not be priced, so haul and return use the default load factors.");
            }

            if (OverPower > 0)
            {
                String CyclesWord = OverPower == 1 ? "cycle" : "cycles";
                Warnings.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Tractor cannot sustain the planned speed/load on {0} {1}. The plan's haul times may be too optimistic.",
                    OverPower,
                    CyclesWord));
            }

            List<PhaseEstimate> Phases = new List<PhaseEstimate>
            {
                PricePhase(Phase.Load, Request.LoadingHours, Load, Request.ReducedThrottleLoad, Request),
                PricePhase(Phase.Haul, Request.HaulingHours, Haul, Request.ReducedThrottleHaul, Request),
                PricePhase(Phase.Dump, Request.DumpingHours, Dump, Request.ReducedThrottleDump, Request),
                PricePhase(Phase.Grade, Request.GradingHours, Grade, Request.ReducedThrottleGrade, Request),
                PricePhase(Phase.Return, Request.ReturningHours, Return, Request.ReducedThrottleReturn, Request),
            };

            Double IdleGallons = Request.MaxFuelGalPerHour * Request.IdleHours * IDLE_FRACTION;
            Double IdleCost = IdleGallons * Request.PricePerGallon;
            Double TotalGallons = IdleGallons;
            Double TotalHours = Request.IdleHours;
            for (Int32 Index = 0; Index < Phases.Count; Index += 1)
            {
                TotalGallons += Phases[Index].Gallons;
                TotalHours += Phases[Index].Hours;
            }

            Double TotalCost = TotalGallons * Request.PricePerGallon;
            Double AverageRate = TotalHours > 0.0 ? TotalGallons / TotalHours : 0.0;
            Double AveragePercent = Request.MaxFuelGalPerHour > 0.0
                ? (AverageRate / Request.MaxFuelGalPerHour) * 100.0
                : 0.0;
            if ((AveragePercent < MIN_AVERAGE_LOAD_PERCENT)
                || (AveragePercent > MAX_AVERAGE_LOAD_PERCENT))
            {
                Warnings.Add("Unusual average load. Check the plan's phase times.");
            }

            return new Result(
                false,
                Phases,
                Request.IdleHours,
                IdleGallons,
                IdleCost,
                TotalGallons,
                TotalCost,
                TotalHours,
                AverageRate,
                AveragePercent,
                Assumptions,
                Warnings);
        }

        /// <summary>A solved load factor and where it came from.</summary>
        private readonly struct SolvedLoad
        {
            /// <summary>Initializes one solved factor.</summary>
            /// <param name="LoadFactor">Fraction of maximum drawbar power, already clamped when calculated.</param>
            /// <param name="Source">Where the factor came from.</param>
            public SolvedLoad
                (
                Double LoadFactor,
                LoadFactorSource Source
                )
            {
                this.LoadFactor = LoadFactor;
                this.Source = Source;
            }

            /// <summary>Gets the load factor.</summary>
            public Double LoadFactor { get; }

            /// <summary>Gets the source.</summary>
            public LoadFactorSource Source { get; }
        }

        /// <summary>
        /// Picks the load factor for one phase.
        /// An override wins. Haul is the only phase that can be calculated, and
        /// the return is solved in the same cycle pass by the caller when needed.
        /// </summary>
        /// <param name="Override">Caller override. Null means none.</param>
        /// <param name="Fallback">Hard-coded factor.</param>
        /// <param name="Calculate">True for the loaded haul, which may use cycle pull.</param>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Resistance">Rolling-resistance ratio.</param>
        /// <param name="Density">Pounds per bank cubic yard.</param>
        /// <returns>The factor to price.</returns>
        private static SolvedLoad Solve
            (
            Double? Override,
            Double Fallback,
            Boolean Calculate,
            Request Request,
            Double Resistance,
            Double Density
            )
        {
            if (Override.HasValue)
            {
                return new SolvedLoad(Override.Value, LoadFactorSource.Override);
            }

            if (!Calculate || !CanCalculate(Request))
            {
                return new SolvedLoad(Fallback, LoadFactorSource.Default);
            }

            Double Weighted = 0.0;
            Double Time = 0.0;
            IReadOnlyList<Cycle> Cycles = Request.Cycles!;
            for (Int32 Index = 0; Index < Cycles.Count; Index += 1)
            {
                Cycle Cycle = Cycles[Index];
                if (!TryHaulFactor(Cycle, Request, Resistance, Density, out Double Factor, out Double LegHours))
                {
                    continue;
                }

                Weighted += Factor * LegHours;
                Time += LegHours;
            }

            if (Time <= 0.0)
            {
                return new SolvedLoad(Fallback, LoadFactorSource.Default);
            }

            return new SolvedLoad(Weighted / Time, LoadFactorSource.Calculated);
        }

        /// <summary>
        /// Solves the empty-return factor. Kept separate so a haul override
        /// does not also freeze the return.
        /// </summary>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Resistance">Rolling-resistance ratio.</param>
        /// <param name="Density">Pounds per bank cubic yard.</param>
        /// <returns>The return factor.</returns>
        private static SolvedLoad SolveReturn
            (
            Request Request,
            Double Resistance,
            Double Density
            )
        {
            if (Request.LoadFactorReturn.HasValue)
            {
                return new SolvedLoad(
                    Request.LoadFactorReturn.Value, LoadFactorSource.Override);
            }

            if (!CanCalculate(Request))
            {
                return new SolvedLoad(X_RETURN, LoadFactorSource.Default);
            }

            Double Weighted = 0.0;
            Double Time = 0.0;
            IReadOnlyList<Cycle> Cycles = Request.Cycles!;
            for (Int32 Index = 0; Index < Cycles.Count; Index += 1)
            {
                Cycle Cycle = Cycles[Index];
                if (!TryReturnFactor(Cycle, Request, Resistance, Density, out Double Factor, out Double LegHours))
                {
                    continue;
                }

                Weighted += Factor * LegHours;
                Time += LegHours;
            }

            if (Time <= 0.0)
            {
                return new SolvedLoad(X_RETURN, LoadFactorSource.Default);
            }

            return new SolvedLoad(Weighted / Time, LoadFactorSource.Calculated);
        }

        /// <summary>
        /// Counts cycles whose unclamped haul or return load exceeds full power.
        /// </summary>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Resistance">Rolling-resistance ratio.</param>
        /// <param name="Density">Pounds per bank cubic yard.</param>
        /// <returns>How many cycles exceed full power on at least one leg.</returns>
        private static Int32 CountOverPowerCycles
            (
            Request Request,
            Double Resistance,
            Double Density
            )
        {
            if (!CanCalculate(Request))
            {
                return 0;
            }

            Int32 Count = 0;
            IReadOnlyList<Cycle> Cycles = Request.Cycles!;
            for (Int32 Index = 0; Index < Cycles.Count; Index += 1)
            {
                Cycle Cycle = Cycles[Index];
                Boolean Over = false;
                if (!Request.LoadFactorHaul.HasValue
                    && TryUnclampedHaul(Cycle, Request, Resistance, Density, out Double HaulFactor)
                    && (HaulFactor > X_MAX))
                {
                    Over = true;
                }

                if (!Request.LoadFactorReturn.HasValue
                    && TryUnclampedReturn(Cycle, Request, Resistance, Density, out Double ReturnFactor)
                    && (ReturnFactor > X_MAX))
                {
                    Over = true;
                }

                if (Over)
                {
                    Count += 1;
                }
            }

            return Count;
        }

        /// <summary>True when weight and at least one cycle are present.</summary>
        /// <param name="Request">The estimate request.</param>
        /// <returns>True when haul pull can be calculated.</returns>
        private static Boolean CanCalculate
            (
            Request Request
            )
        {
            return (Request.EmptyRigWeightLb.GetValueOrDefault() > 0.0)
                && CyclesWereSupplied(Request);
        }

        /// <summary>True when the request includes a cycle list.</summary>
        /// <param name="Request">The estimate request.</param>
        /// <returns>True when the list is non-null and not empty.</returns>
        private static Boolean CyclesWereSupplied
            (
            Request Request
            )
        {
            return (Request.Cycles != null) && (Request.Cycles.Count > 0);
        }

        /// <summary>
        /// Clamped haul factor and the hours it should be weighted by.
        /// </summary>
        /// <param name="Cycle">One cycle.</param>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Resistance">Rolling-resistance ratio.</param>
        /// <param name="Density">Pounds per bank cubic yard.</param>
        /// <param name="Factor">Clamped load factor.</param>
        /// <param name="LegHours">Haul hours, when the leg can be priced.</param>
        /// <returns>False when this cycle has no usable haul leg.</returns>
        private static Boolean TryHaulFactor
            (
            Cycle Cycle,
            Request Request,
            Double Resistance,
            Double Density,
            out Double Factor,
            out Double LegHours
            )
        {
            LegHours = 0.0;
            if (!TryUnclampedHaul(Cycle, Request, Resistance, Density, out Double Unclamped))
            {
                Factor = 0.0;
                return false;
            }

            LegHours = Cycle.HaulTimeHours;
            Factor = ClampCalculated(Unclamped);
            return true;
        }

        /// <summary>
        /// Clamped return factor and the hours it should be weighted by.
        /// </summary>
        /// <param name="Cycle">One cycle.</param>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Resistance">Rolling-resistance ratio.</param>
        /// <param name="Density">Pounds per bank cubic yard.</param>
        /// <param name="Factor">Clamped load factor.</param>
        /// <param name="LegHours">Return hours, when the leg can be priced.</param>
        /// <returns>False when this cycle has no usable return leg.</returns>
        private static Boolean TryReturnFactor
            (
            Cycle Cycle,
            Request Request,
            Double Resistance,
            Double Density,
            out Double Factor,
            out Double LegHours
            )
        {
            LegHours = 0.0;
            if (!TryUnclampedReturn(Cycle, Request, Resistance, Density, out Double Unclamped))
            {
                Factor = 0.0;
                return false;
            }

            LegHours = Cycle.ReturnTimeHours;
            Factor = ClampCalculated(Unclamped);
            return true;
        }

        /// <summary>
        /// Unclamped haul load. A downhill that produces no pull sits at the floor
        /// only after this raw value is inspected for the over-power warning.
        /// </summary>
        /// <param name="Cycle">One cycle.</param>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Resistance">Rolling-resistance ratio.</param>
        /// <param name="Density">Pounds per bank cubic yard.</param>
        /// <param name="Unclamped">Load factor before the floor and ceiling.</param>
        /// <returns>False when distance, time, or weight cannot support a pull.</returns>
        private static Boolean TryUnclampedHaul
            (
            Cycle Cycle,
            Request Request,
            Double Resistance,
            Double Density,
            out Double Unclamped
            )
        {
            Unclamped = 0.0;
            if (!LegInputs(Cycle, Cycle.HaulTimeHours, out Double SpeedMph))
            {
                return false;
            }

            Double EmptyLb = Request.EmptyRigWeightLb.GetValueOrDefault();
            if ((EmptyLb <= 0.0) || !Double.IsFinite(Cycle.SoilVolumeBcy) || (Cycle.SoilVolumeBcy < 0.0))
            {
                return false;
            }

            Double LoadedLb = EmptyLb + (Cycle.SoilVolumeBcy * Density);
            Double PullLb = LoadedLb * (Resistance + Grade(Cycle));
            Unclamped = LoadFromPull(PullLb, SpeedMph, Request.MaxDrawbarHp);
            return true;
        }

        /// <summary>
        /// Unclamped return load. The return climbs the opposite grade.
        /// </summary>
        /// <param name="Cycle">One cycle.</param>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Resistance">Rolling-resistance ratio.</param>
        /// <param name="Density">Pounds per bank cubic yard. Unused; the bowl is empty.</param>
        /// <param name="Unclamped">Load factor before the floor and ceiling.</param>
        /// <returns>False when the return leg cannot be priced.</returns>
        private static Boolean TryUnclampedReturn
            (
            Cycle Cycle,
            Request Request,
            Double Resistance,
            Double Density,
            out Double Unclamped
            )
        {
            Unclamped = 0.0;
            if (!LegInputs(Cycle, Cycle.ReturnTimeHours, out Double SpeedMph))
            {
                return false;
            }

            Double EmptyLb = Request.EmptyRigWeightLb.GetValueOrDefault();
            if (EmptyLb <= 0.0)
            {
                return false;
            }

            // Density is part of the shared signature so both legs read the same way.
            _ = Density;
            Double PullLb = EmptyLb * (Resistance - Grade(Cycle));
            Unclamped = LoadFromPull(PullLb, SpeedMph, Request.MaxDrawbarHp);
            return true;
        }

        /// <summary>
        /// Speed for one leg. Non-positive time or distance cannot define a speed.
        /// </summary>
        /// <param name="Cycle">One cycle.</param>
        /// <param name="TimeHours">Leg time in hours.</param>
        /// <param name="SpeedMph">Miles per hour.</param>
        /// <returns>False when the speed is not finite and positive.</returns>
        private static Boolean LegInputs
            (
            Cycle Cycle,
            Double TimeHours,
            out Double SpeedMph
            )
        {
            SpeedMph = 0.0;
            if (!Double.IsFinite(Cycle.DistanceFt)
                || (Cycle.DistanceFt <= 0.0)
                || !Double.IsFinite(TimeHours)
                || (TimeHours <= 0.0))
            {
                return false;
            }

            SpeedMph = Cycle.DistanceFt / (TimeHours * MINUTES_PER_HOUR * FT_PER_MIN_PER_MPH);
            return Double.IsFinite(SpeedMph) && (SpeedMph > 0.0);
        }

        /// <summary>
        /// Rise over the one-way run. Missing elevations are a level haul.
        /// </summary>
        /// <param name="Cycle">One cycle.</param>
        /// <returns>Positive when the loaded trip goes uphill.</returns>
        private static Double Grade
            (
            Cycle Cycle
            )
        {
            if (!Cycle.CutElevationFt.HasValue
                || !Cycle.FillElevationFt.HasValue
                || (Cycle.DistanceFt <= 0.0))
            {
                return 0.0;
            }

            Double RiseFt = Cycle.FillElevationFt.Value - Cycle.CutElevationFt.Value;
            if (!Double.IsFinite(RiseFt))
            {
                return 0.0;
            }

            return RiseFt / Cycle.DistanceFt;
        }

        /// <summary>
        /// Turns a drawbar pull and speed into a fraction of field drawbar power.
        /// A pull that does not resist the tractor is reported as zero so the
        /// caller can put it on the floor.
        /// </summary>
        /// <param name="PullLb">Drawbar pull in pounds.</param>
        /// <param name="SpeedMph">Travel speed.</param>
        /// <param name="RatedDrawbarHp">Concrete-rated drawbar horsepower.</param>
        /// <returns>Unclamped load factor. Zero when the pull does not load the tractor.</returns>
        private static Double LoadFromPull
            (
            Double PullLb,
            Double SpeedMph,
            Double RatedDrawbarHp
            )
        {
            if (PullLb <= 0.0)
            {
                return 0.0;
            }

            Double AvailableHp = RatedDrawbarHp * TRACTION_DERATE;
            if (AvailableHp <= 0.0)
            {
                return 0.0;
            }

            Double DrawbarHp = (PullLb * SpeedMph) / HP_CONVERSION;
            return DrawbarHp / AvailableHp;
        }

        /// <summary>
        /// Applies the calculated-factor floor and ceiling.
        /// A zero from a non-positive pull uses the floor.
        /// </summary>
        /// <param name="Unclamped">Load factor before the limits.</param>
        /// <returns>The factor used for fuel.</returns>
        private static Double ClampCalculated
            (
            Double Unclamped
            )
        {
            // A coasting downhill still burns the floor, not zero.
            if (Unclamped < X_MIN)
            {
                return X_MIN;
            }

            if (Unclamped > X_MAX)
            {
                return X_MAX;
            }

            return Unclamped;
        }

        /// <summary>Prices one phase, including part-throttle savings when they apply.</summary>
        /// <param name="Phase">The duty.</param>
        /// <param name="PhaseHours">Driving hours.</param>
        /// <param name="Load">Solved load factor.</param>
        /// <param name="ReducedThrottle">Caller asked for part throttle on this duty.</param>
        /// <param name="Request">Tractor rate and price.</param>
        /// <returns>The phase line.</returns>
        private static PhaseEstimate PricePhase
            (
            Phase Phase,
            Double PhaseHours,
            SolvedLoad Load,
            Boolean ReducedThrottle,
            Request Request
            )
        {
            Boolean ApplyThrottle = ReducedThrottle && (Load.LoadFactor < THROTTLE_BACK_MAX_LOAD);
            Double Fraction = BASE_FRACTION + ((1.0 - BASE_FRACTION) * Load.LoadFactor);
            if (ApplyThrottle)
            {
                Fraction *= (1.0 - THROTTLE_BACK_SAVINGS);
            }

            Double Gallons = Request.MaxFuelGalPerHour * PhaseHours * Fraction;
            return new PhaseEstimate(
                Phase,
                PhaseHours,
                Load.LoadFactor,
                Load.Source,
                ApplyThrottle,
                Fraction,
                Gallons,
                Gallons * Request.PricePerGallon);
        }

        /// <summary>Selects the rolling resistance and a short note for the assumption list.</summary>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Note">A parenthetical describing which rule won.</param>
        /// <returns>The ratio of pull to weight on level ground.</returns>
        private static Double ResolveRollingResistance
            (
            Request Request,
            out String Note
            )
        {
            if (Request.RollingResistanceOverride.HasValue
                && Double.IsFinite(Request.RollingResistanceOverride.Value)
                && (Request.RollingResistanceOverride.Value >= 0.0))
            {
                Note = " (override)";
                return Request.RollingResistanceOverride.Value;
            }

            if (Request.SoilCondition == SoilCondition.Firm)
            {
                Note = " (firm soil)";
                return ROLLING_RESISTANCE_FIRM;
            }

            if (Request.SoilCondition == SoilCondition.Soft)
            {
                Note = " (soft soil)";
                return ROLLING_RESISTANCE_SOFT;
            }

            if (Request.SoilCondition == SoilCondition.Tilled)
            {
                Note = " (tilled soil)";
                return ROLLING_RESISTANCE_TILLED;
            }

            Note = " (default)";
            return ROLLING_RESISTANCE;
        }

        /// <summary>Selects soil density and whether it was overridden.</summary>
        /// <param name="Request">The estimate request.</param>
        /// <param name="Note">Empty, or a note that the caller replaced the default.</param>
        /// <returns>Pounds per bank cubic yard.</returns>
        private static Double ResolveDensity
            (
            Request Request,
            out String Note
            )
        {
            if (Request.SoilDensityLbPerBcy.HasValue
                && Double.IsFinite(Request.SoilDensityLbPerBcy.Value)
                && (Request.SoilDensityLbPerBcy.Value > 0.0))
            {
                Note = " (override)";
                return Request.SoilDensityLbPerBcy.Value;
            }

            Note = "";
            return SOIL_DENSITY_LB_PER_BCY;
        }

        /// <summary>Writes the Section 5 values that this run used.</summary>
        /// <param name="Resistance">Rolling resistance in effect.</param>
        /// <param name="ResistanceNote">Why that resistance was chosen.</param>
        /// <param name="Density">Soil density in effect.</param>
        /// <param name="DensityNote">Empty, or an override note.</param>
        /// <returns>One entry per constant.</returns>
        private static IReadOnlyList<String> BuildAssumptions
            (
            Double Resistance,
            String ResistanceNote,
            Double Density,
            String DensityNote
            )
        {
            return new String[]
            {
                "BASE_FRACTION=" + Number(BASE_FRACTION),
                "X_LOAD=" + Number(X_LOAD),
                "X_HAUL=" + Number(X_HAUL),
                "X_DUMP=" + Number(X_DUMP),
                "X_GRADE=" + Number(X_GRADE),
                "X_RETURN=" + Number(X_RETURN),
                "IDLE_FRACTION=" + Number(IDLE_FRACTION),
                "ROLLING_RESISTANCE=" + Number(Resistance) + ResistanceNote,
                "SOIL_DENSITY=" + Number(Density) + DensityNote,
                "TRACTION_DERATE=" + Number(TRACTION_DERATE),
                "THROTTLE_BACK_SAVINGS=" + Number(THROTTLE_BACK_SAVINGS),
                "HP_CONVERSION=" + Number(HP_CONVERSION),
                "FT_PER_MIN_PER_MPH=" + Number(FT_PER_MIN_PER_MPH),
                "X_MIN=" + Number(X_MIN),
                "X_MAX=" + Number(X_MAX),
            };
        }

        /// <summary>Formats a constant for the assumption list.</summary>
        /// <param name="Value">The constant.</param>
        /// <returns>A stable decimal with no trailing culture marks.</returns>
        private static String Number
            (
            Double Value
            )
        {
            if (Math.Abs(Value - Math.Round(Value)) < 1e-9)
            {
                return Math.Round(Value).ToString("0", CultureInfo.InvariantCulture);
            }

            return Value.ToString("0.00", CultureInfo.InvariantCulture);
        }

        /// <summary>Builds a refused estimate.</summary>
        /// <param name="Assumptions">Constants in effect.</param>
        /// <param name="Warnings">Why pricing stopped, plus any earlier checks.</param>
        /// <returns>An estimate with no gallons.</returns>
        private static Result Block
            (
            IReadOnlyList<String> Assumptions,
            List<String> Warnings
            )
        {
            return new Result(
                true,
                new PhaseEstimate[0],
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Assumptions,
                Warnings);
        }

        /// <summary>True when every phase hour is finite and not negative.</summary>
        /// <param name="Request">The estimate request.</param>
        /// <returns>False when a hour cannot be priced.</returns>
        private static Boolean HoursAreUsable
            (
            Request Request
            )
        {
            return NonNegativeFinite(Request.LoadingHours)
                && NonNegativeFinite(Request.HaulingHours)
                && NonNegativeFinite(Request.DumpingHours)
                && NonNegativeFinite(Request.GradingHours)
                && NonNegativeFinite(Request.ReturningHours);
        }

        /// <summary>True for a finite value at least zero.</summary>
        /// <param name="Value">The number to test.</param>
        /// <returns>False for NaN, infinity, or a negative.</returns>
        private static Boolean NonNegativeFinite
            (
            Double Value
            )
        {
            return Double.IsFinite(Value) && (Value >= 0.0);
        }

        /// <summary>True when a leg has positive finite distance and time.</summary>
        /// <param name="DistanceFt">Leg distance in feet.</param>
        /// <param name="TimeHours">Leg time in hours.</param>
        /// <returns>False when speed would not be defined.</returns>
        private static Boolean UsableLeg
            (
            Double DistanceFt,
            Double TimeHours
            )
        {
            return Double.IsFinite(DistanceFt)
                && (DistanceFt > 0.0)
                && Double.IsFinite(TimeHours)
                && (TimeHours > 0.0);
        }

        /// <summary>Reads one phase, treating a missing entry as no time.</summary>
        /// <param name="PhaseSeconds">Guidance seconds by phase.</param>
        /// <param name="Phase">The lifecycle phase.</param>
        /// <returns>Seconds, or zero.</returns>
        private static Double Seconds
            (
            IReadOnlyDictionary<LifecyclePlanner.Phase, Double> PhaseSeconds,
            LifecyclePlanner.Phase Phase
            )
        {
            Double Value;
            if (!PhaseSeconds.TryGetValue(Phase, out Value) || !Double.IsFinite(Value))
            {
                return 0.0;
            }

            return Value;
        }

        /// <summary>Converts seconds to hours.</summary>
        /// <param name="Seconds">A duration in seconds.</param>
        /// <returns>Hours.</returns>
        private static Double Hours
            (
            Double Seconds
            )
        {
            return Seconds / Constants.SECONDS_PER_HOUR;
        }

        /// <summary>True when the caller set any diesel field.</summary>
        /// <param name="Options">Tractor and price inputs.</param>
        /// <returns>False when the estimate should be skipped with no error.</returns>
        private static Boolean IsConfigured
            (
            DieselOptions Options
            )
        {
            return Options.MaxFuelGalPerHour.HasValue
                || Options.MaxDrawbarHp.HasValue
                || Options.PricePerGallon.HasValue
                || Options.EmptyRigWeightLb.HasValue
                || Options.LoadFactorLoad.HasValue
                || Options.LoadFactorHaul.HasValue
                || Options.LoadFactorDump.HasValue
                || Options.LoadFactorGrade.HasValue
                || Options.LoadFactorReturn.HasValue
                || Options.SoilCondition.HasValue
                || Options.RollingResistance.HasValue
                || Options.SoilDensityLbPerBcy.HasValue
                || Options.ReducedThrottleLoad
                || Options.ReducedThrottleHaul
                || Options.ReducedThrottleDump
                || Options.ReducedThrottleGrade
                || Options.ReducedThrottleReturn
                || (Options.IdleHours != 0.0);
        }

        /// <summary>Requires a finite value greater than zero.</summary>
        /// <param name="Value">The number.</param>
        /// <param name="Name">The option name for the exception.</param>
        private static void Positive
            (
            Double Value,
            String Name
            )
        {
            if (!Double.IsFinite(Value) || (Value <= 0.0))
            {
                throw new ArgumentOutOfRangeException(
                    Name, "Value must be finite and greater than zero.");
            }
        }

        /// <summary>Requires a finite value at least zero.</summary>
        /// <param name="Value">The number.</param>
        /// <param name="Name">The option name for the exception.</param>
        private static void NonNegative
            (
            Double Value,
            String Name
            )
        {
            if (!Double.IsFinite(Value) || (Value < 0.0))
            {
                throw new ArgumentOutOfRangeException(
                    Name, "Value must be finite and zero or greater.");
            }
        }

        /// <summary>Requires an override, when set, to lie on the closed unit interval.</summary>
        /// <param name="Value">The override. Null is allowed.</param>
        /// <param name="Name">The option name for the exception.</param>
        private static void UnitInterval
            (
            Double? Value,
            String Name
            )
        {
            if (!Value.HasValue)
            {
                return;
            }

            if (!Double.IsFinite(Value.Value) || (Value.Value < 0.0) || (Value.Value > 1.0))
            {
                throw new ArgumentOutOfRangeException(
                    Name, "Load factor must be from 0 through 1.");
            }
        }
    }
}
