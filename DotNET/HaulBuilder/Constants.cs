namespace HaulBuilder
{
    /// <summary>
    /// Physical unit conversions and default parameters for the haul algorithm (spec §6.2–§6.4).
    /// </summary>
    static public class Constants
    {
        /// <summary>
        /// Assembly version string exposed to callers and CLI parity checks.
        /// </summary>
        public const String VERSION = "0.0.1";

        /// <summary>
        /// Cubic metres to cubic yards (matches fielddesign.py).
        /// </summary>
        public const Double M3_TO_CY = 1.30795061931439;

        /// <summary>
        /// Feet to metres conversion factor.
        /// </summary>
        public const Double FEET_TO_METERS = 0.3048;

        /// <summary>
        /// Miles per hour to metres per second.
        /// </summary>
        public const Double MPH_TO_MPS = 0.44704;

        /// <summary>
        /// Millimetres in one metre.
        /// </summary>
        internal const Double MILLIMETERS_PER_METER = 1000.0;

        /// <summary>
        /// Degrees in one undirected half-turn axis.
        /// </summary>
        internal const Double DEGREES_PER_HALF_TURN = 180.0;

        /// <summary>
        /// Degrees in one complete turn.
        /// </summary>
        internal const Double DEGREES_PER_TURN = 360.0;

        /// <summary>
        /// Degrees in one right-angle quarter turn.
        /// </summary>
        internal const Double DEGREES_PER_QUARTER_TURN = 90.0;

        /// <summary>
        /// Degrees in three quarters of a complete turn.
        /// </summary>
        internal const Double DEGREES_PER_THREE_QUARTER_TURN = 270.0;

        /// <summary>
        /// Positive offset used before reducing a signed angle modulo one turn.
        /// </summary>
        internal const Double ANGLE_NORMALIZATION_OFFSET_DEGREES = 540.0;

        /// <summary>
        /// Seconds in one hour.
        /// </summary>
        internal const Double SECONDS_PER_HOUR = 3600.0;

        /// <summary>
        /// Metres in one statute mile.
        /// </summary>
        internal const Double METERS_PER_MILE = 1609.344;

        /// <summary>
        /// Scale factor that selects one half of a value or interval.
        /// </summary>
        internal const Double HALF_SCALE = 0.5;

        /// <summary>
        /// Scale factor that selects one quarter of a value or interval.
        /// </summary>
        internal const Double QUARTER_SCALE = 0.25;

        /// <summary>
        /// Default tolerance for floating-point comparisons in geometry and
        /// optimization logic.
        /// </summary>
        internal const Double NUMERIC_COMPARISON_TOLERANCE = 1e-9;

        /// <summary>
        /// Stricter tolerance used where near-exact floating-point equality is
        /// required.
        /// </summary>
        internal const Double STRICT_NUMERIC_COMPARISON_TOLERANCE = 1e-12;

        /// <summary>
        /// Tolerance used when comparing independently calculated geometry in
        /// metres or unit-vector space.
        /// </summary>
        internal const Double GEOMETRY_COMPARISON_TOLERANCE = 1e-6;

        /// <summary>
        /// Coarser tolerance used when joining sampled paths and angular
        /// constructions.
        /// </summary>
        internal const Double PATH_JOIN_COMPARISON_TOLERANCE = 1e-3;

        /// <summary>
        /// Maximum value of an eight-bit color or alpha channel.
        /// </summary>
        internal const Byte MAX_COLOR_CHANNEL = 255;

        /// <summary>
        /// Bit mask selecting the low eight bits of a packed value.
        /// </summary>
        internal const Byte BYTE_MASK = 0xFF;

        /// <summary>
        /// Default square grid cell edge length in feet (matches effective blade width).
        /// </summary>
        public const Double DEFAULT_GRID_SIZE_FT = 8.5;

        /// <summary>
        /// Finished-area elevation tolerance in millimetres.
        /// </summary>
        public const Double DEFAULT_TOLERANCE_MM = 10.0;

        /// <summary>
        /// Scraper capacity in loose cubic yards (LCY).
        /// </summary>
        public const Double DEFAULT_SCRAPER_CAPACITY_LCY = 20.0;

        /// <summary>
        /// Swell factor: LCY = BCY × swell.
        /// </summary>
        public const Double DEFAULT_SWELL_FACTOR = 1.3;

        /// <summary>
        /// Cut/fill ratio: bank cut volume per bank fill volume (compaction).
        /// </summary>
        public const Double DEFAULT_COMPACTION = 1.2;

        /// <summary>
        /// Maximum cut/fill depth per pass in millimetres (0.2 ft).
        /// </summary>
        public const Double DEFAULT_MAX_DEPTH_MM = 61.0;

        /// <summary>
        /// Multi-drop hop limit in feet (blade-up reposition between pockets).
        /// </summary>
        internal const Double DEFAULT_MAX_DROP_HOP_FT = 100.0;

        /// <summary>
        /// Default scraper blade width in millimetres (8.5 ft tandem effective width).
        /// </summary>
        public const Double DEFAULT_BLADE_WIDTH_MM = 2590.8;

        /// <summary>
        /// Field efficiency fraction (productive time); 0.8 = 80%.
        /// </summary>
        public const Double DEFAULT_FIELD_EFFICIENCY = 0.8;

        /// <summary>
        /// Cross-field optimizer target measured in bank cubic yards moved per
        /// deterministic production-guidance hour.
        /// </summary>
        public const Double GOLDEN_MOVE_RATE_BCY_PER_HOUR = 200.0;

        /// <summary>
        /// Default cut speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_CUT_SPEED_MPH = 4.0;

        /// <summary>
        /// Default loaded haul speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_LOADED_TRAVEL_SPEED_MPH = 10.0;

        /// <summary>
        /// Default fill speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_FILL_SPEED_MPH = 5.0;

        /// <summary>
        /// Default unloaded travel speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_UNLOADED_TRAVEL_SPEED_MPH = 13.0;

        /// <summary>
        /// Default acceleration phase duration in seconds.
        /// </summary>
        public const Double DEFAULT_ACCELERATION_TIME_S = 12.0;

        /// <summary>
        /// Default deceleration phase duration in seconds.
        /// </summary>
        public const Double DEFAULT_DECELERATION_TIME_S = 12.0;

        /// <summary>
        /// Default turn speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_TURN_SPEED_MPH = 4.0;

        /// <summary>
        /// Default grade/compact return speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_GRADE_SPEED_MPH = 8.0;

        /// <summary>
        /// Default tractor+scraper turning circle diameter in feet (radius 22.5 ft).
        /// This is the tightest blade-down turn, and only on the field edge.
        /// </summary>
        public const Double DEFAULT_TURNING_CIRCLE_FT = 45.0;

        /// <summary>
        /// Blade-down bends inside a cut or a fill use this many turning circles.
        /// The field edge may use one. A tight turn is slow, so the pan
        /// keeps the wider radius while it is cutting or filling inside the field.
        /// </summary>
        public const Double INTERIOR_BLADE_TURN_SCALE = 2.0;

        /// <summary>
        /// Default tractor+scraper overall length in feet (lineup before/after CUT/FILL).
        /// </summary>
        public const Double DEFAULT_EQUIPMENT_LENGTH_FT = 43.0;

        /// <summary>
        /// Default heading-change threshold treated as a true turn in degrees.
        /// </summary>
        public const Double DEFAULT_TURN_THRESHOLD_DEG = 60.0;

        /// <summary>
        /// Default random seed for deterministic zone clustering.
        /// </summary>
        public const Int32 DEFAULT_SEED = 1234;

        /// <summary>
        /// Default PNG pixels per grid cell edge.
        /// </summary>
        public const Int32 DEFAULT_PIXELS_PER_BIN = 50;

        /// <summary>
        /// Default center-pivot wheelbase / arm width in feet.
        /// </summary>
        public const Double DEFAULT_PIVOT_WHEELBASE_FT = 14.0;

        /// <summary>
        /// Default extra center-pivot clearance per side in feet.
        /// </summary>
        public const Double DEFAULT_PIVOT_CLEARANCE_FT = 25.0;
    }
}
