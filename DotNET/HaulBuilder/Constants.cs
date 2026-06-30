namespace HaulBuilder
{
    /// <summary>
    /// Physical unit conversions and default parameters for the haul algorithm (spec §6.2–§6.4).
    /// </summary>
    public static class Constants
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
        /// Default square grid cell edge length in feet (~ one blade width).
        /// </summary>
        public const Double DEFAULT_GRID_SIZE_FT = 7.5;

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
        public const Double DEFAULT_MAX_DROP_HOP_FT = 100.0;

        /// <summary>
        /// Default scraper blade width in millimetres.
        /// </summary>
        public const Double DEFAULT_BLADE_WIDTH_MM = 2271.0;

        /// <summary>
        /// Field efficiency fraction (productive time); 0.8 = 80%.
        /// </summary>
        public const Double DEFAULT_FIELD_EFFICIENCY = 0.8;

        /// <summary>
        /// Default loading speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_LOADING_SPEED_MPH = 4.0;

        /// <summary>
        /// Default loaded haul speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_HAUL_SPEED_MPH = 10.0;

        /// <summary>
        /// Default dumping speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_DUMPING_SPEED_MPH = 5.0;

        /// <summary>
        /// Default deadhead return speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_RETURN_SPEED_MPH = 13.0;

        /// <summary>
        /// Default acceleration phase duration in seconds.
        /// </summary>
        public const Double DEFAULT_ACCEL_TIME_S = 12.0;

        /// <summary>
        /// Default deceleration phase duration in seconds.
        /// </summary>
        public const Double DEFAULT_DECEL_TIME_S = 12.0;

        /// <summary>
        /// Default turn speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_TURN_SPEED_MPH = 4.0;

        /// <summary>
        /// Default grade/compact return speed in miles per hour.
        /// </summary>
        public const Double DEFAULT_GRADE_SPEED_MPH = 8.0;

        /// <summary>
        /// Default tractor+scraper turning circle diameter in feet.
        /// </summary>
        public const Double DEFAULT_TURNING_CIRCLE_FT = 45.0;

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
