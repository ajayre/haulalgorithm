using System;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// Everything one solve needs: the field and equipment as HaulBuilder options,
    /// the values to try, how unfinished soil is charged, and the stopping limits.
    /// </summary>
    public sealed class FieldSolverRequest
    {
        /// <summary>Default charge for unfinished soil, in multiples of the baseline's metres per BCY.</summary>
        public const Double DEFAULT_UNFINISHED_SOIL_FACTOR = 2.0;

        /// <summary>Initializes a request with the standard search and no limits.</summary>
        /// <param name="BaseOptions">
        /// The caller's options: field file, equipment, and current settings. The
        /// solver reads this object and never changes it.
        /// </param>
        /// <exception cref="ArgumentNullException">Thrown when base options is null.</exception>
        public FieldSolverRequest
            (
            HaulBuilderOptions BaseOptions
            )
        {
            if (BaseOptions == null) throw new ArgumentNullException(nameof(BaseOptions));
            this.BaseOptions = BaseOptions;
            // Intent: the standard search's lane axes come from this field's design slope.
            SearchSpace = FieldSolverSearchSpace.CreateStandard(BaseOptions);
            UnfinishedSoilFactor = DEFAULT_UNFINISHED_SOIL_FACTOR;
        }

        /// <summary>Gets the caller's options. The first run uses them unchanged.</summary>
        public HaulBuilderOptions BaseOptions { get; }

        /// <summary>Gets or sets the values to try.</summary>
        public FieldSolverSearchSpace SearchSpace { get; set; }

        /// <summary>
        /// Gets or sets the charge for soil a plan leaves unfinished, as a multiple
        /// of the baseline's driven metres per BCY placed. Zero ranks plans on
        /// driven distance alone.
        /// </summary>
        public Double UnfinishedSoilFactor { get; set; }

        /// <summary>
        /// Gets or sets the largest unfinished soil a plan may leave, as a percentage
        /// of the soil the field allows to be moved. Null means no limit. The
        /// baseline is exempt.
        /// </summary>
        public Double? MaximumUnfinishedPercent { get; set; }

        /// <summary>
        /// Gets or sets the largest manual assist count a plan may have. Null means
        /// no limit. The baseline is exempt.
        /// </summary>
        public Int32? MaximumManualAssists { get; set; }

        /// <summary>Gets or sets the most engine runs to make. Zero means no limit.</summary>
        public Int32 MaximumRuns { get; set; }

        /// <summary>Gets or sets the longest time to search. Null means no limit.</summary>
        public TimeSpan? TimeLimit { get; set; }
    }
}
