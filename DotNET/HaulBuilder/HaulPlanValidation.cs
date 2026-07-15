using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>Result of validating an optimized haul plan.</summary>
    public sealed class HaulPlanValidation
    {
        /// <summary>Initializes a validation result.</summary>
        public HaulPlanValidation
            (
            Boolean Success,
            IReadOnlyList<String> Issues,
            Double MaximumCutErrorM,
            Double MaximumFillErrorM
            )
        {
            this.Success = Success;
            this.Issues = Issues;
            this.MaximumCutErrorM = MaximumCutErrorM;
            this.MaximumFillErrorM = MaximumFillErrorM;
        }

        /// <summary>Gets whether all invariants passed.</summary>
        public Boolean Success { get; }

        /// <summary>Gets structured validation issues.</summary>
        public IReadOnlyList<String> Issues { get; }

        /// <summary>Gets maximum cut-depth reconciliation error.</summary>
        public Double MaximumCutErrorM { get; }

        /// <summary>Gets maximum fill-depth reconciliation error.</summary>
        public Double MaximumFillErrorM { get; }
    }
}
