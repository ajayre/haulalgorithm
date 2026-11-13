using System;

namespace HaulBuilder
{
    /// <summary>Stable reason strings used by deferred-work accounting.</summary>
    public static class HaulDeferredWorkReasons
    {
        /// <summary>Work omitted because blade-down run length is below the equipment minimum.</summary>
        public const String BELOW_MINIMUM_WORK_LENGTH = "BELOW_MINIMUM_WORK_LENGTH";

        /// <summary>
        /// Entire remaining pass deferred because cut/fill sequencing could not
        /// produce a legal forward guidance track after scrap omission.
        /// </summary>
        public const String GUIDANCE_SEQUENCE_FAILED = "GUIDANCE_SEQUENCE_FAILED";

        /// <summary>Interior packing left field-edge cells for later residual completion.</summary>
        public const String FIELD_EDGE_HEADING = "FIELD_EDGE_HEADING";

        /// <summary>Work remaining only because of material surplus or imbalance.</summary>
        public const String MATERIAL_IMBALANCE = "MATERIAL_IMBALANCE";

        /// <summary>Unassigned work after residual and repair stages.</summary>
        public const String OPTIMIZER_UNASSIGNED = "OPTIMIZER_UNASSIGNED";

        /// <summary>
        /// Depth allocated to a cycle that guidance coverage left out of every
        /// run and that no short-run list holds.
        /// </summary>
        public const String GUIDANCE_COVERAGE_OMITTED = "GUIDANCE_COVERAGE_OMITTED";
    }

    /// <summary>One exact depth-layer entry not assigned to the automatic plan.</summary>
    public sealed class HaulDeferredWork
    {
        /// <summary>Initializes one immutable deferred-work entry.</summary>
        public HaulDeferredWork
            (
            HaulLaneBuilder.WorkKind Kind,
            (Int32 Bx, Int32 By) Cell,
            Int32 LayerIndex,
            Double DepthM,
            Double VolumeBcy,
            String Reason
            )
        {
            this.Kind = Kind;
            this.Cell = Cell;
            this.LayerIndex = LayerIndex;
            this.DepthM = DepthM;
            this.VolumeBcy = VolumeBcy;
            this.Reason = Reason;
        }

        /// <summary>Gets whether this entry is cut or fill work.</summary>
        public HaulLaneBuilder.WorkKind Kind { get; }

        /// <summary>Gets the physical cell coordinate.</summary>
        public (Int32 Bx, Int32 By) Cell { get; }

        /// <summary>Gets the zero-based physical layer index.</summary>
        public Int32 LayerIndex { get; }

        /// <summary>Gets exact deferred depth in metres.</summary>
        public Double DepthM { get; }

        /// <summary>Gets bank volume represented by the deferred depth.</summary>
        public Double VolumeBcy { get; }

        /// <summary>Gets the stable deferral reason.</summary>
        public String Reason { get; }
    }
}
