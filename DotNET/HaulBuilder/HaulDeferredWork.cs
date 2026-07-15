using System;

namespace HaulBuilder
{
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
