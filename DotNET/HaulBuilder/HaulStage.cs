namespace HaulBuilder
{
    /// <summary>
    /// Which builder produced a trip. Later trips may come from an earlier
    /// builder, so the label stays on the trip instead of a route boundary.
    /// </summary>
    public enum HaulStage
    {
        /// <summary>The flow ribbon built this trip.</summary>
        Flow       = 0,

        /// <summary>Cluster pairing built this trip.</summary>
        Clusters   = 1,

        /// <summary>A thin-cut tile built this trip.</summary>
        ThinCut    = 2,

        /// <summary>Correcting coverage built this trip.</summary>
        Correcting = 3,
    }
}
