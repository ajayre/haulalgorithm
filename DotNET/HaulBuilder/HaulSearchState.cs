namespace HaulBuilder
{
    /// <summary>
    /// The search state. Hauling follows the flow ribbon. Clusters takes over
    /// when the recent loads fall short of the bowl. Thin-cut takes over when
    /// cluster hauling finds no haulable work. Correcting takes over when
    /// thin-cut finishes. The search stops there.
    /// </summary>
    public enum HaulSearchState
    {
        /// <summary>Flow ribbon along the arrows.</summary>
        Hauling    = 0,

        /// <summary>The trailing mean fell under the cluster fraction.</summary>
        Clusters   = 1,

        /// <summary>Cluster hauling found no haulable work. Thin tiles feed cut and fill.</summary>
        ThinCut    = 2,

        /// <summary>Thin-cut finished. Coverage passes cut and spread to grade.</summary>
        Correcting = 3,
    }
}
