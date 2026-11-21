using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// The triggers that move the search between hauling, clusters, thin-cut, and correcting.
    /// Hauling hands off when the recent loads fall short of the bowl.
    /// Clusters hands off from its own round loop, not from the 50 mm band.
    /// Thin-cut then feeds a short fill from tiles, and correcting follows that.
    /// </summary>
    static public class ClusterMode
    {
        /// <summary>
        /// Clusters hands off at this depth. A cell exactly this far from the
        /// target is still inside the band.
        /// </summary>
        private const Double CORRECTING_BAND_M = 0.050;

        /// <summary>
        /// True when the mean of the last <paramref name="Window"/> loads is
        /// strictly under <paramref name="LoadFraction"/> of bowl capacity.
        /// That is the trigger from hauling to clusters.
        /// </summary>
        /// <param name="Passes">Found-order loads so far.</param>
        /// <param name="CapacityBcy">Bowl capacity in bank cubic yards.</param>
        /// <param name="Window">Loads in the mean. Below one never leaves hauling.</param>
        /// <param name="LoadFraction">Share of capacity. Zero never leaves hauling.</param>
        /// <returns>True when hauling should hand off to clusters.</returns>
        internal static Boolean TrailingMeanIsUnder
            (
            IReadOnlyList<Pass> Passes,
            Double CapacityBcy,
            Int32 Window,
            Double LoadFraction
            )
        {
            if ((Window < 1)
                || (LoadFraction <= 0.0)
                || (Passes.Count < Window)
                || (CapacityBcy <= 0.0))
            {
                return false;
            }
            Double Limit = CapacityBcy * LoadFraction;
            Double Sum = 0.0;
            Int32 Start = Passes.Count - Window;
            for (Int32 Index = Start; Index < Passes.Count; Index += 1)
            {
                Sum += Passes[Index].LoadBcy;
            }
            return (Sum / Window) < Limit;
        }

        /// <summary>
        /// True when no remaining cell is more than 50 mm from the target.
        /// The cluster loop does not switch on this. The end-of-cluster report
        /// still counts cells outside the band.
        /// </summary>
        /// <param name="CutLeft">Remaining cut depth, in metres.</param>
        /// <param name="FillLeft">Remaining fill depth, in metres.</param>
        /// <param name="Grid">Field grid. Cells outside the field are ignored.</param>
        /// <returns>True when correcting should take over.</returns>
        internal static Boolean FieldIsWithinCorrectingBand
            (
            Double[,] CutLeft,
            Double[,] FillLeft,
            Grid Grid
            )
        {
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }
                    // One cell past the band keeps the search in clusters.
                    if ((CutLeft[By, Bx] > CORRECTING_BAND_M) || (FillLeft[By, Bx] > CORRECTING_BAND_M))
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }
}
