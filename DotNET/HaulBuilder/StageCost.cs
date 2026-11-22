using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Picks the next haul by seconds per bank cubic yard.
    /// Flow wins a tie, then clusters, then thin-cut.
    /// A trip dearer than the ceiling is not scheduled.
    /// </summary>
    public static class StageCost
    {
        /// <summary>
        /// A haul trip dearer than this many times the early flow median is not scheduled.
        /// </summary>
        public const Double MAX_COST_RATIO = 3.0;

        /// <summary>
        /// Flow trips that fix the median. The ceiling uses the first this many, in order.
        /// </summary>
        public const Int32 BASELINE_FLOW_TRIPS = 500;

        /// <summary>
        /// One stage's next trip, priced before it is scheduled.
        /// </summary>
        public readonly struct Offer
        {
            /// <summary>
            /// Creates an offer.
            /// </summary>
            /// <param name="Stage">The builder that produced the trip.</param>
            /// <param name="Seconds">Predicted cycle seconds, including efficiency.</param>
            /// <param name="PayloadBcy">Bank cubic yards the trip would move.</param>
            public Offer
                (
                HaulStage Stage,
                Double Seconds,
                Double PayloadBcy
                )
            {
                this.Stage = Stage;
                this.Seconds = Seconds;
                this.PayloadBcy = PayloadBcy;
            }

            /// <summary>Gets the builder.</summary>
            public HaulStage Stage { get; }

            /// <summary>Gets the predicted cycle seconds.</summary>
            public Double Seconds { get; }

            /// <summary>Gets the payload in bank cubic yards.</summary>
            public Double PayloadBcy { get; }

            /// <summary>
            /// Gets seconds per bank cubic yard. A non-positive payload is not a trip.
            /// </summary>
            public Double SecondsPerBcy
            {
                get
                {
                    if (PayloadBcy <= 0.0)
                    {
                        return Double.PositiveInfinity;
                    }
                    return Seconds / PayloadBcy;
                }
            }
        }

        /// <summary>
        /// The cheapest offer at or under the ceiling.
        /// Equal cost keeps the earlier stage: flow, then clusters, then thin-cut.
        /// </summary>
        /// <param name="Offers">The trips each stage can build now. An empty list schedules nothing.</param>
        /// <param name="CeilingSecondsPerBcy">
        /// Maximum seconds per bank cubic yard. Null means the flow median is not fixed yet,
        /// so every positive payload is allowed.
        /// </param>
        /// <returns>The trip to schedule, or null when nothing is under the ceiling.</returns>
        public static Offer? Choose
            (
            IReadOnlyList<Offer> Offers,
            Double? CeilingSecondsPerBcy
            )
        {
            if (Offers == null) throw new ArgumentNullException(nameof(Offers));
            Offer? Best = null;
            for (Int32 Index = 0; Index < Offers.Count; Index += 1)
            {
                Offer Candidate = Offers[Index];
                if (Candidate.PayloadBcy <= 0.0)
                {
                    continue;
                }
                // The ceiling is a hard stop. A dearer trip is not a fallback.
                if (CeilingSecondsPerBcy.HasValue
                    && (Candidate.SecondsPerBcy > CeilingSecondsPerBcy.Value))
                {
                    continue;
                }
                if (!Best.HasValue || Cheaper(Candidate, Best.Value))
                {
                    Best = Candidate;
                }
            }
            return Best;
        }

        /// <summary>
        /// The ceiling once enough flow trips exist. Null until then.
        /// </summary>
        /// <param name="FlowSecondsPerBcy">Cost of each flow trip already taken, in order.</param>
        /// <param name="BaselineTrips">How many leading flow trips fix the median.</param>
        /// <returns>Maximum seconds per bank cubic yard, or null while the baseline is still open.</returns>
        public static Double? CeilingSecondsPerBcy
            (
            IReadOnlyList<Double> FlowSecondsPerBcy,
            Int32 BaselineTrips
            )
        {
            if (FlowSecondsPerBcy == null) throw new ArgumentNullException(nameof(FlowSecondsPerBcy));
            if (BaselineTrips < 1) throw new ArgumentOutOfRangeException(nameof(BaselineTrips));
            if (FlowSecondsPerBcy.Count < BaselineTrips)
            {
                return null;
            }
            return MAX_COST_RATIO * Median(FlowSecondsPerBcy, BaselineTrips);
        }

        /// <summary>
        /// Median of the first <paramref name="Count"/> values.
        /// An even count averages the two middle values after sorting.
        /// </summary>
        /// <param name="Values">Source values. The first count entries are copied.</param>
        /// <param name="Count">How many leading values to include.</param>
        /// <returns>The median.</returns>
        public static Double Median
            (
            IReadOnlyList<Double> Values,
            Int32 Count
            )
        {
            if (Values == null) throw new ArgumentNullException(nameof(Values));
            if ((Count < 1) || (Count > Values.Count)) throw new ArgumentOutOfRangeException(nameof(Count));
            Double[] Copy = new Double[Count];
            for (Int32 Index = 0; Index < Count; Index += 1)
            {
                Copy[Index] = Values[Index];
            }
            Array.Sort(Copy);
            Int32 Mid = Count / 2;
            if ((Count % 2) == 1)
            {
                return Copy[Mid];
            }
            return 0.5 * (Copy[Mid - 1] + Copy[Mid]);
        }

        /// <summary>
        /// True when the first offer should replace the second.
        /// A lower seconds-per-yard wins. A tie keeps the earlier stage.
        /// </summary>
        /// <param name="Candidate">The offer being considered.</param>
        /// <param name="Current">The offer held so far.</param>
        /// <returns>True when the candidate replaces the current offer.</returns>
        private static Boolean Cheaper
            (
            Offer Candidate,
            Offer Current
            )
        {
            Double Left = Candidate.SecondsPerBcy;
            Double Right = Current.SecondsPerBcy;
            if (Left < Right - 1e-9)
            {
                return true;
            }
            if (Right < Left - 1e-9)
            {
                return false;
            }
            return Candidate.Stage < Current.Stage;
        }
    }
}
