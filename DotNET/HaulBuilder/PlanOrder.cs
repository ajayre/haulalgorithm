using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace HaulBuilder
{
    /// <summary>
    /// Drive-order checks for a saved guidance plan. A later phase may not
    /// appear before an earlier one. The header hours, trip count, and stamp
    /// are taken from the same trips the plan saves.
    /// </summary>
    static public class PlanOrder
    {
        /// <summary>How many drive-order trips the report lists.</summary>
        private const Int32 SAMPLE_TRIPS = 100;

        /// <summary>FNV-1a offset basis.</summary>
        private const UInt32 FNV_OFFSET = 2166136261;

        /// <summary>FNV-1a prime.</summary>
        private const UInt32 FNV_PRIME = 16777619;

        /// <summary>
        /// Fails when drive order skips a trip or a phase goes backwards.
        /// Flow, clusters, and thin-cut are one phase. Correcting follows,
        /// then sweep-up, then finishing.
        /// </summary>
        /// <param name="Trips">Trips in planner order.</param>
        /// <param name="DriveOrder">Indexes into <paramref name="Trips"/> in the order they are driven.</param>
        /// <exception cref="ArgumentNullException">A list is null.</exception>
        /// <exception cref="InvalidOperationException">The order is not a forward phase sequence.</exception>
        static public void Require
            (
            IReadOnlyList<TripGuidance.TripCourse> Trips,
            IReadOnlyList<Int32> DriveOrder
            )
        {
            if (Trips == null)
            {
                throw new ArgumentNullException(nameof(Trips));
            }

            if (DriveOrder == null)
            {
                throw new ArgumentNullException(nameof(DriveOrder));
            }

            if (DriveOrder.Count != Trips.Count)
            {
                throw new InvalidOperationException("Drive order does not list every trip.");
            }

            Boolean[] Seen = new Boolean[Trips.Count];
            Int32 Previous = -1;
            for (Int32 Step = 0; Step < DriveOrder.Count; Step += 1)
            {
                Int32 Index = DriveOrder[Step];
                if ((Index < 0) || (Index >= Trips.Count) || Seen[Index])
                {
                    throw new InvalidOperationException("Drive order does not list every trip once.");
                }

                Seen[Index] = true;
                TripGuidance.TripCourse Trip = Trips[Index];
                Int32 Rank = PhaseRank(Trip);
                if (Rank < Previous)
                {
                    throw new InvalidOperationException(String.Format(
                        CultureInfo.InvariantCulture,
                        "Trip {0} is {1} after {2}.",
                        Trip.TripNumber,
                        PhaseName(Trip),
                        PhaseNameOfRank(Previous)));
                }

                Previous = Rank;
            }
        }

        /// <summary>
        /// Charged hours of these trips, field efficiency already applied.
        /// </summary>
        /// <param name="Trips">Trips whose productive seconds are summed.</param>
        /// <param name="Factor">Field efficiency in (0, 1], or 1 when efficiency is off.</param>
        /// <returns>Hours.</returns>
        static public Double ChargedHours
            (
            IReadOnlyList<TripGuidance.TripCourse> Trips,
            Double Factor
            )
        {
            Double Seconds = 0.0;
            foreach (TripGuidance.TripCourse Trip in Trips)
            {
                Seconds += Trip.ProductiveSeconds;
            }

            Double Scale = ((Factor > 0.0) && (Factor <= 1.0)) ? Factor : 1.0;
            return (Seconds / Scale) / Constants.SECONDS_PER_HOUR;
        }

        /// <summary>
        /// Eight-digit stamp of the drive order, the phase of each trip, and the hours.
        /// </summary>
        /// <param name="Trips">Trips in planner order.</param>
        /// <param name="DriveOrder">Indexes into <paramref name="Trips"/>.</param>
        /// <param name="Hours">Charged hours printed in the plan header.</param>
        /// <returns>Eight digits.</returns>
        static public String Stamp
            (
            IReadOnlyList<TripGuidance.TripCourse> Trips,
            IReadOnlyList<Int32> DriveOrder,
            Double Hours
            )
        {
            UInt32 Hash = FNV_OFFSET;
            Mix(ref Hash, Trips.Count.ToString(CultureInfo.InvariantCulture));
            Mix(ref Hash, Hours.ToString("F3", CultureInfo.InvariantCulture));
            foreach (Int32 Index in DriveOrder)
            {
                TripGuidance.TripCourse Trip = Trips[Index];
                Mix(ref Hash, Trip.TripNumber.ToString(CultureInfo.InvariantCulture));
                Mix(ref Hash, PhaseName(Trip));
                Mix(ref Hash, SourceName(Trip));
                Mix(ref Hash, Trip.CutBcy.ToString("F3", CultureInfo.InvariantCulture));
                Mix(ref Hash, Trip.LoadedCrossesFill ? "yes" : "no");
            }

            return (Hash % 100000000).ToString("D8", CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// Warning the viewer shows when a saved stamp is not the stamp of these trips.
        /// </summary>
        /// <param name="ShownStamp">Stamp from the plan header.</param>
        /// <param name="Trips">Trips in planner order.</param>
        /// <param name="DriveOrder">Indexes into <paramref name="Trips"/>.</param>
        /// <param name="Hours">Charged hours printed in the plan header.</param>
        /// <returns>An empty string when the stamp matches. Otherwise one sentence.</returns>
        static public String ViewerWarning
            (
            String ShownStamp,
            IReadOnlyList<TripGuidance.TripCourse> Trips,
            IReadOnlyList<Int32> DriveOrder,
            Double Hours
            )
        {
            String Actual = Stamp(Trips, DriveOrder, Hours);
            if (String.Equals(ShownStamp, Actual, StringComparison.Ordinal))
            {
                return "";
            }

            return String.Format(
                CultureInfo.InvariantCulture,
                "Stamp {0} does not match the trips ({1}).",
                ShownStamp ?? "",
                Actual);
        }

        /// <summary>
        /// Phase name in drive order: flow, correcting, sweep-up, or finishing.
        /// </summary>
        /// <param name="Trip">Trip to name.</param>
        /// <returns>The phase.</returns>
        static public String PhaseName
            (
            TripGuidance.TripCourse Trip
            )
        {
            return PhaseNameOfRank(PhaseRank(Trip));
        }

        /// <summary>
        /// Where the trip's cells came from: flow ribbon, band, correcting line, sweep-up, or finishing.
        /// </summary>
        /// <param name="Trip">Trip to name.</param>
        /// <returns>The source.</returns>
        static public String SourceName
            (
            TripGuidance.TripCourse Trip
            )
        {
            if (Trip.Finishing)
            {
                return "finishing";
            }

            if (Trip.SweepUp)
            {
                return "sweep-up";
            }

            if (Trip.Stage == HaulStage.Correcting)
            {
                return "correcting line";
            }

            if ((Trip.Stage == HaulStage.Flow) && Trip.RibbonOrder)
            {
                return "flow ribbon";
            }

            return "band";
        }

        /// <summary>
        /// True when a loaded connector sample lands on a fill cell of this trip
        /// other than the cell where the spread starts.
        /// </summary>
        /// <param name="Trip">Driven trip.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="FillCells">Fill cells of the trip, spread start first.</param>
        /// <returns>True when the loaded path crosses another fill cell.</returns>
        static public Boolean LoadedCrossesFill
            (
            TripGuidance.TripCourse Trip,
            Grid Grid,
            IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells
            )
        {
            if ((Trip == null) || (Grid == null) || (FillCells == null) || (FillCells.Count == 0))
            {
                return false;
            }

            (Int32 Bx, Int32 By) Arrival = FillCells[0].Cell;
            HashSet<(Int32 Bx, Int32 By)> Other = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in FillCells)
            {
                if (Fill.Cell != Arrival)
                {
                    Other.Add(Fill.Cell);
                }
            }

            if (Other.Count == 0)
            {
                return false;
            }

            foreach (TripGuidance.CourseSegment Segment in Trip.Segments)
            {
                // Loaded travel is the carry between the cut and the spread.
                if (Segment.IsLane || (Segment.Action != TripGuidance.GuidanceAction.Carry))
                {
                    continue;
                }

                foreach (TripGuidance.DrawnPiece Piece in Segment.Pieces)
                {
                    foreach (DubinsConnector.Pose Sample in Piece.Samples)
                    {
                        Int32 Bx = (Int32)Math.Floor((Sample.X - Grid.MinX) / Grid.GridSizeM);
                        Int32 By = (Int32)Math.Floor((Sample.Y - Grid.MinY) / Grid.GridSizeM);
                        if (Other.Contains((Bx, By)))
                        {
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Writes the first 100 trips in drive order.
        /// </summary>
        /// <param name="Text">Report being built.</param>
        /// <param name="Trips">Trips in planner order.</param>
        /// <param name="DriveOrder">Indexes into <paramref name="Trips"/>.</param>
        static public void AppendDriveSample
            (
            StringBuilder Text,
            IReadOnlyList<TripGuidance.TripCourse> Trips,
            IReadOnlyList<Int32> DriveOrder
            )
        {
            Text.AppendLine("First 100 trips in drive order");
            Int32 Limit = Math.Min(SAMPLE_TRIPS, DriveOrder.Count);
            for (Int32 Step = 0; Step < Limit; Step += 1)
            {
                TripGuidance.TripCourse Trip = Trips[DriveOrder[Step]];
                Text.AppendLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "drive {0} trip {1} stage {2} source {3} cut {4:F4} CY loaded-crosses-fill {5}",
                    Step + 1,
                    Trip.TripNumber,
                    PhaseName(Trip),
                    SourceName(Trip),
                    Trip.CutBcy,
                    Trip.LoadedCrossesFill ? "yes" : "no"));
            }

            Text.AppendLine();
        }

        /// <summary>
        /// Rank of a trip. A smaller rank is driven first.
        /// </summary>
        /// <param name="Trip">Trip to rank.</param>
        /// <returns>0 flow, 1 correcting, 2 sweep-up, 3 finishing.</returns>
        private static Int32 PhaseRank
            (
            TripGuidance.TripCourse Trip
            )
        {
            if (Trip.Finishing)
            {
                return 3;
            }

            if (Trip.SweepUp)
            {
                return 2;
            }

            if (Trip.Stage == HaulStage.Correcting)
            {
                return 1;
            }

            return 0;
        }

        /// <summary>
        /// Phase name for a rank.
        /// </summary>
        /// <param name="Rank">0 through 3.</param>
        /// <returns>The phase.</returns>
        private static String PhaseNameOfRank
            (
            Int32 Rank
            )
        {
            if (Rank >= 3)
            {
                return "finishing";
            }

            if (Rank == 2)
            {
                return "sweep-up";
            }

            if (Rank == 1)
            {
                return "correcting";
            }

            return "flow";
        }

        /// <summary>
        /// Mixes one field into the stamp.
        /// </summary>
        /// <param name="Hash">Hash so far.</param>
        /// <param name="Text">Field text.</param>
        private static void Mix
            (
            ref UInt32 Hash,
            String Text
            )
        {
            foreach (Char Character in Text)
            {
                Hash ^= Character;
                Hash *= FNV_PRIME;
            }

            Hash ^= (UInt32)';';
            Hash *= FNV_PRIME;
        }
    }
}
