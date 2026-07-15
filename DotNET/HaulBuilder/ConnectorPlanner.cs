using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder
{
    /// <summary>
    /// Builds isolated forward-only minimum-radius connectors for guidance coverage runs.
    /// </summary>
    public static class ConnectorPlanner
    {
        private const Double TWO_PI = 2.0 * Math.PI;
        private const Double NUMERIC_TOLERANCE = 1e-9;

        /// <summary>
        /// Planar centerline pose with compass heading.
        /// </summary>
        public readonly record struct Pose
        {
            /// <summary>
            /// Initializes a planar pose.
            /// </summary>
            /// <param name="X">Easting in metres.</param>
            /// <param name="Y">Northing in metres.</param>
            /// <param name="HeadingDeg">Compass heading in degrees.</param>
            public Pose
                (
                Double X,
                Double Y,
                Double HeadingDeg
                )
            {
                // Normalize heading once so candidate comparison stays deterministic.
                this.X = X;
                this.Y = Y;
                this.HeadingDeg = ((HeadingDeg % 360.0) + 360.0) % 360.0;
            }

            /// <summary>Gets easting in metres.</summary>
            public Double X { get; }

            /// <summary>Gets northing in metres.</summary>
            public Double Y { get; }

            /// <summary>Gets compass heading in degrees.</summary>
            public Double HeadingDeg { get; }
        }

        /// <summary>
        /// Outcome of attempting one strict in-field automatic connector.
        /// </summary>
        public sealed class Result
        {
            /// <summary>
            /// Initializes a connector result.
            /// </summary>
            /// <param name="Success">Whether an in-field connector was found.</param>
            /// <param name="Word">Selected Dubins word or empty text.</param>
            /// <param name="Points">Sampled centerline including exact start and end.</param>
            /// <param name="LengthM">Sampled centerline length in metres.</param>
            /// <param name="FailureReason">Reason no connector was accepted.</param>
            public Result
                (
                Boolean Success,
                String Word,
                List<(Double X, Double Y)> Points,
                Double LengthM,
                String FailureReason
                )
            {
                // Failed results remain data so diagnostics never invent a straight fallback.
                this.Success = Success;
                this.Word = Word;
                this.Points = Points;
                this.LengthM = LengthM;
                this.FailureReason = FailureReason;
            }

            /// <summary>Gets whether an in-field connector was found.</summary>
            public Boolean Success { get; }

            /// <summary>Gets the selected Dubins word.</summary>
            public String Word { get; }

            /// <summary>Gets sampled centerline including exact endpoints.</summary>
            public List<(Double X, Double Y)> Points { get; }

            /// <summary>Gets sampled centerline length in metres.</summary>
            public Double LengthM { get; }

            /// <summary>Gets failure reason when no connector was accepted.</summary>
            public String FailureReason { get; }
        }

        private sealed class Candidate
        {
            /// <summary>
            /// Initializes one feasible mathematical Dubins candidate.
            /// </summary>
            public Candidate
                (
                String Word,
                (Double T, Double P, Double Q) Parameters,
                Double LengthM
                )
            {
                // Retain normalized parameters until strict field sampling accepts the candidate.
                this.Word = Word;
                this.Parameters = Parameters;
                this.LengthM = LengthM;
            }

            public String Word { get; }
            public (Double T, Double P, Double Q) Parameters { get; }
            public Double LengthM { get; }
        }

        private static readonly Dictionary<String, String[]> SegmentTypes =
            new Dictionary<String, String[]>
            {
                { "LSL", new[] { "L", "S", "L" } },
                { "LSR", new[] { "L", "S", "R" } },
                { "RSL", new[] { "R", "S", "L" } },
                { "RSR", new[] { "R", "S", "R" } },
                { "RLR", new[] { "R", "L", "R" } },
                { "LRL", new[] { "L", "R", "L" } },
            };

        /// <summary>
        /// Selects the shortest sampled Dubins connector wholly inside the raw field.
        /// </summary>
        /// <param name="Start">Start pose.</param>
        /// <param name="End">End pose.</param>
        /// <param name="MinimumRadiusM">Minimum turning radius in metres.</param>
        /// <param name="SampleSpacingM">Maximum sampling interval in metres.</param>
        /// <param name="Field">Raw field mask.</param>
        /// <returns>Strict success or explicit failure; never a straight fallback.</returns>
        static public Result Connect
            (
            Pose Start,
            Pose End,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if (MinimumRadiusM <= 0.0) throw new ArgumentOutOfRangeException(nameof(MinimumRadiusM));
            if (SampleSpacingM <= 0.0) throw new ArgumentOutOfRangeException(nameof(SampleSpacingM));
            if (Field == null) throw new ArgumentNullException(nameof(Field));

            if (!Field.ContainsWorldXy(Start.X, Start.Y) || !Field.ContainsWorldXy(End.X, End.Y))
            {
                return new Result(false, String.Empty, new List<(Double X, Double Y)>(), 0.0,
                    "Connector endpoint is outside the raw field.");
            }

            List<Candidate> Candidates = BuildCandidates(Start, End, MinimumRadiusM)
                .OrderBy(Candidate => Candidate.LengthM)
                .ThenBy(Candidate => Candidate.Word, StringComparer.Ordinal)
                .ToList();
            foreach (Candidate Candidate in Candidates)
            {
                List<(Double X, Double Y)> Points = Sample(
                    Start,
                    End,
                    MinimumRadiusM,
                    Candidate,
                    SampleSpacingM);
                if (Points.All(Point => Field.ContainsWorldXy(Point.X, Point.Y)))
                {
                    return new Result(
                        true,
                        Candidate.Word,
                        Points,
                        PolylineLength(Points),
                        String.Empty);
                }
            }

            return new Result(false, String.Empty, new List<(Double X, Double Y)>(), 0.0,
                Candidates.Count == 0
                    ? "No forward-only Dubins word is mathematically feasible."
                    : "Every minimum-radius Dubins candidate leaves the raw field.");
        }

        /// <summary>
        /// Builds all mathematically feasible Dubins words between two poses.
        /// </summary>
        private static List<Candidate> BuildCandidates
            (
            Pose Start,
            Pose End,
            Double RadiusM
            )
        {
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double NormalizedDistance = Math.Sqrt((Dx * Dx) + (Dy * Dy)) / RadiusM;
            Double Theta = (NormalizedDistance > 0.0)
                ? Mod2Pi(Math.Atan2(Dy, Dx))
                : 0.0;
            Double Alpha = Mod2Pi(CompassToMath(Start.HeadingDeg) - Theta);
            Double Beta = Mod2Pi(CompassToMath(End.HeadingDeg) - Theta);
            Dictionary<String, (Double T, Double P, Double Q)> Words = WordsFor(
                Alpha,
                Beta,
                NormalizedDistance);
            List<Candidate> Output = new List<Candidate>();
            foreach (KeyValuePair<String, (Double T, Double P, Double Q)> Word in Words)
            {
                Double LengthM = (Word.Value.T + Word.Value.P + Word.Value.Q) * RadiusM;
                Output.Add(new Candidate(Word.Key, Word.Value, LengthM));
            }
            return Output;
        }

        /// <summary>
        /// Evaluates the six standard Dubins word formulae.
        /// </summary>
        private static Dictionary<String, (Double T, Double P, Double Q)> WordsFor
            (
            Double Alpha,
            Double Beta,
            Double Distance
            )
        {
            Double SinAlpha = Math.Sin(Alpha);
            Double SinBeta = Math.Sin(Beta);
            Double CosAlpha = Math.Cos(Alpha);
            Double CosBeta = Math.Cos(Beta);
            Double CosDifference = Math.Cos(Alpha - Beta);
            Double DistanceSquared = Distance * Distance;
            Dictionary<String, (Double T, Double P, Double Q)> Output =
                new Dictionary<String, (Double T, Double P, Double Q)>();

            Double Temporary = Distance + SinAlpha - SinBeta;
            Double StraightSquared = 2.0 + DistanceSquared - (2.0 * CosDifference)
                + (2.0 * Distance * (SinAlpha - SinBeta));
            if (StraightSquared >= 0.0)
            {
                Double Angle = Math.Atan2(CosBeta - CosAlpha, Temporary);
                Output["LSL"] = (
                    Mod2Pi(Angle - Alpha),
                    Math.Sqrt(StraightSquared),
                    Mod2Pi(Beta - Angle));
            }

            Temporary = Distance - SinAlpha + SinBeta;
            StraightSquared = 2.0 + DistanceSquared - (2.0 * CosDifference)
                + (2.0 * Distance * (SinBeta - SinAlpha));
            if (StraightSquared >= 0.0)
            {
                Double Angle = Math.Atan2(CosAlpha - CosBeta, Temporary);
                Output["RSR"] = (
                    Mod2Pi(Alpha - Angle),
                    Math.Sqrt(StraightSquared),
                    Mod2Pi(Angle - Beta));
            }

            StraightSquared = -2.0 + DistanceSquared + (2.0 * CosDifference)
                + (2.0 * Distance * (SinAlpha + SinBeta));
            if (StraightSquared >= 0.0)
            {
                Double Straight = Math.Sqrt(StraightSquared);
                Double Angle = Math.Atan2(-CosAlpha - CosBeta, Distance + SinAlpha + SinBeta)
                    - Math.Atan2(-2.0, Straight);
                Output["LSR"] = (
                    Mod2Pi(Angle - Alpha),
                    Straight,
                    Mod2Pi(Angle - Mod2Pi(Beta)));
            }

            StraightSquared = -2.0 + DistanceSquared + (2.0 * CosDifference)
                - (2.0 * Distance * (SinAlpha + SinBeta));
            if (StraightSquared >= 0.0)
            {
                Double Straight = Math.Sqrt(StraightSquared);
                Double Angle = Math.Atan2(CosAlpha + CosBeta, Distance - SinAlpha - SinBeta)
                    - Math.Atan2(2.0, Straight);
                Output["RSL"] = (
                    Mod2Pi(Alpha - Angle),
                    Straight,
                    Mod2Pi(Beta - Angle));
            }

            Temporary = (6.0 - DistanceSquared + (2.0 * CosDifference)
                + (2.0 * Distance * (SinAlpha - SinBeta))) / 8.0;
            Double Phi = Math.Atan2(CosAlpha - CosBeta, Distance - SinAlpha + SinBeta);
            if (Math.Abs(Temporary) <= 1.0)
            {
                Double Middle = Mod2Pi(TWO_PI - Math.Acos(Temporary));
                Double First = Mod2Pi(Alpha - Phi + Mod2Pi(Middle / 2.0));
                Output["RLR"] = (
                    First,
                    Middle,
                    Mod2Pi(Alpha - Beta - First + Mod2Pi(Middle)));
            }

            Temporary = (6.0 - DistanceSquared + (2.0 * CosDifference)
                + (2.0 * Distance * (SinBeta - SinAlpha))) / 8.0;
            Phi = Math.Atan2(CosAlpha - CosBeta, Distance + SinAlpha - SinBeta);
            if (Math.Abs(Temporary) <= 1.0)
            {
                Double Middle = Mod2Pi(TWO_PI - Math.Acos(Temporary));
                Double First = Mod2Pi(-Alpha - Phi + (Middle / 2.0));
                Output["LRL"] = (
                    First,
                    Middle,
                    Mod2Pi(Mod2Pi(Beta) - Alpha - First + Mod2Pi(Middle)));
            }

            return Output;
        }

        /// <summary>
        /// Samples one Dubins candidate and pins exact endpoints.
        /// </summary>
        private static List<(Double X, Double Y)> Sample
            (
            Pose Start,
            Pose End,
            Double RadiusM,
            Candidate Candidate,
            Double SampleSpacingM
            )
        {
            String[] Types = SegmentTypes[Candidate.Word];
            Double[] Parameters =
            {
                Candidate.Parameters.T,
                Candidate.Parameters.P,
                Candidate.Parameters.Q,
            };
            Int32 SampleCount = Math.Max(
                1,
                (Int32)Math.Ceiling(Candidate.LengthM / SampleSpacingM));
            List<(Double X, Double Y)> Output = new List<(Double X, Double Y)>
            {
                (Start.X, Start.Y),
            };
            Double StartMathHeading = CompassToMath(Start.HeadingDeg);
            for (Int32 SampleIndex = 1; SampleIndex < SampleCount; SampleIndex += 1)
            {
                Double NormalizedDistance =
                    (Candidate.LengthM * SampleIndex / SampleCount) / RadiusM;
                (Double X, Double Y, Double Heading) Pose = (0.0, 0.0, StartMathHeading);
                Double Remaining = NormalizedDistance;
                for (Int32 SegmentIndex = 0; SegmentIndex < Parameters.Length; SegmentIndex += 1)
                {
                    Double SegmentLength = Parameters[SegmentIndex];
                    if (Remaining <= SegmentLength + NUMERIC_TOLERANCE)
                    {
                        Pose = Advance(Pose, Remaining, Types[SegmentIndex]);
                        break;
                    }
                    Pose = Advance(Pose, SegmentLength, Types[SegmentIndex]);
                    Remaining -= SegmentLength;
                }
                Output.Add((
                    Start.X + (Pose.X * RadiusM),
                    Start.Y + (Pose.Y * RadiusM)));
            }
            Output.Add((End.X, End.Y));
            return Output;
        }

        /// <summary>
        /// Advances one normalized left, right, or straight segment.
        /// </summary>
        private static (Double X, Double Y, Double Heading) Advance
            (
            (Double X, Double Y, Double Heading) Input,
            Double Distance,
            String SegmentType
            )
        {
            Double SinHeading = Math.Sin(Input.Heading);
            Double CosHeading = Math.Cos(Input.Heading);
            if (SegmentType == "L")
            {
                return (
                    Input.X + Math.Sin(Input.Heading + Distance) - SinHeading,
                    Input.Y - Math.Cos(Input.Heading + Distance) + CosHeading,
                    Input.Heading + Distance);
            }
            if (SegmentType == "R")
            {
                return (
                    Input.X - Math.Sin(Input.Heading - Distance) + SinHeading,
                    Input.Y + Math.Cos(Input.Heading - Distance) - CosHeading,
                    Input.Heading - Distance);
            }
            return (
                Input.X + (CosHeading * Distance),
                Input.Y + (SinHeading * Distance),
                Input.Heading);
        }

        /// <summary>
        /// Wraps radians into the standard Dubins interval.
        /// </summary>
        private static Double Mod2Pi
            (
            Double Angle
            )
        {
            return Angle - (TWO_PI * Math.Floor(Angle / TWO_PI));
        }

        /// <summary>
        /// Converts compass degrees to mathematical radians.
        /// </summary>
        private static Double CompassToMath
            (
            Double HeadingDeg
            )
        {
            return (Math.PI / 2.0) - (HeadingDeg * Math.PI / 180.0);
        }

        /// <summary>
        /// Computes sampled polyline length.
        /// </summary>
        private static Double PolylineLength
            (
            List<(Double X, Double Y)> Points
            )
        {
            Double Total = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double Dx = Points[Index].X - Points[Index - 1].X;
                Double Dy = Points[Index].Y - Points[Index - 1].Y;
                Total += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            return Total;
        }
    }
}
