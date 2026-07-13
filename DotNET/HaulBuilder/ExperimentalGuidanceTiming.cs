using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Estimates experimental lifecycle time from the final driven segment geometry.
    /// </summary>
    public static class ExperimentalGuidanceTiming
    {
        /// <summary>
        /// Configurable speeds and event durations for final-guidance timing.
        /// </summary>
        public sealed class Options
        {
            /// <summary>Gets or sets cut speed in miles per hour.</summary>
            public Double CutSpeedMph { get; set; } = Constants.DEFAULT_LOADING_SPEED_MPH;

            /// <summary>Gets or sets fill speed in miles per hour.</summary>
            public Double FillSpeedMph { get; set; } = Constants.DEFAULT_DUMPING_SPEED_MPH;

            /// <summary>Gets or sets grade speed in miles per hour.</summary>
            public Double GradeSpeedMph { get; set; } = Constants.DEFAULT_GRADE_SPEED_MPH;

            /// <summary>Gets or sets transit speed in miles per hour.</summary>
            public Double TransitSpeedMph { get; set; } = Constants.DEFAULT_RETURN_SPEED_MPH;

            /// <summary>Gets or sets turn, lineup, and exit speed in miles per hour.</summary>
            public Double TurnSpeedMph { get; set; } = Constants.DEFAULT_TURN_SPEED_MPH;

            /// <summary>Gets or sets explicit manual-assist speed in miles per hour.</summary>
            public Double ManualAssistSpeedMph { get; set; } = Constants.DEFAULT_TURN_SPEED_MPH;

            /// <summary>Gets or sets longitudinal acceleration in metres per second squared.</summary>
            public Double AccelerationMps2 { get; set; } =
                Constants.DEFAULT_RETURN_SPEED_MPH * Constants.MPH_TO_MPS / Constants.DEFAULT_ACCEL_TIME_S;

            /// <summary>Gets or sets longitudinal deceleration in metres per second squared.</summary>
            public Double DecelerationMps2 { get; set; } =
                Constants.DEFAULT_RETURN_SPEED_MPH * Constants.MPH_TO_MPS / Constants.DEFAULT_DECEL_TIME_S;

            /// <summary>Gets or sets fixed operator takeover overhead per assist.</summary>
            public Double ManualAssistOverheadS { get; set; }

            /// <summary>Gets or sets fixed blade-lower event duration.</summary>
            public Double BladeLowerS { get; set; }

            /// <summary>Gets or sets fixed blade-raise event duration.</summary>
            public Double BladeRaiseS { get; set; }

            /// <summary>Gets or sets productive field-efficiency fraction.</summary>
            public Double FieldEfficiency { get; set; } = 1.0;
        }

        /// <summary>
        /// Timing summary by lifecycle phase.
        /// </summary>
        public sealed class Result
        {
            /// <summary>Initializes one timing result.</summary>
            public Result
                (
                Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseSeconds,
                Double MotionSeconds,
                Double EventSeconds,
                Double FieldEfficiencyAllowanceSeconds
                )
            {
                this.PhaseSeconds = PhaseSeconds;
                this.MotionSeconds = MotionSeconds;
                this.EventSeconds = EventSeconds;
                this.FieldEfficiencyAllowanceSeconds = FieldEfficiencyAllowanceSeconds;
            }

            /// <summary>Gets motion and phase-owned event seconds.</summary>
            public Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseSeconds { get; }

            /// <summary>Gets modeled motion seconds.</summary>
            public Double MotionSeconds { get; }

            /// <summary>Gets explicit operational event seconds.</summary>
            public Double EventSeconds { get; }

            /// <summary>Gets separately reported field-efficiency allowance.</summary>
            public Double FieldEfficiencyAllowanceSeconds { get; }

            /// <summary>Gets reported total seconds.</summary>
            public Double TotalSeconds => MotionSeconds + EventSeconds + FieldEfficiencyAllowanceSeconds;
        }

        /// <summary>
        /// Times every final lifecycle segment once, independent of waypoint sampling density.
        /// </summary>
        static public Result Estimate
            (
            ExperimentalLifecyclePlanner.Result Guidance,
            Options Settings
            )
        {
            if (Guidance == null) throw new ArgumentNullException(nameof(Guidance));
            if (Settings == null) throw new ArgumentNullException(nameof(Settings));
            if ((Settings.AccelerationMps2 <= 0.0) || (Settings.DecelerationMps2 <= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(Settings));
            }
            if ((Settings.FieldEfficiency <= 0.0) || (Settings.FieldEfficiency > 1.0))
            {
                throw new ArgumentOutOfRangeException(nameof(Settings.FieldEfficiency));
            }

            Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseSeconds =
                new Dictionary<ExperimentalLifecyclePlanner.Phase, Double>();
            Double MotionSeconds = 0.0;
            Double EventSeconds = 0.0;
            Boolean PreviousBladeDown = false;
            foreach (ExperimentalLifecyclePlanner.Segment Segment in Guidance.Segments)
            {
                Double LengthM = PolylineLength(Segment.Points);
                Double SpeedMps = PhaseSpeedMph(Segment.Phase, Settings) * Constants.MPH_TO_MPS;
                Double SegmentSeconds = MotionTime(
                    LengthM, SpeedMps, Settings.AccelerationMps2, Settings.DecelerationMps2);
                Add(PhaseSeconds, Segment.Phase, SegmentSeconds);
                MotionSeconds += SegmentSeconds;

                if (Segment.BladeDown && !PreviousBladeDown)
                {
                    EventSeconds += Settings.BladeLowerS;
                    Add(PhaseSeconds, Segment.Phase, Settings.BladeLowerS);
                }
                if (!Segment.BladeDown && PreviousBladeDown)
                {
                    EventSeconds += Settings.BladeRaiseS;
                    Add(PhaseSeconds, Segment.Phase, Settings.BladeRaiseS);
                }
                if (Segment.Phase == ExperimentalLifecyclePlanner.Phase.ManualAssist)
                {
                    EventSeconds += Settings.ManualAssistOverheadS;
                    Add(PhaseSeconds, Segment.Phase, Settings.ManualAssistOverheadS);
                }
                PreviousBladeDown = Segment.BladeDown;
            }

            Double ProductiveSeconds = MotionSeconds + EventSeconds;
            Double AllowanceSeconds = (ProductiveSeconds / Settings.FieldEfficiency) - ProductiveSeconds;
            return new Result(PhaseSeconds, MotionSeconds, EventSeconds, AllowanceSeconds);
        }

        /// <summary>Maps each lifecycle phase to its configured maximum speed.</summary>
        private static Double PhaseSpeedMph
            (
            ExperimentalLifecyclePlanner.Phase Phase,
            Options Settings
            )
        {
            return Phase switch
            {
                ExperimentalLifecyclePlanner.Phase.Cut => Settings.CutSpeedMph,
                ExperimentalLifecyclePlanner.Phase.Fill => Settings.FillSpeedMph,
                ExperimentalLifecyclePlanner.Phase.Grade => Settings.GradeSpeedMph,
                ExperimentalLifecyclePlanner.Phase.ManualAssist => Settings.ManualAssistSpeedMph,
                ExperimentalLifecyclePlanner.Phase.LineupCut => Settings.TurnSpeedMph,
                ExperimentalLifecyclePlanner.Phase.LineupFill => Settings.TurnSpeedMph,
                ExperimentalLifecyclePlanner.Phase.LineupGrade => Settings.TurnSpeedMph,
                ExperimentalLifecyclePlanner.Phase.ExitCut => Settings.TurnSpeedMph,
                ExperimentalLifecyclePlanner.Phase.ExitFill => Settings.TurnSpeedMph,
                ExperimentalLifecyclePlanner.Phase.ExitGrade => Settings.TurnSpeedMph,
                ExperimentalLifecyclePlanner.Phase.TurnToGrade => Settings.TurnSpeedMph,
                _ => Settings.TransitSpeedMph,
            };
        }

        /// <summary>Computes rest-to-rest trapezoidal or triangular motion time.</summary>
        private static Double MotionTime
            (
            Double DistanceM,
            Double MaximumSpeedMps,
            Double AccelerationMps2,
            Double DecelerationMps2
            )
        {
            if ((DistanceM <= 0.0) || (MaximumSpeedMps <= 0.0)) return 0.0;
            Double AccelerationDistanceM = MaximumSpeedMps * MaximumSpeedMps
                / (2.0 * AccelerationMps2);
            Double DecelerationDistanceM = MaximumSpeedMps * MaximumSpeedMps
                / (2.0 * DecelerationMps2);
            if (DistanceM >= AccelerationDistanceM + DecelerationDistanceM)
            {
                Double CruiseM = DistanceM - AccelerationDistanceM - DecelerationDistanceM;
                return (MaximumSpeedMps / AccelerationMps2)
                    + (CruiseM / MaximumSpeedMps)
                    + (MaximumSpeedMps / DecelerationMps2);
            }
            Double PeakMps = Math.Sqrt(
                (2.0 * DistanceM * AccelerationMps2 * DecelerationMps2)
                / (AccelerationMps2 + DecelerationMps2));
            return (PeakMps / AccelerationMps2) + (PeakMps / DecelerationMps2);
        }

        /// <summary>Computes final polyline distance.</summary>
        private static Double PolylineLength
            (
            List<(Double X, Double Y)> Points
            )
        {
            Double TotalM = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double Dx = Points[Index].X - Points[Index - 1].X;
                Double Dy = Points[Index].Y - Points[Index - 1].Y;
                TotalM += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            return TotalM;
        }

        /// <summary>Adds seconds to one phase bucket.</summary>
        private static void Add
            (
            Dictionary<ExperimentalLifecyclePlanner.Phase, Double> Output,
            ExperimentalLifecyclePlanner.Phase Phase,
            Double Seconds
            )
        {
            Output.TryGetValue(Phase, out Double Existing);
            Output[Phase] = Existing + Seconds;
        }
    }
}
