using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies timing derives from final geometry and explicit lifecycle events.
    /// </summary>
    public sealed class ExperimentalGuidanceTimingTests
    {
        /// <summary>Verifies user-facing speeds feed the guidance timing model.</summary>
        [Fact]
        public void Options_MapsUserFacingHaulSettings
            (
            )
        {
            HaulBuilderOptions Source = new HaulBuilderOptions
            {
                HaulSpeedMph = 9.0,
                ReturnSpeedMph = 12.0,
                TurnSpeedMph = 3.0,
            };

            ExperimentalGuidanceTiming.Options Options =
                new ExperimentalGuidanceTiming.Options(Source);

            Assert.Equal(9.0, Options.LoadedTravelSpeedMph, 9);
            Assert.Equal(12.0, Options.UnloadedTravelSpeedMph, 9);
            Assert.Equal(3.0, Options.TurnSpeedMph, 9);
        }

        /// <summary>
        /// Verifies subdividing identical geometry does not change modeled time.
        /// </summary>
        [Fact]
        public void Estimate_IsInvariantToWaypointSamplingDensity
            (
            )
        {
            ExperimentalLifecyclePlanner.Result Sparse = MakeGuidance(
                ExperimentalLifecyclePlanner.Phase.Transit,
                new List<(Double X, Double Y)> { (0.0, 0.0), (100.0, 0.0) });
            List<(Double X, Double Y)> DensePoints = Enumerable.Range(0, 101)
                .Select(Index => ((Double)Index, 0.0)).ToList();
            ExperimentalLifecyclePlanner.Result Dense = MakeGuidance(
                ExperimentalLifecyclePlanner.Phase.Transit, DensePoints);
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options();

            ExperimentalGuidanceTiming.Result SparseTime =
                ExperimentalGuidanceTiming.Estimate(Sparse, Options);
            ExperimentalGuidanceTiming.Result DenseTime =
                ExperimentalGuidanceTiming.Estimate(Dense, Options);

            Assert.Equal(SparseTime.TotalSeconds, DenseTime.TotalSeconds, 9);
        }

        /// <summary>
        /// Verifies a lifecycle label boundary does not imply a physical stop.
        /// </summary>
        [Fact]
        public void Estimate_IsInvariantToContiguousSegmentPartitioning
            (
            )
        {
            ExperimentalLifecyclePlanner.Result Combined = MakeGuidance(
                ExperimentalLifecyclePlanner.Phase.Transit,
                new List<(Double X, Double Y)> { (0.0, 0.0), (100.0, 0.0) });
            ExperimentalLifecyclePlanner.Result Partitioned = MakeGuidance(
                new List<ExperimentalLifecyclePlanner.Segment>
                {
                    MakeSegment(ExperimentalLifecyclePlanner.Phase.Transit,
                        (0.0, 0.0), (40.0, 0.0)),
                    MakeSegment(ExperimentalLifecyclePlanner.Phase.Transit,
                        (40.0, 0.0), (100.0, 0.0)),
                });
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options();

            ExperimentalGuidanceTiming.Result CombinedTime =
                ExperimentalGuidanceTiming.Estimate(Combined, Options);
            ExperimentalGuidanceTiming.Result PartitionedTime =
                ExperimentalGuidanceTiming.Estimate(Partitioned, Options);

            Assert.Equal(CombinedTime.TotalSeconds, PartitionedTime.TotalSeconds, 9);
            Assert.Equal(0, PartitionedTime.ForcedStopCount);
        }

        /// <summary>Verifies the trapezoidal profile against a hand calculation.</summary>
        [Fact]
        public void Estimate_MatchesAnalyticTrapezoidalProfile
            (
            )
        {
            ExperimentalLifecyclePlanner.Result Guidance = MakeGuidance(
                ExperimentalLifecyclePlanner.Phase.Transit,
                new List<(Double X, Double Y)> { (0.0, 0.0), (100.0, 0.0) });
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options
            {
                UnloadedTravelSpeedMph = 10.0 / Constants.MPH_TO_MPS,
                AccelerationMps2 = 2.0,
                DecelerationMps2 = 2.0,
            };

            ExperimentalGuidanceTiming.Result Time =
                ExperimentalGuidanceTiming.Estimate(Guidance, Options);

            Assert.Equal(15.0, Time.MotionSeconds, 9);
            Assert.Equal(10.0, Time.ConstantSpeedSeconds, 9);
            Assert.Equal(100.0, Time.DistanceM, 9);
        }

        /// <summary>Verifies speed changes at a phase boundary occur without stopping.</summary>
        [Fact]
        public void Estimate_CarriesSpeedAcrossPhaseBoundary
            (
            )
        {
            ExperimentalLifecyclePlanner.Segment Transit = MakeSegment(
                ExperimentalLifecyclePlanner.Phase.Transit,
                (0.0, 0.0), (100.0, 0.0));
            ExperimentalLifecyclePlanner.Segment Cut = MakeSegment(
                ExperimentalLifecyclePlanner.Phase.Cut,
                (100.0, 0.0), (200.0, 0.0));
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options();
            ExperimentalGuidanceTiming.Result Chained = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(new List<ExperimentalLifecyclePlanner.Segment> { Transit, Cut }),
                Options);
            Double SeparatelyStopped = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(new List<ExperimentalLifecyclePlanner.Segment> { Transit }),
                Options).MotionSeconds
                + ExperimentalGuidanceTiming.Estimate(
                    MakeGuidance(new List<ExperimentalLifecyclePlanner.Segment> { Cut }),
                    Options).MotionSeconds;

            Assert.True(Chained.MotionSeconds < SeparatelyStopped);
            Assert.Equal(0, Chained.ForcedStopCount);
        }

        /// <summary>
        /// Verifies minimum-radius transit uses turn speed and gentler transit
        /// linearly approaches the payload-dependent full speed.
        /// </summary>
        [Fact]
        public void Estimate_InterpolatesCurveSpeedByTurnAngle
            (
            )
        {
            const Double MINIMUM_RADIUS_M = 10.0;
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options
            {
                MinimumTurningRadiusM = MINIMUM_RADIUS_M,
                LoadedTravelSpeedMph = 10.0,
                TurnSpeedMph = 4.0,
            };
            ExperimentalGuidanceTiming.Result Tight = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.Transit,
                    MakeSemicircle(MINIMUM_RADIUS_M, 1001), true), Options);
            ExperimentalGuidanceTiming.Result Gentle = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.Transit,
                    MakeSemicircle(2.0 * MINIMUM_RADIUS_M, 1001), true), Options);
            Double TightAverageLimitMph = Tight.DistanceM / Tight.ConstantSpeedSeconds
                / Constants.MPH_TO_MPS;
            Double GentleAverageLimitMph = Gentle.DistanceM / Gentle.ConstantSpeedSeconds
                / Constants.MPH_TO_MPS;

            Assert.InRange(TightAverageLimitMph, 4.0, 4.02);
            Assert.InRange(GentleAverageLimitMph, 6.98, 7.02);
            Assert.True(Tight.CurvatureLimitedDistanceM > 0.0);
            Assert.True(Gentle.CurvatureLimitedDistanceM > 0.0);
        }

        /// <summary>Verifies straight travel uses loaded or unloaded user speed.</summary>
        [Fact]
        public void Estimate_UsesPayloadDependentFullTravelSpeed
            (
            )
        {
            List<(Double X, Double Y)> Points =
                new List<(Double X, Double Y)> { (0.0, 0.0), (1000.0, 0.0) };
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options
            {
                LoadedTravelSpeedMph = 10.0,
                UnloadedTravelSpeedMph = 13.0,
            };
            ExperimentalGuidanceTiming.Result Loaded = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.Transit, Points, true), Options);
            ExperimentalGuidanceTiming.Result Unloaded = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.Transit, Points, false), Options);

            Assert.Equal(10.0, Loaded.DistanceM / Loaded.ConstantSpeedSeconds
                / Constants.MPH_TO_MPS, 9);
            Assert.Equal(13.0, Unloaded.DistanceM / Unloaded.ConstantSpeedSeconds
                / Constants.MPH_TO_MPS, 9);
            Assert.Equal(1000.0, Loaded.LoadedDistanceM, 9);
            Assert.Equal(1000.0, Unloaded.UnloadedDistanceM, 9);
        }

        /// <summary>
        /// Verifies a straight blade-up lineup is not artificially held at turn speed.
        /// </summary>
        [Fact]
        public void Estimate_StraightLineupUsesFullTravelSpeed
            (
            )
        {
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options
            {
                UnloadedTravelSpeedMph = 13.0,
                TurnSpeedMph = 4.0,
            };
            ExperimentalGuidanceTiming.Result Lineup = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.LineupCut,
                    new List<(Double X, Double Y)> { (0.0, 0.0), (1000.0, 0.0) }),
                Options);

            Assert.Equal(13.0, Lineup.DistanceM / Lineup.ConstantSpeedSeconds
                / Constants.MPH_TO_MPS, 9);
        }

        /// <summary>Verifies circular timing is stable as arc sampling is refined.</summary>
        [Fact]
        public void Estimate_CircularTimingIsStableAcrossSamplingDensity
            (
            )
        {
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options();
            ExperimentalGuidanceTiming.Result Sparse = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.Transit,
                    MakeSemicircle(25.0, 159)), Options);
            ExperimentalGuidanceTiming.Result Dense = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.Transit,
                    MakeSemicircle(25.0, 317)), Options);
            Double SparseSecondsPerM = Sparse.MotionSeconds / Sparse.DistanceM;
            Double DenseSecondsPerM = Dense.MotionSeconds / Dense.DistanceM;

            Assert.InRange(
                Math.Abs(SparseSecondsPerM - DenseSecondsPerM) / DenseSecondsPerM,
                0.0, 0.01);
            Assert.InRange(Sparse.MinimumObservedCurveRadiusM, 24.8, 25.2);
            Assert.InRange(Dense.MinimumObservedCurveRadiusM, 24.8, 25.2);
        }

        /// <summary>Verifies straight manual guidance uses unloaded travel speed.</summary>
        [Fact]
        public void Estimate_StraightManualAssistUsesUnloadedTravelSpeed
            (
            )
        {
            List<(Double X, Double Y)> Points =
                new List<(Double X, Double Y)> { (0.0, 0.0), (100.0, 0.0) };
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options();
            ExperimentalGuidanceTiming.Result Manual = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.ManualAssist, Points), Options);
            ExperimentalGuidanceTiming.Result Transit = ExperimentalGuidanceTiming.Estimate(
                MakeGuidance(ExperimentalLifecyclePlanner.Phase.Transit, Points), Options);

            Assert.Equal(Transit.MotionSeconds, Manual.MotionSeconds, 9);
        }

        /// <summary>Verifies a geometry discontinuity is conservatively timed as a stop.</summary>
        [Fact]
        public void Estimate_DiscontinuousGeometryForcesStop
            (
            )
        {
            ExperimentalLifecyclePlanner.Result Guidance = MakeGuidance(
                new List<ExperimentalLifecyclePlanner.Segment>
                {
                    MakeSegment(ExperimentalLifecyclePlanner.Phase.Transit,
                        (0.0, 0.0), (50.0, 0.0)),
                    MakeSegment(ExperimentalLifecyclePlanner.Phase.Transit,
                        (100.0, 0.0), (150.0, 0.0)),
                });

            ExperimentalGuidanceTiming.Result Time = ExperimentalGuidanceTiming.Estimate(
                Guidance, new ExperimentalGuidanceTiming.Options());

            Assert.Equal(1, Time.ForcedStopCount);
            Assert.Equal(100.0, Time.DistanceM, 9);
        }

        /// <summary>
        /// Verifies manual-assist speed and fixed takeover overhead are both charged exactly once.
        /// </summary>
        [Fact]
        public void Estimate_ManualAssistIncludesConfiguredOverhead
            (
            )
        {
            ExperimentalLifecyclePlanner.Result Guidance = MakeGuidance(
                ExperimentalLifecyclePlanner.Phase.ManualAssist,
                new List<(Double X, Double Y)> { (0.0, 0.0), (10.0, 0.0) });
            ExperimentalGuidanceTiming.Options Without = new ExperimentalGuidanceTiming.Options
            {
                ManualAssistOverheadS = 0.0,
            };
            ExperimentalGuidanceTiming.Options With = new ExperimentalGuidanceTiming.Options
            {
                ManualAssistOverheadS = 17.0,
            };

            Double WithoutSeconds = ExperimentalGuidanceTiming.Estimate(Guidance, Without).TotalSeconds;
            ExperimentalGuidanceTiming.Result WithTime =
                ExperimentalGuidanceTiming.Estimate(Guidance, With);

            Assert.Equal(WithoutSeconds + 17.0, WithTime.TotalSeconds, 9);
            Assert.Equal(17.0, WithTime.EventSeconds, 9);
        }

        /// <summary>
        /// Verifies field-efficiency allowance is separate and totals reconcile exactly.
        /// </summary>
        [Fact]
        public void Estimate_ReportsSeparateFieldEfficiencyAllowance
            (
            )
        {
            ExperimentalLifecyclePlanner.Result Guidance = MakeGuidance(
                ExperimentalLifecyclePlanner.Phase.Cut,
                new List<(Double X, Double Y)> { (0.0, 0.0), (20.0, 0.0) });
            ExperimentalGuidanceTiming.Options Options = new ExperimentalGuidanceTiming.Options
            {
                FieldEfficiency = 0.8,
                BladeLowerS = 3.0,
            };

            ExperimentalGuidanceTiming.Result Time =
                ExperimentalGuidanceTiming.Estimate(Guidance, Options);

            Assert.Equal((Time.MotionSeconds + Time.EventSeconds) / 0.8,
                Time.TotalSeconds, 9);
            Assert.Equal(Time.TotalSeconds,
                Time.PhaseSeconds.Values.Sum() + Time.FieldEfficiencyAllowanceSeconds, 9);
        }

        /// <summary>Creates a one-segment successful lifecycle result.</summary>
        private static ExperimentalLifecyclePlanner.Result MakeGuidance
            (
            ExperimentalLifecyclePlanner.Phase Phase,
            List<(Double X, Double Y)> Points
            )
        {
            return MakeGuidance(Phase, Points, false);
        }

        /// <summary>Creates a one-segment lifecycle with explicit payload state.</summary>
        private static ExperimentalLifecyclePlanner.Result MakeGuidance
            (
            ExperimentalLifecyclePlanner.Phase Phase,
            List<(Double X, Double Y)> Points,
            Boolean Loaded
            )
        {
            Boolean BladeDown = (Phase == ExperimentalLifecyclePlanner.Phase.Cut)
                || (Phase == ExperimentalLifecyclePlanner.Phase.Fill)
                || (Phase == ExperimentalLifecyclePlanner.Phase.Grade);
            List<ExperimentalLifecyclePlanner.Segment> Segments =
                new List<ExperimentalLifecyclePlanner.Segment>
                {
                    new ExperimentalLifecyclePlanner.Segment(
                        Phase, BladeDown, Points, Loaded),
                };
            return new ExperimentalLifecyclePlanner.Result(true, Segments, 0, 0.0, String.Empty);
        }

        /// <summary>Creates a successful lifecycle result from explicit segments.</summary>
        private static ExperimentalLifecyclePlanner.Result MakeGuidance
            (
            List<ExperimentalLifecyclePlanner.Segment> Segments
            )
        {
            return new ExperimentalLifecyclePlanner.Result(
                true, Segments, 0, 0.0, String.Empty);
        }

        /// <summary>Creates one straight test segment.</summary>
        private static ExperimentalLifecyclePlanner.Segment MakeSegment
            (
            ExperimentalLifecyclePlanner.Phase Phase,
            (Double X, Double Y) Start,
            (Double X, Double Y) End
            )
        {
            Boolean BladeDown = (Phase == ExperimentalLifecyclePlanner.Phase.Cut)
                || (Phase == ExperimentalLifecyclePlanner.Phase.Fill)
                || (Phase == ExperimentalLifecyclePlanner.Phase.Grade);
            return new ExperimentalLifecyclePlanner.Segment(
                Phase, BladeDown,
                new List<(Double X, Double Y)> { Start, End });
        }

        /// <summary>Creates an evenly sampled semicircular centerline.</summary>
        private static List<(Double X, Double Y)> MakeSemicircle
            (
            Double RadiusM,
            Int32 PointCount
            )
        {
            return Enumerable.Range(0, PointCount)
                .Select(Index =>
                {
                    Double Angle = Math.PI * Index / (PointCount - 1);
                    return (RadiusM * Math.Cos(Angle), RadiusM * Math.Sin(Angle));
                })
                .ToList();
        }
    }
}
