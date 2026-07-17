using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies timing derives from final geometry and explicit lifecycle events.
    /// </summary>
    public sealed class GuidanceTimingTests
    {
        /// <summary>Verifies user-facing speeds feed the guidance timing model.</summary>
        [Fact]
        public void Options_MapsUserFacingHaulSettings
            (
            )
        {
            HaulBuilderOptions Source = new HaulBuilderOptions
            {
                LoadedTravelSpeedMph = 9.0,
                UnloadedTravelSpeedMph = 12.0,
                TurnSpeedMph = 3.0,
            };

            GuidanceTiming.Options Options =
                new GuidanceTiming.Options(Source);

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
            LifecyclePlanner.Result Sparse = MakeGuidance(
                LifecyclePlanner.Phase.Transit,
                new List<(Double X, Double Y)> { (0.0, 0.0), (100.0, 0.0) });
            List<(Double X, Double Y)> DensePoints = Enumerable.Range(0, 101)
                .Select(Index => ((Double)Index, 0.0)).ToList();
            LifecyclePlanner.Result Dense = MakeGuidance(
                LifecyclePlanner.Phase.Transit, DensePoints);
            GuidanceTiming.Options Options = new GuidanceTiming.Options();

            GuidanceTiming.Result SparseTime =
                GuidanceTiming.Estimate(Sparse, Options);
            GuidanceTiming.Result DenseTime =
                GuidanceTiming.Estimate(Dense, Options);

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
            LifecyclePlanner.Result Combined = MakeGuidance(
                LifecyclePlanner.Phase.Transit,
                new List<(Double X, Double Y)> { (0.0, 0.0), (100.0, 0.0) });
            LifecyclePlanner.Result Partitioned = MakeGuidance(
                new List<LifecyclePlanner.Segment>
                {
                    MakeSegment(LifecyclePlanner.Phase.Transit,
                        (0.0, 0.0), (40.0, 0.0)),
                    MakeSegment(LifecyclePlanner.Phase.Transit,
                        (40.0, 0.0), (100.0, 0.0)),
                });
            GuidanceTiming.Options Options = new GuidanceTiming.Options();

            GuidanceTiming.Result CombinedTime =
                GuidanceTiming.Estimate(Combined, Options);
            GuidanceTiming.Result PartitionedTime =
                GuidanceTiming.Estimate(Partitioned, Options);

            Assert.Equal(CombinedTime.TotalSeconds, PartitionedTime.TotalSeconds, 9);
            Assert.Equal(0, PartitionedTime.ForcedStopCount);
        }

        /// <summary>Verifies the trapezoidal profile against a hand calculation.</summary>
        [Fact]
        public void Estimate_MatchesAnalyticTrapezoidalProfile
            (
            )
        {
            LifecyclePlanner.Result Guidance = MakeGuidance(
                LifecyclePlanner.Phase.Transit,
                new List<(Double X, Double Y)> { (0.0, 0.0), (100.0, 0.0) });
            GuidanceTiming.Options Options = new GuidanceTiming.Options
            {
                UnloadedTravelSpeedMph = 10.0 / Constants.MPH_TO_MPS,
                AccelerationMps2 = 2.0,
                DecelerationMps2 = 2.0,
            };

            GuidanceTiming.Result Time =
                GuidanceTiming.Estimate(Guidance, Options);

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
            LifecyclePlanner.Segment Transit = MakeSegment(
                LifecyclePlanner.Phase.Transit,
                (0.0, 0.0), (100.0, 0.0));
            LifecyclePlanner.Segment Cut = MakeSegment(
                LifecyclePlanner.Phase.Cut,
                (100.0, 0.0), (200.0, 0.0));
            GuidanceTiming.Options Options = new GuidanceTiming.Options();
            GuidanceTiming.Result Chained = GuidanceTiming.Estimate(
                MakeGuidance(new List<LifecyclePlanner.Segment> { Transit, Cut }),
                Options);
            Double SeparatelyStopped = GuidanceTiming.Estimate(
                MakeGuidance(new List<LifecyclePlanner.Segment> { Transit }),
                Options).MotionSeconds
                + GuidanceTiming.Estimate(
                    MakeGuidance(new List<LifecyclePlanner.Segment> { Cut }),
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
            GuidanceTiming.Options Options = new GuidanceTiming.Options
            {
                MinimumTurningRadiusM = MINIMUM_RADIUS_M,
                LoadedTravelSpeedMph = 10.0,
                TurnSpeedMph = 4.0,
            };
            GuidanceTiming.Result Tight = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.Transit,
                    MakeSemicircle(MINIMUM_RADIUS_M, 1001), true), Options);
            GuidanceTiming.Result Gentle = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.Transit,
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
            GuidanceTiming.Options Options = new GuidanceTiming.Options
            {
                LoadedTravelSpeedMph = 10.0,
                UnloadedTravelSpeedMph = 13.0,
            };
            GuidanceTiming.Result Loaded = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.Transit, Points, true), Options);
            GuidanceTiming.Result Unloaded = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.Transit, Points, false), Options);

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
            GuidanceTiming.Options Options = new GuidanceTiming.Options
            {
                UnloadedTravelSpeedMph = 13.0,
                TurnSpeedMph = 4.0,
            };
            GuidanceTiming.Result Lineup = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.LineupCut,
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
            GuidanceTiming.Options Options = new GuidanceTiming.Options();
            GuidanceTiming.Result Sparse = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.Transit,
                    MakeSemicircle(25.0, 159)), Options);
            GuidanceTiming.Result Dense = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.Transit,
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
            GuidanceTiming.Options Options = new GuidanceTiming.Options();
            GuidanceTiming.Result Manual = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.ManualAssist, Points), Options);
            GuidanceTiming.Result Transit = GuidanceTiming.Estimate(
                MakeGuidance(LifecyclePlanner.Phase.Transit, Points), Options);

            Assert.Equal(Transit.MotionSeconds, Manual.MotionSeconds, 9);
        }

        /// <summary>Verifies a geometry discontinuity is conservatively timed as a stop.</summary>
        [Fact]
        public void Estimate_DiscontinuousGeometryForcesStop
            (
            )
        {
            LifecyclePlanner.Result Guidance = MakeGuidance(
                new List<LifecyclePlanner.Segment>
                {
                    MakeSegment(LifecyclePlanner.Phase.Transit,
                        (0.0, 0.0), (50.0, 0.0)),
                    MakeSegment(LifecyclePlanner.Phase.Transit,
                        (100.0, 0.0), (150.0, 0.0)),
                });

            GuidanceTiming.Result Time = GuidanceTiming.Estimate(
                Guidance, new GuidanceTiming.Options());

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
            LifecyclePlanner.Result Guidance = MakeGuidance(
                LifecyclePlanner.Phase.ManualAssist,
                new List<(Double X, Double Y)> { (0.0, 0.0), (10.0, 0.0) });
            GuidanceTiming.Options Without = new GuidanceTiming.Options
            {
                ManualAssistOverheadS = 0.0,
            };
            GuidanceTiming.Options With = new GuidanceTiming.Options
            {
                ManualAssistOverheadS = 17.0,
            };

            Double WithoutSeconds = GuidanceTiming.Estimate(Guidance, Without).TotalSeconds;
            GuidanceTiming.Result WithTime =
                GuidanceTiming.Estimate(Guidance, With);

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
            LifecyclePlanner.Result Guidance = MakeGuidance(
                LifecyclePlanner.Phase.Cut,
                new List<(Double X, Double Y)> { (0.0, 0.0), (20.0, 0.0) });
            GuidanceTiming.Options Options = new GuidanceTiming.Options
            {
                FieldEfficiency = 0.8,
                BladeLowerS = 3.0,
            };

            GuidanceTiming.Result Time =
                GuidanceTiming.Estimate(Guidance, Options);

            Assert.Equal((Time.MotionSeconds + Time.EventSeconds) / 0.8,
                Time.TotalSeconds, 9);
            Assert.Equal(Time.TotalSeconds,
                Time.PhaseSeconds.Values.Sum() + Time.FieldEfficiencyAllowanceSeconds, 9);
        }

        /// <summary>Creates a one-segment successful lifecycle result.</summary>
        private static LifecyclePlanner.Result MakeGuidance
            (
            LifecyclePlanner.Phase Phase,
            List<(Double X, Double Y)> Points
            )
        {
            return MakeGuidance(Phase, Points, false);
        }

        /// <summary>Creates a one-segment lifecycle with explicit payload state.</summary>
        private static LifecyclePlanner.Result MakeGuidance
            (
            LifecyclePlanner.Phase Phase,
            List<(Double X, Double Y)> Points,
            Boolean Loaded
            )
        {
            Boolean BladeDown = (Phase == LifecyclePlanner.Phase.Cut)
                || (Phase == LifecyclePlanner.Phase.Fill)
                || (Phase == LifecyclePlanner.Phase.Grade);
            List<LifecyclePlanner.Segment> Segments =
                new List<LifecyclePlanner.Segment>
                {
                    new LifecyclePlanner.Segment(
                        Phase, BladeDown, Points, Loaded),
                };
            return new LifecyclePlanner.Result(true, Segments, 0, 0.0, String.Empty);
        }

        /// <summary>Creates a successful lifecycle result from explicit segments.</summary>
        private static LifecyclePlanner.Result MakeGuidance
            (
            List<LifecyclePlanner.Segment> Segments
            )
        {
            return new LifecyclePlanner.Result(
                true, Segments, 0, 0.0, String.Empty);
        }

        /// <summary>Creates one straight test segment.</summary>
        private static LifecyclePlanner.Segment MakeSegment
            (
            LifecyclePlanner.Phase Phase,
            (Double X, Double Y) Start,
            (Double X, Double Y) End
            )
        {
            Boolean BladeDown = (Phase == LifecyclePlanner.Phase.Cut)
                || (Phase == LifecyclePlanner.Phase.Fill)
                || (Phase == LifecyclePlanner.Phase.Grade);
            return new LifecyclePlanner.Segment(
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
