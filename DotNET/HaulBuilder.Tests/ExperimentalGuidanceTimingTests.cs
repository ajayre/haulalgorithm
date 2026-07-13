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
            Boolean BladeDown = (Phase == ExperimentalLifecyclePlanner.Phase.Cut)
                || (Phase == ExperimentalLifecyclePlanner.Phase.Fill)
                || (Phase == ExperimentalLifecyclePlanner.Phase.Grade);
            List<ExperimentalLifecyclePlanner.Segment> Segments =
                new List<ExperimentalLifecyclePlanner.Segment>
                {
                    new ExperimentalLifecyclePlanner.Segment(Phase, BladeDown, Points),
                };
            return new ExperimentalLifecyclePlanner.Result(true, Segments, 0, 0.0, String.Empty);
        }
    }
}
