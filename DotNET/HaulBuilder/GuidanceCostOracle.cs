using System;

namespace HaulBuilder
{
    /// <summary>
    /// Side-effect-free adapter that evaluates immutable route candidates through
    /// the production guidance and timing implementation.
    /// </summary>
    static public class GuidanceCostOracle
    {
        /// <summary>Exact candidate result returned by production guidance.</summary>
        public sealed class Result
        {
            /// <summary>Initializes one exact candidate result.</summary>
            public Result
                (
                Boolean Success,
                GuidancePlanner.BuildResult? Plan,
                String FailureReason
                )
            {
                this.Success = Success;
                this.Plan = Plan;
                this.FailureReason = FailureReason;
            }

            /// <summary>Gets whether exact guidance succeeded.</summary>
            public Boolean Success { get; }

            /// <summary>Gets production guidance and timing when successful.</summary>
            public GuidancePlanner.BuildResult? Plan { get; }

            /// <summary>Gets structured failure text.</summary>
            public String FailureReason { get; }

            /// <summary>Gets deterministic motion and event seconds.</summary>
            public Double DeterministicSeconds => this.Plan == null
                ? Double.PositiveInfinity
                : this.Plan.Timing.MotionSeconds + this.Plan.Timing.EventSeconds;

            /// <summary>Gets scheduled seconds including operational availability.</summary>
            public Double ScheduledSeconds => this.Plan == null
                ? Double.PositiveInfinity
                : this.Plan.Timing.TotalSeconds;

            /// <summary>Gets final centerline distance.</summary>
            public Double DistanceM => this.Plan == null
                ? Double.PositiveInfinity
                : this.Plan.Timing.DistanceM;
        }

        /// <summary>Evaluates a complete route with the production implementation.</summary>
        static public Result EvaluatePlan
            (
            Route Route,
            Grid Grid,
            AgdField Field,
            Double HeadingDeg,
            HaulBuilderOptions Options,
            Action<String, Int32, Int32>? Progress = null,
            Action<String>? Diagnostic = null
            )
        {
            try
            {
                GuidancePlanner.BuildResult Plan = GuidancePlanner.Build(
                    Route, Grid, Field, HeadingDeg, Options,
                    Progress, Diagnostic);
                return new Result(true, Plan, String.Empty);
            }
            catch (Exception Error)
            {
                return new Result(false, null, Error.ToString());
            }
        }
    }
}
