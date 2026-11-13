using System;
using System.Globalization;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// The eight HaulBuilder option values that the field solver varies. Every
    /// other option (equipment, speeds, tolerance, exclusions, ponding, pivot)
    /// stays as the caller set it. Two settings are equal when all eight values
    /// are equal, which is how the solver avoids running the same plan twice.
    /// </summary>
    public sealed record FieldSolverSettings
    {
        /// <summary>Minimum work length value that means half the equipment length.</summary>
        public const Double MINIMUM_WORK_LENGTH_DEFAULT_FT = 0.0;

        /// <summary>
        /// Minimum work length value the solver uses for "rule off". HaulBuilder
        /// treats zero as the default, so off is a length shorter than any run.
        /// </summary>
        public const Double MINIMUM_WORK_LENGTH_OFF_FT = 0.001;

        /// <summary>Number of option values a settings object carries.</summary>
        public const Int32 VALUE_COUNT = 8;

        /// <summary>Gets the boundary-ring slice cap, <c>Optimization.EdgeMaxLaneSlicesPerCycle</c>.</summary>
        public Int32 EdgeMaxLaneSlicesPerCycle { get; init; }

        /// <summary>Gets the minimum blade-down run length in feet, <c>MinimumWorkLengthFt</c>.</summary>
        public Double MinimumWorkLengthFt { get; init; }

        /// <summary>Gets the lane axis in degrees, normalized into [0, 180), <c>LaneAxisHeadingDeg</c>.</summary>
        public Double LaneAxisHeadingDeg { get; init; }

        /// <summary>Gets the flow penalty weight, <c>Optimization.FlowPenaltyWeight</c>.</summary>
        public Double FlowPenaltyWeight { get; init; }

        /// <summary>Gets the interior slice cap, <c>Optimization.MaxLaneSlicesPerCycle</c>.</summary>
        public Int32 MaxLaneSlicesPerCycle { get; init; }

        /// <summary>Gets the route reorder window, <c>Optimization.RouteReorderWindow</c>.</summary>
        public Int32 RouteReorderWindow { get; init; }

        /// <summary>Gets whether multi-route exchange runs, <c>Optimization.EnableMultiRouteExchange</c>.</summary>
        public Boolean EnableMultiRouteExchange { get; init; }

        /// <summary>Gets whether the cycle chain runs, <c>Optimization.EnableCycleChainOptimization</c>.</summary>
        public Boolean EnableCycleChainOptimization { get; init; }

        /// <summary>Reads the eight values from a HaulBuilder options object.</summary>
        /// <param name="Options">Options to read. Not modified.</param>
        /// <returns>The eight values, with the lane axis normalized.</returns>
        /// <exception cref="ArgumentNullException">Thrown when options is null.</exception>
        static public FieldSolverSettings FromOptions
            (
            HaulBuilderOptions Options
            )
        {
            if (Options == null) throw new ArgumentNullException(nameof(Options));
            return new FieldSolverSettings
            {
                EdgeMaxLaneSlicesPerCycle = Options.Optimization.EdgeMaxLaneSlicesPerCycle,
                MinimumWorkLengthFt = Options.MinimumWorkLengthFt,
                // 0 and 180 name the same lane axis, so settings compare the normalized value.
                LaneAxisHeadingDeg = HaulLaneBuilder.NormalizeAxisHeadingDeg(Options.LaneAxisHeadingDeg),
                FlowPenaltyWeight = Options.Optimization.FlowPenaltyWeight,
                MaxLaneSlicesPerCycle = Options.Optimization.MaxLaneSlicesPerCycle,
                RouteReorderWindow = Options.Optimization.RouteReorderWindow,
                EnableMultiRouteExchange = Options.Optimization.EnableMultiRouteExchange,
                EnableCycleChainOptimization = Options.Optimization.EnableCycleChainOptimization,
            };
        }

        /// <summary>Writes the eight values into a HaulBuilder options object.</summary>
        /// <param name="Options">Options to change. Every other option is left alone.</param>
        /// <exception cref="ArgumentNullException">Thrown when options is null.</exception>
        public void ApplyTo
            (
            HaulBuilderOptions Options
            )
        {
            if (Options == null) throw new ArgumentNullException(nameof(Options));
            Options.Optimization.EdgeMaxLaneSlicesPerCycle = EdgeMaxLaneSlicesPerCycle;
            Options.MinimumWorkLengthFt = MinimumWorkLengthFt;
            Options.LaneAxisHeadingDeg = LaneAxisHeadingDeg;
            Options.Optimization.FlowPenaltyWeight = FlowPenaltyWeight;
            Options.Optimization.MaxLaneSlicesPerCycle = MaxLaneSlicesPerCycle;
            Options.Optimization.RouteReorderWindow = RouteReorderWindow;
            Options.Optimization.EnableMultiRouteExchange = EnableMultiRouteExchange;
            Options.Optimization.EnableCycleChainOptimization = EnableCycleChainOptimization;
        }

        /// <summary>Counts how many of the eight values differ from another settings object.</summary>
        /// <param name="Other">Settings to compare with, normally the baseline.</param>
        /// <returns>Zero when equal, up to <see cref="VALUE_COUNT"/>.</returns>
        /// <exception cref="ArgumentNullException">Thrown when other is null.</exception>
        public Int32 CountDifferences
            (
            FieldSolverSettings Other
            )
        {
            if (Other == null) throw new ArgumentNullException(nameof(Other));
            Int32 Differences = 0;
            if (EdgeMaxLaneSlicesPerCycle != Other.EdgeMaxLaneSlicesPerCycle) Differences += 1;
            if (!MinimumWorkLengthFt.Equals(Other.MinimumWorkLengthFt)) Differences += 1;
            if (!LaneAxisHeadingDeg.Equals(Other.LaneAxisHeadingDeg)) Differences += 1;
            if (!FlowPenaltyWeight.Equals(Other.FlowPenaltyWeight)) Differences += 1;
            if (MaxLaneSlicesPerCycle != Other.MaxLaneSlicesPerCycle) Differences += 1;
            if (RouteReorderWindow != Other.RouteReorderWindow) Differences += 1;
            if (EnableMultiRouteExchange != Other.EnableMultiRouteExchange) Differences += 1;
            if (EnableCycleChainOptimization != Other.EnableCycleChainOptimization) Differences += 1;
            return Differences;
        }

        /// <summary>
        /// Formats the values as the <c>key=value</c> overrides that the HaulRunProof
        /// harness accepts, so one candidate can be repeated from the command line.
        /// </summary>
        /// <returns>Text such as <c>edge=40 minwork=0.001 axis=90 flow=1 slices=10 window=200 exchange=False chain=False</c>.</returns>
        public override String ToString
            (
            )
        {
            // Invariant culture keeps the decimal point a point on every machine.
            return String.Format(
                CultureInfo.InvariantCulture,
                "edge={0} minwork={1} axis={2} flow={3} slices={4} window={5} exchange={6} chain={7}",
                EdgeMaxLaneSlicesPerCycle,
                MinimumWorkLengthFt,
                LaneAxisHeadingDeg,
                FlowPenaltyWeight,
                MaxLaneSlicesPerCycle,
                RouteReorderWindow,
                EnableMultiRouteExchange,
                EnableCycleChainOptimization);
        }
    }
}
