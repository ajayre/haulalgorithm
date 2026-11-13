using System;
using System.Collections.Generic;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// The values the solver may try for each of the eight settings. The first
    /// three lists are combined: every combination of their values is run. The
    /// other five are screened: each value is tried alone on the best plan found
    /// by the combinations. An empty list keeps the caller's value.
    /// </summary>
    public sealed class FieldSolverSearchSpace
    {
        /// <summary>Edge cap the engine ships with.</summary>
        private const Int32 EDGE_CAP_DEFAULT = 12;

        /// <summary>Edge cap at which the cap stopped binding on the measured field.</summary>
        private const Int32 EDGE_CAP_LOOSE = 40;

        /// <summary>Fallback lane axis when the design slope cannot be read: north-south.</summary>
        private const Double LANE_AXIS_FALLBACK_PRIMARY_DEG = 0.0;

        /// <summary>Fallback lane axis when the design slope cannot be read: east-west.</summary>
        private const Double LANE_AXIS_FALLBACK_SECONDARY_DEG = 90.0;

        /// <summary>Flow penalty weight below the engine default of 1.0.</summary>
        private const Double FLOW_WEIGHT_LOW = 0.5;

        /// <summary>Flow penalty weight above the engine default of 1.0.</summary>
        private const Double FLOW_WEIGHT_HIGH = 2.0;

        /// <summary>Interior slice cap below the engine default of 10.</summary>
        private const Int32 INTERIOR_SLICES_LOW = 8;

        /// <summary>Interior slice cap above the engine default of 10.</summary>
        private const Int32 INTERIOR_SLICES_HIGH = 14;

        /// <summary>Route reorder window above the engine default of 200.</summary>
        private const Int32 REORDER_WINDOW_WIDE = 1000;

        /// <summary>Initializes an empty search space that keeps every caller value.</summary>
        public FieldSolverSearchSpace
            (
            )
        {
            EdgeMaxLaneSlicesPerCycleValues = new List<Int32>();
            MinimumWorkLengthFtValues = new List<Double>();
            LaneAxisHeadingDegValues = new List<Double>();
            FlowPenaltyWeightValues = new List<Double>();
            MaxLaneSlicesPerCycleValues = new List<Int32>();
            RouteReorderWindowValues = new List<Int32>();
            EnableMultiRouteExchangeValues = new List<Boolean>();
            EnableCycleChainOptimizationValues = new List<Boolean>();
        }

        /// <summary>Gets the combined edge cap values.</summary>
        public List<Int32> EdgeMaxLaneSlicesPerCycleValues { get; }

        /// <summary>Gets the combined minimum work length values in feet.</summary>
        public List<Double> MinimumWorkLengthFtValues { get; }

        /// <summary>Gets the combined lane axis values in degrees.</summary>
        public List<Double> LaneAxisHeadingDegValues { get; }

        /// <summary>Gets the screened flow penalty weights.</summary>
        public List<Double> FlowPenaltyWeightValues { get; }

        /// <summary>Gets the screened interior slice caps.</summary>
        public List<Int32> MaxLaneSlicesPerCycleValues { get; }

        /// <summary>Gets the screened route reorder windows.</summary>
        public List<Int32> RouteReorderWindowValues { get; }

        /// <summary>Gets the screened multi-route exchange switch values.</summary>
        public List<Boolean> EnableMultiRouteExchangeValues { get; }

        /// <summary>Gets the screened cycle chain switch values.</summary>
        public List<Boolean> EnableCycleChainOptimizationValues { get; }

        /// <summary>
        /// Creates the standard search: edge cap 12 or 40, minimum work length
        /// default or off, and the two lane axes taken from the design-surface
        /// down-slope (and its perpendicular). When <paramref name="BaseOptions"/>
        /// is null or its field file cannot be read, the lane axes fall back to
        /// north-south and east-west. Eight runs when the caller's options hold
        /// engine defaults and the two design axes are distinct from each other.
        /// </summary>
        /// <param name="BaseOptions">
        /// Optional caller options used only to read the AGD and derive the lane
        /// axes. The solver never changes this object.
        /// </param>
        /// <returns>A new search space the caller may edit.</returns>
        static public FieldSolverSearchSpace CreateStandard
            (
            HaulBuilderOptions? BaseOptions = null
            )
        {
            FieldSolverSearchSpace Space = new FieldSolverSearchSpace();
            Space.EdgeMaxLaneSlicesPerCycleValues.Add(EDGE_CAP_DEFAULT);
            Space.EdgeMaxLaneSlicesPerCycleValues.Add(EDGE_CAP_LOOSE);
            Space.MinimumWorkLengthFtValues.Add(FieldSolverSettings.MINIMUM_WORK_LENGTH_DEFAULT_FT);
            Space.MinimumWorkLengthFtValues.Add(FieldSolverSettings.MINIMUM_WORK_LENGTH_OFF_FT);
            AddDesignSlopeLaneAxes(Space, BaseOptions);
            return Space;
        }

        /// <summary>
        /// Creates the thorough search: the standard combinations, then seven
        /// screening runs over the other five settings and at most one run that
        /// combines the screening values that helped.
        /// </summary>
        /// <param name="BaseOptions">
        /// Optional caller options used to derive the design-slope lane axes.
        /// </param>
        /// <returns>A new search space the caller may edit.</returns>
        static public FieldSolverSearchSpace CreateThorough
            (
            HaulBuilderOptions? BaseOptions = null
            )
        {
            FieldSolverSearchSpace Space = CreateStandard(BaseOptions);
            Space.FlowPenaltyWeightValues.Add(FLOW_WEIGHT_LOW);
            Space.FlowPenaltyWeightValues.Add(FLOW_WEIGHT_HIGH);
            Space.MaxLaneSlicesPerCycleValues.Add(INTERIOR_SLICES_LOW);
            Space.MaxLaneSlicesPerCycleValues.Add(INTERIOR_SLICES_HIGH);
            Space.RouteReorderWindowValues.Add(REORDER_WINDOW_WIDE);
            Space.EnableMultiRouteExchangeValues.Add(true);
            Space.EnableCycleChainOptimizationValues.Add(true);
            return Space;
        }

        /// <summary>
        /// Fills <see cref="LaneAxisHeadingDegValues"/> with the design-slope
        /// lane axis and its perpendicular. Falls back to north-south and
        /// east-west when the field cannot be read.
        /// </summary>
        /// <param name="Space">Search space to fill.</param>
        /// <param name="BaseOptions">Options that name the field file, or null.</param>
        static public void AddDesignSlopeLaneAxes
            (
            FieldSolverSearchSpace Space,
            HaulBuilderOptions? BaseOptions
            )
        {
            if (Space == null) throw new ArgumentNullException(nameof(Space));
            Space.LaneAxisHeadingDegValues.Clear();
            // Intent: design slope first, then the perpendicular, so a solve that
            // stops early has tried the slope axis before the cross-slope axis.
            if (DesignSlopeLaneAxis.TryFromOptions(
                BaseOptions,
                out Double SlopeAxisDeg,
                out Double PerpendicularAxisDeg))
            {
                Space.LaneAxisHeadingDegValues.Add(SlopeAxisDeg);
                // Same axis after a half turn: only one value is needed.
                if (HaulLaneBuilder.NormalizeAxisHeadingDeg(PerpendicularAxisDeg)
                    != HaulLaneBuilder.NormalizeAxisHeadingDeg(SlopeAxisDeg))
                {
                    Space.LaneAxisHeadingDegValues.Add(PerpendicularAxisDeg);
                }
                return;
            }

            Space.LaneAxisHeadingDegValues.Add(LANE_AXIS_FALLBACK_PRIMARY_DEG);
            Space.LaneAxisHeadingDegValues.Add(LANE_AXIS_FALLBACK_SECONDARY_DEG);
        }
    }
}
