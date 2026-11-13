using System;
using System.Collections.Generic;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// Turns a search space into the ordered list of settings to run. It holds no
    /// state and runs nothing, so the order can be tested without the engine.
    /// </summary>
    static internal class SearchPlanner
    {
        /// <summary>
        /// Builds every combination of the three combined settings (edge cap,
        /// minimum work length, lane axis). The baseline comes first, then
        /// combinations with one change, then two, then three.
        /// </summary>
        /// <param name="Baseline">The caller's settings.</param>
        /// <param name="Space">Values to try.</param>
        /// <returns>Distinct settings in run order; the first entry equals the baseline.</returns>
        static public List<FieldSolverSettings> BuildCombinations
            (
            FieldSolverSettings Baseline,
            FieldSolverSearchSpace Space
            )
        {
            List<Int32> EdgeValues = WithBaseFirst(Baseline.EdgeMaxLaneSlicesPerCycle, Space.EdgeMaxLaneSlicesPerCycleValues);
            List<Double> WorkValues = WithBaseFirst(Baseline.MinimumWorkLengthFt, Space.MinimumWorkLengthFtValues);
            List<Double> NormalizedAxes = new List<Double>();
            foreach (Double AxisDeg in Space.LaneAxisHeadingDegValues)
            {
                // 0 and 180 name the same lane axis; normalize so they are tried once.
                NormalizedAxes.Add(HaulLaneBuilder.NormalizeAxisHeadingDeg(AxisDeg));
            }
            List<Double> AxisValues = WithBaseFirst(Baseline.LaneAxisHeadingDeg, NormalizedAxes);

            List<(Int32 Edge, Int32 Work, Int32 Axis)> Indexes = new List<(Int32 Edge, Int32 Work, Int32 Axis)>();
            for (Int32 Edge = 0; Edge < EdgeValues.Count; Edge += 1)
            {
                for (Int32 Work = 0; Work < WorkValues.Count; Work += 1)
                {
                    for (Int32 Axis = 0; Axis < AxisValues.Count; Axis += 1)
                    {
                        Indexes.Add((Edge, Work, Axis));
                    }
                }
            }
            // Fewest changes first, so a solve that is stopped early has tried the
            // plans closest to the caller's settings.
            Indexes.Sort(CompareCombinations);

            List<FieldSolverSettings> Output = new List<FieldSolverSettings>();
            foreach ((Int32 Edge, Int32 Work, Int32 Axis) Index in Indexes)
            {
                Output.Add(Baseline with
                {
                    EdgeMaxLaneSlicesPerCycle = EdgeValues[Index.Edge],
                    MinimumWorkLengthFt = WorkValues[Index.Work],
                    LaneAxisHeadingDeg = AxisValues[Index.Axis],
                });
            }
            return Output;
        }

        /// <summary>
        /// Builds the screening runs for the five screened settings. Each run changes
        /// one value on the leader. One inner list per setting, in the order flow
        /// weight, interior slices, reorder window, multi-route exchange, cycle chain.
        /// </summary>
        /// <param name="Leader">The best settings found by the combinations.</param>
        /// <param name="Space">Values to try.</param>
        /// <returns>Five lists; a list is empty when the setting has nothing new to try.</returns>
        static public List<List<FieldSolverSettings>> BuildScreeningGroups
            (
            FieldSolverSettings Leader,
            FieldSolverSearchSpace Space
            )
        {
            List<List<FieldSolverSettings>> Groups = new List<List<FieldSolverSettings>>();

            List<FieldSolverSettings> Flow = new List<FieldSolverSettings>();
            foreach (Double Value in Space.FlowPenaltyWeightValues)
            {
                AddIfNew(Flow, Leader, Leader with { FlowPenaltyWeight = Value });
            }
            Groups.Add(Flow);

            List<FieldSolverSettings> Slices = new List<FieldSolverSettings>();
            foreach (Int32 Value in Space.MaxLaneSlicesPerCycleValues)
            {
                AddIfNew(Slices, Leader, Leader with { MaxLaneSlicesPerCycle = Value });
            }
            Groups.Add(Slices);

            List<FieldSolverSettings> Window = new List<FieldSolverSettings>();
            foreach (Int32 Value in Space.RouteReorderWindowValues)
            {
                AddIfNew(Window, Leader, Leader with { RouteReorderWindow = Value });
            }
            Groups.Add(Window);

            List<FieldSolverSettings> Exchange = new List<FieldSolverSettings>();
            foreach (Boolean Value in Space.EnableMultiRouteExchangeValues)
            {
                AddIfNew(Exchange, Leader, Leader with { EnableMultiRouteExchange = Value });
            }
            Groups.Add(Exchange);

            List<FieldSolverSettings> Chain = new List<FieldSolverSettings>();
            foreach (Boolean Value in Space.EnableCycleChainOptimizationValues)
            {
                AddIfNew(Chain, Leader, Leader with { EnableCycleChainOptimization = Value });
            }
            Groups.Add(Chain);

            return Groups;
        }

        /// <summary>Counts the screening values in a search space, for the planned run count.</summary>
        /// <param name="Space">Values to try.</param>
        /// <returns>The largest number of screening runs the space can cause, without the combining run.</returns>
        static public Int32 CountScreeningValues
            (
            FieldSolverSearchSpace Space
            )
        {
            return Space.FlowPenaltyWeightValues.Count
                + Space.MaxLaneSlicesPerCycleValues.Count
                + Space.RouteReorderWindowValues.Count
                + Space.EnableMultiRouteExchangeValues.Count
                + Space.EnableCycleChainOptimizationValues.Count;
        }

        /// <summary>
        /// Copies onto a target every value that a changed settings object holds
        /// differently from a reference. Used to put several screening winners,
        /// each one change from the leader, into one settings object.
        /// </summary>
        /// <param name="Target">Settings that collect the changes.</param>
        /// <param name="Reference">Settings the changed object was derived from.</param>
        /// <param name="Changed">Settings holding the change to take.</param>
        /// <returns>The target with the changed values applied.</returns>
        static public FieldSolverSettings MergeChanges
            (
            FieldSolverSettings Target,
            FieldSolverSettings Reference,
            FieldSolverSettings Changed
            )
        {
            return Target with
            {
                EdgeMaxLaneSlicesPerCycle = (Changed.EdgeMaxLaneSlicesPerCycle != Reference.EdgeMaxLaneSlicesPerCycle)
                    ? Changed.EdgeMaxLaneSlicesPerCycle
                    : Target.EdgeMaxLaneSlicesPerCycle,
                MinimumWorkLengthFt = !Changed.MinimumWorkLengthFt.Equals(Reference.MinimumWorkLengthFt)
                    ? Changed.MinimumWorkLengthFt
                    : Target.MinimumWorkLengthFt,
                LaneAxisHeadingDeg = !Changed.LaneAxisHeadingDeg.Equals(Reference.LaneAxisHeadingDeg)
                    ? Changed.LaneAxisHeadingDeg
                    : Target.LaneAxisHeadingDeg,
                FlowPenaltyWeight = !Changed.FlowPenaltyWeight.Equals(Reference.FlowPenaltyWeight)
                    ? Changed.FlowPenaltyWeight
                    : Target.FlowPenaltyWeight,
                MaxLaneSlicesPerCycle = (Changed.MaxLaneSlicesPerCycle != Reference.MaxLaneSlicesPerCycle)
                    ? Changed.MaxLaneSlicesPerCycle
                    : Target.MaxLaneSlicesPerCycle,
                RouteReorderWindow = (Changed.RouteReorderWindow != Reference.RouteReorderWindow)
                    ? Changed.RouteReorderWindow
                    : Target.RouteReorderWindow,
                EnableMultiRouteExchange = (Changed.EnableMultiRouteExchange != Reference.EnableMultiRouteExchange)
                    ? Changed.EnableMultiRouteExchange
                    : Target.EnableMultiRouteExchange,
                EnableCycleChainOptimization = (Changed.EnableCycleChainOptimization != Reference.EnableCycleChainOptimization)
                    ? Changed.EnableCycleChainOptimization
                    : Target.EnableCycleChainOptimization,
            };
        }

        /// <summary>Builds a value list that starts with the caller's value and has no repeats.</summary>
        /// <typeparam name="T">Value type of the setting.</typeparam>
        /// <param name="BaseValue">The caller's value; always index 0.</param>
        /// <param name="Values">Values from the search space, in listed order.</param>
        /// <returns>The caller's value followed by each new value once.</returns>
        static private List<T> WithBaseFirst<T>
            (
            T BaseValue,
            IEnumerable<T> Values
            )
        {
            List<T> Output = new List<T>();
            Output.Add(BaseValue);
            foreach (T Value in Values)
            {
                if (!Output.Contains(Value)) Output.Add(Value);
            }
            return Output;
        }

        /// <summary>Adds a screening run unless it equals the leader or is already listed.</summary>
        /// <param name="Group">List to add to.</param>
        /// <param name="Leader">Settings the run was derived from.</param>
        /// <param name="Candidate">Settings to add.</param>
        static private void AddIfNew
            (
            List<FieldSolverSettings> Group,
            FieldSolverSettings Leader,
            FieldSolverSettings Candidate
            )
        {
            // A value equal to the leader's is the leader's own plan, already run.
            if (Candidate.Equals(Leader) || Group.Contains(Candidate)) return;
            Group.Add(Candidate);
        }

        /// <summary>
        /// Orders two combinations. Fewer changed settings first; then a changed
        /// edge cap before an unchanged one, then minimum work length, then lane
        /// axis; among changed values, the order of the search-space list.
        /// </summary>
        /// <param name="Left">Value indexes of the first combination; index 0 is the caller's value.</param>
        /// <param name="Right">Value indexes of the second combination.</param>
        /// <returns>Negative when the first runs earlier, positive when later, zero when equal.</returns>
        static private Int32 CompareCombinations
            (
            (Int32 Edge, Int32 Work, Int32 Axis) Left,
            (Int32 Edge, Int32 Work, Int32 Axis) Right
            )
        {
            Int32 Result = CountChanges(Left).CompareTo(CountChanges(Right));
            if (Result != 0) return Result;
            Result = CompareIndex(Left.Edge, Right.Edge);
            if (Result != 0) return Result;
            Result = CompareIndex(Left.Work, Right.Work);
            if (Result != 0) return Result;
            return CompareIndex(Left.Axis, Right.Axis);
        }

        /// <summary>Counts the settings of a combination that differ from the caller's value.</summary>
        /// <param name="Index">Value indexes; index 0 is the caller's value.</param>
        /// <returns>Zero to three.</returns>
        static private Int32 CountChanges
            (
            (Int32 Edge, Int32 Work, Int32 Axis) Index
            )
        {
            Int32 Changes = 0;
            if (Index.Edge > 0) Changes += 1;
            if (Index.Work > 0) Changes += 1;
            if (Index.Axis > 0) Changes += 1;
            return Changes;
        }

        /// <summary>Orders two value indexes of one setting: changed before unchanged, then list order.</summary>
        /// <param name="Left">First value index; 0 is the caller's value.</param>
        /// <param name="Right">Second value index.</param>
        /// <returns>Negative when the first runs earlier, positive when later, zero when equal.</returns>
        static private Int32 CompareIndex
            (
            Int32 Left,
            Int32 Right
            )
        {
            Boolean LeftChanged = Left > 0;
            Boolean RightChanged = Right > 0;
            if (LeftChanged != RightChanged) return LeftChanged ? -1 : 1;
            return Left.CompareTo(Right);
        }
    }
}
