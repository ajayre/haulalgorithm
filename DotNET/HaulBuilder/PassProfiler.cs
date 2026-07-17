using System;
using System.Diagnostics;

namespace HaulBuilder
{
    /// <summary>
    /// Optional pass-generation counters enabled with environment variable <c>HAUL_PROFILE=1</c>.
    /// </summary>
    internal static class PassProfiler
    {
        static public Boolean Enabled { get; } = String.Equals(
            Environment.GetEnvironmentVariable("HAUL_PROFILE"),
            "1",
            StringComparison.Ordinal);

        static public Int64 PassIterations;

        static public Int64 KdBuilds;

        static public Int64 KdBuildTicks;

        static public Int64 AnchorBatchQueries;

        static public Int64 NearestRemainingCalls;

        static public Int64 NearestRemainingTicks;

        static public Int64 FlowTubeCalls;

        static public Int64 FlowTubeSteps;

        static public Int64 FlowConsumes;

        static public Int64 PruneCalls;

        static public Int64 PruneTicks;

        static public Int64 BuildPassCalls;

        static public Int64 BuildPassTicks;

        static public Int64 ChooseFillSeedCalls;

        static public Int64 CutCellsListed;

        static public Int64 FillCellsListed;

        /// <summary>
        /// Adds ticks.
        /// </summary>
        /// <param name="Accumulator">The accumulator used by the operation.</param>
        /// <param name="StartTimestamp">The start timestamp used by the operation.</param>

        static public void AddTicks
            (
            ref Int64 Accumulator,
            Int64 StartTimestamp
            )
        {
            if (Enabled)
            {
                Accumulator += (Stopwatch.GetTimestamp() - StartTimestamp);
            }
        }

        /// <summary>
        /// Executes the dump operation.
        /// </summary>

        static public void Dump
            (
            )
        {
            if (!Enabled)
            {
                return;
            }

            Double TickToMs = Constants.MILLIMETERS_PER_METER / Stopwatch.Frequency;
            Console.WriteLine("--- pass profiler (C#) ---");
            Console.WriteLine(String.Format("pass_iterations={0}", PassIterations));
            Console.WriteLine(String.Format("cut_cells_listed={0} fill_cells_listed={1}", CutCellsListed, FillCellsListed));
            Console.WriteLine(String.Format("kd_builds={0} kd_build_ms={1:F1}", KdBuilds, KdBuildTicks * TickToMs));
            Console.WriteLine(String.Format("anchor_batch_queries={0}", AnchorBatchQueries));
            Console.WriteLine(String.Format("nearest_remaining_calls={0} nearest_remaining_ms={1:F1}", NearestRemainingCalls, NearestRemainingTicks * TickToMs));
            Console.WriteLine(String.Format("choose_fill_seed_calls={0}", ChooseFillSeedCalls));
            Console.WriteLine(String.Format("flow_tube_calls={0} flow_tube_steps={1} flow_consumes={2}", FlowTubeCalls, FlowTubeSteps, FlowConsumes));
            Console.WriteLine(String.Format("prune_calls={0} prune_ms={1:F1}", PruneCalls, PruneTicks * TickToMs));
            Console.WriteLine(String.Format("build_pass_calls={0} build_pass_ms={1:F1}", BuildPassCalls, BuildPassTicks * TickToMs));
        }
    }
}
