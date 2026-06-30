using System;
using System.Diagnostics;

namespace HaulBuilder
{
    /// <summary>
    /// Optional pass-generation counters enabled with environment variable <c>HAUL_PROFILE=1</c>.
    /// </summary>
    internal static class PassProfiler
    {
        public static Boolean Enabled { get; } = String.Equals(
            Environment.GetEnvironmentVariable("HAUL_PROFILE"),
            "1",
            StringComparison.Ordinal);

        public static Int64 PassIterations;

        public static Int64 KdBuilds;

        public static Int64 KdBuildTicks;

        public static Int64 AnchorBatchQueries;

        public static Int64 NearestRemainingCalls;

        public static Int64 NearestRemainingTicks;

        public static Int64 FlowTubeCalls;

        public static Int64 FlowTubeSteps;

        public static Int64 FlowConsumes;

        public static Int64 PruneCalls;

        public static Int64 PruneTicks;

        public static Int64 BuildPassCalls;

        public static Int64 BuildPassTicks;

        public static Int64 ChooseFillSeedCalls;

        public static Int64 CutCellsListed;

        public static Int64 FillCellsListed;

        public static void AddTicks
            (
            ref Int64 accumulator,
            Int64 startTimestamp
            )
        {
            if (Enabled)
            {
                accumulator += (Stopwatch.GetTimestamp() - startTimestamp);
            }
        }

        public static void Dump
            (
            )
        {
            if (!Enabled)
            {
                return;
            }

            Double TickToMs = 1000.0 / Stopwatch.Frequency;
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
