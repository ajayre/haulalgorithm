using System;
using System.Linq;
using HaulBuilder;

/// <summary>
/// Provides program behavior.
/// </summary>
internal static class Program
{
    /// <summary>
    /// Runs the command-line entry point.
    /// </summary>
    /// <param name="Args">The args used by the operation.</param>
    private static void Main
        (
        String[] Args
        )
    {
        if ((Args.Length > 0)
            && ((Args[0] == "--probe-haul-optimizer")
                || (Args[0] == "--optimize-haul")))
        {
            Environment.Exit(HaulOptimizationBench.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--make-gif"))
        {
            Environment.Exit(MakeLevelingAnimation.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--kd-bench"))
        {
            KdMicrobench.Run(5000, 8000, 100);
            return;
        }

        Console.Error.WriteLine(
            "usage: Bench [--probe-haul-optimizer|--optimize-haul] [--make-gif] [--kd-bench]");
        Environment.Exit(2);
    }
}
