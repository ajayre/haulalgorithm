using System;
using System.Linq;

/// <summary>Provides the isolated ponding comparison entry point.</summary>
internal static class Program
{
    /// <summary>Runs the ponding proof utility or comparison-map generator.</summary>
    /// <param name="Args">
    /// Proof args forwarded to <see cref="FastPondingProof.Run"/>, or
    /// <c>maps</c> followed by comparison-map args.
    /// </param>
    static private void Main
        (
        String[] Args
        )
    {
        if (Args.Length > 0
            && String.Equals(Args[0], "maps", StringComparison.OrdinalIgnoreCase))
        {
            Environment.Exit(PondingComparisonMaps.Run(
                Args.Skip(1).ToArray()));
            return;
        }
        Environment.Exit(FastPondingProof.Run(Args));
    }
}
