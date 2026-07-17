using System;
using System.Diagnostics;
using System.IO;
using HaulBuilder;

/// <summary>
/// Provides compare paths behavior.
/// </summary>
internal static class ComparePaths
{
    /// <summary>
    /// Runs the requested workflow.
    /// </summary>
    /// <param name="Fixture">The fixture used by the operation.</param>

    internal static void Run
        (
        String Fixture
        )
    {
        Double ScraperBcy = 21.0 / 1.25;
        AgdField Field = Agd.Parse(Fixture);
        Grid Grid = GridMath.BuildGrid(Field, 7.5);
        Balance.Compute(Grid, "check", Double.PositiveInfinity);
        FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
        System.Collections.Generic.List<Strip> StripList = Strips.TraceStrips(Grid, Flow, 400);

        RunCase("region_only (FlowAligned, Flow=null)", Grid, StripList, null, true, ScraperBcy);
        RunCase("flow_tube (FlowAligned + Flow)", Grid, StripList, Flow, true, ScraperBcy);
    }

    /// <summary>
    /// Executes the run case operation.
    /// </summary>
    /// <param name="Label">The label used by the operation.</param>
    /// <param name="Grid">Field grid used by the operation.</param>
    /// <param name="Strips">The strips used by the operation.</param>
    /// <param name="Flow">The flow used by the operation.</param>
    /// <param name="FlowAligned">The flow aligned used by the operation.</param>
    /// <param name="ScraperBcy">The scraper bcy used by the operation.</param>

    private static void RunCase
        (
        String Label,
        Grid Grid,
        System.Collections.Generic.List<Strip> Strips,
        FlowField? Flow,
        Boolean FlowAligned,
        Double ScraperBcy
        )
    {
        Stopwatch Sw = Stopwatch.StartNew();
        PassResult Result = Passes.GeneratePasses(
            Grid,
            Strips,
            10.0,
            ScraperBcy,
            0,
            null,
            Constants.HALF_SCALE,
            50.0,
            Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS,
            Flow,
            FlowAligned,
            0.9,
            Obstacles.EMPTY_OBSTACLES);
        Sw.Stop();
        Double Moved = 0.0;
        for (Int32 Iter = 0; Iter < Result.Passes.Count; Iter += 1)
        {
            Moved += Result.Passes[Iter].LoadBcy;
        }

        Console.WriteLine(String.Format("{0}: passes={1} moved={2:F1} time={3:F1}s", Label, Result.Passes.Count, Moved, Sw.Elapsed.TotalSeconds));
    }
}
