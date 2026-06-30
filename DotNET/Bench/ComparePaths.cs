using System;
using System.Diagnostics;
using System.IO;
using HaulBuilder;

internal static class ComparePaths
{
    internal static void Run
        (
        String fixture
        )
    {
        Double ScraperBcy = 21.0 / 1.25;
        AgdField Field = Agd.Parse(fixture);
        Grid Grid = GridMath.BuildGrid(Field, 7.5);
        Balance.Compute(Grid, "check", Double.PositiveInfinity);
        FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
        System.Collections.Generic.List<Strip> StripList = Strips.TraceStrips(Grid, Flow, 400);

        RunCase("region_only (FlowAligned, Flow=null)", Grid, StripList, null, true, ScraperBcy);
        RunCase("flow_tube (FlowAligned + Flow)", Grid, StripList, Flow, true, ScraperBcy);
    }

    private static void RunCase
        (
        String label,
        Grid grid,
        System.Collections.Generic.List<Strip> strips,
        FlowField? flow,
        Boolean flowAligned,
        Double scraperBcy
        )
    {
        Stopwatch Sw = Stopwatch.StartNew();
        PassResult Result = Passes.GeneratePasses(
            grid,
            strips,
            10.0,
            scraperBcy,
            0,
            null,
            0.5,
            50.0,
            Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS,
            flow,
            flowAligned,
            0.9,
            Obstacles.EMPTY_OBSTACLES);
        Sw.Stop();
        Double Moved = 0.0;
        for (Int32 Iter = 0; Iter < Result.Passes.Count; Iter += 1)
        {
            Moved += Result.Passes[Iter].LoadBcy;
        }

        Console.WriteLine(String.Format("{0}: passes={1} moved={2:F1} time={3:F1}s", label, Result.Passes.Count, Moved, Sw.Elapsed.TotalSeconds));
    }
}
