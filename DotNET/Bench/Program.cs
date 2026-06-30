using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using HaulBuilder;

internal static class Program
{
    private static void Main
        (
        String[] args
        )
    {
        if ((args.Length > 0) && (args[0] == "--compare-paths"))
        {
            ComparePaths.Run(Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd")));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--make-gif"))
        {
            Environment.Exit(MakeLevelingAnimation.Run(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--kd-bench"))
        {
            KdMicrobench.Run(5000, 8000, 100);
            return;
        }

        String Fixture = args.Length > 0
            ? args[0]
            : Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd");
        Fixture = Path.GetFullPath(Fixture);

        // Defaults mirror Python landlevel CLI / HaulBuilderOptions (not legacy bench overrides).
        Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
        Stopwatch Sw = Stopwatch.StartNew();
        AgdField Field = Agd.Parse(Fixture);
        TimeSpan T0 = Sw.Elapsed;
        Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
        TimeSpan T1 = Sw.Elapsed;
        Balance.Compute(Grid, "check", Double.PositiveInfinity);
        FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
        TimeSpan T2 = Sw.Elapsed;
        List<Strip> StripList = Strips.TraceStrips(Grid, Flow, 400);
        TimeSpan T3 = Sw.Elapsed;
        PassResult PassResult = Passes.GeneratePasses(
            Grid,
            StripList,
            Constants.DEFAULT_TOLERANCE_MM,
            ScraperBcy,
            0,
            null,
            0.5,
            50.0,
            Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS,
            Flow,
            true,
            0.9,
            Obstacles.EMPTY_OBSTACLES);
        TimeSpan T4 = Sw.Elapsed;
        Route Route = Routing.SequencePasses(
            PassResult.Passes,
            1500,
            true,
            true,
            Constants.DEFAULT_RETURN_SPEED_MPH,
            Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT,
            Obstacles.EMPTY_OBSTACLES);
        TimeSpan T5 = Sw.Elapsed;
        Swaths.ReprofileKnots(
            Route,
            Grid,
            Constants.DEFAULT_BLADE_WIDTH_MM,
            3,
            Passes.EstimateSlopeHeading(Grid),
            0.0);
        TimeSpan T6 = Sw.Elapsed;
        Double MovedBcy = 0.0;
        for (Int32 Iter = 0; Iter < PassResult.Passes.Count; Iter += 1)
        {
            MovedBcy += PassResult.Passes[Iter].LoadBcy;
        }

        TimeParams Params = new TimeParams(
            Constants.DEFAULT_LOADING_SPEED_MPH,
            Constants.DEFAULT_HAUL_SPEED_MPH,
            Constants.DEFAULT_DUMPING_SPEED_MPH,
            Constants.DEFAULT_RETURN_SPEED_MPH,
            Constants.DEFAULT_ACCEL_TIME_S,
            Constants.DEFAULT_DECEL_TIME_S,
            Constants.DEFAULT_FIELD_EFFICIENCY);
        TimeResult Time = Timemodel.EstimateTime(Grid, Route, Params);
        TimeSpan T7 = Sw.Elapsed;
        Console.WriteLine(String.Format("fixture={0}", Fixture));
        Console.WriteLine(String.Format("grid={0}x{1}", Grid.Width, Grid.Height));
        Console.WriteLine(String.Format("parse+grid: {0:F2}s", (T1 - T0).TotalSeconds));
        Console.WriteLine(String.Format("flow: {0:F2}s", (T2 - T1).TotalSeconds));
        Console.WriteLine(String.Format("strips: {0:F2}s", (T3 - T2).TotalSeconds));
        Console.WriteLine(String.Format("passes({0}): {1:F2}s", PassResult.Passes.Count, (T4 - T3).TotalSeconds));
        Console.WriteLine(String.Format("route: {0:F2}s", (T5 - T4).TotalSeconds));
        Console.WriteLine(String.Format("knots: {0:F2}s", (T6 - T5).TotalSeconds));
        Console.WriteLine(String.Format("metrics: {0:F2}s", (T7 - T6).TotalSeconds));
        Console.WriteLine(String.Format("moved_bcy={0:F2} haul_m={1:F1} hours={2:F2}", MovedBcy, Time.HaulingDistM, Time.TotalTimeS / 3600.0));
        Console.WriteLine(String.Format("total: {0:F2}s", Sw.Elapsed.TotalSeconds));
    }
}
