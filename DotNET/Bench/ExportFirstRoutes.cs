using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using HaulBuilder;

/// <summary>
/// Builds a default TheShop2 (or other AGD) plan once and writes guidance PNGs for the first N routes.
/// </summary>
/// <remarks>
/// Re-run often while tuning turns/lineup:
/// <c>dotnet run --project DotNET/Bench -- --export-first-routes</c>
/// Optional args: count, output directory, AGD path.
/// </remarks>
internal static class ExportFirstRoutes
{
    /// <summary>
    /// Default number of leading sequenced routes to export.
    /// </summary>
    public const Int32 DEFAULT_ROUTE_COUNT = 12;

    /// <summary>
    /// Exports guidance PNGs for the first <paramref name="routeCount"/> sequenced passes.
    /// </summary>
    /// <param name="args">
    /// Optional: <c>[count] [outDir] [agdPath]</c>. Defaults: 12 routes,
    /// <c>assets/theshop2_first_routes/</c>, <c>DotNET/TheShop2_2ft.agd</c>.
    /// </param>
    /// <returns>Process exit code (0 on success).</returns>
    public static Int32 Run
        (
        String[] args
        )
    {
        Int32 RouteCount = DEFAULT_ROUTE_COUNT;
        if ((args.Length > 0) && !Int32.TryParse(args[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out RouteCount))
        {
            Console.Error.WriteLine("usage: --export-first-routes [count] [outDir] [agdPath]");
            return 2;
        }

        if (RouteCount < 1)
        {
            Console.Error.WriteLine("count must be >= 1");
            return 2;
        }

        String RepoRoot = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String OutDir = (args.Length > 1)
            ? Path.GetFullPath(args[1])
            : Path.Combine(RepoRoot, "assets", "theshop2_first_routes");
        String AgdPath = (args.Length > 2)
            ? Path.GetFullPath(args[2])
            : Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");

        if (!File.Exists(AgdPath))
        {
            Console.Error.WriteLine("AGD not found: " + AgdPath);
            return 1;
        }

        Directory.CreateDirectory(OutDir);
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "export-first-routes: count={0} grid_ft={1} blade_mm={2} agd={3}",
            RouteCount,
            Constants.DEFAULT_GRID_SIZE_FT,
            Constants.DEFAULT_BLADE_WIDTH_MM,
            AgdPath));

        Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
        AgdField Field = Agd.Parse(AgdPath);
        Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
        Balance.Compute(Grid, "check", Double.PositiveInfinity);
        FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
        List<Strip> StripList = Strips.TraceStrips(Grid, Flow, 400);
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
            0.8,
            Obstacles.EMPTY_OBSTACLES);
        Route FullRoute = Routing.SequencePasses(
            PassResult.Passes,
            1500,
            true,
            true,
            Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
            Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT,
            Obstacles.EMPTY_OBSTACLES);
        Double Heading = Passes.EstimateSlopeHeading(Grid);
        Swaths.ReprofileKnots(FullRoute, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);
        Routing.RefineRouteOrder(
            FullRoute, 50, true, true,
            Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH, Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT, 12, Obstacles.EMPTY_OBSTACLES);

        Int32 N = Math.Min(RouteCount, FullRoute.OrderedPasses.Count);
        Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0;
        for (Int32 PassIndex = 0; PassIndex < N; PassIndex += 1)
        {
            Pass PassN = FullRoute.OrderedPasses[PassIndex];
            Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { PassN }, 0.0);
            GuidanceResult Guidance = Swaths.BuildGuidance(
                Grid, Field, Mini, Constants.DEFAULT_BLADE_WIDTH_MM, Heading);
            List<GuidancePoint> Pts = Guidance.Points.Where(Point => Point.PassNo == 1).ToList();
            Byte[] Png = Render.RenderSinglePairPng(Grid, Pts, BladeM, pixelsPerBin: 40, padCells: 6);
            String OutPath = Path.Combine(
                OutDir,
                String.Format(CultureInfo.InvariantCulture, "route_{0:D2}.png", PassIndex + 1));
            File.WriteAllBytes(OutPath, Png);
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "wrote {0} ({1} bytes, {2} waypoints, haul_m={3:F1})",
                OutPath,
                Png.Length,
                Pts.Count,
                PassN.HaulDistanceM));
        }

        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "done: {0} route PNG(s) in {1} (planned passes={2})",
            N,
            OutDir,
            FullRoute.OrderedPasses.Count));
        return 0;
    }
}
