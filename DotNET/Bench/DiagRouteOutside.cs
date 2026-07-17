using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using HaulBuilder;

/// <summary>
/// Counts guidance waypoints outside the field mask for a sequenced route index.
/// </summary>
internal static class DiagRouteOutside
{
    /// <summary>
    /// Plans TheShop2 defaults and reports outside-field samples for one route.
    /// </summary>
    /// <param name="Args">Optional: 0-based pass index (default 6 = route 7).</param>
    /// <returns>Exit code (0 when no outside points).</returns>
    static public Int32 Run
        (
        String[] Args
        )
    {
        Int32 PassIndex = 6;
        if ((Args.Length > 0) && !Int32.TryParse(Args[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out PassIndex))
        {
            Console.Error.WriteLine("usage: --diag-route-outside [passIndex0]");
            return 2;
        }

        String RepoRoot = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String AgdPath = Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");
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
            Constants.HALF_SCALE,
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
        Pass PassN = FullRoute.OrderedPasses[PassIndex];
        Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { PassN }, 0.0);
        GuidanceResult Guidance = Swaths.BuildGuidance(Grid, Field, Mini, Constants.DEFAULT_BLADE_WIDTH_MM, Heading);
        List<GuidancePoint> Pts = Guidance.Points.Where(Point => Point.PassNo == 1).ToList();
        Int32 Outside = 0;
        Double MinY = Double.PositiveInfinity;
        Dictionary<String, Int32> ByAction = new Dictionary<String, Int32>();
        foreach (GuidancePoint Point in Pts)
        {
            if (Point.Y < MinY)
            {
                MinY = Point.Y;
            }

            if (Grid.ContainsWorldXy(Point.X, Point.Y))
            {
                continue;
            }

            Outside += 1;
            if (!ByAction.ContainsKey(Point.Action))
            {
                ByAction[Point.Action] = 0;
            }

            ByAction[Point.Action] += 1;
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "out {0} x={1:F2} y={2:F2}",
                Point.Action,
                Point.X,
                Point.Y));
        }

        String By = String.Join(", ", ByAction.Select(Kv => Kv.Key + "=" + Kv.Value.ToString(CultureInfo.InvariantCulture)));
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "route{0} pts={1} outside={2} ymin={3:F2} by={4}",
            PassIndex + 1,
            Pts.Count,
            Outside,
            MinY,
            By));
        return (Outside == 0) ? 0 : 1;
    }
}
