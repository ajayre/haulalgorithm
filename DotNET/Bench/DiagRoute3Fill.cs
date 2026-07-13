using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using HaulBuilder;

/// <summary>
/// Diagnostics for fill-vs-map confusion on early TheShop2 routes.
/// </summary>
internal static class DiagRoute3Fill
{
    /// <summary>
    /// Prints HAUL/FILL grade deltas for routes that look like fill-in-cut on PNGs.
    /// </summary>
    /// <param name="args">Unused.</param>
    /// <returns>Process exit code.</returns>
    public static Int32 Run(String[] args)
    {
        String RepoRoot = Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", ".."));
        String AgdPath = Path.Combine(RepoRoot, "DotNET", "TheShop2_2ft.agd");
        Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
        AgdField Field = Agd.Parse(AgdPath);
        Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
        Balance.Compute(Grid, "check", Double.PositiveInfinity);
        FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
        List<Strip> StripList = Strips.TraceStrips(Grid, Flow, 400);
        PassResult PassResult = Passes.GeneratePasses(
            Grid, StripList, Constants.DEFAULT_TOLERANCE_MM, ScraperBcy, 0, null, 0.5, 50.0,
            Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS, Flow, true, 0.8, Obstacles.EMPTY_OBSTACLES);
        Route FullRoute = Routing.SequencePasses(
            PassResult.Passes, 1500, true, true,
            Constants.DEFAULT_RETURN_SPEED_MPH, Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT, Obstacles.EMPTY_OBSTACLES);
        Double Heading = Passes.EstimateSlopeHeading(Grid);
        Swaths.ReprofileKnots(FullRoute, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);

        Console.WriteLine("PNG paints HAUL+FILL as one thick blue strip; cyan marker = first HAUL (not first FILL).");
        Console.WriteLine("delta_mm = existing-target (>0 above grade/cut, <0 below grade/fill)");
        foreach (Int32 RouteNo in new Int32[] { 3, 4, 10, 11, 12 })
        {
            Pass Pass = FullRoute.OrderedPasses[RouteNo - 1];
            Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { Pass }, 0.0);
            GuidanceResult G = Swaths.BuildGuidance(Grid, Field, Mini, Constants.DEFAULT_BLADE_WIDTH_MM, Heading);
            List<GuidancePoint> Pts = G.Points.Where(P => P.PassNo == 1).ToList();
            GuidancePoint? FirstHaul = Pts.FirstOrDefault(P => P.Action == "HAUL");
            GuidancePoint? FirstFill = Pts.FirstOrDefault(P => P.Action == "FILL");
            Int32 FillAbove = 0;
            foreach (var Item in Pass.FillCells)
            {
                if ((Grid.Existing[Item.Cell.By, Item.Cell.Bx] - Grid.Target[Item.Cell.By, Item.Cell.Bx]) > 0.01)
                {
                    FillAbove += 1;
                }
            }

            Console.WriteLine(String.Format(CultureInfo.InvariantCulture,
                "route {0}: firstHAUL_delta_mm={1} firstFILL_delta_mm={2} fillCells_above={3}/{4} haul_pts={5} fill_pts={6}",
                RouteNo,
                FirstHaul is null ? "n/a" : DeltaMm(Grid, FirstHaul).ToString("F1", CultureInfo.InvariantCulture),
                FirstFill is null ? "n/a" : DeltaMm(Grid, FirstFill).ToString("F1", CultureInfo.InvariantCulture),
                FillAbove,
                Pass.FillCells.Count,
                Pts.Count(P => P.Action == "HAUL"),
                Pts.Count(P => P.Action == "FILL")));
        }

        return 0;
    }

    /// <summary>
    /// Existing-minus-target at the grid cell containing a guidance point.
    /// </summary>
    private static Double DeltaMm(Grid grid, GuidancePoint point)
    {
        Int32 Bx = (Int32)Math.Floor((point.X - grid.MinX) / grid.GridSizeM);
        Int32 By = (Int32)Math.Floor((point.Y - grid.MinY) / grid.GridSizeM);
        Bx = Math.Max(0, Math.Min(grid.Width - 1, Bx));
        By = Math.Max(0, Math.Min(grid.Height - 1, By));
        return (grid.Existing[By, Bx] - grid.Target[By, Bx]) * 1000.0;
    }
}
