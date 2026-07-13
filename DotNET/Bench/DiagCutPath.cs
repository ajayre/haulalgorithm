using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using HaulBuilder;

/// <summary>
/// Dumps cut-cell order and knot flags for early TheShop2 routes.
/// </summary>
internal static class DiagCutPath
{
    /// <summary>
    /// Prints cut-cell paths for routes 10 and 12 (0-based 9 and 11).
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
        Int32 Fixed = Swaths.ReprofileKnots(FullRoute, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);
        Console.WriteLine(String.Format(CultureInfo.InvariantCulture, "reprofiled_patches={0}", Fixed));
        foreach (Int32 Idx in new Int32[] { 9, 11 })
        {
            Pass PassN = FullRoute.OrderedPasses[Idx];
            Console.WriteLine(String.Format(CultureInfo.InvariantCulture,
                "route{0} cut={1} knotCut={2}",
                Idx + 1, PassN.CutCells.Count, PassN.CutKnotReduced));
            for (Int32 I = 0; I < PassN.CutCells.Count; I += 1)
            {
                (Int32 Bx, Int32 By) C = PassN.CutCells[I].Cell;
                Console.WriteLine(String.Format(CultureInfo.InvariantCulture, "  [{0}] ({1},{2})", I, C.Bx, C.By));
            }
        }

        return 0;
    }
}
