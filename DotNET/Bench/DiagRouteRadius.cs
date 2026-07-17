using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using HaulBuilder;

/// <summary>
/// Provides diag route radius behavior.
/// </summary>
internal static class DiagRouteRadius
{
    /// <summary>
    /// Reports blade-down bends on a route that violate the minimum turn radius.
    /// </summary>
    static public Int32 Run(String[] Args)
    {
        Int32 PassIndex = 6;
        if ((Args.Length > 0) && !Int32.TryParse(Args[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out PassIndex))
        {
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
            Grid, StripList, Constants.DEFAULT_TOLERANCE_MM, ScraperBcy, 0, null, Constants.HALF_SCALE, 50.0,
            Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS, Flow, true, 0.8, Obstacles.EMPTY_OBSTACLES);
        Route FullRoute = Routing.SequencePasses(
            PassResult.Passes, 1500, true, true,
            Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH, Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT, Obstacles.EMPTY_OBSTACLES);
        Double Heading = Passes.EstimateSlopeHeading(Grid);
        Swaths.ReprofileKnots(FullRoute, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);
        Pass PassN = FullRoute.OrderedPasses[PassIndex];
        Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { PassN }, 0.0);
        Double R = Constants.DEFAULT_TURNING_CIRCLE_FT * Constants.HALF_SCALE * Constants.FEET_TO_METERS;
        GuidanceResult Guidance = Swaths.BuildGuidance(Grid, Field, Mini, Constants.DEFAULT_BLADE_WIDTH_MM, Heading);
        List<GuidancePoint> Pts = Guidance.Points.Where(P => P.PassNo == 1).ToList();

        Int32 Viol = 0;
        Int32 Outside = 0;
        for (Int32 K = 1; K < Pts.Count - 1; K += 1)
        {
            if (!Grid.ContainsWorldXy(Pts[K].X, Pts[K].Y)) Outside += 1;
            String A0 = Pts[K - 1].Action;
            String A1 = Pts[K].Action;
            String A2 = Pts[K + 1].Action;
            if (A0 == "START" || A1 == "START" || A2 == "START" || A0 == "END" || A1 == "END" || A2 == "END") continue;
            Double Dx1 = Pts[K].X - Pts[K - 1].X;
            Double Dy1 = Pts[K].Y - Pts[K - 1].Y;
            Double Dx2 = Pts[K + 1].X - Pts[K].X;
            Double Dy2 = Pts[K + 1].Y - Pts[K].Y;
            Double Lab = Math.Sqrt(Dx1 * Dx1 + Dy1 * Dy1);
            Double Lbc = Math.Sqrt(Dx2 * Dx2 + Dy2 * Dy2);
            if (Lab < Constants.GEOMETRY_COMPARISON_TOLERANCE || Lbc < Constants.GEOMETRY_COMPARISON_TOLERANCE) continue;
            Double Cross = Math.Abs(Dx1 * Dy2 - Dy1 * Dx2);
            Double Dot = Dx1 * Dx2 + Dy1 * Dy2;
            Double Deg = Math.Abs(Math.Atan2(Cross, Dot) * (Constants.DEGREES_PER_HALF_TURN / Math.PI));
            if (Deg < 15.0) continue;
            Double La = Lab, Lb = Lbc;
            Double Lc = Math.Sqrt((Pts[K + 1].X - Pts[K - 1].X) * (Pts[K + 1].X - Pts[K - 1].X)
                + (Pts[K + 1].Y - Pts[K - 1].Y) * (Pts[K + 1].Y - Pts[K - 1].Y));
            Double Cross2 = Math.Abs(
                Pts[K - 1].X * (Pts[K].Y - Pts[K + 1].Y)
                + Pts[K].X * (Pts[K + 1].Y - Pts[K - 1].Y)
                + Pts[K + 1].X * (Pts[K - 1].Y - Pts[K].Y));
            Double Rad = (Cross2 < Constants.NUMERIC_COMPARISON_TOLERANCE) ? Double.PositiveInfinity : (La * Lb * Lc) / (2.0 * Cross2);
            if (Rad >= R * 0.99) continue;
            Viol += 1;
            Console.WriteLine(String.Format(CultureInfo.InvariantCulture,
                "tight[{0}] {1}->{2}->{3} deg={4:F1} r={5:F2}m < R={6:F2}m x={7:F1} y={8:F1}",
                K, A0, A1, A2, Deg, Rad, R, Pts[K].X, Pts[K].Y));
        }
        Console.WriteLine(String.Format(CultureInfo.InvariantCulture,
            "route{0} pts={1} tight={2} outside={3} R={4:F2}",
            PassIndex + 1, Pts.Count, Viol, Outside, R));
        return (Viol == 0 && Outside == 0) ? 0 : 1;
    }
}
