using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using HaulBuilder;

/// <summary>
/// Counts MOVE runs / keyhole-sized loops on selected routes (excess-turn diagnosis).
/// </summary>
internal static class DiagRouteLoops
{
    /// <summary>
    /// Prints MOVE-run stats for one or more 0-based pass indices.
    /// </summary>
    /// <param name="args">Optional pass indices (default 9 11).</param>
    /// <returns>Process exit code.</returns>
    public static Int32 Run(String[] args)
    {
        List<Int32> Indices = new List<Int32>();
        if (args.Length == 0)
        {
            Indices.Add(9);
            Indices.Add(11);
        }
        else
        {
            foreach (String Arg in args)
            {
                if (!Int32.TryParse(Arg, NumberStyles.Integer, CultureInfo.InvariantCulture, out Int32 Idx))
                {
                    Console.Error.WriteLine("usage: --diag-route-loops [passIndex ...]");
                    return 2;
                }

                Indices.Add(Idx);
            }
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
            Grid, StripList, Constants.DEFAULT_TOLERANCE_MM, ScraperBcy, 0, null, 0.5, 50.0,
            Constants.DEFAULT_MAX_DROP_HOP_FT * Constants.FEET_TO_METERS, Flow, true, 0.8, Obstacles.EMPTY_OBSTACLES);
        Route FullRoute = Routing.SequencePasses(
            PassResult.Passes, 1500, true, true,
            Constants.DEFAULT_RETURN_SPEED_MPH, Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT, Obstacles.EMPTY_OBSTACLES);
        Double Heading = Passes.EstimateSlopeHeading(Grid);
        Swaths.ReprofileKnots(FullRoute, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);
        Routing.RefineRouteOrder(
            FullRoute, 50, true, true,
            Constants.DEFAULT_RETURN_SPEED_MPH, Constants.DEFAULT_TURN_SPEED_MPH,
            Constants.DEFAULT_TURNING_CIRCLE_FT, 12, Obstacles.EMPTY_OBSTACLES);
        Double R = Constants.DEFAULT_TURNING_CIRCLE_FT * 0.5 * Constants.FEET_TO_METERS;
        Double Circ = 2.0 * Math.PI * R;

        foreach (Int32 Idx in Indices)
        {
            Pass PassN = FullRoute.OrderedPasses[Idx];
            Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { PassN }, 0.0);
            GuidanceResult Guidance = Swaths.BuildGuidance(
                Grid, Field, Mini, Constants.DEFAULT_BLADE_WIDTH_MM, Heading);
            List<GuidancePoint> Pts = Guidance.Points.Where(P => P.PassNo == 1).ToList();
            Int32 RawRev = CountNearReverseCorners(PassN.CutCells, Grid);
            List<Double> RunLens = new List<Double>();
            Double MoveLen = 0.0;
            for (Int32 I = 0; I < Pts.Count; )
            {
                if (Pts[I].Action != "MOVE")
                {
                    I += 1;
                    continue;
                }

                Int32 Start = I;
                Double Len = 0.0;
                while ((I < Pts.Count) && (Pts[I].Action == "MOVE"))
                {
                    if (I > Start)
                    {
                        Double Dx = Pts[I].X - Pts[I - 1].X;
                        Double Dy = Pts[I].Y - Pts[I - 1].Y;
                        Len += Math.Sqrt((Dx * Dx) + (Dy * Dy));
                    }

                    I += 1;
                }

                RunLens.Add(Len);
                MoveLen += Len;
            }

            Int32 Keyholeish = RunLens.Count(Len => Len > (0.75 * Circ));
            Console.WriteLine(String.Format(CultureInfo.InvariantCulture,
                "route{0}: pts={1} CUT={2} FILL={3} MOVE={4} cutCells={5} rawNearReverse={6}",
                Idx + 1,
                Pts.Count,
                Pts.Count(P => P.Action == "CUT"),
                Pts.Count(P => P.Action == "FILL"),
                Pts.Count(P => P.Action == "MOVE"),
                PassN.CutCells.Count,
                RawRev));
            Console.WriteLine(String.Format(CultureInfo.InvariantCulture,
                "  moveRuns={0} keyholeish={1} moveLen={2:F0}m (~{3:F1} full circles) R={4:F2}",
                RunLens.Count, Keyholeish, MoveLen, MoveLen / Circ, R));
            Console.WriteLine(String.Format(CultureInfo.InvariantCulture,
                "  runLens_m={0}",
                String.Join(",", RunLens.Select(Len => Len.ToString("F0", CultureInfo.InvariantCulture)))));
        }

        return 0;
    }

    /// <summary>
    /// Counts cut-cell corners whose inbound/outbound direction is nearly reverse.
    /// </summary>
    private static Int32 CountNearReverseCorners
        (
        IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)> cells,
        Grid grid
        )
    {
        Int32 Count = 0;
        for (Int32 K = 1; K + 1 < cells.Count; K += 1)
        {
            (Double X, Double Y) A = grid.CellCenterXy(cells[K - 1].Cell.Bx, cells[K - 1].Cell.By);
            (Double X, Double Y) B = grid.CellCenterXy(cells[K].Cell.Bx, cells[K].Cell.By);
            (Double X, Double Y) C = grid.CellCenterXy(cells[K + 1].Cell.Bx, cells[K + 1].Cell.By);
            Double Dx1 = B.X - A.X;
            Double Dy1 = B.Y - A.Y;
            Double Dx2 = C.X - B.X;
            Double Dy2 = C.Y - B.Y;
            Double N1 = Math.Sqrt((Dx1 * Dx1) + (Dy1 * Dy1));
            Double N2 = Math.Sqrt((Dx2 * Dx2) + (Dy2 * Dy2));
            if ((N1 < 1e-6) || (N2 < 1e-6))
            {
                continue;
            }

            if (((Dx1 * Dx2) + (Dy1 * Dy2)) / (N1 * N2) < -0.5)
            {
                Count += 1;
            }
        }

        return Count;
    }
}
