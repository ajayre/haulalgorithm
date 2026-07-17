using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
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
        if ((args.Length > 0)
            && ((args[0] == "--probe-haul-optimizer")
                || (args[0] == "--optimize-haul")))
        {
            Environment.Exit(HaulOptimizationBench.Run(args.Skip(1).ToArray()));
            return;
        }

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

        if ((args.Length > 0) && (args[0] == "--diag-route3-fill"))
        {
            Environment.Exit(DiagRoute3Fill.Run(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--diag-cut-path"))
        {
            Environment.Exit(DiagCutPath.Run(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--diag-route-loops"))
        {
            Environment.Exit(DiagRouteLoops.Run(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--diag-route-radius"))
        {
            Environment.Exit(DiagRouteRadius.Run(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--diag-route-outside"))
        {
            Environment.Exit(DiagRouteOutside.Run(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--export-first-routes"))
        {
            Environment.Exit(ExportFirstRoutes.Run(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--compare-guidance-coverage"))
        {
            Environment.Exit(GuidancePlanner.Run(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--report-whole-guidance"))
        {
            Environment.Exit(GuidancePlanner.ReportWholeField(
                args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--profile-guidance-coverage"))
        {
            Environment.Exit(GuidancePlanner.ProfileGuidanceCoverage(
                args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--diagnose-guidance-connector"))
        {
            Environment.Exit(GuidancePlanner.DiagnoseGuidanceConnector(
                args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--export-routes-1-3"))
        {
            Environment.Exit(GuidancePlanner.ExportFirstThree(args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--export-routes-3-5"))
        {
            Environment.Exit(GuidancePlanner.ExportRoutesThreeToFive(
                args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--export-route-range"))
        {
            Environment.Exit(GuidancePlanner.ExportRouteRange(
                args.Skip(1).ToArray()));
            return;
        }

        if ((args.Length > 0) && (args[0] == "--route2-png"))
        {
            String OutPath = (args.Length > 1)
                ? Path.GetFullPath(args[1])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "assets", "theshop2_route2_actual_dotnet.png"));
            String AgdPath = (args.Length > 2)
                ? Path.GetFullPath(args[2])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd"));
            WriteRoutePng(AgdPath, OutPath, passIndex: 1);
            return;
        }

        if ((args.Length > 0) && (args[0] == "--route11-png"))
        {
            String OutPath = (args.Length > 1)
                ? Path.GetFullPath(args[1])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "assets", "theshop2_route11_actual_dotnet.png"));
            String AgdPath = (args.Length > 2)
                ? Path.GetFullPath(args[2])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd"));
            WriteRoutePng(AgdPath, OutPath, passIndex: 10);
            return;
        }

        if ((args.Length > 0) && (args[0] == "--dump-early-passes"))
        {
            String AgdPath = (args.Length > 1)
                ? Path.GetFullPath(args[1])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd"));
            DumpEarlyPasses(AgdPath);
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
            0.8,
            Obstacles.EMPTY_OBSTACLES);
        TimeSpan T4 = Sw.Elapsed;
        Route Route = Routing.SequencePasses(
            PassResult.Passes,
            1500,
            true,
            true,
            Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
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
            Constants.DEFAULT_CUT_SPEED_MPH,
            Constants.DEFAULT_LOADED_TRAVEL_SPEED_MPH,
            Constants.DEFAULT_FILL_SPEED_MPH,
            Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
            Constants.DEFAULT_ACCELERATION_TIME_S,
            Constants.DEFAULT_DECELERATION_TIME_S,
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

    /// <summary>
    /// Plans TheShop2 (or another AGD) and writes the sequenced route-2 single-pair PNG.
    /// </summary>
    /// <param name="agdPath">Path to the AGD fixture.</param>
    /// <param name="outPath">Destination PNG path.</param>
    /// <summary>
    /// Writes a single-pass guidance PNG for TheShop2 (1-based pass display via 0-based index).
    /// </summary>
    /// <param name="agdPath">Path to the AGD fixture.</param>
    /// <param name="outPath">Output PNG path.</param>
    /// <param name="passIndex">0-based index into the sequenced route.</param>
    private static void WriteRoutePng
        (
        String agdPath,
        String outPath,
        Int32 passIndex
        )
    {
        // Match the Python single-route render: default grid/blade flow-aligned plan, one ordered pass.
        Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
        AgdField Field = Agd.Parse(agdPath);
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

        if ((passIndex < 0) || (passIndex >= FullRoute.OrderedPasses.Count))
        {
            throw new ArgumentOutOfRangeException(
                nameof(passIndex),
                passIndex,
                String.Format(
                    CultureInfo.InvariantCulture,
                    "pass index {0} out of range (n={1})",
                    passIndex,
                    FullRoute.OrderedPasses.Count));
        }

        Pass PassN = FullRoute.OrderedPasses[passIndex];
        Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { PassN }, 0.0);
        GuidanceResult Guidance = Swaths.BuildGuidance(Grid, Field, Mini, Constants.DEFAULT_BLADE_WIDTH_MM, Heading);
        List<GuidancePoint> Pts = Guidance.Points.Where(Point => Point.PassNo == 1).ToList();
        Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / 1000.0;
        Byte[] Png = Render.RenderSinglePairPng(Grid, Pts, BladeM, pixelsPerBin: 40, padCells: 6);

        String? Dir = Path.GetDirectoryName(outPath);
        if (!String.IsNullOrEmpty(Dir))
        {
            Directory.CreateDirectory(Dir);
        }

        File.WriteAllBytes(outPath, Png);
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "wrote {0} ({1} bytes, {2} waypoints, route={3}, haul_m={4:F1})",
            outPath,
            Png.Length,
            Pts.Count,
            passIndex + 1,
            PassN.HaulDistanceM));
    }

    /// <summary>
    /// Prints JSON fingerprints for the first generated and ordered passes (parity debugging).
    /// </summary>
    /// <param name="agdPath">Path to the AGD fixture.</param>
    private static void DumpEarlyPasses
        (
        String agdPath
        )
    {
        Double ScraperBcy = Constants.DEFAULT_SCRAPER_CAPACITY_LCY / Constants.DEFAULT_SWELL_FACTOR;
        AgdField Field = Agd.Parse(agdPath);
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

        // Snapshot generation fingerprints before knot reprofile mutates shared Pass instances.
        String FpPath = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(agdPath)!, "..", "assets", "cs_pass_fps.txt"));
        Directory.CreateDirectory(Path.GetDirectoryName(FpPath)!);
        using (StreamWriter Writer = new StreamWriter(FpPath))
        {
            Writer.WriteLine(String.Format(CultureInfo.InvariantCulture, "n={0}", PassResult.Passes.Count));
            for (Int32 Iter = 0; Iter < PassResult.Passes.Count; Iter += 1)
            {
                Pass Pass = PassResult.Passes[Iter];
                Writer.WriteLine(String.Format(
                    CultureInfo.InvariantCulture,
                    "{0}\t{1}\t{2:F6}\t{3:F6}\t{4}\t{5}",
                    Iter,
                    PassFingerprint(Pass),
                    Pass.HaulDistanceM,
                    Pass.LoadBcy,
                    Pass.CutCells.Count,
                    Pass.CutCells.Count > 0 ? String.Format(CultureInfo.InvariantCulture, "({0}, {1})", Pass.CutCells[0].Cell.Bx, Pass.CutCells[0].Cell.By) : "None"));
            }
        }

        Console.WriteLine("wrote " + FpPath);
        Console.WriteLine("ORDER_PRE " + String.Join(",", FullRoute.Order.Take(20)));
        Pass Pass0 = PassResult.Passes[0];
        Int32 PyRoute2Idx = -1;
        for (Int32 Iter = 0; Iter < PassResult.Passes.Count; Iter += 1)
        {
            Pass Cand = PassResult.Passes[Iter];
            if ((Cand.CutCells.Count == 39)
                && (Math.Abs(Cand.HaulDistanceM - 25.558257) < 0.01)
                && (Cand.CutCells[0].Cell.Bx == 65)
                && (Cand.CutCells[0].Cell.By == 3))
            {
                PyRoute2Idx = Iter;
                break;
            }
        }

        Console.WriteLine(String.Format(CultureInfo.InvariantCulture, "CS_IDX_OF_PY_ROUTE2_GEOM={0}", PyRoute2Idx));
        foreach (Int32 Idx in new Int32[] { 1148, FullRoute.Order[1], PyRoute2Idx })
        {
            if ((Idx < 0) || (Idx >= PassResult.Passes.Count))
            {
                continue;
            }

            Pass Cand = PassResult.Passes[Idx];
            Double Dx = Cand.CutStartXy.X - Pass0.FillStartXy.X;
            Double Dy = Cand.CutStartXy.Y - Pass0.FillStartXy.Y;
            Double Dist = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Console.WriteLine(String.Format(
                CultureInfo.InvariantCulture,
                "FROM0_PRE idx={0} deadhead={1:F6} haul={2:F6} first_cut=({3},{4}) ncut={5}",
                Idx,
                Dist,
                Cand.HaulDistanceM,
                Cand.CutCells[0].Cell.Bx,
                Cand.CutCells[0].Cell.By,
                Cand.CutCells.Count));
        }

        Double Heading = Passes.EstimateSlopeHeading(Grid);
        Swaths.ReprofileKnots(FullRoute, Grid, Constants.DEFAULT_BLADE_WIDTH_MM, 3, Heading, 0.0);

        for (Int32 Iter = 0; Iter < 3; Iter += 1)
        {
            PrintPassFingerprint(PassResult.Passes[Iter], String.Format(CultureInfo.InvariantCulture, "gen{0}", Iter));
        }

        for (Int32 Iter = 0; Iter < 5; Iter += 1)
        {
            PrintPassFingerprint(FullRoute.OrderedPasses[Iter], String.Format(CultureInfo.InvariantCulture, "ord{0}", Iter + 1));
        }

        Double Moved = PassResult.Passes.Sum(Pass => Pass.LoadBcy);
        Double Haul = PassResult.Passes.Sum(Pass => Pass.HaulDistanceM);
        Console.WriteLine(String.Format(CultureInfo.InvariantCulture, "TOTAL {0} {1:F2} {2:F1}", PassResult.Passes.Count, Moved, Haul));
        Console.WriteLine("ORDER " + String.Join(",", FullRoute.Order.Take(20)));

        String CellPath = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(agdPath)!, "..", "assets", "cs_cells.txt"));
        using (StreamWriter Writer = new StreamWriter(CellPath))
        {
            for (Int32 Idx = 0; Idx < 5; Idx += 1)
            {
                Pass Pass = PassResult.Passes[Idx];
                String Cells = String.Join(",", Pass.CutCells.Select(Entry =>
                    String.Format(CultureInfo.InvariantCulture, "({0},{1},{2:F10})", Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM)));
                Writer.WriteLine(String.Format(CultureInfo.InvariantCulture, "CS{0}|{1}|{2:F10}|{3}", Idx, Pass.CutCells.Count, Pass.HaulDistanceM, Cells));
            }

            for (Int32 Idx = 97; Idx <= 99; Idx += 1)
            {
                Pass Pass = PassResult.Passes[Idx];
                String Cells = String.Join(",", Pass.CutCells.Select(Entry =>
                    String.Format(CultureInfo.InvariantCulture, "({0},{1},{2:F8})", Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM)));
                Writer.WriteLine(String.Format(CultureInfo.InvariantCulture, "CS{0}|ncut={1}|haul={2:F10}|load={3:F10}|first={4}|cells={5}", Idx, Pass.CutCells.Count, Pass.HaulDistanceM, Pass.LoadBcy, Pass.CutCells.Count > 0 ? $"({Pass.CutCells[0].Cell.Bx},{Pass.CutCells[0].Cell.By})" : "none", Cells));
            }
        }

        Console.WriteLine("wrote " + CellPath);
    }

    /// <summary>
    /// Builds a short MD5 fingerprint of haul, load, and cut endpoints for parity diffs.
    /// </summary>
    /// <param name="pass">Pass to fingerprint.</param>
    /// <returns>12-character hex digest prefix.</returns>
    private static String PassFingerprint
        (
        Pass pass
        )
    {
        using System.Security.Cryptography.MD5 Md5 = System.Security.Cryptography.MD5.Create();
        List<Byte> Bytes = new List<Byte>();
        Bytes.AddRange(BitConverter.GetBytes(pass.HaulDistanceM));
        Bytes.AddRange(BitConverter.GetBytes(pass.LoadBcy));
        Bytes.AddRange(BitConverter.GetBytes(pass.CutCells.Count));
        Int32 TakeFront = Math.Min(5, pass.CutCells.Count);
        for (Int32 Iter = 0; Iter < TakeFront; Iter += 1)
        {
            Bytes.AddRange(BitConverter.GetBytes(pass.CutCells[Iter].Cell.Bx));
            Bytes.AddRange(BitConverter.GetBytes(pass.CutCells[Iter].Cell.By));
            Bytes.AddRange(BitConverter.GetBytes(pass.CutCells[Iter].DepthM));
        }

        Int32 TailStart = Math.Max(0, pass.CutCells.Count - 3);
        for (Int32 Iter = TailStart; Iter < pass.CutCells.Count; Iter += 1)
        {
            Bytes.AddRange(BitConverter.GetBytes(pass.CutCells[Iter].Cell.Bx));
            Bytes.AddRange(BitConverter.GetBytes(pass.CutCells[Iter].Cell.By));
            Bytes.AddRange(BitConverter.GetBytes(pass.CutCells[Iter].DepthM));
        }

        Byte[] Hash = Md5.ComputeHash(Bytes.ToArray());
        return Convert.ToHexString(Hash).Substring(0, 12).ToLowerInvariant();
    }

    /// <summary>
    /// Writes one pass fingerprint line as compact JSON for cross-language comparison.
    /// </summary>
    /// <param name="pass">Pass to summarize.</param>
    /// <param name="label">Fingerprint label.</param>
    private static void PrintPassFingerprint
        (
        Pass pass,
        String label
        )
    {
        Int32 Uniq = pass.CutCells.Select(Entry => (Entry.Cell.Bx, Entry.Cell.By)).Distinct().Count();
        String First = String.Join(",", pass.CutCells.Take(8).Select(Entry =>
            String.Format(CultureInfo.InvariantCulture, "[{0},{1},{2:F6}]", Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM)));
        String Last = String.Join(",", pass.CutCells.Skip(Math.Max(0, pass.CutCells.Count - 5)).Select(Entry =>
            String.Format(CultureInfo.InvariantCulture, "[{0},{1},{2:F6}]", Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM)));
        String CutH = pass.CutHeadingDeg.HasValue
            ? pass.CutHeadingDeg.Value.ToString("F3", CultureInfo.InvariantCulture)
            : "null";
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "{{\"label\":\"{0}\",\"ncut\":{1},\"nfill\":{2},\"uniq_cut\":{3},\"haul_m\":{4:F3},\"load_bcy\":{5:F4},\"cut_h\":{6},\"cut_start\":[{7:F3},{8:F3}],\"cut_end\":[{9:F3},{10:F3}],\"fill_start\":[{11:F3},{12:F3}],\"first_cut\":[{13}],\"last_cut\":[{14}]}}",
            label,
            pass.CutCells.Count,
            pass.FillCells.Count,
            Uniq,
            pass.HaulDistanceM,
            pass.LoadBcy,
            CutH,
            pass.CutStartXy.X,
            pass.CutStartXy.Y,
            pass.CutEndXy.X,
            pass.CutEndXy.Y,
            pass.FillStartXy.X,
            pass.FillStartXy.Y,
            First,
            Last));
    }
}
