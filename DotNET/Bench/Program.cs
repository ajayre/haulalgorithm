using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using HaulBuilder;

/// <summary>
/// Provides program behavior.
/// </summary>
internal static class Program
{
    /// <summary>
    /// Runs the command-line entry point.
    /// </summary>
    /// <param name="Args">The args used by the operation.</param>

    private static void Main
        (
        String[] Args
        )
    {
        if ((Args.Length > 0)
            && ((Args[0] == "--probe-haul-optimizer")
                || (Args[0] == "--optimize-haul")))
        {
            Environment.Exit(HaulOptimizationBench.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--compare-paths"))
        {
            ComparePaths.Run(Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd")));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--make-gif"))
        {
            Environment.Exit(MakeLevelingAnimation.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--kd-bench"))
        {
            KdMicrobench.Run(5000, 8000, 100);
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--diag-route3-fill"))
        {
            Environment.Exit(DiagRoute3Fill.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--diag-cut-path"))
        {
            Environment.Exit(DiagCutPath.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--diag-route-loops"))
        {
            Environment.Exit(DiagRouteLoops.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--diag-route-radius"))
        {
            Environment.Exit(DiagRouteRadius.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--diag-route-outside"))
        {
            Environment.Exit(DiagRouteOutside.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--export-first-routes"))
        {
            Environment.Exit(ExportFirstRoutes.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--compare-guidance-coverage"))
        {
            Environment.Exit(GuidancePlanner.Run(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--report-whole-guidance"))
        {
            Environment.Exit(GuidancePlanner.ReportWholeField(
                Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--profile-guidance-coverage"))
        {
            Environment.Exit(GuidancePlanner.ProfileGuidanceCoverage(
                Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--diagnose-guidance-connector"))
        {
            Environment.Exit(GuidancePlanner.DiagnoseGuidanceConnector(
                Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--export-routes-1-3"))
        {
            Environment.Exit(GuidancePlanner.ExportFirstThree(Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--export-routes-3-5"))
        {
            Environment.Exit(GuidancePlanner.ExportRoutesThreeToFive(
                Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--export-route-range"))
        {
            Environment.Exit(GuidancePlanner.ExportRouteRange(
                Args.Skip(1).ToArray()));
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--route2-png"))
        {
            String OutPath = (Args.Length > 1)
                ? Path.GetFullPath(Args[1])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "assets", "theshop2_route2_actual_dotnet.png"));
            String AgdPath = (Args.Length > 2)
                ? Path.GetFullPath(Args[2])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd"));
            WriteRoutePng(AgdPath, OutPath, PassIndex: 1);
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--route11-png"))
        {
            String OutPath = (Args.Length > 1)
                ? Path.GetFullPath(Args[1])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", "assets", "theshop2_route11_actual_dotnet.png"));
            String AgdPath = (Args.Length > 2)
                ? Path.GetFullPath(Args[2])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd"));
            WriteRoutePng(AgdPath, OutPath, PassIndex: 10);
            return;
        }

        if ((Args.Length > 0) && (Args[0] == "--dump-early-passes"))
        {
            String AgdPath = (Args.Length > 1)
                ? Path.GetFullPath(Args[1])
                : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd"));
            DumpEarlyPasses(AgdPath);
            return;
        }

        String Fixture = Args.Length > 0
            ? Args[0]
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
            Constants.HALF_SCALE,
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
        Console.WriteLine(String.Format("moved_bcy={0:F2} haul_m={1:F1} hours={2:F2}", MovedBcy, Time.HaulingDistM, Time.TotalTimeS / Constants.SECONDS_PER_HOUR));
        Console.WriteLine(String.Format("total: {0:F2}s", Sw.Elapsed.TotalSeconds));
    }

    /// <summary>
    /// Plans TheShop2 (or another AGD) and writes the sequenced route-2 single-pair PNG.
    /// </summary>
    /// <param name="AgdPath">Path to the AGD fixture.</param>
    /// <param name="OutPath">Destination PNG path.</param>
    /// <summary>
    /// Writes a single-pass guidance PNG for TheShop2 (1-based pass display via 0-based index).
    /// </summary>
    /// <param name="AgdPath">Path to the AGD fixture.</param>
    /// <param name="OutPath">Output PNG path.</param>
    /// <param name="PassIndex">0-based index into the sequenced route.</param>
    private static void WriteRoutePng
        (
        String AgdPath,
        String OutPath,
        Int32 PassIndex
        )
    {
        // Match the Python single-route render: default grid/blade flow-aligned plan, one ordered pass.
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

        if ((PassIndex < 0) || (PassIndex >= FullRoute.OrderedPasses.Count))
        {
            throw new ArgumentOutOfRangeException(
                nameof(PassIndex),
                PassIndex,
                String.Format(
                    CultureInfo.InvariantCulture,
                    "pass index {0} out of range (n={1})",
                    PassIndex,
                    FullRoute.OrderedPasses.Count));
        }

        Pass PassN = FullRoute.OrderedPasses[PassIndex];
        Route Mini = new Route(new List<Int32> { 0 }, new List<Pass> { PassN }, 0.0);
        GuidanceResult Guidance = Swaths.BuildGuidance(Grid, Field, Mini, Constants.DEFAULT_BLADE_WIDTH_MM, Heading);
        List<GuidancePoint> Pts = Guidance.Points.Where(Point => Point.PassNo == 1).ToList();
        Double BladeM = Constants.DEFAULT_BLADE_WIDTH_MM / Constants.MILLIMETERS_PER_METER;
        Byte[] Png = Render.RenderSinglePairPng(Grid, Pts, BladeM, pixelsPerBin: 40, padCells: 6);

        String? Dir = Path.GetDirectoryName(OutPath);
        if (!String.IsNullOrEmpty(Dir))
        {
            Directory.CreateDirectory(Dir);
        }

        File.WriteAllBytes(OutPath, Png);
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "wrote {0} ({1} bytes, {2} waypoints, route={3}, haul_m={4:F1})",
            OutPath,
            Png.Length,
            Pts.Count,
            PassIndex + 1,
            PassN.HaulDistanceM));
    }

    /// <summary>
    /// Prints JSON fingerprints for the first generated and ordered passes (parity debugging).
    /// </summary>
    /// <param name="AgdPath">Path to the AGD fixture.</param>
    private static void DumpEarlyPasses
        (
        String AgdPath
        )
    {
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

        // Snapshot generation fingerprints before knot reprofile mutates shared Pass instances.
        String FpPath = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(AgdPath)!, "..", "assets", "cs_pass_fps.txt"));
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

        String CellPath = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(AgdPath)!, "..", "assets", "cs_cells.txt"));
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
    /// <param name="Pass">Pass to fingerprint.</param>
    /// <returns>12-character hex digest prefix.</returns>
    private static String PassFingerprint
        (
        Pass Pass
        )
    {
        using System.Security.Cryptography.MD5 Md5 = System.Security.Cryptography.MD5.Create();
        List<Byte> Bytes = new List<Byte>();
        Bytes.AddRange(BitConverter.GetBytes(Pass.HaulDistanceM));
        Bytes.AddRange(BitConverter.GetBytes(Pass.LoadBcy));
        Bytes.AddRange(BitConverter.GetBytes(Pass.CutCells.Count));
        Int32 TakeFront = Math.Min(5, Pass.CutCells.Count);
        for (Int32 Iter = 0; Iter < TakeFront; Iter += 1)
        {
            Bytes.AddRange(BitConverter.GetBytes(Pass.CutCells[Iter].Cell.Bx));
            Bytes.AddRange(BitConverter.GetBytes(Pass.CutCells[Iter].Cell.By));
            Bytes.AddRange(BitConverter.GetBytes(Pass.CutCells[Iter].DepthM));
        }

        Int32 TailStart = Math.Max(0, Pass.CutCells.Count - 3);
        for (Int32 Iter = TailStart; Iter < Pass.CutCells.Count; Iter += 1)
        {
            Bytes.AddRange(BitConverter.GetBytes(Pass.CutCells[Iter].Cell.Bx));
            Bytes.AddRange(BitConverter.GetBytes(Pass.CutCells[Iter].Cell.By));
            Bytes.AddRange(BitConverter.GetBytes(Pass.CutCells[Iter].DepthM));
        }

        Byte[] Hash = Md5.ComputeHash(Bytes.ToArray());
        return Convert.ToHexString(Hash).Substring(0, 12).ToLowerInvariant();
    }

    /// <summary>
    /// Writes one pass fingerprint line as compact JSON for cross-language comparison.
    /// </summary>
    /// <param name="Pass">Pass to summarize.</param>
    /// <param name="Label">Fingerprint label.</param>
    private static void PrintPassFingerprint
        (
        Pass Pass,
        String Label
        )
    {
        Int32 Uniq = Pass.CutCells.Select(Entry => (Entry.Cell.Bx, Entry.Cell.By)).Distinct().Count();
        String First = String.Join(",", Pass.CutCells.Take(8).Select(Entry =>
            String.Format(CultureInfo.InvariantCulture, "[{0},{1},{2:F6}]", Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM)));
        String Last = String.Join(",", Pass.CutCells.Skip(Math.Max(0, Pass.CutCells.Count - 5)).Select(Entry =>
            String.Format(CultureInfo.InvariantCulture, "[{0},{1},{2:F6}]", Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM)));
        String CutH = Pass.CutHeadingDeg.HasValue
            ? Pass.CutHeadingDeg.Value.ToString("F3", CultureInfo.InvariantCulture)
            : "null";
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "{{\"label\":\"{0}\",\"ncut\":{1},\"nfill\":{2},\"uniq_cut\":{3},\"haul_m\":{4:F3},\"load_bcy\":{5:F4},\"cut_h\":{6},\"cut_start\":[{7:F3},{8:F3}],\"cut_end\":[{9:F3},{10:F3}],\"fill_start\":[{11:F3},{12:F3}],\"first_cut\":[{13}],\"last_cut\":[{14}]}}",
            Label,
            Pass.CutCells.Count,
            Pass.FillCells.Count,
            Uniq,
            Pass.HaulDistanceM,
            Pass.LoadBcy,
            CutH,
            Pass.CutStartXy.X,
            Pass.CutStartXy.Y,
            Pass.CutEndXy.X,
            Pass.CutEndXy.Y,
            Pass.FillStartXy.X,
            Pass.FillStartXy.Y,
            First,
            Last));
    }
}
