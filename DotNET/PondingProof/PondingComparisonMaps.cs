using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

using HaulBuilder;

/// <summary>
/// Builds labelled side-by-side ponding comparison maps from a proof output
/// directory or by re-running the production plan when routes are unavailable.
/// </summary>
internal static class PondingComparisonMaps
{
    /// <summary>Generates side-by-side comparison PNGs for one proof run.</summary>
    /// <param name="Args">
    /// AGD path, output directory, optional objective mode (average|peak).
    /// </param>
    /// <returns>Zero when maps are written.</returns>
    static public Int32 Run(String[] Args)
    {
        String AgdPath = Args.Length > 0
            ? Path.GetFullPath(Args[0])
            : Path.GetFullPath(Path.Combine(
                AppContext.BaseDirectory, "..", "..", "..", "..",
                "TheShop2_2ft.agd"));
        String OutputDirectory = Args.Length > 1
            ? Path.GetFullPath(Args[1])
            : Path.GetFullPath(Path.Combine(
                AppContext.BaseDirectory, "..", "..", "..", "..", "..",
                "assets", "ponding_proof_avg_depth_20260721"));
        PondingObjectiveMode ObjectiveMode = Args.Length > 2
            && String.Equals(Args[2], "peak",
                StringComparison.OrdinalIgnoreCase)
            ? PondingObjectiveMode.PeakVolume
            : PondingObjectiveMode.AverageDepth;
        Directory.CreateDirectory(OutputDirectory);

        (Grid Grid, PassResult PassResult, Route AuditRoute, Route OptimizedRoute,
            PondingRunResult Audit, PondingRunResult Optimized, _, _, _) =
            FastPondingProof.BuildComparisonState(
                AgdPath, 25, 5, 0.0, ObjectiveMode);

        PondingOutlets Outlets = Audit.Outlets
            ?? Optimized.Outlets
            ?? Ponding.BuildOutlets(Grid, new PondingOptions());
        InProcessPondingAnalyser Analyser = new();
        const Int32 PixelsPerCell = 8;
        PondingAuditResult Before = Audit.FinalAudit!;
        PondingAuditResult After = Optimized.FinalAudit!;
        Int32 SharedPass = Math.Max(
            Before.WorstCheckpoint.PassIndex,
            After.WorstCheckpoint.PassIndex);

        WritePair(
            OutputDirectory,
            "comparison_worst_checkpoints.png",
            Grid,
            Outlets,
            Analyser,
            AuditRoute.OrderedPasses,
            Before.WorstCheckpoint.PassIndex,
            OptimizedRoute.OrderedPasses,
            After.WorstCheckpoint.PassIndex,
            PixelsPerCell,
            useSharedScale: false);

        WritePair(
            OutputDirectory,
            $"comparison_pass_{SharedPass:D4}.png",
            Grid,
            Outlets,
            Analyser,
            AuditRoute.OrderedPasses,
            SharedPass,
            OptimizedRoute.OrderedPasses,
            SharedPass,
            PixelsPerCell,
            useSharedScale: true);

        WritePair(
            OutputDirectory,
            "comparison_midjob_pass_1125.png",
            Grid,
            Outlets,
            Analyser,
            AuditRoute.OrderedPasses,
            1125,
            OptimizedRoute.OrderedPasses,
            1125,
            PixelsPerCell,
            useSharedScale: true);

        StringBuilder Legend = new();
        Legend.AppendLine("Ponding comparison maps");
        Legend.AppendLine("Red stripe = baseline route, green stripe = optimized route");
        Legend.AppendLine("Banner title = pass index + avg pond depth in mm (e.g. 1125118)");
        Legend.AppendLine("Blue = closed depression ponding");
        Legend.AppendLine("Green = declared outlet cells");
        Legend.AppendLine();
        Legend.AppendFormat(CultureInfo.InvariantCulture,
            "comparison_worst_checkpoints.png: baseline pass {0} vs optimized pass {1}\n",
            Before.WorstCheckpoint.PassIndex,
            After.WorstCheckpoint.PassIndex);
        Legend.AppendFormat(CultureInfo.InvariantCulture,
            "comparison_pass_{0:D4}.png: same pass index, shared elevation scale\n",
            SharedPass);
        Legend.AppendLine(
            "comparison_midjob_pass_1125.png: baseline worst-case pass, shared scale");
        Legend.AppendLine();
        Legend.AppendFormat(CultureInfo.InvariantCulture,
            "baseline mean_avg_depth_m={0:F6} penalty={1:F6}\n",
            Before.MeanAveragePondDepthM,
            Before.Penalty);
        Legend.AppendFormat(CultureInfo.InvariantCulture,
            "optimized mean_avg_depth_m={0:F6} penalty={1:F6}\n",
            After.MeanAveragePondDepthM,
            After.Penalty);
        File.WriteAllText(
            Path.Combine(OutputDirectory, "comparison_maps.txt"),
            Legend.ToString());
        Console.WriteLine("Wrote comparison maps to {0}", OutputDirectory);
        return 0;
    }

    /// <summary>Writes one labelled before/after comparison PNG.</summary>
    /// <param name="OutputDirectory">Destination directory.</param>
    /// <param name="FileName">Output PNG file name.</param>
    /// <param name="Grid">Planning grid.</param>
    /// <param name="Outlets">Outlet mask.</param>
    /// <param name="Analyser">Ponding scorer.</param>
    /// <param name="BeforePasses">Baseline route passes.</param>
    /// <param name="BeforePassIndex">Baseline checkpoint prefix length.</param>
    /// <param name="AfterPasses">Optimised route passes.</param>
    /// <param name="AfterPassIndex">Optimised checkpoint prefix length.</param>
    /// <param name="PixelsPerCell">Raster scale.</param>
    /// <param name="UseSharedScale">Whether both panels share one elevation colour scale.</param>
    static private void WritePair(
        String OutputDirectory,
        String FileName,
        Grid Grid,
        PondingOutlets Outlets,
        InProcessPondingAnalyser Analyser,
        IReadOnlyList<Pass> BeforePasses,
        Int32 BeforePassIndex,
        IReadOnlyList<Pass> AfterPasses,
        Int32 AfterPassIndex,
        Int32 PixelsPerCell,
        Boolean UseSharedScale)
    {
        Double[,] BeforeSurface = Ponding.ReplaySurface(
            Grid, BeforePasses, BeforePassIndex);
        Double[,] AfterSurface = Ponding.ReplaySurface(
            Grid, AfterPasses, AfterPassIndex);
        PondingMetrics BeforeMetrics = Analyser.Score(
            BeforeSurface, Grid.Valid, Outlets.Mask, Grid.AreaM2);
        PondingMetrics AfterMetrics = Analyser.Score(
            AfterSurface, Grid.Valid, Outlets.Mask, Grid.AreaM2);
        (Double MinM, Double MaxM) Range = UseSharedScale
            ? Ponding.SharedElevationRange(Grid, BeforeSurface, AfterSurface)
            : (Double.NaN, Double.NaN);
        Double? SharedMin = UseSharedScale ? Range.MinM : null;
        Double? SharedMax = UseSharedScale ? Range.MaxM : null;
        PondingCheckpoint BeforeCheckpoint = new(
            BeforePassIndex, BeforeMetrics, 0.0, BeforeSurface);
        PondingCheckpoint AfterCheckpoint = new(
            AfterPassIndex, AfterMetrics, 0.0, AfterSurface);
        // Banner digits only support 0-9, so encode pass index and depth in mm.
        String BeforeTitle = String.Format(CultureInfo.InvariantCulture,
            "{0}{1:D3}",
            BeforePassIndex,
            (Int32)Math.Round(BeforeMetrics.AveragePondDepthM * 1000.0));
        String AfterTitle = String.Format(CultureInfo.InvariantCulture,
            "{0}{1:D3}",
            AfterPassIndex,
            (Int32)Math.Round(AfterMetrics.AveragePondDepthM * 1000.0));
        Byte[] Composite = Ponding.RenderSideBySideComparisonPng(
            new[]
            {
                new Ponding.PondingComparisonPanel(
                    BeforeTitle,
                    Ponding.RenderCheckpointPng(
                        Grid, Outlets, BeforeCheckpoint, PixelsPerCell,
                        SharedMin, SharedMax)),
                new Ponding.PondingComparisonPanel(
                    AfterTitle,
                    Ponding.RenderCheckpointPng(
                        Grid, Outlets, AfterCheckpoint, PixelsPerCell,
                        SharedMin, SharedMax)),
            });
        File.WriteAllBytes(Path.Combine(OutputDirectory, FileName), Composite);
    }
}
