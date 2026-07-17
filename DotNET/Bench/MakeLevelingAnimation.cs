using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using HaulBuilder;

internal static class MakeLevelingAnimation
{
    internal static Int32 Run
        (
        String[] args
        )
    {
        String Fixture = args.Length > 0
            ? Path.GetFullPath(args[0])
            : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "TheShop2_2ft.agd"));
        Double GridFt = args.Length > 1 ? Double.Parse(args[1], CultureInfo.InvariantCulture) : 7.5;
        Int32 Ppb = args.Length > 2 ? Int32.Parse(args[2], CultureInfo.InvariantCulture) : 4;
        String OutRoot = args.Length > 3
            ? Path.GetFullPath(args[3])
            : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "out"));
        String OutGif = Path.Combine(OutRoot, "TheShop2_leveling.gif");
        String OutFrames = Path.Combine(OutRoot, "TheShop2_leveling_frames");

        if (!File.Exists(Fixture))
        {
            Console.Error.WriteLine(String.Format(CultureInfo.InvariantCulture, "AGD not found: {0}", Fixture));
            return 1;
        }

        Directory.CreateDirectory(OutRoot);
        Directory.CreateDirectory(OutFrames);

        Console.WriteLine(String.Format(CultureInfo.InvariantCulture, "building plan: {0}  grid={1} ft  ppb={2}", Fixture, GridFt, Ppb));
        Stopwatch Sw = Stopwatch.StartNew();
        HaulBuilderEngine Engine = new HaulBuilderEngine();
        HaulBuilderOptions Options = new HaulBuilderOptions
        {
            AgdPath = Fixture,
            GridSizeFt = GridFt,
            ZoneCount = 0,
            PixelsPerCell = Ppb,
        };
        HaulBuilderResult Result = Engine.Run(Options);
        if ((Result.Route is null) || (Result.Grid is null) || (Result.PassResult is null))
        {
            Console.Error.WriteLine("Planning failed: missing route/grid/pass result.");
            return 1;
        }

        Int32 PassCount = Result.PassResult.Passes.Count;
        Console.WriteLine(String.Format(CultureInfo.InvariantCulture, "{0} passes; rendering split-view animation", PassCount));
        Animate.SplitLevelingOptions AnimOptions = new Animate.SplitLevelingOptions
        {
            PixelsPerBin = Ppb,
            Title = "The Shop 2 - land leveling by pass (route order)",
        };
        Animate.AnimationPayload Payload = Animate.AnimateLevelingSplitView(Result.Grid, Result.Route.OrderedPasses, AnimOptions);
        for (Int32 Iter = 0; Iter < Payload.FramePngBytes.Count; Iter += 1)
        {
            File.WriteAllBytes(
                Path.Combine(OutFrames, String.Format(CultureInfo.InvariantCulture, "frame_{0:D5}.png", Iter)),
                Payload.FramePngBytes[Iter]);
        }

        if (Payload.GifBytes is null)
        {
            Console.Error.WriteLine("No GIF bytes produced.");
            return 1;
        }

        File.WriteAllBytes(OutGif, Payload.GifBytes);
        Console.WriteLine(String.Format(
            CultureInfo.InvariantCulture,
            "wrote {0} frames -> {1}/  and  {2}  ({3:F1}s)",
            Payload.FramePngBytes.Count,
            OutFrames,
            OutGif,
            Sw.Elapsed.TotalSeconds));
        return 0;
    }
}
