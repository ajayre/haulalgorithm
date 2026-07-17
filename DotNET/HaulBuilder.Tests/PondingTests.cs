using HaulBuilder;

namespace HaulBuilder.Tests
{
    public sealed class PondingTests
    {
        [Fact]
        public void InProcessAnalyser_FlatPlaneHasNoDepressions()
        {
            Double[,] Surface = new Double[4, 4];
            Boolean[,] Valid = AllValid(4, 4);
            Boolean[,] Outlets = BoundaryMask(4, 4);

            PondingMetrics Metrics = new InProcessPondingAnalyser().Score(
                Surface, Valid, Outlets, 1.0);

            Assert.True(Metrics.AnalyserOk);
            Assert.Equal(0.0, Metrics.ClosedDepressionVolumeBcy, 9);
            Assert.Equal(0.0, Metrics.MaxPondDepthM, 9);
            Assert.Equal(0, Metrics.ClosedDepressionCount);
        }

        [Fact]
        public void InProcessAnalyser_ScoresSinglePitToSpillElevation()
        {
            Double[,] Surface = Filled(5, 5, 1.0);
            Surface[2, 2] = 0.0;

            PondingMetrics Metrics = new InProcessPondingAnalyser().Score(
                Surface, AllValid(5, 5), BoundaryMask(5, 5), 1.0);

            Assert.True(Metrics.AnalyserOk);
            Assert.Equal(Constants.M3_TO_CY,
                Metrics.ClosedDepressionVolumeBcy, 8);
            Assert.Equal(1.0, Metrics.MaxPondDepthM, 9);
            Assert.Equal(1, Metrics.ClosedDepressionCount);
        }

        [Fact]
        public void ReplaySurface_UsesExactStoredDepthsWithoutMutatingGrid()
        {
            Grid Grid = MakeGrid(Filled(3, 3, 10.0));
            Pass Pass = MakePass(
                new[] { ((1, 1), 0.25) },
                new[] { ((2, 1), 0.125) });

            Double[,] Surface = Ponding.ReplaySurface(Grid,
                new List<Pass> { Pass }, 1);

            Assert.Equal(9.75, Surface[1, 1], 9);
            Assert.Equal(10.125, Surface[1, 2], 9);
            Assert.Equal(10.0, Grid.Existing[1, 1], 9);
        }

        [Fact]
        public void AuditOnly_DoesNotModifyRouteAndEmitsEveryPassByDefault()
        {
            Grid Grid = MakeGrid(Filled(4, 4, 0.0));
            Pass First = MakePass(new[] { ((1, 1), 0.1) }, Array.Empty<((Int32, Int32), Double)>());
            Pass Second = MakePass(Array.Empty<((Int32, Int32), Double)>(), new[] { ((1, 1), 0.1) });
            Route Route = new(new List<Int32> { 1, 0 },
                new List<Pass> { Second, First }, 0.0);
            PondingOptions Options = new()
            {
                Audit = true,
                TargetSpillOutlets = false,
                IncludeMap = false,
            };

            PondingRunResult Result = Ponding.Run(Grid,
                new List<Pass> { First, Second }, Route, Options,
                13.0, 4.0, 50.0);

            Assert.Same(Second, Route.OrderedPasses[0]);
            Assert.Same(First, Route.OrderedPasses[1]);
            Assert.Equal(3, Result.FinalAudit!.Checkpoints.Count);
        }

        [Fact]
        public void Optimize_MovesOutletCutBeforeTransientDam()
        {
            Double[,] Existing = Filled(5, 5, 1.0);
            for (Int32 I = 0; I < 5; I += 1)
            {
                Existing[0, I] = 0.0;
                Existing[4, I] = 0.0;
                Existing[I, 0] = 0.0;
                Existing[I, 4] = 0.0;
            }
            Existing[2, 2] = 0.0;
            Existing[1, 2] = 0.0;
            Grid Grid = MakeGrid(Existing);
            Pass DamFill = MakePass(
                Array.Empty<((Int32, Int32), Double)>(),
                new[] { ((2, 1), 1.0) }, X: 0.0);
            Pass OutletCut = MakePass(
                new[] { ((3, 1), 1.0) },
                Array.Empty<((Int32, Int32), Double)>(), X: 10.0);
            Route Route = new(new List<Int32> { 0, 1 },
                new List<Pass> { DamFill, OutletCut }, 0.0);
            PondingOptions Options = new()
            {
                Optimize = true,
                TargetSpillOutlets = false,
                CheckpointInterval = 1,
                Weight = 0.0,
                IncludeMap = false,
            };

            PondingRunResult Result = Ponding.Run(Grid,
                new List<Pass> { DamFill, OutletCut }, Route, Options,
                13.0, 4.0, 50.0);

            Assert.True(Result.Before!.PeakVolumeBcy > 0.0);
            Assert.Equal(0.0, Result.After!.PeakVolumeBcy, 8);
            Assert.Same(OutletCut, Route.OrderedPasses[0]);
            Assert.Equal(1, Result.RelocationsAccepted);
        }

        [Fact]
        public void Optimize_PreservesSharedCellLayerOrder()
        {
            Grid Grid = MakeGrid(Filled(4, 4, 1.0));
            Pass Layer1 = MakePass(new[] { ((1, 1), 0.1) },
                Array.Empty<((Int32, Int32), Double)>(), X: 10.0);
            Pass Layer2 = MakePass(new[] { ((1, 1), 0.1) },
                Array.Empty<((Int32, Int32), Double)>(), X: 0.0);
            Route Route = new(new List<Int32> { 1, 0 },
                new List<Pass> { Layer2, Layer1 }, 0.0);
            PondingOptions Options = new()
            {
                Optimize = true,
                TargetSpillOutlets = false,
                CheckpointInterval = 1,
                IncludeMap = false,
            };

            Ponding.Run(Grid, new List<Pass> { Layer1, Layer2 }, Route,
                Options, 13.0, 4.0, 50.0);

            Assert.Same(Layer1, Route.OrderedPasses[0]);
            Assert.Same(Layer2, Route.OrderedPasses[1]);
            Assert.Equal(1, Layer1.PondingLayerMin);
            Assert.Equal(2, Layer2.PondingLayerMin);
        }

        [Fact]
        public void ExplicitMissingWhiteBoxToolsPath_FailsBeforeRouteMutation()
        {
            Grid Grid = MakeGrid(Filled(3, 3, 0.0));
            Pass Pass = MakePass(Array.Empty<((Int32, Int32), Double)>(),
                Array.Empty<((Int32, Int32), Double)>());
            Route Route = new(new List<Int32> { 0 },
                new List<Pass> { Pass }, 0.0);
            PondingOptions Options = new()
            {
                Optimize = true,
                TargetSpillOutlets = false,
                WhiteBoxToolsPath = Path.Combine(Path.GetTempPath(),
                    $"missing-whitebox-{Guid.NewGuid():N}"),
            };

            InvalidOperationException Error = Assert.Throws<InvalidOperationException>(() =>
                Ponding.Run(Grid, new List<Pass> { Pass }, Route, Options,
                    13.0, 4.0, 50.0));

            Assert.Contains("WhiteBoxTools executable was not found", Error.Message);
            Assert.Same(Pass, Route.OrderedPasses[0]);
        }

        static private Grid MakeGrid(Double[,] Existing)
        {
            Int32 Height = Existing.GetLength(0);
            Int32 Width = Existing.GetLength(1);
            return new Grid(1.0, Width, Height, 500000.0, 3700000.0,
                12, true, 33.0, -111.0, (Double[,])Existing.Clone(),
                (Double[,])Existing.Clone(), AllValid(Height, Width),
                1.3, 1.2, 61.0,
                new List<List<(Double X, Double Y)>>());
        }

        static private Pass MakePass(
            IEnumerable<((Int32 X, Int32 Y) Cell, Double Depth)> Cut,
            IEnumerable<((Int32 X, Int32 Y) Cell, Double Depth)> Fill,
            Double X = 0.0)
        {
            return new Pass(
                Cut.Select(C => ((C.Cell.X, C.Cell.Y), C.Depth)).ToList(),
                Fill.Select(C => ((C.Cell.X, C.Cell.Y), C.Depth)).ToList(),
                1.0, 0.0, (X, 0.0), (X + 1.0, 0.0),
                (X + 2.0, 0.0), (X + 3.0, 0.0));
        }

        static private Double[,] Filled(Int32 Height, Int32 Width, Double Value)
        {
            Double[,] Out = new Double[Height, Width];
            for (Int32 Y = 0; Y < Height; Y += 1)
            for (Int32 X = 0; X < Width; X += 1) Out[Y, X] = Value;
            return Out;
        }

        static private Boolean[,] AllValid(Int32 Height, Int32 Width)
        {
            Boolean[,] Out = new Boolean[Height, Width];
            for (Int32 Y = 0; Y < Height; Y += 1)
            for (Int32 X = 0; X < Width; X += 1) Out[Y, X] = true;
            return Out;
        }

        static private Boolean[,] BoundaryMask(Int32 Height, Int32 Width)
        {
            Boolean[,] Out = new Boolean[Height, Width];
            for (Int32 Y = 0; Y < Height; Y += 1)
            for (Int32 X = 0; X < Width; X += 1)
                Out[Y, X] = X == 0 || Y == 0
                    || X == Width - 1 || Y == Height - 1;
            return Out;
        }
    }
}
