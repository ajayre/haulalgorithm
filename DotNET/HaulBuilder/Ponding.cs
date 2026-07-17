using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.Formats.Png;
using SixLabors.ImageSharp.PixelFormats;

#pragma warning disable CS1591

namespace HaulBuilder
{
    /// <summary>
    /// Options for the optional ponding-aware post-routing stage.
    /// The in-process analyser implements the fill-until-spill equivalent allowed
    /// by PONDING-SPEC 7.1.  The documented WhiteBoxTools reference chain is
    /// breach_depressions_least_cost(flat_increment=0.001m, fill=false), sink,
    /// max_depth_in_watershed, then depth * cell-area volume integration.
    /// </summary>
    public sealed class PondingOptions
    {
        /// <summary>Initializes specification defaults; both master switches remain off.</summary>
        public PondingOptions()
        {
            TargetSpillOutlets = true;
            SpillToleranceFt = 0.1;
            Weight = 3600.0;
            WarnDepthFt = 0.5;
            WarnVolumeBcy = 0.0;
            DesignRainfallIn = 1.0;
            IncludeMap = true;
            MapPixelsPerCell = 8;
            OutletSpecs = new List<String>();
            OutletPolygonSpecs = new List<String>();
        }

        public Boolean Optimize { get; set; }
        public Boolean Audit { get; set; }
        public Boolean NoImplicitAudit { get; set; }
        public Boolean TargetSpillOutlets { get; set; }
        public Double SpillToleranceFt { get; set; }
        public Double Weight { get; set; }
        public Int32? CheckpointInterval { get; set; }
        public String? WhiteBoxToolsPath { get; set; }
        public String? WhiteBoxToolsWorkDirectory { get; set; }
        public Double WarnDepthFt { get; set; }
        public Double WarnVolumeBcy { get; set; }
        public Double DesignRainfallIn { get; set; }
        public Boolean FailOnWarn { get; set; }
        public Boolean IncludeMap { get; set; }
        public Int32 MapPixelsPerCell { get; set; }
        public List<String> OutletSpecs { get; }
        public List<String> OutletPolygonSpecs { get; }
    }

    [Flags]
    public enum PondingOutletSource
    {
        None = 0,
        Boundary = 1,
        Culvert = 2,
        TargetSpill = 4,
    }

    /// <summary>Outlet mask and provenance derived once from the planning grid.</summary>
    public sealed class PondingOutlets
    {
        public PondingOutlets(Boolean[,] Mask, PondingOutletSource[,] Source)
        {
            this.Mask = Mask;
            this.Source = Source;
            Count = Mask.Cast<Boolean>().Count(Value => Value);
        }

        public Boolean[,] Mask { get; }
        public PondingOutletSource[,] Source { get; }
        public Int32 Count { get; }
    }

    /// <summary>Fill-to-spill metrics for one intermediate surface.</summary>
    public sealed class PondingMetrics
    {
        public PondingMetrics(
            Double ClosedDepressionVolumeBcy,
            Double MaxPondDepthM,
            Int32 ClosedDepressionCount,
            Boolean AnalyserOk,
            Double[,]? DepressionDepthM = null,
            Double PondingCatchmentAreaM2 = 0.0)
        {
            this.ClosedDepressionVolumeBcy = ClosedDepressionVolumeBcy;
            this.MaxPondDepthM = MaxPondDepthM;
            this.ClosedDepressionCount = ClosedDepressionCount;
            this.AnalyserOk = AnalyserOk;
            this.DepressionDepthM = DepressionDepthM;
            this.PondingCatchmentAreaM2 = PondingCatchmentAreaM2;
        }

        public Double ClosedDepressionVolumeBcy { get; }
        public Double MaxPondDepthM { get; }
        public Int32 ClosedDepressionCount { get; }
        public Boolean AnalyserOk { get; }
        public Double[,]? DepressionDepthM { get; }
        public Double PondingCatchmentAreaM2 { get; }
    }

    public interface IPondingAnalyser
    {
        PondingMetrics Score(Double[,] Surface, Boolean[,] ValidMask,
            Boolean[,] OutletMask, Double CellAreaM2);
    }

    /// <summary>
    /// Deterministic priority-flood scorer. Only declared outlet cells seed the
    /// flood, so invalid cells are walls rather than implicit drainage outlets.
    /// </summary>
    public sealed class InProcessPondingAnalyser : IPondingAnalyser
    {
        public PondingMetrics Score(Double[,] Surface, Boolean[,] ValidMask,
            Boolean[,] OutletMask, Double CellAreaM2)
        {
            if (Surface == null || ValidMask == null || OutletMask == null)
                throw new ArgumentNullException(nameof(Surface));
            Int32 Height = Surface.GetLength(0);
            Int32 Width = Surface.GetLength(1);
            if (ValidMask.GetLength(0) != Height || ValidMask.GetLength(1) != Width
                || OutletMask.GetLength(0) != Height || OutletMask.GetLength(1) != Width)
                throw new ArgumentException("Ponding raster dimensions must match.");

            Double[,] Filled = new Double[Height, Width];
            Boolean[,] Seen = new Boolean[Height, Width];
            PriorityQueue<(Int32 X, Int32 Y), Double> Queue = new();
            Int32 OutletCount = 0;
            for (Int32 Y = 0; Y < Height; Y += 1)
            for (Int32 X = 0; X < Width; X += 1)
            {
                Filled[Y, X] = Double.NaN;
                if (!ValidMask[Y, X] || !OutletMask[Y, X]) continue;
                Double Z = Surface[Y, X];
                if (!Double.IsFinite(Z)) continue;
                Seen[Y, X] = true;
                Filled[Y, X] = Z;
                Queue.Enqueue((X, Y), Z);
                OutletCount += 1;
            }
            if (OutletCount == 0)
                return new PondingMetrics(0.0, 0.0, 0, false,
                    new Double[Height, Width]);

            while (Queue.TryDequeue(out (Int32 X, Int32 Y) Cell,
                       out Double SpillElevation))
            {
                for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                {
                    if (Dx == 0 && Dy == 0) continue;
                    Int32 Nx = Cell.X + Dx;
                    Int32 Ny = Cell.Y + Dy;
                    if (Nx < 0 || Nx >= Width || Ny < 0 || Ny >= Height
                        || Seen[Ny, Nx] || !ValidMask[Ny, Nx]) continue;
                    Double Z = Surface[Ny, Nx];
                    if (!Double.IsFinite(Z)) continue;
                    Double FilledZ = Math.Max(Z, SpillElevation);
                    Seen[Ny, Nx] = true;
                    Filled[Ny, Nx] = FilledZ;
                    Queue.Enqueue((Nx, Ny), FilledZ);
                }
            }

            Double[,] Depth = new Double[Height, Width];
            Boolean[,] Pond = new Boolean[Height, Width];
            Double VolumeM3 = 0.0;
            Double MaxDepth = 0.0;
            Int32 PondCells = 0;
            Boolean AllReached = true;
            for (Int32 Y = 0; Y < Height; Y += 1)
            for (Int32 X = 0; X < Width; X += 1)
            {
                if (!ValidMask[Y, X]) continue;
                if (!Seen[Y, X]) { AllReached = false; continue; }
                Double D = Math.Max(0.0, Filled[Y, X] - Surface[Y, X]);
                if (D <= 1e-9) continue;
                Depth[Y, X] = D;
                Pond[Y, X] = true;
                VolumeM3 += D * CellAreaM2;
                MaxDepth = Math.Max(MaxDepth, D);
                PondCells += 1;
            }
            Int32 Components = CountComponents(Pond);
            return new PondingMetrics(
                VolumeM3 * Constants.M3_TO_CY,
                MaxDepth,
                Components,
                AllReached,
                Depth,
                PondCells * CellAreaM2);
        }

        static private Int32 CountComponents(Boolean[,] Mask)
        {
            Int32 Height = Mask.GetLength(0);
            Int32 Width = Mask.GetLength(1);
            Boolean[,] Seen = new Boolean[Height, Width];
            Int32 Count = 0;
            for (Int32 Y = 0; Y < Height; Y += 1)
            for (Int32 X = 0; X < Width; X += 1)
            {
                if (!Mask[Y, X] || Seen[Y, X]) continue;
                Count += 1;
                Queue<(Int32 X, Int32 Y)> Queue = new();
                Queue.Enqueue((X, Y));
                Seen[Y, X] = true;
                while (Queue.Count > 0)
                {
                    (Int32 Cx, Int32 Cy) = Queue.Dequeue();
                    for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                    for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                    {
                        if (Dx == 0 && Dy == 0) continue;
                        Int32 Nx = Cx + Dx;
                        Int32 Ny = Cy + Dy;
                        if (Nx < 0 || Nx >= Width || Ny < 0 || Ny >= Height
                            || Seen[Ny, Nx] || !Mask[Ny, Nx]) continue;
                        Seen[Ny, Nx] = true;
                        Queue.Enqueue((Nx, Ny));
                    }
                }
            }
            return Count;
        }
    }

    public sealed class PondingCheckpoint
    {
        public PondingCheckpoint(Int32 PassIndex, PondingMetrics Metrics,
            Double TransitionCostCumulativeS, Double[,]? Surface = null)
        {
            this.PassIndex = PassIndex;
            this.Metrics = Metrics;
            this.TransitionCostCumulativeS = TransitionCostCumulativeS;
            this.Surface = Surface;
        }
        public Int32 PassIndex { get; }
        public PondingMetrics Metrics { get; }
        public Double TransitionCostCumulativeS { get; }
        public Double[,]? Surface { get; internal set; }
    }

    public sealed class PondingAuditResult
    {
        public PondingAuditResult(IReadOnlyList<PondingCheckpoint> Checkpoints)
        {
            this.Checkpoints = Checkpoints;
            WorstCheckpoint = Checkpoints.OrderByDescending(C =>
                    C.Metrics.ClosedDepressionVolumeBcy)
                .ThenByDescending(C => C.Metrics.MaxPondDepthM)
                .First();
            AnalyserFailures = Checkpoints.Count(C => !C.Metrics.AnalyserOk);
        }
        public IReadOnlyList<PondingCheckpoint> Checkpoints { get; }
        public PondingCheckpoint WorstCheckpoint { get; }
        public Double PeakVolumeBcy => WorstCheckpoint.Metrics.ClosedDepressionVolumeBcy;
        public Int32 AnalyserFailures { get; }
    }

    public sealed class PondingRunResult
    {
        public PondingOutlets? Outlets { get; internal set; }
        public PondingAuditResult? Before { get; internal set; }
        public PondingAuditResult? After { get; internal set; }
        public Double TransitionCostBeforeS { get; internal set; }
        public Double TransitionCostAfterS { get; internal set; }
        public Int32 RelocationsAccepted { get; internal set; }
        public List<String> Warnings { get; } = new();
        public Boolean FailOnWarnTriggered { get; internal set; }
        public PondingAuditResult? FinalAudit => After ?? Before;
    }

    /// <summary>Outlet construction, surface replay, audit, and constrained local search.</summary>
    static public class Ponding
    {
        private sealed class PassComparer : IEqualityComparer<Pass>
        {
            public Boolean Equals(Pass? X, Pass? Y) => ReferenceEquals(X, Y);
            public Int32 GetHashCode(Pass Obj) => RuntimeHelpers.GetHashCode(Obj);
        }

        /// <summary>Builds boundary, target-spill, and manual culvert outlets.</summary>
        static public PondingOutlets BuildOutlets(Grid Grid, PondingOptions Options)
        {
            Int32 H = Grid.Height;
            Int32 W = Grid.Width;
            Boolean[,] Mask = new Boolean[H, W];
            PondingOutletSource[,] Source = new PondingOutletSource[H, W];
            for (Int32 Y = 0; Y < H; Y += 1)
            for (Int32 X = 0; X < W; X += 1)
            {
                if (!Grid.Valid[Y, X] || !IsPerimeter(Grid.Valid, X, Y)) continue;
                AddOutlet(Mask, Source, X, Y, PondingOutletSource.Boundary);
            }

            if (Options.TargetSpillOutlets)
            {
                Double Minimum = Double.PositiveInfinity;
                for (Int32 Y = 0; Y < H; Y += 1)
                for (Int32 X = 0; X < W; X += 1)
                    if (Mask[Y, X]) Minimum = Math.Min(Minimum, Grid.Target[Y, X]);
                Boolean[,] ReachesBoundary = ReverseDescendingReachability(Grid, Mask);
                Double ToleranceM = Options.SpillToleranceFt * Constants.FEET_TO_METERS;
                for (Int32 Y = 0; Y < H; Y += 1)
                for (Int32 X = 0; X < W; X += 1)
                    if (Grid.Valid[Y, X] && ReachesBoundary[Y, X]
                        && Grid.Target[Y, X] <= Minimum + ToleranceM)
                        AddOutlet(Mask, Source, X, Y,
                            PondingOutletSource.TargetSpill);
            }

            foreach (String Spec in Options.OutletSpecs)
            {
                (Double Lat, Double Lon) = ParsePoint(Spec);
                UtmCoord U = Agd.UtmFromLatlon(Lat, Lon);
                if (U.Zone != Grid.Zone || U.IsNorthern != Grid.IsNorthern)
                    throw new ArgumentException("Ponding outlet is outside the grid UTM zone.");
                (Int32 X, Int32 Y) = NearestValid(
                    Grid, U.Easting, U.Northing);
                AddOutlet(Mask, Source, X, Y, PondingOutletSource.Culvert);
            }

            foreach (String Spec in Options.OutletPolygonSpecs)
            {
                List<(Double X, Double Y)> Ring = ParseRing(Spec, Grid);
                for (Int32 Y = 0; Y < H; Y += 1)
                for (Int32 X = 0; X < W; X += 1)
                    if (Grid.Valid[Y, X] && CellIntersectsPolygon(Grid, X, Y, Ring))
                        AddOutlet(Mask, Source, X, Y,
                            PondingOutletSource.Culvert);
            }
            return new PondingOutlets(Mask, Source);
        }

        /// <summary>Replays exactly the stored cut and fill depths through prefix K.</summary>
        static public Double[,] ReplaySurface(Grid Grid,
            IReadOnlyList<Pass> OrderedPasses, Int32 K)
        {
            if (K < 0 || K > OrderedPasses.Count)
                throw new ArgumentOutOfRangeException(nameof(K));
            Double[,] Surface = (Double[,])Grid.Existing.Clone();
            for (Int32 Index = 0; Index < K; Index += 1)
                ApplyPass(Surface, OrderedPasses[Index]);
            for (Int32 Y = 0; Y < Grid.Height; Y += 1)
            for (Int32 X = 0; X < Grid.Width; X += 1)
                if (!Grid.Valid[Y, X]) Surface[Y, X] = Double.NaN;
            return Surface;
        }

        /// <summary>Scores a route incrementally at the requested checkpoints.</summary>
        static public PondingAuditResult AuditRoute(Grid Grid,
            IReadOnlyList<Pass> OrderedPasses, PondingOutlets Outlets,
            IPondingAnalyser Analyser, Int32 CheckpointInterval,
            Routing.TransitionCostEvaluator? TransitionEvaluator = null,
            Boolean RetainAllSurfaces = false,
            Dictionary<(UInt64 PrefixHash, Int32 K), PondingMetrics>? MetricsCache = null)
        {
            Dictionary<(UInt64 PrefixHash, Int32 K), PondingMetrics> Cache =
                MetricsCache ?? new();
            List<Int32> Checkpoints = BuildCheckpoints(
                OrderedPasses.Count, CheckpointInterval);
            HashSet<Int32> Wanted = Checkpoints.ToHashSet();
            List<PondingCheckpoint> Rows = new();
            Double[,] Surface = (Double[,])Grid.Existing.Clone();
            Double Transition = 0.0;
            PondingCheckpoint? RetainedWorst = null;
            for (Int32 K = 0; K <= OrderedPasses.Count; K += 1)
            {
                if (K > 0)
                {
                    ApplyPass(Surface, OrderedPasses[K - 1]);
                    if (K > 1 && TransitionEvaluator != null)
                        Transition += TransitionEvaluator.CostSeconds(
                            OrderedPasses[K - 2], OrderedPasses[K - 1]);
                }
                if (!Wanted.Contains(K)) continue;
                (UInt64 PrefixHash, Int32 K) CacheKey =
                    (PrefixHash(OrderedPasses, K), K);
                if (!Cache.TryGetValue(CacheKey, out PondingMetrics? Metrics))
                {
                    Metrics = Analyser.Score(
                        Surface, Grid.Valid, Outlets.Mask, Grid.AreaM2);
                    Cache[CacheKey] = Metrics;
                }
                PondingCheckpoint Row = new PondingCheckpoint(
                    K, Metrics, Transition);
                if (RetainAllSurfaces
                    || RetainedWorst == null
                    || Metrics.ClosedDepressionVolumeBcy
                        > RetainedWorst.Metrics.ClosedDepressionVolumeBcy)
                {
                    if (!RetainAllSurfaces && RetainedWorst != null)
                        RetainedWorst.Surface = null;
                    Row.Surface = (Double[,])Surface.Clone();
                    RetainedWorst = Row;
                }
                Rows.Add(Row);
            }
            return new PondingAuditResult(Rows);
        }

        /// <summary>Runs optional audit and drainage-aware relocation in place.</summary>
        static public PondingRunResult Run(Grid Grid, IReadOnlyList<Pass> GenerationOrder,
            Route Route, PondingOptions Options,
            Double UnloadedTravelSpeedMph, Double TurnSpeedMph,
            Double TurningCircleFt, ObstacleField? Obstacles = null,
            Int32 ReorderWindow = 50)
        {
            PondingRunResult Result = new();
            PondingOutlets Outlets = BuildOutlets(Grid, Options);
            Result.Outlets = Outlets;
            if (Outlets.Count == 0)
            {
                Result.Warnings.Add("WARNING: no valid ponding outlets; optimization skipped.");
                return Result;
            }
            IPondingAnalyser Analyser = CreateAnalyser(Options);
            Routing.TransitionCostEvaluator Transition = new(
                GradeReturn: true, DirectionAware: true,
                UnloadedTravelSpeedMph, TurnSpeedMph, TurningCircleFt,
                Obstacles);
            Int32 Interval = EffectiveInterval(Options);
            Dictionary<(UInt64 PrefixHash, Int32 K), PondingMetrics> MetricsCache = new();
            Boolean AuditBefore = Options.Audit
                || (Options.Optimize && !Options.NoImplicitAudit);
            if (AuditBefore)
                Result.Before = AuditRoute(Grid, Route.OrderedPasses,
                    Outlets, Analyser, Interval, Transition,
                    MetricsCache: MetricsCache);
            Result.TransitionCostBeforeS = TransitionCost(
                Route.OrderedPasses, Transition);

            if (Options.Optimize)
            {
                LayerGraph Graph = BuildLayerGraph(GenerationOrder);
                List<(Pass Pass, Int32 Id)> Tagged = Route.OrderedPasses
                    .Select((Pass Pass, Int32 Index) =>
                        (Pass, Route.Order[Index])).ToList();
                Tagged = StableTopologicalOrder(Tagged, Graph);
                Int32 Window = Math.Max(2, ReorderWindow);
                Int32 Moves = 0;
                PondingAuditResult CurrentAudit = AuditRoute(Grid,
                    Tagged.Select(T => T.Pass).ToList(), Outlets, Analyser,
                    Interval, Transition, RetainAllSurfaces: true,
                    MetricsCache: MetricsCache);
                Double CurrentT = TransitionCost(
                    Tagged.Select(T => T.Pass).ToList(), Transition);
                for (Int32 Round = 0; Round < 12; Round += 1)
                {
                    Boolean Improved = false;
                    for (Int32 S = 0; S < Tagged.Count; S += 1)
                    {
                        Int32 Lo = Math.Max(0, S - Window);
                        Int32 Hi = Math.Min(Tagged.Count - 1, S + Window);
                        Int32 BestD = S;
                        PondingAuditResult? BestAudit = null;
                        Double BestT = CurrentT;
                        Dictionary<Pass, Int32> Positions = Tagged
                            .Select((T, Index) => (T.Pass, Index))
                            .ToDictionary(T => T.Pass, T => T.Index,
                                new PassComparer());
                        for (Int32 D = Lo; D <= Hi; D += 1)
                        {
                            if (D == S || !MoveIsLegal(
                                Tagged, Positions, Graph, S, D))
                                continue;
                            List<(Pass Pass, Int32 Id)> Candidate = Relocate(Tagged, S, D);
                            List<Pass> Passes = Candidate.Select(T => T.Pass).ToList();
                            PondingAuditResult CandidateAudit = AuditRelocation(
                                Grid, Passes, CurrentAudit, S, D,
                                Outlets, Analyser, MetricsCache);
                            Double CandidateT = CurrentT + RelocateTransitionDelta(
                                Tagged.Select(T => T.Pass).ToList(),
                                Transition, S, D);
                            if (Better(CandidateT, CandidateAudit.PeakVolumeBcy,
                                BestD == S ? CurrentT : BestT,
                                BestD == S ? CurrentAudit.PeakVolumeBcy : BestAudit!.PeakVolumeBcy,
                                Options.Weight))
                            {
                                BestD = D;
                                BestAudit = CandidateAudit;
                                BestT = CandidateT;
                            }
                        }
                        if (BestD == S) continue;
                        Tagged = Relocate(Tagged, S, BestD);
                        CurrentAudit = BestAudit!;
                        CurrentT = BestT;
                        Moves += 1;
                        Improved = true;
                    }
                    if (!Improved) break;
                }
                Route.OrderedPasses = Tagged.Select(T => T.Pass).ToList();
                Route.Order = Tagged.Select(T => T.Id).ToList();
                Route.TotalDeadheadM = Routing.CalculateDeadhead(
                    Route.OrderedPasses, true, Obstacles);
                Result.RelocationsAccepted = Moves;
                Result.After = AuditRoute(Grid, Route.OrderedPasses,
                    Outlets, Analyser, Interval, Transition,
                    MetricsCache: MetricsCache);
            }
            else
                Result.After = Result.Before;

            Result.TransitionCostAfterS = TransitionCost(
                Route.OrderedPasses, Transition);
            ApplyCheckpointMetadata(Route, Result.FinalAudit);
            AddWarnings(Result, Options);
            Result.FailOnWarnTriggered = Options.FailOnWarn
                && Result.Warnings.Count > 0;
            return Result;
        }

        static public String BuildCsv(PondingAuditResult Audit,
            Int32 OutletCount)
        {
            StringBuilder Sb = new();
            Sb.AppendLine("pass_index,closed_depression_volume_bcy,max_pond_depth_m,closed_depression_count,transition_cost_cumulative_s,outlet_count,analyser_ok");
            foreach (PondingCheckpoint C in Audit.Checkpoints)
                Sb.AppendFormat(CultureInfo.InvariantCulture,
                    "{0},{1:F6},{2:F6},{3},{4:F3},{5},{6}\n",
                    C.PassIndex, C.Metrics.ClosedDepressionVolumeBcy,
                    C.Metrics.MaxPondDepthM,
                    C.Metrics.ClosedDepressionCount,
                    C.TransitionCostCumulativeS, OutletCount,
                    C.Metrics.AnalyserOk.ToString().ToLowerInvariant());
            return Sb.ToString();
        }

        static public Byte[] RenderWorstCheckpointPng(Grid Grid,
            PondingOutlets Outlets, PondingCheckpoint Worst,
            Int32 PixelsPerCell = 8)
        {
            Double[,] Surface = Worst.Surface
                ?? throw new ArgumentException("Checkpoint surface was not retained.");
            Double Min = Double.PositiveInfinity;
            Double Max = Double.NegativeInfinity;
            for (Int32 Y = 0; Y < Grid.Height; Y += 1)
            for (Int32 X = 0; X < Grid.Width; X += 1)
                if (Grid.Valid[Y, X])
                {
                    Min = Math.Min(Min, Surface[Y, X]);
                    Max = Math.Max(Max, Surface[Y, X]);
                }
            Int32 Ppb = Math.Max(1, PixelsPerCell);
            using Image<Rgba32> Image = Render.ElevationToImage(
                Surface, Grid.Valid, Ppb, Min, Max);
            Double[,]? Depth = Worst.Metrics.DepressionDepthM;
            for (Int32 Y = 0; Y < Grid.Height; Y += 1)
            for (Int32 X = 0; X < Grid.Width; X += 1)
            {
                Boolean Pond = Depth != null && Depth[Y, X] > 1e-9;
                Boolean Outlet = Outlets.Mask[Y, X];
                if (!Pond && !Outlet) continue;
                Rgba32 Color = Outlet
                    ? new Rgba32(0, 255, 80, 255)
                    : new Rgba32(30, 120, 255, 220);
                Int32 ImageY = (Grid.Height - 1 - Y) * Ppb;
                for (Int32 Py = ImageY; Py < ImageY + Ppb; Py += 1)
                for (Int32 Px = X * Ppb; Px < (X + 1) * Ppb; Px += 1)
                    Image[Px, Py] = Color;
            }
            using MemoryStream Stream = new();
            Image.Save(Stream, new PngEncoder());
            return Stream.ToArray();
        }

        private sealed class LayerGraph
        {
            public Dictionary<Pass, HashSet<Pass>> Predecessors { get; }
                = new(new PassComparer());
            public Dictionary<Pass, HashSet<Pass>> Successors { get; }
                = new(new PassComparer());
        }

        static private LayerGraph BuildLayerGraph(IReadOnlyList<Pass> Passes)
        {
            LayerGraph Graph = new();
            Dictionary<(Int32 X, Int32 Y), (Pass Pass, Int32 Layer)> Last = new();
            foreach (Pass Pass in Passes)
            {
                Graph.Predecessors[Pass] = new HashSet<Pass>(new PassComparer());
                Graph.Successors[Pass] = new HashSet<Pass>(new PassComparer());
                Int32 Min = Int32.MaxValue;
                Int32 Max = 0;
                foreach ((Int32 X, Int32 Y) Cell in TouchedCells(Pass))
                {
                    Int32 Layer = 1;
                    if (Last.TryGetValue(
                        Cell,
                        out (Pass Pass, Int32 Layer) Previous))
                    {
                        Layer = Previous.Layer + 1;
                        Graph.Predecessors[Pass].Add(Previous.Pass);
                        Graph.Successors[Previous.Pass].Add(Pass);
                    }
                    Last[Cell] = (Pass, Layer);
                    Min = Math.Min(Min, Layer);
                    Max = Math.Max(Max, Layer);
                }
                Pass.PondingLayerMin = Min == Int32.MaxValue ? null : Min;
                Pass.PondingLayerMax = Max == 0 ? null : Max;
            }
            return Graph;
        }

        static private IEnumerable<(Int32 X, Int32 Y)> TouchedCells(Pass Pass) =>
            Pass.CutCells.Select(C => (C.Cell.Bx, C.Cell.By))
                .Concat(Pass.FillCells.Select(C => (C.Cell.Bx, C.Cell.By)))
                .Distinct();

        static private List<(Pass Pass, Int32 Id)> StableTopologicalOrder(
            List<(Pass Pass, Int32 Id)> Tagged, LayerGraph Graph)
        {
            Dictionary<Pass, Int32> Position = Tagged.Select((T, I) => (T.Pass, I))
                .ToDictionary(T => T.Pass, T => T.I, new PassComparer());
            Dictionary<Pass, Int32> Degree = Graph.Predecessors
                .ToDictionary(P => P.Key, P => P.Value.Count, new PassComparer());
            PriorityQueue<Pass, Int32> Ready = new();
            foreach (Pass P in Tagged.Select(T => T.Pass))
                if (Degree[P] == 0) Ready.Enqueue(P, Position[P]);
            List<Pass> Ordered = new();
            while (Ready.TryDequeue(out Pass? P, out _))
            {
                Ordered.Add(P);
                foreach (Pass Next in Graph.Successors[P])
                    if (--Degree[Next] == 0) Ready.Enqueue(Next, Position[Next]);
            }
            if (Ordered.Count != Tagged.Count)
                throw new InvalidOperationException("Ponding layer graph contains a cycle.");
            Dictionary<Pass, Int32> Id = Tagged.ToDictionary(
                T => T.Pass, T => T.Id, new PassComparer());
            return Ordered.Select(P => (P, Id[P])).ToList();
        }

        static private Boolean MoveIsLegal(List<(Pass Pass, Int32 Id)> Seq,
            Dictionary<Pass, Int32> Position, LayerGraph Graph,
            Int32 S, Int32 D)
        {
            Pass Moving = Seq[S].Pass;
            Int32 NewPosition(Pass P)
            {
                Int32 Pos = Position[P];
                if (P == Moving) return D;
                if (S < D && Pos > S && Pos <= D) return Pos - 1;
                if (D < S && Pos >= D && Pos < S) return Pos + 1;
                return Pos;
            }
            foreach (Pass P in Graph.Predecessors[Moving])
                if (NewPosition(P) >= D) return false;
            foreach (Pass P in Graph.Successors[Moving])
                if (NewPosition(P) <= D) return false;
            return true;
        }

        static private PondingAuditResult AuditRelocation(
            Grid Grid,
            IReadOnlyList<Pass> Candidate,
            PondingAuditResult Current,
            Int32 Source,
            Int32 Destination,
            PondingOutlets Outlets,
            IPondingAnalyser Analyser,
            Dictionary<(UInt64 PrefixHash, Int32 K), PondingMetrics> MetricsCache)
        {
            Int32 Low = Math.Min(Source, Destination);
            Int32 High = Math.Max(Source, Destination);
            List<PondingCheckpoint> Rows = new();
            PondingCheckpoint Start = Current.Checkpoints
                .Where(C => C.PassIndex <= Low)
                .OrderByDescending(C => C.PassIndex)
                .First();
            Double[,] Surface = Start.Surface != null
                ? (Double[,])Start.Surface.Clone()
                : ReplaySurface(Grid, Candidate, Start.PassIndex);
            Int32 Applied = Start.PassIndex;
            foreach (PondingCheckpoint Existing in Current.Checkpoints)
            {
                Int32 K = Existing.PassIndex;
                if (K <= Low || K > High)
                {
                    Rows.Add(Existing);
                    continue;
                }
                while (Applied < K)
                {
                    ApplyPass(Surface, Candidate[Applied]);
                    Applied += 1;
                }
                (UInt64 PrefixHash, Int32 K) CacheKey =
                    (PrefixHash(Candidate, K), K);
                if (!MetricsCache.TryGetValue(
                    CacheKey, out PondingMetrics? Metrics))
                {
                    Metrics = Analyser.Score(
                        Surface, Grid.Valid, Outlets.Mask, Grid.AreaM2);
                    MetricsCache[CacheKey] = Metrics;
                }
                Rows.Add(new PondingCheckpoint(K, Metrics, 0.0,
                    (Double[,])Surface.Clone()));
            }
            return new PondingAuditResult(Rows);
        }

        static private Double RelocateTransitionDelta(
            IReadOnlyList<Pass> Sequence,
            Routing.TransitionCostEvaluator Evaluator,
            Int32 Source,
            Int32 Destination)
        {
            Double Before = LocalTransitionCost(Sequence, Evaluator,
                Source, Destination);
            List<Pass> Candidate = Sequence.ToList();
            Pass Moving = Candidate[Source];
            Candidate.RemoveAt(Source);
            Candidate.Insert(Destination, Moving);
            Double After = LocalTransitionCost(Candidate, Evaluator,
                Source, Destination);
            return After - Before;
        }

        static private Double LocalTransitionCost(
            IReadOnlyList<Pass> Sequence,
            Routing.TransitionCostEvaluator Evaluator,
            Int32 Source,
            Int32 Destination)
        {
            Int32 Low = Math.Max(0, Math.Min(Source, Destination) - 1);
            Int32 High = Math.Min(Sequence.Count - 2,
                Math.Max(Source, Destination) + 1);
            Double Total = 0.0;
            for (Int32 Index = Low; Index <= High; Index += 1)
                Total += Evaluator.CostSeconds(
                    Sequence[Index], Sequence[Index + 1]);
            return Total;
        }

        static private UInt64 PrefixHash(
            IReadOnlyList<Pass> Passes,
            Int32 K)
        {
            const UInt64 FNV_OFFSET = 14695981039346656037UL;
            const UInt64 FNV_PRIME = 1099511628211UL;
            UInt64 Hash = FNV_OFFSET;
            for (Int32 Index = 0; Index < K; Index += 1)
            {
                Hash ^= unchecked((UInt32)RuntimeHelpers.GetHashCode(
                    Passes[Index]));
                Hash *= FNV_PRIME;
            }
            return Hash;
        }

        static private List<(Pass Pass, Int32 Id)> Relocate(
            List<(Pass Pass, Int32 Id)> Source, Int32 S, Int32 D)
        {
            List<(Pass Pass, Int32 Id)> Out = new(Source);
            (Pass Pass, Int32 Id) Item = Out[S];
            Out.RemoveAt(S);
            Out.Insert(D, Item);
            return Out;
        }

        static private Boolean Better(Double CandidateT, Double CandidateP,
            Double CurrentT, Double CurrentP, Double Weight)
        {
            Double CandidateJ = Weight == 0.0 ? CandidateP
                : CandidateT + Weight * CandidateP;
            Double CurrentJ = Weight == 0.0 ? CurrentP
                : CurrentT + Weight * CurrentP;
            if (CandidateJ < CurrentJ - 1e-7) return true;
            if (Math.Abs(CandidateJ - CurrentJ) > 1e-7) return false;
            if (CandidateP < CurrentP - 1e-7) return true;
            return Math.Abs(CandidateP - CurrentP) <= 1e-7
                && CandidateT < CurrentT - 1e-7;
        }

        static private IPondingAnalyser CreateAnalyser(PondingOptions Options)
        {
            if (!String.IsNullOrWhiteSpace(Options.WhiteBoxToolsPath))
            {
                String PathValue = Options.WhiteBoxToolsPath!;
                String Candidate = Directory.Exists(PathValue)
                    ? Path.Combine(PathValue,
                        OperatingSystem.IsWindows() ? "whitebox_tools.exe" : "whitebox_tools")
                    : PathValue;
                if (!File.Exists(Candidate))
                    throw new InvalidOperationException(
                        $"WhiteBoxTools executable was not found: {Candidate}");
            }
            return new InProcessPondingAnalyser();
        }

        static private Int32 EffectiveInterval(PondingOptions Options)
        {
            if (Options.CheckpointInterval.HasValue)
                return Options.CheckpointInterval.Value;
            return Options.Optimize ? 25 : 1;
        }

        static private List<Int32> BuildCheckpoints(Int32 Count, Int32 Interval)
        {
            SortedSet<Int32> Values = new() { 0, Count };
            if (Interval == 0) Values.Add(Count / 2);
            else
            {
                Int32 Step = Math.Max(1, Interval);
                for (Int32 K = Step; K < Count; K += Step) Values.Add(K);
            }
            return Values.ToList();
        }

        static private Double TransitionCost(IReadOnlyList<Pass> Passes,
            Routing.TransitionCostEvaluator Evaluator)
        {
            Double Total = 0.0;
            for (Int32 I = 1; I < Passes.Count; I += 1)
                Total += Evaluator.CostSeconds(Passes[I - 1], Passes[I]);
            return Total;
        }

        static private void ApplyPass(Double[,] Surface, Pass Pass)
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                in Pass.CutCells)
                Surface[Entry.Cell.By, Entry.Cell.Bx] -= Entry.DepthM;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry
                in Pass.FillCells)
                Surface[Entry.Cell.By, Entry.Cell.Bx] += Entry.DepthM;
        }

        static private void ApplyCheckpointMetadata(Route Route,
            PondingAuditResult? Audit)
        {
            if (Audit == null) return;
            foreach (PondingCheckpoint C in Audit.Checkpoints)
            {
                if (C.PassIndex <= 0 || C.PassIndex > Route.OrderedPasses.Count)
                    continue;
                Pass P = Route.OrderedPasses[C.PassIndex - 1];
                P.MaxPondDepthM = C.Metrics.MaxPondDepthM;
                P.ClosedDepressionVolumeBcy =
                    C.Metrics.ClosedDepressionVolumeBcy;
            }
        }

        static private void AddWarnings(PondingRunResult Result,
            PondingOptions Options)
        {
            PondingAuditResult? Audit = Result.FinalAudit;
            if (Audit == null) return;
            PondingMetrics Metrics = Audit.WorstCheckpoint.Metrics;
            if (Metrics.MaxPondDepthM
                > Options.WarnDepthFt * Constants.FEET_TO_METERS)
                Result.Warnings.Add(
                    "WARNING: significant transient ponding risk");
            Double VolumeThreshold = Options.WarnVolumeBcy;
            if (VolumeThreshold == 0.0 && Options.DesignRainfallIn > 0.0)
                VolumeThreshold = Options.DesignRainfallIn * 0.0254
                    * Metrics.PondingCatchmentAreaM2 * Constants.M3_TO_CY;
            if (VolumeThreshold > 0.0
                && Metrics.ClosedDepressionVolumeBcy > VolumeThreshold)
                Result.Warnings.Add(
                    "WARNING: large trapped volume on partial surface");
        }

        static private Boolean IsPerimeter(Boolean[,] Valid, Int32 X, Int32 Y)
        {
            Int32 H = Valid.GetLength(0);
            Int32 W = Valid.GetLength(1);
            if (X == 0 || Y == 0 || X == W - 1 || Y == H - 1) return true;
            return !Valid[Y - 1, X] || !Valid[Y + 1, X]
                || !Valid[Y, X - 1] || !Valid[Y, X + 1];
        }

        static private void AddOutlet(Boolean[,] Mask,
            PondingOutletSource[,] Source, Int32 X, Int32 Y,
            PondingOutletSource Kind)
        {
            Mask[Y, X] = true;
            Source[Y, X] |= Kind;
        }

        static private Boolean[,] ReverseDescendingReachability(Grid Grid,
            Boolean[,] Boundary)
        {
            Boolean[,] Reached = new Boolean[Grid.Height, Grid.Width];
            Queue<(Int32 X, Int32 Y)> Queue = new();
            for (Int32 Y = 0; Y < Grid.Height; Y += 1)
            for (Int32 X = 0; X < Grid.Width; X += 1)
                if (Boundary[Y, X])
                {
                    Reached[Y, X] = true;
                    Queue.Enqueue((X, Y));
                }
            while (Queue.Count > 0)
            {
                (Int32 X, Int32 Y) C = Queue.Dequeue();
                for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                {
                    if (Dx == 0 && Dy == 0) continue;
                    Int32 Nx = C.X + Dx;
                    Int32 Ny = C.Y + Dy;
                    if (Nx < 0 || Nx >= Grid.Width || Ny < 0 || Ny >= Grid.Height
                        || Reached[Ny, Nx] || !Grid.Valid[Ny, Nx]) continue;
                    if (Grid.Target[Ny, Nx] + 1e-9 < Grid.Target[C.Y, C.X])
                        continue;
                    Reached[Ny, Nx] = true;
                    Queue.Enqueue((Nx, Ny));
                }
            }
            return Reached;
        }

        static private (Double Lat, Double Lon) ParsePoint(String Spec)
        {
            String[] Parts = Spec.Split(',',
                StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);
            if (Parts.Length != 2)
                throw new ArgumentException(
                    $"Invalid ponding outlet '{Spec}'. Expected 'lat,lon'.");
            return (Double.Parse(Parts[0], CultureInfo.InvariantCulture),
                Double.Parse(Parts[1], CultureInfo.InvariantCulture));
        }

        static private List<(Double X, Double Y)> ParseRing(String Spec, Grid Grid)
        {
            List<(Double X, Double Y)> Ring = new();
            foreach (String Chunk in Spec.Replace("\n", ";",
                         StringComparison.Ordinal).Split(';',
                         StringSplitOptions.RemoveEmptyEntries))
            {
                (Double Lat, Double Lon) = ParsePoint(Chunk);
                UtmCoord U = Agd.UtmFromLatlon(Lat, Lon);
                if (U.Zone != Grid.Zone || U.IsNorthern != Grid.IsNorthern)
                    throw new ArgumentException(
                        "Ponding outlet polygon is outside the grid UTM zone.");
                Ring.Add((U.Easting, U.Northing));
            }
            if (Ring.Count < 3)
                throw new ArgumentException(
                    "Ponding outlet polygon needs at least 3 vertices.");
            return Ring;
        }

        static private (Int32 X, Int32 Y) NearestValid(Grid Grid,
            Double Easting, Double Northing)
        {
            Double Best = Double.PositiveInfinity;
            (Int32 X, Int32 Y) Cell = (-1, -1);
            for (Int32 Y = 0; Y < Grid.Height; Y += 1)
            for (Int32 X = 0; X < Grid.Width; X += 1)
            {
                if (!Grid.Valid[Y, X]) continue;
                Double Dx = Grid.CenterX[Y, X] - Easting;
                Double Dy = Grid.CenterY[Y, X] - Northing;
                Double D2 = Dx * Dx + Dy * Dy;
                if (D2 < Best) { Best = D2; Cell = (X, Y); }
            }
            if (Cell.X < 0)
                throw new InvalidOperationException("Grid has no valid outlet cell.");
            return Cell;
        }

        static private Boolean CellIntersectsPolygon(Grid Grid, Int32 X,
            Int32 Y, List<(Double X, Double Y)> Ring)
        {
            Double Cx = Grid.CenterX[Y, X];
            Double Cy = Grid.CenterY[Y, X];
            if (GridMath.PointInPolygon(Cx, Cy, Ring)) return true;
            Double Half = Grid.GridSizeM / 2.0;
            foreach ((Double X, Double Y) P in Ring)
                if (P.X >= Cx - Half && P.X <= Cx + Half
                    && P.Y >= Cy - Half && P.Y <= Cy + Half) return true;
            return false;
        }
    }
}
#pragma warning restore CS1591
