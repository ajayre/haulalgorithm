using System.Globalization;
using System.Linq;
using System.Text;

namespace HaulBuilder
{
    /// <summary>
    /// End-to-end in-memory pipeline mirroring <c>landlevel.cli.main</c>.
    /// </summary>
    public sealed class HaulBuilderEngine
    {
        private const Double M2_PER_ACRE = 4046.8564224;
        private const Double METERS_PER_YARD = 0.9144;
        private const Double METERS_PER_MILE = 1609.344;

        /// <summary>
        /// Executes a full haul-planning run and returns typed in-memory results.
        /// </summary>
        /// <param name="options">Pipeline options equivalent to CLI flags.</param>
        /// <returns>Structured pipeline outputs and render payloads.</returns>
        /// <exception cref="ArgumentNullException">Thrown when options is null.</exception>
        /// <exception cref="ArgumentException">Thrown when required options are invalid.</exception>
        public HaulBuilderResult Run
            (
            HaulBuilderOptions options
            )
        {
            // Options are copied so later caller mutations cannot alter an already produced result.
            if (options is null) throw new ArgumentNullException(nameof(options));
            if (String.IsNullOrWhiteSpace(options.AgdPath)) throw new ArgumentException("AGD path is required.", nameof(options));
            if (!File.Exists(options.AgdPath)) throw new ArgumentException($"AGD file not found: {options.AgdPath}", nameof(options));
            if (options.PixelsPerBin <= 0) throw new ArgumentException("PixelsPerBin must be > 0.", nameof(options));
            ValidateEdgePatchOptions(options);
            ValidateHaulOptimizationOptions(options);

            HaulBuilderOptions Opt = CloneOptions(options);
            HaulBuilderResult Result = new HaulBuilderResult
            {
                Options = Opt,
            };

            AgdField Field = Agd.Parse(Opt.AgdPath);
            Grid Grid = GridMath.BuildGrid(Field, Opt.GridSizeFt, Opt.SwellFactor, Opt.CutFillRatio, Opt.MaxDepthMm);
            Result.Field = Field;
            Result.Grid = Grid;

            ObstacleField Obstacles = ResolveExclusions(Result, Field, Grid, Opt);
            Double BalanceThreshold = Obstacles.Active ? Double.PositiveInfinity : Balance.DEFAULT_IMBALANCE_THRESHOLD;
            BalanceResult BalanceResult = Balance.Compute(Grid, Opt.BalanceMode, BalanceThreshold);
            Result.Balance = BalanceResult;

            FlowField Flow = FlowFieldSolver.Compute(Grid, Opt.TransportMode);
            Result.Flow = Flow;
            Result.PngBytesByName["main"] = Render.RenderPng(Grid, Flow, Opt.PixelsPerBin, Opt.ArrowStride);

            if (Opt.NoPlan)
            {
                Result.Messages.Add("Planning stages skipped because NoPlan was set.");
                return Result;
            }

            List<Strip> StripList = Strips.TraceStrips(Grid, Flow, 400);
            Double HeadingDeg = Opt.HeadingDeg.HasValue ? Opt.HeadingDeg.Value : Passes.EstimateSlopeHeading(Grid);
            Boolean FlowAligned = !Opt.NoFlowAligned;
            Double ScraperBcy = (Opt.ScraperCapacityLcy / Opt.SwellFactor);

            PassResult PassResult;
            EdgePatchOmissionResult? EdgePatchResult = null;
            if (Opt.EnableHaulOptimization)
            {
                if (Obstacles.Active)
                {
                    throw new NotSupportedException(
                        "Global haul optimization does not yet support exclusion polygons.");
                }
                HaulWorkLedger Ledger = HaulWorkLedger.Build(
                    Grid, Opt.ToleranceMm);
                HaulOptimizationResult Optimized =
                    HaulCycleOptimizer.Optimize(
                        Grid,
                        Flow,
                        Ledger,
                        Opt.HaulOptimizationAxis,
                        ScraperBcy,
                        Opt.HaulOptimizationMaxRunsPerKind,
                        Opt.HaulOptimizationDeferEdgeWork,
                        Opt.HaulOptimizationFlowPenaltyWeight,
                        Obstacles);
                List<Pass> OptimizedPasses = Optimized.PassResult.Passes;
                HaulPairDistanceRefiner.Result? PairRefinement = null;
                if (Opt.HaulOptimizationPairRefinementRounds > 0)
                {
                    PairRefinement = HaulPairDistanceRefiner.Refine(
                        OptimizedPasses,
                        Grid,
                        Opt.HaulOptimizationPairRefinementRounds,
                        Opt.HaulOptimizationPairMetric,
                        Seed: 0,
                        Obstacles: Obstacles);
                    OptimizedPasses = PairRefinement.Passes;
                }
                HaulDifficultCycleFilter.Result? DifficultFilter = null;
                if (Opt.HaulOptimizationMaximumCompactOmissions > 0)
                {
                    DifficultFilter = HaulDifficultCycleFilter.Apply(
                        OptimizedPasses,
                        Grid,
                        Opt.BladeWidthMm / 1000.0,
                        !Opt.NoGradeReturn,
                        Opt.HaulOptimizationMaximumCompactOmissions,
                        Opt.HaulOptimizationCompactOmissionPriority);
                    OptimizedPasses = DifficultFilter.Passes;
                }
                if ((PairRefinement != null) || (DifficultFilter != null))
                {
                    IReadOnlyList<HaulDeferredWork> Deferred =
                        DifficultFilter == null
                            ? Optimized.DeferredWork
                            : PreserveExistingDeferredReasons(
                                Ledger.BuildDeferredWork(
                                    OptimizedPasses,
                                    "GUIDANCE_COMPACT_PREFLIGHT"),
                                Optimized.DeferredWork);
                    Double UnmatchedCutBcy = Deferred
                        .Where(Entry =>
                            Entry.Kind == HaulLaneBuilder.WorkKind.Cut)
                        .Sum(Entry => Entry.VolumeBcy);
                    Double UnmatchedFillBcy = Deferred
                        .Where(Entry =>
                            Entry.Kind == HaulLaneBuilder.WorkKind.Fill)
                        .Sum(Entry => Entry.VolumeBcy);
                    HaulPlanValidation Validation = Ledger.ValidateComplete(
                        OptimizedPasses,
                        ScraperBcy,
                        Grid.MaxDepthMm / 1000.0,
                        AllowDeferredWork: Deferred.Count > 0);
                    Optimized = new HaulOptimizationResult(
                        new PassResult(
                            OptimizedPasses,
                            Optimized.PassResult.Rounds,
                            UnmatchedCutBcy,
                            UnmatchedFillBcy),
                        Optimized.Axis,
                        Optimized.CutLaneCount,
                        Optimized.FillLaneCount,
                        OptimizedPasses.Sum(Pass => Pass.LoadBcy),
                        UnmatchedCutBcy,
                        UnmatchedFillBcy,
                        Deferred,
                        Validation,
                        PairRefinement?.Swaps ?? 0,
                        PairRefinement?.BeforeM ?? 0.0,
                        PairRefinement?.AfterM ?? 0.0,
                        DifficultFilter?.FailureCount ?? 0,
                        DifficultFilter?.OmittedCycles ?? 0);
                }
                if (!Optimized.Validation.Success)
                {
                    throw new InvalidOperationException(
                        "Optimized haul allocation failed ledger validation: "
                        + String.Join("; ", Optimized.Validation.Issues.Take(10)));
                }
                Result.HaulOptimization = Optimized;
                PassResult = Optimized.PassResult;
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Global haul optimization generated {0} cycles with {1:F3} BCY cut and {2:F3} BCY fill explicitly deferred.",
                    PassResult.Passes.Count,
                    Optimized.UnmatchedCutBcy,
                    Optimized.UnmatchedFillBcy));
                if (Optimized.PairRefinementSwaps > 0)
                {
                    Result.Messages.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Equal-load pair refinement accepted {0} swaps and reduced its geometry proxy from {1:F1} m to {2:F1} m.",
                        Optimized.PairRefinementSwaps,
                        Optimized.PairRefinementBeforeM,
                        Optimized.PairRefinementAfterM));
                }
                if (Optimized.CompactOmittedCycles > 0)
                {
                    Result.Messages.Add(String.Format(
                        CultureInfo.InvariantCulture,
                        "Compact guidance preflight explicitly deferred {0} of {1} failing cycles for later analysis.",
                        Optimized.CompactOmittedCycles,
                        Optimized.CompactPreflightFailures));
                }
            }
            else
            {
                PassResult = Passes.GeneratePasses(
                    Grid,
                    StripList,
                    Opt.ToleranceMm,
                    ScraperBcy,
                    0,
                    FlowAligned ? null : HeadingDeg,
                    Opt.HeadingStrength,
                    Opt.StripBias,
                    (Opt.MaxDropHopFt * Constants.FEET_TO_METERS),
                    Flow,
                    FlowAligned,
                    (Opt.MinLoadFillPct / 100.0),
                    Obstacles);
                EdgePatchResult = EdgePatchOmission.Apply(
                    PassResult,
                    Grid,
                    Opt,
                    Obstacles);
                PassResult = EdgePatchResult.PassResult;
                Result.EdgePatchOmission = EdgePatchResult;
            }
            Result.PassResult = PassResult;
            if ((EdgePatchResult != null)
                && (EdgePatchResult.OmittedComponentCount > 0))
            {
                Result.Messages.Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "Omitted {0} isolated edge cut patch(es): {1:F2} BCY cut and {2:F2} BCY matching fill demand across {3} cut cell(s).",
                    EdgePatchResult.OmittedComponentCount,
                    EdgePatchResult.OmittedCutBcy,
                    EdgePatchResult.OmittedFillBcy,
                    EdgePatchResult.OmittedCutCells.Count));
            }

            Route Route = Routing.SequencePasses(
                PassResult.Passes,
                1500,
                !Opt.NoGradeReturn,
                !Opt.NoDirectionAware,
                Opt.ReturnSpeedMph,
                Opt.TurnSpeedMph,
                Opt.TurningCircleFt,
                Obstacles);
            Result.Route = Route;

            if (!Opt.NoKnotRepair && !Opt.EnableHaulOptimization)
            {
                Swaths.ReprofileKnots(Route, Grid, Opt.BladeWidthMm, 3, HeadingDeg, Opt.LaneOverlap);
            }

            Int32 ActiveReorderWindow = Opt.EnableHaulOptimization
                ? Opt.HaulOptimizationReorderWindow
                : Opt.ReorderWindow;
            if (ActiveReorderWindow > 0)
            {
                Routing.RefineRouteOrder(
                    Route,
                    ActiveReorderWindow,
                    !Opt.NoGradeReturn,
                    !Opt.NoDirectionAware,
                    Opt.ReturnSpeedMph,
                    Opt.TurnSpeedMph,
                    Opt.TurningCircleFt,
                    12,
                    Obstacles);
            }

            BuildZoneSequence(Result, Grid, Field, Route, Opt);
            BuildPairs(Result, Grid, Field, Route, Opt);
            BuildGuidance(Result, Grid, Field, Route, Obstacles, HeadingDeg, Opt);
            BuildTimingAndMetrics(Result, Grid, PassResult, Route, HeadingDeg, FlowAligned, Opt);
            BuildAnimations(Result, Grid, Route, Opt);
            BuildPivot(Result, Grid, Route, Obstacles, Opt);

            // TEMP: route-2 parity diagnostics — remove after host settings are aligned.
            LogRoute2ParityDebug(Result, Opt, Grid, HeadingDeg, FlowAligned, ScraperBcy, StripList.Count);

            return Result;
        }

        /// <summary>
        /// TEMP debug dump of effective settings and sequenced route-2 identity for host vs test parity.
        /// </summary>
        /// <param name="result">Pipeline result to annotate.</param>
        /// <param name="opt">Effective options used for this run.</param>
        /// <param name="grid">Built grid.</param>
        /// <param name="headingDeg">Resolved heading degrees.</param>
        /// <param name="flowAligned">Whether flow-aligned growth was used.</param>
        /// <param name="scraperBcy">Bank load limit used by the matcher.</param>
        /// <param name="stripCount">Number of traced strips.</param>
        static private void LogRoute2ParityDebug
            (
            HaulBuilderResult result,
            HaulBuilderOptions opt,
            Grid grid,
            Double headingDeg,
            Boolean flowAligned,
            Double scraperBcy,
            Int32 stripCount
            )
        {
            const String Prefix = "HAULBUILDER_DEBUG";
            List<String> Lines = new List<String>();

            void Add
                (
                String Line
                )
            {
                String Text = Prefix + " " + Line;
                Lines.Add(Text);
                result.Messages.Add(Text);
                Console.WriteLine(Text);
                System.Diagnostics.Trace.WriteLine(Text);
            }

            Add("===== BEGIN route-2 parity dump (TEMP) =====");
            Add(String.Format(CultureInfo.InvariantCulture, "agd={0}", opt.AgdPath));
            Add(String.Format(
                CultureInfo.InvariantCulture,
                "grid={0}x{1} GridSizeFt={2} GridSizeM={3:F6}",
                grid.Width,
                grid.Height,
                opt.GridSizeFt,
                grid.GridSizeM));
            Add(String.Format(
                CultureInfo.InvariantCulture,
                "settings: ToleranceMm={0} ScraperLCY={1} Swell={2} ScraperBCY={3:F6} CutFillRatio={4} MaxDepthMm={5} BladeWidthMm={6}",
                opt.ToleranceMm,
                opt.ScraperCapacityLcy,
                opt.SwellFactor,
                scraperBcy,
                opt.CutFillRatio,
                opt.MaxDepthMm,
                opt.BladeWidthMm));
            Add(String.Format(
                CultureInfo.InvariantCulture,
                "settings: FlowAligned={0} NoFlowAligned={1} HeadingDeg={2:F3} HeadingStrength={3} StripBias={4} MaxDropHopFt={5} MinLoadFillPct={6}",
                flowAligned,
                opt.NoFlowAligned,
                headingDeg,
                opt.HeadingStrength,
                opt.StripBias,
                opt.MaxDropHopFt,
                opt.MinLoadFillPct));
            Add(String.Format(
                CultureInfo.InvariantCulture,
                "settings: TransportMode={0} BalanceMode={1} NoGradeReturn={2} NoDirectionAware={3} NoKnotRepair={4} ReorderWindow={5} LaneOverlap={6} NoRouteSmoothing={7}",
                opt.TransportMode,
                opt.BalanceMode,
                opt.NoGradeReturn,
                opt.NoDirectionAware,
                opt.NoKnotRepair,
                opt.ReorderWindow,
                opt.LaneOverlap,
                opt.NoRouteSmoothing));
            Add(String.Format(
                CultureInfo.InvariantCulture,
                "settings: ReturnMph={0} TurnMph={1} TurningCircleFt={2} ExcludePolygons={3} StripCount={4}",
                opt.ReturnSpeedMph,
                opt.TurnSpeedMph,
                opt.TurningCircleFt,
                opt.ExcludePolygonSpecs.Count,
                stripCount));

            // Expected TheShop2@7.5ft fingerprint from the parity suite.
            Add("expected_theshop2_7p5ft: passes=3102 order1=2199 route2_haul_m~63.763 route2_ncut=5 route2_uniq=5 route2_first=(64,2) grid~177x173");
            Add(String.Format(
                CultureInfo.InvariantCulture,
                "expected_defaults: ToleranceMm={0} MaxDepthMm={1} BladeWidthMm={2} CutFillRatio={3}",
                Constants.DEFAULT_TOLERANCE_MM,
                Constants.DEFAULT_MAX_DEPTH_MM,
                Constants.DEFAULT_BLADE_WIDTH_MM,
                Constants.DEFAULT_COMPACTION));
            Add(String.Format(
                CultureInfo.InvariantCulture,
                "delta: ToleranceMm={0:F6} MaxDepthMm={1:F6} BladeWidthMm={2:F6} (host-expected)",
                opt.ToleranceMm - Constants.DEFAULT_TOLERANCE_MM,
                opt.MaxDepthMm - Constants.DEFAULT_MAX_DEPTH_MM,
                opt.BladeWidthMm - Constants.DEFAULT_BLADE_WIDTH_MM));

            if (result.PassResult is null || result.Route is null)
            {
                Add("plan missing (NoPlan or early exit)");
                Add("===== END route-2 parity dump =====");
                return;
            }

            PassResult PassResult = result.PassResult;
            Route Route = result.Route;
            Double TotalHaul = PassResult.Passes.Sum(Pass => Pass.HaulDistanceM);
            Double TotalBcy = PassResult.Passes.Sum(Pass => Pass.LoadBcy);
            Add(String.Format(
                CultureInfo.InvariantCulture,
                "plan: passes={0} moved_bcy={1:F2} total_haul_m={2:F1} deadhead_m={3:F1}",
                PassResult.Passes.Count,
                TotalBcy,
                TotalHaul,
                Route.TotalDeadheadM));

            Int32 OrderTake = Math.Min(20, Route.Order.Count);
            Add("order[:20]=" + String.Join(",", Route.Order.Take(OrderTake)));

            for (Int32 Gen = 0; Gen < Math.Min(3, PassResult.Passes.Count); Gen += 1)
            {
                Pass GenPass = PassResult.Passes[Gen];
                (Int32 Bx, Int32 By) First = GenPass.CutCells.Count > 0 ? GenPass.CutCells[0].Cell : (-1, -1);
                Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "gen[{0}]: ncut={1} nfill={2} haul_m={3:F6} load_bcy={4:F6} first_cut=({5},{6}) cut_h={7}",
                    Gen,
                    GenPass.CutCells.Count,
                    GenPass.FillCells.Count,
                    GenPass.HaulDistanceM,
                    GenPass.LoadBcy,
                    First.Bx,
                    First.By,
                    GenPass.CutHeadingDeg.HasValue ? GenPass.CutHeadingDeg.Value.ToString("F3", CultureInfo.InvariantCulture) : "null"));
            }

            if (Route.OrderedPasses.Count >= 2)
            {
                Pass Route2 = Route.OrderedPasses[1];
                Int32 Uniq = Route2.CutCells.Select(Entry => (Entry.Cell.Bx, Entry.Cell.By)).Distinct().Count();
                (Int32 Bx, Int32 By) First = Route2.CutCells.Count > 0 ? Route2.CutCells[0].Cell : (-1, -1);
                (Int32 Bx, Int32 By) Last = Route2.CutCells.Count > 0 ? Route2.CutCells[Route2.CutCells.Count - 1].Cell : (-1, -1);
                String FirstCuts = String.Join(",", Route2.CutCells.Take(8).Select(Entry =>
                    String.Format(CultureInfo.InvariantCulture, "({0},{1},{2:F4})", Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM)));
                Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "route2: order_index={0} ncut={1} uniq_cut={2} nfill={3} haul_m={4:F6} load_bcy={5:F6} first=({6},{7}) last=({8},{9}) cut_h={10}",
                    Route.Order[1],
                    Route2.CutCells.Count,
                    Uniq,
                    Route2.FillCells.Count,
                    Route2.HaulDistanceM,
                    Route2.LoadBcy,
                    First.Bx,
                    First.By,
                    Last.Bx,
                    Last.By,
                    Route2.CutHeadingDeg.HasValue ? Route2.CutHeadingDeg.Value.ToString("F3", CultureInfo.InvariantCulture) : "null"));
                Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "route2: cut_start=({0:F3},{1:F3}) cut_end=({2:F3},{3:F3}) fill_start=({4:F3},{5:F3}) fill_end=({6:F3},{7:F3})",
                    Route2.CutStartXy.X,
                    Route2.CutStartXy.Y,
                    Route2.CutEndXy.X,
                    Route2.CutEndXy.Y,
                    Route2.FillStartXy.X,
                    Route2.FillStartXy.Y,
                    Route2.FillEndXy.X,
                    Route2.FillEndXy.Y));
                Add("route2: first8_cuts=" + FirstCuts);

                Boolean MatchFingerprint =
                    (Route.Order[1] == 2199)
                    && (Math.Abs(Route2.HaulDistanceM - 63.763) < 0.1)
                    && (Route2.CutCells.Count == 5)
                    && (Uniq == 5)
                    && (First.Bx == 64)
                    && (First.By == 2)
                    && (grid.Width == 177)
                    && (grid.Height == 173);
                Boolean SettingsOk =
                    (Math.Abs(opt.GridSizeFt - Constants.DEFAULT_GRID_SIZE_FT) < 1e-9)
                    && (Math.Abs(opt.ToleranceMm - Constants.DEFAULT_TOLERANCE_MM) < 1e-9)
                    && (Math.Abs(opt.MaxDepthMm - Constants.DEFAULT_MAX_DEPTH_MM) < 1e-9)
                    && (Math.Abs(opt.BladeWidthMm - Constants.DEFAULT_BLADE_WIDTH_MM) < 1e-9);
                if (MatchFingerprint)
                {
                    Add("COMPARE: MATCHES TheShop2@7.5ft parity fingerprint");
                }
                else if (!SettingsOk)
                {
                    Add("COMPARE: DOES NOT MATCH — set ToleranceMm=10, MaxDepthMm=61, BladeWidthMm=2271 (host still has converted inch values)");
                }
                else
                {
                    Add("COMPARE: DOES NOT MATCH TheShop2@7.5ft parity fingerprint — settings look right; check AGD identity / DLL build");
                }
            }
            else
            {
                Add("route2: unavailable (fewer than 2 ordered passes)");
            }

            if ((result.Guidance is not null) && (result.Guidance.Points.Count > 0))
            {
                Int32 Route2Cuts = result.Guidance.Points.Count(Point => (Point.PassNo == 2) && (Point.Action == "CUT"));
                Int32 Route2All = result.Guidance.Points.Count(Point => Point.PassNo == 2);
                Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "guidance: total_pts={0} route2_pts={1} route2_CUT={2}",
                    result.Guidance.Points.Count,
                    Route2All,
                    Route2Cuts));
            }

            if (result.Metrics is not null)
            {
                Add(String.Format(
                    CultureInfo.InvariantCulture,
                    "metrics: hours={0:F3} haul_m={1:F1} moved_bcy={2:F2}",
                    result.Metrics.TotalHours,
                    result.TimeResult is not null ? result.TimeResult.HaulingDistM : 0.0,
                    result.Metrics.MoveVolumeBcy));
            }

            Add("===== END route-2 parity dump =====");

            try
            {
                String LogPath = Path.Combine(Path.GetTempPath(), "haulbuilder_route2_debug.log");
                File.WriteAllLines(LogPath, Lines);
                String PathMsg = Prefix + " wrote " + LogPath;
                result.Messages.Add(PathMsg);
                Console.WriteLine(PathMsg);
            }
            catch (Exception Ex)
            {
                Add("log_file_write_failed: " + Ex.Message);
            }
        }

        /// <summary>
        /// Executes a full run with Python-style naming.
        /// </summary>
        static public HaulBuilderResult run
            (
            HaulBuilderOptions options
            )
        {
            // Python naming alias is provided for parity with the source module shape.
            HaulBuilderEngine Engine = new HaulBuilderEngine();
            return Engine.Run(options);
        }

        private static ObstacleField ResolveExclusions
            (
            HaulBuilderResult result,
            AgdField field,
            Grid grid,
            HaulBuilderOptions options
            )
        {
            // Exclusion polygon parsing mirrors CLI behavior and enables both planning and routing detours.
            if (options.ExcludePolygonSpecs.Count <= 0)
            {
                return HaulBuilder.Obstacles.EMPTY_OBSTACLES;
            }

            List<List<(Double Lat, Double Lon)>> Rings = ParsePolygonSpecs(options.ExcludePolygonSpecs);
            List<List<(Double X, Double Y)>> RingsXy = new List<List<(Double X, Double Y)>>();
            foreach (List<(Double Lat, Double Lon)> Ring in Rings)
            {
                RingsXy.Add(Agd.ProjectLatlonRing(Ring));
            }

            GridMath.ApplyExclusions(grid, RingsXy);
            Double ClearanceM = ((options.EquipmentWidthFt * Constants.FEET_TO_METERS) / 2.0);
            ObstacleField ObstacleFieldResult = ObstacleField.FromRings(grid.Exclusions, ClearanceM);
            result.Messages.Add(String.Format(CultureInfo.InvariantCulture, "Applied {0} exclusion polygon(s).", Rings.Count));
            return ObstacleFieldResult;
        }

        private static void BuildZoneSequence
            (
            HaulBuilderResult result,
            Grid grid,
            AgdField field,
            Route route,
            HaulBuilderOptions options
            )
        {
            // Zone sequencing condenses hundreds of passes into a coarse operational work order.
            if (options.Zones <= 0)
            {
                return;
            }

            List<ZoneStep> Steps = Sequence.SequenceCutZones(grid, field, route, options.Zones, options.Seed);
            result.ZoneSteps.AddRange(Steps);
            if (Steps.Count > 0)
            {
                String Csv = BuildZoneCsv(Steps);
                result.CsvTextByName["zone_sequence"] = Csv;
                Byte[] Overlay = Render.OverlayZoneWaypoints(result.PngBytesByName["main"], grid, Steps, options.PixelsPerBin);
                result.PngBytesByName["main_with_zones"] = Overlay;
            }
        }

        private static void BuildPairs
            (
            HaulBuilderResult result,
            Grid grid,
            AgdField field,
            Route route,
            HaulBuilderOptions options
            )
        {
            // Pair rows are built explicitly so the engine never relies on file-backed CSV helpers.
            TimeParams TimeParams = new TimeParams(
                options.LoadingSpeedMph,
                options.HaulSpeedMph,
                options.DumpingSpeedMph,
                options.ReturnSpeedMph,
                options.AccelTimeS,
                options.DecelTimeS,
                options.FieldEfficiency);
            Double TotalCutBcyBefore = 0.0;
            Double TotalFillBcyBefore = 0.0;
            for (Int32 Iter = 0; Iter < route.OrderedPasses.Count; Iter += 1)
            {
                Pass Pass = route.OrderedPasses[Iter];
                Double RouteTimeS = TimeModel.EstimatePassTime(grid, Pass, TimeParams);
                result.CutFillPairs.Add(
                    new CutFillPair(
                        Iter + 1,
                        Pass.LoadBcy,
                        Pass.HaulDistanceM,
                        Pass.CutStartXy,
                        Pass.CutEndXy,
                        Pass.FillStartXy,
                        Pass.FillEndXy,
                        TotalCutBcyBefore,
                        TotalFillBcyBefore,
                        RouteTimeS));
                TotalCutBcyBefore += TimeModel.PassCutBcy(grid, Pass);
                TotalFillBcyBefore += TimeModel.PassFillBcy(grid, Pass);
            }

            if (options.PairsCsvPath is not null)
            {
                result.CsvTextByName["cut_fill_pairs"] = BuildPairsCsv(result.CutFillPairs, field);
            }

            if (options.PairsPngPath is not null)
            {
                result.PngBytesByName["cut_fill_pairs"] = Render.RenderPassPairsPng(
                    grid,
                    route.OrderedPasses,
                    options.PixelsPerBin);
            }
        }

        private static void BuildGuidance
            (
            HaulBuilderResult result,
            Grid grid,
            AgdField field,
            Route route,
            ObstacleField obstacles,
            Double headingDeg,
            HaulBuilderOptions options
            )
        {
            // Guidance stage is optional and only runs when one of its artifacts is requested.
            Boolean NeedGuidance = options.EnableHaulOptimization
                || (options.GuidanceCsvPath is not null)
                || (options.GuidancePngPath is not null)
                || (options.PairSampleCount > 0);
            if (!NeedGuidance)
            {
                return;
            }

            GuidancePlanner.BuildResult Plan = GuidancePlanner.Build(
                route,
                grid,
                field,
                headingDeg,
                options,
                Diagnostic: Message => result.Messages.Add(Message));
            GuidanceResult Guidance = Plan.Guidance;
            result.Guidance = Guidance;
            result.GuidancePoints.AddRange(Guidance.Points);
            result.GuidanceTime = Plan.SummaryTime;
            result.LifecycleGuidance = Plan.Lifecycle;
            result.DetailedGuidanceTime = Plan.Timing;

            result.CsvTextByName["guidance"] = BuildGuidanceCsv(Guidance.Points);
            if (options.GuidancePngPath is not null)
            {
                result.PngBytesByName["guidance"] = Render.RenderGuidancePng(grid, Guidance.Points, options.PixelsPerBin, (options.BladeWidthMm / 1000.0));
            }

            BuildPairSamples(result, grid, Guidance.Points, options);
        }

        private static void BuildPairSamples
            (
            HaulBuilderResult result,
            Grid grid,
            IReadOnlyList<GuidancePoint> points,
            HaulBuilderOptions options
            )
        {
            // Random single-pair PNGs mirror make_pair_images.py for operator-facing cut/haul/fill diagrams.
            if (options.PairSampleCount <= 0)
            {
                return;
            }

            Dictionary<Int32, List<GuidancePoint>> ByPass = new Dictionary<Int32, List<GuidancePoint>>();
            for (Int32 Iter = 0; Iter < points.Count; Iter += 1)
            {
                GuidancePoint Point = points[Iter];
                if (!ByPass.TryGetValue(Point.PassNo, out List<GuidancePoint>? Bucket))
                {
                    Bucket = new List<GuidancePoint>();
                    ByPass[Point.PassNo] = Bucket;
                }

                Bucket.Add(Point);
            }

            List<Int32> Usable = new List<Int32>();
            foreach (KeyValuePair<Int32, List<GuidancePoint>> Entry in ByPass)
            {
                Boolean HasCut = Entry.Value.Any(Point => Point.Action == "CUT");
                Boolean HasFill = Entry.Value.Any(Point => Point.Action == "FILL");
                if (HasCut && HasFill)
                {
                    Usable.Add(Entry.Key);
                }
            }

            if (Usable.Count <= 0)
            {
                return;
            }

            Int32 Take = Math.Min(options.PairSampleCount, Usable.Count);
            Random Rng = new Random(options.Seed);
            List<Int32> Pool = new List<Int32>(Usable);
            for (Int32 Iter = 0; Iter < Pool.Count; Iter += 1)
            {
                Int32 Swap = Iter + Rng.Next(Pool.Count - Iter);
                (Pool[Iter], Pool[Swap]) = (Pool[Swap], Pool[Iter]);
            }

            List<Int32> Chosen = Pool.Take(Take).OrderBy(Value => Value).ToList();
            Double BladeM = (options.BladeWidthMm / 1000.0);
            for (Int32 Iter = 0; Iter < Chosen.Count; Iter += 1)
            {
                Int32 PassNo = Chosen[Iter];
                String Key = String.Format(CultureInfo.InvariantCulture, "pair_{0:D4}", PassNo);
                result.PngBytesByName[Key] = Render.RenderSinglePairPng(grid, ByPass[PassNo], BladeM);
            }
        }

        private static void BuildTimingAndMetrics
            (
            HaulBuilderResult result,
            Grid grid,
            PassResult passResult,
            Route route,
            Double headingDeg,
            Boolean flowAligned,
            HaulBuilderOptions options
            )
        {
            // KPI calculations mirror CLI printouts so API and CLI users see identical operational metrics.
            TimeParams Params = new TimeParams(
                options.LoadingSpeedMph,
                options.HaulSpeedMph,
                options.DumpingSpeedMph,
                options.ReturnSpeedMph,
                options.AccelTimeS,
                options.DecelTimeS,
                options.FieldEfficiency);
            TimeResult Time = Timemodel.EstimateTime(grid, route, Params);
            result.TimeResult = Time;

            Double MovedBcy = 0.0;
            for (Int32 Iter = 0; Iter < passResult.Passes.Count; Iter += 1)
            {
                Pass Pass = passResult.Passes[Iter];
                MovedBcy += Pass.LoadBcy;
            }

            Int32 Valid = 0;
            for (Int32 By = 0; By < grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < grid.Width; Bx += 1)
                {
                    if (grid.Valid[By, Bx]) Valid += 1;
                }
            }

            Double AreaAc = ((Valid * grid.AreaM2) / M2_PER_ACRE);
            Double TotalH = result.DetailedGuidanceTime != null
                ? result.DetailedGuidanceTime.TotalSeconds / 3600.0
                : Time.TotalTimeS / 3600.0;
            Double MoveRate = (TotalH > 0.0) ? (MovedBcy / TotalH) : 0.0;
            Double AvgMoveYd = passResult.Passes.Count > 0 ? ((Time.HaulingDistM / passResult.Passes.Count) / METERS_PER_YARD) : 0.0;
            result.Metrics = new HaulAnalysisMetrics
            {
                ValidAreas = Valid,
                FieldAreaAcres = AreaAc,
                MoveVolumeBcy = MovedBcy,
                TotalHours = TotalH,
                MoveRateBcyPerHour = MoveRate,
                AreaRateAcresPerHour = TotalH > 0.0 ? (AreaAc / TotalH) : 0.0,
                AverageMoveDistanceYards = AvgMoveYd,
                HeadingDeg = flowAligned ? null : headingDeg,
                FlowAligned = flowAligned,
            };
        }

        private static void BuildAnimations
            (
            HaulBuilderResult result,
            Grid grid,
            Route route,
            HaulBuilderOptions options
            )
        {
            // Base leveling animation can be emitted as per-frame PNGs and/or one GIF.
            Boolean NeedAnimation = ((options.AnimateDirPath is not null) || (options.AnimateGifPath is not null));
            if (!NeedAnimation) return;
            Animate.AnimationPayload Payload = Animate.AnimateLeveling(grid, route.OrderedPasses, options.AnimateStride, options.AnimatePixelsPerBin, 80, options.AnimateMode);
            if (Payload.GifBytes is not null) result.AnimationGifBytes = Payload.GifBytes;
            for (Int32 Iter = 0; Iter < Payload.FramePngBytes.Count; Iter += 1)
            {
                result.PngBytesByName[String.Format(CultureInfo.InvariantCulture, "animation_frame_{0:D5}", Iter)] = Payload.FramePngBytes[Iter];
            }
        }

        private static void BuildPivot
            (
            HaulBuilderResult result,
            Grid grid,
            Route route,
            ObstacleField obstacles,
            HaulBuilderOptions options
            )
        {
            // Pivot stage is optional and includes both static schedule artifacts and optional animation.
            if (!options.Pivot)
            {
                return;
            }

            Pivot.Grid PivotGrid = ToPivotGrid(grid);
            List<Pivot.Pass> PivotPasses = ToPivotPasses(route.OrderedPasses);
            Pivot.ObstacleField PivotObstacles = ToPivotObstacleField(obstacles);
            Pivot.PivotGeometry Geometry = Pivot.ComputePivotGeometry(PivotGrid, options.PivotWheelbaseFt, options.PivotClearanceFt, null);
            Pivot.PivotPlan Plan = Pivot.PlanPivot(PivotGrid, PivotPasses, Geometry, 720, options.PivotMoveCost);
            Dictionary<Int32, Pivot.ObstacleField> Arm = Pivot.BuildArmObstacles(Geometry, Plan, PivotObstacles, ((options.EquipmentWidthFt * Constants.FEET_TO_METERS) / 2.0));
            result.PivotGeometry = Geometry;
            result.PivotPlan = Plan;
            result.ArmObstacles = Arm;
            result.PivotCsvData = Pivot.WritePivotCsv(Plan);
            result.CsvTextByName["pivot"] = BuildPivotCsv(result.PivotCsvData);

            String PivotDir = Path.Combine(Path.GetTempPath(), "haulbuilder_pivot_png_" + Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(PivotDir);
            try
            {
                List<String> Paths = Pivot.RenderPivotPngs(PivotGrid, PivotPasses, Plan, PivotDir, options.PivotMode, options.PivotPixelsPerBin);
                foreach (String PathItem in Paths)
                {
                    String Name = Path.GetFileNameWithoutExtension(PathItem);
                    result.PivotPngBytesByName[Name] = File.ReadAllBytes(PathItem);
                }
            }
            finally
            {
                try
                {
                    Directory.Delete(PivotDir, true);
                }
                catch (IOException)
                {
                    // Best-effort cleanup only.
                }
                catch (UnauthorizedAccessException)
                {
                    // Best-effort cleanup only.
                }
            }

            if (!options.NoPivotGif || (options.PivotFramesPath is not null))
            {
                Animate.AnimationPayload Payload = Animate.AnimatePivotLeveling(
                    grid,
                    route.OrderedPasses,
                    Plan,
                    options.PivotAnimationStride,
                    options.PivotPixelsPerBin,
                    options.PivotMode);
                result.PivotAnimationGifBytes = Payload.GifBytes;
                for (Int32 Iter = 0; Iter < Payload.FramePngBytes.Count; Iter += 1)
                {
                    result.PivotPngBytesByName[String.Format(CultureInfo.InvariantCulture, "pivot_animation_frame_{0:D5}", Iter)] = Payload.FramePngBytes[Iter];
                }
            }
        }

        private static Pivot.Grid ToPivotGrid
            (
            Grid grid
            )
        {
            // Adapt top-level grid into Pivot's nested grid DTO.
            return new Pivot.Grid(grid.GridSizeM, grid.MinX, grid.MinY, grid.Zone, grid.IsNorthern, grid.Existing, grid.Target, grid.Valid);
        }

        private static List<Pivot.Pass> ToPivotPasses
            (
            IReadOnlyList<Pass> passes
            )
        {
            // Convert pass tuple cells into Pivot.PassCell sequences.
            List<Pivot.Pass> Out = new List<Pivot.Pass>();
            for (Int32 Iter = 0; Iter < passes.Count; Iter += 1)
            {
                List<Pivot.PassCell> Cut = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in passes[Iter].CutCells)
                {
                    Cut.Add(new Pivot.PassCell(Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM));
                }

                List<Pivot.PassCell> Fill = new List<Pivot.PassCell>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in passes[Iter].FillCells)
                {
                    Fill.Add(new Pivot.PassCell(Entry.Cell.Bx, Entry.Cell.By, Entry.DepthM));
                }

                Out.Add(new Pivot.Pass(Cut, Fill, passes[Iter].CutEndXy, passes[Iter].FillStartXy));
            }

            return Out;
        }

        private static Pivot.ObstacleField ToPivotObstacleField
            (
            ObstacleField obstacles
            )
        {
            // Rebuild Pivot obstacle geometry from top-level obstacle zones.
            if (!obstacles.Active)
            {
                return Pivot.ObstacleField.EMPTY_OBSTACLES;
            }

            List<Pivot.ExclusionZone> Zones = new List<Pivot.ExclusionZone>();
            foreach (ExclusionZone Zone in obstacles.Zones)
            {
                Zones.Add(new Pivot.ExclusionZone(Zone.Ring, Zone.BufferRing));
            }

            return Pivot.ObstacleField.FromZones(Zones);
        }

        private static List<List<(Double Lat, Double Lon)>> ParsePolygonSpecs
            (
            IReadOnlyList<String> specs
            )
        {
            // Parsing follows the CLI grammar: repeated flag, each as lat,lon pairs separated by semicolons.
            List<List<(Double Lat, Double Lon)>> Rings = new List<List<(Double Lat, Double Lon)>>();
            for (Int32 Iter = 0; Iter < specs.Count; Iter += 1)
            {
                String Spec = specs[Iter];
                List<(Double Lat, Double Lon)> Ring = new List<(Double Lat, Double Lon)>();
                String[] Chunks = Spec.Replace("\n", ";", StringComparison.Ordinal).Split(';', StringSplitOptions.RemoveEmptyEntries);
                for (Int32 C = 0; C < Chunks.Length; C += 1)
                {
                    String[] Parts = Chunks[C].Split(',', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);
                    if (Parts.Length != 2)
                    {
                        throw new ArgumentException($"Invalid exclusion vertex '{Chunks[C]}'. Expected 'lat,lon'.");
                    }

                    Ring.Add((Double.Parse(Parts[0], CultureInfo.InvariantCulture), Double.Parse(Parts[1], CultureInfo.InvariantCulture)));
                }

                if (Ring.Count < 3) throw new ArgumentException("Exclusion polygon needs at least 3 vertices.");
                Rings.Add(Ring);
            }

            return Rings;
        }

        private static String BuildZoneCsv
            (
            IReadOnlyList<ZoneStep> steps
            )
        {
            // CSV building in-memory replaces filesystem writes while keeping parity with Python column order.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("work_order,zone_id,compass,loads,moved_bcy,easting_m,northing_m,lat,lon,first_pass,last_pass,mean_pass");
            for (Int32 Iter = 0; Iter < steps.Count; Iter += 1)
            {
                dynamic S = steps[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3},{4:F1},{5:F2},{6:F2},{7:F7},{8:F7},{9},{10},{11:F0}", (Int32)S.WorkOrder, (Int32)S.ZoneId, (String)S.Compass, (Int32)S.NLoads, (Double)S.TotalBcy, (Double)S.Easting, (Double)S.Northing, (Double)S.Lat, (Double)S.Lon, (Int32)S.FirstPass, (Int32)S.LastPass, (Double)S.MeanPass);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        private static String BuildPairsCsv
            (
            IReadOnlyList<CutFillPair> pairs,
            AgdField field
            )
        {
            // Pair CSV mirrors the Python writer while remaining pure in-memory.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("order,load_bcy,haul_m,total_cut_bcy_before,total_fill_bcy_before,route_time_s,cut_start_lat,cut_start_lon,cut_end_lat,cut_end_lon,fill_start_lat,fill_start_lon,fill_end_lat,fill_end_lon");
            for (Int32 Iter = 0; Iter < pairs.Count; Iter += 1)
            {
                CutFillPair P = pairs[Iter];
                (Double CsLat, Double CsLon) = Agd.LatLonFromUtm(P.CutStartXy.X, P.CutStartXy.Y, field.Zone, field.IsNorthern);
                (Double CeLat, Double CeLon) = Agd.LatLonFromUtm(P.CutEndXy.X, P.CutEndXy.Y, field.Zone, field.IsNorthern);
                (Double FsLat, Double FsLon) = Agd.LatLonFromUtm(P.FillStartXy.X, P.FillStartXy.Y, field.Zone, field.IsNorthern);
                (Double FeLat, Double FeLon) = Agd.LatLonFromUtm(P.FillEndXy.X, P.FillEndXy.Y, field.Zone, field.IsNorthern);
                Sb.AppendFormat(
                    CultureInfo.InvariantCulture,
                    "{0},{1:F2},{2:F1},{3:F2},{4:F2},{5:F1},{6:F7},{7:F7},{8:F7},{9:F7},{10:F7},{11:F7},{12:F7},{13:F7}",
                    P.Order,
                    P.LoadBcy,
                    P.HaulDistanceM,
                    P.TotalCutBcyBefore,
                    P.TotalFillBcyBefore,
                    P.RouteTimeS,
                    CsLat,
                    CsLon,
                    CeLat,
                    CeLon,
                    FsLat,
                    FsLon,
                    FeLat,
                    FeLon);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        private static String BuildGuidanceCsv
            (
            IReadOnlyList<GuidancePoint> points
            )
        {
            // Minimal guidance export covers the core turn-by-turn fields consumed by operator tools.
            StringBuilder Sb = new StringBuilder();
            Sb.AppendLine("seq,pass_no,action,x,y,lat,lon,depth_mm,heading_deg,blade,instruction");
            for (Int32 Iter = 0; Iter < points.Count; Iter += 1)
            {
                dynamic P = points[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3:F3},{4:F3},{5:F7},{6:F7},{7:F1},{8:F1},{9},{10}", (Int32)P.Seq, (Int32)P.PassNo, (String)P.Action, (Double)P.X, (Double)P.Y, (Double)P.Lat, (Double)P.Lon, (Double)P.DepthMm, (Double)P.HeadingDeg, (String)P.Blade, (String)P.Instruction);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        private static String BuildPivotCsv
            (
            Pivot.PivotCsvData data
            )
        {
            // Pivot schedule is serialized from typed rows so callers can consume either table or object model.
            StringBuilder Sb = new StringBuilder();
            foreach (KeyValuePair<String, String> Pair in data.Metadata)
            {
                Sb.Append('#');
                Sb.Append(Pair.Key);
                Sb.Append('=');
                Sb.AppendLine(Pair.Value);
            }

            Sb.AppendLine("position,first_pass,last_pass,pass_count,bearing_deg,cardinal,move_deg,tip_lat,tip_lon,conflict_count");
            for (Int32 Iter = 0; Iter < data.Rows.Count; Iter += 1)
            {
                Pivot.PivotCsvRow Row = data.Rows[Iter];
                Sb.AppendFormat(CultureInfo.InvariantCulture, "{0},{1},{2},{3},{4:F2},{5},{6:F2},{7:F7},{8:F7},{9}", Row.Position, Row.FirstPass, Row.LastPass, Row.PassCount, Row.BearingDeg, Row.Cardinal, Row.MoveDeg, Row.TipLat, Row.TipLon, Row.ConflictCount);
                Sb.AppendLine();
            }

            return Sb.ToString();
        }

        private static HaulBuilderOptions CloneOptions
            (
            HaulBuilderOptions options
            )
        {
            // Cloning ensures the result captures immutable run configuration.
            HaulBuilderOptions Copy = new HaulBuilderOptions
            {
                AgdPath = options.AgdPath,
                OutputPngPath = options.OutputPngPath,
                GridSizeFt = options.GridSizeFt,
                ToleranceMm = options.ToleranceMm,
                ScraperCapacityLcy = options.ScraperCapacityLcy,
                SwellFactor = options.SwellFactor,
                CutFillRatio = options.CutFillRatio,
                MaxDepthMm = options.MaxDepthMm,
                BladeWidthMm = options.BladeWidthMm,
                HeadingDeg = options.HeadingDeg,
                NoFlowAligned = options.NoFlowAligned,
                HeadingStrength = options.HeadingStrength,
                StripBias = options.StripBias,
                MaxDropHopFt = options.MaxDropHopFt,
                MinLoadFillPct = options.MinLoadFillPct,
                NoKnotRepair = options.NoKnotRepair,
                FieldEfficiency = options.FieldEfficiency,
                LoadingSpeedMph = options.LoadingSpeedMph,
                HaulSpeedMph = options.HaulSpeedMph,
                DumpingSpeedMph = options.DumpingSpeedMph,
                ReturnSpeedMph = options.ReturnSpeedMph,
                AccelTimeS = options.AccelTimeS,
                DecelTimeS = options.DecelTimeS,
                TransportMode = options.TransportMode,
                BalanceMode = options.BalanceMode,
                PixelsPerBin = options.PixelsPerBin,
                ArrowStride = options.ArrowStride,
                Zones = options.Zones,
                SequenceCsvPath = options.SequenceCsvPath,
                PairsCsvPath = options.PairsCsvPath,
                PairsPngPath = options.PairsPngPath,
                GuidanceCsvPath = options.GuidanceCsvPath,
                GuidancePngPath = options.GuidancePngPath,
                LaneOverlap = options.LaneOverlap,
                TurnSpeedMph = options.TurnSpeedMph,
                GradeSpeedMph = options.GradeSpeedMph,
                TurningCircleFt = options.TurningCircleFt,
                EquipmentLengthFt = options.EquipmentLengthFt,
                TurnThresholdDeg = options.TurnThresholdDeg,
                NoGradeReturn = options.NoGradeReturn,
                NoRouteSmoothing = options.NoRouteSmoothing,
                ReorderWindow = options.ReorderWindow,
                NoDirectionAware = options.NoDirectionAware,
                AnimateDirPath = options.AnimateDirPath,
                AnimateGifPath = options.AnimateGifPath,
                AnimateStride = options.AnimateStride,
                AnimatePixelsPerBin = options.AnimatePixelsPerBin,
                AnimateMode = options.AnimateMode,
                Pivot = options.Pivot,
                PivotCenter = options.PivotCenter,
                PivotWheelbaseFt = options.PivotWheelbaseFt,
                PivotClearanceFt = options.PivotClearanceFt,
                PivotMoveCost = options.PivotMoveCost,
                PivotDirPath = options.PivotDirPath,
                PivotCsvPath = options.PivotCsvPath,
                PivotMode = options.PivotMode,
                PivotPixelsPerBin = options.PivotPixelsPerBin,
                PivotGifPath = options.PivotGifPath,
                NoPivotGif = options.NoPivotGif,
                PivotFramesPath = options.PivotFramesPath,
                PivotAnimationStride = options.PivotAnimationStride,
                EquipmentWidthFt = options.EquipmentWidthFt,
                EnableHaulOptimization = options.EnableHaulOptimization,
                HaulOptimizationAxis = options.HaulOptimizationAxis,
                HaulOptimizationMaxRunsPerKind = options.HaulOptimizationMaxRunsPerKind,
                HaulOptimizationFlowPenaltyWeight = options.HaulOptimizationFlowPenaltyWeight,
                HaulOptimizationDeferEdgeWork = options.HaulOptimizationDeferEdgeWork,
                HaulOptimizationPairRefinementRounds = options.HaulOptimizationPairRefinementRounds,
                HaulOptimizationPairMetric = options.HaulOptimizationPairMetric,
                HaulOptimizationReorderWindow = options.HaulOptimizationReorderWindow,
                HaulOptimizationMaximumCompactOmissions = options.HaulOptimizationMaximumCompactOmissions,
                HaulOptimizationCompactOmissionPriority = options.HaulOptimizationCompactOmissionPriority,
                OmitIsolatedEdgePatches = options.OmitIsolatedEdgePatches,
                MaxIsolatedEdgePatchCells = options.MaxIsolatedEdgePatchCells,
                MaxIsolatedEdgePatchVolumeBcy = options.MaxIsolatedEdgePatchVolumeBcy,
                MaxTotalIsolatedEdgePatchOmissionBcy = options.MaxTotalIsolatedEdgePatchOmissionBcy,
                MaxTotalIsolatedEdgePatchOmissionPercent = options.MaxTotalIsolatedEdgePatchOmissionPercent,
                NoPlan = options.NoPlan,
                PairSampleCount = options.PairSampleCount,
                Seed = options.Seed,
            };
            Copy.ExcludePolygonSpecs.AddRange(options.ExcludePolygonSpecs);
            return Copy;
        }

        private static void ValidateEdgePatchOptions
            (
            HaulBuilderOptions options
            )
        {
            if (!options.OmitIsolatedEdgePatches) return;
            if (options.MaxIsolatedEdgePatchCells < 1)
            {
                throw new ArgumentException(
                    "MaxIsolatedEdgePatchCells must be at least 1 when edge-patch omission is enabled.",
                    nameof(options));
            }
            if (!Double.IsFinite(options.MaxIsolatedEdgePatchVolumeBcy)
                || (options.MaxIsolatedEdgePatchVolumeBcy < 0.0))
            {
                throw new ArgumentException(
                    "MaxIsolatedEdgePatchVolumeBcy must be finite and non-negative.",
                    nameof(options));
            }
            if (!Double.IsFinite(options.MaxTotalIsolatedEdgePatchOmissionBcy)
                || (options.MaxTotalIsolatedEdgePatchOmissionBcy < 0.0))
            {
                throw new ArgumentException(
                    "MaxTotalIsolatedEdgePatchOmissionBcy must be finite and non-negative.",
                    nameof(options));
            }
            if (!Double.IsFinite(options.MaxTotalIsolatedEdgePatchOmissionPercent)
                || (options.MaxTotalIsolatedEdgePatchOmissionPercent < 0.0)
                || (options.MaxTotalIsolatedEdgePatchOmissionPercent > 100.0))
            {
                throw new ArgumentException(
                    "MaxTotalIsolatedEdgePatchOmissionPercent must be between 0 and 100.",
                    nameof(options));
            }
        }

        private static void ValidateHaulOptimizationOptions
            (
            HaulBuilderOptions options
            )
        {
            if (!options.EnableHaulOptimization) return;
            if (options.HaulOptimizationMaxRunsPerKind <= 0)
            {
                throw new ArgumentException(
                    "HaulOptimizationMaxRunsPerKind must be positive.",
                    nameof(options));
            }
            if (!Double.IsFinite(options.HaulOptimizationFlowPenaltyWeight)
                || (options.HaulOptimizationFlowPenaltyWeight < 0.0))
            {
                throw new ArgumentException(
                    "HaulOptimizationFlowPenaltyWeight must be finite and non-negative.",
                    nameof(options));
            }
            if (options.HaulOptimizationPairRefinementRounds < 0)
            {
                throw new ArgumentException(
                    "HaulOptimizationPairRefinementRounds must be non-negative.",
                    nameof(options));
            }
            if (!Enum.IsDefined(options.HaulOptimizationPairMetric))
            {
                throw new ArgumentException(
                    "HaulOptimizationPairMetric is invalid.",
                    nameof(options));
            }
            if (options.HaulOptimizationReorderWindow < 0)
            {
                throw new ArgumentException(
                    "HaulOptimizationReorderWindow must be non-negative.",
                    nameof(options));
            }
            if (options.HaulOptimizationMaximumCompactOmissions < 0)
            {
                throw new ArgumentException(
                    "HaulOptimizationMaximumCompactOmissions must be non-negative.",
                    nameof(options));
            }
            if (!Enum.IsDefined(
                options.HaulOptimizationCompactOmissionPriority))
            {
                throw new ArgumentException(
                    "HaulOptimizationCompactOmissionPriority is invalid.",
                    nameof(options));
            }
        }

        /// <summary>Retains stable reasons for work deferred before compact preflight.</summary>
        private static IReadOnlyList<HaulDeferredWork>
            PreserveExistingDeferredReasons
            (
            IReadOnlyList<HaulDeferredWork> Current,
            IReadOnlyList<HaulDeferredWork> Existing
            )
        {
            Dictionary<(
                HaulLaneBuilder.WorkKind Kind,
                (Int32 Bx, Int32 By) Cell,
                Int32 LayerIndex), String> Reasons = Existing.ToDictionary(
                    Entry => (Entry.Kind, Entry.Cell, Entry.LayerIndex),
                    Entry => Entry.Reason);
            return Current.Select(Entry => Reasons.TryGetValue(
                    (Entry.Kind, Entry.Cell, Entry.LayerIndex),
                    out String? Reason)
                ? new HaulDeferredWork(
                    Entry.Kind,
                    Entry.Cell,
                    Entry.LayerIndex,
                    Entry.DepthM,
                    Entry.VolumeBcy,
                    Reason)
                : Entry).ToList();
        }
    }
}
