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

            PassResult PassResult = Passes.GeneratePasses(
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
            Result.PassResult = PassResult;

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

            if (!Opt.NoKnotRepair)
            {
                Swaths.ReprofileKnots(Route, Grid, Opt.BladeWidthMm, 3, HeadingDeg, Opt.LaneOverlap);
            }

            if (Opt.ReorderWindow > 0)
            {
                Routing.RefineRouteOrder(
                    Route,
                    Opt.ReorderWindow,
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

            return Result;
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
            Boolean NeedGuidance = ((options.GuidanceCsvPath is not null) || (options.GuidancePngPath is not null) || (options.PairSampleCount > 0));
            if (!NeedGuidance)
            {
                return;
            }

            GuidanceResult Guidance = Swaths.BuildGuidance(
                grid,
                field,
                route,
                options.BladeWidthMm,
                headingDeg,
                options.LaneOverlap,
                !options.NoGradeReturn,
                options.TurnThresholdDeg,
                !options.NoRouteSmoothing,
                (options.TurningCircleFt * 0.5 * Constants.FEET_TO_METERS),
                0.7,
                obstacles,
                null);
            result.Guidance = Guidance;
            result.GuidancePoints.AddRange(Guidance.Points);
            result.GuidanceTime = Swaths.EstimateGuidanceTime(
                Guidance,
                options.LoadingSpeedMph,
                options.HaulSpeedMph,
                options.DumpingSpeedMph,
                options.ReturnSpeedMph,
                options.GradeSpeedMph,
                options.TurnSpeedMph,
                options.TurningCircleFt,
                options.AccelTimeS,
                options.DecelTimeS,
                options.FieldEfficiency);

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
            Double TotalH = (Time.TotalTimeS / 3600.0);
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
                NoPlan = options.NoPlan,
                PairSampleCount = options.PairSampleCount,
                Seed = options.Seed,
            };
            Copy.ExcludePolygonSpecs.AddRange(options.ExcludePolygonSpecs);
            return Copy;
        }
    }
}
