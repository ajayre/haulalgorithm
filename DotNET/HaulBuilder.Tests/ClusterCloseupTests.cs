namespace HaulBuilder.Tests
{
    /// <summary>
    /// Close-up sheets of the first cluster routes on Shop 2.
    /// Each route is two frames: the cut patch, then the fill patch, on the ground still left.
    /// </summary>
    public sealed class ClusterCloseupTests
    {
        /// <summary>How many cluster routes to draw, starting at the handoff.</summary>
        private const Int32 ROUTE_COUNT = 10;

        /// <summary>Pixels per cell. The frame is only the cut or the fill, so each cell stays readable.</summary>
        private const Int32 PIXELS_PER_CELL = 48;

        /// <summary>Cells of ground kept around the work patch.</summary>
        private const Int32 PAD_CELLS = 3;

        /// <summary>
        /// Draws the first 10 cluster routes as two tight sheets each: the cut, then the fill.
        /// Each sheet outlines the piece, draws the haul heading, and draws one centreline per band that moved.
        /// The cluster loop stops after these 10 trips. The 500-trip cap is left alone.
        /// </summary>
        [Fact]
        public void Shop2FirstClusterRoutesAreDrawnClose()
        {
            String AgdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = AgdPath,
                SkipGuidance = true,
                IncludeGuidanceCsv = false,
            };
            AgdField Field = Agd.Parse(AgdPath);
            Grid Grid = GridMath.BuildGrid(
                Field,
                Options.GridSizeFt,
                Options.SwellFactor,
                Options.CutFillRatio,
                Options.MaxDepthMm);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, Options.ToleranceMm);
            Double BowlBcy = Options.ScraperCapacityLcy / Options.SwellFactor;
            HaulOptimizationResult Result = FlowRibbonSearch.Optimize(
                Grid,
                Flow,
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                BowlBcy,
                true,
                null,
                false,
                Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                Options.MinimumTurningDiameterFt,
                Options.Optimization.ClusterModeAverageWindow,
                Options.Optimization.ClusterModeLoadFraction,
                ROUTE_COUNT);
            Assert.Equal(HaulSearchState.Clusters, Result.State);
            Int32 First = Result.ClusterModeStartsAtRoute;
            Assert.True(First > 1);
            List<Pass> Passes = Result.PassResult.Passes;
            Assert.True(Passes.Count >= First - 1 + ROUTE_COUNT);
            Double[,] Surface = Grid.Delta();
            for (Int32 Index = 0; Index < First - 1; Index += 1)
            {
                ApplyPass(Surface, Passes[Index]);
            }

            String Folder = Path.GetFullPath(Path.Combine(
                AppContext.BaseDirectory,
                "..",
                "..",
                "..",
                "..",
                "Fixtures",
                "FlowRibbonProof",
                "shop2-2026-11-20-found",
                "cluster-closeups"));
            Directory.CreateDirectory(Folder);
            // The first pass wrote one wide sheet per route. Those span the haul and hide the work.
            foreach (String Old in Directory.GetFiles(Folder, "route-????.png"))
            {
                File.Delete(Old);
            }

            for (Int32 Offset = 0; Offset < ROUTE_COUNT; Offset += 1)
            {
                Int32 Index = First - 1 + Offset;
                Pass Pass = Passes[Index];
                Int32 Route = Index + 1;
                WriteSide(Folder, Route, "cut", Grid, Pass, Surface);
                WriteSide(Folder, Route, "fill", Grid, Pass, Surface);
                ApplyPass(Surface, Pass);
                Console.WriteLine(
                    "cluster close-up {0}: route {1} P {2} Q {3} load {4:F2} bcy cut passes {5} fill passes {6} t_cut {7:F0}s t_fill {8:F0}s",
                    Offset + 1,
                    Route,
                    Pass.AreaCutPiece,
                    Pass.AreaFillPiece,
                    Pass.LoadBcy,
                    Pass.AreaCutPasses,
                    Pass.AreaFillPasses,
                    Pass.StripLoadTimeS ?? 0.0,
                    Pass.StripSpreadTimeS ?? 0.0);
            }
        }

        /// <summary>
        /// Takes one haul off the residual. Cut lowers a cell. Fill raises it.
        /// </summary>
        /// <param name="Surface">Existing-minus-target depth, in metres.</param>
        /// <param name="Pass">The haul to apply.</param>
        private static void ApplyPass
            (
            Double[,] Surface,
            Pass Pass
            )
        {
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Pass.CutCells)
            {
                Surface[Lift.Cell.By, Lift.Cell.Bx] -= Lift.DepthM;
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Pass.FillCells)
            {
                Surface[Lift.Cell.By, Lift.Cell.Bx] += Lift.DepthM;
            }
        }

        /// <summary>
        /// Writes one tight sheet. The piece is outlined and the pass lines are drawn on it.
        /// </summary>
        /// <param name="Folder">Destination folder.</param>
        /// <param name="Route">1-based found-order route number.</param>
        /// <param name="Side">File tag, cut or fill.</param>
        /// <param name="Grid">Field grid.</param>
        /// <param name="Pass">The trip.</param>
        /// <param name="Surface">Ground still left, in metres.</param>
        private static void WriteSide
            (
            String Folder,
            Int32 Route,
            String Side,
            Grid Grid,
            Pass Pass,
            Double[,] Surface
            )
        {
            Boolean Fill = Side == "fill";
            Byte[] Png = Render.RenderAreaPassPng(
                Grid,
                Fill ? Pass.FillCells : Pass.CutCells,
                Fill ? Pass.AreaFillOutline : Pass.AreaCutOutline,
                Pass.AreaDirX,
                Pass.AreaDirY,
                PIXELS_PER_CELL,
                PAD_CELLS,
                Fill,
                Surface);
            String PathName = Path.Combine(
                Folder,
                String.Format(System.Globalization.CultureInfo.InvariantCulture, "route-{0:0000}-{1}.png", Route, Side));
            try
            {
                File.WriteAllBytes(PathName, Png);
            }
            catch (IOException)
            {
                // The sheet is open in the editor. A sibling name still keeps the new picture.
                String Tag = "-" + Guid.NewGuid().ToString("N");
                PathName = Path.Combine(
                    Folder,
                    String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "route-{0:0000}-{1}{2}.png",
                        Route,
                        Side,
                        Tag));
                File.WriteAllBytes(PathName, Png);
                Console.WriteLine("locked, wrote {0}", PathName);
            }
            Assert.True(Png.Length > 1000);
        }

    }
}
