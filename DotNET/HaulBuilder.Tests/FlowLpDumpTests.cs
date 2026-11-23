namespace HaulBuilder.Tests
{
    /// <summary>
    /// Writes the Shop 2 flow-stage residual and haul log for an offline transport bound.
    /// This test does not change the planner. It solves only when FLOW_LP_DUMP=1,
    /// so a normal suite run does not repeat the Shop 2 search.
    /// </summary>
    public sealed class FlowLpDumpTests
    {
        /// <summary>File magic. Python rejects a dump that was not written by this test.</summary>
        private const String MAGIC = "FLW1";

        /// <summary>
        /// Solves Shop 2 the same way as the field picture and writes the flow-stage dump.
        /// </summary>
        [Fact]
        public void Shop2FlowStageIsDumpedForTheLowerBound()
        {
            // Opt-in. The bound is analysis, and the suite must not pay for a second Shop 2 solve.
            String? Flag = Environment.GetEnvironmentVariable("FLOW_LP_DUMP");
            if (!String.Equals(Flag, "1", StringComparison.Ordinal))
            {
                return;
            }

            String AgdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            HaulBuilderOptions Options = new HaulBuilderOptions
            {
                AgdPath = AgdPath,
                SkipGuidance = true,
                IncludeGuidanceCsv = false,
            };
            Options.Optimization.EnableCycleChainOptimization = false;
            Options.Optimization.PairRefinementRounds = 0;
            HaulBuilderResult Result = new HaulBuilderEngine().Run(Options);
            Assert.NotNull(Result.PassResult);
            Assert.NotNull(Result.Grid);
            Assert.True(Result.ClusterModeStartsAtRoute > 1);
            Assert.True(Result.PassResult.Passes.Count >= Result.ClusterModeStartsAtRoute);

            Grid Grid = Result.Grid;
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, Options.ToleranceMm);
            String DumpPath = Path.Combine(Path.GetTempPath(), "flow-lp-shop2.bin");
            WriteDump(DumpPath, Grid, Ledger, Result.PassResult.Passes, Result.ClusterModeStartsAtRoute);
            Console.WriteLine("flow-lp dump: {0}", DumpPath);
            Console.WriteLine("flow hauls: {0}", Result.ClusterModeStartsAtRoute - 1);
            Console.WriteLine(
                "cell_m {0:R} fill_to_bank {1:R} area {2:R}",
                Grid.GridSizeM,
                Grid.Compaction,
                Grid.AreaM2);
            WriteStageClock(Result, Options);
        }

        /// <summary>
        /// Writes found-order loads and the cycle hours of each stage.
        /// Found order matches the stage boundaries. OrderedPasses does not.
        /// </summary>
        /// <param name="Result">The Shop 2 run.</param>
        /// <param name="Options">Speeds and efficiency used for the cycle clock.</param>
        private static void WriteStageClock
            (
            HaulBuilderResult Result,
            HaulBuilderOptions Options
            )
        {
            Grid Grid = Result.Grid!;
            IReadOnlyList<Pass> Passes = Result.PassResult!.Passes;
            String LoadsPath = Path.Combine(Path.GetTempPath(), "window3-loads.txt");
            using (StreamWriter Loads = new StreamWriter(LoadsPath))
            {
                foreach (Pass Pass in Passes)
                {
                    Loads.WriteLine(Pass.LoadBcy.ToString("R", System.Globalization.CultureInfo.InvariantCulture));
                }
            }

            TimeParams Time = new TimeParams(
                Options.CutSpeedMph,
                Options.LoadedTravelSpeedMph,
                Options.FillSpeedMph,
                Options.UnloadedTravelSpeedMph,
                Options.AccelerationTimeS,
                Options.DecelerationTimeS,
                Options.FieldEfficiency);
            Double FlowS = 0.0;
            Double ClusterS = 0.0;
            Double ThinS = 0.0;
            Double CorrectS = 0.0;
            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                Double Seconds = TimeModel.EstimatePassTime(Grid, Passes[Index], Time);
                Int32 Route = Index + 1;
                if (Route < Result.ClusterModeStartsAtRoute)
                {
                    FlowS += Seconds;
                }
                else if (Route < Result.ThinCutStartsAtRoute)
                {
                    ClusterS += Seconds;
                }
                else if (Route < Result.CorrectingStartsAtRoute)
                {
                    ThinS += Seconds;
                }
                else
                {
                    CorrectS += Seconds;
                }
            }

            String ClockPath = Path.Combine(Path.GetTempPath(), "window3-stages.txt");
            File.WriteAllLines(
                ClockPath,
                new[]
                {
                    "routes " + Passes.Count.ToString(System.Globalization.CultureInfo.InvariantCulture),
                    "cluster " + Result.ClusterModeStartsAtRoute.ToString(System.Globalization.CultureInfo.InvariantCulture),
                    "thin " + Result.ThinCutStartsAtRoute.ToString(System.Globalization.CultureInfo.InvariantCulture),
                    "correcting " + Result.CorrectingStartsAtRoute.ToString(System.Globalization.CultureInfo.InvariantCulture),
                    "flow_hours " + (FlowS / 3600.0).ToString("R", System.Globalization.CultureInfo.InvariantCulture),
                    "cluster_hours " + (ClusterS / 3600.0).ToString("R", System.Globalization.CultureInfo.InvariantCulture),
                    "thin_hours " + (ThinS / 3600.0).ToString("R", System.Globalization.CultureInfo.InvariantCulture),
                    "correcting_pass_hours " + (CorrectS / 3600.0).ToString("R", System.Globalization.CultureInfo.InvariantCulture),
                    "window " + Options.Optimization.ClusterModeAverageWindow.ToString(System.Globalization.CultureInfo.InvariantCulture),
                });
            Console.WriteLine("stage clock: {0}", ClockPath);
        }

        /// <summary>
        /// Writes R0 and flow hauls 1..cluster-1. Later stages are not in the file.
        /// </summary>
        /// <param name="DumpPath">Destination binary path.</param>
        /// <param name="Grid">Shop 2 grid. Cell centres and the valid mask come from here.</param>
        /// <param name="Ledger">Residual before haul 1. Depths outside the ±tolerance band are the full delta.</param>
        /// <param name="Passes">Found-order passes. Index 0 is haul 1.</param>
        /// <param name="ClusterRoute">Route number of the first cluster haul.</param>
        private static void WriteDump
            (
            String DumpPath,
            Grid Grid,
            HaulWorkLedger Ledger,
            IReadOnlyList<Pass> Passes,
            Int32 ClusterRoute
            )
        {
            Int32 FlowHauls = ClusterRoute - 1;
            Double CutPerM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPerM = CutPerM * Grid.Compaction;
            using BinaryWriter Writer = new BinaryWriter(File.Create(DumpPath));
            Writer.Write(MAGIC.ToCharArray());
            Writer.Write(Grid.Height);
            Writer.Write(Grid.Width);
            Writer.Write(Grid.GridSizeM);
            Writer.Write(Grid.Compaction);
            Writer.Write(Constants.M3_TO_CY);
            Writer.Write(1.0);
            Writer.Write(FlowHauls);
            Writer.Write(ClusterRoute);
            for (Int32 Row = 0; Row < Grid.Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Grid.Width; Col += 1)
                {
                    Boolean Valid = Grid.Valid[Row, Col];
                    Writer.Write(Valid);
                    Writer.Write(Grid.CenterX[Row, Col]);
                    Writer.Write(Grid.CenterY[Row, Col]);
                    if (!Valid)
                    {
                        Writer.Write(Double.NaN);
                        continue;
                    }

                    // Residual millimetres. Positive is still to cut. Negative is still to fill.
                    Double ResidualM = Ledger.RequiredCutDepthM[Row, Col] - Ledger.RequiredFillDepthM[Row, Col];
                    Writer.Write(ResidualM * 1000.0);
                }
            }

            Writer.Write(FlowHauls);
            for (Int32 Index = 0; Index < FlowHauls; Index += 1)
            {
                Pass Pass = Passes[Index];
                Writer.Write(Index + 1);
                Writer.Write(Pass.LoadBcy);
                Writer.Write(Pass.CutCells.Count);
                Writer.Write(Pass.FillCells.Count);
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Pass.CutCells)
                {
                    Writer.Write(Lift.Cell.Bx);
                    Writer.Write(Lift.Cell.By);
                    Writer.Write(Lift.DepthM * CutPerM);
                }

                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Lift in Pass.FillCells)
                {
                    Writer.Write(Lift.Cell.Bx);
                    Writer.Write(Lift.Cell.By);
                    // Fill depth is a hole. Bank volume multiplies by the compaction factor.
                    Writer.Write(Lift.DepthM * FillPerM);
                }
            }

            Writer.Flush();
        }
    }
}
