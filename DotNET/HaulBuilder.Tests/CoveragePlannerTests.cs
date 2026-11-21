using System;
using System.Collections.Generic;
using System.Linq;

using HaulBuilder;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies the isolated straight-run coverage planner without enabling it in production.
    /// </summary>
    public sealed class CoveragePlannerTests
    {
        private const Double BLADE_WIDTH_M = 2.0;

        /// <summary>Minimum blade-down run length of the balance fixture, in metres.</summary>
        private const Double BALANCE_MINIMUM_WORK_LENGTH_M = 3.0;

        /// <summary>First column of the cut strip and the fill strip of the balance fixture.</summary>
        private const Int32 BALANCE_FIRST_COLUMN = 2;

        /// <summary>Last column of the cut strip and the fill strip of the balance fixture.</summary>
        private const Int32 BALANCE_LAST_COLUMN = 12;

        /// <summary>Row of the cut strip of the balance fixture.</summary>
        private const Int32 BALANCE_CUT_ROW = 5;

        /// <summary>Row of the fill strip of the balance fixture.</summary>
        private const Int32 BALANCE_FILL_ROW = 8;

        /// <summary>Row of the isolated fill cell that the minimum-length gate drops.</summary>
        private const Int32 BALANCE_SCRAP_ROW = 15;

        /// <summary>Depth of column zero of the balance fixture, in metres.</summary>
        private const Double BALANCE_BASE_DEPTH_M = 0.031;

        /// <summary>Depth added per column, so no two cells of a strip hold the same volume.</summary>
        private const Double BALANCE_DEPTH_STEP_M = 0.001;

        /// <summary>Largest difference between two volumes that count as equal, in BCY.</summary>
        private const Double BALANCE_VOLUME_TOLERANCE_BCY = 1e-9;

        /// <summary>Largest difference between two depths that count as equal, in metres.</summary>
        private const Double BALANCE_DEPTH_TOLERANCE_M = 1e-12;

        /// <summary>
        /// Verifies exact cell/depth conservation and confirms the caller's list remains unchanged.
        /// </summary>
        [Fact]
        public void Plan_PreservesExactAllocationAndInputOrder
            (
            )
        {
            Grid Grid = MakeGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells = MakeKnottedCells();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Before =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Cells);

            CoveragePlanner.PlanResult Plan = CoveragePlanner.Plan(
                Cells,
                Grid,
                BLADE_WIDTH_M,
                null,
                Grid.CellCenterXy(15, 15));

            Assert.True(CoveragePlanner.ConservesAllocation(Cells, Plan.OrderedCells));
            Assert.Equal(Before, Cells);
            Assert.Equal(Cells.Count, Plan.Runs.Sum(Run => Run.Cells.Count));
        }

        /// <summary>
        /// Verifies every assigned cell remains within half a blade width of its straight run.
        /// </summary>
        [Fact]
        public void Plan_ProducesStraightBladeWidthCoverageRuns
            (
            )
        {
            Grid Grid = MakeGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells = MakeKnottedCells();

            CoveragePlanner.PlanResult Plan = CoveragePlanner.Plan(
                Cells,
                Grid,
                BLADE_WIDTH_M,
                null,
                Grid.CellCenterXy(15, 15));

            Assert.NotEmpty(Plan.Runs);
            Assert.All(Plan.Runs, Run => Assert.NotEmpty(Run.Cells));
            Assert.True(Plan.MaximumLateralErrorM <= (Constants.HALF_SCALE * BLADE_WIDTH_M) + Constants.NUMERIC_COMPARISON_TOLERANCE);
            Assert.All(Plan.Runs, Run => Assert.True(Run.LengthM >= (0.98 * Grid.GridSizeM) - Constants.NUMERIC_COMPARISON_TOLERANCE));
            Assert.All(Plan.Runs, Run =>
            {
                Assert.True(Grid.ContainsWorldXy(Run.StartXy.X, Run.StartXy.Y));
                Assert.True(Grid.ContainsWorldXy(Run.EndXy.X, Run.EndXy.Y));
            });
        }

        /// <summary>
        /// Verifies repeated planning produces identical run ownership, direction, and metrics.
        /// </summary>
        [Fact]
        public void Plan_IsDeterministic
            (
            )
        {
            Grid Grid = MakeGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells = MakeKnottedCells();
            (Double X, Double Y) Target = Grid.CellCenterXy(15, 15);

            CoveragePlanner.PlanResult First = CoveragePlanner.Plan(
                Cells, Grid, BLADE_WIDTH_M, null, Target);
            CoveragePlanner.PlanResult Second = CoveragePlanner.Plan(
                Cells, Grid, BLADE_WIDTH_M, null, Target);

            Assert.Equal(First.HeadingDeg, Second.HeadingDeg, 9);
            Assert.Equal(First.ConnectorDistanceM, Second.ConnectorDistanceM, 9);
            Assert.Equal(RunSignature(First), RunSignature(Second));
        }

        /// <summary>
        /// Verifies merely evaluating an guidance plan cannot change existing guidance output.
        /// </summary>
        [Fact]
        public void Plan_DoesNotChangeDefaultGuidance
            (
            )
        {
            Grid Grid = MakeGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = MakeKnottedCells();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((14, 14), 0.02),
                    ((15, 14), 0.02),
                };
            Pass Pass = MakePass(Grid, CutCells, FillCells);
            Route Route = new Route(new List<Int32> { 0 }, new List<Pass> { Pass }, 0.0);
            AgdField Field = new AgdField(
                new List<AgdPoint>(), new List<AgdPoint>(), 12, true, 0.0, 0.0, 0);
            GuidanceResult Before = Swaths.BuildGuidance(
                Grid, Field, Route, BLADE_WIDTH_M * Constants.MILLIMETERS_PER_METER, 90.0);

            CoveragePlanner.PlanResult Ignored = CoveragePlanner.Plan(
                Pass.CutCells,
                Grid,
                BLADE_WIDTH_M,
                null,
                Pass.FillStartXy);
            GuidanceResult After = Swaths.BuildGuidance(
                Grid, Field, Route, BLADE_WIDTH_M * Constants.MILLIMETERS_PER_METER, 90.0);

            Assert.NotNull(Ignored);
            Assert.Equal(Before.Points.Count, After.Points.Count);
            for (Int32 Index = 0; Index < Before.Points.Count; Index += 1)
            {
                Assert.Equal(Before.Points[Index].Action, After.Points[Index].Action);
                Assert.Equal(Before.Points[Index].X, After.Points[Index].X, 9);
                Assert.Equal(Before.Points[Index].Y, After.Points[Index].Y, 9);
            }
        }

        /// <summary>
        /// Verifies boundary-adjacent work centerlines never cross invalid raw-field cells.
        /// </summary>
        [Fact]
        public void Plan_BoundaryCells_KeepEntireWorkCenterlineInField
            (
            )
        {
            Grid Grid = MakeSparseGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((2, 2), 0.02),
                    ((2, 3), 0.02),
                    ((3, 3), 0.02),
                };

            CoveragePlanner.PlanResult Plan = CoveragePlanner.Plan(
                Cells, Grid, BLADE_WIDTH_M, null, null);

            Assert.True(CoveragePlanner.ConservesAllocation(Cells, Plan.AccountedCells));
            Assert.True(Plan.Runs.Count < Cells.Count);
            foreach (CoveragePlanner.Run Run in Plan.Runs)
            {
                for (Int32 Index = 0; Index <= 20; Index += 1)
                {
                    Double Fraction = Index / 20.0;
                    Double X = Run.StartXy.X + (Fraction * (Run.EndXy.X - Run.StartXy.X));
                    Double Y = Run.StartXy.Y + (Fraction * (Run.EndXy.Y - Run.StartXy.Y));
                    Assert.True(Grid.ContainsWorldXy(X, Y));
                }
            }
        }

        /// <summary>
        /// Verifies boundary-adjacent work is driven parallel to a straight outer field edge.
        /// </summary>
        [Fact]
        public void Plan_EdgeWork_IsParallelToOuterBoundary
            (
            )
        {
            Grid Grid = MakeInsetGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((3, 2), 0.011),
                    ((4, 2), 0.012),
                    ((5, 2), 0.013),
                    ((6, 2), 0.014),
                };

            CoveragePlanner.PlanResult Plan = CoveragePlanner.Plan(
                Cells, Grid, BLADE_WIDTH_M, null, null);

            Assert.Empty(Plan.OmittedCells);
            Assert.NotEmpty(Plan.Runs);
            Assert.All(Plan.Runs, Run => Assert.True(
                FieldEdgePolicy.AxialHeadingDifferenceDeg(
                    Run.HeadingDeg, 90.0) <= 7.5 + Constants.NUMERIC_COMPARISON_TOLERANCE));
        }

        /// <summary>
        /// Verifies incompatible corner headings are explicitly and minimally omitted, never silently lost.
        /// </summary>
        [Fact]
        public void Plan_IncompatibleCornerWork_IsExplicitlyAccountedAsOmitted
            (
            )
        {
            Grid Grid = MakeInsetGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((4, 2), 0.011),
                    ((5, 2), 0.012),
                    ((2, 4), 0.013),
                };

            CoveragePlanner.PlanResult Plan = CoveragePlanner.Plan(
                Cells, Grid, BLADE_WIDTH_M, null, null);

            Assert.True(CoveragePlanner.ConservesAllocation(
                Cells, Plan.AccountedCells));
            Assert.Single(Plan.OmittedCells);
            Assert.Equal(2, Plan.OrderedCells.Count);
        }

        /// <summary>
        /// Verifies short-run fill omission pulls matching cut volume so the haul stays balanced.
        /// </summary>
        [Fact]
        public void BalanceShortRunVolumes_OmitsMatchingCutWhenFillScrapDropped
            (
            )
        {
            Grid Grid = MakeGrid();
            const Double DepthM = 0.05;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            // Long cut strip that remains after the gate and can donate balance volume.
            for (Int32 Bx = 2; Bx <= 12; Bx += 1)
            {
                CutCells.Add(((Bx, 5), DepthM));
            }
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Bx = 2; Bx <= 12; Bx += 1)
            {
                FillCells.Add(((Bx, 8), DepthM));
            }
            // Isolated fill scrap that the min-length gate must drop.
            FillCells.Add(((2, 15), DepthM));

            const Double MinimumWorkLengthM = 3.0;
            CoveragePlanner.PlanResult Cut = CoveragePlanner.Plan(
                CutCells,
                Grid,
                BLADE_WIDTH_M,
                MinimumWorkLengthM: MinimumWorkLengthM);
            CoveragePlanner.PlanResult Fill = CoveragePlanner.Plan(
                FillCells,
                Grid,
                BLADE_WIDTH_M,
                MinimumWorkLengthM: MinimumWorkLengthM);
            Assert.Empty(Cut.ShortRunOmittedCells);
            Assert.NotEmpty(Fill.ShortRunOmittedCells);

            (CoveragePlanner.PlanResult BalancedCut,
                CoveragePlanner.PlanResult BalancedFill) =
                CoveragePlanner.BalanceShortRunVolumes(
                    Cut, Fill, Grid, MinimumWorkLengthM);

            Double CutFactor = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillFactor = CutFactor * Grid.Compaction;
            Double CutShortBcy = BalancedCut.ShortRunOmittedCells
                .Where(Entry => Entry.DepthM > 0.0)
                .Sum(Entry => Entry.DepthM * CutFactor);
            Double FillShortBcy = BalancedFill.ShortRunOmittedCells
                .Where(Entry => Entry.DepthM > 0.0)
                .Sum(Entry => Entry.DepthM * FillFactor);
            Assert.True(
                CutShortBcy + Constants.NUMERIC_COMPARISON_TOLERANCE >= FillShortBcy);
            Assert.True(
                BalancedCut.ShortRunOmittedCells.Count
                > Cut.ShortRunOmittedCells.Count);
            Assert.True(
                BalancedCut.OrderedCells.Count < Cut.OrderedCells.Count);
        }

        /// <summary>
        /// Verifies balancing defers exactly the scrap volume on both sides when no
        /// set of whole cells adds up to it.
        /// </summary>
        [Fact]
        public void BalanceShortRunVolumes_UnevenDepths_DefersOnlyTheScrapVolume
            (
            )
        {
            Grid Grid = MakeGrid();
            (CoveragePlanner.PlanResult Cut,
                CoveragePlanner.PlanResult Fill,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells) Fixture = MakeUnevenBalanceFixture(Grid);
            Double CutFactor = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillFactor = CutFactor * Grid.Compaction;
            // The gate drops one isolated fill cell; that is all the balance has to match.
            Double ScrapBcy = Fixture.Fill.ShortRunOmittedCells.Sum(Entry => Entry.DepthM) * FillFactor;

            (CoveragePlanner.PlanResult BalancedCut,
                CoveragePlanner.PlanResult BalancedFill) =
                CoveragePlanner.BalanceShortRunVolumes(
                    Fixture.Cut, Fixture.Fill, Grid, BALANCE_MINIMUM_WORK_LENGTH_M);

            Double CutShortBcy = BalancedCut.ShortRunOmittedCells.Sum(Entry => Entry.DepthM) * CutFactor;
            Double FillShortBcy = BalancedFill.ShortRunOmittedCells.Sum(Entry => Entry.DepthM) * FillFactor;
            Assert.True(ScrapBcy > 0.0, "The fixture dropped no fill scrap.");
            Assert.InRange(Math.Abs(CutShortBcy - FillShortBcy), 0.0, BALANCE_VOLUME_TOLERANCE_BCY);
            Assert.InRange(Math.Abs(FillShortBcy - ScrapBcy), 0.0, BALANCE_VOLUME_TOLERANCE_BCY);
        }

        /// <summary>
        /// Verifies a balanced pair of plans is left alone by a second balancing call,
        /// which is what the guidance planner makes.
        /// </summary>
        [Fact]
        public void BalanceShortRunVolumes_CalledTwice_ChangesNothingTheSecondTime
            (
            )
        {
            Grid Grid = MakeGrid();
            (CoveragePlanner.PlanResult Cut,
                CoveragePlanner.PlanResult Fill,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells) Fixture = MakeUnevenBalanceFixture(Grid);
            (CoveragePlanner.PlanResult FirstCut,
                CoveragePlanner.PlanResult FirstFill) =
                CoveragePlanner.BalanceShortRunVolumes(
                    Fixture.Cut, Fixture.Fill, Grid, BALANCE_MINIMUM_WORK_LENGTH_M);

            (CoveragePlanner.PlanResult SecondCut,
                CoveragePlanner.PlanResult SecondFill) =
                CoveragePlanner.BalanceShortRunVolumes(
                    FirstCut, FirstFill, Grid, BALANCE_MINIMUM_WORK_LENGTH_M);

            Assert.Equal(FirstCut.Runs.Count, SecondCut.Runs.Count);
            Assert.Equal(FirstFill.Runs.Count, SecondFill.Runs.Count);
            Assert.InRange(
                Math.Abs(SecondCut.ShortRunOmittedCells.Sum(Entry => Entry.DepthM)
                    - FirstCut.ShortRunOmittedCells.Sum(Entry => Entry.DepthM)),
                0.0,
                BALANCE_DEPTH_TOLERANCE_M);
            Assert.InRange(
                Math.Abs(SecondFill.ShortRunOmittedCells.Sum(Entry => Entry.DepthM)
                    - FirstFill.ShortRunOmittedCells.Sum(Entry => Entry.DepthM)),
                0.0,
                BALANCE_DEPTH_TOLERANCE_M);
        }

        /// <summary>
        /// Verifies balancing loses and invents no soil: for every cell, the depth
        /// kept in runs plus the depth omitted equals the depth that was put in,
        /// also when one cell is split between a run and the deferred list.
        /// </summary>
        [Fact]
        public void BalanceShortRunVolumes_UnevenDepths_ConservesDepthPerCell
            (
            )
        {
            Grid Grid = MakeGrid();
            (CoveragePlanner.PlanResult Cut,
                CoveragePlanner.PlanResult Fill,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells) Fixture = MakeUnevenBalanceFixture(Grid);

            (CoveragePlanner.PlanResult BalancedCut,
                CoveragePlanner.PlanResult BalancedFill) =
                CoveragePlanner.BalanceShortRunVolumes(
                    Fixture.Cut, Fixture.Fill, Grid, BALANCE_MINIMUM_WORK_LENGTH_M);

            AssertDepthPerCell(Fixture.CutCells, BalancedCut.AccountedCells);
            AssertDepthPerCell(Fixture.FillCells, BalancedFill.AccountedCells);
        }

        /// <summary>
        /// Verifies a reverse between runs is charged a half-turn and a same-heading
        /// pair is not, so lineup metres outrank a bare run count.
        /// </summary>
        [Fact]
        public void EstimateRunOverheadM_ChargesAHalfTurnWhenTheNextRunReverses
            (
            )
        {
            CoveragePlanner.Run East = new CoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { ((1, 1), 0.01) },
                (0.0, 0.0),
                (10.0, 0.0),
                90.0,
                0.0);
            CoveragePlanner.Run West = new CoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { ((1, 2), 0.01) },
                (10.0, 2.0),
                (0.0, 2.0),
                270.0,
                0.0);
            CoveragePlanner.Run EastAgain = new CoveragePlanner.Run(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { ((1, 3), 0.01) },
                (0.0, 4.0),
                (10.0, 4.0),
                90.0,
                0.0);
            Double ReversedM = CoveragePlanner.EstimateRunOverheadM(
                new List<CoveragePlanner.Run> { East, West });
            Double ContinuedM = CoveragePlanner.EstimateRunOverheadM(
                new List<CoveragePlanner.Run> { East, EastAgain });
            Double TurnArcM = Constants.DEFAULT_TURNING_CIRCLE_FT
                * Constants.HALF_SCALE
                * Constants.FEET_TO_METERS
                * Math.PI;
            Assert.InRange(Math.Abs((ReversedM - ContinuedM) - TurnArcM), 0.0, 1e-9);
        }

        /// <summary>
        /// Builds the balance fixture: an eleven-cell cut strip, an eleven-cell fill
        /// strip, and one isolated fill cell that the minimum-length gate drops.
        /// Every cell of a strip has a different depth, so whole cells never add up
        /// to the scrap volume.
        /// </summary>
        /// <param name="Grid">Grid the plans are built on.</param>
        /// <returns>The gated cut plan, the gated fill plan, and the two input cell lists.</returns>
        static private (CoveragePlanner.PlanResult Cut,
            CoveragePlanner.PlanResult Fill,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells) MakeUnevenBalanceFixture
            (
            Grid Grid
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Bx = BALANCE_FIRST_COLUMN; Bx <= BALANCE_LAST_COLUMN; Bx += 1)
            {
                Double DepthM = BALANCE_BASE_DEPTH_M + (BALANCE_DEPTH_STEP_M * Bx);
                CutCells.Add(((Bx, BALANCE_CUT_ROW), DepthM));
                FillCells.Add(((Bx, BALANCE_FILL_ROW), DepthM));
            }
            // One cell far from the fill strip forms a run below the minimum length.
            FillCells.Add((
                (BALANCE_FIRST_COLUMN, BALANCE_SCRAP_ROW),
                BALANCE_BASE_DEPTH_M + (BALANCE_DEPTH_STEP_M * BALANCE_FIRST_COLUMN)));

            CoveragePlanner.PlanResult Cut = CoveragePlanner.Plan(
                CutCells,
                Grid,
                BLADE_WIDTH_M,
                MinimumWorkLengthM: BALANCE_MINIMUM_WORK_LENGTH_M);
            CoveragePlanner.PlanResult Fill = CoveragePlanner.Plan(
                FillCells,
                Grid,
                BLADE_WIDTH_M,
                MinimumWorkLengthM: BALANCE_MINIMUM_WORK_LENGTH_M);
            return (Cut, Fill, CutCells, FillCells);
        }

        /// <summary>Asserts that two cell lists hold the same total depth in every cell.</summary>
        /// <param name="Expected">Cells and depths that were put in.</param>
        /// <param name="Actual">Cells and depths that came out; a cell may appear more than once.</param>
        static private void AssertDepthPerCell
            (
            IEnumerable<((Int32 Bx, Int32 By) Cell, Double DepthM)> Expected,
            IEnumerable<((Int32 Bx, Int32 By) Cell, Double DepthM)> Actual
            )
        {
            Dictionary<(Int32 Bx, Int32 By), Double> ExpectedByCell = Expected
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Group => Group.Key, Group => Group.Sum(Entry => Entry.DepthM));
            Dictionary<(Int32 Bx, Int32 By), Double> ActualByCell = Actual
                .GroupBy(Entry => Entry.Cell)
                .ToDictionary(Group => Group.Key, Group => Group.Sum(Entry => Entry.DepthM));
            Assert.Equal(ExpectedByCell.Count, ActualByCell.Count);
            foreach (KeyValuePair<(Int32 Bx, Int32 By), Double> Entry in ExpectedByCell)
            {
                Assert.True(ActualByCell.ContainsKey(Entry.Key), "Cell " + Entry.Key + " is missing.");
                Assert.InRange(
                    Math.Abs(ActualByCell[Entry.Key] - Entry.Value),
                    0.0,
                    BALANCE_DEPTH_TOLERANCE_M);
            }
        }

        /// <summary>
        /// Verifies runs shorter than the minimum work length are omitted and conserved.
        /// </summary>
        [Fact]
        public void Plan_OmitsRunsBelowMinimumWorkLength
            (
            )
        {
            Grid Grid = MakeGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            // Long continuous strip (~8 m centerline) that must remain driven.
            for (Int32 Bx = 2; Bx <= 10; Bx += 1)
            {
                Cells.Add(((Bx, 5), 0.02));
            }
            // Isolated scrap cell that forms its own ~1 m run.
            Cells.Add(((2, 12), 0.03));

            const Double MinimumWorkLengthM = 3.0;
            CoveragePlanner.PlanResult Plan = CoveragePlanner.Plan(
                Cells,
                Grid,
                BLADE_WIDTH_M,
                null,
                null,
                MinimumWorkLengthM);

            Assert.True(CoveragePlanner.ConservesAllocation(
                Cells, Plan.AccountedCells));
            Assert.NotEmpty(Plan.Runs);
            Assert.All(
                Plan.Runs,
                Run => Assert.True(
                    Run.LengthM + Constants.NUMERIC_COMPARISON_TOLERANCE
                    >= MinimumWorkLengthM));
            Assert.NotEmpty(Plan.ShortRunOmittedCells);
            Assert.Contains(
                Plan.ShortRunOmittedCells,
                Entry => Entry.Cell == (2, 12));
            Assert.DoesNotContain(
                Plan.OrderedCells,
                Entry => Entry.Cell == (2, 12));
        }

        /// <summary>
        /// A cut that starts west and then turns south keeps the centerline on
        /// both legs. The opening heading used to average the south leg into
        /// one footprint, so the line left the cells and stopped short.
        /// </summary>
        [Fact]
        public void PlanRibbon_FollowsACutThatTurnsOffItsOpeningHeading
            (
            )
        {
            Grid Grid = MakeOpenGrid();
            // The same shape as found-order route 1036: a short west leg,
            // then a south leg with the blade's side cells mixed in.
            (Int32 Bx, Int32 By)[] Shape =
            {
                (24, 30), (23, 30), (22, 30), (21, 30),
                (20, 31), (20, 30), (20, 29), (19, 29),
                (20, 28), (19, 28), (19, 27), (19, 26),
                (19, 25), (18, 25), (19, 24), (18, 24),
                (19, 23), (18, 23), (19, 22), (18, 22),
                (18, 21), (18, 20), (18, 19), (18, 18),
                (18, 17), (18, 16), (18, 15), (18, 14),
            };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            foreach ((Int32 Bx, Int32 By) Cell in Shape)
            {
                Cells.Add((Cell, 0.061));
            }
            CoveragePlanner.PlanResult Plan = CoveragePlanner.PlanRibbon(Cells, Grid, BLADE_WIDTH_M);
            Assert.NotEmpty(Plan.Runs);
            List<(Double X, Double Y)> Drive = new List<(Double X, Double Y)>();
            foreach (CoveragePlanner.Run Run in Plan.Runs)
            {
                Assert.NotNull(Run.DrivePoints);
                Drive.AddRange(Run.DrivePoints!);
            }
            Assert.True(Drive.Count >= 2);
            (Double X, Double Y) FirstStep = (
                Drive[1].X - Drive[0].X,
                Drive[1].Y - Drive[0].Y);
            Double FirstHeading = (Math.Atan2(FirstStep.X, FirstStep.Y) * 180.0 / Math.PI + 360.0) % 360.0;
            // The pan starts along the west leg. The turn comes after that.
            Double FromWest = Math.Abs(FirstHeading - 270.0);
            if (FromWest > 180.0) FromWest = 360.0 - FromWest;
            Assert.True(FromWest < 20.0, FirstHeading.ToString(System.Globalization.CultureInfo.InvariantCulture));
            (Double X, Double Y) LastCell = Grid.CellCenterXy(18, 14);
            (Double X, Double Y) LastPoint = Drive[Drive.Count - 1];
            Double EndGap = Math.Sqrt(
                ((LastPoint.X - LastCell.X) * (LastPoint.X - LastCell.X))
                + ((LastPoint.Y - LastCell.Y) * (LastPoint.Y - LastCell.Y)));
            // The line reaches the mouth. Stopping on the opening heading left a gap of many cells.
            Assert.True(EndGap < Grid.GridSizeM, EndGap.ToString(System.Globalization.CultureInfo.InvariantCulture));
            foreach ((Double X, Double Y) Point in Drive)
            {
                Double Nearest = Double.MaxValue;
                foreach ((Int32 Bx, Int32 By) Cell in Shape)
                {
                    (Double X, Double Y) Center = Grid.CellCenterXy(Cell.Bx, Cell.By);
                    Double Dist = Math.Sqrt(
                        ((Point.X - Center.X) * (Point.X - Center.X))
                        + ((Point.Y - Center.Y) * (Point.Y - Center.Y)));
                    if (Dist < Nearest) Nearest = Dist;
                }
                // A fillet may leave the corner by about one cell. It may not chord off the strip.
                Assert.True(Nearest < (Grid.GridSizeM * 1.6), Nearest.ToString(System.Globalization.CultureInfo.InvariantCulture));
            }
        }

        /// <summary>
        /// A strip two cells wider than the center still draws one line down
        /// the middle. Treating every side cell as a new step scribbles across the blade.
        /// </summary>
        [Fact]
        public void PlanRibbon_KeepsABladeWideStripOnOneSpine
            (
            )
        {
            Grid Grid = MakeOpenGrid();
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Row = 30; Row >= 18; Row -= 1)
            {
                for (Int32 Column = 15; Column <= 17; Column += 1)
                {
                    Cells.Add(((Column, Row), 0.061));
                }
            }
            CoveragePlanner.PlanResult Plan = CoveragePlanner.PlanRibbon(Cells, Grid, BLADE_WIDTH_M);
            List<(Double X, Double Y)> Drive = new List<(Double X, Double Y)>();
            foreach (CoveragePlanner.Run Run in Plan.Runs)
            {
                Assert.NotNull(Run.DrivePoints);
                Drive.AddRange(Run.DrivePoints!);
            }
            Assert.True(Drive.Count >= 2);
            Int32 OffSouth = 0;
            for (Int32 Index = 1; Index < Drive.Count; Index += 1)
            {
                Double Dx = Drive[Index].X - Drive[Index - 1].X;
                Double Dy = Drive[Index].Y - Drive[Index - 1].Y;
                if (((Dx * Dx) + (Dy * Dy)) < 1e-6) continue;
                Double Heading = (Math.Atan2(Dx, Dy) * 180.0 / Math.PI + 360.0) % 360.0;
                Double FromSouth = Math.Abs(Heading - 180.0);
                if (FromSouth > 180.0) FromSouth = 360.0 - FromSouth;
                // The first step can cross the blade. Later steps go down the strip.
                if ((Index > 1) && (FromSouth > 35.0)) OffSouth += 1;
            }
            Assert.Equal(0, OffSouth);
            (Double X, Double Y) Start = Grid.CellCenterXy(15, 30);
            (Double X, Double Y) End = Grid.CellCenterXy(17, 18);
            Double Straight = Math.Abs(Start.Y - End.Y);
            Double Path = 0.0;
            for (Int32 Index = 1; Index < Drive.Count; Index += 1)
            {
                Double Dx = Drive[Index].X - Drive[Index - 1].X;
                Double Dy = Drive[Index].Y - Drive[Index - 1].Y;
                Path += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }
            Assert.True(Path < (Straight * 1.6), Path.ToString(System.Globalization.CultureInfo.InvariantCulture));
        }

        /// <summary>
        /// Creates a deterministic cell pattern whose supplied ordering repeatedly reverses direction.
        /// </summary>
        private static List<((Int32 Bx, Int32 By) Cell, Double DepthM)> MakeKnottedCells
            (
            )
        {
            // Unique depths make any loss, duplication, or cell/depth reassignment observable.
            return new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                ((2, 2), 0.011),
                ((6, 2), 0.012),
                ((3, 2), 0.013),
                ((5, 2), 0.014),
                ((4, 2), 0.015),
                ((4, 3), 0.016),
                ((2, 3), 0.017),
                ((6, 3), 0.018),
                ((3, 3), 0.019),
                ((5, 3), 0.020),
                ((2, 4), 0.021),
                ((3, 4), 0.022),
            };
        }

        /// <summary>
        /// Creates a pass while retaining supplied allocation lists by reference.
        /// </summary>
        private static Pass MakePass
            (
            Grid Grid,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells
            )
        {
            (Double X, Double Y) CutStart = Grid.CellCenterXy(CutCells[0].Cell.Bx, CutCells[0].Cell.By);
            (Double X, Double Y) CutEnd = Grid.CellCenterXy(
                CutCells[CutCells.Count - 1].Cell.Bx,
                CutCells[CutCells.Count - 1].Cell.By);
            (Double X, Double Y) FillStart = Grid.CellCenterXy(FillCells[0].Cell.Bx, FillCells[0].Cell.By);
            (Double X, Double Y) FillEnd = Grid.CellCenterXy(
                FillCells[FillCells.Count - 1].Cell.Bx,
                FillCells[FillCells.Count - 1].Cell.By);
            return new Pass(CutCells, FillCells, 1.0, 10.0, CutStart, CutEnd, FillStart, FillEnd);
        }

        /// <summary>
        /// Produces a compact comparison signature for run order and direction.
        /// </summary>
        private static String RunSignature
            (
            CoveragePlanner.PlanResult Plan
            )
        {
            return String.Join(
                "|",
                Plan.Runs.Select(Run => String.Format(
                    System.Globalization.CultureInfo.InvariantCulture,
                    "{0:F6},{1:F6}>{2:F6},{3:F6}:{4}",
                    Run.StartXy.X,
                    Run.StartXy.Y,
                    Run.EndXy.X,
                    Run.EndXy.Y,
                    String.Join(",", Run.Cells.Select(Cell => String.Format(
                        System.Globalization.CultureInfo.InvariantCulture,
                        "{0}/{1}/{2:R}",
                        Cell.Cell.Bx,
                        Cell.Cell.By,
                        Cell.DepthM))))));
        }

        /// <summary>
        /// Creates a large all-valid grid so a turned ribbon sits away from the field edge.
        /// </summary>
        private static Grid MakeOpenGrid
            (
            )
        {
            const Int32 SIZE = 40;
            Boolean[,] Valid = new Boolean[SIZE, SIZE];
            for (Int32 By = 0; By < SIZE; By += 1)
            {
                for (Int32 Bx = 0; Bx < SIZE; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                SIZE,
                SIZE,
                500000.0,
                4000000.0,
                12,
                true,
                0.0,
                0.0,
                new Double[SIZE, SIZE],
                new Double[SIZE, SIZE],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>
        /// Creates an all-valid grid for deterministic coverage tests.
        /// </summary>
        private static Grid MakeGrid
            (
            )
        {
            const Int32 SIZE = 20;
            Boolean[,] Valid = new Boolean[SIZE, SIZE];
            for (Int32 By = 0; By < SIZE; By += 1)
            {
                for (Int32 Bx = 0; Bx < SIZE; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                SIZE,
                SIZE,
                500000.0,
                4000000.0,
                12,
                true,
                0.0,
                0.0,
                new Double[SIZE, SIZE],
                new Double[SIZE, SIZE],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>
        /// Creates a sparse L-shaped field for boundary containment testing.
        /// </summary>
        private static Grid MakeSparseGrid
            (
            )
        {
            const Int32 SIZE = 8;
            Boolean[,] Valid = new Boolean[SIZE, SIZE];
            Valid[2, 2] = true;
            Valid[3, 2] = true;
            Valid[3, 3] = true;
            return new Grid(
                1.0,
                SIZE,
                SIZE,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[SIZE, SIZE],
                new Double[SIZE, SIZE],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>Creates a rectangular field surrounded by an invalid exterior margin.</summary>
        private static Grid MakeInsetGrid
            (
            )
        {
            const Int32 SIZE = 12;
            Boolean[,] Valid = new Boolean[SIZE, SIZE];
            for (Int32 By = 2; By <= 9; By += 1)
            {
                for (Int32 Bx = 2; Bx <= 9; Bx += 1)
                {
                    Valid[By, Bx] = true;
                }
            }
            return new Grid(
                1.0,
                SIZE,
                SIZE,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                new Double[SIZE, SIZE],
                new Double[SIZE, SIZE],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
