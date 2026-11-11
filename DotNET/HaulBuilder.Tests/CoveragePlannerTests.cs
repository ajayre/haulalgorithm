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
