using System;
using System.Collections.Generic;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Pan-target finish check: a miss is a warning with counts, the worst cell, and above/below volume.
    /// </summary>
    public sealed class PanTargetMissTests
    {
        /// <summary>Finish tolerance used by the fixture, in millimetres.</summary>
        private const Double TOLERANCE_MM = 10.0;

        /// <summary>Cell edge used by the fixture, in metres. Area is one square metre.</summary>
        private const Double CELL_M = 1.0;

        /// <summary>Compaction applied to fill that is still missing.</summary>
        private const Double COMPACTION = 1.2;

        /// <summary>Field design elevation under the pan, in metres.</summary>
        private const Double FIELD_M = 100.0;

        /// <summary>Pan height above the field design, in metres.</summary>
        private const Double PAN_M = 0.5;

        /// <summary>
        /// Cells on the tolerance, open ends, the casing, and cells off the pan are not misses.
        /// A pass and a blade move are applied before the miss is measured.
        /// </summary>
        [Fact]
        public void MeasurePanTarget_ReportsMissesWithoutFailing()
        {
            const Int32 Size = 3;
            Grid Grid = CreateGrid(Size, COMPACTION);
            RoadPadSurface Surface = new RoadPadSurface(Size, Size);
            Double Target = FIELD_M + PAN_M;
            for (Int32 By = 0; By < Size; By += 1)
            {
                for (Int32 Bx = 0; Bx < Size; Bx += 1)
                {
                    Surface.FieldDesignM[By, Bx] = FIELD_M;
                    Surface.PanM[By, Bx] = PAN_M;
                    Grid.Existing[By, Bx] = Target;
                }
            }

            // 40 mm high. Counted as above.
            Grid.Existing[0, 0] = Target + (40.0 / Constants.MILLIMETERS_PER_METER);
            // 30 mm low. Counted as below.
            Grid.Existing[0, 1] = Target - (30.0 / Constants.MILLIMETERS_PER_METER);
            // Exactly on the tolerance. Not a miss.
            Grid.Existing[0, 2] = Target + (TOLERANCE_MM / Constants.MILLIMETERS_PER_METER);
            // Open end and the well casing are excused, even when they are far off.
            Surface.OpenEnd[1, 0] = true;
            Grid.Existing[1, 0] = Target + (100.0 / Constants.MILLIMETERS_PER_METER);
            Surface.Excluded[1, 1] = true;
            Grid.Existing[1, 1] = Target - (100.0 / Constants.MILLIMETERS_PER_METER);
            // Off the pan. Not checked.
            Surface.PanM[1, 2] = 0.0;
            Grid.Existing[1, 2] = Target + (100.0 / Constants.MILLIMETERS_PER_METER);
            // Outside the field.
            Grid.Valid[2, 0] = false;
            Grid.Existing[2, 0] = Target + (100.0 / Constants.MILLIMETERS_PER_METER);

            // A cut pass drops (2, 1) from the target. That miss is the worst, and it is below.
            List<Pass> FieldPasses = new List<Pass>
            {
                new Pass(
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                    {
                        ((1, 2), 50.0 / Constants.MILLIMETERS_PER_METER),
                    },
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                    0.0,
                    0.0,
                    (0.0, 0.0),
                    (0.0, 0.0),
                    (0.0, 0.0),
                    (0.0, 0.0)),
            };
            // Blade fill lifts (2, 2) above the target.
            Surface.BladeFills.Add(((2, 2), 20.0 / Constants.MILLIMETERS_PER_METER));

            PanTargetMiss? Miss = RoadPadBuild.MeasurePanTarget(
                Grid.Existing,
                Grid,
                Surface,
                FieldPasses,
                new List<Pass>(),
                TOLERANCE_MM);

            Assert.NotNull(Miss);
            Assert.Equal(4, Miss.CellCount);
            Assert.Equal(1, Miss.WorstColumn);
            Assert.Equal(2, Miss.WorstRow);
            Assert.Equal(-50.0, Miss.WorstMm, 6);
            Assert.Contains(Miss.Cells, Cell => (Cell.Column == 0) && (Cell.Row == 0));
            Assert.Contains(Miss.Cells, Cell => (Cell.Column == 1) && (Cell.Row == 0));
            Assert.Contains(Miss.Cells, Cell => (Cell.Column == 1) && (Cell.Row == 2));
            Assert.Contains(Miss.Cells, Cell => (Cell.Column == 2) && (Cell.Row == 2));
            Assert.DoesNotContain(Miss.Cells, Cell => (Cell.Column == 2) && (Cell.Row == 0));

            Double CutPer = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPer = CutPer * COMPACTION;
            Double AboveM = (40.0 + 20.0) / Constants.MILLIMETERS_PER_METER;
            Double BelowM = (30.0 + 50.0) / Constants.MILLIMETERS_PER_METER;
            Assert.Equal(AboveM * CutPer, Miss.AboveBcy, 6);
            Assert.Equal(BelowM * FillPer, Miss.BelowBcy, 6);

            String Text = Miss.FormatMessage();
            Assert.Contains("4 cells", Text, StringComparison.Ordinal);
            Assert.Contains("50.0 mm below at (1, 2)", Text, StringComparison.Ordinal);
            Assert.Contains("Above", Text, StringComparison.Ordinal);
            Assert.Contains("below", Text, StringComparison.Ordinal);
        }

        /// <summary>
        /// A pan that finishes inside the tolerance is not a warning.
        /// </summary>
        [Fact]
        public void MeasurePanTarget_ReturnsNullWhenOnTarget()
        {
            Grid Grid = CreateGrid(1, COMPACTION);
            RoadPadSurface Surface = new RoadPadSurface(1, 1);
            Surface.FieldDesignM[0, 0] = FIELD_M;
            Surface.PanM[0, 0] = PAN_M;
            Grid.Existing[0, 0] = FIELD_M + PAN_M;

            PanTargetMiss? Miss = RoadPadBuild.MeasurePanTarget(
                Grid.Existing,
                Grid,
                Surface,
                new List<Pass>(),
                new List<Pass>(),
                TOLERANCE_MM);

            Assert.Null(Miss);
        }

        /// <summary>
        /// Builds a valid square grid at zero elevation.
        /// </summary>
        /// <param name="Size">Width and height in cells.</param>
        /// <param name="Compaction">Compaction ratio stored on the grid.</param>
        /// <returns>The grid.</returns>
        private static Grid CreateGrid
            (
            Int32 Size,
            Double Compaction
            )
        {
            return new Grid(
                CELL_M,
                Size,
                Size,
                0.0,
                0.0,
                15,
                true,
                36.0,
                -90.0,
                new Double[Size, Size],
                new Double[Size, Size],
                Filled(Size, true),
                1.3,
                Compaction,
                150.0,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>
        /// Fills a square mask with one value.
        /// </summary>
        /// <param name="Size">Width and height.</param>
        /// <param name="Value">Value stored in every cell.</param>
        /// <returns>The mask.</returns>
        private static Boolean[,] Filled
            (
            Int32 Size,
            Boolean Value
            )
        {
            Boolean[,] Mask = new Boolean[Size, Size];
            for (Int32 By = 0; By < Size; By += 1)
            {
                for (Int32 Bx = 0; Bx < Size; Bx += 1)
                {
                    Mask[By, Bx] = Value;
                }
            }

            return Mask;
        }
    }
}
