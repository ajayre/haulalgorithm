using System;
using System.IO;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// Derives the two lane axes of the standard search from the design surface:
    /// the down-slope axis and its perpendicular. The estimate matches
    /// <c>Passes.EstimateSlopeHeading</c> in HaulBuilder, including the twelve-degree
    /// snap to a cardinal bearing. Lane axis is undirected, so the bearing is then
    /// normalized into <c>[0, 180)</c>.
    /// </summary>
    static public class DesignSlopeLaneAxis
    {
        /// <summary>
        /// Degrees within which the raw slope bearing snaps to the nearest
        /// cardinal, matching <c>Passes.EstimateSlopeHeading</c>.
        /// </summary>
        public const Double CARDINAL_SNAP_TOLERANCE_DEG = 12.0;

        /// <summary>Full circle, in degrees.</summary>
        private const Double DEGREES_PER_TURN = 360.0;

        /// <summary>Half turn, in degrees.</summary>
        private const Double DEGREES_PER_HALF_TURN = 180.0;

        /// <summary>Gradient below which the design is treated as flat.</summary>
        private const Double FLAT_GRADIENT = 1e-12;

        /// <summary>
        /// Estimates the undirected lane axis of the design down-slope, and the
        /// axis perpendicular to it.
        /// </summary>
        /// <param name="Grid">Grid whose <see cref="Grid.Target"/> is the design surface.</param>
        /// <param name="SnapToleranceDeg">
        /// Cardinal snap window in degrees. Pass
        /// <see cref="CARDINAL_SNAP_TOLERANCE_DEG"/> to match the engine.
        /// </param>
        /// <returns>
        /// <c>SlopeBearingDeg</c> is the directed down-slope (0 to 360);
        /// <c>LaneAxisDeg</c> and <c>PerpendicularAxisDeg</c> are undirected axes
        /// in <c>[0, 180)</c>.
        /// </returns>
        /// <exception cref="ArgumentNullException">Thrown when grid is null.</exception>
        static public (Double SlopeBearingDeg, Double LaneAxisDeg, Double PerpendicularAxisDeg)
            FromGrid
            (
            Grid Grid,
            Double SnapToleranceDeg = CARDINAL_SNAP_TOLERANCE_DEG
            )
        {
            if (Grid == null) throw new ArgumentNullException(nameof(Grid));
            Double BearingDeg = EstimateDownSlopeBearingDeg(Grid, SnapToleranceDeg);
            Double LaneAxisDeg = HaulLaneBuilder.NormalizeAxisHeadingDeg(BearingDeg);
            Double PerpendicularAxisDeg = HaulLaneBuilder.NormalizeAxisHeadingDeg(
                LaneAxisDeg + DEGREES_PER_HALF_TURN / 2.0);
            return (BearingDeg, LaneAxisDeg, PerpendicularAxisDeg);
        }

        /// <summary>
        /// Builds the grid from the caller's field file and returns the two lane
        /// axes. Returns false when the path is missing or the AGD cannot be read.
        /// </summary>
        /// <param name="Options">Options that name the AGD file and the grid size.</param>
        /// <param name="LaneAxisDeg">Design-slope lane axis on success.</param>
        /// <param name="PerpendicularAxisDeg">Perpendicular lane axis on success.</param>
        /// <returns>True when both axes were computed.</returns>
        static public Boolean TryFromOptions
            (
            HaulBuilderOptions? Options,
            out Double LaneAxisDeg,
            out Double PerpendicularAxisDeg
            )
        {
            LaneAxisDeg = 0.0;
            PerpendicularAxisDeg = DEGREES_PER_HALF_TURN / 2.0;
            if ((Options == null)
                || String.IsNullOrWhiteSpace(Options.AgdPath)
                || !File.Exists(Options.AgdPath))
            {
                return false;
            }

            // Intent: only the design surface is needed; no haul planning.
            AgdField Field = Agd.Parse(Options.AgdPath);
            Grid Grid = GridMath.BuildGrid(
                Field,
                Options.GridSizeFt,
                Options.SwellFactor,
                Options.CutFillRatio,
                Options.MaxDepthMm);
            (_, LaneAxisDeg, PerpendicularAxisDeg) = FromGrid(Grid);
            return true;
        }

        /// <summary>
        /// Mean negative gradient of the design surface, snapped to a cardinal
        /// when it lies within the snap window. Matches
        /// <c>Passes.EstimateSlopeHeading</c>.
        /// </summary>
        /// <param name="Grid">Grid with a design surface.</param>
        /// <param name="SnapToleranceDeg">Cardinal snap window in degrees.</param>
        /// <returns>Directed down-slope bearing in degrees, or 0 when the surface is flat.</returns>
        static private Double EstimateDownSlopeBearingDeg
            (
            Grid Grid,
            Double SnapToleranceDeg
            )
        {
            Double SumE = 0.0;
            Double SumN = 0.0;
            Int32 Count = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double? Gx = CentralDiff(Grid.Target, Grid.Valid, By, Bx, true);
                    Double? Gy = CentralDiff(Grid.Target, Grid.Valid, By, Bx, false);
                    if (Gx.HasValue && Gy.HasValue)
                    {
                        // Negative gradient points downhill.
                        SumE += -Gx.Value;
                        SumN += -Gy.Value;
                        Count += 1;
                    }
                }
            }

            if ((Count == 0)
                || ((Math.Abs(SumE) < FLAT_GRADIENT) && (Math.Abs(SumN) < FLAT_GRADIENT)))
            {
                return 0.0;
            }

            Double Bearing = (Math.Atan2(SumE, SumN) * DEGREES_PER_HALF_TURN / Math.PI
                + DEGREES_PER_TURN) % DEGREES_PER_TURN;
            Double Nearest = Math.Round(Bearing / 90.0) * 90.0;
            Double Diff = Math.Abs(
                (((Bearing - Nearest + DEGREES_PER_HALF_TURN) % DEGREES_PER_TURN)
                    - DEGREES_PER_HALF_TURN));
            if (Diff <= SnapToleranceDeg)
            {
                return (Nearest + DEGREES_PER_TURN) % DEGREES_PER_TURN;
            }

            return Bearing;
        }

        /// <summary>Central difference on one axis of a masked field.</summary>
        /// <param name="Field">Sampled surface.</param>
        /// <param name="Valid">Valid-cell mask.</param>
        /// <param name="By">Row of the centre cell.</param>
        /// <param name="Bx">Column of the centre cell.</param>
        /// <param name="XAxis">True for east-west difference, false for north-south.</param>
        /// <returns>The derivative, or null when a neighbour is missing.</returns>
        static private Double? CentralDiff
            (
            Double[,] Field,
            Boolean[,] Valid,
            Int32 By,
            Int32 Bx,
            Boolean XAxis
            )
        {
            Int32 Height = Field.GetLength(0);
            Int32 Width = Field.GetLength(1);
            Int32 MinusBy = By;
            Int32 MinusBx = Bx;
            Int32 PlusBy = By;
            Int32 PlusBx = Bx;
            if (XAxis)
            {
                MinusBx -= 1;
                PlusBx += 1;
            }
            else
            {
                MinusBy -= 1;
                PlusBy += 1;
            }

            if ((MinusBy < 0) || (MinusBy >= Height) || (MinusBx < 0) || (MinusBx >= Width)
                || (PlusBy < 0) || (PlusBy >= Height) || (PlusBx < 0) || (PlusBx >= Width)
                || !Valid[MinusBy, MinusBx] || !Valid[PlusBy, PlusBx])
            {
                return null;
            }

            return (Field[PlusBy, PlusBx] - Field[MinusBy, MinusBx]) / 2.0;
        }
    }
}
