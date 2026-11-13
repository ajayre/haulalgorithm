using System;
using System.IO;

using Xunit;

using HaulBuilder;

namespace FieldSolver.Tests
{
    /// <summary>
    /// Verifies that the standard search's lane axes come from the design-surface
    /// down-slope, with the same cardinal snap as the engine.
    /// </summary>
    public sealed class DesignSlopeLaneAxisTests
    {
        /// <summary>Expected design down-slope bearing of TheShop2, in degrees true.</summary>
        private const Double SHOP_SLOPE_BEARING_DEG = 270.0;

        /// <summary>Expected undirected lane axis of that bearing.</summary>
        private const Double SHOP_LANE_AXIS_DEG = 90.0;

        /// <summary>Expected perpendicular lane axis.</summary>
        private const Double SHOP_PERPENDICULAR_AXIS_DEG = 0.0;

        /// <summary>
        /// Verifies TheShop2's design slope is east-west after the cardinal snap,
        /// so the standard search on that field still tries axes 90 and 0.
        /// </summary>
        [Fact]
        public void FromOptions_TheShop2_ReturnsEastWestThenNorthSouth
            (
            )
        {
            String? AgdPath = FindTheShop2Agd();
            if (AgdPath == null)
            {
                // Intent: the repository field is optional in a copy of the tests alone.
                return;
            }

            HaulBuilderOptions Options = new HaulBuilderOptions { AgdPath = AgdPath };

            Boolean Ok = DesignSlopeLaneAxis.TryFromOptions(
                Options,
                out Double LaneAxisDeg,
                out Double PerpendicularAxisDeg);

            Assert.True(Ok);
            Assert.Equal(SHOP_LANE_AXIS_DEG, LaneAxisDeg);
            Assert.Equal(SHOP_PERPENDICULAR_AXIS_DEG, PerpendicularAxisDeg);

            FieldSolverSearchSpace Space = FieldSolverSearchSpace.CreateStandard(Options);
            Assert.Equal(
                new[] { SHOP_LANE_AXIS_DEG, SHOP_PERPENDICULAR_AXIS_DEG },
                Space.LaneAxisHeadingDegValues);
        }

        /// <summary>
        /// Verifies a flat design surface yields north-south as the slope axis, so
        /// the standard search still tries 0 and 90.
        /// </summary>
        [Fact]
        public void FromOptions_FlatTinyField_ReturnsNorthSouthThenEastWest
            (
            )
        {
            String AgdPath = SolverFixtures.CreateTinyAgd();
            try
            {
                HaulBuilderOptions Options = new HaulBuilderOptions { AgdPath = AgdPath };

                Boolean Ok = DesignSlopeLaneAxis.TryFromOptions(
                    Options,
                    out Double LaneAxisDeg,
                    out Double PerpendicularAxisDeg);

                Assert.True(Ok);
                Assert.Equal(SolverFixtures.AXIS_NORTH_SOUTH, LaneAxisDeg);
                Assert.Equal(SolverFixtures.AXIS_EAST_WEST, PerpendicularAxisDeg);
            }
            finally
            {
                File.Delete(AgdPath);
            }
        }

        /// <summary>
        /// Verifies a missing field file falls back to north-south and east-west,
        /// so table-driven unit tests keep a fixed eight-run search.
        /// </summary>
        [Fact]
        public void CreateStandard_MissingField_FallsBackToCardinalAxes
            (
            )
        {
            FieldSolverSearchSpace Space = FieldSolverSearchSpace.CreateStandard(
                new HaulBuilderOptions { AgdPath = "no-such-field.agd" });

            Assert.Equal(
                new[] { SolverFixtures.AXIS_NORTH_SOUTH, SolverFixtures.AXIS_EAST_WEST },
                Space.LaneAxisHeadingDegValues);
        }

        /// <summary>
        /// Verifies the unsnapped grid slope of TheShop2 is not what the search
        /// uses: the twelve-degree snap is required.
        /// </summary>
        [Fact]
        public void FromGrid_TheShop2_SnapsBearingTo270
            (
            )
        {
            String? AgdPath = FindTheShop2Agd();
            if (AgdPath == null)
            {
                return;
            }

            HaulBuilderOptions Options = new HaulBuilderOptions { AgdPath = AgdPath };
            AgdField Field = Agd.Parse(AgdPath);
            Grid Grid = GridMath.BuildGrid(
                Field,
                Options.GridSizeFt,
                Options.SwellFactor,
                Options.CutFillRatio,
                Options.MaxDepthMm);

            (Double BearingDeg, Double LaneAxisDeg, Double PerpendicularAxisDeg) =
                DesignSlopeLaneAxis.FromGrid(Grid);

            Assert.Equal(SHOP_SLOPE_BEARING_DEG, BearingDeg);
            Assert.Equal(SHOP_LANE_AXIS_DEG, LaneAxisDeg);
            Assert.Equal(SHOP_PERPENDICULAR_AXIS_DEG, PerpendicularAxisDeg);

            // Without the snap the grid-frame slope is about 268.65 deg true / 88.65
            // axis, which measured 58 km longer than the snapped east-west axis.
            (Double RawBearingDeg, Double RawAxisDeg, _) = DesignSlopeLaneAxis.FromGrid(
                Grid,
                SnapToleranceDeg: 0.0);
            Assert.NotEqual(SHOP_SLOPE_BEARING_DEG, RawBearingDeg);
            Assert.NotEqual(SHOP_LANE_AXIS_DEG, RawAxisDeg);
        }

        /// <summary>Finds TheShop2 beside the repository copy, or null when absent.</summary>
        /// <returns>Absolute AGD path, or null.</returns>
        static private String? FindTheShop2Agd
            (
            )
        {
            // Intent: the repository field is beside this scratch copy of DotNET/.
            String BesideCopy = Path.GetFullPath(Path.Combine(
                AppContext.BaseDirectory,
                "..", "..", "..", "..", "..", "..", "..",
                "OneDrive", "Documents", "Haul Algorithm", "DotNET", "TheShop2_2ft.agd"));
            String KnownPath = @"C:\Users\andy\OneDrive\Documents\Haul Algorithm\DotNET\TheShop2_2ft.agd";
            if (File.Exists(KnownPath))
            {
                return KnownPath;
            }
            return File.Exists(BesideCopy) ? BesideCopy : null;
        }
    }
}
