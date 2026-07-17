using System;
using System.Collections.Generic;
using System.Linq;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Contains automated checks for edge patch omission.
    /// </summary>

    public sealed class EdgePatchOmissionTests
    {
        /// <summary>
        /// Verifies internal legacy policy defaults remain deterministic.
        /// </summary>

        [Fact]
        public void InternalLegacyPolicyDefaultsRemainDeterministic
            (
            )
        {
            Assert.True(new EdgePatchOmission.Settings().Enabled);
        }

        /// <summary>
        /// Verifies policy can be disabled explicitly.
        /// </summary>

        [Fact]
        public void PolicyCanBeDisabledExplicitly
            (
            )
        {
            Grid Grid = MakeInsetGrid();
            PassResult Source = MakeMixedPassResult(Grid, 0.10);

            EdgePatchOmission.Settings Options = new EdgePatchOmission.Settings
            {
                Enabled = false,
            };
            EdgePatchOmissionResult Result = EdgePatchOmission.Apply(
                Source,
                Grid,
                Options,
                Obstacles.EMPTY_OBSTACLES);

            Assert.Same(Source, Result.PassResult);
            Assert.Empty(Result.OmittedCutCells);
            Assert.Empty(Result.OmittedFillCells);
        }

        /// <summary>
        /// Verifies removes eligible edge component and matching fill demand.
        /// </summary>

        [Fact]
        public void RemovesEligibleEdgeComponentAndMatchingFillDemand
            (
            )
        {
            Grid Grid = MakeInsetGrid();
            PassResult Source = MakeMixedPassResult(Grid, 0.10);
            Double OriginalCutBcy = CutBcy(Source.Passes[0], Grid);
            Double OriginalFillBcy = FillBcy(Source.Passes[0], Grid);

            EdgePatchOmissionResult Result = EdgePatchOmission.Apply(
                Source,
                Grid,
                EnabledOptions(),
                Obstacles.EMPTY_OBSTACLES);

            Assert.Equal(1, Result.OmittedComponentCount);
            Assert.Equal(0, Result.RemovedPassCount);
            Assert.Single(Result.OmittedCutCells);
            Assert.Equal((2, 2), Result.OmittedCutCells[0].Cell);
            Assert.DoesNotContain(
                Result.PassResult.Passes.SelectMany(Pass => Pass.CutCells),
                Entry => Entry.Cell == (2, 2));
            Assert.Equal(3, Source.Passes[0].CutCells.Count);
            Assert.Single(Result.PassResult.Passes);

            Pass Remaining = Result.PassResult.Passes[0];
            Assert.Equal(
                Source.Passes[0].FillCells.Select(Entry => Entry.Cell),
                Remaining.FillCells.Select(Entry => Entry.Cell));
            AssertClose(CutBcy(Remaining, Grid), FillBcy(Remaining, Grid));
            AssertClose(
                OriginalCutBcy - CutBcy(Remaining, Grid),
                Result.OmittedCutBcy);
            AssertClose(
                OriginalFillBcy - FillBcy(Remaining, Grid),
                Result.OmittedFillBcy);
            AssertClose(Result.OmittedCutBcy, Result.OmittedFillBcy);
        }

        /// <summary>
        /// Verifies drops pass when its only cut is an eligible edge patch.
        /// </summary>

        [Fact]
        public void DropsPassWhenItsOnlyCutIsAnEligibleEdgePatch
            (
            )
        {
            Grid Grid = MakeInsetGrid();
            Double CutPerM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPerM = CutPerM * Grid.Compaction;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = new()
            {
                ((2, 2), 0.20),
            };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells = new()
            {
                ((8, 8), (0.20 * CutPerM) / FillPerM),
            };
            Pass Pass = Passes.BuildPass(
                Grid,
                CutCells,
                FillCells,
                0.20 * CutPerM,
                Obstacles.EMPTY_OBSTACLES,
                true);
            PassResult Source = new PassResult(new List<Pass> { Pass }, 1, 0.0, 0.0);

            EdgePatchOmissionResult Result = EdgePatchOmission.Apply(
                Source,
                Grid,
                EnabledOptions(),
                Obstacles.EMPTY_OBSTACLES);

            Assert.Empty(Result.PassResult.Passes);
            Assert.Equal(1, Result.OmittedComponentCount);
            Assert.Equal(1, Result.RemovedPassCount);
            AssertClose(0.20 * CutPerM, Result.OmittedCutBcy);
            AssertClose(Result.OmittedCutBcy, Result.OmittedFillBcy);
        }

        /// <summary>
        /// Verifies retains patch above per patch volume limit.
        /// </summary>

        [Fact]
        public void RetainsPatchAbovePerPatchVolumeLimit
            (
            )
        {
            Grid Grid = MakeInsetGrid();
            PassResult Source = MakeMixedPassResult(Grid, 0.80);
            EdgePatchOmission.Settings Options = EnabledOptions();
            Options.MaximumPatchBcy = 0.6;

            EdgePatchOmissionResult Result = EdgePatchOmission.Apply(
                Source,
                Grid,
                Options,
                Obstacles.EMPTY_OBSTACLES);

            Assert.Same(Source, Result.PassResult);
            Assert.Empty(Result.OmittedCutCells);
        }

        /// <summary>
        /// Verifies respects project wide percentage cap.
        /// </summary>

        [Fact]
        public void RespectsProjectWidePercentageCap
            (
            )
        {
            Grid Grid = MakeInsetGrid();
            PassResult Source = MakeMixedPassResult(Grid, 0.10);
            EdgePatchOmission.Settings Options = EnabledOptions();
            Options.MaximumProjectPercent = 0.01;

            EdgePatchOmissionResult Result = EdgePatchOmission.Apply(
                Source,
                Grid,
                Options,
                Obstacles.EMPTY_OBSTACLES);

            Assert.Same(Source, Result.PassResult);
            Assert.Empty(Result.OmittedCutCells);
        }

        /// <summary>
        /// Builds mixed pass result.
        /// </summary>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <param name="EdgeDepthM">The edge depth m used by the operation.</param>
        /// <returns>The resulting mixed pass result.</returns>

        private static PassResult MakeMixedPassResult
            (
            Grid Grid,
            Double EdgeDepthM
            )
        {
            Double CutPerM = Grid.AreaM2 * Constants.M3_TO_CY;
            Double FillPerM = CutPerM * Grid.Compaction;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = new()
            {
                ((2, 2), EdgeDepthM),
                ((5, 5), 0.20),
                ((6, 5), 0.20),
            };
            Double CutDepthM = EdgeDepthM + 0.40;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells = new()
            {
                ((8, 8), (CutDepthM * CutPerM) / FillPerM),
            };
            Pass Pass = Passes.BuildPass(
                Grid,
                CutCells,
                FillCells,
                CutDepthM * CutPerM,
                Obstacles.EMPTY_OBSTACLES,
                true);
            return new PassResult(new List<Pass> { Pass }, 1, 0.0, 0.0);
        }

        /// <summary>
        /// Verifies enabled options.
        /// </summary>
        /// <returns>The resulting enabled options.</returns>

        private static EdgePatchOmission.Settings EnabledOptions
            (
            ) => new EdgePatchOmission.Settings
            {
                Enabled = true,
                MaximumCells = 1,
                MaximumPatchBcy = 2.0,
                MaximumProjectBcy = 10.0,
                MaximumProjectPercent = 100.0,
            };

        /// <summary>
        /// Verifies cut bcy.
        /// </summary>
        /// <param name="Pass">The pass used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting cut bcy.</returns>

        private static Double CutBcy
            (
            Pass Pass,
            Grid Grid
            ) => Pass.CutCells.Sum(Entry =>
                Entry.DepthM * Grid.AreaM2 * Constants.M3_TO_CY);

        /// <summary>
        /// Verifies fill bcy.
        /// </summary>
        /// <param name="Pass">The pass used by the operation.</param>
        /// <param name="Grid">Field grid used by the operation.</param>
        /// <returns>The resulting fill bcy.</returns>

        private static Double FillBcy
            (
            Pass Pass,
            Grid Grid
            ) => Pass.FillCells.Sum(Entry =>
                Entry.DepthM * Grid.AreaM2 * Constants.M3_TO_CY * Grid.Compaction);

        /// <summary>
        /// Verifies assert close.
        /// </summary>
        /// <param name="Expected">The expected used by the operation.</param>
        /// <param name="Actual">The actual used by the operation.</param>

        private static void AssertClose
            (
            Double Expected,
            Double Actual
            ) => Assert.InRange(Actual, Expected - 1e-8, Expected + 1e-8);

        /// <summary>
        /// Builds inset grid.
        /// </summary>
        /// <returns>The resulting inset grid.</returns>

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
