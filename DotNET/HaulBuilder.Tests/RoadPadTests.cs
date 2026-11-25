using System;
using System.Collections.Generic;
using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Roads and well pads: relative surface, earthwork balance, and the road stage.
    /// </summary>
    public sealed class RoadPadTests
    {
        /// <summary>Feet in the default blade, and therefore one lane.</summary>
        private const Double LANE_FT = 8.5;

        /// <summary>
        /// Defaults build a 14 ft finished top on two pan lanes, and the toe sits 10 ft off the centreline.
        /// </summary>
        [Fact]
        public void DefaultsMatchNrcsAndTheDesignVehicle()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            Assert.Equal(14.0, Options.RoadNrcsMinRoadbedFt);
            Assert.Equal(2, Options.RoadMinLanes);
            Assert.Equal(1.0, Options.RoadHeightFt);
            Assert.Equal(3.0, Options.RoadSideSlope);
            Assert.Equal(2.0, Options.RoadCrossSlopePct);
            Assert.Equal(8.0, Options.RoadMaxGradePct);
            Assert.Equal(42.0, Options.PadDesignTurnRadiusFt);
            Assert.Equal(3.0, Options.PadClearanceFt);
            Assert.Equal(10.0, Options.PadExclusionRadiusFt);
            Assert.Equal(1.5, Options.PadSlopeAwayPct);
            Assert.Equal(3.0, Options.PadSideSlope);
            Assert.Equal(0.0, Options.BladeShapingHoursPerFt);
            Assert.Equal(0.0, Options.BladeShapingHoursPerPad);
            Assert.Equal(0.15, Options.RoadMinTripLoadPct);
            Assert.Equal(100.0, Options.RoadBladeMoveMaxFt);
            Assert.Equal(2, RoadPadBuild.PanLaneCount(LANE_FT, 14.0, 2, 3.0, 1.0));
            Assert.Equal(14.0, RoadPadBuild.TopWidthFt(LANE_FT, 14.0, 2, 3.0, 1.0));
            Assert.Equal(45.0, RoadPadBuild.PadTopRadiusFt(42.0, 3.0));
            RoadCentreline Road = Line(0.0, 0.0, 0.0, 10.0);
            Assert.Equal(10.0, RoadPadBuild.RequiredCentrelineInsetFt(Options, Road), 3);
        }

        /// <summary>
        /// The shoulder is the stated height, the crown rises to the centreline, and the slope wraps the end.
        /// </summary>
        [Fact]
        public void CrossSectionFollowsTheCentreline()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            Double Feet = Constants.FEET_TO_METERS;
            RoadCentreline Road = Line(0.0, 0.0, 0.0, 100.0 * Feet);
            // Finished top is 14 ft. Centreline = 1 + 0.02 * 7 = 1.14 ft. Shoulder at 7 ft is 1.00 ft.
            Assert.InRange(FeetOf(RoadPadBuild.RoadHeightM(0.0, 50.0 * Feet, Road, Options)), 1.139, 1.141);
            Assert.InRange(FeetOf(RoadPadBuild.RoadHeightM(4.0 * Feet, 50.0 * Feet, Road, Options)), 1.059, 1.061);
            Assert.InRange(FeetOf(RoadPadBuild.RoadHeightM(7.0 * Feet, 50.0 * Feet, Road, Options)), 0.999, 1.001);
            Assert.InRange(FeetOf(RoadPadBuild.RoadHeightM(8.5 * Feet, 50.0 * Feet, Road, Options)), 0.499, 0.501);
            Assert.InRange(FeetOf(RoadPadBuild.RoadHeightM(10.0 * Feet, 50.0 * Feet, Road, Options)), -0.001, 0.001);
            Assert.InRange(FeetOf(RoadPadBuild.RoadHeightM(20.0 * Feet, 50.0 * Feet, Road, Options)), -0.001, 0.001);
            // Past the end, distance is to the end point, so the section wraps around it.
            Assert.InRange(FeetOf(RoadPadBuild.RoadHeightM(0.0, 105.0 * Feet, Road, Options)), 1.039, 1.041);
        }

        /// <summary>
        /// Untouched defaults build two pan lanes and a 14 ft finished top, and do not set a roadbed override.
        /// </summary>
        [Fact]
        public void DefaultSettingsBuildTwoLanes()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            RoadPadSurface Surface = PrepareRoad(Options, 200.0, 0.0);
            Assert.Equal(14.0, Surface.Roads[0].TopWidthFt, 3);
            Assert.Equal(2, Surface.Roads[0].LaneCount);
            Assert.Equal(17.0, Surface.Roads[0].PanWidthFt, 3);
            Assert.Equal(14.0, Surface.Roads[0].NrcsMinRoadbedFt, 3);
            Assert.False(Surface.Roads[0].HasRoadbedOverride);
            Assert.Equal(1.140, Surface.Roads[0].CentrelineFt, 3);
            String Report = Surface.FormatReport();
            Assert.Contains("NRCS minimum 14.0 ft", Report, StringComparison.Ordinal);
            Assert.DoesNotContain("Roadbed override", Report, StringComparison.Ordinal);
        }

        /// <summary>
        /// The flat pan carries the finished section's volume per foot, and a grid-aligned road is two cells wide.
        /// </summary>
        [Fact]
        public void PanSurfaceMatchesTheFinishedSection()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            Double Finished = RoadPadBuild.FinishedSectionSqFt(LANE_FT, 14.0, 2, 3.0, 1.0, 2.0);
            Double CellFt = LANE_FT;
            Double Cell = CellFt * Constants.FEET_TO_METERS;
            (Grid Grid, List<(Double X, Double Y)> Boundary) Built = Rectangle(85.0, 200.0, CellFt, 0.0, 200.0);
            // The centreline sits on the grid line between two lanes, so each lane is one whole cell.
            Double LineX = Built.Grid.MinX + (5.0 * Cell);
            Double Inset = 20.0 * Constants.FEET_TO_METERS;
            Double North = 200.0 * Constants.FEET_TO_METERS;
            RoadCentreline Road = Line(LineX, Inset, LineX, North - Inset);
            Options.Roads.Add(Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Assert.Equal(2, Surface.Roads[0].LaneCount);
            Assert.Equal(17.0, Surface.Roads[0].PanWidthFt, 3);
            Double Feet = Constants.FEET_TO_METERS;
            Double MidY = (Inset + (North - Inset)) * Constants.HALF_SCALE;
            Double Sum = 0.0;
            Double StepFt = 0.05;
            for (Double Offset = -12.0; Offset < 12.0; Offset += StepFt)
            {
                Double HeightFt = RoadPadBuild.RoadHeightM((LineX) + (Offset * Feet), MidY, Road, Options) / Feet;
                Sum += HeightFt * StepFt;
            }

            Double Pan = Surface.Roads[0].BuildHeightFt * Surface.Roads[0].PanWidthFt;
            Assert.InRange(Pan, Finished * 0.995, Finished * 1.005);
            Assert.InRange(Pan, Sum * 0.995, Sum * 1.005);
            Int32 MidRow = Built.Grid.Height / 2;
            Int32 Across = 0;
            for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
            {
                if (Surface.PanM[MidRow, Bx] > 1e-6)
                {
                    Across += 1;
                    Assert.InRange(Math.Abs(Surface.PanM[MidRow, Bx] - (Surface.Roads[0].BuildHeightFt * Feet)), 0.0, 1e-9);
                }
            }

            Assert.Equal(2, Across);
            Assert.Equal(0.0, Surface.Roads[0].PanShiftFt, 3);
        }

        /// <summary>
        /// A centreline through the middle of a cell row snaps onto two whole cells and keeps the section area.
        /// </summary>
        [Fact]
        public void PanSnapCentresARoadThatRunsThroughCellCentres()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            Double Finished = RoadPadBuild.FinishedSectionSqFt(LANE_FT, 14.0, 2, 3.0, 1.0, 2.0);
            Double CellFt = LANE_FT;
            Double Cell = CellFt * Constants.FEET_TO_METERS;
            (Grid Grid, List<(Double X, Double Y)> Boundary) Built = Rectangle(200.0, 85.0, CellFt, 0.0, 85.0);
            // Row 4's centre is the middle of that cell row, halfway between two boundaries.
            Double LineY = Built.Grid.MinY + (4.5 * Cell);
            Double West = Built.Grid.MinX + (3.0 * Cell);
            Double East = Built.Grid.MinX + (21.0 * Cell);
            Options.Roads.Add(Line(West, LineY, East, LineY));
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Assert.InRange(Math.Abs(Surface.Roads[0].PanShiftFt), 4.24, 4.26);
            Assert.Contains("Pan shift 4.25 ft", Surface.FormatReport(), StringComparison.Ordinal);
            Int32 MidColumn = 10;
            Int32 Across = 0;
            Double AreaFt = 0.0;
            Double Feet = Constants.FEET_TO_METERS;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                if (Surface.PanM[By, MidColumn] <= 1e-6)
                {
                    continue;
                }

                Across += 1;
                Assert.Equal(1.0, Surface.PanCoverage[By, MidColumn], 6);
                AreaFt += (Surface.PanM[By, MidColumn] / Feet) * CellFt;
            }

            Assert.Equal(2, Across);
            Assert.InRange(AreaFt, Finished * 0.995, Finished * 1.005);
        }

        /// <summary>
        /// A diagonal road is not snapped. Part of its pan volume sits in cells that are not fully covered.
        /// </summary>
        [Fact]
        public void DiagonalPanReportsPartialCells()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            Double CellFt = LANE_FT;
            Double Cell = CellFt * Constants.FEET_TO_METERS;
            (Grid Grid, List<(Double X, Double Y)> Boundary) Built = Rectangle(120.0, 120.0, CellFt, 0.0, 120.0);
            Double X0 = Built.Grid.MinX + (2.0 * Cell);
            Double Y0 = Built.Grid.MinY + (2.0 * Cell);
            Double X1 = Built.Grid.MinX + (10.0 * Cell);
            Double Y1 = Built.Grid.MinY + (10.0 * Cell);
            Options.Roads.Add(Line(X0, Y0, X1, Y1));
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Assert.Equal(0.0, Surface.Roads[0].PanShiftFt, 3);
            Assert.True(Surface.Roads[0].PartialPanFraction > 0.0);
            Boolean Partial = false;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    Double Coverage = Surface.PanCoverage[By, Bx];
                    if ((Coverage > 0.02) && (Coverage < 0.98))
                    {
                        Partial = true;
                    }
                }
            }

            Assert.True(Partial);
            Assert.Contains("Partial pan", Surface.FormatReport(), StringComparison.Ordinal);
        }

        /// <summary>
        /// A straight 1,000 ft road, including both side slopes and both end caps, matches the solid.
        /// </summary>
        [Fact]
        public void StraightRoadVolumeMatchesThePrism()
        {
            HaulBuilderOptions Crowned = new HaulBuilderOptions();
            // A 2 ft cell keeps the fixed 4 by 4 sample inside 1% of the solid.
            // An 8.5 ft cell puts the toe in the outer sample and reads about 1.1% low.
            RoadPadSurface WithCrown = PrepareRoad(Crowned, 1000.0, 0.0, 2.0);
            Double CrownedBcy = StraightRoadBcy(1000.0, 2.0);
            Assert.InRange(WithCrown.RoadFillBcy, CrownedBcy * 0.99, CrownedBcy * 1.01);
            Assert.Equal(14.0, WithCrown.Roads[0].TopWidthFt, 3);
            Assert.Equal(14.0, WithCrown.Roads[0].NrcsMinRoadbedFt, 3);
            Assert.Equal(1.0, WithCrown.Roads[0].HeightFt, 3);
            Assert.Equal(1.140, WithCrown.Roads[0].CentrelineFt, 3);
            Assert.False(WithCrown.Roads[0].HasRoadbedOverride);
            Assert.True(WithCrown.RoadTopOk, WithCrown.RoadTopFailure);
            Assert.DoesNotContain(WithCrown.Warnings, Warning => Warning.Contains("boundary", StringComparison.Ordinal));

            HaulBuilderOptions Flat = new HaulBuilderOptions();
            Flat.RoadCrossSlopePct = 0.0;
            RoadPadSurface WithoutCrown = PrepareRoad(Flat, 1000.0, 0.0, 2.0);
            Double FlatBcy = StraightRoadBcy(1000.0, 0.0);
            Assert.InRange(WithoutCrown.RoadFillBcy, FlatBcy * 0.99, FlatBcy * 1.01);
            Assert.Equal(WithCrown.RoadFillBcy, WithCrown.Roads[0].BankBcy);
            Assert.DoesNotContain(WithoutCrown.Warnings, Warning => Warning.Contains("boundary", StringComparison.Ordinal));
        }

        /// <summary>
        /// A lower roadbed override is still reported as an override. One lane cannot meet it after the side slope is removed, so the pan stays two lanes.
        /// </summary>
        [Fact]
        public void SingleLaneUsesARoadbedOverride()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            Assert.Equal(14.0, Options.RoadNrcsMinRoadbedFt);
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(200.0, 0.0);
            Built.Road.MinLanes = 1;
            Built.Road.MinRoadbedFt = LANE_FT;
            Options.Roads.Add(Built.Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Assert.Equal(14.0, Surface.Roads[0].TopWidthFt, 3);
            Assert.Equal(2, Surface.Roads[0].LaneCount);
            Assert.Equal(14.0, Surface.Roads[0].NrcsMinRoadbedFt, 3);
            Assert.True(Surface.Roads[0].HasRoadbedOverride);
            Assert.Equal(LANE_FT, Surface.Roads[0].RoadbedOverrideFt, 3);
            Assert.Equal(1.140, Surface.Roads[0].CentrelineFt, 3);
            String Report = Surface.FormatReport();
            Assert.Contains("Roadbed override 8.5 ft", Report, StringComparison.Ordinal);
            Assert.Contains("NRCS minimum 14.0 ft", Report, StringComparison.Ordinal);
            Assert.DoesNotContain("NRCS minimum 8.5", Report, StringComparison.Ordinal);
        }

        /// <summary>
        /// An exact duplicate centreline is dropped, so the fill is the single road.
        /// </summary>
        [Fact]
        public void DuplicateRoadsDoNotAddFill()
        {
            HaulBuilderOptions One = new HaulBuilderOptions();
            One.RoadCrossSlopePct = 0.0;
            RoadPadSurface Single = PrepareRoad(One, 400.0, 0.0);
            HaulBuilderOptions Two = new HaulBuilderOptions();
            Two.RoadCrossSlopePct = 0.0;
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(400.0, 0.0);
            Two.Roads.Add(Built.Road);
            Two.Roads.Add(new RoadCentreline(Built.Road.Vertices));
            RoadPadSurface Doubled = RoadPadBuild.Prepare(Built.Grid, Two, Built.Boundary);
            Assert.Equal(Single.RoadFillBcy, Doubled.RoadFillBcy, 6);
            Assert.Contains(Doubled.Warnings, Warning => Warning.Contains("Dropped duplicate", StringComparison.Ordinal));
            Assert.Single(Doubled.Roads);
        }

        /// <summary>
        /// A right-angle crossing counts the shared prism once, and both crowns meet at the same height.
        /// </summary>
        [Fact]
        public void CrossingRoadsSubtractTheOverlap()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            Options.RoadCrossSlopePct = 0.0;
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline SouthRoad) Field = RoadField(400.0, 0.0);
            RoadCentreline EastRoad = EastWest(Field.Grid, Field.Boundary);
            HaulBuilderOptions OnlySouth = CopyOptions(Options);
            OnlySouth.Roads.Add(new RoadCentreline(Field.SouthRoad.Vertices));
            HaulBuilderOptions OnlyEast = CopyOptions(Options);
            OnlyEast.Roads.Add(new RoadCentreline(EastRoad.Vertices));
            HaulBuilderOptions Both = CopyOptions(Options);
            Both.Roads.Add(new RoadCentreline(Field.SouthRoad.Vertices));
            Both.Roads.Add(new RoadCentreline(EastRoad.Vertices));
            RoadPadSurface South = RoadPadBuild.Prepare(CloneGrid(Field.Grid), OnlySouth, Field.Boundary);
            RoadPadSurface East = RoadPadBuild.Prepare(CloneGrid(Field.Grid), OnlyEast, Field.Boundary);
            RoadPadSurface Union = RoadPadBuild.Prepare(Field.Grid, Both, Field.Boundary);
            Double Overlap = RoadPadBuild.OverlapBankBcy(CloneGrid(Field.Grid), Field.SouthRoad, EastRoad, Options, Field.Boundary);
            Double Expected = South.RoadFillBcy + East.RoadFillBcy - Overlap;
            Assert.InRange(Math.Abs(Union.RoadFillBcy - Expected), 0.0, 0.05);
            Assert.True(Union.RoadFillBcy < (South.RoadFillBcy + East.RoadFillBcy));
            (Double X, Double Y) Crossing = CrossingPoint(Field.SouthRoad, EastRoad);
            Double SouthHeight = RoadPadBuild.RoadHeightM(Crossing.X, Crossing.Y, Field.SouthRoad, Options);
            Double EastHeight = RoadPadBuild.RoadHeightM(Crossing.X, Crossing.Y, EastRoad, Options);
            Assert.InRange(Math.Abs(SouthHeight - EastHeight), 0.0, 0.001);
            Assert.InRange(Math.Abs(SouthHeight - Constants.FEET_TO_METERS), 0.0, 0.001);
            Assert.InRange(Union.Roads[0].SharedLengthFt, 14.0, 20.0);
            Assert.Equal(Union.RoadFillBcy, Union.Roads[0].BankBcy + Union.Roads[1].BankBcy, 6);
        }

        /// <summary>
        /// A pad shoulder is 1 ft at the top radius and rises toward the well. The casing is not pan work.
        /// </summary>
        [Fact]
        public void PadVolumeMatchesTheSlopedCone()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, WellPad Pad) Built = PadField();
            Options.WellPads.Add(Built.Pad);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, null);
            Double Expected = SlopedPadBcy(45.0, 48.0, 1.0, 0.015);
            Assert.InRange(Surface.RoadFillBcy, Expected * 0.99, Expected * 1.01);
            Assert.Equal(45.0, Surface.Pads[0].TopRadiusFt, 3);
            Double ShoulderM = RoadPadBuild.PadHeightM(
                Built.Pad.X + (45.0 * Constants.FEET_TO_METERS),
                Built.Pad.Y,
                Built.Pad,
                Options);
            Assert.InRange(Math.Abs(ShoulderM - (1.0 * Constants.FEET_TO_METERS)), 0.0, 0.001);
            Assert.True(Surface.Pads[0].ExcludedBcy > 0.0);
            Boolean Found = false;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (!Surface.Excluded[By, Bx])
                    {
                        continue;
                    }

                    Found = true;
                    Assert.True(Surface.RelM[By, Bx] > 0.0);
                }
            }

            Assert.True(Found);
            List<Pass> Passes = RoadPadBuild.BuildPasses(Built.Grid, Surface, Options);
            foreach (Pass Pass in Passes)
            {
                Assert.Equal(HaulStage.Road, Pass.Stage);
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Pass.FillCells)
                {
                    Assert.False(Surface.Excluded[Fill.Cell.By, Fill.Cell.Bx]);
                    Assert.True(Surface.RelM[Fill.Cell.By, Fill.Cell.Bx] > 0.0);
                }
            }

            Assert.Equal(Surface.RoadFillBcy, Surface.Pads[0].BankBcy);
            // The casing is fill the pan did not take, so the blade figure includes it.
            Assert.True(Surface.BladeMoveBcy > Surface.Pads[0].ExcludedBcy * 0.5);
        }

        /// <summary>
        /// On a slope, every road-top cell stays the crowned height above the fitted field design.
        /// </summary>
        [Fact]
        public void SlopingFieldKeepsTheRoadHeight()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(60.0, 0.02, 0.5, 40.0);
            Options.Roads.Add(Built.Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Assert.True(Surface.RoadTopOk, Surface.RoadTopFailure);
            Double HalfLane = LANE_FT * Constants.FEET_TO_METERS * Constants.HALF_SCALE;
            Double HalfTop = 7.0 * Constants.FEET_TO_METERS;
            Double Cross = Options.RoadCrossSlopePct / 100.0;
            Double Shoulder = Options.RoadHeightFt * Constants.FEET_TO_METERS;
            Int32 Checked = 0;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (!Built.Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double Applied = Surface.FinalDesignM[By, Bx] - Surface.FieldDesignM[By, Bx];
                    Assert.InRange(Math.Abs(Applied - Surface.RelM[By, Bx]), 0.0, 1e-9);
                    Double Distance = DistanceToRoad(Built.Road, Built.Grid.CenterX[By, Bx], Built.Grid.CenterY[By, Bx]);
                    if (Distance > HalfLane)
                    {
                        continue;
                    }

                    Double Expected = Shoulder + (Cross * (HalfTop - Distance));
                    Assert.InRange(Math.Abs(Applied - Expected), 0.0, 0.001);
                    Checked += 1;
                }
            }

            Assert.True(Checked > 0);
            AssertBalanced(Built.Grid, Surface);
            Assert.True(Surface.FieldShiftFt < 0.0);
        }

        /// <summary>
        /// The fitted field absorbs the road fill at the project cut/fill ratio.
        /// </summary>
        [Fact]
        public void FieldBalancesWithTheRoadFill()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            RoadPadSurface Surface = PrepareRoad(Options, 400.0, 0.02);
            Assert.True(Surface.FieldShiftFt < 0.0);
            String Report = Surface.FormatReport();
            Assert.Contains("Road fill", Report, StringComparison.Ordinal);
            Assert.Contains("Field design shift", Report, StringComparison.Ordinal);
        }

        /// <summary>
        /// A centreline drawn on the grid edge asks for the 10 ft offset. The volume tests keep the road inside.
        /// </summary>
        [Fact]
        public void EdgeRoadWarnsWithTheRequiredOffset()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary) Built = Rectangle(80.0, 200.0, 8.5);
            // The west side of the rectangle is the grid edge.
            Options.Roads.Add(Line(Built.Boundary[0].X, Built.Boundary[0].Y, Built.Boundary[0].X, Built.Boundary[3].Y));
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Assert.Contains(Surface.Warnings, Warning => Warning.Contains("Required offset 10.0 ft", StringComparison.Ordinal));
        }

        /// <summary>
        /// Road passes are the road stage, and they fill only road or pad cells.
        /// </summary>
        [Fact]
        public void RoadStageFillsOnlyRoadCells()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(200.0, 0.0);
            Options.Roads.Add(Built.Road);
            Options.BladeShapingHoursPerFt = 0.01;
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            List<Pass> Passes = RoadPadBuild.BuildPasses(Built.Grid, Surface, Options);
            Assert.NotEmpty(Passes);
            Assert.InRange(Math.Abs(Surface.BladeShapingHours - (0.01 * Surface.CentrelineLengthFt)), 0.0, 1e-9);
            Double Reserved = 0.0;
            Double CutBank = Built.Grid.AreaM2 * Constants.M3_TO_CY;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    Reserved += Surface.ReservedDepthM[By, Bx] * CutBank;
                }
            }

            Assert.InRange(Math.Abs(Reserved - Surface.PanImportBcy), 0.0, 0.1);
            Assert.Equal(0, RoadPadBuild.CountCutBelowAndFilled(Built.Grid, Surface, Passes));
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.ReservedDepthM[By, Bx] > 0.0)
                    {
                        Assert.True(Surface.PanM[By, Bx] <= 1e-8);
                    }
                }
            }
            foreach (Pass Pass in Passes)
            {
                Assert.Equal(HaulStage.Road, Pass.Stage);
                Assert.NotEmpty(Pass.FillCells);
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Fill in Pass.FillCells)
                {
                    Assert.True(Surface.PanM[Fill.Cell.By, Fill.Cell.Bx] > 0.0);
                    Assert.False(Surface.Excluded[Fill.Cell.By, Fill.Cell.Bx]);
                    Assert.True(Surface.OwnerIndex[Fill.Cell.By, Fill.Cell.Bx] >= 0);
                }
            }
        }

        /// <summary>
        /// Grade, a tight bend, an unreached pad, and disagreeing crowns are warnings.
        /// </summary>
        [Fact]
        public void ValidationWarningsFire()
        {
            HaulBuilderOptions Steep = new HaulBuilderOptions();
            (Grid GradeGrid, List<(Double X, Double Y)> GradeBoundary, RoadCentreline GradeRoad) Grade = RoadField(200.0, 0.12);
            Steep.Roads.Add(Grade.GradeRoad);
            RoadPadSurface GradeSurface = RoadPadBuild.Prepare(Grade.GradeGrid, Steep, Grade.GradeBoundary);
            Assert.Contains(GradeSurface.Warnings, Warning => Warning.Contains("exceeds", StringComparison.Ordinal));

            HaulBuilderOptions Bend = new HaulBuilderOptions();
            (Grid BendGrid, List<(Double X, Double Y)> BendBoundary) BendField = Rectangle(200.0, 200.0, 8.5);
            Double Feet = Constants.FEET_TO_METERS;
            Bend.Roads.Add(new RoadCentreline(new List<(Double X, Double Y)>
            {
                (50.0 * Feet, 50.0 * Feet),
                (100.0 * Feet, 50.0 * Feet),
                (100.0 * Feet, 100.0 * Feet),
            }));
            RoadPadSurface BendSurface = RoadPadBuild.Prepare(BendField.BendGrid, Bend, BendField.BendBoundary);
            Assert.Contains(BendSurface.Warnings, Warning => Warning.Contains("inside edge radius", StringComparison.Ordinal));

            HaulBuilderOptions Lonely = new HaulBuilderOptions();
            (Grid LonelyGrid, List<(Double X, Double Y)> LonelyBoundary) LonelyField = Rectangle(400.0, 400.0, 8.5);
            Lonely.Roads.Add(Line(20.0 * Feet, 20.0 * Feet, 20.0 * Feet, 80.0 * Feet));
            Lonely.WellPads.Add(new WellPad(150.0 * Feet, 200.0 * Feet));
            RoadPadSurface LonelySurface = RoadPadBuild.Prepare(LonelyField.LonelyGrid, Lonely, LonelyField.LonelyBoundary);
            Assert.Contains(LonelySurface.Warnings, Warning => Warning.Contains("not reached", StringComparison.Ordinal));

            HaulBuilderOptions Conflict = new HaulBuilderOptions();
            (Grid ConflictGrid, List<(Double X, Double Y)> ConflictBoundary) ConflictField = Rectangle(160.0, 400.0, 8.5);
            RoadCentreline Low = Line(20.0 * Feet, 20.0 * Feet, 20.0 * Feet, 380.0 * Feet);
            RoadCentreline High = Line(21.0 * Feet, 20.0 * Feet, 21.0 * Feet, 380.0 * Feet);
            High.HeightFt = 2.0;
            Conflict.Roads.Add(Low);
            Conflict.Roads.Add(High);
            RoadPadSurface ConflictSurface = RoadPadBuild.Prepare(ConflictField.ConflictGrid, Conflict, ConflictField.ConflictBoundary);
            Assert.Contains(ConflictSurface.Warnings, Warning => Warning.Contains("0.1 ft", StringComparison.Ordinal));
        }

        /// <summary>
        /// Height of a sample in feet.
        /// </summary>
        /// <param name="Metres">Height in metres.</param>
        /// <returns>Height in feet.</returns>
        private static Double FeetOf
            (
            Double Metres
            )
        {
            return Metres / Constants.FEET_TO_METERS;
        }

        /// <summary>
        /// Builds a north-south road on a fresh field and fits it. The road, including its slopes, sits inside the grid.
        /// </summary>
        /// <param name="Options">Options that receive the road.</param>
        /// <param name="LengthFt">Centreline length in feet.</param>
        /// <param name="Slope">Field slope, rise over run, along the road.</param>
        /// <param name="CellFt">Cell size in feet.</param>
        /// <returns>The fitted surface.</returns>
        private static RoadPadSurface PrepareRoad
            (
            HaulBuilderOptions Options,
            Double LengthFt,
            Double Slope,
            Double CellFt = 8.5
            )
        {
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(LengthFt, Slope, CellFt);
            Options.Roads.Add(Built.Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            AssertBalanced(Built.Grid, Surface);
            return Surface;
        }

        /// <summary>
        /// A trip keeps spreading while the next cell is ahead on the same lane or a
        /// forward S-bend away. It stops short of a full bowl when the other lane
        /// would mean turning back. Pan loads stay at or above the minimum. The
        /// placed volume, pan plus blade, still matches the pan.
        /// </summary>
        [Fact]
        public void ShortLanesSpreadUntilTheBowlIsFull()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(200.0, 0.0);
            Options.Roads.Add(Built.Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            List<Pass> Passes = RoadPadBuild.BuildPasses(Built.Grid, Surface, Options);
            Assert.True(Passes.Count > 1);
            Double Bowl = Options.ScraperCapacityLcy / Options.SwellFactor;
            Double Placed = Surface.BladeMoveBcy;
            for (Int32 Index = 0; Index < Passes.Count; Index += 1)
            {
                Placed += Passes[Index].LoadBcy;
                Assert.NotEmpty(Passes[Index].FillCells);
                Assert.True(Passes[Index].LoadBcy + 1e-6 >= (Bowl * Options.RoadMinTripLoadPct));
            }

            Double PanGap = 0.0;
            Double FillPerM = Built.Grid.AreaM2 * Constants.M3_TO_CY * Built.Grid.Compaction;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-8)
                    {
                        continue;
                    }

                    Double Gap = (Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx]) - Built.Grid.Existing[By, Bx];
                    if (Gap > 0.0)
                    {
                        PanGap += Gap * FillPerM;
                    }
                }
            }

            Double VolumeGap = Math.Abs(Placed - PanGap);
            Assert.True(
                VolumeGap <= Math.Max(1.0, PanGap * 0.05),
                String.Format(
                    System.Globalization.CultureInfo.InvariantCulture,
                    "placed {0:F3} gap {1:F3} blade {2:F3} trips {3} {4}",
                    Placed,
                    PanGap,
                    Surface.BladeMoveBcy,
                    Passes.Count,
                    String.Join(" | ", Surface.Warnings)));
            Assert.True(Passes.Count > 1);
            String Stage = Surface.FormatStage();
            Assert.Contains("hauls", Stage, StringComparison.Ordinal);
            Assert.Contains("mean load", Stage, StringComparison.Ordinal);
            Assert.Contains("pan ", Stage, StringComparison.Ordinal);
            Assert.Contains("blade ", Stage, StringComparison.Ordinal);
            Assert.Contains("mean loaded carry", Stage, StringComparison.Ordinal);
            Assert.Contains("h/BCY", Stage, StringComparison.Ordinal);
            Surface.FieldHoursPerBcy = 0.02;
            Assert.Contains("vs field 0.0200 h/BCY", Surface.FormatStage(), StringComparison.Ordinal);
            Assert.Contains(Surface.FormatStage(), Surface.FormatReport(), StringComparison.Ordinal);
            Assert.DoesNotContain(Surface.Warnings, Warning => Warning.Contains("flow path", StringComparison.Ordinal) || Warning.Contains("culvert", StringComparison.Ordinal) || Warning.Contains("depression", StringComparison.Ordinal));
            Assert.DoesNotContain(Surface.Warnings, Warning => Warning.Contains("not built", StringComparison.Ordinal));
        }

        /// <summary>
        /// An end on the grid edge reports that end and the bank volume left off the field.
        /// </summary>
        [Fact]
        public void OpenEndReportsTheVolumeLeftOffTheGrid()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary) Built = Rectangle(80.0, 200.0, 8.5);
            Options.Roads.Add(Line(Built.Boundary[0].X, Built.Boundary[0].Y, Built.Boundary[0].X, Built.Boundary[3].Y));
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Assert.Contains(Surface.Warnings, Warning => Warning.Contains("open end", StringComparison.Ordinal) && Warning.Contains("not built", StringComparison.Ordinal));
            Assert.Contains("not built", Surface.FormatReport(), StringComparison.Ordinal);
        }

        /// <summary>
        /// A north-south road centered in a field. The centreline is inset so the side slopes and end caps stay inside.
        /// </summary>
        /// <param name="LengthFt">Road length in feet.</param>
        /// <param name="Slope">Rise over run along northing.</param>
        /// <param name="CellFt">Cell size in feet.</param>
        /// <param name="WidthFt">Field width in feet.</param>
        /// <param name="InsetFt">Clearance from the road ends to the north and south grid edges, in feet.</param>
        /// <returns>The grid, its boundary, and the road.</returns>
        private static (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) RoadField
            (
            Double LengthFt,
            Double Slope,
            Double CellFt = 8.5,
            Double WidthFt = 93.5,
            Double InsetFt = 20.0
            )
        {
            // Twenty feet clears the 10 ft toe and one sample past it.
            Double FieldLengthFt = LengthFt + (2.0 * InsetFt);
            (Grid Grid, List<(Double X, Double Y)> Boundary) Built = Rectangle(WidthFt, FieldLengthFt, CellFt, Slope, FieldLengthFt);
            Double MidX = (Built.Boundary[0].X + Built.Boundary[1].X) * Constants.HALF_SCALE;
            Int32 Columns = Built.Grid.Width;
            if ((Columns % 2) == 1)
            {
                MidX = Built.Grid.MinX + ((Columns / 2) + Constants.HALF_SCALE) * Built.Grid.GridSizeM;
            }

            Double InsetM = InsetFt * Constants.FEET_TO_METERS;
            Double NorthM = FieldLengthFt * Constants.FEET_TO_METERS;
            RoadCentreline Road = Line(MidX, InsetM, MidX, NorthM - InsetM);
            return (Built.Grid, Built.Boundary, Road);
        }

        /// <summary>
        /// An east-west road through the middle of a north-south road's field.
        /// </summary>
        /// <param name="Grid">Field the road crosses.</param>
        /// <param name="Boundary">Field boundary.</param>
        /// <returns>The crossing road.</returns>
        private static RoadCentreline EastWest
            (
            Grid Grid,
            List<(Double X, Double Y)> Boundary
            )
        {
            Double MidY = Grid.MinY + (Grid.Height * Constants.HALF_SCALE * Grid.GridSizeM);
            return Line(Boundary[0].X, MidY, Boundary[1].X, MidY);
        }

        /// <summary>
        /// A single pad on a fine grid, centered on a cell.
        /// </summary>
        /// <returns>The grid and the pad.</returns>
        private static (Grid Grid, WellPad Pad) PadField()
        {
            (Grid Grid, List<(Double X, Double Y)> Boundary) Built = Rectangle(141.0, 141.0, 1.0, 0.0, 141.0);
            Int32 Middle = Built.Grid.Width / 2;
            Double X = Built.Grid.MinX + ((Middle + Constants.HALF_SCALE) * Built.Grid.GridSizeM);
            Double Y = Built.Grid.MinY + ((Middle + Constants.HALF_SCALE) * Built.Grid.GridSizeM);
            return (Built.Grid, new WellPad(X, Y));
        }

        /// <summary>
        /// A two-lane bank that needs fill on both lanes changes lanes only by a forward
        /// S-bend of at least the Dubins room, and no trip turns back on the bank.
        /// </summary>
        [Fact]
        public void TwoLaneBankChangesLanesByAForwardSBend()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(400.0, 0.0);
            Options.Roads.Add(Built.Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Int32 WestColumn = Int32.MaxValue;
            Int32 EastColumn = Int32.MinValue;
            Double South = Double.PositiveInfinity;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-6)
                    {
                        continue;
                    }

                    WestColumn = Math.Min(WestColumn, Bx);
                    EastColumn = Math.Max(EastColumn, Bx);
                    South = Math.Min(South, Built.Grid.CenterY[By, Bx]);
                }
            }

            Assert.True(EastColumn > WestColumn);
            ClearStocks(Built.Grid, Surface);
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-6)
                    {
                        continue;
                    }

                    Double Target = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    Built.Grid.Existing[By, Bx] = Target;
                    Double AlongFt = (Built.Grid.CenterY[By, Bx] - South) / Constants.FEET_TO_METERS;
                    // Cut on the west lane, then fill on that lane, then fill one lane over.
                    // Neither fill run holds a bowl, so the trip has to S-bend forward.
                    if ((Bx == WestColumn) && (AlongFt >= 0.0) && (AlongFt <= 160.0))
                    {
                        Surface.StageCutM[By, Bx] = 0.08;
                    }

                    if ((Bx == WestColumn) && (AlongFt >= 220.0) && (AlongFt <= 300.0))
                    {
                        Built.Grid.Existing[By, Bx] = Target - 0.04;
                    }

                    if ((Bx == EastColumn) && (AlongFt >= 340.0) && (AlongFt <= 420.0))
                    {
                        Built.Grid.Existing[By, Bx] = Target - 0.04;
                    }
                }
            }

            List<Pass> Passes = RoadPadBuild.BuildPasses(Built.Grid, Surface, Options);
            Assert.NotEmpty(Passes);
            Assert.Equal(Passes.Count, Surface.RoadCourses.Count);
            Double QuotedFt = TripGuidance.ForwardSBendM(
                8.5 * Constants.FEET_TO_METERS,
                22.5 * Constants.FEET_TO_METERS) / Constants.FEET_TO_METERS;
            Assert.InRange(QuotedFt, 26.0, 26.6);
            Double RadiusM = Options.MinimumTurningDiameterFt * Constants.FEET_TO_METERS * Constants.HALF_SCALE;
            Boolean ChangedLane = false;
            foreach (TripGuidance.TripCourse Course in Surface.RoadCourses)
            {
                List<TripGuidance.CourseSegment> Fills = FillLanes(Course);
                if (Fills.Count == 0)
                {
                    continue;
                }

                Double Heading = LaneHeading(Fills[0]);
                foreach (TripGuidance.CourseSegment Lane in Fills)
                {
                    Double Turn = Math.Abs(DubinsConnector.WrapPi(LaneHeading(Lane) - Heading));
                    Assert.True(Turn < (20.0 * Math.PI / 180.0));
                }

                Int32 LastFill = -1;
                Int32 TurnAt = -1;
                TripGuidance.CourseSegment? PreviousFill = null;
                TripGuidance.CourseSegment? Between = null;
                for (Int32 Index = 0; Index < Course.Segments.Count; Index += 1)
                {
                    TripGuidance.CourseSegment Segment = Course.Segments[Index];
                    if (Segment.IsLane && (Segment.Action == TripGuidance.GuidanceAction.Fill))
                    {
                        if ((PreviousFill != null) && (Between != null))
                        {
                            AssertForwardBend(Between, PreviousFill, Segment, RadiusM);
                            ChangedLane = true;
                        }

                        PreviousFill = Segment;
                        Between = null;
                        LastFill = Index;
                    }
                    else if (!Segment.IsLane)
                    {
                        Between = Segment;
                        if (Segment.Action == TripGuidance.GuidanceAction.Turn)
                        {
                            TurnAt = Index;
                        }
                    }
                }

                Assert.True(TurnAt > LastFill);
            }

            Assert.True(ChangedLane);
        }

        /// <summary>
        /// Heading of a blade-down lane, from its first sample.
        /// </summary>
        /// <param name="Lane">Lane segment.</param>
        /// <returns>Radians anticlockwise from east.</returns>
        private static Double LaneHeading
            (
            TripGuidance.CourseSegment Lane
            )
        {
            return Lane.Pieces[0].Samples[0].HeadingRad;
        }

        /// <summary>
        /// Spread lanes of one trip, in drive order.
        /// </summary>
        /// <param name="Course">Guided trip.</param>
        /// <returns>The fill lanes.</returns>
        private static List<TripGuidance.CourseSegment> FillLanes
            (
            TripGuidance.TripCourse Course
            )
        {
            List<TripGuidance.CourseSegment> Lanes = new List<TripGuidance.CourseSegment>();
            foreach (TripGuidance.CourseSegment Segment in Course.Segments)
            {
                if (Segment.IsLane && (Segment.Action == TripGuidance.GuidanceAction.Fill))
                {
                    Lanes.Add(Segment);
                }
            }

            return Lanes;
        }

        /// <summary>
        /// The join between two spread lanes is a forward S-bend at least as long as the Dubins room.
        /// </summary>
        /// <param name="Join">Connector between the lanes.</param>
        /// <param name="From">Lane the machine left.</param>
        /// <param name="To">Lane the machine entered.</param>
        /// <param name="RadiusM">Turning radius.</param>
        private static void AssertForwardBend
            (
            TripGuidance.CourseSegment Join,
            TripGuidance.CourseSegment From,
            TripGuidance.CourseSegment To,
            Double RadiusM
            )
        {
            Assert.False(Join.IsLane);
            DubinsConnector.Pose Start = From.Pieces[From.Pieces.Count - 1].Samples[From.Pieces[From.Pieces.Count - 1].Samples.Count - 1];
            DubinsConnector.Pose End = To.Pieces[0].Samples[0];
            Assert.True(
                (Join.Word == "LSR") || (Join.Word == "RSL"),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "{0} action={1} len={2:F2} from=({3:F1},{4:F1}) h={5:F2} to=({6:F1},{7:F1}) h={8:F2}",
                    Join.Word,
                    Join.Action,
                    Join.LengthM,
                    Start.X,
                    Start.Y,
                    Start.HeadingRad,
                    End.X,
                    End.Y,
                    End.HeadingRad));
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double Forward = (Dx * Math.Cos(Start.HeadingRad)) + (Dy * Math.Sin(Start.HeadingRad));
            Double Side = (Dx * -Math.Sin(Start.HeadingRad)) + (Dy * Math.Cos(Start.HeadingRad));
            Double Room = TripGuidance.ForwardSBendM(Side, RadiusM);
            Assert.True(Forward + 0.2 >= Room);
            Assert.True(Join.LengthM + 0.05 >= Room);
            Double Turned = 0.0;
            foreach (TripGuidance.DrawnPiece Piece in Join.Pieces)
            {
                if (Piece.Kind != DubinsConnector.PieceKind.Straight)
                {
                    Turned += Piece.LengthM * Piece.CurvaturePerM;
                }
            }

            Assert.True(Turned < (150.0 * Math.PI / 180.0));
        }

        /// <summary>
        /// One trip cuts two lanes and then spreads two lanes, all the same way,
        /// and changes lanes only by a forward S-bend.
        /// </summary>
        [Fact]
        public void OneTripCutsAndSpreadsSeveralLanes()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(400.0, 0.0);
            Options.Roads.Add(Built.Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Int32 West = Int32.MaxValue;
            Int32 East = Int32.MinValue;
            Double South = Double.PositiveInfinity;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-6)
                    {
                        continue;
                    }

                    West = Math.Min(West, Bx);
                    East = Math.Max(East, Bx);
                    South = Math.Min(South, Built.Grid.CenterY[By, Bx]);
                }
            }

            Assert.True(East > West);
            ClearStocks(Built.Grid, Surface);
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-6)
                    {
                        continue;
                    }

                    Double Target = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    Built.Grid.Existing[By, Bx] = Target;
                    Double StationFt = (Built.Grid.CenterY[By, Bx] - South) / Constants.FEET_TO_METERS;
                    // East cut first, then the west lane, so the walk finishes the west lane
                    // and only then bends forward onto the east fill.
                    Boolean EastCut = (Bx == East) && (StationFt >= 0.0) && (StationFt < 70.0);
                    Boolean WestCut = (Bx == West) && (StationFt >= 110.0) && (StationFt < 180.0);
                    Boolean WestFill = (Bx == West) && (StationFt >= 220.0) && (StationFt < 300.0);
                    Boolean EastFill = (Bx == East) && (StationFt >= 340.0) && (StationFt < 420.0);
                    if (WestCut || EastCut)
                    {
                        Surface.StageCutM[By, Bx] = 0.08;
                    }

                    if (WestFill || EastFill)
                    {
                        Built.Grid.Existing[By, Bx] = Target - 0.04;
                    }
                }
            }

            List<Pass> Passes = RoadPadBuild.BuildPasses(Built.Grid, Surface, Options);
            Double Bowl = Options.ScraperCapacityLcy / Options.SwellFactor;
            foreach (Pass Trip in Passes)
            {
                Assert.True(Trip.LoadBcy + 1e-6 >= (Bowl * Options.RoadMinTripLoadPct));
            }

            Boolean Found = false;
            Double RadiusM = Options.MinimumTurningDiameterFt * Constants.FEET_TO_METERS * Constants.HALF_SCALE;
            foreach (TripGuidance.TripCourse Course in Surface.RoadCourses)
            {
                List<TripGuidance.CourseSegment> Cuts = ActionLanes(Course, TripGuidance.GuidanceAction.Cut);
                List<TripGuidance.CourseSegment> Fills = ActionLanes(Course, TripGuidance.GuidanceAction.Fill);
                if ((Cuts.Count < 2) || (Fills.Count < 2))
                {
                    continue;
                }

                Double Heading = LaneHeading(Cuts[0]);
                foreach (TripGuidance.CourseSegment Lane in Cuts)
                {
                    Assert.True(Math.Abs(DubinsConnector.WrapPi(LaneHeading(Lane) - Heading)) < (20.0 * Math.PI / 180.0));
                }

                foreach (TripGuidance.CourseSegment Lane in Fills)
                {
                    Assert.True(Math.Abs(DubinsConnector.WrapPi(LaneHeading(Lane) - Heading)) < (20.0 * Math.PI / 180.0));
                }

                AssertLaneJoins(Course, TripGuidance.GuidanceAction.Cut, RadiusM);
                AssertLaneJoins(Course, TripGuidance.GuidanceAction.Fill, RadiusM);
                Found = true;
            }

            Assert.True(Found);
        }

        /// <summary>
        /// A pocket under 15% of the bowl, and within 100 ft along the bank, is blade work.
        /// </summary>
        [Fact]
        public void LightRoadTripGoesToTheBlade()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(200.0, 0.0);
            Options.Roads.Add(Built.Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Int32 Column = -1;
            Double South = Double.PositiveInfinity;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-6)
                    {
                        continue;
                    }

                    if (Column < 0)
                    {
                        Column = Bx;
                    }

                    South = Math.Min(South, Built.Grid.CenterY[By, Bx]);
                }
            }

            ClearStocks(Built.Grid, Surface);
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-6)
                    {
                        continue;
                    }

                    Double Target = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    Built.Grid.Existing[By, Bx] = Target;
                    if (Bx != Column)
                    {
                        continue;
                    }

                    Double StationFt = (Built.Grid.CenterY[By, Bx] - South) / Constants.FEET_TO_METERS;
                    if ((StationFt >= 0.0) && (StationFt < 25.0))
                    {
                        Surface.StageCutM[By, Bx] = 0.04;
                    }

                    if ((StationFt >= 40.0) && (StationFt < 70.0))
                    {
                        Built.Grid.Existing[By, Bx] = Target - 0.02;
                    }
                }
            }

            List<Pass> Passes = RoadPadBuild.BuildPasses(Built.Grid, Surface, Options);
            Assert.Empty(Passes);
            Assert.True(Surface.BladeMoveBcy > 0.2);
            Assert.Contains("blade ", Surface.FormatStage(), StringComparison.Ordinal);
        }

        /// <summary>
        /// A second lift under 15% of the bowl joins the trip that already drives those cells.
        /// </summary>
        [Fact]
        public void LightLiftMergesIntoTheSameHeadingTrip()
        {
            HaulBuilderOptions Options = new HaulBuilderOptions();
            (Grid Grid, List<(Double X, Double Y)> Boundary, RoadCentreline Road) Built = RoadField(400.0, 0.0);
            Options.Roads.Add(Built.Road);
            RoadPadSurface Surface = RoadPadBuild.Prepare(Built.Grid, Options, Built.Boundary);
            Int32 Column = -1;
            Double South = Double.PositiveInfinity;
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-6)
                    {
                        continue;
                    }

                    if (Column < 0)
                    {
                        Column = Bx;
                    }

                    South = Math.Min(South, Built.Grid.CenterY[By, Bx]);
                }
            }

            ClearStocks(Built.Grid, Surface);
            for (Int32 By = 0; By < Built.Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Built.Grid.Width; Bx += 1)
                {
                    if (Surface.PanM[By, Bx] <= 1e-6)
                    {
                        continue;
                    }

                    Double Target = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    Built.Grid.Existing[By, Bx] = Target;
                    if (Bx != Column)
                    {
                        continue;
                    }

                    Double StationFt = (Built.Grid.CenterY[By, Bx] - South) / Constants.FEET_TO_METERS;
                    if ((StationFt >= 0.0) && (StationFt < 120.0))
                    {
                        Surface.StageCutM[By, Bx] = 0.20;
                    }

                    if ((StationFt >= 140.0) && (StationFt < 210.0))
                    {
                        Built.Grid.Existing[By, Bx] = Target - 0.081;
                    }
                }
            }

            List<Pass> Passes = RoadPadBuild.BuildPasses(Built.Grid, Surface, Options);
            Assert.Single(Passes);
            Assert.InRange(Passes[0].LoadBcy, 6.0, 8.5);
            Assert.True(Surface.BladeMoveBcy < 0.05);
        }

        /// <summary>
        /// Drops stage cut and reserved field cut so a test can supply its own.
        /// </summary>
        /// <param name="Grid">Grid size.</param>
        /// <param name="Surface">Stocks to clear.</param>
        private static void ClearStocks
            (
            Grid Grid,
            RoadPadSurface Surface
            )
        {
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Surface.StageCutM[By, Bx] = 0.0;
                    Surface.ReservedDepthM[By, Bx] = 0.0;
                }
            }
        }

        /// <summary>
        /// Blade-down lanes of one action, in drive order.
        /// </summary>
        /// <param name="Course">Guided trip.</param>
        /// <param name="Action">Cut or fill.</param>
        /// <returns>The lanes.</returns>
        private static List<TripGuidance.CourseSegment> ActionLanes
            (
            TripGuidance.TripCourse Course,
            TripGuidance.GuidanceAction Action
            )
        {
            List<TripGuidance.CourseSegment> Lanes = new List<TripGuidance.CourseSegment>();
            foreach (TripGuidance.CourseSegment Segment in Course.Segments)
            {
                if (Segment.IsLane && (Segment.Action == Action))
                {
                    Lanes.Add(Segment);
                }
            }

            return Lanes;
        }

        /// <summary>
        /// Consecutive lanes of one action are joined by a forward S-bend.
        /// </summary>
        /// <param name="Course">Guided trip.</param>
        /// <param name="Action">Cut or fill.</param>
        /// <param name="RadiusM">Turning radius.</param>
        private static void AssertLaneJoins
            (
            TripGuidance.TripCourse Course,
            TripGuidance.GuidanceAction Action,
            Double RadiusM
            )
        {
            TripGuidance.CourseSegment Previous = null;
            TripGuidance.CourseSegment Between = null;
            foreach (TripGuidance.CourseSegment Segment in Course.Segments)
            {
                if (Segment.IsLane && (Segment.Action == Action))
                {
                    if ((Previous != null) && (Between != null))
                    {
                        AssertForwardBend(Between, Previous, Segment, RadiusM);
                    }

                    Previous = Segment;
                    Between = null;
                }
                else if (!Segment.IsLane)
                {
                    Between = Segment;
                }
            }
        }

        /// <summary>
        /// A rectangular field. Cells whose centres fall outside the length are left invalid.
        /// </summary>
        /// <param name="WidthFt">Width in feet. Rounded up to whole cells.</param>
        /// <param name="LengthFt">Nominal length in feet.</param>
        /// <param name="CellFt">Cell size in feet.</param>
        /// <param name="Slope">Rise over run along northing.</param>
        /// <param name="BoundaryLengthFt">North boundary, in feet. Null uses the whole grid.</param>
        /// <returns>The grid and the boundary ring.</returns>
        private static (Grid Grid, List<(Double X, Double Y)> Boundary) Rectangle
            (
            Double WidthFt,
            Double LengthFt,
            Double CellFt,
            Double Slope = 0.0,
            Double? BoundaryLengthFt = null
            )
        {
            Double Cell = CellFt * Constants.FEET_TO_METERS;
            Int32 Width = Math.Max(1, (Int32)Math.Ceiling((WidthFt / CellFt) - 1e-9));
            Int32 Height = Math.Max(1, (Int32)Math.Ceiling((LengthFt / CellFt) - 1e-9));
            Double MinX = -(Width * Constants.HALF_SCALE) * Cell;
            Double MinY = 0.0;
            Double LimitY = (BoundaryLengthFt ?? (Height * CellFt)) * Constants.FEET_TO_METERS;
            Double[,] Existing = new Double[Height, Width];
            Double[,] Target = new Double[Height, Width];
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                Double CentreY = MinY + ((By + Constants.HALF_SCALE) * Cell);
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Boolean Inside = CentreY <= (LimitY + 1e-6);
                    if (!Inside)
                    {
                        Existing[By, Bx] = Double.NaN;
                        Target[By, Bx] = Double.NaN;
                        Valid[By, Bx] = false;
                        continue;
                    }

                    Double Elevation = 100.0 + (Slope * (CentreY - MinY));
                    Existing[By, Bx] = Elevation;
                    Target[By, Bx] = Elevation;
                    Valid[By, Bx] = true;
                }
            }

            List<(Double X, Double Y)> Boundary = new List<(Double X, Double Y)>
            {
                (MinX, MinY),
                (MinX + (Width * Cell), MinY),
                (MinX + (Width * Cell), LimitY),
                (MinX, LimitY),
            };
            Grid Grid = new Grid(
                Cell,
                Width,
                Height,
                MinX,
                MinY,
                1,
                true,
                0.0,
                0.0,
                Existing,
                Target,
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
            return (Grid, Boundary);
        }

        /// <summary>
        /// Copies elevations so a second prepare does not see the first road fit.
        /// </summary>
        /// <param name="Source">Grid to copy.</param>
        /// <returns>A grid with the same ground and the incoming design.</returns>
        private static Grid CloneGrid
            (
            Grid Source
            )
        {
            return new Grid(
                Source.GridSizeM,
                Source.Width,
                Source.Height,
                Source.MinX,
                Source.MinY,
                Source.Zone,
                Source.IsNorthern,
                Source.MeanLat,
                Source.MeanLon,
                (Double[,])Source.Existing.Clone(),
                (Double[,])Source.Target.Clone(),
                (Boolean[,])Source.Valid.Clone(),
                Source.SwellFactor,
                Source.Compaction,
                Source.MaxDepthMm,
                new List<List<(Double X, Double Y)>>());
        }

        /// <summary>
        /// Copies the road and pad defaults without the geometry.
        /// </summary>
        /// <param name="Source">Defaults to copy.</param>
        /// <returns>A new options object.</returns>
        private static HaulBuilderOptions CopyOptions
            (
            HaulBuilderOptions Source
            )
        {
            HaulBuilderOptions Copy = new HaulBuilderOptions();
            Copy.RoadCrossSlopePct = Source.RoadCrossSlopePct;
            Copy.RoadHeightFt = Source.RoadHeightFt;
            Copy.CutFillRatio = Source.CutFillRatio;
            return Copy;
        }

        /// <summary>
        /// A two-point centreline.
        /// </summary>
        /// <param name="X0">Start easting.</param>
        /// <param name="Y0">Start northing.</param>
        /// <param name="X1">End easting.</param>
        /// <param name="Y1">End northing.</param>
        /// <returns>The road.</returns>
        private static RoadCentreline Line
            (
            Double X0,
            Double Y0,
            Double X1,
            Double Y1
            )
        {
            return new RoadCentreline(new List<(Double X, Double Y)>
            {
                (X0, Y0),
                (X1, Y1),
            });
        }

        /// <summary>
        /// Supply minus demand when pan cells sit on the pan and the field sits on the field design.
        /// </summary>
        /// <param name="Grid">Existing ground and cell area.</param>
        /// <param name="Surface">Fitted field design and road fill.</param>
        private static void AssertBalanced
            (
            Grid Grid,
            RoadPadSurface Surface
            )
        {
            Double Cut = 0.0;
            Double Fill = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double Target = Surface.FieldDesignM[By, Bx] + Surface.PanM[By, Bx];
                    Double Delta = Grid.Existing[By, Bx] - Target;
                    if (Delta > 0.0)
                    {
                        Cut += Delta;
                    }
                    else
                    {
                        Fill += -Delta;
                    }
                }
            }

            Double Supply = Cut * Grid.AreaM2 * Constants.M3_TO_CY;
            Double Demand = Fill * Grid.AreaM2 * Constants.M3_TO_CY * Grid.Compaction;
            Double Gap = Math.Abs(Supply - Demand);
            Double Scale = Math.Max(1.0, Supply);
            Assert.InRange(Gap / Scale, 0.0, 0.001);
        }

        /// <summary>
        /// Distance from a point to a two-point centreline.
        /// </summary>
        /// <param name="Road">Centreline.</param>
        /// <param name="X">Easting.</param>
        /// <param name="Y">Northing.</param>
        /// <returns>Distance in metres.</returns>
        private static Double DistanceToRoad
            (
            RoadCentreline Road,
            Double X,
            Double Y
            )
        {
            (Double X, Double Y) A = Road.Vertices[0];
            (Double X, Double Y) B = Road.Vertices[1];
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            Double Length2 = (Dx * Dx) + (Dy * Dy);
            Double T = 0.0;
            if (Length2 > 0.0)
            {
                T = Math.Clamp((((X - A.X) * Dx) + ((Y - A.Y) * Dy)) / Length2, 0.0, 1.0);
            }

            Double Px = A.X + (T * Dx);
            Double Py = A.Y + (T * Dy);
            Double OffsetX = X - Px;
            Double OffsetY = Y - Py;
            return Math.Sqrt((OffsetX * OffsetX) + (OffsetY * OffsetY));
        }

        /// <summary>
        /// Intersection of a north-south road and an east-west road.
        /// </summary>
        /// <param name="South">North-south centreline.</param>
        /// <param name="East">East-west centreline.</param>
        /// <returns>The crossing point.</returns>
        private static (Double X, Double Y) CrossingPoint
            (
            RoadCentreline South,
            RoadCentreline East
            )
        {
            return (South.Vertices[0].X, East.Vertices[0].Y);
        }

        /// <summary>
        /// Bank volume of a straight road, including both end caps, in BCY.
        /// </summary>
        /// <param name="LengthFt">Centreline length in feet.</param>
        /// <param name="CrossPct">Crown cross slope in percent. Zero is a flat top at the shoulder height.</param>
        /// <returns>Bank cubic yards at a 1.2 fill ratio. Defaults are a 14 ft top, 1 ft shoulder, and 3:1 sides.</returns>
        private static Double StraightRoadBcy
            (
            Double LengthFt,
            Double CrossPct
            )
        {
            Double Height = 1.0;
            Double Half = 7.0;
            Double SideRun = 3.0;
            Double Crown = CrossPct / 100.0;
            Double Centre = Height + (Crown * Half);
            Double Toe = Half + SideRun;
            // The top is a trapezoid. The two side slopes are triangles that meet the shoulder.
            Double TopArea = (Centre + Height) * Half;
            Double SideArea = Height * SideRun;
            Double Prism = (TopArea + SideArea) * LengthFt;
            Int32 Steps = 8000;
            Double Dr = Toe / Steps;
            Double Ends = 0.0;
            for (Int32 Index = 0; Index < Steps; Index += 1)
            {
                Double Radius = (Index + Constants.HALF_SCALE) * Dr;
                Double Profile = Radius <= Half
                    ? Centre - (Crown * Radius)
                    : Height * (1.0 - ((Radius - Half) / SideRun));
                // The two wrapped ends together are one full revolution of the cross-section.
                Ends += 2.0 * Math.PI * Radius * Profile * Dr;
            }

            return ((Prism + Ends) / 27.0) * 1.2;
        }

        /// <summary>
        /// Bank volume of a pad: the shoulder frustum plus the slope-away cone, at a 1.2 fill ratio.
        /// </summary>
        /// <param name="TopRadiusFt">Shoulder radius in feet.</param>
        /// <param name="ToeRadiusFt">Toe radius in feet.</param>
        /// <param name="ShoulderFt">Shoulder height in feet.</param>
        /// <param name="Slope">Slope away, rise over run.</param>
        /// <returns>Bank cubic yards.</returns>
        private static Double SlopedPadBcy
            (
            Double TopRadiusFt,
            Double ToeRadiusFt,
            Double ShoulderFt,
            Double Slope
            )
        {
            Double R1 = TopRadiusFt;
            Double R2 = ToeRadiusFt;
            Double Rise = Slope * R1;
            // π/3·h·(r1² + r1·r2 + r2²) is the flat shoulder and the 3:1 edge. The cone is the rise to the well.
            Double Frustum = (Math.PI / 3.0) * ShoulderFt * ((R1 * R1) + (R1 * R2) + (R2 * R2));
            Double Cone = Math.PI * R1 * R1 * Rise / 3.0;
            return ((Frustum + Cone) / 27.0) * 1.2;
        }
    }
}
