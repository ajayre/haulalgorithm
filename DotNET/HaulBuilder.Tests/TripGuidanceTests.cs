using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Dubins connectors between the existing lanes, the grade pass over the fill,
    /// and the lane directions that keep those connectors in the field.
    /// </summary>
    public sealed class TripGuidanceTests
    {
        /// <summary>Full circle is this fraction of 2πR. Anything shorter is not a loop.</summary>
        private const Double FULL_CIRCLE = 0.98;

        /// <summary>An arc shorter than this is the zero arc a word is allowed to drop.</summary>
        private const Double ARC_DUST_M = 0.05;

        /// <summary>
        /// Collinear poses are one straight of the chord length.
        /// </summary>
        [Fact]
        public void Collinear_IsOneStraight()
        {
            DubinsConnector.Pose Start = new DubinsConnector.Pose(0.0, 0.0, 0.0);
            DubinsConnector.Pose End = new DubinsConnector.Pose(50.0, 0.0, 0.0);
            DubinsConnector.Path Path = DubinsConnector.Connect(Start, End, DubinsConnector.DefaultRadiusM, null);
            Assert.Single(Path.Pieces);
            Assert.Equal(DubinsConnector.PieceKind.Straight, Path.Pieces[0].Kind);
            Assert.InRange(Path.LengthM, 49.999, 50.001);
            AssertConnector(Path, DubinsConnector.DefaultRadiusM);
        }

        /// <summary>
        /// A right-angle join turns right only, through one straight, and does not loop.
        /// The 60 m by 40 m offset is far larger than R, so the shortest word splits the
        /// quarter turn across the two right arcs of RSR. A pose that does lie on one
        /// right circle is exactly one arc and one straight.
        /// </summary>
        [Fact]
        public void RightAngle_IsRightArcsAndAStraight()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            DubinsConnector.Pose Start = new DubinsConnector.Pose(0.0, 0.0, 0.0);
            DubinsConnector.Pose End = new DubinsConnector.Pose(60.0, -40.0, -Math.PI / 2.0);
            DubinsConnector.Path Path = DubinsConnector.Connect(Start, End, Radius, null);
            String Text = Describe(Path);
            Assert.Equal("RSR", Path.Word);
            Assert.Equal(3, Path.Pieces.Count);
            Assert.Equal(DubinsConnector.PieceKind.Right, Path.Pieces[0].Kind);
            Assert.Equal(DubinsConnector.PieceKind.Straight, Path.Pieces[1].Kind);
            Assert.Equal(DubinsConnector.PieceKind.Right, Path.Pieces[2].Kind);
            Double Arc = (Path.Pieces[0].LengthM + Path.Pieces[2].LengthM) / Radius;
            Assert.InRange(Arc, (Math.PI / 2.0) - 0.02, (Math.PI / 2.0) + 0.02);
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                if (Piece.Kind == DubinsConnector.PieceKind.Straight)
                {
                    continue;
                }

                Assert.True(Piece.LengthM < (FULL_CIRCLE * 2.0 * Math.PI * Radius), Text);
            }

            AssertConnector(Path, Radius);

            // This end pose is the end of a straight and one right quarter circle.
            DubinsConnector.Pose OneArcEnd = new DubinsConnector.Pose(30.0 + Radius, -Radius, -Math.PI / 2.0);
            DubinsConnector.Path OneArc = DubinsConnector.Connect(Start, OneArcEnd, Radius, null);
            Int32 Arcs = 0;
            Boolean Straight = false;
            foreach (DubinsConnector.Piece Piece in OneArc.Pieces)
            {
                if (Piece.Kind == DubinsConnector.PieceKind.Straight)
                {
                    Straight = true;
                    continue;
                }

                Assert.Equal(DubinsConnector.PieceKind.Right, Piece.Kind);
                if (Piece.LengthM > ARC_DUST_M)
                {
                    Arcs += 1;
                }

                Assert.True(Piece.LengthM < (FULL_CIRCLE * 2.0 * Math.PI * Radius));
            }

            Assert.True(Straight);
            Assert.Equal(1, Arcs);
            AssertConnector(OneArc, Radius);
        }

        /// <summary>
        /// A heading of almost 2π is the same direction as 0. The join is empty,
        /// not a circle of length 2πR.
        /// </summary>
        [Fact]
        public void SamePose_HeadingNearTwoPi_IsEmpty()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            DubinsConnector.Pose Start = new DubinsConnector.Pose(0.0, 0.0, 0.0);
            DubinsConnector.Pose End = new DubinsConnector.Pose(0.0, 0.0, (2.0 * Math.PI) - 1e-9);
            DubinsConnector.Path Path = DubinsConnector.Connect(Start, End, Radius, null);
            Assert.Empty(Path.Pieces);
            Assert.Equal(0.0, Path.LengthM, 9);
            Assert.True(Path.LengthM < (0.01 * Radius));
        }

        /// <summary>
        /// Turning around on the same point is the light-bulb path, 7πR/3 long.
        /// </summary>
        [Fact]
        public void TurnaroundInPlace_IsTheLightBulb()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            DubinsConnector.Pose Start = new DubinsConnector.Pose(0.0, 0.0, 0.0);
            DubinsConnector.Pose End = new DubinsConnector.Pose(0.0, 0.0, Math.PI);
            DubinsConnector.Path Path = DubinsConnector.Connect(Start, End, Radius, null);
            Double Expected = 7.0 * Math.PI / 3.0 * Radius;
            Assert.InRange(Path.LengthM, Expected * 0.99, Expected * 1.01);
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                Assert.True(Piece.CurvaturePerM <= (1.0 / Radius) + 1e-9);
            }

            AssertConnector(Path, Radius);
        }

        /// <summary>
        /// Every connector meets both poses, keeps heading through the joints,
        /// and never bends tighter than the radius.
        /// </summary>
        [Fact]
        public void Connectors_AreContinuous_And_WithinRadius()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            DubinsConnector.Pose[][] Pairs = new DubinsConnector.Pose[][]
            {
                new DubinsConnector.Pose[]
                {
                    new DubinsConnector.Pose(0.0, 0.0, 0.0),
                    new DubinsConnector.Pose(50.0, 0.0, 0.0),
                },
                new DubinsConnector.Pose[]
                {
                    new DubinsConnector.Pose(0.0, 0.0, 0.0),
                    new DubinsConnector.Pose(60.0, -40.0, -Math.PI / 2.0),
                },
                new DubinsConnector.Pose[]
                {
                    new DubinsConnector.Pose(0.0, 0.0, 0.0),
                    new DubinsConnector.Pose(0.0, 0.0, Math.PI),
                },
                new DubinsConnector.Pose[]
                {
                    new DubinsConnector.Pose(0.0, 0.0, 0.0),
                    new DubinsConnector.Pose(20.0, 10.0, 0.4),
                },
                new DubinsConnector.Pose[]
                {
                    new DubinsConnector.Pose(0.0, 0.0, 1.0),
                    new DubinsConnector.Pose(5.0, -4.0, -1.0),
                },
                new DubinsConnector.Pose[]
                {
                    new DubinsConnector.Pose(10.0, 10.0, 0.5),
                    new DubinsConnector.Pose(10.0, 10.0, 2.0),
                },
                new DubinsConnector.Pose[]
                {
                    new DubinsConnector.Pose(0.0, 0.0, 0.0),
                    new DubinsConnector.Pose(100.0, 0.0, Math.PI),
                },
            };
            foreach (DubinsConnector.Pose[] Pair in Pairs)
            {
                DubinsConnector.Path Path = DubinsConnector.Connect(Pair[0], Pair[1], Radius, null);
                AssertConnector(Path, Radius);
            }
        }

        /// <summary>
        /// A spread that ends two metres from the boundary still produces a turnaround.
        /// It stays in the field, or the connector is marked outside. The run finishes.
        /// </summary>
        [Fact]
        public void EdgeTurnaround_StaysInOrIsMarkedOutside()
        {
            const Int32 Width = 40;
            const Int32 Height = 30;
            Grid Field = MakeGrid(Width, Height, 1.0, new Double[Height, Width]);
            TripGuidance.WorkLane Lane = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (10.0, 15.0),
                    (38.0, 15.0),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Lane },
                null,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow);
            TripGuidance.CourseSegment? Turn = null;
            foreach (TripGuidance.CourseSegment Segment in Course.Segments)
            {
                if (Segment.Action == TripGuidance.GuidanceAction.Turn)
                {
                    Turn = Segment;
                }
            }

            Assert.NotNull(Turn);
            if (!Turn.Outside)
            {
                foreach (TripGuidance.DrawnPiece Piece in Turn.Pieces)
                {
                    foreach (DubinsConnector.Pose Sample in Piece.Samples)
                    {
                        Assert.True(Field.ContainsWorldXy(Sample.X, Sample.Y));
                    }
                }
            }
        }

        /// <summary>
        /// A spread that ends two metres from the boundary is driven back toward
        /// the interior, and the turnaround stays in the field.
        /// </summary>
        [Fact]
        public void SpreadEndingTwoMetresFromTheBoundary_DrivesInward()
        {
            const Int32 Width = 80;
            const Int32 Height = 50;
            const Double EndX = 78.0;
            const Double InteriorX = 34.0;
            const Double LaneY = 25.0;
            Grid Field = MakeGrid(Width, Height, 1.0, new Double[Height, Width]);
            TripGuidance.WorkLane Lane = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (InteriorX, LaneY),
                    (EndX, LaneY),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Lane },
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Flow);
            TripGuidance.CourseSegment? Spread = null;
            TripGuidance.CourseSegment? Turn = null;
            foreach (TripGuidance.CourseSegment Segment in Choice.Course.Segments)
            {
                if (Segment.IsLane && (Segment.Action == TripGuidance.GuidanceAction.Fill))
                {
                    Spread = Segment;
                }

                if (Segment.Action == TripGuidance.GuidanceAction.Turn)
                {
                    Turn = Segment;
                }
            }

            Assert.NotNull(Spread);
            Assert.NotNull(Turn);
            DubinsConnector.Pose Started = Spread.Pieces[0].Samples[0];
            TripGuidance.DrawnPiece SpreadEnd = Spread.Pieces[Spread.Pieces.Count - 1];
            DubinsConnector.Pose Finished = SpreadEnd.Samples[SpreadEnd.Samples.Count - 1];
            // Time alone. The reverse is used only when it is faster than the planned lane.
            TripGuidance.TripCourse PlannedCourse = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Lane },
                null,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow,
                null,
                null,
                2.5,
                false);
            TripGuidance.TripCourse ReversedCourse = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Lane.Reversed() },
                null,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow,
                null,
                null,
                2.5,
                false);
            Boolean ReverseWins = (ReversedCourse.ProductiveSeconds + 1e-6) < PlannedCourse.ProductiveSeconds;
            if (Choice.Fills[0].ArcLengthM <= 1.0)
            {
                if (ReverseWins)
                {
                    Assert.InRange(Started.X, EndX - (2.0 * DubinsConnector.DefaultRadiusM) - 0.5, EndX + 0.05);
                    Assert.True(Started.X > Finished.X);
                }
                else
                {
                    Assert.InRange(Started.X, InteriorX - 0.05, InteriorX + (2.0 * DubinsConnector.DefaultRadiusM) + 0.5);
                    Assert.True(Started.X < Finished.X);
                }
            }

            Assert.True(Choice.Course.ProductiveSeconds <= (PlannedCourse.ProductiveSeconds + 1e-4));
            Assert.True(Choice.Course.ProductiveSeconds <= (ReversedCourse.ProductiveSeconds + 1e-4));
            if (!Turn.Outside)
            {
                foreach (TripGuidance.DrawnPiece Piece in Turn.Pieces)
                {
                    foreach (DubinsConnector.Pose Sample in Piece.Samples)
                    {
                        Assert.True(
                            Field.ContainsWorldXy(Sample.X, Sample.Y),
                            Sample.X.ToString("F2", CultureInfo.InvariantCulture)
                            + ","
                            + Sample.Y.ToString("F2", CultureInfo.InvariantCulture));
                    }
                }
            }
        }

        /// <summary>
        /// A cut that drops toward the boundary stays pointed downhill, even when
        /// reversing it would shorten the haul.
        /// </summary>
        [Fact]
        public void DownhillCut_StaysDownhill_WhenTheReverseHaulIsShorter()
        {
            const Int32 Width = 120;
            const Int32 Height = 60;
            const Double HighX = 40.0;
            const Double LowX = 110.0;
            const Double CutY = 30.0;
            Double[,] Existing = new Double[Height, Width];
            Existing[(Int32)CutY, (Int32)HighX] = 2.0;
            Existing[(Int32)CutY, (Int32)LowX] = 0.0;
            Grid Field = MakeGrid(Width, Height, 1.0, Existing);
            TripGuidance.WorkLane Cut = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (HighX, CutY),
                    (LowX, CutY),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.WorkLane Spread = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (55.0, 18.0),
                    (35.0, 18.0),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { Spread },
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Flow);
            TripGuidance.CourseSegment? CutLane = null;
            foreach (TripGuidance.CourseSegment Segment in Choice.Course.Segments)
            {
                if (Segment.IsLane && (Segment.Action == TripGuidance.GuidanceAction.Cut))
                {
                    CutLane = Segment;
                }
            }

            Assert.NotNull(CutLane);
            DubinsConnector.Pose Started = CutLane.Pieces[0].Samples[0];
            TripGuidance.DrawnPiece CutEnd = CutLane.Pieces[CutLane.Pieces.Count - 1];
            DubinsConnector.Pose Finished = CutEnd.Samples[CutEnd.Samples.Count - 1];
            Assert.InRange(Started.X, HighX - 0.05, HighX + 0.05);
            Assert.InRange(Finished.X, LowX - 0.05, LowX + 0.05);
            // Reversing the cut shortens the haul. The downhill rule refuses that.
            Int32 BestDownOutside = Int32.MaxValue;
            Double BestDownTime = Double.PositiveInfinity;
            Int32 BestUpOutside = Int32.MaxValue;
            Double BestUpTime = Double.PositiveInfinity;
            TripGuidance.WorkLane[] Spreads = new TripGuidance.WorkLane[] { Spread, Spread.Reversed() };
            foreach (TripGuidance.WorkLane Fill in Spreads)
            {
                TripCourseScore Down = ScoreCourse(Cut, Fill, Field);
                TripCourseScore Up = ScoreCourse(Cut.Reversed(), Fill, Field);
                if ((Down.Outside < BestDownOutside)
                    || ((Down.Outside == BestDownOutside) && (Down.Seconds < BestDownTime)))
                {
                    BestDownOutside = Down.Outside;
                    BestDownTime = Down.Seconds;
                }

                if ((Up.Outside < BestUpOutside)
                    || ((Up.Outside == BestUpOutside) && (Up.Seconds < BestUpTime)))
                {
                    BestUpOutside = Up.Outside;
                    BestUpTime = Up.Seconds;
                }
            }

            Boolean UphillIsBetter = (BestUpOutside < BestDownOutside)
                || ((BestUpOutside == BestDownOutside) && (BestUpTime < BestDownTime));
            Assert.True(UphillIsBetter);
        }

        /// <summary>
        /// A spread running west into the west boundary. The faster course wins.
        /// When that course bends, the quarter circle stays on the edge.
        /// </summary>
        [Fact]
        public void SpreadWestIntoTheBoundary_BendsAlongTheEdge()
        {
            const Double BladeM = 2.5;
            const Double LaneY = 50.0;
            Grid Field = MakeGrid(80, 100, 1.0, new Double[100, 80]);
            TripGuidance.WorkLane Spread = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (77.0, LaneY),
                    (3.0, LaneY),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.DirectionChoice Choice = ChooseBoundaryLane(Field, Spread, BladeM);
            TripGuidance.WorkLane Chosen = Choice.Fills[0];
            if (Chosen.ArcLengthM > 1.0)
            {
                Double Quarter = DubinsConnector.DefaultRadiusM * Math.PI * 0.5;
                Assert.InRange(Chosen.ArcLengthM, Quarter - 0.05, Quarter + 0.05);
                Assert.InRange(Math.Abs(DubinsConnector.WrapPi(Chosen.Start.HeadingRad - Math.PI)), 0.0, 0.2);
                Assert.InRange(Chosen.Start.X, 70.0, 78.0);
                Assert.InRange(Chosen.End.X, (BladeM * 0.5) - 0.35, (BladeM * 0.5) + 0.35);
                // North or south: both quarter circles fit on this lane.
                Assert.InRange(Math.Abs(Math.Abs(DubinsConnector.WrapPi(Chosen.End.HeadingRad)) - (Math.PI * 0.5)), 0.0, 0.2);
                AssertCourseStaysIn(Choice.Course, Field);
            }

            TripGuidance.TripCourse Planned = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Spread },
                null,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow,
                null,
                null,
                BladeM,
                false);
            Assert.True(Choice.Course.ProductiveSeconds <= (Planned.ProductiveSeconds + 1e-4));
        }

        /// <summary>
        /// A lane that ends in a corner. The faster course wins. When it bends,
        /// the arc heads into the open side and the turn into the boundary is dropped.
        /// </summary>
        [Fact]
        public void LaneEndingInACorner_BendsTowardTheOpenSide()
        {
            const Double BladeM = 2.5;
            const Double LaneY = 4.0;
            Grid Field = MakeGrid(80, 60, 1.0, new Double[60, 80]);
            TripGuidance.WorkLane Spread = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (77.0, LaneY),
                    (3.0, LaneY),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.DirectionChoice Choice = ChooseBoundaryLane(Field, Spread, BladeM);
            TripGuidance.WorkLane Chosen = Choice.Fills[0];
            if (Chosen.ArcLengthM > 1.0)
            {
                Assert.InRange(Chosen.End.X, (BladeM * 0.5) - 0.35, (BladeM * 0.5) + 0.35);
                // South leaves the field. The surviving arc heads north.
                Assert.InRange(DubinsConnector.WrapPi(Chosen.End.HeadingRad - (Math.PI * 0.5)), -0.2, 0.2);
                Assert.True(Chosen.End.Y > LaneY + 4.0);
                AssertCourseStaysIn(Choice.Course, Field);
            }

            TripGuidance.TripCourse Planned = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Spread },
                null,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow,
                null,
                null,
                BladeM,
                false);
            Assert.True(Choice.Course.ProductiveSeconds <= (Planned.ProductiveSeconds + 1e-4));
        }

        /// <summary>
        /// The bent band and the cells it leaves behind are the straight lane's
        /// cells, with nothing shared and nothing dropped.
        /// </summary>
        [Fact]
        public void BentLane_PartitionsTheStraightCells()
        {
            const Double BladeM = 2.5;
            const Double LaneY = 50.0;
            Grid Field = MakeGrid(80, 100, 1.0, new Double[100, 80]);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((40, 50), 0.10),
                    ((1, 50), 0.10),
                };
            TripGuidance.WorkLane Spread = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (77.0, LaneY),
                    (3.0, LaneY),
                },
                Cells,
                0.0);
            TripGuidance.DirectionChoice Choice = ChooseBoundaryLane(Field, Spread, BladeM);
            TripGuidance.WorkLane Chosen = Choice.Fills[0];
            HashSet<(Int32 Bx, Int32 By)> Swept = new HashSet<(Int32 Bx, Int32 By)>();
            HashSet<(Int32 Bx, Int32 By)> Left = new HashSet<(Int32 Bx, Int32 By)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Chosen.Cells)
            {
                Assert.True(Swept.Add(Entry.Cell));
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Chosen.Deferred)
            {
                Assert.True(Left.Add(Entry.Cell));
                Assert.DoesNotContain(Entry.Cell, Swept);
            }

            Assert.Contains((40, 50), Swept);
            if (Chosen.ArcLengthM > 1.0)
            {
                Assert.Contains((1, 50), Left);
            }

            Assert.Equal(Cells.Count, Swept.Count + Left.Count);
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                Assert.True(Swept.Contains(Entry.Cell) || Left.Contains(Entry.Cell));
            }
        }

        /// <summary>
        /// Drives one spread whose ends both sit near a boundary.
        /// </summary>
        /// <param name="Field">Mask the lane is bent against.</param>
        /// <param name="Spread">Straight spread lane.</param>
        /// <param name="BladeM">Blade width.</param>
        /// <returns>The chosen direction, including any bend.</returns>
        private static TripGuidance.DirectionChoice ChooseBoundaryLane
            (
            Grid Field,
            TripGuidance.WorkLane Spread,
            Double BladeM
            )
        {
            return TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Spread },
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                BladeM,
                1.0,
                1,
                HaulStage.Flow);
        }

        /// <summary>
        /// The turnaround and the grade pass, and every other sample, stay in the mask.
        /// </summary>
        /// <param name="Course">Chosen course.</param>
        /// <param name="Field">Field mask.</param>
        private static void AssertCourseStaysIn
            (
            TripGuidance.TripCourse Course,
            Grid Field
            )
        {
            Boolean SawTurn = false;
            Boolean SawGrade = false;
            foreach (TripGuidance.CourseSegment Segment in Course.Segments)
            {
                Assert.False(Segment.Outside);
                if (Segment.Action == TripGuidance.GuidanceAction.Turn)
                {
                    SawTurn = true;
                }

                if (Segment.Action == TripGuidance.GuidanceAction.Grade)
                {
                    SawGrade = true;
                }

                foreach (TripGuidance.DrawnPiece Piece in Segment.Pieces)
                {
                    foreach (DubinsConnector.Pose Sample in Piece.Samples)
                    {
                        Assert.True(Field.ContainsWorldXy(Sample.X, Sample.Y));
                    }
                }
            }

            Assert.True(SawTurn);
            Assert.True(SawGrade);
        }

        /// <summary>
        /// After a 40 m spread the machine turns around and grades the same line backwards.
        /// </summary>
        [Fact]
        public void GradePass_ReversesTheSpread()
        {
            TripGuidance.WorkLane Lane = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (0.0, 0.0),
                    (40.0, 0.0),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Lane },
                null,
                null,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.0,
                1,
                HaulStage.Flow);
            TripGuidance.CourseSegment? Grade = null;
            Boolean SeenFill = false;
            Boolean SeenTurn = false;
            foreach (TripGuidance.CourseSegment Segment in Course.Segments)
            {
                if ((Segment.Action == TripGuidance.GuidanceAction.Fill) && Segment.IsLane)
                {
                    SeenFill = true;
                }

                if (Segment.Action == TripGuidance.GuidanceAction.Turn)
                {
                    Assert.True(SeenFill);
                    SeenTurn = true;
                }

                if ((Segment.Action == TripGuidance.GuidanceAction.Grade) && Segment.IsLane)
                {
                    Assert.True(SeenTurn);
                    Grade = Segment;
                }
            }

            Assert.NotNull(Grade);
            Assert.InRange(Grade.LengthM, 39.999, 40.001);
            DubinsConnector.Pose First = Grade.Pieces[0].Samples[0];
            DubinsConnector.Pose Last = Grade.Pieces[0].Samples[Grade.Pieces[0].Samples.Count - 1];
            Assert.InRange(First.X, 39.95, 40.05);
            Assert.InRange(Last.X, -0.05, 0.05);
            Assert.InRange(Math.Abs(DubinsConnector.WrapPi(First.HeadingRad)), Math.PI - 0.02, Math.PI + 0.02);
            AssertHeadingSteps(Course.Points, DubinsConnector.DefaultRadiusM);
        }

        /// <summary>
        /// Two fill cells left 5 mm high are cut to target. The soil stays accounted for.
        /// </summary>
        [Fact]
        public void GradePass_CutsFiveMillimetres_AndConservesVolume()
        {
            const Double HighM = 0.005;
            // Longer than half a radius, so the near-empty rule still drives the grade.
            const Int32 Cells = 10;
            Double[,] Existing = new Double[1, Cells];
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Column = 0; Column < Cells; Column += 1)
            {
                Existing[0, Column] = HighM;
                Fills.Add(((Column, 0), 0.0));
            }

            Grid Field = MakeGrid(Cells, 1, 1.0, Existing);
            Pass Trip = new Pass(
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                Fills,
                0.0,
                0.0,
                (0.0, 0.0),
                (0.0, 0.0),
                Field.CellCenterXy(0, 0),
                Field.CellCenterXy(1, 0));
            Trip.RibbonOrder = true;
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Field,
                new List<Pass> { Trip },
                Clock(),
                50.0,
                1.0,
                DubinsConnector.DefaultRadiusM,
                -1);
            for (Int32 Column = 0; Column < Cells; Column += 1)
            {
                Assert.InRange(Job.Surface[0, Column], -1e-9, 1e-9);
            }
            TripGuidance.GradeLedger Ledger = Job.Trips[0].Grade;
            Assert.InRange(Math.Abs(Ledger.CutM3 - (Ledger.PlacedM3 + Ledger.BowlM3)), 0.0, 1e-9);
            Assert.True(Ledger.CutM3 > 0.0);
        }

        /// <summary>
        /// Lines 2.6 m apart cannot be joined by a straight sideways jump when the
        /// radius is larger than half that gap.
        /// </summary>
        [Fact]
        public void CloseLines_UseADubinsPath()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            Assert.True(Radius > 1.3);
            DubinsConnector.Pose EndOfFirst = new DubinsConnector.Pose(40.0, 0.0, 0.0);
            DubinsConnector.Pose StartOfSecond = new DubinsConnector.Pose(40.0, 2.6, Math.PI);
            DubinsConnector.Path Path = DubinsConnector.Connect(EndOfFirst, StartOfSecond, Radius, null);
            Int32 Arcs = 0;
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                if (Piece.Kind != DubinsConnector.PieceKind.Straight)
                {
                    Arcs += 1;
                }
            }

            Assert.True(Arcs >= 1);
            Assert.False((Path.Pieces.Count == 1) && (Path.Pieces[0].Kind == DubinsConnector.PieceKind.Straight));
            AssertConnector(Path, Radius);
        }

        /// <summary>
        /// The loaded connector from the cut end to the spread start is part of
        /// the direction score. The shorter haul wins when both stay in the field.
        /// </summary>
        [Fact]
        public void CutToSpreadConnector_IsInTheDirectionScore()
        {
            const Int32 Width = 180;
            const Int32 Height = 120;
            const Double CutY = 50.0;
            Double[,] Existing = new Double[Height, Width];
            Existing[(Int32)CutY, 30] = 4.0;
            Existing[(Int32)CutY, 80] = 0.0;
            Grid Field = MakeGrid(Width, Height, 1.0, Existing);
            TripGuidance.WorkLane Cut = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (30.0, CutY),
                    (80.0, CutY),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.WorkLane Planned = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (150.0, 70.0),
                    (80.0, 70.0),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                Math.PI);
            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { Planned },
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Flow);
            // Reversed, then the short joint may slide the start forward by up to 2R.
            Assert.True(Choice.Fills[0].Start.X < Choice.Fills[0].End.X);
            Assert.InRange(Choice.Fills[0].Start.X, 79.5, 80.5 + (2.0 * DubinsConnector.DefaultRadiusM));
            Assert.True(Choice.Course.LoadedSeconds > 1.0);
            TripGuidance.TripCourse PlannedCourse = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { Planned },
                null,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow);
            Assert.True(Choice.Course.ProductiveSeconds < PlannedCourse.ProductiveSeconds);
        }

        /// <summary>
        /// Correcting fragments on one line are sorted, driven one way, and joined
        /// by a straight blade-up gap. The trip turns around once and does not grade.
        /// </summary>
        [Fact]
        public void CorrectingFragments_DriveStraightAlongTheLine()
        {
            Grid Field = MakeGrid(100, 80, 1.0, new Double[80, 100]);
            // Cells span the fragment. A single token cell would clip the line down to a point.
            TripGuidance.WorkLane East = CellsAlong((20, 40), (35, 40), 0.02);
            TripGuidance.WorkLane WestFacing = CellsAlong((55, 40), (40, 40), 0.02);
            TripGuidance.WorkLane SpreadFar = CellsAlong((30, 60), (45, 60), 0.02);
            TripGuidance.WorkLane SpreadNear = CellsAlong((55, 60), (70, 60), 0.02);
            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane> { WestFacing, East },
                new List<TripGuidance.WorkLane> { SpreadNear, SpreadFar },
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Correcting);
            List<TripGuidance.CourseSegment> Cuts = LanesOf(Choice.Course, TripGuidance.GuidanceAction.Cut);
            Assert.Equal(2, Cuts.Count);
            DubinsConnector.Pose FirstStart = Cuts[0].Pieces[0].Samples[0];
            DubinsConnector.Pose FirstEnd = LastSample(Cuts[0]);
            DubinsConnector.Pose SecondStart = Cuts[1].Pieces[0].Samples[0];
            DubinsConnector.Pose SecondEnd = LastSample(Cuts[1]);
            // Both fragments travel the same way, from one end of the line to the other.
            Boolean Eastbound = FirstStart.X < FirstEnd.X;
            Assert.Equal(Eastbound, SecondStart.X < SecondEnd.X);
            Double Low = Math.Min(FirstStart.X, SecondEnd.X);
            Double High = Math.Max(FirstStart.X, SecondEnd.X);
            Double Slide = 2.0 * DubinsConnector.DefaultRadiusM;
            // A short joint may trim either end by up to 2R. The trimmed cells stay deferred.
            Assert.InRange(Low, 19.5, 20.5 + Slide);
            Assert.InRange(High, 54.5 - Slide, 55.5);
            Assert.True(Math.Abs(FirstEnd.X - SecondStart.X) < Math.Abs(FirstStart.X - SecondEnd.X));
            TripGuidance.CourseSegment Gap = SegmentAfter(Choice.Course, Cuts[0]);
            Assert.False(Gap.IsLane);
            Assert.Equal(TripGuidance.GuidanceAction.Carry, Gap.Action);
            Assert.Equal("S", Gap.Word);
            Assert.Single(Gap.Pieces);
            Assert.Equal(DubinsConnector.PieceKind.Straight, Gap.Pieces[0].Kind);
            Assert.Equal(1, CountAction(Choice.Course, TripGuidance.GuidanceAction.Turn));
            Assert.Equal(0, CountLanes(Choice.Course, TripGuidance.GuidanceAction.Grade));
        }

        /// <summary>
        /// A correcting trip does not grade. The land plane follows. One turnaround remains.
        /// </summary>
        [Fact]
        public void CorrectingShallowSpread_SkipsTheGradePass()
        {
            Grid Field = MakeGrid(80, 40, 1.0, new Double[40, 80]);
            TripGuidance.WorkLane Spread = Lane((20.0, 20.0), (40.0, 20.0), 0.005);
            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Spread },
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Correcting);
            Assert.Equal(0, CountLanes(Choice.Course, TripGuidance.GuidanceAction.Grade));
            Assert.Equal(1, CountAction(Choice.Course, TripGuidance.GuidanceAction.Turn));
        }

        /// <summary>
        /// Two correcting lines meet with one Dubins end turn, not a straight jump.
        /// </summary>
        [Fact]
        public void CorrectingTwoLines_TurnOnlyAtTheLineEnd()
        {
            Grid Field = MakeGrid(100, 80, 1.0, new Double[80, 100]);
            TripGuidance.WorkLane Low = Lane((20.0, 30.0), (50.0, 30.0), 0.02);
            TripGuidance.WorkLane High = Lane((20.0, 50.0), (50.0, 50.0), 0.02);
            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane> { Low, High },
                new List<TripGuidance.WorkLane>(),
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Correcting);
            List<TripGuidance.CourseSegment> Cuts = LanesOf(Choice.Course, TripGuidance.GuidanceAction.Cut);
            Assert.Equal(2, Cuts.Count);
            TripGuidance.CourseSegment Join = SegmentAfter(Choice.Course, Cuts[0]);
            Assert.False(Join.IsLane);
            Int32 Arcs = 0;
            foreach (TripGuidance.DrawnPiece Piece in Join.Pieces)
            {
                if (Piece.Kind != DubinsConnector.PieceKind.Straight)
                {
                    Arcs += 1;
                }
            }

            Assert.True(Arcs >= 1);
            Assert.Equal(0, CountAction(Choice.Course, TripGuidance.GuidanceAction.Turn));
        }

        /// <summary>
        /// Greedy nearest-cut, then consecutive swaps, shortens the empty returns
        /// without changing which cells a trip carries.
        /// </summary>
        [Fact]
        public void TripOrder_ShortensEmptyReturns()
        {
            const Int32 Width = 45;
            const Int32 Height = 8;
            Grid Field = MakeGrid(Width, Height, 1.0, new Double[Height, Width]);
            List<Pass> Passes = new List<Pass>
            {
                MakePass((2, 2), (3, 2), (10, 2), (11, 2)),
                MakePass((35, 2), (36, 2), (30, 2), (31, 2)),
                MakePass((14, 2), (15, 2), (20, 2), (21, 2)),
            };
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Field,
                Passes,
                Clock(),
                50.0,
                2.5,
                DubinsConnector.DefaultRadiusM,
                -1);
            Assert.Equal(3, Job.DriveOrder.Count);
            Assert.Equal(0, Job.DriveOrder[0]);
            Assert.Equal(2, Job.DriveOrder[1]);
            Assert.Equal(1, Job.DriveOrder[2]);
            Assert.True(Job.EmptyReturnHoursAfter < Job.EmptyReturnHoursBefore);
            String Report = Job.FormatReport(0.0);
            Assert.Contains("Empty return before reorder", Report);
            Assert.Contains("Empty return after reorder", Report);
            Assert.Contains("Worst trips by connector length / productive length", Report);
            Assert.Contains("Worst trips by loaded-connector heading change", Report);
            Assert.Contains("Worst trips by turnaround count", Report);
            Assert.Contains("Reversals (connector turn over 150 deg)", Report);
            Assert.Contains("Outside connectors by type", Report);
            Assert.Contains("Correcting trips merged", Report);
            Assert.Contains("Correcting trips dropped", Report);
            Assert.Contains("this build", Report);
        }

        /// <summary>
        /// Two trips that share a cell keep the planner's order even when the
        /// empty return would rather drive the later one first.
        /// </summary>
        [Fact]
        public void OverlappingTrips_KeepPlannerOrder()
        {
            Grid Field = MakeGrid(50, 8, 1.0, new Double[8, 50]);
            Pass First = MakePass((2, 2), (3, 2), (4, 2), (5, 2));
            Pass Middle = MakePass((35, 2), (36, 2), (30, 2), (31, 2));
            Pass Near = MakePass((14, 2), (15, 2), (30, 2), (16, 2));
            List<Pass> Passes = new List<Pass> { First, Middle, Near };
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Field,
                Passes,
                Clock(),
                50.0,
                2.5,
                DubinsConnector.DefaultRadiusM,
                -1);
            Int32 MiddleAt = Job.DriveOrder.IndexOf(1);
            Int32 NearAt = Job.DriveOrder.IndexOf(2);
            Assert.True(MiddleAt >= 0);
            Assert.True(NearAt > MiddleAt);
        }

        /// <summary>
        /// A short gap with a small heading change and a small offset is a straight.
        /// </summary>
        [Fact]
        public void ShortJoint_SteersStraightAcross()
        {
            TripGuidance.WorkLane Cut = Lane((10.0, 10.0), (30.0, 10.0), 0.05);
            TripGuidance.WorkLane Fill = Lane((32.0, 10.4), (50.0, 10.4), 0.05);
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { Fill },
                null,
                null,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow,
                null,
                null,
                2.5,
                true);
            List<TripGuidance.CourseSegment> Cuts = LanesOf(Course, TripGuidance.GuidanceAction.Cut);
            TripGuidance.CourseSegment Join = SegmentAfter(Course, Cuts[0]);
            Assert.Equal("S", Join.Word);
            Assert.Single(Join.Pieces);
            Assert.Equal(DubinsConnector.PieceKind.Straight, Join.Pieces[0].Kind);
        }

        /// <summary>
        /// A short gap with a large heading change slides the next lane forward
        /// and keeps that shorter connector. The skipped start cell stays deferred.
        /// </summary>
        [Fact]
        public void ShortJoint_SlidesTheNextLane()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            TripGuidance.WorkLane Cut = Lane((0.0, 10.0), (20.0, 10.0), 0.05);
            List<(Double X, Double Y)> North = new List<(Double X, Double Y)>
            {
                (22.0, 12.0),
                (22.0, 50.0),
            };
            TripGuidance.WorkLane Fill = TripGuidance.WorkLane.FromCenterline(
                North,
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((22, 12), 0.05),
                    ((22, 20), 0.05),
                    ((22, 30), 0.05),
                    ((22, 40), 0.05),
                },
                Math.PI / 2.0);
            Double Untrimmed = DubinsConnector.ShortestLengthM(Cut.End, Fill.Start, Radius);
            List<TripGuidance.WorkLane> Cuts = new List<TripGuidance.WorkLane> { Cut };
            List<TripGuidance.WorkLane> Fills = new List<TripGuidance.WorkLane> { Fill };
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                Cuts,
                Fills,
                null,
                null,
                Clock(),
                Radius,
                1.0,
                1,
                HaulStage.Flow,
                null,
                null,
                2.5,
                true);
            List<TripGuidance.CourseSegment> CutLanes = LanesOf(Course, TripGuidance.GuidanceAction.Cut);
            TripGuidance.CourseSegment Join = SegmentAfter(Course, CutLanes[0]);
            Assert.True(Join.LengthM < (Untrimmed - 1.0));
            Assert.True((Fills[0].Start.Y > 13.0) || (Cuts[0].End.X < 19.0));
            Assert.True(Fills[0].Deferred.Count > 0);
        }

        /// <summary>
        /// A correcting trip drives at most three spread lines, snakes them, and does not grade.
        /// </summary>
        [Fact]
        public void CorrectingLines_StopAtThreeAndSnake()
        {
            Grid Field = MakeGrid(80, 50, 1.0, new Double[50, 80]);
            List<TripGuidance.WorkLane> Fills = new List<TripGuidance.WorkLane>();
            for (Int32 Row = 0; Row < 4; Row += 1)
            {
                Double Y = 10.0 + (Row * 4.0);
                Fills.Add(Lane((10.0, Y), (40.0, Y), 0.05));
            }

            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane> { Lane((10.0, 40.0), (40.0, 40.0), 0.05) },
                Fills,
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Correcting);
            Assert.Equal(3, Choice.Fills.Count);
            Assert.True(Choice.Fills[0].Start.X < Choice.Fills[0].End.X);
            Assert.True(Choice.Fills[1].Start.X > Choice.Fills[1].End.X);
            Assert.True(Choice.Fills[2].Start.X < Choice.Fills[2].End.X);
            Assert.True(Choice.HeldFills.Count >= 1);
            Assert.Equal(0, CountLanes(Choice.Course, TripGuidance.GuidanceAction.Grade));
            Assert.Equal(1, CountAction(Choice.Course, TripGuidance.GuidanceAction.Turn));
        }

        /// <summary>
        /// A short correcting trip merges into the adjacent line when the bowl has room.
        /// </summary>
        [Fact]
        public void Leftover_MergesIntoTheAdjacentLine()
        {
            Grid Field = MakeGrid(40, 40, 1.0, new Double[40, 40]);
            Pass Left = MakePass((10, 10), (12, 10), (10, 20), (12, 20));
            Pass Right = MakePass((10, 12), (12, 12), (10, 22), (12, 22));
            Left.Stage = HaulStage.Correcting;
            Right.Stage = HaulStage.Correcting;
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Field,
                new List<Pass> { Left, Right },
                Clock(),
                50.0,
                2.5,
                DubinsConnector.DefaultRadiusM,
                -1);
            Assert.Equal(1, Job.MergedCount);
            Assert.Equal(0, Job.DroppedCount);
            Assert.Single(Job.Trips);
        }

        /// <summary>
        /// A short correcting trip with no neighbour is dropped when its cells are within tolerance.
        /// </summary>
        [Fact]
        public void LeftoverWithinTolerance_IsDropped()
        {
            Grid Field = MakeGrid(40, 40, 1.0, new Double[40, 40]);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((10, 10), 0.005),
                    ((12, 10), 0.005),
                };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((10, 20), 0.005),
                    ((12, 20), 0.005),
                };
            Pass Trip = new Pass(Cuts, Fills, 0.01, 0.0, (10, 10), (12, 10), (10, 20), (12, 20));
            Trip.Stage = HaulStage.Correcting;
            Trip.RibbonOrder = true;
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Field,
                new List<Pass> { Trip },
                Clock(),
                50.0,
                2.5,
                DubinsConnector.DefaultRadiusM,
                -1);
            Assert.Equal(0, Job.MergedCount);
            Assert.Equal(1, Job.DroppedCount);
            Assert.Empty(Job.Trips);
        }

        /// <summary>
        /// A correcting line stops on its first and last fragment. The centreline
        /// that ran out to the field edge is not driven, and the line does not bend.
        /// The connector leaves from that trimmed end.
        /// </summary>
        [Fact]
        public void CorrectingLine_EndsAtTheLastFragment()
        {
            Grid Field = MakeGrid(80, 80, 1.0, new Double[80, 80]);
            TripGuidance.WorkLane Low = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)> { (0.5, 10.5), (70.5, 10.5) },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((20, 10), 0.05),
                    ((40, 10), 0.05),
                },
                0.0);
            TripGuidance.WorkLane High = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)> { (0.5, 50.5), (70.5, 50.5) },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((20, 50), 0.05),
                    ((40, 50), 0.05),
                },
                0.0);
            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane> { Low, High },
                new List<TripGuidance.WorkLane>(),
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Correcting);
            Assert.Equal(2, Choice.Cuts.Count);
            foreach (TripGuidance.WorkLane Chosen in Choice.Cuts)
            {
                Assert.Equal(0.0, Chosen.ArcLengthM);
                Assert.InRange(Chosen.Start.X, 19.0, 42.0);
                Assert.InRange(Chosen.End.X, 19.0, 42.0);
            }

            List<TripGuidance.CourseSegment> Cuts = LanesOf(Choice.Course, TripGuidance.GuidanceAction.Cut);
            Assert.Equal(2, Cuts.Count);
            TripGuidance.CourseSegment Join = SegmentAfter(Choice.Course, Cuts[0]);
            DubinsConnector.Pose LaneEnd = LastSample(Cuts[0]);
            DubinsConnector.Pose JoinStart = Join.Pieces[0].Samples[0];
            Double Dx = JoinStart.X - LaneEnd.X;
            Double Dy = JoinStart.Y - LaneEnd.Y;
            Assert.True(((Dx * Dx) + (Dy * Dy)) < 0.01);
        }

        /// <summary>
        /// A cut heading west along the field edge and a spread heading east on
        /// that same edge meet with a connector that stays in the field. The
        /// turn away from the field is the shorter word. The opposite turn is
        /// close enough that it wins.
        /// </summary>
        [Fact]
        public void EdgeCutWest_SpreadEast_ConnectorStaysInside()
        {
            const Int32 Width = 80;
            const Int32 Height = 40;
            const Double EdgeY = 0.5;
            Grid Field = MakeGrid(Width, Height, 1.0, new Double[Height, Width]);
            TripGuidance.WorkLane Cut = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (50.0, EdgeY),
                    (20.0, EdgeY),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                Math.PI);
            TripGuidance.WorkLane Spread = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)>
                {
                    (40.0, EdgeY),
                    (70.0, EdgeY),
                },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                0.0);
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { Spread },
                null,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow,
                null,
                null,
                2.5,
                true);
            List<TripGuidance.CourseSegment> Cuts = LanesOf(Course, TripGuidance.GuidanceAction.Cut);
            TripGuidance.CourseSegment Join = SegmentAfter(Course, Cuts[0]);
            Assert.False(Join.IsLane);
            Assert.False(Join.Outside, Join.Word + " " + Join.LengthM.ToString("0.0", CultureInfo.InvariantCulture));
            foreach (TripGuidance.DrawnPiece Piece in Join.Pieces)
            {
                foreach (DubinsConnector.Pose Sample in Piece.Samples)
                {
                    Assert.True(
                        Field.ContainsWorldXy(Sample.X, Sample.Y),
                        Join.Word + " " + Sample.X.ToString("0.0", CultureInfo.InvariantCulture)
                            + "," + Sample.Y.ToString("0.0", CultureInfo.InvariantCulture));
                }
            }
        }

        /// <summary>
        /// A line change takes the open path when it is faster. The straight join
        /// crosses a post. The in-field path goes around it.
        /// </summary>
        [Fact]
        public void LineChange_TakesTheFasterOutsidePath()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            Func<Double, Double, Boolean> Inside = delegate (Double X, Double Y)
            {
                if ((X < 0.0) || (X > 100.0) || (Y < 0.0) || (Y > 100.0))
                {
                    return false;
                }

                // A post on the straight join. A loop clears it and stays in the field.
                if ((X > 29.0) && (X < 31.0) && (Y > 9.7) && (Y < 10.3))
                {
                    return false;
                }

                return true;
            };
            TripGuidance.WorkLane Cut = Lane((10.0, 10.0), (22.0, 10.0), 0.05);
            TripGuidance.WorkLane Fill = Lane((40.0, 10.0), (55.0, 10.0), 0.05);
            DubinsConnector.Path Open = DubinsConnector.Connect(Cut.End, Fill.Start, Radius, null);
            DubinsConnector.Path Closed = DubinsConnector.Connect(Cut.End, Fill.Start, Radius, Inside);
            Assert.True(
                Open.LengthM + 1.0 < Closed.LengthM,
                String.Format(
                    System.Globalization.CultureInfo.InvariantCulture,
                    "open {0:F2} {1} inside {2:F2} {3}",
                    Open.LengthM,
                    Open.Word,
                    Closed.LengthM,
                    Closed.Word));
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { Fill },
                null,
                Inside,
                Clock(),
                Radius,
                1.0,
                1,
                HaulStage.Correcting,
                null,
                null,
                0.0,
                false);
            List<TripGuidance.CourseSegment> Cuts = LanesOf(Course, TripGuidance.GuidanceAction.Cut);
            TripGuidance.CourseSegment Join = SegmentAfter(Course, Cuts[0]);
            Assert.True(Join.Outside);
            Assert.InRange(Join.LengthM, Open.LengthM - 0.05, Open.LengthM + 0.05);
        }

        /// <summary>
        /// The turnaround and the empty return have no lane between them, so they
        /// become one Dubins path from the spread end to the next cut.
        /// </summary>
        [Fact]
        public void StackedConnectors_BecomeOneDubins()
        {
            TripGuidance.WorkLane Fill = Lane((10.0, 10.0), (40.0, 10.0), 0.05);
            DubinsConnector.Pose Next = new DubinsConnector.Pose(10.0, 40.0, Math.PI / 2.0);
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane>(),
                new List<TripGuidance.WorkLane> { Fill },
                Next,
                null,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Correcting,
                null,
                null,
                0.0,
                false);
            Assert.Equal(0, CountLanes(Course, TripGuidance.GuidanceAction.Grade));
            Assert.Equal(0, CountAction(Course, TripGuidance.GuidanceAction.Turn));
            Assert.Equal(1, CountAction(Course, TripGuidance.GuidanceAction.Return));
            List<TripGuidance.CourseSegment> Fills = LanesOf(Course, TripGuidance.GuidanceAction.Fill);
            Assert.Single(Fills);
            TripGuidance.CourseSegment Departure = SegmentAfter(Course, Fills[0]);
            Assert.Equal(TripGuidance.GuidanceAction.Return, Departure.Action);
            Assert.Same(Course.Segments[Course.Segments.Count - 1], Departure);
            DubinsConnector.Pose Began = Departure.Pieces[0].Samples[0];
            DubinsConnector.Pose Ended = LastSample(Departure);
            Assert.True(Math.Abs(Began.X - 40.0) < 0.05);
            Assert.True(Math.Abs(Began.Y - 10.0) < 0.05);
            Assert.True(Math.Abs(Ended.X - 10.0) < 0.05);
            Assert.True(Math.Abs(Ended.Y - 40.0) < 0.05);
        }

        /// <summary>
        /// A cell more than four radii past the rest of the line, and shorter
        /// than one radius, is deferred. The line stops on the body.
        /// </summary>
        [Fact]
        public void EndFragment_FartherThanFourRadii_IsDeferred()
        {
            Grid Field = MakeGrid(90, 20, 1.0, new Double[20, 90]);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Column = 10; Column <= 24; Column += 1)
            {
                Cells.Add(((Column, 10), 0.05));
            }

            Cells.Add(((70, 10), 0.05));
            List<(Double X, Double Y)> Line = new List<(Double X, Double Y)>
            {
                (10.5, 10.5),
                (70.5, 10.5),
            };
            TripGuidance.WorkLane Cut = TripGuidance.WorkLane.FromCenterline(Line, Cells, 0.0);
            TripGuidance.DirectionChoice Choice = TripGuidance.ChooseDirections(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane>(),
                null,
                Field,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                2.5,
                1.0,
                1,
                HaulStage.Correcting);
            Assert.Single(Choice.EndCutFragments);
            Assert.Single(Choice.EndCutFragments[0].Cells);
            Assert.Equal(70, Choice.EndCutFragments[0].Cells[0].Cell.Bx);
            Assert.Single(Choice.Cuts);
            Assert.True(Choice.Cuts[0].End.X < 40.0);
        }

        /// <summary>
        /// Deferred end fragments become a sweep-up trip after the correcting pass.
        /// The source trip does not apply them. The sweep-up trip does.
        /// </summary>
        [Fact]
        public void EndFragment_BecomesASweepUpTrip()
        {
            const Int32 Width = 90;
            const Int32 Height = 30;
            Grid Field = MakeGrid(Width, Height, 1.0, new Double[Height, Width]);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            for (Int32 Column = 10; Column <= 24; Column += 1)
            {
                Cuts.Add(((Column, 10), 0.05));
            }

            Cuts.Add(((70, 10), 0.05));
            Cuts.Add(((72, 10), 0.05));
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((10, 20), 0.05),
                    ((16, 20), 0.05),
                };
            Pass Trip = new Pass(Cuts, Fills, 1.0, 0.0, (10.0, 10.0), (24.0, 10.0), (10.0, 20.0), (16.0, 20.0));
            Trip.Stage = HaulStage.Correcting;
            Trip.RibbonOrder = true;
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Field,
                new List<Pass> { Trip },
                Clock(),
                50.0,
                2.5,
                DubinsConnector.DefaultRadiusM,
                -1);
            Assert.True(Job.DeferredFragmentCount >= 1);
            Assert.Contains("Fragments deferred", Job.FormatReport(0.0));
            Assert.Contains("Sweep-up trips", Job.FormatReport(0.0));
            Boolean SawSweep = false;
            foreach (TripGuidance.TripCourse Course in Job.Trips)
            {
                if (Course.SweepUp)
                {
                    SawSweep = true;
                }
            }

            Assert.True(SawSweep);
            // The isolated end is cut on the sweep-up trip. The body stays on the source trip.
            Assert.InRange(Job.Surface[10, 70], -0.06, -0.04);
            Assert.InRange(Job.Surface[10, 72], -0.06, -0.04);
            Assert.InRange(Job.Surface[10, 24], -0.06, -0.04);
        }

        /// <summary>
        /// A spread shorter than half a radius is not a lane between connectors.
        /// The grade follows the long spread. The short spread's cells stay on the pass.
        /// </summary>
        [Fact]
        public void NearEmptyLane_DoesNotSeparateConnectors()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            TripGuidance.WorkLane Cut = Lane((10.0, 30.0), (40.0, 30.0), 0.2);
            TripGuidance.WorkLane LongFill = Lane((10.0, 10.0), (40.0, 10.0), 0.2);
            TripGuidance.WorkLane TinyFill = Lane((41.0, 10.0), (42.0, 10.0), 0.2);
            Assert.True(TinyFill.LengthM < (0.5 * Radius));
            Assert.True(LongFill.LengthM > (0.5 * Radius));
            DubinsConnector.Pose Next = new DubinsConnector.Pose(10.0, 50.0, Math.PI / 2.0);
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { LongFill, TinyFill },
                Next,
                null,
                Clock(),
                Radius,
                1.0,
                1,
                HaulStage.Flow);
            Assert.Single(LanesOf(Course, TripGuidance.GuidanceAction.Fill));
            Assert.Single(LanesOf(Course, TripGuidance.GuidanceAction.Grade));
            foreach (TripGuidance.CourseSegment Segment in Course.Segments)
            {
                if (Segment.IsLane && (Segment.Action != TripGuidance.GuidanceAction.Grade))
                {
                    Assert.True(Segment.LengthM >= (0.5 * Radius));
                }
            }
        }

        /// <summary>
        /// A lane longer than half a radius is still skipped when its volume is
        /// under one percent of the bowl. A heavier lane of the same length stays.
        /// </summary>
        [Fact]
        public void LightLane_DoesNotSeparateConnectors()
        {
            Double Radius = DubinsConnector.DefaultRadiusM;
            TripGuidance.WorkLane Cut = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)> { (0.0, 20.0), (20.0, 20.0) },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { ((0, 20), 2.0) },
                0.0);
            TripGuidance.WorkLane Light = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)> { (0.0, 10.0), (10.0, 10.0) },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { ((0, 10), 0.001) },
                0.0);
            TripGuidance.WorkLane Heavy = TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)> { (0.0, 0.0), (10.0, 0.0) },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)> { ((0, 0), 5.0) },
                0.0);
            Assert.True(Light.LengthM > (0.5 * Radius));
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { Light, Heavy },
                null,
                null,
                Clock(),
                Radius,
                1.0,
                1,
                HaulStage.Flow,
                null,
                null,
                0.0,
                false,
                1.0,
                100.0);
            List<TripGuidance.CourseSegment> Fills = LanesOf(Course, TripGuidance.GuidanceAction.Fill);
            Assert.Single(Fills);
            Assert.InRange(Fills[0].LengthM, 9.0, 11.0);
        }

        /// <summary>
        /// A short correcting trip merges into a later trip whose route passes
        /// within two radii, even when the two lines are not parallel.
        /// </summary>
        [Fact]
        public void Leftover_MergesIntoALaterRouteWithinTwoRadii()
        {
            Grid Field = MakeGrid(40, 40, 1.0, new Double[40, 40]);
            Pass Fragment = MakePass((8, 10), (10, 10), (8, 12), (10, 12));
            Pass Later = MakePass((18, 4), (18, 28), (22, 4), (22, 28));
            Fragment.Stage = HaulStage.Correcting;
            Later.Stage = HaulStage.Correcting;
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Field,
                new List<Pass> { Fragment, Later },
                Clock(),
                50.0,
                2.5,
                DubinsConnector.DefaultRadiusM,
                -1);
            Assert.Equal(1, Job.MergedCount);
            Assert.Equal(0, Job.DroppedCount);
            Assert.Single(Job.Trips);
        }

        /// <summary>
        /// The same lanes produce the same points and the same times.
        /// </summary>
        [Fact]
        public void SameInput_IsIdentical()
        {
            List<TripGuidance.WorkLane> Cuts = new List<TripGuidance.WorkLane>
            {
                TripGuidance.WorkLane.FromCenterline(
                    new List<(Double X, Double Y)> { (0.0, 0.0), (30.0, 0.0) },
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                    0.0),
                TripGuidance.WorkLane.FromCenterline(
                    new List<(Double X, Double Y)> { (30.0, 5.0), (0.0, 5.0) },
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                    Math.PI),
            };
            List<TripGuidance.WorkLane> Fills = new List<TripGuidance.WorkLane>
            {
                TripGuidance.WorkLane.FromCenterline(
                    new List<(Double X, Double Y)> { (0.0, 20.0), (30.0, 20.0) },
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(),
                    0.0),
            };
            DubinsConnector.Pose Next = new DubinsConnector.Pose(0.0, 40.0, 0.0);
            TimeParams Timing = Clock();
            TripGuidance.TripCourse First = TripGuidance.BuildCourse(
                Cuts, Fills, Next, null, Timing, DubinsConnector.DefaultRadiusM, 1.0, 1, HaulStage.Flow);
            TripGuidance.TripCourse Second = TripGuidance.BuildCourse(
                Cuts, Fills, Next, null, Timing, DubinsConnector.DefaultRadiusM, 1.0, 1, HaulStage.Flow);
            Assert.Equal(First.Points.Count, Second.Points.Count);
            for (Int32 Index = 0; Index < First.Points.Count; Index += 1)
            {
                TripGuidance.GuidancePoint Left = First.Points[Index];
                TripGuidance.GuidancePoint Right = Second.Points[Index];
                Assert.Equal(Left.Sequence, Right.Sequence);
                Assert.Equal(Left.X, Right.X);
                Assert.Equal(Left.Y, Right.Y);
                Assert.Equal(Left.HeadingRad, Right.HeadingRad);
                Assert.Equal(Left.Action, Right.Action);
                Assert.Equal(Left.BladeDown, Right.BladeDown);
            }

            Assert.Equal(First.CutSeconds, Second.CutSeconds);
            Assert.Equal(First.LoadedSeconds, Second.LoadedSeconds);
            Assert.Equal(First.SpreadSeconds, Second.SpreadSeconds);
            Assert.Equal(First.TurnaroundSeconds, Second.TurnaroundSeconds);
            Assert.Equal(First.GradeSeconds, Second.GradeSeconds);
            Assert.Equal(First.EmptySeconds, Second.EmptySeconds);
            Assert.Equal(First.Notes.Count, Second.Notes.Count);
            for (Int32 Index = 0; Index < First.Notes.Count; Index += 1)
            {
                Assert.Equal(First.Notes[Index].Word, Second.Notes[Index].Word);
                Assert.Equal(First.Notes[Index].LengthM, Second.Notes[Index].LengthM);
            }
        }

        /// <summary>
        /// Route 2989 keeps a single arc from the cut into the spread, then turns
        /// around. A correcting trip does not grade. The same run writes the Shop 2
        /// hour report and the sheet.
        /// </summary>
        [Fact]
        public void Route2989_CutHaulIsOneArc_AndGradeFollowsSpread()
        {
            String AgdPath = Path.Combine(AppContext.BaseDirectory, "Fixtures", "TheShop2_2ft.agd");
            HaulBuilderOptions Options = new HaulBuilderOptions();
            AgdField Field = Agd.Parse(AgdPath);
            Grid Grid = GridMath.BuildGrid(
                Field,
                Options.GridSizeFt,
                Options.SwellFactor,
                Options.CutFillRatio,
                Options.MaxDepthMm);
            FlowField Flow = FlowFieldSolver.Compute(Grid, "poisson");
            HaulWorkLedger Ledger = HaulWorkLedger.Build(Grid, Options.ToleranceMm);
            Double ScraperBcy = Options.ScraperCapacityLcy / Options.SwellFactor;
            TimeParams Timing = new TimeParams(
                Options.CutSpeedMph,
                Options.LoadedTravelSpeedMph,
                Options.FillSpeedMph,
                Options.UnloadedTravelSpeedMph,
                Options.AccelerationTimeS,
                Options.DecelerationTimeS,
                Options.FieldEfficiency);
            HaulOptimizationResult Solved = FlowRibbonSearch.Optimize(
                Grid,
                Flow,
                Ledger,
                HaulLaneBuilder.LaneAxis.UserHeading,
                ScraperBcy,
                Options.Optimization.MaxLaneSlicesPerCycle,
                true,
                Options.Optimization.FlowPenaltyWeight,
                null,
                HaulLaneBuilder.NormalizeAxisHeadingDeg(Options.LaneAxisHeadingDeg),
                false,
                Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                Options.MinimumTurningDiameterFt,
                Options.Optimization.ClusterModeAverageWindow,
                Options.Optimization.ClusterModeLoadFraction,
                Clock: Timing);
            Assert.True(Solved.Validation.Success);
            List<Pass> Passes = Solved.PassResult.Passes;
            Assert.True(Passes.Count >= 2989);
            Double PreviousSeconds = 0.0;
            foreach (Pass Pass in Passes)
            {
                PreviousSeconds += TimeModel.EstimatePassTime(Grid, Pass, Timing);
            }

            Double PreviousHours = PreviousSeconds / Constants.SECONDS_PER_HOUR;
            Double BowlM3 = ScraperBcy / Constants.M3_TO_CY;
            TripGuidance.JobGuidance Job = TripGuidance.Build(
                Grid,
                Passes,
                Timing,
                BowlM3,
                Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                DubinsConnector.DefaultRadiusM,
                2988);
            String Folder = Path.GetFullPath(Path.Combine(
                AppContext.BaseDirectory,
                "..",
                "..",
                "..",
                "..",
                "Fixtures",
                "FlowRibbonProof",
                "shop2-2026-11-20-found",
                "correcting-sample"));
            Directory.CreateDirectory(Folder);
            File.WriteAllText(
                Path.Combine(Folder, "dubins-guidance-report.txt"),
                Job.FormatReport(PreviousHours));
            TripGuidance.TripCourse? Found = null;
            foreach (TripGuidance.TripCourse Trip in Job.Trips)
            {
                if (Trip.TripNumber == 2989)
                {
                    Found = Trip;
                    break;
                }
            }

            if (Found == null)
            {
                throw new InvalidOperationException("Trip 2989 was merged or dropped.");
            }

            TripGuidance.TripCourse Route = Found;
            Assert.Equal(2989, Route.TripNumber);
            String Picture = Path.Combine(Folder, "haul-2989-dubins.png");
            TripGuidanceSheet.Save(
                Grid,
                Job.Snapshot ?? Job.Surface,
                Route,
                Picture,
                Options.BladeWidthMm / Constants.MILLIMETERS_PER_METER,
                10);
            Int32 LastCut = -1;
            Int32 FirstFill = -1;
            Int32 LastFill = -1;
            for (Int32 Index = 0; Index < Route.Segments.Count; Index += 1)
            {
                TripGuidance.CourseSegment Segment = Route.Segments[Index];
                if (Segment.IsLane && (Segment.Action == TripGuidance.GuidanceAction.Cut))
                {
                    LastCut = Index;
                }

                if (Segment.IsLane && (Segment.Action == TripGuidance.GuidanceAction.Fill))
                {
                    if (FirstFill < 0)
                    {
                        FirstFill = Index;
                    }

                    LastFill = Index;
                }
            }

            Assert.True(LastCut >= 0);
            Assert.True(FirstFill > LastCut);
            TripGuidance.CourseSegment? Haul = null;
            for (Int32 Index = LastCut + 1; Index < FirstFill; Index += 1)
            {
                if (!Route.Segments[Index].IsLane)
                {
                    Haul = Route.Segments[Index];
                }
            }

            Assert.NotNull(Haul);
            String Text = DescribeSegment(Haul);
            File.WriteAllText(Path.Combine(Path.GetTempPath(), "dubins-2989.txt"), Text);
            Double Radius = DubinsConnector.DefaultRadiusM;
            Int32 Arcs = 0;
            DubinsConnector.Pose? Cursor = null;
            foreach (TripGuidance.DrawnPiece Piece in Haul.Pieces)
            {
                if (Piece.Kind != DubinsConnector.PieceKind.Straight)
                {
                    Assert.True(Piece.LengthM < (FULL_CIRCLE * 2.0 * Math.PI * Radius), Text);
                }
                if (Piece.Samples.Count == 0)
                {
                    continue;
                }

                DubinsConnector.Pose Begin = Piece.Samples[0];
                DubinsConnector.Pose Stop = DubinsConnector.Advance(Begin, Piece.Kind, Piece.LengthM, Radius);
                Double Turn = Math.Abs(DubinsConnector.WrapPi(Stop.HeadingRad - Begin.HeadingRad));
                if (Turn > 1e-3)
                {
                    Assert.NotEqual(DubinsConnector.PieceKind.Straight, Piece.Kind);
                }

                if (Cursor.HasValue)
                {
                    Double Joint = Math.Abs(DubinsConnector.WrapPi(Begin.HeadingRad - Cursor.Value.HeadingRad));
                    Assert.True(Joint < 1e-3, Text);
                }

                Cursor = Stop;
                if ((Piece.Kind != DubinsConnector.PieceKind.Straight) && (Piece.LengthM > ARC_DUST_M))
                {
                    Arcs += 1;
                }
            }

            // The haul is the straight between two arcs. Leaving the cut is the
            // first arc. Arriving at the spread is the one arc after that straight.
            Int32 FirstStraight = -1;
            Int32 LastStraight = -1;
            for (Int32 Index = 0; Index < Haul.Pieces.Count; Index += 1)
            {
                if (Haul.Pieces[Index].Kind != DubinsConnector.PieceKind.Straight)
                {
                    continue;
                }

                if (FirstStraight < 0)
                {
                    FirstStraight = Index;
                }

                LastStraight = Index;
            }

            if (FirstStraight < 0)
            {
                Assert.True(Arcs == 1, Text);
            }
            else
            {
                Int32 Before = 0;
                Int32 After = 0;
                for (Int32 Index = 0; Index < Haul.Pieces.Count; Index += 1)
                {
                    TripGuidance.DrawnPiece Piece = Haul.Pieces[Index];
                    if ((Piece.Kind == DubinsConnector.PieceKind.Straight) || (Piece.LengthM <= ARC_DUST_M))
                    {
                        continue;
                    }

                    if (Index < FirstStraight)
                    {
                        Before += 1;
                    }

                    if (Index > LastStraight)
                    {
                        After += 1;
                    }
                }

                Assert.True(Before >= 1, Text);
                Assert.True(After == 1, Text);
            }
            Assert.True(LastFill >= 0);
            // The turnaround and the empty return meet, so they are one connector.
            Int32 Departure = 0;
            for (Int32 Index = LastFill + 1; Index < Route.Segments.Count; Index += 1)
            {
                if (Route.Segments[Index].IsLane)
                {
                    break;
                }

                Departure += 1;
            }

            Assert.Equal(1, Departure);
            Assert.Equal(0, CountLanes(Route, TripGuidance.GuidanceAction.Grade));
        }

        /// <summary>
        /// Position and heading meet both ends. Heading does not jump between pieces.
        /// Curvature stays at or under 1/R.
        /// </summary>
        /// <param name="Path">Connector under test.</param>
        /// <param name="Radius">Turning radius, metres.</param>
        private static void AssertConnector
            (
            DubinsConnector.Path Path,
            Double Radius
            )
        {
            String Text = Describe(Path);
            DubinsConnector.Pose Arrival = DubinsConnector.Integrate(Path.Start, Path.Pieces, Radius);
            Double Dx = Arrival.X - Path.End.X;
            Double Dy = Arrival.Y - Path.End.Y;
            Assert.True(((Dx * Dx) + (Dy * Dy)) < 1e-4, Text);
            Assert.True(Math.Abs(DubinsConnector.WrapPi(Arrival.HeadingRad - Path.End.HeadingRad)) < 1e-3, Text);
            DubinsConnector.Pose Cursor = Path.Start;
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                Assert.True(Piece.CurvaturePerM <= (1.0 / Radius) + 1e-9, Text);
                Double Signed = 0.0;
                if (Piece.Kind == DubinsConnector.PieceKind.Left)
                {
                    Signed = 1.0;
                }
                else if (Piece.Kind == DubinsConnector.PieceKind.Right)
                {
                    Signed = -1.0;
                }

                DubinsConnector.Pose Next = DubinsConnector.Advance(Cursor, Piece.Kind, Piece.LengthM, Radius);
                Double ExpectedTurn = Signed * Piece.LengthM / Radius;
                Double ActualTurn = DubinsConnector.WrapPi(Next.HeadingRad - Cursor.HeadingRad);
                Assert.True(Math.Abs(DubinsConnector.WrapPi(ActualTurn - ExpectedTurn)) < 1e-6, Text);
                Cursor = Next;
            }

            List<DubinsConnector.Pose> Samples = DubinsConnector.Sample(
                Path.Start,
                Path.Pieces,
                DubinsConnector.SAMPLE_M,
                Radius,
                Path.End);
            Assert.InRange(Math.Abs(Samples[0].X - Path.Start.X), 0.0, 1e-6);
            Assert.InRange(Math.Abs(Samples[0].Y - Path.Start.Y), 0.0, 1e-6);
            DubinsConnector.Pose Tail = Samples[Samples.Count - 1];
            Assert.InRange(Math.Abs(Tail.X - Path.End.X), 0.0, 1e-4);
            Assert.InRange(Math.Abs(Tail.Y - Path.End.Y), 0.0, 1e-4);
            Assert.True(Math.Abs(DubinsConnector.WrapPi(Tail.HeadingRad - Path.End.HeadingRad)) < 1e-3, Text);
            for (Int32 Index = 1; Index < Samples.Count; Index += 1)
            {
                Double StepX = Samples[Index].X - Samples[Index - 1].X;
                Double StepY = Samples[Index].Y - Samples[Index - 1].Y;
                Double Step = Math.Sqrt((StepX * StepX) + (StepY * StepY));
                Double Turn = Math.Abs(DubinsConnector.WrapPi(Samples[Index].HeadingRad - Samples[Index - 1].HeadingRad));
                Assert.True(Turn <= (Step / Radius) + 0.02, Text);
            }
        }

        /// <summary>
        /// Heading along a trip changes only as fast as the turning radius allows.
        /// </summary>
        /// <param name="Points">Guidance samples.</param>
        /// <param name="Radius">Turning radius, metres.</param>
        private static void AssertHeadingSteps
            (
            List<TripGuidance.GuidancePoint> Points,
            Double Radius
            )
        {
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                Double Dx = Points[Index].X - Points[Index - 1].X;
                Double Dy = Points[Index].Y - Points[Index - 1].Y;
                Double Step = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                Double Turn = Math.Abs(DubinsConnector.WrapPi(Points[Index].HeadingRad - Points[Index - 1].HeadingRad));
                Assert.True(Turn <= (Step / Radius) + 0.05);
            }
        }

        /// <summary>
        /// Piece list for an assertion message.
        /// </summary>
        /// <param name="Path">Connector.</param>
        /// <returns>One line.</returns>
        private static String Describe
            (
            DubinsConnector.Path Path
            )
        {
            StringBuilder Text = new StringBuilder();
            Text.Append(Path.Word);
            Text.Append(' ');
            Text.Append(Path.LengthM.ToString("F3", CultureInfo.InvariantCulture));
            foreach (DubinsConnector.Piece Piece in Path.Pieces)
            {
                Text.Append(' ');
                Text.Append(Piece.Kind);
                Text.Append(' ');
                Text.Append(Piece.LengthM.ToString("F3", CultureInfo.InvariantCulture));
            }

            return Text.ToString();
        }

        /// <summary>
        /// Piece list for a drawn connector.
        /// </summary>
        /// <param name="Segment">Connector segment.</param>
        /// <returns>One line.</returns>
        private static String DescribeSegment
            (
            TripGuidance.CourseSegment Segment
            )
        {
            StringBuilder Text = new StringBuilder();
            Text.Append(Segment.Word);
            Text.Append(' ');
            Text.Append(Segment.LengthM.ToString("F3", CultureInfo.InvariantCulture));
            foreach (TripGuidance.DrawnPiece Piece in Segment.Pieces)
            {
                Text.Append(' ');
                Text.Append(Piece.Kind);
                Text.Append(' ');
                Text.Append(Piece.LengthM.ToString("F3", CultureInfo.InvariantCulture));
            }

            return Text.ToString();
        }

        /// <summary>
        /// Outside-connector count and productive seconds of one course.
        /// </summary>
        private struct TripCourseScore
        {
            /// <summary>
            /// Initializes a score.
            /// </summary>
            /// <param name="Outside">Connectors that left the field.</param>
            /// <param name="Seconds">Productive seconds.</param>
            public TripCourseScore
                (
                Int32 Outside,
                Double Seconds
                )
            {
                this.Outside = Outside;
                this.Seconds = Seconds;
            }

            /// <summary>Gets the outside-connector count.</summary>
            public Int32 Outside { get; }

            /// <summary>Gets productive seconds.</summary>
            public Double Seconds { get; }
        }

        /// <summary>
        /// Builds one cut and one spread and scores the connectors.
        /// </summary>
        /// <param name="Cut">Cut lane.</param>
        /// <param name="Fill">Spread lane.</param>
        /// <param name="Field">Field mask.</param>
        /// <returns>Outside count, then productive seconds.</returns>
        private static TripCourseScore ScoreCourse
            (
            TripGuidance.WorkLane Cut,
            TripGuidance.WorkLane Fill,
            Grid Field
            )
        {
            TripGuidance.TripCourse Course = TripGuidance.BuildCourse(
                new List<TripGuidance.WorkLane> { Cut },
                new List<TripGuidance.WorkLane> { Fill },
                null,
                Field.ContainsWorldXy,
                Clock(),
                DubinsConnector.DefaultRadiusM,
                1.0,
                1,
                HaulStage.Flow);
            Int32 Outside = 0;
            foreach (TripGuidance.ConnectorNote Note in Course.Notes)
            {
                if (Note.Outside)
                {
                    Outside += 1;
                }
            }

            return new TripCourseScore(Outside, Course.ProductiveSeconds);
        }

        /// <summary>
        /// The machine clock the hour comparison uses.
        /// </summary>
        /// <returns>Default speeds and the 12 second accel and decel.</returns>
        private static TimeParams Clock
            (
            )
        {
            return new TimeParams(
                Constants.DEFAULT_CUT_SPEED_MPH,
                Constants.DEFAULT_LOADED_TRAVEL_SPEED_MPH,
                Constants.DEFAULT_FILL_SPEED_MPH,
                Constants.DEFAULT_UNLOADED_TRAVEL_SPEED_MPH,
                Constants.DEFAULT_ACCELERATION_TIME_S,
                Constants.DEFAULT_DECELERATION_TIME_S,
                Constants.DEFAULT_FIELD_EFFICIENCY);
        }

        /// <summary>
        /// A straight lane whose cells cover the centreline, one per metre.
        /// </summary>
        /// <param name="Start">First end, metres.</param>
        /// <param name="End">Other end, metres.</param>
        /// <param name="DepthM">Depth on every cell.</param>
        /// <returns>The lane.</returns>
        private static TripGuidance.WorkLane CellsAlong
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double DepthM
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double Length = Math.Sqrt((Dx * Dx) + (Dy * Dy));
            Int32 Steps = Math.Max(1, (Int32)Math.Round(Length));
            for (Int32 Step = 0; Step <= Steps; Step += 1)
            {
                Double X = Start.X + (Dx * Step / Steps);
                Double Y = Start.Y + (Dy * Step / Steps);
                Cells.Add((((Int32)Math.Round(X), (Int32)Math.Round(Y)), DepthM));
            }

            return TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)> { Start, End },
                Cells,
                0.0);
        }

        /// <summary>
        /// One straight lane with a single cell depth, so a correcting grade can see it.
        /// </summary>
        /// <param name="Start">Lane start, metres.</param>
        /// <param name="End">Lane end, metres.</param>
        /// <param name="DepthM">Spread or cut depth, metres.</param>
        /// <returns>The lane.</returns>
        private static TripGuidance.WorkLane Lane
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double DepthM
            )
        {
            return TripGuidance.WorkLane.FromCenterline(
                new List<(Double X, Double Y)> { Start, End },
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (((Int32)Start.X, (Int32)Start.Y), DepthM),
                },
                0.0);
        }

        /// <summary>
        /// Blade-down lanes of one action, in drive order.
        /// </summary>
        /// <param name="Course">Trip under test.</param>
        /// <param name="Action">Cut, fill, or grade.</param>
        /// <returns>The lanes.</returns>
        private static List<TripGuidance.CourseSegment> LanesOf
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
        /// Last sample of a lane.
        /// </summary>
        /// <param name="Segment">Lane segment.</param>
        /// <returns>The end pose.</returns>
        private static DubinsConnector.Pose LastSample
            (
            TripGuidance.CourseSegment Segment
            )
        {
            TripGuidance.DrawnPiece Piece = Segment.Pieces[Segment.Pieces.Count - 1];
            return Piece.Samples[Piece.Samples.Count - 1];
        }

        /// <summary>
        /// The segment that follows a lane.
        /// </summary>
        /// <param name="Course">Trip under test.</param>
        /// <param name="Lane">Lane to look after.</param>
        /// <returns>The next segment.</returns>
        private static TripGuidance.CourseSegment SegmentAfter
            (
            TripGuidance.TripCourse Course,
            TripGuidance.CourseSegment Lane
            )
        {
            for (Int32 Index = 0; Index < Course.Segments.Count - 1; Index += 1)
            {
                if (Object.ReferenceEquals(Course.Segments[Index], Lane))
                {
                    return Course.Segments[Index + 1];
                }
            }

            throw new InvalidOperationException("The lane is not followed by a segment.");
        }

        /// <summary>
        /// How many segments use an action.
        /// </summary>
        /// <param name="Course">Trip under test.</param>
        /// <param name="Action">Action to count.</param>
        /// <returns>The count.</returns>
        private static Int32 CountAction
            (
            TripGuidance.TripCourse Course,
            TripGuidance.GuidanceAction Action
            )
        {
            Int32 Count = 0;
            foreach (TripGuidance.CourseSegment Segment in Course.Segments)
            {
                if (Segment.Action == Action)
                {
                    Count += 1;
                }
            }

            return Count;
        }

        /// <summary>
        /// How many blade-down lanes use an action.
        /// </summary>
        /// <param name="Course">Trip under test.</param>
        /// <param name="Action">Action to count.</param>
        /// <returns>The count.</returns>
        private static Int32 CountLanes
            (
            TripGuidance.TripCourse Course,
            TripGuidance.GuidanceAction Action
            )
        {
            return LanesOf(Course, Action).Count;
        }

        /// <summary>
        /// One ribbon trip of two cut cells and two fill cells.
        /// </summary>
        /// <param name="CutA">First cut cell.</param>
        /// <param name="CutB">Second cut cell.</param>
        /// <param name="FillA">First fill cell.</param>
        /// <param name="FillB">Second fill cell.</param>
        /// <returns>The pass.</returns>
        private static Pass MakePass
            (
            (Int32 Bx, Int32 By) CutA,
            (Int32 Bx, Int32 By) CutB,
            (Int32 Bx, Int32 By) FillA,
            (Int32 Bx, Int32 By) FillB
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cuts =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (CutA, 0.05),
                    (CutB, 0.05),
                };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    (FillA, 0.05),
                    (FillB, 0.05),
                };
            Pass Trip = new Pass(Cuts, Fills, 0.05, 0.0, (CutA.Bx, CutA.By), (CutB.Bx, CutB.By), (FillA.Bx, FillA.By), (FillB.Bx, FillB.By));
            Trip.RibbonOrder = true;
            return Trip;
        }

        /// <summary>
        /// A rectangular field of valid cells.
        /// </summary>
        /// <param name="Width">Columns.</param>
        /// <param name="Height">Rows.</param>
        /// <param name="CellM">Cell size, metres.</param>
        /// <param name="Existing">Existing elevation. Target is zero.</param>
        /// <returns>The grid.</returns>
        private static Grid MakeGrid
            (
            Int32 Width,
            Int32 Height,
            Double CellM,
            Double[,] Existing
            )
        {
            Boolean[,] Valid = new Boolean[Height, Width];
            for (Int32 Row = 0; Row < Height; Row += 1)
            {
                for (Int32 Col = 0; Col < Width; Col += 1)
                {
                    Valid[Row, Col] = true;
                }
            }

            return new Grid(
                CellM,
                Width,
                Height,
                0.0,
                0.0,
                12,
                true,
                0.0,
                0.0,
                Existing,
                new Double[Height, Width],
                Valid,
                1.3,
                1.2,
                61.0,
                new List<List<(Double X, Double Y)>>());
        }
    }
}
