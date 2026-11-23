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
            // The stored lane ran out to the boundary. The chosen lane runs back in.
            Assert.InRange(Started.X, EndX - 0.05, EndX + 0.05);
            Assert.InRange(Finished.X, InteriorX - 0.05, InteriorX + 0.05);
            Assert.True(Started.X > Finished.X);
            Assert.False(Turn.Outside);
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
        /// A spread running west into the west boundary takes a quarter circle
        /// along the edge. The turnaround and the grade pass stay in the field.
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
            Double Quarter = DubinsConnector.DefaultRadiusM * Math.PI * 0.5;
            Assert.InRange(Chosen.ArcLengthM, Quarter - 0.05, Quarter + 0.05);
            Assert.InRange(Math.Abs(DubinsConnector.WrapPi(Chosen.Start.HeadingRad - Math.PI)), 0.0, 0.2);
            Assert.InRange(Chosen.Start.X, 70.0, 78.0);
            Assert.InRange(Chosen.End.X, (BladeM * 0.5) - 0.35, (BladeM * 0.5) + 0.35);
            // North or south: both quarter circles fit on this lane.
            Assert.InRange(Math.Abs(Math.Abs(DubinsConnector.WrapPi(Chosen.End.HeadingRad)) - (Math.PI * 0.5)), 0.0, 0.2);
            AssertCourseStaysIn(Choice.Course, Field);
        }

        /// <summary>
        /// A lane that ends in a corner bends toward the open side.
        /// The turn into the boundary is dropped.
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
            Assert.True(Chosen.ArcLengthM > 1.0);
            Assert.InRange(Chosen.End.X, (BladeM * 0.5) - 0.35, (BladeM * 0.5) + 0.35);
            // South leaves the field. The surviving arc heads north.
            Assert.InRange(DubinsConnector.WrapPi(Chosen.End.HeadingRad - (Math.PI * 0.5)), -0.2, 0.2);
            Assert.True(Chosen.End.Y > LaneY + 4.0);
            AssertCourseStaysIn(Choice.Course, Field);
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
            Assert.Contains((1, 50), Left);
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
            Double[,] Existing = new Double[1, 2];
            Existing[0, 0] = HighM;
            Existing[0, 1] = HighM;
            Grid Field = MakeGrid(2, 1, 1.0, Existing);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Fills =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                {
                    ((0, 0), 0.0),
                    ((1, 0), 0.0),
                };
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
            Assert.InRange(Job.Surface[0, 0], -1e-9, 1e-9);
            Assert.InRange(Job.Surface[0, 1], -1e-9, 1e-9);
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
        /// around and grades. The same run writes the Shop 2 hour report and the sheet.
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
            TripGuidance.TripCourse Route = Job.Trips[2988];
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
            Assert.Equal(TripGuidance.GuidanceAction.Turn, Route.Segments[LastFill + 1].Action);
            Boolean Graded = false;
            for (Int32 Index = LastFill + 1; Index < Route.Segments.Count; Index += 1)
            {
                TripGuidance.CourseSegment Segment = Route.Segments[Index];
                if (Segment.IsLane && (Segment.Action == TripGuidance.GuidanceAction.Grade))
                {
                    Graded = true;
                }
            }

            Assert.True(Graded);
            foreach (TripGuidance.CourseSegment Segment in Route.Segments)
            {
                Assert.False(Segment.Outside, DescribeSegment(Segment));
            }
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
