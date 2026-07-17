using System.Globalization;

namespace HaulBuilder.Tests
{
    /// <summary>
    /// Verifies equipment-length straight lineup before and after CUT/FILL runs.
    /// </summary>
    public sealed class EquipmentLineupTests
    {
        /// <summary>
        /// Absolute heading change along a→b→c in degrees.
        /// </summary>
        /// <param name="A">Inbound start.</param>
        /// <param name="B">Corner.</param>
        /// <param name="C">Outbound end.</param>
        /// <returns>Absolute turn angle in degrees.</returns>
        private static Double HeadingChangeDeg
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            (Double X, Double Y) C
            )
        {
            Double Dx1 = B.X - A.X;
            Double Dy1 = B.Y - A.Y;
            Double Dx2 = C.X - B.X;
            Double Dy2 = C.Y - B.Y;
            return Math.Abs((Math.Atan2((Dx1 * Dy2) - (Dy1 * Dx2), (Dx1 * Dx2) + (Dy1 * Dy2))
                * (Constants.DEGREES_PER_HALF_TURN / Math.PI)));
        }

        /// <summary>
        /// Straight blade-up length immediately before the first blade-down point of <paramref name="Action"/>.
        /// </summary>
        /// <param name="Points">Guidance waypoints.</param>
        /// <param name="Action">CUT or FILL.</param>
        /// <returns>Along-track approach length in metres.</returns>
        private static Double StraightApproachBeforeActionM
            (
            IReadOnlyList<GuidancePoint> Points,
            String Action
            )
        {
            Int32 Tip = -1;
            for (Int32 I = 0; I < Points.Count; I += 1)
            {
                if (Points[I].Action == Action)
                {
                    Tip = I;
                    break;
                }
            }

            Assert.True(Tip > 0, $"expected a {Action} tip with prior points");
            (Double X, Double Y) TipXy = (Points[Tip].X, Points[Tip].Y);
            (Double X, Double Y) NextXy = Tip + 1 < Points.Count
                ? (Points[Tip + 1].X, Points[Tip + 1].Y)
                : TipXy;
            Double OutDx = NextXy.X - TipXy.X;
            Double OutDy = NextXy.Y - TipXy.Y;
            Double OutLen = Math.Sqrt((OutDx * OutDx) + (OutDy * OutDy));
            Assert.True(OutLen > Constants.GEOMETRY_COMPARISON_TOLERANCE, $"expected outbound leg after {Action}");
            Double Ux = OutDx / OutLen;
            Double Uy = OutDy / OutLen;

            Double Along = 0.0;
            (Double X, Double Y) Cur = TipXy;
            for (Int32 I = Tip - 1; I >= 0; I -= 1)
            {
                GuidancePoint P = Points[I];
                if ((P.Action != "MOVE") && (P.Action != "START") && (P.Action != "RETURN")
                    && (P.Action != "HAUL"))
                {
                    break;
                }

                (Double X, Double Y) Q = (P.X, P.Y);
                Double Dx = Cur.X - Q.X;
                Double Dy = Cur.Y - Q.Y;
                Double Seg = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Seg < Constants.NUMERIC_COMPARISON_TOLERANCE)
                {
                    Cur = Q;
                    continue;
                }

                Double Dot = ((Dx / Seg) * Ux) + ((Dy / Seg) * Uy);
                if (Dot < 0.95)
                {
                    break;
                }

                Along += Seg;
                Cur = Q;
            }

            return Along;
        }

        /// <summary>
        /// Straight blade-up length immediately after the last contiguous tip of <paramref name="Action"/>.
        /// </summary>
        /// <param name="Points">Guidance waypoints.</param>
        /// <param name="Action">CUT or FILL.</param>
        /// <returns>Along-track exit length in metres.</returns>
        private static Double StraightExitAfterActionM
            (
            IReadOnlyList<GuidancePoint> Points,
            String Action
            )
        {
            Int32 Tip = -1;
            for (Int32 I = 0; I < Points.Count; I += 1)
            {
                if (Points[I].Action == Action)
                {
                    Tip = I;
                }
                else if (Tip >= 0)
                {
                    // Intent: first contiguous run of this action only.
                    break;
                }
            }

            Assert.True(Tip >= 1, $"expected a {Action} tip");
            (Double X, Double Y) TipXy = (Points[Tip].X, Points[Tip].Y);
            (Double X, Double Y) PrevXy = (Points[Tip - 1].X, Points[Tip - 1].Y);
            Double InDx = TipXy.X - PrevXy.X;
            Double InDy = TipXy.Y - PrevXy.Y;
            Double InLen = Math.Sqrt((InDx * InDx) + (InDy * InDy));
            // Intent: tip may sit on a pinned mid-cell; walk back for inbound heading.
            if (InLen < Constants.GEOMETRY_COMPARISON_TOLERANCE)
            {
                for (Int32 J = Tip - 2; J >= 0; J -= 1)
                {
                    if (Points[J].Action != Action)
                    {
                        break;
                    }

                    PrevXy = (Points[J].X, Points[J].Y);
                    InDx = TipXy.X - PrevXy.X;
                    InDy = TipXy.Y - PrevXy.Y;
                    InLen = Math.Sqrt((InDx * InDx) + (InDy * InDy));
                    if (InLen > Constants.GEOMETRY_COMPARISON_TOLERANCE)
                    {
                        break;
                    }
                }
            }

            Assert.True(InLen > Constants.GEOMETRY_COMPARISON_TOLERANCE, $"expected inbound leg into {Action} tip");
            Double Ux = InDx / InLen;
            Double Uy = InDy / InLen;

            Double Along = 0.0;
            (Double X, Double Y) Cur = TipXy;
            for (Int32 I = Tip + 1; I < Points.Count; I += 1)
            {
                GuidancePoint P = Points[I];
                if (P.Action != "MOVE")
                {
                    break;
                }

                (Double X, Double Y) Q = (P.X, P.Y);
                Double Dx = Q.X - Cur.X;
                Double Dy = Q.Y - Cur.Y;
                Double Seg = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (Seg < Constants.NUMERIC_COMPARISON_TOLERANCE)
                {
                    Cur = Q;
                    continue;
                }

                Double Dot = ((Dx / Seg) * Ux) + ((Dy / Seg) * Uy);
                if (Dot < 0.95)
                {
                    break;
                }

                Along += Seg;
                Cur = Q;
            }

            return Along;
        }

        /// <summary>
        /// Default equipment length yields ~L straight MOVE before CUT and after CUT.
        /// </summary>
        [Fact]
        public void Guidance_CutRun_HasEntryAndExitLineup()
        {
            String Path = LandlevelTestsHelpers.CreateMinimalAgd();
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
            Double G = Grid.GridSizeM;

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                // Intent: start inland so a straight entry of L fits inside the field mask.
                ((2, 2), 0.05),
                ((3, 2), 0.05),
                ((4, 2), 0.05),
                ((5, 2), 0.05),
            };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                ((2, 5), 0.05),
                ((3, 5), 0.05),
            };

            (Double X, Double Y) CutStart = Grid.CellCenterXY(2, 2);
            (Double X, Double Y) CutEnd = Grid.CellCenterXY(5, 2);
            (Double X, Double Y) FillStart = Grid.CellCenterXY(2, 5);
            (Double X, Double Y) FillEnd = Grid.CellCenterXY(3, 5);
            Pass Pass = new Pass(CutCells, FillCells, 1.0, G * 3.0, CutStart, CutEnd, FillStart, FillEnd)
            {
                CutKnotReduced = true,
                FillKnotReduced = true,
            };
            Route Route = new Route(new List<Int32> { 0 }, new List<Pass> { Pass }, 0.0);
            Double TurnRadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT * Constants.HALF_SCALE * Constants.FEET_TO_METERS;
            // Intent: L fits in-field west of the cut start; fill is off-axis so exit is not clamped.
            Double LineupM = G;
            GuidanceResult Guidance = Swaths.BuildGuidance(
                Grid,
                Field,
                Route,
                Constants.DEFAULT_BLADE_WIDTH_MM,
                90.0,
                0.0,
                true,
                60.0,
                false,
                TurnRadiusM,
                0.7,
                null,
                null,
                LineupM);

            Double Entry = StraightApproachBeforeActionM(Guidance.Points, "CUT");
            Double Exit = StraightExitAfterActionM(Guidance.Points, "CUT");
            Assert.True(
                Entry >= (0.95 * LineupM),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "CUT entry lineup {0:F2}m < 0.95x{1:F2}m",
                    Entry,
                    LineupM));
            Assert.True(
                Exit >= (0.95 * LineupM),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "CUT exit lineup {0:F2}m < 0.95x{1:F2}m",
                    Exit,
                    LineupM));

            // Intent: exit spur should stay colinear (no early turn within L).
            Int32 CutTip = -1;
            for (Int32 I = 0; I < Guidance.Points.Count; I += 1)
            {
                if (Guidance.Points[I].Action == "CUT")
                {
                    CutTip = I;
                }
                else if (CutTip >= 0)
                {
                    break;
                }
            }

            Assert.True(CutTip >= 1);
            Double Along = 0.0;
            for (Int32 I = CutTip + 1; (I + 1) < Guidance.Points.Count; I += 1)
            {
                if (Guidance.Points[I].Action != "MOVE")
                {
                    break;
                }

                GuidancePoint A = Guidance.Points[I - 1];
                GuidancePoint B = Guidance.Points[I];
                GuidancePoint C = Guidance.Points[I + 1];
                Along += Math.Sqrt(((A.X - B.X) * (A.X - B.X)) + ((A.Y - B.Y) * (A.Y - B.Y)));
                if (Along > (0.9 * LineupM))
                {
                    break;
                }

                if (C.Action != "MOVE")
                {
                    break;
                }

                Double Turn = HeadingChangeDeg((A.X, A.Y), (B.X, B.Y), (C.X, C.Y));
                Assert.True(
                    Turn < 15.0,
                    String.Format(
                        CultureInfo.InvariantCulture,
                        "exit spur turned {0:F1} deg within lineup length",
                        Turn));
            }
        }

        /// <summary>
        /// Absolute signed heading change sum along a polyline (degrees).
        /// </summary>
        /// <param name="Points">Waypoints.</param>
        /// <param name="From">Inclusive start index.</param>
        /// <param name="To">Exclusive end index.</param>
        /// <returns>Signed winding in degrees.</returns>
        private static Double PolylineWindingDeg
            (
            IReadOnlyList<GuidancePoint> Points,
            Int32 From,
            Int32 To
            )
        {
            Double Sum = 0.0;
            for (Int32 I = From + 1; I < (To - 1); I += 1)
            {
                GuidancePoint P0 = Points[I - 1];
                GuidancePoint P1 = Points[I];
                GuidancePoint P2 = Points[I + 1];
                Double Dx1 = P1.X - P0.X;
                Double Dy1 = P1.Y - P0.Y;
                Double Dx2 = P2.X - P1.X;
                Double Dy2 = P2.Y - P1.Y;
                Sum += Math.Atan2((Dx1 * Dy2) - (Dy1 * Dx2), (Dx1 * Dx2) + (Dy1 * Dy2))
                    * (Constants.DEGREES_PER_HALF_TURN / Math.PI);
            }

            return Sum;
        }

        /// <summary>
        /// Same-corridor cut→fill with a short gap must not invent a ~360° lineup loop.
        /// </summary>
        [Fact]
        public void Guidance_ColinearCutFill_NoFullCircleLineup()
        {
            String Path = LandlevelTestsHelpers.CreateMinimalAgd();
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
            Double G = Grid.GridSizeM;

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                ((0, 0), 0.05),
                ((1, 0), 0.05),
                ((2, 0), 0.05),
                ((3, 0), 0.05),
            };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                ((6, 0), 0.05),
                ((7, 0), 0.05),
            };

            Pass Pass = new Pass(
                CutCells,
                FillCells,
                1.0,
                G * 3.0,
                Grid.CellCenterXY(0, 0),
                Grid.CellCenterXY(3, 0),
                Grid.CellCenterXY(6, 0),
                Grid.CellCenterXY(7, 0))
            {
                CutKnotReduced = true,
                FillKnotReduced = true,
            };
            Route Route = new Route(new List<Int32> { 0 }, new List<Pass> { Pass }, 0.0);
            Double TurnRadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT * Constants.HALF_SCALE * Constants.FEET_TO_METERS;
            Double LineupM = Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS;
            GuidanceResult Guidance = Swaths.BuildGuidance(
                Grid,
                Field,
                Route,
                Constants.DEFAULT_BLADE_WIDTH_MM,
                90.0,
                0.0,
                true,
                60.0,
                false,
                TurnRadiusM,
                0.7,
                null,
                null,
                LineupM);

            Int32 CutEnd = -1;
            Int32 FillStart = -1;
            for (Int32 I = 0; I < Guidance.Points.Count; I += 1)
            {
                if (Guidance.Points[I].Action == "CUT")
                {
                    CutEnd = I;
                }

                if ((Guidance.Points[I].Action == "FILL") && (FillStart < 0))
                {
                    FillStart = I;
                }
            }

            Assert.True(CutEnd >= 0 && FillStart > CutEnd);
            Double Winding = PolylineWindingDeg(Guidance.Points, CutEnd, FillStart);
            Assert.True(
                Math.Abs(Winding) < 270.0,
                String.Format(
                    CultureInfo.InvariantCulture,
                    "cut→fill transit winding {0:F0} deg looks like a full-circle lineup",
                    Winding));

            Double TransitLen = 0.0;
            for (Int32 I = CutEnd + 1; I <= FillStart; I += 1)
            {
                GuidancePoint A = Guidance.Points[I - 1];
                GuidancePoint B = Guidance.Points[I];
                TransitLen += Math.Sqrt(((A.X - B.X) * (A.X - B.X)) + ((A.Y - B.Y) * (A.Y - B.Y)));
            }

            Double Chord = Math.Sqrt(
                ((Guidance.Points[FillStart].X - Guidance.Points[CutEnd].X)
                    * (Guidance.Points[FillStart].X - Guidance.Points[CutEnd].X))
                + ((Guidance.Points[FillStart].Y - Guidance.Points[CutEnd].Y)
                    * (Guidance.Points[FillStart].Y - Guidance.Points[CutEnd].Y)));
            Assert.True(
                TransitLen < ((Math.PI * TurnRadiusM) + Chord + LineupM),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "cut→fill transit {0:F1}m too long for short colinear gap (chord={1:F1})",
                    TransitLen,
                    Chord));
        }

        /// <summary>
        /// equipmentLengthM = 0 disables lineup spurs.
        /// </summary>
        [Fact]
        public void Guidance_EquipmentLengthZero_SkipsLineup()
        {
            String Path = LandlevelTestsHelpers.CreateMinimalAgd();
            AgdField Field = Agd.Parse(Path);
            Grid Grid = GridMath.BuildGrid(Field, Constants.DEFAULT_GRID_SIZE_FT);
            Double G = Grid.GridSizeM;

            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> CutCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                ((0, 0), 0.05),
                ((1, 0), 0.05),
                ((2, 0), 0.05),
            };
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> FillCells = new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>
            {
                ((5, 0), 0.05),
                ((6, 0), 0.05),
            };

            Pass Pass = new Pass(
                CutCells,
                FillCells,
                1.0,
                G * 2.0,
                Grid.CellCenterXY(0, 0),
                Grid.CellCenterXY(2, 0),
                Grid.CellCenterXY(5, 0),
                Grid.CellCenterXY(6, 0))
            {
                CutKnotReduced = true,
                FillKnotReduced = true,
            };
            Route Route = new Route(new List<Int32> { 0 }, new List<Pass> { Pass }, 0.0);
            Double TurnRadiusM = Constants.DEFAULT_TURNING_CIRCLE_FT * Constants.HALF_SCALE * Constants.FEET_TO_METERS;
            Double DefaultL = Constants.DEFAULT_EQUIPMENT_LENGTH_FT * Constants.FEET_TO_METERS;
            GuidanceResult Guidance = Swaths.BuildGuidance(
                Grid,
                Field,
                Route,
                Constants.DEFAULT_BLADE_WIDTH_MM,
                90.0,
                0.0,
                true,
                60.0,
                false,
                TurnRadiusM,
                0.7,
                null,
                null,
                0.0);

            Double Entry = StraightApproachBeforeActionM(Guidance.Points, "CUT");
            Double Exit = StraightExitAfterActionM(Guidance.Points, "CUT");
            Assert.True(
                Entry < (Constants.HALF_SCALE * DefaultL),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "expected no full entry lineup when L=0, got {0:F2}m",
                    Entry));
            Assert.True(
                Exit < (Constants.HALF_SCALE * DefaultL),
                String.Format(
                    CultureInfo.InvariantCulture,
                    "expected no full exit lineup when L=0, got {0:F2}m",
                    Exit));
        }
    }
}
