using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// One streamline strip with cell traversal and fractional path.
    /// </summary>
    public sealed class Strip
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="Strip"/> class.
        /// </summary>
        /// <param name="Cells">Ordered visited cells.</param>
        /// <param name="Path">Fractional path points.</param>
        public Strip
            (
            List<(Int32 Bx, Int32 By)> Cells,
            List<(Double X, Double Y)> Path
            )
        {
            // Persist traced discrete and continuous strip geometry.
            this.Cells = Cells;
            this.Path = Path;
        }

        /// <summary>
        /// Gets visited cells.
        /// </summary>
        public List<(Int32 Bx, Int32 By)> Cells { get; }

        /// <summary>
        /// Gets fractional streamline path.
        /// </summary>
        public List<(Double X, Double Y)> Path { get; }
    }

    /// <summary>
    /// Streamline tracing utilities.
    /// </summary>
    public static class Strips
    {
        /// <summary>
        /// Bilinearly samples a field at fractional grid coordinates.
        /// </summary>
        /// <param name="FieldArr">Field to sample.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="X">Fractional x coordinate (bx).</param>
        /// <param name="Y">Fractional y coordinate (by).</param>
        /// <returns>Interpolated value.</returns>
        static public Double Bilinear
            (
            Double[,] FieldArr,
            Boolean[,] Valid,
            Double X,
            Double Y
            )
        {
            // Blend four corners while ignoring invalid neighbors.
            Int32 Height = FieldArr.GetLength(0);
            Int32 Width = FieldArr.GetLength(1);
            Int32 X0 = (Int32)Math.Floor(X);
            Int32 Y0 = (Int32)Math.Floor(Y);
            Double Fx = X - X0;
            Double Fy = Y - Y0;
            Double W00 = (1.0 - Fx) * (1.0 - Fy);
            Double W10 = Fx * (1.0 - Fy);
            Double W01 = (1.0 - Fx) * Fy;
            Double W11 = Fx * Fy;
            Double Acc = 0.0;
            Double WSum = 0.0;
            if ((X0 >= 0) && (X0 < Width) && (Y0 >= 0) && (Y0 < Height) && Valid[Y0, X0])
            {
                Acc += FieldArr[Y0, X0] * W00;
                WSum += W00;
            }

            Int32 X1 = X0 + 1;
            Int32 Y1 = Y0 + 1;
            if ((X1 >= 0) && (X1 < Width) && (Y0 >= 0) && (Y0 < Height) && Valid[Y0, X1])
            {
                Acc += FieldArr[Y0, X1] * W10;
                WSum += W10;
            }

            if ((X0 >= 0) && (X0 < Width) && (Y1 >= 0) && (Y1 < Height) && Valid[Y1, X0])
            {
                Acc += FieldArr[Y1, X0] * W01;
                WSum += W01;
            }

            if ((X1 >= 0) && (X1 < Width) && (Y1 >= 0) && (Y1 < Height) && Valid[Y1, X1])
            {
                Acc += FieldArr[Y1, X1] * W11;
                WSum += W11;
            }

            if (WSum <= 1e-9)
            {
                return 0.0;
            }

            return Acc / WSum;
        }

        /// <summary>
        /// Bilinearly samples paired flow components at fractional grid coordinates.
        /// </summary>
        static public (Double Ux, Double Uy) SampleFlow
            (
            Double[,] FxS,
            Double[,] FyS,
            Boolean[,] Valid,
            Double X,
            Double Y
            )
        {
            // One corner loop for both fields (hot path in pass flow tubes).
            Int32 Height = FxS.GetLength(0);
            Int32 Width = FxS.GetLength(1);
            Int32 X0 = (Int32)Math.Floor(X);
            Int32 Y0 = (Int32)Math.Floor(Y);
            Double Fx = X - X0;
            Double Fy = Y - Y0;
            Double W00 = (1.0 - Fx) * (1.0 - Fy);
            Double W10 = Fx * (1.0 - Fy);
            Double W01 = (1.0 - Fx) * Fy;
            Double W11 = Fx * Fy;
            Double AccX = 0.0;
            Double AccY = 0.0;
            Double WSum = 0.0;
            Int32 X1 = X0 + 1;
            Int32 Y1 = Y0 + 1;
            if ((X0 >= 0) && (X0 < Width) && (Y0 >= 0) && (Y0 < Height) && Valid[Y0, X0])
            {
                AccX += FxS[Y0, X0] * W00;
                AccY += FyS[Y0, X0] * W00;
                WSum += W00;
            }

            if ((X1 >= 0) && (X1 < Width) && (Y0 >= 0) && (Y0 < Height) && Valid[Y0, X1])
            {
                AccX += FxS[Y0, X1] * W10;
                AccY += FyS[Y0, X1] * W10;
                WSum += W10;
            }

            if ((X0 >= 0) && (X0 < Width) && (Y1 >= 0) && (Y1 < Height) && Valid[Y1, X0])
            {
                AccX += FxS[Y1, X0] * W01;
                AccY += FyS[Y1, X0] * W01;
                WSum += W01;
            }

            if ((X1 >= 0) && (X1 < Width) && (Y1 >= 0) && (Y1 < Height) && Valid[Y1, X1])
            {
                AccX += FxS[Y1, X1] * W11;
                AccY += FyS[Y1, X1] * W11;
                WSum += W11;
            }

            if (WSum <= 1e-9)
            {
                return (0.0, 0.0);
            }

            return (AccX / WSum, AccY / WSum);
        }

        /// <summary>
        /// Returns smoothed flow unit vectors used for streamline integration.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Flow">Flow field.</param>
        /// <param name="SmoothPasses">Smoothing passes.</param>
        /// <returns>Smoothed unit vectors.</returns>
        static public (Double[,] FxS, Double[,] FyS) FlowDirectionField
            (
            Grid Grid,
            FlowField Flow,
            Int32 SmoothPasses = 12
            )
        {
            // Smooth potential first, then re-derive normalized downhill direction.
            Double[,] PhiS = SmoothPotential(Flow.Phi, Grid.Valid, SmoothPasses);
            return GradientFromPhi(PhiS, Grid.Valid);
        }

        /// <summary>
        /// Integrates flow from a fractional start until predicate hits.
        /// </summary>
        /// <param name="FxS">Smoothed flow x component.</param>
        /// <param name="FyS">Smoothed flow y component.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="X">Start fractional x.</param>
        /// <param name="Y">Start fractional y.</param>
        /// <param name="Predicate">Target predicate on integer cells.</param>
        /// <param name="Step">Integration step.</param>
        /// <param name="MaxSteps">Max steps.</param>
        /// <returns>First cell meeting predicate, or null.</returns>
        static public (Int32 Bx, Int32 By)? StreamlineTo
            (
            Double[,] FxS,
            Double[,] FyS,
            Boolean[,] Valid,
            Double X,
            Double Y,
            Func<Int32, Int32, Boolean> Predicate,
            Double Step = 0.5,
            Int32 MaxSteps = 4000
            )
        {
            // Walk downstream until target condition is met or flow leaves field.
            Int32 Height = Valid.GetLength(0);
            Int32 Width = Valid.GetLength(1);
            for (Int32 Iter = 0; Iter < MaxSteps; Iter += 1)
            {
                Int32 Bx = (Int32)Math.Floor(X);
                Int32 By = (Int32)Math.Floor(Y);
                if ((Bx < 0) || (Bx >= Width) || (By < 0) || (By >= Height) || !Valid[By, Bx])
                {
                    return null;
                }

                if (Predicate(Bx, By))
                {
                    return (Bx, By);
                }

                Double Ux = Bilinear(FxS, Valid, X, Y);
                Double Uy = Bilinear(FyS, Valid, X, Y);
                Double Norm = Math.Sqrt((Ux * Ux) + (Uy * Uy));
                if (Norm < 1e-6)
                {
                    return null;
                }

                X += Step * Ux / Norm;
                Y += Step * Uy / Norm;
            }

            return null;
        }

        /// <summary>
        /// Traces streamline strips from ridge seeds.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Flow">Flow field.</param>
        /// <param name="MaxSeeds">Maximum seed count.</param>
        /// <param name="Step">Integration step size.</param>
        /// <param name="MaxSteps">Max steps per strip.</param>
        /// <param name="SmoothPasses">Potential smoothing passes.</param>
        /// <returns>Traced strips.</returns>
        static public List<Strip> TraceStrips
            (
            Grid Grid,
            FlowField Flow,
            Int32 MaxSeeds = 4000,
            Double Step = 0.5,
            Int32 MaxSteps = 4000,
            Int32 SmoothPasses = 12
            )
        {
            // Use smoothed potential so traces follow global source-to-sink structure.
            Boolean[,] Valid = Grid.Valid;
            Double[,] PhiS = SmoothPotential(Flow.Phi, Valid, SmoothPasses);
            (Double[,] FxS, Double[,] FyS) = GradientFromPhi(PhiS, Valid);
            List<(Int32 Bx, Int32 By)> Seeds = FindRidges(Grid, PhiS, MaxSeeds);
            Int32 Height = Valid.GetLength(0);
            Int32 Width = Valid.GetLength(1);
            List<Strip> Out = new List<Strip>();
            foreach ((Int32 Bx, Int32 By) Seed in Seeds)
            {
                Double X = Seed.Bx + 0.5;
                Double Y = Seed.By + 0.5;
                List<(Double X, Double Y)> Path = new List<(Double X, Double Y)> { (X, Y) };
                List<(Int32 Bx, Int32 By)> Cells = new List<(Int32 Bx, Int32 By)>();
                HashSet<(Int32 Bx, Int32 By)> Visited = new HashSet<(Int32 Bx, Int32 By)>();
                (Int32 Bx, Int32 By)? LastCell = null;
                for (Int32 Iter = 0; Iter < MaxSteps; Iter += 1)
                {
                    Int32 Bx = (Int32)Math.Floor(X);
                    Int32 By = (Int32)Math.Floor(Y);
                    if ((Bx < 0) || (Bx >= Width) || (By < 0) || (By >= Height) || !Valid[By, Bx])
                    {
                        break;
                    }

                    (Int32 Bx, Int32 By) Cell = (Bx, By);
                    if ((LastCell == null) || (Cell != LastCell.Value))
                    {
                        if (Visited.Contains(Cell))
                        {
                            break;
                        }

                        Visited.Add(Cell);
                        Cells.Add(Cell);
                        LastCell = Cell;
                    }

                    Double Ux = Bilinear(FxS, Valid, X, Y);
                    Double Uy = Bilinear(FyS, Valid, X, Y);
                    Double Norm = Math.Sqrt((Ux * Ux) + (Uy * Uy));
                    if (Norm < 1e-6)
                    {
                        break;
                    }

                    X += Step * Ux / Norm;
                    Y += Step * Uy / Norm;
                    Path.Add((X, Y));
                }

                if (Cells.Count >= 1)
                {
                    Out.Add(new Strip(Cells, Path));
                }
            }

            return Out;
        }

        /// <summary>
        /// Smooths potential over valid mask.
        /// </summary>
        /// <param name="Phi">Input potential.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="Passes">Smoothing pass count.</param>
        /// <returns>Smoothed potential.</returns>
        static private Double[,] SmoothPotential
            (
            Double[,] Phi,
            Boolean[,] Valid,
            Int32 Passes
            )
        {
            // Mask-aware neighborhood averaging damps per-cell spikes.
            Int32 Height = Phi.GetLength(0);
            Int32 Width = Phi.GetLength(1);
            Double[,] Cur = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Cur[By, Bx] = Valid[By, Bx] ? (Double.IsNaN(Phi[By, Bx]) ? 0.0 : Phi[By, Bx]) : 0.0;
                }
            }

            (Int32 Dj, Int32 Di)[] Neigh = new (Int32 Dj, Int32 Di)[] { (-1, 0), (1, 0), (0, -1), (0, 1) };
            for (Int32 Pass = 0; Pass < Math.Max(0, Passes); Pass += 1)
            {
                Double[,] Next = new Double[Height, Width];
                for (Int32 By = 0; By < Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Width; Bx += 1)
                    {
                        if (!Valid[By, Bx])
                        {
                            Next[By, Bx] = 0.0;
                            continue;
                        }

                        Double Sum = Cur[By, Bx];
                        Double Den = 1.0;
                        foreach ((Int32 Dj, Int32 Di) Offset in Neigh)
                        {
                            Int32 Ny = By + Offset.Dj;
                            Int32 Nx = Bx + Offset.Di;
                            if ((Ny >= 0) && (Ny < Height) && (Nx >= 0) && (Nx < Width) && Valid[Ny, Nx])
                            {
                                Sum += Cur[Ny, Nx];
                                Den += 1.0;
                            }
                        }

                        Next[By, Bx] = Sum / Math.Max(Den, 1e-9);
                    }
                }

                Cur = Next;
            }

            Double[,] Out = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Out[By, Bx] = Valid[By, Bx] ? Cur[By, Bx] : Double.NaN;
                }
            }

            return Out;
        }

        /// <summary>
        /// Computes downhill unit vectors from potential.
        /// </summary>
        /// <param name="Phi">Potential matrix.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <returns>Unit flow vectors.</returns>
        static private (Double[,] Fx, Double[,] Fy) GradientFromPhi
            (
            Double[,] Phi,
            Boolean[,] Valid
            )
        {
            // Reuse masked gradient implementation from flow solver.
            (Double[,] Gx, Double[,] Gy) = FlowFieldSolver.MaskedGradient(Phi, Valid, 1.0);
            Int32 Height = Valid.GetLength(0);
            Int32 Width = Valid.GetLength(1);
            Double[,] Fx = new Double[Height, Width];
            Double[,] Fy = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Valid[By, Bx])
                    {
                        continue;
                    }

                    Double RawX = -Gx[By, Bx];
                    Double RawY = -Gy[By, Bx];
                    Double Mag = Math.Sqrt((RawX * RawX) + (RawY * RawY));
                    if (Mag > 1e-12)
                    {
                        Fx[By, Bx] = RawX / Mag;
                        Fy[By, Bx] = RawY / Mag;
                    }
                }
            }

            return (Fx, Fy);
        }

        /// <summary>
        /// Finds seed cells ordered by ridge strength.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Phi">Smoothed potential.</param>
        /// <param name="MaxSeeds">Maximum seed count.</param>
        /// <returns>Seed cells.</returns>
        static private List<(Int32 Bx, Int32 By)> FindRidges
            (
            Grid Grid,
            Double[,] Phi,
            Int32 MaxSeeds
            )
        {
            // Collect local maxima first, then remaining cut cells by depth.
            List<(Double Delta, Int32 Bx, Int32 By)> Seeds = new List<(Double Delta, Int32 Bx, Int32 By)>();
            Int32 Height = Grid.Height;
            Int32 Width = Grid.Width;
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double Delta = Grid.Existing[By, Bx] - Grid.Target[By, Bx];
                    if (Delta <= 0.001)
                    {
                        continue;
                    }

                    Boolean IsMax = true;
                    Double Here = Phi[By, Bx];
                    for (Int32 Dy = -1; Dy <= 1; Dy += 1)
                    {
                        for (Int32 Dx = -1; Dx <= 1; Dx += 1)
                        {
                            if ((Dx == 0) && (Dy == 0))
                            {
                                continue;
                            }

                            Int32 Ny = By + Dy;
                            Int32 Nx = Bx + Dx;
                            if ((Ny >= 0) && (Ny < Height) && (Nx >= 0) && (Nx < Width) && Grid.Valid[Ny, Nx])
                            {
                                if (Phi[Ny, Nx] > (Here + 1e-12))
                                {
                                    IsMax = false;
                                }
                            }
                        }
                    }

                    if (IsMax)
                    {
                        Seeds.Add((Delta, Bx, By));
                    }
                }
            }

            Seeds.Sort((A, B) => B.Delta.CompareTo(A.Delta));
            List<(Int32 Bx, Int32 By)> SeedCells = new List<(Int32 Bx, Int32 By)>();
            HashSet<(Int32 Bx, Int32 By)> Seen = new HashSet<(Int32 Bx, Int32 By)>();
            foreach ((Double Delta, Int32 Bx, Int32 By) Seed in Seeds)
            {
                SeedCells.Add((Seed.Bx, Seed.By));
                Seen.Add((Seed.Bx, Seed.By));
            }

            List<(Double Delta, Int32 Bx, Int32 By)> Others = new List<(Double Delta, Int32 Bx, Int32 By)>();
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double Delta = Grid.Existing[By, Bx] - Grid.Target[By, Bx];
                    if ((Delta > 0.001) && !Seen.Contains((Bx, By)))
                    {
                        Others.Add((Delta, Bx, By));
                    }
                }
            }

            Others.Sort((A, B) => B.Delta.CompareTo(A.Delta));
            foreach ((Double Delta, Int32 Bx, Int32 By) Seed in Others)
            {
                SeedCells.Add((Seed.Bx, Seed.By));
            }

            if (SeedCells.Count > MaxSeeds)
            {
                SeedCells = SeedCells.GetRange(0, MaxSeeds);
            }

            return SeedCells;
        }
    }
}
