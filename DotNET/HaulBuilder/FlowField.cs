using System;
using System.Collections.Generic;

using CSparse;
using CSparse.Double.Factorization;
using CSparse.Storage;

namespace HaulBuilder
{
    /// <summary>
    /// Per-cell potential and flow vectors.
    /// </summary>
    public sealed class FlowField
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="FlowField"/> class.
        /// </summary>
        /// <param name="Phi">Potential matrix.</param>
        /// <param name="Fx">Unit east flow component.</param>
        /// <param name="Fy">Unit north flow component.</param>
        /// <param name="Magnitude">Raw flow magnitude.</param>
        /// <param name="HasArrow">Arrow mask.</param>
        /// <param name="Mode">Computation mode.</param>
        public FlowField
            (
            Double[,] Phi,
            Double[,] Fx,
            Double[,] Fy,
            Double[,] Magnitude,
            Boolean[,] HasArrow,
            String Mode
            )
        {
            // Keep all flow products together for downstream strip tracing.
            this.Phi = Phi;
            this.Fx = Fx;
            this.Fy = Fy;
            this.Magnitude = Magnitude;
            this.HasArrow = HasArrow;
            this.Mode = Mode;
        }

        /// <summary>
        /// Gets potential matrix.
        /// </summary>
        public Double[,] Phi { get; }

        /// <summary>
        /// Gets unit east flow component.
        /// </summary>
        public Double[,] Fx { get; }

        /// <summary>
        /// Gets unit north flow component.
        /// </summary>
        public Double[,] Fy { get; }

        /// <summary>
        /// Gets raw flow magnitude.
        /// </summary>
        public Double[,] Magnitude { get; }

        /// <summary>
        /// Gets arrow visibility mask.
        /// </summary>
        public Boolean[,] HasArrow { get; }

        /// <summary>
        /// Gets solver mode.
        /// </summary>
        public String Mode { get; }

        /// <summary>
        /// Returns compass heading of local flow vector.
        /// </summary>
        /// <param name="Bx">Cell x index.</param>
        /// <param name="By">Cell y index.</param>
        /// <returns>Heading degrees where 0 is north and 90 is east.</returns>
        public Double DirectionDeg
            (
            Int32 Bx,
            Int32 By
            )
        {
            // Match Python heading convention atan2(fx, fy).
            return (Math.Atan2(Fx[By, Bx], Fy[By, Bx]) * 180.0 / Math.PI + 360.0) % 360.0;
        }
    }

    /// <summary>
    /// Flow-field solvers (Poisson default, mincostflow fallback behavior).
    /// </summary>
    public static class FlowFieldSolver
    {
        /// <summary>
        /// Computes a flow field from the grid in selected mode.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Mode">poisson or mincostflow.</param>
        /// <param name="ArrowEpsFrac">Arrow threshold fraction.</param>
        /// <returns>Computed flow field.</returns>
        static public FlowField Compute
            (
            Grid Grid,
            String Mode = "poisson",
            Double ArrowEpsFrac = 0.02
            )
        {
            // Keep API parity: mincostflow currently falls back to Poisson.
            if (Mode == "mincostflow")
            {
                Console.Error.WriteLine("WARNING: transport_mode=mincostflow unavailable in C# port; falling back to poisson.");
            }

            return PoissonField(Grid, ArrowEpsFrac);
        }

        /// <summary>
        /// Computes Poisson-based smooth transport flow.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="ArrowEpsFrac">Arrow threshold fraction.</param>
        /// <returns>Flow field.</returns>
        static public FlowField PoissonField
            (
            Grid Grid,
            Double ArrowEpsFrac
            )
        {
            // Build signed source: supply minus demand over valid cells.
            Double[,] Supply = Grid.SupplyGrid();
            Double[,] Demand = Grid.DemandGrid();
            Double[,] Source = new Double[Grid.Height, Grid.Width];
            Double Sum = 0.0;
            Int32 ValidCount = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        Source[By, Bx] = 0.0;
                        continue;
                    }

                    Double Value = Supply[By, Bx] - Demand[By, Bx];
                    Source[By, Bx] = Value;
                    Sum += Value;
                    ValidCount += 1;
                }
            }

            if (ValidCount == 0)
            {
                throw new InvalidOperationException("No valid cells for flow field.");
            }

            Double Mean = Sum / ValidCount;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx])
                    {
                        Source[By, Bx] -= Mean;
                    }
                }
            }

            Double[,] Phi = SolvePoissonNeumann(Grid.Valid, Source, Grid.GridSizeM);
            (Double[,] Fx, Double[,] Fy, Double[,] Mag) = GradientFlow(Phi, Grid.Valid, Grid.GridSizeM);
            Boolean[,] Arrows = ArrowMask(Grid, Mag, ArrowEpsFrac);
            return new FlowField(Phi, Fx, Fy, Mag, Arrows, "poisson");
        }

        /// <summary>
        /// Solves masked Poisson equation with Neumann boundaries (SciPy sparse direct solver parity).
        /// </summary>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="Source">Source matrix.</param>
        /// <param name="CellSize">Grid cell size.</param>
        /// <returns>Potential matrix.</returns>
        static public Double[,] SolvePoissonNeumann
            (
            Boolean[,] Valid,
            Double[,] Source,
            Double CellSize
            )
        {
            Int32 Height = Valid.GetLength(0);
            Int32 Width = Valid.GetLength(1);
            List<(Int32 By, Int32 Bx)> Coords = BuildValidCoords(Valid);
            if (Coords.Count <= 0)
            {
                throw new InvalidOperationException("No valid cells for Poisson solve.");
            }

            Double[]? Sol = TrySolvePoissonSparse(Valid, Source, CellSize, Coords);
            if (Sol is null)
            {
                (Sol, _) = SolvePoissonJacobi(Valid, Source, CellSize, 20000);
            }

            Double[,] Phi = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    Phi[By, Bx] = Double.NaN;
                }
            }

            for (Int32 Index = 0; Index < Coords.Count; Index += 1)
            {
                Phi[Coords[Index].By, Coords[Index].Bx] = Sol[Index];
            }

            return Phi;
        }

        /// <summary>
        /// Builds graph-Laplacian Poisson system and solves with CSparse LU (Python SciPy path).
        /// </summary>
        static private Double[]? TrySolvePoissonSparse
            (
            Boolean[,] Valid,
            Double[,] Source,
            Double CellSize,
            List<(Int32 By, Int32 Bx)> Coords
            )
        {
            // Mirror landlevel.flowfield._solve_poisson_neumann sparse assembly.
            try
            {
                Int32 Height = Valid.GetLength(0);
                Int32 Width = Valid.GetLength(1);
                Int32 N = Coords.Count;
                Int32[,] Idx = new Int32[Height, Width];
                for (Int32 By = 0; By < Height; By += 1)
                {
                    for (Int32 Bx = 0; Bx < Width; Bx += 1)
                    {
                        Idx[By, Bx] = -1;
                    }
                }

                for (Int32 K = 0; K < N; K += 1)
                {
                    Idx[Coords[K].By, Coords[K].Bx] = K;
                }

                Int32 EstNnz = N * 5;
                CoordinateStorage<Double> Coo = new CoordinateStorage<Double>(N, N, EstNnz);
                Double[] Rhs = new Double[N];
                Double H2 = CellSize * CellSize;
                (Int32 Dj, Int32 Di)[] Neigh = new (Int32 Dj, Int32 Di)[] { (-1, 0), (1, 0), (0, -1), (0, 1) };
                for (Int32 K = 0; K < N; K += 1)
                {
                    if (K == 0)
                    {
                        Coo.At(0, 0, 1.0);
                        Rhs[0] = 0.0;
                        continue;
                    }

                    (Int32 By, Int32 Bx) = Coords[K];
                    Int32 Deg = 0;
                    foreach ((Int32 Dj, Int32 Di) Offset in Neigh)
                    {
                        Int32 Ny = By + Offset.Dj;
                        Int32 Nx = Bx + Offset.Di;
                        if ((Ny >= 0) && (Ny < Height) && (Nx >= 0) && (Nx < Width) && Valid[Ny, Nx])
                        {
                            Deg += 1;
                            Coo.At(K, Idx[Ny, Nx], -1.0);
                        }
                    }

                    Coo.At(K, K, Deg);
                    Rhs[K] = Source[By, Bx] * H2;
                }

                CompressedColumnStorage<Double> Matrix = CompressedColumnStorage<Double>.OfIndexed(Coo, true);
                SparseLU Lu = SparseLU.Create(Matrix, ColumnOrdering.MinimumDegreeAtPlusA, 1.0);
                Double[] Sol = new Double[N];
                Lu.Solve(Rhs, Sol);
                return Sol;
            }
            catch (Exception)
            {
                return null;
            }
        }

        static private List<(Int32 By, Int32 Bx)> BuildValidCoords
            (
            Boolean[,] Valid
            )
        {
            Int32 Height = Valid.GetLength(0);
            Int32 Width = Valid.GetLength(1);
            List<(Int32 By, Int32 Bx)> Coords = new List<(Int32 By, Int32 Bx)>();
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (Valid[By, Bx])
                    {
                        Coords.Add((By, Bx));
                    }
                }
            }

            return Coords;
        }

        /// <summary>
        /// Jacobi iterative Poisson solver over valid cells.
        /// </summary>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="Source">Source matrix.</param>
        /// <param name="CellSize">Cell size.</param>
        /// <param name="Iters">Maximum iterations.</param>
        /// <returns>Flattened solution and coordinate mapping.</returns>
        static public (Double[] Sol, List<(Int32 By, Int32 Bx)> Coords) SolvePoissonJacobi
            (
            Boolean[,] Valid,
            Double[,] Source,
            Double CellSize,
            Int32 Iters
            )
        {
            // Iterate until convergence threshold like the Python Jacobi fallback.
            Int32 Height = Valid.GetLength(0);
            Int32 Width = Valid.GetLength(1);
            List<(Int32 By, Int32 Bx)> Coords = BuildValidCoords(Valid);

            Double[,] Phi = new Double[Height, Width];
            Double H2 = CellSize * CellSize;
            (Int32 Dj, Int32 Di)[] Neigh = new (Int32 Dj, Int32 Di)[] { (-1, 0), (1, 0), (0, -1), (0, 1) };
            for (Int32 Iter = 0; Iter < Iters; Iter += 1)
            {
                Double[,] Next = (Double[,])Phi.Clone();
                Double MaxDelta = 0.0;
                foreach ((Int32 By, Int32 Bx) Coord in Coords)
                {
                    Double Acc = 0.0;
                    Int32 Deg = 0;
                    foreach ((Int32 Dj, Int32 Di) Offset in Neigh)
                    {
                        Int32 Ny = Coord.By + Offset.Dj;
                        Int32 Nx = Coord.Bx + Offset.Di;
                        if ((Ny >= 0) && (Ny < Height) && (Nx >= 0) && (Nx < Width) && Valid[Ny, Nx])
                        {
                            Acc += Phi[Ny, Nx];
                            Deg += 1;
                        }
                    }

                    if (Deg == 0)
                    {
                        continue;
                    }

                    Double Value = (Acc + (Source[Coord.By, Coord.Bx] * H2)) / Deg;
                    MaxDelta = Math.Max(MaxDelta, Math.Abs(Value - Phi[Coord.By, Coord.Bx]));
                    Next[Coord.By, Coord.Bx] = Value;
                }

                Phi = Next;
                if (MaxDelta < 1e-7)
                {
                    break;
                }
            }

            Double[] Sol = new Double[Coords.Count];
            for (Int32 Index = 0; Index < Coords.Count; Index += 1)
            {
                Sol[Index] = Phi[Coords[Index].By, Coords[Index].Bx];
            }

            return (Sol, Coords);
        }

        /// <summary>
        /// Computes one-sided or central masked gradient.
        /// </summary>
        /// <param name="Phi">Potential matrix.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="H">Grid spacing.</param>
        /// <returns>East and north derivatives.</returns>
        static public (Double[,] Gx, Double[,] Gy) MaskedGradient
            (
            Double[,] Phi,
            Boolean[,] Valid,
            Double H = 1.0
            )
        {
            // Match Python edge handling with central and one-sided differences.
            Int32 Height = Valid.GetLength(0);
            Int32 Width = Valid.GetLength(1);
            Double[,] Gx = new Double[Height, Width];
            Double[,] Gy = new Double[Height, Width];
            for (Int32 By = 0; By < Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Width; Bx += 1)
                {
                    if (!Valid[By, Bx])
                    {
                        Gx[By, Bx] = 0.0;
                        Gy[By, Bx] = 0.0;
                        continue;
                    }

                    Boolean PlusX = (Bx + 1 < Width) && Valid[By, Bx + 1];
                    Boolean MinusX = (Bx - 1 >= 0) && Valid[By, Bx - 1];
                    if (PlusX && MinusX)
                    {
                        Gx[By, Bx] = (Phi[By, Bx + 1] - Phi[By, Bx - 1]) / (2.0 * H);
                    }
                    else if (PlusX)
                    {
                        Gx[By, Bx] = (Phi[By, Bx + 1] - Phi[By, Bx]) / H;
                    }
                    else if (MinusX)
                    {
                        Gx[By, Bx] = (Phi[By, Bx] - Phi[By, Bx - 1]) / H;
                    }

                    Boolean PlusY = (By + 1 < Height) && Valid[By + 1, Bx];
                    Boolean MinusY = (By - 1 >= 0) && Valid[By - 1, Bx];
                    if (PlusY && MinusY)
                    {
                        Gy[By, Bx] = (Phi[By + 1, Bx] - Phi[By - 1, Bx]) / (2.0 * H);
                    }
                    else if (PlusY)
                    {
                        Gy[By, Bx] = (Phi[By + 1, Bx] - Phi[By, Bx]) / H;
                    }
                    else if (MinusY)
                    {
                        Gy[By, Bx] = (Phi[By, Bx] - Phi[By - 1, Bx]) / H;
                    }
                }
            }

            return (Gx, Gy);
        }

        /// <summary>
        /// Computes unit flow vectors and magnitude from potential.
        /// </summary>
        /// <param name="Phi">Potential matrix.</param>
        /// <param name="Valid">Valid mask.</param>
        /// <param name="H">Grid spacing.</param>
        /// <returns>Unit flow and magnitude matrices.</returns>
        static public (Double[,] Fx, Double[,] Fy, Double[,] Mag) GradientFlow
            (
            Double[,] Phi,
            Boolean[,] Valid,
            Double H
            )
        {
            // Flow is negative gradient of potential.
            (Double[,] Gx, Double[,] Gy) = MaskedGradient(Phi, Valid, H);
            Int32 Height = Valid.GetLength(0);
            Int32 Width = Valid.GetLength(1);
            Double[,] Fx = new Double[Height, Width];
            Double[,] Fy = new Double[Height, Width];
            Double[,] Mag = new Double[Height, Width];
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
                    Double Magnitude = Math.Sqrt((RawX * RawX) + (RawY * RawY));
                    Mag[By, Bx] = Magnitude;
                    if (Magnitude > 1e-12)
                    {
                        Fx[By, Bx] = RawX / Magnitude;
                        Fy[By, Bx] = RawY / Magnitude;
                    }
                    else
                    {
                        Fx[By, Bx] = 0.0;
                        Fy[By, Bx] = 0.0;
                    }
                }
            }

            return (Fx, Fy, Mag);
        }

        /// <summary>
        /// Computes arrow visibility mask.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Magnitude">Flow magnitude matrix.</param>
        /// <param name="Frac">Threshold fraction of max magnitude.</param>
        /// <returns>Arrow mask.</returns>
        static public Boolean[,] ArrowMask
            (
            Grid Grid,
            Double[,] Magnitude,
            Double Frac
            )
        {
            // Show arrows where flow and measurable cut/fill both exist.
            Double MaxMag = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (Grid.Valid[By, Bx])
                    {
                        MaxMag = Math.Max(MaxMag, Magnitude[By, Bx]);
                    }
                }
            }

            Double Threshold = (MaxMag > 0.0) ? (Frac * MaxMag) : 0.0;
            Boolean[,] Out = new Boolean[Grid.Height, Grid.Width];
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        Out[By, Bx] = false;
                        continue;
                    }

                    Double Delta = Math.Abs(Grid.Existing[By, Bx] - Grid.Target[By, Bx]);
                    Out[By, Bx] = (Magnitude[By, Bx] > Threshold) && (Delta > 0.001);
                }
            }

            return Out;
        }
    }
}
