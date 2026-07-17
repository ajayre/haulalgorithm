using System;

namespace HaulBuilder
{
    /// <summary>
    /// Global cut/fill balance computation and optional vertical target adjustment.
    /// </summary>
    static public class Balance
    {
        /// <summary>
        /// Relative imbalance threshold (2%).
        /// </summary>
        public const Double DEFAULT_IMBALANCE_THRESHOLD = 0.02;

        /// <summary>
        /// Computes balance and optionally adjusts the target surface vertically.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Mode">Mode: check or adjust.</param>
        /// <param name="Threshold">Warning threshold.</param>
        /// <returns>Balance summary.</returns>
        static public BalanceResult Compute
            (
            Grid Grid,
            String Mode = "check",
            Double Threshold = DEFAULT_IMBALANCE_THRESHOLD
            )
        {
            // Start from current totals before any adjustment.
            (Double Supply, Double Demand) = Totals(Grid);
            Double AppliedOffset = 0.0;

            if (Mode == "adjust")
            {
                AppliedOffset = SolveOffset(Grid);
                if (AppliedOffset != 0.0)
                {
                    // Apply one global vertical shift only on valid cells.
                    for (Int32 By = 0; By < Grid.Height; By += 1)
                    {
                        for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                        {
                            if (Grid.Valid[By, Bx])
                            {
                                Grid.Target[By, Bx] += AppliedOffset;
                            }
                        }
                    }
                }

                (Supply, Demand) = Totals(Grid);
                Console.Error.WriteLine($"INFO: balance adjust applied vertical offset {AppliedOffset:+0.0000;-0.0000;+0.0000} m");
            }

            Double Relative = RelativeImbalance(Supply, Demand);
            if (Relative > Threshold)
            {
                // Keep warning message parity with Python output.
                Console.Error.WriteLine(
                    $"WARNING: cut/fill imbalance {Relative * 100.0:0.0}% exceeds {Threshold * 100.0:0.0}% " +
                    $"(supply={Supply:0.0} BCY, demand={Demand:0.0} BCY)");
            }

            return new BalanceResult(Supply, Demand, Relative, AppliedOffset, Mode);
        }

        /// <summary>
        /// Computes total supply and demand BCY for a grid.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <returns>Total supply and demand.</returns>
        static public (Double Supply, Double Demand) Totals
            (
            Grid Grid
            )
        {
            // Sum full matrices to preserve the same BCY totals as Python.
            Double[,] SupplyGrid = Grid.SupplyGrid();
            Double[,] DemandGrid = Grid.DemandGrid();
            Double Supply = 0.0;
            Double Demand = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    Supply += SupplyGrid[By, Bx];
                    Demand += DemandGrid[By, Bx];
                }
            }

            return (Supply, Demand);
        }

        /// <summary>
        /// Computes relative imbalance metric.
        /// </summary>
        /// <param name="Supply">Total supply BCY.</param>
        /// <param name="Demand">Total demand BCY.</param>
        /// <returns>Relative imbalance [0..1+].</returns>
        static public Double RelativeImbalance
            (
            Double Supply,
            Double Demand
            )
        {
            // Use max(supply,demand) denominator with epsilon floor.
            Double Denom = Math.Max(Math.Max(Supply, Demand), Constants.NUMERIC_COMPARISON_TOLERANCE);
            return Math.Abs(Supply - Demand) / Denom;
        }

        /// <summary>
        /// Solves global vertical offset by bisection.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <returns>Offset in metres.</returns>
        static public Double SolveOffset
            (
            Grid Grid
            )
        {
            // Snapshot current delta so candidate offsets can be evaluated quickly.
            Double[,] Delta = new Double[Grid.Height, Grid.Width];
            Double MinDelta = Double.PositiveInfinity;
            Double MaxDelta = Double.NegativeInfinity;
            Int32 ValidCount = 0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double Value = Grid.Existing[By, Bx] - Grid.Target[By, Bx];
                    Delta[By, Bx] = Value;
                    MinDelta = Math.Min(MinDelta, Value);
                    MaxDelta = Math.Max(MaxDelta, Value);
                    ValidCount += 1;
                }
            }

            if (ValidCount == 0)
            {
                return 0.0;
            }

            Double Lo = MinDelta - 1.0;
            Double Hi = MaxDelta + 1.0;
            Double FLo = ImbalanceAt(Grid, Delta, Lo);
            Double FHi = ImbalanceAt(Grid, Delta, Hi);
            if (FLo == 0.0) return Lo;
            if (FHi == 0.0) return Hi;
            if ((FLo > 0.0) == (FHi > 0.0))
            {
                return 0.0;
            }

            for (Int32 Iter = 0; Iter < 100; Iter += 1)
            {
                // Bisection shrinks bracket until root tolerance is met.
                Double Mid = Constants.HALF_SCALE * (Lo + Hi);
                Double FMid = ImbalanceAt(Grid, Delta, Mid);
                if ((Math.Abs(FMid) < Constants.GEOMETRY_COMPARISON_TOLERANCE) || ((Hi - Lo) < Constants.NUMERIC_COMPARISON_TOLERANCE))
                {
                    return Mid;
                }

                if ((FMid > 0.0) == (FLo > 0.0))
                {
                    Lo = Mid;
                    FLo = FMid;
                }
                else
                {
                    Hi = Mid;
                    FHi = FMid;
                }
            }

            return Constants.HALF_SCALE * (Lo + Hi);
        }

        /// <summary>
        /// Computes supply-demand difference for a candidate offset.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Delta">Precomputed base delta matrix.</param>
        /// <param name="Offset">Candidate target offset.</param>
        /// <returns>Supply BCY minus demand BCY.</returns>
        static private Double ImbalanceAt
            (
            Grid Grid,
            Double[,] Delta,
            Double Offset
            )
        {
            // Raising target lowers delta by the same offset.
            Double Supply = 0.0;
            Double Demand = 0.0;
            for (Int32 By = 0; By < Grid.Height; By += 1)
            {
                for (Int32 Bx = 0; Bx < Grid.Width; Bx += 1)
                {
                    if (!Grid.Valid[By, Bx])
                    {
                        continue;
                    }

                    Double Value = Delta[By, Bx] - Offset;
                    if (Value > 0.0)
                    {
                        Supply += Value;
                    }
                    else
                    {
                        Demand += -Value;
                    }
                }
            }

            Double SupplyBcy = Supply * Grid.AreaM2 * Constants.M3_TO_CY;
            Double DemandBcy = Demand * Grid.AreaM2 * Constants.M3_TO_CY * Grid.Compaction;
            return SupplyBcy - DemandBcy;
        }
    }

    /// <summary>
    /// Result payload for global cut/fill balance.
    /// </summary>
    public sealed class BalanceResult
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="BalanceResult"/> class.
        /// </summary>
        /// <param name="TotalSupplyBcy">Total supply BCY.</param>
        /// <param name="TotalDemandBcy">Total demand BCY.</param>
        /// <param name="RelativeImbalance">Relative imbalance ratio.</param>
        /// <param name="AppliedOffsetM">Applied vertical offset.</param>
        /// <param name="Mode">Balance mode.</param>
        public BalanceResult
            (
            Double TotalSupplyBcy,
            Double TotalDemandBcy,
            Double RelativeImbalance,
            Double AppliedOffsetM,
            String Mode
            )
        {
            // Persist totals and adjustment metadata for reporting.
            this.TotalSupplyBcy = TotalSupplyBcy;
            this.TotalDemandBcy = TotalDemandBcy;
            this.RelativeImbalance = RelativeImbalance;
            this.AppliedOffsetM = AppliedOffsetM;
            this.Mode = Mode;
        }

        /// <summary>
        /// Gets total supply in BCY.
        /// </summary>
        public Double TotalSupplyBcy { get; }

        /// <summary>
        /// Gets total demand in BCY.
        /// </summary>
        public Double TotalDemandBcy { get; }

        /// <summary>
        /// Gets relative imbalance ratio.
        /// </summary>
        public Double RelativeImbalance { get; }

        /// <summary>
        /// Gets applied vertical offset in metres.
        /// </summary>
        public Double AppliedOffsetM { get; }

        /// <summary>
        /// Gets balance mode.
        /// </summary>
        public String Mode { get; }
    }
}
