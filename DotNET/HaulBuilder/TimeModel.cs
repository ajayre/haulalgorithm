using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Input parameters for job-time estimation.
    /// </summary>
    public sealed class TimeParams
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="TimeParams"/> class.
        /// </summary>
        /// <param name="LoadingSpeedMph">Loading speed.</param>
        /// <param name="HaulSpeedMph">Max haul speed.</param>
        /// <param name="DumpingSpeedMph">Dumping speed.</param>
        /// <param name="ReturnSpeedMph">Max return speed.</param>
        /// <param name="AccelTimeS">Acceleration time.</param>
        /// <param name="DecelTimeS">Deceleration time.</param>
        /// <param name="FieldEfficiency">Field efficiency fraction.</param>
        public TimeParams
            (
            Double LoadingSpeedMph,
            Double HaulSpeedMph,
            Double DumpingSpeedMph,
            Double ReturnSpeedMph,
            Double AccelTimeS,
            Double DecelTimeS,
            Double FieldEfficiency = 0.8
            )
        {
            // Preserve all machine speed and timing parameters.
            this.LoadingSpeedMph = LoadingSpeedMph;
            this.HaulSpeedMph = HaulSpeedMph;
            this.DumpingSpeedMph = DumpingSpeedMph;
            this.ReturnSpeedMph = ReturnSpeedMph;
            this.AccelTimeS = AccelTimeS;
            this.DecelTimeS = DecelTimeS;
            this.FieldEfficiency = FieldEfficiency;
        }

        /// <summary>
        /// Gets loading speed mph.
        /// </summary>
        public Double LoadingSpeedMph { get; }

        /// <summary>
        /// Gets haul speed mph.
        /// </summary>
        public Double HaulSpeedMph { get; }

        /// <summary>
        /// Gets dumping speed mph.
        /// </summary>
        public Double DumpingSpeedMph { get; }

        /// <summary>
        /// Gets return speed mph.
        /// </summary>
        public Double ReturnSpeedMph { get; }

        /// <summary>
        /// Gets acceleration time seconds.
        /// </summary>
        public Double AccelTimeS { get; }

        /// <summary>
        /// Gets deceleration time seconds.
        /// </summary>
        public Double DecelTimeS { get; }

        /// <summary>
        /// Gets field efficiency fraction.
        /// </summary>
        public Double FieldEfficiency { get; }
    }

    /// <summary>
    /// Time-model output for a routed pass plan.
    /// </summary>
    public sealed class TimeResult
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="TimeResult"/> class.
        /// </summary>
        /// <param name="TotalTimeS">Total job time.</param>
        /// <param name="LoadingDistM">Loading distance.</param>
        /// <param name="HaulingDistM">Hauling distance.</param>
        /// <param name="DumpingDistM">Dumping distance.</param>
        /// <param name="ReturningDistM">Returning distance.</param>
        /// <param name="AvgHaulSpeedMph">Average haul speed.</param>
        /// <param name="AvgReturnSpeedMph">Average return speed.</param>
        /// <param name="LoadingTimeS">Loading time.</param>
        /// <param name="HaulingTimeS">Hauling time.</param>
        /// <param name="DumpingTimeS">Dumping time.</param>
        /// <param name="ReturningTimeS">Returning time.</param>
        /// <param name="OtherTimeS">Other/nonproductive time.</param>
        public TimeResult
            (
            Double TotalTimeS,
            Double LoadingDistM,
            Double HaulingDistM,
            Double DumpingDistM,
            Double ReturningDistM,
            Double AvgHaulSpeedMph,
            Double AvgReturnSpeedMph,
            Double LoadingTimeS,
            Double HaulingTimeS,
            Double DumpingTimeS,
            Double ReturningTimeS,
            Double OtherTimeS
            )
        {
            // Store full distance/time breakdown.
            this.TotalTimeS = TotalTimeS;
            this.LoadingDistM = LoadingDistM;
            this.HaulingDistM = HaulingDistM;
            this.DumpingDistM = DumpingDistM;
            this.ReturningDistM = ReturningDistM;
            this.AvgHaulSpeedMph = AvgHaulSpeedMph;
            this.AvgReturnSpeedMph = AvgReturnSpeedMph;
            this.LoadingTimeS = LoadingTimeS;
            this.HaulingTimeS = HaulingTimeS;
            this.DumpingTimeS = DumpingTimeS;
            this.ReturningTimeS = ReturningTimeS;
            this.OtherTimeS = OtherTimeS;
        }

        /// <summary>
        /// Gets total time seconds.
        /// </summary>
        public Double TotalTimeS { get; }

        /// <summary>
        /// Gets loading distance metres.
        /// </summary>
        public Double LoadingDistM { get; }

        /// <summary>
        /// Gets hauling distance metres.
        /// </summary>
        public Double HaulingDistM { get; }

        /// <summary>
        /// Gets dumping distance metres.
        /// </summary>
        public Double DumpingDistM { get; }

        /// <summary>
        /// Gets returning distance metres.
        /// </summary>
        public Double ReturningDistM { get; }

        /// <summary>
        /// Gets average haul speed mph.
        /// </summary>
        public Double AvgHaulSpeedMph { get; }

        /// <summary>
        /// Gets average return speed mph.
        /// </summary>
        public Double AvgReturnSpeedMph { get; }

        /// <summary>
        /// Gets loading time seconds.
        /// </summary>
        public Double LoadingTimeS { get; }

        /// <summary>
        /// Gets hauling time seconds.
        /// </summary>
        public Double HaulingTimeS { get; }

        /// <summary>
        /// Gets dumping time seconds.
        /// </summary>
        public Double DumpingTimeS { get; }

        /// <summary>
        /// Gets returning time seconds.
        /// </summary>
        public Double ReturningTimeS { get; }

        /// <summary>
        /// Gets other time seconds.
        /// </summary>
        public Double OtherTimeS { get; }

        /// <summary>
        /// Gets total haul distance alias.
        /// </summary>
        public Double TotalHaulDistanceM => HaulingDistM;

        /// <summary>
        /// Gets total deadhead alias.
        /// </summary>
        public Double TotalDeadheadM => ReturningDistM;
    }

    /// <summary>
    /// Time estimation model.
    /// </summary>
    public static class TimeModel
    {
        /// <summary>
        /// Estimates total leveling time for a routed plan.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Route">Routed passes.</param>
        /// <param name="Params">Time-model parameters.</param>
        /// <returns>Time breakdown.</returns>
        static public TimeResult EstimateTime
            (
            Grid Grid,
            Route Route,
            TimeParams Params
            )
        {
            // Convert configured mph speeds once.
            Double LoadV = Params.LoadingSpeedMph * Constants.MPH_TO_MPS;
            Double HaulVMax = Params.HaulSpeedMph * Constants.MPH_TO_MPS;
            Double DumpV = Params.DumpingSpeedMph * Constants.MPH_TO_MPS;
            Double ReturnVMax = Params.ReturnSpeedMph * Constants.MPH_TO_MPS;
            Double Ta = Params.AccelTimeS;
            Double Td = Params.DecelTimeS;

            Double DLoad = 0.0;
            Double DHaul = 0.0;
            Double DDump = 0.0;
            Double TLoad = 0.0;
            Double THaul = 0.0;
            Double TDump = 0.0;
            Double TReturn = 0.0;
            foreach (Pass P in Route.OrderedPasses)
            {
                List<(Double X, Double Y)> CutXy = new List<(Double X, Double Y)>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in P.CutCells)
                {
                    CutXy.Add(Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By));
                }

                List<(Double X, Double Y)> FillXy = new List<(Double X, Double Y)>();
                foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in P.FillCells)
                {
                    FillXy.Add(Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By));
                }

                Double CutLen = SegmentLength(CutXy);
                Double FillLen = SegmentLength(FillXy);
                Double HaulLen = P.HaulDistanceM;
                DLoad += CutLen;
                DHaul += HaulLen;
                DDump += FillLen;

                // Loading and dumping use constant speeds.
                TLoad += (LoadV > 0.0) ? (CutLen / LoadV) : 0.0;
                TDump += (DumpV > 0.0) ? (FillLen / DumpV) : 0.0;

                // Hauling and returning use trapezoidal profile.
                THaul += MoveTime(HaulLen, HaulVMax, Ta, Td);
                TReturn += MoveTime(CutLen, ReturnVMax, Ta, Td);
                TReturn += MoveTime(HaulLen, ReturnVMax, Ta, Td);
                TReturn += MoveTime(FillLen, ReturnVMax, Ta, Td);
            }

            Double DReturn = DLoad + DHaul + DDump;
            Double Productive = TLoad + THaul + TDump + TReturn;
            Double Eff = ((Params.FieldEfficiency > 0.0) && (Params.FieldEfficiency <= 1.0)) ? Params.FieldEfficiency : 1.0;
            Double Total = Productive / Eff;
            Double Other = Total - Productive;
            Double AvgHaul = (THaul > 0.0) ? (DHaul / THaul / Constants.MPH_TO_MPS) : Params.HaulSpeedMph;
            Double AvgReturn = (TReturn > 0.0) ? (DReturn / TReturn / Constants.MPH_TO_MPS) : Params.ReturnSpeedMph;

            return new TimeResult(Total, DLoad, DHaul, DDump, DReturn, AvgHaul, AvgReturn, TLoad, THaul, TDump, TReturn, Other);
        }

        /// <summary>
        /// Computes polyline segment length.
        /// </summary>
        /// <param name="CellsXy">Polyline points.</param>
        /// <returns>Total length.</returns>
        static public Double SegmentLength
            (
            List<(Double X, Double Y)> CellsXy
            )
        {
            // Sum consecutive segment lengths.
            if (CellsXy.Count < 2)
            {
                return 0.0;
            }

            Double Sum = 0.0;
            for (Int32 Index = 0; Index < (CellsXy.Count - 1); Index += 1)
            {
                Double Dx = CellsXy[Index + 1].X - CellsXy[Index].X;
                Double Dy = CellsXy[Index + 1].Y - CellsXy[Index].Y;
                Sum += Math.Sqrt((Dx * Dx) + (Dy * Dy));
            }

            return Sum;
        }

        /// <summary>
        /// Computes move time under trapezoidal profile.
        /// </summary>
        /// <param name="LengthM">Move distance.</param>
        /// <param name="VmaxMps">Max speed.</param>
        /// <param name="AccelS">Accel time.</param>
        /// <param name="DecelS">Decel time.</param>
        /// <returns>Travel time.</returns>
        static public Double MoveTime
            (
            Double LengthM,
            Double VmaxMps,
            Double AccelS,
            Double DecelS
            )
        {
            // Use trapezoid when long enough, otherwise triangular peak profile.
            if ((LengthM <= 0.0) || (VmaxMps <= 0.0))
            {
                return 0.0;
            }

            Double DAcc = 0.5 * VmaxMps * AccelS;
            Double DDec = 0.5 * VmaxMps * DecelS;
            if (LengthM >= (DAcc + DDec))
            {
                Double Cruise = LengthM - DAcc - DDec;
                return AccelS + DecelS + (Cruise / VmaxMps);
            }

            Double A = VmaxMps / AccelS;
            Double B = VmaxMps / DecelS;
            Double Vp = Math.Sqrt((2.0 * LengthM) / ((1.0 / A) + (1.0 / B)));
            return (Vp / A) + (Vp / B);
        }
    }

    /// <summary>
    /// Backward-compatible snake/case-mismatched alias for time model.
    /// </summary>
    public static class Timemodel
    {
        /// <summary>
        /// Delegates to <see cref="TimeModel.EstimateTime"/>.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Route">Route model.</param>
        /// <param name="Params">Time parameters.</param>
        /// <returns>Estimated time result.</returns>
        static public TimeResult EstimateTime
            (
            Grid Grid,
            Route Route,
            TimeParams Params
            )
        {
            // Preserve old class casing used by the engine.
            return TimeModel.EstimateTime(Grid, Route, Params);
        }
    }
}
