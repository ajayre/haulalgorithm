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
        /// <param name="CutSpeedMph">Cut speed.</param>
        /// <param name="LoadedTravelSpeedMph">Maximum loaded travel speed.</param>
        /// <param name="FillSpeedMph">Fill speed.</param>
        /// <param name="UnloadedTravelSpeedMph">Maximum unloaded travel speed.</param>
        /// <param name="AccelerationTimeS">Acceleration time.</param>
        /// <param name="DecelerationTimeS">Deceleration time.</param>
        /// <param name="FieldEfficiency">Field efficiency fraction.</param>
        public TimeParams
            (
            Double CutSpeedMph,
            Double LoadedTravelSpeedMph,
            Double FillSpeedMph,
            Double UnloadedTravelSpeedMph,
            Double AccelerationTimeS,
            Double DecelerationTimeS,
            Double FieldEfficiency = 0.8
            )
        {
            // Preserve all machine speed and timing parameters.
            this.CutSpeedMph = CutSpeedMph;
            this.LoadedTravelSpeedMph = LoadedTravelSpeedMph;
            this.FillSpeedMph = FillSpeedMph;
            this.UnloadedTravelSpeedMph = UnloadedTravelSpeedMph;
            this.AccelerationTimeS = AccelerationTimeS;
            this.DecelerationTimeS = DecelerationTimeS;
            this.FieldEfficiency = FieldEfficiency;
        }

        /// <summary>
        /// Gets loading speed mph.
        /// </summary>
        public Double CutSpeedMph { get; }

        /// <summary>
        /// Gets haul speed mph.
        /// </summary>
        public Double LoadedTravelSpeedMph { get; }

        /// <summary>
        /// Gets dumping speed mph.
        /// </summary>
        public Double FillSpeedMph { get; }

        /// <summary>
        /// Gets return speed mph.
        /// </summary>
        public Double UnloadedTravelSpeedMph { get; }

        /// <summary>
        /// Gets acceleration time seconds.
        /// </summary>
        public Double AccelerationTimeS { get; }

        /// <summary>
        /// Gets deceleration time seconds.
        /// </summary>
        public Double DecelerationTimeS { get; }

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
    static public class TimeModel
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
            Double LoadV = Params.CutSpeedMph * Constants.MPH_TO_MPS;
            Double HaulVMax = Params.LoadedTravelSpeedMph * Constants.MPH_TO_MPS;
            Double DumpV = Params.FillSpeedMph * Constants.MPH_TO_MPS;
            Double ReturnVMax = Params.UnloadedTravelSpeedMph * Constants.MPH_TO_MPS;
            Double Ta = Params.AccelerationTimeS;
            Double Td = Params.DecelerationTimeS;

            Double DLoad = 0.0;
            Double DHaul = 0.0;
            Double DDump = 0.0;
            Double TLoad = 0.0;
            Double THaul = 0.0;
            Double TDump = 0.0;
            Double TReturn = 0.0;
            foreach (Pass P in Route.OrderedPasses)
            {
                PassTimeBreakdown Breakdown = EstimatePassBreakdown(Grid, P, LoadV, HaulVMax, DumpV, ReturnVMax, Ta, Td);
                DLoad += Breakdown.LoadingDistM;
                DHaul += Breakdown.HaulingDistM;
                DDump += Breakdown.DumpingDistM;
                TLoad += Breakdown.LoadingTimeS;
                THaul += Breakdown.HaulingTimeS;
                TDump += Breakdown.DumpingTimeS;
                TReturn += Breakdown.ReturningTimeS;
            }

            Double DReturn = DLoad + DHaul + DDump;
            Double Productive = TLoad + THaul + TDump + TReturn;
            Double Eff = ((Params.FieldEfficiency > 0.0) && (Params.FieldEfficiency <= 1.0)) ? Params.FieldEfficiency : 1.0;
            Double Total = Productive / Eff;
            Double Other = Total - Productive;
            Double AvgHaul = (THaul > 0.0) ? (DHaul / THaul / Constants.MPH_TO_MPS) : Params.LoadedTravelSpeedMph;
            Double AvgReturn = (TReturn > 0.0) ? (DReturn / TReturn / Constants.MPH_TO_MPS) : Params.UnloadedTravelSpeedMph;

            return new TimeResult(Total, DLoad, DHaul, DDump, DReturn, AvgHaul, AvgReturn, TLoad, THaul, TDump, TReturn, Other);
        }

        /// <summary>
        /// Estimates time to complete one routed pass, including field-efficiency overhead.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Pass">Pass to estimate.</param>
        /// <param name="Params">Time-model parameters.</param>
        /// <returns>Estimated route time in seconds.</returns>
        static public Double EstimatePassTime
            (
            Grid Grid,
            Pass Pass,
            TimeParams Params
            )
        {
            // Per-route time uses the same productive segments as the job total, scaled by efficiency.
            Double LoadV = Params.CutSpeedMph * Constants.MPH_TO_MPS;
            Double HaulVMax = Params.LoadedTravelSpeedMph * Constants.MPH_TO_MPS;
            Double DumpV = Params.FillSpeedMph * Constants.MPH_TO_MPS;
            Double ReturnVMax = Params.UnloadedTravelSpeedMph * Constants.MPH_TO_MPS;
            PassTimeBreakdown Breakdown = EstimatePassBreakdown(
                Grid,
                Pass,
                LoadV,
                HaulVMax,
                DumpV,
                ReturnVMax,
                Params.AccelerationTimeS,
                Params.DecelerationTimeS);
            Double Productive = Breakdown.LoadingTimeS
                + Breakdown.HaulingTimeS
                + Breakdown.DumpingTimeS
                + Breakdown.ReturningTimeS;
            Double Eff = ((Params.FieldEfficiency > 0.0) && (Params.FieldEfficiency <= 1.0)) ? Params.FieldEfficiency : 1.0;
            return Productive / Eff;
        }

        /// <summary>
        /// Computes cut volume moved on one pass in BCY.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Pass">Pass to measure.</param>
        /// <returns>Cut volume in BCY.</returns>
        static public Double PassCutBcy
            (
            Grid Grid,
            Pass Pass
            )
        {
            return PassCellVolumeBcy(Grid, Pass.CutCells);
        }

        /// <summary>
        /// Computes fill volume moved on one pass in BCY.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Pass">Pass to measure.</param>
        /// <returns>Fill volume in BCY.</returns>
        static public Double PassFillBcy
            (
            Grid Grid,
            Pass Pass
            )
        {
            return PassCellVolumeBcy(Grid, Pass.FillCells);
        }

        /// <summary>
        /// Computes per-pass distance and time segments used by the job-level estimator.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Pass">Pass to estimate.</param>
        /// <param name="LoadV">Loading speed in m/s.</param>
        /// <param name="HaulVMax">Max haul speed in m/s.</param>
        /// <param name="DumpV">Dumping speed in m/s.</param>
        /// <param name="ReturnVMax">Max return speed in m/s.</param>
        /// <param name="AccelS">Acceleration time in seconds.</param>
        /// <param name="DecelS">Deceleration time in seconds.</param>
        /// <returns>Per-pass distance and time breakdown.</returns>
        static private PassTimeBreakdown EstimatePassBreakdown
            (
            Grid Grid,
            Pass Pass,
            Double LoadV,
            Double HaulVMax,
            Double DumpV,
            Double ReturnVMax,
            Double AccelS,
            Double DecelS
            )
        {
            List<(Double X, Double Y)> CutXy = new List<(Double X, Double Y)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in Pass.CutCells)
            {
                CutXy.Add(Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By));
            }

            List<(Double X, Double Y)> FillXy = new List<(Double X, Double Y)>();
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell in Pass.FillCells)
            {
                FillXy.Add(Grid.CellCenterXy(Cell.Cell.Bx, Cell.Cell.By));
            }

            Double CutLen = Pass.StripLoadDistanceM ?? SegmentLength(CutXy);
            Double FillLen = Pass.StripSpreadDistanceM ?? SegmentLength(FillXy);
            Double HaulLen = Pass.HaulDistanceM;
            // A strip trip already counted its lane turns. The polyline would miss them
            // and would also charge the jump between lanes as if the blade stayed down.
            Double TLoad = Pass.StripLoadTimeS ?? ((LoadV > 0.0) ? (CutLen / LoadV) : 0.0);
            Double TDump = Pass.StripSpreadTimeS ?? ((DumpV > 0.0) ? (FillLen / DumpV) : 0.0);
            Double THaul = MoveTime(HaulLen, HaulVMax, AccelS, DecelS);
            Double TReturn = MoveTime(CutLen, ReturnVMax, AccelS, DecelS)
                + MoveTime(HaulLen, ReturnVMax, AccelS, DecelS)
                + MoveTime(FillLen, ReturnVMax, AccelS, DecelS);
            return new PassTimeBreakdown(CutLen, FillLen, HaulLen, TLoad, THaul, TDump, TReturn);
        }

        /// <summary>
        /// Sums bank volume from pass cell depths.
        /// </summary>
        /// <param name="Grid">Grid model.</param>
        /// <param name="Cells">Pass cells and depths.</param>
        /// <returns>Volume in BCY.</returns>
        static private Double PassCellVolumeBcy
            (
            Grid Grid,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            Double VolumeBcy = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                VolumeBcy += Entry.DepthM * Grid.AreaM2 * Constants.M3_TO_CY;
            }

            return VolumeBcy;
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

            Double DAcc = Constants.HALF_SCALE * VmaxMps * AccelS;
            Double DDec = Constants.HALF_SCALE * VmaxMps * DecelS;
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
    /// Per-pass distance and time segments for one cut-haul-fill cycle.
    /// </summary>
    internal readonly struct PassTimeBreakdown
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="PassTimeBreakdown"/> struct.
        /// </summary>
        /// <param name="LoadingDistM">Loading distance in metres.</param>
        /// <param name="DumpingDistM">Dumping distance in metres.</param>
        /// <param name="HaulingDistM">Hauling distance in metres.</param>
        /// <param name="LoadingTimeS">Loading time in seconds.</param>
        /// <param name="HaulingTimeS">Hauling time in seconds.</param>
        /// <param name="DumpingTimeS">Dumping time in seconds.</param>
        /// <param name="ReturningTimeS">Returning time in seconds.</param>
        public PassTimeBreakdown
            (
            Double LoadingDistM,
            Double DumpingDistM,
            Double HaulingDistM,
            Double LoadingTimeS,
            Double HaulingTimeS,
            Double DumpingTimeS,
            Double ReturningTimeS
            )
        {
            this.LoadingDistM = LoadingDistM;
            this.DumpingDistM = DumpingDistM;
            this.HaulingDistM = HaulingDistM;
            this.LoadingTimeS = LoadingTimeS;
            this.HaulingTimeS = HaulingTimeS;
            this.DumpingTimeS = DumpingTimeS;
            this.ReturningTimeS = ReturningTimeS;
        }

        /// <summary>
        /// Gets loading distance in metres.
        /// </summary>
        public Double LoadingDistM { get; }

        /// <summary>
        /// Gets dumping distance in metres.
        /// </summary>
        public Double DumpingDistM { get; }

        /// <summary>
        /// Gets hauling distance in metres.
        /// </summary>
        public Double HaulingDistM { get; }

        /// <summary>
        /// Gets loading time in seconds.
        /// </summary>
        public Double LoadingTimeS { get; }

        /// <summary>
        /// Gets hauling time in seconds.
        /// </summary>
        public Double HaulingTimeS { get; }

        /// <summary>
        /// Gets dumping time in seconds.
        /// </summary>
        public Double DumpingTimeS { get; }

        /// <summary>
        /// Gets returning time in seconds.
        /// </summary>
        public Double ReturningTimeS { get; }
    }

    /// <summary>
    /// Backward-compatible snake/case-mismatched alias for time model.
    /// </summary>
    static public class Timemodel
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
