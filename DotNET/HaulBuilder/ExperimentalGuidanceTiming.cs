using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Estimates experimental lifecycle time from the final driven segment geometry.
    /// </summary>
    static public class ExperimentalGuidanceTiming
    {
        private const Double ANGLE_TOLERANCE_RAD = 1e-6;
        private const Double AREA_TOLERANCE_M2 = 1e-12;
        private const Double COLLINEAR_RELATIVE_TOLERANCE = 1e-8;
        private const Double CONTINUITY_TOLERANCE_M = 1e-6;
        private const Double DEFAULT_CURVATURE_EVALUATION_SPACING_M = 0.5;
        private const Double DEFAULT_RADIUS_TOLERANCE_FRACTION = 0.02;
        private const Double DIAMETER_TO_RADIUS = 0.5;
        private const Double INTERPOLATION_TOLERANCE_M = 1e-12;
        private const Double MINIMUM_EDGE_LENGTH_M = 1e-9;
        private const Double RADIUS_TOLERANCE_M = 1e-6;
        private const Double RECONCILIATION_RELATIVE_TOLERANCE = 1e-8;
        private const Double SPEED_LIMIT_TOLERANCE_MPS = 1e-9;

        /// <summary>One final-geometry vertex tighter than the configured tractor radius.</summary>
        public sealed class CurveViolation
        {
            /// <summary>Initializes one curve-radius violation.</summary>
            public CurveViolation
                (
                Int32 RouteNumber,
                ExperimentalLifecyclePlanner.Phase PreviousPhase,
                ExperimentalLifecyclePlanner.Phase NextPhase,
                (Double X, Double Y) PreviousPoint,
                Double X,
                Double Y,
                (Double X, Double Y) NextPoint,
                Double RadiusM
                )
            {
                this.RouteNumber = RouteNumber;
                this.PreviousPhase = PreviousPhase;
                this.NextPhase = NextPhase;
                this.PreviousPoint = PreviousPoint;
                this.X = X;
                this.Y = Y;
                this.NextPoint = NextPoint;
                this.RadiusM = RadiusM;
            }

            /// <summary>Gets the owning one-based route.</summary>
            public Int32 RouteNumber { get; }

            /// <summary>Gets the incoming lifecycle phase.</summary>
            public ExperimentalLifecyclePlanner.Phase PreviousPhase { get; }

            /// <summary>Gets the outgoing lifecycle phase.</summary>
            public ExperimentalLifecyclePlanner.Phase NextPhase { get; }

            /// <summary>Gets the point immediately before the violating vertex.</summary>
            public (Double X, Double Y) PreviousPoint { get; }

            /// <summary>Gets the vertex easting.</summary>
            public Double X { get; }

            /// <summary>Gets the vertex northing.</summary>
            public Double Y { get; }

            /// <summary>Gets the point immediately after the violating vertex.</summary>
            public (Double X, Double Y) NextPoint { get; }

            /// <summary>Gets the observed local radius.</summary>
            public Double RadiusM { get; }
        }

        /// <summary>
        /// Configurable speeds and event durations for final-guidance timing.
        /// </summary>
        public sealed class Options
        {
            /// <summary>Initializes timing options with application defaults.</summary>
            public Options
                (
                )
            {
            }

            /// <summary>Initializes timing options from user-facing haul settings.</summary>
            public Options
                (
                HaulBuilderOptions Source
                )
            {
                if (Source == null) throw new ArgumentNullException(nameof(Source));
                CutSpeedMph = Source.LoadingSpeedMph;
                FillSpeedMph = Source.DumpingSpeedMph;
                GradeSpeedMph = Source.GradeSpeedMph;
                LoadedTravelSpeedMph = Source.HaulSpeedMph;
                UnloadedTravelSpeedMph = Source.ReturnSpeedMph;
                TurnSpeedMph = Source.TurnSpeedMph;
                MinimumTurningRadiusM = Source.TurningCircleFt
                    * DIAMETER_TO_RADIUS * Constants.FEET_TO_METERS;
                AccelerationMps2 = Source.ReturnSpeedMph * Constants.MPH_TO_MPS
                    / Source.AccelTimeS;
                DecelerationMps2 = Source.ReturnSpeedMph * Constants.MPH_TO_MPS
                    / Source.DecelTimeS;
                FieldEfficiency = Source.FieldEfficiency;
            }

            /// <summary>Gets or sets cut speed in miles per hour.</summary>
            public Double CutSpeedMph { get; set; } = Constants.DEFAULT_LOADING_SPEED_MPH;

            /// <summary>Gets or sets fill speed in miles per hour.</summary>
            public Double FillSpeedMph { get; set; } = Constants.DEFAULT_DUMPING_SPEED_MPH;

            /// <summary>Gets or sets grade speed in miles per hour.</summary>
            public Double GradeSpeedMph { get; set; } = Constants.DEFAULT_GRADE_SPEED_MPH;

            /// <summary>Gets or sets loaded full travel speed in miles per hour.</summary>
            public Double LoadedTravelSpeedMph { get; set; } = Constants.DEFAULT_HAUL_SPEED_MPH;

            /// <summary>Gets or sets unloaded full travel speed in miles per hour.</summary>
            public Double UnloadedTravelSpeedMph { get; set; } = Constants.DEFAULT_RETURN_SPEED_MPH;

            /// <summary>Gets or sets speed at maximum steering in miles per hour.</summary>
            public Double TurnSpeedMph { get; set; } = Constants.DEFAULT_TURN_SPEED_MPH;

            /// <summary>Gets or sets the tractor's geometric minimum turn radius.</summary>
            public Double MinimumTurningRadiusM { get; set; } =
                Constants.DEFAULT_TURNING_CIRCLE_FT
                * DIAMETER_TO_RADIUS * Constants.FEET_TO_METERS;

            /// <summary>
            /// Gets or sets equal path distance sampled before and after each
            /// vertex for stable curvature measurement.
            /// </summary>
            public Double CurvatureEvaluationSpacingM { get; set; } =
                DEFAULT_CURVATURE_EVALUATION_SPACING_M;

            /// <summary>
            /// Gets or sets relative radius tolerance for sampled minimum-radius
            /// arcs before a vertex is reported as a material tight corner.
            /// </summary>
            public Double MinimumRadiusToleranceFraction { get; set; } =
                DEFAULT_RADIUS_TOLERANCE_FRACTION;

            /// <summary>Gets or sets longitudinal acceleration in metres per second squared.</summary>
            public Double AccelerationMps2 { get; set; } =
                Constants.DEFAULT_RETURN_SPEED_MPH * Constants.MPH_TO_MPS / Constants.DEFAULT_ACCEL_TIME_S;

            /// <summary>Gets or sets longitudinal deceleration in metres per second squared.</summary>
            public Double DecelerationMps2 { get; set; } =
                Constants.DEFAULT_RETURN_SPEED_MPH * Constants.MPH_TO_MPS / Constants.DEFAULT_DECEL_TIME_S;

            /// <summary>Gets or sets fixed operator takeover overhead per assist.</summary>
            public Double ManualAssistOverheadS { get; set; }

            /// <summary>Gets or sets fixed blade-lower event duration.</summary>
            public Double BladeLowerS { get; set; }

            /// <summary>Gets or sets fixed blade-raise event duration.</summary>
            public Double BladeRaiseS { get; set; }

            /// <summary>
            /// Gets or sets operational availability applied after all explicit
            /// guidance motion and events. It must not include turn or deadhead loss.
            /// </summary>
            public Double FieldEfficiency { get; set; } = 1.0;
        }

        /// <summary>
        /// Timing summary by lifecycle phase.
        /// </summary>
        public sealed class Result
        {
            /// <summary>Initializes one timing result.</summary>
            public Result
                (
                Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseSeconds,
                Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseDistanceM,
                Dictionary<Int32, Double> RouteSeconds,
                Dictionary<Int32, Double> RouteDistanceM,
                Double MotionSeconds,
                Double ConstantSpeedSeconds,
                Double EventSeconds,
                Double FieldEfficiencyAllowanceSeconds,
                Double DistanceM,
                Double LoadedSeconds,
                Double LoadedDistanceM,
                Double UnloadedSeconds,
                Double UnloadedDistanceM,
                Double CurvatureLimitedDistanceM,
                Int32 MotionBlockCount,
                Int32 CurvatureConstraintCount,
                Int32 ForcedStopCount,
                Double MinimumObservedCurveRadiusM,
                List<CurveViolation> CurveViolations
                )
            {
                this.PhaseSeconds = PhaseSeconds;
                this.PhaseDistanceM = PhaseDistanceM;
                this.RouteSeconds = RouteSeconds;
                this.RouteDistanceM = RouteDistanceM;
                this.MotionSeconds = MotionSeconds;
                this.ConstantSpeedSeconds = ConstantSpeedSeconds;
                this.EventSeconds = EventSeconds;
                this.FieldEfficiencyAllowanceSeconds = FieldEfficiencyAllowanceSeconds;
                this.DistanceM = DistanceM;
                this.LoadedSeconds = LoadedSeconds;
                this.LoadedDistanceM = LoadedDistanceM;
                this.UnloadedSeconds = UnloadedSeconds;
                this.UnloadedDistanceM = UnloadedDistanceM;
                this.CurvatureLimitedDistanceM = CurvatureLimitedDistanceM;
                this.MotionBlockCount = MotionBlockCount;
                this.CurvatureConstraintCount = CurvatureConstraintCount;
                this.ForcedStopCount = ForcedStopCount;
                this.MinimumObservedCurveRadiusM = MinimumObservedCurveRadiusM;
                this.CurveViolations = CurveViolations;
            }

            /// <summary>Gets motion and phase-owned event seconds.</summary>
            public Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseSeconds { get; }

            /// <summary>Gets final centerline distance by lifecycle phase.</summary>
            public Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseDistanceM { get; }

            /// <summary>Gets motion and route-owned event seconds by one-based route.</summary>
            public Dictionary<Int32, Double> RouteSeconds { get; }

            /// <summary>Gets final centerline distance by one-based route.</summary>
            public Dictionary<Int32, Double> RouteDistanceM { get; }

            /// <summary>Gets modeled motion seconds.</summary>
            public Double MotionSeconds { get; }

            /// <summary>
            /// Gets travel seconds at local speed limits with instantaneous speed
            /// changes. This is the longitudinal-acceleration comparison baseline.
            /// </summary>
            public Double ConstantSpeedSeconds { get; }

            /// <summary>Gets additional seconds caused by longitudinal acceleration limits.</summary>
            public Double AccelerationConstraintSeconds =>
                MotionSeconds - ConstantSpeedSeconds;

            /// <summary>Gets explicit operational event seconds.</summary>
            public Double EventSeconds { get; }

            /// <summary>Gets separately reported operational-availability allowance.</summary>
            public Double FieldEfficiencyAllowanceSeconds { get; }

            /// <summary>Gets total centerline distance timed by the model.</summary>
            public Double DistanceM { get; }

            /// <summary>Gets motion seconds while carrying material.</summary>
            public Double LoadedSeconds { get; }

            /// <summary>Gets centerline distance while carrying material.</summary>
            public Double LoadedDistanceM { get; }

            /// <summary>Gets motion seconds while not carrying material.</summary>
            public Double UnloadedSeconds { get; }

            /// <summary>Gets centerline distance while not carrying material.</summary>
            public Double UnloadedDistanceM { get; }

            /// <summary>Gets distance whose phase speed is reduced by path curvature.</summary>
            public Double CurvatureLimitedDistanceM { get; }

            /// <summary>Gets the number of constant-limit motion blocks after compression.</summary>
            public Int32 MotionBlockCount { get; }

            /// <summary>Gets the number of path vertices with curvature speed constraints.</summary>
            public Int32 CurvatureConstraintCount { get; }

            /// <summary>Gets the number of discontinuous geometry boundaries timed as stops.</summary>
            public Int32 ForcedStopCount { get; }

            /// <summary>Gets the smallest nonzero curve radius observed in final geometry.</summary>
            public Double MinimumObservedCurveRadiusM { get; }

            /// <summary>Gets curved vertices tighter than the configured tractor radius.</summary>
            public Int32 BelowMinimumRadiusVertexCount => CurveViolations.Count;

            /// <summary>Gets details for every curve tighter than the tractor radius.</summary>
            public IReadOnlyList<CurveViolation> CurveViolations { get; }

            /// <summary>Gets reported total seconds.</summary>
            public Double TotalSeconds => MotionSeconds + EventSeconds + FieldEfficiencyAllowanceSeconds;
        }

        /// <summary>
        /// Times the final lifecycle as one continuous, curvature-limited speed profile.
        /// </summary>
        static public Result Estimate
            (
            ExperimentalLifecyclePlanner.Result Guidance,
            Options Settings
            )
        {
            if (Guidance == null) throw new ArgumentNullException(nameof(Guidance));
            if (Settings == null) throw new ArgumentNullException(nameof(Settings));
            if ((!PositiveFinite(Settings.AccelerationMps2))
                || (!PositiveFinite(Settings.DecelerationMps2))
                || (!PositiveFinite(Settings.MinimumTurningRadiusM))
                || (!PositiveFinite(Settings.CurvatureEvaluationSpacingM))
                || (!PositiveFinite(Settings.CutSpeedMph))
                || (!PositiveFinite(Settings.FillSpeedMph))
                || (!PositiveFinite(Settings.GradeSpeedMph))
                || (!PositiveFinite(Settings.TurnSpeedMph))
                || (!PositiveFinite(Settings.LoadedTravelSpeedMph))
                || (!PositiveFinite(Settings.UnloadedTravelSpeedMph))
                || (Settings.LoadedTravelSpeedMph < Settings.TurnSpeedMph)
                || (Settings.UnloadedTravelSpeedMph < Settings.TurnSpeedMph)
                || (!Double.IsFinite(Settings.MinimumRadiusToleranceFraction))
                || (Settings.MinimumRadiusToleranceFraction < 0.0)
                || (Settings.MinimumRadiusToleranceFraction >= 1.0)
                || (!NonnegativeFinite(Settings.ManualAssistOverheadS))
                || (!NonnegativeFinite(Settings.BladeLowerS))
                || (!NonnegativeFinite(Settings.BladeRaiseS)))
            {
                throw new ArgumentOutOfRangeException(nameof(Settings));
            }
            if ((!Double.IsFinite(Settings.FieldEfficiency))
                || (Settings.FieldEfficiency <= 0.0)
                || (Settings.FieldEfficiency > 1.0))
            {
                throw new ArgumentOutOfRangeException(nameof(Settings.FieldEfficiency));
            }

            Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseSeconds =
                new Dictionary<ExperimentalLifecyclePlanner.Phase, Double>();
            Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseDistanceM =
                new Dictionary<ExperimentalLifecyclePlanner.Phase, Double>();
            Dictionary<Int32, Double> RouteSeconds = new Dictionary<Int32, Double>();
            Dictionary<Int32, Double> RouteDistanceM = new Dictionary<Int32, Double>();
            // Flatten phase polylines once so speed stays continuous across every
            // lifecycle label and geometry ownership remains auditable by route.
            List<MotionEdge> Edges = BuildEdges(Guidance, Settings);
            List<Double> BoundaryCurvatureCapsMps = BuildBoundaryCurvatureCaps(
                Edges, Settings.TurnSpeedMph * Constants.MPH_TO_MPS,
                Settings.MinimumTurningRadiusM, Settings.CurvatureEvaluationSpacingM,
                Settings.MinimumRadiusToleranceFraction,
                out Int32 CurvatureConstraintCount, out Int32 ForcedStopCount,
                out Double MinimumObservedCurveRadiusM,
                out List<CurveViolation> CurveViolations);
            ApplySustainedCurvatureLimits(Edges, BoundaryCurvatureCapsMps);
            CompressEdges(Edges, BoundaryCurvatureCapsMps,
                out List<MotionBlock> Blocks, out List<Double> BoundaryCapsMps);
            Double[] BoundarySpeedsMps = BuildSpeedEnvelope(
                Blocks, BoundaryCapsMps, Settings.AccelerationMps2,
                Settings.DecelerationMps2);

            Double MotionSeconds = 0.0;
            Double ConstantSpeedSeconds = 0.0;
            Double DistanceM = 0.0;
            Double LoadedSeconds = 0.0;
            Double LoadedDistanceM = 0.0;
            Double UnloadedSeconds = 0.0;
            Double UnloadedDistanceM = 0.0;
            Double CurvatureLimitedDistanceM = 0.0;
            // Assign interval time to its phase and route without breaking the
            // shared boundary velocities calculated by the global speed envelope.
            for (Int32 Index = 0; Index < Blocks.Count; Index += 1)
            {
                MotionBlock Block = Blocks[Index];
                Double BlockSeconds = MotionTime(
                    Block.LengthM, BoundarySpeedsMps[Index],
                    BoundarySpeedsMps[Index + 1], Block.MaximumSpeedMps,
                    Settings.AccelerationMps2, Settings.DecelerationMps2);
                Double BlockConstantSeconds = Block.LengthM / Block.MaximumSpeedMps;
                Add(PhaseSeconds, Block.Phase, BlockSeconds);
                Add(PhaseDistanceM, Block.Phase, Block.LengthM);
                Add(RouteSeconds, Block.RouteNumber, BlockSeconds);
                Add(RouteDistanceM, Block.RouteNumber, Block.LengthM);
                MotionSeconds += BlockSeconds;
                ConstantSpeedSeconds += BlockConstantSeconds;
                DistanceM += Block.LengthM;
                if (Block.Loaded)
                {
                    LoadedSeconds += BlockSeconds;
                    LoadedDistanceM += Block.LengthM;
                }
                else
                {
                    UnloadedSeconds += BlockSeconds;
                    UnloadedDistanceM += Block.LengthM;
                }
                if (Block.CurvatureLimited)
                {
                    CurvatureLimitedDistanceM += Block.LengthM;
                }
            }

            Double EventSeconds = 0.0;
            Boolean PreviousBladeDown = false;
            Boolean PreviousManualAssist = false;
            // Operational events belong to state transitions, not sampled points
            // or repeated segment fragments inside the same state.
            foreach (ExperimentalLifecyclePlanner.Segment Segment in Guidance.Segments)
            {
                if ((Segment.BladeDown) && (!PreviousBladeDown))
                {
                    EventSeconds += Settings.BladeLowerS;
                    Add(PhaseSeconds, Segment.Phase, Settings.BladeLowerS);
                    Add(RouteSeconds, Segment.RouteNumber, Settings.BladeLowerS);
                }
                if ((!Segment.BladeDown) && (PreviousBladeDown))
                {
                    EventSeconds += Settings.BladeRaiseS;
                    Add(PhaseSeconds, Segment.Phase, Settings.BladeRaiseS);
                    Add(RouteSeconds, Segment.RouteNumber, Settings.BladeRaiseS);
                }
                Boolean IsManualAssist =
                    Segment.Phase == ExperimentalLifecyclePlanner.Phase.ManualAssist;
                if ((IsManualAssist) && (!PreviousManualAssist))
                {
                    EventSeconds += Settings.ManualAssistOverheadS;
                    Add(PhaseSeconds, Segment.Phase, Settings.ManualAssistOverheadS);
                    Add(RouteSeconds, Segment.RouteNumber, Settings.ManualAssistOverheadS);
                }
                PreviousBladeDown = Segment.BladeDown;
                PreviousManualAssist = IsManualAssist;
            }

            Double ProductiveSeconds = MotionSeconds + EventSeconds;
            Double AllowanceSeconds = (ProductiveSeconds / Settings.FieldEfficiency) - ProductiveSeconds;
            Double PayloadTolerance = RECONCILIATION_RELATIVE_TOLERANCE
                * Math.Max(1.0, Math.Max(MotionSeconds, DistanceM));
            if ((Math.Abs(LoadedSeconds + UnloadedSeconds - MotionSeconds)
                    > PayloadTolerance)
                || (Math.Abs(LoadedDistanceM + UnloadedDistanceM - DistanceM)
                    > PayloadTolerance))
            {
                throw new InvalidOperationException(
                    "Guidance timing failed loaded or unloaded reconciliation.");
            }
            ValidateReconciliation(
                Guidance,
                PhaseSeconds,
                PhaseDistanceM,
                RouteSeconds,
                RouteDistanceM,
                MotionSeconds,
                EventSeconds,
                ConstantSpeedSeconds,
                DistanceM);
            return new Result(
                PhaseSeconds, PhaseDistanceM, RouteSeconds, RouteDistanceM,
                MotionSeconds, ConstantSpeedSeconds,
                EventSeconds, AllowanceSeconds, DistanceM,
                LoadedSeconds, LoadedDistanceM,
                UnloadedSeconds, UnloadedDistanceM,
                CurvatureLimitedDistanceM, Blocks.Count,
                CurvatureConstraintCount, ForcedStopCount,
                MinimumObservedCurveRadiusM, CurveViolations);
        }

        /// <summary>Returns whether a physical configuration value is positive and finite.</summary>
        private static Boolean PositiveFinite
            (
            Double Value
            )
        {
            return Double.IsFinite(Value) && (Value > 0.0);
        }

        /// <summary>Returns whether a duration is nonnegative and finite.</summary>
        private static Boolean NonnegativeFinite
            (
            Double Value
            )
        {
            return Double.IsFinite(Value) && (Value >= 0.0);
        }

        /// <summary>Rejects any internal loss or duplication of distance or time.</summary>
        private static void ValidateReconciliation
            (
            ExperimentalLifecyclePlanner.Result Guidance,
            Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseSeconds,
            Dictionary<ExperimentalLifecyclePlanner.Phase, Double> PhaseDistanceM,
            Dictionary<Int32, Double> RouteSeconds,
            Dictionary<Int32, Double> RouteDistanceM,
            Double MotionSeconds,
            Double EventSeconds,
            Double ConstantSpeedSeconds,
            Double DistanceM
            )
        {
            // Independent phase and route ledgers make any lost or duplicated
            // interval a hard failure instead of a plausible-looking report.
            Double TimeToleranceS = RECONCILIATION_RELATIVE_TOLERANCE
                * Math.Max(1.0, MotionSeconds + EventSeconds);
            Double DistanceToleranceM = RECONCILIATION_RELATIVE_TOLERANCE
                * Math.Max(1.0, DistanceM);
            if ((Math.Abs(Sum(PhaseSeconds) - MotionSeconds - EventSeconds) > TimeToleranceS)
                || (Math.Abs(Sum(RouteSeconds) - MotionSeconds - EventSeconds) > TimeToleranceS)
                || (Math.Abs(Sum(PhaseDistanceM) - DistanceM) > DistanceToleranceM)
                || (Math.Abs(Sum(RouteDistanceM) - DistanceM) > DistanceToleranceM)
                || (ConstantSpeedSeconds > MotionSeconds + TimeToleranceS))
            {
                throw new InvalidOperationException(
                    "Guidance timing failed internal time or distance reconciliation.");
            }
            if ((Guidance.DistanceM > 0.0)
                && (Math.Abs(Guidance.DistanceM - DistanceM) > DistanceToleranceM))
            {
                throw new InvalidOperationException(
                    "Timed distance does not reconcile with lifecycle distance.");
            }
        }

        /// <summary>Sums dictionary values without changing their numeric order.</summary>
        /// <typeparam name="TKey">Dictionary key type.</typeparam>
        private static Double Sum<TKey>
            (
            Dictionary<TKey, Double> Values
            ) where TKey : notnull
        {
            Double Total = 0.0;
            foreach (Double Value in Values.Values)
            {
                Total += Value;
            }
            return Total;
        }

        /// <summary>One nonzero polyline edge before speed-limit compression.</summary>
        private sealed class MotionEdge
        {
            /// <summary>Initializes one motion edge.</summary>
            public MotionEdge
                (
                ExperimentalLifecyclePlanner.Phase Phase,
                Int32 RouteNumber,
                Boolean Loaded,
                (Double X, Double Y) Start,
                (Double X, Double Y) End,
                Double LengthM,
                Double PhaseSpeedMps,
                Double FullTravelSpeedMps
                )
            {
                this.Phase = Phase;
                this.RouteNumber = RouteNumber;
                this.Loaded = Loaded;
                this.Start = Start;
                this.End = End;
                this.LengthM = LengthM;
                this.PhaseSpeedMps = PhaseSpeedMps;
                this.FullTravelSpeedMps = FullTravelSpeedMps;
                MaximumSpeedMps = PhaseSpeedMps;
            }

            /// <summary>Gets the lifecycle phase.</summary>
            public ExperimentalLifecyclePlanner.Phase Phase { get; }

            /// <summary>Gets the one-based owning route.</summary>
            public Int32 RouteNumber { get; }

            /// <summary>Gets whether material is carried on this edge.</summary>
            public Boolean Loaded { get; }

            /// <summary>Gets the edge start.</summary>
            public (Double X, Double Y) Start { get; }

            /// <summary>Gets the edge end.</summary>
            public (Double X, Double Y) End { get; }

            /// <summary>Gets the edge length.</summary>
            public Double LengthM { get; }

            /// <summary>Gets the phase speed limit.</summary>
            public Double PhaseSpeedMps { get; }

            /// <summary>Gets the payload-dependent straight travel speed.</summary>
            public Double FullTravelSpeedMps { get; }

            /// <summary>Gets or sets the effective edge speed limit.</summary>
            public Double MaximumSpeedMps { get; set; }

            /// <summary>Gets or sets whether sustained curvature limits this edge.</summary>
            public Boolean CurvatureLimited { get; set; }
        }

        /// <summary>One compressed distance interval with a uniform speed limit.</summary>
        private sealed class MotionBlock
        {
            /// <summary>Initializes one motion block.</summary>
            public MotionBlock
                (
                ExperimentalLifecyclePlanner.Phase Phase,
                Int32 RouteNumber,
                Boolean Loaded,
                Double LengthM,
                Double MaximumSpeedMps,
                Boolean CurvatureLimited
                )
            {
                this.Phase = Phase;
                this.RouteNumber = RouteNumber;
                this.Loaded = Loaded;
                this.LengthM = LengthM;
                this.MaximumSpeedMps = MaximumSpeedMps;
                this.CurvatureLimited = CurvatureLimited;
            }

            /// <summary>Gets the lifecycle phase.</summary>
            public ExperimentalLifecyclePlanner.Phase Phase { get; }

            /// <summary>Gets the one-based owning route.</summary>
            public Int32 RouteNumber { get; }

            /// <summary>Gets whether material is carried in this block.</summary>
            public Boolean Loaded { get; }

            /// <summary>Gets or sets block length.</summary>
            public Double LengthM { get; set; }

            /// <summary>Gets the block speed limit.</summary>
            public Double MaximumSpeedMps { get; }

            /// <summary>Gets whether sustained curvature limits this block.</summary>
            public Boolean CurvatureLimited { get; }
        }

        /// <summary>Flattens lifecycle polylines into nonzero motion edges.</summary>
        private static List<MotionEdge> BuildEdges
            (
            ExperimentalLifecyclePlanner.Result Guidance,
            Options Settings
            )
        {
            List<MotionEdge> Edges = new List<MotionEdge>();
            foreach (ExperimentalLifecyclePlanner.Segment Segment in Guidance.Segments)
            {
                Double FullTravelSpeedMps = (Segment.Loaded
                    ? Settings.LoadedTravelSpeedMph
                    : Settings.UnloadedTravelSpeedMph) * Constants.MPH_TO_MPS;
                Double PhaseSpeedMps = PhaseSpeedMph(
                    Segment.Phase, Segment.Loaded, Settings)
                    * Constants.MPH_TO_MPS;
                if (PhaseSpeedMps <= 0.0)
                {
                    throw new ArgumentOutOfRangeException(nameof(Settings),
                        "Every driven phase must have a positive speed.");
                }
                for (Int32 Index = 1; Index < Segment.Points.Count; Index += 1)
                {
                    (Double X, Double Y) Start = Segment.Points[Index - 1];
                    (Double X, Double Y) End = Segment.Points[Index];
                    Double LengthM = Distance(Start, End);
                    if (LengthM > MINIMUM_EDGE_LENGTH_M)
                    {
                        Edges.Add(new MotionEdge(
                            Segment.Phase, Segment.RouteNumber, Segment.Loaded,
                            Start, End, LengthM, PhaseSpeedMps,
                            FullTravelSpeedMps));
                    }
                }
            }
            return Edges;
        }

        /// <summary>Computes curvature and continuity speed caps at every edge boundary.</summary>
        private static List<Double> BuildBoundaryCurvatureCaps
            (
            List<MotionEdge> Edges,
            Double TurnSpeedMps,
            Double MinimumTurningRadiusM,
            Double CurvatureEvaluationSpacingM,
            Double MinimumRadiusToleranceFraction,
            out Int32 CurvatureConstraintCount,
            out Int32 ForcedStopCount,
            out Double MinimumObservedCurveRadiusM,
            out List<CurveViolation> CurveViolations
            )
        {
            List<Double> CapsMps = new List<Double>(Edges.Count + 1);
            for (Int32 Index = 0; Index <= Edges.Count; Index += 1)
            {
                CapsMps.Add(Double.PositiveInfinity);
            }
            CurvatureConstraintCount = 0;
            ForcedStopCount = 0;
            MinimumObservedCurveRadiusM = Double.PositiveInfinity;
            CurveViolations = new List<CurveViolation>();
            if (Edges.Count == 0)
            {
                return CapsMps;
            }
            CapsMps[0] = 0.0;
            CapsMps[Edges.Count] = 0.0;
            for (Int32 Index = 1; Index < Edges.Count; Index += 1)
            {
                MotionEdge Previous = Edges[Index - 1];
                MotionEdge Next = Edges[Index];
                if (Distance(Previous.End, Next.Start) > CONTINUITY_TOLERANCE_M)
                {
                    CapsMps[Index] = 0.0;
                    ForcedStopCount += 1;
                    continue;
                }
                // Phase slicing can insert millimetre slivers inside an otherwise
                // 0.5 m sampled chord. Recover both complete adjacent chords before
                // estimating curvature so the result is sampling stable.
                Double BackwardAvailableM = StraightSpanBefore(
                    Edges, Index, CurvatureEvaluationSpacingM);
                Double ForwardAvailableM = StraightSpanAfter(
                    Edges, Index, CurvatureEvaluationSpacingM);
                Double EvaluationSpacingM = Math.Min(
                    CurvatureEvaluationSpacingM,
                    Math.Min(BackwardAvailableM, ForwardAvailableM));
                if (EvaluationSpacingM <= MINIMUM_EDGE_LENGTH_M)
                {
                    continue;
                }
                (Double X, Double Y) PreviousPoint = PointBefore(
                    Edges, Index, EvaluationSpacingM);
                (Double X, Double Y) NextPoint = PointAfter(
                    Edges, Index, EvaluationSpacingM);
                Double TurnAngleRad = TurnAngle(
                    PreviousPoint, Previous.End, NextPoint);
                Double RadiusM = CurvatureRadius(
                    PreviousPoint, Previous.End, NextPoint);
                if (!Double.IsPositiveInfinity(RadiusM))
                {
                    // Compare the observed heading change with the heading change
                    // produced by the minimum-radius circle over the same chord.
                    // This makes the linear rule independent of waypoint density.
                    Double MaximumTurnAngleRad = 2.0 * Math.Asin(Math.Min(
                        1.0, EvaluationSpacingM / (2.0 * MinimumTurningRadiusM)));
                    Double TurnSeverity = Math.Min(
                        1.0, TurnAngleRad / MaximumTurnAngleRad);
                    Double FullTravelSpeedMps = Math.Min(
                        Previous.FullTravelSpeedMps, Next.FullTravelSpeedMps);
                    CapsMps[Index] = FullTravelSpeedMps
                        - (TurnSeverity
                            * (FullTravelSpeedMps - TurnSpeedMps));
                    CurvatureConstraintCount += 1;
                    if (RadiusM > 0.0)
                    {
                        MinimumObservedCurveRadiusM = Math.Min(
                            MinimumObservedCurveRadiusM, RadiusM);
                    }
                    if (RadiusM + RADIUS_TOLERANCE_M
                        < MinimumTurningRadiusM
                            * (1.0 - MinimumRadiusToleranceFraction))
                    {
                        CurveViolations.Add(new CurveViolation(
                            Next.RouteNumber,
                            Previous.Phase,
                            Next.Phase,
                            PreviousPoint,
                            Previous.End.X,
                            Previous.End.Y,
                            NextPoint,
                            RadiusM));
                    }
                }
            }
            return CapsMps;
        }

        /// <summary>Returns incoming straight-chord distance before a boundary.</summary>
        private static Double StraightSpanBefore
            (
            List<MotionEdge> Edges,
            Int32 BoundaryIndex,
            Double MaximumDistanceM
            )
        {
            MotionEdge Reference = Edges[BoundaryIndex - 1];
            Double AvailableM = Math.Min(Reference.LengthM, MaximumDistanceM);
            for (Int32 Index = BoundaryIndex - 2;
                (Index >= 0) && (AvailableM < MaximumDistanceM);
                Index -= 1)
            {
                MotionEdge Candidate = Edges[Index];
                if ((Distance(Candidate.End, Edges[Index + 1].Start)
                        > CONTINUITY_TOLERANCE_M)
                    || !SameDirection(Candidate, Reference))
                {
                    break;
                }
                AvailableM += Math.Min(
                    Candidate.LengthM, MaximumDistanceM - AvailableM);
            }
            return AvailableM;
        }

        /// <summary>Returns outgoing straight-chord distance after a boundary.</summary>
        private static Double StraightSpanAfter
            (
            List<MotionEdge> Edges,
            Int32 BoundaryIndex,
            Double MaximumDistanceM
            )
        {
            MotionEdge Reference = Edges[BoundaryIndex];
            Double AvailableM = Math.Min(Reference.LengthM, MaximumDistanceM);
            for (Int32 Index = BoundaryIndex + 1;
                (Index < Edges.Count) && (AvailableM < MaximumDistanceM);
                Index += 1)
            {
                MotionEdge Candidate = Edges[Index];
                if ((Distance(Edges[Index - 1].End, Candidate.Start)
                        > CONTINUITY_TOLERANCE_M)
                    || !SameDirection(Candidate, Reference))
                {
                    break;
                }
                AvailableM += Math.Min(
                    Candidate.LengthM, MaximumDistanceM - AvailableM);
            }
            return AvailableM;
        }

        /// <summary>Returns whether two directed edges lie on the same straight chord.</summary>
        private static Boolean SameDirection
            (
            MotionEdge A,
            MotionEdge B
            )
        {
            Double Adx = A.End.X - A.Start.X;
            Double Ady = A.End.Y - A.Start.Y;
            Double Bdx = B.End.X - B.Start.X;
            Double Bdy = B.End.Y - B.Start.Y;
            Double Cross = Math.Abs((Adx * Bdy) - (Ady * Bdx));
            Double Scale = A.LengthM * B.LengthM;
            Double Dot = (Adx * Bdx) + (Ady * Bdy);
            return (Dot > 0.0)
                && (Cross <= COLLINEAR_RELATIVE_TOLERANCE * Scale);
        }

        /// <summary>Interpolates a point a fixed connected distance before a boundary.</summary>
        private static (Double X, Double Y) PointBefore
            (
            List<MotionEdge> Edges,
            Int32 BoundaryIndex,
            Double DistanceM
            )
        {
            Double RemainingM = DistanceM;
            for (Int32 Index = BoundaryIndex - 1; Index >= 0; Index -= 1)
            {
                MotionEdge Edge = Edges[Index];
                if (RemainingM <= Edge.LengthM + INTERPOLATION_TOLERANCE_M)
                {
                    Double Fraction = Math.Max(0.0,
                        Math.Min(1.0, RemainingM / Edge.LengthM));
                    return (
                        Edge.End.X + (Fraction * (Edge.Start.X - Edge.End.X)),
                        Edge.End.Y + (Fraction * (Edge.Start.Y - Edge.End.Y)));
                }
                RemainingM -= Edge.LengthM;
            }
            return Edges[0].Start;
        }

        /// <summary>Interpolates a point a fixed connected distance after a boundary.</summary>
        private static (Double X, Double Y) PointAfter
            (
            List<MotionEdge> Edges,
            Int32 BoundaryIndex,
            Double DistanceM
            )
        {
            Double RemainingM = DistanceM;
            for (Int32 Index = BoundaryIndex; Index < Edges.Count; Index += 1)
            {
                MotionEdge Edge = Edges[Index];
                if (RemainingM <= Edge.LengthM + INTERPOLATION_TOLERANCE_M)
                {
                    Double Fraction = Math.Max(0.0,
                        Math.Min(1.0, RemainingM / Edge.LengthM));
                    return (
                        Edge.Start.X + (Fraction * (Edge.End.X - Edge.Start.X)),
                        Edge.Start.Y + (Fraction * (Edge.End.Y - Edge.Start.Y)));
                }
                RemainingM -= Edge.LengthM;
            }
            return Edges[Edges.Count - 1].End;
        }

        /// <summary>Applies a curve limit to edges bounded by curved vertices.</summary>
        private static void ApplySustainedCurvatureLimits
            (
            List<MotionEdge> Edges,
            List<Double> BoundaryCurvatureCapsMps
            )
        {
            for (Int32 Index = 0; Index < Edges.Count; Index += 1)
            {
                Double StartCapMps = BoundaryCurvatureCapsMps[Index];
                Double EndCapMps = BoundaryCurvatureCapsMps[Index + 1];
                if (!Double.IsPositiveInfinity(StartCapMps)
                    && !Double.IsPositiveInfinity(EndCapMps)
                    && (StartCapMps > 0.0) && (EndCapMps > 0.0))
                {
                    Double CurveCapMps = Math.Min(StartCapMps, EndCapMps);
                    if (CurveCapMps < Edges[Index].MaximumSpeedMps)
                    {
                        Edges[Index].MaximumSpeedMps = CurveCapMps;
                        Edges[Index].CurvatureLimited = true;
                    }
                }
            }
        }

        /// <summary>Compresses adjacent equivalent edges without losing vertex constraints.</summary>
        private static void CompressEdges
            (
            List<MotionEdge> Edges,
            List<Double> BoundaryCurvatureCapsMps,
            out List<MotionBlock> Blocks,
            out List<Double> BoundaryCapsMps
            )
        {
            Blocks = new List<MotionBlock>();
            BoundaryCapsMps = new List<Double>();
            if (Edges.Count == 0)
            {
                BoundaryCapsMps.Add(0.0);
                return;
            }
            BoundaryCapsMps.Add(0.0);
            MotionEdge First = Edges[0];
            Blocks.Add(new MotionBlock(
                First.Phase, First.RouteNumber, First.Loaded,
                First.LengthM, First.MaximumSpeedMps, First.CurvatureLimited));
            for (Int32 Index = 1; Index < Edges.Count; Index += 1)
            {
                MotionEdge Edge = Edges[Index];
                MotionBlock Previous = Blocks[Blocks.Count - 1];
                Double VertexCapMps = Math.Min(
                    BoundaryCurvatureCapsMps[Index],
                    Math.Min(Previous.MaximumSpeedMps, Edge.MaximumSpeedMps));
                // Preserve a block boundary whenever phase ownership, route
                // ownership, or a lower vertex cap affects the integrated result.
                Boolean CanMerge = (Previous.Phase == Edge.Phase)
                    && (Previous.RouteNumber == Edge.RouteNumber)
                    && (Previous.Loaded == Edge.Loaded)
                    && (Math.Abs(Previous.MaximumSpeedMps - Edge.MaximumSpeedMps)
                        <= SPEED_LIMIT_TOLERANCE_MPS)
                    && (VertexCapMps >= Previous.MaximumSpeedMps
                        - SPEED_LIMIT_TOLERANCE_MPS);
                if (CanMerge)
                {
                    Previous.LengthM += Edge.LengthM;
                }
                else
                {
                    BoundaryCapsMps.Add(VertexCapMps);
                    Blocks.Add(new MotionBlock(
                        Edge.Phase, Edge.RouteNumber, Edge.Loaded,
                        Edge.LengthM, Edge.MaximumSpeedMps,
                        Edge.CurvatureLimited));
                }
            }
            BoundaryCapsMps.Add(0.0);
        }

        /// <summary>Builds the maximum feasible speed at every block boundary.</summary>
        private static Double[] BuildSpeedEnvelope
            (
            List<MotionBlock> Blocks,
            List<Double> BoundaryCapsMps,
            Double AccelerationMps2,
            Double DecelerationMps2
            )
        {
            Double[] SpeedsMps = BoundaryCapsMps.ToArray();
            if (Blocks.Count == 0)
            {
                return SpeedsMps;
            }
            // The forward pass enforces acceleration from the initial stop and
            // every lower local speed cap.
            for (Int32 Index = 0; Index < Blocks.Count; Index += 1)
            {
                Double ReachableMps = Math.Sqrt(
                    (SpeedsMps[Index] * SpeedsMps[Index])
                    + (2.0 * AccelerationMps2 * Blocks[Index].LengthM));
                SpeedsMps[Index + 1] = Math.Min(
                    SpeedsMps[Index + 1],
                    Math.Min(Blocks[Index].MaximumSpeedMps, ReachableMps));
            }
            // The reverse pass propagates braking requirements from the final stop
            // and future lower speed caps without introducing artificial stops.
            for (Int32 Index = Blocks.Count - 1; Index >= 0; Index -= 1)
            {
                Double ReachableMps = Math.Sqrt(
                    (SpeedsMps[Index + 1] * SpeedsMps[Index + 1])
                    + (2.0 * DecelerationMps2 * Blocks[Index].LengthM));
                SpeedsMps[Index] = Math.Min(
                    SpeedsMps[Index],
                    Math.Min(Blocks[Index].MaximumSpeedMps, ReachableMps));
            }
            return SpeedsMps;
        }

        /// <summary>Computes a local curve speed from three consecutive path points.</summary>
        private static Double CurvatureRadius
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            (Double X, Double Y) C
            )
        {
            Double AbM = Distance(A, B);
            Double BcM = Distance(B, C);
            Double AcM = Distance(A, C);
            Double TurnRad = TurnAngle(A, B, C);
            if (TurnRad <= ANGLE_TOLERANCE_RAD)
            {
                return Double.PositiveInfinity;
            }
            if ((Math.PI - TurnRad) <= ANGLE_TOLERANCE_RAD)
            {
                return 0.0;
            }
            Double TwiceArea = Math.Abs(
                ((B.X - A.X) * (C.Y - A.Y))
                - ((B.Y - A.Y) * (C.X - A.X)));
            if (TwiceArea <= AREA_TOLERANCE_M2)
            {
                return 0.0;
            }
            Double RadiusM = (AbM * BcM * AcM) / (2.0 * TwiceArea);
            return RadiusM;
        }

        /// <summary>Computes absolute heading change at a path vertex.</summary>
        private static Double TurnAngle
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B,
            (Double X, Double Y) C
            )
        {
            Double Cross = Math.Abs(
                ((B.X - A.X) * (C.Y - B.Y))
                - ((B.Y - A.Y) * (C.X - B.X)));
            Double Dot = ((B.X - A.X) * (C.X - B.X))
                + ((B.Y - A.Y) * (C.Y - B.Y));
            return Math.Atan2(Cross, Dot);
        }

        /// <summary>Maps each lifecycle phase to its configured maximum speed.</summary>
        private static Double PhaseSpeedMph
            (
            ExperimentalLifecyclePlanner.Phase Phase,
            Boolean Loaded,
            Options Settings
            )
        {
            Double TravelSpeedMph = Loaded
                ? Settings.LoadedTravelSpeedMph
                : Settings.UnloadedTravelSpeedMph;
            return Phase switch
            {
                ExperimentalLifecyclePlanner.Phase.Cut => Settings.CutSpeedMph,
                ExperimentalLifecyclePlanner.Phase.Fill => Settings.FillSpeedMph,
                ExperimentalLifecyclePlanner.Phase.Grade => Settings.GradeSpeedMph,
                _ => TravelSpeedMph,
            };
        }

        /// <summary>Computes one interval's trapezoidal or triangular motion time.</summary>
        private static Double MotionTime
            (
            Double DistanceM,
            Double StartSpeedMps,
            Double EndSpeedMps,
            Double MaximumSpeedMps,
            Double AccelerationMps2,
            Double DecelerationMps2
            )
        {
            if ((DistanceM <= 0.0) || (MaximumSpeedMps <= 0.0)) return 0.0;
            Double AccelerationDistanceM =
                ((MaximumSpeedMps * MaximumSpeedMps)
                - (StartSpeedMps * StartSpeedMps)) / (2.0 * AccelerationMps2);
            Double DecelerationDistanceM =
                ((MaximumSpeedMps * MaximumSpeedMps)
                - (EndSpeedMps * EndSpeedMps)) / (2.0 * DecelerationMps2);
            if (DistanceM >= AccelerationDistanceM + DecelerationDistanceM)
            {
                // Reach the local cap only when both transition distances fit;
                // otherwise the triangular branch solves the reachable peak.
                Double CruiseM = DistanceM - AccelerationDistanceM - DecelerationDistanceM;
                return ((MaximumSpeedMps - StartSpeedMps) / AccelerationMps2)
                    + (CruiseM / MaximumSpeedMps)
                    + ((MaximumSpeedMps - EndSpeedMps) / DecelerationMps2);
            }
            Double PeakMps = Math.Sqrt(
                ((2.0 * DistanceM * AccelerationMps2 * DecelerationMps2)
                + (DecelerationMps2 * StartSpeedMps * StartSpeedMps)
                + (AccelerationMps2 * EndSpeedMps * EndSpeedMps))
                / (AccelerationMps2 + DecelerationMps2));
            return ((PeakMps - StartSpeedMps) / AccelerationMps2)
                + ((PeakMps - EndSpeedMps) / DecelerationMps2);
        }

        /// <summary>Computes point-to-point distance.</summary>
        private static Double Distance
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            Double Dx = B.X - A.X;
            Double Dy = B.Y - A.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>Adds seconds to one phase bucket.</summary>
        private static void Add
            (
            Dictionary<ExperimentalLifecyclePlanner.Phase, Double> Output,
            ExperimentalLifecyclePlanner.Phase Phase,
            Double Seconds
            )
        {
            Output.TryGetValue(Phase, out Double Existing);
            Output[Phase] = Existing + Seconds;
        }

        /// <summary>Adds a value to one integer-keyed bucket.</summary>
        private static void Add
            (
            Dictionary<Int32, Double> Output,
            Int32 Key,
            Double Value
            )
        {
            Output.TryGetValue(Key, out Double Existing);
            Output[Key] = Existing + Value;
        }
    }
}
