using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Orders experimental straight work runs using in-field lineups and strict connectors.
    /// </summary>
    public static class ExperimentalRunSequencer
    {
        private const Double COST_TOLERANCE = 1e-9;
        private const Double LINEUP_SHORTFALL_WEIGHT = 4.0;
        private const Double TURN_CLEARANCE_ANGLE_RAD = 0.5 * Math.PI;
        private const Double MANUAL_ASSIST_COST_M = 100000.0;
        private const Double LOADED_RUN_OVERLAP_COST_M = 100000.0;
        private const Double LOADED_RUN_ALIGNMENT_TOLERANCE_DEG = 20.0;
        private const Int32 EXACT_RUN_LIMIT = 5;
        private const Int32 FALLBACK_EXACT_RUN_LIMIT = 7;

        /// <summary>
        /// One oriented work run with the longest available straight entry and exit lineups.
        /// </summary>
        public sealed class SequencedRun
        {
            /// <summary>
            /// Initializes an oriented run and its lineup geometry.
            /// </summary>
            /// <param name="Run">Oriented straight work run.</param>
            /// <param name="EntryPose">Blade-up pose before the work start.</param>
            /// <param name="ExitPose">Blade-up pose after the work end.</param>
            /// <param name="EntryLineupM">Available straight entry distance.</param>
            /// <param name="ExitLineupM">Available straight exit distance.</param>
            /// <param name="StartCoverageShortfallM">Nominal work omitted before the driven start.</param>
            /// <param name="EndCoverageShortfallM">Nominal work omitted after the driven end.</param>
            public SequencedRun
                (
                ExperimentalCoveragePlanner.Run Run,
                ExperimentalConnectorPlanner.Pose EntryPose,
                ExperimentalConnectorPlanner.Pose ExitPose,
                Double EntryLineupM,
                Double ExitLineupM,
                Double StartCoverageShortfallM,
                Double EndCoverageShortfallM
                )
            {
                // Keep work allocation unchanged while attaching drive geometry.
                this.Run = Run;
                this.EntryPose = EntryPose;
                this.ExitPose = ExitPose;
                this.EntryLineupM = EntryLineupM;
                this.ExitLineupM = ExitLineupM;
                this.StartCoverageShortfallM = StartCoverageShortfallM;
                this.EndCoverageShortfallM = EndCoverageShortfallM;
            }

            /// <summary>Gets the oriented straight work run.</summary>
            public ExperimentalCoveragePlanner.Run Run { get; }

            /// <summary>Gets the blade-up entry pose.</summary>
            public ExperimentalConnectorPlanner.Pose EntryPose { get; }

            /// <summary>Gets the blade-up exit pose.</summary>
            public ExperimentalConnectorPlanner.Pose ExitPose { get; }

            /// <summary>Gets available straight distance before work.</summary>
            public Double EntryLineupM { get; }

            /// <summary>Gets available straight distance after work.</summary>
            public Double ExitLineupM { get; }

            /// <summary>Gets nominal work distance intentionally omitted at the driven start.</summary>
            public Double StartCoverageShortfallM { get; }

            /// <summary>Gets nominal work distance intentionally omitted at the driven end.</summary>
            public Double EndCoverageShortfallM { get; }
        }

        /// <summary>
        /// Result of exact run ordering across both drive orientations.
        /// </summary>
        public sealed class Result
        {
            /// <summary>
            /// Initializes an experimental sequence result.
            /// </summary>
            /// <param name="Success">Whether all required strict connectors exist.</param>
            /// <param name="Runs">Ordered oriented runs with lineups.</param>
            /// <param name="Connectors">Strict connectors between successive runs.</param>
            /// <param name="ConnectorDistanceM">Total curved connector distance.</param>
            /// <param name="MinimumLineupM">Shortest available entry or exit lineup.</param>
            /// <param name="FailureReason">Reason no complete sequence was accepted.</param>
            public Result
                (
                Boolean Success,
                List<SequencedRun> Runs,
                List<ExperimentalTransitPlanner.Result> Connectors,
                Double ConnectorDistanceM,
                Double MinimumLineupM,
                String FailureReason
                )
            {
                // Report failure explicitly instead of silently inserting an invalid chord.
                this.Success = Success;
                this.Runs = Runs;
                this.Connectors = Connectors;
                this.ConnectorDistanceM = ConnectorDistanceM;
                this.MinimumLineupM = MinimumLineupM;
                this.FailureReason = FailureReason;
            }

            /// <summary>Gets whether every connector is valid.</summary>
            public Boolean Success { get; }

            /// <summary>Gets ordered oriented runs.</summary>
            public List<SequencedRun> Runs { get; }

            /// <summary>Gets strict connectors between successive runs.</summary>
            public List<ExperimentalTransitPlanner.Result> Connectors { get; }

            /// <summary>Gets total curved connector distance.</summary>
            public Double ConnectorDistanceM { get; }

            /// <summary>Gets the shortest available lineup.</summary>
            public Double MinimumLineupM { get; }

            /// <summary>Gets the failure reason.</summary>
            public String FailureReason { get; }
        }

        /// <summary>
        /// Finds the lowest-cost complete ordering with field-clamped lineups.
        /// </summary>
        /// <param name="Runs">Unordered straight work runs.</param>
        /// <param name="Start">Optional inbound pose before the first lineup.</param>
        /// <param name="End">Optional outbound pose after the final lineup.</param>
        /// <param name="PreferredLineupM">Desired straight distance before and after work.</param>
        /// <param name="MinimumRadiusM">Minimum connector turning radius.</param>
        /// <param name="SampleSpacingM">Maximum validation sample spacing.</param>
        /// <param name="Field">Raw field mask.</param>
        /// <param name="AvoidLoadedRunOverlap">Whether blade-up travel across unfilled work receives a prohibitive cost.</param>
        /// <returns>Complete strict sequence or explicit failure.</returns>
        static public Result Optimize
            (
            List<ExperimentalCoveragePlanner.Run> Runs,
            ExperimentalConnectorPlanner.Pose? Start,
            ExperimentalConnectorPlanner.Pose? End,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AvoidLoadedRunOverlap = false
            )
        {
            if (Runs == null) throw new ArgumentNullException(nameof(Runs));
            if (PreferredLineupM < 0.0) throw new ArgumentOutOfRangeException(nameof(PreferredLineupM));
            if (Runs.Count == 0)
            {
                return new Result(true, new List<SequencedRun>(),
                    new List<ExperimentalTransitPlanner.Result>(), 0.0, 0.0, String.Empty);
            }
            SequencedRun[,] Options = new SequencedRun[Runs.Count, 2];
            for (Int32 RunIndex = 0; RunIndex < Runs.Count; RunIndex += 1)
            {
                Options[RunIndex, 0] = Prepare(
                    Runs[RunIndex], PreferredLineupM, MinimumRadiusM, SampleSpacingM, Field);
                Options[RunIndex, 1] = Prepare(
                    Reverse(Runs[RunIndex]), PreferredLineupM, MinimumRadiusM, SampleSpacingM, Field);
            }
            if (Runs.Count > EXACT_RUN_LIMIT)
            {
                Result Greedy = OptimizeGreedy(
                    Options,
                    Start,
                    End,
                    PreferredLineupM,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    AvoidLoadedRunOverlap);
                if (Greedy.Success || (Runs.Count > FALLBACK_EXACT_RUN_LIMIT))
                {
                    return Greedy;
                }
            }

            Int32 StateCount = 1 << Runs.Count;
            Double[,,] Costs = new Double[StateCount, Runs.Count, 2];
            Int32[,,] PreviousIndices = new Int32[StateCount, Runs.Count, 2];
            Int32[,,] PreviousOrientations = new Int32[StateCount, Runs.Count, 2];
            InitializeTables(Costs, PreviousIndices, PreviousOrientations);
            Boolean HasDirectLoadedEntry = AvoidLoadedRunOverlap
                && Start.HasValue
                && HasAnyDirectLoadedEntry(
                    Options,
                    Start.Value,
                    PreferredLineupM,
                    Field.GridSizeM);

            for (Int32 RunIndex = 0; RunIndex < Runs.Count; RunIndex += 1)
            {
                for (Int32 Orientation = 0; Orientation < 2; Orientation += 1)
                {
                    SequencedRun Option = Options[RunIndex, Orientation];
                    Boolean DirectLoadedEntry = HasDirectLoadedEntry
                        && CanUseDirectLoadedEntry(
                            Start!.Value,
                            Option,
                            PreferredLineupM,
                            Field.GridSizeM);
                    if (HasDirectLoadedEntry && !DirectLoadedEntry) continue;
                    SequencedRun InitialOption = DirectLoadedEntry
                        ? WithDirectEntry(Option)
                        : Option;
                    Double Cost = LineupPenalty(Option, PreferredLineupM);
                    if (Start.HasValue)
                    {
                        Cost += ConnectorCost(
                            Start.Value,
                            InitialOption.EntryPose,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            AvoidLoadedRunOverlap ? Options : null,
                            0,
                            InitialOption);
                    }
                    Costs[1 << RunIndex, RunIndex, Orientation] = Cost;
                }
            }

            for (Int32 Mask = 1; Mask < StateCount; Mask += 1)
            {
                ExpandState(Mask, Runs.Count, Options, Costs, PreviousIndices,
                    PreviousOrientations, PreferredLineupM, MinimumRadiusM, SampleSpacingM, Field,
                    AvoidLoadedRunOverlap);
            }

            (Int32 Index, Int32 Orientation) Best = FindBestEnd(
                Costs, Options, End, MinimumRadiusM, SampleSpacingM, Field);
            if (Best.Index < 0)
            {
                return Failure("No ordering connects every lineup without leaving the field.");
            }

            List<SequencedRun> Ordered = Reconstruct(
                StateCount - 1, Best.Index, Best.Orientation, Options,
                PreviousIndices, PreviousOrientations);
            if (HasDirectLoadedEntry
                && CanUseDirectLoadedEntry(
                    Start!.Value,
                    Ordered[0],
                    PreferredLineupM,
                    Field.GridSizeM))
            {
                Ordered[0] = WithDirectEntry(Ordered[0]);
            }
            return BuildResult(Ordered, MinimumRadiusM, SampleSpacingM, Field);
        }

        /// <summary>
        /// Builds a deterministic validated sequence when exact subset search would be excessive.
        /// </summary>
        private static Result OptimizeGreedy
            (
            SequencedRun[,] Options,
            ExperimentalConnectorPlanner.Pose? Start,
            ExperimentalConnectorPlanner.Pose? End,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AvoidLoadedRunOverlap
            )
        {
            Int32 RunCount = Options.GetLength(0);
            if (TryGreedyOrder(
                Options, Start, End, PreferredLineupM, MinimumRadiusM,
                SampleSpacingM, Field, AvoidLoadedRunOverlap, -1, -1,
                out List<SequencedRun> Ordered, out Double IgnoredCost))
            {
                return BuildResult(Ordered, MinimumRadiusM, SampleSpacingM, Field);
            }

            List<SequencedRun>? BestOrdered = null;
            Double BestTotalCost = Double.PositiveInfinity;
            for (Int32 FirstIndex = 0; FirstIndex < RunCount; FirstIndex += 1)
            {
                for (Int32 FirstOrientation = 0; FirstOrientation < 2; FirstOrientation += 1)
                {
                    if (!TryGreedyOrder(
                        Options, Start, End, PreferredLineupM, MinimumRadiusM,
                        SampleSpacingM, Field, AvoidLoadedRunOverlap,
                        FirstIndex, FirstOrientation,
                        out List<SequencedRun> Candidate, out Double CandidateCost))
                    {
                        continue;
                    }
                    if (CandidateCost + COST_TOLERANCE < BestTotalCost)
                    {
                        BestTotalCost = CandidateCost;
                        BestOrdered = Candidate;
                    }
                }
            }
            return BestOrdered == null
                ? Failure("Greedy sequencing could not reach every lineup in-field.")
                : BuildResult(BestOrdered, MinimumRadiusM, SampleSpacingM, Field);
        }

        /// <summary>Builds one deterministic greedy order with an optional forced first run.</summary>
        private static Boolean TryGreedyOrder
            (
            SequencedRun[,] Options,
            ExperimentalConnectorPlanner.Pose? Start,
            ExperimentalConnectorPlanner.Pose? End,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AvoidLoadedRunOverlap,
            Int32 ForcedFirstIndex,
            Int32 ForcedFirstOrientation,
            out List<SequencedRun> Ordered,
            out Double TotalCost
            )
        {
            Int32 RunCount = Options.GetLength(0);
            Boolean[] Used = new Boolean[RunCount];
            Ordered = new List<SequencedRun>();
            TotalCost = 0.0;
            Boolean HasDirectLoadedEntry = AvoidLoadedRunOverlap
                && Start.HasValue
                && HasAnyDirectLoadedEntry(
                    Options,
                    Start.Value,
                    PreferredLineupM,
                    Field.GridSizeM);
            for (Int32 Position = 0; Position < RunCount; Position += 1)
            {
                Double BestCost = Double.PositiveInfinity;
                Int32 BestIndex = -1;
                Int32 BestOrientation = -1;
                for (Int32 RunIndex = 0; RunIndex < RunCount; RunIndex += 1)
                {
                    if (Used[RunIndex]) continue;
                    for (Int32 Orientation = 0; Orientation < 2; Orientation += 1)
                    {
                        if ((Position == 0) && (ForcedFirstIndex >= 0)
                            && ((RunIndex != ForcedFirstIndex)
                                || (Orientation != ForcedFirstOrientation)))
                        {
                            continue;
                        }
                        SequencedRun Candidate = Options[RunIndex, Orientation];
                        Boolean DirectLoadedEntry = (Position == 0)
                            && HasDirectLoadedEntry
                            && CanUseDirectLoadedEntry(
                                Start!.Value,
                                Candidate,
                                PreferredLineupM,
                                Field.GridSizeM);
                        if ((Position == 0)
                            && HasDirectLoadedEntry
                            && !DirectLoadedEntry)
                        {
                            continue;
                        }
                        SequencedRun EntryCandidate = DirectLoadedEntry
                            ? WithDirectEntry(Candidate)
                            : Candidate;
                        Double Cost = LineupPenalty(Candidate, PreferredLineupM);
                        if (Ordered.Count > 0)
                        {
                            Cost += ConnectorCost(
                                Ordered[Ordered.Count - 1].ExitPose,
                                Candidate.EntryPose,
                                MinimumRadiusM,
                                SampleSpacingM,
                                Field,
                                AvoidLoadedRunOverlap ? Options : null,
                                UsedMask(Used),
                                Candidate);
                        }
                        else if (Start.HasValue)
                        {
                            Cost += ConnectorCost(
                                Start.Value,
                                EntryCandidate.EntryPose,
                                MinimumRadiusM,
                                SampleSpacingM,
                                Field,
                                AvoidLoadedRunOverlap ? Options : null,
                                UsedMask(Used),
                                EntryCandidate);
                        }
                        if ((Position == RunCount - 1) && End.HasValue
                            && Double.IsFinite(Cost))
                        {
                            Cost += ConnectorCost(
                                Candidate.ExitPose,
                                End.Value,
                                MinimumRadiusM,
                                SampleSpacingM,
                                Field);
                        }
                        if (Cost + COST_TOLERANCE < BestCost)
                        {
                            BestCost = Cost;
                            BestIndex = RunIndex;
                            BestOrientation = Orientation;
                        }
                    }
                }
                if ((BestIndex < 0) || !Double.IsFinite(BestCost))
                {
                    Ordered.Clear();
                    TotalCost = Double.PositiveInfinity;
                    return false;
                }
                Used[BestIndex] = true;
                SequencedRun Selected = Options[BestIndex, BestOrientation];
                if ((Position == 0)
                    && HasDirectLoadedEntry
                    && CanUseDirectLoadedEntry(
                        Start!.Value,
                        Selected,
                        PreferredLineupM,
                        Field.GridSizeM))
                {
                    Selected = WithDirectEntry(Selected);
                }
                Ordered.Add(Selected);
                TotalCost += BestCost;
            }
            return true;
        }

        /// <summary>
        /// Attaches the longest first-contiguous in-field lineups to one oriented run.
        /// </summary>
        static public SequencedRun Prepare
            (
            ExperimentalCoveragePlanner.Run Run,
            Double PreferredLineupM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            return Prepare(Run, PreferredLineupM, 0.0, SampleSpacingM, Field);
        }

        /// <summary>
        /// Attaches lineups and retreats soft work endpoints when field-edge clearance prevents a turn.
        /// </summary>
        static public SequencedRun Prepare
            (
            ExperimentalCoveragePlanner.Run Run,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            ExperimentalCoveragePlanner.Run DriveRun = MinimumRadiusM > 0.0
                ? RetreatForTurnClearance(
                    Run, PreferredLineupM, MinimumRadiusM, SampleSpacingM, Field)
                : Run;
            Double HeadingRad = DriveRun.HeadingDeg * Math.PI / 180.0;
            Double UnitX = Math.Sin(HeadingRad);
            Double UnitY = Math.Cos(HeadingRad);
            Double EntryM = AvailableStraight(
                DriveRun.StartXy, -UnitX, -UnitY, PreferredLineupM, SampleSpacingM, Field);
            Double ExitM = AvailableStraight(
                DriveRun.EndXy, UnitX, UnitY, PreferredLineupM, SampleSpacingM, Field);
            return new SequencedRun(
                DriveRun,
                new ExperimentalConnectorPlanner.Pose(
                    DriveRun.StartXy.X - (UnitX * EntryM),
                    DriveRun.StartXy.Y - (UnitY * EntryM),
                    DriveRun.HeadingDeg),
                new ExperimentalConnectorPlanner.Pose(
                    DriveRun.EndXy.X + (UnitX * ExitM),
                    DriveRun.EndXy.Y + (UnitY * ExitM),
                    DriveRun.HeadingDeg),
                EntryM,
                ExitM,
                Distance(Run.StartXy, DriveRun.StartXy),
                Distance(Run.EndXy, DriveRun.EndXy));
        }

        /// <summary>
        /// Moves work endpoints inward only as far as required to begin an in-field quarter turn.
        /// </summary>
        private static ExperimentalCoveragePlanner.Run RetreatForTurnClearance
            (
            ExperimentalCoveragePlanner.Run Run,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if (Run.LengthM <= Field.GridSizeM) return Run;
            Double HeadingRad = Run.HeadingDeg * Math.PI / 180.0;
            Double UnitX = Math.Sin(HeadingRad);
            Double UnitY = Math.Cos(HeadingRad);
            Double StepM = Math.Min(SampleSpacingM, Field.GridSizeM * 0.5);
            Double MinimumWorkM = Math.Max(Field.GridSizeM * 0.5, StepM);
            Double MaximumTotalRetreatM = Math.Max(0.0, Run.LengthM - MinimumWorkM);

            Double StartRetreatM = RequiredRetreat(
                Run.StartXy,
                UnitX,
                UnitY,
                true,
                Run.HeadingDeg,
                MaximumTotalRetreatM,
                PreferredLineupM,
                MinimumRadiusM,
                SampleSpacingM,
                Field);
            Double EndRetreatM = RequiredRetreat(
                Run.EndXy,
                -UnitX,
                -UnitY,
                false,
                Run.HeadingDeg,
                Math.Max(0.0, MaximumTotalRetreatM - StartRetreatM),
                PreferredLineupM,
                MinimumRadiusM,
                SampleSpacingM,
                Field);
            if ((StartRetreatM <= COST_TOLERANCE) && (EndRetreatM <= COST_TOLERANCE))
            {
                return Run;
            }

            return new ExperimentalCoveragePlanner.Run(
                Run.Cells,
                (
                    Run.StartXy.X + (UnitX * StartRetreatM),
                    Run.StartXy.Y + (UnitY * StartRetreatM)),
                (
                    Run.EndXy.X - (UnitX * EndRetreatM),
                    Run.EndXy.Y - (UnitY * EndRetreatM)),
                Run.HeadingDeg,
                Run.MaximumLateralErrorM);
        }

        /// <summary>Finds the first inward endpoint whose available lineup can begin a turn.</summary>
        private static Double RequiredRetreat
            (
            (Double X, Double Y) Original,
            Double InwardUnitX,
            Double InwardUnitY,
            Boolean Entry,
            Double HeadingDeg,
            Double MaximumRetreatM,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            Double StepM = Math.Min(SampleSpacingM, Field.GridSizeM * 0.5);
            for (Double RetreatM = 0.0;
                RetreatM <= MaximumRetreatM + COST_TOLERANCE;
                RetreatM += StepM)
            {
                Double CandidateM = Math.Min(RetreatM, MaximumRetreatM);
                (Double X, Double Y) WorkPoint = (
                    Original.X + (InwardUnitX * CandidateM),
                    Original.Y + (InwardUnitY * CandidateM));
                Double HeadingRad = HeadingDeg * Math.PI / 180.0;
                Double UnitX = Math.Sin(HeadingRad);
                Double UnitY = Math.Cos(HeadingRad);
                Double LineupM = Entry
                    ? AvailableStraight(
                        WorkPoint, -UnitX, -UnitY,
                        PreferredLineupM, SampleSpacingM, Field)
                    : AvailableStraight(
                        WorkPoint, UnitX, UnitY,
                        PreferredLineupM, SampleSpacingM, Field);
                ExperimentalConnectorPlanner.Pose TurnPose = Entry
                    ? new ExperimentalConnectorPlanner.Pose(
                        WorkPoint.X - (UnitX * LineupM),
                        WorkPoint.Y - (UnitY * LineupM),
                        HeadingDeg + 180.0)
                    : new ExperimentalConnectorPlanner.Pose(
                        WorkPoint.X + (UnitX * LineupM),
                        WorkPoint.Y + (UnitY * LineupM),
                        HeadingDeg);
                if (CanBeginTurn(
                    TurnPose, MinimumRadiusM, SampleSpacingM, Field))
                {
                    return CandidateM;
                }
                if (CandidateM >= MaximumRetreatM - COST_TOLERANCE) break;
            }
            return 0.0;
        }

        /// <summary>Checks whether either minimum-radius turn direction stays in field for 90 degrees.</summary>
        private static Boolean CanBeginTurn
            (
            ExperimentalConnectorPlanner.Pose Pose,
            Double RadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            Double HeadingRad = Pose.HeadingDeg * Math.PI / 180.0;
            Double UnitX = Math.Sin(HeadingRad);
            Double UnitY = Math.Cos(HeadingRad);
            Double LeftX = -UnitY;
            Double LeftY = UnitX;
            Int32 Samples = Math.Max(
                2,
                (Int32)Math.Ceiling(
                    (TURN_CLEARANCE_ANGLE_RAD * RadiusM) / SampleSpacingM));
            foreach (Double Direction in new[] { -1.0, 1.0 })
            {
                Double CenterX = Pose.X + (Direction * RadiusM * LeftX);
                Double CenterY = Pose.Y + (Direction * RadiusM * LeftY);
                Double RadialX = Pose.X - CenterX;
                Double RadialY = Pose.Y - CenterY;
                Boolean Inside = true;
                for (Int32 Index = 0; Index <= Samples; Index += 1)
                {
                    Double Angle = Direction * TURN_CLEARANCE_ANGLE_RAD * Index / Samples;
                    Double Cos = Math.Cos(Angle);
                    Double Sin = Math.Sin(Angle);
                    Double X = CenterX + (RadialX * Cos) - (RadialY * Sin);
                    Double Y = CenterY + (RadialX * Sin) + (RadialY * Cos);
                    if (!Field.ContainsWorldXy(X, Y))
                    {
                        Inside = false;
                        break;
                    }
                }
                if (Inside) return true;
            }
            return false;
        }

        /// <summary>
        /// Expands all feasible transitions out of one dynamic-programming mask.
        /// </summary>
        private static void ExpandState
            (
            Int32 Mask,
            Int32 RunCount,
            SequencedRun[,] Options,
            Double[,,] Costs,
            Int32[,,] PreviousIndices,
            Int32[,,] PreviousOrientations,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AvoidLoadedRunOverlap
            )
        {
            for (Int32 LastIndex = 0; LastIndex < RunCount; LastIndex += 1)
            {
                for (Int32 LastOrientation = 0; LastOrientation < 2; LastOrientation += 1)
                {
                    Double CurrentCost = Costs[Mask, LastIndex, LastOrientation];
                    if (!Double.IsFinite(CurrentCost)) continue;
                    for (Int32 NextIndex = 0; NextIndex < RunCount; NextIndex += 1)
                    {
                        if ((Mask & (1 << NextIndex)) != 0) continue;
                        for (Int32 NextOrientation = 0; NextOrientation < 2; NextOrientation += 1)
                        {
                            SequencedRun Next = Options[NextIndex, NextOrientation];
                            Double TransitionM = ConnectorCost(
                                Options[LastIndex, LastOrientation].ExitPose,
                                Next.EntryPose,
                                MinimumRadiusM,
                                SampleSpacingM,
                                Field,
                                AvoidLoadedRunOverlap ? Options : null,
                                Mask,
                                Next);
                            if (!Double.IsFinite(TransitionM)) continue;
                            Int32 NextMask = Mask | (1 << NextIndex);
                            Double CandidateCost = CurrentCost + TransitionM
                                + LineupPenalty(Next, PreferredLineupM);
                            if (CandidateCost + COST_TOLERANCE
                                < Costs[NextMask, NextIndex, NextOrientation])
                            {
                                Costs[NextMask, NextIndex, NextOrientation] = CandidateCost;
                                PreviousIndices[NextMask, NextIndex, NextOrientation] = LastIndex;
                                PreviousOrientations[NextMask, NextIndex, NextOrientation] = LastOrientation;
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Selects the cheapest complete state, including an optional outbound connector.
        /// </summary>
        private static (Int32 Index, Int32 Orientation) FindBestEnd
            (
            Double[,,] Costs,
            SequencedRun[,] Options,
            ExperimentalConnectorPlanner.Pose? End,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            Int32 FullMask = Costs.GetLength(0) - 1;
            Double BestCost = Double.PositiveInfinity;
            (Int32 Index, Int32 Orientation) Best = (-1, -1);
            for (Int32 RunIndex = 0; RunIndex < Costs.GetLength(1); RunIndex += 1)
            {
                for (Int32 Orientation = 0; Orientation < 2; Orientation += 1)
                {
                    Double CandidateCost = Costs[FullMask, RunIndex, Orientation];
                    if (End.HasValue && Double.IsFinite(CandidateCost))
                    {
                        CandidateCost += ConnectorCost(
                            Options[RunIndex, Orientation].ExitPose,
                            End.Value,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field);
                    }
                    if (CandidateCost + COST_TOLERANCE < BestCost)
                    {
                        BestCost = CandidateCost;
                        Best = (RunIndex, Orientation);
                    }
                }
            }
            return Best;
        }

        /// <summary>
        /// Reconstructs one complete ordered sequence.
        /// </summary>
        private static List<SequencedRun> Reconstruct
            (
            Int32 Mask,
            Int32 Index,
            Int32 Orientation,
            SequencedRun[,] Options,
            Int32[,,] PreviousIndices,
            Int32[,,] PreviousOrientations
            )
        {
            List<SequencedRun> ReverseOrder = new List<SequencedRun>();
            while (Index >= 0)
            {
                ReverseOrder.Add(Options[Index, Orientation]);
                Int32 PreviousIndex = PreviousIndices[Mask, Index, Orientation];
                Int32 PreviousOrientation = PreviousOrientations[Mask, Index, Orientation];
                Mask &= ~(1 << Index);
                Index = PreviousIndex;
                Orientation = PreviousOrientation;
            }
            ReverseOrder.Reverse();
            return ReverseOrder;
        }

        /// <summary>
        /// Recomputes strict connectors for the selected sequence and summarizes its geometry.
        /// </summary>
        private static Result BuildResult
            (
            List<SequencedRun> Runs,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            List<ExperimentalTransitPlanner.Result> Connectors =
                new List<ExperimentalTransitPlanner.Result>();
            Double ConnectorDistanceM = 0.0;
            Double MinimumLineupM = Double.PositiveInfinity;
            foreach (SequencedRun Run in Runs)
            {
                MinimumLineupM = Math.Min(MinimumLineupM,
                    Math.Min(Run.EntryLineupM, Run.ExitLineupM));
            }
            for (Int32 Index = 1; Index < Runs.Count; Index += 1)
            {
                ExperimentalTransitPlanner.Result Connector = ExperimentalTransitPlanner.Connect(
                    Runs[Index - 1].ExitPose,
                    Runs[Index].EntryPose,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field);
                if (!Connector.Success)
                {
                    return Failure("Selected sequence failed strict connector reconstruction.");
                }
                Connectors.Add(Connector);
                ConnectorDistanceM += Connector.LengthM;
            }
            return new Result(true, Runs, Connectors, ConnectorDistanceM,
                Double.IsFinite(MinimumLineupM) ? MinimumLineupM : 0.0, String.Empty);
        }

        /// <summary>
        /// Finds the longest contiguous straight extension that remains inside the field.
        /// </summary>
        private static Double AvailableStraight
            (
            (Double X, Double Y) Origin,
            Double UnitX,
            Double UnitY,
            Double PreferredM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if ((PreferredM <= 0.0) || !Field.ContainsWorldXy(Origin.X, Origin.Y)) return 0.0;
            Double StepM = Math.Min(SampleSpacingM, Field.GridSizeM * 0.5);
            Double GoodM = 0.0;
            while (GoodM + COST_TOLERANCE < PreferredM)
            {
                Double CandidateM = Math.Min(PreferredM, GoodM + StepM);
                if (!Field.ContainsWorldXy(
                    Origin.X + (UnitX * CandidateM), Origin.Y + (UnitY * CandidateM)))
                {
                    break;
                }
                GoodM = CandidateM;
            }
            return GoodM;
        }

        /// <summary>
        /// Computes a strict transition cost or positive infinity.
        /// </summary>
        private static Double ConnectorCost
            (
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            SequencedRun[,]? UnfilledOptions = null,
            Int32 CompletedMask = 0,
            SequencedRun? FollowingRun = null
            )
        {
            ExperimentalTransitPlanner.Result Connector = ExperimentalTransitPlanner.Connect(
                Start, End, MinimumRadiusM, SampleSpacingM, Field);
            if (!Connector.Success) return Double.PositiveInfinity;
            Double CostM = Connector.LengthM
                + (Connector.OperatorAlignmentRequired ? MANUAL_ASSIST_COST_M : 0.0);
            if (UnfilledOptions != null)
            {
                List<(Double X, Double Y)> LoadedBladeUpPoints =
                    new List<(Double X, Double Y)>(Connector.Points);
                if ((FollowingRun != null)
                    && (Distance(
                        LoadedBladeUpPoints[LoadedBladeUpPoints.Count - 1],
                        FollowingRun.Run.StartXy) > COST_TOLERANCE))
                {
                    LoadedBladeUpPoints.Add(FollowingRun.Run.StartXy);
                }
                Double OverlapM = AlignedUnfilledOverlapM(
                    LoadedBladeUpPoints,
                    UnfilledOptions,
                    CompletedMask,
                    Field.GridSizeM);
                if (OverlapM >= 0.5 * Field.GridSizeM)
                {
                    CostM += LOADED_RUN_OVERLAP_COST_M + OverlapM;
                }
            }
            return CostM;
        }

        /// <summary>Converts greedy used-state flags into the exact-search bit mask.</summary>
        private static Int32 UsedMask
            (
            Boolean[] Used
            )
        {
            Int32 Mask = 0;
            for (Int32 Index = 0; Index < Used.Length; Index += 1)
            {
                if (Used[Index]) Mask |= 1 << Index;
            }
            return Mask;
        }

        /// <summary>Returns true when any oriented run can begin directly from the loaded approach.</summary>
        private static Boolean HasAnyDirectLoadedEntry
            (
            SequencedRun[,] Options,
            ExperimentalConnectorPlanner.Pose Inbound,
            Double PreferredLineupM,
            Double CorridorWidthM
            )
        {
            for (Int32 RunIndex = 0; RunIndex < Options.GetLength(0); RunIndex += 1)
            {
                for (Int32 Orientation = 0;
                    Orientation < Options.GetLength(1);
                    Orientation += 1)
                {
                    if (CanUseDirectLoadedEntry(
                        Inbound,
                        Options[RunIndex, Orientation],
                        PreferredLineupM,
                        CorridorWidthM))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        /// <summary>
        /// Returns true when an oriented work start is directly ahead of an aligned
        /// loaded tractor and close enough to use the inbound path as its lineup.
        /// </summary>
        private static Boolean CanUseDirectLoadedEntry
            (
            ExperimentalConnectorPlanner.Pose Inbound,
            SequencedRun Run,
            Double PreferredLineupM,
            Double CorridorWidthM
            )
        {
            if (HeadingDifferenceDeg(
                Inbound.HeadingDeg,
                Run.Run.HeadingDeg) > LOADED_RUN_ALIGNMENT_TOLERANCE_DEG)
            {
                return false;
            }
            Double HeadingRad = Inbound.HeadingDeg * Math.PI / 180.0;
            Double UnitX = Math.Sin(HeadingRad);
            Double UnitY = Math.Cos(HeadingRad);
            Double Dx = Run.Run.StartXy.X - Inbound.X;
            Double Dy = Run.Run.StartXy.Y - Inbound.Y;
            Double ForwardM = (Dx * UnitX) + (Dy * UnitY);
            Double LateralM = Math.Abs((-Dx * UnitY) + (Dy * UnitX));
            return (ForwardM >= -0.25 * CorridorWidthM)
                && (ForwardM <= PreferredLineupM + CorridorWidthM)
                && (LateralM <= CorridorWidthM);
        }

        /// <summary>
        /// Uses the already aligned inbound path as the first run's shortened lineup.
        /// </summary>
        private static SequencedRun WithDirectEntry
            (
            SequencedRun Run
            ) => new SequencedRun(
                Run.Run,
                new ExperimentalConnectorPlanner.Pose(
                    Run.Run.StartXy.X,
                    Run.Run.StartXy.Y,
                    Run.Run.HeadingDeg),
                Run.ExitPose,
                0.0,
                Run.ExitLineupM,
                Run.StartCoverageShortfallM,
                Run.EndCoverageShortfallM);

        /// <summary>
        /// Measures blade-up connector travel along work that has not yet been filled.
        /// Only orientation zero is inspected because both options own the same work line.
        /// </summary>
        private static Double AlignedUnfilledOverlapM
            (
            List<(Double X, Double Y)> ConnectorPoints,
            SequencedRun[,] Options,
            Int32 CompletedMask,
            Double CorridorWidthM
            )
        {
            Double OverlapM = 0.0;
            for (Int32 PointIndex = 1;
                PointIndex < ConnectorPoints.Count;
                PointIndex += 1)
            {
                (Double X, Double Y) Start = ConnectorPoints[PointIndex - 1];
                (Double X, Double Y) End = ConnectorPoints[PointIndex];
                Double SegmentLengthM = Distance(Start, End);
                if (SegmentLengthM <= COST_TOLERANCE) continue;
                Double SegmentHeadingDeg = BearingDeg(Start, End);
                (Double X, Double Y) Midpoint = (
                    0.5 * (Start.X + End.X),
                    0.5 * (Start.Y + End.Y));
                for (Int32 RunIndex = 0;
                    RunIndex < Options.GetLength(0);
                    RunIndex += 1)
                {
                    if ((CompletedMask & (1 << RunIndex)) != 0) continue;
                    ExperimentalCoveragePlanner.Run Run = Options[RunIndex, 0].Run;
                    Double AlignmentDeg = HeadingDifferenceDeg(
                        SegmentHeadingDeg, Run.HeadingDeg);
                    AlignmentDeg = Math.Min(AlignmentDeg, 180.0 - AlignmentDeg);
                    if (AlignmentDeg > LOADED_RUN_ALIGNMENT_TOLERANCE_DEG) continue;
                    if (PointToSegmentDistanceM(
                            Midpoint,
                            Run.StartXy,
                            Run.EndXy)
                        <= CorridorWidthM)
                    {
                        OverlapM += SegmentLengthM;
                        break;
                    }
                }
            }
            return OverlapM;
        }

        /// <summary>Returns the shortest point-to-segment distance.</summary>
        private static Double PointToSegmentDistanceM
            (
            (Double X, Double Y) Point,
            (Double X, Double Y) Start,
            (Double X, Double Y) End
            )
        {
            Double Dx = End.X - Start.X;
            Double Dy = End.Y - Start.Y;
            Double LengthSquaredM = (Dx * Dx) + (Dy * Dy);
            if (LengthSquaredM <= 1e-12) return Distance(Point, Start);
            Double Fraction = ((Point.X - Start.X) * Dx
                + (Point.Y - Start.Y) * Dy) / LengthSquaredM;
            Fraction = Math.Max(0.0, Math.Min(1.0, Fraction));
            return Distance(Point, (
                Start.X + (Fraction * Dx),
                Start.Y + (Fraction * Dy)));
        }

        /// <summary>Returns the navigation bearing used by work-run headings.</summary>
        private static Double BearingDeg
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End
            )
        {
            Double Bearing = Math.Atan2(
                End.X - Start.X,
                End.Y - Start.Y) * 180.0 / Math.PI;
            return (Bearing + 360.0) % 360.0;
        }

        /// <summary>Returns the unsigned shortest heading difference.</summary>
        private static Double HeadingDifferenceDeg
            (
            Double First,
            Double Second
            ) => Math.Abs(((First - Second + 540.0) % 360.0) - 180.0);

        /// <summary>Computes planar distance between two work points.</summary>
        private static Double Distance
            (
            (Double X, Double Y) First,
            (Double X, Double Y) Second
            )
        {
            Double Dx = Second.X - First.X;
            Double Dy = Second.Y - First.Y;
            return Math.Sqrt((Dx * Dx) + (Dy * Dy));
        }

        /// <summary>
        /// Penalizes orientations which cannot provide the preferred lineups.
        /// </summary>
        private static Double LineupPenalty
            (
            SequencedRun Run,
            Double PreferredLineupM
            )
        {
            return LINEUP_SHORTFALL_WEIGHT
                * ((PreferredLineupM - Run.EntryLineupM) + (PreferredLineupM - Run.ExitLineupM));
        }

        /// <summary>
        /// Reverses one run without changing its allocated cells or geometry.
        /// </summary>
        private static ExperimentalCoveragePlanner.Run Reverse
            (
            ExperimentalCoveragePlanner.Run Input
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Input.Cells);
            Cells.Reverse();
            return new ExperimentalCoveragePlanner.Run(
                Cells,
                Input.EndXy,
                Input.StartXy,
                (Input.HeadingDeg + 180.0) % 360.0,
                Input.MaximumLateralErrorM);
        }

        /// <summary>
        /// Initializes dynamic-programming tables with unreachable sentinels.
        /// </summary>
        private static void InitializeTables
            (
            Double[,,] Costs,
            Int32[,,] PreviousIndices,
            Int32[,,] PreviousOrientations
            )
        {
            for (Int32 Mask = 0; Mask < Costs.GetLength(0); Mask += 1)
            {
                for (Int32 Index = 0; Index < Costs.GetLength(1); Index += 1)
                {
                    for (Int32 Orientation = 0; Orientation < 2; Orientation += 1)
                    {
                        Costs[Mask, Index, Orientation] = Double.PositiveInfinity;
                        PreviousIndices[Mask, Index, Orientation] = -1;
                        PreviousOrientations[Mask, Index, Orientation] = -1;
                    }
                }
            }
        }

        /// <summary>
        /// Creates an empty explicit failure result.
        /// </summary>
        private static Result Failure
            (
            String Reason
            )
        {
            return new Result(false, new List<SequencedRun>(),
                new List<ExperimentalTransitPlanner.Result>(), 0.0, 0.0, Reason);
        }
    }
}
