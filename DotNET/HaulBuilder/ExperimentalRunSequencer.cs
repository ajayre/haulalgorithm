using System;
using System.Collections.Generic;
using System.Linq;

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
        private const Double COVERAGE_SHORTFALL_WEIGHT = 8.0;
        private const Int32 EXACT_RUN_LIMIT = 5;
        private const Int32 FALLBACK_EXACT_RUN_LIMIT = 7;
        private const Int32 FLEXIBLE_EXACT_RUN_LIMIT = 5;
        private const Int32 FLEXIBLE_OPTIONS_PER_ORIENTATION = 6;
        private const Int32 FLEXIBLE_BEAM_WIDTH = 24;
        private const Int32 FLEXIBLE_EXACT_RETRY_RUN_LIMIT = 10;
        private const Int32 TARGETED_DETOUR_ATTEMPT_LIMIT_PER_RUN = 96;
        private const Int32 TARGETED_DETOUR_SUCCESS_LIMIT_PER_RUN = 24;
        private const Int32 TARGETED_DETOUR_OPTIONS_PER_DESTINATION = 12;

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
            /// <param name="InfeasibleRunIndices">Run indices proven to have no useful forward exit.</param>
            public Result
                (
                Boolean Success,
                List<SequencedRun> Runs,
                List<ExperimentalTransitPlanner.Result> Connectors,
                Double ConnectorDistanceM,
                Double MinimumLineupM,
                String FailureReason,
                IReadOnlyList<Int32>? InfeasibleRunIndices = null
                )
            {
                // Report failure explicitly instead of silently inserting an invalid chord.
                this.Success = Success;
                this.Runs = Runs;
                this.Connectors = Connectors;
                this.ConnectorDistanceM = ConnectorDistanceM;
                this.MinimumLineupM = MinimumLineupM;
                this.FailureReason = FailureReason;
                this.InfeasibleRunIndices = InfeasibleRunIndices
                    ?? Array.Empty<Int32>();
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

            /// <summary>Gets run indices proven to have no useful forward exit.</summary>
            public IReadOnlyList<Int32> InfeasibleRunIndices { get; }
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
        /// Retries an otherwise infeasible exact sequence with bounded work-end retreats.
        /// A retreated endpoint starts or finishes its connector directly at the work
        /// point, allowing the turn to begin before a field edge while recording the
        /// resulting nominal coverage shortfall.
        /// </summary>
        static public Result OptimizeFlexibleEndpoints
            (
            List<ExperimentalCoveragePlanner.Run> Runs,
            ExperimentalConnectorPlanner.Pose? Start,
            ExperimentalConnectorPlanner.Pose? End,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AllowForwardDetour = false
            )
        {
            List<List<SequencedRun>> Options = new List<List<SequencedRun>>(Runs.Count);
            Int32 MaximumOptionCount = 0;
            for (Int32 RunIndex = 0; RunIndex < Runs.Count; RunIndex += 1)
            {
                List<SequencedRun> ForwardOptions = new List<SequencedRun>();
                AppendFlexibleOptions(
                    ForwardOptions,
                    Prepare(
                        Runs[RunIndex], PreferredLineupM,
                        MinimumRadiusM, SampleSpacingM, Field),
                    MinimumRadiusM,
                    Field);
                List<SequencedRun> ReverseOptions = new List<SequencedRun>();
                AppendFlexibleOptions(
                    ReverseOptions,
                    Prepare(
                        Reverse(Runs[RunIndex]), PreferredLineupM,
                        MinimumRadiusM, SampleSpacingM, Field),
                    MinimumRadiusM,
                    Field);
                List<SequencedRun> RunOptions;
                if (Runs.Count > FLEXIBLE_EXACT_RUN_LIMIT)
                {
                    RunOptions = SelectBoundedFlexibleOptions(
                            ForwardOptions, PreferredLineupM)
                        .Concat(SelectBoundedFlexibleOptions(
                            ReverseOptions, PreferredLineupM))
                        .ToList();
                }
                else
                {
                    RunOptions = ForwardOptions.Concat(ReverseOptions).ToList();
                }
                Options.Add(RunOptions);
                MaximumOptionCount = Math.Max(MaximumOptionCount, RunOptions.Count);
            }

            if (Runs.Count > FLEXIBLE_EXACT_RUN_LIMIT)
            {
                return OptimizeFlexibleGreedy(
                    Options,
                    Start,
                    End,
                    PreferredLineupM,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    AllowForwardDetour);
            }

            Int32 StateCount = 1 << Runs.Count;
            Double[,,] Costs = new Double[StateCount, Runs.Count, MaximumOptionCount];
            Int32[,,] PreviousIndices = new Int32[StateCount, Runs.Count, MaximumOptionCount];
            Int32[,,] PreviousOptions = new Int32[StateCount, Runs.Count, MaximumOptionCount];
            for (Int32 Mask = 0; Mask < StateCount; Mask += 1)
            {
                for (Int32 RunIndex = 0; RunIndex < Runs.Count; RunIndex += 1)
                {
                    for (Int32 OptionIndex = 0; OptionIndex < MaximumOptionCount; OptionIndex += 1)
                    {
                        Costs[Mask, RunIndex, OptionIndex] = Double.PositiveInfinity;
                        PreviousIndices[Mask, RunIndex, OptionIndex] = -1;
                        PreviousOptions[Mask, RunIndex, OptionIndex] = -1;
                    }
                }
            }

            for (Int32 RunIndex = 0; RunIndex < Runs.Count; RunIndex += 1)
            {
                for (Int32 OptionIndex = 0; OptionIndex < Options[RunIndex].Count; OptionIndex += 1)
                {
                    SequencedRun Option = Options[RunIndex][OptionIndex];
                    Double Cost = FlexibleOptionPenalty(Option, PreferredLineupM);
                    if (Start.HasValue)
                    {
                        Cost += ConnectorCost(
                            Start.Value,
                            Option.EntryPose,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            AllowForwardDetour: AllowForwardDetour);
                    }
                    Costs[1 << RunIndex, RunIndex, OptionIndex] = Cost;
                }
            }

            for (Int32 Mask = 1; Mask < StateCount; Mask += 1)
            {
                for (Int32 LastIndex = 0; LastIndex < Runs.Count; LastIndex += 1)
                {
                    for (Int32 LastOption = 0;
                        LastOption < Options[LastIndex].Count;
                        LastOption += 1)
                    {
                        Double CurrentCost = Costs[Mask, LastIndex, LastOption];
                        if (!Double.IsFinite(CurrentCost)) continue;
                        for (Int32 NextIndex = 0; NextIndex < Runs.Count; NextIndex += 1)
                        {
                            if ((Mask & (1 << NextIndex)) != 0) continue;
                            for (Int32 NextOption = 0;
                                NextOption < Options[NextIndex].Count;
                                NextOption += 1)
                            {
                                SequencedRun Next = Options[NextIndex][NextOption];
                                Double TransitionM = ConnectorCost(
                                    Options[LastIndex][LastOption].ExitPose,
                                    Next.EntryPose,
                                    MinimumRadiusM,
                                    SampleSpacingM,
                                    Field,
                                    AllowForwardDetour: AllowForwardDetour);
                                if (!Double.IsFinite(TransitionM)) continue;
                                Int32 NextMask = Mask | (1 << NextIndex);
                                Double CandidateCost = CurrentCost
                                    + TransitionM
                                    + FlexibleOptionPenalty(Next, PreferredLineupM);
                                if (CandidateCost + COST_TOLERANCE
                                    >= Costs[NextMask, NextIndex, NextOption])
                                {
                                    continue;
                                }
                                Costs[NextMask, NextIndex, NextOption] = CandidateCost;
                                PreviousIndices[NextMask, NextIndex, NextOption] = LastIndex;
                                PreviousOptions[NextMask, NextIndex, NextOption] = LastOption;
                            }
                        }
                    }
                }
            }

            Int32 FullMask = StateCount - 1;
            Double BestCost = Double.PositiveInfinity;
            Int32 BestIndex = -1;
            Int32 BestOption = -1;
            for (Int32 RunIndex = 0; RunIndex < Runs.Count; RunIndex += 1)
            {
                for (Int32 OptionIndex = 0; OptionIndex < Options[RunIndex].Count; OptionIndex += 1)
                {
                    Double CandidateCost = Costs[FullMask, RunIndex, OptionIndex];
                    if (End.HasValue && Double.IsFinite(CandidateCost))
                    {
                        CandidateCost += ConnectorCost(
                            Options[RunIndex][OptionIndex].ExitPose,
                            End.Value,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            AllowForwardDetour: AllowForwardDetour);
                    }
                    if (CandidateCost + COST_TOLERANCE >= BestCost) continue;
                    BestCost = CandidateCost;
                    BestIndex = RunIndex;
                    BestOption = OptionIndex;
                }
            }
            if (BestIndex < 0)
            {
                return Failure("No flexible-endpoint ordering satisfies the radius and field constraints.");
            }

            List<SequencedRun> ReverseOrder = new List<SequencedRun>();
            Int32 ReconstructMask = FullMask;
            while (BestIndex >= 0)
            {
                ReverseOrder.Add(Options[BestIndex][BestOption]);
                Int32 PreviousIndex = PreviousIndices[
                    ReconstructMask, BestIndex, BestOption];
                Int32 PreviousOption = PreviousOptions[
                    ReconstructMask, BestIndex, BestOption];
                ReconstructMask &= ~(1 << BestIndex);
                BestIndex = PreviousIndex;
                BestOption = PreviousOption;
            }
            ReverseOrder.Reverse();
            return BuildResult(
                ReverseOrder, MinimumRadiusM, SampleSpacingM, Field,
                AllowForwardDetour);
        }

        /// <summary>
        /// Retains low-cost flexible variants plus the deepest entry, exit, and
        /// combined retreats so bounded search does not discard edge clearance.
        /// </summary>
        private static List<SequencedRun> SelectBoundedFlexibleOptions
            (
            List<SequencedRun> Options,
            Double PreferredLineupM
            )
        {
            List<SequencedRun> Selected = new List<SequencedRun>();
            void Add(SequencedRun Option)
            {
                if (!Selected.Contains(Option)) Selected.Add(Option);
            }
            if (Options.Count == 0) return Selected;
            Add(Options
                .OrderBy(Option => FlexibleOptionPenalty(
                    Option, PreferredLineupM))
                .First());
            Add(Options
                .OrderByDescending(Option => Option.StartCoverageShortfallM)
                .ThenBy(Option => FlexibleOptionPenalty(
                    Option, PreferredLineupM))
                .First());
            Add(Options
                .OrderByDescending(Option => Option.EndCoverageShortfallM)
                .ThenBy(Option => FlexibleOptionPenalty(
                    Option, PreferredLineupM))
                .First());
            Add(Options
                .OrderByDescending(Option =>
                    Option.StartCoverageShortfallM
                        + Option.EndCoverageShortfallM)
                .ThenBy(Option => FlexibleOptionPenalty(
                    Option, PreferredLineupM))
                .First());
            Add(Options
                .OrderBy(Option => Option.EntryLineupM)
                .ThenBy(Option => FlexibleOptionPenalty(
                    Option, PreferredLineupM))
                .First());
            Add(Options
                .OrderBy(Option => Option.ExitLineupM)
                .ThenBy(Option => FlexibleOptionPenalty(
                    Option, PreferredLineupM))
                .First());
            foreach (SequencedRun Option in Options
                .OrderBy(Option => FlexibleOptionPenalty(
                    Option, PreferredLineupM)))
            {
                Add(Option);
                if (Selected.Count >= FLEXIBLE_OPTIONS_PER_ORIENTATION) break;
            }
            return Selected;
        }

        /// <summary>
        /// Builds a deterministic bounded flexible-endpoint sequence when subset
        /// search would grow exponentially. A small number of alternate first
        /// choices is tried only when the default greedy walk cannot finish.
        /// </summary>
        private static Result OptimizeFlexibleGreedy
            (
            List<List<SequencedRun>> Options,
            ExperimentalConnectorPlanner.Pose? Start,
            ExperimentalConnectorPlanner.Pose? End,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AllowForwardDetour
            )
        {
            if (AllowForwardDetour
                && (Options.Count <= FLEXIBLE_EXACT_RETRY_RUN_LIMIT))
            {
                Boolean ExactSuccess = TryFlexibleExactOrder(
                    Options,
                    Start,
                    End,
                    PreferredLineupM,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    true,
                    out List<SequencedRun> ExactOrdered,
                    out String ExactDiagnostic,
                    out List<Int32> ExactInfeasibleRuns,
                    null);
                return !ExactSuccess
                    ? Failure(
                        "Targeted final-connector detour could not complete the flexible sequence. "
                        + ExactDiagnostic,
                        ExactInfeasibleRuns)
                    : BuildResult(
                        ExactOrdered,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        AllowForwardDetour: true);
            }
            if (TryFlexibleGreedyOrder(
                Options,
                Start,
                End,
                PreferredLineupM,
                MinimumRadiusM,
                SampleSpacingM,
                Field,
                AllowForwardDetour,
                -1,
                -1,
                out List<SequencedRun> Ordered,
                out Double IgnoredCost))
            {
                return BuildResult(
                    Ordered, MinimumRadiusM, SampleSpacingM, Field,
                    AllowForwardDetour);
            }

            Boolean BeamSuccess = TryFlexibleBeamOrder(
                Options,
                Start,
                End,
                PreferredLineupM,
                MinimumRadiusM,
                SampleSpacingM,
                Field,
                AllowForwardDetour,
                FLEXIBLE_BEAM_WIDTH,
                out List<SequencedRun> BeamOrdered,
                out String BeamDiagnostic);
            List<Int32> BeamInfeasibleRuns = new List<Int32>();
            if (!BeamSuccess && (Options.Count <= FLEXIBLE_EXACT_RETRY_RUN_LIMIT))
            {
                BeamSuccess = TryFlexibleExactOrder(
                    Options,
                    Start,
                    End,
                    PreferredLineupM,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    AllowForwardDetour,
                    out BeamOrdered,
                    out BeamDiagnostic,
                    out BeamInfeasibleRuns,
                    null);
            }
            return !BeamSuccess
                ? Failure(
                    "Bounded flexible sequencing could not reach every lineup in-field. "
                    + BeamDiagnostic,
                    BeamInfeasibleRuns)
                : BuildResult(
                    BeamOrdered, MinimumRadiusM, SampleSpacingM, Field,
                    AllowForwardDetour);
        }

        /// <summary>
        /// Keeps only a fixed number of lowest-cost partial flexible sequences at
        /// each depth, preventing subset-state growth while avoiding greedy traps.
        /// </summary>
        private static Boolean TryFlexibleBeamOrder
            (
            List<List<SequencedRun>> Options,
            ExperimentalConnectorPlanner.Pose? Start,
            ExperimentalConnectorPlanner.Pose? End,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AllowForwardDetour,
            Int32 BeamWidth,
            out List<SequencedRun> Ordered,
            out String Diagnostic
            )
        {
            List<(Int32 Mask, Int32 LastRun, Int32 LastOption,
                Double Cost, List<SequencedRun> Runs)> Beam =
                    new List<(Int32, Int32, Int32, Double, List<SequencedRun>)>
                    {
                        (0, -1, -1, 0.0, new List<SequencedRun>()),
                    };
            for (Int32 Position = 0; Position < Options.Count; Position += 1)
            {
                Int32 PreviousStateCount = Beam.Count;
                Int32 AttemptedTransitions = 0;
                Int32 RejectedTransitions = 0;
                Dictionary<(Int32 Mask, Int32 LastRun, Int32 LastOption),
                    (Double Cost, List<SequencedRun> Runs)> BestByState =
                        new Dictionary<(Int32, Int32, Int32),
                            (Double, List<SequencedRun>)>();
                foreach ((Int32 Mask, Int32 LastRun, Int32 LastOption,
                    Double Cost, List<SequencedRun> Runs) State in Beam)
                {
                    for (Int32 RunIndex = 0;
                        RunIndex < Options.Count;
                        RunIndex += 1)
                    {
                        if ((State.Mask & (1 << RunIndex)) != 0) continue;
                        for (Int32 OptionIndex = 0;
                            OptionIndex < Options[RunIndex].Count;
                            OptionIndex += 1)
                        {
                            AttemptedTransitions += 1;
                            SequencedRun Option = Options[RunIndex][OptionIndex];
                            Double CandidateCost = State.Cost
                                + FlexibleOptionPenalty(Option, PreferredLineupM);
                            if (State.LastRun >= 0)
                            {
                                CandidateCost += ConnectorCost(
                                    Options[State.LastRun][State.LastOption].ExitPose,
                                    Option.EntryPose,
                                    MinimumRadiusM,
                                    SampleSpacingM,
                                    Field,
                                    AllowForwardDetour: AllowForwardDetour);
                            }
                            else if (Start.HasValue)
                            {
                                CandidateCost += ConnectorCost(
                                    Start.Value,
                                    Option.EntryPose,
                                    MinimumRadiusM,
                                    SampleSpacingM,
                                    Field,
                                    AllowForwardDetour: AllowForwardDetour);
                            }
                            if ((Position == Options.Count - 1)
                                && End.HasValue
                                && Double.IsFinite(CandidateCost))
                            {
                                CandidateCost += ConnectorCost(
                                    Option.ExitPose,
                                    End.Value,
                                    MinimumRadiusM,
                                    SampleSpacingM,
                                    Field,
                                    AllowForwardDetour: AllowForwardDetour);
                            }
                            if (!Double.IsFinite(CandidateCost))
                            {
                                RejectedTransitions += 1;
                                continue;
                            }
                            Int32 CandidateMask = State.Mask | (1 << RunIndex);
                            (Int32 Mask, Int32 LastRun, Int32 LastOption) Key =
                                (CandidateMask, RunIndex, OptionIndex);
                            if (BestByState.TryGetValue(
                                    Key,
                                    out (Double Cost, List<SequencedRun> Runs) Existing)
                                && (CandidateCost + COST_TOLERANCE >= Existing.Cost))
                            {
                                continue;
                            }
                            List<SequencedRun> CandidateRuns =
                                new List<SequencedRun>(State.Runs) { Option };
                            BestByState[Key] = (CandidateCost, CandidateRuns);
                        }
                    }
                }
                Beam = BestByState
                    .Select(Item => (
                        Item.Key.Mask,
                        Item.Key.LastRun,
                        Item.Key.LastOption,
                        Item.Value.Cost,
                        Item.Value.Runs))
                    .OrderBy(State => State.Cost)
                    .ThenBy(State => State.Mask)
                    .ThenBy(State => State.LastRun)
                    .ThenBy(State => State.LastOption)
                    .Take(BeamWidth)
                    .ToList();
                if (Beam.Count == 0)
                {
                    Ordered = new List<SequencedRun>();
                    Diagnostic = String.Format(
                        "beam_width={0} stopped_depth={1}/{2} previous_states={3} "
                        + "attempted={4} rejected={5} unique_feasible={6}.",
                        BeamWidth,
                        Position,
                        Options.Count,
                        PreviousStateCount,
                        AttemptedTransitions,
                        RejectedTransitions,
                        BestByState.Count);
                    return false;
                }
            }
            Ordered = Beam[0].Runs;
            Diagnostic = String.Format(
                "beam_width={0} completed_depth={1} retained_states={2}.",
                BeamWidth,
                Options.Count,
                Beam.Count);
            return true;
        }

        /// <summary>
        /// Exhaustively orders the already-pruned flexible endpoint options for a
        /// small run group. Connector costs are cached by endpoint pair, so the
        /// exponential portion is limited to at most 2^10 subset states rather
        /// than repeatedly invoking the geometric planner for every ordering.
        /// </summary>
        private static Boolean TryFlexibleExactOrder
            (
            List<List<SequencedRun>> Options,
            ExperimentalConnectorPlanner.Pose? Start,
            ExperimentalConnectorPlanner.Pose? End,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AllowForwardDetour,
            out List<SequencedRun> Ordered,
            out String Diagnostic,
            out List<Int32> InfeasibleRuns,
            Dictionary<(Int32 FromRun, Int32 FromOption,
                Int32 ToRun, Int32 ToOption), Double>?
                    TargetedInternalDetourCosts
            )
        {
            InfeasibleRuns = new List<Int32>();
            Int32 RunCount = Options.Count;
            Int32 StateCount = 1 << RunCount;
            Int32 MaximumOptionCount = Options.Max(Run => Run.Count);
            Double[,,] Costs = new Double[
                StateCount, RunCount, MaximumOptionCount];
            Int32[,,] PreviousRuns = new Int32[
                StateCount, RunCount, MaximumOptionCount];
            Int32[,,] PreviousOptions = new Int32[
                StateCount, RunCount, MaximumOptionCount];
            for (Int32 Mask = 0; Mask < StateCount; Mask += 1)
            {
                for (Int32 RunIndex = 0; RunIndex < RunCount; RunIndex += 1)
                {
                    for (Int32 OptionIndex = 0;
                        OptionIndex < MaximumOptionCount;
                        OptionIndex += 1)
                    {
                        Costs[Mask, RunIndex, OptionIndex] =
                            Double.PositiveInfinity;
                        PreviousRuns[Mask, RunIndex, OptionIndex] = -1;
                        PreviousOptions[Mask, RunIndex, OptionIndex] = -1;
                    }
                }
            }

            Int32 ConnectorEvaluations = 0;
            Int32 RejectedConnectors = 0;
            Int32 MaximumReachedDepth = 0;
            HashSet<Int32> StartReachableRuns = new HashSet<Int32>();
            HashSet<(Int32 FromRun, Int32 ToRun)> FeasibleRunEdges =
                new HashSet<(Int32, Int32)>();
            Dictionary<(Int32 FromRun, Int32 FromOption,
                Int32 ToRun, Int32 ToOption), Double> TransitionCosts =
                    new Dictionary<(Int32, Int32, Int32, Int32), Double>();
            Double TransitionCost(
                Int32 FromRun,
                Int32 FromOption,
                Int32 ToRun,
                Int32 ToOption)
            {
                (Int32, Int32, Int32, Int32) Key =
                    (FromRun, FromOption, ToRun, ToOption);
                if (TransitionCosts.TryGetValue(Key, out Double Cached))
                {
                    return Cached;
                }
                ConnectorEvaluations += 1;
                Double Cost = ConnectorCost(
                    Options[FromRun][FromOption].ExitPose,
                    Options[ToRun][ToOption].EntryPose,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field);
                if (!Double.IsFinite(Cost)
                    && (TargetedInternalDetourCosts != null)
                    && TargetedInternalDetourCosts.TryGetValue(
                        Key, out Double TargetedDetourCost))
                {
                    Cost = TargetedDetourCost;
                    FeasibleRunEdges.Add((FromRun, ToRun));
                }
                if (!Double.IsFinite(Cost)) RejectedConnectors += 1;
                else FeasibleRunEdges.Add((FromRun, ToRun));
                TransitionCosts[Key] = Cost;
                return Cost;
            }

            for (Int32 RunIndex = 0; RunIndex < RunCount; RunIndex += 1)
            {
                for (Int32 OptionIndex = 0;
                    OptionIndex < Options[RunIndex].Count;
                    OptionIndex += 1)
                {
                    Double Cost = FlexibleOptionPenalty(
                        Options[RunIndex][OptionIndex], PreferredLineupM);
                    if (Start.HasValue)
                    {
                        ConnectorEvaluations += 1;
                        Double StartCost = ConnectorCost(
                            Start.Value,
                            Options[RunIndex][OptionIndex].EntryPose,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field);
                        if (!Double.IsFinite(StartCost))
                        {
                            RejectedConnectors += 1;
                            continue;
                        }
                        Cost += StartCost;
                    }
                    StartReachableRuns.Add(RunIndex);
                    Costs[1 << RunIndex, RunIndex, OptionIndex] = Cost;
                }
            }

            Int32 ReachableStates = 0;
            for (Int32 Mask = 1; Mask < StateCount; Mask += 1)
            {
                for (Int32 LastRun = 0; LastRun < RunCount; LastRun += 1)
                {
                    if ((Mask & (1 << LastRun)) == 0) continue;
                    for (Int32 LastOption = 0;
                        LastOption < Options[LastRun].Count;
                        LastOption += 1)
                    {
                        Double CurrentCost = Costs[Mask, LastRun, LastOption];
                        if (!Double.IsFinite(CurrentCost)) continue;
                        ReachableStates += 1;
                        Int32 Depth = 0;
                        for (Int32 BitMask = Mask;
                            BitMask != 0;
                            BitMask >>= 1)
                        {
                            Depth += BitMask & 1;
                        }
                        MaximumReachedDepth = Math.Max(
                            MaximumReachedDepth, Depth);
                        for (Int32 NextRun = 0;
                            NextRun < RunCount;
                            NextRun += 1)
                        {
                            if ((Mask & (1 << NextRun)) != 0) continue;
                            for (Int32 NextOption = 0;
                                NextOption < Options[NextRun].Count;
                                NextOption += 1)
                            {
                                Double ConnectorM = TransitionCost(
                                    LastRun,
                                    LastOption,
                                    NextRun,
                                    NextOption);
                                if (!Double.IsFinite(ConnectorM)) continue;
                                Int32 NextMask = Mask | (1 << NextRun);
                                Double CandidateCost = CurrentCost
                                    + ConnectorM
                                    + FlexibleOptionPenalty(
                                        Options[NextRun][NextOption],
                                        PreferredLineupM);
                                if (CandidateCost + COST_TOLERANCE
                                    >= Costs[NextMask, NextRun, NextOption])
                                {
                                    continue;
                                }
                                Costs[NextMask, NextRun, NextOption] =
                                    CandidateCost;
                                PreviousRuns[NextMask, NextRun, NextOption] =
                                    LastRun;
                                PreviousOptions[NextMask, NextRun, NextOption] =
                                    LastOption;
                            }
                        }
                    }
                }
            }

            Int32 FullMask = StateCount - 1;
            Double BestCost = Double.PositiveInfinity;
            Int32 BestRun = -1;
            Int32 BestOption = -1;
            Int32 FullMaskOptionCount = 0;
            HashSet<Int32> FullMaskRuns = new HashSet<Int32>();
            HashSet<Int32> EndReachableRuns = new HashSet<Int32>();
            List<String> ClosestFullEndpoints = new List<String>();
            for (Int32 RunIndex = 0; RunIndex < RunCount; RunIndex += 1)
            {
                for (Int32 OptionIndex = 0;
                    OptionIndex < Options[RunIndex].Count;
                    OptionIndex += 1)
                {
                    Double CandidateCost = Costs[FullMask, RunIndex, OptionIndex];
                    if (!Double.IsFinite(CandidateCost)) continue;
                    FullMaskOptionCount += 1;
                    FullMaskRuns.Add(RunIndex);
                    if (End.HasValue)
                    {
                        ConnectorEvaluations += 1;
                        Double EndCost = ConnectorCost(
                            Options[RunIndex][OptionIndex].ExitPose,
                            End.Value,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            AllowForwardDetour: AllowForwardDetour);
                        if (!Double.IsFinite(EndCost))
                        {
                            RejectedConnectors += 1;
                            ClosestFullEndpoints.Add(String.Format(
                                "{0}:{1}@({2:F1},{3:F1},{4:F0})d={5:F1}",
                                RunIndex,
                                OptionIndex,
                                Options[RunIndex][OptionIndex].ExitPose.X,
                                Options[RunIndex][OptionIndex].ExitPose.Y,
                                Options[RunIndex][OptionIndex]
                                    .ExitPose.HeadingDeg,
                                Distance(
                                    (
                                        Options[RunIndex][OptionIndex]
                                            .ExitPose.X,
                                        Options[RunIndex][OptionIndex]
                                            .ExitPose.Y),
                                    (End.Value.X, End.Value.Y))));
                            continue;
                        }
                        EndReachableRuns.Add(RunIndex);
                        CandidateCost += EndCost;
                    }
                    if (CandidateCost + COST_TOLERANCE >= BestCost) continue;
                    BestCost = CandidateCost;
                    BestRun = RunIndex;
                    BestOption = OptionIndex;
                }
            }
            if (BestRun < 0)
            {
                List<Int32> DeadEndRuns = Enumerable.Range(0, RunCount)
                    .Where(RunIndex => !FeasibleRunEdges.Any(
                        Edge => Edge.FromRun == RunIndex))
                    .ToList();
                InfeasibleRuns = DeadEndRuns;
                if (AllowForwardDetour
                    && (TargetedInternalDetourCosts == null)
                    && (DeadEndRuns.Count > 0))
                {
                    HashSet<(Int32 FromRun, Int32 FromOption, Int32 ToRun)>
                        ViableDetourTransitions =
                            new HashSet<(Int32, Int32, Int32)>();
                    foreach (Int32 FromRun in DeadEndRuns)
                    {
                        for (Int32 Mask = 1; Mask < StateCount; Mask += 1)
                        {
                            if ((Mask & (1 << FromRun)) == 0) continue;
                            for (Int32 FromOption = 0;
                                FromOption < Options[FromRun].Count;
                                FromOption += 1)
                            {
                                if (!Double.IsFinite(Costs[
                                    Mask, FromRun, FromOption]))
                                {
                                    continue;
                                }
                                for (Int32 ToRun = 0;
                                    ToRun < RunCount;
                                    ToRun += 1)
                                {
                                    if ((Mask & (1 << ToRun)) == 0)
                                    {
                                        ViableDetourTransitions.Add((
                                            FromRun, FromOption, ToRun));
                                    }
                                }
                            }
                        }
                    }
                    Dictionary<(Int32 FromRun, Int32 FromOption,
                        Int32 ToRun, Int32 ToOption), Double> DetourCosts =
                            BuildTargetedInternalDetourCosts(
                                Options,
                                DeadEndRuns,
                                ViableDetourTransitions,
                                MinimumRadiusM,
                                SampleSpacingM,
                                Field,
                                out Int32 DetourAttempts);
                    if (DetourCosts.Count > 0)
                    {
                        Boolean Recovered = TryFlexibleExactOrder(
                            Options,
                            Start,
                            End,
                            PreferredLineupM,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            AllowForwardDetour,
                            out Ordered,
                            out Diagnostic,
                            out InfeasibleRuns,
                            DetourCosts);
                        Diagnostic = String.Format(
                            "targeted_internal_detour_sources=[{0}] attempts={1} edges={2} run_edges=[{3}]. {4}",
                            String.Join(",", DeadEndRuns),
                            DetourAttempts,
                            DetourCosts.Count,
                            String.Join(",", DetourCosts.Keys
                                .Select(Key => (Key.FromRun, Key.ToRun))
                                .Distinct()
                                .OrderBy(Key => Key.FromRun)
                                .ThenBy(Key => Key.ToRun)
                                .Select(Key => String.Format(
                                    "{0}>{1}", Key.FromRun, Key.ToRun))),
                            Diagnostic);
                        return Recovered;
                    }
                    Diagnostic = String.Format(
                        "targeted_internal_detour_sources=[{0}] attempts={1} edges=0. ",
                        String.Join(",", DeadEndRuns),
                        DetourAttempts);
                }
                else
                {
                    Diagnostic = String.Empty;
                }
                Ordered = new List<SequencedRun>();
                Diagnostic += String.Format(
                    "exact_masks={0} reachable_states={1} connector_evaluations={2} "
                    + "rejected_connectors={3} max_depth={4}/{5} options=[{6}] "
                    + "start_runs=[{7}] run_edges=[{8}] full_mask_options={9} "
                    + "full_mask_runs=[{10}] end=({11:F1},{12:F1},{13:F0}) "
                    + "end_runs=[{14}] closest_failed_ends=[{15}] "
                    + "complete_path=False.",
                    StateCount,
                    ReachableStates,
                    ConnectorEvaluations,
                    RejectedConnectors,
                    MaximumReachedDepth,
                    RunCount,
                    String.Join(",", Options.Select(Run => Run.Count)),
                    String.Join(",", StartReachableRuns.OrderBy(Index => Index)),
                    String.Join(",", FeasibleRunEdges
                        .OrderBy(Edge => Edge.FromRun)
                        .ThenBy(Edge => Edge.ToRun)
                        .Select(Edge => String.Format(
                            "{0}>{1}", Edge.FromRun, Edge.ToRun))),
                    FullMaskOptionCount,
                    String.Join(",", FullMaskRuns.OrderBy(Index => Index)),
                    End.HasValue ? End.Value.X : Double.NaN,
                    End.HasValue ? End.Value.Y : Double.NaN,
                    End.HasValue ? End.Value.HeadingDeg : Double.NaN,
                    String.Join(",", EndReachableRuns.OrderBy(Index => Index)),
                    String.Join(",", ClosestFullEndpoints
                        .OrderBy(Value => Value)
                        .Take(24)));
                return false;
            }

            List<SequencedRun> ReverseOrder = new List<SequencedRun>();
            Int32 ReconstructMask = FullMask;
            while (BestRun >= 0)
            {
                ReverseOrder.Add(Options[BestRun][BestOption]);
                Int32 PreviousRun = PreviousRuns[
                    ReconstructMask, BestRun, BestOption];
                Int32 PreviousOption = PreviousOptions[
                    ReconstructMask, BestRun, BestOption];
                ReconstructMask &= ~(1 << BestRun);
                BestRun = PreviousRun;
                BestOption = PreviousOption;
            }
            ReverseOrder.Reverse();
            Ordered = ReverseOrder;
            InfeasibleRuns = new List<Int32>();
            Diagnostic = String.Format(
                "exact_masks={0} reachable_states={1} connector_evaluations={2} "
                + "rejected_connectors={3} complete_path=True.",
                StateCount,
                ReachableStates,
                ConnectorEvaluations,
                RejectedConnectors);
            return true;
        }

        /// <summary>
        /// Tries a bounded nearest-first set of forward-only transitions from runs
        /// which have no ordinary outbound edge. Successful costs are fed into a
        /// second exact-order pass; normal runs never pay for this search.
        /// </summary>
        private static Dictionary<(Int32 FromRun, Int32 FromOption,
            Int32 ToRun, Int32 ToOption), Double>
                BuildTargetedInternalDetourCosts
            (
            List<List<SequencedRun>> Options,
            List<Int32> DeadEndRuns,
            HashSet<(Int32 FromRun, Int32 FromOption, Int32 ToRun)>
                ViableTransitions,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            out Int32 AttemptCount
            )
        {
            Dictionary<(Int32 FromRun, Int32 FromOption,
                Int32 ToRun, Int32 ToOption), Double> Costs =
                    new Dictionary<(Int32, Int32, Int32, Int32), Double>();
            AttemptCount = 0;
            foreach (Int32 FromRun in DeadEndRuns)
            {
                List<((Int32 FromRun, Int32 FromOption, Int32 ToRun,
                    Int32 ToOption) Key, Double DistanceM, Double Penalty)>
                        Candidates = Enumerable.Range(
                        0, Options[FromRun].Count)
                    .SelectMany(FromOption => Enumerable.Range(
                            0, Options.Count)
                        .Where(ToRun => ToRun != FromRun)
                        .Where(ToRun => ViableTransitions.Contains((
                            FromRun, FromOption, ToRun)))
                        .SelectMany(ToRun => Enumerable.Range(
                                0, Options[ToRun].Count)
                            .Select(ToOption => (
                                Key: (FromRun, FromOption, ToRun, ToOption),
                                DistanceM: Distance(
                                    (
                                        Options[FromRun][FromOption].ExitPose.X,
                                        Options[FromRun][FromOption].ExitPose.Y),
                                    (
                                        Options[ToRun][ToOption].EntryPose.X,
                                        Options[ToRun][ToOption].EntryPose.Y)),
                                Penalty: Options[ToRun][ToOption]
                                    .StartCoverageShortfallM
                                    + Options[ToRun][ToOption]
                                        .EndCoverageShortfallM))))
                    .GroupBy(Candidate => Candidate.Key.ToRun)
                    .SelectMany(Group => Group
                        .OrderBy(Candidate => Candidate.DistanceM)
                        .ThenBy(Candidate => Candidate.Penalty)
                        .ThenBy(Candidate => Candidate.Key.FromOption)
                        .ThenBy(Candidate => Candidate.Key.ToOption)
                        .Take(TARGETED_DETOUR_OPTIONS_PER_DESTINATION)
                        .Select((Candidate, Rank) => (
                            Candidate: Candidate,
                            Rank: Rank)))
                    // Interleave destinations. A pile of close variants leading
                    // to one run does not repair the sequence graph if that run
                    // cannot visit all remaining work.
                    .OrderBy(Item => Item.Rank)
                    .ThenBy(Item => Item.Candidate.DistanceM)
                    .ThenBy(Item => Item.Candidate.Key.ToRun)
                    .Select(Item => Item.Candidate)
                    .ToList();
                HashSet<(Int64 StartX, Int64 StartY, Int64 StartHeading,
                    Int64 EndX, Int64 EndY, Int64 EndHeading)> SeenGeometry =
                        new HashSet<(Int64, Int64, Int64, Int64, Int64, Int64)>();
                Int32 RunAttempts = 0;
                Int32 RunSuccesses = 0;
                foreach (((Int32 FromRun, Int32 FromOption, Int32 ToRun,
                    Int32 ToOption) Key, Double DistanceM, Double Penalty)
                        Candidate in Candidates)
                {
                    SequencedRun From = Options[
                        Candidate.Key.FromRun][Candidate.Key.FromOption];
                    SequencedRun To = Options[
                        Candidate.Key.ToRun][Candidate.Key.ToOption];
                    (Int64, Int64, Int64, Int64, Int64, Int64) Geometry = (
                        BitConverter.DoubleToInt64Bits(From.ExitPose.X),
                        BitConverter.DoubleToInt64Bits(From.ExitPose.Y),
                        BitConverter.DoubleToInt64Bits(
                            From.ExitPose.HeadingDeg % 360.0),
                        BitConverter.DoubleToInt64Bits(To.EntryPose.X),
                        BitConverter.DoubleToInt64Bits(To.EntryPose.Y),
                        BitConverter.DoubleToInt64Bits(
                            To.EntryPose.HeadingDeg % 360.0));
                    if (!SeenGeometry.Add(Geometry)) continue;
                    if (RunAttempts
                        >= TARGETED_DETOUR_ATTEMPT_LIMIT_PER_RUN)
                    {
                        break;
                    }
                    RunAttempts += 1;
                    AttemptCount += 1;
                    Double Cost = ConnectorCost(
                        From.ExitPose,
                        To.EntryPose,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        AllowForwardDetour: true);
                    if (!Double.IsFinite(Cost)) continue;
                    Costs[Candidate.Key] = Cost;
                    RunSuccesses += 1;
                    if (RunSuccesses
                        >= TARGETED_DETOUR_SUCCESS_LIMIT_PER_RUN)
                    {
                        break;
                    }
                }
            }
            return Costs;
        }

        /// <summary>Builds one bounded greedy flexible order.</summary>
        private static Boolean TryFlexibleGreedyOrder
            (
            List<List<SequencedRun>> Options,
            ExperimentalConnectorPlanner.Pose? Start,
            ExperimentalConnectorPlanner.Pose? End,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean AllowForwardDetour,
            Int32 ForcedFirstRun,
            Int32 ForcedFirstOption,
            out List<SequencedRun> Ordered,
            out Double TotalCost
            )
        {
            Boolean[] Used = new Boolean[Options.Count];
            Ordered = new List<SequencedRun>(Options.Count);
            TotalCost = 0.0;
            for (Int32 Position = 0; Position < Options.Count; Position += 1)
            {
                Double BestCost = Double.PositiveInfinity;
                Int32 BestRun = -1;
                Int32 BestOption = -1;
                for (Int32 RunIndex = 0; RunIndex < Options.Count; RunIndex += 1)
                {
                    if (Used[RunIndex]) continue;
                    for (Int32 OptionIndex = 0;
                        OptionIndex < Options[RunIndex].Count;
                        OptionIndex += 1)
                    {
                        if ((Position == 0) && (ForcedFirstRun >= 0)
                            && ((RunIndex != ForcedFirstRun)
                                || (OptionIndex != ForcedFirstOption)))
                        {
                            continue;
                        }
                        SequencedRun Option = Options[RunIndex][OptionIndex];
                        Double Cost = FlexibleOptionPenalty(
                            Option, PreferredLineupM);
                        if (Ordered.Count > 0)
                        {
                            Cost += ConnectorCost(
                                Ordered[Ordered.Count - 1].ExitPose,
                                Option.EntryPose,
                                MinimumRadiusM,
                                SampleSpacingM,
                                Field,
                                AllowForwardDetour: AllowForwardDetour);
                        }
                        else if (Start.HasValue)
                        {
                            Cost += ConnectorCost(
                                Start.Value,
                                Option.EntryPose,
                                MinimumRadiusM,
                                SampleSpacingM,
                                Field,
                                AllowForwardDetour: AllowForwardDetour);
                        }
                        if ((Position == Options.Count - 1)
                            && End.HasValue
                            && Double.IsFinite(Cost))
                        {
                            Cost += ConnectorCost(
                                Option.ExitPose,
                                End.Value,
                                MinimumRadiusM,
                                SampleSpacingM,
                                Field,
                                AllowForwardDetour: AllowForwardDetour);
                        }
                        if (Cost + COST_TOLERANCE < BestCost)
                        {
                            BestCost = Cost;
                            BestRun = RunIndex;
                            BestOption = OptionIndex;
                        }
                    }
                }
                if ((BestRun < 0) || !Double.IsFinite(BestCost))
                {
                    Ordered.Clear();
                    TotalCost = Double.PositiveInfinity;
                    return false;
                }
                Used[BestRun] = true;
                Ordered.Add(Options[BestRun][BestOption]);
                TotalCost += BestCost;
            }
            return true;
        }

        /// <summary>Appends deterministic start/end retreat combinations for one orientation.</summary>
        private static void AppendFlexibleOptions
            (
            List<SequencedRun> Output,
            SequencedRun Base,
            Double MinimumRadiusM,
            Grid Field
            )
        {
            Double MinimumWorkM = Math.Min(
                0.5 * Field.GridSizeM,
                0.1 * Base.Run.LengthM);
            Double MaximumTotalRetreatM = Math.Max(
                0.0, Base.Run.LengthM - MinimumWorkM);
            List<Double> Levels = RetreatLevels(
                MaximumTotalRetreatM, MinimumRadiusM, Field.GridSizeM);
            Double HeadingRad = Base.Run.HeadingDeg * Math.PI / 180.0;
            Double UnitX = Math.Sin(HeadingRad);
            Double UnitY = Math.Cos(HeadingRad);
            Output.Add(Base);
            if (Base.EntryLineupM > COST_TOLERANCE)
            {
                Output.Add(new SequencedRun(
                    Base.Run,
                    new ExperimentalConnectorPlanner.Pose(
                        Base.Run.StartXy.X,
                        Base.Run.StartXy.Y,
                        Base.Run.HeadingDeg),
                    Base.ExitPose,
                    0.0,
                    Base.ExitLineupM,
                    Base.StartCoverageShortfallM,
                    Base.EndCoverageShortfallM));
            }
            if (Base.ExitLineupM > COST_TOLERANCE)
            {
                Output.Add(new SequencedRun(
                    Base.Run,
                    Base.EntryPose,
                    new ExperimentalConnectorPlanner.Pose(
                        Base.Run.EndXy.X,
                        Base.Run.EndXy.Y,
                        Base.Run.HeadingDeg),
                    Base.EntryLineupM,
                    0.0,
                    Base.StartCoverageShortfallM,
                    Base.EndCoverageShortfallM));
            }
            if ((Base.EntryLineupM > COST_TOLERANCE)
                && (Base.ExitLineupM > COST_TOLERANCE))
            {
                Output.Add(new SequencedRun(
                    Base.Run,
                    new ExperimentalConnectorPlanner.Pose(
                        Base.Run.StartXy.X,
                        Base.Run.StartXy.Y,
                        Base.Run.HeadingDeg),
                    new ExperimentalConnectorPlanner.Pose(
                        Base.Run.EndXy.X,
                        Base.Run.EndXy.Y,
                        Base.Run.HeadingDeg),
                    0.0,
                    0.0,
                    Base.StartCoverageShortfallM,
                    Base.EndCoverageShortfallM));
            }
            List<(Double StartM, Double EndM)> RetreatPairs =
                new List<(Double StartM, Double EndM)> { (0.0, 0.0) };
            for (Int32 LevelIndex = 1; LevelIndex < Levels.Count; LevelIndex += 1)
            {
                Double LevelM = Levels[LevelIndex];
                RetreatPairs.Add((LevelM, 0.0));
                RetreatPairs.Add((0.0, LevelM));
                if ((2.0 * LevelM) <= MaximumTotalRetreatM + COST_TOLERANCE)
                {
                    RetreatPairs.Add((LevelM, LevelM));
                }
            }
            foreach ((Double StartM, Double EndM) Retreat in RetreatPairs)
            {
                Double StartRetreatM = Retreat.StartM;
                Double EndRetreatM = Retreat.EndM;
                if ((StartRetreatM <= COST_TOLERANCE)
                    && (EndRetreatM <= COST_TOLERANCE))
                {
                    continue;
                }
                    ExperimentalCoveragePlanner.Run AdjustedRun =
                        new ExperimentalCoveragePlanner.Run(
                            Base.Run.Cells,
                            (
                                Base.Run.StartXy.X + (UnitX * StartRetreatM),
                                Base.Run.StartXy.Y + (UnitY * StartRetreatM)),
                            (
                                Base.Run.EndXy.X - (UnitX * EndRetreatM),
                                Base.Run.EndXy.Y - (UnitY * EndRetreatM)),
                            Base.Run.HeadingDeg,
                            Base.Run.MaximumLateralErrorM);
                    Output.Add(new SequencedRun(
                        AdjustedRun,
                        StartRetreatM > COST_TOLERANCE
                            ? new ExperimentalConnectorPlanner.Pose(
                                AdjustedRun.StartXy.X,
                                AdjustedRun.StartXy.Y,
                                AdjustedRun.HeadingDeg)
                            : Base.EntryPose,
                        EndRetreatM > COST_TOLERANCE
                            ? new ExperimentalConnectorPlanner.Pose(
                                AdjustedRun.EndXy.X,
                                AdjustedRun.EndXy.Y,
                                AdjustedRun.HeadingDeg)
                            : Base.ExitPose,
                        StartRetreatM > COST_TOLERANCE ? 0.0 : Base.EntryLineupM,
                        EndRetreatM > COST_TOLERANCE ? 0.0 : Base.ExitLineupM,
                        Base.StartCoverageShortfallM + StartRetreatM,
                        Base.EndCoverageShortfallM + EndRetreatM));
            }
        }

        /// <summary>Returns a small deterministic retreat lattice, including radius-scale options.</summary>
        private static List<Double> RetreatLevels
            (
            Double MaximumM,
            Double MinimumRadiusM,
            Double GridSizeM
            )
        {
            List<Double> Candidates = new List<Double>
            {
                0.0,
                0.5 * GridSizeM,
                GridSizeM,
                2.0 * GridSizeM,
                MinimumRadiusM,
                2.0 * MinimumRadiusM,
                MaximumM,
            };
            Candidates.Sort();
            List<Double> Output = new List<Double>();
            foreach (Double Candidate in Candidates)
            {
                Double Clamped = Math.Max(0.0, Math.Min(Candidate, MaximumM));
                if ((Output.Count == 0)
                    || (Math.Abs(Output[Output.Count - 1] - Clamped) > COST_TOLERANCE))
                {
                    Output.Add(Clamped);
                }
            }
            return Output;
        }

        /// <summary>Scores lineups and explicitly recorded unworked endpoint distance.</summary>
        private static Double FlexibleOptionPenalty
            (
            SequencedRun Run,
            Double PreferredLineupM
            ) => LineupPenalty(Run, PreferredLineupM)
                + (COVERAGE_SHORTFALL_WEIGHT
                    * (Run.StartCoverageShortfallM + Run.EndCoverageShortfallM));

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
            Double HeadingRad = Run.HeadingDeg * Math.PI / 180.0;
            Double UnitX = Math.Sin(HeadingRad);
            Double UnitY = Math.Cos(HeadingRad);
            Double StepM = Math.Min(SampleSpacingM, Field.GridSizeM * 0.5);
            // A one-cell edge fragment may need to begin turning almost
            // immediately. Preserve a small driven core and record the rest as
            // coverage shortfall rather than disabling retreat for short runs.
            Double MinimumWorkM = Math.Min(
                Math.Max(Field.GridSizeM * 0.5, StepM),
                0.1 * Run.LengthM);
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
            Grid Field,
            Boolean AllowForwardDetour = false
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
                    Field,
                    EdgeAllowanceM: Field.GridSizeM,
                    AllowForwardDetour: AllowForwardDetour);
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
            SequencedRun? FollowingRun = null,
            Boolean AllowForwardDetour = false
            )
        {
            ExperimentalTransitPlanner.Result Connector = ExperimentalTransitPlanner.Connect(
                Start, End, MinimumRadiusM, SampleSpacingM, Field,
                EdgeAllowanceM: Field.GridSizeM,
                AllowForwardDetour: AllowForwardDetour);
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
            String Reason,
            IReadOnlyList<Int32>? InfeasibleRunIndices = null
            )
        {
            return new Result(false, new List<SequencedRun>(),
                new List<ExperimentalTransitPlanner.Result>(), 0.0, 0.0, Reason,
                InfeasibleRunIndices);
        }
    }
}
