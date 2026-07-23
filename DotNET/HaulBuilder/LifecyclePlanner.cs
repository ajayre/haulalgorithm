using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

namespace HaulBuilder
{
    /// <summary>
    /// Builds the isolated cut, fill, optional grade, and next-cut guidance lifecycle.
    /// </summary>
    static public class LifecyclePlanner
    {
        private const Double CORRIDOR_HEADING_TOLERANCE_DEG = 20.0;
        private const Double CORRIDOR_BRIDGE_HEADING_TOLERANCE_DEG = 30.0;
        private const Double CORRIDOR_LATERAL_TOLERANCE_CELLS = Constants.HALF_SCALE;
        private const Double WORK_LANE_SHIFT_LATERAL_TOLERANCE_CELLS = 2.0;
        private const Double CORRIDOR_GAP_TOLERANCE_CELLS = 2.0;
        private const Double NESTED_FILL_FRAGMENT_MAX_LENGTH_CELLS = 1.25;
        private const Double NESTED_FILL_FRAGMENT_MAX_OFFSET_CELLS = 1.0;
        private const Double FILL_ENVELOPE_MAX_ALONG_GAP_CELLS = 12.0;
        private const Double FILL_ENVELOPE_MAX_WORK_INFLATION = 2.0;
        private const Int32 FILL_ENVELOPE_MINIMUM_FRAGMENT_COUNT = 4;
        private const Double FILL_ENVELOPE_HEADING_BIN_DEG = 5.0;
        private const Int32 FILL_ENVELOPE_PROXIMITY_HEADING_LIMIT = 8;
        private const Int32 FILL_ENVELOPE_SEQUENCE_CANDIDATE_LIMIT = 8;
        private const Double MANUAL_CONNECTOR_PENALTY_M = 100000.0;
        private const Double LOADED_FILL_OVERLAP_PENALTY_M = 100000.0;
        private const Double LOADED_FILL_ALIGNMENT_TOLERANCE_DEG = 20.0;
        private const Int32 CUT_CORRIDOR_EXACT_RUN_LIMIT = 7;

        /// <summary>
        /// Strongly typed production guidance phases.
        /// </summary>
        public enum Phase
        {
            /// <summary>Initial reached pose.</summary>
            Start,
            /// <summary>Blade-up transit within a pass.</summary>
            Transit,
            /// <summary>Explicit blade-up operator takeover.</summary>
            ManualAssist,
            /// <summary>Straight blade-up cut lineup.</summary>
            LineupCut,
            /// <summary>Blade-down cut work.</summary>
            Cut,
            /// <summary>Straight blade-up cut exit.</summary>
            ExitCut,
            /// <summary>Straight blade-up fill lineup.</summary>
            LineupFill,
            /// <summary>Blade-down fill work.</summary>
            Fill,
            /// <summary>Straight blade-up fill exit.</summary>
            ExitFill,
            /// <summary>Blade-up turnaround toward grade work.</summary>
            TurnToGrade,
            /// <summary>Straight blade-up grade lineup.</summary>
            LineupGrade,
            /// <summary>Blade-down grade work over completed fill.</summary>
            Grade,
            /// <summary>Straight blade-up grade exit.</summary>
            ExitGrade,
            /// <summary>Blade-up post-fill or post-grade route to the next cut.</summary>
            TransitToNextCut,
            /// <summary>Final reached pose.</summary>
            End,
        }

        /// <summary>
        /// One pass's pre-sequenced cut and fill work.
        /// </summary>
        public sealed class PassPlan
        {
            /// <summary>Initializes one pass plan.</summary>
            public PassPlan
                (
                RunSequencer.Result Cut,
                RunSequencer.Result Fill,
                IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>?
                    CutOmittedCells = null,
                IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>?
                    FillOmittedCells = null
                )
            {
                this.Cut = Cut;
                this.Fill = Fill;
                this.CutOmittedCells = CutOmittedCells
                    ?? Array.Empty<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
                this.FillOmittedCells = FillOmittedCells
                    ?? Array.Empty<((Int32 Bx, Int32 By) Cell, Double DepthM)>();
            }

            /// <summary>Gets sequenced cut work.</summary>
            public RunSequencer.Result Cut { get; }

            /// <summary>Gets sequenced fill work.</summary>
            public RunSequencer.Result Fill { get; }

            /// <summary>Gets cut allocations explicitly omitted from guidance.</summary>
            public IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                CutOmittedCells { get; }

            /// <summary>Gets fill allocations explicitly omitted from guidance.</summary>
            public IReadOnlyList<((Int32 Bx, Int32 By) Cell, Double DepthM)>
                FillOmittedCells { get; }

            /// <summary>Gets the number of omitted cut allocations.</summary>
            public Int32 CutOmittedCellCount => CutOmittedCells.Count;

            /// <summary>Gets the number of omitted fill allocations.</summary>
            public Int32 FillOmittedCellCount => FillOmittedCells.Count;
        }

        /// <summary>
        /// One explicit lifecycle segment.
        /// </summary>
        public sealed class Segment
        {
            /// <summary>Initializes one lifecycle segment.</summary>
            public Segment
                (
                Phase Phase,
                Boolean BladeDown,
                List<(Double X, Double Y)> Points
                ) : this(Phase, BladeDown, Points, false)
            {
            }

            /// <summary>Initializes one lifecycle segment with explicit payload state.</summary>
            public Segment
                (
                Phase Phase,
                Boolean BladeDown,
                List<(Double X, Double Y)> Points,
                Boolean Loaded
                )
            {
                this.Phase = Phase;
                this.BladeDown = BladeDown;
                this.Points = Points;
                this.Loaded = Loaded;
            }

            /// <summary>Gets the strongly typed phase.</summary>
            public Phase Phase { get; }

            /// <summary>Gets whether this is blade-down work.</summary>
            public Boolean BladeDown { get; }

            /// <summary>Gets ordered centerline points.</summary>
            public List<(Double X, Double Y)> Points { get; }

            /// <summary>Gets the one-based haul route that owns this segment.</summary>
            public Int32 RouteNumber { get; internal set; }

            /// <summary>Gets whether the tractor-scraper is carrying material.</summary>
            public Boolean Loaded { get; internal set; }
        }

        /// <summary>
        /// Complete production lifecycle result.
        /// </summary>
        public sealed class Result
        {
            /// <summary>Initializes one lifecycle result.</summary>
            public Result
                (
                Boolean Success,
                List<Segment> Segments,
                Int32 ManualAssistCount,
                Double DistanceM,
                String FailureReason
                )
            {
                this.Success = Success;
                this.Segments = Segments;
                this.ManualAssistCount = ManualAssistCount;
                this.DistanceM = DistanceM;
                this.FailureReason = FailureReason;
            }

            /// <summary>Gets whether every required nominal transit exists.</summary>
            public Boolean Success { get; }

            /// <summary>Gets all explicit lifecycle segments.</summary>
            public List<Segment> Segments { get; }

            /// <summary>Gets the number of explicit operator assists.</summary>
            public Int32 ManualAssistCount { get; }

            /// <summary>Gets total nominal centerline distance.</summary>
            public Double DistanceM { get; }

            /// <summary>Gets an explicit failure reason.</summary>
            public String FailureReason { get; }
        }

        /// <summary>
        /// Builds every pass and the mandatory post-fill transition.
        /// </summary>
        static public Result Build
            (
            List<PassPlan> Passes,
            Boolean GradeReturn,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Action<Int32, Int32>? Progress = null,
            Boolean FillSequencesPrepared = false,
            CancellationToken CancellationToken = default
            )
        {
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            List<Segment> Segments = new List<Segment>();
            Int32 ManualAssistCount = 0;
            ConnectorPlanner.Pose? Current = null;
            Progress?.Invoke(0, Passes.Count);

            for (Int32 PassIndex = 0; PassIndex < Passes.Count; PassIndex += 1)
            {
                CancellationToken.ThrowIfCancellationRequested();
                Int32 RouteSegmentStartIndex = Segments.Count;
                PassPlan Pass = Passes[PassIndex];
                if (!Pass.Cut.Success || !Pass.Fill.Success)
                {
                    return Failure(Segments, ManualAssistCount,
                        String.Format(System.Globalization.CultureInfo.InvariantCulture,
                            "Pass {0} cut or fill sequencing failed before lifecycle construction.",
                            PassIndex + 1));
                }

                RunSequencer.Result LifecycleCut = Pass.Cut;
                if (Pass.Cut.Runs.Count > 0)
                {
                    LifecycleCut = BuildContinuousCutSequence(
                        Pass.Cut,
                        Current,
                        Pass.Fill.Runs.Count == 0
                            ? null
                            : Pass.Fill.Runs[0].EntryPose,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field);
                    ConnectorPlanner.Pose CutEntry =
                        LifecycleCut.Runs[0].EntryPose;
                    if (Current.HasValue)
                    {
                        if (!AppendTransit(
                            Segments, ref ManualAssistCount, Current.Value, CutEntry,
                            Phase.TransitToNextCut, MinimumRadiusM, SampleSpacingM, Field,
                            out String FailureReason))
                        {
                            return Failure(Segments, ManualAssistCount,
                                String.Format(System.Globalization.CultureInfo.InvariantCulture,
                                    "Pass {0} fill or grade exit cannot reach the next cut lineup: {1}",
                                    PassIndex + 1,
                                    FailureReason));
                        }
                    }
                    if (!Current.HasValue)
                    {
                        Segments.Add(new Segment(Phase.Start, false,
                            new List<(Double X, Double Y)> { (CutEntry.X, CutEntry.Y) }));
                    }
                    AppendWorkSequence(Segments, LifecycleCut, Phase.LineupCut,
                        Phase.Cut, Phase.ExitCut, Phase.Transit, ref ManualAssistCount);
                    Current = LifecycleCut.Runs[
                        LifecycleCut.Runs.Count - 1].ExitPose;
                }

                RunSequencer.Result LifecycleFill = Pass.Fill;
                if (Pass.Fill.Runs.Count > 0)
                {
                    LifecycleFill = GradeReturn && !FillSequencesPrepared
                        ? BuildContinuousFillSequence(
                            Pass.Fill,
                            Current,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field)
                        : Pass.Fill;
                    ConnectorPlanner.Pose FillEntry =
                        LifecycleFill.Runs[0].EntryPose;
                    if (Current.HasValue)
                    {
                        if (!AppendTransit(
                            Segments, ref ManualAssistCount, Current.Value, FillEntry,
                            Phase.Transit, MinimumRadiusM, SampleSpacingM, Field,
                            out String FailureReason))
                        {
                            return Failure(Segments, ManualAssistCount,
                                String.Format(System.Globalization.CultureInfo.InvariantCulture,
                                    "Pass {0} cut exit cannot reach the fill lineup: {1}",
                                    PassIndex + 1,
                                    FailureReason));
                        }
                    }
                    AppendWorkSequence(Segments, LifecycleFill, Phase.LineupFill,
                        Phase.Fill, Phase.ExitFill, Phase.Transit, ref ManualAssistCount);
                    Current = LifecycleFill.Runs[
                        LifecycleFill.Runs.Count - 1].ExitPose;
                }

                if (GradeReturn && (Pass.Fill.Runs.Count > 0))
                {
                    List<RunSequencer.SequencedRun> GradeRuns = BuildGradeRuns(
                        LifecycleFill.Runs,
                        PreferredLineupM,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field);
                    ConnectorPlanner.Pose GradeEntry = GradeRuns[0].EntryPose;
                    if (Current.HasValue)
                    {
                        if (!AppendTransit(
                            Segments, ref ManualAssistCount, Current.Value, GradeEntry,
                            Phase.TurnToGrade, MinimumRadiusM, SampleSpacingM, Field,
                            out String FailureReason))
                        {
                            return Failure(Segments, ManualAssistCount,
                                String.Format(System.Globalization.CultureInfo.InvariantCulture,
                                    "Pass {0} fill exit cannot reach the grade lineup: {1}",
                                    PassIndex + 1,
                                    FailureReason));
                        }
                    }
                    Boolean GradeRunsAppended = CanRetraceFillConnectors(LifecycleFill)
                        ? AppendPreparedRunsWithReversedConnectors(
                            Segments,
                            GradeRuns,
                            LifecycleFill.Connectors,
                            Phase.LineupGrade,
                            Phase.Grade,
                            Phase.ExitGrade,
                            ref ManualAssistCount)
                        : AppendPreparedRuns(
                            Segments,
                            GradeRuns,
                            Phase.LineupGrade,
                            Phase.Grade,
                            Phase.ExitGrade,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            ref ManualAssistCount,
                            out String GradeFailureReason);
                    if (!GradeRunsAppended)
                    {
                        return Failure(Segments, ManualAssistCount,
                            String.Format(System.Globalization.CultureInfo.InvariantCulture,
                                "Pass {0} disconnected grade runs cannot be connected in-field.",
                                PassIndex + 1));
                    }
                    Current = GradeRuns[GradeRuns.Count - 1].ExitPose;
                }

                // Assign ownership only after every segment for this pass is known;
                // this includes inbound next-cut transit constructed at pass start.
                Boolean Loaded = false;
                for (Int32 SegmentIndex = RouteSegmentStartIndex;
                    SegmentIndex < Segments.Count;
                    SegmentIndex += 1)
                {
                    Segment Segment = Segments[SegmentIndex];
                    Segment.RouteNumber = PassIndex + 1;
                    // Every haul route arrives at its first cut empty. Cutting
                    // changes the payload to loaded; it stays loaded through all
                    // fill runs, then grading or the next-cut transit is empty.
                    if ((Segment.Phase == Phase.TransitToNextCut)
                        || (Segment.Phase == Phase.TurnToGrade)
                        || (Segment.Phase == Phase.LineupGrade)
                        || (Segment.Phase == Phase.Grade)
                        || (Segment.Phase == Phase.ExitGrade))
                    {
                        Loaded = false;
                    }
                    else if ((Segment.Phase == Phase.LineupFill)
                        || (Segment.Phase == Phase.Fill)
                        || (Segment.Phase == Phase.ExitFill))
                    {
                        Loaded = true;
                    }
                    Segment.Loaded = Loaded;
                    if (Segment.Phase == Phase.Cut)
                    {
                        Loaded = true;
                    }
                }

                Progress?.Invoke(PassIndex + 1, Passes.Count);
            }

            if (Current.HasValue)
            {
                Segment End = new Segment(Phase.End, false,
                    new List<(Double X, Double Y)> { (Current.Value.X, Current.Value.Y) });
                End.RouteNumber = Passes.Count;
                Segments.Add(End);
            }
            return new Result(true, Segments, ManualAssistCount,
                TotalDistance(Segments), String.Empty);
        }

        /// <summary>Appends cut or fill runs and their already validated internal connectors.</summary>
        private static void AppendWorkSequence
            (
            List<Segment> Output,
            RunSequencer.Result Sequence,
            Phase LineupPhase,
            Phase WorkPhase,
            Phase ExitPhase,
            Phase TransitPhase,
            ref Int32 ManualAssistCount
            )
        {
            for (Int32 Index = 0; Index < Sequence.Runs.Count; Index += 1)
            {
                if (Index > 0)
                {
                    TransitPlanner.Result Connector = Sequence.Connectors[Index - 1];
                    AppendSequenceConnector(
                        Output,
                        ref ManualAssistCount,
                        Connector,
                        TransitPhase,
                        WorkPhase,
                        Sequence.Runs[Index - 1].EndCoverageShortfallM,
                        Sequence.Runs[Index].StartCoverageShortfallM);
                }
                AppendPreparedRun(Output, Sequence.Runs[Index], LineupPhase, WorkPhase, ExitPhase);
            }
        }

        /// <summary>
        /// Replaces compatible internal cut turns with short blade-up jogs. The first
        /// entry lineup and final exit lineup remain unchanged; only the poses between
        /// nearby, similarly aligned cut fragments are compacted.
        /// </summary>
        static public RunSequencer.Result BuildContinuousCutSequence
            (
            RunSequencer.Result Sequence,
            ConnectorPlanner.Pose? Inbound,
            ConnectorPlanner.Pose? Outbound,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if (Sequence == null) throw new ArgumentNullException(nameof(Sequence));
            if (!Sequence.Success || (Sequence.Runs.Count <= 1)) return Sequence;
            if (Sequence.Runs.Count > CUT_CORRIDOR_EXACT_RUN_LIMIT)
            {
                return CompactAdjacentCutRuns(Sequence, SampleSpacingM, Field);
            }

            Int32 RunCount = Sequence.Runs.Count;
            RunSequencer.SequencedRun[,] Options =
                new RunSequencer.SequencedRun[RunCount, 2];
            for (Int32 RunIndex = 0; RunIndex < RunCount; RunIndex += 1)
            {
                Options[RunIndex, 0] = Sequence.Runs[RunIndex];
                Options[RunIndex, 1] = ReverseSequencedRun(Sequence.Runs[RunIndex]);
            }

            TransitPlanner.Result?[,,,] Transitions =
                new TransitPlanner.Result?[RunCount, 2, RunCount, 2];
            for (Int32 FirstIndex = 0; FirstIndex < RunCount; FirstIndex += 1)
            {
                for (Int32 FirstOrientation = 0; FirstOrientation < 2; FirstOrientation += 1)
                {
                    for (Int32 SecondIndex = 0; SecondIndex < RunCount; SecondIndex += 1)
                    {
                        if (FirstIndex == SecondIndex) continue;
                        for (Int32 SecondOrientation = 0; SecondOrientation < 2; SecondOrientation += 1)
                        {
                            Transitions[
                                FirstIndex,
                                FirstOrientation,
                                SecondIndex,
                                SecondOrientation] = BuildCutConnector(
                                    Options[FirstIndex, FirstOrientation],
                                    Options[SecondIndex, SecondOrientation],
                                    MinimumRadiusM,
                                    SampleSpacingM,
                                    Field);
                        }
                    }
                }
            }

            Int32 StateCount = 1 << RunCount;
            Double[,,] Costs = new Double[StateCount, RunCount, 2];
            Int32[,,] PreviousIndices = new Int32[StateCount, RunCount, 2];
            Int32[,,] PreviousOrientations = new Int32[StateCount, RunCount, 2];
            for (Int32 Mask = 0; Mask < StateCount; Mask += 1)
            {
                for (Int32 RunIndex = 0; RunIndex < RunCount; RunIndex += 1)
                {
                    for (Int32 Orientation = 0; Orientation < 2; Orientation += 1)
                    {
                        Costs[Mask, RunIndex, Orientation] = Double.PositiveInfinity;
                        PreviousIndices[Mask, RunIndex, Orientation] = -1;
                        PreviousOrientations[Mask, RunIndex, Orientation] = -1;
                    }
                }
            }

            for (Int32 RunIndex = 0; RunIndex < RunCount; RunIndex += 1)
            {
                for (Int32 Orientation = 0; Orientation < 2; Orientation += 1)
                {
                    Double StartCostM = AnchorConnectorCost(
                        Inbound,
                        Options[RunIndex, Orientation].EntryPose,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field);
                    Costs[1 << RunIndex, RunIndex, Orientation] = StartCostM;
                }
            }

            for (Int32 Mask = 1; Mask < StateCount; Mask += 1)
            {
                for (Int32 LastIndex = 0; LastIndex < RunCount; LastIndex += 1)
                {
                    for (Int32 LastOrientation = 0; LastOrientation < 2; LastOrientation += 1)
                    {
                        Double CurrentCostM = Costs[Mask, LastIndex, LastOrientation];
                        if (!Double.IsFinite(CurrentCostM)) continue;
                        for (Int32 NextIndex = 0; NextIndex < RunCount; NextIndex += 1)
                        {
                            if ((Mask & (1 << NextIndex)) != 0) continue;
                            for (Int32 NextOrientation = 0; NextOrientation < 2; NextOrientation += 1)
                            {
                                TransitPlanner.Result? Connector = Transitions[
                                    LastIndex,
                                    LastOrientation,
                                    NextIndex,
                                    NextOrientation];
                                if ((Connector == null) || !Connector.Success) continue;
                                Int32 NextMask = Mask | (1 << NextIndex);
                                Double CandidateCostM = CurrentCostM + ConnectorScoreM(Connector);
                                if (CandidateCostM + Constants.NUMERIC_COMPARISON_TOLERANCE
                                    >= Costs[NextMask, NextIndex, NextOrientation])
                                {
                                    continue;
                                }
                                Costs[NextMask, NextIndex, NextOrientation] = CandidateCostM;
                                PreviousIndices[NextMask, NextIndex, NextOrientation] = LastIndex;
                                PreviousOrientations[NextMask, NextIndex, NextOrientation] = LastOrientation;
                            }
                        }
                    }
                }
            }

            Int32 FullMask = StateCount - 1;
            Double BestCostM = Double.PositiveInfinity;
            Int32 BestIndex = -1;
            Int32 BestOrientation = -1;
            for (Int32 RunIndex = 0; RunIndex < RunCount; RunIndex += 1)
            {
                for (Int32 Orientation = 0; Orientation < 2; Orientation += 1)
                {
                    Double CandidateCostM = Costs[FullMask, RunIndex, Orientation]
                        + AnchorConnectorCost(
                            Outbound,
                            Options[RunIndex, Orientation].ExitPose,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            Reverse: true);
                    if (CandidateCostM + Constants.NUMERIC_COMPARISON_TOLERANCE >= BestCostM) continue;
                    BestCostM = CandidateCostM;
                    BestIndex = RunIndex;
                    BestOrientation = Orientation;
                }
            }
            if (BestIndex < 0)
            {
                return CompactAdjacentCutRuns(Sequence, SampleSpacingM, Field);
            }

            List<(Int32 Index, Int32 Orientation)> OrderedStates =
                new List<(Int32 Index, Int32 Orientation)>();
            Int32 ReconstructMask = FullMask;
            while (BestIndex >= 0)
            {
                OrderedStates.Add((BestIndex, BestOrientation));
                Int32 PreviousIndex = PreviousIndices[
                    ReconstructMask, BestIndex, BestOrientation];
                Int32 PreviousOrientation = PreviousOrientations[
                    ReconstructMask, BestIndex, BestOrientation];
                ReconstructMask &= ~(1 << BestIndex);
                BestIndex = PreviousIndex;
                BestOrientation = PreviousOrientation;
            }
            OrderedStates.Reverse();

            List<TransitPlanner.Result> Connectors =
                new List<TransitPlanner.Result>();
            Boolean HasDirectBridge = false;
            for (Int32 Index = 1; Index < OrderedStates.Count; Index += 1)
            {
                (Int32 Index, Int32 Orientation) Previous = OrderedStates[Index - 1];
                (Int32 Index, Int32 Orientation) Next = OrderedStates[Index];
                TransitPlanner.Result Connector = Transitions[
                    Previous.Index,
                    Previous.Orientation,
                    Next.Index,
                    Next.Orientation]!;
                Connectors.Add(Connector);
                HasDirectBridge = HasDirectBridge || IsCorridorBridge(Connector);
            }
            if (!HasDirectBridge)
            {
                return CompactAdjacentCutRuns(Sequence, SampleSpacingM, Field);
            }

            List<RunSequencer.SequencedRun> OrderedRuns =
                new List<RunSequencer.SequencedRun>();
            for (Int32 Index = 0; Index < OrderedStates.Count; Index += 1)
            {
                (Int32 Index, Int32 Orientation) State = OrderedStates[Index];
                RunSequencer.SequencedRun Source =
                    Options[State.Index, State.Orientation];
                Boolean DirectEntry = (Index > 0) && IsCorridorBridge(Connectors[Index - 1]);
                Boolean DirectExit = (Index < Connectors.Count) && IsCorridorBridge(Connectors[Index]);
                OrderedRuns.Add(WithInternalCutPoses(Source, DirectEntry, DirectExit));
            }
            return BuildCutSequenceResult(OrderedRuns, Connectors);
        }

        /// <summary>Compacts only already adjacent cut fragments without changing their order.</summary>
        private static RunSequencer.Result CompactAdjacentCutRuns
            (
            RunSequencer.Result Sequence,
            Double SampleSpacingM,
            Grid Field
            )
        {

            Int32 TransitionCount = Sequence.Runs.Count - 1;
            TransitPlanner.Result?[] DirectBridges =
                new TransitPlanner.Result?[TransitionCount];
            Boolean AnyBridge = false;
            for (Int32 Index = 0; Index < TransitionCount; Index += 1)
            {
                CoveragePlanner.Run Previous = Sequence.Runs[Index].Run;
                CoveragePlanner.Run Next = Sequence.Runs[Index + 1].Run;
                TransitPlanner.Result? Bridge = TryBuildDirectCutBridge(
                    Previous, Next, SampleSpacingM, Field);
                if (Bridge == null) continue;
                DirectBridges[Index] = Bridge;
                AnyBridge = true;
            }
            if (!AnyBridge) return Sequence;

            List<RunSequencer.SequencedRun> Runs =
                new List<RunSequencer.SequencedRun>();
            for (Int32 Index = 0; Index < Sequence.Runs.Count; Index += 1)
            {
                RunSequencer.SequencedRun Source = Sequence.Runs[Index];
                Boolean DirectEntry = (Index > 0) && (DirectBridges[Index - 1] != null);
                Boolean DirectExit = (Index < TransitionCount) && (DirectBridges[Index] != null);
                Runs.Add(WithInternalCutPoses(Source, DirectEntry, DirectExit));
            }

            List<TransitPlanner.Result> Connectors =
                new List<TransitPlanner.Result>();
            for (Int32 Index = 0; Index < TransitionCount; Index += 1)
            {
                TransitPlanner.Result Connector =
                    DirectBridges[Index] ?? Sequence.Connectors[Index];
                Connectors.Add(Connector);
            }
            return BuildCutSequenceResult(Runs, Connectors);
        }

        /// <summary>
        /// Builds cut connector.
        /// </summary>
        /// <param name="Previous">The previous used by the operation.</param>
        /// <param name="Next">The next used by the operation.</param>
        /// <param name="MinimumRadiusM">The minimum radius m used by the operation.</param>
        /// <param name="SampleSpacingM">The sample spacing m used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <returns>The resulting cut connector.</returns>

        private static TransitPlanner.Result? BuildCutConnector
            (
            RunSequencer.SequencedRun Previous,
            RunSequencer.SequencedRun Next,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            TransitPlanner.Result? Direct = TryBuildDirectCutBridge(
                Previous.Run, Next.Run, SampleSpacingM, Field);
            if (Direct != null) return Direct;
            TransitPlanner.Result Strict = TransitPlanner.Connect(
                Previous.ExitPose,
                Next.EntryPose,
                MinimumRadiusM,
                SampleSpacingM,
                Field,
                EdgeAllowanceM: Field.GridSizeM);
            return Strict.Success ? Strict : null;
        }

        /// <summary>
        /// Executes the try build direct cut bridge operation.
        /// </summary>
        /// <param name="Previous">The previous used by the operation.</param>
        /// <param name="Next">The next used by the operation.</param>
        /// <param name="SampleSpacingM">The sample spacing m used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <returns>The resulting try build direct cut bridge.</returns>

        private static TransitPlanner.Result? TryBuildDirectCutBridge
            (
            CoveragePlanner.Run Previous,
            CoveragePlanner.Run Next,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if (HeadingDifferenceDeg(Previous.HeadingDeg, Next.HeadingDeg)
                > CORRIDOR_HEADING_TOLERANCE_DEG) return null;
            Double DirectM = Distance(Previous.EndXy, Next.StartXy);
            if ((DirectM <= Constants.NUMERIC_COMPARISON_TOLERANCE)
                || (DirectM > CORRIDOR_GAP_TOLERANCE_CELLS * Field.GridSizeM)) return null;
            Double BridgeHeadingDeg = BearingDeg(Previous.EndXy, Next.StartXy);
            if ((HeadingDifferenceDeg(Previous.HeadingDeg, BridgeHeadingDeg)
                    > CORRIDOR_BRIDGE_HEADING_TOLERANCE_DEG)
                || (HeadingDifferenceDeg(BridgeHeadingDeg, Next.HeadingDeg)
                    > CORRIDOR_BRIDGE_HEADING_TOLERANCE_DEG)) return null;
            return BuildDirectCorridorBridge(
                Previous.EndXy, Next.StartXy, SampleSpacingM, Field);
        }

        /// <summary>
        /// Executes the with internal cut poses operation.
        /// </summary>
        /// <param name="Source">The source used by the operation.</param>
        /// <param name="DirectEntry">The direct entry used by the operation.</param>
        /// <param name="DirectExit">The direct exit used by the operation.</param>
        /// <returns>The resulting with internal cut poses.</returns>

        private static RunSequencer.SequencedRun WithInternalCutPoses
            (
            RunSequencer.SequencedRun Source,
            Boolean DirectEntry,
            Boolean DirectExit
            ) => new RunSequencer.SequencedRun(
                Source.Run,
                DirectEntry ? WorkStartPose(Source.Run) : Source.EntryPose,
                DirectExit ? WorkEndPose(Source.Run) : Source.ExitPose,
                DirectEntry ? 0.0 : Source.EntryLineupM,
                DirectExit ? 0.0 : Source.ExitLineupM,
                Source.StartCoverageShortfallM,
                Source.EndCoverageShortfallM);

        /// <summary>
        /// Builds cut sequence result.
        /// </summary>
        /// <param name="Runs">The runs used by the operation.</param>
        /// <param name="Connectors">The connectors used by the operation.</param>
        /// <returns>The resulting cut sequence result.</returns>

        private static RunSequencer.Result BuildCutSequenceResult
            (
            List<RunSequencer.SequencedRun> Runs,
            List<TransitPlanner.Result> Connectors
            )
        {
            Double ConnectorDistanceM = Connectors.Sum(Connector => Connector.LengthM);
            Double MinimumLineupM = Runs.Min(Run =>
                Math.Min(Run.EntryLineupM, Run.ExitLineupM));
            return new RunSequencer.Result(
                true, Runs, Connectors, ConnectorDistanceM,
                MinimumLineupM, String.Empty);
        }

        /// <summary>
        /// Executes the anchor connector cost operation.
        /// </summary>
        /// <param name="Anchor">The anchor used by the operation.</param>
        /// <param name="RunPose">The run pose used by the operation.</param>
        /// <param name="MinimumRadiusM">The minimum radius m used by the operation.</param>
        /// <param name="SampleSpacingM">The sample spacing m used by the operation.</param>
        /// <param name="Field">The field used by the operation.</param>
        /// <param name="Reverse">The reverse used by the operation.</param>
        /// <returns>The resulting anchor connector cost.</returns>

        private static Double AnchorConnectorCost
            (
            ConnectorPlanner.Pose? Anchor,
            ConnectorPlanner.Pose RunPose,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            Boolean Reverse = false
            )
        {
            if (!Anchor.HasValue) return 0.0;
            TransitPlanner.Result Connector = Reverse
                ? TransitPlanner.Connect(
                    RunPose, Anchor.Value, MinimumRadiusM, SampleSpacingM, Field,
                    EdgeAllowanceM: Field.GridSizeM)
                : TransitPlanner.Connect(
                    Anchor.Value, RunPose, MinimumRadiusM, SampleSpacingM, Field,
                    EdgeAllowanceM: Field.GridSizeM);
            return Connector.Success ? ConnectorScoreM(Connector) : Double.PositiveInfinity;
        }

        /// <summary>
        /// Executes the connector score m operation.
        /// </summary>
        /// <param name="Connector">The connector used by the operation.</param>
        /// <returns>The resulting connector score m.</returns>

        private static Double ConnectorScoreM
            (
            TransitPlanner.Result Connector
            ) => Connector.LengthM + (Connector.OperatorAlignmentRequired
                ? MANUAL_CONNECTOR_PENALTY_M
                : 0.0);

        /// <summary>
        /// Determines whether corridor bridge.
        /// </summary>
        /// <param name="Connector">The connector used by the operation.</param>
        /// <returns>True when corridor bridge; otherwise false.</returns>

        private static Boolean IsCorridorBridge
            (
            TransitPlanner.Result Connector
            ) => Connector.Mode.Contains("CORRIDOR_BRIDGE", StringComparison.Ordinal);

        /// <summary>
        /// Stops the last fill at its already retreated work endpoint so a grade
        /// turnaround begins before an edge instead of consuming the exit lineup.
        /// </summary>
        private static RunSequencer.Result StopFinalRunAtWorkEnd
            (
            RunSequencer.Result Sequence
            )
        {
            List<RunSequencer.SequencedRun> Runs =
                new List<RunSequencer.SequencedRun>(Sequence.Runs);
            RunSequencer.SequencedRun Last = Runs[Runs.Count - 1];
            Runs[Runs.Count - 1] = new RunSequencer.SequencedRun(
                Last.Run,
                Last.EntryPose,
                new ConnectorPlanner.Pose(
                    Last.Run.EndXy.X,
                    Last.Run.EndXy.Y,
                    Last.Run.HeadingDeg),
                Last.EntryLineupM,
                0.0,
                Last.StartCoverageShortfallM,
                Last.EndCoverageShortfallM);
            return new RunSequencer.Result(
                Sequence.Success,
                Runs,
                Sequence.Connectors,
                Sequence.ConnectorDistanceM,
                0.0,
                Sequence.FailureReason);
        }

        /// <summary>
        /// Converts aligned fragments from one route into a continuous fill corridor.
        /// Only the first work fragment retains an entry lineup; internal fragments
        /// are connected directly at blade-up work endpoints and the final exit is
        /// reserved for the single fill-to-grade turnaround.
        /// </summary>
        static public RunSequencer.Result BuildContinuousFillSequence
            (
            RunSequencer.Result Sequence,
            ConnectorPlanner.Pose? Inbound,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if (Sequence == null) throw new ArgumentNullException(nameof(Sequence));
            if (!Sequence.Success || (Sequence.Runs.Count <= 1))
            {
                return Sequence.Runs.Count == 0
                    ? Sequence
                    : RetreatFinalFillForGradeTurn(
                        StopFinalRunAtWorkEnd(Sequence),
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field);
            }
            Sequence = RepackFillEnvelope(
                Sequence,
                Inbound,
                MinimumRadiusM,
                SampleSpacingM,
                Field);
            if (Sequence.Runs.Count <= 1)
            {
                return RetreatFinalFillForGradeTurn(
                    StopFinalRunAtWorkEnd(Sequence),
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field);
            }
            if (UsesCorridorBridges(Sequence))
            {
                return RetreatFinalFillForGradeTurn(
                    Sequence, MinimumRadiusM, SampleSpacingM, Field);
            }

            Double PreferredLineupM = Sequence.Runs.Max(Run => Math.Max(
                Run.EntryLineupM,
                Run.ExitLineupM));
            Sequence = MergeNestedFillFragments(
                Sequence,
                Inbound,
                PreferredLineupM,
                MinimumRadiusM,
                SampleSpacingM,
                Field);
            if (Sequence.Runs.Count <= 1)
            {
                return RetreatFinalFillForGradeTurn(
                    StopFinalRunAtWorkEnd(Sequence),
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field);
            }

            if (Inbound.HasValue)
            {
                RunSequencer.Result LoadedAwareSequence =
                    RunSequencer.Optimize(
                        Sequence.Runs.Select(Run => Run.Run).ToList(),
                        Inbound,
                        null,
                        PreferredLineupM,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        AvoidLoadedRunOverlap: true);
                if (LoadedAwareSequence.Success)
                {
                    Sequence = LoadedAwareSequence;
                }
            }

            RunSequencer.Result? Best = null;
            Double BestScoreM = Double.PositiveInfinity;
            Double FirstHeadingDeg = Sequence.Runs[0].Run.HeadingDeg;
            for (Int32 DirectionIndex = 0; DirectionIndex < 2; DirectionIndex += 1)
            {
                Double DirectionDeg = (FirstHeadingDeg + (Constants.DEGREES_PER_HALF_TURN * DirectionIndex)) % Constants.DEGREES_PER_TURN;
                if (!TryBuildCorridorCandidate(
                    Sequence,
                    Inbound,
                    DirectionDeg,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    out RunSequencer.Result Candidate,
                    out Double ScoreM))
                {
                    continue;
                }
                if (ScoreM + Constants.NUMERIC_COMPARISON_TOLERANCE < BestScoreM)
                {
                    Best = Candidate;
                    BestScoreM = ScoreM;
                }
            }
            return Best ?? RetreatFinalFillForGradeTurn(
                StopFinalRunAtWorkEnd(Sequence),
                MinimumRadiusM,
                SampleSpacingM,
                Field);
        }

        /// <summary>
        /// Converts close-proximity fragments into an oriented blade-width fill
        /// envelope. Allocation ownership stays on the original cells while the
        /// longer blade-down centerline represents temporary placement that the
        /// mandatory reverse grade redistributes to the final surface.
        /// </summary>
        private static RunSequencer.Result RepackFillEnvelope
            (
            RunSequencer.Result Sequence,
            ConnectorPlanner.Pose? Inbound,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if (!Sequence.Success
                || (Sequence.Runs.Count < FILL_ENVELOPE_MINIMUM_FRAGMENT_COUNT))
            {
                return Sequence;
            }
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                Sequence.Runs.SelectMany(Run => Run.Run.Cells).ToList();
            Double OriginalWorkM = Sequence.Runs.Sum(Run => Run.Run.LengthM);
            FieldEdgePolicy EdgePolicy = new FieldEdgePolicy(
                Field, Field.GridSizeM);
            RunSequencer.Result? Best = null;
            Double BestScoreM = Double.PositiveInfinity;
            Double BestHeadingDeg = Double.PositiveInfinity;
            List<(Double HeadingDeg, CoveragePlanner.PlanResult Envelope,
                Double ProxyScoreM)> Candidates =
                new List<(Double HeadingDeg, CoveragePlanner.PlanResult Envelope,
                    Double ProxyScoreM)>();
            foreach (Double HeadingDeg in FillEnvelopeCandidateHeadings(
                Cells, Sequence, EdgePolicy))
            {
                if (!Cells.All(Entry => EdgePolicy.IsParallelToEdge(
                    Entry.Cell, HeadingDeg, 7.5)))
                {
                    continue;
                }
                CoveragePlanner.PlanResult Envelope =
                    CoveragePlanner.BuildFillEnvelopeCandidate(
                        Cells,
                        Field,
                        Field.GridSizeM,
                        HeadingDeg,
                        FILL_ENVELOPE_MAX_ALONG_GAP_CELLS);
                if ((Envelope.Runs.Count >= Sequence.Runs.Count)
                    || (Envelope.WorkDistanceM > OriginalWorkM
                        * FILL_ENVELOPE_MAX_WORK_INFLATION)
                    || (Envelope.MaximumLateralErrorM
                        > (Constants.HALF_SCALE * Field.GridSizeM) + Constants.NUMERIC_COMPARISON_TOLERANCE))
                {
                    continue;
                }
                Double ProxyScoreM = 2.0 * (Envelope.WorkDistanceM
                    + Envelope.ConnectorDistanceM
                    + (Envelope.Runs.Count * Sequence.MinimumLineupM));
                Candidates.Add((HeadingDeg, Envelope, ProxyScoreM));
            }
            foreach ((Double HeadingDeg, CoveragePlanner.PlanResult Envelope,
                Double ProxyScoreM) Candidate in Candidates
                    .OrderBy(Candidate => Candidate.ProxyScoreM)
                    .ThenBy(Candidate => Candidate.Envelope.Runs.Count)
                    .ThenBy(Candidate => Candidate.HeadingDeg)
                    .Take(FILL_ENVELOPE_SEQUENCE_CANDIDATE_LIMIT))
            {
                Double HeadingDeg = Candidate.HeadingDeg;
                CoveragePlanner.PlanResult Envelope = Candidate.Envelope;
                RunSequencer.Result Repacked = RunSequencer.Optimize(
                    Envelope.Runs,
                    Inbound,
                    null,
                    Sequence.MinimumLineupM,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    AvoidLoadedRunOverlap: true);
                if (!Repacked.Success) continue;
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> RepackedCells =
                    Repacked.Runs.SelectMany(Run => Run.Run.Cells).ToList();
                if (!CoveragePlanner.ConservesAllocation(Cells, RepackedCells))
                {
                    continue;
                }
                RunSequencer.Result Prepared;
                if (Repacked.Runs.Count <= 1)
                {
                    Prepared = RetreatFinalFillForGradeTurn(
                        StopFinalRunAtWorkEnd(Repacked),
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field);
                }
                else
                {
                    RunSequencer.Result? BestCorridor = null;
                    Double BestCorridorScoreM = Double.PositiveInfinity;
                    for (Int32 DirectionIndex = 0;
                        DirectionIndex < 2;
                        DirectionIndex += 1)
                    {
                        Double DirectionDeg = (HeadingDeg
                            + (Constants.DEGREES_PER_HALF_TURN * DirectionIndex)) % Constants.DEGREES_PER_TURN;
                        if (TryBuildCorridorCandidate(
                            Repacked,
                            Inbound,
                            DirectionDeg,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            out RunSequencer.Result Corridor,
                            out Double CorridorScoreM)
                            && (CorridorScoreM + Constants.NUMERIC_COMPARISON_TOLERANCE < BestCorridorScoreM))
                        {
                            BestCorridor = Corridor;
                            BestCorridorScoreM = CorridorScoreM;
                        }
                    }
                    Prepared = BestCorridor
                        ?? RetreatFinalFillForGradeTurn(
                            StopFinalRunAtWorkEnd(Repacked),
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field);
                }
                Prepared = RetreatFinalFillForGradeTurn(
                    Prepared, MinimumRadiusM, SampleSpacingM, Field);
                if (!SupportsPreparedGradeReturn(
                    Prepared,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    out _))
                {
                    continue;
                }
                Double LineupDistanceM = Prepared.Runs.Sum(Run =>
                    Run.EntryLineupM + Run.ExitLineupM);
                Double ManualPenaltyM = Prepared.Connectors.Sum(Connector =>
                    Connector.ManualAssistDistanceM > Constants.NUMERIC_COMPARISON_TOLERANCE
                        ? MANUAL_CONNECTOR_PENALTY_M
                            + Connector.ManualAssistDistanceM
                        : 0.0);
                // Fill and reverse grade both traverse the envelope.  Include
                // lineups and inter-run movement so a very long one-run box does
                // not beat a shorter, better-oriented multi-run envelope merely
                // because its run count is smaller.
                Double ScoreM = 2.0 * (Envelope.WorkDistanceM
                    + Prepared.ConnectorDistanceM
                    + LineupDistanceM)
                    + ManualPenaltyM;
                if ((ScoreM + Constants.NUMERIC_COMPARISON_TOLERANCE < BestScoreM)
                    || ((Math.Abs(ScoreM - BestScoreM) <= Constants.NUMERIC_COMPARISON_TOLERANCE)
                        && (HeadingDeg < BestHeadingDeg)))
                {
                    Best = Prepared;
                    BestScoreM = ScoreM;
                    BestHeadingDeg = HeadingDeg;
                }
            }
            return Best ?? RepackFillEnvelopeComponents(
                Sequence,
                Cells,
                Inbound,
                OriginalWorkM,
                MinimumRadiusM,
                SampleSpacingM,
                Field,
                EdgePolicy);
        }

        /// <summary>
        /// Compacts separate close-proximity components independently.  This keeps
        /// an isolated difficult cell from forcing a poor common heading on a dense
        /// neighboring comb, while still sequencing and grading the exact union.
        /// </summary>
        private static RunSequencer.Result RepackFillEnvelopeComponents
            (
            RunSequencer.Result Sequence,
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            ConnectorPlanner.Pose? Inbound,
            Double OriginalWorkM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            FieldEdgePolicy EdgePolicy
            )
        {
            List<HashSet<(Int32 Bx, Int32 By)>> Components =
                BuildFillEnvelopeComponents(Cells);
            if (Components.Count <= 1) return Sequence;

            Dictionary<(Int32 Bx, Int32 By), Int32> ComponentByCell =
                new Dictionary<(Int32 Bx, Int32 By), Int32>();
            for (Int32 ComponentIndex = 0;
                ComponentIndex < Components.Count;
                ComponentIndex += 1)
            {
                foreach ((Int32 Bx, Int32 By) Cell in Components[ComponentIndex])
                {
                    ComponentByCell[Cell] = ComponentIndex;
                }
            }
            List<List<RunSequencer.SequencedRun>> RunsByComponent =
                Enumerable.Range(0, Components.Count)
                    .Select(_ => new List<RunSequencer.SequencedRun>())
                    .ToList();
            foreach (RunSequencer.SequencedRun Run in Sequence.Runs)
            {
                if (Run.Run.Cells.Count == 0) return Sequence;
                Int32 ComponentIndex = ComponentByCell[Run.Run.Cells[0].Cell];
                if (Run.Run.Cells.Any(Entry =>
                    ComponentByCell[Entry.Cell] != ComponentIndex))
                {
                    return Sequence;
                }
                RunsByComponent[ComponentIndex].Add(Run);
            }

            List<CoveragePlanner.Run> ReplacementRuns =
                new List<CoveragePlanner.Run>();
            Boolean Changed = false;
            for (Int32 ComponentIndex = 0;
                ComponentIndex < Components.Count;
                ComponentIndex += 1)
            {
                List<RunSequencer.SequencedRun> ComponentRuns =
                    RunsByComponent[ComponentIndex];
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> ComponentCells =
                    Cells.Where(Entry =>
                        ComponentByCell[Entry.Cell] == ComponentIndex).ToList();
                if (ComponentRuns.Count < FILL_ENVELOPE_MINIMUM_FRAGMENT_COUNT)
                {
                    ReplacementRuns.AddRange(ComponentRuns.Select(Run => Run.Run));
                    continue;
                }
                Double ComponentWorkM =
                    ComponentRuns.Sum(Run => Run.Run.LengthM);
                RunSequencer.Result ComponentSequence =
                    new RunSequencer.Result(
                        true,
                        ComponentRuns,
                        new List<TransitPlanner.Result>(),
                        0.0,
                        Sequence.MinimumLineupM,
                        String.Empty);
                CoveragePlanner.PlanResult? BestEnvelope =
                    FillEnvelopeCandidateHeadings(
                        ComponentCells, ComponentSequence, EdgePolicy)
                    .Where(HeadingDeg => ComponentCells.All(Entry =>
                        EdgePolicy.IsParallelToEdge(
                            Entry.Cell, HeadingDeg, 7.5)))
                    .Select(HeadingDeg =>
                        CoveragePlanner.BuildFillEnvelopeCandidate(
                            ComponentCells,
                            Field,
                            Field.GridSizeM,
                            HeadingDeg,
                            FILL_ENVELOPE_MAX_ALONG_GAP_CELLS))
                    .Where(Envelope =>
                        Envelope.Runs.Count < ComponentRuns.Count
                        && Envelope.WorkDistanceM <= ComponentWorkM
                            * FILL_ENVELOPE_MAX_WORK_INFLATION
                        && Envelope.MaximumLateralErrorM
                            <= (Constants.HALF_SCALE * Field.GridSizeM) + Constants.NUMERIC_COMPARISON_TOLERANCE)
                    .OrderBy(Envelope => 2.0 * (
                        Envelope.WorkDistanceM
                        + Envelope.ConnectorDistanceM
                        + (Envelope.Runs.Count * Sequence.MinimumLineupM)))
                    .ThenBy(Envelope => Envelope.Runs.Count)
                    .ThenBy(Envelope => Envelope.HeadingDeg)
                    .FirstOrDefault();
                if (BestEnvelope == null)
                {
                    ReplacementRuns.AddRange(ComponentRuns.Select(Run => Run.Run));
                    continue;
                }
                ReplacementRuns.AddRange(BestEnvelope.Runs);
                Changed = true;
            }
            if (!Changed || (ReplacementRuns.Count >= Sequence.Runs.Count))
            {
                return Sequence;
            }
            Double ReplacementWorkM =
                ReplacementRuns.Sum(Run => Run.LengthM);
            if (ReplacementWorkM > OriginalWorkM
                * FILL_ENVELOPE_MAX_WORK_INFLATION)
            {
                return Sequence;
            }
            RunSequencer.Result Repacked = RunSequencer.Optimize(
                ReplacementRuns,
                Inbound,
                null,
                Sequence.MinimumLineupM,
                MinimumRadiusM,
                SampleSpacingM,
                Field,
                AvoidLoadedRunOverlap: true);
            if (!Repacked.Success) return Sequence;
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> RepackedCells =
                Repacked.Runs.SelectMany(Run => Run.Run.Cells).ToList();
            if (!CoveragePlanner.ConservesAllocation(Cells, RepackedCells))
            {
                return Sequence;
            }
            RunSequencer.Result Prepared = RetreatFinalFillForGradeTurn(
                StopFinalRunAtWorkEnd(Repacked),
                MinimumRadiusM,
                SampleSpacingM,
                Field);
            return SupportsPreparedGradeReturn(
                Prepared,
                MinimumRadiusM,
                SampleSpacingM,
                Field,
                out _)
                ? Prepared
                : Sequence;
        }

        /// <summary>Builds connected components of allocation cells within the bounded gap.</summary>
        private static List<HashSet<(Int32 Bx, Int32 By)>> BuildFillEnvelopeComponents
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells
            )
        {
            HashSet<(Int32 Bx, Int32 By)> Remaining =
                Cells.Select(Entry => Entry.Cell).ToHashSet();
            List<HashSet<(Int32 Bx, Int32 By)>> Components =
                new List<HashSet<(Int32 Bx, Int32 By)>>();
            Double MaximumDistanceCells =
                FILL_ENVELOPE_MAX_ALONG_GAP_CELLS + 1.0;
            Double MaximumDistanceSquared =
                MaximumDistanceCells * MaximumDistanceCells;
            while (Remaining.Count > 0)
            {
                (Int32 Bx, Int32 By) Seed = Remaining.First();
                Remaining.Remove(Seed);
                HashSet<(Int32 Bx, Int32 By)> Component =
                    new HashSet<(Int32 Bx, Int32 By)> { Seed };
                Queue<(Int32 Bx, Int32 By)> Pending =
                    new Queue<(Int32 Bx, Int32 By)>();
                Pending.Enqueue(Seed);
                while (Pending.Count > 0)
                {
                    (Int32 Bx, Int32 By) Current = Pending.Dequeue();
                    List<(Int32 Bx, Int32 By)> Neighbors = Remaining
                        .Where(Candidate =>
                        {
                            Double Dx = Candidate.Bx - Current.Bx;
                            Double Dy = Candidate.By - Current.By;
                            return ((Dx * Dx) + (Dy * Dy))
                                <= MaximumDistanceSquared + Constants.NUMERIC_COMPARISON_TOLERANCE;
                        })
                        .ToList();
                    foreach ((Int32 Bx, Int32 By) Neighbor in Neighbors)
                    {
                        Remaining.Remove(Neighbor);
                        Component.Add(Neighbor);
                        Pending.Enqueue(Neighbor);
                    }
                }
                Components.Add(Component);
            }
            return Components;
        }

        /// <summary>
        /// Derives envelope axes from the allocation rather than assuming cardinal
        /// travel.  Principal axes describe the overall cluster; short pair links
        /// describe the dominant directions of its close-proximity concave shape.
        /// Existing run and mandatory edge headings remain candidates.
        /// </summary>
        private static List<Double> FillEnvelopeCandidateHeadings
            (
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells,
            RunSequencer.Result Sequence,
            FieldEdgePolicy EdgePolicy
            )
        {
            List<Double> Headings = new List<Double>();
            Action<Double> AddHeading = HeadingDeg =>
            {
                Double Normalized = NormalizeAxialHeading(HeadingDeg);
                if (!Headings.Any(Existing =>
                    FieldEdgePolicy.AxialHeadingDifferenceDeg(
                        Existing, Normalized) <= Constants.QUARTER_SCALE))
                {
                    Headings.Add(Normalized);
                }
            };
            foreach (RunSequencer.SequencedRun Run in Sequence.Runs)
            {
                AddHeading(Run.Run.HeadingDeg);
            }

            Double MeanX = Cells.Average(Entry => (Double)Entry.Cell.Bx);
            Double MeanY = Cells.Average(Entry => (Double)Entry.Cell.By);
            Double CovarianceXx = 0.0;
            Double CovarianceXy = 0.0;
            Double CovarianceYy = 0.0;
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                Double Dx = Entry.Cell.Bx - MeanX;
                Double Dy = Entry.Cell.By - MeanY;
                CovarianceXx += Dx * Dx;
                CovarianceXy += Dx * Dy;
                CovarianceYy += Dy * Dy;
            }
            if ((CovarianceXx + CovarianceYy) > Constants.NUMERIC_COMPARISON_TOLERANCE)
            {
                Double MajorAxisFromXRad = Constants.HALF_SCALE * Math.Atan2(
                    2.0 * CovarianceXy,
                    CovarianceXx - CovarianceYy);
                Double MajorHeadingDeg = Math.Atan2(
                    Math.Cos(MajorAxisFromXRad),
                    Math.Sin(MajorAxisFromXRad)) * Constants.DEGREES_PER_HALF_TURN / Math.PI;
                AddHeading(MajorHeadingDeg);
                AddHeading(MajorHeadingDeg + 90.0);
            }

            Int32 BinCount = (Int32)Math.Round(
                Constants.DEGREES_PER_HALF_TURN / FILL_ENVELOPE_HEADING_BIN_DEG);
            Double[] BinWeights = new Double[BinCount];
            Double[] BinSinTwice = new Double[BinCount];
            Double[] BinCosTwice = new Double[BinCount];
            Double MaximumPairDistanceCells =
                FILL_ENVELOPE_MAX_ALONG_GAP_CELLS + 1.0;
            for (Int32 First = 0; First < Cells.Count; First += 1)
            {
                for (Int32 Second = First + 1; Second < Cells.Count; Second += 1)
                {
                    Double Dx = Cells[Second].Cell.Bx - Cells[First].Cell.Bx;
                    Double Dy = Cells[Second].Cell.By - Cells[First].Cell.By;
                    Double DistanceCells = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                    if ((DistanceCells <= Constants.NUMERIC_COMPARISON_TOLERANCE)
                        || (DistanceCells > MaximumPairDistanceCells))
                    {
                        continue;
                    }
                    Double HeadingDeg = NormalizeAxialHeading(
                        Math.Atan2(Dx, Dy) * Constants.DEGREES_PER_HALF_TURN / Math.PI);
                    Int32 Bin = (Int32)Math.Floor(
                        (HeadingDeg + (Constants.HALF_SCALE * FILL_ENVELOPE_HEADING_BIN_DEG))
                            / FILL_ENVELOPE_HEADING_BIN_DEG) % BinCount;
                    Double Weight = 1.0 / DistanceCells;
                    Double TwiceRad = 2.0 * HeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
                    BinWeights[Bin] += Weight;
                    BinSinTwice[Bin] += Weight * Math.Sin(TwiceRad);
                    BinCosTwice[Bin] += Weight * Math.Cos(TwiceRad);
                }
            }
            foreach (Int32 Bin in Enumerable.Range(0, BinCount)
                .Where(Index => BinWeights[Index] > 0.0)
                .OrderByDescending(Index => BinWeights[Index])
                .ThenBy(Index => Index)
                .Take(FILL_ENVELOPE_PROXIMITY_HEADING_LIMIT))
            {
                AddHeading(Constants.HALF_SCALE * Math.Atan2(
                    BinSinTwice[Bin],
                    BinCosTwice[Bin]) * Constants.DEGREES_PER_HALF_TURN / Math.PI);
            }

            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Entry in Cells)
            {
                if (EdgePolicy.TryGetParallelHeadingDeg(
                    Entry.Cell, out Double EdgeHeadingDeg))
                {
                    AddHeading(EdgeHeadingDeg);
                }
            }
            return Headings.OrderBy(HeadingDeg => HeadingDeg).ToList();
        }

        /// <summary>Normalizes an undirected compass heading to [0, 180).</summary>
        private static Double NormalizeAxialHeading
            (
            Double HeadingDeg
            ) => ((HeadingDeg % Constants.DEGREES_PER_HALF_TURN) + Constants.DEGREES_PER_HALF_TURN) % Constants.DEGREES_PER_HALF_TURN;

        /// <summary>
        /// Stops the final fill early only when needed to leave enough forward
        /// clearance for the mandatory turn into reverse-direction grading.
        /// </summary>
        private static RunSequencer.Result RetreatFinalFillForGradeTurn
            (
            RunSequencer.Result Sequence,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            if (!Sequence.Success || (Sequence.Runs.Count == 0)) return Sequence;
            List<RunSequencer.SequencedRun> Runs =
                new List<RunSequencer.SequencedRun>(Sequence.Runs);
            RunSequencer.SequencedRun Original =
                Runs[Runs.Count - 1];
            Double MinimumWorkM = Math.Max(
                Constants.HALF_SCALE * Field.GridSizeM,
                SampleSpacingM);
            Double MaximumRetreatM = Math.Max(
                0.0,
                Original.Run.LengthM - MinimumWorkM);
            Double StepM = Math.Min(SampleSpacingM, Constants.HALF_SCALE * Field.GridSizeM);
            Double HeadingRad = Original.Run.HeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            Double UnitX = Math.Sin(HeadingRad);
            Double UnitY = Math.Cos(HeadingRad);
            for (Double RetreatM = 0.0;
                RetreatM <= MaximumRetreatM + Constants.NUMERIC_COMPARISON_TOLERANCE;
                RetreatM += StepM)
            {
                Double CandidateRetreatM = Math.Min(RetreatM, MaximumRetreatM);
                (Double X, Double Y) End = (
                    Original.Run.EndXy.X - (UnitX * CandidateRetreatM),
                    Original.Run.EndXy.Y - (UnitY * CandidateRetreatM));
                ConnectorPlanner.Pose FillExit =
                    new ConnectorPlanner.Pose(
                        End.X,
                        End.Y,
                        Original.Run.HeadingDeg);
                ConnectorPlanner.Pose GradeEntry =
                    new ConnectorPlanner.Pose(
                        End.X,
                        End.Y,
                        Original.Run.HeadingDeg + Constants.DEGREES_PER_HALF_TURN);
                TransitPlanner.Result GradeTurn =
                    TransitPlanner.Connect(
                        FillExit,
                        GradeEntry,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        EdgeAllowanceM: Field.GridSizeM,
                        AllowForwardDetour: true);
                if (!GradeTurn.Success)
                {
                    if (CandidateRetreatM >= MaximumRetreatM - Constants.NUMERIC_COMPARISON_TOLERANCE) break;
                    continue;
                }
                CoveragePlanner.Run RetreatedRun =
                    new CoveragePlanner.Run(
                        Original.Run.Cells,
                        Original.Run.StartXy,
                        End,
                        Original.Run.HeadingDeg,
                        Original.Run.MaximumLateralErrorM);
                Runs[Runs.Count - 1] =
                    new RunSequencer.SequencedRun(
                        RetreatedRun,
                        Original.EntryPose,
                        FillExit,
                        Original.EntryLineupM,
                        0.0,
                        Original.StartCoverageShortfallM,
                        Original.EndCoverageShortfallM + CandidateRetreatM);
                return new RunSequencer.Result(
                    true,
                    Runs,
                    Sequence.Connectors,
                    Sequence.ConnectorDistanceM,
                    Sequence.MinimumLineupM,
                    String.Empty);
            }
            return Sequence;
        }

        /// <summary>
        /// Absorbs a one-cell fill fragment whose complete centerline lies inside the
        /// blade-width corridor of a longer parallel run. The retained centerline is
        /// shifted halfway toward the fragment so one pass covers both allocations;
        /// this prevents an embedded sliver from forcing a loop before fill and again
        /// before reverse grading.
        /// </summary>
        private static RunSequencer.Result MergeNestedFillFragments
            (
            RunSequencer.Result Sequence,
            ConnectorPlanner.Pose? Inbound,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            List<RunSequencer.SequencedRun> Runs =
                new List<RunSequencer.SequencedRun>(Sequence.Runs);
            Boolean Changed = false;
            while (true)
            {
                Int32 FragmentIndex = -1;
                Int32 HostIndex = -1;
                RunSequencer.SequencedRun? Merged = null;
                for (Int32 ShortIndex = 0; ShortIndex < Runs.Count; ShortIndex += 1)
                {
                    RunSequencer.SequencedRun Fragment = Runs[ShortIndex];
                    if (Fragment.Run.LengthM
                        > NESTED_FILL_FRAGMENT_MAX_LENGTH_CELLS * Field.GridSizeM)
                    {
                        continue;
                    }
                    for (Int32 LongIndex = 0; LongIndex < Runs.Count; LongIndex += 1)
                    {
                        if (LongIndex == ShortIndex) continue;
                        RunSequencer.SequencedRun Host = Runs[LongIndex];
                        if (Host.Run.LengthM
                            < Fragment.Run.LengthM + (2.0 * Field.GridSizeM))
                        {
                            continue;
                        }
                        if (!TryMergeNestedFillFragment(
                            Host,
                            Fragment,
                            PreferredLineupM,
                            MinimumRadiusM,
                            SampleSpacingM,
                            Field,
                            out RunSequencer.SequencedRun Candidate))
                        {
                            continue;
                        }
                        FragmentIndex = ShortIndex;
                        HostIndex = LongIndex;
                        Merged = Candidate;
                        break;
                    }
                    if (Merged != null) break;
                }
                if ((Merged == null) || (FragmentIndex < 0) || (HostIndex < 0)) break;
                Runs[HostIndex] = Merged;
                Runs.RemoveAt(FragmentIndex);
                Changed = true;
            }
            if (!Changed) return Sequence;
            RunSequencer.Result Optimized =
                RunSequencer.Optimize(
                    Runs.Select(Run => Run.Run).ToList(),
                    Inbound,
                    null,
                    PreferredLineupM,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    AvoidLoadedRunOverlap: true);
            return Optimized.Success ? Optimized : Sequence;
        }

        /// <summary>Builds one blade-width merge of a nested parallel fragment.</summary>
        private static Boolean TryMergeNestedFillFragment
            (
            RunSequencer.SequencedRun Host,
            RunSequencer.SequencedRun Fragment,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            out RunSequencer.SequencedRun Merged
            )
        {
            Merged = Host;
            Double HeadingDifference = HeadingDifferenceDeg(
                Host.Run.HeadingDeg,
                Fragment.Run.HeadingDeg);
            if (Math.Min(HeadingDifference, Constants.DEGREES_PER_HALF_TURN - HeadingDifference)
                > CORRIDOR_HEADING_TOLERANCE_DEG)
            {
                return false;
            }
            Double Dx = Host.Run.EndXy.X - Host.Run.StartXy.X;
            Double Dy = Host.Run.EndXy.Y - Host.Run.StartXy.Y;
            Double LengthSquaredM = (Dx * Dx) + (Dy * Dy);
            if (LengthSquaredM <= Constants.NUMERIC_COMPARISON_TOLERANCE) return false;
            Double StartFraction = ((Fragment.Run.StartXy.X - Host.Run.StartXy.X) * Dx
                + (Fragment.Run.StartXy.Y - Host.Run.StartXy.Y) * Dy)
                / LengthSquaredM;
            Double EndFraction = ((Fragment.Run.EndXy.X - Host.Run.StartXy.X) * Dx
                + (Fragment.Run.EndXy.Y - Host.Run.StartXy.Y) * Dy)
                / LengthSquaredM;
            if ((StartFraction < -Constants.GEOMETRY_COMPARISON_TOLERANCE) || (StartFraction > 1.0 + Constants.GEOMETRY_COMPARISON_TOLERANCE)
                || (EndFraction < -Constants.GEOMETRY_COMPARISON_TOLERANCE) || (EndFraction > 1.0 + Constants.GEOMETRY_COMPARISON_TOLERANCE))
            {
                return false;
            }
            Double MaximumOffsetM = NESTED_FILL_FRAGMENT_MAX_OFFSET_CELLS
                * Field.GridSizeM;
            if ((PointToSegmentDistanceM(
                    Fragment.Run.StartXy, Host.Run.StartXy, Host.Run.EndXy)
                    > MaximumOffsetM)
                || (PointToSegmentDistanceM(
                    Fragment.Run.EndXy, Host.Run.StartXy, Host.Run.EndXy)
                    > MaximumOffsetM))
            {
                return false;
            }

            (Double X, Double Y) FragmentMidpoint = (
                Constants.HALF_SCALE * (Fragment.Run.StartXy.X + Fragment.Run.EndXy.X),
                Constants.HALF_SCALE * (Fragment.Run.StartXy.Y + Fragment.Run.EndXy.Y));
            Double MidpointFraction = ((FragmentMidpoint.X - Host.Run.StartXy.X) * Dx
                + (FragmentMidpoint.Y - Host.Run.StartXy.Y) * Dy)
                / LengthSquaredM;
            (Double X, Double Y) Projection = (
                Host.Run.StartXy.X + (MidpointFraction * Dx),
                Host.Run.StartXy.Y + (MidpointFraction * Dy));
            Double ShiftX = Constants.HALF_SCALE * (FragmentMidpoint.X - Projection.X);
            Double ShiftY = Constants.HALF_SCALE * (FragmentMidpoint.Y - Projection.Y);
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Host.Run.Cells);
            foreach (((Int32 Bx, Int32 By) Cell, Double DepthM) Cell
                in Fragment.Run.Cells)
            {
                if (!Cells.Any(Existing => Existing.Cell == Cell.Cell)) Cells.Add(Cell);
            }
            CoveragePlanner.Run Combined =
                new CoveragePlanner.Run(
                    Cells,
                    (Host.Run.StartXy.X + ShiftX, Host.Run.StartXy.Y + ShiftY),
                    (Host.Run.EndXy.X + ShiftX, Host.Run.EndXy.Y + ShiftY),
                    Host.Run.HeadingDeg,
                    Math.Max(Host.Run.MaximumLateralErrorM,
                        Fragment.Run.MaximumLateralErrorM));
            Merged = RunSequencer.Prepare(
                Combined,
                PreferredLineupM,
                MinimumRadiusM,
                SampleSpacingM,
                Field);
            return Merged.Run.LengthM > Constants.NUMERIC_COMPARISON_TOLERANCE;
        }

        /// <summary>Builds one consistently oriented, projection-ordered corridor candidate.</summary>
        private static Boolean TryBuildCorridorCandidate
            (
            RunSequencer.Result Sequence,
            ConnectorPlanner.Pose? Inbound,
            Double DirectionDeg,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            out RunSequencer.Result Candidate,
            out Double ScoreM
            )
        {
            Candidate = Sequence;
            ScoreM = Double.PositiveInfinity;
            Double DirectionRad = DirectionDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            Double UnitX = Math.Sin(DirectionRad);
            Double UnitY = Math.Cos(DirectionRad);
            Double PerpendicularX = UnitY;
            Double PerpendicularY = -UnitX;
            List<RunSequencer.SequencedRun> Oriented =
                new List<RunSequencer.SequencedRun>();
            foreach (RunSequencer.SequencedRun Source in Sequence.Runs)
            {
                RunSequencer.SequencedRun Run =
                    HeadingDifferenceDeg(Source.Run.HeadingDeg, DirectionDeg) <= 90.0
                        ? Source
                        : ReverseSequencedRun(Source);
                if (HeadingDifferenceDeg(Run.Run.HeadingDeg, DirectionDeg)
                    > CORRIDOR_HEADING_TOLERANCE_DEG)
                {
                    return false;
                }
                Oriented.Add(Run);
            }
            Oriented.Sort((Left, Right) => Projection(
                Left.Run.StartXy, UnitX, UnitY).CompareTo(Projection(
                    Right.Run.StartXy, UnitX, UnitY)));

            CoveragePlanner.Run Reference = Oriented[0].Run;
            Double ReferenceLateralM = Projection((
                Constants.HALF_SCALE * (Reference.StartXy.X + Reference.EndXy.X),
                Constants.HALF_SCALE * (Reference.StartXy.Y + Reference.EndXy.Y)),
                PerpendicularX,
                PerpendicularY);
            foreach (RunSequencer.SequencedRun Run in Oriented)
            {
                Double LateralM = Projection((
                    Constants.HALF_SCALE * (Run.Run.StartXy.X + Run.Run.EndXy.X),
                    Constants.HALF_SCALE * (Run.Run.StartXy.Y + Run.Run.EndXy.Y)),
                    PerpendicularX,
                    PerpendicularY);
                if (Math.Abs(LateralM - ReferenceLateralM)
                    > WORK_LANE_SHIFT_LATERAL_TOLERANCE_CELLS * Field.GridSizeM)
                {
                    return false;
                }
            }

            List<RunSequencer.SequencedRun> ContinuousRuns =
                new List<RunSequencer.SequencedRun>(Oriented);
            List<TransitPlanner.Result> Connectors =
                new List<TransitPlanner.Result>();
            Double ConnectorDistanceM = 0.0;
            for (Int32 Index = 1; Index < ContinuousRuns.Count; Index += 1)
            {
                RunSequencer.SequencedRun Previous = ContinuousRuns[Index - 1];
                RunSequencer.SequencedRun Next = ContinuousRuns[Index];
                Double DirectM = Distance(Previous.Run.EndXy, Next.Run.StartXy);
                if ((DirectM > CORRIDOR_GAP_TOLERANCE_CELLS * Field.GridSizeM)
                    || (DirectM <= Constants.NUMERIC_COMPARISON_TOLERANCE))
                {
                    return false;
                }
                Double BridgeHeadingDeg = BearingDeg(
                    Previous.Run.EndXy, Next.Run.StartXy);
                Boolean DirectBridge = (HeadingDifferenceDeg(
                        Previous.Run.HeadingDeg, BridgeHeadingDeg)
                            <= CORRIDOR_BRIDGE_HEADING_TOLERANCE_DEG)
                    && (HeadingDifferenceDeg(
                        BridgeHeadingDeg, Next.Run.HeadingDeg)
                            <= CORRIDOR_BRIDGE_HEADING_TOLERANCE_DEG);
                TransitPlanner.Result? Connector;
                if (DirectBridge)
                {
                    Connector = BuildDirectCorridorBridge(
                        Previous.Run.EndXy,
                        Next.Run.StartXy,
                        SampleSpacingM,
                        Field);
                }
                else if (TryBuildWorkLaneShift(
                    Previous,
                    Next,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    out RunSequencer.SequencedRun AdjustedPrevious,
                    out TransitPlanner.Result WorkConnector))
                {
                    ContinuousRuns[Index - 1] = AdjustedPrevious;
                    Connector = WorkConnector;
                }
                else
                {
                    return false;
                }
                if (Connector == null) return false;
                Connectors.Add(Connector);
                ConnectorDistanceM += Connector.LengthM;
            }

            List<RunSequencer.SequencedRun> Runs =
                new List<RunSequencer.SequencedRun>();
            for (Int32 Index = 0; Index < ContinuousRuns.Count; Index += 1)
            {
                RunSequencer.SequencedRun Source = ContinuousRuns[Index];
                Runs.Add(new RunSequencer.SequencedRun(
                    Source.Run,
                    Index == 0 ? Source.EntryPose : WorkStartPose(Source.Run),
                    WorkEndPose(Source.Run),
                    Index == 0 ? Source.EntryLineupM : 0.0,
                    0.0,
                    Source.StartCoverageShortfallM,
                    Source.EndCoverageShortfallM));
            }

            Double InboundM = 0.0;
            Double LoadedFillOverlapM = 0.0;
            if (Inbound.HasValue)
            {
                TransitPlanner.Result Incoming =
                    TransitPlanner.Connect(
                        Inbound.Value,
                        Runs[0].EntryPose,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        EdgeAllowanceM: Field.GridSizeM,
                        AllowForwardDetour: true);
                if (!Incoming.Success) return false;
                InboundM = Incoming.LengthM
                    + (Incoming.OperatorAlignmentRequired
                        ? MANUAL_CONNECTOR_PENALTY_M
                        : 0.0);
                LoadedFillOverlapM = AlignedLoadedOverlapM(
                    Incoming.Points, Runs, Field.GridSizeM);
            }
            RunSequencer.SequencedRun LastRun = Runs[Runs.Count - 1];
            ConnectorPlanner.Pose GradeEntry =
                new ConnectorPlanner.Pose(
                    LastRun.Run.EndXy.X,
                    LastRun.Run.EndXy.Y,
                    LastRun.Run.HeadingDeg + Constants.DEGREES_PER_HALF_TURN);
            TransitPlanner.Result GradeTurn =
                TransitPlanner.Connect(
                    LastRun.ExitPose,
                    GradeEntry,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    EdgeAllowanceM: Field.GridSizeM,
                    AllowForwardDetour: true);
            if (!GradeTurn.Success) return false;
            Double GradeTurnM = GradeTurn.LengthM
                + (GradeTurn.OperatorAlignmentRequired
                    ? MANUAL_CONNECTOR_PENALTY_M
                    : 0.0);
            Double LoadedOverlapPenaltyM = LoadedFillOverlapM
                >= Constants.HALF_SCALE * Field.GridSizeM
                    ? LOADED_FILL_OVERLAP_PENALTY_M + LoadedFillOverlapM
                    : 0.0;
            ScoreM = InboundM
                + (2.0 * ConnectorDistanceM)
                + GradeTurnM
                + LoadedOverlapPenaltyM;
            Candidate = new RunSequencer.Result(
                true,
                Runs,
                Connectors,
                ConnectorDistanceM,
                0.0,
                String.Empty);
            return true;
        }

        /// <summary>
        /// Builds a short direct bridge between compatible work fragments. This is
        /// deliberately not sent through the general turn planner: endpoint heading
        /// differences were already limited above and represent a slight correction,
        /// not a request for a full-radius loop.
        /// </summary>
        private static TransitPlanner.Result? BuildDirectCorridorBridge
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double SampleSpacingM,
            Grid Field
            )
        {
            List<(Double X, Double Y)> Points = SampleStraight(
                Start, End, SampleSpacingM);
            FieldEdgePolicy Policy =
                new FieldEdgePolicy(Field, Field.GridSizeM);
            if (!Points.TrueForAll(Point =>
                Policy.ContainsWithExteriorAllowance(Point.X, Point.Y)))
            {
                return null;
            }
            Boolean LeavesField = Points.Exists(Point =>
                !Field.ContainsWorldXy(Point.X, Point.Y));
            Double LengthM = Distance(Start, End);
            String Mode = LeavesField
                ? "MANUAL_EDGE_CORRIDOR_BRIDGE"
                : "AUTO_CORRIDOR_BRIDGE";
            return new TransitPlanner.Result(
                true,
                LeavesField,
                Points,
                LengthM,
                new List<TransitPlanner.Leg>
                {
                    new TransitPlanner.Leg(LeavesField, Points),
                },
                LeavesField ? LengthM : 0.0,
                Mode,
                String.Empty,
                LeavesField);
        }

        /// <summary>
        /// Borrows the end of a long work run to make a forward, curvature-valid
        /// blade-down lane shift into a nearby parallel fragment. The same sampled
        /// bridge is later retraced during grading, avoiding a pair of full loops.
        /// </summary>
        private static Boolean TryBuildWorkLaneShift
            (
            RunSequencer.SequencedRun Previous,
            RunSequencer.SequencedRun Next,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            out RunSequencer.SequencedRun AdjustedPrevious,
            out TransitPlanner.Result Connector
            )
        {
            AdjustedPrevious = Previous;
            Connector = new TransitPlanner.Result(
                false,
                false,
                new List<(Double X, Double Y)>(),
                0.0,
                new List<TransitPlanner.Leg>(),
                0.0,
                "FAILED",
                "No bounded work lane shift exists.");
            if (HeadingDifferenceDeg(
                Previous.Run.HeadingDeg,
                Next.Run.HeadingDeg) > CORRIDOR_HEADING_TOLERANCE_DEG)
            {
                return false;
            }

            Double HeadingRad = Previous.Run.HeadingDeg * Math.PI / Constants.DEGREES_PER_HALF_TURN;
            Double UnitX = Math.Sin(HeadingRad);
            Double UnitY = Math.Cos(HeadingRad);
            Double Dx = Next.Run.StartXy.X - Previous.Run.EndXy.X;
            Double Dy = Next.Run.StartXy.Y - Previous.Run.EndXy.Y;
            Double ForwardM = (Dx * UnitX) + (Dy * UnitY);
            Double LateralM = Math.Abs((-Dx * UnitY) + (Dy * UnitX));
            if ((ForwardM < -Constants.QUARTER_SCALE * Field.GridSizeM)
                || (LateralM <= CORRIDOR_LATERAL_TOLERANCE_CELLS * Field.GridSizeM)
                || (LateralM > WORK_LANE_SHIFT_LATERAL_TOLERANCE_CELLS
                    * Field.GridSizeM))
            {
                return false;
            }

            Double MinimumRemainingWorkM = Constants.HALF_SCALE * Field.GridSizeM;
            Double MaximumBorrowM = Math.Min(
                4.0 * MinimumRadiusM,
                Previous.Run.LengthM - MinimumRemainingWorkM);
            if (MaximumBorrowM <= SampleSpacingM) return false;
            Double BorrowStepM = Math.Max(
                SampleSpacingM,
                0.125 * Field.GridSizeM);
            for (Double BorrowM = BorrowStepM;
                BorrowM <= MaximumBorrowM + Constants.NUMERIC_COMPARISON_TOLERANCE;
                BorrowM += BorrowStepM)
            {
                Double CandidateBorrowM = Math.Min(BorrowM, MaximumBorrowM);
                (Double X, Double Y) ShiftStart = (
                    Previous.Run.EndXy.X - (UnitX * CandidateBorrowM),
                    Previous.Run.EndXy.Y - (UnitY * CandidateBorrowM));
                TransitPlanner.Result Candidate =
                    TransitPlanner.Connect(
                        new ConnectorPlanner.Pose(
                            ShiftStart.X,
                            ShiftStart.Y,
                            Previous.Run.HeadingDeg),
                        new ConnectorPlanner.Pose(
                            Next.Run.StartXy.X,
                            Next.Run.StartXy.Y,
                            Next.Run.HeadingDeg),
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        AllowManualAssist: false,
                        EdgeAllowanceM: Field.GridSizeM);
                Double DirectCandidateM = Distance(
                    ShiftStart,
                    Next.Run.StartXy);
                if (!Candidate.Success
                    || (Candidate.LengthM > 1.5 * DirectCandidateM + Constants.NUMERIC_COMPARISON_TOLERANCE))
                {
                    if (CandidateBorrowM >= MaximumBorrowM - Constants.NUMERIC_COMPARISON_TOLERANCE) break;
                    continue;
                }

                CoveragePlanner.Run AdjustedRun =
                    new CoveragePlanner.Run(
                        Previous.Run.Cells,
                        Previous.Run.StartXy,
                        ShiftStart,
                        Previous.Run.HeadingDeg,
                        Previous.Run.MaximumLateralErrorM);
                AdjustedPrevious = new RunSequencer.SequencedRun(
                    AdjustedRun,
                    Previous.EntryPose,
                    WorkEndPose(AdjustedRun),
                    Previous.EntryLineupM,
                    0.0,
                    Previous.StartCoverageShortfallM,
                    Previous.EndCoverageShortfallM);
                Connector = new TransitPlanner.Result(
                    true,
                    false,
                    Candidate.Points,
                    Candidate.LengthM,
                    Candidate.Legs,
                    0.0,
                    "AUTO_WORK_CORRIDOR_BRIDGE",
                    String.Empty);
                return true;
            }
            return false;
        }

        /// <summary>Samples one direct bridge densely enough for boundary validation.</summary>
        private static List<(Double X, Double Y)> SampleStraight
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double SampleSpacingM
            )
        {
            Double LengthM = Distance(Start, End);
            Int32 Steps = Math.Max(1, (Int32)Math.Ceiling(LengthM / SampleSpacingM));
            List<(Double X, Double Y)> Points =
                new List<(Double X, Double Y)>(Steps + 1);
            for (Int32 Index = 0; Index <= Steps; Index += 1)
            {
                Double T = (Double)Index / Steps;
                Points.Add((
                    Start.X + ((End.X - Start.X) * T),
                    Start.Y + ((End.Y - Start.Y) * T)));
            }
            return Points;
        }

        /// <summary>Reverses one sequenced run while retaining its available lineups.</summary>
        private static RunSequencer.SequencedRun ReverseSequencedRun
            (
            RunSequencer.SequencedRun Source
            )
        {
            List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Source.Run.Cells);
            Cells.Reverse();
            CoveragePlanner.Run Reversed = new CoveragePlanner.Run(
                Cells,
                Source.Run.EndXy,
                Source.Run.StartXy,
                (Source.Run.HeadingDeg + Constants.DEGREES_PER_HALF_TURN) % Constants.DEGREES_PER_TURN,
                Source.Run.MaximumLateralErrorM);
            return new RunSequencer.SequencedRun(
                Reversed,
                ReversePose(Source.ExitPose),
                ReversePose(Source.EntryPose),
                Source.ExitLineupM,
                Source.EntryLineupM,
                Source.EndCoverageShortfallM,
                Source.StartCoverageShortfallM);
        }

        /// <summary>Creates a work-start pose with no preceding lineup.</summary>
        private static ConnectorPlanner.Pose WorkStartPose
            (
            CoveragePlanner.Run Run
            ) => new ConnectorPlanner.Pose(
                Run.StartXy.X, Run.StartXy.Y, Run.HeadingDeg);

        /// <summary>Creates a work-end pose with no following exit lineup.</summary>
        private static ConnectorPlanner.Pose WorkEndPose
            (
            CoveragePlanner.Run Run
            ) => new ConnectorPlanner.Pose(
                Run.EndXy.X, Run.EndXy.Y, Run.HeadingDeg);

        /// <summary>Reverses a pose heading without moving its position.</summary>
        private static ConnectorPlanner.Pose ReversePose
            (
            ConnectorPlanner.Pose Pose
            ) => new ConnectorPlanner.Pose(
                Pose.X, Pose.Y, (Pose.HeadingDeg + Constants.DEGREES_PER_HALF_TURN) % Constants.DEGREES_PER_TURN);

        /// <summary>Projects one point onto a unit direction.</summary>
        private static Double Projection
            (
            (Double X, Double Y) Point,
            Double UnitX,
            Double UnitY
            ) => (Point.X * UnitX) + (Point.Y * UnitY);

        /// <summary>Returns Euclidean point distance.</summary>
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

        /// <summary>Returns the navigation bearing used by work-run headings.</summary>
        private static Double BearingDeg
            (
            (Double X, Double Y) A,
            (Double X, Double Y) B
            )
        {
            Double Bearing = Math.Atan2(B.X - A.X, B.Y - A.Y) * Constants.DEGREES_PER_HALF_TURN / Math.PI;
            return (Bearing + Constants.DEGREES_PER_TURN) % Constants.DEGREES_PER_TURN;
        }

        /// <summary>Returns the unsigned shortest heading difference.</summary>
        private static Double HeadingDifferenceDeg
            (
            Double A,
            Double B
            )
        {
            Double Delta = Math.Abs(((A - B + Constants.ANGLE_NORMALIZATION_OFFSET_DEGREES) % Constants.DEGREES_PER_TURN) - Constants.DEGREES_PER_HALF_TURN);
            return Delta;
        }

        /// <summary>
        /// Measures loaded inbound travel that already follows an assigned fill corridor.
        /// Such travel should normally become fill work instead of blade-up deadheading.
        /// </summary>
        private static Double AlignedLoadedOverlapM
            (
            List<(Double X, Double Y)> IncomingPoints,
            List<RunSequencer.SequencedRun> FillRuns,
            Double CorridorWidthM
            )
        {
            Double OverlapM = 0.0;
            Double MaximumLateralDistanceM = CorridorWidthM;
            for (Int32 Index = 1; Index < IncomingPoints.Count; Index += 1)
            {
                (Double X, Double Y) Start = IncomingPoints[Index - 1];
                (Double X, Double Y) End = IncomingPoints[Index];
                Double SegmentLengthM = Distance(Start, End);
                if (SegmentLengthM <= Constants.NUMERIC_COMPARISON_TOLERANCE) continue;

                Double SegmentHeadingDeg = BearingDeg(Start, End);
                (Double X, Double Y) Midpoint = (
                    Constants.HALF_SCALE * (Start.X + End.X),
                    Constants.HALF_SCALE * (Start.Y + End.Y));
                foreach (RunSequencer.SequencedRun FillRun in FillRuns)
                {
                    Double AlignmentDeg = HeadingDifferenceDeg(
                        SegmentHeadingDeg, FillRun.Run.HeadingDeg);
                    AlignmentDeg = Math.Min(AlignmentDeg, Constants.DEGREES_PER_HALF_TURN - AlignmentDeg);
                    if (AlignmentDeg > LOADED_FILL_ALIGNMENT_TOLERANCE_DEG)
                    {
                        continue;
                    }
                    if (PointToSegmentDistanceM(
                            Midpoint,
                            FillRun.Run.StartXy,
                            FillRun.Run.EndXy)
                        <= MaximumLateralDistanceM)
                    {
                        OverlapM += SegmentLengthM;
                        break;
                    }
                }
            }
            return OverlapM;
        }

        /// <summary>Returns the shortest distance from a point to a finite line segment.</summary>
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
            if (LengthSquaredM <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) return Distance(Point, Start);
            Double Fraction = ((Point.X - Start.X) * Dx
                + (Point.Y - Start.Y) * Dy) / LengthSquaredM;
            Fraction = Math.Max(0.0, Math.Min(1.0, Fraction));
            return Distance(Point, (
                Start.X + (Fraction * Dx),
                Start.Y + (Fraction * Dy)));
        }

        /// <summary>Appends grade runs with newly computed connectors.</summary>
        private static Boolean AppendPreparedRuns
            (
            List<Segment> Output,
            List<RunSequencer.SequencedRun> Runs,
            Phase LineupPhase,
            Phase WorkPhase,
            Phase ExitPhase,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            ref Int32 ManualAssistCount,
            out String FailureReason
            )
        {
            FailureReason = String.Empty;
            for (Int32 Index = 0; Index < Runs.Count; Index += 1)
            {
                if ((Index > 0) && !AppendTransit(
                    Output, ref ManualAssistCount, Runs[Index - 1].ExitPose,
                    Runs[Index].EntryPose, Phase.Transit, MinimumRadiusM, SampleSpacingM, Field,
                    out FailureReason))
                {
                    return false;
                }
                AppendPreparedRun(Output, Runs[Index], LineupPhase, WorkPhase, ExitPhase);
            }
            return true;
        }

        /// <summary>Returns true when every internal fill transition is a direct corridor bridge.</summary>
        private static Boolean UsesCorridorBridges
            (
            RunSequencer.Result Sequence
            )
        {
            if ((Sequence.Runs.Count <= 1)
                || (Sequence.Connectors.Count != Sequence.Runs.Count - 1))
            {
                return false;
            }
            foreach (TransitPlanner.Result Connector in Sequence.Connectors)
            {
                if (!Connector.Mode.EndsWith(
                    "CORRIDOR_BRIDGE", StringComparison.Ordinal)) return false;
            }
            return true;
        }

        /// <summary>
        /// Returns true when grading can safely retrace every accepted fill connector.
        /// Reversing a sampled forward-only automatic curve preserves its radius while
        /// avoiding a second, unrelated connector plan between the same work runs.
        /// </summary>
        private static Boolean CanRetraceFillConnectors
            (
            RunSequencer.Result Sequence
            )
        {
            if ((Sequence.Runs.Count <= 1)
                || (Sequence.Connectors.Count != Sequence.Runs.Count - 1))
            {
                return false;
            }
            foreach (TransitPlanner.Result Connector in Sequence.Connectors)
            {
                if (!Connector.Success || Connector.OperatorAlignmentRequired) return false;
            }
            return true;
        }

        /// <summary>
        /// Appends reverse-order grading while retracing each accepted fill bridge.
        /// This preserves one continuous grade phase without replanning short gaps as turns.
        /// </summary>
        private static Boolean AppendPreparedRunsWithReversedConnectors
            (
            List<Segment> Output,
            List<RunSequencer.SequencedRun> Runs,
            List<TransitPlanner.Result> FillConnectors,
            Phase LineupPhase,
            Phase WorkPhase,
            Phase ExitPhase,
            ref Int32 ManualAssistCount
            )
        {
            if (FillConnectors.Count != Runs.Count - 1) return false;
            for (Int32 Index = 0; Index < Runs.Count; Index += 1)
            {
                if (Index > 0)
                {
                    TransitPlanner.Result Connector = ReverseTransit(
                        FillConnectors[FillConnectors.Count - Index]);
                    AppendSequenceConnector(
                        Output,
                        ref ManualAssistCount,
                        Connector,
                        Phase.Transit,
                        WorkPhase,
                        Runs[Index - 1].EndCoverageShortfallM,
                        Runs[Index].StartCoverageShortfallM);
                }
                AppendPreparedRun(Output, Runs[Index], LineupPhase, WorkPhase, ExitPhase);
            }
            return true;
        }

        /// <summary>Reverses one fully sampled transit and all of its action legs.</summary>
        private static TransitPlanner.Result ReverseTransit
            (
            TransitPlanner.Result Source
            )
        {
            List<(Double X, Double Y)> Points =
                new List<(Double X, Double Y)>(Source.Points);
            Points.Reverse();
            List<TransitPlanner.Leg> Legs =
                new List<TransitPlanner.Leg>();
            for (Int32 Index = Source.Legs.Count - 1; Index >= 0; Index -= 1)
            {
                TransitPlanner.Leg SourceLeg = Source.Legs[Index];
                List<(Double X, Double Y)> LegPoints =
                    new List<(Double X, Double Y)>(SourceLeg.Points);
                LegPoints.Reverse();
                Legs.Add(new TransitPlanner.Leg(
                    SourceLeg.OperatorAlignmentRequired, LegPoints));
            }
            return new TransitPlanner.Result(
                Source.Success,
                Source.OperatorAlignmentRequired,
                Points,
                Source.LengthM,
                Legs,
                Source.ManualAssistDistanceM,
                "REVERSE_" + Source.Mode,
                Source.FailureReason,
                Source.LeavesField);
        }

        /// <summary>Appends entry lineup, blade-down work, and exit lineup.</summary>
        private static void AppendPreparedRun
            (
            List<Segment> Output,
            RunSequencer.SequencedRun Run,
            Phase LineupPhase,
            Phase WorkPhase,
            Phase ExitPhase
            )
        {
            Output.Add(new Segment(LineupPhase, false, new List<(Double X, Double Y)>
            {
                (Run.EntryPose.X, Run.EntryPose.Y),
                Run.Run.StartXy,
            }));
            Output.Add(new Segment(WorkPhase, true, new List<(Double X, Double Y)>
            {
                Run.Run.StartXy,
                Run.Run.EndXy,
            }));
            Output.Add(new Segment(ExitPhase, false, new List<(Double X, Double Y)>
            {
                Run.Run.EndXy,
                (Run.ExitPose.X, Run.ExitPose.Y),
            }));
        }

        /// <summary>Builds reverse-drive-order grade runs over completed fill work.</summary>
        static public List<RunSequencer.SequencedRun> BuildGradeRuns
            (
            List<RunSequencer.SequencedRun> FillRuns,
            Double PreferredLineupM,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            List<RunSequencer.SequencedRun> Output =
                new List<RunSequencer.SequencedRun>();
            for (Int32 Index = FillRuns.Count - 1; Index >= 0; Index -= 1)
            {
                Output.Add(ReverseSequencedRun(FillRuns[Index]));
            }
            return Output;
        }

        /// <summary>
        /// Reverses the complete drive order of a prepared work sequence.  Every
        /// work run, sampled connector, lineup, and coverage-shortfall endpoint
        /// is reversed without changing the assigned cells or depths.
        /// </summary>
        static internal RunSequencer.Result ReversePreparedSequence
            (
            RunSequencer.Result Sequence
            )
        {
            if (Sequence == null) throw new ArgumentNullException(nameof(Sequence));
            List<RunSequencer.SequencedRun> Runs = Sequence.Runs
                .AsEnumerable()
                .Reverse()
                .Select(ReverseSequencedRun)
                .ToList();
            List<TransitPlanner.Result> Connectors = Sequence.Connectors
                .AsEnumerable()
                .Reverse()
                .Select(ReverseTransit)
                .ToList();
            return new RunSequencer.Result(
                Sequence.Success,
                Runs,
                Connectors,
                Sequence.ConnectorDistanceM,
                Sequence.MinimumLineupM,
                Sequence.FailureReason,
                Sequence.InfeasibleRunIndices);
        }

        /// <summary>
        /// Checks the forward-only turnaround and reverse-order connectors needed to
        /// grade an already prepared fill sequence.  Unlike <see cref="Build"/>, this
        /// does not rebuild the fill sequence or allocate lifecycle segments merely
        /// to answer a feasibility question.
        /// </summary>
        static public Boolean SupportsPreparedGradeReturn
            (
            RunSequencer.Result FillSequence,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            out String FailureReason,
            Boolean RetainResults = false
            )
        {
            FailureReason = String.Empty;
            if (!FillSequence.Success)
            {
                FailureReason = FillSequence.FailureReason;
                return false;
            }
            if (FillSequence.Runs.Count == 0) return true;

            List<RunSequencer.SequencedRun> GradeRuns = BuildGradeRuns(
                FillSequence.Runs,
                0.0,
                MinimumRadiusM,
                SampleSpacingM,
                Field);
            TransitPlanner.Result TurnToGrade =
                TransitPlanner.Connect(
                    FillSequence.Runs[FillSequence.Runs.Count - 1].ExitPose,
                    GradeRuns[0].EntryPose,
                    MinimumRadiusM,
                    SampleSpacingM,
                    Field,
                    EdgeAllowanceM: Field.GridSizeM,
                    AllowForwardDetour: true,
                    RetainResult: RetainResults);
            if (!TurnToGrade.Success)
            {
                FailureReason = TurnToGrade.FailureReason;
                return false;
            }

            if (CanRetraceFillConnectors(FillSequence)) return true;
            for (Int32 Index = 1; Index < GradeRuns.Count; Index += 1)
            {
                TransitPlanner.Result Connector =
                    TransitPlanner.Connect(
                        GradeRuns[Index - 1].ExitPose,
                        GradeRuns[Index].EntryPose,
                        MinimumRadiusM,
                        SampleSpacingM,
                        Field,
                        EdgeAllowanceM: Field.GridSizeM,
                        AllowForwardDetour: true,
                        RetainResult: RetainResults);
                if (Connector.Success) continue;
                FailureReason = Connector.FailureReason;
                return false;
            }
            return true;
        }

        /// <summary>Plans and appends one lifecycle transit.</summary>
        private static Boolean AppendTransit
            (
            List<Segment> Output,
            ref Int32 ManualAssistCount,
            ConnectorPlanner.Pose Start,
            ConnectorPlanner.Pose End,
            Phase TransitPhase,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            out String FailureReason
            )
        {
            TransitPlanner.Result Transit = TransitPlanner.Connect(
                Start,
                End,
                MinimumRadiusM,
                SampleSpacingM,
                Field,
                EdgeAllowanceM: Field.GridSizeM,
                AllowForwardDetour: true);
            FailureReason = Transit.FailureReason;
            if (!Transit.Success) return false;
            AppendTransitResult(Output, ref ManualAssistCount, Transit, TransitPhase);
            return true;
        }

        /// <summary>Appends a transit under its automatic or explicit manual-assist phase.</summary>
        private static void AppendTransitResult
            (
            List<Segment> Output,
            ref Int32 ManualAssistCount,
            TransitPlanner.Result Transit,
            Phase TransitPhase
            )
        {
            foreach (TransitPlanner.Leg Leg in Transit.Legs)
            {
                Phase SelectedPhase = Leg.OperatorAlignmentRequired
                    ? Phase.ManualAssist
                    : TransitPhase;
                if (Leg.OperatorAlignmentRequired) ManualAssistCount += 1;
                Output.Add(new Segment(SelectedPhase, false, Leg.Points));
            }
        }

        /// <summary>
        /// Appends a normal blade-up connector or a curvature-valid blade-down work
        /// bridge that was intentionally borrowed from adjacent fill endpoints.
        /// </summary>
        private static void AppendSequenceConnector
            (
            List<Segment> Output,
            ref Int32 ManualAssistCount,
            TransitPlanner.Result Connector,
            Phase TransitPhase,
            Phase WorkPhase,
            Double PreviousEndWorkM = 0.0,
            Double NextStartWorkM = 0.0
            )
        {
            if (Connector.Mode.Contains(
                "WORK_CORRIDOR_BRIDGE",
                StringComparison.Ordinal))
            {
                foreach (TransitPlanner.Leg Leg in Connector.Legs)
                {
                    if (Leg.OperatorAlignmentRequired)
                    {
                        ManualAssistCount += 1;
                        Output.Add(new Segment(Phase.ManualAssist, false, Leg.Points));
                    }
                    else
                    {
                        Output.Add(new Segment(WorkPhase, true, Leg.Points));
                    }
                }
                return;
            }

            if (Connector.OperatorAlignmentRequired
                || ((PreviousEndWorkM <= Constants.NUMERIC_COMPARISON_TOLERANCE) && (NextStartWorkM <= Constants.NUMERIC_COMPARISON_TOLERANCE)))
            {
                AppendTransitResult(
                    Output, ref ManualAssistCount, Connector, TransitPhase);
                return;
            }

            Double LengthM = PolylineLength(Connector.Points);
            if (LengthM <= Constants.NUMERIC_COMPARISON_TOLERANCE)
            {
                AppendTransitResult(
                    Output, ref ManualAssistCount, Connector, TransitPhase);
                return;
            }
            Double RequestedWorkM = Math.Max(0.0, PreviousEndWorkM)
                + Math.Max(0.0, NextStartWorkM);
            Double Scale = RequestedWorkM > LengthM
                ? LengthM / RequestedWorkM
                : 1.0;
            Double PrefixWorkM = Math.Max(0.0, PreviousEndWorkM) * Scale;
            Double SuffixWorkM = Math.Max(0.0, NextStartWorkM) * Scale;
            Double TransitStartM = PrefixWorkM;
            Double TransitEndM = LengthM - SuffixWorkM;
            AppendPolylineSlice(Output, Connector.Points, 0.0, TransitStartM,
                WorkPhase, true);
            AppendPolylineSlice(Output, Connector.Points, TransitStartM, TransitEndM,
                TransitPhase, false);
            AppendPolylineSlice(Output, Connector.Points, TransitEndM, LengthM,
                WorkPhase, true);
        }

        /// <summary>Appends a non-empty arc-length slice of a sampled connector.</summary>
        private static void AppendPolylineSlice
            (
            List<Segment> Output,
            List<(Double X, Double Y)> Points,
            Double StartM,
            Double EndM,
            Phase SelectedPhase,
            Boolean BladeDown
            )
        {
            List<(Double X, Double Y)> Slice = PolylineSlice(Points, StartM, EndM);
            if ((Slice.Count >= 2) && (PolylineLength(Slice) > Constants.NUMERIC_COMPARISON_TOLERANCE))
            {
                Output.Add(new Segment(SelectedPhase, BladeDown, Slice));
            }
        }

        /// <summary>Returns an interpolated arc-length slice of a polyline.</summary>
        private static List<(Double X, Double Y)> PolylineSlice
            (
            List<(Double X, Double Y)> Points,
            Double StartM,
            Double EndM
            )
        {
            List<(Double X, Double Y)> Output = new List<(Double X, Double Y)>();
            if (Points.Count == 0) return Output;
            Double TotalM = PolylineLength(Points);
            Double ClampedStartM = Math.Max(0.0, Math.Min(StartM, TotalM));
            Double ClampedEndM = Math.Max(ClampedStartM, Math.Min(EndM, TotalM));
            Double TravelM = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                (Double X, Double Y) A = Points[Index - 1];
                (Double X, Double Y) B = Points[Index];
                Double Dx = B.X - A.X;
                Double Dy = B.Y - A.Y;
                Double SegmentM = Math.Sqrt((Dx * Dx) + (Dy * Dy));
                if (SegmentM <= Constants.STRICT_NUMERIC_COMPARISON_TOLERANCE) continue;
                Double SegmentStartM = TravelM;
                Double SegmentEndM = TravelM + SegmentM;
                if ((SegmentEndM + Constants.NUMERIC_COMPARISON_TOLERANCE >= ClampedStartM)
                    && (SegmentStartM - Constants.NUMERIC_COMPARISON_TOLERANCE <= ClampedEndM))
                {
                    Double LocalStartM = Math.Max(ClampedStartM, SegmentStartM);
                    Double LocalEndM = Math.Min(ClampedEndM, SegmentEndM);
                    if (LocalEndM + Constants.NUMERIC_COMPARISON_TOLERANCE >= LocalStartM)
                    {
                        (Double X, Double Y) SliceStart = InterpolatePoint(
                            A, B, (LocalStartM - SegmentStartM) / SegmentM);
                        (Double X, Double Y) SliceEnd = InterpolatePoint(
                            A, B, (LocalEndM - SegmentStartM) / SegmentM);
                        if ((Output.Count == 0)
                            || (Distance(Output[Output.Count - 1], SliceStart) > Constants.NUMERIC_COMPARISON_TOLERANCE))
                        {
                            Output.Add(SliceStart);
                        }
                        if ((Output.Count == 0)
                            || (Distance(Output[Output.Count - 1], SliceEnd) > Constants.NUMERIC_COMPARISON_TOLERANCE))
                        {
                            Output.Add(SliceEnd);
                        }
                    }
                }
                TravelM = SegmentEndM;
                if (TravelM > ClampedEndM + Constants.NUMERIC_COMPARISON_TOLERANCE) break;
            }
            return Output;
        }

        /// <summary>Returns the point at a clamped fraction along one segment.</summary>
        private static (Double X, Double Y) InterpolatePoint
            (
            (Double X, Double Y) Start,
            (Double X, Double Y) End,
            Double Fraction
            )
        {
            Double Clamped = Math.Max(0.0, Math.Min(1.0, Fraction));
            return (
                Start.X + ((End.X - Start.X) * Clamped),
                Start.Y + ((End.Y - Start.Y) * Clamped));
        }

        /// <summary>Computes sampled polyline length.</summary>
        private static Double PolylineLength
            (
            List<(Double X, Double Y)> Points
            )
        {
            Double LengthM = 0.0;
            for (Int32 Index = 1; Index < Points.Count; Index += 1)
            {
                LengthM += Distance(Points[Index - 1], Points[Index]);
            }
            return LengthM;
        }

        /// <summary>Computes total segment distance without counting shared endpoints twice.</summary>
        private static Double TotalDistance
            (
            List<Segment> Segments
            )
        {
            Double TotalM = 0.0;
            foreach (Segment Segment in Segments)
            {
                for (Int32 Index = 1; Index < Segment.Points.Count; Index += 1)
                {
                    Double Dx = Segment.Points[Index].X - Segment.Points[Index - 1].X;
                    Double Dy = Segment.Points[Index].Y - Segment.Points[Index - 1].Y;
                    TotalM += Math.Sqrt((Dx * Dx) + (Dy * Dy));
                }
            }
            return TotalM;
        }

        /// <summary>Creates a failed partial lifecycle for diagnostics.</summary>
        private static Result Failure
            (
            List<Segment> Segments,
            Int32 ManualAssistCount,
            String Reason
            )
        {
            return new Result(false, Segments, ManualAssistCount,
                TotalDistance(Segments), Reason);
        }
    }
}
