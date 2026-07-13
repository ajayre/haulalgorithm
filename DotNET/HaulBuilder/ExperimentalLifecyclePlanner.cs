using System;
using System.Collections.Generic;

namespace HaulBuilder
{
    /// <summary>
    /// Builds the isolated cut, fill, optional grade, and next-cut guidance lifecycle.
    /// </summary>
    public static class ExperimentalLifecyclePlanner
    {
        /// <summary>
        /// Strongly typed experimental guidance phases.
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
                ExperimentalRunSequencer.Result Cut,
                ExperimentalRunSequencer.Result Fill
                )
            {
                this.Cut = Cut;
                this.Fill = Fill;
            }

            /// <summary>Gets sequenced cut work.</summary>
            public ExperimentalRunSequencer.Result Cut { get; }

            /// <summary>Gets sequenced fill work.</summary>
            public ExperimentalRunSequencer.Result Fill { get; }
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
                )
            {
                this.Phase = Phase;
                this.BladeDown = BladeDown;
                this.Points = Points;
            }

            /// <summary>Gets the strongly typed phase.</summary>
            public Phase Phase { get; }

            /// <summary>Gets whether this is blade-down work.</summary>
            public Boolean BladeDown { get; }

            /// <summary>Gets ordered centerline points.</summary>
            public List<(Double X, Double Y)> Points { get; }
        }

        /// <summary>
        /// Complete experimental lifecycle result.
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
            Grid Field
            )
        {
            if (Passes == null) throw new ArgumentNullException(nameof(Passes));
            List<Segment> Segments = new List<Segment>();
            Int32 ManualAssistCount = 0;
            ExperimentalConnectorPlanner.Pose? Current = null;

            for (Int32 PassIndex = 0; PassIndex < Passes.Count; PassIndex += 1)
            {
                PassPlan Pass = Passes[PassIndex];
                if (!Pass.Cut.Success || !Pass.Fill.Success)
                {
                    return Failure(Segments, ManualAssistCount,
                        String.Format(System.Globalization.CultureInfo.InvariantCulture,
                            "Pass {0} cut or fill sequencing failed before lifecycle construction.",
                            PassIndex + 1));
                }

                if (Pass.Cut.Runs.Count > 0)
                {
                    ExperimentalConnectorPlanner.Pose CutEntry = Pass.Cut.Runs[0].EntryPose;
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
                    AppendWorkSequence(Segments, Pass.Cut, Phase.LineupCut,
                        Phase.Cut, Phase.ExitCut, Phase.Transit, ref ManualAssistCount);
                    Current = Pass.Cut.Runs[Pass.Cut.Runs.Count - 1].ExitPose;
                }

                if (Pass.Fill.Runs.Count > 0)
                {
                    ExperimentalConnectorPlanner.Pose FillEntry = Pass.Fill.Runs[0].EntryPose;
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
                    AppendWorkSequence(Segments, Pass.Fill, Phase.LineupFill,
                        Phase.Fill, Phase.ExitFill, Phase.Transit, ref ManualAssistCount);
                    Current = Pass.Fill.Runs[Pass.Fill.Runs.Count - 1].ExitPose;
                }

                if (GradeReturn && (Pass.Fill.Runs.Count > 0))
                {
                    List<ExperimentalRunSequencer.SequencedRun> GradeRuns = BuildGradeRuns(
                        Pass.Fill.Runs, PreferredLineupM, SampleSpacingM, Field);
                    ExperimentalConnectorPlanner.Pose GradeEntry = GradeRuns[0].EntryPose;
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
                    if (!AppendPreparedRuns(Segments, GradeRuns, Phase.LineupGrade,
                        Phase.Grade, Phase.ExitGrade, MinimumRadiusM, SampleSpacingM,
                        Field, ref ManualAssistCount, out String GradeFailureReason))
                    {
                        return Failure(Segments, ManualAssistCount,
                            String.Format(System.Globalization.CultureInfo.InvariantCulture,
                                "Pass {0} disconnected grade runs cannot be connected in-field: {1}",
                                PassIndex + 1,
                                GradeFailureReason));
                    }
                    Current = GradeRuns[GradeRuns.Count - 1].ExitPose;
                }
            }

            if (Current.HasValue)
            {
                Segments.Add(new Segment(Phase.End, false,
                    new List<(Double X, Double Y)> { (Current.Value.X, Current.Value.Y) }));
            }
            return new Result(true, Segments, ManualAssistCount,
                TotalDistance(Segments), String.Empty);
        }

        /// <summary>Appends cut or fill runs and their already validated internal connectors.</summary>
        private static void AppendWorkSequence
            (
            List<Segment> Output,
            ExperimentalRunSequencer.Result Sequence,
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
                    ExperimentalTransitPlanner.Result Connector = Sequence.Connectors[Index - 1];
                    AppendTransitResult(Output, ref ManualAssistCount, Connector, TransitPhase);
                }
                AppendPreparedRun(Output, Sequence.Runs[Index], LineupPhase, WorkPhase, ExitPhase);
            }
        }

        /// <summary>Appends grade runs with newly computed connectors.</summary>
        private static Boolean AppendPreparedRuns
            (
            List<Segment> Output,
            List<ExperimentalRunSequencer.SequencedRun> Runs,
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

        /// <summary>Appends entry lineup, blade-down work, and exit lineup.</summary>
        private static void AppendPreparedRun
            (
            List<Segment> Output,
            ExperimentalRunSequencer.SequencedRun Run,
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
        private static List<ExperimentalRunSequencer.SequencedRun> BuildGradeRuns
            (
            List<ExperimentalRunSequencer.SequencedRun> FillRuns,
            Double PreferredLineupM,
            Double SampleSpacingM,
            Grid Field
            )
        {
            List<ExperimentalRunSequencer.SequencedRun> Output =
                new List<ExperimentalRunSequencer.SequencedRun>();
            for (Int32 Index = FillRuns.Count - 1; Index >= 0; Index -= 1)
            {
                ExperimentalCoveragePlanner.Run Fill = FillRuns[Index].Run;
                List<((Int32 Bx, Int32 By) Cell, Double DepthM)> Cells =
                    new List<((Int32 Bx, Int32 By) Cell, Double DepthM)>(Fill.Cells);
                Cells.Reverse();
                ExperimentalCoveragePlanner.Run Grade = new ExperimentalCoveragePlanner.Run(
                    Cells,
                    Fill.EndXy,
                    Fill.StartXy,
                    (Fill.HeadingDeg + 180.0) % 360.0,
                    Fill.MaximumLateralErrorM);
                Output.Add(ExperimentalRunSequencer.Prepare(
                    Grade, PreferredLineupM, SampleSpacingM, Field));
            }
            return Output;
        }

        /// <summary>Plans and appends one lifecycle transit.</summary>
        private static Boolean AppendTransit
            (
            List<Segment> Output,
            ref Int32 ManualAssistCount,
            ExperimentalConnectorPlanner.Pose Start,
            ExperimentalConnectorPlanner.Pose End,
            Phase TransitPhase,
            Double MinimumRadiusM,
            Double SampleSpacingM,
            Grid Field,
            out String FailureReason
            )
        {
            ExperimentalTransitPlanner.Result Transit = ExperimentalTransitPlanner.Connect(
                Start, End, MinimumRadiusM, SampleSpacingM, Field);
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
            ExperimentalTransitPlanner.Result Transit,
            Phase TransitPhase
            )
        {
            foreach (ExperimentalTransitPlanner.Leg Leg in Transit.Legs)
            {
                Phase SelectedPhase = Leg.OperatorAlignmentRequired
                    ? Phase.ManualAssist
                    : TransitPhase;
                if (Leg.OperatorAlignmentRequired) ManualAssistCount += 1;
                Output.Add(new Segment(SelectedPhase, false, Leg.Points));
            }
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
