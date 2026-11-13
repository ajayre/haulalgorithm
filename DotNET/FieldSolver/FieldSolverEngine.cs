using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// Searches for the HaulBuilder settings that give one field the shortest
    /// whole-job driving distance. It runs the HaulBuilder engine once per
    /// candidate, one run at a time, and returns the best settings found together
    /// with every run it made. It returns settings, not a plan: the caller builds
    /// the plan by running HaulBuilder with <see cref="FieldSolverResult.BestOptions"/>.
    /// </summary>
    public sealed class FieldSolverEngine
    {
        /// <summary>Percent scale.</summary>
        private const Double PERCENT = 100.0;

        /// <summary>Value of <see cref="_ActiveSolveCount"/> while no solve is running.</summary>
        private const Int32 IDLE = 0;

        /// <summary>Value of <see cref="_ActiveSolveCount"/> while a solve is running.</summary>
        private const Int32 BUSY = 1;

        /// <summary>Guards the process-wide rule of one solve at a time.</summary>
        static private Int32 _ActiveSolveCount;

        /// <summary>Builds and measures one plan.</summary>
        private readonly ICandidateRunner _Runner;

        /// <summary>Initializes a solver that runs the HaulBuilder engine.</summary>
        public FieldSolverEngine
            (
            )
            : this(new HaulBuilderCandidateRunner())
        {
        }

        /// <summary>Initializes a solver with a substitute runner, for tests.</summary>
        /// <param name="Runner">Runner that builds and measures one plan.</param>
        /// <exception cref="ArgumentNullException">Thrown when runner is null.</exception>
        internal FieldSolverEngine
            (
            ICandidateRunner Runner
            )
        {
            if (Runner == null) throw new ArgumentNullException(nameof(Runner));
            _Runner = Runner;
        }

        /// <summary>
        /// Runs the search. The call blocks until the search ends, so a user
        /// interface must call it from a worker thread.
        /// </summary>
        /// <param name="Request">Field, equipment, values to try, and limits.</param>
        /// <param name="Progress">Optional progress sink. It is called from the solving thread and from engine worker threads.</param>
        /// <param name="CancellationToken">Token that stops the search. The engine observes it only while it builds guidance.</param>
        /// <returns>The best settings found and every run made. Returned for every status once the baseline run has finished.</returns>
        /// <exception cref="ArgumentNullException">Thrown when request is null.</exception>
        /// <exception cref="ArgumentException">Thrown when the request or a search value is invalid.</exception>
        /// <exception cref="InvalidOperationException">Thrown when another solve is running in this process, or when the baseline run fails inside the engine.</exception>
        /// <exception cref="OperationCanceledException">Thrown when the caller cancels before the baseline run has finished.</exception>
        /// <exception cref="TimeoutException">Thrown when the time limit ends before the baseline run has finished.</exception>
        public FieldSolverResult Solve
            (
            FieldSolverRequest Request,
            IProgress<FieldSolverProgress>? Progress = null,
            CancellationToken CancellationToken = default
            )
        {
            ValidateRequest(Request);
            // HaulBuilder's connector cache is process-wide and this solver clears it
            // after every run, so a second solve would clear the first one's cache.
            if (Interlocked.CompareExchange(ref _ActiveSolveCount, BUSY, IDLE) != IDLE)
            {
                throw new InvalidOperationException("A field solve is already running in this process.");
            }
            try
            {
                using (CancellationTokenSource TimeLimitSource = new CancellationTokenSource())
                {
                    // One token for both reasons to stop; the caller's own token is
                    // passed as well so the two reasons can be told apart afterwards.
                    using (CancellationTokenSource Linked = CancellationTokenSource.CreateLinkedTokenSource(
                        CancellationToken, TimeLimitSource.Token))
                    {
                        if (Request.TimeLimit.HasValue) TimeLimitSource.CancelAfter(Request.TimeLimit.Value);
                        Session Active = new Session(_Runner, Request, Progress, CancellationToken, Linked.Token);
                        return Active.Run();
                    }
                }
            }
            finally
            {
                Interlocked.Exchange(ref _ActiveSolveCount, IDLE);
            }
        }

        /// <summary>Rejects a request the search cannot run, before any engine time is spent.</summary>
        /// <param name="Request">Request to check.</param>
        /// <exception cref="ArgumentNullException">Thrown when request or its search space is null.</exception>
        /// <exception cref="ArgumentException">Thrown when the options skip planning or guidance.</exception>
        /// <exception cref="ArgumentOutOfRangeException">Thrown when a limit or a search value is out of range.</exception>
        static private void ValidateRequest
            (
            FieldSolverRequest Request
            )
        {
            if (Request == null) throw new ArgumentNullException(nameof(Request));
            if (Request.SearchSpace == null) throw new ArgumentNullException(nameof(Request.SearchSpace));
            // Whole-job distance exists only after exact guidance.
            if (Request.BaseOptions.SkipHaulPlan || Request.BaseOptions.SkipGuidance)
            {
                throw new ArgumentException("SkipHaulPlan and SkipGuidance must be false for a solve.", nameof(Request));
            }
            if (!Double.IsFinite(Request.UnfinishedSoilFactor) || (Request.UnfinishedSoilFactor < 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(Request.UnfinishedSoilFactor));
            }
            if (Request.MaximumUnfinishedPercent.HasValue
                && (!Double.IsFinite(Request.MaximumUnfinishedPercent.Value)
                    || (Request.MaximumUnfinishedPercent.Value < 0.0)))
            {
                throw new ArgumentOutOfRangeException(nameof(Request.MaximumUnfinishedPercent));
            }
            if (Request.MaximumManualAssists.HasValue && (Request.MaximumManualAssists.Value < 0))
            {
                throw new ArgumentOutOfRangeException(nameof(Request.MaximumManualAssists));
            }
            if (Request.MaximumRuns < 0) throw new ArgumentOutOfRangeException(nameof(Request.MaximumRuns));
            if (Request.TimeLimit.HasValue && (Request.TimeLimit.Value <= TimeSpan.Zero))
            {
                throw new ArgumentOutOfRangeException(nameof(Request.TimeLimit));
            }

            // The same ranges HaulBuilder enforces, checked here so a bad search value
            // fails at once instead of an hour into the search.
            FieldSolverSearchSpace Space = Request.SearchSpace;
            foreach (Int32 Value in Space.EdgeMaxLaneSlicesPerCycleValues)
            {
                if (Value <= 0) throw new ArgumentOutOfRangeException(nameof(Space.EdgeMaxLaneSlicesPerCycleValues));
            }
            foreach (Double Value in Space.MinimumWorkLengthFtValues)
            {
                if (!Double.IsFinite(Value) || (Value < 0.0))
                {
                    throw new ArgumentOutOfRangeException(nameof(Space.MinimumWorkLengthFtValues));
                }
            }
            foreach (Double Value in Space.LaneAxisHeadingDegValues)
            {
                if (!Double.IsFinite(Value)) throw new ArgumentOutOfRangeException(nameof(Space.LaneAxisHeadingDegValues));
            }
            foreach (Double Value in Space.FlowPenaltyWeightValues)
            {
                if (!Double.IsFinite(Value) || (Value < 0.0))
                {
                    throw new ArgumentOutOfRangeException(nameof(Space.FlowPenaltyWeightValues));
                }
            }
            foreach (Int32 Value in Space.MaxLaneSlicesPerCycleValues)
            {
                if (Value <= 0) throw new ArgumentOutOfRangeException(nameof(Space.MaxLaneSlicesPerCycleValues));
            }
            foreach (Int32 Value in Space.RouteReorderWindowValues)
            {
                if (Value < 0) throw new ArgumentOutOfRangeException(nameof(Space.RouteReorderWindowValues));
            }
        }

        /// <summary>
        /// The state of one solve: the runs made so far, the best of them, and the
        /// price of unfinished soil that the baseline run sets.
        /// </summary>
        private sealed class Session : IProgress<GuidanceBuildProgress>
        {
            /// <summary>Builds and measures one plan.</summary>
            private readonly ICandidateRunner _Runner;

            /// <summary>The caller's request.</summary>
            private readonly FieldSolverRequest _Request;

            /// <summary>The caller's progress sink, or null.</summary>
            private readonly IProgress<FieldSolverProgress>? _Progress;

            /// <summary>The caller's own token, kept apart to tell a cancel from the time limit.</summary>
            private readonly CancellationToken _CallerToken;

            /// <summary>Token that fires on a caller cancel or at the time limit.</summary>
            private readonly CancellationToken _StopToken;

            /// <summary>Wall clock of the solve.</summary>
            private readonly Stopwatch _Clock;

            /// <summary>The one options object every run uses. HaulBuilder copies options on entry, so it is safe to change between runs.</summary>
            private readonly HaulBuilderOptions _Working;

            /// <summary>The caller's settings.</summary>
            private readonly FieldSolverSettings _BaselineSettings;

            /// <summary>Every run in the order it was made.</summary>
            private readonly List<FieldSolverCandidate> _Candidates;

            /// <summary>Runs by settings, so equal settings are never run twice.</summary>
            private readonly Dictionary<FieldSolverSettings, FieldSolverCandidate> _Tried;

            /// <summary>The baseline run, or null until it finishes.</summary>
            private FieldSolverCandidate? _Baseline;

            /// <summary>The best run so far, or null until the baseline finishes.</summary>
            private FieldSolverCandidate? _Best;

            /// <summary>Metres charged per BCY of unfinished soil; set by the baseline run.</summary>
            private Double _ChargeMPerBcy;

            /// <summary>Why the solve stopped.</summary>
            private FieldSolverStatus _Status;

            /// <summary>Upper bound on the runs the search wants to make.</summary>
            private Int32 _PlannedRunCount;

            /// <summary>Settings of the run in progress, for engine progress reports.</summary>
            private FieldSolverSettings _CurrentSettings;

            /// <summary>Initializes the state of one solve.</summary>
            /// <param name="Runner">Runner that builds and measures one plan.</param>
            /// <param name="Request">The caller's request, already validated.</param>
            /// <param name="Progress">The caller's progress sink, or null.</param>
            /// <param name="CallerToken">The caller's own token.</param>
            /// <param name="StopToken">Token that fires on a caller cancel or at the time limit.</param>
            public Session
                (
                ICandidateRunner Runner,
                FieldSolverRequest Request,
                IProgress<FieldSolverProgress>? Progress,
                CancellationToken CallerToken,
                CancellationToken StopToken
                )
            {
                _Runner = Runner;
                _Request = Request;
                _Progress = Progress;
                _CallerToken = CallerToken;
                _StopToken = StopToken;
                _Clock = Stopwatch.StartNew();
                // The caller's object may be bound to a user interface: work on a copy.
                _Working = OptionsCopier.Copy(Request.BaseOptions);
                DisableOutputs(_Working);
                _BaselineSettings = FieldSolverSettings.FromOptions(Request.BaseOptions);
                _Candidates = new List<FieldSolverCandidate>();
                _Tried = new Dictionary<FieldSolverSettings, FieldSolverCandidate>();
                _Status = FieldSolverStatus.Completed;
                _CurrentSettings = _BaselineSettings;
            }

            /// <summary>Runs the combinations, then the screening, and builds the result.</summary>
            /// <returns>The finished result.</returns>
            public FieldSolverResult Run
                (
                )
            {
                FieldSolverSearchSpace Space = _Request.SearchSpace;
                List<FieldSolverSettings> Combinations = SearchPlanner.BuildCombinations(_BaselineSettings, Space);
                Int32 ScreeningValues = SearchPlanner.CountScreeningValues(Space);
                // One more run may be needed to combine the screening values that helped.
                _PlannedRunCount = Combinations.Count + ScreeningValues + ((ScreeningValues > 1) ? 1 : 0);

                foreach (FieldSolverSettings Settings in Combinations)
                {
                    if (!TryRun(Settings, out FieldSolverCandidate? _)) return BuildResult();
                }

                // Screening starts from the best combination, because the screened
                // settings were measured as second-order on the reference field.
                FieldSolverCandidate Leader = _Best!;
                FieldSolverSettings Combined = Leader.Settings;
                Int32 ImprovedGroups = 0;
                foreach (List<FieldSolverSettings> Group in SearchPlanner.BuildScreeningGroups(Leader.Settings, Space))
                {
                    FieldSolverCandidate? GroupBest = null;
                    foreach (FieldSolverSettings Settings in Group)
                    {
                        if (!TryRun(Settings, out FieldSolverCandidate? Candidate)) return BuildResult();
                        if ((Candidate != null)
                            && IsUsable(Candidate)
                            && (Candidate.ScoreM < Leader.ScoreM)
                            && ((GroupBest == null) || (Candidate.ScoreM < GroupBest.ScoreM)))
                        {
                            GroupBest = Candidate;
                        }
                    }
                    if (GroupBest != null)
                    {
                        Combined = SearchPlanner.MergeChanges(Combined, Leader.Settings, GroupBest.Settings);
                        ImprovedGroups += 1;
                    }
                }
                // One helpful setting is already a finished run. Two or more have only
                // been measured apart, so they need one run together.
                if (ImprovedGroups > 1) TryRun(Combined, out FieldSolverCandidate? _);
                return BuildResult();
            }

            /// <summary>Receives the engine's guidance progress for the run in progress.</summary>
            /// <param name="Value">Engine progress snapshot.</param>
            public void Report
                (
                GuidanceBuildProgress Value
                )
            {
                ReportProgress(_Candidates.Count + 1, Value.PercentComplete, null);
            }

            /// <summary>Makes one engine run unless the settings were already run or a limit stops the solve.</summary>
            /// <param name="Settings">Settings to run.</param>
            /// <param name="Candidate">The run for these settings, new or made earlier; null when the solve stopped first.</param>
            /// <returns>True to continue the search; false when the solve must stop.</returns>
            private Boolean TryRun
                (
                FieldSolverSettings Settings,
                out FieldSolverCandidate? Candidate
                )
            {
                // The engine is deterministic, so equal settings give an equal plan.
                if (_Tried.TryGetValue(Settings, out Candidate)) return true;
                if ((_Request.MaximumRuns > 0) && (_Candidates.Count >= _Request.MaximumRuns))
                {
                    _Status = FieldSolverStatus.RunLimitReached;
                    return false;
                }
                if (_StopToken.IsCancellationRequested) return StopForToken();

                Int32 RunIndex = _Candidates.Count + 1;
                Boolean IsBaseline = RunIndex == 1;
                _CurrentSettings = Settings;
                Settings.ApplyTo(_Working);
                ReportProgress(RunIndex, 0.0, null);
                try
                {
                    CandidateMeasurement Measurement = _Runner.Run(_Working, this, _StopToken);
                    Candidate = Score(RunIndex, Settings, Measurement);
                }
                catch (OperationCanceledException)
                {
                    return StopForToken();
                }
                catch (Exception Error) when ((!IsBaseline) && (Error is not OutOfMemoryException))
                {
                    // One settings combination that the engine cannot plan must not
                    // end a search that has already cost many minutes. A baseline
                    // failure is not caught: the caller's own settings do not plan.
                    Candidate = new FieldSolverCandidate
                    {
                        RunIndex = RunIndex,
                        Settings = Settings,
                        ChangeCount = Settings.CountDifferences(_BaselineSettings),
                        Succeeded = false,
                        FailureMessage = Error.GetType().Name + ": " + Error.Message,
                    };
                }

                _Candidates.Add(Candidate);
                _Tried[Settings] = Candidate;
                if (IsBaseline) _Baseline = Candidate;
                if (IsUsable(Candidate) && ((_Best == null) || IsBetter(Candidate, _Best))) _Best = Candidate;
                ReportProgress(RunIndex, PERCENT, Candidate);
                return true;
            }

            /// <summary>Ends the solve for a cancel or the time limit.</summary>
            /// <returns>Always false, so the search stops.</returns>
            /// <exception cref="OperationCanceledException">Thrown when the caller cancelled before the baseline finished.</exception>
            /// <exception cref="TimeoutException">Thrown when the time limit ended before the baseline finished.</exception>
            private Boolean StopForToken
                (
                )
            {
                Boolean CallerCancelled = _CallerToken.IsCancellationRequested;
                if (_Baseline == null)
                {
                    // Without a baseline there is no best plan to return.
                    if (CallerCancelled) throw new OperationCanceledException(_CallerToken);
                    throw new TimeoutException("The time limit ended before the baseline run finished.");
                }
                _Status = CallerCancelled ? FieldSolverStatus.Cancelled : FieldSolverStatus.TimeLimitReached;
                return false;
            }

            /// <summary>Turns one run's measurements into a scored candidate.</summary>
            /// <param name="RunIndex">1-based index of the run.</param>
            /// <param name="Settings">Settings of the run.</param>
            /// <param name="Measurement">What the engine reported.</param>
            /// <returns>The scored candidate.</returns>
            private FieldSolverCandidate Score
                (
                Int32 RunIndex,
                FieldSolverSettings Settings,
                CandidateMeasurement Measurement
                )
            {
                Boolean IsBaseline = RunIndex == 1;
                // A field with more cut than fill has surplus no plan can place, and a
                // field with less cut than fill has a shortfall no plan can supply.
                // Neither is the plan's doing, so neither counts as unfinished.
                Double SurplusCutBcy = Math.Max(0.0, Measurement.RequiredCutBcy - Measurement.RequiredFillBcy);
                Double ShortfallFillBcy = Math.Max(0.0, Measurement.RequiredFillBcy - Measurement.RequiredCutBcy);
                // What is left on either side is work somebody still has to do by hand.
                Double UnfinishedBcy = Math.Max(
                    0.0,
                    Math.Max(
                        Measurement.DeferredCutBcy - SurplusCutBcy,
                        Measurement.DeferredFillBcy - ShortfallFillBcy));
                Double MovableBcy = Math.Min(Measurement.RequiredCutBcy, Measurement.RequiredFillBcy);
                Double UnfinishedPercent = (MovableBcy > 0.0)
                    ? (PERCENT * UnfinishedBcy) / MovableBcy
                    : 0.0;
                if (IsBaseline)
                {
                    // The baseline prices unfinished soil for the whole solve: its own
                    // driven metres per BCY placed, times the caller's factor.
                    Double PlacedBcy = MovableBcy - UnfinishedBcy;
                    _ChargeMPerBcy = (PlacedBcy > 0.0)
                        ? (_Request.UnfinishedSoilFactor * Measurement.DistanceM) / PlacedBcy
                        : 0.0;
                }
                Double ChargeM = UnfinishedBcy * _ChargeMPerBcy;
                // The baseline is the plan the caller already has, so limits never remove it.
                Boolean WithinLimits = IsBaseline
                    || (((!_Request.MaximumUnfinishedPercent.HasValue)
                            || (UnfinishedPercent <= _Request.MaximumUnfinishedPercent.Value))
                        && ((!_Request.MaximumManualAssists.HasValue)
                            || (Measurement.ManualAssistCount <= _Request.MaximumManualAssists.Value)));
                return new FieldSolverCandidate
                {
                    RunIndex = RunIndex,
                    Settings = Settings,
                    ChangeCount = Settings.CountDifferences(_BaselineSettings),
                    Succeeded = true,
                    DistanceM = Measurement.DistanceM,
                    LoadedDistanceM = Measurement.LoadedDistanceM,
                    UnloadedDistanceM = Measurement.UnloadedDistanceM,
                    DeterministicHours = Measurement.DeterministicHours,
                    RequiredCutBcy = Measurement.RequiredCutBcy,
                    RequiredFillBcy = Measurement.RequiredFillBcy,
                    DeferredCutBcy = Measurement.DeferredCutBcy,
                    DeferredFillBcy = Measurement.DeferredFillBcy,
                    UnfinishedBcy = UnfinishedBcy,
                    UnfinishedPercent = UnfinishedPercent,
                    ManualAssistCount = Measurement.ManualAssistCount,
                    UnfinishedChargeM = ChargeM,
                    ScoreM = Measurement.DistanceM + ChargeM,
                    WithinLimits = WithinLimits,
                    ElapsedSeconds = Measurement.ElapsedSeconds,
                };
            }

            /// <summary>Builds the result from the runs made so far.</summary>
            /// <returns>The finished result.</returns>
            private FieldSolverResult BuildResult
                (
                )
            {
                // TryRun throws before the baseline exists, so both are set here.
                FieldSolverCandidate Baseline = _Baseline!;
                FieldSolverCandidate Best = _Best!;
                // The caller's own output switches come back with the best settings,
                // so the caller's normal run path builds the plan it expects.
                HaulBuilderOptions BestOptions = OptionsCopier.Copy(_Request.BaseOptions);
                Best.Settings.ApplyTo(BestOptions);
                return new FieldSolverResult(
                    _Status,
                    Baseline,
                    Best,
                    _Candidates.AsReadOnly(),
                    BestOptions,
                    _PlannedRunCount,
                    _Clock.Elapsed.TotalSeconds);
            }

            /// <summary>Sends one progress snapshot to the caller.</summary>
            /// <param name="RunIndex">1-based index of the run in progress or just finished.</param>
            /// <param name="RunPercent">Progress of that run, 0 to 100.</param>
            /// <param name="Completed">The run that just finished, or null.</param>
            private void ReportProgress
                (
                Int32 RunIndex,
                Double RunPercent,
                FieldSolverCandidate? Completed
                )
            {
                if (_Progress == null) return;
                // Finished runs count whole; the run in progress counts by its own percent.
                Double Overall = (_PlannedRunCount > 0)
                    ? (PERCENT * ((RunIndex - 1) + (RunPercent / PERCENT))) / _PlannedRunCount
                    : 0.0;
                _Progress.Report(new FieldSolverProgress(
                    RunIndex,
                    _PlannedRunCount,
                    _CurrentSettings,
                    RunPercent,
                    Math.Min(PERCENT, Overall),
                    Completed,
                    _Best,
                    _Clock.Elapsed.TotalSeconds));
            }

            /// <summary>Tells whether a run may be chosen as the best plan.</summary>
            /// <param name="Candidate">Run to check.</param>
            /// <returns>True when the engine returned a plan and the plan is inside the hard limits.</returns>
            static private Boolean IsUsable
                (
                FieldSolverCandidate Candidate
                )
            {
                return Candidate.Succeeded && Candidate.WithinLimits;
            }

            /// <summary>Tells whether one run ranks ahead of another.</summary>
            /// <param name="Candidate">Run being considered.</param>
            /// <param name="Incumbent">Current best run, made earlier.</param>
            /// <returns>True when the candidate has the lower score, or an equal score with fewer changed settings.</returns>
            static private Boolean IsBetter
                (
                FieldSolverCandidate Candidate,
                FieldSolverCandidate Incumbent
                )
            {
                if (Candidate.ScoreM < Incumbent.ScoreM) return true;
                if (Candidate.ScoreM > Incumbent.ScoreM) return false;
                // Equal plans: prefer the one closer to the caller's settings; the
                // earlier run keeps the place on a full tie.
                return Candidate.ChangeCount < Incumbent.ChangeCount;
            }

            /// <summary>Switches off every optional output so a search run builds only the plan.</summary>
            /// <param name="Options">The solver's working copy of the options.</param>
            static private void DisableOutputs
                (
                HaulBuilderOptions Options
                )
            {
                // None of these changes the plan; each only adds render time and memory.
                Options.IncludeCutFillPairCsv = false;
                Options.IncludeCutFillPairPng = false;
                Options.IncludeGuidancePng = false;
                Options.IncludeAnimationGif = false;
                Options.IncludeAnimationFrames = false;
                Options.IncludePivotAnimationGif = false;
                Options.IncludePivotAnimationFrames = false;
                Options.RouteGuidanceImageCount = 0;
                Options.RandomRouteTripletCount = 0;
                Options.RandomRouteImageCount = 0;
                Options.Ponding.IncludeMap = false;
            }
        }
    }
}
