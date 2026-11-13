using System;
using System.Collections.Generic;
using System.Threading;

using HaulBuilder;

namespace FieldSolver.Tests
{
    /// <summary>
    /// A candidate runner that answers from a table of results instead of running
    /// the engine, so the search logic can be tested in milliseconds.
    /// </summary>
    internal sealed class TableRunner : ICandidateRunner
    {
        /// <summary>Longest time a run waits for the stop token, so a broken solver fails a test instead of hanging it.</summary>
        private const Int32 WAIT_LIMIT_MS = 20000;

        /// <summary>Results by settings.</summary>
        private readonly Dictionary<FieldSolverSettings, CandidateMeasurement> _Rows;

        /// <summary>Initializes an empty table.</summary>
        public TableRunner
            (
            )
        {
            _Rows = new Dictionary<FieldSolverSettings, CandidateMeasurement>();
            Calls = new List<FieldSolverSettings>();
            SeenOutputSwitches = new List<Boolean>();
        }

        /// <summary>Gets the settings of every run requested, in order.</summary>
        public List<FieldSolverSettings> Calls { get; }

        /// <summary>Gets, per run, whether any optional output was still switched on.</summary>
        public List<Boolean> SeenOutputSwitches { get; }

        /// <summary>Gets or sets an action invoked at the start of every run with its 1-based index.</summary>
        public Action<Int32>? BeforeRun { get; set; }

        /// <summary>
        /// Gets or sets the 1-based run that blocks until the stop token fires, which
        /// stands in for an engine run that outlasts the time limit. Zero means none.
        /// </summary>
        public Int32 WaitForStopAtRun { get; set; }

        /// <summary>Adds one row, replacing a row with equal settings.</summary>
        /// <param name="Settings">Settings the row belongs to.</param>
        /// <param name="Measurement">What the engine reported, or is pretended to report.</param>
        public void Add
            (
            FieldSolverSettings Settings,
            CandidateMeasurement Measurement
            )
        {
            _Rows[Settings] = Measurement;
        }

        /// <summary>Returns the table row for the options' settings.</summary>
        /// <param name="Options">Complete options for the run.</param>
        /// <param name="Progress">Unused; the table has no progress.</param>
        /// <param name="CancellationToken">Token that stops the run.</param>
        /// <returns>The row for the settings.</returns>
        /// <exception cref="InvalidOperationException">Thrown when the table has no row for the settings.</exception>
        /// <exception cref="OperationCanceledException">Thrown when the token has fired.</exception>
        public CandidateMeasurement Run
            (
            HaulBuilderOptions Options,
            IProgress<GuidanceBuildProgress>? Progress,
            CancellationToken CancellationToken
            )
        {
            FieldSolverSettings Settings = FieldSolverSettings.FromOptions(Options);
            Calls.Add(Settings);
            // The solver must hand the engine options with every optional output off.
            SeenOutputSwitches.Add(
                Options.IncludeCutFillPairCsv
                || Options.IncludeCutFillPairPng
                || Options.IncludeGuidancePng
                || Options.IncludeAnimationGif
                || Options.IncludeAnimationFrames
                || Options.IncludePivotAnimationGif
                || Options.IncludePivotAnimationFrames
                || (Options.RouteGuidanceImageCount != 0)
                || (Options.RandomRouteTripletCount != 0)
                || (Options.RandomRouteImageCount != 0)
                || Options.Ponding.IncludeMap);
            BeforeRun?.Invoke(Calls.Count);
            if (Calls.Count == WaitForStopAtRun)
            {
                // Like the engine, the run ends only when it notices the token.
                CancellationToken.WaitHandle.WaitOne(WAIT_LIMIT_MS);
            }
            CancellationToken.ThrowIfCancellationRequested();
            if (!_Rows.TryGetValue(Settings, out CandidateMeasurement? Row))
            {
                throw new InvalidOperationException("No table row for " + Settings.ToString());
            }
            return Row;
        }
    }
}
