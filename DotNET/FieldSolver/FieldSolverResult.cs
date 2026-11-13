using System;
using System.Collections.Generic;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// The outcome of one solve: the best settings found, the options that carry
    /// them, and every run that was made so a caller can show the evidence.
    /// </summary>
    public sealed class FieldSolverResult
    {
        /// <summary>Percent scale for the saving.</summary>
        private const Double PERCENT = 100.0;

        /// <summary>Initializes a finished result.</summary>
        /// <param name="Status">Why the solve stopped.</param>
        /// <param name="Baseline">The run made with the caller's own settings.</param>
        /// <param name="Best">The lowest-scoring successful run inside the limits.</param>
        /// <param name="Candidates">Every run in the order it was made.</param>
        /// <param name="BestOptions">A copy of the caller's options carrying the best settings.</param>
        /// <param name="PlannedRunCount">Runs the search wanted to make.</param>
        /// <param name="ElapsedSeconds">Wall-clock seconds for the whole solve.</param>
        public FieldSolverResult
            (
            FieldSolverStatus Status,
            FieldSolverCandidate Baseline,
            FieldSolverCandidate Best,
            IReadOnlyList<FieldSolverCandidate> Candidates,
            HaulBuilderOptions BestOptions,
            Int32 PlannedRunCount,
            Double ElapsedSeconds
            )
        {
            this.Status = Status;
            this.Baseline = Baseline;
            this.Best = Best;
            this.Candidates = Candidates;
            this.BestOptions = BestOptions;
            this.PlannedRunCount = PlannedRunCount;
            this.ElapsedSeconds = ElapsedSeconds;
        }

        /// <summary>Gets why the solve stopped.</summary>
        public FieldSolverStatus Status { get; }

        /// <summary>Gets the run made with the caller's own settings.</summary>
        public FieldSolverCandidate Baseline { get; }

        /// <summary>Gets the best run. It is the baseline when nothing beat it.</summary>
        public FieldSolverCandidate Best { get; }

        /// <summary>Gets every run in the order it was made, failed runs included.</summary>
        public IReadOnlyList<FieldSolverCandidate> Candidates { get; }

        /// <summary>
        /// Gets a copy of the caller's options with the best settings applied and the
        /// caller's own output switches. Pass it to <c>HaulBuilderEngine.Run</c> to
        /// build the plan.
        /// </summary>
        public HaulBuilderOptions BestOptions { get; }

        /// <summary>Gets the number of runs the search wanted to make.</summary>
        public Int32 PlannedRunCount { get; }

        /// <summary>Gets wall-clock seconds for the whole solve.</summary>
        public Double ElapsedSeconds { get; }

        /// <summary>Gets driven metres saved against the baseline. Zero when the baseline is best.</summary>
        public Double SavingM => Baseline.DistanceM - Best.DistanceM;

        /// <summary>Gets the saving as a percentage of the baseline's driven metres.</summary>
        public Double SavingPercent => Baseline.DistanceM > 0.0
            ? (PERCENT * SavingM) / Baseline.DistanceM
            : 0.0;
    }
}
