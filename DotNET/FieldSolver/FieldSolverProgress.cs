using System;

namespace FieldSolver
{
    /// <summary>
    /// A progress snapshot reported while a solve runs. One is sent when a run
    /// starts, as the engine reports guidance progress, and when a run finishes.
    /// </summary>
    public sealed class FieldSolverProgress
    {
        /// <summary>Initializes one progress snapshot.</summary>
        /// <param name="RunIndex">1-based index of the run in progress or just finished.</param>
        /// <param name="PlannedRunCount">Runs the search wants to make; an upper bound.</param>
        /// <param name="Settings">Settings of that run.</param>
        /// <param name="RunPercentComplete">Engine guidance progress of that run, 0 to 100.</param>
        /// <param name="OverallPercentComplete">Progress of the whole solve, 0 to 100.</param>
        /// <param name="Completed">The run that just finished, or null while a run is in progress.</param>
        /// <param name="BestSoFar">The best finished run so far, or null before the baseline finishes.</param>
        /// <param name="ElapsedSeconds">Wall-clock seconds since the solve started.</param>
        public FieldSolverProgress
            (
            Int32 RunIndex,
            Int32 PlannedRunCount,
            FieldSolverSettings Settings,
            Double RunPercentComplete,
            Double OverallPercentComplete,
            FieldSolverCandidate? Completed,
            FieldSolverCandidate? BestSoFar,
            Double ElapsedSeconds
            )
        {
            this.RunIndex = RunIndex;
            this.PlannedRunCount = PlannedRunCount;
            this.Settings = Settings;
            this.RunPercentComplete = RunPercentComplete;
            this.OverallPercentComplete = OverallPercentComplete;
            this.Completed = Completed;
            this.BestSoFar = BestSoFar;
            this.ElapsedSeconds = ElapsedSeconds;
        }

        /// <summary>Gets the 1-based index of the run in progress or just finished.</summary>
        public Int32 RunIndex { get; }

        /// <summary>Gets the number of runs the search wants to make. It is an upper bound.</summary>
        public Int32 PlannedRunCount { get; }

        /// <summary>Gets the settings of the run in progress or just finished.</summary>
        public FieldSolverSettings Settings { get; }

        /// <summary>Gets the engine's guidance progress for the run, 0 to 100.</summary>
        public Double RunPercentComplete { get; }

        /// <summary>Gets the progress of the whole solve, 0 to 100.</summary>
        public Double OverallPercentComplete { get; }

        /// <summary>Gets the run that just finished, or null while a run is in progress.</summary>
        public FieldSolverCandidate? Completed { get; }

        /// <summary>Gets the best finished run so far, or null before the baseline finishes.</summary>
        public FieldSolverCandidate? BestSoFar { get; }

        /// <summary>Gets wall-clock seconds since the solve started.</summary>
        public Double ElapsedSeconds { get; }
    }
}
