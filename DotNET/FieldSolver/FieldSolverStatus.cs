namespace FieldSolver
{
    /// <summary>Why a solve stopped. Every status still carries the best plan found so far.</summary>
    public enum FieldSolverStatus
    {
        /// <summary>Every planned run was made.</summary>
        Completed        = 0,

        /// <summary>The caller cancelled after the baseline run finished.</summary>
        Cancelled        = 1,

        /// <summary>The time limit ended the search after the baseline run finished.</summary>
        TimeLimitReached = 2,

        /// <summary>The run limit ended the search.</summary>
        RunLimitReached  = 3,
    }
}
