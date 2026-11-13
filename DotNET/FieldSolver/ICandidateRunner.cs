using System;
using System.Threading;

using HaulBuilder;

namespace FieldSolver
{
    /// <summary>
    /// Runs one plan and measures it. The search talks to the engine only through
    /// this interface, so tests can drive the search with a table of results
    /// instead of minutes-long engine runs.
    /// </summary>
    internal interface ICandidateRunner
    {
        /// <summary>Builds one plan and returns its measurements.</summary>
        /// <param name="Options">Complete options for the run.</param>
        /// <param name="Progress">Optional sink for the engine's guidance progress.</param>
        /// <param name="CancellationToken">Token that stops the run.</param>
        /// <returns>The measurements of the finished plan.</returns>
        CandidateMeasurement Run
            (
            HaulBuilderOptions Options,
            IProgress<GuidanceBuildProgress>? Progress,
            CancellationToken CancellationToken
            );
    }
}
