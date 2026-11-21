namespace HaulBuilder.Tests
{
    /// <summary>
    /// Serializes tests that read process-wide engine state, such as the
    /// connector cache counters, so no other engine run is active beside them.
    /// </summary>
    [CollectionDefinition(EngineStateCollection.NAME, DisableParallelization = true)]
    public sealed class EngineStateCollection
    {
        /// <summary>Collection name for <c>[Collection]</c> attributes.</summary>
        public const String NAME = "EngineState";
    }
}
