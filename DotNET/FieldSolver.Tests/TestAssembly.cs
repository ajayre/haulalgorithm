using Xunit;

// Only one solve may run in a process, and the engine's connector cache is shared
// by the whole process, so the test classes of this assembly must run one after
// another instead of side by side.
[assembly: CollectionBehavior(DisableTestParallelization = true)]
