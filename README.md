# Haul Algorithm

Plans land leveling for one tractor and scraper. An AGD survey supplies existing ground and the proposed grade. The planner returns an ordered set of cut–haul–fill trips, a forward-only centerline the machine can drive, and a time estimate.

The production library is [`DotNET/HaulBuilder`](DotNET/HaulBuilder). Call `HaulBuilderEngine.Run`. Results stay in memory. The caller decides whether to write them out.

[`DotNET/README.md`](DotNET/README.md) lists every option and result field.

## The job

Each grid cell is cut where existing ground is above the design and filled where it is below. One trip loads the bowl, hauls, and empties it. A later grade pass runs back over each fill. The search prefers trips that move a yard of soil in fewer seconds. Volume is kept in bank cubic yards (BCY). Swell applies only when the loose bowl capacity is converted to a bank load.

| Quantity | Default |
|---|---|
| Cell size | 8.5 ft |
| Finish tolerance | 10 mm |
| Bowl | 20 LCY loose, swell 1.3, about 15.4 BCY bank |
| Fill compaction | 1.2 bank cut per bank fill |
| Deepest cut or fill in one visit | 61 mm |
| Blade width | 2590.8 mm |
| Turning diameter | 45 ft |
| Tractor and scraper length | 43 ft |
| Cut / haul / fill / return | 4 / 10 / 5 / 13 mph |
| Field efficiency | 0.8, applied to scheduled hours |

## What one run does

`HaulBuilderEngine.Run` always takes the same path:

1. **Read the AGD file** and project the points to UTM. Rows that are missing, zero, or too short are skipped and reported.
2. **Build the grid.** Each valid cell gets a cut supply and a fill demand. When road or well-pad centrelines are supplied, the extra fill they need is reserved before anything else is planned.
3. **Drop exclusion polygons** (wells, structures) from the field. Blade-up travel later routes around them with clearance for the machine body. Travel that cannot clear an obstruction is marked as a manual move.
4. **Check mass balance.** A cut/fill gap above 2% is a warning. With exclusions active, imbalance is recorded and the warning threshold is not applied.
5. **Solve a Poisson flow.** Cut is a source and fill is a sink. The gradient is the haul arrow at each cell: soil moves from ridges toward basins.
6. **Build the trips** with the search below. The ledger checks that assigned depths stay inside the required cut and fill, inside the bowl, and inside the depth cap.
7. **Order the trips.** Full one-strip interior loads go first, then other interior loads, then the field edge, then short scraps. Inside each band the order scores the empty hop plus the turn arc.
8. **Optional extras**, when the caller turns them on: ponding-aware reordering, a center-pivot arm schedule, zone summaries, and cut/fill pair maps.
9. **Build guidance.** Coverage lanes, Dubins connectors, a grade return, and cleanup trips for scraps the main track could not drive.
10. **Time the centerline** and, if asked, render maps, CSVs, and a leveling animation. All of those come back as bytes or text on the result.

`SkipHaulPlan` stops after the flow and the cut/fill map. `SkipGuidance` stops after the trip list and uses the speed model for the hours.

## How a trip is chosen

Production defaults compare three ways of taking the next load and keep the one with the lowest seconds per bank cubic yard. A tie goes to the flow ribbon, then cluster hauling, then thin-cut. After 500 flow trips, the median of those costs is fixed. A later trip more expensive than three times that median is left unscheduled. When nothing under that ceiling remains, correcting starts.

### Flow ribbon

The ribbon starts where remaining cut meets remaining fill and walks backward along the smoothed Poisson arrows.

- The cut follows the arrow uphill toward the ridge. The fill follows it on into the basin.
- The blade is one cell wide. A cell counts only when the blade covers at least a quarter of it, so a diagonal nick is not taken.
- A step has to stay with the arrow (about 32°) and with the step just taken (about 45°). Wider than that, the strip stops being a strip.
- Cells already at grade are driven across. The dirt stays in the bowl until the next cell that is still low.
- One load may add another strip within 12 cells when that fills the bowl. The ribbon stays within about three cells of width.
- The search tries a full strip first, then a short strip, then a fold that turns back on itself. A fold waits until no straight strip remains, so the last dirt on a line is not abandoned.
- If the arrow itself cannot fill a bowl, the blade may leave the arrow, and may cross a small leftover ridge to finish the bowl in the basin the arrow reaches. The haul is only the gap to the first low cell.

### Cluster hauling

Residual cut and fill are grouped into clusters, paired, and driven. A cluster grows from deep cells (150 mm) through cells above 50 mm. A piece larger than two bowls is split along its long axis. The field is clustered again after ten trips. Cells are cut and filled to the target.

### Thin-cut

What the clusters leave is tiled. A cut tile is soil above 10 mm. A fill tile is a hole below −30 mm, taken all the way to the target. Pairing and the ten-trip replan are the same as cluster hauling.

### Correcting

Correcting is the last stage. Passes run the length of the field along the mean flow, a blade width apart, as a serpentine, then a headland lap along the boundary. The blade is at the design. High cells load the bowl and low cells take a spread of at most 100 mm. Another coverage runs only when the last one cut the haul error by at least one percent and the surface is still outside the planing gate.

### Roads and pads

When centrelines are supplied, those trips are inserted after the haul search and before correcting. Every lane on a road faces the same way. A lane change is a forward S-bend. Unused reserved fill is closed out after correcting.

A cluster window of zero keeps the flow ribbon for the whole field. The cost comparison, cluster hauling, thin-cut, and correcting stay off.

## Driving it

`TripGuidance` joins the blade-down lanes with forward Dubins paths (LSL, RSR, LSR, RSL, RLR, LRL). The machine does not reverse except for the grade pass, which runs back over the fill. Correcting does not grade: the land plane follows the coverage. Lane direction is chosen by time. A road trip keeps one facing.

A connector within 2% of the shortest, or half a turning radius when that is wider, prefers the path with the least length outside the field. A short joint may be a straight steer instead of a loop. Two connectors with no lane between them become one path. After every trip is fixed, trips inside one stage are reordered to shorten the empty returns. Two trips that share a cell keep their relative order.

A blade-down run shorter than half the machine length (21.5 ft at the default 43 ft) is left off the main track. Those scraps are rebuilt into cleanup trips at the end of the job, with the length gate turned off. What still will not pair is reported as manual cleanup, separate from cut that has no matching fill.

An optional center pivot is a parked arm. Bearings are scheduled across the trip order, and haul legs that would cross the arm are routed around it.

## What comes back

`HaulBuilderResult` holds the grid, the flow, the trips, the drive order, the guidance waypoints, and the hours.

- `Metrics.DeterministicHours` is the centerline at the configured speeds. `ScheduledHours` divides that by field efficiency.
- `DieselEstimate` is the diesel gallons and cost when `options.Diesel` has a full-load fuel rate, drawbar horsepower, and a price per gallon. Otherwise it is null.
- `DeferredWorkSummary` separates material surplus from balanced work left for hand cleanup.
- `State` and `ClusterModeStartsAtRoute`, `ThinCutStartsAtRoute`, and `CorrectingStartsAtRoute` record where the search changed stage. Those indexes are in the order trips were found.
- `PngBytesByName`, `CsvTextByName`, and the animation byte arrays are empty unless the matching option is set. There is no output-path argument.

## Build and run

.NET 8. The HaulBuilder project builds the native KD-tree (`haul_kdtree`) before it compiles. On Windows that is `DotNET/Native/build_native.ps1`.

```powershell
dotnet build DotNET/HaulBuilder/HaulBuilder.csproj -c Release
dotnet test DotNET/HaulBuilder.Tests/HaulBuilder.Tests.csproj -c Release
```

```csharp
using HaulBuilder;

var options = new HaulBuilderOptions
{
    AgdPath = @"C:\fields\example.agd",
};

HaulBuilderResult result = new HaulBuilderEngine().Run(options);

Console.WriteLine($"Routes: {result.Route?.OrderedPasses.Count ?? 0}");
Console.WriteLine($"Deterministic hours: {result.Metrics?.DeterministicHours:F3}");
Console.WriteLine($"Scheduled hours: {result.Metrics?.ScheduledHours:F3}");
Console.WriteLine($"Manual cleanup: {result.DeferredWorkSummary?.ManualCleanupBcy:F3} BCY");
```

`Run` accepts an `IProgress<GuidanceBuildProgress>` and a `CancellationToken`. Progress callbacks arrive on worker threads. Cancellation during guidance throws `OperationCanceledException`.

A sample field is `DotNET/TheShop2_2ft.agd`. `DotNET/Bench` and `DotNET/HaulRunProof` are measurement harnesses. They are not part of the library API.

## Layout

```
DotNET/HaulBuilder/           production library
DotNET/HaulBuilder.Tests/     tests
DotNET/HaulBuilder.Visual/    sheet and map rendering
DotNET/Bench/                 timing harness
DotNET/HaulRunProof/          field-run proofs
DotNET/PondingProof/          ponding comparison maps
DotNET/Native/                KD-tree used by the planner
Python/                       earlier pipeline for the same problem
```
