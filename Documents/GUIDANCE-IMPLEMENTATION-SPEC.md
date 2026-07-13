# Guidance Remediation Implementation Specification

## 1. Authority and use

This document is the deterministic implementation companion to `GUIDANCE-REMEDIATION-PLAN.md`. It removes design choices from the implementation agent.

Implement work units in numerical order. Do not start a work unit until its listed tests pass. Do not combine work units into one large change.

This specification deliberately uses a centerline plus conservative clearance envelope. It does not model tractor-scraper articulation. Small corrections may be delegated to the in-cab operator only through an explicit, blade-up `MANUAL_ASSIST` phase.

## 2. Target production call graph

The completed C# engine must execute this order:

```text
HaulBuilderEngine.Run
  Agd.Parse
  GridMath.BuildGrid
  ResolveExclusions
  Balance.Compute
  FlowFieldSolver.Compute
  Strips.TraceStrips
  Passes.GeneratePasses
  Routing.SequencePasses
  Routing.RefineRouteOrder
  OperationalField.Build
  CoveragePlanner.BuildPlansForRoute
  GuidancePlanner.BuildRoute
    build pass 1 lifecycle
    connect pass 1 to pass 2
    ...
    build final pass lifecycle
    terminate at END
  GuidanceValidation.ValidateFinal
  GuidanceTiming.Estimate
  BuildReportsAndMetrics
  Render/export (optional)
```

Important ordering rules:

1. Route refinement happens before final guidance construction.
2. Coverage planning happens once for the final route order.
3. Connector construction happens once.
4. Final validation happens after all geometry is complete.
5. Nothing may smooth, relocate, or repair coordinates after final validation.
6. Timing consumes the exact validated/exported coordinates.
7. Lightweight guidance is always built for timing. PNG and CSV creation remain optional.

## 3. Exact source-file changes

### 3.1 Add these files

```text
DotNET/HaulBuilder/GuidanceModels.cs
DotNET/HaulBuilder/OperationalField.cs
DotNET/HaulBuilder/CoveragePlanner.cs
DotNET/HaulBuilder/GuidanceTransitions.cs
DotNET/HaulBuilder/GuidancePlanner.cs
DotNET/HaulBuilder/GuidanceValidation.cs
DotNET/HaulBuilder/GuidanceTiming.cs
```

Add tests:

```text
DotNET/HaulBuilder.Tests/OperationalFieldTests.cs
DotNET/HaulBuilder.Tests/CoveragePlannerTests.cs
DotNET/HaulBuilder.Tests/GuidanceLifecycleTests.cs
DotNET/HaulBuilder.Tests/GuidanceValidationTests.cs
DotNET/HaulBuilder.Tests/GuidanceTimingTests.cs
DotNET/HaulBuilder.Tests/TheShop2GuidanceAcceptanceTests.cs
```

### 3.2 Modify these files

```text
DotNET/HaulBuilder/HaulBuilderEngine.cs
DotNET/HaulBuilder/HaulBuilderOptions.cs
DotNET/HaulBuilder/HaulBuilderResult.cs
DotNET/HaulBuilder/Swaths.cs
DotNET/HaulBuilder/Routing.cs
DotNET/Bench/Program.cs
DotNET/Bench/ExportFirstRoutes.cs
```

Do not modify `Passes.cs` until work unit 8. The first seven work units must improve guidance without changing material allocation.

## 4. Legacy method replacement map

The implementation agent must follow this table.

| Existing symbol | Final disposition | Instruction |
|---|---|---|
| `Swaths.BuildGuidance` | Compatibility wrapper | Keep public signature temporarily. Delegate to `GuidancePlanner.BuildRoute`. Do not retain old orchestration body. |
| `Swaths.EstimateGuidanceTime` | Compatibility wrapper | Delegate to `GuidanceTiming.Estimate`. |
| `Swaths.ReprofileKnots` | Retire from engine call graph | Keep during migration for old tests, but `HaulBuilderEngine.Run` must stop calling it after `CoveragePlanner` is active. |
| `Swaths.SmoothLane` | Retire from production call graph | Do not Gaussian-smooth final guidance. It may remain private until cleanup. |
| `Swaths.EnforceMinTurnRadius` | Retire as whole-route repair | Reuse mathematical helpers only. Connectors are constructed radius-valid rather than repaired afterward. |
| `Swaths.EnforceEquipmentLineup` | Retire as post-process | `CoveragePlanner` and `GuidancePlanner` create lineups explicitly. |
| `Swaths.EnforceBladeUpTransitTurns` | Retire as post-process | `GuidanceTransitions.Connect` owns blade-up connectors. |
| `Swaths.RepairSharpSteeringBends` | Retire as post-process | A failed connector is rejected; it is not repeatedly repaired. |
| `Swaths.LanesFlow` | Reuse logic through extracted helper | Move or expose the run extraction needed by `CoveragePlanner`; add heading-reversal splitting. |
| `Swaths.RepairIfTangled` | Replace | Candidate scoring in `CoveragePlanner` replaces the conservative hairpin-only decision. |
| `Swaths.PatchDrivableLanes` | Reuse after extraction | Change heading resolution to 5 degrees and use full candidate cost. |
| Dubins/CSC/CCC/keyhole helpers in `Swaths.cs` | Reuse | Move to `GuidanceTransitions.cs` or expose as `internal`. Do not duplicate mathematics. |
| `Swaths.TurnMetrics` | Diagnostic only | Timing must use final path geometry and phase speed limits, not extra turn-angle time. |
| `TimeModel.EstimateTime` | Secondary comparison | Preserve allocation estimate, but final headline metrics use `GuidanceTiming`. |

During migration, do not call both the old and new orchestration paths. Add an internal test-only comparison if required, but production selects exactly one.

## 5. Required data contracts

Add the following types to `GuidanceModels.cs`. Follow repository naming style, but preserve these fields and meanings.

```csharp
internal enum GuidancePhase
{
    Start,
    TransitToCut,
    LineupCut,
    Cut,
    ExitCut,
    Haul,
    LineupFill,
    Fill,
    ExitFill,
    TurnToGrade,
    LineupGrade,
    Grade,
    ExitGrade,
    TransitToNextCut,
    ManualAssist,
    End,
}

internal enum BladeCommand
{
    Up,
    Cut,
    Fill,
    Grade,
}

internal readonly record struct Pose(
    Double X,
    Double Y,
    Double HeadingDeg);

internal sealed record TrajectoryPoint(
    Double X,
    Double Y,
    Int32 PassNo,
    GuidancePhase Phase,
    BladeCommand Blade,
    Double DepthM,
    Boolean Automatic,
    Double CoverageContributionM);

internal sealed class CoverageRun
{
    public required Int32 PassNo { get; init; }
    public required GuidancePhase Phase { get; init; }
    public required List<TrajectoryPoint> CoveragePoints { get; init; }
    public required Pose LineupStartPose { get; init; }
    public required Pose EngagePose { get; init; }
    public required Pose DisengagePose { get; init; }
    public required Pose ExitPose { get; init; }
    public required Double LineupLengthM { get; init; }
    public required Double ExitLengthM { get; init; }
}

internal sealed class PassCoveragePlan
{
    public required Int32 PassNo { get; init; }
    public required Int32 OriginalPassIndex { get; init; }
    public required List<CoverageRun> CutRuns { get; init; }
    public required List<CoverageRun> FillRuns { get; init; }
    public required Double EstimatedAutomaticTimeS { get; init; }
    public required Int32 ReversalCount { get; init; }
}

internal sealed class ConnectorResult
{
    public required Boolean Success { get; init; }
    public required Boolean Automatic { get; init; }
    public required String FailureReason { get; init; }
    public required List<TrajectoryPoint> Points { get; init; }
    public required Double LengthM { get; init; }
    public required Double EstimatedTimeS { get; init; }
    public required Int32 KeyholeCount { get; init; }
}
```

Extend the public `GuidancePoint` rather than replacing it. Add:

```csharp
public String Phase { get; }
public Boolean Automatic { get; }
public Double DrivenSegDistM { get; }
public Double CoverageSegDistM { get; }
public Double DrivenCumDistM { get; }
public Double CoverageCumDistM { get; }
```

Compatibility rule:

```text
SegDistM = DrivenSegDistM
CumDistM = DrivenCumDistM
```

Never calculate driven distance from raw coordinates while exporting smoothed coordinates.

Add validation contracts:

```csharp
public enum GuidanceIssueCode
{
    OutsideRawField,
    OutsideOperationalField,
    DiscontinuousPath,
    RadiusViolation,
    MissingLineup,
    LineupHeadingViolation,
    BladeStateViolation,
    ManualAssistTooLong,
    ManualAssistBladeDown,
    CoverageMismatch,
    VolumeMismatch,
}

public sealed record GuidanceIssue(
    GuidanceIssueCode Code,
    Int32 PassNo,
    Int32 PointIndex,
    String Message);

public sealed class GuidanceValidationResult
{
    public required Boolean IsValid { get; init; }
    public required List<GuidanceIssue> Issues { get; init; }
    public required Int32 AutomaticPointCount { get; init; }
    public required Int32 ManualAssistPointCount { get; init; }
    public required Double MinimumObservedRadiusM { get; init; }
    public required Double ManualAssistDistanceM { get; init; }
}
```

Add `GuidanceValidationResult? GuidanceValidation` to `HaulBuilderResult`.

## 6. Exact option additions and defaults

Add these options in `HaulBuilderOptions` and copy them in `CloneOptions`:

```csharp
BoundaryMarginFt = 2.0;
AllowManualAssist = true;
MaxManualAssistFt = 45.0;
ManualAssistSpeedMph = 2.0;
ManualAssistTakeoverS = 10.0;
LineupHeadingToleranceDeg = 5.0;
RadiusTolerancePct = 1.0;
PathSampleSpacingM = 0.5;
MaxAutomaticKeyholesPerConnector = 1;
```

Validation:

- distances and speeds must be non-negative;
- `RadiusTolerancePct` must be in `[0, 10]`;
- `LineupHeadingToleranceDeg` must be in `[0, 30]`;
- `PathSampleSpacingM` must be in `(0, gridSizeM]` after grid construction.

Do not add an option that permits leaving the raw field.

## 7. Operational field algorithm

Implement this exact raster erosion in `OperationalField.Build`.

Inputs:

```text
Grid.Valid
grid size in metres
clearance = equipmentWidthM / 2 + boundaryMarginM
```

Algorithm:

```text
offsets = empty list
maxOffset = ceil(clearance / gridSize) + 1

for dy from -maxOffset to +maxOffset:
  for dx from -maxOffset to +maxOffset:
    nearestX = max(abs(dx) - 0.5, 0) * gridSize
    nearestY = max(abs(dy) - 0.5, 0) * gridSize
    nearestDistance = hypot(nearestX, nearestY)
    if nearestDistance <= clearance:
      add (dx, dy) to offsets

for each grid cell c:
  operational[c] = rawValid[c]
  if operational[c]:
    for each offset:
      neighbor = c + offset
      if neighbor is outside grid or rawValid[neighbor] is false:
        operational[c] = false
        break
```

This treats each neighboring raster square as an occupied/outside area and conservatively removes any center cell whose clearance disk touches it.

Path lookup must not only check waypoint cells. Implement:

```text
SampleSegment(a, b, spacing):
  n = max(1, ceil(distance(a,b) / spacing))
  yield lerp(a,b,k/n) for k=0..n
```

Every automatic segment sample must be in `Operational`; every manual-assist sample must be in `Raw`.

Do not use the old `0.4 * gridSize` clearance ring as the final safety test.

## 8. Coverage planning algorithm

Implement `CoveragePlanner.BuildPlansForRoute` without changing cell assignments or depths.

### 8.1 Split the existing flow order

Start a new logical run before cell `i` when any condition is true:

```text
cell i is not 8-adjacent to cell i-1
heading change at i-1 exceeds 100 degrees
distance from i-1 to i exceeds 1.5 * grid diagonal
the source multi-drop tube changed (add a tube/run marker during extraction)
```

If adding a tube marker to `Pass` would break public compatibility, reconstruct run breaks at every non-adjacent jump and every heading reversal first; add the marker in work unit 8.

Blade-down paths may not connect these runs directly. They require blade-up connectors.

### 8.2 Candidate headings

For every connected pocket, build a deterministic candidate set:

```text
0, 5, 10, ..., 175 degrees
principal-axis heading normalized to [0,180)
mean flow heading normalized to [0,180)
arrival heading normalized to [0,180)
departure heading normalized to [0,180)
```

Round special headings to 0.1 degree and remove duplicates within 0.1 degree.

For each heading:

1. Partition cells into blade-width lanes.
2. Sort cells along each lane.
3. Try both first-lane directions.
4. Alternate subsequent lane directions.
5. Construct required entry and exit lineups.
6. Attempt a validated connector between each lane.
7. Reject the candidate if an automatic blade-down run violates the operational field or radius.

### 8.3 Candidate cost

Use this lexicographic score, not one blended scalar:

```text
(
  infeasibleAutomaticCount,
  manualAssistCount,
  keyholeCount,
  bladeDownReversalCount,
  estimatedTimeMillisecondsRounded,
  drivenLengthMillimetresRounded,
  headingTenthsOfDegree
)
```

Choose the lexicographically smallest tuple. This makes safety dominate time and keeps ties deterministic.

### 8.4 Lineup creation

For a blade-down run from `P0` to `P1`:

```text
heading = bearing(P0,P1)
engagePose = (P0.x, P0.y, heading)
lineupStart = P0 - unit(heading) * equipmentLengthM
disengagePose = (last.x, last.y, final heading)
exitPose = last + unit(final heading) * equipmentLengthM
```

Both straight segments must sample inside the operational field. If not:

1. reverse the run and retry;
2. try the next candidate heading;
3. split the pocket along the failing lane boundary and retry;
4. if all candidates fail, create a manual-assist alignment request, blade-up only.

Never clip the lineup and report it as full length.

### 8.5 Cell and depth invariant

After choosing all runs:

```text
multiset(original (bx,by,depth))
  == multiset(planned (bx,by,depth))
```

Use a tolerance of `1e-9 m` for depth comparison. Fail planning if the invariant fails.

## 9. Connector algorithm

Move/expose the existing curve primitives in `GuidanceTransitions.cs`; do not reimplement Dubins formulas.

Implement:

```csharp
internal static ConnectorResult Connect(
    Pose start,
    Pose goal,
    Int32 passNo,
    GuidancePhase phase,
    OperationalField field,
    GuidanceSettings settings);
```

Candidate order is fixed:

1. straight pose-compatible connector;
2. same-side CSC candidates;
3. remaining CSC candidates;
4. one CCC/keyhole candidate if heading difference is at least 120 degrees;
5. manual-assist path if allowed.

For every automatic candidate:

1. Sample at `PathSampleSpacingM` or finer.
2. Reject if any sample is outside the operational field.
3. Reject if final radius validation fails.
4. Reject if it contains more than `MaxAutomaticKeyholesPerConnector`.
5. Reject if its length exceeds:
   - `straightDistance + pi*R` for CSC; or
   - `straightDistance + 3*pi*R` for CCC/keyhole.
6. Score by `(keyholeCount, estimatedTimeMs, lengthMm, candidateFamilyOrder)`.
7. Select the lowest score.

Manual assist fallback:

1. Use a straight segment if it stays inside the raw field.
2. Otherwise use the existing obstacle/visibility shortest-path logic over the raw field boundary representation.
3. Sample and require every point inside the raw field.
4. Require length `<= MaxManualAssistFt`.
5. Set `Automatic=false`, `Phase=ManualAssist`, and `Blade=Up`.
6. If this also fails, return `Success=false`. Do not emit a chord or out-of-field fallback.

## 10. Exact pass lifecycle algorithm

Implement the following in `GuidancePlanner.BuildRoute`. Build the whole route in one call.

```text
output = empty
currentPose = null

precompute chosen coverage plan for every ordered pass

for passIndex = 0 .. passCount-1:
  plan = plans[passIndex]

  if passIndex == 0:
    append START at first cut lineup start
    currentPose = first cut lineup start
  else:
    assert currentPose equals first cut lineup start within 0.05 m
    // The preceding pass already emitted TransitToNextCut.

  for each cut run in plan.CutRuns:
    if currentPose does not equal run.LineupStartPose within 0.05 m:
      append Connect(currentPose, run.LineupStartPose, TransitToCut)
    append straight LINEUP_CUT
    append CUT coverage
    append straight EXIT_CUT
    currentPose = run.ExitPose

  append Connect(currentPose, first fill lineup start, Haul)

  for each fill run in plan.FillRuns:
    if not first fill run:
      append Connect(currentPose, run.LineupStartPose, Haul)
    append straight LINEUP_FILL
    append FILL coverage
    append straight EXIT_FILL
    currentPose = run.ExitPose

  if gradeReturn is enabled:
    for each fill run in reverse plan.FillRuns order:
      gradeCoverage = reverse(fillRun.CoveragePoints)
      gradeEntryHeading = bearing(first two gradeCoverage points)
      gradeLineupStart = first grade point - equipmentLength * grade heading
      append Connect(currentPose, gradeLineupStart, TurnToGrade)
      append straight LINEUP_GRADE
      append GRADE coverage with DepthM=0
      append straight EXIT_GRADE
      currentPose = grade exit pose

  if passIndex + 1 < passCount:
    nextStart = plans[passIndex+1].CutRuns[0].LineupStartPose
    append Connect(currentPose, nextStart, TransitToNextCut)
    currentPose = nextStart
  else:
    append END at currentPose
```

Rules:

- `LINEUP_CUT`, `LINEUP_FILL`, and `LINEUP_GRADE` are blade-up until their terminal engagement point.
- `CUT`, `FILL`, and `GRADE` are blade-down.
- `EXIT_*`, transit, haul, turn-to-grade, and manual assist are blade-up.
- The connector to the next cut is generated before the next loop iteration. The next iteration only asserts that it starts at the already-reached pose; it must not generate the connector a second time.
- There is no instantaneous reversal from FILL to GRADE.
- GRADE follows completed fill geometry in reverse order and uses an actual turnaround connector.
- Disconnected fill runs use blade-up connectors while grading.
- If any required connector returns `Success=false`, fail guidance construction with a structured message naming pass and phase.

## 11. Final coordinate construction

Use `TrajectoryPoint.X/Y` as the canonical coordinates. Do not store separate display coordinates in the new planner.

Conversion to public `GuidancePoint`:

```text
driven segment = distance(previous final XY, current final XY)
coverage segment =
  driven segment when current phase is Cut, Fill, or Grade
  otherwise 0
```

At action boundaries, the inbound segment owns the current point's phase. Ensure the first point of every phase is duplicated only when necessary to express a blade/phase event; event duplicates have zero driven and coverage distance and must be ignored by heading/radius validation.

Heading is the bearing to the next point with non-zero distance. The final heading carries forward the previous non-zero heading.

## 12. Final validation algorithm

Implement `ValidateFinal` in this order.

### 12.1 Continuity

Consecutive points must have distance no greater than the maximum expected sampled step plus 1 mm, except that a connector primitive may explicitly provide longer straight samples. To avoid a false rule, the authoritative continuity condition is:

```text
every phase start equals the preceding phase end within 0.05 m
every pass start is reached by preceding geometry
no NaN or infinity
```

### 12.2 Field containment

- Sample every segment at `<= min(0.5 m, PathSampleSpacingM)`.
- Automatic samples require operational-field membership.
- Manual samples require raw-field membership.

### 12.3 Radius

First resample each continuous automatic polyline at uniform `0.5 m` spacing. Skip duplicate points.

Use a sliding window:

```text
window = max(2, round((R / 4) / 0.5))
for center index i with i-window and i+window available:
  A = p[i-window]
  B = p[i]
  C = p[i+window]
  if absolute heading change A-B-C < 5 degrees: continue
  circumradius = |AB|*|BC|*|CA| / (4*triangleArea)
  require circumradius >= R - max(0.01*R, 0.10m)
```

Do not evaluate across manual assist, zero-distance events, stops, or phase discontinuities. Do evaluate across automatic phase boundaries where motion is continuous.

### 12.4 Lineup

For each blade-down run:

1. Locate the immediately preceding matching lineup phase.
2. Measure final-coordinate polyline length.
3. Require length `>= EquipmentLengthM - 0.05 m`.
4. Require maximum heading deviation within the lineup `<= 1 degree`.
5. Require lineup terminal heading versus first blade-down segment `<= LineupHeadingToleranceDeg`.
6. Require all lineup samples inside the operational field.

### 12.5 Blade rules

- Manual assist is always blade-up.
- Transit, haul, exits, turns, and lineups are blade-up.
- Cut, fill, and grade are blade-down.
- Grade depth is exactly zero.

### 12.6 Coverage

Compare planned cut/fill cell-depth multisets before and after guidance construction. Grade does not add material coverage or volume.

## 13. Timing algorithm

Implement timing from final points in `GuidanceTiming.Estimate`.

### 13.1 Group runs

Create a new motion run when any condition changes:

- pass number;
- phase;
- automatic/manual status;
- blade command;
- explicit stop event.

Do not split a run merely because it contains more waypoints.

### 13.2 Base speed per phase

```text
Cut                 LoadingSpeedMph
Fill                DumpingSpeedMph
Grade               GradeSpeedMph
Haul                 HaulSpeedMph
TransitToCut         ReturnSpeedMph
TransitToNextCut     ReturnSpeedMph
Lineup*              min(ReturnSpeedMph, TurnSpeedMph)
Exit*                min(ReturnSpeedMph, TurnSpeedMph)
TurnToGrade          TurnSpeedMph
ManualAssist         ManualAssistSpeedMph
```

### 13.3 Curvature speed cap

For automatic sampled point `i`, estimate curvature `k=1/r` using the same radius window. Use:

```text
curveCapMps = TurnSpeedMps when r < 2R
curveCapMps = phaseBaseMps otherwise
pointCap = min(phaseBaseMps, curveCapMps)
```

This is intentionally simple and does not model vehicle dynamics.

### 13.4 Forward/backward speed pass

Use constant acceleration values derived from current options:

```text
a = phase maximum speed / max(AccelTimeS, epsilon)
d = phase maximum speed / max(DecelTimeS, epsilon)
```

For distances `ds[i]` and point speed caps:

```text
v[0] = 0 at explicit stop, otherwise min(previous exit speed, cap[0])
forward:  v[i+1] = min(cap[i+1], sqrt(v[i]^2 + 2*a*ds[i]))
backward from required end speed:
          v[i] = min(v[i], sqrt(v[i+1]^2 + 2*d*ds[i]))
segment time = 2*ds / max(v[i]+v[i+1], epsilon)
```

Require zero speed at START, END, manual takeover start/end, and configurable blade-event stops. Initially, lineups flow continuously into their blade-down runs.

Do not call `MoveTime` per waypoint. Do not add separate `R * angle / speed` time.

### 13.5 Overhead

Add `ManualAssistTakeoverS` once per contiguous manual-assist run. Apply field efficiency after productive motion and explicit events, and report it separately.

## 14. Engine integration recipe

Modify `HaulBuilderEngine.Run` in these exact stages.

### Stage A: introduce new planner without changing headline metrics

1. Build `OperationalField` after final route order.
2. Build new guidance unconditionally.
3. Validate it.
4. Store guidance, validation, and new timing on result.
5. Keep existing `TimeResult` and headline metrics temporarily.
6. Add comparison messages for old versus new hours.

### Stage B: switch headline metrics

Only after TheShop2 whole-plan acceptance passes:

1. Set `Metrics.TotalHours` from `GuidanceTime.TotalTimeS`.
2. Derive move and area rates from this total.
3. Retain `TimeResult` as `AllocationTimeEstimate` semantics in documentation.
4. Remove guidance-artifact gating from `BuildGuidance`; gate rendering/export only.

### Stage C: remove old orchestration

1. Remove the engine call to `Swaths.ReprofileKnots`.
2. Make `Swaths.BuildGuidance` a wrapper.
3. Make `Swaths.EstimateGuidanceTime` a wrapper.
4. Confirm no production reference remains to the four legacy repair passes.
5. Remove the temporary route-2 parity dump.

## 15. Exact red/green test fixtures

The following tests must be written before their production work unit.

### 15.1 Operational field

Fixture: `10x10`, all-valid grid, `gridSize=1 m`, `clearance=1 m`.

Expected:

- raw valid count = 100;
- operational valid cells are rows 1-8 and columns 1-8;
- operational valid count = 64;
- `(1.5,1.5)` is operational;
- `(0.5,5.5)` is raw-valid but not operational.

Add a one-cell internal hole and assert erosion removes all cells whose clearance square/disk touches it according to the offset formula.

### 15.2 Radius validator

Generate arcs sampled every `0.5 m`:

- radius `10.0 m`, required `10.0 m`: pass;
- radius `9.4 m`, required `10.0 m`: fail;
- straight line: pass with infinite radius;
- same 10 m arc resampled every `0.25 m`: same result.

### 15.3 Lineup validator

Required length `13.1064 m` and heading tolerance `5 degrees`:

- straight `13.1064 m`, aligned: pass;
- straight `12.0 m`: fail `MissingLineup`;
- length `13.2 m`, final heading error `6 degrees`: fail `LineupHeadingViolation`;
- curved `13.2 m` lineup: fail straightness rule.

### 15.4 Distance separation

Path:

```text
CUT (0,0) -> CUT (10,0)
MOVE arc/connector length 20 m
CUT (10,10) -> CUT (20,10)
```

Expected:

- driven distance = 40 m;
- coverage distance = 20 m;
- no MOVE distance appears as CUT coverage;
- increasing connector sample count does not change totals beyond 1 mm.

### 15.5 Timing sampling invariance

Straight run `100 m`, maximum speed `5 m/s`, acceleration and deceleration time `10 s` from/to that speed.

Expected trapezoidal time:

```text
acceleration distance = 25 m
deceleration distance = 25 m
cruise distance = 50 m
time = 10 + 10 + 10 = 30 s
```

Assert 2-point, 21-point, and 201-point representations all calculate `30 s +/- 0.01 s`.

### 15.6 Post-fill lifecycle without grade

Two synthetic passes, each with one cut run and one fill run.

Expected phase subsequence:

```text
... LineupFill, Fill, ExitFill,
TransitToNextCut, LineupCut, Cut ...
```

Assert the end of `ExitFill` equals the start of `TransitToNextCut`, and the transit end equals the next `LineupCut` start within `0.05 m`.

### 15.7 Post-fill lifecycle with grade

Same two passes, grade enabled.

Expected phase subsequence:

```text
... LineupFill, Fill, ExitFill,
TurnToGrade, LineupGrade, Grade, ExitGrade,
TransitToNextCut, LineupCut, Cut ...
```

Assertions:

- no instantaneous Fill-to-Grade reversal;
- TurnToGrade is blade-up;
- Grade points equal Fill coverage points in reverse order within coordinate tolerance;
- Grade depth is zero;
- TransitToNextCut begins at grade exit, not fill exit.

### 15.8 Manual assist

Construct poses for which no radius-valid automatic connector fits the operational field but a `10 m` raw-field straight correction exists.

Expected:

- one ManualAssist run;
- blade up;
- automatic false;
- length 10 m;
- one takeover overhead;
- raw-field containment passes.

Repeat with `MaxManualAssist=8 m`; planning must fail instead of emitting the segment.

### 15.9 Coverage conservation

Create a knotted list of 12 cells with unique depths. After candidate planning:

- same 12 `(cell,depth)` entries;
- no duplicate cell;
- zero blade-down reversal greater than 100 degrees;
- volume difference below `1e-9 BCY` or the existing stricter practical tolerance.

## 16. Work-unit execution checklist

### Unit 0: shared baseline fixture

Edit:

- add shared builder under `DotNET/Bench` or test helpers;
- update route loop/radius/outside diagnostics to use refined order.

Green gate:

- existing tests pass;
- route 8-10 diagnostics match baseline identities.

### Unit 1: models and distance separation

Edit:

- add `GuidanceModels.cs`;
- extend `GuidancePoint` and CSV;
- do not change path shape.

Green gate:

- distance-separation tests pass;
- existing image geometry remains unchanged.

### Unit 2: operational field and validator skeleton

Edit:

- add `OperationalField.cs`;
- add containment/continuity validation.

Green gate:

- operational-field tests pass;
- validator reports current failures without changing paths.

### Unit 3: radius and lineup validation

Edit:

- complete `GuidanceValidation`.

Green gate:

- numeric radius and lineup fixtures pass;
- current TheShop failures are reported, not suppressed.

### Unit 4: coverage planner

Edit:

- add `CoveragePlanner.cs`;
- extract required lane helpers;
- keep old planner behind test-only comparison during this unit.

Green gate:

- coverage conservation and knot tests pass;
- no production double-processing.

### Unit 5: connector planner

Edit:

- add `GuidanceTransitions.cs`;
- move/expose curve helpers;
- implement candidate rejection and manual fallback.

Green gate:

- connector/radius/field/manual tests pass.

### Unit 6: explicit lifecycle

Edit:

- add `GuidancePlanner.cs`;
- implement grade-off and grade-on state machines.

Green gate:

- both post-fill lifecycle fixtures pass;
- no cross-pass jumps.

### Unit 7: final timing

Edit:

- add `GuidanceTiming.cs`;
- integrate as secondary result.

Green gate:

- timing sampling-invariance test passes;
- grade toggling changes time by the exact added lifecycle.

### Unit 8: planning feedback

Edit:

- add tube/run markers in `Passes.cs` if still required;
- add connector estimates to multi-drop and route sequencing.

Green gate:

- material parity passes;
- deterministic ordering passes;
- TheShop route 8-10 turn burden meets remediation targets.

### Unit 9: engine switch and cleanup

Edit:

- switch headline metrics;
- remove production calls to old post-processes;
- update rendering and docs.

Green gate:

- full .NET suite;
- whole-plan TheShop validator;
- route 1-12 exports;
- no production reference to retired orchestration.

## 17. Mandatory commands after every unit

```powershell
dotnet build DotNET/HaulBuilder.sln -c Release
dotnet test DotNET/HaulBuilder.Tests/HaulBuilder.Tests.csproj -c Release --no-build
```

After Unit 2 and later:

```powershell
dotnet run -c Release --no-build --project DotNET/Bench/Bench.csproj -- --validate-guidance-all
```

After geometry units 4-9:

```powershell
dotnet run -c Release --no-build --project DotNET/Bench/Bench.csproj -- --export-first-routes 12
```

The implementation agent must include command output summaries in its handoff. It must not claim completion when only focused tests pass.

## 18. Stop conditions

Stop and report a blocker instead of improvising when:

1. Cell/depth multiset conservation fails.
2. The only connector leaves the raw field.
3. Manual assist exceeds its configured limit.
4. Final validation fails after all documented candidates are exhausted.
5. A required public API compatibility decision is not covered here.
6. A test can pass only by weakening radius, containment, lineup, or continuity requirements.

Do not solve a stop condition by restoring one of the legacy whole-route repair passes after final validation.

## 19. Final implementation audit

Before declaring the remediation complete, search the production project and verify:

```text
HaulBuilderEngine no longer calls ReprofileKnots
new BuildRoute is called exactly once per engine run
no SmoothLane call occurs after BuildRoute
no EnforceMinTurnRadius whole-route call occurs after BuildRoute
no EnforceEquipmentLineup post-process occurs after BuildRoute
no EnforceBladeUpTransitTurns post-process occurs after BuildRoute
no RepairSharpSteeringBends post-process occurs after BuildRoute
GuidanceTiming consumes DrivenSegDistM/final XY
Metrics.TotalHours comes from GuidanceTime
final validation runs before render/export
```

Then confirm:

- every fill has an explicit post-fill path;
- grade-off ends at the next cut lineup or END;
- grade-on turns, lines up, grades, exits, then travels to the next cut or END;
- manual assist is visible and blade-up;
- routes 8-10 meet the quantitative targets in the remediation plan;
- all automatic final coordinates pass the operational-field and radius contracts; and
- timing is invariant to waypoint sampling density.
