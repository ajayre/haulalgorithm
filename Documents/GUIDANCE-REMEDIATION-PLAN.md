# Guidance and Timing Remediation Plan

> **Implementation companion:** This plan defines requirements and work-package order. A less-capable implementation agent must also follow [GUIDANCE-IMPLEMENTATION-SPEC.md](GUIDANCE-IMPLEMENTATION-SPEC.md), which fixes the target call graph, data contracts, replacement map, pseudocode, and red/green tests. Where the two documents differ in implementation detail, the implementation specification controls; the non-negotiable outcomes in this document still control acceptance.

## 1. Purpose

This plan converts the existing cut/fill guidance into a centerline guidance system that is suitable for supervised autosteer and defensible timing analysis.

The cut/fill allocation and volume ledger are already useful and must be preserved. The work in this plan focuses on:

- eliminating tangled or "knotted" blade-down paths;
- enforcing the configured minimum turning radius on the final exported path;
- providing a real straight lineup before every cut, fill, and grade run;
- ensuring the planned centerline and a conservative equipment envelope remain inside the field;
- explicitly routing every transition, especially the transition after a fill;
- making manual operator intervention explicit rather than silently emitting an invalid maneuver; and
- calculating time from the final route the tractor will actually drive.

Detailed tractor-scraper articulation and offtracking simulation are out of scope. Use a conservative circular/width clearance envelope around the guidance centerline. The operator remains in the cab and may take over for small, explicitly marked, blade-up corrections.

## 2. Non-negotiable outcomes

The implementation is complete only when all of the following are true:

1. Every exported automated waypoint is inside the operational field envelope.
2. Every automated curve on the final exported coordinates respects the configured minimum radius within the documented numerical tolerance.
3. Every automated `CUT`, `FILL`, and `GRADE` run has a verified straight entry lineup.
4. No discontinuity exists between a completed fill and the next operation.
5. With grade return disabled, guidance routes from the fill exit to the next cut lineup.
6. With grade return enabled, guidance turns around, lines up, grades over the completed fill, exits the grade run, and then routes to the next cut lineup.
7. Infeasible automatic connectors are represented as `MANUAL_ASSIST`; they are never silently represented as valid autosteer paths.
8. `MANUAL_ASSIST` is blade-up and remains inside the raw field boundary.
9. Timing is calculated from final driven geometry and does not restart acceleration at every sampled waypoint.
10. Cut/fill cells, depths, and moved volume remain unchanged unless a task explicitly introduces pass splitting with a conservation test.

## 3. Scope and implementation strategy

Implement the C# production path first. Do not attempt simultaneous Python parity while the guidance model is changing. Once the C# acceptance suite passes, port the stabilized behavior to Python in a separate final phase.

Primary implementation files:

- `DotNET/HaulBuilder/Swaths.cs`
- `DotNET/HaulBuilder/Routing.cs`
- `DotNET/HaulBuilder/Passes.cs`
- `DotNET/HaulBuilder/TimeModel.cs`
- `DotNET/HaulBuilder/HaulBuilderEngine.cs`
- `DotNET/HaulBuilder/HaulBuilderOptions.cs`
- `DotNET/HaulBuilder/HaulBuilderResult.cs`

Recommended new files:

- `DotNET/HaulBuilder/OperationalField.cs`
- `DotNET/HaulBuilder/GuidanceValidation.cs`
- `DotNET/HaulBuilder/GuidanceTiming.cs`
- `DotNET/HaulBuilder/GuidanceTransitions.cs`
- `DotNET/HaulBuilder.Tests/GuidanceLifecycleTests.cs`
- `DotNET/HaulBuilder.Tests/GuidanceValidationTests.cs`
- `DotNET/HaulBuilder.Tests/GuidanceTimingTests.cs`
- `DotNET/HaulBuilder.Tests/TheShop2GuidanceAcceptanceTests.cs`

Avoid a single rewrite of `Swaths.cs`. Complete the work packages below in order, with all tests green after each package.

## 4. Definitions

### 4.1 Raw field

The authoritative field polygon or grid mask in which the tractor centerline may exist. For fields with `2PER` points, preserve their survey order when valid; do not automatically centroid-sort a concave boundary. Validate and close the ring.

### 4.2 Operational field

The raw field eroded by a conservative clearance:

```text
operational clearance
    = equipment width / 2
    + configurable boundary margin
```

This is a centerline approximation, not a detailed articulated vehicle model.

Suggested new option:

```csharp
public Double BoundaryMarginFt { get; set; } = 2.0;
```

The erosion may be raster-based initially. It must be deterministic and conservative. A waypoint is automatic only if it lies in the operational field; a `MANUAL_ASSIST` waypoint must at minimum lie inside the raw field.

### 4.3 Coverage path

The blade-down centerline needed to cover assigned cut or fill cells. Coverage distance is an accounting value and must not be confused with driven distance.

### 4.4 Driven path

The final continuous path followed by the tractor, including lineups, turns, haul, grade return, repositioning, and travel to the next cut.

### 4.5 Manual assist

A short, explicitly identified blade-up segment for which the operator must temporarily take control. It is allowed only when automatic connection planning fails. It is not permission to leave the raw field or to perform blade-down work manually without a separate design decision.

## 5. Required guidance lifecycle

Replace implicit concatenation with an explicit state machine. For every pass `i`, generate the following phases in order.

```text
START or previous pass exit
  -> TRANSIT_TO_CUT
  -> LINEUP_CUT
  -> CUT
  -> EXIT_CUT
  -> HAUL
  -> LINEUP_FILL
  -> FILL
  -> EXIT_FILL
  -> optional grade-return lifecycle
  -> TRANSIT_TO_NEXT_CUT or END
```

When grade return is disabled:

```text
EXIT_FILL
  -> blade-up route from the fill exit pose
  -> next pass LINEUP_CUT
```

When grade return is enabled:

```text
EXIT_FILL
  -> TURN_TO_GRADE (blade up, in field, radius valid)
  -> LINEUP_GRADE
  -> GRADE back over the completed fill path
  -> EXIT_GRADE
  -> blade-up route from the grade exit pose
  -> next pass LINEUP_CUT
```

For a fill containing multiple continuous runs, grade the runs in reverse drive order. Use blade-up connectors between disconnected runs. Never jump from the end of one run to the start of another.

The first point of a new pass must be reached from the preceding pass through generated geometry. A change in `PassNo` is not itself a route.

## 6. Work package 0: freeze and measure the baseline

### Goal

Create reproducible diagnostics before changing behavior.

### Tasks

1. Update the existing route diagnostics so all of them use the same pipeline as `ExportFirstRoutes`:
   - generate passes;
   - sequence passes;
   - reprofile knots;
   - refine route order; and
   - build guidance with the same options.
2. Create one shared test/diagnostic fixture builder instead of copying pipeline code between diagnostic classes.
3. Record for every TheShop2 pass:
   - pass index and original pass identity;
   - cut/fill cell count;
   - raw near-reversal count;
   - automatic MOVE distance;
   - number and length of MOVE runs;
   - number of keyhole-like runs;
   - minimum observed radius on final exported coordinates;
   - raw-field and operational-field violations;
   - achieved entry lineup length for CUT, FILL, and GRADE;
   - manual-assist distance; and
   - final calculated time.
4. Save the baseline as a human-readable text or CSV artifact under `DotNET/Bench`, not as assertions that lock in bad behavior.

### Baseline facts that must be retained in the report

- Route 8: 25 cut cells, 4 raw near-reversals, 9 MOVE runs, 5 keyhole-like runs, approximately 325 m MOVE.
- Route 9: 25 cut cells, 5 raw near-reversals, 8 MOVE runs, 3 keyhole-like runs, approximately 195 m MOVE.
- Route 10: 27 cut cells, 3 raw near-reversals, 7 MOVE runs, 3 keyhole-like runs, approximately 195 m MOVE.

### Tests

- Diagnostic route numbering matches `route_08.png`, `route_09.png`, and `route_10.png` after route refinement.
- Running the diagnostic twice produces identical results.

### Completion gate

No production guidance behavior changes in this work package. All existing tests pass.

## 7. Work package 1: separate geometry and accounting

### Goal

Create one unambiguous final path and stop using coverage distance as driven distance.

### Tasks

1. Extend `GuidancePoint` with separate values:

```csharp
Double DrivenSegDistM
Double CoverageSegDistM
Double DrivenCumDistM
Double CoverageCumDistM
Boolean Automatic
String Phase
```

2. Keep `SegDistM` temporarily as a compatibility alias for `DrivenSegDistM`. Mark it obsolete in documentation, not with a compiler attribute until consumers are migrated.
3. Calculate `DrivenSegDistM` from consecutive final exported `X/Y` coordinates only.
4. Calculate coverage distance separately from blade-down coverage centerlines.
5. Stop exporting coordinates derived from one geometry while validating and timing another. The coordinates tested for containment and curvature must be exactly those exported in CSV.
6. Retain raw/planned coordinates internally only for diagnostics.
7. Add an internal action/phase representation. Strings may remain in CSV, but production code should not infer lifecycle solely from adjacent string values.

### Tests

- Sum of `DrivenSegDistM` equals the polyline length of exported `X/Y` to within 1 mm per route plus floating-point accumulation tolerance.
- `CoverageSegDistM` is zero for blade-up phases.
- Inserting a MOVE headland increases driven distance but does not increase coverage distance.
- Smoothing cannot change exported geometry without changing driven distance and re-running validation.
- Existing CSV columns remain readable; new columns are appended rather than reordered.

### Completion gate

No timing code may use `CoverageSegDistM` as vehicle travel.

## 8. Work package 2: operational field and final-path validator

### Goal

Turn containment, curvature, continuity, and lineup into measurable contracts.

### Tasks

1. Implement `OperationalField` with:
   - raw mask lookup;
   - operational mask lookup;
   - segment sampling at no more than `min(gridSize/4, 0.5 m)`;
   - conservative erosion based on equipment half-width and boundary margin; and
   - optional exclusion-zone integration.
2. Implement `GuidanceValidation.Validate(...)` returning structured issues, not only a Boolean.
3. Validate:
   - every waypoint;
   - every segment between waypoints;
   - continuity between phases and passes;
   - duplicate/zero-length points where they alter headings;
   - minimum radius on the final exported path;
   - straight lineup length and heading tolerance;
   - blade state rules;
   - maximum length of `MANUAL_ASSIST`; and
   - coverage and volume invariants.
4. Use a radius check robust to sampling. Evaluate heading change over traveled arc length or a sliding circumcircle window; do not rely only on one three-point combination.
5. Add configurable validation tolerances:

```text
radius tolerance: max(1%, 0.10 m)
lineup heading tolerance: 5 degrees
path continuity tolerance: 0.05 m
```

6. Run validation after every transformation and once at the end. The final validation result is authoritative.
7. Store validation results on `HaulBuilderResult` and optionally export a validation CSV.

### Tests

- A segment whose endpoints are inside but whose interior crosses outside is rejected.
- A centerline inside the raw field but outside the eroded operational field is rejected for automatic operation.
- The configured equipment width and margin change the operational mask predictably.
- An exact-radius arc passes; a `0.95R` arc fails.
- A smoothed path that violates radius fails even if its raw path passed.
- A cross-pass coordinate jump fails continuity validation.
- `MANUAL_ASSIST` outside the raw field fails.

### Completion gate

The engine can produce a complete validation report for every TheShop2 route, even though some routes will still fail at this stage.

## 9. Work package 3: eliminate knots before turn insertion

### Goal

Produce simple blade-down coverage runs before adding headland turns.

### Tasks

1. Split a flow-ordered cell list when any of the following is true:
   - cells are spatially disconnected;
   - heading reverses beyond a configurable threshold, initially 100 degrees;
   - accumulated local curvature would require less than the configured radius; or
   - a new multi-drop tube begins.
2. Treat each split as a separate coverage run connected blade-up. Do not let 8-neighbor adjacency hide a direction reversal.
3. Replace the current knot-repair score with candidate trajectory cost.
4. Generate candidate sweep headings at 5-degree intervals initially. Include:
   - principal-axis heading;
   - local flow heading;
   - haul arrival/departure heading; and
   - boundary-aligned headings where available.
5. For each candidate, calculate:
   - number of lanes;
   - total coverage length;
   - estimated headland connector length;
   - number of reversals;
   - lineup feasibility;
   - operational-field feasibility; and
   - estimated execution time.
6. Use `AlongSpan` as part of scoring; do not ignore it when lane counts tie.
7. Prefer candidates in this order:
   - fully automatic and valid;
   - fewest headland reversals;
   - lowest estimated time;
   - shortest driven distance;
   - deterministic heading tie-break.
8. If no automatic candidate exists:
   - split the patch into connected or directionally coherent sub-patches;
   - retry each sub-patch;
   - preserve all cells and depths; and
   - only then permit a short blade-up `MANUAL_ASSIST` connector.
9. Do not allow `MANUAL_ASSIST` during CUT, FILL, or GRADE.

### Tests

- A synthetic spiral/knotted cell order is converted into simple runs with no blade-down near-reversal.
- Adjacent cells with opposing headings are split despite being 8-neighbors.
- Candidate selection is deterministic.
- Every original cell and depth appears exactly once after reordering/splitting.
- Total cut and fill BCY are unchanged.
- Routes 8-10 have zero blade-down near-reversals after coverage planning.

### Completion gate

Turn insertion receives only continuous, directionally coherent blade-down runs.

## 10. Work package 4: explicit connectors and post-fill routing

### Goal

Generate every blade-up movement between operational phases, including the previously omitted route after a fill.

### Tasks

1. Introduce a `Pose` value containing `X`, `Y`, and `HeadingDeg`.
2. Make every coverage run expose:
   - lineup-start pose;
   - blade-engage pose;
   - blade-disengage pose; and
   - exit-lineup pose.
3. Implement one connector API:

```csharp
ConnectorResult Connect(
    Pose start,
    Pose goal,
    OperationalField field,
    Double minRadiusM,
    ConnectorPurpose purpose);
```

4. Candidate connectors may use the existing CSC/CCC/keyhole helpers, but they must be scored and validated on their final sampled coordinates.
5. Prefer connectors in this order:
   - straight;
   - one same-side CSC bend;
   - other valid CSC;
   - one compact keyhole when genuinely required;
   - `MANUAL_ASSIST` fallback.
6. Never emit multiple full-circle/keyhole maneuvers for one connector.
7. Build guidance over the complete route rather than building isolated mini-routes and relying on later discontinuity repair.
8. Implement the no-grade post-fill transition:
   - complete FILL;
   - follow `EXIT_FILL` lineup with blade up;
   - connect from the exit pose to the next pass's cut-lineup start pose;
   - label the connector `RETURN` or `TRANSIT_TO_CUT`; and
   - enter `LINEUP_CUT` before engaging the blade.
9. Implement the grade-return transition:
   - complete FILL;
   - follow `EXIT_FILL` with blade up;
   - create an in-field, radius-valid turnaround to the grade-lineup start;
   - traverse a straight `LINEUP_GRADE` segment;
   - drive the completed fill coverage in reverse run order as `GRADE`;
   - use blade-up connectors between disconnected fill runs;
   - follow `EXIT_GRADE` with blade up; and
   - connect from the grade exit pose to the next cut-lineup start pose.
10. For the last pass, replace transit to the next cut with an explicit `END` pose.
11. Add a maximum automatic connector cost. A bizarre but mathematically valid series of loops must lose to a short manual assist.

### Tests

Create lifecycle tests for all four combinations:

| Case | Grade return | Next pass | Expected result |
|---|---|---|---|
| A | Off | Yes | Fill exit routes to next cut lineup |
| B | On | Yes | Turn, lineup, grade fill, then route to next cut |
| C | Off | No | Fill exit routes to END |
| D | On | No | Turn, lineup, grade fill, then route to END |

Additional assertions:

- No coordinate jump occurs at a pass boundary.
- Grade follows the completed fill geometry in reverse run order.
- The turnaround before grade is blade-up.
- Grade depth is zero and blade state is the configured grading state.
- No-grade mode contains no `GRADE`, `LINEUP_GRADE`, or `TURN_TO_GRADE` phases.
- A failed automatic connector creates one bounded `MANUAL_ASSIST`, not an invalid curve.

### Completion gate

Every pass transition is explicit, continuous, timed, rendered, and validated.

## 11. Work package 5: make lineup a contract

### Goal

Ensure the tractor is genuinely aligned before blade engagement.

### Tasks

1. Define required lineup length from `EquipmentLengthFt`.
2. A valid lineup must be:
   - straight for the required length;
   - inside the operational field;
   - within 5 degrees of the first blade-down segment; and
   - immediately adjacent to blade engagement, with no turn between lineup and engagement.
3. Apply the same rule to CUT, FILL, and GRADE.
4. Choose coverage orientation partly from lineup feasibility. Do not choose a coverage direction and later shorten its lineup silently.
5. If full lineup does not fit:
   - try reversing the coverage run;
   - try another sweep heading;
   - split the run;
   - try a shorter configurable supervised-autosteer lineup only if enabled; or
   - mark a blade-up `MANUAL_ASSIST` alignment phase.
6. Add options:

```csharp
public Boolean AllowManualAssist { get; set; } = true;
public Double MaxManualAssistFt { get; set; } = 45.0;
public Double ManualAssistSpeedMph { get; set; } = 2.0;
public Double LineupHeadingToleranceDeg { get; set; } = 5.0;
```

7. Never label a shortened lineup as a valid full lineup.

### Tests

- Full 43 ft lineup is measured on final geometry.
- Reversing a run is selected when it makes lineup feasible.
- A boundary-constrained run is split or marked for manual assist rather than silently receiving a short lineup.
- No blade-down action begins immediately after a curved segment.

### Completion gate

Every blade engagement reports `AutomaticLineupValid=true` or has a preceding explicit manual-assist alignment phase.

## 12. Work package 6: simplify and certify turns

### Goal

Retain existing useful turn primitives while preventing loop multiplication.

### Tasks

1. Remove repeated repair passes that independently reinterpret the same corner.
2. Build each connector once, then validate it. If invalid, select another candidate or return failure.
3. Do not Gaussian-smooth after radius validation. Either:
   - smooth before connector construction and validate afterward; or
   - use the sampled connector geometry directly without further smoothing.
4. Coalesce one lane reversal into at most one headland maneuver.
5. Reject connector candidates whose length exceeds a purpose-specific bound, for example:

```text
soft connector: <= straight distance + pi * R
single keyhole: <= straight distance + 3 * pi * R
```

The exact constants may be tuned, but a candidate cannot grow without limit.
6. Preserve the raw field as a hard bound and the operational field as the automated bound.
7. Add a final full-route validation call after all connectors and transitions are present.

### Tests

- One reversal produces at most one keyhole-like MOVE run.
- A soft 20-90 degree bend never becomes a full-circle bulb.
- No post-validation smoothing can introduce a radius violation.
- All final TheShop2 automatic segments satisfy the radius constraint.

### Completion gate

The final exported path—not an internal precursor—passes radius and containment validation.

## 13. Work package 7: rebuild timing from final guidance

### Goal

Calculate time from the route actually driven.

### Tasks

1. Move guidance timing into `GuidanceTiming.cs` and make it the source of headline job hours when guidance is generated.
2. Always generate lightweight final guidance for timing, even when PNG/CSV output is not requested. Rendering remains optional.
3. Group adjacent points into continuous motion runs by phase, blade state, pass, and stop condition.
4. Apply acceleration and deceleration once per continuous run, not once per waypoint.
5. Calculate motion time with a forward/backward speed pass:
   - phase maximum speed;
   - curvature-based speed cap;
   - acceleration limit;
   - deceleration needed for the following run; and
   - explicit stops or blade-state events.
6. Use existing speed options for CUT, HAUL, FILL, RETURN, GRADE, and TURN.
7. Add explicit fixed event durations as configurable options, initially defaulting to zero until calibrated:
   - blade lower;
   - blade raise;
   - scraper transition;
   - operator/manual-assist takeover overhead.
8. Time `MANUAL_ASSIST` at `ManualAssistSpeedMph` plus the configured takeover overhead.
9. Do not add `R * turnAngle / turnSpeed` on top of arc travel. Curved-path distance and curvature speed limits already represent turn travel.
10. Keep field efficiency as a separately reported allowance:

```text
modeled productive time
+ explicit operational events
+ field-efficiency allowance
= reported job time
```

11. Report both base allocation time and final-guidance time during migration, but make final-guidance time the primary metric once acceptance tests pass.

### Tests

- Subdividing one straight 100 m run into more waypoints does not change its calculated time.
- One 50 m arc is not charged once as MOVE and again as a turn.
- Enabling grade return adds turnaround, lineup, grade, grade exit, and subsequent transit time.
- Disabling grade return still includes fill-exit-to-next-cut transit.
- Manual assist time uses manual-assist speed and overhead.
- Final total equals the sum of reported phase totals and allowance.

### Completion gate

Timing is invariant to waypoint sampling density and accounts for every post-fill movement.

## 14. Work package 8: feed executable cost back into planning

### Goal

Prevent pass and route selection from preferring geometrically cheap but operationally expensive work.

### Tasks

1. Add a lightweight coverage feasibility estimate during pass construction.
2. Change multi-drop acceptance from Euclidean distance alone to:

```text
candidate hop cost
    = feasible connector time
    + added lineup time
    + added coverage-turn time
    + manual-assist penalty
```

3. Reject a multi-drop hop when:
   - no automatic or allowed manual connector exists;
   - it creates a blade-down reversal;
   - required lineup cannot be provided; or
   - estimated connector time exceeds the time saved by filling the load further.
4. Extend route sequencing cost with each pass's validated exit pose and next pass's lineup-start pose.
5. Run route refinement against actual connector estimates rather than center-to-center deadhead only.
6. Cache connector estimates so the route optimizer remains practical.
7. Preserve a deterministic tie-break based on original pass identity.

### Tests

- A nearby pocket requiring a keyhole loses to a slightly farther pocket with a straight connector when total time is lower.
- Disabling multi-drop still preserves volume and terminates.
- Route ordering is deterministic.
- Route optimization never chooses an infeasible transition when a feasible alternative exists.

### Completion gate

The planner's optimization objective correlates with final driven time and turn burden.

## 15. Work package 9: TheShop2 acceptance suite

### Goal

Make the primary fixture a whole-plan regression test, not a collection of selected route checks.

### Required whole-plan assertions

For `DotNET/TheShop2_2ft.agd` with default settings:

1. Pass generation completes and volume parity remains within the existing tolerance.
2. Every final automatic point and sampled segment is in the operational field.
3. Every manual-assist point and sampled segment is inside the raw field.
4. Every automatic curve respects the configured 7.62 m minimum radius within tolerance.
5. Every CUT, FILL, and GRADE run has a valid lineup or an explicitly allowed manual alignment.
6. No blade-down near-reversal remains.
7. No pass-boundary discontinuity exists.
8. The grade-off lifecycle routes every fill exit to the next cut.
9. The grade-on lifecycle turns around, grades the completed fill, then routes to the next cut.
10. Sampling the same geometry more densely does not materially change time.
11. No cell is cut or filled twice within one load.
12. All assigned cell depths remain at or below the configured per-pass maximum.

### Route 8-10 improvement gates

Use the refined route identities that correspond to the exported images.

- Zero blade-down near-reversals.
- At most one keyhole-like automatic MOVE run per route; target zero.
- Route 8 automatic MOVE distance below 130 m unless a documented geometric lower bound proves this infeasible.
- Routes 9 and 10 automatic MOVE distance below 100 m each unless a documented geometric lower bound proves this infeasible.
- No automatic full-circle loop used merely to obtain lineup.
- Any manual assist is blade-up, individually bounded by `MaxManualAssistFt`, and clearly rendered/reported.

### Visual artifacts

Continue exporting routes 1-12. Update the renderer so colors distinguish:

- CUT;
- HAUL;
- FILL;
- GRADE;
- automatic blade-up transit;
- lineup;
- manual assist; and
- validation failure markers.

The PNG is diagnostic only. Tests must inspect structured geometry, not pixels.

## 16. Work package 10: API, CSV, and documentation migration

### Tasks

1. Add new options with documented defaults.
2. Add validation and phase summaries to `HaulBuilderResult`.
3. Append CSV fields:

```text
phase
automatic
driven_seg_dist_m
coverage_seg_dist_m
driven_cum_dist_m
coverage_cum_dist_m
validation_status
```

4. Preserve existing columns during one compatibility period.
5. Document the difference between:
   - raw field;
   - operational field;
   - automatic guidance; and
   - manual assist.
6. Document that centerline clearance is a conservative approximation and not a detailed articulated equipment simulation.
7. Remove the temporary route-2 parity diagnostic from `HaulBuilderEngine` after equivalent structured diagnostics exist.
8. Port stable behavior to Python and add parity tests for data and metrics. Exact waypoint-by-waypoint parity is not required if both final paths pass the same contracts.

## 17. Test commands

Run from the repository root.

```powershell
dotnet build DotNET/HaulBuilder.sln -c Release
```

```powershell
dotnet test DotNET/HaulBuilder.Tests/HaulBuilder.Tests.csproj -c Release --no-build
```

```powershell
dotnet run -c Release --no-build --project DotNET/Bench/Bench.csproj -- --export-first-routes 12
```

Add one new diagnostic command and run it after every geometry-related package:

```powershell
dotnet run -c Release --no-build --project DotNET/Bench/Bench.csproj -- --validate-guidance-all
```

When Python is ported:

```powershell
python -m pytest Python/tests -q
```

## 18. Rules for the implementing AI

1. Work on one work package at a time.
2. Before editing, identify the exact existing functions being replaced or wrapped.
3. Add failing focused tests before changing production behavior.
4. Do not weaken an assertion merely to make the suite pass.
5. Do not change cut/fill volumes while solving a routing problem unless the active work package explicitly permits pass splitting.
6. Do not accept an out-of-field path as a fallback.
7. Do not accept a short lineup while reporting it as a full lineup.
8. Do not hide infeasibility. Return a structured failure or explicit `MANUAL_ASSIST`.
9. Do not use image appearance as proof of curvature or containment.
10. Validate the exact coordinates written to CSV.
11. Keep deterministic ordering and fixed tie-breaks.
12. After each package, run the focused tests, the whole .NET suite, and the full TheShop2 guidance validator.
13. Record before/after route 8-10 metrics in the commit or handoff notes.
14. Avoid unrelated refactors while guidance behavior is under active change.

## 19. Definition of done

This remediation is done when:

- the complete fill-to-next-cut and fill-to-grade-to-next-cut lifecycles are explicit;
- final exported automatic geometry passes whole-plan containment, radius, continuity, and lineup validation;
- the knot-heavy routes no longer expand into repeated keyhole loops;
- manual intervention is short, blade-up, bounded, and visible;
- timing is based on continuous final guidance and is invariant to waypoint sampling density;
- headline metrics use the final guidance time;
- the full .NET test suite and TheShop2 whole-plan acceptance suite pass; and
- the updated route 1-12 images and metrics have been reviewed against the baseline.
