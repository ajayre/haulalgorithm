# Haul Optimization and Refinement Specification **[0.0.1]**

**[0.0.2]** Paragraph identifiers use **`[section.subsection.paragraph]`** and are the stable references for implementation, tests, reviews, benchmark findings, and change requests.

**[0.0.3]** **Project name:** Haul Algorithm.

**[0.0.4]** **Document type:** Application-level allocation, optimization, and validation specification.

**[0.0.5]** **Language and implementation:** C# in `DotNET/HaulBuilder`, with deterministic diagnostics and acceptance fixtures in `DotNET/Bench` and `DotNET/HaulBuilder.Tests`.

**[0.0.6]** **Scope:** This specification replaces the local-only haul-refinement design with a lane-first, flow-constrained, machine-cycle optimizer. The optimizer constructs a new complete cut-haul-fill allocation from the unchanged cell ledger, sequences the resulting cycles, evaluates them through the production guidance and timing implementation, and applies bounded local repair only after the global plan exists.

**[0.0.7]** **Relationship to `LAND-LEVELING-SPEC-2.md`:** The baseline remains normative for AGD parsing, target-surface construction, cut/fill ratio, grid construction, volume conversion, scraper capacity, maximum per-pass depth, and equipment settings. This document unconditionally supersedes the baseline's retired greedy pass-matching procedure.

**[0.0.8]** **Relationship to `GUIDANCE-IMPLEMENTATION-SPEC.md`:** Guidance rules are immutable inputs to this optimization. This document does not weaken or reinterpret forward-only travel, minimum turn radius, field containment, the one-grid-cell exterior manual allowance, equipment lineups, fill coverage, mandatory grade return, path continuity, or final-centerline timing.

**[0.0.9]** **Relationship to `GUIDANCE-TIMING-AUDIT.md`:** The promoted full-field guidance report is the measured performance baseline. Its timing implementation and final centerline are authoritative for acceptance comparisons until a later audited baseline is deliberately promoted.

**[0.0.10]** **Implementation status:** Production optimized path implemented. The canonical ledger, depth-layer lanes, balanced cycle packing, equal-load global pair refinement, adaptive fill-redistribution envelopes, compact production-guidance preflight, pragmatic difficult-cycle deferral, capacity-flexible consolidation, exact-gated field-wide cycle-chain refinement, focused unit tests, and non-interactive exact benchmark are present. The pragmatic profile and its exact deferred-work summary are recorded in section 0.2.

**[0.0.11]** **Final default:** The lane-first optimizer is the only production allocation path. The legacy `Passes.GeneratePasses` implementation is internal diagnostic code and shall not be exposed by `HaulBuilderOptions` or the public C# API.

### 0.1 Specification revision history

**[0.1.1]**

| Version | Date | Change |
|---|---|---|
| **1.0.0** | 2026-07-13 | Initial local refinement specification. |
| **2.0.0** | 2026-07-15 | Replaced local-only refinement with global lane-first allocation, flow-constrained matching, exact guidance costing, a fixed-work performance gate, explicit deferred-work accounting, and a checkpointed iteration harness. |
| **2.1.0** | 2026-07-15 | Recorded the first implemented exact checkpoint, its explicit perimeter deferral, test status, and unresolved 50% gate. |
| **2.2.0** | 2026-07-15 | Promoted six-run cycle packing, deterministic equal-load pair refinement, exact compact-cycle preflight, and an opt-in bounded difficult-work ledger; recorded both the best complete automatic-work checkpoint and the first raw-time checkpoint below 50% of baseline. |
| **2.3.0** | 2026-07-15 | Added geometry-derived, proximity-component fill redistribution envelopes with mandatory grade-return validation, exact route diagnostics, focused tests, and the improved complete-work checkpoint. |
| **2.4.0** | 2026-07-15 | Added residual-ledger edge completion, independently frozen interior and edge route batches, edge-only guidance repair and bounded omission, deterministic random exact-route PNG export, and the first exact edge-completion checkpoint. |
| **2.5.0** | 2026-07-15 | Expanded exact fill-envelope heading finalists, recorded the improved full-guidance checkpoint, and documented rejected local-flow, global-flow, endpoint-pairing, and long-gap experiments. |
| **2.6.0** | 2026-07-15 | Redefined the existing heading input as a normalized undirected lane axis, added arbitrary-angle lane construction for rectangular fields, retained per-run bearing selection by guidance, and added consecutive-route diagnostic rendering. |
| **2.7.0** | 2026-07-15 | Added compact-cut endpoint recovery before fragmented-heading fallback, required every recovered outer cut endpoint to retain complete coverage, added blade-down internal-shortfall tests and exact cut-run diagnostics, and recorded the improved whole-field checkpoint. |
| **2.8.0** | 2026-07-16 | Added an optional bounded multi-route fragment exchange, exact pooled-ledger transactions, per-route invariant quotas, compact-guidance/run-count acceptance gates, production and benchmark controls, focused tests, and the improved exact checkpoint. |
| **2.9.0** | 2026-07-16 | Promoted ten-slice cycle packing and residual edge completion, added capacity-flexible balanced route consolidation, made manual-cleanup volume a first-class result, added multi-start open-route sequencing, and recorded the exact full-field checkpoint. |
| **2.10.0** | 2026-07-16 | Added a field-wide opportunity audit and an opt-in whole-fill cycle-chain optimizer that couples cut/fill ownership to next-cut routing, scores exchanges in production seconds, preserves exact ledgers and route loads, and compact-preflights every changed route. |
| **2.11.0** | 2026-07-16 | Established `200 BCY` per deterministic production-guidance hour as the normalized cross-field golden target and added separate per-acre, deterministic, scheduled, achievement, and target-status result metrics. |
| **2.12.0** | 2026-07-16 | Added grade-return-aware fill-bearing selection, corrected the fill endpoint used for next-cut lookahead, exact-valid A/B orientation screening, focused conservation tests, and consecutive-route diagnostic rendering. |
| **2.13.0** | 2026-07-16 | Promoted the measured optimizer as the sole production path, removed legacy and unused C# options, grouped expert search tuning under `HaulOptimizationOptions`, clarified timing and lane-axis names, and added a reflection-locked API contract. |

### 0.2 Current implementation checkpoint

**[0.2.1]** The current production-guidance-feasible profile uses north-south lanes, at most six same-kind lane slices per cycle, unchanged Poisson flow with weight `1.0`, eight deterministic equal-load fill-pair refinement rounds using centroid distance, a route-relocation window of `200`, and explicit field-edge deferral.

**[0.2.2]** Without additional compact-cycle omission, the profile produces `3,043` automatic cycles, `236.004 h` deterministic, `295.005 h` scheduled, and `2,225,765.0 m` final centerline. This is a `45.556%` deterministic improvement over `433.483 h`, but remains `19.262 h` above the fixed-work release limit.

**[0.2.3]** The profile explicitly defers `1,354.234 BCY` cut and `1,234.005 BCY` fill at the field edge, equal to `2.896%` and `2.646%` of the respective required ledgers. The benchmark reports an approximately `2.221 h` blade-work-only lower bound for this deferred ledger; transport and completion planning remain unresolved.

**[0.2.4]** With opt-in bounded omission of the `149` most fragmented cycles that fail the compact production-guidance preflight, the exact routed plan contains `2,894` cycles and completes in `216.732 h` deterministic, `270.914 h` scheduled, and `2,054,649.5 m`. This is `50.002%` below baseline and passes every guidance rule for retained cycles. The total explicit deferred ledger becomes `3,246.927 BCY` cut and `3,126.698 BCY` fill, or `6.943%` and `6.703%`. The blade-work-only lower bound for this complete deferred ledger is `7.222 h`; transport and maneuvering are additional. This is a valid interim difficult-work checkpoint, not a fixed-work release result.

**[0.2.5]** The `149`-cycle checkpoint has `57.607 h` loaded transit, `32.803 h` transit to the next cut, `26.464 h` fill-to-grade turning, `24.308 h` acceleration overhead, `92` manual assists, and zero sequencing failures. The next optimization target is completion or reinsertion of the deferred ledger without exceeding the approximately `0.010 h` remaining raw-time margin. The adjacent `148`-cycle candidate is exact-valid but fails the time gate at `216.894 h`, establishing `149` as the minimum omission count under the measured most-fragmented-first ranking.

**[0.2.6]** Focused `CoveragePlannerTests` and `LifecyclePlannerTests` pass `22/22`, and the Release benchmark builds successfully. The complete suite reports `107` passed and the same `11` known failures in lineup, Python pass/route parity, guidance conservation, engine-guidance, and PNG-guidance contracts. The production optimizer remains default-off until those failures and the fixed-work performance gate are resolved.

**[0.2.7]** Every production run uses the measured lane-first optimizer. The current defaults use a north-south lane axis, ten source-lane slices per cycle, twelve residual-edge slices, eight centroid-refinement rounds, Poisson flow weight `1.0`, capacity-flexible consolidation, exact-gated cycle-chain refinement, and route window `200`. Difficult residual work is reported explicitly through `HaulBuilderResult.DeferredWorkSummary`; there is no public arbitrary omission-count control.

**[0.2.8]** The current complete automatic-work checkpoint adds adaptive fill redistribution envelopes without changing pairing, target surface, cut/fill ratio, allocation ownership, or the deferred ledger. It produces `3,043` cycles, `229.290 h` deterministic, `286.612 h` scheduled, `2,172,722.3 m`, `54,221` lifecycle segments, and `257` manual assists with zero sequence failures. This is `47.105%` below the `433.483 h` baseline and remains `12.548 h` above the fixed-work release limit.

**[0.2.9]** Relative to the `236.004 h` checkpoint in [0.2.2], adaptive whole-allocation and proximity-component envelopes save `6.714 h` deterministic and `53,042.7 m`. The principal measured reduction is blade-up transit and repeated lineup/exit geometry; blade-down fill and grade distance increases slightly because accepted bridge corridors deliberately redistribute temporary overfill during the mandatory grade return.

**[0.2.10]** The exact diagnostic canaries prove the intended component behavior. Routes `2526`, `2613`, `2728`, and `2858` each change from fourteen selected fill fragments to two final runs: one `25`-cell north-south corridor and one retained isolated cell. Route `2488` changes from seventeen pre-envelope fill fragments, or three fixed-heading envelope runs, to one geometry-selected corridor. Every canary retains zero guidance omissions and an automatic cut-to-fill link.

**[0.2.11]** The reproducible command is `dotnet DotNET/Bench/bin/Release/net8.0/Bench.dll --optimize-haul DotNET/TheShop2_2ft.agd northsouth 6 --defer-edge --reorder-window=200 --flow-weight=1.0 --pair-2opt --exact`. `--diagnostic-pass=N[,N...]` additionally prints each selected exact fill run, its allocation bounds, heading, length, and applicable field-edge headings.

**[0.2.12]** Residual-ledger edge completion now freezes the exact `3,043`-cycle interior allocation and route order, constructs supplemental cycles from required minus assigned work, routes and refines that batch independently, and concatenates it after the interior batch. This avoids the `364.937 h` global-rebuild regression caused by allowing perimeter work to perturb every interior pairing and route. The residual ledger preserves exact cell depths, volume conversion, cut/fill ratio, target surface, and the original flow field.

**[0.2.13]** With at most twelve same-kind residual lane slices per supplemental cycle, equal-load pair refinement, edge-only compact pair repair, and omission of the nine residual compact failures, the exact plan contains `3,182` guided cycles and completes in `240.554 h` deterministic, `300.692 h` scheduled, and `2,298,100.9 m`. It contains `56,945` lifecycle segments and `368` manual assists, with zero sequence failures and no relaxed guidance rule. This is `44.507%` below the `433.483 h` baseline and adds `11.264 h` to the `229.290 h` interior-only checkpoint.

**[0.2.14]** The edge checkpoint completes `1,168.456 BCY`, or `94.688%`, of the previously deferred matched edge work. The remaining guidance-related deferral is `65.549 BCY` cut and `65.549 BCY` fill, equal to `0.1405%` of required fill. The reported final cut deferral is `185.778 BCY` because it also includes the immutable `120.229 BCY` field cut surplus. These nine cycles remain unresolved work under section 12 and shall not be described as completed.

**[0.2.15]** The edge-completion command is `dotnet DotNET/Bench/bin/Release/net8.0/Bench.dll --optimize-haul DotNET/TheShop2_2ft.agd northsouth 6 --defer-edge --complete-deferred-edge --edge-max-runs=12 --reorder-window=200 --flow-weight=1.0 --pair-2opt --exact`. `--render-random-routes=N --render-seed=S --render-dir=PATH` exports a deterministic random sample using the exact production lifecycle geometry rather than proxy polylines. `--render-leveling-gif=PATH` exports a 101-frame split cut/fill and topology replay with `100 ms` progress frames and a `4,000 ms` final hold, giving exactly `14.0 s` total duration.

**[0.2.16]** Expanding the fill-envelope production-sequencing portfolio from the best three screened headings to the best eight reduces the full plan to `240.440 h` deterministic, `300.550 h` scheduled, and `2,296,941.9 m`. The exact result contains `56,913` lifecycle segments and `369` manual assists, has zero sequence failures, retains the same `3,182` cycles and deferred ledger, and relaxes no guidance rule. This saves `0.114 h` and `1,159.0 m` relative to [0.2.13]. Sixteen finalists reproduce the same plan and are therefore rejected as unnecessary runtime.

**[0.2.17]** Flow-following experiments establish that coherence is more important than local angular agreement on this field. Fifteen-degree local flow sectors with an eight-cell smoothing radius create `16,558` lane fragments and regress to `337.422` proxy hours. A field-wide smoothed flow heading reduces fragmentation to `6,495` lanes but still regresses to `283.269` proxy hours. Both are rejected. Endpoint-only equal-volume fill refinement improves the proxy from `255.980 h` to `254.759 h` but regresses exact guidance to `241.320 h` and `2,312,817.7 m`; it is also rejected. Extending the accepted fill-envelope gap from twelve to twenty-four cells remains exact-valid but regresses to `240.561 h`, because added fill and reverse-grade distance exceeds the saved maneuver distance. The retained policy is therefore coherent north-south allocation, flow-constrained cut-to-fill pairing, arbitrary geometry-derived local fill headings, eight exact sequencing finalists, a twelve-cell gap bound, and the existing `2.0` work-inflation bound.

**[0.2.18]** `LaneAxisHeadingDeg` denotes an undirected work-lane axis rather than a commanded travel bearing. Inputs normalize to `[0, 180)`, so `0` and `180` produce the same north-south lane cover and `90` and `270` produce the same east-west cover; production guidance remains responsible for choosing either bearing independently for every cut, fill, and connector. Arbitrary axes are supported on rectangular fields. The default value is `0`, the measured north-south axis. The exact `0`-degree fixture reproduces [0.2.16] at `240.440 h` deterministic, `300.550 h` scheduled, `2,296,941.9 m`, `3,182` routes, and zero sequence failures. A `37.5`-degree real-field run also validates with zero allocation or ledger violations, demonstrating that the axis is no longer cardinal-only; it is not promoted as the optimum for this fixture. Focused lane-ledger and optimizer tests pass `15/15`; the complete suite reports `118` passed and the same `11` known legacy/parity/guidance-contract failures.

**[0.2.19]** Compact cut layouts now receive bounded flexible-endpoint sequencing before guidance abandons them for a fragmented alternate heading. Only internal endpoint distance is eligible: it is performed blade-down on the adjoining validated connector, while any recovered first-run start or last-run end shortfall rejects the candidate transactionally. The marked route `2709` consequently retains its `0`-degree, four-run cut instead of the former `140`/`320`-degree, fifteen-run fallback. The complete exact plan improves to `235.591 h` deterministic, `294.489 h` scheduled, `2,259,747.5 m`, `55,297` lifecycle segments, and `175` manual assists with zero sequence failures and the unchanged `3,182` routes and deferred ledger. This saves `4.849 h`, `37,194.4 m`, `1,616` lifecycle segments, and `194` manual assists relative to [0.2.16]. Focused coverage, sequencing, and lifecycle tests pass `28/28`; the complete suite reports `119` passed and the same `11` known legacy/parity/guidance-contract failures. A separate experiment that forced all guidance candidates into the requested-axis family is rejected because residual edge route `3044` requires an edge-tangent exception and produced one sequence failure.

**[0.2.20]** The optional multi-route exchange pools one work kind across a bounded local route group, then deterministically transports the exact pooled cell depths back into each route's unchanged work-volume quota. This generalizes isolated-fragment repair without a capacity ripple: every committed group preserves per-route cut/fill balance, scraper load, maximum cell depth, and the global cut/fill cell-depth multisets. A group is transactional and is rejected unless its geometry proxy improves, all involved routes pass compact production guidance before and after, total cut/fill run count does not increase, and compact-guidance distance improves. With two rounds, at most eight routes per group, and at most 100 committed groups, the exact field run commits eight closed groups across 63 routes and improves the complete plan to `235.359 h` deterministic, `294.199 h` scheduled, `2,257,295.6 m`, `55,223` lifecycle segments, and `169` manual assists. It retains `3,182` routes, the same `185.778 BCY` cut and `65.549 BCY` fill deferred ledger, and zero sequence failures. This saves `0.232 h`, `2,451.9 m`, 74 lifecycle segments, and six manual assists relative to [0.2.19]. The ungated geometry-only experiment is rejected at `242.650 h`; a twelve-route group bound is also rejected after its proxy regresses to `256.011 h`. The pass remains default-off because it adds approximately three minutes of search to this fixture and the fixed-work release target remains unmet. The reproducible exact flag is `--multi-route-exchange` appended to [0.2.15]. Focused exchange tests pass `3/3`; the complete suite reports `122` passed and the same `11` known legacy/parity/guidance-contract failures.

**[0.2.21]** The promoted pragmatic profile packs at most ten same-kind lane slices, completes the residual edge ledger with a twelve-slice bound, and then runs capacity-flexible route consolidation. Consolidation pools both cut and fill ledgers across at most eight nearby routes, removes an underfilled cycle only when the remaining loads fit scraper capacity, rebuilds identical cut/fill BCY targets, preserves every cell depth exactly, and accepts the transaction only when all replacement cycles pass compact production guidance without increasing run count or compact-guidance distance. The exact `TheShop2_2ft.agd` run removes five cycles in `1,245` attempted groups and reduces accepted compact-guidance distance from `8,793.5 m` to `7,364.4 m`. The final plan contains `3,114` guided routes, `2,232,538.5 m`, `55,221` lifecycle segments, `155` manual assists, and zero guidance sequence failures. It completes in `233.725 h` deterministic and `292.157 h` scheduled, a `46.082%` deterministic reduction from `433.483 h`. Relative to the otherwise identical ten-slice checkpoint without consolidation (`234.248 h`, `166` assists), consolidation saves `0.523 h` and eleven assists, so it is enabled by default.

**[0.2.22]** The automatic plan explicitly leaves `55.715 BCY` of balanced work for manual cleanup, equal to `0.1194%` of required fill and below the accepted `65.549 BCY` bound. The separately reported deferred cut is `175.944 BCY`: `55.715 BCY` is the matching cleanup cut and `120.229 BCY` is immutable field cut surplus, not missing fill work. `HaulBuilderResult.DeferredWorkSummary`, `HaulBuilderMetrics.ManualCleanupBcy`, `DeferredCutBcy`, and `DeferredFillBcy`, plus the run messages, expose these values to callers. Deferred entries remain exact cell/layer/depth records; neither the target surface nor the cut/fill ratio changes. The reproducible benchmark adds `--consolidate-routes` to [0.2.15] and uses `northsouth 10`.

**[0.2.23]** A field-wide audit confirms that route count is already close to saturated: `46,588.037 BCY` assigned at `15.3846 BCY` bank capacity has a `3,029`-route capacity lower bound versus `3,114` actual routes, only `85` routes or `2.806%` above the bound, with `97.245%` average utilization. The remaining theoretical opportunity is therefore allocation and routing geometry, not further load consolidation. Relaxing unique fill ownership, exact guidance feasibility, and open-route subtour constraints produces optimistic lower bounds of `53,879.0 s` for next-cut transitions versus `102,109.4 s` current, and `359,134.7 m` joint-equivalent geometry versus `1,016,025.3 m` current. These deliberately non-unique bounds identify search opportunity; they are not achievable-plan estimates and cannot promote a candidate.

**[0.2.24]** The optional field-wide cycle-chain pass holds every cut ledger and route load fixed, considers whole-fill exchanges only between exactly equal-load cycles, and couples each proposed cut-to-fill ownership with its following next-cut transition. Every committed exchange must preserve the exact global cut and fill cell-depth multisets, pass compact production guidance for both changed routes, improve the production-time-plus-transition objective by at least `0.25 s`, and be Pareto non-increasing in both base production time and compact-guidance distance. One round with one exact candidate per visited route accepts `108` exchanges, improves the proxy from `254.010 h` to `253.914 h`, reduces compact-guidance distance by `1,319.0 m`, and reduces the rerouted transition model by `221.7 s`.

**[0.2.25]** Full production guidance validates the [0.2.24] plan at `233.546 h` deterministic, `291.933 h` scheduled, `2,230,431.2 m`, `55,218` lifecycle segments, `157` manual assists, and zero sequence failures across all `3,114` routes. Relative to [0.2.21], this saves `0.179 h` deterministic, `0.224 h` scheduled, and `2,107.3 m` without changing route count, deferred work, target surface, cut/fill ratio, or any guidance rule. An earlier time-surrogate-only candidate accepted `337` exchanges but regressed exact guidance to `233.850 h`; it is rejected evidence. Production integration therefore evaluates both the baseline and provisional complete plans through `GuidanceCostOracle` and transactionally rolls back unless candidate exact deterministic time strictly improves. Production enables the one-round, one-candidate profile because exact evaluation promotes it only when it strictly improves the complete plan. Focused audit, conservation, Pareto, and default-option tests pass `3/3`; the complete suite reports `128` passed and the same `11` known legacy parity, lineup, guidance-contract, and PNG-contract failures. The reproducible benchmark adds `--joint-chain --joint-chain-rounds=1 --joint-chain-candidates=1 --exact` to the command in [0.2.22].

**[0.2.26]** Expanding the exact shortlist to two fill candidates per visited route accepts `188` Pareto-safe exchanges and improves every screen relative to [0.2.24], but exact guidance regresses relative to the one-candidate profile: `233.662 h` deterministic, `292.077 h` scheduled, `2,232,341.7 m`, `55,227` segments, and `159` manual assists. It remains `0.063 h` better than [0.2.21] and has zero sequence failures, but is rejected because it is `0.116 h` slower than [0.2.25]. This establishes one candidate per route as the retained bounded profile and demonstrates that the screens are useful search filters, not promotion authority.

**[0.2.27]** The [0.2.25] plan automatically moves `46,588.037 BCY` across `37.0820 acres`, equal to `1,256.35 BCY/acre`. Its exact productivity is `199.48 BCY` per deterministic guidance hour and `159.59 BCY` per scheduled hour. The normalized cross-field golden target is therefore `200 BCY` per deterministic production-guidance hour. Scheduled productivity remains separately reported and shall not be used to judge algorithm geometry because it includes configurable operational availability. The automatic volume numerator excludes explicitly deferred manual cleanup; required, automatic, and deferred volumes remain separately visible.

**[0.2.28]** Fill-bearing selection now models the mandatory reverse grading pass explicitly. Because grading traverses the fill in reverse, the machine finishes at the same physical end where filling began, with the opposite heading. A grade-return route therefore evaluates each guidance-feasible fill orientation against the complete local chain: cut exit to fill entry, fill coverage and internal connectors, fill-to-grade turn, reverse grade coverage, and the actual post-grade pose to the next route's cut. Manual-assist count is the primary comparison and exact centerline distance is secondary. Reversing a candidate preserves every allocation, depth, load, cut/fill balance, target elevation, and run centerline; candidates that violate any existing guidance rule are rejected rather than forced.

**[0.2.29]** On the canonical [0.2.25] field plan, grade-return-aware selection validates all `3,114` routes with zero guidance sequence failures and improves the exact result from `233.546 h` deterministic, `291.933 h` scheduled, `2,230,431.2 m`, `55,218` lifecycle segments, and `157` manual assists to `222.067 h` deterministic, `277.584 h` scheduled, `2,070,325.9 m`, `55,186` lifecycle segments, and `153` manual assists. This saves `11.479 h` deterministic, `14.349 h` scheduled, `160,105.3 m`, 32 lifecycle segments, and four manual assists without changing the `3,114` routes or the `175.944 BCY` cut and `55.715 BCY` fill deferred ledger. Productivity increases to `209.79 BCY` per deterministic guidance hour and `167.83 BCY` per scheduled hour, equal to `104.90%` of the `200 BCY/h` golden target. The highlighted route `491` now fills south-to-north (`0`-degree bearing), grades north-to-south, and exits from the southern fill end as intended. A stronger experiment that preserved literal whole-sequence reversals before inbound resequencing remained exact-valid but regressed to `223.462 h`, `2,087,575.4 m`, and `155` manual assists; it is rejected in favor of this checkpoint. Focused grade-direction and animation-timing tests pass `4/4`.

**[0.1.2]** Revisions follow semantic versioning: **MAJOR** denotes an incompatible normative change, **MINOR** a backward-compatible addition, and **PATCH** a clarification that does not change an obligation.

---

## 1. Purpose and measured problem

**[1.0.1]** The former allocation and timing estimate reported approximately `205 h`, but the production forward-only guidance exposes `433.483 h` of deterministic driving and `541.854 h` at `80%` operational availability for the same field. The increase is predominantly real path geometry rather than a timing-accounting defect.

**[1.0.2]** The promoted `TheShop2_2ft.agd` plan contains `3,152` haul cycles and `46,535` BCY. The cycle count is close to the capacity lower bound and therefore shall not be treated as the primary optimization variable.

**[1.0.3]** The plan contains `25,931` cut and fill coverage runs, averaging approximately `8.23` work runs per haul cycle. `2,937` cycles contain multiple cut runs and `2,642` cycles contain multiple fill runs.

**[1.0.4]** Only `71.579 h` is blade-down cut, fill, and grade work. Lineups, exits, and fill-to-grade turns consume `123.681 h`; transit, next-cut transit, and manual assistance consume `238.223 h`. The optimization shall therefore target run fragmentation, intra-cycle cut-to-fill travel, mandatory turnaround geometry, and cycle-to-cycle travel rather than merely minimizing straight-line haul distance.

**[1.0.5]** The existing algorithm selects the nearest cut anchor, follows the flow direction to grow one cut patch, selects a fill seed, and then grows a fill patch. It does not allocate material by minimizing the complete machine cycle generated later by guidance.

**[1.0.6]** The existing flow field matches the desired commercial transport output and shall not be replaced or tuned to manufacture a faster result. Its potential, direction, and magnitude shall instead constrain and inform the new material assignment.

**[1.0.7]** The purpose of this specification is to produce a one-shot, whole-field allocation improvement large enough to materially reduce the completion time while preserving every fixed physical and guidance rule.

## 2. Non-negotiable outcomes

**[2.0.1]** A conforming implementation shall satisfy all of the following outcomes.

**[2.0.2]** The AGD target elevation at every cell shall remain bitwise unchanged from the grid supplied to the optimizer.

**[2.0.3]** `CutFillRatio`, compaction, swell, grid size, blade width, maximum per-pass depth, scraper capacity, tolerance, and every equipment and speed setting shall remain unchanged during a baseline-versus-candidate comparison.

**[2.0.4]** For every physical cell, work kind, and depth layer, `AssignedDepth + DeferredDepth + ToleranceRemovedDepth` shall equal `RequiredDepth` within `1e-9 m`.

**[2.0.5]** For the fixed automatic-work set used in a performance comparison, assigned cut BCY shall equal assigned fill-required BCY within `1e-8 BCY` and each haul cycle shall be balanced within the same tolerance.

**[2.0.6]** No cycle shall exceed bank-volume capacity, exceed the maximum depth removed or placed at one cell in one cycle, cut or fill the same physical cell twice in one cycle, invent work in an at-grade cell, or move work to another coordinate.

**[2.0.7]** Every accepted plan shall use the unmodified production guidance lifecycle and production `GuidanceTiming.Estimate` result as its final feasibility and time authority.

**[2.0.8]** Every accepted automatic segment shall remain forward-only, continuous, radius-valid, and field-contained. Manual travel is permitted only under the existing one-grid-cell exterior allowance and shall be explicitly reported.

**[2.0.9]** Grade return shall remain enabled for the primary acceptance fixture. The optimizer shall not improve time by disabling, batching, shortening, or omitting the mandatory grade-return lifecycle defined by guidance.

**[2.0.10]** The optimizer shall preserve the promoted flow-field values within numerical parity tolerance. It may consume the flow field but shall not change `FlowFieldSolver` as part of haul optimization.

**[2.0.11]** Difficult work may be deferred for separate analysis only under section 12. Deferral shall never be described as completed work, shall never alter the requested final surface, and shall never be allowed to create a misleading time improvement.

## 3. Design goals and non-goals

### 3.1 Design goals

**[3.1.1]** The optimizer shall construct contiguous, guidance-aware work lanes before it partitions those lanes into scraper loads.

**[3.1.2]** The optimizer shall use the validated Poisson flow potential, direction, and magnitude to limit plausible cut-to-fill assignments while retaining a documented fallback for material that cannot be assigned through the preferred corridor.

**[3.1.3]** The primary construction objective shall be seconds of complete machine operation, including coverage, lineups, exits, loaded transit, fill-to-grade turn, grade return, unloaded transit, curvature limits, acceleration, and braking.

**[3.1.4]** The implementation shall prefer one continuous cut run and one continuous fill run per load. Additional runs shall be admitted only when they reduce exact or calibrated operating time and remain within configured fragmentation limits.

**[3.1.5]** The optimizer shall create a portfolio of deterministic complete plans in one invocation, screen them cheaply, exactly evaluate the best finalists, and retain the best fully validated plan.

**[3.1.6]** Bounded local repair shall operate on expensive cycles and neighboring material only after the global allocation and sequencing stages have completed.

**[3.1.7]** The benchmark harness shall allow an implementation agent to change optimizer profiles, rerun the complete experiment, compare against the best checkpoint, and continue until the performance gate is achieved or a documented lower bound demonstrates why it cannot be achieved.

### 3.2 Non-goals

**[3.2.1]** This work shall not change the final landform, rebalance by changing target elevation, change the cut/fill ratio, or change the equipment model.

**[3.2.2]** This work shall not simplify or repair final guidance coordinates after validation.

**[3.2.3]** This work shall not permit reversing, unrestricted off-field travel, zero-radius turns, instantaneous heading reversals, or generic unvalidated manual segments.

**[3.2.4]** This work shall not optimize the timing constants or operational-availability factor to obtain a lower headline value.

**[3.2.5]** This work shall not replace the commercial-matching flow field with a visually convenient field or claim that the Poisson flow alone is a load allocation.

**[3.2.6]** This work shall not require exact Python parity for optimized allocations. Python parity remains applicable to the explicitly selected legacy allocation mode.

**[3.2.7]** This work shall not accept a plan merely because it reduces pass count, straight-line distance, proxy score, work-run count, or a rendered loop count.

## 4. Current-state findings and required corrections

### 4.1 Allocation findings

**[4.1.1]** `Passes.GeneratePasses` is a greedy depletion algorithm. It commits cut work before the complete fill and guidance consequences of that choice are known.

**[4.1.2]** Flow-aligned growth consumes one maximum-depth bite per visited cell per phase, follows a single upstream or downstream streamline, prunes separated fragments, and may hop to another pocket to reach the configured load fraction. This preserves volume but commonly creates several later coverage runs within one load.

**[4.1.3]** Fill selection uses `Strips.StreamlineTo` or a nearest-fill heuristic. The flow magnitude and network flux are not used to reserve destination volume or jointly optimize competing cut sources.

**[4.1.4]** `CoveragePlanner` chooses straight coverage geometry only after cell ownership is fixed. Its current million-, hundred-, and trillion-scale weights are useful feasibility priorities but are not physical time and shall not be used as the optimization objective.

**[4.1.5]** `Routing.SequencePasses` can only order fixed cycles. It cannot correct a poor cut-to-fill pairing or fragmented work ownership. Its direction-aware route starts from original pass `0`, and large direction-aware plans do not use the symmetric two-opt path.

### 4.2 Guidance and timing findings

**[4.2.1]** `GuidancePlanner.Build` now constructs production coverage, forward-only connectors, lifecycle phases, and final-centerline timing. This implementation shall be reused, not copied into the optimizer.

**[4.2.2]** The current exact builder is whole-plan oriented and exposes no small, side-effect-free cost oracle for lane, cycle, transition, or candidate-plan evaluation. Section 9 defines the required adapter.

**[4.2.3]** `HaulBuilderEngine.BuildGuidance` currently runs only when a guidance artifact or pair sample is requested. Optimized production metrics shall always build lightweight guidance and exact timing; CSV and PNG emission shall remain optional.

**[4.2.4]** The promoted report already contains `180` unique guidance-deferred cells and `1.97 BCY` of separately omitted edge cut. These quantities shall be converted into explicit ledgers before candidate comparisons are allowed.

### 4.3 Test findings

**[4.3.1]** At the start of this revision, the Release test run reports `83` passed and `10` failed tests. Failures include lineup, PNG/Python parity, TheShop2 pass and route parity, and a guidance conservation expectation.

**[4.3.2]** Work unit 0 shall classify each existing failure as a production defect, stale legacy parity expectation, nondeterministic test, or obsolete test contract. No failure may be silently skipped, weakened, or attributed to the new optimizer without evidence.

**[4.3.3]** The optimizer shall not be enabled in production until the required test matrix in section 14 is green.

## 5. Terminology and data model

### 5.1 Required work ledger

**[5.1.1]** The **required work ledger** is the canonical per-cell cut or fill depth derived from `Grid.Existing - Grid.Target`, the unchanged cut/fill ratio, and tolerance. It is the authority for final-surface conservation.

### 5.2 Depth layer

**[5.2.1]** A **depth layer** is a cell's required work divided into ordered slices no deeper than `MaxDepthMm`. A partial final layer is permitted. A layer may be owned by only one cycle except for an explicitly recorded boundary split required for exact load balance.

### 5.3 Work lane

**[5.3.1]** A **work lane** is a deterministic, connected, blade-width-compatible ordered sequence of same-kind layer entries that can be served as one guidance coverage run or as bounded collinear sub-runs joined by an allowed short continuation.

### 5.4 Lane slice

**[5.4.1]** A **lane slice** is a contiguous interval of a work lane with exact cell-depth ownership and a bank volume no greater than one scraper load.

### 5.5 Flow corridor

**[5.5.1]** A **flow corridor** is a directed set of valid cells or lane nodes reachable generally down the Poisson potential from cut toward fill, widened by a configured cross-flow tolerance to admit practical vehicle geometry.

### 5.6 Haul cycle

**[5.6.1]** A **haul cycle** is one capacity-bounded, volume-balanced collection of cut lane slices and fill lane slices together with their selected orientations. It becomes one `Pass` only after its allocation is finalized.

### 5.7 Candidate plan

**[5.7.1]** A **candidate plan** is a complete immutable set of haul cycles, a cycle order, exact ledgers, deferred-work state, optimizer profile, and deterministic fingerprint.

### 5.8 Fixed-work comparison

**[5.8.1]** A **fixed-work comparison** evaluates baseline and candidate against the same required and deferred cell-depth multisets. It prevents an apparent time saving caused by completing less work.

### 5.9 Exact evaluation

**[5.9.1]** **Exact evaluation** means using the production `CoveragePlanner`, `RunSequencer`, `TransitPlanner`, `LifecyclePlanner`, and `GuidanceTiming` behavior that would be used for exported guidance. It is exact with respect to the implemented centerline model.

## 6. Target architecture and pipeline

### 6.1 Production pipeline

**[6.1.1]** The optimized production pipeline shall execute in the following order.

**[6.1.2]**

```text
Agd.Parse
GridMath.BuildGrid
Balance.Compute
FlowFieldSolver.Compute                       # unchanged commercial-matching field
HaulWorkLedger.Build
HaulLaneBuilder.BuildCutAndFillLanes
HaulTransportGraph.Build
HaulCycleOptimizer.GenerateCompletePlans
  BuildFlowConstrainedPairings
  PackCapacityBoundedCycles
  SelectCycleOrientations
  SequenceCycles
  ScreenCompletePlans
  EvaluateExactFinalists
HaulRefinement.RepairExpensiveNeighborhoods
HaulPlanValidator.ValidateAndSelectBest
GuidancePlanner.Build                         # production implementation
GuidanceTiming.Estimate                       # production implementation
Reporting and optional export
```

**[6.1.3]** `Passes.GeneratePasses` shall remain callable as `LegacyGreedy` mode and shall provide the frozen baseline plan. It shall not be the provisional input to the new global optimizer.

**[6.1.4]** `HaulCycleOptimizer` shall return immutable candidate allocations. Production `Pass`, `Route`, and result objects shall be created only for a candidate undergoing exact evaluation or final commit.

**[6.1.5]** `HaulRefinement` shall be a bounded repair layer over the global candidate. It shall not be the primary constructor and shall not be restricted to original legacy-pass ownership.

### 6.2 Separation of responsibility

**[6.2.1]** The work ledger shall own required, assigned, deferred, and tolerance-removed depth and volume.

**[6.2.2]** The lane builder shall own serviceable work topology and candidate coverage orientations. It shall not pair cut with fill.

**[6.2.3]** The transport graph shall own flow-compatible source-to-destination alternatives. It shall not create final guidance geometry.

**[6.2.4]** The cycle optimizer shall own material pairing, load packing, orientation choice, and route order.

**[6.2.5]** Guidance shall remain the sole owner of final coverage geometry, lineups, connectors, containment classification, manual allowance, turn feasibility, lifecycle phases, and final timing.

**[6.2.6]** The validator shall own cross-stage invariants and shall reject rather than repair any target, ratio, volume, capacity, depth, guidance, or timing violation.

## 7. Lane-first work construction

### 7.1 Canonical layering

**[7.1.1]** The lane builder shall split every required cell into deterministic depth layers ordered from first service to final service. The sum of layer depths shall reproduce the required cell depth within `1e-12 m` before tolerance handling.

**[7.1.2]** Layer identity shall contain work kind, cell coordinate, zero-based layer index, depth, volume, and a checksum of the unchanged grid and material settings.

**[7.1.3]** Input enumeration order, dictionary order, thread completion order, and locale shall not change layer identity or lane membership.

### 7.2 Candidate lane formation

**[7.2.1]** Cut and fill lanes shall be built independently from the complete same-kind layer inventory before any load owns a cell.

**[7.2.2]** Candidate headings shall include the local flow heading, deterministic offsets around that heading, compatible field-boundary headings, and any heading already proven feasible by `CoveragePlanner` for the same cells.

**[7.2.3]** Lane growth shall prefer monotonic progress along the local flow direction, blade-width lateral containment, continuous work cells, low heading variation, and endpoints with adequate lineup and turn space.

**[7.2.4]** A lane shall break at an invalid cell, obstacle, zero-work gap beyond the allowed continuation, layer boundary conflict, field-edge infeasibility, or predicted coverage discontinuity.

**[7.2.5]** Lane formation shall not require every cell in a connected component to share one heading. Curved flow shall be represented by several serviceable lane sections rather than by one undriveable curved ownership chain.

**[7.2.6]** Each layer entry shall appear in at least one lane alternative. The selected lane cover shall own every non-deferred entry exactly once.

### 7.3 Lane slicing

**[7.3.1]** Selected lanes shall be sliced only at cell-depth boundaries, capacity boundaries, feasibility boundaries, or an exact final balance split.

**[7.3.2]** The optimizer shall prefer whole layer entries. When a boundary entry must be split, both pieces shall retain the same cell, layer identity, and combined exact depth; the split shall be reported.

**[7.3.3]** No cycle shall contain two pieces of the same cell and work kind. A split cell shall therefore be serviced by different cycles.

**[7.3.4]** The lane inventory shall report total lanes, lane lengths, volumes, endpoint poses, boundary clearance, predicted run count, split entries, and unlaneable entries.

### 7.4 Fill redistribution envelopes

**[7.4.1]** Fill guidance may join close-proximity allocated fill fragments with a continuous blade-down envelope when the mandatory reverse grading pass can redistribute temporary overplacement to the unchanged requested final surface. This rule applies to fill only; cut guidance shall not remove material from intervening non-cut cells.

**[7.4.2]** Envelope headings shall be derived from the allocation geometry rather than limited to north-south or east-west. Deterministic candidates shall include established run headings, the allocation's principal axes, dominant axial headings from bounded nearby-cell links, and locally required field-edge tangents.

**[7.4.3]** Disconnected proximity components shall be evaluated independently. A dense component may select its own heading while isolated or difficult cells remain separate; a distant cell shall not force a poor common heading on neighboring contiguous work.

**[7.4.4]** An envelope shall retain the exact input `(cell, depth)` multiset. It shall not change ownership, depth, volume, cycle balance, cut/fill ratio, target elevation, or deferred work. Additional centerline distance represents temporary placement and grading redistribution, not a ledger reassignment.

**[7.4.5]** The default envelope shall bound along-axis gaps to `12` grid cells, maximum blade-down work inflation to `2.0` times the replaced fragments, and assigned-cell lateral error to one-half grid cell. These are candidate bounds, not permissions to violate field containment, edge-parallel work, blade width, or any guidance rule.

**[7.4.6]** A candidate shall reduce run count, receive forward-only sequencing with loaded-overlap avoidance, conserve allocation bit-for-bit, and prove the complete fill-to-grade turnaround plus reverse connector sequence before acceptance. An infeasible candidate shall be rejected transactionally and the prior sequence retained.

**[7.4.7]** Feasible candidates shall be ranked by an estimate covering both fill and reverse-grade work, inter-run movement, lineups, and a dominant manual-assist penalty. Fewest runs alone shall not select an excessively long or operator-dependent envelope.

**[7.4.8]** Unit tests shall cover a close comb, a non-cardinal diagonal cluster, a dense component with a distant outlier, a sparse rejection, exact cell/depth conservation, and mandatory grade-return feasibility. The exact whole-field benchmark remains the promotion authority.

## 8. Flow-constrained global pairing and cycle packing

### 8.1 Transport graph

**[8.1.1]** The transport graph shall use the existing Poisson potential, unit direction, and magnitude without modifying them.

**[8.1.2]** Directed corridor edges shall prefer decreasing potential from cut toward fill. Cross-flow and short uphill connector edges may be admitted only within configured bounds and shall receive explicit time cost and diagnostic classification.

**[8.1.3]** Every cut lane slice shall receive a deterministic shortlist of compatible fill lane slices based on corridor reachability, capacity availability, approximate loaded travel time, endpoint orientation, and field feasibility.

**[8.1.4]** A fallback edge may connect otherwise stranded balanced work. Fallback use shall receive a calibrated operating-time or risk cost, shall be bounded by profile policy, and shall be listed by cell and reason; flow preference shall not conceal a slower complete machine plan.

**[8.1.5]** Flow magnitude shall be used as a corridor-capacity or preference signal so multiple cut sources do not independently claim the same attractive downstream fill without accounting for destination demand.

### 8.2 Global material assignment

**[8.2.1]** Material pairing shall solve a balanced transportation problem over lane-slice supply and demand, not a sequence of irrevocable nearest-neighbor choices.

**[8.2.2]** The solver may use deterministic successive-shortest augmenting paths, min-cost flow, column generation, or a bounded equivalent, provided it preserves continuous BCY quantities and passes the same tests.

**[8.2.3]** Candidate edge cost shall be expressed in seconds and shall include a lower bound for cut service, cut exit, loaded connector, fill lineup and service, fill exit, grade turn, grade service, and expected next-cut positioning.

**[8.2.4]** Distance, flow deviation, and fragmentation may be reported separately but shall enter selection only through calibrated seconds or an explicit lexicographic feasibility rule.

### 8.3 Cycle packing

**[8.3.1]** The transport assignment shall be packed into cycles no larger than `ScraperCapacityLcy / SwellFactor` BCY.

**[8.3.2]** A cycle shall contain one primary cut lane slice and one primary fill lane slice whenever volume and feasibility permit.

**[8.3.3]** A secondary same-kind slice may be appended only when it is a direct continuation or when the candidate cost proves that the extra run is faster than leaving a low-load cycle. The default maximum shall be two cut runs and two fill runs per cycle.

**[8.3.4]** A cycle below `MinimumOptimizedLoadPercent` may remain only for the global tail, a disconnected material component, a capacity-balancing boundary, or proven guidance infeasibility of fuller alternatives. Its reason shall be reported.

**[8.3.5]** Cut and fill cycle volumes shall match within `1e-8 BCY`. Proportional reduction of unrelated fill entries is forbidden.

### 8.4 Orientation and cycle sequencing

**[8.4.1]** Each lane slice shall expose both service orientations when both are coverage-feasible. Orientation shall be selected jointly with cut-to-fill pairing rather than flipped later by a geometric nearest-end heuristic.

**[8.4.2]** Cycle sequencing shall price the exact or calibrated transition from mandatory grade exit to the next cut lineup.

**[8.4.3]** The route constructor shall evaluate multiple deterministic start cycles and shall not force original cycle `0` to be first.

**[8.4.4]** Direction-safe relocate, swap, and bounded chain moves shall be permitted. A move shall be accepted only when its exact or calibrated transition objective decreases and the final whole-plan exact result also improves.

## 9. Guidance cost oracle and candidate evaluation

### 9.1 Required API behavior

**[9.1.1]** The implementation shall expose a side-effect-free `GuidanceCostOracle` adapter over the production guidance components. It shall not write PNG, GIF, CSV, progress, or production result files unless diagnostics are explicitly enabled.

**[9.1.2]** The oracle shall accept immutable lane, cycle, transition, or complete-plan inputs plus the complete guidance and timing options checksum.

**[9.1.3]** The oracle shall return feasibility; deterministic seconds; scheduled seconds; distance by phase and payload; work, lineup, exit, transit, grade-turn, grade, and manual seconds; coverage-run count; turn metrics; field-exit metrics; deferred entries; and structured failure codes.

**[9.1.4]** Repeated evaluation of identical canonical input shall return identical discrete decisions and numerically stable metrics.

### 9.2 Evaluation tiers

**[9.2.1]** `LaneLowerBound` shall estimate service time and endpoint feasibility without building full connectors.

**[9.2.2]** `CycleEstimate` shall use production coverage candidates plus cached connector lower bounds to price a complete cut-fill-grade cycle in seconds.

**[9.2.3]** `CycleExact` shall build the production intra-cycle coverage and lifecycle geometry for finalist cycles, excluding only the unknown preceding and following cycle transitions.

**[9.2.4]** `TransitionExact` shall evaluate a production-valid grade-exit-to-next-cut connector for fixed endpoint poses.

**[9.2.5]** `PlanExact` shall invoke the same whole-plan production guidance and `GuidanceTiming.Estimate` path used for the committed result. Only `PlanExact` may authorize plan promotion.

### 9.3 Caching and incremental evaluation

**[9.3.1]** Lane, cycle, transition, and plan results shall be cached by canonical allocation fingerprint, ordered endpoint poses, field checksum, guidance options checksum, timing options checksum, and oracle version.

**[9.3.2]** Local repair shall invalidate only changed cycles, their coverage plans, and adjacent route transitions. Unchanged exact results shall be reused.

**[9.3.3]** Cache hits shall never bypass final conservation or whole-plan validation.

### 9.4 Complete-plan portfolio

**[9.4.1]** One optimizer invocation shall generate a deterministic portfolio including at least a flow-strict profile, run-minimizing profile, transit-minimizing profile, balanced-time profile, and legacy baseline.

**[9.4.2]** All complete plans shall pass ledger and capacity validation before screening.

**[9.4.3]** The best `MaxExactCompletePlans` feasible screened plans, plus the current best checkpoint and legacy baseline, shall receive `PlanExact` evaluation.

**[9.4.4]** A proxy result shall never promote or commit a plan.

## 10. Objective, selection, and significant-improvement gate

### 10.1 Lexicographic feasibility

**[10.1.1]** Candidate comparison shall first reject target, ratio, ledger, balance, capacity, depth, duplicate-cell, guidance, radius, containment, continuity, or fixed-work violations.

**[10.1.2]** Among feasible candidates, comparison shall minimize newly deferred work before comparing time. Flow fallback and permitted manual travel shall be bounded, included in exact time, and reported; within those bounds they shall not override a materially faster fixed-work plan.

### 10.2 Primary time objective

**[10.2.1]** The primary numeric objective shall be `GuidanceTiming.Result.TotalSeconds` under unchanged operational availability and timing options.

**[10.2.2]** Deterministic driving seconds, operational-availability allowance, and completion-equivalent deferred-work bounds shall also be reported separately so one number cannot conceal the source of improvement.

**[10.2.3]** Ties within `0.1 s` shall prefer less manual time, shorter total distance, fewer coverage runs, fewer split entries, and finally the lexicographically smaller candidate fingerprint.

### 10.3 TheShop2 release gate

**[10.3.1]** The immutable baseline shall be the full promoted report associated with AGD SHA-256 `78B1599FEB6E63FE4EA51D4D65E14EC3E08BB08E24FF09E394EA1C52A41A764F` and source commit `68f61b7`, after work unit 0 reconciles the exact options and deferred-work ledger.

**[10.3.2]** The recorded baseline values are `433.483` deterministic driving hours, `541.854` scheduled hours at `80%` availability, `3,667,783.146 m` total centerline, `3,152` cycles, and `25,931` cut/fill coverage runs.

**[10.3.3]** The cross-field release gate is at least `200 BCY` moved per deterministic production-guidance hour, using automatic moved BCY as the numerator and exact final-centerline hours as the denominator. Deterministic and scheduled hours, deferred work, and fixture-relative savings remain separately reported; scheduled availability shall not alter the geometry gate.

**[10.3.4]** The release gate shall also require a reduction in total centerline distance and total cut/fill coverage-run count. It shall not prescribe their exact percentages because a shorter valid route may contain necessary curvature and lineup distance.

**[10.3.5]** A candidate that improves by less than `50%` may be retained as an experimental checkpoint but shall not enable the optimizer by default or be described as satisfying this specification.

**[10.3.6]** If the `200 BCY/h` gate cannot be met, implementation shall continue through the iteration workflow in section 13 until either it is met or a reproducible bound and hotspot decomposition show that the remaining target is incompatible with the fixed guidance and equipment rules. That evidence shall quantify irreducible blade-down work, minimum per-cycle lineup and grade-return geometry, loaded transport, unloaded transport, curvature, acceleration, and deferred-work completion. It requires user review and is not an automatic waiver.

### 10.4 Cross-field golden productivity target

**[10.4.1]** Future fields shall be evaluated against a golden target of `200 BCY` automatically moved per exact deterministic production-guidance hour. The numerator is the sum of final automatic cycle loads in bank cubic yards; the denominator is `GuidanceTiming.Result.MotionSeconds + EventSeconds`. Field-efficiency allowance, scheduled downtime, and manual-cleanup volume are excluded from this normalized algorithm KPI.

**[10.4.2]** Every result shall separately report automatic `BCY/acre`, deterministic `BCY/hour`, scheduled `BCY/hour`, achieved percentage of the `200 BCY/hour` target, and whether the target was met. `TotalHours` and the backward-compatible `MoveRateBcyPerHour` remain scheduled metrics.

**[10.4.3]** Falling below `200 BCY/hour` shall trigger hotspot and opportunity reporting rather than relaxing guidance, changing the target surface or cut/fill ratio, hiding deferred work, or changing equipment assumptions. Field geometry may make the target unattainable; any exception requires exact lower-bound evidence rather than a proxy-only conclusion.

## 11. Global repair after construction

### 11.1 Hotspot inventory

**[11.1.1]** After exact evaluation, the optimizer shall rank cycles and transitions by avoidable seconds, seconds per BCY, run count, manual distance, fallback-flow volume, and excess over the best available lower bound.

**[11.1.2]** Hotspots shall be selected by measured time contribution, not by route number or rendered appearance.

### 11.2 Repair neighborhoods

**[11.2.1]** A repair neighborhood shall include the hotspot cycle, its predecessor and successor, cycles sharing its lanes or cells, and flow-compatible alternate destinations within the configured corridor expansion.

**[11.2.2]** Candidate repair operations shall include lane-boundary shift, cut-slice swap, fill-slice swap, whole-lane reassignment, orientation change, cycle merge/split, split-depth coalescing, and local route reorder.

**[11.2.3]** Every repair shall operate on immutable snapshots and produce a complete before-and-after ledger.

**[11.2.4]** A repair may pass through calibrated screening but shall be committed only after affected exact cycle and transition evaluation. The final repaired plan shall receive `PlanExact` before promotion.

**[11.2.5]** Repair shall stop when no exact improvement remains, configured iteration or runtime limits are reached, or the significant-improvement gate passes and no candidate in the current portfolio improves by `MinimumExactSavingsSeconds`.

## 12. Difficult and deferred work

### 12.1 Required semantics

**[12.1.1]** Deferred work is required work not assigned to the automatic plan. Deferral does not alter `Grid.Target`, `CutFillRatio`, required depth, or final-surface intent.

**[12.1.2]** Each deferred entry shall retain work kind, cell coordinate, exact depth, BCY, layer identity, reason, failed candidate fingerprints, and guidance failure evidence.

**[12.1.3]** The required ledger shall reconcile `Automatic + Deferred + ToleranceRemoved` per cell. A report that lacks this reconciliation shall be rejected.

### 12.2 Default policy

**[12.2.1]** New optimizer-created deferral shall default to disabled for TheShop2 acceptance.

**[12.2.2]** Existing promoted guidance omissions and edge omissions shall first be reconstructed into an explicit frozen deferred set. Baseline and candidate performance shall then be compared on the same automatic-work set.

**[12.2.3]** The existing `EdgePatchOmission.Apply` proportional fill reduction shall not run in the optimized path. It may remain in `LegacyGreedy` mode for baseline reproduction only.

### 12.3 Authorized later analysis

**[12.3.1]** When explicitly enabled, a cell may be deferred only after all configured flow corridors, lane headings, orientations, and bounded manual-edge alternatives fail exact guidance or make the complete fixed-work plan slower than the retained plan.

**[12.3.2]** Deferring cut and fill work shall preserve a balanced automatic plan. The paired deferred entries may occur at different coordinates, but neither entry may be proportionally smeared over unrelated cells.

**[12.3.3]** A deferred-work time lower bound and a separate completion plan or `UNRESOLVED` reason shall be reported. Automatic-plan time and completion-equivalent time shall be shown side by side.

**[12.3.4]** A candidate with more deferred BCY than the frozen baseline shall not satisfy the significant-improvement gate without explicit user authorization after reviewing the deferred ledger.

## 13. Iteration and benchmark workflow

### 13.1 Required bench command

**[13.1.1]** `DotNET/Bench` shall provide a non-interactive `--optimize-haul` command that accepts the AGD path, optimizer-profile file, output directory, cache directory, exact-finalist count, runtime budget, and optional resume checkpoint.

**[13.1.2]** The command shall write `run.json`, `baseline.json`, `candidates.csv`, `best-plan.json`, `ledger.csv`, `hotspots.csv`, `deferred-work.csv`, `exact-guidance.txt`, and a concise progress file.

**[13.1.3]** The command shall return exit code `0` only when all invariants pass and the requested gate passes, `2` for a valid improvement below the gate, `3` for no valid improvement, and a nonzero failure code for invariant, guidance, or execution failure.

### 13.2 Profile-driven search

**[13.2.1]** Search weights, corridor widths, candidate counts, run limits, load thresholds, portfolio profiles, and repair limits shall be serializable configuration rather than hard-coded experimental edits.

**[13.2.2]** Every run shall embed the complete resolved configuration, source commit, dirty-worktree flag, AGD checksum, options checksum, guidance-oracle version, cache statistics, wall time, and random seed. Production decisions shall remain deterministic for a fixed seed.

**[13.2.3]** The harness shall support a fast mode that ends after validated screening and an exact mode that evaluates finalists through production whole-plan guidance.

### 13.3 Checkpoint loop

**[13.3.1]** Each iteration shall load the last fully validated best checkpoint, generate a new deterministic portfolio, reject invalid plans, screen valid plans, exactly evaluate finalists, and promote only a strictly better exact result.

**[13.3.2]** A promoted checkpoint shall be written atomically and shall include the complete allocation and ledger needed to reproduce it without rerunning the search.

**[13.3.3]** A failed or interrupted iteration shall leave the prior checkpoint unchanged.

**[13.3.4]** The next iteration shall be driven by the previous exact hotspot report. It shall not repeat an identical candidate or neighborhood checksum.

**[13.3.5]** The harness shall emit a one-line comparison suitable for automated agent review: baseline hours, best hours, percent savings, distance, run count, deferred BCY, manual distance, exact status, and gate status.

### 13.4 Runtime controls

**[13.4.1]** Exact cycle and transition caches shall persist between benchmark invocations when their full keys match.

**[13.4.2]** Search and exact evaluation shall have separate runtime budgets. A budget stop shall retain the best validated result and report `SEARCH_TIME_LIMIT`; it shall not imply convergence.

**[13.4.3]** Full-field exact evaluation shall show periodic progress and shall not require image rendering.

### 13.5 Required options and rollout defaults

**[13.5.1]** The following options shall be represented in `HaulOptimizationOptions`, copied into every run report, and overrideable by the benchmark profile. Defaults are normative starting points for the first complete implementation; section 13 permits measured profile iteration before production promotion.

**[13.5.2]**

| Option | Production default | Unit or meaning |
|---|---:|---|
| `HaulBuilderOptions.LaneAxisHeadingDeg` | `0.0` | Undirected lane axis normalized to `[0, 180)`; guidance independently selects each run bearing. |
| `Optimization.MaxLaneSlicesPerCycle` | `10` | Maximum source-lane slices packed per work kind and cycle. |
| `Optimization.FlowPenaltyWeight` | `1.0` | Deviation weight against the unchanged Poisson field. |
| `Optimization.EdgeMaxLaneSlicesPerCycle` | `12` | Residual-edge lane-slice bound. |
| `Optimization.EnableRouteConsolidation` | `true` | Enables exact-balanced capacity-flexible consolidation. |
| `Optimization.RouteConsolidationGroupSize` | `8` | Maximum routes in one consolidation transaction. |
| `Optimization.RouteConsolidationMaximumCycles` | `50` | Maximum cycles removed by consolidation. |
| `Optimization.PairRefinementRounds` | `8` | Deterministic equal-load centroid refinement rounds. |
| `Optimization.EnableMultiRouteExchange` | `false` | Optional slower closed fragment exchange. |
| `Optimization.MultiRouteExchangeRounds` | `2` | Deterministic fragment-exchange rounds. |
| `Optimization.MultiRouteExchangeGroupSize` | `8` | Maximum routes in one exchange group. |
| `Optimization.MultiRouteExchangeMaximumGroups` | `100` | Maximum committed exchange groups. |
| `Optimization.MultiRouteExchangeMinimumImprovementM` | `1.0` | Minimum accepted compact-guidance improvement. |
| `Optimization.EnableCycleChainOptimization` | `true` | Enables exact-gated whole-fill/next-cut refinement. |
| `Optimization.CycleChainRounds` | `1` | Field-wide exchange rounds. |
| `Optimization.CycleChainCandidatesPerRoute` | `1` | Exact candidates checked per route. |
| `Optimization.CycleChainMinimumImprovementSeconds` | `0.25` | Minimum provisional production-objective saving. |
| `Optimization.RouteReorderWindow` | `200` | Deterministic route relocation window. |

Poisson flow, residual-edge completion, exact ledger validation, direction-aware sequencing,
mandatory grade return, final-surface preservation, cut/fill-ratio preservation, and
obstruction-aware guidance are fixed production rules and are not optional parameters.

**[13.5.3]** A configured limit that truncates lane alternatives, transport edges, portfolio plans, exact finalists, or repair candidates shall emit a structured truncation diagnostic. Reaching a limit shall not be reported as proof of optimality.

**[13.5.4]** Options used to establish the section 10.3 release result shall be promoted together as a named, immutable default profile after sensitivity runs demonstrate that small option perturbations do not destroy feasibility or create a large time regression.

## 14. Testing requirements

### 14.1 Ledger and lane unit tests

**[14.1.1]** Tests shall prove exact required-depth reconstruction from canonical layers, including partial last layers and tolerance removal.

**[14.1.2]** Tests shall prove every non-deferred layer entry is owned exactly once by the selected lane cover and exactly once by the final cycle set.

**[14.1.3]** Tests shall prove lane construction is deterministic under shuffled input and parallel execution.

**[14.1.4]** Tests shall prove lane boundaries respect invalid cells, obstacles, blade width, work kind, maximum depth, and configured gap rules.

### 14.2 Transport and packing unit tests

**[14.2.1]** A synthetic ridge-to-valley fixture shall pair material through decreasing Poisson potential and exactly exhaust supply and demand.

**[14.2.2]** Competing cut sources shall not over-claim one fill lane; graph capacity and destination demand shall be enforced.

**[14.2.3]** A stranded component shall use a reported fallback edge or fail with a structured reason; it shall never disappear.

**[14.2.4]** Cycle packing tests shall prove capacity, balance, minimum-load exceptions, maximum run count, no duplicate cell within a cycle, and exact boundary-split recombination.

**[14.2.5]** Orientation tests shall demonstrate a case where the nearest geometric endpoints are slower than a longer radius-valid orientation and select the faster exact cycle.

### 14.3 Cost-oracle tests

**[14.3.1]** `PlanExact` shall produce identical lifecycle geometry and timing to `GuidancePlanner.Build` for the same canonical route and options.

**[14.3.2]** Cache-on and cache-off evaluations shall produce identical results.

**[14.3.3]** Proxy ranking tests shall include cases where fewer runs are slower, shorter Euclidean haul is slower, and a longer connector is faster because it avoids a turn or manual segment.

**[14.3.4]** No proxy or lower-bound evaluation shall be able to commit a candidate.

### 14.4 Guidance invariant tests

**[14.4.1]** All existing forward-only, minimum-radius, field-containment, one-cell manual allowance, lineup, coverage, lifecycle, grade-return, continuity, and timing-invariance tests shall apply unchanged to optimized plans.

**[14.4.2]** Optimized-plan tests shall prove no reverse, no instantaneous heading reversal, no geometry discontinuity, no automatic travel beyond the operational field, no manual travel beyond the existing allowance, and no post-validation coordinate mutation.

**[14.4.3]** Guidance construction shall preserve the exact selected cell-depth multiset. Guidance-deferred entries shall reconcile with the optimizer deferred ledger rather than disappearing inside coverage planning.

### 14.5 Property and failure tests

**[14.5.1]** Small randomized balanced grids shall verify conservation, capacity, determinism, transactional rollback, and `PlanExact` feasibility without relying on TheShop2.

**[14.5.2]** Failure injection shall prove that an exception, timeout, invalid cache entry, or failed exact candidate cannot corrupt the best checkpoint or production plan.

### 14.6 TheShop2 acceptance

**[14.6.1]** The full-field test shall verify the AGD checksum, options checksum, baseline ledger, baseline exact timing, and promoted report before comparing candidates.

**[14.6.2]** The acceptance test shall run the complete field, not a route subset, PNG, proxy, or extrapolation.

**[14.6.3]** The selected plan shall pass the `50%` time gate in section 10.3 with no new deferred work, unchanged flow field, exact per-cell reconciliation, unchanged cut/fill ratio, and all guidance invariants.

**[14.6.4]** Acceptance shall report phase and payload time changes so savings can be attributed to fewer work runs, shorter loaded transit, shorter maneuvering, shorter next-cut transit, or other exact phases.

### 14.7 Test matrix and known baseline failures

**[14.7.1]** The required matrix shall include focused optimizer unit tests, all `HaulBuilder.Tests`, legacy plan parity tests in `LegacyGreedy` mode, guidance tests, deterministic repeated TheShop2 planning, and full exact TheShop2 acceptance.

**[14.7.2]** The ten failures recorded in section 4.3 shall be resolved or deliberately replaced with reviewed contracts before optimizer rollout. Updating a stale expected legacy fingerprint is permitted only after proving the legacy result is deterministic and explaining the changed source of truth.

**[14.7.3]** Python parity shall not assert that the new optimized plan reproduces greedy Python pass ownership. It shall continue to verify shared grid, volume, flow, and explicitly selected legacy-mode behavior.

## 15. Source layout and integration

### 15.1 New production files

**[15.1.1]** The implementation shall add the following primary files, subject to the repository C# style guide.

**[15.1.2]**

```text
DotNET/HaulBuilder/HaulWorkLedger.cs
DotNET/HaulBuilder/HaulLaneBuilder.cs
DotNET/HaulBuilder/HaulTransportGraph.cs
DotNET/HaulBuilder/HaulCycleOptimizer.cs
DotNET/HaulBuilder/HaulOptimizationOptions.cs
DotNET/HaulBuilder/HaulOptimizationResult.cs
DotNET/HaulBuilder/GuidanceCostOracle.cs
DotNET/HaulBuilder/HaulRefinement.cs
DotNET/HaulBuilder/HaulPlanValidator.cs
DotNET/HaulBuilder/HaulOptimizationReport.cs
```

**[15.1.3]** One primary top-level type per file is the norm. Large experimental algorithms shall not be embedded into `HaulBuilderEngine.cs`, `Passes.cs`, or `GuidancePlanner.cs`.

### 15.2 Existing production files

**[15.2.1]** `FlowField.cs` shall remain algorithmically unchanged. A parity test may expose additional read-only graph inputs if required.

**[15.2.2]** `Passes.cs` shall retain legacy generation and `BuildPass`. Optimized allocation logic shall not be added to its greedy depletion loops.

**[15.2.3]** `GuidancePlanner.cs` may expose internal, side-effect-free production adapters needed by `GuidanceCostOracle`; connector or timing mathematics shall not be duplicated.

**[15.2.4]** `HaulBuilderEngine.cs` shall select `LegacyGreedy` or `GlobalFlowGuidance` allocation, always build lightweight final guidance for production timing, and gate only artifact rendering and export.

**[15.2.5]** `HaulBuilderOptions.cs` and `HaulBuilderResult.cs` shall expose resolved optimization mode, diagnostics, ledgers, exact baseline/candidate metrics, and deferred-work state.

**[15.2.6]** `EdgePatchOmission.cs` shall be bypassed in optimized mode and clearly marked as a legacy allocation policy.

### 15.3 New test files

**[15.3.1]** The implementation shall add focused tests in the following files.

**[15.3.2]**

```text
DotNET/HaulBuilder.Tests/HaulWorkLedgerTests.cs
DotNET/HaulBuilder.Tests/HaulLaneBuilderTests.cs
DotNET/HaulBuilder.Tests/HaulTransportGraphTests.cs
DotNET/HaulBuilder.Tests/HaulCycleOptimizerTests.cs
DotNET/HaulBuilder.Tests/GuidanceCostOracleTests.cs
DotNET/HaulBuilder.Tests/HaulRefinementTests.cs
DotNET/HaulBuilder.Tests/HaulPlanValidatorTests.cs
DotNET/HaulBuilder.Tests/HaulOptimizationDeterminismTests.cs
DotNET/HaulBuilder.Tests/TheShop2HaulOptimizationAcceptanceTests.cs
```

## 16. Work-unit implementation sequence

### 16.1 Work unit 0 - freeze and reconcile evidence

**[16.1.1]** Freeze the AGD, resolved options, flow-field checksum, legacy allocation, complete required/assigned/deferred ledgers, promoted guidance report, and exact test command.

**[16.1.2]** Reproduce the `433.483 h` baseline from current source or document and resolve any drift before optimizer code is enabled.

**[16.1.3]** Classify and resolve the ten existing test failures described in section 4.3. No optimization behavior changes are permitted in this work unit.

### 16.2 Work unit 1 - canonical ledger and deferral model

**[16.2.1]** Implement the required work ledger, depth layers, fixed-work comparison, and explicit deferred ledger.

**[16.2.2]** Convert baseline edge and guidance omissions into machine-readable evidence without changing the legacy plan.

### 16.3 Work unit 2 - guidance cost oracle

**[16.3.1]** Add side-effect-free lane, cycle, transition, and plan evaluation adapters plus cache keys and equality tests.

**[16.3.2]** Prove `PlanExact` parity with production output before any optimizer score is trusted.

### 16.4 Work unit 3 - lane inventory

**[16.4.1]** Implement flow-aware lane candidates, lane cover selection, slicing, endpoint metrics, and diagnostics.

**[16.4.2]** Demonstrate a substantial reduction in predicted runs per load on TheShop2 without assigning cut to fill yet.

### 16.5 Work unit 4 - transport graph and global material assignment

**[16.5.1]** Implement flow corridors, destination capacity, global balanced pairing, fallback classification, and deterministic solver tests.

**[16.5.2]** Audit complete material assignment before creating `Pass` objects.

### 16.6 Work unit 5 - cycle packing and orientation

**[16.6.1]** Implement capacity-bounded cycles, exact balance, run limits, orientation candidates, and cycle screening.

**[16.6.2]** Require all ledger, capacity, and duplicate-cell tests to pass before sequencing.

### 16.7 Work unit 6 - sequence and complete-plan portfolio

**[16.7.1]** Implement multi-start sequencing, direction-safe improvements, deterministic profiles, complete-plan screening, and exact finalists.

**[16.7.2]** Retain legacy baseline and best checkpoint in every portfolio.

### 16.8 Work unit 7 - hotspot refinement and iteration harness

**[16.8.1]** Implement exact hotspot reports, bounded repair neighborhoods, persistent caches, atomic checkpoints, resume behavior, and bench exit codes.

**[16.8.2]** Run repeated iterations until the section 10.3 gate passes or the evidence required by section 10.3.6 is produced.

### 16.9 Work unit 8 - production integration

**[16.9.1]** Integrate optimized allocation behind an explicit option, always build lightweight guidance timing, and run the full test matrix.

**[16.9.2]** Production default shall remain legacy until the exact full-field gate passes in a dedicated checkpoint.

### 16.10 Work unit 9 - default promotion

**[16.10.1]** Promote `GlobalFlowGuidance` to the default only after a clean-worktree reproduction passes all invariants, the full test matrix, exact TheShop2 acceptance, deterministic repetition, and artifact review.

**[16.10.2]** Preserve `LegacyGreedy` and the promoted baseline command for regression diagnosis.

## 17. Stop conditions and prohibited shortcuts

**[17.0.1]** Stop and preserve the last passing checkpoint if any candidate changes target elevation, cut/fill ratio, grid, capacity, equipment, speeds, or guidance rules.

**[17.0.2]** Stop if any cell's required depth does not reconcile exactly to assigned, deferred, and tolerance-removed depth.

**[17.0.3]** Stop if a cycle is unbalanced, over capacity, exceeds maximum per-cycle depth, or services the same cell twice.

**[17.0.4]** Stop if exact oracle output differs from production guidance and timing for the same canonical plan.

**[17.0.5]** Stop if a promoted improvement depends on proxy score, changed work, additional deferral, reverse travel, weakened radius, excessive field exit, unvalidated manual travel, disabled grade return, or changed timing assumptions.

**[17.0.6]** Stop if identical repeated runs change allocation, route order, guidance feasibility, exact metrics, or diagnostics beyond documented floating-point tolerance.

**[17.0.7]** Do not reduce the 50% gate, mark a failing test skipped, weaken a guidance assertion, edit a golden report, or alter the baseline fixture merely to declare success.

**[17.0.8]** A stop condition shall emit the minimal fixture or affected ledger, candidate fingerprint, exact failure, current best checkpoint, and next actionable investigation.

## 18. Implementation reinforcement checklist

**[18.0.1]** ☐ Baseline AGD, options, source commit, flow field, allocation, deferral, and exact timing are frozen and reproducible (section 16.1).

**[18.0.2]** ☐ The pre-existing ten test failures are resolved or replaced by reviewed current contracts (sections 4.3 and 14.7).

**[18.0.3]** ☐ Every required cell-depth layer reconciles to automatic, deferred, or tolerance-removed work (sections 2 and 5).

**[18.0.4]** ☐ Lane construction precedes load ownership and materially reduces fragmented work-run candidates (section 7).

**[18.0.5]** ☐ Global pairing accounts for destination demand and flow magnitude rather than making irrevocable nearest-fill choices (section 8).

**[18.0.6]** ☐ Every cycle is balanced, within capacity, and contains no duplicate physical-cell service (section 8.3).

**[18.0.7]** ☐ Every cost is seconds or an explicit feasibility tier; proxy scores cannot commit plans (sections 9 and 10).

**[18.0.8]** ☐ Exact candidate evaluation is identical to production guidance and timing (section 9).

**[18.0.9]** ☐ Guidance remains forward-only, radius-valid, continuous, field-aware, and grade-return complete (sections 2 and 14.4).

**[18.0.10]** ☐ Difficult cells remain exact deferred ledger entries and cannot manufacture the accepted savings (section 12).

**[18.0.11]** ☐ The bench can resume from an atomic checkpoint, reuse valid caches, and drive the next iteration from exact hotspots (section 13).

**[18.0.12]** ☑ The full TheShop2 production plan reaches at least `200 BCY` per deterministic guidance hour while separately reporting scheduled time and deferred work (sections 0.2.27, 0.2.29, and 10.3).

**[18.0.13]** ☐ The optimizer becomes default only after clean-worktree, full-suite, deterministic, exact full-field reproduction (section 16.10).

## 19. Summary

**[19.0.1]** The large guidance-time increase is caused by globally poor machine-cycle geometry hidden by the former straight-line time model. Near-capacity load count is not the dominant defect; fragmentation, maneuvering, and transit are.

**[19.0.2]** The required correction is to construct serviceable lanes first, globally pair their material through the unchanged commercial-matching flow field, pack and orient complete machine cycles, sequence them, and select only plans whose production guidance proves the saving.

**[19.0.3]** Local fragment reassignment remains useful as a final repair operation, but it is no longer the main strategy. Exact ledgers, a fixed-work comparison, immutable guidance rules, a quantitative `50%` gate, and a checkpointed benchmark loop make the improvement both safe and iteratively achievable.
