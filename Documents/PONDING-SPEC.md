# Ponding-Aware Sequencing Specification **[0.0.1]**

**[0.0.1]** This document specifies an **optional extension** to the land leveling haul algorithm: **ponding-aware sequencing**. When enabled, the implementation **replays** the field surface after each pass in route order, **scores** intermediate topography for **closed-depression ponding risk** using an external **ponding analyser** (WhiteBoxTools or equivalent), and **reorders** passes to reduce transient ponding during long jobs — without changing pass geometry, moved volume, or the finished surface.

**[0.0.2]** **Relationship to LAND-LEVELING-SPEC-2.md.** The baseline haul algorithm (**LAND-LEVELING-SPEC-2.md**, hereafter **baseline spec**) remains authoritative for AGD reading, grid model, flow field, pass generation, haul leg geometry, guidance, and time models. This document **extends** baseline **§12** (sequencing and routing) and adds one post-routing stage after baseline **§12.2** (windowed order refinement). It **shall not** change volume accounting (**§7.3**), pass matching (**§11**), haul leg geometry (**§11.0.13**), or blade-scale guidance (**§13**) except for optional pass metadata and reporting called out below.

**[0.0.3]** **Language / implementation.** This specification is **implementation-language-neutral**. Any conforming implementation of the baseline spec may implement this extension. The reference ponding analyser is **WhiteBoxTools** invoked as an external process; implementations **may** substitute an in-process equivalent that produces **comparable metrics** (**§7**). Command-line options use POSIX-style **`--name`** flags.

**[0.0.4]** **Related documents:** **LAND-LEVELING-SPEC-2.md** (baseline algorithm), **LAND_LEVELING_PROBLEM.md** (physical model), **TOPSOIL-OPTIMIZATION-SPEC.md** (optional material-class extension; independent of this document), **ANDY-SPECSTYLEGUIDE.md** (authoring rules).

**[0.0.5]** **Assumed knowledge.** Readers **shall** be familiar with baseline **§7** (grid), **§11** (pass generation and layering), **§12** (sequencing), and baseline **§12.0.3** (*anti-dam ordering*, informative). In this document, bare **§N** means **this document**; **baseline §N** means **LAND-LEVELING-SPEC-2.md**.

**[0.0.6]** Every normative paragraph carries a bold marker **`[<S>.<U>.<P>]`** where **`<S>`** is the section, **`<U>`** is the subsection (**`0`** for body text before the first **`###`**), and **`<P>`** is the 1-based paragraph index within that group.

### 0.1 Specification revision history

**[0.1.1]** | Version | Date | Change |
|---|---|---|
| **0.0.1** | 2026-07-09 | Initial draft: intermediate-surface replay, outlet model, WhiteBoxTools ponding scorer, layer-order constraints, drainage-aware reorder, audit mode, CLI, outputs. |

**[0.1.2]** This document follows [semantic versioning](https://semver.org/): **MAJOR** = incompatible normative change, **MINOR** = backward-compatible addition, **PATCH** = clarification.

---
## 1. Purpose

**[1.0.1]** Land leveling on large fields can run for **many consecutive hours** or **multiple days**. Rainfall during that window can pond on a **partially graded** surface. If fill is placed before natural **outlet paths** are opened, the job can create **transient dams** that turn workable ground into bog and cause **days of delay** — even when the **final** target surface drains correctly.

**[1.0.2]** Baseline **§12.0.3** states an informative preference to favour **anti-dam ordering** (complete downhill flow first). This extension **formalises** that preference as an **optional, measurable constraint** backed by ponding analysis on **intermediate surfaces**, not only on the finished design.

**[1.0.3]** Ponding-aware sequencing **shall**, when enabled:

**[1.0.4]** 1. **Replay** the elevation surface after each pass in the current route order (**§6**).

**[1.0.5]** 2. **Score** each checkpoint surface for **closed-depression ponding** relative to declared **outlets** (**§5**, **§7**).

**[1.0.6]** 3. **Reorder** passes — subject to **layer-order** and baseline routing rules — to reduce a combined **time + ponding** objective (**§8**).

**[1.0.7]** 4. **Report** worst-case ponding windows, metrics, and any **residual risk** the operator should treat as a work-stoppage flag (**§10**).

**[1.0.8]** When disabled (default), behaviour **shall** be identical to the baseline spec (including baseline **§12.2** refinement when requested).

---
## 2. Design goals

**[2.0.1]** The extension **shall provide**:

**[2.0.2]** 1. **Intermediate-surface replay** using the same cell depths recorded on each pass as baseline animation (**baseline §14**, **`animate.py`** pattern) — **§6**.

**[2.0.3]** 2. A **ponding analyser interface** pluggable behind a single scorer API, with **WhiteBoxTools** as the reference backend — **§7**.

**[2.0.4]** 3. **Hard layer-order invariants** so reordering never violates baseline **§12.0.2** (shallower layer before deeper layer on each cell) — **§8.2**.

**[2.0.5]** 4. **Drainage-aware local search** extending baseline **§12.2** relocation moves with a **ponding penalty**, preserving each pass's internal geometry — **§8**.

**[2.0.6]** 5. An **audit-only mode** that scores the baseline route without reordering, for field-office review before execution — **§8.5**, **§10**.

**[2.0.7]** 6. **Inspectable outputs** (summary, CSV, optional worst-checkpoint map) so ponding risk is visible before the machine starts — **§10**.

**[2.0.8]** 7. A **minimal integration surface**: one post-routing stage, pass metadata for layer indices, scorer subprocess or library — no matcher changes.

---
## 3. Non-goals

**[3.0.1]** This extension **shall not** attempt the following in this version:

**[3.0.2]** 1. **Temporary drainage channels** or **drainage-first pass generation** — prioritising cuts along thalweges, inserting dedicated channel passes, or biasing the baseline matcher anchor (**baseline §11**). Those are a separate future extension.

**[3.0.3]** 2. **Changing pass geometry** — cut/fill cell sets, depths per cell, haul legs, or load sizes. Reordering **shall not** add, remove, split, or merge passes (**baseline §12.2.5**).

**[3.0.4]** 3. **Changing the target surface** or re-solving field drainage design. Outlets are **derived** from the supplied target and boundary, not redesigned (**§5**).

**[3.0.5]** 4. **Rainfall hyetograph simulation**, infiltration modelling, or sizing storms. Metrics describe **topographic ponding capacity** on the partial surface, not weather forecasts.

**[3.0.6]** 5. **Online adaptive replanning** during execution from as-built survey feedback. This spec covers **pre-job plan optimisation** only.

**[3.0.7]** 6. **Guaranteeing zero ponding** under all rainfall. The objective **reduces** transient closed-depression risk; residual risk **shall** be reported when thresholds are exceeded (**§10.2**).

**[3.0.8]** 7. Replacing baseline **shortest-haul-first** pass generation or the baseline flow field (**baseline §9**) with a hydrological flow model. Soil-transport flow and water flow remain distinct.

---
## 4. Terminology

### 4.1 Intermediate surface

**[4.1.1]** The **intermediate surface** at pass index **`k`** (0-based, **`k = 0`** before any work) is the elevation raster obtained by starting from **`grid.existing`** and applying passes **`0 … k−1`** of the **current route order**: subtract each pass's cut depths and add each pass's fill depths on the affected cells (**§6**). It is the surface the field would present if work stopped after pass **`k−1`**.

### 4.2 Outlet and outlet cell

**[4.2.1]** An **outlet** is a location where water **may leave** the field during ponding analysis — typically the field **perimeter**, a **culvert**, or a **design low point** on the target that breaches to the boundary. An **outlet cell** is a valid grid cell marked **`true`** in **`outlet_mask[by, bx]`** (**§5**).

**[4.2.2]** Ponding analysis **shall** treat water as able to drain from any cell that has a **monotonically descending path** (D8 or equivalent) to an outlet cell without crossing invalid (off-field) cells.

### 4.3 Closed depression

**[4.3.1]** A **closed depression** is a connected set of valid cells whose **intermediate surface** elevations are **lower than all neighbouring valid cells** (or lower than the spill elevation to an outlet), such that water would **pond** before reaching an outlet under a **fill-until-spill** model. Implementations **shall** use the ponding analyser's depression detection (**§7.2**) as the normative definition at runtime.

### 4.4 Ponding metrics

**[4.4.1]** A **ponding score** is a non-negative scalar summarising risk at one checkpoint. The **primary metric** **shall** be **`closed_depression_volume_bcy`**: the total **banked cubic yards** of void space in all closed depressions on the intermediate surface, computed as **`depth_to_spill × cell_area`** summed over depression cells and converted with **`M3_TO_CY`** (**baseline §7.3**).

**[4.4.2]** **Secondary metrics** **shall** include **`max_pond_depth_m`** (deepest closed depression relative to its spill edge) and **`closed_depression_count`**. Implementations **may** record additional analyser outputs in the report CSV (**§10.3**).

### 4.5 Layer index

**[4.5.1]** For each pass **`p`** and each cell **`c`** that **`p`** modifies, the **layer index** **`layer(p, c)`** is the 1-based count of prior passes in the **generation order** (baseline pass list index, **not** route order) that already removed or placed material on **`c`**. Because baseline matching caps depth at **61 mm** per visit (**baseline §11.0.20**), **`layer(p, c)`** corresponds to the **shallow-to-deep layer** of that cell.

**[4.5.2]** **Layer-order constraint:** for a fixed cell **`c`**, if pass **`a`** and pass **`b`** both modify **`c`**, and **`layer(a, c) < layer(b, c)`**, then **`a`** **shall** appear **before** **`b`** in any legal route order.

### 4.6 Drainage checkpoint

**[4.6.1]** A **drainage checkpoint** is a pass index **`k`** at which the implementation **materialises** the intermediate surface and invokes the ponding analyser. Checkpoints **shall** include **`k = n_passes`** (final surface) and **shall** be chosen per **§8.3** during optimisation.

### 4.7 Anti-dam ordering

**[4.7.1]** **Anti-dam ordering** is any route order that **reduces** **`closed_depression_volume_bcy`** on intermediate surfaces compared to the baseline route, while respecting **§4.5.2**. It operationalises baseline **§12.0.3** as a **soft objective** weighted against transition time (**§8.1**).

---
## 5. Stage P1 — Outlets and boundary conditions

**[5.0.1]** Ponding-aware sequencing **shall** run **once** after the grid is built (**baseline §7.2**) and **before** pass generation completes scoring (outlets depend only on target and boundary). Outlet construction **shall** attach:

| Array / structure | Type | Meaning |
|---|---|---|
| **`outlet_mask`** | bool `[by, bx]` | **`true`** on outlet cells (**§5.2**–**§5.4**) |
| **`outlet_source`** | enum per cell | How the cell became an outlet (boundary, culvert, target-spill) |

**[5.0.2]** If **`outlet_mask`** is empty on all valid cells, ponding optimisation **shall not** run; the implementation **shall** emit a **WARNING** and skip to audit reporting with an error metric (**§10.2**).

### 5.1 Field boundary outlets

**[5.1.1]** When AGD **`2PER`** boundary rows are present (**baseline §6.1.12**), every **valid** cell that shares an edge with an **invalid** cell (field perimeter on the grid) **shall** be marked an outlet with source **`BOUNDARY`**.

**[5.1.2]** When no **`2PER`** rows are present, the **largest connected valid component** mask (**baseline §6.1.12**) **shall** be used; perimeter valid cells adjacent to invalid cells **shall** be **`BOUNDARY`** outlets.

### 5.2 Target-spill outlets

**[5.2.1]** When **`--ponding-target-spill-outlets`** is enabled (default **on**), each valid cell whose **target** elevation is within **`--ponding-spill-tolerance-ft`** (default **0.1 ft**) of the **minimum target** among valid perimeter cells **shall** be marked an outlet with source **`TARGET_SPILL`**, provided it has a D8 path to a **`BOUNDARY`** outlet on the **target** surface. This captures design daylight points that may not lie exactly on the grid edge.

**[5.2.2]** **`--no-ponding-target-spill-outlets`** restricts outlets to **`BOUNDARY`** and manual culverts only.

### 5.3 Manual culvert outlets

**[5.3.1]** **`--ponding-outlet "lat,lon"`** (repeatable) **shall** mark the containing valid cell (nearest valid cell if the point falls outside the mask) as an outlet with source **`CULVERT`**. Projection rules **shall** match baseline exclusion polygons (**baseline §7.2.8**).

**[5.3.2]** **`--ponding-outlet-polygon "lat,lon;..."`** (repeatable, **≥ 3** vertices) **shall** mark all valid cells intersecting the polygon as **`CULVERT`** outlets.

### 5.4 Outlet precedence

**[5.4.1]** Manual culverts **shall** override nothing — they **add** outlets. All outlet sources are **unioned** into **`outlet_mask`**.

---
## 6. Stage P2 — Intermediate surface replay

**[6.0.1]** The implementation **shall** provide a deterministic **surface replay** function that, given **`grid.existing`**, an ordered list of passes, and a prefix length **`k`**, returns the intermediate surface **`Z_k[by, bx]`** in metres on valid cells (NaN or sentinel elsewhere), by applying:

```text
Z_0       = existing
Z_{k+1}   = Z_k − cut_depth(p_k, c)   on each cut cell c of pass p_k
          = Z_k + fill_depth(p_k, c)  on each fill cell c of pass p_k
```

**[6.0.2]** Replay **shall** use the **exact** per-cell depths stored on each **`Pass`** (**baseline §11.0.21**), not the remaining ledger. Invalid cells **shall** remain invalid and **shall not** participate in ponding analysis.

**[6.0.3]** Replay **shall** be **incremental** during optimisation: when evaluating a relocation move, implementations **should** update **`Z`** by delta from the previous checkpoint rather than rebuilding from scratch for every candidate.

**[6.0.4]** For export to the ponding analyser (**§7.1**), the implementation **shall** write a **GeoTIFF** or **WhiteBoxTools-readable ASCII grid** with the same georeferencing as the baseline grid (**baseline §7.1** UTM origin and **`grid_size`**).

---
## 7. Stage P3 — Ponding analysis

### 7.1 Ponding analyser interface (normative)

**[7.1.1]** Implementations **shall** expose an internal **`PondingAnalyser`** (name may vary) with at minimum:

```text
score(surface_raster, valid_mask, outlet_mask, cell_area_m2) → PondingMetrics
```

**[7.1.2]** **`PondingMetrics`** **shall** contain **`closed_depression_volume_bcy`**, **`max_pond_depth_m`**, and **`closed_depression_count`**. **`analyser_ok`** (bool) **shall** indicate whether the backend completed successfully.

**[7.1.3]** The reference backend **shall** invoke **WhiteBoxTools** (external executable) with a documented tool chain (**§7.2**). Implementations **may** ship an in-process fallback for CI; production runs **should** use WhiteBoxTools when **`--whitebox-tools-path`** is set.

### 7.2 WhiteBoxTools reference pipeline (normative for reference implementation)

**[7.2.1]** The reference implementation **shall** use the following sequence (tool names are WhiteBoxTools **snake_case** identifiers; equivalent flags **shall** be documented in code comments):

**[7.2.2]** 1. **`breach_depressions_least_cost`** — input: intermediate surface DEM, **`flat_increment`** small (default **0.001 m**), **`fill`** **false** (measure depressions, do not modify the job surface). Outlets **shall** be enforced by masking or post-processing so breaches only occur toward **`outlet_mask`** cells.

**[7.2.3]** 2. **`sink`** or **`extract_valleys`** / depression labelling — identify contiguous closed depressions on the **unmodified** intermediate surface relative to outlet connectivity.

**[7.2.4]** 3. **`max_depth_in_watershed`** or per-depression depth statistic — compute **`max_pond_depth_m`**.

**[7.2.5]** 4. Volume integration — for each depression cell, **`depth_to_spill × cell_area_m2`**, summed and converted to BCY.

**[7.2.6]** Implementations **shall** document the exact tool list and parameters in the reference **`ponding.py`** (Python) / **`Ponding.cs`** (C#) module header. Changing the tool chain **shall** bump this spec's **MINOR** version and update golden tests (**§12**).

### 7.3 Performance and caching

**[7.3.1]** Full ponding analysis on every pass **may** be prohibitively slow on large fields. During **optimisation**, implementations **shall** evaluate ponding only at **drainage checkpoints** (**§8.3**), not necessarily after every pass.

**[7.3.2]** Implementations **shall** cache **`PondingMetrics`** keyed by **`(route_prefix_hash, k)`** when the same prefix reappears during local search.

**[7.3.3]** **`--ponding-checkpoint-interval N`** (default **25**) defines the maximum pass index gap between consecutive optimisation checkpoints. **`N = 1`** forces every pass (audit-quality, slow); **`N = 0`** means **checkpoints only at pass 0, pass n/2, and pass n** (fast, coarse).

---
## 8. Stage P4 — Drainage-aware reorder

**[8.0.1]** When **`--ponding-optimize`** is enabled, the implementation **shall** run **after** baseline routing (**baseline §12.0**) and baseline knot re-profiling (**baseline §13.3b**), and **after** baseline **`--reorder-window`** refinement (**baseline §12.2**) when that is enabled — replacing or extending the **§12.2** objective with **§8.1**. It **shall** run **before** the time model (**baseline §13.0**), zone sequencing (**baseline §12.1**), and all exports that depend on route order.

### 8.1 Combined objective

**[8.1.1]** Let **`T(route)`** be the baseline **order-dependent transition cost** — the sum of **`cost(a, b)`** from **baseline §12.0.5** over consecutive passes (seconds when direction-aware, else deadhead metres). Let **`P(route)`** be the **ponding penalty**:

```text
P(route) = max over checkpoints k of closed_depression_volume_bcy(Z_k, route)
```

**[8.1.2]** The drainage-aware search **shall** minimise:

```text
J(route) = T(route) + λ × P(route)
```

**[8.1.3]** **`λ`** is **`--ponding-weight`** (default **3600**): penalty units are **BCY·seconds/BCY** when **`T`** is in seconds, so **`λ = 3600`** treats **1 BCY** of peak trapped volume as worth **one hour** of extra deadhead. Implementations **shall** print **`λ`**, **`T`**, and **`P`** before and after optimisation.

**[8.1.4]** When **`--ponding-weight 0`**, the search **shall** optimise **`P`** alone subject to **§8.2**, ignoring **`T`** except for tie-breaking (prefer lower **`T`** at equal **`P`**).

### 8.2 Layer-order constraints (normative)

**[8.2.1]** Before search begins, the implementation **shall** compute **`layer(p, c)`** for every pass **`p`** and modified cell **`c`** (**§4.5**).

**[8.2.2]** A relocation move (**baseline §12.2.3**) **shall** be **legal** only if, in the candidate order, for every cell **`c`**, passes touching **`c`** appear in **strictly increasing** **`layer(p, c)`** order.

**[8.2.3]** Implementations **shall** evaluate legality in **O(1)** or **O(log n)** per candidate using precomputed per-cell pass lists, not by scanning the full route naïvely on every move.

**[8.2.4]** Illegal moves **shall** be discarded without scoring. They **shall not** count as "no improvement" termination.

### 8.3 Search algorithm

**[8.3.1]** The search **shall** use the same **relocation (or-opt-1)** move operator as **baseline §12.2.3** — lift one pass, re-insert elsewhere; **never** reverse a contiguous segment.

**[8.3.2]** Window, sweep, and round cap **shall** default to baseline **`--reorder-window`** values when ponding optimisation is on: window **`W`** (default **50** if unset), **12** round cap, strict improvement threshold **`1e-7`**.

**[8.3.3]** For each **legal** candidate move, **`ΔT`** **shall** be computed in **O(1)** (**baseline §12.2.3**). **`ΔP`** **shall** be computed by replaying and scoring only checkpoints whose prefix changed — implementations **should** restrict recomputation to the index range **`[min(k, k′), max(k, k′)]`** affected by the move.

**[8.3.4]** A move **shall** be accepted when **`ΔJ < −ε`**. Among equal **`J`**, prefer lower **`P`**, then lower **`T`**.

**[8.3.5]** When ponding optimisation completes, **`route.ordered_passes`**, **`route.order`**, and **`route.total_deadhead_m`** **shall** be updated in place exactly as **baseline §12.2.5**.

### 8.4 Pass metadata

**[8.4.1]** Each pass **shall** carry optional **`ponding_layer_min`** and **`ponding_layer_max`** — the min and max of **`layer(p, c)`** over cells **`c`** modified by pass **`p`**. When **`--ponding-optimize`** is off, these **may** be omitted.

**[8.4.2]** **`max_pond_depth_m`** and **`closed_depression_volume_bcy`** at the checkpoint **after** pass **`p`** **may** be stored on pass **`p`** in the final audit pass for reporting (**§10.3**).

### 8.5 Audit-only mode

**[8.5.1]** **`--ponding-audit`** **shall** run **§6**–**§7** on the **current route** (after baseline routing and any **§12.2** refinement) **without** modifying route order.

**[8.5.2]** **`--ponding-audit`** **may** be combined with **`--ponding-optimize`**; when combined, audit metrics **shall** be reported for **both** before and after states.

**[8.5.3]** **`--ponding-audit`** alone **shall** imply checkpoint evaluation at every pass when **`--ponding-checkpoint-interval`** is unset; otherwise honour the interval.

---
## 9. Integration with baseline pipeline

**[9.0.1]** The following baseline stages **shall not** be modified except as noted:

| Stage | Ponding interaction |
|---|---|
| AGD read, grid binning (**baseline §6–§7**) | Unchanged |
| **P1** Outlets (**§5**) | **New**, after grid |
| Volume balance, flow field, strips (**baseline §8–§10**) | Unchanged |
| Pass generation (**baseline §11**) | Unchanged; layer tags computed post-hoc from pass list |
| Baseline routing (**baseline §12.0**) | Unchanged |
| Baseline **`--reorder-window`** (**baseline §12.2**) | Runs **before** ponding reorder when both enabled |
| **P4** Ponding reorder (**§8**) | **New**, after **§12.2** |
| Time model, guidance (**baseline §13**) | Unchanged; uses final route |
| Animation (**baseline §14**) | Unchanged; reflects final route |

**[9.0.2]** Implementations **shall** expose ponding features as **`--ponding-optimize`** and **`--ponding-audit`** defaulting **off** so existing workflows and regression tests remain valid without flags.

**[9.0.3]** **`--ponding-optimize`** **shall not** change total moved BCY, cut/fill assignments, or the **final** surface model relative to the same pass set — only **execution order**.

---
## 10. Outputs and reporting

### 10.1 Run summary

**[10.1.1]** When **`--ponding-audit`** or **`--ponding-optimize`** is on, the program **shall** print a **ponding summary** including at minimum:

| Metric | Description |
|---|---|
| Peak **`closed_depression_volume_bcy`** | Maximum over checkpoints |
| Pass index at peak | **`worst_checkpoint_pass`** |
| Peak **`max_pond_depth_m`** | Deepest pond at worst checkpoint |
| Outlet count | Cells in **`outlet_mask`** |
| Transition cost **`T`** | Before / after when optimising |
| Ponding penalty **`P`** | Before / after when optimising |
| Relocations accepted | Count when optimising |
| Analyser failures | Count of checkpoints where **`analyser_ok`** is false |

### 10.2 Warnings and work-stoppage flags

**[10.2.1]** When peak **`max_pond_depth_m`** exceeds **`--ponding-warn-depth-ft`** (default **0.5 ft**, converted internally), the summary **shall** include **WARNING: significant transient ponding risk**.

**[10.2.2]** When peak **`closed_depression_volume_bcy`** exceeds **`--ponding-warn-volume-bcy`** (default **0** = auto), the summary **shall** include **WARNING: large trapped volume on partial surface**. Auto mode **shall** set the threshold to **`rainfall_runoff_proxy_bcy`** when **`--ponding-design-rainfall-in`** (default **1.0 in**) is supplied: runoff volume estimated as **`rainfall_depth × ponding_catchment_area`**, where **`ponding_catchment_area`** is the valid cell area contributing to the worst closed depression at the worst checkpoint.

**[10.2.3]** Warnings **shall not** abort the plan unless **`--ponding-fail-on-warn`** is set; in that case the process **shall** exit with non-zero status after writing reports.

### 10.3 CSV report

**[10.3.1]** **`--ponding-csv PATH`** **shall** write one row per checkpoint **`k`**:

```text
pass_index, closed_depression_volume_bcy, max_pond_depth_m, closed_depression_count,
transition_cost_cumulative_s, outlet_count, analyser_ok
```

**[10.3.2]** Default when flag is given without path: **`<out>_ponding.csv`**.

### 10.4 Map overlay

**[10.4.1]** **`--ponding-map-png PATH`** **shall** render the **worst-checkpoint** intermediate surface with closed depressions highlighted (e.g. blue fill) and **`outlet_mask`** cells marked (e.g. green edge).

**[10.4.2]** When **`--ponding-map-png`** is omitted but ponding features are on, the default path **shall** be **`<out>_ponding.png`**.

---
## 11. Command-line interface

**[11.0.1]** Implementations **shall** expose the following options in addition to the baseline CLI (**baseline §15**):

```text
# Master switches (default: off — baseline behaviour)
--ponding-optimize            Enable drainage-aware reorder (§8)
--ponding-audit               Score ponding on current route without reordering (§8.5)
--no-ponding-optimize         Disable explicitly (default)

# Outlets (§5)
--ponding-outlet "lat,lon"    Repeatable; culvert outlet cell
--ponding-outlet-polygon "lat,lon;..."
                              Repeatable; culvert outlet region
--ponding-target-spill-outlets
                              Enable target-spill outlets (default on)
--no-ponding-target-spill-outlets
--ponding-spill-tolerance-ft FLOAT
                              default 0.1

# Objective (§8.1)
--ponding-weight FLOAT        default 3600   (λ: BCY penalty vs seconds)
--ponding-checkpoint-interval INT
                              default 25     (0 = coarse: 0, n/2, n only)

# WhiteBoxTools backend (§7.2)
--whitebox-tools-path PATH    Directory containing whitebox_tools executable
--ponding-wbt-workdir PATH    Scratch directory for rasters (default: temp)

# Warnings (§10.2)
--ponding-warn-depth-ft FLOAT default 0.5
--ponding-warn-volume-bcy FLOAT
                              default 0 (= auto when rainfall set)
--ponding-design-rainfall-in FLOAT
                              default 1.0    (for auto volume warning)
--ponding-fail-on-warn        Exit non-zero if warnings triggered

# Outputs (§10)
--ponding-csv PATH            Checkpoint metrics CSV
--ponding-map-png PATH        Worst-checkpoint depression map
--no-ponding-map              Suppress default _ponding.png
```

**[11.0.2]** **`--ponding-optimize`** **shall** imply **`--ponding-audit`** before and after reorder unless **`--no-ponding-audit`** is provided.

**[11.0.3]** **`--ponding-optimize`** **shall** require a usable ponding analyser (**§7**). If WhiteBoxTools is not found and no fallback is configured, the implementation **shall** exit with a clear error before mutating the route.

**[11.0.4]** All length options **shall** be interpreted in **feet** (or **inches** where named) regardless of locale; internal computation **shall** convert to metres consistently with the baseline grid.

---
## 12. Worked example (informative)

**[12.0.1]** A **40 ac** field requires **~205 h** of machine time. The baseline route places several **fill patches** in a swale **before** the cut that opens the swale outlet to the **north boundary** is executed. During a **1 in** rain event on day three, water ponds **~0.8 ft** deep over **~2 ac** until the outlet cut is finished.

**[12.0.2]** With **`--ponding-audit`**, the plan reports peak **`closed_depression_volume_bcy ≈ 3200`** at pass index **840**, **`max_pond_depth_m ≈ 0.24 m`**, and **WARNING** under default thresholds.

**[12.0.3]** With **`--ponding-optimize --ponding-weight 3600`**, the search **relocates** twelve fill passes later in the route (within layer constraints) so the **outlet cut** completes first. Peak trapped volume falls to **~400 BCY**; transition cost **`T`** rises **~4%**. The **`_ponding.png`** map shows the improvement at the worst checkpoint.

**[12.0.4]** This example is **informative**; exact numbers depend on field geometry and analyser settings.

---
## 13. Implementation checklist

**[13.0.1]** A conforming implementation **shall** satisfy:

**[13.0.2]** - ☑ **`--no-ponding-optimize`** and no audit flags (default): bit-identical route and outputs to baseline on the same inputs.

**[13.0.3]** - ☑ **P1** outlet mask from boundary and options (**§5**).

**[13.0.4]** - ☑ **P2** surface replay matches animation replay on a golden route (**§6**).

**[13.0.5]** - ☑ **P3** scorer returns stable metrics on synthetic pits and plane surfaces (**§7**).

**[13.0.6]** - ☑ **P4** layer-order legality enforced; illegal moves never accepted (**§8.2**).

**[13.0.7]** - ☑ Optimisation never changes pass sets, depths, or final surface (**§9.0.3**).

**[13.0.8]** - ☑ Summary, CSV, and default **`_ponding.png`** (**§10**).

**[13.0.9]** - ☑ Unit tests: flat plane (zero depression), single pit with/out outlet, layer-order rejection, audit-only mode, optimise reduces **`P`** on crafted dam scenario, WhiteBoxTools missing → clean error.

**[13.0.10]** - ☑ Integration test: TheShop2 (or equivalent) baseline route audited in **< 60 s** with **`--ponding-checkpoint-interval 25`**.

---
## 14. Summary

**[14.0.1]** Ponding-aware sequencing reduces **transient closed-depression risk** during long land leveling jobs by **replaying** intermediate surfaces, **scoring** them with a **WhiteBoxTools-backed** ponding analyser, and **reordering** passes under **layer-order** constraints — operationalising baseline **§12.0.3** without changing pass geometry or earthwork quantities. **Audit mode** supports field-office review; **optimisation mode** trades modest extra deadhead for lower peak ponding. **Temporary drainage channels** and **matcher changes** are explicitly out of scope for this version. The feature defaults **off** and is controlled by **`--ponding-optimize`**, **`--ponding-audit`**, and the options in **§11**.
