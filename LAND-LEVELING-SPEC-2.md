# Land Leveling Haul Algorithm Specification — v2 **[0.0.1]**

**[0.0.2]** This document is the authoritative specification (revision 2) for the **land leveling haul algorithm**: a Python program that reads an OptiSurface-style **AGD** field file, plans a sequence of cut–haul–fill operations that bring the field to its target grade with minimum haul distance and time, and renders a **PNG** of the cut/fill map with haul-direction arrows, plus optional sequencing CSVs, a haul "heat map" PNG, and a leveling animation.

**[0.0.3]** This v2 supersedes **LAND-LEVELING-SPEC.md** (v0.0.1). It folds in the optimizations and outputs added during implementation: a **15 ft** default grid, **contiguous region-grown** cut/fill patches, the **true transit haul leg** (cut-end → fill-start), an optional **haul heading** bias, an **OptiHaul-matched** time model, **zone-level work sequencing**, per-pass **lat/lon CSV** export, a **pass-pairs heat-map** PNG, and a frame-by-frame **leveling animation**. Where v1 and the code disagreed, the code is treated as authoritative and this document is updated to match.

**[0.0.4]** The problem background and physical model are described in **LAND_LEVELING_PROBLEM.md**; this specification turns that description into an implementation-complete contract.

**[0.0.5]** Every normative paragraph carries a bold marker **`[<S>.<U>.<P>]`** where **`<S>`** is the section, **`<U>`** is the subsection (**`0`** for body text before the first **`###`**), and **`<P>`** is the 1-based paragraph index within that group.

**[0.0.6]** - **Project name:** Land Leveling Haul Algorithm
**[0.0.7]** - **Document type:** Application specification (algorithm + tool), revision 2
**[0.0.8]** - **Language / implementation:** Python 3.10+ (NumPy, Pillow; SciPy and networkx optional)
**[0.0.9]** - **Target platforms:** Windows, Linux, macOS (CPython)
**[0.0.10]** - **Scope:** From a single AGD file plus equipment and time parameters, produce a haul plan, a rendered PNG, and optional CSV/animation outputs. Field design (choosing the target plane) is an upstream concern and is **out of scope** except for the optional global vertical offset in **§8**.
**[0.0.11]** - **Related documents:** **LAND_LEVELING_PROBLEM.md** (problem statement), **README.md** (run guide), **ANDY-SPECSTYLEGUIDE.md** (authoring rules). AGD format and rendering conventions derive from **`fielddesign.py`**, **`extract_agd_heights.py`**, **`fieldcreator-checker.py`**, and **`render_existing_heights.py`**.

### 0.1 Specification revision history

**[0.1.1]** | Version | Date | Change |
|---|---|---|
| **0.0.1** | 2026-06-26 | Initial draft: inputs, grid model, six-stage algorithm, PNG output, CLI. |
| **2.0.0** | 2026-06-27 | v2: 15 ft default grid; LCY/BCY clarified; region-grown contiguous passes; true transit haul leg; optional haul heading bias; OptiHaul-matched time model (trapezoidal haul/return, return = forward, field efficiency); zone work-order sequencing + CSV + waypoint overlay; per-pass lat/lon pairs CSV; pass-pairs heat-map PNG; leveling animation (cutfill/topology). |
| **2.1.0** | 2026-06-27 | Self-containment patch for the core route algorithm: specified grid gap-fill (iterative 8-neighbour averaging, §7.2.1a); stated explicitly that the route plan is independent of the flow field/strips (§5.0.3a, §9.0.2, §11.0.1); added the matcher state and normative region-growing, sweep-ordering/haul-leg, and fill-seed/heading procedures (§11.0.1a–§11.0.3b); pinned routing construction + bounded 2-opt parameters (§12.0.4). |

**[0.1.2]** This document follows [semantic versioning](https://semver.org/): **MAJOR** = incompatible normative change, **MINOR** = backward-compatible addition, **PATCH** = clarification.

---
## 1. Purpose

**[1.0.1]** The land leveling haul algorithm shall compute, for one earthmoving machine, an ordered set of cut–haul–fill **passes** that bring every field **area** within a configured elevation **tolerance** of its target surface, while minimizing total **haul distance** and, secondarily, total **time**.

**[1.0.2]** The program shall accept its field input exclusively as an **AGD file** (**§6.1**) and shall accept equipment, time, and output parameters as described in **§6.2**–**§6.4** and **§15**.

**[1.0.3]** The program shall produce, as its primary deliverable, a **PNG image** containing a **cut/fill map** and overlaid **haul-direction arrows** (**§14**). It shall additionally produce, on request, the sequencing, CSV, heat-map, and animation outputs of **§14.4**–**§14.6** and **§12.1**.

**[1.0.4]** This specification is written for both human maintainers and agentic implementers; obligations use **shall**, **should**, **may** as defined in **ANDY-SPECSTYLEGUIDE.md §6**.

---

## 2. Design goals

**[2.0.1]** The implementation **shall provide**:

**[2.0.2]** 1. Deterministic, reproducible output for a given AGD file and parameter set (**§16.4**).
**[2.0.3]** 2. A single-file AGD reader that mirrors the parsing rules of the existing tools (**§6.1**).
**[2.0.4]** 3. A grid model derived from the AGD points (default **15 ft** areas), decoupled from blade width (**§7**).
**[2.0.5]** 4. Volume accounting in **banked cubic yards (BCY)** for cut supply, fill demand, and transported volume; swell applies only when converting scraper **LCY** capacity to a bank load limit; compaction via the cut/fill ratio (**§7.3**).
**[2.0.6]** 5. A global haul **flow field** computed by optimal transport, for haul arrows and strip visualization (**§9**).
**[2.0.7]** 6. **Strips** traced as streamlines of the flow field, from ridges to the cut/fill transition (**§10**).
**[2.0.8]** 7. **Passes** packed to the scraper load limit (20 LCY ≈ 15.4 BCY) under the 61 mm depth-per-pass limit, each cutting and filling a **compact, contiguous patch** (**§11**).
**[2.0.9]** 8. A pass **sequence** that minimizes non-productive (deadhead) return travel and respects layer ordering (**§12**).
**[2.0.10]** 9. A **time estimate** matching the OptiSurface OptiHaul methodology (trapezoidal haul/return speeds, return distance = forward distance, field efficiency) (**§13**).
**[2.0.11]** 10. A **PNG renderer** whose cut/fill palette, arrow glyphs, and image geometry match the existing AgGrade tools (**§14**).
**[2.0.12]** 11. Optional **zone work-order sequencing**, **per-pass lat/lon CSV**, a **pass-pairs heat-map PNG**, and a **leveling animation** (**§12.1**, **§14.4**–**§14.6**).
**[2.0.13]** 12. A documented **command-line interface** (**§15**).

## 3. Non-goals

**[3.0.1]** The implementation **shall not** attempt the following in this version:

**[3.0.2]** 1. Choosing or optimizing the **target surface** (plane fitting, slope/heading solve). The target is read from the AGD proposed elevations; see **§8** for the only permitted exception. (The haul **heading** in **§11.0.3b** biases *fill selection* only; it does not alter the target surface.)
**[3.0.3]** 2. Multi-machine fleet scheduling.
**[3.0.4]** 3. Modeling machine turning radius, traction, or fuel.
**[3.0.5]** 4. Reading any input from a database; the AGD path replaces the legacy SQLite path.
**[3.0.6]** 5. Producing GIS/shapefile or 3D outputs.

---

## 4. Terminology

### 4.1 Area (cell, bin)

**[4.1.1]** An **area** is one square grid cell of the field. The default cell is **15 ft × 15 ft** (**§7.2**); it is configurable. Each area carries one existing and one target elevation and is the atomic unit of cut, fill, and haul.

### 4.2 Cut and fill

**[4.2.1]** **Cut** is soil removed where existing exceeds target; **fill** is soil placed where existing is below target. Internally **`delta = existing − target`** is positive for cut, negative for fill.

### 4.3 BCY, LCY, swell, compaction

**[4.3.1]** **BCY** (banked cubic yards) is in-ground volume — the unit for mass balance, cut supply, fill demand, and transported volume. **LCY** (loose cubic yards) is the loose, in-scraper volume. The **swell factor** relates them: **`LCY = BCY × swell`** (equivalently **`BCY = LCY ÷ swell`**).

**[4.3.2]** Scraper capacity is specified in **LCY**. The per-pass bank load limit is **`scraper_LCY ÷ swell`** (e.g. 20 LCY ÷ 1.3 ≈ **15.4 BCY**). Swell shall **not** be applied to cut supply or fill demand.

**[4.3.3]** The **cut/fill ratio** (compaction) accounts for fill compacting when placed: creating one bank cubic yard of fill space requires delivering **`cutfill_ratio`** bank cubic yards of cut soil (default 1.2).

### 4.4 Pass, segment, haul leg

**[4.4.1]** A **pass** is one cut–haul–fill cycle. Its **cut segment** is the ordered set of cut cells (with depths) swept while loading; its **fill segment** is the ordered set of fill cells swept while dumping. The **haul leg** is the straight-line transit from the cut-segment **end** to the fill-segment **start** (**§11.0.3**).

### 4.5 Strip

**[4.5.1]** A **strip** is a continuous centerline streamline of the flow field, used for haul-direction visualization. Strips may curve and are not restricted to cardinal directions.

### 4.6 Flow field and haul arrows

**[4.6.1]** The **flow field** is a per-area unit direction (plus magnitude) giving where soil should move. A **haul arrow** is a rendered glyph of that direction (**§14.2**).

### 4.7 Ridge and transition

**[4.7.1]** A **ridge** is a local high of `delta` (a cut source). The **transition** is where existing crosses target (cut switches to fill).

### 4.8 Tolerance

**[4.8.1]** The **tolerance** is the maximum absolute deviation from target for an area to count as finished (default 10 mm).

### 4.9 Heading

**[4.9.1]** The **haul heading** is a compass bearing (0 = N, 90 = E) along which soil is preferentially moved. Default is the design surface's main down-slope, auto-detected from the target gradient (**§11.0.3b**).

### 4.10 Zone

**[4.10.1]** A **zone** is a spatial cluster of cut work used only for the human-readable rough **work order** (**§12.1**). Zones are a reporting summary of the per-pass route, not a separate operational constraint.

---
## 5. High-level architecture

**[5.0.1]** The program is a pipeline; each stage consumes the previous stage's output and is independently testable.

**[5.0.2]**

```text
AGD file ──► [1] Parse + grid (15 ft)          agd.py, grid.py   §6, §7
          ──► [2] Volume balance (check/adjust)  balance.py        §8
          ──► [3] Flow field (optimal transport) flowfield.py      §9
          ──► [4] Strips (streamlines)           strips.py         §10
          ──► [5] Passes (≈15.4 BCY / 61 mm)     passes.py         §11
          ──► [6] Sequence + route               routing.py        §12
          ──► [7] Time estimate                  timemodel.py      §13
          ──► PNG: cut/fill map + haul arrows    render.py         §14
          ──► (optional) zones / CSVs / heat map sequence.py, render.py  §12.1, §14.4–14.5
          ──► (optional) leveling animation      animate.py        §14.6
```

**[5.0.3]** Equipment (**§6.2**) and time/motion (**§6.3**) parameters feed stages [5]–[7]. The cut/fill map depends only on stages [1]–[2]; the haul arrows depend on stage [3].

**[5.0.3a]** *The optimized haul-route plan does not depend on the flow field or strips.* Stages [3] (flow field, **§9**) and [4] (strips, **§10**) exist **only** for haul-direction visualization (the arrows and curving strips in the PNG). The pass plan in stage [5] (**§11**) is computed directly from the per-area cut/fill geometry — a nearest-fill anchor plus distance-ordered region growing — and reads neither the flow field nor the strips. An implementer who only wants the haul routes, sequence, and time (e.g. the CSV pairs, **§14.4**) may therefore **skip §9 and §10 entirely**; the routes are unchanged. The flow field/strips are retained as a separate, optional visualization path.

**[5.0.4]** Stages [1]–[2] (plus optional [3] for arrows) fully determine the base PNG and shall be runnable without stages [4]–[7]; conversely the pass/sequence/time plan (stages [5]–[7]) shall be runnable without stages [3]–[4]. The pass list, time, and all optional outputs are additive. The CLI exposes the PNG-only path as `--no-plan`.

---

## 6. Inputs

### 6.1 AGD file format (normative)

**[6.1.1]** The AGD file is a UTF-8, comma-separated text file. The first line is a header and shall be skipped. Canonical header:

**[6.1.2]**

```text
Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments
```

**[6.1.3]** Each subsequent non-empty line has these fields by index:

**[6.1.4]**

```text
[0] Latitude            decimal degrees (WGS-84)
[1] Longitude           decimal degrees (WGS-84)
[2] Elevation Existing  metres
[3] Elevation Proposed  metres (target surface)
[4] CutFill             metres, on disk = Proposed - Existing
[5] Code                row classification (see below)
[6] Comments            free text (ignored)
```

**[6.1.5]** Field semantics and parsing rules (mirroring **`extract_agd_heights.py`** and **`fielddesign.py`**):

**[6.1.6]** - The reader shall split each line on commas and strip surrounding whitespace from every field.
**[6.1.7]** - A row is a **benchmark** if its **Code** (case-insensitive) contains **`MB`** or **`BM`**. Benchmark rows shall be skipped for grid construction.
**[6.1.8]** - A row is a **boundary** point if its **Code** is **`2PER`**; otherwise a data row is an **interior** grid point (typically **`3GRD`**).
**[6.1.9]** - A data row shall be used only if both **Elevation Existing** and **Elevation Proposed** are present and non-zero; rows failing this shall be skipped (with a warning).
**[6.1.10]** - Elevations are in **metres**; latitude/longitude are decimal degrees.

**[6.1.11]** Sign convention (normative): on disk **`CutFill = Proposed − Existing`**, so negative CutFill is a **cut** and positive is a **fill**. Internally the algorithm shall use **`delta = Existing − Proposed`** (positive = cut). The reader shall recompute **`delta`** from columns [2] and [3] and shall not trust column [4].

**[6.1.12]** Boundary handling: if **`2PER`** rows are present they define the field perimeter polygon; areas whose centers fall outside it shall be invalid and excluded everywhere. If no **`2PER`** rows are present, the field mask shall be inferred from the observed points (row/column span fill, hole fill, largest connected component), consistent with **`fielddesign.py`**.

**[6.1.13]** The reader shall also expose the inverse projection (UTM → lat/lon, `latlon_from_utm`) and the field's UTM **zone** and **hemisphere**, so downstream stages can emit geographic coordinates in the CSV outputs (**§14.4**).

### 6.2 Equipment specifications (normative)

**[6.2.1]** Defaults match **LAND_LEVELING_PROBLEM.md §4** and shall be overridable on the command line.

**[6.2.2]** | Parameter | Default | Meaning |
|---|---|---|
| **Scraper capacity** | 20 LCY | Loose volume per pass; bank limit = 20 ÷ swell ≈ 15.4 BCY. |
| **Swell factor** | 1.3 | `LCY = BCY × 1.3` (`BCY = LCY ÷ 1.3`). |
| **Cut/fill ratio** | 1.2 | Bank cut soil delivered per bank cubic yard of fill space (compaction). Alias: `--compaction`. |
| **Max depth per pass** | 61 mm | Maximum cut or fill depth in one area for one pass (0.2 ft). |
| **Blade width** | 2271 mm | Cutting/filling blade width (informative; not the grid size). |
| **Blade height control** | 10 Hz | Blade height update rate (informative). |
| **Haul heading** | auto | Compass bearing soil is hauled along; default is the design surface's main down-slope. Biases fill selection (**§11.0.3b**). |
| **Heading strength** | 0.5 | 0–0.95; how strongly fill selection follows the heading (0 = off). |

### 6.3 Time and motion parameters (normative)

**[6.3.1]** The time model (**§13**) consumes these; defaults apply when not supplied and shall be overridable (**§15**).

**[6.3.2]** | Parameter | Default | Unit | Meaning |
|---|---|---|---|
| **Loading speed** | 4 | MPH | Constant travel speed while cutting. |
| **Haul speed** | 10 | MPH | Maximum travel speed from cut to fill (trapezoidal profile). |
| **Dumping speed** | 5 | MPH | Constant travel speed while filling. |
| **Return speed** | 13 | MPH | Maximum deadhead return speed (trapezoidal profile). |
| **Acceleration time** | 12 | s | Time to reach max speed on a haul/return move. |
| **Deceleration time** | 12 | s | Time to stop at the end of a haul/return move. |
| **Field efficiency** | 0.8 | – | Productive fraction of total time; "Other" time = total × (1 − eff). |

### 6.4 Configuration and output parameters (normative)

**[6.4.1]** | Parameter | Default | Meaning |
|---|---|---|
| **Grid size** | 15 ft | Edge length of each square area (**§7.2**). |
| **Tolerance** | 10 mm | Finished-area elevation tolerance (**§4.8**). |
| **Pixels per bin** | 50 | PNG pixels per area edge for the main plan PNG (**§14.3**). |
| **Arrow stride** | 2 | Draw a haul arrow every N areas (0 disables). |
| **Transport mode** | `poisson` | `poisson` (L²) or `mincostflow` (L¹) flow field (**§9**). |
| **Balance mode** | `check` | `check` (warn) or `adjust` (re-level target, **§8**). |
| **Zones** | 8 | Number of cut zones for the rough work order (0 disables) (**§12.1**). |
| **Seed** | 1234 | Random seed for deterministic zone clustering (**§16.4**). |

**[6.4.2]** The optional output toggles (sequence CSV, pairs CSV, pairs PNG, animation directory/GIF, animation mode/stride/pixels-per-bin) are defined with the CLI in **§15** and the outputs in **§12.1** and **§14.4**–**§14.6**.

---

## 7. Coordinate and grid model

### 7.1 Projection

**[7.1.1]** The reader shall project each AGD point's lat/lon to a local planar (UTM, WGS-84) system in metres, with the field's south-west corner as the local origin (`utm_from_latlon`). A field crossing a UTM zone boundary shall be rejected with a clear error.

**[7.1.2]** The inverse projection (`latlon_from_utm`) shall be provided for CSV outputs (**§14.4**) and shall be consistent with **§14.3** geometry.

### 7.2 Binning to the grid

**[7.2.1]** The field bounding rectangle shall be partitioned into square areas of edge **`grid_size`** (default 15 ft = 4.572 m). The 15 ft default balances resolution against speed; the grid size is configurable (**§6.4**) and coarser grids (e.g. 30 ft) run faster. Each AGD interior point is assigned to its containing area (floor of `(coord − min) / grid_size`, clamped to the grid); an area's existing and target elevations are the **mean** of the points it contains. Areas outside the mask are invalid.

**[7.2.1a]** *Gap filling (normative).* Areas that are inside the field mask but contain no points shall be filled by **iterative nearest-neighbour averaging**, applied to the existing and target grids in parallel:

```text
repeat up to 2 * max(width, height) times, until no cell is still pending:
    for each pending (masked, still-empty) cell:
        let n = its 8-connected neighbours that already have a finite value
        if n is non-empty:
            cell.existing = mean(existing of n)
            cell.target   = mean(target   of n)
            mark cell filled (no longer pending)
```

Each iteration uses only values that were finite at the **start** of that iteration (the four cardinal and four diagonal neighbours), so the fill is a deterministic flood inward from the observed cells. A cell that never acquires a finite neighbour (e.g. an isolated masked region) remains invalid.

**[7.2.2]** Each area's existing and target elevations shall be treated as **uniform over the whole cell** (no intra-cell slope). All volume, depth, cut/fill, and rendering computations use these per-cell constants.

**[7.2.3]** The grid shall be indexed by integer `(bx, by)` with `bx` increasing east and `by` increasing north. Image row order is defined in **§14.3**. The grid shall expose a `cell_center_xy(bx, by)` helper returning the area-center world coordinates used throughout stages [5]–[7] and the CSV/animation outputs.

**[7.2.4]** Blade width shall not affect the grid size; each area is atomic for cut, fill, and haul.

### 7.3 Cell quantities and BCY conversion (normative)

**[7.3.1]** For each valid area, let `delta = existing − target` (m, positive = cut) and `A = grid_size²` (m²). `M3_TO_CY = 1.30795061931439` converts m³ to yd³.

**[7.3.2]** Cut areas (delta > 0):

**[7.3.3]**

```text
in_place_m3 = delta * A
supply_BCY  = in_place_m3 * M3_TO_CY      # bank cut volume available (BCY)
```

The bank cut volume is BCY directly — swell does **not** enter cut supply or fill demand. Swell applies **only** to the scraper LCY → bank limit conversion: `load_limit_BCY = scraper_LCY ÷ swell`.

**[7.3.4]** Fill areas (delta < 0): bank cut volume to deliver to raise the surface by `-delta`, with compaction via the cut/fill ratio:

**[7.3.5]**

```text
fill_space_m3 = (-delta) * A
demand_BCY    = fill_space_m3 * M3_TO_CY * cutfill_ratio   # bank cut volume to deliver
```

**[7.3.6]** All transport (stages [3]–[6]) shall be expressed in **BCY**. The 61 mm depth-per-pass limit shall be applied per area as `ceil(|delta| / max_depth)` layers, with a partial final layer allowed.

**[7.3.7]** These conversions shall be pure functions with unit tests (**§17**), including the LCY → BCY scraper conversion (`lcy_to_bcy`).

---
## 8. Stage 1 — Volume balance

**[8.0.1]** Because the target is supplied by the AGD file, total cut and fill (BCY) are normally consistent. The implementation shall still compute global `Σ supply_BCY` and `Σ demand_BCY` and report the relative imbalance.

**[8.0.2]** In **`balance_mode = check`** (default) it shall warn if the relative imbalance exceeds a small threshold (default 2%) and continue.

**[8.0.3]** In **`balance_mode = adjust`** it **may** apply a single global vertical offset to the target to drive `Σ supply_BCY` toward `Σ demand_BCY` (bisection on the offset), reporting the chosen offset. Tilting/re-fitting the plane is out of scope (**§3**).

---

## 9. Stage 2 — Global flow field (optimal transport)

**[9.0.1]** The flow field determines haul directions for the arrows and strips. The implementation shall support two modes via **`transport_mode`**.

**[9.0.2]** *Visualization only.* This stage and the strips of **§10** are **not** inputs to the pass plan (**§5.0.3a**); they produce the haul-direction overlay. An implementation that needs only the optimized routes/time may omit this entire section.

### 9.1 Poisson potential mode (default)

**[9.1.1]** Define a signed source density `s` over valid areas: `s = supply_BCY` for cut, `s = −demand_BCY` for fill. Solve

**[9.1.2]**

```text
laplacian(phi) = s      with Neumann (zero-flux) boundary on the field mask
```

**[9.1.3]** and set `F = −grad(phi)`; soil flows from high `phi` (ridges) toward low `phi` (sinks). The system is assembled over valid areas only and solved with a sparse solver (SciPy if available, else a NumPy iterative fallback). The potential **may** be smoothed before differentiation to yield cleaner streamlines (**§10**).

**[9.1.4]** This mode minimizes a quadratic (L²) transport cost; it is fast and yields smooth, curving haul directions.

### 9.2 Min-cost flow mode

**[9.2.1]** When **`transport_mode = mincostflow`**, solve the balanced transportation problem minimizing `Σ f·d` on the grid graph for the distance-optimal (L¹) flow; the net per-area flow direction is the flow field. If the optional dependency (**networkx**) is unavailable, the implementation shall fall back to `poisson` with a warning.

### 9.3 Flow field output

**[9.3.1]** The flow field shall be a per-area unit direction plus magnitude. Areas with negligible magnitude (at grade) shall be marked as having no arrow. The flow field feeds the haul-arrow overlay (**§14.2**) and strip tracing (**§10**).

---

## 10. Stage 3 — Strip generation (visualization)

**[10.0.1]** Strips shall be traced as streamlines of the flow field, starting at ridges (local maxima of `phi` or highest-`delta` cut areas) and integrating along `F` with bilinear interpolation so paths curve smoothly. A smoothed potential **may** be used for tracing.

**[10.0.2]** A streamline shall be followed across the cut region, through the transition, and into the fill region, capturing both source and sink of a haul.

**[10.0.3]** Tracing shall terminate when a streamline leaves the mask, reaches an area with no remaining fill capacity, or forms a closed loop.

**[10.0.4]** Strips are used for haul-direction visualization and as informative seeds; the authoritative volume plan is produced by the pass matcher (**§11**), which does not require strips to pack full loads.

---

## 11. Stage 4 — Pass generation

**[11.0.1]** Passes shall accumulate volume in BCY subject to **§6.2**, packing each pass to the scraper load limit (`scraper_LCY ÷ swell` ≈ 15.4 BCY) under the 61 mm depth-per-pass cap. The volume plan is produced by the **shortest-haul-first, contiguous patch matcher** below, operating **solely** on the per-area cut/fill ledger (`delta` and the BCY conversions of **§7.3**). It does not consume the flow field (**§9**) or strips (**§10**); this section together with **§7** is sufficient to reproduce the haul-route plan.

**[11.0.1a]** *State (normative).* The matcher maintains two non-negative per-area ledgers in metres of remaining depth: `remaining_cut[by,bx] = max(delta, 0)` and `remaining_fill[by,bx] = max(−delta, 0)` over valid areas (zero elsewhere). The per-metre BCY rates are `cut_bcy_per_m = A · M3_TO_CY` and `fill_bcy_per_m = A · M3_TO_CY · cutfill_ratio` (**§7.3**). A cell is "still holds material" when its remaining depth exceeds the tolerance (**§4.8**). Passes are emitted in a loop until no cut **or** no fill cell remains above tolerance (**§11.0.6**).

**[11.0.2]** *Anchor.* Passes shall be generated **shortest-haul first**. At each step, over all cut cells still above tolerance, the **anchor** is the cut cell whose nearest still-above-tolerance fill cell is closest (Euclidean distance between area centers). Ties shall be broken deterministically (e.g. lowest flattened index). The per-pass load cap is `load_cap = min(load_limit_BCY, remaining_fill_total_BCY)`, where `remaining_fill_total_BCY` is the sum over all fill cells above tolerance of `remaining_fill · fill_bcy_per_m`; if `load_cap ≤ 0` the loop terminates.

**[11.0.2a]** *Region growing (normative).* From a seed cell the machine **region-grows a compact, contiguous patch** of material to a `target_bcy`, used for both the cut patch (seed = anchor, `target_bcy = load_cap`, `remaining_cut`, `cut_bcy_per_m`) and the fill patch (**§11.0.3**). The growth is a best-first flood keyed by squared Euclidean distance from the seed:

```text
visited = {}; used = []; acc = 0
push seed with priority 0 (ties broken by insertion order)
while frontier not empty and acc < target_bcy:
    pop cell c with smallest (dist2_to_seed, insertion_order)
    if c in visited: continue
    visited.add(c)
    rem = remaining_depth[c]
    if rem <= tolerance:            # empty: do NOT consume and do NOT expand through c
        continue
    depth = min(max_depth_m, rem)   # one 61 mm layer cap
    bcy = depth * per_m
    if acc + bcy > target_bcy:       # final cell takes only the shortfall
        bcy = target_bcy - acc; depth = bcy / per_m
    remaining_depth[c] -= depth; acc += bcy; used.append((c, depth))
    for each of the 8 neighbours n in-bounds and not visited:
        push n with priority (||n - seed||^2, insertion_order)
```

Because empty cells are not expanded through, the consumed set stays a single connected blob around the seed, which keeps the swept loading/dumping length near its minimum. `used` (cells with depths) and the achieved `acc` (BCY) are returned. The cut patch and depths define the **cut segment**.

**[11.0.3]** *Fill patch and haul leg.* The carried load (`acc` from the cut grow) is deposited into a compact, contiguous **fill patch** region-grown by **§11.0.2a** from a fill seed (**§11.0.3b**), with `target_bcy = carried load` and the fill ledger/rate. If no fill cell can be grown, the cut depths shall be returned to `remaining_cut` and the loop terminates. Any **placed < carried** shortfall (`leftover`) shall be returned to the most-recently-consumed cut cells (iterate the cut `used` list in reverse, refunding `min(leftover, depth·cut_bcy_per_m)` to each) so **volume is conserved exactly**.

**[11.0.3a]** *Sweep ordering and haul leg (normative).* The two segments shall be ordered so the haul is the true transit between them, via the nearest-neighbour chain `_nn_chain` (start at a chosen cell, repeatedly append the nearest not-yet-used cell):

```text
fill_centroid = mean of fill-patch cell centers
cut_start  = cut cell FARTHEST from fill_centroid        # so the sweep ends toward the fill
cut_chain  = nn_chain(cut cells, start = cut_start)
cut_end    = center of last cell in cut_chain
fill_start = fill cell NEAREST to cut_end                # dumping begins where hauling ends
fill_chain = nn_chain(fill cells, start = fill_start)
haul_distance = euclidean(cut_end, center of fill_start)
```

Centroid-to-centroid haul (which double-counts the sweeps) shall **not** be used. The loading distance is the swept length of `cut_chain`, the dumping distance the swept length of `fill_chain` (**§13**).

**[11.0.3b]** *Fill-seed selection and heading (optional, §6.2).* Without a heading (or `strength = 0`), the fill seed is simply the fill cell nearest the cut-patch centroid. When a haul **heading** is supplied (or auto-detected as the design surface's main down-slope — `estimate_slope_heading`: the bearing of `−∇target` averaged over valid areas, as a compass bearing 0 = N, 90 = E), the seed is chosen from the **200 nearest** fill cells to the cut centroid (or all fill cells if fewer), re-ranked by the anisotropic cost `dist × (1 − strength × cos θ)`, where `θ` is the angle between the cut→candidate direction and the heading unit vector and `strength ∈ [0, 0.95]`. The minimum-cost candidate is the seed. This emulates the lane-oriented hauling of tools such as OptiSurface. The realised haul distance can never fall below the field's optimal-transport minimum; heading mainly organises the work and trims sweep/haul overhead rather than beating the transport lower bound.

**[11.0.4]** *Layering.* Because 61 mm caps per-area depth, deep cuts/fills need multiple passes: a cell needing depth `d` contributes about `ceil(d / 61 mm)` passes over successive layers, with the transition receding as boundary cells reach grade. The final layer of an area may be a partial pass (**§7.3**).

**[11.0.5]** Each pass shall record: ordered cut cells with depths, ordered fill cells with depths, load in BCY, the haul distance, and the cut-segment start/end and fill-segment start/end world coordinates (used by sequencing, the time model, and the CSV/heat-map outputs).

**[11.0.6]** The implementation shall report the number of passes and any **unmatched** cut or fill BCY remaining below tolerance at termination.

---
## 12. Stage 5 — Sequencing and routing

**[12.0.1]** The passes from **§11** form a set; the implementation shall order them to minimize total **non-productive (deadhead) return travel** — the distance from the end of one pass's fill segment to the start of the next pass's cut segment.

**[12.0.2]** Sequencing shall respect **layer ordering**: a deeper layer of an area shall not be worked before the shallower layer above it. Because passes are generated layer-by-layer (shallower first), this holds by construction.

**[12.0.3]** *Anti-dam ordering (informative).* Ordering should favor completing downhill flow first to avoid transient berms that could pond water; this is a soft preference, not a hard constraint in this version.

**[12.0.4]** The ordering shall be a constructive heuristic followed by bounded local improvement. The deadhead metric between an earlier pass `a` and a later pass `b` is `deadhead(a, b) = euclidean(a.fill_end, b.cut_start)`. Construction is **nearest-neighbour starting from pass index 0** (deterministic): repeatedly append the unvisited pass minimizing `deadhead(current, ·)`. Then a **bounded 2-opt** pass improves the order: it is applied **only when the pass count ≤ 1500** (above that the construction order is kept, to stay fast and deterministic), runs at most **30 sweeps**, and accepts a segment reversal only when it strictly reduces the summed deadhead (improvement threshold `1e-6` m), taking the first improving move in each sweep. Exact optimization is not required. The route shall expose the ordered passes, the original indices, and the total deadhead distance.

### 12.1 Zone work-order sequencing (optional output)

**[12.1.1]** On request (`--zones N`, default 8; 0 disables), the implementation shall summarise the fine-grained route into a small number of spatial **zones** for a human-readable rough work order. This is a reporting summary; it does not change the route or the plan.

**[12.1.2]** The cut-patch centroids of the routed passes shall be clustered into `k = min(N, n_passes)` zones with a **deterministic k-means++** (seeded from **§6.4**). Each zone shall report: a stable `zone_id`; a `work_order` rank (1 = worked earliest); load count; moved BCY; centroid in local XY **and** lat/lon (via `latlon_from_utm`); the first/last/mean pass index over which it is worked; and a rough compass position within the field.

**[12.1.3]** Zones shall be ordered by their **load-weighted mean pass index** (`mean_pass`), so `work_order` reflects when each zone is mostly worked. Because the route interleaves passes across zones to minimize travel, the work order is an *average* ordering, not a strict "finish zone 1 before starting zone 2" sequence; the reported first/last pass ranges make the overlap explicit.

**[12.1.4]** For overlay rendering (**§14.4**) each cut cell shall be assigned to the single zone that removed the most depth there, and the **exact cell-boundary outline** (jagged perimeter of that zone's cells) shall be computed as world-metre segments, so zone footprints partition the field without overlap.

**[12.1.5]** The zone work order shall be writable to a **sequence CSV** (**§14.4**) and drawn as numbered waypoints + outlines on the plan PNG (**§14.4**).

---

## 13. Stage 6 — Time model (OptiHaul-matched)

**[13.0.1]** Each pass comprises four motions — **loading** (cutting), **hauling**, **dumping** (filling), and **returning** — using **§6.3** parameters. The model matches the OptiSurface OptiHaul methodology.

**[13.0.2]** *Distances.* Per pass: the loading distance is the swept length of the cut segment, the dumping distance is the swept length of the fill segment, and the hauling distance is the haul leg (**§11.0.3**). The **returning distance equals the loading + hauling + dumping distance** — every forward trip has an equal-length return.

**[13.0.3]** *Speeds.* Loading and dumping run at their **constant** configured speeds. Hauling and returning use a **trapezoidal speed profile**: the machine accelerates for `acceleration_time` and decelerates for `deceleration_time`, reducing the *average* speed below the configured maximum. For a move of length `L` at maximum speed `v`:

```text
d_acc = 0.5 * v * acceleration_time
d_dec = 0.5 * v * deceleration_time
if L >= d_acc + d_dec:
    time = acceleration_time + deceleration_time + (L - d_acc - d_dec) / v
else:                                   # triangular: peak speed vp < v
    a, b = v / acceleration_time, v / deceleration_time
    vp   = sqrt(2 * L / (1/a + 1/b))
    time = vp / a + vp / b
```

The return time is computed by applying this profile to each forward segment length (cut, haul, fill) at the **return** max speed.

**[13.0.4]** Speeds shall be converted MPH → m/s. Loading/dumping travel times (`distance / speed`) inherently include cutting/placing time; there is no separate per-BCY term.

**[13.0.5]** *Field efficiency.* Productive time = loading + hauling + dumping + returning. **Total time = productive ÷ field_efficiency**; the remainder is reported as **"Other"** time. The implementation shall report total time, the four distances, the achieved average haul and return speeds, and the moved volume (BCY).

---
## 14. Outputs

**[14.0.1]** The program shall always write one **plan PNG** (cut/fill map + haul arrows, **§14.1**–**§14.3**). On request it shall also write the zone overlay (**§14.4**), CSV exports (**§14.4**), a pass-pairs heat-map PNG (**§14.5**), and a leveling animation (**§14.6**). Rendering conventions match the existing AgGrade tools so outputs are visually comparable.

### 14.1 Cut/fill map

**[14.1.1]** Each valid area shall be filled with the band color for `diff = existing − target` (m), using the nine-band palette and thresholds from **`fielddesign.py`**:

**[14.1.2]**

```text
diff <  -0.80  ->  violet     #800080
diff <  -0.50  ->  indigo     #9A31FF
diff <  -0.05  ->  blue       #0066FF
diff <  -0.01  ->  cyan       #36F5F5
diff <=  0.01  ->  green      #00CD00   (on grade)
diff <=  0.05  ->  yellow     #FFFF00
diff <=  0.50  ->  orange     #FF8000
diff <=  0.80  ->  red        #FF0000
diff >   0.80  ->  dark red   #B40000
```

**[14.1.3]** Negative `diff` (fill) maps to cool colors, positive `diff` (cut) to warm colors. Areas outside the mask are transparent (alpha 0). A fast vectorised renderer (`diff_to_image`) shall be available for the many animation frames (**§14.6**).

### 14.2 Haul arrows

**[14.2.1]** Haul-direction arrows shall be hollow (outline-only) triangles pointing in the flow direction, using the glyph from **`render_existing_heights.py`**. Direction is in compass degrees (0 = N, 90 = E), converted to image vector `(dx, dy) = (sin θ, −cos θ)`.

**[14.2.2]** Arrows shall be sampled on a regular sub-grid (`--arrow-stride`), drawn on top of the cut/fill fill, in black; areas with no arrow (at grade) are skipped.

### 14.3 Image layout and geometry

**[14.3.1]** The plan image shall be `grid_width × pixels_per_bin` wide and `grid_height × pixels_per_bin` tall (`pixels_per_bin` default 50). Origin is top-left; image Y is flipped relative to grid Y so larger `by` is drawn higher: area `(bx, by)` occupies the block with top-left `x0 = bx·ppb`, `y0 = (grid_height − 1 − by)·ppb`; arrow centers at `px = (bx + 0.5)·ppb`, `py = (grid_height − 0.5 − by)·ppb`. A `_world_to_pixel` helper shall map world (easting, northing) to pixels with the same flip, for overlays (**§14.4**) and the heat map (**§14.5**).

### 14.4 Zone waypoints, outlines, and CSV exports (optional)

**[14.4.1]** When zones are computed (**§12.1**), the implementation shall overlay on the plan PNG, beneath/above the cut/fill fill respectively: each zone's **exact cell-boundary outline** (white halo under a black line) and a numbered **waypoint** marker at the zone centroid, connected in `work_order` order. Marker radius and font shall scale with the image's short side so numbers stay legible at any resolution.

**[14.4.2]** *Sequence CSV* (`--sequence-csv`, default `<out>_cut_sequence.csv` when zones are on): one row per zone with `work_order, zone_id, compass, loads, moved_bcy, easting_m, northing_m, lat, lon, first_pass, last_pass, mean_pass`.

**[14.4.3]** *Pairs CSV* (`--pairs-csv`, default `<out>_cut_fill_pairs.csv`): every pass, in optimized route order, with `order, load_bcy, haul_m` and the start/end **lat/lon** of both the cut segment and the fill segment (`cut_start_*`, `cut_end_*`, `fill_start_*`, `fill_end_*`), via `latlon_from_utm`.

### 14.5 Pass-pairs heat map (optional)

**[14.5.1]** On request (`--pairs-png`, default `<out>_cut_fill_pairs.png`) the implementation shall render the cut/fill map overlaid with **one line per cut-fill pair** along its haul leg (cut end → fill start), producing a "heat map" of haul corridors.

**[14.5.2]** The view shall render at a modest resolution (lines several pixels wide via a disk stamp) so individual lines are visible; the underlying cut/fill map shall be faded so the lines stand out. Line density shall accumulate (each pixel counted once per line) and map through a **hot ramp** (yellow for a single load → orange → deep red where many overlap), using a robust percentile scale and a minimum alpha floor so even isolated lines are clearly visible.

### 14.6 Leveling animation (optional)

**[14.6.1]** On request the implementation shall reconstruct the field surface as each pass is applied in route order and render a frame every `stride` applied passes (`--animate-stride`, default 10), at `--animate-ppb` pixels per area (default 8). Frame 0 is the initial surface; the final pass always produces a frame. Frames may be written as PNGs to a directory (`--animate-dir`) and/or assembled into an animated **GIF** (`--animate-gif`, default `<out>_leveling.gif`).

**[14.6.2]** Two render modes (`--animate-mode`): **`cutfill`** (default) renders the cut/fill map (which turns green as areas reach grade) via `diff_to_image`; **`topology`** renders the absolute elevation on a terrain ramp via `elevation_to_image`, with the color scale fixed to the initial surface range so the terrain visibly flattens toward grade.

**[14.6.3]** Any assembled GIF shall **hold its final frame for 4 seconds** so the end result is visible before the loop restarts; intermediate frames use the normal cadence.

---
## 15. Command-line interface

**[15.0.1]** The program shall expose a CLI (`python -m landlevel.cli`) with at least the following options:

**[15.0.2]**

```text
--agd PATH                 (required) input AGD file
--out PATH                 (required) output PNG path (base name for derived outputs)

# Field model & volumes
--grid-size-ft FLOAT          default 15
--tolerance-mm FLOAT          default 10
--scraper-capacity-lcy FLOAT  default 20    (bank limit = value / swell)
--swell-factor FLOAT          default 1.3   (LCY = BCY * swell)
--cutfill-ratio FLOAT         default 1.2   (alias: --compaction)
--max-depth-mm FLOAT          default 61
--blade-width-mm FLOAT        default 2271

# Haul behaviour
--heading-deg FLOAT           default auto  (design-surface main slope)
--heading-strength FLOAT      default 0.5   (0 disables heading bias)
--transport-mode {poisson,mincostflow}   default poisson
--balance-mode {check,adjust}            default check

# Time model
--field-efficiency FLOAT      default 0.8
--loading-speed-mph FLOAT     default 4
--haul-speed-mph FLOAT        default 10
--dumping-speed-mph FLOAT     default 5
--return-speed-mph FLOAT      default 13
--accel-time-s FLOAT          default 12
--decel-time-s FLOAT          default 12

# PNG rendering
--pixels-per-bin INT          default 50
--arrow-stride INT            default 2     (0 disables arrows)
--no-plan                     render cut/fill PNG from stages 1-3 only

# Sequencing & extra outputs
--zones INT                   default 8     (0 disables)
--sequence-csv PATH           default <out>_cut_sequence.csv (when zones on)
--pairs-csv [PATH]            per-pass lat/lon pairs; flag alone -> <out>_cut_fill_pairs.csv
--pairs-png [PATH]            haul heat map; flag alone -> <out>_cut_fill_pairs.png

# Animation
--animate-dir DIR             write PNG frames into this directory
--animate-gif [PATH]          assemble a GIF; flag alone -> <out>_leveling.gif
--animate-mode {cutfill,topology}  default cutfill
--animate-stride INT          default 10
--animate-ppb INT             default 8

# Misc
--seed INT                    default 1234
--version
```

**[15.0.3]** On success the program shall print an **OptiHaul-style report** (moved volume, pass count, haul heading, the four distances and achieved speeds, and the time breakdown incl. "Other" and TOTAL), write the plan PNG, and write any requested CSV/PNG/animation outputs, reporting each path. Invalid arguments shall produce a clear error and a non-zero exit code.

**[15.0.4]** Flags that derive a path from `--out` (`--pairs-csv`, `--pairs-png`, `--animate-gif`, and `--sequence-csv` when zones are on) shall, when passed without an explicit path, name the file from the `--out` stem.

---

## 16. Reference implementation, build, and deliverables

### 16.1 Module layout

**[16.1.1]** Each pipeline stage shall be a separate, importable module:

**[16.1.2]**

```text
landlevel/
  __init__.py     # constants/defaults (grid size, swell, capacities, speeds)
  agd.py          # §6.1 AGD reader, §7.1 projection (utm_from_latlon, latlon_from_utm)
  grid.py         # §7.2 binning, §7.3 BCY conversions, cell_center_xy
  balance.py      # §8 volume balance
  flowfield.py    # §9 optimal transport (poisson + mincostflow)
  strips.py       # §10 streamline tracing
  passes.py       # §11 contiguous patch matcher, heading, 61 mm layering
  routing.py      # §12 sequencing (NN + bounded 2-opt)
  sequence.py     # §12.1 zone work order + CSV exports
  timemodel.py    # §13 OptiHaul-matched time model
  render.py       # §14 PNG (cut/fill + arrows), zone overlay, heat map, frame renderers
  animate.py      # §14.6 leveling animation
  cli.py          # §15 command-line entry point
tools/
  number_spec_paragraphs.py
```

**[16.1.3]** The renderer shall reuse the palette, arrow glyph, and geometry conventions of the existing tools.

### 16.2 Dependencies

**[16.2.1]** Core functionality shall depend only on **NumPy** and **Pillow** plus the standard library. **SciPy** (optional) provides a fast sparse Poisson solver and a KD-tree for nearest-fill queries; absent it, NumPy fallbacks shall be used. **networkx** (optional) enables `mincostflow`; absent it, that mode falls back to `poisson` with a warning. Dependencies and pinned versions shall be listed in **`requirements.txt`**.

### 16.3 Build layout

**[16.3.1]** The package shall run as `python -m landlevel.cli`. Source files shall be UTF-8 without BOM.

### 16.4 Determinism

**[16.4.1]** Given identical inputs and parameters, the program shall produce byte-identical PNG output. Randomized heuristics (zone k-means++, any routing restarts) shall be seeded from a fixed default seed overridable via `--seed`.

---

## 17. Testing

**[17.0.1]** The implementation shall include automated tests on small synthetic fields (no external data):

**[17.0.2]** 1. **AGD round-trip** — a synthetic AGD parses to expected areas, codes, and `delta` signs (**§6.1**).
**[17.0.3]** 2. **BCY conversions** — cut/fill volume functions and the LCY → BCY scraper conversion match **§7.3** on hand-checked values.
**[17.0.4]** 3. **Mass balance** — a balanced field reports near-zero imbalance; an unbalanced one is flagged (**§8**).
**[17.0.5]** 4. **Flow field** — for a one-ridge-one-valley field, Poisson flow points ridge → valley (**§9**).
**[17.0.6]** 5. **Pass limits** — generated passes never exceed the scraper load limit (≈ 15.4 BCY) or 61 mm per area per pass, and volume is conserved per pass (**§11**).
**[17.0.7]** 6. **Render geometry** — a known area maps to the expected pixel block and arrow center (**§14.3**).

**[17.0.8]** The default field set shall include at least one field with explicit **`2PER`** boundary rows and one without, to exercise both masking paths (**§6.1**).

---

## 18. Open design decisions

**[18.0.1]** 1. **(Open)** Arrow sampling by fixed pixel stride vs area stride (currently area stride, **§14.2**).
**[18.0.2]** 2. **(Open)** Whether `balance_mode = adjust` may tilt the target plane (currently vertical offset only, **§8**).
**[18.0.3]** 3. **(Open)** Graph connectivity (4 vs 8) for min-cost flow (**§9.2**).
**[18.0.4]** 4. **(Open)** Whether the haul-heading default should track the *existing* surface slope rather than the *design* surface slope on fields where they differ materially (**§11.0.3b**).

---

## 19. Implementation checklist

**[19.0.1]** - ☑ AGD reader per **§6.1**, incl. `delta` recomputation and inverse projection.
**[19.0.2]** - ☑ UTM projection and 15 ft binning per **§7.1**–**§7.2**.
**[19.0.3]** - ☑ BCY conversions and 61 mm layering per **§7.3**.
**[19.0.4]** - ☑ Volume balance check (and optional offset adjust) per **§8**.
**[19.0.5]** - ☑ Poisson flow field per **§9.1**; min-cost-flow mode per **§9.2**.
**[19.0.6]** - ☑ Streamline strips per **§10**.
**[19.0.7]** - ☑ Contiguous patch passes (≈ 15.4 BCY / 61 mm), true haul leg, heading bias per **§11**.
**[19.0.8]** - ☑ Sequencing (NN + bounded 2-opt) per **§12**; zone work order + CSV per **§12.1**.
**[19.0.9]** - ☑ OptiHaul-matched time model per **§13**.
**[19.0.10]** - ☑ PNG cut/fill, arrows, zone overlay, heat map, animation per **§14**.
**[19.0.11]** - ☑ CLI per **§15**.
**[19.0.12]** - ☑ Tests per **§17**; deterministic output per **§16.4**.
**[19.0.13]** - ☑ All source and this spec stored UTF-8 without BOM.

---

## 20. Summary

**[20.0.1]** This v2 specification defines a Python program that turns an AGD field file plus equipment and time parameters into a haul plan, a rendered PNG, and optional sequencing/CSV/heat-map/animation outputs. The field is modeled on a **15 ft** grid (configurable); volumes are tracked in **BCY**, with **swell** applied only to convert the scraper's **LCY** capacity to a bank load limit (20 LCY ÷ 1.3 ≈ 15.4 BCY) and a **cut/fill ratio** for compaction. Haul directions come from an optimal-transport flow field traced into curving strips; the volume plan is produced by a shortest-haul-first matcher that cuts and fills **compact, contiguous patches**, with the per-pass **haul leg measured as the true transit** (cut-end → fill-start) and an optional **heading** that biases fill selection down the field's main slope. Passes are sequenced to minimize deadhead return travel and costed with an **OptiHaul-matched** time model (trapezoidal haul/return speeds, return distance = forward distance, field-efficiency "Other" time). Optional outputs summarise the plan as a **zone work order** (with CSV and numbered PNG waypoints), export **per-pass lat/lon pairs** to CSV, render a **haul heat map**, and produce a frame-by-frame **leveling animation** in cut/fill or topology mode. The PNG reuses the existing AgGrade palette, arrow glyph, and image geometry so output is directly comparable to the current tools.
