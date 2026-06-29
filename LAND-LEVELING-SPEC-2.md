# Land Leveling Haul Algorithm Specification — v2 **[0.0.1]**

**[0.0.2]** This document is the authoritative specification (revision 2) for the **land leveling haul algorithm**: a Python program that reads an OptiSurface-style **AGD** field file, plans a sequence of cut–haul–fill operations that bring the field to its target grade with minimum haul distance and time, and renders a **PNG** of the cut/fill map with haul-direction arrows, plus optional sequencing CSVs, a haul "heat map" PNG, and a leveling animation.

**[0.0.3]** This v2 supersedes **LAND-LEVELING-SPEC.md** (v0.0.1). It folds in the optimizations and outputs added during implementation: a **7.5 ft** default grid (≈ one blade width), **contiguous region-grown** cut/fill patches grown as long **blade-wide strips**, **multi-drop** cuts/fills across nearby pockets, the **true transit haul leg** (cut-end → fill-start), an optional **haul heading** bias (snapped to the nearest cardinal axis), an **OptiHaul-matched** time model, **direction-aware** turn-minimizing sequencing, a **blade-scale operator guidance** track (turn-by-turn waypoints, a grade/compact return pass, and a realistic drivable time that counts turning), **zone-level work sequencing**, per-pass **lat/lon CSV** export, a **pass-pairs heat-map** PNG, and a frame-by-frame **leveling animation**. Where v1 and the code disagreed, the code is treated as authoritative and this document is updated to match.

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
| **2.2.0** | 2026-06-27 | Blade-scale execution layer. Default grid **15 → 7.5 ft** (≈ one blade width, §7.2). Strip-biased region growing so each load is one long blade-wide lane (§11.0.2b). Multi-drop cuts/fills across nearby pockets within an economical hop (§11.0.3c). Heading snapped to the nearest cardinal axis when within tolerance (§11.0.3b). Direction-aware, grade-aware sequencing that minimizes turning, not just distance (§12.0.5). New **Stage 7 — blade-scale operator guidance** (§13.1–§13.6): turn-by-turn waypoints, a grade/compact return pass over every fill, a turning model (45 ft circle, turn speed, turn threshold), multi-drop `MOVE` repositioning, and a realistic drivable time. Tandem 7 ft + 10 ft scrapers modeled as one ≈ 8.5 ft effective blade (§6.2). New CLI flags in §15. |
| **2.3.0** | 2026-06-28 | **Flow-aligned passes (default).** Each load's cut/fill grows along the local transport flow — cut traced *upstream* to the ridge, fill *downstream* into the sink — giving long, curved, with-the-grain lanes; the matcher now reads the flow field in this mode (§9.0.2, §11.0.2c). `--no-flow-aligned` reverts to the single global-heading strip mode of §11.0.2b. **Guidance route smoothing (default, display-only):** the displayed sweep coordinates are smoothed into a drivable curve whose corner radius respects the turning circle (§13.3a), while all distance/turn/time stay on the raw coverage geometry — smoothing must never drop a sweep below the coverage floor `volume ÷ (max_depth × blade_width)`. `--no-route-smoothing` disables it. New CLI flags in §15. |
| **2.4.0** | 2026-06-28 | **Knot re-profiling + load-fullness cap.** Some compact multi-drop loads chain into an undriveable "knot" (a sweep that hairpins back on itself). After sequencing, the guidance now re-profiles such sweeps into a **boustrophedon** — parallel blade-width back-and-forth lanes — but only where the rendered sweep is still a knot **and** the boustrophedon strictly reduces hairpins, so clean flowing sweeps are never made worse; the re-profile is **pocket-aware** and **conservative** (no bin/load/haul/surface change, only the in-patch path) (§13.3b). `--no-knot-repair` disables it. New **`--min-load-fill`** (default 90%) caps multi-drop chasing once a load is "full enough", keeping more loads single clean lanes instead of U-turn blobs (§11.0.3c-i). New CLI flags in §15. |

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
**[2.0.4]** 3. A grid model derived from the AGD points (default **7.5 ft** areas, ≈ one blade width), decoupled from blade width (**§7**).
**[2.0.5]** 4. Volume accounting in **banked cubic yards (BCY)** for cut supply, fill demand, and transported volume; swell applies only when converting scraper **LCY** capacity to a bank load limit; compaction via the cut/fill ratio (**§7.3**).
**[2.0.6]** 5. A global haul **flow field** computed by optimal transport, for haul arrows and strip visualization (**§9**).
**[2.0.7]** 6. **Strips** traced as streamlines of the flow field, from ridges to the cut/fill transition (**§10**).
**[2.0.8]** 7. **Passes** packed to the scraper load limit (20 LCY ≈ 15.4 BCY) under the 61 mm depth-per-pass limit, each cutting and filling a **compact, contiguous patch** (**§11**).
**[2.0.9]** 8. A pass **sequence** that minimizes non-productive (deadhead) return travel and respects layer ordering (**§12**).
**[2.0.10]** 9. A **time estimate** matching the OptiSurface OptiHaul methodology (trapezoidal haul/return speeds, return distance = forward distance, field efficiency) (**§13**).
**[2.0.11]** 10. A **PNG renderer** whose cut/fill palette, arrow glyphs, and image geometry match the existing AgGrade tools (**§14**).
**[2.0.12]** 11. Optional **zone work-order sequencing**, **per-pass lat/lon CSV**, a **pass-pairs heat-map PNG**, and a **leveling animation** (**§12.1**, **§14.4**–**§14.6**).
**[2.0.13]** 12. A documented **command-line interface** (**§15**).
**[2.0.14]** 13. A **blade-scale operator guidance** track that turns the optimized passes into turn-by-turn waypoints (action, blade up/down, target depth, next bearing, instruction) plus a **realistic drivable time** that — unlike the OptiHaul-matched core model — counts **turning** and a **grade/compact return pass** over every fill (**§13.1**–**§13.6**). The guidance shall not worsen the optimized passes' haul/time; it realizes them at blade scale.

## 3. Non-goals

**[3.0.1]** The implementation **shall not** attempt the following in this version:

**[3.0.2]** 1. Choosing or optimizing the **target surface** (plane fitting, slope/heading solve). The target is read from the AGD proposed elevations; see **§8** for the only permitted exception. (The haul **heading** in **§11.0.3b** biases *fill selection* only; it does not alter the target surface.)
**[3.0.3]** 2. Multi-machine fleet scheduling.
**[3.0.4]** 3. Modeling machine traction or fuel. (The **turning circle** *is* modeled — but only in the blade-scale guidance time and the direction-aware sequencer, **§13.1**, **§12.0.5** — not in the core OptiHaul-matched time of **§13.0**.)
**[3.0.5]** 4. Reading any input from a database; the AGD path replaces the legacy SQLite path.
**[3.0.6]** 5. Producing GIS/shapefile or 3D outputs.

---

## 4. Terminology

### 4.1 Area (cell, bin)

**[4.1.1]** An **area** is one square grid cell of the field. The default cell is **7.5 ft × 7.5 ft** (≈ one blade width) (**§7.2**); it is configurable. Each area carries one existing and one target elevation and is the atomic unit of cut, fill, and haul.

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

### 4.11 Strip bias and blade-wide lane

**[4.11.1]** **Strip bias** is the anisotropy applied during region growing (**§11.0.2b**) so a load's cut (or fill) patch grows as one long, **single-blade-wide lane** along the heading rather than a compact blob. A **lane** is the resulting near-1-cell-wide run the operator drives in a straight line, minimizing weaving and turns.

### 4.12 Pocket and multi-drop

**[4.12.1]** A **pocket** is a connected island of remaining cut (or fill) material. **Multi-drop** is the behavior whereby, when a load cannot be filled (or emptied) within one pocket, the machine drives **blade up** to the next pocket within an **economical hop** (`max_drop_hop`, **§6.4**) and continues loading (or spreading) the same load, instead of ending the pass part-full and paying for a whole separate cut–haul–fill cycle later.

### 4.13 Grade/compact return pass

**[4.13.1]** A **grade/compact return pass** is one return traverse back over a just-placed fill (blade down) to grade and compact it (**§13.2**). It is required finishing work; the core OptiHaul-matched time (**§13.0**) does not include it, the blade-scale guidance time (**§13.1**) does.

### 4.14 Turning circle and turn threshold

**[4.14.1]** The **turning circle** is the tractor + scraper turn diameter (default 45 ft); its radius sets the arc length `R·θ` travelled at the reduced **turn speed** during a heading change of `θ`. A heading change below the **turn threshold** (default 60°) is a gentle bend driven in stride at travel speed (no slowdown); only sharper turns cost turn time (**§13.1**).

### 4.15 Guidance track and waypoint

**[4.15.1]** The **guidance track** is the ordered list of blade-scale **waypoints** produced by Stage 7 (**§13.3**). Each waypoint carries an **action** (`START`, `CUT`, `HAUL`, `MOVE`, `FILL`, `GRADE`, `RETURN`), a blade up/down state, world coordinates, a target depth, the bearing to the next waypoint, and a human instruction.

### 4.16 Flow-aligned lane (streamline tube)

**[4.16.1]** A **flow-aligned lane** is a cut or fill patch grown along the **local transport flow** (a streamline of the flow field, **§9**) rather than a fixed compass heading. The cut lane is traced **upstream** (toward the ridge the soil is drawn from) and the fill lane **downstream** (into the sink it feeds), so cuts, hauls and fills become long, curved, *with-the-grain* runs. Flow-aligned growth is the **default**; **`--no-flow-aligned`** selects the single global-heading strip of **§4.11** instead.

### 4.17 Route smoothing (steering line)

**[4.17.1]** **Route smoothing** rounds a sweep's **displayed** waypoint coordinates into a drivable curve whose corner radius respects the turning circle (**§13.3a**), because raw grid waypoints step in 45/90° jogs no tractor+scraper can track. Smoothing is a **steering-display transform only**: it never changes the distance, turn or time accounting, which stay on the raw coverage geometry (the blade must still cover the full cell band).

---
## 5. High-level architecture

**[5.0.1]** The program is a pipeline; each stage consumes the previous stage's output and is independently testable.

**[5.0.2]**

```text
AGD file ──► [1] Parse + grid (7.5 ft)         agd.py, grid.py   §6, §7
          ──► [2] Volume balance (check/adjust)  balance.py        §8
          ──► [3] Flow field (optimal transport) flowfield.py      §9
          ──► [4] Strips (streamlines)           strips.py         §10
          ──► [5] Passes (≈15.4 BCY / 61 mm)     passes.py         §11
          ──► [6] Sequence + route               routing.py        §12
          ──► [7] Time estimate                  timemodel.py      §13
          ──► [8] Blade-scale guidance (opt.)    swaths.py         §13.1–§13.6
          ──► PNG: cut/fill map + haul arrows    render.py         §14
          ──► (optional) zones / CSVs / heat map sequence.py, render.py  §12.1, §14.4–14.5
          ──► (optional) guidance CSV / PNG      swaths.py, render.py  §13.6
          ──► (optional) leveling animation      animate.py        §14.6
```

**[5.0.3]** Equipment (**§6.2**) and time/motion (**§6.3**) parameters feed stages [5]–[7]. The cut/fill map depends only on stages [1]–[2]; the haul arrows depend on stage [3].

**[5.0.3a]** *Flow-field dependency depends on the pass mode.* Under **`--no-flow-aligned`** (the single global-heading mode, **§11.0.2b**) the optimized haul-route plan does **not** depend on the flow field or strips: stages [3] (flow field, **§9**) and [4] (strips, **§10**) exist only for haul-direction visualization, and the pass plan in stage [5] (**§11**) is computed directly from the per-area cut/fill geometry — a nearest-fill anchor plus distance-ordered region growing. An implementer who only wants the single-heading routes/sequence/time may therefore **skip §9 and §10**. In the **default flow-aligned mode** (**§11.0.2c**), however, the pass grower **does** consume the flow field (stage [3]) to grow each load's cut/fill along the local transport streamline, so stage [3] is then a required input to the plan (strips, **§10**, remain visualization-only).

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
| **Blade width** | 2271 mm | Cutting/filling blade width (informative; not the grid size). The real machine pulls **two scrapers in tandem** (front 7 ft / 8 LCY, rear 10 ft / 12 LCY), filled sequentially; its volume-weighted effective width is **≈ 8.5 ft** (`1/W = 0.4/7 + 0.6/10 ⇒ 8.54 ft`), modeled by passing `--blade-width-mm 2590.8`. |
| **Blade height control** | 10 Hz | Blade height update rate (informative). |
| **Turning circle** | 45 ft | Tractor + scraper turn diameter; radius sets the turn-arc length in the guidance time and sequencer (**§13.1**, **§12.0.5**). |
| **Turn speed** | 4 MPH | Speed through a turn (the machine slows to turn). |
| **Grade speed** | 8 MPH | Speed of the grade/compact return pass over a fill (**§13.2**). |
| **Turn threshold** | 60° | Heading changes below this are gentle bends driven in stride (no slowdown); only sharper turns cost turn time (**§13.1**). |
| **Flow-aligned passes** | on | Default: grow each load's cut/fill along the local transport flow (cut upstream, fill downstream) for curved, with-the-grain lanes (**§11.0.2c**). `--no-flow-aligned` selects the single global-heading strip mode below. |
| **Haul heading** | auto | Compass bearing soil is hauled along; default is the design surface's main down-slope, **snapped to the nearest cardinal axis** when within tolerance. The global strip heading under `--no-flow-aligned`, and a fallback where the flow is flat; biases fill selection (**§11.0.3b**). |
| **Heading strength** | 0.5 | 0–0.95; how strongly fill selection follows the heading (0 = off). |
| **Strip bias** | 50 | Single-heading mode only: anisotropy of region growing — grow each load as one long blade-wide lane along the heading (**§11.0.2b**); 1 = compact blob. |
| **Multi-drop hop** | 100 ft | Max blade-up drive to the next pocket to keep loading/spreading the same load (**§11.0.3c**); 0 disables multi-drop. |
| **Minimum load fill** | 90% | Drivability vs load-fullness trade-off: once a load reaches this fraction of capacity, further multi-drop hops are skipped so it stays a single clean lane (**§11.0.3c-i**); 100 = chase every load to full. |
| **Knot re-profiling** | on | Re-profile undriveable knot sweeps into boustrophedon parallel passes after sequencing (**§13.3b**); `--no-knot-repair` disables it. |

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
| **Grid size** | 7.5 ft | Edge length of each square area (**§7.2**); ≈ one blade width so each load is one blade-wide lane. Use 15 ft to match an OptiHaul coarse calculation grid. |
| **Lane overlap** | 0.0 | 0–0.9 fractional overlap between adjacent blade lanes in the guidance (**§13.3**); 0 = lanes butt edge-to-edge. |
| **Tolerance** | 10 mm | Finished-area elevation tolerance (**§4.8**). |
| **Pixels per bin** | 50 | PNG pixels per area edge for the main plan PNG (**§14.3**). |
| **Arrow stride** | 2 | Draw a haul arrow every N areas (0 disables). |
| **Transport mode** | `poisson` | `poisson` (L²) or `mincostflow` (L¹) flow field (**§9**). |
| **Balance mode** | `check` | `check` (warn) or `adjust` (re-level target, **§8**). |
| **Zones** | 8 | Number of cut zones for the rough work order (0 disables) (**§12.1**). |
| **Seed** | 1234 | Random seed for deterministic zone clustering (**§16.4**). |

**[6.4.2]** The optional output toggles (sequence CSV, pairs CSV, pairs PNG, **guidance CSV/PNG**, animation directory/GIF, animation mode/stride/pixels-per-bin) are defined with the CLI in **§15** and the outputs in **§12.1**, **§13.6**, and **§14.4**–**§14.6**. The maneuvering parameters (turn speed, grade speed, turning circle, turn threshold) and the strip-bias / multi-drop / lane-overlap controls are listed in **§6.2** and exposed in **§15**.

---

## 7. Coordinate and grid model

### 7.1 Projection

**[7.1.1]** The reader shall project each AGD point's lat/lon to a local planar (UTM, WGS-84) system in metres, with the field's south-west corner as the local origin (`utm_from_latlon`). A field crossing a UTM zone boundary shall be rejected with a clear error.

**[7.1.2]** The inverse projection (`latlon_from_utm`) shall be provided for CSV outputs (**§14.4**) and shall be consistent with **§14.3** geometry.

### 7.2 Binning to the grid

**[7.2.1]** The field bounding rectangle shall be partitioned into square areas of edge **`grid_size`** (default **7.5 ft = 2.286 m**). The 7.5 ft default is chosen so a cell is ≈ one blade width (2271 mm), making each scraper load **one continuous blade-wide lane** rather than a multi-lane patch that would force tight turns (**§11.0.2b**, **§13.3**); the grid size is configurable (**§6.4**) and coarser grids (e.g. 15 ft to match an OptiHaul calculation grid, or 30 ft) run faster. Each AGD interior point is assigned to its containing area (floor of `(coord − min) / grid_size`, clamped to the grid); an area's existing and target elevations are the **mean** of the points it contains. Areas outside the mask are invalid.

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

**[9.0.2]** *Required for flow-aligned passes; otherwise visualization only.* Under `--no-flow-aligned` this stage and the strips of **§10** are **not** inputs to the pass plan (**§5.0.3a**) — they only produce the haul-direction overlay, and may be omitted if only the single-heading routes/time are needed. In the default **flow-aligned** mode (**§11.0.2c**) the pass grower reads this flow field to trace each load's cut/fill along the local streamline, so it is then a required stage.

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

**[11.0.1]** Passes shall accumulate volume in BCY subject to **§6.2**, packing each pass to the scraper load limit (`scraper_LCY ÷ swell` ≈ 15.4 BCY) under the 61 mm depth-per-pass cap. The volume plan is produced by the **shortest-haul-first, contiguous patch matcher** below. Two patch-growing modes are available: the **flow-aligned** grower (**§11.0.2c**, default) traces each load's cut/fill along the local flow streamline (**§9**); the **single-heading** grower (**§11.0.2a**–**§11.0.2b**, `--no-flow-aligned`) operates **solely** on the per-area cut/fill ledger (`delta` and the BCY conversions of **§7.3**) and consumes neither the flow field nor strips. In the single-heading mode this section together with **§7** is sufficient to reproduce the haul-route plan; in the flow-aligned mode the flow field of **§9** is an additional required input.

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

**[11.0.2b]** *Strip bias (normative).* When a haul **heading** is supplied (or auto-detected) and `strip_bias > 1`, the growth priority key in **§11.0.2a** is replaced by an **anisotropic** key so the patch grows as one long, single-blade-wide **lane** along the heading instead of a compact blob (which would force the operator to weave). Let `(ux, uy)` be the heading unit vector and `(vx, vy)` its perpendicular; for a candidate cell at offset `(ddx, ddy)` from the seed:

```text
along = ddx*ux + ddy*uy          # distance along the heading
perp  = ddx*vx + ddy*vy          # distance across the heading
key   = abs(along) + strip_bias * perp^2
```

The along term is **linear** (cheap) while the perpendicular term is quadratic and scaled by `strip_bias` (expensive), so a load fills one blade-wide line end-to-end before the frontier ever steps sideways; the lane only widens when it runs out of material along its length. `strip_bias` is roughly the lane length (in cells) the grower will chase before stepping over. Default **50** (set above a full load's cell count to keep each load a single continuous line); `strip_bias = 1` restores the isotropic compact-blob growth of **§11.0.2a**.

**[11.0.2c]** *Flow-aligned growth (normative, default).* Unless **`--no-flow-aligned`** is set, each load's patch is grown as a **flow-aligned lane** (**§4.16**) instead of by **§11.0.2a**/**§11.0.2b**. From a smoothed unit flow-direction field `(fx, fy)` derived from the flow potential (**§9**, exposed by `strips.py:flow_direction_field`), the grower integrates a **single streamline** through the seed cell with a small fixed step (bilinear interpolation of `(fx, fy)`), consuming the cells it passes through (one 61 mm layer each) until the load reaches `target_bcy`, the streamline leaves the mask, or it runs out of material. The trace is **directional**: a **cut** patch is integrated **upstream** only (against the flow, toward the ridge it draws from) and a **fill** patch **downstream** only (with the flow, into the sink it feeds), so the consumed cells form one monotonic, non-self-crossing run in streamline order — long, curved, with-the-grain lanes rather than the staircase a fixed heading would force across diagonal flow. The cells are returned **in streamline order** (no nearest-neighbour re-chaining); each pass records its local flow bearings (`cut_heading_deg`, `fill_heading_deg`) for the guidance lane builder (**§13.3**). Multi-drop (**§11.0.3c**) and exact volume conservation (**§11.0.3**) apply unchanged. `--no-flow-aligned` restores the single global-heading grower of **§11.0.2b**.

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

**[11.0.3b-i]** *Cardinal snapping (normative).* Because the grid is axis-aligned (N/S/E/W), a heading even a degree or two off an axis makes strips (**§11.0.2b**) grow diagonally across cells and fragment into many small pockets (more multi-drop hops, **§11.0.3c**). `estimate_slope_heading` therefore **snaps** the auto-estimated bearing to the nearest of {0, 90, 180, 270}° when it is within `snap_tol_deg` (default **12°**) of that axis; otherwise the raw bearing is returned. An explicit `--heading-deg` is used as given (not snapped).

**[11.0.3c]** *Multi-drop cuts and fills (normative).* A single pocket often cannot fill the scraper (cut side) or fully absorb the load (fill side), which would leave many near-empty "crumb" passes. When `max_drop_hop > 0`, after growing a patch (**§11.0.2b**) the matcher checks whether the load is still short of `load_cap` (cut) or whether material remains carried (fill); if so it finds the **nearest remaining cell of the same kind** (`_nearest_remaining`, KD-tree when SciPy is present, else brute force) from the current sweep end. If that cell is within `max_drop_hop` metres, the machine drives there **blade up** and grows another patch from it into the same load, repeating until the load is full/empty or the next pocket exceeds the hop. The blade-up drive between pockets is a `MOVE` segment in the guidance (**§13.4**), distinct from the optimized `HAUL` leg. Continuing one load across nearby pockets is cheaper than a whole separate cut–haul–fill–grade cycle, so multi-drop materially raises load fullness and cuts pass count. `max_drop_hop = 0` disables it (one pocket per cut and per fill).

**[11.0.3c-i]** *Minimum load fraction (optional, §6.2).* Topping a load to the last few percent of capacity often forces a multi-drop hop into a **laterally offset** pocket, which turns a single flowing lane into a side-by-side blob the blade can only cover with tight U-turns (the knots of **§13.3b**). To trade load-fullness for drivability, the cut multi-drop loop (**§11.0.3c**) shall stop adding hops once the load reaches `min_load_frac × load_cap` (default **0.90**, i.e. `--min-load-fill 90`), taking the load as complete. Lower values keep more loads as single clean lanes at the cost of slightly under-filled scrapers; **`1.0`** (`--min-load-fill 100`) restores the original chase-to-full behaviour.

**[11.0.4]** *Layering.* Because 61 mm caps per-area depth, deep cuts/fills need multiple passes: a cell needing depth `d` contributes about `ceil(d / 61 mm)` passes over successive layers, with the transition receding as boundary cells reach grade. The final layer of an area may be a partial pass (**§7.3**).

**[11.0.5]** Each pass shall record: ordered cut cells with depths, ordered fill cells with depths, load in BCY, the haul distance, and the cut-segment start/end and fill-segment start/end world coordinates (used by sequencing, the time model, and the CSV/heat-map outputs).

**[11.0.6]** The implementation shall report the number of passes and any **unmatched** cut or fill BCY remaining below tolerance at termination.

---
## 12. Stage 5 — Sequencing and routing

**[12.0.1]** The passes from **§11** form a set; the implementation shall order them to minimize total **non-productive (deadhead) return travel** — the distance from where one pass ends (the fill end, or the grade-return end `fill_start` when the grade pass is on, **§12.0.5**) to the start of the next pass's cut segment. By default this is generalised to a **direction-aware time** that also charges the turn into the next cut (**§12.0.5**).

**[12.0.2]** Sequencing shall respect **layer ordering**: a deeper layer of an area shall not be worked before the shallower layer above it. Because passes are generated layer-by-layer (shallower first), this holds by construction.

**[12.0.3]** *Anti-dam ordering (informative).* Ordering should favor completing downhill flow first to avoid transient berms that could pond water; this is a soft preference, not a hard constraint in this version.

**[12.0.4]** The ordering shall be a constructive heuristic followed by bounded local improvement. The deadhead metric between an earlier pass `a` and a later pass `b` is `deadhead(a, b) = euclidean(exit_xy(a), b.cut_start)` (for `exit_xy` see **§12.0.5**). Construction is **nearest-neighbour starting from pass index 0** (deterministic): repeatedly append the unvisited pass minimizing the transition cost from `current`. Then a **bounded 2-opt** pass improves the order: it is applied **only when direction-aware sequencing is off (§12.0.5) and the pass count ≤ 1500** (above either, the construction order is kept, to stay fast and deterministic), runs at most **30 sweeps**, and accepts a segment reversal only when it strictly reduces the summed deadhead (improvement threshold `1e-6` m), taking the first improving move in each sweep. Exact optimization is not required. The route shall expose the ordered passes, the original indices, and the total deadhead distance.

**[12.0.5]** *Direction-aware, grade-aware transition cost (normative, default on).* The next cut should be cheap to **drive to and turn into**, not merely close. The transition cost from `a` to `b` is a **time** that combines the deadhead drive with the turn arcs, using the same turning-circle model as the guidance (**§13.1**):

```text
exit_xy(a)  = a.fill_start  if grade_return else a.fill_end   # where the machine ends
dist        = euclidean(exit_xy(a), b.cut_start)
drive_t     = dist / return_speed
angle       = turn(exit_dir(a) → deadhead_dir) + turn(deadhead_dir → cut_dir(b))
turn_t      = (turning_circle/2) * angle / turn_speed
cost(a, b)  = drive_t + turn_t
```

where `exit_dir(a)` is the machine's heading as it finishes `a` (the grade-return direction `fill_end → fill_start` when the grade pass is on, else the spread direction), `cut_dir(b)` is `b`'s cut heading, and `deadhead_dir` is the unit vector of the deadhead leg. **Grade-aware exit:** with the grade/compact pass on (**§13.2**) the machine ends at `fill_start` (the cut-side edge of the fill), so the grade return doubles as travel toward the next cut instead of being a detour. Because these costs are **asymmetric** (a reversed segment flips every interior heading), 2-opt segment reversal is **not** applied in this mode (**§12.0.4**); the turn-aware nearest-neighbour construction already orders passes against the time objective. Setting `--no-direction-aware` reverts the cost to plain deadhead distance and re-enables bounded 2-opt.

### 12.1 Zone work-order sequencing (optional output)

**[12.1.1]** On request (`--zones N`, default 8; 0 disables), the implementation shall summarise the fine-grained route into a small number of spatial **zones** for a human-readable rough work order. This is a reporting summary; it does not change the route or the plan.

**[12.1.2]** The cut-patch centroids of the routed passes shall be clustered into `k = min(N, n_passes)` zones with a **deterministic k-means++** (seeded from **§6.4**). Each zone shall report: a stable `zone_id`; a `work_order` rank (1 = worked earliest); load count; moved BCY; centroid in local XY **and** lat/lon (via `latlon_from_utm`); the first/last/mean pass index over which it is worked; and a rough compass position within the field.

**[12.1.3]** Zones shall be ordered by their **load-weighted mean pass index** (`mean_pass`), so `work_order` reflects when each zone is mostly worked. Because the route interleaves passes across zones to minimize travel, the work order is an *average* ordering, not a strict "finish zone 1 before starting zone 2" sequence; the reported first/last pass ranges make the overlap explicit.

**[12.1.4]** For overlay rendering (**§14.4**) each cut cell shall be assigned to the single zone that removed the most depth there, and the **exact cell-boundary outline** (jagged perimeter of that zone's cells) shall be computed as world-metre segments, so zone footprints partition the field without overlap.

**[12.1.5]** The zone work order shall be writable to a **sequence CSV** (**§14.4**) and drawn as numbered waypoints + outlines on the plan PNG (**§14.4**).

---

## 13. Stage 6 — Time model (OptiHaul-matched) & Stage 7 — blade-scale guidance

**[13.0.0]** This section has two parts. **§13.0** is the **core OptiHaul-matched** time model (four motions, no turning, no grade pass) used for direct comparison with OptiSurface OptiHaul. **§13.1**–**§13.6** are the optional **blade-scale operator guidance** (Stage 7): a drivable realization of the same passes that additionally counts **turning** and a **grade/compact return pass**, and emits turn-by-turn waypoints. Both consume the routed passes of **§12**; the guidance never changes the passes or their optimized haul, it only realizes them at blade scale.

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

### 13.1 Stage 7 — blade-scale guidance overview

**[13.1.1]** On request (**§13.6**), the implementation shall build a **guidance track** (`swaths.py`, `build_guidance`) that tiles each routed pass's cut and fill patches into **blade-width lanes** and emits an ordered list of **waypoints** (**§4.15**, **§13.3**). This realizes the optimized passes at the scale the operator actually drives, without re-planning soil movement: the per-pass haul leg is preserved as the optimizer's transit (the cut sweep is oriented to *end* at the haul target and the fill sweep to *start* there).

**[13.1.2]** *Turning model.* A heading change of `θ` at a point in the track costs arc travel `R·θ` at the **turn speed**, where `R = turning_circle / 2`. Turn metrics are computed on each sub-path **after Ramer–Douglas–Peucker simplification** (so cell-to-cell stair-stepping is not miscounted as turns): the path is simplified, the interior vertex angles are measured, and only those **≥ `turn_threshold_deg`** (default 60°) are counted as **tight turns** and summed into the total turn angle. Gentler bends are driven in stride at travel speed (no slowdown). The track reports the tight-turn **count** and the summed turn **angle**.

### 13.2 Grade/compact return pass

**[13.2.1]** Unless `--no-grade-return` is set, each fill shall be followed by **one return pass back over the fill** (blade down) to grade and compact it, retracing the fill lane in reverse. Its distance equals the fill sweep length and it is driven at the **grade speed** (default 8 MPH). With the grade pass on, the machine **ends each pass at the fill's cut-side edge** (`fill_start`), which the sequencer exploits so the grade return doubles as travel toward the next cut (**§12.0.5**). The core model (**§13.0**) does **not** include this pass; the guidance time (**§13.5**) does.

### 13.3 Guidance waypoints and lane building

**[13.3.1]** *Single-heading lanes.* For single-heading passes (**§11.0.2b**), each patch shall be split into 8-connected **pockets** (**§4.12**) and the pockets ordered to optimize the haul leg: the **cut** sweep is ordered to finish nearest the fill start, the **fill** sweep to start nearest the haul arrival. Within a pocket, the cells are tiled into serpentine **blade-width lanes** spaced by `blade_width × (1 − lane_overlap)`; a second lane is added only when the patch is genuinely wider than one blade (guarding against spurious second lanes for slightly off-axis strips).

**[13.3.1a]** *Flow-aligned lanes.* For flow-aligned passes (**§11.0.2c**, signalled by a per-pass `cut_heading_deg`/`fill_heading_deg`), the lane builder shall **preserve the streamline order** in which the grower returned the cells rather than re-tiling them by heading. The stream-ordered cells are split into maximal runs of consecutive 8-adjacent cells (a multi-drop hop is a break to a non-adjacent cell); each run is driven in order, and **whole runs** are flipped only to orient the cut sweep to finish near the fill (and the fill sweep to start near the haul arrival). This avoids nearest-neighbour re-chaining, which would zig-zag a curved lane.

**[13.3.2]** Each waypoint shall carry: an **action** ∈ {`START`, `CUT`, `HAUL`, `MOVE`, `FILL`, `GRADE`, `RETURN`}, a **blade state** (down for `CUT`/`FILL`/`GRADE`; up for `START`/`RETURN`/`HAUL`/`MOVE`), world coordinates (and lat/lon), the **target depth** (mm) at that point, the **bearing** to the next waypoint, and a human-readable **instruction**.

### 13.3a Route smoothing (drivable steering line)

**[13.3a.1]** Raw waypoints sit on grid-cell centers, so a lane that drifts off-axis steps in 45/90° jogs no tractor+scraper can physically track. Unless **`--no-route-smoothing`** is set, the implementation shall smooth each cut/fill sweep's **emitted coordinates** into a drivable curve: a Gaussian over the waypoint sequence whose width is tied to the machine **turning radius** (`turning_circle / 2`), applied **per contiguous run** (split at multi-drop hops) with the run's **endpoints pinned**. Pinning the endpoints leaves the optimized haul/return junctions exactly where the sequencer placed them; the emitted waypoint `x, y` (and lat/lon) are the smoothed steering points the operator follows on the in-cab map, and the next-bearing is computed along that smoothed line.

**[13.3a.2]** *Smoothing is steering-display only (normative).* Smoothing **shall not** change any distance, turn-count, turn-angle or time. All such accounting (**§13.4**, **§13.5**) shall be computed on the **raw coverage geometry** (the unsmoothed cell sweep), not the shortened smoothed line. This is a hard correctness requirement: a one-blade-wide lane that staircases along a diagonal still requires the blade to cover the whole cell band, so the loading/dumping sweep length is bounded below by `volume ÷ (max_depth × blade_width)`. Smoothing the centerline to a shorter curve would drop the sweep **below that physical floor** and silently under-cut the field; therefore the coverage length, not the steering line, drives the time.

### 13.3b Knot re-profiling (boustrophedon)

**[13.3b.1]** *Detection (normative).* Some compact multi-drop loads (**§11.0.3c**) chain into an undriveable **knot** — a sweep that doubles back on itself with several hairpin reversals in a tiny area that no 45 ft-circle machine can follow. After sequencing (**§12**) and before the guidance time/outputs are produced, unless **`--no-knot-repair`** is set, the implementation shall scan every pass and, for the cut and the fill sweep independently, build the path the guidance would otherwise draw (the flow/lane tiling of **§13.3**/**§13.3.1a** **plus** the serpentine find-and-fix) and count its hairpins on the Ramer–Douglas–Peucker-simplified path (**§13.1.2**). A sweep is a knot when that count is **≥ the hairpin threshold** (default **3**).

**[13.3b.2]** *Boustrophedon re-profile (normative).* For a knotted sweep the implementation shall build a **boustrophedon** alternative: the patch is decomposed into parallel **blade-width lanes** along its principal axis (PCA of the cell centers) and the lanes are walked in serpentine order — every other lane reversed — so the machine drives one straight lane, makes a single headland turn, and drives the next lane back (multiple straight **back-and-forth sweeps** rather than a zig-zag). The boustrophedon shall be adopted **only when it has strictly fewer hairpins than the rendered sweep** (**§13.3b.1**); otherwise the sweep is left unchanged. This guarantees a sweep that already drives cleanly (e.g. a gentle flowing fill) is **never made worse** by the repair.

**[13.3b.3]** *Pocket-aware (normative).* A load may span several disjoint pockets (a multi-drop cut/fill). The boustrophedon shall be built **pocket by pocket**: the patch is split into 8-connected pockets (**§4.12**), each pocket is swept as its own local boustrophedon, and the pockets are ordered to finish nearest the fill (cut side) or start nearest the haul arrival (fill side) — exactly as the single-heading lane builder orders pockets (**§13.3.1**). Binning the whole spread as one lane field is forbidden, because it would stitch distant pockets together with long blade-up jumps; pocket-aware ordering keeps the inter-pocket repositions (`MOVE`, **§13.4**) to the genuine, unavoidable hops the matcher already created.

**[13.3b.4]** *Conservative (normative).* Re-profiling only **re-orders the cells** of a flagged patch. It **shall not** drop, add, or relocate any cut/fill bin, change the load volume, or alter the optimized haul leg — so the soil routing and the simulated finished surface are **identical** to the un-repaired plan; only the in-patch driving path changes. A re-profiled patch shall be flagged so the lane builder follows its **stored lane order directly** instead of re-deriving (and re-tangling) the sweep by **§13.3.1a**. The only cost is the extra headland turning and short repositions of the parallel sweeps (a few percent of total time, **§13.5**).

### 13.4 Multi-drop repositioning (`MOVE`)

**[13.4.1]** A multi-drop pass (**§11.0.3c**) spans several pockets; the blade-up drive between two pockets within a sweep shall be emitted as a **`MOVE`** segment (blade up), distinct from the optimized **`HAUL`** leg. `MOVE` distance is accumulated separately (`reposition_dist_m`) and shall **not** inflate the reported haul distance, though it is timed as travel (**§13.5**).

### 13.5 Guidance time estimate

**[13.5.1]** `estimate_guidance_time` shall sum, over the track: `CUT` length ÷ loading speed; `FILL` ÷ dumping speed; `GRADE` ÷ grade speed; `HAUL` and `MOVE` via the trapezoidal profile (**§13.0.3**) at haul max speed; `RETURN`/`START` via the trapezoidal profile at return max speed; and the **turning** time `R · total_turn_angle / turn_speed` (**§13.1.2**). Productive = the sum of those; **total = productive ÷ field_efficiency**, the remainder reported as "Other". The breakdown (`GuidanceTime`) shall expose loading, dumping, grading, hauling (incl. `MOVE`), returning, turning, other, and total seconds, plus the tight-turn count.

**[13.5.2]** *Relationship to the core model.* The guidance total is normally **higher** than the core OptiHaul-matched total (**§13.0**) because it adds turning, the grade/compact pass, and the extra headland turns of any knot re-profiling (**§13.3b**, a few percent), partly offset by direction-aware sequencing reducing the empty return. This is expected and is the realistic, drivable field time. The guidance shall **not** increase the optimized per-pass haul distance, and knot re-profiling shall **not** change the moved volume or finished surface (**§13.3b.4**).

### 13.6 Guidance outputs

**[13.6.1]** *Guidance CSV* (`--guidance-csv`, default `<out>_guidance.csv`): one row per waypoint in track order with the sequence index, pass number, action, blade state, lat/lon (and easting/northing), target depth, next-bearing, segment distance, and instruction. The lat/lon (and easting/northing) are the **smoothed steering points** (**§13.3a**); the segment distance is the **raw coverage** travel from the previous waypoint (it is not the straight-line gap between smoothed points — distance/time are charged on the coverage geometry, **§13.3a.2**).

**[13.6.2]** *Guidance PNG* (`--guidance-png`, default `<out>_guidance.png`): the cut/fill map overlaid with the blade lanes coloured by action — cut, fill, the `GRADE` return (dashed, over the fill), and `MOVE`/`HAUL`/`RETURN` blade-up travel — so the drivable plan is visually inspectable. On printing the guidance the report shall also state the preserved haul, blade-scale loading/dumping/grade/reposition distances, the coverage factor (blade width ÷ cell width), the tight-turn count, the sequencing mode, and the drivable time breakdown and total.

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
--grid-size-ft FLOAT          default 7.5   (~ one blade width; use 15 for an OptiHaul grid)
--tolerance-mm FLOAT          default 10
--scraper-capacity-lcy FLOAT  default 20    (bank limit = value / swell)
--swell-factor FLOAT          default 1.3   (LCY = BCY * swell)
--cutfill-ratio FLOAT         default 1.2   (alias: --compaction)
--max-depth-mm FLOAT          default 61
--blade-width-mm FLOAT        default 2271  (tandem 7+10 ft -> use 2590.8 for ~8.5 ft)
--strip-bias FLOAT            default 50    (grow each load as one blade-wide lane; 1 = blob)
--max-drop-hop-ft FLOAT       default 100   (multi-drop hop to next pocket; 0 disables)
--min-load-fill FLOAT         default 90    (% capacity at which multi-drop stops chasing; 100 = chase to full)

# Haul behaviour
--no-flow-aligned             grow strips along one global --heading-deg instead of the local flow (default: flow-aligned)
--heading-deg FLOAT           default auto  (design-surface main slope, cardinal-snapped; global heading only with --no-flow-aligned)
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

# Maneuvering & blade-scale guidance (Stage 7)
--turn-speed-mph FLOAT        default 4     (speed through a turn)
--grade-speed-mph FLOAT       default 8     (grade/compact return pass speed)
--turning-circle-ft FLOAT     default 45    (turn diameter; sets turn-arc length)
--turn-threshold-deg FLOAT    default 60    (gentler bends below this cost no turn time)
--no-grade-return             do not add a grade/compact return pass over each fill
--no-route-smoothing          emit raw grid-stepped waypoints (default: smooth the displayed sweep into a drivable curve; display-only, time unchanged)
--no-knot-repair              leave undriveable knot sweeps as-is (default: re-profile them into boustrophedon parallel passes; §13.3b)
--no-direction-aware          revert to deadhead-distance ordering + 2-opt (no turn penalty)
--lane-overlap FLOAT          default 0.0   (0-0.9 overlap between adjacent blade lanes)
--guidance-csv [PATH]         turn-by-turn waypoints; flag alone -> <out>_guidance.csv
--guidance-png [PATH]         blade lanes over the map; flag alone -> <out>_guidance.png

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

**[15.0.4]** Flags that derive a path from `--out` (`--pairs-csv`, `--pairs-png`, `--guidance-csv`, `--guidance-png`, `--animate-gif`, and `--sequence-csv` when zones are on) shall, when passed without an explicit path, name the file from the `--out` stem.

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
  strips.py       # §10 streamline tracing; flow_direction_field + streamline_to (for §11.0.2c)
  passes.py       # §11 contiguous patch matcher: flow-aligned tube (default) + single-heading strip bias, multi-drop, heading snap, 61 mm layering
  routing.py      # §12 sequencing (NN + bounded 2-opt; direction-aware/grade-aware transition cost)
  sequence.py     # §12.1 zone work order + CSV exports
  timemodel.py    # §13.0 OptiHaul-matched core time model
  swaths.py       # §13.1–§13.6 Stage 7 blade-scale guidance (flow/heading lanes, route smoothing, grade pass, turns, guidance time, CSV)
  render.py       # §14 PNG (cut/fill + arrows), zone overlay, heat map, guidance lanes, frame renderers
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
**[19.0.2]** - ☑ UTM projection and 7.5 ft binning (default) per **§7.1**–**§7.2**.
**[19.0.3]** - ☑ BCY conversions and 61 mm layering per **§7.3**.
**[19.0.4]** - ☑ Volume balance check (and optional offset adjust) per **§8**.
**[19.0.5]** - ☑ Poisson flow field per **§9.1**; min-cost-flow mode per **§9.2**.
**[19.0.6]** - ☑ Streamline strips per **§10**.
**[19.0.7]** - ☑ Contiguous patch passes (≈ 15.4 BCY / 61 mm), true haul leg, heading bias, **flow-aligned tube growth (§11.0.2c, default)**, **strip-biased lanes (§11.0.2b)**, **multi-drop (§11.0.3c)**, **cardinal heading snap (§11.0.3b-i)** per **§11**.
**[19.0.8]** - ☑ Sequencing (NN + bounded 2-opt) and **direction-aware, grade-aware transition cost (§12.0.5)** per **§12**; zone work order + CSV per **§12.1**.
**[19.0.9]** - ☑ OptiHaul-matched core time model per **§13.0**.
**[19.0.9a]** - ☑ Stage 7 blade-scale guidance per **§13.1**–**§13.6**: flow/heading lanes, **route smoothing (§13.3a, display-only)**, grade/compact pass, turning model, multi-drop `MOVE`, guidance time, guidance CSV/PNG.
**[19.0.10]** - ☑ PNG cut/fill, arrows, zone overlay, heat map, animation per **§14**.
**[19.0.11]** - ☑ CLI per **§15**.
**[19.0.12]** - ☑ Tests per **§17**; deterministic output per **§16.4**.
**[19.0.13]** - ☑ All source and this spec stored UTF-8 without BOM.

---

## 20. Summary

**[20.0.1]** This v2 specification defines a Python program that turns an AGD field file plus equipment and time parameters into a haul plan, a rendered PNG, and optional sequencing/CSV/heat-map/animation/guidance outputs. The field is modeled on a **7.5 ft** grid (≈ one blade width; configurable, e.g. 15 ft to match an OptiHaul calculation grid); volumes are tracked in **BCY**, with **swell** applied only to convert the scraper's **LCY** capacity to a bank load limit (20 LCY ÷ 1.3 ≈ 15.4 BCY) and a **cut/fill ratio** for compaction. Haul directions come from an optimal-transport flow field traced into curving strips; the volume plan is produced by a shortest-haul-first matcher that, by **default**, grows each load **along the local transport flow** — cut traced upstream to the ridge, fill downstream into the sink — for long, curved, with-the-grain lanes (`--no-flow-aligned` reverts to one straight **blade-wide strip** along a cardinal-snapped heading via strip bias). The matcher keeps loading/spreading across nearby pockets via **multi-drop** (capped by **`--min-load-fill`** so it does not chase the last few percent into a U-turn blob) and measures the per-pass **haul leg as the true transit** (cut-end → fill-start). Passes are sequenced to minimize non-productive travel using a **direction-aware, grade-aware** transition cost that trades deadhead drive against turning, and costed with an **OptiHaul-matched** core time model. On request, a **blade-scale operator guidance** track (Stage 7) realizes the passes as turn-by-turn waypoints on a **smoothed, drivable GPS route** (corner radius tied to the turning circle; a steering-display transform that never alters distance/time, which stay on the raw coverage geometry), with a **grade/compact return pass** over every fill and a **realistic drivable time** that counts **turning** on the 45 ft circle — giving the honest field time, distinct from the OptiHaul-matched core figure. Sweeps that would still chain into an undriveable **knot** are re-profiled into **boustrophedon** parallel back-and-forth lanes (only where that strictly reduces hairpins, pocket-aware, and without changing any cell, load, haul, or the finished surface; `--no-knot-repair` disables it). Optional outputs also summarise the plan as a **zone work order** (with CSV and numbered PNG waypoints), export **per-pass lat/lon pairs** to CSV, render a **haul heat map** and the **guidance lanes**, and produce a frame-by-frame **leveling animation** in cut/fill or topology mode. The PNG reuses the existing AgGrade palette, arrow glyph, and image geometry so output is directly comparable to the current tools.
