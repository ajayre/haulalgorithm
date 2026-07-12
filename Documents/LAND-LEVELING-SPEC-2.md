# Land Leveling Haul Algorithm Specification â€” v2 **[0.0.1]**

**[0.0.2]** This document is the authoritative specification (revision 2) for the **land leveling haul algorithm**: a Python program that reads an OptiSurface-style **AGD** field file, plans a sequence of cutâ€“haulâ€“fill operations that bring the field to its target grade with minimum haul distance and time, and renders a **PNG** of the cut/fill map with haul-direction arrows, plus optional sequencing CSVs, a haul "heat map" PNG, and a leveling animation.

**[0.0.3]** This v2 supersedes **LAND-LEVELING-SPEC.md** (v0.0.1). It folds in the optimizations and outputs added during implementation: a **7.5 ft** default grid (â‰ˆ one blade width), **contiguous region-grown** cut/fill patches grown as long **blade-wide strips**, **multi-drop** cuts/fills across nearby pockets, the **true transit haul leg** (cut-end â†’ fill-start), an optional **haul heading** bias (snapped to the nearest cardinal axis), an **OptiHaul-matched** time model, **direction-aware** turn-minimizing sequencing, a **blade-scale operator guidance** track (turn-by-turn waypoints, a grade/compact return pass, and a realistic drivable time that counts turning), **zone-level work sequencing**, per-pass **lat/lon CSV** export, a **pass-pairs heat-map** PNG, a frame-by-frame **leveling animation**, **exclusion zones** with **visibility-graph obstacle routing** around no-go polygons (wells, structures), and optional **center-pivot irrigation scheduling** with dynamic-programming arm placement, per-pass haul detours around the parked arm in the guidance track, and a pivot leveling animation (**Â§12.4**). Where v1 and the code disagreeed, the code is treated as authoritative and this document is updated to match.

**[0.0.4]** The problem background and physical model are described in **LAND_LEVELING_PROBLEM.md**; this specification turns that description into an implementation-complete contract.

**[0.0.5]** Every normative paragraph carries a bold marker **`[<S>.<U>.<P>]`** where **`<S>`** is the section, **`<U>`** is the subsection (**`0`** for body text before the first **`###`**), and **`<P>`** is the 1-based paragraph index within that group.

**[0.0.6]** - **Project name:** Land Leveling Haul Algorithm
**[0.0.7]** - **Document type:** Application specification (algorithm + tool), revision 2
**[0.0.8]** - **Language / implementation:** Python 3.10+ (NumPy, Pillow; SciPy and networkx optional)
**[0.0.9]** - **Target platforms:** Windows, Linux, macOS (CPython)
**[0.0.10]** - **Scope:** From a single AGD file plus equipment and time parameters, produce a haul plan, a rendered PNG, and optional CSV/animation outputs. Field design (choosing the target plane) is an upstream concern and is **out of scope** except for the optional global vertical offset in **Â§8**.
**[0.0.11]** - **Related documents:** **LAND_LEVELING_PROBLEM.md** (problem statement), **README.md** (run guide), **ANDY-SPECSTYLEGUIDE.md** (authoring rules). AGD format and rendering conventions derive from **`fielddesign.py`**, **`extract_agd_heights.py`**, **`fieldcreator-checker.py`**, and **`render_existing_heights.py`**.

### 0.1 Specification revision history

**[0.1.1]** | Version | Date | Change |
|---|---|---|
| **0.0.1** | 2026-06-26 | Initial draft: inputs, grid model, six-stage algorithm, PNG output, CLI. |
| **2.0.0** | 2026-06-27 | v2: 15 ft default grid; LCY/BCY clarified; region-grown contiguous passes; true transit haul leg; optional haul heading bias; OptiHaul-matched time model (trapezoidal haul/return, return = forward, field efficiency); zone work-order sequencing + CSV + waypoint overlay; per-pass lat/lon pairs CSV; pass-pairs heat-map PNG; leveling animation (cutfill/topology). |
| **2.1.0** | 2026-06-27 | Self-containment patch for the core route algorithm: specified grid gap-fill (iterative 8-neighbour averaging, Â§7.2.1a); stated explicitly that the route plan is independent of the flow field/strips (Â§5.0.3a, Â§9.0.2, Â§11.0.1); added the matcher state and normative region-growing, sweep-ordering/haul-leg, and fill-seed/heading procedures (Â§11.0.1aâ€“Â§11.0.3b); pinned routing construction + bounded 2-opt parameters (Â§12.0.4). |
| **2.2.0** | 2026-06-27 | Blade-scale execution layer. Default grid **15 â†’ 7.5 ft** (â‰ˆ one blade width, Â§7.2). Strip-biased region growing so each load is one long blade-wide lane (Â§11.0.2b). Multi-drop cuts/fills across nearby pockets within an economical hop (Â§11.0.3c). Heading snapped to the nearest cardinal axis when within tolerance (Â§11.0.3b). Direction-aware, grade-aware sequencing that minimizes turning, not just distance (Â§12.0.5). New **Stage 7 â€” blade-scale operator guidance** (Â§13.1â€“Â§13.6): turn-by-turn waypoints, a grade/compact return pass over every fill, a turning model (45 ft circle, turn speed, turn threshold), multi-drop `MOVE` repositioning, and a realistic drivable time. Tandem 7 ft + 10 ft scrapers modeled as one â‰ˆ 8.5 ft effective blade (Â§6.2). New CLI flags in Â§15. |
| **2.3.0** | 2026-06-28 | **Flow-aligned passes (default).** Each load's cut/fill grows along the local transport flow â€” cut traced *upstream* to the ridge, fill *downstream* into the sink â€” giving long, curved, with-the-grain lanes; the matcher now reads the flow field in this mode (Â§9.0.2, Â§11.0.2c). `--no-flow-aligned` reverts to the single global-heading strip mode of Â§11.0.2b. **Guidance route smoothing (default, display-only):** the displayed sweep coordinates are smoothed into a drivable curve whose corner radius respects the turning circle (Â§13.3a), while all distance/turn/time stay on the raw coverage geometry â€” smoothing must never drop a sweep below the coverage floor `volume Ã· (max_depth Ã— blade_width)`. `--no-route-smoothing` disables it. New CLI flags in Â§15. |
| **2.5.0** | 2026-06-28 | **Spatially-compact loads (no pointless blade-up moves).** The flow-tube grower could integrate a long streamline that picked up a scatter of cells across empty ground, forcing a long blade-up `MOVE` covering nothing. A **compactness guard** (Â§11.0.2c-i) now keeps only the seed-connected fragment (within one multi-drop hop) and refunds the far cells to the ledger for a later local pass (volume conserved, surface unchanged). The lane builders also order multi-fragment sweeps by a **nearest-neighbour chain** instead of raw distance-to-junction (Â§13.3.1b). On TheShop2 this cut total reposition travel â‰ˆ28%, long no-coverage moves â‰ˆ75%, and shaved â‰ˆ1.5% off total time. No new flags (reuses `--max-drop-hop-ft`). |
| **2.4.0** | 2026-06-28 | **Knot re-profiling + load-fullness cap.** Some compact multi-drop loads chain into an undriveable "knot" (a sweep that hairpins back on itself). After sequencing, the guidance now re-profiles such sweeps into a **boustrophedon** â€” parallel blade-width back-and-forth lanes â€” but only where the rendered sweep is still a knot **and** the boustrophedon strictly reduces hairpins, so clean flowing sweeps are never made worse; the re-profile is **pocket-aware** and **conservative** (no bin/load/haul/surface change, only the in-patch path) (Â§13.3b). `--no-knot-repair` disables it. New **`--min-load-fill`** (default 90%) caps multi-drop chasing once a load is "full enough", keeping more loads single clean lanes instead of U-turn blobs (Â§11.0.3c-i). New CLI flags in Â§15. |
| **2.6.0** | 2026-06-28 | **Windowed order refinement (optional, default off).** After routing and knot re-profiling, an optional local search walks the passes one at a time and, for each, relocates it within a window of nearby passes whenever that lowers the order-dependent time (the Â§12.0.5 transition cost). Relocation (or-opt) never reverses a run, so â€” unlike 2-opt (Â§12.0.4) â€” it is safe with the asymmetric direction-aware turn cost, supplying the improvement step that the turn-aware nearest-neighbour construction otherwise lacks. Every accepted move strictly lowers the objective, so the search only ever reduces (never increases) total time, and the refined order flows into the time model, zone sequencing, pair exports, and guidance (Â§12.2). New **`--reorder-window`** flag in Â§15 (default 0 = disabled; typical value 50). |
| **2.7.0** | 2026-06-29 | **Exclusion zones and obstacle-aware routing.** One or more irregular **no-go polygons** (e.g. a well head) may be supplied on the command line. Each polygon is projected to UTM, mapped to the rectangular block of grid bins whose bounding box contains it, and dropped from the field mask â€” no cut/fill is planned inside even if the AGD has bins there. Any resulting cut/fill imbalance is treated as **exported from** (or **imported to**) the field rather than a warning. Blade-up travel legs (loaded **HAUL**, deadhead **RETURN**, multi-drop **MOVE**) route **around** every zone using a **visibility graph + Dijkstra** shortest path on a clearance-buffered polygon (buffer = half **`--equipment-width-ft`**, default 14 ft). The same obstacle distances feed pass haul lengths, the sequencer transition cost, and the guidance track (detour waypoints inserted on travel legs). Optional hatched overlay on plan/guidance PNGs. New **`--exclude-polygon`** (repeatable) and **`--equipment-width-ft`** flags in Â§15; new module **`obstacles.py`** (Â§12.3, Â§16.1). |
| **2.9.0** | 2026-07-11 | **Hard minimum turning-circle bound on guidance.** Gaussian route smoothing remains stair-step display only (Â§13.3a). After smoothing (and even when `--no-route-smoothing` is set), any blade-down corner tighter than `turning_circle/2` is replaced by an R-radius blade-up `MOVE` headland arc so the emitted steering line never violates the machine turning circle. CUT/FILL/GRADE coverage length stays on the raw cell polyline; headland arcs add reposition distance only. |
| **2.9.1** | 2026-07-11 | **In-field keyhole (bulb) headlands for tight 180Â° U-turns.** Near-reverse lane U-turns use a Dubins CCC keyhole/bulb (LRL/RLR). Among feasible CCC candidates the builder **shall** prefer least forward excursion past the inbound headland (stay in-field). One-cell serpentine U-turns are coalesced into a single keyhole. Multi-loop CSC is not used when a CCC keyhole exists. |
| **2.9.2** | 2026-07-11 | **Full-polyline R bound at keyhole entry/exit.** Hard radius enforcement uses a ~15Â° noise floor (not the 60Â° turn-time threshold), denser Dubins sampling, and a post-pass that repairs residual CUTâ†”MOVE splices. Blade-down apexes are never dropped; only blade-up residual junctions may be respliced. |
| **2.9.3** | 2026-07-11 | **Soft bends use short R-fillets; CCC keyholes only for near-reverse.** Soft W pinches no longer pick Walker CCC bulbs (~2Ï€R). Soft bends use a classic constant-R corner fillet when legs are long enough, else same-side CSC (LSL/RSR) capped near one keyhole. Reverse windows within 4R coalesce to one keyhole. Short-leg soft pinches that cannot host an R fillet are not expanded into stacked Dubins loops. |
| **2.9.5** | 2026-07-12 | **Flip out-of-field turns.** When choosing among Dubins CCC (LRL/RLR) or same-side CSC (LSL/RSR) candidates for a headland/transit turn, the builder **shall** prefer the chirality with fewer samples outside the valid field mask; ties break on least forward excursion (CCC) then shortest length. |
| **2.9.6** | 2026-07-12 | **No keyhole from a soft bend.** CCC reverse-window coalesce **shall** start only when the local corner heading change is â‰¥ ~60Â° (same gate as the Dubins dogleg fallback). Soft pinches that trip the R check on short cell legs **shall** try a classic fillet only; they **shall not** open a 4R keyhole to a later reverse. A U-turn exit **shall** require sustained reverse (2+ reverse segments, or one segment â‰¥ ~0.5R), not a one-cell zig-zag. |
| **2.9.8** | 2026-07-12 | **Equipment-length lineup before and after CUT/FILL.** New `--equipment-length-ft` (default **43 ft**) is the tractor+scraper overall length. Before each contiguous blade-down **CUT** or **FILL** run the guidance **shall** include a straight blade-up approach of that length along the outbound heading so the train is aligned when the blade engages; after each run it **shall** include a matching straight blade-up exit along the inbound heading before the next turn/HAUL/MOVE. Coalesced U-turn keyholes **shall** leave the inbound tip on a straight exit spur, finish at the reverse lineup entry, then drive straight onto coverage. Default turning circle is **50 ft** (radius **25 ft**). |
| **2.9.15** | 2026-07-12 | **Fewer turns / lower guidance time defaults.** Default `--min-load-fill` **80%** (was 90) stops lateral multi-drop earlier; default `--reorder-window` **50** (was off) refines pass order for turn-aware time. Knot re-profiling adopts boustrophedon only when U-turn proxy `max(hairpins, lanes−1)` drops, and uses min-lane sweep heading when it beats PCA (§11.0.3c-i, §12.2, §13.3b.2).
| **2.9.14** | 2026-07-12 | **Fewer stacked turn bulbs on compact knots.** Knot re-profiling also keys on unsmoothed near-reverse cell corners (RDP under-counted when blade≈grid). Soft 90° stairs may not spend a full keyhole length budget—only near-reverse corners may. |
| **2.9.13** | 2026-07-12 | **Single-pair PNG: thick strokes are blade-down only.** `render_single_pair_png` / `RenderSinglePairPng` draw thick red for **`CUT`** and thick blue for **`FILL`** only. Blade-up **`HAUL`** / **`MOVE`** / **`START`** / **`RETURN`** stay on the thin GPS route; the black haul arrow and cyan/orange markers use last CUT → first FILL. |
| **2.9.12** | 2026-07-12 | **Extended R-fillets on short cell legs.** Blade-down stair-step corners that cannot host an R fillet on a single cell shall peel earlier/later along the run (or inbound MOVE) and insert an in-field CSC/keyhole so the emitted steering polyline never undercuts R, while still refusing out-of-field samples. |
| **2.9.11** | 2026-07-12 | **In-field guidance turns (hard).** Keyholes, CSC headlands, and cut to fill HAUL transit shall not emit samples outside the valid field mask; side-arc keyholes are preferred over tip CCC, U-turn lineup tips are clamped to in-field rays, and out-of-field candidates are rejected. |
| **2.9.10** | 2026-07-12 | **Default grid and blade width 8.5 ft.** Default `--grid-size-ft` and effective blade width are **8.5 ft** (`--blade-width-mm` **2590.8**) to match the tandem scraper volume-weighted width. |
| **2.9.7** | 2026-07-12 | **In-field side-arc keyholes.** When a tip Dubins CCC leaves the field (or is longer than needed), the builder **shall** prefer a constant-R side arc (optional short forward spur, arc into available field, reconnect) scored by outside-count then length. Forward-slid CCC remains a fallback. Dubins word ties break on outside-count then length. |
| **2.8.0** | 2026-06-29 | **Center-pivot irrigation scheduling (optional, default off).** A center-pivot system anchored in the field (default: valid-cell centroid; override with **`--pivot-center "lat,lon"`**) is sized as large as fits inside the boundary. A **dynamic-programming** scheduler finds the arm-bearing sequence that minimises `move_cost Ã— (repositions) + (detoured passes)` over the executed pass order (**Â§12.4**). Blade-up haul/return legs that would cross the parked arm are routed around it in the guidance track (same visibility-graph mechanism as **Â§12.3**, per pass, per arm position); extra loaded-haul distance is added to the OptiHaul time report. Outputs: move schedule CSV, per-position PNG maps, and a leveling animation with a held frame at every arm move. New **`--pivot` / `--no-pivot`**, **`--pivot-center`**, **`--pivot-wheelbase-ft`**, **`--pivot-clearance-ft`**, **`--pivot-move-cost`**, and animation/output flags in **Â§15**; new module **`pivot.py`** (**Â§12.4**, **Â§14.7**, **Â§16.1**). |

**[0.1.2]** This document follows [semantic versioning](https://semver.org/): **MAJOR** = incompatible normative change, **MINOR** = backward-compatible addition, **PATCH** = clarification.

---
## 1. Purpose

**[1.0.1]** The land leveling haul algorithm shall compute, for one earthmoving machine, an ordered set of cutâ€“haulâ€“fill **passes** that bring every field **area** within a configured elevation **tolerance** of its target surface, while minimizing total **haul distance** and, secondarily, total **time**.

**[1.0.2]** The program shall accept its field input exclusively as an **AGD file** (**Â§6.1**) and shall accept equipment, time, and output parameters as described in **Â§6.2**â€“**Â§6.4** and **Â§15**.

**[1.0.3]** The program shall produce, as its primary deliverable, a **PNG image** containing a **cut/fill map** and overlaid **haul-direction arrows** (**Â§14**). It shall additionally produce, on request, the sequencing, CSV, heat-map, animation, and **center-pivot** outputs of **Â§14.4**â€“**Â§14.7** and **Â§12.1**, **Â§12.4**.

**[1.0.4]** This specification is written for both human maintainers and agentic implementers; obligations use **shall**, **should**, **may** as defined in **ANDY-SPECSTYLEGUIDE.md Â§6**.

---

## 2. Design goals

**[2.0.1]** The implementation **shall provide**:

**[2.0.2]** 1. Deterministic, reproducible output for a given AGD file and parameter set (**Â§16.4**).
**[2.0.3]** 2. A single-file AGD reader that mirrors the parsing rules of the existing tools (**Â§6.1**).
**[2.0.4]** 3. A grid model derived from the AGD points (default **8.5 ft** areas, matching the default blade width), decoupled from blade width (**Â§7**).
**[2.0.5]** 4. Volume accounting in **banked cubic yards (BCY)** for cut supply, fill demand, and transported volume; swell applies only when converting scraper **LCY** capacity to a bank load limit; compaction via the cut/fill ratio (**Â§7.3**).
**[2.0.6]** 5. A global haul **flow field** computed by optimal transport, for haul arrows and strip visualization (**Â§9**).
**[2.0.7]** 6. **Strips** traced as streamlines of the flow field, from ridges to the cut/fill transition (**Â§10**).
**[2.0.8]** 7. **Passes** packed to the scraper load limit (20 LCY â‰ˆ 15.4 BCY) under the 61 mm depth-per-pass limit, each cutting and filling a **compact, contiguous patch** (**Â§11**).
**[2.0.9]** 8. A pass **sequence** that minimizes non-productive (deadhead) return travel and respects layer ordering (**Â§12**).
**[2.0.10]** 9. A **time estimate** matching the OptiSurface OptiHaul methodology (trapezoidal haul/return speeds, return distance = forward distance, field efficiency) (**Â§13**).
**[2.0.11]** 10. A **PNG renderer** whose cut/fill palette, arrow glyphs, and image geometry match the existing AgGrade tools (**Â§14**).
**[2.0.12]** 11. Optional **zone work-order sequencing**, **per-pass lat/lon CSV**, a **pass-pairs heat-map PNG**, and a **leveling animation** (**Â§12.1**, **Â§14.4**â€“**Â§14.6**).
**[2.0.13]** 12. A documented **command-line interface** (**Â§15**).
**[2.0.14]** 13. A **blade-scale operator guidance** track that turns the optimized passes into turn-by-turn waypoints (action, blade up/down, target depth, next bearing, instruction) plus a **realistic drivable time** that â€” unlike the OptiHaul-matched core model â€” counts **turning** and a **grade/compact return pass** over every fill (**Â§13.1**â€“**Â§13.6**). The guidance shall not worsen the optimized passes' haul/time; it realizes them at blade scale.
**[2.0.15]** 14. Optional **exclusion zones**: one or more irregular no-go polygons (e.g. a well head) that are neither levelled nor driven through; blade-up travel routes around them via a visibility graph (**Â§12.3**).
**[2.0.16]** 15. Optional **center-pivot irrigation scheduling**: a pivot anchored in the field (default centroid, user-specified lat/lon optional) and sized as large as fits, with a dynamic-programming schedule of arm bearings held across the leveling passes, per-pass haul detours around the parked arm in the guidance track, and optional schedule CSV / PNG maps / leveling animation (**Â§12.4**, **Â§14.7**).

## 3. Non-goals

**[3.0.1]** The implementation **shall not** attempt the following in this version:

**[3.0.2]** 1. Choosing or optimizing the **target surface** (plane fitting, slope/heading solve). The target is read from the AGD proposed elevations; see **Â§8** for the only permitted exception. (The haul **heading** in **Â§11.0.3b** biases *fill selection* only; it does not alter the target surface.)
**[3.0.3]** 2. Multi-machine fleet scheduling.
**[3.0.4]** 3. Modeling machine traction or fuel. (The **turning circle** *is* modeled â€” but only in the blade-scale guidance time and the direction-aware sequencer, **Â§13.1**, **Â§12.0.5** â€” not in the core OptiHaul-matched time of **Â§13.0**.)
**[3.0.5]** 4. Reading any input from a database; the AGD path replaces the legacy SQLite path.
**[3.0.6]** 5. Producing GIS/shapefile or 3D outputs.

---

## 4. Terminology

### 4.1 Area (cell, bin)

**[4.1.1]** An **area** is one square grid cell of the field. The default cell is **8.5 ft Ã— 8.5 ft** (matches the default blade width) (**Â§7.2**); it is configurable. Each area carries one existing and one target elevation and is the atomic unit of cut, fill, and haul.

### 4.2 Cut and fill

**[4.2.1]** **Cut** is soil removed where existing exceeds target; **fill** is soil placed where existing is below target. Internally **`delta = existing âˆ’ target`** is positive for cut, negative for fill.

### 4.3 BCY, LCY, swell, compaction

**[4.3.1]** **BCY** (banked cubic yards) is in-ground volume â€” the unit for mass balance, cut supply, fill demand, and transported volume. **LCY** (loose cubic yards) is the loose, in-scraper volume. The **swell factor** relates them: **`LCY = BCY Ã— swell`** (equivalently **`BCY = LCY Ã· swell`**).

**[4.3.2]** Scraper capacity is specified in **LCY**. The per-pass bank load limit is **`scraper_LCY Ã· swell`** (e.g. 20 LCY Ã· 1.3 â‰ˆ **15.4 BCY**). Swell shall **not** be applied to cut supply or fill demand.

**[4.3.3]** The **cut/fill ratio** (compaction) accounts for fill compacting when placed: creating one bank cubic yard of fill space requires delivering **`cutfill_ratio`** bank cubic yards of cut soil (default 1.2).

### 4.4 Pass, segment, haul leg

**[4.4.1]** A **pass** is one cutâ€“haulâ€“fill cycle. Its **cut segment** is the ordered set of cut cells (with depths) swept while loading; its **fill segment** is the ordered set of fill cells swept while dumping. The **haul leg** is the transit from the cut-segment **end** to the fill-segment **start** (**Â§11.0.3**). When exclusion zones are active (**Â§12.3**), the haul leg length is the **shortest obstacle-avoiding path** around every zone, not the straight-line distance.

### 4.5 Strip

**[4.5.1]** A **strip** is a continuous centerline streamline of the flow field, used for haul-direction visualization. Strips may curve and are not restricted to cardinal directions.

### 4.6 Flow field and haul arrows

**[4.6.1]** The **flow field** is a per-area unit direction (plus magnitude) giving where soil should move. A **haul arrow** is a rendered glyph of that direction (**Â§14.2**).

### 4.7 Ridge and transition

**[4.7.1]** A **ridge** is a local high of `delta` (a cut source). The **transition** is where existing crosses target (cut switches to fill).

### 4.8 Tolerance

**[4.8.1]** The **tolerance** is the maximum absolute deviation from target for an area to count as finished (default 10 mm).

### 4.9 Heading

**[4.9.1]** The **haul heading** is a compass bearing (0 = N, 90 = E) along which soil is preferentially moved. Default is the design surface's main down-slope, auto-detected from the target gradient (**Â§11.0.3b**).

### 4.10 Zone

**[4.10.1]** A **zone** is a spatial cluster of cut work used only for the human-readable rough **work order** (**Â§12.1**). Zones are a reporting summary of the per-pass route, not a separate operational constraint.

### 4.11 Strip bias and blade-wide lane

**[4.11.1]** **Strip bias** is the anisotropy applied during region growing (**Â§11.0.2b**) so a load's cut (or fill) patch grows as one long, **single-blade-wide lane** along the heading rather than a compact blob. A **lane** is the resulting near-1-cell-wide run the operator drives in a straight line, minimizing weaving and turns.

### 4.12 Pocket and multi-drop

**[4.12.1]** A **pocket** is a connected island of remaining cut (or fill) material. **Multi-drop** is the behavior whereby, when a load cannot be filled (or emptied) within one pocket, the machine drives **blade up** to the next pocket within an **economical hop** (`max_drop_hop`, **Â§6.4**) and continues loading (or spreading) the same load, instead of ending the pass part-full and paying for a whole separate cutâ€“haulâ€“fill cycle later.

### 4.13 Grade/compact return pass

**[4.13.1]** A **grade/compact return pass** is one return traverse back over a just-placed fill (blade down) to grade and compact it (**Â§13.2**). It is required finishing work; the core OptiHaul-matched time (**Â§13.0**) does not include it, the blade-scale guidance time (**Â§13.1**) does.

### 4.14 Turning circle and turn threshold

**[4.14.1]** The **turning circle** is the tractor + scraper turn diameter (default 50 ft; radius 25 ft); its radius sets the arc length `RÂ·Î¸` travelled at the reduced **turn speed** during a heading change of `Î¸`. A heading change below the **turn threshold** (default 60Â°) is a gentle bend driven in stride at travel speed (no slowdown); only sharper turns cost turn time (**Â§13.1**).

### 4.15 Guidance track and waypoint

**[4.15.1]** The **guidance track** is the ordered list of blade-scale **waypoints** produced by Stage 7 (**Â§13.3**). Each waypoint carries an **action** (`START`, `CUT`, `HAUL`, `MOVE`, `FILL`, `GRADE`, `RETURN`), a blade up/down state, world coordinates, a target depth, the bearing to the next waypoint, and a human instruction.

### 4.16 Flow-aligned lane (streamline tube)

**[4.16.1]** A **flow-aligned lane** is a cut or fill patch grown along the **local transport flow** (a streamline of the flow field, **Â§9**) rather than a fixed compass heading. The cut lane is traced **upstream** (toward the ridge the soil is drawn from) and the fill lane **downstream** (into the sink it feeds), so cuts, hauls and fills become long, curved, *with-the-grain* runs. Flow-aligned growth is the **default**; **`--no-flow-aligned`** selects the single global-heading strip of **Â§4.11** instead.

### 4.17 Route smoothing (steering line)

**[4.17.1]** **Route smoothing** first rounds a sweep's **displayed** waypoint coordinates to remove grid stair-steps (Gaussian, **Â§13.3a.1**). A second hard pass then enforces the machine **turning circle**: any blade-down corner tighter than `turning_circle/2` becomes an R-radius blade-up `MOVE` headland â€” a compact keyhole/bulb (Dubins CCC) for ~180Â° U-turns (**Â§13.3a.3**) â€” and sharp cutâ†’fill **`HAUL`** / multi-drop **`MOVE`** chords get R-bounded transit arcs. A third pass inserts the equipment-length straight lineup before and after each CUT/FILL run (**Â§13.3a.4**). Coverage length accounting stays on the raw cell geometry; headland/HAUL arcs add reposition only.

### 4.18 Exclusion zone

**[4.18.1]** An **exclusion zone** is an irregular polygon (given as lat/lon vertices) that the machine may **neither level nor drive through** â€” e.g. a well head, structure, or other permanent obstacle. Its bins are dropped from the field mask (**Â§7.2.8**); cut/fill work may abut the cleared perimeter but never occurs inside the zone.

### 4.19 Clearance buffer and obstacle field

**[4.19.1]** For routing, each exclusion polygon is offset outward by a **clearance** equal to half the configured **equipment width** (`--equipment-width-ft`, default 14 ft), so the machine **body**, not just the GPS centerline, stays clear of the zone. The set of buffered polygons forms an **obstacle field**; blade-up travel legs (**Â§12.3**) use a **visibility graph + Dijkstra** shortest path that never crosses any buffered interior.

### 4.20 Center pivot (irrigation arm)

**[4.20.1]** A **center pivot** is a linear irrigation arm anchored at a point in the field and rotating about that anchor. In this specification the arm is modeled as a **wheelbase-wide bar** one **radius** long (anchor â†’ tip), parked at a compass **bearing** while the field is leveled. The pivot does not level soil; it is a **temporary obstacle** the scraper must not drive through while the arm is parked there.

**[4.20.2]** A **pivot position** is a contiguous run of executed passes during which the arm is held at one bearing. A **pivot move** is the rotation from one position's bearing to the next. A **routed haul detour** is a loaded **HAUL** leg (or its mirrored **RETURN**) whose straight line crosses the parked arm but whose visibility-graph path around the arm is longer; **work under the arm** is cut/fill activity in cells beneath the arm where no bearing clears the work (cannot be rerouted â€” the arm must move or those cells be skipped).

---
## 5. High-level architecture

**[5.0.1]** The program is a pipeline; each stage consumes the previous stage's output and is independently testable.

**[5.0.2]**

```text
AGD file           â”€â”€â–º [1] Parse + grid (8.5 ft)         agd.py, grid.py   Â§6, Â§7
          â”€â”€â–º     (optional) exclusion mask      grid.py, obstacles.py  Â§7.2.8, Â§12.3
          â”€â”€â–º [2] Volume balance (check/adjust)  balance.py        Â§8
          â”€â”€â–º [3] Flow field (optimal transport) flowfield.py      Â§9
          â”€â”€â–º [4] Strips (streamlines)           strips.py         Â§10
          â”€â”€â–º [5] Passes (â‰ˆ15.4 BCY / 61 mm)     passes.py         Â§11
          â”€â”€â–º [6] Sequence + route               routing.py        Â§12
          â”€â”€â–º     (optional) pivot schedule      pivot.py          Â§12.4
          â”€â”€â–º [7] Time estimate                  timemodel.py      Â§13
          â”€â”€â–º [8] Blade-scale guidance (opt.)    swaths.py         Â§13.1â€“Â§13.6
          â”€â”€â–º PNG: cut/fill map + haul arrows    render.py         Â§14
          â”€â”€â–º (optional) zones / CSVs / heat map sequence.py, render.py  Â§12.1, Â§14.4â€“14.5
          â”€â”€â–º (optional) guidance CSV / PNG      swaths.py, render.py  Â§13.6
          â”€â”€â–º (optional) leveling animation      animate.py        Â§14.6
          â”€â”€â–º (optional) pivot CSV / maps / GIF  pivot.py          Â§12.4, Â§14.7
```

**[5.0.3]** Equipment (**Â§6.2**) and time/motion (**Â§6.3**) parameters feed stages [5]â€“[7]. The cut/fill map depends only on stages [1]â€“[2]; the haul arrows depend on stage [3].

**[5.0.4]** *Flow-field dependency depends on the pass mode.* Under **`--no-flow-aligned`** (the single global-heading mode, **Â§11.0.2b**) the optimized haul-route plan does **not** depend on the flow field or strips: stages [3] (flow field, **Â§9**) and [4] (strips, **Â§10**) exist only for haul-direction visualization, and the pass plan in stage [5] (**Â§11**) is computed directly from the per-area cut/fill geometry â€” a nearest-fill anchor plus distance-ordered region growing. An implementer who only wants the single-heading routes/sequence/time may therefore **skip Â§9 and Â§10**. In the **default flow-aligned mode** (**Â§11.0.2c**), however, the pass grower **does** consume the flow field (stage [3]) to grow each load's cut/fill along the local transport streamline, so stage [3] is then a required input to the plan (strips, **Â§10**, remain visualization-only).

**[5.0.5]** Stages [1]â€“[2] (plus optional [3] for arrows) fully determine the base PNG and shall be runnable without stages [4]â€“[7]; conversely the pass/sequence/time plan (stages [5]â€“[7]) shall be runnable without stages [3]â€“[4]. The pass list, time, and all optional outputs are additive. The CLI exposes the PNG-only path as `--no-plan`.

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

**[6.1.11]** Sign convention (normative): on disk **`CutFill = Proposed âˆ’ Existing`**, so negative CutFill is a **cut** and positive is a **fill**. Internally the algorithm shall use **`delta = Existing âˆ’ Proposed`** (positive = cut). The reader shall recompute **`delta`** from columns [2] and [3] and shall not trust column [4].

**[6.1.12]** Boundary handling: if **`2PER`** rows are present they define the field perimeter polygon; areas whose centers fall outside it shall be invalid and excluded everywhere. If no **`2PER`** rows are present, the field mask shall be inferred from the observed points (row/column span fill, hole fill, largest connected component), consistent with **`fielddesign.py`**.

**[6.1.13]** The reader shall also expose the inverse projection (UTM â†’ lat/lon, `latlon_from_utm`), the forward ring projection (`project_latlon_ring`: a list of lat/lon pairs â†’ UTM metres, **Â§12.3**), and the field's UTM **zone** and **hemisphere**, so downstream stages can emit geographic coordinates in the CSV outputs (**Â§14.4**) and parse exclusion polygons.

### 6.2 Equipment specifications (normative)

**[6.2.1]** Defaults match **LAND_LEVELING_PROBLEM.md Â§4** and shall be overridable on the command line.

**[6.2.2]** | Parameter | Default | Meaning |
|---|---|---|
| **Scraper capacity** | 20 LCY | Loose volume per pass; bank limit = 20 Ã· swell â‰ˆ 15.4 BCY. |
| **Swell factor** | 1.3 | `LCY = BCY Ã— 1.3` (`BCY = LCY Ã· 1.3`). |
| **Cut/fill ratio** | 1.2 | Bank cut soil delivered per bank cubic yard of fill space (compaction). Alias: `--compaction`. |
| **Max depth per pass** | 61 mm | Maximum cut or fill depth in one area for one pass (0.2 ft). |
| **Blade width** | 2590.8 mm | Cutting/filling blade width (default **8.5 ft** tandem volume-weighted effective width: `1/W = 0.4/7 + 0.6/10`). |
| **Blade height control** | 10 Hz | Blade height update rate (informative). |
| **Turning circle** | 50 ft | Tractor + scraper turn diameter (radius 25 ft); radius sets the turn-arc length in the guidance time and sequencer (**Â§13.1**, **Â§12.0.5**). |
| **Turn speed** | 4 MPH | Speed through a turn (the machine slows to turn). |
| **Grade speed** | 8 MPH | Speed of the grade/compact return pass over a fill (**Â§13.2**). |
| **Turn threshold** | 60Â° | Heading changes below this are gentle bends driven in stride (no slowdown); only sharper turns cost turn time (**Â§13.1**). |
| **Flow-aligned passes** | on | Default: grow each load's cut/fill along the local transport flow (cut upstream, fill downstream) for curved, with-the-grain lanes (**Â§11.0.2c**). `--no-flow-aligned` selects the single global-heading strip mode below. |
| **Haul heading** | auto | Compass bearing soil is hauled along; default is the design surface's main down-slope, **snapped to the nearest cardinal axis** when within tolerance. The global strip heading under `--no-flow-aligned`, and a fallback where the flow is flat; biases fill selection (**Â§11.0.3b**). |
| **Heading strength** | 0.5 | 0â€“0.95; how strongly fill selection follows the heading (0 = off). |
| **Strip bias** | 50 | Single-heading mode only: anisotropy of region growing â€” grow each load as one long blade-wide lane along the heading (**Â§11.0.2b**); 1 = compact blob. |
| **Multi-drop hop** | 100 ft | Max blade-up drive to the next pocket to keep loading/spreading the same load (**Â§11.0.3c**); 0 disables multi-drop. |
| **Minimum load fill** | 90% | Drivability vs load-fullness trade-off: once a load reaches this fraction of capacity, further multi-drop hops are skipped so it stays a single clean lane (**Â§11.0.3c-i**); 100 = chase every load to full. |
| **Knot re-profiling** | on | Re-profile undriveable knot sweeps into boustrophedon parallel passes after sequencing (**Â§13.3b**); `--no-knot-repair` disables it. |
| **Equipment length** | 43 ft | Tractor+scraper overall length; straight blade-up lineup before and after each CUT/FILL run (Â§13.3a.4). |
| **Equipment width** | 14 ft | Machine width used as the clearance buffer around exclusion zones: each no-go polygon is offset outward by **half** this value for obstacle routing (**Â§12.3**). Also used as the clearance buffer around the parked pivot arm (**Â§12.4**). |
| **Pivot wheelbase** | 14 ft | Center-pivot arm width (**Â§12.4**); `--pivot-wheelbase-ft`. |
| **Pivot clearance** | 25 ft | Extra clearance per side between the parked arm and active leveling work, on top of half the wheelbase (**Â§12.4**); `--pivot-clearance-ft`. |
| **Pivot move cost** | 25 passes | Cost of one arm reposition, expressed as the number of single-pass haul detours it is worth avoiding (**Â§12.4**); `--pivot-move-cost`. |

### 6.3 Time and motion parameters (normative)

**[6.3.1]** The time model (**Â§13**) consumes these; defaults apply when not supplied and shall be overridable (**Â§15**).

**[6.3.2]** | Parameter | Default | Unit | Meaning |
|---|---|---|---|
| **Loading speed** | 4 | MPH | Constant travel speed while cutting. |
| **Haul speed** | 10 | MPH | Maximum travel speed from cut to fill (trapezoidal profile). |
| **Dumping speed** | 5 | MPH | Constant travel speed while filling. |
| **Return speed** | 13 | MPH | Maximum deadhead return speed (trapezoidal profile). |
| **Acceleration time** | 12 | s | Time to reach max speed on a haul/return move. |
| **Deceleration time** | 12 | s | Time to stop at the end of a haul/return move. |
| **Field efficiency** | 0.8 | â€“ | Productive fraction of total time; "Other" time = total Ã— (1 âˆ’ eff). |

### 6.4 Configuration and output parameters (normative)

**[6.4.1]** | Parameter | Default | Meaning |
|---|---|---|
| **Grid size** | 8.5 ft | Edge length of each square area (**Â§7.2**); matches the default blade width so each load is one blade-wide lane. Use 15 ft to match an OptiHaul coarse calculation grid. |
| **Lane overlap** | 0.0 | 0â€“0.9 fractional overlap between adjacent blade lanes in the guidance (**Â§13.3**); 0 = lanes butt edge-to-edge. |
| **Tolerance** | 10 mm | Finished-area elevation tolerance (**Â§4.8**). |
| **Pixels per bin** | 50 | PNG pixels per area edge for the main plan PNG (**Â§14.3**). |
| **Arrow stride** | 2 | Draw a haul arrow every N areas (0 disables). |
| **Transport mode** | `poisson` | `poisson` (LÂ²) or `mincostflow` (LÂ¹) flow field (**Â§9**). |
| **Balance mode** | `check` | `check` (warn) or `adjust` (re-level target, **Â§8**). |
| **Exclusion polygons** | (none) | Zero or more no-go polygons, each â‰¥ 3 lat/lon vertices (**Â§7.2.8**, **Â§12.3**); repeat `--exclude-polygon` for multiple zones. |
| **Pivot analysis** | off | Center-pivot scheduling, detour routing, CSV/PNG/animation outputs (**Â§12.4**); `--pivot` / `--no-pivot`. |
| **Pivot center** | field centroid | WGS-84 lat/lon anchor for the pivot (**Â§12.4**); `--pivot-center "lat,lon"`. |
| **Zones** | 8 | Number of cut zones for the rough work order (0 disables) (**Â§12.1**). |
| **Seed** | 1234 | Random seed for deterministic zone clustering (**Â§16.4**). |

**[6.4.2]** The optional output toggles (sequence CSV, pairs CSV, pairs PNG, **guidance CSV/PNG**, animation directory/GIF, animation mode/stride/pixels-per-bin, **pivot CSV/PNG maps/animation**) are defined with the CLI in **Â§15** and the outputs in **Â§12.1**, **Â§12.4**, **Â§13.6**, **Â§14.4**â€“**Â§14.7**. The maneuvering parameters (turn speed, grade speed, turning circle, turn threshold) and the strip-bias / multi-drop / lane-overlap controls are listed in **Â§6.2** and exposed in **Â§15**.

---

## 7. Coordinate and grid model

### 7.1 Projection

**[7.1.1]** The reader shall project each AGD point's lat/lon to a local planar (UTM, WGS-84) system in metres, with the field's south-west corner as the local origin (`utm_from_latlon`). A field crossing a UTM zone boundary shall be rejected with a clear error.

**[7.1.2]** The inverse projection (`latlon_from_utm`) shall be provided for CSV outputs (**Â§14.4**) and shall be consistent with **Â§14.3** geometry.

### 7.2 Binning to the grid

**[7.2.1]** The field bounding rectangle shall be partitioned into square areas of edge **`grid_size`** (default **8.5 ft = 2.5908 m**). The 8.5 ft default matches the tandem scraper volume-weighted effective blade width (2590.8 mm), making each scraper load **one continuous blade-wide lane** rather than a multi-lane patch that would force tight turns (**Â§11.0.2b**, **Â§13.3**); the grid size is configurable (**Â§6.4**) and coarser grids (e.g. 15 ft to match an OptiHaul calculation grid, or 30 ft) run faster. Each AGD interior point is assigned to its containing area (floor of `(coord âˆ’ min) / grid_size`, clamped to the grid); an area's existing and target elevations are the **mean** of the points it contains. Areas outside the mask are invalid.

**[7.2.2]** *Gap filling (normative).* Areas that are inside the field mask but contain no points shall be filled by **iterative nearest-neighbour averaging**, applied to the existing and target grids in parallel:

**[7.2.3]**

```text
repeat up to 2 * max(width, height) times, until no cell is still pending:
    for each pending (masked, still-empty) cell:
        let n = its 8-connected neighbours that already have a finite value
        if n is non-empty:
            cell.existing = mean(existing of n)
            cell.target   = mean(target   of n)
            mark cell filled (no longer pending)
```

**[7.2.4]** Each iteration uses only values that were finite at the **start** of that iteration (the four cardinal and four diagonal neighbours), so the fill is a deterministic flood inward from the observed cells. A cell that never acquires a finite neighbour (e.g. an isolated masked region) remains invalid.

**[7.2.5]** Each area's existing and target elevations shall be treated as **uniform over the whole cell** (no intra-cell slope). All volume, depth, cut/fill, and rendering computations use these per-cell constants.

**[7.2.6]** The grid shall be indexed by integer `(bx, by)` with `bx` increasing east and `by` increasing north. Image row order is defined in **Â§14.3**. The grid shall expose a `cell_center_xy(bx, by)` helper returning the area-center world coordinates used throughout stages [5]â€“[7] and the CSV/animation outputs.

**[7.2.7]** Blade width shall not affect the grid size; each area is atomic for cut, fill, and haul.

**[7.2.8]** *Exclusion mask (normative).* When one or more **exclusion polygons** are supplied (**Â§6.4**, **Â§12.3**), each polygon shall be projected to UTM using the same projection as the field (**Â§7.1**). An irregular polygon shall be mapped to the **rectangular block of bins** spanning its projected axis-aligned bounding box: bin indices from `floor((min âˆ’ origin) / grid_size)` through `floor((max âˆ’ origin) / grid_size)` on each axis, clamped to the grid. That block's bounding box entirely contains the polygon, so every bin the polygon touches â€” and any rounding-out to a rectangle â€” is excluded. Those bins shall be removed from `valid`; their `existing` and `target` shall be set non-finite so they contribute neither cut nor fill. The projected rings shall be stored on the grid for downstream routing and rendering. No cut/fill pass may consume an excluded bin.

### 7.3 Cell quantities and BCY conversion (normative)

**[7.3.1]** For each valid area, let `delta = existing âˆ’ target` (m, positive = cut) and `A = grid_sizeÂ²` (mÂ²). `M3_TO_CY = 1.30795061931439` converts mÂ³ to ydÂ³.

**[7.3.2]** Cut areas (delta > 0):

**[7.3.3]**

```text
in_place_m3 = delta * A
supply_BCY  = in_place_m3 * M3_TO_CY      # bank cut volume available (BCY)
```

**[7.3.4]** The bank cut volume is BCY directly â€” swell does **not** enter cut supply or fill demand. Swell applies **only** to the scraper LCY â†’ bank limit conversion: `load_limit_BCY = scraper_LCY Ã· swell`.

**[7.3.5]** Fill areas (delta < 0): bank cut volume to deliver to raise the surface by `-delta`, with compaction via the cut/fill ratio:

**[7.3.6]**

```text
fill_space_m3 = (-delta) * A
demand_BCY    = fill_space_m3 * M3_TO_CY * cutfill_ratio   # bank cut volume to deliver
```

**[7.3.7]** All transport (stages [3]â€“[6]) shall be expressed in **BCY**. The 61 mm depth-per-pass limit shall be applied per area as `ceil(|delta| / max_depth)` layers, with a partial final layer allowed.

**[7.3.8]** These conversions shall be pure functions with unit tests (**Â§17**), including the LCY â†’ BCY scraper conversion (`lcy_to_bcy`).

---
## 8. Stage 1 â€” Volume balance

**[8.0.1]** Because the target is supplied by the AGD file, total cut and fill (BCY) are normally consistent. The implementation shall still compute global `Î£ supply_BCY` and `Î£ demand_BCY` and report the relative imbalance.

**[8.0.2]** In **`balance_mode = check`** (default) it shall warn if the relative imbalance exceeds a small threshold (default 2%) and continue. When one or more exclusion zones are active, the imbalance threshold shall be suppressed and any net surplus cut or fill deficit shall be reported as **exported from** or **imported to** the field respectively â€” the removed bins broke the field's volume closure and that material is assumed to leave or enter at the boundary.

**[8.0.3]** In **`balance_mode = adjust`** it **may** apply a single global vertical offset to the target to drive `Î£ supply_BCY` toward `Î£ demand_BCY` (bisection on the offset), reporting the chosen offset. Tilting/re-fitting the plane is out of scope (**Â§3**). Exclusion-induced imbalance is not corrected by offset alone; the export/import reporting of **Â§8.0.2** still applies.

---

## 9. Stage 2 â€” Global flow field (optimal transport)

**[9.0.1]** The flow field determines haul directions for the arrows and strips. The implementation shall support two modes via **`transport_mode`**.

**[9.0.2]** *Required for flow-aligned passes; otherwise visualization only.* Under `--no-flow-aligned` this stage and the strips of **Â§10** are **not** inputs to the pass plan (**Â§5.0.3a**) â€” they only produce the haul-direction overlay, and may be omitted if only the single-heading routes/time are needed. In the default **flow-aligned** mode (**Â§11.0.2c**) the pass grower reads this flow field to trace each load's cut/fill along the local streamline, so it is then a required stage.

### 9.1 Poisson potential mode (default)

**[9.1.1]** Define a signed source density `s` over valid areas: `s = supply_BCY` for cut, `s = âˆ’demand_BCY` for fill. Solve

**[9.1.2]**

```text
laplacian(phi) = s      with Neumann (zero-flux) boundary on the field mask
```

**[9.1.3]** and set `F = âˆ’grad(phi)`; soil flows from high `phi` (ridges) toward low `phi` (sinks). The system is assembled over valid areas only and solved with a sparse solver (SciPy if available, else a NumPy iterative fallback). The potential **may** be smoothed before differentiation to yield cleaner streamlines (**Â§10**).

**[9.1.4]** This mode minimizes a quadratic (LÂ²) transport cost; it is fast and yields smooth, curving haul directions.

### 9.2 Min-cost flow mode

**[9.2.1]** When **`transport_mode = mincostflow`**, solve the balanced transportation problem minimizing `Î£ fÂ·d` on the grid graph for the distance-optimal (LÂ¹) flow; the net per-area flow direction is the flow field. If the optional dependency (**networkx**) is unavailable, the implementation shall fall back to `poisson` with a warning.

### 9.3 Flow field output

**[9.3.1]** The flow field shall be a per-area unit direction plus magnitude. Areas with negligible magnitude (at grade) shall be marked as having no arrow. The flow field feeds the haul-arrow overlay (**Â§14.2**) and strip tracing (**Â§10**).

---

## 10. Stage 3 â€” Strip generation (visualization)

**[10.0.1]** Strips shall be traced as streamlines of the flow field, starting at ridges (local maxima of `phi` or highest-`delta` cut areas) and integrating along `F` with bilinear interpolation so paths curve smoothly. A smoothed potential **may** be used for tracing.

**[10.0.2]** A streamline shall be followed across the cut region, through the transition, and into the fill region, capturing both source and sink of a haul.

**[10.0.3]** Tracing shall terminate when a streamline leaves the mask, reaches an area with no remaining fill capacity, or forms a closed loop.

**[10.0.4]** Strips are used for haul-direction visualization and as informative seeds; the authoritative volume plan is produced by the pass matcher (**Â§11**), which does not require strips to pack full loads.

---

## 11. Stage 4 â€” Pass generation

**[11.0.1]** Passes shall accumulate volume in BCY subject to **Â§6.2**, packing each pass to the scraper load limit (`scraper_LCY Ã· swell` â‰ˆ 15.4 BCY) under the 61 mm depth-per-pass cap. The volume plan is produced by the **shortest-haul-first, contiguous patch matcher** below. Two patch-growing modes are available: the **flow-aligned** grower (**Â§11.0.2c**, default) traces each load's cut/fill along the local flow streamline (**Â§9**); the **single-heading** grower (**Â§11.0.2a**â€“**Â§11.0.2b**, `--no-flow-aligned`) operates **solely** on the per-area cut/fill ledger (`delta` and the BCY conversions of **Â§7.3**) and consumes neither the flow field nor strips. In the single-heading mode this section together with **Â§7** is sufficient to reproduce the haul-route plan; in the flow-aligned mode the flow field of **Â§9** is an additional required input.

**[11.0.2]** *State (normative).* The matcher maintains two non-negative per-area ledgers in metres of remaining depth: `remaining_cut[by,bx] = max(delta, 0)` and `remaining_fill[by,bx] = max(âˆ’delta, 0)` over valid areas (zero elsewhere). The per-metre BCY rates are `cut_bcy_per_m = A Â· M3_TO_CY` and `fill_bcy_per_m = A Â· M3_TO_CY Â· cutfill_ratio` (**Â§7.3**). A cell is "still holds material" when its remaining depth exceeds the tolerance (**Â§4.8**). Passes are emitted in a loop until no cut **or** no fill cell remains above tolerance (**Â§11.0.6**).

**[11.0.3]** *Anchor.* Passes shall be generated **shortest-haul first**. At each step, over all cut cells still above tolerance, the **anchor** is the cut cell whose nearest still-above-tolerance fill cell is closest (Euclidean distance between area centers). Ties shall be broken deterministically (e.g. lowest flattened index). The per-pass load cap is `load_cap = min(load_limit_BCY, remaining_fill_total_BCY)`, where `remaining_fill_total_BCY` is the sum over all fill cells above tolerance of `remaining_fill Â· fill_bcy_per_m`; if `load_cap â‰¤ 0` the loop terminates.

**[11.0.4]** *Region growing (normative).* From a seed cell the machine **region-grows a compact, contiguous patch** of material to a `target_bcy`, used for both the cut patch (seed = anchor, `target_bcy = load_cap`, `remaining_cut`, `cut_bcy_per_m`) and the fill patch (**Â§11.0.3**). The growth is a best-first flood keyed by squared Euclidean distance from the seed:

**[11.0.5]**

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

**[11.0.6]** Because empty cells are not expanded through, the consumed set stays a single connected blob around the seed, which keeps the swept loading/dumping length near its minimum. `used` (cells with depths) and the achieved `acc` (BCY) are returned. The cut patch and depths define the **cut segment**.

**[11.0.7]** *Strip bias (normative).* When a haul **heading** is supplied (or auto-detected) and `strip_bias > 1`, the growth priority key in **Â§11.0.2a** is replaced by an **anisotropic** key so the patch grows as one long, single-blade-wide **lane** along the heading instead of a compact blob (which would force the operator to weave). Let `(ux, uy)` be the heading unit vector and `(vx, vy)` its perpendicular; for a candidate cell at offset `(ddx, ddy)` from the seed:

**[11.0.8]**

```text
along = ddx*ux + ddy*uy          # distance along the heading
perp  = ddx*vx + ddy*vy          # distance across the heading
key   = abs(along) + strip_bias * perp^2
```

**[11.0.9]** The along term is **linear** (cheap) while the perpendicular term is quadratic and scaled by `strip_bias` (expensive), so a load fills one blade-wide line end-to-end before the frontier ever steps sideways; the lane only widens when it runs out of material along its length. `strip_bias` is roughly the lane length (in cells) the grower will chase before stepping over. Default **50** (set above a full load's cell count to keep each load a single continuous line); `strip_bias = 1` restores the isotropic compact-blob growth of **Â§11.0.2a**.

**[11.0.10]** *Flow-aligned growth (normative, default).* Unless **`--no-flow-aligned`** is set, each load's patch is grown as a **flow-aligned lane** (**Â§4.16**) instead of by **Â§11.0.2a**/**Â§11.0.2b**. From a smoothed unit flow-direction field `(fx, fy)` derived from the flow potential (**Â§9**, exposed by `strips.py:flow_direction_field`), the grower integrates a **single streamline** through the seed cell with a small fixed step (bilinear interpolation of `(fx, fy)`), consuming the cells it passes through (one 61 mm layer each) until the load reaches `target_bcy`, the streamline leaves the mask, or it runs out of material. The trace is **directional**: a **cut** patch is integrated **upstream** only (against the flow, toward the ridge it draws from) and a **fill** patch **downstream** only (with the flow, into the sink it feeds), so the consumed cells form one monotonic, non-self-crossing run in streamline order â€” long, curved, with-the-grain lanes rather than the staircase a fixed heading would force across diagonal flow. The cells are returned **in streamline order** (no nearest-neighbour re-chaining); each pass records its local flow bearings (`cut_heading_deg`, `fill_heading_deg`) for the guidance lane builder (**Â§13.3**). Multi-drop (**Â§11.0.3c**) and exact volume conservation (**Â§11.0.3**) apply unchanged. `--no-flow-aligned` restores the single global-heading grower of **Â§11.0.2b**.

**[11.0.11]** **[11.0.2c-i]** *Compactness guard (normative).* A streamline can run a long way and pick up a **scatter of cells separated by empty ground** (the cells in between are already on grade), producing a spatially fragmented load. Sweeping such a load forces a blade-up reposition across the gap that **covers nothing** â€” wasted travel and diesel â€” and a thin guidance line with no cut/fill beneath it. After each tube grow, the implementation shall therefore split the consumed cells into fragments that are connected only when within **one multi-drop hop** (`max_drop_hop`, **Â§11.0.3c**; floored at 1.5 cells so the load stays contiguous even when multi-drop is off), **keep the fragment holding the seed** (the haul anchor; the largest-volume fragment if the seed was not consumed), and **refund every dropped cell to the ledger** (`remaining_cut`/`remaining_fill`). The refunded cells are picked up by a later pass anchored near them (a short local haul, **Â§11.0.2**); volume is conserved exactly and the finished surface is unchanged â€” only this load's footprint becomes compact, so no rendered sweep needs a blade-up move longer than a multi-drop hop. The guard applies to both the cut and fill growers.

**[11.0.12]** *Fill patch and haul leg.* The carried load (`acc` from the cut grow) is deposited into a compact, contiguous **fill patch** region-grown by **Â§11.0.2a** from a fill seed (**Â§11.0.3b**), with `target_bcy = carried load` and the fill ledger/rate. If no fill cell can be grown, the cut depths shall be returned to `remaining_cut` and the loop terminates. Any **placed < carried** shortfall (`leftover`) shall be returned to the most-recently-consumed cut cells (iterate the cut `used` list in reverse, refunding `min(leftover, depthÂ·cut_bcy_per_m)` to each) so **volume is conserved exactly**.

**[11.0.13]** *Sweep ordering and haul leg (normative).* The two segments shall be ordered so the haul is the true transit between them, via the nearest-neighbour chain `_nn_chain` (start at a chosen cell, repeatedly append the nearest not-yet-used cell):

**[11.0.14]**

```text
fill_centroid = mean of fill-patch cell centers
cut_start  = cut cell FARTHEST from fill_centroid        # so the sweep ends toward the fill
cut_chain  = nn_chain(cut cells, start = cut_start)
cut_end    = center of last cell in cut_chain
fill_start = fill cell NEAREST to cut_end                # dumping begins where hauling ends
fill_chain = nn_chain(fill cells, start = fill_start)
haul_distance = obstacle_path_length(cut_end, fill_start)   # Â§12.3 when exclusions active; else euclidean
```

**[11.0.15]** Centroid-to-centroid haul (which double-counts the sweeps) shall **not** be used. The loading distance is the swept length of `cut_chain`, the dumping distance the swept length of `fill_chain` (**Â§13**).

**[11.0.16]** *Fill-seed selection and heading (optional, Â§6.2).* Without a heading (or `strength = 0`), the fill seed is simply the fill cell nearest the cut-patch centroid. When a haul **heading** is supplied (or auto-detected as the design surface's main down-slope â€” `estimate_slope_heading`: the bearing of `âˆ’âˆ‡target` averaged over valid areas, as a compass bearing 0 = N, 90 = E), the seed is chosen from the **200 nearest** fill cells to the cut centroid (or all fill cells if fewer), re-ranked by the anisotropic cost `dist Ã— (1 âˆ’ strength Ã— cos Î¸)`, where `Î¸` is the angle between the cutâ†’candidate direction and the heading unit vector and `strength âˆˆ [0, 0.95]`. The minimum-cost candidate is the seed. This emulates the lane-oriented hauling of tools such as OptiSurface. The realised haul distance can never fall below the field's optimal-transport minimum; heading mainly organises the work and trims sweep/haul overhead rather than beating the transport lower bound.

**[11.0.17]** **[11.0.3b-i]** *Cardinal snapping (normative).* Because the grid is axis-aligned (N/S/E/W), a heading even a degree or two off an axis makes strips (**Â§11.0.2b**) grow diagonally across cells and fragment into many small pockets (more multi-drop hops, **Â§11.0.3c**). `estimate_slope_heading` therefore **snaps** the auto-estimated bearing to the nearest of {0, 90, 180, 270}Â° when it is within `snap_tol_deg` (default **12Â°**) of that axis; otherwise the raw bearing is returned. An explicit `--heading-deg` is used as given (not snapped).

**[11.0.18]** *Multi-drop cuts and fills (normative).* A single pocket often cannot fill the scraper (cut side) or fully absorb the load (fill side), which would leave many near-empty "crumb" passes. When `max_drop_hop > 0`, after growing a patch (**Â§11.0.2b**) the matcher checks whether the load is still short of `load_cap` (cut) or whether material remains carried (fill); if so it finds the **nearest remaining cell of the same kind** (`_nearest_remaining`, KD-tree when SciPy is present, else brute force) from the current sweep end. If that cell is within `max_drop_hop` metres, the machine drives there **blade up** and grows another patch from it into the same load, repeating until the load is full/empty or the next pocket exceeds the hop. The blade-up drive between pockets is a `MOVE` segment in the guidance (**Â§13.4**), distinct from the optimized `HAUL` leg. Continuing one load across nearby pockets is cheaper than a whole separate cutâ€“haulâ€“fillâ€“grade cycle, so multi-drop materially raises load fullness and cuts pass count. `max_drop_hop = 0` disables it (one pocket per cut and per fill).

**[11.0.19]** **[11.0.3c-i]** *Minimum load fraction (optional, Â§6.2).* Topping a load to the last few percent of capacity often forces a multi-drop hop into a **laterally offset** pocket, which turns a single flowing lane into a side-by-side blob the blade can only cover with tight U-turns (the knots of **Â§13.3b**). To trade load-fullness for drivability, the cut multi-drop loop (**Â§11.0.3c**) shall stop adding hops once the load reaches `min_load_frac Ã— load_cap` (default **0.80**, i.e. `--min-load-fill 80`), taking the load as complete. Lower values keep more loads as single clean lanes at the cost of slightly under-filled scrapers; **`1.0`** (`--min-load-fill 100`) restores the original chase-to-full behaviour.

**[11.0.20]** *Layering.* Because 61 mm caps per-area depth, deep cuts/fills need multiple passes: a cell needing depth `d` contributes about `ceil(d / 61 mm)` passes over successive layers, with the transition receding as boundary cells reach grade. The final layer of an area may be a partial pass (**Â§7.3**).

**[11.0.21]** Each pass shall record: ordered cut cells with depths, ordered fill cells with depths, load in BCY, the haul distance, and the cut-segment start/end and fill-segment start/end world coordinates (used by sequencing, the time model, and the CSV/heat-map outputs).

**[11.0.22]** The implementation shall report the number of passes and any **unmatched** cut or fill BCY remaining below tolerance at termination.

---
## 12. Stage 5 â€” Sequencing and routing

**[12.0.1]** The passes from **Â§11** form a set; the implementation shall order them to minimize total **non-productive (deadhead) return travel** â€” the distance from where one pass ends (the fill end, or the grade-return end `fill_start` when the grade pass is on, **Â§12.0.5**) to the start of the next pass's cut segment. By default this is generalised to a **direction-aware time** that also charges the turn into the next cut (**Â§12.0.5**).

**[12.0.2]** Sequencing shall respect **layer ordering**: a deeper layer of an area shall not be worked before the shallower layer above it. Because passes are generated layer-by-layer (shallower first), this holds by construction.

**[12.0.3]** *Anti-dam ordering (informative).* Ordering should favor completing downhill flow first to avoid transient berms that could pond water; this is a soft preference, not a hard constraint in this version.

**[12.0.4]** The ordering shall be a constructive heuristic followed by bounded local improvement. The deadhead metric between an earlier pass `a` and a later pass `b` is `deadhead(a, b) = path_length(exit_xy(a), b.cut_start)` (for `exit_xy` see **Â§12.0.5**), where `path_length` is the obstacle-aware shortest path of **Â§12.3** when exclusion zones are active and otherwise the Euclidean distance. Construction is **nearest-neighbour starting from pass index 0** (deterministic): repeatedly append the unvisited pass minimizing the transition cost from `current`. Then a **bounded 2-opt** pass improves the order: it is applied **only when direction-aware sequencing is off (Â§12.0.5) and the pass count â‰¤ 1500** (above either, the construction order is kept, to stay fast and deterministic), runs at most **30 sweeps**, and accepts a segment reversal only when it strictly reduces the summed deadhead (improvement threshold `1e-6` m), taking the first improving move in each sweep. Exact optimization is not required. The route shall expose the ordered passes, the original indices, and the total deadhead distance.

**[12.0.5]** *Direction-aware, grade-aware transition cost (normative, default on).* The next cut should be cheap to **drive to and turn into**, not merely close. The transition cost from `a` to `b` is a **time** that combines the deadhead drive with the turn arcs, using the same turning-circle model as the guidance (**Â§13.1**):

**[12.0.6]**

```text
exit_xy(a)  = a.fill_start  if grade_return else a.fill_end   # where the machine ends
dist        = path_length(exit_xy(a), b.cut_start)           # Â§12.3 when exclusions active
drive_t     = dist / return_speed
angle       = turn(exit_dir(a) â†’ deadhead_dir) + turn(deadhead_dir â†’ cut_dir(b))
turn_t      = (turning_circle/2) * angle / turn_speed
cost(a, b)  = drive_t + turn_t
```

**[12.0.7]** where `exit_dir(a)` is the machine's heading as it finishes `a` (the grade-return direction `fill_end â†’ fill_start` when the grade pass is on, else the spread direction), `cut_dir(b)` is `b`'s cut heading, and `deadhead_dir` is the unit vector of the deadhead leg. **Grade-aware exit:** with the grade/compact pass on (**Â§13.2**) the machine ends at `fill_start` (the cut-side edge of the fill), so the grade return doubles as travel toward the next cut instead of being a detour. Because these costs are **asymmetric** (a reversed segment flips every interior heading), 2-opt segment reversal is **not** applied in this mode (**Â§12.0.4**); the turn-aware nearest-neighbour construction already orders passes against the time objective. Setting `--no-direction-aware` reverts the cost to plain deadhead distance and re-enables bounded 2-opt.

### 12.1 Zone work-order sequencing (optional output)

**[12.1.1]** On request (`--zones N`, default 8; 0 disables), the implementation shall summarise the fine-grained route into a small number of spatial **zones** for a human-readable rough work order. This is a reporting summary; it does not change the route or the plan.

**[12.1.2]** The cut-patch centroids of the routed passes shall be clustered into `k = min(N, n_passes)` zones with a **deterministic k-means++** (seeded from **Â§6.4**). Each zone shall report: a stable `zone_id`; a `work_order` rank (1 = worked earliest); load count; moved BCY; centroid in local XY **and** lat/lon (via `latlon_from_utm`); the first/last/mean pass index over which it is worked; and a rough compass position within the field.

**[12.1.3]** Zones shall be ordered by their **load-weighted mean pass index** (`mean_pass`), so `work_order` reflects when each zone is mostly worked. Because the route interleaves passes across zones to minimize travel, the work order is an *average* ordering, not a strict "finish zone 1 before starting zone 2" sequence; the reported first/last pass ranges make the overlap explicit.

**[12.1.4]** For overlay rendering (**Â§14.4**) each cut cell shall be assigned to the single zone that removed the most depth there, and the **exact cell-boundary outline** (jagged perimeter of that zone's cells) shall be computed as world-metre segments, so zone footprints partition the field without overlap.

**[12.1.5]** The zone work order shall be writable to a **sequence CSV** (**Â§14.4**) and drawn as numbered waypoints + outlines on the plan PNG (**Â§14.4**).

### 12.2 Windowed order refinement (optional, default off)

**[12.2.1]** *Motivation.* The direction-aware transition cost of **Â§12.0.5** is **asymmetric** (a reversed segment flips every interior heading), so the bounded 2-opt of **Â§12.0.4** is **not** applied in that mode and the route is left at its nearest-neighbour construction order with no local-improvement step. On request the implementation shall apply a **windowed relocation local search** (`routing.py`, `refine_route_order`) that supplies that missing improvement step while remaining safe with the directed cost.

**[12.2.2]** *Objective.* The search shall minimize the **order-dependent** part of the job time only â€” the sum over consecutive passes of the transition cost `cost(a, b)` of **Â§12.0.5** (seconds when direction-aware; deadhead distance in metres when `--no-direction-aware`). Each pass's own loading/hauling/dumping/grade sweep is fixed by its geometry and is invariant under reordering, so this sum is exactly what reordering can change. Reducing it reduces overall time.

**[12.2.3]** *Move operator (relocation / or-opt-1).* The only permitted move shall be to **lift a single pass and re-insert it** at another position; a contiguous run of passes is **never reversed**. Relocation therefore preserves every pass's internal cut/haul/fill headings, which is why it is sound under the asymmetric turn cost where segment-reversing 2-opt is not (**Â§12.0.4**, **Â§12.0.5**). The change in the **Â§12.2.2** objective from relocating one pass is computed in **O(1)** from the few transitions adjacent to the source and destination slots.

**[12.2.4]** *Window and sweep (normative).* On request (`--reorder-window N`, default **50**), with `N â‰¥ 2` and at least 3 passes, the implementation shall walk the passes **one at a time**. For the pass at position `k` it shall evaluate relocating that pass to each slot whose index lies within `N` positions of `k` (i.e. the window `[max(0, kâˆ’N), min(last, k+N)]`), and shall apply the single **strictly improving** relocation of greatest benefit (improvement threshold `1e-7`). Because every accepted move strictly lowers the **Â§12.2.2** objective and no move ever raises it, the search **monotonically converges**; it shall repeat full sweeps until a sweep makes no move or a bounded **round cap (12)** is reached.

**[12.2.5]** *Placement and effects.* The refinement shall run **after** routing (**Â§12.0**) and knot re-profiling (**Â§13.3b**) and **before** the time model (**Â§13.0**), zone sequencing (**Â§12.1**), pair exports (**Â§14**), and the blade-scale guidance (**Â§13.1**), so all downstream outputs reflect the refined order. It shall mutate the route in place â€” the ordered passes, the original indices, and the total deadhead distance â€” and shall report the number of relocations and the before/after objective. It shall **not** add, remove, split, or alter any pass, cut/fill bin, load, or haul leg, so the moved volume and the finished surface are unchanged. `--reorder-window 0` skips the refinement entirely, leaving the route exactly as **Â§12.0** produced it.

### 12.3 Exclusion zones and obstacle-aware routing

**[12.3.1]** *Purpose.* A field may contain areas the machine must **neither level nor drive through** â€” e.g. a well head in the middle of the paddock. Such areas are supplied as one or more **exclusion polygons** (**Â§4.18**, **Â§7.2.8**). Two consequences follow from one polygon: (1) no cut/fill is planned inside the zone; (2) every **blade-up travel leg** (loaded **HAUL**, deadhead **RETURN**, multi-drop **MOVE**) must **detour around** the zone rather than crossing it in a straight line.

**[12.3.2]** *Clearance buffer (normative).* Each exclusion polygon shall be offset outward by a **clearance** equal to half the configured **equipment width** (`--equipment-width-ft`, default **14 ft**), producing a larger **buffer ring** in projected metres. The buffer ensures the machine **body**, not merely the GPS centerline, stays out of the no-go area. Offsetting shall use a convex miter join on the CCW-oriented ring (`obstacles.py`).

**[12.3.3]** *Visibility graph + Dijkstra (normative).* The implementation shall build an **obstacle field** (`ObstacleField`) over all buffer rings:

**[12.3.4]**

```text
nodes  = all vertices of all buffer rings
edge(i,j) exists iff straight segment (node_i, node_j) does not cross any buffer interior
         (segmentâ€“polygon intersection test; endpoints on the perimeter are allowed)
```

**[12.3.5]** For a travel leg from point `A` to point `B`, if the straight segment is blocked the shortest path shall be found by **Dijkstra** on the visibility graph with temporary nodes at `A` and `B` connected to every visible graph node. The returned polyline includes `A` and `B`; its summed edge lengths is **`path_length(A, B)`**. When no exclusion zones are active, or the straight segment is clear, `path_length` shall equal the Euclidean distance (O(1) axis-aligned bounding-box reject before any graph work).

**[12.3.6]** *Where obstacle distances apply (normative).* The same `ObstacleField` instance shall be threaded through:

**[12.3.7]** 1. **Pass haul length** â€” `haul_distance` in **Â§11.0.14** (`passes.py`, `_build_pass`).
**[12.3.8]** 2. **Sequencer transition cost** â€” `dist` in **Â§12.0.5â€“Â§12.0.6** and total deadhead (`routing.py`).
**[12.3.9]** 3. **Windowed refinement** â€” **Â§12.2** objective when `--reorder-window` is on.
**[12.3.10]** 4. **Guidance travel legs** â€” before emitting waypoints, each consecutive blade-up leg (`RETURN`, `HAUL`, `MOVE`) whose straight line crosses a zone shall have the detour vertices of `path(A, B)` inserted into the track (**Â§13.4.2**). Blade-down coverage (`CUT`, `FILL`, `GRADE`) is **never** rerouted â€” the blade legitimately works up to the bins abutting the cleared perimeter.

**[12.3.11]** *Volume balance (normative).* Removing exclusion bins breaks the field's cut/fill closure. When any exclusion zone is active, the balance check (**Â§8.0.2**) shall suppress the relative-imbalance warning and report any net surplus cut as **exported from the field** and any net fill deficit as **imported to the field**.

**[12.3.12]** *CLI input format (normative).* Each `--exclude-polygon` value is a string of **`lat,lon` vertices separated by `;`**, with at least three vertices per polygon. The flag may be repeated for multiple polygons. Vertices are projected with `project_latlon_ring` (**Â§6.1**, `agd.py`). Malformed input shall produce a clear error and a non-zero exit code.

**[12.3.13]** *Rendering (informative).* When exclusion zones are present, the plan and guidance PNG renderers **may** draw the no-go polygon as a hatched overlay (**Â§14**). This is visual only and does not affect the plan.

### 12.4 Center-pivot irrigation scheduling (optional)

**[12.4.1]** *Purpose.* A field may carry a **center-pivot irrigation system**: a linear arm anchored at a point in the field and rotating about that anchor. While the field is being leveled the arm must stay clear of the scraper's active cutâ€“haulâ€“fill work so the machine does not have to drive detours around it; rotating the arm costs effort, so the schedule shall **minimise arm moves** while accepting a bounded number of **haul detours** when holding the arm still is cheaper than repositioning. This stage is **optional** and **off by default** (`--pivot` / `--no-pivot`, **Â§15**).

**[12.4.2]** *Anchor and radius (normative).* When `--pivot` is set, the implementation shall compute a **pivot geometry** (`PivotGeometry`, `pivot.py`):

**[12.4.3]** 1. **Anchor** â€” the centroid of all valid field cells (projected easting/northing), unless **`--pivot-center "lat,lon"`** supplies a WGS-84 point, in which case that point is projected to UTM with `utm_from_latlon` (**Â§6.1**, **Â§7.1**). If the supplied point falls in a different UTM zone than the field, the implementation **shall warn** but continue.
**[12.4.4]** 2. **Radius** â€” the largest circle, centred at the anchor, that contains no invalid cell and stays inside the grid envelope, minus half a cell diagonal and half the **wheelbase** so the arm's physical width also fits inside the field boundary.
**[12.4.5]** 3. **Wheelbase** â€” `--pivot-wheelbase-ft` (default **14 ft**), converted to metres; the arm is modeled as a bar this wide running anchor â†’ tip.
**[12.4.6]** 4. **Clearance** â€” `--pivot-clearance-ft` (default **25 ft**), converted to metres; extra room kept between the parked arm and active leveling work **per side**, on top of half the wheelbase.

**[12.4.7]** *Angular occupancy (normative).* For each executed pass (in route order) and each candidate arm bearing (discretised into `n_bins`, default **720**), the implementation shall mark bearings that would collide with that pass's work:

**[12.4.8]** - every **cut cell** and **fill cell** centre;
**[12.4.9]** - every sample point along the loaded **haul leg** (cut-end â†’ fill-start).

A work point at radius `r` and bearing `Ï†` from the anchor blocks bearings within `Â± (half_wheelbase + clearance + cell_radius) / r` of `Ï†` (clamped to Ï€). Points beyond the arm's reach do not block.

**[12.4.10]** *Dynamic-programming schedule (normative).* Let `blocked[i, b]` be whether bearing bin `b` collides with pass `i`, and `move_cost = --pivot-move-cost` (default **25**, interpreted as the number of single-pass haul detours one arm reposition is worth avoiding). The implementation shall find the bearing sequence `chosen[i]` minimising

```text
Î£_i  blocked[i, chosen[i]]
     + move_cost Ã— (number of i where chosen[i] â‰  chosen[iâˆ’1])
```

via the forward DP

```text
dp[i][b] = blocked[i][b] + min( dp[iâˆ’1][b],                    # hold bearing
                                  move_cost + min_b' dp[iâˆ’1][b'] )  # rotate
```

Backtracking shall **prefer holding** the previous bearing; when a move is forced, the new bearing shall be the optimal predecessor **closest in circular angle** (minimum rotation). Equal-bearing contiguous runs are collapsed into **pivot positions**, each reporting: 1-based position index, first/last pass (inclusive, route order), compass bearing, rotation from the previous position, arm-tip lat/lon, and the count of collided passes in that run.

**[12.4.11]** *Placement in the pipeline (normative).* Pivot scheduling shall run **after** routing (**Â§12.0**) and knot re-profiling (**Â§13.3b**) and **before** the time model (**Â§13.0**), zone sequencing (**Â§12.1**), pair exports (**Â§14**), and blade-scale guidance (**Â§13.1**), so downstream outputs reflect the same executed pass order. It shall **not** add, remove, split, or alter any pass, cut/fill bin, load, or haul leg â€” only the arm schedule and the detour routing described below.

**[12.4.12]** *Arm obstacles and guidance detours (normative).* For each pivot position the parked arm at its held bearing shall be represented as a **wheelbase-wide rectangle** from anchor to tip (`arm_ring`, `pivot.py`), buffered outward by half **`--equipment-width-ft`** (default 14 ft) into an `ExclusionZone`, and combined with any static exclusion zones (**Â§12.3**) into a per-pass **`ObstacleField`**. Every pass held at that position shares the same field.

When `--pivot` is set and guidance is built (**Â§13.1**), the obstacle detour pass (**Â§13.4.2**) shall use the **per-pass arm field** (falling back to the static exclusion field when no arm field applies) so each blade-up travel leg (`RETURN`, `HAUL`, `MOVE`) whose straight segment crosses the parked arm receives the visibility-graph detour vertices of `path(prev, cur)`. Blade-down coverage (`CUT`, `FILL`, `GRADE`) is **never** rerouted.

**[12.4.13]** *Detour statistics and time (normative).* For each pass whose loaded haul leg crosses the parked arm, the implementation shall measure `extra = path_length(cut_end, fill_start) âˆ’ euclidean(cut_end, fill_start)`. Legs with `extra` above a small threshold (default 1 mm) are **routed haul detours**; legs blocked but with negligible extra distance are **work under the arm** (the work itself sits beneath the arm and cannot be rerouted). The sum of routed extras shall be added to the OptiHaul report as **`+ pivot detours`** time at the achieved haul and return speeds (**Â§13.0.3**), divided by field efficiency, and a **`TOTAL w/ pivot`** line shall be printed when `--pivot` is set.

**[12.4.14]** *Outputs (normative).* When `--pivot` is set the implementation shall:

**[12.4.15]** 1. Print a **center-pivot schedule** block: anchor lat/lon (and whether user-specified or field middle), max radius, position/move counts, total rotation, routed detour count and extra distance/time, and work-under-arm count.
**[12.4.16]** 2. Write a **pivot CSV** (`--pivot-csv`, default `<out>_pivot.csv`): header comments (anchor, radius, wheelbase, moves, total rotation, detoured-pass count, move cost) and one row per position (pass range, bearing, cardinal label, move degrees, tip lat/lon, collided-pass count).
**[12.4.17]** 3. Write **per-position PNG maps** to `--pivot-dir` (default `<out>_pivot/`): one map per position showing the cut/fill (or topology) state at the moment the arm moves, the arm at its new bearing (grey ghost of the previous bearing), the swept irrigation circle, and that run's haul operations; plus a **`pivot_overview.png`** with all bearings colour-graded firstâ†’last. Stale maps from a prior run shall be removed before writing.
**[12.4.18]** 4. Write a **pivot leveling animation GIF** (`--pivot-gif`, default `<out>_pivot_leveling.gif`, on by default with `--pivot`; `--no-pivot-gif` disables). The animation shall apply passes in route order, draw the current haul leg, show the arm at its scheduled bearing, and emit a **held frame at every arm move** (arm flashes red, grey ghost of previous bearing). Frame cadence otherwise follows `--pivot-anim-stride` (default 15). `--pivot-frames DIR` may additionally write individual PNG frames. Render mode `--pivot-mode {cutfill,topology}` (default `cutfill`) and resolution `--pivot-ppb` (default 6) apply to maps and animation.

---

## 13. Stage 6 â€” Time model (OptiHaul-matched) & Stage 7 â€” blade-scale guidance

**[13.0.1]** This section has two parts. **Â§13.0** is the **core OptiHaul-matched** time model (four motions, no turning, no grade pass) used for direct comparison with OptiSurface OptiHaul. **Â§13.1**â€“**Â§13.6** are the optional **blade-scale operator guidance** (Stage 7): a drivable realization of the same passes that additionally counts **turning** and a **grade/compact return pass**, and emits turn-by-turn waypoints. Both consume the routed passes of **Â§12**; the guidance never changes the passes or their optimized haul, it only realizes them at blade scale.

**[13.0.2]** Each pass comprises four motions â€” **loading** (cutting), **hauling**, **dumping** (filling), and **returning** â€” using **Â§6.3** parameters. The model matches the OptiSurface OptiHaul methodology.

**[13.0.3]** *Distances.* Per pass: the loading distance is the swept length of the cut segment, the dumping distance is the swept length of the fill segment, and the hauling distance is the haul leg (**Â§11.0.3**). The **returning distance equals the loading + hauling + dumping distance** â€” every forward trip has an equal-length return.

**[13.0.4]** *Speeds.* Loading and dumping run at their **constant** configured speeds. Hauling and returning use a **trapezoidal speed profile**: the machine accelerates for `acceleration_time` and decelerates for `deceleration_time`, reducing the *average* speed below the configured maximum. For a move of length `L` at maximum speed `v`:

**[13.0.5]**

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

**[13.0.6]** The return time is computed by applying this profile to each forward segment length (cut, haul, fill) at the **return** max speed.

**[13.0.7]** Speeds shall be converted MPH â†’ m/s. Loading/dumping travel times (`distance / speed`) inherently include cutting/placing time; there is no separate per-BCY term.

**[13.0.8]** *Field efficiency.* Productive time = loading + hauling + dumping + returning. **Total time = productive Ã· field_efficiency**; the remainder is reported as **"Other"** time. The implementation shall report total time, the four distances, the achieved average haul and return speeds, and the moved volume (BCY). When **`--pivot`** is set and routed haul detours add measurable loaded-haul distance (**Â§12.4.13**), the report shall additionally print **`+ pivot detours`** and **`TOTAL w/ pivot`** lines.

### 13.1 Stage 7 â€” blade-scale guidance overview

**[13.1.1]** On request (**Â§13.6**), the implementation shall build a **guidance track** (`swaths.py`, `build_guidance`) that tiles each routed pass's cut and fill patches into **blade-width lanes** and emits an ordered list of **waypoints** (**Â§4.15**, **Â§13.3**). This realizes the optimized passes at the scale the operator actually drives, without re-planning soil movement: the per-pass haul leg is preserved as the optimizer's transit (the cut sweep is oriented to *end* at the haul target and the fill sweep to *start* there).

**[13.1.2]** *Turning model.* A heading change of `Î¸` at a point in the track costs arc travel `RÂ·Î¸` at the **turn speed**, where `R = turning_circle / 2`. Turn metrics are computed on each sub-path **after Ramerâ€“Douglasâ€“Peucker simplification** (so cell-to-cell stair-stepping is not miscounted as turns): the path is simplified, the interior vertex angles are measured, and only those **â‰¥ `turn_threshold_deg`** (default 60Â°) are counted as **tight turns** and summed into the total turn angle. Gentler bends are driven in stride at travel speed (no slowdown). The track reports the tight-turn **count** and the summed turn **angle**.

### 13.2 Grade/compact return pass

**[13.2.1]** Unless `--no-grade-return` is set, each fill shall be followed by **one return pass back over the fill** (blade down) to grade and compact it, retracing the fill lane in reverse. Its distance equals the fill sweep length and it is driven at the **grade speed** (default 8 MPH). With the grade pass on, the machine **ends each pass at the fill's cut-side edge** (`fill_start`), which the sequencer exploits so the grade return doubles as travel toward the next cut (**Â§12.0.5**). The core model (**Â§13.0**) does **not** include this pass; the guidance time (**Â§13.5**) does.

### 13.3 Guidance waypoints and lane building

**[13.3.1]** *Single-heading lanes.* For single-heading passes (**Â§11.0.2b**), each patch shall be split into 8-connected **pockets** (**Â§4.12**) and the pockets ordered to optimize the haul leg: the **cut** sweep is ordered to finish nearest the fill start, the **fill** sweep to start nearest the haul arrival. Within a pocket, the cells are tiled into serpentine **blade-width lanes** spaced by `blade_width Ã— (1 âˆ’ lane_overlap)`; a second lane is added only when the patch is genuinely wider than one blade (guarding against spurious second lanes for slightly off-axis strips).

**[13.3.2]** *Flow-aligned lanes.* For flow-aligned passes (**Â§11.0.2c**, signalled by a per-pass `cut_heading_deg`/`fill_heading_deg`), the lane builder shall **preserve the streamline order** in which the grower returned the cells rather than re-tiling them by heading. The stream-ordered cells are split into maximal runs of consecutive 8-adjacent cells (a multi-drop hop is a break to a non-adjacent cell); each run is driven in order, and **whole runs** are flipped only to orient the cut sweep to finish near the fill (and the fill sweep to start near the haul arrival). This avoids nearest-neighbour re-chaining, which would zig-zag a curved lane.

**[13.3.3]** *Nearest-neighbour fragment order (normative).* When a sweep spans several fragments (multi-drop pockets, **Â§11.0.3c**, or the runs of **Â§13.3.1a**), the builder shall order them by a **greedy nearest-neighbour chain**, not by raw distance to the haul junction. Sorting purely by distance-to-junction can place two far-apart fragments next to each other and force a long blade-up `MOVE` (**Â§13.4**) that covers nothing; nearest-neighbour chaining makes each inter-fragment hop the shortest available while still **anchoring** the sweep to the haul junction â€” the **cut** sweep finishes at the fragment nearest the fill, the **fill** sweep starts at the fragment nearest the haul arrival. This applies to the flow lane builder, the single-heading pocket builder, and the boustrophedon re-profile (**Â§13.3b**).

**[13.3.4]** Each waypoint shall carry: an **action** âˆˆ {`START`, `CUT`, `HAUL`, `MOVE`, `FILL`, `GRADE`, `RETURN`}, a **blade state** (down for `CUT`/`FILL`/`GRADE`; up for `START`/`RETURN`/`HAUL`/`MOVE`), world coordinates (and lat/lon), the **target depth** (mm) at that point, the **bearing** to the next waypoint, and a human-readable **instruction**.

### 13.3a Route smoothing and minimum turning circle (drivable steering line) **[13.3.5]**

**[13.3.6]** **[13.3a.1]** Raw waypoints sit on grid-cell centers, so a lane that drifts off-axis steps in 45/90Â° jogs no tractor+scraper can physically track. Unless **`--no-route-smoothing`** is set, the implementation shall first Gaussian-smooth each cut/fill sweep's **emitted coordinates** to remove stair-steps: a Gaussian over the waypoint sequence whose width is tied to the machine **turning radius** (`turning_circle / 2`), applied **per contiguous run** (split at multi-drop hops) with the run's **endpoints pinned**. Pinning the endpoints leaves the optimized haul/return junctions exactly where the sequencer placed them. Gaussian smoothing alone **does not** guarantee a minimum path radius.

**[13.3.7]** **[13.3a.2]** *Smoothing is steering-display only for coverage length (normative).* Gaussian smoothing **shall not** shorten CUT/FILL/GRADE coverage length below the raw cell-sweep floor. Coverage distance, and the coverage component of time, shall be computed on the **raw coverage geometry** (the unsmoothed cell sweep). A one-blade-wide lane that staircases along a diagonal still requires the blade to cover the whole cell band, so the loading/dumping sweep length is bounded below by `volume Ã· (max_depth Ã— blade_width)`.

**[13.3.7a]** **[13.3a.3]** *Hard turning-circle bound (normative).* After Gaussian smoothing (and also when **`--no-route-smoothing`** is set), the implementation **shall** enforce a hard minimum steering radius of `R = turning_circle / 2` on the **full emitted steering polyline** (blade-down runs, blade-up headland splices, loaded **`HAUL`** transit, and multi-drop **`MOVE`** hops). Detection uses a small heading-change noise floor (~15Â°), not the 60Â° turn-time threshold of **Â§13.1.2**, so soft pinches with `r â‰ª R` are still considered. At any interior corner whose local circumradius is strictly less than `R`, the builder **shall** insert a constant-radius-`R` blade-up **`MOVE`** headland that reconnects with the required outbound heading when a classic corner fillet fits on the inbound/outbound legs. For near-reverse (~180 deg) lane U-turns the headland **shall** be a compact Dubins CCC keyhole/bulb (LRL or RLR); a long multi-loop CSC (~540 deg) **shall not** be used when a CCC keyhole exists. Among feasible CCC/CSC candidates the builder **shall** prefer the chirality with fewer samples outside the valid field mask, then shortest length, then least forward excursion. For near-reverse U-turns the builder **shall** also consider a constant-R **side-arc** keyhole (optional short forward spur, arc into available field, reconnect to the reverse tip) and adopt it when it has fewer outside samples or equal outside with shorter length than tip CCC. When every tip CCC still leaves the field, the builder **may** slide the inbound pose forward along the inbound heading (and may also slide the outbound pose) by up to ~2.5R and re-solve CCC. Soft bends that cannot host a classic fillet **shall** use same-side CSC (LSL/RSR) capped near one keyhole length â€” **not** a CCC bulb per soft pinch. Reverse windows within ~4R **shall** coalesce to one keyhole **only when the local corner heading change is â‰¥ ~60Â°**; softer corners **shall not** open that coalesce (they fillet or are left for upstream knot repair). A candidate reverse exit **shall** show sustained reverse (2+ reverse segments, or one segment â‰¥ ~0.5R), not a one-cell zig-zag. Short-leg soft pinches that cannot host an R-bounded splice without a multi-loop path **shall not** be expanded into stacked Dubins bulbs (those need upstream knot/boustrophedon repair). A dedicated pass **shall** replace sharp cut-end â†’ fill-start **`HAUL`** chords and sharp multi-drop **`MOVE`** chords (same blade-down action on both sides) with R-bounded transit: long legs use CSC exit + middle + CSC entry; only a short near-reverse hop may use one CCC keyhole; inserted samples keep the original **`HAUL`** / **`MOVE`** action so distance accounts for the curved path. A post-pass **shall** repair residual sharp CUTâ†”MOVE junctions without cascading CCC bulbs and **without dropping** blade-down coverage apexes. Headland arc length **shall** accumulate as `reposition_dist_m` (**Â§13.4**) and **shall not** reduce the raw CUT/FILL/GRADE coverage length used for the volume floor (**Â§13.3a.2**). Obstacle detours (**Â§13.4.2**) apply to these headland `MOVE` legs and to curved `HAUL`/`MOVE` transit as for any other blade-up travel.

**[13.3.7b]** **[13.3a.4]** *Equipment-length lineup (normative).* Let `L` be **`--equipment-length-ft`** converted to metres (default **43 ft**). After the hard turning-circle pass (**Â§13.3a.3**), the guidance builder **shall** ensure that immediately before each contiguous blade-down **`CUT`** or **`FILL`** run the steering polyline includes a blade-up approach of length approximately `L` along the run's outbound heading, and immediately after each such run a matching blade-up exit along the inbound heading â€” so the tractor+scraper train is aligned when the blade engages and again when it lifts. **Staying inside the valid field mask is mandatory** and overrides a full straight `L`: when a straight spur of length `L` would leave the field, the builder **shall** use the longest in-field straight available and, if a reconnect is still required, the **minimum in-field R-curve** (CSC capped near `Ï€R + chord`) â€” it **shall not** leave the field and **shall not** invent a ~360Â° loop merely to force full `L`. An existing colinear approach/exit already â‰¥ ~0.95 of the achievable in-field straight **shall** be left alone. When the next **`FILL`**/**`GRADE`** tip lies ahead on the exit ray in the same corridor, the exit **shall** be clamped to stop short of that tip (the gap is the approach). Arrival tips **`START`** / **`RETURN`** near a run start **may** be relocated to the (in-field) lineup entry; a **`HAUL`**/**`MOVE`** tip that already arrived on the tip **shall not** be jumped back to force full `L`. Coalesced U-turn keyholes (**Â§13.3a.3**) **shall** leave the inbound tip on a straight exit spur of length `L` when that spur is in-field, finish the turn at the reverse lineup entry (`End âˆ’ LÂ·outbound`), then drive straight onto coverage. Setting `L = 0` disables lineup.

### 13.3b Knot re-profiling (boustrophedon) **[13.3.8]**

**[13.3.9]** **[13.3b.1]** *Detection (normative).* Some compact multi-drop loads (**Â§11.0.3c**) chain into an undriveable **knot** â€” a sweep that doubles back on itself with several hairpin reversals in a tiny area that no 45 ft-circle machine can follow. After sequencing (**Â§12**) and before the guidance time/outputs are produced, unless **`--no-knot-repair`** is set, the implementation shall scan every pass and, for the cut and the fill sweep independently, build the path the guidance would otherwise draw (the flow/lane tiling of **Â§13.3**/**Â§13.3.1a** **plus** the serpentine find-and-fix) and count its hairpins on the Ramerâ€“Douglasâ€“Peucker-simplified path (**Â§13.1.2**). A sweep is a knot when that count is **â‰¥ the hairpin threshold** (default **3**).

**[13.3.10]** **[13.3b.2]** *Boustrophedon re-profile (normative).* For a knotted sweep the implementation shall build a **boustrophedon** alternative: the patch is decomposed into parallel **blade-width lanes** along the sweep heading that **minimises lane count** (search headings every 10°, tie-break longest along-span; PCA alone is not required) and the lanes are walked in serpentine order â€” every other lane reversed â€” so the machine drives one straight lane, makes a single headland turn, and drives the next lane back (multiple straight **back-and-forth sweeps** rather than a zig-zag). The boustrophedon shall be adopted **only when it has strictly fewer hairpins than the rendered sweep** (**Â§13.3b.1**); otherwise the sweep is left unchanged. This guarantees a sweep that already drives cleanly (e.g. a gentle flowing fill) is **never made worse** by the repair.

**[13.3.11]** **[13.3b.3]** *Pocket-aware (normative).* A load may span several disjoint pockets (a multi-drop cut/fill). The boustrophedon shall be built **pocket by pocket**: the patch is split into 8-connected pockets (**Â§4.12**), each pocket is swept as its own local boustrophedon, and the pockets are ordered to finish nearest the fill (cut side) or start nearest the haul arrival (fill side) â€” exactly as the single-heading lane builder orders pockets (**Â§13.3.1**). Binning the whole spread as one lane field is forbidden, because it would stitch distant pockets together with long blade-up jumps; pocket-aware ordering keeps the inter-pocket repositions (`MOVE`, **Â§13.4**) to the genuine, unavoidable hops the matcher already created.

**[13.3.12]** **[13.3b.4]** *Conservative (normative).* Re-profiling only **re-orders the cells** of a flagged patch. It **shall not** drop, add, or relocate any cut/fill bin, change the load volume, or alter the optimized haul leg â€” so the soil routing and the simulated finished surface are **identical** to the un-repaired plan; only the in-patch driving path changes. A re-profiled patch shall be flagged so the lane builder follows its **stored lane order directly** instead of re-deriving (and re-tangling) the sweep by **Â§13.3.1a**. The only cost is the extra headland turning and short repositions of the parallel sweeps (a few percent of total time, **Â§13.5**).

### 13.4 Multi-drop repositioning (`MOVE`)

**[13.4.1]** A multi-drop pass (**Â§11.0.3c**) spans several pockets; the blade-up drive between two pockets within a sweep shall be emitted as a **`MOVE`** segment (blade up), distinct from the optimized **`HAUL`** leg. `MOVE` distance is accumulated separately (`reposition_dist_m`) and shall **not** inflate the reported haul distance, though it is timed as travel (**Â§13.5**). Each `MOVE` covers **no ground** (blade up), so it is pure overhead: the compactness guard (**Â§11.0.2c-i**) keeps loads from spanning empty gaps and the nearest-neighbour fragment order (**Â§13.3.1b**) keeps each hop the shortest available, so a `MOVE` is bounded to a genuine pocket-to-pocket reposition rather than a long excursion that covers nothing.

**[13.4.2]** *Obstacle detour waypoints (normative).* When exclusion zones are active (**Â§12.3**) and/or center-pivot scheduling is active (**Â§12.4**), after the raw cut/fill/grade waypoints are assembled the guidance builder shall walk consecutive waypoints and, for each leg whose action is blade-up travel (`RETURN`, `HAUL`, `MOVE`, but not `START`), test whether the straight segment crosses any buffered zone interior (static exclusion and/or the parked pivot arm for that pass). If blocked, it shall insert the intermediate vertices of `path(prev, cur)` (excluding the endpoints) as additional waypoints with the same action and zero depth before emitting `cur`. Distance and time accounting (**Â§13.5**) shall follow the detour polyline on the raw geometry. Blade-down sweeps are not detoured.

### 13.5 Guidance time estimate

**[13.5.1]** `estimate_guidance_time` shall sum, over the track: `CUT` length Ã· loading speed; `FILL` Ã· dumping speed; `GRADE` Ã· grade speed; `HAUL` and `MOVE` via the trapezoidal profile (**Â§13.0.3**) at haul max speed; `RETURN`/`START` via the trapezoidal profile at return max speed; and the **turning** time `R Â· total_turn_angle / turn_speed` (**Â§13.1.2**). Productive = the sum of those; **total = productive Ã· field_efficiency**, the remainder reported as "Other". The breakdown (`GuidanceTime`) shall expose loading, dumping, grading, hauling (incl. `MOVE`), returning, turning, other, and total seconds, plus the tight-turn count.

**[13.5.2]** *Relationship to the core model.* The guidance total is normally **higher** than the core OptiHaul-matched total (**Â§13.0**) because it adds turning, the grade/compact pass, and the extra headland turns of any knot re-profiling (**Â§13.3b**, a few percent), partly offset by direction-aware sequencing reducing the empty return. This is expected and is the realistic, drivable field time. The guidance shall **not** increase the optimized per-pass haul distance, and knot re-profiling shall **not** change the moved volume or finished surface (**Â§13.3b.4**).

### 13.6 Guidance outputs

**[13.6.1]** *Guidance CSV* (`--guidance-csv`, default `<out>_guidance.csv`): one row per waypoint in track order with the sequence index, pass number, action, blade state, lat/lon (and easting/northing), target depth, next-bearing, segment distance, and instruction. The lat/lon (and easting/northing) are the **smoothed steering points** (**Â§13.3a**); the segment distance is the **raw coverage** travel from the previous waypoint (it is not the straight-line gap between smoothed points â€” distance/time are charged on the coverage geometry, **Â§13.3a.2**).

**[13.6.2]** *Guidance PNG* (`--guidance-png`, default `<out>_guidance.png`): the cut/fill map overlaid with the blade lanes coloured by action â€” cut, fill, the `GRADE` return (dashed, over the fill), and `MOVE`/`HAUL`/`RETURN` blade-up travel â€” so the drivable plan is visually inspectable. On printing the guidance the report shall also state the preserved haul, blade-scale loading/dumping/grade/reposition distances, the coverage factor (blade width Ã· cell width), the tight-turn count, the sequencing mode, and the drivable time breakdown and total.

---
## 14. Outputs

**[14.0.1]** The program shall always write one **plan PNG** (cut/fill map + haul arrows, **Â§14.1**â€“**Â§14.3**). On request it shall also write the zone overlay (**Â§14.4**), CSV exports (**Â§14.4**), a pass-pairs heat-map PNG (**Â§14.5**), and a leveling animation (**Â§14.6**). Rendering conventions match the existing AgGrade tools so outputs are visually comparable.

### 14.1 Cut/fill map

**[14.1.1]** Each valid area shall be filled with the band color for `diff = existing âˆ’ target` (m), using the nine-band palette and thresholds from **`fielddesign.py`**:

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

**[14.1.3]** Negative `diff` (fill) maps to cool colors, positive `diff` (cut) to warm colors. Areas outside the mask are transparent (alpha 0). A fast vectorised renderer (`diff_to_image`) shall be available for the many animation frames (**Â§14.6**).

### 14.2 Haul arrows

**[14.2.1]** Haul-direction arrows shall be hollow (outline-only) triangles pointing in the flow direction, using the glyph from **`render_existing_heights.py`**. Direction is in compass degrees (0 = N, 90 = E), converted to image vector `(dx, dy) = (sin Î¸, âˆ’cos Î¸)`.

**[14.2.2]** Arrows shall be sampled on a regular sub-grid (`--arrow-stride`), drawn on top of the cut/fill fill, in black; areas with no arrow (at grade) are skipped.

### 14.3 Image layout and geometry

**[14.3.1]** The plan image shall be `grid_width Ã— pixels_per_bin` wide and `grid_height Ã— pixels_per_bin` tall (`pixels_per_bin` default 50). Origin is top-left; image Y is flipped relative to grid Y so larger `by` is drawn higher: area `(bx, by)` occupies the block with top-left `x0 = bxÂ·ppb`, `y0 = (grid_height âˆ’ 1 âˆ’ by)Â·ppb`; arrow centers at `px = (bx + 0.5)Â·ppb`, `py = (grid_height âˆ’ 0.5 âˆ’ by)Â·ppb`. A `_world_to_pixel` helper shall map world (easting, northing) to pixels with the same flip, for overlays (**Â§14.4**) and the heat map (**Â§14.5**).

### 14.4 Zone waypoints, outlines, and CSV exports (optional)

**[14.4.1]** When zones are computed (**Â§12.1**), the implementation shall overlay on the plan PNG, beneath/above the cut/fill fill respectively: each zone's **exact cell-boundary outline** (white halo under a black line) and a numbered **waypoint** marker at the zone centroid, connected in `work_order` order. Marker radius and font shall scale with the image's short side so numbers stay legible at any resolution.

**[14.4.2]** *Sequence CSV* (`--sequence-csv`, default `<out>_cut_sequence.csv` when zones are on): one row per zone with `work_order, zone_id, compass, loads, moved_bcy, easting_m, northing_m, lat, lon, first_pass, last_pass, mean_pass`.

**[14.4.3]** *Pairs CSV* (`--pairs-csv`, default `<out>_cut_fill_pairs.csv`): every pass, in optimized route order, with `order, load_bcy, haul_m` and the start/end **lat/lon** of both the cut segment and the fill segment (`cut_start_*`, `cut_end_*`, `fill_start_*`, `fill_end_*`), via `latlon_from_utm`.

### 14.5 Pass-pairs heat map (optional)

**[14.5.1]** On request (`--pairs-png`, default `<out>_cut_fill_pairs.png`) the implementation shall render the cut/fill map overlaid with **one line per cut-fill pair** along its haul leg (cut end â†’ fill start), producing a "heat map" of haul corridors.

**[14.5.2]** The view shall render at a modest resolution (lines several pixels wide via a disk stamp) so individual lines are visible; the underlying cut/fill map shall be faded so the lines stand out. Line density shall accumulate (each pixel counted once per line) and map through a **hot ramp** (yellow for a single load â†’ orange â†’ deep red where many overlap), using a robust percentile scale and a minimum alpha floor so even isolated lines are clearly visible.

### 14.6 Leveling animation (optional)

**[14.6.1]** On request the implementation shall reconstruct the field surface as each pass is applied in route order and render a frame every `stride` applied passes (`--animate-stride`, default 10), at `--animate-ppb` pixels per area (default 8). Frame 0 is the initial surface; the final pass always produces a frame. Frames may be written as PNGs to a directory (`--animate-dir`) and/or assembled into an animated **GIF** (`--animate-gif`, default `<out>_leveling.gif`).

**[14.6.2]** Two render modes (`--animate-mode`): **`cutfill`** (default) renders the cut/fill map (which turns green as areas reach grade) via `diff_to_image`; **`topology`** renders the absolute elevation on a terrain ramp via `elevation_to_image`, with the color scale fixed to the initial surface range so the terrain visibly flattens toward grade.

**[14.6.3]** Any assembled GIF shall **hold its final frame for 4 seconds** so the end result is visible before the loop restarts; intermediate frames use the normal cadence.

### 14.7 Center-pivot maps and animation (optional)

**[14.7.1]** On request (`--pivot`, **Â§12.4**), the implementation shall render pivot outputs using the same cut/fill palette (**Â§14.1**) or a fixed-scale topology ramp (**Â§14.6.2**), at `--pivot-ppb` pixels per area (default 6).

**[14.7.2]** *Per-position maps.* For each pivot position, one PNG shall show the field state **immediately before** that position's first pass is applied (the surface after all prior passes), with: the swept irrigation circle (blue outline), the anchor (black/white dot), the held arm (black bar, yellow tip marker), an optional grey **ghost** of the previous bearing, faint orange haul lines for passes in that run, and a text banner (position index, pass range, bearing, move degrees, radius, anchor lat/lon, collided-pass count).

**[14.7.3]** *Overview map.* `pivot_overview.png` shall show all arm bearings over a faded initial cut/fill map, colour-graded first (blue) â†’ last (red), numbered at each tip.

**[14.7.4]** *Leveling animation.* `animate_pivot_leveling` shall produce a GIF (default `<out>_pivot_leveling.gif`) and/or PNG frames (`--pivot-frames`): frame 0 is the untouched field with the first bearing; every `--pivot-anim-stride` applied passes emit a progress frame (current haul leg, arm at scheduled bearing, percent-to-grade banner); **every arm move** emits a dedicated longer-held frame (red arm, grey ghost, "PIVOT MOVE k/n" banner); the final frame holds 5 s.

---
## 15. Command-line interface

**[15.0.1]** The program shall expose a CLI (`python -m landlevel.cli`) with at least the following options:

**[15.0.2]**

```text
--agd PATH                 (required) input AGD file
--out PATH                 (required) output PNG path (base name for derived outputs)

# Field model & volumes
--grid-size-ft FLOAT          default 8.5   (matches default blade width; use 15 for an OptiHaul grid)
--tolerance-mm FLOAT          default 10
--scraper-capacity-lcy FLOAT  default 20    (bank limit = value / swell)
--swell-factor FLOAT          default 1.3   (LCY = BCY * swell)
--cutfill-ratio FLOAT         default 1.2   (alias: --compaction)
--max-depth-mm FLOAT          default 61
--blade-width-mm FLOAT        default 2590.8  (8.5 ft tandem effective width)
--strip-bias FLOAT            default 50    (grow each load as one blade-wide lane; 1 = blob)
--max-drop-hop-ft FLOAT       default 100   (multi-drop hop to next pocket; 0 disables)
--min-load-fill FLOAT         default 80    (% capacity at which multi-drop stops chasing; 100 = chase to full)

# Haul behaviour
--no-flow-aligned             grow strips along one global --heading-deg instead of the local flow (default: flow-aligned)
--heading-deg FLOAT           default auto  (design-surface main slope, cardinal-snapped; global heading only with --no-flow-aligned)
--heading-strength FLOAT      default 0.5   (0 disables heading bias)
--transport-mode {poisson,mincostflow}   default poisson
--balance-mode {check,adjust}            default check

# Exclusion zones (no-go polygons)
--equipment-width-ft FLOAT    default 14    (machine width; clearance buffer = half this)
--exclude-polygon "lat,lon;..."  repeatable; >= 3 vertices per polygon (Â§12.3)

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
--turning-circle-ft FLOAT     default 50    (turn diameter; radius 25 ft; sets turn-arc length)
--equipment-length-ft FLOAT    default 43    (train length; straight lineup before/after CUT/FILL)
--turn-threshold-deg FLOAT    default 60    (gentler bends below this cost no turn time)
--no-grade-return             do not add a grade/compact return pass over each fill
--no-route-smoothing          emit raw grid-stepped waypoints (default: smooth the displayed sweep into a drivable curve; display-only, time unchanged)
--no-knot-repair              leave undriveable knot sweeps as-is (default: re-profile them into boustrophedon parallel passes; Â§13.3b)
--no-direction-aware          revert to deadhead-distance ordering + 2-opt (no turn penalty)
--reorder-window INT          default 50; N>=2 enables windowed relocation refinement of the pass order (Â§12.2; typical 50)
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

# Center-pivot irrigation (optional, default off)
--pivot, --no-pivot           enable / disable pivot scheduling + outputs (Â§12.4)
--pivot-center "lat,lon"      WGS-84 anchor; default = field centroid
--pivot-wheelbase-ft FLOAT    default 14    (arm width, ft)
--pivot-clearance-ft FLOAT    default 25    (extra clearance per side, ft)
--pivot-move-cost FLOAT       default 25    (detour-equivalents per reposition)
--pivot-dir DIR               per-position PNG maps; default <out>_pivot
--pivot-csv [PATH]            move schedule; flag alone -> <out>_pivot.csv
--pivot-mode {cutfill,topology}  default cutfill (maps + animation)
--pivot-ppb INT               default 6     (pixels per area edge)
--pivot-gif [PATH]            leveling animation w/ arm moves; default <out>_pivot_leveling.gif
--no-pivot-gif                skip the pivot animation GIF
--pivot-frames DIR            also write animation as individual PNG frames
--pivot-anim-stride INT       default 15    (one frame per N passes; every move always framed)

# Misc
--seed INT                    default 1234
--version
```

**[15.0.3]** On success the program shall print an **OptiHaul-style report** (moved volume, pass count, haul heading, the four distances and achieved speeds, and the time breakdown incl. "Other" and TOTAL), write the plan PNG, and write any requested CSV/PNG/animation outputs, reporting each path. Invalid arguments shall produce a clear error and a non-zero exit code.

**[15.0.4]** Flags that derive a path from `--out` (`--pairs-csv`, `--pairs-png`, `--guidance-csv`, `--guidance-png`, `--animate-gif`, `--pivot-csv`, `--pivot-gif`, and `--sequence-csv` when zones are on) shall, when passed without an explicit path, name the file from the `--out` stem. **`--pivot-dir`** defaults to `<out>_pivot` when `--pivot` is set.

---

## 16. Reference implementation, build, and deliverables

### 16.1 Module layout

**[16.1.1]** Each pipeline stage shall be a separate, importable module:

**[16.1.2]**

```text
landlevel/
  __init__.py     # constants/defaults (grid size, swell, capacities, speeds)
  agd.py          # Â§6.1 AGD reader, Â§7.1 projection (utm_from_latlon, latlon_from_utm, project_latlon_ring)
  grid.py         # Â§7.2 binning, Â§7.2.8 exclusion mask (apply_exclusions), Â§7.3 BCY conversions, cell_center_xy
  balance.py      # Â§8 volume balance
  flowfield.py    # Â§9 optimal transport (poisson + mincostflow)
  strips.py       # Â§10 streamline tracing; flow_direction_field + streamline_to (for Â§11.0.2c)
  passes.py       # Â§11 contiguous patch matcher: flow-aligned tube (default) + single-heading strip bias, multi-drop, heading snap, 61 mm layering, obstacle-aware haul (Â§12.3)
  obstacles.py    # Â§12.3 exclusion zones: clearance buffer, visibility graph, Dijkstra path_length/path/blocked
  routing.py      # Â§12 sequencing (NN + bounded 2-opt; direction-aware/grade-aware transition cost; obstacle-aware deadhead) + Â§12.2 windowed relocation refinement (refine_route_order)
  sequence.py     # Â§12.1 zone work order + CSV exports
  timemodel.py    # Â§13.0 OptiHaul-matched core time model
  swaths.py       # Â§13.1â€“Â§13.6 Stage 7 blade-scale guidance (flow/heading lanes, route smoothing, obstacle + pivot-arm detours Â§13.4.2, grade pass, turns, guidance time, CSV)
  render.py       # Â§14 PNG (cut/fill + arrows), zone overlay, exclusion overlay, heat map, guidance lanes, frame renderers
  animate.py      # Â§14.6 leveling animation
  pivot.py        # Â§12.4 center-pivot scheduling, arm obstacles, detour stats, pivot PNG maps + animation (Â§14.7)
  cli.py          # Â§15 command-line entry point
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

**[17.0.2]** 1. **AGD round-trip** â€” a synthetic AGD parses to expected areas, codes, and `delta` signs (**Â§6.1**).
**[17.0.3]** 2. **BCY conversions** â€” cut/fill volume functions and the LCY â†’ BCY scraper conversion match **Â§7.3** on hand-checked values.
**[17.0.4]** 3. **Mass balance** â€” a balanced field reports near-zero imbalance; an unbalanced one is flagged (**Â§8**).
**[17.0.5]** 4. **Flow field** â€” for a one-ridge-one-valley field, Poisson flow points ridge â†’ valley (**Â§9**).
**[17.0.6]** 5. **Pass limits** â€” generated passes never exceed the scraper load limit (â‰ˆ 15.4 BCY) or 61 mm per area per pass, and volume is conserved per pass (**Â§11**).
**[17.0.7]** 6. **Render geometry** â€” a known area maps to the expected pixel block and arrow center (**Â§14.3**).
**[17.0.8]** 7. **Exclusion zones** â€” visibility-graph detour around a unit-square obstacle is longer than the straight line and clears the buffered zone; `apply_exclusions` removes the expected bin block and stores the polygon (**Â§7.2.8**, **Â§12.3**).
**[17.0.9]** 8. **Center pivot** â€” DP schedule on a synthetic pass list produces the expected number of positions; `arm_ring` + `build_arm_obstacles` block a straight haul through the parked arm; routed detour length exceeds the straight line (**Â§12.4**).

**[17.0.10]** The default field set shall include at least one field with explicit **`2PER`** boundary rows and one without, to exercise both masking paths (**Â§6.1**).

---

## 18. Open design decisions

**[18.0.1]** 1. **(Open)** Arrow sampling by fixed pixel stride vs area stride (currently area stride, **Â§14.2**).
**[18.0.2]** 2. **(Open)** Whether `balance_mode = adjust` may tilt the target plane (currently vertical offset only, **Â§8**).
**[18.0.3]** 3. **(Open)** Graph connectivity (4 vs 8) for min-cost flow (**Â§9.2**).
**[18.0.4]** 4. **(Open)** Whether the haul-heading default should track the *existing* surface slope rather than the *design* surface slope on fields where they differ materially (**Â§11.0.3b**).

---

## 19. Implementation checklist

**[19.0.1]** - â˜‘ AGD reader per **Â§6.1**, incl. `delta` recomputation and inverse projection.
**[19.0.2]** - â˜‘ UTM projection and 8.5 ft binning (default) per **Â§7.1**â€“**Â§7.2**.
**[19.0.3]** - â˜‘ BCY conversions and 61 mm layering per **Â§7.3**.
**[19.0.4]** - â˜‘ Volume balance check (and optional offset adjust) per **Â§8**.
**[19.0.5]** - â˜‘ Poisson flow field per **Â§9.1**; min-cost-flow mode per **Â§9.2**.
**[19.0.6]** - â˜‘ Streamline strips per **Â§10**.
**[19.0.7]** - â˜‘ Contiguous patch passes (â‰ˆ 15.4 BCY / 61 mm), true haul leg, heading bias, **flow-aligned tube growth (Â§11.0.2c, default)**, **strip-biased lanes (Â§11.0.2b)**, **multi-drop (Â§11.0.3c)**, **cardinal heading snap (Â§11.0.3b-i)** per **Â§11**.
**[19.0.8]** - â˜‘ Sequencing (NN + bounded 2-opt) and **direction-aware, grade-aware transition cost (Â§12.0.5)** per **Â§12**; zone work order + CSV per **Â§12.1**; **exclusion zones + visibility-graph routing (Â§12.3)** per **Â§12.3**.
**[19.0.9]** - â˜‘ OptiHaul-matched core time model per **Â§13.0**.
**[19.0.10]** - â˜‘ Stage 7 blade-scale guidance per **Â§13.1**â€“**Â§13.6**: flow/heading lanes, **route smoothing (Â§13.3a, display-only)**, **obstacle + pivot-arm detours (Â§13.4.2)**, grade/compact pass, turning model, multi-drop `MOVE`, guidance time, guidance CSV/PNG.
**[19.0.11]** - â˜‘ PNG cut/fill, arrows, zone overlay, **exclusion overlay**, heat map, animation per **Â§14**.
**[19.0.12]** - â˜‘ CLI per **Â§15** (incl. `--exclude-polygon`, `--equipment-width-ft`, **`--pivot` / `--no-pivot`**, **`--pivot-center`**, pivot animation/output flags).
**[19.0.13]** - â˜‘ **Center-pivot scheduling** per **Â§12.4**: DP arm placement, per-pass arm obstacles, guidance detours (**Â§13.4.2**), detour time in report, CSV/PNG/animation (**Â§14.7**); module **`pivot.py`**.
**[19.0.14]** - â˜‘ Tests per **Â§17**; deterministic output per **Â§16.4**.
**[19.0.15]** - â˜‘ All source and this spec stored UTF-8 without BOM.

---

## 20. Summary

**[20.0.1]** This v2 specification defines a Python program that turns an AGD field file plus equipment and time parameters into a haul plan, a rendered PNG, and optional sequencing/CSV/heat-map/animation/guidance/**pivot** outputs. The field is modeled on a **7.5 ft** grid (â‰ˆ one blade width; configurable, e.g. 15 ft to match an OptiHaul calculation grid); volumes are tracked in **BCY**, with **swell** applied only to convert the scraper's **LCY** capacity to a bank load limit (20 LCY Ã· 1.3 â‰ˆ 15.4 BCY) and a **cut/fill ratio** for compaction. Haul directions come from an optimal-transport flow field traced into curving strips; the volume plan is produced by a shortest-haul-first matcher that, by **default**, grows each load **along the local transport flow** â€” cut traced upstream to the ridge, fill downstream into the sink â€” for long, curved, with-the-grain lanes (`--no-flow-aligned` reverts to one straight **blade-wide strip** along a cardinal-snapped heading via strip bias). The matcher keeps loading/spreading across nearby pockets via **multi-drop** (capped by **`--min-load-fill`** so it does not chase the last few percent into a U-turn blob, and kept **spatially compact** so no load spans an empty gap that would force a blade-up move covering nothing) and measures the per-pass **haul leg as the true transit** (cut-end â†’ fill-start), **routing around any exclusion zone** via a **visibility graph + Dijkstra** when `--exclude-polygon` is set. **Exclusion zones** drop the bins covering each no-go polygon from the field mask (volume imbalance is treated as exported/imported) and buffer each polygon outward by half **`--equipment-width-ft`** (default 14 ft) so blade-up travel never crosses the zone. Passes are sequenced to minimize non-productive travel using a **direction-aware, grade-aware** transition cost that trades deadhead drive against turning, and costed with an **OptiHaul-matched** core time model. On request, a **center-pivot irrigation schedule** (**Â§12.4**, `--pivot`) places a pivot at the field centroid or a user-specified lat/lon, sizes it as large as fits, and uses **dynamic programming** to minimise arm moves vs haul detours; blade-up legs that cross the parked arm are routed around it in the **guidance track** (same visibility-graph mechanism, per pass), with extra loaded-haul time added to the report. On request, a **blade-scale operator guidance** track (Stage 7) realizes the passes as turn-by-turn waypoints on a **smoothed, drivable GPS route** (corner radius tied to the turning circle; a steering-display transform that never alters distance/time, which stay on the raw coverage geometry), with **detour vertices on travel legs** around exclusion zones and the parked pivot arm, a **grade/compact return pass** over every fill and a **realistic drivable time** that counts **turning** on the 45 ft circle â€” giving the honest field time, distinct from the OptiHaul-matched core figure. Sweeps that would still chain into an undriveable **knot** are re-profiled into **boustrophedon** parallel back-and-forth lanes (only where that strictly reduces hairpins, pocket-aware, and without changing any cell, load, haul, or the finished surface; `--no-knot-repair` disables it). Optional outputs also summarise the plan as a **zone work order** (with CSV and numbered PNG waypoints), export **per-pass lat/lon pairs** to CSV, render a **haul heat map** and the **guidance lanes**, produce a frame-by-frame **leveling animation** in cut/fill or topology mode, and â€” when `--pivot` is set â€” a **pivot move schedule** (CSV), **per-position PNG maps**, and a **pivot leveling animation** with a held frame at every arm move. The PNG reuses the existing AgGrade palette, arrow glyph, and image geometry so output is directly comparable to the current tools.
