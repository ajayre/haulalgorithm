# Topsoil Optimization Specification **[0.0.1]**

**[0.0.1]** This document specifies an **optional extension** to the land leveling haul algorithm: **topsoil optimization**. When enabled, the pass matcher distinguishes **precision-grade fill** (the final field surface) from **structural fill** (roads, pads, berms, and any other build-up above the precision grade), and routes **subgrade material from deeper cuts** to structural fill while preserving good material for precision grading.

**[0.0.2]** **Relationship to LAND-LEVELING-SPEC-2.md.** The baseline haul algorithm (**LAND-LEVELING-SPEC-2.md**, hereafter **baseline spec**) remains authoritative for AGD reading, grid model, flow field, pass geometry, sequencing, guidance, and time models. This document **extends** baseline **§11** (pass generation) and adds one preprocessing stage after **§7** (grid build). It **shall not** change volume accounting (**§7.3**), haul leg geometry (**§11.0.13**), routing (**§12**), or blade-scale guidance (**§13**) except for optional pass metadata and reporting called out below.

**[0.0.3]** **Language / implementation.** This specification is **implementation-language-neutral**. Any conforming implementation of the baseline spec may implement this extension. Command-line options use POSIX-style **`--name`** flags; implementations may expose the same parameters via configuration files or APIs using equivalent names.

**[0.0.4]** **Related documents:** **LAND-LEVELING-SPEC-2.md** (baseline algorithm), **LAND_LEVELING_PROBLEM.md** (physical model), **ANDY-SPECSTYLEGUIDE.md** (authoring rules).

**[0.0.5]** Every normative paragraph carries a bold marker **`[<S>.<U>.<P>]`** where **`<S>`** is the section, **`<U>`** is the subsection (**`0`** for body text before the first **`###`**), and **`<P>`** is the 1-based paragraph index within that group.

### 0.1 Specification revision history

**[0.1.1]** | Version | Date | Change |
|---|---|---|
| **0.0.1** | 2026-07-05 | Initial draft: precision-grade vs structural fill, feature-lift auto-detection, subgrade cut eligibility, two-phase matcher, CLI, outputs. |

**[0.1.2]** This document follows [semantic versioning](https://semver.org/): **MAJOR** = incompatible normative change, **MINOR** = backward-compatible addition, **PATCH** = clarification.

---
## 1. Purpose

**[1.0.1]** Earthmoving for leveled fields often wastes **good topsoil** when the same undifferentiated cut–haul–fill matcher places premium material into **non-production features** — access roads, turnout pads, berms, and other localized build-ups that sit **above** the final precision-drained field surface.

**[1.0.2]** Topsoil optimization **shall**, when enabled, automatically:

**[1.0.3]** 1. Identify areas whose **target elevation exceeds the precision grade** at the same location (feature build-up), without requiring the operator to draw a polygon for every road or pad.

**[1.0.4]** 2. Classify fill demand in those areas as **structural fill**, suitable for **subgrade** material from **deeper cuts**.

**[1.0.5]** 3. Reserve **shallow cut material** (early layers) primarily for **precision-grade fill** — the fill needed to bring low areas up to the production surface.

**[1.0.6]** 4. Preserve **volume conservation**, **layer ordering**, and all baseline pass geometry, sequencing, and guidance behaviour outside the matcher compatibility rules defined in **§6**.

**[1.0.7]** When disabled (default), behaviour **shall** be identical to the baseline spec.

---
## 2. Design goals

**[2.0.1]** The extension **shall provide**:

**[2.0.2]** 1. **Automatic** structural-fill detection from the AGD target surface alone (**§4**, **§5**).

**[2.0.3]** 2. A **simple depth proxy** for material quality using cumulative cut depth already removed from each cell (**§6.2**), without soil-lab data or extra AGD layers.

**[2.0.4]** 3. **One material class per pass** (grade or subgrade) so operator guidance and field execution stay straightforward.

**[2.0.5]** 4. **Rare manual overrides** only for exceptions — not a polygon per feature (**§8**).

**[2.0.6]** 5. **Inspectable outputs** (map overlay and summary statistics) so misclassification is visible before execution (**§9**).

**[2.0.7]** 6. A **minimal integration surface**: one preprocessing stage, one ledger counter, matcher anchor/filter changes only (**§7**).

---
## 3. Non-goals

**[3.0.1]** This extension **shall not** attempt the following in this version:

**[3.0.2]** 1. Reading **soil horizon**, **USCS class**, or **laboratory quality** from the AGD or external databases.

**[3.0.3]** 2. **Splitting a single scraper load** into mixed grade and subgrade destinations.

**[3.0.4]** 3. **Stockpile management** (export/import yards, separate spoil piles) beyond reporting unmatched structural demand.

**[3.0.5]** 4. **Road corridor geometry** (width, crown, one-sided slope, multi-pass lane layout). Material routing is in scope; drivable road construction is a separate future extension.

**[3.0.6]** 5. Changing the **target surface** or re-solving field drainage. The precision grade is **derived** from the supplied target, not redesigned.

**[3.0.7]** 6. Replacing the baseline **shortest-haul-first** objective globally. Compatibility filtering applies **before** haul minimization among remaining legal pairs.

---
## 4. Terminology

### 4.1 Precision grade

**[4.1.1]** The **precision grade** is the **large-scale production surface** implied by the AGD target elevations — the smooth field drainage plane onto which the field is precision-leveled. Localized features above that plane are **not** part of the precision grade.

### 4.2 Feature build-up (structural lift)

**[4.2.1]** **Feature build-up** is the positive vertical offset of the design target above the precision grade at the same horizontal location:

```text
feature_lift = max(0, target − precision_target)
```

**[4.2.2]** Examples include access roads, square pads, berms, and raised turnouts. Depressions filled **up to** the precision grade have **`feature_lift ≈ 0`** and are **not** feature build-ups.

### 4.3 Precision-grade fill and structural fill

**[4.3.1]** **Precision-grade fill** (also **grade fill**) is fill demand in areas where **`feature_lift`** is below the configured threshold (**§5.3**). This material **should** preferentially come from **shallow cuts** (any available cut source under baseline rules).

**[4.3.2]** **Structural fill** is fill demand in areas where **`feature_lift`** meets or exceeds the configured threshold. This material **shall** be sourced only from **subgrade-eligible cuts** (**§6.2**).

### 4.4 Grade material and subgrade material

**[4.4.1]** **Grade material** is soil excavated from a cut cell in **early layers** — before the cell has contributed cumulative cut depth reaching the subgrade threshold (**§6.2**).

**[4.4.2]** **Subgrade material** is soil excavated from a cut cell once **`cut_removed`** (cumulative depth already taken from that cell) **≥ subgrade_after_depth**. This is the implementation's proxy for **lower-quality, deeper** material suitable for roads, pads, and berms.

### 4.5 Material class (pass attribute)

**[4.5.1]** Each pass **shall** carry a **material class**:

| Value | Meaning |
|---|---|
| **`GRADE`** | Cut supplies precision-grade fill only. |
| **`SUBGRADE`** | Cut supplies structural fill only; cut cells must be subgrade-eligible. |

**[4.5.2]** A pass **shall not** mix grade and structural fill destinations, nor mix grade and subgrade cut sources.

---
## 5. Stage T1 — Precision surface and feature lift

**[5.0.1]** Topsoil optimization **shall** run **once** immediately after the baseline grid is built (**baseline §7.2**) and **before** pass generation (**baseline §11**). It **shall** attach the following per-cell arrays on the grid (or equivalent runtime structure), all indexed **`[by, bx]`** on valid areas:

| Array | Type | Meaning |
|---|---|---|
| **`precision_target`** | float (m) | Large-scale precision grade elevation. |
| **`feature_lift`** | float (m) | Non-negative build-up above precision grade. |
| **`structural_fill_mask`** | bool | **`true`** where fill is structural (**§5.4**). |
| **`grade_fill_mask`** | bool | Valid fill areas that are not structural. |

**[5.0.2]** **`precision_target`** **shall** be computed only from **target** elevations on **valid** cells, using a **field-scale smoother** that removes localized features while retaining basin-scale drainage. Implementations **shall** support at least the modes in **§5.2**.

### 5.1 Smoothing scale

**[5.1.1]** The smoother **shall** be controlled by **`--precision-smooth-ft`** (default **150 ft**): a characteristic length in **feet** over which localized build-ups are absorbed into the precision surface. This scale **shall** be **much larger** than the grid cell size and **much smaller** than the field diameter — it follows **field-grade** geometry, not blade-scale noise.

**[5.1.2]** **`--precision-smooth-ft`** that is too small **shall** be warned about (risk: misclassifying precision grade as structural). Values below **`10 × grid_size_ft`** should trigger a warning.

### 5.2 Precision surface modes

**[5.2.1]** **`--precision-mode`** selects how **`precision_target`** is computed:

| Mode | Description |
|---|---|
| **`plane`** (default) | Best-fit plane to **target** over valid cells (least squares). Suitable for typical drained fields. |
| **`median`** | Per-cell target minus a wide **median filter** with window derived from **`precision-smooth-ft`**. Suitable when a plane is a poor fit. |
| **`gaussian`** | Per-cell **Gaussian smooth** of **target** with sigma derived from **`precision-smooth-ft`**. |

**[5.2.2]** Implementations **may** add modes; new modes **shall** be documented in revision history. Default behaviour **shall** match **`plane`** when **`--precision-mode`** is omitted.

### 5.3 Feature lift and structural threshold

**[5.3.1]** For each valid cell:

```text
feature_lift = max(0, target − precision_target)
```

**[5.3.2]** **`--min-feature-lift-ft`** (default **0.4 ft**) is the minimum **feature_lift** for structural classification. Cells with **`feature_lift ≥ min_feature_lift_ft`** and **`delta < −tolerance`** (needs fill) **shall** be marked structural. All other fill cells **shall** be grade fill.

**[5.3.3]** **`min_feature_lift_ft`** **shall** be small enough to catch intentional pads and roads (typically **≥ 0.3 ft**) and large enough to ignore grid noise and minor smoothing residuals (typically **≤ 0.6 ft** for default grid sizes).

### 5.4 Mask combination with manual overrides

**[5.4.1]** After auto-detection, masks **shall** be combined with optional override polygons (**§8**):

```text
structural_fill_mask =
    (auto_structural OR manual_add_polygon)
    AND NOT manual_grade_polygon
    AND valid
    AND needs_fill

grade_fill_mask = valid AND needs_fill AND NOT structural_fill_mask
```

**[5.4.2]** **`needs_fill`** follows the baseline tolerance: **`remaining_fill > tolerance`** at planning start, equivalently **`existing − target < −tolerance`** on the initial surface.

---
## 6. Stage T2 — Material-aware pass matching

**[6.0.1]** When **`--topsoil-optimize`** is enabled, pass generation **shall** extend the baseline matcher (**baseline §11**) with a **`cut_removed`** ledger and **two-phase fill routing** below. All other grow, multi-drop, haul-leg, and volume-conservation rules **shall** remain unchanged.

### 6.1 Cut-removed ledger

**[6.1.1]** The matcher **shall** maintain **`cut_removed[by, bx]`** (metres), initialized to **0**, recording **cumulative cut depth already taken** from each cut cell across all passes.

**[6.1.2]** After each pass, for every cut cell **`c`** with depth **`d`** removed in that pass:

```text
cut_removed[c] += d
```

**[6.1.3]** **`cut_removed`** **shall not** be decremented except by pass rollback during the same matcher iteration (e.g. baseline leftover refund paths). It is monotonic over the job.

### 6.2 Subgrade eligibility

**[6.2.1]** **`--subgrade-after-ft`** (default **0.4 ft**) defines **`subgrade_after_m`**, the cumulative cut depth after which material from a cell counts as **subgrade**.

**[6.2.2]** At pass-generation time, a cut cell **`c`** is **subgrade-eligible** iff:

```text
cut_removed[c] ≥ subgrade_after_m
```

**[6.2.3]** **`subgrade_after_ft`** **shall** default near **`min_feature_lift_ft`** so that by the time feature build-ups are filled, typical cut cells have begun producing subgrade material. Implementations **should** document recommended pairs in the run summary when both differ by more than **0.2 ft**.

### 6.3 Two-phase matcher

**[6.0.2]** Each matcher iteration **shall** operate in one of two **phases**. Phase selection **shall** depend on remaining **grade** vs **structural** fill demand and subgrade supply:

**[6.3.1]** **Phase A — grade fill (default when grade fill demand remains).**

| Ledger / filter | Rule |
|---|---|
| Fill mask | **`grade_fill_mask`** AND remaining fill |
| Cut filter | **None** (any cut cell with remaining cut) |
| Pass material class | **`GRADE`** |

**[6.3.2]** **Phase B — structural fill (when grade fill is exhausted, or when Phase A cannot proceed).**

| Ledger / filter | Rule |
|---|---|
| Fill mask | **`structural_fill_mask`** AND remaining fill |
| Cut filter | **Subgrade-eligible** cut cells only |
| Pass material class | **`SUBGRADE`** |

**[6.3.3]** **Phase B hold.** If structural fill demand remains but **no** subgrade-eligible cut supply remains **above tolerance**, the matcher **shall not** place structural fill using grade material. It **shall** leave structural **`remaining_fill`** in place and continue with any remaining **grade** cut/fill pairs that do not require structural destinations, or terminate with reported **held structural demand** (**§9**).

**[6.3.4]** **Anchor selection (normative).** Within the active phase, the anchor cut cell **shall** be chosen exactly as baseline **§11** (nearest remaining fill among **compatible** fill cells — minimum haul distance, same KD-tree or brute-force logic), except that:

**[6.3.5]** - In Phase A, only **grade** fill cells are eligible fill targets.

**[6.3.6]** - In Phase B, only **structural** fill cells are eligible fill targets, and only **subgrade-eligible** cut cells may anchor or contribute to the cut segment.

**[6.3.7]** **Fill seed selection** **shall** apply the same compatibility filters after the cut patch is fixed (**baseline §11.0.16**, flow-aligned seeding unchanged).

**[6.3.8]** **Multi-drop** cut and fill loops (**baseline §11.0.18**) **shall** respect the active phase masks: growth **shall not** consume cells outside the permitted fill or cut sets.

### 6.4 Layer ordering interaction

**[6.4.1]** Baseline **layer ordering** (**baseline §11.0.20**, **§12.0.2**) **shall** be preserved: shallow layers before deeper layers on each cell.

**[6.4.2]** Topsoil optimization **uses** that ordering deliberately: early passes excavate **grade material**; later passes on the same cut cells become **subgrade-eligible** as **`cut_removed`** accumulates — aligning structural fills with deeper material in time as well as in depth.

### 6.5 Volume conservation

**[6.5.1]** Restricting cut–fill pairings **shall not** create or destroy volume. Unmatched cut or fill BCY **shall** be reported exactly as baseline **§11.0.22**, with additional structural/grade breakdown (**§9**).

**[6.5.2]** Refund and leftover paths (**baseline §11.0.12**, **§11.0.18**) **shall** restore **`remaining_cut`**, **`remaining_fill`**, and **`cut_removed`** consistently so the finished surface model is unchanged whether or not optimization is enabled — only **which source fed which destination** changes.

---
## 7. Integration with baseline pipeline

**[7.0.1]** The following baseline stages **shall not** be modified except as noted:

| Stage | Topsoil interaction |
|---|---|
| AGD read, grid binning (**§6–§7**) | Unchanged |
| **T1** Precision / feature lift (**§5**) | **New**, after grid |
| Volume balance (**§8**) | Unchanged; may report grade vs structural demand split |
| Flow field, strips (**§9–§10**) | Unchanged |
| **T2** Pass matcher (**§6**) | **Extended** when enabled |
| Sequencing (**§12**) | Unchanged; passes carry **`material_class`** |
| Guidance (**§13**) | Unchanged paths; **may** emit material class in CSV |
| Rendering (**§14**) | **May** overlay structural mask (**§9**) |

**[7.0.2]** Implementations **shall** expose topsoil optimization as a single master switch defaulting **off** so existing workflows and regression tests remain valid without flags.

---
## 8. Manual overrides (exceptions only)

**[8.0.1]** Manual polygons **shall not** be required for normal operation. They exist to **correct** auto-detection edge cases.

**[8.0.2]** **`--structural-fill-polygon "lat,lon;..."`** (repeatable, **≥ 3** vertices per polygon) **shall** **add** cells intersecting the polygon to **`structural_fill_mask`**, subject to **`needs_fill`**. Projection rules **shall** match baseline exclusion polygons (**baseline §7.2.8**, **§12.3**).

**[8.0.3]** **`--grade-fill-polygon "lat,lon;..."`** (repeatable) **shall** **force** cells to **grade fill**, removing them from **`structural_fill_mask`** even when **`feature_lift`** exceeds the threshold.

**[8.0.4]** Overrides **shall** be applied after auto-detection (**§5.4**). **`--grade-fill-polygon`** **shall** win over **`--structural-fill-polygon`** on overlap.

**[8.0.5]** **`--no-feature-auto-detect`** **shall** disable **§5.3** auto classification. When set, **`structural_fill_mask`** **shall** contain only manual **`--structural-fill-polygon`** cells (if any). With no polygons and auto off, behaviour **shall** revert to baseline matching (all fill is grade fill).

---
## 9. Outputs and reporting

### 9.1 Run summary (required when enabled)

**[9.1.1]** When **`--topsoil-optimize`** is on, the program **shall** print a **topsoil summary** including at minimum:

| Metric | Description |
|---|---|
| Grade fill demand | BCY in **`grade_fill_mask`** at start |
| Structural fill demand | BCY in **`structural_fill_mask`** at start |
| Grade fill placed | BCY placed in grade passes |
| Structural fill placed | BCY placed in subgrade passes |
| Grade passes / subgrade passes | Pass counts by **`material_class`** |
| Subgrade supply | BCY cut from subgrade-eligible cells (cumulative) |
| Held structural demand | BCY structural still unmatched because subgrade supply was insufficient |
| Unmatched grade / structural | Remaining BCY by class at termination |

**[9.1.2]** If **held structural demand > 0**, the summary **shall** include an explicit **WARNING** suggesting: lower **`--subgrade-after-ft`**, raise cut depth availability, import structural material, or review **`--min-feature-lift-ft`** / **`--precision-smooth-ft`**.

### 9.2 Map overlay

**[9.2.1]** **`--topsoil-map-png PATH`** **shall** write a PNG overlay on the cut/fill map showing:

**[9.2.2]** - **`precision_target`** contours or a faint smooth surface reference (implementation choice).

**[9.2.3]** - **`feature_lift`** as a colour ramp or hatch on fill areas.

**[9.2.4]** - **`structural_fill_mask`** cells highlighted (e.g. cross-hatch or distinct fill colour).

**[9.2.5]** When **`--topsoil-map-png`** is omitted but **`--topsoil-optimize`** is on, the default path **shall** be **`<out>_topsoil.png`** derived from **`--out`** (**baseline §14** base name rules).

### 9.3 Optional CSV

**[9.3.1]** **`--topsoil-csv PATH`** **may** emit one row per valid cell: **`bx, by, lat, lon, target_m, precision_target_m, feature_lift_ft, structural_bool, grade_fill_demand_bcy, structural_fill_demand_bcy`**.

**[9.3.2]** Default when flag is given without path: **`<out>_topsoil.csv`**.

### 9.4 Pass metadata

**[9.4.1]** Pass records **shall** include **`material_class ∈ {GRADE, SUBGRADE}`** when optimization is enabled.

**[9.4.2]** Guidance CSV (**baseline §13.6**) **should** add a **`material_class`** column on each row (same value for all waypoints in a pass). When optimization is off, the column **may** be omitted or set to **`GRADE`** for all passes.

---
## 10. Command-line interface

**[10.0.1]** Implementations **shall** expose the following options in addition to the baseline CLI (**baseline §15**). Names and defaults are normative; short aliases **may** be provided.

**[10.0.2]**

```text
# Master switch (default: off — baseline behaviour)
--topsoil-optimize            Enable topsoil optimization (§5–§6)
--no-topsoil-optimize         Disable explicitly (default)

# Precision surface & feature detection (§5)
--precision-mode {plane,median,gaussian}
                              default plane
--precision-smooth-ft FLOAT   default 150   (field-scale smoother width, ft)
--min-feature-lift-ft FLOAT   default 0.4   (target − precision threshold, ft)
--no-feature-auto-detect      Structural mask from polygons only (§8.0.5)

# Subgrade source rule (§6.2)
--subgrade-after-ft FLOAT     default 0.4   (cumulative cut depth before
                              material is subgrade-eligible, ft)

# Manual overrides — exceptions only (§8)
--structural-fill-polygon "lat,lon;..."
                              Repeatable; force structural classification
--grade-fill-polygon "lat,lon;..."
                              Repeatable; force grade classification

# Outputs (§9)
--topsoil-map-png PATH        Structural / feature-lift overlay PNG
--topsoil-csv PATH            Per-cell classification CSV (optional)
```

**[10.0.3]** **`--topsoil-optimize`** **shall** imply **`--topsoil-map-png`** default output unless **`--no-topsoil-map`** is provided (implementations **may** add **`--no-topsoil-map`** as a convenience to suppress the default PNG only).

**[10.0.4]** All length options **shall** be interpreted in **feet** regardless of locale; internal computation **shall** convert to metres consistently with the baseline grid (**baseline §7.1**).

---
## 11. Worked example (informative)

**[11.0.1]** A field includes a **16 ft × 800 ft** access road at **+1.5 ft** above the surrounding precision grade, and a **40 ft × 40 ft** equipment pad at **+1.0 ft**. Both appear in the AGD target as localized raises.

**[11.0.2]** With defaults (**`precision-smooth-ft = 150`**, **`min-feature-lift-ft = 0.4`**, **`subgrade-after-ft = 0.4`**):

**[11.0.3]** 1. **T1** computes **`precision_target`** as the field-scale plane (or smooth surface). Road and pad cells show **`feature_lift ≈ 1.5 ft`** and **`≈ 1.0 ft`** → both marked **structural**.

**[11.0.4]** 2. **Phase A** matches shallow cuts to depression fills until grade fill demand is satisfied.

**[11.0.5]** 3. As **`cut_removed`** accumulates, cut cells exceed **0.4 ft** removed → **Phase B** routes that material to the road and pad.

**[11.0.6]** 4. No polygons were drawn. **`_topsoil.png`** shows the road and pad hatched as structural before execution.

---
## 12. Implementation checklist

**[12.0.1]** A conforming implementation **shall** satisfy:

**[12.0.2]** - ☑ **`--no-topsoil-optimize`** (default): bit-identical pass plan to baseline on the same inputs (within floating-point tolerance).

**[12.0.3]** - ☑ **T1** arrays computed deterministically from target and flags (**§5**).

**[12.0.4]** - ☑ **`cut_removed`** updated on every pass (**§6.1**).

**[12.0.5]** - ☑ Phase A/B rules and hold behaviour (**§6.3**).

**[12.0.6]** - ☑ Volume conservation and refund paths (**§6.5**).

**[12.0.7]** - ☑ Summary metrics and default **`_topsoil.png`** (**§9**).

**[12.0.8]** - ☑ Unit tests: plane pad, linear road, depression-only fill, held structural demand, polygon overrides, auto-off mode.

---
## 13. Summary

**[13.0.1]** Topsoil optimization separates **precision-grade fill** from **structural fill** by detecting **feature build-up** above a **smoothed precision grade** derived from the AGD target — covering roads, pads, berms, and any raised feature without per-feature polygons. The matcher routes **shallow cut material** to grade fill first and **deeper cut material** (after **`subgrade-after-ft`**) to structural fill, preserving baseline geometry, sequencing, and guidance while improving material use. The feature defaults **off** and is controlled by **`--topsoil-optimize`** and the options in **§10**.
