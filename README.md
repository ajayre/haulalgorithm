# Land Leveling Haul Algorithm

Reference implementation of **LAND-LEVELING-SPEC.md**: a Python pipeline that
reads an OptiSurface-style **AGD** field file, plans a sequence of
cut–haul–fill **passes** that bring the field to its target grade with minimum
haul distance and time, and renders a **PNG** of the cut/fill map with overlaid
haul-direction arrows. It also produces an OptiHaul-style time report and a
range of optional outputs (zone work order, per-pass CSVs, a haul "heat map",
a leveling animation, and a **blade-scale operator guidance** track with a
realistic drivable time that counts turning and the grade/compact pass).

## Install

```bash
pip install -r requirements.txt
```

Core dependencies are **NumPy** and **Pillow**. **SciPy** (optional) provides a
fast sparse solver for the Poisson flow field; without it a NumPy iterative
solver is used. **networkx** (optional) enables the `mincostflow` transport
mode; without it that mode falls back to `poisson` with a warning.

## Quick start

Process an AGD file and write the cut/fill PNG plus the time report:

```bash
python -m landlevel.cli --agd TheShop2_2ft.agd --out theshop_plan.png
```

`--agd` (input field) and `--out` (output PNG) are the only required arguments.
On success the tool writes the PNG and prints the **OptiHaul Analysis Results**
(see [Output](#output) below).

### Common recipes

PNG only — stages 1–3, skips the pass/time plan (fast):

```bash
python -m landlevel.cli --agd TheShop2_2ft.agd --out cutfill.png --no-plan
```

Coarser/faster grid (spec's original 30 ft areas):

```bash
python -m landlevel.cli --agd TheShop2_2ft.agd --out plan.png --grid-size-ft 30
```

Write the rough cut work order and every cut/fill pair to CSV:

```bash
python -m landlevel.cli --agd TheShop2_2ft.agd --out plan.png --pairs-csv
```

Haul "heat map" — one semi-transparent line per cut/fill pair:

```bash
python -m landlevel.cli --agd TheShop2_2ft.agd --out plan.png --pairs-png
```

Leveling animation as a GIF (topology view that flattens to grade):

```bash
python -m landlevel.cli --agd TheShop2_2ft.agd --out plan.png \
  --animate-gif --animate-mode topology
```

Pin the haul heading to a known slope direction (e.g. due west = 270°):

```bash
python -m landlevel.cli --agd TheShop2_2ft.agd --out plan.png --heading-deg 270
```

Blade-scale operator guidance — turn-by-turn waypoints + a realistic drivable
time (turns, grade/compact pass, multi-drop), with the lanes drawn over the map:

```bash
python -m landlevel.cli --agd TheShop2_2ft.agd --out plan.png \
  --guidance-csv --guidance-png
```

## Command-line options

Full list also available via `python -m landlevel.cli --help`. Defaults come
from spec §6.2–§6.4.

### Required

| Option | Meaning |
|---|---|
| `--agd PATH` | input AGD field file |
| `--out PATH` | output PNG path (also the base name for derived outputs) |

### Field model & volumes

| Option | Default | Meaning |
|---|---|---|
| `--grid-size-ft` | 7.5 | square area (cell) edge length in feet (≈ one blade width, so each load is one blade-wide lane; use 15 to match an OptiHaul coarse grid) |
| `--tolerance-mm` | 10 | finished-area elevation tolerance (cells within this of grade are "on grade") |
| `--scraper-capacity-lcy` | 20 | scraper capacity in **LCY**; per-pass bank limit = value ÷ swell ≈ 15.4 BCY |
| `--swell-factor` | 1.3 | `LCY = BCY × swell` (loose soil is bulkier than bank soil) |
| `--cutfill-ratio`, `--compaction` | 1.2 | bank cut volume needed per bank fill volume (compaction allowance) |
| `--max-depth-mm` | 61 | max cut/fill depth per area per pass (0.2 ft) |
| `--blade-width-mm` | 2271 | scraper blade/cut width (the tandem 7 ft + 10 ft scrapers have an ≈ 8.5 ft volume-weighted effective width — pass `2590.8` to model that) |
| `--strip-bias` | 50 | grow each load's cut/fill as one long blade-wide lane along the heading (≈ the lane length in cells chased before stepping sideways; 1 = compact blob) |
| `--max-drop-hop-ft` | 100 | multi-drop: how far the scraper drives (blade up) to the next cut/fill pocket to keep loading/spreading the same load instead of ending a pass part-full (0 disables multi-drop) |

### Haul behaviour

| Option | Default | Meaning |
|---|---|---|
| `--heading-deg` | auto | haul bearing (compass deg, 0=N, 90=E); default = the design surface's main down-slope |
| `--heading-strength` | 0.5 | 0–0.95: how strongly fill selection follows the heading (0 disables heading bias) |
| `--transport-mode` | `poisson` | flow field: `poisson` (L²) or `mincostflow` (L¹, needs networkx) |
| `--balance-mode` | `check` | `check` (warn on imbalance) or `adjust` (re-level the target to balance) |

### Time model

| Option | Default | Meaning |
|---|---|---|
| `--field-efficiency` | 0.8 | productive fraction of total time; 0.8 = 80% (the rest is "Other" time) |
| `--loading-speed-mph` | 4 | constant loading (cutting) sweep speed |
| `--haul-speed-mph` | 10 | maximum haul speed (trapezoidal profile reduces the average) |
| `--dumping-speed-mph` | 5 | constant dumping (spreading) sweep speed |
| `--return-speed-mph` | 13 | maximum return (empty) speed |
| `--accel-time-s` | 12 | time to accelerate to max speed (lowers average haul/return speed) |
| `--decel-time-s` | 12 | time to decelerate from max speed |

### Maneuvering & blade-scale guidance

These drive the realistic, drivable plan: the turn-aware guidance time, the
grade/compact return pass, and the turn-aware sequencing.

| Option | Default | Meaning |
|---|---|---|
| `--turn-speed-mph` | 4 | speed through a turn (the machine slows to turn); used by the turn-aware guidance time and sequencer |
| `--grade-speed-mph` | 8 | speed of the return grade/compact pass driven back over each fill |
| `--turning-circle-ft` | 45 | tractor + scraper turning circle (diameter); sets the turn-arc length in the guidance time |
| `--turn-threshold-deg` | 60 | heading changes below this are gentle bends driven in stride (no slowdown); only sharper turns cost turn time |
| `--no-grade-return` | off | do **not** add a grade/compact return pass over each fill |
| `--no-direction-aware` | off | disable direction-aware sequencing (turn-into-cut penalty); fall back to pure deadhead-distance ordering + 2-opt |
| `--lane-overlap` | 0.0 | 0–0.9: fractional overlap between adjacent blade lanes (0 = lanes butt edge-to-edge) |
| `--guidance-csv [PATH]` | off | write the blade-scale operator guidance track (turn-by-turn waypoints with depth/heading/instruction); pass the flag **alone** for `<out>_guidance.csv` |
| `--guidance-png [PATH]` | off | render the blade-scale guidance lanes over the cut/fill map; pass the flag **alone** for `<out>_guidance.png` |

### PNG rendering

| Option | Default | Meaning |
|---|---|---|
| `--pixels-per-bin` | 50 | PNG pixels per area edge (resolution of the main plan PNG) |
| `--arrow-stride` | 2 | draw a haul-direction arrow every N areas (0 disables arrows) |
| `--no-plan` | off | render the cut/fill PNG from stages 1–3 only (skip passes/time) |

### Sequencing & extra outputs

| Option | Default | Meaning |
|---|---|---|
| `--zones` | 8 | number of cut zones for the rough work-order sequence (0 disables) |
| `--sequence-csv PATH` | `<out>_cut_sequence.csv` | write the cut-zone work order (incl. `lat`/`lon`) |
| `--pairs-csv [PATH]` | off | per-pass cut/fill pairs in optimized order with start/end lat/lon; pass the flag **alone** for `<out>_cut_fill_pairs.csv` |
| `--pairs-png [PATH]` | off | cut/fill map overlaid with one semi-transparent line per cut/fill pair (heat-map look); pass the flag **alone** for `<out>_cut_fill_pairs.png` |

### Animation

| Option | Default | Meaning |
|---|---|---|
| `--animate-dir DIR` | off | write a PNG frame per stride of applied passes into this directory |
| `--animate-gif [PATH]` | off | assemble the frames into an animated GIF; pass the flag **alone** for `<out>_leveling.gif` (final frame holds 4 s) |
| `--animate-mode` | `cutfill` | frame contents: `cutfill` (cut/fill map turning green at grade) or `topology` (elevation terrain flattening to grade) |
| `--animate-stride` | 10 | emit one animation frame per N applied passes |
| `--animate-ppb` | 8 | pixels per area edge for animation frames |

### Misc

| Option | Default | Meaning |
|---|---|---|
| `--seed` | 1234 | random seed (deterministic zone clustering) |
| `--version` | — | print the version and exit |

### Outputs that derive their name from `--out`

When you pass these flags **without** an explicit path, the file is named from
the `--out` stem (e.g. `--out theshop_plan.png` →):

| Flag (alone) | File written |
|---|---|
| `--sequence-csv` (with `--zones > 0`) | `theshop_plan_cut_sequence.csv` |
| `--pairs-csv` | `theshop_plan_cut_fill_pairs.csv` |
| `--pairs-png` | `theshop_plan_cut_fill_pairs.png` |
| `--guidance-csv` | `theshop_plan_guidance.csv` |
| `--guidance-png` | `theshop_plan_guidance.png` |
| `--animate-gif` | `theshop_plan_leveling.gif` |

The numbered zone waypoints and exact cell-boundary outlines are drawn directly
onto the main `--out` PNG when `--zones > 0`.

## Output

On success the tool writes the PNG and prints an **OptiHaul-style report**:
moved volume, pass count, the loading / hauling / dumping / returning distances
and achieved speeds, and the **time to level the field** (matching the
OptiSurface methodology — trapezoidal haul/return speeds, return distance =
forward distance, and a field-efficiency factor for "Other" time):

```
========================================================
OptiHaul Analysis Results
  Grid spacing (ft):      15 (60 x 62, 3120 areas)
  Field Area (ac):        31.9
  Move Vol. (BCY):        15,930
  Move Rate (BCY/hr):     91
  Area Rate (ac/hr):      0.182
  Ave. Move Dist. (yd):   157
  Haul heading (deg):     268.6
  Passes:                 3144
--------------------------------------------------------
Distances                       Speeds
  Loading (mi):     ...        Loading (mph):       4.0
  Hauling (mi):     ...        Ave. Hauling (mph):  ...
  Dumping (mi):     ...        Dumping (mph):       5.0
  Returning(mi):    ...        Ave. Return (mph):   ...
--------------------------------------------------------
Times (hr)
  Loading:    ...
  Hauling:    ...
  Dumping:    ...
  Returning:  ...
  Other:      ...   (field eff. 80%)
  TOTAL:      ...
========================================================
```

If `--zones > 0` it also prints the rough cut work order (zone, load count, BCY,
centroid lat/lon, and the pass range each zone was worked over).

## Pipeline (spec §5)

```
AGD file ─► [1] Parse + grid (7.5 ft)         agd.py, grid.py   §6, §7
         ─► [2] Volume balance (check/adjust)  balance.py        §8
         ─► [3] Flow field (optimal transport) flowfield.py      §9
         ─► [4] Strips (streamlines)           strips.py         §10
         ─► [5] Passes (15.4 BCY / 61 mm)      passes.py         §11
         ─► [6] Sequence + route               routing.py        §12
         ─► [7] Time estimate                  timemodel.py      §13
         ─► [8] Blade-scale guidance (opt.)    swaths.py         §13
         ─► PNG: cut/fill map + haul arrows    render.py         §14
```

The cut/fill PNG depends only on stages [1]–[3]; the pass list and time are
additive (`--no-plan` produces the PNG without them). The blade-scale guidance
track (stage [8]) is built on request via `--guidance-csv` / `--guidance-png`.

## Notes on the model

- **Sign convention.** On disk `CutFill = Proposed − Existing`; the reader
  ignores that column and recomputes `delta = Existing − Proposed` so that
  **positive delta = cut** (§6.1.11).
- **Volume units.** Bank volume (in ground) **is** BCY. Loose volume relates by
  `LCY = BCY × swell`, so the 20 LCY scraper holds `20 ÷ 1.3 ≈ 15.4 BCY` of
  bank soil per pass. Fill demand uses the cut/fill ratio (compaction): building
  one bank yard of fill consumes `cutfill_ratio` bank yards of cut.
- **Flow field.** The Poisson potential is solved so that φ is high at the cut
  ridges and low at the fill sinks; `F = −grad(φ)` therefore flows from cut to
  fill. The arrows in the PNG converge from the cut ring into the fill basin.
- **Passes.** The plan is a shortest-haul-first patch matcher: the cut cell
  whose nearest fill is closest anchors each pass, a compact **contiguous** cut
  patch is region-grown to a full scraper load (≈ 15.4 BCY) under the 61 mm
  layer cap, and the load is deposited into a compact contiguous fill patch.
  Keeping both patches contiguous keeps the loading/dumping sweeps near minimum;
  the haul is the true transit leg (cut-end → fill-start). Volume is conserved
  exactly. An optional **heading** (default = the design surface's main slope)
  biases fill selection to move soil down-slope, emulating OptiSurface's
  lane-oriented hauling.
- **Time model (OptiHaul-matched).** Loading/dumping run at constant speed;
  hauling/returning use a trapezoidal speed profile so acceleration/deceleration
  *reduce the average speed*. The return distance equals the
  loading + hauling + dumping distance, and a field-efficiency factor (80%) adds
  "Other" (non-productive) time.
- **Strips & multi-drop (drivability).** Each load's cut and fill are grown as a
  single long **blade-wide lane** along the heading (`--strip-bias`) instead of a
  compact blob, so the operator drives straight lines rather than weaving. When a
  pass would end part-full, **multi-drop** lets the scraper hop (blade up) to a
  nearby pocket within `--max-drop-hop-ft` to keep loading/spreading the same
  load, cutting the number of near-empty "crumb" passes.
- **Direction-aware sequencing.** The sequencer orders passes by a *time* cost
  that combines the empty deadhead drive with the **turn arc** needed to leave one
  pass and swing into the next cut (same 45 ft circle / turn-speed model as the
  guidance), so it minimizes turning, not just distance. With a grade/compact pass
  the machine ends at the fill's cut-side edge, so the grade return doubles as
  travel toward the next cut (`--no-direction-aware` reverts to distance + 2-opt).
- **Blade-scale guidance (drivable plan).** `--guidance-csv` / `--guidance-png`
  translate the optimized passes into turn-by-turn operator waypoints (action,
  blade up/down, target depth, next bearing, instruction) and a **realistic
  drivable time** that, unlike the OptiHaul-matched core model, counts **turning**
  (turn circle at turn speed, only for turns sharper than `--turn-threshold-deg`),
  a **grade/compact return pass** over every fill (`--grade-speed-mph`, disable
  with `--no-grade-return`), and **multi-drop repositioning** between pockets.

## Tests

```bash
python -m pytest tests/ -q
```

Tests cover AGD round-trip and `delta` sign, BCY conversions, the LCY→BCY
scraper conversion, mass balance, ridge→valley flow direction, pass limits
(≤ 15.4 BCY, ≤ 61 mm), the cut/fill palette and render geometry, and both
masking paths (with and without `2PER` boundary rows). See spec §17.

