# The Shop 2 — OptiHaul vs. Our Algorithm

A side-by-side time comparison for **The Shop 2** between OptiSurface **OptiHaul**
and this algorithm, run at the same equipment and field-efficiency inputs. The
purpose is to (a) confirm our core time model agrees with OptiHaul when given the
same assumptions, and (b) show what changes once the plan is made **blade-scale
and drivable** (real blade width, turning, and the mandatory grade/compact pass).

## Equipment note — tandem scrapers as one 8.5 ft blade

The machine pulls **two scrapers in tandem**, filled sequentially:

| Scraper | Blade width | Capacity | Share of a load |
|---|---|---|---|
| Front | 7 ft | 8 LCY | 40% |
| Rear | 10 ft | 12 LCY | 60% |
| **Combined** | **8.5 ft effective** | **20 LCY (≈ 15.4 BCY)** | 100% |

Because loading/dumping distance is `volume ÷ (depth × width)`, the correct
single-blade equivalent is the **volume-weighted harmonic mean** of the two
widths:

```
1 / W_eff = 0.40/7 + 0.60/10   ⇒   W_eff = 8.54 ft
```

That is within **0.4%** of a flat 8.5 ft average, so treating the tandem as a
single **8.5 ft** scraper is accurate for the load/dump geometry. All runs below
use 8.5 ft.

## Comparison (8.5 ft blade)

| Metric | OptiHaul (8.5 ft) | Ours — core (15 ft grid) | Ours — +guidance (8.5 ft) |
|---|---|---|---|
| Field Area (ac) | 36.7 | 37.4 | 37.1 |
| Move Vol. (BCY) | 46,278 | 46,953 | 46,536 |
| Passes | ~3,005 | 3,071 | 3,051 |
| **Distances (mi)** | | | |
| Loading | 176.2 | 86.5 | 139.9 |
| Hauling | 107.2 | 185.5 | 136.5 |
| Dumping | 151.3 | 71.1 | 122.6 |
| Returning (deadhead) | 434.8 | 343.1 | 251.3 |
| Grade/compact return | — | — | 122.6 |
| Multi-drop reposition | — | — | 34.1 |
| **Times (hr)** | | | |
| Loading | 44.1 | 21.6 | 35.0 |
| Hauling † | 19.7 | 28.2 | 35.4 |
| Dumping | 30.3 | 14.2 | 24.5 |
| Grading/compaction | — | — | 15.3 |
| Returning | 65.5 | 54.8 | 29.7 |
| Turning | — | — | 24.0 |
| Other (20% field eff.) | 39.9 | 29.7 | 41.1 |
| **TOTAL** | **199.5** | **148.7** | **205.0** |

† Our guidance "Hauling" time includes the multi-drop reposition hops
(blade-up moves between pockets within a single load); its distance is broken
out on the separate "Multi-drop reposition" row.

Run commands:

```bash
# Ours — core model at OptiHaul's grid (no guidance)
python -m landlevel.cli --agd TheShop2_2ft.agd --out out/shop2_core15.png \
  --grid-size-ft 15 --heading-deg 270

# Ours — full blade-scale guidance, 8.5 ft blade on a matched 8.5 ft grid
python -m landlevel.cli --agd TheShop2_2ft.agd --out out/shop2_85.png \
  --grid-size-ft 8.5 --blade-width-mm 2590.8 --heading-deg 270 --guidance-csv
```

(The default passes are flow-aligned; `--heading-deg 270` only sets the fallback
heading where the flow is flat. An optional `--reorder-window` refinement of the
pass order is available — see the **Optional order refinement** section below.)

## Reading the table

- **OptiHaul vs. our core model.** Both run on a coarse 15 ft calculation grid
  with no turning and no grade pass. They differ in how loading/dumping distance
  is computed: OptiHaul derives it from a scraper-**width** formula
  (`vol ÷ (depth × width)`), while our core model sums the swept length of each
  pass's cut/fill **cells** (a function of grid size, not blade). At an 8.5 ft
  blade on a 15 ft grid this makes our core read lower — see
  [Why we matched OptiHaul at a 15 ft blade but now read lower](#why-we-matched-optihaul-at-a-15-ft-blade-but-now-read-lower-same-grid)
  below for the full explanation and the apples-to-apples fix.
- **Our core vs. our guidance.** The guidance column is the **realistic, drivable**
  plan: the real 8.5 ft blade (so loading/dumping cover more lane distance), the
  mandatory **grade/compact return pass**, and **turning time** on the 45 ft
  turning circle at 4 mph. These are the operations OptiHaul does not model.

## Why we matched OptiHaul at a 15 ft blade but now read lower (same grid)

A natural question: when OptiHaul used a 15 ft blade our core model matched it,
but now that OptiHaul uses a more accurate 8.5 ft blade (still on a 15 ft grid)
our core reads **lower** — even though the grid size is unchanged. The reason is a
**modeling distinction in what each model ties the loading/dumping distance to**:

- **OptiHaul ties load/dump distance to the BLADE width:**
  `distance = volume ÷ (depth × blade_width)`.
- **Our core model ties load/dump distance to the GRID cell size:** it sums the
  swept length of the cut/fill *cells*, which depends on grid resolution and
  **not on blade width at all**.

Everything else is the same formula — both compute
`returning = loading + hauling + dumping` (OptiHaul's 434.8 mi = 176.2 + 107.2 +
151.3; our core's 343.1 = 86.5 + 185.5 + 71.1).

The two load/dump distances only agree when **blade width ≈ grid size**:

| Config | OptiHaul (blade-based) | Our core, 15 ft grid (blade-blind) | Result |
|---|---|---|---|
| 15 ft blade | load 99.9 / dump 85.8 mi | load 86.5 / dump 71.1 mi | close → totals matched (147.8 vs 148.7) |
| 8.5 ft blade | load 176.2 / dump 151.3 mi | load 86.5 / dump 71.1 mi | diverge (199.5 vs 148.7) |

When OptiHaul ran a **15 ft blade on a 15 ft grid**, its blade-based sweep length
roughly equaled our grid-based sweep length, so the totals lined up. Switching to
an **8.5 ft blade** (keeping the 15 ft grid) scaled OptiHaul's load/dump distance
up by `15 ÷ 8.5 = 1.76×` (a narrower blade drives farther to cover the same area).
**Our core didn't move**, because it never reads the blade — it is still measuring
sweeps on the 15 ft grid. Same grid; one model responds to the blade, the other
does not.

**Honest caveat — this is a coarse-grid artifact, not a real efficiency gain.** A
15 ft grid under-resolves the true sweep length of an 8.5 ft blade. Our core also
scales with resolution: run it on an **8.5 ft grid** (≈ the 8.5 ft blade) and our
planned loading sweep is **177.1 mi**, essentially matching OptiHaul's blade-based
176.2 mi. In our model the **grid** plays the role the **blade** plays in OptiHaul,
so to compare apples-to-apples the grid should be set near the blade width. The
genuinely blade-accurate figure is our **8.5 ft guidance run (205.0 hr)**. Where we
are *legitimately* better than OptiHaul is the **deadhead** — direction-aware
sequencing cuts the empty return to 251.3 mi vs OptiHaul's 434.8 mi — not the
load/dump distance.

## Why OptiHaul's 199.5 hr is a floor, not the real time

OptiHaul's 199.5 hr now uses the correct 8.5 ft blade, so it is a sound estimate
of the **load + haul + dump + return** work. It is not the achievable field time,
because it prices two mandatory operations at **zero**:

1. **Grade/compact return pass (your spec).** The machine must drive back over
   each fill to grade and compact it. At the 8.5 ft dumping distance (122.6 mi)
   our guidance measures this at **15.3 hr** (8 MPH default grade speed).
2. **Turning.** A 45 ft turning circle forces the machine to slow to 4 MPH at
   every lane-end and at the haul→fill and grade→return swings. Across ~3,000
   loads our guidance measures **24.0 hr from 8,874 tight turns** (≥ 60°).

Adding only these two unavoidable operations — at the *measured* rates of our own
(already deadhead-optimized) plan — to OptiHaul's own productive legs and
re-applying the 80% field-efficiency factor gives a conservative floor for
OptiHaul's real field time:

| Scenario | Productive (hr) | Total (hr) |
|---|---|---|
| OptiHaul as reported (no grade, no turning) | 159.6 | 199.5 |
| OptiHaul + grade (15.3) + turning (24.0) | 198.9 | **≈ 248.6** |
| Ours — full drivable (grade + turning included) | 163.9 | **205.0** |

The floor of **≈ 249 hr** is conservative: OptiHaul's own route has nearly **twice
our deadhead** (434.8 mi vs 251.3 mi) and correspondingly more turning, so its true
turning time would *exceed* the 24.0 hr we borrowed from our tighter plan. The key
result is that our **full, drivable** estimate (**205.0 hr**) — which already prices
the grade pass and every turn — comes in only **~5.5 hr above** OptiHaul's
turning-free, grade-free **199.5 hr**, and well **below** OptiHaul's realistic floor,
because direction-aware sequencing nearly halves the empty return travel.

## Optional order refinement (`--reorder-window`)

The optional windowed relocation refinement (`--reorder-window N`, §12.2;
disabled by default) was run on this field at `N = 50`. It performed ~1,380 pass
relocations and lowered the route's **order-dependent transition-time surrogate**
(return drive + turn-into-cut) from **26.38 h to 26.27 h**. On a field this dense
(~3,000 passes) the realized blade-scale drivable **TOTAL is unchanged to within
±0.1 hr** (205.0 → 205.1 hr): the direction-aware nearest-neighbour route is
already near-optimal here, the deadhead distance is unmoved at 251.3 mi, and the
small surrogate turn-angle reduction does not lower the count of
Ramer–Douglas–Peucker-measured tight turns. The refinement is provably monotone
(it never increases its objective) and yields larger *relative* gains on smaller
or sparser fields, where the constructive route has more slack; it is therefore
left **off by default** and is not used in the comparison figures above.

## Bottom line

- Our **core time model agrees with OptiHaul** on the four productive legs once
  the blade width is correct (the earlier 147.8 hr OptiHaul figure used a 15 ft
  blade — twice reality — and was off by ~2× on load/dump).
- The honest, drivable field time for The Shop 2 is **~205 hr** — only slightly
  above OptiHaul's grade- and turn-free 199.5 hr, and far below OptiHaul's
  realistic floor (~249 hr), because direction-aware sequencing cuts the deadhead
  from 434.8 mi to 251.3 mi. The remaining gap above OptiHaul's optimistic number
  is the **grade/compact pass and turning** an operator must actually perform.
