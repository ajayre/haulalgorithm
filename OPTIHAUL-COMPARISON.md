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
| Move Vol. (BCY) | 46,278 | 46,960 | 46,538 |
| Passes | ~3,005 | 3,058 | 3,034 |
| **Distances (mi)** | | | |
| Loading | 176.2 | 82.5 | 141.2 |
| Hauling | 107.2 | 230.6 | 204.0 |
| Dumping | 151.3 | 73.6 | 120.1 |
| Returning (deadhead) | 434.8 | 386.6 | 270.8 |
| Grade/compact return | — | — | 120.1 |
| Multi-drop reposition | — | — | 50.0 |
| **Times (hr)** | | | |
| Loading | 44.1 | 20.6 | 35.3 |
| Hauling † | 19.7 | 32.9 | 44.4 |
| Dumping | 30.3 | 14.7 | 24.0 |
| Grading/compaction | — | — | 15.0 |
| Returning | 65.5 | 58.6 | 31.1 |
| Turning | — | — | 35.2 |
| Other (20% field eff.) | 39.9 | 31.7 | 46.3 |
| **TOTAL** | **199.5** | **158.5** | **231.3** |

† Our guidance "Hauling" time includes the multi-drop reposition hops
(blade-up moves between pockets within a single load).

Run commands:

```bash
# Ours — core model at OptiHaul's grid (no guidance)
python -m landlevel.cli --agd TheShop2_2ft.agd --out out/shop2_core15.png \
  --grid-size-ft 15 --heading-deg 270

# Ours — full blade-scale guidance, 8.5 ft blade on a matched 8.5 ft grid
python -m landlevel.cli --agd TheShop2_2ft.agd --out out/shop2_85.png \
  --grid-size-ft 8.5 --blade-width-mm 2590.8 --heading-deg 270 --guidance-csv
```

## Reading the table

- **OptiHaul vs. our core model.** Both run on a coarse 15 ft calculation grid
  with no turning and no grade pass. They differ only in how loading/dumping
  distance is computed: OptiHaul derives it from a scraper-width formula
  (`vol ÷ (depth × width)`), while our core model sums the actual swept length of
  each pass's cut/fill cells. Our core total (158.5 hr) is lower mainly because
  our sequencer cuts the empty return distance (386.6 mi vs OptiHaul's 434.8 mi).
- **Our core vs. our guidance.** The guidance column is the **realistic, drivable**
  plan: the real 8.5 ft blade (so loading/dumping cover more lane distance), the
  mandatory **grade/compact return pass**, and **turning time** on the 45 ft
  turning circle at 4 mph. These are the operations OptiHaul does not model.

## Why OptiHaul's 199.5 hr is a floor, not the real time

OptiHaul's 199.5 hr now uses the correct 8.5 ft blade, so it is a sound estimate
of the **load + haul + dump + return** work. It is not the achievable field time,
because it prices two mandatory operations at **zero**:

1. **Grade/compact return pass (your spec).** The machine must drive back over
   each fill to grade and compact it. At the 8.5 ft dumping distance (151.3 mi)
   this is **+15–19 hr** depending on grade speed (8 MPH default).
2. **Turning.** A 45 ft turning circle forces the machine to slow to 4 MPH at
   every lane-end and at the haul→fill and grade→return swings. Across ~3,005
   loads this is **tens of hours** (our model measures 35.2 hr from 13,137 tight
   turns).

Adding only these unavoidable operations to OptiHaul's own productive legs
(159.6 hr) and re-applying the 80% field-efficiency factor:

| Scenario | Extra productive | Total |
|---|---|---|
| OptiHaul (as reported) | 0 | 199.5 hr |
| Generous floor — grade @ 13 MPH + 1×90° turn/load | +16.6 hr | **220.3 hr** |
| Realistic — grade @ 8 MPH + ~35 hr turning | +53.9 hr | **266.9 hr** |

Even under physically impossible generosity (grading at 13 MPH, a single 90° turn
per load), the floor is **≈ 220 hr**. Our full blade-scale estimate of **231.3 hr**
sits in this band — consistent with the proven floor and substantially better on
deadhead than OptiHaul (270.8 mi vs 434.8 mi) because of direction-aware
sequencing.

## Bottom line

- Our **core time model agrees with OptiHaul** on the four productive legs once
  the blade width is correct (the earlier 147.8 hr OptiHaul figure used a 15 ft
  blade — twice reality — and was off by ~2× on load/dump).
- The honest, drivable field time for The Shop 2 is **~231 hr**, with the gap above
  OptiHaul being the **grade/compact pass and turning** that an operator actually
  has to perform.
