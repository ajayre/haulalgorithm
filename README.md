# Land Leveling Haul Algorithm

A complete implementation of the **LAND-LEVELING-SPEC-2** pipeline for agricultural land
leveling — taking an AGD survey file and producing a minimum-haul
cut/fill plan, sequenced scraper passes, operator guidance, and time estimates.

Available in two implementations:

| Implementation | Entry point | Status |
|---|---|---|
| [**Python**](Python/) (`landlevel`) | `python -m landlevel.cli` | Reference implementation |
| [**C# / .NET**](DotNET/) (`HaulBuilder`) | `HaulBuilderEngine.Run(options)` | Production port, API-first |

---

## What it does

The pipeline reads a georeferenced field survey (existing elevation + proposed target grade)
and produces a complete earthmoving plan:

1. **Builds a grid** — divides the field into ≈ blade-width cells (default 8.5 ft) and
   computes the cut/fill volume at every cell.
2. **Checks / adjusts mass balance** — ensures total cut equals total fill, accounting for
   swell and compaction factors.
3. **Computes a transport flow field** — solves a Poisson equation over the grid so the
   field flows from cut ridges toward fill basins, establishing the natural haul direction.
4. **Traces streamlines** — derives curved, with-the-grain haul lanes from the flow field.
5. **Generates scraper passes** — plans each load (≈ 15.4 BCY) as a contiguous cut patch
   matched to a contiguous fill patch, using a shortest-haul-first KD-tree anchoring
   strategy with a 61 mm per-pass depth cap. Supports **flow-aligned curved lanes** (default)
   or straight lanes along a fixed heading. **Multi-drop** keeps loads full by hopping to
   nearby pockets without excessive U-turn moves.
6. **Sequences and routes passes** — orders passes with a direction-aware cost that
   combines deadhead distance with the turn arc needed to swing into the next cut, then
   optionally refines the order with a windowed relocation search.
7. **Estimates time** — constant speeds for loading and
   dumping, trapezoidal haul/return speed profiles, and an 80% field-efficiency factor
   for non-productive time.
8. **Produces blade-scale guidance** — emits turn-by-turn operator waypoints with depth,
   heading, and instructions, smoothed into drivable GPS curves. Counts turning arcs, a
   grade/compact return pass over every fill, and multi-drop hops for a realistic drivable
   time estimate.
9. **Handles exclusion zones** — drops bins inside no-go polygons (wells, structures) and
   routes blade-up travel around them via a visibility graph + Dijkstra shortest path.
10. **Schedules center-pivot irrigation** — finds arm bearings via dynamic programming
    that minimize arm moves against haul detours, then routes haul legs around the parked
    arm in the guidance track.

---

## Capabilities

| Feature | Python | C# |
|---|---|---|
| Flow-aligned curved haul lanes (Poisson) | ✓ | ✓ |
| Straight lanes along fixed heading (`--no-flow-aligned`) | ✓ | ✓ |
| Multi-drop pass loading | ✓ | ✓ |
| Compact load fragmenting (refunds far cells) | ✓ | ✓ |
| Direction-aware sequencing (turn-into-cut cost) | ✓ | ✓ |
| Windowed relocation order refinement | ✓ | ✓ |
| Knot re-profiling → boustrophedon sweeps | ✓ | ✓ |
| Route smoothing (drivable GPS curves) | ✓ | ✓ |
| Blade-scale turn-by-turn guidance CSV | ✓ | ✓ |
| Realistic guidance time (turn arcs, grade pass) | ✓ | ✓ |
| OptiHaul-matched time report | ✓ | ✓ |
| Exclusion zones (visibility graph + Dijkstra) | ✓ | ✓ |
| Center-pivot irrigation scheduling (DP) | ✓ | ✓ |
| Cut/fill PNG with haul-direction arrows | ✓ | ✓ |
| Zone work-order sequencing | ✓ | ✓ |
| Per-pass cut/fill pair CSV + heat-map PNG | ✓ | ✓ |
| Leveling animation (GIF, cutfill / topology) | ✓ | ✓ |
| Pivot leveling animation with arm moves | ✓ | ✓ |
| Mass-balance check / auto-adjust | ✓ | ✓ |
| CLI entry point | ✓ | — |
| In-memory API (no filesystem I/O) | — | ✓ |
| Native nanoflann KD-tree | — | ✓ |

---

## Performance

Representative timings on a **38-acre field** (177×173 grid, ~46 400 BCY moved), single
desktop core, Release build, CLI / `HaulBuilderOptions` defaults:

```bash
python DotNET/Bench/bench_theshop2.py
dotnet run -c Release --project DotNET/Bench/Bench.csproj -- DotNET/TheShop2_2ft.agd
```

| Stage | Python | C# |
|---|---|---|
| Parse + grid | 2.5 s | 0.03 s |
| Flow field (Poisson) | 1.0 s | 0.31 s |
| Strip tracing | 0.9 s | 0.04 s |
| Pass generation | 55.2 s | 17.2 s |
| Routing + knots | 11.7 s | 0.73 s |
| **Total** | **~72 s** | **~19 s** |

C# runs the full plan+route pipeline in roughly **one quarter** of the Python wall time
(~3.8× faster). Pass generation — the heaviest stage — is about **3.2× faster** in C#
thanks to the native nanoflann KD-tree; every other stage is also significantly faster.

On the same input, both implementations agree on the overall earthmoving plan:

| Metric | Python | C# |
|---|---|---|
| Passes | 3 049 | 3 047 |
| Moved BCY | 46 448 | 46 449 |
| Haul distance | 200 103 m | 199 568 m |
| Hours | 212.2 | 212.1 |

---

## Quick start

### Python

```bash
pip install -r Python/requirements.txt
python -m landlevel.cli --agd DotNET/TheShop2_2ft.agd --out plan.png
```

See [Python/README.md](Python/README.md) for the full CLI reference.

### C# / .NET

```bash
cd DotNET
dotnet build HaulBuilder.sln -c Release
```

```csharp
using HaulBuilder;

var options = new HaulBuilderOptions
{
    AgdPath       = "TheShop2_2ft.agd",
    OutputPngPath = "plan.png",
};
HaulBuilderResult result = new HaulBuilderEngine().Run(options);
```

See [DotNET/README.md](DotNET/README.md) for the full API reference.

---

## Repository layout

```
Python/             Reference implementation (landlevel package + CLI)
DotNET/             C# port (HaulBuilder class library + tests + bench)
Documents/          Specification and design documents
Patent Search/      Prior-art search materials
```

---

## Algorithm overview

The core algorithm is a **shortest-haul-first contiguous patch matcher** operating on an
optimal-transport flow field:

1. A **Poisson flow field** (`F = −∇φ`) transports soil from cut ridges to fill basins,
   following the natural slope of the design surface.
2. **Streamlines** are traced along the flow field to define curved, with-the-grain haul
   lanes.
3. Each **pass** is seeded at the unfinished cut cell whose nearest fill is closest (KD-tree
   query), then region-grown upstream along its streamline to fill a full scraper load
   (≈ 15.4 BCY, capped at 61 mm depth per cell). The corresponding fill is grown
   downstream into the matching basin.
4. **Multi-drop** extends partial loads by hopping blade-up to the nearest unfinished
   pocket within a configurable radius, capping load fullness to keep lanes from bloating
   into U-turns.
5. After all passes are generated, a **direction-aware sequencer** orders them by a combined
   cost of deadhead distance + turn-arc time, optionally refined by a windowed relocation
   local search.
6. The **blade-scale guidance** stage converts the ordered pass list into a drivable
   turn-by-turn operator track, smoothing grid-stepped coordinates into curves whose
   corner radius respects the tractor+scraper turning circle (45 ft diameter), and
   reprofiles any undriveable "knot" sweeps into boustrophedon back-and-forth lanes.

Volume is conserved exactly throughout: no soil is created or destroyed, and the finished
surface is identical regardless of whether flow-aligned or straight-lane mode is used, or
whether knots are reprofiled.

---

## Specification

The full algorithm specification is in [`Documents/LAND-LEVELING-SPEC-2.md`](Documents/LAND-LEVELING-SPEC-2.md).
