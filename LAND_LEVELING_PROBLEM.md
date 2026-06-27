# Land Leveling Optimization: Problem Description

This document describes the **land leveling** problem we are solving: how to move soil from high areas (cuts) to low areas (fills) so the field reaches a target elevation surface, using a single piece of earthmoving equipment, while minimizing haul distance and total time.

---

## 1. The Problem

A field has an **existing** elevation surface (current ground) and a **target** elevation surface (desired final grade). The difference defines **cut** (soil to remove) and **fill** (soil to add). Land leveling is the process of:

1. **Cutting** soil from **areas** where existing > target
2. **Hauling** that soil
3. **Filling** (placing) it in **areas** where existing < target

We assume one load is cut, then hauled, then fully placed before the next cut. The goal is to choose a **sequence of cut–haul–fill operations** so that:

- Every **area** ends within a small **tolerance** of its target elevation (e.g. 1 cm).  
- **Total haul distance** is as short as possible.  
- **Total time** (haul + load/unload) is as short as possible.

We treat **shortest haul paths** as the primary objective and **shortest time** as a direct consequence (distance and volume drive the time model).

---

## 2. Observations of the OptiSurface Algorithm

OptiSurface approaches land leveling by first dividing the field into **equal-sized rectangles**. For each rectangle it then calculates:

- The **direction** soil must move **out of** the rectangle (cut) or **into** it (fill).
- The **number of passes** required for that rectangle to reach the target grade within tolerance.

These per-rectangle flow directions and pass counts define a coarse map of where soil is sourced, where it is deposited, and how much work each area needs. That map aligns with the concepts used in this document: area-based elevations, haul-direction vectors, and pass-based cut–haul–fill operations along flow-aligned strips.

The size of the rectangles is configurable, and isn't tied to the physical equipment size.

The algorithm has the following inputs:

Loading speed (MPH)
Haul speed (MPH)
Dumping speed (MPH)
Return speed (MPH)
Acceleration time (s)
Deceleration time (s)

As the rectangle size is increased, the calculations perform by OptiSurface require increasing corrections to "rebalance" the earthworks.

The algorithm works even if large rectangles are used, such as 500ft spacing on a field 1,500ft by 1,500ft in size.

---

## 3. Available OptiSurface Data

OptiSurface exports visual and spatial outputs that can be used to analyze how its algorithm interprets a field. We have:

- **Cut/fill maps** — showing where soil must be removed or placed and the relative cut/fill intensity across the field.
- **Haul arrows** — indicating the direction soil is expected to move between cut and fill regions.

These are provided on **variable-sized calculation grids**: the same field can be rendered at different rectangle spacings (see §2), so maps and arrows can be compared at coarse or fine resolution. That flexibility supports analysis of how grid size affects flow patterns, pass estimates, and overall earthworks balance without tying the data to a single fixed cell size.

---

## 4. Equipment Specifications

The model assumes one earthmoving machine (e.g. scraper or grader) with these constraints:


| Parameter              | Value                       | Meaning                                                                                 |
| ---------------------- | --------------------------- | --------------------------------------------------------------------------------------- |
| **Scraper capacity**   | 20 LCY                      | Maximum loose volume carried in one trip (in the scraper). Bank limit ≈ 20 ÷ 1.3 ≈ **15.4 BCY**. |
| **Swell factor**       | 1.3                         | **`LCY = BCY × 1.3`** (equivalently **`BCY = LCY ÷ 1.3`**). Excavated soil bulks up loose. |
| **Cut/fill ratio**     | 1.2                         | Bank cut soil that must be delivered per bank cubic yard of **fill space** created (compaction). |
| **Max depth per pass** | 61 mm                       | Maximum cut or fill depth in any one **area** for a single pass.                             |
| **Blade width**        | 2,271 mm                    | Width of the cutting/filling blade |
| **Blade height control** | 10 Hz                    | The blade moves up and down 10 times per second to control its height, never exceeding the maximum depth (61 mm). |

**Volume units.** **BCY** (banked cubic yards) is in-ground volume — the unit used for cut supply, fill demand, mass balance, and transported volume. **LCY** (loose cubic yards) is the volume in the scraper after excavation. Swell relates them but applies **only** to scraper capacity, not to how much cut or fill an area needs.

Volume relationships (let `M3_TO_CY = 1.30795`):

- **Cut supply** (area needs cutting): `delta × area → m³ → × M3_TO_CY → supply_BCY`. No swell factor — this is bank volume in the ground.
- **Fill demand** (area needs filling): `(-delta) × area → fill_space_m³ → × M3_TO_CY × cutfill_ratio → demand_BCY`. The cut/fill ratio accounts for compaction; swell does not appear here.
- **Scraper load limit**: `load_limit_BCY = scraper_LCY ÷ swell` (e.g. 20 LCY ÷ 1.3 ≈ 15.4 BCY bank soil per pass).

If a full scraper load cannot be cut (e.g. near final grade), smaller loads are allowed; efficiency drops but the operation remains feasible.

**Blade height behavior.** Because the blade is adjusted 10 times per second, it can follow varying cut or fill depths along the path. If one **area** requires a 61 mm cut but the next **area** along the path requires less (or no cut), the blade automatically raises as needed. The same applies on fill: the blade lowers only as much as each **area’s** required fill depth, and never exceeds 61 mm in a single **area**.

---

## 5. How the Equipment Moves: Strips

Movement is modeled in **passes**. Each pass is:

1. **Cut** – Travel along a **cut strip**, removing soil until the load is full (or the strip’s available cut is exhausted).
2. **Haul** – Travel from the cut strip to the fill strip (straight-line distance in the model).
3. **Fill** – Travel along a **fill strip**, placing soil until the load is empty (or the strip’s fill capacity is used).

**Strips** are the key geometric concept:

- **Shape**: Strips can follow the flow field, so they are **not** restricted to cardinal directions; they can curve.  
- The equipment **moves along** the strip (along the centerline), cutting or filling, and stops when the **accumulated volume** reaches the load limit (20 LCY ≈ 15.4 BCY bank) or the strip runs out of cut/fill.

So:

**Strips and variable blade height.** Because the blade height is constantly adjusted (see §4), a cut strip can **pass over** **areas** that are already at target height without cutting there, then **resume cutting** on the other side of that section. Similarly, a fill strip can pass over **areas** that need no fill and then resume placing soil. The path is continuous along the flow direction; only the blade depth changes. The goal is to maximize the load (e.g. approach 20 LCY / ≈ 15.4 BCY bank) when possible by accumulating cut or fill along the strip even when some **areas** in between are already at grade.

**Optimal haul route.** The most efficient pattern is to start at the **top of a ridge** (a local high in the existing surface) and move **downhill** in the direction given by the haul direction vectors. The equipment **cuts** along that path until it reaches the **transition** from cut **areas** to fill **areas** (where existing elevation crosses the target). There it switches to **filling**, continuing in the same direction and still following the haul direction vectors until the load is empty. This way the route follows the flow field, minimizes cross-haul distance, and uses gravity and the natural cut-to-fill transition along the flow. It also minimizes the accidental creation of dams that would increase flooding if rain were to fall during the levelling project.

---

## 6. Goals: Shortest Haul Paths, Then Shortest Time

**Primary objective:** Minimize **total haul distance** (sum over all passes of the distance from cut strip to fill strip). Shorter paths mean less travel and less cost.

**Secondary objective:** Minimize **total time**. Time is modeled as:

- **Travel time** = total distance ÷ travel speed (e.g. m/s).  
- **Load/unload time** = function of volume (e.g. per-BCY time).  
- **Total time** = sum over passes of travel time + load/unload time.

Because each pass moves at most one load and distance is proportional to travel time, **shortest total distance** strongly correlates with **shortest total time**. The aim is to achieve the smallest total distance (and hence typically the smallest total time) while bringing all **areas** within the chosen elevation tolerance.
