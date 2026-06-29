# METHOD AND SYSTEM FOR GENERATING A TIME-OPTIMIZED, DRIVABLE EARTHMOVING HAUL PLAN FROM A DIGITAL TERRAIN MODEL

*The following document is drafted in the form and register of a United States*
*non-provisional utility patent application. It is an engineering description of*
*the land-leveling haul algorithm implemented in this repository and is provided*
*for technical and illustrative purposes; it is **not** legal advice and has not*
*been filed with any patent office. Bracketed numerals (e.g. **[0001]**) are*
*specification paragraph numbers in the conventional patent style.*

---

## CROSS-REFERENCE TO RELATED APPLICATIONS

**[0001]** This application claims no priority and is submitted as an original
disclosure. The subject matter herein corresponds to the reference
implementation embodied in the accompanying software package (the "landlevel"
Python package) and to the normative specification entitled *Land Leveling Haul
Algorithm Specification — v2*, the contents of which describe a particular
embodiment of the invention.

## FIELD OF THE INVENTION

**[0002]** The present invention relates generally to automated earthmoving and
agricultural land leveling, and more particularly to a computer-implemented
method and system for converting a digital terrain model of a field into a
spatially compact, drivable, and time-optimized sequence of cut-haul-fill
operations for execution by a grading machine such as a tractor-drawn scraper or
land plane, together with turn-by-turn operator guidance suitable for a
satellite-positioning ("GPS") in-cab display.

## BACKGROUND OF THE INVENTION

**[0003]** Land leveling is the process of reshaping a field's surface to a
designed target grade — commonly a plane or a smoothly varying surface — so that
irrigation water flows uniformly, drainage is controlled, and machine operations
are efficient. The work is performed by excavating soil where the existing
surface is above the target ("cut") and depositing that soil where the existing
surface is below the target ("fill"). The soil so moved is hauled by the grading
machine from cut locations to fill locations.

**[0004]** The dominant cost of a land-leveling project is the time the machine
spends moving soil. That time is governed by (i) the total volume of soil moved,
(ii) the distance each load is hauled, (iii) the distance swept while loading
(cutting) and dumping (spreading), (iv) the non-productive "deadhead" travel
between successive loads, and (v) the time lost decelerating, turning, and
re-accelerating the machine. Prior commercial tools, such as terrain-design and
haul-estimation software, compute a target surface and an aggregate haul estimate
but typically do not produce a fully sequenced, drivable, turn-by-turn route that
accounts for the physical width of the cutting blade, for the machine's turning
radius, and for the asymmetry of turning costs.

**[0005]** Several technical difficulties have impeded the generation of such a
route. First, an optimal transport of soil from cut to fill is a large-scale
assignment problem, and naive solutions produce tangled, undrivable paths.
Second, the planning grid used to balance volume is typically coarser than the
physical cutting blade, so a plan expressed at grid resolution under-counts the
sweeping a narrow blade must actually perform to cover a wider cell. Third, the
cost of proceeding from one load to the next is not merely a distance; it depends
on the direction the machine is travelling when it finishes one load and the
direction it must face to begin the next, because reorienting a long
tractor-and-scraper combination requires a slow, wide turning arc. This
directional cost is **asymmetric**: reversing the order in which a contiguous run
of loads is performed flips the machine's heading at every interior junction, so
a route-improvement technique that reverses sub-sequences — such as the
well-known 2-opt heuristic for the symmetric travelling-salesman problem — can
silently re-introduce the very turns it was meant to remove.

**[0006]** Accordingly, there is a need for a computer-implemented method that
transforms a digital terrain model into a drivable, time-optimized haul plan
that (a) conserves volume and respects per-pass load and depth limits, (b)
realizes the plan at the physical scale of the cutting blade, (c) sequences the
loads against a directional, turn-aware time cost, and (d) refines that sequence
with a local-search improvement step that is provably safe under the asymmetric
turn cost. The present invention addresses these needs.

## SUMMARY OF THE INVENTION

**[0007]** The invention provides, in one aspect, a computer-implemented method
for generating an earthmoving haul plan, the method comprising: receiving a
digital terrain model defining, over a field, an existing surface elevation and a
target surface elevation; binning the field into a grid of areas and computing,
for each area, a signed volumetric quantity representing a cut or a fill; solving
a global flow field that defines an optimal transport direction of soil from cut
regions to fill regions; generating a plurality of passes, each pass representing
one cut-haul-fill cycle in which a contiguous patch of cut areas is excavated to
a bounded per-pass load and deposited into a contiguous patch of fill areas;
ordering the plurality of passes into a route that minimizes a non-productive
travel cost; and outputting the route.

**[0008]** In another aspect, the ordering comprises a directional, turn-aware
transition cost between a first pass and a second pass that combines (i) a
deadhead drive time from where the machine finishes the first pass to where it
begins the cut of the second pass with (ii) a turning time proportional to the
heading change required to leave the first pass, traverse the deadhead leg, and
swing into the cut of the second pass, the turning time being computed from a
machine turning radius and a turning speed.

**[0009]** In a further aspect, and constituting a particular advance of the
invention, the method comprises a **windowed relocation refinement** of the
route. Because the directional transition cost is asymmetric, sub-sequence
reversal cannot be safely used to improve the route. Instead, the method walks
the ordered passes one at a time and, for each subject pass, evaluates relocating
that subject pass — by lifting it from its current position and re-inserting it at
a different position within a bounded window of nearby positions — and applies the
relocation that most reduces a total order-dependent time, repeating until no
improving relocation is found. Because a relocation moves a single pass without
reversing any contiguous run of passes, it preserves the internal heading of
every pass and is therefore sound under the asymmetric turn cost, supplying an
improvement step that sub-sequence reversal cannot. Each accepted relocation
strictly decreases the total order-dependent time, so the refinement converges
and never increases the time it seeks to reduce.

**[0010]** In yet a further aspect, the method realizes each pass at blade scale by
tiling its cut patch and its fill patch into a series of blade-width lanes driven
in a serpentine order, optionally following the local transport direction of the
flow field so that the lanes curve with the grain of the field; emits an ordered
list of waypoints, each waypoint carrying a position, a blade state, a target
depth, a bearing to a next waypoint, and a human-readable instruction; smooths a
displayed steering line for the waypoints to a curvature drivable within the
turning radius while charging all distance and time on the unsmoothed coverage
geometry; appends a grade-and-compact return pass over each fill; and estimates a
drivable total time that counts loading, hauling, dumping, grading, returning,
and turning, divided by a field-efficiency factor.

**[0011]** Further aspects provide a system comprising one or more processors and
memory storing instructions that, when executed, perform the foregoing method,
and a non-transitory computer-readable medium storing such instructions. The
foregoing summary is illustrative and is not intended to limit the scope of the
claims.

## BRIEF DESCRIPTION OF THE DRAWINGS

**[0012]** FIG. 1 is a block diagram of the multi-stage processing pipeline
according to an embodiment, proceeding from a digital terrain model to a rendered
plan, a sequenced route, and a blade-scale guidance track.

**[0013]** FIG. 2 depicts a cut/fill map of an example field overlaid with
haul-direction arrows derived from the global flow field, in which cut regions
and fill regions are distinguished by color and arrows converge from cut ridges
into fill basins.

**[0014]** FIG. 3 depicts a blade-scale guidance track for the example field, in
which each pass's cut patch and fill patch are tiled into blade-width serpentine
lanes that follow the local transport direction, the lanes being distinguished by
action type. A corresponding rendered image is provided in the accompanying file
`out/theshop2_plan_15ft_guidance.png`.

**[0015]** FIG. 4 depicts a cut-haul-fill "pairs" heat map for the example field,
in which each of a plurality of loads is drawn as a semi-transparent line segment
from a cut location to a corresponding fill location, the superposition of the
segments revealing the dominant haul corridors. A corresponding rendered image is
provided in the accompanying file `out/theshop2_plan_15ft_cut_fill_pairs.png`,
and a tabular export of the same pairs in route order is provided in
`out/theshop2_plan_15ft_cut_fill_pairs.csv`.

**[0016]** FIG. 5 is a flowchart of the windowed relocation refinement of the
route, illustrating the one-at-a-time traversal of passes, the evaluation of
candidate relocations within a bounded window, the acceptance of a strictly
improving relocation, and the convergence test.

**[0017]** FIG. 6 is a sequence of frames of a leveling animation showing the
field's surface progressively flattening to the target grade as the passes are
applied in route order. A corresponding animated image is provided in the
accompanying file `out/theshop2_plan_15ft_leveling.gif`.

## DETAILED DESCRIPTION OF THE INVENTION

**[0018]** Reference will now be made in detail to embodiments of the invention,
examples of which are illustrated in the accompanying drawings and reference
implementation. The following description sets forth numerous specific details,
such as particular grid spacings, scraper capacities, and speeds, in order to
provide a thorough understanding of the invention. It will be apparent to one of
ordinary skill in the art that the invention may be practiced without these
specific details, and that the recited numerical values are illustrative defaults
of one embodiment and not limitations of the invention.

### A. Overview of the pipeline

**[0019]** With reference to FIG. 1, an embodiment comprises a sequence of
processing stages performed by one or more processors: (1) parsing a digital
terrain model and binning it to a grid; (2) checking or adjusting volume balance;
(3) computing a global flow field; (4) tracing streamlines of the flow field; (5)
generating passes; (6) sequencing the passes into a route, including the
directional transition cost and the windowed relocation refinement; (7)
estimating an aggregate time; and (8) building a blade-scale operator guidance
track and an estimate of drivable time. The stages are described in turn. The
several stages may be embodied as separate, independently invocable software
modules, and certain stages (for example, the rendered map) may be produced
without performing later stages.

### B. Terrain model, grid, and volumes

**[0020]** A digital terrain model is received that defines, over the field, an
existing surface elevation and a target ("proposed") surface elevation. In one
embodiment the model is provided as a grid-design ("AGD") file enumerating
points, each point having a position and the two elevations, optionally with
boundary rows delimiting the worked area. A signed difference per location is
computed such that a positive value denotes material to be removed (cut) and a
negative value denotes material to be added (fill).

**[0021]** The field is binned into a grid of square areas, also termed cells or
bins, having an edge length that in one embodiment is approximately equal to one
blade width (for example, 7.5 feet), so that each load corresponds to
approximately one blade-wide lane; a coarser grid (for example, 15 or 30 feet)
may be used for faster, lower-resolution planning. For each area, the signed
elevation difference is converted to a volumetric quantity expressed in bank
cubic measure. Conversions between bank volume, loose volume, and compacted fill
volume are applied using a swell factor (loose volume equals bank volume times
swell) and a cut/fill ratio (the bank cut volume required to build one bank unit
of fill), so that the volume bookkeeping is physically consistent.

**[0022]** A volume balance stage verifies that total cut supply approximately
equals total fill demand, optionally adjusting a global vertical offset of the
target surface to bring the field into balance. Material conservation is enforced
throughout the subsequent stages.

### C. Global flow field and streamlines

**[0023]** A global flow field is computed that assigns to each area a transport
direction along which soil should move from cut regions to fill regions. In one
embodiment a scalar potential is obtained by solving a Poisson equation with the
signed volumetric quantity as a source term, such that the potential is high over
cut ridges and low over fill basins, and the flow field is taken as the negative
gradient of the potential, whereby flow proceeds from cut to fill. In an
alternative embodiment the flow is obtained from a minimum-cost-flow formulation
under a different distance metric. Streamlines of the flow field may be traced for
visualization and to guide the growth of passes along the grain of the field.

### D. Pass generation

**[0024]** A plurality of passes is generated, each pass representing one
cut-haul-fill cycle. Each pass is constrained by a maximum per-pass load (for
example, a scraper capacity expressed in loose measure and converted to bank
measure, on the order of fifteen bank cubic yards) and by a maximum cut or fill
depth per area per pass (for example, sixty-one millimeters), so that deep cuts
and fills are worked over several passes in successive layers, shallower layers
first.

**[0025]** In one embodiment a pass is formed by selecting an anchor cut area
whose nearest fill is closest, region-growing a contiguous patch of cut areas
from the anchor until the per-pass load is reached under the depth cap, and
depositing the load into a contiguous patch of fill areas grown from a nearby fill
seed. Keeping each patch contiguous keeps the loading and dumping sweeps near
their minimum length. The haul leg of the pass is the transit from the end of the
cut sweep to the start of the fill sweep, and is recorded as the optimized
transport distance to be preserved by later stages.

**[0026]** In a flow-aligned embodiment, the cut patch is grown upstream along the
local flow streamline toward the ridge from which it draws, and the fill patch is
grown downstream along the streamline into the basin it feeds, so that the cut,
the haul, and the fill together form a long, curved, with-the-grain lane. In an
alternative single-heading embodiment, each patch is grown as a straight
blade-wide lane along a global heading derived from the design surface's main
slope. A multi-drop capability permits a load that would otherwise end partially
full to continue, with the blade raised, to a nearby pocket within a bounded hop
distance, so as to fill the load while keeping the lane continuous; a
load-fullness cap halts such hopping once the load is sufficiently full, to avoid
degenerate doubling-back. A compactness guard discards from a given load any
fragment not connected to the load's seed within one hop and refunds the
corresponding material to be worked by a later, locally anchored pass, thereby
preventing long blade-raised moves that cover no ground.

### E. Sequencing and the directional transition cost

**[0027]** The plurality of passes is ordered into a route so as to minimize total
non-productive travel. With reference to the directional transition cost, the cost
of proceeding from a first pass `a` to a second pass `b` is computed, in a
direction-aware embodiment, as a time comprising: a deadhead drive time equal to
the distance from an exit point of the first pass to the cut-start point of the
second pass divided by a return speed; plus a turning time equal to a turning
radius multiplied by a total heading change and divided by a turning speed, the
total heading change being the sum of (i) the change from the heading at which the
machine finishes the first pass to the heading of the deadhead leg and (ii) the
change from the heading of the deadhead leg to the cut heading of the second pass.
Where a grade-and-compact return pass is enabled, the exit point of a pass is the
cut-side edge of its fill, so that the grade return doubles as travel toward the
next cut.

**[0028]** An initial route is constructed by a nearest-neighbor heuristic that
begins at a determined pass and repeatedly appends the unvisited pass minimizing
the transition cost from the current pass. In a symmetric embodiment in which the
transition cost is reduced to plain deadhead distance (the direction-aware turn
penalty being disabled), a bounded sub-sequence-reversal (2-opt) improvement may
be applied. In the direction-aware embodiment, however, sub-sequence reversal is
not applied, because reversing a contiguous run of passes flips the machine's
heading at every interior junction and the asymmetric turn cost would be
mis-estimated, potentially re-introducing turns.

### F. Windowed relocation refinement (a particular advance)

**[0029]** To improve the direction-aware route without the unsafe reversal
operation, the method performs a windowed relocation refinement, illustrated in
FIG. 5. There is defined a total order-dependent cost equal to the sum, over each
consecutive pair of passes in the route, of the transition cost of paragraph
[0027]. Because each pass's own loading, hauling, dumping, and grade sweeps are
fixed by the geometry of that pass and do not depend on the order of the route,
this sum is precisely the portion of the total time that re-ordering can alter;
reducing it reduces the overall time.

**[0030]** The refinement employs a single move operator, termed a relocation or
"or-opt" move, that lifts exactly one subject pass out of the route and re-inserts
it at a different position, without reversing any contiguous run of passes.
Because no run is reversed, the internal cut, haul, and fill headings of every
pass are preserved, and the relocation is therefore sound under the asymmetric
transition cost of paragraph [0027]. The change in the total order-dependent cost
caused by a candidate relocation is computed in constant time from the few
transitions adjacent to the source position and the destination position, the
remaining transitions being unaffected.

**[0031]** The refinement traverses the passes one at a time. For a subject pass
at a current position, the method considers a bounded window of candidate
destination positions lying within a configurable number `N` of positions of the
current position, evaluates the constant-time cost change for each candidate
relocation in the window, and, if any candidate strictly reduces the total
order-dependent cost beyond a small threshold, applies the candidate that reduces
it most. The traversal is repeated in successive sweeps until a sweep applies no
relocation or until a bounded number of sweeps has been performed. The
configurable number `N` (the "window") governs how far a pass may migrate; in one
embodiment `N` defaults to a value that disables the refinement and may be set to a
representative value such as fifty to enable it.

**[0032]** Because each accepted relocation strictly decreases the total
order-dependent cost and no relocation ever increases it, the sequence of accepted
moves is monotone and the refinement converges; moreover, the refinement can only
reduce, and never increase, the time it is designed to minimize. Upon completion,
the route is updated in place — the ordered passes, the original indices of those
passes, and the accumulated deadhead distance — without adding, removing,
splitting, or otherwise altering any pass, cut or fill area, load, or haul leg, so
that the moved volume and the finished surface are unchanged. In one embodiment
the refinement is performed after the route construction and after any
in-patch knot re-profiling (paragraph [0037]) and before the time estimate, the
optional zone summary, the pair exports, and the blade-scale guidance, so that all
downstream outputs reflect the refined order.

**[0033]** It will be appreciated that, while a particular bidirectional bounded
window is described, the window may equivalently be expressed as a forward
look-ahead of the next `N` passes, that the move operator may be applied to
relocate either the subject pass forward or a later pass to the subject position,
and that the traversal order and the sweep cap may be varied, all without
departing from the principle that a reversal-free relocation move provides a
convergent, monotone improvement that is sound under an asymmetric, direction-
dependent transition cost.

### G. Aggregate time model

**[0034]** An aggregate time may be estimated according to a methodology in which
each pass comprises loading, hauling, dumping, and returning motions. Loading and
dumping proceed at respective constant speeds; hauling and returning use a
trapezoidal speed profile in which the machine accelerates over an acceleration
time and decelerates over a deceleration time, thereby reducing the average speed
below a configured maximum. The returning distance is taken equal to the
loading-plus-hauling-plus-dumping distance, and a field-efficiency factor accounts
for non-productive time. This aggregate model is suitable for comparison with
conventional haul estimates.

### H. Blade-scale operator guidance

**[0035]** With reference to FIG. 3, the method realizes the route at the physical
scale of the cutting blade. For each pass, the cut patch and the fill patch are
each tiled into a plurality of lanes spaced approximately one blade width apart
across the patch and traversed in a serpentine order so that lane-to-lane turns
are short, the lanes running along the local flow direction in a flow-aligned
embodiment or along a global heading otherwise. The cut sweep is oriented to end
near the start of the haul leg and the fill sweep to start there, so that the
optimized haul leg of paragraph [0025] is preserved.

**[0036]** The method emits an ordered list of waypoints, each waypoint comprising
a geographic position, a blade state (raised or lowered), a target cut or fill
depth, a bearing to the next waypoint, and a human-readable instruction (for
example, to start a cut, to raise the blade and haul to the fill, to spread, to
turn back over the fill to grade and compact, or to reposition to a next pocket).
A displayed steering line is optionally smoothed so that its curvature is drivable
within the machine turning radius; importantly, the smoothing affects only the
displayed line, while all distance, turning, and time are charged on the
unsmoothed coverage geometry, because the narrow blade must still cover the full
width of each cell, the coverage length being bounded below by the load volume
divided by the product of the maximum depth and the blade width. A smoothed,
shortened sweep would otherwise silently under-cut the field.

**[0037]** The method further detects sweeps that, as rendered, form an undrivable
"knot" exhibiting repeated hairpin reversals within a small area, and re-profiles
such sweeps into a boustrophedon of parallel blade-width back-and-forth lanes
joined by single headland turns, adopting the re-profiled sweep only where it
strictly reduces the count of hairpin reversals, so that genuinely flowing sweeps
are never made worse. The re-profiling re-orders the cells of a patch only; it
does not change the load, the haul, the cut and fill quantities, or the finished
surface.

**[0038]** A drivable total time is estimated from the guidance track that, unlike
the aggregate model of paragraph [0034], additionally counts turning time — the
machine slowing to a turn speed and traversing a turning arc for each heading
change exceeding a threshold, gentler bends being driven in stride — and a
grade-and-compact return pass driven over each fill at a grade speed. A
field-efficiency factor is applied to obtain a total.

### I. Outputs and exports

**[0039]** The method produces one or more outputs, which may include: a rendered
map of the field colored by cut and fill and overlaid with haul-direction arrows
(FIG. 2); a tabular export of the cut-haul-fill pairs in route order, each row
giving the load, the haul distance, and the geographic start and end of the cut
segment and of the fill segment (FIG. 4 and the accompanying pairs file); a heat
map in which each pair is drawn as a semi-transparent segment (FIG. 4); a
turn-by-turn guidance track suitable for an in-cab positioning display, exported
as a table of waypoints; a rendered overlay of the blade-scale lanes (FIG. 3); a
zone-level rough work order summarizing the route into a small number of spatial
clusters; and a frame-by-frame animation of the surface flattening to grade (FIG.
6). The outputs are deterministic given identical inputs and parameters.

### J. Worked example

**[0040]** In a worked example on an approximately thirty-acre field at a fifteen-
foot planning grid, the method generated on the order of three thousand passes
conserving the field's cut and fill volumes. Enabling the windowed relocation
refinement with a window of fifty performed on the order of one thousand five
hundred relocations and reduced the total order-dependent transition time of the
route, with a corresponding reduction in the estimated drivable total, while
leaving the moved volume and the finished surface unchanged. Increasing the window
yielded further but diminishing reductions, and in no case increased the time,
consistent with the monotone convergence of paragraph [0032].

### K. Algorithmic embodiment (illustrative pseudocode and formulas)

**[0041]** This section sets forth, for enablement, the specific algorithms of one
embodiment in pseudocode and closed form. The notation is illustrative; symbols
denote scalars unless noted. The pseudocode is expressed at the level of ordinary
skill in numerical and combinatorial programming, and the recited constants are
the defaults of one embodiment.

**[0042]** *Grid binning and per-area volume.* For an area of side `s` (metres),
let `A = s²` be its plan area and let `δ` (metres) be the signed
existing-minus-target elevation, positive for cut. The per-area bank volume in
cubic yards is computed as:

```text
m3_to_cy = 1 / 0.764554857984           # cubic metres -> cubic yards
cut_bcy(area)  =  δ · A · m3_to_cy                       for δ > 0
fill_bcy(area) = -δ · A · m3_to_cy · cutfill_ratio       for δ < 0   (compaction)
```

A per-pass bank load limit `L_max` is obtained from the scraper's loose capacity
`C_lcy` and the swell factor `σ` as `L_max = C_lcy / σ`. Cut and fill depths
applied in any single pass are clamped to `d_max` (e.g. 0.061 m).

**[0043]** *Flow field.* A scalar potential `φ` is obtained on the valid grid by
solving the Poisson problem `∇²φ = ρ`, where the source `ρ` is the signed volume
density (positive at cut, negative at fill), with homogeneous Neumann boundary
conditions on the field mask. The transport field is the negative gradient
`F = −∇φ`, discretized by central differences:

```text
solve  ∇²φ = ρ          (sparse direct solve, or Jacobi/CG fallback)
Fx(i,j) = −(φ(i+1,j) − φ(i−1,j)) / (2s)
Fy(i,j) = −(φ(i,j+1) − φ(i,j−1)) / (2s)
```

so that `F` points from cut ridges toward fill basins; streamlines of `F` are
traced by fixed-step integration for lane growth and visualization.

**[0044]** *Pass generation (region growing).* Each pass packs one load:

```text
function generate_pass(ledger, flow):
    anchor = argmin over cut cells c of  dist(c, nearest_fill(c))
    cut_patch = grow_contiguous(anchor, ledger.remaining_cut,
                                stop when Σ cut_bcy ≥ L_max or depth cap reached,
                                bias growth along flow streamline upstream)
    load = Σ cut_bcy(cut_patch)            # ≤ L_max
    fill_seed = nearest_fill(centroid(cut_patch)) biased by heading
    fill_patch = grow_contiguous(fill_seed, ledger.remaining_fill,
                                 stop when Σ fill_bcy ≥ load,
                                 bias growth along flow streamline downstream)
    haul_leg = dist(cut_end(cut_patch), fill_start(fill_patch))
    debit ledger by the worked depths; return Pass(cut_patch, fill_patch, load, haul_leg)
```

Passes are produced layer-by-layer (shallower layers first) so deeper layers of an
area are never worked before the layer above; volume is conserved by debiting the
ledger exactly. Multi-drop, the load-fullness cap, and the seed-connected
compactness guard are applied during growth as described in paragraphs [0026].

**[0045]** *Trapezoidal travel time.* Hauling and returning over a length `Lₘ` at
maximum speed `v` with acceleration time `tₐ` and deceleration time `t_d` take:

```text
d_acc = ½ · v · t_a
d_dec = ½ · v · t_d
if L_m ≥ d_acc + d_dec:                       # trapezoid (reaches v)
    time = t_a + t_d + (L_m − d_acc − d_dec) / v
else:                                          # triangle (peak v_p < v)
    a  = v / t_a ;  b = v / t_d
    v_p = sqrt( 2 · L_m / (1/a + 1/b) )
    time = v_p / a + v_p / b
```

Loading and dumping use constant speeds (`time = L / v`). The aggregate total is
`(loading + hauling + dumping + returning) / field_efficiency`.

**[0046]** *Direction-aware transition cost.* Let `a` and `b` be successive passes.
Let `exit(a) = a.fill_start` when the grade/compact return is enabled, else
`a.fill_end`. Then:

```text
dist        = euclidean(exit(a), b.cut_start)
drive_t     = dist / return_speed
dead_dir    = unit(b.cut_start − exit(a))
angle       = angle_between(exit_dir(a), dead_dir)
            + angle_between(dead_dir, cut_dir(b))
turn_t      = (turning_circle / 2) · angle / turn_speed
cost(a, b)  = drive_t + turn_t                 (seconds; direction-aware)
```

where `exit_dir(a)` is the heading at which the machine finishes `a` (the grade
return direction `fill_end → fill_start` when grading, else the spread direction)
and `cut_dir(b)` is `b`'s cut heading. With `--no-direction-aware`,
`cost(a, b) = dist` (deadhead distance only). The route is first built by
nearest-neighbour: starting from pass 0, repeatedly append the unvisited pass
minimizing `cost(current, ·)`. Sub-sequence-reversal (2-opt) is applied only in
the symmetric (distance-only) mode, never under the asymmetric cost above.

**[0047]** *Total order-dependent cost.* For a route given as an ordered list
`R = [p₀, p₁, …, p_{n−1}]`, define `J(R) = Σ_{k=0}^{n−2} cost(p_k, p_{k+1})`. Each
pass's own loading/hauling/dumping/grade sweep is fixed by its geometry and is
absent from `J`, so `J` is exactly the order-dependent part of the time, and
minimizing `J` minimizes overall time.

**[0048]** *Constant-time relocation delta.* Relocating the pass `x` at index `s`
to index `d` (the index it occupies after removal) changes only the transitions
adjacent to the source and destination. With `c(·,·) = cost(·,·)` and `c(·, ·)=0`
when either operand is absent (route ends), and writing `prev_s`, `next_s` for the
neighbours of `x`, and `left`, `right` for the neighbours straddling the insertion
point in the list with `x` removed:

```text
removed     = c(prev_s, x) + c(x, next_s)        # edges deleted at the source
bridge      = c(prev_s, next_s)                  # edge healed at the source
removed_ins = c(left, right)                      # edge broken at the destination
added_ins   = c(left, x) + c(x, right)            # edges created at the destination
Δ(s → d)    = (bridge − removed) + (added_ins − removed_ins)
```

`Δ < 0` is an improvement. All other transitions of `J` are unchanged, so `Δ` is
the exact change in `J(R)` and is evaluated in O(1).

**[0049]** *Windowed relocation refinement.* Using the delta of paragraph [0048]:

```text
function refine(route R, window N, max_rounds = 12, eps = 1e-7):
    for round in 1 … max_rounds:
        improved = false
        for k in 0 … |R|−1:                       # one pass at a time
            best_d, best_Δ = k, −eps
            for d in [max(0, k−N) … min(|R|−1, k+N)]:   # bounded window
                if d == k: continue
                if Δ(k → d) < best_Δ:
                    best_Δ, best_d = Δ(k → d), d
            if best_d ≠ k:
                relocate R[k] to best_d            # lift + re-insert; no reversal
                improved = true
        if not improved: break
    recompute total deadhead; return R
```

Each applied move has `best_Δ < 0`, so `J` strictly decreases on every acceptance;
`J ≥ 0` is bounded below, hence the loop converges and never increases `J`. Because
a relocation lifts and re-inserts a single pass without reversing any contiguous
run, every pass's internal heading is preserved, making the move sound under the
asymmetric `cost(·,·)` of paragraph [0046]. `N = 0` skips the refinement.

**[0050]** *Blade-scale lane tiling.* For a patch of cells with centroid `o` and a
lane heading `h`, let `û = (sin h, cos h)` and `v̂ = (cos h, −sin h)`. Each cell
centre `p` projects to along-lane coordinate `u = (p − o)·û` and cross-lane
coordinate `w = (p − o)·v̂`. Cells are binned into lanes spaced one blade width `Bₘ`
apart in `w`; within each lane, cells are sorted by `u`, and alternate lanes are
reversed to form a serpentine. The sweep is flipped end-for-end if necessary so its
terminus lies nearest the haul junction (cut sweeps end toward the fill start; fill
sweeps start from the cut end), thereby preserving the optimized haul leg.

**[0051]** *Turn counting and coverage floor.* A sweep's drivable centreline is
simplified by the Ramer–Douglas–Peucker procedure to a tolerance on the order of a
blade width; the interior vertex heading changes are measured, and those at or
above `turn_threshold_deg` are counted as tight turns and summed into a total turn
angle `Θ`. The guidance turning time is `(turning_circle/2)·Θ / turn_speed`. A
displayed steering line may be smoothed to a drivable curvature, but distance and
time are charged on the unsmoothed coverage geometry, whose length is bounded below
by `load / (d_max · B_m)`; a shorter smoothed sweep would under-cut the field and
is therefore not used for accounting.

### L. Variations and scope

**[0052]** The foregoing embodiments are illustrative. The grid spacing, scraper
capacity, swell factor, cut/fill ratio, depth cap, blade width, speeds, turning
radius, turn threshold, window size, and other parameters may be varied. The flow
field may be computed by other potential or transport formulations. The pass
generator may grow patches by other contiguity or streamline criteria. The
transition cost may weight drive and turn components differently or incorporate
additional penalties. The relocation move may be generalized to relocate short
contiguous segments without reversal. The guidance may be expressed in other
coordinate systems and emitted in other formats. Such variations are within the
scope of the invention, which is defined by the claims that follow.

## CLAIMS

**[0053]** What is claimed is:

**[0054]** 1. A computer-implemented method for generating an earthmoving haul
plan for leveling a field to a target grade, the method comprising, by one or more
processors:

&nbsp;&nbsp;&nbsp;&nbsp;receiving a digital terrain model defining, over the
field, an existing surface elevation and a target surface elevation, and binning
the field into a grid of areas and computing for each area a signed volumetric
quantity representing a cut or a fill;

&nbsp;&nbsp;&nbsp;&nbsp;generating a plurality of passes, each pass representing a
cut-haul-fill cycle in which a contiguous patch of cut areas is excavated to a
bounded per-pass load and deposited into a contiguous patch of fill areas, each
pass having a haul leg from an end of a cut sweep to a start of a fill sweep;

&nbsp;&nbsp;&nbsp;&nbsp;ordering the plurality of passes into a route by, for each
ordered pair of a first pass and a second pass, computing a direction-aware
transition cost comprising a deadhead drive time from an exit point of the first
pass to a cut-start of the second pass and a turning time proportional to a total
heading change required to leave the first pass, traverse a deadhead leg, and
enter the cut of the second pass, the turning time being determined from a turning
radius and a turning speed;

&nbsp;&nbsp;&nbsp;&nbsp;refining the route by a relocation local search that
traverses the passes one at a time and, for a subject pass, relocates the subject
pass from a current position to a destination position, within a bounded window of
positions, that reduces a total order-dependent cost equal to a sum of the
transition costs over consecutive passes of the route, the relocation lifting and
re-inserting the subject pass without reversing any contiguous run of passes; and

&nbsp;&nbsp;&nbsp;&nbsp;outputting the refined route as the earthmoving haul plan.

**[0055]** 2. The method of claim 1, wherein the relocation local search applies,
for the subject pass, the relocation within the bounded window that most reduces
the total order-dependent cost, and repeats the traversal in successive sweeps
until a sweep applies no relocation or a bounded number of sweeps is reached.

**[0056]** 3. The method of claim 1, wherein each accepted relocation strictly
decreases the total order-dependent cost such that the relocation local search is
monotone and convergent and does not increase the total order-dependent cost.

**[0057]** 4. The method of claim 1, wherein a change in the total order-dependent
cost resulting from a candidate relocation is computed in constant time from
transitions adjacent to the current position and the destination position.

**[0058]** 5. The method of claim 1, wherein the bounded window comprises positions
within a configurable number N of the current position, and wherein N is settable
to a value that disables the relocation local search and to a value that enables
it.

**[0059]** 6. The method of claim 1, wherein the ordering further comprises
constructing an initial route by a nearest-neighbor heuristic that repeatedly
appends an unvisited pass minimizing the transition cost from a current pass, and
wherein a sub-sequence-reversal improvement is withheld from the route on account
of the asymmetry of the direction-aware transition cost.

**[0060]** 7. The method of claim 1, wherein the refining of the route adds,
removes, splits, or alters no pass, no cut or fill area, no load, and no haul leg,
such that a moved volume and a finished surface defined by the route are unchanged
by the refining.

**[0061]** 8. The method of claim 1, further comprising computing a global flow
field assigning to each area a transport direction from cut regions to fill
regions by solving a potential equation having the signed volumetric quantity as a
source and taking a negative gradient of the potential, and growing the cut patch
of a pass upstream along the flow field and the fill patch downstream along the
flow field.

**[0062]** 9. The method of claim 1, further comprising, for each pass, tiling the
contiguous patch of cut areas and the contiguous patch of fill areas into a
plurality of lanes spaced approximately one blade width apart and traversed in a
serpentine order, and emitting an ordered list of waypoints each comprising a
position, a blade state, a target depth, a bearing to a next waypoint, and an
instruction.

**[0063]** 10. The method of claim 9, further comprising smoothing a displayed
steering line of the waypoints to a curvature drivable within the turning radius
while charging distance and time for the pass on an unsmoothed coverage geometry,
the unsmoothed coverage geometry having a length bounded below by the load divided
by a product of a maximum depth and the blade width.

**[0064]** 11. The method of claim 9, further comprising detecting a sweep that
forms a knot exhibiting repeated hairpin reversals and re-profiling the sweep into
parallel blade-width back-and-forth lanes only where the re-profiling strictly
reduces a count of hairpin reversals, the re-profiling re-ordering cells of a
patch without changing a load, a haul, a cut or fill quantity, or a finished
surface.

**[0065]** 12. The method of claim 1, further comprising estimating a drivable
total time of the route that counts a loading time, a hauling time, a dumping
time, a returning time, a grading time of a return pass driven over each fill, and
a turning time for heading changes exceeding a threshold, divided by a
field-efficiency factor.

**[0066]** 13. The method of claim 1, wherein each pass is constrained by a maximum
per-pass load and a maximum cut or fill depth per area per pass, deep cuts and
fills being worked over a plurality of passes in successive layers with shallower
layers worked first, and wherein soil volume is conserved across the passes.

**[0067]** 14. The method of claim 1, wherein an exit point of a pass is a
cut-side edge of a fill of the pass when a grade-and-compact return pass is
enabled, such that the return pass also constitutes travel toward a next cut.

**[0068]** 15. The method of claim 1, further comprising outputting a tabular
export of cut-haul-fill pairs in an order of the refined route, each pair
specifying a load, a haul distance, and a geographic start and end of a cut
segment and of a fill segment, and rendering a heat map in which each pair is
drawn as a semi-transparent segment from a cut location to a fill location.

**[0069]** 16. A system for generating an earthmoving haul plan, the system
comprising one or more processors and a memory storing instructions that, when
executed by the one or more processors, cause the system to:

&nbsp;&nbsp;&nbsp;&nbsp;bin a field defined by a digital terrain model into a grid
of areas and compute for each area a signed volumetric quantity representing a cut
or a fill;

&nbsp;&nbsp;&nbsp;&nbsp;generate a plurality of passes, each representing a
cut-haul-fill cycle having a bounded load and a haul leg;

&nbsp;&nbsp;&nbsp;&nbsp;order the plurality of passes into a route according to a
direction-aware transition cost that combines a deadhead drive time with a turning
time determined from a turning radius and a turning speed; and

&nbsp;&nbsp;&nbsp;&nbsp;refine the route by repeatedly relocating a single pass,
without reversing any contiguous run of passes, to a position within a bounded
window that reduces a sum of the transition costs over consecutive passes of the
route, until no such relocation reduces the sum.

**[0070]** 17. The system of claim 16, wherein the instructions cause the system to
compute, in constant time, a change in the sum resulting from a candidate
relocation, and to apply a relocation only when the relocation strictly reduces
the sum, whereby the refining is monotone and convergent.

**[0071]** 18. The system of claim 16, wherein the instructions further cause the
system to tile each pass into blade-width serpentine lanes following a local
transport direction of a flow field and to emit a turn-by-turn guidance track of
waypoints for an in-cab positioning display.

**[0072]** 19. A non-transitory computer-readable medium storing instructions
that, when executed by one or more processors, cause the one or more processors to
perform operations comprising: generating a plurality of cut-haul-fill passes from
a digital terrain model of a field; ordering the passes into a route according to
a direction-aware transition cost that includes a turning time; and refining the
route by a relocation local search that, traversing the passes one at a time,
lifts a single subject pass and re-inserts it at a position within a bounded
window that reduces a total order-dependent cost of the route, without reversing
any contiguous run of passes, the relocation local search being monotone such that
it does not increase the total order-dependent cost.

**[0073]** 20. The non-transitory computer-readable medium of claim 19, wherein
the operations further comprise withholding a sub-sequence-reversal improvement
from the route on account of an asymmetry of the direction-aware transition cost,
the relocation local search thereby supplying an order-improvement step that is
sound under the asymmetric transition cost.

## ABSTRACT

**[0074]** A computer-implemented method and system transform a digital terrain
model of a field into a drivable, time-optimized earthmoving haul plan. The field
is binned into a grid and per-area cut/fill volumes are computed; a global flow
field defines transport directions from cut to fill; and a plurality of
cut-haul-fill passes, each bounded by a per-pass load and a depth cap, is
generated, optionally as long lanes following the flow. The passes are ordered by
a direction-aware transition cost that combines deadhead drive time with a turning
time derived from a turning radius and turn speed. Because the turn cost is
asymmetric, sub-sequence reversal is unsafe; instead a windowed relocation local
search walks the passes one at a time and relocates each, without reversing any
run, to a nearby position that reduces a total order-dependent time, converging
monotonically and never increasing the time. The plan is realized at blade scale
as serpentine lanes with smoothed steering, a grade/compact return pass, and a
turn-by-turn waypoint track, and is exported as maps, cut-haul pairs, guidance,
and an animation.
