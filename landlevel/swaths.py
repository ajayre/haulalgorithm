"""Stage 5.5 - blade-scale swath builder (operator guidance).

The route plan (``routing.sequence_passes``) is optimized at the *calculation
grid* scale (default 15 ft cells), which is wider than the physical blade
(2271 mm ~= 7.45 ft). A 15 ft-wide cell therefore needs roughly two side-by-side
blade widths to actually be cut or filled. This module turns the optimized plan
into a **blade-scale, drivable guidance track** that an in-cab system (AgGrade)
can follow turn-by-turn, *without changing the soil-routing decisions* that the
optimizer already made.

What is preserved (so the optimized time/distance is not worsened):

* The cut->fill **assignment** of every pass (which soil goes where) is untouched.
* The pass **sequence** (deadhead return order) is untouched.
* The **haul leg** of each pass (the optimized cut-end -> fill-start transit, the
  primary objective) is reproduced as-is.

What is added (physical reality, previously under-counted):

* Each pass's cut patch and fill patch are tiled into **blade-width lanes** run
  along the haul heading, in a serpentine order, so the narrow blade physically
  covers the full width of the wider grid cells. The extra loading/dumping sweep
  this implies is genuine blade coverage, not an algorithmic regression; both the
  planned (centerline) and blade-scale sweep distances are reported so the
  difference is explicit.

The output is an ordered list of :class:`GuidancePoint` waypoints, each with a
lat/lon, an action (drive / cut / haul / spread), a target blade depth, the
bearing to the next waypoint, and a plain-language instruction. It can be written
to a guidance CSV and drawn as a lane overlay PNG.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .agd import AgdField, latlon_from_utm
from .grid import Grid
from .routing import Route

Cell = Tuple[int, int]
XY = Tuple[float, float]

# Leg/action types (the move performed on the way to a waypoint).
START = "START"     # very first move of the whole job (blade up)
RETURN = "RETURN"   # deadhead from previous fill to the next cut start (blade up)
HAUL = "HAUL"       # loaded transit from cut end to fill start (blade up)
CUT = "CUT"         # cutting along a blade lane (blade down)
FILL = "FILL"       # spreading along a blade lane (blade down)
GRADE = "GRADE"     # return pass back over the fill to grade + compact (blade down)
MOVE = "MOVE"       # multi-drop reposition between pockets within a pass (blade up)

_BLADE_UP = {START, RETURN, HAUL, MOVE}


@dataclass
class GuidancePoint:
    """One waypoint in the blade-scale operator guidance track."""

    seq: int               # 1-based global step
    pass_no: int           # 1-based pass index in the optimized route
    action: str            # START | RETURN | HAUL | CUT | FILL
    blade: str             # "UP" | "DOWN"
    x: float               # world easting (m)
    y: float               # world northing (m)
    lat: float
    lon: float
    depth_mm: float        # target cut/fill depth at this point (magnitude); 0 if blade up
    heading_deg: float     # compass bearing to the NEXT waypoint (0=N, 90=E)
    seg_dist_m: float      # distance travelled from the previous waypoint
    cum_dist_m: float      # cumulative distance from the start
    instruction: str       # plain-language, idiot-proof cue


@dataclass
class GuidanceResult:
    points: List[GuidancePoint]
    blade_width_m: float
    grid_size_m: float
    heading_deg: float
    n_passes: int
    # Distance accounting (m).
    return_dist_m: float
    haul_dist_m: float
    cut_dist_m: float          # blade-scale loading sweep (with lane coverage)
    fill_dist_m: float         # blade-scale dumping sweep (with lane coverage)
    grade_dist_m: float        # grade/compact return pass over fills
    reposition_dist_m: float   # multi-drop blade-up hops between pockets within passes
    planned_cut_dist_m: float  # original centerline loading sweep (pre-blade)
    planned_fill_dist_m: float
    route_haul_dist_m: float   # optimizer's haul leg total (objective; must match haul_dist_m)
    # Turning (45 ft circle etc.).
    n_turns: int               # count of significant heading changes (> 30 deg)
    turn_angle_rad: float      # total absolute heading change over the track

    def coverage_factor(self) -> float:
        """Blade-scale sweep length / planned centerline sweep length (>= 1)."""
        planned = self.planned_cut_dist_m + self.planned_fill_dist_m
        actual = self.cut_dist_m + self.fill_dist_m
        return actual / planned if planned > 0 else 1.0


@dataclass
class GuidanceTime:
    """Realistic time built from the drivable guidance track (incl. turns)."""

    total_s: float
    loading_s: float
    dumping_s: float
    grading_s: float
    hauling_s: float
    returning_s: float
    turning_s: float
    other_s: float
    n_turns: int


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _bearing_deg(p: XY, q: XY) -> float:
    """Compass bearing from p to q (0=N, 90=E)."""
    de = q[0] - p[0]   # east
    dn = q[1] - p[1]   # north
    if abs(de) < 1e-12 and abs(dn) < 1e-12:
        return 0.0
    return math.degrees(math.atan2(de, dn)) % 360.0


def _d2(p: XY, q: XY) -> float:
    return (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2


def _centroid(points: List[XY]) -> XY:
    n = len(points)
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)


def _seg_length(points: List[XY]) -> float:
    return sum(math.dist(points[k], points[k + 1]) for k in range(len(points) - 1))


def _lanes_for_patch(
    cell_list: List[Tuple[Cell, float]],
    grid: Grid,
    blade_m: float,
    heading_deg: float,
    overlap: float = 0.0,
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> List[Tuple[float, float, float]]:
    """Tile a patch of cells into blade-width lanes; return drive-ordered points.

    Lanes run along ``heading_deg`` (the haul heading / main slope), spaced one
    blade width apart across the perpendicular extent of the patch, traversed in
    a serpentine order so lane-to-lane turns are short. Each returned tuple is
    ``(x, y, depth_m)``: a waypoint on a lane centerline and the target depth of
    the cell it covers.

    A cell of size ``grid_size`` wider than the blade is assigned to every lane
    whose band overlaps it, so the narrow blade covers the full cell width over
    successive lanes (volume is conserved by coverage geometry: lanes * blade *
    length ~= cell area). When the grid is already ~one blade wide (e.g. a 7.5 ft
    grid), this reduces to one lane per cell row.

    ``orient_end_xy`` (preferred) orients the sweep to *end* near that point (used
    for cut patches, so loading ends toward the fill); ``orient_start_xy`` orients
    it to *start* near that point (used for fill patches, so dumping starts where
    the haul ended). Either keeps the per-pass haul leg ~unchanged.
    """
    if not cell_list:
        return []
    centers = [grid.cell_center_xy(*c) for c, _d in cell_list]
    depths = [d for _c, d in cell_list]
    if len(cell_list) == 1:
        return [(centers[0][0], centers[0][1], depths[0])]

    th = math.radians(heading_deg)
    uh = (math.sin(th), math.cos(th))     # down-heading unit (east, north)
    vh = (math.cos(th), -math.sin(th))    # perpendicular unit
    ox, oy = _centroid(centers)
    us = [(x - ox) * uh[0] + (y - oy) * uh[1] for x, y in centers]
    vs = [(x - ox) * vh[0] + (y - oy) * vh[1] for x, y in centers]

    G = grid.grid_size_m
    overlap = min(max(overlap, 0.0), 0.9)
    step = max(blade_m * (1.0 - overlap), 1e-3)
    v_min, v_max = min(vs), max(vs)
    n_lanes = max(1, int(math.floor((v_max - v_min) / step + 1e-9)) + 1)
    lane_v = [v_min + step * k for k in range(n_lanes)]
    # Only add a final edge lane when there is at least half a blade of width
    # still uncovered. Appending for any tiny residual (e.g. a strip tilted a
    # degree off the grid axis) spuriously splits a single-row strip into a
    # two-lane serpentine, flipping its exit end and inflating the haul leg. Stray
    # cells beyond the last lane are still captured by the coverage loop below.
    if v_max - lane_v[-1] > 0.5 * step:
        lane_v.append(v_max)

    half = G / 2.0 + 1e-6
    lanes: List[Tuple[float, List[Tuple[float, float, int]]]] = []
    for vc in lane_v:
        members = [(us[i], depths[i], i) for i in range(len(cell_list)) if abs(vs[i] - vc) <= half]
        lanes.append((vc, members))

    # Guarantee every cell is represented by at least its nearest lane.
    covered = {i for _vc, mem in lanes for (_u, _d, i) in mem}
    for i in range(len(cell_list)):
        if i not in covered:
            j = min(range(len(lanes)), key=lambda L: abs(lanes[L][0] - vs[i]))
            lanes[j][1].append((us[i], depths[i], i))

    lanes = [(vc, mem) for vc, mem in lanes if mem]

    pts: List[Tuple[float, float, float]] = []
    for k, (vc, mem) in enumerate(lanes):
        mem.sort(key=lambda t: t[0])      # along-heading order
        if k % 2 == 1:                    # serpentine: reverse every other lane
            mem = list(reversed(mem))
        for (u_i, d_i, _i) in mem:
            x = ox + u_i * uh[0] + vc * vh[0]
            y = oy + u_i * uh[1] + vc * vh[1]
            pts.append((x, y, d_i))

    # Orient the sweep so the haul leg stays the optimized transit.
    if orient_end_xy is not None:
        if _d2((pts[0][0], pts[0][1]), orient_end_xy) < _d2((pts[-1][0], pts[-1][1]), orient_end_xy):
            pts.reverse()
    elif orient_start_xy is not None:
        if _d2((pts[-1][0], pts[-1][1]), orient_start_xy) < _d2((pts[0][0], pts[0][1]), orient_start_xy):
            pts.reverse()
    return pts


def _connected_pockets(cell_list: List[Tuple[Cell, float]]) -> List[List[Tuple[Cell, float]]]:
    """Split a (possibly multi-drop) patch into 8-connected pockets of cells."""
    cells = {c for c, _d in cell_list}
    seen: set = set()
    comps: List[set] = []
    for start in cells:
        if start in seen:
            continue
        comp = {start}
        seen.add(start)
        stack = [start]
        while stack:
            cx, cy = stack.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nb = (cx + dx, cy + dy)
                    if nb in cells and nb not in seen:
                        seen.add(nb)
                        comp.add(nb)
                        stack.append(nb)
        comps.append(comp)
    # Re-attach depths (keeping every entry, including any duplicate cells).
    return [[(c, d) for c, d in cell_list if c in comp] for comp in comps]


def _lanes_multi(
    cell_list: List[Tuple[Cell, float]],
    grid: Grid,
    blade_m: float,
    heading_deg: float,
    overlap: float,
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> List[Tuple[float, float, float]]:
    """Pocket-aware lane builder for multi-drop passes.

    A single pass may span several disjoint pockets. Tiling them all as one lane
    field lets the sweep finish anywhere, which inflates the haul leg. Instead we
    split the patch into connected pockets and order them so the load *finishes*
    at the pocket nearest the fill (``orient_end_xy``, the cut side) or *starts*
    at the pocket nearest the haul arrival (``orient_start_xy``, the fill side).
    Each pocket is oriented toward the next so inter-pocket hops stay short.
    """
    pockets = _connected_pockets(cell_list)
    if len(pockets) <= 1:
        return _lanes_for_patch(cell_list, grid, blade_m, heading_deg, overlap,
                                orient_start_xy=orient_start_xy, orient_end_xy=orient_end_xy)

    cents = [_centroid([grid.cell_center_xy(*c) for c, _d in pk]) for pk in pockets]
    pts: List[Tuple[float, float, float]] = []
    if orient_end_xy is not None:
        # Cut side: visit farthest-from-fill first, finish nearest the fill.
        order = sorted(range(len(pockets)), key=lambda i: -_d2(cents[i], orient_end_xy))
        for rank, i in enumerate(order):
            tgt = orient_end_xy if rank == len(order) - 1 else cents[order[rank + 1]]
            pts += _lanes_for_patch(pockets[i], grid, blade_m, heading_deg, overlap,
                                    orient_end_xy=tgt)
    else:
        # Fill side: start nearest the haul arrival, then chain outward.
        src = orient_start_xy if orient_start_xy is not None else cents[0]
        order = sorted(range(len(pockets)), key=lambda i: _d2(cents[i], src))
        for rank, i in enumerate(order):
            st = src if rank == 0 else cents[order[rank - 1]]
            pts += _lanes_for_patch(pockets[i], grid, blade_m, heading_deg, overlap,
                                    orient_start_xy=st)
    return pts


# ---------------------------------------------------------------------------
# Guidance track builder
# ---------------------------------------------------------------------------

def build_guidance(
    grid: Grid,
    field: AgdField,
    route: Route,
    blade_width_mm: float,
    heading_deg: float,
    lane_overlap: float = 0.0,
    grade_return: bool = True,
    turn_threshold_deg: float = 60.0,
) -> GuidanceResult:
    """Build the blade-scale operator guidance track from the optimized route.

    The route's passes are walked in order; each pass's cut and fill patches are
    tiled into blade-width serpentine lanes (:func:`_lanes_for_patch`). When
    ``grade_return`` is set, each fill is followed by a single return pass back
    over the fill to grade and compact it. The resulting waypoints are emitted as
    a single ordered track with per-point actions, target depths, next-bearings,
    and instructions.
    """
    blade_m = blade_width_mm / 1000.0
    zone, north = field.zone, field.is_northern

    points: List[GuidancePoint] = []
    return_d = haul_d = cut_d = fill_d = grade_d = reposition_d = 0.0
    planned_cut_d = planned_fill_d = route_haul_d = 0.0

    prev_xy: Optional[XY] = None       # end of the previous waypoint
    seq = 0

    # Raw records first (x, y, depth_m, action, pass_no); distances/bearings after.
    raw: List[Tuple[float, float, float, str, int]] = []

    for pass_no, p in enumerate(route.ordered_passes, start=1):
        route_haul_d += p.haul_distance_m
        planned_cut_d += _seg_length([grid.cell_center_xy(*c) for c, _d in p.cut_cells])
        planned_fill_d += _seg_length([grid.cell_center_xy(*c) for c, _d in p.fill_cells])

        # Orient the cut sweep to end at the optimized haul target (where dumping
        # begins) so the per-pass haul leg stays the optimizer's transit.
        cut_lane = _lanes_multi(
            p.cut_cells, grid, blade_m, heading_deg, lane_overlap,
            orient_end_xy=p.fill_start_xy,
        )
        cut_end_xy = (cut_lane[-1][0], cut_lane[-1][1]) if cut_lane else p.cut_end_xy
        fill_lane = _lanes_multi(
            p.fill_cells, grid, blade_m, heading_deg, lane_overlap,
            orient_start_xy=cut_end_xy,
        )

        # A multi-drop pass may span several pockets; the blade-up drive between
        # two pockets shows up as an unusually long step within a sweep. Emit
        # those gaps as HAUL repositions (blade up) instead of cutting/filling
        # across empty ground.
        hop_thresh = max(3.0 * grid.grid_size_m, 2.0 * blade_m)
        for i, (x, y, d) in enumerate(cut_lane):
            if i == 0:
                action = START if pass_no == 1 else RETURN
            elif math.dist((cut_lane[i - 1][0], cut_lane[i - 1][1]), (x, y)) > hop_thresh:
                action = MOVE
            else:
                action = CUT
            raw.append((x, y, d, action, pass_no))
        for i, (x, y, d) in enumerate(fill_lane):
            if i == 0:
                action = HAUL
            elif math.dist((fill_lane[i - 1][0], fill_lane[i - 1][1]), (x, y)) > hop_thresh:
                action = MOVE
            else:
                action = FILL
            raw.append((x, y, d, action, pass_no))
        # Grade/compact: turn at the fill end and drive back over the fill. The
        # return retraces the fill lane in reverse (blade down, no new material);
        # inter-pocket hops are blade-up repositions, not graded ground.
        if grade_return and len(fill_lane) >= 2:
            rev = list(reversed(fill_lane))
            for i, (x, y, _d) in enumerate(rev):
                if i > 0 and math.dist((rev[i - 1][0], rev[i - 1][1]), (x, y)) > hop_thresh:
                    raw.append((x, y, 0.0, MOVE, pass_no))
                else:
                    raw.append((x, y, 0.0, GRADE, pass_no))

    # Second pass: distances, bearings, instructions.
    cum = 0.0
    for idx, (x, y, depth_m, action, pass_no) in enumerate(raw):
        xy = (x, y)
        seg = math.dist(prev_xy, xy) if prev_xy is not None else 0.0
        if action in (START,):
            seg = 0.0  # no travel attributed before the very first waypoint
        if action == RETURN:
            return_d += seg
        elif action == HAUL:
            haul_d += seg
        elif action == CUT:
            cut_d += seg
        elif action == FILL:
            fill_d += seg
        elif action == GRADE:
            grade_d += seg
        elif action == MOVE:
            reposition_d += seg
        cum += seg

        nxt = raw[idx + 1] if idx + 1 < len(raw) else None
        heading = _bearing_deg(xy, (nxt[0], nxt[1])) if nxt is not None else 0.0

        # ``blade`` is the state on the inbound leg (UP while repositioning). Every
        # waypoint sits on a real cut/fill cell, so each carries its own target
        # depth - including the first cell of a patch, which is reached by a
        # blade-up reposition but is still cut/filled to depth once the blade drops.
        blade = "UP" if action in _BLADE_UP else "DOWN"
        depth_mm = abs(depth_m) * 1000.0
        lat, lon = latlon_from_utm(x, y, zone, north)

        seq += 1
        points.append(GuidancePoint(
            seq=seq, pass_no=pass_no, action=action, blade=blade,
            x=x, y=y, lat=lat, lon=lon, depth_mm=depth_mm,
            heading_deg=heading, seg_dist_m=seg, cum_dist_m=cum,
            instruction=_instruction(action, depth_mm),
        ))
        prev_xy = xy

    n_turns, turn_angle = _turn_metrics(points, grid.grid_size_m, blade_m,
                                        turn_threshold_deg=turn_threshold_deg)

    return GuidanceResult(
        points=points,
        blade_width_m=blade_m,
        grid_size_m=grid.grid_size_m,
        heading_deg=heading_deg,
        n_passes=len(route.ordered_passes),
        return_dist_m=return_d,
        haul_dist_m=haul_d,
        cut_dist_m=cut_d,
        fill_dist_m=fill_d,
        grade_dist_m=grade_d,
        reposition_dist_m=reposition_d,
        planned_cut_dist_m=planned_cut_d,
        planned_fill_dist_m=planned_fill_d,
        route_haul_dist_m=route_haul_d,
        n_turns=n_turns,
        turn_angle_rad=turn_angle,
    )


def _rdp_keep(xs: List[float], ys: List[float], tol: float) -> List[int]:
    """Ramer-Douglas-Peucker: indices of vertices to keep within ``tol`` (m).

    Iterative (stack-based) so it handles the full guidance track without
    recursion limits. Collapses the cell-by-cell staircase of a discretized lane
    into the straight line an operator actually drives, while preserving real
    corners (lane-end U-turns, cut->haul->fill transitions).
    """
    n = len(xs)
    if n < 3:
        return list(range(n))
    keep = [False] * n
    keep[0] = keep[-1] = True
    tol2 = tol * tol
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        ax, ay, bx, by = xs[a], ys[a], xs[b], ys[b]
        dx, dy = bx - ax, by - ay
        seglen2 = dx * dx + dy * dy
        dmax = 0.0
        idx = -1
        for i in range(a + 1, b):
            if seglen2 == 0.0:
                d2 = (xs[i] - ax) ** 2 + (ys[i] - ay) ** 2
            else:
                t = ((xs[i] - ax) * dx + (ys[i] - ay) * dy) / seglen2
                px, py = ax + t * dx, ay + t * dy
                d2 = (xs[i] - px) ** 2 + (ys[i] - py) ** 2
            if d2 > dmax:
                dmax = d2
                idx = i
        if idx != -1 and dmax > tol2:
            keep[idx] = True
            stack.append((a, idx))
            stack.append((idx, b))
    return [i for i in range(n) if keep[i]]


def _turn_metrics(
    points: List[GuidancePoint],
    grid_size_m: float,
    blade_m: float,
    turn_threshold_deg: float = 60.0,
):
    """Total heading change and count of slow-down turns along the track.

    The raw track has one waypoint per cell, so a slightly diagonal lane looks
    like a staircase of 90 deg jogs. We first simplify the path (RDP) to the line
    the operator really drives, then look at the heading change at each remaining
    vertex. Heading changes **below** ``turn_threshold_deg`` are gentle bends
    taken in stride at travel speed (no slowdown, not counted); only changes at or
    above the threshold are genuine tight turns that force a slowdown and an arc,
    so only those are counted and summed into the turn-time angle.
    """
    if len(points) < 3:
        return 0, 0.0
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    tol = max(blade_m, grid_size_m) * 1.5
    idx = _rdp_keep(xs, ys, tol)
    bearings: List[float] = []
    for k in range(len(idx) - 1):
        i, j = idx[k], idx[k + 1]
        bearings.append(math.degrees(math.atan2(xs[j] - xs[i], ys[j] - ys[i])))
    n_turns = 0
    total = 0.0
    for k in range(1, len(bearings)):
        d = abs(((bearings[k] - bearings[k - 1] + 180.0) % 360.0) - 180.0)
        # Only sharp maneuvers force the slow-down-and-arc turn; gentle bends
        # (below the threshold) are absorbed into normal travel at speed.
        if d >= turn_threshold_deg:
            total += math.radians(d)
            n_turns += 1
    return n_turns, total


def estimate_guidance_time(
    result: GuidanceResult,
    loading_speed_mph: float,
    haul_speed_mph: float,
    dumping_speed_mph: float,
    return_speed_mph: float,
    grade_speed_mph: float,
    turn_speed_mph: float,
    turning_circle_ft: float,
    accel_time_s: float,
    decel_time_s: float,
    field_efficiency: float,
) -> GuidanceTime:
    """Time the *drivable* guidance track, including turns and the grade return.

    Loading, dumping and grading run at constant speeds; hauling and returning
    use the trapezoidal accel/decel profile (per leg). Every significant heading
    change costs turning time: the machine slows to ``turn_speed`` and travels the
    turn arc ``R * angle`` where ``R`` is the turning-circle radius. Field
    efficiency adds non-productive ("Other") time.
    """
    from . import FEET_TO_METERS, MPH_TO_MPS
    from .timemodel import _move_time

    load_v = loading_speed_mph * MPH_TO_MPS
    haul_vmax = haul_speed_mph * MPH_TO_MPS
    dump_v = dumping_speed_mph * MPH_TO_MPS
    return_vmax = return_speed_mph * MPH_TO_MPS
    grade_v = grade_speed_mph * MPH_TO_MPS
    turn_v = turn_speed_mph * MPH_TO_MPS
    R = (turning_circle_ft * FEET_TO_METERS) / 2.0

    t_load = t_dump = t_grade = t_haul = t_return = 0.0
    for pt in result.points:
        s = pt.seg_dist_m
        if pt.action == CUT:
            t_load += s / load_v if load_v > 0 else 0.0
        elif pt.action == FILL:
            t_dump += s / dump_v if dump_v > 0 else 0.0
        elif pt.action == GRADE:
            t_grade += s / grade_v if grade_v > 0 else 0.0
        elif pt.action in (HAUL, MOVE):
            t_haul += _move_time(s, haul_vmax, accel_time_s, decel_time_s)
        elif pt.action in (RETURN, START):
            t_return += _move_time(s, return_vmax, accel_time_s, decel_time_s)

    t_turn = (R * result.turn_angle_rad / turn_v) if turn_v > 0 else 0.0

    productive = t_load + t_dump + t_grade + t_haul + t_return + t_turn
    eff = field_efficiency if 0.0 < field_efficiency <= 1.0 else 1.0
    total = productive / eff
    other = total - productive
    return GuidanceTime(
        total_s=total, loading_s=t_load, dumping_s=t_dump, grading_s=t_grade,
        hauling_s=t_haul, returning_s=t_return, turning_s=t_turn, other_s=other,
        n_turns=result.n_turns,
    )


def _instruction(action: str, depth_mm: float) -> str:
    d = int(round(depth_mm))
    if action == START:
        return f"Start: drive to first cut, lower blade, cut {d} mm"
    if action == RETURN:
        return f"Raise blade, drive to next cut, lower blade, cut {d} mm"
    if action == CUT:
        return f"Cut {d} mm"
    if action == HAUL:
        return f"Raise blade, haul to fill, lower blade, spread {d} mm"
    if action == FILL:
        return f"Spread {d} mm"
    if action == GRADE:
        return "Turn back over fill, grade & compact"
    if action == MOVE:
        return f"Raise blade, drive to next pocket, lower blade, {d} mm"
    return ""


def write_guidance_csv(result: GuidanceResult, path) -> None:
    """Write the blade-scale guidance track as a CSV for in-cab consumption."""
    import csv

    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow([
            "seq", "pass", "action", "blade", "lat", "lon",
            "depth_mm", "heading_deg", "seg_dist_m", "cum_dist_m", "instruction",
        ])
        for pt in result.points:
            w.writerow([
                pt.seq, pt.pass_no, pt.action, pt.blade,
                f"{pt.lat:.7f}", f"{pt.lon:.7f}", f"{pt.depth_mm:.0f}",
                f"{pt.heading_deg:.1f}", f"{pt.seg_dist_m:.2f}", f"{pt.cum_dist_m:.1f}",
                pt.instruction,
            ])
