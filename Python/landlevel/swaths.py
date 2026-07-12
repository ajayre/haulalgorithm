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

from . import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS, M3_TO_CY
from .agd import AgdField, latlon_from_utm
from .grid import Grid
from .obstacles import EMPTY_OBSTACLES, ObstacleField
from .passes import Pass
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
    x: float               # world easting (m) - drivable (smoothed) steering point
    y: float               # world northing (m) - drivable (smoothed) steering point
    lat: float
    lon: float
    depth_mm: float        # target cut/fill depth at this point (magnitude); 0 if blade up
    heading_deg: float     # compass bearing to the NEXT waypoint (0=N, 90=E)
    seg_dist_m: float      # blade-coverage travel from the previous waypoint (NOT
                           # the straight-line gap between smoothed points: the
                           # blade must cover the full cell band, so distance/time
                           # are charged on the raw coverage path, not the shorter
                           # smoothed steering line)
    cum_dist_m: float      # cumulative coverage distance from the start
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
    # Find-and-fix: sweeps whose self-crossing knot was re-tiled into a clean
    # principal-axis serpentine (§13.3b).
    n_repaired_sweeps: int = 0

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


def _hard_turn_count(
    lane: List[Tuple[float, float, float]],
    grid: Grid,
    blade_m: float,
    deg_threshold: float = 100.0,
) -> int:
    """Number of near-reversal (hairpin) turns on a sweep's drivable centreline.

    The raw lane steps one waypoint per cell, so a diagonal lane is a staircase
    of 45/90 deg jogs; we first RDP-simplify to the line the operator actually
    drives, then count interior vertices whose heading change is at or above
    ``deg_threshold`` (a hairpin no 45 ft-circle machine can track in place).

    A clean serpentine only hairpins at its lane *ends* (so a k-lane patch scores
    ~k-1); a fragmented "blob" chained by nearest-run order hairpins repeatedly in
    a tiny area and scores much higher. The gap is what the repair keys on.
    """
    n = len(lane)
    if n < 3:
        return 0
    xs = [p[0] for p in lane]
    ys = [p[1] for p in lane]
    tol = max(0.5 * blade_m, 0.5 * grid.grid_size_m)
    keep = _rdp_keep(xs, ys, tol)
    if len(keep) < 3:
        return 0
    cnt = 0
    for k in range(1, len(keep) - 1):
        a = (xs[keep[k - 1]], ys[keep[k - 1]])
        b = (xs[keep[k]], ys[keep[k]])
        c = (xs[keep[k + 1]], ys[keep[k + 1]])
        b0 = _bearing_deg(a, b)
        b1 = _bearing_deg(b, c)
        if abs(((b1 - b0 + 180.0) % 360.0) - 180.0) >= deg_threshold:
            cnt += 1
    return cnt


def _principal_heading_deg(centers: List[XY]) -> float:
    """Compass bearing (0=N, 90=E) of a point cloud's principal (major) axis.

    Used to re-tile a tangled patch into clean parallel lanes that run along the
    patch's long axis (fewest lanes -> fewest U-turns).
    """
    n = len(centers)
    if n < 2:
        return 0.0
    ox, oy = _centroid(centers)
    sxx = sxy = syy = 0.0
    for x, y in centers:
        dx, dy = x - ox, y - oy
        sxx += dx * dx
        sxy += dx * dy
        syy += dy * dy
    tr = sxx + syy
    det = sxx * syy - sxy * sxy
    disc = max(0.0, (tr * 0.5) ** 2 - det)
    lam = tr * 0.5 + math.sqrt(disc)          # major eigenvalue
    ex, ey = sxy, lam - sxx                     # corresponding eigenvector
    if abs(ex) < 1e-12 and abs(ey) < 1e-12:
        ex, ey = lam - syy, sxy
    if abs(ex) < 1e-12 and abs(ey) < 1e-12:
        return 0.0
    return math.degrees(math.atan2(ex, ey)) % 360.0


def _smooth_run_xy(xys: List[XY], sigma_idx: float) -> List[XY]:
    """Gaussian-smooth a single drivable run, keeping the same waypoint count.

    The grid lays cut/fill waypoints on cell centres, so a streamline that drifts
    off-axis steps in 45/90 deg jogs that no tractor+scraper can physically track.
    A discrete Gaussian over the waypoint sequence (sigma in *waypoints*, ~one per
    cell) rounds those jogs into the curve the machine actually drives. The first
    and last waypoint are pinned so the haul leg / deadhead junctions they feed are
    left exactly as the optimizer placed them.
    """
    n = len(xys)
    if n < 3 or sigma_idx <= 0.0:
        return list(xys)
    rad = min(max(1, int(round(3.0 * sigma_idx))), n - 1)
    ks = list(range(-rad, rad + 1))
    w = [math.exp(-(k * k) / (2.0 * sigma_idx * sigma_idx)) for k in ks]
    sw = sum(w)
    w = [v / sw for v in w]
    out: List[XY] = []
    for i in range(n):
        sx = sy = 0.0
        for k, wk in zip(ks, w):
            j = i + k
            if j < 0:               # reflect at the ends to avoid shrinking inward
                j = -j
            elif j >= n:
                j = 2 * (n - 1) - j
            j = min(max(j, 0), n - 1)
            sx += wk * xys[j][0]
            sy += wk * xys[j][1]
        out.append((sx, sy))
    out[0] = xys[0]
    out[-1] = xys[-1]
    return out


def _smooth_lane(
    lane: List[Tuple[float, float, float]],
    sigma_idx: float,
    hop_thresh: float,
) -> List[Tuple[float, float, float]]:
    """Smooth a lane's geometry within each contiguous run, preserving depths.

    The lane is split at multi-drop hops (jumps > ``hop_thresh``, which become
    blade-up MOVE repositions) so smoothing never bridges two separate pockets;
    each contiguous run is rounded with :func:`_smooth_run_xy`. Depths and the
    waypoint count are untouched, so actions/instructions map 1:1 as before.
    """
    if sigma_idx <= 0.0 or len(lane) < 3:
        return lane
    runs: List[List[int]] = []
    cur = [0]
    for i in range(1, len(lane)):
        if math.dist((lane[i - 1][0], lane[i - 1][1]), (lane[i][0], lane[i][1])) > hop_thresh:
            runs.append(cur)
            cur = [i]
        else:
            cur.append(i)
    runs.append(cur)

    out = list(lane)
    for run in runs:
        if len(run) < 3:
            continue
        sm = _smooth_run_xy([(lane[i][0], lane[i][1]) for i in run], sigma_idx)
        for k, i in enumerate(run):
            out[i] = (sm[k][0], sm[k][1], lane[i][2])
    return out


def _circumradius_m(a: XY, b: XY, c: XY) -> float:
    """Circumradius of triangle ABC in metres; inf when nearly collinear."""
    la = math.dist(b, c)
    lb = math.dist(a, c)
    lc = math.dist(a, b)
    cross = abs(a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if cross < 1e-9:
        return float("inf")
    # Shoelace magnitude is 2K; R = abc / (4K) = abc / (2 * cross).
    return (la * lb * lc) / (2.0 * cross)


def _heading_change_deg(a: XY, b: XY, c: XY) -> float:
    """Absolute heading change at b along a→b→c (degrees)."""
    dx1 = b[0] - a[0]
    dy1 = b[1] - a[1]
    dx2 = c[0] - b[0]
    dy2 = c[1] - b[1]
    return abs(math.degrees(math.atan2(dx1 * dy2 - dy1 * dx2, dx1 * dx2 + dy1 * dy2)))


def _compass_to_math(heading_deg: float) -> float:
    """Convert compass bearing (0=N, 90=E) to math angle CCW from +X (east)."""
    return math.pi / 2.0 - math.radians(heading_deg)


def _mod2pi(angle: float) -> float:
    """Wrap angle into [0, 2π)."""
    a = math.fmod(angle, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a


def _forward_en(heading_deg: float) -> XY:
    """Unit forward vector (east, north) for a compass heading."""
    Rad = math.radians(heading_deg)
    return (math.sin(Rad), math.cos(Rad))


def _circle_intersections(c0: XY, c1: XY, radius: float) -> List[XY]:
    """Intersection points of two circles of equal radius (0, 1, or 2 points)."""
    Dx = c1[0] - c0[0]
    Dy = c1[1] - c0[1]
    Dist = math.hypot(Dx, Dy)
    if Dist < 1e-9 or Dist > (2.0 * radius) + 1e-9:
        return []
    # Midline distance from c0.
    A = Dist / 2.0
    H_sq = max(0.0, radius * radius - A * A)
    H = math.sqrt(H_sq)
    Mx = c0[0] + Dx * 0.5
    My = c0[1] + Dy * 0.5
    if H < 1e-9:
        return [(Mx, My)]
    Ux = -Dy / Dist
    Uy = Dx / Dist
    return [(Mx + H * Ux, My + H * Uy), (Mx - H * Ux, My - H * Uy)]


def _sample_arc_sense(
    center: XY,
    start_pt: XY,
    end_pt: XY,
    side: float,
    radius: float,
    sample_m: float,
) -> List[XY]:
    """Sample an arc from start_pt to end_pt around center in turn sense ``side`` (+1=CCW)."""
    A0 = math.atan2(start_pt[1] - center[1], start_pt[0] - center[0])
    A1 = math.atan2(end_pt[1] - center[1], end_pt[0] - center[0])
    if side >= 0.0:
        Da = _mod2pi(A1 - A0)
    else:
        Da = _mod2pi(A0 - A1)
    if Da < 1e-9:
        return []
    Step = max(sample_m, radius / 8.0)
    N = max(1, int(math.ceil(radius * Da / Step)))
    Out: List[XY] = []
    for I in range(1, N + 1):
        A = A0 + side * Da * (I / N)
        Out.append((center[0] + radius * math.cos(A), center[1] + radius * math.sin(A)))
    return Out


def _dubins_mod2pi(theta: float) -> float:
    """Wrap angle into [0, 2π) using floor (Walker Dubins convention)."""
    return theta - 2.0 * math.pi * math.floor(theta / (2.0 * math.pi))


def _dubins_segment_step(t: float, qi: Tuple[float, float, float], typ: str) -> Tuple[float, float, float]:
    """Advance a normalized Dubins segment (L/R/S) from pose ``qi`` by length ``t``."""
    St = math.sin(qi[2])
    Ct = math.cos(qi[2])
    if typ == "L":
        return (qi[0] + math.sin(qi[2] + t) - St,
                qi[1] - math.cos(qi[2] + t) + Ct,
                qi[2] + t)
    if typ == "R":
        return (qi[0] - math.sin(qi[2] - t) + St,
                qi[1] + math.cos(qi[2] - t) - Ct,
                qi[2] - t)
    return (qi[0] + Ct * t, qi[1] + St * t, qi[2])


_DUBINS_DIR = {
    "LSL": ("L", "S", "L"),
    "LSR": ("L", "S", "R"),
    "RSL": ("R", "S", "L"),
    "RSR": ("R", "S", "R"),
    "RLR": ("R", "L", "R"),
    "LRL": ("L", "R", "L"),
}


def _dubins_words(
    alpha: float,
    beta: float,
    d: float,
    sa: float,
    sb: float,
    ca: float,
    cb: float,
    cab: float,
    dsq: float,
) -> dict:
    """Normalized segment lengths for each Dubins word (Andrew Walker formulas)."""
    Out: dict = {}
    Tmp0 = d + sa - sb
    Psq = 2.0 + dsq - (2.0 * cab) + (2.0 * d * (sa - sb))
    if Psq >= 0.0:
        Tmp1 = math.atan2((cb - ca), Tmp0)
        Out["LSL"] = (_dubins_mod2pi(Tmp1 - alpha), math.sqrt(Psq), _dubins_mod2pi(beta - Tmp1))
    Tmp0 = d - sa + sb
    Psq = 2.0 + dsq - (2.0 * cab) + (2.0 * d * (sb - sa))
    if Psq >= 0.0:
        Tmp1 = math.atan2((ca - cb), Tmp0)
        Out["RSR"] = (_dubins_mod2pi(alpha - Tmp1), math.sqrt(Psq), _dubins_mod2pi(Tmp1 - beta))
    Psq = -2.0 + dsq + (2.0 * cab) + (2.0 * d * (sa + sb))
    if Psq >= 0.0:
        P = math.sqrt(Psq)
        Tmp0 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, P)
        Out["LSR"] = (_dubins_mod2pi(Tmp0 - alpha), P, _dubins_mod2pi(Tmp0 - _dubins_mod2pi(beta)))
    Psq = -2.0 + dsq + (2.0 * cab) - (2.0 * d * (sa + sb))
    if Psq >= 0.0:
        P = math.sqrt(Psq)
        Tmp0 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, P)
        Out["RSL"] = (_dubins_mod2pi(alpha - Tmp0), P, _dubins_mod2pi(beta - Tmp0))
    Tmp0 = (6.0 - dsq + (2.0 * cab) + (2.0 * d * (sa - sb))) / 8.0
    Phi = math.atan2(ca - cb, d - sa + sb)
    if abs(Tmp0) <= 1.0:
        P = _dubins_mod2pi((2.0 * math.pi) - math.acos(Tmp0))
        T = _dubins_mod2pi(alpha - Phi + _dubins_mod2pi(P / 2.0))
        Out["RLR"] = (T, P, _dubins_mod2pi(alpha - beta - T + _dubins_mod2pi(P)))
    Tmp0 = (6.0 - dsq + (2.0 * cab) + (2.0 * d * (sb - sa))) / 8.0
    Phi = math.atan2(ca - cb, d + sa - sb)
    if abs(Tmp0) <= 1.0:
        P = _dubins_mod2pi((2.0 * math.pi) - math.acos(Tmp0))
        T = _dubins_mod2pi(-alpha - Phi + (P / 2.0))
        Out["LRL"] = (T, P, _dubins_mod2pi(_dubins_mod2pi(beta) - alpha - T + _dubins_mod2pi(P)))
    return Out


def _dubins_sample(
    q0: Tuple[float, float, float],
    rho: float,
    typ: str,
    params: Tuple[float, float, float],
    step_m: float,
) -> List[XY]:
    """Sample a Dubins path; returns intermediate points (excludes exact start)."""
    Types = _DUBINS_DIR[typ]
    Length = (params[0] + params[1] + params[2]) * rho
    if Length < 1e-9:
        return []
    # Dense enough that sampled triples on arc/straight junctions stay near R.
    Step = min(max(step_m, 1e-6), max(rho / 12.0, 0.25))
    N = max(24, int(math.ceil(Length / Step)))
    Pts: List[XY] = []
    Qi = (0.0, 0.0, q0[2])
    for I in range(1, N + 1):
        Tp = (Length * (I / N)) / rho
        Q = Qi
        Rem = Tp
        for P, Name in zip(params, Types):
            if Rem <= (P + 1e-12):
                Q = _dubins_segment_step(Rem, Q, Name)
                break
            Q = _dubins_segment_step(P, Q, Name)
            Rem -= P
        Pts.append((Q[0] * rho + q0[0], Q[1] * rho + q0[1]))
    return Pts


def _dubins_shortest(
    sx: float,
    sy: float,
    stheta: float,
    ex: float,
    ey: float,
    etheta: float,
    radius: float,
    sample_m: float,
    ccc_only: bool = False,
    csc_only: bool = False,
) -> Optional[List[XY]]:
    """Shortest Dubins path (Walker); optional CCC-only or CSC-only word filters."""
    if radius <= 1e-9:
        return []
    Dx = ex - sx
    Dy = ey - sy
    D = math.hypot(Dx, Dy)
    DistN = D / radius
    Theta = _dubins_mod2pi(math.atan2(Dy, Dx)) if DistN > 0.0 else 0.0
    Alpha = _dubins_mod2pi(stheta - Theta)
    Beta = _dubins_mod2pi(etheta - Theta)
    Sa, Sb, Ca, Cb = math.sin(Alpha), math.sin(Beta), math.cos(Alpha), math.cos(Beta)
    Cab = math.cos(Alpha - Beta)
    Words = _dubins_words(Alpha, Beta, DistN, Sa, Sb, Ca, Cb, Cab, DistN * DistN)
    if ccc_only:
        Words = {K: V for K, V in Words.items() if K in ("LRL", "RLR")}
    elif csc_only:
        # Soft pinches use same-side CSC (LSL/RSR) only — LSR/RSL can loop ~2πR.
        Words = {K: V for K, V in Words.items() if K in ("LSL", "RSR")}
    if not Words:
        return None
    Typ, Params = min(Words.items(), key=lambda KV: KV[1][0] + KV[1][1] + KV[1][2])
    Pts = _dubins_sample((sx, sy, stheta), radius, Typ, Params, sample_m)
    if Pts and math.dist(Pts[-1], (ex, ey)) < 1e-6:
        Pts = Pts[:-1]
    return Pts


def _keyhole_u_turn(
    start: XY,
    heading_in_deg: float,
    end: XY,
    heading_out_deg: float,
    turn_radius_m: float,
    sample_m: float,
) -> Optional[List[XY]]:
    """Agricultural keyhole/bulb: shortest Dubins CCC (LRL/RLR) for a ~180 deg reverse."""
    St = _compass_to_math(heading_in_deg)
    Et = _compass_to_math(heading_out_deg)
    return _dubins_shortest(
        start[0], start[1], St, end[0], end[1], Et, turn_radius_m, sample_m, ccc_only=True
    )


def _dubins_csc(
    sx: float,
    sy: float,
    stheta: float,
    ex: float,
    ey: float,
    etheta: float,
    radius: float,
) -> Optional[List[XY]]:
    """Shortest Dubins CSC path for non-keyhole soft bends."""
    return _dubins_shortest(
        sx, sy, stheta, ex, ey, etheta, radius, max(radius / 8.0, 0.5), csc_only=True
    )


_SOFT_BEND_MAX_HALF_TURNS = 1.0


def _polyline_length_m(start: XY, pts: List[XY]) -> float:
    """Polyline length in metres including the start→first leg."""
    Len = 0.0
    Prev = start
    for P in pts:
        Len += math.dist(Prev, P)
        Prev = P
    return Len


def _sample_straight(p0: XY, p1: XY, step: float) -> List[XY]:
    """Samples a straight segment (excluding the start point)."""
    D = math.dist(p0, p1)
    if D < 1e-9:
        return []
    N = max(1, int(math.ceil(D / max(step, 1e-6))))
    return [
        (p0[0] + (p1[0] - p0[0]) * (I / N), p0[1] + (p1[1] - p0[1]) * (I / N))
        for I in range(1, N + 1)
    ]


def _classic_corner_fillet(
    corner: XY,
    heading_in_deg: float,
    end: XY,
    heading_out_deg: float,
    turn_radius_m: float,
    sample_m: float,
    inbound_len_m: float,
) -> Optional[Tuple[XY, List[XY]]]:
    """Classic R fillet tangent to inbound/outbound. Returns (tangent_in, move_pts) or None."""
    U = _forward_en(heading_in_deg)
    V = _forward_en(heading_out_deg)
    Cross = U[0] * V[1] - U[1] * V[0]
    Dot = max(-1.0, min(1.0, U[0] * V[0] + U[1] * V[1]))
    Phi = math.atan2(Cross, Dot)
    if abs(Phi) < 1e-3:
        return None
    Half = abs(Phi) * 0.5
    TanHalf = math.tan(Half)
    if TanHalf < 1e-9:
        return None
    TangLen = turn_radius_m * TanHalf
    if inbound_len_m < TangLen - 1e-6 or math.dist(corner, end) < TangLen - 1e-6:
        return None
    TangentIn = (corner[0] - TangLen * U[0], corner[1] - TangLen * U[1])
    TangentOut = (corner[0] + TangLen * V[0], corner[1] + TangLen * V[1])
    Left = Phi > 0.0
    Rx, Ry = (-U[1], U[0]) if Left else (U[1], -U[0])
    Center = (TangentIn[0] + turn_radius_m * Rx, TangentIn[1] + turn_radius_m * Ry)
    A0 = math.atan2(TangentIn[1] - Center[1], TangentIn[0] - Center[0])
    A1 = math.atan2(TangentOut[1] - Center[1], TangentOut[0] - Center[0])
    Sweep = A1 - A0
    if Left:
        while Sweep < 0.0:
            Sweep += 2.0 * math.pi
    else:
        while Sweep > 0.0:
            Sweep -= 2.0 * math.pi
    ArcLen = abs(Sweep) * turn_radius_m
    Step = max(sample_m, 1e-6)
    N = max(4, int(math.ceil(ArcLen / Step)))
    Pts: List[XY] = []
    for I in range(1, N + 1):
        T = I / N
        A = A0 + T * Sweep
        Pts.append((Center[0] + turn_radius_m * math.cos(A), Center[1] + turn_radius_m * math.sin(A)))
    if math.dist(TangentOut, end) > 1e-6:
        Pts.extend(_sample_straight(TangentOut, end, Step))
    if Pts and math.dist(Pts[-1], end) < 1e-6:
        Pts = Pts[:-1]
    return (TangentIn, Pts)


def _soft_bend_fillet(
    start: XY,
    heading_in_deg: float,
    end: XY,
    heading_out_deg: float,
    turn_radius_m: float,
    sample_m: float,
) -> List[XY]:
    """Last-resort same-side CSC (no CCC); empty when no word applies (never a chord)."""
    St = _compass_to_math(heading_in_deg)
    Et = _compass_to_math(heading_out_deg)
    Pts = _dubins_shortest(
        start[0], start[1], St, end[0], end[1], Et, turn_radius_m, sample_m, csc_only=True
    )
    return Pts or []


def _headland_move_points(
    start: XY,
    heading_in_deg: float,
    end: XY,
    heading_out_deg: float,
    turn_radius_m: float,
    sample_m: float,
) -> List[XY]:
    """R-bounded blade-up path; CCC keyhole for ~180 deg U-turns, else short CSC/fillet."""
    U = _forward_en(heading_in_deg)
    V = _forward_en(heading_out_deg)
    St = _compass_to_math(heading_in_deg)
    Et = _compass_to_math(heading_out_deg)
    # Near-reverse => CCC keyhole bulb (54°+~289°+54° style), not CSC 270°+270°.
    if (U[0] * V[0] + U[1] * V[1]) < -0.5:
        Key = _dubins_shortest(
            start[0], start[1], St, end[0], end[1], Et, turn_radius_m, sample_m, ccc_only=True
        )
        if Key:
            return Key
        # Never collapse a U-turn to a straight chord — try any Dubins word.
        Any = _dubins_shortest(
            start[0], start[1], St, end[0], end[1], Et, turn_radius_m, sample_m
        )
        if Any:
            return Any

    # Soft bend: same-side CSC (LSL/RSR); SoftBendFillet is last resort.
    Pts = _dubins_shortest(
        start[0], start[1], St, end[0], end[1], Et, turn_radius_m, sample_m, csc_only=True
    )
    if Pts:
        Chord = math.dist(start, end)
        Cap = (_SOFT_BEND_MAX_HALF_TURNS * math.pi * turn_radius_m) + Chord
        if _polyline_length_m(start, Pts) <= Cap:
            return Pts
    return _soft_bend_fillet(
        start, heading_in_deg, end, heading_out_deg, turn_radius_m, sample_m
    )


def _find_uturn_exit(
    run: List[Tuple[float, float, float, str, int, float, float]],
    corner_k: int,
    turn_radius_m: float,
) -> Optional[Tuple[int, float]]:
    """If ``corner_k`` starts a tight serpentine U-turn, return ``(end_k, hout_deg)``.

    Searches within 4R for the first reverse segment and coalesces the whole
    reverse window (W), not only the immediate next cell.
    """
    if corner_k < 1 or corner_k >= len(run) - 1:
        return None
    A = (run[corner_k - 1][5], run[corner_k - 1][6])
    B = (run[corner_k][5], run[corner_k][6])
    Hin = _bearing_deg(A, B)
    U = _forward_en(Hin)
    Along = 0.0
    MaxAlong = 4.0 * turn_radius_m
    for M in range(corner_k, len(run) - 1):
        P0 = (run[M][5], run[M][6])
        P1 = (run[M + 1][5], run[M + 1][6])
        if M > corner_k:
            PrevPt = (run[M - 1][5], run[M - 1][6])
            Along += math.dist(PrevPt, P0)
            if Along > MaxAlong:
                break
        V = _forward_en(_bearing_deg(P0, P1))
        if (U[0] * V[0] + U[1] * V[1]) < -0.5:
            return (M, _bearing_deg(P0, P1))
    return None


def _steer_violates_min_radius(
    a: XY,
    b: XY,
    c: XY,
    turn_radius_m: float,
    turn_threshold_deg: float,
) -> bool:
    """True when the bend at b is tighter than the turning radius.

    Hard R bound applies to every real bend; ``turn_threshold_deg`` is only a
    noise floor (clamped to at least 15 deg) so micro-wiggles are ignored.
    """
    # Intent: 40–60 deg pinches with r≪R must still get headlands (route-7 W).
    NoiseFloor = min(15.0, turn_threshold_deg)
    if _heading_change_deg(a, b, c) < NoiseFloor:
        return False
    return _circumradius_m(a, b, c) < (turn_radius_m * 0.99)


def _enforce_min_turn_radius(
    raw: List[Tuple[float, float, float, str, int, float, float]],
    turn_radius_m: float,
    turn_threshold_deg: float,
    sample_m: float,
) -> List[Tuple[float, float, float, str, int, float, float]]:
    """Insert R-radius MOVE headlands wherever a blade-down corner is too sharp.

    Coverage points keep their raw coordinates for distance accounting; only the
    steering polyline is rewritten. Newly inserted arc samples are MOVE (blade up).

    Serpentine U-turns are coalesced into one keyhole so the track stays a single
    readable bulb instead of stacked Dubins loops.
    """
    if turn_radius_m <= 0.0 or len(raw) < 3:
        return raw

    blade_down = {CUT, FILL, GRADE}
    out: List[Tuple[float, float, float, str, int, float, float]] = []
    i = 0
    n = len(raw)
    while i < n:
        x, y, d, action, pass_no, xs, ys = raw[i]
        if action not in blade_down:
            out.append(raw[i])
            i += 1
            continue

        # Contiguous blade-down run of one action (CUT / FILL / GRADE).
        j = i
        while j + 1 < n and raw[j + 1][3] == action:
            j += 1
        run = raw[i:j + 1]
        out.append(run[0])
        skip_turn: set[int] = set()
        coalesce_emitted: set[int] = set()
        for k in range(1, len(run) - 1):
            prev = run[k - 1]
            cur = run[k]
            nxt = run[k + 1]
            a = (prev[5], prev[6])
            b = (cur[5], cur[6])
            c = (nxt[5], nxt[6])
            if k in skip_turn:
                if k not in coalesce_emitted:
                    out.append(cur)
                continue
            out.append(cur)
            if not _steer_violates_min_radius(a, b, c, turn_radius_m, turn_threshold_deg):
                continue

            hin = _bearing_deg(a, b)
            uturn = _find_uturn_exit(run, k, turn_radius_m)
            if uturn is not None:
                end_k, hout = uturn
                end_pt = (run[end_k][5], run[end_k][6])
                KeyMoves = _headland_move_points(
                    b, hin, end_pt, hout, turn_radius_m, sample_m
                )
                # Never SkipTurn a reverse window if the keyhole builder produced nothing.
                if KeyMoves or math.dist(b, end_pt) < 1e-3:
                    for mx, my in KeyMoves:
                        out.append((mx, my, 0.0, MOVE, cur[4], mx, my))
                    for s in range(k + 1, end_k):
                        Mid = run[s]
                        out.append((Mid[0], Mid[1], Mid[2], Mid[3], Mid[4], end_pt[0], end_pt[1]))
                        skip_turn.add(s)
                        coalesce_emitted.add(s)
                    skip_turn.add(end_k)
                    continue

            hout = _bearing_deg(b, c)
            Fillet = _classic_corner_fillet(
                b, hin, c, hout, turn_radius_m, sample_m, math.dist(a, b)
            )
            if Fillet is not None:
                TangentIn, ArcPts = Fillet
                Last = out[-1]
                out[-1] = (Last[0], Last[1], Last[2], Last[3], Last[4], TangentIn[0], TangentIn[1])
                for mx, my in ArcPts:
                    out.append((mx, my, 0.0, MOVE, cur[4], mx, my))
            elif _heading_change_deg(a, b, c) >= 60.0:
                Moves = _headland_move_points(
                    b, hin, c, hout, turn_radius_m, sample_m
                )
                Cap = 2.75 * math.pi * turn_radius_m
                if (not Moves) or _polyline_length_m(b, Moves) > Cap:
                    St = _compass_to_math(hin)
                    Et = _compass_to_math(hout)
                    Moves = _dubins_shortest(
                        b[0], b[1], St, c[0], c[1], Et, turn_radius_m, sample_m
                    ) or []
                if Moves and _polyline_length_m(b, Moves) <= Cap:
                    for mx, my in Moves:
                        out.append((mx, my, 0.0, MOVE, cur[4], mx, my))
        if len(run) > 1:
            out.append(run[-1])
        i = j + 1
    return _repair_sharp_steering_bends(out, turn_radius_m, sample_m)


def _repair_sharp_steering_bends(
    raw: List[Tuple[float, float, float, str, int, float, float]],
    turn_radius_m: float,
    sample_m: float,
) -> List[Tuple[float, float, float, str, int, float, float]]:
    """Replace residual sharp splices with short R-bounded arcs (no CCC cascade).

    Blade-down apexes are kept (coverage floor); only blade-up residual
    junctions may drop the apex and splice A→C. Interior MOVE samples of an
    already-inserted headland are left alone.
    """
    if turn_radius_m <= 0.0 or len(raw) < 3:
        return raw
    blade_down = {CUT, FILL, GRADE}
    Cur = raw
    for _sweep in range(2):
        Next = [Cur[0]]
        Changed = False
        I = 1
        while I < len(Cur) - 1:
            A = Next[-1]
            B = Cur[I]
            C = Cur[I + 1]
            Pa, Pb, Pc = (A[5], A[6]), (B[5], B[6]), (C[5], C[6])
            # Do not treat samples on an existing R-arc as new soft pinches.
            if A[3] == MOVE and B[3] == MOVE and C[3] == MOVE:
                Next.append(B)
                I += 1
                continue
            if not _steer_violates_min_radius(Pa, Pb, Pc, turn_radius_m, 15.0):
                Next.append(B)
                I += 1
                continue
            Changed = True
            if B[3] in blade_down and A[3] == MOVE:
                # Keyhole exit: classic fillet or short CSC only — never a new CCC bulb.
                Hin = _bearing_deg(Pa, Pb)
                Hout = _bearing_deg(Pb, Pc)
                Fillet = _classic_corner_fillet(
                    Pb, Hin, Pc, Hout, turn_radius_m, sample_m, math.dist(Pa, Pb)
                )
                if Fillet is not None:
                    TangentIn, ArcPts = Fillet
                    Next.append((B[0], B[1], B[2], B[3], B[4], TangentIn[0], TangentIn[1]))
                    for Mx, My in ArcPts:
                        Next.append((Mx, My, 0.0, MOVE, B[4], Mx, My))
                else:
                    St = _compass_to_math(Hin)
                    Et = _compass_to_math(Hout)
                    Short = _dubins_shortest(
                        Pa[0], Pa[1], St, Pc[0], Pc[1], Et, turn_radius_m, sample_m, csc_only=True
                    )
                    Cap = (_SOFT_BEND_MAX_HALF_TURNS * math.pi * turn_radius_m) + math.dist(Pa, Pc)
                    if Short and _polyline_length_m(Pa, Short) <= Cap:
                        for Mx, My in Short:
                            Next.append((Mx, My, 0.0, MOVE, B[4], Mx, My))
                    Next.append(B)
                Next.append(C)
                I += 2
                continue
            if B[3] in blade_down:
                # Soft CUT pinches: classic R fillet when it fits; else Dubins only for ≥90°.
                Next.append(B)
                Hin = _bearing_deg(Pa, Pb)
                Hout = _bearing_deg(Pb, Pc)
                Fillet = _classic_corner_fillet(
                    Pb, Hin, Pc, Hout, turn_radius_m, sample_m, math.dist(Pa, Pb)
                )
                if Fillet is not None:
                    TangentIn, ArcPts = Fillet
                    Last = Next[-1]
                    Next[-1] = (Last[0], Last[1], Last[2], Last[3], Last[4], TangentIn[0], TangentIn[1])
                    for Mx, My in ArcPts:
                        Next.append((Mx, My, 0.0, MOVE, B[4], Mx, My))
                elif _heading_change_deg(Pa, Pb, Pc) >= 60.0:
                    Moves = _headland_move_points(
                        Pb, Hin, Pc, Hout, turn_radius_m, sample_m
                    )
                    Cap = 2.75 * math.pi * turn_radius_m
                    if Moves and _polyline_length_m(Pb, Moves) <= Cap:
                        for Mx, My in Moves:
                            Next.append((Mx, My, 0.0, MOVE, B[4], Mx, My))
                Next.append(C)
                I += 2
                continue
            # Blade-up residual (e.g. CUT→MOVE→CUT): short CSC only.
            Hin = _bearing_deg(Pa, Pb)
            Hout = _bearing_deg(Pb, Pc)
            St = _compass_to_math(Hin)
            Et = _compass_to_math(Hout)
            Short = _dubins_shortest(
                Pa[0], Pa[1], St, Pc[0], Pc[1], Et, turn_radius_m, sample_m, csc_only=True
            )
            Cap = (_SOFT_BEND_MAX_HALF_TURNS * math.pi * turn_radius_m) + math.dist(Pa, Pc)
            if Short and _polyline_length_m(Pa, Short) <= Cap:
                for Mx, My in Short:
                    Next.append((Mx, My, 0.0, MOVE, B[4], Mx, My))
            Next.append(C)
            I += 2
        if I < len(Cur):
            Next.append(Cur[-1])
        Cur = Next
        if not Changed:
            break
    return Cur


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


def _nn_order(
    cents: List[XY],
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> List[int]:
    """Greedy nearest-neighbour visiting order over fragment centroids.

    The matcher builds a multi-pocket load as a short-hop chain, but the lane
    builders used to re-sort fragments purely by distance to the haul junction,
    which can place two far-apart fragments next to each other and force a long
    blade-up reposition that covers nothing. This instead chains fragments by
    nearest neighbour so each inter-fragment hop is the shortest available, while
    still anchoring the sweep to the haul junction: the **cut** sweep (``orient_
    end_xy``) finishes at the fragment nearest the fill; the **fill** sweep
    (``orient_start_xy``) starts at the fragment nearest the haul arrival.
    """
    n = len(cents)
    if n <= 1:
        return list(range(n))
    remaining = set(range(n))
    if orient_end_xy is not None:
        # End-anchored: fix the last fragment (nearest the fill) and grow the
        # chain backwards by nearest neighbour from the current front.
        last = min(remaining, key=lambda i: _d2(cents[i], orient_end_xy))
        chain = [last]
        remaining.discard(last)
        while remaining:
            front = chain[0]
            nxt = min(remaining, key=lambda i: _d2(cents[i], cents[front]))
            chain.insert(0, nxt)
            remaining.discard(nxt)
        return chain
    src = orient_start_xy if orient_start_xy is not None else cents[0]
    first = min(remaining, key=lambda i: _d2(cents[i], src))
    chain = [first]
    remaining.discard(first)
    while remaining:
        back = chain[-1]
        nxt = min(remaining, key=lambda i: _d2(cents[i], cents[back]))
        chain.append(nxt)
        remaining.discard(nxt)
    return chain


def _ordered_runs(cell_list: List[Tuple[Cell, float]]) -> List[List[Tuple[Cell, float]]]:
    """Split a flow strip into maximal runs of consecutive 8-adjacent cells.

    The flow-tube grower returns cells in true streamline order (one flowing run
    per pocket); a multi-drop hop shows up as a break to a non-adjacent cell.
    Splitting on those breaks - while *preserving the order within each run* -
    gives long flowing lanes with no nearest-neighbour doubling back.
    """
    runs: List[List[Tuple[Cell, float]]] = []
    cur: List[Tuple[Cell, float]] = []
    prev: Optional[Cell] = None
    for c, d in cell_list:
        if prev is None or (abs(c[0] - prev[0]) <= 1 and abs(c[1] - prev[1]) <= 1):
            cur.append((c, d))
        else:
            runs.append(cur)
            cur = [(c, d)]
        prev = c
    if cur:
        runs.append(cur)
    return runs


def _lanes_flow(
    cell_list: List[Tuple[Cell, float]],
    grid: Grid,
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> List[Tuple[float, float, float]]:
    """Flow-following lane builder: drive each flow run along its streamline order.

    Cells arrive already in streamline order; this keeps that order (no
    re-chaining) and only flips whole runs so the cut side finishes at the run
    nearest the fill (``orient_end_xy``) and the fill side starts at the run
    nearest the haul arrival (``orient_start_xy``). The result is long, flowing
    cut/fill lanes that curve with the flow.
    """
    runs = _ordered_runs(cell_list)
    segs: List[List[Tuple[float, float, float]]] = []
    for r in runs:
        if r:
            segs.append([(grid.cell_center_xy(*c)[0], grid.cell_center_xy(*c)[1], d)
                         for c, d in r])
    if not segs:
        return []
    cents = [_centroid([(p[0], p[1]) for p in s]) for s in segs]

    pts: List[Tuple[float, float, float]] = []
    if orient_end_xy is not None:
        order = _nn_order(cents, orient_end_xy=orient_end_xy)
        for rank, i in enumerate(order):
            tgt = orient_end_xy if rank == len(order) - 1 else cents[order[rank + 1]]
            s = segs[i]
            if len(s) >= 2 and _d2((s[0][0], s[0][1]), tgt) < _d2((s[-1][0], s[-1][1]), tgt):
                s = list(reversed(s))
            pts += s
    else:
        src = orient_start_xy if orient_start_xy is not None else cents[0]
        order = _nn_order(cents, orient_start_xy=orient_start_xy)
        for rank, i in enumerate(order):
            st = src if rank == 0 else cents[order[rank - 1]]
            s = segs[i]
            if len(s) >= 2 and _d2((s[-1][0], s[-1][1]), st) < _d2((s[0][0], s[0][1]), st):
                s = list(reversed(s))
            pts += s
    return pts


def _lane_direct(
    cell_list: List[Tuple[Cell, float]],
    grid: Grid,
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> List[Tuple[float, float, float]]:
    """Sweep cells in their stored order (knot-reduced patches only).

    The matcher already threaded a single bounded-curvature lane through these
    cells, so the list *is* the drivable route. We must not re-chain it (that is
    what re-tangles it); we only flip the whole sweep to keep the haul junction.
    """
    pts = [(grid.cell_center_xy(*c)[0], grid.cell_center_xy(*c)[1], d)
           for c, d in cell_list]
    if len(pts) >= 2:
        if orient_end_xy is not None:
            if _d2((pts[0][0], pts[0][1]), orient_end_xy) < _d2((pts[-1][0], pts[-1][1]), orient_end_xy):
                pts.reverse()
        elif orient_start_xy is not None:
            if _d2((pts[-1][0], pts[-1][1]), orient_start_xy) < _d2((pts[0][0], pts[0][1]), orient_start_xy):
                pts.reverse()
    return pts


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
        # Cut side: chain pockets by nearest neighbour, finishing nearest the fill.
        order = _nn_order(cents, orient_end_xy=orient_end_xy)
        for rank, i in enumerate(order):
            tgt = orient_end_xy if rank == len(order) - 1 else cents[order[rank + 1]]
            pts += _lanes_for_patch(pockets[i], grid, blade_m, heading_deg, overlap,
                                    orient_end_xy=tgt)
    else:
        # Fill side: start nearest the haul arrival, then chain by nearest neighbour.
        src = orient_start_xy if orient_start_xy is not None else cents[0]
        order = _nn_order(cents, orient_start_xy=orient_start_xy)
        for rank, i in enumerate(order):
            st = src if rank == 0 else cents[order[rank - 1]]
            pts += _lanes_for_patch(pockets[i], grid, blade_m, heading_deg, overlap,
                                    orient_start_xy=st)
    return pts


def _repair_if_tangled(
    lane: List[Tuple[float, float, float]],
    cells: List[Tuple[Cell, float]],
    grid: Grid,
    blade_m: float,
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> Tuple[List[Tuple[float, float, float]], int]:
    """Find-and-fix pass for a non-drivable sweep.

    When a compact "blob" patch is chained by nearest-run order the sweep folds
    back over itself into an undriveable knot (many hairpin reversals in a tiny
    area). This re-tiles the same cells as a clean serpentine of blade-width lanes
    along the patch's principal axis (``_lanes_multi`` with the PCA heading),
    oriented to the same haul junction, and keeps it only if it **strictly
    reduces** the hairpin-turn count (so the repair can never make a sweep worse;
    genuine flowing streamlines, which already score ~0, are left untouched).
    Returns ``(lane, 1)`` if repaired, else ``(lane, 0)``.
    """
    c0 = _hard_turn_count(lane, grid, blade_m)
    if c0 == 0:
        return lane, 0
    centers = [grid.cell_center_xy(*c) for c, _d in cells]
    pca_h = _principal_heading_deg(centers)
    alt = _lanes_multi(cells, grid, blade_m, pca_h, 0.0,
                       orient_start_xy=orient_start_xy, orient_end_xy=orient_end_xy)
    if alt and _hard_turn_count(alt, grid, blade_m) < c0:
        return alt, 1
    return lane, 0


def _patch_drivable_lanes(
    cells: List[Tuple[Cell, float]],
    grid: Grid,
    blade_m: float,
) -> List[List[Tuple[Cell, float]]]:
    """Decompose a patch into its parallel blade-width lanes (drivable units).

    The cells are projected onto the patch's principal axis (:func:`_principal_
    heading_deg`) and binned into blade-wide bands across it. Each band, swept
    end-to-end along the long axis, is one straight (gently flowing) lane a
    scraper can drive without a hairpin. Returned in cross-axis band order, each
    lane's cells sorted along the axis. This is the natural decomposition that
    turns a knotted serpentine (whose U-turns are the knot) into a set of
    individually drivable lanes.
    """
    if len(cells) <= 1:
        return [list(cells)]
    centers = [grid.cell_center_xy(*c) for c, _d in cells]
    h = _principal_heading_deg(centers)
    th = math.radians(h)
    uh = (math.sin(th), math.cos(th))     # along the long axis
    vh = (math.cos(th), -math.sin(th))    # across it (lane-to-lane)
    ox, oy = _centroid(centers)
    us = [(x - ox) * uh[0] + (y - oy) * uh[1] for x, y in centers]
    vs = [(x - ox) * vh[0] + (y - oy) * vh[1] for x, y in centers]
    step = max(blade_m, 1e-3)
    v_min, v_max = min(vs), max(vs)
    n_lanes = max(1, int(math.floor((v_max - v_min) / step + 1e-9)) + 1)
    lane_v = [v_min + step * k for k in range(n_lanes)]
    if v_max - lane_v[-1] > 0.5 * step:
        lane_v.append(v_max)
    groups: List[List[int]] = [[] for _ in lane_v]
    for i in range(len(cells)):
        k = min(range(len(lane_v)), key=lambda L: abs(vs[i] - lane_v[L]))
        groups[k].append(i)
    lanes: List[List[Tuple[Cell, float]]] = []
    for members in groups:
        if not members:
            continue
        members.sort(key=lambda i: us[i])
        lanes.append([cells[i] for i in members])
    return lanes


def _boustrophedon_one(
    cells: List[Tuple[Cell, float]],
    grid: Grid,
    blade_m: float,
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> List[Tuple[Cell, float]]:
    """Boustrophedon a single connected pocket (parallel back-and-forth lanes).

    The pocket is decomposed into its parallel blade-width lanes along the
    principal axis (:func:`_patch_drivable_lanes`) and the lanes are walked in
    serpentine order - every other lane reversed - so the machine drives one
    straight lane, makes a single turn at the headland, and drives the next lane
    back. The whole sweep is flipped so it starts/ends at the haul junction.
    """
    lanes = _patch_drivable_lanes(cells, grid, blade_m)
    out: List[Tuple[Cell, float]] = []
    for j, lane in enumerate(lanes):
        out += list(reversed(lane)) if (j % 2) else list(lane)
    if len(out) >= 2:
        c0 = grid.cell_center_xy(*out[0][0])
        c1 = grid.cell_center_xy(*out[-1][0])
        if orient_end_xy is not None:
            if _d2(c0, orient_end_xy) < _d2(c1, orient_end_xy):
                out.reverse()
        elif orient_start_xy is not None:
            if _d2(c1, orient_start_xy) < _d2(c0, orient_start_xy):
                out.reverse()
    return out


def _boustrophedon_cells(
    cells: List[Tuple[Cell, float]],
    grid: Grid,
    blade_m: float,
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> List[Tuple[Cell, float]]:
    """Re-order a patch's cells as a boustrophedon, **pocket by pocket**.

    A single load may span several disjoint pockets (a multi-drop cut/fill).
    Binning the whole spread as one lane field would stitch distant pockets
    together with long blade-up jumps, so - exactly like :func:`_lanes_multi` -
    the patch is split into connected pockets, each pocket is boustrophedon'd
    locally (:func:`_boustrophedon_one`), and the pockets are ordered to finish
    nearest the fill (``orient_end_xy``, cut side) or start nearest the haul
    arrival (``orient_start_xy``, fill side) so inter-pocket repositions stay
    minimal. Every original cell is kept, so the soil routing and simulated
    surface are unchanged - only the path through the patch changes.
    """
    pockets = _connected_pockets(cells)
    if len(pockets) <= 1:
        return _boustrophedon_one(cells, grid, blade_m,
                                  orient_start_xy=orient_start_xy,
                                  orient_end_xy=orient_end_xy)
    cents = [_centroid([grid.cell_center_xy(*c) for c, _d in pk]) for pk in pockets]
    out: List[Tuple[Cell, float]] = []
    if orient_end_xy is not None:
        order = _nn_order(cents, orient_end_xy=orient_end_xy)
        for rank, i in enumerate(order):
            tgt = orient_end_xy if rank == len(order) - 1 else cents[order[rank + 1]]
            out += _boustrophedon_one(pockets[i], grid, blade_m, orient_end_xy=tgt)
    else:
        src = orient_start_xy if orient_start_xy is not None else cents[0]
        order = _nn_order(cents, orient_start_xy=orient_start_xy)
        for rank, i in enumerate(order):
            st = src if rank == 0 else cents[order[rank - 1]]
            out += _boustrophedon_one(pockets[i], grid, blade_m, orient_start_xy=st)
    return out


def _rendered_lane(
    cells: List[Tuple[Cell, float]],
    grid: Grid,
    blade_m: float,
    flow_pass: bool,
    heading_deg: float,
    overlap: float,
    orient_start_xy: Optional[XY] = None,
    orient_end_xy: Optional[XY] = None,
) -> List[Tuple[float, float, float]]:
    """The sweep the guidance would draw today (mirrors :func:`build_guidance`).

    Flow-aligned passes follow the streamline (:func:`_lanes_flow`); single-
    heading passes are tiled into straight lanes (:func:`_lanes_multi`). Either
    way the serpentine find-and-fix (:func:`_repair_if_tangled`) runs, so this is
    exactly the path scored for drivability - not the raw, unrepaired order.
    """
    if flow_pass:
        lane = _lanes_flow(cells, grid, orient_start_xy=orient_start_xy,
                           orient_end_xy=orient_end_xy)
    else:
        lane = _lanes_multi(cells, grid, blade_m, heading_deg, overlap,
                            orient_start_xy=orient_start_xy, orient_end_xy=orient_end_xy)
    lane, _ = _repair_if_tangled(lane, cells, grid, blade_m,
                                 orient_start_xy=orient_start_xy,
                                 orient_end_xy=orient_end_xy)
    return lane


def _pass_is_knot(p: Pass, grid: Grid, blade_m: float, hairpin_min: int,
                  heading_deg: float = 0.0, overlap: float = 0.0) -> bool:
    """True when the cut or fill sweep *as the guidance renders it* still hairpins.

    Uses the rendered path (flow/lane tiling + serpentine find-and-fix), so a
    sweep the existing pipeline already drives cleanly is **not** flagged - only
    genuine, still-tangled knots are.
    """
    flow_pass = p.cut_heading_deg is not None or p.fill_heading_deg is not None
    cut_h = p.cut_heading_deg if p.cut_heading_deg is not None else heading_deg
    fill_h = p.fill_heading_deg if p.fill_heading_deg is not None else heading_deg
    if len(p.cut_cells) > 3:
        lane = _rendered_lane(p.cut_cells, grid, blade_m, flow_pass, cut_h, overlap,
                              orient_end_xy=p.fill_start_xy)
        if _hard_turn_count(lane, grid, blade_m) >= hairpin_min:
            return True
    if len(p.fill_cells) > 3:
        cut_end = grid.cell_center_xy(*p.cut_cells[-1][0]) if p.cut_cells else None
        lane = _rendered_lane(p.fill_cells, grid, blade_m, flow_pass, fill_h, overlap,
                              orient_start_xy=cut_end)
        if _hard_turn_count(lane, grid, blade_m) >= hairpin_min:
            return True
    return False


def reprofile_knots(
    route: Route,
    grid: Grid,
    blade_width_mm: float,
    hairpin_min: int = 3,
    heading_deg: float = 0.0,
    lane_overlap: float = 0.0,
) -> int:
    """Re-profile tangled knot sweeps as boustrophedon parallel passes, in place.

    The matcher emits the full plan; some compact loads chain into an
    undriveable zig-zag knot. This walks each sequenced pass and, for the cut and
    fill sweeps, compares the path the guidance renders today (flow/lane tiling +
    the serpentine find-and-fix, :func:`_rendered_lane`) against a boustrophedon
    of straight back-and-forth blade lanes (:func:`_boustrophedon_cells`). The
    boustrophedon is adopted **only when the rendered path is still a knot**
    (``>= hairpin_min`` hairpins) **and the boustrophedon strictly fewer** - so a
    sweep that already drives cleanly (e.g. a gentle flowing fill) is never made
    worse. The load, haul and every cut/fill bin are unchanged, so neither the
    soil routing nor the simulated surface moves; only the in-patch path changes.
    Returns the number of patches re-profiled.
    """
    blade_m = blade_width_mm / 1000.0
    n = 0
    for p in route.ordered_passes:
        flow_pass = p.cut_heading_deg is not None or p.fill_heading_deg is not None
        cut_h = p.cut_heading_deg if p.cut_heading_deg is not None else heading_deg
        fill_h = p.fill_heading_deg if p.fill_heading_deg is not None else heading_deg

        if len(p.cut_cells) > 3:
            rendered = _rendered_lane(p.cut_cells, grid, blade_m, flow_pass, cut_h,
                                      lane_overlap, orient_end_xy=p.fill_start_xy)
            c_now = _hard_turn_count(rendered, grid, blade_m)
            if c_now >= hairpin_min:
                bcells = _boustrophedon_cells(p.cut_cells, grid, blade_m,
                                              orient_end_xy=p.fill_start_xy)
                blane = _lane_direct(bcells, grid, orient_end_xy=p.fill_start_xy)
                if _hard_turn_count(blane, grid, blade_m) < c_now:
                    p.cut_cells = bcells
                    p.cut_knot_reduced = True
                    n += 1

        cut_end = grid.cell_center_xy(*p.cut_cells[-1][0]) if p.cut_cells else None
        if len(p.fill_cells) > 3:
            rendered = _rendered_lane(p.fill_cells, grid, blade_m, flow_pass, fill_h,
                                      lane_overlap, orient_start_xy=cut_end)
            c_now = _hard_turn_count(rendered, grid, blade_m)
            if c_now >= hairpin_min:
                bcells = _boustrophedon_cells(p.fill_cells, grid, blade_m,
                                              orient_start_xy=cut_end)
                blane = _lane_direct(bcells, grid, orient_start_xy=cut_end)
                if _hard_turn_count(blane, grid, blade_m) < c_now:
                    p.fill_cells = bcells
                    p.fill_knot_reduced = True
                    n += 1
    return n


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
    smooth_route: bool = True,
    turn_radius_m: Optional[float] = None,
    smooth_scale: float = 0.7,
    obstacles: ObstacleField = EMPTY_OBSTACLES,
    arm_obstacles: Optional[dict] = None,
) -> GuidanceResult:
    """Build the blade-scale operator guidance track from the optimized route.

    The route's passes are walked in order; each pass's cut and fill patches are
    tiled into blade-width serpentine lanes (:func:`_lanes_for_patch`). When
    ``grade_return`` is set, each fill is followed by a single return pass back
    over the fill to grade and compact it. The resulting waypoints are emitted as
    a single ordered track with per-point actions, target depths, next-bearings,
    and instructions.

    When ``smooth_route`` is set (default), the emitted waypoint *coordinates* of
    each cut/fill sweep are first Gaussian-smoothed to remove grid stair-steps.
    A second **hard** pass then enforces the machine turning radius
    (``turn_radius_m``; default half the turning circle): any blade-down corner
    tighter than R is split and joined by an R-radius blade-up ``MOVE`` headland
    arc. Distance/turn/time for CUT/FILL/GRADE stay on the raw coverage geometry
    (so sweeps are never shortened below the coverage floor); headland arcs add
    to reposition distance only. ``--no-route-smoothing`` skips the Gaussian step
    but still enforces the turning-radius bound.
    """
    blade_m = blade_width_mm / 1000.0
    zone, north = field.zone, field.is_northern

    # Multi-drop hop threshold (blade-up reposition between pockets) and the route
    # smoothing width, both constant for the job. Gaussian sigma only softens
    # stair-steps; EnforceMinTurnRadius guarantees the emitted steering line
    # never bends tighter than R.
    hop_thresh = max(3.0 * grid.grid_size_m, 2.0 * blade_m)
    if turn_radius_m is None:
        turn_radius_m = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    sigma_idx = (turn_radius_m / grid.grid_size_m) * smooth_scale if smooth_route else 0.0

    points: List[GuidancePoint] = []
    return_d = haul_d = cut_d = fill_d = grade_d = reposition_d = 0.0
    planned_cut_d = planned_fill_d = route_haul_d = 0.0
    n_repaired = 0

    prev_xy: Optional[XY] = None       # end of the previous waypoint (raw coverage)
    seq = 0

    # Raw records first; distances/bearings after. Each record carries both the
    # raw coverage point (xr, yr - used for all distance/turn/time accounting,
    # because the blade must cover the full cell band) and the smoothed steering
    # point (xs, ys - what the operator follows on the in-cab map). Smoothing only
    # changes the displayed line; it never shortens the coverage the blade does.
    raw: List[Tuple[float, float, float, str, int, float, float]] = []

    for pass_no, p in enumerate(route.ordered_passes, start=1):
        route_haul_d += p.haul_distance_m
        planned_cut_d += _seg_length([grid.cell_center_xy(*c) for c, _d in p.cut_cells])
        planned_fill_d += _seg_length([grid.cell_center_xy(*c) for c, _d in p.fill_cells])

        # Tile each patch's blade lanes along the direction the soil actually
        # moves there: the pass's local flow bearing when flow-aligned, else the
        # single global heading. This makes the lanes follow the flow (curved
        # across the field) instead of one fixed direction.
        flow_pass = p.cut_heading_deg is not None or p.fill_heading_deg is not None
        cut_h = p.cut_heading_deg if p.cut_heading_deg is not None else heading_deg
        fill_h = p.fill_heading_deg if p.fill_heading_deg is not None else heading_deg

        # Orient the cut sweep to end at the optimized haul target (where dumping
        # begins) so the per-pass haul leg stays the optimizer's transit. Flow
        # strips are driven along their (curved) spine; single-heading patches are
        # tiled into straight blade lanes.
        # A knot-reduced patch is already a single drivable lane in stored order:
        # follow it directly (no re-chaining / no re-tangle). Otherwise tile the
        # flow lane (or single-heading lanes) and run the serpentine find-and-fix.
        if getattr(p, "cut_knot_reduced", False):
            cut_lane = _lane_direct(p.cut_cells, grid, orient_end_xy=p.fill_start_xy)
        elif flow_pass:
            cut_lane = _lanes_flow(p.cut_cells, grid, orient_end_xy=p.fill_start_xy)
            cut_lane, fixed = _repair_if_tangled(cut_lane, p.cut_cells, grid, blade_m,
                                                 orient_end_xy=p.fill_start_xy)
            n_repaired += fixed
        else:
            cut_lane = _lanes_multi(p.cut_cells, grid, blade_m, cut_h, lane_overlap,
                                    orient_end_xy=p.fill_start_xy)
            cut_lane, fixed = _repair_if_tangled(cut_lane, p.cut_cells, grid, blade_m,
                                                 orient_end_xy=p.fill_start_xy)
            n_repaired += fixed
        cut_end_xy = (cut_lane[-1][0], cut_lane[-1][1]) if cut_lane else p.cut_end_xy
        if getattr(p, "fill_knot_reduced", False):
            fill_lane = _lane_direct(p.fill_cells, grid, orient_start_xy=cut_end_xy)
        elif flow_pass:
            fill_lane = _lanes_flow(p.fill_cells, grid, orient_start_xy=cut_end_xy)
            fill_lane, fixed = _repair_if_tangled(fill_lane, p.fill_cells, grid, blade_m,
                                                  orient_start_xy=cut_end_xy)
            n_repaired += fixed
        else:
            fill_lane = _lanes_multi(p.fill_cells, grid, blade_m, fill_h, lane_overlap,
                                     orient_start_xy=cut_end_xy)
            fill_lane, fixed = _repair_if_tangled(fill_lane, p.fill_cells, grid, blade_m,
                                                  orient_start_xy=cut_end_xy)
            n_repaired += fixed

        # Drivable steering line: smooth each sweep's *shape* (endpoints pinned, so
        # the haul/return junctions are untouched). This is what the operator
        # follows on the map; the raw lane below still drives all distance/time.
        cut_disp = _smooth_lane(cut_lane, sigma_idx, hop_thresh)
        fill_disp = _smooth_lane(fill_lane, sigma_idx, hop_thresh)

        # A multi-drop pass may span several pockets; the blade-up drive between
        # two pockets shows up as an unusually long step within a sweep. Emit
        # those gaps as HAUL repositions (blade up) instead of cutting/filling
        # across empty ground. Hops are detected on the raw lane.
        for i, (x, y, d) in enumerate(cut_lane):
            if i == 0:
                action = START if pass_no == 1 else RETURN
            elif math.dist((cut_lane[i - 1][0], cut_lane[i - 1][1]), (x, y)) > hop_thresh:
                action = MOVE
            else:
                action = CUT
            raw.append((x, y, d, action, pass_no, cut_disp[i][0], cut_disp[i][1]))
        for i, (x, y, d) in enumerate(fill_lane):
            if i == 0:
                action = HAUL
            elif math.dist((fill_lane[i - 1][0], fill_lane[i - 1][1]), (x, y)) > hop_thresh:
                action = MOVE
            else:
                action = FILL
            raw.append((x, y, d, action, pass_no, fill_disp[i][0], fill_disp[i][1]))
        # Grade/compact: turn at the fill end and drive back over the fill. The
        # return retraces the fill lane in reverse (blade down, no new material);
        # inter-pocket hops are blade-up repositions, not graded ground.
        if grade_return and len(fill_lane) >= 2:
            rev = list(reversed(fill_lane))
            rev_disp = list(reversed(fill_disp))
            for i, (x, y, _d) in enumerate(rev):
                if i > 0 and math.dist((rev[i - 1][0], rev[i - 1][1]), (x, y)) > hop_thresh:
                    raw.append((x, y, 0.0, MOVE, pass_no, rev_disp[i][0], rev_disp[i][1]))
                else:
                    raw.append((x, y, 0.0, GRADE, pass_no, rev_disp[i][0], rev_disp[i][1]))

    # Hard turning-radius bound on blade-down sweeps: replace undrivable corners
    # with R-radius MOVE headlands before obstacle detours (so detours also wrap
    # the new blade-up arcs).
    raw = _enforce_min_turn_radius(
        raw, turn_radius_m, turn_threshold_deg, max(grid.grid_size_m, 0.5)
    )

    # Obstacle detour pass (§12.3): any blade-up travel leg (RETURN deadhead,
    # loaded HAUL, multi-drop MOVE) whose straight line crosses an exclusion zone
    # is replaced by the shortest visibility-graph path around it. Blade-down
    # coverage (CUT/FILL/GRADE) is never rerouted - the blade legitimately works
    # right up to the cleared bins abutting the zone. The optimizer already
    # counted these detours in the haul/deadhead distances; this realizes them on
    # the drivable track so the in-cab line bends around the no-go area.
    # When ``arm_obstacles`` is given (center-pivot analysis), each pass routes
    # around the arm where it is parked during that pass, in addition to any
    # static exclusion zones (already folded into each per-pass field).
    if (obstacles.active or arm_obstacles) and len(raw) >= 2:
        detoured: List[Tuple[float, float, float, str, int, float, float]] = [raw[0]]
        for i in range(1, len(raw)):
            x, y, d, action, pass_no, xs_, ys_ = raw[i]
            prev = detoured[-1]
            field_i = (arm_obstacles.get(pass_no, obstacles)
                       if arm_obstacles else obstacles)
            if action in _BLADE_UP and action != START and field_i.active:
                chain = field_i.path((prev[0], prev[1]), (x, y))
                for vx, vy in chain[1:-1]:
                    detoured.append((vx, vy, 0.0, action, pass_no, vx, vy))
            detoured.append((x, y, d, action, pass_no, xs_, ys_))
        raw = detoured

    # Second pass: distances, bearings, instructions. CUT/FILL/GRADE distances are
    # charged on the RAW coverage polyline (skipping inserted MOVE headlands so the
    # coverage floor is preserved); MOVE/HAUL/RETURN use consecutive steering steps.
    # Emitted coordinates and bearings use the SMOOTHED / headland-repaired display
    # geometry (xs, ys).
    cum = 0.0
    prev_coverage_xy: Optional[XY] = None
    for idx, (xr, yr, depth_m, action, pass_no, xs_, ys_) in enumerate(raw):
        xy = (xr, yr)
        if action in (CUT, FILL, GRADE):
            if prev_coverage_xy is not None:
                seg = math.dist(prev_coverage_xy, xy)
            elif prev_xy is not None:
                seg = math.dist(prev_xy, xy)
            else:
                seg = 0.0
            prev_coverage_xy = xy
        else:
            seg = math.dist(prev_xy, xy) if prev_xy is not None else 0.0
            if action != MOVE:
                # Travel legs break coverage chaining so the next sweep measures
                # from the haul/return arrival, not a prior pocket's last cell.
                prev_coverage_xy = None
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
        heading = _bearing_deg((xs_, ys_), (nxt[5], nxt[6])) if nxt is not None else 0.0

        # ``blade`` is the state on the inbound leg (UP while repositioning). Every
        # waypoint sits on a real cut/fill cell, so each carries its own target
        # depth - including the first cell of a patch, which is reached by a
        # blade-up reposition but is still cut/filled to depth once the blade drops.
        blade = "UP" if action in _BLADE_UP else "DOWN"
        depth_mm = abs(depth_m) * 1000.0
        lat, lon = latlon_from_utm(xs_, ys_, zone, north)

        seq += 1
        points.append(GuidancePoint(
            seq=seq, pass_no=pass_no, action=action, blade=blade,
            x=xs_, y=ys_, lat=lat, lon=lon, depth_mm=depth_mm,
            heading_deg=heading, seg_dist_m=seg, cum_dist_m=cum,
            instruction=_instruction(action, depth_mm),
        ))
        prev_xy = xy

    n_turns, turn_angle = _turn_metrics([r[0] for r in raw], [r[1] for r in raw],
                                        grid.grid_size_m, blade_m,
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
        n_repaired_sweeps=n_repaired,
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
    xs: List[float],
    ys: List[float],
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

    Measured on the *raw coverage* path (not the smoothed steering line) so the
    turn count/time is unaffected by display smoothing.
    """
    if len(xs) < 3:
        return 0, 0.0
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
