"""Center-pivot irrigation scheduling (spec §12.4, optional stage).

The field is assumed to carry a center-pivot irrigation system anchored in the
middle and sized as large as will still fit inside the field boundary. Its arm
(a bar one radius long and one wheelbase wide) can be rotated to any bearing.

While the field is being leveled, the parked arm must stay clear of the scraper's
active cut-haul-fill pass so the machine never has to drive a detour around it.
Rotating the arm costs effort, so we want to:

* keep the arm clear of each pass's work footprint, and
* move the arm as seldom as possible (and, when it must move, as little as
  possible).

For every pass (in the executed route order) we compute the set of arm bearings
that would collide with that pass's work cells / haul leg, then greedily hold the
arm at one bearing across the longest possible run of consecutive passes before
it has to move. This yields the minimum number of repositions; ties on *where* to
move are broken toward the previous bearing to minimize total rotation.

Outputs:

* a schedule of pivot positions (which pass triggers each move and the new
  bearing), printed and written to CSV, and
* a set of PNG maps - one per pivot position - showing the arm over the field's
  cut/fill (or topology) state at the moment that position takes effect.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from . import FEET_TO_METERS
from .agd import latlon_from_utm
from .grid import Grid
from .obstacles import EMPTY_OBSTACLES, ExclusionZone, ObstacleField
from .passes import Pass
from .render import _world_to_pixel, diff_to_image, elevation_to_image

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Pillow is required. Install with: pip install Pillow") from exc


# ---------------------------------------------------------------------------
# Data containers
# ---------------------------------------------------------------------------


@dataclass
class PivotGeometry:
    """Center-pivot footprint: anchor (UTM xy), arm length and width (metres)."""

    center_x: float
    center_y: float
    radius_m: float
    wheelbase_m: float
    clearance_m: float
    zone: int
    is_northern: bool

    @property
    def center_latlon(self) -> Tuple[float, float]:
        return latlon_from_utm(self.center_x, self.center_y, self.zone, self.is_northern)

    def tip_xy(self, bearing_deg: float) -> Tuple[float, float]:
        """UTM xy of the arm tip at ``bearing_deg`` (compass: 0=N, 90=E)."""
        rad = math.radians(bearing_deg)
        return (
            self.center_x + self.radius_m * math.sin(rad),
            self.center_y + self.radius_m * math.cos(rad),
        )

    def tip_latlon(self, bearing_deg: float) -> Tuple[float, float]:
        x, y = self.tip_xy(bearing_deg)
        return latlon_from_utm(x, y, self.zone, self.is_northern)


@dataclass
class PivotPosition:
    """One held arm bearing across a contiguous run of passes (1-based route order)."""

    index: int            # 1-based position number in the schedule
    first_pass: int       # first pass (1-based) this bearing is held for
    last_pass: int        # last pass (1-based) this bearing is held for
    bearing_deg: float    # compass bearing (0=N, 90=E)
    move_deg: float       # rotation from the previous position (0 for the first)
    n_conflicts: int      # passes in this run with no collision-free bearing

    @property
    def n_passes(self) -> int:
        return self.last_pass - self.first_pass + 1


@dataclass
class PivotPlan:
    geom: PivotGeometry
    positions: List[PivotPosition] = field(default_factory=list)
    total_rotation_deg: float = 0.0
    conflict_passes: List[int] = field(default_factory=list)  # 1-based
    n_bins: int = 720
    move_cost: float = 0.0
    # Compass bearing (deg) held during each pass, indexed 0-based by route order.
    bearing_by_pass: np.ndarray = field(default_factory=lambda: np.zeros(0))

    @property
    def n_moves(self) -> int:
        return max(0, len(self.positions) - 1)

    @property
    def n_detours(self) -> int:
        return len(self.conflict_passes)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def compute_pivot_geometry(
    grid: Grid,
    wheelbase_ft: float,
    clearance_ft: float,
    center_xy: Optional[Tuple[float, float]] = None,
) -> PivotGeometry:
    """Anchor a pivot in the field's middle, sized as large as fits inside it.

    The anchor defaults to the centroid of the valid field cells ("the middle"),
    or ``center_xy`` (projected easting/northing) when given. The radius is the
    largest circle, centered there, that contains no invalid cell and stays
    inside the grid, less half the wheelbase so the arm's width also fits.
    """
    size = grid.grid_size_m
    ys, xs = np.mgrid[0:grid.height, 0:grid.width]
    cxw = grid.min_x + (xs + 0.5) * size
    cyw = grid.min_y + (ys + 0.5) * size
    valid = grid.valid

    if center_xy is not None:
        cx, cy = float(center_xy[0]), float(center_xy[1])
    else:
        cx = float(cxw[valid].mean())
        cy = float(cyw[valid].mean())

    dist = np.hypot(cxw - cx, cyw - cy)
    invalid = ~valid
    r_invalid = float(dist[invalid].min()) if invalid.any() else float("inf")

    # Distance to the grid borders (everything outside the grid is off-field).
    r_border = min(
        cx - grid.min_x,
        (grid.min_x + grid.width * size) - cx,
        cy - grid.min_y,
        (grid.min_y + grid.height * size) - cy,
    )

    wheelbase_m = wheelbase_ft * FEET_TO_METERS
    clearance_m = clearance_ft * FEET_TO_METERS
    half_diag = size * 0.5 * math.sqrt(2.0)
    radius = min(r_invalid, r_border) - half_diag - wheelbase_m / 2.0
    radius = max(radius, size)

    return PivotGeometry(
        center_x=cx,
        center_y=cy,
        radius_m=radius,
        wheelbase_m=wheelbase_m,
        clearance_m=clearance_m,
        zone=grid.zone,
        is_northern=grid.is_northern,
    )


# ---------------------------------------------------------------------------
# Angular-occupancy model
# ---------------------------------------------------------------------------


def _gather_work_points(grid: Grid, p: Pass) -> List[Tuple[float, float]]:
    """All world-xy points a pass disturbs: cut cells, fill cells, haul leg."""
    pts: List[Tuple[float, float]] = []
    for (bx, by), _ in p.cut_cells:
        pts.append(grid.cell_center_xy(bx, by))
    for (bx, by), _ in p.fill_cells:
        pts.append(grid.cell_center_xy(bx, by))
    # Sample the loaded haul leg (cut end -> fill start), the longest transit.
    ax, ay = p.cut_end_xy
    bx2, by2 = p.fill_start_xy
    seg = math.hypot(bx2 - ax, by2 - ay)
    n = max(1, int(seg / grid.grid_size_m))
    for k in range(n + 1):
        t = k / n
        pts.append((ax + (bx2 - ax) * t, ay + (by2 - ay) * t))
    return pts


def _blocked_bins(grid: Grid, geom: PivotGeometry, p: Pass, n_bins: int) -> np.ndarray:
    """Boolean array (length ``n_bins``) of arm bearings that collide with ``p``.

    A work point at radius ``r`` and bearing ``phi`` blocks the bearings within
    ``+/- (half_width + cell_radius)/r`` of ``phi`` (the arm has finite width and
    a cell has finite extent). Points beyond the arm's reach do not block.
    """
    blocked = np.zeros(n_bins, dtype=bool)
    cx, cy, R = geom.center_x, geom.center_y, geom.radius_m
    half_w = geom.wheelbase_m / 2.0 + geom.clearance_m
    cell_r = grid.grid_size_m * 0.5 * math.sqrt(2.0)
    bin_w = 2.0 * math.pi / n_bins

    for (x, y) in _gather_work_points(grid, p):
        dx = x - cx
        dy = y - cy
        r = math.hypot(dx, dy)
        if r > R + cell_r:
            continue  # outside the arm's swept disk -> never under the arm
        if r < 1e-6:
            return np.ones(n_bins, dtype=bool)  # work at the anchor blocks all
        ang = math.atan2(dx, dy) % (2.0 * math.pi)  # compass: atan2(east, north)
        delta = min((half_w + cell_r) / r, math.pi)
        ilo = int(math.floor((ang - delta) / bin_w))
        ihi = int(math.ceil((ang + delta) / bin_w))
        idx = np.arange(ilo, ihi + 1) % n_bins
        blocked[idx] = True
    return blocked


# ---------------------------------------------------------------------------
# Bearing-set helpers
# ---------------------------------------------------------------------------


def _circ_dist_deg(a: float, b: float) -> float:
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def cardinal(bearing_deg: float) -> str:
    """16-point compass label for a bearing."""
    dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    return dirs[int((bearing_deg % 360.0) / 22.5 + 0.5) % 16]


# ---------------------------------------------------------------------------
# Scheduling (global optimum trade-off: repositions vs. haul detours)
# ---------------------------------------------------------------------------


def plan_pivot(
    grid: Grid,
    ordered_passes: List[Pass],
    geom: PivotGeometry,
    n_bins: int = 720,
    move_cost: float = 25.0,
) -> PivotPlan:
    """Schedule arm bearings by global dynamic programming.

    For every pass and every candidate bearing bin we know whether the parked
    arm would collide with that pass's work (cut/fill cells + loaded haul leg).
    A collision left in place costs one *detour*; rotating the arm to a new
    bearing costs ``move_cost`` detour-equivalents (plus a tiny per-degree
    rotation term, used only to break ties toward the nearest bearing).

    The DP

        dp[i][b] = collide(i, b) + min( dp[i-1][b],            # hold bearing
                                        move_cost + min_b' dp[i-1][b'] )  # rotate

    finds, in O(passes x bins), the bearing sequence minimizing

        move_cost x (repositions) + (detoured passes)

    so the arm only repositions when holding still would cost more than
    ``move_cost`` passes' worth of detours - eliminating the back-and-forth
    jitter of a strict "never collide" rule. Set ``move_cost`` below 1 to
    recover the strict behaviour (avoid every avoidable collision); raise it to
    keep the arm parked longer and accept more detours.
    """
    n = len(ordered_passes)
    plan = PivotPlan(geom=geom, n_bins=n_bins, move_cost=move_cost)
    if n == 0:
        return plan

    blocked = np.empty((n, n_bins), dtype=bool)
    for i, p in enumerate(ordered_passes):
        blocked[i] = _blocked_bins(grid, geom, p, n_bins)
    detour = blocked.astype(np.float64)  # 1 per collided pass

    # Forward DP. The rotation tie-breaker is folded in during backtracking, so
    # the forward sweep is fully vectorised (flat move cost).
    dp = np.empty((n, n_bins), dtype=np.float64)
    dp[0] = detour[0]
    for i in range(1, n):
        minprev = float(dp[i - 1].min())
        dp[i] = detour[i] + np.minimum(dp[i - 1], minprev + move_cost)

    # Backtrack: prefer holding the bearing; when a move is forced, rotate to the
    # nearest optimal predecessor bearing (shortest rotation).
    eps = 1e-9
    chosen = np.empty(n, dtype=np.int64)
    chosen[n - 1] = int(np.argmin(dp[n - 1]))
    for i in range(n - 1, 0, -1):
        b = int(chosen[i])
        if abs(dp[i][b] - (detour[i][b] + dp[i - 1][b])) <= eps:
            chosen[i - 1] = b  # held the bearing (no move)
        else:
            minprev = float(dp[i - 1].min())
            cand = np.flatnonzero(dp[i - 1] <= minprev + eps)
            d = np.abs(cand - b)
            d = np.minimum(d, n_bins - d)
            chosen[i - 1] = int(cand[int(np.argmin(d))])

    bin_w_deg = 360.0 / n_bins
    plan.bearing_by_pass = (chosen.astype(np.float64) + 0.5) * bin_w_deg

    # Collapse equal-bearing runs into positions.
    prev_bearing: Optional[float] = None
    total_rot = 0.0
    start = 0
    idx = 1
    for i in range(1, n + 1):
        if i == n or chosen[i] != chosen[start]:
            b = int(chosen[start])
            bearing = (b + 0.5) * bin_w_deg
            move = 0.0 if prev_bearing is None else _circ_dist_deg(bearing, prev_bearing)
            total_rot += move
            plan.positions.append(
                PivotPosition(
                    index=idx,
                    first_pass=start + 1,
                    last_pass=i,
                    bearing_deg=bearing,
                    move_deg=move,
                    n_conflicts=int(blocked[start:i, b].sum()),
                )
            )
            prev_bearing = bearing
            start = i
            idx += 1

    plan.total_rotation_deg = total_rot
    plan.conflict_passes = [i + 1 for i in range(n) if blocked[i, int(chosen[i])]]
    return plan


# ---------------------------------------------------------------------------
# Routing the leveling machine around the parked arm (detour GPS paths)
# ---------------------------------------------------------------------------


def arm_ring(geom: PivotGeometry, bearing_deg: float) -> List[Tuple[float, float]]:
    """Rectangle (projected xy) covering the arm at ``bearing_deg``.

    Width is the wheelbase; it runs from the anchor out to the tip. Routing code
    buffers it by the machine's clearance.
    """
    rad = math.radians(bearing_deg)
    dx, dy = math.sin(rad), math.cos(rad)          # arm direction (east, north)
    px, py = math.cos(rad), -math.sin(rad)         # perpendicular
    half = geom.wheelbase_m / 2.0
    cx, cy = geom.center_x, geom.center_y
    tx, ty = geom.tip_xy(bearing_deg)
    # A little overshoot past the tip so a leg cannot clip around the very end.
    tx += dx * half
    ty += dy * half
    return [
        (cx + px * half, cy + py * half),
        (tx + px * half, ty + py * half),
        (tx - px * half, ty - py * half),
        (cx - px * half, cy - py * half),
    ]


def build_arm_obstacles(
    geom: PivotGeometry,
    plan: PivotPlan,
    base_obstacles: ObstacleField = EMPTY_OBSTACLES,
    clearance_m: float = 0.0,
) -> dict:
    """Map each pass (1-based) to an obstacle field for the arm parked then.

    Each pivot position contributes one buffered arm zone (combined with any
    static exclusion zones from ``base_obstacles``); every pass held at that
    position shares the field, so the leveling machine's haul/return legs route
    around the arm exactly where it sits during that pass.
    """
    by_pass: dict = {}
    base_zones = list(getattr(base_obstacles, "zones", []))
    for pos in plan.positions:
        ring = arm_ring(geom, pos.bearing_deg)
        zone = ExclusionZone.from_ring(ring, clearance_m)
        field_obs = ObstacleField.from_zones(base_zones + [zone])
        for q in range(pos.first_pass, pos.last_pass + 1):
            by_pass[q] = field_obs
    return by_pass


def pivot_detour_stats(
    ordered_passes: List[Pass],
    arm_obstacles: dict,
    min_detour_m: float = 1e-3,
) -> Tuple[int, float, int, List[int]]:
    """Measure how the parked arm affects each pass's loaded haul leg.

    For each pass whose haul leg (cut end -> fill start) crosses the arm, route
    around it. A leg whose endpoints are clear gets a real detour (counted in
    ``routed`` with its extra distance); a leg that starts or ends *under* the
    arm cannot be rerouted (the work itself is beneath the arm) and is counted in
    ``under_arm`` instead. Returns
    ``(routed, extra_haul_m, under_arm, routed_pass_numbers)``.
    """
    routed = 0
    under_arm = 0
    extra = 0.0
    passes: List[int] = []
    for i, p in enumerate(ordered_passes, start=1):
        af = arm_obstacles.get(i)
        if af is None or not af.active:
            continue
        a, b = p.cut_end_xy, p.fill_start_xy
        if not af.blocked(a, b):
            continue
        d_extra = af.path_length(a, b) - math.dist(a, b)
        if d_extra > min_detour_m:
            routed += 1
            extra += d_extra
            passes.append(i)
        else:
            under_arm += 1
    return routed, extra, under_arm, passes


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------


def _load_font(size: int) -> "ImageFont.ImageFont":
    try:
        return ImageFont.load_default(size=size)
    except Exception:  # pragma: no cover - older Pillow
        return ImageFont.load_default()


def _draw_pivot(
    image: "Image.Image",
    grid: Grid,
    geom: PivotGeometry,
    bearing_deg: float,
    ppb: int,
    ghost_bearing: Optional[float] = None,
    arm_color: Tuple[int, int, int, int] = (10, 10, 10, 255),
) -> None:
    """Overlay the pivot circle, anchor and arm (plus an optional ghost arm)."""
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)

    ccx, ccy = _world_to_pixel(grid, geom.center_x, geom.center_y, ppb)
    r_px = geom.radius_m / grid.grid_size_m * ppb
    arm_w = max(2, round(geom.wheelbase_m / grid.grid_size_m * ppb))

    # Swept circle (the irrigated area).
    od.ellipse(
        (ccx - r_px, ccy - r_px, ccx + r_px, ccy + r_px),
        outline=(20, 60, 160, 230), width=max(2, ppb // 3),
    )

    # Ghost of the previous arm position, for context.
    if ghost_bearing is not None:
        gx, gy = _world_to_pixel(grid, *geom.tip_xy(ghost_bearing), ppb)
        od.line([(round(ccx), round(ccy)), (round(gx), round(gy))],
                fill=(120, 120, 120, 150), width=arm_w)

    # The held arm position.
    tx, ty = _world_to_pixel(grid, *geom.tip_xy(bearing_deg), ppb)
    od.line([(round(ccx), round(ccy)), (round(tx), round(ty))],
            fill=arm_color, width=arm_w)
    # Outer tower (arm tip) marker.
    tr = max(3, arm_w)
    od.ellipse((tx - tr, ty - tr, tx + tr, ty + tr),
               fill=(255, 215, 0, 255), outline=(10, 10, 10, 255), width=max(1, arm_w // 3))

    # Anchor.
    ar = max(3, arm_w)
    od.ellipse((ccx - ar, ccy - ar, ccx + ar, ccy + ar),
               fill=(10, 10, 10, 255), outline=(255, 255, 255, 255), width=max(1, arm_w // 3))

    image.alpha_composite(overlay)


def _draw_run_ops(
    image: "Image.Image",
    grid: Grid,
    passes: List[Pass],
    ppb: int,
) -> None:
    """Faint orange haul lines marking where the leveling works during this run."""
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    for p in passes:
        a = _world_to_pixel(grid, p.cut_end_xy[0], p.cut_end_xy[1], ppb)
        b = _world_to_pixel(grid, p.fill_start_xy[0], p.fill_start_xy[1], ppb)
        od.line([(round(a[0]), round(a[1])), (round(b[0]), round(b[1]))],
                fill=(255, 120, 0, 110), width=max(1, ppb // 3))
    image.alpha_composite(overlay)


def _banner(image: "Image.Image", lines: List[str], ppb: int) -> None:
    draw = ImageDraw.Draw(image)
    font = _load_font(max(13, ppb * 2))
    lh = max(16, ppb * 2 + 4)
    pad = 8
    box_w = max(int(draw.textlength(s, font=font)) for s in lines) + pad * 2
    box_h = pad * 2 + lh * len(lines)
    draw.rectangle((6, 6, 6 + box_w, 6 + box_h), fill=(255, 255, 255, 235), outline=(0, 0, 0, 255))
    for i, s in enumerate(lines):
        draw.text((6 + pad, 6 + pad + i * lh), s, fill=(0, 0, 0, 255), font=font)


def render_pivot_pngs(
    grid: Grid,
    ordered_passes: List[Pass],
    plan: PivotPlan,
    out_dir: Path,
    mode: str = "cutfill",
    pixels_per_bin: int = 6,
) -> List[Path]:
    """One PNG per pivot position: the arm over the field state when it takes effect.

    The field state shown is the cut/fill (or topology) surface after every pass
    *before* the position's first pass has been applied - i.e. the topology at the
    moment the operator would rotate the arm to that bearing.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ppb = max(1, pixels_per_bin)
    # Clear stale maps from a previous (possibly longer) schedule.
    for old in out_dir.glob("pivot_*.png"):
        old.unlink()

    existing = grid.existing.astype(float).copy()
    target = grid.target
    valid = grid.valid

    if mode == "topology":
        ev = existing[valid]
        tv = target[valid]
        vmin = float(min(np.nanmin(ev), np.nanmin(tv)))
        vmax = float(max(np.nanmax(ev), np.nanmax(tv)))
    else:
        vmin = vmax = 0.0

    # Map 0-based first-pass index -> position (and its predecessor bearing).
    starts = {pos.first_pass - 1: pos for pos in plan.positions}
    prev_bearing_of = {}
    prev = None
    for pos in plan.positions:
        prev_bearing_of[pos.first_pass - 1] = prev
        prev = pos.bearing_deg

    geom = plan.geom
    paths: List[Path] = []
    n = len(ordered_passes)

    def render_at(i: int) -> None:
        pos = starts[i]
        if mode == "topology":
            img = elevation_to_image(existing, valid, ppb, vmin, vmax).convert("RGBA")
        else:
            img = diff_to_image(existing - target, valid, ppb).convert("RGBA")
        run_passes = ordered_passes[pos.first_pass - 1: pos.last_pass]
        _draw_run_ops(img, grid, run_passes, ppb)
        _draw_pivot(img, grid, geom, pos.bearing_deg, ppb,
                    ghost_bearing=prev_bearing_of.get(i))
        clat, clon = geom.center_latlon
        _banner(img, [
            f"Pivot position {pos.index}/{len(plan.positions)}  bearing "
            f"{pos.bearing_deg:.1f} deg ({cardinal(pos.bearing_deg)})",
            f"passes {pos.first_pass}-{pos.last_pass} ({pos.n_passes})  "
            f"move {pos.move_deg:.1f} deg" + ("  [START]" if pos.move_deg == 0 else ""),
            f"radius {geom.radius_m / FEET_TO_METERS:.0f} ft  anchor "
            f"{clat:.5f}, {clon:.5f}" + (f"  conflicts {pos.n_conflicts}" if pos.n_conflicts else ""),
        ], ppb)
        path = out_dir / f"pivot_{pos.index:03d}_pass{pos.first_pass:05d}.png"
        img.save(path)
        paths.append(path)

    for i in range(n):
        if i in starts:
            render_at(i)
        p = ordered_passes[i]
        for (bx, by), depth in p.cut_cells:
            existing[by, bx] -= depth
        for (bx, by), depth in p.fill_cells:
            existing[by, bx] += depth

    # Render an overview of all arm bearings over the initial cut/fill map.
    _render_overview(grid, plan, out_dir, ppb)
    return paths


def _render_overview(grid: Grid, plan: PivotPlan, out_dir: Path, ppb: int) -> None:
    img = diff_to_image(grid.delta, grid.valid, ppb).convert("RGBA")
    # Fade the map so the arms stand out.
    arr = np.asarray(img).astype(float)
    white = np.array([255, 255, 255], dtype=float)
    arr[..., :3] = arr[..., :3] * 0.45 + white * 0.55
    img = Image.fromarray(arr.astype(np.uint8), "RGBA")

    overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    geom = plan.geom
    ccx, ccy = _world_to_pixel(grid, geom.center_x, geom.center_y, ppb)
    r_px = geom.radius_m / grid.grid_size_m * ppb
    od.ellipse((ccx - r_px, ccy - r_px, ccx + r_px, ccy + r_px),
               outline=(20, 60, 160, 230), width=max(2, ppb // 3))
    arm_w = max(2, round(geom.wheelbase_m / grid.grid_size_m * ppb))
    npos = len(plan.positions)
    for pos in plan.positions:
        frac = (pos.index - 1) / max(1, npos - 1)
        col = (int(40 + 200 * frac), int(40), int(220 - 180 * frac), 235)
        tx, ty = _world_to_pixel(grid, *geom.tip_xy(pos.bearing_deg), ppb)
        od.line([(round(ccx), round(ccy)), (round(tx), round(ty))], fill=col, width=arm_w)
        font = _load_font(max(12, ppb * 2))
        od.text((tx + 2, ty + 2), str(pos.index), fill=col, font=font)
    ar = max(3, arm_w)
    od.ellipse((ccx - ar, ccy - ar, ccx + ar, ccy + ar),
               fill=(10, 10, 10, 255), outline=(255, 255, 255, 255))
    img.alpha_composite(overlay)
    _banner(img, [
        f"All pivot positions: {npos} (moves {plan.n_moves}, "
        f"total rotation {plan.total_rotation_deg:.0f} deg)",
        "arm color: blue=first -> red=last",
    ], ppb)
    img.save(out_dir / "pivot_overview.png")


# ---------------------------------------------------------------------------
# CSV
# ---------------------------------------------------------------------------


def write_pivot_csv(plan: PivotPlan, path: Path) -> None:
    geom = plan.geom
    clat, clon = geom.center_latlon
    lines = [
        "# center-pivot schedule",
        f"# anchor_lat,{clat:.7f}",
        f"# anchor_lon,{clon:.7f}",
        f"# radius_ft,{geom.radius_m / FEET_TO_METERS:.2f}",
        f"# wheelbase_ft,{geom.wheelbase_m / FEET_TO_METERS:.2f}",
        f"# moves,{plan.n_moves}",
        f"# total_rotation_deg,{plan.total_rotation_deg:.2f}",
        f"# detoured_passes,{plan.n_detours}",
        f"# move_cost_passes,{plan.move_cost:g}",
        "position,first_pass,last_pass,n_passes,bearing_deg,cardinal,"
        "move_deg,tip_lat,tip_lon,n_conflicts",
    ]
    for pos in plan.positions:
        tlat, tlon = geom.tip_latlon(pos.bearing_deg)
        lines.append(
            f"{pos.index},{pos.first_pass},{pos.last_pass},{pos.n_passes},"
            f"{pos.bearing_deg:.2f},{cardinal(pos.bearing_deg)},{pos.move_deg:.2f},"
            f"{tlat:.7f},{tlon:.7f},{pos.n_conflicts}"
        )
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------------------
# Leveling animation with pivot moves
# ---------------------------------------------------------------------------


def animate_pivot_leveling(
    grid: Grid,
    ordered_passes: List[Pass],
    plan: PivotPlan,
    gif_path: Optional[Path] = None,
    out_dir: Optional[Path] = None,
    stride: int = 15,
    pixels_per_bin: int = 6,
    mode: str = "cutfill",
    gif_ms: int = 90,
) -> int:
    """Animate the field leveling with the pivot arm, pausing on every move.

    A frame is emitted every ``stride`` applied passes, plus a dedicated,
    longer-held frame at each pivot reposition (the arm flashes red and a grey
    ghost shows where it rotated from), so every scheduled move is visible. The
    current pass's loaded haul leg is drawn so it is clear the arm sits clear of
    the active work. Returns the number of frames produced.
    """
    if gif_path is None and out_dir is None:
        return 0
    n = len(ordered_passes)
    if n == 0:
        return 0

    ppb = max(1, pixels_per_bin)
    existing = grid.existing.astype(float).copy()
    target = grid.target
    valid = grid.valid

    if mode == "topology":
        ev = existing[valid]
        tv = target[valid]
        vmin = float(min(np.nanmin(ev), np.nanmin(tv)))
        vmax = float(max(np.nanmax(ev), np.nanmax(tv)))
    else:
        vmin = vmax = 0.0

    geom = plan.geom
    bearings = plan.bearing_by_pass  # degrees, 0-based by pass
    # 1-based pass index -> (move number, previous bearing) when a move happens.
    move_info = {}
    for k, pos in enumerate(plan.positions):
        if pos.move_deg > 0:
            move_info[pos.first_pass] = (k, plan.positions[k - 1].bearing_deg)
    n_moves = plan.n_moves

    init_total = float(np.abs(np.nan_to_num(grid.delta, nan=0.0)).sum())

    if out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

    gif_frames: List["Image.Image"] = []
    durations: List[int] = []
    frame_no = 0

    def base_image() -> "Image.Image":
        if mode == "topology":
            return elevation_to_image(existing, valid, ppb, vmin, vmax).convert("RGBA")
        return diff_to_image(existing - target, valid, ppb).convert("RGBA")

    def pct_done() -> float:
        if init_total <= 0:
            return 100.0
        rem = float(np.abs(np.where(valid, existing - target, 0.0)).sum())
        return max(0.0, min(100.0, 100.0 * (1.0 - rem / init_total)))

    def emit(img: "Image.Image", hold_ms: int) -> None:
        nonlocal frame_no
        if out_dir is not None:
            img.save(out_dir / f"frame_{frame_no:05d}.png")
        if gif_path is not None:
            bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
            gif_frames.append(
                Image.alpha_composite(bg, img).convert("P", palette=Image.ADAPTIVE)
            )
            durations.append(hold_ms)
        frame_no += 1

    # Initial frame (field untouched, arm at the first scheduled bearing).
    img = base_image()
    _draw_pivot(img, grid, geom, float(bearings[0]), ppb)
    _banner(img, [
        f"Leveling start - pass 0/{n}  (0% to grade)",
        f"pivot bearing {bearings[0]:.0f} deg ({cardinal(float(bearings[0]))})  "
        f"radius {geom.radius_m / FEET_TO_METERS:.0f} ft",
        f"plan: {len(plan.positions)} positions, {n_moves} moves, "
        f"{plan.n_detours} detoured passes",
    ], ppb)
    emit(img, 1500)

    for i in range(1, n + 1):
        # A move takes effect just before pass i runs: show the arm rotating
        # into its new clear bearing while the field is still in its pre-pass
        # state, then continue applying passes.
        if i in move_info:
            mk, prev_bearing = move_info[i]
            new_bearing = float(bearings[i - 1])
            upcoming = ordered_passes[i - 1:i]
            img = base_image()
            _draw_run_ops(img, grid, upcoming, ppb)
            _draw_pivot(img, grid, geom, new_bearing, ppb,
                        ghost_bearing=prev_bearing, arm_color=(220, 0, 0, 255))
            _banner(img, [
                f"PIVOT MOVE {mk}/{n_moves}  before pass {i}/{n}  "
                f"({pct_done():.0f}% to grade)",
                f"rotate {prev_bearing:.0f} -> {new_bearing:.0f} deg "
                f"({_circ_dist_deg(new_bearing, prev_bearing):.0f} deg, "
                f"{cardinal(prev_bearing)} -> {cardinal(new_bearing)})",
                "red = new arm position, grey = previous",
            ], ppb)
            emit(img, 900)

        p = ordered_passes[i - 1]
        for (bx, by), depth in p.cut_cells:
            existing[by, bx] -= depth
        for (bx, by), depth in p.fill_cells:
            existing[by, bx] += depth

        if (i % stride == 0) or (i == n):
            cur_bearing = float(bearings[i - 1])
            img = base_image()
            _draw_run_ops(img, grid, [p], ppb)
            _draw_pivot(img, grid, geom, cur_bearing, ppb)
            _banner(img, [
                f"Leveling pass {i}/{n}  ({pct_done():.0f}% to grade)",
                f"pivot bearing {cur_bearing:.0f} deg ({cardinal(cur_bearing)})  "
                f"position {_position_of_pass(plan, i)}/{len(plan.positions)}",
                "orange = current haul (machine at work)",
            ], ppb)
            emit(img, gif_ms)

    if gif_path is not None and gif_frames:
        durations[-1] = 5000  # hold the finished field
        gif_frames[0].save(
            Path(gif_path),
            save_all=True,
            append_images=gif_frames[1:],
            duration=durations,
            loop=0,
            disposal=2,
        )

    return frame_no


def _position_of_pass(plan: PivotPlan, pass_1based: int) -> int:
    for pos in plan.positions:
        if pos.first_pass <= pass_1based <= pos.last_pass:
            return pos.index
    return len(plan.positions)
