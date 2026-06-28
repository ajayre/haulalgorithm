"""Stage 4 - pass generation (spec §11).

Each pass is one cut-haul-fill cycle packed to the scraper load limit (in BCY,
derived from the LCY capacity and swell) under the 61 mm depth-per-pass limit.

To pack a full load and keep hauls short, each pass region-grows a compact,
contiguous patch of cut cells from an anchor (the cut cell whose nearest fill is
closest) and deposits into a compact, contiguous fill patch grown from a nearby
fill seed. Keeping both patches contiguous keeps the swept loading/dumping
distances near their minimum. An optional ``heading`` biases fill selection so
soil moves down the field's main slope (lane-oriented hauling, as OptiSurface
does). Deep cuts and fills span several 61 mm layers and therefore several
passes; the transition recedes as boundary cells reach grade.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from . import DEFAULT_MAX_DROP_HOP_FT, FEET_TO_METERS, M3_TO_CY
from .flowfield import FlowField
from .grid import Grid
from .strips import Strip, _bilinear, flow_direction_field, streamline_to

Cell = Tuple[int, int]


@dataclass
class Pass:
    """One cut-haul-fill cycle (spec §11.5)."""

    cut_cells: List[Tuple[Cell, float]]   # ((bx, by), depth_m)
    fill_cells: List[Tuple[Cell, float]]  # ((bx, by), depth_m)
    load_bcy: float
    haul_distance_m: float
    cut_start_xy: Tuple[float, float]
    cut_end_xy: Tuple[float, float]
    fill_start_xy: Tuple[float, float]
    fill_end_xy: Tuple[float, float]
    # Local haul-flow bearing (compass deg) the cut/fill strip was grown along;
    # ``None`` in single-heading mode. Guidance tiles each patch's blade lanes
    # along these so the lanes follow the flow rather than one global heading.
    cut_heading_deg: Optional[float] = None
    fill_heading_deg: Optional[float] = None


class _Ledger:
    """Tracks remaining cut/fill depth (metres) per cell and BCY conversions."""

    def __init__(self, grid: Grid, tolerance_m: float):
        self.grid = grid
        self.tol = tolerance_m
        d = np.nan_to_num(grid.delta, nan=0.0)
        self.remaining_cut = np.where(grid.valid & (d > 0), d, 0.0)
        self.remaining_fill = np.where(grid.valid & (d < 0), -d, 0.0)
        self.area = grid.area_m2
        self.swell = grid.swell_factor
        self.compaction = grid.compaction
        self.max_depth_m = grid.max_depth_mm / 1000.0

    def cut_bcy_per_m(self) -> float:
        """Bank BCY per metre of cut depth."""
        return self.area * M3_TO_CY

    def fill_bcy_per_m(self) -> float:
        """Bank cut-soil BCY needed per metre of fill depth (× cut/fill ratio)."""
        return self.area * M3_TO_CY * self.compaction


@dataclass
class PassResult:
    passes: List[Pass]
    rounds: int
    unmatched_cut_bcy: float
    unmatched_fill_bcy: float


def estimate_slope_heading(grid: Grid, snap_tol_deg: float = 12.0) -> float:
    """Compass bearing (deg, 0=N, 90=E) of the target surface's main down-slope.

    Soil on a sloped design surface is moved down-slope; this is used as the
    default haul heading when one is not supplied.

    The calculation grid is built axis-aligned (N/S/E/W), and both strip growth
    and the blade-lane tiling are cleanest when the haul heading lies *on* a grid
    axis (this is how OptiHaul works - its grid is aligned to the haul
    direction). A heading a few degrees off-axis pushes strips diagonally across
    cells, fragmenting them into extra pockets and inflating repositioning/haul.
    So when the estimated slope is within ``snap_tol_deg`` of a grid axis we snap
    to it; pronounced diagonal slopes are left as-is.
    """
    t = np.where(grid.valid, grid.target, np.nan)
    gy, gx = np.gradient(t)  # gx along +x (east), gy along +y (north)
    e = -np.nanmean(gx)      # down-slope points opposite the gradient
    n = -np.nanmean(gy)
    if not np.isfinite(e) or not np.isfinite(n) or (abs(e) < 1e-12 and abs(n) < 1e-12):
        return 0.0
    bearing = math.degrees(math.atan2(e, n)) % 360.0
    nearest_axis = round(bearing / 90.0) * 90.0
    if abs(((bearing - nearest_axis + 180.0) % 360.0) - 180.0) <= snap_tol_deg:
        return nearest_axis % 360.0
    return bearing


def _heading_vector(heading_deg: float) -> Tuple[float, float]:
    """Compass bearing -> unit (east, north) vector pointing down-heading."""
    r = math.radians(heading_deg)
    return math.sin(r), math.cos(r)


def generate_passes(
    grid: Grid,
    strips: List[Strip],
    tolerance_mm: float,
    max_load_bcy: float,
    max_rounds: int = 0,  # unused; kept for API compatibility
    heading_deg: Optional[float] = None,
    heading_strength: float = 0.5,
    strip_bias: float = 50.0,
    max_drop_hop_m: float = DEFAULT_MAX_DROP_HOP_FT * FEET_TO_METERS,
    flow: Optional[FlowField] = None,
    flow_aligned: bool = False,
) -> PassResult:
    """Generate cut-haul-fill passes packed to ``max_load_bcy`` (spec §11).

    Each pass cuts a compact, *contiguous* patch of cut cells (region grown from
    an anchor) and deposits into a compact, contiguous fill patch, so the swept
    loading/dumping distances stay near their minimum.

    Two ways the cut->fill direction is chosen:

    * **Single heading** (``flow_aligned=False``): every cut/fill strip is grown
      along the one global ``heading_deg`` (the field's main slope) and fill
      selection is biased down that bearing - lane-oriented hauling as OptiSurface
      does. Simplest to drive (parallel lanes) but can push soil across the grain
      of the optimal transport when that runs diagonal to the heading.
    * **Flow-aligned** (``flow_aligned=True`` with a ``flow`` field): each load's
      cut strip is grown along the *local* flow direction (``-grad(phi)``) at its
      anchor, the fill is found by following the streamline downstream from the
      cut into the deficit it feeds, and the fill strip is grown along the local
      flow there. Soil moves with the grain (shorter, with-flow hauls); the
      resulting lanes curve across the field. The per-pass cut/fill bearings are
      recorded on each :class:`Pass` so the guidance builder tiles blade lanes
      along the flow rather than one global heading.
    """
    tol_m = tolerance_mm / 1000.0
    led = _Ledger(grid, tol_m)
    cut_per_m = led.cut_bcy_per_m()
    fill_per_m = led.fill_bcy_per_m()
    W, H = grid.width, grid.height

    hvec = _heading_vector(heading_deg) if heading_deg is not None else None
    k_head = max(0.0, min(0.95, heading_strength)) if hvec is not None else 0.0

    # Heading unit vector in grid (east=bx, north=by) space, used to grow cut/fill
    # patches as long strips along the haul heading instead of compact blobs.
    strip_uv = None
    if heading_deg is not None and strip_bias > 1.0:
        r = math.radians(heading_deg)
        strip_uv = (math.sin(r), math.cos(r))  # (east, north) = (bx, by)

    # Flow-aligned mode: precompute the smoothed unit flow-direction field once.
    fxs = fys = None
    if flow_aligned and flow is not None:
        fxs, fys = flow_direction_field(grid, flow)

    def _flow_heading_for_cells(cells):
        """Dominant flow bearing (compass deg) over a patch, or None if flat/off."""
        if fxs is None:
            return None
        sx = sum(float(fxs[c[1], c[0]]) for c, _d in cells)
        sy = sum(float(fys[c[1], c[0]]) for c, _d in cells)
        if sx * sx + sy * sy < 1e-12:
            return None
        return math.degrees(math.atan2(sx, sy)) % 360.0

    def _streamline_fill_seed(cut_centroid):
        """Cell with remaining fill found by following the flow down from the cut."""
        if fxs is None:
            return None
        xf = (cut_centroid[0] - grid.min_x) / grid.grid_size_m
        yf = (cut_centroid[1] - grid.min_y) / grid.grid_size_m
        return streamline_to(fxs, fys, grid.valid, xf, yf,
                             lambda bx, by: led.remaining_fill[by, bx] > tol_m)

    centers = _cell_centers(grid)
    passes: List[Pass] = []

    try:
        from scipy.spatial import cKDTree
        have_kd = True
    except ImportError:
        have_kd = False

    guard = 0
    max_guard = grid.width * grid.height * 64 + 10000
    while guard < max_guard:
        guard += 1
        cut_idx = np.argwhere(led.remaining_cut > tol_m)   # (j, i)
        fill_idx = np.argwhere(led.remaining_fill > tol_m)
        if cut_idx.shape[0] == 0 or fill_idx.shape[0] == 0:
            break

        cut_xy = centers[cut_idx[:, 0], cut_idx[:, 1]]
        fill_xy = centers[fill_idx[:, 0], fill_idx[:, 1]]

        total_fill_bcy = float(led.remaining_fill[led.remaining_fill > tol_m].sum()) * fill_per_m
        load_cap = min(max_load_bcy, total_fill_bcy)
        if load_cap <= 1e-9:
            break

        ftree = cKDTree(fill_xy) if have_kd else None
        ctree = cKDTree(cut_xy) if have_kd else None

        # Anchor: cut cell whose nearest remaining fill is closest (shortest haul).
        if have_kd:
            dists, _nn = ftree.query(cut_xy, k=1)
            a = int(np.argmin(dists))
        else:
            a = _anchor_bruteforce(cut_xy, fill_xy)
        anchor = (int(cut_idx[a, 1]), int(cut_idx[a, 0]))

        # --- Multi-drop CUT: keep loading across nearby cut pockets until the
        # scraper is full. Grow a strip from the anchor; if it doesn't fill the
        # load and the next remaining cut pocket is within an economical hop,
        # drive there (blade up) and keep cutting, rather than hauling a partial
        # load and leaving a near-empty job for a future pass.
        cut_cells: List[Tuple[Cell, float]] = []
        load = 0.0
        seed = anchor
        while load < load_cap - 1e-9:
            if fxs is not None:
                cells, got = _grow_flow_tube(led.remaining_cut, led.max_depth_m, tol_m,
                                             seed, load_cap - load, cut_per_m, W, H,
                                             fxs, fys, grid.valid, signs=(-1.0,))
            else:
                cells, got = _grow_region(led.remaining_cut, led.max_depth_m, tol_m,
                                          seed, load_cap - load, cut_per_m, W, H,
                                          heading_uv=strip_uv, strip_bias=strip_bias)
            if cells:
                cut_cells.extend(cells)
                load += got
            if load >= load_cap - 1e-9:
                break
            last_xy = (grid.cell_center_xy(*cells[-1][0]) if cells
                       else grid.cell_center_xy(*seed))
            nxt = _nearest_remaining(last_xy, ctree, cut_xy, cut_idx,
                                     led.remaining_cut, tol_m)
            if nxt is None or nxt[1] > max_drop_hop_m:
                break
            seed = nxt[0]
        if not cut_cells:
            break
        cut_centroid = _centroid([grid.cell_center_xy(*c) for c, _ in cut_cells])

        # --- Multi-drop FILL: spread the whole load across nearby fill pockets.
        # Grow a fill patch from a seed near the cut; if it can't absorb the full
        # load and the next remaining fill pocket is within an economical hop,
        # drive there (still loaded, blade up) and keep dumping until empty.
        fill_cells: List[Tuple[Cell, float]] = []
        placed = 0.0
        # Flow-aligned: seed the fill where the streamline from the cut first
        # reaches a deficit (haul follows the flow). Otherwise pick the nearest /
        # down-heading fill as before.
        fseed = _streamline_fill_seed(cut_centroid)
        if fseed is None:
            fseed = _choose_fill_seed(fill_xy, fill_idx, cut_centroid, hvec, k_head, have_kd)
        while placed < load - 1e-9:
            if fxs is not None:
                cells, got = _grow_flow_tube(led.remaining_fill, led.max_depth_m, tol_m,
                                             fseed, load - placed, fill_per_m, W, H,
                                             fxs, fys, grid.valid, signs=(1.0,))
            else:
                cells, got = _grow_region(led.remaining_fill, led.max_depth_m, tol_m,
                                          fseed, load - placed, fill_per_m, W, H,
                                          heading_uv=strip_uv, strip_bias=strip_bias)
            if cells:
                fill_cells.extend(cells)
                placed += got
            if placed >= load - 1e-9:
                break
            last_xy = (grid.cell_center_xy(*cells[-1][0]) if cells
                       else grid.cell_center_xy(*fseed))
            nxt = _nearest_remaining(last_xy, ftree, fill_xy, fill_idx,
                                     led.remaining_fill, tol_m)
            if nxt is None or nxt[1] > max_drop_hop_m:
                break
            fseed = nxt[0]
        if not fill_cells:
            for (c, d) in cut_cells:
                led.remaining_cut[c[1], c[0]] += d
            break
        # If even multi-drop couldn't place the whole load (fill genuinely
        # exhausted within reach, e.g. the very end of the job), restore the
        # unplaced remainder to the ledger AND trim those cells out of this pass,
        # so the recorded cut footprint matches what was actually moved. The
        # far-from-anchor cells are given back first, keeping the cut nearest the
        # fill it served.
        leftover = load - placed
        if leftover > 1e-9:
            kept_rev: List[Tuple[Cell, float]] = []
            for (c, d) in reversed(cut_cells):
                if leftover <= 1e-9:
                    kept_rev.append((c, d))
                    continue
                cell_bcy = d * cut_per_m
                give = min(leftover, cell_bcy)
                led.remaining_cut[c[1], c[0]] += give / cut_per_m
                leftover -= give
                kept_depth = d - give / cut_per_m
                if kept_depth > 1e-12:
                    kept_rev.append((c, kept_depth))
            cut_cells = list(reversed(kept_rev))
        if not cut_cells:
            continue

        pass_obj = _build_pass(grid, cut_cells, fill_cells, placed,
                               ordered=fxs is not None)
        pass_obj.cut_heading_deg = _flow_heading_for_cells(cut_cells)
        pass_obj.fill_heading_deg = _flow_heading_for_cells(fill_cells)
        passes.append(pass_obj)

    unmatched_cut = float(led.remaining_cut[led.remaining_cut > tol_m].sum()) * cut_per_m
    unmatched_fill = float(led.remaining_fill[led.remaining_fill > tol_m].sum()) * fill_per_m
    return PassResult(passes=passes, rounds=1,
                      unmatched_cut_bcy=unmatched_cut, unmatched_fill_bcy=unmatched_fill)


def _cell_centers(grid: Grid) -> np.ndarray:
    """Array [by, bx, 2] of cell-center (x, y) in metres."""
    xs = grid.min_x + (np.arange(grid.width) + 0.5) * grid.grid_size_m
    ys = grid.min_y + (np.arange(grid.height) + 0.5) * grid.grid_size_m
    gx, gy = np.meshgrid(xs, ys)
    return np.stack([gx, gy], axis=-1)


def _nearest_remaining(query_xy, tree, xy_arr, idx_arr, remaining, tol, k: int = 24):
    """Nearest cell to ``query_xy`` that still holds material (> tol).

    Returns ``((bx, by), distance_m)`` or ``None``. Used to find the next pocket
    a multi-drop pass should hop to. With a KD-tree we only inspect the ``k``
    nearest candidates (cells consumed earlier this pass are skipped); if all of
    them are spent the next pocket is far away, so we report no economical hop.
    """
    n = int(xy_arr.shape[0])
    if n == 0:
        return None
    if tree is not None:
        kk = min(k, n)
        d, ii = tree.query(query_xy, k=kk)
        for dist, j in zip(np.atleast_1d(d), np.atleast_1d(ii)):
            by, bx = int(idx_arr[j, 0]), int(idx_arr[j, 1])
            if remaining[by, bx] > tol:
                return (bx, by), float(dist)
        return None
    diff = xy_arr - np.asarray(query_xy)
    order = np.argsort(diff[:, 0] ** 2 + diff[:, 1] ** 2)
    for j in order:
        by, bx = int(idx_arr[j, 0]), int(idx_arr[j, 1])
        if remaining[by, bx] > tol:
            return (bx, by), float(math.hypot(diff[j, 0], diff[j, 1]))
    return None


def _anchor_bruteforce(cut_xy: np.ndarray, fill_xy: np.ndarray) -> int:
    best = 0
    best_d = None
    for a in range(cut_xy.shape[0]):
        d2 = ((fill_xy - cut_xy[a]) ** 2).sum(axis=1).min()
        if best_d is None or d2 < best_d:
            best_d = d2
            best = a
    return best


def _choose_fill_seed(fill_xy, fill_idx, cut_centroid, hvec, k_head, have_kd):
    """Pick the fill seed cell near the cut centroid, optionally biased down-heading.

    Without a heading this is simply the nearest fill cell. With one, candidates
    near the cut are re-ranked by ``dist * (1 - k * cos)`` where ``cos`` is the
    alignment of the cut->fill direction with the heading, so down-heading fills
    are preferred without allowing arbitrarily long hauls.
    """
    cc = np.asarray(cut_centroid)
    if hvec is None or k_head <= 0.0:
        if have_kd:
            from scipy.spatial import cKDTree
            _d, m = cKDTree(fill_xy).query(cc, k=1)
            m = int(np.atleast_1d(m)[0])
        else:
            m = int(((fill_xy - cc) ** 2).sum(axis=1).argmin())
        return (int(fill_idx[m, 1]), int(fill_idx[m, 0]))

    # Restrict to a local candidate set, then re-rank by anisotropic cost.
    n = fill_xy.shape[0]
    kk = min(200, n)
    if have_kd:
        from scipy.spatial import cKDTree
        _d, cand = cKDTree(fill_xy).query(cc, k=kk)
        cand = np.atleast_1d(cand)
    else:
        cand = np.argsort(((fill_xy - cc) ** 2).sum(axis=1))[:kk]
    d = fill_xy[cand] - cc
    dist = np.hypot(d[:, 0], d[:, 1])
    safe = np.where(dist > 1e-9, dist, 1e-9)
    cosang = (d[:, 0] * hvec[0] + d[:, 1] * hvec[1]) / safe
    cost = dist * (1.0 - k_head * cosang)
    m = int(cand[int(cost.argmin())])
    return (int(fill_idx[m, 1]), int(fill_idx[m, 0]))


def _grow_region(remaining, max_depth, tol, seed_cell, target_bcy, per_m, width, height,
                 heading_uv=None, strip_bias=1.0):
    """Region-grow a contiguous patch of material around ``seed_cell``.

    Cells are consumed in best-first order of a distance-from-seed key, expanding
    only through cells that still hold material (>tol) so the patch stays a single
    connected run. Each cell yields at most ``max_depth`` (one 61 mm layer). Stops
    at ``target_bcy`` or when the connected material around the seed is exhausted.

    Shape control: with the default isotropic key (``strip_bias == 1`` or no
    heading) the key is squared Euclidean distance, giving a **compact blob**.
    When a ``heading_uv`` unit vector (in grid (east, north) = (bx, by) space) is
    supplied with ``strip_bias > 1``, the key makes travel *along* the heading
    cheap (linear) and stepping *perpendicular* expensive (``strip_bias`` x
    squared):

        key = abs(along) + strip_bias * perp^2

    Because the along term is linear, a load fills one blade-wide line end-to-end
    (in both directions from the seed) before it ever steps sideways: the strip
    only widens once it runs out of material along the lane. ``strip_bias`` is
    roughly the lane length (in cells) the grower will chase before it prefers a
    sideways step, so it should exceed a full load's cell count to yield a single
    continuous cut/fill line. This is the scraper's natural continuous pass and
    removes the side-by-side "weave"/blob (and its turns) without changing the
    anchor or the load packing.
    """
    import heapq

    sx, sy = seed_cell
    anisotropic = heading_uv is not None and strip_bias != 1.0
    if anisotropic:
        ux, uy = heading_uv
        vx, vy = uy, -ux  # perpendicular to the heading (rotate -90 deg)
    visited = set()
    used: List[Tuple[Cell, float]] = []
    acc = 0.0
    cnt = 0
    heap = [(0.0, 0, sx, sy)]
    while heap and acc < target_bcy - 1e-9:
        _key, _t, x, y = heapq.heappop(heap)
        if (x, y) in visited:
            continue
        visited.add((x, y))
        rem = remaining[y, x]
        if rem <= tol:
            continue  # empty: don't consume, don't expand through (keeps patch tight)
        depth = min(max_depth, rem)
        bcy = depth * per_m
        if acc + bcy > target_bcy:
            bcy = target_bcy - acc
            depth = bcy / per_m
        remaining[y, x] -= depth
        acc += bcy
        used.append(((x, y), depth))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                    ddx, ddy = nx - sx, ny - sy
                    cnt += 1
                    if anisotropic:
                        along = ddx * ux + ddy * uy
                        perp = ddx * vx + ddy * vy
                        key = abs(along) + strip_bias * perp * perp
                    else:
                        key = float(ddx * ddx + ddy * ddy)
                    heapq.heappush(heap, (key, cnt, nx, ny))
    return used, acc


def _grow_flow_tube(remaining, max_depth, tol, seed_cell, target_bcy, per_m,
                    width, height, fxs, fys, valid, step: float = 0.5,
                    max_steps: int = 4000, miss_gap: int = 4,
                    signs: Tuple[float, ...] = (1.0, -1.0)):
    """Consume one load along the *curving streamline* through ``seed_cell``.

    Integrates the smoothed flow, taking one ``max_depth`` layer from each cell
    that still holds material, giving one long continuous blade-wide lane that
    re-steers with the curving flow - the natural continuous pass for a scraper
    working with the grain.

    ``signs`` selects which way(s) to follow the streamline from the seed:

    * ``(-1.0,)`` upstream only, toward the ridge - used for **cuts** (the anchor
      sits near the fill; we collect high ground back toward the ridge and end
      the cut near the fill). Avoids curling into the convergence zone downstream.
    * ``(1.0,)`` downstream only, into the sink - used for **fills** (start near
      the haul arrival, spread outward down the flow).
    * ``(1.0, -1.0)`` both directions (default).

    Tracing a direction stops after ``miss_gap`` consecutive empty cells (run past
    the material), the load is full, the flow goes flat, or the path leaves the
    field. Returns the cells as one monotonic streamline-ordered run (ridge end ->
    seed -> fill end), so the guidance follows them directly without
    nearest-neighbour re-chaining. Assumes the grid is ~one blade wide.
    """
    sx, sy = seed_cell
    consumed: set = set()
    acc = 0.0

    def consume(bx: int, by: int, bucket: List[Tuple[Cell, float]]) -> bool:
        nonlocal acc
        if (bx, by) in consumed:
            return False
        rem = remaining[by, bx]
        if rem <= tol:
            return False
        depth = min(max_depth, rem)
        bcy = depth * per_m
        if acc + bcy > target_bcy:
            bcy = target_bcy - acc
            depth = bcy / per_m if per_m > 0 else 0.0
            if depth <= 1e-12:
                return False
        remaining[by, bx] -= depth
        acc += bcy
        bucket.append(((bx, by), depth))
        consumed.add((bx, by))
        return True

    seed_entry: List[Tuple[Cell, float]] = []
    consume(sx, sy, seed_entry)

    def _trace(sign: float) -> List[Tuple[Cell, float]]:
        out: List[Tuple[Cell, float]] = []
        x, y = sx + 0.5, sy + 0.5
        misses = 0
        for _ in range(max_steps):
            if acc >= target_bcy - 1e-9:
                break
            ux = _bilinear(fxs, valid, x, y)
            uy = _bilinear(fys, valid, x, y)
            norm = (ux * ux + uy * uy) ** 0.5
            if norm < 1e-6:
                break
            x += sign * step * ux / norm
            y += sign * step * uy / norm
            bx, by = int(math.floor(x)), int(math.floor(y))
            if not (0 <= bx < width and 0 <= by < height) or not valid[by, bx]:
                break
            if consume(bx, by, out):
                misses = 0
            elif (bx, by) not in consumed:
                misses += 1
                if misses >= miss_gap and out:
                    break
        return out

    down = _trace(1.0) if 1.0 in signs else []    # toward the fill (with the flow)
    up = _trace(-1.0) if -1.0 in signs else []     # toward the ridge (against flow)
    ordered = list(reversed(up)) + seed_entry + down
    return ordered, acc


def _centroid(points) -> Tuple[float, float]:
    n = len(points)
    return (sum(p[0] for p in points) / n, sum(p[1] for p in points) / n)


def _nn_chain(cells, grid: Grid, start_idx: int = 0):
    """Order ``cells`` as a nearest-neighbour path from ``start_idx``.

    Patches are small (~10-15 cells) so the O(n^2) cost is negligible.
    """
    if len(cells) <= 2:
        return list(cells)
    pts = [grid.cell_center_xy(*c) for c, _ in cells]
    remaining = list(range(len(cells)))
    remaining.remove(start_idx)
    order = [start_idx]
    while remaining:
        last = pts[order[-1]]
        nxt = min(remaining, key=lambda m: (pts[m][0] - last[0]) ** 2 + (pts[m][1] - last[1]) ** 2)
        remaining.remove(nxt)
        order.append(nxt)
    return [cells[m] for m in order]


def _build_pass(grid: Grid, cut_cells, fill_cells, load_bcy, ordered: bool = False) -> Pass:
    # Order the cut sweep to *end* near the fill patch and the fill sweep to
    # *start* near where the cut ended, so the haul is the true transit leg
    # (cut end -> fill start) and not double-counted against the sweeps.
    fill_c = _centroid([grid.cell_center_xy(*c) for c, _ in fill_cells])
    if ordered:
        # Flow-tube cells are already in streamline order (one flowing run); keep
        # that order and only flip whole sweeps so the cut ends toward the fill
        # and the fill starts toward the cut end. No nearest-neighbour re-chaining
        # (which is what introduced the zig-zag).
        cut_pts = [grid.cell_center_xy(*c) for c, _ in cut_cells]
        if (len(cut_pts) >= 2
                and ((cut_pts[0][0] - fill_c[0]) ** 2 + (cut_pts[0][1] - fill_c[1]) ** 2)
                < ((cut_pts[-1][0] - fill_c[0]) ** 2 + (cut_pts[-1][1] - fill_c[1]) ** 2)):
            cut_cells = list(reversed(cut_cells))
        cut_end = grid.cell_center_xy(*cut_cells[-1][0])
        fill_pts = [grid.cell_center_xy(*c) for c, _ in fill_cells]
        if (len(fill_pts) >= 2
                and ((fill_pts[-1][0] - cut_end[0]) ** 2 + (fill_pts[-1][1] - cut_end[1]) ** 2)
                < ((fill_pts[0][0] - cut_end[0]) ** 2 + (fill_pts[0][1] - cut_end[1]) ** 2)):
            fill_cells = list(reversed(fill_cells))
    else:
        cut_pts = [grid.cell_center_xy(*c) for c, _ in cut_cells]
        cut_start_idx = max(range(len(cut_pts)),
                            key=lambda m: (cut_pts[m][0] - fill_c[0]) ** 2 + (cut_pts[m][1] - fill_c[1]) ** 2)
        cut_cells = _nn_chain(cut_cells, grid, cut_start_idx)
        cut_end = grid.cell_center_xy(*cut_cells[-1][0])

        fill_pts = [grid.cell_center_xy(*c) for c, _ in fill_cells]
        fill_start_idx = min(range(len(fill_pts)),
                             key=lambda m: (fill_pts[m][0] - cut_end[0]) ** 2 + (fill_pts[m][1] - cut_end[1]) ** 2)
        fill_cells = _nn_chain(fill_cells, grid, fill_start_idx)

    cut_start = grid.cell_center_xy(*cut_cells[0][0])
    fill_start = grid.cell_center_xy(*fill_cells[0][0])
    fill_end = grid.cell_center_xy(*fill_cells[-1][0])
    # Haul: transit from the end of loading to the start of dumping.
    haul = math.dist(cut_end, fill_start)
    return Pass(
        cut_cells=cut_cells,
        fill_cells=fill_cells,
        load_bcy=load_bcy,
        haul_distance_m=haul,
        cut_start_xy=cut_start,
        cut_end_xy=cut_end,
        fill_start_xy=fill_start,
        fill_end_xy=fill_end,
    )
