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

from . import M3_TO_CY
from .grid import Grid
from .strips import Strip

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


def estimate_slope_heading(grid: Grid) -> float:
    """Compass bearing (deg, 0=N, 90=E) of the target surface's main down-slope.

    Soil on a sloped design surface is moved down-slope; this is used as the
    default haul heading when one is not supplied.
    """
    t = np.where(grid.valid, grid.target, np.nan)
    gy, gx = np.gradient(t)  # gx along +x (east), gy along +y (north)
    e = -np.nanmean(gx)      # down-slope points opposite the gradient
    n = -np.nanmean(gy)
    if not np.isfinite(e) or not np.isfinite(n) or (abs(e) < 1e-12 and abs(n) < 1e-12):
        return 0.0
    return math.degrees(math.atan2(e, n)) % 360.0


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
) -> PassResult:
    """Generate cut-haul-fill passes packed to ``max_load_bcy`` (spec §11).

    Each pass cuts a compact, *contiguous* patch of cut cells (region grown from
    an anchor) and deposits into a compact, contiguous fill patch, so the swept
    loading/dumping distances stay near their minimum. When ``heading_deg`` is
    given, fill selection is biased to move soil down that bearing (the field's
    main slope), emulating OptiSurface's lane-oriented hauling. ``strips`` is
    accepted for API/spec continuity (streamlines inform the visualization).
    """
    tol_m = tolerance_mm / 1000.0
    led = _Ledger(grid, tol_m)
    cut_per_m = led.cut_bcy_per_m()
    fill_per_m = led.fill_bcy_per_m()
    W, H = grid.width, grid.height

    hvec = _heading_vector(heading_deg) if heading_deg is not None else None
    k_head = max(0.0, min(0.95, heading_strength)) if hvec is not None else 0.0

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

        # Anchor: cut cell whose nearest remaining fill is closest (shortest haul).
        if have_kd:
            ftree = cKDTree(fill_xy)
            dists, _nn = ftree.query(cut_xy, k=1)
            a = int(np.argmin(dists))
        else:
            a = _anchor_bruteforce(cut_xy, fill_xy)
        anchor = (int(cut_idx[a, 1]), int(cut_idx[a, 0]))

        # Grow a compact cut patch outward from the anchor through cut material.
        cut_cells, load = _grow_region(led.remaining_cut, led.max_depth_m, tol_m,
                                       anchor, load_cap, cut_per_m, W, H)
        if not cut_cells:
            break
        cut_centroid = _centroid([grid.cell_center_xy(*c) for c, _ in cut_cells])

        # Choose a fill seed near the cut (optionally biased down-heading), then
        # grow a compact fill patch from it.
        fseed = _choose_fill_seed(fill_xy, fill_idx, cut_centroid, hvec, k_head, have_kd)
        fill_cells, placed = _grow_region(led.remaining_fill, led.max_depth_m, tol_m,
                                          fseed, load, fill_per_m, W, H)
        if not fill_cells:
            for (c, d) in cut_cells:
                led.remaining_cut[c[1], c[0]] += d
            break
        # Restore any unplaced remainder so volume is conserved.
        leftover = load - placed
        if leftover > 1e-9:
            for (c, d) in reversed(cut_cells):
                if leftover <= 1e-9:
                    break
                give = min(leftover, d * cut_per_m)
                led.remaining_cut[c[1], c[0]] += give / cut_per_m
                leftover -= give

        passes.append(_build_pass(grid, cut_cells, fill_cells, placed))

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


def _grow_region(remaining, max_depth, tol, seed_cell, target_bcy, per_m, width, height):
    """Region-grow a compact, contiguous patch of material around ``seed_cell``.

    Cells are consumed in order of distance from the seed, expanding only through
    cells that still hold material (>tol), so the patch stays a connected blob.
    Each cell yields at most ``max_depth`` (one 61 mm layer). Stops at
    ``target_bcy`` or when the connected material around the seed is exhausted.
    """
    import heapq

    sx, sy = seed_cell
    visited = set()
    used: List[Tuple[Cell, float]] = []
    acc = 0.0
    cnt = 0
    heap = [(0, 0, sx, sy)]
    while heap and acc < target_bcy - 1e-9:
        _d2, _t, x, y = heapq.heappop(heap)
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
                    heapq.heappush(heap, (ddx * ddx + ddy * ddy, cnt, nx, ny))
    return used, acc


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


def _build_pass(grid: Grid, cut_cells, fill_cells, load_bcy) -> Pass:
    # Order the cut sweep to *end* near the fill patch and the fill sweep to
    # *start* near where the cut ended, so the haul is the true transit leg
    # (cut end -> fill start) and not double-counted against the sweeps.
    fill_c = _centroid([grid.cell_center_xy(*c) for c, _ in fill_cells])
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
