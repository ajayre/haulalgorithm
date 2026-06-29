"""30 ft grid model and BCY conversions (spec §7.2, §7.3).

Each area (cell/bin) is a square of edge ``grid_size`` and carries a single
existing and target elevation, treated as uniform over the whole footprint
(spec §7.2.2). Volumes are tracked in banked cubic yards (BCY).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from . import (
    DEFAULT_COMPACTION,
    DEFAULT_GRID_SIZE_FT,
    DEFAULT_MAX_DEPTH_MM,
    DEFAULT_SWELL_FACTOR,
    FEET_TO_METERS,
    M3_TO_CY,
)
from .agd import AgdField


# ---------------------------------------------------------------------------
# BCY conversion pure functions (spec §7.3, unit-tested)
#
# BCY (bank cubic yards) is the in-ground volume. LCY (loose cubic yards, in the
# scraper) relate by the swell factor: LCY = BCY * swell (loose soil is bulkier).
# The cut/fill ratio (compaction) accounts for fill compacting when placed: to
# build one bank yard of fill you must cut ``cutfill_ratio`` bank yards.
# ---------------------------------------------------------------------------


def supply_bcy(delta_m: float, area_m2: float) -> float:
    """Bank cut volume (BCY) available from a cut area (delta > 0). 0 for fill (§7.3.3)."""
    if delta_m <= 0.0:
        return 0.0
    return delta_m * area_m2 * M3_TO_CY


def demand_bcy(
    delta_m: float,
    area_m2: float,
    cutfill_ratio: float = DEFAULT_COMPACTION,
) -> float:
    """Bank cut volume (BCY) that must be delivered to a fill area (delta < 0).

    The fill space itself is ``|delta| * area`` (bank); accounting for compaction
    you must deliver ``cutfill_ratio`` times that in cut soil. 0 for cut areas.
    """
    if delta_m >= 0.0:
        return 0.0
    fill_space_m3 = (-delta_m) * area_m2
    return fill_space_m3 * M3_TO_CY * cutfill_ratio


def lcy_to_bcy(lcy: float, swell_factor: float = DEFAULT_SWELL_FACTOR) -> float:
    """Convert a loose volume (e.g. scraper capacity in LCY) to bank BCY."""
    return lcy / swell_factor


def layers_for_depth(abs_delta_m: float, max_depth_mm: float = DEFAULT_MAX_DEPTH_MM) -> int:
    """Number of <= max_depth passes needed to move ``abs_delta`` (§7.3.6)."""
    if abs_delta_m <= 0.0:
        return 0
    return int(math.ceil(abs_delta_m / (max_depth_mm / 1000.0)))


# ---------------------------------------------------------------------------
# Mask helpers (ported conventions from fielddesign.py)
# ---------------------------------------------------------------------------


def _binary_dilate(mask: np.ndarray, passes: int = 1) -> np.ndarray:
    out = mask.copy()
    for _ in range(max(0, passes)):
        p = np.pad(out, ((1, 1), (1, 1)), mode="constant", constant_values=False)
        out = (
            p[:-2, :-2] | p[:-2, 1:-1] | p[:-2, 2:]
            | p[1:-1, :-2] | p[1:-1, 1:-1] | p[1:-1, 2:]
            | p[2:, :-2] | p[2:, 1:-1] | p[2:, 2:]
        )
    return out


def _binary_erode(mask: np.ndarray, passes: int = 1) -> np.ndarray:
    out = mask.copy()
    for _ in range(max(0, passes)):
        p = np.pad(out, ((1, 1), (1, 1)), mode="constant", constant_values=False)
        out = (
            p[:-2, :-2] & p[:-2, 1:-1] & p[:-2, 2:]
            & p[1:-1, :-2] & p[1:-1, 1:-1] & p[1:-1, 2:]
            & p[2:, :-2] & p[2:, 1:-1] & p[2:, 2:]
        )
    return out


def _fill_holes(binary_mask: np.ndarray) -> np.ndarray:
    h, w = binary_mask.shape
    inv = ~binary_mask
    outside = np.zeros((h, w), dtype=bool)
    stack: List[Tuple[int, int]] = []
    for x in range(w):
        if inv[0, x]:
            stack.append((x, 0))
        if inv[h - 1, x]:
            stack.append((x, h - 1))
    for y in range(h):
        if inv[y, 0]:
            stack.append((0, y))
        if inv[y, w - 1]:
            stack.append((w - 1, y))
    while stack:
        x, y = stack.pop()
        if x < 0 or x >= w or y < 0 or y >= h:
            continue
        if outside[y, x] or not inv[y, x]:
            continue
        outside[y, x] = True
        stack.extend([(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)])
    holes = inv & (~outside)
    return binary_mask | holes


def _largest_component(mask: np.ndarray) -> np.ndarray:
    h, w = mask.shape
    seen = np.zeros((h, w), dtype=bool)
    best: List[Tuple[int, int]] = []
    for y0 in range(h):
        for x0 in range(w):
            if seen[y0, x0] or not mask[y0, x0]:
                continue
            comp: List[Tuple[int, int]] = []
            stack = [(x0, y0)]
            seen[y0, x0] = True
            while stack:
                x, y = stack.pop()
                comp.append((x, y))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < w and 0 <= ny < h and not seen[ny, nx] and mask[ny, nx]:
                            seen[ny, nx] = True
                            stack.append((nx, ny))
            if len(comp) > len(best):
                best = comp
    out = np.zeros((h, w), dtype=bool)
    for x, y in best:
        out[y, x] = True
    return out


def _point_in_polygon(x: float, y: float, ring: List[Tuple[float, float]]) -> bool:
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y):
            x_at_y = (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            if x <= x_at_y:
                inside = not inside
        j = i
    return inside


def _order_boundary_ring(pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Order boundary points into a polygon ring by angle about the centroid."""
    if len(pts) < 3:
        return pts
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


# ---------------------------------------------------------------------------
# Grid container
# ---------------------------------------------------------------------------


@dataclass
class Grid:
    """30 ft grid model (spec §7).

    Arrays are indexed ``[by, bx]`` with ``bx`` east and ``by`` north. ``valid``
    is the field mask; invalid cells have NaN elevations and zero volume.
    """

    grid_size_m: float
    width: int   # number of bins east (bx)
    height: int  # number of bins north (by)
    min_x: float
    min_y: float
    zone: int
    is_northern: bool
    mean_lat: float
    mean_lon: float

    existing: np.ndarray  # [by, bx] metres
    target: np.ndarray    # [by, bx] metres
    valid: np.ndarray     # [by, bx] bool

    swell_factor: float = DEFAULT_SWELL_FACTOR
    compaction: float = DEFAULT_COMPACTION
    max_depth_mm: float = DEFAULT_MAX_DEPTH_MM

    # Exclusion polygons (projected xy rings) the machine may neither level nor
    # drive across (spec §12.3). Stored for rendering and obstacle routing.
    exclusions: List[List[Tuple[float, float]]] = field(default_factory=list)

    @property
    def area_m2(self) -> float:
        return self.grid_size_m * self.grid_size_m

    @property
    def delta(self) -> np.ndarray:
        """existing - target (metres, positive == cut). NaN outside mask."""
        d = self.existing - self.target
        d = np.where(self.valid, d, np.nan)
        return d

    def supply_grid(self) -> np.ndarray:
        """Per-cell bank cut volume (BCY)."""
        d = np.nan_to_num(self.delta, nan=0.0)
        s = np.clip(d, 0.0, None) * self.area_m2 * M3_TO_CY
        return np.where(self.valid, s, 0.0)

    def demand_grid(self) -> np.ndarray:
        """Per-cell bank cut volume (BCY) needed to build the fill (× cut/fill ratio)."""
        d = np.nan_to_num(self.delta, nan=0.0)
        f = np.clip(-d, 0.0, None) * self.area_m2 * M3_TO_CY * self.compaction
        return np.where(self.valid, f, 0.0)

    def cell_center_xy(self, bx: int, by: int) -> Tuple[float, float]:
        return (
            self.min_x + (bx + 0.5) * self.grid_size_m,
            self.min_y + (by + 0.5) * self.grid_size_m,
        )


def build_grid(
    agd: AgdField,
    grid_size_ft: float = DEFAULT_GRID_SIZE_FT,
    swell_factor: float = DEFAULT_SWELL_FACTOR,
    compaction: float = DEFAULT_COMPACTION,
    max_depth_mm: float = DEFAULT_MAX_DEPTH_MM,
) -> Grid:
    """Bin AGD interior points onto a square grid (spec §7.2)."""
    grid_size_m = grid_size_ft * FEET_TO_METERS

    eastings = np.array([p.easting for p in agd.points], dtype=np.float64)
    northings = np.array([p.northing for p in agd.points], dtype=np.float64)
    existing_v = np.array([p.existing for p in agd.points], dtype=np.float64)
    target_v = np.array([p.proposed for p in agd.points], dtype=np.float64)

    min_x = float(eastings.min())
    min_y = float(northings.min())
    max_x = float(eastings.max())
    max_y = float(northings.max())

    width = max(1, int(math.ceil((max_x - min_x) / grid_size_m)) + 1)
    height = max(1, int(math.ceil((max_y - min_y) / grid_size_m)) + 1)

    bx = np.floor((eastings - min_x) / grid_size_m).astype(np.int64)
    by = np.floor((northings - min_y) / grid_size_m).astype(np.int64)
    np.clip(bx, 0, width - 1, out=bx)
    np.clip(by, 0, height - 1, out=by)

    flat = by * width + bx
    n_cells = width * height
    counts = np.bincount(flat, minlength=n_cells).astype(np.float64)
    sum_exist = np.bincount(flat, weights=existing_v, minlength=n_cells)
    sum_target = np.bincount(flat, weights=target_v, minlength=n_cells)

    occupied = counts > 0
    with np.errstate(invalid="ignore", divide="ignore"):
        mean_exist = np.where(occupied, sum_exist / counts, np.nan)
        mean_target = np.where(occupied, sum_target / counts, np.nan)

    existing = mean_exist.reshape(height, width)
    target = mean_target.reshape(height, width)
    occupied2d = occupied.reshape(height, width)

    # Field mask (spec §6.1.12).
    if agd.boundary:
        ring = _order_boundary_ring(
            [(p.easting, p.northing) for p in agd.boundary]
        )
        mask = np.zeros((height, width), dtype=bool)
        for j in range(height):
            cy = min_y + (j + 0.5) * grid_size_m
            for i in range(width):
                cx = min_x + (i + 0.5) * grid_size_m
                mask[j, i] = _point_in_polygon(cx, cy, ring)
        mask |= occupied2d  # always keep observed cells
    else:
        mask = _mask_from_observed(occupied2d)

    # Interpolate elevations for masked cells that have no points (spec §7.2.1).
    existing, target = _fill_missing(existing, target, occupied2d, mask)

    # Cells outside the mask are invalid (NaN).
    existing = np.where(mask, existing, np.nan)
    target = np.where(mask, target, np.nan)
    valid = mask & np.isfinite(existing) & np.isfinite(target)

    return Grid(
        grid_size_m=grid_size_m,
        width=width,
        height=height,
        min_x=min_x,
        min_y=min_y,
        zone=agd.zone,
        is_northern=agd.is_northern,
        mean_lat=agd.mean_lat,
        mean_lon=agd.mean_lon,
        existing=existing,
        target=target,
        valid=valid,
        swell_factor=swell_factor,
        compaction=compaction,
        max_depth_mm=max_depth_mm,
    )


@dataclass
class ExclusionResult:
    """Outcome of removing exclusion-zone bins from the field mask (§12.3)."""

    n_bins_removed: int
    removed_supply_bcy: float   # cut volume that was inside the zones (now exported)
    removed_demand_bcy: float   # fill volume that was inside the zones (not imported)


def apply_exclusions(
    grid: Grid,
    rings: List[List[Tuple[float, float]]],
) -> ExclusionResult:
    """Drop the bins covering each exclusion polygon from the field mask.

    An irregular polygon is mapped to the **rectangular block of bins** spanning
    its projected bounding box (floor of the min corner, the bin containing the
    max corner). That block's bounding box therefore entirely contains the
    polygon (spec §12.3): every bin the polygon touches — and the few that round
    it out to a rectangle — is excluded, so no cut/fill is ever planned inside the
    zone even if the AGD carries bins there. The polygons are stored on the grid
    for obstacle routing and rendering.
    """
    if not rings:
        return ExclusionResult(0, 0.0, 0.0)

    supply_before = grid.supply_grid()
    demand_before = grid.demand_grid()
    removed = np.zeros_like(grid.valid)

    for ring in rings:
        if len(ring) < 3:
            continue
        xs = [p[0] for p in ring]
        ys = [p[1] for p in ring]
        bx_lo = int(math.floor((min(xs) - grid.min_x) / grid.grid_size_m))
        bx_hi = int(math.floor((max(xs) - grid.min_x) / grid.grid_size_m))
        by_lo = int(math.floor((min(ys) - grid.min_y) / grid.grid_size_m))
        by_hi = int(math.floor((max(ys) - grid.min_y) / grid.grid_size_m))
        bx_lo = max(0, min(grid.width - 1, bx_lo))
        bx_hi = max(0, min(grid.width - 1, bx_hi))
        by_lo = max(0, min(grid.height - 1, by_lo))
        by_hi = max(0, min(grid.height - 1, by_hi))
        removed[by_lo:by_hi + 1, bx_lo:bx_hi + 1] = True

    removed &= grid.valid  # only count bins that were part of the field
    n = int(removed.sum())
    rem_supply = float(supply_before[removed].sum())
    rem_demand = float(demand_before[removed].sum())

    grid.valid = grid.valid & (~removed)
    grid.existing = np.where(grid.valid, grid.existing, np.nan)
    grid.target = np.where(grid.valid, grid.target, np.nan)
    grid.exclusions = [list(r) for r in rings if len(r) >= 3]
    return ExclusionResult(n_bins_removed=n, removed_supply_bcy=rem_supply,
                           removed_demand_bcy=rem_demand)


def _mask_from_observed(occupied: np.ndarray) -> np.ndarray:
    """Row/column span fill + hole fill + largest component (spec §6.1.12)."""
    h, w = occupied.shape
    if not occupied.any():
        return np.ones((h, w), dtype=bool)
    row_fill = occupied.copy()
    for j in range(h):
        xs = np.flatnonzero(occupied[j, :])
        if xs.size >= 2:
            row_fill[j, xs[0]: xs[-1] + 1] = True
    col_fill = occupied.copy()
    for i in range(w):
        ys = np.flatnonzero(occupied[:, i])
        if ys.size >= 2:
            col_fill[ys[0]: ys[-1] + 1, i] = True
    envelope = row_fill | col_fill
    closed = _binary_erode(_binary_dilate(envelope, passes=2), passes=2)
    filled = _fill_holes(closed)
    main = _largest_component(filled)
    main |= occupied
    return main


def _fill_missing(
    existing: np.ndarray,
    target: np.ndarray,
    occupied: np.ndarray,
    mask: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Fill masked-but-empty cells by iterative nearest-neighbour averaging."""
    need = mask & (~occupied)
    if not need.any():
        return existing, target

    ex = existing.copy()
    tg = target.copy()
    h, w = existing.shape
    pending = need.copy()
    for _ in range(max(h, w) * 2):
        if not pending.any():
            break
        known = np.isfinite(ex)
        sum_e = np.zeros_like(ex)
        sum_t = np.zeros_like(tg)
        cnt = np.zeros_like(ex)
        ke = np.where(known, ex, 0.0)
        kt = np.where(known, tg, 0.0)
        kf = known.astype(np.float64)
        for dj, di in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            sum_e += np.roll(np.roll(ke, dj, axis=0), di, axis=1)
            sum_t += np.roll(np.roll(kt, dj, axis=0), di, axis=1)
            cnt += np.roll(np.roll(kf, dj, axis=0), di, axis=1)
        can = pending & (cnt > 0)
        ex = np.where(can, sum_e / np.maximum(cnt, 1e-9), ex)
        tg = np.where(can, sum_t / np.maximum(cnt, 1e-9), tg)
        pending = pending & (~can)
    return ex, tg
