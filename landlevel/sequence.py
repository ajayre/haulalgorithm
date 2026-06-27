"""Rough cut-location sequencing (zone-level work order).

The full plan already orders every pass (``routing.sequence_passes``); this
module summarises that fine-grained order into a handful of spatial **zones** so
the result is human-digestible: "work zone 1 first (NE shoulder, 120 loads),
then zone 2, ...". Cut-patch centroids of the routed passes are clustered with a
deterministic k-means; each zone reports its load count, moved volume, centroid
(local XY and lat/lon), and where in the execution it is mostly worked.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .agd import AgdField, latlon_from_utm
from .grid import Grid
from .routing import Route


@dataclass
class ZoneStep:
    work_order: int          # 1 = worked earliest
    zone_id: int             # stable cluster id
    n_loads: int
    total_bcy: float
    easting: float
    northing: float
    lat: float
    lon: float
    first_pass: int          # 1-based index in the routed sequence
    last_pass: int
    mean_pass: float
    compass: str             # rough position within the field (e.g. "NE")
    # Exact cell-boundary outline of the zone footprint: list of world-metre
    # segments ((x1, y1), (x2, y2)) tracing the perimeter of the zone's cells.
    outline: List[Tuple[Tuple[float, float], Tuple[float, float]]] = None


Cell = Tuple[int, int]


def _zone_boundary_edges(cells: set, grid, half: float):
    """Perimeter segments (world m) of a set of grid cells (exact, jagged).

    An edge is on the boundary when the neighbouring cell across it is not in
    the same zone. Returns a list of ((x1, y1), (x2, y2)) segments.
    """
    segs = []
    for (bx, by) in cells:
        cx, cy = grid.cell_center_xy(bx, by)
        x0, x1 = cx - half, cx + half
        y0, y1 = cy - half, cy + half
        if (bx + 1, by) not in cells:      # east edge
            segs.append(((x1, y0), (x1, y1)))
        if (bx - 1, by) not in cells:      # west edge
            segs.append(((x0, y0), (x0, y1)))
        if (bx, by + 1) not in cells:      # north edge (world +y)
            segs.append(((x0, y1), (x1, y1)))
        if (bx, by - 1) not in cells:      # south edge
            segs.append(((x0, y0), (x1, y0)))
    return segs


def _kmeans(points: np.ndarray, k: int, seed: int, iters: int = 100):
    """Deterministic k-means++ (small k, ~hundreds of points)."""
    rng = np.random.default_rng(seed)
    n = points.shape[0]
    first = int(rng.integers(n))
    centers = [points[first]]
    for _ in range(1, k):
        d2 = np.min(
            np.stack([((points - c) ** 2).sum(axis=1) for c in centers], axis=0), axis=0
        )
        total = d2.sum()
        if total <= 0:
            centers.append(points[int(rng.integers(n))])
            continue
        idx = int(rng.choice(n, p=d2 / total))
        centers.append(points[idx])
    centers = np.asarray(centers, dtype=float)

    labels = np.zeros(n, dtype=int)
    for _ in range(iters):
        dists = ((points[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)
        new_labels = dists.argmin(axis=1)
        new_centers = np.array([
            points[new_labels == j].mean(axis=0) if np.any(new_labels == j) else centers[j]
            for j in range(k)
        ])
        if np.array_equal(new_labels, labels) and np.allclose(new_centers, centers):
            labels = new_labels
            break
        labels, centers = new_labels, new_centers
    return labels, centers


def _compass(x: float, y: float, cx: float, cy: float) -> str:
    ns = "N" if y >= cy else "S"
    ew = "E" if x >= cx else "W"
    # Use a single letter when close to an axis, else the quadrant.
    span_x = abs(x - cx)
    span_y = abs(y - cy)
    if span_x < 0.3 * (span_x + span_y + 1e-9):
        return ns
    if span_y < 0.3 * (span_x + span_y + 1e-9):
        return ew
    return ns + ew


def sequence_cut_zones(
    grid: Grid,
    field: AgdField,
    route: Route,
    n_zones: int = 8,
    seed: int = 0,
) -> List[ZoneStep]:
    """Cluster the routed cut patches into ``n_zones`` and order them by when worked."""
    passes = route.ordered_passes
    if not passes:
        return []

    centroids = []
    loads = []
    pass_cut_cells: List[List[Tuple[Cell, float]]] = []
    for p in passes:
        pts = [grid.cell_center_xy(*c) for c, _d in p.cut_cells]
        cx = sum(q[0] for q in pts) / len(pts)
        cy = sum(q[1] for q in pts) / len(pts)
        centroids.append((cx, cy))
        loads.append(p.load_bcy)
        pass_cut_cells.append(list(p.cut_cells))
    centroids = np.asarray(centroids, dtype=float)
    loads = np.asarray(loads, dtype=float)
    exec_idx = np.arange(len(passes), dtype=float)

    k = max(1, min(n_zones, centroids.shape[0]))
    labels, centers = _kmeans(centroids, k, seed)

    field_cx = float(centroids[:, 0].mean())
    field_cy = float(centroids[:, 1].mean())

    # Assign each cut cell to the zone that removed the most depth there, so the
    # cell-boundary outlines partition the field with no overlap between zones.
    cell_zone_depth: dict = {}
    for pi, cells in enumerate(pass_cut_cells):
        zj = int(labels[pi])
        for (cx_i, cy_i), depth in cells:
            key = (cx_i, cy_i)
            arr = cell_zone_depth.get(key)
            if arr is None:
                arr = [0.0] * k
                cell_zone_depth[key] = arr
            arr[zj] += depth
    zone_cells = [set() for _ in range(k)]
    for key, arr in cell_zone_depth.items():
        zone_cells[int(np.argmax(arr))].add(key)

    half = grid.grid_size_m / 2.0

    raw = []
    for j in range(k):
        mask = labels == j
        if not np.any(mask):
            continue
        w = loads[mask]
        wsum = float(w.sum()) or 1.0
        cx = float((centroids[mask, 0] * w).sum() / wsum)
        cy = float((centroids[mask, 1] * w).sum() / wsum)
        mean_pass = float((exec_idx[mask] * w).sum() / wsum)
        lat, lon = latlon_from_utm(cx, cy, field.zone, field.is_northern)
        outline = _zone_boundary_edges(zone_cells[j], grid, half)
        raw.append({
            "zone_id": j,
            "n_loads": int(mask.sum()),
            "total_bcy": float(w.sum()),
            "easting": cx, "northing": cy,
            "lat": lat, "lon": lon,
            "first_pass": int(exec_idx[mask].min()) + 1,
            "last_pass": int(exec_idx[mask].max()) + 1,
            "mean_pass": mean_pass,
            "compass": _compass(cx, cy, field_cx, field_cy),
            "outline": outline,
        })

    raw.sort(key=lambda z: z["mean_pass"])
    steps: List[ZoneStep] = []
    for order, z in enumerate(raw, start=1):
        steps.append(ZoneStep(work_order=order, **z))
    return steps


def write_pass_pairs_csv(field: AgdField, route: Route, path) -> None:
    """Write every cut/fill pass, in the optimized route order, with lat/lon.

    Each row is one cut-haul-fill pair: the start and end (lat, lon) of both the
    cut segment and the fill segment, plus the carried load and haul distance.
    """
    import csv

    zone, north = field.zone, field.is_northern

    def ll(xy):
        lat, lon = latlon_from_utm(xy[0], xy[1], zone, north)
        return f"{lat:.7f}", f"{lon:.7f}"

    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow([
            "order", "load_bcy", "haul_m",
            "cut_start_lat", "cut_start_lon", "cut_end_lat", "cut_end_lon",
            "fill_start_lat", "fill_start_lon", "fill_end_lat", "fill_end_lon",
        ])
        for i, p in enumerate(route.ordered_passes, start=1):
            cs_lat, cs_lon = ll(p.cut_start_xy)
            ce_lat, ce_lon = ll(p.cut_end_xy)
            fs_lat, fs_lon = ll(p.fill_start_xy)
            fe_lat, fe_lon = ll(p.fill_end_xy)
            w.writerow([
                i, f"{p.load_bcy:.2f}", f"{p.haul_distance_m:.1f}",
                cs_lat, cs_lon, ce_lat, ce_lon,
                fs_lat, fs_lon, fe_lat, fe_lon,
            ])


def write_sequence_csv(steps: List[ZoneStep], path) -> None:
    import csv

    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow([
            "work_order", "zone_id", "compass", "loads", "moved_bcy",
            "easting_m", "northing_m", "lat", "lon",
            "first_pass", "last_pass", "mean_pass",
        ])
        for s in steps:
            w.writerow([
                s.work_order, s.zone_id, s.compass, s.n_loads, f"{s.total_bcy:.1f}",
                f"{s.easting:.2f}", f"{s.northing:.2f}", f"{s.lat:.7f}", f"{s.lon:.7f}",
                s.first_pass, s.last_pass, f"{s.mean_pass:.0f}",
            ])
