"""Stage 3 - strip generation by streamline tracing (spec §10).

Strips are traced as streamlines of the flow field, starting at ridges (local
maxima of ``phi`` / highest-delta cut areas) and integrating downhill along
``F`` with bilinear interpolation so paths curve smoothly. A streamline crosses
the cut region, through the transition, and into the fill region, capturing both
the source and the sink of a haul in one strip.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from .flowfield import FlowField, masked_gradient
from .grid import Grid


@dataclass
class Strip:
    """One traced streamline.

    ``cells`` is the ordered list of distinct ``(bx, by)`` areas the strip
    visits (ridge first). ``path`` is the fractional polyline in grid units
    ``(bx_f, by_f)`` used for distance computations.
    """

    cells: List[Tuple[int, int]]
    path: List[Tuple[float, float]]


def _bilinear(field_arr: np.ndarray, valid: np.ndarray, x: float, y: float) -> float:
    """Bilinearly sample ``field_arr`` at fractional grid coords (x=bx, y=by)."""
    h, w = field_arr.shape
    x0 = int(np.floor(x))
    y0 = int(np.floor(y))
    fx = x - x0
    fy = y - y0
    acc = 0.0
    wsum = 0.0
    for (cx, cy, weight) in (
        (x0, y0, (1 - fx) * (1 - fy)),
        (x0 + 1, y0, fx * (1 - fy)),
        (x0, y0 + 1, (1 - fx) * fy),
        (x0 + 1, y0 + 1, fx * fy),
    ):
        if 0 <= cx < w and 0 <= cy < h and valid[cy, cx]:
            acc += field_arr[cy, cx] * weight
            wsum += weight
    if wsum <= 1e-9:
        return 0.0
    return acc / wsum


def _smooth_potential(phi: np.ndarray, valid: np.ndarray, passes: int = 12) -> np.ndarray:
    """Mask-aware box blur of the potential.

    The raw Poisson potential carries a spike at every per-cell source, so nearly
    every cut cell is a local maximum and streamlines stall. Smoothing removes the
    per-cell spikes while preserving the global ridge->sink structure, giving
    streamlines that flow from the ridges across the transition into the fill.
    """
    h, w = phi.shape
    val = valid.astype(np.float64)
    cur = np.where(valid, np.nan_to_num(phi, nan=0.0), 0.0)
    for _ in range(max(0, passes)):
        num = cur * val
        s = num.copy()
        d = val.copy()
        for dj, di in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            s += np.roll(np.roll(num, dj, axis=0), di, axis=1)
            d += np.roll(np.roll(val, dj, axis=0), di, axis=1)
        cur = np.where(valid, s / np.maximum(d, 1e-9), 0.0)
    return np.where(valid, cur, np.nan)


def _gradient_from_phi(phi: np.ndarray, valid: np.ndarray):
    """Unit flow field F = -grad(phi) over valid cells (grid axes)."""
    gx, gy = masked_gradient(phi, valid, h=1.0)
    fx = -gx
    fy = -gy
    mag = np.hypot(fx, fy)
    with np.errstate(invalid="ignore", divide="ignore"):
        ux = np.where(mag > 1e-12, fx / mag, 0.0)
        uy = np.where(mag > 1e-12, fy / mag, 0.0)
    return ux, uy


def _find_ridges(grid: Grid, phi: np.ndarray, max_seeds: int) -> List[Tuple[int, int]]:
    """Seed points: cut cells that are local maxima of (smoothed) phi, ranked by delta."""
    valid = grid.valid
    delta = np.nan_to_num(grid.delta, nan=-1e9)
    h, w = valid.shape
    seeds: List[Tuple[float, int, int]] = []
    for j in range(h):
        for i in range(w):
            if not valid[j, i] or delta[j, i] <= 0.001:
                continue
            here = phi[j, i]
            is_max = True
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    if dj == 0 and di == 0:
                        continue
                    nj, ni = j + dj, i + di
                    if 0 <= nj < h and 0 <= ni < w and valid[nj, ni]:
                        if phi[nj, ni] > here + 1e-12:
                            is_max = False
                            break
                if not is_max:
                    break
            if is_max:
                seeds.append((float(delta[j, i]), i, j))
    seeds.sort(reverse=True)

    # Streamlines started only from the few strict local maxima of the smoothed
    # potential do not cover the whole field. The spec also permits seeding from
    # the highest-delta cut areas (§10.0.1), so trace from every cut cell: ridges
    # first, then the remaining cut cells ordered by depth. This gives full cut
    # coverage; pass generation depletes shared volume across overlapping strips.
    seed_cells = [(i, j) for _d, i, j in seeds]
    seen = set(seed_cells)
    others: List[Tuple[float, int, int]] = []
    for j in range(h):
        for i in range(w):
            if valid[j, i] and delta[j, i] > 0.001 and (i, j) not in seen:
                others.append((float(delta[j, i]), i, j))
    others.sort(reverse=True)
    seed_cells.extend((i, j) for _d, i, j in others)
    return seed_cells[:max_seeds]


def trace_strips(
    grid: Grid,
    flow: FlowField,
    max_seeds: int = 4000,
    step: float = 0.5,
    max_steps: int = 4000,
    smooth_passes: int = 12,
) -> List[Strip]:
    """Trace streamlines from ridge seeds downhill along a smoothed flow field.

    Tracing uses a smoothed potential (and its gradient) so streamlines follow
    the global ridge->transition->fill structure instead of stalling on the
    per-cell source spikes in the raw Poisson potential.
    """
    valid = grid.valid
    h, w = valid.shape
    phi_s = _smooth_potential(flow.phi, valid, passes=smooth_passes)
    fx_s, fy_s = _gradient_from_phi(phi_s, valid)
    seeds = _find_ridges(grid, phi_s, max_seeds)

    strips: List[Strip] = []
    for (si, sj) in seeds:
        x = si + 0.5
        y = sj + 0.5
        path: List[Tuple[float, float]] = [(x, y)]
        cells: List[Tuple[int, int]] = []
        visited = set()
        last_cell = None
        for _ in range(max_steps):
            bx, by = int(np.floor(x)), int(np.floor(y))
            if not (0 <= bx < w and 0 <= by < h) or not valid[by, bx]:
                break
            cell = (bx, by)
            if cell != last_cell:
                if cell in visited:
                    break  # closed loop
                visited.add(cell)
                cells.append(cell)
                last_cell = cell
            ux = _bilinear(fx_s, valid, x, y)
            uy = _bilinear(fy_s, valid, x, y)
            norm = (ux * ux + uy * uy) ** 0.5
            if norm < 1e-6:
                break
            x += step * ux / norm
            y += step * uy / norm
            path.append((x, y))
        if len(cells) >= 1:
            strips.append(Strip(cells=cells, path=path))
    return strips
