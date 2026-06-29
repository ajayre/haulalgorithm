"""Stage 2 - global haul flow field via optimal transport (spec §9).

Two modes:

* ``poisson`` (default, §9.1): solve ``laplacian(phi) = s`` with Neumann
  (zero-flux) boundaries on the field mask and set ``F = -grad(phi)``. This
  minimizes a quadratic (L2) transport cost and gives smooth, curving haul
  directions.
* ``mincostflow`` (§9.2): solve the balanced transportation problem minimizing
  ``sum f*d`` on the grid graph (L1 / distance-optimal). Requires ``networkx``;
  if unavailable the implementation falls back to ``poisson`` with a warning.

The output is a per-area unit direction plus magnitude (§9.3); cells with
negligible magnitude (already at grade) carry no arrow.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

import numpy as np

from .grid import Grid


@dataclass
class FlowField:
    """Per-area potential and flow vectors (image/grid convention).

    ``fx``/``fy`` are flow components in grid axes: ``fx`` east (+bx), ``fy``
    north (+by). ``has_arrow`` marks cells with non-negligible magnitude.
    """

    phi: np.ndarray         # [by, bx] potential (NaN outside mask)
    fx: np.ndarray          # [by, bx] east component (unit-normalized)
    fy: np.ndarray          # [by, bx] north component (unit-normalized)
    magnitude: np.ndarray   # [by, bx] raw gradient magnitude
    has_arrow: np.ndarray   # [by, bx] bool
    mode: str

    def direction_deg(self, bx: int, by: int) -> float:
        """Compass degrees (0=N, 90=E) of the flow at a cell."""
        import math

        return math.degrees(math.atan2(self.fx[by, bx], self.fy[by, bx])) % 360.0


def compute_flow_field(grid: Grid, mode: str = "poisson", arrow_eps_frac: float = 0.02) -> FlowField:
    if mode == "mincostflow":
        try:
            return _mincostflow_field(grid, arrow_eps_frac)
        except ImportError:
            sys.stderr.write(
                "WARNING: transport_mode=mincostflow requires networkx; "
                "falling back to poisson.\n"
            )
            mode = "poisson"
    return _poisson_field(grid, arrow_eps_frac)


# ---------------------------------------------------------------------------
# Poisson potential mode
# ---------------------------------------------------------------------------


def _poisson_field(grid: Grid, arrow_eps_frac: float) -> FlowField:
    valid = grid.valid
    h, w = valid.shape

    # Signed source density: +supply for cut, -demand for fill (per cell).
    s = grid.supply_grid() - grid.demand_grid()
    # Enforce Neumann solvability: zero mean over valid cells.
    n_valid = int(valid.sum())
    if n_valid == 0:
        raise ValueError("No valid cells for flow field.")
    s = np.where(valid, s - s[valid].mean(), 0.0)

    phi = _solve_poisson_neumann(valid, s, grid.grid_size_m)

    fx, fy, mag = _gradient_flow(phi, valid, grid.grid_size_m)
    has_arrow = _arrow_mask(grid, mag, arrow_eps_frac)
    return FlowField(phi=phi, fx=fx, fy=fy, magnitude=mag, has_arrow=has_arrow, mode="poisson")


def _solve_poisson_neumann(valid: np.ndarray, s: np.ndarray, h: float) -> np.ndarray:
    """Solve graph-Laplacian Poisson with zero-flux boundary on ``valid``.

    The graph Laplacian ``L`` satisfies ``L phi = -h^2 * laplacian(phi)``. We
    solve ``L phi = +s * h^2`` (i.e. ``laplacian(phi) = -s``) so that phi is a
    local maximum at the cut sources (s > 0); then ``F = -grad(phi)`` flows from
    the cut ridges (high phi) toward the fill sinks (low phi). One cell is pinned
    to remove the constant null space.
    """
    height, width = valid.shape
    idx = -np.ones((height, width), dtype=np.int64)
    coords = np.argwhere(valid)
    for k, (j, i) in enumerate(coords):
        idx[j, i] = k
    n = len(coords)

    try:
        from scipy.sparse import csr_matrix
        from scipy.sparse.linalg import spsolve

        rows: list[int] = []
        cols: list[int] = []
        data: list[float] = []
        rhs = np.zeros(n, dtype=np.float64)
        h2 = h * h
        for k, (j, i) in enumerate(coords):
            deg = 0
            for dj, di in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nj, ni = j + dj, i + di
                if 0 <= nj < height and 0 <= ni < width and valid[nj, ni]:
                    deg += 1
                    rows.append(k)
                    cols.append(int(idx[nj, ni]))
                    data.append(-1.0)
            rows.append(k)
            cols.append(k)
            data.append(float(deg))
            rhs[k] = s[j, i] * h2
        # Pin one node (k=0) to fix the constant.
        # Replace its row with phi_0 = 0.
        A = csr_matrix((data, (rows, cols)), shape=(n, n)).tolil()
        A.rows[0] = [0]
        A.data[0] = [1.0]
        rhs[0] = 0.0
        sol = spsolve(A.tocsr(), rhs)
    except ImportError:
        sol = _solve_poisson_jacobi(valid, s, h, coords, idx)

    phi = np.full((height, width), np.nan, dtype=np.float64)
    for k, (j, i) in enumerate(coords):
        phi[j, i] = sol[k]
    return phi


def _solve_poisson_jacobi(valid, s, h, coords, idx, iters: int = 20000) -> np.ndarray:
    """NumPy fallback iterative solver (Gauss-Seidel-ish via Jacobi sweeps)."""
    height, width = valid.shape
    phi = np.zeros((height, width), dtype=np.float64)
    h2 = h * h
    neigh = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for _ in range(iters):
        new = phi.copy()
        max_delta = 0.0
        for (j, i) in coords:
            acc = 0.0
            deg = 0
            for dj, di in neigh:
                nj, ni = j + dj, i + di
                if 0 <= nj < height and 0 <= ni < width and valid[nj, ni]:
                    acc += phi[nj, ni]
                    deg += 1
            if deg == 0:
                continue
            val = (acc + s[j, i] * h2) / deg
            max_delta = max(max_delta, abs(val - phi[j, i]))
            new[j, i] = val
        phi = new
        if max_delta < 1e-7:
            break
    out = np.zeros(len(coords), dtype=np.float64)
    for k, (j, i) in enumerate(coords):
        out[k] = phi[j, i]
    return out


def masked_gradient(phi: np.ndarray, valid: np.ndarray, h: float = 1.0):
    """Vectorized one-sided/central gradient of ``phi`` over a valid mask.

    Returns ``(gx, gy)`` where ``gx`` is the east (axis=1) and ``gy`` the north
    (axis=0) derivative, using central differences where both neighbours are
    valid and one-sided differences at mask edges. Spacing ``h`` divides out.
    """
    p = np.where(valid, np.nan_to_num(phi, nan=0.0), 0.0)

    def comp(axis: int) -> np.ndarray:
        plus = np.zeros_like(p)
        minus = np.zeros_like(p)
        plus_v = np.zeros_like(valid)
        minus_v = np.zeros_like(valid)
        if axis == 1:  # east
            plus[:, :-1] = p[:, 1:]
            minus[:, 1:] = p[:, :-1]
            plus_v[:, :-1] = valid[:, 1:]
            minus_v[:, 1:] = valid[:, :-1]
        else:  # north
            plus[:-1, :] = p[1:, :]
            minus[1:, :] = p[:-1, :]
            plus_v[:-1, :] = valid[1:, :]
            minus_v[1:, :] = valid[:-1, :]
        both = plus_v & minus_v
        only_plus = plus_v & ~minus_v
        only_minus = minus_v & ~plus_v
        g = np.zeros_like(p)
        g = np.where(both, (plus - minus) / (2.0 * h), g)
        g = np.where(only_plus, (plus - p) / h, g)
        g = np.where(only_minus, (p - minus) / h, g)
        return np.where(valid, g, 0.0)

    return comp(1), comp(0)


def _gradient_flow(phi: np.ndarray, valid: np.ndarray, h: float):
    """F = -grad(phi) over valid cells, returned as unit vectors plus magnitude."""
    gx, gy = masked_gradient(phi, valid, h)
    fx = -gx
    fy = -gy
    mag = np.hypot(fx, fy)
    with np.errstate(invalid="ignore", divide="ignore"):
        ux = np.where(mag > 1e-12, fx / mag, 0.0)
        uy = np.where(mag > 1e-12, fy / mag, 0.0)
    return ux, uy, mag


def _arrow_mask(grid: Grid, mag: np.ndarray, frac: float) -> np.ndarray:
    """Cells with non-negligible magnitude and meaningful cut/fill get arrows."""
    delta = np.abs(np.nan_to_num(grid.delta, nan=0.0))
    tol_m = 0.0  # arrows shown wherever there is flow + measurable delta
    finite_mag = mag[grid.valid]
    if finite_mag.size == 0:
        return np.zeros_like(grid.valid)
    thresh = frac * float(finite_mag.max()) if finite_mag.max() > 0 else 0.0
    return grid.valid & (mag > thresh) & (delta > 0.001)


# ---------------------------------------------------------------------------
# Min-cost-flow mode (optional reference, §9.2)
# ---------------------------------------------------------------------------


def _mincostflow_field(grid: Grid, arrow_eps_frac: float) -> FlowField:
    import networkx as nx  # raises ImportError -> caller falls back to poisson

    valid = grid.valid
    height, width = valid.shape
    supply = grid.supply_grid()
    demand = grid.demand_grid()

    # Scale to integer units for the network simplex (BCY * SCALE).
    scale = 100.0
    net = np.where(valid, supply - demand, 0.0)
    inode = (np.rint(net * scale)).astype(np.int64)
    # Fix rounding so total demand == total supply.
    diff = int(inode[valid].sum())
    if diff != 0:
        flat_valid = np.argwhere(valid)
        j, i = flat_valid[0]
        inode[j, i] -= diff

    G = nx.DiGraph()
    for (j, i) in np.argwhere(valid):
        G.add_node((int(j), int(i)), demand=-int(inode[j, i]))  # nx demand = -supply

    # 8-connected edges with Euclidean weight; both directions.
    diag = grid.grid_size_m * (2 ** 0.5)
    straight = grid.grid_size_m
    for (j, i) in np.argwhere(valid):
        for dj, di, wcost in (
            (-1, 0, straight), (1, 0, straight), (0, -1, straight), (0, 1, straight),
            (-1, -1, diag), (-1, 1, diag), (1, -1, diag), (1, 1, diag),
        ):
            nj, ni = j + dj, i + di
            if 0 <= nj < height and 0 <= ni < width and valid[nj, ni]:
                G.add_edge((int(j), int(i)), (int(nj), int(ni)), weight=int(round(wcost * 100)))

    flow = nx.min_cost_flow(G)

    fx = np.zeros((height, width), dtype=np.float64)
    fy = np.zeros((height, width), dtype=np.float64)
    for (j, i), targets in flow.items():
        for (nj, ni), f in targets.items():
            if f <= 0:
                continue
            di = ni - i
            dj = nj - j
            norm = (di * di + dj * dj) ** 0.5
            if norm > 0:
                fx[j, i] += f * di / norm
                fy[j, i] += f * dj / norm

    mag = np.hypot(fx, fy)
    with np.errstate(invalid="ignore", divide="ignore"):
        ux = np.where(mag > 1e-12, fx / mag, 0.0)
        uy = np.where(mag > 1e-12, fy / mag, 0.0)
    ux = np.where(valid, ux, 0.0)
    uy = np.where(valid, uy, 0.0)

    # Reuse phi via Poisson for streamline tracing convenience.
    s = np.where(valid, (supply - demand) - (supply - demand)[valid].mean(), 0.0)
    phi = _solve_poisson_neumann(valid, s, grid.grid_size_m)
    has_arrow = _arrow_mask(grid, mag, arrow_eps_frac)
    return FlowField(phi=phi, fx=ux, fy=uy, magnitude=mag, has_arrow=has_arrow, mode="mincostflow")
