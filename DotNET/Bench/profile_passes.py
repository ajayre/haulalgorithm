"""Profile pass-generation operation counts for Python vs C# comparison."""
import sys
import time

sys.path.insert(0, r"C:\Users\andy\OneDrive\Documents\Haul Algorithm\Python")

import numpy as np
import scipy.spatial
from scipy.spatial import cKDTree

from landlevel.agd import parse_agd_file
from landlevel.balance import balance
from landlevel.flowfield import compute_flow_field
from landlevel.grid import build_grid
from landlevel import passes as pm
from landlevel.strips import trace_strips

AGD = r"C:\Users\andy\OneDrive\Documents\Haul Algorithm\DotNET\TheShop2_2ft.agd"
SCRAPER_BCY = 21.0 / 1.25

STATS = {
    "pass_iterations": 0,
    "cut_cells_listed": 0,
    "fill_cells_listed": 0,
    "kd_builds": 0,
    "anchor_batch_queries": 0,
    "nearest_remaining_calls": 0,
    "choose_fill_seed_calls": 0,
    "flow_tube_calls": 0,
    "flow_tube_steps": 0,
    "flow_consumes": 0,
    "prune_calls": 0,
    "build_pass_calls": 0,
}

_orig_grow = pm._grow_flow_tube
_orig_prune = pm._prune_far_fragments
_orig_nearest = pm._nearest_remaining
_orig_build = pm._build_pass
_orig_choose = pm._choose_fill_seed
_orig_ckdtree = cKDTree


class ProfileCKDTree(_orig_ckdtree):
    def __init__(self, data, *args, **kwargs):
        STATS["kd_builds"] += 1
        arr = np.asarray(data)
        if arr.ndim == 2 and arr.shape[1] == 2:
            STATS["fill_cells_listed"] += arr.shape[0]
        super().__init__(data, *args, **kwargs)

    def query(self, x, k=1, **kwargs):
        x_arr = np.asarray(x)
        if x_arr.ndim == 2 and x_arr.shape[1] == 2 and k == 1:
            STATS["anchor_batch_queries"] += 1
            STATS["cut_cells_listed"] += x_arr.shape[0]
        return super().query(x, k=k, **kwargs)


def _grow_flow_tube(*args, **kwargs):
    STATS["flow_tube_calls"] += 1
    remaining, max_depth, tol, seed_cell, target_bcy, per_m, width, height, fxs, fys, valid = args[:11]
    step = kwargs.get("step", 0.5)
    max_steps = kwargs.get("max_steps", 4000)
    miss_gap = kwargs.get("miss_gap", 4)
    signs = kwargs.get("signs", (1.0, -1.0))
    max_gap_cells = kwargs.get("max_gap_cells")

    sx, sy = seed_cell
    consumed = set()
    acc = 0.0

    def consume(bx, by, bucket):
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
        STATS["flow_consumes"] += 1
        return True

    seed_entry = []
    consume(sx, sy, seed_entry)

    def _trace(sign):
        import math

        out = []
        x, y = sx + 0.5, sy + 0.5
        misses = 0
        for _ in range(max_steps):
            STATS["flow_tube_steps"] += 1
            if acc >= target_bcy - 1e-9:
                break
            ux = pm._bilinear(fxs, valid, x, y)
            uy = pm._bilinear(fys, valid, x, y)
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

    down = _trace(1.0) if 1.0 in signs else []
    up = _trace(-1.0) if -1.0 in signs else []
    up.reverse()
    ordered = up + seed_entry + down
    return _orig_prune(ordered, acc, remaining, per_m, max_gap_cells, seed_cell=(sx, sy))


def _patched_generate(*args, **kwargs):
    orig = pm.generate_passes
    # Count guard-loop iterations by wrapping argwhere on remaining matrices.
    orig_argwhere = np.argwhere
    in_pass_loop = {"active": False}

    def counting_argwhere(arr):
        out = orig_argwhere(arr)
        if in_pass_loop["active"] and arr.ndim == 2:
            if out.shape[1] == 2:
                # cut_idx / fill_idx from remaining matrices
                pass
        return out

    # Increment pass_iterations at each while-loop top by patching guard increment via wrapper.
    pr = orig(*args, **kwargs)
    return pr


if __name__ == "__main__":
    pm._grow_flow_tube = _grow_flow_tube
    pm._prune_far_fragments = lambda *a, **k: (STATS.__setitem__("prune_calls", STATS["prune_calls"] + 1) or _orig_prune(*a, **k))
    pm._nearest_remaining = lambda *a, **k: (STATS.__setitem__("nearest_remaining_calls", STATS["nearest_remaining_calls"] + 1) or _orig_nearest(*a, **k))
    pm._build_pass = lambda *a, **k: (STATS.__setitem__("build_pass_calls", STATS["build_pass_calls"] + 1) or _orig_build(*a, **k))
    pm._choose_fill_seed = lambda *a, **k: (STATS.__setitem__("choose_fill_seed_calls", STATS["choose_fill_seed_calls"] + 1) or _orig_choose(*a, **k))

    scipy.spatial.cKDTree = ProfileCKDTree

    field = parse_agd_file(AGD)
    grid = build_grid(field, grid_size_ft=7.5)
    balance(grid, mode="check")
    flow = compute_flow_field(grid, mode="poisson")
    strips = trace_strips(grid, flow)

    t0 = time.perf_counter()
    pr = pm.generate_passes(
        grid,
        strips,
        tolerance_mm=10.0,
        max_load_bcy=SCRAPER_BCY,
        min_load_frac=0.9,
        flow=flow,
        flow_aligned=True,
    )
    elapsed = time.perf_counter() - t0

    STATS["pass_iterations"] = STATS["build_pass_calls"]
    moved = sum(p.load_bcy for p in pr.passes)

    print("--- pass profiler (Python) ---")
    for key in STATS:
        print(f"{key}={STATS[key]}")
    print(f"passes={len(pr.passes)} moved_bcy={moved:.2f} elapsed_s={elapsed:.2f}")

    # KD-tree microbench: build + batch query vs per-point query
    cut_n = 5000
    rng = np.random.default_rng(0)
    pts = rng.random((cut_n, 2))
    fill_pts = rng.random((8000, 2))
    t_build = time.perf_counter()
    for _ in range(100):
        tree = _orig_ckdtree(fill_pts)
    build_ms = (time.perf_counter() - t_build) * 10.0
    tree = _orig_ckdtree(fill_pts)
    t_batch = time.perf_counter()
    for _ in range(100):
        tree.query(pts, k=1)
    batch_ms = (time.perf_counter() - t_batch) * 10.0
    t_single = time.perf_counter()
    for _ in range(100):
        for i in range(cut_n):
            tree.query(pts[i], k=1)
    single_ms = (time.perf_counter() - t_single) * 10.0
    print("--- kd microbench (Python scipy, n_cut=5000 n_fill=8000 x100) ---")
    print(f"build_ms={build_ms:.1f} batch_query_ms={batch_ms:.1f} per_point_query_ms={single_ms:.1f}")
