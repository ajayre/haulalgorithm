"""Stage 1 - volume balance (spec §8).

Computes global supply (cut) and demand (fill) in BCY and reports the
imbalance. In ``check`` mode it warns when the relative imbalance exceeds a
threshold; in ``adjust`` mode it applies a single global vertical offset to the
target surface (bisection) to drive supply toward demand.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

import numpy as np

from .grid import Grid

DEFAULT_IMBALANCE_THRESHOLD = 0.02  # 2 percent (spec §8.0.2)


@dataclass
class BalanceResult:
    total_supply_bcy: float
    total_demand_bcy: float
    relative_imbalance: float
    applied_offset_m: float
    mode: str


def _totals(grid: Grid) -> tuple[float, float]:
    return float(grid.supply_grid().sum()), float(grid.demand_grid().sum())


def _relative_imbalance(supply: float, demand: float) -> float:
    denom = max(supply, demand, 1e-9)
    return abs(supply - demand) / denom


def balance(
    grid: Grid,
    mode: str = "check",
    threshold: float = DEFAULT_IMBALANCE_THRESHOLD,
) -> BalanceResult:
    """Compute balance and optionally adjust the target by a vertical offset."""
    supply, demand = _totals(grid)
    applied_offset = 0.0

    if mode == "adjust":
        applied_offset = _solve_offset(grid)
        if applied_offset != 0.0:
            grid.target = np.where(grid.valid, grid.target + applied_offset, grid.target)
        supply, demand = _totals(grid)
        sys.stderr.write(f"INFO: balance adjust applied vertical offset {applied_offset:+.4f} m\n")

    rel = _relative_imbalance(supply, demand)
    if rel > threshold:
        sys.stderr.write(
            f"WARNING: cut/fill imbalance {rel * 100:.1f}% exceeds "
            f"{threshold * 100:.1f}% (supply={supply:.1f} BCY, demand={demand:.1f} BCY)\n"
        )

    return BalanceResult(
        total_supply_bcy=supply,
        total_demand_bcy=demand,
        relative_imbalance=rel,
        applied_offset_m=applied_offset,
        mode=mode,
    )


def _solve_offset(grid: Grid) -> float:
    """Bisection on a global target offset to balance supply and demand (§8.0.3)."""
    base_target = grid.target.copy()
    delta = grid.existing - base_target

    def imbalance_at(offset: float) -> float:
        d = delta - offset  # raising target by offset lowers delta
        supply = np.clip(np.where(grid.valid, d, 0.0), 0.0, None).sum()
        demand = np.clip(np.where(grid.valid, -d, 0.0), 0.0, None).sum()
        # supply is bank BCY; demand is bank cut needed = fill space × cut/fill ratio.
        supply_bcy = supply * grid.area_m2 * 1.30795061931439
        demand_bcy = demand * grid.area_m2 * 1.30795061931439 * grid.compaction
        return supply_bcy - demand_bcy

    valid_delta = delta[grid.valid]
    if valid_delta.size == 0:
        return 0.0
    lo = float(valid_delta.min()) - 1.0
    hi = float(valid_delta.max()) + 1.0
    f_lo = imbalance_at(lo)
    f_hi = imbalance_at(hi)
    if f_lo == 0.0:
        return lo
    if f_hi == 0.0:
        return hi
    if (f_lo > 0) == (f_hi > 0):
        return 0.0  # no sign change; cannot balance with a pure offset

    for _ in range(100):
        mid = 0.5 * (lo + hi)
        f_mid = imbalance_at(mid)
        if abs(f_mid) < 1e-6 or (hi - lo) < 1e-9:
            return mid
        if (f_mid > 0) == (f_lo > 0):
            lo, f_lo = mid, f_mid
        else:
            hi, f_hi = mid, f_mid
    return 0.5 * (lo + hi)
