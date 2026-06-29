"""Stage 6 - time model (spec §13), matched to the OptiSurface OptiHaul model.

For each pass the machine performs four motions: loading (cutting), hauling,
dumping (filling) and returning. The return distance equals the loading +
hauling + dumping distance (every forward trip has an equal-length return).

Loading and dumping run at their constant configured speeds. Hauling and
returning use a trapezoidal speed profile: the machine accelerates for
``accel_time`` and decelerates for ``decel_time``, which reduces the *average*
haul/return speed below the configured maximum. Field efficiency accounts for
non-productive ("Other") time: ``total = productive / field_efficiency``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from . import MPH_TO_MPS
from .passes import Pass
from .routing import Route


@dataclass
class TimeParams:
    loading_speed_mph: float
    haul_speed_mph: float       # maximum haul speed
    dumping_speed_mph: float
    return_speed_mph: float     # maximum return speed
    accel_time_s: float
    decel_time_s: float
    field_efficiency: float = 0.8  # 80%: accounts for "Other" (lost) time


@dataclass
class TimeResult:
    total_time_s: float
    # Distances (metres).
    loading_dist_m: float
    hauling_dist_m: float
    dumping_dist_m: float
    returning_dist_m: float
    # Average achieved speeds (mph).
    avg_haul_speed_mph: float
    avg_return_speed_mph: float
    # Times (seconds).
    loading_time_s: float
    hauling_time_s: float
    dumping_time_s: float
    returning_time_s: float
    other_time_s: float

    # Back-compat aliases used elsewhere.
    @property
    def total_haul_distance_m(self) -> float:
        return self.hauling_dist_m

    @property
    def total_deadhead_m(self) -> float:
        return self.returning_dist_m


def _segment_length(cells_xy: List) -> float:
    if len(cells_xy) < 2:
        return 0.0
    return sum(math.dist(cells_xy[k], cells_xy[k + 1]) for k in range(len(cells_xy) - 1))


def _move_time(length_m: float, vmax_mps: float, accel_s: float, decel_s: float) -> float:
    """Travel time over ``length_m`` with a trapezoidal speed profile.

    Accelerates at ``vmax/accel_s`` and decelerates at ``vmax/decel_s``. If the
    move is too short to reach ``vmax`` the profile is triangular (a lower peak
    speed is reached). This reduces the average speed below ``vmax``.
    """
    if length_m <= 0.0 or vmax_mps <= 0.0:
        return 0.0
    d_acc = 0.5 * vmax_mps * accel_s
    d_dec = 0.5 * vmax_mps * decel_s
    if length_m >= d_acc + d_dec:
        cruise = length_m - d_acc - d_dec
        return accel_s + decel_s + cruise / vmax_mps
    # Triangular profile: never reaches vmax.
    a = vmax_mps / accel_s
    b = vmax_mps / decel_s
    vp = math.sqrt(2.0 * length_m / (1.0 / a + 1.0 / b))
    return vp / a + vp / b


def estimate_time(grid, route: Route, params: TimeParams) -> TimeResult:
    """Total time to level the field, summed over the sequenced passes (§13)."""
    load_v = params.loading_speed_mph * MPH_TO_MPS
    haul_vmax = params.haul_speed_mph * MPH_TO_MPS
    dump_v = params.dumping_speed_mph * MPH_TO_MPS
    return_vmax = params.return_speed_mph * MPH_TO_MPS
    ta, td = params.accel_time_s, params.decel_time_s

    d_load = d_haul = d_dump = 0.0
    t_load = t_haul = t_dump = t_return = 0.0

    for p in route.ordered_passes:
        cut_xy = [grid.cell_center_xy(*c) for c, _d in p.cut_cells]
        fill_xy = [grid.cell_center_xy(*c) for c, _d in p.fill_cells]
        cut_len = _segment_length(cut_xy)
        fill_len = _segment_length(fill_xy)
        haul_len = p.haul_distance_m

        d_load += cut_len
        d_haul += haul_len
        d_dump += fill_len

        # Loading and dumping at constant speed (no accel/decel reduction).
        t_load += cut_len / load_v if load_v > 0 else 0.0
        t_dump += fill_len / dump_v if dump_v > 0 else 0.0
        # Hauling at reduced average speed (trapezoidal).
        t_haul += _move_time(haul_len, haul_vmax, ta, td)
        # Return mirrors each forward segment, travelled at return speed.
        t_return += _move_time(cut_len, return_vmax, ta, td)
        t_return += _move_time(haul_len, return_vmax, ta, td)
        t_return += _move_time(fill_len, return_vmax, ta, td)

    d_return = d_load + d_haul + d_dump
    productive = t_load + t_haul + t_dump + t_return
    eff = params.field_efficiency if 0.0 < params.field_efficiency <= 1.0 else 1.0
    total = productive / eff
    other = total - productive

    avg_haul = (d_haul / t_haul / MPH_TO_MPS) if t_haul > 0 else params.haul_speed_mph
    avg_return = (d_return / t_return / MPH_TO_MPS) if t_return > 0 else params.return_speed_mph

    return TimeResult(
        total_time_s=total,
        loading_dist_m=d_load,
        hauling_dist_m=d_haul,
        dumping_dist_m=d_dump,
        returning_dist_m=d_return,
        avg_haul_speed_mph=avg_haul,
        avg_return_speed_mph=avg_return,
        loading_time_s=t_load,
        hauling_time_s=t_haul,
        dumping_time_s=t_dump,
        returning_time_s=t_return,
        other_time_s=other,
    )
