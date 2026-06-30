"""Benchmark TheShop2 plan+route stages for Python vs C# comparison.

Uses the same defaults as ``landlevel.cli`` / ``HaulBuilderOptions`` so results
are directly comparable (pass count, haul distance, modeled hours, wall time).
"""
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Python"))

from landlevel import (
    DEFAULT_ACCEL_TIME_S,
    DEFAULT_BLADE_WIDTH_MM,
    DEFAULT_DECEL_TIME_S,
    DEFAULT_DUMPING_SPEED_MPH,
    DEFAULT_FIELD_EFFICIENCY,
    DEFAULT_GRID_SIZE_FT,
    DEFAULT_HAUL_SPEED_MPH,
    DEFAULT_LOADING_SPEED_MPH,
    DEFAULT_MAX_DROP_HOP_FT,
    DEFAULT_RETURN_SPEED_MPH,
    DEFAULT_SCRAPER_CAPACITY_LCY,
    DEFAULT_SWELL_FACTOR,
    DEFAULT_TOLERANCE_MM,
    DEFAULT_TURNING_CIRCLE_FT,
    DEFAULT_TURN_SPEED_MPH,
    FEET_TO_METERS,
)
from landlevel.agd import parse_agd_file
from landlevel.balance import balance
from landlevel.flowfield import compute_flow_field
from landlevel.grid import build_grid
from landlevel.obstacles import EMPTY_OBSTACLES
from landlevel.passes import generate_passes, estimate_slope_heading
from landlevel.routing import sequence_passes
from landlevel.strips import trace_strips
from landlevel.timemodel import TimeParams, estimate_time
from landlevel.swaths import reprofile_knots

AGD = REPO / "DotNET" / "TheShop2_2ft.agd"
SCRAPER_BCY = DEFAULT_SCRAPER_CAPACITY_LCY / DEFAULT_SWELL_FACTOR

t0 = time.perf_counter()
field = parse_agd_file(AGD)
t1 = time.perf_counter()
grid = build_grid(field, grid_size_ft=DEFAULT_GRID_SIZE_FT)
t2 = time.perf_counter()
balance(grid, mode="check")
flow = compute_flow_field(grid, mode="poisson")
t3 = time.perf_counter()
strips = trace_strips(grid, flow, max_seeds=400)
t4 = time.perf_counter()
pr = generate_passes(
    grid,
    strips,
    tolerance_mm=DEFAULT_TOLERANCE_MM,
    max_load_bcy=SCRAPER_BCY,
    max_drop_hop_m=DEFAULT_MAX_DROP_HOP_FT * FEET_TO_METERS,
    flow=flow,
    flow_aligned=True,
    min_load_frac=0.9,
    obstacles=EMPTY_OBSTACLES,
)
t5 = time.perf_counter()
route = sequence_passes(
    pr.passes,
    grade_return=True,
    direction_aware=True,
    return_speed_mph=DEFAULT_RETURN_SPEED_MPH,
    turn_speed_mph=DEFAULT_TURN_SPEED_MPH,
    turning_circle_ft=DEFAULT_TURNING_CIRCLE_FT,
    obstacles=EMPTY_OBSTACLES,
)
t6 = time.perf_counter()
reprofile_knots(
    route,
    grid,
    blade_width_mm=DEFAULT_BLADE_WIDTH_MM,
    heading_deg=estimate_slope_heading(grid),
    lane_overlap=0.0,
)
t7 = time.perf_counter()
moved = sum(p.load_bcy for p in pr.passes)
tr = estimate_time(
    grid,
    route,
    TimeParams(
        DEFAULT_LOADING_SPEED_MPH,
        DEFAULT_HAUL_SPEED_MPH,
        DEFAULT_DUMPING_SPEED_MPH,
        DEFAULT_RETURN_SPEED_MPH,
        DEFAULT_ACCEL_TIME_S,
        DEFAULT_DECEL_TIME_S,
        DEFAULT_FIELD_EFFICIENCY,
    ),
)
t8 = time.perf_counter()

print(f"fixture={AGD}")
print(f"grid={grid.width}x{grid.height}")
print(f"parse+grid: {t1 - t0:.2f}s")
print(f"flow: {t3 - t2:.2f}s")
print(f"strips: {t4 - t3:.2f}s")
print(f"passes({len(pr.passes)}): {t5 - t4:.2f}s")
print(f"route: {t6 - t5:.2f}s")
print(f"knots: {t7 - t6:.2f}s")
print(f"metrics: {t8 - t7:.2f}s")
print(f"moved_bcy={moved:.2f} haul_m={tr.hauling_dist_m:.1f} hours={tr.total_time_s / 3600:.2f}")
print(f"total: {t8 - t0:.2f}s")
