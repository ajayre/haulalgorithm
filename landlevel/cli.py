"""Command-line interface (spec §15)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from . import (
    DEFAULT_ACCEL_TIME_S,
    DEFAULT_BLADE_WIDTH_MM,
    DEFAULT_COMPACTION,
    DEFAULT_DECEL_TIME_S,
    DEFAULT_DUMPING_SPEED_MPH,
    DEFAULT_FIELD_EFFICIENCY,
    DEFAULT_GRADE_SPEED_MPH,
    DEFAULT_GRID_SIZE_FT,
    DEFAULT_HAUL_SPEED_MPH,
    DEFAULT_LOADING_SPEED_MPH,
    DEFAULT_MAX_DEPTH_MM,
    DEFAULT_MAX_DROP_HOP_FT,
    DEFAULT_PIXELS_PER_BIN,
    FEET_TO_METERS,
    DEFAULT_RETURN_SPEED_MPH,
    DEFAULT_SCRAPER_CAPACITY_LCY,
    DEFAULT_SWELL_FACTOR,
    DEFAULT_TOLERANCE_MM,
    DEFAULT_TURN_SPEED_MPH,
    DEFAULT_TURN_THRESHOLD_DEG,
    DEFAULT_TURNING_CIRCLE_FT,
    __version__,
)
from .agd import parse_agd_file
from .balance import balance
from .flowfield import compute_flow_field
from .grid import build_grid
from .passes import estimate_slope_heading, generate_passes
from .animate import animate_leveling
from .render import (
    overlay_zone_waypoints,
    render_guidance_png,
    render_pass_pairs_png,
    render_png,
)
from .routing import sequence_passes
from .sequence import sequence_cut_zones, write_pass_pairs_csv, write_sequence_csv
from .swaths import build_guidance, estimate_guidance_time, write_guidance_csv
from .strips import trace_strips
from .timemodel import TimeParams, estimate_time


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="landlevel",
        description="Land leveling haul algorithm: AGD -> haul plan + cut/fill PNG.",
    )
    p.add_argument("--agd", required=True, type=Path, help="input AGD file")
    p.add_argument("--out", required=True, type=Path, help="output PNG path")
    p.add_argument("--grid-size-ft", type=float, default=DEFAULT_GRID_SIZE_FT)
    p.add_argument("--tolerance-mm", type=float, default=DEFAULT_TOLERANCE_MM)
    p.add_argument("--scraper-capacity-lcy", type=float, default=DEFAULT_SCRAPER_CAPACITY_LCY,
                   help="scraper capacity in loose CY (LCY = BCY × swell)")
    p.add_argument("--swell-factor", type=float, default=DEFAULT_SWELL_FACTOR,
                   help="LCY = BCY × swell (loose soil is bulkier)")
    p.add_argument("--cutfill-ratio", "--compaction", dest="cutfill_ratio", type=float,
                   default=DEFAULT_COMPACTION,
                   help="cut/fill ratio: bank cut volume needed per bank fill volume")
    p.add_argument("--max-depth-mm", type=float, default=DEFAULT_MAX_DEPTH_MM)
    p.add_argument("--blade-width-mm", type=float, default=DEFAULT_BLADE_WIDTH_MM)
    p.add_argument("--heading-deg", type=float, default=None,
                   help="haul heading / grid direction (compass deg, 0=N, 90=E). "
                        "Default: auto-detected main slope of the design surface.")
    p.add_argument("--heading-strength", type=float, default=0.5,
                   help="0-0.95: how strongly fill selection follows the heading "
                        "(0 disables heading bias)")
    p.add_argument("--strip-bias", type=float, default=50.0,
                   help="grow each load's cut/fill as one long blade-wide lane "
                        "along the heading. Value ~= the lane length (in cells) the "
                        "grower chases before stepping sideways; set it above a "
                        "full load's cell count for a single continuous line "
                        "(1 = compact blob).")
    p.add_argument("--max-drop-hop-ft", type=float, default=DEFAULT_MAX_DROP_HOP_FT,
                   help="multi-drop: how far the scraper will drive (blade up) to "
                        "the next cut/fill pocket to keep loading/spreading the same "
                        "load, instead of ending a pass partially full (default "
                        "%(default)g ft). Larger = fewer, fuller passes but longer "
                        "in-pass repositioning; 0 disables multi-drop.")
    p.add_argument("--field-efficiency", type=float, default=DEFAULT_FIELD_EFFICIENCY,
                   help="fraction of productive time (0-1); 0.8 = 80%%")
    p.add_argument("--loading-speed-mph", type=float, default=DEFAULT_LOADING_SPEED_MPH)
    p.add_argument("--haul-speed-mph", type=float, default=DEFAULT_HAUL_SPEED_MPH)
    p.add_argument("--dumping-speed-mph", type=float, default=DEFAULT_DUMPING_SPEED_MPH)
    p.add_argument("--return-speed-mph", type=float, default=DEFAULT_RETURN_SPEED_MPH)
    p.add_argument("--accel-time-s", type=float, default=DEFAULT_ACCEL_TIME_S)
    p.add_argument("--decel-time-s", type=float, default=DEFAULT_DECEL_TIME_S)
    p.add_argument("--transport-mode", choices=["poisson", "mincostflow"], default="poisson")
    p.add_argument("--balance-mode", choices=["check", "adjust"], default="check")
    p.add_argument("--pixels-per-bin", type=int, default=DEFAULT_PIXELS_PER_BIN)
    p.add_argument("--arrow-stride", type=int, default=2, help="arrows every N areas")
    p.add_argument("--zones", type=int, default=8,
                   help="number of cut zones for the rough work-order sequence (0 disables)")
    p.add_argument("--sequence-csv", type=Path, default=None,
                   help="write the cut-zone work order to this CSV "
                        "(default: <out>_cut_sequence.csv)")
    p.add_argument("--pairs-csv", nargs="?", const="__AUTO__", default=None,
                   help="write every cut/fill pass (optimized order) with start/end "
                        "lat/lon to this CSV; pass the flag alone for "
                        "<out>_cut_fill_pairs.csv")
    p.add_argument("--pairs-png", nargs="?", const="__AUTO__", default=None,
                   help="write a cut/fill PNG overlaid with one semi-transparent "
                        "line per cut-fill pair (heat-map look); pass the flag "
                        "alone for <out>_cut_fill_pairs.png")
    p.add_argument("--guidance-csv", nargs="?", const="__AUTO__", default=None,
                   help="write the blade-scale operator guidance track (turn-by-turn "
                        "waypoints with depth/heading/instruction) to this CSV; pass "
                        "the flag alone for <out>_guidance.csv")
    p.add_argument("--guidance-png", nargs="?", const="__AUTO__", default=None,
                   help="render the blade-scale guidance lanes over the cut/fill map; "
                        "pass the flag alone for <out>_guidance.png")
    p.add_argument("--lane-overlap", type=float, default=0.0,
                   help="0-0.9: fractional overlap between adjacent blade lanes "
                        "(0 = lanes butt edge-to-edge)")
    p.add_argument("--turn-speed-mph", type=float, default=DEFAULT_TURN_SPEED_MPH,
                   help="speed through a turn (the machine slows to turn); used by "
                        "the turn-aware guidance time")
    p.add_argument("--grade-speed-mph", type=float, default=DEFAULT_GRADE_SPEED_MPH,
                   help="speed of the return grade/compact pass driven back over "
                        "each fill (default %(default)g MPH)")
    p.add_argument("--turning-circle-ft", type=float, default=DEFAULT_TURNING_CIRCLE_FT,
                   help="tractor+scraper turning circle (diameter, ft); sets the "
                        "turn arc length in the guidance time")
    p.add_argument("--turn-threshold-deg", type=float, default=DEFAULT_TURN_THRESHOLD_DEG,
                   help="heading changes below this (deg) are gentle bends driven "
                        "in stride (no slowdown); only sharper turns cost turn time "
                        "(default %(default)g)")
    p.add_argument("--no-grade-return", action="store_true",
                   help="do not add a grade/compact return pass over each fill")
    p.add_argument("--no-direction-aware", action="store_true",
                   help="disable direction-aware sequencing (penalising the turn "
                        "needed to leave a pass and enter the next cut); falls back "
                        "to pure deadhead-distance ordering + 2-opt")
    p.add_argument("--animate-dir", type=Path, default=None,
                   help="write a cut/fill PNG frame per stride of applied passes "
                        "into this directory (leveling simulation)")
    p.add_argument("--animate-gif", nargs="?", const="__AUTO__", default=None,
                   help="assemble the leveling frames into an animated GIF; "
                        "pass the flag alone for <out>_leveling.gif")
    p.add_argument("--animate-stride", type=int, default=10,
                   help="emit one animation frame per N applied passes (default 10)")
    p.add_argument("--animate-ppb", type=int, default=8,
                   help="pixels per area edge for animation frames (default 8)")
    p.add_argument("--animate-mode", choices=["cutfill", "topology"], default="cutfill",
                   help="animation render: cutfill map (default) or topology "
                        "(elevation terrain that flattens to grade)")
    p.add_argument("--no-plan", action="store_true",
                   help="render the PNG from stages 1-3 only (skip passes/time)")
    p.add_argument("--seed", type=int, default=1234)
    p.add_argument("--version", action="version", version=f"landlevel {__version__}")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.pixels_per_bin <= 0:
        sys.stderr.write("ERROR: --pixels-per-bin must be > 0\n")
        return 2
    if not args.agd.is_file():
        sys.stderr.write(f"ERROR: AGD file not found: {args.agd}\n")
        return 2

    t0 = time.time()
    sys.stderr.write(f"INFO: parsing {args.agd}\n")
    field = parse_agd_file(args.agd)
    sys.stderr.write(f"INFO: {len(field.points)} interior points, {len(field.boundary)} boundary points\n")

    grid = build_grid(
        field,
        grid_size_ft=args.grid_size_ft,
        swell_factor=args.swell_factor,
        compaction=args.cutfill_ratio,
        max_depth_mm=args.max_depth_mm,
    )
    n_valid = int(grid.valid.sum())
    sys.stderr.write(f"INFO: grid {grid.width} x {grid.height} ({n_valid} valid areas)\n")

    bal = balance(grid, mode=args.balance_mode)
    sys.stderr.write(
        f"INFO: supply={bal.total_supply_bcy:.1f} BCY, demand={bal.total_demand_bcy:.1f} BCY, "
        f"imbalance={bal.relative_imbalance * 100:.2f}%\n"
    )

    flow = compute_flow_field(grid, mode=args.transport_mode)

    render_png(grid, flow, args.out, pixels_per_bin=args.pixels_per_bin, arrow_stride=args.arrow_stride)
    sys.stderr.write(f"INFO: wrote {args.out}\n")

    if args.no_plan:
        print(f"Cut/fill PNG written: {args.out}")
        print(f"Supply: {bal.total_supply_bcy:.1f} BCY  Demand: {bal.total_demand_bcy:.1f} BCY")
        return 0

    sys.stderr.write("INFO: tracing strips\n")
    strips = trace_strips(grid, flow, max_seeds=400)
    sys.stderr.write(f"INFO: {len(strips)} strips\n")

    sys.stderr.write("INFO: generating passes\n")
    scraper_bcy = args.scraper_capacity_lcy / args.swell_factor
    sys.stderr.write(
        f"INFO: scraper capacity {args.scraper_capacity_lcy:g} LCY = {scraper_bcy:.2f} BCY/pass\n"
    )
    heading_deg = args.heading_deg if args.heading_deg is not None else estimate_slope_heading(grid)
    sys.stderr.write(
        f"INFO: haul heading {heading_deg:.1f} deg "
        f"({'auto slope' if args.heading_deg is None else 'user'}), "
        f"strength {args.heading_strength:g}\n"
    )
    pass_result = generate_passes(
        grid, strips,
        tolerance_mm=args.tolerance_mm,
        max_load_bcy=scraper_bcy,
        heading_deg=heading_deg,
        heading_strength=args.heading_strength,
        strip_bias=args.strip_bias,
        max_drop_hop_m=args.max_drop_hop_ft * FEET_TO_METERS,
    )
    sys.stderr.write(
        f"INFO: {len(pass_result.passes)} passes in {pass_result.rounds} rounds "
        f"(unmatched cut {pass_result.unmatched_cut_bcy:.1f} BCY, "
        f"fill {pass_result.unmatched_fill_bcy:.1f} BCY)\n"
    )

    route = sequence_passes(
        pass_result.passes,
        grade_return=not args.no_grade_return,
        direction_aware=not args.no_direction_aware,
        return_speed_mph=args.return_speed_mph,
        turn_speed_mph=args.turn_speed_mph,
        turning_circle_ft=args.turning_circle_ft,
    )

    zone_steps = []
    if args.zones and args.zones > 0:
        zone_steps = sequence_cut_zones(grid, field, route, n_zones=args.zones, seed=args.seed)

    params = TimeParams(
        loading_speed_mph=args.loading_speed_mph,
        haul_speed_mph=args.haul_speed_mph,
        dumping_speed_mph=args.dumping_speed_mph,
        return_speed_mph=args.return_speed_mph,
        accel_time_s=args.accel_time_s,
        decel_time_s=args.decel_time_s,
        field_efficiency=args.field_efficiency,
    )
    tres = estimate_time(grid, route, params)

    elapsed = time.time() - t0

    moved_bcy = sum(p.load_bcy for p in pass_result.passes)
    area_ac = n_valid * grid.area_m2 / 4046.8564224
    total_h = tres.total_time_s / 3600.0
    move_rate = moved_bcy / total_h if total_h > 0 else 0.0
    avg_move_yd = (tres.hauling_dist_m / len(pass_result.passes) / 0.9144) if pass_result.passes else 0.0
    M = 1609.344

    def mi(m: float) -> str:
        return f"{m / M:,.1f}"

    def hh(s: float) -> str:
        return f"{s / 3600.0:,.1f}"

    print("=" * 56)
    print("OptiHaul Analysis Results")
    print(f"  Grid spacing (ft):      {args.grid_size_ft:g} "
          f"({grid.width} x {grid.height}, {n_valid} areas)")
    print(f"  Field Area (ac):        {area_ac:,.1f}")
    print(f"  Move Vol. (BCY):        {moved_bcy:,.0f}")
    print(f"  Move Rate (BCY/hr):     {move_rate:,.0f}")
    print(f"  Area Rate (ac/hr):      {area_ac / total_h if total_h else 0:,.3f}")
    print(f"  Ave. Move Dist. (yd):   {avg_move_yd:,.0f}")
    print(f"  Haul heading (deg):     {heading_deg:,.1f}")
    print(f"  Passes:                 {len(pass_result.passes)}")
    print("-" * 56)
    print("Distances                       Speeds")
    print(f"  Loading (mi):  {mi(tres.loading_dist_m):>7}        Loading (mph):     "
          f"{args.loading_speed_mph:>5.1f}")
    print(f"  Hauling (mi):  {mi(tres.hauling_dist_m):>7}        Ave. Hauling (mph):"
          f"{tres.avg_haul_speed_mph:>6.1f}")
    print(f"  Dumping (mi):  {mi(tres.dumping_dist_m):>7}        Dumping (mph):     "
          f"{args.dumping_speed_mph:>5.1f}")
    print(f"  Returning(mi): {mi(tres.returning_dist_m):>7}        Ave. Return (mph): "
          f"{tres.avg_return_speed_mph:>5.1f}")
    print("-" * 56)
    print("Times (hr)")
    print(f"  Loading:    {hh(tres.loading_time_s):>6}")
    print(f"  Hauling:    {hh(tres.hauling_time_s):>6}")
    print(f"  Dumping:    {hh(tres.dumping_time_s):>6}")
    print(f"  Returning:  {hh(tres.returning_time_s):>6}")
    print(f"  Other:      {hh(tres.other_time_s):>6}   (field eff. {args.field_efficiency:.0%})")
    print(f"  TOTAL:      {hh(tres.total_time_s):>6}")
    print("=" * 56)

    if zone_steps:
        print(f"Rough cut work order ({len(zone_steps)} zones)")
        print("  # pos    loads     BCY   lat        lon         worked@pass")
        for s in zone_steps:
            print(f"  {s.work_order:<2}{s.compass:<4} {s.n_loads:>5}  {s.total_bcy:>7,.0f}  "
                  f"{s.lat:>9.5f}  {s.lon:>10.5f}   {s.first_pass}-{s.last_pass}")
        seq_csv = args.sequence_csv or args.out.with_name(args.out.stem + "_cut_sequence.csv")
        write_sequence_csv(zone_steps, seq_csv)
        overlay_zone_waypoints(args.out, grid, zone_steps, pixels_per_bin=args.pixels_per_bin)
        print(f"Sequence CSV: {seq_csv}")
        print("=" * 56)

    if args.pairs_csv is not None:
        if args.pairs_csv == "__AUTO__":
            pairs_csv = args.out.with_name(args.out.stem + "_cut_fill_pairs.csv")
        else:
            pairs_csv = Path(args.pairs_csv)
        write_pass_pairs_csv(field, route, pairs_csv)
        print(f"Cut/fill pairs CSV: {pairs_csv} ({len(route.ordered_passes)} passes)")
        print("=" * 56)

    if args.pairs_png is not None:
        if args.pairs_png == "__AUTO__":
            pairs_png = args.out.with_name(args.out.stem + "_cut_fill_pairs.png")
        else:
            pairs_png = Path(args.pairs_png)
        render_pass_pairs_png(grid, route.ordered_passes, pairs_png,
                              pixels_per_bin=args.pixels_per_bin)
        print(f"Cut/fill pairs PNG: {pairs_png} ({len(route.ordered_passes)} lines)")
        print("=" * 56)

    if args.guidance_csv is not None or args.guidance_png is not None:
        sys.stderr.write("INFO: building blade-scale guidance track\n")
        guidance = build_guidance(
            grid, field, route,
            blade_width_mm=args.blade_width_mm,
            heading_deg=heading_deg,
            lane_overlap=args.lane_overlap,
            grade_return=not args.no_grade_return,
            turn_threshold_deg=args.turn_threshold_deg,
        )
        gtime = estimate_guidance_time(
            guidance,
            loading_speed_mph=args.loading_speed_mph,
            haul_speed_mph=args.haul_speed_mph,
            dumping_speed_mph=args.dumping_speed_mph,
            return_speed_mph=args.return_speed_mph,
            grade_speed_mph=args.grade_speed_mph,
            turn_speed_mph=args.turn_speed_mph,
            turning_circle_ft=args.turning_circle_ft,
            accel_time_s=args.accel_time_s,
            decel_time_s=args.decel_time_s,
            field_efficiency=args.field_efficiency,
        )
        cov = guidance.coverage_factor()
        print(f"Blade-scale guidance ({len(guidance.points)} waypoints, "
              f"blade {args.blade_width_mm:g} mm on {args.grid_size_ft:g} ft grid)")
        print(f"  Haul (optimized, preserved):  {guidance.haul_dist_m / M:,.1f} mi "
              f"(route {guidance.route_haul_dist_m / M:,.1f} mi)")
        print(f"  Loading sweep:  planned {guidance.planned_cut_dist_m / M:,.1f} mi "
              f"-> blade-scale {guidance.cut_dist_m / M:,.1f} mi")
        print(f"  Dumping sweep:  planned {guidance.planned_fill_dist_m / M:,.1f} mi "
              f"-> blade-scale {guidance.fill_dist_m / M:,.1f} mi")
        print(f"  Grade/compact return:  {guidance.grade_dist_m / M:,.1f} mi")
        print(f"  Multi-drop repositioning:  {guidance.reposition_dist_m / M:,.1f} mi "
              f"(blade-up hops <= {args.max_drop_hop_ft:g} ft between pockets)")
        print(f"  Coverage factor: {cov:,.2f}x  (blade width / cell width = "
              f"{guidance.blade_width_m / guidance.grid_size_m:,.2f})")
        print(f"  Turns: {guidance.n_turns:,} tight (>= {args.turn_threshold_deg:g} deg)  "
              f"(turn circle {args.turning_circle_ft:g} ft @ {args.turn_speed_mph:g} mph; "
              f"gentler bends driven in stride)")
        seq_mode = ("off (deadhead distance + 2-opt)" if args.no_direction_aware
                    else "on (deadhead time + turn-into-cut penalty)")
        print(f"  Sequencing: direction-aware {seq_mode}; "
              f"deadhead {route.total_deadhead_m / M:,.1f} mi")
        print("  Drivable time (hr): "
              f"load {gtime.loading_s / 3600:,.1f}  haul {gtime.hauling_s / 3600:,.1f}  "
              f"dump {gtime.dumping_s / 3600:,.1f}  grade {gtime.grading_s / 3600:,.1f}  "
              f"return {gtime.returning_s / 3600:,.1f}  turn {gtime.turning_s / 3600:,.1f}")
        print(f"  Drivable TOTAL: {gtime.total_s / 3600:,.1f} hr "
              f"(incl. turns + grade/compact; field eff. {args.field_efficiency:.0%})")
        if args.guidance_csv is not None:
            gpath = (args.out.with_name(args.out.stem + "_guidance.csv")
                     if args.guidance_csv == "__AUTO__" else Path(args.guidance_csv))
            write_guidance_csv(guidance, gpath)
            print(f"Guidance CSV: {gpath}")
        if args.guidance_png is not None:
            gpng = (args.out.with_name(args.out.stem + "_guidance.png")
                    if args.guidance_png == "__AUTO__" else Path(args.guidance_png))
            render_guidance_png(grid, guidance.points, gpng,
                                pixels_per_bin=args.pixels_per_bin,
                                blade_width_m=guidance.blade_width_m)
            print(f"Guidance PNG: {gpng}")
        print("=" * 56)

    if args.animate_dir is not None or args.animate_gif is not None:
        gif_path = None
        if args.animate_gif is not None:
            gif_path = (args.out.with_name(args.out.stem + "_leveling.gif")
                        if args.animate_gif == "__AUTO__" else Path(args.animate_gif))
        sys.stderr.write("INFO: rendering leveling animation frames\n")
        n_frames = animate_leveling(
            grid, route.ordered_passes,
            out_dir=args.animate_dir,
            stride=max(1, args.animate_stride),
            pixels_per_bin=max(1, args.animate_ppb),
            gif_path=gif_path,
            mode=args.animate_mode,
        )
        if args.animate_dir is not None:
            print(f"Animation frames: {n_frames} PNGs in {args.animate_dir}")
        if gif_path is not None:
            print(f"Animation GIF: {gif_path}")
        print("=" * 56)

    print(f"PNG: {args.out}")
    sys.stderr.write(f"INFO: done in {elapsed:.1f}s\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
