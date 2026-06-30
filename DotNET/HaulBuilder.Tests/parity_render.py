"""Emit PNG outputs for cross-language render parity checks.

Called from C# xUnit tests via subprocess. Writes PNG files into ``out_dir`` and
prints a short manifest on stdout (one ``name=path`` per line).
"""
from __future__ import annotations

import sys
from pathlib import Path

from landlevel import DEFAULT_BLADE_WIDTH_MM, DEFAULT_SCRAPER_CAPACITY_LCY, DEFAULT_SWELL_FACTOR
from landlevel.agd import parse_agd_file
from landlevel.balance import balance
from landlevel.flowfield import compute_flow_field
from landlevel.grid import build_grid
from landlevel.obstacles import EMPTY_OBSTACLES
from landlevel.passes import generate_passes
from landlevel.render import (
    render_guidance_png,
    render_pass_pairs_png,
    render_png,
    render_single_pair_png,
)
from landlevel.routing import sequence_passes
from landlevel.strips import trace_strips
from landlevel.swaths import build_guidance, reprofile_knots


def _first_pair_pass(points) -> int | None:
    by_pass: dict[int, list] = {}
    for pt in points:
        by_pass.setdefault(pt.pass_no, []).append(pt)
    for pno in sorted(by_pass):
        actions = {p.action for p in by_pass[pno]}
        if "CUT" in actions and "FILL" in actions:
            return pno
    return None


def main() -> int:
    if len(sys.argv) < 4:
        sys.stderr.write("usage: parity_render.py AGD OUT_DIR PPB [ARROW_STRIDE]\n")
        return 2

    agd_path = Path(sys.argv[1])
    out_dir = Path(sys.argv[2])
    ppb = int(sys.argv[3])
    arrow_stride = int(sys.argv[4]) if len(sys.argv) > 4 else 2
    out_dir.mkdir(parents=True, exist_ok=True)

    field = parse_agd_file(agd_path)
    grid = build_grid(field, grid_size_ft=15.0)
    balance(grid, mode="check")
    flow = compute_flow_field(grid, mode="poisson")
    strips = trace_strips(grid, flow)
    scraper_bcy = DEFAULT_SCRAPER_CAPACITY_LCY / DEFAULT_SWELL_FACTOR
    pr = generate_passes(
        grid,
        strips,
        tolerance_mm=10.0,
        max_load_bcy=scraper_bcy,
        heading_deg=90.0,
        heading_strength=0.5,
    )
    route = sequence_passes(pr.passes, obstacles=EMPTY_OBSTACLES)
    reprofile_knots(route, grid, blade_width_mm=DEFAULT_BLADE_WIDTH_MM, heading_deg=90.0, lane_overlap=0.0)
    guidance = build_guidance(
        grid,
        field,
        route,
        blade_width_mm=DEFAULT_BLADE_WIDTH_MM,
        heading_deg=90.0,
        lane_overlap=0.0,
        grade_return=True,
        turn_threshold_deg=60.0,
        smooth_route=True,
        obstacles=EMPTY_OBSTACLES,
    )

    main_png = out_dir / "main.png"
    pairs_png = out_dir / "pass_pairs.png"
    guidance_png = out_dir / "guidance.png"
    render_png(grid, flow, main_png, pixels_per_bin=ppb, arrow_stride=arrow_stride)
    render_pass_pairs_png(grid, route.ordered_passes, pairs_png, pixels_per_bin=ppb)
    render_guidance_png(
        grid,
        guidance.points,
        guidance_png,
        pixels_per_bin=ppb,
        blade_width_m=DEFAULT_BLADE_WIDTH_MM / 1000.0,
    )

    pair_png = out_dir / "single_pair.png"
    pair_pass = _first_pair_pass(guidance.points)
    if pair_pass is not None:
        pts = [p for p in guidance.points if p.pass_no == pair_pass]
        render_single_pair_png(
            grid,
            pts,
            pair_png,
            blade_width_m=DEFAULT_BLADE_WIDTH_MM / 1000.0,
            pixels_per_bin=ppb,
            show_route=False,
            curve_joint=False,
        )
        print(f"single_pair_pass={pair_pass}")
        print("single_pair_sig=" + ",".join(p.action for p in pts))
        cut_pts = [p for p in pts if p.action in ("START", "RETURN", "CUT")]
        fill_pts = [p for p in pts if p.action in ("HAUL", "FILL")]
        if cut_pts and fill_pts:
            from landlevel.render import _world_to_pixel
            a = _world_to_pixel(grid, cut_pts[-1].x, cut_pts[-1].y, ppb)
            b = _world_to_pixel(grid, fill_pts[0].x, fill_pts[0].y, ppb)
            print(f"haul_seg={round(a[0])},{round(a[1])},{round(b[0])},{round(b[1])}")
    else:
        print("single_pair_pass=")

    print(f"main={main_png}")
    print(f"pass_pairs={pairs_png}")
    print(f"guidance={guidance_png}")
    print(f"single_pair={pair_png}")
    print(f"passes={len(pr.passes)}")
    print("guidance_sig=" + ",".join(p.action for p in guidance.points))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
