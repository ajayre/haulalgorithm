"""Detailed top-down leveling animation for methodology verification.

Renders the field as each routed pass is applied (in optimized route order),
side by side:

  * LEFT  - the cut/fill map (existing - target): warm = still-high (cut),
            cool = still-low (fill), green = within tolerance (on grade).
  * RIGHT - the topology (absolute elevation) on a fixed terrain ramp, so the
            ridges visibly cut down and the hollows fill up toward a flat plane.

A header HUD reports the route position, passes applied, percent of the field
on grade, and the cumulative bank volume moved, so the methodology can be
checked frame by frame: soil leaves the cut (warm) cells, lands in the fill
(cool) cells, and both converge to green / flat as the plan completes.

Frames are written individually (so any pass can be inspected) and assembled
into a GIF. This is a verification artifact; it does not change the plan.

Usage:
    python make_leveling_animation.py            # defaults below
    python make_leveling_animation.py 8.5 270 4  # grid_ft heading_deg ppb
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from landlevel import M3_TO_CY
from landlevel.agd import parse_agd_file
from landlevel.grid import build_grid
from landlevel.balance import balance
from landlevel.flowfield import compute_flow_field
from landlevel.strips import trace_strips
from landlevel.passes import estimate_slope_heading, generate_passes
from landlevel.routing import sequence_passes
from landlevel.render import diff_to_image, elevation_to_image

# ----------------------------- parameters ---------------------------------
AGD = "TheShop2_2ft.agd"
GRID_FT = float(sys.argv[1]) if len(sys.argv) > 1 else 8.5
HEADING = float(sys.argv[2]) if len(sys.argv) > 2 else 270.0
PPB = int(sys.argv[3]) if len(sys.argv) > 3 else 4          # pixels per cell
TARGET_FRAMES = 500            # ~ this many frames across the whole job
EARLY_PER_PASS = 40            # first N passes are shown one-per-frame
GIF_MS = 90                    # ms per frame in the GIF
OUT_GIF = Path("out/TheShop2_leveling.gif")
OUT_FRAMES = Path("out/TheShop2_leveling_frames")
TOL_M = 0.010                  # on-grade tolerance (10 mm)
HEADER_H = 64
GAP = 14
# ---------------------------------------------------------------------------


def _font(size: int):
    for name in ("arialbd.ttf", "arial.ttf", "DejaVuSans-Bold.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def main() -> int:
    print(f"building plan: {AGD}  grid={GRID_FT} ft  heading={HEADING}")
    field = parse_agd_file(AGD)
    grid = build_grid(field, grid_size_ft=GRID_FT)
    balance(grid, mode="check")
    flow = compute_flow_field(grid, mode="poisson")
    strips = trace_strips(grid, flow, max_seeds=400)
    scraper_bcy = 20.0 / 1.3
    heading = HEADING if HEADING is not None else estimate_slope_heading(grid)
    pr = generate_passes(grid, strips, tolerance_mm=TOL_M * 1000.0,
                         max_load_bcy=scraper_bcy, heading_deg=heading)
    route = sequence_passes(pr.passes)
    passes = route.ordered_passes
    n = len(passes)
    print(f"{n} passes; rendering animation")

    valid = grid.valid
    target = grid.target
    existing = grid.existing.astype(float).copy()
    area = grid.area_m2

    # Fixed elevation colour scale (initial surface range) for the topology view.
    ev, tv = existing[valid], target[valid]
    vmin = float(min(np.nanmin(ev), np.nanmin(tv)))
    vmax = float(max(np.nanmax(ev), np.nanmax(tv)))

    n_valid = int(valid.sum())
    total_cut_bcy = float(np.clip(np.nan_to_num(grid.delta), 0, None).sum() * area * M3_TO_CY)

    # Which passes end a frame: every pass for the first EARLY_PER_PASS, then a
    # stride that yields ~TARGET_FRAMES overall (so early detail is pass-by-pass).
    rest = max(1, round((n - EARLY_PER_PASS) / max(1, TARGET_FRAMES - EARLY_PER_PASS)))
    emit_at = set(range(1, min(EARLY_PER_PASS, n) + 1))
    emit_at.update(range(EARLY_PER_PASS, n + 1, rest))
    emit_at.add(n)

    OUT_FRAMES.mkdir(parents=True, exist_ok=True)
    OUT_GIF.parent.mkdir(parents=True, exist_ok=True)

    panel_w, panel_h = grid.width * PPB, grid.height * PPB
    W = panel_w * 2 + GAP * 3
    H = HEADER_H + panel_h + GAP
    f_big = _font(max(13, PPB * 4))
    f_small = _font(max(11, PPB * 3))

    moved_bcy = 0.0
    gif_frames: list[Image.Image] = []
    frame_no = 0

    def label(draw, x, y, text, font, fill=(20, 20, 20)):
        draw.text((x, y), text, font=font, fill=fill)

    def emit(pass_done: int):
        nonlocal frame_no
        diff = existing - target
        cut_img = diff_to_image(diff, valid, PPB).convert("RGBA")
        topo_img = elevation_to_image(existing, valid, PPB, vmin, vmax).convert("RGBA")

        canvas = Image.new("RGBA", (W, H), (255, 255, 255, 255))
        canvas.paste(cut_img, (GAP, HEADER_H), cut_img)
        canvas.paste(topo_img, (GAP * 2 + panel_w, HEADER_H), topo_img)
        d = ImageDraw.Draw(canvas)

        on_grade = float((np.abs(diff[valid]) <= TOL_M).mean()) * 100.0
        pct_pass = 100.0 * pass_done / n
        label(d, GAP, 6, "The Shop 2 - land leveling by pass (route order)", f_big)
        label(d, GAP, 6 + PPB * 5 + 6,
              f"pass {pass_done:,}/{n:,} ({pct_pass:4.1f}%)   "
              f"on grade: {on_grade:5.1f}%   "
              f"moved: {moved_bcy:,.0f} / {total_cut_bcy:,.0f} BCY", f_small)
        # Panel captions.
        label(d, GAP + 4, HEADER_H + 2, "CUT / FILL  (green = on grade)", f_small,
              fill=(255, 255, 255))
        label(d, GAP * 2 + panel_w + 4, HEADER_H + 2, "TOPOLOGY  (flattens to grade)",
              f_small, fill=(255, 255, 255))

        canvas.convert("RGB").save(OUT_FRAMES / f"frame_{frame_no:05d}.png")
        gif_frames.append(canvas.convert("P", palette=Image.ADAPTIVE, colors=128))
        frame_no += 1

    emit(0)  # initial surface
    for i, p in enumerate(passes, start=1):
        for (bx, by), depth in p.cut_cells:
            existing[by, bx] -= depth
            moved_bcy += depth * area * M3_TO_CY
        for (bx, by), depth in p.fill_cells:
            existing[by, bx] += depth
        if i in emit_at:
            emit(i)

    if gif_frames:
        durations = [GIF_MS] * len(gif_frames)
        durations[-1] = 4000
        gif_frames[0].save(OUT_GIF, save_all=True, append_images=gif_frames[1:],
                           duration=durations, loop=0, disposal=2)
    print(f"wrote {frame_no} frames -> {OUT_FRAMES}/  and  {OUT_GIF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
