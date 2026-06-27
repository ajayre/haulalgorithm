"""Stage output - PNG rendering (spec §14).

Renders the cut/fill map using the nine-band palette from ``fielddesign.py``
and overlays haul-direction arrows as hollow triangles (the glyph from
``render_existing_heights.py``). Image geometry matches ``fieldcreator-checker.py``
and ``render_existing_heights.py``: image Y is flipped relative to grid Y so
larger ``by`` is drawn higher.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

try:
    from PIL import Image, ImageDraw
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Pillow is required. Install with: pip install Pillow") from exc

from .flowfield import FlowField
from .grid import Grid

# Nine-band cut/fill palette (spec §14.1.2). Thresholds on diff = existing - target.
_VIOLET = (0x80, 0x00, 0x80)
_INDIGO = (0x9A, 0x31, 0xFF)
_BLUE = (0x00, 0x66, 0xFF)
_CYAN = (0x36, 0xF5, 0xF5)
_GREEN = (0x00, 0xCD, 0x00)
_YELLOW = (0xFF, 0xFF, 0x00)
_ORANGE = (0xFF, 0x80, 0x00)
_RED = (0xFF, 0x00, 0x00)
_DARKRED = (0xB4, 0x00, 0x00)

ARROW_COLOR = (0, 0, 0)


def cutfill_color(diff: float) -> Tuple[int, int, int]:
    """Map diff = existing - target (metres) to the nine-band palette (§14.1)."""
    if diff < -0.80:
        return _VIOLET
    if diff < -0.50:
        return _INDIGO
    if diff < -0.05:
        return _BLUE
    if diff < -0.01:
        return _CYAN
    if diff <= 0.01:
        return _GREEN
    if diff <= 0.05:
        return _YELLOW
    if diff <= 0.50:
        return _ORANGE
    if diff <= 0.80:
        return _RED
    return _DARKRED


# Palette as an array indexed by band (matches cutfill_color ordering).
_PALETTE = np.array(
    [_VIOLET, _INDIGO, _BLUE, _CYAN, _GREEN, _YELLOW, _ORANGE, _RED, _DARKRED],
    dtype=np.uint8,
)
_BAND_EDGES = np.array([-0.80, -0.50, -0.05, -0.01, 0.01, 0.05, 0.50, 0.80])


def diff_to_image(diff: np.ndarray, valid: np.ndarray, ppb: int) -> "Image.Image":
    """Vectorised cut/fill render of a diff grid (existing - target) to an RGBA image.

    Far faster than the per-pixel ``render_png`` path, suitable for the many
    frames of an animation. Image Y is flipped so larger ``by`` is drawn higher.
    """
    d = np.where(valid, diff, np.nan)
    idx = np.digitize(np.nan_to_num(d, nan=1e9), _BAND_EDGES)
    rgb = _PALETTE[idx]
    alpha = np.where(valid & np.isfinite(d), 255, 0).astype(np.uint8)
    rgba = np.dstack([rgb, alpha])
    rgba = np.flipud(rgba)  # larger by -> higher in image
    img = Image.fromarray(rgba, "RGBA")
    if ppb > 1:
        img = img.resize((diff.shape[1] * ppb, diff.shape[0] * ppb), Image.NEAREST)
    return img


# Terrain ramp for topology (elevation) rendering: low -> high.
_TERRAIN_STOPS = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
_TERRAIN_COLORS = np.array(
    [[30, 90, 40], [120, 170, 70], [210, 200, 120], [150, 110, 70], [245, 245, 245]],
    dtype=float,
)


def elevation_to_image(
    elev: np.ndarray, valid: np.ndarray, ppb: int, vmin: float, vmax: float
) -> "Image.Image":
    """Vectorised terrain render of an elevation grid (metres) to an RGBA image.

    ``vmin``/``vmax`` fix the colour scale across frames so the surface visibly
    flattens toward grade. Image Y is flipped so larger ``by`` is drawn higher.
    """
    rng = (vmax - vmin) or 1.0
    e = np.nan_to_num(elev, nan=vmin)
    t = np.clip((e - vmin) / rng, 0.0, 1.0)
    r = np.interp(t, _TERRAIN_STOPS, _TERRAIN_COLORS[:, 0])
    g = np.interp(t, _TERRAIN_STOPS, _TERRAIN_COLORS[:, 1])
    b = np.interp(t, _TERRAIN_STOPS, _TERRAIN_COLORS[:, 2])
    rgb = np.stack([r, g, b], axis=-1).astype(np.uint8)
    alpha = np.where(valid, 255, 0).astype(np.uint8)
    rgba = np.flipud(np.dstack([rgb, alpha]))
    img = Image.fromarray(rgba, "RGBA")
    if ppb > 1:
        img = img.resize((elev.shape[1] * ppb, elev.shape[0] * ppb), Image.NEAREST)
    return img


def _direction_to_image_vector(deg: float) -> Tuple[float, float]:
    """Compass degrees (0=N, 90=E) -> image vector (dx, dy) = (sin, -cos)."""
    rad = math.radians(deg)
    return math.sin(rad), -math.cos(rad)


def _draw_arrow_hollow_triangle(
    draw: "ImageDraw.ImageDraw",
    px: float,
    py: float,
    direction_deg: float,
    color: Tuple[int, int, int] = ARROW_COLOR,
    length_px: float = 12.0,
    base_half_width_px: float = 4.0,
) -> None:
    """Hollow (outline-only) triangle pointing in the flow direction (§14.2.1)."""
    dx, dy = _direction_to_image_vector(direction_deg)
    end_x = px + length_px * dx
    end_y = py + length_px * dy
    base_x = px - 0.35 * length_px * dx
    base_y = py - 0.35 * length_px * dy
    perp_x, perp_y = -dy, dx
    tip = (round(end_x), round(end_y))
    left = (round(base_x + base_half_width_px * perp_x), round(base_y + base_half_width_px * perp_y))
    right = (round(base_x - base_half_width_px * perp_x), round(base_y - base_half_width_px * perp_y))
    draw.polygon([tip, left, right], outline=color, fill=None)


def render_png(
    grid: Grid,
    flow: Optional[FlowField],
    out_path: Path,
    pixels_per_bin: int = 50,
    arrow_stride: int = 2,
) -> None:
    """Render the cut/fill map + haul arrows to a PNG (spec §14.3)."""
    ppb = pixels_per_bin
    w = grid.width * ppb
    h = grid.height * ppb
    image = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    pixels = image.load()

    delta = grid.delta  # existing - target, NaN outside mask

    for by in range(grid.height):
        for bx in range(grid.width):
            if not grid.valid[by, bx]:
                continue
            d = delta[by, bx]
            if not np.isfinite(d):
                continue
            r, g, b = cutfill_color(float(d))
            x0 = bx * ppb
            y0 = (grid.height - 1 - by) * ppb
            for dy in range(ppb):
                yy = y0 + dy
                for dx in range(ppb):
                    pixels[x0 + dx, yy] = (r, g, b, 255)

    if flow is not None and arrow_stride > 0:
        draw = ImageDraw.Draw(image)
        length_px = ppb * 0.45
        base_half = ppb * 0.16
        for by in range(0, grid.height, arrow_stride):
            for bx in range(0, grid.width, arrow_stride):
                if not grid.valid[by, bx] or not flow.has_arrow[by, bx]:
                    continue
                px = (bx + 0.5) * ppb
                py = (grid.height - 0.5 - by) * ppb
                direction_deg = flow.direction_deg(bx, by)
                _draw_arrow_hollow_triangle(
                    draw, px, py, direction_deg,
                    length_px=length_px, base_half_width_px=base_half,
                )

    out_path = Path(out_path)
    image.save(out_path)


def _world_to_pixel(grid: Grid, x: float, y: float, ppb: int) -> Tuple[float, float]:
    """Map a world (easting, northing) point to image pixels (Y flipped)."""
    px = (x - grid.min_x) / grid.grid_size_m * ppb
    py = (grid.height - (y - grid.min_y) / grid.grid_size_m) * ppb
    return px, py


# Hot ramp for haul-density rendering: even a single load (low end) is a clearly
# visible yellow, building through orange to deep red where many loads overlap.
_HEAT_STOPS = np.array([0.0, 0.5, 1.0])
_HEAT_COLORS = np.array([[255, 225, 0], [255, 110, 0], [200, 0, 0]], dtype=float)


def render_pass_pairs_png(
    grid: Grid,
    passes,
    out_path: Path,
    pixels_per_bin: int = 50,
    base_dim: float = 0.28,
) -> None:
    """Cut/fill map overlaid with one semi-transparent line per cut-fill pair.

    Each pass contributes one line along its haul leg (cut end -> fill start).
    Lines accumulate, so dense haul corridors build up into a hot "heat map" look
    (yellow where a single load passes, deep red where many do). ``base_dim``
    (0-1) fades the underlying cut/fill map so the lines stand out.
    """
    # The plan PNG uses ~50 px/cell; 1-px lines vanish on a canvas that large, so
    # render the pairs view at a modest resolution and draw lines several px wide.
    ppb = max(1, min(pixels_per_bin, 16))
    base = diff_to_image(grid.delta, grid.valid, ppb).convert("RGBA")
    if 0.0 <= base_dim < 1.0:
        arr = np.asarray(base).astype(float)
        white = np.array([255, 255, 255], dtype=float)
        arr[..., :3] = arr[..., :3] * base_dim + white * (1.0 - base_dim)
        base = Image.fromarray(arr.astype(np.uint8), "RGBA")
    w, h = base.size

    # Disk stamp so each line is a few pixels wide regardless of resolution.
    r = max(1, round(ppb / 6))
    offs = [(dx, dy) for dy in range(-r, r + 1) for dx in range(-r, r + 1)
            if dx * dx + dy * dy <= r * r]
    off_dx = np.array([o[0] for o in offs], dtype=np.int64)
    off_dy = np.array([o[1] for o in offs], dtype=np.int64)

    acc = np.zeros((h, w), dtype=np.float32)
    for p in passes:
        x0, y0 = _world_to_pixel(grid, p.cut_end_xy[0], p.cut_end_xy[1], ppb)
        x1, y1 = _world_to_pixel(grid, p.fill_start_xy[0], p.fill_start_xy[1], ppb)
        n = max(2, int(math.hypot(x1 - x0, y1 - y0)) + 1)
        xs = np.round(np.linspace(x0, x1, n)).astype(np.int64)
        ys = np.round(np.linspace(y0, y1, n)).astype(np.int64)
        # Thicken via the disk stamp, then count each pixel once per line.
        px = np.clip((xs[:, None] + off_dx[None, :]).ravel(), 0, w - 1)
        py = np.clip((ys[:, None] + off_dy[None, :]).ravel(), 0, h - 1)
        lin = np.unique(py * w + px)
        np.add.at(acc, (lin // w, lin % w), 1.0)

    overlay = np.zeros((h, w, 4), dtype=np.uint8)
    pos = acc > 0
    if pos.any():
        # Robust scale (90th pct) so a few hotspots don't wash everything out.
        scale = max(float(np.percentile(acc[pos], 90)), 1.0)
        t = np.sqrt(np.clip(acc / scale, 0.0, 1.0))  # sqrt spreads the low end
        overlay[..., 0] = np.interp(t, _HEAT_STOPS, _HEAT_COLORS[:, 0]).astype(np.uint8)
        overlay[..., 1] = np.interp(t, _HEAT_STOPS, _HEAT_COLORS[:, 1]).astype(np.uint8)
        overlay[..., 2] = np.interp(t, _HEAT_STOPS, _HEAT_COLORS[:, 2]).astype(np.uint8)
        # Solid alpha floor so even isolated single lines are clearly visible.
        alpha = np.where(pos, np.clip(150.0 + t * 105.0, 0.0, 255.0), 0.0)
        overlay[..., 3] = alpha.astype(np.uint8)

    out = Image.alpha_composite(base, Image.fromarray(overlay, "RGBA"))
    out.save(Path(out_path))


def overlay_zone_waypoints(
    in_out_path: Path,
    grid: Grid,
    steps,
    pixels_per_bin: int = 50,
) -> None:
    """Draw numbered work-order waypoints (zone centroids) onto an existing PNG.

    ``steps`` is a list of :class:`landlevel.sequence.ZoneStep`. Markers are
    numbered by ``work_order`` and connected by a thin line in that order.
    """
    if not steps:
        return
    path = Path(in_out_path)
    image = Image.open(path).convert("RGBA")
    draw = ImageDraw.Draw(image)
    ppb = pixels_per_bin
    # Size markers relative to the image so numbers are legible at any resolution.
    short_side = min(image.size)
    radius = max(ppb // 2, short_side // 38)
    line_w = max(2, radius // 4)
    font_size = max(14, int(radius * 1.3))

    # Zone footprint outlines (unfilled, exact cell boundary): a white halo under
    # a black line so they read over any palette colour. Drawn first, beneath
    # markers/path. Each zone is a set of unit cell-edge segments.
    edge_w = max(2, line_w - 1)
    for s in steps:
        outline = getattr(s, "outline", None)
        if not outline:
            continue
        seg_px = [
            ((round(ax), round(ay)), (round(bx), round(by)))
            for (ax, ay), (bx, by) in (
                (_world_to_pixel(grid, x1, y1, ppb), _world_to_pixel(grid, x2, y2, ppb))
                for (x1, y1), (x2, y2) in outline
            )
        ]
        for a, b in seg_px:
            draw.line([a, b], fill=(255, 255, 255, 230), width=edge_w + 2)
        for a, b in seg_px:
            draw.line([a, b], fill=(0, 0, 0, 255), width=edge_w)

    pts = [_world_to_pixel(grid, s.easting, s.northing, ppb) for s in steps]
    if len(pts) > 1:
        draw.line([(round(x), round(y)) for x, y in pts], fill=(0, 0, 0, 220), width=line_w)

    try:
        from PIL import ImageFont
        font = ImageFont.load_default(size=font_size)
    except Exception:  # pragma: no cover - older Pillow without size arg
        from PIL import ImageFont
        font = ImageFont.load_default()

    for s, (px, py) in zip(steps, pts):
        bbox = (px - radius, py - radius, px + radius, py + radius)
        draw.ellipse(bbox, fill=(255, 255, 255, 235), outline=(0, 0, 0, 255), width=line_w)
        label = str(s.work_order)
        try:
            tb = draw.textbbox((0, 0), label, font=font)
            tw, th = tb[2] - tb[0], tb[3] - tb[1]
            draw.text((px - tw / 2 - tb[0], py - th / 2 - tb[1]), label, fill=(0, 0, 0, 255), font=font)
        except Exception:  # pragma: no cover
            draw.text((px - 4, py - 6), label, fill=(0, 0, 0, 255))

    image.save(path)
