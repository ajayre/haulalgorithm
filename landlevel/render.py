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


def render_guidance_png(
    grid: Grid,
    points,
    out_path: Path,
    pixels_per_bin: int = 50,
    blade_width_m: float = 2.271,
    base_dim: float = 0.30,
    draw_travel: bool = True,
) -> None:
    """Draw the blade-scale guidance track over a faded cut/fill map.

    ``points`` is the ordered list of :class:`landlevel.swaths.GuidancePoint`.
    Each leg into a point is coloured by that point's action: CUT lanes red,
    FILL lanes blue (drawn at the blade width so coverage is visible), HAUL legs
    faint magenta, RETURN legs faint grey. This is a visual check that the lanes
    tile the field at blade scale; it does not change the plan.
    """
    ppb = max(1, min(pixels_per_bin, 16))
    base = diff_to_image(grid.delta, grid.valid, ppb).convert("RGBA")
    if 0.0 <= base_dim < 1.0:
        arr = np.asarray(base).astype(float)
        white = np.array([255, 255, 255], dtype=float)
        arr[..., :3] = arr[..., :3] * base_dim + white * (1.0 - base_dim)
        base = Image.fromarray(arr.astype(np.uint8), "RGBA")

    draw = ImageDraw.Draw(base)
    blade_px = max(1, round(blade_width_m / grid.grid_size_m * ppb))
    colors = {
        "CUT": (200, 0, 0, 255),
        "FILL": (0, 90, 220, 255),
        "GRADE": (40, 160, 40, 200),
        "HAUL": (200, 0, 200, 90),
        "MOVE": (255, 140, 0, 120),
        "RETURN": (120, 120, 120, 70),
        "START": (120, 120, 120, 70),
    }
    prev = None
    for pt in points:
        cur = _world_to_pixel(grid, pt.x, pt.y, ppb)
        if prev is not None:
            action = pt.action
            if action in ("HAUL", "MOVE", "RETURN", "START") and not draw_travel:
                prev = cur
                continue
            width = blade_px if action in ("CUT", "FILL", "GRADE") else 1
            draw.line(
                [(round(prev[0]), round(prev[1])), (round(cur[0]), round(cur[1]))],
                fill=colors.get(action, (0, 0, 0, 255)), width=width,
            )
        prev = cur

    base.save(Path(out_path))


def render_single_pair_png(
    grid: Grid,
    pts,
    out_path: Path,
    blade_width_m: float = 2.271,
    pixels_per_bin: int = 26,
    pad_cells: int = 4,
) -> None:
    """Render ONE pass's guidance (cut strip -> haul leg -> fill strip), cropped.

    ``pts`` is the list of :class:`landlevel.swaths.GuidancePoint` for a single
    pass. The cut strip is drawn red, the fill strip blue (both at blade width),
    and the haul leg as a black arrow from the cut-segment end to the fill-segment
    start. Start/end of each segment are marked so the cut->haul->fill flow is
    obvious. The image is cropped to the pass plus a few cells of padding.
    """
    ppb = pixels_per_bin
    base = diff_to_image(grid.delta, grid.valid, ppb).convert("RGBA")
    arr = np.asarray(base).astype(float)
    white = np.array([255, 255, 255], dtype=float)
    arr[..., :3] = arr[..., :3] * 0.45 + white * 0.55  # fade the map
    base = Image.fromarray(arr.astype(np.uint8), "RGBA")
    draw = ImageDraw.Draw(base)

    blade_px = max(3, round(blade_width_m / grid.grid_size_m * ppb))
    cut_pts = [p for p in pts if p.action in ("START", "RETURN", "CUT")]
    fill_pts = [p for p in pts if p.action in ("HAUL", "FILL")]

    def px(p):
        return _world_to_pixel(grid, p.x, p.y, ppb)

    if len(cut_pts) >= 2:
        draw.line([(round(x), round(y)) for x, y in (px(p) for p in cut_pts)],
                  fill=(200, 0, 0, 255), width=blade_px, joint="curve")
    if len(fill_pts) >= 2:
        draw.line([(round(x), round(y)) for x, y in (px(p) for p in fill_pts)],
                  fill=(0, 90, 220, 255), width=blade_px, joint="curve")

    # Haul leg: cut-segment end -> fill-segment start (the true transit).
    if cut_pts and fill_pts:
        a = px(cut_pts[-1])
        b = px(fill_pts[0])
        draw.line([(round(a[0]), round(a[1])), (round(b[0]), round(b[1]))],
                  fill=(0, 0, 0, 255), width=max(2, blade_px // 2))
        # Arrowhead at the fill end of the haul.
        ang = math.atan2(b[1] - a[1], b[0] - a[0])
        ah = max(8, blade_px * 1.6)
        for da in (math.radians(150), math.radians(-150)):
            draw.line([(round(b[0]), round(b[1])),
                       (round(b[0] + ah * math.cos(ang + da)), round(b[1] + ah * math.sin(ang + da)))],
                      fill=(0, 0, 0, 255), width=max(2, blade_px // 2))

    def dot(p, color, r):
        x, y = px(p)
        draw.ellipse((x - r, y - r, x + r, y + r), fill=color, outline=(0, 0, 0, 255), width=2)

    rr = max(5, blade_px)
    if cut_pts:
        dot(cut_pts[0], (0, 200, 0, 255), rr)      # cut start (green)
        dot(cut_pts[-1], (255, 140, 0, 255), rr)   # cut end / haul start (orange)
    if fill_pts:
        dot(fill_pts[0], (0, 220, 255, 255), rr)   # fill start / haul end (cyan)
        dot(fill_pts[-1], (160, 0, 200, 255), rr)  # fill end (purple)

    # Crop to the pass bounding box + padding.
    xy = [px(p) for p in pts]
    xs = [p[0] for p in xy]
    ys = [p[1] for p in xy]
    pad = pad_cells * ppb
    x0 = max(0, int(min(xs) - pad))
    y0 = max(0, int(min(ys) - pad))
    x1 = min(base.size[0], int(max(xs) + pad))
    y1 = min(base.size[1], int(max(ys) + pad))
    base.crop((x0, y0, x1, y1)).save(Path(out_path))


def _arrowhead(draw, a, b, color, width, size):
    """Draw an arrowhead at point ``b`` for a segment pointing a -> b."""
    ang = math.atan2(b[1] - a[1], b[0] - a[0])
    for da in (math.radians(150), math.radians(-150)):
        draw.line([(round(b[0]), round(b[1])),
                   (round(b[0] + size * math.cos(ang + da)),
                    round(b[1] + size * math.sin(ang + da)))],
                  fill=color, width=width)


def render_pair_sequence_png(
    grid: Grid,
    pts,
    out_path: Path,
    blade_width_m: float = 2.271,
    pixels_per_bin: int = 26,
    pad_cells: int = 4,
) -> None:
    """Render several consecutive passes as a connected operator track.

    ``pts`` is the ordered list of :class:`landlevel.swaths.GuidancePoint` for two
    or more back-to-back passes. Each leg is coloured by action: CUT red, FILL
    blue, the **grade/compact return green** (the marked pass), HAUL black with an
    arrowhead, and the deadhead RETURN dashed grey. Per-pass transition dots and a
    legend make the cut -> haul -> fill -> grade -> next-cut flow explicit.
    """
    from collections import OrderedDict

    ppb = pixels_per_bin
    base = diff_to_image(grid.delta, grid.valid, ppb).convert("RGBA")
    arr = np.asarray(base).astype(float)
    white = np.array([255, 255, 255], dtype=float)
    arr[..., :3] = arr[..., :3] * 0.4 + white * 0.6
    base = Image.fromarray(arr.astype(np.uint8), "RGBA")
    draw = ImageDraw.Draw(base)

    blade_px = max(3, round(blade_width_m / grid.grid_size_m * ppb))
    colors = {
        "CUT": (200, 0, 0, 255),
        "FILL": (0, 90, 220, 255),
        "GRADE": (30, 160, 30, 255),
        "HAUL": (0, 0, 0, 255),
        "MOVE": (255, 140, 0, 255),
        "RETURN": (110, 110, 110, 255),
        "START": (110, 110, 110, 255),
    }

    def px(p):
        return _world_to_pixel(grid, p.x, p.y, ppb)

    # Draw each leg coloured by the action of the point being entered.
    prev = None
    for pt in pts:
        cur = px(pt)
        if prev is not None:
            act = pt.action
            col = colors.get(act, (0, 0, 0, 255))
            if act in ("CUT", "FILL", "GRADE"):
                w = blade_px
            else:
                w = max(2, blade_px // 3)
            a = (round(prev[0]), round(prev[1]))
            b = (round(cur[0]), round(cur[1]))
            if act in ("RETURN", "START", "MOVE"):
                _dashed_line(draw, a, b, col, w, dash=max(6, blade_px))
            elif act == "GRADE":
                # Dashed green over the (already-drawn) blue fill lane, so both the
                # fill and its grade/compact return pass are visible on the figure.
                _dashed_line(draw, a, b, col, max(2, blade_px // 2), dash=max(6, blade_px))
            else:
                draw.line([a, b], fill=col, width=w, joint="curve")
            if act == "HAUL":
                _arrowhead(draw, prev, cur, col, max(2, blade_px // 2), max(10, blade_px * 1.8))
        prev = cur

    # Group points by pass (preserve order) for transition markers + labels.
    by_pass = OrderedDict()
    for p in pts:
        by_pass.setdefault(p.pass_no, []).append(p)

    def dot(p, color, r):
        x, y = px(p)
        draw.ellipse((x - r, y - r, x + r, y + r), fill=color, outline=(0, 0, 0, 255), width=2)

    rr = max(5, blade_px)
    try:
        from PIL import ImageFont
        font = ImageFont.load_default(size=max(13, blade_px * 2))
    except Exception:  # pragma: no cover
        from PIL import ImageFont
        font = ImageFont.load_default()

    grade_label_xy = []
    for pno, pp in by_pass.items():
        cut = [p for p in pp if p.action in ("START", "RETURN", "CUT")]
        fill = [p for p in pp if p.action in ("HAUL", "FILL")]
        grade = [p for p in pp if p.action == "GRADE"]
        if cut:
            dot(cut[0], (0, 200, 0, 255), rr)       # cut start
            dot(cut[-1], (255, 140, 0, 255), rr)    # cut end / haul start
        if fill:
            dot(fill[0], (0, 220, 255, 255), rr)    # fill start
            dot(fill[-1], (160, 0, 200, 255), rr)   # fill end
        if grade:
            grade_label_xy.append((pno, px(grade[len(grade) // 2])))

    # Crop to the bounding box of all points + padding.
    xy = [px(p) for p in pts]
    xs = [p[0] for p in xy]
    ys = [p[1] for p in xy]
    pad = pad_cells * ppb
    x0 = max(0, int(min(xs) - pad))
    y0 = max(0, int(min(ys) - pad))
    x1 = min(base.size[0], int(max(xs) + pad))
    y1 = min(base.size[1], int(max(ys) + pad))
    cropped = base.crop((x0, y0, x1, y1))

    # Legend (drawn in cropped-image coordinates).
    legend = [
        ("Cut (load)", colors["CUT"]),
        ("Haul (loaded)", colors["HAUL"]),
        ("Fill (spread)", colors["FILL"]),
        ("Grade/compact return", colors["GRADE"]),
        ("Reposition (multi-drop hop)", colors["MOVE"]),
        ("Return (deadhead)", colors["RETURN"]),
    ]
    ld = ImageDraw.Draw(cropped)
    sw = max(18, blade_px * 3)
    lh = max(18, blade_px * 2)
    pad_l = 8
    box_w = sw + 12 + 220
    box_h = pad_l * 2 + lh * len(legend)
    ld.rectangle((6, 6, 6 + box_w, 6 + box_h), fill=(255, 255, 255, 230), outline=(0, 0, 0, 255))
    for i, (label, col) in enumerate(legend):
        yy = 6 + pad_l + i * lh
        ld.line([(14, yy + lh // 2), (14 + sw, yy + lh // 2)], fill=col, width=max(3, blade_px))
        ld.text((14 + sw + 8, yy), label, fill=(0, 0, 0, 255), font=font)

    # Grade-pass callouts, clamped so the text stays inside the cropped image.
    cw, ch = cropped.size
    for pno, (gx, gy) in grade_label_xy:
        text = f"grade/compact (pass {pno})"
        tw = ld.textlength(text, font=font)
        lx = min(max(4, gx - x0 + rr), cw - tw - 4)
        ly = min(max(4, gy - y0 - rr - lh), ch - lh - 2)
        ld.text((lx, ly), text, fill=(0, 90, 0, 255), font=font)

    cropped.save(Path(out_path))


def _dashed_line(draw, a, b, color, width, dash=10):
    """Draw a dashed line from a to b."""
    x0, y0 = a
    x1, y1 = b
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist < 1e-6:
        return
    ux, uy = (x1 - x0) / dist, (y1 - y0) / dist
    n = int(dist // dash)
    for i in range(0, n + 1, 2):
        s = i * dash
        e = min((i + 1) * dash, dist)
        draw.line([(round(x0 + ux * s), round(y0 + uy * s)),
                   (round(x0 + ux * e), round(y0 + uy * e))], fill=color, width=width)


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
