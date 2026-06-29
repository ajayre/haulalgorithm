"""Frame-by-frame leveling simulation.

Reconstructs the field surface as each cut-haul-fill pass is applied (in the
optimized route order) and renders a PNG per frame, so the sequence can be
animated. Two render modes are available:

* ``cutfill`` (default) - the cut/fill map, which turns green as the field
  reaches grade.
* ``topology`` - the actual elevation surface on a terrain ramp, which visibly
  flattens as the ridges are cut and the hollows filled.

Frames may also be assembled into an animated GIF.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import numpy as np

from .grid import Grid
from .passes import Pass
from .render import diff_to_image, elevation_to_image

try:
    from PIL import Image
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Pillow is required. Install with: pip install Pillow") from exc


def animate_leveling(
    grid: Grid,
    ordered_passes: List[Pass],
    out_dir: Optional[Path] = None,
    stride: int = 10,
    pixels_per_bin: int = 8,
    gif_path: Optional[Path] = None,
    gif_ms: int = 80,
    mode: str = "cutfill",
) -> int:
    """Render a frame per ``stride`` applied passes; optionally build a GIF.

    ``mode`` is ``"cutfill"`` (cut/fill map) or ``"topology"`` (elevation terrain
    ramp). Returns the number of frames produced. Frame 0 is the initial surface
    and the final pass always produces a frame.
    """
    if out_dir is None and gif_path is None:
        return 0

    existing = grid.existing.astype(float).copy()
    target = grid.target
    valid = grid.valid

    # Fixed elevation colour scale (initial surface range) for topology mode, so
    # the terrain visibly flattens toward grade instead of re-normalising.
    if mode == "topology":
        ev = existing[valid]
        tv = target[valid]
        vmin = float(min(np.nanmin(ev), np.nanmin(tv)))
        vmax = float(max(np.nanmax(ev), np.nanmax(tv)))
    else:
        vmin = vmax = 0.0

    if out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

    gif_frames: List["Image.Image"] = []
    frame_no = 0

    def emit() -> None:
        nonlocal frame_no
        if mode == "topology":
            img = elevation_to_image(existing, valid, pixels_per_bin, vmin, vmax)
        else:
            img = diff_to_image(existing - target, valid, pixels_per_bin)
        if out_dir is not None:
            img.save(out_dir / f"frame_{frame_no:05d}.png")
        if gif_path is not None:
            # Composite onto white so the GIF has a solid background.
            bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
            gif_frames.append(Image.alpha_composite(bg, img).convert("P", palette=Image.ADAPTIVE))
        frame_no += 1

    emit()  # initial surface (frame 0)

    n = len(ordered_passes)
    for i, p in enumerate(ordered_passes, start=1):
        for (bx, by), depth in p.cut_cells:
            existing[by, bx] -= depth
        for (bx, by), depth in p.fill_cells:
            existing[by, bx] += depth
        if (i % stride == 0) or (i == n):
            emit()

    if gif_path is not None and gif_frames:
        # Hold the final frame for 4 s so the end result is visible.
        durations = [gif_ms] * len(gif_frames)
        durations[-1] = 4000
        gif_frames[0].save(
            Path(gif_path),
            save_all=True,
            append_images=gif_frames[1:],
            duration=durations,
            loop=0,
            disposal=2,
        )

    return frame_no
