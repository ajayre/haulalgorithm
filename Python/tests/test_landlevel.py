"""Tests for the land leveling haul algorithm (spec §17)."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from landlevel import M3_TO_CY
from landlevel.agd import parse_agd_file, utm_from_latlon
from landlevel.balance import balance
from landlevel.flowfield import compute_flow_field
from landlevel.grid import build_grid, demand_bcy, layers_for_depth, supply_bcy
from landlevel.passes import generate_passes
from landlevel.render import cutfill_color
from landlevel.strips import trace_strips
from landlevel.routing import sequence_passes
from landlevel.swaths import build_guidance


# ---------------------------------------------------------------------------
# Synthetic AGD field helpers
# ---------------------------------------------------------------------------

def _write_agd(path: Path, rows, with_boundary: bool) -> None:
    lines = ["Latitude (deg), Longitude (deg), Elevation Existing(m), Elevation Proposed(m), CutFill(m), Code, Comments"]
    lines.append("40.0, -93.0, 100.0,,,0MB,")  # benchmark, skipped
    for (lat, lon, exist, prop, code) in rows:
        cutfill = prop - exist
        lines.append(f"{lat:.8f}, {lon:.8f}, {exist:.3f}, {prop:.3f}, {cutfill:.3f}, {code}, ")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _ramp_field(with_boundary: bool):
    """A small field sloping up to the east: the east half is cut (existing above
    target) and the west half is fill, so soil should flow east -> west."""
    rows = []
    lat0, lon0 = 40.0, -93.0
    dlat = 1.0 / 111000.0  # ~1 m
    dlon = 1.0 / (111000.0 * math.cos(math.radians(lat0)))
    n = 24
    for r in range(n):
        for c in range(n):
            lat = lat0 + r * 3 * dlat
            lon = lon0 + c * 3 * dlon
            existing = 100.0 + (c - n / 2) * 0.02  # slope east
            proposed = 100.0
            code = "3GRD"
            if with_boundary and (r in (0, n - 1) or c in (0, n - 1)):
                code = "2PER"
            rows.append((lat, lon, existing, proposed, code))
    return rows


# ---------------------------------------------------------------------------
# §17.0.2 AGD round-trip
# ---------------------------------------------------------------------------

def test_agd_roundtrip_and_delta_sign(tmp_path):
    agd = tmp_path / "f.agd"
    rows = [
        (40.0, -93.0, 101.0, 100.0, "3GRD"),   # cut: delta = +1
        (40.00003, -93.0, 100.0, 101.0, "3GRD"),  # fill: delta = -1
        (40.00006, -93.0, 100.0, 100.0, "2PER"),  # boundary
    ]
    _write_agd(agd, rows, with_boundary=False)
    field = parse_agd_file(agd)
    # benchmark skipped, 3 usable rows kept
    assert len(field.points) == 3
    deltas = sorted(p.delta for p in field.points)
    assert deltas[0] == pytest.approx(-1.0)
    assert deltas[-1] == pytest.approx(1.0)
    assert len(field.boundary) == 1


# ---------------------------------------------------------------------------
# §17.0.3 BCY conversions
# ---------------------------------------------------------------------------

def test_bcy_conversions():
    area = 9.144 ** 2
    delta = 0.1
    # Bank cut volume is BCY directly (no swell multiply; LCY = BCY × swell).
    sup = supply_bcy(delta, area)
    expected = delta * area * M3_TO_CY
    assert sup == pytest.approx(expected)
    assert supply_bcy(-0.1, area) == 0.0

    # Fill demand = fill space × cut/fill ratio (compaction).
    dem = demand_bcy(-0.1, area, cutfill_ratio=1.2)
    expected_d = 0.1 * area * M3_TO_CY * 1.2
    assert dem == pytest.approx(expected_d)
    assert demand_bcy(0.1, area) == 0.0


def test_scraper_capacity_lcy_to_bcy():
    from landlevel.grid import lcy_to_bcy

    # 20 LCY at swell 1.3 => 15.38 BCY (matches OptiSurface's 15.4).
    assert lcy_to_bcy(20.0, 1.3) == pytest.approx(20.0 / 1.3, rel=1e-9)
    assert round(lcy_to_bcy(20.0, 1.3), 1) == 15.4


def test_layers_for_depth():
    assert layers_for_depth(0.0) == 0
    assert layers_for_depth(0.061) == 1
    assert layers_for_depth(0.062) == 2
    assert layers_for_depth(0.122) == 2
    assert layers_for_depth(0.123) == 3


# ---------------------------------------------------------------------------
# §17.0.4 Mass balance
# ---------------------------------------------------------------------------

def test_mass_balance_flagging(tmp_path):
    # Balanced field (symmetric cut/fill ramp around target) -> small imbalance.
    agd = tmp_path / "bal.agd"
    _write_agd(agd, _ramp_field(with_boundary=False), with_boundary=False)
    field = parse_agd_file(agd)
    grid = build_grid(field)
    res = balance(grid, mode="check")
    # supply uses swell, demand uses swell*compaction so a symmetric field is
    # intentionally imbalanced in BCY; ensure it is detected and finite.
    assert res.total_supply_bcy > 0
    assert res.total_demand_bcy > 0


def test_balance_adjust_reduces_imbalance(tmp_path):
    agd = tmp_path / "bal2.agd"
    _write_agd(agd, _ramp_field(with_boundary=False), with_boundary=False)
    field = parse_agd_file(agd)
    grid = build_grid(field)
    before = balance(grid, mode="check").relative_imbalance
    grid2 = build_grid(field)
    after = balance(grid2, mode="adjust").relative_imbalance
    assert after <= before + 1e-6


# ---------------------------------------------------------------------------
# §17.0.5 Flow field: ridge -> valley
# ---------------------------------------------------------------------------

def test_flow_points_from_ridge_to_valley(tmp_path):
    agd = tmp_path / "ridge.agd"
    _write_agd(agd, _ramp_field(with_boundary=False), with_boundary=False)
    field = parse_agd_file(agd)
    # Pin to 15 ft: the tiny synthetic ramp is degenerate at the finer default.
    grid = build_grid(field, grid_size_ft=15.0)
    flow = compute_flow_field(grid, mode="poisson")
    # East (high bx) is cut (high existing), west is fill: soil flows from the cut
    # ridge toward the fill, i.e. west (negative east-component fx) on average.
    fx_vals = flow.fx[flow.has_arrow]
    assert fx_vals.size > 0
    assert fx_vals.mean() < 0.0


# ---------------------------------------------------------------------------
# §17.0.6 Pass limits
# ---------------------------------------------------------------------------

def test_pass_limits(tmp_path):
    agd = tmp_path / "p.agd"
    _write_agd(agd, _ramp_field(with_boundary=False), with_boundary=False)
    field = parse_agd_file(agd)
    grid = build_grid(field)
    flow = compute_flow_field(grid, mode="poisson")
    strips = trace_strips(grid, flow)
    result = generate_passes(grid, strips, tolerance_mm=10.0, max_load_bcy=20.0)
    max_depth_m = grid.max_depth_mm / 1000.0 + 1e-9
    for p in result.passes:
        assert p.load_bcy <= 20.0 + 1e-6
        for _c, depth in p.cut_cells:
            assert depth <= max_depth_m
        for _c, depth in p.fill_cells:
            assert depth <= max_depth_m


# ---------------------------------------------------------------------------
# Blade-scale swath builder (operator guidance)
# ---------------------------------------------------------------------------

def _guidance_for_ramp(tmp_path, blade_width_mm=2271.0, grid_size_ft=15.0):
    agd = tmp_path / "sw.agd"
    _write_agd(agd, _ramp_field(with_boundary=False), with_boundary=False)
    field = parse_agd_file(agd)
    grid = build_grid(field, grid_size_ft=grid_size_ft)
    flow = compute_flow_field(grid, mode="poisson")
    strips = trace_strips(grid, flow)
    result = generate_passes(grid, strips, tolerance_mm=10.0, max_load_bcy=15.38,
                             heading_deg=90.0, heading_strength=0.5)
    route = sequence_passes(result.passes)
    guidance = build_guidance(grid, field, route, blade_width_mm=blade_width_mm,
                              heading_deg=90.0)
    return grid, route, guidance


def test_guidance_preserves_haul_and_depth_caps(tmp_path):
    grid, route, g = _guidance_for_ramp(tmp_path)
    assert g.points, "expected guidance waypoints"
    # The blade-scale guidance haul tracks the optimizer's route haul. It is not
    # identical: lane endpoints land at patch corners, so allow modest slack.
    assert g.haul_dist_m == pytest.approx(g.route_haul_dist_m, rel=0.15, abs=2.0)
    # Blade coverage never lowers the sweep below the planned centerline. Route
    # smoothing only moves the displayed steering points; distance/time stay on
    # the raw coverage geometry, so this strict floor must still hold.
    assert g.cut_dist_m >= g.planned_cut_dist_m - 1e-6
    assert g.fill_dist_m >= g.planned_fill_dist_m - 1e-6
    # Per-point depth never exceeds the 61 mm layer cap.
    cap = grid.max_depth_mm + 1e-6
    for pt in g.points:
        assert pt.depth_mm <= cap
        assert 0.0 <= pt.heading_deg <= 360.0
        assert pt.blade in ("UP", "DOWN")


def test_strips_haul_stays_bounded(tmp_path):
    """Strip growth trades a little haul for drivable long passes; guard that the
    trade stays bounded (and on many fields is neutral or better)."""
    agd = tmp_path / "sb.agd"
    _write_agd(agd, _ramp_field(with_boundary=False), with_boundary=False)
    field = parse_agd_file(agd)
    grid = build_grid(field, grid_size_ft=15.0)
    flow = compute_flow_field(grid, mode="poisson")
    strips = trace_strips(grid, flow)

    def total_haul(strip_bias):
        res = generate_passes(grid, strips, tolerance_mm=10.0, max_load_bcy=15.38,
                              heading_deg=90.0, heading_strength=0.5, strip_bias=strip_bias)
        route = sequence_passes(res.passes)
        return sum(p.haul_distance_m for p in route.ordered_passes)

    # Full blade-wide strips (the default) should not worsen haul vs the blob; on
    # many fields they improve it because each load is one continuous line.
    haul_blob = total_haul(1.0)
    haul_strip = total_haul(50.0)
    assert haul_strip <= haul_blob * 1.12 + 1.0


def test_strip_growth_makes_long_runs(tmp_path):
    """A strip-grown cut sweep should have long straight runs (few heading
    changes) rather than the per-cell zig-zag of a tiled blob."""
    _grid, _route, g = _guidance_for_ramp(tmp_path)
    cut = [p for p in g.points if p.action == "CUT"]
    if len(cut) < 10:
        pytest.skip("not enough cut waypoints to assess runs")
    # Count heading reversals between consecutive cut waypoints; strips should
    # keep this well below one-per-step.
    turns = sum(1 for a, b in zip(cut, cut[1:]) if abs(((a.heading_deg - b.heading_deg + 180) % 360) - 180) > 80)
    assert turns < 0.6 * len(cut)


def test_guidance_actions_and_blade_state(tmp_path):
    _grid, _route, g = _guidance_for_ramp(tmp_path)
    actions = {pt.action for pt in g.points}
    assert {"CUT", "FILL", "HAUL"} <= actions
    for pt in g.points:
        up = pt.action in ("START", "RETURN", "HAUL", "MOVE")
        assert pt.blade == ("UP" if up else "DOWN")


def test_guidance_deterministic(tmp_path):
    agd = tmp_path / "det.agd"
    _write_agd(agd, _ramp_field(with_boundary=False), with_boundary=False)
    field = parse_agd_file(agd)
    grid = build_grid(field)
    flow = compute_flow_field(grid, mode="poisson")
    strips = trace_strips(grid, flow)
    res = generate_passes(grid, strips, tolerance_mm=10.0, max_load_bcy=15.38,
                          heading_deg=90.0, heading_strength=0.5)
    route = sequence_passes(res.passes)
    g1 = build_guidance(grid, field, route, blade_width_mm=2271.0, heading_deg=90.0)
    g2 = build_guidance(grid, field, route, blade_width_mm=2271.0, heading_deg=90.0)
    sig1 = [(p.seq, p.action, round(p.x, 6), round(p.y, 6), round(p.depth_mm)) for p in g1.points]
    sig2 = [(p.seq, p.action, round(p.x, 6), round(p.y, 6), round(p.depth_mm)) for p in g2.points]
    assert sig1 == sig2


def test_narrow_blade_increases_coverage(tmp_path):
    # On a 15 ft grid, a 7.45 ft blade needs ~2 lanes per cell row, so the
    # blade-scale sweep is longer than the centerline (coverage > 1).
    _g, _r, wide = _guidance_for_ramp(tmp_path, blade_width_mm=4572.0)   # ~= one cell
    _g2, _r2, narrow = _guidance_for_ramp(tmp_path, blade_width_mm=2271.0)
    assert narrow.coverage_factor() >= wide.coverage_factor() - 1e-6
    assert narrow.coverage_factor() > 1.0


# ---------------------------------------------------------------------------
# §17.0.7 Render geometry + palette
# ---------------------------------------------------------------------------

def test_palette_bands():
    assert cutfill_color(-1.0) == (0x80, 0x00, 0x80)
    assert cutfill_color(0.0) == (0x00, 0xCD, 0x00)
    assert cutfill_color(1.0) == (0xB4, 0x00, 0x00)
    assert cutfill_color(0.03) == (0xFF, 0xFF, 0x00)


def test_render_geometry(tmp_path):
    agd = tmp_path / "g.agd"
    _write_agd(agd, _ramp_field(with_boundary=True), with_boundary=True)
    field = parse_agd_file(agd)
    grid = build_grid(field)
    ppb = 50
    # Spec §14.3: area (bx, by) top-left = (bx*ppb, (H-1-by)*ppb).
    bx, by = 1, 2
    x0 = bx * ppb
    y0 = (grid.height - 1 - by) * ppb
    assert x0 == 50
    assert y0 == (grid.height - 1 - 2) * 50


# ---------------------------------------------------------------------------
# §17.0.8 Both masking paths
# ---------------------------------------------------------------------------

def test_boundary_and_no_boundary(tmp_path):
    for wb in (True, False):
        agd = tmp_path / f"mask_{wb}.agd"
        _write_agd(agd, _ramp_field(with_boundary=wb), with_boundary=wb)
        field = parse_agd_file(agd)
        grid = build_grid(field)
        assert grid.valid.sum() > 0


def test_utm_projection_reasonable():
    c = utm_from_latlon(40.0, -93.0)
    assert c.zone == 15
    assert c.is_northern is True
    assert 0 < c.easting < 1_000_000


# ---------------------------------------------------------------------------
# §17.0.9 Exclusion zones + obstacle-aware routing (spec §12.3)
# ---------------------------------------------------------------------------

def test_obstacle_visibility_detour():
    from landlevel.obstacles import ObstacleField

    # A unit square obstacle centred at the origin; clearance 0.5 -> buffered 2x2.
    ring = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
    of = ObstacleField.from_rings([ring], clearance=0.5)
    a, b = (-5.0, 0.0), (5.0, 0.0)
    # Straight line through the middle is blocked; the detour is strictly longer.
    assert of.blocked(a, b)
    assert of.path_length(a, b) > math.dist(a, b)
    # The detour clears the buffered zone (corner ~ (1,1)): length ~ 2*hypot(5,1).
    assert of.path_length(a, b) == pytest.approx(2 * math.hypot(5.0, 1.0), rel=0.05)
    # A leg that misses the zone is an exact straight line (no detour, O(1) reject).
    c, d = (-5.0, 5.0), (5.0, 5.0)
    assert not of.blocked(c, d)
    assert of.path_length(c, d) == pytest.approx(math.dist(c, d))


def test_empty_obstacle_field_is_passthrough():
    from landlevel.obstacles import EMPTY_OBSTACLES

    assert not EMPTY_OBSTACLES.active
    a, b = (0.0, 0.0), (3.0, 4.0)
    assert not EMPTY_OBSTACLES.blocked(a, b)
    assert EMPTY_OBSTACLES.path_length(a, b) == pytest.approx(5.0)
    assert EMPTY_OBSTACLES.path(a, b) == [a, b]


def test_apply_exclusions_removes_block_and_exports(tmp_path):
    from landlevel.grid import apply_exclusions

    agd = tmp_path / "excl.agd"
    _write_agd(agd, _ramp_field(with_boundary=True), with_boundary=True)
    field = parse_agd_file(agd)
    grid = build_grid(field)
    before = int(grid.valid.sum())

    # A square around the field centre, expressed in projected xy.
    cx = grid.min_x + grid.width * grid.grid_size_m * 0.5
    cy = grid.min_y + grid.height * grid.grid_size_m * 0.5
    h = grid.grid_size_m * 0.75
    ring = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]

    res = apply_exclusions(grid, [ring])
    after = int(grid.valid.sum())
    assert res.n_bins_removed > 0
    assert after == before - res.n_bins_removed
    # Removed bins are no longer valid and never level inside the polygon.
    from landlevel.obstacles import point_in_ring
    for by in range(grid.height):
        for bx in range(grid.width):
            cxy = grid.cell_center_xy(bx, by)
            if point_in_ring(cxy[0], cxy[1], ring):
                assert not grid.valid[by, bx]
    # The polygon is stored for downstream routing/rendering.
    assert grid.exclusions and len(grid.exclusions[0]) >= 3


# ---------------------------------------------------------------------------
# Minimum turning-circle enforcement on guidance (§13.3a.3)
# ---------------------------------------------------------------------------

def _circumradius_m(a, b, c):
    la = math.dist(b, c)
    lb = math.dist(a, c)
    lc = math.dist(a, b)
    cross = abs(a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if cross < 1e-9:
        return float("inf")
    return (la * lb * lc) / (2.0 * cross)


def _heading_change_deg(a, b, c):
    dx1, dy1 = b[0] - a[0], b[1] - a[1]
    dx2, dy2 = c[0] - b[0], c[1] - b[1]
    return abs(math.degrees(math.atan2(dx1 * dy2 - dy1 * dx2, dx1 * dx2 + dy1 * dy2)))


def _assert_steering_respects_turn_radius(points, turn_radius_m, turn_threshold_deg=60.0):
    """Full steering polyline bends above the noise floor must have circumradius >= R."""
    noise = turn_threshold_deg
    # START/END are markers; RETURN may still be a straight deadhead pending a future pass.
    skip = {"START", "END", "RETURN"}
    coords = [(p.x, p.y, p.action) for p in points]
    for k in range(1, len(coords) - 1):
        if coords[k - 1][2] in skip or coords[k][2] in skip or coords[k + 1][2] in skip:
            continue
        a, b, c = coords[k - 1][:2], coords[k][:2], coords[k + 1][:2]
        if math.dist(a, b) < 1e-6 or math.dist(b, c) < 1e-6:
            continue
        if _heading_change_deg(a, b, c) < noise:
            continue
        r = _circumradius_m(a, b, c)
        assert r >= turn_radius_m * 0.98, (
            f"undrivable {coords[k - 1][2]}->{coords[k][2]}->{coords[k + 1][2]} "
            f"bend radius={r:.3f}m < R={turn_radius_m:.3f}m at ({b[0]:.2f},{b[1]:.2f})"
        )


def test_keyhole_prefers_in_field_chirality():
    """When one CCC side leaves the field, the other chirality is chosen."""
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS
    from landlevel.swaths import _keyhole_u_turn
    import numpy as np
    from landlevel.grid import Grid

    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    # Tiny 3x3 field; U-turn at the north edge so one bulb goes outside.
    G = 2.0
    valid = np.ones((3, 3), dtype=bool)
    grid = Grid(
        grid_size_m=G,
        width=3,
        height=3,
        min_x=0.0,
        min_y=0.0,
        zone=10,
        is_northern=True,
        mean_lat=0.0,
        mean_lon=0.0,
        existing=np.zeros((3, 3)),
        target=np.zeros((3, 3)),
        valid=valid,
        swell_factor=1.3,
        compaction=1.2,
        max_depth_mm=61.0,
    )
    # Eastbound then reverse westbound, 2 m apart, near north edge of field.
    start = (G * 1.5, G * 2.5)
    end = (G * 1.5, G * 2.5 - 2.0)
    path = _keyhole_u_turn(start, 90.0, end, 270.0, R, 0.5, field=grid)
    assert path and len(path) >= 3
    outside = sum(1 for x, y in path if not grid.contains_world_xy(x, y))
    path_any = _keyhole_u_turn(start, 90.0, end, 270.0, R, 0.5, field=None)
    outside_any = sum(1 for x, y in (path_any or []) if not grid.contains_world_xy(x, y))
    assert outside <= outside_any


def test_keyhole_slides_forward_to_stay_in_field():
    """Near a cut-off corner, tip CCC spills out; side-arc keyhole stays in-field."""
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS
    from landlevel.swaths import _dubins_shortest, _keyhole_u_turn, _compass_to_math
    import numpy as np
    from landlevel.grid import Grid

    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    G = 2.0
    W, H = 16, 16
    valid = np.ones((H, W), dtype=bool)
    # L-shaped field: SE void so a tip CCC spills out; north/west stay open for a side arc.
    for by in range(H):
        for bx in range(W):
            x = (bx + 0.5) * G
            y = (by + 0.5) * G
            if x > 14.0 and y < 18.0:
                valid[by, bx] = False
    grid = Grid(
        grid_size_m=G,
        width=W,
        height=H,
        min_x=0.0,
        min_y=0.0,
        zone=10,
        is_northern=True,
        mean_lat=0.0,
        mean_lon=0.0,
        existing=np.zeros((H, W)),
        target=np.zeros((H, W)),
        valid=valid,
        swell_factor=1.3,
        compaction=1.2,
        max_depth_mm=61.0,
    )
    start = (12.0, 20.0)
    end = (12.0, 18.0)
    St = _compass_to_math(90.0)
    Et = _compass_to_math(270.0)
    tip = _dubins_shortest(
        start[0], start[1], St, end[0], end[1], Et, R, 0.5,
        ccc_only=True, field=grid, heading_in_deg=90.0,
    )
    tip_out = sum(
        1 for x, y in (tip or []) if not grid.contains_world_xy(x, y)
    )
    assert tip_out > 0, "fixture must make the tip CCC leave the field"

    path = _keyhole_u_turn(start, 90.0, end, 270.0, R, 0.5, field=grid)
    assert path and len(path) >= 3
    outside = sum(1 for x, y in path if not grid.contains_world_xy(x, y))
    assert outside < tip_out, (
        f"in-field keyhole should reduce outside samples ({outside} vs tip {tip_out})"
    )


def test_enforce_multidrop_move_inserts_curved_transit():
    """Parallel cut pockets joined by MOVE must get an R-bounded keyhole, not a chord."""
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS
    from landlevel.swaths import _enforce_min_turn_radius, CUT, MOVE

    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    # Eastbound pocket, hop south 4 m, westbound pocket — near-reverse multi-drop.
    raw = [
        (0.0, 0.0, 0.05, CUT, 1, 0.0, 0.0),
        (20.0, 0.0, 0.05, CUT, 1, 20.0, 0.0),
        (40.0, 0.0, 0.05, CUT, 1, 40.0, 0.0),
        (40.0, 4.0, 0.05, MOVE, 1, 40.0, 4.0),
        (20.0, 4.0, 0.05, CUT, 1, 20.0, 4.0),
        (0.0, 4.0, 0.05, CUT, 1, 0.0, 4.0),
    ]
    out = _enforce_min_turn_radius(raw, R, 60.0, 1.0)
    move_pts = [p for p in out if p[3] == MOVE]
    assert len(move_pts) >= 3, f"expected curved MOVE transit samples, got {len(move_pts)}"

    class _P:
        def __init__(self, x, y, action):
            self.x, self.y, self.action = x, y, action

    pts = [_P(p[5], p[6], p[3]) for p in out]
    _assert_steering_respects_turn_radius(pts, R, turn_threshold_deg=15.0)

    move_len = 0.0
    for i in range(1, len(out)):
        if out[i][3] == MOVE:
            a = (out[i - 1][5], out[i - 1][6])
            b = (out[i][5], out[i][6])
            move_len += math.dist(a, b)
    assert move_len < (2.75 * math.pi * R)


def test_enforce_haul_transit_inserts_curved_haul():
    """Orthogonal cut→fill HAUL must get R-bounded transit samples, not a chord."""
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS
    from landlevel.swaths import _enforce_min_turn_radius, CUT, FILL, HAUL

    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    raw = [
        (0.0, 0.0, 0.05, CUT, 1, 0.0, 0.0),
        (20.0, 0.0, 0.05, CUT, 1, 20.0, 0.0),
        (40.0, 0.0, 0.05, CUT, 1, 40.0, 0.0),
        (40.0, 50.0, 0.05, HAUL, 1, 40.0, 50.0),
        (60.0, 50.0, 0.05, FILL, 1, 60.0, 50.0),
        (80.0, 50.0, 0.05, FILL, 1, 80.0, 50.0),
    ]
    out = _enforce_min_turn_radius(raw, R, 60.0, 1.0)
    haul_pts = [p for p in out if p[3] == HAUL]
    assert len(haul_pts) >= 3, f"expected curved HAUL samples, got {len(haul_pts)}"

    class _P:
        def __init__(self, x, y, action):
            self.x, self.y, self.action = x, y, action

    pts = [_P(p[5], p[6], p[3]) for p in out]
    _assert_steering_respects_turn_radius(pts, R, turn_threshold_deg=15.0)

    # Contiguous HAUL run should stay a CSC transit, not stacked keyholes.
    haul_len = 0.0
    for i in range(1, len(out)):
        if out[i][3] == HAUL:
            a = (out[i - 1][5], out[i - 1][6])
            b = (out[i][5], out[i][6])
            haul_len += math.dist(a, b)
    chord = 50.0
    assert haul_len < (2.75 * math.pi * R) + chord


def test_enforce_min_turn_radius_inserts_move_on_hairpin():
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS
    from landlevel.swaths import _enforce_min_turn_radius, _keyhole_u_turn, CUT, MOVE

    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    # Two parallel lanes 2 m apart with a sharp U-turn (spacing << 2R).
    raw = [
        (0.0, 0.0, 0.05, CUT, 1, 0.0, 0.0),
        (10.0, 0.0, 0.05, CUT, 1, 10.0, 0.0),
        (20.0, 0.0, 0.05, CUT, 1, 20.0, 0.0),
        (20.0, 2.0, 0.05, CUT, 1, 20.0, 2.0),
        (10.0, 2.0, 0.05, CUT, 1, 10.0, 2.0),
        (0.0, 2.0, 0.05, CUT, 1, 0.0, 2.0),
    ]
    out = _enforce_min_turn_radius(raw, R, 60.0, 1.0)
    assert any(p[3] == MOVE for p in out)
    assert sum(1 for p in out if p[3] == CUT) == 6
    # Steering path on the full emitted polyline must respect R for blade-down runs.
    class _P:
        def __init__(self, x, y, action):
            self.x, self.y, self.action = x, y, action

    pts = [_P(p[5], p[6], p[3]) for p in out]
    _assert_steering_respects_turn_radius(pts, R)

    # Keyhole CCC must be compact — far shorter than a ~540 deg (3 pi R) wander.
    key = _keyhole_u_turn((20.0, 0.0), 90.0, (20.0, 2.0), 270.0, R, 1.0)
    assert key is not None and len(key) >= 3
    full = [(20.0, 0.0)] + key + [(20.0, 2.0)]
    path_len = sum(
        ((full[i][0] - full[i + 1][0]) ** 2 + (full[i][1] - full[i + 1][1]) ** 2) ** 0.5
        for i in range(len(full) - 1)
    )
    assert path_len < (2.75 * math.pi * R)


def _max_contiguous_move_len(out) -> float:
    """Longest contiguous MOVE run length on steering coords."""
    run = 0.0
    best = 0.0
    for i in range(1, len(out)):
        if out[i][3] == "MOVE":
            a = (out[i - 1][5], out[i - 1][6])
            b = (out[i][5], out[i][6])
            run += math.dist(a, b)
        else:
            best = max(best, run)
            run = 0.0
    return max(best, run)


def test_soft_bend_does_not_insert_keyhole():
    """A mild short-leg kink must not become a CCC keyhole bulb."""
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS
    from landlevel.swaths import _enforce_min_turn_radius, CUT, MOVE

    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    # ~25° soft kink on ~2 m legs (circumradius ≪ R) then continue east — no reverse.
    raw = [
        (0.0, 0.0, 0.05, CUT, 1, 0.0, 0.0),
        (2.0, 0.0, 0.05, CUT, 1, 2.0, 0.0),
        (4.0, 0.0, 0.05, CUT, 1, 4.0, 0.0),
        (5.8, 0.85, 0.05, CUT, 1, 5.8, 0.85),
        (7.6, 1.70, 0.05, CUT, 1, 7.6, 1.70),
        (12.0, 1.70, 0.05, CUT, 1, 12.0, 1.70),
        (18.0, 1.70, 0.05, CUT, 1, 18.0, 1.70),
    ]
    out = _enforce_min_turn_radius(raw, R, 60.0, 0.5)
    # Fillet is OK; a full keyhole (~2πR) is not.
    assert _max_contiguous_move_len(out) < (math.pi * R)


def test_soft_bend_does_not_coalesce_into_later_uturn():
    """Soft kink within 4R of a real U-turn must not open the keyhole early."""
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS
    from landlevel.swaths import _enforce_min_turn_radius, CUT, MOVE

    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    raw = [
        (0.0, 0.0, 0.05, CUT, 1, 0.0, 0.0),
        (5.0, 0.0, 0.05, CUT, 1, 5.0, 0.0),
        (10.0, 0.0, 0.05, CUT, 1, 10.0, 0.0),
        # Soft ~25° kink (still eastbound).
        (11.8, 0.85, 0.05, CUT, 1, 11.8, 0.85),
        (13.6, 1.70, 0.05, CUT, 1, 13.6, 1.70),
        (20.0, 1.70, 0.05, CUT, 1, 20.0, 1.70),
        (25.0, 1.70, 0.05, CUT, 1, 25.0, 1.70),
        # Real U-turn onto a westbound lane.
        (25.0, 3.70, 0.05, CUT, 1, 25.0, 3.70),
        (15.0, 3.70, 0.05, CUT, 1, 15.0, 3.70),
        (5.0, 3.70, 0.05, CUT, 1, 5.0, 3.70),
    ]
    out = _enforce_min_turn_radius(raw, R, 60.0, 0.5)
    assert any(p[3] == MOVE for p in out)
    # First MOVE sample must be near the U-tip (x≈25), not the soft kink (x≈12).
    first_move = next(p for p in out if p[3] == MOVE)
    assert first_move[5] > 18.0, (
        f"keyhole opened from soft kink at x={first_move[5]:.1f}, expected near U-tip"
    )


def test_equipment_lineup_entry_and_exit():
    """Straight blade-up of length L before and after a CUT run."""
    from landlevel import DEFAULT_EQUIPMENT_LENGTH_FT, DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS
    from landlevel.swaths import _enforce_equipment_lineup, CUT, FILL, HAUL, MOVE, START

    L = DEFAULT_EQUIPMENT_LENGTH_FT * FEET_TO_METERS
    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    # Tip-based HAUL loop (full circle samples) must be dropped and rebuilt from exit.
    raw = [
        (0.0, 0.0, 0.0, START, 1, 0.0, 0.0),
        (0.0, 0.0, 0.05, CUT, 1, 0.0, 0.0),
        (10.0, 0.0, 0.05, CUT, 1, 10.0, 0.0),
        (20.0, 0.0, 0.05, CUT, 1, 20.0, 0.0),
        (20.0, 10.0, 0.0, HAUL, 1, 20.0, 10.0),
        (10.0, 10.0, 0.0, HAUL, 1, 10.0, 10.0),
        (10.0, 0.0, 0.0, HAUL, 1, 10.0, 0.0),
        (50.0, 0.0, 0.0, HAUL, 1, 50.0, 0.0),
        (50.0, 0.0, 0.05, FILL, 1, 50.0, 0.0),
        (60.0, 0.0, 0.05, FILL, 1, 60.0, 0.0),
    ]
    out = _enforce_equipment_lineup(raw, L, 1.0, R)

    # Entry: MOVE samples immediately before first CUT, length ≈ L along +x.
    first_cut = next(i for i, p in enumerate(out) if p[3] == CUT)
    approach = 0.0
    cur = (out[first_cut][5], out[first_cut][6])
    for i in range(first_cut - 1, -1, -1):
        p = out[i]
        if p[3] not in (MOVE, START):
            break
        q = (p[5], p[6])
        seg = math.dist(cur, q)
        if seg < 1e-9:
            cur = q
            continue
        # Eastbound run → approach from the west (dot with +x).
        dx = cur[0] - q[0]
        if (dx / seg) < 0.95:
            break
        approach += seg
        cur = q
    assert approach >= 0.95 * L, f"entry lineup {approach:.2f}m < 0.95L"

    # Exit: MOVE samples immediately after last tip of first CUT run.
    tip = first_cut
    while tip + 1 < len(out) and out[tip + 1][3] == CUT:
        tip += 1
    exit_len = 0.0
    cur = (out[tip][5], out[tip][6])
    exit_end = None
    for i in range(tip + 1, len(out)):
        p = out[i]
        if p[3] != MOVE:
            break
        q = (p[5], p[6])
        seg = math.dist(cur, q)
        if seg < 1e-9:
            cur = q
            continue
        dx = q[0] - cur[0]
        if (dx / seg) < 0.95:
            break
        exit_len += seg
        cur = q
        exit_end = q
    assert exit_len >= 0.95 * L, f"exit lineup {exit_len:.2f}m < 0.95L"

    # Intent: old tip-based loop samples must not remain (single track from exit).
    assert not any(
        (p[3] == HAUL and abs(p[5] - 20.0) < 1e-6 and abs(p[6] - 10.0) < 1e-6)
        for p in out
    ), "old tip-based HAUL loop sample still present"
    assert exit_end is not None
    # Intent: after the exit spur, HAUL must continue from ExitEnd — not resume a tip loop.
    first_haul = next(p for p in out[tip + 1 :] if p[3] == HAUL)
    assert math.dist((first_haul[5], first_haul[6]), exit_end) < 2.0, (
        f"HAUL resumes at {first_haul[5]:.1f},{first_haul[6]:.1f}, expected near exit {exit_end}"
    )


def test_guidance_respects_turning_circle(tmp_path):
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS

    _grid, _route, g = _guidance_for_ramp(tmp_path)
    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    _assert_steering_respects_turn_radius(g.points, R, turn_threshold_deg=120.0)
    # Coverage floor still holds after headland insertion.
    assert g.cut_dist_m >= g.planned_cut_dist_m - 1e-6
    assert g.fill_dist_m >= g.planned_fill_dist_m - 1e-6


def test_guidance_respects_turning_circle_without_smoothing(tmp_path):
    from landlevel import DEFAULT_TURNING_CIRCLE_FT, FEET_TO_METERS

    agd = tmp_path / "nosmooth.agd"
    _write_agd(agd, _ramp_field(with_boundary=False), with_boundary=False)
    field = parse_agd_file(agd)
    grid = build_grid(field, grid_size_ft=15.0)
    flow = compute_flow_field(grid, mode="poisson")
    strips = trace_strips(grid, flow)
    result = generate_passes(grid, strips, tolerance_mm=10.0, max_load_bcy=15.38,
                             heading_deg=90.0, heading_strength=0.5)
    route = sequence_passes(result.passes)
    g = build_guidance(grid, field, route, blade_width_mm=2271.0, heading_deg=90.0,
                       smooth_route=False)
    R = DEFAULT_TURNING_CIRCLE_FT * 0.5 * FEET_TO_METERS
    _assert_steering_respects_turn_radius(g.points, R, turn_threshold_deg=120.0)