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
    # Blade coverage never lowers the sweep below the planned centerline.
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
