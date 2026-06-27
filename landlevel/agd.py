"""AGD file reader and UTM projection (spec §6.1, §7.1).

Parsing rules mirror ``extract_agd_heights.py`` and ``fielddesign.py``:

* The first line is a header and is skipped.
* Fields are comma separated; surrounding whitespace is stripped.
* A row whose Code (case-insensitive) contains ``MB`` or ``BM`` is a benchmark
  and is skipped for grid construction.
* A row whose Code is ``2PER`` is a boundary/perimeter point.
* A data row is used only if both Existing and Proposed elevations are present
  and non-zero.
* On disk ``CutFill = Proposed - Existing``; the reader does **not** trust that
  column and recomputes ``delta = Existing - Proposed`` so positive == cut.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# UTM WGS-84 projection (ported from fielddesign.py utm_from_latlon)
# ---------------------------------------------------------------------------

_UTM_A = 6378137.0
_UTM_F = 1.0 / 298.257223563
_UTM_K0 = 0.9996
_UTM_B = _UTM_A * (1.0 - _UTM_F)
_UTM_E2 = 1.0 - (_UTM_B * _UTM_B) / (_UTM_A * _UTM_A)
_UTM_EPRIME2 = _UTM_E2 / (1.0 - _UTM_E2)


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


@dataclass(frozen=True)
class UTMCoord:
    zone: int
    is_northern: bool
    easting: float
    northing: float


def utm_from_latlon(latitude_deg: float, longitude_deg: float) -> UTMCoord:
    """Project a WGS-84 lat/lon to UTM easting/northing in metres."""
    if latitude_deg < -80.0 or latitude_deg > 84.0:
        raise ValueError("Latitude out of UTM bounds (-80 to 84 degrees).")

    lon_norm = ((longitude_deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0
    zone = int(math.floor((lon_norm + 180.0) / 6.0) + 1)
    is_northern = latitude_deg >= 0.0

    lat_rad = _deg_to_rad(latitude_deg)
    lon_rad = _deg_to_rad(lon_norm)
    lon_origin = (zone - 1) * 6 - 180 + 3
    lon_origin_rad = _deg_to_rad(lon_origin)

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    N = _UTM_A / math.sqrt(1.0 - _UTM_E2 * sin_lat * sin_lat)
    T = math.tan(lat_rad) ** 2
    C = _UTM_EPRIME2 * cos_lat * cos_lat
    A = cos_lat * (lon_rad - lon_origin_rad)

    M = _UTM_A * (
        (1.0 - _UTM_E2 / 4.0 - 3.0 * _UTM_E2 * _UTM_E2 / 64.0 - 5.0 * _UTM_E2 ** 3 / 256.0) * lat_rad
        - (3.0 * _UTM_E2 / 8.0 + 3.0 * _UTM_E2 * _UTM_E2 / 32.0 + 45.0 * _UTM_E2 ** 3 / 1024.0)
        * math.sin(2.0 * lat_rad)
        + (15.0 * _UTM_E2 * _UTM_E2 / 256.0 + 45.0 * _UTM_E2 ** 3 / 1024.0) * math.sin(4.0 * lat_rad)
        - (35.0 * _UTM_E2 ** 3 / 3072.0) * math.sin(6.0 * lat_rad)
    )

    x = _UTM_K0 * N * (
        A
        + (1.0 - T + C) * A ** 3 / 6.0
        + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * _UTM_EPRIME2) * A ** 5 / 120.0
    )
    y = _UTM_K0 * (
        M
        + N
        * math.tan(lat_rad)
        * (
            A * A / 2.0
            + (5.0 - T + 9.0 * C + 4.0 * C * C) * A ** 4 / 24.0
            + (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * _UTM_EPRIME2) * A ** 6 / 720.0
        )
    )

    easting = x + 500000.0
    northing = y + (0.0 if is_northern else 10000000.0)
    return UTMCoord(zone, is_northern, easting, northing)


def latlon_from_utm(easting: float, northing: float, zone: int, is_northern: bool) -> Tuple[float, float]:
    """Inverse of :func:`utm_from_latlon`: UTM easting/northing -> (lat, lon) degrees."""
    x = easting - 500000.0
    y = northing - (0.0 if is_northern else 10000000.0)

    M = y / _UTM_K0
    mu = M / (_UTM_A * (1.0 - _UTM_E2 / 4.0 - 3.0 * _UTM_E2 ** 2 / 64.0 - 5.0 * _UTM_E2 ** 3 / 256.0))
    e1 = (1.0 - math.sqrt(1.0 - _UTM_E2)) / (1.0 + math.sqrt(1.0 - _UTM_E2))

    phi1 = (
        mu
        + (3.0 * e1 / 2.0 - 27.0 * e1 ** 3 / 32.0) * math.sin(2.0 * mu)
        + (21.0 * e1 ** 2 / 16.0 - 55.0 * e1 ** 4 / 32.0) * math.sin(4.0 * mu)
        + (151.0 * e1 ** 3 / 96.0) * math.sin(6.0 * mu)
        + (1097.0 * e1 ** 4 / 512.0) * math.sin(8.0 * mu)
    )

    sin_p = math.sin(phi1)
    cos_p = math.cos(phi1)
    tan_p = math.tan(phi1)
    N1 = _UTM_A / math.sqrt(1.0 - _UTM_E2 * sin_p * sin_p)
    T1 = tan_p * tan_p
    C1 = _UTM_EPRIME2 * cos_p * cos_p
    R1 = _UTM_A * (1.0 - _UTM_E2) / (1.0 - _UTM_E2 * sin_p * sin_p) ** 1.5
    D = x / (N1 * _UTM_K0)

    lat_rad = phi1 - (N1 * tan_p / R1) * (
        D ** 2 / 2.0
        - (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * _UTM_EPRIME2) * D ** 4 / 24.0
        + (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * _UTM_EPRIME2 - 3.0 * C1 * C1)
        * D ** 6 / 720.0
    )
    lon_rad_offset = (
        D
        - (1.0 + 2.0 * T1 + C1) * D ** 3 / 6.0
        + (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * _UTM_EPRIME2 + 24.0 * T1 * T1)
        * D ** 5 / 120.0
    ) / cos_p

    lon_origin = (zone - 1) * 6 - 180 + 3
    lat_deg = lat_rad * 180.0 / math.pi
    lon_deg = lon_origin + lon_rad_offset * 180.0 / math.pi
    return lat_deg, lon_deg


# ---------------------------------------------------------------------------
# AGD points
# ---------------------------------------------------------------------------


@dataclass
class AgdPoint:
    """One parsed AGD data point in projected metres.

    ``delta`` is ``existing - proposed`` (metres, positive == cut), recomputed
    from columns [2] and [3] regardless of the on-disk CutFill column.
    """

    lat: float
    lon: float
    existing: float
    proposed: float
    is_boundary: bool
    easting: float = 0.0
    northing: float = 0.0

    @property
    def delta(self) -> float:
        return self.existing - self.proposed


@dataclass
class AgdField:
    """All projected interior/boundary points from an AGD file."""

    points: List[AgdPoint]
    boundary: List[AgdPoint]
    zone: int
    is_northern: bool
    mean_lat: float
    mean_lon: float
    skipped: int

    @property
    def all_used(self) -> List[AgdPoint]:
        return self.points


def _log_warning(message: str) -> None:
    sys.stderr.write(f"WARNING: {message}\n")


def parse_agd_file(path: Path) -> AgdField:
    """Parse an AGD file into projected interior + boundary points (spec §6.1)."""
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        text = path.read_text(errors="ignore")

    lines = text.splitlines()
    interior: List[AgdPoint] = []
    boundary: List[AgdPoint] = []
    skipped = 0

    for line_number, line in enumerate(lines[1:], start=2):  # skip header
        raw = line.strip()
        if not raw:
            continue
        parts = [p.strip() for p in raw.split(",")]
        if len(parts) < 4:
            skipped += 1
            continue

        lat_s, lon_s, existing_s, proposed_s = parts[:4]
        code_s = parts[5].strip() if len(parts) > 5 else ""
        code_upper = code_s.upper()

        # Benchmarks are skipped for grid construction.
        if "MB" in code_upper or "BM" in code_upper:
            continue

        if not existing_s or not proposed_s:
            skipped += 1
            continue

        try:
            lat = float(lat_s)
            lon = float(lon_s)
            existing = float(existing_s)
            proposed = float(proposed_s)
        except ValueError:
            skipped += 1
            continue

        if existing == 0.0 or proposed == 0.0:
            skipped += 1
            continue

        is_boundary = code_upper == "2PER"
        pt = AgdPoint(lat=lat, lon=lon, existing=existing, proposed=proposed, is_boundary=is_boundary)
        if is_boundary:
            boundary.append(pt)
        interior.append(pt)

    if not interior:
        raise ValueError(f"No valid elevation points found in AGD file: {path}")

    # Project to UTM with a single zone; reject zone-crossing fields.
    zones = set()
    norths = set()
    sum_lat = 0.0
    sum_lon = 0.0
    for pt in interior:
        c = utm_from_latlon(pt.lat, pt.lon)
        pt.easting = c.easting
        pt.northing = c.northing
        zones.add(c.zone)
        norths.add(c.is_northern)
        sum_lat += pt.lat
        sum_lon += pt.lon

    if len(zones) > 1 or len(norths) > 1:
        raise ValueError(
            f"Field crosses a UTM zone boundary (zones={sorted(zones)}); not supported."
        )

    n = len(interior)
    field = AgdField(
        points=interior,
        boundary=boundary,
        zone=zones.pop(),
        is_northern=norths.pop(),
        mean_lat=sum_lat / n,
        mean_lon=sum_lon / n,
        skipped=skipped,
    )
    if skipped:
        _log_warning(f"Skipped {skipped} unusable AGD rows (missing/zero/short).")
    return field
