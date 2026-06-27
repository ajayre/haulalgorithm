"""Land Leveling Haul Algorithm.

A pipeline that reads an OptiSurface-style AGD field file, plans a sequence of
cut-haul-fill operations to bring the field to its target grade with minimum
haul distance, and renders a PNG of the cut/fill map plus haul-direction arrows.

See LAND-LEVELING-SPEC.md for the authoritative specification.
"""

from __future__ import annotations

__version__ = "0.0.1"

# ---------------------------------------------------------------------------
# Physical / unit constants (spec §7.3, §6.2)
# ---------------------------------------------------------------------------

# Cubic metres -> cubic yards (matches fielddesign.py).
M3_TO_CY = 1.30795061931439

FEET_TO_METERS = 0.3048
MPH_TO_MPS = 0.44704

# Defaults (spec §6.2, §6.3, §6.4).
DEFAULT_GRID_SIZE_FT = 15.0
DEFAULT_TOLERANCE_MM = 10.0
# Scraper capacity is specified in LCY (loose, in the scraper). LCY = BCY × swell,
# so the per-pass bank limit is scraper_lcy / swell (20 LCY ÷ 1.3 ≈ 15.4 BCY).
DEFAULT_SCRAPER_CAPACITY_LCY = 20.0
DEFAULT_SWELL_FACTOR = 1.3
DEFAULT_COMPACTION = 1.2          # cut/fill ratio: bank cut per bank fill
DEFAULT_MAX_DEPTH_MM = 61.0       # 0.2 ft
DEFAULT_BLADE_WIDTH_MM = 2271.0
DEFAULT_FIELD_EFFICIENCY = 0.8    # 80%: accounts for "Other" (lost) time

DEFAULT_LOADING_SPEED_MPH = 4.0
DEFAULT_HAUL_SPEED_MPH = 10.0
DEFAULT_DUMPING_SPEED_MPH = 5.0
DEFAULT_RETURN_SPEED_MPH = 13.0
DEFAULT_ACCEL_TIME_S = 12.0
DEFAULT_DECEL_TIME_S = 12.0

DEFAULT_PIXELS_PER_BIN = 50
DEFAULT_SEED = 1234
