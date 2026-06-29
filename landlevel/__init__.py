"""Land Leveling Haul Algorithm.

A pipeline that reads an OptiSurface-style AGD field file, plans a sequence of
cut-haul-fill operations to bring the field to its target grade with minimum
haul distance, and renders a PNG of the cut/fill map plus haul-direction arrows.

See LAND-LEVELING-SPEC-2.md for the authoritative specification.
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
# 7.5 ft ~= one blade width (2271 mm), so each scraper load is one continuous
# blade-wide lane rather than a multi-lane patch that would force tight turns.
DEFAULT_GRID_SIZE_FT = 7.5
DEFAULT_TOLERANCE_MM = 10.0
# Scraper capacity is specified in LCY (loose, in the scraper). LCY = BCY × swell,
# so the per-pass bank limit is scraper_lcy / swell (20 LCY ÷ 1.3 ≈ 15.4 BCY).
DEFAULT_SCRAPER_CAPACITY_LCY = 20.0
DEFAULT_SWELL_FACTOR = 1.3
DEFAULT_COMPACTION = 1.2          # cut/fill ratio: bank cut per bank fill
DEFAULT_MAX_DEPTH_MM = 61.0       # 0.2 ft
# Multi-drop: how far the scraper will drive (blade up) to the next cut/fill
# pocket to keep loading/spreading the same load, instead of ending the pass
# partially full and paying for a whole separate cut-haul-fill-grade cycle later.
# Within this hop, continuing the current load is cheaper than a new pass.
DEFAULT_MAX_DROP_HOP_FT = 100.0
DEFAULT_BLADE_WIDTH_MM = 2271.0
DEFAULT_FIELD_EFFICIENCY = 0.8    # 80%: accounts for "Other" (lost) time

DEFAULT_LOADING_SPEED_MPH = 4.0
DEFAULT_HAUL_SPEED_MPH = 10.0
DEFAULT_DUMPING_SPEED_MPH = 5.0
DEFAULT_RETURN_SPEED_MPH = 13.0
DEFAULT_ACCEL_TIME_S = 12.0
DEFAULT_DECEL_TIME_S = 12.0

# Maneuvering / finishing (operator-guidance realism).
DEFAULT_TURN_SPEED_MPH = 4.0        # speed through a turn (machine slows to turn)
DEFAULT_GRADE_SPEED_MPH = 8.0       # speed of the grade/compact return pass over a fill
DEFAULT_TURNING_CIRCLE_FT = 45.0    # tractor+scraper turning circle (diameter)
# Equipment (machine) width used as the clearance buffer around exclusion zones:
# the no-go polygon is offset outward by half this so the machine BODY, not just
# the GPS centerline, stays out of the zone (spec §12.3).
DEFAULT_EQUIPMENT_WIDTH_FT = 14.0
# Heading changes below this are gentle bends taken in stride at travel speed
# (no slowdown / no turn-time); only sharper turns incur the slow-arc cost.
DEFAULT_TURN_THRESHOLD_DEG = 60.0

DEFAULT_PIXELS_PER_BIN = 50
DEFAULT_SEED = 1234
