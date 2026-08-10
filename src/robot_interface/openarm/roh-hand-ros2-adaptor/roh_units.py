"""Unit conversion between ROH SDK raw counts and canonical radians.

Canonical topic convention (see pipeline-roh-hand/util_paths.py):
- 0 counts  = fully open   = 0.0 rad
- 65535 counts = fully closed = calibration.closed_rad[finger] (per finger)

Per-finger closed_rad values live in config/roh_hand.yaml under
``calibration.closed_rad``. They are PLACEHOLDERS (default 1.6 rad ≈ 92°)
until measured on hardware — see the plan doc, D4.
"""

from __future__ import annotations

import math

COUNTS_OPEN = 0
COUNTS_CLOSED = 65535
NUM_FINGERS = 6

# Semantic sanity bound: anything beyond this is not a plausible radian value
# for these fingers (methodology: unit sanity by range).
RAD_SANITY_MAX = 2.0 * math.pi


def counts_to_rad(counts: int, closed_rad: float) -> float:
    """Convert raw SDK counts to radians (0 = open)."""
    c = min(max(int(counts), COUNTS_OPEN), COUNTS_CLOSED)
    return (c / COUNTS_CLOSED) * closed_rad


def rad_to_counts(rad: float, closed_rad: float) -> int:
    """Convert radians to raw SDK counts, clamped to the valid range."""
    if not math.isfinite(rad):
        raise ValueError(f"non-finite radian command: {rad}")
    if abs(rad) > RAD_SANITY_MAX:
        raise ValueError(f"radian command {rad} exceeds sanity bound ±2π")
    if closed_rad <= 0:
        raise ValueError(f"closed_rad must be positive, got {closed_rad}")
    frac = min(max(rad / closed_rad, 0.0), 1.0)
    return int(round(frac * COUNTS_CLOSED))


def validate_command(positions_rad: list[float], closed_rad: list[float]) -> None:
    """Validate a 6-finger radian command before conversion."""
    if len(positions_rad) != NUM_FINGERS:
        raise ValueError(f"expected {NUM_FINGERS} joint positions, got {len(positions_rad)}")
    if len(closed_rad) != NUM_FINGERS:
        raise ValueError(f"closed_rad must have {NUM_FINGERS} entries, got {len(closed_rad)}")
    for i, rad in enumerate(positions_rad):
        rad_to_counts(rad, closed_rad[i])  # raises on non-finite / out-of-sanity
