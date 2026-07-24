"""WindField — the steady-uniform wind value (Phase 5a).

The load-bearing check is the Eq 1 sign convention: meteorological "coming-from" bearings map to
the correct velocity-vector components. Getting this sign wrong flips every downwind result, so it
is pinned for all four cardinal winds. Plus: ``NO_WIND`` is zero, and ``from_met`` round-trips.
"""

from __future__ import annotations

import math

from opencdarr.wind import NO_WIND, WindField


def test_no_wind_is_zero() -> None:
    """The calm field has zero components and zero speed."""
    assert NO_WIND.components() == (0.0, 0.0)
    assert NO_WIND.speed == 0.0


def test_cardinal_winds_map_to_correct_vectors() -> None:
    """Eq 1: a wind 'coming from' X moves the air the opposite way (aviation convention)."""
    north = WindField.from_met(0.0, 5.0)  # from the north -> air moves south
    assert math.isclose(north.w_east, 0.0, abs_tol=1e-12)
    assert math.isclose(north.w_north, -5.0)
    west = WindField.from_met(270.0, 5.0)  # from the west -> air moves east
    assert math.isclose(west.w_east, 5.0)
    assert math.isclose(west.w_north, 0.0, abs_tol=1e-12)
    east = WindField.from_met(90.0, 5.0)  # from the east -> air moves west
    assert math.isclose(east.w_east, -5.0)
    assert math.isclose(east.w_north, 0.0, abs_tol=1e-12)
    south = WindField.from_met(180.0, 5.0)  # from the south -> air moves north
    assert math.isclose(south.w_east, 0.0, abs_tol=1e-12)
    assert math.isclose(south.w_north, 5.0)


def test_speed_and_coming_from_round_trip() -> None:
    """``speed`` recovers the magnitude; ``coming_from`` inverts ``from_met``."""
    for bearing in (0.0, 30.0, 123.0, 270.0, 359.0):
        w = WindField.from_met(bearing, 7.5)
        assert math.isclose(w.speed, 7.5)
        assert math.isclose(w.coming_from, bearing, abs_tol=1e-9)
    assert NO_WIND.coming_from == 0.0  # calm has no direction -> defined as 0.0
