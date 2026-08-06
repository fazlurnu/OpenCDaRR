"""WindField — the steady-uniform wind value (Phase 5a).

The load-bearing check is the Eq 1 sign convention: meteorological "coming-from" bearings map to
the correct velocity-vector components. Getting this sign wrong flips every downwind result, so it
is pinned for all four cardinal winds. Plus: ``NO_WIND`` is zero, and ``from_met`` round-trips.
"""

from __future__ import annotations

import math

from opencdarr.kinematics import FixedWing, MotionCommand, Multirotor
from opencdarr.performance import M600, SMALL_FIXEDWING
from opencdarr.state import AircraftState
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


# --- 5a wind plumbing: passing wind=NO_WIND explicitly is byte-identical to omitting it, for both
# airframes (the second half of the 5a gate — wind is inert until a non-zero field is supplied).
def test_no_wind_step_is_identical_to_omitting_wind() -> None:
    """``step(..., wind=NO_WIND) == step(...)`` for the multirotor and the fixed-wing."""
    mr_state = AircraftState(id="M", lat=52.0, lon=4.0, trk=30.0, gs=10.0, yaw=45.0)
    mr_cmd = MotionCommand.from_track_speed(90.0, 12.0)
    assert Multirotor().step(mr_state, mr_cmd, M600, 1.0) == Multirotor().step(
        mr_state, mr_cmd, M600, 1.0, NO_WIND
    )
    fw_state = AircraftState(id="F", lat=52.0, lon=4.0, trk=0.0, gs=17.0, yaw=0.0, bank=0.0)
    fw_cmd = MotionCommand(target_course=20.0, target_airspeed=18.0)
    assert FixedWing().step(fw_state, fw_cmd, SMALL_FIXEDWING, 0.1) == FixedWing().step(
        fw_state, fw_cmd, SMALL_FIXEDWING, 0.1, NO_WIND
    )
