"""Multirotor under wind (Phase 5b) — ground-velocity command, airspeed-limited envelope.

Decision 4 made concrete: ``target_velocity`` is a **ground** velocity, but ``v_max``/``ax`` limit
the **airspeed**. Three regimes, on a small multirotor (``v_max = 8``) so the envelope binds at
realistic winds:

- **feasible** — the commanded ground velocity is met *exactly* by crabbing into the wind;
- **infeasible** — required airspeed exceeds ``v_max``, so it clamps and **drifts downwind**;
- **station-keeping** — a zero ground command **hovers into wind** when ``V_WS ≤ v_max``, and is
  **blown downwind** when the wind exceeds it.

Plus the ``V_WS = 0`` regression: with ``NO_WIND`` the model is byte-identical to Phase 4.
"""

from __future__ import annotations

import math

from opencdarr import geo
from opencdarr.dynamics import MotionCommand, Multirotor
from opencdarr.performance import Performance
from opencdarr.relative import velocity_enu
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField

# A small multirotor: top speed 8 m/s so a single-digit wind can exceed the envelope.
_SMALL = Performance(v_max=8.0, v_min=-8.0, ax=4.0, yaw_rate_max=90.0)
_MR = Multirotor()


def _at(gs: float, trk: float = 0.0) -> AircraftState:
    return AircraftState(id="M", lat=52.0, lon=4.0, trk=trk, gs=gs)


def _settle(
    state: AircraftState, cmd: MotionCommand, wind: WindField, n: int = 400
) -> AircraftState:
    for _ in range(n):
        state = _MR.step(state, cmd, _SMALL, 0.1, wind)
    return state


def test_feasible_command_is_met_exactly_by_crabbing() -> None:
    """A ground velocity within the airspeed envelope is met exactly, whatever the crosswind."""
    cmd = MotionCommand.from_track_speed(0.0, 5.0)  # 5 m/s north (ground)
    wind = WindField.from_met(270.0, 4.0)  # 4 m/s from the west; req airspeed = √41 ≈ 6.4 < 8
    settled = _settle(_at(5.0), cmd, wind)
    ve, vn = velocity_enu(settled)
    assert math.isclose(ve, 0.0, abs_tol=1e-6)  # no east drift — the crab cancels it
    assert math.isclose(vn, 5.0, abs_tol=1e-6)  # exact north ground speed


def test_infeasible_command_clamps_and_drifts_downwind() -> None:
    """When the required airspeed exceeds v_max, ground speed falls short and drifts downwind."""
    cmd = MotionCommand.from_track_speed(0.0, 5.0)  # 5 m/s north (ground)
    wind = WindField.from_met(270.0, 7.0)  # 7 m/s from the west; req airspeed = √74 ≈ 8.6 > 8
    settled = _settle(_at(5.0), cmd, wind)
    ve, vn = velocity_enu(settled)
    assert ve > 0.3  # pushed east — downwind of the westerly (air moves east)
    assert vn < 5.0  # falls short of the commanded northward ground speed
    # steady state: the airspeed vector sits on the v_max circle (clamped)
    air = math.hypot(ve - 7.0, vn)  # ground − wind, wind = (+7, 0)
    assert math.isclose(air, _SMALL.v_max, abs_tol=1e-6)


def test_zero_command_hovers_into_wind() -> None:
    """A zero ground command holds station when V_WS ≤ v_max (position barely moves)."""
    cmd = MotionCommand.from_track_speed(0.0, 0.0)
    wind = WindField.from_met(270.0, 5.0)  # 5 ≤ 8: can null the drift
    start = _at(0.0)
    settled = _settle(start, cmd, wind)
    ve, vn = velocity_enu(settled)
    assert math.hypot(ve, vn) < 1e-6  # ground velocity nulled — hovering into wind
    _, drift = geo.qdrdist(start.lat, start.lon, settled.lat, settled.lon)
    assert drift < 5.0  # essentially stationary (only the initial ramp moved it)


def test_zero_command_is_blown_downwind_above_envelope() -> None:
    """A wind stronger than v_max blows a station-keeping multirotor downwind at V_WS − v_max."""
    cmd = MotionCommand.from_track_speed(0.0, 0.0)
    wind = WindField.from_met(270.0, 10.0)  # 10 > 8: cannot hold
    settled = _settle(_at(0.0), cmd, wind)
    ve, vn = velocity_enu(settled)
    assert math.isclose(vn, 0.0, abs_tol=1e-6)
    assert math.isclose(ve, 10.0 - _SMALL.v_max, abs_tol=1e-6)  # blown east at 2 m/s


def test_no_wind_reproduces_phase4_multirotor() -> None:
    """With NO_WIND the step is byte-identical to omitting the wind argument (Phase 4)."""
    state = AircraftState(id="M", lat=52.0, lon=4.0, trk=20.0, gs=6.0, yaw=80.0)
    cmd = MotionCommand.from_track_speed(110.0, 7.5)
    assert _MR.step(state, cmd, _SMALL, 0.1) == _MR.step(state, cmd, _SMALL, 0.1, NO_WIND)
