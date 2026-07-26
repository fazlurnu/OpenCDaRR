"""``nominal_velocity`` — the autopilot's live nominal read as a velocity, for FTR to revert to.

The load-bearing properties: a velocity command (CruiseAutopilot) passes through unchanged, so
frozen-cruise runs stay byte-identical; a position command (WaypointAutopilot) becomes *head at
the active waypoint at the cruise airspeed*, independent of the current (possibly noisy) velocity.
"""

from __future__ import annotations

import math

from opencdarr import geo
from opencdarr.autopilot import nominal_velocity
from opencdarr.dynamics import MotionCommand
from opencdarr.state import AircraftState, DesiredVelocity


def _state(**over: float) -> AircraftState:
    base = dict(id="OWN", lat=52.0, lon=4.0, trk=90.0, gs=10.0)
    base.update(over)
    return AircraftState(**base)  # type: ignore[arg-type]


def test_velocity_command_passes_through() -> None:
    """A cruise (velocity) command yields exactly its velocity: the frozen-cruise byte-identity."""
    cmd = MotionCommand.from_track_speed(30.0, 12.0)
    d = nominal_velocity(cmd, _state())
    exp = DesiredVelocity.from_track_speed(30.0, 12.0)
    assert math.isclose(d.v_east, exp.v_east) and math.isclose(d.v_north, exp.v_north)


def test_position_command_aims_at_waypoint_at_cruise() -> None:
    """A goto/position command heads at the waypoint at the cruise airspeed, ignoring the state's
    current velocity (which may be a noisy or mid-avoidance one)."""
    target = geo.forward(52.0, 4.0, 0.0, 500.0)  # 500 m due north
    d = nominal_velocity(MotionCommand(target_position=target, target_airspeed=15.0),
                         _state(trk=250.0, gs=8.0))
    assert math.isclose(d.gs, 15.0, abs_tol=1e-6)  # cruise, not the current 8 m/s
    assert min(d.trk % 360.0, 360.0 - d.trk % 360.0) < 1.0  # ~ due north, not the current 250


def test_position_command_without_airspeed_uses_ground_speed() -> None:
    target = geo.forward(52.0, 4.0, 90.0, 500.0)  # due east
    d = nominal_velocity(MotionCommand(target_position=target), _state(gs=11.0))
    assert math.isclose(d.gs, 11.0, abs_tol=1e-6)


def test_no_channel_holds_current_velocity() -> None:
    d = nominal_velocity(MotionCommand(), _state(trk=123.0, gs=9.0))
    exp = DesiredVelocity.from_track_speed(123.0, 9.0)
    assert math.isclose(d.v_east, exp.v_east) and math.isclose(d.v_north, exp.v_north)
