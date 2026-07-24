"""FixedWing L1 path-following guidance (Phase 4d.4, ADR 0014).

The fixed-wing tracker consumes a position/leg setpoint and steers with the L1 law: it follows the
leg *line* (nulling cross-track error), not just the endpoint. Checks: a cross-track offset is
driven to zero; a multi-waypoint plan is flown in order; a bare goto (no leg) is pure-pursuit.
"""

from __future__ import annotations

import math

from opencdarr import geo
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot
from opencdarr.dynamics import FixedWing, MotionCommand
from opencdarr.mission import Mission, Waypoint
from opencdarr.performance import SMALL_FIXEDWING as P
from opencdarr.state import AircraftState

_FW = FixedWing()


def _state(lat: float, lon: float, trk: float = 0.0) -> AircraftState:
    return AircraftState(id="F", lat=lat, lon=lon, trk=trk, gs=17.0, yaw=trk, bank=0.0)


def _east_offset(lat0: float, lon0: float, lat: float, lon: float) -> float:
    """Signed east offset [m] of (lat, lon) from the meridian through (lat0, lon0)."""
    qdr, dist = geo.qdrdist(lat0, lon0, lat, lon)
    return dist * math.sin(math.radians(qdr))


def test_l1_nulls_cross_track_offset() -> None:
    """Starting 100 m off a due-north leg, the fixed-wing captures the line and tracks it."""
    a = (52.0, 4.0)
    b = geo.forward(52.0, 4.0, 0.0, 1500.0)  # leg due north
    start = geo.forward(52.0, 4.0, 90.0, 100.0)  # 100 m east of A
    s = _state(*start, trk=0.0)
    cmd = MotionCommand(target_position=(b[0], b[1]), target_leg_start=a, target_airspeed=17.0)
    for _ in range(600):  # 60 s
        s = _FW.step(s, cmd, P, 0.1)
    assert abs(_east_offset(52.0, 4.0, s.lat, s.lon)) < 1.0  # captured the line
    assert abs(((s.trk - 0.0 + 180.0) % 360.0) - 180.0) < 1.0  # tracking along it (due north)


def test_l1_captures_from_either_side() -> None:
    """Cross-track is nulled whether the aircraft starts left or right of the leg."""
    a = (52.0, 4.0)
    b = geo.forward(52.0, 4.0, 0.0, 1500.0)
    for side in (90.0, 270.0):  # east / west of the leg
        start = geo.forward(52.0, 4.0, side, 120.0)
        s = _state(*start, trk=0.0)
        cmd = MotionCommand(target_position=(b[0], b[1]), target_leg_start=a, target_airspeed=17.0)
        for _ in range(700):
            s = _FW.step(s, cmd, P, 0.1)
        assert abs(_east_offset(52.0, 4.0, s.lat, s.lon)) < 1.0


def test_fixedwing_flies_waypoint_plan_in_order() -> None:
    """A fixed-wing (WaypointAutopilot + FixedWing) captures a 3-waypoint plan, legs in order."""
    a = geo.forward(52.0, 4.0, 0.0, 600.0)  # north
    b = geo.forward(a[0], a[1], 90.0, 600.0)  # then east
    c = geo.forward(b[0], b[1], 0.0, 600.0)  # then north again
    mission = Mission(flight_plan=(Waypoint(*a), Waypoint(*b), Waypoint(*c)))
    ap = WaypointAutopilot(mission, cruise_airspeed=17.0, capture_radius=50.0)
    gm = GuidanceMemory()
    s = AircraftState(id="F", lat=52.0, lon=4.0, trk=0.0, gs=17.0, yaw=0.0)
    seen = [gm.leg_index]
    for _ in range(2500):  # 250 s
        cmd, gm = ap.step(s, gm, P)
        s = _FW.step(s, cmd, P, 0.1)
        if gm.leg_index != seen[-1]:
            seen.append(gm.leg_index)
        if gm.leg_index == 2:
            _, dfin = geo.qdrdist(s.lat, s.lon, c[0], c[1])
            if dfin < 50.0:
                break
    assert seen == [0, 1, 2]  # each leg reached, in order


def test_bare_goto_is_pure_pursuit() -> None:
    """With no leg start, the commanded course points straight at the target (pursuit)."""
    from opencdarr.dynamics.fixedwing import _guidance_course

    s = _state(52.0, 4.0)
    target = geo.forward(52.0, 4.0, 30.0, 500.0)  # bearing 30 deg from the aircraft
    course = _guidance_course(s.lat, s.lon, None, (target[0], target[1]))
    assert abs(((course - 30.0 + 180.0) % 360.0) - 180.0) < 0.5  # steers at the target


def test_position_command_is_accepted_not_failed_fast() -> None:
    """A position setpoint is a valid fixed-wing channel (it drives L1) — it must not fail fast."""
    s = _state(52.0, 4.0)
    b = geo.forward(52.0, 4.0, 0.0, 500.0)
    out = _FW.step(s, MotionCommand(target_position=(b[0], b[1]), target_airspeed=17.0), P, 0.1)
    assert out is not s  # stepped without raising
