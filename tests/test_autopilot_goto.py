"""Goto guidance (Phase 4d.2): a multirotor flies to a point and settles into a hover.

Drives the navigator (:class:`WaypointAutopilot`) + the controller (:class:`Multirotor`) directly —
the autopilot emits a position setpoint, the airframe tracks it. The load-bearing checks: it
*arrives* (range → 0) and it *settles* (ground speed → 0, i.e. hover), on a direct path.
"""

from __future__ import annotations

from opencdarr import geo
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot
from opencdarr.dynamics import Multirotor
from opencdarr.mission import Mission
from opencdarr.performance import M600
from opencdarr.state import AircraftState

_MR = Multirotor()


def _fly(start: AircraftState, mission: Mission, n: int, dt: float = 0.1) -> AircraftState:
    ap = WaypointAutopilot(mission)
    gm = GuidanceMemory()
    s = start
    for _ in range(n):
        cmd, gm = ap.step(s, gm, M600)
        s = _MR.step(s, cmd, M600, dt)
    return s


def test_multirotor_reaches_goto_and_hovers() -> None:
    """From rest, fly to a point 200 m NE and settle into a hover on it."""
    start = AircraftState(id="M", lat=52.0, lon=4.0, trk=0.0, gs=0.0)
    tgt_lat, tgt_lon = geo.forward(52.0, 4.0, 45.0, 200.0)
    s = _fly(start, Mission(goto=(tgt_lat, tgt_lon)), n=600)
    _, dist = geo.qdrdist(s.lat, s.lon, tgt_lat, tgt_lon)
    assert dist < 1.0  # arrived at the point
    assert s.gs < 0.1  # settled into a hover


def test_goto_path_is_direct() -> None:
    """The multirotor heads straight at the point — its track stays on the bearing to it."""
    start = AircraftState(id="M", lat=52.0, lon=4.0, trk=0.0, gs=0.0)
    tgt_lat, tgt_lon = geo.forward(52.0, 4.0, 90.0, 150.0)  # due east
    ap = WaypointAutopilot(Mission(goto=(tgt_lat, tgt_lon)))
    gm = GuidanceMemory()
    s = start
    for _ in range(150):  # 15 s, still en route
        cmd, gm = ap.step(s, gm, M600)
        s = _MR.step(s, cmd, M600, 0.1)
        _, dist = geo.qdrdist(s.lat, s.lon, tgt_lat, tgt_lon)
        if s.gs > 1.0 and dist > 10.0:  # en route (not terminal maneuvering), track = bearing
            bearing, _ = geo.qdrdist(s.lat, s.lon, tgt_lat, tgt_lon)
            assert abs(((s.trk - bearing + 180.0) % 360.0) - 180.0) < 1.0


def test_goto_reached_from_a_moving_start() -> None:
    """Arrives and hovers even when starting at speed in the wrong direction (decelerates in)."""
    start = AircraftState(id="M", lat=52.0, lon=4.0, trk=270.0, gs=12.0)  # flying west
    tgt_lat, tgt_lon = geo.forward(52.0, 4.0, 90.0, 120.0)
    s = _fly(start, Mission(goto=(tgt_lat, tgt_lon)), n=600)
    _, dist = geo.qdrdist(s.lat, s.lon, tgt_lat, tgt_lon)
    assert dist < 1.0
    assert s.gs < 0.1


def test_goto_captures_without_flying_away() -> None:
    """Once it first reaches the point it stays captured — it doesn't overshoot and orbit off."""
    start = AircraftState(id="M", lat=52.0, lon=4.0, trk=0.0, gs=0.0)
    tgt_lat, tgt_lon = geo.forward(52.0, 4.0, 30.0, 100.0)
    ap = WaypointAutopilot(Mission(goto=(tgt_lat, tgt_lon)))
    gm = GuidanceMemory()
    s = start
    captured = False
    for _ in range(400):
        cmd, gm = ap.step(s, gm, M600)
        s = _MR.step(s, cmd, M600, 0.1)
        _, dist = geo.qdrdist(s.lat, s.lon, tgt_lat, tgt_lon)
        captured = captured or dist < 1.0
        if captured:
            assert dist < 2.0  # never flies more than ~a body-length past the point once there

