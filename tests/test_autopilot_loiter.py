"""Loiter at the final waypoint (Phase 4d.5): multirotor hovers, fixed-wing orbits.

Same mission, same WaypointAutopilot — the loiter behaviour emerges from the airframe's tracker: a
multirotor holds position (hover); a fixed-wing, which cannot stop, flies a min-radius orbit around
the loiter centre.
"""

from __future__ import annotations

from opencdarr import geo
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot
from opencdarr.dynamics import FixedWing, Multirotor
from opencdarr.mission import Mission
from opencdarr.performance import M600
from opencdarr.performance import SMALL_FIXEDWING as PF
from opencdarr.state import AircraftState

_CENTER = geo.forward(52.0, 4.0, 0.0, 400.0)  # loiter point 400 m north of the start


def test_multirotor_hovers_at_the_loiter_point() -> None:
    """A multirotor loiter is a hover: it reaches the point and stops (radius ignored)."""
    ap = WaypointAutopilot(Mission(goto=_CENTER), loiter_radius=80.0)
    gm = GuidanceMemory()
    mr = Multirotor()
    s = AircraftState(id="M", lat=52.0, lon=4.0, trk=0.0, gs=0.0)
    for _ in range(700):
        cmd, gm = ap.step(s, gm, M600)
        s = mr.step(s, cmd, M600, 0.1)
    _, dist = geo.qdrdist(s.lat, s.lon, _CENTER[0], _CENTER[1])
    assert dist < 1.0 and s.gs < 0.1  # hovering on the point


def test_fixedwing_orbits_the_loiter_point() -> None:
    """A fixed-wing loiter is a min-radius orbit: it settles onto a circle at the loiter radius,
    keeps flying (cannot stop), and completes at least one full loop."""
    radius = 80.0
    ap = WaypointAutopilot(Mission(goto=_CENTER), cruise_airspeed=17.0, loiter_radius=radius)
    gm = GuidanceMemory()
    fw = FixedWing()
    s = AircraftState(id="F", lat=52.0, lon=4.0, trk=0.0, gs=17.0, yaw=0.0, bank=0.0)
    dists, swept, prev_trk = [], 0.0, s.trk
    for i in range(1500):  # ~2 orbit periods after capture
        cmd, gm = ap.step(s, gm, PF)
        s = fw.step(s, cmd, PF, 0.1)
        if i > 600:  # settled into the orbit
            _, d = geo.qdrdist(s.lat, s.lon, _CENTER[0], _CENTER[1])
            dists.append(d)
            swept += abs(((s.trk - prev_trk + 180.0) % 360.0) - 180.0)
        prev_trk = s.trk
    assert 0.85 * radius < min(dists) and max(dists) < 1.2 * radius  # holds the circle
    assert max(dists) - min(dists) < 10.0  # a steady orbit, not spiralling
    assert s.gs > 15.0  # never stopped (airspeed held)
    assert swept > 350.0  # completed at least one full loop
