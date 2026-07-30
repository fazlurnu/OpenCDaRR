"""Waypoint sequencing (Phase 4d.3): a multirotor flies a 3-waypoint plan in order.

Two load-bearing checks:
- the active-leg index advances 0 → 1 → 2 as the vehicle captures each waypoint, in order, and it
  hovers at the final one;
- the leg index is **clonable value state** (lives in the threaded ``GuidanceMemory``, not on the
  autopilot object): a clone taken mid-plan continues from the *same* leg — the IPS invariant.
"""

from __future__ import annotations

import dataclasses

from opencdarr import geo
from opencdarr.autopilot import GuidanceMemory, WaypointAutopilot
from opencdarr.kinematics import Multirotor
from opencdarr.mission import Mission, Waypoint
from opencdarr.performance import M600
from opencdarr.state import AircraftState

_MR = Multirotor()


def _plan() -> Mission:
    a = geo.forward(52.0, 4.0, 0.0, 300.0)  # 300 m north
    b = geo.forward(a[0], a[1], 90.0, 300.0)  # then 300 m east
    c = geo.forward(b[0], b[1], 180.0, 300.0)  # then 300 m south
    return Mission(flight_plan=(Waypoint(*a), Waypoint(*b), Waypoint(*c)))


def _start() -> AircraftState:
    return AircraftState(id="M", lat=52.0, lon=4.0, trk=0.0, gs=0.0)


def test_three_waypoint_plan_flown_in_order() -> None:
    """Legs advance 0->1->2 monotonically; the vehicle ends hovering at the final waypoint."""
    mission = _plan()
    wps = mission.waypoints()
    ap = WaypointAutopilot(mission, capture_radius=30.0)
    gm = GuidanceMemory()
    s = _start()
    seen = [gm.leg_index]
    for _ in range(1200):  # 120 s
        cmd, gm = ap.step(s, gm, M600)
        s = _MR.step(s, cmd, M600, 0.1)
        if gm.leg_index != seen[-1]:
            seen.append(gm.leg_index)
    assert seen == [0, 1, 2]  # each leg reached, in order, none skipped
    _, dist = geo.qdrdist(s.lat, s.lon, wps[2].lat, wps[2].lon)
    assert dist < 1.0 and s.gs < 0.1  # hovering at the final waypoint


def test_leg_index_lives_in_threaded_memory_not_the_object() -> None:
    """A fresh autopilot given the same (state, memory) emits the same command — the leg is in the
    memory, not hidden on the autopilot. This is what makes a mid-plan clone resume correctly."""
    mission = _plan()
    ap = WaypointAutopilot(mission, capture_radius=30.0)
    gm = GuidanceMemory()
    s = _start()
    # fly until the plan has advanced past leg 0
    while gm.leg_index == 0:
        cmd, gm = ap.step(s, gm, M600)
        s = _MR.step(s, cmd, M600, 0.1)
    assert gm.leg_index >= 1  # genuinely mid-plan

    # a brand-new autopilot fed the captured (state, memory) targets the SAME active leg
    fresh = WaypointAutopilot(mission, capture_radius=30.0)
    cmd_a, mem_a = ap.step(s, gm, M600)
    cmd_b, mem_b = fresh.step(s, gm, M600)
    assert cmd_a == cmd_b and mem_a == mem_b
    # and it is leg >=1's waypoint, not leg 0's (the memory drove it, not a restart)
    wps = mission.waypoints()
    assert cmd_b.target_position == (wps[gm.leg_index].lat, wps[gm.leg_index].lon)


def test_clone_mid_plan_continues_from_same_leg() -> None:
    """Cloning the (state, memory) mid-plan and flying both forward gives identical results."""
    mission = _plan()
    ap = WaypointAutopilot(mission, capture_radius=30.0)
    gm = GuidanceMemory()
    s = _start()
    while gm.leg_index < 1:  # advance into the plan
        cmd, gm = ap.step(s, gm, M600)
        s = _MR.step(s, cmd, M600, 0.1)

    # clone the particle (state + guidance memory) and fly both 300 steps
    s_a, gm_a = s, gm
    s_b, gm_b = dataclasses.replace(s), dataclasses.replace(gm)
    for _ in range(300):
        c_a, gm_a = ap.step(s_a, gm_a, M600)
        s_a = _MR.step(s_a, c_a, M600, 0.1)
        c_b, gm_b = ap.step(s_b, gm_b, M600)
        s_b = _MR.step(s_b, c_b, M600, 0.1)
    assert gm_a == gm_b and s_a == s_b  # clone tracked its parent exactly (same leg throughout)
