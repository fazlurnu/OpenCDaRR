"""Mission ↔ Offboard resume (Phase 4d.6): interrupt a mission, avoid, resume the same leg.

A mission-flying multirotor is interrupted mid-leg by a crossing intruder; the SeparationManager
overlays MVP (the offboard interrupt) and releases to the mission on recovery. The load-bearing
properties:

- the mission **resumes** — the aircraft still reaches its waypoints after the excursion;
- it resumes the **same leg** it was interrupted on (the leg index is never reset by the override);
- the leg index (and pair memory) **survive a clone taken during the override** — the IPS invariant
  that makes the whole "guidance progress is threaded, not hidden" design correct.

The resume is automatic: the autopilot re-plans toward the active waypoint every tick, so releasing
the override simply continues the mission — there is no mode machine.
"""

from __future__ import annotations

import dataclasses

from opencdarr import geo
from opencdarr.autopilot import Autopilot, GuidanceMemory, WaypointAutopilot
from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.kinematics import MotionCommand, Multirotor
from opencdarr.mission import Mission, Waypoint
from opencdarr.performance import M600
from opencdarr.scenario import create_conflict
from opencdarr.separation import INACTIVE, FleetMemory, SeparationManager
from opencdarr.state import AircraftState

_RPZ, _LOOKAHEAD, _DT = 50.0, 20.0, 0.1
_MR = Multirotor()


def _setup() -> tuple[Mission, AircraftState, AircraftState]:
    """OWN flies north past a near waypoint (leg 0) onto a long leg (leg 1); INT crosses leg 1."""
    a = geo.forward(52.0, 4.0, 0.0, 70.0)  # near waypoint — captured within seconds -> onto leg 1
    b = geo.forward(52.0, 4.0, 0.0, 900.0)  # far waypoint, straight ahead (still north)
    mission = Mission(flight_plan=(Waypoint(*a), Waypoint(*b)))
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=18.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=26.0, rpz=_RPZ, side=1)
    return mission, own, intr


def _decide(
    own: AircraftState, intr: AircraftState, ap: Autopilot, gm: GuidanceMemory,
    sep: SeparationManager, mem: FleetMemory,
) -> tuple[MotionCommand, GuidanceMemory, FleetMemory]:
    """One broadcast decision: nominal from the autopilot, overlaid by the separation manager."""
    nom, gm = ap.step(own, gm, M600)
    cmd, mem = sep.step(
        own, [intr], nom, mem, _RPZ, _LOOKAHEAD, StateBased(), MVP(margin=1.15), PastCPA()
    )
    return cmd, gm, mem


def test_mission_interrupts_and_resumes_the_same_leg() -> None:
    mission, own, intr = _setup()
    b = mission.waypoints()[1]
    ap = WaypointAutopilot(mission, capture_radius=30.0)
    gm, sep, mem = GuidanceMemory(), SeparationManager(), INACTIVE
    intr_cmd = MotionCommand.from_track_speed(intr.trk, intr.gs)
    cmd = MotionCommand(target_position=(mission.waypoints()[0].lat, mission.waypoints()[0].lon))

    first_conflict_leg = None
    legs_seen = []
    t, nb = 0.0, 0.0
    while t < 90.0:
        if t + 1e-9 >= nb:
            cmd, gm, mem = _decide(own, intr, ap, gm, sep, mem)
            if mem.resolving and first_conflict_leg is None:
                first_conflict_leg = gm.leg_index  # the leg we were on when the DAA engaged
            nb += 1.0
        legs_seen.append(gm.leg_index)
        own = _MR.step(own, cmd, M600, _DT)
        intr = _MR.step(intr, intr_cmd, M600, _DT)
        _, dfin = geo.qdrdist(own.lat, own.lon, b.lat, b.lon)
        if dfin < 1.0 and own.gs < 0.1:
            break
        t += _DT

    assert first_conflict_leg == 1  # interrupted while flying leg 1 (not leg 0)
    assert min(legs_seen[legs_seen.index(1):]) == 1  # once on leg 1, never reset back to 0
    _, dfin = geo.qdrdist(own.lat, own.lon, b.lat, b.lon)
    assert dfin < 1.0 and own.gs < 0.1  # resumed and reached the final waypoint (hovering)


def _fly(
    mission: Mission, own: AircraftState, intr: AircraftState, gm: GuidanceMemory,
    mem: FleetMemory, intr_cmd: MotionCommand, n: int,
) -> tuple[AircraftState, GuidanceMemory]:
    """Fly forward ``n`` broadcast-cadence seconds from a captured particle (stateless ap/sep)."""
    ap, sep = WaypointAutopilot(mission, capture_radius=30.0), SeparationManager()
    cmd = MotionCommand(target_position=(mission.waypoints()[0].lat, mission.waypoints()[0].lon))
    t, nb = 0.0, 0.0
    while t < float(n):
        if t + 1e-9 >= nb:
            cmd, gm, mem = _decide(own, intr, ap, gm, sep, mem)
            nb += 1.0
        own = _MR.step(own, cmd, M600, _DT)
        intr = _MR.step(intr, intr_cmd, M600, _DT)
        t += _DT
    return own, gm


def test_leg_index_and_pair_memory_survive_a_clone_during_the_override() -> None:
    """Clone the particle (state + guidance memory + pair memory) while resolving; both copies fly
    forward identically — the resume is carried entirely by clonable value state."""
    mission, own, intr = _setup()
    ap = WaypointAutopilot(mission, capture_radius=30.0)
    gm, sep, mem = GuidanceMemory(), SeparationManager(), INACTIVE
    intr_cmd = MotionCommand.from_track_speed(intr.trk, intr.gs)
    cmd = MotionCommand(target_position=(mission.waypoints()[0].lat, mission.waypoints()[0].lon))

    # fly until the DAA is actively overriding the mission
    t, nb = 0.0, 0.0
    while not mem.resolving and t < 90.0:
        if t + 1e-9 >= nb:
            cmd, gm, mem = _decide(own, intr, ap, gm, sep, mem)
            nb += 1.0
        own = _MR.step(own, cmd, M600, _DT)
        intr = _MR.step(intr, intr_cmd, M600, _DT)
        t += _DT
    assert mem.resolving and gm.leg_index >= 1  # genuinely mid-override, mid-plan

    # clone every piece of the particle's clonable state and fly both copies forward
    a = _fly(mission, own, intr, gm, mem, intr_cmd, 40)
    b = _fly(
        mission, dataclasses.replace(own), dataclasses.replace(intr),
        dataclasses.replace(gm), dataclasses.replace(mem), intr_cmd, 40,
    )
    assert a == b  # the clone tracked its parent exactly (same leg, same trajectory)
