"""Waypoint guidance: :class:`WaypointAutopilot` (ADR 0014, Phase 4d).

The vehicle-neutral navigator. It turns a :class:`~opencdarr.mission.Mission` into a **position
setpoint** for the active waypoint — ``MotionCommand(target_position=…)`` — and lets each
airframe's controller (the :class:`~opencdarr.dynamics.base.Dynamics` step) track it (a multirotor
flies straight in and hovers; a fixed-wing tracks the leg line and orbits). Because it emits a
position rather than a pre-computed velocity/course, one autopilot serves both airframes.

It advances through the ``flight_plan`` on a capture radius (flying *through* intermediate
waypoints) and emits a loiter setpoint at the final one (a fixed-wing orbits it; a multirotor
hovers). The active-leg index lives in the threaded
:class:`~opencdarr.autopilot.base.GuidanceMemory`, never on this object (ADR 0014).
"""

from __future__ import annotations

from opencdarr import geo
from opencdarr.autopilot.base import Autopilot, GuidanceMemory
from opencdarr.dynamics import MotionCommand
from opencdarr.mission import Mission, Waypoint
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


class WaypointAutopilot(Autopilot):
    """Navigate a :class:`~opencdarr.mission.Mission` by emitting the active waypoint as a position
    setpoint (+ a cruise airspeed for the fixed-wing longitudinal channel).

    ``cruise_airspeed`` is the fixed-wing equivalent-airspeed target (a multirotor ignores it and
    uses its own position tracker); ``capture_radius`` is the arrival tolerance used for leg
    sequencing (Phase 4d.3).
    """

    def __init__(
        self,
        mission: Mission,
        cruise_airspeed: float = 17.0,
        capture_radius: float = 30.0,
        loiter_radius: float = 80.0,
    ) -> None:
        self._waypoints: tuple[Waypoint, ...] = mission.waypoints()
        self._cruise_airspeed = cruise_airspeed
        self._capture_radius = capture_radius
        self._loiter_radius = loiter_radius

    def step(
        self, state: AircraftState, memory: GuidanceMemory, perf: Performance
    ) -> tuple[MotionCommand, GuidanceMemory]:
        wps = self._waypoints
        if not wps:
            # empty mission: hold heading/speed (nothing to navigate to)
            return MotionCommand.from_track_speed(state.trk, state.gs), memory

        leg = min(memory.leg_index, len(wps) - 1)
        # advance to the next leg once within the capture radius of the active waypoint (fly
        # *through* intermediate waypoints; only the final one is held / loitered). The advanced
        # index rides out in the returned memory — the clonable progress (ADR 0014).
        _, dist = geo.qdrdist(state.lat, state.lon, wps[leg].lat, wps[leg].lon)
        if dist <= self._capture_radius and leg < len(wps) - 1:
            leg += 1
        memory = GuidanceMemory(leg_index=leg)

        wp = wps[leg]
        leg_start = wps[leg - 1] if leg > 0 else None  # the leg line for the fixed-wing L1 tracker
        final = leg == len(wps) - 1  # loiter at the last waypoint (FW orbits; MC hovers)
        command = MotionCommand(
            target_position=(wp.lat, wp.lon),
            target_leg_start=None if leg_start is None else (leg_start.lat, leg_start.lon),
            target_airspeed=self._cruise_airspeed,
            target_loiter_radius=self._loiter_radius if final else None,
        )
        return command, memory
