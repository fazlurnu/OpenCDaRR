"""Constant-cruise guidance: :class:`CruiseAutopilot` (ADR 0011, Phase 4a).

The behaviour-preserving stand-in for the loop's old frozen nominal. Before Phase 4a,
``run_encounter`` computed ``nom_own = Command.from_track_speed(own.trk, own.gs)`` once from the
*true initial* state and held it for the whole encounter — the aircraft could only cruise a fixed
heading, never navigate. :class:`CruiseAutopilot` is exactly that behaviour given a home: it
returns the same command every tick, **independent of the (noisy) state passed in**, which is what
makes the Phase-4a layer split reproduce the pre-refactor IPR bit-for-bit (the old nominal was
frozen from the true initial state, never re-derived from the noisy self-fix).

Real navigation — goto, waypoints, loiter, and reacting to the live state — arrives with the goto
autopilot (Phase 4d); this class deliberately does none of it.
"""

from __future__ import annotations

from opencdarr.autopilot.base import Autopilot, GuidanceMemory
from opencdarr.kinematics import MotionCommand
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


class CruiseAutopilot(Autopilot):
    """Hold a fixed cruise ``(heading, speed)`` — a constant velocity command every tick.

    Constructed from the aircraft's initial cruise track and ground speed; :meth:`step` ignores the
    state, memory, and performance it is passed and returns the constant command (threading the
    memory through untouched), so the encounter's nominal is frozen exactly as the pre-Phase-4a
    loop froze it — the behaviour-preserving default the loop still uses when no mission is given.
    """

    def __init__(self, heading: float, speed: float) -> None:
        # Precompute the constant command once — its value is this autopilot's whole behaviour.
        self._command = MotionCommand.from_track_speed(heading, speed)

    def step(
        self, state: AircraftState, memory: GuidanceMemory, perf: Performance
    ) -> tuple[MotionCommand, GuidanceMemory]:
        """Return the fixed cruise command and the unchanged memory (see class docstring)."""
        return self._command, memory
