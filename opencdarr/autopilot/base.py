"""Guidance fundamentals — the :class:`Autopilot` contribution surface (ADR 0011, Phase 4a).

An autopilot answers *how should the aircraft achieve its mission?* — it turns intent into an
immediate :class:`~opencdarr.dynamics.MotionCommand` at the decision cadence (1 Hz), vehicle-aware.
It is the setpoint **producer**; the low-level setpoint **tracker** (PX4's position / attitude /
rate controllers) lives inside :class:`~opencdarr.dynamics.base.Dynamics`. This mirrors every
other model family in the library (:class:`~opencdarr.cd.base.ConflictDetector`,
:class:`~opencdarr.cr.base.ConflictResolver`, ...): a new guidance strategy or vehicle class adds
a file beside this one, not a fork of the loop (``design_brief.md``: the interface is the
contribution surface; [[0007-dynamics-as-pluggable-interface]] applied to guidance).

Implementations live beside this file:

- :class:`~opencdarr.autopilot.cruise.CruiseAutopilot` — a constant cruise command, the
  behaviour-preserving stand-in for the loop's old frozen nominal (``cruise.py``).
- a goto / waypoint / loiter autopilot — *future*, Phase 4d.

No-hidden-state obligation (ADR 0011 §5): any guidance progress an autopilot accumulates (e.g. the
active-waypoint index, Phase 4d) must live in **clonable value state**, threaded through, never as
a mutable attribute on the autopilot object — the same invariant ``PairMemory`` obeys, for the
same reason (an IPS clone that lost it would fly differently from its parent).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from opencdarr import geo
from opencdarr.dynamics import MotionCommand
from opencdarr.performance import Performance
from opencdarr.state import AircraftState, DesiredVelocity


@dataclass(frozen=True)
class GuidanceMemory:
    """An autopilot's threaded progress through a mission — the clonable guidance state (ADR 0014).

    Currently just the active-leg index into the mission's ``flight_plan``. Held as a value passed
    **in** to :meth:`Autopilot.step` and returned **out**, never as a mutable attribute on the
    autopilot object — the same no-hidden-state invariant :class:`~opencdarr.separation.PairMemory`
    obeys, for the same reason: an IPS clone taken mid-plan must resume the *same* leg, so the
    index has to travel inside the clonable particle. A stateless autopilot (``CruiseAutopilot``)
    simply threads it through untouched.
    """

    leg_index: int = 0


class Autopilot(ABC):
    """Base class every guidance strategy implements — the mission-executor (navigator) layer.

    Passed into :func:`~opencdarr.loop.run_encounter` per aircraft; ``step`` runs at the broadcast
    decision cadence and produces the aircraft's **nominal** command (a position/leg setpoint the
    airframe then tracks), which the :class:`~opencdarr.separation.SeparationManager` may
    override for safety. Guidance *progress* rides in the threaded :class:`GuidanceMemory`, not on
    the object (ADR 0011 §5 / ADR 0014).
    """

    @abstractmethod
    def step(
        self, state: AircraftState, memory: GuidanceMemory, perf: Performance
    ) -> tuple[MotionCommand, GuidanceMemory]:
        """The nominal :class:`MotionCommand` and updated :class:`GuidanceMemory` for ``state``.

        Pure — a function of the given arguments (and the autopilot's own immutable configuration)
        only; no global or module state is read or written, so a clone (IPS particle) evolved
        through this call stays independent of its source. The returned memory carries any guidance
        progress (e.g. an advanced leg index) forward to the next tick and into a clone.
        """

    def goal(self) -> tuple[float, float] | None:
        """The aircraft's final destination as ``(lat, lon)``, or ``None`` if it has no fixed goal.

        Read by :meth:`~opencdarr.fleet.FleetEnv.is_terminal` for the optional stop-at-waypoint
        condition (``run_fleet(..., stop_within=...)``). A guidance strategy with a destination
        (:class:`~opencdarr.autopilot.WaypointAutopilot`) overrides this; a heading/velocity holder
        (:class:`~opencdarr.autopilot.CruiseAutopilot`) leaves it ``None`` and never triggers the
        stop. Default ``None`` keeps every existing autopilot unchanged.
        """
        return None


def nominal_velocity(command: MotionCommand, state: AircraftState) -> DesiredVelocity:
    """The velocity the aircraft *intends* to fly — its current nominal command read as a ground
    velocity, for intent-based recovery (:class:`~opencdarr.crr.FTR`) to test reverting to.

    This is what makes FTR's revert-check track a *mission*: ``desired`` is the velocity the loop
    stamps on the state each tick from the live nominal, not a value frozen at ``t = 0``. A
    velocity command (:class:`~opencdarr.autopilot.CruiseAutopilot`) is returned unchanged, so a
    frozen-cruise run is byte-identical to before this existed. A position command
    (:class:`~opencdarr.autopilot.WaypointAutopilot`) becomes *head at the active waypoint, at the
    cruise airspeed*: exact for a ``goto`` / pure-pursuit leg, and a goal-direction approximation
    for an L1 leg (the return-to-the-leg-line component is deferred — the same spirit as the
    fixed-wing course projection, ADR 0013 §4). With no channel it holds the current velocity.
    """
    if command.target_velocity is not None:
        ve, vn = command.target_velocity
        return DesiredVelocity(v_east=ve, v_north=vn)
    if command.target_position is not None:
        qdr, _ = geo.qdrdist(state.lat, state.lon,
                             command.target_position[0], command.target_position[1])
        speed = state.gs if command.target_airspeed is None else command.target_airspeed
        return DesiredVelocity.from_track_speed(qdr, speed)
    return DesiredVelocity.from_track_speed(state.trk, state.gs)
