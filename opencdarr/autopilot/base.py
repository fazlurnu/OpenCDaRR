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

from opencdarr.dynamics import MotionCommand
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


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
