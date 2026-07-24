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

from opencdarr.dynamics import MotionCommand
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


class Autopilot(ABC):
    """Base class every guidance strategy implements — the mission-executor layer.

    Passed into :func:`~opencdarr.loop.run_encounter` per aircraft; ``step`` runs at the broadcast
    decision cadence and produces the aircraft's **nominal** motion command, which the
    :class:`~opencdarr.separation.SeparationManager` may then override for safety.
    """

    @abstractmethod
    def step(self, state: AircraftState, perf: Performance) -> MotionCommand:
        """Return the nominal :class:`MotionCommand` for ``state`` under this airframe's ``perf``.

        Pure — a function of the given arguments (and the autopilot's own immutable configuration)
        only; no global or module state is read or written, so a clone (IPS particle) evolved
        through this call stays independent of its source.
        """
