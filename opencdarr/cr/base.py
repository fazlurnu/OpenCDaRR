"""Conflict-resolution interface — the contribution surface for resolution algorithms.

An algorithm subclasses :class:`ConflictResolver` and implements ``resolve``, returning a
:class:`~opencdarr.dynamics.MotionCommand` (with its ``target_velocity`` channel set — a resolver
computes a ground-velocity vector) that flows straight into a :class:`~opencdarr.dynamics.Dynamics`
step. Resolution is **directed and cooperative**: each aircraft resolves from its own perception of
the traffic.

``resolve`` takes the **set** of intruders in conflict (Phase 6, ADR 0004) — the pairwise
``len == 1`` case is what Phases 2–5 used. Crucially, **how the set composes is algorithm-specific,
not a generic "sum"**:

- :class:`~opencdarr.cr.MVP` is a *potential field* — it **sums** the pairwise avoidance vectors
  (``v_own − Σ dv_i``);
- :class:`~opencdarr.cr.VO` is a *feasibility* problem — each intruder forbids a **cone** of
  velocities and the resolution is the velocity outside the **union** of the cones nearest the
  preferred velocity, *not* a sum (a summed velocity can re-enter a cone).

Implementations live beside this file, one per algorithm:

- ``mvp.py`` → :class:`MVP` (Modified Voltage Potential).
- ``vo.py`` → :class:`VO` (Velocity Obstacle, shortest way out of the union).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Sequence

from opencdarr.dynamics import MotionCommand
from opencdarr.state import AircraftState


class ConflictResolver(ABC):
    """Base class every conflict-resolution algorithm implements."""

    @abstractmethod
    def resolve(
        self,
        own: AircraftState,
        intruders: Sequence[AircraftState],
        rpz: float,
        preferred: tuple[float, float] | None = None,
    ) -> MotionCommand:
        """The :class:`MotionCommand` ``own`` follows to resolve against **all** ``intruders``.

        Directed and pure — a function of the given states only. ``intruders`` is the conflicting
        set (``len == 1`` is the pairwise case). ``preferred`` is the ground velocity
        the resolution stays closest to — the aircraft's **nominal** — or ``None`` to default to
        ``own``'s current velocity. Potential-field resolvers (MVP) ignore ``preferred`` and steer
        away from the current velocity; feasibility resolvers (VO) minimise the deviation from
        ``preferred``. An empty set ⇒ hold the preferred/current velocity.
        """
