"""Conflict-resolution interface — the contribution surface for resolution algorithms.

An algorithm subclasses :class:`ConflictResolver` and implements ``resolve``, returning a
:class:`~opencdarr.dynamics.Command` (target track + speed) that flows straight into
``step_dynamics``. Resolution is **directed and cooperative**: each aircraft resolves from its
own perception of the intruder.

Implementations live beside this file, one per algorithm:

- ``mvp.py`` → :class:`MVP` (Modified Voltage Potential) — implemented next.
- e.g. ``vo.py`` → a Velocity-Obstacle resolver — *example, not implemented*.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from opencdarr.dynamics import Command
from opencdarr.state import AircraftState


class ConflictResolver(ABC):
    """Base class every conflict-resolution algorithm implements."""

    @abstractmethod
    def resolve(self, own: AircraftState, intr: AircraftState, rpz: float) -> Command:
        """Return the :class:`Command` ``own`` should follow to resolve against ``intr``.

        Directed and pure — a function of the given states only.
        """
