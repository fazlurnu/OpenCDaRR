"""Conflict-detection interface — the contribution surface for detection algorithms.

An algorithm subclasses :class:`ConflictDetector` and implements ``detect``; its parameters
live on the instance. Detection is **directed**: ``detect`` predicts whether ``own`` loses
separation with its (perceived) ``intr`` within the lookahead. Loss of separation *now* is a
separate, algorithm-independent fact — see :func:`is_los`.

Implementations live beside this file, one per algorithm — a new algorithm adds a file, not a
fork of the core (``design_brief.md``: the interface is the contribution surface):

- ``statebased.py`` → :class:`StateBased` (horizontal CPA) — implemented.
- e.g. ``probconfdetect.py`` → a probabilistic detector — *example, not implemented*.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from opencdarr import geo
from opencdarr.state import AircraftState


class ConflictDetector(ABC):
    """Base class every conflict-detection algorithm implements."""

    @abstractmethod
    def detect(
        self, own: AircraftState, intr: AircraftState, rpz: float, t_lookahead: float
    ) -> bool:
        """Whether ``own`` is predicted to lose separation with ``intr`` within ``t_lookahead``.

        Directed and pure — a function of the given states only. Returns the boolean verdict
        only; algorithm-specific diagnostics (dcpa, probabilities, …) are not part of the
        general contract.
        """


def is_los(own: AircraftState, intr: AircraftState, rpz: float) -> bool:
    """Current loss of separation (``dist < rpz``) — independent of any detection algorithm."""
    _, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    return dist < rpz
