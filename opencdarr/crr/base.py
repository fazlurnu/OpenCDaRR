"""Conflict-recovery interface — the contribution surface for recovery criteria.

A criterion subclasses :class:`RecoveryCriterion` and implements ``should_resume``: whether
``own`` may stop resolving and resume its nominal navigation now. Directed and per-ownship.

Implementations live beside this file:

- ``pastcpa.py`` → :class:`PastCPA` (resume once diverging and separated) — implemented.
- e.g. ``ftr.py`` → a Free-Track-Recovery criterion that re-checks whether resuming to
  ``own``'s nominal navigation would re-trigger a conflict (holds a detector + lookahead on
  the instance) — *example, not implemented*.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from opencdarr.state import AircraftState


class RecoveryCriterion(ABC):
    """Base class every conflict-recovery criterion implements."""

    @abstractmethod
    def should_resume(self, own: AircraftState, intr: AircraftState, rpz: float) -> bool:
        """Whether ``own`` may resume its nominal navigation (stop resolving) now.

        Directed and pure. Criteria that need the aircraft's intended navigation read it from
        ``own`` (a state field, added when the first such criterion lands); criteria that
        re-check for conflict hold their own detector + lookahead on the instance — hence
        neither appears in this signature.
        """
