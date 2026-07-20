"""Conflict-recovery interface — the contribution surface for recovery criteria.

A criterion subclasses :class:`RecoveryCriterion` and implements ``should_resume``: whether
``own`` may stop resolving and resume its nominal navigation now. Directed and per-ownship.

Implementations live beside this file:

- ``pastcpa.py`` → :class:`PastCPA` (resume once diverging and separated) — implemented.
- ``ftr.py`` → :class:`FTR` (Free-To-Revert): resume once reverting to ``own``'s desired
  (nominal) velocity would keep the pair clear — reads ``own.desired`` from the state —
  implemented.
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
