"""Past-CPA conflict recovery (2D, directed).

Implements :class:`~opencdarr.crr.base.RecoveryCriterion`. Governing equations:
``vault/derivations/pastcpa-recovery.md``.
"""

from __future__ import annotations

import math

from opencdarr import geo
from opencdarr.cd import is_los
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.state import AircraftState

_BOUNCE_ANGLE_DEG = 30.0  # tracks within this are "near-parallel"
_BOUNCE_MARGIN = 1.05  # ... and within this * rpz -> bouncing


def _r_dot_v(own: AircraftState, intr: AircraftState) -> float:
    """r · v with r, v = intr − own (position, velocity), East–North.

    Its sign is opposite ``t_cpa``: ``r·v > 0`` ⟺ ``t_cpa < 0`` ⟺ past CPA (diverging).
    """
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    rx, ry = dist * math.sin(q), dist * math.cos(q)
    vx = intr.gs * math.sin(math.radians(intr.trk)) - own.gs * math.sin(math.radians(own.trk))
    vy = intr.gs * math.cos(math.radians(intr.trk)) - own.gs * math.cos(math.radians(own.trk))
    return rx * vx + ry * vy


def _is_bouncing(own: AircraftState, intr: AircraftState, rpz: float) -> bool:
    """Near-parallel tracks still close to the zone — where resume/re-resolve oscillates."""
    angle_diff = abs(((own.trk - intr.trk + 180.0) % 360.0) - 180.0)
    _, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    return angle_diff < _BOUNCE_ANGLE_DEG and dist < rpz * _BOUNCE_MARGIN


class PastCPA(RecoveryCriterion):
    """Resume once past CPA (diverging) and no longer in loss of separation.

    ``bouncing_guard``: if True, also refuse to resume on a near-parallel conflict still close
    to the zone (prevents resume/re-resolve oscillation) — the old code's behaviour. Off by
    default, so the base criterion is exactly ``past_cpa ∧ ¬is_los``.
    """

    def __init__(self, bouncing_guard: bool = False) -> None:
        self.bouncing_guard = bouncing_guard

    def should_resume(self, own: AircraftState, intr: AircraftState, rpz: float) -> bool:
        if _r_dot_v(own, intr) <= 0.0 or is_los(own, intr, rpz):
            return False  # not yet past CPA, or still in loss of separation
        if self.bouncing_guard and _is_bouncing(own, intr, rpz):
            return False
        return True
