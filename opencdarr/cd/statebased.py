"""State-based conflict detection (2D horizontal CPA).

Implements :class:`~opencdarr.cd.base.ConflictDetector` via closest point of approach.
Governing equations: ``vault/derivations/cpa-detection.md``.
"""

from __future__ import annotations

import math

from opencdarr import geo
from opencdarr.cd.base import ConflictDetector
from opencdarr.state import AircraftState

_PARALLEL_EPS = 1e-9  # |v_rel|^2 below this = no approach (parallel, equal speed)


def _relative_enu(
    own: AircraftState, intr: AircraftState
) -> tuple[float, float, float, float]:
    """Relative position and velocity (intr − own) in East–North: (rx, ry, vx, vy)."""
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    rx, ry = dist * math.sin(q), dist * math.cos(q)
    vox = own.gs * math.sin(math.radians(own.trk))
    voy = own.gs * math.cos(math.radians(own.trk))
    vix = intr.gs * math.sin(math.radians(intr.trk))
    viy = intr.gs * math.cos(math.radians(intr.trk))
    return rx, ry, vix - vox, viy - voy


class StateBased(ConflictDetector):
    """Horizontal state-based CPA detection (no tunable parameters)."""

    def detect(
        self, own: AircraftState, intr: AircraftState, rpz: float, t_lookahead: float
    ) -> bool:
        rx, ry, vx, vy = _relative_enu(own, intr)
        v2 = vx * vx + vy * vy
        if v2 < _PARALLEL_EPS:
            return False  # no relative motion

        t_cpa = -(rx * vx + ry * vy) / v2
        dcpa = math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)
        if dcpa >= rpz:
            return False

        tau = math.sqrt(rpz * rpz - dcpa * dcpa) / math.sqrt(v2)
        return t_cpa - tau < t_lookahead and t_cpa + tau > 0.0
