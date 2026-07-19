"""State-based conflict detection (2D horizontal CPA).

Implements :class:`~opencdarr.cd.base.ConflictDetector` via closest point of approach.
Governing equations: ``vault/derivations/cpa-detection.md``.
"""

from __future__ import annotations

import math

from opencdarr.cd.base import ConflictDetector
from opencdarr.kinematics import relative_enu
from opencdarr.state import AircraftState

_PARALLEL_EPS = 1e-9  # |v_rel|^2 below this = no approach (parallel, equal speed)


class StateBased(ConflictDetector):
    """Horizontal state-based CPA detection (no tunable parameters)."""

    def detect(
        self, own: AircraftState, intr: AircraftState, rpz: float, t_lookahead: float
    ) -> bool:
        rel = relative_enu(own, intr)
        v2 = rel.vx * rel.vx + rel.vy * rel.vy
        if v2 < _PARALLEL_EPS:
            return False  # no relative motion

        t_cpa = -(rel.rx * rel.vx + rel.ry * rel.vy) / v2
        dcpa = math.hypot(rel.rx + rel.vx * t_cpa, rel.ry + rel.vy * t_cpa)
        if dcpa >= rpz:
            return False

        tau = math.sqrt(rpz * rpz - dcpa * dcpa) / math.sqrt(v2)
        return t_cpa - tau < t_lookahead and t_cpa + tau > 0.0
