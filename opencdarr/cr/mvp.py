"""Modified Voltage Potential (MVP) conflict resolution (2D horizontal).

Implements :class:`~opencdarr.cr.base.ConflictResolver`. Governing equations:
``vault/derivations/mvp-resolution.md``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from opencdarr.cr.base import ConflictResolver
from opencdarr.kinematics import MotionCommand
from opencdarr.relative import relative_enu, velocity_enu
from opencdarr.state import AircraftState

# m: floor on the predicted miss used to *bias* the resolution direction. Below it the actual CPA
# offset is noise-dominated, so its direction is ill-conditioned (a near-head-on then drags into a
# livelock, ``vault/observations/headon-threshold.md``); flooring it picks a clean perpendicular
# side. BlueSky uses 10 m here; this value is likely **separation-minima (rpz) dependent** and
# should be generalised to a fraction of rpz later, not left a bare constant.
_BIAS_EPS = 0.1
_PARALLEL_EPS = 1e-9  # |v_rel|^2 below this -> no relative motion
_TCPA_EPS = 1e-6  # s: floor on |t_cpa| to avoid division blow-up at CPA


def _pairwise_dv(own: AircraftState, intr: AircraftState, rpz_eff: float) -> tuple[float, float]:
    """The velocity change ``dv`` to **subtract** from ``own``'s velocity to clear this one pair.

    The MVP avoidance vector for a single directed pair (``own`` steers away from the CPA point):
    the ownship's new velocity is ``v_own − dv``. ``(0, 0)`` when there is no relative motion
    (nothing to resolve for this pair). Isolated as a pairwise primitive so the N-aircraft resolve
    is the **sum** of these (ADR 0004 / Phase 6): MVP is a potential-field method, so the pairwise
    avoidance vectors superpose.
    """
    rel = relative_enu(own, intr)
    rx, ry, vx, vy, dist = rel.rx, rel.ry, rel.vx, rel.vy, rel.dist
    v2 = vx * vx + vy * vy
    if v2 < _PARALLEL_EPS:
        return 0.0, 0.0  # no relative motion: this pair contributes nothing

    t_cpa = -(rx * vx + ry * vy) / v2
    cx, cy = rx + vx * t_cpa, ry + vy * t_cpa  # relative position at CPA (own -> intr)
    d_miss = math.hypot(cx, cy)
    if d_miss <= _BIAS_EPS:
        d_miss = _BIAS_EPS
        cx, cy = ry / dist * d_miss, -rx / dist * d_miss  # perpendicular to r: pick a side

    # outward gain to make the trajectory tangent to the resolution zone
    if rpz_eff < dist and d_miss < dist:
        erratum = math.cos(math.asin(rpz_eff / dist) - math.asin(d_miss / dist))
        gain = rpz_eff / erratum - d_miss
    else:
        gain = rpz_eff - d_miss

    scale = gain / (max(abs(t_cpa), _TCPA_EPS) * d_miss)
    return scale * cx, scale * cy  # dv points own -> intr; own steers away by subtracting it


class MVP(ConflictResolver):
    """Modified Voltage Potential resolution — a potential-field method (sums over conflicts).

    ``margin`` (>= 1) enlarges the resolution zone beyond ``rpz`` — the old code's
    ``asas_marh`` (1.05) — so the aircraft clears with a buffer. Default 1.0 = clear exactly
    to ``rpz``. A genuine per-algorithm parameter, which is why this is a class.

    Multi-aircraft (Phase 6): the avoidance vector is the **sum** of the pairwise ``dv``s over the
    conflicting set (``v_own − Σ dv_i``). The pairwise ``len == 1`` case is byte-identical to
    Phases 2–5. ``preferred`` is ignored — MVP steers away from the *current* velocity.
    """

    def __init__(self, margin: float = 1.0) -> None:
        self.margin = margin

    def resolve(
        self,
        own: AircraftState,
        intruders: Sequence[AircraftState],
        rpz: float,
        preferred: tuple[float, float] | None = None,
    ) -> MotionCommand:
        rpz_eff = rpz * self.margin
        vox, voy = velocity_enu(own)
        dv_e = dv_n = 0.0
        for intr in intruders:
            d_e, d_n = _pairwise_dv(own, intr, rpz_eff)
            dv_e += d_e
            dv_n += d_n
        # own's new velocity = current − Σ dv (an empty/no-motion set holds the current velocity)
        return MotionCommand(target_velocity=(vox - dv_e, voy - dv_n))
