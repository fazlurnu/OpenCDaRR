"""Modified Voltage Potential (MVP) conflict resolution (2D horizontal).

Implements :class:`~opencdarr.cr.base.ConflictResolver`. Governing equations:
``vault/derivations/mvp-resolution.md``.
"""

from __future__ import annotations

import math

from opencdarr.cr.base import ConflictResolver
from opencdarr.dynamics import Command
from opencdarr.kinematics import relative_enu, velocity_enu
from opencdarr.state import AircraftState

_HEADON_EPS = 1e-3  # m: miss below this -> CPA direction set perpendicular (pick a side)
_PARALLEL_EPS = 1e-9  # |v_rel|^2 below this -> no relative motion
_TCPA_EPS = 1e-6  # s: floor on |t_cpa| to avoid division blow-up at CPA


class MVP(ConflictResolver):
    """Modified Voltage Potential resolution.

    ``margin`` (>= 1) enlarges the resolution zone beyond ``rpz`` — the old code's
    ``asas_marh`` (1.05) — so the aircraft clears with a buffer. Default 1.0 = clear exactly
    to ``rpz``. A genuine per-algorithm parameter, which is why this is a class.
    """

    def __init__(self, margin: float = 1.0) -> None:
        self.margin = margin

    def resolve(self, own: AircraftState, intr: AircraftState, rpz: float) -> Command:
        rpz_eff = rpz * self.margin

        rel = relative_enu(own, intr)
        rx, ry, vx, vy, dist = rel.rx, rel.ry, rel.vx, rel.vy, rel.dist
        v2 = vx * vx + vy * vy
        if v2 < _PARALLEL_EPS:
            return Command(hdg=own.trk, spd=own.gs)  # no relative motion: nothing to resolve

        t_cpa = -(rx * vx + ry * vy) / v2
        cx, cy = rx + vx * t_cpa, ry + vy * t_cpa  # relative position at CPA (own -> intr)
        d_miss = math.hypot(cx, cy)
        if d_miss <= _HEADON_EPS:
            d_miss = _HEADON_EPS
            cx, cy = ry / dist * d_miss, -rx / dist * d_miss  # perpendicular to r: pick a side

        # outward gain to make the trajectory tangent to the resolution zone
        if rpz_eff < dist and d_miss < dist:
            erratum = math.cos(math.asin(rpz_eff / dist) - math.asin(d_miss / dist))
            gain = rpz_eff / erratum - d_miss
        else:
            gain = rpz_eff - d_miss

        scale = gain / (max(abs(t_cpa), _TCPA_EPS) * d_miss)
        # steer away: own's new velocity = v_own - dv, with dv = scale * c (points own -> intr)
        vox, voy = velocity_enu(own)
        new_vx = vox - scale * cx
        new_vy = voy - scale * cy
        return Command(
            hdg=math.degrees(math.atan2(new_vx, new_vy)) % 360.0,
            spd=math.hypot(new_vx, new_vy),
        )
