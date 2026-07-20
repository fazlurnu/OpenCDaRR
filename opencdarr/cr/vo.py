"""Velocity-Obstacle (VO) conflict resolution — shortest way out (2D horizontal).

Implements :class:`~opencdarr.cr.base.ConflictResolver`. The **velocity obstacle** is the set of
ownship velocities that lead to a future incursion of the protected zone: a cone in velocity space
with apex at the intruder's velocity and edges parallel to the two tangent lines from the ownship
to the ``rpz``-circle around the intruder. The *shortest way out* picks the new velocity as the
point on the **nearer cone edge** closest to the current velocity — the minimal velocity change
that leaves the obstacle. Re-derived from ``CDaRR_git/sim_models/cr_vo.py`` (method 0, "opt"),
analytic instead of shapely; our ENU (East, North) convention, no wind, 2D.
"""

from __future__ import annotations

import math

from opencdarr.cr.base import ConflictResolver
from opencdarr.dynamics import Command
from opencdarr.kinematics import relative_enu, velocity_enu
from opencdarr.state import AircraftState

_SEG_EPS = 1e-12  # m^2/s^2: degenerate (zero-length) cone edge


def _nearest_on_segment(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> tuple[float, float]:
    """Point on segment A→B closest to P (clamped to the endpoints)."""
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    if ab2 <= _SEG_EPS:
        return ax, ay
    t = ((px - ax) * abx + (py - ay) * aby) / ab2
    t = max(0.0, min(1.0, t))
    return ax + t * abx, ay + t * aby


class VO(ConflictResolver):
    """Velocity-Obstacle resolution (shortest way out).

    ``margin`` (>= 1) enlarges the protected zone the cone is built around — the old code's
    ``asas_marh`` (1.05). A genuine per-algorithm parameter, mirroring :class:`~opencdarr.cr.MVP`.
    """

    def __init__(self, margin: float = 1.0) -> None:
        self.margin = margin

    def resolve(self, own: AircraftState, intr: AircraftState, rpz: float) -> Command:
        rpz_eff = rpz * self.margin

        rel = relative_enu(own, intr)  # rx,ry = intr − own position; vx,vy unused here
        dist = rel.dist
        vox, voy = velocity_enu(own)
        vix, viy = velocity_enu(intr)
        if dist <= rpz_eff:
            return Command(hdg=own.trk, spd=own.gs)  # already inside: no cone, hold velocity

        # tangent geometry of the rpz_eff circle around the intruder, seen from the ownship
        bearing = math.atan2(rel.rx, rel.ry)  # qdr to intruder (atan2 of East, North)
        half_angle = math.asin(rpz_eff / dist)
        reach = math.sqrt(dist * dist - rpz_eff * rpz_eff)  # ownship → tangent-point distance

        # each cone edge is the segment from the apex (intruder velocity) to (tangent point +
        # intruder velocity); the new velocity is the nearer edge's closest point to v_own
        best = (vox, voy)
        best_d2 = math.inf
        for sign in (-1.0, 1.0):
            ang = bearing + sign * half_angle
            tp_e = reach * math.sin(ang)  # tangent point relative to ownship, East
            tp_n = reach * math.cos(ang)  # ... North
            qe, qn = _nearest_on_segment(vox, voy, vix, viy, vix + tp_e, viy + tp_n)
            d2 = (vox - qe) ** 2 + (voy - qn) ** 2
            if d2 < best_d2:
                best_d2, best = d2, (qe, qn)

        new_e, new_n = best
        return Command(
            hdg=math.degrees(math.atan2(new_e, new_n)) % 360.0,
            spd=math.hypot(new_e, new_n),
        )
