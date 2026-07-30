"""Velocity-Obstacle (VO) conflict resolution — shortest way out of the union (2D horizontal).

Implements :class:`~opencdarr.cr.base.ConflictResolver`. The **velocity obstacle** of one intruder
is the set of ownship velocities that lead to a future incursion of the protected zone: a **cone**
in velocity space with apex at the intruder's velocity, axis along the bearing to the intruder, and
half-angle ``asin(rpz_eff / dist)``. A velocity is unsafe for that pair iff it lies in the cone.

**Single intruder** (Phases 2–5): the *shortest way out* is the velocity on the nearer cone edge
closest to the preferred velocity — the minimal change that leaves the cone.

**Multiple intruders** (Phase 6, ADR 0004): the resolution is the velocity outside the **union** of
all the cones, closest to the preferred velocity — **not** a sum of pairwise resolutions (a summed
velocity can land back inside a cone; the forbidden region is a non-convex union, not a
superposable force). Computed by an **analytic candidate search** (decided in the Phase-6 plan):
the nearest exterior point of a union of cones lies on the union's boundary, so the candidates are
the projections of the preferred velocity onto each cone edge **plus the pairwise intersections of
edges from different cones** (the union's vertices); keep those outside *all* cones, nearest the
preferred velocity wins. An over-constrained fleet (no reachable exterior velocity) falls
back to the least-penetration candidate. At one intruder this reduces to the single-cone shortest
way out.

The **preferred** velocity — the point the resolution stays closest to — defaults to the
**current** velocity (the classic shortest way out, the pre-Phase-6 single-VO behaviour, what the
:class:`~opencdarr.separation.SeparationManager` passes). Biasing it toward the *nominal* was tried
and destabilised the resolver — greedy nearest-to-nominal cone projection snaps back to the nominal
when it momentarily looks feasible and re-enters the conflict, losing separation; returning to the
nominal is the recovery layer's job, not the resolver's. The ``preferred`` channel is kept for a
future stable (ORCA-style, reciprocal half-plane) resolver. Re-derived from
``CDaRR_git/sim_models/cr_vo.py``; our ENU (East, North) convention, no wind, 2D.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from opencdarr.cr.base import ConflictResolver
from opencdarr.kinematics import MotionCommand
from opencdarr.relative import relative_enu, velocity_enu
from opencdarr.state import AircraftState

_ANG_EPS = 1e-9  # rad: a velocity within this of a cone edge counts as *outside* (on the boundary)
_DENOM_EPS = 1e-12  # near-parallel edges: no ray intersection
_VEC_EPS = 1e-12  # m/s: a ~zero relative velocity is not closing


class _Cone:
    """One intruder's velocity obstacle: apex (intruder velocity), axis bearing, half-angle."""

    __slots__ = ("apex_e", "apex_n", "bearing", "half")

    def __init__(self, apex_e: float, apex_n: float, bearing: float, half: float) -> None:
        self.apex_e, self.apex_n, self.bearing, self.half = apex_e, apex_n, bearing, half

    def contains(self, ve: float, vn: float) -> bool:
        """Is velocity ``(ve, vn)`` strictly inside the cone (unsafe for this pair)?"""
        de, dn = ve - self.apex_e, vn - self.apex_n
        if math.hypot(de, dn) < _VEC_EPS:
            return False  # matching the intruder's velocity never closes
        axis_e, axis_n = math.sin(self.bearing), math.cos(self.bearing)
        along = de * axis_e + dn * axis_n
        if along <= 0.0:
            return False  # pointing away from the intruder
        perp = abs(de * axis_n - dn * axis_e)  # |cross| = component perpendicular to the axis
        return math.atan2(perp, along) < self.half - _ANG_EPS

    def edges(self) -> list[tuple[float, float, float, float]]:
        """The two edge rays as ``(origin_e, origin_n, dir_e, dir_n)`` from the apex."""
        out = []
        for sign in (-1.0, 1.0):
            ang = self.bearing + sign * self.half
            out.append((self.apex_e, self.apex_n, math.sin(ang), math.cos(ang)))
        return out


def _cone(own: AircraftState, intr: AircraftState, rpz_eff: float) -> _Cone | None:
    """The intruder's VO cone, or ``None`` if already inside the zone (no cone to leave)."""
    rel = relative_enu(own, intr)
    dist = rel.dist
    if dist <= rpz_eff:
        return None
    vix, viy = velocity_enu(intr)
    bearing = math.atan2(rel.rx, rel.ry)  # qdr to the intruder (atan2 of East, North)
    return _Cone(vix, viy, bearing, math.asin(rpz_eff / dist))


def _project_to_ray(
    pe: float, pn: float, oe: float, on: float, de: float, dn: float
) -> tuple[float, float]:
    """The point on the ray ``origin + t·dir`` (``t ≥ 0``) closest to ``(pe, pn)``."""
    t = max(0.0, (pe - oe) * de + (pn - on) * dn)
    return oe + t * de, on + t * dn


def _ray_intersection(
    e1: tuple[float, float, float, float], e2: tuple[float, float, float, float]
) -> tuple[float, float] | None:
    """Intersection of two rays (each ``origin + t·dir``, ``t ≥ 0``), or ``None``."""
    o1e, o1n, d1e, d1n = e1
    o2e, o2n, d2e, d2n = e2
    denom = d1e * d2n - d1n * d2e
    if abs(denom) < _DENOM_EPS:
        return None
    we, wn = o2e - o1e, o2n - o1n
    t = (we * d2n - wn * d2e) / denom
    s = (we * d1n - wn * d1e) / denom
    if t < 0.0 or s < 0.0:
        return None  # the intersection is behind one of the rays
    return o1e + t * d1e, o1n + t * d1n


def _penetration(ve: float, vn: float, cones: list[_Cone]) -> float:
    """How far inside the union a velocity is — the max over cones of angle-inside-the-cone."""
    worst = 0.0
    for c in cones:
        de, dn = ve - c.apex_e, vn - c.apex_n
        if math.hypot(de, dn) < _VEC_EPS:
            continue
        axis_e, axis_n = math.sin(c.bearing), math.cos(c.bearing)
        along = de * axis_e + dn * axis_n
        if along <= 0.0:
            continue
        depth = c.half - math.atan2(abs(de * axis_n - dn * axis_e), along)
        worst = max(worst, depth)
    return worst


class VO(ConflictResolver):
    """Velocity-Obstacle resolution — shortest way out of the union of cones.

    ``margin`` (>= 1) enlarges the protected zone the cones are built around — the old code's
    ``asas_marh`` (1.05). A genuine per-algorithm parameter, mirroring :class:`~opencdarr.cr.MVP`.
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
        pref = preferred if preferred is not None else velocity_enu(own)
        cones = [c for c in (_cone(own, i, rpz_eff) for i in intruders) if c is not None]

        # nothing to leave (no cones), or the preferred velocity is already outside every cone
        if not cones or not any(c.contains(*pref) for c in cones):
            return MotionCommand.from_velocity(*pref)

        # candidates on the union's boundary: per-cone edge projections of the preferred velocity,
        # plus the pairwise intersections of edges from different cones (the union's vertices)
        candidates: list[tuple[float, float]] = []
        for c in cones:
            for edge in c.edges():
                candidates.append(_project_to_ray(pref[0], pref[1], *edge))
        for a in range(len(cones)):
            for b in range(a + 1, len(cones)):
                for e1 in cones[a].edges():
                    for e2 in cones[b].edges():
                        hit = _ray_intersection(e1, e2)
                        if hit is not None:
                            candidates.append(hit)

        feasible = [v for v in candidates if not any(c.contains(*v) for c in cones)]
        if feasible:
            best = min(feasible, key=lambda v: (v[0] - pref[0]) ** 2 + (v[1] - pref[1]) ** 2)
            return MotionCommand.from_velocity(*best)
        # over-constrained: no reachable exterior velocity -> least-penetration fallback
        best = min(candidates, key=lambda v: _penetration(v[0], v[1], cones))
        return MotionCommand.from_velocity(*best)
