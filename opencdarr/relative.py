"""Relative geometry shared by detection, resolution, and recovery.

``relative_enu`` extracts the relative position and velocity of ``intr`` with respect to
``own`` in the local East–North frame — the common front-end of the CPA algebra. Centralising
it keeps the ``intr − own`` sign convention in *one* place (see ``cpa-detection.md``), so no
algorithm can accidentally flip it. The CPA equations themselves (``t_cpa``, ``d_cpa``, …)
deliberately stay in each algorithm, where a reviewer reads them (``design-philosophy.md``
#11: in plumbing DRY wins; in the core math legibility wins).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from opencdarr import geo
from opencdarr.state import AircraftState
from opencdarr.wind import WindField


def velocity_enu(state: AircraftState) -> tuple[float, float]:
    """Ground velocity as (East, North) components in m/s."""
    r = math.radians(state.trk)
    return state.gs * math.sin(r), state.gs * math.cos(r)


# --- Wind triangle (Phase 5): the Eq 9 vector sum and its consequences -----------------------
# The one relation everything reduces to is  ground velocity = airspeed vector + wind  (Eq 9). The
# helpers below are that sum, its inverse, and the two scalar read-outs (ground speed as a closed
# form, and the crab a fixed-wing must hold to make good a course). All in the ENU frame, angles in
# aviation convention (0 = North, clockwise). At ``NO_WIND`` every one degenerates to its no-wind
# value (``air_to_ground`` is the identity, ``ground_speed`` = ``v_tas``, the crab is zero).


def air_to_ground(v_air_enu: tuple[float, float], wind: WindField) -> tuple[float, float]:
    """Ground velocity ``(east, north)`` from an airspeed vector and the wind (Eq 9 vector sum)."""
    ae, an = v_air_enu
    we, wn = wind.components()
    return ae + we, an + wn


def ground_to_air(v_ground_enu: tuple[float, float], wind: WindField) -> tuple[float, float]:
    """Airspeed vector ``(east, north)`` needed to make good a ground velocity — Eq 9 inverted."""
    ge, gn = v_ground_enu
    we, wn = wind.components()
    return ge - we, gn - wn


def ground_track(v_ground_enu: tuple[float, float]) -> float:
    """Ground course χ [deg, aviation] of a ground-velocity vector (``0.0`` for a zero vector)."""
    ge, gn = v_ground_enu
    return math.degrees(math.atan2(ge, gn)) % 360.0


def ground_speed(v_tas: float, wind: WindField, psi: float) -> float:
    """Ground speed V_GS [m/s] for airspeed ``v_tas`` flown at heading ``psi`` — Eq 4 closed form.

    ``V_GS = √(V_TAS² + V_WS² − 2·V_TAS·V_WS·cos(ψ − θ_wa))`` (cosine rule on the wind triangle,
    ``θ_wa`` the wind's meteorological coming-from bearing). Independent of :func:`air_to_ground`,
    so the two agreeing is a real cross-check, not a tautology.
    """
    theta_wa = math.radians(wind.coming_from)
    delta = math.radians(psi) - theta_wa
    return math.sqrt(v_tas * v_tas + wind.speed * wind.speed
                     - 2.0 * v_tas * wind.speed * math.cos(delta))


def wind_correction_angle(v_tas: float, wind: WindField, chi: float) -> float | None:
    """Crab angle θ_w = ψ − χ [deg] to make good course ``chi`` at airspeed ``v_tas`` (Eq 3).

    ``θ_w = arcsin((V_WS/V_TAS)·sin(θ_wa − χ))``. Add it to the desired course to get the heading
    to fly: ``ψ = χ + θ_w``. Returns ``None`` when ``|(V_WS/V_TAS)·sin(θ_wa − χ)| > 1`` — the
    course is **unachievable** (the wind's cross-course component exceeds the airspeed, the
    downwind-dominated regime), the fixed-wing analog of insufficient bank; the caller steers the
    closest achievable course rather than pretending (Phase-5 plan decision 4). Raises on
    ``v_tas ≤ 0`` (a fixed-wing
    always flies a positive airspeed; a zero would make the crab undefined).
    """
    if v_tas <= 0.0:
        raise ValueError(f"wind_correction_angle needs a positive airspeed, got {v_tas}")
    theta_wa = math.radians(wind.coming_from)
    arg = (wind.speed / v_tas) * math.sin(theta_wa - math.radians(chi))
    if abs(arg) > 1.0:
        return None  # unachievable course (downwind-dominated) — caller handles, decision 4
    return math.degrees(math.asin(arg))


@dataclass(frozen=True)
class Relative:
    """Position and velocity of intr relative to own, East–North (intr − own)."""

    rx: float  # East position [m]
    ry: float  # North position [m]
    vx: float  # East velocity [m/s]
    vy: float  # North velocity [m/s]

    @property
    def dist(self) -> float:
        """Current range [m]."""
        return math.hypot(self.rx, self.ry)


def relative_enu(own: AircraftState, intr: AircraftState) -> Relative:
    """Relative position and velocity (intr − own) in the local East–North frame."""
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    vox, voy = velocity_enu(own)
    vix, viy = velocity_enu(intr)
    return Relative(rx=dist * math.sin(q), ry=dist * math.cos(q), vx=vix - vox, vy=viy - voy)


def segment_min_range(r0: Relative, r1: Relative) -> float:
    """Closest range [m] reached over one integration step, endpoints included.

    Takes a pair's relative position at both ends of a ``dt`` step and returns the minimum of the
    straight segment between them. This is a **measurement** helper, not a CDR algorithm — it lives
    here rather than beside the CPA equations in ``cd/`` because separation measurement is the
    runner's concern (:mod:`~opencdarr.fleet`), and sharing the one copy here is what keeps every
    consumer measuring identically.

    Reading separation only at step endpoints under-reports it: a pass that dips inside a threshold
    and back out within one step leaves no sampled point inside, and the error is one-sided — the
    reported minimum can only ever be too *large*. Harmless next to ``rpz``, severe at the small
    radii a rare-event estimator splits on, where the relative error in ``P(min_sep <= d)`` scales
    as ``(v_rel*dt)^2 / (24 d^2)``.

    Interpolates **positions**; it does not extrapolate ``r0``'s velocity. A velocity extrapolation
    would leave the flown path whenever an aircraft is turning and can report a range at a point
    the aircraft never occupied — measured inventing losses of separation that never were. Every
    range returned here lies on the segment between two states the simulation actually produced.
    Velocities on ``r0`` / ``r1`` are therefore unused.
    """
    dx, dy = r1.rx - r0.rx, r1.ry - r0.ry
    d2 = dx * dx + dy * dy
    if d2 <= 0.0:
        return r0.dist  # no relative displacement over the step
    # s parameterises the step over [0, 1]; the unconstrained minimum sits at s
    s = -(r0.rx * dx + r0.ry * dy) / d2
    if s <= 0.0:
        return r0.dist  # already past closest approach at the step's start
    if s >= 1.0:
        return r1.dist  # still closing at the step's end
    return math.hypot(r0.rx + s * dx, r0.ry + s * dy)


def pairwise_min_sep(states: tuple[AircraftState, ...] | list[AircraftState]) -> float:
    """Smallest separation over all unordered pairs [m], at this instant."""
    smallest = float("inf")
    for i in range(len(states)):
        for j in range(i + 1, len(states)):
            _, dist = geo.qdrdist(states[i].lat, states[i].lon, states[j].lat, states[j].lon)
            smallest = min(smallest, dist)
    return smallest


def pairwise_relative(
    states: tuple[AircraftState, ...] | list[AircraftState]
) -> tuple[Relative, ...]:
    """Relative position of every unordered pair, in the fixed ``i < j`` order [m].

    The vector form of :func:`pairwise_min_sep` — same one ``geo.qdrdist`` per pair, so it costs
    essentially the same, but it returns the geometry rather than only its magnitude. That is what
    :func:`segment_min_sep` needs to close the gap *between* two sampled instants.
    """
    return tuple(
        relative_enu(states[i], states[j])
        for i in range(len(states))
        for j in range(i + 1, len(states))
    )


def pair_ids(n: int) -> tuple[tuple[int, int], ...]:
    """The unordered pairs ``(i, j)``, ``i < j`` — the order every pairwise helper walks."""
    return tuple((i, j) for i in range(n) for j in range(i + 1, n))


def pair_min_ranges(pre: tuple[Relative, ...], post: tuple[Relative, ...]) -> tuple[float, ...]:
    """Each pair's smallest separation across the step [m], in the fixed ``i < j`` order.

    The per-pair form of :func:`segment_min_sep` — the ``min`` of this *is* that. Keeping the
    whole vector lets :meth:`~opencdarr.fleet.FleetEnv.advance` name *which* pairs crossed ``rpz``
    (K, A) rather than only whether one did, at no extra geometry: the same one
    :func:`segment_min_range` per pair either way.
    """
    return tuple(segment_min_range(r0, r1) for r0, r1 in zip(pre, post, strict=True))


def segment_min_sep(pre: tuple[Relative, ...], post: tuple[Relative, ...]) -> float:
    """Smallest separation over all pairs across a whole ``dt`` step, endpoints included [m].

    Interpolates each pair's relative position linearly between the two ends of the step and takes
    the closed-form minimum of that segment. Sampling separation only at the step *endpoints*
    misses a pass that dips inside a threshold and back out within one step, and the miss is
    one-sided — it can only report *more* separation than there was. The bias is negligible against
    ``rpz`` but severe at the small radii IPS splits on: the relative error in
    ``P(min_sep <= d)`` goes as ``(v_rel*dt)^2 / (24 d^2)``, i.e. it grows as the target tightens.
    See ``vault/observations/segment-min-separation.md`` for the measurements.

    The per-pair algebra is :func:`segment_min_range`. The ``min`` is taken through
    :func:`pair_min_ranges`, the one place the per-pair vector is formed, so
    :meth:`~opencdarr.fleet.FleetEnv.advance`'s K/A read and this scalar cannot disagree.
    """
    return min(pair_min_ranges(pre, post), default=float("inf"))
