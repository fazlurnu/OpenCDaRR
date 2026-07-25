"""Conflict-encounter geometry — the scenario layer's generator.

`create_conflict` places an intruder in conflict with a given ownship at a chosen crossing
angle, miss distance, and time-to-loss-of-separation — the horizontal part of BlueSky's
`creconfs`, re-derived in our convention (relative velocity = intr − own; no wind; 2D).

Governing equations: ``vault/derivations/conflict-geometry.md``.
"""

from __future__ import annotations

import math

import numpy as np

from opencdarr import geo
from opencdarr.state import AircraftState

_PARALLEL_EPS = 1e-9  # |v_rel| below this = no closing geometry
_DPSI_MIN = 5.0  # deg: exclude a band around 0/360 (near-parallel, near-degenerate closing)


def create_conflict(
    own: AircraftState,
    *,
    intr_id: str,
    dpsi: float,
    dcpa: float,
    tlos: float,
    rpz: float,
    gs_intr: float | None = None,
    side: int = 1,
    pos_ci95: float | None = None,
    vel_ci95: float | None = None,
) -> AircraftState:
    """Return an intruder in conflict with ``own``.

    The intruder crosses at ``dpsi`` degrees, with closest approach ``dcpa`` metres reached
    such that separation is first lost (enters ``rpz``) ``tlos`` seconds from now. Speed
    defaults to the ownship's; ``side`` (+1/−1) selects which side it passes. ``pos_ci95``/
    ``vel_ci95`` (the intruder's own declared measurement accuracy) default to matching
    ``own``'s, the same way ``gs_intr`` defaults to ``own.gs``.
    """
    if tlos < 0 or dcpa < 0 or rpz <= 0:
        raise ValueError(f"require tlos>=0, dcpa>=0, rpz>0; got {tlos=}, {dcpa=}, {rpz=}")
    if side not in (-1, 1):
        raise ValueError(f"side must be +1 or -1, got {side}")

    gs_i = own.gs if gs_intr is None else gs_intr
    psi_i = (own.trk + dpsi) % 360.0

    # velocities and relative velocity (intr − own), East–North
    vox = own.gs * math.sin(math.radians(own.trk))
    voy = own.gs * math.cos(math.radians(own.trk))
    vix = gs_i * math.sin(math.radians(psi_i))
    viy = gs_i * math.cos(math.radians(psi_i))
    we = vix - vox
    wn = viy - voy
    vrel = math.hypot(we, wn)
    if vrel < _PARALLEL_EPS:
        raise ValueError("cannot construct a conflict with zero relative velocity")

    # distances: to CPA along closing, and the initial range
    half_chord = math.sqrt(rpz * rpz - dcpa * dcpa) if dcpa < rpz else 0.0
    d_rel = tlos * vrel + half_chord
    dist = math.hypot(d_rel, dcpa)

    # r0 = -d_rel * w_hat + dcpa * n_hat, with n_hat = side * (-wn, we)/vrel
    we_hat = we / vrel
    wn_hat = wn / vrel
    r0e = -d_rel * we_hat + dcpa * side * (-wn_hat)
    r0n = -d_rel * wn_hat + dcpa * side * we_hat
    bearing = math.degrees(math.atan2(r0e, r0n)) % 360.0

    lat, lon = geo.forward(own.lat, own.lon, bearing, dist)
    ci95_p = own.pos_ci95 if pos_ci95 is None else pos_ci95
    ci95_v = own.vel_ci95 if vel_ci95 is None else vel_ci95
    return AircraftState(
        id=intr_id, lat=lat, lon=lon, trk=psi_i, gs=gs_i, pos_ci95=ci95_p, vel_ci95=ci95_v
    )


def sample_pairwise(
    rng: np.random.Generator,
    *,
    speed: float,
    dcpa_max: float,
    tlos: float,
    rpz: float,
    own_id: str = "OWN",
    intr_id: str = "INT",
    pos_ci95: float = 0.0,
    vel_ci95: float = 0.0,
) -> tuple[AircraftState, AircraftState]:
    """Draw one random pairwise encounter from the seeded generator.

    Ownship flies north from a fixed origin at ``speed``; the intruder crosses at a random
    angle ``dpsi`` (uniform over the full range, excluding a near-0/360 band), miss distance
    ``dcpa`` ~ U(0, dcpa_max), and random side. This is the encounter distribution the plain-MC
    estimator samples. ``pos_ci95``/``vel_ci95`` set both aircraft's declared measurement
    accuracy (default 0 = perfect); the intruder inherits the ownship's via ``create_conflict``.
    """
    dpsi = float(rng.uniform(_DPSI_MIN, 360.0 - _DPSI_MIN))
    dcpa = float(rng.uniform(0.0, dcpa_max))
    side = 1 if rng.random() < 0.5 else -1
    own = AircraftState(
        id=own_id, lat=52.0, lon=4.0, trk=0.0, gs=speed, pos_ci95=pos_ci95, vel_ci95=vel_ci95
    )
    intr = create_conflict(
        own, intr_id=intr_id, dpsi=dpsi, dcpa=dcpa, tlos=tlos, rpz=rpz, side=side
    )
    return own, intr


# --- N-aircraft fleet scenarios (Phase 6d) --------------------------------------------------
# Each builder returns a list of ``(AircraftState, goto_target)`` pairs — an aircraft heading at
# its destination ``(lat, lon)``, the geometry the fleet loop needs. The caller wraps each in a
# ``WaypointAutopilot`` mission + its airframe (an ``Agent``); the scenario stays airframe-neutral.

FleetScenario = list[tuple[AircraftState, tuple[float, float]]]


def _heading_to(lat: float, lon: float, target: tuple[float, float], speed: float,
                ac_id: str) -> AircraftState:
    """An aircraft at ``(lat, lon)`` flying at ``speed`` toward ``target`` (nose on the bearing)"""
    trk, _ = geo.qdrdist(lat, lon, target[0], target[1])
    return AircraftState(id=ac_id, lat=lat, lon=lon, trk=trk % 360.0, gs=speed)


def swap_pair(
    *, speed: float = 10.0, span: float = 3000.0, lat0: float = 52.0, lon0: float = 4.0
) -> FleetScenario:
    """Two aircraft ``span`` m apart, each flying to the *other's* start — a head-on swap
    (Phase-6 scenario 1). Placed so the DAA clears with the waypoint still ahead.
    """
    a = geo.forward(lat0, lon0, 270.0, span / 2)  # west
    b = geo.forward(lat0, lon0, 90.0, span / 2)  # east
    return [
        (_heading_to(a[0], a[1], (b[0], b[1]), speed, "A"), (b[0], b[1])),
        (_heading_to(b[0], b[1], (a[0], a[1]), speed, "B"), (a[0], a[1])),
    ]


def swap_ring(
    n: int = 8, *, speed: float = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
) -> FleetScenario:
    """``n`` aircraft uniformly on a ring, each flying to the **diametrically-opposite** start
    (Phase-6 scenario 2) — ``n/2`` head-on pairs all crossing the centre.
    """
    starts = [geo.forward(lat0, lon0, 360.0 * k / n, radius) for k in range(n)]
    out: FleetScenario = []
    for k in range(n):
        target = starts[(k + n // 2) % n]
        out.append((_heading_to(starts[k][0], starts[k][1], target, speed, f"A{k}"), target))
    return out


def converging_ring(
    n: int = 8, *, speed: float = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
) -> FleetScenario:
    """``n`` aircraft uniformly on a circle, all flying to the **same** waypoint — the ring centre
    (Phase-6 scenario 3), the symmetric converging superconflict. They cannot all occupy the centre
    (``rpz`` forbids it), so the DAA can only hold them apart as they converge.
    """
    centre = (lat0, lon0)
    ring = [geo.forward(lat0, lon0, 360.0 * k / n, radius) for k in range(n)]
    return [(_heading_to(s[0], s[1], centre, speed, f"A{k}"), centre) for k, s in enumerate(ring)]


def near_parallel(
    *, speed: float = 10.0, dpsi: float = 5.0, tlos: float = 90.0, rpz: float = 50.0,
    reach: float = 3000.0, lat0: float = 52.0, lon0: float = 4.0
) -> FleetScenario:
    """Two aircraft crossing at a shallow ``dpsi`` (default 5°) — the near-parallel, slow-closing
    hard case (Phase-6 scenario 4). The ownship heads north; the intruder is placed by
    :func:`create_conflict`; each aircraft's waypoint is ``reach`` metres ahead along its track.
    """
    own = AircraftState(id="OWN", lat=lat0, lon=lon0, trk=0.0, gs=speed)
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=0.0, tlos=tlos, rpz=rpz, side=1)
    out: FleetScenario = []
    for ac in (own, intr):
        target = geo.forward(ac.lat, ac.lon, ac.trk, reach)
        out.append((ac, (target[0], target[1])))
    return out
