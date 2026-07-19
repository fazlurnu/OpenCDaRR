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
) -> AircraftState:
    """Return an intruder in conflict with ``own``.

    The intruder crosses at ``dpsi`` degrees, with closest approach ``dcpa`` metres reached
    such that separation is first lost (enters ``rpz``) ``tlos`` seconds from now. Speed
    defaults to the ownship's; ``side`` (+1/−1) selects which side it passes.
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
    return AircraftState(id=intr_id, lat=lat, lon=lon, trk=psi_i, gs=gs_i)


def sample_pairwise(
    rng: np.random.Generator,
    *,
    speed: float,
    dcpa_max: float,
    tlos: float,
    rpz: float,
    own_id: str = "OWN",
    intr_id: str = "INT",
) -> tuple[AircraftState, AircraftState]:
    """Draw one random pairwise encounter from the seeded generator.

    Ownship flies north from a fixed origin at ``speed``; the intruder crosses at a random
    angle ``dpsi`` (uniform over the full range, excluding a near-0/360 band), miss distance
    ``dcpa`` ~ U(0, dcpa_max), and random side. This is the encounter distribution the plain-MC
    estimator samples.
    """
    dpsi = float(rng.uniform(_DPSI_MIN, 360.0 - _DPSI_MIN))
    dcpa = float(rng.uniform(0.0, dcpa_max))
    side = 1 if rng.random() < 0.5 else -1
    own = AircraftState(id=own_id, lat=52.0, lon=4.0, trk=0.0, gs=speed)
    intr = create_conflict(
        own, intr_id=intr_id, dpsi=dpsi, dcpa=dcpa, tlos=tlos, rpz=rpz, side=side
    )
    return own, intr
