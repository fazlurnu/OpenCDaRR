"""Two aircraft in one conflict — the geometry, the sampler, and the declarable scenario.

``create_conflict`` builds **one named geometry**: an intruder placed in conflict with a given
ownship at a chosen crossing angle, miss distance and time-to-loss-of-separation. It is the
horizontal part of BlueSky's ``creconfs``, re-derived in our convention (relative velocity =
intr − own; no wind; 2D). ``sample_pairwise`` turns **one seed into one encounter**, drawing
whichever parameters the caller has not pinned, and :class:`PairwiseEncounter` is that sampler as
a declarable value. The two fixed geometries here — a head-on swap and a shallow crossing — are
named cases of the same construction.

Governing equations: ``vault/derivations/conflict-geometry.md``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr import geo
from opencdarr.config import Config
from opencdarr.scenario.base import (
    Draw,
    FleetScenario,
    Scenario,
    _heading_to,
    _per_aircraft_speeds,
)
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
        id=intr_id, lat=lat, lon=lon, trk=psi_i, gs=gs_i, pos_ci95=ci95_p, vel_ci95=ci95_v,
        # The intruder always inherits the ownship's *declaration policy*, with no override
        # parameter of its own: ``None`` already means "declare honestly" on the state, so it
        # cannot double as the "inherit from own" sentinel the two above use. An asymmetric study
        # -- one aircraft over-declaring while the other is honest -- would need a real sentinel,
        # and there is no second implementation asking for one yet.
        pos_ci95_declared=own.pos_ci95_declared,
        vel_ci95_declared=own.vel_ci95_declared,
    )

def _resolve(spec: float | Draw | None, rng: np.random.Generator, drawn: float) -> float:
    """One geometry slot's value: the built-in draw, a pinned constant, or a custom distribution.

    ``drawn`` has *already* been taken from ``rng`` by the caller, whether or not it is used — see
    :func:`sample_pairwise` on why a pinned slot still consumes its draw.
    """
    if spec is None:
        return drawn
    if callable(spec):
        return float(spec(rng))
    return float(spec)


def sample_pairwise(
    rng: np.random.Generator,
    *,
    speed: float,
    dcpa_max: float,
    tlos: float,
    rpz: float,
    dpsi: float | Draw | None = None,
    dcpa: float | Draw | None = None,
    side: int | Draw | None = None,
    gs_intr: float | Draw | None = None,
    own_id: str = "OWN",
    intr_id: str = "INT",
    pos_ci95: float = 0.0,
    vel_ci95: float = 0.0,
    pos_ci95_declared: float | None = None,
    vel_ci95_declared: float | None = None,
) -> tuple[AircraftState, AircraftState]:
    """Draw one pairwise encounter from the seeded generator.

    Ownship flies north from a fixed origin at ``speed``; the intruder crosses at ``dpsi`` degrees
    with miss distance ``dcpa``, passing on ``side``, at ``gs_intr``. Left alone, every one of
    those four is **drawn** — the encounter distribution the plain-MC estimator integrates over:
    ``dpsi`` uniform over the full range bar a near-0/360 band, ``dcpa`` ~ U(0, ``dcpa_max``),
    either side equally likely, and the intruder matching the ownship's speed.

    Each of the four also takes an override, so one call expresses a whole family of scenarios:

    - **a constant** — ``dpsi=90.0`` pins a 90° crossing, the single-geometry case a rare-event run
      or a per-angle response curve needs (and what ``scripts/ipr_angle_sweep.py`` open-coded);
    - **a callable** ``(rng) -> float`` — a custom distribution for that parameter, drawn per
      encounter from this encounter's own generator (e.g. a von Mises crossing angle rather
      than the uniform one).

    A *list* of values is deliberately not accepted: sweeping a parameter means several independent
    estimates, each with its own counts, interval and cache entry, so it belongs to the caller that
    fans conditions out — not to a function whose job is one encounter from one seed.

    **Why a pinned slot still consumes its draw.** The three built-in draws are taken in a fixed
    order (``dpsi``, ``dcpa``, ``side``) *before* any override is applied, and a custom
    distribution draws only afterwards. So pinning the crossing angle cannot shift the miss
    distance or the passing side, and the all-default call is bit-identical to the pre-override
    one. This is the same config-invariant-stream discipline the per-encounter substream fan-out
    follows one level up (ADR 0006 §6, ``estimator.estimate_ipr``): draw the same things in the
    same order regardless of which are used, so the tree never moves. The cost is a couple of
    discarded ``uniform`` calls.

    ``pos_ci95``/``vel_ci95`` set both aircraft's **actual** measurement accuracy (default 0 =
    perfect); the intruder inherits the ownship's via :func:`create_conflict`.
    ``pos_ci95_declared``/``vel_ci95_declared`` set what their broadcasts claim instead, ``None``
    (default) claiming the truth — swept against the two above, that is the over- or
    under-confident-declaration experiment. Both aircraft share one declaration policy, for the
    reason :func:`create_conflict` records.

    Note that the near-0/360 exclusion band (``_DPSI_MIN``) constrains only the *built-in* angle
    draw, which avoids near-parallel geometries whose closing speed is degenerate. A pinned or
    custom ``dpsi`` is passed through as given, so a deliberate shallow-crossing study (the
    published sweeps start at 2°) is not silently clamped — a genuinely unconstructable geometry
    fails in :func:`create_conflict` instead.
    """
    # every built-in draw happens, in this order, whether or not its value survives the override
    dpsi_drawn = float(rng.uniform(_DPSI_MIN, 360.0 - _DPSI_MIN))
    dcpa_drawn = float(rng.uniform(0.0, dcpa_max))
    side_drawn = 1.0 if rng.random() < 0.5 else -1.0

    dpsi_v = _resolve(dpsi, rng, dpsi_drawn)
    dcpa_v = _resolve(dcpa, rng, dcpa_drawn)
    side_v = int(_resolve(side, rng, side_drawn))
    # the intruder's speed has no built-in draw — it defaults to the ownship's inside
    # ``create_conflict`` — so an absent override consumes nothing and appends no draw
    gs_intr_v = None if gs_intr is None else _resolve(gs_intr, rng, float("nan"))

    own = AircraftState(
        id=own_id, lat=52.0, lon=4.0, trk=0.0, gs=speed, pos_ci95=pos_ci95, vel_ci95=vel_ci95,
        pos_ci95_declared=pos_ci95_declared, vel_ci95_declared=vel_ci95_declared,
    )
    intr = create_conflict(
        own, intr_id=intr_id, dpsi=dpsi_v, dcpa=dcpa_v, tlos=tlos, rpz=rpz, side=side_v,
        gs_intr=gs_intr_v,
    )
    return own, intr

def swap_pair(
    *, speed: float | Sequence[float] = 10.0, span: float = 3000.0,
    lat0: float = 52.0, lon0: float = 4.0
) -> FleetScenario:
    """Two aircraft ``span`` m apart, each flying to the *other's* start — a head-on swap
    (Phase-6 scenario 1). Placed so the DAA clears with the waypoint still ahead.
    """
    speeds = _per_aircraft_speeds(speed, 2)
    a = geo.forward(lat0, lon0, 270.0, span / 2)  # west
    b = geo.forward(lat0, lon0, 90.0, span / 2)  # east
    return [
        (_heading_to(a[0], a[1], (b[0], b[1]), speeds[0], "A"), (b[0], b[1])),
        (_heading_to(b[0], b[1], (a[0], a[1]), speeds[1], "B"), (a[0], a[1])),
    ]

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

@dataclass(frozen=True)
class PairwiseEncounter(Scenario):
    """Two aircraft in a sampled conflict — :func:`sample_pairwise` as a declarable value.

    Every geometry slot left ``None`` is drawn per encounter; pinning one (``dpsi=90``) fixes it
    without disturbing the others' draws. This is the default scenario, and the historical one:
    it reproduces what ``run_experiment`` did before scenarios existed, encounter for encounter.
    """

    dpsi: float | Draw | None = None
    dcpa: float | Draw | None = None
    side: int | Draw | None = None
    gs_intr: float | Draw | None = None

    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        own, intr = sample_pairwise(
            rng,
            speed=config.scenario.speed,
            dcpa_max=config.scenario.dcpa_max,
            tlos=config.scenario.tlos,
            rpz=config.conflict.rpz,
            pos_ci95=config.scenario.pos_ci95,
            vel_ci95=config.scenario.vel_ci95,
            pos_ci95_declared=config.scenario.pos_ci95_declared,
            vel_ci95_declared=config.scenario.vel_ci95_declared,
            dpsi=self.dpsi,
            dcpa=self.dcpa,
            side=self.side,
            gs_intr=self.gs_intr,
        )
        return [(own, None), (intr, None)]  # no destination: the geometry is the experiment

    def size(self) -> int:
        return 2
