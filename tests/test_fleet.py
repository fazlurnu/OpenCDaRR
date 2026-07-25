"""The N-aircraft fleet runner (Phase 6b).

Two properties:

1. **reduction** — ``run_fleet`` with two agents reproduces ``run_encounter`` bit-for-bit
   (noiseless and seeded-noisy), the free multi-aircraft regression (ADR 0004);
2. **cooperation** — a fleet where *every* aircraft resolves against all the others clears a
   superconflict that, unresolved, collides — and it is deterministic and reproducible from seed.
"""

from __future__ import annotations

import math

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.cr import MVP, VO
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, run_fleet
from opencdarr.loop import run_encounter
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0
_LOOKAHEAD = 120.0


def _pair() -> tuple[AircraftState, AircraftState]:
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=90.0, rpz=_RPZ)
    return own, intr


def _noisy_pair() -> tuple[AircraftState, AircraftState]:
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0, pos_ci95=10.0, vel_ci95=1.0)
    intr = create_conflict(own, intr_id="INT", dpsi=45.0, dcpa=0.0, tlos=180.0, rpz=_RPZ, side=1)
    return own, intr


def test_run_fleet_reduces_to_run_encounter_noiseless() -> None:
    """Two agents, no noise: run_fleet == run_encounter for MVP and VO (conflict/los/min_sep)."""
    for resolver in (MVP(margin=1.1), VO(margin=1.1)):
        own, intr = _pair()
        kw = dict(rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=1.0, detector=StateBased(),
                  resolver=resolver, recovery=PastCPA(bouncing_guard=True))
        enc = run_encounter(own, intr, perf=M600, **kw)
        flt = run_fleet([Agent(own, M600), Agent(intr, M600)], **kw)
        assert (flt.conflict, flt.los, flt.min_sep) == (enc.conflict, enc.los, enc.min_sep)


def test_run_fleet_reduces_to_run_encounter_noisy() -> None:
    """Two agents, seeded GNSS noise: run_fleet == run_encounter on the same substream."""
    for resolver in (MVP(margin=1.05), VO(margin=1.05)):
        own, intr = _noisy_pair()
        kw = dict(rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.2, detector=StateBased(),
                  resolver=resolver, recovery=PastCPA(bouncing_guard=True),
                  navigation=GnssNavigation())
        seq_e = list(spawn(root_seed_sequence(0), 1))[0]
        enc = run_encounter(own, intr, perf=M600, rng=generator(seq_e), **kw)
        seq_f = list(spawn(root_seed_sequence(0), 1))[0]
        flt = run_fleet([Agent(own, M600), Agent(intr, M600)], rng=generator(seq_f), **kw)
        assert flt.min_sep == enc.min_sep
        assert (flt.conflict, flt.los) == (enc.conflict, enc.los)


def _ring(n: int, radius: float = 1500.0, speed: float = 10.0) -> list[Agent]:
    """n aircraft uniformly on a circle, each flying toward the diametrically-opposite start."""
    agents = []
    for k in range(n):
        bearing = 360.0 * k / n
        lat, lon = geo.forward(52.0, 4.0, bearing, radius)
        trk = (bearing + 180.0) % 360.0  # head across the circle
        agents.append(Agent(AircraftState(id=f"A{k}", lat=lat, lon=lon, trk=trk, gs=speed), M600))
    return agents


def _run_ring(n: int, resolver: ConflictResolver | None) -> object:
    return run_fleet(
        _ring(n), rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.5, detector=StateBased(),
        resolver=resolver, recovery=PastCPA(bouncing_guard=True) if resolver else None,
    )


def test_cooperative_ring_resolves_where_unresolved_collides() -> None:
    """8 aircraft crossing to the opposite side: unresolved collide; cooperating, they clear."""
    unresolved = _run_ring(8, None)
    assert unresolved.los is True and unresolved.min_sep < _RPZ  # a genuine superconflict
    for resolver in (MVP(margin=1.1), VO(margin=1.1)):
        resolved = _run_ring(8, resolver)
        assert resolved.conflict is True
        assert resolved.los is False
        assert resolved.min_sep >= _RPZ  # every pair clears — the whole fleet manoeuvres


def test_fleet_is_deterministic() -> None:
    """No RNG: identical inputs -> identical outcome."""
    a, b = _run_ring(4, MVP(margin=1.1)), _run_ring(4, MVP(margin=1.1))
    assert (a.conflict, a.los, a.min_sep) == (b.conflict, b.los, b.min_sep)
    assert math.isfinite(a.min_sep)
