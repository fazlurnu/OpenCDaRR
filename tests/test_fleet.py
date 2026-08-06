"""The N-aircraft fleet runner (Phase 6b).

Two properties:

1. **reduction** — ``run_fleet`` with two agents reproduces ``run_encounter`` bit-for-bit
   (noiseless and seeded-noisy), the free multi-aircraft regression (ADR 0004);
2. **cooperation** — a fleet where *every* aircraft resolves against all the others clears a
   superconflict that, unresolved, collides — and it is deterministic and reproducible from seed.
"""

from __future__ import annotations

import math

import pytest

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cns import Comm, GnssNavigation, lognormal_latency
from opencdarr.cns.broadcast import BroadcastSchedule
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


# The crossing angles the plain-MC estimator actually samples: ``sample_pairwise`` draws
# ``dpsi ~ U(5, 355)``, so the support runs from near-parallel through head-on and back. Both
# passing sides are drawn too. These are the ends where the closing geometry is most degenerate.
_SWEEP_ANGLES = (5.0, 45.0, 90.0, 180.0, 270.0, 355.0)


def _sweep_kw() -> dict:
    """The estimator's own run parameters (the published pairwise validation values)."""
    return dict(rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.5, detector=StateBased(),
                resolver=MVP(margin=1.05), recovery=PastCPA(bouncing_guard=True),
                t_max=250.0, done_timeout=10.0)


def test_run_fleet_reduces_to_run_encounter_across_the_angle_sweep() -> None:
    """The reduction holds across the whole crossing-angle support the MC estimator samples.

    The reduction tests above each pin *one* geometry (90°, 45°). The estimator sweeps ``dpsi``
    over ``(5, 355)`` and both passing sides, so this walks that support — including the
    near-parallel and head-on ends, where a divergence between the two runners would be likeliest
    to hide. It is the precondition for driving plain MC through ``run_fleet``.
    """
    for dpsi in _SWEEP_ANGLES:
        for side in (1, -1):
            own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.2889)
            intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=0.0, tlos=90.0,
                                   rpz=_RPZ, side=side)
            kw = _sweep_kw()
            enc = run_encounter(own, intr, perf=M600, **kw)
            flt = run_fleet([Agent(own, M600), Agent(intr, M600)], **kw)
            assert (flt.conflict, flt.los, flt.min_sep) == (enc.conflict, enc.los, enc.min_sep), (
                f"reduction broke at dpsi={dpsi}, side={side}"
            )


def test_run_fleet_reduces_to_run_encounter_across_the_angle_sweep_noisy() -> None:
    """The same sweep under GNSS noise, on the **estimator's own substream layout**.

    Mirrors ``estimate_p_los``'s split — ``spawn(seq, 3)`` into geometry / navigation /
    communication, always three regardless of which layers are live (ADR 0006 §6) — so this pins
    the exact wiring the MC estimator hands its runner. The geometry substream is spawned and left
    unread here because the geometry is pinned; that is the point of a config-invariant tree.
    """
    for i, dpsi in enumerate(_SWEEP_ANGLES):
        for side in (1, -1):
            own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.2889,
                                pos_ci95=10.0, vel_ci95=1.0)
            intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=0.0, tlos=90.0,
                                   rpz=_RPZ, side=side)
            kw = _sweep_kw() | dict(navigation=GnssNavigation())
            agents = [Agent(own, M600), Agent(intr, M600)]

            _, nav_e, _ = spawn(spawn(root_seed_sequence(11), len(_SWEEP_ANGLES))[i], 3)
            enc = run_encounter(own, intr, perf=M600, rng=generator(nav_e), **kw)
            _, nav_f, _ = spawn(spawn(root_seed_sequence(11), len(_SWEEP_ANGLES))[i], 3)
            flt = run_fleet(agents, rng=generator(nav_f), **kw)

            assert (flt.conflict, flt.los, flt.min_sep) == (enc.conflict, enc.los, enc.min_sep), (
                f"noisy reduction broke at dpsi={dpsi}, side={side}"
            )


def _comm() -> Comm:
    """A lossy link: 80% reception, lognormal latency — the same on every call."""
    return Comm(reception_prob=0.8, latency=lognormal_latency(0.1, 0.25))


def test_run_fleet_lossy_reduces_to_run_encounter() -> None:
    """N=2 lossy gate: run_fleet == run_encounter under the *same* comm model + substream."""
    own, intr = _pair()
    kw = dict(rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=1.0, detector=StateBased(),
              resolver=MVP(margin=1.1), recovery=PastCPA(bouncing_guard=True))
    seq_e = spawn(root_seed_sequence(0), 1)[0]
    enc = run_encounter(own, intr, perf=M600, communication=_comm(),
                        comm_rng=generator(seq_e), **kw)
    seq_f = spawn(root_seed_sequence(0), 1)[0]
    flt = run_fleet([Agent(own, M600), Agent(intr, M600)],
                    communication=_comm(), comm_rng=generator(seq_f), **kw)
    assert (flt.conflict, flt.los, flt.min_sep) == (enc.conflict, enc.los, enc.min_sep)


def test_run_fleet_lossy_reduces_to_run_encounter_noisy() -> None:
    """N=2 lossy gate with GNSS noise too: nav and comm substreams both match run_encounter."""
    own, intr = _noisy_pair()
    kw = dict(rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.2, detector=StateBased(),
              resolver=MVP(margin=1.05), recovery=PastCPA(bouncing_guard=True),
              navigation=GnssNavigation())
    nav_e, comm_e = spawn(root_seed_sequence(3), 2)
    enc = run_encounter(own, intr, perf=M600, rng=generator(nav_e),
                        communication=_comm(), comm_rng=generator(comm_e), **kw)
    nav_f, comm_f = spawn(root_seed_sequence(3), 2)
    flt = run_fleet([Agent(own, M600), Agent(intr, M600)], rng=generator(nav_f),
                    communication=_comm(), comm_rng=generator(comm_f), **kw)
    assert flt.min_sep == enc.min_sep
    assert (flt.conflict, flt.los) == (enc.conflict, enc.los)


def test_run_fleet_reduces_to_run_encounter_off_phase_and_jittered() -> None:
    """The reduction holds at *any* schedule, not only the aligned default.

    ``run_encounter`` used to advance a single global ``next_broadcast`` by a scalar interval, so
    the two aircraft always fired together and this case could not be written at all — the pairwise
    runner had no off-phase or jitter to compare. Both runners now thread the same
    :class:`~opencdarr.cns.broadcast.BroadcastSchedule`, so a per-aircraft phase offset and a
    per-transmission dither reduce like every other CNS effect. ``phase`` is deliberately not a
    multiple of ``dt``, so the two aircraft fire on genuinely different ticks.
    """
    own, intr = _noisy_pair()
    sched = BroadcastSchedule(interval=1.0, phase=[0.0, 0.37], jitter=0.2)
    kw = dict(rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.1, detector=StateBased(),
              resolver=MVP(margin=1.05), recovery=PastCPA(bouncing_guard=True),
              navigation=GnssNavigation(), schedule=sched)
    nav_e, bc_e = spawn(root_seed_sequence(4), 2)
    enc = run_encounter(own, intr, perf=M600, rng=generator(nav_e),
                        broadcast_rng=generator(bc_e), **kw)
    nav_f, bc_f = spawn(root_seed_sequence(4), 2)
    flt = run_fleet([Agent(own, M600), Agent(intr, M600)], rng=generator(nav_f),
                    broadcast_rng=generator(bc_f), **kw)
    assert flt.min_sep == enc.min_sep
    assert (flt.conflict, flt.los) == (enc.conflict, enc.los)


def test_run_encounter_jitter_without_a_stream_is_rejected() -> None:
    """The same guard ``run_fleet`` has: a dithered gap needs its own substream (ADR 0006 §6)."""
    own, intr = _pair()
    with pytest.raises(ValueError, match="broadcast jitter requires broadcast_rng"):
        run_encounter(own, intr, perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=1.0,
                      detector=StateBased(), schedule=BroadcastSchedule(1.0, jitter=0.2))


def test_fleet_perception_gates_avoidance() -> None:
    """Asymmetric perception is exercised, not bypassed: safety tracks who can hear whom.

    A 3-aircraft ring (every pair a conflict) with MVP. Perfect perception clears; a total
    blackout (nothing delivered) collapses to the unresolved collision; and a *mutually blind pair*
    (A0<->A1 links down, A2 fully connected) loses separation on that pair — while A2, on a full
    picture, keeps the fleet off a full collision. Three different perceived sets, three fates.
    """
    def ring() -> list[Agent]:
        return _ring(3)

    def run(comm: Comm) -> object:
        seq = spawn(root_seed_sequence(1), 1)[0]
        return run_fleet(ring(), rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.5, detector=StateBased(),
                         resolver=MVP(margin=1.1), recovery=PastCPA(bouncing_guard=True),
                         communication=comm, comm_rng=generator(seq))

    perfect = run(Comm(reception_prob=1.0))
    mutual = run(Comm(reception_prob={("A0", "A1"): 0.0, ("A1", "A0"): 0.0}))  # A0/A1 blind
    blackout = run(Comm(reception_prob=0.0))  # nobody ever hears anybody

    assert perfect.los is False and perfect.min_sep >= _RPZ          # full picture -> clears
    assert blackout.los is True and blackout.min_sep < _RPZ          # no picture -> collides
    assert mutual.los is True                                        # the blinded pair loses sep
    # monotone in perception quality: perfect > one-pair-blind > total blackout
    assert perfect.min_sep > mutual.min_sep > blackout.min_sep


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
