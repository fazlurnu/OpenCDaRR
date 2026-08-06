"""The N-aircraft fleet runner (Phase 6b).

Two properties:

1. **the n = 2 anchors** — ``run_fleet`` with two agents reproduces the pairwise ``min_sep``
   anchors bit-for-bit (noiseless and seeded-noisy), carried over unchanged from the pairwise
   runner they were first pinned on (ADR 0004: the free multi-aircraft regression);
2. **cooperation** — a fleet where *every* aircraft resolves against all the others clears a
   superconflict that, unresolved, collides — and it is deterministic and reproducible from seed.
"""

from __future__ import annotations

import math

import pytest

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cns import Comm, GnssNavigation
from opencdarr.cns.broadcast import BroadcastSchedule
from opencdarr.cr import MVP, VO
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, run_fleet
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND

_RPZ = 50.0
_LOOKAHEAD = 120.0
_DT = 1.0


def _pair() -> tuple[AircraftState, AircraftState]:
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=90.0, rpz=_RPZ)
    return own, intr


def test_unresolved_encounter_loses_separation() -> None:
    """Baseline: no resolver -> the conflict becomes a loss of separation."""
    own, intr = _pair()
    outcome = run_fleet(
        [Agent(own, M600), Agent(intr, M600)],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT, detector=StateBased(),
    )
    assert outcome.conflict is True
    assert outcome.los is True
    assert outcome.min_sep < _RPZ


def test_resolved_encounter_keeps_separation() -> None:
    """With MVP + Past-CPA the same conflict is cleared with no loss of separation."""
    own, intr = _pair()
    outcome = run_fleet(
        [Agent(own, M600), Agent(intr, M600)],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT, detector=StateBased(),
        resolver=MVP(margin=1.1), recovery=PastCPA(),
    )
    assert outcome.conflict is True
    assert outcome.los is False
    assert outcome.min_sep >= _RPZ


# --- Deterministic n = 2 regression: the layered flow (CruiseAutopilot + SeparationManager) on
# the default kinematics gives an exact, reproducible ``min_sep`` per seed — a strictly stronger
# check than any aggregate rate. First pinned on the pairwise runner and reproduced bit-for-bit
# when plain MC moved onto ``run_fleet``; they now guard the runner directly. Re-anchored on
# ``Multirotor`` in Phase 4c (the new default after Dubins was deleted, ADR 0013): the noiseless
# gentle-maneuver anchors are unchanged from the coupled-heading model (the turn-rate limit never
# bound there), the noisy ones moved (Multirotor sidesteps cleanly where that turn-rate limit used
# to bind). The MVP/VO anchors on the *fixed-wing* airframe live in ``test_mixed_fleet.py``.
#
# Re-anchored again for the segment-minimum measurement (``relative.segment_min_range``): min_sep
# is now the minimum over each *step* rather than at its endpoints, so every anchor moved **down**,
# by 1.8e-5 m to 0.20 m here. The direction is guaranteed, not observed — a segment minimum can
# never exceed the minimum of its own endpoints, pinned as a property in
# ``test_segment_min_sep.py`` so a future change cannot move one of these *up* unnoticed. The
# trajectories themselves are untouched: nothing in the decision path reads min_sep.
#
# Compared with pytest.approx(rel=1e-8), not ==: trig calls compounded over many steps land on a
# different last bit depending on the platform's libm (e.g. macOS vs glibc), even with identical
# code and seed. The tolerance is tight enough to still catch a real modelling regression.
_ANCHOR_NOISELESS_MVP = 109.29398339330471
_ANCHOR_NOISELESS_VO = 109.82844921479813
_ANCHOR_NOISY_MVP = 267.74238306504367
_ANCHOR_NOISY_VO = 261.9739565914773


def test_bit_for_bit_noiseless_mvp() -> None:
    """Deterministic (no-noise) MVP encounter reproduces the pinned min_sep exactly."""
    own, intr = _pair()
    out = run_fleet(
        [Agent(own, M600), Agent(intr, M600)],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT,
        detector=StateBased(), resolver=MVP(margin=1.1), recovery=PastCPA(),
    )
    assert out.min_sep == pytest.approx(_ANCHOR_NOISELESS_MVP, rel=1e-8)


def test_bit_for_bit_noiseless_vo() -> None:
    """Deterministic (no-noise) VO encounter reproduces the pinned min_sep exactly."""
    own, intr = _pair()
    out = run_fleet(
        [Agent(own, M600), Agent(intr, M600)],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT,
        detector=StateBased(), resolver=VO(margin=1.1), recovery=PastCPA(),
    )
    assert out.min_sep == pytest.approx(_ANCHOR_NOISELESS_VO, rel=1e-8)


def _noisy_min_sep(resolver: MVP | VO) -> float:
    """One seeded, GPS-noisy encounter through the full self-fix path (exercises CruiseAutopilot's
    state-independence and the SeparationManager under noise). Seed 0, single substream."""
    seq = list(spawn(root_seed_sequence(0), 1))[0]
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.2889, pos_ci95=10.0, vel_ci95=1.0
    )
    intr = create_conflict(own, intr_id="INT", dpsi=45.0, dcpa=0.0, tlos=180.0, rpz=_RPZ, side=1)
    return run_fleet(
        [Agent(own, M600), Agent(intr, M600)],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.2,
        detector=StateBased(), resolver=resolver, recovery=PastCPA(bouncing_guard=True),
        navigation=GnssNavigation(), rng=generator(seq),
    ).min_sep


def test_bit_for_bit_noisy_mvp() -> None:
    """Seeded GPS-noisy MVP encounter reproduces the pinned min_sep exactly (noisy path)."""
    assert _noisy_min_sep(MVP(margin=1.05)) == pytest.approx(_ANCHOR_NOISY_MVP, rel=1e-8)


def test_bit_for_bit_noisy_vo() -> None:
    """Seeded GPS-noisy VO encounter reproduces the pinned min_sep exactly (noisy path)."""
    assert _noisy_min_sep(VO(margin=1.05)) == pytest.approx(_ANCHOR_NOISY_VO, rel=1e-8)


def test_run_fleet_no_wind_matches_default() -> None:
    """Threading ``wind=NO_WIND`` through the runner reproduces the default-run outcome exactly."""
    own, intr = _pair()
    kw = dict(
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT, detector=StateBased(),
        resolver=MVP(margin=1.1), recovery=PastCPA(),
    )
    agents = [Agent(own, M600), Agent(intr, M600)]
    assert run_fleet(agents, wind=NO_WIND, **kw) == run_fleet(agents, **kw)


def test_jitter_without_a_stream_is_rejected() -> None:
    """A dithered gap needs its own substream (ADR 0006 §6)."""
    own, intr = _pair()
    with pytest.raises(ValueError, match="broadcast jitter requires broadcast_rng"):
        run_fleet([Agent(own, M600), Agent(intr, M600)],
                  rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT,
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
