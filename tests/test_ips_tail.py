"""The IPS tail leg — the continuation that makes K and A observable (ADR 0022).

The ladder stops each survivor the instant its running minimum crosses a shell, so at the rare
boundary every survivor has exactly one losing pair *by construction*: K = 1, A = 2, whatever the
fleet actually went on to do. Those are artefacts of where the ladder stopped, not measurements.
The tail leg flies the final cloud on to ``is_terminal`` so ``E[K | rare set]`` and
``E[A | rare set]`` are observed, which is what lets this backend report ``mean_k`` and
``p_los_ac`` at all.

It rides a **third** child of the replication's seed, and a ``SeedSequence`` child depends only on
its index and its parent — so the splitting is bit-identical whether or not the tail runs. That is
asserted here rather than argued.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from opencdarr.cd import StateBased
from opencdarr.cns.broadcast import schedule_for
from opencdarr.fleet import Agent, build_env
from opencdarr.ips import Particle, estimate_rare_prob, ips_once, replication_seeds
from opencdarr.performance import M600
from opencdarr.rng import generator
from opencdarr.scenario import sample_pairwise

_RPZ, _SHELLS = 50.0, [80.0, 65.0, 50.0]


def _build_initial(n_pairs: int):
    """A particle of ``2 * n_pairs`` aircraft: independent crossing pairs, ~34 km apart."""
    def build(seq: np.random.SeedSequence) -> Particle:
        rng = generator(seq)
        agents: list[Agent] = []
        for shift in [0.0, 0.5][:n_pairs]:
            own, intr = sample_pairwise(rng, speed=10.2889, dcpa_max=100.0, tlos=60.0, rpz=_RPZ)
            agents += [Agent(replace(s, lon=s.lon + shift), M600) for s in (own, intr)]
        env = build_env(agents, rpz=_RPZ, t_lookahead=120.0, dt=1.0, detector=StateBased(),
                        resolver=None, recovery=None, t_max=300.0, done_timeout=10.0,
                        schedule=schedule_for(len(agents), 1.0, rng))
        return Particle(env=env, state=env.initial_state(agents))
    return build


def _estimate(n_pairs: int, *, tail: bool, seed: int = 7, n: int = 80):
    return estimate_rare_prob(_build_initial(n_pairs), _SHELLS,
                              n_particles=n, reps=2, seed=seed, tail=tail)


def test_the_tail_does_not_move_the_ladder() -> None:
    """Running the tail changes no splitting number — it reads its own child of the seed.

    The claim that makes the tail safe to leave on by default: ``p_los_run`` and every per-level
    survival fraction are what they were, so switching it on cannot silently restate a published
    probability.
    """
    on, off = _estimate(2, tail=True), _estimate(2, tail=False)
    assert on.p_los_run == off.p_los_run
    assert [r.survival for r in on.reps] == [r.survival for r in off.reps]
    assert [r.prob for r in on.reps] == [r.prob for r in off.reps]


def test_without_the_tail_the_per_aircraft_metrics_are_absent_not_zero() -> None:
    """``nan``, not 0.0 — "not measured" and "measured as none" are different statements."""
    off = _estimate(2, tail=False)
    assert np.isnan(off.p_los_ac) and np.isnan(off.mean_k)
    assert off.p_los_run > 0.0        # the ladder still reports


def test_at_two_aircraft_the_three_metrics_coincide() -> None:
    """N = 2: one pair, so A is 2 exactly when K is 1 — the per-aircraft rate *is* the per-run one.

    The same invariant the MC side has, reached by a completely different route (a tail mean rather
    than a direct count), which is what makes it a real check on the tail's arithmetic.
    """
    est = _estimate(1, tail=True)
    assert est.p_los_ac == pytest.approx(est.p_los_run, rel=1e-12)
    assert est.mean_k == pytest.approx(est.p_los_run, rel=1e-12)


def test_past_two_aircraft_the_per_aircraft_rate_falls_below_the_per_run_one() -> None:
    """N = 4 as two independent pairs: a loss involves 2 of 4, so the per-run rate saturates above.

    Unreachable without the tail: at the rare boundary every survivor looks like exactly one losing
    pair, so a ladder-only answer would put A = 2 and report the two as equal.
    """
    est = _estimate(2, tail=True)
    assert est.p_los_ac < est.p_los_run
    assert est.mean_k > est.p_los_run      # E[K] counts every losing pair, so it can exceed P(run)
    assert est.n_lineages > 0              # the tail's effective sample size is reported


def test_lineages_come_from_the_draw_not_from_object_identity() -> None:
    """``n_lineages`` counts the *distinct survivors drawn*, so it survives a trip through pickle.

    Clones share one immutable state object. Counting distinct lineages with ``id()`` or ``set()``
    over the cloud therefore gives one answer in-process and another after a worker has pickled it
    — one shared object arriving as several equal copies. Taking the count from the resampling draw
    is what makes serial and parallel agree, so it is pinned as a property of the draw here.
    """
    rep = ips_once(_build_initial(1), _SHELLS, 40, replication_seeds(3, 1)[0], tail=True)
    assert rep.n_lineages is not None
    assert 0 < rep.n_lineages <= rep.n_particles
    # a clone-heavy cloud has fewer lineages than particles; equality would mean no resampling
    assert rep.n_lineages <= 40


def test_the_sharded_parallel_path_reproduces_every_tail_field() -> None:
    """Lockstep sharding is scheduling only — the tail fields included.

    ``reps=3`` over 2 workers is not a whole multiple, so this takes the *sharded* path rather than
    one-replication-per-worker: the cloud crosses a process boundary and comes back, which is
    exactly the trip that broke an identity-counted ``n_lineages``.
    """
    pytest.importorskip("joblib")
    from opencdarr import parallel as par

    seqs = replication_seeds(5, 3)
    serial = [ips_once(_build_initial(2), _SHELLS, 60, s, tail=True) for s in seqs]
    sharded = par.ips_replications(_build_initial(2), _SHELLS, 60, seqs, n_jobs=2, min_shard=1)
    assert serial == sharded  # dataclass equality: every field, tail and lineages included
