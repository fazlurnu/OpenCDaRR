"""Fast mechanics locks for the IPS estimator (``opencdarr/ips.py``, ADR 0017).

These do **not** re-run the slow IPS-vs-MC validation — that is ``scripts/ips_validate.py`` and the
[[ips-gate1-correctness]] / [[ips-gate2-efficiency]] observations. They lock the cheap invariants:
the estimate is the product of survival fractions, an unreachable shell collapses (``prob=0``,
``collapsed_at`` set), a single level does no resampling, runs are reproducible from seed, the
serial and parallel paths agree, and the replication combiner behaves. Small particle counts and a
short encounter keep each test well under a second.
"""

from __future__ import annotations

import math

import pytest

from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, build_env
from opencdarr.ips import (
    IPSResult,
    Particle,
    combine_replications,
    estimate_rare_prob,
    ips_once,
    replication_seeds,
)
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

RPZ = 50.0


def _make_build(pos: float):
    """A 90-deg conflict-pair factory at a chosen GNSS noise; short encounter, so it runs fast.

    The seed feeds only the forward evolution (the geometry is fixed) — exactly how IPS uses
    ``build_initial`` for a pinned geometry. High noise gives particle diversity (deep shells
    reachable); low noise makes the resolver reliable (deep shells unreachable, for the collapse
    test).
    """
    def build(seq) -> Particle:
        own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.2889,
                            pos_ci95=pos, vel_ci95=pos / 10.0)
        intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=40.0, rpz=RPZ, side=1)
        agents = [Agent(own, M600), Agent(intr, M600)]
        env = build_env(agents, rpz=RPZ, t_lookahead=60.0, dt=0.5, detector=StateBased(),
                        resolver=MVP(margin=1.05), recovery=PastCPA(bouncing_guard=True),
                        navigation=GnssNavigation(), t_max=80.0, done_timeout=5.0)
        return Particle(env=env, state=env.initial_state(agents))
    return build


_build_initial = _make_build(80.0)  # high noise -> diverse particles, healthy survival


def _one(levels: list[float], n: int = 40, seed: int = 0) -> IPSResult:
    return ips_once(_build_initial, levels, n, root_seed_sequence(seed))


def test_prob_is_product_of_survival() -> None:
    """The estimate is exactly the product of the per-level survival fractions (ADR 0017 §2)."""
    r = _one([60.0, 55.0, 52.0, 50.0], n=40)
    assert r.collapsed_at is None  # high-noise ladder should not collapse at n=40
    assert r.prob == pytest.approx(math.prod(r.survival))
    assert 0.0 <= r.prob <= 1.0
    assert len(r.survival) == 4
    assert all(0.0 < s <= 1.0 for s in r.survival)


def test_single_level_does_no_resampling() -> None:
    """One shell at rpz: no resampling, so the estimate is a single binomial survival k/n."""
    r = _one([RPZ], n=40)
    assert r.collapsed_at is None
    assert len(r.survival) == 1
    assert r.prob == r.survival[0]
    assert (r.prob * 40) == pytest.approx(round(r.prob * 40))  # integer survivor count


def test_unreachable_shell_collapses() -> None:
    """A shell far below what a *working* resolver ever allows -> zero survivors -> a flagged
    collapse, reported as prob=0 with collapsed_at set (ADR 0017 §2: not a valid zero)."""
    # low noise: the resolver clears this 90-deg crossing to ~margin*rpz, so 40 m is never reached
    r = ips_once(_make_build(2.0), [40.0], 20, root_seed_sequence(0))
    assert r.collapsed_at == 0
    assert r.prob == 0.0


def test_reusing_a_seed_object_is_rejected() -> None:
    """``seq`` is consumed: a second run on the same object would walk a different stream tree.

    Left unguarded this is silent — same call, same seed, different answer — so it is refused with
    a message pointing at ``replication_seeds``. Re-running a replication means a *fresh* sequence.
    """
    seq = root_seed_sequence(4)
    ips_once(_build_initial, [60.0, 55.0], 20, seq)
    with pytest.raises(ValueError, match="not been spawned from"):
        ips_once(_build_initial, [60.0, 55.0], 20, seq)


def test_deterministic_from_seed() -> None:
    """Same seed -> identical estimate and survival vector (reproducibility, ADR 0001)."""
    a = _one([60.0, 55.0, 50.0], n=40, seed=7)
    b = _one([60.0, 55.0, 50.0], n=40, seed=7)
    assert a.prob == b.prob
    assert a.survival == b.survival


def test_different_seed_differs() -> None:
    """Different seeds give independent runs (not a frozen constant)."""
    a = _one([60.0, 55.0, 50.0], n=40, seed=1)
    b = _one([60.0, 55.0, 50.0], n=40, seed=2)
    assert a.survival != b.survival


def test_estimate_rare_prob_equals_manual_parallel_combine() -> None:
    """estimate_rare_prob == combine_replications over replication_seeds — the serial/parallel
    equivalence the module promises (a caller may parallelise ips_once and get the same result)."""
    levels = [60.0, 55.0, 52.0, 50.0]
    est = estimate_rare_prob(_build_initial, levels, n_particles=24, reps=3, seed=3)
    manual = combine_replications(
        [ips_once(_build_initial, levels, 24, s) for s in replication_seeds(3, 3)]
    )
    assert est.prob == manual.prob
    assert est.ci == manual.ci
    assert est.n_collapsed == manual.n_collapsed
    assert len(est.reps) == 3


def _fake(prob: float, collapsed_at: int | None = None) -> IPSResult:
    return IPSResult(prob=prob, levels=(50.0,), survival=(prob,), n_particles=10,
                     collapsed_at=collapsed_at)


def test_combine_mean_and_log_ci() -> None:
    """combine_replications reports the arithmetic mean (unbiased point) and a positive log CI."""
    c = combine_replications([_fake(0.01), _fake(0.02), _fake(0.03), _fake(0.04)])
    assert c.prob == pytest.approx(0.025)
    assert c.n_collapsed == 0
    lo, hi = c.ci
    assert 0.0 < lo < hi


def test_combine_counts_collapses_and_falls_back_to_span() -> None:
    """A collapsed replication (prob 0) is counted, and the CI falls back to the min/max span
    because a zero has no logarithm (ADR 0017 §5)."""
    c = combine_replications([_fake(0.02), _fake(0.0, collapsed_at=1), _fake(0.03)])
    assert c.n_collapsed == 1
    assert c.prob == pytest.approx((0.02 + 0.0 + 0.03) / 3)
    assert c.ci == (0.0, 0.03)


def test_replication_seeds_deterministic_and_distinct() -> None:
    """replication_seeds is reproducible and yields distinct independent substreams."""
    a = replication_seeds(5, 3)
    b = replication_seeds(5, 3)
    assert len(a) == 3
    draws_a = [int(generator(s).integers(0, 1_000_000)) for s in a]
    draws_b = [int(generator(s).integers(0, 1_000_000)) for s in b]
    assert draws_a == draws_b           # reproducible
    assert len(set(draws_a)) == 3       # distinct substreams
