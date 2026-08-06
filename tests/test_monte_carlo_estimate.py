"""MonteCarloEstimate — the per-aircraft metric fields on the plain-MC estimate (ADR 0022).

``estimate_p_los`` is pairwise (N = 2), so here the three normalisations coincide: ``p_los_ac``,
``p_los_run`` and ``mean_k`` are one number, and ``p_los_run`` reproduces the old ``p_los``.
The N > 2 *divergence* is a fleet property, checked in ``test_per_aircraft_normalisation``.
What this file locks is the N = 2 identity (so no pairwise result moves) and the new counters that
carry K and A up from the outcome.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from opencdarr.cd import StateBased
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.estimator import MonteCarloEstimate, combine_p_los, estimate_p_los
from opencdarr.fleet import Agent
from opencdarr.performance import M600
from opencdarr.scenario import pairwise, sample_pairwise


def _config(seed: int = 1, n: int = 200) -> Config:
    # dcpa_max = 2 * rpz: with no resolver about half the encounters lose separation
    return Config(
        seed=seed, n_encounters=n,
        scenario=ScenarioConfig("M600", 10.2889, 100.0, 60.0),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 600.0, 10.0),
    )


def _estimate(seed: int = 1, n: int = 200) -> MonteCarloEstimate:
    """A pairwise estimate, no resolver — a moderate LoS rate so the counters are non-trivial."""
    return estimate_p_los(pairwise(M600), _config(seed, n), StateBased(), None, None)


def test_pairwise_three_metrics_coincide_and_reproduce_p_los() -> None:
    """N = 2: p_los_ac, p_los_run and mean_k are one number, and p_los_run is the old p_los."""
    est = _estimate()
    assert est.sum_n == 2 * est.n_encounters                 # two aircraft flew per encounter
    assert est.p_los_ac == est.p_los_run == est.mean_k       # one number at N = 2
    assert est.p_los_run == est.n_los / est.n_encounters     # == the old p_los
    assert 0 < est.n_los < est.n_encounters                  # non-degenerate: a real mix
    assert est.sum_k == est.n_los                            # one pair: K = 1{los}
    assert est.sum_a == 2 * est.n_los                        # one pair: A = 2 * 1{los}


def test_bare_p_los_and_ipr_are_gone() -> None:
    """The ambiguous names are dropped: p_los -> p_los_run / p_los_ac, and ipr (1 - P) removed."""
    est = _estimate()
    assert not hasattr(est, "p_los")
    assert not hasattr(est, "ipr")


def test_combine_sums_the_new_counters() -> None:
    """combine_p_los pools K and A the way it pools n_los — summed, then the rates recomputed."""
    a, b = _estimate(seed=1, n=120), _estimate(seed=2, n=120)
    c = combine_p_los([a, b])
    assert c.sum_k == a.sum_k + b.sum_k
    assert c.sum_a == a.sum_a + b.sum_a
    assert c.sum_n == a.sum_n + b.sum_n
    assert c.mean_k == c.sum_k / c.n_encounters
    assert c.p_los_ac == c.sum_a / c.sum_n


def _two_pairs(rng: np.random.Generator, config: Config) -> list[Agent]:
    """Four aircraft as two independent pairs — an encounter builder that is not pairwise."""
    agents: list[Agent] = []
    for shift in (0.0, 0.5):  # the second pair sits ~34 km east: the two never interact
        own, intr = sample_pairwise(
            rng, speed=config.scenario.speed, dcpa_max=config.scenario.dcpa_max,
            tlos=config.scenario.tlos, rpz=config.conflict.rpz,
        )
        agents += [Agent(replace(s, lon=s.lon + shift), M600) for s in (own, intr)]
    return agents


def test_the_builder_sets_n_so_the_estimator_is_not_pairwise_bound() -> None:
    """A four-aircraft builder runs through the same estimator, and N follows the builder.

    Nothing in the loop or the aggregation knows about 2 — ``N`` is ``len(agents)``, so the
    per-aircraft denominator is the aircraft that actually flew. With two independent pairs a loss
    involves 2 of 4, so ``p_los_ac`` sits strictly below ``p_los_run``: the per-run rate counts a
    run whole, the per-aircraft rate counts the fraction. That gap is the metric's whole point, and
    it is unreachable while the estimator can only build pairwise encounters.
    """
    est = estimate_p_los(_two_pairs, _config(n=60), StateBased(), None, None)
    assert est.sum_n == 4 * est.n_encounters      # four aircraft flew per encounter
    assert 0.0 < est.p_los_ac < est.p_los_run     # per-run saturates above per-aircraft
    assert est.mean_k == est.sum_k / est.n_encounters
