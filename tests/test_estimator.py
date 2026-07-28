"""Functional tests for the plain-MC IPR estimator."""

from __future__ import annotations

from opencdarr.cd import StateBased
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.estimator import combine_ipr, estimate_ipr
from opencdarr.performance import M600
from opencdarr.rng import children, root_seed_sequence


def _config(seed: int = 1, n: int = 200) -> Config:
    return Config(
        seed=seed,
        n_encounters=n,
        scenario=ScenarioConfig("M600", 10.2889, 50.0, 60.0),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 600.0, 10.0),
    )


def test_ipr_is_reproducible() -> None:
    cfg = _config()
    r1 = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA())
    r2 = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA())
    assert r1 == r2


def test_every_sampled_encounter_is_a_conflict() -> None:
    cfg = _config()
    result = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA())
    assert result.n_conflict == cfg.n_encounters


def test_chunked_run_pools_back_to_the_serial_estimate() -> None:
    """Slicing the encounter fan-out and pooling the counts reproduces the whole run exactly.

    This is what makes a parallel MC anchor reproducible: chunks address slices of the *one* seed
    tree the serial run walks. Rooting each chunk at ``seed + i`` instead would give a different
    tree altogether — the pattern this seam exists to replace.
    """
    cfg = _config(n=120)
    whole = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA())
    for jobs in (1, 3, 7):  # 7 divides 120 unevenly, so the bounds are ragged
        bounds = [(120 * i // jobs, 120 * (i + 1) // jobs) for i in range(jobs)]
        pooled = combine_ipr([
            estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA(),
                         seqs=children(root_seed_sequence(cfg.seed), lo, hi))
            for lo, hi in bounds if hi > lo
        ])
        assert pooled == whole


def test_combine_ipr_recomputes_the_ratio_from_pooled_counts() -> None:
    """IPR is a ratio, so chunks pool by counts — not by averaging their per-chunk ratios."""
    from opencdarr.estimator import IPRResult

    pooled = combine_ipr([IPRResult(ipr=0.5, n_conflict=2, n_los=1),
                          IPRResult(ipr=1.0, n_conflict=98, n_los=0)])
    assert pooled.n_conflict == 100
    assert pooled.n_los == 1
    assert pooled.ipr == 0.99  # not (0.5 + 1.0) / 2 = 0.75


def test_resolution_raises_ipr_far_above_baseline() -> None:
    cfg = _config()
    resolved = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA())
    baseline = estimate_ipr(cfg, M600, StateBased(), None, None)
    assert resolved.ipr > 0.9  # good CDR prevents nearly all LoS
    assert baseline.ipr < 0.2  # no resolution -> nearly all conflicts become LoS
    assert resolved.ipr > baseline.ipr
