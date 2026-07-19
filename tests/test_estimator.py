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
from opencdarr.estimator import estimate_ipr
from opencdarr.performance import M600


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


def test_resolution_raises_ipr_far_above_baseline() -> None:
    cfg = _config()
    resolved = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA())
    baseline = estimate_ipr(cfg, M600, StateBased(), None, None)
    assert resolved.ipr > 0.9  # good CDR prevents nearly all LoS
    assert baseline.ipr < 0.2  # no resolution -> nearly all conflicts become LoS
    assert resolved.ipr > baseline.ipr
