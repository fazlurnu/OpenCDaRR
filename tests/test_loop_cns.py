"""Functional tests for CNS navigation in the encounter loop / estimator (3a)."""

from __future__ import annotations

from opencdarr.cd import StateBased
from opencdarr.cns import GpsNavigation
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


def _ipr(nav: GpsNavigation | None) -> float:
    return estimate_ipr(
        _config(), M600, StateBased(), MVP(1.05), PastCPA(), navigation=nav
    ).ipr


def test_zero_noise_navigation_matches_no_navigation() -> None:
    """GpsNavigation(0, 0) perceives the truth -> same clean result as no navigation."""
    assert _ipr(None) == 1.0
    assert _ipr(GpsNavigation(0.0, 0.0)) == 1.0


def test_ipr_degrades_monotonically_with_gps_noise() -> None:
    clean = _ipr(GpsNavigation(0.0, 0.0))
    mild = _ipr(GpsNavigation(50.0, 2.0))
    severe = _ipr(GpsNavigation(200.0, 2.0))
    assert clean > mild > severe
    assert clean == 1.0
    assert severe < 0.8  # heavy CNS uncertainty -> a clear drop (the over-clear buffer + the
    #                      1 Hz broadcast cadence tolerate noise far better than under-clearing did)


def test_reproducible_with_navigation() -> None:
    nav = GpsNavigation(50.0, 2.0)
    r1 = estimate_ipr(_config(), M600, StateBased(), MVP(1.05), PastCPA(), navigation=nav)
    r2 = estimate_ipr(_config(), M600, StateBased(), MVP(1.05), PastCPA(), navigation=nav)
    assert r1 == r2
