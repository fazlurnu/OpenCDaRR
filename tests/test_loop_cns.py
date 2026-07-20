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

# GpsNavigation reads its noise magnitude from the sampled states' own vel_ci95 (95% radial),
# not a per-axis sigma: 2.4477x a sigma value, so these keep the same effective noise level as
# before the pos-ci95/vel-ci95 rename.
_VEL_CI95_2SIGMA = 2.0 * 2.4477


def _config(seed: int = 1, n: int = 200, pos_ci95: float = 0.0, vel_ci95: float = 0.0) -> Config:
    return Config(
        seed=seed,
        n_encounters=n,
        scenario=ScenarioConfig("M600", 10.2889, 50.0, 60.0, pos_ci95, vel_ci95),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 600.0, 10.0),
    )


def _ipr(pos_ci95: float, vel_ci95: float, navigation: GpsNavigation | None) -> float:
    return estimate_ipr(
        _config(pos_ci95=pos_ci95, vel_ci95=vel_ci95),
        M600, StateBased(), MVP(1.05), PastCPA(), navigation=navigation,
    ).ipr


def test_zero_noise_navigation_matches_no_navigation() -> None:
    """Perfect-accuracy states (ci95=0) perceive the truth -> same clean result as no navigation."""
    assert _ipr(0.0, 0.0, None) == 1.0
    assert _ipr(0.0, 0.0, GpsNavigation()) == 1.0


def test_ipr_degrades_monotonically_with_gps_noise() -> None:
    clean = _ipr(0.0, 0.0, GpsNavigation())
    mild = _ipr(50.0, _VEL_CI95_2SIGMA, GpsNavigation())
    severe = _ipr(200.0, _VEL_CI95_2SIGMA, GpsNavigation())
    assert clean > mild > severe
    assert clean == 1.0
    assert severe < 0.8  # heavy CNS uncertainty -> a clear drop (the over-clear buffer + the
    #                      1 Hz broadcast cadence tolerate noise far better than under-clearing did)


def test_reproducible_with_navigation() -> None:
    cfg = _config(pos_ci95=50.0, vel_ci95=_VEL_CI95_2SIGMA)
    nav = GpsNavigation()
    r1 = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA(), navigation=nav)
    r2 = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA(), navigation=nav)
    assert r1 == r2
