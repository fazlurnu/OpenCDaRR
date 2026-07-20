"""Functional tests for CNS in the encounter loop / estimator — navigation (3a) and
communication (3b)."""

from __future__ import annotations

import pytest

from opencdarr.cd import StateBased
from opencdarr.cns import Comm, GpsNavigation, uniform_latency
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
from opencdarr.loop import run_encounter
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import sample_pairwise

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
    """Perfect-accuracy states (ci95=0) perceive the truth -> same clean result as no
    navigation."""
    assert _ipr(0.0, 0.0, None) == 1.0
    assert _ipr(0.0, 0.0, GpsNavigation()) == 1.0


def test_ipr_degrades_monotonically_with_gps_noise() -> None:
    clean = _ipr(0.0, 0.0, GpsNavigation())
    mild = _ipr(50.0, _VEL_CI95_2SIGMA, GpsNavigation())
    severe = _ipr(200.0, _VEL_CI95_2SIGMA, GpsNavigation())
    assert clean > mild > severe
    assert clean == 1.0
    assert severe < 0.8  # heavy CNS uncertainty -> a clear drop (the over-clear buffer + the
    #                      1 Hz broadcast cadence tolerate noise far better than under-clearing
    #                      did)


def test_reproducible_with_navigation() -> None:
    cfg = _config(pos_ci95=50.0, vel_ci95=_VEL_CI95_2SIGMA)
    nav = GpsNavigation()
    r1 = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA(), navigation=nav)
    r2 = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA(), navigation=nav)
    assert r1 == r2


# --- 3b: communication (reception + latency), loop-integration tests --------------------------


def _ipr_comm(communication) -> float:
    """No navigation noise, so communication is the only stochastic driver of the outcome."""
    return estimate_ipr(
        _config(), M600, StateBased(), MVP(1.05), PastCPA(), communication=communication
    ).ipr


def test_perfect_communication_matches_no_communication() -> None:
    """p=1, latency=0 reduces exactly to the no-communication path (ADR 0006 / plan exit gate)."""
    baseline = _ipr_comm(None)
    perfect = _ipr_comm(Comm(reception_prob=1.0, latency=0.0))
    assert perfect == baseline == 1.0


def test_ipr_degrades_monotonically_with_reception_loss() -> None:
    """Reception needs to drop quite low before it bites: the encounter window (60s tlos) is
    long relative to the 1 Hz broadcast interval, so a few dropped ticks rarely delay first
    contact past CPA — calibrated empirically, mirroring the GPS-noise monotonicity test."""
    clean = _ipr_comm(Comm(reception_prob=1.0, latency=0.0))
    mild = _ipr_comm(Comm(reception_prob=0.03, latency=0.0))
    severe = _ipr_comm(Comm(reception_prob=0.005, latency=0.0))
    assert clean > mild > severe
    assert clean == 1.0
    assert severe < 0.6


def test_ipr_degrades_with_latency_too() -> None:
    """Perfect reception but latency that regularly exceeds the broadcast interval still delays
    first contact (and every later update), so IPR should drop below the zero-latency case."""
    no_latency = _ipr_comm(Comm(reception_prob=1.0, latency=0.0))
    laggy = _ipr_comm(Comm(reception_prob=1.0, latency=uniform_latency(0.0, 30.0)))
    assert laggy < no_latency


def test_comm_rng_required_when_communication_set() -> None:
    own, intr = sample_pairwise(
        generator(root_seed_sequence(0)), speed=10.2889, dcpa_max=20.0, tlos=60.0, rpz=50.0
    )
    with pytest.raises(ValueError):
        run_encounter(
            own, intr, perf=M600, rpz=50.0, t_lookahead=120.0, dt=1.0,
            detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(),
            communication=Comm(reception_prob=0.5), comm_rng=None,
        )


def test_reproducible_with_communication() -> None:
    cfg = _config()
    comm = Comm(reception_prob=0.02, latency=uniform_latency(0.0, 5.0))
    r1 = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA(), communication=comm)
    r2 = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA(), communication=comm)
    assert r1 == r2


def test_communication_and_navigation_are_independent_substreams() -> None:
    """Adding a comm layer must not change navigation-only results (ADR 0006 §6: nav_seq and
    comm_seq are independent substreams; drawing from one must not perturb the other)."""
    seq = root_seed_sequence(0)
    (encounter_seq,) = spawn(seq, 1)
    geom_seq, nav_seq, comm_seq = spawn(encounter_seq, 3)
    own, intr = sample_pairwise(
        generator(geom_seq), speed=10.2889, dcpa_max=20.0, tlos=60.0, rpz=50.0
    )
    kwargs = dict(
        perf=M600, rpz=50.0, t_lookahead=120.0, dt=1.0,
        detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(),
        navigation=GpsNavigation(), broadcast_interval=1.0,
    )
    without_comm = run_encounter(own, intr, rng=generator(nav_seq), **kwargs)
    with_comm = run_encounter(
        own, intr, rng=generator(nav_seq),
        communication=Comm(reception_prob=0.9), comm_rng=generator(comm_seq),
        **kwargs,
    )
    assert without_comm == with_comm  # same nav_seq draws -> identical outcome regardless of comm
