"""Functional tests for the plain-MC IPR estimator."""

from __future__ import annotations

import dataclasses

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.dynamics import Dynamics, MotionCommand
from opencdarr.dynamics.base import odometry_update
from opencdarr.estimator import combine_ipr, estimate_ipr
from opencdarr.performance import M600, Performance
from opencdarr.rng import children, root_seed_sequence
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField


def _config(seed: int = 1, n: int = 200) -> Config:
    return Config(
        seed=seed,
        n_encounters=n,
        scenario=ScenarioConfig("M600", 10.2889, 50.0, 60.0),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 600.0, 10.0),
    )


def _noisy_config(seed: int = 1, n: int = 200) -> Config:
    """``_config`` at a GNSS noise level where LoS is common but far from certain.

    Picked deliberately mid-range (22/200 LoS): a golden anchor at 0/200 or 200/200 would agree
    with almost any broken refactor, so it would not be an anchor at all.
    """
    cfg = _config(seed, n)
    return dataclasses.replace(
        cfg, scenario=dataclasses.replace(cfg.scenario, pos_ci95=60.0, vel_ci95=6.0)
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


def test_golden_ipr_at_midrange_noise() -> None:
    """**Golden anchor.** Exact counts for a fixed seed, config and noise level.

    Every other test here pins a *relation* (reproducible, pooled, above baseline), all of which a
    refactor can satisfy while silently changing the numbers. This pins the numbers themselves, so
    that moving the estimator onto a different runner — as Phase-v1 did, from ``run_encounter`` to
    ``run_fleet`` at n=2 — has to prove it changed nothing. Update these values only alongside a
    deliberate, recorded modelling change.
    """
    result = estimate_ipr(
        _noisy_config(), M600, StateBased(), MVP(1.05), PastCPA(bouncing_guard=False),
        GnssNavigation(),
    )
    assert (result.n_los, result.n_conflict) == (22, 200)
    assert result.ipr == 0.89


class _Ballistic(Dynamics):
    """A dynamics that ignores every command and coasts on the current track.

    Deliberately incapable of avoidance: with it fitted, no resolver can prevent a loss of
    separation, whatever it commands. That makes it a *detector* for whether ``dynamics=`` actually
    reaches the encounter — see :func:`test_dynamics_reaches_the_mc_path`.
    """

    def step(
        self,
        state: AircraftState,
        command: MotionCommand,
        perf: Performance,
        dt: float,
        wind: WindField = NO_WIND,
    ) -> AircraftState:
        lat, lon = geo.forward(state.lat, state.lon, state.trk, state.gs * dt)
        return dataclasses.replace(
            state, lat=lat, lon=lon, **odometry_update(state, state.gs, dt)
        )


def test_dynamics_reaches_the_mc_path() -> None:
    """A contributed ``Dynamics`` must actually fly the MC encounters, not be silently dropped.

    Plain MC used to call ``run_encounter`` without forwarding ``dynamics``, so a custom airframe
    was ignored here while IPS (which builds its own ``FleetEnv``) honoured it — the same model
    giving different answers on the two backends, with nothing in either result to show why.
    Fitting an airframe that *cannot* manoeuvre is the cheapest way to prove the wiring.

    The sharp form of the claim is an **exact** one: a resolver whose commands are thrown away by
    the airframe must give bit-for-bit what flying with *no resolver at all* gives, because both
    hold the initial cruise. So ``ballistic + MVP == multirotor + None``, while the multirotor
    actually fitted with MVP resolves every one of the same encounters. If ``dynamics=`` were
    dropped again, the first equality is what breaks.
    """
    cfg = _config()
    fitted = estimate_ipr(
        cfg, M600, StateBased(), MVP(1.05), PastCPA(), dynamics=_Ballistic()
    )
    unresolved = estimate_ipr(cfg, M600, StateBased(), None, None)
    resolved = estimate_ipr(cfg, M600, StateBased(), MVP(1.05), PastCPA())

    assert fitted == unresolved  # the airframe discarded every resolution command
    assert resolved.n_los == 0  # the same conflicts, resolved, on the default airframe
    assert fitted.ipr < resolved.ipr
