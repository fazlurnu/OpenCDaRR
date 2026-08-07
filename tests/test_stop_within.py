"""``stop_within`` threads from the config to **both** backends, or to neither.

The mission-completion stop existed on ``run_fleet`` / ``build_env`` but no config field carried
it, so an experiment could not declare it. These lock the new ``SimulationConfig.stop_within``
end to end — through ``estimate_p_los`` for MC and through ``run_experiment``'s IPS cell — because
a per-run setting that reaches one backend and not the other is this experiment layer's known
failure mode (``montecarlo.py``'s ``kinematics`` story). The probe value is deliberately absurd
(1e6 m): every goal-carrying aircraft is "arrived" before its first step, so an honored stop is
unmistakable — encounters end at t = 0 with ``min_sep`` still infinite — and an ignored one flies
the ring as usual.
"""

from __future__ import annotations

import math

from opencdarr.autopilot import WaypointAutopilot
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
from opencdarr.estimate.montecarlo import estimate_p_los
from opencdarr.experiment import IPS, Fixed, Methods, run_experiment
from opencdarr.fleet import Agent
from opencdarr.mission import Mission
from opencdarr.performance import M600
from opencdarr.scenario import CrossingRing

RPZ = 50.0
RING = CrossingRing(n=3, radius=300.0)


def _config(stop_within: float | None, n: int = 6) -> Config:
    return Config(
        seed=0, n_encounters=n,
        scenario=ScenarioConfig("M600", 10.2889, RPZ, 40.0, 0.0, 0.0),
        conflict=ConflictConfig(RPZ, 60.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(0.5, 80.0, 5.0, stop_within=stop_within),
    )


def _build(rng, config):
    """The ring with its goals attached — the wiring ``_agents_from`` does for a fleet."""
    agents = []
    for state, goal in RING.draw(rng, config):
        autopilot = (
            None if goal is None
            else WaypointAutopilot(Mission(goto=goal), cruise_airspeed=state.gs)
        )
        agents.append(Agent(state, M600, autopilot=autopilot))
    return agents


def test_mc_honours_declared_stop_within() -> None:
    """An absurd radius ends every encounter before its first step; ``None`` flies the ring."""
    stopped = estimate_p_los(_build, _config(1e6), StateBased(), MVP(1.05),
                             PastCPA(bouncing_guard=True))
    assert all(math.isinf(s) for s in stopped.min_seps)

    flown = estimate_p_los(_build, _config(None), StateBased(), MVP(1.05),
                           PastCPA(bouncing_guard=True))
    assert all(math.isfinite(s) for s in flown.min_seps)


def test_ips_cell_honours_declared_stop_within() -> None:
    """Through ``run_experiment``: stopped-at-birth particles never leave ``min_sep = inf``, so a
    one-shell ladder collapses every replication — the unmistakable signature that the IPS
    ``build_initial`` passed the config's value into its env."""
    methods = Methods(
        detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(bouncing_guard=True),
        perf=M600, scenario=RING,
    )
    cell = run_experiment(
        {"pos_ci95": Fixed(0.0), "vel_ci95": Fixed(0.0)},
        methods=methods, backend=IPS(shells=[RPZ], n_particles=8, reps=2),
        base_config=_config(1e6, n=8), seed=0, n_jobs=1,
    ).cell()
    assert cell.n_collapsed == 2
    assert cell.p_los_run == 0.0
