"""Run configuration — YAML to a typed, validated `Config` (nested dataclasses).

One place that turns a file into typed config, validated on load (fail fast). Mirrors the old
``sim_config.json`` fields. The ``config + seed -> result`` contract (``design-philosophy.md``
#4) starts here; consumed by ``experiment.run_one_experiment``.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class ScenarioConfig:
    """The encounter distribution to sample."""

    aircraft_type: str  # e.g. "M600"
    speed: float  # ground speed [m/s]
    dcpa_max: float  # miss distance sampled in [0, dcpa_max] [m]
    tlos: float  # time to loss of separation [s]
    pos_ci95: float = 0.0  # actual position measurement accuracy, 95% radial [m] (0 = perfect)
    vel_ci95: float = 0.0  # actual velocity measurement accuracy, 95% radial [m/s] (0 = perfect)
    # what the broadcast claims instead; None (default) = claim the truth. Sweep against the two
    # above to study an over- or under-confident declaration (AircraftState's docstring).
    pos_ci95_declared: float | None = None
    vel_ci95_declared: float | None = None


@dataclass(frozen=True)
class ConflictConfig:
    rpz: float  # protected-zone radius [m]
    t_lookahead: float  # detection lookahead [s]


@dataclass(frozen=True)
class MethodsConfig:
    detection: str  # detector name, e.g. "statebased"
    resolution: str | None  # resolver name, e.g. "mvp", or null for the baseline
    recovery: str | None  # recovery name, e.g. "pastcpa", or null
    margin: float  # MVP resolution-zone margin (>= 1)
    bouncing_guard: bool  # Past-CPA bouncing guard


@dataclass(frozen=True)
class SimulationConfig:
    dt: float  # integration step [s]
    t_max: float  # max encounter time [s]
    done_timeout: float  # sustained-divergence time to terminate [s]
    broadcast_interval: float = 1.0  # CDR/measurement cadence [s] (ADS-L rate); held between ticks
    broadcast_jitter: float = 0.0  # per-transmission slot dither U(-j, +j) [s]; 0 = fixed gaps
    broadcast_random_phase: bool = False  # draw each aircraft's start offset in [0, interval)


@dataclass(frozen=True)
class Config:
    seed: int
    n_encounters: int
    scenario: ScenarioConfig
    conflict: ConflictConfig
    methods: MethodsConfig
    simulation: SimulationConfig


def load_config(path: str | Path) -> Config:
    """Load and validate a run configuration from a YAML file."""
    raw = yaml.safe_load(Path(path).read_text())
    try:
        cfg = Config(
            seed=int(raw["seed"]),
            n_encounters=int(raw["n_encounters"]),
            scenario=ScenarioConfig(**raw["scenario"]),
            conflict=ConflictConfig(**raw["conflict"]),
            methods=MethodsConfig(**raw["methods"]),
            simulation=SimulationConfig(**raw["simulation"]),
        )
    except (KeyError, TypeError) as exc:
        raise ValueError(f"invalid config {path}: {exc}") from exc
    _validate(cfg)
    return cfg


def _validate(cfg: Config) -> None:
    checks = {
        "seed >= 0": cfg.seed >= 0,
        "n_encounters > 0": cfg.n_encounters > 0,
        "scenario.speed > 0": cfg.scenario.speed > 0,
        "scenario.dcpa_max >= 0": cfg.scenario.dcpa_max >= 0,
        # every sampled encounter must be a genuine conflict, so that P(LoS)'s denominator (the
        # encounter count) needs no filtering: create_conflict only breaches the protected zone
        # when the miss distance is inside it, so dcpa drawn from U(0, dcpa_max) must stay within
        # rpz. Above it, a fraction of encounters would silently be non-conflicts and P(LoS) would
        # be reported over a mixed population (opencdarr.estimator.MonteCarloEstimate).
        "scenario.dcpa_max <= conflict.rpz": cfg.scenario.dcpa_max <= cfg.conflict.rpz,
        "scenario.tlos > 0": cfg.scenario.tlos > 0,
        "scenario.pos_ci95 >= 0": cfg.scenario.pos_ci95 >= 0,
        "scenario.vel_ci95 >= 0": cfg.scenario.vel_ci95 >= 0,
        "conflict.rpz > 0": cfg.conflict.rpz > 0,
        "conflict.t_lookahead > 0": cfg.conflict.t_lookahead > 0,
        "methods.margin >= 1": cfg.methods.margin >= 1.0,
        "simulation.dt > 0": cfg.simulation.dt > 0,
        "simulation.t_max > 0": cfg.simulation.t_max > 0,
        "simulation.done_timeout >= 0": cfg.simulation.done_timeout >= 0,
        "simulation.broadcast_interval >= dt": (
            cfg.simulation.broadcast_interval >= cfg.simulation.dt
        ),
        "simulation.broadcast_jitter >= 0": cfg.simulation.broadcast_jitter >= 0,
        # a gap of interval + U(-j, +j) must stay positive, so the dither cannot reach the period.
        # BroadcastSchedule enforces this too; failing here reports it in the config's vocabulary,
        # at load time, rather than part-way into a run.
        "simulation.broadcast_jitter < broadcast_interval": (
            cfg.simulation.broadcast_jitter < cfg.simulation.broadcast_interval
        ),
    }
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        raise ValueError(f"config constraints violated: {'; '.join(failed)}")
