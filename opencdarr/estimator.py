"""Plain Monte Carlo IPR estimator.

Samples ``config.n_encounters`` independent pairwise encounters and aggregates
``IPR = 1 - n_los/n_conflict``. Each encounter gets its own RNG substream spawned from the
run seed (ADR 0001), so the estimate is reproducible and order-independent — and parallel-ready
(joblib) later. Pure: no I/O.
"""

from __future__ import annotations

from dataclasses import dataclass

from opencdarr.cd.base import ConflictDetector
from opencdarr.config import Config
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.loop import run_encounter
from opencdarr.performance import Performance
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import sample_pairwise


@dataclass(frozen=True)
class IPRResult:
    """The intrusion-prevention rate and the counts behind it."""

    ipr: float
    n_conflict: int
    n_los: int


def estimate_ipr(
    config: Config,
    perf: Performance,
    detector: ConflictDetector,
    resolver: ConflictResolver | None,
    recovery: RecoveryCriterion | None,
) -> IPRResult:
    """Run the plain-MC estimate over ``config.n_encounters`` sampled encounters."""
    n_conflict = 0
    n_los = 0
    for seq in spawn(root_seed_sequence(config.seed), config.n_encounters):
        rng = generator(seq)
        own, intr = sample_pairwise(
            rng,
            speed=config.scenario.speed,
            dcpa_max=config.scenario.dcpa_max,
            tlos=config.scenario.tlos,
            rpz=config.conflict.rpz,
        )
        outcome = run_encounter(
            own,
            intr,
            perf=perf,
            rpz=config.conflict.rpz,
            t_lookahead=config.conflict.t_lookahead,
            dt=config.simulation.dt,
            detector=detector,
            resolver=resolver,
            recovery=recovery,
            t_max=config.simulation.t_max,
            done_timeout=config.simulation.done_timeout,
        )
        if outcome.conflict:
            n_conflict += 1
            if outcome.los:
                n_los += 1

    ipr = 1.0 - n_los / n_conflict if n_conflict else float("nan")
    return IPRResult(ipr=ipr, n_conflict=n_conflict, n_los=n_los)
