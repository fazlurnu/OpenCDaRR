"""Plain Monte Carlo IPR estimator.

Samples ``config.n_encounters`` independent pairwise encounters and aggregates
``IPR = 1 - n_los/n_conflict``. Each encounter gets its own RNG substream spawned from the
run seed (ADR 0001), so the estimate is reproducible and order-independent — which is what lets a
caller hand slices of the encounter fan-out to different processes (``seqs=``) and pool the counts
with :func:`combine_ipr` for exactly the serial answer. Pure: no I/O.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import CommunicationModel, NavigationModel, SurveillanceModel
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


def combine_ipr(results: Sequence[IPRResult]) -> IPRResult:
    """Pool chunked runs into the result a single serial run over the same encounters would give.

    IPR is a ratio, so it has to be recomputed from the pooled counts — averaging the per-chunk
    ratios would weight a chunk that detected few conflicts as heavily as one that detected many.
    """
    n_conflict = sum(r.n_conflict for r in results)
    n_los = sum(r.n_los for r in results)
    return IPRResult(
        ipr=1.0 - n_los / n_conflict if n_conflict else float("nan"),
        n_conflict=n_conflict,
        n_los=n_los,
    )


def estimate_ipr(
    config: Config,
    perf: Performance,
    detector: ConflictDetector,
    resolver: ConflictResolver | None,
    recovery: RecoveryCriterion | None,
    navigation: NavigationModel | None = None,
    communication: CommunicationModel | None = None,
    surveillance: SurveillanceModel | None = None,
    *,
    seqs: Sequence[np.random.SeedSequence] | None = None,
) -> IPRResult:
    """Run the plain-MC estimate over ``config.n_encounters`` sampled encounters.

    ``seqs`` overrides which per-encounter substreams to run, defaulting to the whole fan-out
    ``spawn(root_seed_sequence(config.seed), config.n_encounters)``. It exists so a caller can run
    *contiguous slices* of that same fan-out in parallel — ``children(root, lo, hi)``,
    :mod:`opencdarr.rng` — and pool them with :func:`combine_ipr` for a result bit-identical to the
    serial run. That is the reproducible way to chunk; offsetting the seed per chunk (``seed + i``)
    is not, because those trees can correlate and their union is not the serial run's tree at all.
    """
    n_conflict = 0
    n_los = 0
    encounters = (
        spawn(root_seed_sequence(config.seed), config.n_encounters) if seqs is None else seqs
    )
    for seq in encounters:
        # always 3 substreams (geometry, navigation, communication), regardless of which CNS
        # layers are enabled for this run — the stream tree stays config-invariant (ADR 0006 §6)
        geom_seq, nav_seq, comm_seq = spawn(seq, 3)
        own, intr = sample_pairwise(
            generator(geom_seq),
            speed=config.scenario.speed,
            dcpa_max=config.scenario.dcpa_max,
            tlos=config.scenario.tlos,
            rpz=config.conflict.rpz,
            pos_ci95=config.scenario.pos_ci95,
            vel_ci95=config.scenario.vel_ci95,
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
            navigation=navigation,
            rng=generator(nav_seq),
            communication=communication,
            surveillance=surveillance,
            comm_rng=generator(comm_seq),
            t_max=config.simulation.t_max,
            done_timeout=config.simulation.done_timeout,
            broadcast_interval=config.simulation.broadcast_interval,
        )
        if outcome.conflict:
            n_conflict += 1
            if outcome.los:
                n_los += 1

    ipr = 1.0 - n_los / n_conflict if n_conflict else float("nan")
    return IPRResult(ipr=ipr, n_conflict=n_conflict, n_los=n_los)
