"""Parallel execution of the rare-event estimator — scheduling only, no new statistics.

:mod:`opencdarr.ips` stays the serial reference: readable, numpy-only, and the thing this module
is validated against. What lives here is the *scheduling* half of ``docs/roadmap.md``'s "batching
across particles, then joblib across CPUs" — how the same work is spread over cores, never what
the work is. Every function here returns results **bit-identical** to its serial twin
(``tests/test_parallel.py``), so switching to it is a performance decision and nothing else.

**Why it exists.** Callers used to parallelise by handing one whole replication to each worker
(``Parallel(delayed(ips_once)) for s in replication_seeds(...)``), which caps usable cores at
``reps``. A production sweep cell running ``reps=10`` on a 96-core box left ~89 % of the machine
idle for the 918 s the IPS phase took. Replication count is a *statistical* choice (ADR 0017 §5) —
and it should not also decide how much of the machine gets used. This module decouples the two.

**How.** Two modes, picked from the shape of the design:

- **whole replications**, when ``reps`` is an exact multiple of the worker count. Nothing crosses
  a process boundary except the seed and the result, so this is the zero-overhead best case. The
  multiple matters: at ``reps=100`` on 96 workers, four workers would run a second replication
  while 92 sat idle, roughly doubling the wall time — so that case takes the sharded path instead.
- **lockstep**, otherwise. Every live replication advances one shell at a time *together*, and each
  level's particles are split into contiguous shards spread across all workers. Bit-identity comes
  from the seed tree already being index-addressed (:func:`opencdarr.rng.children`): particle *i*
  gets stream *i* whatever shard it lands in, and joblib returns results in submission order, so
  re-concatenating the shards reproduces the serial list exactly.

**Determinism caveat.** Identical on the same machine and numpy build — the same promise the serial
path already makes. Nothing here reorders a floating-point reduction.

**Using it.** Same arguments as the serial estimator plus ``n_jobs`` — ``-1`` means every core::

    from opencdarr.parallel import estimate_rare_prob

    est = estimate_rare_prob(
        build_initial,          # (SeedSequence) -> Particle; see opencdarr.ips.BuildInitial
        levels=[150, 120, 100, 85, 75, 68, 62, 58, 55, 52, 50],   # decreasing, ends at rpz
        n_particles=10_000,     # per shell
        reps=10,                # independent replications
        seed=20260728,
        n_jobs=-1,              # every core, whatever `reps` happens to be
    )
    print(est.p_los_run, est.p_los_ac, est.n_collapsed)

``build_initial`` should return a *shared* particle when the geometry is pinned — build the env
once outside it. Pass ``verbose=10`` to watch joblib's progress on a long run.

joblib is an optional dependency (``pip install 'opencdarr[parallel]'``), imported on use: this
module must import cleanly without it, and only fails when actually asked for more than one worker.
Export ``OMP_NUM_THREADS=1`` (and the OpenBLAS/MKL equivalents) before a large run so N workers do
not each start their own thread pool; the hot path makes no BLAS calls, so this is about not
oversubscribing the machine rather than about correctness.
"""

from __future__ import annotations

import os
from collections.abc import Sequence
from typing import Any

import numpy as np

from opencdarr.fleet import FleetState
from opencdarr.ips import (
    BuildInitial,
    IPSResult,
    Particle,
    RareEventEstimate,
    _evolve_to_terminal,
    _streams,
    combine_replications,
    evolve_shard,
    ips_once,
    replication_seeds,
    resample_level,
)
from opencdarr.rng import child, children


def resolve_jobs(n_jobs: int) -> int:
    """Turn joblib's ``n_jobs`` convention into a concrete worker count.

    ``-1`` is every core, ``-2`` all but one, and so on; ``0`` is an error, as in joblib. Resolved
    up front because the scheduler has to know how wide the machine is *before* it can decide how
    to partition the work — unlike :class:`joblib.Parallel`, which only needs it at dispatch.
    """
    if n_jobs == 0:
        raise ValueError("n_jobs must be non-zero (-1 means all cores)")
    if n_jobs > 0:
        return n_jobs
    try:  # joblib's count honours cgroup limits and LOKY_MAX_CPU_COUNT; os.cpu_count does not
        from joblib import cpu_count

        total = int(cpu_count())
    except ImportError:
        total = os.cpu_count() or 1
    return max(1, total + 1 + n_jobs)


def _joblib() -> tuple[Any, Any]:
    """``(Parallel, delayed)``, imported on use — joblib is an optional extra, not a core dep."""
    try:
        from joblib import Parallel, delayed
    except ImportError as exc:  # pragma: no cover - depends on the install extras
        raise ImportError(
            "parallel execution needs joblib: pip install 'opencdarr[parallel]'"
        ) from exc
    return Parallel, delayed


def _whole_replications(reps: int, workers: int) -> bool:
    """Whether whole replications pack the workers evenly, with no half-empty final wave.

    An exact multiple means every wave is full. Anything else — ``reps=100`` on 96 workers being
    the pointed example — leaves most of the machine idle through a second wave that takes as long
    as the first, so the sharded path wins even though it pays for serialisation.
    """
    return reps % workers == 0


def _shard_count(
    live_reps: int, workers: int, n_particles: int, oversubscribe: int, min_shard: int
) -> int:
    """How many shards to cut each live replication's cloud into this level.

    ``oversubscribe`` asks for more tasks than workers on purpose. Per-particle cost is bimodal —
    a survivor stops at the shell, a non-survivor runs on to ``is_terminal`` — so equal-sized
    shards do *not* take equal time, and one task per worker means everyone waits for the slowest.
    Extra tasks let joblib's dynamic dispatch refill workers as they free up. ``min_shard`` stops
    the split going so fine that per-task overhead shows up.
    """
    want = -(-oversubscribe * workers // live_reps)  # ceil
    return max(1, min(want, max(1, n_particles // min_shard)))


def _shard_bounds(n: int, shards: int) -> list[tuple[int, int]]:
    """Contiguous half-open slices tiling ``range(n)``, remainder spread over the leading ones."""
    base, extra = divmod(n, shards)
    bounds: list[tuple[int, int]] = []
    lo = 0
    for i in range(shards):
        hi = lo + base + (1 if i < extra else 0)
        if hi > lo:
            bounds.append((lo, hi))
        lo = hi
    return bounds


def _evolve_slice(
    particles: Sequence[Particle],
    target: float,
    level_seq: np.random.SeedSequence,
    lo: int,
    hi: int,
) -> list[FleetState]:
    """One worker's slice of one replication's level.

    Rebuilds its own per-particle streams from ``level_seq`` by index rather than receiving them,
    so a 10 000-particle level ships one seed instead of 10 000 (and no worker re-spawns the
    siblings it will not use). Returns bare :class:`~opencdarr.fleet.FleetState` values: the parent
    still holds every ``env``, so only the evolved world needs to travel back.
    """
    return [p.state for p in evolve_shard(particles, target, children(level_seq, lo, hi))]


def _lockstep(
    build_initial: BuildInitial,
    levels: Sequence[float],
    n_particles: int,
    seqs: Sequence[np.random.SeedSequence],
    *,
    workers: int,
    oversubscribe: int,
    min_shard: int,
    verbose: int,
    tail: bool = True,
) -> list[IPSResult]:
    """Advance every replication shell-by-shell together, sharding each level across all workers.

    The seed tree is built exactly as :func:`~opencdarr.ips.ips_once` builds it, and this function
    never calls ``.spawn()`` on a level's sequence — it addresses that sequence's children by index
    instead — so each level's per-particle streams are the ones the serial run would have used.
    """
    parallel_cls, delayed = _joblib()
    reps = len(seqs)

    clouds: list[list[Particle]] = []
    level_seqs: list[list[np.random.SeedSequence]] = []
    tail_seqs: list[np.random.SeedSequence] = []
    for seq in seqs:
        # addressed by index, exactly as ips_once does it, so the tree is the same one and the
        # caller's sequence is left untouched
        # three children, exactly as ips_once asks for them — the third is the tail's, and a
        # SeedSequence child depends only on its index, so the first two are unmoved by it
        init_seq, evolve_seq, tail_seq = children(seq, 0, 3)
        tail_seqs.append(tail_seq)
        # Built in the parent on purpose: building in workers would give each one its own `env`
        # objects, and pickle only collapses repeated references it can see are *the same object*.
        # Distinct-but-equal envs cost ~2x the bytes and ~4x the serialisation time per level.
        clouds.append([build_initial(s) for s in children(init_seq, 0, n_particles)])
        level_seqs.append(children(evolve_seq, 0, len(levels)))

    survival: list[list[float]] = [[] for _ in range(reps)]
    collapsed: dict[int, IPSResult] = {}
    lineages: list[int] = [0] * reps
    live = list(range(reps))
    n_aircraft = len(clouds[0][0].state.states) if clouds and clouds[0] else 0

    # One pool for the whole ladder: re-creating it per level would re-spawn every worker process
    # and re-import opencdarr in each, 17 times over for a production ladder.
    with parallel_cls(
        n_jobs=workers, batch_size=1, pre_dispatch=str(workers), verbose=verbose
    ) as run:
        for k, target in enumerate(levels):
            shards = _shard_count(len(live), workers, n_particles, oversubscribe, min_shard)
            bounds = _shard_bounds(n_particles, shards)
            plan = [(r, lo, hi) for r in live for lo, hi in bounds]
            done = run(
                delayed(_evolve_slice)(clouds[r][lo:hi], target, level_seqs[r][k], lo, hi)
                for r, lo, hi in plan
            )

            # results arrive in submission order, so appending by plan order rebuilds each
            # replication's evolved list in exactly the order the serial loop produced it
            regrouped: dict[int, list[FleetState]] = {r: [] for r in live}
            for (r, _, _), states in zip(plan, done, strict=True):
                regrouped[r].extend(states)

            for r in live:
                evolved = [
                    Particle(env=p.env, state=s)
                    for p, s in zip(clouds[r], regrouped[r], strict=True)
                ]
                fraction, cloud, drawn = resample_level(
                    evolved, target, n_particles, child(level_seqs[r][k], n_particles)
                )
                survival[r].append(fraction)
                if cloud:
                    clouds[r] = cloud
                    lineages[r] = drawn
                else:
                    collapsed[r] = IPSResult(
                        prob=0.0,
                        levels=tuple(levels),
                        survival=tuple(survival[r]),
                        n_particles=n_particles,
                        collapsed_at=k,
                        n_aircraft=n_aircraft,
                    )
            live = [r for r in live if r not in collapsed]
            if not live:
                break

    # The tail leg, on the same per-replication seed subtree ips_once uses, so a tail field is the
    # serial one exactly. Run here rather than sharded: it is one pass over the final cloud, with
    # no barrier to wait on, and the pool above has already closed.
    finals: dict[int, tuple[float, float]] = {}
    if tail:
        for r in range(reps):
            if r in collapsed:
                continue
            ends = [
                _evolve_to_terminal(p, _streams(s))
                for p, s in zip(clouds[r], children(tail_seqs[r], 0, n_particles), strict=True)
            ]
            finals[r] = (float(np.mean([e.n_los_pairs for e in ends])),
                         float(np.mean([e.n_los_aircraft for e in ends])))

    return [
        collapsed[r]
        if r in collapsed
        else IPSResult(
            prob=float(np.prod(survival[r])),
            levels=tuple(levels),
            survival=tuple(survival[r]),
            n_particles=n_particles,
            collapsed_at=None,
            tail_k=finals[r][0] if r in finals else None,
            tail_a=finals[r][1] if r in finals else None,
            n_lineages=lineages[r] if tail else None,
            n_aircraft=n_aircraft,
        )
        for r in range(reps)
    ]


def ips_replications(
    build_initial: BuildInitial,
    levels: Sequence[float],
    n_particles: int,
    seqs: Sequence[np.random.SeedSequence],
    *,
    n_jobs: int = 1,
    oversubscribe: int = 2,
    min_shard: int = 64,
    verbose: int = 0,
    tail: bool = True,
) -> list[IPSResult]:
    """Run one :func:`~opencdarr.ips.ips_once` per seed in ``seqs``, over ``n_jobs`` workers.

    Identical, value for value, to ``[ips_once(build_initial, levels, n_particles, s) for s in
    seqs]`` — only the wall time differs. Takes seeds rather than a seed integer so it composes
    with :func:`~opencdarr.ips.replication_seeds` the same way the serial path does.
    """
    workers = resolve_jobs(n_jobs)
    if workers <= 1 or not seqs:
        return [ips_once(build_initial, levels, n_particles, s, tail=tail)
                for s in seqs]
    if _whole_replications(len(seqs), workers):
        parallel_cls, delayed = _joblib()
        results: list[IPSResult] = list(
            parallel_cls(n_jobs=workers, verbose=verbose)(
                delayed(ips_once)(build_initial, levels, n_particles, s, tail=tail)
                for s in seqs
            )
        )
        return results
    return _lockstep(
        build_initial,
        levels,
        n_particles,
        seqs,
        workers=workers,
        oversubscribe=oversubscribe,
        min_shard=min_shard,
        verbose=verbose,
        tail=tail,
    )


def estimate_rare_prob(
    build_initial: BuildInitial,
    levels: Sequence[float],
    *,
    n_particles: int,
    reps: int,
    seed: int,
    n_jobs: int = -1,
    oversubscribe: int = 2,
    min_shard: int = 64,
    verbose: int = 0,
    tail: bool = True,
) -> RareEventEstimate:
    """The parallel twin of :func:`opencdarr.ips.estimate_rare_prob` — same result, more cores.

    Unlike the serial version, the worker count is independent of ``reps``: pick ``reps`` for the
    stability you want (ADR 0017 §5) and ``n_jobs`` for the machine you have.
    """
    return combine_replications(
        ips_replications(
            build_initial,
            levels,
            n_particles,
            replication_seeds(seed, reps),
            n_jobs=n_jobs,
            oversubscribe=oversubscribe,
            min_shard=min_shard,
            verbose=verbose,
            tail=tail,
        )
    )
