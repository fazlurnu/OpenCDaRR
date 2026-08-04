#!/usr/bin/env python3
"""Core-count scaling sweep for the parallel IPS driver: where does a big box stop paying off?

``scripts/bench_ips_parallel.py`` answers "does the answer move when I add workers?" (it must
not) for a handful of hand-picked ``--jobs``. This script answers the other question — *how many
of my cores are actually worth using* — and is built for boxes wide enough that guessing wrong
wastes an hour:

* **The ladder is geometric, not linear.** Speedup curves are flat-then-bent; ``50 100 150`` are
  three points on the flat part of whatever curve you have and cannot distinguish "saturates at
  32" from "scales to 200". Doubling (1 2 4 … N) finds the knee in the same wall time, because
  the cheap points cost almost nothing: the whole ladder runs in ~2x the serial run.
* **Pool startup is timed separately.** On 200 cores, spawning workers and importing opencdarr in
  each is seconds of one-time cost that has no business inside a steady-state number. Each point
  gets a throwaway warm-up run first (reported as ``pool_s``); the timed run reuses loky's
  persistent pool.
* **Under-filled schedules are flagged before you read the timings.** The sharded path cuts at
  most ``n_particles // min_shard`` shards per replication, so a design can be structurally
  incapable of occupying 200 workers no matter what ``--jobs`` says. The ``tasks`` column and the
  warnings say so explicitly, instead of leaving it to look like poor scaling.
* **The correctness gate is kept.** Every row must produce the same survival fingerprint;
  one mismatch fails the run (exit 1), exactly as the sibling script does.

Typical use on the big machine::

    # 1. size the design so the serial baseline is ~5 min (the whole ladder is then ~10 min)
    python scripts/bench_ips_cores.py --auto-size 300 --reps 10 --jobs 1

    # 2. run the ladder it printed, with the particle count it chose
    python scripts/bench_ips_cores.py --particles 4000 --reps 10 \\
        --jobs 1 2 4 8 16 32 64 128 200 --repeat 2 --csv scaling.csv

Omit ``--jobs`` and the ladder is built from the machine's own core count. BLAS threading is
pinned to 1 by this script before numpy is imported, so N workers do not each start a thread pool
and fight; export the variables yourself to override.

**Reading the result.** ``eff`` is speedup / workers. The recommendation line reports the widest
point still above ``--eff-floor`` (the economical choice) and the outright fastest point (the
"I don't care about idle cores" choice); on a shared box the first is the one to submit with.
"""

from __future__ import annotations

import os

# Before numpy lands in the process, here or in any import below: N workers each starting their
# own BLAS pool on a 200-core box is the classic way to make scaling look worse than it is.
for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
             "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_var, "1")

import argparse  # noqa: E402
import csv  # noqa: E402
import hashlib  # noqa: E402
import platform  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402

from opencdarr.ips import RareEventEstimate  # noqa: E402
from opencdarr.parallel import (  # noqa: E402
    _shard_count,  # private on purpose: the capacity warning must use the *real* formula, and
    _whole_replications,  # break loudly if it ever changes rather than report stale arithmetic
    describe_schedule,
    estimate_rare_prob,
    resolve_jobs,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ips_validate import Scenario  # noqa: E402  - sibling script, path fixed up above

# The 17-shell production ladder, shared with bench_ips_parallel.py so timings are comparable.
DEFAULT_LEVELS = [150, 135, 122, 112, 104, 97, 90, 82, 74, 68, 63, 59, 56, 54, 52, 51, 50]


def core_ladder(total: int) -> list[int]:
    """Doubling ladder 1, 2, 4, … capped at ``total``, with ``total`` itself as the last rung.

    Geometric rather than linear because the quantity of interest is *where the curve bends*, and
    a bend is a ratio. The cheap low rungs are nearly free — a run at ``jobs`` costs about
    ``T1/jobs`` — so the ladder as a whole costs roughly twice the serial point it starts from.
    """
    rungs = []
    j = 1
    while j < total:
        rungs.append(j)
        j *= 2
    rungs.append(total)
    return rungs


def fingerprint(est: RareEventEstimate) -> str:
    """Short hash of every per-shell survival fraction — the thing scheduling must not change.

    Hashing the whole survival vector rather than ``prob`` catches a scheduler that reached the
    right product through the wrong factors.
    """
    return hashlib.sha1(repr([r.survival for r in est.reps]).encode()).hexdigest()[:12]


def schedule_capacity(reps: int, n_particles: int, workers: int, min_shard: int,
                      oversubscribe: int) -> tuple[int, str]:
    """``(tasks per level, why-it-might-not-fill)`` for one worker count.

    A design occupies ``workers`` cores only if it can be cut into at least that many tasks. The
    sharded path caps shards at ``n_particles // min_shard``, so past a certain width the answer
    is a property of the *design*, not of the scheduler — and the fix is more particles or a
    smaller ``--min-shard``, not more cores.
    """
    if workers <= 1:
        return 1, ""
    if _whole_replications(reps, workers):
        # one task per replication; waves of `workers`, so reps below workers cannot fill
        return reps, ""
    shards = _shard_count(reps, workers, n_particles, oversubscribe, min_shard)
    tasks = shards * reps
    if tasks < workers:
        return tasks, f"only {tasks} tasks for {workers} workers — {workers - tasks} idle"
    if tasks < 2 * workers:
        return tasks, f"{tasks / workers:.1f} tasks/worker — too coarse to load-balance"
    return tasks, ""


def time_point(scn: Scenario, a: argparse.Namespace, jobs: int) -> tuple[float, float, RareEventEstimate]:
    """``(pool_s, wall_s, estimate)`` for one worker count, warm-up excluded from ``wall_s``.

    The warm-up is a deliberately tiny design at the same ``jobs``: it forces loky to spawn and
    import into every worker, and loky keeps that pool alive for the timed call that follows.
    Without it the first level of the real run carries several seconds of process startup that
    then gets divided by nothing and read as bad scaling.
    """
    t0 = time.perf_counter()
    estimate_rare_prob(scn.build_initial, a.levels[:2], n_particles=max(64, a.min_shard),
                       reps=a.reps, seed=a.seed, n_jobs=jobs, min_shard=a.min_shard)
    pool_s = time.perf_counter() - t0

    best: float | None = None
    est: RareEventEstimate | None = None
    for _ in range(a.repeat):
        t0 = time.perf_counter()
        est = estimate_rare_prob(scn.build_initial, a.levels, n_particles=a.particles,
                                 reps=a.reps, seed=a.seed, n_jobs=jobs,
                                 oversubscribe=a.oversubscribe, min_shard=a.min_shard)
        wall = time.perf_counter() - t0
        # min, not mean: repeats can only be slowed by interference from the rest of the box, so
        # the fastest observation is the least contaminated estimate of the true cost.
        best = wall if best is None else min(best, wall)
    assert best is not None and est is not None
    return pool_s, best, est


def auto_size(scn: Scenario, a: argparse.Namespace, target_s: float) -> int:
    """Pick ``n_particles`` so the *serial* run lands near ``target_s`` seconds.

    Times a small serial probe and scales linearly in particle count. Linear is an approximation —
    resampling is cheap next to segment integration, but a bigger cloud also survives deeper — so
    treat the number as a starting point, not a promise. Sizing matters because the whole ladder
    costs about ``2 x T1``: aim T1 at a few minutes and the sweep is a coffee break, aim it at an
    hour and it is an afternoon.
    """
    n = a.probe_particles
    t0 = time.perf_counter()
    est = estimate_rare_prob(scn.build_initial, a.levels, n_particles=n, reps=1, seed=a.seed,
                             n_jobs=1)
    probe_s = time.perf_counter() - t0
    if est.n_collapsed:
        print(f"  warning: the {n}-particle probe collapsed — it stopped early, so the estimate "
              f"below is a lower bound on cost. Raise --probe-particles.")
    per_particle = probe_s / n  # the probe is one replication, so this is cost per particle-rep
    want = max(100, int(round(target_s / (per_particle * a.reps) / 100.0)) * 100)
    print(f"auto-size: probe {n} particles x 1 rep took {probe_s:.1f}s "
          f"-> {want} particles x {a.reps} reps ~ {want * a.reps * per_particle:.0f}s serial")
    return want


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--jobs", type=int, nargs="+", default=None,
                   help="worker counts to compare (default: doubling ladder up to every core)")
    p.add_argument("--particles", type=int, default=2000, help="IPS particles per level")
    p.add_argument("--reps", type=int, default=10,
                   help="independent replications; a statistical choice (the CI width), but it "
                        "also caps the whole-replication path — see the 'schedule' column")
    p.add_argument("--levels", type=float, nargs="+", default=DEFAULT_LEVELS,
                   help="shell distances [m], ending at rpz")
    p.add_argument("--repeat", type=int, default=1,
                   help="timed runs per worker count; the minimum is reported (default: 1)")
    p.add_argument("--auto-size", type=float, default=None, metavar="SECONDS",
                   help="ignore --particles; probe, then pick a count whose serial run takes "
                        "about SECONDS. The full ladder costs roughly 2x that.")
    p.add_argument("--probe-particles", type=int, default=200,
                   help="particle count for the --auto-size probe (default: 200)")
    p.add_argument("--eff-floor", type=float, default=0.5,
                   help="parallel efficiency below which a rung counts as wasted cores "
                        "(default: 0.5)")
    p.add_argument("--csv", type=str, default=None, help="also write the table to this CSV path")
    p.add_argument("--pos", type=float, default=3.0, help="pos_ci95 [m]")
    p.add_argument("--vel", type=float, default=1.0, help="vel_ci95 [m/s]")
    p.add_argument("--dpsi", type=float, default=90.0, help="fixed crossing angle [deg]")
    p.add_argument("--tlos", type=float, default=70.0, help="time to loss of separation [s]")
    p.add_argument("--lookahead", type=float, default=60.0, help="detection lookahead [s]")
    p.add_argument("--dt", type=float, default=0.5, help="integration step [s]")
    p.add_argument("--reception", type=float, default=1.0, help="P(broadcast received) per link")
    p.add_argument("--min-shard", dest="min_shard", type=int, default=64,
                   help="smallest particle shard handed to a worker (default: 64)")
    p.add_argument("--oversubscribe", type=int, default=2,
                   help="tasks per worker the scheduler aims for (default: 2)")
    p.add_argument("--seed", type=int, default=20260728)
    a = p.parse_args()

    total_cores = resolve_jobs(-1)
    if a.jobs is None:
        a.jobs = core_ladder(total_cores)
    a.jobs = sorted({resolve_jobs(j) for j in a.jobs})

    scn = Scenario(pos_ci95=a.pos, vel_ci95=a.vel, dpsi=a.dpsi, dt=a.dt,
                   lookahead=a.lookahead, tlos=a.tlos, reception_prob=a.reception)

    print(f"machine : {platform.platform()}, {total_cores} usable cores")
    print(f"scenario: fixed {a.dpsi:.0f}deg crossing, pos_ci95={a.pos} vel_ci95={a.vel} "
          f"dt={a.dt} rx={a.reception}")
    if a.auto_size is not None:
        a.particles = auto_size(scn, a, a.auto_size)
    print(f"design  : {a.reps} reps x {a.particles} particles x {len(a.levels)} shells "
          f"(~{a.reps * a.particles * len(a.levels)} segments), seed={a.seed}")
    print(f"ladder  : {' '.join(str(j) for j in a.jobs)}   "
          f"({a.repeat} timed run{'s' if a.repeat > 1 else ''} each, min reported)")

    widest = max(a.jobs)
    _, why = schedule_capacity(a.reps, a.particles, widest, a.min_shard, a.oversubscribe)
    if why:
        need = 2 * widest * a.min_shard // max(1, a.reps)
        print(f"  WARNING at --jobs {widest}: {why}.")
        print(f"           the design cannot fill the box — use --particles {need} or more, "
              f"or --min-shard {max(1, a.particles * a.reps // (2 * widest))}.")
    print()

    header = (f"{'jobs':>5}  {'schedule':<44}  {'tasks':>6}  {'pool_s':>7}  {'wall_s':>9}  "
              f"{'speedup':>8}  {'eff':>5}  {'P':>10}  {'survival':>12}")
    print(header)
    print("-" * len(header))

    rows: list[dict[str, object]] = []
    baseline: float | None = None
    seen: dict[str, list[int]] = {}
    for jobs in a.jobs:
        tasks, why = schedule_capacity(a.reps, a.particles, jobs, a.min_shard, a.oversubscribe)
        pool_s, wall, est = time_point(scn, a, jobs)
        if baseline is None:
            baseline = wall
        speedup = baseline / wall
        digest = fingerprint(est)
        seen.setdefault(digest, []).append(jobs)
        print(f"{jobs:>5}  {describe_schedule(a.reps, a.particles, jobs, min_shard=a.min_shard):<44}"
              f"  {tasks:>6}  {pool_s:>7.1f}  {wall:>9.1f}  {speedup:>7.2f}x  "
              f"{speedup / jobs:>5.0%}  {est.prob:>10.3e}  {digest:>12}"
              + (f"   <- {why}" if why else ""), flush=True)
        rows.append({"jobs": jobs, "tasks_per_level": tasks, "pool_s": round(pool_s, 3),
                     "wall_s": round(wall, 3), "speedup": round(speedup, 3),
                     "efficiency": round(speedup / jobs, 4), "prob": est.prob,
                     "n_collapsed": est.n_collapsed, "fingerprint": digest,
                     "particles": a.particles, "reps": a.reps, "warning": why})

    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nwrote {a.csv}")

    print()
    if len(seen) > 1:
        print("FAIL: scheduling changed the estimate — these worker counts disagree:")
        for digest, jobs in seen.items():
            print(f"  {digest}  from --jobs {' '.join(str(j) for j in jobs)}")
        return 1
    print(f"OK: identical survival fingerprint across every worker count ({next(iter(seen))})")

    fastest = min(rows, key=lambda r: r["wall_s"])
    economical = max((r for r in rows if r["efficiency"] >= a.eff_floor),
                     key=lambda r: r["jobs"], default=rows[0])
    print(f"\nfastest    : --jobs {fastest['jobs']} at {fastest['wall_s']:.1f}s "
          f"({fastest['speedup']:.1f}x, {fastest['efficiency']:.0%} efficient)")
    print(f"economical : --jobs {economical['jobs']} at {economical['wall_s']:.1f}s "
          f"({economical['speedup']:.1f}x, {economical['efficiency']:.0%} efficient) "
          f"— widest rung still above --eff-floor {a.eff_floor:.0%}")
    if fastest["jobs"] != economical["jobs"]:
        extra = fastest["jobs"] - economical["jobs"]
        saved = economical["wall_s"] - fastest["wall_s"]
        print(f"             the last {extra} cores buy {saved:.1f}s "
              f"({saved / economical['wall_s']:.0%}); on a shared box they are better spent "
              f"running {fastest['jobs'] // economical['jobs']} designs at once.")
    if len(a.jobs) > 1 and fastest["jobs"] == max(a.jobs) and fastest["efficiency"] >= a.eff_floor:
        print(f"             still scaling at the top of the ladder — extend past "
              f"{max(a.jobs)} if the box has more.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
