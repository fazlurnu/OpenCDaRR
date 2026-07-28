"""Scaling gate for the parallel IPS driver: same design, several worker counts, one table.

Runs one identical ``(build_initial, levels, particles, reps, seed)`` design at each ``--jobs``
value and reports how the wall time scales. Two things are being checked at once, and the first
matters more:

1. **The answer never moves.** Every row must carry the same ``survival`` fingerprint. Scheduling
   is not allowed to change a probability, so a single mismatch fails the run (exit 1). This
   re-verifies bit-identity at production scale, not just at the couple-of-dozen particles
   ``tests/test_parallel.py`` can afford.
2. **The machine actually fills.** ``reps=2`` is the interesting case: before the parallel driver,
   two replications could occupy at most two cores no matter what ``--jobs`` said. ``--jobs 4``
   beating ``--jobs 2`` is the demonstration that the ceiling is gone.

Run it before trusting a big box, then again on the big box:

    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \\
    python scripts/bench_ips_parallel.py --dpsi 90 --tlos 70 --lookahead 60 --dt 0.5 \\
        --pos 3 --vel 1.0 --particles 2000 --reps 2 --jobs 1 2 4

    # the drop-out branch, under real workers: a ladder nothing can reach
    python scripts/bench_ips_parallel.py --particles 200 --reps 2 --jobs 1 4 \\
        --pos 2 --levels 70 40

The BLAS environment variables matter once ``--jobs`` is large: without them each worker starts
its own thread pool and they fight. The simulation makes no BLAS calls, so this is about not
oversubscribing the machine rather than about correctness.

**Reading the scaling numbers.** Speedup is measured against the *first* ``--jobs`` value. On a
heterogeneous CPU (Apple silicon's performance + efficiency cores, say) expect visibly less than
linear — 4 workers on a 4+4 machine land nearer 3x than 4x, because some of them run on the slow
cores. That is the hardware, not the scheduler. Peak RSS comes from ``getrusage`` and counts only
children that have been reaped, so with a persistent worker pool it under-reports; treat it as a
floor.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import resource
import sys
import time

from opencdarr.ips import RareEventEstimate
from opencdarr.parallel import describe_schedule, estimate_rare_prob, resolve_jobs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ips_validate import Scenario  # noqa: E402  - sibling script, path fixed up above

# ru_maxrss is bytes on macOS and kilobytes on Linux; normalise to MiB.
_RSS_SCALE = 1024**2 if sys.platform == "darwin" else 1024


def peak_rss_mib() -> float:
    """Peak resident memory of this process and its reaped children, in MiB."""
    here = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    kids = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    return (here + kids) / _RSS_SCALE


def fingerprint(est: RareEventEstimate) -> str:
    """A short hash of every per-shell survival fraction — the thing scheduling must not change.

    Hashing the whole survival vector rather than just ``prob`` catches a scheduler that got the
    right product from the wrong factors.
    """
    payload = repr([r.survival for r in est.reps]).encode()
    return hashlib.sha1(payload).hexdigest()[:12]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--jobs", type=int, nargs="+", default=[1, 2, 4],
                   help="worker counts to compare (default: 1 2 4); -1 means every core")
    p.add_argument("--particles", type=int, default=500, help="IPS particles per level")
    p.add_argument("--reps", type=int, default=2, help="independent replications; keep it BELOW "
                   "the largest --jobs, or every run picks the whole-replication path and the "
                   "sharded scheduler is never exercised")
    p.add_argument("--levels", type=float, nargs="+",
                   default=[150, 135, 122, 112, 104, 97, 90, 82, 74, 68, 63, 59, 56, 54, 52, 51,
                            50], help="shell distances [m], ending at rpz")
    p.add_argument("--pos", type=float, default=3.0, help="pos_ci95 [m]")
    p.add_argument("--vel", type=float, default=1.0, help="vel_ci95 [m/s]")
    p.add_argument("--dpsi", type=float, default=90.0, help="fixed crossing angle [deg]")
    p.add_argument("--tlos", type=float, default=70.0, help="time to loss of separation [s]")
    p.add_argument("--lookahead", type=float, default=60.0, help="detection lookahead [s]")
    p.add_argument("--dt", type=float, default=0.5, help="integration step [s]")
    p.add_argument("--reception", type=float, default=1.0, help="P(broadcast received) per link")
    p.add_argument("--min-shard", dest="min_shard", type=int, default=64,
                   help="smallest particle shard handed to a worker (default: 64)")
    p.add_argument("--seed", type=int, default=20260728)
    a = p.parse_args()

    scn = Scenario(pos_ci95=a.pos, vel_ci95=a.vel, dpsi=a.dpsi, dt=a.dt,
                   lookahead=a.lookahead, tlos=a.tlos, reception_prob=a.reception)

    budget = a.reps * a.particles * len(a.levels)
    print(f"scenario: fixed {a.dpsi:.0f}deg crossing, pos_ci95={a.pos} vel_ci95={a.vel} "
          f"dt={a.dt} rx={a.reception}")
    print(f"design  : {a.reps} reps x {a.particles} particles x {len(a.levels)} shells "
          f"(~{budget} segments), seed={a.seed}")
    if not any(t in os.environ for t in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS")):
        print("  note: OMP_NUM_THREADS is unset — set it to 1 for a clean multi-worker timing")
    if a.reps >= max(a.jobs):
        print(f"  note: reps ({a.reps}) >= max jobs ({max(a.jobs)}), so every run takes the "
              f"whole-replication path; lower --reps to exercise sharding")
    print()

    header = f"{'jobs':>5}  {'schedule':<44}  {'wall_s':>8}  {'speedup':>7}  {'eff':>5}  " \
             f"{'P':>10}  {'survival':>12}  {'rss_MiB':>7}"
    print(header)
    print("-" * len(header))

    baseline: float | None = None
    prints: list[str] = []
    seen: dict[str, list[int]] = {}
    for jobs in a.jobs:
        workers = resolve_jobs(jobs)
        schedule = describe_schedule(a.reps, a.particles, jobs, min_shard=a.min_shard)
        t0 = time.perf_counter()
        est = estimate_rare_prob(
            scn.build_initial, a.levels, n_particles=a.particles, reps=a.reps, seed=a.seed,
            n_jobs=jobs, min_shard=a.min_shard,
        )
        wall = time.perf_counter() - t0
        if baseline is None:
            baseline = wall
        speedup = baseline / wall
        digest = fingerprint(est)
        seen.setdefault(digest, []).append(jobs)
        row = (f"{jobs:>5}  {schedule:<44}  {wall:>8.1f}  {speedup:>6.2f}x  "
               f"{speedup / workers:>5.0%}  {est.prob:>10.3e}  {digest:>12}  "
               f"{peak_rss_mib():>7.0f}")
        print(row, flush=True)
        prints.append(row)

    print()
    if len(seen) == 1:
        print(f"OK: identical survival fingerprint across every worker count ({next(iter(seen))})")
        return 0
    print("FAIL: scheduling changed the estimate — these worker counts disagree:")
    for digest, jobs in seen.items():
        print(f"  {digest}  from --jobs {' '.join(str(j) for j in jobs)}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
