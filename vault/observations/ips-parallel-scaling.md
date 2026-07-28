# IPS parallel scaling: cutting the tie between replication count and core count

**Status: measured on the M2 dev box; the 96-core confirmation is still to run.** Until now IPS
parallelised one way only — one replication per worker — so a run could never use more cores than
it had replications. But replication count is a *statistical* choice (ADR 0017 §5: the honest CI
comes from independent runs), not a hardware one. [[0018-parallel-ips-scheduling]] separates them:
`opencdarr/parallel.py` also splits the particles *inside* a shell across workers, so any
`(reps, particles)` design fills any machine. Written 2026-07-28. Reproduce:

    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
    python scripts/bench_ips_parallel.py --particles 2000 --reps 2 --jobs 1 2 4 8

## What it cost before

From the committed sweep, `scripts/cns_sweep_20260728_085447/nav_pos3_vel1.0_rx1.0.log`, run with
`--jobs 96`:

    MC   n=2000000  P(LoS)=0.00009   (1582s)
    IPS  10x10000p x17 shells        (918s)     <- 10 of 96 workers busy

918 s of wall for 9 180 core-seconds of work. Worth stating the honest version, because the obvious
reading overstates the case: the **MC half already chunked across all 96 cores**, so the sweep as a
whole ran at ~67 % utilisation, not 11 %. Running its 13 cells concurrently in bash would have cut
9.45 h to ~6.06 h with no Python at all. The parallel driver is not justified by this sweep — it is
justified by single-run latency (ladder tuning becomes interactive), by making an **IPS-only** cell
viable at ~135 s against 2500 s, and by the regime below `rpz` where MC is not slow but impossible.

## Measured scaling (M2, `reps=2`, 2000 particles × 17 shells, `dt=0.5`)

| jobs | schedule | wall | speedup | efficiency |
|---|---|---|---|---|
| 1 | serial | 112.8 s | 1.00× | 100 % |
| 2 | whole-reps | 60.7 s | 1.86× | 93 % |
| 4 | **lockstep**, 8 tasks/level | 40.2 s | 2.81× | 70 % |
| 8 | **lockstep**, 16 tasks/level | 33.8 s | 3.34× | 42 % |

`reps=2` is the point of the configuration: the old code could not exceed two workers, so **60.7 s
was its floor**. Lockstep reaches 33.8 s — a further 1.8× that was previously unreachable. On a
96-core box at `reps=10` the same lever is far longer (10 workers → 96).

Falling efficiency is the hardware, not the scheduler: the M2 is 4 performance + 4 efficiency
cores, so "8 workers" was never 8× the compute. A homogeneous box should do materially better.

## The answer does not move

Every row above carried the same SHA-1 of the concatenated per-shell survival vectors. That is a
stronger check than comparing `P`, which could match from the wrong factors, and it re-verifies
bit-identity at 68 000 segments rather than at the couple of dozen particles a unit test can afford.

It holds because substreams are **addressed by index rather than handed out in sequence**:
`spawn(parent, n)[i]` extends the parent's `spawn_key` by `i`, independent of `n`, so a worker
rebuilds exactly its own slice (`rng.child` / `rng.children`) and particle *i* gets stream *i*
whatever shard it lands in. `tests/test_parallel.py` crosses a real process boundary at six
`(reps, n_jobs)` shapes and was mutation-checked — mis-indexing the shard seeds by one token fails
8 of its tests.

End to end: `examples/handbook/rare_event_ips.ipynb` re-runs to `P(LoS) = 4.17e-05`, CI and all 17
survival fractions identical to its stored output.

## Where the ceiling is

Per particle-level the parent pays ~13 µs of pickling against ~5.4 ms of compute — a serial
fraction near 0.25 %, so the Amdahl limit is a few hundred cores. Expect ~70 % efficiency at 96,
not 100 %. Supporting measurements (2-aircraft sweep-cell config):

| | |
|---|---|
| `env.advance` one step, `dt=0.2` | 65–115 µs |
| `FleetState` pickled | 1 343 B |
| 10 000 particles, **shared** env | 3.3 MB · dumps 25 ms |
| 10 000 particles, **distinct-but-equal** envs | 7.0 MB · dumps 95 ms (**2.1× bytes, 3.8× time**) |

That last row is why pinned geometry now builds its starting particle once and shares it: pickle
collapses repeated references to *one* object, so identity is worth as much as the saved CPU.

## Two things that bit during implementation

**`SeedSequence.spawn` is stateful.** It hands out children from the parent's
`n_children_spawned`, so calling `ips_once` twice with the *same* seed object silently gives
different answers — the second run walks a different subtree. This produced a convincing false
"the results changed" signal and cost real debugging time. No current caller is wrong
(`replication_seeds` returns fresh objects), but it is a sharp edge worth a guard.

**Neither obvious memo spelling survives a worker.** `functools.cached_property` holds an `RLock`
on Python 3.11; an `lru_cache` wrapper is a C object that pickles by *reference*, so the worker
looks it up in loky's bootstrap `__main__` and fails. A scenario that travels to workers has to
memoise by hand into `__dict__`.

## How to run it

Always export the thread caps first — without them each of N workers starts its own BLAS pool and
they fight for the same cores:

    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1

**1. Check the box before trusting it.** Identical results at every worker count, and a speedup
that keeps climbing past `--reps`. Exit code is non-zero if any two runs disagree, so it works as a
gate in a script:

    python scripts/bench_ips_parallel.py --particles 2000 --reps 2 --jobs 1 2 8 32 96

Keep `--reps` *below* the largest `--jobs`, or every run takes the whole-replication path and the
sharded scheduler is never exercised. Also worth one run on a ladder nothing can reach, so the
collapse branch is exercised under real workers:

    python scripts/bench_ips_parallel.py --particles 200 --reps 2 --jobs 1 8 --pos 2 --levels 70 40

**2. One validation cell** — IPS against its MC anchor. `--jobs` is now independent of `--reps`,
so set it to the core count and choose `--reps` purely for the CI width you want:

    python scripts/ips_validate.py --dpsi 90 --tlos 70 --lookahead 60 --dt 0.2 \
        --pos 3 --vel 1.0 --reception 1.0 --mc-n 2000000 --particles 10000 --reps 10 \
        --levels 150 135 122 112 104 97 90 82 74 68 63 59 56 54 52 51 50 --jobs 96

The log now carries a `schedule:` line next to the estimate, so an under-filled machine is visible
in the record rather than only in a wall-clock number nobody compares. Baseline to beat, from the
committed run of this exact cell: IPS 918 s, `P=0.000092`, `collapsed=0/10`. **`P` and every
per-shell survival must be unchanged** — only the wall time may move.

**3. The full sweep**, unchanged in shape (`REPS=10 ; JOBS=96` is now the latency-optimal pairing;
`REPS=96` buys a tighter CI at roughly the same wall clock on an idle box):

    bash scripts/cns_sweep.sh

**From Python**, e.g. in a notebook — see `examples/handbook/rare_event_ips.ipynb`:

    from opencdarr.parallel import estimate_rare_prob
    est = estimate_rare_prob(build_initial, LEVELS, n_particles=10_000,
                             reps=10, seed=20260728, n_jobs=-1)

`opencdarr.ips.estimate_rare_prob` remains the serial reference and is unchanged; swapping the
import is the whole difference.

## Related

[[ips-gate1-correctness]] · [[ips-gate2-efficiency]] · [[0018-parallel-ips-scheduling]] ·
[[0017-ips-level-and-splitting]] · [[0001-rng-per-particle-spawn]]. Still open: the SoA vectorised
`step_batch` (`docs/roadmap.md`), which is the other half of the performance story, and
[[ips-adaptive-levels]], which reduces the work rather than spreading it.
