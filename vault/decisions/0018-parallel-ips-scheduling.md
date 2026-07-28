# ADR 0018 — Parallel IPS: a scheduling module beside the estimator, not inside it

- Status: accepted
- Date: 2026-07-28
- Deciders: Fazlur Rahman

## Context

[[0017-ips-level-and-splitting]] fixed the estimator; this ADR fixes how it is *executed*.

Until now the only parallelism lived in callers, and it had one shape:
`Parallel(delayed(ips_once))` over `replication_seeds`. That caps usable cores at `reps` — and
`reps` is a **statistical** choice (§5 of ADR 0017: the honest CI comes from independent
replications), not a hardware one. The two were accidentally welded together.

The cost is on record. In `scripts/cns_sweep_20260728_085447/`, a production cell run with
`--jobs 96`:

```
MC   n=2000000  P(LoS)=0.00009   (1582s)
IPS  10x10000p x17 shells        (918s)     <- 10 of 96 workers busy
```

918 s of wall for 9 180 core-seconds of work. Note the honest version of this number: the MC half of
each cell already chunks across all 96 cores, so the *sweep* was running at ~67 % aggregate
utilisation, not 11 %. Running the sweep's 13 cells concurrently in bash would have cut it from
9.45 h to ~6.06 h with no Python at all. **This ADR is therefore not justified by the sweep.** It is
justified by three things the bash trick cannot buy:

1. **Single-run latency.** 918 s → ~135 s makes shell-ladder tuning (watching the `survival/shell`
   line and moving `--levels`) an interactive loop instead of a 15-minute wait per experiment.
2. **Retiring the MC anchor.** At `P ≈ 3e-5` the 2-million-sample anchor reads ~60 events — a
   Wilson CI of roughly ±25 %. It is already weak. Once IPS fills the box, an IPS-only cell is
   ~135 s against 2500 s, and MC stays on the cheap correctness rung (`--pos 40`, `P ≈ 0.028`)
   where it is genuinely tight.
3. **Pushing `d_m` below `rpz`** toward a physical collision radius (ADR 0017 §1). There MC is not
   slow, it is impossible, and IPS wall-time is the only cost that exists.

## Decision

### 1. The scheduler lives in `opencdarr/parallel.py`, not in `ips.py`

`ips.py` and `estimator.py` stay serial, joblib-free, and readable — they remain the reference the
parallel path is *validated against*, the same discipline as the analytical ⊂ MC ⊂ IPS ladder one
level up. `design-philosophy.md` #126 licenses breaking purity on a measured bottleneck; it licenses
adding a fast path, not contaminating the reference.

joblib is imported lazily and declared in a new `parallel` extra, so `import opencdarr.parallel`
succeeds without it and only `n_jobs != 1` raises. Core install stays `numpy + pyyaml`.

`ips_once`'s signature is unchanged — five scripts and a notebook depend on it.

### 2. Two scheduling modes, chosen by the shape of the design

- **Whole replications** when `reps % workers == 0`. Nothing crosses a process boundary but a seed
  and a result: the zero-overhead best case. The condition is an *exact multiple*, not `>=`. At
  `reps=100` on 96 workers, four workers would run a second replication while 92 idle, roughly
  doubling wall time — so that case takes the sharded path instead.
- **Lockstep** otherwise. Every live replication advances one shell at a time together, and each
  level's particles are cut into contiguous shards spread across all workers. Replications that
  collapse leave the live set, the shard count is recomputed, and the machine refills itself.

Shards **oversubscribe** workers (default 2×). Per-particle cost is bimodal — a survivor stops at
the shell, a non-survivor runs on to `is_terminal` — so equal-sized shards do not take equal time,
and one task per worker means everyone waits for the slowest.

One `Parallel` pool is held open across the whole ladder; re-creating it per level would re-spawn
every worker and re-import `opencdarr` in each, 17 times over.

### 3. Bit-identical, and locked by a test that actually spawns workers

The claim is exact equality, not statistical equivalence, and it rests on:

- the per-level map is order-independent — particle *i* reads only `particles[i]` and `seeds[i]`;
- **substreams are addressed by index, not consumed in sequence.** `spawn(parent, n)[i]` extends
  the parent's `spawn_key` by `i`, independent of `n`, so a worker rebuilds exactly its own slice
  (`rng.child` / `rng.children`) without spawning siblings and without touching the parent.
  `ips_once` and the lockstep driver now build their *whole* tree this way, which additionally
  makes them pure functions of the sequence handed in — see §6;
- joblib returns results in submission order, so concatenating shards reproduces the serial list;
- everything after the map — survivor filter, survival fraction, the resample draw from its own
  stream — runs in the parent on that identical list.

Same machine, same numpy. Cross-machine identity is not claimed, and never was for the serial path
either.

`tests/test_parallel.py` crosses a real process boundary at six `(reps, n_jobs)` shapes plus a
collapsing ladder, compares with `==`, and was mutation-checked: mis-indexing the shard seeds by one
token fails 8 of its tests. `tests/test_rng.py` locks the index-addressing property itself, so a
numpy change to `SeedSequence` numbering fails loudly instead of silently moving every estimate.

### 4. `estimate_ipr` gains a `seqs=` seam; `seed + i` chunking is retired

`scripts/ips_validate.py` and `ips_unified_validate.py` split MC work by rooting each chunk at
`seed + i` — the pattern `rng.py` warns against, because those trees can correlate and their union
is not the tree a serial run of the same `n` walks. The chunked estimate was therefore never
reproducible against the serial one. Chunks now address slices of the *one* tree via
`children(root, lo, hi)` and pool with `combine_ipr`, which recomputes IPR from pooled counts (a
ratio cannot be averaged across chunks). Locked by `tests/test_estimator.py`.

This is a correctness fix. There is no speed win available — MC already saturated the box.

### 5. Pinned geometry builds its starting particle once

`Scenario.build_initial` called `build_env` per particle: 10 000 calls producing 10 000 equal
objects. Sharing one is safe because `FleetEnv`/`FleetState` are deeply immutable — the same
property that lets IPS clone a survivor by sharing it. Identity, not just the saved CPU, is the
point: pickle collapses repeated references to *one* object within a payload, so distinct-but-equal
envs cost ~2× the bytes and ~4× the serialisation time, a toll paid on every level.

Memoised by hand into `__dict__`, because the scenario travels to workers and both obvious
spellings break there: `functools.cached_property` holds an `RLock` on Python 3.11, and an
`lru_cache` wrapper is a C object that pickles by *reference* — either one fails when cloudpickle
ships a `__main__` class to a worker. Both were hit during implementation.

### 6. The estimators address their seed tree by index rather than spawning from it

`SeedSequence.spawn` is stateful, so `ips_once`'s old opening line `seq.spawn(2)` *consumed* its
argument: a second call on the same object walked a different subtree and returned a different
estimate, with nothing in the result to reveal it. That cost real debugging time during this work.

The obvious fix — reject a already-spawned-from sequence — turns out not to work, and writing the
test is what showed it. In whole-replication mode the seeds are pickled to workers, which consume
*copies*; the caller's originals stay untouched. So reuse would have been silently harmless at
`n_jobs=2` and silently wrong at `n_jobs=1` or 96, and no guard can reconcile that, because in the
tolerant case there is no state change to observe.

Building the tree with `children(seq, 0, n)` instead — which reads the same children without
advancing anything — makes `ips_once` and `_lockstep` pure functions of their arguments on every
path. Reuse becomes safe rather than detected, and the behaviour no longer depends on `n_jobs`.
Bit-identical for every existing caller, all of which passed fresh sequences.

## Consequences

**Measured on the M2 dev box** (`scripts/bench_ips_parallel.py`, `reps=2`, 2000 particles × 17
shells, `dt=0.5`) — the gate that runs before any big-box deployment:

| jobs | schedule | wall (3 runs) |
|---|---|---|
| 1 | serial | 112.8 / 124.2 / 119.1 s |
| 2 | whole-reps | 60.7 / 72.3 / 69.4 s |
| 4 | lockstep, 8 tasks/level | 40.2 / 58.4 / 47.6 s |
| 8 | lockstep, 16 tasks/level | 33.8 / 55.5 / 48.4 s |

Identical survival fingerprint at every worker count in every run. With `reps=2` the old code could
not exceed two workers, so the whole-reps row was its floor; lockstep beats it in all three. The
spread is thermal — serial varies 10 % across runs while 8-way varies 64 %, which is what a
throttling laptop looks like — so the ordering is the result and the ratios are local colour. Four
workers is this machine's practical ceiling (it has 4 performance + 4 efficiency cores, and by run
3 eight jobs bought nothing over four). The 96-core figure is still to be measured.

`examples/handbook/rare_event_ips.ipynb` re-runs to `P(LoS) = 4.17e-05`, CI and all 17 survival
fractions unchanged from its stored output.

**Costs and limits.**

- Serialisation sets the ceiling. Per particle-level the parent pays ~13 µs of pickling against
  ~5.4 ms of compute, a serial fraction near 0.25 %, so the Amdahl limit is a few hundred cores.
  Expect ~70 % efficiency at 96, not 100 %.
- Lockstep holds every live replication's cloud in the parent (~1–2 GB at `reps=10 × 10000`), where
  whole-replication mode holds one cloud per worker.
- Persistent workers now run many tasks each. `docs/lesson-learnt.md:113` records a real bug of
  exactly that shape. Audited: no shared mutable state is reachable from `advance` — every
  module-level singleton is frozen or stateless. **Do not add a worker-side env or particle cache**;
  shipping an env costs ~0.16 s across a whole run, and the cache is the bug.
- Export `OMP_NUM_THREADS=1` for large runs. The hot path makes no BLAS calls, so this is about not
  oversubscribing the machine rather than correctness.

**Not done here.** The SoA vectorised `step_batch` (`docs/roadmap.md:78-94`) is the other half of
the performance story and stays gated on a profile. Making `Particle` stdlib-picklable (the
setpoint-adapter and comm-latency lambdas) is worth doing for serialisability, but is measured at
only ~5–8 % and touches three core modules, so it is deliberately separate work.
