# What I tried to do between `0295f6b` and `5bfbc54`

Record written **before reverting to `0295f6b`** ("Campaign runner for the ring and
random-traffic studies"). The intent across these seven commits was sound; the execution is
being thrown away and re-attempted from the clean base. This file captures the *intent* and the
design decisions worth keeping, so the redo does not start from a blank page.

This file is intentionally **untracked** — it survives `git reset --hard 0295f6b`. A
`git clean -fd` would remove it, so move it out of the tree first if you clean.

The seven commits, oldest first:

| # | commit | subject |
|---|--------|---------|
| 1 | `d9067f3` | add scenario and mc ips campaign |
| 2 | `c69e67b` | scenario/: a package, an illustrated README, per-aircraft speed |
| 3 | `cfce9c5` | updating metrics |
| 4 | `505d046` | metrics: P(LoS) per aircraft, and no confidence intervals |
| 5 | `ebec6e5` | metrics: P(LoS) per aircraft, no intervals, and a worker budget that lands |
| 6 | `bb9d341` | validation: time each condition on both backends |
| 7 | `5bfbc54` | update experiment |

They braid together **three separate efforts**. Splitting them apart is the first thing the redo
should do — see "How to re-attempt" at the bottom.

---

## Thread A — the metric rewrite: per-run → per-aircraft P(LoS)

**This is the heart of the whole range.** Commits 3, 4, 5. The plan lived in
`TODO-metric-rewrite.md` (which the revert deletes — its substance is reproduced here).

### Goal

Stop reporting a **per-run** loss probability and report a **per-aircraft** one, following
Blom & Bakker (2015), *Safety Evaluation of Advanced Self-Separation Under Very High En Route
Traffic Demand*, JAIS, DOI `10.2514/1.I010243` (their Figs. 2, 4, and 9). Two reasons:

1. It makes our multi-aircraft numbers directly comparable to the reference literature.
2. The per-run rate **saturates at 1** in dense traffic — a superconflict pins `P_run ≈ 1` and can
   no longer tell a good resolver from no resolver at all. The per-aircraft count still resolves.

### Definitions to preserve

For run `r` with `N` aircraft, true separation `d_ij(t)`:

    d_ij_min(r) = min_t d_ij(t)                       # per pair
    d_i_min(r)  = min_{j != i} d_ij_min(r)            # per aircraft
    s(r)        = min_{i<j} d_ij_min(r)               # == FleetState.min_sep today

    K(r) = #{pairs (i,j) : d_ij_min(r) < rpz}         # losing pairs
    A(r) = #{aircraft i  : d_i_min(r)  < rpz}         # aircraft involved

Metrics:

    P_run = (1/n) Σ_r 1{K(r) >= 1}                    # what we report today
    E[K]  = (1/n) Σ_r K(r)                            # loss frequency
    P_ac  = (1/(nN)) Σ_r A(r)                         # Blom's normalisation  <-- new headline
    λ     = Σ_r A(r) / Σ_r (N * T_r) = P_ac / T       # per aircraft per flight hour (random traffic)

`T_r` is the **measured** flight time (inside `MeasurementArea`), not the whole run. Exact bounds,
no assumptions:

    (2/N) * P_run  <=  P_ac  <=  P_run
    P_run          <=  E[K]

**At `N = 2` all three collapse to one number — every pairwise result is unchanged, bit for bit.**
That invariant is the safety net for the whole rewrite: if any pairwise number moves, the change is
wrong. In the rare limit `P_run ≈ (N/2) * P_ac`; at `N = 8` the old number was ~4× Blom's.

Answered design question worth keeping: `A(r)` counts an aircraft that loses separation with two
others **once** — it is a per-aircraft indicator. Multiplicity lives in `K`.

### What was attempted, by module

- **`fleet.py`** — carry per-aircraft loss flags on `FleetState` (a length-`N` bool tuple / a
  frozenset of losing pairs) beside `los`/`min_sep`. Must live *on the state* so IPS clones it.
  Mark the crossing pairs/aircraft in `_advance` where `los` is computed, under the same
  `MeasurementArea` gate as `min_sep`. Expose `K` (`n_los_pairs`) and `A` (`n_los_aircraft`) on
  `FleetOutcome`. Cost note: the per-step pairwise loop already computes every pair — this is
  bookkeeping, not new geometry.
- **`estimator.py`** — `IPRResult` records `sum_k`, `sum_a`, `n_aircraft`; derives `p_ac`, `mean_k`.
  `combine_ipr` sums the new counters. Commits 4/5 then went further: **`p_los` itself became the
  per-aircraft quantity** and the old per-run rate + `n_los` counter were deleted outright — "one
  metric under one name."
- **`ips.py`** — the IPS shells stop a particle at its **first** crossing, so `K`/`A` are not
  observable from the surviving cloud. Add a **tail leg**: after the last shell, fly each survivor
  to `is_terminal` on fresh streams so `E[A | rare set]` is *measured* rather than assumed to be 2.
  `IPS(tail=True)` became the default; without it the per-aircraft number undercounts at `N > 2`.
  Precision of the tail means is set by the number of **distinct lineages**, not `n_particles`
  (resampling fills the cloud with clones) — so report the effective sample size, `n_lineages`.

### Two things bundled into the metric commits that should have been their own change

1. **Confidence intervals were removed everywhere** — `wilson_interval`, `IPRResult.ci95`,
   `RareEventEstimate.ci`, `_log_ci`, and the `p_los_lo` / `p_los_hi` columns. Estimator agreement
   is judged on the **ratio** instead: within a factor of two, or five at `1e-4` and below where the
   Monte-Carlo anchor is itself built on few events. (The open question in the TODO — Wilson with a
   caveat vs. a run-level bootstrap for `P_ac`, since within-run per-aircraft indicators are *not*
   independent — was resolved by dropping intervals rather than answering it.)
2. **The `n_lineages` identity bug fix** — `n_lineages` was counted by object identity. Clones
   share one state object, but pickling to a worker turns that one shared object into several equal
   copies, so serial and parallel disagreed. Fix: `resample_level` returns the count taken from the
   draw. This is a real bug fix and deserves to be isolated so it is bisectable.

### Re-runs the metric change requires (N > 2 only; pairwise is untouched)

Highest value is **`converging_ring(8)`** — a superconflict where `P_run` pins near 1 while `E[K]`
and `P_ac` still resolve. That is the one figure that demonstrates *why* the metric changed; lead
with it. Others: the ring MC-vs-IPS sweep, any `random_traffic(n)` density sweep,
`swap_ring(n)` fleet/comm sweeps, `converging_ring(8)` / `swap_ring(8)` demo panels. **No re-run**
for any pairwise notebook or `scripts/handbook/*` figure — those numbers are *identical*, not
merely close.

---

## Thread B — the `scenario/` package + per-aircraft speed

Commits 1, 2. Independent of the metric work; got tangled in because it landed first.

### Goal

`scenario.py` had grown to ~496 lines doing three jobs, and `Scenario` is a contribution surface.
Make it a package following the established `base.py` + one-file-per-implementation pattern the six
other packages already use.

### What was attempted

- Split into `scenario/base.py`, `scenario/pairwise.py`, `scenario/ring.py`, `scenario/traffic.py`.
  The cut is **by encounter family, not by construct** — `random_traffic` and `RandomTraffic`
  change for the same reason, so they stay together. `__init__` re-exports everything. The move was
  meant to be mechanical; the 512 tests passing untouched is the proof.
- `scenario/README.md` documenting each scenario with two figures: a gallery of the eight
  geometries, and `swap_ring` vs `crossing_ring` at `n = 3, 4, 5` (at odd `n`, `swap_ring` aims each
  aircraft at another's *start* rather than the antipode, so routes miss the centre by 750 m at
  n=3). Text in ASD-STE100 Simplified Technical English.
- **Per-aircraft speed**: fleet builders take `speed: float | Sequence[float]`. One speed cannot
  serve a mixed fleet — `SMALL_FIXEDWING` stalls at 12 m/s, above a multirotor's 10 m/s cruise, so a
  shared speed either stalls one airframe or flies the other 40 % fast. Makes the speed *difference*
  a subject of study (needed for GA-vs-UAS encounters), not an obstacle.
- Measured while documenting: a fixed-wing does **not** fly past its final waypoint — it orbits at
  the loiter radius (81.4 m, never closer than 80). So `stop_within` must be ≥ that radius for one
  to register as arrived. Keep this fact; it is easy to rediscover the hard way.

---

## Thread C — the validation campaign, its timing, and a worker budget that lands

Commits 4, 5, 6, 7. Replaced the old ad-hoc MC-vs-IPS scripts with a structured campaign, then
made the experiment runner and the campaign timing actually usable at scale.

### Goal

Turn "MC vs IPS agreement" from a pile of one-off scripts into a reproducible campaign that (a)
covers the three encounter families, (b) caches per condition, and (c) reports **cost**, not just
agreement.

### What was attempted

- **`scripts/validation/`** — the campaign in three parts (`pairwise.py` angles, the crossing
  `ring.py`, `random_traffic.py`) at a fixed **10 m / 1 m/s** resolution, both backends per
  condition, worker count as a CLI argument, **one cache entry per condition**, and `run_all.sh` to
  run them in order. `examples/handbook/validation_campaign.ipynb` **reads that cache and simulates
  nothing**. Deleted the superseded `mc_vs_ips_campaign.py`, `plot_mc_vs_ips_campaign.py`, and a
  large pile of old probes (`bench_ips_*`, `cns_sweep`, `ips_distribution`, `ips_validate`,
  `ips_unified_validate`, `ips_validation_probe`, `ips_dcpa_prototype`, the `ips_dist_20260730`
  output, `random_spawn_conflict`).
- **Worker budget that lands (`experiment.py`, commit 5)** — `run_experiment` now spends its whole
  budget where it helps. Up to one worker per condition it fans conditions out (as before); past
  that, conditions run in turn and the budget goes **inside** each one: Monte Carlo splits its
  encounter fan-out into seed slices pooled by `combine_ipr`; IPS shards a level across workers
  (ADR 0018). **Never both** — loky pools must not nest. A sweep of three conditions can now use a
  hundred cores. `n_jobs` is deliberately **not** part of the cache key (it changes no number).
  `parallel.estimate_rare_prob` threads `tail` through `_lockstep` on the same three-wide seed tree
  `ips_once` uses, so the splitting streams are unchanged whether or not the tail runs.
- **Per-condition timing (`campaign.py`, commit 6)** — the campaign reported agreement but not
  cost. It now runs **one condition at a time (MC then IPS for that cell)** so wall time is
  attributable to the cell, not the whole sweep. Rows gain `mc_seconds`, `ips_seconds`, `gain`; the
  file gains a timing block with UTC stamps, totals, and the resolved worker count. A **cached**
  condition reads `cached` and stays out of the totals — timing a pickle read and writing it down as
  the cost of a 50,000-encounter batch would be a *wrong* number. `--no-cache` times a campaign
  whose cells are already stored. Splitting the sweep changes no number: a cell is keyed on its
  resolved values, so a one-level and a six-level declaration hit the same cache entry.
- **`update experiment` (commit 7)** — factored the shared MC core out of `_run_mc` into
  `_estimate_ipr(scenario, cfg, perf, m, jobs)`, then made the **IPS ladder pilot shard like MC
  does**. The pilot is plain Monte Carlo, so left serial it was a fixed one-core cost in front of an
  otherwise parallel cell — the share that *grows* as worker count rises. Sharing one implementation
  matters because the pilot's `min_seps` *is* the ladder (`ladder_from_record`): a second copy that
  sliced the seed tree even slightly differently would move every shell with nothing downstream
  saying so. Added `test_parallel_ips_gives_the_serial_answer_ladder_included` (one condition, four
  workers — the case that puts the budget *inside* the cell and asserts on the ladder, not just the
  numbers). Also a small mypy fix: `extra: dict[str, Any]`.

---

## Observations for the redo (factual, from the log — not a verdict on what went wrong)

- **The metric change was committed twice.** `505d046` and `ebec6e5` carry almost the same message;
  the second re-does the metric change *and* adds the worker budget + `n_lineages` fix + `tail`
  threading on top. The clean history is one metric commit, not two.
- **Two commits have empty/near-empty bodies** (`cfce9c5` "updating metrics", `5bfbc54` "update
  experiment") yet carry real, non-obvious changes — `cfce9c5` is where the whole metric rewrite and
  `TODO-metric-rewrite.md` first land. Losing the message means the reasoning only exists here now.
- **`cfce9c5` mixes concerns**: it added `TODO-metric-rewrite.md`, the fleet/estimator/ips metric
  code, `config.py` changes, five handbook notebooks + their build tools, *and* deleted vault
  observations (`fleet-ipr-sweep`, `fleet-lossy-ipr` + their images) in one commit.
- **The three threads interleave.** Scenario refactor (A), metric rewrite (B... err, thread A) and
  the campaign (C) all move together, so no single commit is a clean unit to keep or drop.

## How to re-attempt from `0295f6b`, in order

1. **Thread B first (scenario package).** Purely structural, tests prove it mechanical. Land it
   alone: the package split, then the README, then `speed: float | Sequence[float]` as its own step.
2. **The `n_lineages` identity-count bug fix** as a standalone commit — it is a real bug and should
   be independently bisectable, not buried in the metric change.
3. **Thread A (metric rewrite), split into two:**
   a. per-aircraft `P_ac` / `K` / `A` plumbed through `fleet` → `estimator` → `ips` + the IPS tail
      leg, holding the `N = 2` invariant (every pairwise number identical) as the gate.
   b. removing confidence intervals + switching agreement to the ratio criterion — a genuinely
      separate decision, so a separate commit.
4. **Thread C (campaign + scale):** `scripts/validation/` and the cache-reading notebook; then the
   worker-budget change in `experiment.py` (with the ladder-pilot sharding and its test); then the
   per-condition timing in `campaign.py`.
5. **Re-runs and docs last**, led by `converging_ring(8)` — the figure that justifies the whole
   metric change — plus the site pages (`docs/estimators/monte-carlo.md` "what the numerator counts",
   `docs/estimators/index.md`, the `ring`/`random-traffic` scenario pages).

Give each commit a real body. The reasoning above is exactly what those bodies should say.
