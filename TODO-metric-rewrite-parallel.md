# TODO — metric rewrite + reachable IPS parallelism + scenario package + validation campaign

Clean redo plan, to be executed from **`0295f6b`** ("Campaign runner for the ring and
random-traffic studies") after reverting. The previous attempt (`0295f6b..5bfbc54`) had the right
intent and bad execution — see [`RETRO-0295f6b-to-5bfbc54.md`](RETRO-0295f6b-to-5bfbc54.md) for the
full record and rationale. This file is the actionable checklist. Backward compatibility is not a
concern (single user).

This file is **untracked** — it survives `git reset --hard 0295f6b`. A `git clean -fd` would remove
it; move it out first if you clean.

## Order & dependencies

Ordered so each item has its prerequisites already in place:

- **Items 1 and 2 are the two halves of the IPS estimator — correct, then fast.** Both live in
  `ips.py` / `parallel.py` and share one serial-vs-parallel gate. Item 1's `n_lineages` fix is
  exactly what makes item 2's gate pass, so do them **back to back**.
- **Item 3 (scenario) is independent and mechanical.** It sits before the campaign because the
  campaign's parts are written against the scenario package. Its `__init__` re-exports keep imports
  stable, so item 1's re-runs are not invalidated by the refactor.
- **Item 4 (campaign) comes last, after both its prerequisites** — parallelism (item 2) for scale,
  the scenario package (item 3) for its parts. No "build now, run at scale later" split.

Each item is one or more **separate commits** with real message bodies. The previous attempt's worst
habit was one commit doing five things — do not repeat it.

---

## Prep — before the first commit

- [x] **Safety net in place:** `attempt/metric-rewrite-v1` branches `5bfbc54`, so the whole prior
  attempt stays cherry-pickable (scenario split, tail-leg impl, `n_lineages` fix) and the revert is
  reversible.
- [ ] **`git reset --hard 0295f6b`** — the revert. It spares every untracked file; **do not
  `git clean`**. Untracked survivors to keep: `RETRO-0295f6b-to-5bfbc54.md`, this file, and
  `vault/decisions/0022-per-aircraft-los-normalisation.md`.
- [x] **Green baseline captured** → `BASELINE-0295f6b.md`. **496 tests pass** (exit 0). `ruff 0.15.22`
  is **not** clean at the base (8 in `opencdarr/`, 519 repo-wide) — assessed as version drift, not a
  regression, so the ruff gate is **"no new findings on touched files vs. baseline,"** not zero. The
  pairwise invariance oracle is the in-suite golden anchor (item 1) — the rescued `0.050/53.3` number
  did not reproduce and was corrected there.
- [ ] **ADR 0022** reserved (next free number at `0295f6b`); stub written in `vault/decisions/`, body
  finalised with the implementation.
- [ ] **Cache** is cleared before the item-4 campaign — the schema + code-hash change makes old
  pickles stale (also noted there).

---

## 1. Metric rewrite: per-run → per-aircraft P(LoS)

**Goal.** Report a **per-aircraft** loss probability (Blom & Bakker 2015, JAIS,
DOI `10.2514/1.I010243`, Figs. 2/4/9), not a per-run one. The per-run rate saturates at 1 in dense
traffic — a superconflict pins `P_run ≈ 1` and cannot tell a good resolver from none. Per-aircraft
still resolves, and is directly comparable to the reference literature.

### Definitions

For run `r` with `N` aircraft, true separation `d_ij(t)`:

    d_ij_min(r) = min_t d_ij(t)                     # per pair
    d_i_min(r)  = min_{j != i} d_ij_min(r)          # per aircraft
    s(r)        = min_{i<j} d_ij_min(r)             # == FleetState.min_sep today

    K(r) = #{pairs (i,j) : d_ij_min(r) < rpz}       # losing pairs
    A(r) = #{aircraft i  : d_i_min(r)  < rpz}       # aircraft involved

    P_run = (1/n) Σ_r 1{K(r) >= 1}                  # old headline
    E[K]  = (1/n) Σ_r K(r)                          # loss frequency
    P_ac  = (1/(nN)) Σ_r A(r)                       # NEW headline (Blom)
    λ     = P_ac / T                                # per aircraft per flight hour — DEFERRED, not this rewrite

`T_r` = measured flight time inside `MeasurementArea`, not the whole run. Bounds:
`(2/N)·P_run ≤ P_ac ≤ P_run` and `P_run ≤ E[K]`.

**Code names (all three are reported — P_ac is only the *headline*, not the only metric):**
`p_los_ac` = P_ac (per aircraft, Blom), `p_los_run` = P_run (per run, = today's `p_los`),
`mean_k` = E[K] (loss frequency, mean losing pairs). The shared `p_los_` prefix marks the two
probabilities as one quantity under two normalisations; `mean_k` stays outside it because it is a
frequency, not a probability. **Drop the bare `p_los`** — every reference must say which. At
`N = 2`, `p_los_ac == p_los_run`, so the pairwise pages cite `p_los_ac` (its N=2 special case).
**`λ` (per aircraft per flight hour) is deferred** — not computed or reported in this rewrite.

**Gate: at `N = 2` all collapse to one number.** Every pairwise result must be bit-identical. If any
pairwise number moves, the change is wrong. `A(r)` counts a doubly-involved aircraft **once**;
multiplicity lives in `K`.

### Test oracles (rescued from the now-deleted `TODO-metric-rewrite.md`)

Two hand-verified references — write them as **locked tests before** wiring the estimator/IPS layers:

1. **Pairwise invariance — the N=2 gate (authoritative: the in-suite golden anchor).**
   `tests/test_estimator.py::test_golden_ipr_at_midrange_noise` already pins, bit-exact and passing
   at `0295f6b`: `(n_los, n_conflict) = (22, 200)`, `ipr = 0.89`, `median_min_sep = 126.45469556207351`
   (rel 1e-8). The rewrite must keep these — with `p_los_run = 1 − ipr = 0.11` and the `ipr` assert
   migrated to `p_los_run`. **This is the real guard** — keep/extend it, don't invent one.

   > **Correction from the baseline capture:** the number rescued from the old TODO —
   > `pairwise.yaml`, seed 0, 500 enc, `dpsi=2` → `p_los=0.050`, `median=53.3` — was recorded at a
   > *later* state (post-`d9067f3`) and does **not** reproduce here. Fresh `0295f6b` values for that
   > run: MVP → `p_los=0.0`, `median=53.607`; no-resolver → `p_los=1.0`, `median=22.256` — both
   > degenerate, so the mid-range golden anchor (0.11) is the useful one. See `BASELINE-0295f6b.md`.

2. **Fleet K/A invariants** (rpz 50 m, seed 7) — ground truth for the `_advance` counting:

   | fleet | resolver | K | A | min_sep |
   |---|---|---|---|---|
   | `swap_ring(4)` | none | 6 (all pairs) | 4 (all aircraft) | 0.0 |
   | `swap_ring(4)` | MVP | 0 | 0 | 102.7 |
   | `converging_ring(6)` | none | 15 (all pairs) | 6 | 0.0 |
   | `converging_ring(6)` | MVP | 10 | 6 | 41.2 |

   Last row in one line: `los` is `True` either way, so `p_los_run` cannot tell MVP from no resolver,
   while `K` drops 15 → 10 — the whole argument for the metric, on one geometry.

### Steps

- [x] **`fleet.py`** ✅ — `FleetState` carries a `frozenset[tuple[int, int]]` of losing pairs beside
  `los`/`min_sep` (pairs, not a bool-tuple — one field gives `K = len(...)` and `A = |⋃ aircraft|`),
  on the state so IPS clones it. `advance` forms the per-pair ranges once (their `min` *is* the old
  scalar, so `los`/`min_sep` stay bit-identical) and unions the pairs that crossed `rpz`. **No
  `MeasurementArea` exists at `0295f6b`** (that was the later attempt) — so this mirrors `min_sep`:
  every step. `FleetOutcome` exposes `n_los_pairs` (K) / `n_los_aircraft` (A). Purely additive: 500 pass
  (496 + 4) — `tests/test_fleet_los_pairs.py` pins the geometry invariants + MVP golden anchors;
  `tests/test_per_aircraft_normalisation.py` measures the premise (p_los_ac == p_los_run at N=2,
  strictly below it at N=4).
- [x] **`estimator.py`** ✅ — the whole IPR cluster retired: `IPRResult` → **`MonteCarloEstimate`**,
  `estimate_ipr` → **`estimate_p_los`**, `combine_ipr` → **`combine_p_los`** (underscore spelling,
  per decision). Records `sum_k`, `sum_a`, `n_aircraft`; reports **`p_los_ac`** (headline, Blom),
  **`p_los_run`** (the old `p_los`, kept), **`mean_k`** (E[K]). Dropped the bare `p_los` and the
  `ipr` property. `ci95`/`wilson_interval` kept (removal is its own commit); interval columns renamed
  `p_los_run_lo/hi`; `_metrics` reports the three, plot defaults to `p_los_run` (→ `p_los_ac` once
  IPS reports it — tail leg). Golden anchor migrated in place (`ipr==0.89` → `p_los_run==0.11` + the
  N=2 identity). 12 files, **503 tests pass**, ruff clean.
- [x] **`estimate_p_los` generalized to any N** ✅ — takes an `EncounterBuilder`
  (`(rng, config) -> list[Agent]`) as its **required** first argument; `N = len(agents)` and the
  denominator is `sum_n` (Σ N over runs, so a variable-N builder is normalised honestly) rather than
  an assumed `n_encounters · 2`. `pairwise(perf, kinematics=…, airframes=…, dpsi=…, …)` is the
  shipped two-aircraft builder and now owns every pairwise-only knob, so the estimator itself says
  nothing about N — one construction path, no vestigial kwargs. Mirrors IPS's `build_initial`, which
  is what lets the campaign (item 4) drive both backends identically. Verified through the estimator:
  N=2 → `p_los_run == p_los_ac == 0.5167`; N=4 (two independent pairs) → `p_los_run 0.7667`
  saturating above `p_los_ac 0.5333`, `mean_k 1.0667 > 1`. Golden anchor unmoved. **504 tests pass**.
- [x] **Both backends take the scenario** ✅ *(pulled forward from
  `TODO-clean-up-generalisation.md` #1)* — `_run_ips` was still opening `sample_pairwise` inline, so
  an IPS fleet study was unreachable and the campaign could not compare backends on a ring.
  `Methods.scenario: ScenarioFactory = pairwise` makes the encounter model a declared component;
  one `_encounter_builder()` serves `_run_mc` and `_run_ips`. Because it is a `Methods` field it is
  swept by the existing override machinery — `scenario=Sweep([4, 8], build=…, name="n")` is a
  fleet-size axis, no `_GEOMETRY_SLOTS` change. RNG draw order preserved, so every IPS number is
  unchanged. Demonstrated end-to-end on `converging_ring(6)` through `run_experiment`:
  MC `p_los_run 1.000` (saturated) vs `p_los_ac 0.500`, `mean_k 2.00`; IPS `prob 1.000`.
  **506 tests pass.**
  **`RareEventEstimate` is deliberately *not* renamed to match** — `MC → MonteCarloEstimate` is an
  obvious mapping, and "rare event" names the estimator's purpose better than its mechanism
  (splitting) would; the mixed naming axis (backend vs. quantity) is a chosen trade, not an oversight.
- [x] **`ips.py` — the tail leg** ✅ — `_evolve_to_terminal` flies the final cloud past its first
  breach; `IPS(tail=True)` is the default. `RareEventEstimate` now reports `p_los_run` (ladder),
  `p_los_ac` and `mean_k` (tail), plus `n_lineages`; `prob` kept as a property alias. Combined
  **per replication** (each rep's own `P̂ × E[·|rare]`), so each term stays unbiased and a collapsed
  rep contributes the zero it found; `nan` when the tail did not run — absent ≠ zero. Rides a
  **third** seed child, so the ladder is bit-identical with the tail on or off (asserted).
  `n_lineages` comes from the resampling **draw**, not object identity — the fix that makes serial
  and parallel agree after pickling. `tail` threaded through `parallel.py` too: serial == sharded
  lockstep on every field. **Validated against MC**: N=2 → all three coincide (ratio 1.00×);
  N=4 → `p_los_ac` 0.4968 (MC) vs 0.5300 (IPS), `mean_k` 0.9937 vs 1.0600 — ratio 1.07×, inside
  the factor-of-two criterion. **512 tests pass.**
- [ ] ~~**`ips.py` — the tail leg.**~~ *(original entry below)* Shells stop a particle at its
  **first** crossing, so `K`/`A` are
  not observable from the surviving cloud. The ladder gives `p_los_run` natively; add a continuation
  leg for the per-aircraft number: after the last shell, fly each survivor to `is_terminal` on fresh
  streams and report `p_los_ac = p_los_run · mean(A | survivors) / N`. Make `IPS(tail=True)` the
  **default** — without it `p_los_ac` undercounts at `N > 2`.
- [ ] **`n_lineages` (effective sample size of the tail).** Precision of the tail means is set by the
  number of **distinct lineages**, not `n_particles` (resampling fills the cloud with clones). Count
  it from the **draw** (`resample_level` returns the count taken), **not by object identity** —
  clones share one state object, and pickling to a worker turns that one object into several equal
  copies, so identity-counting makes serial and parallel disagree. This is item 2's gate; get it
  right here.

### Separate decision — drop confidence intervals (own commit)

- [x] ✅ **Done.** Removed `wilson_interval`, `MonteCarloEstimate.ci95`, `RareEventEstimate.ci`,
  `_log_ci`, the `p_los_run_lo/hi` columns and `plot()`'s shaded band. Agreement is judged on the
  ratio. **511 tests pass**, ruff clean on every touched file. ⚠️ **Left broken:** four superseded
  scripts (`mc_vs_ips_campaign`, `ips_validate`, `ips_distribution`, `ips_unified_validate`) — all
  already on item 4's delete list, and two were *already* broken by the earlier `estimate_ipr`
  rename (nothing caught it: `testpaths = ["tests"]`, so `scripts/` is never imported).
- [ ] ~~Remove~~ *(original entry)* `wilson_interval`, `IPRResult.ci95`, `RareEventEstimate.ci`,
  `_log_ci`, and the
  `p_los_lo` / `p_los_hi` columns. Judge estimator agreement on the **ratio**: within a factor of
  two, or five at `1e-4` and below (where the MC anchor itself rests on few events). This sidesteps
  the unanswered question of whether within-run per-aircraft indicators (not independent) need a
  run-level bootstrap vs. a caveated Wilson. Keep it out of the metric-plumbing commit.

### Return shape

`run_experiment(...)` → **`ExperimentResult`**. Surface: `.records()` (one dict per condition =
`{**swept levels, **_metrics(result)}`), `.frame()` (the same rows as a DataFrame), `.cell(**levels)`
(the raw per-cell estimate), `.plot(metric="p_los_ac")` (default was `"p_los"`; the shaded CI band
goes away with the intervals). `_metrics()` is the **single** place the three names are emitted.

Record columns — intervals removed (no `p_los_lo` / `p_los_hi`) and the now-ambiguous `ipr` dropped:

- **MC** (`MonteCarloEstimate`): `p_los_ac`, `p_los_run`, `mean_k`, `median_min_sep`,
  `n_encounters`, `detection_rate`
- **IPS** (`RareEventEstimate`): `p_los_ac`, `p_los_run`, `mean_k`, `n_lineages`, `n_collapsed`,
  `reps`

Raw objects via `.cell()` — three derived properties over stored counters:

    MonteCarloEstimate   stored: min_seps, sum_k, sum_a, n_aircraft   (n_encounters = len(min_seps))
        p_los_run = Σ 1{K>=1} / n_encounters       # == the old p_los
        p_los_ac  = sum_a / (n_encounters * N)      # Blom
        mean_k    = sum_k / n_encounters
    RareEventEstimate    p_los_run native (the ladder probability); p_los_ac & mean_k from the tail
        (mean_k = p_los_run * mean(K | survivors)); plus n_lineages, n_collapsed, reps

A row at **N=2** has the three equal, so pairwise rows are unchanged. At **N>2** they diverge — a
`converging_ring(8)` row reads `p_los_run ≈ 1` (saturated), `p_los_ac` still resolving, `mean_k > 1`.

### Re-runs (after the code lands)

Pairwise notebooks and every `scripts/handbook/*` figure are **identical** — no re-run. Re-run only
`N > 2` artifacts present at `0295f6b`. **Lead with `converging_ring(8)`**: a superconflict where
`p_los_run` pins near 1 while `mean_k` and `p_los_ac` still resolve — the one figure that justifies
the change (and the reason `p_los_run` is kept rather than deleted).

---

## 2. Make IPS parallelism reachable from `run_experiment`

> **Checked at `0295f6b`: NOT reachable.** `experiment.py` imports the *serial*
> `ips.estimate_rare_prob`; `_run_ips`/`_run_mc` take no `jobs` arg; `run_experiment`'s `n_jobs`
> fans **conditions** across processes only. The intra-condition engine
> (`parallel.estimate_rare_prob`, `_lockstep`, `_shard_count`, `ips_replications`, ADR 0018
> sharding) exists in `opencdarr/parallel.py` but is reachable only by calling it directly. So a
> single IPS condition runs serially and leaves cores idle. **This item is needed.**

**Goal.** A worker budget that lands: `run_experiment` spends its whole budget where it helps. Do it
right after item 1 — the tail leg and `n_lineages` it threads through are freshly in hand.

- [x] ✅ **Done.** `run_experiment` now spends the budget where it helps: serial at 1 worker,
  fan conditions out while `workers <= len(conditions)`, and past that run the conditions in turn
  with the whole budget **inside** each cell. Never both, so no nested loky pools. `_run_mc` slices
  the encounter fan-out into contiguous `children(root, lo, hi)` and pools with `combine_p_los`;
  `_run_ips` hands off to `parallel.estimate_rare_prob` (ADR 0018 sharding). `n_jobs` is not in the
  cache key. **517 tests pass**; the new `tests/test_experiment_parallel_budget.py` asserts equality
  against the serial answer at 1/2/8 workers, for both backends, tail fields included.

  **Measured at the rare setting** (`pos_ci95 = 10 m`, the recorded 4.7e-4 rung), one condition,
  `n_jobs=-1` on 8 cores — the case that previously used a single core:

  | | estimate | cost |
  |---|---|---|
  | MC, 120 000 encounters | `4.417e-4` (53 events) | 322 s |
  | IPS, 8 reps x 800 particles, 7 shells | `5.701e-4`, 0 collapsed | 57 s |

  Agreement **1.29x** (criterion: 5x at 1e-4 and below). **IPS 5.6x faster.** MC at 4 000
  encounters finds *one* event — the starvation IPS exists to avoid. The MC anchor also reproduces
  the vault's recorded 4.7e-4 independently, which is a check on the whole rewrite.

- [ ] ~~Up to one worker per condition~~ *(original entry)* → fan conditions out. **Past that** →
  conditions run in turn and the budget goes **inside** each: MC splits its encounter fan-out into
  seed slices pooled by `combine_ipr`; IPS shards a level across workers via
  `parallel.estimate_rare_prob` (ADR 0018). **Never both** — loky pools must not nest.
- [ ] Give `_run_ips` / `_run_mc` a `jobs` parameter and thread it from `run_experiment`. Wire
  `_run_ips` to the **parallel** `estimate_rare_prob` when `jobs > 1`.
- [ ] Thread `tail` through `parallel.estimate_rare_prob` → `_lockstep` on the **same three-wide seed
  tree** `ips_once` uses, so the splitting streams are identical whether or not the tail runs.
- [ ] **`n_jobs` is not part of the cache key** — it changes no number.
- [ ] **Ladder pilot shards too.** The `Ladder` pilot is plain Monte Carlo; left serial it is a
  fixed one-core cost in front of an otherwise parallel cell — the share that *grows* with worker
  count. Factor the shared MC core into one `_estimate_ipr(scenario, cfg, perf, m, jobs)` used by
  both `_run_mc` and the pilot. One implementation matters because the pilot's `min_seps` **is** the
  ladder (`ladder_from_record`): a divergent copy would move every shell silently.

### Gate / tests (this item is correctness-critical)

- [ ] Serial == parallel **value for value, including every tail field**.
- [ ] `run_experiment` gives **identical `p_los_ac` (and every other field) at 1, 2, and 8
  workers**; MC wall time drops.
- [ ] A **one-condition, four-worker** test — the case that puts the budget *inside* the cell and
  exercises the intra-cell seam (`test_parallel_ips_gives_the_serial_answer_ladder_included`). Assert
  on the **ladder** as well as the numbers: a pilot that sliced the seed tree differently would place
  every shell elsewhere and still return a plausible `p_los_ac` — a wrong answer with nothing that
  looks wrong.

---

## 3. `scenario/` package + per-aircraft speed

**Goal.** `scenario.py` is ~496 lines doing three jobs, and `Scenario` is a contribution surface.
Make it a package on the established `base.py` + one-file-per-implementation pattern.

- [ ] Split into `scenario/base.py`, `scenario/pairwise.py`, `scenario/ring.py`,
  `scenario/traffic.py`. Cut **by encounter family, not construct** (`random_traffic` and
  `RandomTraffic` change together → one file). `__init__` re-exports everything. The move is
  mechanical; **tests passing untouched is the proof** — land it as its own commit.
- [ ] `scenario/README.md`: each scenario with two figures — a gallery of the eight geometries, and
  `swap_ring` vs `crossing_ring` at `n = 3,4,5` (odd `n`: `swap_ring` aims at another aircraft's
  *start*, so routes miss centre by 750 m at n=3). Text in ASD-STE100 Simplified Technical English.
- [ ] **Per-aircraft speed**: fleet builders take `speed: float | Sequence[float]`. One speed cannot
  serve a mixed fleet — `SMALL_FIXEDWING` stalls at 12 m/s, above a multirotor's 10 m/s cruise.
  Makes the speed *difference* a subject of study (GA-vs-UAS), not an obstacle. Own commit.
- [ ] **Keep this fact:** a fixed-wing does not fly past its final waypoint — it orbits at the loiter
  radius (81.4 m, never closer than 80). So `stop_within` must be ≥ that radius for one to register
  as arrived.

---

## 4. Validation campaign

**Goal.** Replace the ad-hoc MC-vs-IPS scripts with a reproducible campaign that reports **cost**,
not just agreement. Both prerequisites are in place by now — parallelism (item 2) for scale, the
scenario package (item 3) for its parts — so build **and** run it at scale directly.

> **First, clear the cache dir.** The rewrite changed the schema and the code hash, so any pickles
> from the old attempt are stale and must not be read back.

- [ ] **`scripts/validation/`** — three parts (`pairwise.py` angles, crossing `ring.py`,
  `random_traffic.py`) at a fixed **10 m / 1 m/s** resolution, both backends per condition, worker
  count as a CLI argument, **one cache entry per condition**, and `run_all.sh` to run them in order.
- [ ] `examples/handbook/validation_campaign.ipynb` **reads that cache and simulates nothing**.
- [ ] Delete the superseded probes as they are replaced (`mc_vs_ips_campaign.py`,
  `plot_mc_vs_ips_campaign.py`, `bench_ips_*`, `cns_sweep`, `ips_distribution`, `ips_validate`,
  `ips_unified_validate`, `ips_validation_probe`, `ips_dcpa_prototype`, old campaign output).
- [ ] **Per-condition timing.** Run **one condition at a time (MC then IPS for that cell)** so wall
  time is attributable to the cell, not the whole sweep. Rows gain `mc_seconds`, `ips_seconds`,
  `gain`; the file gains a timing block with UTC stamps, totals, resolved worker count. A **cached**
  condition reads `cached` and stays **out of the totals** (timing a pickle read as the cost of a
  50k-encounter batch is a wrong number). `--no-cache` re-times cells that are already stored.
  Splitting the sweep changes no number — a cell is keyed on its resolved values.

---

## Done when

- [ ] Pairwise numbers bit-identical to `0295f6b`; `N > 2` re-run, led by `converging_ring(8)`.
- [ ] `run_experiment(n_jobs>1)` on a single IPS condition uses all the cores and returns the serial
  answer exactly.
- [ ] `scenario/` is a package, tests unchanged; README with both figures; mixed-speed fleets work.
- [ ] `scripts/validation/` + cache-reading notebook produce a timed MC-vs-IPS table.
- [ ] Every commit has a real body. Full test suite green (496 at base); **no new ruff findings on
  touched files** vs. the `0295f6b` baseline (the base is not clean under ruff 0.15.22 — version drift).
