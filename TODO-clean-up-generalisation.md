# TODO — clean-up: generalise N, and break up `experiment.py`

Deferred work, written down while the reasons are fresh. **Not part of
[`TODO-metric-rewrite-parallel.md`](TODO-metric-rewrite-parallel.md)** — that plan should finish
first; this one is the tidy-up its later items will make obvious. Everything below was checked
against the tree, not remembered.

The theme: the simulator is N-agnostic, but **N = 2 is still wired into the layers above it** —
the estimator's siblings, the config, and the experiment vocabulary. Each place is small; together
they are why "run a fleet study" is still not the same motion as "run a pairwise study".

---

## What is actually still N = 2 bound (verified)

| # | Where | What |
|---|---|---|
| ~~1~~ | ~~`_run_ips`~~ | ✅ **DONE** — pulled into the current plan. `Methods.scenario` is a declared component (default `pairwise`), and one `_encounter_builder()` serves both backends. A fleet study now reaches MC *and* IPS from one declaration, and is sweepable. |
| 2 | `experiment.py:89` | `_GEOMETRY_SLOTS = {dpsi, dcpa, side, gs_intr}` — the declarable geometry vocabulary is a hardcoded pairwise allowlist. **Softened, not solved:** a scenario binds its own size at construction and is swept via the `scenario` axis, so this no longer blocks fleet work; it still means pairwise pins are privileged names. |
| 3 | `config.py:17-29` (`ScenarioConfig`) | `dcpa_max` and `tlos` are **pairwise** encounter geometry, but live in the global `Config` every scenario must carry. |
| 4 | `experiment.py` (`Methods.airframes`) | one `Airframe` per aircraft, "ownship first" — a 2-element convention. |
| 5 | `estimator.py` (`pairwise`) | the two-aircraft builder lives *inside the estimator module* rather than beside the other scenarios. |

### ~~The sharp one: #1~~ — fixed, 2026-08-06

Kept below for the reasoning; the fix is described in the row above and in
`TODO-metric-rewrite-parallel.md`.

<details><summary>original entry</summary>

#### #1 — the two backends disagree about what an encounter is

`_run_mc` now says *"here is a builder, fly it"*. `_run_ips` still opens `sample_pairwise`, unpacks
`(own, intr)`, and then **re-implements the agent + env construction** (`experiment.py:422-453`)
that the MC path gets from the builder. So:

- an IPS fleet study is **unreachable** through `run_experiment`, even though `ips.estimate_rare_prob`
  itself takes a `build_initial` and has always been N-agnostic;
- the same encounter is described **twice, differently**, in one file — exactly the drift the
  `_estimate_ipr`/ladder-pilot sharing in the old attempt existed to prevent;
- **item 4 (the validation campaign) compares MC against IPS per condition.** It cannot do that on a
  ring or on random traffic until both backends take the same builder.

**This is the one piece of the list that is not cosmetic.** It is arguably in scope for the current
plan rather than here — see "Sequencing".

</details>

---

## Breaking up `experiment.py`

**1018 lines** today. It already carries its own seams as `# ---` banners, and they map almost 1:1
onto modules — the split is close to mechanical:

| Lines | Banner | → module | ~size |
|---|---|---|---|
| 75-148 | what can be declared | `declaration.py` (`Fixed`, `Sweep`, `Axis`) | 75 |
| 149-199 | backends | `backends.py` (`MC`, `IPS`) | 50 |
| 200-248 | the methods bundle | `methods.py` (`Methods`) | 50 |
| 249-294 | conditions | `conditions.py` (`Condition`, `expand`) | 45 |
| 295-479 | running one condition | `cell.py` (`_run_mc`, `_run_ips`, `_config_for`, …) | 185 |
| 480-644 | cache identity | `identity.py` (`identity`, digests, `CacheConfig`) | 165 |
| 645-817 | results | `results.py` (`ExperimentResult`, `_metrics`) | 170 |
| 818-889 | provenance | `card.py` (`_write_card`) | 70 |
| 890-1018 | the entry point | `__init__.py` (`run_experiment`, `run_one_experiment`) | 130 |

Notes:

- `identity.py` is the most independent chunk (source digests, hashing, cache keys) — it has almost
  nothing to do with experiments and would read better standing alone. Distinct from the existing
  `opencdarr/cache.py`; name them so nobody has to guess which is which.
- `cell.py` is where #1 above gets fixed: `_run_mc` and `_run_ips` become one "build the encounter,
  hand it to a backend" path.
- Follow the established `base.py` + one-file-per-implementation pattern the other six packages use,
  and re-export from `__init__` so no import moves.
- Do it **after** the behavioural changes land, not during — a move commit whose tests pass untouched
  is the proof it was mechanical (same argument as the `scenario/` split, item 3 of the other plan).

---

## `pairwise()` and `run_encounter` — half agreed

### `pairwise()`: keep it, but move it

Not redundant — a two-aircraft encounter is a real research object, and it is the *only* geometry the
published pairwise pages use. What is wrong is **where it lives**: `estimator.py`, as though the
estimator had a favourite scenario. Once `scenario/` is a package (item 3 of the other plan), it
belongs there beside `ring` and `traffic`, and `sample_pairwise` with it. Then:

    scenario.pairwise(...)     ->  2 agents
    scenario.ring(n, ...)      ->  n agents      # same type, same call shape
    scenario.traffic(n, ...)   ->  n agents

...and the estimator imports none of them. That is the version where N genuinely stops being special:
the estimator has no pairwise import at all, rather than one it happens not to default to.

### `run_encounter`: probably delete — but it costs an oracle

`loop.py` is 331 lines and `run_fleet` at n = 2 reproduces it **bit-for-bit** (asserted in
`tests/test_fleet.py`). As a runtime path it is dead weight: `estimate_p_los` and IPS both go through
`run_fleet`. Remaining users are the top-level export, `tests/`, and two scripts
(`mixed_fleet_demo.py`, `mixed_fleet_daa_demo.py`).

**But** that bit-for-bit test is only meaningful because `run_encounter` is an *independent
implementation*. Delete it and the assertion becomes `run_fleet == run_fleet`; the n = 2 reduction
stops being checked against anything. So this is a genuine trade, not an obvious win:

- **Delete** — one runner, 331 fewer lines, no chance of the two drifting. The n = 2 reduction is
  then guarded by the golden anchors (`test_golden_ipr_at_midrange_noise` and friends) instead of by
  a parallel implementation.
- **Keep** — as a deliberately frozen reference implementation, documented as such, not as a runner
  anyone should call. Costs 331 lines of maintenance that must track every model change.

Decide explicitly; do not let it rot into "kept because nobody dared".

---

## The end state worth aiming at

1. **One encounter interface.** `EncounterBuilder` (or a `Scenario` producing one) is what both
   backends take. `estimate_p_los(build, …)` and `estimate_rare_prob(build_initial, …)` already have
   the same shape — finish the job so `run_experiment` drives them identically.
2. **`Config` holds numerics, scenarios hold geometry.** `dt`, `t_max`, `rpz`, `seed`,
   `n_encounters` are universal; `dcpa_max` / `tlos` belong to the pairwise scenario that uses them.
   Today a ring study must fill in a `dcpa_max` that means nothing to it.
3. **Declaration is not an allowlist.** A sweep should be able to declare `n`, `radius` or
   `density` the same way it declares `dpsi` — the axis machinery is generic, only
   `_GEOMETRY_SLOTS` is not.
4. **`experiment/` is a package** of ~8 focused modules, none over ~200 lines.
5. **N appears nowhere above `run_fleet`** except as `len(agents)`.

---

## Sequencing

- **Now (current plan):** finish the metric rewrite + reachable IPS parallelism.
- **Pull forward?** #1 (IPS takes a builder) is a prerequisite for a fleet-capable item 4 campaign,
  and it is small now that `EncounterBuilder` exists. Strong candidate to fold into the current plan
  rather than defer.
- **With item 3 (`scenario/` package):** relocate `pairwise` + `sample_pairwise`.
- **After the plan lands:** the `experiment/` split (mechanical, tests unchanged), then
  `Config`/`ScenarioConfig` separation, then the `run_encounter` decision.

Do the behavioural fixes first and the file moves last, so every move commit can prove itself by
leaving the tests untouched.
