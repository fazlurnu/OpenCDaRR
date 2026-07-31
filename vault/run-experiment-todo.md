# `run_experiment` — build order

Working checklist for the v1 slice agreed in the review of [[run-experiment-design]]. One item at a
time, in order; each is independently verifiable and leaves `pytest` green before the next starts.

**Scope decisions settled in review (2026-07-29):**

- **Audience: contributors**, not paper reproduction. The V1.0/V1.1 gates in [[TODO]] are the target:
  write your own CD/CR/CRR → run under MC → run under IPS → write your own dynamics. So v1's job is
  *the same user objects run unchanged under both backends, in three lines*.
- **Estimand: unconditional** over encounters (not the paper's `n_conf` denominator). Store all
  three counts so the paper's conditional IPR stays derivable.
- **Anisotropic noise orientation: unresolved**, spun off as its own investigation. Does not block
  the signature widening.
- **Result object:** `frame()` + `cell()` + `plot()`, all three.

Deferred out of v1 (they belong to a later paper-reproduction driver): `Random`/quadrature marginals
and their CI treatment, ragged/derived/zipped axes, the `Conditioned` latent-event role, pluggable
IPS `level=`, AMS adaptive shells, phase-9 state accumulators, `n_particles` seed-tree decoupling.

---

## 1. Wire the code hash into the experiment card — **done 2026-07-29**

- [x] Replace the literal `code_hash: (deferred)` in `experiment.py:_write_card` with
      `cache.code_fingerprint()`. Stale module docstring ("A code-hash stamp is deferred") updated
      to match. Full suite green (331 passed); a card now reads `code_hash: 4985ab5efa747ce1`, and
      the digest was checked against an independent reimplementation and shown to move when any
      `opencdarr/*.py` byte changes.

`opencdarr/cache.py` has computed this since it landed — a sha256 over every `opencdarr/**/*.py`,
memoised per process. The card has simply never called it. This is the `config + seed + code-hash →
result` contract the whole reproducibility story rests on
(`design_brief.md`: "Never trust a 1e-9 number that isn't anchored to something checkable").

- **Files:** `opencdarr/experiment.py`
- **Verify:** `pytest tests/test_experiment.py`; the card's hash changes when any `opencdarr/*.py`
  changes and is stable otherwise. No existing test pins the placeholder string.

## 2. Route plain MC through `build_env` / `run_fleet` — **done 2026-07-29**

- [x] `estimate_ipr` now runs each encounter through `run_fleet` at n=2 (so `build_env` /
      `advance` / `is_terminal` — the same interface IPS drives) instead of calling
      `run_encounter`. Gained three keyword-only arguments that were previously reachable through
      IPS but not through MC: `dynamics`, `wind`, `share_intent`. `config.simulation.
      broadcast_interval` maps to `BroadcastSchedule(interval=...)` — aligned phase, no jitter, so
      no broadcast stream is ever drawn from and the substream tree is unchanged (ADR 0006 §6).

**Verified.** Parity was established *before* the refactor, then re-checked after:

- New reduction tests over the estimator's actual crossing-angle support — `dpsi ∈ {5, 45, 90, 180,
  270, 355}` × both sides, noiseless and noisy, on `estimate_ipr`'s own `spawn(seq, 3)` layout.
  The pre-existing reduction tests each pinned a single geometry (90°, 45°), so the sweep ends —
  where the closing geometry is most degenerate — were untested.
- Old-vs-new parity: the pre-refactor body reconstructed and compared across five CNS regimes
  (noiseless ± resolver, nav noise with MVP+PastCPA and with VO+FTR, nav noise + lossy comm) —
  identical counts throughout. Then a sharper per-encounter check on *sampled* geometry comparing
  exact float `min_sep`: **0 mismatches in 480 encounters**, at noise levels giving 0, 2, 16 and 47
  LoS out of 120 (so the comparison actually discriminates rather than matching 0 against 0).
- Two permanent anchors added to `tests/test_estimator.py`, which previously pinned only
  *relations* (reproducible, pools, above baseline) and so would have accepted a refactor that
  changed every number: a **golden anchor** on exact counts (22/200, IPR 0.89 at pos_ci95=60), and
  `test_dynamics_reaches_the_mc_path`, the regression that would have caught this defect. The
  latter asserts an exact identity — an airframe that discards resolution commands must give
  bit-for-bit what flying with no resolver gives — rather than a loose inequality.
- Full suite 343 passed; `ruff` clean on the touched files; `mypy` clean on them (the 12 remaining
  errors are the pre-existing ones [[TODO]] #5 catalogues in other modules);
  `scripts/ips_validate.py --pos 40` still returns **PASS** with MC and IPS CIs overlapping.

**Deliberately still single-airframe.** `Agent` supports per-aircraft `dynamics`/`perf` (mixed
fleet, ADR 0011 §7) but `estimate_ipr` passes one of each to both aircraft, because `Config` carries
one `aircraft_type`. A mixed-fleet MC sweep is a separate change with its own config question.

**The defect this fixes.** `estimate_ipr` calls `run_encounter(...)` without passing `dynamics=`, so
MC always uses the default `Multirotor` while IPS honours `Agent.dynamics` via `build_env`. A
contributor's custom `Dynamics` is **silently ignored on MC and works on IPS**. Same for `wind`,
`share_intent`, per-aircraft `perf`, and `BroadcastSchedule`. That single asymmetry breaks three of
the four release gates (V1.0.2, V1.0.3, V1.1.1), and it makes §6 of the design doc — "the same
instance runs unchanged under both backends" — false today.

One env, one seam, both backends. `record=True` and `stop_within` come along free.

- **Files:** `opencdarr/estimator.py` (primary), reusing `fleet.build_env` / `fleet.run_fleet`
- **Verify:** existing IPR numbers **bit-identical** — `pytest tests/test_estimator.py
  tests/test_loop.py tests/test_loop_mixed_fleet.py tests/test_fleet.py tests/test_smoke.py`. Add a
  direct assertion that `run_fleet` at n=2 and `run_encounter` agree on
  `(conflict, los, min_sep)` for a fixed seed across a spread of `dpsi`, with and without nav noise.
  The docstrings already claim this reduction; this makes it a test rather than a claim.

## 3. Parametric pairwise sampler — **done 2026-07-29**

- [x] `scenario.sample_pairwise` now takes `dpsi` / `dcpa` / `side` / `gs_intr` as **either** a
      pinned constant **or** a callable `(rng) -> float` (a custom per-encounter distribution);
      absent means the built-in draw, exactly as before. All four are keyword-only with `None`
      defaults, so every existing caller (`estimator`, `scripts/ips_validate.py`,
      `tests/test_loop_cns.py`) is untouched.

**A list is deliberately *not* accepted.** `Sweep(dpsi=[5, 8, 10])` is three independent estimates,
each with its own counts, interval and cache entry — a fan-out over *conditions*, which belongs to
the runner (item 6), not to a function whose contract is one encounter from one seed. At the sampler
boundary a sweep collapses to `Fixed`: the runner loops conditions and pins a value per cell. So
`Fixed`/`Random` are sampler concepts and `Sweep` is a runner concept, and item 6 needs no new
sampler code.

**The subtlety that shaped the implementation: a pinned slot still consumes its draw.** The three
built-in draws happen in a fixed order (`dpsi`, `dcpa`, `side`) *before* any override is applied;
custom distributions draw only afterwards; `gs_intr` has no built-in draw so an absent override
appends nothing. Had a pinned slot *skipped* its draw, every later slot would slide up into it —
measured at seed 3, pinning `dpsi` would have moved `dcpa` from 11.84 m to 4.28 m **and flipped the
passing side**. Two conditions of an angle sweep would then differ in more than the angle, which
defeats the point of sweeping. This is ADR 0006 §6's config-invariant-stream discipline one level
down from the substream fan-out: draw the same things in the same order regardless of which are
used. Cost is a couple of discarded `uniform` calls.

**Also settled:** the `_DPSI_MIN = 5°` exclusion band constrains only the *built-in draw* (it avoids
degenerate closing speeds). A pinned or custom `dpsi` passes through as given, so the published
sweeps starting at **2°** are not silently relabelled as 5° — an unconstructable geometry fails in
`create_conflict` instead.

**Verified.** Five tests added to `tests/test_scenario.py`, which previously did not exercise
`sample_pairwise` at all:

- a **golden anchor** for the all-default draw, referenced against the *old algorithm written out
  longhand* rather than constants read back out of the new code — self-consistency proves nothing
  (`design-philosophy.md` #15);
- pinning `dpsi` ∈ {2, 45, 90, 180} leaves `dcpa` and `tlos` exactly where the draw put them;
- a pinned 2° angle is not clamped to the exclusion band;
- a callable slot really draws (per encounter, from that encounter's generator);
- `side` and `gs_intr` reach the geometry — neither was reachable through this seam before.

Suite 348 passed; `ruff` and `mypy` clean on the touched files; `scripts/ips_validate.py` still PASS.

**Not done here (item 6's business):** no `Config` field for a pinned `dcpa`. `estimate_ipr` still
passes `dcpa_max` only, so reaching the new slots from a config-driven run arrives with the runner.

## 4. The denominator, and what an encounter is — **4a done, 4b/4c deferred**

> **This item was redesigned on 2026-07-30 after a review discussion that superseded the original
> plan.** The original framing — "unconditional vs conditional estimand, keep both, IPR ≠ 1−P(LoS)"
> — was wrong. See *Superseded reasoning* at the end of this item; the corrected conclusion is that
> `estimate_ipr`'s denominator is a **bug**, and IPR = 1 − P(LoS) exactly.

### 4a. Fix the denominator — **done 2026-07-30**

- [x] `IPRResult` now carries `n_encounters` / `n_los` / `n_conflict` and derives `p_los`, `ipr`,
      `ci95` (Wilson) and `detection_rate` as properties — nothing stored twice, so the rates cannot
      drift out of step with the counts. `n_conflict` is retained as a labelled diagnostic, never a
      divisor. `config.py` now validates `dcpa_max <= rpz`, enforcing the sampler's
      conflict-by-construction guarantee instead of assuming it.

Also fixed in passing: `n_los` was counted *inside* the `if outcome.conflict` branch, so an
undetected breach was dropped from the numerator as well as the denominator. Both are now counted
unconditionally and independently.

**Consumers updated.** The experiment card reports `P(LoS)` with its CI, `IPR`, and
`detection_rate`. `scripts/ips_validate.py` loses its workaround: it had documented that the
conditional denominator "coincides with the unconditional P(LoS) only when lookahead >= tlos", routed
the fixed-geometry path around `estimate_ipr` to dodge it, and printed a runtime WARNING steering
users out of sampled mode — all three now obsolete and removed. `scripts/ips_validation_probe.py`
likewise now pools over encounters rather than detected conflicts.

**Verified.** Suite **350 passed** (+2). The item-2 golden anchor (22/200, IPR 0.89) did **not**
move, as predicted — its config has `tlos < lookahead`, where the two denominators coincide.
`scripts/ips_validate.py --pos 40` still PASS with CIs overlapping. `dcpa_max=100, rpz=50` is now
rejected at load. `ruff` and `mypy` clean on the touched files. Two new tests:

- `test_denominator_does_not_move_with_the_resolver` — at `tlos = 1.5 × t_lookahead`, the resolved
  and unresolved runs share an identical denominator while `detection_rate` differs (1.0 vs < 1.0).
  This is the regression for the bug itself.
- `test_wilson_interval_...` — brackets the estimate, tightens with `n`, and at `k = 0` returns a
  *positive* upper bound rather than the normal approximation's false `(0, 0)`.

See [[todo-might-be-a-bug]] entry 4 for the measurements and the general rule.

**The bug.** `estimate_ipr` divides by `outcome.conflict` — "did the detector fire on the **true**
states at any step". But `StateBased.detect` returns `False` whenever predicted `dcpa >= rpz`
(`cd/statebased.py:31`), so once a resolver has built separation the true-state detector returns
`False` *forever after*: the resolver does not merely avoid the conflict, it **erases it from its own
denominator**. Measured at `tlos=180 / lookahead=120`, n=300:

| noise (pos/vel) | resolver | n_conf |
|---|---|---|
| 0 / 0 | none | 300 |
| 10 / 1 | none | 300 |
| 60 / 6 | none | 300 |
| 0 / 0 | MVP | 300 |
| **10 / 1** | **MVP** | **178** |
| **60 / 6** | **MVP** | **274** |

With no resolver it is always 300 — every sampled encounter *is* a conflict. The shortfall appears
only when resolution is active, and is not even monotonic in noise. A resolver is therefore graded
only on the conflicts it failed to pre-empt.

**Why not the other candidate denominators.** Three were considered and all fail the same test:

- *true-state detection during the run* — contaminated by the resolver (above);
- *perceived/noisy detection* — contaminated by the noise model **and** the resolver, and counts
  false positives as conflicts;
- *nominal geometry at t = 0, or a resolver-disabled nominal reference run* — uncontaminated but
  **blind to induced conflicts**. In a cascade (A avoids C, which puts A on a path that breaches B)
  the nominal run never has A manoeuvre, so the induced A–B conflict never comes into existence at
  all. This killed the nominal-reference-run proposal.
- *number of resolution episodes* — the tempting "count what the system actually handled". Measured
  and rejected: episodes per encounter vary **~9× with the sweep axis itself** (24.8 at `dpsi=8`
  vs 2.67 at `dpsi=90`, both with **zero** LoS), so it would systematically deflate the LoS rate
  exactly where CD&R is hardest. It is also gameable from the other side — a design that bounces
  earns a bigger denominator and scores better, which is the behaviour `PastCPA(bouncing_guard=…)`
  exists to suppress.

**The rule that settles all of them:** *a denominator must be fixed by the experiment design, never
discovered from the run.* Anything whose count depends on behaviour or geometry is a numerator or a
distribution, never a divisor.

**So: the encounter is the unit.** One encounter = **one simulation run from one seed** —
`spawn(root_seed_sequence(seed), n)[i]` → one sampled geometry → one `run_fleet` from `t=0` to
`is_terminal` → one outcome. It qualifies because its number is *chosen* (`n_encounters`) not
discovered, draws are independent by seed construction (ADR 0001), and it is the same unit IPS
samples (one particle = one initial condition) — which is what finally makes MC and IPS estimate
*literally* the same quantity. It matches the paper's own arithmetic too: "a single simulator run
contains 100 independent pairwise encounters", 100 × 100 = 10 000 encounters.

Consequently **IPR = 1 − P(LoS)** exactly, with `n_conflict == n_encounters` whenever
`dcpa_max <= rpz` (which `create_conflict` guarantees). Conflict-ness moves out of the metric and
into the sampler as a *guarantee*; `config.py` currently validates `dcpa_max >= 0` but **not**
`dcpa_max <= rpz`, so that one-line validation is the real fix.

**Known limitation to state, not hide (N > 2).** One fleet run is one encounter whatever N is, so an
8-aircraft ring is *one* sample containing many pairs. Using pairs as the denominator is tempting
(N(N−1)/2 is design-fixed) but pairs within a run share aircraft and trajectories, so their outcomes
are **correlated** and a binomial CI over them would be understated. P(LoS) per fleet-run is the
sound probability; per-pair rates are descriptive statistics carrying that caveat. The two coincide
in the pairwise case, which is why the papers never had to distinguish them.

- **Files:** `opencdarr/estimator.py` (`IPRResult`), `opencdarr/config.py`, `opencdarr/experiment.py`
- **Verify:** the item-2 golden anchor (22/200, IPR 0.89) must **not** move — its config has
  `tlos=60 < lookahead=120`, so `n_conflict` already equals `n_encounters` there. Add a test at
  `tlos > t_lookahead` pinning that `P(LoS)` is now independent of the resolver's denominator
  feedback. Re-document `test_every_sampled_encounter_is_a_conflict`, which currently passes by
  accident of the default config rather than by construction.

### 4b. The encounter event record — **deferred** (designed, not built)

- [ ] Record raw directed events during the run; derive every count from them.

Per the review discussion: *record things as they are*, and derive. Two frozen values accumulated in
`FleetState` (the ADR-0010 accumulator pattern, so an IPS clone carries them) and surfaced on the
outcome:

```python
@dataclass(frozen=True)
class Engagement:
    """One directed resolution episode: `actor` manoeuvred on account of `intruder`."""
    actor: str          # who manoeuvred
    intruder: str       # because of whom — ac1→ac2 is NOT the same event as ac2→ac1
    t_start: float
    t_end: float | None # None = still resolving at termination

@dataclass(frozen=True)
class LosEvent:         # unordered: losing separation is a fact about a pair, not directed
    a: str
    b: str
    t_start: float
    t_end: float | None
    min_sep: float
```

Repeats are written out — if `ac1→ac2` happens twice there are two `Engagement`s — so uniqueness is
a derived read, not a recording decision:

| quantity | derivation |
|---|---|
| nb of loss | `len(los_events)` |
| repeated manoeuvring pairs | `len(engagements)` |
| nb of manoeuvring pairs | `len({(e.actor, e.intruder) for e in engagements})` |
| re-entries | the difference of those two |
| **total time resolving** | `sum(e.t_end - e.t_start)` — [[TODO]] #2 / phase-9, free |
| `los` (bool) | `bool(los_events)` |

The source for an engagement is the aircraft's **own** record, `FleetMemory.resopairs`
(`separation.py:95`) — a pair entering `resopairs` starts an episode, leaving it ends one. That is
the honest answer to "we know we are resolving nonetheless": it is what the system did, not what a
detector predicate says about it. It is perceived-driven, which is *correct for a count* and only
poisonous for a denominator.

**Storage.** Measured ~25 episodes/encounter at `dpsi=8`, so full event lists for Experiment 1
(1080 conditions × 10 000 encounters) run to ~10 GB against ~40 MB for the derived scalars. Agreed
resolution: **the outcome always carries the full lists in memory** (so a drill-down, or a metric
nobody has thought of yet, sees everything), while the per-condition cache persists the **derived
scalars** by default with `record_events=True` to keep full lists for a chosen condition or
subsample.

This also subsumes the review's decision 1: these are the per-encounter accumulators that replace
the terminal-vs-trajectory dilemma.

### 4c. Split trajectory recording from full-state recording — **deferred** (designed, not built)

- [ ] Add a `record_tracks` switch that accumulates straight into `viz.Tracks`, leaving
      `record=True` (full `FleetState` frames) exactly as it is.

**Measured** on a 234-frame, 2-aircraft encounter (`dt=0.5`, `dpsi=90`, `pos_ci95=20`): the full
frames log is **279 KiB — 1220 B per frame**, attributed roughly as `states` 30%, `cns_state` 27%
(37% with lossy comms), `cmds` 14%, `mems` 13%, `gms` 7%. So the trajectory is the single largest
field but only ~30%; the other ~70% is the rest of the world. (The per-field split is indicative
rather than exact — shared references, e.g. a held `Message` wrapping a broadcast `AircraftState`,
are charged to each field that reaches them.)

`viz.extract_tracks` reads **only** `f.t`, `f.states[k].lat/lon`, and `m.resolving` from `f.mems` —
so the minimal plottable log is `t` + 2N floats + N bools ≈ **42 B/frame, a ~29× reduction**. Two
savings compose: dropping ~70% of the fields, then dropping Python object overhead by packing into
arrays. Today `extract_tracks` retains every frame and then discards ~97% of it.

Three tiers, three consumers, ~4 orders of magnitude apart:

| switch | contents | per encounter | 10 000 encounters |
|---|---|---|---|
| *(neither)* | scalars + 4b event record | ~40 B | ~400 KB |
| `record_tracks=True` | `t`, lat/lon, resolving → `Tracks` | ~10 KiB | ~100 MB |
| `record=True` *(unchanged)* | full `FleetState` frames | 279 KiB | **2.7 GB** |

**Additive, not a redesign:** `record=True` has 102 call sites across the handbook, every example
notebook and `examples/README.md`, so it is documented public surface and keeps its meaning. The new
switch is independent, and its reduction target is the existing `viz.Tracks` type — no new type, and
`plot_pairwise` gains the ability to take a `Tracks` directly alongside a `Run`.

Once 4b lands, `Engagement(t_start, t_end)` makes the per-frame `resolving` flag derivable, so the
tracks log collapses further to pure geometry (`t` + positions).

- **Files:** `opencdarr/fleet.py` (`run_fleet`), `opencdarr/viz.py`
- **Verify:** `record_tracks=True` and `extract_tracks(record=True run)` produce identical `Tracks`
  for the same seed — the packed path must be a pure reduction, not a second trajectory.

**Open question to settle before building 4b.** An `Engagement` needs a `t_end`, but `resopairs`
membership is driven by *perceived* state on the broadcast cadence, so episode boundaries are
quantised to `broadcast_interval` and can **flicker** — a pair may leave and re-enter `resopairs` on
consecutive ticks under noise. The measured ~25 episodes/encounter at `dpsi=8` is very likely partly
this. Whether to debounce (a minimum episode duration, or a hysteresis window), and with what
parameter, changes **every** derived count — episodes, unique pairs, re-entries and total time
resolving. Decide the debounce policy first; otherwise the counts are a property of `dt` and the
noise level rather than of the resolver. See also the thrashing entry in [[todo-might-be-a-bug]].

<details>
<summary><b>Superseded reasoning</b> (kept for the trail, per <code>design-philosophy.md</code> #19)</summary>

The review's R3 read `estimate_ipr`'s `n_conflict` denominator as a deliberate
conditional-vs-unconditional **design choice**, ranked it as a risk, and asked which estimand to
adopt. A measurement at the papers' spawn rule appeared to confirm a real divergence
(`1 − IPR = 0.0511` vs `P(LoS) = 0.0467`). That divergence was an **artifact of the bug**, not
evidence of two estimands: the resolver was deleting its own successes from the denominator. The
"keep both names, they're different quantities" conclusion was wrong — they are one quantity.

Also superseded: the proposal to define the conflict set from a resolver-disabled **nominal
reference run**. It cannot see induced conflicts (see above). The nominal run survives, but for a
different job — it is the **efficiency baseline** every phase-9 "extra vs nominal" metric needs
(extra flight time, extra distance, path deviation), and it caches well because it is independent of
resolver, recovery and γ: one nominal run per geometry serves every cell of a sweep.
</details>

## 5. Top-level exports — **done 2026-07-30**

- [x] `opencdarr/__init__.py` re-exports 45 names across four groups: the contribution surfaces (the
      ABCs/protocols), the reference implementations to compare against, the runners and estimators,
      and the values you construct. [[TODO]] #3's blocker — "still exports nothing, so every import
      is submodule-qualified" — is closed.

Kept deliberately shallow: submodule imports still reach everything, and this is the short path for
the common case rather than a mirror of the tree. `opencdarr.parallel` is **not** re-exported (a
scheduling concern with its own install extra).

**Both optional extras stay optional.** `matplotlib` (via `viz`) and `joblib` (via `parallel`) are
imported lazily inside the functions that need them, so `import opencdarr` is still numpy + pyyaml.
Verified in a **subprocess**, because this test session imports both long before the assertion runs.

Two things corrected while doing it, both mine:

- I ordered the imports leaf-first and wrote a comment claiming an alphabetical block "would risk a
  partially-initialised import". Tested it: **alphabetical works fine**, so the comment was a false
  claim about the code. Now sorted per ruff `I001`, with the role grouping in the module docstring
  where it belongs.
- A first draft asserted `__all__` was sorted. `RUF` is not in this project's selected ruff rules, so
  `__all__` ordering is not a project convention — asserting it would have invented a rule *and*
  re-implemented a linter in a test. Replaced with a duplicate check, which is a real defect.

- **Files:** `opencdarr/__init__.py`, `tests/test_public_api.py` (new)
- **Verify:** suite 355 passed; `ruff` clean. `tests/test_public_api.py` locks that every `__all__`
  name resolves, that the documented contributor one-liners import, and that a fresh interpreter
  importing the package pulls neither optional extra.

## 6a. The experiment runner — **done 2026-07-30**

- [x] `opencdarr/experiment.py`: `Fixed`/`Sweep` axes → conditions → per-condition estimate →
      `ExperimentResult` with `records()`, `frame()` and `cell()`. `MC(n_encounters=…)` and
      `IPS(shells=…, n_particles=…, reps=…)` swap over the **same** `Methods` bundle.

**Renamed 2026-07-30** to the design doc's own vocabulary. It shipped as `study.run_study` /
`StudyResult` to avoid colliding with the then-existing `experiment.ExperimentResult`; that
collision was resolved by splitting the old module — the name registry moved to `registry.py` and
`run_one_experiment` re-homed here as the all-`Fixed` wrapper, so there is one card writer.

**What it does.** `Fixed` holds a parameter, `Sweep` fans it out; an all-`Fixed` declaration is the
single-cell case rather than a separate path. `Sweep(values, name=…, build=…)` is the piece worth
having got right: `build` maps a level onto the value the run needs, so a *component* can be swept
over a readable scalar axis — `Sweep([1.05, 1.4], build=lambda m: MVP(margin=m), name="margin")`
puts numbers in the table and objects in the run. Bare object levels work too (`Sweep([MVP(1.05),
VO(1.05)])`) for the categorical benchmark case. Unknown keys fail immediately with the declarable
list, so a typo or an aspirational axis cannot be silently ignored — 23 keys, every one wired.

Geometry pins now reach MC: `estimate_ipr` gained `dpsi`/`dcpa`/`side`/`gs_intr`, forwarded to
`sample_pairwise` — the "item 6's business" left open by item 3.

**Columns adapt to the backend** rather than being forced into one schema: MC reports counts, a
Wilson interval and `detection_rate`; IPS reports the replicated probability, its log-space interval,
`reps` and `n_collapsed`. Neither can honestly fill the other's columns.

**Verified — the release gate, split into two isolations.** Each rests on the *other* seam working,
because one combined test would pass with either wired and the other silently dropped:

- *airframe half* (v1.1.1): a working MVP plus a command-discarding airframe. MVP clears this
  geometry always, so near-certain LoS has one explanation — the airframe was flown. Asserted under
  MC **and** IPS; this is the assertion that would have caught the item-2 defect on the MC side while
  IPS stayed green.
- *resolver half* (v1.0.2 → v1.0.3): the default airframe plus a resolver that declines to avoid,
  against MVP on the identical geometry and seed.

Suite **368 passed** (+13); `ruff` and `mypy` clean on the touched files. End to end, the
crossing-angle profile comes out with the expected shape (P(LoS) 0.060 / 0.005 / 0.000 at 5° / 45° /
90°) — shallow crossings hardest, as the papers report.

**Two things the tests taught me, both worth keeping:**

- A resolver that "does nothing" by re-commanding its own **perceived** velocity is *not* equivalent
  to no resolver: the noisy self-fix makes it re-aim slightly wrong each tick, and that random walk
  does real avoidance work. Measured, its LoS rate falls 0.97 → 0.20 → 0.00 as `pos_ci95` goes
  1 → 5 → 10 m, while `resolver=None` stays at 1.00 throughout (because `CruiseAutopilot` holds the
  *true* initial cruise, not the noisy fix). The resolver-isolation test therefore runs near-quiet.
- The IPS floor in that test is deliberately loose (`> 0.5`): at a 20-particle / 2-rep budget the
  estimate is genuinely noisy — per-replication values of 0.76 and 1.00 for one setup. The property
  is "high, not ~0"; tightening it to the number it happens to produce would be tuning the threshold
  to the run.

- **Files:** `opencdarr/experiment.py` (new), `opencdarr/estimator.py` (geometry pins),
  `opencdarr/__init__.py` (exports), `tests/test_experiment.py` (new)

## 6b. Cache, provenance card, `plot()`, `n_jobs` — **done 2026-07-30**

- [x] `run_experiment(..., cache=…, n_jobs=…, card_dir=…)` plus `ExperimentResult.plot()`.

**The cache, and the identity problem it turned on.** `cache.run_key` stringifies non-JSON values,
and a live component is hostile to keying: `GnssNavigation` and `Comm` hold **function objects**
whose `repr` carries a memory address (unstable across processes), and a factory's lambda keys on its
qualname alone — so `Comm(latency=constant_latency(0))` and `constant_latency(5)` would have
**collided**. Different physics, one key.

`identity()` derives it structurally instead, and the two hazards turned out to be solvable rather
than fatal:

- a closure's captured arguments are readable through `__closure__`, so `constant_latency(0)` and
  `(5)` now key **distinctly** — verified;
- `make_mixture_gaussian` memoises its solved sigma in a dict inside the closure, which would make
  the key depend on *how far the run got*. Private (underscore) free variables and attributes are
  therefore treated as derived — their names are keyed, their values are not — so the identity is
  **stable across use** while `k` and `tail_weight` still reach the key. All three verified.

It also hashes the **class's own source** (`inspect.getsource`), which is what catches a contributor
editing their resolver's logic without changing its constructor arguments —
`cache.code_fingerprint()` covers only `opencdarr/**/*.py`, so without this the cache's "can only
save time, never change a result" promise would fail precisely for the v1 audience. Where identity
cannot be established (an opaque attribute, a REPL-defined class with no readable source) it
**raises `CacheIdentityError`** rather than falling back to a weaker key, with `cache_id` as the
documented escape hatch. A wrong key is worse than no cache.

Measured: 6 conditions cold 12.3 s → warm 0.02 s (**~660×**), identical rows, one entry per
condition; changing a resolver's margin writes a second entry rather than serving the first.

**`n_jobs`** spreads conditions over processes via `parallel.resolve_jobs` and `_joblib` (matching
`parallel.py`'s `parallel_cls, delayed` naming). Conditions are independent seeded fan-outs, so this
is pure scheduling — verified identical to serial.

**`plot()`** lays itself out from the axis roles: first `Sweep` on x, the rest one line each, the
interval already in the table as a shaded band, log y by default for `IPS`. No grid, no figure title,
`legend(frameon=False)` — matching `viz.plot_pairwise` and the house convention that the figure
carries axes and legend while the prose carries the rest. Raises when every parameter is `Fixed`
(nothing to plot against) or the metric is absent on that backend.

**The card** generalises `experiment._write_card` from one run to a sweep: the declaration with each
parameter's *role*, the component identities (**the same strings the cache keys on**, so a card and
a cache entry cannot disagree about what was run), backend, seed, `code_fingerprint()`, base config,
and the results table. Identity is best-effort here — an unkeyable component is recorded as such
rather than aborting the write.

**Verified.** Suite **379 passed** (+11); `ruff` and `mypy` clean on the touched files (the 15
remaining tree-wide findings are the pre-existing ones [[TODO]] #5 catalogues).

**Process note.** A line-rewrapping helper I used to fix `E501` findings mangled two comment blocks
into invalid syntax by joining code-adjacent lines. Caught immediately by `ast.parse`, repaired by
hand, and the helper is retired — targeted edits from here.

- [ ] `Fixed` / `Sweep` axes → a conditions list → run each cell → `ExperimentResult` with
      `frame()`, `cell()`, `plot()`.

Deliberately thin: no `Random`, no quadrature, no axis algebra beyond the product of `Sweep`s.
The load-bearing requirement is that `MC(...)` ↔ `IPS(...)` swap over the **same** user objects —
that is the V1.0.2 → V1.0.3 gate, and the whole reason for item 2.

Two details worth getting right now rather than later:

- **Axes carry `(levels, build, name)`**, e.g.
  `Sweep(values=[1e-5, 1e-6, 1e-7], build=lambda p: LatchingComm(p), name="p_comm_fail")`. This is
  how a contributor sweeps a parameter of *their own* component. The bare object-sweep form
  (`communication=Sweep([LatchingComm(1e-5), …])`) works but leaves the frame column full of objects
  with no numeric coordinate to plot against.
- **Metric/backend compatibility fails fast**, and the incompatible set is smaller than the design
  doc claims — see item 8.

Reuse rather than rebuild: `cache.run_key` / `cache.load_or_run` per cell (the cache §8 asks for
already exists, `.opencdarr_cache/` and all), `parallel.resolve_jobs` for `n_jobs`,
`estimator.combine_ipr` for pooling, and `cache.code_fingerprint()` on the card.

- **Files:** new `opencdarr/experiment.py` (or similar), `opencdarr/experiment.py`
- **Verify:** the **release-gate test** — a toy custom `ConflictResolver` *and* a toy custom
  `Dynamics`, run under `MC(...)` then `IPS(...)` with the same objects, asserting both backends
  actually *used* them (a resolver that always brakes must measurably move `min_sep` on both paths).
  This is the test that would have caught the item-2 defect. Plus: reproduce an angle-sweep profile
  and cross-check `scripts/ipr_angle_sweep.py` on a handful of angles.

## 7. A seam for stateful user models — **done 2026-07-30**

- [x] `CommunicationModel.initial_state() -> CommState` (default `CommState()`), called by
      `CnsState.initial(n, communication=None)` — so a stateful model's own `CommState` subclass is
      in place on the **first** tick instead of being handed a plain one to detect and upgrade.

`CommState` is a closed frozen dataclass — exactly `held` and `in_flight`. A user model that must
remember something (the motivating case: a `comm_is_on` latch that fails once and stays failed) has
nowhere to put it. Subclassing does survive the round trip, because `CNS.sense` threads whatever
`step` returns straight back into `CnsState` — but `CnsState.initial(n)` handed the model a plain
`CommState()` on the first tick, so every such model had to detect and upgrade it by hand.

~5 lines, and it is the general seam for **any** stateful contribution, which is squarely the v1
audience. Same argument applies to `SurveillanceModel` if a dead-reckoning model ever lands.

**`initial_state()` takes no arguments**, which was the one open question — a *per-aircraft* state
(the transceiver model of item 10) looks like it needs the roster at `t = 0`. It does not:
`CommState.held` already reads an absent key as "nothing has happened on that link yet", so a
per-aircraft field keys the same way and starts empty. That keeps the hook at the width item 7 was
designed for, and keeps the roster check where it already lives (`validate_ids`, called at the
composition root against the real ids).

**A strict no-op for every shipped model.** `Comm` does not override the hook, so `initial_state()`
returns the same `CommState()` the call site built literally before — which is why the golden
anchors (22/200, IPR 0.89) and the fleet↔pairwise bit-for-bit reductions all hold unchanged.

- **Files:** `opencdarr/cns/base.py`, `opencdarr/cns/stack.py`, `opencdarr/fleet.py` (one call
  site), `opencdarr/loop.py` (the other)
- **Verify:** suite **396 passed** (+3), `tests/test_cns_communication.py` 13 → 16; `ruff` and
  `mypy` clean on the touched files. `scripts/ips_validate.py --pos 40` still **PASS** — MC
  `P(LoS)=0.03025 [0.02538, 0.03603]` against IPS `0.028452 [0.026179, 0.030574]`, `collapsed=0/8`.
  The load-bearing test drives a toy stateful model through a
  whole `run_fleet` and reads its own field **unguarded** — `assert isinstance(state,
  _TickCountState)` inside `step` rather than an isinstance *fallback*, so a model written the
  honest way fails loudly if the seam regresses instead of quietly working around it. Its counter
  is checked at `t = 0` (zero, i.e. present before anything is offered) and at termination.

Two notes for whoever writes such a model: draw the latch from the **existing** `streams.comm`
generator — do not add a fourth substream, since ADR 0006 §6 requires the stream tree stay
config-invariant and `ips.py:_streams` pins exactly three children. And do not expect IPS to reach a
1e-6 latching failure: that is the discrete-jump pathway [[important-ips-gap]] measured collapsing
8/8 replications, because `min_sep` carries no information about whether the latch fired. Condition
on the latch time instead and reweight — see the design review for the decomposition.

- **Files:** `opencdarr/cns/base.py`, `opencdarr/cns/stack.py`
- **Verify:** `pytest tests/test_cns_communication.py tests/test_loop_cns.py` unchanged; a new test
  that a subclassed `CommState` survives a full `run_fleet` without the model upgrading it manually.

## 8. Honest IPS metrics

- [ ] Surface what IPS *can* report, and fail fast only on what it genuinely cannot.

The design doc says IPS yields nothing but `P(LoS)`. That is overstated, and contradicts
`design_brief.md`'s own requirement: "**Rare-event outputs**, not IPR: … the intermediate
level-crossing probabilities, and effective sample size."

Already computed and currently discarded:

- **The level-crossing curve.** `IPSResult.survival` is per shell, so
  `P(min_sep ≤ d_k) = Π_{j≤k} S_j` for every `k` — which *is* [[phase-9-plan]] item 1
  (`P(dcpa < 50)`, `P(dcpa < 25)`, `P(dcpa < 10)`, `P(dcpa < 5)`), free, from a single run.
- **`collapsed_at` / `n_collapsed`** — must be a loud health flag, since ADR 0017 §2 is explicit
  that `prob = 0` is not a real zero.
- **Effective sample size** — required by the brief, computed nowhere.

The correct rule is *IPS gives the rare-event probability and anything conditional on the rare set;
it cannot give unconditional population expectations* — because dropped particles are discarded and
survivors cloned. So median-dCPA-**given-LoS** is available (with the caveat that clones share
ancestry, so ESS ≪ N and there is no cheap CI); median dCPA over all encounters is not. That matches
the instinct already written down in [[phase-9-plan]] item 2 — efficiency metrics want a separate
cheap MC.

- **Files:** `opencdarr/ips.py`, plus the metric layer from item 6
- **Verify:** the level-crossing curve from one `ips_once` matches a plain-MC CCDF of `min_sep` in a
  regime where MC is feasible (the `pos=40` correctness rung).

## 9. Widen the noise-distribution signature — **rejected 2026-07-31**

- [x] ~~`NoiseDistribution.__call__(rng, ci95)` → also receive heading and ground speed; add the two
      missing latency models.~~ **Not done, and deliberately not deferred: rejected.** The signature
      stays `(rng, ci95)`.

**Why: the latency models would double-count.** `LastKnown` is hold-as-is with no dead-reckoning
(ADR 0006 §2), so a receiver acting at `t` on a message measured at `t_meas` is *already* looking at
a position `t - t_meas` seconds stale, and the source has already moved `(t - t_meas)·g` along track
since. That displacement is already emergent from C + S. Folding a `−ℓ·g` bias into the navigation
error would apply it a second time.

The paper's lumped "Latency" model is the right modelling choice for a simulator with **no channel
model** — which is what CDaRR was. OpenCDaRR has `LatencyDistribution` and hold-as-is surveillance,
so it produces the same displacement for free and with the *correct distribution*: whatever
`t - t_meas` actually is under jitter, drops and broadcast cadence, rather than a fixed `ℓ`.

**Experiment 3's two Latency models are still runnable** — as `constant_latency(ℓ)` in the comm
layer rather than as a noise distribution. That is a better-founded comparison, but the numbers will
not match the paper's lumped model exactly, and that difference needs stating in the write-up rather
than glossing.

**The anisotropy question is closed with it, in the code's favour.** Track-oriented anisotropy was
the only other reason to pass `trk`. `noise_distributions.py` already argues the physics: GPS
position-error anisotropy comes from satellite geometry, not the vehicle's heading, so the ellipse
is axis-aligned. The paper disagrees; that is a disagreement about the physics, not an
unimplemented feature, and it is now recorded as such in [[gps-noise]] rather than left open.

**A bias needs no signature change either.** A *static* ENU bias is a five-line `NoiseDistribution`
closure over the existing protocol — it belongs in a user's file, not beside `gaussian`, because it
breaks that module's containment guarantee. A *drifting* bias needs memory, which is what the
`NavEffect` seam is for ([[0021-navigation-extension-by-quality-effects]] §1).

- **Files touched instead:** `vault/derivations/gps-noise.md` (new "Why the noise model does not see
  heading" section), `vault/architecture-dataflow.md` (the map documented `(rng, ci95, trk)`, which
  never existed), `vault/run-experiment-design.md` §"cannot see heading".

## 10. A stateful comm model: independent transmitter and receiver outages — **done 2026-07-30**

- [x] `TransceiverComm` — a `Comm` subclass whose per-aircraft **transmitter** and **receiver**
      fail and recover on their own, threading a `RadioState` (`tx_down`, `rx_down`, `t_prev`).
      The first real user of item 7's seam.

`Comm` loses *messages* — every tick is an independent draw, so it is the channel and has no
memory. `TransceiverComm` loses *radios*. Four rates in 1/s (`tx_fail_rate`, `rx_fail_rate`,
`tx_recover_rate`, `rx_recover_rate`), separate rather than shared because a transmitter and a
receiver have no reason to share a reliability figure — and separate reads better at the call site
too: `TransceiverComm(rx_fail_rate=1e-3)` says exactly what it does.

Settled with the user (2026-07-30), five decisions:

- **Two subsystems, not one "comm down" flag.** A transmitter and a receiver are separate hardware.
  An aircraft whose *receiver* fails flies blind while its transmitter keeps squittering, so the
  rest of the fleet still sees it perfectly — which is the asymmetric case worth studying and is
  invisible to a single flag. State is two per-aircraft sets, `tx_down` / `rx_down`, both keyed by
  id and both starting empty (item 7: absent ⇒ nothing has happened to that radio yet).
- **Two-state, recovery optional.** `recover_rate` defaults to `0`, which *is* the permanent latch
  item 7 was written for; set it and the radio comes back. One class covers both.
- **Rates (per second), not probabilities per broadcast.** A per-broadcast probability makes the
  mean time to failure a function of the cadence, so sweeping 1 Hz → 2 Hz would halve it and a
  cadence sweep would be measuring two things at once. Per step the model converts
  `1 − exp(−rate·Δt)` from the elapsed time it reads off its own state, so it is correct under
  offset phases and jitter too, where the gap between `step` calls is not the interval.
- **Gate at offer time.** A down receiver is simply not offered the broadcast, so nothing is
  enqueued for it and it keeps holding stale data (`LastKnown`) — the interesting behaviour.
  Messages *already* in flight still deliver: at the default `latency=0` the two rules coincide
  exactly, and gating delivery as well would mean reaching into `Comm.step`'s delivery loop.
- **Its own class, not a parameter on `Comm`.** An outage draw added inside `Comm` would have to be
  unconditional to keep the stream config-invariant (item 3's rule), which would move **every**
  published number. A subclass draws only what it needs, so `Comm`'s draw sequence stays
  bit-identical and every existing golden anchor holds.

Do not expect IPS to reach a small failure rate: that is the discrete-jump pathway
[[important-ips-gap]] measured collapsing 8/8 replications, because `min_sep` carries no
information about whether the radio failed. Condition on the failure time and reweight instead.

**The one claim above that turned out to be wrong.** This item was planned with "at `fail_rate=0`
the model must be bit-identical to `Comm`". It cannot be, and *should* not be: the outage draws are
made every step whatever the rates, so a zero-rate `TransceiverComm` sits two draws per aircraft
ahead of `Comm` in the stream. Bit-identity to `Comm` and a rate sweep sharing one reception stream
are mutually exclusive, and the sweep matters more — the alternative puts the `fail_rate=0` cell of
every sweep on a different noise stream from its neighbours, which is item 3's mistake exactly.
What survives is the weaker, true statement: with no radio down the *deliveries* are `Comm`'s
(`test_a_working_radio_delivers_exactly_like_comm`). Both directions are now pinned, and the second
test is what stops the first passing vacuously — if the draws were skipped, both would pass.

**Measured, end to end.** A four-level rate sweep through `run_experiment` at `dpsi=8`,
`pos_ci95=20`, n=300, MVP + Past-CPA:

| `fail_rate` [1/s] | P(LoS) | 95% CI |
|---|---|---|
| 0 | 0.060 | [0.038, 0.093] |
| 0.02 | 0.280 | [0.232, 0.333] |
| 0.05 | 0.347 | [0.295, 0.402] |
| 0.20 | 0.437 | [0.382, 0.493] |

Monotone and saturating, as it should be — past a point the radios are down essentially all the
time (measured: a radio is out on **99.7%** of ticks at 0.2/s) and more rate cannot hurt further.
The axis needs no new runner code: `Sweep(..., build=…, name="fail_rate")` puts the rate in the
table as a plottable numeric column, and `identity()` keys each rate distinctly, so a sweep writes
one cache entry per condition rather than colliding.

**A trap worth recording, because it cost half an hour.** The first sweep returned P(LoS) = 0 at
*every* rate, with identical intervals — which reads exactly like "the model is not wired". It was
wired (a spy counted 18 000 `step` calls and outages on 99.7% of ticks); the declaration was simply
missing `navigation`, and `estimate_ipr` defaults it to `None` = **exact self-fixes**. So
`pos_ci95=20` was declared and did nothing, and with perfect data a frozen stale fix still resolves
cleanly. Three different comm models producing bit-identical results is the signature to watch for,
and the lesson is that `pos_ci95` without a `navigation` model is silently inert — a real sharp
edge in the declaration surface, and a candidate for a fail-fast check of its own.

- **Files:** `opencdarr/cns/communication.py`, `opencdarr/cns/__init__.py`, `opencdarr/__init__.py`,
  `tests/test_cns_transceiver.py` (new)
- **Verify:** suite **410 passed** (+14); `ruff` and `mypy` clean on the new files (the 17
  tree-wide ruff findings are the pre-existing catalogue). The tests split into the **hazard** and
  the **gate**, because a model with one right and the other wrong still produces plausible
  numbers. Gate tests construct the health they want on a `RadioState` with every rate at zero, so
  a deterministic assertion is not fighting a random draw. The load-bearing measurement is mean
  time to failure at two cadences: **10.32 s at 1 Hz vs 10.23 s at 2 Hz** for `rate=0.1` (theory
  `dt/(1-exp(-rate·dt))` = 10.51 / 10.25), where a probability quoted per broadcast would have given
  **5.25 s** at 2 Hz — a 2× separation, so the test genuinely discriminates the two
  parameterisations rather than just passing. `scripts/ips_validate.py --pos 40` still **PASS** and
  **bit-identical** to the pre-item-10 run (MC 0.03025, IPS 0.028452, same per-shell survival) —
  the check that a new subclass left `Comm` and every existing path alone.

## 11. Broadcast cadence: a Hz spelling, and phase/jitter from a declaration — **done 2026-07-30**

- [x] `BroadcastSchedule.at_rate(hz)`, `broadcast_jitter` / `broadcast_random_phase` as config
      fields and declarable axes, and — the thing this item turned out to be really about — the
      transmit schedule now reaches **IPS**, which it never did.

**The defect the item uncovered.** `_run_ips` called `build_env` **without `schedule=`**, so it
silently took the 1 s default while MC honoured `config.simulation.broadcast_interval`. A declared
cadence therefore meant two different things depending on the backend — the same shape as item 2's
defect, and the same consequence: an IPS sweep over the cadence returns a confident null result.
Measured at `dpsi=45`, `pos_ci95=40`, n=300 / 60 particles × 2 reps:

| `broadcast_interval` | MC P(LoS) | IPS before | IPS after |
|---|---|---|---|
| 1.0 | 0.0333 | 0.015842 | 0.015842 |
| 3.0 | 0.0267 | **0.015842** | 0.049964 |
| 6.0 | 0.0767 | **0.015842** | 0.028521 |

IPS was returning *the same number to six digits* for every cadence. Note the `interval=1.0` cell
does not move: 1 s is the one value the accidental default happened to get right, which is exactly
why this survived — every config in `configs/` uses it.

Both backends now build their schedule through one function, `broadcast.schedule_for`, so they
cannot drift apart again by one call site being updated and the other not.

**Two things that differ from the plan above, both deliberate:**

- **`at_rate` is a classmethod, not a `rate=` field.** The plan said to accept both and raise on
  `BroadcastSchedule(interval=1.0, rate=2.0)`. A named constructor makes that contradiction
  *unrepresentable* instead — one stored spelling, so equality, `repr` and the cache identity see
  one number however it was written. Same argument as `MC`/`IPS` carrying only their own
  estimator's parameters.
- **A config file stays seconds-only.** Two YAML keys for one physical quantity is worse in a
  committable file than in Python, where a second constructor is unambiguous.

**Why the new MC substream is free.** Jitter needs a broadcast generator, and `estimate_ipr` span
exactly three children per encounter (geometry, navigation, communication). It now spawns four —
and a `SeedSequence`'s *i*-th child depends only on `i` and its parent, so the first three come out
**bit-identical** to the three-child tree every published number came from. Verified directly, and
then end to end: the MC column above is unchanged to the last count (10 / 8 / 23 LoS) across the
change. The broadcast child was added *last* for exactly this reason.

The random phase draws from the generator the **geometry** was sampled from, after
`sample_pairwise` has finished with it and where nothing else reads. So enabling the phase *appends*
draws rather than shifting existing ones, and a fixed-phase run is unmoved. This is the same
"append, never insert" property the 4-child spawn relies on, one level down.

`BroadcastSchedule` owns the transmit cadence and stays the single owner — putting an interval on
the comm model too would give one physical quantity two spellings and force `run_fleet` to pick a
winner. What is actually missing is smaller than that:

- **no Hz spelling anywhere.** "2 Hz" has to be hand-converted to `interval=0.5`. Mutually
  exclusive with `interval`, so `BroadcastSchedule(interval=1.0, rate=2.0)` must raise rather than
  silently prefer one.
- **`phase` and `jitter` are unreachable from a declaration or a config file.** `experiment.py`
  declares `broadcast_interval` (it is in `_SIMULATION_FIELDS`) and `estimator.py:189` builds
  `BroadcastSchedule(interval=…)` and nothing else — so the unsynchronised-transmitter model
  (`random_broadcast_phase`) and ADS-B slot dithering are Python-only today, reachable through
  `run_fleet` but not through the runner the v1 audience is pointed at.

**`phase` is declared as a boolean, not a list.** It needs one entry per aircraft, so it cannot be
a literal in a declaration that does not know `n`; `broadcast_random_phase: bool` asks for the draw
and `schedule_for` performs it per encounter, from that encounter's own seeded generator. A literal
phase vector remains available through `run_fleet(schedule=…)`, which is where a user who wants
specific offsets already is.

**Found on the way, not fixed here:** a declared parameter bypasses **every** config constraint —
`_config_for` builds its `Config` with `dataclasses.replace` and never calls `_validate`. Harmless
for the transmit fields (a bad jitter is still caught loudly by `BroadcastSchedule`), a wrong number
for others. See [[todo-might-be-a-bug]] entry 6; it is a behaviour change and deserves its own item.

- **Files:** `opencdarr/cns/broadcast.py` (`at_rate`, `schedule_for`), `opencdarr/config.py`
  (two fields + two constraints), `opencdarr/estimator.py` (4th substream, schedule per encounter),
  `opencdarr/experiment.py` (the `schedule=` fix, two declarable keys),
  `tests/test_broadcast_schedule.py` (new)
- **Verify:** suite **422 passed** (+12); `ruff` and `mypy` clean on the touched files. Jitter and
  phase had **no tests at all** before this. The regression for the defect is white-box on purpose
  — it captures the `FleetEnv` the IPS backend actually builds and asserts on its schedule, rather
  than inferring the wiring from a statistical difference between two runs, which would be slower
  and flakier. `scripts/ips_validate.py --pos 40` unchanged (its config transmits at 1 s, the one
  cadence the old default got right).

---

## Related

- [[run-experiment-design]] — the proposal this implements, and whose §1/§4.1/§6/§8 claims the
  review corrected
- [[TODO]] — items 2 (phase-9 metrics) and 3 (minimal user-facing interface), and the V1.0–V1.2 gates
- [[important-ips-gap]] — why a discrete-jump rare event needs a different coordinate, and the
  measured collapse
- [[0017-ips-level-and-splitting]] — the estimand, the fixed-shell obligation, and the replication CI
- [[phase-9-plan]] — the metrics items 8 partly satisfies for free
