# Phase 2 plan — one clean pairwise encounter (the tracer bullet)

`how-to-step-by-step.md` Step 2 and the spine of **v0.1**: own-state + `step_dynamics` + one
CDR method (state-based detection + MVP resolution) + plain Monte Carlo, run end-to-end from
`config + seed`. Same working style — one file at a time, read each diff, tick the box here.

**Scope of this pass: functional testing only.** We build the pipeline and verify each piece
*behaves correctly*, without yet matching a numeric IPR to the old code. Freezing and matching
an old-code anchor — and the single-`dpsi` vs full-sweep choice behind it — is a **deferred
follow-on** (see "Anchor, deferred" below).

**Exit gate (this pass):** the functional tests pass, and the end-to-end pipeline runs from a
`config + seed` and behaves sensibly — e.g. MVP prevents LoS in a resolvable encounter, an
unresolved encounter *does* lose separation, and the IPR is reproducible given the seed. Each
test must *bite* (Part A #5). The numeric anchor match is a separate, later gate.

---

## Anchor, deferred

`IPR = 1 − n_los / n_conflict` (old code, `test_stochastic_sim_single_job_deterministic.py`).

Matching a frozen old-code IPR is **deferred to a later pass**, and with it the choice of a
single representative `dpsi` (e.g. 90°) vs the full `dpsi` sweep. When we take it up, the
proposed reference is a *deterministic* pairwise MVP IPR (2D, M600, `sep = 50 m`,
speed ≈ 10.29 m/s, cooperative), frozen from a trusted `cdarr`-env run into
`vault/experiments/` — deterministic and KI-1-free (that bug was a *recovery*-state leak),
so a clean reference when we want it. For now, no numeric anchor: functional tests only.

---

## Scope — what Phase 2 is and is NOT

**Is:** one pairwise encounter family, end to end, deterministic then plain-MC, matched to the
anchor. The layers that finally appear: config, scenario, cd, cr, the loop, a plain-MC
estimator, and the experiment/provenance entry point.

**Is NOT** (deferred, with reasons):

- [ ] **Recovery (CRR)** — Step 3. IPR only asks whether resolution prevented LoS; resuming
  nav afterwards is not needed to measure it.
- [ ] **CNS noise / comms** — Step 3. Step 2's Monte Carlo samples *geometry*, not sensor
  noise (`design_brief.md`); the model stays deterministic given an encounter.
- [ ] **VO resolution, vertical resolution** — MVP + horizontal only (2D, per `state.py`).
- [ ] **Multi-aircraft + the coordination-model ADR** — v0.3. Pairwise uses the natural
  cooperative-symmetric rule (both maneuver); recorded as a light ADR, not the full v0.3 one.
- [ ] **IPS / `advance`/`level`/`is_terminal` refactor** — Steps 5–6. Step 2 is plain MC.

---

## Checklist

Each item: **path · purpose · design justification · check · relations.**

### 2a — one deterministic encounter

- [ ] **`opencdarr/config.py`** (new — deferred from Phase 0, now has a caller)
  - *Purpose:* load a run spec from YAML into a typed, frozen `Config` dataclass (scenario +
    aircraft + conflict params + seed).
  - *Design:* YAML → dataclass with full type hints; validated on load (fail fast). One place
    that turns a file into typed config; no scattered `dict["key"]` access. Mirrors the old
    `sim_config.json` fields but typed.
  - *Check:* a sample YAML round-trips to the expected `Config`; bad/missing fields raise.
  - *Relations:* the `config + seed → result` contract (`design-philosophy.md` #4); consumed
    by `experiment.py`. YAML was fixed in Phase 0 (ADR-less decision, recorded in phase-0-plan).

- [x] **`opencdarr/scenario.py`** (new — pairwise encounter generator)
  - *Purpose:* build a pairwise encounter — ownship + intruder `AircraftState`s placed to be
    in conflict at a given crossing angle `dpsi`, miss distance `dcpa`, and time-to-LoS.
  - *Design:* re-derive the conflict-construction geometry (the old `creconfs`) as a pure
    function `pairwise(rng/params) -> (own, intr)`, using `create_aircraft` (validated). A
    seeded RNG samples the encounter distribution for MC (2b).
  - *Check:* generated geometry actually produces a detected conflict at ≈ the intended
    `dcpa`/`tLOS`; derivation note matches.
  - *Relations:* re-derives `creconfs` (read, not ported); feeds cd/cr/loop; the `scenario`
    layer (`design_brief.md`).

- [x] **`opencdarr/cd/`** (package — `ConflictDetector` interface in `base.py`, `StateBased`
  in `statebased.py`; `is_los` alongside) — state-based detection, 2D, directed. The interface
  is the contribution surface: a new detector (e.g. `probconfdetect.py`) adds a file.
  - *Purpose:* pure `detect(own, intr, rpz, t_lookahead) -> bool` — the **prediction only**:
    does `own` foresee losing separation with `intr` within the lookahead? Loss of separation
    is measured **separately** by `is_los(own, intr, rpz) -> bool` (a current-state geometric
    fact, algorithm-independent). Both are **directed** (per observer → target); for n=2 the
    loop calls A→B and B→A.
  - *Design:* re-derive the horizontal state-based CPA from `sim_models/cd_statebased.py`,
    **dropping the vertical branch** (2D). `detect` computes `dcpa`/`tcpa` internally to reach
    its verdict but **returns only the boolean** — `dcpa`/`tcpa`/`t_in` are CPA-specific
    diagnostics, not a general detection output (a probabilistic or reachability detector
    produces none of them). Pure, scalar; the batched n×n form is deferred to the perf step.
  - *Extensibility:* if a later detector needs to expose algorithm-specific metrics, it returns
    a richer record that still satisfies the "predicts conflict" contract — introduced only when
    that second detector arrives, not now (same rule as pluggable dynamics).
  - *Check:* hand-constructed crossing / head-on cases give the correct verdict; `is_los` is
    True exactly when `dist < rpz`. Each check bites.
  - *Relations:* derivation `vault/derivations/cpa-detection.md`; the loop counts conflicts
    (`detect`) and LoS (`is_los`) separately; MVP does **not** consume `detect`'s internals —
    it computes its own geometry.

- [x] **`opencdarr/cr/`** (package — `ConflictResolver` interface in `base.py`; `MVP` in
  `mvp.py`) — a new resolver (e.g. `vo.py`) adds a file.
  - *Purpose:* `MVP.resolve(own, intr, rpz) -> Command` — the maneuver `own` should take
    against `intr`, as a `Command` (target track/heading + speed). **Reuses
    `dynamics.Command`**, so the output flows straight into `step_dynamics`.
  - *Design:* re-derive the horizontal MVP from `sim_models/cr_mvp.py` (the `drel`/`vrel`/
    `dcpa`/`dabsH`/erratum core), **dropping vertical `dv3`**. MVP **computes its own CPA
    geometry** from `own`+`intr` — it does not consume `detect`'s output beyond the trigger, so
    detection stays general and resolution stays self-contained (a cheap recompute; sharing it
    is a perf-step concern). Governing equation in the docstring.
  - *Directed / coordination:* like `detect`, `resolve` is **per-ownship** — each aircraft
    resolves from its own perception of the intruder. Cooperative-symmetric pairwise = call
    `resolve` for both directed pairs (A→B, B→A); both maneuver, and under surveillance
    asymmetry each uses its own view. Recorded in ADR 0004.
  - *Check:* for a known conflict, applying the returned `Command` opens `dcpa` to ≥ `rpz`; a
    resolved encounter yields no LoS.
  - *Relations:* reuses `opencdarr.dynamics.Command`; derivation
    `vault/derivations/mvp-resolution.md`; ADR 0004 (pairwise cooperative-symmetric, directed);
    consumed by the loop → `step_dynamics`.

- [ ] **`opencdarr/loop.py`** (new — the encounter runner)
  - *Purpose:* advance one pairwise encounter to termination. Each step, for **both directed
    pairs** (A→B, B→A): `detect` (predict) → if conflict, `resolve` to a `Command`, else the
    nominal command → `step_dynamics` each aircraft. Separately measure `is_los` each step.
    Record whether any conflict was predicted over the encounter and whether LoS ever occurred.
  - *Design:* pure given `(own, intr, params, dt)`; terminates on the done-timeout (as old
    `done_with_timeout`) or on LoS. Returns an `EncounterOutcome` (conflict: bool, los: bool,
    min-sep). No globals; both aircraft evolve as explicit state (the pairwise precursor to
    `advance`). In Step 2 each aircraft's perceived intruder is the *true* other aircraft; the
    directed structure is where CNS perception plugs in at Step 3.
  - *Check:* 2a gate — a single deterministic encounter: conflict detected, resolved, **no
    LoS**, trajectories sane (min separation ≥ `rpz`), runs to termination.
  - *Relations:* the `environment` layer; becomes `advance`/`is_terminal` at Step 5.

### 2b — plain-MC IPR + provenance

- [ ] **`opencdarr/estimator.py`** (new — plain Monte Carlo)
  - *Purpose:* run N sampled encounters and aggregate `IPR = 1 − n_los/n_conflict`.
  - *Design:* pure MC over the scenario distribution; per-encounter RNG spawned from the root
    (ADR 0001) so it is reproducible and parallel-ready (joblib later). Reports IPR with a
    count-based confidence interval, not a bare fraction.
  - *Check:* IPR is deterministic given the seed; independent of run order / worker count.
  - *Relations:* the `estimator` layer (MC now, IPS at Step 6); ADR 0001.

- [ ] **`opencdarr/experiment.py`** (new — the entry point)
  - *Purpose:* `run_one_experiment(config, seed) -> Result` — the single, readable top-level a
    newcomer can read straight through; writes a provenance card.
  - *Design:* wires config → scenario → loop → estimator; records `config + seed + code-hash →
    IPR` as a provenance card in `vault/experiments/`. Effects (file write) at this edge only.
  - *Check:* end-to-end run reproduces its own IPR from the card; card is complete.
  - *Relations:* `experiment / provenance` layer; the Step 2 gate runs through here.

- [ ] **ADR + derivations**
  - [ ] `vault/decisions/0004-pairwise-cooperative-resolution.md` — both aircraft maneuver
    symmetrically (the pairwise coordination rule); the general >2 model is deferred to v0.3.
  - [ ] `vault/derivations/cpa-detection.md`, `vault/derivations/mvp-resolution.md`,
    `vault/derivations/conflict-geometry.md` — the three equation sets, linked to module+test.

- [ ] **`tests/test_*` — functional (this pass's gate)**
  - [x] `test_cd.py` — `detect` gives the right verdict on hand-built crossing/head-on cases;
    `is_los` True exactly when `dist < rpz`; each bites.
  - [x] `test_cr.py` — applying the returned `Command` opens the miss distance to ~`rpz`
    (within 0.5 m) across crossing angles; `margin` opens to ~`margin·rpz`.
  - [x] `test_scenario.py` — a generated encounter reproduces the requested `dcpa`/`tlos` to
    1e-6 and is detected as a conflict.
  - [ ] `test_loop.py` — a resolvable encounter runs to termination with **no LoS**; an
    encounter with resolution disabled **does** lose separation (the contrast proves the loop
    and CR are doing real work).
  - [ ] `test_experiment.py` — end-to-end from `config + seed`: the IPR is reproducible given
    the seed and independent of run order. (No numeric anchor value yet.)
  - [ ] *Deferred:* `test_ipr_anchor.py` — match the frozen old-code IPR — and any guarded
    BlueSky cross-check — belong to the later anchoring pass.

---

## Decisions (confirmed)

1. **CDR method:** state-based detection + **MVP**, horizontal only.
2. **Pairwise coordination:** cooperative-symmetric (both maneuver, each from its own view).
3. **Anchor:** *deferred* — functional testing only this pass; single-`dpsi` vs sweep decided
   when we take up anchoring.

## Relations to the companion docs

- `design_brief.md` — the first milestone in full ("one pairwise encounter → own state +
  `step_dynamics` + one CDR method + plain-MC IPR → match a known result"); the reusable
  `cd`/`cr` functions "slot in" — but **re-derived, not ported** (`lesson-learnt.md`).
- `design-philosophy.md` — pure `state → value` cd/cr (#1), one owner of state (#2), name it
  like the paper (#6), equation in docstring (#7), reproducibility tested (#4), tracer bullet
  before the estimator framework (#10).
- `how-to-step-by-step.md` — this is Step 2; do not build the estimator framework before one
  clean encounter runs.

## References (read, not ported)

- Detection: `CDaRR_git/sim_models/cd_statebased.py` (horizontal CPA).
- Resolution: `CDaRR_git/sim_models/cr_mvp.py` (`MVP`, horizontal core).
- Encounter geometry: `creconfs` usage in `CDaRR_git/envs/pairwise_conflict.py`.
- IPR + deterministic scenario: `CDaRR_git/test/test_stochastic_sim_single_job_deterministic.py`.
- Params: `CDaRR_git/sim_configs/sim_config.json`.
