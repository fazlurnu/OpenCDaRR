# Phase 1 plan — extract & validate `step_dynamics` (the make-or-break)

The one real risk in the whole rebuild (`how-to-step-by-step.md` Step 1; `design_brief.md`
Decision #4): a **pure** `step_dynamics(state, command, perf, dt) -> state` that reproduces
BlueSky's M600 turn/accel/speed-limited motion, with no `bs.traf` and no `bs.sim.step()`
driving it. Same working style as Phase 0 — one file at a time, you read each diff, we tick
its box here.

**Exit gate** (`how-to-step-by-step.md` Step 1): the three analytical checks below pass, and
you have *seen them run*. Per `design_brief.md`: **if this can't be made to work in a few
days, STOP and reconsider scope** — do not push through. Green on the checks is necessary,
not sufficient (Part A #5): we also ask "what would make these pass while the code is still
wrong?" and make each test bite.

---

## Validation strategy — analytical, not trajectory-matching (the Step 1 decision)

`how-to-step-by-step.md` Step 1 leaves *"what 'close enough' to BlueSky means"* as your call.
Decision: **validate the integrator against first-principles physics, and anchor the
constants to BlueSky.** We do **not** bit-match a recorded BlueSky trajectory.

Why this is sound, not a shortcut:
- `design_brief.md` non-goal: *"Not bit-compatibility with the old pipeline — this is a
  redesign, so it produces new, independently-validated numbers."* Analytical checks are the
  *analytical* rung of the validation ladder (`design_brief.md` reproducibility & validation).
- We still take the **numbers** from BlueSky (max turn rate, turn accel, v_max), so the model
  is the M600, not an invention. We own the *integration*; BlueSky owns the *constants* and
  the *geo math*.
- Behavioural checks are more legible than a golden blob: a reviewer can read
  "10 m/s for 10 s → 100 m" and know what it means.

Recorded as `vault/decisions/0002-analytical-validation-of-dynamics.md`.

## M600 constants (pinned, with source)

Extracted from the BlueSky fork at `~/Projects/bluesky`. These are *read* to source the
numbers; the limiter logic is *re-derived*, not imported (`lesson-learnt.md`: don't port).

| constant | value | meaning | source |
|----------|-------|---------|--------|
| `max_tr`   | 15.0 deg/s  | max turn rate                | `bluesky/traffic/traffic.py:288` |
| `max_dtr2` | 10.0 deg/s² | max turn-rate acceleration   | `bluesky/traffic/traffic.py:289` |
| `v_max`    | 18 m/s      | max ground speed             | `resources/performance/OpenAP/rotor/aircraft.json` (M600 envelop) |
| `v_min`    | −18 m/s     | envelope min (see note)      | same |

The turn-rate-limited heading integration we re-derive is `traffic.py:518-542`.

---

## Scope — what Phase 1 is and is NOT

**Is:** the pure horizontal M600 integrator + the state it needs + its derivation + the three
validation checks. Nothing above it.

**Is NOT** (deferred, with reasons):

- [ ] **Speed-acceleration limiting** — BlueSky's perf ramps speed toward a target via an
  acceleration limit; Phase 1 clamps commanded speed to the envelope and applies it directly
  (no `ax` ramp). None of the three checks exercise a speed ramp, and modelling it now is
  fidelity we haven't shown we need (`design-philosophy.md` #12). Flagged here so it is a
  *known* simplification, not a hidden one (#13). Add it when an experiment needs it.
- [ ] **Vertical motion** — 2D horizontal only (decided; see `state.py`). `vs_max = 5` m/s
  from the envelope is recorded but unused.
- [ ] **Wind** — none, so track over ground equals heading; the state carries one `trk`.
- [ ] **CD / CR / CRR, the loop, scenarios, estimators** — later steps. Phase 1 is one
  aircraft moving correctly, nothing more (tracer bullet, `design-philosophy.md` #10).

---

## Checklist

Each item: **path · purpose · design justification · what goes in · check · relations.**

### Core

- [ ] **`opencdarr/state.py`** (edit — grow the state)
  - *Purpose:* add `turn_rate: float = 0.0` [deg/s] to `AircraftState`.
  - *Design:* the `max_dtr2` limit makes the instantaneous turn rate *future-affecting* state
    — the next step's turn rate is bounded relative to this one. By our invariant it must live
    *inside* the clonable state, or an IPS clone would diverge. Default `0.0` (a spawned
    aircraft is not turning) keeps existing constructors and the Phase 0 smoke test valid.
  - *Check:* smoke test still green; a turning aircraft carries a non-zero `turn_rate` between
    steps; mypy strict clean.
  - *Relations:* first growth of the particle (`design_brief.md` state); consumed by
    `step_dynamics`; embodies the no-hidden-state invariant the file already documents.

- [ ] **`opencdarr/performance.py`** (new)
  - *Purpose:* the aircraft-type flight-envelope limits as plain data.
  - *Design:* a frozen `Performance` dataclass (`max_tr`, `max_dtr2`, `v_max`, `v_min`) and an
    `M600 = Performance(...)` instance with the pinned constants above. Separated from
    `dynamics.py` so the *integrator* (how it moves) is not tangled with the *limits* (how
    fast/tight this airframe can move) — and so a new airframe is a new instance, not a code
    edit (`design_brief.md` open-source: the interface is the contribution surface).
  - *Check:* values equal the BlueSky source; passed explicitly into `step_dynamics`.
  - *Relations:* the numbers trace to `vault/derivations/step-dynamics-m600.md`; future
    aircraft types slot in here.

- [ ] **`opencdarr/dynamics.py`** (new — the make-or-break)
  - *Purpose:* the pure integrator. A `Command` (`hdg` [deg], `spd` [m/s]) type and
    `step_dynamics(state, command, perf, dt) -> AircraftState`.
  - *Design:* one step, in order, all pure — (1) clamp commanded speed to the envelope → new
    `gs`; (2) signed `hdg_err`; (3) the turn-rate limiter re-derived from `traffic.py:518-542`
    using `state.turn_rate` as the previous rate, bounded by `max_tr` and `max_dtr2*dt` → new
    `turn_rate`, new `trk`; (4) propagate position with `bluesky.tools.geo` (bearings/
    distances — the *only* BlueSky call, and it lives behind this boundary so the engine stays
    swappable, `design-philosophy.md` #5). Returns a new state via `dataclasses.replace`.
    Governing equations in the docstring (#7). No globals, no `bs.traf` (#1, #2).
  - *Check:* the three tests below.
  - *Relations:* implements `vault/derivations/step-dynamics-m600.md`; the first of the three
    pure functions the whole interface is built on (`advance`, `design_brief.md`); the layer
    the brief flags as the one real engineering risk (Decision #4). Adds `bluesky` as a
    dependency **only** here, behind the boundary (update `pyproject.toml`).

### Vault

- [ ] **`vault/decisions/0002-analytical-validation-of-dynamics.md`** (ADR)
  - *Purpose:* record *why* we validate analytically with BlueSky-sourced constants rather
    than bit-matching a trajectory, and name the three checks as the acceptance criteria.
  - *Relations:* the Step 1 "what is close enough" decision; cites the non-goal in
    `design_brief.md`; governs `test_dynamics.py`.
- [ ] **`vault/derivations/step-dynamics-m600.md`** (derivation)
  - *Purpose:* the turn-rate-limited kinematics written out — `hdg_err`, the `max_tr` /
    `max_dtr2` limiter, the speed clamp, the geo propagation — linked to the BlueSky source
    lines, to `dynamics.py`, and to `test_dynamics.py`.
  - *Relations:* the `derivations/` pattern (`design_brief.md` #4); duplicating the equation
    here so a reviewer can check it is exactly the DRY-vs-legibility call in
    `design-philosophy.md` #11.

### Test — the three checks (`tests/test_dynamics.py`)

- [ ] **Check 1 — straight-line distance.** Command heading = current track, `spd = 10` m/s;
  advance 10 s. *Gate:* displacement ≈ 100 m (tight tolerance), `trk` unchanged,
  `turn_rate` stays 0. *Guards:* the geo propagation and the "no spurious turning" path.
- [ ] **Check 2 — turn respects the limits, speed held.** At 10 m/s, command a 90° heading
  change. *Gate:* at every step `|turn_rate| ≤ max_tr` (15) and the per-step change
  `|Δturn_rate| ≤ max_dtr2·dt` (10·dt); the heading reaches 90° and holds; `gs` stays 10 m/s
  throughout (turning does not bleed speed, per the command). *Guards:* the `max_tr` cap, the
  `max_dtr2` ramp, and speed/heading independence.
- [ ] **Check 3 — speed cap.** Command `spd = 30` m/s. *Gate:* `gs` clamps to `v_max` = 18
  m/s, not 30. *Guards:* the envelope clamp.
- [ ] **Bite check.** For each, confirm what would make it pass while wrong (e.g. a missing
  `max_dtr2` step would let `turn_rate` jump — Check 2 must catch it).

---

## Relations to the companion docs

- `docs/design_brief.md` — this *is* Decision #4 and the first milestone's first half; the
  spine ("you own the state and the loop; BlueSky provides stateless math") is what makes a
  *pure* `step_dynamics` possible at all.
- `docs/design-philosophy.md` — purity (#1), no hidden state (#2), boundary the third party
  (#5), equation in the docstring (#7), don't gold-plate (#12), logged deferrals (#13).
- `docs/how-to-step-by-step.md` — Step 1; the exit gate and the STOP-if-stuck rule are its.
- `docs/lesson-learnt.md` — "don't port, rebuild": we re-derive the limiter from the BlueSky
  source rather than importing it; only the standard geo math is called through the boundary.

## References (read, not ported)

- Turn-rate-limited heading integration: `~/Projects/bluesky/bluesky/traffic/traffic.py`
  lines 288-289 (constants) and 518-542 (integration).
- M600 envelope: `~/Projects/bluesky/bluesky/resources/performance/OpenAP/rotor/aircraft.json`.
- Old driving loop being replaced: `~/Projects/CDaRR_git/envs/pairwise_conflict.py` (`step`
  = `bs.sim.step()` on `bs.traf`).
