# ADR 0016 — Steady-uniform wind: a threaded environment field + ground/air command semantics

- Status: accepted (Phase 5 gate green)
- Date: 2026-07-24
- Deciders: Fazlur Rahman

## Context

Phase 4 built the fixed-wing integrator *around* a wind term but held it at zero (ADR 0013 §5).
Phase 5 turns it on: the aircraft's ground-relative motion becomes the vector sum of its airspeed and
the wind (the paper's Eqs 1–9; `vault/papers/drones-wind.pdf`, re-derived and analytically validated
per ADR 0002, never ported). Two quantities that were one at zero wind split apart — **heading `ψ`
vs ground track `χ`** (their difference is the crab angle), and **airspeed `V_TAS` vs ground speed
`V_GS`** — and a constant-bank fixed-wing turn becomes a **trochoid over the ground**. This ADR
records how wind enters the model (`vault/phase-5-plan.md`, decisions 1–6).

## Decision

### 1. Wind is a threaded, read-only **environment** input — not aircraft state

A steady uniform field is identical for every aircraft and cannot be affected by one, so it is **not**
hidden, future-affecting per-particle state. It is threaded into `Dynamics.step` as a `wind`
argument, the same category as `perf` / `dt` (which `AircraftState` also does not store). Storing a
copy on each state would be a second source of truth (the redundancy ADR 0010 §4 rejects) and would
put a shared value on every IPS clone. The no-hidden-state invariant is about *hidden* state
(globals/singletons/closures); an explicit argument satisfies it. The only per-aircraft consequence,
the crab/heading, lives in the existing clonable `yaw`.

### 2. A `WindField` value + a defaulted `Dynamics.step` signature change

`opencdarr/wind.py`: a frozen `WindField` storing inertial ENU components `(w_east, w_north)`, with a
`from_met(coming_from_deg, speed)` factory applying Eq 1 (the meteorological "coming-from" sign is the
one thing easiest to get wrong, documented and tested at the boundary), plus `NO_WIND`.
`Dynamics.step(state, command, perf, dt, wind=NO_WIND)` — **defaulted**, so every pre-wind call site
and test is byte-identical and Phase-4 behaviour is the literal default (the 5a gate: MVP/VO anchors
bit-for-bit + an explicit `step(..., wind=NO_WIND) == step(...)` identity). The field is
**uniform-constant now**; the *type* is the seam a spatial/temporal `(lat, lon, t) → vector` field
slots behind later (gusts/shear), with no dead machinery today.

### 3. The heading `ψ` field is the **existing `yaw`**, reused — not a new field

ADR 0012/0013 already shipped `yaw` (default `None` ⇒ track-aligned, `ψ == trk` at zero wind), and
the fixed-wing already integrates it as primary. So Phase 5 adds **no** state field — it redeems the
`heading`-with-wind deferral of [[0010-dynamics-subpackage-and-odometry-state]] §4 by giving the
existing `yaw` its wind *meaning*, exactly as `state.py`'s `yaw` docstring anticipated. No airspeed
field either: `V_TAS = |velocity_enu(state) − wind|` is derivable (a stored copy would be the same
redundancy). `yaw` is the one genuinely non-derivable quantity (integrated history), which is why it
is already state. *(This reverses the plan's original "add `AircraftState.heading`" — the Phase-4
reshuffle made the field exist early; see the plan's 2026-07-24 reconciliation note.)*

### 4. Command semantics: ground-frame velocity, air-frame envelope

The `MotionCommand` linear channels are interpreted in the frame each vehicle controls:

- `target_velocity` (multirotor, and every resolver output) is a **ground** velocity. The airframe
  solves for the airspeed vector `v_air = v_ground_cmd − wind`, applies its `v_max`/`ax` limits **in
  the air frame** (that is where the envelope lives), and integrates position by `v_air_clamped +
  wind`. Feasible ⇒ the ground command is met exactly (pure crab); infeasible (`|v_air| > v_max`) ⇒
  it clamps and drifts downwind, reported not hidden. A zero ground command **holds station / hovers
  into wind** when `V_WS ≤ v_max`.
- `target_course` (fixed-wing) is a desired **ground course** `χ`; the airframe crabs
  (`ψ = χ + θ_w`, Eq 3) so the *track*, not the nose, points where guidance wants. Unachievable
  course (`|(V_WS/V_TAS)·sin(θ_wa − χ)| > 1`, downwind-dominated) steers the closest achievable
  heading (crab clamped to ±90°), not silently. `target_airspeed` is an **airspeed** setpoint;
  `target_airspeed_direction` is a heading `ψ` directly (no crab). The crab lives in the fixed-wing
  tracker (`fixedwing._heading_for_course`), keeping the one vehicle-neutral `WaypointAutopilot`
  (ADR 0014 §1). This extends the Phase-4 feasibility taxonomy with the ground-vs-air frame axis.

### 5. CD / CR / CRR are untouched — they read the ground frame, which is the threat

They already separate conflicts on `velocity_enu` (ground velocity), precisely the frame wind changes
and precisely what a resolver should see (the intruder's wind-blown ground track *is* the threat). A
resolver may emit a ground velocity the airframe can't hold (decision 4's clamp/crab); no CDR file
changes. This is why the IPR-under-wind sweep is a one-argument change to `run_encounter`.

### 6. `run_encounter` threads one shared `wind` into both aircraft

The field is the shared environment, same for the pair, defaulted to `NO_WIND`. Reuses the
per-aircraft `(dynamics, perf)` threading of ADR 0011 §7 / 0015.

## Alternatives rejected

- **Store wind on `AircraftState`.** Rejected (§1): a shared read-only environment value is not
  per-particle state; storing it duplicates the threaded value and burdens every clone.
- **Add a new `AircraftState.heading` field.** Rejected (§3): `yaw` already exists and means ψ; a
  second field is a redundant source of truth.
- **Store `V_TAS` alongside `(trk, gs)`.** Rejected (§3): derivable from the ground velocity and the
  wind.
- **Interpret `target_velocity` as an airspeed vector.** Rejected (§4): resolvers and guidance target
  a *ground* outcome (reach a point, open a ground miss distance); the airspeed is what the airframe
  solves for, not what it is told.
- **A spatial/temporal wind field now.** Rejected (§2): no consumer yet; the `WindField` type is the
  seam, matching the paper's own deferral of gusts/shear/turbulence.

## Consequences

- **Good:** both airframes fly in wind — the multirotor crabs and can hover into wind, the fixed-wing
  crabs its courses and turns trochoidally; all analytically validated (Eq 3 crab, Eq 4 ground speed,
  the `wind × period` trochoid drift). `NO_WIND` reproduces Phase 4 byte-for-byte. The research payoff
  lands: **the DAA stack is largely wind-robust** (uniform wind translates both, resolvers read the
  ground frame), with a **small bearing-dependent fixed-wing residual** ([[ipr-under-wind]]).
- **Cost:** `Dynamics.step` / `run_encounter` grew a `wind` argument; `kinematics.py` grew the Eq-3/4
  and vector-sum helpers; the fixed-wing airspeed is now recovered from the ground velocity minus wind
  (a latent `v_cur = state.gs` assumption, correct only at zero wind, was fixed).
- **Obligation / deferred:** gusts/shear/turbulence, spatial/temporal fields, sideslip, 3-D/vertical
  wind — all deferred, matching the paper's §3.1; the energy/power model (body-frame wind components)
  is an optional future hook, not built.

## Relations

- Redeems the `heading`-with-wind deferral of [[0010-dynamics-subpackage-and-odometry-state]] §4 and
  [[0008-velocity-vector-command]] §4 (via the reused `yaw`); turns on the wind-ready integrator of
  [[0013-fixedwing-coordinated-turn]] §5.
- Extends the `MotionCommand`/feasibility model of [[0011-motioncommand-and-guidance-separation]] with
  the ground-vs-air command frame; builds on [[0007-dynamics-as-pluggable-interface]] (a new physical
  effect = a threaded input + math, not a loop fork) and reuses the per-aircraft threading of
  [[0015-velocity-to-fixedwing-projection]].
- Implements [[phase-5-plan]]; results are [[wind-multirotor-vs-fixedwing]] and [[ipr-under-wind]].
  Demos: `scripts/wind_multirotor_demo.py`, `scripts/wind_fixedwing_demo.py`,
  `scripts/wind_conflict_demo.py`, `scripts/ipr_wind_sweep.py`.
