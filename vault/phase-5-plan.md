# Phase 5 plan — Wind: steady-uniform wind on multirotor and fixed-wing dynamics

Turns on the wind term that [[phase-4-plan|Phase 4]] built the fixed-wing integrator *around but held
at zero*. Phase 5 makes the simulation fly in moving air: the aircraft's ground-relative motion
becomes the vector sum of its airspeed and the wind, so ground speed, ground track, turn geometry,
and — for the fixed-wing — the whole shape of a turn all change with the wind, exactly as the source
paper describes.

**Source (kinematics only, not the planner).** Reyner & Liem, *Energy-Efficient Trochoidal Path
Planning for Unmanned Aircraft Under Wind and Performance Constraints* (Drones 2026, 10, 426) —
`papers/drones-wind.pdf`. We re-derive and analytically validate its **point-mass kinematic model**
(its Eqs 1–9), *never* port it ([[lesson-learnt]]). We deliberately do **not** take its
path-planning contribution — the BSB/BBB/SBB trochoidal maneuver synthesis, the Integrated Energy
Metric (IEM), the two-stage Bayesian path-angle optimization. That is a *coverage-path planner* for
aerial mapping; our simulator needs the wind-affected **dynamics**, and lets the CDR/CR/CRR stack it
already has decide conflicts on the resulting ground-frame tracks.

Same working style as Phases 2–4: one file at a time, read each diff, tick the box here; each rung
green before the next; every new physical effect adds a file behind an existing interface, not a
fork of the loop.

---

## The physics we are adding (the paper's Eqs 1–9)

Convention (matches our aviation frame: `x`=East, `y`=North, angles clockwise from north, so a
heading/track `ψ` maps to `atan2(v_east, v_north)` — the same `trk` convention `kinematics.py`
already uses). Wind is **steady, uniform, horizontal**, given by a *meteorological* bearing
`θ_wa` ("coming **from**") and magnitude `V_WS`:

```
  wind vector (inertial):     w_x = −V_WS·sin θ_wa ,   w_y = −V_WS·cos θ_wa            (Eq 1)

  ground velocity = airspeed vector + wind:
      ẋ = V_TAS·sin ψ + w_x ,   ẏ = V_TAS·cos ψ + w_y                                 (Eq 9)

  ground speed (magnitude of that sum, closed form):
      V_GS = √(V_TAS² + V_WS² − 2·V_TAS·V_WS·cos(ψ − θ_wa))                            (Eq 4)

  ground-track course:        χ = atan2(ẋ, ẏ)                                          (Eq)

  crab / wind-correction angle (heading − course), and the crab to *hold* a course χ:
      θ_w = ψ − χ  ;   θ_w = arcsin( (V_WS / V_TAS)·sin(θ_wa − χ) )                    (Eqs 2–3)

  fixed-wing coordinated turn (unchanged from Phase 4 4d, now embedded in wind):
      ψ̇ = g·tan(ϕ) / V_TAS                                                            (Eq 8)
```

Two consequences that drive the whole plan:

- **Heading `ψ` and ground track `χ` split apart.** At `V_WS = 0` they are equal (Phase 4's
  behaviour-preserving special case); under wind they differ by the crab `θ_w`. This is the
  `heading ≠ track` field [[0010-dynamics-subpackage-and-odometry-state]] §4 deferred *"with the
  wind / independent-yaw model that gives it meaning and its own ADR."* Phase 5 is that model — so
  `AircraftState.heading` is **added here**, not in Phase 4.
- **Airspeed and ground speed split apart.** `V_TAS` (what the airframe flies, what stall/`v_max`
  and the drag polar see) is now distinct from `V_GS` (what it makes good over the ground, what
  CD/CR/CRR and odometry see). A constant-`V_TAS` fixed-wing turn traces a *circle in the air frame*
  but a **trochoid over the ground** (a full circle does not return to its start — it is displaced by
  `wind × turn-period`), which is the paper's central figure (its Fig. 4).

Fixed-wing vs multirotor differ in **how wind enters**, not in the vector sum itself:

| | Multirotor (`HolonomicDynamics`) | Fixed-wing (`FixedWingDynamics`) |
|---|---|---|
| Controlled/limited quantity | **airspeed** vector, ≤ `v_max` in any direction | **airspeed magnitude** `V_TAS` (stall ≤ `V_TAS` ≤ `v_max`) + bank-limited turn |
| Response to wind | crab freely; can null drift instantly (up to `v_max`); can **hold station / hover into wind** if `V_WS ≤ v_max` | crab only by yawing the whole airframe; turns become trochoidal; **cannot stop** — is pushed downwind through every maneuver |
| Heading `ψ` | decoupled (its own yaw, from 4e) — the crab is just a velocity offset | = travel-through-air direction; `ψ = χ + θ_w`, integrated via bank |
| Failure mode under wind | required airspeed `|v_ground_cmd − wind|` exceeds `v_max` → clamp, accept ground-track drift | course unachievable when `|(V_WS/V_TAS)·sin(θ_wa−χ)| > 1` (downwind-dominated); or turn radius/`V_GS` demand exceeds envelope |

Multirotor energy note (the paper's Pang et al. [15] reference): power depends on the **body-frame
axial/lateral wind components**, not ground speed. We flag an optional energy hook (§ out-of-scope)
but do not need it for conflict resolution.

---

## Scope, and the exit gate

**IN (steady-uniform wind, dynamics only):** the wind vector-sum kinematics for *both* existing
vehicle classes; the crab / wind-correction angle; trochoidal fixed-wing ground tracks by numerical
integration; the `heading` state field; the airspeed/ground-speed distinction; wind-aware course
holding in the autopilots; and the research payoff — **CDR/CR performance (IPR) under wind**.

**OUT — deferred exactly as the paper defers them** (its §3.1, "Gusts, wind shear, turbulence,
sideslip, and unsteady aerodynamic effects are not modeled … planned for future work"): gusts, shear,
turbulence, time-/space-varying wind fields, sideslip (`β ≈ 0` coordinated-flight assumption holds),
unsteady aero, and any vertical/3-D wind (2-D level, still deferred — [[phase-4-plan]] decision 1).

**OUT — not our problem (it's the paper's *planner*, not its dynamics):** the BSB/BBB/SBB trochoidal
maneuver synthesis and feasibility tests, the IEM energy metric, the Bayesian path-angle
optimization. Our autopilots already navigate (goto/waypoint/loiter, Phase 4 4b–4c); wind just
changes the feasible set they steer within.

**Exit gate:**
1. **`V_WS = 0` reproduces Phase 4 bit-for-bit** — every dynamics model with a zero wind field is
   byte-identical to its Phase-4 self (`ψ == trk`, `V_GS == V_TAS`/`gs`), and the MVP/VO IPR sweep
   reproduces the same anchors. Wind is inert until a non-zero field is supplied (same free-
   regression discipline as Phase 3's zero-noise and Phase 4a's layer-split gates).
2. **Analytical validation against the paper** (the [[0002-analytical-validation-of-dynamics]]
   discipline), not eyeballing: integrated `V_GS` matches the closed form Eq 4; a held course yields
   the crab of Eq 3; a full constant-bank turn traces a circle in the air frame and the paper's
   trochoid over the ground with the predicted endpoint displacement.
3. **The no-hidden-state invariant holds.** Wind is a **read-only environment input threaded
   explicitly into `step`** — never a module global, never mutable manager state — and the only
   *per-aircraft* wind consequence (`heading`/crab) lives in clonable `AircraftState`. An IPS clone
   in wind carries its own heading and inherits the same shared field.

---

## Settled up front (the pushbacks — decided before building)

1. **Wind is environment, not aircraft state.** A steady uniform field is identical for every
   aircraft and read-only, so it is **not** future-affecting *hidden* per-particle state — it is an
   explicit input. It is threaded into `Dynamics.step` as a `wind` argument (see decision 2), the
   same way `perf` and `dt` are. What *is* per-aircraft — the induced heading/crab — lives in
   `AircraftState.heading` (decision 3). This keeps the [[0010-dynamics-subpackage-and-odometry-state]]
   invariant intact: nothing that differs between clones sits outside the clonable state.

2. **A `WindField` value + a `Dynamics.step` signature change (new ADR).** Add
   `opencdarr/wind.py` with a small frozen `WindField` (steady-uniform: constructed from
   `(θ_wa, V_WS)`, exposes `components() -> (w_x, w_y)`), plus a module constant `NO_WIND`
   (`V_WS = 0`). `Dynamics.step(state, command, perf, dt)` gains a trailing
   `wind: WindField = NO_WIND` — **defaulted**, so every existing call site and test is unchanged and
   the Phase-4 behaviour is the literal default. Keep the value *uniform-constant now* (no
   `(lat, lon, t) → vector` field yet — that is the deferred gust/shear generalization, added only
   when a scenario needs it; the [[state|no-speculative-structure]] rule). The `WindField` *type* is
   the seam a spatial field slots behind later, without today carrying dead machinery.

3. **`AircraftState.heading` (ψ) — the deferred field, added here.** New field, **default `= trk`**
   (via `__post_init__`/factory so every existing construction that never sets it is unaffected and
   `ψ == trk` at `V_WS = 0`) — clonable, for the identical reason `turn_rate` must be
   (`state.py`). Fixed-wing integrates it as primary and derives `(trk, gs)` as *ground outputs*;
   holonomic sets it from its yaw (4e) or air-velocity direction. **No new airspeed field** —
   `V_TAS = |velocity_enu(state) − wind|` is derivable from the stored ground `(trk, gs)` and the
   wind, so storing it too would be the redundant second-source-of-truth
   [[0010-dynamics-subpackage-and-odometry-state]] §4 rejected. Only `heading` is genuinely
   non-derivable (a fixed-wing's `ψ` is integrated history, not a function of the current ground
   vector alone).

4. **Command semantics under wind — the load-bearing decision (new ADR).** A `MotionCommand`'s
   linear channels are interpreted in the frame each vehicle naturally controls:
   - `target_velocity` (holonomic, and every resolver output) is a **ground** velocity — you want to
     reach a point / open a miss distance *in the ground frame*. The airframe solves for the airspeed
     vector `v_air = v_ground_cmd − wind`, clamps `|v_air| ≤ v_max`, and integrates position by the
     **actual** ground velocity `v_air_clamped + wind`. Unclamped, the ground target is met exactly
     (crab is free); clamped, the aircraft drifts — reported honestly, not silently obeyed.
   - `target_speed` (fixed-wing) is an **airspeed** setpoint (`V_TAS`) — what a fixed-wing actually
     holds; `V_GS` then varies with heading and wind (Eq 4), which is the whole point.
   - `target_heading` (fixed-wing) is the desired **ground course** `χ`; the fixed-wing autopilot/
     dynamics adds the crab `θ_w` (Eq 3) so the *track* — not the nose — points where guidance wants.
     When `|(V_WS/V_TAS)·sin(θ_wa − χ)| > 1` the course is unachievable (downwind-dominated); this is
     the fixed-wing analog of the paper's insufficient-bank regime and is handled as an out-of-range
     projection (steer the closest achievable course), not silently.
   This extends the Phase-4 4d command-feasibility taxonomy (missing-channel fail-fast / out-of-range
   clamp-or-converge / absent-DOF ignore) with the ground-vs-air frame as an explicit axis.

5. **CD / CR / CRR are untouched.** They already separate conflicts in the **ground frame**
   (`velocity_enu` = ground velocity), which is precisely the frame wind changes and precisely what
   they should see — the intruder's wind-blown ground track *is* the threat. The only interaction:
   a resolver emits a desired *ground* velocity the airframe may be unable to hold under wind
   (airspeed or turn/crab limits) — that is decision 4's projection, reusing the Phase-4 4d
   feasibility handling. No CDR file opens except to confirm this end-to-end.

6. **Fixed-wing depends on Phase 4 4d being wind-ready.** 4d already commits to the air-relative
   integrator with the wind term present-but-zero and `ψ` integrated as primary
   ([[phase-4-plan]] 4d "Wind-ready by construction"). Phase 5's fixed-wing rung (5c) should therefore
   be *"feed a non-zero wind field and add the crab-holding autopilot,"* not a re-derivation. If 4d
   shipped without that structure, 5c absorbs the restructuring first — but the intent is that it
   does not have to.

---

## Phasing (each rung green before the next)

### 5a — `WindField` + kinematics + the `heading` state field (behaviour-preserving at `V_WS = 0`)

The point of 5a is that **nothing observable changes** while zero wind is supplied — pure plumbing,
gated by the bit-for-bit regression, before any aircraft actually feels wind.

- [ ] **`opencdarr/wind.py`** (new) — `WindField` (frozen value; `from_met(theta_wa_deg, v_ws)`
  factory applying Eq 1; `components() -> (w_x, w_y)`) and `NO_WIND = WindField(0.0, 0.0)`.
  - *Design:* uniform-constant now; the type is the seam for a future spatial/temporal field
    (decision 2). Meteorological "coming-from" convention documented at the boundary (Eq 1), because
    the sign is the single easiest thing to get wrong.
  - *Check:* a north wind (`θ_wa = 0`) gives `(w_x, w_y) = (0, −V_WS)` — air moving *toward* the
    south; a west wind (`θ_wa = 270`) gives `(+V_WS, 0)`.

- [ ] **`opencdarr/kinematics.py`** — wind helpers used by both dynamics and the autopilots:
  `wind_correction_angle(v_tas, wind, chi)` (Eq 3, returns `None`/raises on the `arcsin`
  out-of-range unachievable-course case), `ground_speed(v_tas, wind, psi)` (Eq 4),
  `air_to_ground(v_air_enu, wind)` / `ground_to_air(v_ground_enu, wind)` (the Eq 9 vector sum and its
  inverse), and `ground_track(v_ground_enu)`.
  - *Check:* `ground_speed` (closed form, Eq 4) equals `|air_to_ground(...)|` (integrated form) across
    a sweep of `ψ`; `air_to_ground`∘`ground_to_air` is identity.

- [ ] **`opencdarr/state.py`** — add `heading: float` (default `= trk`; behaviour-preserving
  construction, decision 3). Document it as the air-relative heading `ψ`, equal to `trk` exactly when
  `V_WS = 0` or for a holonomic vehicle not independently yawing; the crab is `heading − trk`.
  - *Check:* every existing test constructs `AircraftState` unchanged and reads `heading == trk`.

- [ ] **`opencdarr/dynamics/base.py`** — `Dynamics.step` gains `wind: WindField = NO_WIND` (decision
  2); `odometry_update` still takes the **ground** speed (odometry is a ground-path odometer —
  unchanged meaning, now fed the wind-affected `V_GS`). Update the ABC docstring's obligation list.
  - *Check:* every existing `.step(...)` call (no `wind=`) compiles and behaves identically.

- [ ] **`opencdarr/loop.py`** — `run_encounter` accepts an optional `wind: WindField = NO_WIND` and
  threads it into each `dynamics.step(...)`. No other change.
  - *Check (the 5a gate):* full suite green; **MVP/VO IPR sweep bit-for-bit** vs the ADR-0010 anchors
    with the default `NO_WIND`. One moved bit means 5a is not done.

### 5b — Multirotor (holonomic) under wind

- [ ] **`opencdarr/dynamics/holonomic.py`** — interpret `command.target_velocity` as a **ground**
  velocity (decision 4): compute required airspeed vector `v_air = v_ground_cmd − wind`, apply the
  existing isotropic `v_max`/`ax` limits **in the air frame** (that is where the airframe's envelope
  lives), then set the new ground velocity `= v_air + wind` and update `(trk, gs)` from it. `heading`
  tracks the air-velocity direction (or the independent yaw from 4e, unchanged).
  - *Check:* with a feasible command the ground velocity is met exactly regardless of wind (pure
    crab); commanding zero ground velocity **holds station** against a wind with `V_WS ≤ v_max`
    (hover into wind); a ground command needing airspeed `> v_max` clamps and drifts downwind by the
    reported amount. `V_WS = 0` reproduces Phase-4 holonomic bit-for-bit.

### 5c — Fixed-wing under wind (trochoidal ground tracks + wind-correction guidance)

- [ ] **`opencdarr/dynamics/fixedwing.py`** — supply the real wind to the already-air-relative
  integrator (4d): integrate `ψ` via bank (Eq 8, unchanged), form ground velocity by the Eq 9 vector
  sum, derive `(trk, gs)` and advance ground odometry. No structural change if 4d shipped wind-ready
  (decision 6).
  - *Check (analytical, vs the paper):* a full constant-bank turn at `V_TAS = 17 m/s`, `ϕ = 30°`
    traces a **circle in the air frame** (subtract the steady drift) and the paper's **trochoid over
    the ground** (reproduce its Fig. 4 for several `θ_wa`); the endpoint of one full revolution is
    displaced from the start by `wind × turn-period`; instantaneous `V_GS(t)` matches Eq 4 (its
    Fig. 3).

- [ ] **`opencdarr/autopilot/fixedwing.py`** — add wind correction to the guidance: to make good a
  desired ground course `χ` (from goto/waypoint steering, Phase 4 4b–4c), command heading
  `ψ = χ + θ_w(χ)` using Eq 3; on the unachievable-course case (`arcsin` out of range) steer the
  closest achievable course and surface it, don't silently stall the guidance (decision 4).
  - *Check:* the fixed-wing flies the same waypoint plan as in Phase-4 4c but *crabbed into wind*, and
    its **ground track** (not its nose) passes through the waypoints within capture radius; a
    cross-wind leg shows a steady non-zero crab equal to Eq 3.

- [ ] **`vault/observations/wind-multirotor-vs-fixedwing.md`** — the contrast doc (mirrors
  [[controlling-dubins-vs-holonomic]] / [[mixed-fleet-dubins-holonomic]]): same wind field, the
  multirotor crabs and can hold station while the fixed-wing turns trochoidally and is pushed
  downwind; side-by-side ground tracks + the `V_GS(ψ)` and crab plots. This is the qualitative payoff
  that shows the physics is right before the quantitative IPR sweep.

### 5d — Wind-aware separation and the IPR-under-wind sweep (the research payoff)

- [ ] **Mixed-fleet + wind through `run_encounter`.** The Phase-4 4d mixed multirotor-vs-fixed-wing
  encounter, now in a non-zero wind field, through the same entry point the IPR sweeps use.
  - *Check:* resolves (min-sep ≥ `rpz`) and produces a reproducible IPR; the fixed-wing's
    wind-limited feasible set (decision 4) is exercised, not bypassed.

- [ ] **IPR-vs-wind sweep.** Sweep wind magnitude/bearing (and airframe mix) and record how the
  resolved IPR moves — wind biases every aircraft's ground track, changing conflict geometry and the
  feasible avoidance set (a fixed-wing loses maneuver authority upwind). This is *the* Phase-5
  question the whole build exists to answer; capture it as an experiment note with a reproducible
  seed, like the Phase-2/3 sweeps.

- [ ] **ADR 001a — steady-uniform wind model + ground/air command semantics.** Records decisions
  1–6: wind as a threaded read-only environment input (not hidden state); the `WindField` value and
  `Dynamics.step` signature change; **`AircraftState.heading` added here** (redeeming the
  [[0010-dynamics-subpackage-and-odometry-state]] §4 deferral — heading lands *with* the wind model,
  as promised); ground-velocity vs airspeed command interpretation; and the explicit deferral of
  gusts/shear/turbulence/3-D wind (matching the paper's own future-work boundary). Supersedes nothing;
  extends 0007–0010 and Phase-4's ADR 0011.

---

## Tests (the gate for each rung)

- [ ] `test_wind.py` — Eq 1 sign convention (N/E/S/W winds → correct `(w_x, w_y)`); `NO_WIND` is zero.
- [ ] `test_kinematics_wind.py` — Eq 3 crab and Eq 4 ground speed match the integrated vector sum;
  `air_to_ground`/`ground_to_air` inverse; unachievable-course case detected.
- [ ] `test_state.py` (extend) — `heading` defaults to `trk`; existing constructions unaffected;
  clones carry `heading`.
- [ ] `test_loop.py` (extend) — **5a regression: MVP/VO IPR bit-for-bit** with `NO_WIND`.
- [ ] `test_holonomic_wind.py` — exact ground-velocity tracking under wind; hover-into-wind
  station-keeping; over-`v_max` clamp + downwind drift; `V_WS = 0` reproduces Phase-4 holonomic.
- [ ] `test_fixedwing_wind.py` — circle-in-air / trochoid-over-ground; one-revolution endpoint
  displacement; `V_GS(t)` vs Eq 4; crab vs Eq 3 on a held course; `V_WS = 0` reproduces Phase-4
  fixed-wing.
- [ ] `test_loop_wind.py` — mixed-fleet-in-wind resolves; IPR reproducible; wind-limited fixed-wing
  feasible set exercised.

---

## Out of scope, on purpose (with the escape hatch)

- **Energy / power model (Pang et al. [15] body-frame axial/lateral wind decomposition).** A natural
  and paper-cited extension — power depends on airspeed and the body-frame wind components, not ground
  speed — but not needed to *resolve conflicts*. If an endurance/energy-aware CR criterion ever wants
  it, it lands as a `crr/` or performance-model file with its own ADR, reading the airspeed the wind
  model already exposes. Flagged in `future-features/`, not built.
- **Trochoidal path *planner* (BSB/BBB/SBB, IEM, Bayesian optimization).** The paper's actual
  contribution, and firmly a *coverage-path-planning* problem, not vehicle dynamics or conflict
  resolution. Our autopilots navigate; wind constrains them. Not in this project's scope unless a
  mapping-mission feature is ever added.
- **Non-steady / non-uniform wind (gusts, shear, turbulence, 3-D wind, sideslip, unsteady aero).**
  Deferred *by the paper itself* (its §3.1 future-work note) and by us; the `WindField` type is the
  seam a spatial/temporal field slots behind later (decision 2), 3-D wind waits on the same 3-D
  extension as altitude ([[phase-4-plan]] decision 1).

## Relations to the companion docs

- `design_brief.md` / `design-philosophy.md` — wind is one more physical effect behind the existing
  `Dynamics` contribution surface (#10, a new effect = a file + a threaded input, not a loop fork);
  pure `state → value` layers (#1); one owner of clonable state (#2 — heading rides in
  `AircraftState`, the wind field is an explicit input, nothing hidden).
- ADRs: redeems the `heading`-with-wind deferral of [[0010-dynamics-subpackage-and-odometry-state]]
  §4 and [[0008-velocity-vector-command]] §4; builds on [[0007-dynamics-as-pluggable-interface]]
  (pluggable physics) and [[0009-holonomic-dynamics]]; extends Phase-4's `MotionCommand`/feasibility
  ADR 0011 with the ground-vs-air command frame.
- Phase 4: strictly downstream of 4a (`MotionCommand`) and 4d (wind-ready fixed-wing dynamics);
  closes 4d's forward pointer. The mixed-fleet entry point (4a decision 7) is what makes the
  IPR-under-wind sweep a one-argument change.
- Observations: new [[wind-multirotor-vs-fixedwing]] joins [[controlling-dubins-vs-holonomic]] and
  [[mixed-fleet-dubins-holonomic]] as the qualitative-contrast series.

## References (read, not ported)

- Reyner & Liem, *Energy-Efficient Trochoidal Path Planning for Unmanned Aircraft Under Wind and
  Performance Constraints*, Drones 2026, 10, 426 — `papers/drones-wind.pdf`. **Used:** the kinematic
  model, Eqs 1–9 (wind components, coordinated-turn yaw, ground vector-sum kinematics, ground speed,
  crab angle) and Figs. 3–4 (as analytical-validation targets). **Not used:** Section 3's
  path-planning methodology (BSB/BBB/SBB synthesis, IEM, Bayesian optimization). Re-derived and
  analytically validated ([[0002-analytical-validation-of-dynamics]]), never ported ([[lesson-learnt]]).
