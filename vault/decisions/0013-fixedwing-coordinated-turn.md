# ADR 0013 — FixedWing dynamics (coordinated-turn, PX4 lateral/longitudinal setpoints); replaces Dubins

- Status: accepted (Phase 4c gate green)
- Date: 2026-07-24
- Deciders: Fazlur Rahman

## Context

Phase 4 re-anchors the dynamics on **PX4 offboard semantics** (`vault/phase-4-plan.md`): a `Dynamics`
model consumes a PX4 offboard *setpoint*, and models are named for real vehicles. 4b delivered
`Multirotor` (replacing `HolonomicDynamics`). This ADR delivers the genuinely new physics — a
**fixed-wing** coordinated-turn point mass — and **hard-replaces** `DubinsDynamics` / `step_dynamics`.

The physics source is Reyner & Liem, *Energy-Efficient Trochoidal Path Planning under Wind and
Performance Constraints* (Drones 2026, 10, 426; `vault/papers/drones-wind.pdf`). We take only its
**kinematic point-mass model** (its Eqs 1–9, 13–17: coordinated-turn yaw + wind vector-sum
kinematics), re-derived and analytically validated (ADR 0002), never its path planner (BSB/BBB/SBB
maneuver synthesis, the IEM energy metric, the Bayesian optimisation). Verified during planning: that
kinematic model **is** the model PX4's `fw_lateral_longitudinal_control` implements.

## Decision

### 1. The coordinated-turn model (the equations of motion)

Inertial frame (x=east, y=north), all angles CW from north. State = position + heading **ψ** + bank
**φ**; constant true airspeed `V_TAS`; steady horizontal wind `(w_x, w_y)`:

```
ẋ = V_TAS·sin ψ + w_x          (Eq 9)
ẏ = V_TAS·cos ψ + w_y          (Eq 9)
ψ̇ = g·tan φ / V_TAS            (Eq 8)   coordinated turn: bank produces the yaw rate
```

Ground speed `V_GS = √(ẋ²+ẏ²)` (Eq 4) and course `χ = atan2(ẋ, ẏ)` are **outputs** derived each step;
the crab angle is `θ_w = ψ − χ` (Eq 2). Turn geometry: min-radius `R = V_TAS²/(g·tan φ)`, load factor
`n = 1/cos φ`, stall-in-turn `V_stall(φ) = V_stall·√n`. Derivation, symbols matching the code:
`vault/derivations/fixedwing-coordinated-turn.md`.

### 2. Paper/PX4-faithful, not bit-for-bit Dubins (decided with the user)

A fixed-wing's turn rate is **speed-dependent** (`g·tan φ / V`) and bounded by stall — a fixed
turn-rate cap (Dubins' `max_tr`) cannot express that. So `FixedWing` integrates ψ from **bank**, with a
real stall/load-factor envelope, and is validated **analytically against the paper's closed forms**
(ADR 0002 discipline: `tests/test_fixedwing_dynamics.py` — steady-turn radius, finite-roll heading
change, cannot-stop, cannot-side-slip, stall-in-turn). It does **not** reproduce the Dubins MVP/VO IPR
bit-for-bit, because the physics legitimately differs; the alternative (keep a turn-rate core and
reparametrise bank as cosmetic) was considered and rejected as less faithful (see Alternatives).

### 3. Finite roll (decided with the user)

Bank φ is **state**, roll-rate-limited (`|φ̇| ≤ roll_rate_max`, Eqs 15–17) — the analog of the deleted
`max_dtr2`. `AircraftState.turn_rate` is **removed** (its only reader, Dubins, is gone) and replaced by
`AircraftState.bank`. `Performance` loses `max_tr` / `max_dtr2` and gains `phi_max` (bank limit) and
`roll_rate_max` (roll rate). The heading ψ is carried in the existing `AircraftState.yaw` field (added
ADR 0012 for the multirotor's nose): a `FixedWing` always sets `yaw = ψ`; at zero wind `yaw == trk`.

### 4. Consumes the PX4 fixed-wing ROS 2 setpoint channels

`MotionCommand` gains `target_course` (χ, `FixedWingLateralSetpoint.course`),
`target_airspeed_direction` (ψ, `.airspeed_direction`, **overrides course when set**, PX4 semantics),
`target_airspeed` (equivalent airspeed, `FixedWingLongitudinalSetpoint.equivalent_airspeed`), and
`target_lateral_accel` (`.lateral_acceleration`, optional). The old `target_heading` / `target_speed`
are removed. `target_altitude` / `target_vertical_speed` stay defined-but-ignored (2D, ADR 0011 §4).
Feasibility (ADR 0011 §1): a command with **no** fixed-wing channel fails fast; airspeed below stall
clamps; the multirotor channels (`target_velocity` / `target_yaw`) are an **absent DOF** and are
ignored — so a raw velocity command (a resolver's native output) cannot fly a fixed-wing until it is
projected to a course/airspeed setpoint (Phase 4e).

### 5. Wind-ready by construction

The wind term `(w_x, w_y)` is present in the position update but **fixed at zero this pass**. At zero
wind `ψ == χ` and `V_GS == V_TAS` (an analytical invariant, asserted every step). Phase 5 turns wind on
by feeding a non-zero vector — no change to the integrator, and the ψ/χ split already distinguishes
"the speed I fly" (airspeed) from "the speed I make good" (ground speed) and heading from course.

## Alternatives rejected

- **Keep the Dubins turn-rate integrator as the validated core, reparametrise bank/stall over it.**
  Rejected: it preserves the bit-for-bit IPR anchor but the turn transient stays turn-accel-limited
  (Dubins), not roll-rate-limited (the paper), and turn radius stays speed-independent — the opposite
  of what a fixed-wing does. The user chose the faithful path.
- **Instantaneous bank (no finite roll) first.** Rejected by the user in favour of finite roll now
  (Eqs 15–17), for realistic transients from the start.
- **Keep `DubinsDynamics` as a frozen BlueSky reference alongside FixedWing.** Considered (the plan
  flagged it at approval) and rejected — hard replace; see the BlueSky-anchor consequence below.

## Consequences

- **Good:** the fixed-wing model is genuinely non-holonomic — speed-dependent turn radius, load factor,
  stall, finite roll — and PX4-interface-faithful; wind-ready for Phase 5. It is validated analytically
  against the paper's closed forms.
- **The BlueSky trajectory anchor (ADR 0005) is retired.** Deleting Dubins removed the only model our
  BlueSky fork validated against (`test_dynamics_vs_bluesky.py`, deleted); nothing is BlueSky-anchored
  after 4c. The validation basis is now analytical-vs-paper (ADR 0002) + PX4-interface fidelity.
- **The MVP/VO IPR re-anchors.** The loop default is now `Multirotor` (ADR 0012); the deterministic
  loop regression re-anchored on it (`tests/test_loop.py` — noiseless anchors unchanged, noisy ones
  moved). The MVP/VO IPR on the *fixed-wing* airframe lands in Phase 4e, which adds the velocity→course
  projection resolvers need to command a fixed-wing.
- **Cost:** `type: ignore`-free except the shared `**odometry_update` splat (the established pattern,
  ADR 0010). `AircraftState` grew `bank` and dropped `turn_rate`; `Performance` swapped
  `max_tr`/`max_dtr2` for `phi_max`/`roll_rate_max`.

## Relations

- Supersedes the coupled-heading Dubins integrator (`step_dynamics`) and retires its BlueSky anchor
  [[0005-trajectory-validated-against-bluesky]]; validation moves to [[0002-analytical-validation-of-dynamics]].
- Extends [[0011-motioncommand-and-guidance-separation]] (the fixed-wing channels of `MotionCommand`,
  the D1 ROS 2 semantics its Update anticipated) and sits beside [[0012-multirotor-and-yaw-carrying-state]]
  (reuses the `yaw` field as heading ψ).
- Builds the wind-ready hook [[phase-5-plan|Phase 5]] switches on; the velocity→course projection it
  needs for DAA lands in [[phase-4e-mixed-fleet-daa]].
- Implements [[phase-4c-fixedwing-dynamics]]; the ψ/χ contrast with the multirotor's velocity-space
  turn is the [[controlling-dubins-vs-holonomic]] observation's successor.
