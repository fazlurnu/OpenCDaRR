# Phase 4c — `FixedWing` dynamics (replaces Dubins) + PX4 lateral/longitudinal setpoints

Parent: [[phase-4-plan]]. The genuinely new physics. `FixedWing` consumes PX4's **ROS 2 fixed-wing
setpoints** (D1) — `FixedWingLateralSetpoint` (course / airspeed_direction / lateral_accel) +
`FixedWingLongitudinalSetpoint` (altitude / height_rate + equivalent_airspeed) — and chases them under
non-holonomic limits (stall, load factor, finite roll). **Hard-replaces `DubinsDynamics`**. Built
**wind-ready** (ψ primary, χ derived, `w=0`) so [[phase-5-plan|Phase 5]] only feeds a non-zero wind
vector. Full record: [[0013-fixedwing-coordinated-turn]] and
[[fixedwing-coordinated-turn|the derivation]].

## Migration discipline — DECIDED WITH THE USER (superseded the D2 bit-for-bit plan)

The user chose the **paper/PX4-faithful** path over a bit-for-bit-Dubins gate: a fixed-wing's turn rate
is speed-dependent (`g·tan φ/V`) and stall-bounded, which a fixed turn-rate cap cannot express. So:

1. `FixedWing` integrates ψ from **bank** (Reyner & Liem Eqs 8–9), with a real stall/load-factor
   envelope and **finite roll** (Eqs 15–17) — not a reparametrised Dubins core.
2. Validated **analytically against the paper's closed forms** (ADR 0002), **not** bit-for-bit Dubins.
3. `DubinsDynamics` / `step_dynamics` deleted; the **BlueSky trajectory anchor (ADR 0005) retired**
   with it (validation basis → analytical + PX4-interface fidelity).
4. The MVP/VO IPR **re-anchors**: the loop default is now `Multirotor`; the fixed-wing IPR (via the
   velocity→course projection resolvers need) lands in [[phase-4e-mixed-fleet-daa]].

## Checklist — DONE (2026-07-24)

- [x] **`MotionCommand` fixed-wing channels** — `target_course` (χ), `target_airspeed_direction` (ψ,
  overrides course), `target_airspeed` (equivalent airspeed), `target_lateral_accel` (optional).
  Replaced `target_heading` / `target_speed`. `target_altitude` / `target_vertical_speed` kept
  (defined, ignored, 2D).
- [x] **`AircraftState`** — `turn_rate` **removed** (Dubins-only), `bank` (φ) **added** as the carried
  angular state; `yaw` doubles as heading ψ. `create_aircraft` validates `|bank| ≤ phi_max`.
- [x] **`Performance`** — `max_tr`/`max_dtr2` **removed**; `phi_max` (bank limit) + `roll_rate_max`
  (roll rate) **added**. New airframe `SMALL_FIXEDWING` from the paper's example (V_TAS=17, φ_max≈44°,
  p_max=60°/s, stall 12, sourced comments).
- [x] **`opencdarr/dynamics/fixedwing.py`** — `FixedWing(Dynamics)`: bank-integrated ψ
  (`ψ̇ = g·tan φ/V`), finite roll, stall-in-turn bank limit, air-relative + wind (w=0) position,
  V_GS/χ as outputs, odometry. Feasibility: no fixed-wing channel → fail fast; airspeed clamped to
  `[v_stall, v_max]`; multirotor channels (`target_velocity`/`target_yaw`) ignored (absent DOF).
- [x] **Deleted `DubinsDynamics` + `step_dynamics`**; loop default → `Multirotor`; repointed
  `dynamics/__init__.py`, docstrings, and the three demo scripts; deleted `test_dynamics.py` +
  `test_dynamics_vs_bluesky.py`.
- [x] **ADR 0013** — [[0013-fixedwing-coordinated-turn]]; derivation [[fixedwing-coordinated-turn]];
  `step-dynamics-m600.md` annotated historical.

## Gate — GREEN

- [x] `test_fixedwing_dynamics.py` (11 tests) — wind-readiness (`ψ==trk`, `V_GS==V_TAS` every step at
  w=0); steady-turn radius = `V²/(g·tan φ)`; finite-roll Δψ vs Eq 15; cannot-stop; cannot-side-slip
  (velocity command fails fast); stall-in-turn bank limit; odometry; no-mutation.
- [x] `test_loop.py` — deterministic loop regression **re-anchored on `Multirotor`** (noiseless anchors
  unchanged, noisy ones updated). Fixed-wing MVP/VO IPR → Phase 4e (needs velocity→course projection).
- [x] Full suite green after the deletion; my files add zero ruff/mypy errors (mypy total unchanged at
  46 — the `FixedWing` `**odometry_update` splat mirrors the established pattern; deleting Dubins' two
  offsets it).
