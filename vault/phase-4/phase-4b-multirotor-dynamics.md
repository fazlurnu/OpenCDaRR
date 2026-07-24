# Phase 4b — `Multirotor` dynamics (replaces Holonomic) + yaw-carrying state

Parent: [[phase-4-plan]]. Delivers the first PX4-named model and makes yaw first-class (D2, D3).
`Multirotor` consumes a PX4 **`TrajectorySetpoint`**-shaped command (position / velocity / accel in
NED + yaw / yawspeed) and applies multirotor limits. **Hard-replaces `HolonomicDynamics`**, with a
bit-for-bit reproduction gate so the BlueSky-transferred validation is not lost.

## Migration discipline (D2)

1. Build `Multirotor` so that, with **no yaw command** (`yaw == trk` throughout) and a velocity-only
   command, its math is **byte-identical** to `HolonomicDynamics` (isotropic accel toward the commanded
   velocity vector, `v_max` clamp, hold-track on ~zero vector).
2. Prove it: port `test_holonomic_dynamics.py` onto `Multirotor`; the numeric anchors reproduce exactly.
3. **Then** delete `HolonomicDynamics` (class, file, exports) and repoint imports.
4. **Then** add the independent-yaw channel (below) — new capability, its own test that bites.

## Checklist — DONE (2026-07-24)

- [x] **`AircraftState.yaw`** — `yaw: float | None = None` (`None` = nose aligned with track, so every
  existing construction is unaffected — cleaner than a `= trk` default and it keeps the D2 reproduction
  exact). Clonable, same rationale as `turn_rate`.
- [x] **`MotionCommand.target_yaw` / `target_yawspeed`** — the PX4 `TrajectorySetpoint` yaw channel.
- [x] **`Performance.yaw_rate_max`** (deg/s, default `0.0`; M600 = `90.0`, a documented spec value, NOT
  from BlueSky). `max_tr`/`max_dtr2`/`v_min` left on `Performance` (still read by `DubinsDynamics` until
  4c) but documented as unused by `Multirotor`.
- [x] **`opencdarr/dynamics/multirotor.py`** — `Multirotor(Dynamics)`: Holonomic translation core
  (verbatim) + decoupled `_step_yaw`. **`target_position` tracking deferred to 4d** (the goto autopilot
  is its consumer; adding it now with no caller is speculative). Feasibility: missing velocity → fail
  fast; fixed-wing channels ignored (verified no-op).
- [x] **Deleted `HolonomicDynamics`** (`holonomic.py`, exports); repointed `dynamics/__init__.py`,
  `dubins.py`/`base.py` docstrings, and the two demo scripts to `Multirotor`.
- [x] **ADR 0012** — [[0012-multirotor-and-yaw-carrying-state]].

## Gate — GREEN

- [x] `test_multirotor_dynamics.py` — (a) reproduces the ADR-0009 Holonomic anchors byte-identical
  (envelope, isotropic accel, reversal-without-a-loop, odometry); (b) independent yaw: `(velocity east,
  yaw 45°)` → translates east, `yaw`→45°, `trk` stays 90°, never re-couple; rate-limited; hold-when-
  uncommanded; `target_yawspeed` integrates; (c) hover holds position and yaw; (d) feasibility.
- [x] Full suite green after the deletion (no dangling imports); my files add zero ruff/mypy errors
  (mypy total unchanged at 46 — the 2 `Multirotor` `**odometry_update` splat notes mirror `dubins.py`'s
  identical, pre-existing pattern).
