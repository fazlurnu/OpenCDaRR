# ADR 0012 — Multirotor dynamics (replaces Holonomic) + yaw-carrying state

- Status: accepted (Phase 4b gate green)
- Date: 2026-07-24
- Deciders: Fazlur Rahman

## Context

Phase 4 re-anchors the dynamics on **PX4 offboard-control semantics** (`vault/phase-4-plan.md`): a
`Dynamics` model's input is a PX4 offboard *setpoint*, and the models are named for the real vehicles
they simulate. This ADR delivers the first of those models, the **multirotor**, and — because a PX4
multirotor `TrajectorySetpoint` carries yaw / yawspeed as a native channel alongside
position/velocity — the **yaw-carrying state** that [[0008-velocity-vector-command]] §4 and
[[0010-dynamics-subpackage-and-odometry-state]] §4 deliberately deferred "to the model that gives it
meaning."

Two forces set the shape:

- **Hard replace (`vault/phase-4-plan.md` D2).** `HolonomicDynamics` (ADR 0009) is a rotorcraft
  approximation named for a property, not the vehicle. `Multirotor` supersedes it: same physics core,
  named for the airframe, PX4 setpoint as input, plus independent yaw.
- **Yaw is first-class, not an afterthought (D3).** The previous plan deferred independent yaw to a
  final rung; the PX4 offboard interface makes it a core multirotor channel, so it lands *here*.

## Decision

### 1. `Multirotor` replaces `HolonomicDynamics`; the validation transfers, the class does not

`HolonomicDynamics` is **deleted** (class, `holonomic.py`, exports). `Multirotor` (`multirotor.py`)
is the multirotor model going forward. The BlueSky-transferred validation ([[0002-analytical-validation-of-dynamics]],
[[0005-trajectory-validated-against-bluesky]]) is **not** lost: the migration was gated on `Multirotor`
reproducing the former Holonomic model **byte-for-byte** on its translation core before deletion. The
gate is `tests/test_multirotor_dynamics.py` — the exact ADR-0009 analytical anchors (envelope,
isotropic-acceleration limit, reversal-without-a-loop, shared odometry), ported verbatim onto
`Multirotor` and green. The class name is gone; the physics and its validation carried across.

### 2. Two decoupled channels, from one PX4 `TrajectorySetpoint`-shaped command

- **Translation** — the ADR-0009 holonomic core, unchanged: the ground-velocity vector chases
  `target_velocity` under an isotropic acceleration limit (`perf.ax`), clamped to `perf.v_max`; it can
  slow, stop, and hover. No coupled heading, no turn-rate limit.
- **Yaw** — the nose heading `AircraftState.yaw` converges toward `target_yaw` (or integrates
  `target_yawspeed`) under `perf.yaw_rate_max`, **independent of translation**. The two channels are
  decoupled by construction: a command `(target_velocity=east, target_yaw=45°)` translates east while
  the nose turns to 45°, and `trk` / `yaw` never re-couple. That decoupling is the capability the
  yaw-carrying state exists to express (camera-pointing, independent-yaw missions).

### 3. `AircraftState.yaw` — new field, `None`-default = nose aligned with track

`yaw: float | None = None`. `None` means the nose is aligned with the direction of travel (no
independent yaw commanded), so **every existing construction is unchanged** (none set it), and a
coupled-heading airframe never has to. A concrete value is the independently-controlled heading. It is
**clonable state**, not a derived quantity — the same reasoning `turn_rate` uses (`state.py`): an IPS
clone that lost it would point differently from its parent. Under wind it becomes the heading `ψ` whose
difference from track is the crab angle ([[phase-5-plan|Phase 5]]).

This resolves the exact deferral [[0008-velocity-vector-command]] §4 ("a future yaw-carrying state,
decided on its own — not smuggled back through signed speed") and
[[0010-dynamics-subpackage-and-odometry-state]] §4 ("the right field to add *with* the wind /
independent-yaw model that gives it meaning and its own ADR") named — now that a real consumer
(`Multirotor`) and a real interface reason (PX4 yaw setpoints) exist.

### 4. `MotionCommand.target_yaw` / `target_yawspeed`; `Performance.yaw_rate_max`

`target_yaw` [deg] / `target_yawspeed` [deg/s] are the PX4 `TrajectorySetpoint.yaw` / `.yawspeed`
channels (`None` = yaw not commanded → hold). `Performance` gains `yaw_rate_max` (deg/s, default
`0.0`), the multirotor yaw-rate limit; the M600 declares `90.0` — **not** a BlueSky value (BlueSky's
point-mass rotor model couples heading to track, so it has no independent yaw rate), a spec-level
figure used *only* by this decoupled channel. The Dubins-era `max_tr` / `max_dtr2` / `v_min` are not
read by `Multirotor`.

### 5. Feasibility taxonomy (ADR 0011 §1, applied)

- **Missing channel** — a command with no `target_velocity` is under-specified for a multirotor and
  **fails fast** (reading the velocity channel raises).
- **Out-of-range** — a velocity above `v_max` clamps; yaw converges under the rate limit.
- **Absent DOF** — the fixed-wing course/airspeed channels are **ignored** (no-ops), verified: adding
  them to a velocity command does not change the output.

## Alternatives rejected

- **Keep `HolonomicDynamics` alongside `Multirotor`.** Rejected (D2): two half-named models for one
  vehicle. The reproduction gate transfers the validation, so keeping the old class buys nothing.
- **Defer yaw to a final rung (the previous plan's ordering).** Rejected (D3): PX4 offboard carries
  yaw natively, so the multirotor interface is incomplete without it; adding it later would reshape
  the model twice.
- **`yaw` non-optional, defaulting to `trk` via `__post_init__`.** Rejected: `None` is a cleaner
  "not independently controlled" sentinel (matches `desired: … | None`), keeps every existing
  construction and the reproduction gate untouched, and lets an uncommanded multirotor stay
  track-aligned rather than freezing a concrete heading on step one.
- **Model roll/pitch/thrust (a lower-level multirotor).** Rejected (out of scope): the point-mass
  kinematic model is what CD/CR/CRR and the rare-event method need; attitude dynamics are a separate
  fidelity level, only if a scenario demands it (`state.py`'s no-speculative-structure rule).

## Consequences

- **Good:** the first PX4-named model exists; independent yaw is a first-class, tested capability; the
  BlueSky validation carried across the hard replace intact (bit-for-bit gate green). `AircraftState`
  grows the one field wind (Phase 5) and camera-pointing missions both need, without disturbing any
  existing construction.
- **Cost:** `AircraftState` has a nullable field whose `None` semantics ("aligned with track") a reader
  must know; documented on the field and resolved centrally in `Multirotor._step_yaw`.
- **Obligation:** the fixed-wing model (Phase 4c) must treat `target_yaw` as an absent DOF (ignore, or
  reject on inconsistency); 3D / altitude remains its own future ADR.

## Relations

- Supersedes [[0009-holonomic-dynamics]] (the class is replaced; its physics core and validation are
  reproduced by `Multirotor`), and builds on [[0010-dynamics-subpackage-and-odometry-state]] (odometry
  obligation, honoured via the shared `odometry_update`).
- Resolves the yaw-carrying-state deferral of [[0008-velocity-vector-command]] §4 and
  [[0010-dynamics-subpackage-and-odometry-state]] §4 (§3 above).
- Extends [[0011-motioncommand-and-guidance-separation]] — one of the per-vehicle model interpretations
  that ADR anticipated; the `target_yaw` channel is the D3 resolution recorded in its Update.
- Implements [[phase-4b-multirotor-dynamics]]; the wind heading `ψ` it sets up is [[phase-5-plan|Phase 5]]'s hook.
- Contrast with the coupled-heading `DubinsDynamics` (superseded by FixedWing in Phase 4c) is the
  [[controlling-dubins-vs-holonomic]] observation, now multirotor-vs-fixed-wing.
