# ADR 0014 — Mission + position-setpoint guidance (navigator/controller split), L1, automatic resume

- Status: accepted (Phase 4d gate green)
- Date: 2026-07-24
- Deciders: Fazlur Rahman

## Context

4a–4c gave the PX4-aligned layer split and the two airframes, but the only guidance was
`CruiseAutopilot` — a frozen heading. Phase 4d turns "the aircraft can be commanded" into "the
aircraft can navigate": fly a `Mission` (goto → waypoints → loiter), be interrupted by the DAA, and
**resume from the same leg** — the operational flow a DAA-equipped drone actually flies
(`vault/phase-4-plan.md`, `phase-4/phase-4d-mission-autopilot.md`).

## Decision

### 1. Position setpoints, tracked in the dynamics (the navigator/controller split)

The autopilot (**navigator**) emits a **position setpoint** for the active waypoint —
`MotionCommand(target_position=B, target_leg_start=A, target_airspeed=…)` — and each airframe's `step`
(the **controller**) tracks it. This is what the `Autopilot` ABC already committed to ("the low-level
setpoint tracker lives inside `Dynamics`") and mirrors PX4 mission mode (navigator sets the setpoint;
the position controller flies it). Consequence: **one vehicle-neutral autopilot** (`WaypointAutopilot`)
serves both airframes, and loiter (hover vs orbit) *emerges from the tracker*, not the autopilot.

- **Multirotor** tracks `target_position` with the stopping-distance law: desired speed
  `min(v_max, √(2·ax·range))` toward the point, plus a small hover-capture deadband — it flies
  straight in and settles to a hover, with no tuning constant.
- **FixedWing** tracks the leg with **L1 guidance** (§2), and orbits at a loiter setpoint (§3).

Channel priority follows PX4 `OffboardControlMode` (position > velocity): a mission nominal carries a
position; a DAA override carries a velocity; the airframe interprets whichever is set. The autopilot
emitting a *position* (rather than a pre-computed velocity/course) is what keeps it vehicle-neutral —
the alternative (vehicle-specific autopilots emitting velocity/course, dynamics untouched) was
rejected as less PX4-faithful and as duplicating the sequencing logic.

### 2. L1/NPFG cross-track tracking for the fixed-wing (chosen over pure-pursuit)

`FixedWing` follows the **leg line** (previous → current waypoint), nulling cross-track error, via L1
guidance (`vault/derivations/l1-guidance.md`): steer toward an L1 reference point a lookahead ahead
on the line, so an off-track aircraft curves *onto* the line and tracks *along* it — and, crucially,
**re-intercepts the planned track after a DAA excursion** rather than cutting a chord to the distant
endpoint (which pure-pursuit would). A bare goto (no `target_leg_start`) degenerates to pure-pursuit.
Wind is zero this pass (NPFG ≈ a well-tuned L1 without wind); the wind-robust NPFG form and the crab
angle land with Phase 5.

### 3. Loiter emerges from the tracker

`MotionCommand.target_loiter_radius` (set on arrival at the final waypoint). A multirotor **hovers**
at the centre (radius ignored); a fixed-wing, which cannot stop, flies a **min-radius orbit** — a
single law `course = bearing_to_centre + asin(radius/range)` that captures the circle from outside
and holds it. Same autopilot, same setpoint, airframe-appropriate behaviour.

### 4. Guidance progress is threaded clonable state, never hidden (the IPS invariant)

The active-leg index lives in a `GuidanceMemory` value threaded **in** to `Autopilot.step` and
returned **out** — the same no-hidden-state discipline `PairMemory` obeys (ADR 0011 §5). The ABC is
`step(state, memory, perf) -> (MotionCommand, GuidanceMemory)`; `CruiseAutopilot` threads it
untouched. This is load-bearing for the rare-event method: an IPS clone taken mid-plan must resume
the *same* leg, so the index must ride inside the clonable particle (settled explicitly in the mode
discussion — the label `mode = OFFBOARD if resolving else MISSION` is a *derived view*; only the
underlying `resolving` / leg-index are state). Mission positions are WGS84 `lat/lon` (the frame
`AircraftState` uses); the tracker converts to bearings/ranges via `geo`.

### 5. Mission ↔ Offboard resume is automatic — no mode machine

The `SeparationManager` override *is* the offboard interrupt; releasing to the nominal on recovery
*is* the return to Mission. Because the autopilot is re-invoked every tick from the live state + the
persisted leg index, releasing the override simply continues the mission from wherever the avoidance
left the aircraft — and the fixed-wing's L1 re-intercepts the leg line. The leg index is never reset
by the override (it lives in the separately-threaded `GuidanceMemory`). No `FlightMode` state is
introduced (ADR 0011 D-mission decision resolved: keep it implicit).

## Alternatives rejected

- **Guidance in the autopilot (emit velocity/course), dynamics untouched.** Rejected (§1): less
  PX4-faithful, and needs vehicle-specific autopilots that duplicate sequencing.
- **Pure-pursuit for the fixed-wing.** Rejected (§2): it cuts corners and, after a DAA excursion,
  flies to the endpoint from wherever it ended up rather than re-intercepting the planned line — the
  L1 re-intercept is exactly the property a DAA study wants.
- **Leg index on `AircraftState` or the autopilot object.** Rejected (§4): kinematics vs guidance
  memory separation, and a mutable autopilot attribute would not clone (the KI-1 hazard).
- **An explicit `FlightMode {MISSION, OFFBOARD}` machine.** Rejected (§5): the mode is derivable from
  `resolving` + leg index; storing it is a second source of truth (see the mode discussion).

## Consequences

- **Good:** both airframes navigate real missions through one vehicle-neutral autopilot; the fixed-wing
  tracks legs and re-intercepts after avoidance; loiter, sequencing, and clone-correct resume all work
  (tests bite). The airframe owns the vehicle-specific tracking, the autopilot stays neutral.
- **Cost:** `Multirotor` and `FixedWing` grew a position channel; `MotionCommand` grew
  `target_leg_start` / `target_loiter_radius`; the `Autopilot` ABC now threads memory.
- **Obligation / deferred:**
  - **Fixed-wing DAA interrupt** needs the velocity→course projection (MVP/VO emit a velocity a
    fixed-wing can't fly) — **Phase 4e** (prototyped in `scripts/l1_reintercept_demo.py`). 4d's resume
    is demonstrated + tested on the multirotor.
  - **NPFG / wind-robust L1** and the crab angle → Phase 5; **L1-on-a-circle** loiter is a refinement.
  - Per-aircraft external `dynamics`/`perf` (mixed fleet) remains Phase 4e.

## Relations

- Realises the guidance layer of [[0011-motioncommand-and-guidance-separation]] (the `Autopilot`
  family it defined) and uses its `PairMemory` threading pattern for `GuidanceMemory`.
- Consumes the airframes of [[0012-multirotor-and-yaw-carrying-state]] /
  [[0013-fixedwing-coordinated-turn]] as position/leg trackers; the L1 law is
  `vault/derivations/l1-guidance.md`.
- Implements [[phase-4d-mission-autopilot]]; the deferred fixed-wing DAA is [[phase-4e-mixed-fleet-daa]].
- Demos: `scripts/mission_demo.py`, `scripts/resume_demo.py`, `scripts/l1_reintercept_demo.py`.
