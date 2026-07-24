# Phase 4 plan — PX4-aligned vehicle dynamics (Multirotor / FixedWing) + Mission / Autopilot / Separation split

Implements [[TODO-autopilot-separator-dynamic-integration]], re-anchored on a concrete target:
**the input a `Dynamics` model consumes is a PX4 *offboard setpoint*, and the layers above it are the
PX4 mode structure a real DAA-equipped drone flies under.** The end state we are simulating:

```
     my drone flies its mission autonomously        (PX4 Mission mode)
                     |
     detect-and-avoid perceives a conflict          (switch to Offboard)
                     |
     DAA streams avoidance setpoints                 (Offboard override)
                     |
     conflict clears -> resume the mission           (back to Mission, from the same waypoint)
```

The simulator must keep the four responsibilities a real flight stack keeps apart, and — this is the
new hard constraint versus the previous draft — the currency between the guidance/DAA layers and the
vehicle must be **exactly what PX4 accepts over its offboard interface**, so that this whole stack can
later be pointed at a real PX4 SITL/vehicle by swapping the `Dynamics` model for a MAVLink/uXRCE-DDS
transport, with nothing above it changing.

```
        Mission            = what should happen            (mission.py)          — inert intent
           |
           v
     Autopilot / Guidance  = how to achieve it             (autopilot/)          — Mission -> setpoint
           |                  (the "separate module" — the mission executor,
           |                   PX4's navigator, NOT the low-level controller)
           v
     Nominal MotionCommand  = a PX4 offboard setpoint
           |
           v
     SeparationManager      = is it safe near traffic?     (separation.py)       — the DAA overlay
           |                  Override (Offboard) if not; release (Mission) on recovery.
           v
     Final MotionCommand    = a PX4 offboard setpoint
           |
           v
       Dynamics             = the vehicle + its onboard controller   (dynamics/) — setpoint -> motion
           |                  (PX4's mc_pos_control / fw_lateral_longitudinal_control lives HERE)
           v
     AircraftState update
```

> **Terminology note (the one clash worth pinning).** PX4 calls the *whole* flight stack the
> "autopilot." In this codebase **Autopilot** names only the *guidance / mission-executor* layer — the
> setpoint **producer**. The low-level setpoint **tracker** (PX4's position / attitude / rate
> controllers) lives inside **Dynamics**. When the TODO says "the autopilot is a separate module," it
> means this producer layer is separate from the vehicle physics — which is what the split enforces.

Same working style as Phases 2–3: **one file at a time, read each diff, tick the box, one rung green
before the next.** This master plan is distributed into per-rung sub-plans under `vault/phase-4/`; we
execute them one by one.

---

## The three decisions that reshaped this draft (the pushbacks, resolved)

Recorded here because they overturn parts of both the TODO *and* the previous `phase-4-plan.md`.

### D1 — Fixed-wing offboard **does** take a heading/speed/altitude setpoint — over ROS 2, not MAVLink

The previous draft said a fixed-wing's natural command is `(target_heading, target_speed,
target_altitude)`. Over **MAVLink** that is false — fixed-wing offboard there is *position-only*
(`SET_POSITION_TARGET_LOCAL_NED`, velocity/acceleration ignored;
[PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard)). But over **ROS 2 / uXRCE-DDS**,
PX4 **v1.17** (May 2026) added fixed-wing-native setpoints
([release](https://px4.io/px4-autopilot-release-v1-17-what-you-need-to-know/),
[control interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface)):

- **[`FixedWingLateralSetpoint`](https://docs.px4.io/main/en/msg_docs/FixedWingLateralSetpoint)** —
  at least one of `course` (ground-track direction over ground, χ), `airspeed_direction` (the
  airspeed vector's heading, ψ — **overrides `course` when finite**), `lateral_acceleration`
  (m/s², FRD, feedforward). Units radians, NaN = "not directly controlled."
- **[`FixedWingLongitudinalSetpoint`](https://docs.px4.io/main/en/msg_docs/FixedWingLongitudinalSetpoint)**
  — `altitude` (m AMSL) **or** `height_rate` (m/s), plus `equivalent_airspeed` (m/s); or direct
  `pitch`/`throttle`. NaN = "use default / not controlled."
- Consumed by PX4's `fw_lateral_longitudinal_control` module — which in our architecture **is the
  FixedWing `Dynamics` model.** The waypoint→course guidance law (NPFG / L1) stays *above* it, in the
  autopilot, exactly as this plan wants.

**Consequence:** the fixed-wing command survives, but with PX4's real semantics:
- The old single `target_heading` **splits** into **`target_course` (χ, ground track)** and
  **`target_airspeed_direction` (ψ, heading)**. This is precisely the ψ≠χ crab distinction the prior
  4d "wind-ready" design already committed to integrating (`ψ` primary, `χ` derived) — PX4 now gives
  the two channels their own names, so we adopt them instead of inventing our own.
- The old `target_speed` becomes **`target_airspeed` (equivalent airspeed)**, not a ground speed —
  matching the prior 4d note that the fixed-wing envelope is an *airspeed* envelope.
- `target_altitude` / `target_height_rate` are the longitudinal channel — **defined now, ignored
  while 2D** (see D-scope below), which is what makes them PX4-complete without dead 3D state.

### D2 — **Hard-replace** Dubins/Holonomic with Multirotor/FixedWing (decided)

`DubinsDynamics` and `HolonomicDynamics` are **removed**; `Multirotor` and `FixedWing` are the only
models going forward. This is a deliberate namespace/clarity choice: the models are named for the real
vehicles and speak PX4 setpoints, and we do not keep two half-named approximations around.

**What we must not lose in doing so — the migration's safety gate.** Dubins/Holonomic are the
BlueSky-validated integrators ([[0005-trajectory-validated-against-bluesky]],
[[0002-analytical-validation-of-dynamics]]) and the source of the rare-event method's reproducible IPR
anchors ([[0010-dynamics-subpackage-and-odometry-state]]). We keep that **validation** even though we
delete the **classes**, by construction:

- **`FixedWing` reproduces the Dubins coupled-heading math bit-for-bit** on its lateral/speed core
  (course-tracking, turn-rate-limited, no stall active, no wind) **before** stall/airspeed physics are
  layered on. Gate: the MVP/VO IPR sweep that today runs on `DubinsDynamics` (MVP **0.9550**, VO
  **0.2050**) reproduces **byte-identical** on `FixedWing` at that core. The coupled-heading model was
  always physically a fixed-wing-shaped approximation — this is where it belongs.
- **`Multirotor` reproduces the Holonomic velocity-space math bit-for-bit** (isotropic accel, no yaw
  coupling) before independent yaw is layered on. Gate: the existing `test_holonomic_dynamics.py`
  numeric anchors reproduce byte-identical on `Multirotor`.

So "hard replace" is safe: each deletion is preceded by a green bit-for-bit reproduction on its
replacement. The BlueSky lineage transfers to the new class; only the old name goes away. (The
divergence between the two models is real and expected — it is the [[controlling-dubins-vs-holonomic]]
observation — so multirotor experiments re-anchor on `Multirotor`, fixed-wing on `FixedWing`.)

### D3 — Yaw is first-class, and lands **with** `Multirotor` (not deferred to the last rung)

A PX4 multirotor `TrajectorySetpoint` carries **yaw / yawspeed** as a native channel alongside
position/velocity — it is not an add-on. So `AircraftState.yaw` and `MotionCommand.target_yaw` /
`target_yawspeed` are built **as part of the Multirotor rung** (4b), under their own ADR, not left to a
final rung. `yaw` defaults to `= trk`, so every existing construction is unaffected and the
Multirotor-reproduces-Holonomic gate (D2) still holds at `yaw == trk`. This closes the deferral
[[0008-velocity-vector-command]] §4 and [[0010-dynamics-subpackage-and-odometry-state]] §4 named ("a
future yaw-carrying state, with its own ADR"): the offboard interface *is* the consumer that gives yaw
meaning.

---

## `MotionCommand` = the PX4 offboard setpoint (the single currency)

One vehicle-neutral value crosses every layer boundary (ADR 0011 §3 stance kept), but its fields and
their resolution now **mirror the PX4 offboard messages** so the mapping to a real vehicle is 1:1. A
model reads the subset its airframe controls and validates what it requires; every field defaults to
`None` ("not commanded" — PX4's `NaN` / `type_mask` unset).

| `MotionCommand` field | PX4 message · field | Vehicle | Units | Status this pass |
|---|---|---|---|---|
| `target_position` | `TrajectorySetpoint.position` (NED) / `SET_POSITION_TARGET_*` | Multirotor, FixedWing* | (lat, lon) [+alt] | live (2D) |
| `target_velocity` | `TrajectorySetpoint.velocity` (NED) | Multirotor | (v_e, v_n) [m/s] | live — MVP/VO native output |
| `target_acceleration` | `TrajectorySetpoint.acceleration` | Multirotor | (a_e, a_n) [m/s²] | optional / feedforward |
| `target_yaw` | `TrajectorySetpoint.yaw` | Multirotor | [deg] | **new in 4b** (D3) |
| `target_yawspeed` | `TrajectorySetpoint.yawspeed` | Multirotor | [deg/s] | **new in 4b** (D3) |
| `target_course` | `FixedWingLateralSetpoint.course` (χ) | FixedWing | [deg] | **new in 4c** (was `target_heading`) |
| `target_airspeed_direction` | `FixedWingLateralSetpoint.airspeed_direction` (ψ) | FixedWing | [deg] | **new in 4c**; wind field |
| `target_airspeed` | `FixedWingLongitudinalSetpoint.equivalent_airspeed` | FixedWing | [m/s] | **new in 4c** (was `target_speed`) |
| `target_altitude` | `FixedWingLongitudinalSetpoint.altitude` | FixedWing | [m AMSL] | defined, **ignored (2D)** |
| `target_height_rate` | `FixedWingLongitudinalSetpoint.height_rate` | FixedWing | [m/s] | defined, **ignored (2D)** |

\* Fixed-wing `target_position` is the *guidance input* the autopilot turns into a `target_course`;
the FixedWing dynamics consumes the lateral/longitudinal channels, not raw position (that is PX4's
own division: navigator → `fw_lateral_longitudinal_control`).

**Channel resolution mirrors `OffboardControlMode` priority**, not a flat fail-fast: PX4 selects the
single active channel by priority (`position > velocity > acceleration > attitude > rate`). So a model
resolves which channel it is being commanded on by that order and validates that **at least one** it
understands is present — an all-`None` command for an airframe is the programming error that fails
fast (ADR 0011 §1 refined: fail on *none*, resolve *multiple* by PX4 priority, ignore channels the
airframe lacks).

The Phase-4a work already in the tree (`MotionCommand`, `from_track_speed`, `gs`/`trk` helpers,
MVP/VO returning `MotionCommand(target_velocity=...)`, the `Command` alias) is the seed of this — 4a
finishes it and locks the bit-for-bit regression; 4b/4c grow the PX4 field set above.

---

## Settled scope (firm — shapes every rung)

1. **2D only, this pass.** `target_altitude` / `target_height_rate` are *defined* (PX4-complete
   interface) but **ignored** by every model; `AircraftState` stays horizontal. Altitude + vertical
   detection/level math land with a dedicated 3D ADR when the fixed-wing longitudinal channel is
   actually flown — the deferral `state.py` and [[0010-dynamics-subpackage-and-odometry-state]] §4
   already commit to. No dead 3D state now.
2. **`SeparationManager` holds no mutable object state** (unchanged from ADR 0011 §5, load-bearing).
   `PairMemory` is threaded in/out; the KI-1 recovery-state leak must not return. The **mission-resume
   index** (D-mission) is bound by the same rule: it is clonable value state, never an attribute on the
   autopilot.
3. **Pairwise (n=2) stays.** `SeparationManager.step` takes `perceived_traffic: list[AircraftState]`
   for n>2 future-proofing; the loop feeds `[other]` / `[]`. Coordination is the separate IPS roadmap
   ([[0004-layered-directed-design-for-multiaircraft-and-ips]]).
4. **CD / CR / CRR untouched.** Already dynamics-agnostic ([[mixed-fleet-dubins-holonomic]]); the only
   edit is MVP/VO returning `MotionCommand` (done in 4a). MVP/VO emit a **velocity** setpoint, which is
   natively a valid *multirotor* offboard channel — the multirotor DAA path is PX4-faithful end to end.
   Projecting a velocity avoidance onto a *fixed-wing* setpoint (course/airspeed) is a real open
   question flagged in 4e, not silently assumed.
5. **Per-aircraft `(mission, autopilot, dynamics, perf)`** threaded through `run_encounter` (ADR 0011
   §7) — what lets a mixed multirotor-vs-fixedwing encounter run through the IPR entry point.

**D-mission — Mission ↔ Offboard as an explicit, resumable mode.** The DAA interrupt/resume the user
described is PX4's Mission↔Offboard switch. The autopilot's active-waypoint index persists across the
interruption **in clonable state**, so recovery resumes the mission from the same leg — the
"resume later" requirement. Whether we surface a literal `FlightMode {MISSION, OFFBOARD}` enum or keep
it implicit in nominal-vs-final command selection is decided in 4d; either way the resume index is
clonable and the SeparationManager owns no mode state.

---

## Rungs (each green before the next; one sub-plan file each)

| Rung | Sub-plan | Delivers | Gate |
|---|---|---|---|
| **4a** | [[phase-4a-motioncommand-layer-split]] | Finish `MotionCommand` as the offboard currency; homes for Autopilot / SeparationManager; rewire `run_encounter` to the layered flow, per aircraft | MVP/VO IPR **bit-for-bit** vs ADR 0010 anchors (behaviour-preserving) |
| **4b** | [[phase-4b-multirotor-dynamics]] | `Multirotor` (replaces Holonomic) + `AircraftState.yaw` + `target_yaw`/`target_yawspeed`; delete `HolonomicDynamics` | Reproduces Holonomic anchors byte-identical at `yaw==trk`, **then** independent yaw bites |
| **4c** | [[phase-4c-fixedwing-dynamics]] | `FixedWing` (replaces Dubins) — coupled-heading core + stall + PX4 lateral/longitudinal channels, wind-ready (ψ primary, χ derived, w=0); delete `DubinsDynamics` | Reproduces the **Dubins** MVP/VO IPR (0.9550 / 0.2050) byte-identical at the w=0 core, **then** stall/airspeed bites |
| **4d** | [[phase-4d-mission-autopilot]] | `Mission` + `Autopilot` ladder (goto → waypoints → loiter) for both airframes; Mission↔Offboard mode switch + clonable resume index | Each rung's functional test bites on multirotor **and** fixed-wing; mid-plan clone resumes the same leg |
| **4e** | [[phase-4e-mixed-fleet-daa]] | Mixed multirotor-vs-fixedwing through `run_encounter`; the velocity→fixed-wing-setpoint projection for DAA | Mixed encounter resolves (min-sep ≥ rpz), reproducible IPR |

*(VTOL — composition over Multirotor/FixedWing — deferred out of Phase 4 for now.)*

ADRs land with their rung: **ADR 0011** (4a, already drafted — update its fixed-wing §3 to the
D1/ROS 2 semantics), **ADR 001x — Multirotor + yaw-carrying state** (4b), **ADR 001y — FixedWing +
PX4 lateral/longitudinal setpoints** (4c, cites Reyner & Liem `papers/drones-wind.pdf` for the
coordinated-turn kinematics, re-derived not ported), **ADR 001z — 3D/altitude** (when the longitudinal
channel is flown).

---

## Command feasibility — the taxonomy every airframe applies

A `MotionCommand` is a *setpoint*, never a state to snap to; each model projects it onto its feasible
set. Three distinct failures, three handlings (pinned per model + per test):

| Infeasibility | Example | Handling |
|---|---|---|
| **No channel this airframe controls is set** | all fields `None` | **fail fast** — programming error |
| **Out-of-range value** | fixed-wing `target_airspeed = 2` m/s (< stall); a 179° course step | **clamp / converge** — reachable over time under rate/envelope limits |
| **Absent degree of freedom** | `target_altitude` (2D); `target_yaw` on a fixed-wing; `target_course` on a multirotor | **ignore** (no actuator); optional fail-fast on a value that *contradicts* the airframe (e.g. a fixed-wing `airspeed_direction` disagreeing with `course` beyond crab tolerance ⇒ likely a wiring bug) |

This is the same taxonomy the prior draft carried, retained because it is airframe-general and PX4's
`NaN`/priority model is exactly "ignore the channels you don't control, converge on the ones you do."

---

## Relations

- **Supersedes** the previous `phase-4-plan.md` framing (vehicle *classes over kept Dubins/Holonomic*)
  → **hard-replace** into PX4-named models (D2).
- ADRs: extends [[0007-dynamics-as-pluggable-interface]]; **replaces** the models of
  [[0009-holonomic-dynamics]] / the Dubins integrator while **transferring their validation**
  ([[0002-analytical-validation-of-dynamics]], [[0005-trajectory-validated-against-bluesky]]) via the
  bit-for-bit gates (D2); [[0011-motioncommand-and-guidance-separation]] gets its fixed-wing §3
  updated to the ROS 2 semantics (D1) and its yaw deferral resolved (D3).
- Observations: [[controlling-dubins-vs-holonomic]] (why the two new models legitimately diverge),
  [[mixed-fleet-dubins-holonomic]] (the per-aircraft-dynamics follow-up closed in 4a/4e).
- [[phase-5-plan|Phase 5]] turns the FixedWing wind term on (the ψ/χ split built inert at w=0 in 4c).

## References (read, not ported)

- **PX4 offboard surface** (the interface we mirror): [PX4 Offboard mode](https://docs.px4.io/main/en/flight_modes/offboard),
  [ROS 2 control interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface),
  [`FixedWingLateralSetpoint`](https://docs.px4.io/main/en/msg_docs/FixedWingLateralSetpoint),
  [`FixedWingLongitudinalSetpoint`](https://docs.px4.io/main/en/msg_docs/FixedWingLongitudinalSetpoint),
  [v1.17 release](https://px4.io/px4-autopilot-release-v1-17-what-you-need-to-know/).
- **Fixed-wing kinematics:** Reyner & Liem, *Energy-Efficient Trochoidal Path Planning…* (Drones 2026,
  10, 426) — `papers/drones-wind.pdf`; we take only its kinematic coordinated-turn + wind vector-sum
  point-mass model (Eqs 1–9), re-derived and analytically validated, never the path planner.
- **The TODO this implements:** [[TODO-autopilot-separator-dynamic-integration]].
