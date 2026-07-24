# ADR 0015 — Velocity→course/airspeed projection for fixed-wing DAA; per-aircraft `run_encounter`

- Status: accepted (Phase 4e gate green)
- Date: 2026-07-24
- Deciders: Fazlur Rahman

## Context

4a–4d gave the layer split, the two airframes, and mission guidance. The one honest gap remained
(named in ADR 0011 §7, ADR 0013 §4, ADR 0014's Obligations): **a `MotionCommand.target_velocity` is
a native multirotor offboard channel but not a fixed-wing one.** `MVP`/`VO` emit an avoidance
*velocity*; `Multirotor` flies it directly; `FixedWing.step` **fails fast** on a raw velocity (it
takes a lateral `course` + a longitudinal `airspeed`). So a velocity avoidance could not reach a
fixed-wing at all, and a mixed multirotor-vs-fixed-wing encounter could not run through the normal
entry point (`run_encounter` still took one shared `dynamics`/`perf`). Phase 4e
(`vault/phase-4/phase-4e-mixed-fleet-daa.md`) decides how a velocity avoidance reaches a fixed-wing,
and threads the per-aircraft bundle ADR 0011 §7 deferred. Prototyped in
`scripts/l1_reintercept_demo.py`'s `_project_velocity`.

## Decision

### 1. The projection is a thin adapter at the separation layer, not in `MVP`/`VO`

`separation.project_to_fixedwing(command, perf)` lowers a velocity command onto the fixed-wing
channels:

```
target_course   = atan2(v_east, v_north)                     (track of the avoidance velocity)
target_airspeed = clamp(|velocity|, [perf.v_min, perf.v_max]) (stall .. max airspeed)
```

A command with **no** `target_velocity` (a position/leg nominal, or an already-projected course) is
a valid fixed-wing setpoint and passes through untouched. `MVP`/`VO` are **unchanged** — they still
emit a vehicle-neutral velocity (ADR 0011 §2). The adapter is a `SetpointAdapter =
Callable[[MotionCommand], MotionCommand]` threaded into `SeparationManager.step` (default `None` =
identity, a multirotor) and applied to **every** command the manager returns — nominal, override,
*and* coast. Applying it to all exits is load-bearing: the DAA **override** (not just the mission
nominal) is what carries the velocity a fixed-wing cannot fly, and a `CruiseAutopilot` nominal is
itself a velocity, so a fixed-wing must never leave the separation layer holding one.

### 2. It lives at the separation layer, not in a fixed-wing autopilot or the dynamics

- **Not a fixed-wing autopilot:** the autopilot only produces the *nominal*; it never sees the
  resolver's override velocity, which is the command that most needs projecting. Only the layer that
  emits the override (the `SeparationManager`) can project it.
- **Not the dynamics:** `FixedWing.step` deliberately fails fast on a raw velocity (ADR 0013 §4, its
  test bites). The projection *supplies* a valid setpoint above that boundary; it does not weaken the
  fail-fast contract. Vehicle interpretation of a *valid* channel stays in the airframe (ADR 0011
  §3); manufacturing a channel the resolver didn't emit is a separation-layer concern.

The `SeparationManager` stays vehicle-neutral: it applies whatever adapter it is given. The loop is
the composition root that pairs an airframe with its adapter (`loop._setpoint_adapter`: a `FixedWing`
gets `project_to_fixedwing` bound to its `perf`; a `Multirotor` gets `None`).

### 3. It is an approximation — and the approximation is the physics

The projected velocity is one a multirotor reaches essentially instantly, but a fixed-wing only
**converges** to: `FixedWing.step` turns onto `target_course` under its bank/roll limit and ramps to
`target_airspeed` under `ax`. The lag between the velocity the resolver asked for and the velocity
the airframe is making good is the correct airframe difference, not a defect ([[mixed-fleet-daa]]
draws it as the commanded-vs-achieved-track gap). The airspeed clamp means a resolver demanding a
speed outside `[v_min, v_max]` is silently clamped (the fixed-wing then resolves more by heading) —
accepted this pass, not a modelled infeasibility signal.

### 4. `run_encounter` threads `(dynamics, perf)` per aircraft (ADR 0011 §7, realised)

`own_dynamics` / `own_perf` / `intr_dynamics` / `intr_perf`, each defaulting to the shared
`dynamics` / `perf`. Single-airframe callers and the bit-for-bit multirotor anchors are unchanged
(the per-aircraft args are `None` → shared → identical); a mixed pair passes both airframes and runs
through the same entry point the IPR sweeps use. No `Vehicle` grouping class (ADR 0011 §7's
"no speculative structure"): plain per-aircraft arguments.

## Alternatives rejected

- **Project inside `MVP`/`VO`.** Rejected: it makes the resolvers vehicle-aware, reversing ADR 0011
  §2 / ADR 0008 — the velocity is the resolvers' native, neutral output.
- **Project inside `FixedWing.step` (interpret velocity as course/airspeed).** Rejected: it deletes
  the fail-fast contract (ADR 0013 §4) and hides a lossy approximation inside the integrator where a
  reader expects faithful kinematics.
- **A fixed-wing autopilot that emits course/airspeed.** Rejected: the autopilot never sees the DAA
  override velocity (§2), so it cannot be where the projection lives.
- **A `Vehicle` class bundling `(autopilot, dynamics, perf)`.** Rejected again (ADR 0011 §7): no
  behaviour yet; threaded arguments suffice.

## Consequences

- **Good:** a fixed-wing can fly `MVP`/`VO` output; a mixed multirotor-vs-fixed-wing encounter runs
  through `run_encounter` and is sweepable for an IPR — closing the follow-up
  [[mixed-fleet-dubins-holonomic]] flagged. The resolvers, the airframes' fail-fast, and the
  multirotor IPR anchors are all untouched. The projection is unit-tested in isolation and the mixed
  encounter is pinned (deterministic + seeded + IPR).
- **The fixed-wing MVP/VO IPR is re-anchored** (deferred here by ADR 0013's Consequences): the
  both-fixed-wing 90° crossing pins MVP 53.34 m / VO 53.41 m — two slow-turning fixed-wings clear by
  a tighter margin than the mixed/multirotor case (91.7 m), the non-holonomic turn limit costing miss
  distance.
- **Cost:** `SeparationManager.step` grew an optional `adapter`; `run_encounter` grew four optional
  per-aircraft arguments; the loop gained a small airframe→adapter resolver.
- **Obligation / deferred:** wind (`ψ = χ` here; crab angle + NPFG → Phase 5); an out-of-envelope
  avoidance speed is clamped, not flagged.

## Relations

- Realises the deferred fixed-wing DAA of [[0013-fixedwing-coordinated-turn]] §4 and the per-aircraft
  threading of [[0011-motioncommand-and-guidance-separation]] §7; keeps `MVP`/`VO` neutral per
  [[0008-velocity-vector-command]] / [[0011-motioncommand-and-guidance-separation]] §2.
- Consumes the airframes of [[0012-multirotor-and-yaw-carrying-state]] /
  [[0013-fixedwing-coordinated-turn]] and the guidance of [[0014-mission-position-guidance]].
- Implements [[phase-4e-mixed-fleet-daa]]; the result is [[mixed-fleet-daa]], in the
  [[mixed-fleet-dubins-holonomic]] lineage. Demo: `scripts/mixed_fleet_daa_demo.py`.
