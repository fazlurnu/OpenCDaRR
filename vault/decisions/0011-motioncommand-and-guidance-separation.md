# ADR 0011 — MotionCommand supersedes Command; Mission / Autopilot / SeparationManager separation

- Status: proposed
- Date: 2026-07-22
- Deciders: Fazlur Rahman

## Context

`run_encounter` currently fuses three responsibilities that real flight stacks keep apart:

- **guidance** — "nominal" is a *static velocity vector* frozen once from the initial state
  (`nom_own = Command.from_track_speed(own.trk, own.gs)`), so the aircraft can only ever hold a
  cruise heading; it cannot navigate to a point or follow a route;
- **separation** — `loop._decide` *is* the separation manager (detect → resolve → recover), but it
  lives as a free function inside the loop with no name or home of its own;
- **physics** — a single `dynamics=` / `perf=` is shared by both aircraft, so a mixed-fleet
  encounter (different airframes per side) cannot run through the normal entry point.

Two forces make this the right moment to separate them. First, [[TODO-autopilot-separator-dynamic-integration]]
asks for three **vehicle classes** — multirotor, fixed-wing, VTOL — reached through one set of
interfaces; a fixed-wing autopilot naturally commands `(heading, speed)`, which the current control
type cannot express. Second, [[0008-velocity-vector-command]] deliberately made `Command` a *pure
velocity vector* so the resolvers emit velocity directly — correct for `MVP`/`VO`, but by design it
carries no heading/speed/position channel, which is exactly what a guidance layer for a
non-holonomic airframe needs. The velocity command was right for resolvers and **insufficient for
guidance**; that tension is what this ADR resolves.

This ADR records the Phase 4a layer split (`vault/phase-4-plan.md`) and the decisions that shape
every rung built on top of it. It does **not** add fixed-wing physics, altitude, or multi-aircraft
coordination — those are downstream rungs with their own ADRs.

## Decision

### 1. `MotionCommand` supersedes the velocity-vector `Command`

The control input every `Dynamics.step` consumes becomes a richer, vehicle-neutral value:

```python
@dataclass(frozen=True)
class MotionCommand:
    target_velocity: tuple[float, float] | None = None   # (v_east, v_north) [m/s]
    target_position: tuple[float, float] | None = None   # (lat, lon)
    target_heading:  float | None = None                 # [deg, aviation]
    target_speed:    float | None = None                 # [m/s]
    target_altitude: float | None = None                 # [m]      — defined, ignored (§4)
    target_vertical_speed: float | None = None           # [m/s]    — defined, ignored (§4)
```

The old pure-velocity command becomes **one populated field** (`target_velocity`). `from_track_speed`
stays as a classmethod and `gs` / `trk` stay as derived properties over `target_velocity`, so
existing call sites and tests read unchanged (the same ergonomics [[0008-velocity-vector-command]]
§1 kept). A `Dynamics` model reads whichever fields its vehicle understands and **asserts the ones it
requires are present** — an under-specified command for that airframe fails fast rather than moving
the aircraft on a guessed default.

This extends rather than reverses ADR 0008: the velocity vector is still the resolvers' native
output and still the holonomic model's native input. `MotionCommand` adds the heading/speed/position
channels a guidance layer and a non-holonomic airframe need, without taking the velocity channel
away.

### 2. `MVP` / `VO` are unchanged in substance — they wrap their output

The only edit to the CD/CR/CRR core is that the resolvers return
`MotionCommand(target_velocity=...)` instead of a bare velocity `Command`. Their governing equations,
their computed avoidance vector, and their tests' *behaviour* are untouched (the miss distance still
opens to ~`rpz`). CD/CR/CRR remain dynamics-agnostic, already proven end-to-end
([[mixed-fleet-dubins-holonomic]]).

### 3. Three layers, each with a home; `MotionCommand` is the single currency between them

```
Mission            (mission.py)      — what should happen (goto / flight_plan). Inert data.
   → Autopilot     (autopilot/)      — how to achieve it: Mission → nominal MotionCommand, vehicle-aware
   → SeparationManager (separation.py) — is it safe near traffic? nominal → final MotionCommand
   → Dynamics      (dynamics/)        — what is physically possible: final MotionCommand → next state
```

`Autopilot` is a new pluggable family (ABC in `autopilot/base.py`, one implementation per file),
mirroring `cd/` / `cr/` / `crr/` / `cns/` — a new guidance strategy or vehicle class adds a file, not
a fork of the loop ([[0007-dynamics-as-pluggable-interface]]'s pattern applied to guidance). It runs
at the existing 1 Hz decision cadence; the loop already decouples that from the `dt` integration, so
no new timing machinery is introduced.

**Per-vehicle setpoints stay internal.** The TODO sketches `FixedWingSetpoint` / `MultirotorSetpoint`.
These are **not** a second public type crossing layer boundaries — they are how each `Dynamics` model
*interprets* a `MotionCommand` internally (fixed-wing lowers `target_heading`/`target_speed` to its
own chase; holonomic lowers `target_velocity` to `v_east`/`v_north`). One currency across the
boundaries, per-vehicle interpretation inside the airframe — this keeps `Autopilot` and
`SeparationManager` vehicle-neutral where they can be.

### 4. 2D only — altitude fields are defined but ignored

`target_altitude` / `target_vertical_speed` exist on `MotionCommand` so the interface is complete and
a fixed-wing autopilot need not be reshaped later, but **every 2D dynamics model ignores them** and
`AircraftState` stays horizontal. Altitude, vertical rate, and the vertical detection/level math land
**with the fixed-wing paper**, under their own ADR — the exact deferral `state.py` and
[[0010-dynamics-subpackage-and-odometry-state]] §4 already commit to. No dead 3D fields on the state
now; the two command fields are the interface's forward declaration, not live state.

### 5. `SeparationManager` holds no mutable object state

`loop._decide` becomes `SeparationManager.step(state, perceived_traffic, nominal, memory, ...) ->
(MotionCommand, PairMemory)` — renamed and given a home, **substantively unchanged**. The TODO says
the manager "may maintain internal pair memory (active conflicts, resolution state, onset velocity,
recovery status)"; that is **rejected as written**. All such memory stays the existing `PairMemory`
value, threaded **in** and returned **out**, never stored on the manager object.

This is load-bearing, not stylistic: the interacting-particle system clones a particle by copying its
state, and any future-affecting value kept *outside* the state is silently shared between clones —
which is exactly the KI-1 recovery-state leak (`docs/lesson-learnt.md`), invisible at 1e-9. A
stateful `SeparationManager` would reintroduce precisely the hazard the clonable design exists to
prevent (`state.py`'s no-hidden-state invariant, [[0010-dynamics-subpackage-and-odometry-state]] §3).
The same rule binds every autopilot: any guidance progress it accumulates (e.g. the active-waypoint
index, Phase 4c) must be clonable value state, never an attribute on the autopilot object.

### 6. Pairwise (n=2) stays; the signature is future-proofed

`SeparationManager.step` accepts `perceived_traffic: list[AircraftState]` so the type does not have to
change when n>2 arrives, but the loop feeds it the single perceived other (`[other]` or `[]`) and no
multi-aircraft coordination logic is added. n>2 and the coordination model remain the separate
multi-aircraft / IPS roadmap ([[0004-layered-directed-design-for-multiaircraft-and-ips]]).

### 7. `(mission, autopilot, dynamics, perf)` are threaded **per aircraft**

`run_encounter` stops taking one shared `dynamics=` / `perf=`. Each aircraft carries its own bundle,
which is what lets a mixed multirotor-vs-fixed-wing encounter run through the same entry point the IPR
sweeps use — closing the follow-up [[mixed-fleet-dubins-holonomic]] flagged ("wiring `run_encounter`
itself for per-aircraft Dynamics/Performance"). No heavyweight `Vehicle` class is introduced; the
bundle is threaded as plain per-aircraft arguments until a real need for a named grouping appears
(the `state.py` "no speculative structure" rule).

## Alternatives rejected

- **Keep the velocity-vector `Command`, lower `MotionCommand` to it before Dynamics.** Rejected: a
  velocity vector cannot express a native fixed-wing `(heading, speed)` setpoint to the dynamics — the
  autopilot would have to pre-integrate a heading law into a velocity, duplicating what the airframe
  model already does under its turn-rate limit, and losing the airframe's ability to interpret the
  setpoint under its own constraints. The whole reason ADR 0008 relocated the polar reconstruction
  *into* the model was to let each airframe chase a target its own way; a fixed-wing target is a
  heading/speed, so that must survive to the model.
- **Make `SeparationManager` stateful, as the TODO literally sketches.** Rejected (see §5) — it
  violates the invariant IPS depends on and re-opens the KI-1 class of bug. Threading `PairMemory` as
  a value costs nothing and is already how `_decide` works today.
- **Two public types (`MotionCommand` + per-vehicle setpoints) across layer boundaries.** Rejected
  (see §3) — it forces `Autopilot` and `SeparationManager` to know each vehicle's setpoint shape,
  spreading vehicle-awareness into layers that can otherwise stay neutral. Per-vehicle interpretation
  belongs inside the airframe.
- **Go 3D now, so `MotionCommand` is fully live.** Rejected (see §4) — altitude has no consumer until
  the fixed-wing model exists; adding it now is the speculative-dead-field move `state.py` warns
  against. Defined-but-ignored command fields are cheap forward declarations; live state fields with
  no reader are not.
- **Introduce a `Vehicle` class bundling `(autopilot, dynamics, perf)`.** Rejected for now (see §7) —
  a struct with no behaviour yet; threaded arguments suffice until a real grouping need appears.

## Consequences

- **Good (intended):** guidance becomes real — an autopilot can navigate (goto → waypoints → loiter),
  built one rung at a time on top of an untouched CD/CR/CRR core. A fixed-wing autopilot can command
  `(heading, speed)` natively. A mixed-fleet encounter runs through the normal entry point and is
  sweepable for an IPR. The separation logic gains a name and a home without gaining hidden state.
- **Gate (not yet achieved — this ADR is `proposed`):** the 4a split must reproduce the MVP/VO IPR
  **bit-for-bit** against the [[0010-dynamics-subpackage-and-odometry-state]] anchors (MVP 0.9550,
  VO 0.2050) with CruiseAutopilot + SeparationManager standing in for today's frozen nominal +
  `_decide`. Behaviour-preserving means byte-identical; if a bit moves, 4a is not done. The Consequences
  here are promoted to "accepted / verified" once that gate is green — matching how ADR 0008/0010
  recorded their regression as an achieved fact, which this one cannot yet claim.
- **Cost:** `MotionCommand`'s optional fields mean a model must validate its required fields are
  present (a fail-fast assert per model) — the price of one command type serving several airframes.
- **Obligation:** altitude / 3D state, fixed-wing physics, and VTOL modes each land with their own
  ADR (§4); any autopilot that accumulates progress must keep it as clonable value state (§5).

## Relations

- Supersedes/extends [[0008-velocity-vector-command]] — keeps the velocity vector as one field, adds
  the heading/speed/position channels guidance needs; the resolvers' output is unchanged (§1–2).
- Applies [[0007-dynamics-as-pluggable-interface]]'s contribution-surface pattern to a new
  `Autopilot` family, and builds on the multirotor models + odometry obligation of
  [[0009-holonomic-dynamics]] / [[0010-dynamics-subpackage-and-odometry-state]].
- Preserves the directed, pairwise-now / n>2-later design of
  [[0004-layered-directed-design-for-multiaircraft-and-ips]] (§6).
- Closes the per-aircraft-dynamics follow-up in [[mixed-fleet-dubins-holonomic]] (§7).
- Implements the layer separation of [[TODO-autopilot-separator-dynamic-integration]]; the full rung
  plan is `vault/phase-4-plan.md`.
- The fixed-wing and VTOL ADRs this anticipates are not yet written — downstream of Phase 4d / 4e.
