# ADR 0009 — HolonomicDynamics: the second `Dynamics` implementation

- Status: accepted
- Date: 2026-07-21
- Deciders: Fazlur Rahman

## Context

ADR 0007 built the `Dynamics` ABC with exactly one implementation (`PointMassDynamics`,
turn-rate-limited — a Dubins car). Its own "Obligation" section named the trigger for revisiting
open questions: *"when [a second implementation] lands, it becomes the concrete trigger to
revisit the subpackage question."* Holonomic motion — a vehicle whose ground-velocity vector can
move in any direction, not just the one its "heading" points — is that second implementation, and
it is also the case ADR 0008 (velocity-vector `Command`) was explicitly justified by: a real
multirotor has no coupled heading the way a fixed-wing does, so a model that doesn't invent one is
closer to what the M600 (or any multirotor) actually is.

## Decision

### 1. `HolonomicDynamics.step` chases the velocity vector directly, isotropically rate-limited

```python
tgt_e, tgt_n = _clip_magnitude(command.v_east, command.v_north, perf.v_max)   # envelope
cur_e, cur_n = velocity_enu(state)
step_e, step_n = _clip_magnitude(tgt_e - cur_e, tgt_n - cur_n, perf.ax * dt)  # accel limit
new_e, new_n = cur_e + step_e, cur_n + step_n
```

No `atan2`, no turn-rate step, no `turn_rate` state — ADR 0008's velocity `Command` is consumed
with zero polar reconstruction, which is exactly what that ADR set up. The envelope check
(`_clip_magnitude` to `v_max`) happens *before* the bounded step toward it, so the result never
leaves the `v_max` disk: the disk is convex, both the current vector and the clipped target sit
inside it, and the step moves along the straight line between them, which convexity guarantees
stays inside too. No extra clamp needed after the step.

### 2. `Performance` is reinterpreted, not extended

Only `v_max` (top speed) and `ax` (now an *isotropic* acceleration cap — equally hard to
accelerate in any direction, not decomposed into a turn-rate and a speed-ramp) are read.
`max_tr`/`max_dtr2` don't apply — there is no turn rate to limit. `v_min` doesn't apply either:
for a coupled-heading vehicle, `v_min < 0` was "backward flight" (facing one way, moving the
other); for a holonomic vehicle facing is already decoupled from travel, so "backward" is just
another direction the vector can already point — not a separate capability needing its own bound.
This mirrors ADR 0008 s4's decision to drop command-driven backward flight from the point-mass
model too: **neither implementation can be commanded backward now**, for the same underlying
reason (facing-vs-travel), reached independently at each model's own boundary.

### 3. No `AircraftState` changes

`trk`/`gs` are derived from the new vector each step (`atan2`/`hypot`) exactly as
`PointMassDynamics` produces them — same meaning ("direction and magnitude of ground travel"), so
CD/CR/CRR, and a Dubins-car aircraft sharing the same encounter, read either aircraft's state
identically; only *how* each vehicle reaches a given `(trk, gs)` differs. `turn_rate` is left at
its default and never read or written by this model — confirmed (as for ADR 0007) that no CD/CR/
CRR code reads it, so an aircraft using this `Dynamics` simply carries an inert field, same as
today's `PointMassDynamics` aircraft carry unused `desired`/`pos_ci95` until something sets them.
This is the "minimal" holonomic model discussed before building it: a jerk-limited (acceleration
itself rate-limited) version would need a persistent acceleration-vector field and is deferred —
no concrete use needs it yet.

### 4. Zero-vector rule matches `step_dynamics`

A zero command has no defined direction; both models hold the current track rather than snapping
toward the arbitrary `trk=0` a naive `atan2(0,0)` would produce (`_SPD_EPS` guard, shared
constant).

## Alternatives rejected

- **Split `dynamics.py` into a subpackage now** (`dynamics/base.py`, `dynamics/point_mass.py`,
  `dynamics/holonomic.py`), matching `cd/`/`cr/`/`crr/`. This is the exact trigger ADR 0007 named
  — deferred anyway. Two implementations in one ~230-line file is still legible; the split is
  mechanical whenever it's actually needed (an import-path change, not a design decision), and
  doing it now would touch `loop.py`, three test files, and a script for no behavioural gain.
  Revisit if a third implementation (wind) lands and the file gets unwieldy.
- **A jerk-limited (second-order) holonomic model**, with a persistent acceleration-vector state
  field. Rejected for now: more physically faithful, but nothing in the current scenarios needs
  it, and it's exactly the kind of speculative field `state.py`'s own docstring warns against
  ("a deliberate, re-validated change... not a set of dead fields now"). Add it, and its own ADR,
  when a use case actually needs the extra fidelity.

## Consequences

- **Good:** proves the `Dynamics` seam ADR 0007 built is real for a model that doesn't share the
  Dubins heading assumption — the thing ADR 0008's velocity `Command` was specifically justified
  by. `AircraftState` and `Command` needed zero changes. Full suite green (101 tests) with no
  changes to `PointMassDynamics`, `loop.py`, or any CD/CR/CRR code.
- **Cost:** a mixed-fleet encounter (one Dubins aircraft, one holonomic) is not yet wireable
  through `run_encounter` — it still takes one shared `dynamics=`/`perf=` for both sides. Making
  that per-aircraft was discussed but is out of scope here; tracked as a follow-up, not designed
  in this ADR.
- **Obligation:** if a jerk-limited variant or a wind-aware model lands next, revisit the
  subpackage question again — two implementations was the threshold for asking, not for acting.

## Relations

- Implements the second `Dynamics` [[0007-dynamics-as-pluggable-interface]] anticipated, and
  exercises the velocity-vector `Command` [[0008-velocity-vector-command]] was built to enable.
- The comparison trajectory (Dubins sweeping a rate-limited arc vs. holonomic cutting directly
  through velocity-space) is in `vault/observations/controlling-dubins-vs-holonomic.md`.
- The heterogeneous-dynamics (mixed-fleet) discussion this ADR's "Cost" section defers is recorded
  in conversation, not yet its own document — a candidate future ADR once `run_encounter` grows
  per-aircraft `dynamics`/`perf`.
