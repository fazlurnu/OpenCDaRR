# ADR 0010 — Dynamics subpackage + rename + odometry state (and what stays out)

- Status: accepted
- Date: 2026-07-21
- Deciders: Fazlur Rahman

## Context

With two `Dynamics` implementations now landed ([[0007-dynamics-as-pluggable-interface]],
[[0009-holonomic-dynamics]]), three deferred questions came due at once, plus a request to make
the aircraft state carry more per-aircraft telemetry. This ADR settles all of it in one pass so
the boundaries are clean before multi-aircraft / IPS work builds on them.

## Decision

### 1. `dynamics.py` becomes a `dynamics/` subpackage

Mirroring `cd/`, `cr/`, `crr/`, `cns/` — one implementation per file:

```
dynamics/
  __init__.py   # re-exports the public surface (below)
  base.py       # Command, Dynamics (ABC), _clip, _SPD_EPS, odometry_update
  dubins.py     # step_dynamics (the integrator) + DubinsDynamics
  holonomic.py  # HolonomicDynamics + _clip_magnitude
```

`__init__.py` re-exports `Command`, `Dynamics`, `DubinsDynamics`, `HolonomicDynamics`,
`step_dynamics`, so every existing `from opencdarr.dynamics import ...` is unchanged — the split
is transparent to consumers. ADR 0007 and 0009 both explicitly named "when a second implementation
lands" as the trigger to reconsider this and (both times) deferred it as "mechanical, an
import-path change, not a design decision." Two implementations plus an explicit request is that
trigger; the move is exactly as mechanical as promised.

### 2. `PointMassDynamics` → `DubinsDynamics`

`PointMassDynamics` was a weak name: `HolonomicDynamics` is *also* a point mass, so "point mass"
didn't name the distinguishing property — a **heading coupled to direction of travel**, which is
what "Dubins" conveys to the robotics/UTM audience. The name is used loosely and the docstrings say
so: this is not the textbook Dubins car (constant speed, fixed minimum-radius arc); it has a
variable, acceleration-limited speed and a limited turn *rate*/turn-*acceleration*, not a fixed
radius. The precise-but-clunky alternative (`NonholonomicDynamics`) was considered and rejected in
favour of the communicative term, with the caveat documented rather than encoded in a longer name.
The free integrator keeps its name `step_dynamics` (widely referenced; it lives in `dubins.py`).

### 3. Two odometry accumulators on `AircraftState`

`flight_time` (seconds advanced) and `distance_flown` (ground path length, metres) — both default
`0.0`, both advanced by every `Dynamics.step` via a single shared helper `base.odometry_update`,
so no implementation (present or future) can advance them inconsistently or forget them. They are
**diagnostics, not dynamics inputs** — nothing reads them back to decide the next step — but they
live *in* `AircraftState`, not in the loop, for one reason: an IPS clone taken mid-flight must
inherit its parent's elapsed time and path length, and the only thing that clones with the particle
is its state (the no-hidden-state invariant, `state.py`). `distance_flown` is an *odometer*
(`Σ gs·dt`), so a there-and-back path keeps growing while net displacement returns toward the
start. This is a deliberate, small widening of `AircraftState`'s scope from "certain kinematic
core" to "kinematic core + odometry accumulators," recorded here rather than done silently.

### 4. Deliberately **not** added: velocity components, and a separate heading

Both were proposed; both are declined, for reasons that are the same principle `state.py` already
documents ("not a set of dead fields now"):

- **`v_east` / `v_north` as stored fields** — rejected as *redundant*. They are exactly `(trk, gs)`
  in Cartesian form, already available via `kinematics.velocity_enu`. Storing them too creates a
  second source of truth for one fact, which can drift out of sync — precisely the
  hidden/duplicated-state hazard the clonable design exists to prevent. If ergonomic access is
  wanted later, they can be *derived properties* (like `Command.gs`/`.trk`), never fields.
- **A `heading` distinct from `trk`** — rejected as a *dead field today*. `heading ≠ track` means
  something only under wind (crab angle) or a holonomic vehicle that yaws independently of travel —
  and neither is modelled (no wind; `HolonomicDynamics` faces its travel direction; `Command`
  carries no yaw). A `heading` field now would always equal `trk`, be written by every model, and
  be read by nothing. It is the *right* field to add **with** the wind / independent-yaw model that
  gives it meaning and its own ADR — exactly the deferral [[0008-velocity-vector-command]] §4 and
  [[0009-holonomic-dynamics]] already committed to for "facing decoupled from travel."

## Alternatives rejected

- **Keep a flat `dynamics.py`.** Rejected now (see §1) — two implementations is the agreed
  threshold, and the file was heading past ~230 lines with two distinct models plus shared helpers.
- **`NonholonomicDynamics` instead of `DubinsDynamics`.** Rejected (see §2) — precise but clunky;
  the communicative name plus an honest docstring is the better trade for this audience.
- **Compute `flight_time` / `distance_flown` in the loop, not the state.** Rejected (see §3) — they
  would then be unavailable after a bare `.step()` and, worse, would not clone with an IPS particle
  unless the loop threaded them separately, reintroducing exactly the kind of out-of-state
  per-particle quantity the design forbids.
- **Add velocity components / heading as fields "while we're here".** Rejected (see §4) — redundant
  and dead-now respectively; adding speculative fields is the specific thing `state.py` warns
  against.

## Consequences

- **Good:** dynamics now matches every other model family's on-disk shape; the rename names the
  real distinction; odometry rides with the particle for free and is guaranteed consistent across
  implementations by the shared helper. Behaviour unchanged: full suite green (105 tests, incl. the
  BlueSky equivalence anchor), and the MVP-vs-VO IPR sweep reproduces bit-for-bit (MVP 0.9550, VO
  0.2050) — the odometry fields never feed back into dynamics, so outcomes can't move.
- **Cost:** one more shared helper (`odometry_update`) that a new `Dynamics` implementation must
  remember to call — mitigated by it being a one-liner splatted into `replace`, and named in the
  `Dynamics` ABC docstring as an obligation.
- **Obligation:** the wind / independent-yaw model, when it lands, is where a real `heading` field
  (and possibly stored velocity, if a model needs it as primary state) gets added — with its own
  ADR, not retrofitted here.

## Relations

- Restructures what [[0007-dynamics-as-pluggable-interface]] created and
  [[0009-holonomic-dynamics]] extended; both named this as a follow-up.
- The velocity-vector `Command` that makes `HolonomicDynamics` clean is
  [[0008-velocity-vector-command]]; its §4 backward-flight deferral is the same reasoning as §4
  here for `heading`.
- `vault/observations/controlling-dubins-vs-holonomic.md` uses the renamed classes.
