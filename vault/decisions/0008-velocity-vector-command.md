# ADR 0008 — Command (and DesiredVelocity) as a velocity vector, not polar (trk, gs)

- Status: accepted
- Date: 2026-07-21
- Deciders: Fazlur Rahman

## Context

`Command` — the control target every resolver emits and `step_dynamics` consumes — was polar:
`Command(hdg, spd)`. `DesiredVelocity` — an aircraft's nominal intent, read by the intent-based
recovery criteria — was the same shape, `DesiredVelocity(trk, gs)`.

Polar looks natural (it reads like an ATC instruction), but it quietly bakes an airframe
assumption into the *shared* control interface: that the vehicle has a **heading distinct from its
direction of travel**. That is true for a Dubins car / fixed wing and **false for a holonomic
multirotor**, which can move its velocity vector without a coupled heading. As the design opens up
to multiple `Dynamics` models (ADR 0007 — wind, holonomic, other airframes), the interface both
models must speak should assume the *least* about the physics.

Two observations made the case concrete:

- **The resolvers already compute in velocity space.** `MVP` and `VO` both build a Cartesian
  velocity vector (`new_vx, new_vy` / `new_e, new_n`) and then spend an `atan2`/`hypot` purely to
  fit the polar `Command`. `step_dynamics`, conversely, consumes polar and needs a heading to
  apply its turn-rate limit. So the conversion count across the CR→Dynamics boundary is
  **identical** either way (1 total) — a velocity `Command` doesn't remove the conversion, it
  **relocates** it from the shared type into the one model that actually assumes a heading.
- **`DesiredVelocity` reads back out as a vector too.** `FTR` / `ProbabilisticFTR` immediately
  convert `own.desired.trk/gs` into `(vo_e, vo_n)` via sin/cos. Storing the vector lets them read
  the components directly, with no trig at their edge.

## Decision

### 1. `Command` and `DesiredVelocity` store `(v_east, v_north)` [m/s]

Both become East–North velocity vectors — the same convention as `kinematics.velocity_enu` and
`Relative.vx/vy`. Each keeps ergonomics via:

- a `from_track_speed(trk|hdg, gs|spd)` classmethod (aviation heading + speed → vector), so call
  sites and tests still read in aviation terms;
- `trk` and `gs` **derived properties** (`atan2` / `hypot`), so consumers that want polar still get
  it.

Numerics check: `from_track_speed` round-trips cardinal headings (0/90/180/270/45) to `gs`/`trk`
**exactly**, so the analytical dynamics gate's exact-equality assertions (`trk == 90.0`,
`turn_rate == 0.0`) still hold; off-axis headings differ by ~1 ULP, which no test asserts on.

### 2. The polar reconstruction lives inside the point-mass model, not the interface

`step_dynamics` reads `command.gs` as the target speed and `command.trk` as the target track, then
turn-rate-limits toward it exactly as before. This is the whole point: the airframe that assumes a
heading is the one that reconstructs it. A future holonomic `Dynamics` may drive `v_east/v_north`
directly and never form a heading.

### 3. Zero-vector rule: hold current heading

A zero velocity vector has no defined direction (`trk` returns 0). `step_dynamics` special-cases
`|v_cmd| < 1e-9` → hold the current track (don't turn toward the arbitrary north). Defensive today
(no live path emits a zero command), but it removes a latent "spin to north on stop" bug the polar
form implicitly avoided.

### 4. Backward flight via command is deliberately dropped

The old *signed* `gs` let a command mean "face this way, move backward" (`spd < 0`, clamped to
`v_min = -18`). A velocity vector cannot encode facing decoupled from travel — "backward at 18" is
just a velocity pointing backward. Nothing in the CR layer ever emitted a negative speed (MVP/VO
magnitudes are `hypot ≥ 0`); only a synthetic unit test exercised it. So this capability is
**intentionally narrowed away** from the velocity command. If facing-decoupled-from-travel is ever
wanted (it is the same yaw-vs-track split holonomic motion raises), it belongs in a future
yaw-carrying `AircraftState` field, decided on its own — not smuggled back through signed speed.
`AircraftState.gs` itself stays a signed scalar (unchanged); only the *command* loses the sign.

## Alternatives rejected

- **Keep polar `Command`.** Rejected: it is not wrong, but it bakes the "heading exists" airframe
  assumption into the shared interface, which is exactly what breaks for holonomic motion. The
  conversion count is a wash, so there is no efficiency reason to keep it either.
- **Change `Command` but leave `DesiredVelocity` polar.** Rejected: they are the same shape (a 2D
  velocity), sit side by side, and `DesiredVelocity` is *also* consumed as a vector (FTR). Leaving
  one polar just trades one inconsistency for another.
- **Do it after the holonomic model lands.** Rejected — the opposite of ADR 0007's "defer the
  subpackage split." That split touches nothing until a second implementation exists; this touches
  a *shared type with existing consumers* (MVP, VO, `step_dynamics`, the loop's nominal/coast
  commands, `DesiredVelocity`, FTR, and their tests). The migration cost only grows once holonomic
  adds a third consumer, and doing it now is what makes the holonomic model clean to write rather
  than retrofitted.
- **Preserve command-driven backward flight.** Rejected: unrepresentable in a velocity vector
  without re-introducing a facing channel, and unused by any real path (see §4).

## Consequences

- **Good:** the control interface is now airframe-neutral — a `Dynamics` model decides how to
  chase a target velocity, and a holonomic one need not invent a heading. `MVP`/`VO` return their
  computed vector directly (polar round-trip gone); `FTR`/`ProbabilisticFTR` read `desired.v_east/
  v_north` directly (sin/cos gone; `math` dropped from `ftr.py`). End-to-end behaviour is
  unchanged: the full test suite (incl. the BlueSky equivalence anchor) passes untouched except
  the one backward-flight test, and the MVP-vs-VO IPR sweep at 2° reproduces bit-for-bit
  (MVP 0.9550, VO 0.2050, identical LoS counts and median CPA) — a whole-loop regression check.
- **Cost:** one behavioural narrowing (no command-driven backward flight, §4), recorded here so it
  is a decision, not a silent regression. `test_speed_ramps_down_to_v_min` is replaced by
  `test_reversed_command_turns_around_not_backward`, which pins the new semantics (a reversed
  command turns the aircraft around at forward speed).
- **Obligation:** a future `Dynamics` that genuinely needs a facing separate from velocity
  direction (holonomic yaw, or backward flight) must add that as an `AircraftState` field with its
  own ADR — it must not come back as a signed-speed convention on `Command`.

## Relations

- Builds on [[0007-dynamics-as-pluggable-interface]] — this makes the command that flows into
  every `Dynamics` airframe-neutral, so the pluggability ADR 0007 established is real for models
  that do not share the Dubins heading assumption.
- Touches the resolvers ([[../derivations/mvp-resolution]], VO) and the intent-based recovery
  criteria ([[../derivations/ftr-recovery]], [[../derivations/probabilistic-ftr-recovery]]) at
  their type boundary only; their governing equations are unchanged.
- The holonomic / yaw-carrying-state extension this anticipates is not yet designed — a future ADR.
