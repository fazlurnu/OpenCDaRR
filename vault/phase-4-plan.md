# Phase 4 plan — Mission / Autopilot / SeparationManager separation + vehicle classes

Implements [[TODO-autopilot-separator-dynamic-integration]]. The realization behind it: today the
loop conflates three responsibilities that real flight systems keep apart —

```
        Mission            = What should happen
           |
           v
     Autopilot / Guidance  = How should we achieve it
           |
           v
   Nominal MotionCommand
           |
           v
     SeparationManager     = Is this command safe near traffic? Override if not.
           |
           v
    Final MotionCommand
           |
           v
       Dynamics            = What is physically possible (per airframe)
           |
           v
     AircraftState update
```

Right now `run_encounter` fuses all of it: "nominal" is a *static velocity vector* frozen from the
initial state (`nom_own = Command.from_track_speed(own.trk, own.gs)`), `_decide` is the separation manager, and the airframe is a single `dynamics=` shared by both aircraft. Phase 4 splits these into named layers so that (a) an autopilot can actually *navigate* (goto, waypoints, loiter) rather than hold a frozen heading, (b) the separation logic is a clean overlay on that navigation, and (c) each aircraft can be a different **vehicle class** — multirotor, fixed-wing, VTOL — through the same interfaces.

Same working style as Phases 2–3: one file at a time, read each diff, tick the box here. Build the
autopilot capability ladder **one rung at a time**, each rung green before the next.

---

## Vehicle classes (the three the TODO names)

| Class | Dynamics (exists?) | Natural MotionCommand | Priority |
|---|---|---|---|
| **multirotor** | `HolonomicDynamics` ✅ (approx. holonomic) | `target_position` / `target_velocity` | first — refactor validated here |
| **fixed-wing** | *new, from a paper you'll provide* | `target_heading` + `target_speed` (+ `target_altitude` when 3D lands) | second |
| **VTOL** | mode-switching wrapper over the two above | mode-dependent | **last** |

`DubinsDynamics` stays as the turn-rate-limited multirotor variant already validated against
BlueSky ([[0005-trajectory-validated-against-bluesky]]) — it is *a* multirotor model, not fixed-wing
(a coupled heading and a turn-*rate* limit are not the same as a non-holonomic minimum-radius
airframe with a stall speed). The genuinely new dynamics is fixed-wing; VTOL is composition, not new physics.

---

## Scope of this pass, and the exit gate

**Functional + behaviour-preserving refactor first, then capability.** Phase 4 does not change any
physics of the *existing* multirotor path; it re-homes responsibilities and then adds navigation and
vehicle classes on top of the untouched CD/CR/CRR core.

**Exit gate:**
1. The layer split (4a) reproduces today's IPR **bit-for-bit** — CruiseAutopilot + SeparationManager through the new `run_encounter` gives byte-identical outcomes to the current loop (a free regression, same discipline as Phase 3's zero-noise reproduction).
2. Each new autopilot rung (goto → waypoint → loiter) has a functional test that *bites*, and runs
   for the multirotor **and** through the fixed-wing autopilot once that lands.
3. The no-hidden-state invariant is preserved: all CDR/recovery memory and all autopilot progress (e.g. active-waypoint index) live in **clonable value state**, nothing on a manager object (verified the same way Phase 3 verified comm state is clonable).

---

## Settled up front (the pushbacks — decided before building, like cd/cr/crr)

These are firm; they shape every checklist item below.

1. **2D only.** `target_altitude` / `target_vertical_speed` are *defined on `MotionCommand`* (the
   interface is complete) but **ignored** by every 2D dynamics model this pass. `AircraftState`
   stays horizontal. Altitude, vertical rate, and vertical detection/level math land **with the
   fixed-wing paper**, under their own ADR — exactly the deferral `state.py` and
   [[0010-dynamics-subpackage-and-odometry-state]] §4 already commit to. No dead 3D fields on the state now.

2. **`MotionCommand` supersedes the velocity-vector `Command`** (new ADR, extends
   [[0008-velocity-vector-command]]). The old pure-velocity command becomes **one populated field** (`target_velocity`); `MVP`/`VO` keep emitting a velocity vector and are **unchanged** — they now return a `MotionCommand` with only `target_velocity` set. A dynamics model reads whichever fields
   its vehicle understands and ignores the rest. This is why ADR 0008's pure-velocity command was
   *right for resolvers but insufficient for a fixed-wing autopilot* — a fixed-wing naturally
   commands `(heading, speed)`, not a ground-velocity vector.

3. **`MotionCommand` is the single currency between layers; per-vehicle setpoints stay internal.**
   The TODO sketches `FixedWingSetpoint` / `MultirotorSetpoint`. These are **not** a second public
   type crossing layer boundaries — they are how each `Dynamics` model *interprets* a `MotionCommand`
   internally (e.g. fixed-wing dynamics lowers `target_heading`/`target_speed` to its own state
   chase; holonomic lowers `target_velocity` to `v_east`/`v_north`). One currency, per-vehicle
   interpretation — this keeps Autopilot and SeparationManager vehicle-neutral where they can be.

4. **SeparationManager holds no mutable object state.** The TODO says it "may maintain internal pair
   memory (active conflicts, resolution state, onset velocity, recovery status)." That is **rejected**
   as written: it violates the invariant IPS depends on (`state.py`, [[0010-dynamics-subpackage-and-odometry-state]]).
   The existing `PairMemory` value stays the carrier of that memory — passed **into** `.step` and
   returned **out**, never stored on the manager. `SeparationManager.step` is `_decide` renamed and
   given a home, substantively unchanged. (This is the KI-1 lesson: a recovery-state leak outside the
   clonable state corrupts the estimate invisibly at 1e-9.)

5. **Pairwise (n=2) stays.** The TODO's `perceived_traffic: list[AircraftState]` multi-aircraft shape
   is real, but n>2 + the coordination model is the existing multi-aircraft/IPS roadmap
   ([[0004-layered-directed-design-for-multiaircraft-and-ips]]), not this refactor. `SeparationManager.step`
   *accepts* a list so the signature is future-proof, but the loop feeds it the single perceived
   other. No multi-aircraft coordination logic this pass.

6. **CD / CR / CRR are untouched.** Already dynamics-agnostic and proven so end-to-end
   ([[mixed-fleet-dubins-holonomic]]). Phase 4 does not open those files except to have `MVP`/`VO`
   return a `MotionCommand` (see decision 2).

7. **Per-aircraft `(autopilot, dynamics, perf)`.** `run_encounter` currently takes one shared
   `dynamics=` / `perf=`. Phase 4 threads them **per aircraft**, closing the follow-up flagged in
   [[mixed-fleet-dubins-holonomic]] "What this still doesn't cover" — a mixed multirotor-vs-fixed-wing
   encounter must run through the same entry point as the IPR sweeps. A light per-aircraft bundle
   carries `(mission, autopilot, dynamics, perf)`; no heavyweight `Vehicle` class yet (added only if
   a real need appears — the `state.py` "no speculative structure" rule).

---

## Phasing (each rung green before the next)

### 4a — `MotionCommand` + the layer split (behaviour-preserving)

The whole point of 4a is that **nothing observable changes** — it is pure re-homing, gated by a
bit-for-bit regression. Build it, prove the IPR is identical, then start adding capability.

- [ ] **`opencdarr/dynamics/base.py`** — replace `Command` with `MotionCommand`.
  - *Purpose:* the vehicle-neutral motion command:
    ```python
    @dataclass(frozen=True)
    class MotionCommand:
        target_velocity: tuple[float, float] | None = None   # (v_east, v_north) [m/s]
        target_position: tuple[float, float] | None = None   # (lat, lon) or local — TBD in 4b
        target_heading:  float | None = None                 # [deg, aviation]
        target_speed:    float | None = None                 # [m/s]
        target_altitude: float | None = None                 # [m] — defined, ignored (2D)
        target_vertical_speed: float | None = None           # [m/s] — defined, ignored (2D)
    ```
  - *Design:* supersedes [[0008-velocity-vector-command]] (decision 2). Keep the
    `from_track_speed` / `gs` / `trk` ergonomics as constructors/derived helpers over
    `target_velocity` so existing call sites read the same. A model asserts the fields it needs are
    present (fail fast on an under-specified command for that vehicle).
  - *Check:* `MotionCommand.from_track_speed(...).target_velocity` round-trips; `gs`/`trk` derived
    from `target_velocity` match the old `Command`.
  - *Relations:* consumed by every `Dynamics.step`; produced by autopilots, `MVP`/`VO`, and
    `SeparationManager`.

- [ ] **`MVP` / `VO` return `MotionCommand`** (`cr/mvp.py`, `cr/vo.py`) — the *only* edit to the CDR
  core: their computed avoidance velocity is wrapped as `MotionCommand(target_velocity=...)`.
  Behaviour identical; signatures now speak the new currency.
  - *Check:* existing `test_cr.py` passes unchanged in effect (miss distance still opens to ~`rpz`).

- [ ] **`opencdarr/autopilot/base.py`** (new package — mirrors `cd/`, `cr/`, `crr/`, `cns/`)
  - *Purpose:* `Autopilot` ABC — `step(state, mission, perf) -> MotionCommand`. Converts a mission
    objective into an immediate motion target, at the **decision cadence** (1 Hz), vehicle-aware.
  - *Design:* the contribution surface for guidance — a new guidance strategy or vehicle class adds
    a file. Runs in the same broadcast tick as the CDR decision (no new timing machinery; the loop
    already decouples the 1 Hz decision cadence from the `dt` integration).
  - *Relations:* [[0007-dynamics-as-pluggable-interface]]'s pattern, applied to guidance.

- [ ] **`opencdarr/autopilot/cruise.py`** — `CruiseAutopilot(target_heading, target_speed)`.
  - *Purpose:* return a constant `MotionCommand(target_velocity=from(heading,speed))` regardless of
    state — the behaviour-preserving stand-in for today's frozen nominal.
  - *Design (load-bearing for the regression):* the cruise target is a **mission parameter**, *not*
    derived from the live (noisy) self-fix each tick — that is exactly what makes 4a bit-identical to
    today, where `nom_own` is frozen from the *true* initial state, never the noisy measurement.
  - *Check:* returns the same command every tick; independent of the (noisy) state passed in.

- [ ] **`opencdarr/separation.py`** (new — `SeparationManager`)
  - *Purpose:* `step(state, perceived_traffic, nominal, memory, rpz, t_lookahead, detector,
    resolver, recovery, dt) -> (MotionCommand, PairMemory)`. This is `loop._decide` given a home and
    a name — detect → resolve → recover, releasing to `nominal` on recovery (decision 4).
  - *Design:* **stateless object** — `PairMemory` is threaded in/out, nothing stored on `self`
    (decision 4). `perceived_traffic` is a list for n>2 future-proofing; the loop passes `[other]`
    or `[]` (decision 5). Returns the nominal unchanged when no resolver / nothing perceived.
  - *Check:* byte-identical decisions to today's `_decide` on the same inputs (a direct port test).
  - *Relations:* consumes the autopilot's nominal, feeds `Dynamics`; the `_decide` docstring's
    control-flow doc moves here.

- [ ] **`opencdarr/loop.py`** — rewire `run_encounter` to the layered flow, per aircraft.
  - *Purpose:* at each broadcast tick, per aircraft: `nominal = autopilot.step(self_fix, mission,
    perf)` → `final, memory = separation.step(self_fix, [perceived_other], nominal, memory, ...)`;
    hold `final`; integrate `dynamics.step(state, final, perf, dt)` each `dt`.
  - *Design:* thread `(mission, autopilot, dynamics, perf)` **per aircraft** (decision 7). Keep the
    self-fix / broadcast / communication plumbing exactly as-is — only the "what produces the
    command" changes. CruiseAutopilot + SeparationManager == today.
  - *Check (the 4a gate):* the full suite stays green and the **MVP/VO IPR sweep reproduces
    bit-for-bit** (MVP 0.9550, VO 0.2050 — the same anchors ADR 0010 used). If a single bit moves,
    4a is not done.
  - *Relations:* the environment layer; still the pairwise precursor to `advance`/`is_terminal`.

- [ ] **ADR 0011 — `MotionCommand` + the Mission/Autopilot/SeparationManager separation.** Records
  decisions 1–7; supersedes/extends [[0008-velocity-vector-command]]. Why the pure-velocity command
  was right for resolvers but insufficient for guidance; why the separation manager stays a stateless
  object over clonable `PairMemory`.

### 4b — Mission + `goto` autopilot (multirotor first)

- [ ] **`opencdarr/mission.py`** (new) — `Mission` value: `goto: tuple | None`, `flight_plan:
  list[Waypoint] | None`. Start with `goto` only; `Waypoint` + `flight_plan` land in 4c.
  - *Design:* a plain frozen value (intent, "what should happen"); it does **not** control the
    aircraft. Decide the position frame here (WGS84 lat/lon vs a local ENU origin) — recommend
    lat/lon to match `AircraftState`, converting via `geo`/`kinematics` inside the autopilot.
  - *Relations:* consumed only by the autopilot.

- [ ] **`opencdarr/autopilot/goto.py`** — `GotoAutopilot` for the holonomic multirotor.
  - *Purpose:* steer toward `mission.goto`: bearing/range to target → `MotionCommand(target_velocity
    = unit(bearing) * cruise_speed)`, optionally slowing near the point (holonomic can stop/hover —
    the TODO's "can slow down, can stop and hover").
  - *Check:* from an offset start with no traffic, the multirotor reaches the goto point and
    settles; path is direct (holonomic), min-approach within tolerance.
  - *Relations:* pairs with `HolonomicDynamics`.

### 4c — Waypoint sequencing + loiter

- [ ] **`Waypoint` + `flight_plan` in `mission.py`**, and **active-waypoint progress as clonable
  state.** The current-leg index must live in the aircraft's clonable state (or the threaded
  autopilot memory value), **never** on the autopilot object — same invariant as `PairMemory`
  (decision 4). This is the one place 4c can quietly reintroduce hidden state; the plan calls it out
  so it doesn't.
  - *Design:* the autopilot advances to the next waypoint on an arrival test (within a capture
    radius) and emits the leg's `MotionCommand`; the index update is returned as value state.
  - *Check:* a 3-waypoint plan is flown in order; the leg index clones correctly (an IPS-style clone
    mid-plan continues from the same leg).

- [ ] **Loiter after arrival.** On reaching the final waypoint (or a `loiter` waypoint), the
  autopilot emits a loiter pattern — for the holonomic multirotor, hover (`target_velocity = 0`);
  for fixed-wing (4d), a turning orbit at min-radius (it cannot stop — the TODO's "may enter loiter
  after arrival").
  - *Check:* multirotor holds position at the goal; (fixed-wing loiter checked in 4d).

### 4d — Fixed-wing dynamics (from the paper — when you're ready)

*Blocked on the paper you'll provide.* Everything above is validated on the multirotor first so this
rung is purely "add one `Dynamics` + one `Autopilot` file," not a refactor.

- [ ] **`opencdarr/dynamics/fixedwing.py`** — `FixedWingDynamics(Dynamics)`, re-derived from the
  paper (not ported — `lesson-learnt.md`). Non-holonomic: min/max airspeed (stall), turn
  rate/radius, acceleration limits; consumes `MotionCommand(target_heading, target_speed)` and chases
  them under those constraints. Advances odometry via `odometry_update` (obligation from
  [[0010-dynamics-subpackage-and-odometry-state]]). **Stays 2D / level** this pass (altitude
  deferred, decision 1); the `Performance` envelope grows a stall-speed / min-radius as needed (its
  own new airframe instance, not an edit to a step function).
  - *Check:* analytical validation against the paper's closed-form turn/climb responses (the
    [[0002-analytical-validation-of-dynamics]] discipline); a fixed-wing cannot stop, cannot move
    sideways, follows a curved feasible path to a target — contrast documented like
    [[controlling-dubins-vs-holonomic]].
  - **Wind-ready by construction (obligation for [[phase-5-plan|Phase 5]]).** Build the integrator in
    the **air-relative coordinated-turn form** the same paper states — heading `ψ` integrated from
    bank, `ψ̇ = g·tan(ϕ)/V_TAS`, and the position update written as the vector sum
    `ẋ = V_TAS·sin ψ + w_x`, `ẏ = V_TAS·cos ψ + w_y` — **with the wind term present but fixed at
    `(w_x, w_y) = 0` this pass.** The no-wind fixed-wing is then the `w=0` special case of the wind
    model, and Phase 5 turns wind on by feeding a non-zero wind vector and adding the `heading` field,
    **not** by re-deriving the step. Concretely, this pass already commits to:
    - integrating **heading `ψ`** (not ground track `χ`) as the primary angular state — the
      `heading ≠ track` field [[0010-dynamics-subpackage-and-odometry-state]] §4 deferred "with the
      wind / independent-yaw model"; at `w=0` it equals `trk`, so it stays behaviour-neutral now but
      is the field wind gives meaning to (crab angle `θ_w = ψ − χ`);
    - treating `perf.v_max`/stall as an **airspeed** envelope (`V_TAS`), and deriving ground speed
      `V_GS = |airspeed vector + wind|` as an **output** each step — so that when wind is non-zero the
      "speed I fly" and the "speed I make good over the ground" are already distinct quantities, not a
      single stored `gs` that would have to be split later.
    - *Check (wind-readiness, `w=0`):* with the wind vector zero, the air-relative integrator
      reproduces the coupled-heading Dubins path **bit-for-bit** — i.e. `ψ == trk` and
      `V_GS == V_TAS` every step. This is the regression that proves the wind hook is inert until
      Phase 5 lights it up.

- [ ] **Command feasibility — the three infeasibility categories** (how `FixedWingDynamics`, and
  every airframe, reconciles a `MotionCommand` it cannot obey literally). A command is a *setpoint*,
  never a state to snap to; the model projects it onto its feasible set. Three distinct failures,
  three distinct handlings — pin them in the fixed-wing model and in `test_fixedwing_dynamics.py`:

  | Infeasibility | Example (fixed-wing) | Handling |
  |---|---|---|
  | **Missing required channel** | no direction at all (`target_velocity`, `target_heading` both `None`) | **fail fast** — an under-specified command for this vehicle is a programming error (ADR 0011 §1) |
  | **Out-of-range value** | `target_speed = 2` m/s (< stall); a 179° heading step | **clamp / converge** — clamp speed into `[v_stall, v_max]`, turn-rate-limit heading toward the target; feasible *over time* |
  | **Absent degree of freedom** | `target_yaw = 45°` while travelling east (nose decoupled from track); `target_altitude` in 2D | **ignore** — the airframe has no actuator for it (its heading is a derived *output*, = track + crab), so the field is a no-op; *optionally* **fail-fast on inconsistency** (a yaw that disagrees with track by more than tolerance almost certainly means a multirotor stack was wired to a fixed-wing) |

  The distinction that matters: an out-of-range value is reachable eventually (clamp + turn *toward*
  it), so the model converges; an absent DOF has nothing to converge *on* (no control channel), so
  the model drops it. Silently obeying either — flying at 2 m/s, or holding a 45° sideslip — is the
  bug this taxonomy exists to prevent. Altitude (2D, ADR 0011 §4) and decoupled yaw (4e) are the two
  absent-DOF fields today; they are handled identically (defined on the command, ignored by the
  airframe that lacks the dimension).
  - *Relations:* extends ADR 0011 §1 (missing-channel fail-fast) and §4 (defined-but-ignored
    fields); the yaw row is the multirotor/fixed-wing split that 4e's yaw-carrying state introduces.

- [ ] **`opencdarr/autopilot/fixedwing.py`** — the fixed-wing guidance: same `Mission` (goto,
  waypoints, loiter from 4b/4c) → `MotionCommand(target_heading, target_speed)` (an L1/pure-pursuit
  style steering law, TBD with the paper). The autopilot ladder from 4b/4c must produce *fixed-wing*
  commands here, per your instruction that guidance applies to fixed-wing too — same mission, a
  different `Autopilot` implementation and a different natural `MotionCommand`.
  - *Check:* fixed-wing flies the same 3-waypoint plan as the multirotor did in 4c, on a feasible
    curved path; loiter becomes a min-radius orbit.

- [ ] **Mixed-fleet through `run_encounter`.** A multirotor-vs-fixed-wing encounter runs through the
  same entry point the IPR sweeps use (now that dynamics/perf/autopilot are per-aircraft, 4a decision
  7) — closing [[mixed-fleet-dubins-holonomic]]'s remaining follow-up, this time with genuinely
  different physics on each side, and sweepable for an IPR.
  - *Check:* the mixed encounter resolves (min-sep ≥ rpz) and produces a reproducible IPR.

- [ ] **ADR 001x — fixed-wing dynamics** (and, if the paper forces altitude, a *separate* 3D ADR for
  `AircraftState` + vertical detection/level — decision 1's escape hatch, taken deliberately, not
  slipped in).

### 4e — Yaw-carrying state (multirotor independent yaw)

The prerequisite for the multirotor's still-unbuilt capability the TODO names — "rotate independently
from velocity direction" — surfaced by the fixed-wing-vs-yaw discussion: a command like
`target_velocity=(2,0), target_yaw=45°` is legal for a multirotor (translate east, nose pointing NE)
and structurally *inexpressible* on a fixed-wing (§ command-feasibility table in 4d — an absent DOF,
not a clamp). Today there is nowhere to put it: no `yaw` on `AircraftState`, no `target_yaw` on
`MotionCommand` — deliberately deferred by [[0008-velocity-vector-command]] §4 ("a future
yaw-carrying state, decided on its own — not smuggled back through signed speed") and
[[0010-dynamics-subpackage-and-odometry-state]] §4 ("the right field to add *with* the wind /
independent-yaw model that gives it meaning and its own ADR"). This rung is that model. Independent
of 4f (VTOL) and 4d (fixed-wing) — it can be pulled forward whenever independent-yaw is actually
needed by a scenario, e.g. camera-pointing missions.

- [ ] **`AircraftState.yaw`** (new field, default `= trk`, so every existing construction — which
  never sets it — is unaffected) — clonable, like `turn_rate` (state.py's own reasoning for why
  `turn_rate` must live in state applies identically here).
- [ ] **`MotionCommand.target_yaw`** (new field, alongside the existing 2D/altitude "defined but not
  every vehicle acts on it" fields) + a yaw-rate limit on `Performance` (multirotor only).
- [ ] **`HolonomicDynamics`** — converges `yaw` toward `target_yaw` under the rate limit,
  **independent of** `target_velocity`'s convergence — the two channels are decoupled by
  construction, which is the entire point of this rung.
- [ ] **`FixedWingDynamics`** (4d) — `target_yaw` is the **absent-DOF** case from the feasibility
  table: ignored, with `yaw` reported as `trk` (+ crab, once wind exists), and an optional
  consistency assert if `target_yaw` disagrees with track beyond tolerance (catches a
  multirotor-guidance-wired-to-fixed-wing bug rather than silently flying it).
- *Check:* a multirotor commanded `(velocity east, yaw 45°)` translates east while `yaw` converges to
  45° and `trk` stays 90° — the two never re-couple; a fixed-wing given the same command flies its
  track unchanged and either ignores or rejects `target_yaw` per the chosen policy.
- *Relations:* the trigger case is this conversation's east-velocity/45°-yaw command; extends
  [[0008-velocity-vector-command]] §4 and [[0010-dynamics-subpackage-and-odometry-state]] §4, which
  both named this exact deferral. Needs its own ADR (below) rather than folding into ADR 0011 — it
  changes `AircraftState`, which 0011 deliberately did not touch.

- [ ] **ADR 001y — yaw-carrying state (independent multirotor yaw).** Why now (a real consumer
  exists: camera-pointing / independent-yaw missions, and the fixed-wing absent-DOF case makes the
  contrast concrete); why `yaw` defaults to `trk` (behaviour-preserving for every existing
  construction); why fixed-wing treats it as an output, not an input.

### 4f — VTOL (last)

- [ ] **`opencdarr/dynamics/vtol.py`** — `VTOLDynamics` with `VTOLMode {MULTIROTOR, TRANSITION,
  FIXED_WING}`, delegating to `HolonomicDynamics` / `FixedWingDynamics` by mode; the transition model
  handles mode switching + changing performance limits, kept **out of** the command interface (the
  TODO: "low-level transition details should not leak into the command interface"). Mode is clonable
  state on the aircraft.
  - *Check:* a VTOL takes off (multirotor mode), transitions, cruises (fixed-wing mode); the same
    `MotionCommand` produces mode-appropriate behaviour.
  - *Relations:* composition over the two existing models — no new physics beyond the transition.

- [ ] **ADR 001z — VTOL mode model.**

---

## Tests (the gate for each rung)

- [ ] `test_motion_command.py` — `from_track_speed`/`gs`/`trk` round-trip; under-specified command
  for a vehicle fails fast.
- [ ] `test_autopilot_cruise.py` — constant command, independent of (noisy) state — the 4a
  behaviour-preserving property.
- [ ] `test_separation.py` — byte-identical decisions to the old `_decide` on shared inputs; no state
  on the manager object (memory threaded in/out).
- [ ] `test_loop.py` (extend) — **4a regression: MVP/VO IPR bit-for-bit** vs the ADR 0010 anchors;
  reproducible from seed.
- [ ] `test_autopilot_goto.py` — multirotor reaches a goto point, settles, direct path.
- [ ] `test_autopilot_waypoints.py` — 3-waypoint plan flown in order; leg index is clonable (mid-plan
  clone continues correctly).
- [ ] `test_autopilot_loiter.py` — multirotor holds at goal (hover); fixed-wing orbits (4d).
- [ ] `test_fixedwing_dynamics.py` — analytical validation vs the paper; cannot stop / cannot side-slip
  (4d).
- [ ] `test_loop_mixed_fleet.py` — multirotor vs fixed-wing through `run_encounter` resolves; IPR
  reproducible (4d).
- [ ] `test_yaw_carrying_state.py` — multirotor: yaw converges to `target_yaw` independent of
  `target_velocity`'s convergence, `trk` unaffected; fixed-wing: `target_yaw` ignored (or rejected
  under the consistency-assert policy), `yaw` reported `== trk` (4e).
- [ ] `test_vtol.py` — mode transitions produce mode-appropriate behaviour (4f).

---

## Relations to the companion docs

- `design_brief.md` — Mission/Autopilot/Dynamics is the layering the brief anticipates; the
  autopilot and separation manager are the "how" and "safety overlay" over the pure-value CD/CR/CRR.
- `design-philosophy.md` — pure `state → value` layers (#1), one owner of clonable state (#2, the
  crux of decision 4), name it like the domain (#6, "autopilot"/"separation manager"), interface as
  contribution surface (#10, a new vehicle class = new files, not a loop fork).
- ADRs: extends [[0007-dynamics-as-pluggable-interface]] (pluggable physics), supersedes
  [[0008-velocity-vector-command]] (MotionCommand), builds on
  [[0009-holonomic-dynamics]]/[[0010-dynamics-subpackage-and-odometry-state]] (the multirotor
  models + odometry obligation), preserves [[0004-layered-directed-design-for-multiaircraft-and-ips]]
  (directed, pairwise-now/n>2-later).
- Observations: [[controlling-dubins-vs-holonomic]] and [[mixed-fleet-dubins-holonomic]] — the
  per-aircraft-dynamics follow-up they flag is closed in 4a/4d.

## References (read, not ported)

- The TODO this implements: [[TODO-autopilot-separator-dynamic-integration]].
- Fixed-wing dynamics: Reyner & Liem, *Energy-Efficient Trochoidal Path Planning for Unmanned
  Aircraft Under Wind and Performance Constraints* (Drones 2026, 10, 426) — `papers/drones-wind.pdf`
  (4d) — re-derived, analytically validated ([[0002-analytical-validation-of-dynamics]]), never
  ported. We take only its **kinematic point-mass model** (its Eqs 1–9: coordinated-turn yaw +
  wind vector-sum kinematics), not its path-planning methodology (BSB/BBB/SBB maneuver synthesis,
  the IEM energy metric, the Bayesian path-angle optimization) — those are a *planner*, not
  vehicle dynamics. The wind terms of that same model are what [[phase-5-plan|Phase 5]] switches on.
- Guidance laws (goto / waypoint / L1 pure-pursuit): PX4 / ArduPilot guidance as *reference for the
  effect*, not the implementation (same discipline as the ADS-L modelling in Phase 3).
