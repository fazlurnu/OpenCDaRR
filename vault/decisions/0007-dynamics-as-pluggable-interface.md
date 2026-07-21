# ADR 0007 — Dynamics as a pluggable interface (`Dynamics` ABC)

- Status: accepted
- Date: 2026-07-21
- Deciders: Fazlur Rahman

## Context

Every other model family in this codebase — detection, resolution, recovery, navigation,
communication, surveillance — is behind an `ABC` and threaded through `run_encounter` as a
parameter: "the interface is the contribution surface" (`docs/design_brief.md`, "Open source &
community contribution"). `step_dynamics` was the one exception: a bare function, imported by
name and called directly inside `run_encounter` (`opencdarr/loop.py`), with no seam for a
different airframe or a different physical model.

This surfaced concretely, not hypothetically, while comparing OpenCDaRR against BlueSky/CDaRR_git
for the *same straight-flight scenario*. BlueSky's `SPD` command sets CAS, converted to TAS/GS
through the ISA atmosphere at altitude (`bluesky/traffic/autopilot.py:721-729`,
`vcasormach2tas`) — a persistent ~0.48%-at-100m ground-speed bias `step_dynamics` doesn't
reproduce, because it has no atmosphere model. Investigating whether to import that correction
raised the real question: should a multirotor's commanded speed be interpreted as *calibrated
airspeed* at all, given it has no pitot-static system and is GPS/IMU-controlled? Checking
BlueSky's source confirmed the CAS interpretation is applied uniformly regardless of airframe
class (no rotor-vs-fixed-wing branch in `Autopilot.selspdcmd`/`vcasormach2tas`) — it's inherited
manned-aviation baggage, not something specifically validated for the M600. The physically real
correction for a drone is instead the **wind triangle** (ground velocity = airspeed vector + wind
vector) — a legitimate *second* `Dynamics` implementation, not a variant of the first.

Without an interface, adding that (or any other airframe/effect) would mean forking
`run_encounter` or monkeypatching `opencdarr.dynamics.step_dynamics` — exactly the "fork the
core" outcome the rest of the architecture exists to avoid.

## Decision

### 1. `Dynamics` ABC

```python
class Dynamics(ABC):
    @abstractmethod
    def step(
        self, state: AircraftState, command: Command, perf: Performance, dt: float
    ) -> AircraftState:
        ...
```

One method, the same shape `step_dynamics` already had — directed, pure, no globals, so a clone
(IPS particle) evolved through it stays independent of its source (`state.py`'s no-hidden-state
invariant). Lives in `opencdarr/dynamics.py` beside `step_dynamics`, not a new subpackage (see
Alternatives) — mirrors `ConflictDetector` / `ConflictResolver` / `RecoveryCriterion`.

### 2. `PointMassDynamics` is the default, a thin wrapper

```python
class PointMassDynamics(Dynamics):
    def step(self, state, command, perf, dt):
        return step_dynamics(state, command, perf, dt)
```

`step_dynamics` itself is untouched — same signature, same behaviour, still directly importable
and used by `test_dynamics.py`, `test_dynamics_vs_bluesky.py`, and
`scripts/trajectory_comparison/run_ours.py`. `PointMassDynamics` adds no math of its own; it's
the object identity `run_encounter` needs in order to be pluggable.

Naming: `PointMassDynamics`, not `M600Dynamics`. The turn-rate/acceleration-limiter law is
airframe-agnostic — it works for any `Performance` instance — only the *constants* (`max_tr`,
`ax`, ...) are M600-specific. `performance.py` already draws this line ("a new airframe is a new
`Performance` instance, not an edit to the step function"); the class name follows it. "M600"
belongs to the `Performance` value passed in, not to the dynamics class.

### 3. `run_encounter` takes `dynamics: Dynamics = PointMassDynamics()`

Added next to `perf` in `loop.py`. Every existing caller is unaffected — the default reproduces
prior behaviour exactly, proven by `test_point_mass_dynamics_matches_step_dynamics` (bit-for-bit
against `step_dynamics`) and by the full existing suite passing unmodified. A caller wanting a
different airframe, or a future wind-aware model, subclasses `Dynamics` and passes an instance;
no edit to `loop.py` is needed.

## Alternatives rejected

- **A subpackage (`dynamics/base.py` + `dynamics/m600.py`), matching `cd/`/`cr/`/`crr/`.**
  Rejected for now: those subpackages exist because they already hold *multiple*
  implementations; dynamics has one (`PointMassDynamics`) plus a *planned* second (wind).
  Restructuring the module preemptively touches every import site (`loop.py`, `state.py`, three
  test files, one script) for a benefit that only materialises once the wind model actually
  exists. Revisit when it lands — at that point it's a mechanical, low-risk move, not a design
  decision.
- **`Protocol` instead of `ABC`.** Rejected for consistency: every other *model-family*
  interface here (`ConflictDetector`, `ConflictResolver`, `RecoveryCriterion`, `NavigationModel`,
  `CommunicationModel`, `SurveillanceModel`) is an `ABC`; `Protocol` is reserved for smaller
  compositional pieces passed *into* those models (`NoiseDistribution`, `LatencyDistribution`).
  `Dynamics` is a model family, not a compositional piece.
- **Import CAS/TAS into `PointMassDynamics` to chase BlueSky parity.** Rejected as the wrong
  question, not just deferred. A real M600 has no pitot-static system, so modelling calibrated
  airspeed for it isn't "more accurate" — it imports a manned-aviation abstraction BlueSky
  applies uniformly regardless of airframe class (confirmed in `autopilot.py`: no rotor-vs-fixed-
  wing branch). Reproducing it would make numbers match a specific existing computation, not make
  the model more correct — those are different goals, and conflating them is exactly why this
  needed to become a *second implementation* behind an interface, not a parameter bolted onto the
  first.

## Consequences

- **Good:** dynamics now follows the same contribution pattern as every other layer — a new
  airframe or physical effect is a file and a class, not a fork of `run_encounter`. Proven by
  `test_dynamics_is_pluggable` (`test_loop.py`), which swaps in a frozen no-op `Dynamics` and
  shows the encounter's outcome changes accordingly — the parameter is load-bearing, not
  decorative.
- **Cost:** one more constructor argument on `run_encounter`. A contributor implementing a new
  `Dynamics` must keep it pure (no globals) themselves — nothing in the ABC enforces this beyond
  the docstring, the same trust model already accepted for `ConflictDetector`/`ConflictResolver`.
- **Obligation:** when the wind-aware model lands, it becomes the second `Dynamics`
  implementation and the concrete trigger to revisit the subpackage question above.

## Relations

- Extends the "interface is the contribution surface" principle (`docs/design_brief.md`, "Open
  source & community contribution") to the one core layer that didn't yet follow it.
- Sibling to [[0002-analytical-validation-of-dynamics]] and
  [[0003-own-the-geodesy-bluesky-free-runtime]] — those established `step_dynamics` itself; this
  ADR is about how it's *reached*, not its math.
- The wind-aware `Dynamics` implementation this was motivated by is not yet designed — tracked as
  a future ADR, not written here (the wind-triangle equations were sketched in conversation but
  not yet committed to `vault/derivations/`).
