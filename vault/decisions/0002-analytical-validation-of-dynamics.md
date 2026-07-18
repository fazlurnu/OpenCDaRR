# ADR 0002 — Validate `step_dynamics` analytically, with BlueSky-sourced constants

- Status: accepted
- Date: 2026-07-18
- Deciders: Fazlur Rahman

## Context

`step_dynamics` is extracted from BlueSky's M600 model (`how-to-step-by-step.md` Step 1). We
must decide what "close enough to BlueSky" means — its own phrasing leaves this as the
builder's call. Two routes: bit-match a recorded BlueSky trajectory, or check the integrator
against first-principles physics while sourcing the *constants* from BlueSky.

`design_brief.md` is explicit that this is a **redesign, not a port**: *"Not bit-compatibility
with the old pipeline — this is a redesign, so it produces new, independently-validated
numbers."*

## Decision

**Validate analytically.** The acceptance gate is three first-principles checks the integrator
must satisfy, with the flight-envelope constants taken from BlueSky's M600 data (so it is the
M600, not an invention):

1. **Straight line** — 10 m/s held for 10 s covers ≈ 100 m; heading and turn rate unchanged.
2. **Turn under limits** — a 90° command turns with `|ω| ≤ max_tr` and per-step
   `|Δω| ≤ max_dtr2·dt`, holds commanded speed, and converges.
3. **Speed cap** — a command above the envelope clamps to `v_max` (and below to `v_min`).

These live in `tests/test_dynamics.py` — pure, numpy-only, always in the gate.

Separately, a BlueSky **equivalence anchor** (`tests/test_dynamics_vs_bluesky.py`) cross-checks
the geometry against BlueSky's own integrator where BlueSky is available. It is
`importorskip`-guarded and NOT part of the core gate (ADR 0003).

## Alternatives rejected

- **Bit-match a recorded trajectory.** Against the redesign non-goal, and brittle: a golden
  blob is opaque, whereas "10 m/s for 10 s → 100 m" is legible to a reviewer
  (`design-philosophy.md` #11). We would also be locking onto BlueSky's exact integration
  scheme, which we are deliberately free to improve.

## Consequences

- **Good:** the gate is legible, fast, and dependency-free; it states physics, not a hash.
  The optional anchor still gives empirical confidence (in practice: heading matches BlueSky
  to machine precision, position to sub-metre).
- **Cost:** analytical checks do not, by themselves, prove agreement with BlueSky's exact
  numbers — that is what the anchor adds, when run.
- **Obligation:** each check must *bite* (`how-to-step-by-step.md` Part A #5) — we confirm a
  wrong implementation fails it.

## Relations

- Governs `tests/test_dynamics.py`; anchored by `tests/test_dynamics_vs_bluesky.py`.
- Derivation: `vault/derivations/step-dynamics-m600.md`.
- Constants: `opencdarr/performance.py`; the runtime-independence decision is ADR 0003.
