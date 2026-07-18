# ADR 0003 — Own the geodesy; BlueSky-free at runtime

- Status: accepted
- Date: 2026-07-18
- Deciders: Fazlur Rahman
- Supersedes: the note in `phase-1-plan.md` to call `bluesky.tools.geo` from `dynamics.py`.

## Context

With `step_dynamics` extracted (ADR-adjacent, Phase 1) and `Performance` owned as plain
data, the *only* remaining tie to BlueSky at runtime was a single call —
`bluesky.tools.geo.qdrpos` — to propagate lat/lon. Three things make keeping it a bad trade:

- **It is ~5 lines of standard great-circle geodesy.** Owning it is trivial.
- **BlueSky is a heavy, fragile runtime dependency here.** It would not even boot in the base
  env (a stale `navdata.p` pickle vs numpy 2.x); it is a custom fork, not a PyPI package.
- **The platform is drone-only** and the dynamics are already ours, so BlueSky's traffic and
  performance machinery buys nothing at runtime.

## Decision

Own the geodesy in `opencdarr/geo.py` (`forward`, `earth_radius`) and remove every BlueSky
import from `opencdarr/`. **`opencdarr` depends only on numpy at runtime.**

BlueSky is retained purely as an **offline validation anchor**: a single, `importorskip`-
guarded integration test cross-checks `step_dynamics` against BlueSky's M600 integrator where
BlueSky happens to be installed. It never runs in the core gate and is never imported by
shipping code.

`geo.forward` mirrors BlueSky's `qdrpos` math (same WGS84 latitude-dependent radius), so the
two agree to floating-point precision — we own the code *and* keep the equivalence.

## Alternatives rejected

- **Keep calling `bluesky.tools.geo` (the earlier plan).** Rejected: a whole fork as a
  runtime dependency, plus its boot fragility, for five lines of textbook math.
- **Use a third-party geodesy lib (pyproj/geographiclib).** Rejected for now: another
  dependency for something this small; revisit only if we need full geodesic accuracy.

## Consequences

- **Good:** `pip install -e .` needs only numpy; no fork, no navdata cache, no env drift.
  Parallel/CI runs are clean. The dynamics boundary is now genuinely swappable (the spine).
- **Cost:** we own ~5 lines of geodesy and its test. Cheap, and validated against BlueSky.
- **Obligation:** the BlueSky anchor test must stay runnable (in an env where BlueSky boots,
  e.g. `cdarr`) as the periodic equivalence check; keep it green when we touch dynamics.

## Relations

- Implemented by `opencdarr/geo.py`; consumed by `opencdarr/dynamics.py`.
- Realises the `design_brief.md` spine ("BlueSky provides stateless math… the engine is a
  dependency, not the architecture") — taken to its endpoint for a drone-only platform.
- Enables the roadmap's *pluggable dynamics / OpenAP* line without any BlueSky coupling.
- Anchored by `tests/test_dynamics_vs_bluesky.py` (guarded) and
  `vault/derivations/step-dynamics-m600.md`.
