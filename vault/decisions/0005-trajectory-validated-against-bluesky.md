# ADR 0005 — Trajectory validated against BlueSky; near-parallel divergence deferred

- Status: accepted
- Date: 2026-07-20
- Deciders: Fazlur Rahman

## Context

OpenCDaRR re-derives BlueSky's CD&R stack (detection, MVP resolution, past-CPA recovery, M600
point-mass dynamics) BlueSky-free (ADR 0003). To trust the port we validated it against the
reference (`CDaRR_git`, which runs BlueSky) at every level:

- **Algorithms** — feeding 100 identical conflict states into both `statebased` and `mvp` gives
  identical detection (100/100) and MVP commands (Δhdg ≤ 0.03°, Δspd ≤ 0.004 m/s — geodesy only).
- **Dynamics** — an isolated command-sequence replay matches BlueSky's turn integration exactly
  (`|Δtrack| = 0.000°`); the M600 acceleration was corrected to the measured BlueSky value
  (`ax = 3.5 m/s²`; the static `perf.axmax` reads a 2.0 placeholder before the aircraft moves).
- **Whole-trajectory, no noise** — a single conflict pair replayed in both stacks (2° and 90°),
  recorded in [[trajectory-level-comparison]].

The deterministic trajectories agree in heading and position, but **not in ground speed**: the
reference's ground speed drifts/ramps up during avoidance while ours stays near nominal. Under
noise this speed-channel difference is what makes our near-parallel IPR less fragile than the
reference's (ours ~0.9 vs ref ~0.12) — the reference sheds its maneuver back toward the collision
heading (resolving by acceleration), while we hold a lateral deflection until genuinely past CPA.

## Decision

**Accept OpenCDaRR's current behaviour as validated.** The port matches BlueSky wherever the two
*should* agree (algorithms, turn dynamics, recovery logic, no-noise position/heading). The
remaining divergence is isolated to the **ground-speed channel** and traced to a **BlueSky
ground-speed drift** — a large MVP-commanded acceleration during resolution plus a small CAS/TAS
offset (`SPD` is calibrated airspeed; the aircraft flies TAS ≈ CAS × 1.005). Our pure-ground-speed
`step_dynamics` reproduces neither.

We treat the near-parallel IPR gap as a consequence of that speed-channel difference and, on
balance, judge our behaviour (maintain avoidance until past CPA) at least as defensible as the
reference's. **The exact BlueSky ground-speed drift is flagged for later inspection, not chased
now** — it does not block Phase 3, and every other component is verified equivalent.

Evidence and reproduction: [[trajectory-level-comparison]] and
[`scripts/trajectory_comparison/`](../../scripts/trajectory_comparison/).

## Alternatives rejected

- **Match the reference bit-for-bit at near-parallel by shedding the maneuver.** Would make us
  reproduce the reference's near-parallel fragility (re-converging into LoS off a collision
  heading) — behaviour we consider a reference quirk, not ground truth. Rejected.
- **Port BlueSky's CAS/TAS + atmosphere into `step_dynamics` now.** Adds an atmosphere model to
  the dynamics boundary for a sub-percent, symmetric speed offset, before we know it matters.
  Deferred to the flagged inspection instead. Rejected for now.
- **Leave the divergence undocumented.** It would be re-litigated. Rejected — hence this ADR.

## Consequences

- **Good:** the port is validated with a concrete, reproducible cross-check; the one open
  difference is named, bounded (ground-speed channel), and pointed at a follow-up rather than
  blocking. Phase 3 proceeds.
- **Cost / risk:** our near-parallel IPR will read higher than the reference's until the
  ground-speed drift is understood; anyone comparing raw IPR to `CDaRR_git` must read
  [[trajectory-level-comparison]] first.
- **Follow-up:** inspect the BlueSky ground-speed drift (MVP-commanded acceleration profile at
  near-parallel + the CAS/TAS handling) and decide whether to model it in `step_dynamics`.
