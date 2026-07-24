# Phase 4e — Mixed-fleet DAA through `run_encounter`

Parent: [[phase-4-plan]]. Closes the per-aircraft-dynamics follow-up [[mixed-fleet-dubins-holonomic]]
flagged, now with genuinely different physics per side, and confronts the one honest gap in the
"MVP/VO emit velocity" story: **a velocity setpoint is a native multirotor offboard channel but not a
fixed-wing one** (parent §4). This rung decides how a velocity avoidance reaches a fixed-wing.

## The velocity→fixed-wing-setpoint problem

MVP/VO produce an avoidance **velocity vector**. For a **multirotor** that is directly a valid
`TrajectorySetpoint.velocity` — nothing to solve. For a **fixed-wing**, PX4 offboard does not take a
velocity; it takes `course` / `airspeed_direction` + `airspeed`. So the resolver's velocity must be
**projected** onto the fixed-wing channels:

- `target_course` = direction of the avoidance velocity (its track);
- `target_airspeed` = clamp(|avoidance velocity|, `[v_stall, v_max]`);
- feasibility handled by `FixedWing` (turn-rate-limited convergence; it cannot snap).

Decide where this projection lives: **recommend a thin adapter in `SeparationManager` / the fixed-wing
autopilot**, not in MVP/VO (which stay vehicle-neutral, ADR 0011 §2). Document it as an approximation —
a velocity a multirotor achieves instantly, a fixed-wing only converges to under its turn limit, which
is the physically correct difference, not a bug.

## Checklist

- [ ] **Velocity→fixed-wing projection adapter** (per above), with the vehicle-neutral resolver output
  unchanged.
- [ ] **Per-aircraft bundle wired end-to-end** — `run_encounter` runs a multirotor vs a fixed-wing
  (each its own `dynamics` / `perf` / `autopilot`), the same entry point the IPR sweeps use (ADR 0011
  §7, done in 4a).
- [ ] **Observation write-up** — the mixed-fleet DAA result, contrasting how each airframe resolves the
  same geometry (multirotor sidesteps; fixed-wing turns through a feasible arc), in the
  [[mixed-fleet-dubins-holonomic]] lineage.

## Gate

- [ ] `test_loop_mixed_fleet.py` — multirotor-vs-fixed-wing through `run_encounter` **resolves**
  (min-sep ≥ rpz) and produces a **reproducible IPR** from seed.
- [ ] The projection adapter has a unit test: an avoidance velocity yields a feasible
  `(target_course, target_airspeed)` the fixed-wing converges to without violating stall / turn limits.
