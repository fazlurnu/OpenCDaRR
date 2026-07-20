# Future feature — time-varying reception probability

**Target version: post-v1.0** (`docs/roadmap.md`'s "community long game, beyond v1.0" bucket) —
tentative. Not urgent: revisit once there's an actual case that needs it, not on a schedule.

## What

`Comm.reception_prob` is fixed at construction — a scalar or a `{(source, receiver): p}` mapping,
evaluated identically every tick for the whole encounter (`opencdarr/cns/communication.py`,
`Comm._reception_for`). There is no `t` dependence: a link can't degrade mid-encounter (terrain
masking as geometry changes, range-dependent link budget, a transponder that's intermittently
failing).

## Why deferred

Came up while discussing `latency`'s pluggable-distribution design (`LatencyDistribution`,
`_as_latency`) — the natural question was whether `reception_prob` should have the same kind of
callable/schedule escape hatch. Not needed for the Phase 3b exit gate: reception loss and latency
both already demonstrably degrade IPR through the real loop
([[loop-communication-integration]]). A real extension, not a hypothetical one — just not
load-bearing for the current research goal.

## Sketch (shape only, not designed)

Mirror `LatencyDistribution`: a `ReceptionSchedule` protocol, `(rng, t) -> float`, and
`Comm._reception_for` gains a `t` parameter — cheap plumbing, since `step()` already has `t` in
scope. The hard part isn't the interface, it's *why* it varies (link-budget-vs-range model,
terrain/geometry-dependent masking, or just a scripted schedule for an experiment) — that's a
modeling decision to make when a concrete case demands it, not an engineering one to guess at now.

## Relations

- [[0006-communication-model-design]] — the `Comm` / `LatencyDistribution` design this would
  extend, same pattern applied to the other stochastic parameter.
- `opencdarr/cns/communication.py` — `Comm._reception_for`, the one function that would change.
