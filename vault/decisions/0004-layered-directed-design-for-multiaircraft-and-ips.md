# ADR 0004 — Layered, directed design so N-aircraft and IPS generalize without a rewrite

- Status: accepted
- Date: 2026-07-19
- Deciders: Fazlur Rahman

## Context

Phase 2 builds a **pairwise** encounter (2 aircraft, plain Monte Carlo). But the research needs
two generalizations that must not require rewriting the core:

- **Multi-aircraft** simultaneous conflict — e.g. 5 aircraft in conflict at once (roadmap v0.3,
  reviewer item #1).
- **Rare-event estimation** via the Blom–Bakker interacting particle system (IPS, v0.4).

This ADR records *why* the model and estimator are shaped the way they are, so both fall out of
the existing design rather than a redesign.

## Decision

Two architectural commitments, already reflected in the Phase 1–2 code:

1. **The CDR model is directed, pairwise-primitive, over clonable state.** `detect`, `resolve`,
   `should_resume` each act on one directed pair *(observer → its perceived target)* and return a
   value; algorithms **compose** (MVP sums the pairwise resolution vectors over an aircraft's
   conflicts). Everything an aircraft's future depends on lives in its (clonable) state.
2. **The estimator sits over an abstract interface, independent of the environment:**
   `advance(state, dt, rng)`, `level(state)`, `is_terminal(state)`. Monte Carlo today, IPS later,
   see only these three functions — never the number of aircraft.

## Consequences — how 5 aircraft generalize (v0.3)

- **State → a fleet** of N `AircraftState`s (+ each one's nominal, resolving flag, perf). Still
  plain, clonable data.
- **Detection is unchanged** — `detect(own, intr, …)` is directed and pairwise by design; for N
  aircraft you iterate it over the **conflict graph** (`n(n−1)` directed pairs). The primitive
  does not change.
- **Resolution generalizes to a set:** `resolve(own, intruders, rpz)` — MVP sums the pairwise
  `dv`s (the old code already does this). Today's `resolve(own, intr)` is the `len==1` case.
- **Recovery** becomes "resume when clear of *all* my conflicts."
- **New piece — the coordination model** (cooperative / priority / sequential): a genuine research
  decision, written as its own ADR when v0.3 lands. Pairwise cooperative-symmetric is its N=2
  instance.
- **Gate:** at N=2 the N-aircraft environment must reduce to the pairwise result — a free
  regression check.

## Consequences — how IPS generalizes (v0.4)

- **IPS is oblivious to N.** It runs entirely on `advance / level / is_terminal`; the aircraft
  count appears in exactly one place — `level`.
- **Particle = the full N-aircraft world** (all N states + every aircraft's recovery state +
  the RNG substream), clonable. With more aircraft there is *more* per-aircraft recovery state,
  so the **no-hidden-state invariant is more load-bearing, not less** — a clone that lost any of
  it would diverge (KI-1 at scale). This is the brief's "recovery state per-particle by design."
- **`level(state)` carries the multi-aircraft meaning:** start with the **minimum pairwise
  separation across all pairs**; a smarter importance function for *simultaneous* conflicts is
  itself a contribution (its own ADR at v0.4).
- **Cloning** an N-aircraft particle is a deeper deep-copy + a fresh `SeedSequence.spawn`; the
  stream tree just gets deeper, which `spawn` handles reproducibly (ADR 0001).

## One-line version

The **environment** grows from 2 to N aircraft (state → fleet, detection over a conflict graph, a
coordination-model ADR, MVP-summed resolution). The **estimator** does not change at all — it sees
only `advance / level / is_terminal`, and "N aircraft" appears only inside `level`. That
separation is the whole point of building the model and the estimator as independent layers.

## Relations

- Realises the `design_brief.md` spine and load-bearing interface; depends on ADR 0001 (per-
  particle RNG).
- Forward-links to future ADRs: **coordination model** (v0.3) and **IPS level/importance
  function** (v0.4).
- Roadmap: v0.3 (multi-aircraft), v0.4 (rare events).
