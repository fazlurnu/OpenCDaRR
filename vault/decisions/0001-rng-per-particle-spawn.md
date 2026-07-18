# ADR 0001 — Per-particle RNG via SeedSequence.spawn

- Status: accepted
- Date: 2026-07-18
- Deciders: Fazlur Rahman

## Context

This is a rare-event research platform: the headline output is a collision probability as
small as 1e-9, reported **with a confidence interval** (`design_brief.md` #2). A number that
small is worthless if it cannot be regenerated exactly — reproducibility is a *feature*, not
a nicety (`design-philosophy.md` #4). Randomness enters in several independent places: CNS
noise, message reception, encounter generation, and — critically — the Blom–Bakker
interacting particle system (IPS), which **clones** particles and must let each clone explore
an *independent* future.

The old code got this wrong in two documented ways (`lesson-learnt.md`): a shared/global RNG
(the ADSL bug, where reception and noise drew from one stream and shifted it mid-run) and a
singleton that leaked state across runs (KI-1). Both are the same root cause — **implicit,
shared stochastic state**.

## Decision

Every stochastic component receives its **own** random generator, derived reproducibly from a
single root seed using `numpy.random.SeedSequence`:

- A run has **one integer seed**. From it we build a root `SeedSequence`.
- Substreams are created with `SeedSequence.spawn(n)`, which produces `n` **statistically
  independent** child sequences. Each child yields a `numpy.random.Generator`
  (`default_rng(child)`), backed by PCG64.
- **No module-level, global, or shared RNG anywhere.** A function that needs randomness takes
  a `Generator` as an explicit argument (`design-philosophy.md` #2, #3).
- The **stream layout is documented, not implicit**: which substream feeds which component,
  and — for IPS — how a particle spawns its children's streams, is written down (in `rng.py`'s
  docstring and the derivation note), so the assignment is part of the provenance.

## Alternatives rejected

- **One global/shared `np.random` RNG.** This is exactly the ADSL bug. Any reordering or
  added draw silently changes every downstream result. Rejected.
- **Re-seeding each component with `seed + k` integer offsets.** Nearby seeds can produce
  correlated streams; there is no independence guarantee. `SeedSequence.spawn` exists
  precisely to avoid this. Rejected.
- **Passing raw integer seeds around and re-creating generators ad hoc.** Loses the
  documented tree structure and invites accidental reuse of the same stream. Rejected.

## Consequences

- **Good:** deterministic given a seed; independent substreams by construction; IPS can clone
  a particle and hand each child an independent stream, so cloned futures do not correlate —
  the property that makes a 1e-9 estimate trustworthy. Directly forecloses the ADSL/KI-1 bug
  class.
- **Cost:** every stochastic function signature carries an explicit `rng` argument (more
  verbose than reaching for a global). We accept this — explicitness is the point.
- **Obligation:** the substream-to-component mapping must be kept in sync with the code and
  recorded in each experiment's provenance card. A test asserts reproducibility and
  independence from Phase 0 onward.

## Relations

- Implemented by `opencdarr/rng.py` (Phase 0, file #6) and exercised by `tests/test_smoke.py`.
- Enables the IPS estimator (`design_brief.md` #2; roadmap v0.4).
- Fixes the root cause of the ADSL shared-RNG and KI-1 leaks (`lesson-learnt.md`).
- Embodies `design-philosophy.md` #3 ("every stochastic thing takes its own RNG") and #4
  (reproducibility tested like a feature).
