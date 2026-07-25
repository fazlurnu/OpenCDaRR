# Future feature — priority / give-way coordination (and an explicit coordination model)

**Target version: v0.3+ (a follow-up within the multi-aircraft line)** — a real design question
raised during Phase 6, deferred so the fleet lands on the cooperative baseline first; a marker, not
scheduled.

## What

Today the fleet coordination is **implicit and cooperative-symmetric**: in `run_fleet`
(`opencdarr/fleet.py`) every aircraft independently runs its own detect → resolve → recover against
all the others, so in a conflicting pair **both** aircraft maneuver. There is no explicit
coordination object; "everyone cooperates equally" is baked into the loop.

Two related pieces are deferred here:

1. **An explicit, swappable coordination model** — a small `opencdarr/coordination/` family (an ABC +
   the current `CooperativeSymmetric` as its default) that decides *how the resolution burden is
   allocated per conflicting pair*, so a different policy is a new file, not a fork of the loop
   (ADR 0004's forward-linked "coordination-model ADR"). It only becomes worth its abstraction weight
   once there is a *second* model to host — which is:
2. **A priority / give-way model** — rules-of-the-air style: the **lower-priority** aircraft in a pair
   takes the *full* avoidance vector while the **higher-priority** one holds, instead of both
   maneuvering. This is exactly BlueSky's `applyprio` (`bluesky/traffic/asas/mvp.py`,
   `dv1 = dv1 − dv_mvp` vs `dv2 = dv2 + dv_mvp`, with FF1/FF2/FF3/LAY priority codes).

## Why it matters (why it came up)

The Phase-6 cooperative demos showed the cost of everyone-maneuvers-symmetrically: on the ±60°
three-aircraft and the 8-aircraft converging-ring superconflicts, cooperative-symmetric MVP produces
**large, inefficient give-way detours** and, before the `_BIAS_EPS` fix, a near-head-on **livelock**
([[fleet-cooperative-ring]], [[headon-threshold]], [[multi-intruder-vo-vs-mvp]]). A priority model —
one aircraft gives way, the other holds — is the standard fix: it breaks the symmetry, halves the
combined maneuvering, and matches how real rules-of-the-air allocate responsibility. The
**symmetric-vs-priority contrast on the superconflict** is the plot this feature would produce.

## Why deferred

The cooperative-symmetric baseline is enough to build and validate the multi-aircraft *environment*,
the *scenarios*, and the *IPR sweep* (Phase 6d/6e) — those don't need a second coordination policy.
Adding the priority model (and the coordination abstraction to host it) is a genuine, self-contained
research increment with its own ADR, best done once the cooperative baseline and the scenarios exist
to compare against. Deferring it keeps Phase 6 focused on the environment + scenarios + metrics, and
avoids introducing a one-implementation abstraction before its second consumer exists (the
`state.py` "no speculative structure" rule).

## When it lands

- A `Coordination` ABC + `CooperativeSymmetric` (a no-op allocation = today's behaviour, so the fleet
  reduces exactly), threaded into the fleet resolve step.
- A `Priority` model (per-aircraft priority; lower gives way, higher holds), its own ADR.
- The symmetric-vs-priority contrast observation on the converging-ring superconflict.
