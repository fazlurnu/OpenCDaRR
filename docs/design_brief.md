# Design Brief — CDaRR, rebuilt with BlueSky as a library

Written 2026-07-18. Successor to `refactor_fp.md` and the (now abandoned) engine-rewrite
spec. This is a **brief, not an implementation plan** — deliberately lean, per
`lesson-learnt.md`: it fixes the goal and the spine, names the decisions, and stops. No
36 KB phase-by-phase blueprint this time.

---

## Goal

A conflict-detection-&-resolution (CDR) research platform that:

1. supports **diverse environments**, including **multi-aircraft simultaneous conflict**
   (not just pairwise);
2. enables **rare-event estimation** via the Blom & Bakker **interacting particle system
   (IPS)**;
3. is **legible to a human** — structure, data, and functions readable top-to-bottom;
4. is documented in a linked **Obsidian-style knowledge vault**.

In service of the paper (the reviewer action items) and of the recovery-criteria research
you actually want to do (`my-observation.md` #14–16: signed-tCPA / balanced criteria).

**Non-goals.** Not a BlueSky rewrite — we keep it as the engine *for now*, behind a
boundary that lets it be replaced later without a rewrite. Not bit-compatibility with the old
pipeline — this is a *redesign*, so it produces new, independently-validated numbers. Not
maximal generality — build for the experiments you run.

---

## The spine: BlueSky as a library, not the runtime

The one decision everything else hangs on.

> **You own the state and the loop. BlueSky provides stateless math.**

Concretely: no more `bs.traf` owning the world, no `bs.sim.step()` driving it, no
`Entity`-singleton CDR objects. The world is **plain, copyable data** (arrays / a small
dataclass). BlueSky is called only for its pure functions — `bluesky.tools.geo`
(bearings, distances), `bluesky.tools.aero` (unit conversions), and the **M600
turn-rate-limited point-mass kinematics** (its `MAX_TR`/`MAX_DTR2` behaviour), which gets
**extracted into a pure `step_dynamics(state, command, dt) -> state`**.

Why this specific inversion:

- It fixes the actual "bad" from last time — the global-state coupling that produced both
  bugs (KI-1 singleton leak, ADSL shared-RNG).
- It is a **hard prerequisite for IPS**: you cannot clone a global-singleton world, but
  you can clone a dataclass. Requirement #2 forces this; requirement #3 (legibility) is
  paid for by it.
- It makes **the simulator engine itself replaceable.** BlueSky is the engine *for now* —
  used for its validated M600 kinematics and geo/aero math. But it sits behind the
  `step_dynamics` (and geo/aero) boundary, so it can later be swapped for a different
  integrator or simulator without touching state, CNS, CDR, or the estimators. The engine
  is a *dependency*, not the architecture. (Design every use of BlueSky asking: "when this
  gets replaced, what breaks?" — the answer should be "only the code behind the boundary.")

The reusable payoff from the fp-refactor: the pure `cd/`, `cr/`, `crr/`, `cns/` packages
already operate on explicit inputs — they slot straight into this design as the CDR and
sensing layers. The piece that was never solved (and is the real work here) is the
**state ownership + dynamics layer**.

---

## The load-bearing interface

Design every environment and estimator around three pure functions:

```
advance(state, dt, rng) -> state       # one time step of the whole world
level(state) -> float                  # importance / distance-to-collision
is_terminal(state) -> bool             # horizon reached, or collision/clear
```

Plain Monte Carlo, importance sampling, multilevel splitting, and **Blom–Bakker IPS all
plug in over this same interface** without touching the model. Nail this and the rest is
composition.

---

## Architecture (thin layers, each independently testable)

- **state** — explicit, clonable: aircraft kinematics + **per-particle CDR recovery
  state** (`resopairs`, initial intruder velocities — the exact thing that leaked in
  KI-1 becomes part of the particle *by design*) + an RNG substream.
- **dynamics** — `step_dynamics`, the extracted turn/accel-limited M600 model.
- **cns** — noise / comms as pure functions, RNG threaded (salvage from fp-refactor).
- **cdr** — detection / resolution / recovery as pure functions (salvage `cd`/`cr`/`crr`).
- **environment** — assembles the above into `advance` / `level` / `is_terminal`; one per
  scenario family (pairwise, multi-aircraft, …).
- **estimator** — MC and IPS over the interface, with **rare-event metrics** (see below).
- **scenario** — encounter generators: pairwise, multi-aircraft, and *operationally
  realistic* distributions (reviewer item #10), not uniform geometry.
- **experiment / provenance** — one run = `config + seed + git-hash + env → result`.
- **analysis** — plotting + the validation ladder.

---

## What each requirement needs (the parts easy to forget)

**Multi-aircraft (#1) is a modeling choice, not bigger arrays.** You need (a) a conflict
graph / connected components (who conflicts with whom), and (b) an explicit
**coordination model** for >2 aircraft — cooperative vs. priority vs. sequential. This is
a genuine research decision the reviewers flagged; make it an ADR, not an accident.

**IPS (#2).**
- A **level function** `φ(state)` monotone toward collision (start with min pairwise
  separation; a smarter importance function is itself a contribution).
- **Particle = full clonable state**, advanceable independently. This is why the spine
  matters.
- **RNG must spawn reproducibly per particle** — `numpy.random.SeedSequence.spawn()` (or
  PCG streams). A 1e-9 estimate you can't reproduce is worthless; seed provenance matters
  more here than anywhere in the old code.
- **Rare-event outputs**, not IPR: estimated collision probability **with a confidence
  interval**, the intermediate level-crossing probabilities, and effective sample size.
  IPR (a fraction) is meaningless at 1e-9.

**Legibility (#3).** Small pure functions with the governing equation in the docstring;
one obvious `state` type (no 4-node ADSL message dance); a single `run_one_experiment`
entry point a newcomer can read straight through; architecture + data-flow diagrams live
in the vault.

**Knowledge vault (#4).** Structure that links *to the code*, not floating beside it:
```
decisions/     ADRs — WHY BlueSky-as-library, WHY this coordination model, WHY this level fn
derivations/   the math (dCPA dist, projected-normal, IPS levels) in LaTeX, linked to module + test
observations/  my-observation.md, split into notes, each linked to the experiment that shows it
algorithms/    one note per algorithm  <->  its module
papers/        lit notes: Blom & Bakker (IPS), Schaefer & Jonas (ADS-B noise)
experiments/   one provenance card per run
```

---

## Reproducibility & validation (keep the good habits)

- **Provenance card per result**; deterministic given seed; the RNG stream layout is
  documented, not implicit.
- **Golden/regression tests repointed at the new clean code** (guardrail, not a cage
  around old behaviour) — seeded from a trusted run.
- **Validation ladder:** analytical ⊂ brute-force MC ⊂ IPS, cross-checked in the
  overlapping regime where two are both feasible (extends your `appendixB` discipline).
  Never trust a 1e-9 number that isn't anchored to something checkable.
- Keep CDR criteria as **pure boolean predicates** → property-based tests (Hypothesis)
  and a real path to the "formal verification / trust-vs-guarantees" discussion
  (reviewer items #3, #4).

---

## Decisions to pin before building (write these as ADRs first)

1. Multi-aircraft coordination model.
2. IPS level/importance function.
3. RNG scheme (per-particle spawning).
4. How much of BlueSky's dynamics to *extract* as pure vs. *wrap* — the one real
   engineering risk. Prototype `step_dynamics` against the current BlueSky M600 behaviour
   and validate it matches before building anything on top.

---

## First milestone (resist over-planning)

The smallest thing that runs end-to-end, cleanly:

> one pairwise encounter → your own `state` + extracted `step_dynamics` + one CDR method
> + plain-MC IPR → matching a known result from the current code.

Only *then* add multi-aircraft, and only *then* the IPS estimator. **Do not build the
estimator framework before one clean encounter runs.** If `step_dynamics` can't be made
to match BlueSky's M600 model in a few days, that's the signal to reconsider scope — not
to push through.

---

## Deliberately deferred

IPS variance-theory refinement; formal verification; **replacing the BlueSky engine** (the
`step_dynamics` boundary makes it possible — do it only when there's a concrete reason like
speed, licensing, or missing physics, not on spec); any performance work (Rust, etc. —
only if MC/IPS turns out too slow, measured, not assumed). And carry forward the two bug
fixes' *understanding*: recovery state is per-particle, and every stochastic component
takes its own RNG.

---

## Open source & community contribution (a design driver, not an afterthought)

The intent is to **open-source this** and invite the community to contribute — new
conflict **detection**, **resolution**, and **recovery** algorithms; new **environments**
(encounter types, airspace / traffic models); new **noise / CNS models** and
**estimators**. This is *why* the architecture looks the way it does, not a late add-on:

- **The interface is the contribution surface.** A new CDR algorithm is a pure function
  matching the layer's signature; a new environment is an `advance` / `level` /
  `is_terminal` triple; a new estimator plugs over that same interface. A contributor adds
  a file, not a fork of the core.
- **Legibility (#3) and the vault (#4) are for contributors, not just you.** "Name it like
  the paper" and small pure functions are what let a stranger read an algorithm and trust
  it; the vault is the onboarding.
- **Reproducibility is the contribution contract.** The golden anchors + validation ladder
  let a contributor prove a new algorithm/environment against the same reference everyone
  else uses — so you can accept a PR on *evidence*, not faith.

Design *toward* this, but build it only when there's a second contributor (don't build a
plugin framework for an audience of one — same over-build trap as `lesson-learnt.md`). The
eventual shape:

- a **stable, documented signature** per layer (what a contribution must match) and a
  **registry** so an algorithm is selectable by name in a config;
- a **contributor guide** + a worked **example contribution** (a toy CDR method, end to
  end) as the template;
- a **license**, and a light **CI gate** a PR must pass (reproduces its own claimed result,
  doesn't break the golden anchors);
- **citation / attribution** guidance — this is a research community, so academic credit
  for a contributed algorithm matters as much as the merge.

The first public release can be minimal: the core + one example + the validation harness.
The registry and CI follow the first outside PR.
