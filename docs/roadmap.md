# Roadmap — OpenCDaRR

**Milestone-based, not calendar-based.** Versions ship when their gate is green, not on a
date — dates you can't keep just manufacture guilt. Later milestones are *direction, not
promises*; reorder as the research demands.

This is the *trajectory* view (versions, research goals, community). For the engineering
steps *inside* the early versions, see the build order in `how-to-step-by-step.md` Part B.

## Scope boundary (read first)

- **The current paper finishes on the old code** + the two bug fixes (KI-1, ADSL). It is
  **not** built on OpenCDaRR — don't couple a paper deadline to a rebuild (`lesson-learnt.md`).
- **OpenCDaRR is for the *next* paper, and the research after it.**

---

## Toward the next paper

**v0.1 — Tracer bullet.** Own-state core + validated `step_dynamics` + one pairwise
encounter + one CDR method + plain Monte Carlo.
*Done:* an end-to-end run from `config + seed` reproduces a known anchor within MC error.
*(how-to Steps 0–2.)*

**v0.2 — Full CDR under CNS uncertainty.** All three CDR stages (detection, resolution,
recovery) + the CNS noise / comms models, pairwise.
*Done:* each CDR method reproduces its old-code anchor under uncertainty. *(how-to Step 3.)*

**v0.3 — Multi-aircraft.** N-aircraft environment + an explicit coordination model (written
up as an ADR: cooperative / priority / sequential).
*Done:* reduces to the v0.1 pairwise result at N=2. *(how-to Step 4; reviewer item #1.)*

**v0.4 — Rare events.** The `advance` / `level` / `is_terminal` interface + Blom–Bakker
interacting particle system (IPS).
*Done:* IPS agrees with brute-force MC in a *not-too-rare* regime; collision probability is
reported **with a confidence interval**. *(how-to Steps 5–6.)*

> **The next paper is written from v0.1–v0.4:** reproducible CDR robustness under CNS
> uncertainty, extended to multi-aircraft encounters and rare-event collision-risk
> estimation.

**v1.0 — Open.** Public open-source release, **aligned with the next paper's publication**
(the natural citation moment — revisit if you'd rather build in public earlier). Vault
docs, a worked example contribution, a plugin registry, a license, a light CI gate, and
citation guidance.
*Done:* a stranger can add a CDR method through a documented signature and validate it
against the golden anchors.

---

## After the next paper — the research priority

**v0.5 — New recovery criteria in multi-conflict.** The signed-tCPA / balanced-criteria
ideas from `my-observation.md` #14–16 — combining the *divergence* signal (tCPA sign /
Past-CPA) with the almost-parallel *safety* of Probabilistic-FTR, without a naive AND/OR —
evaluated in **multi-aircraft simultaneous conflict**. This is the headline research goal
for the paper after next, and it's cheap now: a new criterion is a new pure function
(CDR is pluggable), and multi-conflict already exists (v0.3).
*Done:* the new criterion runs across pairwise **and** multi-conflict scenarios, benchmarked
against Past-CPA / FTR / Probabilistic-FTR.

---

## Community long game (beyond v1.0)

- **Contributed algorithms** — detection / resolution / recovery — plus **environments**,
  **noise models**, and **estimators**. The interface *is* the contribution surface: add a
  file, validate against the anchors, open a PR.
- **Formal verification / trust-vs-guarantees** thread (reviewer items #3–4, carried
  forward).
- **Pluggable dynamics / OpenAP aircraft** — separate the *dynamics interface* from its
  implementation: `DronePointMass` (the v0.1 M600 model) and an `OpenAPDynamics` behind the
  same `step(state, command, perf, dt) -> state` seam, selectable by config. OpenAP is a
  standalone library (the M600 envelope already comes from its rotor database), so richer
  aircraft need no BlueSky. Richer models grow the state (alt, vertical rate, mass) — a
  deliberate, re-validated change. Build the abstraction only when the *second* model
  (OpenAP) actually arrives, not before (don't frame for an audience of one).
- **Engine replacement** — only if a *measured* reason appears (speed, licensing, missing
  physics). The `step_dynamics` boundary makes it cheap; do it on evidence, not on spec.

---

## How this roadmap stays honest

- A version ships when its **gate is green**, never on a date.
- A version isn't "done" until it's **reproducible** — every milestone carries its own
  validation (an anchor, or the analytical ⊂ MC ⊂ IPS ladder).
- Later milestones are a **direction to reorder freely**, not a contract.

---
*Companion docs:* `design_brief.md` (what) · `design-philosophy.md` (how) ·
`how-to-step-by-step.md` (build steps) · `lesson-learnt.md` (why).
