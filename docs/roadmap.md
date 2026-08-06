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

## Where things stand

The v0.1–v0.4 gates below are green: the CDaRR stack, the CNS layer, wind, the N-aircraft fleet
environment and the rare-event estimator all run and are covered by tests. No version has been
tagged — `pyproject.toml` still carries `version = "0.0.0"` and the public interface is not frozen —
so read the milestones as capability gates, not as releases. v0.5 and v1.0 are open.

---

## Toward the next paper

**v0.1 — Tracer bullet.** Own-state core + validated `step_dynamics` + one pairwise
encounter + one CDR method + plain Monte Carlo.
*Done:* an end-to-end run from `config + seed` reproduces a known anchor within MC error.
*(how-to Steps 0–2.)*
*Gate green:* `run_encounter` in `opencdarr/loop.py`, `run_one_experiment` in `experiment.py`
(`config + seed + code-hash → result`), and `estimate_ipr` in `estimator.py`. The kinematics are
validated analytically (ADR 0002) and, for the multirotor, against a recorded
[BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) trajectory (ADR 0005). `step_dynamics` itself
became the `Kinematics` interface — see the note under *Pluggable kinematics* below.

**v0.2 — Full CDR under CNS uncertainty.** All three CDR stages (detection, resolution,
recovery) + the CNS noise / comms models, pairwise.
*Done:* each CDR method reproduces its old-code anchor under uncertainty. *(how-to Step 3.)*
*Gate green:* `cd/` (`StateBased`), `cr/` (`MVP`, `VO`), `crr/` (`FTR`, `PastCPA`,
`ProbabilisticFTR`), and the `cns/` stack — one `sense()` call chaining navigation, communication
and surveillance. Trajectory-level agreement with the reference pipeline is recorded in
`vault/observations/trajectory-level-comparison.md`.

**v0.3 — Multi-aircraft.** N-aircraft environment + an explicit coordination model (written
up as an ADR: cooperative / priority / sequential).
*Done:* reduces to the v0.1 pairwise result at N=2. *(how-to Step 4; reviewer item #1.)*
*Gate green:* `run_fleet` / `FleetEnv` in `opencdarr/fleet.py`, with the layered-directed
coordination model in ADR 0004. The N=2 reduction is asserted bit-for-bit — `tests/test_fleet.py`,
and the `--verify-n2` check that `scripts/ipr_fleet_sweep.py` runs by default.

**v0.4 — Rare events.** The `advance` / `level` / `is_terminal` interface + Blom–Bakker
interacting particle system (IPS).
*Done:* IPS agrees with brute-force MC in a *not-too-rare* regime; collision probability is
reported **with a confidence interval**. *(how-to Steps 5–6.)*
*Gate green:* `opencdarr/ips.py` (levels and splitting, ADR 0017) and `opencdarr/parallel.py`
(scheduling across particles and replications, ADR 0018). The agreement gate is written up in
`vault/observations/ips-gate1-correctness.md`; the efficiency gate is
`ips-gate2-efficiency.md`. The interval that gate reported has since been dropped (ADR 0022):
the estimators are compared on the ratio of their estimates instead, and the script that ran
the comparison is superseded by `scripts/validation/`.

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
- **OpenAP aircraft** — the *kinematics interface* is done: `Kinematics` is an abstract base class
  with `Multirotor` and `FixedWing` behind a `step(state, command, perf, dt, wind) -> state` seam,
  and `Performance` is plain data a user can write
  (`examples/03_build_your_own_performance.ipynb`). What remains is a *second family* of models —
  an `OpenAPKinematics`. OpenAP is a standalone library (the M600 envelope already comes from its
  rotor database), so richer aircraft need no
  [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky). Richer models grow the state (alt,
  vertical rate, mass) — a deliberate, re-validated change, and the reason to wait until a real
  OpenAP model arrives rather than generalising for an audience of one.
- **Performance: vectorized SoA step** — reach BlueSky-class speed by advancing a
  Structure-of-Arrays world/particle batch in one vectorized numpy step, instead of a Python
  loop over per-aircraft `AircraftState`. The pure, plain-data design already allows this; the
  math already vectorizes (`geo.forward` uses numpy ufuncs; the turn-rate limiter was
  vectorized in the BlueSky source we extracted from). For rare-event work the highest-value
  axis is batching across **particles** (each a small clonable world), then joblib across
  CPUs — the joblib half of that already shipped as `opencdarr/parallel.py` (ADR 0018), which
  spreads an IPS run over particles *and* replications; the vectorized step below has not. Do it
  on a *measured* profile, not on spec (`design-philosophy.md`: purity wins until a measured
  bottleneck), in three steps:
  1. **Keep the scalar model now** — the legible per-aircraft `Kinematics.step` is the reference;
     do not vectorize prematurely.
  2. **When a profile shows the loop dominates, add `step_batch`** — a SoA step over
     particles/aircraft, behind the same `Kinematics` seam, validated to match the scalar
     reference (the analytical ⊂ … validation ladder).
  3. **If numpy still isn't enough, escalate** — `numba`-JIT the pure functions, and only as a
     last resort the Rust engine (`engine_rewrite_spec`), each on measured evidence.
- **Engine replacement** — only if a *measured* reason appears (speed, licensing, missing
  physics). The `Kinematics` boundary makes it cheap; do it on evidence, not on spec.

---

## How this roadmap stays honest

- A version ships when its **gate is green**, never on a date.
- A version isn't "done" until it's **reproducible** — every milestone carries its own
  validation (an anchor, or the analytical ⊂ MC ⊂ IPS ladder).
- Later milestones are a **direction to reorder freely**, not a contract.

---
*Companion docs:* `design_brief.md` (what) · `design-philosophy.md` (how) ·
`how-to-step-by-step.md` (build steps) · `lesson-learnt.md` (why) ·
`fixedwing-vs-bluesky.md` (the fixed-wing equations of motion, compared) ·
`vault/architecture-dataflow.md` (the architecture as built).
