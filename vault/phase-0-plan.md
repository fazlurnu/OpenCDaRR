# Phase 0 plan — scaffolding

The scaffolding milestone (`how-to-step-by-step.md` Step 0). This is a **living checklist**, not a blueprint — kept deliberately thin per `lesson-learnt.md` (the 36 KB trap). We build the files **one at a time**; you review each diff, then we tick its box here. A box is checked only when the file exists, is understood, and its check passes.

**Exit gate for the whole phase** (from how-to Step 0): imports work · a trivial test runs green · vault folders exist · one RNG/seed convention is chosen and reproducible · one config format is chosen. Nothing more. Then we go to Step 1 (`step_dynamics`), the make-or-break.

---

## Scope — what Phase 0 is and is NOT

**Is:** the minimum seam that lets Step 1 begin — a package that imports, the RNG
convention (load-bearing, hardest to add later, `design-philosophy.md` #3), the one owner
of state as a clonable type (`design_brief.md` spine, philosophy #2), tooling that enforces
type hints, and the vault.

**Is NOT** (deferred, with reasons — this *is* the pushback, recorded so it doesn't rot):

- [ ] `cns/ cdr/ environment/ estimator/ scenario/ analysis/` packages — **deferred to the
  step that fills each with real code.** Empty stubs now = the "built infrastructure
  instead" anti-pattern (`lesson-learnt.md`), against philosophy #10 (tracer bullets before
  frameworks). The target layout is *documented* below, not *created* yet.
- [ ] `config.py` (YAML → typed dataclass loader) — **deferred to Step 2**, the first
  `config + seed → result` run. Phase 0 only *locks the format* (YAML). A loader with no
  caller is code written ahead of need.
- [ ] `dynamics.py` / `step_dynamics` — that **is** Step 1, not Step 0.
- [ ] ADRs for coordination model / IPS level function / BlueSky-extraction extent —
  written **when their step needs them** (v0.3, v0.4, Step 1). Only the RNG ADR is a Phase 0
  decision. Writing all four now is planning-ahead (`lesson-learnt.md` meta-lesson #4).

If any deferral looks wrong to you, push back before we start the item.

---

## Target layout (documented, not built in Phase 0)

Mirrors `design_brief.md` → Architecture. Directories are created lazily, per step.

```
opencdarr/
  __init__.py       [Phase 0]  package root
  rng.py            [Phase 0]  SeedSequence.spawn convention — one place, documented
  state.py          [Phase 0]  the clonable world/aircraft state (certain fields only)
  config.py         [Step 2]   YAML -> typed dataclass
  dynamics.py       [Step 1]   step_dynamics — extracted M600 kinematics
  cns/              [Step 3]   noise / comms, pure fns
  cdr/              [Step 2-3] cd / cr / crr — detection / resolution / recovery
  environment/      [Step 2/4] assembles advance / level / is_terminal
  estimator/        [Step 2/6] MC now, IPS later
  scenario/         [Step 2]   encounter generators
  experiment.py     [Step 2]   run_one_experiment + provenance card
  analysis/         [Step 7]   plotting + validation ladder
tests/              [Phase 0]  starts with one smoke test
vault/              [Phase 0]  this file lives here
```

---

## Checklist

Each item: **path · purpose · design justification · what goes in · check · relations.**

### A. Repo & tooling

- [x] **`pyproject.toml`**
  - *Purpose:* package metadata + dependencies + the tooling that enforces our standards.
  - *Design:* pin `numpy` (RNG + arrays) and dev tools `pytest`, `ruff`, `mypy`. Configure
    `mypy` in **strict** mode so *type hints everywhere* is enforced by the machine, not by
    willpower — this is a standing project rule, not just a Phase 0 nicety. `ruff` for lint.
    No `pyyaml` yet — config loader is Step 2.
  - *Check:* `pip install -e .[dev]` succeeds; `mypy opencdarr` and `pytest` both run.
  - *Relations:* enforces the type-hint rule that governs every later file; supports
    philosophy #4 (reproducibility — pinned deps).

- [x] **`.gitignore`**
  - *Purpose:* keep junk and generated artifacts out of history.
  - *Design:* ignore `__pycache__/`, `.DS_Store` (already present in the repo), `*.egg-info`,
    `.venv/`, `.mypy_cache/`, `.ruff_cache/`, and a `results/`-style output dir. Results are
    *regenerated from config + seed + hash* (philosophy #4), so they are not source.
  - *Check:* `git status` (you run it) shows no junk staged.
  - *Relations:* you own all git (see memory `user-handles-git`) — I only maintain this file.

- [x] **`README.md`**
  - *Purpose:* one-screen orientation for a newcomer / future contributor.
  - *Design:* what OpenCDaRR is (one paragraph from `design_brief.md` Goal), how to install
    and run the test, and pointers to `docs/` (the four companion docs) and `vault/`. Keep it
    short; the vault is the real onboarding (`design_brief.md` #4, open-source driver).
  - *Check:* the install + test commands in it actually work when followed literally.
  - *Relations:* links out to `docs/design_brief.md`, `docs/how-to-step-by-step.md`, this vault.

### B. Vault

- [x] **`vault/phase-0-plan.md`** — this file. *Check:* it exists and we agree on it.
- [x] **vault skeleton** — `decisions/ derivations/ observations/ algorithms/ papers/
  experiments/`, each with a `.gitkeep`.
  - *Design:* the exact structure from `design_brief.md` #4. Folders links *to the code*, so
    they exist from the start even while empty. Gate item: "vault folders exist."
  - *Check:* the six directories exist and are tracked.
  - *Relations:* every later ADR / derivation / provenance card lands here.
- [x] **`vault/decisions/0001-rng-per-particle-spawn.md`** (ADR)
  - *Purpose:* record *why* the RNG scheme is `numpy.random.SeedSequence.spawn()`,
    per-particle, no shared/global RNG.
  - *Design:* the one Phase 0 decision that is load-bearing and referenced everywhere. States
    the rule, the alternative rejected (shared/global RNG — the ADSL bug), and the
    consequence for IPS (clones must explore *independent* futures). Short.
  - *Check:* you read it and agree it captures the decision.
  - *Relations:* justifies `rng.py`; ties to `design_brief.md` IPS (#2), philosophy #3,
    `lesson-learnt.md` (ADSL shared-RNG bug). Anchors the future IPS estimator (v0.4).

### C. Core seams

- [x] **`opencdarr/__init__.py`**
  - *Purpose:* make `opencdarr` an importable package (the gate's "imports work").
  - *Design:* minimal — package docstring + `__version__`. No re-exports yet (nothing to
    export). Naming: `opencdarr`, close to the domain (philosophy #6).
  - *Check:* `python -c "import opencdarr"` succeeds.
  - *Relations:* root for every module in the target layout.

- [x] **`opencdarr/rng.py`**
  - *Purpose:* the single, documented place that turns a seed into reproducible,
    independent substreams.
  - *Design:* thin wrapper over `numpy.random.SeedSequence` / `default_rng`, exposing
    (a) `root_rng(seed)` and (b) `spawn(seq, n)` for per-particle children. Fully type-hinted.
    Docstring states the stream-layout contract so it is *documented, not implicit*
    (`design_brief.md` reproducibility). Pure: no module-level RNG, nothing global
    (philosophy #2, #3).
  - *Check:* covered by the smoke test — same seed → identical draws; two spawned children →
    *different* independent draws.
  - *Relations:* implements ADR 0001; the backbone of IPS particle cloning (v0.4,
    `design_brief.md` #2); every stochastic module later takes an rng from here.

- [x] **`opencdarr/state.py`**
  - *Purpose:* the one owner of state — plain, clonable data (the spine of the whole design).
  - *Design:* a frozen `@dataclass` (or NumPy-backed) holding **only the fields we are certain
    of**: horizontal point-mass kinematics — `lat`, `lon`, `trk` (track/heading), `gs`
    (ground speed), and an id/index. **No** CDR/recovery/particle fields yet — those are added
    in the step that needs them (Step 2+/IPS), to avoid guessing structure before
    `step_dynamics` (Step 1) tells us what it needs (philosophy #10, #12; anti-over-build,
    `lesson-learnt.md`). Frozen → clonable-by-copy, which is *why* IPS is possible
    (`design_brief.md` spine) and directly fixes the KI-1 singleton class of bug.
  - *Check:* smoke test — an instance is immutable, copyable, and a mutation produces a new
    object (no aliasing); mypy passes on its type hints.
  - *Relations:* consumed first by `dynamics.py` (Step 1: `step_dynamics(state, cmd, dt) ->
    state`); grows per-particle CDR/recovery state later (`design_brief.md` Architecture →
    state). Embodies philosophy #2 ("one owner of state").

### D. Test

- [x] **`tests/test_smoke.py`**
  - *Purpose:* the trivial-but-real gate test — proves the seams work and the two
    load-bearing properties hold.
  - *Design:* three tiny tests — (1) `import opencdarr` works; (2) RNG reproducibility +
    substream independence (from `rng.py`); (3) `state` is immutable/clonable. This makes
    *reproducibility a tested feature from day one* (philosophy #4), not an afterthought.
    Green here is necessary, not sufficient (how-to Part A #5).
  - *Check:* `pytest` is green; and — per the how-to red-flags — we can make each test fail on
    purpose (e.g. break the seed) to prove it bites.
  - *Relations:* the seed of the golden/anchor test discipline (`lesson-learnt.md` "freeze a
    reference"), repointed at new clean code; grows into the validation ladder (Step 2+).

---

## Relations to the companion docs

- `docs/design_brief.md` — the spine (state you own + BlueSky-as-library) is why `state.py`
  and `rng.py` exist and why the package tree is shaped as above.
- `docs/design-philosophy.md` — the per-file *design* notes cite its numbered principles;
  `pyproject.toml` (strict mypy) enforces the type-hint rule mechanically.
- `docs/how-to-step-by-step.md` — this file **is** Step 0; the exit gate is its Step 0 gate;
  Step 1 (`step_dynamics`) starts only once every box here is checked.
- `docs/lesson-learnt.md` — the Scope/deferred section is a direct application of its
  meta-lessons (don't build infrastructure ahead of need; make the deferral explicit).

## Old-code references (for later steps, not Phase 0)

Recorded so we do not lose them. We **read** these to re-derive equations and to record a
reference trajectory — we do **not** port them (`lesson-learnt.md`: "don't port, rebuild").

- Dynamics to extract (Step 1): motion is `bs.sim.step()` on `bs.traf` in
  `CDaRR_git/envs/pairwise_conflict.py`; the M600 kinematics live inside BlueSky's perf model.
- Equations to re-derive (Steps 2-3): `CDaRR_git/sim_models/` — `cd_statebased.py` (CPA
  detection), `cr_mvp.py`, `cr_vo.py` (resolution), `crr_resumenav_*.py` (recovery criteria),
  `cns_adsl.py` / `noise_*.py` (CNS).
</content>
