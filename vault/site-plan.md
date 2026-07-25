# OpenCDaRR site population plan (v0.3 freeze)

A plan for populating the public handbook (`opencdarr.github.io`) from the vault, alongside the
code freeze. Companion to [[phase-tracker]]; the prose voice follows [[prose-human-voice]].

## The two repos

| Repo | Role |
|---|---|
| `OpenCDaRR` (this one) | Source of truth — code + the `vault/` research record |
| `~/Projects/opencdarr.github.io` | Public **handbook** — a curated *distillation* of the vault (MkDocs Material) |

The site is **not** a vault dump. The vault is an internal research log (dense, dated, cross-linked);
the handbook is a clean, pedagogical reference for someone who has never seen the project. Different
audience, different register — handbook prose is the outward-facing voice of [[prose-human-voice]]
(so it waits on the user's writing sample, same as the README/docstrings).

## What already exists on the site

MkDocs Material, fully configured: light/dark theme + logo, **MathJax via arithmatex** (equations
work), **Mermaid** (diagrams), a GitHub Actions deploy workflow, and a **complete nav**. The content
pages exist but are mostly `!!! note "Draft"` stubs — **`modules/cns.md` (216 lines) is already
filled and is the template** for depth and structure. Only `.gitignore` + the deploy workflow are
committed; everything else is uncommitted WIP.

Nav (already designed): Preface · Installation · Getting Started (Introduction, Architecture,
Examples) · Modules (Dynamics, Autopilot, CD, CR, Recovery, CNS) · Environments (Pairwise,
Multi-aircraft).

## Content map (vault → page)

| Page | Equations (`vault/derivations/`) | Findings + figures (`vault/observations/`) |
|---|---|---|
| Getting Started / Introduction | — | `docs/design_brief.md`, `docs/design-philosophy.md`, README |
| Getting Started / Architecture | — | ADR 0004 (layered directed design), `docs/how-to-step-by-step.md` |
| Getting Started / Examples | — | flagship `scripts/*` (runnable) |
| Modules / Dynamics | `step-dynamics-m600`, `fixedwing-coordinated-turn` | `controlling-dubins-vs-holonomic`, `wind-multirotor-vs-fixedwing` |
| Modules / Autopilot | `l1-guidance` | `trajectory-level-comparison`, `mixed-fleet-dubins-holonomic` |
| Modules / Conflict Detection | `conflict-geometry`, `cpa-detection` | `headon-threshold` |
| Modules / Conflict Resolution | `mvp-resolution` | `multi-intruder-vo-vs-mvp`, `near-parallel-ipr-inversion`, `dorca-directional-selectivity` |
| Modules / Recovery | `ftr-recovery`, `pastcpa-recovery`, `probabilistic-ftr-recovery` | `recovery-criteria-comparison` |
| Modules / CNS ✅ | `gps-noise` | `communication-reception-latency`, `surveillance-hold-as-is`, `surveillance-asymmetric-perception`, `broadcast-phase-offset`, `broadcast-jitter`, `loop-communication-integration` |
| Environments / Pairwise | — | `headon-threshold`, `ipr-under-wind`, `near-parallel-ipr-inversion`, `recovery-criteria-comparison`, `mixed-fleet-daa` |
| Environments / Multi-aircraft | — | `fleet-cooperative-ring`, `fleet-scenarios`, `fleet-ipr-sweep`, `fleet-lossy-ipr` |

27 figures live in `vault/observations/img/`. Some are research-grade multi-panel plots; for the
handbook, decide per figure: **reuse as-is** (clean enough) vs **redraw simpler** (a busy 2×2 that
needs one clear panel). Track that decision in the per-page checklist.

## Per-page template (from `cns.md`)

Each page: **concept** (what it is, one paragraph) → **the model / equations** (lifted from the
derivation, arithmatex `$$…$$`) → **figure(s)** (embedded, captioned) → **what we found** (the
observation's result, distilled) → **in the code** (link to the `opencdarr/` module + a runnable
`scripts/` line). Keep it a reference a newcomer can read top-to-bottom, not a lab notebook.

## The per-page loop (how we populate, one at a time)

1. Read the mapped derivations + observations for the page.
2. Copy the needed figures into `docs/assets/img/` (reuse or redraw per the decision above).
3. Draft the page against the template, in the handbook voice.
4. Preview locally, confirm math + figures + links render.
5. Review with the user, iterate.
6. Commit (site repo).

## Sequence

**Phase 0 — stand it up (before any content):**
- Get the local preview running (`docker-compose up`, or `pip install -r requirements-docs.txt &&
  mkdocs serve`). Confirm arithmatex math and a test figure render.
- Commit the current scaffold as the baseline (it's uncommitted).
- Set up `docs/assets/img/` and a simple way to pull figures from the vault.

**Phase 1 — lock the template + voice (one exemplar):**
- Fully populate **Conflict Resolution** end-to-end (mvp-resolution equations + the VO/MVP figure +
  the key finding). It is core, has a clean derivation, and one good figure. Confirm depth + voice
  with the user before rolling out. (Needs the writing sample.)

**Phase 2 — Modules** (teaching order, each end-to-end):
Dynamics → Autopilot → Conflict Detection → Conflict Resolution (Phase 1) → Recovery. CNS is done.

**Phase 3 — Environments:** Pairwise → Multi-aircraft (the fleet story: cooperative ring →
scenarios → IPR sweeps → lossy IPR).

**Phase 4 — framing pages:** Getting Started (Introduction, Architecture, Examples) once the modules
exist to link into; then Installation + Preface polish.

**Phase 5 — finish:** cross-linking pass, nav check, mobile/dark-mode spot check, deploy.

## What to start with (the answer to "where do I begin")

1. **Stand up the preview + baseline commit** (Phase 0) — nothing else can be checked without it.
2. **One exemplar page: Conflict Resolution** (Phase 1) — locks the template, depth, figure handling,
   and the handbook voice in one place, so Phases 2–4 are repetition, not redesign.
3. Then roll out module by module.

## Decisions (settled 2026-07-25)

- **Figures — redraw.** Handbook-grade, clean, single-purpose plots written fresh into
  `docs/assets/img/` (not the dense research PNGs). The user beautifies them later.
- **ADRs — do not republish.** Reference/keep them in the vault; the site does not mirror them.
- **Hand-written pages only.** No `mkdocstrings` auto-gen; every page authored like `cns.md`.
- **Voice — OpenAP Handbook style** (`opencdarr.github.io/openap-writing-style.md`): "we", calm
  expert, honest about uncertainty, interpret every result with a number, acronyms defined once,
  code-font names, admonition titles as full claims. See [[prose-human-voice]].

## Content-correctness fixes found so far (freeze catch)

- `modules/cns.md` admonition "Own detection uses the true own state" **contradicts the code** —
  `run_fleet`/`run_encounter` decide on the ownship's *noisy* self-fix (`selfs[i] =
  navigation.measure(...)`; loop.py: "both endpoints carry noise"). Correct the page to match, or
  confirm the intended model. (Watch for the same claim wherever navigation/detection is described.)
