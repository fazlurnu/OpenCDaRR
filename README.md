# OpenCDaRR

A conflict-detection, -resolution & -recovery (CDR) research platform for airborne traffic.
The aircraft state and the simulation loop live in this codebase as plain, explicit data;
BlueSky is used only as a **library of stateless math**, not as the runtime. The design is
built for three things: **reproducibility, readability, and maintainability** — research code
that must be defensible to reviewers, extended to multi-aircraft conflict, and used for
rare-event collision-risk estimation.

> Status: **v0.1 in progress** — scaffolding and the `step_dynamics` model are complete;
> the pairwise encounter is next. See [`vault/`](vault/) for plans and decisions.

## Install

Using conda (recommended):

```bash
conda create -n opencdarr python=3.11
conda activate opencdarr
pip install -e ".[dev]"
```

Or using venv:

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
```

## Test

```bash
pytest        # run the test suite
mypy          # type-check (strict; type hints everywhere)
ruff check    # lint
```

## Documentation

The **why / what / how** live in [`docs/`](docs/); the linked knowledge vault lives in
[`vault/`](vault/).

- [`docs/design_brief.md`](docs/design_brief.md) — **what** to build (goal, architecture,
  BlueSky-as-a-library spine).
- [`docs/design-philosophy.md`](docs/design-philosophy.md) — **how** to write it (the
  standards; the tiebreaker).
- [`docs/how-to-step-by-step.md`](docs/how-to-step-by-step.md) — the **build order** and the
  process for each step.
- [`docs/roadmap.md`](docs/roadmap.md) — the milestone trajectory (v0.1 → v1.0).
- [`docs/lesson-learnt.md`](docs/lesson-learnt.md) — **why** the project is built this way.

`vault/` is the contributor-facing knowledge base — decisions (ADRs), derivations,
observations, algorithms, papers, and one provenance card per experiment.

## License

[MIT](LICENSE).
