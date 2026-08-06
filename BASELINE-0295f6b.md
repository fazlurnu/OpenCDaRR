# Green baseline at `0295f6b` — captured before the rewrite

Captured 2026-08-06, immediately after `git reset --hard 0295f6b`, before any code edit. This is the
reference the rewrite is measured against. Untracked (survives the reset; a `git clean` would remove
it).

## Tests — GREEN

    python -m pytest -q   → 496 passed, 0 failed, 0 skipped   (exit 0)

496 tests collected, all pass. This is the known-good base.

## Ruff — NOT clean under the installed ruff, and that is *version drift*, not a regression

    ruff 0.15.22
    ruff check opencdarr/   → 8 errors   (3 B027, 5 E501)
    ruff check .            → 519 errors (E501 158, E402 136, E702 82, I001 55, B905 34, F401 18, …)

A repo genuinely clean under its own `select = ["E","F","I","UP","B","N"]` (pyproject `[tool.ruff]`,
line-length 99) would not carry 136 E402 + 82 E702 + 55 I001. The commit history's "ruff clean" was
true under the ruff version used then; the system ruff (0.15.22) lints more strictly. There is **no**
pinned ruff in-repo (no pre-commit, Makefile, or CI workflow found).

**Consequence for the gate:** "ruff clean" cannot mean zero here. The gate is **no *new* ruff
findings on the files you touch, vs. this baseline** — or pin the ruff version first. Do not let a
mid-rewrite `ruff check` reporting hundreds of errors be mis-blamed on the rewrite.

## Pairwise invariance oracle — use the in-suite golden anchor

The authoritative N=2 reference is already in the suite and passing, bit-exact:

    tests/test_estimator.py::test_golden_ipr_at_midrange_noise
        (n_los, n_conflict) == (22, 200)
        ipr == 0.89
        median_min_sep == 126.45469556207351   (rel=1e-8)

Noisy config (GnssNavigation), seed 1, 200 encounters, MVP(1.05), PastCPA(bouncing_guard=False). The
rewrite must keep these — with `p_los_run = 1 − ipr = 0.11` and the `ipr` assertion migrated to
`p_los_run`. Keep/extend this test; it *is* the "pairwise bit-identical" guard. Its own docstring
says values change only alongside a deliberate, recorded modelling change.

### Correction: the rescued `0.050 / 53.3` oracle does not hold at `0295f6b`

The number lifted from the old `TODO-metric-rewrite.md` — `configs/pairwise.yaml`, seed 0, 500 enc,
`dpsi=2` → `p_los=0.050`, `median=53.3` — was recorded at a **later** code state (post-`d9067f3`,
which reworked the pairwise path). Reproduced fresh at `0295f6b` it does **not** match:

    pairwise.yaml, seed 0, 500 enc, dpsi=2:
        MVP(1.05):     p_los = 0.0    median_min_sep = 53.60693628115375   (0/500 LoS)
        no resolver:   p_los = 1.0    median_min_sep = 22.255618085493836  (500/500 LoS)

Machinery is sound (MVP takes p_los 1.0 → 0.0). But both are **degenerate** (0 or 1), so neither is a
useful regression oracle — which is exactly why the mid-range golden anchor (0.11) is the one to use.
The plan's Test-oracle #1 has been corrected to point at the golden anchor.

## Net

- Start from a green suite (496 passing).
- Treat the ruff gate as "no new findings on touched files," not zero.
- Guard pairwise invariance with `test_golden_ipr_at_midrange_noise`, not the stale `0.050`.
- The K/A geometry oracle (`swap_ring`, `converging_ring`) is usable from item 1 — the builders
  already exist in `scenario.py`; only the package split is item 3.
