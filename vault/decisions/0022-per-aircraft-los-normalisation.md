# ADR 0022 — P(LoS) per aircraft (Blom), and no confidence intervals

- Status: proposed
- Date: 2026-08-06
- Deciders: Fazlur Rahman

> **Stub.** The decision is settled; the prose is finalised with the implementation. See
> `TODO-metric-rewrite-parallel.md` (item 1) for the plan and `RETRO-0295f6b-to-5bfbc54.md` for what
> the first attempt got wrong. Related: [[0017-ips-level-and-splitting]],
> [[0018-parallel-ips-scheduling]].

## Context

The estimator reports a per-run loss probability, `P_run = (1/n) Σ 1{K(r) ≥ 1}`. In dense traffic it
saturates: a superconflict pins `P_run ≈ 1`, so it can no longer tell a good resolver from no
resolver at all. It is also not the quantity the reference literature reports, which makes our
multi-aircraft numbers hard to place against it.

## Decision

- Headline **`p_los_ac`** `= (1/(nN)) Σ A(r)` — Blom & Bakker (2015), *Safety Evaluation of Advanced
  Self-Separation…*, JAIS, DOI `10.2514/1.I010243` (per aircraft; Figs. 2 and 4).
- **Keep** `p_los_run` (the old `p_los`, the pairwise-equivalent and the saturation reference) and
  add `mean_k = E[K]`. Drop the bare name `p_los` — every reference states its normalisation.
- IPS measures `p_los_ac` through a survivor **tail leg** (`IPS(tail=True)` by default); the ladder
  gives `p_los_run` natively. Tail precision is reported as `n_lineages`, not `n_particles`.
- **Drop confidence intervals** (`wilson_interval`, the log-space CI, `p_los_lo`/`p_los_hi`). Judge
  estimator agreement on the ratio: within 2×, or 5× at ≤ 1e-4 where the MC anchor is itself thin.
- Rename `IPRResult` → `MonteCarloEstimate` (IPR is now ambiguous between the two normalisations);
  `RareEventEstimate` is left as-is on purpose.

## Consequences

- At `N = 2` the three metrics coincide, so **every pairwise result is bit-identical**. `N > 2`
  artifacts re-run, led by `converging_ring(8)`: `p_los_run` saturates near 1 while `p_los_ac` and
  `mean_k` still resolve — the figure that justifies the change.
- Per-aircraft loss is recorded as a `frozenset` of losing pairs on `FleetState` (so it survives the
  IPS clone), giving both `K` and `A` from one field.
- `λ` (per aircraft per flight hour) is **deferred** — not computed or reported here.
