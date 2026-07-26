# Phase 8 plan — Rare-event collision risk via the Blom–Bakker IPS

v0.4 of the roadmap and the reason the whole stack was shaped the way it is
([[0004-layered-directed-design-for-multiaircraft-and-ips]]): estimate a **rare collision/LoS
probability, reported with a confidence interval**, in a regime where plain Monte Carlo cannot. The
[[rare-event-validation-ladder]] makes the need concrete — at `pos_ci95 = 10 m` the LoS rate is
`≈ 4.7×10⁻⁴`, and a single 10 000-encounter MC run there returned **zero events**. The method is the
**Blom–Bakker interacting particle system** (importance splitting with resampling), run over the
Step-5 estimator interface — *no* changes to the CDR core or the environment.

This is **not** the consolidated metrics/logging work (Phase 9), and **not** a smarter importance
function for simultaneous multi-aircraft conflict (a later contribution). Phase 8 is the estimator on
top of the environment, validated against MC before it is trusted.

Same working style as Phases 2–6: a **tracer bullet first** (design-philosophy #10) — the thinnest
IPS that runs end-to-end and agrees with brute-force MC in a not-too-rare regime — then push into the
rare regime. Read each diff; each rung green before the next.

---

## Design settled — [[0017-ips-level-and-splitting]] (ratified 2026-07-26)

The full "why" is the ADR; in one line each:

1. **Importance = running-minimum separation** (`FleetState.min_sep`) — monotone, so shell crossings
   are one-way (instantaneous min-sep is non-monotone; rejected).
2. **Fixed-effort splitting** — N particles per level, `p̂_k = S_k/N`, `P̂ = Π p̂_k`, **no weights**;
   a clone is a shared `FleetState` + a fresh `FleetStreams` (ADR 0001).
3. **`is_terminal` unchanged** (`done_timeout`/`t_max`) — no absorbing past-CPA kill (removes a
   re-approach bias; the extra sim is cheap vs MC).
4. **Initial cloud samples geometry; splitting acts on forward noise** — so IPS estimates the *same*
   `P(LoS)` MC does (apples-to-apples validation).
5. **CI from R independent replications** (log-space) — within-run particle spread is invalid
   (particles interact).
6. **Two-rung gate** — pos=40 correctness, pos=10 efficiency.

## The original open questions — now resolved

This file began as three questions; the ADR answers each:

- *"When can we completely stop the sim?"* → `is_terminal` unchanged; a particle is dropped when it
  reaches `done_timeout`/`t_max` without crossing the next shell. No new terminal logic (ADR §3).
- *"Benchmark against classical MC."* → the two-rung gate: IPS ≈ MC at pos=40 (correctness), IPS
  tight where MC's CI explodes at pos=10 (efficiency) (ADR §6, [[rare-event-validation-ladder]]).
- *"What is the variance of the estimate?"* → the across-replication sample variance from R
  independent IPS runs, reported as a log-space CI (ADR §5).

## Done — the estimator interface (Step 5)

`run_fleet`'s loop is now a driver over `advance / level / is_terminal` (`opencdarr/fleet.py`):
`FleetEnv` (fixed rules, shared), `FleetState` (the particle — deeply immutable, so a clone is
reference-sharing), `FleetStreams` (per-particle RNG, re-spawned on clone). Behaviour-preserving (the
`run_fleet == run_encounter` reduction tests and `fleet_trace` anti-drift are bit-identical), with
`tests/test_fleet_interface.py` locking advance-purity, clone-safety, and `level`.

## Build rungs (the tracer bullet)

- [ ] **Promote the probe** to `scripts/ips_validation_probe.py` (seeded, `--pos`/`--n` args) so the
  MC anchor regenerates from one command — the [[rare-event-validation-ladder]]'s loose end.
- [ ] **`opencdarr/ips.py`** — fixed-effort splitting over `FleetEnv`: fixed geometric shells on
  `state.min_sep`, evolve-to-next-shell-or-`is_terminal`, resample-to-N with fresh `FleetStreams`,
  `P̂ = Π (S_k/N)`. `N`, the shell sequence, and `R` are config knobs.
- [ ] **Replication CI** — R independent IPS runs on independent seed subtrees; mean + log-space CI.
- [ ] **Gate 1 — pos=40:** IPS mean±CI agrees with `estimate_ipr` (correctness).
- [ ] **Gate 2 — pos=10:** IPS returns a tight CI at fixed budget where MC reads 0 (efficiency).
- [ ] **Report `P ± CI`**, never a bare number (design-philosophy #4); provenance card for the run.

## The gate (both required)

pos=40 proves the machinery is **unbiased**; pos=10 proves it is **efficient**. Passing pos=40 alone
is not victory — at `P ≈ 0.03` splitting barely helps. Only trust a rarer regime after pos=40 is
green.

## Deferred (own ADRs when they land)

- **Adaptive multilevel splitting (AMS)** — [[ips-adaptive-levels]]: quantile shells, no
  hand-tuned spacing and no `S_k = 0` collapse. The robustness upgrade once fixed-shell IPS passes.
- **tCPA-informed importance** — projected miss rather than current running-min; a variance-reduction
  contribution of its own.
- **Collision-radius rare event** — replace `d_m = rpz` (LoS) with a physical collision radius, where
  MC is outright infeasible and IPS is essential.
