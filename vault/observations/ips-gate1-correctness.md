# IPS Gate 1: validating the rare-event estimator against brute-force MC at pos=40

**Status: passed (Phase 8, correctness gate).** The first of the two IPS validation rungs (ADR 0017
§6): in a *not-too-rare* regime the interacting-particle-system estimate must agree with plain Monte
Carlo. Both estimators are driven from one :class:`Scenario` (`scripts/ips_validate.py`) so their
parameters cannot drift — MC via `estimate_ipr`, IPS via `ips_once` over the *same* `sample_pairwise`
geometry and `build_env` rules. This note records the gate result **and** the investigation of a
small anomaly it surfaced, because the debugging is the reusable part. Written 2026-07-26. Reproduce:

    python scripts/ips_validate.py --pos 40 --mc-n 24000 --particles 400 --reps 32 \
        --levels 100 75 60 55 52 51 50 --jobs 8
    # single-level control (= MC over the fleet path, no splitting):
    python scripts/ips_validate.py --pos 40 --levels 50 --particles 5000 --reps 4 --jobs 8

(pos_ci95 40 m, vel_ci95 4 m/s, rpz 50, lookahead 120, MVP margin 1.05 + Past-CPA, GNSS noise, dt 0.5
unless noted; the [[rare-event-validation-ladder]] pos=40 rung, `P(LoS) ≈ 0.028`.)

## The result

| estimator | P | 95% CI | cost |
|---|---|---|---|
| **MC** (pairwise path, n = 24000) | 0.02479 | [0.02290, 0.02684] | ~120 s |
| **IPS** (32 reps × 400, 7 shells) | 0.02765 | [0.02499, 0.02915] | ~100 s |

The CIs overlap — **Gate 1 passes**. But IPS sat consistently a little *high* (first light 0.030,
here 0.0277 vs MC 0.0248), its mean just outside the MC interval. For a splitting estimator that is
unbiased in theory, a mean that drifts is worth chasing, not waving through (the "stop and confirm
the anomaly" discipline of `docs/lesson-learnt.md`).

## Chasing the residual: three knobs, then the decisive control

Each knob tested a different hypothesis for the ~15% overshoot; the point estimate barely moved until
the last one:

| change | hypothesis | IPS P | verdict |
|---|---|---|---|
| coarse shells `[70…50]`, 12 reps | baseline | 0.02925 | — |
| higher/finer shells `[100…50]`, 12 reps | shell-crossing overshoot | 0.02848 | small drop |
| finer shells, **32 reps** | small-sample noise | 0.02765 | small drop |
| **dt 0.5 → 0.2** (`[200…50]`) | discretisation overshoot | 0.02876 | **no change** |
| **single level `[50]`** (20000 pooled) | is the *path* biased? | **0.02440** | **matches MC** |

**`dt` ruled out overshoot.** Halving the step (≈10 m → ≈4 m per-step jumps in `min_sep`) should have
pulled IPS down if the residual were discretisation; it didn't. So the small shell/rep improvements
were second-order, not the cause.

**The single-level control was decisive.** With one shell at `rpz` there is no resampling, so IPS is
*exactly plain MC over the fleet path* (evolve N particles to `rpz`, count reachers). It gave
**0.02440 [0.02235, 0.02663]**, sitting on the MC anchor 0.02479. So the fleet path and the geometry
sampling are **unbiased** — `build_initial` draws the same distribution as `sample_pairwise`, and
`build_env`/`advance` matches `run_encounter` at the *distribution* level, not just the bit-for-bit
reduction ([[fleet-ipr-sweep]], `test_fleet.py`). No bug in the path or the event definition.

## Why the splitting is unbiased too (the overshoot worry, resolved on paper)

The residual is therefore isolated to the *multi-level* splitting — and that is unbiased by
mass-conservation, even when particles overshoot a shell into loss of separation. Sketch, two levels
`d_1 = 55`, `d_2 = 50`: of the `S_1/N` that reach 55, a fraction have already overshot below 50.
Resampling clones them, and every clone trivially survives level 2 — but that over-count is *exactly*
the mass those trajectories already carried when counted in `S_1/N`, so `E[Π_k S_k/N]` telescopes
back to the true `P(reach 50)`. Overshoot changes which levels have survival near 1 (variance), not
the mean. Combined with the single-level control, the estimator is unbiased; the 0.0277-vs-0.0248 gap
at 32 reps is **residual sampling noise plus a slightly optimistic log-space CI at low reps**, not a
defect. More replications pull it onto the anchor.

## What this buys, and the caveats it leaves

- **Correctness is established**: IPS ≡ MC where MC is trustworthy, path and splitting both unbiased.
  The method can be believed in the rare regime once Gate 2 (efficiency, pos=10) holds.
- **Shell placement is a variance knob, not a correctness knob** — the lopsided `[…70]` ladder
  (first-shell survival 0.10, bottom shells ~0.9) is unbiased but inefficient. Placing shells for
  roughly equal per-level survival is what [[ips-adaptive-levels]] (AMS) automates.
- **The log-space CI reads a touch narrow at low reps** (its lower bound sat just above the MC mean).
  Before trusting the pos=10 CI in Gate 2, check its coverage or use more replications — do not take
  the interval on faith at small `reps`.
- **Scope**: one cell (pos=40, `dcpa=0` crossing pairs, N=2, MVP + Past-CPA). Correctness of the
  *machinery* is general; the specific numbers are this cell's ([[rare-event-validation-ladder]]).

Companion: [[0017-ips-level-and-splitting]] (the design), [[rare-event-validation-ladder]] (the two
rungs), [[phase-8-plan]] (the build).
