# ⚠️ IMPORTANT: the pilot cannot see where the ladder works — fixed shells below its horizon are extrapolation

**Status: accepted limitation — decided 2026-08-07 to leave the ladder as it is.** The
per-condition pilot ladder (`scripts/validation/campaign.py`, `_ladder_for`) stays: geometric in
metres (each shell closes 62 % of the remaining distance to `rpz`), calibrated from a
200-encounter pilot. This note records *why* that ladder mis-spaces its deep shells and its
anchor-rung shells — both visible in campaign output — what a probability-spaced ladder would and
would not fix, and why the real alternative is adaptive multilevel splitting (AMS), deferred by
ADR 0017 and still deferred now. `examples/observation/pilot_horizon.ipynb` demonstrates the
phenomenon end to end on a pairwise encounter at `pos_ci95 = 20`: the 200-encounter pilot's
horizon lands at 61.4 m, 6 of the ladder's 13 shells fall below it, and the survival sequence
turns erratic exactly there (dip to 0.28 at the horizon, replication spread 0.18–0.93 in the
shaded region) while the shells above it stay smooth.

## The horizon is an order-statistics fact

The pilot is 200 plain-MC encounters, one `min_sep` each. An empirical CDF on 200 samples has
data down to roughly its smallest observation — in expectation the ~1/200 ≈ 5e-3 quantile — and
**zero observations below it**. So the pilot can *place* a shell wherever
`P(min_sep <= d) >= ~5e-3`, and can only *extrapolate* deeper. At the 5 m rung the event sits
near 1e-5: a 0.5-survival ladder needs ~log2(1/p) ≈ 17 shells, the pilot's horizon covers the
first ~log2(200) ≈ 7–8, and the remaining nine — the hard half — are placed by a formula about
territory no pilot encounter ever visited.

The horizon is economic, not incidental: a pilot able to resolve the 1e-5 region would need
~10⁵–10⁶ encounters, at which point it *is* the MC estimate and splitting is pointless. Any
calibration cheap enough to afford is blind exactly where splitting operates.

## The deep region has structure the body does not reveal

Below ~55–60 m the cloud is inside the resolver's protected radius and every metre is contested:
the conditional probability of gaining ground is set by MVP fighting the noise, not by the
unconditional spread the pilot measured. The signature is a survival *dip then rise* — hard
going at the resolver's barrier, then easy metres once it has effectively lost. Campaign rows
show it directly (ring n=6, 10 m rung, 1500 m ring):

    survival [0.547, 0.484, 0.317, 0.196, 0.348, 0.444, 0.616, 0.616]

Nothing in the upper CDF predicts where that barrier sits or how steep it is — it is a property
of the dynamics *conditioned on being deep*, and it moves with geometry, noise rung, and the
resolver's margin.

## Both mis-spacing pathologies, in one ladder

- **Below the horizon — starved shells.** 5 m-rung ladders (2026-08 campaign, 900 m ring) ended
  in survivals of 0.114, 0.041, 0.03; two pairwise replications collapsed outright; a
  600-particle probe collapsed at the second-to-last shell with survival 0.0. A blind shell
  straddling the barrier is a coin the cloud keeps losing.
- **Above the horizon at the anchor rung — wasted shells.** High-p cells get 9 shells with
  survival 0.5–0.9 (ring n=8 @ 30 m: `0.488 … 0.88, 0.847`) because metres-spacing over-resolves
  where probability is dense. Each such shell is a full `reps × N` = 20 000-particle sweep that
  buys ~0.2 nats of decomposition — a real slice of why IPS wall time stays flat while MC gets
  cheap as p grows (gain 0.09 → 0.68 across the 30 m and 10 m rungs; the crossover sits near
  1e-3).

## What probability spacing fixes, and what it cannot

Placing shells at pilot-CDF quantiles (target ~0.5 conditional survival) fixes only the region
the pilot can see: the anchor-rung over-resolution collapses from ~9 shells to 3–4. **It does
not touch the starved deep shells** — below 5e-3 the quantiles come from a fitted tail, which is
the same guesswork with better manners. The half-step economises where cost no longer hurts
(anchor cells are 100–200 s since the sharding commits) and leaves the collapse-prone region
exactly as it is.

## The full step is AMS, and it is a method change, not a patch

Adaptive multilevel splitting (Cérou & Guyader 2007) places no levels at all: evolve the cloud,
set the next level at the empirical quantile of the running minima (kill the worst fraction,
~half), resample, repeat until `rpz`. The cloud is a 2 000-sample pilot *of the conditional law
at every depth* — the distribution no pre-run pilot can reach — so survival is ~0.5 on both
sides of the resolver barrier by construction. In this codebase that would buy: no collapse
(ADR 0017 §2's failure mode becomes unrepresentable), no pilot phase (`_ladder_for` and its
median-inside-rpz special case exist only to place shells), and per-geometry tuning gone.

It costs three real things, which is why it stays deferred:

1. **The seed-tree contract.** The level count becomes data-dependent; `ips_once` addresses
   `children(evolve_seq, 0, len(levels))` up front and the lockstep driver's bit-identity rests
   on that indexing (ADR 0001). AMS needs iteration-indexed addressing with no known bound —
   doable, but it touches the reproducibility core and its whole test suite.
2. **The unbiasedness claim weakens.** Fixed levels make each replication's product of fractions
   exactly unbiased (`combine_replications` says so and means it). Quantile levels estimated
   from the same cloud give consistency with O(1/N) bias; the exactly-unbiased last-particle
   variant is maximally sequential — hostile to a 100-worker box.
3. **An ADR.** ADR 0017 explicitly accepted fixed shells and deferred adaptive levels; reversing
   that is a stated methods decision for the paper, not a refactor.

**Decision recorded:** stay on fixed pilot-placed ladders. Collapses are visible and honest
(`n_collapsed`, never a silent zero), the anchor-rung waste is bounded, and the deep-shell
spacing has been good enough for every validated rung so far. Revisit if a rung below 1e-6 is
ever needed, or if collapse rates rise past what replication redundancy absorbs.

## Related

- [[rare-event-validation-ladder]] — the rung design this ladder serves
- [[ips-gate1-correctness]], [[ips-gate2-efficiency]] — the fixed-ladder validation record
- [[important-ips-gap]] — the other importance-function limit (comms jumps vs `min_sep`);
  AMS fixes level *placement*, not level *coordinate* — the two gaps are orthogonal
