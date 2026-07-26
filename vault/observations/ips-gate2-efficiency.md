# IPS Gate 2: the rare regime — where MC reads zero, IPS still estimates

**Status: passed (Phase 8, efficiency gate).** The second IPS validation rung (ADR 0017 §6): once
[[ips-gate1-correctness]] has shown the estimator is *unbiased* where MC is trustworthy, Gate 2 shows
it is *efficient* where MC is not — it returns a tight, non-zero probability with a CI in a regime
where brute-force Monte Carlo is loose or reads exactly zero. Two cases, one sampled-geometry and one
fixed-geometry, the second the more striking. Written 2026-07-26. Reproduce:

    # sampled crossing angles (the ladder pos=10 rung, lookahead 120):
    python scripts/ips_validate.py --pos 10 --mc-n 10000 --particles 600 --reps 16 \
        --levels 90 80 70 60 56 54 53 52 51 50 --jobs 8
    # fixed 90-deg crossing (lookahead 60): scratch fixed90_mc.py / fixed90_ips.py — the fixed-
    # geometry path is not yet in ips_validate.py (it samples angles); a --dpsi mode would fold it in.

## Case 1 — sampled crossing angles, pos_ci95 = 10 m (`P(LoS) ≈ 4×10⁻⁴`)

The [[rare-event-validation-ladder]] pos=10 rung: random crossing angles (`dpsi ~ U(5°,355°)`),
`dcpa=0`, lookahead 120, rpz 50. At matched wall-cost (~60–130 s):

| estimator | P(LoS) | 95% CI | reads 0? |
|---|---|---|---|
| MC, n = 8–10k | 0.0003–0.0004 | ~[0.0001, 0.0010] (≈9× span) | **yes** — one seed gave 0/10000 |
| IPS, 8–16 reps | 0.00032–0.00041 | ~[0.00018, 0.00047] | no, no collapses |
| MC pooled 30k (rough anchor) | 0.00047 | [0.00028, 0.00078] | — |

The IPS estimates are tight, never zero, and centered on the 30k-MC anchor, while a single MC batch
at matched cost spans ~9× or reads exactly 0. Efficiency here is **modest (~2–3× for equal CI
width)** — at `P≈4×10⁻⁴` MC still catches a few events — but the *reliability* gap is already stark.

## Case 2 — fixed 90° crossing, lookahead 60: MC 30000 reads **exactly zero**

A single fixed geometry (90° crossing, 20 kts = 10.2889 m/s, `dcpa=0`, tlos 90, rpz 50, GNSS
pos_ci95 = 10 m); the only randomness is the noise. This crossing is **safe** — the MC min-sep
distribution never touches rpz:

    min_sep percentiles (30000 runs): p50=153  p25=103  p10=76  p5=68  p1=60  p0.2=56
    min_sep: min = 50.72   max = 461   mean = 164

| estimator | result | cost |
|---|---|---|
| **MC, 30000** | **0 / 30000** → P = 0, CI [0, 1.3×10⁻⁴] — an upper bound, no estimate | 172 s |
| **IPS, 8×2000, 11 shells** | **P ≈ 1.3×10⁻⁴**, CI [5.8×10⁻⁵, 1.8×10⁻⁴], 0 collapses | 127 s |

MC ran 30 000 samples and the event **never occurred** (closest approach 50.72 m), so it yields only
"P < 1.3×10⁻⁴". IPS, at *less* wall-time, turns that into an actual estimate ~1.3×10⁻⁴ with a finite
CI. For MC to *estimate* (not just bound) a 10⁻⁴ event needs ~10⁵–10⁶ samples (~1 h); IPS did it in
~2 min. **Consistency:** if true `P ≈ 1.3×10⁻⁴`, MC's expected count in 30k is ~4 and drawing 0 has
~2% probability — a plausible low draw; IPS's point sits at MC's CI upper bound, intervals overlapping
in [5.8×10⁻⁵, 1.3×10⁻⁴]. The shells were placed straight from the MC min-sep percentiles (survival
~0.4 per shell) — a manual stand-in for what [[ips-adaptive-levels]] (AMS) automates.

## The lesson that bites: collapse biases the estimate **low**

At 500 particles the same fixed-90° run gave `P ≈ 4.9×10⁻⁵` with **3/8 reps collapsed** (a level hit
zero survivors); at 2000 particles, `1.3×10⁻⁴` with **0 collapses**. A collapsed replication returns
`P̂ = 0`, which the mean silently averages in — so **too few particles under-estimate a rare
probability**, and the tell is the collapse count, not the number itself. **N must scale with rarity;
a collapse rate > 0 means "add particles," and ignoring it is a silent low-bias.** A production run
should treat any collapse as a failed run, not a data point — worth a guard in the estimator.

## What Gate 2 establishes, and the honest limits

- **IPS estimates where MC cannot.** For a geometry MC 30k is blind to (0 events), IPS recovers a
  ~10⁻⁴ probability with a CI at lower cost — the whole point of the method.
- **Efficiency scales with rarity.** Modest (~2–3×) at sampled pos=10 where MC still sees events;
  decisive at the fixed 90° case where MC reads 0; and MC-infeasible at the true target — a physical
  collision radius (`P ~ 10⁻⁷`), which this validates the machinery *for* without paying to show it.
- **The exact rare value is order-of-magnitude, not nailed.** At 8 reps IPS may run slightly high
  (as in [[ips-gate1-correctness]]); the robust claim for the fixed-90° case is **~10⁻⁴**, not
  1.35×10⁻⁴ to three figures. More replications tighten it; the collapse guard above must hold.
- **A clean 90° crossing is easy for MVP** (median min-sep 153 m) — the sampled-angle case is harder
  because it mixes in near-head-on and shallow geometries. So this fixed case is *safer*, not a
  representative worst case; a worst-angle sweep is separate.

Companions: [[ips-gate1-correctness]] (correctness), [[rare-event-validation-ladder]] (the ladder and
the two rungs), [[0017-ips-level-and-splitting]] (design), [[phase-8-plan]] (build),
[[ips-adaptive-levels]] (AMS — the shell auto-placement this note tuned by hand).
