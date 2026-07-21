# Recovery criteria comparison — PastCPA vs FTR vs Probabilistic FTR

**Status: validated, through the real loop.** All three recovery criteria compared head-to-head
on identical seeds/pairs per angle, via `estimate_ipr` → `run_encounter` (not a standalone
calculation). Written 2026-07-20.

Reproduce with [`scripts/ipr_angle_sweep.py`](../../scripts/ipr_angle_sweep.py):

```bash
python scripts/ipr_angle_sweep.py --recoveries pastcpa ftr probabilistic_ftr \
  --angles 2 10 45 90 180 --n 400 --jobs 4
```

## Scenario

Fixed-crossing-angle pairs, `dcpa = 0` (worst-case miss distance), swept over crossing angle
`dpsi ∈ {2°, 10°, 45°, 90°, 180°}`. 400 independent noise realisations per angle per criterion,
same seeds reused across criteria for a direct comparison (not three separately-noisy runs).

| parameter | value |
|---|---|
| resolver | MVP, margin 1.05 |
| rpz | 50 m |
| detection lookahead | 120 s |
| spawn time-to-LoS | 180 s |
| speed | 10.2889 m/s (~20 kt), M600 |
| GPS noise | pos_ci95 = 10 m, vel_ci95 = 1 m/s |
| broadcast cadence | 1 Hz (`broadcast_interval = 1.0 s`) |
| `ProbabilisticFTR` | `prob_threshold = 0.999`, `ktheta = 256` |
| intent-sharing | **off** (`share_intent = False`) |

Intent-sharing being off matters: `FTR`'s and `ProbabilisticFTR`'s second criterion never sees a
*declared* desired velocity here — every result below exercises the **onset-velocity fallback**
([[loop-communication-integration]]'s `PairMemory`, i.e. the intruder's perceived velocity when
the pair first became active, used as an inferred stand-in for intent). This comparison is also,
incidentally, an end-to-end check that the fallback works across a full angle sweep, not just the
unit tests.

## IPR

| dpsi | PastCPA | FTR | Probabilistic FTR |
|---:|---:|---:|---:|
| 2° | 0.9550 (18/400 LoS) | **1.0000** | **1.0000** |
| 10° | 0.9950 (2/400 LoS) | **1.0000** | **1.0000** |
| 45° | 1.0000 | 1.0000 | 1.0000 |
| 90° | 1.0000 | 1.0000 | 1.0000 |
| 180° | 1.0000 | 1.0000 | 1.0000 |

## Median CPA

| dpsi | PastCPA | FTR | Probabilistic FTR |
|---:|---:|---:|---:|
| 2° | 52.9 m | 72.4 m | 97.3 m |
| 10° | 80.6 m | 78.0 m | 133.0 m |
| 45° | 225.2 m | 73.5 m | 109.2 m |
| 90° | 245.7 m | 74.7 m | 102.3 m |
| 180° | 238.0 m | 71.7 m | 100.0 m |

## Reading it

**PastCPA is purely reactive** — it only resumes once the pair is *already* diverging, with no
forward check. Two consequences, both visible above:

- **Near-parallel (2°/10°) it is measurably less safe** — 18 and 2 LoS out of 400, the only
  nonzero LoS counts in the whole comparison. A near-parallel pair's divergence signal is weak and
  noise-sensitive (the whole subject of [[near-parallel-ipr-inversion]]), so "wait until clearly
  diverging" is a fragile criterion exactly where it matters most.
- **At real crossings it massively over-holds** — median CPA balloons to 225–246 m at 45°/90°/180°,
  3× or more the other two criteria. Once resolving, momentum carries the aircraft well past a
  safe separation before "diverging" is satisfied, so the maneuver runs far longer than necessary.

**FTR and Probabilistic FTR are both proactive** — "would reverting to nominal clear the
intruder?" — and both are perfect (0 LoS) at every angle, including near-parallel, without the
PastCPA over-hold: resuming the instant it's safe to, not waiting for actual divergence.

**FTR holds a visibly tighter, near-flat margin (~72–78 m) than Probabilistic FTR (~97–133 m) —
by design, not because it's "better."** `ProbabilisticFTR` here runs at `prob_threshold = 0.999`:
it demands 99.9% confidence, under declared CI95 uncertainty, before resuming — a much higher bar
than FTR's plain deterministic "offset > rpz" check, which has no uncertainty margin built in at
all. The gap is the price of that confidence margin, not a sign Probabilistic FTR is less
efficient; a lower `prob_threshold` would close it (see [[probabilistic-ftr-recovery]]'s
uncertainty derivation).

**Why FTR's margin is so flat across angle, unlike PastCPA's:** FTR's target is always the same
geometric quantity (a deterministic, margin-independent-of-approach-angle clearance check), so
median CPA sits close to the same value everywhere; PastCPA's margin is driven by how far the
resolved trajectory happens to travel before divergence is detected, which scales with how far
off-nominal MVP had to push the geometry to escape a given crossing angle in the first place —
hence its steep rise from 2° to 90°.
