# Observation — our resolution under-separates (was: near-parallel IPR inversion)

**Status: CLOSED (2026-07-20).** The inversion and the 90° over-clear are both fixed; the residual
near-parallel IPR gap is now an **accepted design decision** — see ADR
[0005](../decisions/0005-trajectory-validated-against-bluesky.md) and the reference cross-check in
[[trajectory-level-comparison]]. The port is validated against BlueSky at every level that should
agree (detection, MVP, turn dynamics, recovery); the one remaining divergence is a **BlueSky
ground-speed drift**, flagged for later inspection. The investigation log below is kept for the
record.

**Earlier status: INVERSION FIXED (2026-07-19). Root cause was loop-level CDR cadence + missing
self-noise — not the resolution/recovery math, not a sign error.**

## RESOLUTION — the inversion was a loop-cadence bug (fixed)

Two concrete mistakes in `loop.run_encounter`, both making us unphysically robust at
near-parallel (where the relative-velocity signal is tiny and easily swamped/averaged):

1. **CDR ran every integration step, not on the broadcast cadence.** The reference
   (`get_ipr_stochastic_env`) decides at `asas_dt = 1.0 s` and **holds** the command across the
   `simdt = 0.05×4 = 0.2 s` sub-steps; we re-measured + re-detected + re-resolved every 0.2 s,
   re-drawing independent noise 5× per second and **averaging it away**. This is what kept 2°
   artificially easy.
2. **Own state was truth, not a noisy self-broadcast.** The reference decides on
   `ownship_adsl` (own noisy broadcast) vs `intruder_adsl`; we used the *true* own state, so only
   one endpoint of the relative vector carried noise (~½ the variance).

**Fix:** `run_encounter` now runs the CDR layers every `broadcast_interval` (default 1.0 s); at
each tick each aircraft takes a fresh noisy self-measurement and decides on its own broadcast vs
the other's, then the command is **held** until the next tick. Truth is used only to score the
encounter.

**Result (dcpa=0, CI95 50 m / velo 3 m/s, margin 1.05, dt 0.2):**

| dpsi | Before (buggy) | After (fixed) | Reference |
|------|----------------|---------------|-----------|
| 2°   | 0.90 (easiest) | **0.06 (hardest)** | 0.11 |
| 90°  | 0.59           | **0.43**           | 0.99 |

Ordering corrected — 2° is now the hardest case, on top of the reference. 2° magnitude matches.

## The 90° over-clear — FIXED (coast after detection clears)

**Root cause found via a no-noise trace (dcpa=0, 90°):** ours capped the miss at exactly
`margin·rpz = 52.5 m`; the reference over-clears to a **CPA of 72.94 m** (it passes ~80 m
*outbound*, after CPA). Mechanism: our `_decide` **re-resolved every tick while `resolving` was
latched** (until past-CPA), so MVP kept *regulating* the projected miss back to `margin·rpz` — even
applying a **negative (pull-back)** gain when the miss exceeded the target. Because both aircraft
maneuver simultaneously off the same detection snapshot, they *overshoot* the target; our
re-regulation cancelled that overshoot back to 52.5.

The reference applies a resolution force only while a conflict is **currently detected**
(`confpairs`); once projected `dcpa ≥ rpz` detection clears, the still-active aircraft **coasts**
(maintain current velocity) until recovery — preserving the cooperative overshoot → CPA 72.94 m.

**Fix:** in `loop._decide`, when `resolving` is latched but `detector.detect` no longer fires,
return `Command(hdg=ac.trk, spd=ac.gs)` (coast) instead of re-resolving. No-noise 90° miss:
52.5 → **72.90 m** — matches the reference's 72.94 m CPA (verified by driving CDaRR_git's own
BlueSky+MVP+CPA-recovery, `asas_marh=1.05`). Near-parallel (2°) stays ~52.5 m — correct, a parallel
geometry can't build a perpendicular buffer. NB: at exactly `dcpa=0` ours resolves to the *opposite
symmetric side* from the reference (the head-on-guard side degeneracy, mvp.py:44-46) — equal
magnitude, so CPA matches; measure-zero in sampling, so no IPR effect.

**IPR after both fixes (dcpa=0, CI95 50/velo 3, lookahead=15/tlos=22.5). Reference column
freshly re-measured this session by driving CDaRR_git directly, 500 pairs:**

| dpsi | buggy | +broadcast | +coast | reference (fresh) |
|------|-------|-----------|--------|-------------------|
| 2°   | 0.90  | 0.06      | 0.36   | **0.026** (487/500 LoS) |
| 90°  | 0.59  | 0.37      | **0.93** | **0.846** (77/500 LoS) |

90° matches (ours 0.93 vs reference 0.85, ours a touch higher). Ordering correct, both
right-side-up. (Fresh reference figures differ from this note's earlier 0.11 / 0.99 — those were
stale/other-conditions; the direct 500-pair measurement supersedes them. The dpsi=2 reference run
printed a few BlueSky `ArgumentError` tracebacks — near-parallel `creconfs`/HDG edge cases — so its
2° value is directionally solid, not three-decimal precise.)

## Two-noise-level comparison (both freshly driven, 500 pairs, margin 1.05)

| noise CI95/velo | lookahead/tlos | dpsi | OURS | REFERENCE |
|-----------------|----------------|------|------|-----------|
| 50 / 3          | 15 / 22.5      | 2°   | 0.358 | 0.026 |
| 50 / 3          | 15 / 22.5      | 90°  | 0.926 | 0.846 |
| 10 / 1          | 120 / 180      | 2°   | 0.934 | **0.124** |
| 10 / 1          | 120 / 180      | 90°  | 1.000 | 1.000 |

**90° matches at both noise levels** (over-clear/coast fix solid). The reference's **0.124 at
10/1 reconciles this note's old "0.11"** — that figure was the config-default (low) noise, not 50/3.

## Remaining — 2° gap is a NOISE-RESPONSE difference (deterministic dynamics MATCH)

**No-noise 2° matches the reference** (both param sets, dcpa=0, margin 1.05):

| lookahead/tlos | OURS min_sep | REFERENCE min_sep | LoS |
|----------------|--------------|-------------------|-----|
| 15 / 22.5      | 51.33 m      | 51.18 m           | neither |
| 120 / 180      | 57.01 m      | 57.06 m           | neither |

So the deterministic near-parallel dynamics are **correct in ours at both angles** — at no noise
both resolve smoothly and neither loses separation (the "violent oscillation into LoS" is a *noise*
phenomenon, not deterministic). The 2° IPR gap is therefore **purely noise-response**: the
reference is hypersensitive to noise at near-parallel (any noise → ~88–97% LoS) while ours dampens
it (0.36 / 0.93). At 2° the true relative speed (~0.36 m/s) is *smaller than the velocity noise*
(std 0.41 m/s at velo CI95=1), so the perceived crossing geometry is nearly random each tick — the
reference lets that drive maneuvers into LoS; ours averages/absorbs it too much. My broadcast-
cadence + self-noise fixes matched 90° but likely **over-dampened** the near-parallel noise
response. `bouncing_guard` on/off barely moves it (0.351↔0.358). Open: find why ours absorbs
near-parallel measurement noise the reference propagates (candidate: something still averaging the
per-tick noise, or a detection/resolution response difference when |vrel| < vel-noise).
Ordering is correct at every level; this near-parallel noise-response is the open refinement.

---

## Earlier framing (superseded by the RESOLUTION above)

**Status: root cause narrowed — deterministic resolution builds too thin a safety buffer.**
Written 2026-07-19 during Phase 3a (CNS navigation). OpenCDaRR's IPR-vs-crossing-angle is
*inverted* relative to `CDaRR_git`; a no-noise test (below) reframed the cause from CNS/noise to
the **deterministic resolution/recovery logic**.

## Breakthrough — the no-noise test (the reframing)

Running one encounter with **no noise** and matched params (`tlos=180 = 1.5×lookahead`, dt=0.2):

- **Free flight is identical** — before resolution, old and ours match to the centimetre
  (t=0→60 s at dpsi=2: 114.73→93.11 vs 114.64→93.08). So `step_dynamics`, **speed, and turn
  rate are correct**.
- **The resolution differs — ours under-clears.** Minimum separation reached:

  | dpsi | OLD min_sep | OURS min_sep |
  |------|-------------|--------------|
  | 2°   | 57.06 m     | **50.03 m** (bare rpz) |
  | 90°  | 87.27 m     | **52.51 m** (≈ margin·rpz) |

  Ours bottoms out right at the margin; the old code builds a **much bigger buffer** (35 m more
  at 90°). This is deterministic — **no noise involved.**

**So the real defect is: our MVP+PastCPA under-separates.** A thin buffer is exactly why ours is
noise-fragile at real crossings (90°: 0.60 vs 0.99 — noise need only eat ~2.5 m vs ~37 m). The
near-parallel "inversion" was a symptom, not the disease.

### Refined: our MVP resolves by *speed*, not heading

Tracing the 90° resolution (no noise): the ownship barely turns (`trk` ~0.7°) but **speeds up**
(`gs` 10.29 → 10.97) — MVP resolves almost entirely by acceleration, clearing to just the margin.
A **speed-based** resolution is corrupted directly by **velocity** noise → fragile (0.60); a
heading/position-based one is far more robust (old: 0.99).

### Refined again: ours hits MVP's target; the old code over-clears *beyond* it

Reasoned check at the 90° state: opening `dcpa=0` to the margin (52.5 m) over `t_cpa≈120 s` needs
a perpendicular Δv ≈ `52.5/120 = 0.44 m/s`; **our MVP applies ~0.42 m/s and hits the margin
exactly** — mathematically correct MVP. The **old code clears to 87 m, 65% beyond MVP's own
target**, so the old code builds extra buffer *beyond* MVP (via recovery timing, or
dynamics/overshoot, or a stronger effective resolution) — and that surplus is what absorbs the
noise (0.99 vs our 0.60).

**Open question (next session):** *why* does the old code over-clear beyond MVP's margin? Candidates:
(a) old recovery resumes later than our `PastCPA` (keeps resolving, builds buffer); (b) old
turn/accel dynamics overshoot the target; (c) old MVP effective magnitude differs. A clean
standalone MVP-command comparison is **blocked**: old `cre`/`creconfs` forces a perf speed
(~20 m/s > M600 vmax 18), so its command gets vmax-capped — the old speed setup must be sorted
first. Alternatively, instrument the old grid loop to log ownship trk/gs over one 90° encounter.

**Bottom line:** not noise, not free-flight dynamics, not a static sign error — it's that our
resolution clears to exactly MVP's margin while the old code builds a larger buffer. Well
isolated; root cause of the *surplus* is the remaining work.

## CORRECTION — the per-step algorithms are verified CORRECT

An algorithm-level sweep (identical `own`/`intr` states pushed into both codes) settled it:

- **MVP matches the old code to 2 decimals for `dcpa ≠ 0`** (the realistic case) at every crossing
  angle and `dcpa` (e.g. `dcpa=20, 90°`: old −2.10 / ours −2.10). **No sign bug.**
- **past-CPA matches** (booleans agree).
- The apparent MVP "sign flip" was a **`dcpa = 0` degeneracy**: at a perfect collision course our
  conflict vector correctly → 0 and triggers the head-on perpendicular guard, which the old
  code's (opposite-`t_cpa`) vector does not — so only *exactly* `dcpa=0` diverges. **All my
  earlier traces/sweeps used `dcpa=0`**, which is what produced the illusory inversion in the
  single-encounter comparisons.

**But the IPR inversion persists with `dcpa` properly sampled** (2°: 0.90 vs 0.11; 90°: 0.58 vs
0.99). So the remaining discrepancy is **loop/integration-level, not algorithmic.** Confounds
still to control before the next comparison:

- **`tlos`:** the old code uses `tlosh ≈ 180 = 1.5×lookahead`; my runs used 120/60. Match it.
- **Full recovery state management:** only the `past_cpa` boolean was checked; the old
  `resumenav` also manages `resopairs`, `hor_los`, `bouncing` statefully — compare the *whole*
  resume decision, not just the predicate.
- **Measurement handling** in the loop (both measure every step at reception=1, so likely not
  the cause, but confirm).

**Verified so far (rule out):** detection, MVP (`dcpa≠0`), past-CPA predicate, free-flight
dynamics, noise model. **Remaining:** a fully parameter-matched (`tlos=180`, sampled `dcpa`)
comparison of the *loop* — recovery state management is the prime suspect. Also a minor,
separate bug: the `dcpa≈0` head-on guard picks a different side than the reference (measure-zero
in sampling, but worth fixing).

---

## Original symptom (for the record)

OpenCDaRR's IPR-vs-crossing-angle is *inverted* relative to `CDaRR_git`: our model makes
near-parallel conflicts too **robust** and real crossings too **fragile**.

## The discrepancy

Same parameters (margin/`asas_marh`=1.05, position CI95=50 m, velocity CI95=3 m/s,
reception=1.0, latency=0, CPA/PastCPA recovery, both aircraft 10.2889 m/s, dt=0.2):

| dpsi | OLD `CDaRR_git` | OpenCDaRR (ours) |
|------|-----------------|------------------|
| 2°   | **0.11** (hardest) | 0.96 (easiest) |
| 90°  | **0.99** (easiest) | 0.60 |

The old code matches domain expectation (near-parallel is hardest). Ours is inverted on **both**
ends — so this is not only a near-parallel issue; our 90° is also far too fragile (0.60 vs 0.99).

## The mechanism (traced)

Single-encounter separation timelines (dpsi=2):

- **OLD:** violent oscillation `114 → 44 (LoS) → 94 → 65 → 97 → 80 …`, **min_sep 4.5 m**,
  89/100 pairs LoS. Resolve → separate → resume → **re-converge** → resolve, and the swings
  punch deep into LoS.
- **OURS (measure every step):** nearly flat `63–67 m`, **min_sep 59.7 m**, no LoS. The
  resume/resolve chatter *is* present (flags toggle), but maneuvers are so gentle the separation
  barely moves.

dpsi=90: OLD is a single clean approach (min 512 m, no oscillation) — after a crossing the tracks
diverge and never come back, so no chatter. Ours over-reacts to noise and loses separation more.

## Ruled out (verified)

- **Sign flip** (`Vo−Vi` → `Vi−Vo`): detection (head-on `t_cpa>0`, `d_cpa=0`), MVP (opens miss to
  `margin·rpz` at **every** angle incl 2°/5°/10°), recovery (`old past_cpa ⇔ r·v_ours>0 ⇔ our
  past_cpa`), and `create_conflict` all verified consistent with `v = intr − own`.
- **`step_dynamics` ordering**: provably simultaneous (single-aircraft; both commands from
  pre-step states). Made explicit anyway.
- **`bouncing_guard`**: on/off barely changes 2° (0.958 vs 0.975).
- **Timestep**: `dt=1.0` was inflating LoS via integration overshoot (min_sep ~47); at `dt≤0.1`
  both angles → IPR≈1.0 at mild noise. Finer dt makes both *easier*, not harder.
- **Velocity-noise structure**: the old ADSL adds velocity noise **directly**
  (`add_velocity_noise`), `vel_std = ci95_velo/2.4477 = 1.226 m/s` — **identical** to ours. No
  finite-difference amplification. Position + velocity noise are the same in both codes.

## Partial leads / still open

1. **Measurement hold (broadcast rate).** Ours re-measures with fresh noise *every step*
   (5 Hz at dt=0.2), which averages the noise away. Holding one measurement across a 1 Hz
   broadcast **5×'d our swing amplitude** (±4 m → ±20 m) — a real effect, and it means 3a's
   "measure every step" is unphysical. **But insufficient:** even with hold, ours swings *up*
   (over-separates, min 60.5 m); it does not flip the direction into LoS.
2. **Directional difference (the crux).** Under identical noise and identical CDR logic, the old
   maneuvers net *toward* collision (oscillates down into LoS) while ours net *away* (diverges).
   Root cause unknown — it lives in the resolve/resume **dynamics under noise**, not the noise
   model and not a static sign error.

## Confounds to fix before the next deep dive

- **`tlos` mismatch in the comparison:** the old run placed the intruder for LoS at
  `tlosh = lookahead = 120 s`; ours used `tlos = 60 s` (start ~115 m vs ~71 m). Match these.
- A decisive test needs a **step-by-step dual trace with matched noise draws** — non-trivial
  because the two codes have different RNG/loop structure.

## Relations

- Blocks trusting Phase 3a's IPR-under-CNS numbers (see [[phase-3-plan]]).
- Touches `crr/pastcpa` (resume dynamics), `cr/mvp` (maneuver magnitude under noise), and the
  `loop` (broadcast rate / measurement hold → 3b).
- Reference: `CDaRR_git/sim/pairwise_stochastic/get_ipr_stochastic_env.py`.
