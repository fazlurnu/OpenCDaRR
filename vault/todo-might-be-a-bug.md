# Might be a bug

Suspected defects and smells found while reviewing [[run-experiment-design]] and building
[[run-experiment-todo]] items 1–4a. Each is *unconfirmed unless marked otherwise* — written down so a
measured, logged issue stays a decision rather than neglect (`design-philosophy.md` #13). Ordered by
how badly it could corrupt a number.

---

## 1. Loss of separation is only sampled at `dt`, so a fast tangential pass can be missed

**Status: confirmed behaviour, impact unquantified. Highest concern of anything here.**

`FleetEnv.advance` measures separation once per `dt` step and sets `los = cur < rpz`
(`fleet.py:316`). Nothing interpolates *between* steps, so a pair whose true minimum separation dips
below `rpz` and back out inside one step is never recorded as a loss.

Observed while writing a test: with `dcpa ~ U(0, dcpa_max)` and `dcpa_max == rpz == 50`, a
non-manoeuvring baseline gives **199/200** losses rather than 200/200 — one encounter drew a
near-tangential `dcpa ≈ 50` and grazed without a sampled point falling strictly inside. That
particular case is benign (the geometry really is marginal), but the mechanism is general and the
error is **one-sided**: it can only *under*-count losses, i.e. flatter safety.

Why it matters more than it looks:

- the miss is worst exactly where closing speed is highest and the chord through the protected zone
  is shortest — the encounters that matter most;
- the ADR-0017 IPS shells are crossed on `state.min_sep`, the same per-`dt` running minimum, so a
  shell crossing can be missed the same way. At a 1e-9 target a systematic one-sided bias is not
  noise;
- it is invisible in every current test, because they all compare `los` against other quantities
  derived from the same sampled trajectory.

Worth checking: compare `los` against the analytic chord test (`dcpa < rpz` and the crossing window
overlapping `[0, t_max]`) over a spread of `dpsi`/`dcpa`/`dt`, and quantify the disagreement rate as
a function of `dt`. If it is non-negligible, the fix is to test the *segment* rather than the
endpoint — closed-form CPA between the pre- and post-step states, which the CPA math in
`cd/statebased.py` already has.

Related: `ips.py` uses `min_sep <= target` (non-strict) while `fleet.py` uses `cur < rpz` (strict).
Measure-zero in floating point, but they should agree on the boundary by construction, not by luck.

## 2. Resolution thrashes ~9× harder at shallow crossing angles

**Status: measured, cause not established.**

Counting resolution episodes (a pair entering `FleetMemory.resopairs`) from a recorded run, at
`pos_ci95=20`, `dcpa=0`, `dt=0.5`, 12 seeds:

| geometry | episodes / encounter | LoS |
|---|---|---|
| `dpsi = 8` (near-parallel) | **24.8** | 0 |
| `dpsi = 90` (crossing) | **2.67** | 0 |

Both are perfectly safe by IPR (1.0 in every case), so the metric set in use today cannot see this at
all. Two candidate causes, not yet separated:

- **genuine oscillation** — the resolver engages, clears, and re-engages, i.e. a real control
  problem at low relative speed. `PastCPA(bouncing_guard=True)` exists to damp exactly this, but
  switching it off barely moved the count (298 → 286 episodes, and *identical* at `dpsi=90`), which
  is itself suspicious — either the guard is not effective here or it is not reached;
- **measurement artifact** — `resopairs` membership is recomputed from *perceived* state every
  broadcast tick, so under noise it can flicker on/off between consecutive ticks without any real
  re-engagement.

Separating those two matters before [[run-experiment-todo]] 4b, because if it is flicker then every
derived count (episodes, unique pairs, re-entries, total time resolving) is a property of `dt` and
the noise level rather than of the resolver. Probe: log per-episode durations — flicker shows up as a
mass of single-tick episodes.

## 3. The IPS stream tree depends on `n_particles`

**Status: confirmed by reading; no wrong numbers, but it blocks a planned feature.**

`ips_once` allocates `children(level_seqs[k], 0, n_particles + 1)` and then uses index
`n_particles` as the resampling stream (`ips.py:172-174`). So the resampling stream's *address*
moves when `N` changes, and with it the whole subtree below it.

Consequences: growing `N` cannot reuse any of a previous run's tree, so "run more particles
incrementally" is impossible (whereas growing `n_encounters` or `reps` reuses its prefix exactly,
because both are plain index-addressed fan-outs). It also means two runs differing only in `N` share
no structure at all, which makes them harder to compare than they need to be.

Fix is cheap and behaviour-changing-but-not-wrong: put the resampling stream at index 0 and the
particles at `1 … N`. Then the tree stops depending on `N`. Costs a one-off change of every IPS
number (new code hash, so the cache handles it), so it should land deliberately with its own
re-validation against the ADR-0017 §6 gates.

## 4. `estimate_ipr` used a denominator the resolver could move — **FIXED (item 4a, 2026-07-30)**

**Status: confirmed and fixed. Recorded because the same shape of mistake can recur elsewhere.**

`IPR = 1 - n_los/n_conflict` divided by "encounters where the detector fired on the *true* states".
`StateBased.detect` returns `False` once predicted `dcpa >= rpz` (`cd/statebased.py:31`), so a
resolver that built separation before the conflict entered the lookahead horizon **erased its own
successes from the denominator**. Measured at `tlos=180 / lookahead=120`, n=300:

| noise (pos/vel) | resolver | n_conflict |
|---|---|---|
| any | none | 300 |
| 10 / 1 | MVP | **178** |
| 60 / 6 | MVP | **274** |

`n_los` was also nested *inside* the conflict branch, so an undetected breach was dropped from the
numerator too.

The repo had already half-diagnosed this: `scripts/ips_validate.py` documented that the conditional
denominator "coincides with the unconditional P(LoS) only when lookahead >= tlos", routed the
fixed-geometry path around `estimate_ipr` to avoid it, and printed a runtime WARNING steering users
out of the affected mode. All three are now removed as obsolete.

Fixed by dividing by `n_encounters` and keeping `n_conflict` as a labelled `detection_rate`
diagnostic. **The general rule, worth applying to any future metric: a denominator must be fixed by
the experiment design, never discovered from the run.** Anything whose count depends on behaviour is
a numerator or a distribution, never a divisor.

No published repo result was affected — every config in `configs/` and `scripts/` has
`tlos < t_lookahead`, where the two denominators coincide. Anyone reproducing the *papers'* spawn
rule (`tlos = 1.5 × t_lookahead`) would have hit it.

## 5. Two of Experiment 3's six noise models are missing, and two disagree with the paper

**Status: confirmed. Spun off as its own investigation.**

`cns/noise_distributions.py` implements four models; the paper's Appendix defines six. **Latency**
and **Latency + Anisotropic** are absent, because `NoiseDistribution`'s `(rng, ci95)` signature
cannot see heading or ground speed, which an along-track bias `−ℓg` needs. Separately, the two
anisotropic models are implemented **axis-aligned (North/East)** on the stated grounds that "GPS
position-error anisotropy comes from satellite geometry, not the vehicle's heading", while
`99AppNoiseModels.tex` specifies `Σ = T(ψ) Σ_ac T(ψ)ᵀ` — **rotated with the heading**. So only
*Normal* and *Heavy-tail* are faithful to the paper, and Experiment 3 is not reproducible today.

The orientation question is a physics question, not a code question, and is being investigated
separately. The signature widening is unblocked either way (the latency models need ψ and `g`
regardless) — [[run-experiment-todo]] item 9.

## 6. Smaller things, noted in passing

- **`test_every_sampled_encounter_is_a_conflict` passed by accident** of its config
  (`tlos < t_lookahead`), not by construction. Re-documented and given an explicit precondition
  assertion in item 4a; it would have failed under the papers' own spawn rule.
- **`CommState` is a closed frozen dataclass** (`held`, `in_flight` only), so a user-contributed
  stateful communication model has nowhere to keep its state. Subclassing survives the round trip
  through `CNS.sense`, but `CnsState.initial(n)` hands the model a plain `CommState()` on the first
  tick, so every such model must detect and upgrade it by hand. Not a bug in current behaviour — a
  gap in the contribution surface. [[run-experiment-todo]] item 7.
- **Per-tick comm-stream consumption is data-dependent** — one reception draw per directed link, and
  a latency draw only if received. Harmless for correctness (per-particle streams isolate clones)
  but it means the comm stream position is not a function of tick count alone. Already noted in
  [[important-ips-gap]].
- **`Performance` is one flat bag shared across airframes**, so an unused field sits at its `0.0`
  default and `0.0` is *also* a value the reading integrator interprets — a mismatched envelope flies
  silently wrong. Guarded at runtime by `Dynamics.validate_performance`; the typed fix is [[TODO]]
  item 9 (v2).

---

## Related

- [[run-experiment-todo]] — the build order these were found in; items 4a (fixed), 4b, 7, 9 touch them
- [[important-ips-gap]] — the discrete-jump coordinate problem, and the comm-stream note
- [[0017-ips-level-and-splitting]] — the shells that entry 1 could cause to be missed
- [[TODO]] — item 5 (lint/type scoping), item 9 (typed `Performance`)
