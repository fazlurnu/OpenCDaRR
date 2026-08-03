# Probabilistic FTR's angular grid undersamples the very peak it integrates

**Status: found and fixed.** `ProbabilisticFTR` used to discretise its angular integral on `ktheta`
points spread **uniformly around the full circle** (256, inherited from the reference implementation
— [[probabilistic-ftr-recovery]]). That grid is the wrong shape for the integrand: at ordinary
velocity accuracy it lands only a handful of nodes on a density spike that carries all the mass, so
the returned probability is decided by one or two samples and moves with the intruder's heading. A
grid **centred on the mean velocity direction** fixes it at a quarter of the node count. Written
2026-08-03.

**Now shipped**: `grid="centred"` is the default (`ktheta` 128), `grid="uniform"` (`ktheta` 256)
stays available for reproducing the reference. `ktheta` defaults per grid because the two rules
need different node counts.

**It is an accuracy change, not a speed one.** The first draft of this note claimed ~2-3x for the
grid, measured while `erf` went through `np.vectorize` — a Python-level loop whose cost tracks
`ktheta` directly, so halving the nodes halved the bill. `_phi` now prefers SciPy's `erf`, and once
`erf` stops being the bottleneck the node count mostly stops mattering: what remains is fixed
overhead in `_log_p_theta` (the 2x2 `inv`/`det`, the stacks, the matmuls, none of which scale with
`ktheta`). Measured on a quiet machine, `_p_offset_gt` at the shipped encounter:

| | scipy erf | numpy fallback | old `np.vectorize` |
|---|---|---|---|
| uniform 256 (old default) | 258 µs | 437 µs | 430 µs |
| **centred 128 (new default)** | **233 µs** | 349 µs | 336 µs |
| centred 64 | 238 µs | 272 µs | 284 µs |

So the grid is worth **1.11x** and the `erf` backend **1.50x**, for **1.85x** end to end from the
old configuration to the new one. Note the fallback is no faster than the `np.vectorize` it
replaced (0.96x) at these sizes; it is there to avoid the per-call ufunc rebuild, not as a win.
**The grid earns its place on correctness and would be worth making even if it cost more.**

**`ktheta=64` did not survive contact.** The isolated sweep below says 64 centred is fine, and it
was the intended default. It failed a real end-to-end encounter —
`test_the_declaration_changes_a_whole_encounter_when_recovery_reads_it`, velocity SNR ~13 — where
96 passes and 64 does not, breaking a monotonic `min_sep` ladder that every other configuration
(including uniform 256) gets right. The cause is the tail-step limit in *Honest limits* below, and
the lesson is that **a per-call error bound of 2.4e-2 is not the same as a safe default**: one
flipped resume decision changes a whole trajectory. 128 is 96 with margin.

Reproduce: `examples/handbook/probftr_angular_grid.ipynb` (top to bottom, ~20 s, numpy only). The
notebook compares the two rules as options rather than as shipped-vs-candidate.

## The mechanism: two sharp features, one blind grid

The integrand is `p_theta(theta) * P(offset > rpz | theta)`, and both factors are narrow, for
unrelated reasons:

| feature | width | why |
|---|---|---|
| projected-normal density (`_log_p_theta`) | `~ 1/SNR` rad, `SNR = |mu_v| / sigma_v` | a well-known velocity is a well-known direction |
| conditional folded-normal tail | `~ sigma_r / |mu_r|` rad | a small angle swings the projection of a long `mu_r` past `rpz` |

Neither width is known to the grid, which is fixed at `2*pi/ktheta`. For a perfectly ordinary
encounter — 320 m apart, closing at 34.3 m/s, both sides declaring `pos_ci95` 6 m and `vel_ci95`
1 m/s — the numbers are:

    SNR 59.4    density peak 0.0168 rad    tail step 0.0108 rad    grid step 0.0245 rad (ktheta 256)

**Four of the 256 nodes land inside the ±3-sigma peak.** At `ktheta = 64` it is one node, and that
node carries **99.6% of the normalised weight**. The integral is a one-point estimate wearing a
256-point costume.

## What it does to the answer

Same encounter, reference value 0.967489 (a grid resolved to 30 points across the narrower feature,
cross-checked against a 4×10⁶-sample Monte Carlo at 0.967487 ± 0.000174, gap 2×10⁻⁶):

| `ktheta` | uniform (shipped rule) | centred |
|---|---|---|
| 16 | **0.000000** | 0.967388 |
| 32 | **1.000000** | 0.967489 |
| 64 | 0.633890 | 0.967489 |
| 128 | 0.934799 | 0.967489 |
| **256 (shipped)** | 0.965969 | 0.967489 |
| 1024 | 0.967489 | 0.967489 |

The uniform column is **not a converging sequence**. 0.000 and 1.000 are not approximations of
0.967, they are different answers. Non-monotonicity is the tell: doubling `ktheta` re-rolls which
sample sits near the peak rather than resolving it. The centred grid is exact from 32 nodes.

## Across 400 sampled encounters

Separations 100–800 m, speeds 0.5–45 m/s, `pos_ci95` 1–30 m, `vel_ci95` 0.05–6 m/s; `rpz` 50 m,
`prob_threshold` 0.999. "Decisions changed" counts encounters where the grid flips what
`should_resume` returns:

| grid | max error in P | median error | decisions changed |
|---|---|---|---|
| uniform, `ktheta` 64 | 6.7×10⁻¹ | 2×10⁻¹⁶ | 4 / 400 (1.0%) |
| **uniform, `ktheta` 256 (shipped)** | **1.2×10⁻¹** | 1×10⁻¹⁶ | **1 / 400 (0.2%)** |
| centred, `ktheta` 64 | 2.4×10⁻² | 1×10⁻¹⁶ | 0 / 400 |

The median is at the double-precision floor in every row — **the typical encounter is fine under
every rule**, which is exactly why this survived to now. The failures are a rare minority and they
are total, and an error of 0.12 in a number being compared against 0.999 is not a rounding issue.

**The two rules fail at opposite ends of SNR**, worst-per-bin:

| SNR bin | 3–9 | 9–25 | 25–70 | 70–190 | 190–526 |
|---|---|---|---|---|---|
| uniform 256 | 3.3×10⁻³ | 2.9×10⁻² | 6.5×10⁻² | 1.0×10⁻¹ | 1.2×10⁻¹ |
| centred 64 | 5.6×10⁻³ | 2.4×10⁻² | 3.6×10⁻³ | 5×10⁻¹⁴ | 5×10⁻¹⁴ |

The uniform grid **degrades as the velocity fix improves**, which is backwards and is the operating
direction that matters: a better `vel_ci95` narrows the peak, and a fixed grid then resolves it
worse. GNSS velocity accuracy well under 1 m/s puts real runs at SNR 30–300, the worst part of that
row.

## The candidate rule

Ten lines, no new dependency, no change to the weights:

    snr        = |mu_v| / sigma_v
    half_width = min(pi, 8 / snr)          # 8 angular sigma covers the peak
    theta      = atan2(mu_v) + linspace(-half_width, +half_width, ktheta)

Spacing stays uniform *inside* the window, so the existing `w / w.sum()` normalisation is still
correct. At low SNR the window reaches `pi` on its own and the rule degenerates to the uniform grid,
so there is no special case and no discontinuity. Cost is one `atan2` and one `hypot` over the old
path, i.e. nothing. `should_resume` calls it up to twice per active pair per timestep, so the
accuracy change lands on every timestep of every conflict.

**On measuring any of this here**: the development machine ran 35 Jupyter kernels at load average
180 during much of this work, and absolute timings swung by 3-5x between runs of an identical call.
Only tightly interleaved ratios — every variant re-measured inside every round — held up, and the
first non-interleaved table produced was self-contradictory (it put SciPy's `erf` *slower* than the
Python-loop fallback). Numbers taken on a loaded laptop deserve that check before being quoted.

## Honest limits

- **The window is built from the density only, and ignores the tail.** Below SNR ~25 the tail step
  is the narrower feature and the centred grid inherits the same undersampling: error climbs to
  2.4×10⁻² at `vel_ci95` 6 m/s once the window opens past 0.8 rad. **This is the limit that set the
  default `ktheta`**, via the SNR ~13 encounter above. Fixing it properly means sizing the grid from
  *both* features rather than raising the node count until the symptom goes away — `_theta_grid`
  would need `mu_r` and `sigma_r`, which `_p_offset_gt` already has and simply does not pass. That
  is the obvious next step and is not built. **The centred grid is a large improvement, not a
  converged quadrature.**
- **Zero decision flips in 400 is not zero flips.** The population is a sample, and the threshold
  0.999 makes the flip count sensitive to where the errors happen to land.
- **No fleet-level effect has been measured.** Everything above is `_p_offset_gt` in isolation. What
  a 0.2% resume-decision flip rate does to IPR or P(LoS) over a run is unmeasured, and it may well
  be nothing — [[recovery-criteria-comparison]] would be the place to check.
- **`8` angular sigma is a round number, not a tuned one.** It was not swept.
- The 400-encounter population is uniform in its parameters, not drawn from any traffic model, so
  the 0.2% and 1.0% rates describe *that* population and are not an operational frequency.

## Why this was not caught

There *is* a resolution test — `test_ktheta_is_configurable_and_stays_a_valid_probability` — and it
comes within one line of catching this. It runs one geometry at `ktheta` 32 and 512 and asserts the
two agree. But it compares the **booleans**, not the probabilities, and its own comment already
hedges the gap: *"low resolution changes precision, not the qualitative answer here"*. The word
doing the work is **here**. On that one geometry the two probabilities straddle no threshold, so the
booleans match while the numbers need not.

Comparing the probabilities instead, over a handful of geometries at `ktheta` and `2*ktheta`, is
about as cheap and fails immediately on the encounter above. That is now
`test_centred_grid_is_self_consistent_under_refinement`, asserted on the probability rather than the
bool, since self-consistency under refinement is the property `ktheta` actually exists to give.

## The suite had encoded the bug

Switching the default flipped two long-standing tests, and **both were asserting the wrong answer**.

`test_uncertainty_degrades_clearance_confidence` opened with "a geometry that clears comfortably at
zero declared uncertainty" and asserted `should_resume is True` for an intruder 380 m off at
bearing 5°. That geometry's closest-approach offset is **49.60 m, inside the 50 m protected zone**,
so the correct answer at zero uncertainty is `False`. The uniform grid returned P = 1.000000 at
`ktheta` 256 and P = 0.000000 from 4096 upward; the test had been calibrated against the error.
`test_higher_threshold_is_stricter` reused the same geometry and inherited the same problem.

Both now run at bearing 10° (offset 82.3 m, genuinely clearing) with the noisy leg at `pos_ci95`
25 m, and the expected values were checked against a 200 000-node reference rather than against
whichever grid happened to be default.

**The lesson is about calibration, not about this grid.** A test written by observing what the code
returns will encode whatever the code gets wrong, and will then defend it. The tests that survived
the switch unchanged are the ones asserting a property derived independently — agreement with
[[ftr-recovery]], the radial-trajectory divergence, monotonicity in threshold.

Companions: [[probabilistic-ftr-recovery]] (the derivation this refines), [[ftr-recovery]] and
[[recovery-criteria-comparison]] (the deterministic sibling and the comparison), [[gps-noise]]
(where `CI95_TO_SIGMA` and the declared accuracies come from).
