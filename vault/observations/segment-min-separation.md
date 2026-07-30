# Separation measured over the step, not at its ends

**Status: fixed 2026-07-30** ([[todo-might-be-a-bug]] entry 1). `FleetEnv.advance` sampled
separation once per `dt` and set `los = cur < rpz`, so a pass that dipped inside a threshold and back
out within one step left no sampled point inside. The error is **one-sided** — it can only ever
report *more* separation than there was — and it worsens as the threshold tightens, which is why it
barely touched `P(LoS)` at `rpz` and badly corrupted the small radii IPS splits on. Replaced by a
closed-form minimum over each whole step (`kinematics.segment_min_range`). Reproduce every table
below with [`scripts/segment_min_sep_probe.py`](../../scripts/segment_min_sep_probe.py) —
`plos`, `shells`, `cost`, `compare`.

The headline: **`P(min_sep <= 5 m)` was under-counted by 44% at `dt = 1.0` and 9.5% at `dt = 0.5`**,
against an analytic reference. `P(LoS)` at `rpz = 50 m` was never off by more than 0.6%.

---

## 1. The reference is analytic, not another read of the simulation

Every pre-existing test compared `los` against quantities derived from the *same* sampled trajectory,
so none of them could see a measurement that systematically misses part of that trajectory. The fix
needed a reference from outside the code (`design-philosophy.md` #15).

`scenario.create_conflict` supplies one: it constructs a straight-line encounter whose true minimum
separation **is** the requested `dcpa`. So for a non-manoeuvring pair with `dcpa` on a uniform grid,
the exact level-crossing curve is known in closed form — `P(min_sep <= d) = d / dcpa_max` — with no
simulation involved at all. The simulation is purely the thing being graded.

The construction was itself cross-checked: the flown geometry achieves the nominal `dcpa` to within
**~3 cm** (geodesy plus integration), independent of `dt`. That residual is the floor on every
accuracy claim below.

## 2. Time and accuracy of the counted `P(LoS)`, by `dt`

1000 straight-line encounters (4 crossing angles × 250 miss distances), `dcpa` uniform on
`[0, 100] m` with `rpz = 50 m`, so **exact `P(LoS)` = 0.5000 by construction**. Each `dt` runs twice:
once with `record=False` for the timing, once with `record=True` to reconstruct what the old
endpoint reading would have returned from the identical trajectory.

| `dt` | steps/enc | wall clock | µs/enc | `P(LoS)` **segment** | err | `P(LoS)` **endpoint** | err |
|---|---|---|---|---|---|---|---|
| 1.00 | 72.2 | **7.13 s** | 7 131 | 0.5000 | **+0.00%** | 0.4970 | −0.60% |
| 0.50 | 145.1 | 11.93 s | 11 934 | 0.5000 | **+0.00%** | 0.4990 | −0.20% |
| 0.20 | 364.8 | 26.69 s | 26 692 | 0.5000 | **+0.00%** | 0.5000 | +0.00% |

Two readings of this table, and the second matters more than the first:

- The segment measurement is **exact at every `dt`**, including the coarsest. Endpoint sampling needs
  `dt = 0.2` to reach the same answer, at **3.7× the wall clock**.
- `P(LoS)` at `rpz` is a **weak discriminator**. Even the worst case here is only −0.6%, which is why
  this defect survived so long and why no published IPR number in the repo is materially wrong. The
  quantity that was badly wrong is the one nothing was checking.

## 3. Where it was actually severe: the IPS shells

Same analytic reference, `dpsi = 90°`, 6000-point `dcpa` grid, exact `P(min_sep <= d) = d/50`.
`ips.py:101,137` cross shells on `state.min_sep` — the *same* running minimum — so this is the error
the rare-event estimator inherits directly.

Endpoint error (the defect), by `dt`, then the segment error, which is `dt`-independent:

| shell | exact | endpoint `dt=1.0` | `dt=0.5` | `dt=0.2` | **segment**, any `dt` |
|---|---|---|---|---|---|
| 50 m | 1.0000 | −0.30% | 0.00% | 0.00% | +0.00% |
| 25 m | 0.5000 | −1.40% | −0.20% | +0.10% | +0.10% |
| 10 m | 0.2000 | **−9.50%** | −2.25% | 0.00% | +0.25% |
| 5 m | 0.1000 | **−44.00%** | **−9.50%** | −0.50% | +0.50% |
| 2 m | 0.0400 | **−77.50%** | **−55.00%** | **−7.50%** | +1.25% |
| 1 m | 0.0200 | **−90.00%** | **−75.00%** | **−40.00%** | +2.50% |

The segment column is **not zero, and I first wrote it up as "exact" — wrong.** It is a constant
**absolute** offset of 3–4 grid points at every shell (~0.033 m of `dcpa`, i.e. the ~3 cm
nominal-vs-flown residual from §1), and because the exact probability shrinks with the shell, the same
absolute offset reads as a growing *relative* figure: +0.1% at 25 m, +2.5% at 1 m. Two things make it
benign where the endpoint error is not — it does not scale with `dt`, and it is **positive**, so it
over-counts crossings rather than hiding them. A residual belonging to the *measurement* would scale
as `1/d²` like the endpoint column and would show a growing grid-point count, not a constant one;
this one is a property of the reference geometry.

The `law` column in the script's output over-predicts once the error approaches unity (8.82 predicted
against −90% measured at `d=1, dt=1.0`) — the bias saturates at −100%, the law does not. It is a
small-error approximation and should be read as one.

Per-shell survivals telescope, so an IPS estimate of `P(min_sep <= d_m)` inherits exactly the
innermost row — a factor-2 error at a 5 m target under the shipped `dt = 1.0`, far outside the
replication CI IPS reports alongside it. On real manoeuvring runs `min_sep` was overestimated by up
to **8.3 m** at `dt = 1.0`, more than one shell spacing.

## 4. The scaling law, and how to choose `dt`

The measured relative bias tracks

```
relative error in P(min_sep <= d)  ≈  (v_rel · dt)² / (24 d²)
```

across three decades (predicted 8.8e-2 vs measured 9.3e-2 at `d=10, dt=1`; 1.4e-2 vs 0.8e-2 at
`d=5, dt=0.2`). Derived from chord-versus-step-length geometry, not fitted.

The `1/d²` is the whole problem: **halving the target costs four times the resolution.** Rearranged,
sub-1% wants `dt < d / (2 v_rel)` — at head-on (`v_rel = 20.6 m/s`) that is `dt < 0.12 s` for a 5 m
shell and `dt < 0.024 s` for 1 m. No practical step size reaches the depth a 1e-9 target implies, so
this could not have been fixed by refining `dt`.

## 5. Interpolate positions; do **not** extrapolate velocity

[[todo-might-be-a-bug]] entry 1 proposed "closed-form CPA between the pre- and post-step states".
That reads two ways, and they are not equivalent — measured on manoeuvring runs:

| | max \|chord − cpa_v\| | `P(LoS)` raw / chord / cpa_v |
|---|---|---|
| `dt=1.0`, straight line | 0.000 m | 0.9975 / 1.0000 / 1.0000 |
| `dt=1.0`, MVP + noise | **9.72 m** | 0.1225 / 0.1225 / **0.1350** |
| `dt=0.5`, MVP + noise | **2.48 m** | 0.1075 / 0.1075 / **0.1175** |

Extrapolating the pre-step relative velocity across the step (`cpa_v`) **invents losses of
separation** — `P(LoS)` inflated 10% at `dt=0.5` — because while MVP is turning hard a straight-line
extrapolation leaves the flown path and evaluates separation where the aircraft never was. The two
agree to 0.000 m on straight lines, so the entire gap is curvature.

Interpolation cannot do this: every range it reports lies on the segment between two states the
simulation actually produced. `segment_min_range` therefore ignores the velocities on `Relative`
entirely, and a test pins that (feed it absurd velocities, get the same answer).

## 6. Cost: +0.95% of a step

The refinement needs the relative *vector* at both ends of a step, and `_pairwise_min_sep` already
paid a `geo.qdrdist` per pair per step for a bare scalar. Swapping that call for `relative_enu` —
same geodesy plus a subtraction — buys the vector almost free, and each step's post-step vector *is*
the next step's pre-step vector, so there is no second call.

| per pair per step | µs |
|---|---|
| `advance()` (n=2, nav noise + MVP, `dt=0.5`) | 120.05 |
| `geo.qdrdist` — paid before | 8.91 |
| `relative_enu` — paid now | 9.75 |
| segment arithmetic — new | 0.30 |
| **marginal** | **1.14 → +0.95%** |

Against +150% for dropping `dt` from 0.5 to 0.2. The refinement is ~160× cheaper than the step
reduction it replaces, and correct two decades deeper.

## 7. What changed

- **`kinematics.segment_min_range(r0, r1)`** — the per-pair algebra. It lives here, beside
  `relative_enu`, rather than next to the CPA equations in `cd/`, because both encounter runners must
  measure separation *identically* for the n = 2 reduction to hold, and two copies of this drifting
  apart would be worse than the mild tension with `kinematics`' "core math stays legible where it is
  read" note. It is a measurement helper, not a CDR algorithm.
- **`fleet._pairwise_relative` / `_segment_min_sep`** — the pairwise wrapper. `advance` now captures
  the pre-step geometry, detects as before, and measures separation *after* integrating, over the
  step just flown. Consecutive segments share an endpoint, so the running minimum covers the
  trajectory **continuously from `t=0`** instead of at a comb of instants — including the terminal
  state, which the old top-of-loop reading never measured at all.
- **`loop.run_encounter`** — same treatment, reusing the `relative_enu` it already computed
  post-step, so the pairwise reference gets the refinement at *zero* extra geodesy.
- **`FleetState` is unchanged.** Both ends of a step are available within one `advance` call, so
  nothing had to be carried across calls — the particle stays exactly as wide as it was.

**MC trajectories are bit-identical.** `min_sep` / `los` / `conflict` are write-only accumulators:
nothing in the decision path reads them (the control path takes perceived states, `nom`, `mems`,
`rpz`, `t_lookahead`; `is_terminal` reads `t` and `done_timer`; `_all_clear` recomputes `rel.dist`
fresh). Only `estimator` (counts), `ips` (shell crossing) and `viz` (reporting) read them. So
detection and resolution behave exactly as before — this is an observer, not a controller.

**IPS trajectories do change**, by design: a refined minimum crosses shells earlier, so resampling
fires at different points and every IPS number re-bases.

## 8. Twelve golden anchors moved, all downward

`min_sep` anchors in `test_loop`, `test_loop_mixed_fleet` and `test_loop_wind` moved by 1.8e-5 m to
0.20 m, **every one of them down**. The direction is guaranteed rather than observed — a segment
minimum cannot exceed the minimum of its own endpoints — and
`test_segment_minimum_never_exceeds_the_sampled_comb` now pins it as a property, so a later change
cannot move one *up* unnoticed.

`test_fleet_interface`'s `level`-versus-`min_sep` test asserted the two were **equal**. That is
precisely what a per-`dt` sample-and-compare produces, so the test had been pinning the defect it
looked like it was guarding. It now asserts the accumulator is bounded above by the sampled comb, and
strictly tighter on a converging ring.

Worth recording: I first wrote a comment claiming one anchor (`_ANCHOR_MIXED_NOISY_VO`) was unchanged
because its closest approach fell on a step endpoint. It was not — the earlier run had
short-circuited on the assertion above it, so that anchor was never reached. Caught by running with
`-x`. A confident explanation for a number I had not actually seen move.

## 9. Two things this does **not** fix

**The boundary-alignment artifact.** `create_conflict` puts the LoS *entry* at exactly `t = tlos`, so
whenever `tlos` is a multiple of `dt` — which the shipped `tlos=60` is for every `dt` in use — a
sample landed exactly *on* the `cur < rpz` boundary. At `dpsi=180` that made the whole
near-tangential band read as no-LoS (200/200) while `dpsi=45`/`90` read 0/200: same geometry,
opposite verdicts, decided by float rounding. This is independent of `dt` magnitude and was still
100% present at `dt=0.2`. The segment measurement largely dissolves it (the minimum is now a
continuous function of the geometry rather than a sampled instant, so exact equality returns to
measure-zero), but the *cause* is untouched.

**The strict/non-strict mismatch.** `fleet` uses `cur < rpz`; `ips.py:101,137` uses
`min_sep <= target`. ADR 0017 §1 sets the innermost shell `d_m = rpz` and claims IPS "estimates the
*same* `P(LoS)` plain MC does" — a claim that fails exactly at the boundary. Measure-zero in floats,
but it should hold by construction, not by luck. Not bundled in here: it is a semantic change to a
documented ADR and deserves its own decision rather than riding along in a change that already
re-bases every IPS number.

**Still outstanding:** the ADR-0017 §6 gate re-validation. Every IPS number moves, so
`scripts/ips_validate.py` and the gate observations need re-running before any IPS result is quoted
again.

## 10. On choosing `dt`

`dt = 0.2` was under consideration as a new default. With the segment measurement it is no longer
needed for accuracy: `dt = 1.0` is exact on `P(LoS)` and on every shell down to 1 m, at 27% of the
`dt = 0.2` wall clock. `dt` should now be chosen for **dynamics** fidelity — how finely the airframe
integrator and the resolver cadence need resolving — not to chase a measurement artifact. Note that
`P(LoS)` at `pos_ci95=20` did shift 0.0050 → 0.0017 between `dt=1.0` and `dt=0.2` on sampled runs
(3 events vs 1 of 600, not significant), and that difference is integration, not measurement: the
segment and endpoint readings agree with each other at each `dt` there.

## Related

- [[todo-might-be-a-bug]] — entry 1, which this closes; the general rule from entry 4 applies here
  too (a *measurement* must not depend on the sampling grid any more than a denominator may depend
  on behaviour)
- [[0017-ips-level-and-splitting]] — §1's monotone running minimum, and the `d_m = rpz` claim §9
  flags
- [[important-ips-gap]] — the other reason `min_sep` can be the wrong coordinate
- [[run-experiment-todo]] — item 8's level-crossing curve, which is only trustworthy after this
- [[experiment-layer-architecture]] — §7's `fleet → loop` back-edge, the reason the shared algebra
  went to `kinematics`
