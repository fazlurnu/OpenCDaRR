# Might be a bug

Suspected defects and smells found while reviewing [[run-experiment-design]] and building
[[run-experiment-todo]] items 1–10. Each is *unconfirmed unless marked otherwise* — written down so a
measured, logged issue stays a decision rather than neglect (`design-philosophy.md` #13). Ordered by
how badly it could corrupt a number.

---

## 1. Loss of separation was only sampled at `dt` — **FIXED (2026-07-30)**

**Status: confirmed, quantified, fixed.** Separation is now measured over each whole step
(`kinematics.segment_min_range`), not at its endpoints. Full measurements, the cost, and what the fix
does *not* cover in [[segment-min-separation]].

**The impact, against an analytic reference.** `P(LoS)` at `rpz` was barely affected — worst case
**−0.6%** at `dt=1.0` — which is why no published IPR number here was materially wrong and why this
survived so long. The damage was at the small radii IPS splits on, where the relative error goes as
`(v_rel·dt)² / (24 d²)`:

| shell | `dt=1.0` | `dt=0.5` | `dt=0.2` | after |
|---|---|---|---|---|
| 10 m | −9.3% | −2.0% | 0.0% | exact |
| 5 m | **−43.8%** | −9.2% | −0.8% | exact |
| 1 m | **−89.2%** | −75.8% | −42.5% | exact |

The `1/d²` is why refining `dt` was never the answer: sub-1% needs `dt < d/(2 v_rel)`, i.e. 0.024 s
for a 1 m shell. The fix costs **+0.95%** of a step and is exact at `dt=1.0`.

Two findings worth carrying forward: the fix must **interpolate positions, not extrapolate the
pre-step velocity** (the latter invents losses of separation on turning aircraft — `P(LoS)` inflated
10%), and the strict/non-strict boundary below is *still* open, as is the `tlos ≡ 0 mod dt` alignment
artifact, which was 100% present at every `dt` tested and is untouched by the fix.

<details>
<summary><b>Original entry</b> (kept for the trail, per <code>design-philosophy.md</code> #19)</summary>

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

</details>

**Still open from this entry** (deliberately not bundled into the fix): the `<` / `<=` mismatch
above — ADR 0017 §1 sets `d_m = rpz` and claims IPS estimates the *same* `P(LoS)` as MC, which fails
exactly at the boundary — and the ADR-0017 §6 gate re-validation, since every IPS number re-bases.

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

## 6. A declared parameter bypasses every config constraint

**Status: confirmed by measurement (2026-07-30). Not yet fixed — found while building
[[run-experiment-todo]] item 11.**

`config.load_config` validates thirteen constraints, but `experiment._config_for` builds a
condition's `Config` with `dataclasses.replace` and **never calls `_validate`**. So every
constraint holds for a config *file* and none of them hold for a `Fixed`/`Sweep` declaration —
which is the surface the v1 audience is actually pointed at.

Measured against the invariant item 4a added:

```python
run_experiment({"dcpa_max": Fixed(500.0)}, ...)   # base config has rpz = 50
# -> runs, with dcpa_max = 500 > rpz = 50
```

`_validate` on that same built config raises `scenario.dcpa_max <= conflict.rpz`. That constraint
exists precisely so every sampled encounter *is* a conflict and P(LoS)'s denominator needs no
filtering; bypassed, a fraction of encounters are silently non-conflicts and **P(LoS) is reported
over a mixed population** — a wrong number, reported confidently, with no warning. The other
constraints (`rpz > 0`, `t_lookahead > 0`, `margin >= 1`, `dt > 0`, `broadcast_interval >= dt`) are
equally unenforced.

Not every field fails silently: a declared `broadcast_jitter >= broadcast_interval` is caught
downstream by `BroadcastSchedule.__post_init__`, loudly, because that value reaches a constructor
that checks it. The hazard is the fields whose only check lives in `_validate`.

The fix looks like one line — call `_validate` at the end of `_config_for` — and the reason to do
it deliberately rather than in passing is that it is a **behaviour change**: any existing
declaration that violates a constraint starts raising. Worth a quick sweep of the repo's own
scripts and notebooks before it lands. Fixing it would also let item 11's transmit fields fail at
declaration time in the config's vocabulary rather than part-way into a run.

## 7. `pos_ci95` is silently inert without a navigation model — **fixed 2026-07-31**

**Status: fixed on the declaration surface.** `run_experiment` now raises when a resolved condition
declares a non-zero `pos_ci95`/`vel_ci95` and its stack has neither a navigation model to draw the
error nor a ci95-reading recovery criterion. Checked **per condition** after `expand()`, because
`pos_ci95`, `navigation` and `recovery` are each independently sweepable — a navigation sweep that
includes `None` is legitimate wherever the accuracy is zero and a mistake only where it is not.

Deliberately a *contradiction* check and not an implication, per this note's own ruling below: a
declared accuracy read only by `ProbabilisticFTR`, with no noise model present, stays valid.

**Two limits worth knowing.** (a) The guard is on `run_experiment` only. The 24 scripts that call
`run_fleet` directly, and `estimate_ipr` itself, still default `navigation=None` and are not
covered — pushing it down to `estimate_ipr`/`build_env` is a separate change. (b) The YAML path was
*not* wired up: `configs/pairwise.yaml` has no readers outside two test modules, `run_one_experiment`
none outside one, and `registry.py` still has no `make_navigation`, so building one would be
generality for a surface with no users.

**Original diagnosis (2026-07-30) follows.**

`pos_ci95` / `vel_ci95` are read by exactly two consumers in the package: `cns/navigation.py`
(`GnssNavigation`, which draws the error) and `crr/probabilistic_ftr.py` (`ProbabilisticFTR`, which
sizes its uncertainty). With neither in the stack the fields are stamped onto every
`AircraftState`, carried through the whole run, and **never read**. `estimate_ipr` defaults
`navigation=None`, so the default MC path is exactly that case.

The failure mode is quiet and plausible-looking: declare `pos_ci95=Sweep([0, 10, 20, 40])` without
declaring `navigation`, and four cells run **bit-identical**, so the table reads "navigation
accuracy has no effect on safety" — a publishable-looking null result produced by a no-op. Found
while building [[run-experiment-todo]] item 10, where a comm-outage sweep returned P(LoS) = 0 at
every rate and looked for all the world like the new model was unwired; it was wired, and the
declaration was missing `navigation`. Adding it turned the same sweep into 0.060 → 0.437.

Neither field can simply *imply* a navigation model — `pos_ci95` is a declared accuracy, and
`ProbabilisticFTR` legitimately reads it with no noise model present, so a non-zero value with
`navigation=None` is a valid (if unusual) configuration rather than a mistake per se. The
fail-fast candidate is narrower: warn or raise when a **declaration** carries a non-zero
`pos_ci95` / `vel_ci95` while neither consumer is in the stack, since at that layer it is
certainly not what the caller meant. Worth checking the other declarable keys for the same
property before fixing just this one.

## 8. A swept component never reached the cache key — **fixed 2026-07-31**

**The worst kind of bug this project can have: `cache=True` returned wrong numbers, silently, and
they looked right.** Two conditions differing only by a swept component shared one cache key, so
the first cell computed and every later one was served *its* result under a different name.
Measured at `dpsi=2°`, `pos_ci95=10`, n=60:

| | uncached | `cache=True` |
|---|---|---|
| MVP | 0.050 | 0.050 |
| **VO** | **0.783** | **0.050** |

VO reported MVP's number. Nothing in the table, the card or the entry count says so — the run is
fast, the row is plausible, and the conclusion ("the two resolvers are equivalent") is exactly the
kind of null result a sweep is run to find.

**Cause.** `_run_one` passed the caller's `Methods` bundle to `_cache_params`, which keyed
`identity()` off it. But a component declared as an axis lives on the *condition*, not the bundle —
`_run_mc` calls `_resolved_methods(condition, methods)` and the key never did. All eight of
`_COMPONENTS` were affected (`detector`, `resolver`, `recovery`, `navigation`, `communication`,
`surveillance`, `kinematics`, `perf`), i.e. every categorical benchmark the runner exists to
support. `identity()` itself was never at fault: it was being handed the wrong object.

**Fix.** `_cache_params` resolves the condition itself rather than trusting its caller to have done
it, so the key and the run read the same objects and cannot drift apart again. One line, no
behaviour change anywhere else: `_resolved_methods` returns the bundle unchanged when no component
is declared, so **a declaration without a component axis keeps the key it had** — verified — and
existing cache entries stay valid.

**Why it survived.** `test_changing_a_component_changes_the_cache_key` covers exactly this hazard,
but changes the resolver through the `Methods` *bundle*, which was always keyed. The axis path had
no test. And the cold/warm test (`warm.records() == cold.records()`) passes happily with the bug —
both halves are wrong in the same way.

The regression added asserts **cached equals uncached**, which is the promise the cache actually
makes (*may only ever save time, never change a result*), rather than counting entries. A count
would have said the keys differ without saying the right numbers came back — and wrong-but-plausible
numbers were the whole failure mode. It also asserts the two levels are distinguishable at all
first, so it cannot pass vacuously.

**The general rule this is the second instance of:** *anything that varies per condition must be
derived from the condition, not from the declaration.* Entry 6 is the same shape one layer over —
`_config_for` builds its `Config` with `dataclasses.replace` and never re-validates, so a declared
parameter bypasses every config constraint. Both come from a per-condition value being read off the
wrong object.

Found while writing `examples/handbook/resolver_comparison.ipynb`, by an assertion that a cached
re-run of a sweep reproduces the uncached one. Worth keeping that assertion in the notebook.

## 9. Three docstrings name `level` as the importance function; IPS never calls it

**Documentation defect, not a behaviour bug — the right quantity is being used.** But it
misdescribes the extension interface, which is exactly the surface the v1 contributor audience is
pointed at.

Three places describe the estimator seam as `advance` / `level` / `is_terminal`:

- `fleet.py:26` — "`level` is the importance function (minimum pairwise separation)"
- `ips.py:3` — "multilevel splitting over the fleet estimator interface (`advance / level /
  is_terminal`)"
- `fleet.py:511` — "IPS replaces this loop with resample-and-split over the *same* `advance` /
  `level` / `is_terminal`"

`ips.py` calls **none** of them. `_evolve_to_shell` reads `state.min_sep` directly, and
`resample_level` selects on `p.state.min_sep <= target`. Grepping the tree, `fleet.level` has no
production caller at all — only `tests/test_fleet_interface.py` exercises it.

**The two are different quantities, and the difference is the whole point of ADR 0017.**
`level(state)` returns `_pairwise_min_sep(state.states)`, the separation **at that instant**.
`state.min_sep` is the **running minimum** over the run so far. ADR 0017 §"running-min level
function" chose the running minimum deliberately: the instantaneous separation is *non-monotone*, so
a particle would cross a shell going in and cross it again coming out — ambiguous for splitting.
The running minimum is monotone non-increasing by construction, so a crossing is permanent.

So the docstrings name the function ADR 0017 explicitly **rejected**. A contributor writing a custom
environment against that stated interface would implement `level` faithfully, find IPS ignoring it,
and have no way to know their importance function was never consulted.

Two candidate fixes, not yet chosen:

- **Re-document.** Say the interface is `advance` / `is_terminal` plus the `min_sep` accumulator on
  the state, and demote `level` to what it is — an instantaneous read used by tests. Smallest change,
  but it leaves a public function with no caller.
- **Make `level` the real seam.** Have `ips.py` call it, redefined to return the running minimum, so
  the importance function is genuinely pluggable — which is what a Phase-8 refinement to a
  look-ahead coordinate would need anyway (`scripts/ips_unified_validate.py` already prototypes
  exactly that with its own `LookaheadCoord.in_level`). Larger, and it must not move any number.

The second is the better shape if the importance function is ever going to be swappable, and
[[important-ips-gap]] argues it will need to be. `tests/test_fleet_interface.py:100` already
documents the instant-vs-accumulated distinction correctly, so the test suite is not part of the
problem.

Found 2026-07-31 while tracing what the environment actually hands back to IPS for the handbook's
architecture diagram.

## 10. Smaller things, noted in passing

- **`test_every_sampled_encounter_is_a_conflict` passed by accident** of its config
  (`tlos < t_lookahead`), not by construction. Re-documented and given an explicit precondition
  assertion in item 4a; it would have failed under the papers' own spawn rule.
- **`CommState` was a closed frozen dataclass** (`held`, `in_flight` only) — **FIXED (item 7,
  2026-07-30)**. Subclassing already survived the round trip through `CNS.sense`, but
  `CnsState.initial(n)` handed the model a plain `CommState()` on the first tick, so every stateful
  model had to detect and upgrade it by hand. `CommunicationModel.initial_state()` now supplies that
  first value, so a model's own subclass is in place from `t = 0`. Never a bug in current behaviour
  — a gap in the contribution surface. [[run-experiment-todo]] item 7.
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

- [[segment-min-separation]] — entry 1's measurements, fix and remaining gaps
- [[run-experiment-todo]] — the build order these were found in; items 4a and 7 (both fixed), 4b,
  10 and 9 touch them
- [[important-ips-gap]] — the discrete-jump coordinate problem, and the comm-stream note
- [[0017-ips-level-and-splitting]] — the shells that entry 1 could cause to be missed
- [[TODO]] — item 5 (lint/type scoping), item 9 (typed `Performance`)
