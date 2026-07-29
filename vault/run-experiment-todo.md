# `run_experiment` — build order

Working checklist for the v1 slice agreed in the review of [[run-experiment-design]]. One item at a
time, in order; each is independently verifiable and leaves `pytest` green before the next starts.

**Scope decisions settled in review (2026-07-29):**

- **Audience: contributors**, not paper reproduction. The V1.0/V1.1 gates in [[TODO]] are the target:
  write your own CD/CR/CRR → run under MC → run under IPS → write your own dynamics. So v1's job is
  *the same user objects run unchanged under both backends, in three lines*.
- **Estimand: unconditional** over encounters (not the paper's `n_conf` denominator). Store all
  three counts so the paper's conditional IPR stays derivable.
- **Anisotropic noise orientation: unresolved**, spun off as its own investigation. Does not block
  the signature widening.
- **Result object:** `frame()` + `cell()` + `plot()`, all three.

Deferred out of v1 (they belong to a later paper-reproduction driver): `Random`/quadrature marginals
and their CI treatment, ragged/derived/zipped axes, the `Conditioned` latent-event role, pluggable
IPS `level=`, AMS adaptive shells, phase-9 state accumulators, `n_particles` seed-tree decoupling.

---

## 1. Wire the code hash into the experiment card — **done 2026-07-29**

- [x] Replace the literal `code_hash: (deferred)` in `experiment.py:_write_card` with
      `cache.code_fingerprint()`. Stale module docstring ("A code-hash stamp is deferred") updated
      to match. Full suite green (331 passed); a card now reads `code_hash: 4985ab5efa747ce1`, and
      the digest was checked against an independent reimplementation and shown to move when any
      `opencdarr/*.py` byte changes.

`opencdarr/cache.py` has computed this since it landed — a sha256 over every `opencdarr/**/*.py`,
memoised per process. The card has simply never called it. This is the `config + seed + code-hash →
result` contract the whole reproducibility story rests on
(`design_brief.md`: "Never trust a 1e-9 number that isn't anchored to something checkable").

- **Files:** `opencdarr/experiment.py`
- **Verify:** `pytest tests/test_experiment.py`; the card's hash changes when any `opencdarr/*.py`
  changes and is stable otherwise. No existing test pins the placeholder string.

## 2. Route plain MC through `build_env` / `run_fleet` — **done 2026-07-29**

- [x] `estimate_ipr` now runs each encounter through `run_fleet` at n=2 (so `build_env` /
      `advance` / `is_terminal` — the same interface IPS drives) instead of calling
      `run_encounter`. Gained three keyword-only arguments that were previously reachable through
      IPS but not through MC: `dynamics`, `wind`, `share_intent`. `config.simulation.
      broadcast_interval` maps to `BroadcastSchedule(interval=...)` — aligned phase, no jitter, so
      no broadcast stream is ever drawn from and the substream tree is unchanged (ADR 0006 §6).

**Verified.** Parity was established *before* the refactor, then re-checked after:

- New reduction tests over the estimator's actual crossing-angle support — `dpsi ∈ {5, 45, 90, 180,
  270, 355}` × both sides, noiseless and noisy, on `estimate_ipr`'s own `spawn(seq, 3)` layout.
  The pre-existing reduction tests each pinned a single geometry (90°, 45°), so the sweep ends —
  where the closing geometry is most degenerate — were untested.
- Old-vs-new parity: the pre-refactor body reconstructed and compared across five CNS regimes
  (noiseless ± resolver, nav noise with MVP+PastCPA and with VO+FTR, nav noise + lossy comm) —
  identical counts throughout. Then a sharper per-encounter check on *sampled* geometry comparing
  exact float `min_sep`: **0 mismatches in 480 encounters**, at noise levels giving 0, 2, 16 and 47
  LoS out of 120 (so the comparison actually discriminates rather than matching 0 against 0).
- Two permanent anchors added to `tests/test_estimator.py`, which previously pinned only
  *relations* (reproducible, pools, above baseline) and so would have accepted a refactor that
  changed every number: a **golden anchor** on exact counts (22/200, IPR 0.89 at pos_ci95=60), and
  `test_dynamics_reaches_the_mc_path`, the regression that would have caught this defect. The
  latter asserts an exact identity — an airframe that discards resolution commands must give
  bit-for-bit what flying with no resolver gives — rather than a loose inequality.
- Full suite 343 passed; `ruff` clean on the touched files; `mypy` clean on them (the 12 remaining
  errors are the pre-existing ones [[TODO]] #5 catalogues in other modules);
  `scripts/ips_validate.py --pos 40` still returns **PASS** with MC and IPS CIs overlapping.

**Deliberately still single-airframe.** `Agent` supports per-aircraft `dynamics`/`perf` (mixed
fleet, ADR 0011 §7) but `estimate_ipr` passes one of each to both aircraft, because `Config` carries
one `aircraft_type`. A mixed-fleet MC sweep is a separate change with its own config question.

**The defect this fixes.** `estimate_ipr` calls `run_encounter(...)` without passing `dynamics=`, so
MC always uses the default `Multirotor` while IPS honours `Agent.dynamics` via `build_env`. A
contributor's custom `Dynamics` is **silently ignored on MC and works on IPS**. Same for `wind`,
`share_intent`, per-aircraft `perf`, and `BroadcastSchedule`. That single asymmetry breaks three of
the four release gates (V1.0.2, V1.0.3, V1.1.1), and it makes §6 of the design doc — "the same
instance runs unchanged under both backends" — false today.

One env, one seam, both backends. `record=True` and `stop_within` come along free.

- **Files:** `opencdarr/estimator.py` (primary), reusing `fleet.build_env` / `fleet.run_fleet`
- **Verify:** existing IPR numbers **bit-identical** — `pytest tests/test_estimator.py
  tests/test_loop.py tests/test_loop_mixed_fleet.py tests/test_fleet.py tests/test_smoke.py`. Add a
  direct assertion that `run_fleet` at n=2 and `run_encounter` agree on
  `(conflict, los, min_sep)` for a fixed seed across a spread of `dpsi`, with and without nav noise.
  The docstrings already claim this reduction; this makes it a test rather than a claim.

## 3. Parametric pairwise sampler ← **current**

- [ ] Let `scenario.sample_pairwise` take `dpsi` / `dcpa` / `side` / `gs_intr` as either a fixed
      value or a draw.

Today it accepts none of them — it draws `dpsi ~ U(5,355)`, `dcpa ~ U(0,dcpa_max)`, `side ~ ±1`
internally, so **`Fixed(dpsi=90)` has no MC implementation**, and `Fixed(dcpa=0.0)` is
unrepresentable (config carries only `dcpa_max`). Pinning a geometry is the first thing anyone
writing a resolver wants: watch the algorithm work at 90°, then open it up.

The code already exists — `scripts/ipr_angle_sweep.py:_one` does exactly this
(`create_conflict(dpsi=…, dcpa=0.0, side=1)` → `run_encounter`) while bypassing `estimator.py`.
Promote it into the library rather than writing it again.

- **Files:** `opencdarr/scenario.py`, `opencdarr/config.py` (a fixed `dcpa` alongside `dcpa_max`)
- **Verify:** the current U(5,355) / U(0,dcpa_max) / random-side behaviour stays the **default** and
  stays bit-identical — golden-anchor test in `tests/test_scenario.py`. Then a pinned-geometry test:
  `dpsi=90, side=1` reproduces `scripts/ipr_angle_sweep.py` for the same seed.

## 4. Counts and the estimand

- [ ] Record `n_encounters`, `n_conflict` and `n_los` per condition; report unconditional
      `P(LoS)` as primary, with a Wilson CI on the fixed `n`.

Three denominators currently coexist: `estimate_ipr` divides by `n_conflict` (the paper's Eq. IPR),
`scripts/ipr_angle_sweep.py` by `n_pair`, and IPS by `n_particles`. Unconditional was chosen, which
buys two real things: ADR 0017 §4's claim that IPS estimates *the identical* `P(LoS)` MC does becomes
actually true (the analytical ⊂ MC ⊂ IPS ladder is finally apples-to-apples), and the denominator
becomes deterministic, so Wilson is properly valid where before it was a ratio with a random
denominator.

Keeping all three counts means the paper's conditional IPR is derivable from the same row at zero
cost, and the conflict-detection rate becomes a free diagnostic. No fork, no re-runs, papers stay
reproducible.

- **Files:** `opencdarr/estimator.py` (`IPRResult`), `opencdarr/experiment.py`
- **Verify:** on a config with `tlos > t_lookahead` (the papers' own spawn rule is
  `tlos = 1.5 × t_lookahead`), assert `n_conflict < n_encounters` and that the conditional and
  unconditional numbers differ — pinning the distinction rather than leaving it latent.

## 5. Top-level exports

- [ ] Give `opencdarr/__init__.py` a real export list.

[[TODO]] #3 names this as the concrete blocker: "`opencdarr/__init__.py` still exports nothing, so
every import is submodule-qualified." For a *minimal user-facing interface* the export list is part
of the deliverable, not cosmetics.

- **Files:** `opencdarr/__init__.py`
- **Verify:** `from opencdarr import run_experiment, MVP, StateBased, PastCPA, M600` (final list TBD)
  works in a fresh interpreter; `examples/handbook/a_first_run.ipynb` still runs.

## 6. `run_experiment` itself

- [ ] `Fixed` / `Sweep` axes → a conditions list → run each cell → `ExperimentResult` with
      `frame()`, `cell()`, `plot()`.

Deliberately thin: no `Random`, no quadrature, no axis algebra beyond the product of `Sweep`s.
The load-bearing requirement is that `MC(...)` ↔ `IPS(...)` swap over the **same** user objects —
that is the V1.0.2 → V1.0.3 gate, and the whole reason for item 2.

Two details worth getting right now rather than later:

- **Axes carry `(levels, build, name)`**, e.g.
  `Sweep(values=[1e-5, 1e-6, 1e-7], build=lambda p: LatchingComm(p), name="p_comm_fail")`. This is
  how a contributor sweeps a parameter of *their own* component. The bare object-sweep form
  (`communication=Sweep([LatchingComm(1e-5), …])`) works but leaves the frame column full of objects
  with no numeric coordinate to plot against.
- **Metric/backend compatibility fails fast**, and the incompatible set is smaller than the design
  doc claims — see item 8.

Reuse rather than rebuild: `cache.run_key` / `cache.load_or_run` per cell (the cache §8 asks for
already exists, `.opencdarr_cache/` and all), `parallel.resolve_jobs` for `n_jobs`,
`estimator.combine_ipr` for pooling, and `cache.code_fingerprint()` on the card.

- **Files:** new `opencdarr/study.py` (or similar), `opencdarr/experiment.py`
- **Verify:** the **release-gate test** — a toy custom `ConflictResolver` *and* a toy custom
  `Dynamics`, run under `MC(...)` then `IPS(...)` with the same objects, asserting both backends
  actually *used* them (a resolver that always brakes must measurably move `min_sep` on both paths).
  This is the test that would have caught the item-2 defect. Plus: reproduce an angle-sweep profile
  and cross-check `scripts/ipr_angle_sweep.py` on a handful of angles.

## 7. A seam for stateful user models

- [ ] Add `CommunicationModel.initial_state() -> CommState` (default `CommState()`), and have
      `CnsState.initial(n)` call it.

`CommState` is a closed frozen dataclass — exactly `held` and `in_flight`. A user model that must
remember something (the motivating case: a `comm_is_on` latch that fails once and stays failed) has
nowhere to put it. Subclassing does survive the round trip, because `CNS.sense` threads whatever
`step` returns straight back into `CnsState` — but `CnsState.initial(n)` hands the model a plain
`CommState()` on the first tick, so every such model has to detect and upgrade it by hand.

~5 lines, and it is the general seam for **any** stateful contribution, which is squarely the v1
audience. Same argument applies to `SurveillanceModel` if a dead-reckoning model ever lands.

Two notes for whoever writes such a model: draw the latch from the **existing** `streams.comm`
generator — do not add a fourth substream, since ADR 0006 §6 requires the stream tree stay
config-invariant and `ips.py:_streams` pins exactly three children. And do not expect IPS to reach a
1e-6 latching failure: that is the discrete-jump pathway [[important-ips-gap]] measured collapsing
8/8 replications, because `min_sep` carries no information about whether the latch fired. Condition
on the latch time instead and reweight — see the design review for the decomposition.

- **Files:** `opencdarr/cns/base.py`, `opencdarr/cns/stack.py`
- **Verify:** `pytest tests/test_cns_communication.py tests/test_loop_cns.py` unchanged; a new test
  that a subclassed `CommState` survives a full `run_fleet` without the model upgrading it manually.

## 8. Honest IPS metrics

- [ ] Surface what IPS *can* report, and fail fast only on what it genuinely cannot.

The design doc says IPS yields nothing but `P(LoS)`. That is overstated, and contradicts
`design_brief.md`'s own requirement: "**Rare-event outputs**, not IPR: … the intermediate
level-crossing probabilities, and effective sample size."

Already computed and currently discarded:

- **The level-crossing curve.** `IPSResult.survival` is per shell, so
  `P(min_sep ≤ d_k) = Π_{j≤k} S_j` for every `k` — which *is* [[phase-9-plan]] item 1
  (`P(dcpa < 50)`, `P(dcpa < 25)`, `P(dcpa < 10)`, `P(dcpa < 5)`), free, from a single run.
- **`collapsed_at` / `n_collapsed`** — must be a loud health flag, since ADR 0017 §2 is explicit
  that `prob = 0` is not a real zero.
- **Effective sample size** — required by the brief, computed nowhere.

The correct rule is *IPS gives the rare-event probability and anything conditional on the rare set;
it cannot give unconditional population expectations* — because dropped particles are discarded and
survivors cloned. So median-dCPA-**given-LoS** is available (with the caveat that clones share
ancestry, so ESS ≪ N and there is no cheap CI); median dCPA over all encounters is not. That matches
the instinct already written down in [[phase-9-plan]] item 2 — efficiency metrics want a separate
cheap MC.

- **Files:** `opencdarr/ips.py`, plus the metric layer from item 6
- **Verify:** the level-crossing curve from one `ips_once` matches a plain-MC CCDF of `min_sep` in a
  regime where MC is feasible (the `pos=40` correctness rung).

## 9. Widen the noise-distribution signature

- [ ] `NoiseDistribution.__call__(rng, ci95)` → also receive heading and ground speed; add the two
      missing latency models.

Of Experiment 3's six position-noise models, **four need heading** and two of those also need ground
speed — the design doc says three. Worse: two are **absent entirely** (Latency, Latency+Anisotropic),
and the sole reason is that `(rng, ci95)` cannot see `ψ` or `g`. The fix is small and local:
`NavigationModel.measure(true, t, rng)` already receives the whole `AircraftState`, so both are in
hand — only the inner call at `navigation.py:47` drops them.

Not blocked by the open anisotropy question: the latency bias `−ℓg` along-track needs `ψ` and `g`
whichever way that resolves. **Only the reorientation of the two anisotropic models waits** — see
the separate investigation into whether the error ellipse rotates with track (paper) or is fixed
North/East from satellite geometry (`noise_distributions.py` / `gps-noise.md`). The two disagree, and
Experiment 3's whole point is the shape of the error.

- **Files:** `opencdarr/cns/base.py`, `opencdarr/cns/navigation.py`,
  `opencdarr/cns/noise_distributions.py`
- **Verify:** re-derive the calibration constants in the paper's `tab:noise_sigmas`
  (σ = 4.085, σ₁ = 2.776, σ_c = 1.675, …) numerically from the new signatures. Those numbers come
  from *outside* the code, which is what `design-philosophy.md` #15 asks of a test.

---

## Related

- [[run-experiment-design]] — the proposal this implements, and whose §1/§4.1/§6/§8 claims the
  review corrected
- [[TODO]] — items 2 (phase-9 metrics) and 3 (minimal user-facing interface), and the V1.0–V1.2 gates
- [[important-ips-gap]] — why a discrete-jump rare event needs a different coordinate, and the
  measured collapse
- [[0017-ips-level-and-splitting]] — the estimand, the fixed-shell obligation, and the replication CI
- [[phase-9-plan]] — the metrics items 8 partly satisfies for free
