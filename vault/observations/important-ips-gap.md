# ⚠️ IMPORTANT: the IPS estimator has never seen CNS uncertainty

**Status: HIGH PRIORITY — open gap (Phase 8).** Everything needed to drive the rare event with
communication uncertainty already exists in the code; the estimator has simply never been *run or
validated* with it switched on. Until it is, our IPS numbers only cover nav-noise-driven rare
events, and we do not know whether fixed-effort splitting still holds up when the rare event is
caused by dropped/late messages. Written 2026-07-26.

**Update 2026-07-27 — largely RESOLVED, see [[important-unified-coordinate]].** The coordinate-matrix
question (no single simple coordinate ladders both pathways) is answered by a hand-built committor
surrogate, `phi = max(nav_progress, cap*comm_progress)`, validated *once* against MC on the real
`FleetEnv` sim (`scripts/ips_unified_validate.py`) across all three regimes: nav 2.1e-4 (MC 2.7e-4),
comms 1.4e-3 (MC 1.2e-3), both 2.6e-2 (MC 2.7e-2) — every IPS CI overlaps MC. Two caveats carried
forward there: comms LoS needs *both* aircraft blind (so staleness = time since any link delivered,
not worst-link age), and the comms cell is high-variance because `comm_progress` ladders blind
*duration* but not blind-*at-CPA* — a CPA-timed coordinate is the open next step for reaching 1e-5/1e-6.

## The gap

The pieces are all in place and wired:

- **The models exist.** `Comm` does Bernoulli reception per directed link plus a drawn latency
  (`opencdarr/cns/communication.py`), with `constant_latency` / `uniform_latency` /
  `lognormal_latency`. Demonstrated in `surveillance_demo`, `comm_ipr_sweep`,
  `broadcast_jitter_demo`. See [[communication-reception-latency]].
- **The RNG plumbing exists.** `ips_once` spawns a `comm` substream per particle per level
  (`opencdarr/ips.py`, `_streams`), and splitting already acts on forward noise. So the moment a
  scenario sets `reception_prob < 1` or nonzero latency, **splitting acts on comms noise for free**
  — no estimator change required.
- **But it has never been exercised.** `scripts/ips_validate.py` builds envs with
  `GnssNavigation()` only — no `Comm` with drops or latency. So **every IPS-vs-MC validation to
  date is nav-noise-driven** ([[rare-event-validation-ladder]], [[ips-gate1-correctness]],
  [[ips-gate2-efficiency]]). Comms-driven rare events are untested in the estimator.

## Why it matters (the Blom connection)

This is exactly the boundary between our simplified estimator and Blom & Bakker's full method (see
`vault/papers/rare-event-sim/`). We nest levels on the running-minimum separation `min_sep`, a
*continuous, monotone* importance function. That is well-matched to a rare event driven by
continuous nav noise. **Reception is different: it is a discrete Bernoulli event per link per
tick.** With drops, loss of separation is often caused by a *run of consecutive missed updates* — a
discrete cause that `min_sep` is only loosely coupled to. Separation can stay large right up until
perception goes stale and the aircraft are suddenly close.

The concrete risk: fixed-effort splitting on `min_sep` may give **weak variance reduction over the
drop-burst pathway**, so IPS could **underestimate vs brute MC** (it never resolves the discrete
pathway) or **collapse levels**. This is precisely the regime the Blom–Ma–Bakker 2018 IPS paper
addresses with Bernoulli sampling of the jump events —
`vault/papers/rare-event-sim/interacting-particle-system.pdf`.

## Demonstrated (2026-07-26): the discrete-jump collapse, with a closed-form ground truth

Wiring `Comm` into `scripts/ips_validate.py` (`--reception` / `--latency`) and running the CNS-only
rung confirmed the gap concretely. Setup: perfect nav (`--pos 0`), fixed 90° crossing, spawned just
**outside** the detection horizon (`--tlos 65 --lookahead 60`) so the pair starts with no detected
conflict and the blind window is a clean ~60 broadcasts.

- The rare event is **a run of consecutive dropped broadcasts**. With `broadcast_interval = 1 s`,
  P(ownship never hears the intruder across the 60 s window) `= (1 - rx)^60`. At `rx = 0.15` that is
  `0.85^60 ≈ 5.8e-5` (one link; `≈ 3e-9` if LoS needs *both* links blind). So MC at 5000 expects
  ~0.3 events and reads **0** — twice — and at 1000 particles IPS **collapsed all 8 replications**.
- **Why it collapses:** we split on `min_sep`, but separation does **not** decrease monotonically
  with the number of consecutive drops. A particle 30 drops into a blind run looks identical, in
  `min_sep`, to one 3 drops in — until the run is long enough to breach. The intermediate shells
  (80, 72, 65 m…) hold no partial progress toward the discrete cause, so every deep shell has zero
  survivors. **Adding particles does not fix it** — the importance function is blind to the pathway.
- Confirmed it is the *importance function*, not a no-op: perfect comms gives a degenerate
  `min_sep` (104.2 m, std 0); `rx = 0.15` spreads it (mean 138, min 91, max 220, std 26). Drops do
  perturb the forward evolution — the ladder just cannot climb toward the discrete tail.

This hands us a **closed-form benchmark** for any fix: `(1 - rx)^60`. Choosing `rx = 0.06` lifts the
event to `0.94^60 ≈ 0.024` (one link) / `≈ 6e-4` (both) — inside MC's reach, so we get a real CI to
validate a new estimator against.

## Proven (2026-07-26): staleness splitting recovers (1-rx)^W exactly

`scripts/staleness_proof.py` validates the estimator *machinery* against a closed form, isolated from
resolver physics. The rare event is a **communication blackout** — one directed link drops all `W`
broadcasts — with exact probability `(1 - rx)^W` (P(LoS) has no clean closed form here, since a
one-link blackout does not reliably cause LoS; the blackout event does, and exercises the same
machinery). The level variable is **staleness** = consecutive-drop count, the accumulating discrete
cause. Each shell's conditional survival is exactly `1 - rx`, so the product over `W` shells is
`(1 - rx)^W`. Drop semantics match `Comm` (dropped iff `rng.random() >= reception_prob`); the
fixed-effort loop mirrors `ips_once` (fresh per-particle stream per shell, resample survivors to N).

| regime | theory | staleness splitting | plain MC |
|---|---|---|---|
| `rx=0.15, W=60`  | `0.85^60 = 5.82e-5` | **5.82e-5** (0.1% off, theory in CI), survival/shell 0.8500 | 1 event / 20k, CI 30× wider |
| `rx=0.15, W=120` | `0.85^120 = 3.39e-9` | **3.32e-9** (2.2% off, theory in CI), survival/shell 0.8499 | **0 events / 20k** |

So the splitting arithmetic and its variance reduction are correct arbitrarily deep into the tail,
on exactly the discrete pathway `min_sep` was blind to. This is Blom's "split the geometry / sample
the Bernoulli jumps" run in its purest form. **Caveat:** this proves the *machinery* on staleness; it
does not yet estimate LoS, and staleness is comms-specific — the unified-coordinate question (below)
is still open.

## The coordinate matrix — no single simple coordinate unifies (2026-07-26)

Tested each candidate importance function in each regime, against MC ground truth:

| coordinate | nav (drift), pos=40 | comms (jumps), rx blackout |
|---|---|---|
| **min_sep** (actual separation) | ✅ PASS — 0.00493 vs MC 0.00475, 0 collapsed | ❌ collapses (flat until CPA) |
| **dcpa-shortfall** `∫max(0,rpz-dcpa)dt` (≈ staleness) | ❌ collapses — P=0 vs 0.00475, 6/8 collapsed | ✅ recovers `(1-rx)^W` exactly |
| **raw dcpa** | ❌ (reasoned: dcpa₀≈0 on a collision course) | ❌ (flat at 0) |

**Conclusion:** each regime needs a coordinate matched to *its* randomness — nav noise drifts the
**geometry** (min_sep ladders), comms drops jump the **information** (staleness ladders); they are
structurally different. dcpa fails in *both* (collision-course start / flat). The only genuinely
unifying coordinate is the **committor**, which would have to be *learned* (research). Blom's stance
supports keeping one geometric importance function rather than building per-mechanism coordinates.

**Practical escape hatch:** the pathways differ by orders of magnitude — nav-driven LoS ~5e-3 vs
comms-blackout LoS (both links, rx=0.15) ~3e-9. In a realistic combined scenario the total P(LoS) is
dominated by the nav pathway, so **min_sep alone is likely adequate for the total** whenever comms is
not catastrophically degraded. Staleness earns its keep only for the *pure comms-blackout*
probability, or when comms is unreliable enough that blackout-LoS rivals nav-LoS. Next check
(untested): confirm min_sep captures the total in the combined nav+comms regime.

## The fix direction: dcpa, not min_sep (2026-07-26) — SUPERSEDED

> **Superseded by "The coordinate matrix" above.** This section captured the initial dcpa hypothesis
> (dcpa as a leading indicator). It held as a *discriminator* but failed as a *splitting coordinate*
> in both regimes; kept for the reasoning trail. The matrix section is the current conclusion.

`min_sep` is a **lagging** signal — it only moves near CPA, after the drop damage is done, so its
shells cannot stratify *how badly resolution is failing*. **dcpa (predicted miss distance) is a
leading signal**: at every instant it reports how much miss distance the resolver has managed to
build, which is exactly what drops attack. It shares units with `rpz` (plugs into the shell
machinery) and degrades under nav noise too, so it could be **one importance function for both the
nav-noise and comms regimes** where `min_sep` only ever worked for nav noise. dcpa is computable
from `FleetState.states` via `relative_enu` + the detector's CPA formula
(`t_cpa = -(r·v)/|v|²`, `dcpa = |r + t_cpa·v|`).

**The catch — polarity.** Our encounters start on a collision course (`dcpa_max = 0`), so *true*
dcpa ≈ 0 at t=0 and the resolver *grows* it. "dcpa small" is therefore the **starting condition**,
not the rare set; the rare event is "**dcpa fails to grow**," the opposite polarity to "min_sep
shrinks." Raw running-min dcpa is ~0 for everyone and will not ladder. And physically, a blind
ownship extrapolating a **constant-velocity** intruder is *accurate* — drops only bite when the
intruder **maneuvers** and the stale picture causes a coordination failure. So the sharp signal is a
**deviation-from-safe** measure, which starts at 0 for all particles and grows only for the
afflicted. Two candidate importance functions that keep the dcpa instinct but fix the polarity:

1. **dcpa deficit vs nominal** — `dcpa_nominal(t) − dcpa_true(t)`, nominal = the perfect-comms
   (deterministic when `pos=0`) resolution trajectory. Scenario-agnostic; works for nav noise too.
2. **Perceived-vs-true dcpa gap** — `|dcpa_perceived − dcpa_true|`. Directly measures the
   stale-picture coordination failure; grows monotonically during a blind run. Comms-specific.

Next: prototype (1)/(2) as the level variable, validate on the nav-noise gate (must still match MC,
proving unification) **and** on `rx = 0.06` against `0.94^60`.

## What to do next, in order

1. **Plumb `Comm` into the IPS `build_initial`** and re-run MC-vs-IPS validation with
   `reception_prob < 1` / nonzero latency. Cheapest, highest-value step; confirms whether IPS still
   matches brute MC when the rare event is comms-driven.
2. **Add a "CNS-only" rung to the validation ladder** ([[rare-event-validation-ladder]]): perfect
   nav, `reception_prob < 1`, so LOS is driven *purely* by discrete dropped updates. Isolates the
   exact case where Blom's Bernoulli-sampling result matters and gives clean MC ground truth.
3. **Watch for the discrete-jump failure mode.** If step 2 shows IPS underestimating or collapsing
   levels, the fix is a **design decision on the importance function**: level on something that also
   reflects comms staleness (e.g. combine `min_sep` with perceived-vs-true position error, or
   time-since-last-update), so particles accumulating drops get selected *before* they are already
   in LOS. That closes the gap between our `min_sep`-only nesting and Blom's GSHS reach-set
   factorization.
4. **(Minor, later)** Note that per-tick comm-stream consumption is data-dependent (one reception
   draw per directed link; latency drawn only if received). Harmless for correctness — per-particle
   streams already isolate clones — but worth a one-line comment.

**Recommendation:** do 1 + 2 first. They are small and either confirm we are fine or surface the
importance-function problem concretely instead of in the abstract. Hold off on redesigning the level
variable until the CNS-only rung actually shows splitting degrading.

## Related

- [[communication-reception-latency]] — the reception/latency model this would switch on
- [[rare-event-validation-ladder]] — where the CNS-only rung belongs
- [[ips-splitting-tree]], [[ips-gate1-correctness]], [[ips-gate2-efficiency]] — the current
  (nav-noise-only) IPS validation
