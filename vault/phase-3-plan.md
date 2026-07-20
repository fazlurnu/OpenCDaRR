# Phase 3 plan — CNS uncertainty (navigation noise + communication)

`how-to-step-by-step.md` Step 3, and the heart of the next paper: **CDR robustness under CNS
uncertainty**. Each aircraft measures its *own* state with GPS error, **broadcasts** it, and
others act on what they *received* — noisy, dropped, and **stale**. This is what the whole
directed design was built for: `detect(A, B_as_A_holds) ≠ detect(B, A_as_B_holds)`.

Same working style — one file at a time, read each diff, tick the box here.

## The model (corrected C‑N‑S decomposition)

> A measures its own state via GPS at **t1** → broadcasts one value → (reception + latency) →
> B *holds and uses* it at **t2**, by which time A has moved.

- **N (Navigation):** GPS error on an aircraft's *own* position/velocity — applied once, at the
  source, per measurement. This is what makes B's view of A noisy (A's error propagates to
  others via the broadcast, not via B's sensor).
- **C (Communication):** reception (Bernoulli — does A's broadcast reach B?) **and latency**
  (the t1→t2 delay). Its own layer, with explicit timing and in-flight/held messages — *not* a
  noise term (the old code's layering flaw). Staleness means B acts on where A *was*.
- **S (Surveillance):** what B holds — the last delivered message per source, aged by
  `t_now − t1`. This held-message set is **per-particle state** (clonable for IPS).

Asymmetry comes from independent GPS errors (A's vs B's self-measurement) + independent
per-link reception/latency — and a shared broadcast (all receivers get A's *same* measurement,
differing only by what their link delivered).

**Why latency is first-class (for IPS, v0.4):** a tail collision happens through a sequence of
discrete stochastic events, and "B reacted late because A's message was stale/dropped" *is* one
of them. Only as an explicit communication event (sent t1 / delivered t2, with in-flight + held
messages as clonable state) can IPS branch on it. Baked into noise, it's unrepresentable.

## Scope of this pass

Functional: build the layers and verify **behaviour** — IPR degrades as CNS uncertainty grows,
runs are reproducible given the seed, and the comm state is explicit and clonable. **The sim
becomes stochastic given geometry** (each encounter is one noise/reception realization), so
plain-MC now samples geometry × CNS.

**Exit gate:** functional tests pass; IPR is a monotone-ish decreasing function of GPS CI95 /
reception loss; reproducible from `config + seed`; no shared RNG across components.

**✅ Met (2026-07-20), end to end.** All four conditions verified through the loop, not just in
isolated layer tests — see the Checklist below.

## Scope — what Phase 3 is NOT

- [x] ~~**Own-state used in an aircraft's *own* detection** — an aircraft uses its **true** own
  state for its own awareness (own GPS error treated negligible, as the old code did)~~
  **REVERSED, validated against BlueSky:** both endpoints of a decision are the aircraft's own
  *noisy self-measurement* — `_decide(self_own, tx_intr, …)` in `loop.py` uses `fix_own =
  navigation.measure(own, …)`, not the true `own`. This was the fix for the broadcast-cadence /
  self-noise bug in [[near-parallel-ipr-inversion]] — the reference (BlueSky) decides on
  broadcast-own vs. broadcast-intr, and matching that closed the IPR inversion. Kept here as a
  correction, not silently dropped, since this doc originally recommended the opposite.
- [ ] **The ADSL protocol dance** — we model its *effect* (noisy, dropped, stale surveillance),
  not the 4-node message protocol (`lesson-learnt.md`).
- [ ] **Heavy-tail / anisotropic distributions** — Gaussian first (3a); the pluggable family is
  3c.
- [ ] **IPS itself** — v0.4. Phase 3 keeps plain MC; it just makes the model stochastic and the
  comm state clonable, *ready* for IPS.

## Phasing

- **3a — Navigation only.** Broadcast a GPS-noisy self-state; perfect reception, zero latency.
  IPR degrades with GPS CI95 — the first robustness curve.
- **3b — Communication.** Reception probability `< 1` **and** latency (delay + staleness), with
  in-flight/held messages as explicit state.
- **3c — Distribution family.** Pluggable GPS-noise distributions (mixture, anisotropic).

---

## Checklist

Each item: **path · purpose · design justification · check · relations.**

### 3a — navigation ✅ done

- [x] **`opencdarr/cns/base.py`** — interfaces (the contribution surface):
  - `Message` (frozen): `source: str`, `state: AircraftState` (the noisy self-measurement),
    `t_meas: float`.
  - `NoiseDistribution` — pluggable `(rng, ci95, trk) -> (err_x, err_y)` (a new distribution
    adds a file, per the brief).
  - `NavigationModel` (ABC): `measure(true, t, rng) -> Message`.
  - `CommunicationModel` (ABC): pure transform on the comm state (see 3b) — **plus
    `SurveillanceModel`**, added when 3b actually landed (not originally listed here).
- [x] **`opencdarr/cns/noise_distributions.py`** — `gaussian` (isotropic, CI95-parameterised);
  mixture / anisotropic deferred to 3c.
- [x] **`opencdarr/cns/navigation.py`** — `GpsNavigation()`: **signature evolved from the plan**
  — `pos_ci95`/`vel_ci95` are no longer constructor args, they're read off the `AircraftState`
  being measured (moved onto the state so accuracy can differ per aircraft and evolve over a
  run, same as `turn_rate`; see `vault/derivations/gps-noise.md`). Applies position + velocity
  noise to the true state → a `Message`, and copies the same ci95 onto the broadcast. Pure
  given `rng`.
  - *Check:* mean error ≈ 0, spread matches CI95; reproducible per substream. ✅
    `tests/test_cns_navigation.py`.
- [x] **Loop integration (3a):** `run_encounter` gains an `rng` + a navigation model; each
  broadcast tick every aircraft measures+broadcasts, and **both endpoints of a decision are
  noisy self-measurements** (see the corrected "what Phase 3 is NOT" item above — own state is
  *not* kept true).
  - *Check:* with zero noise the Phase-2 result is reproduced exactly (a free regression) ✅;
    IPR falls as CI95 rises ✅. `tests/test_loop_cns.py`.

### 3b — communication (reception + latency) ✅ done

- [x] **`opencdarr/cns/communication.py`** — `CommState` (frozen): `held[(receiver, source)] ->
  Message`, `in_flight: tuple[InFlight, …]`. `Comm(reception_prob, latency)`: pure
  `step(state, broadcasts, receivers, t, rng) -> CommState` — draw reception per directed link
  (scalar or a `(source, receiver)` mapping, so links can be asymmetric — e.g. 0.8 one way,
  0.99 the other), draw latency only if received, enqueue in-flight, deliver those due, update
  `held` **freshest-by-`t_meas`** (a late message can't regress a receiver past what it already
  has — settled in ADR 0006 §4, needed once latency can exceed the broadcast interval).
  - *Check:* `p=1, latency=0` → instant delivery (reduces to 3a) ✅; dropped messages → receiver
    keeps the stale held message ✅; delivery time = `t_meas + latency` ✅; Bernoulli rate and
    lognormal-latency both calibrated ✅. `tests/test_cns_communication.py` (13 tests). Visually
    validated: [[communication-reception-latency]].
- [x] **`opencdarr/cns/surveillance.py`** — `LastKnown.perceived(state, receiver, source,
  t_now) -> AircraftState | None`: **settled as hold-as-is, not dead-reckoned** (ADR 0006 §2 —
  a stale message is used unchanged; dead-reckoning would assume the source kept flying
  straight, wrong exactly when it just started maneuvering). `age()` is a separate function for
  instrumentation only — it doesn't change what's perceived.
  - *Check (revised from the plan's dead-reckon assumption):* perceived state is byte-identical
    regardless of `t_now` (no drift); `age` resets to exactly 0 at delivery and grows linearly
    between. ✅ `tests/test_cns_surveillance.py`. Visually validated: [[surveillance-hold-as-is]]
    — a continuously-changing truth vs. a perceived value that only ever steps at deliveries.
- [x] **Loop integration (3b).** `run_encounter` gains `communication`/`surveillance`/`comm_rng`
  params (all optional — `communication=None` preserves 3a behaviour exactly, byte-for-byte,
  verified by every pre-existing test still passing unmodified). When set: each broadcast is
  offered to `communication.step(...)` (its own `comm_rng`, never `rng`) before a decision's
  *other* is read via `surveillance.perceived(...)` (defaults to `LastKnown()` when
  `communication` is set but `surveillance` isn't); `_decide` now accepts `other: AircraftState |
  None` and flies nominal on `None` (ADR 0006 §5). Intent-sharing is stripped **before**
  broadcast, not at perceive time, so a held/stale message never carries intent it wasn't sent
  with. So `detect(A, B_as_A_holds) ≠ detect(B, A_as_B_holds)` (this doc's opening line) is now
  actually driven by reception/latency, not just independent GPS noise. Visually validated end
  to end: [[loop-communication-integration]].
  - *Check:* reception loss degrades IPR (needs to drop quite low — p ≈ 0.03/0.005 — before it
    bites, since the ~60s encounter window is long relative to the 1 Hz broadcast interval;
    calibrated empirically) ✅; latency alone (perfect reception) also degrades IPR ✅; `p=1,
    latency=0` is bit-identical to no communication ✅; reproducible from seed ✅; adding
    communication does not perturb navigation's draws (independent substreams, ADR 0006 §6) ✅.
    `tests/test_loop_cns.py` (7 new tests).

### Vault

- [x] **RNG substream layout — documented and wired into code.** The plan's `0005-cns-rng-
  substream-layout.md` never got its own file; the tree (`encounter → spawn(3) → geom_seq,
  nav_seq, comm_seq`) is recorded instead in **ADR 0006 §6**
  ([[0006-communication-model-design]]) — `0005` was already taken by
  [[0005-trajectory-validated-against-bluesky]] by the time 3b started. `estimator.py` now does
  `spawn(seq, 3)` unconditionally (config-invariant tree, ADR 0006 §6) and passes `nav_seq` as
  `rng`, `comm_seq` as `comm_rng`; verified independent (`test_communication_and_navigation_are_
  independent_substreams`).
- [ ] **`vault/decisions/0006-latency-in-communication-not-noise.md`** — not split out as its
  own ADR; the reasoning ("why latency is first-class") is currently inline in this doc's intro
  and in ADR 0006's Context section. ADR 0006's Relations section flags this as still
  splittable if warranted — hasn't been judged necessary yet.
- [x] **`vault/derivations/gps-noise.md`** — CI95 → per-axis σ (2D radial), position + velocity
  error, the pluggable-distribution signature, **and** where CI95 lives (`AircraftState`, not
  `GpsNavigation` — added when ci95 moved onto the state).
- [ ] **`vault/derivations/comm-latency.md`** — not written as its own file. Its content
  currently lives split across ADR 0006 (delivery/timing model) and the two observation docs
  ([[communication-reception-latency]] for the t1→t2/staleness numbers,
  [[surveillance-hold-as-is]] for the hold-as-is proof) — may turn out not to need a separate
  derivation doc given that coverage; revisit once loop integration is done and there's a
  complete picture.

### Tests

- [x] `test_cns_navigation.py` — noise is zero-mean, CI95-calibrated, reproducible; broadcast
  declares the source's own accuracy.
- [x] `test_cns_communication.py` — reception Bernoulli rate (incl. per-link/asymmetric), latency
  delivery timing (incl. lognormal), stale hold on drops, freshest-by-`t_meas` ordering guard,
  `p=1,latency=0` reduces to instant, purity (no mutation).
- [x] `test_cns_surveillance.py` — perceived is unchanged regardless of `t_now` (hold-as-is, the
  opposite of the plan's original dead-reckon assumption — see 3b above); `age` resets at
  delivery.
- [x] `test_loop_cns.py` — zero-noise reproduces Phase 2 (regression) ✅; IPR falls monotone-ish
  with CI95 ✅ **and with reception loss / latency** ✅; `p=1,latency=0` bit-identical to no
  communication ✅; reproducible from seed (navigation-level and comm-level) ✅; navigation and
  communication substreams verified independent ✅.

---

## Decisions to settle before building (nail these first, like cd/cr/crr)

All seven now settled:

1. **`Message` shape** — settled as planned: `state: AircraftState`.
2. **Communication delivery/hold semantics** — settled in ADR 0006: `CommState{held, in_flight}`,
   `Comm.step(state, broadcasts, receivers, t, rng) -> CommState`.
3. **Dead-reckon vs hold** — settled as **hold-as-is** (the plan leaned dead-reckon in its 3b
   checklist text above; that was superseded — see ADR 0006 §2 for why).
4. **Own-state in own detection** — settled as **own GPS measurement**, reversing this doc's
   original "true (recommended)" lean. Validated empirically against BlueSky during the
   near-parallel-IPR investigation ([[near-parallel-ipr-inversion]]): the reference decides on
   broadcast-own vs. broadcast-intr, and matching that fixed the IPR inversion. See the corrected
   "what Phase 3 is NOT" item above.
5. **Broadcast interval** — settled as a fixed rate: `broadcast_interval` param (default 1.0 s),
   now wired through `SimulationConfig`.
6. **CI95 parameterisation** — settled as planned (kept CI95), and taken further: it's now a
   per-aircraft `AircraftState` field, not a fixed simulation-wide constant.
7. **RNG substream layout** — settled in ADR 0006 §6 (not ADR 0005 — see the Vault section
   above); `spawn(3)` code wiring still pending.

## Relations to the companion docs

- `design_brief.md` — CNS as pure functions with RNG threaded; the perceived/comm state is the
  "per-particle CDR/CNS state" the brief names; sets up IPS.
- `design-philosophy.md` — pure by default with effects/RNG at the edges (#1), every stochastic
  thing its own RNG (#3), one owner of state (#2).
- ADR 0004 — the directed design that makes surveillance asymmetry expressible here.

## References (read, not ported)

- `CDaRR_git/sim_models/noise_model.py` (position/velocity Gaussian, CI95, latency bias — which
  we relocate to communication), `noise_distributions.py` (gaussian/mixture/anisotropic),
  `reception_model.py` (Bernoulli reception), `cns_adsl.py` / `adsl_module.py` (the protocol —
  effect only). Papers: Schaefer & Jonas (ADS-B noise).
