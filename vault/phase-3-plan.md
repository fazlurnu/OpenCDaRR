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

## Scope — what Phase 3 is NOT

- [ ] **Own-state used in an aircraft's *own* detection** — an aircraft uses its **true** own
  state for its own awareness (own GPS error treated negligible, as the old code did); GPS error
  matters for how *others* see it. Modelling own-perception error is deferred.
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

### 3a — navigation

- [ ] **`opencdarr/cns/base.py`** — interfaces (the contribution surface):
  - `Message` (frozen): `source: str`, `state: AircraftState` (the noisy self-measurement),
    `t_meas: float`.
  - `NoiseDistribution` — pluggable `(rng, ci95, trk) -> (err_x, err_y)` (a new distribution
    adds a file, per the brief).
  - `NavigationModel` (ABC): `measure(true, t, rng) -> Message`.
  - `CommunicationModel` (ABC): pure transform on the comm state (see 3b).
- [ ] **`opencdarr/cns/noise_distributions.py`** — `gaussian` first (isotropic, CI95-parameter-
  ised); mixture / anisotropic deferred to 3c.
- [ ] **`opencdarr/cns/navigation.py`** — `GpsNavigation(pos_ci95, vel_std)`:
  applies position + velocity noise to the true state → a `Message`. Pure given `rng`.
  - *Check:* mean error ≈ 0, spread matches CI95; reproducible per substream.
- [ ] **Loop integration (3a):** `run_encounter` gains an `rng` + a navigation model; each step
  every aircraft measures+broadcasts, and each other acts on the **perceived** (received)
  state. Own state stays true for its own detection.
  - *Check:* with zero noise the Phase-2 result is reproduced exactly (a free regression);
    IPR falls as CI95 rises.

### 3b — communication (reception + latency)

- [ ] **`opencdarr/cns/communication.py`** — `CommState` (frozen): `held[(receiver, source)] ->
  Message`, `in_flight: tuple[(message, deliver_t, receiver), …]`. `Comm(reception_prob,
  latency)`: pure `step(comm_state, broadcasts, t, rng) -> CommState` — draw reception, draw
  latency → enqueue in-flight, deliver those due, update held. **Model is pure; state is
  threaded** (the invariant — clonable for IPS).
  - *Check:* `p=1, latency=0` → instant delivery (reduces to 3a); dropped messages → receiver
    keeps the stale held message; delivery time = `t_meas + latency`.
- [ ] **`opencdarr/cns/surveillance.py`** — `perceived(held, t_now) -> AircraftState`: the held
  message's state, **dead-reckoned** by age `t_now − t_meas` (or held as-is — a decision).
  - *Check:* staleness makes the perceived position lag the true position by ≈ `age · gs`.
- [ ] **Loop integration (3b):** thread `CommState` through the encounter; decisions use
  `perceived(held, t)`. `CommState` becomes part of the (future) particle.
  - *Check:* reception loss / latency degrade IPR; reproducible.

### Vault

- [ ] **`vault/decisions/0005-cns-rng-substream-layout.md`** — `encounter → {geometry, and per
  aircraft: gps; per link: reception, latency}`; documents the tree so the ADSL shared-RNG bug
  cannot recur (ADR 0001 applied). 
- [ ] **`vault/decisions/0006-latency-in-communication-not-noise.md`** — record *why* latency is
  its own layer (timing event, clonable comm state, IPS branching) rather than a noise bias.
- [ ] **`vault/derivations/gps-noise.md`** — CI95 → per-axis σ (2D radial), position + velocity
  error; the pluggable-distribution signature.
- [ ] **`vault/derivations/comm-latency.md`** — the t1→t2 delivery model, staleness /
  dead-reckoning.

### Tests

- [ ] `test_cns_navigation.py` — noise is zero-mean, CI95-calibrated, reproducible.
- [ ] `test_cns_communication.py` — reception Bernoulli rate, latency delivery timing, stale
  hold on drops; `p=1,latency=0` reduces to instant.
- [ ] `test_cns_surveillance.py` — perceived staleness lags true by `age·gs`.
- [ ] `test_loop_cns.py` — zero-noise reproduces Phase 2 (regression); IPR falls monotone-ish
  with CI95 and with reception loss; reproducible from seed.

---

## Decisions to settle before building (nail these first, like cd/cr/crr)

1. **`Message` shape** — `state: AircraftState` (convenient for detect/resolve) vs a lighter
   position/velocity record. Lean: `AircraftState`.
2. **Communication delivery/hold semantics** — the `CommState` structure (held + in-flight) and
   the pure `Comm.step` signature. *This is the part IPS leans on — settle it carefully.*
3. **Dead-reckon vs hold** the stale estimate — extrapolate the held message forward by its age,
   or use it as-is?
4. **Own-state in own detection** — true (recommended) vs own GPS measurement.
5. **Broadcast interval** — every step (dt), or a fixed broadcast rate (≈1 Hz)?
6. **CI95 parameterisation** — keep CI95 (ADS-B convention, old code) over σ.
7. **RNG substream layout** — the tree in ADR 0005.

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
