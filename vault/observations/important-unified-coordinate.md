# ✅ IMPORTANT: one importance coordinate for both nav-drift and comms-jump rare events

**Status: RESOLVED in principle (Phase 8) — closes the open question in [[important-ips-gap]].** The
coordinate matrix there showed that no *single simple* level variable ladders both rare-event
pathways: `min_sep` ladders continuous nav drift but collapses on the discrete comms-drop pathway;
staleness ladders the drops but is identically zero under perfect comms. The note concluded the only
genuine unifier was a *learned* committor. It turns out a **hand-built committor surrogate** — the
per-pathway max — does the job, and passes all three regimes against MC ground truth.

    phi = max( nav_progress , cap * comm_progress )
      nav_progress  = clip( (d_nominal - min_sep) / (d_nominal - rpz), 0, 1 )   # 1 exactly at LoS
      comm_progress = clip( staleness / L_crit,                        0, 1 )   # blind-run depth
      cap ~ 0.9

Written 2026-07-27. Demonstrated end-to-end in `blom-comparison/opus4p8_unified_ips.py` (a minimal
encounter with both mechanisms, a brute-force MC ground truth, and the fixed-effort splitting of
[[ips-splitting-tree]] / `opencdarr.ips.ips_once`). Grew out of the Blom–Ma–Bakker reproduction in
`blom-comparison/car_ips.py` (see [[important-ips-gap]] §"the Blom connection").

    python blom-comparison/opus4p8_unified_ips.py            # full 3x3 validation
    python blom-comparison/opus4p8_unified_ips.py --mc-only  # just the MC ground truth

## The idea, plainly: one danger score 0→1

Splitting needs a monotone "how close to loss of separation" score for every particle, so it knows
which to clone at each shell. There are two unrelated ways to reach LoS, and each has its own natural
progress meter:

- **nav drift** — GPS error slowly shrinks the true gap. You can *watch* it: `nav_progress` is the
  shortfall of the running-min separation from the nominal miss toward `rpz` (0 at nominal, 1 at LoS).
- **comms blackout** — a run of dropped updates flies the resolver blind; separation looks nominal
  right up to a sudden breach. `min_sep` sees nothing until too late, but `comm_progress` (the
  consecutive-drop count, normalised) climbs the whole time.

`phi = max(...)` takes **whichever danger is further developed right now**. Perfect comms →
`comm_progress ≡ 0` → `phi = nav_progress` (the min_sep ladder). Perfect nav → `nav_progress` flat
until the breach → `phi` rides `comm_progress` (the staleness ladder). Both on → `max` ladders
whichever pathway each particle is advancing along. One score, both pathways.

## Why the cap (the `cap *` on comm_progress)

Being blind a long time is scary but is **not the same as LoS** — a blackout does not reliably cause a
breach (see the [[important-ips-gap]] staleness proof: a one-link blackout is not a reliable LoS). So
`comm_progress` must never, by itself, reach the deepest shell (`phi = 1`), or the estimator counts
*blackouts* as collisions. The cap holds `comm_progress`'s contribution to ≤ 0.9; the last band
(0.9 → 1.0) can only be filled by `nav_progress` — i.e. by the geometry *actually* breaching `rpz`.
The `ips_once` estimator returns `Π S_k` with no separate terminal factor, so this invariant
(`phi = 1 ⇔ nav_progress = 1 ⇔ min_sep ≤ rpz`) is what keeps it unbiased.

Smoke-alarm analogy: smoke (staleness) can wind the alarm up to 90% loud — good, it focuses effort on
the scary particles — but you only shout *fire* (100%) when there are real flames (a measured
near-miss). **Staleness gets a particle to the doorstep; only the real gap opens the door.**

- **cap = 1.0** → blackouts trip the deepest shell → overcount. An early pure-staleness coordinate did
  exactly this: **1.85e-3 vs a true 1.23e-4, ~15× high.** So cap must be < 1.
- **cap too low (~0.3)** → `comm_progress` can only warm the score a little, leaving a dead band up to
  1.0 that `nav_progress` (flat until the sudden breach) cannot ladder → variance blows up / collapses.
- **cap ~ 0.9** is the sweet spot and forgiving — a cap sweep on the comms regime passed for every
  value 0.2–1.0, with the CI tightening monotonically as cap rose (blind-meter spread over more
  shells). ~0.6–0.95 all safe.

## Validated on the toy (3×3, vs MC ground truth)

Minimal encounter: nav = continuous per-tick drift; comms = discrete breach after `L_crit`
consecutive drops (a cliff `min_sep` is bimodal about, so its intermediate shells hold no partial
progress). IPS 12 reps × 4000 particles × 14 shells; MC over 5e6 encounters/regime.

| coordinate | nav (drift) | comms (jumps) | nav + comms | regimes |
|---|---|---|---|---|
| `min_sep`   | PASS (5.1e-4) | **collapse** | PASS (5.1e-4) | 2/3 |
| `staleness` | **collapse**  | PASS (1.1e-5) | FAIL (1.0e-5, misses nav) | 1/3 |
| **`unified`** | **PASS (5.1e-4)** | **PASS (1.0e-5)** | **PASS (5.0e-4)** | **3/3** |

MC truths: nav 5.39e-4, comms 1.04e-5, both 5.41e-4. `min_sep` passing *both* (nav dominates the
total) is the escape-hatch of [[important-ips-gap]] confirmed — and its comms **structural zero** is
the pathway that hatch silently omits when comms is degraded enough to matter. Only `unified` tracks
MC everywhere.

## Caveats (honest, before trusting it on the fleet sim)

1. **It is a toy.** nav/comms are stand-in mechanisms chosen so `min_sep` genuinely collapses on
   comms (matching the real matrix). The MC it is checked against is real; the *dynamics* are not the
   fleet sim.
2. **`max` ignores compounding.** A particle half-developed on both pathways reads `phi = 0.5` when a
   compounding committor would be higher. Separable pathways (orders apart, as here) are fine; if the
   real *both* regime compounds strongly, switch to noisy-OR `1 - (1-nav)(1-cap*comm)`.
3. **The cap<1 correctness bite only shows when blackout ≠ LoS.** This toy has breach ⇔ LoS, so
   `cap=1.0` also passes here; the 15× overcount was the earlier toy where the deepest shell was
   staleness. The real sim (a blind run only *sometimes* breaches) is the one that will punish cap=1.

## Porting to the real sim (next)

1. **Expose staleness on `FleetState`** — a per-receiver time-since-last-update / consecutive-drop
   count (the [[communication-reception-latency]] layer already drops per directed link, so the count
   is there to accumulate). `L_crit` = the staleness at which LoS becomes *likely but not certain*.
2. **Swap the `ips_once` level function** from `min_sep <= target` to `phi >= target` with `phi =
   max(nav_progress, cap*comm_progress)` — everything else (fresh per-particle streams per shell,
   resample survivors to N) is unchanged, so the Bernoulli-correct divergence still holds.
3. **Validate against MC** on the three regimes with `P(LoS) ≈ 1e-5 … 1e-6` (rare enough to be a
   faithful test), MC run **once** (it is expensive). Accept = IPS CI overlaps MC CI in all three.

## Related

- [[important-ips-gap]] — the gap this closes; the coordinate matrix and the staleness proof
- [[ips-splitting-tree]], [[ips-gate1-correctness]], [[ips-gate2-efficiency]] — the current
  (nav-noise-only) IPS validation this extends
- [[communication-reception-latency]] — the drop/latency model that supplies staleness
