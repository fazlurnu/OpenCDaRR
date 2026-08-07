# TODO — IPS wall-clock: what remains after the sharding commits

Context: the 2026-08 validation campaign showed IPS wall time was not the splitting but the
phases `_lockstep` ran serially in the parent while the pool sat idle — the tail leg above all
(~90 % of a fleet cell: every continuation flies to `t_max` on a ring that never clears), then
the initial cloud build (84 s of a random-traffic cell). `6c54068` and `cfcc77a` shard both over
the pool, bit-identical to the serial estimator. What follows is what those commits deliberately
did **not** touch, ordered cheap-and-certain first.

Numbers cited below come from the campaign log (100-core box) and from M2 Air probes of the same
geometries (ladders taken verbatim); the function-level profile is ring n = 8, full ladder + tail.

## 1. Re-validate, then relaunch the campaign

- [ ] Re-run one 40 m fleet cell with `--no-cache` on the server (`ring n=8` expected
      ~2 293 s → ~200 s, `random d=5` ~6 821 s → ~500 s). Confirms the projection before paying
      for the rest.
- [ ] Relaunch the remaining conditions — `random_traffic density=10` (25 aircraft, 300 pairs)
      never finished; it was bounded by the serial tail and should now be bounded by its sharded
      ladder.
- Note: the `--jobs 10` workaround (any divisor of `reps` takes the whole-replication path, tail
  inside workers) is **superseded** by the sharding commits. Keep it in mind only as a fallback
  if lockstep misbehaves.

## 2. Tail policy — whether it runs, and how far

The tail's only products are K and A (`p_los_ac`, `mean_k`); `tail=False` changes no other
number (`experiment/backends.py`).

- [ ] **Anchor rung:** decide whether the campaign should pass `tail=False` at 40 m. The anchor
      exists to check agreement on `p_los`, and MC is anchored there anyway — flying 20 000
      continuations to re-measure K/A that MC already measures may be pure cost. Target rungs
      keep the tail; it is the only K/A source where MC starves.
- [x] **Early stop — measured 2026-08-07, and declined** (`scripts/early_stop_probe.py`; rings
      n=6/8 on the 1500 m ring plus a random-traffic density sweep n = 5/8/10/15 on the 900 m
      disc; 30 m rung; thresholds 0.5/2/5 s of sustained clear; ~1 600 encounters). K and A
      never moved after a single clear step anywhere — but the sweep shows *why*, and it is not
      that dense traffic is benign: the predicate is **global**, and its firing rate collapses
      with density (69 % of encounters at n=5 → 14 % → 4 % → **0 % at n=15**, where every
      encounter dies at `t_max`). An aircraft clearing one encounter and meeting another — the
      real sequential-conflict mechanism in dense traffic — keeps some pair converging, so the
      fleet-level clear never arrives; where it can arrive, it means the wave has dispersed and
      nobody is left to meet. So the stop is *vacuously* safe where it fires and unavailable on
      the expensive fleets (ring n=8: 28/300; savings elsewhere 13–34 s of sim, a few percent
      of wall post-sharding). The running minimum is **not** absorbing regardless — a 24 m
      `min_sep` dip arrived after five seconds of sustained clear (n=5) — so the level legs are
      disqualified outright. Not implemented. **Scope caveat:** all of this is a property of
      the encounter-based design (one wave through a disc, no arrivals). In continuous traffic
      a global clear would be followed by fresh conflicts from new entrants; if that scenario
      class is ever added, this measurement must be redone there.

## 3. Geodesy / per-step cost — ideas parked, not committed

The deep cut, kept as a list until measured designs exist. Profile (ring n = 8): `geo.qdrdist`
2.43 M calls, ~39 % of cumulative time; the `relative_enu` pipeline ~57 %; ~101 `qdrdist` per
`advance`, i.e. ~3.6 per pair per step, roughly half of them recomputing geometry already
computed that step. `dataclasses.replace` churn is another ~15 %.

- [x] **Done 2026-08-07 (with the `_all_clear` reuse and a third, unplanned one — per-aircraft
      velocities computed once inside `pairwise_relative`).** The post-step geometry rides
      `FleetState` as a derived cache: `compare=False`, dropped on pickle, `None` recomputes —
      so the value semantics, the wire format and ADR 0004 are untouched. Verified bit-identical
      on the MC record (540 encounters, every `min_sep`), on every IPS field serial *and*
      lockstep (the pickle boundary included), and on step counts; full suite green. Measured
      per step: **−3–5 % at n=2, −17–19 % at n=8, −22 % at 13 aircraft** — every backend pays
      this path, so MC anchors gain the same.
- [x] **Detection-sweep halving — declined on inspection.** The true-state sweep is guarded by
      the *sticky* `conflict` flag, so it already runs ~once per encounter; the ~28 `detect`
      calls/step in the profile come from `separation.step` — each observer's own *perceived*
      picture, where `i`'s view of `j` and `j`'s view of `i` are different noisy states with no
      symmetry to share. (And `qdrdist` uses `earth_radius(lat1)`, so even the true-state pair
      is not bit-symmetric — the halving would have been boundary-equivalent, not identical.)
      That cost belongs to the vectorisation below or a separation-layer redesign.
- [ ] **Flat-ENU fast path — benchmarked 2026-08-07, gain confirmed, adoption deferred (user:
      "maybe later").** Tangent plane at ownship (`ry = dlat·R`, `rx = dlon·R·cos(lat_own)`,
      same WGS84 radius), reaching `relative_enu` everywhere the CDR stack reads it — detect,
      MVP, PastCPA, the pairwise sweep — which is the pool vectorisation could not touch. The
      error argument, measured: quadratic `d²/2R` — 0.1 mm at rpz range, 3.5 mm at 300 m,
      0.9 m at 3 km spawn range; four orders under the noise floor at every decisive radius.
      End-to-end (monkeypatched, campaign stack): **−17 % per step at n=8, −24 % at 13
      aircraft** on top of the phase-1 reuse — ≈ −30 %/−38 % combined vs the original.
      Blocked on a *methods* decision, not code: the CDR layer would measure separation on a
      local tangent plane while aircraft still fly the sphere — a deliberate departure from the
      BlueSky-mirroring geodesy of ADR 0003, deserving its own ADR + observation + the
      re-baseline protocol (suite, serial≡parallel, MC/IPS statistical shift checks, one
      re-anchored campaign cell). Prototype: `brouillon/bench_flat_enu.py`.
- [x] **Vectorise the pairwise geometry — prototyped 2026-08-07, measured, and parked.** A
      condensed-upper-triangle numpy `pairwise_relative` (formula-for-formula `qdrdist`,
      `np.triu_indices` in `pair_ids` order, floats within 4e-16 of scalar) benchmarked against
      the post-phase-1 scalar path: **0.05× at n=2, 0.98× at n=8, 1.59× at n=13, 2.24× at
      n=28** — numpy's ~30 µs fixed overhead swamps arrays this small, and the crossover sits
      near n=13 where the sweep is only ~a fifth of the step, so the whole-step gain is −7 %
      at n=13 and ~−12 % at n=28. Not worth a re-baseline while campaign fleets stay ≤ 13
      aircraft. Revisit if n ≥ 20 cells become routine — and then batch the *whole* measurement
      block (post sweep + `pair_min_ranges` + the LoS mask) rather than the sweep alone, which
      is where the remaining Python-object churn lives.
- [ ] Open question, not an action yet: does separation need *measuring* every `dt`? The
      segment-interpolation argument in `relative.py` says thinning the measurement grid
      re-opens the one-sided bias `segment_min_sep` exists to close, and the splitting radii are
      exactly where it bites (`(v_rel·dt)² / (24 d²)`). Any thinning proposal needs that bound
      re-derived for the coarser grid before it touches the estimator.

## 4. P(mission completed) — parked until the scenarios can carry it

`SimulationConfig.stop_within` now threads to both backends (locked by
`tests/test_stop_within.py`), so the mission-completion *stop* is declarable. The *metric* is
not a quick add, because today it would read zero by construction: random-traffic goals sit
`3 × radius` beyond entry ("beyond the disc; never reached" — the goal is a direction, not a
destination), and a 1500 m ring's goals are a 291 s flight against `t_max = 250` before any
resolution detour. Before the number can exist:

- [ ] Scenarios whose goals are reachable inside the encounter (ring radius / `t_max` chosen
      together; a random-traffic variant with real destinations), with `stop_within` sized to
      the airframe (a fixed-wing orbits at its loiter radius — `autopilot/waypoint.py`).
- [ ] The estimand, decided before the code: per-run "all arrived" vs per-aircraft arrival
      fraction, and unconditional (MC) vs conditional-on-LoS (the only thing the IPS tail can
      measure). Denominators fixed by design, per `MonteCarloEstimate`'s own rule.
- [ ] The plumbing: arrival read off the terminal state into `FleetOutcome` / the results
      schema, and `_tail_slice`'s return widened past `(K, A)` if the conditional version is
      wanted.
