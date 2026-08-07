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
- [ ] **Early stop:** K and A can only change while some pair can still breach. Candidate
      absorbing test: all pairs diverging and separated (the `_all_clear` predicate without the
      done-timeout wait). **Check before trusting it:** under noisy CDR a resolver reaction can
      re-converge a diverging pair, so measure how often K/A would still have grown after the
      first all-diverging instant. If the test holds, a never-clearing ring stops flying the
      ~100 s of encounter it currently burns per continuation (breach at t ≈ 148, `t_max` 250).

## 3. Geodesy / per-step cost — ideas parked, not committed

The deep cut, kept as a list until measured designs exist. Profile (ring n = 8): `geo.qdrdist`
2.43 M calls, ~39 % of cumulative time; the `relative_enu` pipeline ~57 %; ~101 `qdrdist` per
`advance`, i.e. ~3.6 per pair per step, roughly half of them recomputing geometry already
computed that step. `dataclasses.replace` churn is another ~15 %.

- [ ] Reuse the post-step `pairwise_relative` as the next step's `rel_pre` — it is recomputed
      identically today. Needs the geometry carried alongside the state without breaking the
      immutable-particle contract (ADR 0004); pure caching, no number may change.
- [ ] Let `_all_clear` read that same post-step geometry instead of its own `relative_enu` pass
      (the third recomputation per step).
- [ ] The detection sweep runs **ordered** pairs; `detect(i, j) == detect(j, i)` by CPA symmetry,
      so half those calls are free to drop.
- [ ] Flat-ENU fast path for the O(n²) inner loop: one tangent-plane projection per aircraft per
      step, pair math in metres — needs the error argument at the ~1 km scale these scenarios
      span (it will be far below the 40 m noise floor, but write it down).
- [ ] Vectorise the pairwise geometry: one numpy matrix operation over all pairs instead of the
      Python loop — condensed upper-triangle form (pdist-shaped), since the matrix is symmetric.
      This is a rewrite of the hottest path; profile before/after on the same ring probe.
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
