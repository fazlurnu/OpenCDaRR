# Phase 6 plan — Multi-aircraft (n > 2): the fleet environment, coordination, and scenarios

Generalises the whole stack from **pairwise (n = 2)** to **N aircraft in simultaneous conflict** —
v0.3 of the roadmap, the multi-aircraft commitment [[0004-layered-directed-design-for-multiaircraft-and-ips]]
made the model shaped for. Everything through Phase 5 is pairwise: `run_encounter(own, intr)`, and
`MVP.resolve(own, intr, rpz)` / `should_resume(own, intr, rpz)` take a *single* intruder. Phase 6
turns the directed pairwise **primitive** into an N-aircraft **environment** without rewriting the
core: detection iterates a conflict graph, resolution sums over an aircraft's conflicts, recovery
waits until clear of *all* of them, and a **coordination model** (a genuine research choice) decides
how the fleet's independent resolutions compose.

This is **not** rare-event / IPS (that is Phase 8) and **not** the consolidated metrics/logging
(Phase 9). Phase 6 is the fleet *environment* and the scenarios that exercise it.

Same working style as Phases 2–5: one file at a time, read each diff, tick the box; each rung green
before the next; the load-bearing gate is that **at N = 2 the fleet reduces to the pairwise result** —
a free regression against every Phase-4/5 anchor. One deliberate exception: **VO re-anchors** in 6a
(its preferred velocity changes from current to nominal, decision 4), so the pairwise VO anchors move
to new pinned values; MVP stays bit-for-bit.

> Correction to ADR 0004: it says "the old code already sums the pairwise `dv`s" — that referred to
> the BlueSky port. The current clean `MVP` is single-intruder, so the set-resolution is real new
> work in 6a.

---

## Settled up front (decided with the user, 2026-07-24)

1. **Coordination: cooperative-symmetric first, priority/give-way as a follow-on.** The first model
   (6c) is **cooperative-symmetric** — every aircraft independently resolves against *all* its
   conflicts by summing the pairwise MVP/VO avoidance vectors, no negotiation. It is the direct N = 2
   generalisation of today's behaviour (and the ASAS/BlueSky default), so N = 2 reduces exactly. A
   **priority / give-way** model is a *second* model behind the same interface (6f), its own ADR, not
   a rewrite. The perfectly-symmetric superconflict (scenario 3) is the stress test that motivates it.
2. **Deliverable includes a quantitative multi-aircraft IPR.** Beyond the qualitative scenario demos
   (6d), a **seeded multi-aircraft IPR sweep** (6e): LoS across *any* pair, over noise realisations —
   the N-aircraft analogue of the pairwise sweeps, with a reproducible seed.
3. **Perfect perception first.** Each aircraft sees the others' broadcasts directly (instant, perfect
   delivery — the Phase-3a behaviour at N), with optional **GNSS self-noise** (Phase 3a) so the IPR
   sweep has a noise source. The **lossy communication / surveillance** model (Phase 3b) generalised
   over all n(n−1) directed links is **deferred** — a clean later rung, not part of 6b.
4. **Set-resolution is algorithm-specific — MVP sums, VO takes the union (the load-bearing fix).**
   The `resolve` interface takes the intruder **set**; how the set composes is each algorithm's own
   business, *not* a generic "sum the `dv`s":
   - **MVP** (potential field) sums the pairwise avoidance vectors: `v_own − Σ dv_i`. Reference stays
     the **current** velocity, so MVP's `n = 1` and `n = 2` are **bit-for-bit** with today.
   - **VO** (velocity obstacles) is a *feasibility* problem: each intruder forbids a **cone** of
     velocities, and the resolution is the velocity **outside the union of all cones** closest to the
     preferred velocity — *not* a sum (a summed velocity can re-enter a cone). Computed by **analytic
     candidate search** (decided with the user): candidates = per-cone edge projections of the
     preferred velocity **+ cross-cone edge intersections** (the union's boundary vertices), filtered
     to those outside *all* cones (with `margin`), nearest to preferred wins; cone edges treated as
     **rays** (fixing the current finite-segment simplification); an **over-constrained fallback**
     (least-penetration) when no reachable exterior velocity exists.
   - VO's **preferred velocity is the nominal/desired** (decided with the user), not the current
     velocity — so VO biases back toward the intended track. This **changes VO's single-intruder
     result**, so the **pairwise VO anchors re-anchor** (new pinned values, as the fixed-wing IPR did
     in 4e). MVP is unaffected. The preferred (nominal) velocity is threaded into `resolve` (the
     `SeparationManager` already holds the nominal command).

---

## The generalisation (how the pairwise primitive becomes a fleet)

The directed, pairwise-primitive design (ADR 0004) means the *primitives* barely change; what grows
is the *environment* that iterates them and the *memory* that rides along:

| Layer | Pairwise now | N-aircraft (Phase 6) |
|---|---|---|
| **Detect** | `detect(own, intr, …) -> bool`, directed | unchanged — iterate over the **conflict graph** (each aircraft vs each other it perceives) |
| **Resolve** | `resolve(own, intr, rpz)` | `resolve(own, intruders, rpz)` — the interface takes the **set**; **each algorithm composes its own way** (MVP *sums* the pairwise `dv`s; VO finds the velocity outside the **union** of the cones). Not a universal "sum." |
| **Recover** | `should_resume(own, intr, rpz)` | `should_resume(own, intruders, rpz)` — resume only when clear of **all** conflicts |
| **Separation memory** | one `PairMemory` per directed pair | a per-aircraft **`FleetMemory`**: `resopairs` = a set/map of active directed pairs + each one's onset velocity |
| **Environment** | `run_encounter(own, intr)` | `run_fleet([aircraft…])` — N states, each its own autopilot/dynamics/perf, all advance simultaneously |
| **Metric** | pair separation | **min pairwise separation across all pairs**; LoS = any pair < rpz; conflict = any pair predicted |
| **Coordination** | implicit (both cooperate) | an explicit **model** — cooperative-symmetric (6c), priority (6f) |

The no-hidden-state invariant becomes **more** load-bearing (ADR 0004): with more aircraft there is
more per-aircraft recovery memory, and a clone that lost any of it would diverge (KI-1 at scale). So
`FleetMemory` is clonable value state, threaded in/out, never on an object — exactly `PairMemory`'s
discipline, one level up.

---

## Phasing (each rung green before the next)

### 6a — CDR primitives generalise to a set (settled decision 4)

- [ ] **`opencdarr/cr/base.py`** — `resolve(own, intruders: Sequence[AircraftState], rpz, preferred=None)`;
  the interface takes the **set**, and an optional **preferred** ground velocity (the nominal, for VO;
  defaults to `own`'s current velocity). Empty set ⇒ hold. Docstring is explicit that composition is
  algorithm-specific (MVP sums, VO unions — decision 4).
- [ ] **`opencdarr/cr/mvp.py`** — sum the pairwise `dv`s over the conflicting set; reference is the
  current velocity. `n = 1` and `n = 2` **bit-for-bit** with today.
  - *Check:* `resolve(own, [intr], rpz) ==` the old `resolve(own, intr, rpz)`; a symmetric
    two-intruder case sums to the expected combined vector.
- [ ] **`opencdarr/cr/vo.py`** — the **union-of-VOs** analytic candidate search (decision 4): build
  each cone, generate edge-projection + cross-cone-intersection candidates, filter to those outside
  all cones, pick nearest to **preferred** (the nominal); least-penetration fallback. `n = 1` reduces
  to the current single-cone geometry *except* the preferred velocity is now the nominal ⇒ **VO
  re-anchors**.
  - *Check:* a two-intruder union case picks a velocity provably outside **both** cones (a summed
    resolution would not); the re-anchored single-intruder VO is pinned to its new value.
- [ ] **`opencdarr/crr/base.py` + `pastcpa.py` + `ftr.py` + `probabilistic_ftr.py`** —
  `should_resume(own, intruders, rpz)` = clear of **all** (∀ intruders past-CPA / free-to-revert).
  `len == 1` reproduces today.
- [ ] **`opencdarr/separation.py`** — `SeparationManager.step` consumes the **whole**
  `perceived_traffic` list (it already accepts it, ADR 0011 §6): detect each → the conflicting set,
  resolve against that set (passing the nominal as `preferred`), recover when clear of all;
  `FleetMemory` (per-aircraft resopairs map) replaces the single `PairMemory`. The setpoint adapter
  (Phase 4e) and per-airframe projection are unchanged.
  - *Check:* the pairwise `_decide` shim + `run_encounter` reproduce every Phase-4/5 **MVP** anchor
    bit-for-bit; the **VO** anchors move to their re-anchored values (the one intended change).

### 6b — The N-aircraft environment (`run_fleet`)

- [ ] **A per-aircraft bundle** — introduce a small frozen `Agent` (or `Aircraft`) value grouping
  `(state, autopilot, dynamics, perf)` + its threaded `FleetMemory` / `GuidanceMemory`. ADR 0011 §7
  deferred a `Vehicle` class "until a real grouping need appears" — N parallel lists *is* that need,
  so the bundle lands here (its own short note in the coordination ADR or a small 6b ADR).
- [ ] **`opencdarr/loop.py` (or `fleet.py`)** — `run_fleet(agents, *, rpz, t_lookahead, dt, detector,
  resolver, recovery, wind=NO_WIND, navigation=None, rng=None, …)`: on the broadcast cadence each
  aircraft takes its (optionally noisy) self-fix, **perceives all others** (perfect delivery this
  pass), decides against its perceived traffic (detect graph → resolve set → recover), and all N
  advance simultaneously by their own dynamics. Outcome: per-pair `min_sep`, the fleet `min_sep`, LoS
  (any pair), conflict (any pair).
  - *Check (the 6b gate):* `run_fleet` with **2** agents reproduces `run_encounter` **bit-for-bit**
    (noiseless and seeded-noisy), across the Phase-4/5 airframe and wind cases.

### 6c — Coordination model: cooperative-symmetric (ADR)

- [ ] **`opencdarr/coordination/` (new family) + ADR 001x** — the coordination model as a pluggable
  contribution surface (mirrors `cd/`/`cr/`/`crr/`). `CooperativeSymmetric`: every aircraft resolves
  against all its conflicts independently, MVP/VO-summed — the fleet behaviour is the emergent
  composition, no negotiation. This is what 6b calls; making it explicit names the seam a priority
  model (6f) slots into. The ADR records *why* coordination is a first-class model (ADR 0004's
  forward-linked "coordination-model ADR").
  - *Check:* cooperative-symmetric at N = 2 is the pairwise cooperative behaviour (the 6b gate covers
    it); a 3-aircraft symmetric case resolves as the sum-of-pairs predicts.

### 6d — Scenario builders + the scenarios (qualitative, min-pairwise-sep)

- [ ] **`opencdarr/scenario.py`** — N-aircraft builders beside `create_conflict`: `swap_pair`
  (scenario 1), `swap_ring` (scenario 2), `converging_ring` (scenario 3), `near_parallel`
  (scenario 4). Each returns the fleet of `AircraftState` + each aircraft's `Mission`
  (goto its target), placed far enough apart that after avoidance the waypoint is still **ahead**
  (resume, don't double back).
- [ ] **`scripts/` demos + `vault/observations/`** — run each through `run_fleet`, plot the ground
  tracks + the min-pairwise-separation over time (rpz line), one observation per scenario (or one
  combined), in the [[mixed-fleet-dubins-holonomic]] contrast lineage. The **converging-ring
  superconflict** is the headline stress test: does cooperative-symmetric MVP resolve it, or does the
  symmetry deadlock/oscillate? Either way is a finding.

### 6e — Multi-aircraft IPR sweep (the quantitative payoff)

- [ ] **`scripts/ipr_fleet_sweep.py` + observation** — sweep fleet size / geometry (and, later, wind),
  N seeded GNSS-noisy realisations per point; IPR = 1 − (runs with any-pair LoS)/N. The N-aircraft
  analogue of `ipr_angle_sweep` / `ipr_wind_sweep`, reproducible from seed. Reports how the resolved
  IPR degrades with fleet density — the "does DAA hold as traffic thickens?" question.
  - *Check:* at N = 2 the fleet IPR equals the pairwise IPR for the same geometry/seed.

### 6f — Priority / give-way coordination (second model)

- [ ] **`opencdarr/coordination/priority.py` + ADR** — a second coordination model (rules-of-the-air
  give-way: lower-priority aircraft maneuvers, higher holds), behind the 6c interface. Motivated by
  the superconflict (6d) if cooperative-symmetric struggles. Own ADR; **may slip** past Phase 6 if the
  cooperative baseline is enough for the scenarios.

### 6g — Communication & surveillance uncertainty at N (asymmetric perception)

Generalises the Phase-3b comm/surveillance model over the fleet, so perception becomes **asymmetric**:
aircraft A may have heard from C while B has not, each acting on a *different, stale* picture of the
same sky. This is where the multi-aircraft DAA gets genuinely hard — the symmetric-perception
assumption 6b–6f rely on breaks, and the `FleetMemory` / coordination model must stay correct when no
two aircraft agree on the traffic.

- [ ] **`opencdarr/cns/` at N** — each aircraft broadcasts once per tick; the
  :class:`CommunicationModel` steps over all **n(n−1) directed links** (per-link reception + latency,
  each drawing from its own substream, ADR 0006 §6); each decision reads
  :class:`SurveillanceModel`'s `perceived(...)` for that directed link — the last message *that* link
  delivered, or `None` before first contact (fly nominal). An aircraft's own self-fix never goes
  through comm.
- [ ] **`run_fleet`** threads `communication` / `surveillance` / `comm_rng` (as `run_encounter`
  already does pairwise), building each aircraft's perceived-traffic list per-link.
  - *Check:* at N = 2 with a comm model, `run_fleet` reproduces `run_encounter`'s lossy-comm result
    bit-for-bit; a 3+-aircraft case where a link is down shows an aircraft flying on a stale/absent
    perception of one neighbour while resolving another (asymmetric perception exercised, not bypassed).
- [ ] **Observation** — how asymmetric perception degrades the fleet IPR vs perfect perception (6e),
  reusing the sweep with a lossy link.

---

## Tests (the gate for each rung)

- [ ] `test_cr.py` (extend) — **MVP** `resolve(own, [intr], rpz)` bit-for-bit vs the old
  single-intruder call; a two-intruder sum equals the expected combined vector.
- [ ] `test_vo.py` (extend) — **VO** re-anchored single-intruder value pinned; a two-intruder case
  returns a velocity provably outside **both** cones (the union property a sum would violate); the
  over-constrained fallback returns least-penetration.
- [ ] `test_crr.py` / `test_ftr.py` (extend) — `should_resume` over a set = clear-of-all; `len == 1`
  unchanged.
- [ ] `test_separation.py` (extend) — the manager over a multi-intruder list; `FleetMemory` threaded,
  not hidden; pairwise reduction.
- [ ] `test_loop.py` (extend) — **the 6a/6b regression:** every Phase-4/5 anchor reproduced through
  the generalised primitives and `run_fleet(2)`.
- [ ] `test_fleet.py` (new) — `run_fleet` at N = 2 == `run_encounter`; a 3–8 aircraft scenario
  resolves (fleet min-sep ≥ rpz where the coordination model is expected to clear); reproducible IPR
  from seed.

---

## Scenarios (the concrete cases — please confirm the geometry)

1. **Two aircraft swap.** A and B each fly to the *other's* start; placed far apart so the crossing
   resolves with the waypoint still ahead (resume, no double-back). Pairwise mission + DAA + resume.
2. **Eight aircraft swap.** 8 uniformly on a ring, each flies to the **diametrically-opposite**
   aircraft's start — all converge through the centre region toward *distinct* destinations (four
   opposing pairs superimposed). Far enough apart that each reaches its waypoint after avoidance.
3. **Eight-aircraft converging ring.** 8 uniformly on a circle, all → the **same** waypoint (the
   centre) — the symmetric superconflict; the cooperative-symmetric stress test.
4. **Near-parallel pair.** Two aircraft whose initial points and waypoints give a **5° crossing
   angle** at the encounter — the near-parallel / slow-closing hard case.

---

## Out of scope, on purpose (with the escape hatch)

- **Rare-event / IPS estimation and the `level` function** — Phase 8. Phase 6 stays plain-MC over the
  fleet (ADR 0004: the estimator is a separate layer that only sees `advance/level/is_terminal`; N
  appears only inside `level`, added in Phase 8).
- **Consolidated logging / P(dcpa < X) metrics** — Phase 9.
- **The priority coordination model (6f)** and **comm/surveillance uncertainty (6g)** — in Phase 6's
  arc but toward the end; each lands behind an existing interface, not a core change, and either may
  slip if the earlier rungs fill the phase.
- **Sector / constant-density flow scenario** — dropped for now (was scenario 5); a flow model
  (density-driven spawning) rather than a fixed fleet, to be reconsidered later with its own detail.

## Relations

- Realises the multi-aircraft half of [[0004-layered-directed-design-for-multiaircraft-and-ips]] (the
  IPS half is Phase 8); makes the `perceived_traffic` list of
  [[0011-motioncommand-and-guidance-separation]] §6 load-bearing and reuses its per-aircraft threading
  (§7 / [[0015-velocity-to-fixedwing-projection]]).
- Consumes the airframes (0012/0013), guidance (0014), and wind (0016) unchanged — the fleet is
  airframe- and wind-agnostic by construction.
- Forward-links: the **coordination-model ADR** (6c) and the **priority model** (6f); Phase 8 (IPS),
  Phase 9 (metrics).
