# Future feature — aircraft joining the fleet mid-run

**Target version: v0.5+** (`docs/roadmap.md`) — tentative. Prototyped end to end against the
current `fleet.py` and then removed; this file is what was learned, so the next attempt starts
from the design rather than from the discovery.

## What

Every run today has a fixed roster: `run_fleet` takes a list of `Agent`s, and `n` is set once in
`FleetEnv.initial_state` and never changes. A **traffic-density** study wants the other shape — a
stream of aircraft entering an airspace over a fixed horizon, so the question becomes "how does
the LoS rate scale with arrival rate", not "what happened in this one encounter".

The shape that fits: a frozen list of `Arrival(t, agent)`, written **before** the run and never
changed while it runs, admitted at the first step at or after each time.

## Why deferred

Not blocked on the runner — the `fleet.py` half is small and worked. Deferred because the two
layers above it need decisions that are not made yet:

1. **The metric changes meaning.** `IPRResult.p_los` is `n_los / n_encounters`, a *per-encounter*
   probability. Under a spawn stream one "encounter" becomes one traffic realization of duration
   `t_max` containing an unknown number of conflicts, so `p_los` silently becomes *P(at least one
   LoS somewhere in T seconds)* — which grows with both `t_max` and density, for two different
   reasons that cannot be separated afterwards. A density sweep wants a **rate** (LoS per flight
   hour, or per conflict-pair encountered), which means `run_fleet` counting LoS *events* and pair
   exposure rather than latching booleans, plus a result type beside `IPRResult`. That is an
   ADR-shaped decision and it should be made before any code lands.
2. **There is no scenario seam.** Both backends are hardwired to `sample_pairwise` — `_run_mc` via
   `estimate_ipr`, and `_run_ips`'s `build_initial` inline. Arrivals cannot reach `run_experiment`
   without one, and neither can the N-aircraft scenarios that *already exist* (`swap_ring`,
   `converging_ring`). Open that seam first: it unblocks both, and it is useful on its own.

## The design, as prototyped

- **`Arrival(t, agent)`**, pre-materialized. A sampler called inside `advance` would put a mutable
  generator in `FleetEnv` — the immutable half shared across IPS particles — and two clones of one
  survivor would then draw *different* rosters from it (ADR 0001). Draw the traffic up front.
- **`build_env` covers the whole roster.** The per-aircraft tuples (`kinematics`, `perfs`,
  `adapters`, `aps`) are built over `agents + [a.agent for a in arrivals]`, so a joiner's config
  already sits at its own index. `FleetState` holds only the airborne prefix, plus an `n_admitted`
  cursor; arrivals only ever *append*, so index `i` in the state always means `env.aps[i]`.
- **Admit at the top of `advance`, before `rel_pre`.** A roster that grows partway through a step
  leaves the pre- and post-step pair lists different lengths, and the `i < j` pair sequence for
  `n + 1` is not an extension of the one for `n`, so `_segment_min_sep` could not match them up
  even in principle. Admitting first means both ends see one roster and the newcomer flies the
  whole `dt`.
- **A joiner enters as it would have at `t = 0`** — nominal command, empty memories, `last_tx =
  None` so nobody perceives it before its first transmit — and starts its broadcast clock at *its
  own* arrival time plus its schedule phase, rather than inheriting a cadence running since `t=0`.
- **`is_terminal` must veto while arrivals are pending.** `t_max` first, then the veto, then the
  clear/goal tests. Both of those ask whether the aircraft airborne *now* are finished, and in a
  quiet gap before the next arrival they are — without the veto the run ends before most of its
  traffic ever flies. Measured on a 4-aircraft ring: 205 s with the veto, ~160 s without.

## Two defects it surfaced, both live today

- **Duplicate aircraft ids are not caught.** `build_env` builds `ids = frozenset(...)`, which
  silently swallows a duplicate. The CNS layers key per aircraft (`CommState.held` on
  `(receiver, source)`, `NavState` on the id), so two aircraft sharing an id cross-feed each
  other's beliefs rather than failing. A generated roster hits this easily; a hand-written one
  rarely does. Worth fixing on its own — it is one `len(set(ids)) != n` check.
- **`viz.extract_tracks` would silently omit late arrivals.** It reads `n` and `ids` from
  `frames[0]` and indexes `f.states[k]`. Because arrivals append, those indices stay valid, so it
  does not raise — it just drops the joiners from the plotted tracks while still counting them in
  the separation curve. Silent, not loud, which is the worse failure for a figure.

## Relations

- `opencdarr/fleet.py` — `FleetEnv` / `FleetState` / `build_env`, where this lands.
- `opencdarr/experiment.py` — the `Methods` bundle and `_COMPONENTS`, where a `TrafficModel` seam
  would go so density is sweepable and cache-keyed like any other component.
- `opencdarr/estimator.py` — `estimate_ipr`, hardwired to two aircraft.
- `vault/decisions/0001-rng-per-particle-spawn.md` — why the arrival list is frozen, not sampled.
- `vault/decisions/0004-layered-directed-design-for-multiaircraft-and-ips.md` — the env/state/
  streams split the design has to respect.
