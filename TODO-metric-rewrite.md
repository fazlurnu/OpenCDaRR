# TODO — metric rewrite: per-run → per-aircraft (Blom)

Status: **code done, re-runs pending.** Backward compatibility is not a concern (single user).

- [x] `fleet.py` — per-pair loss flags on the state, `K` / `A` on the outcome
- [x] `estimator.py` — `p_ac`, `mean_los_pairs`, per-encounter records, pooling
- [x] `ips.py` — optional tail leg, `p_ac` / `expected_los_pairs` / `n_lineages`
- [ ] re-runs (the table below)
- [ ] docs on the site

## Why

The estimator currently reports a **per-run** loss probability. Blom & Bakker (2015),
*Safety Evaluation of Advanced Self-Separation Under Very High En Route Traffic Demand*,
JAIS, DOI 10.2514/1.I010243, normalise **per aircraft** (Figs. 2 and 4), and for dense random
traffic **per aircraft per flight hour** (Fig. 9). Adopting their normalisation makes our
multi-aircraft numbers directly comparable with the reference literature, and removes a metric
that saturates at 1 in dense traffic.

## Definitions

For run `r` with `N` aircraft, true separation `d_ij(t)`:

    d_ij_min(r) = min_t d_ij(t)                     # per pair
    d_i_min(r)  = min_{j != i} d_ij_min(r)          # per aircraft
    s(r)        = min_{i<j} d_ij_min(r) = min_i d_i_min(r)   # == FleetState.min_sep today

    K(r) = #{pairs (i,j) : d_ij_min(r) < rpz}       # losing pairs
    A(r) = #{aircraft i  : d_i_min(r)  < rpz}       # aircraft involved

Metrics:

    P_run = (1/n) Σ_r 1{K(r) >= 1}                  # what we report today
    E[K]  = (1/n) Σ_r K(r)                          # frequency, unbounded above by 1
    P_ac  = (1/(nN)) Σ_r A(r)                       # Blom's normalisation  <-- new headline
    λ     = Σ_r A(r) / Σ_r (N * T_r) = P_ac / T     # per aircraft per flight hour (random traffic)

`T_r` is the **measured** flight time (inside `MeasurementArea`), not the whole run.

Exact bounds, no assumptions:

    (2/N) * P_run  <=  P_ac  <=  P_run
    P_run <= E[K]

At `N = 2` all three collapse to one number, so **every pairwise result is unchanged**.
In the rare limit `P_run ≈ (N/2) * P_ac`; at `N = 8` our current number is ~4x Blom's.

## Code changes (done — recorded here for the ADR that should follow)

Verified after the change: the published pairwise run (`configs/pairwise.yaml`, seed 0, 500
encounters, `dpsi=2`) still gives `p_los = 0.050`, `ci95 = (0.034, 0.073)`,
`median_min_sep = 53.3 m`, and now `p_ac == p_los` exactly. IPS `prob` is bit-identical with and
without the tail leg. Full suite passes; ruff clean on the three changed files.

Invariants spot-checked on fleets (rpz 50 m, seed 7):

| fleet | resolver | K | A | min_sep |
|---|---|---|---|---|
| `swap_ring(4)` | none | 6 (all pairs) | 4 (all aircraft) | 0.0 |
| `swap_ring(4)` | MVP | 0 | 0 | 102.7 |
| `converging_ring(6)` | none | 15 (all pairs) | 6 | 0.0 |
| `converging_ring(6)` | MVP | 10 | 6 | 41.2 |

The last row is the argument for the change in one line: `los` is `True` either way, so `P_run`
cannot tell the resolver apart from no resolver, while `K` drops 15 → 10.

### What was changed

1. `opencdarr/fleet.py`
   - `FleetState` (~line 179): carry per-aircraft loss flags (a bool tuple of length N, or a
     frozenset of losing pairs) beside `los` / `min_sep`. Must live on the state so IPS clones it.
   - `_advance` (~line 518): where `los = state.los or cur < self.rpz` is computed, also mark the
     pairs/aircraft that crossed. Respect the `MeasurementArea` mask — same gate as `min_sep`.
   - `FleetOutcome` (~line 151): expose `n_los_pairs` (K) and `n_los_aircraft` (A) alongside `los`.
   - Cost note: the per-step pairwise loop already computes every pair, so this is bookkeeping,
     not extra geometry.

2. `opencdarr/estimator.py`
   - `IPRResult` (~line 108): record `sum_k`, `sum_a`, and `n_aircraft`; derive `p_ac`, `mean_k`
     next to `p_los`. Keep `p_los` — it is still the right quantity for the pairwise pages.
   - `ci95`: Wilson on `p_ac` uses `n * N` trials, not `n`. The per-aircraft indicators within a
     run are **not** independent, so a plain Wilson interval understates the width. Either use a
     run-level bootstrap or state the caveat where it is reported.
   - `combine_ipr` (~line 180): sum the new counters too.

3. `opencdarr/ips.py`
   - `_evolve_to_shell` (~line 101) stops the particle at the **first** crossing, so K and A are
     not observable from the surviving cloud.
   - Add a continuation leg after the last shell: evolve each survivor to `is_terminal` on fresh
     streams, then apply
         E[K]  = P_run * mean(K | survivors)
         P_ac  = P_run * mean(A | survivors) / N
   - Precision of the conditional means is set by the number of **distinct lineages**, not by
     `n_particles` — resampling fills the cloud with clones. Report the effective sample size.

## Re-runs needed (N > 2 only)

| artifact | traffic | note |
|---|---|---|
| `examples/handbook/ring_mc_vs_ips.ipynb` | `random_traffic(n)`, sweeps N | most expensive (IPS reps × N) |
| `scripts/mc_vs_ips_campaign.py` | `random_traffic` | second most expensive |
| `examples/handbook/circle_scenario.ipynb` | `circle_fleet(N)` | |
| `examples/handbook/traffic_density.ipynb` | density sweep | |
| `scripts/ipr_fleet_sweep.py` | `swap_ring(n)` | its `swap_ring(2)` baseline is unaffected |
| `scripts/ipr_fleet_comm_sweep.py` | `swap_ring(n)` | |
| `scripts/random_spawn_conflict.py` | `random_traffic` | |
| `scripts/fleet_scenarios_demo.py` | `swap_ring(8)`, `converging_ring(8)` | 2 of 4 panels only |

Highest value: `converging_ring(8)` — a superconflict, so `P_run` pins near 1 while `E[K]` and
`P_ac` still resolve. That is the case that demonstrates why the metric changed.

**No re-run:** every pairwise notebook and every `scripts/handbook/*` figure — `a_first_run`,
`monte_carlo`, `rare_event_ips`, `resolver_comparison`, `communication`, `navigation`,
`mixed_fleet`, `the_whole_chain`, `byo_*`, `tutorial_*`. Numbers are identical, not merely close.

## Docs follow-up (opencdarr.github.io)

- `docs/estimators/monte-carlo.md` — the "What the numerator counts" subsection states the
  definitions; make `P_ac` the headline and keep `P_run` as the pairwise-equivalent.
- `docs/estimators/index.md` — the MC vs IPS table describes what each estimator gives.
- `docs/scenario/{ring,random-traffic}.md` — still placeholders, so write them against the new
  metric directly.

## Open questions

- Interval for `p_ac`: bootstrap over runs, or Wilson with a stated caveat about within-run
  dependence?
- Report `λ` (per flight hour) for random traffic only, or everywhere `MeasurementArea` is set?
- Does `A(r)` count an aircraft that loses separation with two others once or twice? (Once —
  it is a per-aircraft indicator. `K` is where multiplicity lives.)
