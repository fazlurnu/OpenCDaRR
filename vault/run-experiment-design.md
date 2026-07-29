# Design proposal — `run_experiment`: a distribution-driven experiment wrapper

> **Status: proposed, not built.** This is a design sketch for review, not an implementation
> plan. It fixes the shape and the decisions and stops. Names of types (`run_experiment`,
> `Sweep`, `IPS`, `ExperimentResult`, …) are proposals. Reviewers: please push on the *seams*
> and the *constraints*, not the bikeshed names.

## 1. Goal

One entry point that turns a **declared distribution over the encounter model** into
**P(LoS) (and other metrics), with a plot and a provenance trail**:

```python
res = run_experiment(independent_vars=…, methods=…, backend=…, metrics=…, seed=0)
res.frame()      # tidy table, one row per condition
res.plot("ipr")  # the figure, laid out from the declared axes
```

It is the user-facing face of the *scenario / encounter-distribution* layer the design brief
already names (reviewer item #10: "operationally realistic distributions … not uniform
geometry") and of `TODO` #3 ("make the user-facing interface very minimal"). It **generalises the
existing `run_one_experiment`** (one config → one IPR + card): that becomes the all-`Fixed`,
single-cell special case of `run_experiment`.

Nothing here is a new estimator. The wrapper *composes* what exists:

- geometry model — `scenario.create_conflict(dpsi, dcpa, tlos, rpz, gs_intr, side, …)`;
- the two "sample one encounter from a seed" seams — `scenario.sample_pairwise` (MC) and the
  `build_initial: SeedSequence → Particle` closure (IPS);
- the two estimators — `estimator.estimate_ipr` (plain MC) and `ips.estimate_rare_prob`
  (rare-event IPS);
- the pluggable model interfaces — `Dynamics`, `ConflictResolver`, `NoiseDistribution` /
  `NavigationModel`, `ConflictDetector`, `RecoveryCriterion`.

## 2. The core framing: independent vars → dependent vars

The wrapper is deliberately modelled on the papers' own Design-of-Experiments vocabulary
(`05ExperimentSetup.tex`: "Independent variables", "Dependent variables", Experiments 1–3):

- **Independent variables** = the encounter/method parameters and *what role each plays*.
- **Dependent variables** = the metrics computed from the runs (IPR, median dCPA, …).
- **Backend** = how the probability is estimated (plain MC vs rare-event IPS).

Proposed signature:

```python
def run_experiment(
    independent_vars: dict[str, Fixed | Sweep | Random],
    *,
    methods: MethodSet,                      # detector / resolver / recovery instances
    perf: Performance,
    dynamics: Dynamics = Multirotor(),
    navigation: NavigationModel | None = None,
    communication: CommunicationModel | None = None,
    surveillance: SurveillanceModel | None = None,
    backend: MC | IPS,
    metrics: Sequence[Metric] = (IPR, MedianCPA),
    seed: int = 0,
    cache: bool | CacheConfig = False,
    card_dir: Path | None = Path("vault/experiments"),
) -> ExperimentResult: ...
```

## 3. Independent variables

### 3.1 Three roles unify "list" and "distribution"

Every encounter parameter is declared as exactly one of:

| role | you give it | meaning |
|---|---|---|
| `Fixed(v)` | a constant | held at `v` |
| `Sweep([...])` | a list / grid | an **output axis** → P(LoS) *as a function of* it (a conditional response curve) |
| `Random(dist)` | a distribution | **integrated out** → contributes to a single marginal |

A crossing-angle *list* and a crossing-angle *distribution* are not two code paths — they are the
same axis in two roles. `Sweep` yields the conditional surface `θ ↦ P(LoS|θ)`; `Random` yields
the marginal `E_{θ~p}[P(LoS|θ)]`. These are different mathematical objects, and both are useful;
a typical study sweeps some axes and marginalises others.

```python
independent_vars = dict(
    dpsi     = Sweep(range(2, 359, 4)),   # crossing angle → an output axis
    pos_ci95 = Sweep([0.0, 5.0, 10.0]),   # uncertainty level → another output axis
    dcpa     = Fixed(0.0),
    tlos     = Fixed(180.0),
    rpz      = Fixed(50.0),
)
# same spec, angle re-roled → the marginal (paper's Experiment 3):
independent_vars["dpsi"] = Random(uniform_angle(0, 360))
```

### 3.2 How the marginal is computed (a decision to pin)

Two routes to a `Random` axis's marginal:

- **(a) direct sampling** — draw `θ_i ~ p(θ)`, one encounter each, aggregate. This is exactly
  today's MC with `p(θ)` swapped for the built-in uniform. Trivial, unbiased, but wasteful and
  **starves in the rare regime** (the reason IPS exists).
- **(b) quadrature over the conditional curve** — estimate `f(θ)=P(LoS|θ)` on a grid once, then
  `marginal = Σ_k w_k f(θ_k)` with weights from `p(θ)`. Reusable across input distributions,
  yields a per-angle **risk decomposition**, and pairs naturally with IPS per grid point.

**Recommendation:** quadrature is the default; direct sampling is the cheap cross-check (they must
agree in the feasible regime — the analytical ⊂ MC ⊂ IPS ladder). Reviewer: is this the right
default, and should it be user-selectable per axis?

### 3.3 The full catalogue

Legend: **✓** natural · **~** possible but special (fleet/categorical mixture) · **✗** N/A.
`Fixed` is always available; the informative columns are **Sweep** and **Dist**.

| family | variable (code name) | kind | Sweep | Dist |
|---|---|---|---|---|
| geometry | `dpsi` (crossing angle) | continuous, **circular** | ✓ | ✓ (von Mises / wrapped / truncated to `[5,355]`) |
| geometry | `dcpa` (nominal miss) | continuous ≥0 | ✓ | ✓ |
| geometry | `tlos` | continuous >0 | ✓ | ✓ |
| geometry | `side` | ±1 | ✓ | ✓ (Bernoulli) |
| geometry | `speed` (ownship gs) | continuous >0 | ✓ | ✓ |
| geometry | `gs_intr` (intruder speed / ratio) | continuous >0 | ✓ | ✓ |
| uncertainty | `pos_ci95` | continuous ≥0 | ✓ | ✓ (per-aircraft → 2 axes) |
| uncertainty | `vel_ci95` | continuous ≥0 | ✓ | ✓ (per-aircraft → 2 axes) |
| noise shape | `pos_distribution` (6 models) | categorical | ✓ | ~ |
| noise shape | `vel_distribution` | categorical | ✓ | ~ |
| noise shape | model hyper-params (`k, w, ρ, ℓ`) | continuous | ✓ | ✓ |
| CDR structure | `detector` | categorical | ✓ | ~ |
| CDR structure | `resolver` | categorical | ✓ | ~ |
| CDR structure | `recovery` | categorical | ✓ | ~ |
| CDR params | `margin` (MVP) | continuous ≥1 | ✓ | ✓ |
| CDR params | `gamma` / `prob_threshold` | continuous (0,1) | ✓ | ✓ |
| CDR params | `ktheta` | integer (solver knob) | ~ (convergence only) | ✗ |
| CDR params | `bouncing_guard` | boolean | ✓ | ✗ |
| airframe | `perf` / `aircraft_type` | categorical bundle | ✓ | ~ |
| airframe | individual limits (`yaw_rate_max`, `ax`, `v_max`) | continuous | ✓ | ✓ |
| detection | `rpz` | continuous >0 | ✓ | ~ |
| detection | `t_lookahead` | continuous >0 | ✓ | ~ |
| comms | `broadcast_interval` | continuous >0 | ✓ | ~ |
| comms | reception probability | continuous [0,1] | ✓ | ✓ |
| comms | comm latency | continuous ≥0 | ✓ | ✓ |
| numerics | `dt`, `t_max`, `done_timeout` | continuous | ~ (convergence only) | ✗ |

Notes the reviewer should check:

- **Only continuous scalars support all three roles.** Categorical/structural axes are
  Fixed-or-Sweep (a `Random` over them means a *fleet mixture*, rarely wanted). Solver/numeric
  knobs (`ktheta`, `dt`) should be Fixed; "Sweep" only for a convergence study; never `Random`.
- **Circular support.** `Random(dpsi)` must be a circular/bounded distribution respecting the
  `_DPSI_MIN=5°` exclusion and the 180° head-on symmetry — a raw Gaussian is ill-posed at the
  wrap.
- **Per-aircraft splitting silently doubles axes.** `pos_ci95`, `vel_ci95`, `speed`, `perf`, noise
  model can each be own-vs-intruder. Default them **shared** (homogeneous — both papers assume
  this); split only on demand.

### 3.4 Taming the curse of dimensionality

The space is ~25 axes; that is fine because a well-formed experiment activates **≤3**. The wrapper
enforces this by construction: **every axis defaults to `Fixed`**, and you opt each one into
`Sweep`/`Random` explicitly. The result's plot layout is then fully determined by which few you
opted in (1 `Sweep` → a curve; 2 → curves-with-hue / heatmap; a `Random` axis → it collapses out).
The papers are exactly this: Exp 1 = `Sweep(dpsi) × Sweep(4 uncertainty) × Sweep(3 recovery)`;
Exp 2 = `Sweep(dpsi) × Sweep(5 gamma)`; Exp 3 = `Random(dpsi) × Sweep(6 noise) × Sweep(3 recovery)`.

## 4. Backends

Each backend is a typed object carrying **exactly** its estimator's parameters, so an illegal
combination (`MC` with `n_particles`) is unrepresentable:

```python
backend = MC(n_encounters=10_000)                        # → estimator.estimate_ipr
backend = IPS(n_particles=2000, reps=10, levels="auto")  # → ips.estimate_rare_prob
```

- **`seed` stays at the top level** — it is the reproducibility root (`config + seed → result`),
  common to both, and roots the `SeedSequence` tree either backend spawns from.
- **The CI story is why they differ.** MC encounters are independent → CI is a closed-form
  Wilson/binomial interval on pooled counts; one count `N` suffices. IPS particles *within* a run
  interact (shared ancestors) → no valid within-run CI; the CI comes from `reps` independent
  replications (log-space). So `reps` is structural for IPS and meaningless for MC.
- **`levels`** (the decreasing shell ladder ending at `rpz`) is an IPS-only parameter; `"auto"`
  would pick shells adaptively per cell. Reviewer: per-cell adaptive levels vs one shared ladder
  is an open question — different cells (angles/uncertainty) have very different rare-event depths.

### 4.1 IPS restricts the dependent-variable set — important

**When `backend=IPS`, the only computable dependent variable is the rare-event probability:
`P(LoS)` (equivalently `IPR = 1 − P(LoS)`). Median dCPA, CPA distributions, and any
trajectory-derived metric are *not* available from IPS.**

Why: IPS concentrates effort by cloning/resampling particles toward the rare set, so the surviving
ensemble is a **biased, importance-weighted** population, not an unbiased sample of encounter
outcomes. A "median dCPA" taken over IPS particles would be biased toward the rare set and is
meaningless. Those metrics need an unbiased population — which only plain MC provides. The metric
layer must therefore know each metric's **backend compatibility** and refuse (fail fast, not
silently mislead) an IPS + median-dCPA request.

## 5. Dependent variables (metrics)

Metrics are **first-class, pluggable dependent variables**, not hard-coded. A metric reduces the
per-condition runs to a value (+ CI). Default set: `IPR` / `PLoS`, `MedianCPA`. The `TODO` #2 /
phase-9 wishlist (total Δv, extra flight time, extra distance, path deviation, time resolving) are
each a new metric object, added without touching the loop.

Proposed metric interface (reviewer: this is the load-bearing decision — see §8):

```python
class Metric(Protocol):
    name: str
    backends: frozenset[str]                     # {"mc"} | {"mc","ips"} — enforced compatibility
    def reduce(self, records: Sequence[Record], backend: MC | IPS) -> MetricValue: ...
    # MetricValue = value + CI (+ optional distribution payload for violin/histogram metrics)
```

Backend-compatibility matrix (encodes §4.1):

| metric | MC | IPS |
|---|---|---|
| `IPR` / `PLoS` (inter-convertible) | ✓ (Wilson CI) | ✓ (log-space CI) |
| `MedianCPA`, `CPADistribution` | ✓ | ✗ (biased ensemble) |
| Δv, path deviation, time resolving | ✓ | ✗ (needs trajectory + unbiased population) |

The metric also determines the CI treatment (Wilson from counts on MC; log-space across
replications on IPS), so the caller never specifies CI handling twice — the metric reads
`res.backend`.

## 6. User contributions — the contribution surface

Each thing a user overrides is an existing ABC/Protocol; the user passes an **instance**, and the
same instance runs unchanged under both backends. Four override points:

**Dynamics** (`opencdarr/dynamics/base.py`) — how the aircraft moves:
```python
class InstantTurnRotor(Dynamics):
    def step(self, state: AircraftState, command: MotionCommand, perf: Performance,
             dt: float, wind: WindField = NO_WIND) -> AircraftState:
        ve, vn = command.target_velocity
        gs = _clip(math.hypot(ve, vn), abs(perf.v_min), perf.v_max)
        trk = math.degrees(math.atan2(ve, vn)) % 360.0
        lat, lon = geo.forward(state.lat, state.lon, trk, gs * dt)
        return dataclasses.replace(state, lat=lat, lon=lon, trk=trk, gs=gs,
                                   **odometry_update(state, gs, dt))   # mandatory (ADR 0010)
```

**Navigation position-error distribution** (`opencdarr/cns/base.py`, `NoiseDistribution`) — a
callable `(rng, ci95) → (east, north)`, factory form matching `make_mixture_gaussian`:
```python
def make_student_t(df: float = 3.0):
    SCALE = 0.4085  # calibrate so 95% radial ≈ ci95 (bisection, like the built-ins)
    def student_t_pos(rng, ci95):
        s = ci95 * SCALE
        return float(rng.standard_t(df) * s), float(rng.standard_t(df) * s)
    return student_t_pos

navigation = GnssNavigation(pos_distribution=make_student_t(df=3))
```

**Resolution velocity** (`opencdarr/cr/base.py`, `ConflictResolver`) — returns a `MotionCommand`
with `target_velocity` set:
```python
class BrakeResolver(ConflictResolver):
    def __init__(self, brake=0.5): self.brake = brake
    def resolve(self, own, intruders, rpz, preferred=None) -> MotionCommand:
        vox, voy = velocity_enu(own)
        return MotionCommand(target_velocity=(vox * self.brake, voy * self.brake))
```

**Metric** (§5) — a new dependent variable.

Passing them in:
```python
res = run_experiment(
    independent_vars = {...},
    methods    = MethodSet(detector=StateBased(), resolver=BrakeResolver(0.6), recovery=PastCPA()),
    dynamics   = InstantTurnRotor(),
    perf       = Performance(v_max=18.0, v_min=-18.0, ax=5.0),
    navigation = navigation,
    backend    = MC(n_encounters=10_000),
    metrics    = [IPR, MedianCPA],
    seed       = 0,
)
```

Two constraints the reviewer should weigh:

- **`NoiseDistribution` cannot see heading or ground speed** — its signature is strictly
  `(rng, ci95) → (east, north)`. So a *heading-dependent* position model (along/cross-track
  anisotropy, a latency bias `−ℓ·g`) **cannot be expressed** without widening the Protocol to
  `(rng, ci95, heading, gs)`. This is the same gap that blocks 3 of the JRESS Experiment-3 noise
  models. Widen it, or accept that track-oriented models are out of scope for v1?
- **Any override can also be a `Sweep` axis** — e.g. `resolver = Sweep([BrakeResolver(0.6),
  MVP(1.05), VO(1.05)])` or `dynamics = Sweep([InstantTurnRotor(), Multirotor()])` fan out as a
  categorical axis (a hue/facet), so a contributor benchmarks their model against the built-ins in
  one call.

## 7. Results & post-processing

`run_experiment` returns one `ExperimentResult` that tabulates, plots, drills down, and records
provenance.

```python
ExperimentResult(
    backend, independent_vars, metrics=['ipr', 'median_cpa'],
    conditions=270, card_path='vault/experiments/…md',
)
```

**`res.frame()` — the tidy table** (one row per swept cell; swept axes as columns, then each
metric with CI, then the counts):
```
    dpsi  pos_ci95    ipr  ipr_lo  ipr_hi  median_cpa  n_los  n_conf
0      2       0.0  0.998   0.995   0.999        61.2     18   10000
2      2      10.0  0.947   0.943   0.951        96.1    530   10000
```

**`res.plot(metric)` — auto-laid-out from the axis roles.** First `Sweep` axis → x, remaining
`Sweep` axes → hue/facets, CI as a shaded band. So `res.plot("ipr")` reproduces
`fig_crossing_angle_vs_ipr`; `res.plot("median_cpa")` adds the dashed `rpz` reference line. Styled
to house conventions (no grid, no figure title, concise legend; detail belongs in the caption).

**`res.cell(**levels)` — drill into one condition**, including the **retained raw outcomes** so a
*new* metric recomputes without re-running:
```python
cell = res.cell(dpsi=90, pos_ci95=10.0)
cell.ipr        # 0.9993
cell.ipr_ci     # (0.9989, 0.9996)
cell.outcomes   # (EncounterOutcome × 10_000)  ← the material for post-hoc metrics
```

**The output shape adapts** to two things without the caller restating them:

- *Backend*: MC → `ipr` + Wilson CI + counts; IPS → `plos` + log-space CI + `n_collapsed`,
  and `res.plot("plos")` uses a log y-axis.
- *A `Random` axis*: that axis's column disappears; each remaining cell is one aggregate, and a
  distributional metric (`CPADistribution`) carries the violin/histogram payload.

**`res.card_path` — provenance.** One Markdown card per experiment (generalising what
`run_one_experiment` writes): the full spec (axis roles, methods, dynamics/nav identities, backend,
seed, **code/git hash**), the embedded frame, and per-cell cache hit/miss (§8). A result is
reproducible from its card alone.

## 8. Caching

Experiments are expensive (IPS especially) and are re-run constantly — to extend a sweep, to add a
metric, to resume after a crash. Caching is a first-class option, `cache=True` or
`cache=CacheConfig(dir=…)`, off by default.

**Granularity: per-condition, storing the raw runs — not the reduced metrics.**
- Cache one entry **per swept cell** (per condition). This lets a partial sweep resume, and lets
  *extending* a sweep (adding angles/levels, or another `Sweep` axis) reuse every already-computed
  cell.
- Store the **raw per-encounter outcomes** (MC) or **per-replication `IPSResult`s** (IPS), *not*
  the metric values. Metrics are cheap functions of these, so **adding a new metric recomputes from
  cache with zero new simulation** — the point of retaining outcomes in §7.

**Key = content hash of everything that determines the numbers:**
```
key(cell) = hash(
    fixed_values, swept_level_values_of_this_cell, methods_identity,
    dynamics_identity, navigation/comms_identity, backend + its params,
    seed, CODE_HASH,
)
```

**The critical correctness rule: the key MUST include a code/git hash.** A cache that ignores code
identity will silently serve numbers from *old* dynamics/resolver/loop code after an edit — which
violates the reproducibility contract ("never trust a 1e-9 number that isn't anchored",
design brief). On any key mismatch the wrapper **recomputes**; it never serves a stale entry. This
is also the forcing function to finally implement the `code_hash: (deferred)` placeholder already
sitting on the experiment card.

**Storage & lifecycle:**
- On-disk under a cache dir (e.g. `.opencdarr_cache/`), one file per cell (parquet/npz), so cells
  are written/read independently and a crashed run loses at most the in-flight cell.
- The provenance card records, per cell, the cache key and whether it was a hit or a recompute, so
  "was this figure recomputed or reused?" is answerable after the fact.

Open cache questions for the reviewer: (a) how is `dynamics_identity` / `methods_identity` hashed
for *user* objects — by source hash, by a declared `id`, or by config? (b) should the cache be
seed-*inclusive* (exact reuse) or seed-*family* aware (reuse a prefix of the `SeedSequence` tree
when only `n_encounters`/`reps` grew)? The RNG design (`children`/`spawn`) supports the latter and
it would make "run more samples" incremental rather than a full recompute.

## 9. Decisions to pin (for the reviewing agent)

1. **Metric interface — terminal vs trajectory.** `MedianCPA` needs only the terminal
   `EncounterOutcome` (has `min_sep`); Δv / path-deviation / time-resolving need per-step data the
   loop does not currently retain (`AircraftState` accumulates only `flight_time`,
   `distance_flown`). Does `Record` = terminal outcome (cheap, covers the paper metrics) or a
   trajectory log (enables phase-9 metrics, costs memory + a loop change)? This gates §5 and the
   cache payload size in §8.
2. **Marginal computation** — quadrature-default vs direct-sampling (§3.2), and per-axis
   selectable?
3. **`NoiseDistribution` widening** — add `heading`/`gs` to the signature (unlocks track-oriented
   models, JRESS Exp 3) or keep it minimal?
4. **One environment object?** Should `methods` + `dynamics` + `navigation`/`comms` collapse into a
   single `Environment` value (matching the brief's `advance/level/is_terminal` framing) rather
   than loose keyword args?
5. **IPS per-cell levels** — adaptive per condition vs one shared ladder (§4).
6. **The IPS metric restriction's ergonomics** (§4.1) — fail fast on `IPS + MedianCPA`, or silently
   drop incompatible metrics with a warning?

## 10. Non-goals / relationship to existing code

- Generalises `run_one_experiment`; does **not** replace the estimators or the model interfaces —
  it rides them. A new algorithm is still "add a file, not a fork."
- Not a GUI (that is the v1.2 line in `TODO`). Not multi-aircraft yet (pairwise first; the
  `FleetEnv` interface already generalises, so the wrapper should not hard-code `n=2`).
- Reproducibility is the contract: every result is seed-deterministic, code-hash-stamped, and
  carries a provenance card — the cache is built *around* that contract, not beside it.
