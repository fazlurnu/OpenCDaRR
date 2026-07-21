# Architecture & data flow — a complete simulation setup

How every module in `opencdarr/` connects in one full run: `config + seed → IPR`. The
spine is the design decision that *you own the state and the loop; BlueSky is a library of
stateless math* (`docs/design_brief.md`). Everything here is pure values threaded as arguments —
no globals, so any state is clonable for the interacting particle system later.

Read this top-to-bottom: **backbone flow** (the whole run), then **one tick** (the heart of the
loop), then the **pluggable interfaces** (the contribution surface), then a **module-by-module
I/O reference** so nothing is left implicit.

> Legend: rounded/box nodes are functions or classes; edge labels are the value passed. `(ABC)`
> = abstract base class (a model family); `(Protocol)` = a small compositional callable. ABCs are
> passed *into* `run_encounter` / `estimate_ipr`; swapping one is how a contributor adds an
> algorithm without forking the core.

---

## 1. Backbone — the complete run (`config + seed → IPR`)

```mermaid
flowchart TB
    subgraph entry["config.py — configuration"]
        YAML["config.yaml + seed"]
        CFG["Config<br/>(Scenario / Conflict /<br/>Methods / Simulation)"]
        YAML -->|load_config| CFG
    end

    subgraph exp["experiment.py — entry point + provenance"]
        RUN1["run_one_experiment(config)"]
        COMPS["_make_perf / _make_detector /<br/>_make_resolver / _make_recovery"]
        CARD["provenance card (.md)"]
        RES["ExperimentResult"]
    end

    CFG --> RUN1 --> COMPS

    subgraph rng["rng.py — reproducible streams (ADR 0001)"]
        ROOT["root_seed_sequence(seed)"]
        SP["spawn(parent, n)"]
        GEN["generator(seq)"]
        ROOT --> SP --> GEN
    end

    subgraph est["estimator.py — plain Monte Carlo"]
        EST["estimate_ipr(config, perf, detector,<br/>resolver, recovery, nav, comm, surv)"]
        IPR["IPRResult<br/>(ipr, n_conflict, n_los)"]
    end

    COMPS --> EST
    EST -->|"per encounter: spawn(3)"| SEQS["geom_seq · nav_seq · comm_seq"]
    EST -. builds .-> ROOT

    subgraph scen["scenario.py — encounter geometry"]
        SAMP["sample_pairwise(rng, speed,<br/>dcpa_max, tlos, rpz, ci95)"]
        CREATE["create_conflict(own, dpsi,<br/>dcpa, tlos, rpz)"]
        SAMP --> CREATE
    end

    SEQS -->|geom_seq| SAMP
    SAMP -->|"(own, intr)"| PAIR["AircraftState x2"]

    subgraph loopy["loop.py — the environment"]
        LOOP["run_encounter(own, intr, perf, dynamics,<br/>detector, resolver, recovery,<br/>navigation, communication, surveillance, ...)"]
        OUT["EncounterOutcome<br/>(conflict, los, min_sep)"]
        LOOP --> OUT
    end

    PAIR --> LOOP
    SEQS -->|nav_seq, comm_seq| LOOP
    COMPS --> LOOP
    OUT -->|"aggregate: 1 - n_los/n_conflict"| EST
    EST --> IPR --> RES
    RUN1 --> CARD

    classDef entryc fill:#e3f2fd,stroke:#1565c0,color:#0d47a1
    classDef flowc fill:#f3e5f5,stroke:#7b1fa2,color:#4a148c
    class entry,exp entryc
    class est,scen,loopy,rng flowc
```

The **estimator never sees the number of aircraft or the algorithms** — it sees `sample →
run_encounter → outcome`, then counts. This is what lets IPS (roadmap v0.4) replace plain Monte
Carlo without touching the loop (ADR 0004). The alternative CLI entry point,
`scripts/ipr_angle_sweep.py`, skips `estimate_ipr` and calls `run_encounter` directly per fixed
crossing angle (joblib-parallel), reusing the same substreams across resolvers for a fair
comparison.

---

## 2. One tick inside `run_encounter` — the heart of the loop

Two cadences: aircraft **decide** every `broadcast_interval` (the ADS-L/ASAS rate) on their
*perceived* view, then that `Command` is **held** while dynamics **integrate** every `dt`. Truth
is used only to score the encounter.

```mermaid
flowchart TB
    subgraph truth["True states — owned by the loop, never global"]
        OWN["own : AircraftState"]
        INTR["intr : AircraftState"]
    end

    subgraph bcast["Decision (every broadcast_interval)"]
        MEAS["navigation.measure(true, t, rng)<br/>-> Message (noisy self-fix)"]
        COMM["communication.step(comm_state,<br/>broadcasts, receivers, t, comm_rng)<br/>-> CommState"]
        SURV["surveillance.perceived(comm_state,<br/>receiver, source, t)<br/>-> AircraftState or None"]
        DECIDE["_decide(self, perceived_other,<br/>nominal, memory)"]
        DET["detector.detect(own, intr,<br/>rpz, t_lookahead) -> bool"]
        REC["recovery.should_resume(own,<br/>intr, rpz) -> bool"]
        RESO["resolver.resolve(own, intr, rpz)<br/>-> Command"]
        CMD["Command(hdg, spd)<br/>+ PairMemory"]
    end

    OWN --> MEAS
    INTR --> MEAS
    MEAS -->|broadcasts| COMM --> SURV
    SURV -->|perceived other| DECIDE
    DECIDE --> DET --> CMD
    DECIDE --> REC --> CMD
    DECIDE --> RESO --> CMD

    subgraph integ["Integration (every dt, command held)"]
        DYN["dynamics.step(state, command, perf, dt)<br/>-> AircraftState"]
        SCORE["geo.qdrdist -> separation<br/>detector.detect -> conflict?<br/>relative_enu -> past-CPA / done"]
    end

    CMD -->|held between ticks| DYN
    DYN --> OWN
    DYN --> INTR
    OWN --> SCORE
    INTR --> SCORE
    SCORE --> OUTCOME["EncounterOutcome<br/>(conflict, los, min_sep)"]

    classDef truthc fill:#fff3e0,stroke:#e65100,color:#bf360c
    classDef decc fill:#e8f5e9,stroke:#2e7d32,color:#1b5e20
    classDef intc fill:#e1f5fe,stroke:#0277bd,color:#01579b
    class truth truthc
    class bcast decc
    class integ intc
```

Key points the diagram encodes:

- **The CNS chain is `measure → communicate → perceive`.** Without a `communication` model, the
  perceived other *is* the broadcast directly (instant, perfect delivery). With one, a decision
  reads only what the link actually delivered — or `None` before first contact, which flies that
  pair nominal (ADR 0006 §5).
- **Directed everywhere.** Each arrow runs twice per tick — A→B and B→A are independent draws.
- **`dynamics.step` is the one swap point** for physics (ADR 0007): Dubins-car point mass today,
  a wind-aware or holonomic model later, without editing the loop.

---

## 3. The pluggable interfaces — the contribution surface

Every model family is an `ABC` threaded into the loop as a parameter; a new algorithm is a new
file implementing the interface, not a fork (`docs/design_brief.md`). `Protocol`s are the smaller
callables fed *into* those models.

```mermaid
flowchart LR
    subgraph CDf["cd/ — detection"]
        CDABC["ConflictDetector (ABC)<br/>detect(own, intr, rpz, tla) -> bool"]
        SB["StateBased"]
        ISLOS["is_los(own, intr, rpz) -> bool"]
        CDABC --> SB
    end
    subgraph CRf["cr/ — resolution"]
        CRABC["ConflictResolver (ABC)<br/>resolve(own, intr, rpz) -> Command"]
        CRABC --> MVPn["MVP(margin)"]
        CRABC --> VOn["VO(margin)"]
    end
    subgraph CRRf["crr/ — recovery"]
        CRRABC["RecoveryCriterion (ABC)<br/>should_resume(own, intr, rpz) -> bool"]
        CRRABC --> PC["PastCPA(bouncing_guard)"]
        CRRABC --> FTRn["FTR"]
        CRRABC --> PFTR["ProbabilisticFTR(...)"]
    end
    subgraph DYNf["dynamics.py — physics (ADR 0007)"]
        DABC["Dynamics (ABC)<br/>step(state, command, perf, dt) -> AircraftState"]
        DABC --> PMD["PointMassDynamics<br/>wraps step_dynamics()"]
        DABC -. future .-> WIND["wind / holonomic /<br/>other airframe"]
    end
    subgraph CNSf["cns/ — communication-navigation-surveillance"]
        NAVABC["NavigationModel (ABC)<br/>measure(true, t, rng) -> Message"]
        NAVABC --> GPS["GpsNavigation(distribution)"]
        NDP["NoiseDistribution (Protocol)<br/>(rng, ci95, trk) -> (e, n)"]
        NDP --> GAUSS["gaussian"]
        GPS -. uses .-> NDP
        COMMABC["CommunicationModel (ABC)<br/>step(state, bcasts, rcvrs, t, rng) -> CommState"]
        COMMABC --> COMMc["Comm(reception_prob, latency)"]
        LDP["LatencyDistribution (Protocol)<br/>(rng) -> delay"]
        LDP --> LATc["constant / uniform / lognormal"]
        COMMc -. uses .-> LDP
        SURVABC["SurveillanceModel (ABC)<br/>perceived(state, rcv, src, t) -> AircraftState or None"]
        SURVABC --> LK["LastKnown (hold-as-is)"]
    end

    classDef abc fill:#ede7f6,stroke:#4527a0,color:#311b92
    classDef impl fill:#f1f8e9,stroke:#558b2f,color:#33691e
    class CDABC,CRABC,CRRABC,DABC,NAVABC,COMMABC,SURVABC abc
    class SB,MVPn,VOn,PC,FTRn,PFTR,PMD,GPS,GAUSS,COMMc,LATc,LK impl
```

---

## 4. Foundational values — what everything reads & writes

All are frozen dataclasses (clonable, no aliasing). `AircraftState` is the certain kinematic
core; `Command` is the one control message every resolver emits and every dynamics consumes.

```mermaid
flowchart LR
    AS["AircraftState<br/>id, lat, lon, trk, gs,<br/>turn_rate, desired,<br/>pos_ci95, vel_ci95"]
    DV["DesiredVelocity<br/>v_east, v_north<br/>(.trk / .gs derived)"]
    CMD["Command<br/>v_east, v_north<br/>(.trk / .gs derived)"]
    MSG["Message<br/>source, state, t_meas"]
    CS["CommState<br/>held, in_flight"]
    IF["InFlight<br/>message, receiver, deliver_t"]
    PM["PairMemory<br/>resolving, onset_velocity"]
    REL["Relative<br/>rx, ry, vx, vy, dist"]
    PERF["Performance<br/>max_tr, max_dtr2,<br/>v_max, v_min, ax"]

    AS -->|desired| DV
    AS -->|measure| MSG
    MSG -->|held / in_flight| CS
    CS --> IF
    IF --> MSG
    AS -->|relative_enu| REL
    PERF -. envelope for .-> AS

    subgraph geo_km["Pure math (BlueSky-free)"]
        GEO["geo.py<br/>forward, qdrdist, earth_radius"]
        KM["kinematics.py<br/>relative_enu, velocity_enu"]
    end
    KM --> REL
    GEO -. used by .-> KM

    classDef val fill:#fce4ec,stroke:#ad1457,color:#880e4f
    classDef math fill:#e0f2f1,stroke:#00695c,color:#004d40
    class AS,DV,CMD,MSG,CS,IF,PM,REL,PERF val
    class geo_km math
```

---

## 5. Module-by-module I/O reference

Every `.py` in `opencdarr/`, its public surface, and what flows in/out.

### Orchestration

| Module | Symbol | Input | Output |
|---|---|---|---|
| `experiment.py` | `run_one_experiment(config, card_dir)` | `Config` | `ExperimentResult(ipr, card_path)` — and writes a provenance card |
| `estimator.py` | `estimate_ipr(config, perf, detector, resolver, recovery, nav?, comm?, surv?)` | config + built components | `IPRResult(ipr, n_conflict, n_los)` |
| `loop.py` | `run_encounter(own, intr, *, perf, dynamics, rpz, t_lookahead, dt, detector, resolver?, recovery?, navigation?, rng?, communication?, surveillance?, comm_rng?, t_max, done_timeout, broadcast_interval, share_intent)` | two `AircraftState` + all models | `EncounterOutcome(conflict, los, min_sep)` |
| `loop.py` | `_decide(ac, other, nominal, memory, rpz, tla, detector, resolver, recovery)` | one directed view | `(Command, PairMemory)` |
| `config.py` | `load_config(path)` | YAML path | validated `Config` |
| `rng.py` | `root_seed_sequence(seed)` / `spawn(parent, n)` / `generator(seq)` | int seed / seq | `SeedSequence` / list / `np.random.Generator` |

### Scenario & state

| Module | Symbol | Input | Output |
|---|---|---|---|
| `scenario.py` | `sample_pairwise(rng, speed, dcpa_max, tlos, rpz, ci95…)` | RNG + distribution params | `(own, intr): AircraftState` |
| `scenario.py` | `create_conflict(own, dpsi, dcpa, tlos, rpz, …)` | ownship + geometry | intruder `AircraftState` |
| `state.py` | `AircraftState` / `DesiredVelocity` | — | frozen kinematic value |
| `state.py` | `create_aircraft(perf, …)` | `Performance` + fields | envelope-validated `AircraftState` |
| `performance.py` | `Performance`, `M600` | — | frozen envelope limits |

### Dynamics (ADR 0007)

| Module | Symbol | Input | Output |
|---|---|---|---|
| `dynamics.py` | `Command` | — | control target: velocity vector `(v_east, v_north)`, ADR 0008 |
| `dynamics.py` | `Dynamics` (ABC) `.step(state, command, perf, dt)` | one aircraft + command | next `AircraftState` |
| `dynamics.py` | `PointMassDynamics` | — | default impl (wraps `step_dynamics`) |
| `dynamics.py` | `step_dynamics(state, command, perf, dt)` | one aircraft + command | next `AircraftState` |

### CD / CR / CRR

| Module | Symbol | Input | Output |
|---|---|---|---|
| `cd/base.py` | `ConflictDetector` (ABC) `.detect(own, intr, rpz, tla)` | directed pair | `bool` (conflict predicted) |
| `cd/base.py` | `is_los(own, intr, rpz)` | directed pair | `bool` (in loss of separation now) |
| `cd/statebased.py` | `StateBased` | — | CPA detector impl |
| `cr/base.py` | `ConflictResolver` (ABC) `.resolve(own, intr, rpz)` | directed pair | `Command` |
| `cr/mvp.py` / `cr/vo.py` | `MVP(margin)` / `VO(margin)` | — | resolver impls |
| `crr/base.py` | `RecoveryCriterion` (ABC) `.should_resume(own, intr, rpz)` | directed pair | `bool` (resume nominal?) |
| `crr/pastcpa.py` … | `PastCPA` / `FTR` / `ProbabilisticFTR` | — | recovery impls |

### CNS

| Module | Symbol | Input | Output |
|---|---|---|---|
| `cns/base.py` | `NavigationModel` (ABC) `.measure(true, t, rng)` | true state + RNG | `Message` (noisy self-fix) |
| `cns/navigation.py` | `GpsNavigation(distribution)` | — | nav impl (uses `geo`, `kinematics`, noise) |
| `cns/base.py` | `NoiseDistribution` (Protocol) `(rng, ci95, trk)` | — | `(east, north)` error |
| `cns/noise_distributions.py` | `gaussian`, `CI95_TO_SIGMA` | — | isotropic position noise |
| `cns/base.py` | `CommunicationModel` (ABC) `.step(state, bcasts, rcvrs, t, rng)` | comm state + broadcasts | new `CommState` |
| `cns/communication.py` | `Comm(reception_prob, latency)` | — | reception+latency impl |
| `cns/base.py` | `LatencyDistribution` (Protocol) `(rng)` | — | `delay` [s] |
| `cns/communication.py` | `constant_/uniform_/lognormal_latency` | params | a `LatencyDistribution` |
| `cns/base.py` | `SurveillanceModel` (ABC) `.perceived(state, rcv, src, t)` | comm state + link | `AircraftState` or `None` |
| `cns/surveillance.py` | `LastKnown`, `age(...)` | — | hold-as-is belief / staleness |
| `cns/base.py` | `Message`, `CommState`, `InFlight` | — | frozen comm values |

### Pure math

| Module | Symbol | Input | Output |
|---|---|---|---|
| `geo.py` | `forward(lat, lon, bearing, dist)` | point + vector | new `(lat, lon)` |
| `geo.py` | `qdrdist(lat1, lon1, lat2, lon2)` | two points | `(bearing, distance)` |
| `kinematics.py` | `relative_enu(own, intr)` | two states | `Relative(rx, ry, vx, vy, dist)` |
| `kinematics.py` | `velocity_enu(state)` | one state | `(v_east, v_north)` |

---

## Related

- [[decisions/0001-rng-per-particle-spawn]] — the substream tree wired in §1.
- [[decisions/0004-layered-directed-design-for-multiaircraft-and-ips]] — why the estimator is
  oblivious to N, and why every model is directed/pairwise-primitive.
- [[decisions/0006-communication-model-design]] — the `measure → communicate → perceive` chain in §2.
- [[decisions/0007-dynamics-as-pluggable-interface]] — the one swap point for physics in §2/§3.
- [[decisions/0008-velocity-vector-command]] — why `Command`/`DesiredVelocity` in §4 are velocity
  vectors, not polar.
- Governing equations per algorithm live under `vault/derivations/`.
