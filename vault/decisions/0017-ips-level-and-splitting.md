# ADR 0017 — IPS: running-min level function + fixed-effort multilevel splitting

- Status: accepted
- Date: 2026-07-26
- Deciders: Fazlur Rahman

## Context

v0.4's headline is a **rare-event collision-risk estimate reported with a confidence interval**
(`design_brief.md` #2, roadmap v0.4, how-to Step 6). Plain Monte Carlo cannot reach it: the
[[rare-event-validation-ladder]] shows that at `pos_ci95 = 10 m` the loss-of-separation rate is
`P(LoS) ≈ 4.7×10⁻⁴`, and a *single* 10 000-encounter MC run there returned **zero events** — MC in
the rare regime is not just imprecise, it can read the wrong answer. The remedy the project committed
to is the **Blom–Bakker interacting particle system** (IPS) — importance splitting with resampling
([[0004-layered-directed-design-for-multiaircraft-and-ips]], which forward-links *this* ADR as "the
IPS level/importance function").

Phase 8 Step 5 already built what IPS needs: the environment is split into `FleetEnv` (fixed rules,
shared across particles), `FleetState` (the particle — the whole world as a **deeply immutable**
value, so a clone is reference-sharing), and `FleetStreams` (per-particle RNG, re-spawned on clone),
exposed as `advance / level / is_terminal` (`opencdarr/fleet.py`). This ADR fixes the IPS specifics
that ride that interface: the importance function, the splitting scheme, the terminal condition, the
estimate + CI, and the validation gate.

## Decision

### 1. Split on the **running-minimum** separation, not the instantaneous one

The rare event is `min_sep < d_m`. For the first cut `d_m = rpz` (loss of separation), so IPS
estimates the *same* `P(LoS)` plain MC does — later ADRs push `d_m` down to a physical collision
radius. Nested shells `B_0 ⊃ B_1 ⊃ … ⊃ B_m`, with `B_k = { φ ≤ d_k }` for a **fixed**, decreasing
sequence `d_0 > d_1 > … > d_m = rpz` (geometric spacing).

The importance function is the **running minimum pairwise separation reached so far** —
`FleetState.min_sep`, which the particle already accumulates — *not* the instantaneous
`level(state)`. True separation dips to CPA and rises again, so instantaneous min-sep is
**non-monotone**: a particle would cross a shell going in and again coming out, ambiguous for
splitting. The running minimum is **monotone non-increasing by construction**, so every shell
crossing is strictly one-way and unambiguous. (Note: `level(state)` stays the instantaneous read for
diagnostics; the *splitting* reads `state.min_sep`.)

### 2. Fixed-effort resampling (the Blom–Bakker IPS), no weights

Keep a **constant N particles at every level**. For level `k`:

1. **Evolve** each particle with `env.advance` until it either crosses the next shell
   (`state.min_sep ≤ d_{k+1}`) → *survivor*, frozen at its crossing state, or hits `is_terminal`
   without crossing → *dropped*.
2. **Estimate** the conditional `p̂_k = S_k / N`, where `S_k` = number of survivors.
3. **Resample** with replacement from the `S_k` survivors back up to N. Each clone is a shared
   `FleetState` reference **+ a freshly spawned `FleetStreams`** (`SeedSequence.spawn`, ADR 0001) —
   the Step-5 clone, so cloned futures are independent by construction.

The estimate is the product `P̂ = Π_k p̂_k = Π_k (S_k / N)`. Because the population is pinned at N and
resampling equalises particles each level, there are **no per-particle weights** — the estimator is
just a product of survival ratios. If `S_k = 0` at any level the run collapses to `P̂ = 0`; shell
spacing must therefore keep the per-level survival fraction healthy (~10–50%), which is a tuning
obligation (and the motivation for the AMS upgrade below).

### 3. Terminal condition is **unchanged** — no absorbing past-CPA kill

`env.is_terminal` stays exactly as Step 5 left it: cleared for `done_timeout`, or `t_max`. IPS adds
**no** rare-event absorbing kill. A dropped particle is simply one whose running-min never reached
the next shell before `is_terminal` fired; resampling discards it automatically. Deliberately *not*
killing a particle at CPA means that if a recovery manoeuvre re-approaches, the particle stays alive
and its running-min can still cross a shell — removing a real bias an aggressive kill would introduce
(discarding a particle that would have re-entered). The cost is that a non-survivor runs the extra
~`done_timeout` instead of stopping at CPA; this is negligible, because an IPS particle only sims
short shell-to-shell segments and the whole run is far cheaper than the equivalent-precision MC in
the rare regime. Success is detected as the final shell (`d_m = rpz`) being crossed — no separate
success terminal.

### 4. The initial cloud samples geometry; splitting acts on the forward CNS noise

The N initial particles are drawn from the encounter distribution (`sample_pairwise`), the *same*
distribution plain MC integrates over. Splitting then concentrates effort on the forward
(nav/comm-noise) evolution of those particles. So IPS integrates over both the sampled geometry and
the trajectory noise, estimating the identical `P(LoS)` — which is what makes the MC comparison in §6
apples-to-apples.

### 5. Estimate + CI from **independent replications**

Within one IPS run the particles interact (they share ancestors through resampling), so within-run
particle spread is **not** a valid CI. Report the interval from **R independent replications**: run
the whole IPS `R` times on independent seed subtrees, get `P̂_1 … P̂_R`, and report their mean with a
CI from the across-replication sample variance (in **log space**, since the product estimator is
right-skewed). This is the honest, delta-method-free answer to `phase-8-plan`'s "what is the variance
of the estimate?". `N` (particles/level), the shell sequence `d_k`, and `R` are config-driven tuning
knobs, not architecture.

### 6. Validation gate — two rungs, both required ([[rare-event-validation-ladder]])

- **Correctness — `pos_ci95 = 40` (`P ≈ 0.028`):** the IPS mean±CI must agree with a plain-MC
  estimate (a few thousand encounters give MC a tight CI here). This proves the machinery is
  unbiased.
- **Efficiency — `pos_ci95 = 10` (`P ≈ 4.7×10⁻⁴`):** IPS must return a tight CI at a fixed compute
  budget where MC's CI explodes (and a single 10⁴ run reads 0). This proves it earns its keep.

Only trust IPS in a rarer regime once the pos=40 rung is green. Passing pos=40 alone is *not* victory
— at `P ≈ 0.03` splitting barely helps, so it tests correctness but not the efficiency claim.

## Alternatives considered / deferred

- **Adaptive multilevel splitting (AMS)** — choose each shell as a quantile of the current cloud's
  running-min (e.g. drop the worst fraction each iteration) instead of a fixed `d_k` sequence. This
  removes the shell-spacing guesswork *and* the `S_k = 0` collapse, and is the natural robustness
  **upgrade once fixed-shell IPS passes the gate**. Deferred: more machinery, and a fixed ladder is
  easier to validate against MC first. Its own future ADR — [[ips-adaptive-levels]].
- **Fixed-splitting / RESTART** — spawn a fixed number of offspring per crossing; the population
  fluctuates and every particle carries a weight. **Rejected:** weight bookkeeping and a muddier CI;
  fixed-effort keeps the population at N and needs no weights.
- **Importance sampling / subset simulation on the noise** — a plausible alternative for a
  noise-driven rare event. Not pursued: IPS (the Blom–Bakker lineage this project cites) handles the
  path-dependent CDR dynamics naturally and is the committed method (ADR 0004, roadmap v0.4).
- **tCPA-informed importance** (projected miss distance rather than current running-min) — a smarter
  φ that could reduce variance, but is itself a research contribution; deferred behind the simple
  running-min baseline.
- **Instantaneous `min_sep` as the importance** — rejected for the non-monotonicity in §1; the
  running minimum is used instead.

## Consequences

- **Good:** rides the Step-5 interface unchanged; cloning is a shared `FleetState` + a fresh
  `FleetStreams` (the KI-1 no-hidden-state invariant, so cloned futures never correlate — the
  property a 1e-9 estimate rests on, ADR 0001); no per-particle weights; `is_terminal` unchanged; an
  unbiased product estimator; a defensible CI from replications; and the *same* `P(LoS)` as MC, so it
  is validatable rather than taken on faith.
- **Cost / obligation:** fixed shells need hand-tuned spacing or the run can collapse (`S_k = 0`) —
  carried until AMS. The pos=40 rung must be green before any rare-regime number is trusted, and
  every result is reported as **P ± CI**, never a bare probability (design-philosophy #4). The
  validation-ladder probe must be promoted from scratch to `scripts/ips_validation_probe.py` so the
  MC anchor regenerates from one command.
- **Reproducibility:** the per-particle RNG subtree deepens by one `SeedSequence.spawn` at each
  resampling (ADR 0001 handles this reproducibly); a run stays `config + seed + code-hash → (P, CI)`.

## Relations

- Realises the "IPS level/importance function" decision forward-linked from
  [[0004-layered-directed-design-for-multiaircraft-and-ips]]; depends on
  [[0001-rng-per-particle-spawn]] and the Step-5 estimator interface (`opencdarr/fleet.py`:
  `FleetEnv` / `FleetState` / `FleetStreams`).
- Anchored by [[rare-event-validation-ladder]]; resolves the open questions in `phase-8-plan`
  (**when to stop** = `is_terminal` unchanged; **benchmark vs classical MC** = the §6 gate;
  **variance** = the §5 replication CI).
- Forward-links: [[ips-adaptive-levels]] (AMS), a future tCPA-importance ADR, and the collision-radius
  rare event that replaces `d_m = rpz`.
- Roadmap v0.4; how-to Step 6.
