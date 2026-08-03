# SIDs 2026 paper — outline

**Venue:** 15th SESAR Innovation Days (2026). IEEE two-column conference format, **8 pages
including references**. Same venue as `my-paper/auto-sep-cns-effect.pdf` (SIDs 2024), so the
paper can be framed openly as the continuation of that contribution.

**Topic areas claimed** (in order of fit): *Unmanned aerial systems (UAS), U-space and Urban Air
Mobility*; *Communications, navigation and surveillance (CNS)*; *Uncertainty, applied modelling and
optimisation techniques*; *Safety, security and resilience*.

**Voice:** `my-paper/writing-style.md`, proceedings register — impersonal by default, a light
first-person plural allowed in the roadmap and method framing, British spelling throughout.

---

## 1. The thesis (what makes this more than a tool paper)

A pure "here is our software" paper is weak at SIDs. The load-bearing argument has to be a
**measurement that the prior setup could not make**, with the platform as the instrument that
makes it possible.

That measurement is already named in the author's own corpus. `uq-cdr` (RESS 2026) closes on:
*"the look-ahead horizon and protected zone can be tuned to achieve a desired target level of
safety."* Nothing in that paper — or in `auto-sep-cns-effect`, or `effect-of-cns` — can actually
**measure** a target level of safety (TLS). Separation standards sit near 10⁻⁹ per flight hour;
plain Monte Carlo needs on the order of 10¹¹ encounters to resolve a probability that small, and the
handbook-scale Monte Carlo runs in the prior papers read `P(LoS) = 0` in every well-resolved cell.
A recommendation stated at 10⁻² and applied at 10⁻⁹ is an extrapolation, not a result.

**Thesis:** OpenCDaRR closes that loop. It carries the CNS uncertainty models of the prior corpus
into an environment whose *same* interface is driven by both plain Monte Carlo and a Blom–Bakker
interacting particle system (IPS), so a CNS performance specification can be propagated all the way
to a loss-of-separation probability in the rare regime, **with a confidence interval**, and a
protected-zone or look-ahead requirement can be read off rather than extrapolated.

The capability tour then earns its place: every module shown is a term in that propagation chain,
and each one is anchored to a finding from a previous paper (Table I).

---

## 2. Title options

1. **OpenCDaRR: An Open Platform for Conflict Detection, Resolution and Recovery under CNS
   Uncertainty at Rare-Event Scale** — descriptive, safest.
2. **From CNS Performance to a Target Level of Safety: Rare-Event Evaluation of Tactical Separation
   in U-Space** — leads with the result, mentions the tool in the abstract. *Recommended.*
3. **Measuring What Monte Carlo Cannot Reach: Separation Performance under CNS Uncertainty in
   U-Space** — punchier, slightly less conventional for the venue.

Recommend **(2)** with the platform named in the subtitle or the first abstract sentence: it keeps
the paper reviewable as a *results* paper while still delivering the tool.

---

## 3. Contributions (numbered list to appear at the end of Section I)

1. An open, reproducible simulation platform in which conflict **detection, resolution and
   recovery** (CDaRR) and the full **CNS chain** — navigation error, broadcast cadence, reception
   loss, latency, surveillance staleness — are pluggable, so a separation algorithm is evaluated on
   what CNS delivers rather than on the ground truth.
2. An explicit model of **asymmetric situational awareness**: each aircraft's perception of itself
   and of the intruder is generated separately, so the two aircraft in one encounter do not share a
   world state. Prior studies (including the author's) applied a single symmetric noise draw.
3. A **rare-event estimator** (fixed-effort multilevel splitting, Blom–Bakker IPS) driven through
   the same environment interface as plain Monte Carlo, validated against it in a not-too-rare
   regime and shown to return a bounded estimate where Monte Carlo reads exactly zero.
4. A **cross-implementation replication** of the pairwise MVP-versus-VO results of
   [SIDs 2024 / RESS 2026], obtained on an independent code base with no BlueSky runtime
   dependency — evidence that those findings are properties of the algorithms, not of one simulator.
5. A demonstration that carries a **protected-zone requirement** down into the rare regime, where
   the earlier recommendation could only be stated as an extrapolation.

---

## 4. Abstract (draft, follows the six-step recipe in `writing-style.md`)

> *(context)* Tactical separation in U-space is expected to keep uncrewed aircraft apart at
> separation minima of 50 to 200 metres, using state information delivered by communication,
> navigation and surveillance (CNS) systems whose errors are of the same order as the separation
> itself. *(gap)* Previous work has quantified how navigation error and communication limitations
> degrade conflict detection and resolution, but the resulting recommendations are measured at
> probabilities near 10⁻², whereas the safety targets they are meant to inform lie many orders of
> magnitude lower; brute-force Monte Carlo cannot bridge that distance. *(what)* This paper presents
> OpenCDaRR, an open platform that evaluates conflict detection, resolution and recovery under CNS
> uncertainty, and applies it to propagate a CNS performance specification to a loss-of-separation
> probability in the rare regime. *(method)* The CNS chain — navigation error, broadcast cadence and
> reception probability, latency and surveillance staleness — is modelled per aircraft, producing
> asymmetric situational awareness; the same environment interface is driven by plain Monte Carlo
> and by a Blom–Bakker interacting particle system. *(results, with numbers)* The platform
> reproduces the reported advantage of Modified Voltage Potential over Velocity Obstacle on an
> independent implementation; the rare-event estimator agrees with Monte Carlo at
> P(LoS) ≈ 2.8 × 10⁻² and returns a bounded estimate in a regime where 30 000 Monte Carlo encounters
> observe no event at all; *(payoff — number to be filled from the final run)* and a protected-zone
> requirement is read off at P(LoS) ≈ 10⁻⁵ rather than extrapolated from 10⁻². *(implication)* The
> platform and its models are released openly, so CNS requirements for U-space separation can be
> derived at the probability scale at which they are specified.

**Keywords:** U-space; conflict detection and resolution; CNS uncertainty; rare-event simulation;
target level of safety; UAS.

---

## 5. Section outline with page budget

Total 8 pages including references. Budget in column-pages (2 columns/page).

### I. Introduction — 1.0 page

- **Motivation-first**, per house style: projected UAS traffic (reuse the cited figure from
  `val-test` — 400 000–800 000 drones in European airspace by 2030 — or the several-hundred-thousand
  by 2030 estimate used in `uq-cdr`); U-space / CORUS CONOPS; the tactical separation layer.
- Why CNS matters more for UAS than for crewed aircraft: 50–200 m minima versus 5 NM, so a 10 m
  navigation error is proportionally enormous. (Straight from `uq-cdr` §1 — reuse the framing, not
  the sentences.)
- **The author's own three findings, stated as established ground**, each one sentence with its
  citation: navigation error makes state-based detection probabilistic and favours resolution rules
  that explicitly maximise ‖d_CPA‖ [RESS 2026]; update rate and reception probability degrade
  resolution independently of navigation accuracy [ICRAT 2024], and reception probability itself is
  now modelled from operational data [JOAS 2024]; flight validation shows latencies beyond five
  seconds degrading a real advisory service [CERTIFLIGHT].
- **The gap, named explicitly.** Three parts, in this order:
  (i) each channel has been studied largely in isolation, on a symmetric world state;
  (ii) recovery — when an aircraft may return to its mission — is absent from the CD&R literature
  the corpus sits in, yet it decides the tail of the miss-distance distribution;
  (iii) decisively, all of it is measured at 10⁻²–10⁻³ while the requirement is a TLS orders of
  magnitude lower, and plain Monte Carlo cannot get there.
- **Aim statement**, plainly: *"The aim of this work is to make the propagation from a CNS
  performance specification to a loss-of-separation probability measurable at the scale at which
  safety targets are stated, and to release the platform that does it."*
- Contributions list (Section 3 above).
- **Roadmap paragraph.** "The remainder of this paper is organised as follows…"

### II. Background and positioning — 0.75 page

Three compact paragraphs, each ending in the gap it leaves:

1. **Simulation platforms.** BlueSky (airspace-scale, mature plugin ecosystem, whole-day scenarios —
   and the tool the author's own prior work used) versus the deliberately narrow scope here: a few
   aircraft in one encounter, run 10⁴–10⁶ times, with the estimator as a first-class component.
   Be explicit and generous — this is *complementary*, not a replacement, and say so. Link BlueSky
   to its fixed repository URL.
2. **CD&R under uncertainty.** Probabilistic detection methods (Gaussian integration over collision
   zones, stochastic prediction, geometric probability bounds) and optimisation-based systems
   (ACAS X, ACAS sXu) — each with benefit *and* downside, per house style — then the state-based
   geometric family this work stays in, and why (analytical tractability, implicit coordination).
3. **Rare-event estimation in ATM.** Blom et al.'s interacting particle system for collision risk in
   free flight; multilevel splitting and importance splitting generally. Gap: these are applied to
   airspace-scale free-flight models, not to a CNS-uncertainty-driven tactical separation loop, and
   not in a form a CD&R researcher can attach their own resolver to.

### III. Platform architecture — 1.5 pages

*One-line orienting sentence, then:*

- **III-A. The simulation loop** (~0.4 p). One step: perceive → detect → resolve → recover →
  command → integrate. State the loop as the fixed spine and every box in it as a swappable
  interface. **Fig. 1.**
- **III-B. Kinematics and flight envelope** (~0.3 p). Multirotor (holonomic, independent yaw) and
  fixed-wing (coordinated turn, finite roll rate, bank-limited turn radius R = V²/(g tan φ));
  performance limits held as data, separate from the integrator, so a new airframe is a value, not a
  code change. Validation: analytically, and against a recorded BlueSky trajectory. **This is the
  hook for the GA–UAS mixed encounter of `val-test`** — say so here in one sentence.
- **III-C. Separation: detection, resolution, recovery** (~0.4 p). State-based detection
  (t_CPA, ‖d_CPA‖, t_in — reuse the notation of `uq-cdr` exactly); MVP and VO as resolvers, with the
  composition rule over an intruder *set* (MVP sums avoidance vectors, VO takes the union of
  velocity obstacles); and the recovery stage — Past-CPA, FTR, Probabilistic FTR — introduced as the
  genuinely new stage relative to the CD&R framing of the prior corpus. Define **CDaRR** once here.
- **III-D. Environment: wind and the fleet** (~0.2 p). Steady uniform wind; N-aircraft environment
  with a layered-directed coordination model that reduces exactly to the pairwise case at N = 2
  (assert the reduction — it is tested bit-for-bit, and it is the credibility claim that lets the
  reader trust the multi-aircraft results).
- **III-E. Reproducibility by construction** (~0.2 p). Per-encounter RNG substreams spawned from one
  run seed, so a result is order-independent and parallel-safe; content-addressed caching;
  provenance cards written at run time. Frame as: an experiment in this paper is re-runnable from
  `config + seed + code hash`. Short, but reviewers at this venue reward it.

**Table I — capability ↔ prior finding map.** This table is the paper's answer to "why these
features": every module is there because a previous paper needed it and had to improvise it.

| OpenCDaRR module | Modelled quantity | Prior finding it carries forward |
|---|---|---|
| `cns/navigation` (GNSS, declared accuracy, outage) | position/velocity error, ADS-L-consistent 95 % bounds | detection becomes probabilistic; MVP robustness [RESS 2026], [SIDs 2024] |
| `cns/communication` (cadence, jitter, phase, reception probability, latency distributions) | update interval and message loss | update rate and reception probability degrade CD&R [ICRAT 2024]; reception model from OpenSky [JOAS 2024]; > 5 s latency observed in flight [CERTIFLIGHT] |
| `cns/surveillance` (last-known, staleness) | age of the intruder state each aircraft holds | asymmetric situational awareness — *new*; hinted at by the flight-test asymmetry |
| `cd/statebased` | t_CPA, ‖d_CPA‖, t_in | the three detection variables of [RESS 2026] |
| `cr/mvp`, `cr/vo` | resolution velocity, margin | MVP vs VO comparison [RESS 2026], [SIDs 2024] |
| `crr/pastcpa`, `crr/ftr`, `crr/probabilistic_ftr` | when to return to mission | *new stage* — absent from the prior CD&R framing |
| `kinematics/fixedwing`, `kinematics/multirotor` | mixed GA–UAS encounter | the flight-test geometry of [CERTIFLIGHT], in simulation |
| `wind` | steady uniform wind | *new* — an unmodelled term in all prior runs |
| `fleet` | N-aircraft, multi-intruder composition | the multi-conflict future work of [RESS 2026] |
| `ips` | P(LoS) in the rare regime, with CI | the TLS statement of [RESS 2026], now measurable |

### IV. Modelling CNS uncertainty — 0.8 page

The section that most directly extends the prior corpus; give it real technical content, not a
feature list.

- The chain per aircraft per tick: **navigation** produces each aircraft's own perceived state;
  **communication** decides whether and when that state is transmitted and received (cadence,
  jitter, phase offset, reception probability, latency drawn from a constant / uniform / lognormal
  distribution); **surveillance** decides what the receiver holds between receptions (last-known,
  with an explicit age).
- **The key claim, stated as its own short paragraph:** because navigation is drawn per aircraft and
  reception is decided per link, the two aircraft in one encounter hold *different* views of the
  same geometry, and neither view is the truth. A resolver evaluated on ground truth never meets
  that gap. **Fig. 2** (the 2×3 perceived-position panel) carries this.
- **Assumptions stated and justified**, per house style: zero-mean Gaussian navigation error
  (analytical tractability, alignment with ADS-L 95 % bounds ≈ ±2σ), with the limits acknowledged;
  steady uniform wind; horizontal-only encounters.
- Note the extension points in one sentence each — link gates for the communication channel, quality
  effects for navigation (outage) — so the "bring your own model" claim is concrete rather than
  asserted.

### V. Rare-event estimation — 0.75 page

- **Why it is needed**, in numbers: TLS ~10⁻⁹/fh; plain Monte Carlo needs ~10¹¹ runs; every
  well-resolved cell in the prior Monte Carlo campaigns read `P(LoS) = 0`, which is not a small
  number but an absence of measurement.
- **The method**, compactly: nest the rare event in shrinking shells on the *running minimum*
  separation (monotone, so crossings are one-way); fixed N particles; at each shell evolve until
  crossing (survivor) or termination (dropped), then resample survivors with replacement back to N;
  estimate is the product of survival fractions ∏ₖ Sₖ/N. Give the one equation and unpack it in
  words, per house style.
- **What is split and what is sampled** — the initial cloud samples the encounter geometry, the
  splitting acts on the forward CNS noise — so IPS estimates the same probability Monte Carlo does.
  This is the sentence that makes the validation meaningful; do not omit it.
- **Confidence intervals by independent replications**, because particles within one run interact
  through shared ancestors.
- Half a paragraph on parallel scheduling across particles and replications, no more.

### VI. Demonstration — 2.25 pages

Four case studies, each one figure, each opening with what it establishes and closing with the link
back to the thesis. Ordered so credibility is built before the payoff is claimed.

**VI-A. Replicating the pairwise result (≈0.5 p, Fig. 3).**
MVP versus VO under position uncertainty across crossing angle, IPR and post-resolution ‖d_CPA‖ —
the experiment of [SIDs 2024] / [RESS 2026], re-run on an independent implementation. The point is
*not* novelty; it is that the earlier conclusion survives a change of code base, which is exactly
the claim a platform paper must earn before anyone trusts its new numbers. State the agreement
quantitatively.
*Hedge the VO result explicitly* — the VO implementation here is the platform's own, and its
behaviour at shallow angles and low relative speeds should be reported as consistent with, not
independent confirmation of, the earlier finding.
→ *Evidence: `scripts/ipr_angle_sweep.py`; needs one clean run against the published numbers.*

**VI-B. The CNS chain, one channel at a time (≈0.6 p, Fig. 4).**
IPR against navigation accuracy for four nested CNS configurations: navigation only; + broadcast
cadence; + reception loss; + latency. The compounding is the finding — the channels the prior papers
studied separately do not simply add. Anchor the latency axis at the > 5 s value observed in the
flight test, and the reception probability at the OpenSky-derived model, so both prior results enter
as *inputs* rather than as citations.
→ *Evidence: `scripts/comm_ipr_sweep.py`, `scripts/ipr_fleet_comm_sweep.py`,
`vault/observations/{communication-reception-latency,broadcast-jitter,broadcast-phase-offset}.md` —
have; needs assembling into one figure.*

**VI-C. Recovery, mixed fleet, and beyond pairwise (≈0.6 p, Fig. 5).**
Three short results in one subsection, each a paragraph:
- **Recovery criteria** — Past-CPA vs FTR vs Probabilistic FTR on identical seeds across crossing
  angle. The near-parallel and shallow-angle geometries are where they separate, which is the same
  region [RESS 2026] identified as VO's weakness — make that connection explicitly.
- **Mixed GA–UAS fleet** — a fixed-wing and a multirotor in one encounter, i.e. the flight-test
  geometry of [CERTIFLIGHT] in simulation, with the bank-limited turn radius doing work the
  holonomic model cannot represent. One wind number alongside (steady 10 m/s dropping IPR from 0.99
  to 0.96 in the reference crossing) to show the environment term is live.
- **Multi-intruder composition** — MVP sums pairwise avoidance vectors and can under-correct on a
  symmetric double conflict; VO takes the union of velocity obstacles. Report the miss distances.
  **Hedge:** report this as a property of these implementations under a specific symmetric geometry,
  and as consistent with the known limitation of potential-field superposition — not as a general
  ranking. This is the multi-conflict item [RESS 2026] listed as future work.
→ *Evidence: `vault/observations/{recovery-criteria-comparison,mixed-fleet-daa,multi-intruder-vo-vs-mvp,ipr-under-wind}.md` — all have.*

**VI-D. Reaching the rare regime (≈0.55 p, Fig. 6 + Table II).**
The payoff, in three steps:
1. **The ladder.** Sweeping declared GNSS accuracy turns the rare event on continuously, from
   P(LoS) = 1 with the resolver off down to ≈4.7 × 10⁻⁴ at 10 m. This gives Monte Carlo something to
   agree *with*.
2. **The two gates (Table II).** Correctness — at P ≈ 2.8 × 10⁻² the IPS and Monte Carlo intervals
   overlap at comparable cost. Efficiency — at a fixed 90° crossing, 30 000 Monte Carlo encounters
   observe zero events while IPS returns a bounded estimate with a confidence interval. Report the
   small positive bias seen in the correctness gate openly; house style calls for caveats stated,
   not hidden.
3. **The requirement curve.** P(min sep < 50 m) against protected-zone radius and MVP margin, taken
   into the rare regime — the question [RESS 2026] could only answer at 10⁻². Close on the number:
   what radius the platform says is needed for a stated probability, and how far that is from the
   value the earlier extrapolation would have given.
→ *Evidence: `vault/observations/{rare-event-validation-ladder,ips-gate1-correctness,ips-gate2-efficiency}.md` — have.
Step 3 is in progress in `brouillon/rpz_50_vs_100.ipynb`; **this is the one result the paper still
needs**, and the schedule should be built around it.*

### VII. Discussion and limitations — 0.4 page

- What the platform deliberately does not do: airspace-scale traffic, real navaid/route data, live
  GUI, ATC scenario commands — point the reader to BlueSky, generously.
- Horizontal-only encounters; steady uniform wind; Gaussian navigation error; a small algorithm set.
- The VO implementation is the platform's own and its results should be read as provisional.
- IPS assumes the shell variable is monotone and that splitting acts only on the forward noise;
  the small positive bias observed at the correctness gate bounds how far the rare numbers should be
  trusted.
- One honest sentence on what "validated" means here: agreement with plain Monte Carlo where Monte
  Carlo is trustworthy, and with a recorded BlueSky trajectory for the kinematics — not flight data.

### VIII. Conclusion and future work — 0.35 page

- Recap what was done and found; restate the headline in plain terms — a CNS specification can now
  be propagated to a probability at the scale safety targets are written in, and the protected-zone
  recommendation of the earlier work can be checked rather than extrapolated.
- Generalise to the single takeaway principle, in the style of the prior conclusions: a separation
  rule should be evaluated on the information CNS actually delivers, and at the probability at which
  it will be certified.
- **Future work, each item with its reason:** new recovery criteria combining the divergence signal
  with the near-parallel safety of Probabilistic FTR, evaluated in simultaneous multi-conflict (the
  stated research priority); vertical manoeuvres; a reception-probability model driven by the
  OpenSky-derived fit rather than a fixed probability; wind fields with structure; and validation of
  the rare-event numbers against operational encounter data.
- Availability: repository URL, handbook URL, licence, and the exact commit the results were run at.

### References — ~0.5 page

Buckets: U-space / CORUS / EASA ADS-L specification; UAS traffic forecasts; state-based CD&R and the
MVP and VO originals; probabilistic detection and ACAS X / sXu; Blom et al. and the splitting /
rare-event literature; BlueSky; and the author's own four papers, cited where the prose above says
[RESS 2026], [SIDs 2024], [ICRAT 2024], [JOAS 2024], [CERTIFLIGHT].

---

## 6. Figures and tables

| # | Content | Width | Status |
|---|---|---|---|
| Fig. 1 | Simulation loop / architecture: one step, with the swappable interfaces marked | 1 col | to draw |
| Fig. 2 | Asymmetric situational awareness — perceived own / intruder / relative position vs truth | 2 col | **have** (`docs/img/perceived-position.png`) |
| Fig. 3 | MVP vs VO: IPR and ‖d_CPA‖ vs crossing angle under position uncertainty | 2 col | needs one clean run |
| Fig. 4 | IPR vs navigation accuracy for four nested CNS configurations | 1 col | data have, figure to assemble |
| Fig. 5 | Recovery criteria across angle + mixed GA–UAS encounter trajectories | 2 col | **have**, needs merging into one panel |
| Fig. 6 | Rare-event: splitting ladder and P(LoS) vs protected-zone radius into the rare regime | 2 col | ladder/gates **have**; requirement curve **to run** |
| Table I | Capability ↔ prior finding map | 1 col | drafted above |
| Table II | IPS validation gates: MC vs IPS, both regimes | 1 col | **have** |

Six figures plus two tables is at the upper limit for 8 IEEE pages. **Compression valves, in order:**
merge Section II into Section I; drop Fig. 4 to a two-line result in prose; shorten III-E to two
sentences. Do not cut VI-D.

Plot style, per the house convention: PNG, no grid, no figure title, concise subplot titles and
legends, with the interpretation carried in the caption.

---

## 7. What still has to be produced

1. **The requirement curve (VI-D step 3).** The only genuinely missing result, and the one the
   thesis rests on. `brouillon/rpz_50_vs_100.ipynb` is the seed.
2. **A clean MVP-vs-VO angle sweep** matched to the published [SIDs 2024] / [RESS 2026] conditions,
   so the replication claim in VI-A can be quantified rather than asserted.
3. **Fig. 1** — the architecture diagram; nothing equivalent exists yet.
4. **Figure assembly** for VI-B and VI-C from existing observation runs.
5. A tagged commit and an archived DOI for the availability statement.

## 8. Risks

- **"This is a tool paper."** Mitigated by leading with the rare-event measurement and keeping the
  capability tour subordinate to it (Table I is the device that does this).
- **Self-replication reads as thin novelty.** Mitigated by framing VI-A honestly as
  cross-implementation validation, spending only half a page on it, and putting the new material in
  VI-B through VI-D.
- **Eight pages is tight.** The outline is deliberately over-budget by about half a page; the
  compression valves are listed above.
- **Overlap with the RESS paper.** Keep the uncertainty-propagation derivations to a citation and a
  sentence; this paper's derivation content is the splitting estimator, not the detection algebra.
