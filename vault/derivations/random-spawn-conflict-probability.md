# Derivation — conflict probability for the simultaneous random-spawn disc

How often a fleet drawn by the Groot–Ellerbroek–Hoekstra entry rule contains a conflict, before
any separation logic acts. This is the **demand** the separation manager is handed; `P(LoS)` is
what is left after it acts, and at the settings these scenarios run the two differ by a factor of
about 4300 (0.60 against 1.40e-4 at `N = 6`, [[mc-vs-ips-campaign]]). Because every aircraft flies
a straight line from a common start time, the
closest approach of a pair is a closed form in the drawn parameters, so the probability is an
integral rather than a simulation.

- Traffic model: [`opencdarr.scenario.random_traffic`](../../opencdarr/scenario/traffic.py) and
  [`opencdarr.fleet.MeasurementArea`](../../opencdarr/fleet.py) — the shipped environment, flown by
  [`examples/handbook/ring_mc_vs_ips.ipynb`](../../examples/handbook/ring_mc_vs_ips.ipynb) part 2
  and by [`scripts/mc_vs_ips_campaign.py`](../../scripts/mc_vs_ips_campaign.py).
- Computed by: [`scripts/random_spawn_conflict.py`](../../scripts/random_spawn_conflict.py). Every
  number below is from one run of

  ```
  PYTHONPATH=. python scripts/random_spawn_conflict.py --log2-points 24 \
      --fleet-draws 500000 --sim --draws 20000
  ```
- Validated by: the same draws flown through [`opencdarr/fleet.py`](../../opencdarr/fleet.py) with
  the resolver removed (`--sim`), §7.
- Sibling: [`cpa-detection.md`](cpa-detection.md) — the same relative-motion algebra, on perceived
  states rather than drawn ones.
- Source: Groot, D.J., Ellerbroek, J., Hoekstra, J.M. (2024), *Analysis of the impact of traffic
  density on training of reinforcement learning based conflict resolution methods for drones*,
  Eng. Appl. Artif. Intell. 133:108066 — §3.1.2 and Fig. 4 (left), Eq. (8).

## Symbols

| symbol | code | meaning | unit |
|--------|------|---------|------|
| $R$ | `R_INNER` | measured (experimental) disc radius | m |
| $R_o$ | `R_OUTER` | spawn circle radius | m |
| $D$ | `D`, `rpz` | protected-zone radius | m |
| $V$ | `V` | common ground speed | m/s |
| $\psi_k$ | `heading` | aircraft $k$'s track, $\sim U[0, 2\pi)$ | rad |
| $x_k$ | `offset` | signed perpendicular offset of the track from the centre | m |
| $N$ | `n` | aircraft released together | — |
| $p$ | `p_pair` | probability that one *pair* conflicts | — |

Values throughout: $R = 1000$, $R_o = 1200$, $D = 50$, $V = 10$.

## 1. The traffic model

Aircraft $k$ draws a heading $\psi_k \sim U[0,2\pi)$ and an offset $x_k \sim U[-R, R]$. With
$\hat{\mathbf d} = (\sin\psi, \cos\psi)$ and $\hat{\mathbf n} = (\cos\psi, -\sin\psi)$, its track
is the line through $\mathbf f = x\,\hat{\mathbf n}$ (the point of closest approach to the centre)
along $\hat{\mathbf d}$. It is released on the spawn circle, a half-chord

$$ h(x) = \sqrt{R_o^2 - x^2} $$

back along its own track, so at time $t$ (all aircraft share $t = 0$)

$$ \mathbf p(t) = \mathbf f + \bigl(Vt - h(x)\bigr)\,\hat{\mathbf d} $$

**Why $\arcsin$ appears in the paper.** Eq. (8) writes the entry *bearing* as
$\psi + 180^\circ + \arcsin(u)$, $u \sim U[-1,1]$; that is this construction in polar form, since
$\arcsin$ maps a uniform $u$ onto the bearing whose perpendicular offset is uniform. Spreading
entry bearings uniformly around the perimeter instead would crowd traffic toward the edge; it is
the *offset* that must be uniform for the traffic to be homogeneous over the measured area.

**One deviation, deliberate.** The paper references the offset to $R_o$, so ~17% of aircraft graze
past the inner disc and never enter — harmless when the controlled quantity is a density, awkward
when it is a count of 4, 6 or 8. Here $x \sim U[-R, R]$ (the *inner* radius) and the entry point is
projected back out to $R_o$, so all $N$ cross the measured disc.

The disc is entered and left at

$$ t^{\text{in}} = \frac{h(x) - c(x)}{V}, \qquad t^{\text{out}} = \frac{h(x) + c(x)}{V},
\qquad c(x) = \sqrt{R^2 - x^2} $$

## 2. The conflict criterion — timing is in it

For a pair, relative position is linear in time, $\mathbf r(t) = \mathbf A + \mathbf B t$, with

$$ \mathbf A = \mathbf p_1(0) - \mathbf p_2(0), \qquad \mathbf B = V(\hat{\mathbf d}_1 - \hat{\mathbf d}_2) $$

and the pair is measurable only while **both** are inside the disc,
$t \in [t_{\text{lo}}, t_{\text{hi}}]$ with $t_{\text{lo}} = \max_k t^{\text{in}}_k$ and
$t_{\text{hi}} = \min_k t^{\text{out}}_k$. The unconstrained closest approach is at
$t^\star = -(\mathbf A\cdot\mathbf B)/|\mathbf B|^2$, so

$$ s_{\min} = \bigl|\mathbf A + \mathbf B\,\mathrm{clip}(t^\star, t_{\text{lo}}, t_{\text{hi}})\bigr|,
\qquad \text{conflict} \iff t_{\text{hi}} > t_{\text{lo}} \ \wedge\ s_{\min} < D $$

This is **not** a test of whether the two tracks intersect. Two tracks can cross and be separated
by minutes; two tracks that never cross can converge to within $D$. §8 measures how much of the
answer that distinction is.

Degenerate case: $|\mathbf B| = 0$ (identical headings) leaves $s_{\min} = |\mathbf A|$, constant.

## 3. Reduction to three dimensions

The whole construction — both circles, the heading law, the offset law — is invariant under
rotation about the centre, so only the *relative* heading survives. Fix $\psi_1 = 0$ and let
$\Delta\psi = \psi_2 - \psi_1 \sim U[0, 2\pi)$:

$$ p = \frac{1}{2\pi\,(2R)^2} \int_0^{2\pi}\!\!\int_{-R}^{R}\!\!\int_{-R}^{R}
        \mathbb{1}\bigl[s_{\min}(\Delta\psi, x_1, x_2) < D\bigr]\; dx_2\, dx_1\, d(\Delta\psi) $$

Three dimensions, with a closed-form integrand — cheap to evaluate to five digits.

## 4. The pair probability

Sobol quasi-Monte-Carlo, scrambled, half-sample spread as the error indicator:

| points | $p$ | half-sample spread |
|---|---|---|
| $2^{16}$ | 0.06308 | 0.00003 |
| $2^{18}$ | 0.06272 | 0.00028 |
| $2^{20}$ | 0.06270 | 0.00002 |
| $2^{22}$ | 0.06274 | 0.00001 |
| $2^{24}$ | 0.06278 | 0.00003 |

$$ \boxed{p = 0.0628} $$

## 5. From pairs to fleets

The expected number of conflicting pairs is **exact** by linearity — pair events need not be
independent for their expectations to add:

$$ \mathbb{E}[\text{conflicting pairs}] = \binom{N}{2}\,p $$

$P(\text{at least one})$ is a different quantity and needs the dependence, because pairs share
aircraft. Sampled over 500 000 fleets (exact sampling of the draw law, still no trajectory
integration):

| $N$ | pairs | $\binom{N}{2}p$ | sampled mean | $P(\ge 1)$ | $1-(1-p)^{\binom N2}$ | Var | binomial Var |
|---|---|---|---|---|---|---|---|
| 2 | 1 | 0.0628 | 0.0625 | **0.0625** | 0.0628 | 0.0586 | 0.0588 |
| 4 | 6 | 0.3767 | 0.3771 | **0.3155** | 0.3223 | 0.3784 | 0.3530 |
| 6 | 15 | 0.9417 | 0.9409 | **0.6022** | 0.6219 | 1.0090 | 0.8826 |

Two things to read off, both of which have bitten a reader already:

**$\binom N2 p$ is a count, not a probability.** At $N=6$ it is 0.94 *pairs per fleet*; at $N=8$ it
is 1.76, which no probability can be. The identity relating the two is exact:

$$ P(\ge 1) = \mathbb{E}[\text{count}] - \sum_{k \ge 2}(k-1)\,P(\text{count}=k) $$

$0.9409 - 0.3391 = 0.602$ ✓ (both terms from the sampled distribution, so the check is internal).
Per 1000 fleets of six: 398 have no conflict, 364 have one, 164 have
two, 54 have three, 14 have four, 5 have five or more — 940 pairs spread over 602 fleets, so a
fleet that has any averages **1.56** of them.

**Pairs are positively dependent, so the independence formula overshoots.** Var(count) = 1.009
against the binomial's 0.883 at $N=6$, 14% over-dispersed; the sampled distribution has *more*
conflict-free fleets than independence predicts (0.398 vs 0.378) *and* more heavily loaded ones
(0.0144 vs 0.0104). Conflicts cluster: an aircraft crossing the middle of the disc while the others
are there tends to conflict with several at once, and a fleet whose members graze the edge at
staggered times has none. Clustering concentrates the same expected count into fewer fleets, which
pushes $P(\ge1)$ *below* $1-(1-p)^{\binom N2}$.

## 6. Cross-check — the swept-area gas model

An independent derivation, to catch a construction error rather than to refine the number. With
headings uniform, $\mathbb{E}|v_{\text{rel}}| = 2V\,\mathbb{E}|\sin(\Delta\psi/2)| = 4V/\pi$, and
the mean time in the disc is the mean chord over the speed, $\bar T = \pi R/2V$:

$$ \mathbb{E}[\text{conflicts}] \approx \binom{N}{2}\,\frac{2 D\,\mathbb{E}|v_{\text{rel}}|\,\bar T}{\pi R^2}
   = \binom{N}{2}\,\frac{4 D}{\pi R} $$

giving 0.0637, 0.3820, 0.9549 for $N = 2, 4, 6$ against the exact 0.0628, 0.3767, 0.9417 — within
1.4%. The formula reduces to a pleasant statement: **the pair conflict probability is
$4D/\pi R$**, the protected-zone diameter over a quarter-circumference, independent of speed.

## 7. Validation — the same draws, flown

`--sim` re-flies each draw through `build_env` with `resolver=None`, `recovery=None` and no
navigation model, and applies the same disc gate. This exercises the whole environment —
kinematics, waypoint guidance, the segment-minimum measurement, the gate — against algebra that
shares none of it. 20 000 draws per size, seed `root_seed_sequence(3)`:

| $N$ | analytic $P(\ge 1)$ | flown | 95% Wilson |
|---|---|---|---|
| 2 | 0.0625 | **0.0640** | [0.0606, 0.0674] |
| 4 | 0.3155 | **0.3152** | [0.3088, 0.3217] |
| 6 | 0.6022 | **0.6051** | [0.5984, 0.6119] |

Every analytic value lies inside the flown interval. ✓

## 8. What the timing contributes

Over $2^{24}$ pairs, comparing the time-blind question with the real one:

| | |
|---|---|
| the two tracks cross inside the disc | 0.5000 |
| the two aircraft pass within $D$ | 0.0628 |
| of the crossing pairs, those in conflict | 0.1151 |
| of the conflicts, those whose tracks never cross | 0.0832 |
| as above, but with the pair's start times decorrelated | 0.0161 |

Half of all pairs cross tracks inside the disc — an exact-looking 0.5000 worth its own note — but
87% of those crossings are separated in time, and 8% of the conflicts are between tracks that never
meet at all (near-parallel pairs converging slowly). Timing is most of the answer.

The last row is the sharpest caveat. Offsetting one aircraft's clock by $\pm$ one mean transit
drops the pair probability from 0.0628 to 0.0161, a factor of 3.9. **These numbers belong to the
simultaneous-release model**, where $N$ aircraft are launched together; the paper's own model
releases them on an interval set by a density, and a steady stream at the same instantaneous count
is a far safer airspace purely from timing. Anything derived here transfers to a stream only
through that factor.

## Limitations

- **Simultaneous release**, per the note above — the dominant caveat.
- **Unresolved straight-line flight.** This is conflict demand. With `MVP` + `ProbabilisticFTR` and
  a 10 m / 1 m/s fix, the full campaign measures `P(LoS)` = 1.40e-4 [1.15e-4, 1.70e-4] at $N = 6$
  ([[mc-vs-ips-campaign]]), so the separation manager converts a 60% chance of a conflict into a
  0.014% chance of losing separation. Note the asymmetry in what each number costs: the demand
  above is a closed form, while pinning the failure to ±10% took 720 000 flown encounters.
- **Common speed.** All aircraft fly $V$; a speed distribution changes
  $\mathbb{E}|v_{\text{rel}}|$ and both the integral and the gas model with it.
- **Horizontal only**, and no wind. The source paper separates by heading into vertical layers and
  studies vertical manoeuvres; none of that is modelled here.
- **The conflict criterion is geometric**, not `StateBased`. It asks whether the pair *would* come
  within $D$, not whether a detector with a finite lookahead on noisy perceived states would say so
  — which is the right question for demand, and the wrong one for detector performance.
