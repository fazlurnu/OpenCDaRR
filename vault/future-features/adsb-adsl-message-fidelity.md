# Future feature — reported states that look like real ADS-B / ADS-L messages

**Target version: post-v1.0** (`docs/roadmap.md`'s "community long game") — tentative. Nothing here
is load-bearing for the next paper; it becomes worth doing when a study needs to be *calibrated
against published ADS-B quality statistics* rather than against chosen parameters.

## What

A broadcast in this package is an `AircraftState` with a declared accuracy attached. A real ADS-B
or ADS-L message is a **quantised** state plus a set of **category-coded** quality indicators. The
gap between those two is small in places and structural in others, and it is worth naming in one
place rather than rediscovering it per experiment.

Three things separate what we broadcast from what a receiver would actually decode.

### 1. The quality indicators are categories, not numbers

`pos_ci95` and `vel_ci95` map cleanly onto **NACp** and **NACv** — not by analogy, they are the
same quantity. NACp encodes EPU, the 95% horizontal position accuracy; NACv the same for velocity.
So `pos_ci95_declared` already *is* "the NACp this transponder puts on the wire".

What is missing is that the wire carries a **bucket**, so a receiver never recovers the number:

| `pos_ci95` | NACp | receiver reads back | inflation |
|---|---|---|---|
| 3.0 m | 10 | 10.0 m | 3.3× |
| 12.0 m | 9 | 30.0 m | 2.5× |
| 20.0 m | 9 | 30.0 m | 1.5× |
| 40.0 m | 8 | 92.6 m | 2.3× |
| 200.0 m | 6 | 555.6 m | 2.8× |

Quantisation is therefore a systematic **under-declaration** of 1.3–3.3×: every aircraft looks
worse than it is, and anything sizing uncertainty off the broadcast — `ProbabilisticFTR` — is
correspondingly conservative. The model can already express this exactly
(`pos_ci95=12.0, pos_ci95_declared=30.0`); what it lacks is the helper that does it for you.

### 2. NIC / SIL — integrity, which is a different axis from accuracy

NIC encodes a containment radius `Rc`; SIL bounds `P(|error| > Rc)`. That is a statement about the
**tail**, where NACp is a statement about the **scale**. A single 95% figure cannot carry it.

The useful observation is that NIC is not independent data. It is a second point on the same radial
CDF that `ci95` already anchors, so **the distribution you chose has already decided it**. Measured
at a fixed `ci95 = 10 m`, i.e. an identical NACp:

| distribution | `Rc` @ 1e-5 | ratio to ci95 |
|---|---|---|
| `gaussian` | 19.7 m | 1.97× |
| `anisotropic (3:1)` | 21.7 m | 2.17× |
| `mixture (3.0, 0.10)` | 35.6 m | 3.56× |
| `mixture (5.0, 0.02)` | 74.2 m | 7.42× |

A 3.8× spread — three NIC categories — across distributions that all report the same NACp. That is
the shape axis NACp is blind to, and it is exactly what NIC exposes.

### 3. The state itself is quantised

Position is not sent as a float pair. ADS-B uses Compact Position Reporting, and ground speed and
track are sent in finite increments, so a receiver's decoded state is on a lattice regardless of
how good the sensor was. This is a second, independent quantisation on top of the accuracy
metadata, and unlike the metadata it perturbs the number the CDR logic actually acts on.

## Why deferred

Came up while writing the navigation handbook page: the question "how do `pos_ci95` / `vel_ci95`
translate into NIC and NAC" turned out to have a clean answer for NAC, a *derived* answer for NIC,
and no answer at all for state quantisation. None of it blocks current work — every published
result here sets accuracy directly and compares like with like, so the buckets would cancel.

It becomes load-bearing the moment a study wants to say "these are DO-260B NACp 9 aircraft" and
have that mean the same thing as it means in someone else's data.

**The ADS-L side is the less certain half.** ADS-L (EASA's lower-airspace standard, the one that
actually matters for the BVLOS drone case this package targets) has its own field set and its own
resolutions, and I have not checked them against the specification. The NAC/NIC/SIL structure above
is DO-260B / ED-102A and should itself be verified against the standard before anything is
published on it — particularly NIC 6, which has sub-cases keyed on the NIC supplement bits.

## Sketch (shape only, not designed)

An `opencdarr/cns/adsb_quality.py`, in three layers of increasing value:

1. **`nacp_for(ci95)` / `nacv_for(vel_ci95)` / `ci95_for_nacp(code)`** — the bucket tables, and a
   `quantise_declaration(state)` that sets `*_ci95_declared` to the bucket bound. Small, and makes
   the 1.3–3.3× effect above a sweepable axis rather than a footnote.
2. **`containment_radius(distribution, ci95, p_exceed)` and `nic_for(rc)`** — `Rc` **derived** from
   whatever distribution is configured, never stored.
3. **`mixture_for(ci95, rc, p_exceed)`** — the inverse, and the one with real research value: given
   a published (NACp, NIC, SIL) triple, solve `tail_ratio` / `tail_weight` so the distribution
   satisfies both constraints. This turns third-party ADS-B quality statistics into a calibrated
   noise model instead of a chosen one. It also explains why `make_mixture_gaussian` exists at all:
   `gaussian` has one free parameter and can satisfy only one constraint.

Layer 2 is cheaper than it looks. The isotropic mixture has a closed-form tail,

    P(r > R) = p * exp(-R^2 / 2*sigma1^2) + (1 - p) * exp(-R^2 / 2*sigma2^2)

verified against 2M samples at 25.27 vs 25.26 m (1e-3) and 35.74 vs 36.17 m (1e-5). The closed form
is not a convenience — **SIL 3 is 1e-7 and sampling cannot reach it** (4M draws barely resolve
1e-5), so an analytic tail is the only way to quote a containment radius at the SIL that matters.
The anisotropic variants have no closed form, but `_radial_cdf` already integrates numerically and
can be reused.

Layer 3 needs real data to be worth anything, which is the honest reason this is deferred rather
than built.

## The design constraint, if this is ever built

**`Rc` must be derived, never a settable field.** A stored containment radius could be set
inconsistent with the error actually being drawn — an unfalsifiable number — and nothing in the
stack would consume it. That is precisely the failure [[todo-might-be-a-bug]] §7 records for
`pos_ci95`, and the check `run_experiment` now carries exists to stop a repeat.

The one place a stored integrity field *would* belong is a **declared** NIC that disagrees with the
true one: a transponder claiming tighter containment than its receiver delivers. That is the same
pattern as `GnssOutage(declare=False)` and belongs with the other declared metadata, not with the
distribution parameters.

## Relations

- [[0021-navigation-extension-by-quality-effects]] §2 — the declared-vs-sensed split these
  indicators would ride on; quantisation is a third source of the same disagreement, alongside the
  scenario's static claim and an effect's dynamic one.
- [[gps-noise]] — where `ci95` is defined and why the ellipse is axis-aligned. The "a scalar
  carries scale but never shape" argument there is what makes NIC a genuinely separate axis.
- [[todo-might-be-a-bug]] §7 — the inert-declared-field failure mode the derived-not-stored rule
  above exists to avoid.
- `opencdarr/cns/noise_distributions.py` — `make_mixture_gaussian`'s two shape parameters are what
  a two-constraint calibration would solve for.
