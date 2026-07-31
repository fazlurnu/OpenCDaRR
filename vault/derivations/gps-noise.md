	# Derivation — GPS self-measurement noise (navigation, 2D)

How an aircraft measures its **own** state (position + velocity) with error, to broadcast. This
is the **N** of CNS: the error is at the source, applied once; others perceive it via the
broadcast (Phase 3a). Re-derived from `sim_models/noise_model.py` /
`noise_distributions.py`, 2D.

- Implemented by: [`opencdarr/cns/navigation.py`](../../opencdarr/cns/navigation.py) (`GnssNavigation`)
- Distributions: [`opencdarr/cns/noise_distributions.py`](../../opencdarr/cns/noise_distributions.py)
- Validated by: [`tests/test_cns_navigation.py`](../../tests/test_cns_navigation.py)

## Where CI95 lives

`pos_ci95` / `vel_ci95` are **fields on `AircraftState`** (its own declared measurement
accuracy), not constructor parameters of `GnssNavigation` — the same reasoning as `turn_rate`:
accuracy is a property of *that aircraft's* sensor, can differ between aircraft, and may evolve
over a run (e.g. degrading GPS coverage), so it must travel with the clonable state, not sit on a
shared, fixed navigation object. `GnssNavigation.measure(state, true, t, rng)` reads
`true.pos_ci95` / `true.vel_ci95` to size the noise, and stamps the **declared** accuracy onto the
broadcast — a receiver gets the sender's claim *with* the message, as ordinary state, with no
separate channel. Default `0.0` on both fields means a perfect sensor (no noise), the same
"neutral default" convention as `bank = 0.0`.

The drawn and the declared accuracy are the same number unless the aircraft says otherwise.
`pos_ci95_declared` / `vel_ci95_declared` (default `None` = claim the truth) let a transmitter
declare something *other* than what its sensor delivers — an over-confident claim being the
integrity failure RAIM exists to catch. A broadcast still carries exactly one accuracy, in
`pos_ci95`: only a *true* state ever needs both numbers. A `NavEffect` can scale either one on top
of that, which is how a degrading receiver either admits its degradation or hides it
([[0021-navigation-extension-by-quality-effects]] §2).

## Position error — CI95 to σ

Position error is a zero-mean 2D isotropic Gaussian, each axis $N(0, \sigma^2)$. GPS/ADS-B
accuracy is quoted as a **95% radial CI** (the radius containing 95% of fixes). The radial
distance is Rayleigh; its 95% quantile is $\sigma\sqrt{\chi^2_{2,0.95}}$ with
$\chi^2_{2,0.95} = 5.9915$, so

$$ \text{CI95} = \sigma\sqrt{5.9915} = 2.4477\,\sigma \quad\Longrightarrow\quad \sigma = \frac{\text{CI95}}{2.4477} \approx 0.4085\,\text{CI95} $$

The error is drawn in the local East–North frame by a **pluggable distribution**
$(\text{rng}, \text{CI95}) \mapsto (e_E, e_N)$ — isotropic Gaussian by default, with heavy-tail
mixtures and axis-aligned anisotropic variants beside it. The measured position is the
true position offset by that error, via our own geodesy:

$$ \beta = \operatorname{atan2}(e_E, e_N), \quad \rho = \sqrt{e_E^2 + e_N^2}, \quad (\varphi', \lambda') = \texttt{geo.forward}(\varphi, \lambda, \beta, \rho) $$

## Velocity error

Velocity error is per-axis Gaussian $N(0, \sigma_v^2)$ on the East–North components. Like
position, accuracy is quoted as a **95% radial CI** (parameter `vel_ci95`, m/s) and converted to
a per-axis σ by the same isotropic-2D formula as position: $\sigma_v = \text{vel\_ci95} /
2.4477$. The error is then applied and converted back to a measured track and ground speed:

$$ (v_E, v_N) = \big(v\sin\psi + \varepsilon_E,\; v\cos\psi + \varepsilon_N\big), \quad \psi' = \operatorname{atan2}(v_E, v_N), \quad v' = \sqrt{v_E^2 + v_N^2} $$

## Result

The measurement is a `Message(source, state=AircraftState(φ', λ', ψ', v'), t_meas=t)` —
timestamped for the communication layer (3b). `turn_rate` is not observed (set 0).

## Notes

- **Own detection uses the true own state** (own GPS error treated negligible); the GPS error
  matters for how *others* see this aircraft, via the broadcast.
- **Reproducible & isolated RNG — one shared `nav` substream, drawn in agent order.** Every
  aircraft's GPS draws come from the *same* per-encounter generator (`CnsStreams.nav`, singular;
  `cns/stack.py`), consumed in `firing` order. Independence between aircraft comes from position in
  one stream, not from separate streams — and that fixed order is exactly what makes `run_fleet` at
  n = 2 bit-for-bit equal to `run_encounter` (`tests/test_fleet.py`). Per-aircraft substreams would
  break that reduction *and* need a fifth child in a tree ADR 0006 §6 pins closed at four. The ADSL
  bug ADR 0001 forecloses is sharing a stream *between layers*, and that is foreclosed: `nav` and
  `comm` are separate children.
- With `pos_ci95` = 0 and `vel_ci95` = 0 the measurement equals the true state (a free regression
  to Phase 2).

## Why the noise model does not see heading

The distribution signature is `(rng, CI95)` and stays that way. Widening it to `(rng, CI95, ψ, g)`
was proposed (to express the paper's along-track *latency* models, and a track-oriented error
ellipse) and is **rejected** — see [[run-experiment-todo]] §9 for the closing argument. In short:

- **The latency displacement already exists, one layer down.** `LastKnown` is hold-as-is with no
  dead-reckoning, so a receiver acting at $t$ on a message measured at $t_\text{meas}$ is already
  looking at a position $t - t_\text{meas}$ seconds stale, and the source has already moved
  $(t - t_\text{meas})\,g$ along track since. Folding a $-\ell g$ bias into the *error* would apply
  that displacement a second time. The paper's lumped model is right for a simulator with no
  channel; this one has `LatencyDistribution` and hold-as-is surveillance, and they produce the
  displacement with the **correct distribution** — whatever $t - t_\text{meas}$ actually is under
  jitter, drops and broadcast cadence — rather than a fixed $\ell$.
- **The error ellipse is not oriented by track.** GPS position-error anisotropy comes from
  satellite geometry, not from where the vehicle happens to be pointing, so
  `make_anisotropic_gaussian` is axis-aligned (North the larger variance) and needs no ψ. This is
  the one place the code and the paper disagree, and it is a disagreement on the physics rather
  than an unimplemented feature.

A **static** bias needs no signature change either: it is a five-line `NoiseDistribution` closure
over the existing protocol. It belongs in a user's own file rather than beside `gaussian`, because
it breaks the containment guarantee every distribution in that module preserves — the 95th
percentile of the radial error stops equalling `CI95`. A **drifting** bias is the one case that
needs more than the protocol offers, and that is what the `NavEffect` seam is for
([[0021-navigation-extension-by-quality-effects]] §1).
