	# Derivation — GPS self-measurement noise (navigation, 2D)

How an aircraft measures its **own** state (position + velocity) with error, to broadcast. This
is the **N** of CNS: the error is at the source, applied once; others perceive it via the
broadcast (Phase 3a). Re-derived from `sim_models/noise_model.py` /
`noise_distributions.py`, 2D.

- Implemented by: [`opencdarr/cns/navigation.py`](../../opencdarr/cns/navigation.py) (`GpsNavigation`)
- Distributions: [`opencdarr/cns/noise_distributions.py`](../../opencdarr/cns/noise_distributions.py)
- Validated by: [`tests/test_cns_navigation.py`](../../tests/test_cns_navigation.py)

## Where CI95 lives

`pos_ci95` / `vel_ci95` are **fields on `AircraftState`** (its own declared measurement
accuracy), not constructor parameters of `GpsNavigation` — the same reasoning as `turn_rate`:
accuracy is a property of *that aircraft's* sensor, can differ between aircraft, and may evolve
over a run (e.g. degrading GPS coverage), so it must travel with the clonable state, not sit on a
shared, fixed navigation object. `GpsNavigation.measure(true, t, rng)` reads `true.pos_ci95` /
`true.vel_ci95` to size the noise, and **copies the same values onto the broadcast state** — a
receiver gets the sender's declared accuracy *with* the message, as ordinary state, with no
separate channel. Default `0.0` on both fields means a perfect sensor (no noise), the same
"neutral default" convention as `turn_rate = 0.0`.

## Position error — CI95 to σ

Position error is a zero-mean 2D isotropic Gaussian, each axis $N(0, \sigma^2)$. GPS/ADS-B
accuracy is quoted as a **95% radial CI** (the radius containing 95% of fixes). The radial
distance is Rayleigh; its 95% quantile is $\sigma\sqrt{\chi^2_{2,0.95}}$ with
$\chi^2_{2,0.95} = 5.9915$, so

$$ \text{CI95} = \sigma\sqrt{5.9915} = 2.4477\,\sigma \quad\Longrightarrow\quad \sigma = \frac{\text{CI95}}{2.4477} \approx 0.4085\,\text{CI95} $$

The error is drawn in the local East–North frame by a **pluggable distribution**
$(\text{rng}, \text{CI95}, \psi) \mapsto (e_E, e_N)$ — isotropic Gaussian for now; anisotropic
(along/cross-track, using $\psi$) and heavy-tail mixtures are 3c. The measured position is the
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
- **Reproducible & isolated RNG:** each aircraft's GPS draws come from its own substream
  (ADR 0001 / 0005) — the old ADSL shared-RNG bug cannot recur.
- With `pos_ci95` = 0 and `vel_ci95` = 0 the measurement equals the true state (a free regression
  to Phase 2).
