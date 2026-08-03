# Derivation — Probabilistic FTR recovery (2D)

Generalises [[ftr-recovery]]'s (`opencdarr.crr.FTR`) deterministic "would reverting to the
desired velocity clear the intruder?" into a probabilistic one, using the CNS uncertainty each
aircraft has already declared on its own state (`AircraftState.pos_ci95` / `vel_ci95`).
Re-derived from `CDaRR_git/sim_models/crr_resumenav_probabilistic_ftr.py`
(`resumenav_probabilistic_ftr`), named in `docs/roadmap.md`'s v0.5 benchmark set
(Past-CPA / FTR / Probabilistic-FTR).

- Implemented by: [`opencdarr/crr/probabilistic_ftr.py`](../../opencdarr/crr/probabilistic_ftr.py)
  (`ProbabilisticFTR`)
- Validated by: [`tests/test_probabilistic_ftr.py`](../../tests/test_probabilistic_ftr.py)

## The quantity: P(closest-approach offset > rpz)

Relative position $r \sim N(\mu_r, \Sigma_r)$ and relative velocity $v \sim N(\mu_v, \Sigma_v)$,
independent 2D Gaussians. The **unconstrained closest-approach offset**

$$ d = r - v\,\frac{r \cdot v}{v \cdot v} $$

is the perpendicular component of $r$ relative to the line through the origin with direction
$v$ — the same algebraic quantity [[cpa-detection]]'s $t_{cpa}$ formula computes the distance
*at*, except this does **not** gate on the sign of $t_{cpa}$: it is the perpendicular distance
from the origin to the *infinite* line $\{r + tv : t \in \mathbb{R}\}$, whether the closest point
falls in the future or the past. The criterion resumes once $P(\lVert d\rVert > \text{rpz})$
clears a threshold.

## Computing it: integrate over the uncertain direction of $v$

Condition on $v$'s direction $\theta$: $d$'s magnitude becomes the projection of the Gaussian
$r$ onto the perpendicular unit vector $u_\perp(\theta) = (-\sin\theta, \cos\theta)$ — itself
Gaussian, $N(m(\theta), s(\theta)^2)$ with $m = u_\perp \cdot \mu_r$,
$s^2 = u_\perp^\top \Sigma_r\, u_\perp$. The conditional tail is a folded-normal probability:

$$ P(\lVert d\rVert > x \mid \theta) = 1 - \left[\Phi\!\left(\frac{x-m}{s}\right) - \Phi\!\left(\frac{-x-m}{s}\right)\right] $$

Averaging over $\theta$ needs the **angular density of $v$'s direction** — a 2D Gaussian
projected onto the unit circle (the "projected normal" distribution). Its closed form is evaluated
in log-space (`_log_p_theta`) because the naive formula over/underflows at high velocity SNR
($\lvert\mu_v\rvert \gg \sqrt{\Sigma_v}$, the common case near a resolved conflict — the intruder's
mean velocity is well separated from zero relative to its uncertainty). The final integral,
discretised over `ktheta` angle samples (default 256, matching the reference):

$$ P(\lVert d\rVert > x) = \sum_{k} P(\lVert d\rVert > x \mid \theta_k)\; p_\Theta(\theta_k)\,\Delta\theta $$

**The uniform grid is the weak point of this formula, not the formula itself.** The same high-SNR
regime that forces the log-space evaluation also makes $p_\Theta$ a spike of width $\approx 1/$SNR,
which a grid of fixed spacing $2\pi/\texttt{ktheta}$ does not resolve — at `ktheta = 256` and
`vel_ci95 = 1` m/s only four nodes land inside it, and the sum becomes a one-point estimate.
[[probftr-angular-grid]] measures the error this causes and tests a grid centred on the mean
velocity direction instead. Not fixed here; `ktheta = 256` uniform is still what ships.

## Where $\Sigma_r$, $\Sigma_v$ come from

Uncertainty is **per-aircraft, per-quantity** (`pos_ci95`/`vel_ci95` on `AircraftState`), not one
flat pair of numbers from run config as in the reference. Both are converted to $\sigma$ by the
same `CI95_TO_SIGMA` [[gps-noise]] uses.

- **$\Sigma_r = \Sigma_o + \Sigma_i$**, always. Both `own` and `intr`'s *position* are noisy
  perceived quantities (own's own noisy self-measurement, `intr`'s noisy broadcast), so their
  declared `pos_ci95` add ($\mathrm{Var}(A-B) = \mathrm{Var}(A) + \mathrm{Var}(B)$ for independent
  errors).
- **$\Sigma_v$ — a knob, `velocity_uncertainty`**, the same value for both criteria:
  - `"both"` (default): $\Sigma_v = \Sigma_{v_o} + \Sigma_{v_i}$, the independent-errors sum, the
    symmetric counterpart of $\Sigma_r$.
  - `"intruder"`: $\Sigma_v = \Sigma_{v_i}$ alone. `own`'s side of *both* criteria is
    `own.desired` — its own declared intent, which carries no measurement error, since an aircraft
    does not misread the velocity it is about to fly. On this reading `own.vel_ci95` describes a
    perceived *current* velocity that neither criterion consults.

Neither reading is obviously wrong and they differ in how conservative recovery becomes, which is
why it is a parameter. Both criteria share one $\Sigma_v$: `intr.desired` is the intruder's
declared intent when shared, and otherwise the **onset velocity** — the velocity perceived when
the pair became active, substituted by `SeparationManager` (`separation.py`, `FleetMemory`). That
substituted value is a noisy observation, so it carries the intruder's velocity uncertainty just as
its current velocity does.

## Known, deliberate divergence from `FTR` at zero uncertainty

As $\Sigma_r,\Sigma_v \to 0$, `ProbabilisticFTR` does **not** always reduce to `FTR`'s answer,
because `FTR`'s `_clears` special-cases already-diverging geometry ($t_{cpa} \le 0$): it falls
back to the **current** separation rather than the algebraic (possibly-past) offset. This formula
is deliberately **unconstrained** — it never makes that substitution.

**The precise condition, and it is not a rare corner case:** whenever $r$ and $v$ are (anti)
parallel — a **radial** trajectory, moving directly toward or directly away along the line
connecting the two aircraft — the unconstrained line $\{r + tv\}$ passes *exactly* through the
origin, so the offset is exactly 0 regardless of which direction along that line the motion goes.
`FTR`'s own test suite has exactly this geometry: `test_reverting_would_still_clear_resumes`
(intruder 500 m dead ahead, on the *same* track, pulling away faster) — `FTR` says *clears*
(current separation is what matters, and it's comfortably beyond `rpz`); `ProbabilisticFTR` at
zero uncertainty says *does not clear* (the radial line has zero perpendicular offset, full stop,
independent of current distance or direction of travel). Verified directly in
`test_probabilistic_ftr.py` — not a bug, the faithful consequence of porting the reference's
actual formula rather than reconciling it with `FTR`'s. Off the radial special case, for
**converging, non-colinear** geometry ($t_{cpa} > 0$) the two agree at zero uncertainty, since
there both formulas measure the same future closest approach.

## Result

`ProbabilisticFTR.should_resume(own, intr, rpz)` resumes once both criteria's probability clear
`prob_threshold` (default 0.999) — `FTR`'s boolean AND, replaced by two probability thresholds.

## Notes

- Not ported: `analytical_past_cpa_prob` (a separate $P(t_{cpa}<0)$ delta-method approximation
  in the reference file) — defined there but never called by `resumenav_probabilistic_ftr`
  (`docs/lesson-learnt.md`: don't port unused code).
- No `scipy` dependency: `math.erf` vectorised with `np.vectorize`, the same fallback the
  reference itself uses when `scipy` is unavailable.
- Provenance of "Probabilistic-FTR" as a named benchmark: `docs/roadmap.md` v0.5, citing
  `my-observation.md` #14–16 (not present in this repo; the reference implementation is the
  available source of truth here).
