"""Probabilistic Fly-The-Route (Probabilistic FTR) recovery — uncertainty-aware (2D, directed).

Implements :class:`~opencdarr.crr.base.RecoveryCriterion`. Governing equations:
``vault/derivations/probabilistic-ftr-recovery.md``.

Generalises :class:`~opencdarr.crr.FTR`'s deterministic "would reverting to the desired velocity
keep the closest-approach offset beyond ``rpz``?" into a probabilistic one: given the CNS
uncertainty each aircraft has already declared on its own state (``pos_ci95``/``vel_ci95``,
``opencdarr.state.AircraftState``), what is P(closest-approach offset > rpz)? Resume only once
that probability clears ``prob_threshold`` for both FTR criteria.

Whether the ownship's own declared ``vel_ci95`` belongs in the relative-velocity covariance is a
modelling choice, not a fact — see ``velocity_uncertainty`` on :class:`ProbabilisticFTR`.

Re-derived from ``CDaRR_git/sim_models/crr_resumenav_probabilistic_ftr.py``
(``resumenav_probabilistic_ftr``); only the criterion actually exercised there is ported — the
file also defines ``analytical_past_cpa_prob`` (a separate P(t_cpa<0) delta-method
approximation) but never calls it, so it isn't ported (``docs/lesson-learnt.md``: don't port
unused code).
"""

from __future__ import annotations

import math
from typing import Literal

import numpy as np

from opencdarr.cns.noise_distributions import CI95_TO_SIGMA
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.relative import relative_enu, velocity_enu
from opencdarr.state import AircraftState

_SQRT2PI = math.sqrt(2.0 * math.pi)
_EPS = 1e-9  # covariance regularisation: keeps Sigma invertible at zero declared uncertainty
_erf = np.vectorize(math.erf)  # no scipy dependency; same fallback the reference uses


def _phi(x: np.ndarray) -> np.ndarray:
    """Standard normal CDF."""
    return 0.5 * (1.0 + _erf(x / math.sqrt(2.0)))


def _iso_cov(ci95: float) -> np.ndarray:
    """Isotropic 2D covariance from a declared 95%-radial CI, regularised to stay invertible.

    Uses the same CI95 -> per-axis-sigma conversion as :class:`~opencdarr.cns.GnssNavigation`
    (``vault/derivations/gps-noise.md``), so "zero declared uncertainty" degenerates to a tiny,
    numerically-safe covariance rather than an exact (singular) zero.
    """
    sigma2 = (ci95 * CI95_TO_SIGMA) ** 2
    return (sigma2 + _EPS) * np.eye(2)


def _log_p_theta(theta: np.ndarray, mu: np.ndarray, sigma: np.ndarray) -> np.ndarray:
    """Log-density of the *direction* of a 2D Gaussian vector ``N(mu, sigma)`` — the "projected
    normal" distribution on the circle.

    Evaluated in log-space throughout: at high velocity SNR (``|mu| >> sqrt(sigma)``) the naive
    formula multiplies a huge coefficient by an underflowing exponential (or the reverse), which
    over/underflows in float64. Re-derived from the reference's numerically-stable form.
    """
    q = np.linalg.inv(sigma)
    c = float(mu @ q @ mu)
    u = np.stack([np.cos(theta), np.sin(theta)], axis=0)  # (2, K): unit vectors at each angle
    qu = q @ u
    a = np.maximum(np.sum(u * qu, axis=0), 1e-15)  # (K,)
    b = u.T @ (q @ mu)  # (K,)
    z = b / np.sqrt(a)

    log_term1 = -np.log(a)
    log_phi_z = np.log(np.maximum(_phi(z), 1e-300))
    log_term2_abs = (
        np.log(np.maximum(np.abs(b), 1e-300)) + math.log(_SQRT2PI)
        - 1.5 * np.log(a) + 0.5 * z * z + log_phi_z
    )
    log_term = np.where(
        b >= 0,
        np.logaddexp(log_term1, log_term2_abs),
        log_term1 + np.log(np.maximum(
            1.0 - np.exp(np.minimum(log_term2_abs - log_term1, 500.0)), 1e-300
        )),
    )
    log_const = -math.log(2.0 * math.pi) - 0.5 * math.log(float(np.linalg.det(sigma)))
    return log_const - 0.5 * c + log_term


def _p_offset_gt(
    x: float, mu_r: np.ndarray, sigma_r: np.ndarray, mu_v: np.ndarray, sigma_v: np.ndarray,
    ktheta: int,
) -> float:
    """P(closest-approach offset magnitude > x), integrating over the uncertain direction of v.

    The (unconstrained, sign-of-t_cpa-agnostic) closest-approach offset is
    ``d = r - v(r.v)/(v.v)``. Conditional on v's direction theta, ``d``'s magnitude equals the
    projection of ``r`` onto the perpendicular ``u_perp(theta)`` — a linear map of the Gaussian
    ``r``, hence itself Gaussian. Average that conditional tail probability over theta, weighted
    by the angular density of v's own direction under its uncertainty (:func:`_log_p_theta`).
    """
    if x < 0.0:
        return 1.0
    theta = np.linspace(0.0, 2.0 * math.pi, ktheta, endpoint=False)
    log_pth = _log_p_theta(theta, mu_v, sigma_v)
    log_pth = log_pth - np.max(log_pth)
    weights = np.exp(log_pth)
    weights = weights / weights.sum()

    u_perp = np.stack([-np.sin(theta), np.cos(theta)], axis=0)  # (2, K)
    m = u_perp.T @ mu_r  # (K,)
    s = np.sqrt(np.maximum(np.sum(u_perp * (sigma_r @ u_perp), axis=0), 1e-15))
    tail = 1.0 - np.clip(_phi((x - m) / s) - _phi((-x - m) / s), 0.0, 1.0)
    return float(np.clip(np.sum(tail * weights), 0.0, 1.0))


class ProbabilisticFTR(RecoveryCriterion):
    """Resume once P(closest-approach offset > rpz) clears ``prob_threshold`` for both FTR
    criteria.

    Both criteria compare the *intruder's* velocity (current, then desired) against ``own``'s
    **desired** velocity, never ``own``'s own noisy current one — same as
    :class:`~opencdarr.crr.FTR`. The intruder's "desired" is its declared intent when
    intent-sharing is on, and otherwise the velocity perceived when the pair became active, which
    :class:`~opencdarr.separation.SeparationManager` substitutes in — so criterion 2 stays live
    either way, and matches the reference formulation's "intruder reverts to its velocity at
    conflict initiation".

    ``velocity_uncertainty`` selects which declared ``vel_ci95`` feed the relative-velocity
    covariance:

    - ``"both"`` (default) — ``Sigma_Vrel = Sigma_Vo + Sigma_Vi``, the independent-errors sum used
      for the relative *position* covariance and stated for the relative velocity in the
      uncertainty model.
    - ``"intruder"`` — ``Sigma_Vrel = Sigma_Vi`` only. The argument for it: ``own``'s side of every
      criterion is ``own.desired``, its own declared intent, which carries no measurement error —
      an aircraft does not misread the velocity it is *about* to fly. Under this reading
      ``Sigma_Vo`` describes ``own``'s perceived *current* velocity, which neither criterion uses.

    Both readings are defensible and they disagree in how conservative recovery becomes, so the
    choice is a knob rather than a hard-coded assumption. ``Sigma_rel`` (position) always sums both
    sides — there both states genuinely are noisy measurements.

    ``prob_threshold`` (default 0.999) is the confidence each criterion must clear; ``ktheta``
    (angular integration resolution, default 256) is the quadrature grid size.
    """

    def __init__(
        self,
        prob_threshold: float = 0.999,
        ktheta: int = 256,
        velocity_uncertainty: Literal["both", "intruder"] = "both",
    ) -> None:
        if velocity_uncertainty not in ("both", "intruder"):
            raise ValueError(
                f"velocity_uncertainty must be 'both' or 'intruder', got {velocity_uncertainty!r}"
            )
        self.prob_threshold = prob_threshold
        self.ktheta = ktheta
        self.velocity_uncertainty = velocity_uncertainty

    def _sigma_v(self, own: AircraftState, intr: AircraftState) -> np.ndarray:
        """Relative-velocity covariance: ``Sigma_Vo + Sigma_Vi``, or the intruder's alone."""
        if self.velocity_uncertainty == "intruder":
            return _iso_cov(intr.vel_ci95)
        return _iso_cov(own.vel_ci95) + _iso_cov(intr.vel_ci95)

    def should_resume(self, own: AircraftState, intr: AircraftState, rpz: float) -> bool:
        if own.desired is None:
            raise ValueError(
                "ProbabilisticFTR needs the ownship's desired (nominal) velocity; "
                "run_encounter sets it, or set AircraftState.desired explicitly."
            )
        rel = relative_enu(own, intr)  # rx,ry = intr − own position
        mu_r = np.array([rel.rx, rel.ry])
        sigma_r = _iso_cov(own.pos_ci95) + _iso_cov(intr.pos_ci95)

        vo_e, vo_n = own.desired.v_east, own.desired.v_north  # desired velocity, read directly
        sigma_v = self._sigma_v(own, intr)  # same relative-velocity covariance for both criteria

        # criterion 1: the intruder holds its current (observed, noisy) velocity
        vi_e, vi_n = velocity_enu(intr)
        mu_v1 = np.array([vi_e - vo_e, vi_n - vo_n])
        p1 = _p_offset_gt(rpz, mu_r, sigma_r, mu_v1, sigma_v, self.ktheta)
        if p1 <= self.prob_threshold:
            return False

        # criterion 2 (intent-based): the intruder reverts to its own desired velocity too — its
        # declared intent, or the onset velocity SeparationManager infers when intent isn't shared.
        if intr.desired is not None:
            vir_e, vir_n = intr.desired.v_east, intr.desired.v_north
            mu_v2 = np.array([vir_e - vo_e, vir_n - vo_n])
            p2 = _p_offset_gt(rpz, mu_r, sigma_r, mu_v2, sigma_v, self.ktheta)
            if p2 <= self.prob_threshold:
                return False

        return True
