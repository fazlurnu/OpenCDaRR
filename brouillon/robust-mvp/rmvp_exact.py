"""RMVP with the **exact** angular quantile in place of the Gaussian one — §9.1, priced.

:class:`~rmvp.RMVP` sizes its rotation from a first-order Gaussian model of the achieved angular
margin, ``alpha + theta - gamma >= k sigma_m(theta)``. That model is what fails at a shallow
crossing: the true angular perturbation of a 2D Gaussian velocity is a **projected normal**, which
is near-Gaussian only at high signal-to-noise and near-uniform at low, and at Δψ = 2° with
``vel_ci95`` 3 m/s the pre-manoeuvre SNR is 0.21. The measured consequence is that RMVP delivers
0.848 where it was designed for 0.95.

This class replaces the quantile with the integral that has no such approximation. Given the
perceived state, the *true* post-manoeuvre geometry is

    r ~ N(r_perceived, Sigma_r),        v' ~ N(v'_intended(theta), Sigma_v)

and ``P(|closest-approach offset| > R)`` under exactly that pair is what
:func:`opencdarr.crr.probabilistic_ftr._p_offset_gt` computes — the same quadrature
:class:`~opencdarr.crr.ProbabilisticFTR` uses to decide when to resume. So the resolver and the
recovery criterion end up evaluating **one function**, one before the manoeuvre and one after.

The rule becomes: take the smallest ``theta >= 0`` with ``p(theta) >= confidence``, where

    p(theta) = P(|d| > margin * rpz),   v'_intended(theta) = v_perceived + |v| tan(theta) c_hat

Everything else — the perpendicular step along ``c_hat``, MVP's ``_BIAS_EPS`` tie-break, the sum
over intruders — is :class:`~rmvp.RMVP` unchanged, so the two differ in the quantile and in nothing
else.

**What this costs, and what it gives up.**

- **Speed.** Each evaluation of ``p`` is a 128-node quadrature (~65 us), against a ``hypot``
  for the Gaussian rule. The closed-form :func:`~rmvp.rotation` seeds the bracket, so the search
  needs about ten evaluations rather than thirty.
- **The MVP limit.** ``confidence = 0.5`` no longer recovers MVP. ``p`` is the *two-sided*
  ``P(|d| > R)``, and at MVP's own solution that is 0.6–0.9 rather than 0.5, so the ``k = 0``
  identity :class:`~rmvp.RMVP` is built around does not survive. That is a real loss: it is what
  made RMVP a controlled one-parameter change to MVP.
- **One-sided against two-sided.** :class:`~rmvp.RMVP` constrains the one-sided margin (clear
  on the side it is steering toward); this constrains ``|d| > R`` either side. Two-sided is the
  more generous of the two, and also the one matching the safety event actually measured — a loss
  of separation does not care which side the intruder passes.
- **Closed form.** Gone. The rule is now a quadrature inside a root find, which is why it lives
  here and not in ``rmvp.py``: the study's claim is about a *geometric* rule, and this is the
  measurement of what that claim costs.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from statistics import NormalDist
from typing import Literal

import numpy as np
from rmvp import relative_sigmas, rotation

from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.probabilistic_ftr import _p_offset_gt
from opencdarr.kinematics import MotionCommand
from opencdarr.relative import relative_enu, velocity_enu
from opencdarr.state import AircraftState

_BIAS_EPS = 0.1  # m: MVP's head-on floor, kept verbatim
_PARALLEL_EPS = 1e-9  # |v_rel|^2 below this -> no relative motion
_COV_EPS = 1e-9  # keeps a zero-uncertainty covariance invertible, as ProbabilisticFTR does
_DEFAULT_KTHETA = 128  # the "centred" grid's own default in ProbabilisticFTR


def _iso(sigma: float) -> np.ndarray:
    """Isotropic 2x2 covariance from a per-axis sigma, regularised to stay invertible."""
    return (sigma * sigma + _COV_EPS) * np.eye(2)


def _pairwise_dv(
    own: AircraftState,
    intr: AircraftState,
    rpz_eff: float,
    confidence: float,
    k: float,
    velocity_uncertainty: Literal["both", "intruder"],
    ktheta: int,
    tol: float,
) -> tuple[float, float]:
    """The velocity change ``dv`` to **subtract** from ``own``'s velocity to clear this one pair.

    Identical to :func:`rmvp._pairwise_dv` except for how ``theta`` is chosen: the Gaussian
    quantile is replaced by a bisection on the exact probability, bracketed using the closed-form
    solution as a hint.
    """
    rel = relative_enu(own, intr)
    rx, ry, vx, vy, dist = rel.rx, rel.ry, rel.vx, rel.vy, rel.dist
    v2 = vx * vx + vy * vy
    if v2 < _PARALLEL_EPS:
        return 0.0, 0.0
    v_mag = math.sqrt(v2)

    t_cpa = -(rx * vx + ry * vy) / v2
    cx, cy = rx + vx * t_cpa, ry + vy * t_cpa  # c ⊥ v, points own -> intr at CPA
    d_miss = math.hypot(cx, cy)
    if d_miss <= _BIAS_EPS:  # head-on: the CPA direction is ill-conditioned, pick a side
        d_miss = _BIAS_EPS
        cx, cy = ry / dist * d_miss, -rx / dist * d_miss
    ce, cn = cx / d_miss, cy / d_miss

    sigma_r, sigma_v = relative_sigmas(own, intr, velocity_uncertainty)
    mu_r, cov_r, cov_v = np.array([rx, ry]), _iso(sigma_r), _iso(sigma_v)

    def p(theta: float) -> float:
        """P(|achieved offset| > rpz_eff) if the relative velocity is rotated by ``theta``."""
        a = v_mag * math.tan(theta)
        mu_v = np.array([vx + a * ce, vy + a * cn])
        return _p_offset_gt(rpz_eff, mu_r, cov_r, mu_v, cov_v, ktheta)

    alpha = math.asin(min(d_miss / dist, 1.0))
    theta_max = 0.5 * math.pi - alpha
    if theta_max <= 0.0 or p(0.0) >= confidence:
        return 0.0, 0.0  # already clear at the required confidence
    if p(theta_max) < confidence:
        theta = theta_max  # unattainable here: take the max-offset rotation, as RMVP does
    else:
        # The closed-form answer is a good guess, so spend the first evaluation halving the
        # bracket with it rather than starting from the full [0, theta_max].
        gamma = math.asin(min(rpz_eff / dist, 1.0))
        seed = min(max(rotation(alpha, gamma, k, sigma_v / v_mag,
                                sigma_r / (dist * max(math.cos(gamma), 1e-12))), 0.0), theta_max)
        lo, hi = (0.0, seed) if p(seed) >= confidence else (seed, theta_max)
        while hi - lo > tol:
            mid = 0.5 * (lo + hi)
            if p(mid) >= confidence:
                hi = mid
            else:
                lo = mid
        theta = hi  # the feasible end, so the constraint is met rather than nearly met

    scale = v_mag * math.tan(theta) / d_miss
    return scale * cx, scale * cy


class RMVPExact(ConflictResolver):
    """RMVP with the projected-normal quantile in place of the Gaussian one.

    ``margin``, ``confidence`` and ``velocity_uncertainty`` mean what they do on
    :class:`~rmvp.RMVP`. ``ktheta`` is the quadrature grid handed to
    :func:`~opencdarr.crr.probabilistic_ftr._p_offset_gt`; ``tol`` [rad] is the rotation the
    bisection resolves to, 1e-3 rad being 0.06° and far below anything a resolution command needs.

    ``preferred`` is ignored: like MVP, this steers away from the *current* velocity.
    """

    def __init__(
        self,
        margin: float = 1.0,
        confidence: float = 0.95,
        velocity_uncertainty: Literal["both", "intruder"] = "both",
        ktheta: int = _DEFAULT_KTHETA,
        tol: float = 1e-3,
    ) -> None:
        if margin < 1.0:
            raise ValueError(f"margin must be >= 1, got {margin}")
        if not 0.0 < confidence < 1.0:
            raise ValueError(f"confidence must be in (0, 1); got {confidence}")
        self.margin = margin
        self.confidence = confidence
        self.velocity_uncertainty = velocity_uncertainty
        self.ktheta = ktheta
        self.tol = tol
        self.k = NormalDist().inv_cdf(confidence)  # only for the closed-form bracket hint

    def __repr__(self) -> str:
        return (f"RMVPExact(margin={self.margin}, confidence={self.confidence}, "
                f"velocity_uncertainty={self.velocity_uncertainty!r}, ktheta={self.ktheta})")

    def resolve(
        self,
        own: AircraftState,
        intruders: Sequence[AircraftState],
        rpz: float,
        preferred: tuple[float, float] | None = None,
    ) -> MotionCommand:
        rpz_eff = rpz * self.margin
        vox, voy = velocity_enu(own)
        dv_e = dv_n = 0.0
        for intr in intruders:
            d_e, d_n = _pairwise_dv(own, intr, rpz_eff, self.confidence, self.k,
                                    self.velocity_uncertainty, self.ktheta, self.tol)
            dv_e += d_e
            dv_n += d_n
        return MotionCommand(target_velocity=(vox - dv_e, voy - dv_n))
