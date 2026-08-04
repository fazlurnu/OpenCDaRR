"""Robust MVP (RMVP) — MVP's rotation with a self-consistent **angular** margin.

Prototype resolver for the "design an MVP that is also robust against velocity uncertainty" study;
see ``robust-mvp/README.md`` for the derivation. Implements
:class:`~opencdarr.cr.base.ConflictResolver`, so it drops into the same
:class:`~opencdarr.separation.SeparationManager`, :func:`~opencdarr.loop.run_encounter` and
:mod:`opencdarr.ips` machinery as :class:`~opencdarr.cr.MVP` and :class:`~opencdarr.cr.VO`.

**The one fact this rule is built on.** The closest-approach offset

    d = |r x v| / |v| = |r| sin(alpha),      alpha = angle between v_rel and the line of sight

depends on the *direction* of the relative velocity and on nothing else about it. So the whole
question "will this manoeuvre clear the protected zone?" is a question about **one angle**, and the
uncertainty that matters is the uncertainty of that angle. Velocity noise enters it as

    sigma_alpha,v = sigma_v / |v_rel|

— an *angular* signal-to-noise ratio. At a 2 deg crossing between two 20 kt aircraft the true
relative speed is 0.36 m/s against a relative velocity sigma of 1.73 m/s at ``vel_ci95`` 3 m/s, so
the direction of ``v_rel`` is not merely noisy, it is unknown. That single number is the entire
reason a shallow crossing is fragile, and it is the number this resolver acts on.

**Why the angle domain and not the offset domain.** Propagating that angular noise into the offset
gives ``sigma_d = |r| cos(alpha) sigma_v / |v_rel|``, which is
``uncertainty-aware-mvp``'s ``sqrt(sigma_r^2 + t_cpa^2 sigma_v^2)`` written differently. At the 2
deg case it evaluates to 553 m against a range of 114.6 m — a spread the offset **cannot have**,
since ``d <= |r|`` always. A chance constraint written on ``d`` therefore asks for a miss distance
no velocity achieves, saturates, and silently degrades to a floor. Written on ``alpha`` the same
constraint is always feasible, because the angle is bounded: ``alpha = pi/2`` is fully
perpendicular and gives the largest offset the current range permits. **Nothing about the model
changes; only the coordinate the constraint is written in.**

**The rule.** Let ``gamma = asin(R / |r|)`` with ``R = margin * rpz`` (the angle at which
``d = R``; the velocity obstacle's half-angle, never constructed) and let ``theta`` be the rotation
applied to ``v_rel``. Require the post-manoeuvre angular margin to cover ``k = Phi^-1(confidence)``
standard deviations:

    alpha + theta - gamma  >=  k * sigma_m(theta),
    sigma_m(theta)^2 = (sigma_v cos(theta) / |v_rel|)^2 + (sigma_r / (|r| cos(gamma)))^2

and take the **smallest** ``theta >= 0`` that satisfies it. MVP is ``k = 0``, where the condition
collapses to ``theta = gamma - alpha`` — MVP's exact rotation.

**Why ``sigma_m`` depends on ``theta``, and why that is the whole mechanism.** MVP's step is
perpendicular to ``v_rel``, so it rotates the relative velocity *and lengthens it*:
``|v'| = |v_rel| / cos(theta)``. The angular noise of the post-manoeuvre relative velocity is
therefore ``sigma_v / |v'| = sigma_v cos(theta) / |v_rel|`` — **turning harder makes the outcome
better known, not just further away.** The requirement and the means of meeting it are coupled, so
the rule is a fixed point rather than a formula, solved by :func:`rotation` (safeguarded Newton on
a strictly monotone scalar, ~5 iterations, no construction and no search over velocity space).

**Four consequences, all checked in ``verify_rmvp.py``.**

1. ``confidence = 0.5`` gives ``k = 0`` and reproduces MVP to ~1e-12 rad — the same rotation, from
   the angle form of MVP's erratum-corrected gain.
2. ``sigma -> 0`` gives MVP at any confidence.
3. **The step stays bounded as the geometry degenerates.** As ``|v_rel| -> 0`` the rotation needed
   tends to ``pi/2`` but the step tends to ``k sigma_v / (pi/2 + alpha - gamma)``, an O(1) multiple
   of the declared velocity accuracy. A vanishing relative speed does not produce a vanishing or an
   infinite manoeuvre; it produces the manoeuvre that buys back the angular SNR the confidence asks
   for.
4. The post-manoeuvre relative speed obeys ``|v'| >= sigma_v * k / (alpha + theta - gamma)``, so a
   relative-speed floor is a **derived** consequence here rather than a chosen one
   (``finding-best-cr``'s CVP imposes such a floor by hand, keyed on the *perceived* relative
   speed, which the noise itself inflates).

**Head-on.** MVP's ``_BIAS_EPS`` floor is kept verbatim, for the reason ``finding-best-cr/cvp.py``
documents at length: ``r x v`` **is** ``d_miss |v|``, so taking the turn direction from its sign is
a catastrophic cancellation at a head-on and lets two cooperating aircraft turn the same way.
Flooring ``d_miss`` and taking the perpendicular to the line of sight is antisymmetric under
``(r, v) -> (-r, -v)`` by construction. Keeping MVP's tie-break is also what makes ``k = 0``
a controlled comparison rather than a merely similar algorithm.

**Multi-aircraft.** Like MVP this is a potential field: the resolve is the **sum** of the pairwise
avoidance vectors (ADR 0004). No cone, no union, no minimum-norm projection over a constructed set.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from statistics import NormalDist
from typing import Literal

from opencdarr.cns.noise_distributions import CI95_TO_SIGMA
from opencdarr.cr.base import ConflictResolver
from opencdarr.kinematics import MotionCommand
from opencdarr.relative import relative_enu, velocity_enu
from opencdarr.state import AircraftState

_BIAS_EPS = 0.1  # m: MVP's head-on floor, kept verbatim -- see the module docstring on why
_PARALLEL_EPS = 1e-9  # |v_rel|^2 below this -> no relative motion, nothing to resolve
_ROOT_TOL = 1e-12  # rad: residual at which :func:`rotation` stops
_ROOT_ITERS = 40  # hard cap; the safeguarded Newton below needs ~5, the bisection fallback ~40


def _clamp_unit(x: float) -> float:
    """``x`` clamped to [0, 1] — the argument of an ``asin`` that geodesy can push slightly out."""
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def relative_sigmas(
    own: AircraftState,
    intr: AircraftState,
    velocity_uncertainty: Literal["both", "intruder"] = "both",
) -> tuple[float, float]:
    """Per-axis 1-sigma of the relative position and relative velocity [m], [m/s].

    Built from the two aircraft's declared ``pos_ci95`` / ``vel_ci95`` with the same
    ``CI95_TO_SIGMA`` conversion :class:`~opencdarr.cns.GnssNavigation` and
    :class:`~opencdarr.crr.ProbabilisticFTR` use, so the resolver and the recovery criterion size
    the same uncertainty from the same two numbers.

    Position always sums both sides (independent errors, ``Var(A-B) = Var(A) + Var(B)``). Velocity
    follows ``velocity_uncertainty``, mirroring :class:`~opencdarr.crr.ProbabilisticFTR`'s knob of
    the same name; ``"both"`` is the reading this study runs, because the resolver reads two
    perceived velocities and neither of them is an intent.
    """
    sigma_r = math.hypot(own.pos_ci95, intr.pos_ci95) * CI95_TO_SIGMA
    if velocity_uncertainty == "intruder":
        sigma_v = intr.vel_ci95 * CI95_TO_SIGMA
    else:
        sigma_v = math.hypot(own.vel_ci95, intr.vel_ci95) * CI95_TO_SIGMA
    return sigma_r, sigma_v


def angular_sigma(theta: float, sigma_phi: float, sigma_los: float) -> float:
    """Standard deviation of the achieved angular margin [rad] after rotating ``v_rel`` by
    ``theta``.

    ``sqrt((sigma_phi cos(theta))^2 + sigma_los^2)`` — this resolver's whole uncertainty model in
    one line, and the only place ``theta`` feeds back into its own requirement.

    ``sigma_phi = sigma_v / |v_rel|`` is the *pre-manoeuvre* angular noise of the relative velocity
    direction; the ``cos(theta)`` is the manoeuvre paying it down, since a perpendicular step of
    rotation ``theta`` leaves ``|v'| = |v_rel| / cos(theta)``.

    ``sigma_los`` is the position side, ``sigma_r / (|r| cos(gamma))``: the line-of-sight direction
    is uncertain by ``sigma_r / |r|`` and the required angle ``gamma = asin(R/|r|)`` is itself
    uncertain by ``tan(gamma) sigma_r / |r|`` through the range. Those two read *orthogonal*
    components of an isotropic position error, so they are independent and add in quadrature to
    ``sigma_r / (|r| cos(gamma))``. It carries no ``theta``: a velocity change cannot buy down
    position uncertainty, which is exactly why this term sets the floor on what confidence is
    available at a given range.
    """
    return math.hypot(sigma_phi * math.cos(theta), sigma_los)


def rotation(
    alpha: float, gamma: float, k: float, sigma_phi: float, sigma_los: float
) -> float:
    """The smallest rotation ``theta >= 0`` [rad] whose angular margin covers ``k`` sigmas.

    Solves ``f(theta) = alpha + theta - gamma - k * sigma_m(theta) = 0`` for the residual
    :func:`angular_sigma` defines. ``f`` is **strictly increasing** with ``f' >= 1`` on
    ``[0, pi/2]``: ``d(sigma_m)/d(theta)`` is ``-sigma_phi^2 cos(theta) sin(theta) / sigma_m``,
    never positive there. So the root is unique and Newton cannot stall. The iteration below is
    Newton kept inside a bracket, falling back to bisection whenever a step would leave it
    — deterministic, and about five iterations (a median of 11 :func:`angular_sigma` evaluations
    including the two branch checks, 17 at worst) against the thirty a plain bisection needs.

    Two branches close the problem off at its ends:

    - ``f(0) >= 0`` — the pair is *already* robustly clear, so no manoeuvre. With ``k = 0`` this is
      MVP's "no conflict, no resolution".
    - ``f(theta_max) < 0`` at ``theta_max = pi/2 - alpha`` — the confidence is **not available at
      this geometry**, because a fully perpendicular relative velocity is the largest offset
      (``|r|``) the range permits. The rule then returns that best-available rotation, which is the
      paper's "maximise the distance at CPA" taken to its limit.

    ``theta_max`` is the *physical* cap and nothing tighter is imposed. Rotating past perpendicular
    reduces the offset again, so there is nothing beyond it worth asking for; an arbitrary clamp
    just short of ``pi/2`` — the obvious way to keep ``tan`` finite — is not equivalent, because it
    truncates the step by a constant factor wherever it binds (13% at this study's 2 deg geometry,
    ``|v_rel| cot(pi/2 - 1e-3)`` against ``|v_rel| cot(alpha)``). The effect is small and confined
    to the cap branch, which at that geometry needs ``|v_rel|`` below about 2.3e-3 m/s and never
    fires in the campaign — the reason to use the physical cap is that it is the one with a
    geometric meaning, not that the other was measurably wrong. At ``theta_max`` the step is
    ``|v_rel| cot(alpha)``, finite because ``_BIAS_EPS`` floors the miss distance and so bounds
    ``alpha`` away from zero.

    ``k = 0`` short-circuits to MVP's rotation ``max(0, gamma - alpha)`` exactly, so the comparison
    against MVP is controlled rather than approximate.
    """
    theta_max = 0.5 * math.pi - alpha
    if k <= 0.0:
        return min(max(gamma - alpha, 0.0), theta_max)

    def f(theta: float) -> float:
        return alpha + theta - gamma - k * angular_sigma(theta, sigma_phi, sigma_los)

    if theta_max <= 0.0 or f(0.0) >= 0.0:
        return 0.0
    if f(theta_max) < 0.0:
        return theta_max

    lo, hi = 0.0, theta_max  # f(lo) < 0 <= f(hi) is the invariant the safeguard maintains
    theta = 0.5 * (lo + hi)
    for _ in range(_ROOT_ITERS):
        value = f(theta)
        if abs(value) < _ROOT_TOL:
            return theta
        if value < 0.0:
            lo = theta
        else:
            hi = theta
        sigma_m = max(angular_sigma(theta, sigma_phi, sigma_los), 1e-300)  # zero sigma => slope 1
        slope = 1.0 + k * sigma_phi * sigma_phi * math.cos(theta) * math.sin(theta) / sigma_m
        step = theta - value / slope
        theta = step if lo < step < hi else 0.5 * (lo + hi)
    return theta


def _pairwise_dv(
    own: AircraftState,
    intr: AircraftState,
    rpz_eff: float,
    k: float,
    velocity_uncertainty: Literal["both", "intruder"],
) -> tuple[float, float]:
    """The velocity change ``dv`` to **subtract** from ``own``'s velocity to clear this one pair.

    ``(0, 0)`` when there is no relative motion. Isolated as a pairwise primitive because the
    N-aircraft resolve is the **sum** of these: like MVP, this is a potential-field method, so the
    pairwise avoidance vectors superpose (ADR 0004).

    The step is ``|v_rel| tan(theta)`` along ``c_hat``, the unit vector to the predicted
    closest-approach point. ``c`` is perpendicular to ``v_rel`` by construction (``c . v = r . v +
    |v|^2 t_cpa = 0``), so stepping along it is exactly the rotation :func:`rotation` sized, and
    subtracting it moves ``own`` away from that point rather than toward it.
    """
    rel = relative_enu(own, intr)
    rx, ry, vx, vy, dist = rel.rx, rel.ry, rel.vx, rel.vy, rel.dist
    v2 = vx * vx + vy * vy
    if v2 < _PARALLEL_EPS:
        return 0.0, 0.0  # no relative motion: this pair contributes nothing
    v_mag = math.sqrt(v2)

    t_cpa = -(rx * vx + ry * vy) / v2
    cx, cy = rx + vx * t_cpa, ry + vy * t_cpa  # relative position at CPA (own -> intr), c ⊥ v
    d_miss = math.hypot(cx, cy)
    if d_miss <= _BIAS_EPS:  # head-on: the CPA direction is ill-conditioned, pick a side
        d_miss = _BIAS_EPS
        cx, cy = ry / dist * d_miss, -rx / dist * d_miss

    alpha = math.asin(_clamp_unit(d_miss / dist))  # v_rel's angular offset from the line of sight
    gamma = math.asin(_clamp_unit(rpz_eff / dist))  # the offset that puts d = rpz_eff
    if k > 0.0:
        sigma_r, sigma_v = relative_sigmas(own, intr, velocity_uncertainty)
        sigma_phi = sigma_v / v_mag
        sigma_los = sigma_r / (dist * max(math.cos(gamma), 1e-12))
    else:
        sigma_phi = sigma_los = 0.0

    theta = rotation(alpha, gamma, k, sigma_phi, sigma_los)
    scale = v_mag * math.tan(theta) / d_miss
    return scale * cx, scale * cy  # dv points own -> intr; own steers away by subtracting it


class RMVP(ConflictResolver):
    """Robust Modified Voltage Potential — MVP's rotation, sized by an angular chance constraint.

    ``margin`` (>= 1) enlarges the deterministic resolution zone beyond ``rpz`` exactly as in
    :class:`~opencdarr.cr.MVP`; the uncertainty margin is added **on top** of it in the angle
    domain, so the two knobs stay separable — ``margin`` is a fixed buffer chosen once,
    ``confidence`` a buffer that grows with how badly the geometry is known.

    ``confidence`` is the probability the manoeuvre is designed to clear ``rpz`` with, under the
    aircraft's own declared uncertainty: ``k = Phi^-1(confidence)`` sigmas of the achieved angular
    margin. **0.5 recovers MVP** (``k = 0``), which is what makes this a controlled comparison
    rather than a different algorithm. 0.95 is the default, the usual one-sided engineering choice.

    ``velocity_uncertainty`` selects whose declared ``vel_ci95`` enters the relative-velocity
    sigma, mirroring :class:`~opencdarr.crr.ProbabilisticFTR`'s knob (see :func:`relative_sigmas`).

    ``preferred`` is ignored: like MVP, this steers away from the *current* velocity.
    """

    def __init__(
        self,
        margin: float = 1.0,
        confidence: float = 0.95,
        velocity_uncertainty: Literal["both", "intruder"] = "both",
    ) -> None:
        if margin < 1.0:
            raise ValueError(f"margin must be >= 1, got {margin}")
        if not 0.5 <= confidence < 1.0:
            raise ValueError(
                f"confidence must be in [0.5, 1); got {confidence}. Below 0.5 the constraint "
                "would aim *inside* rpz (k < 0), and 1.0 is unreachable (k -> inf)."
            )
        if velocity_uncertainty not in ("both", "intruder"):
            raise ValueError(
                f"velocity_uncertainty must be 'both' or 'intruder', got {velocity_uncertainty!r}"
            )
        self.margin = margin
        self.confidence = confidence
        self.velocity_uncertainty = velocity_uncertainty
        self.k = NormalDist().inv_cdf(confidence)  # sigmas of angular margin the rule asks for

    def __repr__(self) -> str:
        return (f"RMVP(margin={self.margin}, confidence={self.confidence}, "
                f"velocity_uncertainty={self.velocity_uncertainty!r})")

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
            d_e, d_n = _pairwise_dv(own, intr, rpz_eff, self.k, self.velocity_uncertainty)
            dv_e += d_e
            dv_n += d_n
        # own's new velocity = current − Σ dv (an empty/no-motion set holds the current velocity)
        return MotionCommand(target_velocity=(vox - dv_e, voy - dv_n))
