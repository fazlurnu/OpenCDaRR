"""Pluggable position-error distributions for navigation (GPS) noise.

Each matches :class:`~opencdarr.cns.base.NoiseDistribution`: a callable
``(rng, ci95) -> (east, north)`` returning one 2D measurement error [m]. Besides
the isotropic :func:`gaussian`, three factories build calibrated alternatives —
a heavy-tail mixture (:func:`make_mixture_gaussian`), an anisotropic Gaussian
(:func:`make_anisotropic_gaussian`), and their combination
(:func:`make_anisotropic_mixture_gaussian`). Ported from CDaRR's
``sim_models/noise_distributions.py``.

Every distribution here preserves the same containment guarantee: the 95th
percentile of the 2D radial error equals ``ci95``. The calibrating scale is
solved once per ``ci95`` by bisection and cached in the factory's closure, so the
per-sample calls the navigation layer makes stay cheap.

Every distribution here also draws a **constant** number of times, whatever
``ci95`` is -- including zero, where the error is exactly ``(0.0, 0.0)`` but the
draws still happen. ``pos_ci95 = Sweep([0, 10, 20, 40])`` would otherwise run its
first cell on a different stream from the rest, so the cells would stop being
comparable (ADR 0006 §6 -- the same rule
:meth:`~opencdarr.cns.base.LinkGate.evolve` states for the channel, and the one
``sample_pairwise``'s pinned slots exist to keep). Sigma only scales the output,
so drawing unconditionally costs nothing at ``ci95 = 0`` and leaves the generator
in the same place it would reach for any other value.

The anisotropic distributions are **axis-aligned** — the larger-variance axis is
North, the smaller East. GPS position-error anisotropy comes from satellite
geometry, not the vehicle's heading, so the error ellipse is not oriented by
track (see ``vault/derivations/gps-noise.md``).
"""

from __future__ import annotations

import math

import numpy as np

# 95% radial CI -> per-axis 1-sigma for a 2D isotropic Gaussian: sigma = CI95 / sqrt(chi2_2,0.95).
# Shared by position and velocity error: both are per-axis-Gaussian, isotropic 2D quantities
# (vault/derivations/gps-noise.md).
CI95_TO_SIGMA = 1.0 / math.sqrt(5.991464547)  # ~= 0.4085


def gaussian(rng: np.random.Generator, ci95: float) -> tuple[float, float]:
    """Zero-mean isotropic 2D Gaussian position error [m] as (East, North)."""
    sigma = ci95 * CI95_TO_SIGMA
    return float(rng.normal(0.0, sigma)), float(rng.normal(0.0, sigma))


def make_mixture_gaussian(tail_ratio: float = 3.0, tail_weight: float = 0.1):
    """Two-component zero-mean isotropic Gaussian mixture with a preserved 2D radial ci95.

    With probability ``(1 - tail_weight)``: draw from N(0, sigma1^2 I).
    With probability ``tail_weight``:       draw from N(0, sigma2^2 I),
    sigma2 = tail_ratio * sigma1 (a wider tail component).

    sigma1 is solved by bisection so the 95th percentile of the 2D radial
    distance equals ``ci95`` exactly, preserving the same containment guarantee
    as :func:`gaussian`.

    Constraint solved:  p*exp(-u) + (1-p)*exp(-u/k^2) = 0.05
    with u = ci95^2 / (2 sigma1^2), k = tail_ratio, p = 1 - tail_weight.
    """
    if not 0.0 < tail_weight < 1.0:
        raise ValueError(f"tail_weight must be in (0, 1), got {tail_weight}")
    if tail_ratio <= 1.0:
        raise ValueError(f"tail_ratio must be > 1, got {tail_ratio}")

    p = 1.0 - tail_weight
    k = float(tail_ratio)
    _cache: dict[float, float] = {}

    def _sigma1(ci95_val: float) -> float:
        key = round(ci95_val, 8)
        if key in _cache:
            return _cache[key]
        lo, hi = ci95_val * 1e-5, ci95_val
        for _ in range(64):
            mid = 0.5 * (lo + hi)
            u = ci95_val ** 2 / (2.0 * mid ** 2)
            val = p * np.exp(-u) + (1.0 - p) * np.exp(-u / k ** 2)
            if val < 0.05:
                lo = mid
            else:
                hi = mid
        _cache[key] = 0.5 * (lo + hi)
        return _cache[key]

    def mixture_gaussian(rng: np.random.Generator, ci95: float) -> tuple[float, float]:
        # The bisection is only defined for a positive ci95, but the draws happen
        # **unconditionally** -- see this module's docstring on the constant draw count.
        s1 = _sigma1(float(ci95)) if ci95 > 0.0 else 0.0
        sigma = s1 * k if rng.random() < tail_weight else s1
        return float(rng.normal(0.0, sigma)), float(rng.normal(0.0, sigma))

    return mixture_gaussian


def _radial_cdf(r: float, sigma_along: float, sigma_cross: float, n_grid: int = 4001) -> float:
    """P(sqrt(X^2 + Y^2) <= r) for independent X ~ N(0, sigma_along^2),
    Y ~ N(0, sigma_cross^2). Computed by numerical integration (no closed
    form exists for sigma_along != sigma_cross)."""
    if sigma_along == sigma_cross:
        return 1.0 - math.exp(-r ** 2 / (2.0 * sigma_along ** 2))
    x = np.linspace(-r, r, n_grid)
    fx = np.exp(-0.5 * (x / sigma_along) ** 2) / (sigma_along * math.sqrt(2.0 * math.pi))
    y_bound = np.sqrt(np.maximum(r ** 2 - x ** 2, 0.0))
    z = y_bound / (sigma_cross * math.sqrt(2.0))
    erf_z = np.array([math.erf(v) for v in z])
    integrand = fx * erf_z  # 2*Phi(z*sqrt2) - 1 == erf(z)
    return float(np.trapz(integrand, x))


def make_anisotropic_gaussian(var_ratio: float = 3.0):
    """Anisotropic (axis-aligned) 2D Gaussian position error: the North-axis
    variance is ``var_ratio`` times the East-axis variance, while the overall
    95% radial containment still matches ``ci95`` (same guarantee as
    :func:`gaussian` / :func:`make_mixture_gaussian`).

    ``sigma_cross`` (East) is solved by bisection (via :func:`_radial_cdf`) so the
    95th percentile of the 2D radial distance equals ``ci95`` exactly.
    """
    if var_ratio <= 1.0:
        raise ValueError(f"var_ratio must be > 1, got {var_ratio}")

    std_ratio = math.sqrt(var_ratio)
    _cache: dict[float, float] = {}

    def _sigma_cross(ci95_val: float) -> float:
        key = round(ci95_val, 8)
        if key in _cache:
            return _cache[key]
        lo, hi = ci95_val * 1e-5, ci95_val
        for _ in range(64):
            mid = 0.5 * (lo + hi)
            val = _radial_cdf(ci95_val, std_ratio * mid, mid)
            # CDF decreases as sigma (mid) grows, so shrink the interval the
            # opposite way from the tail-probability bisection in
            # make_mixture_gaussian.
            if val < 0.95:
                hi = mid
            else:
                lo = mid
        _cache[key] = 0.5 * (lo + hi)
        return _cache[key]

    def anisotropic_gaussian(rng: np.random.Generator, ci95: float) -> tuple[float, float]:
        # Draws are unconditional -- see this module's docstring on the constant draw count.
        sigma_cross = _sigma_cross(float(ci95)) if ci95 > 0.0 else 0.0
        sigma_along = std_ratio * sigma_cross
        east = float(rng.normal(0.0, sigma_cross))
        north = float(rng.normal(0.0, sigma_along))
        return east, north

    return anisotropic_gaussian


def make_anisotropic_mixture_gaussian(
    var_ratio: float = 3.0, tail_ratio: float = 3.0, tail_weight: float = 0.1
):
    """Combines :func:`make_anisotropic_gaussian` and :func:`make_mixture_gaussian`:
    a two-component Gaussian mixture where every component has the same
    North-/East-axis variance ratio ``var_ratio``, and the tail component's axes
    are both scaled up by ``tail_ratio`` relative to the core component (same
    shape, larger spread), drawn with probability ``tail_weight``.

    ``sigma_cross`` (of the core component) is solved by bisection so the overall
    radial 95th percentile still equals ``ci95`` -- same containment guarantee as
    the other distributions in this module.
    """
    if var_ratio <= 1.0:
        raise ValueError(f"var_ratio must be > 1, got {var_ratio}")
    if not 0.0 < tail_weight < 1.0:
        raise ValueError(f"tail_weight must be in (0, 1), got {tail_weight}")
    if tail_ratio <= 1.0:
        raise ValueError(f"tail_ratio must be > 1, got {tail_ratio}")

    std_ratio = math.sqrt(var_ratio)
    p = 1.0 - tail_weight
    k = float(tail_ratio)
    _cache: dict[float, float] = {}

    def _sigma_cross(ci95_val: float) -> float:
        key = round(ci95_val, 8)
        if key in _cache:
            return _cache[key]
        lo, hi = ci95_val * 1e-5, ci95_val
        for _ in range(64):
            mid = 0.5 * (lo + hi)
            val = (p * _radial_cdf(ci95_val, std_ratio * mid, mid)
                   + (1.0 - p) * _radial_cdf(ci95_val, std_ratio * k * mid, k * mid))
            if val < 0.95:
                hi = mid
            else:
                lo = mid
        _cache[key] = 0.5 * (lo + hi)
        return _cache[key]

    def anisotropic_mixture_gaussian(
        rng: np.random.Generator, ci95: float
    ) -> tuple[float, float]:
        # Draws are unconditional -- see this module's docstring on the constant draw count.
        s1 = _sigma_cross(float(ci95)) if ci95 > 0.0 else 0.0
        sigma_cross = s1 * k if rng.random() < tail_weight else s1
        sigma_along = std_ratio * sigma_cross
        east = float(rng.normal(0.0, sigma_cross))
        north = float(rng.normal(0.0, sigma_along))
        return east, north

    return anisotropic_mixture_gaussian
