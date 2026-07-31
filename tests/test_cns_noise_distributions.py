"""Calibration tests for the pluggable position-error distributions.

Every distribution preserves one contract: the 95th percentile of the 2D radial
error equals ``ci95``. Beyond that, the mixture is heavier-tailed than a plain
Gaussian and the anisotropic ones have a larger North (along) than East (cross)
spread.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from opencdarr.cns import (
    GnssNavigation,
    gaussian,
    make_anisotropic_gaussian,
    make_anisotropic_mixture_gaussian,
    make_mixture_gaussian,
)
from opencdarr.cns.noise_distributions import _radial_cdf, _trapezoid
from opencdarr.state import AircraftState

_CI95 = 20.0
_N = 8000


def _draw(dist, ci95: float = _CI95, n: int = _N, seed: int = 0) -> np.ndarray:
    """``n`` (east, north) samples from ``dist`` as an ``(n, 2)`` array."""
    rng = np.random.default_rng(seed)
    return np.array([dist(rng, ci95) for _ in range(n)])


@pytest.mark.parametrize(
    "dist",
    [
        gaussian,
        make_mixture_gaussian(),
        make_anisotropic_gaussian(),
        make_anisotropic_mixture_gaussian(),
    ],
)
def test_zero_mean_and_radial_ci95_calibrated(dist) -> None:
    """Shared contract: zero-mean per axis, radial 95th percentile == ci95."""
    s = _draw(dist)
    assert abs(s[:, 0].mean()) < 1.0
    assert abs(s[:, 1].mean()) < 1.0
    radial = np.hypot(s[:, 0], s[:, 1])
    assert abs(float(np.quantile(radial, 0.95)) - _CI95) < 1.5


def test_mixture_is_heavier_tailed_than_gaussian() -> None:
    """The tail component lifts the radial excess kurtosis above the Gaussian's."""
    g = np.hypot(*_draw(gaussian).T)
    m = np.hypot(*_draw(make_mixture_gaussian()).T)

    def excess_kurtosis(x: np.ndarray) -> float:
        z = (x - x.mean()) / x.std()
        return float((z ** 4).mean() - 3.0)

    assert excess_kurtosis(m) > excess_kurtosis(g) + 0.5


@pytest.mark.parametrize(
    "dist, var_ratio",
    [
        (make_anisotropic_gaussian(var_ratio=3.0), 3.0),
        (make_anisotropic_mixture_gaussian(var_ratio=4.0), 4.0),
    ],
)
def test_anisotropic_north_wider_than_east(dist, var_ratio: float) -> None:
    """North (along) std ~= sqrt(var_ratio) * East (cross) std."""
    s = _draw(dist)
    east_std, north_std = s[:, 0].std(), s[:, 1].std()
    assert north_std / east_std == pytest.approx(math.sqrt(var_ratio), rel=0.1)


def test_reproducible_per_seed() -> None:
    dist = make_mixture_gaussian()
    assert _draw(dist, seed=42).tolist() == _draw(dist, seed=42).tolist()


def test_plugs_into_navigation_end_to_end() -> None:
    """Each distribution satisfies the NoiseDistribution protocol GnssNavigation calls."""
    true = AircraftState(id="A", lat=52.0, lon=4.0, trk=30.0, gs=10.0, pos_ci95=20.0, vel_ci95=2.0)
    for dist in (make_mixture_gaussian(), make_anisotropic_gaussian(),
                 make_anisotropic_mixture_gaussian()):
        nav = GnssNavigation(pos_distribution=dist, vel_distribution=dist)
        msg = nav.measure(nav.initial_state(), true, t=1.0, rng=np.random.default_rng(0))
        assert msg.source == "A"
        assert math.isfinite(msg.state.lat) and math.isfinite(msg.state.lon)


def test_trapezoid_matches_a_closed_form() -> None:
    """The integrator is ours now (``np.trapz`` was removed in NumPy 2, ``np.trapezoid`` is absent
    on 1.x), so it needs a check against a number from *outside* the code.

    Half the unit disc: ``int_-1^1 sqrt(1 - x^2) dx == pi / 2``.

    This test passes on either NumPy because it no longer touches either name — but that is the
    point of the change, so the *other* version has to be exercised deliberately. There is no CI to
    do it, so run it by hand when this file or ``pyproject.toml``'s numpy pin changes::

        python -m venv /tmp/np2 && /tmp/np2/bin/pip install -q 'numpy>=2' pyyaml pytest
        PYTHONPATH=. /tmp/np2/bin/python -m pytest tests/

    Last run 2026-07-31 on numpy 2.4.6: 442 passed, 25 skipped (the skips are the optional
    pandas / joblib / matplotlib extras, absent from that venv by design).
    """
    x = np.linspace(-1.0, 1.0, 200_001)
    y = np.sqrt(np.maximum(1.0 - x**2, 0.0))
    assert _trapezoid(y, x) == pytest.approx(math.pi / 2, abs=1e-7)


def test_the_two_radial_cdf_branches_agree_at_the_boundary() -> None:
    """``_radial_cdf`` special-cases the isotropic case with an exact Rayleigh form and integrates
    numerically otherwise. The two must meet where they touch, or the bisection either side of that
    boundary is solving a discontinuous function.

    They agree to ~1.4e-6, which is the 4001-point grid's own quadrature error against the closed
    form (the integrand has a square-root singularity at both endpoints), not a defect. Asserting
    exact equality would be asserting that numerical integration is exact.
    """
    sigma = 7.0
    exact = _radial_cdf(12.0, sigma, sigma)
    nearly = _radial_cdf(12.0, sigma * (1 + 1e-9), sigma)
    assert nearly == pytest.approx(exact, abs=1e-5)
