"""Pluggable position-error distributions for navigation (GPS) noise.

Each matches :class:`~opencdarr.cns.base.NoiseDistribution`. Gaussian (isotropic) now; heavy-tail
mixtures and anisotropic (along/cross-track) distributions are Phase 3c — a new distribution
adds a function here, per the brief's "new noise models".
"""

from __future__ import annotations

import math

import numpy as np

# 95% radial CI -> per-axis 1-sigma for a 2D isotropic Gaussian: sigma = CI95 / sqrt(chi2_2,0.95).
_CI95_TO_SIGMA = 1.0 / math.sqrt(5.991464547)  # ~= 0.4085


def gaussian(rng: np.random.Generator, ci95: float, trk_deg: float) -> tuple[float, float]:
    """Zero-mean isotropic 2D Gaussian position error [m] as (East, North). ``trk_deg`` unused."""
    sigma = ci95 * _CI95_TO_SIGMA
    return float(rng.normal(0.0, sigma)), float(rng.normal(0.0, sigma))
