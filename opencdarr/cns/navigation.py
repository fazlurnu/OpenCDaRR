"""GPS self-measurement (navigation noise).

Implements :class:`~opencdarr.cns.base.NavigationModel`. Governing equations:
``vault/derivations/gps-noise.md``.
"""

from __future__ import annotations

import math

import numpy as np

from opencdarr import geo
from opencdarr.cns.base import Message, NavigationModel, NoiseDistribution
from opencdarr.cns.noise_distributions import CI95_TO_SIGMA, gaussian
from opencdarr.kinematics import velocity_enu
from opencdarr.state import AircraftState


class GpsNavigation(NavigationModel):
    """Measure own position (pluggable distribution) and velocity with Gaussian error.

    The noise magnitude is **not** a constructor parameter here: it is read from the aircraft
    being measured (``true.pos_ci95``, ``true.vel_ci95`` — 95% radial position [m] / velocity
    [m/s] accuracy), since accuracy is a property of *that* aircraft's sensor, can differ per
    aircraft, and may evolve over a run (``AircraftState``'s docstring). ``distribution`` is the
    position-error model (default isotropic Gaussian); velocity always uses a per-axis Gaussian
    converted from CI95 the same way as position (see ``vault/derivations/gps-noise.md``).
    """

    def __init__(self, distribution: NoiseDistribution = gaussian) -> None:
        self.distribution = distribution

    def measure(self, true: AircraftState, t: float, rng: np.random.Generator) -> Message:
        # position error (East, North) -> offset the true position via our geodesy
        err_e, err_n = self.distribution(rng, true.pos_ci95, true.trk)
        bearing = math.degrees(math.atan2(err_e, err_n)) % 360.0
        lat, lon = geo.forward(true.lat, true.lon, bearing, math.hypot(err_e, err_n))

        # velocity error (East, North) -> measured track and ground speed
        vel_sigma = true.vel_ci95 * CI95_TO_SIGMA
        ve, vn = velocity_enu(true)
        ve += float(rng.normal(0.0, vel_sigma))
        vn += float(rng.normal(0.0, vel_sigma))
        trk = math.degrees(math.atan2(ve, vn)) % 360.0
        gs = math.hypot(ve, vn)

        # the broadcast declares the same accuracy the sensor had when it took this measurement
        measured = AircraftState(
            id=true.id, lat=lat, lon=lon, trk=trk, gs=gs,
            pos_ci95=true.pos_ci95, vel_ci95=true.vel_ci95,
        )
        return Message(source=true.id, state=measured, t_meas=t)
