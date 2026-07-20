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

    ``pos_ci95`` — 95% radial position accuracy [m]; ``vel_ci95`` — 95% radial velocity accuracy
    [m/s], converted to a per-axis sigma the same way as position (isotropic 2D Gaussian, see
    ``vault/derivations/gps-noise.md``); ``distribution`` — the position-error model (default
    isotropic Gaussian).
    """

    def __init__(
        self,
        pos_ci95: float,
        vel_ci95: float,
        distribution: NoiseDistribution = gaussian,
    ) -> None:
        self.pos_ci95 = pos_ci95
        self.vel_ci95 = vel_ci95
        self.distribution = distribution

    def measure(self, true: AircraftState, t: float, rng: np.random.Generator) -> Message:
        # position error (East, North) -> offset the true position via our geodesy
        err_e, err_n = self.distribution(rng, self.pos_ci95, true.trk)
        bearing = math.degrees(math.atan2(err_e, err_n)) % 360.0
        lat, lon = geo.forward(true.lat, true.lon, bearing, math.hypot(err_e, err_n))

        # velocity error (East, North) -> measured track and ground speed
        vel_sigma = self.vel_ci95 * CI95_TO_SIGMA
        ve, vn = velocity_enu(true)
        ve += float(rng.normal(0.0, vel_sigma))
        vn += float(rng.normal(0.0, vel_sigma))
        trk = math.degrees(math.atan2(ve, vn)) % 360.0
        gs = math.hypot(ve, vn)

        measured = AircraftState(id=true.id, lat=lat, lon=lon, trk=trk, gs=gs)
        return Message(source=true.id, state=measured, t_meas=t)
