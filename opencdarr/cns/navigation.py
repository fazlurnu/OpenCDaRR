"""GNSS self-measurement (navigation noise).

Implements :class:`~opencdarr.cns.base.NavigationModel`. Governing equations:
``vault/derivations/gps-noise.md``.
"""

from __future__ import annotations

import math

import numpy as np

from opencdarr import geo
from opencdarr.cns.base import Message, NavigationModel, NoiseDistribution
from opencdarr.cns.noise_distributions import gaussian
from opencdarr.relative import velocity_enu
from opencdarr.state import AircraftState


class GnssNavigation(NavigationModel):
    """Measure own position and velocity, each with a pluggable 2D error distribution.

    The noise magnitude is **not** a constructor parameter here: it is read from the aircraft
    being measured (``true.pos_ci95``, ``true.vel_ci95`` — 95% radial position [m] / velocity
    [m/s] accuracy), since accuracy is a property of *that* aircraft's sensor, can differ per
    aircraft, and may evolve over a run (``AircraftState``'s docstring). ``pos_distribution`` and
    ``vel_distribution`` are the position- and velocity-error models (default isotropic Gaussian
    for both); each is a :class:`~opencdarr.cns.base.NoiseDistribution` mapping a CI95 to a 2D
    ``(east, north)`` error. Position and velocity are separate quantities from the same receiver
    (pseudorange vs Doppler), so they take independent distributions
    (see ``vault/derivations/gps-noise.md``).

    The error is drawn from the aircraft's **actual** accuracy and the broadcast carries its
    **declared** one. They are the same number unless the aircraft sets
    ``pos_ci95_declared``/``vel_ci95_declared``, which is how a mismatch between what a sensor
    delivers and what its transponder claims is expressed — see :meth:`measure`.
    """

    def __init__(
        self,
        pos_distribution: NoiseDistribution = gaussian,
        vel_distribution: NoiseDistribution = gaussian,
    ) -> None:
        self.pos_distribution = pos_distribution
        self.vel_distribution = vel_distribution

    def measure(self, true: AircraftState, t: float, rng: np.random.Generator) -> Message:
        # position error (East, North) -> offset the true position via our geodesy
        err_e, err_n = self.pos_distribution(rng, true.pos_ci95)
        bearing = math.degrees(math.atan2(err_e, err_n)) % 360.0
        lat, lon = geo.forward(true.lat, true.lon, bearing, math.hypot(err_e, err_n))

        # velocity error (East, North) -> measured track and ground speed
        verr_e, verr_n = self.vel_distribution(rng, true.vel_ci95)
        ve, vn = velocity_enu(true)
        ve += verr_e
        vn += verr_n
        trk = math.degrees(math.atan2(ve, vn)) % 360.0
        gs = math.hypot(ve, vn)

        # The broadcast declares *one* accuracy -- the sender's claim, which is what a receiver
        # reads. That is the accuracy the sensor actually had unless the aircraft says otherwise
        # (`pos_ci95_declared`/`vel_ci95_declared`, `None` = honest), so the error above is drawn
        # from the truth while the message carries the claim. The two agreeing is the default and
        # the ordinary case; them disagreeing is the integrity failure RAIM exists to catch.
        measured = AircraftState(
            id=true.id, lat=lat, lon=lon, trk=trk, gs=gs,
            pos_ci95=true.pos_ci95 if true.pos_ci95_declared is None else true.pos_ci95_declared,
            vel_ci95=true.vel_ci95 if true.vel_ci95_declared is None else true.vel_ci95_declared,
        )
        return Message(source=true.id, state=measured, t_meas=t)
