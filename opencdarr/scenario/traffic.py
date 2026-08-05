"""Traffic drawn rather than arranged — the entry rule of Groot et al. (2024).

Aircraft cross a disc on random headings, with the perpendicular offset of each track uniform
across the diameter so the traffic is homogeneous over the area being measured. They are released
on a larger circle and enter the disc already in flight, which is what keeps the first seconds
after release — where two aircraft can appear close together with no history of having been
separated — flown but not measured.

Derived, with the one deliberate deviation from the paper's Eq. (8), in
``vault/derivations/random-spawn-conflict-probability.md``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from opencdarr import geo
from opencdarr.config import Config
from opencdarr.fleet import MeasurementArea
from opencdarr.scenario.base import _LAT0, _LON0, FleetScenario, Scenario
from opencdarr.state import AircraftState


def random_traffic(
    n: int, rng: np.random.Generator, *, speed: float = 10.0,
    r_inner: float = 1000.0, r_outer: float = 1200.0,
    lat0: float = 52.0, lon0: float = 4.0,
    pos_ci95: float = 0.0, vel_ci95: float = 0.0,
) -> FleetScenario:
    """``n`` aircraft crossing a disc of radius ``r_inner`` on random headings — traffic *drawn*
    rather than arranged, after Groot, Ellerbroek & Hoekstra (2024) §3.1.2.

    Each aircraft draws a heading uniformly and a perpendicular offset uniformly across the
    diameter, then flies the chord that offset cuts — a diameter only when the offset is zero. The
    uniform *offset* is the point: spreading entry *bearings* uniformly around the perimeter
    instead would crowd the traffic toward the edge, whereas this makes it homogeneous across the
    area being measured (the paper's Fig. 4, left, and the ``asin`` in its Eq. (8) — that equation
    is this construction written in polar form).

    Aircraft are released on a **larger** circle ``r_outer`` and enter the disc already in flight.
    That run-in matters whenever a fleet is released simultaneously: entry bearings are uniform, so
    two aircraft can start close together, and a pair that materialises 150 m apart and converging
    has had no chance to be separated earlier. Pair it with
    :class:`~opencdarr.fleet.MeasurementArea` so those first seconds are flown but not measured.

    The offset is drawn across the **inner** diameter, so all ``n`` aircraft cross the measured
    disc; the paper references it to the outer radius, where ~17% graze past — right when the
    controlled quantity is a density, awkward when it is a count. Derived, with that deviation
    stated, in ``vault/derivations/random-spawn-conflict-probability.md``.
    """
    if not r_outer >= r_inner:
        raise ValueError(f"r_outer ({r_outer}) must be at least r_inner ({r_inner})")
    out: FleetScenario = []
    for k in range(n):
        heading = float(rng.uniform(0.0, 360.0))
        offset = r_inner * float(rng.uniform(-1.0, 1.0))
        half = math.sqrt(r_outer**2 - offset**2)  # half-chord across the release circle
        side = (heading + 90.0) % 360.0 if offset >= 0 else (heading - 90.0) % 360.0
        foot = geo.forward(lat0, lon0, side, abs(offset))  # closest point of the track to centre
        start = geo.forward(foot[0], foot[1], (heading + 180.0) % 360.0, half)
        target = geo.forward(foot[0], foot[1], heading, half)
        out.append((
            AircraftState(id=f"A{k}", lat=start[0], lon=start[1], trk=heading, gs=speed,
                          pos_ci95=pos_ci95, vel_ci95=vel_ci95),
            target,
        ))
    return out

@dataclass(frozen=True)
class RandomTraffic(Scenario):
    """``n`` aircraft crossing a disc on random headings — traffic drawn rather than arranged.

    Released on ``r_outer`` and measured inside ``r_inner``, so the seconds after release — where
    two aircraft can appear close together with no history of having been separated — are flown
    but not counted. See :func:`random_traffic` and
    ``vault/derivations/random-spawn-conflict-probability.md``.
    """

    n: int = 6
    r_inner: float = 1000.0
    r_outer: float = 1200.0

    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        return random_traffic(
            self.n, rng, speed=config.scenario.speed,
            r_inner=self.r_inner, r_outer=self.r_outer,
            pos_ci95=config.scenario.pos_ci95, vel_ci95=config.scenario.vel_ci95,
        )

    def measurement_area(self) -> MeasurementArea:
        return MeasurementArea((_LAT0, _LON0), self.r_inner)

    def size(self) -> int:
        return self.n
