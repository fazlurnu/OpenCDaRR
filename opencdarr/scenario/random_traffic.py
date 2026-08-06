"""Traffic drawn rather than arranged — the entry rule of Groot, Ellerbroek & Hoekstra (2024).

Every other scenario here *places* a geometry: a ring, a crossing pair, a head-on swap. This one
draws one, and the drawing rule is the whole point. Aircraft cross a disc on random headings with
the perpendicular offset of each track uniform across the diameter, which is what makes the traffic
homogeneous over the area being measured.

**The entry rule (their Eq. 8).** Draw a heading uniformly, then enter at

    bearing = heading + 180 + asin(x),     x ~ U(-1, 1)

Drawing the entry bearing uniformly around the rim instead — the obvious thing — concentrates
traffic near the edge, because a chord entering at a shallow angle spends very little time inside.
``examples/handbook/traffic_density.ipynb`` reproduces the paper's Fig. 4 and measures the
difference between the two rules.

Every draw is inbound by construction. The offset ``asin(x)`` is at most a quarter turn from the
inward radial, so the track's perpendicular distance from the centre works out at exactly ``R x``
— uniform across the diameter, which is the rule's whole claim — and the chord it flies is
``2R sqrt(1 - x^2)``.

**Release on the ring.** The whole fleet starts on the boundary of the simulation disc and flies
inward. :func:`~opencdarr.fleet.run_fleet` takes a fixed list of aircraft and cannot create new
ones mid-run, so this is a single cohort crossing rather than the paper's continuous arrivals: the
disc fills, is crossed, and empties. Density inside is therefore a transient rather than a
steady state, and a study that depends on holding a density should say over what window it reads
it.

**Two concentric areas, and why the release ring matters.** The traffic fills a simulation disc but
results are measured in a smaller experimental disc inside it (:func:`measurement_area`). The
annulus between them is airspace that is flown and not counted, and releasing on the outer boundary
is what makes that buffer do its job: an aircraft is not measured until it has crossed the annulus,
by which time it has been flying for some seconds among the others and has a *history* of having
been separated from them. Placing aircraft directly inside the measured disc would defeat it — two
of them can land within ``rpz`` of each other at the first step, having never been separated at
all, and no spatial gate can tell that apart from a real loss. The ratio is the paper's
1.35 NM / 1.62 NM.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr import geo
from opencdarr.config import Config
from opencdarr.measurement import Disc, MeasurementArea
from opencdarr.scenario.base import FleetScenario, Scenario, _per_aircraft_speeds
from opencdarr.state import AircraftState

# The paper's two concentric areas: measure inside 1.35 NM of a 1.62 NM simulation disc.
MEASURED_FRACTION = 1.35 / 1.62


def aircraft_for_density(density: float, radius: float) -> int:
    """How many aircraft give ``density`` aircraft/km^2 over a disc of ``radius`` metres.

    Density is defined over the **simulation** disc, as in the paper: it is a property of the
    traffic being generated, not of the smaller region the results are read from.
    """
    return max(2, round(density * math.pi * (radius / 1000.0) ** 2))


def measurement_area(radius: float = 1000.0, centre: tuple[float, float] = (52.0, 4.0)) -> Disc:
    """The experimental disc inside a simulation disc of ``radius`` — where results are counted.

    Returned rather than left to the caller because the pairing *is* the scenario: "fill this disc,
    measure inside that one" is one design, and splitting it across two declarations is how the two
    drift apart.
    """
    return Disc(centre=centre, radius=radius * MEASURED_FRACTION)


def random_traffic(
    rng: np.random.Generator,
    n: int,
    *,
    radius: float = 1000.0,
    speed: float | Sequence[float] = 10.0,
    lat0: float = 52.0,
    lon0: float = 4.0,
    pos_ci95: float = 0.0,
    vel_ci95: float = 0.0,
) -> FleetScenario:
    """``n`` aircraft released on the boundary of a disc of ``radius``, all flying inward.

    Unlike the placed scenarios this one is **drawn**, so it takes the encounter's own generator.
    Two draws from the same seed give the same traffic; two different seeds give independent
    traffic at the same density.

    Every aircraft starts on the ring, which is what gives the annulus between the release ring and
    the measured disc something to do: nothing is counted until an aircraft has crossed it.

    Each aircraft's goal is a waypoint far beyond the disc along its own heading. Traffic aircraft
    have no destination — they cross and leave — so the goal exists only to give the fleet builders
    one shape; an aircraft leaves the disc long before reaching it.
    """
    if n < 1:
        raise ValueError(f"a traffic sample needs at least one aircraft, got {n}")
    speeds = _per_aircraft_speeds(speed, n)

    heading = rng.uniform(0.0, 360.0, n)
    offset = rng.uniform(-1.0, 1.0, n)
    # the entry rule. asin(x) is at most a quarter turn off the inward radial, so every draw flies
    # into the disc and the track's perpendicular offset from the centre comes out at exactly R*x.
    bearing = (heading + 180.0 + np.degrees(np.arcsin(offset))) % 360.0

    out: FleetScenario = []
    for k in range(n):
        lat, lon = geo.forward(lat0, lon0, float(bearing[k]), radius)
        trk = float(heading[k]) % 360.0
        far = geo.forward(lat, lon, trk, 3.0 * radius)  # beyond the disc; never reached
        out.append((
            AircraftState(id=f"T{k}", lat=lat, lon=lon, trk=trk, gs=speeds[k],
                          pos_ci95=pos_ci95, vel_ci95=vel_ci95),
            (far[0], far[1]),
        ))
    return out


@dataclass(frozen=True)
class RandomTraffic(Scenario):
    """Traffic at a given density, with the measured disc it belongs to.

    Give it ``density`` (aircraft/km^2 over the simulation disc, the paper's unit) or ``n``
    directly. The two are alternatives: declaring both is ambiguous and is refused.

    This is the scenario that motivates :meth:`~opencdarr.scenario.base.Scenario.measurement_area`.
    The traffic fills the simulation disc and the results are read from the smaller experimental
    one; carrying the pair here is what stops a caller filling one disc and measuring another.
    """

    density: float | None = None
    n: int | None = None
    radius: float = 1000.0
    centre: tuple[float, float] = (52.0, 4.0)

    def __post_init__(self) -> None:
        if (self.density is None) == (self.n is None):
            raise ValueError("give either density (aircraft/km^2) or n, not both and not neither")

    def size(self) -> int:
        if self.n is not None:
            return self.n
        assert self.density is not None  # guaranteed by __post_init__
        return aircraft_for_density(self.density, self.radius)

    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        return random_traffic(
            rng, self.size(), radius=self.radius, speed=config.scenario.speed,
            lat0=self.centre[0], lon0=self.centre[1],
            pos_ci95=config.scenario.pos_ci95, vel_ci95=config.scenario.vel_ci95,
        )

    def measurement_area(self) -> MeasurementArea:
        return measurement_area(self.radius, self.centre)
