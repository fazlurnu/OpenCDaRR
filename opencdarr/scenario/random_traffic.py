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

**Steady state, not spawning.** :func:`~opencdarr.fleet.run_fleet` flies a fixed list of aircraft
and cannot create new ones mid-run, so rather than releasing aircraft continuously at the border
this places the whole fleet in the steady state that process would produce: each aircraft is drawn
in proportion to the time it would spend inside the disc (its chord length), and is then placed
uniformly along its own chord. The result is a snapshot of established traffic rather than a
start-of-run artefact where every aircraft sits on the boundary at once.

**Two concentric areas.** The traffic fills a simulation disc, but results are measured in a
smaller experimental disc inside it (:func:`measurement_area`). The annulus between them is
airspace that is flown and not counted, which is what keeps an aircraft that has only just entered
— and has no history of ever having been separated from anything — out of the numbers. The ratio
is the paper's 1.35 NM / 1.62 NM.
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

_CHORD_EPS = 1.0  # m — drop outbound and grazing draws, which contribute no time inside


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
) -> FleetScenario:
    """``n`` aircraft crossing a disc of ``radius`` on random headings, in steady state.

    Unlike the placed scenarios this one is **drawn**, so it takes the encounter's own generator.
    Two draws from the same seed give the same traffic; two different seeds give independent
    traffic at the same density.

    Each aircraft's goal is a waypoint far beyond the disc along its own heading. Traffic aircraft
    have no destination — they cross and leave — so the goal exists only to give the fleet builders
    one shape; an aircraft exits the measured region long before reaching it.
    """
    if n < 1:
        raise ValueError(f"a traffic sample needs at least one aircraft, got {n}")
    speeds = _per_aircraft_speeds(speed, n)

    # Oversample, then keep the inbound draws and weight them by time inside. The pool is large
    # relative to n so that the chord-weighted choice has something to choose from.
    pool = 40 * n + 20_000
    heading = rng.uniform(0.0, 360.0, pool)
    offset = rng.uniform(-1.0, 1.0, pool)
    bearing = (heading + 180.0 + np.degrees(np.arcsin(offset))) % 360.0

    br, hr = np.radians(bearing), np.radians(heading)
    entry_e, entry_n = radius * np.sin(br), radius * np.cos(br)
    unit_e, unit_n = np.sin(hr), np.cos(hr)
    chord = -2.0 * (entry_e * unit_e + entry_n * unit_n)  # distance to the far side of the disc

    keep = chord > _CHORD_EPS
    entry_e, entry_n = entry_e[keep], entry_n[keep]
    unit_e, unit_n = unit_e[keep], unit_n[keep]
    chord, heading = chord[keep], heading[keep]

    # an aircraft is present in proportion to the time it spends inside, i.e. to its chord
    idx = rng.choice(chord.size, size=n, p=chord / chord.sum())
    along = rng.uniform(0.0, 1.0, n) * chord[idx]
    east = entry_e[idx] + unit_e[idx] * along
    north = entry_n[idx] + unit_n[idx] * along

    out: FleetScenario = []
    for k in range(n):
        qdr = math.degrees(math.atan2(east[k], north[k])) % 360.0
        lat, lon = geo.forward(lat0, lon0, qdr, math.hypot(east[k], north[k]))
        trk = float(heading[idx][k]) % 360.0
        far = geo.forward(lat, lon, trk, 3.0 * radius)  # beyond the disc; never reached
        out.append((
            AircraftState(id=f"T{k}", lat=lat, lon=lon, trk=trk, gs=speeds[k]),
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
        )

    def measurement_area(self) -> MeasurementArea:
        return measurement_area(self.radius, self.centre)
