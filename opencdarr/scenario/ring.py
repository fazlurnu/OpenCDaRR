"""Ring geometries: ``n`` aircraft on a circle, converging on one point or on each other.

Both are *placed*, not drawn — the geometry is the experiment, so a ring run from one seed is the
ring run. They are the multi-aircraft stress cases: a swap ring is ``n/2`` simultaneous head-on
pairs, and a converging ring is the symmetric superconflict where the goal itself is incompatible
with separation.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr import geo
from opencdarr.config import Config
from opencdarr.scenario.base import (
    FleetScenario,
    Scenario,
    _heading_to,
    _per_aircraft_speeds,
)


def swap_ring(
    n: int = 8, *, speed: float | Sequence[float] = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
    pos_ci95: float = 0.0, vel_ci95: float = 0.0,
) -> FleetScenario:
    """``n`` aircraft uniformly on a ring, each flying to the **diametrically-opposite** start
    (Phase-6 scenario 2) — ``n/2`` head-on pairs all crossing the centre.

    ``speed`` is one value for the fleet, or one per aircraft in ring order
    (:func:`~opencdarr.scenario.base._per_aircraft_speeds`).
    """
    speeds = _per_aircraft_speeds(speed, n)
    starts = [geo.forward(lat0, lon0, 360.0 * k / n, radius) for k in range(n)]
    out: FleetScenario = []
    for k in range(n):
        target = starts[(k + n // 2) % n]
        out.append((_heading_to(starts[k][0], starts[k][1], target, speeds[k], f"A{k}",
                             pos_ci95, vel_ci95), target))
    return out



def crossing_ring(
    n: int = 8, *, speed: float | Sequence[float] = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
    pos_ci95: float = 0.0, vel_ci95: float = 0.0,
) -> FleetScenario:
    """``n`` aircraft uniformly on a ring, each flying to the diametrically-opposite **point**.

    Every route is a full diameter, so the whole fleet meets at the centre at every ``n``. That is
    the difference from :func:`swap_ring`, which aims each aircraft at another aircraft's *start* —
    the one ``n // 2`` places round the ring. At even ``n`` that start **is** the antipode and the
    two builders place the identical fleet; at odd ``n`` it is not, and ``swap_ring``'s routes miss
    the centre by ``radius * cos(180 * (n // 2) / n)`` — 750 m at ``n = 3`` on a 1500 m ring.

    Reach for this one when the **fleet size is the variable**. Sweeping ``n`` over ``swap_ring``
    steps the geometry at every odd value as well as the size, so a trend across that sweep mixes
    the two effects; here only the size changes.

    ``speed`` is one value for the fleet, or one per aircraft in ring order.
    """
    speeds = _per_aircraft_speeds(speed, n)
    out: FleetScenario = []
    for k in range(n):
        bearing = 360.0 * k / n
        start = geo.forward(lat0, lon0, bearing, radius)
        far = geo.forward(lat0, lon0, (bearing + 180.0) % 360.0, radius)
        target = (far[0], far[1])
        out.append((_heading_to(start[0], start[1], target, speeds[k], f"A{k}",
                             pos_ci95, vel_ci95), target))
    return out


def converging_ring(
    n: int = 8, *, speed: float | Sequence[float] = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
    pos_ci95: float = 0.0, vel_ci95: float = 0.0,
) -> FleetScenario:
    """``n`` aircraft uniformly on a circle, all flying to the **same** waypoint — the ring centre
    (Phase-6 scenario 3), the symmetric converging superconflict. They cannot all occupy the centre
    (``rpz`` forbids it), so the DAA can only hold them apart as they converge.

    ``speed`` is one value for the fleet, or one per aircraft in ring order.
    """
    speeds = _per_aircraft_speeds(speed, n)
    centre = (lat0, lon0)
    ring = [geo.forward(lat0, lon0, 360.0 * k / n, radius) for k in range(n)]
    return [(_heading_to(s[0], s[1], centre, speeds[k], f"A{k}", pos_ci95, vel_ci95),
             centre) for k, s in enumerate(ring)]




@dataclass(frozen=True)
class _Ring(Scenario):
    """Shared shape of the ring scenarios: a fixed fleet on a circle, placed not drawn."""

    n: int = 8
    radius: float = 1500.0

    def size(self) -> int:
        return self.n


@dataclass(frozen=True)
class SwapRing(_Ring):
    """:func:`swap_ring` as a scenario — each aircraft to another aircraft's start."""

    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        return swap_ring(self.n, speed=config.scenario.speed, radius=self.radius,
                         pos_ci95=config.scenario.pos_ci95,
                         vel_ci95=config.scenario.vel_ci95)


@dataclass(frozen=True)
class CrossingRing(_Ring):
    """:func:`crossing_ring` as a scenario — every route a diameter, at any fleet size."""

    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        return crossing_ring(self.n, speed=config.scenario.speed, radius=self.radius,
                             pos_ci95=config.scenario.pos_ci95,
                             vel_ci95=config.scenario.vel_ci95)


@dataclass(frozen=True)
class ConvergingRing(_Ring):
    """:func:`converging_ring` as a scenario — the symmetric superconflict."""

    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        return converging_ring(self.n, speed=config.scenario.speed, radius=self.radius,
                               pos_ci95=config.scenario.pos_ci95,
                               vel_ci95=config.scenario.vel_ci95)
