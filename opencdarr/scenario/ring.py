"""Aircraft arranged on a circle — the worst case you build on purpose.

Three arrangements, and the difference between them is where each aircraft is *aimed*.
:func:`crossing_ring` sends every aircraft to the point diametrically opposite its own start, so
every route is a diameter and the whole fleet arrives at the centre together. :func:`swap_ring`
sends each aircraft to *another aircraft's start*, which is the same thing only when ``n`` is even
— see ``README.md`` for the picture. :func:`converging_ring` sends them all to one shared point,
a goal that is incompatible with separation and therefore a scenario no resolver can clear.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from opencdarr import geo
from opencdarr.config import Config
from opencdarr.scenario.base import FleetScenario, Scenario, _heading_to


def swap_ring(
    n: int = 8, *, speed: float = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
) -> FleetScenario:
    """``n`` aircraft uniformly on a ring, each flying to the **diametrically-opposite** start
    (Phase-6 scenario 2) — ``n/2`` head-on pairs all crossing the centre.
    """
    starts = [geo.forward(lat0, lon0, 360.0 * k / n, radius) for k in range(n)]
    out: FleetScenario = []
    for k in range(n):
        target = starts[(k + n // 2) % n]
        out.append((_heading_to(starts[k][0], starts[k][1], target, speed, f"A{k}"), target))
    return out

def crossing_ring(
    n: int = 4, *, speed: float = 10.0, radius: float = 500.0,
    lat0: float = 52.0, lon0: float = 4.0,
    pos_ci95: float = 0.0, vel_ci95: float = 0.0,
) -> FleetScenario:
    """``n`` aircraft on a ring, each flying to the point **diametrically opposite its own start**
    — every route is a diameter, so all ``n`` arrive at the centre together. The arranged worst
    case: each aircraft is in conflict with every other one at the same instant, and the symmetry
    gives nobody priority.

    Differs from :func:`swap_ring` only at **odd** ``n``. That builder sends each aircraft to
    *another aircraft's start* (``(k + n//2) % n``), which is the antipode only when ``n`` is even;
    at ``n = 3`` it aims each one 120° round the ring instead. Here the target is the antipodal
    *point* whatever ``n`` is, so the fleet size can be swept through odd values without the
    geometry changing character. The two are identical for even ``n``.

    ``radius`` is the ring radius — half the "ring diameter" the encounter is usually described by.
    """
    out: FleetScenario = []
    for k in range(n):
        bearing = 360.0 * k / n
        start = geo.forward(lat0, lon0, bearing, radius)
        target = geo.forward(lat0, lon0, (bearing + 180.0) % 360.0, radius)
        out.append((_heading_to(start[0], start[1], target, speed, f"A{k}", pos_ci95, vel_ci95),
                    target))
    return out

def converging_ring(
    n: int = 8, *, speed: float = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
) -> FleetScenario:
    """``n`` aircraft uniformly on a circle, all flying to the **same** waypoint — the ring centre
    (Phase-6 scenario 3), the symmetric converging superconflict. They cannot all occupy the centre
    (``rpz`` forbids it), so the DAA can only hold them apart as they converge.
    """
    centre = (lat0, lon0)
    ring = [geo.forward(lat0, lon0, 360.0 * k / n, radius) for k in range(n)]
    return [(_heading_to(s[0], s[1], centre, speed, f"A{k}"), centre) for k, s in enumerate(ring)]

@dataclass(frozen=True)
class CrossingRing(Scenario):
    """``n`` aircraft on a ring, each flying to the point opposite — the arranged worst case.

    Deterministic: ``draw`` ignores its generator, so every encounter is the same geometry and the
    only thing varying between them is the CNS noise. That makes it the scenario that isolates
    what uncertainty alone does to a fixed conflict.
    """

    n: int = 4
    radius: float = 500.0

    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        return crossing_ring(
            self.n, speed=config.scenario.speed, radius=self.radius,
            pos_ci95=config.scenario.pos_ci95, vel_ci95=config.scenario.vel_ci95,
        )

    def size(self) -> int:
        return self.n
