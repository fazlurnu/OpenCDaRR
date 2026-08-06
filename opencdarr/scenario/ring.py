"""Ring geometries: ``n`` aircraft on a circle, converging on one point or on each other.

Both are *placed*, not drawn — the geometry is the experiment, so a ring run from one seed is the
ring run. They are the multi-aircraft stress cases: a swap ring is ``n/2`` simultaneous head-on
pairs, and a converging ring is the symmetric superconflict where the goal itself is incompatible
with separation.
"""

from __future__ import annotations

from collections.abc import Sequence

from opencdarr import geo
from opencdarr.scenario.base import FleetScenario, _heading_to, _per_aircraft_speeds


def swap_ring(
    n: int = 8, *, speed: float | Sequence[float] = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
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
        out.append((_heading_to(starts[k][0], starts[k][1], target, speeds[k], f"A{k}"), target))
    return out



def converging_ring(
    n: int = 8, *, speed: float | Sequence[float] = 10.0, radius: float = 1500.0,
    lat0: float = 52.0, lon0: float = 4.0,
) -> FleetScenario:
    """``n`` aircraft uniformly on a circle, all flying to the **same** waypoint — the ring centre
    (Phase-6 scenario 3), the symmetric converging superconflict. They cannot all occupy the centre
    (``rpz`` forbids it), so the DAA can only hold them apart as they converge.

    ``speed`` is one value for the fleet, or one per aircraft in ring order.
    """
    speeds = _per_aircraft_speeds(speed, n)
    centre = (lat0, lon0)
    ring = [geo.forward(lat0, lon0, 360.0 * k / n, radius) for k in range(n)]
    return [(_heading_to(s[0], s[1], centre, speeds[k], f"A{k}"), centre)
            for k, s in enumerate(ring)]


