"""Relative kinematics shared by detection, resolution, and recovery.

``relative_enu`` extracts the relative position and velocity of ``intr`` with respect to
``own`` in the local East–North frame — the common front-end of the CPA algebra. Centralising
it keeps the ``intr − own`` sign convention in *one* place (see ``cpa-detection.md``), so no
algorithm can accidentally flip it. The CPA equations themselves (``t_cpa``, ``d_cpa``, …)
deliberately stay in each algorithm, where a reviewer reads them (``design-philosophy.md``
#11: in plumbing DRY wins; in the core math legibility wins).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from opencdarr import geo
from opencdarr.state import AircraftState


def velocity_enu(state: AircraftState) -> tuple[float, float]:
    """Ground velocity as (East, North) components in m/s."""
    r = math.radians(state.trk)
    return state.gs * math.sin(r), state.gs * math.cos(r)


@dataclass(frozen=True)
class Relative:
    """Position and velocity of intr relative to own, East–North (intr − own)."""

    rx: float  # East position [m]
    ry: float  # North position [m]
    vx: float  # East velocity [m/s]
    vy: float  # North velocity [m/s]

    @property
    def dist(self) -> float:
        """Current range [m]."""
        return math.hypot(self.rx, self.ry)


def relative_enu(own: AircraftState, intr: AircraftState) -> Relative:
    """Relative position and velocity (intr − own) in the local East–North frame."""
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    vox, voy = velocity_enu(own)
    vix, viy = velocity_enu(intr)
    return Relative(rx=dist * math.sin(q), ry=dist * math.cos(q), vx=vix - vox, vy=viy - voy)
