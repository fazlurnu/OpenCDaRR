"""What every scenario shares: the fleet type, the per-parameter draw, and the placement helper.

A scenario turns one seed into one encounter. What differs between an angle sweep and a ring study
is only that geometry; everything downstream — the rules, both estimators, the caching and the
reporting — is shared. The concrete families live one per file beside this module.

The cut between those files is by **encounter family**, not by construct: a builder and the type
that describes it change for the same reason, so they stay together.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from opencdarr import geo
from opencdarr.state import AircraftState

Draw = Callable[[np.random.Generator], float]
"""A per-encounter draw of one geometry parameter from that encounter's generator."""



def _resolve(spec: float | Draw | None, rng: np.random.Generator, drawn: float) -> float:
    """One geometry slot's value: the built-in draw, a pinned constant, or a custom distribution.

    ``drawn`` has *already* been taken from ``rng`` by the caller, whether or not it is used — see
    :func:`sample_pairwise` on why a pinned slot still consumes its draw.
    """
    if spec is None:
        return drawn
    if callable(spec):
        return float(spec(rng))
    return float(spec)



# --- N-aircraft fleet scenarios -------------------------------------------------------------
# Each builder returns a list of ``(AircraftState, goto_target)`` pairs — an aircraft heading at
# its destination ``(lat, lon)``, the geometry the fleet loop needs. The caller wraps each in a
# ``WaypointAutopilot`` mission + its airframe (an ``Agent``); the scenario stays airframe-neutral.

FleetScenario = list[tuple[AircraftState, tuple[float, float]]]


def _heading_to(lat: float, lon: float, target: tuple[float, float], speed: float,
                ac_id: str) -> AircraftState:
    """An aircraft at ``(lat, lon)`` flying at ``speed`` toward ``target`` (nose on the bearing)"""
    trk, _ = geo.qdrdist(lat, lon, target[0], target[1])
    return AircraftState(id=ac_id, lat=lat, lon=lon, trk=trk % 360.0, gs=speed)


