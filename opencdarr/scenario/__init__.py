"""Scenario builders — one import surface, one file per encounter family.

Unchanged from when this was a single module: everything is re-exported here, so
``from opencdarr.scenario import sample_pairwise`` still works.
"""

from opencdarr.scenario.base import Draw, FleetScenario
from opencdarr.scenario.pairwise import (
    create_conflict,
    near_parallel,
    sample_pairwise,
    swap_pair,
)
from opencdarr.scenario.ring import converging_ring, swap_ring

__all__ = [
    "Draw",
    "FleetScenario",
    "converging_ring",
    "create_conflict",
    "near_parallel",
    "sample_pairwise",
    "swap_pair",
    "swap_ring",
]
