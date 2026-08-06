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
from opencdarr.scenario.random_traffic import (
    MEASURED_FRACTION,
    aircraft_for_density,
    measurement_area,
    random_traffic,
)
from opencdarr.scenario.ring import converging_ring, crossing_ring, swap_ring

__all__ = [
    "MEASURED_FRACTION",
    "Draw",
    "FleetScenario",
    "converging_ring",
    "create_conflict",
    "crossing_ring",
    "aircraft_for_density",
    "measurement_area",
    "near_parallel",
    "random_traffic",
    "sample_pairwise",
    "swap_pair",
    "swap_ring",
]
