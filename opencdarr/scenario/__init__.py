"""Scenario builders — one import surface, one file per encounter family.

Unchanged from when this was a single module: everything is re-exported here, so
``from opencdarr.scenario import sample_pairwise`` still works.
"""

from opencdarr.scenario.base import Draw, FleetScenario, Scenario
from opencdarr.scenario.pairwise import (
    PairwiseEncounter,
    create_conflict,
    near_parallel,
    pairwise,
    sample_pairwise,
    swap_pair,
)
from opencdarr.scenario.random_traffic import (
    MEASURED_FRACTION,
    RandomTraffic,
    aircraft_for_density,
    measurement_area,
    random_traffic,
)
from opencdarr.scenario.ring import (
    ConvergingRing,
    CrossingRing,
    SwapRing,
    converging_ring,
    crossing_ring,
    swap_ring,
)

__all__ = [
    "MEASURED_FRACTION",
    "ConvergingRing",
    "CrossingRing",
    "Draw",
    "FleetScenario",
    "SwapRing",
    "Scenario",
    "RandomTraffic",
    "PairwiseEncounter",
    "converging_ring",
    "create_conflict",
    "crossing_ring",
    "aircraft_for_density",
    "measurement_area",
    "near_parallel",
    "pairwise",
    "random_traffic",
    "sample_pairwise",
    "swap_pair",
    "swap_ring",
]
