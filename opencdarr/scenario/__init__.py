"""Scenario builders and the declarative :class:`Scenario` layer.

One import surface, unchanged from when this was a single module: everything below is re-exported
here, so ``from opencdarr.scenario import sample_pairwise`` still works. What each scenario is,
and when to reach for it, is in ``README.md``.
"""

from opencdarr.scenario.base import Draw, FleetScenario, Scenario
from opencdarr.scenario.pairwise import (
    PairwiseEncounter,
    create_conflict,
    near_parallel,
    sample_pairwise,
    swap_pair,
)
from opencdarr.scenario.ring import (
    CrossingRing,
    converging_ring,
    crossing_ring,
    swap_ring,
)
from opencdarr.scenario.traffic import RandomTraffic, random_traffic

__all__ = [
    "CrossingRing",
    "Draw",
    "FleetScenario",
    "PairwiseEncounter",
    "RandomTraffic",
    "Scenario",
    "converging_ring",
    "create_conflict",
    "crossing_ring",
    "near_parallel",
    "random_traffic",
    "sample_pairwise",
    "swap_pair",
    "swap_ring",
]
