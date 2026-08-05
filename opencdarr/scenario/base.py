"""What every scenario is, and what they share.

A :class:`Scenario` is one encounter drawn from a seed — the only thing that differs between an
angle sweep, a ring study and a traffic study. Everything downstream (the rules, both estimators,
the caching and the reporting) is shared, so implementing :meth:`Scenario.draw` is the whole cost
of a new experiment family. The concrete ones live one per file beside this module; see
``README.md`` for what each is and when to reach for it.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable

import numpy as np

from opencdarr import geo
from opencdarr.config import Config
from opencdarr.fleet import MeasurementArea
from opencdarr.state import AircraftState

_LAT0, _LON0 = 52.0, 4.0  # the default origin every fleet builder is centred on

Draw = Callable[[np.random.Generator], float]
"""A per-encounter draw of one geometry parameter from that encounter's generator."""

# ``None`` as the goal means the aircraft has no destination and simply holds its cruise — the
# shape a pairwise encounter takes, where the geometry *is* the experiment and there is nowhere to
# arrive. Every N-aircraft builder below supplies a real goal.
FleetScenario = list[tuple[AircraftState, tuple[float, float] | None]]


def _heading_to(lat: float, lon: float, target: tuple[float, float], speed: float,
                ac_id: str, pos_ci95: float = 0.0, vel_ci95: float = 0.0) -> AircraftState:
    """An aircraft at ``(lat, lon)`` flying at ``speed`` toward ``target`` (nose on the bearing)"""
    trk, _ = geo.qdrdist(lat, lon, target[0], target[1])
    return AircraftState(id=ac_id, lat=lat, lon=lon, trk=trk % 360.0, gs=speed,
                         pos_ci95=pos_ci95, vel_ci95=vel_ci95)


def _heading_to(lat: float, lon: float, target: tuple[float, float], speed: float,
                ac_id: str, pos_ci95: float = 0.0, vel_ci95: float = 0.0) -> AircraftState:
    """An aircraft at ``(lat, lon)`` flying at ``speed`` toward ``target`` (nose on the bearing)"""
    trk, _ = geo.qdrdist(lat, lon, target[0], target[1])
    return AircraftState(id=ac_id, lat=lat, lon=lon, trk=trk % 360.0, gs=speed,
                         pos_ci95=pos_ci95, vel_ci95=vel_ci95)


class Scenario(ABC):
    """One encounter, drawn from a seed — the contribution surface for a new experiment family.

    Implement :meth:`draw` and you have a scenario both estimators can run: plain Monte Carlo
    calls it once per encounter, and the rare-event estimator calls it once per particle to build
    its initial cloud. Neither knows what geometry came out.

    Stay **airframe-neutral**: return states and goals, and let the caller's ``perf`` /
    ``kinematics`` / ``airframes`` decide what flies them. A scenario that needs a particular
    airframe mix (a fixed-wing against a multirotor) declares its :meth:`size` and leaves the
    pairing to ``Methods.airframes``.
    """

    @abstractmethod
    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        """One fleet as ``(state, goal)`` pairs; ``goal=None`` means hold cruise.

        Draw everything random from ``rng`` — it is this encounter's own substream, so a scenario
        that ignores it is deterministic by construction (the ring), and one that uses it samples
        a distribution (traffic). Read the encounter distribution's parameters from ``config``
        (speed, accuracies, ``rpz``) rather than hard-coding them, so a sweep over those still
        reaches the geometry.
        """

    def measurement_area(self) -> MeasurementArea | None:
        """The experimental area, when this scenario has one; ``None`` measures everywhere.

        Belongs to the scenario because it is part of the design — "release at 1.2 km, measure
        inside 1 km" *is* the traffic scenario — and coupling them makes a mismatched pair
        unrepresentable rather than a silent inconsistency between two declarations.
        """
        return None

    def size(self) -> int | None:
        """Fleet size when it is fixed, or ``None`` when the scenario decides per draw.

        Read to check a mixed-fleet ``airframes`` list against the fleet it will have to fly.
        """
        return None

    def supports_splitting(self) -> bool:
        """Whether the rare-event estimator can be pointed at this scenario.

        True for a scenario whose encounter is a bounded engagement, so that the running minimum
        separation is a meaningful importance function. A future open-ended traffic *stream* would
        return False: over hours of arrivals "at least one loss" stops being rare, and the running
        minimum stops discriminating between particles, so splitting would return a number near 1
        after a great deal of compute. Declared here so that combination fails at declaration time.
        """
        return True
