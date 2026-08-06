"""What every scenario shares: the fleet type, the per-parameter draw, and the placement helper.

A scenario turns one seed into one encounter. What differs between an angle sweep and a ring study
is only that geometry; everything downstream — the rules, both estimators, the caching and the
reporting — is shared. The concrete families live one per file beside this module.

The cut between those files is by **encounter family**, not by construct: a builder and the type
that describes it change for the same reason, so they stay together.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence

import numpy as np

from opencdarr import geo
from opencdarr.config import Config
from opencdarr.measurement import MeasurementArea
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

FleetScenario = list[tuple[AircraftState, tuple[float, float] | None]]


def _heading_to(lat: float, lon: float, target: tuple[float, float], speed: float,
                ac_id: str, pos_ci95: float = 0.0, vel_ci95: float = 0.0) -> AircraftState:
    """An aircraft at ``(lat, lon)`` flying at ``speed`` toward ``target`` (nose on the bearing).

    ``pos_ci95`` / ``vel_ci95`` are the accuracies this aircraft measures itself to. They must be
    carried here rather than left at zero: a navigation model reads them off the state, so a fleet
    built without them flies *noiselessly* whatever CNS stack is declared — deterministic, and with
    a declared noise sweep that silently does nothing.
    """
    trk, _ = geo.qdrdist(lat, lon, target[0], target[1])
    return AircraftState(id=ac_id, lat=lat, lon=lon, trk=trk % 360.0, gs=speed,
                         pos_ci95=pos_ci95, vel_ci95=vel_ci95)


class Scenario(ABC):
    """One encounter, drawn from a seed — the contribution surface for a new experiment family.

    Implement :meth:`draw` and both estimators can run it: plain Monte Carlo calls it once per
    encounter, the rare-event estimator once per particle to build its initial cloud. Neither knows
    what geometry came out.

    Stay **airframe-neutral**. Return states and goals; let the caller's ``perf`` /
    ``kinematics`` / ``airframes`` decide what flies them. That is what lets one geometry be flown
    by a multirotor fleet, a fixed-wing fleet or a mixed one without the scenario knowing.

    The three optional methods exist because a scenario knows things about itself that the runner
    would otherwise have to be told separately — and a fact told separately is a fact that can
    disagree.
    """

    @abstractmethod
    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        """One fleet as ``(state, goal)`` pairs.

        Draw everything random from ``rng``: it is this encounter's own substream, so a scenario
        that ignores it is deterministic by construction (a ring) and one that uses it samples a
        distribution (traffic). Read the encounter's parameters from ``config`` — speed, the
        declared accuracies, ``rpz`` — rather than hard-coding them, so a sweep over those still
        reaches the geometry.
        """

    def measurement_area(self) -> MeasurementArea | None:
        """Where this scenario's results count; ``None`` measures everywhere.

        Belongs to the scenario because it is part of the design — "fill this disc, measure inside
        that one" *is* the traffic scenario — and coupling them makes a mismatched pair
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

        True for a bounded engagement, where the running minimum separation is a meaningful
        importance function. An open-ended traffic *stream* would return False: over hours of
        arrivals "at least one loss" stops being rare, so the running minimum stops discriminating
        between particles and splitting returns a number near 1 after a great deal of compute.
        Declared here so the combination can fail at declaration time rather than after the run.
        """
        return True

    def with_pins(self, **pins: object) -> Scenario:
        """This scenario with geometry parameters pinned, for a sweep over one of them.

        The default refuses pins it cannot honour, which is what turns ``dpsi=Sweep([...])`` over a
        ring into an error at declaration rather than a swept axis that quietly does nothing.
        """
        if pins:
            raise ValueError(
                f"{type(self).__name__} has no geometry slots, so it cannot pin "
                f"{sorted(pins)}. Those slots belong to the pairwise encounter."
            )
        return self


def _per_aircraft_speeds(speed: float | Sequence[float], n: int) -> list[float]:
    """One cruise speed per aircraft: a scalar applies to all, a sequence gives each its own.

    A mixed fleet needs the sequence form. :data:`~opencdarr.performance.SMALL_FIXEDWING` stalls at
    12 m/s, above the 10 m/s a multirotor normally cruises at, so a single fleet speed either
    stalls one airframe or flies the other well above its real cruise.
    :class:`~opencdarr.fleet.Agent` already refuses an out-of-envelope speed, so the mismatch fails
    where it is written; this is what lets the *scenario* express the difference instead of the
    caller working around it — and it makes the speed difference a subject of study, which is what
    a GA-versus-UAS encounter is.

    A length mismatch is refused rather than recycled or truncated: either would fly a fleet nobody
    declared, and the run would look entirely normal afterwards.
    """
    if isinstance(speed, int | float):
        return [float(speed)] * n
    speeds = [float(v) for v in speed]
    if len(speeds) != n:
        raise ValueError(
            f"speed has {len(speeds)} entries but the scenario places {n} aircraft"
        )
    return speeds


