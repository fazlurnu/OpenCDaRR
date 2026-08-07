"""The methods bundle — the CDR stack and the environment a declaration runs on."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import CommunicationModel, NavigationModel, SurveillanceModel
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.fleet import Airframe
from opencdarr.kinematics import Kinematics
from opencdarr.performance import Performance
from opencdarr.scenario import PairwiseEncounter, Scenario
from opencdarr.wind import NO_WIND, WindField


@dataclass(frozen=True)
class Methods:
    """The CDR stack and the environment it runs in — everything that is not a swept parameter.

    A plain bundle of defaults, not a new abstraction: each field is passed straight through to the
    estimator, and any of them can be overridden per condition by declaring the same name as an
    axis. ``perf`` defaults are the caller's business; the airframe defaults to the fleet's
    multirotor when ``kinematics`` is ``None`` (ADR 0007).

    **A mixed fleet is spelled ``airframes``**: one :class:`~opencdarr.fleet.Airframe` per aircraft
    (ownship first), replacing ``perf``/``kinematics`` rather than joining them. The single fields
    are the right shape when every aircraft is the same airframe, which is the ordinary case;
    ``airframes`` is how a declaration says otherwise, and it is what lets a multirotor-versus-
    fixed-wing encounter be *swept* rather than only run once through
    :func:`~opencdarr.fleet.run_fleet`. Bundling ``perf`` with ``kinematics`` also makes a
    mismatched pair unrepresentable in the declaration instead of caught a layer down.

    ``scenario`` is the **encounter model** — a :class:`~opencdarr.scenario.Scenario`, which turns
    one seed into one fleet. It defaults to
    :class:`~opencdarr.scenario.PairwiseEncounter`, so an undeclared experiment is the two-aircraft
    study it always was. Because it is a field of this bundle it is swept like any other component
    — ``scenario=Sweep([4, 8], build=lambda n: CrossingRing(n=n), name="n")`` is a fleet-size axis
    — and because *both* backends build their encounter from it, a ring or a traffic sample reaches
    MC and IPS from one declaration.

    The scenario also carries its own measurement area, so "fill this disc, measure inside that
    one" is a single declaration rather than two that can disagree. Nothing here takes an area
    separately.

    ``wind`` is the odd one out: it is a steady environment input rather than a pluggable model, so
    it has no ABC and lives here for the same reason ``perf`` does — the run needs it and no
    scenario field carries it. It reaches **both** backends; it previously reached neither,
    because :func:`~opencdarr.estimate.montecarlo.estimate_p_los` and
    :func:`~opencdarr.fleet.build_env` accept a ``wind`` this module never passed.
    """

    detector: ConflictDetector
    resolver: ConflictResolver | None = None
    recovery: RecoveryCriterion | None = None
    navigation: NavigationModel | None = None
    communication: CommunicationModel | None = None
    surveillance: SurveillanceModel | None = None
    kinematics: Kinematics | None = None
    perf: Performance | None = None
    wind: WindField = NO_WIND
    airframes: Sequence[Airframe] | None = None
    scenario: Scenario = PairwiseEncounter()

    def __post_init__(self) -> None:
        # One spelling for one thing. ``perf``/``kinematics`` say "every aircraft is this
        # airframe"; ``airframes`` says "here is each one". Both at once has no meaning, so it is
        # refused rather than silently resolved in some order.
        if self.airframes is not None and (self.perf is not None or self.kinematics is not None):
            raise ValueError(
                "give either airframes=[...] (one per aircraft) or perf=/kinematics= (one shared "
                "airframe), not both"
            )
