"""A fleet study through ``run_experiment`` — both backends, from one declaration.

The encounter model is a declared component (``Methods.scenario``), so the fleet size is a property
of the scenario rather than of the estimator. Both backends build their encounter from that one
object: MC hands it to :func:`~opencdarr.estimate.montecarlo.estimate_p_los`, IPS wraps its
agents in the
initial particle. Before this, ``_run_ips`` opened ``sample_pairwise`` itself, so an IPS fleet
study was unreachable from here and the campaign could not compare the two backends on anything
but a pair.

The scenario also carries its own measurement area, so nothing here takes one separately — which
is what stops a caller filling one region and measuring another.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from opencdarr.config import Config
from opencdarr.estimate.ips import RareEventEstimate
from opencdarr.estimate.montecarlo import MonteCarloEstimate
from opencdarr.experiment import IPS, MC, Fixed, Sweep, run_experiment
from opencdarr.measurement import Disc, MeasurementArea
from opencdarr.scenario import ConvergingRing, CrossingRing, RandomTraffic, Scenario
from opencdarr.scenario.base import FleetScenario
from tests.test_experiment import _base, _methods

_NOISY = {"pos_ci95": Fixed(40.0), "vel_ci95": Fixed(4.0)}
_SHELLS = [70.0, 60.0, 50.0]


def test_a_fleet_scenario_runs_on_both_backends_from_one_declaration() -> None:
    """The declaration names the scenario once; MC and IPS both fly it, four aircraft each.

    This is what the validation campaign needs: the same condition, both estimators, a geometry
    that is not a pair.
    """
    methods = _methods(scenario=ConvergingRing(n=4, radius=600.0))

    mc = run_experiment(_NOISY, methods=methods, backend=MC(n_encounters=8),
                        base_config=_base(), seed=0).cell()
    assert isinstance(mc, MonteCarloEstimate)
    assert mc.sum_n == 4 * mc.n_encounters       # N came from the scenario, not the estimator
    assert mc.p_los_ac <= mc.p_los_run

    ips = run_experiment(_NOISY, methods=methods, backend=IPS(
        shells=_SHELLS, n_particles=12, reps=2), base_config=_base(), seed=0).cell()
    assert isinstance(ips, RareEventEstimate)
    assert 0.0 <= ips.p_los_run <= 1.0


def test_the_scenario_is_sweepable_like_any_other_component() -> None:
    """``scenario`` is a ``Methods`` field, so an axis overrides it per condition.

    That is why it is spelled as a component rather than as another geometry slot: the fleet size
    becomes an ordinary swept axis, through machinery that already existed.
    """
    res = run_experiment(
        {**_NOISY, "scenario": Sweep([3, 5], build=lambda n: CrossingRing(n=n, radius=600.0),
                                     name="n_aircraft")},
        methods=_methods(), backend=MC(n_encounters=6), base_config=_base(), seed=0,
    )
    assert [r["n_aircraft"] for r in res.records()] == [3, 5]
    three, five = (res.cell(n_aircraft=k) for k in (3, 5))
    assert three.sum_n == 3 * three.n_encounters
    assert five.sum_n == 5 * five.n_encounters


def test_a_pairwise_declaration_is_unchanged() -> None:
    """The default is still the two-aircraft study, and its pins still reach the sampler."""
    res = run_experiment({**_NOISY, "dpsi": Fixed(90.0), "dcpa": Fixed(0.0)},
                         methods=_methods(), backend=MC(n_encounters=8),
                         base_config=_base(), seed=0).cell()
    assert res.sum_n == 2 * res.n_encounters
    assert res.p_los_ac == res.p_los_run        # N = 2, so the two normalisations coincide


# --- the measurement area arrives with the scenario --------------------------------------------


@dataclass(frozen=True)
class _RingIn(Scenario):
    """A ring that measures only inside ``area`` — the same geometry, two measured regions."""

    area: MeasurementArea | None = None

    def draw(self, rng: np.random.Generator, config: Config) -> FleetScenario:
        return ConvergingRing(n=4, radius=600.0).draw(rng, config)

    def size(self) -> int:
        return 4

    def measurement_area(self) -> MeasurementArea | None:
        return self.area


def test_the_scenarios_measurement_area_reaches_the_run() -> None:
    """Nothing passes an area to ``run_experiment`` — the scenario carries it, and it takes effect.

    The same converging ring is flown twice. Measured everywhere it loses separation; measured only
    inside a 1 m disc nothing is counted at all, because no pair is ever both-inside. The geometry
    is identical, so any difference is the area doing its job.
    """
    kw = dict(backend=MC(n_encounters=6), base_config=_base(), seed=0)

    everywhere = run_experiment(_NOISY, methods=_methods(scenario=_RingIn(area=None)), **kw).cell()
    nowhere = run_experiment(
        _NOISY, methods=_methods(scenario=_RingIn(area=Disc(centre=(0.0, 0.0), radius=1.0))), **kw
    ).cell()

    assert everywhere.p_los_run > 0.0            # the ring really does lose separation
    assert nowhere.p_los_run == 0.0              # ... and none of it is inside the measured disc
    assert nowhere.sum_k == 0 and nowhere.sum_a == 0


def test_traffic_brings_its_own_two_areas() -> None:
    """``RandomTraffic`` declares the measured disc, not just the traffic that fills the rest."""
    traffic = RandomTraffic(density=8.0, radius=800.0)
    area = traffic.measurement_area()
    assert isinstance(area, Disc)
    assert area.radius == pytest.approx(800.0 * 1.35 / 1.62)

    cell = run_experiment(_NOISY, methods=_methods(scenario=traffic),
                          backend=MC(n_encounters=4), base_config=_base(), seed=0).cell()
    assert cell.sum_n == traffic.size() * cell.n_encounters
