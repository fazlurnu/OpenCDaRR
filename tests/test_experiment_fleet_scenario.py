"""A fleet study through ``run_experiment`` — on **both** backends, from one declaration.

The encounter model is a declared component (``Methods.scenario``), so the fleet size is a property
of the scenario rather than of the estimator. Both backends build their encounter from that one
factory: MC hands it to :func:`~opencdarr.estimator.estimate_p_los`, IPS wraps its agents in the
initial particle. Before this, ``_run_ips`` opened ``sample_pairwise`` itself, so an IPS fleet
study was unreachable from here and the campaign could not compare the two backends on anything
but a pair.
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from opencdarr.config import Config
from opencdarr.estimator import EncounterBuilder, MonteCarloEstimate
from opencdarr.experiment import IPS, MC, Fixed, run_experiment
from opencdarr.fleet import Agent
from opencdarr.ips import RareEventEstimate
from opencdarr.kinematics import Kinematics
from opencdarr.performance import M600, Performance
from opencdarr.scenario import sample_pairwise
from tests.test_experiment import _base, _methods

_FOUR = {"pos_ci95": Fixed(40.0), "vel_ci95": Fixed(4.0)}


def two_pairs(
    perf: Performance | None = None,
    *,
    kinematics: Kinematics | None = None,
    airframes: object | None = None,
) -> EncounterBuilder:
    """A four-aircraft scenario factory: two independent crossing pairs, ~34 km apart.

    Deliberately *not* pairwise, and deliberately separable — the two pairs never interact, so
    ``A = 2 K`` holds exactly and the per-aircraft rate must sit below the per-run one.
    """
    def build(rng: np.random.Generator, config: Config) -> list[Agent]:
        agents: list[Agent] = []
        for shift in (0.0, 0.5):
            own, intr = sample_pairwise(
                rng, speed=config.scenario.speed, dcpa_max=config.scenario.dcpa_max,
                tlos=config.scenario.tlos, rpz=config.conflict.rpz,
                pos_ci95=config.scenario.pos_ci95, vel_ci95=config.scenario.vel_ci95,
            )
            agents += [Agent(replace(s, lon=s.lon + shift), perf or M600, kinematics=kinematics)
                       for s in (own, intr)]
        return agents

    return build


def test_a_fleet_scenario_runs_on_both_backends_from_one_declaration() -> None:
    """The declaration names the scenario once; MC and IPS both fly it, four aircraft each.

    This is what the validation campaign needs: the same condition, both estimators, a geometry
    that is not a pair. The two numbers are not compared here — that is the campaign's job, and it
    needs the tail leg before the per-aircraft ones are comparable.
    """
    methods = _methods(scenario=two_pairs)

    mc = run_experiment(_FOUR, methods=methods, backend=MC(n_encounters=12),
                        base_config=_base(), seed=0).cell()
    assert isinstance(mc, MonteCarloEstimate)
    assert mc.sum_n == 4 * mc.n_encounters       # N came from the scenario, not the estimator
    assert mc.p_los_ac <= mc.p_los_run           # per-aircraft never exceeds per-run

    ips = run_experiment(_FOUR, methods=methods, backend=IPS(
        shells=[70.0, 60.0, 50.0], n_particles=16, reps=2), base_config=_base(), seed=0).cell()
    assert isinstance(ips, RareEventEstimate)
    assert 0.0 <= ips.prob <= 1.0                # IPS flew the same four-aircraft scenario


def test_the_scenario_is_sweepable_like_any_other_component() -> None:
    """``scenario`` is a ``Methods`` field, so an axis overrides it per condition.

    That is the whole reason it is spelled as a component rather than as another geometry slot: the
    fleet size becomes an ordinary swept axis, through machinery that already existed.
    """
    from opencdarr.estimator import pairwise
    from opencdarr.experiment import Sweep

    res = run_experiment(
        {**_FOUR, "scenario": Sweep([2, 4], build=lambda n: pairwise if n == 2 else two_pairs,
                                    name="n_aircraft")},
        methods=_methods(), backend=MC(n_encounters=8), base_config=_base(), seed=0,
    )
    rows = res.records()
    assert [r["n_aircraft"] for r in rows] == [2, 4]        # the axis labels the table
    two, four = (res.cell(n_aircraft=k) for k in (2, 4))
    assert two.sum_n == 2 * two.n_encounters
    assert four.sum_n == 4 * four.n_encounters
