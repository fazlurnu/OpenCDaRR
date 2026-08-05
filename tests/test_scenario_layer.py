"""The declarative scenario layer: one `Scenario` per experiment family, both backends.

The property that matters most is the reduction — a `PairwiseEncounter` through the new path has
to reproduce what `run_experiment` did before scenarios existed, encounter for encounter, or every
published pairwise number silently moves. The rest of the file checks that the layer refuses the
mistakes it was built to refuse rather than quietly doing something else.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from opencdarr import scenario as sc
from opencdarr.cd import StateBased
from opencdarr.cns.navigation import GnssNavigation
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.estimator import estimate_ipr, estimate_ipr_over
from opencdarr.experiment import IPS, MC, Fixed, Ladder, Methods, Sweep, run_experiment
from opencdarr.ips import ladder_from_record
from opencdarr.performance import M600
from opencdarr.rng import generator, root_seed_sequence

_CFG = Config(
    seed=0,
    n_encounters=12,
    scenario=ScenarioConfig(aircraft_type="M600", speed=10.0, dcpa_max=50.0, tlos=60.0,
                            pos_ci95=10.0, vel_ci95=1.0),
    conflict=ConflictConfig(rpz=50.0, t_lookahead=30.0),
    methods=MethodsConfig(detection="statebased", resolution="mvp", recovery="pastcpa",
                          margin=1.05, bouncing_guard=True),
    simulation=SimulationConfig(dt=0.5, t_max=300.0, done_timeout=10.0),
)
_METHODS = Methods(detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(),
                   navigation=GnssNavigation(), perf=M600)


def _with_n(cfg: Config, n: int) -> Config:
    return dataclasses.replace(cfg, n_encounters=n)


def test_pairwise_scenario_reproduces_the_open_coded_estimator() -> None:
    """The reduction: the scenario path and the historical pairwise loop agree exactly."""
    direct = estimate_ipr(_CFG, M600, StateBased(), MVP(1.05), PastCPA(), GnssNavigation(),
                          dpsi=90.0)
    through = estimate_ipr_over(sc.PairwiseEncounter(dpsi=90.0), _CFG, M600, StateBased(),
                                MVP(1.05), PastCPA(), GnssNavigation())
    assert direct.min_seps == through.min_seps
    assert (direct.n_los, direct.n_conflict) == (through.n_los, through.n_conflict)


def test_geometry_slots_still_declare_a_pairwise_sweep() -> None:
    """`{"dpsi": Sweep([...])}` keeps meaning what it did before scenarios existed."""
    res = run_experiment({"dpsi": Sweep([45.0, 90.0])}, methods=_METHODS,
                         backend=MC(n_encounters=8), base_config=_CFG)
    assert res.axes == ("dpsi",)
    assert len(res.results) == 2
    pinned = estimate_ipr_over(sc.PairwiseEncounter(dpsi=45.0), _with_n(_CFG, 8), M600,
                               StateBased(), MVP(1.05), PastCPA(), GnssNavigation())
    assert res.results[0].min_seps == pinned.min_seps


def test_geometry_slot_with_a_fleet_scenario_is_refused_not_ignored() -> None:
    """Declaring dpsi against a ring is a mistake; it must fail rather than be dropped."""
    with pytest.raises(ValueError, match="PairwiseEncounter"):
        run_experiment(
            {"dpsi": Fixed(90.0), "scenario": Fixed(sc.CrossingRing(3))},
            methods=_METHODS, backend=MC(n_encounters=4), base_config=_CFG,
        )


def test_a_fleet_scenario_runs_through_the_same_declaration() -> None:
    """The ring and the traffic disc need no new plumbing — they are just other scenarios."""
    res = run_experiment(
        {"scenario": Sweep([3, 4], name="n_aircraft", build=lambda n: sc.CrossingRing(n))},
        methods=_METHODS, backend=MC(n_encounters=4), base_config=_CFG,
    )
    assert res.axes == ("n_aircraft",)
    assert all(r.n_encounters == 4 for r in res.results)


def test_traffic_scenario_carries_its_own_measurement_area() -> None:
    """The disc travels with the scenario, so it cannot be declared inconsistently."""
    traffic = sc.RandomTraffic(4, r_inner=800.0, r_outer=1000.0)
    area = traffic.measurement_area()
    assert area is not None and area.radius == 800.0
    assert sc.CrossingRing(4).measurement_area() is None
    assert traffic.size() == 4


def test_scenarios_draw_from_the_generator_they_are_given() -> None:
    """Traffic is a distribution; the ring is deterministic. Both honour the seed contract."""
    a = sc.RandomTraffic(4).draw(generator(root_seed_sequence(1)), _CFG)
    b = sc.RandomTraffic(4).draw(generator(root_seed_sequence(1)), _CFG)
    c = sc.RandomTraffic(4).draw(generator(root_seed_sequence(2)), _CFG)
    assert [s.lat for s, _ in a] == [s.lat for s, _ in b]
    assert [s.lat for s, _ in a] != [s.lat for s, _ in c]

    ring_a = sc.CrossingRing(4).draw(generator(root_seed_sequence(1)), _CFG)
    ring_b = sc.CrossingRing(4).draw(generator(root_seed_sequence(9)), _CFG)
    assert [s.lat for s, _ in ring_a] == [s.lat for s, _ in ring_b]  # seed-independent


def test_ladder_descends_to_the_boundary_and_no_further() -> None:
    rng = np.random.default_rng(0)
    shells = ladder_from_record(50.0 + rng.gamma(2.0, 12.0, size=3000), 50.0, step=1.0)
    assert shells[-1] == 50.0
    assert all(b < a for a, b in zip(shells, shells[1:], strict=False))
    with pytest.raises(ValueError, match="no finite"):
        ladder_from_record([float("inf")] * 5, 50.0)


def test_ips_accepts_a_pilot_derived_ladder() -> None:
    """A Ladder means each condition gets shells built from its own pilot run."""
    res = run_experiment(
        {"scenario": Fixed(sc.PairwiseEncounter(dpsi=90.0))},
        methods=_METHODS,
        backend=IPS(shells=Ladder(pilot=60, step=2.0), n_particles=40, reps=2),
        base_config=_CFG,
    )
    est = res.cell()
    assert est.reps[0].levels[-1] == 50.0        # the ladder ends on the protected zone
    assert len(est.reps[0].levels) > 1
