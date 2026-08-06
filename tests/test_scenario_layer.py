"""The :class:`Scenario` layer — one interface over placed and drawn encounters.

A ring is *placed* and ignores its generator; traffic is *drawn* and depends on it entirely. Both
answer ``draw(rng, config)``, which is what lets one estimator run either without knowing which it
has. The optional methods carry the facts a scenario knows about itself — how many aircraft, where
results count, whether splitting is meaningful — so the runner never has to be told them separately
and cannot be told something that disagrees.
"""

from __future__ import annotations

import numpy as np
import pytest

from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.measurement import Disc
from opencdarr.scenario import (
    ConvergingRing,
    CrossingRing,
    PairwiseEncounter,
    RandomTraffic,
    Scenario,
    SwapRing,
)


def _config(speed: float = 10.0, pos_ci95: float = 0.0, vel_ci95: float = 0.0) -> Config:
    return Config(
        seed=0, n_encounters=1,
        scenario=ScenarioConfig("M600", speed, 100.0, 60.0, pos_ci95, vel_ci95),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 300.0, 10.0),
    )


_ALL: list[Scenario] = [
    PairwiseEncounter(),
    SwapRing(n=4), CrossingRing(n=4), ConvergingRing(n=4),
    RandomTraffic(n=6),
]


@pytest.mark.parametrize("scenario", _ALL, ids=lambda s: type(s).__name__)
def test_every_scenario_draws_the_fleet_it_declares(scenario: Scenario) -> None:
    """``size()`` and the length of a draw must agree — the runner trusts the first."""
    fleet = scenario.draw(np.random.default_rng(0), _config())
    assert len(fleet) == scenario.size()
    assert all(isinstance(goal, tuple) or goal is None for _, goal in fleet)


@pytest.mark.parametrize("scenario", _ALL, ids=lambda s: type(s).__name__)
def test_every_scenario_reads_its_speed_from_the_config(scenario: Scenario) -> None:
    """A sweep over ``speed`` has to reach the geometry, so it is read rather than hard-coded."""
    fleet = scenario.draw(np.random.default_rng(0), _config(speed=13.0))
    assert all(state.gs == pytest.approx(13.0) for state, _ in fleet)


@pytest.mark.parametrize("scenario", _ALL, ids=lambda s: type(s).__name__)
def test_every_scenario_carries_the_declared_accuracies(scenario: Scenario) -> None:
    """A navigation model reads ``pos_ci95`` off the aircraft, so the scenario must put it there.

    Left at zero the fleet flies *noiselessly* whatever CNS stack is declared: the run is
    deterministic, ``P(LoS)`` collapses to 0 or 1, and a swept noise axis silently does nothing.
    That is a whole campaign of meaningless rows, and nothing in the output says so — which is why
    it is asserted for every scenario rather than for the one that happened to be written first.
    """
    fleet = scenario.draw(np.random.default_rng(0), _config(pos_ci95=17.0, vel_ci95=1.7))
    assert all(state.pos_ci95 == pytest.approx(17.0) for state, _ in fleet)
    assert all(state.vel_ci95 == pytest.approx(1.7) for state, _ in fleet)


def test_a_placed_scenario_ignores_the_generator_and_a_drawn_one_does_not() -> None:
    """The asymmetry the interface hides: same call, deterministic one side, sampled the other."""
    ring = CrossingRing(n=4)
    a = ring.draw(np.random.default_rng(1), _config())
    b = ring.draw(np.random.default_rng(2), _config())
    assert [s.lat for s, _ in a] == [s.lat for s, _ in b]        # placed: the seed is irrelevant

    traffic = RandomTraffic(n=20)
    c = traffic.draw(np.random.default_rng(1), _config())
    d = traffic.draw(np.random.default_rng(2), _config())
    assert [s.lat for s, _ in c] != [s.lat for s, _ in d]        # drawn: the seed is everything


def test_only_traffic_declares_a_measurement_area() -> None:
    """``None`` means measure everywhere, which is right for a placed engagement."""
    for scenario in (PairwiseEncounter(), SwapRing(n=4), CrossingRing(n=4), ConvergingRing(n=4)):
        assert scenario.measurement_area() is None

    area = RandomTraffic(n=6, radius=1000.0).measurement_area()
    assert isinstance(area, Disc)
    assert area.radius == pytest.approx(1000.0 * 1.35 / 1.62)


def test_a_pairwise_encounter_has_no_destination() -> None:
    """The geometry *is* the experiment, so a goal would add a manoeuvre nobody asked for."""
    fleet = PairwiseEncounter().draw(np.random.default_rng(0), _config())
    assert [goal for _, goal in fleet] == [None, None]


def test_geometry_pins_reach_the_sampler_and_are_checked() -> None:
    """``with_pins`` is what makes ``dpsi=Sweep([...])`` a real axis rather than a silent no-op."""
    pinned = PairwiseEncounter().with_pins(dpsi=90.0, dcpa=0.0)
    assert (pinned.dpsi, pinned.dcpa) == (90.0, 0.0)

    fleet = pinned.draw(np.random.default_rng(0), _config())
    own, intr = (s for s, _ in fleet)
    assert (intr.trk - own.trk) % 360.0 == pytest.approx(90.0, abs=1e-6)

    with pytest.raises(ValueError, match="not a pairwise geometry slot"):
        PairwiseEncounter().with_pins(radius=1000.0)


def test_a_ring_refuses_pairwise_pins_rather_than_ignoring_them() -> None:
    """Declaring ``dpsi`` over a ring is a mistake, and it fails instead of doing nothing."""
    with pytest.raises(ValueError, match="no geometry slots"):
        SwapRing(n=4).with_pins(dpsi=90.0)
    assert SwapRing(n=4).with_pins() == SwapRing(n=4)      # no pins is a no-op


def test_traffic_takes_a_density_or_a_count_but_not_both() -> None:
    """Two spellings of one number: allowing both is an ambiguity nobody can resolve later."""
    assert RandomTraffic(density=10.0, radius=1000.0).size() == round(10.0 * np.pi)
    assert RandomTraffic(n=7).size() == 7
    for bad in ({}, {"density": 10.0, "n": 7}):
        with pytest.raises(ValueError, match="either density"):
            RandomTraffic(**bad)


def test_splitting_is_supported_by_default() -> None:
    """Every scenario here is a bounded engagement, so the running minimum is meaningful."""
    assert all(s.supports_splitting() for s in _ALL)
