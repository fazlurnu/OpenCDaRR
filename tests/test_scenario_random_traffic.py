"""Random traffic: the entry rule has to produce *homogeneous* traffic, or it is not the rule.

The whole reason Groot et al.'s Eq. 8 exists is that the obvious alternative — entry bearing drawn
uniformly around the rim — is edge-heavy: a chord entering at a shallow angle spends almost no time
inside, so the aircraft that are present bunch near the boundary. That bias is what these tests
measure, because a density study built on biased traffic reports the bias.

Homogeneous over an area means the radial positions satisfy ``r^2 ~ U(0, R^2)``, not ``r ~ U(0,R)``
— there is more area further out.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from opencdarr import geo
from opencdarr.measurement import Disc
from opencdarr.scenario import aircraft_for_density, measurement_area, random_traffic

_R = 1000.0
_C = (52.0, 4.0)


def _radii(fleet) -> np.ndarray:
    """Each aircraft's distance from the disc centre [m]."""
    return np.array([geo.qdrdist(_C[0], _C[1], s.lat, s.lon)[1] for s, _ in fleet])


def test_every_aircraft_starts_inside_the_disc() -> None:
    fleet = random_traffic(np.random.default_rng(0), 200, radius=_R)
    assert len(fleet) == 200
    assert _radii(fleet).max() <= _R + 1.0


def test_the_traffic_is_homogeneous_over_the_area() -> None:
    """``r^2`` uniform, which is what "homogeneous over the disc" means.

    Checked on the mean of ``(r/R)^2``: exactly 1/2 for area-uniform traffic. Drawing the entry
    bearing uniformly around the rim instead — the obvious alternative — measures 0.556 in steady
    state, so the tolerance here is tight enough to reject that rule rather than tolerate it.
    """
    fleet = random_traffic(np.random.default_rng(1), 4000, radius=_R)
    normalised = (_radii(fleet) / _R) ** 2
    assert normalised.mean() == pytest.approx(0.5, abs=0.03)

    # and the quartiles land where an area-uniform law puts them: r = R*sqrt(q)
    for q in (0.25, 0.5, 0.75):
        assert np.quantile(_radii(fleet), q) / _R == pytest.approx(math.sqrt(q), abs=0.04)


def test_headings_are_uniform() -> None:
    """No preferred direction: the mean unit heading vector is near zero."""
    fleet = random_traffic(np.random.default_rng(2), 4000, radius=_R)
    trk = np.radians([s.trk for s, _ in fleet])
    assert math.hypot(np.cos(trk).mean(), np.sin(trk).mean()) < 0.05


def test_the_same_seed_gives_the_same_traffic() -> None:
    """Drawn, not placed — so reproducibility is a property of the seed, and must be exact."""
    a = random_traffic(np.random.default_rng(7), 50, radius=_R)
    b = random_traffic(np.random.default_rng(7), 50, radius=_R)
    key = lambda fleet: [(s.lat, s.lon, s.trk, s.gs) for s, _ in fleet]  # noqa: E731
    assert key(a) == key(b)

    other = random_traffic(np.random.default_rng(8), 50, radius=_R)
    assert [s.lat for s, _ in a] != [s.lat for s, _ in other]


def test_each_aircraft_heads_away_from_where_it_is() -> None:
    """The goal lies along the aircraft's own track, beyond the disc — it crosses and leaves."""
    for state, goal in random_traffic(np.random.default_rng(3), 60, radius=_R):
        qdr, dist = geo.qdrdist(state.lat, state.lon, goal[0], goal[1])
        assert qdr % 360.0 == pytest.approx(state.trk % 360.0, abs=0.5)
        assert dist > _R          # outside the disc, so it is never arrived at


def test_speeds_can_be_per_aircraft() -> None:
    fleet = random_traffic(np.random.default_rng(4), 3, radius=_R, speed=[9.0, 10.0, 11.0])
    assert [s.gs for s, _ in fleet] == [9.0, 10.0, 11.0]


# --- the two areas ----------------------------------------------------------------------------


def test_the_measured_disc_is_smaller_than_the_simulated_one() -> None:
    """The annulus between them is flown but not counted — that is the point of having two."""
    area = measurement_area(_R, _C)
    assert isinstance(area, Disc)
    assert area.radius == pytest.approx(_R * 1.35 / 1.62)
    assert area.radius < _R
    # an aircraft just inside the simulation disc is outside the measured one
    assert not area.contains(*geo.forward(_C[0], _C[1], 0.0, 0.99 * _R))


def test_density_is_defined_over_the_simulation_disc() -> None:
    """As in the paper: density describes the traffic generated, not the region measured."""
    # a 1000 m disc is pi km^2, so 10 AC/km^2 is about 31 aircraft
    assert aircraft_for_density(10.0, _R) == round(10.0 * math.pi)
    assert aircraft_for_density(0.0, _R) == 2       # never fewer than a pair to have an encounter
