"""Random traffic: released on the ring, and homogeneous because of *how* it is released.

Groot et al.'s Eq. 8 draws a heading uniformly and then enters at ``heading + 180 + asin(x)``,
``x ~ U(-1, 1)``. The claim that buys is not about where the aircraft start — they all start on the
ring — but about the *tracks*: the perpendicular offset of each track from the centre comes out
at exactly ``R x``, so the tracks are spread uniformly across the diameter. Entering at a bearing
drawn uniformly around the rim instead crowds the tracks toward the edge, and a density study
built on that reports the crowding.

The release ring is also what makes the measured disc mean anything. Aircraft are packed onto a
one-dimensional circle at t = 0, so pairs of them start close together — but outside the measured
disc, so nothing is counted until each has crossed the annulus and has a history of having been
separated. That is asserted here, because it is the whole reason the two radii differ.
"""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

from opencdarr import geo
from opencdarr.measurement import Disc
from opencdarr.scenario import aircraft_for_density, measurement_area, random_traffic

_R = 1000.0
_C = (52.0, 4.0)
_RPZ = 50.0


def _radii(fleet) -> np.ndarray:
    return np.array([geo.qdrdist(_C[0], _C[1], s.lat, s.lon)[1] for s, _ in fleet])


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(_C[0], _C[1], lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _offsets(fleet) -> np.ndarray:
    """Each track's signed perpendicular distance from the centre — Eq. 8's actual subject."""
    out = []
    for state, _ in fleet:
        east, north = _enu(state.lat, state.lon)
        trk = math.radians(state.trk)
        out.append(east * math.cos(trk) - north * math.sin(trk))
    return np.array(out)


def test_every_aircraft_is_released_on_the_ring() -> None:
    """Not spread through the disc: the fleet is a cohort entering together from the boundary."""
    fleet = random_traffic(np.random.default_rng(0), 200, radius=_R)
    assert len(fleet) == 200
    assert _radii(fleet) == pytest.approx(_R, rel=1e-6)


def test_every_aircraft_flies_into_the_disc() -> None:
    """``asin(x)`` is at most a quarter turn off the inward radial, so no draw is outbound."""
    for state, _ in random_traffic(np.random.default_rng(1), 400, radius=_R):
        east, north = _enu(state.lat, state.lon)
        trk = math.radians(state.trk)
        # the chord it will fly: positive means it enters the disc
        chord = -2.0 * (east * math.sin(trk) + north * math.cos(trk))
        assert chord >= -1e-6


def test_the_tracks_are_spread_uniformly_across_the_diameter() -> None:
    """The rule's claim, and the reason it is not simply "draw a bearing".

    The offsets are ``R x`` with ``x ~ U(-1, 1)``, so their mean is 0 and their standard deviation
    is ``R / sqrt(3)``. Drawing the entry bearing uniformly around the rim instead gives offsets
    with a standard deviation near ``R / sqrt(2)`` — far enough apart that this rejects it.
    """
    offsets = _offsets(random_traffic(np.random.default_rng(2), 6000, radius=_R)) / _R
    assert offsets.mean() == pytest.approx(0.0, abs=0.03)
    assert offsets.std() == pytest.approx(1 / math.sqrt(3), abs=0.02)
    # and the quartiles of a uniform law, which a crowded-at-the-edge rule would not match
    for q, expected in ((0.25, -0.5), (0.5, 0.0), (0.75, 0.5)):
        assert np.quantile(offsets, q) == pytest.approx(expected, abs=0.05)


def test_headings_are_uniform() -> None:
    """No preferred direction: the mean unit heading vector is near zero."""
    fleet = random_traffic(np.random.default_rng(3), 4000, radius=_R)
    trk = np.radians([s.trk for s, _ in fleet])
    assert math.hypot(np.cos(trk).mean(), np.sin(trk).mean()) < 0.05


def test_the_same_seed_gives_the_same_traffic() -> None:
    """Drawn, not placed — so reproducibility is a property of the seed, and must be exact."""
    def key(fleet):
        return [(s.lat, s.lon, s.trk, s.gs) for s, _ in fleet]

    assert key(random_traffic(np.random.default_rng(7), 50, radius=_R)) == key(
        random_traffic(np.random.default_rng(7), 50, radius=_R))
    assert key(random_traffic(np.random.default_rng(7), 50, radius=_R)) != key(
        random_traffic(np.random.default_rng(8), 50, radius=_R))


def test_each_aircraft_heads_away_from_where_it_is() -> None:
    """The goal lies along the aircraft's own track, beyond the disc — it crosses and leaves."""
    for state, goal in random_traffic(np.random.default_rng(4), 60, radius=_R):
        qdr, dist = geo.qdrdist(state.lat, state.lon, goal[0], goal[1])
        assert qdr % 360.0 == pytest.approx(state.trk % 360.0, abs=0.5)
        assert dist > _R


def test_speeds_can_be_per_aircraft() -> None:
    fleet = random_traffic(np.random.default_rng(5), 3, radius=_R, speed=[9.0, 10.0, 11.0])
    assert [s.gs for s, _ in fleet] == [9.0, 10.0, 11.0]


# --- the two areas ----------------------------------------------------------------------------


def test_the_measured_disc_is_smaller_than_the_release_ring() -> None:
    """The annulus between them is flown but not counted — that is the point of having two."""
    area = measurement_area(_R, _C)
    assert isinstance(area, Disc)
    assert area.radius == pytest.approx(_R * 1.35 / 1.62)
    assert area.radius < _R


def test_nothing_is_counted_at_the_moment_of_release() -> None:
    """The design in one assertion: aircraft start close together, and none of it counts.

    Packing n aircraft onto a circle puts pairs of them within ``rpz`` at t = 0 — at 10 AC/km^2
    that is every draw. They have never been separated, so counting those would be measuring the
    release rule. They are all on the ring, outside the measured disc, so the gate excludes them
    without needing to know anything about time.
    """
    area = measurement_area(_R, _C)
    for density in (5.0, 10.0, 25.0):
        n = aircraft_for_density(density, _R)
        raw = counted = 0
        for seed in range(40):
            fleet = random_traffic(np.random.default_rng(seed), n, radius=_R)
            pos = [(s.lat, s.lon) for s, _ in fleet]
            inside = [area.contains(*p) for p in pos]
            gaps = {(i, j): geo.qdrdist(*pos[i], *pos[j])[1]
                    for i, j in itertools.combinations(range(n), 2)}
            raw += any(g < _RPZ for g in gaps.values())
            counted += any(g < _RPZ and inside[i] and inside[j] for (i, j), g in gaps.items())
        assert raw > 0, "the ring really does start aircraft close together"
        assert counted == 0, "and the measured disc excludes every one of them"


def test_density_is_defined_over_the_simulation_disc() -> None:
    """As in the paper: density describes the traffic generated, not the region measured."""
    assert aircraft_for_density(10.0, _R) == round(10.0 * math.pi)
    assert aircraft_for_density(0.0, _R) == 2       # never fewer than a pair to have an encounter
