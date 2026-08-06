"""``crossing_ring`` — the ring whose routes cross the centre at *every* fleet size.

``swap_ring`` sends each aircraft to another aircraft's **start**, the one ``n // 2`` places round.
At even ``n`` that start is the antipode, so every route is a diameter and the fleet meets in the
middle. At odd ``n`` it is not, and the routes miss the centre — by 750 m at ``n = 3``, on a 1500 m
ring.

That matters for a fleet-size sweep. Stepping ``n`` through 3, 4, 5 on ``swap_ring`` changes the
geometry as well as the size, so any trend in the result mixes the two. ``crossing_ring`` aims at
the antipodal **point** instead, which is a diameter at every ``n``, so the size is the only thing
that varies.
"""

from __future__ import annotations

import math

import pytest

from opencdarr import geo
from opencdarr.scenario import crossing_ring, swap_ring

_R = 1500.0
_LAT0, _LON0 = 52.0, 4.0


def _enu(lat: float, lon: float) -> tuple[float, float]:
    """(east, north) in metres from the ring centre."""
    qdr, dist = geo.qdrdist(_LAT0, _LON0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _miss_distance(state, target) -> float:
    """How far the centre is from the straight route this aircraft flies [m]."""
    x1, y1 = _enu(state.lat, state.lon)
    x2, y2 = _enu(*target)
    span = math.hypot(x2 - x1, y2 - y1)
    return abs(x1 * y2 - x2 * y1) / span      # distance from the origin to the line


@pytest.mark.parametrize("n", [4, 6, 8])
def test_at_even_n_the_two_rings_are_the_same_fleet(n: int) -> None:
    """At even ``n`` the ``n // 2`` neighbour *is* the antipode, so the two builders agree.

    Asserted rather than assumed: it is what makes ``crossing_ring`` a generalisation of
    ``swap_ring`` and not a second, subtly different geometry.
    """
    crossing, swap = crossing_ring(n, radius=_R), swap_ring(n, radius=_R)
    for (c_state, c_goal), (s_state, s_goal) in zip(crossing, swap, strict=True):
        assert (c_state.lat, c_state.lon) == (s_state.lat, s_state.lon)
        assert c_state.trk == pytest.approx(s_state.trk, abs=1e-9)
        assert c_goal == pytest.approx(s_goal, abs=1e-9)


@pytest.mark.parametrize("n,expected_miss", [(3, 750.0), (5, 463.5), (7, 333.8)])
def test_at_odd_n_only_crossing_ring_reaches_the_centre(n: int, expected_miss: float) -> None:
    """The reason the geometry exists: ``swap_ring`` misses the centre at odd ``n``.

    The miss is ``R cos(theta / 2)`` for the chord subtending ``theta = 360 (n // 2) / n``, so it
    is 750 m at ``n = 3``. ``crossing_ring`` flies a diameter, so its miss is zero at every ``n``.
    """
    for state, goal in crossing_ring(n, radius=_R):
        assert _miss_distance(state, goal) == pytest.approx(0.0, abs=1.0)

    misses = [_miss_distance(s, g) for s, g in swap_ring(n, radius=_R)]
    assert misses == pytest.approx([expected_miss] * n, rel=2e-3)


def test_the_route_is_a_full_diameter() -> None:
    """Each aircraft starts on the ring and finishes on the far side of it, through the centre."""
    for state, goal in crossing_ring(5, radius=_R):
        assert math.hypot(*_enu(state.lat, state.lon)) == pytest.approx(_R, rel=1e-6)
        assert math.hypot(*_enu(*goal)) == pytest.approx(_R, rel=1e-6)
        span = geo.qdrdist(state.lat, state.lon, goal[0], goal[1])[1]
        assert span == pytest.approx(2 * _R, rel=1e-3)          # a diameter, not a chord


def test_it_takes_a_speed_per_aircraft_like_the_others() -> None:
    speeds = [10.0, 12.0, 14.0]
    assert [s.gs for s, _ in crossing_ring(3, speed=speeds)] == speeds
    with pytest.raises(ValueError, match="2 entries but the scenario places 3"):
        crossing_ring(3, speed=[10.0, 12.0])
