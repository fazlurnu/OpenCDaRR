"""Measurement areas: the containment gate and the area a density divides by.

Each shape is checked against geometry that is known without the code — a point at a named bearing
and range, a corner of a box, the area of a shape whose formula is on paper. The shapes overlap on
purpose (a square is a rectangle; a rectangle is also expressible as a polygon), so where they
describe the same region they are asserted to agree.
"""

from __future__ import annotations

import math

import pytest

from opencdarr import geo
from opencdarr.measurement import Disc, Polygon, Rectangle

_C = (52.0, 4.0)


def _at(bearing: float, distance: float, origin: tuple[float, float] = _C):
    """The position ``distance`` m from ``origin`` on ``bearing`` degrees."""
    return geo.forward(origin[0], origin[1], bearing, distance)


# --- disc -------------------------------------------------------------------------------------


def test_a_disc_contains_what_is_inside_its_radius() -> None:
    """Exactly spherical: the test is a great-circle distance, so it holds at any bearing."""
    disc = Disc(centre=_C, radius=1000.0)
    for bearing in (0.0, 45.0, 90.0, 180.0, 270.0, 359.0):
        assert disc.contains(*_at(bearing, 990.0))
        assert not disc.contains(*_at(bearing, 1010.0))
    assert disc.contains(*_C)


def test_a_disc_reports_pi_r_squared() -> None:
    assert Disc(centre=_C, radius=1000.0).area() == pytest.approx(math.pi * 1e6)


def test_a_disc_of_zero_or_negative_radius_is_refused() -> None:
    with pytest.raises(ValueError, match="radius must be positive"):
        Disc(centre=_C, radius=0.0)


# --- rectangle --------------------------------------------------------------------------------


def test_a_rectangle_is_east_north_aligned() -> None:
    """2000 m east-west by 1000 m north-south: half-extents are 1000 m and 500 m."""
    box = Rectangle(centre=_C, width=2000.0, height=1000.0)
    assert box.contains(*_at(90.0, 990.0))       # inside the east edge
    assert not box.contains(*_at(90.0, 1010.0))  # outside it
    assert box.contains(*_at(0.0, 490.0))        # inside the north edge
    assert not box.contains(*_at(0.0, 510.0))    # outside it
    assert box.area() == pytest.approx(2.0e6)


def test_a_square_is_a_rectangle_with_equal_sides() -> None:
    """The factory is the only spelling — there is no separate square shape to disagree with it."""
    square = Rectangle.square(_C, 1000.0)
    assert (square.width, square.height) == (1000.0, 1000.0)
    assert square == Rectangle(centre=_C, width=1000.0, height=1000.0)
    assert square.area() == pytest.approx(1.0e6)


def test_a_corner_is_outside_a_disc_that_shares_the_box_half_width() -> None:
    """A sanity check on the two shapes together: the corner is further out than the edge.

    Half-width 500 m, so the corner is 500*sqrt(2) = 707 m from the centre. It is inside the box
    and outside a 600 m disc — which is the difference between the two shapes, made concrete.
    """
    box = Rectangle.square(_C, 1000.0)
    disc = Disc(centre=_C, radius=600.0)
    corner = _at(45.0, 500.0 * math.sqrt(2) - 1.0)
    assert box.contains(*corner)
    assert not disc.contains(*corner)


# --- polygon ----------------------------------------------------------------------------------


def _box_as_polygon(half: float) -> Polygon:
    """The same square as ``Rectangle.square(_C, 2*half)``, written as four corners."""
    return Polygon(vertices=[
        _at(45.0, half * math.sqrt(2)), _at(135.0, half * math.sqrt(2)),
        _at(225.0, half * math.sqrt(2)), _at(315.0, half * math.sqrt(2)),
    ])


def test_a_polygon_agrees_with_the_rectangle_describing_the_same_square() -> None:
    """Two shapes, one region: containment and area must match, or one of them is wrong."""
    half = 500.0
    poly, box = _box_as_polygon(half), Rectangle.square(_C, 2 * half)
    assert poly.area() == pytest.approx(box.area(), rel=1e-6)
    for bearing, dist in ((0.0, 400.0), (90.0, 400.0), (45.0, 600.0), (0.0, 600.0)):
        assert poly.contains(*_at(bearing, dist)) == box.contains(*_at(bearing, dist))


def test_polygon_winding_direction_does_not_matter() -> None:
    """Clockwise or anticlockwise describe the same region, so both must give the same answers."""
    forward = _box_as_polygon(500.0)
    reversed_ring = Polygon(vertices=list(reversed(list(forward.vertices))))
    assert reversed_ring.area() == pytest.approx(forward.area(), rel=1e-9)
    assert reversed_ring.contains(*_C) == forward.contains(*_C) is True


def test_a_concave_polygon_excludes_its_notch() -> None:
    """The case a disc or a box cannot express — an L, where the missing quarter is outside."""
    # an L: the north-east quarter of a 1000 m square is cut away
    ell = Polygon(vertices=[
        _at(225.0, 707.1),                       # south-west corner
        _at(315.0, 707.1),                       # south-east
        _at(0.0, 500.0), _at(45.0, 353.6),       # up, then in — the notch
        _at(90.0, 500.0),                        # ... leaving the NE quarter outside
        _at(135.0, 707.1),                       # north-west
    ])
    assert ell.contains(*_at(225.0, 400.0))      # in the body
    assert not ell.contains(*_at(45.0, 600.0))   # in the removed quarter


def test_too_few_vertices_is_refused() -> None:
    with pytest.raises(ValueError, match="at least 3 vertices"):
        Polygon(vertices=[_C, _at(0.0, 100.0)])


# --- density ----------------------------------------------------------------------------------


def test_density_is_aircraft_per_square_kilometre() -> None:
    """The unit the traffic literature reports, derived from the shape so the two cannot drift."""
    disc = Disc(centre=_C, radius=1000.0)        # pi km^2
    assert disc.density(10) == pytest.approx(10.0 / math.pi, rel=1e-9)
    assert Rectangle.square(_C, 1000.0).density(5) == pytest.approx(5.0)   # exactly 1 km^2
