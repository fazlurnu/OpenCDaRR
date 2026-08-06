"""Where a result counts: the measurement area.

An experiment often flies aircraft over a larger region than it wants to *measure*. Traffic studies
are the clearest case — aircraft are released on a ring outside the study disc and enter it already
in flight, so the first seconds after release, where two aircraft can appear close together with no
history of ever having been separated, are flown but not measured. Without that separation between
the flown region and the measured one, the release rule itself shows up in the answer.

A :class:`MeasurementArea` answers one question for the simulator — *does this position count?* —
and one for the caller: how large the measured region is, which is what turns a count into a
density or a rate. The shape behind those two answers is an implementation detail, so a new one is
a new subclass and nothing else changes.

**Geodesy.** :class:`Disc` is exact: containment is a great-circle distance against the radius.
:class:`Rectangle` and :class:`Polygon` test in the local east/north tangent plane about their own
reference point, which is planar rather than spherical. Over the kilometres these studies span the
difference is far below the metre — but it is an approximation, and it is written down here rather
than discovered later (ADR 0003 owns the geodesy deliberately).
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Sequence
from dataclasses import dataclass

from opencdarr import geo

LatLon = tuple[float, float]


def _enu(origin: LatLon, point: LatLon) -> tuple[float, float]:
    """``point`` as (east, north) metres from ``origin`` — the local tangent plane."""
    qdr, dist = geo.qdrdist(origin[0], origin[1], point[0], point[1])
    bearing = math.radians(qdr)
    return dist * math.sin(bearing), dist * math.cos(bearing)


class MeasurementArea(ABC):
    """The region an experiment measures in. ``None`` anywhere one is expected means *everywhere*.

    Two questions, because a study needs both: :meth:`contains` gates whether an event counts, and
    :meth:`area` is what a count is divided by to become a density. Keeping them on one object is
    what stops a caller pairing a disc with somebody else's idea of its size.
    """

    @abstractmethod
    def contains(self, lat: float, lon: float) -> bool:
        """Whether this position is inside the measured region."""

    @abstractmethod
    def area(self) -> float:
        """The measured region's area [m^2] — the denominator of a density."""

    def density(self, n_aircraft: int) -> float:
        """``n_aircraft`` spread over this area, in aircraft per square kilometre.

        Derived rather than stored so it cannot fall out of step with the shape, and given in
        aircraft/km^2 because that is the unit the traffic literature reports.
        """
        return n_aircraft / (self.area() / 1e6)


@dataclass(frozen=True)
class Disc(MeasurementArea):
    """A circular region — the traffic studies' shape, and the only exactly-spherical one here."""

    centre: LatLon
    radius: float  # [m]

    def __post_init__(self) -> None:
        if self.radius <= 0.0:
            raise ValueError(f"radius must be positive, got {self.radius}")

    def contains(self, lat: float, lon: float) -> bool:
        return bool(geo.qdrdist(self.centre[0], self.centre[1], lat, lon)[1] <= self.radius)

    def area(self) -> float:
        return math.pi * self.radius**2


@dataclass(frozen=True)
class Rectangle(MeasurementArea):
    """An east/north aligned box, given by its centre and its two side lengths.

    Axis-aligned on purpose: a rotated box is a :class:`Polygon`, and keeping this one aligned
    makes containment two comparisons rather than a ray cast.
    """

    centre: LatLon
    width: float  # east-west extent [m]
    height: float  # north-south extent [m]

    def __post_init__(self) -> None:
        if self.width <= 0.0 or self.height <= 0.0:
            raise ValueError(f"sides must be positive, got {self.width} x {self.height}")

    @classmethod
    def square(cls, centre: LatLon, side: float) -> Rectangle:
        """A square is a rectangle with equal sides — a constructor, not a separate shape."""
        return cls(centre=centre, width=side, height=side)

    def contains(self, lat: float, lon: float) -> bool:
        east, north = _enu(self.centre, (lat, lon))
        return abs(east) <= self.width / 2 and abs(north) <= self.height / 2

    def area(self) -> float:
        return self.width * self.height


@dataclass(frozen=True)
class Polygon(MeasurementArea):
    """An arbitrary simple polygon, given as ``(lat, lon)`` vertices in order.

    The general shape: an airspace sector or a city boundary that no disc or box describes. Winding
    direction does not matter, and the ring is closed automatically. Containment is a ray cast in
    the local tangent plane about the first vertex, so a point exactly on an edge may fall either
    way — irrelevant against a boundary that is itself a modelling choice.
    """

    vertices: Sequence[LatLon]

    def __post_init__(self) -> None:
        if len(self.vertices) < 3:
            raise ValueError(f"a polygon needs at least 3 vertices, got {len(self.vertices)}")

    def _plane(self) -> list[tuple[float, float]]:
        origin = self.vertices[0]
        return [_enu(origin, v) for v in self.vertices]

    def contains(self, lat: float, lon: float) -> bool:
        x, y = _enu(self.vertices[0], (lat, lon))
        ring = self._plane()
        inside = False
        for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1], strict=True):
            # does the edge straddle the ray's latitude, and is the crossing to the right?
            if (y1 > y) != (y2 > y):
                x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                if x < x_cross:
                    inside = not inside
        return inside

    def area(self) -> float:
        """The shoelace area of the ring, in the local plane. Sign-free, so winding is free too."""
        ring = self._plane()
        twice = sum(x1 * y2 - x2 * y1
                    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1], strict=True))
        return abs(twice) / 2.0
