"""Steady, uniform, horizontal wind — the environment field the kinematics fly in (Phase 5).

:class:`WindField` is a **read-only environment input**, not aircraft state (ADR 0016 / Phase-5
plan decision 1): a steady uniform field is identical for every aircraft and cannot be affected by
one, so it is threaded into :meth:`~opencdarr.kinematics.base.Kinematics.step` as an argument — the
same category as ``perf`` / ``dt`` — never stored on an ``AircraftState``. The only per-aircraft
consequence of wind, the crab / heading ``ψ``, already lives in the clonable ``yaw`` field
(ADR 0012/0013).

The field is stored as its **inertial East/North components** ``(w_east, w_north)`` — the form the
kinematics use (the Eq 9 vector sum) — with :meth:`from_met` as the ergonomic constructor from a
*meteorological* wind (a bearing the wind is **coming from**, and a speed). The sign of Eq 1 is the
single easiest thing to get wrong, so it is documented and tested at this boundary.

Uniform-constant only, on purpose (Phase-5 plan decision 2): there is no ``(lat, lon, t) → vector``
spatial/temporal field yet — that is the deferred gust/shear generalisation. This *type* is the
seam a spatial field slots behind later, without carrying dead machinery now.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class WindField:
    """A steady, uniform, horizontal wind, as inertial ENU components [m/s].

    ``w_east`` / ``w_north`` are the components of the wind **velocity vector** — the direction the
    air is *moving toward* — so the ground velocity of an aircraft is its airspeed vector plus this
    (the Eq 9 vector sum). Build one from a meteorological wind with :meth:`from_met`.

    Attributes
    ----------
    w_east:
        East component of the wind velocity [m/s] (positive = air moving east).
    w_north:
        North component of the wind velocity [m/s] (positive = air moving north).
    """

    w_east: float
    w_north: float

    @classmethod
    def from_met(cls, coming_from_deg: float, speed: float) -> WindField:
        """Build from a meteorological wind: a bearing it is **coming from** and a speed (Eq 1).

        Meteorology names a wind by where it blows *from* (a "north wind" comes from the north and
        moves *toward* the south), aviation convention (0 = North, clockwise). The velocity vector
        therefore points the opposite way::

            w_east  = -speed · sin(coming_from)
            w_north = -speed · cos(coming_from)

        So a north wind (``coming_from = 0``) gives ``(0, -speed)`` — air moving south; a west wind
        (``coming_from = 270``) gives ``(+speed, 0)`` — air moving east.
        """
        r = math.radians(coming_from_deg)
        return cls(w_east=-speed * math.sin(r), w_north=-speed * math.cos(r))

    def components(self) -> tuple[float, float]:
        """The inertial ``(w_east, w_north)`` components [m/s] — the form the kinematics use."""
        return self.w_east, self.w_north

    @property
    def speed(self) -> float:
        """Wind speed [m/s] — the magnitude of the velocity vector."""
        return math.hypot(self.w_east, self.w_north)

    @property
    def coming_from(self) -> float:
        """Meteorological bearing the wind comes **from** [deg, aviation] (0.0 when calm)."""
        if self.speed == 0.0:
            return 0.0
        return math.degrees(math.atan2(-self.w_east, -self.w_north)) % 360.0


#: The calm field — zero wind. The default everywhere, so Phase-4 behaviour is the literal default
#: and every pre-wind call site and test is byte-identical (Phase-5 plan 5a gate).
NO_WIND = WindField(0.0, 0.0)
