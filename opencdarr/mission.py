"""Mission — the aircraft's intent (ADR 0014, Phase 4d).

*What* the aircraft should achieve, as an inert frozen value — it does **not** fly the aircraft.
The :class:`~opencdarr.autopilot.Autopilot` (the navigator) turns a :class:`Mission` into a
:class:`~opencdarr.dynamics.MotionCommand`; the airframe (the controller) tracks that. This mirrors
the PX4 split: the mission/navigator holds the plan; the position controller flies it.

Positions are WGS84 ``lat/lon`` (the frame :class:`~opencdarr.state.AircraftState` uses), so no
origin/frame bookkeeping is needed; the autopilot converts to bearings/ranges via
:mod:`opencdarr.geo`. *How fast* to fly, the capture radius, and loiter behaviour are the
autopilot's configuration, not the mission's — the mission is pure geometry.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    """A single mission point, WGS84 ``lat/lon`` [deg]."""

    lat: float
    lon: float


@dataclass(frozen=True)
class Mission:
    """The aircraft's objective — a single ``goto`` point or an ordered ``flight_plan``.

    Exactly one is expected to be set. ``goto`` is the one-waypoint special case (fly there and
    loiter); ``flight_plan`` is a sequence flown in order, loitering at the final waypoint. Inert:
    consumed only by the autopilot, never read by the dynamics or the CDR core.
    """

    goto: tuple[float, float] | None = None  # (lat, lon)
    flight_plan: tuple[Waypoint, ...] | None = None

    def waypoints(self) -> tuple[Waypoint, ...]:
        """The mission as an ordered waypoint tuple (``goto`` is a one-waypoint plan)."""
        if self.flight_plan is not None:
            return self.flight_plan
        if self.goto is not None:
            return (Waypoint(self.goto[0], self.goto[1]),)
        return ()
