"""Dynamics fundamentals shared by every implementation (ADR 0010).

Holds the control input (:class:`Command`), the contribution-surface ABC (:class:`Dynamics`),
and the small helpers every implementation reuses (``_clip``, the zero-speed guard, and the
odometry accumulator update). The concrete models live beside this file — ``dubins.py`` and
``holonomic.py`` — one per file, mirroring ``cd/``, ``cr/``, ``crr/``.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from opencdarr.performance import Performance
from opencdarr.state import AircraftState

_SPD_EPS = 1e-9  # m/s: below this a command has no meaningful direction -> hold current heading


@dataclass(frozen=True)
class Command:
    """A control command: the desired ground **velocity** vector, East–North [m/s] (ADR 0008).

    A command says *where the aircraft wants to go*, as a velocity vector — not a heading and a
    speed. This keeps the control interface neutral about the airframe: a :class:`Dynamics` model
    decides how to chase the target vector (:class:`~opencdarr.dynamics.DubinsDynamics`
    reconstructs a track and turns toward it; :class:`~opencdarr.dynamics.HolonomicDynamics` drives
    ``v_east`` / ``v_north`` directly). The resolvers (:class:`~opencdarr.cr.MVP`,
    :class:`~opencdarr.cr.VO`) already compute a velocity vector internally, so they return one
    with no polar round-trip.

    Build one from an aviation heading and speed with :meth:`from_track_speed`; read ``trk`` /
    ``gs`` back as derived properties. A zero vector has no defined direction (``trk`` returns 0)
    — a coupled-heading model reads that as "hold current heading". Backward flight (facing
    decoupled from travel), which the old signed-speed command could express, is deliberately not
    representable here — it belongs to a future yaw-carrying state, not the velocity command
    (ADR 0008).

    Attributes
    ----------
    v_east, v_north:
        Desired ground-velocity components, East and North, in metres per second.
    """

    v_east: float
    v_north: float

    @classmethod
    def from_track_speed(cls, hdg: float, spd: float) -> Command:
        """Build a command from an aviation heading [deg] and ground speed [m/s]."""
        r = math.radians(hdg)
        return cls(v_east=spd * math.sin(r), v_north=spd * math.cos(r))

    @property
    def gs(self) -> float:
        """Commanded ground speed [m/s] — the vector's magnitude."""
        return math.hypot(self.v_east, self.v_north)

    @property
    def trk(self) -> float:
        """Commanded track [deg, aviation convention] — direction of the vector (0 if zero)."""
        return math.degrees(math.atan2(self.v_east, self.v_north)) % 360.0


def _clip(value: float, low: float, high: float) -> float:
    """Clamp ``value`` to ``[low, high]``."""
    return max(low, min(value, high))


def odometry_update(state: AircraftState, gs: float, dt: float) -> dict[str, float]:
    """The odometry-accumulator changes for a step ending at ground speed ``gs`` over ``dt``.

    Returned as a dict to splat into ``dataclasses.replace(state, ..., **odometry_update(...))``,
    so every :class:`Dynamics` implementation advances ``flight_time`` and ``distance_flown`` the
    same way and none can forget them (ADR 0010). ``gs`` is the *new* (post-step) ground speed,
    matching the distance the position update actually moves (``gs * dt`` along the new track).
    """
    return {
        "flight_time": state.flight_time + dt,
        "distance_flown": state.distance_flown + gs * dt,
    }


class Dynamics(ABC):
    """Base class every dynamics model implements — the contribution surface for how an
    aircraft's kinematics evolve (ADR 0007).

    A model subclasses :class:`Dynamics` and implements ``step``; it is passed into
    :func:`~opencdarr.loop.run_encounter` as ``dynamics=...`` in place of the default. This
    mirrors every other model family in the library (:class:`~opencdarr.cd.base.ConflictDetector`,
    :class:`~opencdarr.cr.base.ConflictResolver`, ...): a new physical effect adds a file, not a
    fork of the loop (``design_brief.md``: the interface is the contribution surface).

    Implementations live beside this file:

    - :class:`~opencdarr.dynamics.DubinsDynamics` — turn-rate-limited, coupled heading
      (``dubins.py``).
    - :class:`~opencdarr.dynamics.HolonomicDynamics` — isotropic accel, no coupled heading
      (``holonomic.py``).
    - e.g. a wind-aware dynamics model — *future, not implemented* (ADR 0007 names the shape).

    Every implementation must advance the odometry accumulators (via :func:`odometry_update`) so
    ``flight_time`` / ``distance_flown`` stay correct whichever model ran (ADR 0010).
    """

    @abstractmethod
    def step(
        self, state: AircraftState, command: Command, perf: Performance, dt: float
    ) -> AircraftState:
        """Advance ``state`` by ``dt`` seconds under ``command``.

        Pure — a function of the given arguments only; no global or module state is read or
        written, so a clone (IPS particle) evolved through this call stays independent of its
        source.
        """
