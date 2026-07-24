"""Dynamics fundamentals shared by every implementation (ADR 0010 / 0011).

Holds the control input (:class:`MotionCommand`), the contribution-surface ABC
(:class:`Dynamics`), and the small helpers every implementation reuses (``_clip``, the zero-speed
guard, and the odometry accumulator update). The concrete models live beside this file —
``multirotor.py`` and ``dubins.py`` — one per file, mirroring ``cd/``, ``cr/``, ``crr/``.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

from opencdarr.performance import Performance
from opencdarr.state import AircraftState

_SPD_EPS = 1e-9  # m/s: below this a command has no meaningful direction -> hold current heading


@dataclass(frozen=True)
class MotionCommand:
    """The vehicle-neutral motion command — the single currency between guidance, separation, and
    dynamics (ADR 0011, superseding the pure-velocity command of ADR 0008).

    A command is a PX4-offboard-shaped *setpoint*, not a state to snap to: each field targets one
    channel of motion, and a :class:`Dynamics` model reads whichever fields its vehicle understands
    and ignores the rest — :class:`~opencdarr.dynamics.Multirotor` reads ``target_velocity`` (and
    ``target_yaw``); a fixed-wing (Phase 4c) reads the course/airspeed channels. Every field
    defaults to ``None`` ("unspecified", PX4's ``NaN`` / unset ``type_mask``); a model **fails
    fast** when no channel it requires is present (the missing-channel case of the ADR 0011
    feasibility taxonomy — an under-specified command for that vehicle is a programming error,
    surfaced here rather than silently obeyed).

    ``target_velocity`` is the resolvers' native output (the old velocity-vector command is exactly
    a :class:`MotionCommand` with just that field set), so the :meth:`from_track_speed` /
    :meth:`from_velocity` constructors and the ``gs`` / ``trk`` / ``v_east`` / ``v_north`` derived
    reads are preserved over it, and every existing call site reads unchanged. ``target_yaw`` /
    ``target_yawspeed`` land with :class:`~opencdarr.dynamics.Multirotor` (ADR 0012); the
    fixed-wing course/airspeed channels with Phase 4c; ``target_altitude`` /
    ``target_vertical_speed`` are defined but ignored until 3D lands (ADR 0011 §1).

    Facing decoupled from travel — flying one way while pointing another — is now expressible via
    ``target_yaw`` (the yaw-carrying state [[0008-velocity-vector-command]] §4 deferred, realised
    in ADR 0012), not by smuggling a sign into the velocity channel.

    Attributes
    ----------
    target_velocity:
        Desired **ground** velocity ``(v_east, v_north)`` [m/s], inertial ENU frame — the resolver
        / multirotor channel (PX4 ``TrajectorySetpoint.velocity`` in ``MAV_FRAME_LOCAL_NED``).
    target_body_velocity:
        Desired velocity in the **body frame** ``(v_forward, v_right)`` [m/s] — forward is the nose
        direction (``yaw``), right is 90° clockwise from it (PX4 ``MAV_FRAME_BODY_FRD``). Resolved
        to an inertial ``(v_east, v_north)`` through the current ``yaw`` inside
        :class:`~opencdarr.dynamics.Multirotor`, so "forward" is a fixed world direction only when
        ``yaw`` says so. A multirotor-only channel (it needs the decoupled yaw); an absent DOF for
        a fixed-wing. Takes precedence over ``target_velocity`` when both are set.
    target_position:
        Desired position ``(lat, lon)`` [deg] — the goto / waypoint channel (PX4
        ``TrajectorySetpoint.position``; the active waypoint). A
        :class:`~opencdarr.dynamics.Multirotor` flies straight to it and hovers; a
        :class:`~opencdarr.dynamics.FixedWing` tracks the leg to it.
    target_leg_start:
        The previous waypoint ``(lat, lon)`` [deg] — with ``target_position`` (the current
        waypoint) it is the **leg line** the fixed-wing L1 tracker follows (nulls cross-track).
    target_loiter_radius:
        Loiter radius [m] about ``target_position`` (the loiter centre) — set on arrival at the
        final waypoint. A :class:`~opencdarr.dynamics.FixedWing` flies a min-radius **orbit** at
        this radius (it cannot stop); a :class:`~opencdarr.dynamics.Multirotor` ignores it and
        simply **hovers** at the centre (PX4 ``MAV_CMD_NAV_LOITER_*``).
        ``None`` for a bare goto ⇒ pure-pursuit to ``target_position``. A multirotor ignores it (it
        flies to the point, not along the line).
    target_yaw:
        Desired nose heading [deg, aviation] — the multirotor yaw channel (PX4
        ``TrajectorySetpoint.yaw``), **decoupled from the direction of travel**. ``None`` = yaw not
        commanded (hold current yaw). Read by :class:`~opencdarr.dynamics.Multirotor` (ADR 0012);
        an *absent degree of freedom* for a coupled-heading fixed-wing, ignored there (Phase 4c).
    target_yawspeed:
        Desired yaw rate [deg/s] — the multirotor yaw-rate channel (PX4
        ``TrajectorySetpoint.yawspeed``); used when ``target_yaw`` is unset.
    target_course:
        Desired ground-track course χ [deg, aviation] — the fixed-wing lateral channel (PX4
        ``FixedWingLateralSetpoint.course``). Read by :class:`~opencdarr.dynamics.FixedWing`
        (ADR 0013); an absent DOF for a multirotor, ignored there.
    target_airspeed_direction:
        Desired heading ψ of the airspeed vector [deg, aviation] — the fixed-wing lateral channel
        (PX4 ``FixedWingLateralSetpoint.airspeed_direction``). **Overrides ``target_course`` when
        set** (PX4 semantics). Equals ``target_course`` when there is no wind; their difference is
        the crab angle (Phase 5).
    target_airspeed:
        Desired equivalent airspeed [m/s] — the fixed-wing longitudinal channel (PX4
        ``FixedWingLongitudinalSetpoint.equivalent_airspeed``). An *airspeed*, not a ground speed
        (they differ under wind, Phase 5).
    target_lateral_accel:
        Desired lateral acceleration [m/s²] — the fixed-wing lateral feedforward channel (PX4
        ``FixedWingLateralSetpoint.lateral_acceleration``); optional, a feedforward on the bank.
    target_altitude:
        Desired altitude [m] — the fixed-wing longitudinal channel (PX4
        ``FixedWingLongitudinalSetpoint.altitude``); defined, ignored (2D this pass, ADR 0011 §1).
    target_vertical_speed:
        Desired vertical / height rate [m/s] — PX4 ``FixedWingLongitudinalSetpoint.height_rate``;
        defined, ignored (2D this pass).
    """

    target_velocity: tuple[float, float] | None = None
    target_body_velocity: tuple[float, float] | None = None
    target_position: tuple[float, float] | None = None
    target_leg_start: tuple[float, float] | None = None
    target_loiter_radius: float | None = None
    target_yaw: float | None = None
    target_yawspeed: float | None = None
    target_course: float | None = None
    target_airspeed_direction: float | None = None
    target_airspeed: float | None = None
    target_lateral_accel: float | None = None
    target_altitude: float | None = None
    target_vertical_speed: float | None = None

    @classmethod
    def from_track_speed(cls, hdg: float, spd: float) -> MotionCommand:
        """Build a velocity command from an aviation heading [deg] and ground speed [m/s]."""
        r = math.radians(hdg)
        return cls(target_velocity=(spd * math.sin(r), spd * math.cos(r)))

    @classmethod
    def from_velocity(cls, v_east: float, v_north: float) -> MotionCommand:
        """Build a velocity command from East/North ground-velocity components [m/s]."""
        return cls(target_velocity=(v_east, v_north))

    @property
    def _velocity(self) -> tuple[float, float]:
        """``target_velocity``, or fail fast if a caller needs it while unset (ADR 0011 §1)."""
        if self.target_velocity is None:
            raise ValueError(
                "MotionCommand has no target_velocity: this channel requires a ground-velocity "
                "vector (v_east/v_north/gs/trk are derived from it). An under-specified command "
                "for this vehicle is a programming error."
            )
        return self.target_velocity

    @property
    def v_east(self) -> float:
        """East component of ``target_velocity`` [m/s] (raises if it is unset)."""
        return self._velocity[0]

    @property
    def v_north(self) -> float:
        """North component of ``target_velocity`` [m/s] (raises if it is unset)."""
        return self._velocity[1]

    @property
    def gs(self) -> float:
        """Commanded ground speed [m/s] — magnitude of ``target_velocity``."""
        v_east, v_north = self._velocity
        return math.hypot(v_east, v_north)

    @property
    def trk(self) -> float:
        """Commanded track [deg, aviation] — direction of ``target_velocity`` (0 if zero)."""
        v_east, v_north = self._velocity
        return math.degrees(math.atan2(v_east, v_north)) % 360.0


# Backward-compatible name for the pure-velocity command (ADR 0008), which :class:`MotionCommand`
# (ADR 0011) supersedes. The alias keeps ``from_track_speed`` / ``gs`` / ``trk`` / ``isinstance``
# call sites reading unchanged through the Phase-4a migration; it is removed once the loop and its
# callers speak ``MotionCommand`` directly. Note: direct ``Command(v_east=, v_north=)`` positional
# construction no longer exists — build a velocity command via :meth:`MotionCommand.from_velocity`
# or ``MotionCommand(target_velocity=(...))``.
Command = MotionCommand


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

    - :class:`~opencdarr.dynamics.Multirotor` — isotropic accel, no coupled heading, independent
      yaw; consumes a PX4 ``TrajectorySetpoint``-shaped command (``multirotor.py``, ADR 0012).
    - :class:`~opencdarr.dynamics.FixedWing` — non-holonomic coordinated-turn point mass:
      bank-limited heading, stall/load envelope, finite roll, wind-ready (``fixedwing.py``, ADR
      0013). Superseded the former ``DubinsDynamics``.

    Every implementation must advance the odometry accumulators (via :func:`odometry_update`) so
    ``flight_time`` / ``distance_flown`` stay correct whichever model ran (ADR 0010).
    """

    @abstractmethod
    def step(
        self, state: AircraftState, command: MotionCommand, perf: Performance, dt: float
    ) -> AircraftState:
        """Advance ``state`` by ``dt`` seconds under ``command``.

        Pure — a function of the given arguments only; no global or module state is read or
        written, so a clone (IPS particle) evolved through this call stays independent of its
        source.
        """
