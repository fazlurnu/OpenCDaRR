"""Multirotor point-mass dynamics (ADR 0012): :class:`Multirotor`.

The PX4-facing multirotor model. It consumes a PX4 ``TrajectorySetpoint``-shaped
:class:`~opencdarr.dynamics.MotionCommand` and applies multirotor limits, in **two decoupled
channels**:

- **translation** — the ground-velocity vector moves directly toward ``target_velocity`` under an
  isotropic acceleration limit (``perf.ax``), clamped to ``perf.v_max``. No coupled heading, no
  turn-rate limit: a 90° direction change is one bounded step in velocity-space, and the vehicle
  can slow, stop, and hover. This is the holonomic core validated in ADR 0009 — reproduced here
  byte-for-byte (the Phase-4b migration gate) before yaw is layered on.
- **yaw** — the nose heading ``AircraftState.yaw`` converges toward ``target_yaw`` (or integrates
  ``target_yawspeed``) under ``perf.yaw_rate_max``, **independent of translation**. A multirotor
  can translate one way while pointing another (camera-pointing); ``trk`` and ``yaw`` never
  re-couple.

Supersedes ``HolonomicDynamics`` (ADR 0009): the coupled-heading approximation is gone and the
model is named for the real vehicle, with the PX4 offboard setpoint as its input and independent
yaw as a first-class channel. Governing equations for the translation core:
``vault/decisions/0009-holonomic-dynamics.md``.
"""

from __future__ import annotations

import math
from dataclasses import replace

from opencdarr import geo
from opencdarr.dynamics.base import _SPD_EPS, Dynamics, MotionCommand, _clip, odometry_update
from opencdarr.kinematics import velocity_enu
from opencdarr.performance import Performance
from opencdarr.state import AircraftState

_HOVER_CAPTURE = 0.5  # m: within this range of a target position, command a full stop (hover)


def _clip_magnitude(vx: float, vy: float, max_mag: float) -> tuple[float, float]:
    """Scale ``(vx, vy)`` down to at most ``max_mag``, preserving direction; leave it if within."""
    mag = math.hypot(vx, vy)
    if mag <= max_mag:
        return vx, vy
    scale = max_mag / mag
    return vx * scale, vy * scale


def _position_to_velocity(
    state: AircraftState, target: tuple[float, float], perf: Performance
) -> tuple[float, float]:
    """The velocity toward a ``target`` position that slows to a hover at it (ADR 0014).

    The multirotor's position tracker (PX4 position controller, in our layering): bearing/range to
    the target via :func:`opencdarr.geo.qdrdist`, then a speed capped so the vehicle can always
    decelerate to a stop *at* the point — ``sqrt(2·ax·range)`` (the stopping-distance law) clamped
    to ``v_max``. So it flies directly to the point and settles (range → 0 ⇒ speed → 0 ⇒ hover),
    with no tuning constant. Inside a small capture ball (``_HOVER_CAPTURE``) it commands a full
    stop, so it settles cleanly instead of chattering across the point at the step granularity.
    """
    bearing, dist = geo.qdrdist(state.lat, state.lon, target[0], target[1])
    if dist <= _HOVER_CAPTURE:
        return 0.0, 0.0  # arrived: hold position (kills the sub-step limit cycle across the point)
    speed = min(perf.v_max, math.sqrt(2.0 * perf.ax * dist))
    r = math.radians(bearing)
    return speed * math.sin(r), speed * math.cos(r)


def _body_to_enu(v_fwd: float, v_right: float, yaw_deg: float) -> tuple[float, float]:
    """Rotate a body-frame velocity ``(forward, right)`` into inertial ``(east, north)`` by yaw.

    Forward points along ``yaw`` (aviation, 0=N, CW); right is 90° clockwise from forward. This is
    the PX4 ``MAV_FRAME_BODY_FRD`` velocity setpoint resolved through the vehicle's heading — a
    nose-relative command, so it only means a fixed world direction for a given ``yaw``.
    """
    r = math.radians(yaw_deg)
    v_east = v_fwd * math.sin(r) + v_right * math.cos(r)
    v_north = v_fwd * math.cos(r) - v_right * math.sin(r)
    return v_east, v_north


def _step_yaw(
    state: AircraftState, command: MotionCommand, perf: Performance, dt: float
) -> float | None:
    """The new nose heading after ``dt`` — the decoupled yaw channel.

    Returns ``state.yaw`` unchanged (possibly ``None``) when no yaw is commanded — an uncommanded
    multirotor holds its heading, and an airframe that never set ``yaw`` stays nose-aligned with
    its track (``None``). A commanded ``target_yaw`` / ``target_yawspeed`` resolves the effective
    current yaw (``trk`` when it was ``None``) and returns a concrete heading, converged under the
    yaw-rate limit. Never reads or writes the translation channel — that is the point of the split.
    """
    if command.target_yaw is None and command.target_yawspeed is None:
        return state.yaw  # hold: None stays None (track-aligned), a concrete yaw stays put
    cur = state.yaw if state.yaw is not None else state.trk
    max_step = perf.yaw_rate_max * dt
    if command.target_yaw is not None:
        err = ((command.target_yaw - cur + 180.0) % 360.0) - 180.0  # signed, shortest way
        if abs(err) <= max_step:
            return command.target_yaw % 360.0  # reachable this step -> snap onto the target
        return (cur + math.copysign(max_step, err)) % 360.0
    # target_yawspeed only: integrate the commanded rate, clamped to the yaw-rate limit
    assert command.target_yawspeed is not None  # narrowed: target_yaw is None but not both (guard)
    rate = _clip(command.target_yawspeed, -perf.yaw_rate_max, perf.yaw_rate_max)
    return (cur + rate * dt) % 360.0


class Multirotor(Dynamics):
    """A multirotor point mass (ADR 0012): decoupled translation + yaw, PX4-setpoint input.

    Translation reuses ``perf.v_max`` (top speed) and ``perf.ax`` (isotropic acceleration); yaw
    reuses ``perf.yaw_rate_max``. The fixed-wing limits (``perf.phi_max`` / ``perf.roll_rate_max``)
    do not apply — a multirotor does not bank — and ``perf.v_min`` does not apply either: facing is
    decoupled from travel, so "backward" is just another direction reachable via the velocity
    vector.

    ``trk`` / ``gs`` mean exactly what they do under :class:`~opencdarr.dynamics.FixedWing` —
    direction and magnitude of ground travel — so CD/CR/CRR and any other airframe sharing the
    encounter read this aircraft's state identically. ``yaw`` is the *additional* decoupled nose
    heading (``None`` until independently commanded); only *how the vehicle reaches* a velocity,
    and that it can point independently, differ.
    """

    def step(
        self, state: AircraftState, command: MotionCommand, perf: Performance, dt: float
    ) -> AircraftState:
        # 1. translation target, by PX4 OffboardControlMode priority (position > velocity):
        #    a ``target_position`` (mission nominal) is tracked to a hover; else a body-frame or an
        #    inertial ``target_velocity`` (DAA override / resolver). Clamped to the top-speed
        #    envelope (no v_min — a multirotor has no separate backward capability); clamp first,
        #    bound the step toward it, so the result stays in the v_max disk throughout (convex:
        #    both step endpoints inside -> every point inside).
        if command.target_position is not None:
            cmd_e, cmd_n = _position_to_velocity(state, command.target_position, perf)
        elif command.target_body_velocity is not None:
            yaw = state.yaw if state.yaw is not None else state.trk
            cmd_e, cmd_n = _body_to_enu(*command.target_body_velocity, yaw)
        else:
            cmd_e, cmd_n = command.v_east, command.v_north  # raises if no channel is set
        tgt_e, tgt_n = _clip_magnitude(cmd_e, cmd_n, perf.v_max)

        # 2. isotropic acceleration limit: bound the *vector* step by ax*dt in any direction (not
        #    two independent 1D limits — that would be a coupled-heading turn-rate + speed ramp).
        cur_e, cur_n = velocity_enu(state)
        step_e, step_n = _clip_magnitude(tgt_e - cur_e, tgt_n - cur_n, perf.ax * dt)
        new_e, new_n = cur_e + step_e, cur_n + step_n

        # 3. direction/magnitude of ground travel from the new vector. A ~zero vector has no
        #    defined direction -> hold the current track.
        new_gs = math.hypot(new_e, new_n)
        new_trk = (
            state.trk if new_gs <= _SPD_EPS else math.degrees(math.atan2(new_e, new_n)) % 360.0
        )

        # 4. position: great-circle forward step (metres) along the new track
        lat, lon = geo.forward(state.lat, state.lon, new_trk, new_gs * dt)

        # 5. yaw: the decoupled nose-heading channel (independent of steps 1-4)
        new_yaw = _step_yaw(state, command, perf, dt)

        return replace(
            state,
            lat=float(lat),
            lon=float(lon),
            trk=new_trk,
            gs=new_gs,
            yaw=new_yaw,
            **odometry_update(state, new_gs, dt),
        )
