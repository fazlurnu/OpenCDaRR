"""Fixed-wing coordinated-turn dynamics (ADR 0013): :class:`FixedWing`.

The PX4-facing fixed-wing model, re-derived from the kinematic point-mass model of Reyner & Liem
(*Energy-Efficient Trochoidal Path Planning...*, Drones 2026, 10, 426;
``vault/papers/drones-wind.pdf``, Eqs 1-9, 13-17) — the same model PX4's
``fw_lateral_longitudinal_control`` implements. We take only its kinematics, never its path planner
(``lesson-learnt.md``: don't port).

Non-holonomic: the airframe flies its **airspeed vector** (heading ``ψ``), turns by **banking**
(coordinated, negligible sideslip), and cannot stop or move sideways. It consumes the PX4
fixed-wing setpoint channels — ``target_course`` (χ) / ``target_airspeed_direction`` (ψ) /
``target_airspeed`` (equivalent airspeed) — and chases them under a stall/load envelope and a
finite roll rate.

Coordinated-turn model (inertial frame, x=east, y=north, angles CW from north; ``vault/derivations/
fixedwing-coordinated-turn.md``):

    ẋ = V_TAS·sin ψ + w_x            (Eq 9)   position, air-relative + wind vector sum
    ẏ = V_TAS·cos ψ + w_y            (Eq 9)
    ψ̇ = g·tan φ / V_TAS              (Eq 8)   bank φ produces the yaw rate

with ground speed ``V_GS = √(ẋ²+ẏ²)`` (Eq 4) and course ``χ = atan2(ẋ, ẏ)`` derived each step. This
model is **wind-ready by construction**: the wind term ``(w_x, w_y)`` is present but **fixed at
zero this pass** — Phase 5 turns wind on by feeding a non-zero vector, with no change to the
integrator. At zero wind ``ψ = χ`` and ``V_GS = V_TAS`` (the inert Phase-5 hook, checked in tests).

Supersedes the coupled-heading ``DubinsDynamics`` (deleted): a fixed-wing's turn radius is
speed-dependent (``R = V²/(g·tan φ)``) and bounded by stall, which a fixed turn-rate cap cannot
express. Validated analytically against the paper's closed forms (ADR 0002), not against BlueSky —
deleting Dubins retired the BlueSky trajectory anchor (ADR 0005), recorded in ADR 0013.
"""

from __future__ import annotations

import math
from dataclasses import replace

from opencdarr import geo
from opencdarr.dynamics.base import _SPD_EPS, Dynamics, MotionCommand, _clip, odometry_update
from opencdarr.performance import Performance
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField

_G = 9.80665  # m/s^2, standard gravity (the g in ψ̇ = g·tan φ / V_TAS)
_L1_DISTANCE = 80.0  # m: L1 lookahead distance — the path-follower's capture-vs-tracking knob


def _guidance_course(
    lat: float, lon: float, leg_start: tuple[float, float] | None, target: tuple[float, float]
) -> float:
    """Desired ground course [deg, aviation] from the L1 path-follower (ADR 0014).

    Steers toward an **L1 reference point** on the leg line a lookahead ``_L1_DISTANCE`` ahead of
    the aircraft's foot-of-perpendicular — so an off-track aircraft curves *onto* the line and then
    tracks *along* it (cross-track error is nulled), rather than cutting to the endpoint. When
    farther off track than ``_L1_DISTANCE`` it steers at the foot (maximum correction).
    ``leg_start = None`` (a bare goto) ⇒ pure-pursuit straight at ``target``. Wind is zero this
    pass, so course = heading.

    Geometry is in a local ENU frame centred on the aircraft (``geo.qdrdist`` bearings/ranges).
    """
    te, tn = _enu_from(lat, lon, target)  # target relative to the aircraft
    if leg_start is None:
        return math.degrees(math.atan2(te, tn)) % 360.0  # pursuit: steer straight at the target
    ae, an = _enu_from(lat, lon, leg_start)  # leg start relative to the aircraft
    ux, uy = te - ae, tn - an  # leg direction (A -> B)
    leglen = math.hypot(ux, uy)
    if leglen < _SPD_EPS:
        return math.degrees(math.atan2(te, tn)) % 360.0  # degenerate leg -> pursuit
    ux, uy = ux / leglen, uy / leglen
    # foot of the perpendicular from the aircraft (origin) onto the line through A: F = A + (-A·u)u
    proj = -(ae * ux + an * uy)
    fx, fy = ae + proj * ux, an + proj * uy
    cross = math.hypot(fx, fy)  # cross-track distance
    ahead = math.sqrt(max(0.0, _L1_DISTANCE * _L1_DISTANCE - cross * cross))
    rx, ry = fx + ahead * ux, fy + ahead * uy  # the L1 reference point on the leg line
    return math.degrees(math.atan2(rx, ry)) % 360.0


def _enu_from(lat: float, lon: float, point: tuple[float, float]) -> tuple[float, float]:
    """``point`` (lat, lon) as ENU ``(east, north)`` metres relative to ``(lat, lon)``."""
    qdr, dist = geo.qdrdist(lat, lon, point[0], point[1])
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def _loiter_course(lat: float, lon: float, center: tuple[float, float], radius: float) -> float:
    """Desired ground course [deg] to orbit ``center`` at ``radius`` — the fixed-wing loiter (a
    fixed-wing cannot hover, so it circles; ADR 0014).

    A single law captures *and* holds the circle: steer at an angle ``asin(radius/d)`` off the
    bearing to the centre, where ``d`` is the range. Far outside (``d ≫ radius``) that offset → 0,
    so it flies toward the centre; on the circle (``d = radius``) it is 90°, a pure tangent; inside
    (``d < radius``, clip) it is 90° outward. The radius must exceed the airframe's min turn radius
    (``V²/(g·tan φ_max)``) for the orbit to be holdable.
    """
    brg, dist = geo.qdrdist(lat, lon, center[0], center[1])
    offset = math.degrees(math.asin(min(1.0, radius / max(dist, _SPD_EPS))))
    return (brg + offset) % 360.0


def _stall_bank_limit(airspeed: float, v_stall: float, phi_max: float) -> float:
    """Bank limit [deg] from the stall-in-turn constraint at this airspeed (ADR 0013).

    A coordinated turn raises the stall speed by the load factor:
    ``V_stall(φ) = v_stall·√(1/cos φ)``. Requiring the airspeed to stay above it gives
    ``cos φ ≥ (v_stall/airspeed)²``, i.e. a bank cap that tightens as the airframe slows toward
    stall. Combined (min) with the structural ``phi_max``.
    """
    ratio = v_stall / airspeed
    cos_min = min(1.0, ratio * ratio)  # airspeed >= v_stall, so ratio <= 1; guard rounding
    phi_stall = math.degrees(math.acos(cos_min))
    return min(phi_max, phi_stall)


class FixedWing(Dynamics):
    """A fixed-wing coordinated-turn point mass (ADR 0013): airspeed + bank-limited heading.

    Reads ``perf.v_min`` (**stall speed**, positive), ``perf.v_max`` (max airspeed), ``perf.ax``
    (airspeed acceleration), ``perf.phi_max`` (bank limit) and ``perf.roll_rate_max`` (roll rate).
    Carries ``yaw`` as the heading ``ψ`` and ``bank`` as the roll angle ``φ`` (both clone with the
    state). The multirotor channels (``target_velocity`` / ``target_yaw``) are an *absent degree of
    freedom* here and are ignored — a raw velocity command carries no course/airspeed for a
    fixed-wing, so a command with **no** fixed-wing channel set fails fast (the velocity→course
    projection for resolver output is a separate concern, Phase 4e).
    """

    def step(
        self,
        state: AircraftState,
        command: MotionCommand,
        perf: Performance,
        dt: float,
        wind: WindField = NO_WIND,
    ) -> AircraftState:
        # fail fast on an under-specified command: a fixed-wing needs a lateral, airspeed, or
        # position channel (not a raw velocity — that is a multirotor channel, an absent DOF here)
        if (
            command.target_position is None
            and command.target_course is None
            and command.target_airspeed_direction is None
            and command.target_airspeed is None
        ):
            raise ValueError(
                "MotionCommand has no fixed-wing channel (target_position / target_course / "
                "target_airspeed_direction / target_airspeed): a fixed-wing cannot fly a raw "
                "velocity command — project it to a course/airspeed setpoint first (Phase 4e)."
            )

        psi = state.yaw if state.yaw is not None else state.trk  # heading ψ (nose = airspeed vec)

        # 1. airspeed (longitudinal): clamp the command into the envelope, then ramp by ax*dt.
        #    v_min is the level stall speed; at zero wind the airspeed equals the ground speed.
        v_cur = state.gs
        v_tgt = _clip(command.target_airspeed, perf.v_min, perf.v_max) if (
            command.target_airspeed is not None
        ) else v_cur
        v = v_cur + _clip(v_tgt - v_cur, -perf.ax * dt, perf.ax * dt)

        # 2. bank authority at this airspeed: the structural limit, tightened by stall-in-turn
        phi_max_eff = _stall_bank_limit(v, perf.v_min, perf.phi_max)

        # 3. heading target ψ_cmd, by channel priority: a target_position runs the L1 path-follower
        #    (leg line -> course); else airspeed_direction is ψ directly (overrides course);
        #    else course is χ (= ψ at zero wind, WCA lands in Phase 5); else hold heading.
        if command.target_position is not None:
            if command.target_loiter_radius is not None:
                psi_cmd = _loiter_course(
                    state.lat, state.lon, command.target_position, command.target_loiter_radius
                )
            else:
                psi_cmd = _guidance_course(
                    state.lat, state.lon, command.target_leg_start, command.target_position
                )
        elif command.target_airspeed_direction is not None:
            psi_cmd = command.target_airspeed_direction
        elif command.target_course is not None:
            psi_cmd = command.target_course
        else:
            psi_cmd = psi  # airspeed-only command: hold heading

        # 4. heading error (signed, shortest way) and the desired turn rate / bank to null it.
        #    ω_max = g·tan(φ_max_eff)/V is the speed-dependent turn-rate cap; the controller is
        #    proportional (gain 1, deg error -> deg/s), like the former coupled-heading model.
        e = ((psi_cmd - psi + 180.0) % 360.0) - 180.0
        w_max = math.degrees(_G * math.tan(math.radians(phi_max_eff)) / v)
        w_des = _clip(e, -w_max, w_max)
        phi_des = math.degrees(math.atan(math.radians(w_des) * v / _G))

        # 5. finite roll: bank moves toward φ_des at no more than roll_rate_max*dt (this is why
        #    ``bank`` is state — the bound is relative to the previous bank), then clamped.
        roll_step = _clip(phi_des - state.bank, -perf.roll_rate_max * dt, perf.roll_rate_max * dt)
        phi = _clip(state.bank + roll_step, -phi_max_eff, phi_max_eff)

        # 6. heading: integrate ψ̇ = g·tan φ / V, or snap onto the target if reachable this step.
        w_new = math.degrees(_G * math.tan(math.radians(phi)) / v)
        if abs(e) > abs(w_new * dt):
            psi_new = (psi + w_new * dt) % 360.0
        else:
            psi_new = psi_cmd % 360.0

        # 7. position: air-relative velocity + wind (Eq 9 vector sum), then the ground-track course
        #    and ground speed as OUTPUTS, and a great-circle step along χ. At NO_WIND the wind term
        #    is (0, 0), so ψ == χ and V_GS == V_TAS (byte-identical to the pre-Phase-5 integrator).
        w_x, w_y = wind.components()
        r = math.radians(psi_new)
        vx = v * math.sin(r) + w_x
        vy = v * math.cos(r) + w_y
        v_gs = math.hypot(vx, vy)
        chi = state.trk if v_gs <= _SPD_EPS else math.degrees(math.atan2(vx, vy)) % 360.0
        lat, lon = geo.forward(state.lat, state.lon, chi, v_gs * dt)

        return replace(
            state,
            lat=float(lat),
            lon=float(lon),
            trk=chi,
            gs=v_gs,
            yaw=psi_new,
            bank=phi,
            **odometry_update(state, v_gs, dt),
        )
