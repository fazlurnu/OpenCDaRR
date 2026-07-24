"""Analytical validation of :class:`FixedWing` (ADR 0013, ADR 0002 discipline).

The fixed-wing model is validated against the paper's closed forms, not against BlueSky (deleting
Dubins retired the BlueSky trajectory anchor). The load-bearing checks:

- **wind-readiness (w=0):** heading ψ equals course χ and ground speed equals airspeed every step —
  the Phase-5 wind hook is inert until a non-zero wind vector is fed;
- **steady-turn radius** matches ``R = V²/(g·tan φ)`` at the settled bank;
- **finite roll:** bank changes by at most ``roll_rate_max·dt`` per step, and the heading change
  accumulated during a roll-in matches the paper's closed form (Eq 15);
- the airframe **cannot stop** (airspeed clamped ≥ stall) and **cannot side-slip** (no velocity
  channel; ground velocity is always along the course);
- **stall-in-turn:** the effective bank limit shrinks as airspeed approaches stall.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from opencdarr import geo
from opencdarr.dynamics import FixedWing, MotionCommand
from opencdarr.performance import SMALL_FIXEDWING as P
from opencdarr.state import AircraftState

_FW = FixedWing()
_G = 9.80665


def _start(trk: float = 0.0, gs: float = 17.0, bank: float = 0.0) -> AircraftState:
    return AircraftState(id="F0", lat=52.0, lon=4.0, trk=trk, gs=gs, yaw=trk, bank=bank)


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = geo.earth_radius((lat1 + lat2) / 2.0)
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(a))


def _ang_close(a: float, b: float, tol: float = 1e-6) -> bool:
    return abs(((a - b + 180.0) % 360.0) - 180.0) < tol


def _yaw(s: AircraftState) -> float:
    """The heading ψ, narrowed: a FixedWing always carries a concrete yaw (nose = airspeed vec)."""
    assert s.yaw is not None
    return s.yaw


# --- Straight flight + wind-readiness (w=0) ---------------------------------------------------


def test_straight_cruise_travels_expected_distance() -> None:
    """17 m/s held straight for 10 s covers ~170 m; heading holds and equals course."""
    s = _start(trk=90.0, gs=17.0)
    cmd = MotionCommand(target_course=90.0, target_airspeed=17.0)
    for _ in range(100):
        s = _FW.step(s, cmd, P, dt=0.1)
    assert abs(_distance_m(52.0, 4.0, s.lat, s.lon) - 170.0) < 0.5
    assert _ang_close(s.trk, 90.0) and _ang_close(_yaw(s), 90.0)
    assert s.bank == 0.0


def test_wind_readiness_heading_equals_course_and_gs_equals_airspeed() -> None:
    """At zero wind, ψ == χ and V_GS == V_TAS at EVERY step through a full turn — the inert
    Phase-5 hook. If this ever fails, wind was switched on by accident."""
    s = _start(trk=0.0, gs=17.0)
    cmd = MotionCommand(target_course=90.0, target_airspeed=17.0)
    for _ in range(400):
        s = _FW.step(s, cmd, P, dt=0.1)
        assert _ang_close(_yaw(s), s.trk, tol=1e-9)  # ψ == χ
        assert abs(s.gs - 17.0) < 1e-9  # V_GS == V_TAS (airspeed held)
    assert _ang_close(s.trk, 90.0)  # reached the commanded course


# --- Turning: radius, load factor, finite roll ------------------------------------------------


def test_steady_turn_radius_matches_closed_form() -> None:
    """Settled into a turn at φ_max, the radius equals V²/(g·tan φ) (paper Eq, §2.2)."""
    s = _start(trk=0.0, gs=17.0)
    cmd = MotionCommand(target_course=200.0, target_airspeed=17.0)  # a long turn -> saturates bank
    for _ in range(40):
        s = _FW.step(s, cmd, P, dt=0.1)
    assert abs(abs(s.bank) - P.phi_max) < 1e-6  # rolled to the bank limit
    turn_rate = math.degrees(_G * math.tan(math.radians(abs(s.bank))) / s.gs)  # deg/s
    radius = s.gs / math.radians(turn_rate)  # V / ψ̇
    expected = s.gs**2 / (_G * math.tan(math.radians(P.phi_max)))
    assert abs(radius - expected) < 1e-6


def test_finite_roll_rate_is_bounded() -> None:
    """Bank changes by at most roll_rate_max·dt each step (this is why bank is state)."""
    dt = 0.1
    s = _start(trk=0.0, gs=17.0)
    cmd = MotionCommand(target_course=180.0, target_airspeed=17.0)
    prev = s.bank
    for _ in range(200):
        s = _FW.step(s, cmd, P, dt)
        assert abs(s.bank - prev) <= P.roll_rate_max * dt + 1e-9
        prev = s.bank


def test_roll_in_heading_change_matches_eq15() -> None:
    """During a constant-rate roll-in from level to φ_max, the accumulated heading change matches
    the paper's closed form Δψ = (g/(V·p))·ln(cos φ_a / cos φ_b) (Eq 15)."""
    dt = 0.01  # fine step so the discrete roll-in tracks the continuous integral
    s = _start(trk=0.0, gs=17.0)
    cmd = MotionCommand(target_course=250.0, target_airspeed=17.0)  # saturates -> rolls at p_max
    psi0 = _yaw(s)
    p_rad = math.radians(P.roll_rate_max)
    while abs(s.bank) < P.phi_max - 1e-6:  # integrate through the roll-in only
        s = _FW.step(s, cmd, P, dt)
    dpsi_meas = abs(((_yaw(s) - psi0 + 180.0) % 360.0) - 180.0)
    dpsi_theory = (_G / (s.gs * p_rad)) * math.log(1.0 / math.cos(math.radians(P.phi_max)))
    dpsi_theory_deg = math.degrees(dpsi_theory)
    assert abs(dpsi_meas - dpsi_theory_deg) < 0.5  # first-order integrator vs continuous form


# --- Non-holonomic constraints ----------------------------------------------------------------


def test_cannot_stop_airspeed_clamped_to_stall() -> None:
    """Commanding an airspeed below stall clamps to v_min (>0): a fixed-wing cannot hover."""
    s = _start(trk=0.0, gs=17.0)
    cmd = MotionCommand(target_course=0.0, target_airspeed=2.0)  # well below stall (v_min=12)
    for _ in range(300):
        s = _FW.step(s, cmd, P, dt=0.1)
        assert s.gs >= P.v_min - 1e-9
    assert abs(s.gs - P.v_min) < 1e-6  # settled at the stall floor, never zero


def test_velocity_command_fails_fast_no_sideslip() -> None:
    """A raw velocity command carries no course/airspeed for a fixed-wing -> fail fast (it cannot
    be flown sideways; the velocity->course projection is Phase 4e)."""
    with pytest.raises(ValueError, match="no fixed-wing channel"):
        _FW.step(_start(), MotionCommand(target_velocity=(3.0, 4.0)), P, 0.1)


def test_multirotor_yaw_channel_is_ignored() -> None:
    """target_yaw (a multirotor channel) is an absent DOF: adding it to a fixed-wing command is a
    no-op (the nose is the airspeed vector, an output, not an independent input)."""
    s = _start(trk=0.0, gs=17.0)
    base = MotionCommand(target_course=45.0, target_airspeed=17.0)
    with_yaw = MotionCommand(target_course=45.0, target_airspeed=17.0, target_yaw=270.0)
    assert _FW.step(s, base, P, 0.1) == _FW.step(s, with_yaw, P, 0.1)


def test_stall_in_turn_tightens_the_bank_limit() -> None:
    """Near stall the load factor caps bank below φ_max; well above stall it reaches φ_max."""
    slow = _start(trk=0.0, gs=13.0)  # just above stall (12): stall-in-turn bites
    cmd_slow = MotionCommand(target_course=200.0, target_airspeed=13.0)
    peak_slow = 0.0  # peak bank reached during the turn (it rolls out once the turn completes)
    for _ in range(80):
        slow = _FW.step(slow, cmd_slow, P, dt=0.1)
        peak_slow = max(peak_slow, abs(slow.bank))
    phi_stall = math.degrees(math.acos((P.v_min / 13.0) ** 2))
    assert abs(peak_slow - min(P.phi_max, phi_stall)) < 1e-6  # bank capped by stall, not phi_max
    assert peak_slow < P.phi_max  # genuinely limited below the structural bank

    fast = _start(trk=0.0, gs=25.0)  # well above stall: full bank available
    cmd_fast = MotionCommand(target_course=200.0, target_airspeed=25.0)
    peak_fast = 0.0
    for _ in range(80):
        fast = _FW.step(fast, cmd_fast, P, dt=0.1)
        peak_fast = max(peak_fast, abs(fast.bank))
    assert abs(peak_fast - P.phi_max) < 1e-6  # full structural bank reachable well above stall


# --- Bookkeeping ------------------------------------------------------------------------------


def test_odometry_accumulates_via_shared_helper() -> None:
    """flight_time / distance_flown advance the shared way (ADR 0010): 17 m/s straight for 5 s."""
    s = _start(trk=90.0, gs=17.0)
    cmd = MotionCommand(target_course=90.0, target_airspeed=17.0)
    for _ in range(50):
        s = _FW.step(s, cmd, P, dt=0.1)
    assert abs(s.flight_time - 5.0) < 1e-9
    assert abs(s.distance_flown - 85.0) < 1e-6  # 17 m/s * 5 s


def test_step_does_not_mutate_input() -> None:
    s = _start(trk=0.0, gs=17.0)
    snapshot = dataclasses.replace(s)
    out = _FW.step(s, MotionCommand(target_course=90.0, target_airspeed=20.0), P, dt=0.1)
    assert s == snapshot
    assert out is not s
