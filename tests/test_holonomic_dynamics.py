"""Analytical validation of `HolonomicDynamics` (ADR 0009).

Mirrors `test_dynamics.py`'s style for `step_dynamics`/`DubinsDynamics`: first-principles
checks on the envelope and acceleration limit, plus the degenerate reversal case that makes the
"no coupled heading" property concrete and checkable (not just visible in a plot).
"""

from __future__ import annotations

import dataclasses
import math

from opencdarr import geo
from opencdarr.dynamics import Command, HolonomicDynamics
from opencdarr.kinematics import velocity_enu
from opencdarr.performance import M600
from opencdarr.state import AircraftState

_HOLO = HolonomicDynamics()


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres (haversine at the local WGS84 radius)."""
    radius = geo.earth_radius((lat1 + lat2) / 2.0)
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(a))


def _start(trk: float = 90.0, gs: float = 10.0) -> AircraftState:
    return AircraftState(id="D0", lat=52.0, lon=4.0, trk=trk, gs=gs)


def test_straight_line_travels_expected_distance() -> None:
    """10 m/s held straight for 10 s covers ~100 m — same sanity check as the point-mass model."""
    s = _start(trk=90.0, gs=10.0)
    cmd = Command.from_track_speed(90.0, 10.0)
    for _ in range(10):
        s = _HOLO.step(s, cmd, M600, dt=1.0)
    assert abs(_distance_m(52.0, 4.0, s.lat, s.lon) - 100.0) < 0.01
    assert s.trk == 90.0


def test_never_exceeds_v_max_envelope() -> None:
    """A command above v_max ramps up to it (never past), same envelope as DubinsDynamics."""
    dt = 0.1
    s = _start(trk=45.0, gs=0.0)
    cmd = Command.from_track_speed(45.0, 30.0)  # above M600.v_max = 18
    reached = False
    for _ in range(400):
        s = _HOLO.step(s, cmd, M600, dt)
        assert s.gs <= M600.v_max + 1e-9
        if abs(s.gs - M600.v_max) < 1e-6:
            reached = True
            break
    assert reached, "speed did not converge to v_max"


def test_acceleration_is_bounded_isotropically() -> None:
    """The velocity *vector* changes by at most ax*dt per step, in any direction — a single
    isotropic limit, not two independent 1D limits (that would be the point-mass model)."""
    dt = 0.1
    s = _start(trk=0.0, gs=5.0)
    cmd = Command.from_track_speed(120.0, 15.0)  # a large, oblique direction+speed change
    prev_e, prev_n = velocity_enu(s)
    for _ in range(300):
        s = _HOLO.step(s, cmd, M600, dt)
        cur_e, cur_n = velocity_enu(s)
        step_mag = math.hypot(cur_e - prev_e, cur_n - prev_n)
        assert step_mag <= M600.ax * dt + 1e-9
        prev_e, prev_n = cur_e, cur_n


def test_reversal_travels_a_straight_line_not_a_loop() -> None:
    """Commanding the exact opposite of the current velocity: the vector ramps down through zero
    and back up *without ever picking up an East component* — a straight north-south path. A
    turn-rate-limited (Dubins) model cannot do this; it must sweep through a wide arc instead
    (see the trajectory comparison in the vault)."""
    dt = 0.1
    s = _start(trk=0.0, gs=10.0)  # flying north
    cmd = Command.from_track_speed(180.0, 10.0)  # command due south
    min_gs = s.gs
    for _ in range(300):
        s = _HOLO.step(s, cmd, M600, dt)
        ve, _vn = velocity_enu(s)
        assert abs(ve) < 1e-9  # no East component ever appears
        min_gs = min(min_gs, s.gs)
    assert min_gs < M600.ax * dt + 1e-6  # passed through (near) zero speed on the way
    assert abs(s.gs - 10.0) < 1e-6  # converged to the commanded speed
    assert abs(((s.trk - 180.0 + 180.0) % 360.0) - 180.0) < 1e-3  # ... now heading south


def test_zero_command_holds_heading_while_stopping() -> None:
    """Commanded zero velocity decelerates to a stop; once stopped, track holds at whatever it
    last was rather than snapping to the arbitrary zero-vector direction."""
    dt = 0.1
    s = _start(trk=45.0, gs=10.0)
    zero = Command(v_east=0.0, v_north=0.0)
    for _ in range(300):
        s = _HOLO.step(s, zero, M600, dt)
        if s.gs == 0.0:
            break
    assert s.gs == 0.0
    trk_at_stop = s.trk
    s = _HOLO.step(s, zero, M600, dt)  # one more step at rest
    assert s.trk == trk_at_stop  # heading did not move once stopped


def test_odometry_accumulates_like_dubins() -> None:
    """HolonomicDynamics advances flight_time / distance_flown the same way (shared helper,
    ADR 0010): straight cruise at 10 m/s for 5 s -> 5 s elapsed, ~50 m flown."""
    s = _start(trk=90.0, gs=10.0)
    cmd = Command.from_track_speed(90.0, 10.0)
    for _ in range(50):
        s = _HOLO.step(s, cmd, M600, dt=0.1)
    assert abs(s.flight_time - 5.0) < 1e-9
    assert abs(s.distance_flown - 50.0) < 1e-9


def test_step_does_not_mutate_input() -> None:
    """The input state is untouched; a new object is returned (safe to clone/parallelise)."""
    s = _start(trk=0.0, gs=10.0)
    snapshot = dataclasses.replace(s)
    out = _HOLO.step(s, Command.from_track_speed(90.0, 15.0), M600, dt=0.1)
    assert s == snapshot
    assert out is not s
