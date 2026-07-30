"""Analytical validation of :class:`Multirotor` (ADR 0012).

Two jobs:

1. **Migration gate (D2).** The translation core reproduces the former ``HolonomicDynamics`` (ADR
   0009) byte-for-byte — these are that model's analytical checks, ported verbatim onto
   ``Multirotor``: the envelope, the isotropic acceleration limit, the reversal-without-a-loop
   case, and the shared odometry. If they pass, the BlueSky-transferred validation carried across
   the hard replace.
2. **New capability — independent yaw.** The nose heading ``yaw`` converges toward ``target_yaw``
   (or integrates ``target_yawspeed``) decoupled from translation; ``trk`` and ``yaw`` never
   re-couple. Plus the feasibility taxonomy: a missing velocity channel fails fast, fixed-wing
   channels are ignored (absent DOF).
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from opencdarr import geo
from opencdarr.dynamics import MotionCommand, Multirotor
from opencdarr.performance import M600
from opencdarr.relative import velocity_enu
from opencdarr.state import AircraftState

_MR = Multirotor()


def _distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres (haversine at the local WGS84 radius)."""
    radius = geo.earth_radius((lat1 + lat2) / 2.0)
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(a))


def _start(trk: float = 90.0, gs: float = 10.0, yaw: float | None = None) -> AircraftState:
    return AircraftState(id="D0", lat=52.0, lon=4.0, trk=trk, gs=gs, yaw=yaw)


# --- Migration gate: the Holonomic (ADR 0009) analytical anchors, on Multirotor ---------------


def test_straight_line_travels_expected_distance() -> None:
    """10 m/s held straight for 10 s covers ~100 m; track holds."""
    s = _start(trk=90.0, gs=10.0)
    cmd = MotionCommand.from_track_speed(90.0, 10.0)
    for _ in range(10):
        s = _MR.step(s, cmd, M600, dt=1.0)
    assert abs(_distance_m(52.0, 4.0, s.lat, s.lon) - 100.0) < 0.01
    assert s.trk == 90.0


def test_never_exceeds_v_max_envelope() -> None:
    """A command above v_max ramps up to it (never past)."""
    dt = 0.1
    s = _start(trk=45.0, gs=0.0)
    cmd = MotionCommand.from_track_speed(45.0, 30.0)  # above M600.v_max = 18
    reached = False
    for _ in range(400):
        s = _MR.step(s, cmd, M600, dt)
        assert s.gs <= M600.v_max + 1e-9
        if abs(s.gs - M600.v_max) < 1e-6:
            reached = True
            break
    assert reached, "speed did not converge to v_max"


def test_acceleration_is_bounded_isotropically() -> None:
    """The velocity *vector* changes by at most ax*dt per step, in any direction."""
    dt = 0.1
    s = _start(trk=0.0, gs=5.0)
    cmd = MotionCommand.from_track_speed(120.0, 15.0)  # a large, oblique direction+speed change
    prev_e, prev_n = velocity_enu(s)
    for _ in range(300):
        s = _MR.step(s, cmd, M600, dt)
        cur_e, cur_n = velocity_enu(s)
        step_mag = math.hypot(cur_e - prev_e, cur_n - prev_n)
        assert step_mag <= M600.ax * dt + 1e-9
        prev_e, prev_n = cur_e, cur_n


def test_reversal_travels_a_straight_line_not_a_loop() -> None:
    """Commanding the exact opposite velocity: the vector ramps down through zero and back up
    *without ever picking up an East component* — a straight path a turn-rate model cannot fly."""
    dt = 0.1
    s = _start(trk=0.0, gs=10.0)  # flying north
    cmd = MotionCommand.from_track_speed(180.0, 10.0)  # command due south
    min_gs = s.gs
    for _ in range(300):
        s = _MR.step(s, cmd, M600, dt)
        ve, _vn = velocity_enu(s)
        assert abs(ve) < 1e-9  # no East component ever appears
        min_gs = min(min_gs, s.gs)
    assert min_gs < M600.ax * dt + 1e-6  # passed through (near) zero speed on the way
    assert abs(s.gs - 10.0) < 1e-6  # converged to the commanded speed
    assert abs(((s.trk - 180.0 + 180.0) % 360.0) - 180.0) < 1e-3  # ... now heading south


def test_odometry_accumulates_via_shared_helper() -> None:
    """Multirotor advances flight_time / distance_flown the same way (ADR 0010): 10 m/s for 5 s."""
    s = _start(trk=90.0, gs=10.0)
    cmd = MotionCommand.from_track_speed(90.0, 10.0)
    for _ in range(50):
        s = _MR.step(s, cmd, M600, dt=0.1)
    assert abs(s.flight_time - 5.0) < 1e-9
    assert abs(s.distance_flown - 50.0) < 1e-9


def test_step_does_not_mutate_input() -> None:
    """The input state is untouched; a new object is returned (safe to clone/parallelise)."""
    s = _start(trk=0.0, gs=10.0)
    snapshot = dataclasses.replace(s)
    out = _MR.step(s, MotionCommand.from_track_speed(90.0, 15.0), M600, dt=0.1)
    assert s == snapshot
    assert out is not s


# --- New capability: independent yaw (the D3 headline) ----------------------------------------


def test_yaw_converges_independent_of_track() -> None:
    """(velocity east, yaw 45°): the multirotor keeps travelling east while its nose turns to 45° —
    the two channels never re-couple."""
    s = _start(trk=90.0, gs=10.0)  # flying east, no independent yaw yet (yaw is None -> track)
    cmd = MotionCommand(target_velocity=velocity_enu(_start(trk=90.0, gs=10.0)), target_yaw=45.0)
    for _ in range(200):
        s = _MR.step(s, cmd, M600, dt=0.1)
    assert abs(((s.trk - 90.0 + 180.0) % 360.0) - 180.0) < 1e-9  # still travelling east
    assert s.yaw is not None
    assert abs(((s.yaw - 45.0 + 180.0) % 360.0) - 180.0) < 1e-9  # nose settled at 45°
    # and it stays decoupled: more steps do not move either channel
    s2 = _MR.step(s, cmd, M600, dt=0.1)
    assert s2.trk == s.trk and s2.yaw == s.yaw


def test_yaw_is_rate_limited() -> None:
    """Yaw moves by at most yaw_rate_max*dt per step on its way to the target."""
    dt = 0.1
    s = _start(trk=90.0, gs=10.0, yaw=0.0)
    cmd = MotionCommand(target_velocity=velocity_enu(_start(90.0, 10.0)), target_yaw=170.0)
    prev = s.yaw
    for _ in range(100):
        s = _MR.step(s, cmd, M600, dt)
        assert prev is not None and s.yaw is not None
        d = abs(((s.yaw - prev + 180.0) % 360.0) - 180.0)
        assert d <= M600.yaw_rate_max * dt + 1e-9
        prev = s.yaw


def test_uncommanded_yaw_holds() -> None:
    """No yaw command -> yaw is held: None stays track-aligned, a concrete heading stays put."""
    cmd = MotionCommand.from_track_speed(90.0, 10.0)  # velocity only, no yaw channel
    assert _MR.step(_start(trk=90.0, gs=10.0, yaw=None), cmd, M600, 0.1).yaw is None
    assert _MR.step(_start(trk=90.0, gs=10.0, yaw=30.0), cmd, M600, 0.1).yaw == 30.0


def test_target_yawspeed_integrates() -> None:
    """target_yawspeed (with no target_yaw) integrates a rate, clamped to yaw_rate_max."""
    s = _start(trk=90.0, gs=10.0, yaw=0.0)
    cmd = MotionCommand(target_velocity=velocity_enu(_start(90.0, 10.0)), target_yawspeed=30.0)
    for _ in range(10):
        s = _MR.step(s, cmd, M600, dt=0.1)  # 30 deg/s * 1.0 s
    assert s.yaw is not None and abs(s.yaw - 30.0) < 1e-9


def test_hover_holds_position_and_yaw() -> None:
    """Zero velocity decelerates to a stop, then holds position; uncommanded yaw holds too."""
    dt = 0.1
    s = _start(trk=45.0, gs=10.0, yaw=45.0)
    zero = MotionCommand(target_velocity=(0.0, 0.0))
    for _ in range(300):
        s = _MR.step(s, zero, M600, dt)
        if s.gs == 0.0:
            break
    assert s.gs == 0.0
    at_rest = _MR.step(s, zero, M600, dt)  # one more step, hovering
    assert at_rest.gs == 0.0
    assert at_rest.lat == s.lat and at_rest.lon == s.lon  # position held
    assert at_rest.yaw == 45.0  # yaw held


# --- Feasibility taxonomy (ADR 0011 §1 / ADR 0012) --------------------------------------------


def test_missing_velocity_channel_fails_fast() -> None:
    """A command with no velocity channel is under-specified for a multirotor -> fail fast."""
    cmd = MotionCommand(target_yaw=45.0)  # yaw but no translation
    with pytest.raises(ValueError, match="no target_velocity"):
        _MR.step(_start(), cmd, M600, 0.1)


def test_fixed_wing_channels_are_ignored() -> None:
    """target_course / target_airspeed (fixed-wing channels) are an absent DOF here: no-ops."""
    s = _start(trk=0.0, gs=10.0, yaw=10.0)
    base = MotionCommand(target_velocity=(3.0, 4.0))
    with_fw = MotionCommand(
        target_velocity=(3.0, 4.0), target_course=200.0, target_airspeed=99.0
    )
    assert _MR.step(s, base, M600, 0.1) == _MR.step(s, with_fw, M600, 0.1)


# --- Body-frame velocity channel (PX4 MAV_FRAME_BODY_FRD) -------------------------------------


def test_body_forward_resolves_through_yaw() -> None:
    """A body-forward command is nose-relative: nose north -> travels north; nose east -> east."""
    # nose north (yaw=0): forward -> north
    n = _MR.step(_start(trk=0.0, gs=0.0, yaw=0.0),
                 MotionCommand(target_body_velocity=(10.0, 0.0)), M600, 1.0)
    ve, vn = velocity_enu(n)
    assert abs(ve) < 1e-9 and vn > 0.0
    # nose east (yaw=90): the *same* forward command now goes east
    e = _MR.step(_start(trk=0.0, gs=0.0, yaw=90.0),
                 MotionCommand(target_body_velocity=(10.0, 0.0)), M600, 1.0)
    ve, vn = velocity_enu(e)
    assert ve > 0.0 and abs(vn) < 1e-9


def test_body_right_is_ninety_clockwise_from_nose() -> None:
    """Body-right with the nose north points east (90° CW from forward)."""
    s = _MR.step(_start(trk=0.0, gs=0.0, yaw=0.0),
                 MotionCommand(target_body_velocity=(0.0, 10.0)), M600, 1.0)
    ve, vn = velocity_enu(s)
    assert ve > 0.0 and abs(vn) < 1e-9


def test_body_forward_matches_equivalent_inertial_command() -> None:
    """At a fixed yaw, a body-forward command equals the inertial command it rotates to."""
    s = _start(trk=0.0, gs=5.0, yaw=90.0)  # nose east
    body = _MR.step(s, MotionCommand(target_body_velocity=(12.0, 0.0)), M600, 0.3)
    inertial = _MR.step(s, MotionCommand(target_velocity=(12.0, 0.0)), M600, 0.3)  # 12 east
    assert body == inertial


def test_body_forward_while_yawing_curves_the_inertial_path() -> None:
    """Body-forward held while the nose slews turns the inertial travel with the yaw (the reason a
    body-frame nominal cannot be a frozen constant)."""
    s = _start(trk=0.0, gs=10.0, yaw=0.0)
    cmd = MotionCommand(target_body_velocity=(10.0, 0.0), target_yaw=90.0)
    for _ in range(200):
        s = _MR.step(s, cmd, M600, 0.1)
    assert s.yaw is not None and abs(((s.yaw - 90.0 + 180.0) % 360.0) - 180.0) < 1e-6  # nose east
    assert abs(((s.trk - 90.0 + 180.0) % 360.0) - 180.0) < 0.5  # travel followed the nose to east
