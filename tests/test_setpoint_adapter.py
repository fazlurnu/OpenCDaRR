"""The velocity→fixed-wing setpoint projection (Phase 4e, ADR 0013 §4).

A resolver emits a ground-**velocity** command; a fixed-wing cannot fly it (it takes course +
airspeed). :func:`~opencdarr.separation.project_to_fixedwing` bridges the gap. Two properties:

1. the projection maps a velocity to the right ``(target_course, target_airspeed)`` and keeps the
   airspeed inside the airframe envelope (clamped to ``[v_min, v_max]``);
2. the projected setpoint is one a :class:`~opencdarr.dynamics.FixedWing` actually **converges** to
   — it turns onto the commanded course and never violates the stall / bank envelope on the way.
"""

from __future__ import annotations

import math

from opencdarr.dynamics import FixedWing, MotionCommand
from opencdarr.performance import SMALL_FIXEDWING as P
from opencdarr.separation import project_to_fixedwing
from opencdarr.state import AircraftState


def test_velocity_maps_to_course_and_airspeed() -> None:
    """A velocity at bearing 60 deg, speed 17 m/s -> course 60, airspeed 17 (inside envelope)."""
    speed, course = 17.0, 60.0
    r = math.radians(course)
    cmd = MotionCommand(target_velocity=(speed * math.sin(r), speed * math.cos(r)))
    out = project_to_fixedwing(cmd, P)
    assert out.target_velocity is None  # the raw velocity channel is consumed
    assert out.target_course is not None and abs(out.target_course - course) < 1e-9
    assert out.target_airspeed is not None and abs(out.target_airspeed - speed) < 1e-9


def test_airspeed_is_clamped_into_the_envelope() -> None:
    """A too-fast / too-slow avoidance velocity clamps to v_max / v_min (never leaves the box)."""
    fast = project_to_fixedwing(MotionCommand(target_velocity=(0.0, 100.0)), P)
    slow = project_to_fixedwing(MotionCommand(target_velocity=(0.0, 3.0)), P)
    assert fast.target_airspeed == P.v_max
    assert slow.target_airspeed == P.v_min


def test_position_command_passes_through_untouched() -> None:
    """A position/leg nominal is already a fixed-wing setpoint — it must pass through unchanged."""
    nominal = MotionCommand(target_position=(52.01, 4.0), target_leg_start=(52.0, 4.0),
                            target_airspeed=17.0)
    assert project_to_fixedwing(nominal, P) is nominal


def test_fixedwing_converges_to_the_projected_setpoint() -> None:
    """The projected (course, airspeed) is feasible: the fixed-wing turns onto it, envelope-safe.

    It converges to the commanded course and airspeed without ever leaving the stall/bank envelope.
    """
    fw = FixedWing()
    dt = 0.1
    # an avoidance velocity 70 deg off the nose at 20 m/s -> a real turn the airframe must fly out
    r = math.radians(70.0)
    cmd = project_to_fixedwing(MotionCommand(target_velocity=(20.0 * math.sin(r),
                                                              20.0 * math.cos(r))), P)
    s = AircraftState(id="F", lat=52.0, lon=4.0, trk=0.0, gs=17.0, yaw=0.0, bank=0.0)
    for _ in range(400):  # 40 s
        s = fw.step(s, cmd, P, dt)
        assert s.gs >= P.v_min - 1e-6  # never stalls (airspeed stays above v_min)
        assert abs(s.bank) <= P.phi_max + 1e-6  # never exceeds the structural bank limit
    assert abs(((s.yaw - 70.0 + 180.0) % 360.0) - 180.0) < 0.5  # converged onto the course
    assert abs(s.gs - 20.0) < 1e-6  # ramped to the commanded airspeed
