"""Phase 1 gate — analytical validation of `step_dynamics` (ADR 0002).

Three first-principles checks (the acceptance criteria) plus a couple of bite tests that
would catch a plausibly-wrong implementation. Pure and numpy-only: no BlueSky. The BlueSky
equivalence anchor lives in ``test_dynamics_vs_bluesky.py`` and is skipped where BlueSky is
absent.
"""

from __future__ import annotations

import dataclasses
import math

from opencdarr import geo
from opencdarr.dynamics import Command, step_dynamics
from opencdarr.performance import M600
from opencdarr.state import AircraftState


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


# --- Check 1: straight-line distance -----------------------------------------


def test_straight_line_travels_expected_distance() -> None:
    """10 m/s held straight for 10 s covers ~100 m; heading and turn rate unchanged."""
    s = _start(trk=90.0, gs=10.0)
    cmd = Command(hdg=90.0, spd=10.0)
    for _ in range(10):
        s = step_dynamics(s, cmd, M600, dt=1.0)
    assert abs(_distance_m(52.0, 4.0, s.lat, s.lon) - 100.0) < 0.01
    assert s.trk == 90.0
    assert s.turn_rate == 0.0


# --- Check 2: turn respects the limits, speed held ---------------------------


def test_turn_respects_limits_and_holds_speed() -> None:
    """A 90 deg command turns within max_tr / max_dtr2, holds speed, and converges."""
    dt = 0.1
    s = _start(trk=0.0, gs=10.0)
    cmd = Command(hdg=90.0, spd=10.0)
    prev_tr = s.turn_rate
    reached = False
    for _ in range(400):
        s = step_dynamics(s, cmd, M600, dt)
        assert abs(s.turn_rate) <= M600.max_tr + 1e-9  # never exceeds max turn rate
        assert abs(s.turn_rate - prev_tr) <= M600.max_dtr2 * dt + 1e-9  # bounded change
        assert s.gs == 10.0  # turning does not bleed the commanded speed
        prev_tr = s.turn_rate
        if abs(((90.0 - s.trk + 180.0) % 360.0) - 180.0) < 0.5:
            reached = True
            break
    assert reached, "did not converge to the commanded heading"


def test_turn_takes_the_shortest_way() -> None:
    """Commanding 350 deg from 0 turns the short way (negative), not +350."""
    s = _start(trk=0.0, gs=10.0)
    s = step_dynamics(s, Command(hdg=350.0, spd=10.0), M600, dt=0.1)
    assert s.turn_rate < 0.0


# --- Check 3: speed cap ------------------------------------------------------


def test_speed_ramps_to_envelope_within_accel_limit() -> None:
    """Commanding 30 m/s ramps up to v_max=18 (never past), bounded by ax*dt each step."""
    dt = 0.1
    s = _start(trk=90.0, gs=0.0)  # start from rest
    cmd = Command(hdg=90.0, spd=30.0)  # above the envelope
    prev = s.gs
    reached = False
    for _ in range(400):
        s = step_dynamics(s, cmd, M600, dt)
        assert abs(s.gs - prev) <= M600.ax * dt + 1e-9  # bounded acceleration
        assert s.gs <= M600.v_max + 1e-9  # never exceeds the envelope
        prev = s.gs
        if abs(s.gs - M600.v_max) < 1e-9:
            reached = True
            break
    assert reached, "speed did not converge to v_max"


def test_speed_ramps_down_to_v_min() -> None:
    """Commanding -30 m/s clamps the target to v_min=-18 and ramps down to it."""
    s = _start(trk=90.0, gs=0.0)
    for _ in range(400):
        s = step_dynamics(s, Command(hdg=90.0, spd=-30.0), M600, dt=0.1)
    assert abs(s.gs - M600.v_min) < 1e-9


# --- Purity ------------------------------------------------------------------


def test_step_does_not_mutate_input() -> None:
    """The input state is untouched; a new object is returned (safe to clone/parallelise)."""
    s = _start(trk=0.0, gs=10.0)
    snapshot = dataclasses.replace(s)
    out = step_dynamics(s, Command(hdg=90.0, spd=15.0), M600, dt=0.1)
    assert s == snapshot
    assert out is not s
