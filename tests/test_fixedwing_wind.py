"""Fixed-wing under wind (Phase 5c) — crab guidance and trochoidal ground tracks.

Analytical validation against the source paper's kinematics (the
[[0002-analytical-validation-of-dynamics]] discipline), not eyeballing:

- holding a ground course, the crab ``ψ − χ`` matches Eq 3 and the ground speed matches Eq 4;
- a constant-bank turn traces a **circle in the air frame** and a **trochoid over the ground**: one
  full revolution's net ground drift is ``wind × turn-period`` (the air-circle nets to zero);
- an L1 leg is made good *over the ground* under a crosswind (the track, not the nose, follows it);
- and ``V_WS = 0`` reproduces the Phase-4 fixed-wing byte-for-byte.
"""

from __future__ import annotations

import math

from opencdarr import geo
from opencdarr.dynamics import FixedWing, MotionCommand
from opencdarr.performance import SMALL_FIXEDWING as P
from opencdarr.relative import ground_speed, wind_correction_angle
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField

_FW = FixedWing()
_V = 17.0


def _state(trk: float = 0.0) -> AircraftState:
    return AircraftState(id="F", lat=52.0, lon=4.0, trk=trk, gs=_V, yaw=trk, bank=0.0)


def _enu(lat: float, lon: float) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(52.0, 4.0, lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


def test_crab_holds_the_ground_course() -> None:
    """A commanded due-north ground course in a crosswind: track goes north, ψ crabs (Eq 3)."""
    wind = WindField.from_met(270.0, 5.0)  # 5 m/s from the west
    cmd = MotionCommand(target_course=0.0, target_airspeed=_V)
    s = _state()
    for _ in range(2000):  # 200 s to steady state
        s = _FW.step(s, cmd, P, 0.1, wind)
    assert abs(((s.trk - 0.0 + 180.0) % 360.0) - 180.0) < 0.3  # ground track is due north
    crab = ((s.yaw - s.trk + 180.0) % 360.0) - 180.0
    expected = wind_correction_angle(_V, wind, 0.0)
    assert expected is not None
    assert abs(crab - expected) < 0.3  # heading leads the track by the Eq-3 crab


def test_ground_speed_matches_eq4() -> None:
    """At steady state the ground speed equals the Eq-4 closed form for the flown heading."""
    wind = WindField.from_met(300.0, 6.0)
    cmd = MotionCommand(target_course=20.0, target_airspeed=_V)
    s = _state(trk=20.0)
    for _ in range(2000):
        s = _FW.step(s, cmd, P, 0.1, wind)
    assert s.yaw is not None
    assert math.isclose(s.gs, ground_speed(_V, wind, s.yaw), abs_tol=0.05)


def _turn_rows(wind: WindField, tmax: float = 40.0, dt: float = 0.05) -> list[tuple[float, ...]]:
    """A continuous max-bank right turn (airspeed_direction always 90° ahead); tracks heading."""
    s = _state()
    rows: list[tuple[float, ...]] = []
    t, cum, prev = 0.0, 0.0, s.yaw if s.yaw is not None else s.trk
    while t < tmax:
        e, n = _enu(s.lat, s.lon)
        rows.append((t, e, n, cum))
        cmd = MotionCommand(target_airspeed_direction=((s.yaw or 0.0) + 90.0) % 360.0,
                            target_airspeed=_V)
        s = _FW.step(s, cmd, P, dt, wind)
        cum += ((s.yaw - prev + 180.0) % 360.0) - 180.0 if s.yaw is not None else 0.0
        prev = s.yaw if s.yaw is not None else prev
        t += dt
    return rows


def test_constant_bank_turn_is_a_trochoid() -> None:
    """One revolution's net ground drift is wind × period (the air-circle nets to zero)."""
    we, wn = 5.0, 0.0  # wind velocity vector: air moves east at 5 m/s (a west wind)
    wind = WindField(we, wn)
    rows = _turn_rows(wind)
    cum = [r[3] for r in rows]
    # revolution boundaries: cumulative heading crossing 360° (i0) and 720° (i1) — bank saturated
    i0 = next(i for i, c in enumerate(cum) if abs(c) >= 360.0)
    i1 = next(i for i, c in enumerate(cum) if abs(c) >= 720.0)
    period = rows[i1][0] - rows[i0][0]
    net_e = rows[i1][1] - rows[i0][1]
    net_n = rows[i1][2] - rows[i0][2]
    assert math.isclose(net_e, we * period, rel_tol=0.05)  # drifts downwind by wind × period
    assert abs(net_n - wn * period) < 3.0  # no net cross-wind displacement over a full turn


def test_l1_leg_is_made_good_over_ground_under_wind() -> None:
    """A due-north leg is tracked by the *ground* track under a crosswind (cross-track -> 0)."""
    wind = WindField.from_met(270.0, 5.0)
    b = geo.forward(52.0, 4.0, 0.0, 1500.0)  # leg due north
    cmd = MotionCommand(target_position=(b[0], b[1]), target_leg_start=(52.0, 4.0),
                        target_airspeed=_V)
    s = _state()
    for _ in range(700):  # 70 s
        s = _FW.step(s, cmd, P, 0.1, wind)
    east, _ = _enu(s.lat, s.lon)
    assert abs(east) < 3.0  # the ground track stayed on the (north) leg line despite the crosswind


def test_no_wind_reproduces_phase4_fixedwing() -> None:
    """With NO_WIND the step is byte-identical to omitting the wind argument (Phase 4)."""
    s = AircraftState(id="F", lat=52.0, lon=4.0, trk=10.0, gs=18.0, yaw=10.0, bank=5.0)
    cmd = MotionCommand(target_course=40.0, target_airspeed=19.0)
    assert _FW.step(s, cmd, P, 0.1) == _FW.step(s, cmd, P, 0.1, NO_WIND)
