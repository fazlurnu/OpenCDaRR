"""Functional tests for VO (shortest-way-out) resolution.

Core check: the shortest-way-out velocity lands on the collision-cone edge, so applying it makes
the ownship trajectory tangent to the resolution zone — the miss opens to (margin ×) rpz.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from opencdarr import geo
from opencdarr.cr import VO
from opencdarr.dynamics import Command
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0


def _own() -> AircraftState:
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)


def _miss_distance(own: AircraftState, intr: AircraftState) -> float:
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    rx, ry = dist * math.sin(q), dist * math.cos(q)
    vx = intr.gs * math.sin(math.radians(intr.trk)) - own.gs * math.sin(math.radians(own.trk))
    vy = intr.gs * math.cos(math.radians(intr.trk)) - own.gs * math.cos(math.radians(own.trk))
    v2 = vx * vx + vy * vy
    t_cpa = -(rx * vx + ry * vy) / v2
    return math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)


def _apply(own: AircraftState, cmd: Command) -> AircraftState:
    return dataclasses.replace(own, trk=cmd.hdg, gs=cmd.spd)


@pytest.mark.parametrize("dpsi", [30.0, 90.0, 135.0, 250.0])
@pytest.mark.parametrize("dcpa", [0.0, 20.0, 40.0])
def test_vo_opens_miss_to_zone(dpsi: float, dcpa: float) -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=dcpa, tlos=60.0, rpz=_RPZ)
    assert _miss_distance(own, intr) < _RPZ  # genuinely a conflict to start

    cmd = VO().resolve(own, intr, _RPZ)
    # shortest way out -> velocity on the cone edge -> trajectory tangent to the zone
    assert _miss_distance(_apply(own, cmd), intr) == pytest.approx(_RPZ, abs=0.5)


def test_vo_margin_opens_beyond_rpz() -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    cmd = VO(margin=1.2).resolve(own, intr, _RPZ)
    assert _miss_distance(_apply(own, cmd), intr) == pytest.approx(1.2 * _RPZ, abs=0.5)


def test_vo_is_the_minimal_change_to_the_edge() -> None:
    """Shortest way out: the resolved speed change is small (edge is near the current velocity)."""
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    cmd = VO().resolve(own, intr, _RPZ)
    assert abs(cmd.spd - own.gs) < own.gs  # a fraction of the speed, not a reversal


def test_vo_returns_a_command() -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    cmd = VO().resolve(own, intr, _RPZ)
    assert isinstance(cmd, Command)
    assert 0.0 <= cmd.hdg < 360.0
