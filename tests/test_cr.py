"""Functional tests for MVP resolution.

Core check: applying the returned command to the ownship opens the miss distance to (at
least) the resolution zone — the resolver does real geometric work.
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from opencdarr import geo
from opencdarr.cr import MVP
from opencdarr.dynamics import Command
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0


def _own() -> AircraftState:
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)


def _miss_distance(own: AircraftState, intr: AircraftState) -> float:
    """dcpa for the pair as they stand now (positions fixed, current velocities)."""
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    rx, ry = dist * math.sin(q), dist * math.cos(q)
    vx = intr.gs * math.sin(math.radians(intr.trk)) - own.gs * math.sin(math.radians(own.trk))
    vy = intr.gs * math.cos(math.radians(intr.trk)) - own.gs * math.cos(math.radians(own.trk))
    v2 = vx * vx + vy * vy
    t_cpa = -(rx * vx + ry * vy) / v2
    return math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)


def _apply(own: AircraftState, cmd: Command) -> AircraftState:
    """Adopt the commanded velocity instantly (unclamped) to test the raw MVP geometry."""
    return dataclasses.replace(own, trk=cmd.hdg, gs=cmd.spd)


@pytest.mark.parametrize("dpsi", [30.0, 90.0, 135.0, 180.0, 250.0])
@pytest.mark.parametrize("dcpa", [0.0, 20.0, 40.0])
def test_resolution_opens_miss_to_zone(dpsi: float, dcpa: float) -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=dcpa, tlos=60.0, rpz=_RPZ)
    assert _miss_distance(own, intr) < _RPZ  # genuinely a conflict to start

    cmd = MVP().resolve(own, intr, _RPZ)
    resolved = _apply(own, cmd)
    # own alone maneuvers to make its trajectory tangent to the zone -> miss ~ rpz
    assert _miss_distance(resolved, intr) == pytest.approx(_RPZ, abs=0.5)


def test_margin_opens_beyond_rpz() -> None:
    """A margin > 1 clears with a buffer: miss reaches ~margin*rpz."""
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    cmd = MVP(margin=1.2).resolve(own, intr, _RPZ)
    assert _miss_distance(_apply(own, cmd), intr) == pytest.approx(1.2 * _RPZ, abs=0.5)


def test_resolve_returns_a_command() -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    cmd = MVP().resolve(own, intr, _RPZ)
    assert isinstance(cmd, Command)
    assert 0.0 <= cmd.hdg < 360.0


def test_no_relative_motion_returns_nominal() -> None:
    """Parallel, equal speed -> nothing to resolve -> unchanged command."""
    own = _own()
    intr = AircraftState(id="INT", lat=52.0, lon=4.009, trk=0.0, gs=10.0)
    cmd = MVP().resolve(own, intr, _RPZ)
    assert cmd.hdg == pytest.approx(own.trk)
    assert cmd.spd == pytest.approx(own.gs)
