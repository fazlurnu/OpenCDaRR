"""Functional tests for the conflict-encounter generator (`create_conflict`).

The core check: an encounter built for a requested (dcpa, tlos) reproduces exactly those
values when the generated pair is fed back through the CPA equations.
"""

from __future__ import annotations

import math

import pytest

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0
_DET = StateBased()


def _own() -> AircraftState:
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)


def _cpa_and_tlos(own: AircraftState, intr: AircraftState, rpz: float) -> tuple[float, float]:
    """Recover (dcpa, t_in) for a directed pair — the inverse of create_conflict."""
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    rx, ry = dist * math.sin(q), dist * math.cos(q)
    vx = intr.gs * math.sin(math.radians(intr.trk)) - own.gs * math.sin(math.radians(own.trk))
    vy = intr.gs * math.cos(math.radians(intr.trk)) - own.gs * math.cos(math.radians(own.trk))
    v2 = vx * vx + vy * vy
    t_cpa = -(rx * vx + ry * vy) / v2
    dcpa = math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)
    t_in = t_cpa - math.sqrt(rpz * rpz - dcpa * dcpa) / math.sqrt(v2)
    return dcpa, t_in


@pytest.mark.parametrize("dpsi", [30.0, 90.0, 135.0, 180.0, 225.0, 315.0])
@pytest.mark.parametrize("dcpa", [0.0, 20.0, 45.0])
def test_generated_encounter_reproduces_dcpa_and_tlos(dpsi: float, dcpa: float) -> None:
    own = _own()
    tlos = 60.0
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=dcpa, tlos=tlos, rpz=_RPZ)
    got_dcpa, got_tlos = _cpa_and_tlos(own, intr, _RPZ)
    assert got_dcpa == pytest.approx(dcpa, abs=1e-6)
    assert got_tlos == pytest.approx(tlos, abs=1e-6)


@pytest.mark.parametrize("dpsi", [30.0, 90.0, 180.0, 300.0])
def test_generated_encounter_is_detected_as_conflict(dpsi: float) -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    assert _DET.detect(own, intr, _RPZ, t_lookahead=120.0) is True


def test_intruder_track_is_own_plus_dpsi() -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    assert intr.trk == pytest.approx((own.trk + 90.0) % 360.0)


def test_side_mirrors_position_same_dcpa_tlos() -> None:
    own = _own()
    left = create_conflict(own, intr_id="L", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ, side=1)
    right = create_conflict(own, intr_id="R", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ, side=-1)
    assert (left.lat, left.lon) != (right.lat, right.lon)
    assert _cpa_and_tlos(own, left, _RPZ)[0] == pytest.approx(_cpa_and_tlos(own, right, _RPZ)[0])


def test_zero_relative_velocity_raises() -> None:
    own = _own()
    with pytest.raises(ValueError, match="zero relative velocity"):
        create_conflict(own, intr_id="INT", dpsi=0.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
