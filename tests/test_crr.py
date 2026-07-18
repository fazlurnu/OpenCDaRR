"""Functional tests for Past-CPA recovery."""

from __future__ import annotations

from opencdarr import geo
from opencdarr.crr import PastCPA
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0


def _own() -> AircraftState:
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)


def _ahead(dist_m: float, trk: float, gs: float) -> AircraftState:
    """An intruder placed dist_m straight ahead (due north) of the ownship."""
    lat, lon = geo.forward(52.0, 4.0, 0.0, dist_m)
    return AircraftState(id="INT", lat=lat, lon=lon, trk=trk, gs=gs)


def test_pre_cpa_does_not_resume() -> None:
    """A still-approaching conflict (t_cpa > 0) must keep resolving."""
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    assert PastCPA().should_resume(own, intr, _RPZ) is False


def test_past_cpa_and_separated_resumes() -> None:
    """Diverging and outside the zone -> resume."""
    own = _own()
    intr = _ahead(500.0, trk=0.0, gs=15.0)  # ahead, same heading, pulling away
    assert PastCPA().should_resume(own, intr, _RPZ) is True


def test_past_cpa_but_in_los_does_not_resume() -> None:
    """Diverging but still inside the zone -> keep resolving until LoS clears."""
    own = _own()
    intr = _ahead(30.0, trk=0.0, gs=15.0)  # only 30 m ahead (< rpz), pulling away
    assert PastCPA().should_resume(own, intr, _RPZ) is False


def test_bouncing_guard_holds_resolution() -> None:
    """Near-parallel and just outside the zone: guard off resumes, guard on holds."""
    own = _own()
    intr = _ahead(51.0, trk=10.0, gs=15.0)  # ~51 m (in [rpz, 1.05*rpz)), near-parallel, diverging
    assert PastCPA(bouncing_guard=False).should_resume(own, intr, _RPZ) is True
    assert PastCPA(bouncing_guard=True).should_resume(own, intr, _RPZ) is False
