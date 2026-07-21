"""Functional tests for FTR (Free-To-Revert) recovery."""

from __future__ import annotations

import dataclasses

import pytest

from opencdarr import geo
from opencdarr.crr import FTR
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState, DesiredVelocity

_RPZ = 50.0


def _own(desired: DesiredVelocity | None = None) -> AircraftState:
    d = desired if desired is not None else DesiredVelocity.from_track_speed(0.0, 10.0)
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0, desired=d)


def _ahead(dist_m: float, trk: float, gs: float, desired: DesiredVelocity | None = None) -> AircraftState:
    """An intruder placed dist_m straight ahead (due north) of the ownship."""
    lat, lon = geo.forward(52.0, 4.0, 0.0, dist_m)
    return AircraftState(id="INT", lat=lat, lon=lon, trk=trk, gs=gs, desired=desired)


def test_no_desired_velocity_raises() -> None:
    own = dataclasses.replace(_own(), desired=None)
    intr = _ahead(500.0, trk=0.0, gs=15.0)
    with pytest.raises(ValueError):
        FTR().should_resume(own, intr, _RPZ)


def test_reverting_would_still_clear_resumes() -> None:
    """Own's desired velocity == its current one, and it already clears -> resume."""
    own = _own()
    intr = _ahead(500.0, trk=0.0, gs=15.0)  # ahead, pulling away
    assert FTR().should_resume(own, intr, _RPZ) is True


def test_reverting_would_reconverge_does_not_resume() -> None:
    """Own is currently deflected (clear), but reverting to its *desired* velocity re-aims at the intruder."""
    desired = DesiredVelocity.from_track_speed(0.0, 10.0)
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=30.0, gs=10.0, desired=desired)
    # intruder sits close ahead, holding a track that reverting to own's nominal would re-converge on
    intr = _ahead(60.0, trk=180.0, gs=10.0)  # head-on to own's *desired* (nominal) track
    assert FTR().should_resume(own, intr, _RPZ) is False


def test_intent_based_second_criterion_blocks_resume_when_shared() -> None:
    """Intruder's *current* velocity clears, but reverting to its shared *desired* velocity re-converges."""
    own = _own(DesiredVelocity.from_track_speed(0.0, 10.0))  # own heads north at 10 m/s
    # 1000 m north of own; current velocity (east, 5 m/s) diverges; desired (south, 5 m/s) doesn't
    intr = _ahead(1000.0, trk=90.0, gs=5.0, desired=DesiredVelocity.from_track_speed(180.0, 5.0))
    assert FTR().should_resume(own, intr, _RPZ) is False


def test_without_shared_intent_only_first_criterion_applies() -> None:
    """Same geometry, but the intruder's intent is not shared (desired=None) -> resumes."""
    own = _own(DesiredVelocity.from_track_speed(0.0, 10.0))
    intr = _ahead(1000.0, trk=90.0, gs=5.0, desired=None)
    assert FTR().should_resume(own, intr, _RPZ) is True
