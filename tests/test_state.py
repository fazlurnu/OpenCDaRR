"""Tests for the validated aircraft factory (`create_aircraft`)."""

from __future__ import annotations

import pytest

from opencdarr.performance import M600
from opencdarr.state import AircraftState, create_aircraft


def test_create_aircraft_returns_valid_state() -> None:
    """A within-envelope spec yields the expected AircraftState."""
    ac = create_aircraft(M600, id="D0", lat=52.0, lon=4.0, trk=90.0, gs=10.0)
    assert ac == AircraftState(id="D0", lat=52.0, lon=4.0, trk=90.0, gs=10.0)
    assert ac.turn_rate == 0.0


def test_create_aircraft_rejects_overspeed() -> None:
    """An initial speed above v_max is a spec error and raises (not clamped)."""
    with pytest.raises(ValueError, match="outside the envelope"):
        create_aircraft(M600, id="D0", lat=52.0, lon=4.0, trk=90.0, gs=30.0)


def test_create_aircraft_rejects_underspeed() -> None:
    """An initial speed below v_min raises."""
    with pytest.raises(ValueError, match="outside the envelope"):
        create_aircraft(M600, id="D0", lat=52.0, lon=4.0, trk=90.0, gs=-30.0)


def test_create_aircraft_accepts_envelope_bounds() -> None:
    """Exactly v_max / v_min are allowed (inclusive envelope)."""
    assert create_aircraft(M600, id="D0", lat=52.0, lon=4.0, trk=0.0, gs=M600.v_max).gs == 18.0
    assert create_aircraft(M600, id="D0", lat=52.0, lon=4.0, trk=0.0, gs=M600.v_min).gs == -18.0


def test_create_aircraft_rejects_turn_rate_over_limit() -> None:
    """An initial turn rate beyond max_tr raises."""
    with pytest.raises(ValueError, match="max turn rate"):
        create_aircraft(M600, id="D0", lat=52.0, lon=4.0, trk=0.0, gs=10.0, turn_rate=20.0)
