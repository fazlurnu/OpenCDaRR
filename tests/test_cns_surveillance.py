"""Functional tests for surveillance (Phase 3b): what a receiver believes, given what
communication delivered."""

from __future__ import annotations

import dataclasses

from opencdarr.cns import CommState, LastKnown, Message, age
from opencdarr.state import AircraftState

_STATE = AircraftState(id="INT", lat=52.0, lon=4.0, trk=90.0, gs=12.0)
_MSG = Message(source="INT", state=_STATE, t_meas=3.0)


def _held(msg: Message = _MSG) -> CommState:
    return CommState(held={("OWN", "INT"): msg})


def test_never_received_is_none() -> None:
    assert LastKnown().perceived(CommState(), "OWN", "INT", t_now=10.0) is None
    assert age(CommState(), "OWN", "INT", t_now=10.0) is None


def test_perceived_is_exactly_the_held_state() -> None:
    got = LastKnown().perceived(_held(), "OWN", "INT", t_now=3.0)
    assert got == _STATE


def test_hold_as_is_does_not_move_with_t_now() -> None:
    """The defining property: no dead-reckoning. Perceived state is identical at any t_now."""
    state = _held()
    at_t3 = LastKnown().perceived(state, "OWN", "INT", t_now=3.0)
    at_t50 = LastKnown().perceived(state, "OWN", "INT", t_now=50.0)
    assert at_t3 == at_t50 == _STATE  # unchanged despite 47s of elapsed time


def test_age_reports_staleness_without_changing_perceived() -> None:
    state = _held()
    assert age(state, "OWN", "INT", t_now=3.0) == 0.0
    assert age(state, "OWN", "INT", t_now=10.0) == 7.0
    # age moved; the perceived state (checked above) did not
    assert LastKnown().perceived(state, "OWN", "INT", t_now=10.0) == _STATE


def test_directed_lookup_key_order() -> None:
    """held is keyed (receiver, source); OWN's belief about INT is independent of the reverse."""
    state = CommState(held={("OWN", "INT"): _MSG})
    assert LastKnown().perceived(state, "OWN", "INT", 3.0) is not None
    assert LastKnown().perceived(state, "INT", "OWN", 3.0) is None  # not populated


def test_perceived_updates_only_when_held_changes() -> None:
    """Perceiving twice against the same CommState with different messages held gives different
    answers -> perceived truly reflects `held`, not some cached/stateful surveillance object."""
    older = _held(Message(source="INT", state=_STATE, t_meas=1.0))
    newer_state = dataclasses.replace(_STATE, gs=20.0)
    newer = _held(Message(source="INT", state=newer_state, t_meas=2.0))
    lk = LastKnown()  # the same stateless model instance, used against two CommStates
    assert lk.perceived(older, "OWN", "INT", 5.0).gs == 12.0
    assert lk.perceived(newer, "OWN", "INT", 5.0).gs == 20.0
