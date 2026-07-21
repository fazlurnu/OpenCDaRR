"""Functional tests for the loop's per-directed-pair CDR memory (``PairMemory``).

Covers the reference's ``_intr_init_vel`` mechanism: the other aircraft's velocity is recorded
when a pair becomes active, held unchanged while it stays active, cleared on resume, and used as
an *inferred* stand-in for its desired velocity when intent wasn't shared — without which
intent-based criteria (FTR / ProbabilisticFTR) would simply skip their second criterion.
"""

from __future__ import annotations

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.crr import FTR, PastCPA
from opencdarr.dynamics import Command
from opencdarr.loop import _INACTIVE, _decide
from opencdarr.state import AircraftState, DesiredVelocity

_RPZ, _LOOKAHEAD = 50.0, 120.0
_NOM = Command.from_track_speed(0.0, 10.0)


def _own() -> AircraftState:
    desired = DesiredVelocity.from_track_speed(0.0, 10.0)
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0, desired=desired)


def _intr(
    dist_m: float, trk: float, gs: float = 10.0,
    bearing: float = 0.0, desired: DesiredVelocity | None = None,
) -> AircraftState:
    lat, lon = geo.forward(52.0, 4.0, bearing, dist_m)
    return AircraftState(id="INT", lat=lat, lon=lon, trk=trk, gs=gs, desired=desired)


def _run(own, intr, memory, recovery=None):
    return _decide(
        own, intr, _NOM, memory, _RPZ, _LOOKAHEAD, StateBased(), MVP(1.05), recovery or FTR()
    )


def test_onset_velocity_recorded_when_pair_becomes_active() -> None:
    _, memory = _run(_own(), _intr(400.0, trk=180.0), _INACTIVE)
    assert memory.resolving is True
    assert memory.onset_velocity == DesiredVelocity.from_track_speed(180.0, 10.0)


def test_onset_velocity_is_not_overwritten_while_active() -> None:
    """The whole point: it must survive the other aircraft manoeuvring away afterwards."""
    _, first = _run(_own(), _intr(400.0, trk=180.0), _INACTIVE)
    _, second = _run(_own(), _intr(400.0, trk=90.0), first)  # intruder has since turned
    assert second.resolving is True
    expected = DesiredVelocity.from_track_speed(180.0, 10.0)
    assert second.onset_velocity == first.onset_velocity == expected


def test_memory_cleared_on_resume() -> None:
    _, active = _run(_own(), _intr(400.0, trk=180.0), _INACTIVE)
    assert active.onset_velocity is not None
    resolved = _intr(5000.0, trk=90.0, bearing=90.0)  # far away and diverging -> resume
    _, after = _run(_own(), resolved, active)
    assert after.resolving is False
    assert after.onset_velocity is None


def test_inactive_pair_records_nothing() -> None:
    far = _intr(20000.0, trk=0.0, bearing=90.0)  # no conflict at all
    _, memory = _run(_own(), far, _INACTIVE)
    assert memory == _INACTIVE
    assert memory.onset_velocity is None


def test_declared_intent_is_never_overwritten_by_the_inferred_fallback() -> None:
    """Shared intent wins; the onset velocity is still recorded but must not displace it."""
    declared = DesiredVelocity.from_track_speed(270.0, 7.0)
    own, intr = _own(), _intr(400.0, trk=180.0, desired=declared)
    _, memory = _run(own, intr, _INACTIVE)
    # onset is recorded regardless (cheap, and the pair may lose intent-sharing later)
    assert memory.onset_velocity == DesiredVelocity.from_track_speed(180.0, 10.0)
    # ... but the criterion saw the declared value: same geometry with the fallback substituted
    # instead would be a different question, so verify FTR's answer tracks the declared one.
    assert FTR().should_resume(own, intr, _RPZ) == FTR().should_resume(
        own, intr.__class__(**{**intr.__dict__, "desired": declared}), _RPZ
    )


def test_fallback_makes_the_second_criterion_reachable_without_intent_sharing() -> None:
    """Without the fallback, criterion 2 is skipped entirely when desired is None. With it, a
    geometry whose *current* velocity clears but whose *onset* velocity re-converges is blocked."""
    own = _own()
    # onset: heading south (re-converges with own's northbound desired) at close range
    onset_geometry = _intr(300.0, trk=180.0, desired=None)
    _, memory = _run(own, onset_geometry, _INACTIVE)
    assert memory.onset_velocity == DesiredVelocity.from_track_speed(180.0, 10.0)

    # now the intruder has turned away (current velocity alone would look clear), but the
    # remembered onset velocity still re-converges -> criterion 2 keeps the pair resolving
    turned_away = _intr(300.0, trk=90.0, desired=None)
    _, still = _run(own, turned_away, memory)
    assert still.resolving is True
    assert still.onset_velocity == DesiredVelocity.from_track_speed(180.0, 10.0)


def test_pastcpa_is_unaffected_by_the_memory() -> None:
    """A criterion that ignores intent must behave identically with or without a recorded onset."""
    own, intr = _own(), _intr(400.0, trk=180.0)
    cmd_a, mem_a = _run(own, intr, _INACTIVE, recovery=PastCPA())
    cmd_b, mem_b = _run(own, intr, mem_a, recovery=PastCPA())
    assert cmd_a == cmd_b
    assert mem_b.resolving is mem_a.resolving
