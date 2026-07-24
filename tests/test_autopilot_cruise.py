"""The Phase-4a behaviour-preserving property of :class:`CruiseAutopilot`.

The whole reason the layer split reproduces the pre-refactor IPR bit-for-bit is that the cruise
nominal is a **mission parameter frozen from the true initial state**, never re-derived from the
live (noisy) self-fix each tick. So the load-bearing test is: ``step`` returns the *same* command
regardless of the state (and performance) it is handed.
"""

from __future__ import annotations

from opencdarr.autopilot import CruiseAutopilot
from opencdarr.dynamics import MotionCommand
from opencdarr.performance import M600
from opencdarr.state import AircraftState


def _state(**over: float) -> AircraftState:
    base = dict(id="OWN", lat=52.0, lon=4.0, trk=90.0, gs=10.0)
    base.update(over)
    return AircraftState(**base)  # type: ignore[arg-type]


def test_cruise_command_matches_from_track_speed() -> None:
    """The nominal is exactly the old frozen ``Command.from_track_speed(heading, speed)``."""
    ap = CruiseAutopilot(heading=30.0, speed=12.0)
    assert ap.step(_state(), M600) == MotionCommand.from_track_speed(30.0, 12.0)


def test_cruise_is_independent_of_state() -> None:
    """step() ignores the (noisy) state — the frozen-nominal property the 4a regression needs."""
    ap = CruiseAutopilot(heading=0.0, speed=10.0)
    baseline = ap.step(_state(trk=0.0, gs=10.0), M600)
    # a wildly different (e.g. noise-perturbed) state must not move the command at all
    for perturbed in (_state(trk=45.0, gs=3.0), _state(lat=51.0, lon=5.0, trk=200.0, gs=17.0)):
        assert ap.step(perturbed, M600) == baseline


def test_cruise_is_constant_across_ticks() -> None:
    """Repeated calls return the identical command object — a truly frozen nominal."""
    ap = CruiseAutopilot(heading=270.0, speed=8.0)
    first = ap.step(_state(), M600)
    for _ in range(5):
        assert ap.step(_state(), M600) is first


def test_cruise_holds_no_evolving_state() -> None:
    """The autopilot accumulates nothing across steps (no hidden guidance state, ADR 0011 §5)."""
    ap = CruiseAutopilot(heading=90.0, speed=10.0)
    snapshot = dict(vars(ap))
    ap.step(_state(), M600)
    ap.step(_state(trk=123.0), M600)
    assert vars(ap) == snapshot  # nothing was written to the instance by stepping
