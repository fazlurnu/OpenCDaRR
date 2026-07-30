"""SeparationManager — the ported ``_decide`` (ADR 0011, Phase 4a).

Two properties the layer split rests on:

1. **byte-identical decisions** to the pre-Phase-4a ``loop._decide`` on shared inputs — the port
   is substantive-behaviour-preserving (the loop-level bit-for-bit IPR regression lives in
   ``test_loop.py``; here we pin the unit directly);
2. **no state on the manager object** — the per-pair memory is threaded in/out as ``PairMemory``,
   nothing is written to ``self`` (the load-bearing no-hidden-state invariant, ADR 0011 §5).
"""

from __future__ import annotations

from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.kinematics import MotionCommand
from opencdarr.loop import _decide  # the backward-compat shim, for the equivalence check
from opencdarr.scenario import create_conflict
from opencdarr.separation import INACTIVE, PairMemory, SeparationManager
from opencdarr.state import AircraftState

_RPZ = 50.0
_LOOKAHEAD = 120.0


def _pair() -> tuple[AircraftState, AircraftState]:
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=90.0, rpz=_RPZ)
    return own, intr


def _nominal(ac: AircraftState) -> MotionCommand:
    return MotionCommand.from_track_speed(ac.trk, ac.gs)


def _step(
    mgr: SeparationManager,
    ac: AircraftState,
    other: AircraftState | None,
    memory: PairMemory,
) -> tuple[MotionCommand, PairMemory]:
    return mgr.step(
        ac, [] if other is None else [other], _nominal(ac), memory,
        _RPZ, _LOOKAHEAD, StateBased(), MVP(margin=1.1), PastCPA(),
    )


def test_matches_old_decide_on_a_live_conflict() -> None:
    """A detected pair: manager.step == the loop._decide shim, command and memory both."""
    own, intr = _pair()
    cmd_m, mem_m = _step(SeparationManager(), own, intr, INACTIVE)
    cmd_d, mem_d = _decide(
        own, intr, _nominal(own), INACTIVE, _RPZ, _LOOKAHEAD, StateBased(), MVP(margin=1.1),
        PastCPA(),
    )
    assert cmd_m == cmd_d
    assert mem_m == mem_d
    assert mem_m.resolving is True  # the scenario really does engage resolution (the test bites)


def test_no_resolver_or_no_traffic_flies_nominal() -> None:
    """Resolution disabled or empty perceived traffic -> nominal + INACTIVE (fly on)."""
    own, intr = _pair()
    mgr = SeparationManager()
    nominal = _nominal(own)
    # no perceived traffic
    cmd, mem = mgr.step(
        own, [], nominal, INACTIVE, _RPZ, _LOOKAHEAD, StateBased(), MVP(margin=1.1), PastCPA()
    )
    assert cmd == nominal and mem == INACTIVE
    # no resolver
    cmd2, mem2 = mgr.step(
        own, [intr], nominal, INACTIVE, _RPZ, _LOOKAHEAD, StateBased(), None, PastCPA()
    )
    assert cmd2 == nominal and mem2 == INACTIVE


def test_manager_is_stateless() -> None:
    """Stepping writes nothing to the instance; a shared manager is reuse-safe (ADR 0011 §5)."""
    own, intr = _pair()
    mgr = SeparationManager()
    snapshot = dict(vars(mgr))
    r1 = _step(mgr, own, intr, INACTIVE)
    r2 = _step(mgr, own, intr, INACTIVE)
    assert vars(mgr) == snapshot  # no attribute was set on the manager by stepping
    assert r1 == r2  # pure: identical inputs -> identical outputs, no accumulated drift


def test_memory_is_threaded_not_hidden() -> None:
    """The active memory returned by one step drives the next — carried by value, not by self."""
    own, intr = _pair()
    mgr = SeparationManager()
    _, active = _step(mgr, own, intr, INACTIVE)
    assert isinstance(active, PairMemory) and active.resolving
    # a *fresh* manager fed that same memory must decide identically (memory is the only carrier)
    same, _ = _step(SeparationManager(), own, intr, active)
    also, _ = _step(mgr, own, intr, active)
    assert same == also
