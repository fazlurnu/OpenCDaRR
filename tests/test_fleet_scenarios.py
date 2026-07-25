"""The Phase-6 fleet scenarios (6d) — builders in ``scenario.py``, run through ``run_fleet``.

Each scenario, unresolved, collides; with cooperative MVP three of the four clear (min-sep ≥ rpz).
The **converging ring** (all aircraft to the *same* centre point) is the exception: the goal itself
is incompatible with separation (eight aircraft cannot occupy one point), so the DAA only mitigates
it — a genuine, documented limit, not a bug.
"""

from __future__ import annotations

from opencdarr import scenario as sc
from opencdarr.autopilot import WaypointAutopilot
from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, FleetOutcome, run_fleet
from opencdarr.mission import Mission
from opencdarr.performance import M600

_RPZ = 50.0
_LOOKAHEAD = 30.0


def _run(fleet: sc.FleetScenario, resolver: ConflictResolver | None) -> FleetOutcome:
    agents = [
        Agent(state, M600, autopilot=WaypointAutopilot(
            Mission(goto=target), cruise_airspeed=10.0, capture_radius=60.0))
        for state, target in fleet
    ]
    return run_fleet(
        agents, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.5, detector=StateBased(),
        resolver=resolver, t_max=400.0,
        recovery=PastCPA(bouncing_guard=True) if resolver else None,
    )


def test_swap_pair_resolves() -> None:
    """Two aircraft swapping positions (head-on) clear where, unresolved, they nearly collide."""
    fleet = sc.swap_pair()
    assert _run(fleet, None).min_sep < _RPZ  # a genuine conflict
    out = _run(fleet, MVP(margin=1.1))
    assert out.conflict is True and out.los is False and out.min_sep >= _RPZ


def test_swap_ring_resolves() -> None:
    """8 aircraft each crossing to the diametrically-opposite start all clear (scenario 2)."""
    assert _run(sc.swap_ring(8), None).los is True
    out = _run(sc.swap_ring(8), MVP(margin=1.1))
    assert out.los is False and out.min_sep >= _RPZ


def test_near_parallel_resolves() -> None:
    """A 5° near-parallel crossing (slow-closing) clears (scenario 4)."""
    assert _run(sc.near_parallel(), None).los is True
    out = _run(sc.near_parallel(), MVP(margin=1.1))
    assert out.los is False and out.min_sep >= _RPZ


def test_converging_ring_is_mitigated_not_resolved() -> None:
    """All 8 to the same centre: the DAA hugely improves separation but cannot fully clear —
    the goal (one shared point) is incompatible with rpz (scenario 3)."""
    unresolved = _run(sc.converging_ring(8), None)
    resolved = _run(sc.converging_ring(8), MVP(margin=1.1))
    assert unresolved.min_sep < 5.0  # unresolved: they pile onto the centre
    assert resolved.min_sep > unresolved.min_sep + 30.0  # DAA holds them far apart...
    assert resolved.min_sep < _RPZ  # ... but cannot reach rpz — everyone wants one point


def test_builders_shape() -> None:
    """The builders return the expected fleet sizes and aim each aircraft at its target."""
    assert len(sc.swap_pair()) == 2
    assert len(sc.swap_ring(8)) == 8
    assert len(sc.converging_ring(6)) == 6
    assert len(sc.near_parallel()) == 2
    # every aircraft's track points (roughly) at its goto target
    for state, target in sc.swap_ring(8):
        from opencdarr import geo
        brg, _ = geo.qdrdist(state.lat, state.lon, target[0], target[1])
        assert abs(((state.trk - brg + 180.0) % 360.0) - 180.0) < 1e-6
