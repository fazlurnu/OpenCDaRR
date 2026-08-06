"""K and A on the fleet outcome — the losing-pairs record the per-aircraft metric is built on.

Ground truth here is geometric, not empirical: in a symmetric superconflict with no resolver every
aircraft converges, so *every* pair loses separation — ``n_los_pairs`` (K) is ``C(n, 2)`` and
``n_los_aircraft`` (A) is ``n``. The reason the record exists is the resolver case: a
``converging_ring`` the DAA can only *mitigate* (n aircraft cannot occupy one point) stays
``los=True`` while K falls below the maximum. That drop is exactly the signal a single ``los`` bool
cannot carry, and the whole argument for normalising per aircraft (ADR 0022).
"""

from __future__ import annotations

from opencdarr import scenario as sc
from opencdarr.autopilot import WaypointAutopilot
from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, FleetOutcome, run_fleet
from opencdarr.mission import Mission
from opencdarr.performance import M600

_RPZ, _LOOKAHEAD, _DT, _SPEED = 50.0, 30.0, 0.5, 10.0


def _run(fleet: sc.FleetScenario, resolver: MVP | None) -> FleetOutcome:
    """Fly ``fleet`` to termination through ``run_fleet`` — noiseless, deterministic."""
    agents = [
        Agent(
            state, M600,
            autopilot=WaypointAutopilot(Mission(goto=target), cruise_airspeed=_SPEED,
                                        capture_radius=60.0),
        )
        for state, target in fleet
    ]
    return run_fleet(
        agents, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT,
        detector=StateBased(), resolver=resolver,
        recovery=PastCPA(bouncing_guard=True) if resolver is not None else None,
        t_max=400.0, done_timeout=10.0,
    )


def test_no_resolver_superconflict_every_pair_loses() -> None:
    """Geometry alone: unresolved, every aircraft crosses the centre, so K = C(n, 2) and A = n."""
    swap = _run(sc.swap_ring(4), None)
    assert (swap.n_los_pairs, swap.n_los_aircraft) == (6, 4)
    assert swap.los and swap.min_sep < _RPZ

    conv = _run(sc.converging_ring(6), None)
    assert (conv.n_los_pairs, conv.n_los_aircraft) == (15, 6)
    assert conv.los and conv.min_sep < _RPZ


def test_resolver_drops_losing_pairs_while_los_cannot_tell() -> None:
    """The reason for the metric, shown in two geometries.

    ``swap_ring(4)``: the resolver clears it — ``los`` flips off and no pair is left losing.
    ``converging_ring(6)``: it cannot (n aircraft cannot occupy one point), so ``los`` stays
    ``True`` with or without the resolver — yet K falls 15 → 12. ``los`` is blind to that
    improvement; K (and hence the per-aircraft rate) is not.

    The MVP counts are a golden anchor — noiseless, exact, reproducible. Update them only with a
    deliberate, recorded modelling change.
    """
    swap = _run(sc.swap_ring(4), MVP(1.1))
    assert (swap.n_los_pairs, swap.n_los_aircraft, swap.los) == (0, 0, False)
    assert swap.min_sep >= _RPZ                         # cleared: never within rpz

    conv_none = _run(sc.converging_ring(6), None)
    conv_mvp = _run(sc.converging_ring(6), MVP(1.1))
    assert conv_none.los and conv_mvp.los               # los cannot separate the two
    assert (conv_none.n_los_pairs, conv_mvp.n_los_pairs) == (15, 12)   # ... but K can, 15 → 12
    assert conv_mvp.n_los_aircraft == 6
    assert conv_mvp.min_sep > conv_none.min_sep         # and separation improves
