"""The estimator interface — advance / level / is_terminal — the IPS runs on (ADR 0004, Step 5).

``run_fleet``'s own tests (``test_fleet``) prove the *driver* is correct. These lock the three
properties IPS depends on that the driver tests don't exercise:

1. driving the interface by hand reproduces ``run_fleet`` — the environment is usable standalone;
2. ``advance`` is **pure** — it never mutates its input, and with no RNG two advances from one
   state are identical, which is exactly what makes an immutable ``FleetState`` safe to clone by
   sharing the reference (no deep copy, the KI-1 fix at scale);
3. ``level`` is the running minimum separation the outcome accumulates — what IPS splits on.
"""

from __future__ import annotations

from opencdarr import geo
from opencdarr.autopilot import CruiseAutopilot
from opencdarr.cd import StateBased
from opencdarr.cns.broadcast import BroadcastSchedule
from opencdarr.cns.stack import CNS
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.dynamics import Multirotor
from opencdarr.fleet import Agent, FleetEnv, FleetStreams, level, run_fleet
from opencdarr.performance import M600
from opencdarr.separation import SeparationManager
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND

_RPZ, _LOOKAHEAD, _DT = 50.0, 120.0, 0.5


def _ring(n: int, radius: float = 1500.0, speed: float = 10.0) -> list[Agent]:
    """n aircraft on a circle, each crossing to the opposite side — a superconflict, no RNG."""
    agents = []
    for k in range(n):
        bearing = 360.0 * k / n
        lat, lon = geo.forward(52.0, 4.0, bearing, radius)
        trk = (bearing + 180.0) % 360.0
        agents.append(Agent(AircraftState(id=f"A{k}", lat=lat, lon=lon, trk=trk, gs=speed), M600))
    return agents


def _env(agents: list[Agent]) -> FleetEnv:
    """A FleetEnv built by hand — how IPS will assemble the fixed rules once, per experiment."""
    n = len(agents)
    return FleetEnv(
        dyns=tuple(Multirotor() for _ in range(n)),
        perfs=tuple(a.perf for a in agents),
        adapters=tuple(None for _ in range(n)),  # multirotor flies velocity directly
        aps=tuple(CruiseAutopilot(a.state.trk, a.state.gs) for a in agents),
        separation=SeparationManager(),
        detector=StateBased(),
        resolver=MVP(margin=1.1),
        recovery=PastCPA(bouncing_guard=True),
        cns=CNS(),  # perfect information: exact self-fixes, instant delivery
        schedule=BroadcastSchedule(),
        wind=NO_WIND,
        rpz=_RPZ,
        t_lookahead=_LOOKAHEAD,
        dt=_DT,
        t_max=600.0,
        done_timeout=10.0,
    )


def test_interface_drives_to_the_run_fleet_outcome() -> None:
    """Stepping advance to is_terminal reproduces run_fleet — the IPS interface is faithful."""
    agents = _ring(4)
    env = _env(agents)
    state = env.initial_state(agents)
    while not env.is_terminal(state):
        state = env.advance(state, FleetStreams())
    ref = run_fleet(agents, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=_DT, detector=StateBased(),
                    resolver=MVP(margin=1.1), recovery=PastCPA(bouncing_guard=True))
    assert (state.conflict, state.los, state.min_sep) == (ref.conflict, ref.los, ref.min_sep)


def test_advance_is_pure_and_deterministic() -> None:
    """advance never mutates its input, and (no RNG) two advances from one state are identical —
    so an immutable FleetState is safe to clone by sharing the reference, not deep-copying."""
    agents = _ring(4)
    env = _env(agents)
    snapshot = env.initial_state(agents)
    for _ in range(20):  # drive into the conflict so the state is non-trivial
        snapshot = env.advance(snapshot, FleetStreams())

    before = (snapshot.states, snapshot.mems, snapshot.cmds, snapshot.t, snapshot.min_sep)
    a = env.advance(snapshot, FleetStreams())
    b = env.advance(snapshot, FleetStreams())

    # the shared state is untouched by advancing from it
    assert (snapshot.states, snapshot.mems, snapshot.cmds, snapshot.t, snapshot.min_sep) == before
    # and the two futures are bit-identical -> "clone" is a shared reference, nothing more
    assert (a.states, a.mems, a.cmds, a.next_bc, a.t, a.min_sep) == (
        b.states, b.mems, b.cmds, b.next_bc, b.t, b.min_sep
    )


def test_level_is_the_running_minimum_the_outcome_accumulates() -> None:
    """level(state) is the fleet's current minimum pairwise separation, measured at each pre-step
    instant; the final outcome's min_sep is the minimum over those — the quantity IPS splits on."""
    agents = _ring(4)
    env = _env(agents)
    state = env.initial_state(agents)
    assert env.is_terminal(state) is False
    assert level(state) > _RPZ  # a 1500 m ring starts well separated

    seen: list[float] = []
    while not env.is_terminal(state):
        seen.append(level(state))  # the pre-step instant the accumulator measures on
        state = env.advance(state, FleetStreams())
    assert state.min_sep == min(seen)
