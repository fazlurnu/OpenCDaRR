"""Traced fleet runs for handbook figures.

:func:`~opencdarr.fleet.run_fleet` returns only the scalar :class:`~opencdarr.fleet.FleetOutcome`,
but a figure needs the per-tick ground tracks and separation history. :func:`run_fleet_traced`
mirrors the fleet loop faithfully while recording each step, so the picture it draws is the same
run ``run_fleet`` scores — any handbook figure that plots a fleet encounter can reuse it. Running
it (``python scripts/handbook/tools/fleet_trace.py``) asserts the min-sep it records equals
``run_fleet``'s, on both a clean and a noisy run, so the two cannot drift apart unnoticed.

    from scripts.handbook.tools.fleet_trace import run_fleet_traced, enu
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

from opencdarr import geo
from opencdarr.autopilot import CruiseAutopilot, GuidanceMemory
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import NavigationModel
from opencdarr.cns.stack import CNS, CnsStreams
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.fleet import Agent, _all_clear, _pairwise_min_sep
from opencdarr.kinematics import Kinematics, Multirotor
from opencdarr.loop import _setpoint_adapter
from opencdarr.separation import INACTIVE, FleetMemory, SeparationManager
from opencdarr.state import AircraftState, DesiredVelocity

_DEFAULT_KINEMATICS = Multirotor()

LatLon = tuple[float, float]
Point = tuple[float, float]


def enu(origin: LatLon, lat: float, lon: float) -> Point:
    """(lat, lon) as (east, north) metres from ``origin`` — the frame the tracks are plotted in."""
    qdr, dist = geo.qdrdist(origin[0], origin[1], lat, lon)
    r = math.radians(qdr)
    return dist * math.sin(r), dist * math.cos(r)


@dataclass
class FleetTrace:
    """One fleet run captured tick by tick, in the same frame ``run_fleet`` measures on."""

    t: list[float] = field(default_factory=list)
    tracks: list[list[Point]] = field(default_factory=list)  # per aircraft, ENU from the origin
    min_sep: list[float] = field(default_factory=list)  # min pairwise separation each tick [m]
    resolving: list[bool] = field(default_factory=list)  # was any aircraft avoiding this tick?

    @property
    def worst_sep(self) -> float:
        """The run's minimum separation — equal to ``run_fleet(...).min_sep``."""
        return min(self.min_sep)

    def first_resolving(self) -> float | None:
        """The time the first avoidance began (when the conflict was acted on), or ``None``."""
        return next((t for t, r in zip(self.t, self.resolving, strict=True) if r), None)


def run_fleet_traced(
    agents: list[Agent],
    *,
    rpz: float,
    t_lookahead: float,
    dt: float,
    detector: ConflictDetector,
    resolver: ConflictResolver | None = None,
    recovery: RecoveryCriterion | None = None,
    navigation: NavigationModel | None = None,
    rng: np.random.Generator | None = None,
    broadcast_interval: float = 1.0,
    t_max: float = 600.0,
    done_timeout: float = 10.0,
    origin: LatLon | None = None,
) -> FleetTrace:
    """Run a fleet to termination like :func:`~opencdarr.fleet.run_fleet`, recording every tick.

    Runs the fleet loop's own datalink stack (:class:`~opencdarr.cns.stack.CNS`) and its own
    termination test, so the only thing this adds is the recording — for the aligned,
    perfect-delivery case (no per-link comm loss or broadcast phase offsets).
    ``tracks`` are ENU metres from ``origin`` (the first aircraft's start if omitted). The recorded
    ``worst_sep`` equals the ``run_fleet`` outcome for the same inputs (asserted in ``__main__``).
    """
    n = len(agents)
    origin = origin or (agents[0].state.lat, agents[0].state.lon)
    kinematics: list[Kinematics] = [a.kinematics or _DEFAULT_KINEMATICS for a in agents]
    perfs = [a.perf for a in agents]
    adapters = [_setpoint_adapter(kinematics[i], perfs[i]) for i in range(n)]
    aps = [a.autopilot or CruiseAutopilot(a.state.trk, a.state.gs) for a in agents]
    states = [replace(a.state, desired=DesiredVelocity.from_track_speed(a.state.trk, a.state.gs))
              for a in agents]
    gms = [GuidanceMemory() for _ in range(n)]
    mems: list[FleetMemory] = [INACTIVE for _ in range(n)]
    sep = SeparationManager()
    cns = CNS(navigation=navigation)  # perfect delivery, intent private
    cns_streams = CnsStreams(nav=rng)
    cns_state = cns.initial_state(n)
    cmds = [aps[i].step(states[i], gms[i], perfs[i])[0] for i in range(n)]

    tr = FleetTrace(tracks=[[] for _ in range(n)])
    t, next_bcast, done_timer = 0.0, 0.0, 0.0
    while t < t_max:
        tr.t.append(t)
        for i in range(n):
            tr.tracks[i].append(enu(origin, states[i].lat, states[i].lon))
        tr.min_sep.append(_pairwise_min_sep(states))
        tr.resolving.append(any(m.resolving for m in mems))

        if t + 1e-9 >= next_bcast:
            cns_state, perception = cns.sense(states, range(n), t, cns_state, cns_streams)
            for i in range(n):
                see = perception[i]
                nom, gms[i] = aps[i].step(see.own, gms[i], perfs[i])
                cmds[i], mems[i] = sep.step(see.own, see.traffic, nom, mems[i], rpz, t_lookahead,
                                            detector, resolver, recovery, adapters[i])
            next_bcast += broadcast_interval

        states = [kinematics[i].step(states[i], cmds[i], perfs[i], dt) for i in range(n)]
        t += dt
        done_timer = done_timer + dt if _all_clear(states, mems, rpz) else 0.0
        if done_timer >= done_timeout:
            break
    return tr


def _self_test() -> None:
    """Assert the traced min-sep equals ``run_fleet``'s, clean and noisy — the anti-drift guard."""
    from opencdarr.cd import StateBased
    from opencdarr.cns.navigation import GnssNavigation
    from opencdarr.cr import MVP
    from opencdarr.crr import PastCPA
    from opencdarr.fleet import run_fleet
    from opencdarr.performance import M600
    from opencdarr.rng import generator, root_seed_sequence
    from opencdarr.scenario import create_conflict

    def fleet(noisy: bool) -> list[Agent]:
        ci = (15.0, 1.5) if noisy else (0.0, 0.0)
        own = AircraftState(id="A", lat=52.0, lon=4.0, trk=0.0, gs=15.0,
                            pos_ci95=ci[0], vel_ci95=ci[1])
        intr = create_conflict(own, intr_id="B", dpsi=90.0, dcpa=0.0, tlos=30.0, rpz=50.0)
        return [Agent(own, M600), Agent(intr, M600)]

    cdr = dict(rpz=50.0, t_lookahead=20.0, dt=0.5, detector=StateBased(),
               resolver=MVP(margin=1.1), recovery=PastCPA(bouncing_guard=True))
    for noisy in (False, True):
        nav = GnssNavigation() if noisy else None
        rng = generator(root_seed_sequence(7)) if noisy else None
        traced = run_fleet_traced(fleet(noisy), navigation=nav, rng=rng, **cdr).worst_sep
        scored = run_fleet(fleet(noisy),
                           navigation=GnssNavigation() if noisy else None,
                           rng=generator(root_seed_sequence(7)) if noisy else None,
                           **cdr).min_sep
        assert abs(traced - scored) < 1e-9, f"drift ({'noisy' if noisy else 'clean'}): " \
                                            f"traced {traced} vs run_fleet {scored}"
        print(f"{'noisy' if noisy else 'clean'}: traced == run_fleet == {scored:.3f} m")
    print("ok — run_fleet_traced matches run_fleet")


if __name__ == "__main__":
    _self_test()
