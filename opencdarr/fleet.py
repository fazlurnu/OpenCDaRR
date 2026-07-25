"""The N-aircraft encounter runner — the fleet environment (Phase 6b).

:func:`run_fleet` is the multi-aircraft generalisation of
:func:`~opencdarr.loop.run_encounter`: a **list of aircraft**, each with its own
:class:`~opencdarr.autopilot.Autopilot` / :class:`~opencdarr.dynamics.Dynamics` /
:class:`~opencdarr.performance.Performance`, all advancing simultaneously. Every aircraft runs its
own detect → resolve → recover against **all the others it perceives** (the cooperative fleet — no
central controller), so in a conflict *everyone* manoeuvres, not just one side. The directed,
pairwise-primitive design (ADR 0004) makes this a change of *environment*, not of the CDR core:
detection iterates the conflict graph, resolution composes the set (MVP sums, VO unions, Phase 6a),
and recovery waits until an aircraft is clear of **all** its conflicts.

**Perfect perception this pass** (Phase-6 plan decision 3): each aircraft sees the other broadcasts
directly (instant, perfect delivery), with optional GNSS self-noise (``navigation`` + ``rng``). The
lossy communication / surveillance model over the n(n−1) directed links is deferred to 6g.

**Reduces to the pairwise runner** at n = 2: ``run_fleet`` with two agents reproduces
:func:`~opencdarr.loop.run_encounter` (no-communication path) bit-for-bit — the free multi-aircraft
regression (ADR 0004). Pure given its inputs; no globals. The fleet of states migrates into the IPS
particle when Phase 8 lands.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from opencdarr import geo
from opencdarr.autopilot import Autopilot, CruiseAutopilot, GuidanceMemory
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import NavigationModel
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.dynamics import Dynamics, MotionCommand
from opencdarr.kinematics import relative_enu
from opencdarr.loop import _DEFAULT_DYNAMICS, _setpoint_adapter
from opencdarr.performance import Performance
from opencdarr.separation import INACTIVE, FleetMemory, SeparationManager
from opencdarr.state import AircraftState, DesiredVelocity
from opencdarr.wind import NO_WIND, WindField


@dataclass(frozen=True)
class Agent:
    """One aircraft's bundle in the fleet: its state + how it navigates, flies, and is limited.

    ADR 0011 §7 deferred a per-aircraft grouping "until a real grouping need appears"; N parallel
    lists is that need, so the bundle lands here. ``dynamics`` defaults to the shared
    :class:`~opencdarr.dynamics.Multirotor`; ``autopilot`` defaults to a
    :class:`~opencdarr.autopilot.CruiseAutopilot` holding the state's initial cruise.
    """

    state: AircraftState
    perf: Performance
    dynamics: Dynamics | None = None
    autopilot: Autopilot | None = None


@dataclass(frozen=True)
class FleetOutcome:
    """What one fleet encounter produced (measured on the true states, every step)."""

    conflict: bool  # was any directed pair predicted in conflict at any step?
    los: bool  # was any pair ever in loss of separation?
    min_sep: float  # minimum pairwise separation reached across all pairs [m]


def _pairwise_min_sep(states: list[AircraftState]) -> float:
    """Smallest separation over all unordered pairs [m]."""
    smallest = float("inf")
    for i in range(len(states)):
        for j in range(i + 1, len(states)):
            _, dist = geo.qdrdist(states[i].lat, states[i].lon, states[j].lat, states[j].lon)
            smallest = min(smallest, dist)
    return smallest


def run_fleet(
    agents: list[Agent],
    *,
    rpz: float,
    t_lookahead: float,
    dt: float,
    detector: ConflictDetector,
    resolver: ConflictResolver | None = None,
    recovery: RecoveryCriterion | None = None,
    wind: WindField = NO_WIND,
    navigation: NavigationModel | None = None,
    rng: np.random.Generator | None = None,
    t_max: float = 600.0,
    done_timeout: float = 10.0,
    broadcast_interval: float = 1.0,
    share_intent: bool = False,
) -> FleetOutcome:
    """Advance the fleet to termination and report its outcome (see the module docstring).

    Each aircraft decides on the broadcast cadence from its (optionally noisy) self-fix against its
    perceived traffic — every *other* aircraft's broadcast (perfect delivery here). The command
    is held while the dynamics integrate at ``dt``; all aircraft advance together. The outcome
    (conflict / LoS / min-sep) is measured on the **true** states every step. Terminates once every
    pair is diverging and separated and no aircraft is resolving for ``done_timeout``, or at
    ``t_max``.
    """
    n = len(agents)
    dyns: list[Dynamics] = [a.dynamics or _DEFAULT_DYNAMICS for a in agents]
    perfs: list[Performance] = [a.perf for a in agents]
    adapters = [_setpoint_adapter(dyns[i], perfs[i]) for i in range(n)]
    aps: list[Autopilot] = [
        a.autopilot or CruiseAutopilot(a.state.trk, a.state.gs) for a in agents
    ]
    # intent (nominal velocity) on each true state, private unless share_intent (as run_encounter)
    states = [
        replace(a.state, desired=DesiredVelocity.from_track_speed(a.state.trk, a.state.gs))
        for a in agents
    ]
    gms = [GuidanceMemory() for _ in range(n)]
    mems: list[FleetMemory] = [INACTIVE for _ in range(n)]
    separation = SeparationManager()  # stateless; memory rides in mems (ADR 0011 §5 / 0004)
    cmds: list[MotionCommand] = []
    for i in range(n):
        cmd, gms[i] = aps[i].step(states[i], gms[i], perfs[i])
        cmds.append(cmd)

    conflict = los = False
    min_sep = float("inf")
    done_timer = 0.0
    t = 0.0
    next_broadcast = 0.0
    eps = 1e-9  # float guard so a tick lands on t = k*broadcast_interval reached by dt steps

    while t < t_max:
        min_sep = min(min_sep, _pairwise_min_sep(states))
        if min_sep < rpz:
            los = True
        if any(
            i != j and detector.detect(states[i], states[j], rpz, t_lookahead)
            for i in range(n)
            for j in range(n)
        ):
            conflict = True

        if t + eps >= next_broadcast:
            # each aircraft's fresh (noisy) self-fix, in agent order (matches run_encounter draws)
            if navigation is not None and rng is not None:
                fixes = [navigation.measure(states[i], t, rng).state for i in range(n)]
            else:
                fixes = list(states)
            # an aircraft knows its own intent; what it transmits strips intent unless shared
            selfs = [replace(fixes[i], desired=states[i].desired) for i in range(n)]
            txs = [
                replace(fixes[i], desired=states[i].desired if share_intent else None)
                for i in range(n)
            ]
            for i in range(n):
                nom, gms[i] = aps[i].step(selfs[i], gms[i], perfs[i])
                perceived = [txs[j] for j in range(n) if j != i]  # every other aircraft (perfect)
                cmds[i], mems[i] = separation.step(
                    selfs[i], perceived, nom, mems[i], rpz, t_lookahead,
                    detector, resolver, recovery, adapters[i],
                )
            next_broadcast += broadcast_interval

        # advance all aircraft from their pre-step states (explicitly simultaneous)
        states = [dyns[i].step(states[i], cmds[i], perfs[i], dt, wind) for i in range(n)]
        t += dt

        # done when every pair is past CPA and separated and no aircraft is resolving
        resolving = any(m.resolving for m in mems)
        all_clear = not resolving
        for i in range(n):
            for j in range(i + 1, n):
                rel = relative_enu(states[i], states[j])
                diverging = rel.rx * rel.vx + rel.ry * rel.vy > 0.0
                if not (diverging and rel.dist >= rpz):
                    all_clear = False
        done_timer = done_timer + dt if all_clear else 0.0
        if done_timer >= done_timeout:
            break

    return FleetOutcome(conflict=conflict, los=los, min_sep=min_sep)
