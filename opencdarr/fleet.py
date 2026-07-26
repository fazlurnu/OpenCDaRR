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

**Perception**: by default each aircraft sees the others' broadcasts directly (instant, perfect
delivery), with optional GNSS self-noise (``navigation`` + ``rng``). Passing ``communication`` /
``surveillance`` / ``comm_rng`` (6f) makes perception **lossy and asymmetric** over the n(n−1)
directed links — per-link reception + latency, each aircraft acting on the last message *that* link
delivered (or ``None`` before first contact ⇒ fly nominal) — mirroring :func:`run_encounter`.

**Reduces to the pairwise runner** at n = 2: ``run_fleet`` with two agents reproduces
:func:`~opencdarr.loop.run_encounter` (no-communication path) bit-for-bit — the free multi-aircraft
regression (ADR 0004). Pure given its inputs; no globals. The fleet of states migrates into the IPS
particle when Phase 8 lands.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from opencdarr import geo
from opencdarr.autopilot import Autopilot, CruiseAutopilot, GuidanceMemory, nominal_velocity
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import (
    CommunicationModel,
    NavigationModel,
    SurveillanceModel,
)
from opencdarr.cns.broadcast import BroadcastSchedule
from opencdarr.cns.stack import CNS, CnsState, CnsStreams
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.dynamics import Dynamics, MotionCommand
from opencdarr.kinematics import relative_enu
from opencdarr.loop import _DEFAULT_DYNAMICS, _setpoint_adapter
from opencdarr.performance import Performance
from opencdarr.separation import INACTIVE, FleetMemory, SeparationManager
from opencdarr.state import AircraftState, DesiredVelocity
from opencdarr.wind import NO_WIND, WindField

_DEFAULT_SCHEDULE = BroadcastSchedule()  # interval 1 s, aligned, no jitter (default singleton)


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


def _all_clear(states: list[AircraftState], mems: list[FleetMemory], rpz: float) -> bool:
    """Is the whole fleet done — every pair past CPA and separated, and nobody still resolving?"""
    if any(m.resolving for m in mems):
        return False
    for i in range(len(states)):
        for j in range(i + 1, len(states)):
            rel = relative_enu(states[i], states[j])
            diverging = rel.rx * rel.vx + rel.ry * rel.vy > 0.0
            if not (diverging and rel.dist >= rpz):
                return False
    return True


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
    communication: CommunicationModel | None = None,
    surveillance: SurveillanceModel | None = None,
    comm_rng: np.random.Generator | None = None,
    t_max: float = 600.0,
    done_timeout: float = 10.0,
    schedule: BroadcastSchedule = _DEFAULT_SCHEDULE,
    broadcast_rng: np.random.Generator | None = None,
    share_intent: bool = False,
) -> FleetOutcome:
    """Advance the fleet to termination and report its outcome (see the module docstring).

    Each aircraft decides on the broadcast cadence from its (optionally noisy) self-fix against its
    perceived traffic. Without ``communication`` this is every *other* aircraft's current broadcast
    (perfect delivery); with it, each aircraft reads :class:`SurveillanceModel`'s ``perceived`` per
    directed link — the last message *that* link delivered, or ``None`` (absent) before first
    contact, so that neighbour is dropped from the perceived set until first heard. The command
    is held while the dynamics integrate at ``dt``; all aircraft advance together. The outcome
    (conflict / LoS / min-sep) is measured on the **true** states every step. Terminates once every
    pair is diverging and separated and no aircraft is resolving for ``done_timeout``, or at
    ``t_max``.

    ``schedule`` (a :class:`~opencdarr.cns.broadcast.BroadcastSchedule`) owns the transmit timing —
    the interval, an optional per-aircraft phase offset, and optional per-transmission jitter. The
    default (interval 1 s, aligned phase, no jitter) is today's behaviour and the reduction to
    :func:`~opencdarr.loop.run_encounter` at n = 2. A non-zero ``schedule.jitter`` requires
    ``broadcast_rng`` (its own substream, ADR 0006 §6); aircraft ``i`` broadcasts *and* decides on
    its own clock, reading each other aircraft's **last** transmitted state rather than a
    synchronous snapshot.
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
    # each aircraft's own broadcast clock, owned by the schedule: aligned at t = 0 by default (one
    # shared cadence, the pessimally-correlated case), or offset per aircraft for unsynchronised
    # transmitters (BroadcastSchedule.phase / vault/observations/broadcast-phase-offset.md)
    next_bc = schedule.initial(n)
    if schedule.jitter > 0.0 and broadcast_rng is None:
        raise ValueError("broadcast jitter requires broadcast_rng (a substream, ADR 0006 §6)")
    # the datalink as one stack (N → C → S): CNS is the shared, immutable config; its generators
    # ride in a per-particle CnsStreams and its value state threads as a clonable CnsState
    cns = CNS(
        navigation=navigation,
        communication=communication,
        surveillance=surveillance,
        share_intent=share_intent,
    )
    cns_streams = CnsStreams(nav=rng, comm=comm_rng)
    cns_state = CnsState.initial(n)
    eps = 1e-9  # float guard so a tick lands on a broadcast time reached by dt steps

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

        # aircraft whose own broadcast clock is due this tick: all of them together in the aligned
        # default, a per-aircraft subset once phases are offset
        firing = schedule.due(next_bc, t, eps)
        if firing:
            # the datalink runs first and whole (fix → transmit → hear), so every firing aircraft
            # transmits a pre-decision snapshot and nobody reacts to an unbroadcast manoeuvre
            cns_state, perception = cns.sense(states, firing, t, cns_state, cns_streams)
            for i in firing:
                see = perception[i]
                nom, gms[i] = aps[i].step(see.own, gms[i], perfs[i])
                # intent as a velocity: what this aircraft would fly if it reverted to nominal now
                # (the live mission command), stamped for the decision and persisted on the true
                # state for the next transmit. Byte-identical for a frozen CruiseAutopilot.
                self_i = replace(see.own, desired=nominal_velocity(nom, see.own))
                states[i] = replace(states[i], desired=self_i.desired)
                cmds[i], mems[i] = separation.step(
                    self_i, see.traffic, nom, mems[i], rpz, t_lookahead,
                    detector, resolver, recovery, adapters[i],
                )
                # next broadcast time: a fixed interval, or dithered per transmission by the
                # schedule's jitter (ADS-B slot randomisation), drawn in agent order
                next_bc[i] = schedule.advance(next_bc[i], broadcast_rng)

        # advance all aircraft from their pre-step states (explicitly simultaneous)
        states = [dyns[i].step(states[i], cmds[i], perfs[i], dt, wind) for i in range(n)]
        t += dt

        done_timer = done_timer + dt if _all_clear(states, mems, rpz) else 0.0
        if done_timer >= done_timeout:
            break

    return FleetOutcome(conflict=conflict, los=los, min_sep=min_sep)
