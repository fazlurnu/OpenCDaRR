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

from collections.abc import Sequence
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


def random_broadcast_phase(
    n: int, broadcast_interval: float, rng: np.random.Generator
) -> list[float]:
    """Draw an independent initial broadcast offset in ``[0, broadcast_interval)`` per aircraft.

    The realistic unsynchronised-transmitter model: aircraft spawn at different times, so each runs
    the *same* interval at a *random phase* rather than a shared ``t = 0`` tick — the aligned default
    is the pessimally-correlated case where every aircraft's staleness peaks together (real ADS-B even
    dithers its slot to avoid exactly that). Seed-reproducible and clone-safe when ``rng`` is a
    spawned substream (ADR 0001). Pass the result as ``run_fleet(..., broadcast_phase=...)``. See
    ``vault/observations/broadcast-phase-offset.md``.
    """
    return [float(x) for x in rng.uniform(0.0, broadcast_interval, n)]


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
    broadcast_phase: Sequence[float] | None = None,
    broadcast_jitter: float = 0.0,
    broadcast_rng: np.random.Generator | None = None,
    share_intent: bool = False,
) -> FleetOutcome:
    """Advance the fleet to termination and report its outcome (see the module docstring).

    Each aircraft decides on the broadcast cadence from its (optionally noisy) self-fix against its
    perceived traffic — every *other* aircraft's broadcast (perfect delivery here). The command
    is held while the dynamics integrate at ``dt``; all aircraft advance together. The outcome
    (conflict / LoS / min-sep) is measured on the **true** states every step. Terminates once every
    pair is diverging and separated and no aircraft is resolving for ``done_timeout``, or at
    ``t_max``.

    ``broadcast_phase`` offsets each aircraft's broadcast clock: ``None`` (default) aligns all of
    them at ``t = 0`` (one shared cadence — today's behaviour, and the reduction to
    :func:`~opencdarr.loop.run_encounter` at n = 2). Pass one offset per aircraft (e.g. from
    :func:`random_broadcast_phase`) to model unsynchronised transmitters spawned at different times;
    aircraft ``i`` then broadcasts *and* decides at ``phase[i] + k·broadcast_interval``, reading each
    other aircraft's **last** transmitted state rather than a synchronous snapshot.

    ``broadcast_jitter`` (seconds, default 0 ⇒ fixed interval) dithers *each* gap: the next broadcast
    lands ``broadcast_interval + U(-jitter, +jitter)`` later, the per-transmission slot randomisation
    real ADS-B uses to avoid systematic co-channel collisions (a fixed offset only shifts the comb;
    jitter breaks its regularity). Requires ``broadcast_rng`` — its own substream (ADR 0006 §6) — and
    must be ``< broadcast_interval`` so gaps stay positive.
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
    # each aircraft's own broadcast clock: aligned at t = 0 by default (one shared cadence, the
    # pessimally-correlated case), or offset per aircraft for unsynchronised transmitters
    # (random_broadcast_phase / vault/observations/broadcast-phase-offset.md)
    if broadcast_phase is None:
        phases = [0.0] * n
    else:
        if len(broadcast_phase) != n:
            raise ValueError(
                f"broadcast_phase must have one entry per aircraft (got {len(broadcast_phase)}, n={n})"
            )
        if any(p < 0.0 for p in broadcast_phase):
            raise ValueError(f"broadcast_phase entries must be >= 0, got {list(broadcast_phase)}")
        phases = [float(p) for p in broadcast_phase]
    if broadcast_jitter < 0.0:
        raise ValueError(f"broadcast_jitter must be >= 0, got {broadcast_jitter}")
    if broadcast_jitter >= broadcast_interval:
        raise ValueError(
            f"broadcast_jitter must be < broadcast_interval ({broadcast_interval}), "
            f"got {broadcast_jitter}"
        )
    if broadcast_jitter > 0.0 and broadcast_rng is None:
        raise ValueError("broadcast_jitter requires broadcast_rng (its own substream, ADR 0006 §6)")
    next_bc = list(phases)
    last_tx: list[AircraftState | None] = [None] * n  # each aircraft's last transmitted self-fix
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

        # aircraft whose own broadcast clock is due this tick: all of them together in the aligned
        # default, a per-aircraft subset once phases are offset
        firing = [i for i in range(n) if t + eps >= next_bc[i]]
        if firing:
            # pass 1 — each firing aircraft takes its (noisy) self-fix and latches what it transmits,
            # in agent order, BEFORE any decision. Aligned phases fire all n in order, so the draws
            # and the transmit snapshot are bit-for-bit with the old single-cadence path. An aircraft
            # keeps its own intent; the broadcast strips it unless shared.
            selfs: dict[int, AircraftState] = {}
            for i in firing:
                if navigation is not None and rng is not None:
                    fix = navigation.measure(states[i], t, rng).state
                else:
                    fix = states[i]
                selfs[i] = replace(fix, desired=states[i].desired)
                last_tx[i] = replace(fix, desired=states[i].desired if share_intent else None)
            # pass 2 — each firing aircraft decides against the latest broadcast it holds of every
            # other aircraft (its last_tx latch). Perfect delivery this pass: the latch is each
            # aircraft's most recent transmit, None before it has ever broadcast (fly nominal).
            for i in firing:
                nom, gms[i] = aps[i].step(selfs[i], gms[i], perfs[i])
                perceived = [
                    tx for j in range(n) if j != i and (tx := last_tx[j]) is not None
                ]
                cmds[i], mems[i] = separation.step(
                    selfs[i], perceived, nom, mems[i], rpz, t_lookahead,
                    detector, resolver, recovery, adapters[i],
                )
                # next gap: fixed by default, or dithered per transmission (ADS-B slot randomisation)
                step = broadcast_interval
                if broadcast_jitter > 0.0:
                    assert broadcast_rng is not None  # guaranteed by validation above
                    step += float(broadcast_rng.uniform(-broadcast_jitter, broadcast_jitter))
                next_bc[i] += step

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
