"""The pairwise encounter runner — the environment for one directed encounter.

`run_encounter` advances two aircraft to termination. The CDR layers run on a **broadcast
cadence** (``broadcast_interval``, the ADS-L/ASAS decision rate — 1 Hz in the reference), not
every integration step: at each broadcast tick each aircraft takes a fresh noisy self-measurement
and decides (detect → resolve, or recover → resume) for **both directed pairs** (A→B, B→A) on
its *perceived* view; the resulting command is then **held** while `step_dynamics` integrates at
``dt`` until the next tick. Deciding every step instead would re-draw independent noise 1/``dt``×
per second and average it away — unphysically robust. Truth is used only to score the encounter
(conflict predicted? separation lost? minimum separation?) — the raw material for IPR.

**Without a ``communication`` model** (Phase 2/3a): each broadcast is the *other's* perceived
view directly — instant, perfect delivery. **With one** (Phase 3b): each broadcast is offered to
:class:`~opencdarr.cns.base.CommunicationModel` (reception + latency), and a decision reads
:class:`~opencdarr.cns.base.SurveillanceModel`'s ``perceived`` — the last message that link
actually delivered, or ``None`` before first contact (ADR 0006 §5: no data ⇒ fly nominal). An
aircraft's own self-fix never goes through communication — it always knows itself exactly.

Pure given its inputs; no globals. Each aircraft's nominal navigation is captured from its
initial state and held here (it migrates into the particle state when IPS lands, Step 5).
This is the pairwise precursor to the `advance` / `is_terminal` interface.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from opencdarr import geo
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import (
    CommState,
    CommunicationModel,
    Message,
    NavigationModel,
    SurveillanceModel,
)
from opencdarr.cns.surveillance import LastKnown
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.dynamics import Command, step_dynamics
from opencdarr.kinematics import relative_enu
from opencdarr.performance import Performance
from opencdarr.state import AircraftState, DesiredVelocity


@dataclass(frozen=True)
class EncounterOutcome:
    """What one encounter produced."""

    conflict: bool  # was a conflict predicted at any step?
    los: bool  # was separation ever lost?
    min_sep: float  # minimum separation reached [m]


def _decide(
    ac: AircraftState,
    other: AircraftState | None,
    nominal: Command,
    resolving: bool,
    rpz: float,
    t_lookahead: float,
    detector: ConflictDetector,
    resolver: ConflictResolver | None,
    recovery: RecoveryCriterion | None,
) -> tuple[Command, bool]:
    """One aircraft's command and its new ``resopairs`` flag (directed: ac vs its perceived other).

    Mirrors the reference control flow exactly (``resumenav_cpa`` + ``resopairs`` + the env's
    apply step). ``resolving`` is our ``resopairs`` membership. Each tick:

    1. ``resopairs = resopairs ∪ confpairs`` — a current detection makes the pair active.
    2. **Recovery runs on every active pair**, including a freshly-detected one: if
       ``should_resume`` (past-CPA, not in LoS, not bouncing) the pair leaves ``resopairs`` and
       reverts to **nominal**. This is the key point — a pair that is detected *but already past
       CPA* (common under near-parallel measurement noise) reverts rather than maneuvering.
    3. Otherwise the aircraft follows the resolution: MVP while currently in ``confpairs``
       (detected), else **coast** on its current velocity (active but detection cleared).

    A resolution force therefore acts only on ``confpairs``; recovery acts on all of ``resopairs``.

    ``other`` is ``None`` when Phase 3b's :class:`~opencdarr.cns.base.SurveillanceModel` reports
    that ``ac`` has never received anything from that source (before first contact on a lossy
    link) — it cannot avoid a threat it has never heard of, so it flies nominal (ADR 0006 §5).
    """
    if resolver is None or other is None:
        return nominal, False  # resolution disabled, or nothing ever received: fly nominal

    detected = detector.detect(ac, other, rpz, t_lookahead)
    active = resolving or detected  # resopairs.update(confpairs)
    if not active:
        return nominal, False

    if recovery is not None and recovery.should_resume(ac, other, rpz):
        return nominal, False  # recovery clears the pair from resopairs -> nominal
    if detected:
        return resolver.resolve(ac, other, rpz), True  # in confpairs: MVP
    return Command(hdg=ac.trk, spd=ac.gs), True  # in resopairs, detection cleared: coast


def run_encounter(
    own: AircraftState,
    intr: AircraftState,
    *,
    perf: Performance,
    rpz: float,
    t_lookahead: float,
    dt: float,
    detector: ConflictDetector,
    resolver: ConflictResolver | None = None,
    recovery: RecoveryCriterion | None = None,
    navigation: NavigationModel | None = None,
    rng: np.random.Generator | None = None,
    communication: CommunicationModel | None = None,
    surveillance: SurveillanceModel | None = None,
    comm_rng: np.random.Generator | None = None,
    t_max: float = 600.0,
    done_timeout: float = 10.0,
    broadcast_interval: float = 1.0,
    share_intent: bool = False,
) -> EncounterOutcome:
    """Run one pairwise encounter to termination and report its outcome.

    With ``resolver=None`` the aircraft fly their nominal paths (a baseline that *should* lose
    separation). With a resolver (and ideally a recovery criterion), they maneuver to clear.

    The CDR layers run every ``broadcast_interval`` seconds (the ADS-L/ASAS decision rate), not
    every ``dt``: at each tick each aircraft takes a fresh noisy self-measurement and **decides**
    on its *perceived* view; the resulting command is **held** until the next tick. Without a
    ``navigation`` model (and ``rng``) the self-measurement is the true state (Phase 2 behaviour).

    **``communication`` (Phase 3b, optional):** without it, a decision's *other* is the other
    aircraft's broadcast directly — instant, perfect delivery (Phase 3a behaviour, unchanged).
    With it, each broadcast is offered to ``communication`` (which needs ``comm_rng``, drawn from
    its **own** substream — ADR 0006 §6, never the same generator as ``rng``), and a decision's
    *other* is ``surveillance.perceived(...)`` — the last message that specific directed link
    actually delivered (``LastKnown``/hold-as-is by default: no dead-reckoning), or ``None``
    before the link's first delivery, which flies that pair nominal (see :func:`_decide`). An
    aircraft's own self-fix never passes through ``communication`` — it always knows itself
    exactly, whether or not it has ever heard from the other.

    The outcome (conflict, LoS, separation) is always measured on the **true** states, every
    step, regardless of communication. Terminates once the pair has been diverging and separated
    for ``done_timeout`` seconds, or at ``t_max``.

    Each aircraft's **intent** (its ``desired`` nominal velocity) is its initial state, held on the
    true state. It is private: another aircraft perceives it only when ``share_intent`` is True —
    stripped from the state **before** it is broadcast (so a dropped/held message never carries
    intent it wasn't sent with). Intent-based recovery (:class:`~opencdarr.crr.FTR`) reads the
    ownship's own, which is never stripped.
    """
    if communication is not None and comm_rng is None:
        raise ValueError("communication requires comm_rng (its own RNG substream, ADR 0006 §6)")
    surveil = surveillance or LastKnown()
    own = replace(own, desired=DesiredVelocity(own.trk, own.gs))
    intr = replace(intr, desired=DesiredVelocity(intr.trk, intr.gs))
    nom_own = Command(hdg=own.trk, spd=own.gs)
    nom_intr = Command(hdg=intr.trk, spd=intr.gs)
    resolving_own = resolving_intr = False
    cmd_own, cmd_intr = nom_own, nom_intr
    comm_state = CommState()

    conflict = los = False
    min_sep = float("inf")
    done_timer = 0.0
    t = 0.0
    next_broadcast = 0.0
    eps = 1e-9  # float guard so a tick lands on t = k*broadcast_interval reached by dt steps

    while t < t_max:
        _, sep = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
        min_sep = min(min_sep, sep)
        if sep < rpz:
            los = True
        if detector.detect(own, intr, rpz, t_lookahead) or detector.detect(
            intr, own, rpz, t_lookahead
        ):
            conflict = True

        # CDR decisions on the broadcast cadence; the command is held between ticks
        if t + eps >= next_broadcast:
            # each aircraft's fresh (noisy) self-fix; both endpoints carry noise
            if navigation is not None and rng is not None:
                fix_own = navigation.measure(own, t, rng).state
                fix_intr = navigation.measure(intr, t, rng).state
            else:
                fix_own, fix_intr = own, intr

            # an aircraft knows its own intent exactly, never through communication
            self_own = replace(fix_own, desired=own.desired)
            self_intr = replace(fix_intr, desired=intr.desired)
            # what leaves the transmitter: intent stripped here (before comm), not at perceive
            # time, so a dropped/held message never carries intent it was never sent with
            tx_own = replace(fix_own, desired=own.desired if share_intent else None)
            tx_intr = replace(fix_intr, desired=intr.desired if share_intent else None)

            if communication is not None:
                broadcasts = (
                    Message(source=own.id, state=tx_own, t_meas=t),
                    Message(source=intr.id, state=tx_intr, t_meas=t),
                )
                comm_state = communication.step(
                    comm_state, broadcasts, (own.id, intr.id), t, comm_rng
                )
                perceived_intr = surveil.perceived(comm_state, own.id, intr.id, t)
                perceived_own = surveil.perceived(comm_state, intr.id, own.id, t)
            else:
                perceived_intr, perceived_own = tx_intr, tx_own  # instant, perfect delivery

            cmd_own, resolving_own = _decide(
                self_own, perceived_intr, nom_own, resolving_own,
                rpz, t_lookahead, detector, resolver, recovery,
            )
            cmd_intr, resolving_intr = _decide(
                self_intr, perceived_own, nom_intr, resolving_intr,
                rpz, t_lookahead, detector, resolver, recovery,
            )
            next_broadcast += broadcast_interval

        # advance both from their pre-step states (explicitly simultaneous)
        own, intr = (
            step_dynamics(own, cmd_own, perf, dt),
            step_dynamics(intr, cmd_intr, perf, dt),
        )
        t += dt

        rel = relative_enu(own, intr)
        diverging = rel.rx * rel.vx + rel.ry * rel.vy > 0.0  # past CPA
        clear = diverging and rel.dist >= rpz and not resolving_own and not resolving_intr
        done_timer = done_timer + dt if clear else 0.0
        if done_timer >= done_timeout:
            break

    return EncounterOutcome(conflict=conflict, los=los, min_sep=min_sep)
