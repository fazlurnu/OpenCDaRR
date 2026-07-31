"""The pairwise encounter runner — the environment for one directed encounter.

`run_encounter` advances two aircraft to termination. The CDR layers run on a **broadcast
cadence** (``broadcast_interval``, the ADS-L/ASAS decision rate — 1 Hz in the reference), not
every integration step: at each broadcast tick each aircraft takes a fresh noisy self-measurement
and decides (detect → resolve, or recover → resume) for **both directed pairs** (A→B, B→A) on
its *perceived* view; the resulting command is then **held** while the encounter's
:class:`~opencdarr.kinematics.Kinematics` model (:class:`~opencdarr.kinematics.Multirotor` by
default, ADR 0007) integrates at ``dt`` until the next tick. Deciding every step instead would
re-draw independent noise 1/``dt``×
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

from opencdarr.autopilot import Autopilot, CruiseAutopilot, GuidanceMemory, nominal_velocity
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import (
    CommunicationModel,
    NavigationModel,
    SurveillanceModel,
)
from opencdarr.cns.broadcast import BroadcastSchedule
from opencdarr.cns.stack import CNS, CnsStreams
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.kinematics import FixedWing, Kinematics, MotionCommand, Multirotor
from opencdarr.performance import Performance
from opencdarr.relative import relative_enu, segment_min_range
from opencdarr.separation import (
    INACTIVE,
    PairMemory,
    SeparationManager,
    SetpointAdapter,
    project_to_fixedwing,
)
from opencdarr.state import AircraftState, DesiredVelocity
from opencdarr.wind import NO_WIND, WindField

# module-level singleton, not a call in the signature default (ruff B008) - safe to share
# since Multirotor is stateless (ADR 0007)
_DEFAULT_KINEMATICS: Kinematics = Multirotor()

_BROADCAST_EPS = 1e-9  # float guard so a tick lands on a broadcast time reached by dt steps


def _setpoint_adapter(kinematics: Kinematics, perf: Performance) -> SetpointAdapter | None:
    """The command projection this airframe needs before its final command reaches the kinematics.

    A :class:`~opencdarr.kinematics.FixedWing` cannot fly a raw velocity (ADR 0013 §4), so its
    final command is projected onto course/airspeed
    (:func:`~opencdarr.separation.project_to_fixedwing`, Phase 4e); a
    :class:`~opencdarr.kinematics.Multirotor` flies the resolver's velocity directly, so it needs
    no projection (``None`` = identity, the byte-identical pre-Phase-4e path). The loop is the
    composition root pairing an airframe with its adapter — the manager stays vehicle-neutral.
    """
    if isinstance(kinematics, FixedWing):
        return lambda command: project_to_fixedwing(command, perf)
    return None


@dataclass(frozen=True)
class EncounterOutcome:
    """What one encounter produced."""

    conflict: bool  # was a conflict predicted at any step?
    los: bool  # was separation ever lost?
    min_sep: float  # minimum separation reached [m]


# The separation (detect → resolve → recover) logic now lives in ``opencdarr/separation.py`` as
# :class:`~opencdarr.separation.SeparationManager` (ADR 0011, Phase 4a). ``PairMemory`` /
# ``INACTIVE`` are re-exported from there; the module-level ``_INACTIVE`` alias and the ``_decide``
# shim below keep the pre-Phase-4a call surface (``loop._decide`` / ``loop._INACTIVE``) working
# byte-for-byte for the tests and scripts that import them directly, via a shared manager.
_INACTIVE = INACTIVE
_SEPARATION = SeparationManager()


def _decide(
    ac: AircraftState,
    other: AircraftState | None,
    nominal: MotionCommand,
    memory: PairMemory,
    rpz: float,
    t_lookahead: float,
    detector: ConflictDetector,
    resolver: ConflictResolver | None,
    recovery: RecoveryCriterion | None,
) -> tuple[MotionCommand, PairMemory]:
    """Backward-compatible shim for :meth:`SeparationManager.step` (see the note above).

    Adapts the single perceived ``other`` (possibly ``None``) to the manager's
    ``perceived_traffic`` list, then delegates — identical decisions to the old ``_decide``.
    """
    return _SEPARATION.step(
        ac,
        [] if other is None else [other],
        nominal,
        memory,
        rpz,
        t_lookahead,
        detector,
        resolver,
        recovery,
    )


def run_encounter(
    own: AircraftState,
    intr: AircraftState,
    *,
    perf: Performance,
    kinematics: Kinematics = _DEFAULT_KINEMATICS,
    own_kinematics: Kinematics | None = None,
    intr_kinematics: Kinematics | None = None,
    own_perf: Performance | None = None,
    intr_perf: Performance | None = None,
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
    schedule: BroadcastSchedule | None = None,
    broadcast_rng: np.random.Generator | None = None,
    share_intent: bool = False,
    own_autopilot: Autopilot | None = None,
    intr_autopilot: Autopilot | None = None,
    wind: WindField = NO_WIND,
) -> EncounterOutcome:
    """Run one pairwise encounter to termination and report its outcome.

    With ``resolver=None`` the aircraft fly their nominal paths (a baseline that *should* lose
    separation). With a resolver (and ideally a recovery criterion), they maneuver to clear.

    ``kinematics`` (default :class:`~opencdarr.kinematics.Multirotor`, ADR 0007) is how a
    :class:`Command` becomes motion each ``dt``; swap it for a different :class:`Kinematics`
    implementation (a different airframe, or a future wind-aware model) without forking this
    function.

    **Mixed fleet (ADR 0011 §7, Phase 4e):** ``kinematics`` / ``perf`` are the *shared* airframe;
    pass ``own_kinematics`` / ``own_perf`` / ``intr_kinematics`` / ``intr_perf`` to give a side its
    own bundle (each defaults to the shared one), so a multirotor-vs-fixed-wing encounter runs
    through this same entry point the IPR sweeps use. Each aircraft's autopilot and separation
    overlay are stepped with *its* ``perf``, it is advanced by *its* ``kinematics``, and a
    fixed-wing airframe automatically gets the velocity→course projection its final command needs
    (:func:`_setpoint_adapter`) — MVP/VO stay vehicle-neutral (they still emit a velocity).

    ``wind`` (default :data:`~opencdarr.wind.NO_WIND`, Phase 5) is the shared, steady environment
    field threaded into every ``kinematics.step`` — one field for both aircraft, read-only, never
    stored on either state (ADR 0016). At ``NO_WIND`` the encounter is byte-identical to Phase 4.

    The CDR layers run every ``broadcast_interval`` seconds (the ADS-L/ASAS decision rate), not
    every ``dt``: at each tick each aircraft takes a fresh noisy self-measurement and **decides**
    on its *perceived* view; the resulting command is **held** until the next tick. Without a
    ``navigation`` model (and ``rng``) the self-measurement is the true state (Phase 2 behaviour).

    ``broadcast_interval`` is the scalar spelling of that cadence and covers the common case.
    ``schedule`` is the full one: a :class:`~opencdarr.cns.broadcast.BroadcastSchedule` carrying
    per-aircraft **phase** offsets and per-transmission **jitter** as well as the interval, which
    the scalar cannot express. Passing it supersedes ``broadcast_interval``; a non-zero
    ``schedule.jitter`` needs ``broadcast_rng`` (its own substream, ADR 0006 §6). This is the same
    object ``run_fleet`` takes, and giving both runners one transmit-timing model is what makes the
    n = 2 reduction hold at *any* schedule rather than only at the aligned default — before this,
    the two aircraft here shared one global broadcast clock and could not be off-phased at all.

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
    ownship's own, which is never stripped; for the *other* aircraft it falls back to the
    velocity perceived when the pair became active (:class:`PairMemory`) when intent wasn't
    shared.
    """
    # the datalink as one stack (N → C → S), shared with ``run_fleet``: CNS is the immutable
    # config, its generators ride in a per-particle CnsStreams, its value state threads as a
    # clonable CnsState. Same split and order as run_fleet — the two are provably equal at n = 2.
    cns = CNS(
        navigation=navigation,
        communication=communication,
        surveillance=surveillance,
        share_intent=share_intent,
    )
    cns_streams = CnsStreams(nav=rng, comm=comm_rng)
    # transmit timing: the scalar spelling unless a full schedule is given. Both runners now share
    # one BroadcastSchedule, so phase offsets and jitter mean the same thing here as in run_fleet
    # and the n = 2 reduction is no longer conditional on the aligned default.
    sched = BroadcastSchedule(interval=broadcast_interval) if schedule is None else schedule
    if sched.jitter > 0.0 and broadcast_rng is None:
        raise ValueError("broadcast jitter requires broadcast_rng (a substream, ADR 0006 §6)")
    # per-aircraft bundle (ADR 0011 §7): each side falls back to the shared kinematics/perf, so the
    # single-airframe callers (and the bit-for-bit anchors) are unchanged; a mixed-fleet caller
    # overrides one or both sides. The setpoint adapter is airframe-derived (fixed-wing: project).
    # Held per aircraft in index order (own = 0, intr = 1) — the same shape run_fleet threads, so
    # the broadcast tick below can act on the subset that is actually firing.
    per_side = [own_kinematics or kinematics, intr_kinematics or kinematics]
    perfs = [own_perf or perf, intr_perf or perf]
    adapters = [_setpoint_adapter(per_side[i], perfs[i]) for i in range(2)]
    states = [
        replace(ac, desired=DesiredVelocity.from_track_speed(ac.trk, ac.gs)) for ac in (own, intr)
    ]
    # Layered flow (ADR 0011): a per-aircraft Autopilot produces the nominal command, the
    # SeparationManager overlays safety on it. CruiseAutopilot holds each aircraft's cruise
    # (heading, speed) frozen from the *true initial* state — byte-identical to the old frozen
    # ``nom_own`` / ``nom_intr`` — so this split reproduces the pre-Phase-4a IPR bit-for-bit.
    # Default to the frozen-cruise nominal (behaviour-preserving); a caller navigating a mission
    # passes a WaypointAutopilot per aircraft. Guidance progress rides in the threaded
    # GuidanceMemory (leg index), clonable like PairMemory (ADR 0014).
    aps: list[Autopilot] = [
        own_autopilot or CruiseAutopilot(own.trk, own.gs),
        intr_autopilot or CruiseAutopilot(intr.trk, intr.gs),
    ]
    separation = SeparationManager()  # stateless; memory rides in ``mems`` (ADR 0011 §5)
    mems = [INACTIVE, INACTIVE]  # per-direction resopairs membership + inferred-intent memory
    gms: list[GuidanceMemory] = []
    cmds: list[MotionCommand] = []
    for i in range(2):
        cmd, gm = aps[i].step(states[i], GuidanceMemory(), perfs[i])
        cmds.append(cmd)
        gms.append(gm)
    cns_state = cns.initial_state(2)

    conflict = los = False
    min_sep = float("inf")
    done_timer = 0.0
    t = 0.0
    next_bc = sched.initial(2)  # per-aircraft broadcast clock: aligned at 0, or the phase offsets

    while t < t_max:
        # the pre-step geometry; separation itself is measured across the whole step, after
        # integrating, so a pass that dips inside rpz and back out within one dt is not missed
        # (``relative.segment_min_range``)
        rel_pre = relative_enu(states[0], states[1])
        if detector.detect(states[0], states[1], rpz, t_lookahead) or detector.detect(
            states[1], states[0], rpz, t_lookahead
        ):
            conflict = True

        # CDR decisions on the broadcast cadence; the command is held between ticks. Both aircraft
        # fire together on the aligned default; once phases are offset it is a per-aircraft subset.
        firing = sched.due(next_bc, t, _BROADCAST_EPS)
        if firing:
            # the datalink, whole: each firing aircraft takes its (noisy) self-fix and puts it on
            # the air (intent stripped at transmit time unless shared), then is told what it now
            # holds of the other — absent before first contact on a lossy link, which flies that
            # pair nominal (ADR 0006 §5). Same stack, same order, as ``run_fleet``.
            cns_state, perception = cns.sense(states, firing, t, cns_state, cns_streams)
            for i in firing:
                see = perception[i]
                # guidance: this aircraft's nominal command + advanced guidance memory. A mission
                # autopilot navigates from the live self-fix (re-planned each tick, which is what
                # makes resume-after-avoidance automatic); CruiseAutopilot ignores it and holds.
                nom, gms[i] = aps[i].step(see.own, gms[i], perfs[i])
                # intent as a velocity: what it would fly if it reverted to nominal *now* (the live
                # mission command, not a value frozen at t=0), so intent-based recovery (FTR) tests
                # the velocity the aircraft will actually resume. Byte-identical for a frozen
                # CruiseAutopilot. Stamped on the self-fix for this decision and persisted on the
                # true state so the next tick's transmit carries it under ``share_intent``.
                self_i = replace(see.own, desired=nominal_velocity(nom, see.own))
                states[i] = replace(states[i], desired=self_i.desired)
                # safety overlay: SeparationManager may override the nominal, releasing back on
                # recovery. adapters[i] projects the final command onto this airframe's channels
                # (fixed-wing: a velocity override -> course/airspeed; multirotor: None, direct).
                cmds[i], mems[i] = separation.step(
                    self_i, see.traffic, nom, mems[i],
                    rpz, t_lookahead, detector, resolver, recovery, adapters[i],
                )
                # next broadcast: a fixed interval, or one dithered per transmission by the
                # schedule's jitter (ADS-B slot randomisation), drawn in agent order
                next_bc[i] = sched.advance(next_bc[i], broadcast_rng)

        # advance both from their pre-step states (explicitly simultaneous), each by its airframe.
        # ``wind`` is the shared environment field (default NO_WIND -> Phase-4 behaviour, 5a).
        states = [per_side[i].step(states[i], cmds[i], perfs[i], dt, wind) for i in range(2)]
        t += dt

        rel = relative_enu(states[0], states[1])

        # separation over the step just flown; consecutive segments share an endpoint, so the
        # running minimum covers the trajectory continuously rather than at sampled instants
        sep = segment_min_range(rel_pre, rel)
        min_sep = min(min_sep, sep)
        if sep < rpz:
            los = True

        diverging = rel.rx * rel.vx + rel.ry * rel.vy > 0.0  # past CPA
        clear = diverging and rel.dist >= rpz and not any(m.resolving for m in mems)
        done_timer = done_timer + dt if clear else 0.0
        if done_timer >= done_timeout:
            break

    return EncounterOutcome(conflict=conflict, los=los, min_sep=min_sep)
