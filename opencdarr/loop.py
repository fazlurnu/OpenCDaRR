"""The pairwise encounter runner — the environment for one directed encounter.

`run_encounter` advances two aircraft to termination. The CDR layers run on a **broadcast
cadence** (``broadcast_interval``, the ADS-L/ASAS decision rate — 1 Hz in the reference), not
every integration step: at each broadcast tick each aircraft takes a fresh noisy self-measurement
and decides (detect → resolve, or recover → resume) for **both directed pairs** (A→B, B→A) on
its *perceived* view; the resulting command is then **held** while the encounter's
:class:`~opencdarr.dynamics.Dynamics` model (:class:`~opencdarr.dynamics.Multirotor` by
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

from opencdarr import geo
from opencdarr.autopilot import Autopilot, CruiseAutopilot, GuidanceMemory
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
from opencdarr.dynamics import Dynamics, MotionCommand, Multirotor
from opencdarr.kinematics import relative_enu
from opencdarr.performance import Performance
from opencdarr.separation import INACTIVE, PairMemory, SeparationManager
from opencdarr.state import AircraftState, DesiredVelocity

# module-level singleton, not a call in the signature default (ruff B008) - safe to share
# since Multirotor is stateless (ADR 0007)
_DEFAULT_DYNAMICS: Dynamics = Multirotor()


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
    dynamics: Dynamics = _DEFAULT_DYNAMICS,
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
    own_autopilot: Autopilot | None = None,
    intr_autopilot: Autopilot | None = None,
) -> EncounterOutcome:
    """Run one pairwise encounter to termination and report its outcome.

    With ``resolver=None`` the aircraft fly their nominal paths (a baseline that *should* lose
    separation). With a resolver (and ideally a recovery criterion), they maneuver to clear.

    ``dynamics`` (default :class:`~opencdarr.dynamics.Multirotor`, ADR 0007) is how a
    :class:`Command` becomes motion each ``dt``; swap it for a different :class:`Dynamics`
    implementation (a different airframe, or a future wind-aware model) without forking this
    function.

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
    ownship's own, which is never stripped; for the *other* aircraft it falls back to the
    velocity perceived when the pair became active (:class:`PairMemory`) when intent wasn't
    shared.
    """
    if communication is not None and comm_rng is None:
        raise ValueError("communication requires comm_rng (its own RNG substream, ADR 0006 §6)")
    surveil = surveillance or LastKnown()
    own = replace(own, desired=DesiredVelocity.from_track_speed(own.trk, own.gs))
    intr = replace(intr, desired=DesiredVelocity.from_track_speed(intr.trk, intr.gs))
    # Layered flow (ADR 0011): a per-aircraft Autopilot produces the nominal command, the
    # SeparationManager overlays safety on it. CruiseAutopilot holds each aircraft's cruise
    # (heading, speed) frozen from the *true initial* state — byte-identical to the old frozen
    # ``nom_own`` / ``nom_intr`` — so this split reproduces the pre-Phase-4a IPR bit-for-bit.
    # Default to the frozen-cruise nominal (behaviour-preserving); a caller navigating a mission
    # passes a WaypointAutopilot per aircraft. Guidance progress rides in the threaded
    # GuidanceMemory (leg index), clonable like PairMemory (ADR 0014).
    ap_own: Autopilot = own_autopilot or CruiseAutopilot(own.trk, own.gs)
    ap_intr: Autopilot = intr_autopilot or CruiseAutopilot(intr.trk, intr.gs)
    gm_own = gm_intr = GuidanceMemory()
    separation = SeparationManager()  # stateless; memory rides in mem_own / mem_intr (ADR 0011 §5)
    mem_own = mem_intr = INACTIVE  # per-direction resopairs membership + inferred-intent memory
    cmd_own, gm_own = ap_own.step(own, gm_own, perf)
    cmd_intr, gm_intr = ap_intr.step(intr, gm_intr, perf)
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

            # guidance: each aircraft's nominal command + advanced guidance memory. A mission
            # autopilot navigates from the live self-fix (re-planned each tick, which is what makes
            # the resume-after-avoidance automatic); CruiseAutopilot ignores it and holds.
            nom_own, gm_own = ap_own.step(self_own, gm_own, perf)
            nom_intr, gm_intr = ap_intr.step(self_intr, gm_intr, perf)
            # safety overlay: SeparationManager may override the nominal, releasing back on
            # recovery. perceived_* is None before first contact on a lossy link -> fly nominal
            cmd_own, mem_own = separation.step(
                self_own, [] if perceived_intr is None else [perceived_intr], nom_own, mem_own,
                rpz, t_lookahead, detector, resolver, recovery,
            )
            cmd_intr, mem_intr = separation.step(
                self_intr, [] if perceived_own is None else [perceived_own], nom_intr, mem_intr,
                rpz, t_lookahead, detector, resolver, recovery,
            )
            next_broadcast += broadcast_interval

        # advance both from their pre-step states (explicitly simultaneous)
        own, intr = (
            dynamics.step(own, cmd_own, perf, dt),
            dynamics.step(intr, cmd_intr, perf, dt),
        )
        t += dt

        rel = relative_enu(own, intr)
        diverging = rel.rx * rel.vx + rel.ry * rel.vy > 0.0  # past CPA
        clear = diverging and rel.dist >= rpz and not mem_own.resolving and not mem_intr.resolving
        done_timer = done_timer + dt if clear else 0.0
        if done_timer >= done_timeout:
            break

    return EncounterOutcome(conflict=conflict, los=los, min_sep=min_sep)
