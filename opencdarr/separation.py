"""The separation manager — the detect-and-avoid safety overlay (ADR 0011, Phase 4a).

:class:`SeparationManager` answers *is the nominal command safe given nearby traffic, and should it
be temporarily overridden?* It is the DAA layer between the :class:`~opencdarr.autopilot.Autopilot`
(what the aircraft *wants* to do) and the :class:`~opencdarr.dynamics.base.Dynamics` (what it is
physically able to do): it runs detect → resolve → recover, replacing the nominal with an avoidance
command while a conflict is live and **releasing back to the nominal** on recovery — the
Mission → Offboard → Mission switch a real DAA-equipped vehicle flies (``vault/phase-4-plan.md``).

This is the loop's old ``_decide`` given a home and a name, **substantively unchanged** — the
control flow, the ``resopairs`` semantics, and the inferred-intent fallback are ported verbatim.

No-hidden-state invariant (load-bearing, ADR 0011 §5)
-----------------------------------------------------
:class:`SeparationManager` holds **no mutable object state**. The per-directed-pair CDR/recovery
memory is the :class:`PairMemory` value, threaded **in** to ``step`` and returned **out**, never
stored on ``self``. This is not stylistic: the interacting-particle system clones a particle by
copying its state, and any future-affecting value kept *outside* the state is silently shared
between clones — exactly the KI-1 recovery-state leak (``docs/lesson-learnt.md``), invisible at
1e-9. A stateful manager would reintroduce precisely the hazard the clonable design exists to
prevent — ``state.py``'s no-hidden-state invariant, and
[[0010-dynamics-subpackage-and-odometry-state]] §3.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from opencdarr.cd.base import ConflictDetector
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.dynamics import MotionCommand
from opencdarr.state import AircraftState, DesiredVelocity


@dataclass(frozen=True)
class PairMemory:
    """One aircraft's CDR memory about a **directed** pair — its ``resopairs`` entry.

    Born when the pair first becomes active, cleared when recovery resumes. A plain frozen value,
    threaded through :meth:`SeparationManager.step` rather than held on the manager object, so it
    clones with the particle when IPS lands (Step 5) — ``state.py``'s docstring names exactly these
    two fields together as the per-aircraft CDR/recovery memory the particle will carry.

    ``onset_velocity`` is the other aircraft's velocity **as perceived when the pair became
    active**, an *inferred* stand-in for its desired velocity when that wasn't shared: before a
    conflict starts the other was presumably flying its nominal path, so its velocity at onset
    approximates its intent. Declared intent always wins when present; this is the fallback
    (:class:`~opencdarr.state.DesiredVelocity`).
    """

    resolving: bool = False
    onset_velocity: DesiredVelocity | None = None


#: The inactive (no live conflict) memory — a fresh pair carries this.
INACTIVE = PairMemory()


class SeparationManager:
    """Detect → resolve → recover overlay: nominal command → final command (stateless object).

    A single instance is shared across an encounter's directed pairs; it holds nothing — all memory
    rides in the :class:`PairMemory` values threaded through :meth:`step`.
    """

    def step(
        self,
        state: AircraftState,
        perceived_traffic: list[AircraftState],
        nominal: MotionCommand,
        memory: PairMemory,
        rpz: float,
        t_lookahead: float,
        detector: ConflictDetector,
        resolver: ConflictResolver | None,
        recovery: RecoveryCriterion | None,
    ) -> tuple[MotionCommand, PairMemory]:
        """This aircraft's command and new :class:`PairMemory` (directed: ``state`` vs its
        perceived other).

        Mirrors the reference control flow exactly (``resumenav_cpa`` + ``resopairs`` + the env's
        apply step). ``memory.resolving`` is our ``resopairs`` membership. Each tick:

        1. ``resopairs = resopairs ∪ confpairs`` — a current detection makes the pair active. On
           the tick a pair *becomes* active, the other's currently-perceived velocity is recorded
           as ``onset_velocity`` (the reference's ``_intr_init_vel``, at the same moment).
        2. **Recovery runs on every active pair**, including a freshly-detected one: if
           ``should_resume`` (past-CPA, not in LoS, not bouncing) the pair leaves ``resopairs``,
           reverts to **nominal**, and its memory is cleared. This is the key point — a pair that
           is detected *but already past CPA* (common under near-parallel measurement noise)
           reverts rather than maneuvering.
        3. Otherwise the aircraft follows the resolution: MVP while currently in ``confpairs``
           (detected), else **coast** on its current velocity (active but detection cleared).

        A resolution force therefore acts only on ``confpairs``; recovery on all of ``resopairs``.

        Intent-based recovery criteria (:class:`~opencdarr.crr.FTR`,
        :class:`~opencdarr.crr.ProbabilisticFTR`) read the other's ``desired`` velocity. When it
        was not shared, ``onset_velocity`` is substituted into ``other.desired`` here, so those
        criteria need no extra argument and stay unchanged — declared intent, when present, is
        never overwritten.

        ``perceived_traffic`` is empty when Phase 3b's
        :class:`~opencdarr.cns.base.SurveillanceModel` reports that ``state`` has never received
        anything from that source (before first contact on a lossy link) — it cannot avoid a threat
        it has never heard of, so it flies nominal (ADR 0006 §5). The list is the n>2
        future-proofed shape (ADR 0011 §6); the loop feeds the single perceived other (or ``[]``).
        """
        other = perceived_traffic[0] if perceived_traffic else None
        if resolver is None or other is None:
            return nominal, INACTIVE  # resolution disabled, or nothing received: fly nominal

        detected = detector.detect(state, other, rpz, t_lookahead)
        if not (memory.resolving or detected):  # resopairs.update(confpairs)
            return nominal, INACTIVE

        # record the other's velocity on the tick this pair becomes active — the inferred-intent
        # fallback, captured before any avoidance maneuver has had a chance to distort it
        onset = memory.onset_velocity or DesiredVelocity.from_track_speed(other.trk, other.gs)
        active = PairMemory(resolving=True, onset_velocity=onset)
        if other.desired is None:
            other = replace(other, desired=onset)  # inferred; declared intent is never overwritten

        if recovery is not None and recovery.should_resume(state, other, rpz):
            return nominal, INACTIVE  # recovery clears the pair from resopairs -> nominal
        if detected:
            return resolver.resolve(state, other, rpz), active  # in confpairs: MVP
        # active but detection cleared: coast on the current velocity
        return MotionCommand.from_track_speed(state.trk, state.gs), active
