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
:class:`SeparationManager` holds **no mutable object state**. The per-aircraft CDR/recovery memory
is the :class:`FleetMemory` value (a set of active directed pairs, Phase 6), threaded **in** to
``step`` and returned **out**, never stored on ``self``. This is not stylistic: the
interacting-particle system clones a particle by
copying its state, and any future-affecting value kept *outside* the state is silently shared
between clones — exactly the KI-1 recovery-state leak (``docs/lesson-learnt.md``), invisible at
1e-9. A stateful manager would reintroduce precisely the hazard the clonable design exists to
prevent — ``state.py``'s no-hidden-state invariant, and
[[0010-dynamics-subpackage-and-odometry-state]] §3.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, replace

from opencdarr.cd.base import ConflictDetector
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.dynamics import MotionCommand
from opencdarr.performance import Performance
from opencdarr.state import AircraftState, DesiredVelocity

#: A projection from the vehicle-neutral final command onto the channels one airframe can fly —
#: injected per aircraft into :meth:`SeparationManager.step`. ``None`` means *no projection* (a
#: multirotor flies the resolver's velocity command directly). The one non-trivial adapter today is
#: :func:`project_to_fixedwing` (bound to an airframe's :class:`Performance`).
SetpointAdapter = Callable[[MotionCommand], MotionCommand]


def project_to_fixedwing(command: MotionCommand, perf: Performance) -> MotionCommand:
    """Project a resolver's avoidance **velocity** onto the fixed-wing course/airspeed channels.

    MVP/VO (and the coast fallback) emit a ground-**velocity** command — a native multirotor
    setpoint (PX4 ``TrajectorySetpoint.velocity``), but *not* a fixed-wing one: a fixed-wing takes
    a lateral course + a longitudinal airspeed, and :class:`~opencdarr.dynamics.FixedWing` fails
    fast on a raw velocity (ADR 0013 §4). This adapter is the missing link — it lowers the velocity
    onto the channels a fixed-wing can fly:

    - ``target_course`` = the **track** of the avoidance velocity (``atan2(v_east, v_north)``);
    - ``target_airspeed`` = its **magnitude**, clamped into ``[v_min, v_max]`` (stall .. max
      airspeed), so the projected setpoint is always inside the airframe envelope.

    It is deliberately an **approximation**, and the approximation *is* the physics: a multirotor
    reaches the commanded velocity essentially instantly, whereas a fixed-wing can only *converge*
    to that course under its bank/roll limit and ramp to that airspeed under ``ax``
    (:meth:`FixedWing.step` tracks the course turn-rate-limited, clamps the airspeed rate). The
    lag between the velocity the resolver asked for and the velocity the airframe is making good is
    the correct airframe difference, not a defect — MVP/VO stay vehicle-neutral (ADR 0011 §2), and
    the projection is realised here, at the separation layer, because the DAA *override* (not just
    the mission nominal) is what carries the velocity a fixed-wing cannot fly.

    A command that carries no ``target_velocity`` — a position/leg nominal from a
    :class:`~opencdarr.autopilot.WaypointAutopilot`, or an already-projected course command — is a
    valid fixed-wing setpoint and passes through untouched.
    """
    if command.target_velocity is None:
        return command  # a position/course command (mission nominal) -> already fixed-wing-flyable
    v_east, v_north = command.target_velocity
    course = math.degrees(math.atan2(v_east, v_north)) % 360.0
    airspeed = min(perf.v_max, max(perf.v_min, math.hypot(v_east, v_north)))
    return MotionCommand(target_course=course, target_airspeed=airspeed)


@dataclass(frozen=True)
class FleetMemory:
    """One aircraft's CDR memory — its ``resopairs`` set of active **directed** pairs (Phase 6).

    ``resopairs`` maps each active intruder's ``id`` to the **onset velocity** for that pair — the
    intruder's velocity as perceived when the pair became active, an *inferred* stand-in for its
    desired velocity when that wasn't shared (before a conflict the other was presumably flying its
    nominal path). Declared intent always wins when present; this is the fallback. Held as a sorted
    tuple so the value is immutable, comparable, and hashable — a clonable value threaded through
    :meth:`SeparationManager.step`, never on the manager object (the no-hidden-state invariant, one
    level up from the single-pair memory Phases 2–5 used). With more aircraft there is *more* of
    this per-aircraft memory, so a clone that lost any of it would diverge — the invariant is more
    load-bearing at scale, not less (ADR 0004).
    """

    resopairs: tuple[tuple[str, DesiredVelocity], ...] = ()

    @property
    def resolving(self) -> bool:
        """Whether this aircraft is actively resolving against *any* pair."""
        return bool(self.resopairs)

    @property
    def onset_velocity(self) -> DesiredVelocity | None:
        """Pairwise back-compat: the sole active pair's onset velocity, or ``None`` (n≠1)."""
        return self.resopairs[0][1] if len(self.resopairs) == 1 else None

    def onset_for(self, intruder_id: str) -> DesiredVelocity | None:
        """The recorded onset velocity for ``intruder_id`` if its pair is active, else ``None``."""
        for pair_id, onset in self.resopairs:
            if pair_id == intruder_id:
                return onset
        return None


#: Backward-compatible alias for the pre-Phase-6 single-pair memory name. The value is now a
#: per-aircraft :class:`FleetMemory` (a set of directed pairs); at n = 2 it carries one entry whose
#: ``resolving`` / ``onset_velocity`` read exactly as the old ``PairMemory`` did.
PairMemory = FleetMemory

#: The inactive (no live conflict) memory — a fresh aircraft carries this.
INACTIVE = FleetMemory()


class SeparationManager:
    """Detect → resolve → recover overlay: nominal command → final command (stateless object).

    A single instance is shared across the fleet; it holds nothing — all memory rides in the
    :class:`FleetMemory` values threaded through :meth:`step`.
    """

    def step(
        self,
        state: AircraftState,
        perceived_traffic: list[AircraftState],
        nominal: MotionCommand,
        memory: FleetMemory,
        rpz: float,
        t_lookahead: float,
        detector: ConflictDetector,
        resolver: ConflictResolver | None,
        recovery: RecoveryCriterion | None,
        adapter: SetpointAdapter | None = None,
    ) -> tuple[MotionCommand, FleetMemory]:
        """This aircraft's command and new :class:`FleetMemory` (directed: ``state`` vs its
        perceived traffic).

        The N-aircraft generalisation of the reference ``resopairs`` control flow (Phase 6, ADR
        0004); at ``len(perceived_traffic) == 1`` it is byte-identical to Phases 2–5. Each tick:

        1. ``resopairs = resopairs ∪ confpairs`` — a current detection against **any** perceived
           intruder makes that directed pair active. On the tick a pair *becomes* active, that
           intruder's currently-perceived velocity is recorded as its onset velocity.
        2. **Recovery runs per active pair**: if ``should_resume`` (past-CPA, not in LoS, not
           bouncing) for that pair, it leaves ``resopairs``. The aircraft reverts to **nominal**
           only once ``resopairs`` is empty — i.e. it is clear of **all** its conflicts (the
           aggregate "resume when clear of all" emerges from the per-pair removals, so the recovery
           criterion stays a directed pairwise primitive, unchanged).
        3. Otherwise it follows the resolution **against the currently-detected set** (the active
           ``confpairs``): the resolver composes them its own way — MVP sums, VO unions (ADR 0004 /
           Phase 6). With no current detection but a still-active pair it **coasts**.

        Resolution acts on ``confpairs``; recovery on all of ``resopairs``. The resolver is called
        with ``preferred=None`` (stay closest to the *current* velocity) — biasing VO toward the
        nominal destabilised it (see the note at the call site); return-to-nominal is CRR's job.

        Intent-based recovery (:class:`~opencdarr.crr.FTR`) reads the other's ``desired``; when not
        shared, the pair's onset velocity is substituted in here so the criterion stays unchanged —
        declared intent, when present, is never overwritten.

        ``perceived_traffic`` empty ⇒ nothing received (before first contact on a lossy link, or no
        traffic) ⇒ fly nominal (ADR 0006 §5). ``adapter`` (default ``None`` = identity) projects
        each exit's command onto the aircraft's airframe channels (Phase 4e /
        :func:`project_to_fixedwing`); a multirotor passes ``None`` (byte-identical path).
        """
        def emit(command: MotionCommand, mem: FleetMemory) -> tuple[MotionCommand, FleetMemory]:
            return (command if adapter is None else adapter(command)), mem

        if resolver is None or not perceived_traffic:
            return emit(nominal, INACTIVE)  # resolution disabled, or nothing received: fly nominal

        new_resopairs: list[tuple[str, DesiredVelocity]] = []
        conflicting: list[AircraftState] = []  # detected pairs still active -> the resolution set
        for other in perceived_traffic:
            detected = detector.detect(state, other, rpz, t_lookahead)
            onset = memory.onset_for(other.id)  # None if this pair was not already active
            if onset is None and not detected:
                continue  # resopairs ∪ confpairs: neither active nor newly detected -> not a pair
            # record the onset velocity on the tick the pair becomes active (inferred intent)
            onset = onset or DesiredVelocity.from_track_speed(other.trk, other.gs)
            other_eff = other if other.desired is not None else replace(other, desired=onset)
            # per-pair recovery: a cleared pair leaves resopairs (aggregate resume when all clear)
            if recovery is not None and recovery.should_resume(state, other_eff, rpz):
                continue
            new_resopairs.append((other.id, onset))
            if detected:
                conflicting.append(other_eff)  # resolution acts on confpairs

        if not new_resopairs:
            return emit(nominal, INACTIVE)  # clear of all conflicts -> resume nominal
        active = FleetMemory(resopairs=tuple(sorted(new_resopairs, key=lambda p: p[0])))
        if conflicting:
            # preferred=None -> the resolver stays closest to the CURRENT velocity (VO shortest way
            # out). Biasing VO toward the *nominal* was tried and destabilised it: greedy
            # nearest-to-nominal cone projection snaps back to the nominal when it briefly seems
            # feasible, re-enters the conflict, oscillates, and lost separation (min_sep 4 m vs rpz
            # 50). Returning to the nominal is the recovery layer's job (CRR), not the resolver's.
            # The `preferred` channel stays in the interface for a future stable (ORCA) resolver.
            return emit(resolver.resolve(state, conflicting, rpz, None), active)
        # active but no current detection: coast on the current velocity
        return emit(MotionCommand.from_track_speed(state.trk, state.gs), active)
