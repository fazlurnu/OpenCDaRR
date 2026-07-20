"""The pairwise encounter runner — the environment for one directed encounter.

`run_encounter` advances two aircraft to termination. The CDR layers run on a **broadcast
cadence** (``broadcast_interval``, the ADS-L/ASAS decision rate — 1 Hz in the reference), not
every integration step: at each broadcast tick each aircraft takes a fresh noisy self-measurement
and decides (detect → resolve, or recover → resume) for **both directed pairs** (A→B, B→A) on its
*perceived* view — its own broadcast against the other's broadcast; the resulting command is then
**held** while `step_dynamics` integrates at ``dt`` until the next tick. Deciding every step
instead would re-draw independent noise 1/``dt``× per second and average it away — unphysically
robust (the surveillance state proper lands in 3b). Truth is used only to score the encounter
(conflict predicted? separation lost? minimum separation?) — the raw material for IPR.

Pure given its inputs; no globals. Each aircraft's nominal navigation is captured from its
initial state and held here (it migrates into the particle state when IPS lands, Step 5).
This is the pairwise precursor to the `advance` / `is_terminal` interface.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from opencdarr import geo
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import NavigationModel
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.dynamics import Command, step_dynamics
from opencdarr.kinematics import relative_enu
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


@dataclass(frozen=True)
class EncounterOutcome:
    """What one encounter produced."""

    conflict: bool  # was a conflict predicted at any step?
    los: bool  # was separation ever lost?
    min_sep: float  # minimum separation reached [m]


def _decide(
    ac: AircraftState,
    other: AircraftState,
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
    """
    if resolver is None:
        return nominal, False  # resolution disabled: always fly nominal

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
    t_max: float = 600.0,
    done_timeout: float = 10.0,
    broadcast_interval: float = 1.0,
) -> EncounterOutcome:
    """Run one pairwise encounter to termination and report its outcome.

    With ``resolver=None`` the aircraft fly their nominal paths (a baseline that *should* lose
    separation). With a resolver (and ideally a recovery criterion), they maneuver to clear.

    The CDR layers run every ``broadcast_interval`` seconds (the ADS-L/ASAS decision rate), not
    every ``dt``: at each tick each aircraft takes a fresh noisy self-measurement and **decides**
    on its *perceived* view — its own broadcast against the other's broadcast — then the command
    is **held** until the next tick. Without a ``navigation`` model (and ``rng``) the perceived
    view is the true state (Phase 2 behaviour). The outcome (conflict, LoS, separation) is always
    measured on the **true** states, every step. Terminates once the pair has been diverging and
    separated for ``done_timeout`` seconds, or at ``t_max``.
    """
    nom_own = Command(hdg=own.trk, spd=own.gs)
    nom_intr = Command(hdg=intr.trk, spd=intr.gs)
    resolving_own = resolving_intr = False
    cmd_own, cmd_intr = nom_own, nom_intr

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
            # each aircraft's fresh (noisy) self-broadcast; both endpoints carry noise
            if navigation is not None and rng is not None:
                bcast_own = navigation.measure(own, t, rng).state
                bcast_intr = navigation.measure(intr, t, rng).state
            else:
                bcast_own, bcast_intr = own, intr

            cmd_own, resolving_own = _decide(
                bcast_own, bcast_intr, nom_own, resolving_own,
                rpz, t_lookahead, detector, resolver, recovery,
            )
            cmd_intr, resolving_intr = _decide(
                bcast_intr, bcast_own, nom_intr, resolving_intr,
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
