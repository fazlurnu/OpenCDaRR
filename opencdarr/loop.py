"""The pairwise encounter runner — the environment for one directed encounter.

`run_encounter` advances two aircraft to termination, wiring the CDR layers together each
step: for **both directed pairs** (A→B, B→A) decide a command (detect → resolve, or recover →
resume), then `step_dynamics` each aircraft. It records whether a conflict was predicted and
whether separation was ever lost — the raw material for IPR.

Pure given its inputs; no globals. Each aircraft's nominal navigation is captured from its
initial state and held here (it migrates into the particle state when IPS lands, Step 5).
This is the pairwise precursor to the `advance` / `is_terminal` interface.
"""

from __future__ import annotations

from dataclasses import dataclass

from opencdarr import geo
from opencdarr.cd.base import ConflictDetector
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
    """One aircraft's command and its new resolving flag (directed: ac vs its perceived other)."""
    if resolver is None:
        return nominal, False  # resolution disabled: always fly nominal
    if resolving:
        if recovery is not None and recovery.should_resume(ac, other, rpz):
            return nominal, False  # clear to resume
        return resolver.resolve(ac, other, rpz), True  # keep resolving
    if detector.detect(ac, other, rpz, t_lookahead):
        return resolver.resolve(ac, other, rpz), True  # start resolving
    return nominal, False


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
    t_max: float = 600.0,
    done_timeout: float = 10.0,
) -> EncounterOutcome:
    """Run one pairwise encounter to termination and report its outcome.

    With ``resolver=None`` the aircraft fly their nominal paths (a baseline that *should* lose
    separation). With a resolver (and ideally a recovery criterion), they maneuver to clear.
    Terminates once the pair has been diverging and separated for ``done_timeout`` seconds, or
    at ``t_max``.
    """
    nom_own = Command(hdg=own.trk, spd=own.gs)
    nom_intr = Command(hdg=intr.trk, spd=intr.gs)
    resolving_own = resolving_intr = False

    conflict = los = False
    min_sep = float("inf")
    done_timer = 0.0
    t = 0.0

    while t < t_max:
        _, sep = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
        min_sep = min(min_sep, sep)
        if sep < rpz:
            los = True
        if detector.detect(own, intr, rpz, t_lookahead) or detector.detect(
            intr, own, rpz, t_lookahead
        ):
            conflict = True

        cmd_own, resolving_own = _decide(
            own, intr, nom_own, resolving_own, rpz, t_lookahead, detector, resolver, recovery
        )
        cmd_intr, resolving_intr = _decide(
            intr, own, nom_intr, resolving_intr, rpz, t_lookahead, detector, resolver, recovery
        )
        own = step_dynamics(own, cmd_own, perf, dt)
        intr = step_dynamics(intr, cmd_intr, perf, dt)
        t += dt

        rel = relative_enu(own, intr)
        diverging = rel.rx * rel.vx + rel.ry * rel.vy > 0.0  # past CPA
        clear = diverging and rel.dist >= rpz and not resolving_own and not resolving_intr
        done_timer = done_timer + dt if clear else 0.0
        if done_timer >= done_timeout:
            break

    return EncounterOutcome(conflict=conflict, los=los, min_sep=min_sep)
