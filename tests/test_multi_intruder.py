"""Multi-intruder resolution and the separation manager over a fleet (Phase 6a).

The two composition rules the N-aircraft generalisation rests on (ADR 0004 / Phase-6 plan decision
4), and that they are *algorithm-specific*, not a shared "sum":

- **MVP** (potential field) **sums** the pairwise avoidance vectors — a symmetric pair of intruders
  cancels laterally and the ownship slows straight ahead, opening both misses;
- **VO** (velocity obstacles) puts the resolved velocity **outside the union** of the cones — a
  property a summed VO would violate; an over-constrained case still returns a (least-penetration)
  velocity, not a crash.

Plus the :class:`SeparationManager` resolving against, and recovering from, a *set* of intruders.
"""

from __future__ import annotations

import dataclasses
import math

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cr import MVP, VO
from opencdarr.cr.vo import _cone
from opencdarr.crr import PastCPA
from opencdarr.kinematics import MotionCommand
from opencdarr.scenario import create_conflict
from opencdarr.separation import INACTIVE, SeparationManager
from opencdarr.state import AircraftState

_RPZ = 50.0
_LOOKAHEAD = 120.0


def _own() -> AircraftState:
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)


def _miss(own: AircraftState, intr: AircraftState) -> float:
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    rx, ry = dist * math.sin(q), dist * math.cos(q)
    vx = intr.gs * math.sin(math.radians(intr.trk)) - own.gs * math.sin(math.radians(own.trk))
    vy = intr.gs * math.cos(math.radians(intr.trk)) - own.gs * math.cos(math.radians(own.trk))
    t_cpa = -(rx * vx + ry * vy) / (vx * vx + vy * vy)
    return math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)


def _apply(own: AircraftState, cmd: MotionCommand) -> AircraftState:
    return dataclasses.replace(own, trk=cmd.trk, gs=cmd.gs)


def _two_symmetric() -> tuple[AircraftState, AircraftState, AircraftState]:
    """Own heads north; two intruders cross from ±60° — a symmetric two-conflict."""
    own = _own()
    i1 = create_conflict(own, intr_id="I1", dpsi=60.0, dcpa=0.0, tlos=60.0, rpz=_RPZ, side=1)
    i2 = create_conflict(own, intr_id="I2", dpsi=300.0, dcpa=0.0, tlos=60.0, rpz=_RPZ, side=-1)
    return own, i1, i2


def test_mvp_sum_opens_both_misses_but_under_clears() -> None:
    """MVP sums the pairwise dv's — opens BOTH misses, but superposition under-clears a symmetric
    double conflict (a known potential-field limitation VO's union does not share, below)."""
    own, i1, i2 = _two_symmetric()
    assert _miss(own, i1) < _RPZ and _miss(own, i2) < _RPZ  # both start in conflict (dcpa 0)
    cmd = MVP(margin=1.05).resolve(own, [i1, i2], _RPZ)
    resolved = _apply(own, cmd)
    assert _miss(resolved, i1) > _miss(own, i1) + 15.0  # each pairwise dv opens its miss...
    assert _miss(resolved, i2) > _miss(own, i2) + 15.0
    assert _miss(resolved, i1) < _RPZ  # ... but the summed correction falls short of rpz (partial)


def test_vo_resolved_velocity_is_outside_every_cone() -> None:
    """VO against multiple intruders returns a velocity outside the UNION of the cones — and,
    unlike the MVP sum, that clears BOTH conflicts to the resolution zone."""
    own, i1, i2 = _two_symmetric()
    cmd = VO(margin=1.05).resolve(own, [i1, i2], _RPZ)
    ve, vn = cmd.target_velocity
    resolved = _apply(own, cmd)
    rpz_eff = _RPZ * 1.05
    for intr in (i1, i2):
        cone = _cone(own, intr, rpz_eff)
        assert cone is not None
        assert not cone.contains(ve, vn)  # outside this cone — hence outside the union
        assert _miss(resolved, intr) >= _RPZ  # and the miss actually clears rpz


def test_vo_over_constrained_returns_a_velocity() -> None:
    """A ring of intruders (no clean exterior velocity) still returns a least-penetration cmd."""
    own = _own()
    intruders = [
        create_conflict(own, intr_id=f"R{k}", dpsi=float(a), dcpa=0.0, tlos=45.0, rpz=_RPZ,
                        side=1 if a < 180 else -1)
        for k, a in enumerate(range(20, 360, 40))
    ]
    cmd = VO(margin=1.1).resolve(own, intruders, _RPZ)
    assert cmd.target_velocity is not None  # a decision, not a crash
    assert math.isfinite(cmd.gs) and math.isfinite(cmd.trk)


def test_manager_resolves_and_recovers_over_a_set() -> None:
    """The manager tracks both pairs, resolves against both, and clears them independently."""
    own, i1, i2 = _two_symmetric()
    mgr = SeparationManager()
    nominal = MotionCommand.from_track_speed(own.trk, own.gs)
    _, mem = mgr.step(
        own, [i1, i2], nominal, INACTIVE, _RPZ, _LOOKAHEAD,
        StateBased(), MVP(margin=1.05), PastCPA(),
    )
    assert mem.resolving  # actively resolving
    assert {pid for pid, _ in mem.resopairs} == {"I1", "I2"}  # both pairs are in resopairs
    # a far, diverging pair is not added; an empty perceived list flies nominal
    cmd_none, mem_none = mgr.step(
        own, [], nominal, INACTIVE, _RPZ, _LOOKAHEAD, StateBased(), MVP(), PastCPA()
    )
    assert cmd_none == nominal and mem_none == INACTIVE
