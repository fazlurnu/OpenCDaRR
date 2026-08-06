"""Mixed-fleet DAA through ``run_fleet`` (Phase 4e, ADR 0011 §7 + ADR 0013 §4).

A multirotor and a fixed-wing meet in the *same* conflict and both run detect → resolve → recover
through the normal entry point, each advanced by its own ``kinematics`` / ``perf`` bundle. Two
things are proven here:

1. **it resolves** — the pair clears (min-sep ≥ rpz) even though the two airframes obey different
   physics and the fixed-wing cannot fly the resolver's raw velocity (the runner projects it via
   :func:`~opencdarr.separation.project_to_fixedwing`, wired from the airframe);
2. **it is reproducible** — a seeded noisy run pins an exact ``min_sep`` (a strictly stronger check
   than an aggregate IPR), and a small seeded sweep gives a stable IPR.

The **fixed-wing MVP/VO IPR re-anchor** (deferred to 4e by ADR 0013's Consequences) is the
both-fixed-wing pair pinned at the bottom — the analogue, on the fixed-wing airframe, of the
multirotor ``min_sep`` anchors in ``test_fleet.py``.
"""

from __future__ import annotations

import pytest

from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.cr import MVP, VO
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, FleetOutcome, run_fleet
from opencdarr.kinematics import FixedWing, Multirotor
from opencdarr.performance import M600, SMALL_FIXEDWING
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0
_LOOKAHEAD = 120.0
# A 90° crossing at 15 m/s, tlos 60 s: fast enough for the fixed-wing's bank-limited turn to clear
# and then cleanly recover to its track (the pair terminates on done_timeout, not the MVP "dance").
_GS = 15.0
_DPSI = 90.0
_TLOS = 60.0

# Deterministic (noiseless) min_sep anchors — a moved bit means the mixed-fleet physics changed.
# All six moved **down** for the segment-minimum measurement (see ``test_fleet.py``'s anchor
# block). Compared with pytest.approx(rel=1e-8), not ==: the platform's libm gives trig calls a
# different last bit (see ``test_fleet.py``'s anchor block for why).
_ANCHOR_MIXED_MVP = 95.87550302578735
_ANCHOR_MIXED_VO = 96.11937007400364
# Seeded noisy anchors (seed 0, single substream) through the full GPS-noise self-fix path.
_ANCHOR_MIXED_NOISY_MVP = 116.6778697599543
_ANCHOR_MIXED_NOISY_VO = 118.31313341407984
# Fixed-wing MVP/VO re-anchor (both aircraft fixed-wing), noiseless: two slow-turning fixed-wings
# clear the same crossing by a tighter margin than the mixed/multirotor case.
_ANCHOR_FW_MVP = 53.43304097492288
_ANCHOR_FW_VO = 53.287583586957155


def _mixed(
    resolver: ConflictResolver,
    *,
    dt: float = 0.5,
    noisy: bool = False,
    seed: int = 0,
) -> FleetOutcome:
    """A fixed-wing OWN vs a multirotor INT in a 90° crossing conflict, through ``run_fleet``.

    OWN flies a :class:`~opencdarr.kinematics.FixedWing` (its avoidance velocity is projected to
    course/airspeed by the runner's airframe adapter); INT flies a
    :class:`~opencdarr.kinematics.Multirotor` (velocity flown directly). ``noisy`` turns on the
    seeded GPS self-fix path.
    """
    ci_p, ci_v = (10.0, 1.0) if noisy else (0.0, 0.0)
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=_GS, yaw=0.0, bank=0.0,
        pos_ci95=ci_p, vel_ci95=ci_v,
    )
    intr = create_conflict(own, intr_id="INT", dpsi=_DPSI, dcpa=0.0, tlos=_TLOS, rpz=_RPZ, side=1)
    nav = GnssNavigation() if noisy else None
    rng = generator(list(spawn(root_seed_sequence(seed), 1))[0]) if noisy else None
    return run_fleet(
        [Agent(own, SMALL_FIXEDWING, kinematics=FixedWing()),
         Agent(intr, M600, kinematics=Multirotor())],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=dt,
        detector=StateBased(), resolver=resolver, recovery=PastCPA(bouncing_guard=True),
        navigation=nav, rng=rng,
    )


def test_mixed_fleet_mvp_resolves() -> None:
    """MVP: the mixed pair is a real conflict and clears with no loss of separation."""
    out = _mixed(MVP(margin=1.1))
    assert out.conflict is True
    assert out.los is False
    assert out.min_sep >= _RPZ
    assert out.min_sep == pytest.approx(_ANCHOR_MIXED_MVP, rel=1e-8)  # deterministic anchor


def test_mixed_fleet_vo_resolves() -> None:
    """VO (shortest-way-out): the same mixed pair clears; its own deterministic anchor."""
    out = _mixed(VO(margin=1.1))
    assert out.conflict is True
    assert out.los is False
    assert out.min_sep >= _RPZ
    assert out.min_sep == pytest.approx(_ANCHOR_MIXED_VO, rel=1e-8)


def test_mixed_fleet_noisy_is_reproducible() -> None:
    """A seeded GPS-noisy mixed encounter reproduces an exact min_sep (MVP and VO), clears."""
    mvp = _mixed(MVP(margin=1.05), dt=0.2, noisy=True)
    vo = _mixed(VO(margin=1.05), dt=0.2, noisy=True)
    assert mvp.los is False and vo.los is False
    assert mvp.min_sep == pytest.approx(_ANCHOR_MIXED_NOISY_MVP, rel=1e-8)
    assert vo.min_sep == pytest.approx(_ANCHOR_MIXED_NOISY_VO, rel=1e-8)
    # bit-for-bit on re-run: same seed -> identical outcome (the reproducibility the IPR rests on)
    assert _mixed(MVP(margin=1.05), dt=0.2, noisy=True) == mvp


def test_mixed_fleet_ipr_from_seed() -> None:
    """A small seeded sweep gives a stable IPR: every seeded conflict is resolved (IPR = 1.0)."""
    seqs = list(spawn(root_seed_sequence(0), 16))
    outs = []
    for seq in seqs:
        own = AircraftState(
            id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=_GS, yaw=0.0, bank=0.0,
            pos_ci95=10.0, vel_ci95=1.0,
        )
        intr = create_conflict(
            own, intr_id="INT", dpsi=_DPSI, dcpa=0.0, tlos=_TLOS, rpz=_RPZ, side=1)
        outs.append(run_fleet(
            [Agent(own, SMALL_FIXEDWING, kinematics=FixedWing()),
             Agent(intr, M600, kinematics=Multirotor())],
            rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.2,
            detector=StateBased(), resolver=MVP(margin=1.05),
            recovery=PastCPA(bouncing_guard=True),
            navigation=GnssNavigation(), rng=generator(seq),
        ))
    ipr = 1.0 - sum(o.los for o in outs) / len(outs)
    assert ipr == 1.0


def _both_fixedwing(resolver: ConflictResolver) -> FleetOutcome:
    """The 90° crossing conflict flown by two fixed-wings — the MVP/VO fixed-wing re-anchor."""
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=_GS, yaw=0.0, bank=0.0)
    intr = create_conflict(own, intr_id="INT", dpsi=_DPSI, dcpa=0.0, tlos=_TLOS, rpz=_RPZ, side=1)
    return run_fleet(
        [Agent(own, SMALL_FIXEDWING, kinematics=FixedWing()),
         Agent(intr, SMALL_FIXEDWING, kinematics=FixedWing())],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.5,
        detector=StateBased(), resolver=resolver, recovery=PastCPA(bouncing_guard=True),
    )


def test_fixedwing_mvp_ipr_reanchor() -> None:
    """Fixed-wing MVP re-anchor (deferred to 4e by ADR 0013): clears, exact min_sep pinned."""
    out = _both_fixedwing(MVP(margin=1.1))
    assert out.los is False
    assert out.min_sep == pytest.approx(_ANCHOR_FW_MVP, rel=1e-8)


def test_fixedwing_vo_ipr_reanchor() -> None:
    """Fixed-wing VO re-anchor: clears, exact min_sep pinned."""
    out = _both_fixedwing(VO(margin=1.1))
    assert out.los is False
    assert out.min_sep == pytest.approx(_ANCHOR_FW_VO, rel=1e-8)
