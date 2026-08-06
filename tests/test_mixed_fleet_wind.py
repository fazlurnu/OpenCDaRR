"""Mixed-fleet DAA in wind through ``run_fleet`` (Phase 5d).

The Phase-4e mixed multirotor-vs-fixed-wing encounter, now flown in a non-zero steady wind through
the same entry point the estimator uses (``wind=`` threaded since 5a). Two properties:

1. **it still resolves** — the pair clears (min-sep ≥ rpz) with the fixed-wing crabbing its
   avoidance course and the multirotor crabbing its ground velocity, and the result is
   **reproducible** (a deterministic anchor + a seeded noisy anchor);
2. **the wind actually bites** — the wind result differs from the ``NO_WIND`` result, so the
   fixed-wing's wind-limited feasible set is exercised, not bypassed.
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
from opencdarr.wind import NO_WIND, WindField

_RPZ = 50.0
_LOOKAHEAD = 120.0
_WIND = WindField.from_met(270.0, 6.0)  # 6 m/s from the west — a crosswind on the north-bound OWN

# Deterministic (noiseless) min_sep anchors in the west wind; a moved bit means the wind coupling
# changed. VO clears by only ~0.9 m — the wind pushes this geometry close to the rpz limit, and the
# segment-minimum measurement (see ``test_fleet.py``'s anchor block) took another 0.17 m off it:
# this is the case where reading separation only at step endpoints most flatters the result.
# Compared with pytest.approx(rel=1e-8), not ==: the platform's libm gives trig calls a different
# last bit (see ``test_fleet.py``'s anchor block for why).
_ANCHOR_WIND_MVP = 54.839298823969486
_ANCHOR_WIND_VO = 50.85881790533006
# Seeded noisy anchor (seed 0, single substream) through the full GPS-noise self-fix path.
_ANCHOR_WIND_NOISY_MVP = 335.00445769084274


def _mixed(
    resolver: ConflictResolver, *, wind: WindField = _WIND, dt: float = 0.5, noisy: bool = False,
) -> FleetOutcome:
    """Fixed-wing OWN vs multirotor INT, 90° crossing, in ``wind`` — through ``run_fleet``."""
    ci_p, ci_v = (10.0, 1.0) if noisy else (0.0, 0.0)
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=15.0, yaw=0.0, bank=0.0,
        pos_ci95=ci_p, vel_ci95=ci_v,
    )
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=60.0, rpz=_RPZ, side=1)
    nav = GnssNavigation() if noisy else None
    rng = generator(list(spawn(root_seed_sequence(0), 1))[0]) if noisy else None
    return run_fleet(
        [Agent(own, SMALL_FIXEDWING, kinematics=FixedWing()),
         Agent(intr, M600, kinematics=Multirotor())],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=dt,
        detector=StateBased(), resolver=resolver, recovery=PastCPA(bouncing_guard=True),
        wind=wind, navigation=nav, rng=rng,
    )


def test_mixed_fleet_in_wind_mvp_resolves() -> None:
    """MVP: the mixed pair clears in a crosswind; deterministic anchor."""
    out = _mixed(MVP(margin=1.1))
    assert out.conflict is True
    assert out.los is False
    assert out.min_sep >= _RPZ
    assert out.min_sep == pytest.approx(_ANCHOR_WIND_MVP, rel=1e-8)


def test_mixed_fleet_in_wind_vo_resolves() -> None:
    """VO: the same pair clears, but the wind pushes it close to the rpz limit."""
    out = _mixed(VO(margin=1.1))
    assert out.los is False
    assert out.min_sep >= _RPZ
    assert out.min_sep == pytest.approx(_ANCHOR_WIND_VO, rel=1e-8)


def test_wind_changes_the_outcome() -> None:
    """The wind materially bites: the crosswind min_sep differs from the still-air min_sep."""
    windy = _mixed(MVP(margin=1.1)).min_sep
    calm = _mixed(MVP(margin=1.1), wind=NO_WIND).min_sep
    assert abs(windy - calm) > 1.0  # not a no-op — the fixed-wing's feasible set is exercised


def test_mixed_fleet_in_wind_noisy_is_reproducible() -> None:
    """A seeded GPS-noisy encounter in wind reproduces an exact min_sep and re-runs identically."""
    out = _mixed(MVP(margin=1.05), dt=0.2, noisy=True)
    assert out.los is False
    assert out.min_sep == pytest.approx(_ANCHOR_WIND_NOISY_MVP, rel=1e-8)
    assert _mixed(MVP(margin=1.05), dt=0.2, noisy=True) == out
