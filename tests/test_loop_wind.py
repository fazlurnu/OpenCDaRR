"""Mixed-fleet DAA in wind through ``run_encounter`` (Phase 5d).

The Phase-4e mixed multirotor-vs-fixed-wing encounter, now flown in a non-zero steady wind through
the same entry point the IPR sweeps use (``wind=`` threaded since 5a). Two properties:

1. **it still resolves** — the pair clears (min-sep ≥ rpz) with the fixed-wing crabbing its
   avoidance course and the multirotor crabbing its ground velocity, and the result is
   **reproducible** (a deterministic anchor + a seeded noisy anchor);
2. **the wind actually bites** — the wind result differs from the ``NO_WIND`` result, so the
   fixed-wing's wind-limited feasible set is exercised, not bypassed.
"""

from __future__ import annotations

from opencdarr.cd import StateBased
from opencdarr.cns import GnssNavigation
from opencdarr.cr import MVP, VO
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import PastCPA
from opencdarr.dynamics import FixedWing, Multirotor
from opencdarr.loop import EncounterOutcome, run_encounter
from opencdarr.performance import M600, SMALL_FIXEDWING
from opencdarr.rng import generator, root_seed_sequence, spawn
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField

_RPZ = 50.0
_LOOKAHEAD = 120.0
_WIND = WindField.from_met(270.0, 6.0)  # 6 m/s from the west — a crosswind on the north-bound OWN

# Deterministic (noiseless) min_sep anchors in the west wind; a moved bit means the wind coupling
# changed. VO clears by only ~1 m — the wind pushes this geometry close to the rpz limit.
_ANCHOR_WIND_MVP = 54.898044526517275
_ANCHOR_WIND_VO = 51.02865852313175
# Seeded noisy anchor (seed 0, single substream) through the full GPS-noise self-fix path.
_ANCHOR_WIND_NOISY_MVP = 335.00927541159706


def _mixed(
    resolver: ConflictResolver, *, wind: WindField = _WIND, dt: float = 0.5, noisy: bool = False,
) -> EncounterOutcome:
    """Fixed-wing OWN vs multirotor INT, 90° crossing, in ``wind`` — through ``run_encounter``."""
    ci_p, ci_v = (10.0, 1.0) if noisy else (0.0, 0.0)
    own = AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=15.0, yaw=0.0, bank=0.0,
        pos_ci95=ci_p, vel_ci95=ci_v,
    )
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=60.0, rpz=_RPZ, side=1)
    nav = GnssNavigation() if noisy else None
    rng = generator(list(spawn(root_seed_sequence(0), 1))[0]) if noisy else None
    return run_encounter(
        own, intr, perf=M600, rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=dt,
        detector=StateBased(), resolver=resolver, recovery=PastCPA(bouncing_guard=True),
        own_dynamics=FixedWing(), own_perf=SMALL_FIXEDWING,
        intr_dynamics=Multirotor(), intr_perf=M600,
        wind=wind, navigation=nav, rng=rng,
    )


def test_mixed_fleet_in_wind_mvp_resolves() -> None:
    """MVP: the mixed pair clears in a crosswind; deterministic anchor."""
    out = _mixed(MVP(margin=1.1))
    assert out.conflict is True
    assert out.los is False
    assert out.min_sep >= _RPZ
    assert out.min_sep == _ANCHOR_WIND_MVP


def test_mixed_fleet_in_wind_vo_resolves() -> None:
    """VO: the same pair clears, but the wind pushes it close to the rpz limit."""
    out = _mixed(VO(margin=1.1))
    assert out.los is False
    assert out.min_sep >= _RPZ
    assert out.min_sep == _ANCHOR_WIND_VO


def test_wind_changes_the_outcome() -> None:
    """The wind materially bites: the crosswind min_sep differs from the still-air min_sep."""
    windy = _mixed(MVP(margin=1.1)).min_sep
    calm = _mixed(MVP(margin=1.1), wind=NO_WIND).min_sep
    assert abs(windy - calm) > 1.0  # not a no-op — the fixed-wing's feasible set is exercised


def test_mixed_fleet_in_wind_noisy_is_reproducible() -> None:
    """A seeded GPS-noisy encounter in wind reproduces an exact min_sep and re-runs identically."""
    out = _mixed(MVP(margin=1.05), dt=0.2, noisy=True)
    assert out.los is False
    assert out.min_sep == _ANCHOR_WIND_NOISY_MVP
    assert _mixed(MVP(margin=1.05), dt=0.2, noisy=True) == out
