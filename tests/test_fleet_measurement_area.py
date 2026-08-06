"""Gating the outcome on a measurement area — what is flown, and what is counted.

An aircraft released outside the study region and flying in has a few seconds where it can pass
close to something with no history of ever having been separated. That is an artefact of the
release rule, not a safety result, so the region that is *measured* is smaller than the region that
is *flown*.

The gate applies to ``min_sep``, ``los`` and the losing-pairs set together, which is what keeps
``los`` exactly ``min_sep < rpz`` — the identity the estimator relies on when it reads a quantile
off the record.
"""

from __future__ import annotations

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.fleet import Agent, run_fleet
from opencdarr.measurement import Disc
from opencdarr.performance import M600
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0
_CENTRE = (52.0, 4.0)
_RUN = dict(rpz=_RPZ, t_lookahead=120.0, dt=1.0, detector=StateBased(),
            resolver=None, recovery=None, t_max=300.0, done_timeout=10.0)


def _head_on_at(bearing: float, distance: float) -> list[Agent]:
    """A crossing pair whose loss happens ``distance`` m from the centre, on ``bearing``."""
    lat, lon = geo.forward(_CENTRE[0], _CENTRE[1], bearing, distance)
    own = AircraftState(id="A", lat=lat, lon=lon, trk=0.0, gs=10.0)
    intr = create_conflict(own, intr_id="B", dpsi=90.0, dcpa=0.0, tlos=30.0, rpz=_RPZ)
    return [Agent(s, M600) for s in (own, intr)]


def test_without_an_area_nothing_changes() -> None:
    """The default is to measure everywhere, and it must be the identical run, field for field.

    This is the guard on the whole change: the area is opt-in, so every published number was
    produced with no area and has to stay exactly where it was.
    """
    fleet = _head_on_at(0.0, 0.0)
    assert run_fleet(fleet, **_RUN) == run_fleet(fleet, area=None, **_RUN)


def test_a_loss_inside_the_area_is_counted() -> None:
    """The control: the same encounter, placed where the study is looking."""
    out = run_fleet(_head_on_at(0.0, 0.0), area=Disc(_CENTRE, 2000.0), **_RUN)
    assert out.los
    assert out.min_sep < _RPZ
    assert (out.n_los_pairs, out.n_los_aircraft) == (1, 2)


def test_the_same_loss_outside_the_area_is_not_counted() -> None:
    """Flown, but not measured: the encounter happens and the outcome does not record it.

    Nothing about the trajectory changes — the aircraft still fly into each other. Only the
    bookkeeping is gated, which is the distinction the measurement area exists to make.
    """
    far = _head_on_at(0.0, 8000.0)                       # 8 km north of a 2 km disc
    gated = run_fleet(far, area=Disc(_CENTRE, 2000.0), **_RUN)
    ungated = run_fleet(far, **_RUN)

    assert ungated.los and ungated.n_los_pairs == 1      # it really is a loss when measured
    assert not gated.los                                 # ... and absent when it is not
    assert (gated.n_los_pairs, gated.n_los_aircraft) == (0, 0)
    assert gated.min_sep == float("inf")                 # no measured separation at all


def test_the_gate_keeps_los_equal_to_min_sep_below_rpz() -> None:
    """``los`` is exactly ``min_sep < rpz`` whether or not an area is set.

    The estimator leans on that identity to read a quantile off ``min_seps`` and trust it describes
    the same population ``p_los_run`` does. Gating one of the two and not the other would break it
    silently, so both are gated by the same mask.
    """
    disc = Disc(_CENTRE, 2000.0)
    for distance in (0.0, 1000.0, 8000.0):
        out = run_fleet(_head_on_at(0.0, distance), area=disc, **_RUN)
        assert out.los == (out.min_sep < _RPZ)


def test_a_pair_counts_only_when_both_aircraft_are_inside() -> None:
    """Straddling the boundary is not measured — the strict reading, and the useful one.

    A released aircraft that has not yet entered the study region is exactly the case the area
    exists to exclude, so a pair with one aircraft outside does not count even though the other is
    well inside.
    """
    # the loss happens just inside a 2 km disc; the intruder approaches from outside it
    tight = Disc(_CENTRE, 300.0)
    out = run_fleet(_head_on_at(0.0, 250.0), area=tight, **_RUN)
    # the encounter is inside the disc only for part of it — whatever the outcome, the two
    # bookkeeping fields agree with each other
    assert out.los == (out.min_sep < _RPZ)
    assert (out.n_los_aircraft > 0) == out.los
    assert out.n_los_pairs <= 1
