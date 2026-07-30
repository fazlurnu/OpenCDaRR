"""Separation is measured over each whole step, not at its endpoints (``segment_min_range``).

The reference here is **analytic, not simulated**. ``scenario.create_conflict`` constructs a
straight-line encounter whose true minimum separation *is* the requested ``dcpa``, so for a
non-manoeuvring pair the exact answer is known in closed form and the run can be graded against a
number that came from outside the code (``design-philosophy.md`` #15). Grading ``los`` or
``min_sep`` against another read of the same trajectory — which is what every pre-existing test
did — cannot detect a measurement that systematically misses part of the trajectory.

The defect: ``FleetEnv.advance`` sampled separation once per ``dt`` and set ``los = cur < rpz``, so
a pass that dipped inside a threshold and back out within one step left no sampled point inside.
The error is one-sided (it can only report *more* separation than there was) and it worsens as the
threshold tightens, which is why it mattered far more to the IPS shells than to ``P(LoS)`` at
``rpz``. See ``vault/observations/segment-min-separation.md``.
"""

from __future__ import annotations

import math

from opencdarr.cd import StateBased
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, run_fleet
from opencdarr.loop import run_encounter
from opencdarr.performance import M600
from opencdarr.relative import Relative, segment_min_range
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RPZ = 50.0
_LOOKAHEAD = 120.0
_SPEED = 10.2889
_TLOS = 60.0


def _straight(dpsi: float, dcpa: float, dt: float) -> tuple[float, bool]:
    """One non-manoeuvring encounter: measured (min_sep, los). True min separation is ``dcpa``."""
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=_SPEED)
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=dcpa, tlos=_TLOS, rpz=_RPZ)
    out = run_fleet(
        [Agent(own, M600), Agent(intr, M600)],
        rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=dt, detector=StateBased(), t_max=600.0,
    )
    return out.min_sep, out.los


# --- the algebra itself, against hand-computed geometry ---------------------------------------


def test_segment_minimum_of_a_pass_that_straddles_the_step() -> None:
    """A pair 3 m apart across-track, passing from 40 m behind to 40 m ahead in one step.

    Closest approach is the perpendicular distance, 3 m, and it falls at the segment's midpoint —
    while *both* endpoints read hypot(40, 3) = 40.11 m. Endpoint sampling would report 40.11 m.
    """
    r0 = Relative(rx=-40.0, ry=3.0, vx=0.0, vy=0.0)
    r1 = Relative(rx=40.0, ry=3.0, vx=0.0, vy=0.0)
    assert math.isclose(segment_min_range(r0, r1), 3.0, rel_tol=1e-12)
    assert math.isclose(r0.dist, math.hypot(40.0, 3.0), rel_tol=1e-12)


def test_segment_minimum_clamps_to_the_endpoints() -> None:
    """Closest approach outside the step returns the nearer *endpoint*, never an extrapolation.

    Both aircraft are closing throughout (or separating throughout), so the minimum over the step
    is at an end. Extrapolating past it would report a range at a point never occupied.
    """
    closing = (Relative(rx=100.0, ry=0.0, vx=0.0, vy=0.0),
               Relative(rx=60.0, ry=0.0, vx=0.0, vy=0.0))
    assert math.isclose(segment_min_range(*closing), 60.0, rel_tol=1e-12)
    opening = (Relative(rx=60.0, ry=0.0, vx=0.0, vy=0.0),
               Relative(rx=100.0, ry=0.0, vx=0.0, vy=0.0))
    assert math.isclose(segment_min_range(*opening), 60.0, rel_tol=1e-12)


def test_segment_minimum_ignores_velocity() -> None:
    """It interpolates positions; the velocities on ``Relative`` must not enter the answer.

    This is the guard against re-implementing it as a velocity extrapolation, which was measured
    inventing losses of separation on turning aircraft.
    """
    a = Relative(rx=-40.0, ry=3.0, vx=0.0, vy=0.0)
    b = Relative(rx=40.0, ry=3.0, vx=0.0, vy=0.0)
    wild = (Relative(rx=-40.0, ry=3.0, vx=999.0, vy=-999.0),
            Relative(rx=40.0, ry=3.0, vx=-500.0, vy=750.0))
    assert segment_min_range(a, b) == segment_min_range(*wild)


# --- against the analytic geometry, through the whole runner -----------------------------------


def test_min_sep_recovers_the_constructed_dcpa() -> None:
    """On a straight-line encounter the measured min_sep must equal the *constructed* dcpa.

    Held to 5 cm: the residual is the geodesy/integration gap between the nominal geometry and the
    one actually flown, measured at ~3 cm and independent of dt. Endpoint sampling misses by up to
    2.1 m at dt=1.0 on these same geometries, so the tolerance discriminates.
    """
    for dt in (1.0, 0.5, 0.2):
        for dpsi in (45.0, 90.0, 180.0):
            for dcpa in (0.0, 12.5, 30.0, 47.0):
                min_sep, _ = _straight(dpsi, dcpa, dt)
                assert abs(min_sep - dcpa) < 0.05, f"{dt=} {dpsi=} {dcpa=} gave {min_sep}"


def test_endpoint_sampling_would_have_missed_these() -> None:
    """The refinement is doing real work — and it is needed at *every* dt, not just a coarse one.

    Reconstructs from the same recorded run what endpoint sampling would have reported, and needs
    it to fall visibly short. Without this, the accuracy test above could pass on a build where the
    refinement had been removed and ``dt`` merely happened to be fine.

    The worst case is a head-on ``dcpa = 0`` — the whole closing speed, and a true minimum of zero,
    so any sample offset shows up undiluted. Measured shortfall: 8.84 m at dt=1.0, 1.45 m at
    dt=0.5, 0.61 m at dt=0.2. Note the last: shrinking dt does not make endpoint sampling correct.
    """
    from opencdarr.relative import relative_enu

    for dt in (1.0, 0.5, 0.2):
        worst = 0.0
        own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=_SPEED)
        intr = create_conflict(own, intr_id="INT", dpsi=180.0, dcpa=0.0, tlos=_TLOS, rpz=_RPZ)
        out = run_fleet(
            [Agent(own, M600), Agent(intr, M600)],
            rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=dt, detector=StateBased(),
            t_max=600.0, record=True,
        )
        assert out.frames is not None
        endpoint_only = min(relative_enu(f.states[0], f.states[1]).dist for f in out.frames)
        worst = max(worst, endpoint_only - out.min_sep)
        assert out.min_sep < 0.05, f"refined answer wrong at {dt=}: {out.min_sep}"
        assert worst > 0.3, f"endpoint sampling looked fine at {dt=} ({worst=}), test is toothless"


def test_a_true_loss_of_separation_is_never_missed() -> None:
    """Every constructed dcpa below rpz is a real LoS, including the near-tangential band.

    ``dcpa = 49.9`` at a head-on closure spends ~0.3 s inside rpz — under one step at dt=1.0, so
    endpoint sampling missed the whole band (measured: the *entire* band, at head-on, because
    ``create_conflict`` puts the entry instant at exactly ``t = tlos`` and tlos/dt is an integer).
    """
    for dt in (1.0, 0.5, 0.2):
        for dcpa in (49.0, 49.5, 49.9, 49.99):
            _, los = _straight(180.0, dcpa, dt)
            assert los is True, f"missed a true LoS at {dcpa=} {dt=}"


def test_separation_above_rpz_is_not_reported_as_a_loss() -> None:
    """The other side of the boundary: the refinement must not *invent* losses either.

    A velocity-extrapolating implementation fails this on manoeuvring runs; interpolation cannot,
    because every range it reports lies between two states the run actually produced.
    """
    for dt in (1.0, 0.5, 0.2):
        own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=_SPEED)
        # dcpa above rpz: a conflict is not constructible, so place a clean miss by hand
        intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=_TLOS, rpz=_RPZ)
        out = run_fleet(
            [Agent(own, M600), Agent(intr, M600)],
            rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=dt, detector=StateBased(),
            resolver=MVP(margin=1.05), recovery=PastCPA(), t_max=600.0,
        )
        assert out.los is False
        assert out.min_sep >= _RPZ


# --- the two runners must measure identically ---------------------------------------------------


def test_fleet_and_loop_agree_on_the_refined_minimum() -> None:
    """The n=2 reduction still holds bit-for-bit — both runners share ``segment_min_range``.

    ``loop`` keeps its own separation loop, so the refinement had to land in both; this is what
    would catch it landing in only one.
    """
    for dpsi in (45.0, 90.0, 180.0):
        own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=_SPEED)
        intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=10.0, tlos=_TLOS, rpz=_RPZ)
        kw = dict(rpz=_RPZ, t_lookahead=_LOOKAHEAD, dt=0.5, detector=StateBased(),
                  resolver=MVP(margin=1.05), recovery=PastCPA())
        fleet_out = run_fleet([Agent(own, M600), Agent(intr, M600)], **kw)  # type: ignore[arg-type]
        loop_out = run_encounter(own, intr, perf=M600, **kw)  # type: ignore[arg-type]
        assert fleet_out.min_sep == loop_out.min_sep
        assert fleet_out.los == loop_out.los
        assert fleet_out.conflict == loop_out.conflict
