"""Functional tests for Probabilistic FTR (uncertainty-aware) recovery.

Mirrors ``test_ftr.py``'s geometries where possible, plus the properties specific to the
probabilistic generalisation: matches ``FTR`` at zero uncertainty for non-radial geometry,
*deliberately diverges* from it for radial trajectories (``vault/derivations/
probabilistic-ftr-recovery.md``), and degrades confidence as declared uncertainty grows.
"""

from __future__ import annotations

import pytest

from opencdarr import geo
from opencdarr.crr import FTR, ProbabilisticFTR
from opencdarr.state import AircraftState, DesiredVelocity

_RPZ = 50.0


def _own(desired: DesiredVelocity | None = None, pos_ci95: float = 0.0) -> AircraftState:
    d = desired if desired is not None else DesiredVelocity.from_track_speed(0.0, 10.0)
    return AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0, desired=d, pos_ci95=pos_ci95
    )


def _ahead(
    dist_m: float, trk: float, gs: float,
    desired: DesiredVelocity | None = None, bearing: float = 0.0, pos_ci95: float = 0.0,
) -> AircraftState:
    """An intruder placed dist_m along ``bearing`` from the ownship (0 = due north)."""
    lat, lon = geo.forward(52.0, 4.0, bearing, dist_m)
    return AircraftState(
        id="INT", lat=lat, lon=lon, trk=trk, gs=gs, desired=desired, pos_ci95=pos_ci95
    )


def test_no_desired_velocity_raises() -> None:
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)  # desired=None (default)
    intr = _ahead(500.0, trk=0.0, gs=15.0)
    with pytest.raises(ValueError):
        ProbabilisticFTR().should_resume(own, intr, _RPZ)


def test_zero_uncertainty_matches_ftr_for_converging_intent_criterion() -> None:
    """Off the radial special case, zero declared uncertainty reduces to FTR's answer."""
    own = _own(DesiredVelocity.from_track_speed(0.0, 10.0))
    resumes = _ahead(1000.0, trk=90.0, gs=5.0, desired=None)  # current velocity diverges
    # shared desired re-aims at own:
    shared = DesiredVelocity.from_track_speed(180.0, 5.0)
    reconverges = _ahead(1000.0, trk=90.0, gs=5.0, desired=shared)
    for intr in (resumes, reconverges):
        prob = ProbabilisticFTR().should_resume(own, intr, _RPZ)
        det = FTR().should_resume(own, intr, _RPZ)
        assert prob == det
    assert ProbabilisticFTR().should_resume(own, resumes, _RPZ) is True
    assert ProbabilisticFTR().should_resume(own, reconverges, _RPZ) is False


def test_radial_trajectory_deliberately_diverges_from_ftr_at_zero_uncertainty() -> None:
    """r and v (anti)parallel -> the unconstrained offset is exactly 0, regardless of current
    distance or direction of travel. FTR's own 'still clears' geometry is exactly this case."""
    own = _own()  # heading north, desired north, 10 m/s
    intr = _ahead(500.0, trk=0.0, gs=15.0)  # 500 m dead ahead, same track, pulling away faster
    assert FTR().should_resume(own, intr, _RPZ) is True  # current separation is what FTR looks at
    assert ProbabilisticFTR().should_resume(own, intr, _RPZ) is False  # radial line has 0 offset


def test_uncertainty_degrades_clearance_confidence() -> None:
    """A geometry that clears comfortably at zero declared uncertainty stops clearing once
    enough position uncertainty is declared -- more spread pulls probability mass under rpz."""
    own_clean, intr_clean = _own(), _ahead(380.0, trk=175.0, gs=10.0, bearing=5.0)
    assert ProbabilisticFTR().should_resume(own_clean, intr_clean, _RPZ) is True

    own_noisy = _own(pos_ci95=5.0)
    intr_noisy = _ahead(380.0, trk=175.0, gs=10.0, bearing=5.0, pos_ci95=5.0)
    assert ProbabilisticFTR().should_resume(own_noisy, intr_noisy, _RPZ) is False


def test_intent_based_second_criterion_blocks_resume_when_shared() -> None:
    own = _own(DesiredVelocity.from_track_speed(0.0, 10.0))
    intr = _ahead(1000.0, trk=90.0, gs=5.0, desired=DesiredVelocity.from_track_speed(180.0, 5.0))
    assert ProbabilisticFTR().should_resume(own, intr, _RPZ) is False


def test_without_shared_intent_only_first_criterion_applies() -> None:
    own = _own(DesiredVelocity.from_track_speed(0.0, 10.0))
    intr = _ahead(1000.0, trk=90.0, gs=5.0, desired=None)
    assert ProbabilisticFTR().should_resume(own, intr, _RPZ) is True


def test_higher_threshold_is_stricter() -> None:
    """Raising prob_threshold can flip a marginal case from resume to not-resume."""
    own, intr = _own(pos_ci95=5.0), _ahead(380.0, trk=175.0, gs=10.0, bearing=5.0, pos_ci95=5.0)
    lenient = ProbabilisticFTR(prob_threshold=0.5).should_resume(own, intr, _RPZ)
    strict = ProbabilisticFTR(prob_threshold=0.999).should_resume(own, intr, _RPZ)
    assert lenient is True
    assert strict is False


def test_ktheta_is_configurable_and_stays_a_valid_probability() -> None:
    own, intr = _own(pos_ci95=10.0), _ahead(380.0, trk=175.0, gs=10.0, bearing=5.0, pos_ci95=10.0)
    coarse = ProbabilisticFTR(ktheta=32).should_resume(own, intr, _RPZ)
    fine = ProbabilisticFTR(ktheta=512).should_resume(own, intr, _RPZ)
    assert coarse == fine  # low resolution changes precision, not the qualitative answer here


def test_returns_a_bool() -> None:
    own, intr = _own(), _ahead(500.0, trk=0.0, gs=15.0, bearing=5.0)
    result = ProbabilisticFTR().should_resume(own, intr, _RPZ)
    assert isinstance(result, bool)
