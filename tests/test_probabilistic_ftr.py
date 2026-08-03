"""Functional tests for Probabilistic FTR (uncertainty-aware) recovery.

Mirrors ``test_ftr.py``'s geometries where possible, plus the properties specific to the
probabilistic generalisation: matches ``FTR`` at zero uncertainty for non-radial geometry,
*deliberately diverges* from it for radial trajectories (``vault/derivations/
probabilistic-ftr-recovery.md``), and degrades confidence as declared uncertainty grows.
"""

from __future__ import annotations

import numpy as np
import pytest

from opencdarr import geo
from opencdarr.crr import FTR, ProbabilisticFTR
from opencdarr.crr.probabilistic_ftr import _iso_cov, _p_offset_gt
from opencdarr.relative import relative_enu, velocity_enu
from opencdarr.state import AircraftState, DesiredVelocity

_RPZ = 50.0


def _probability(
    criterion: ProbabilisticFTR, own: AircraftState, intr: AircraftState, rpz: float,
) -> float:
    """Criterion 1's probability — the quantity ``should_resume`` compares to ``prob_threshold``.

    The grid tests assert on this rather than on the returned bool, because a bool only exposes a
    quadrature disagreement when the two values happen to straddle the threshold.
    """
    rel = relative_enu(own, intr)
    vi_e, vi_n = velocity_enu(intr)
    assert own.desired is not None
    return _p_offset_gt(
        rpz,
        np.array([rel.rx, rel.ry]),
        _iso_cov(own.pos_ci95) + _iso_cov(intr.pos_ci95),
        np.array([vi_e - own.desired.v_east, vi_n - own.desired.v_north]),
        criterion._sigma_v(own, intr),
        criterion.ktheta,
        criterion.grid,
    )


def _own(
    desired: DesiredVelocity | None = None, pos_ci95: float = 0.0, vel_ci95: float = 0.0,
) -> AircraftState:
    d = desired if desired is not None else DesiredVelocity.from_track_speed(0.0, 10.0)
    return AircraftState(
        id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0, desired=d,
        pos_ci95=pos_ci95, vel_ci95=vel_ci95,
    )


def _ahead(
    dist_m: float, trk: float, gs: float,
    desired: DesiredVelocity | None = None, bearing: float = 0.0, pos_ci95: float = 0.0,
    vel_ci95: float = 0.0,
) -> AircraftState:
    """An intruder placed dist_m along ``bearing`` from the ownship (0 = due north)."""
    lat, lon = geo.forward(52.0, 4.0, bearing, dist_m)
    return AircraftState(
        id="INT", lat=lat, lon=lon, trk=trk, gs=gs, desired=desired,
        pos_ci95=pos_ci95, vel_ci95=vel_ci95,
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
    enough position uncertainty is declared -- more spread pulls probability mass under rpz.

    ``bearing=10.0`` puts the closest-approach offset at 82.3 m, comfortably outside ``_RPZ``. It
    used to be 5.0, where the offset is 49.6 m and therefore *inside* the protected zone, so the
    "clears comfortably" assertion below was only ever satisfied by the uniform grid's
    undersampling error (``vault/observations/probftr-angular-grid.md``).
    """
    own_clean, intr_clean = _own(), _ahead(380.0, trk=175.0, gs=10.0, bearing=10.0)
    assert ProbabilisticFTR().should_resume(own_clean, intr_clean, _RPZ) is True

    own_noisy = _own(pos_ci95=25.0)
    intr_noisy = _ahead(380.0, trk=175.0, gs=10.0, bearing=10.0, pos_ci95=25.0)
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
    """Raising prob_threshold can flip a marginal case from resume to not-resume.

    P is 0.987 here, which is the point: it sits between the two thresholds. Same geometry
    correction as ``test_uncertainty_degrades_clearance_confidence``.
    """
    own, intr = _own(pos_ci95=25.0), _ahead(380.0, trk=175.0, gs=10.0, bearing=10.0, pos_ci95=25.0)
    lenient = ProbabilisticFTR(prob_threshold=0.5).should_resume(own, intr, _RPZ)
    strict = ProbabilisticFTR(prob_threshold=0.999).should_resume(own, intr, _RPZ)
    assert lenient is True
    assert strict is False


def test_ktheta_is_configurable_and_stays_a_valid_probability() -> None:
    own, intr = _own(pos_ci95=10.0), _ahead(380.0, trk=175.0, gs=10.0, bearing=5.0, pos_ci95=10.0)
    coarse = ProbabilisticFTR(ktheta=32).should_resume(own, intr, _RPZ)
    fine = ProbabilisticFTR(ktheta=512).should_resume(own, intr, _RPZ)
    assert coarse == fine  # low resolution changes precision, not the qualitative answer here


def test_centred_grid_is_the_default() -> None:
    assert ProbabilisticFTR().grid == "centred"


def test_unknown_grid_raises() -> None:
    with pytest.raises(ValueError, match="grid must be"):
        ProbabilisticFTR(grid="polar")  # type: ignore[arg-type]


def test_both_grids_agree_once_the_uniform_one_is_resolved() -> None:
    """The grids are two quadratures of one integral, so a fine enough uniform grid matches."""
    own, intr = _own(pos_ci95=6.0, vel_ci95=1.0), _ahead(
        320.0, trk=185.0, gs=17.0, bearing=4.0, pos_ci95=6.0, vel_ci95=1.0,
    )
    args = (own, intr, _RPZ)
    fine_uniform = _probability(ProbabilisticFTR(ktheta=200_000, grid="uniform"), *args)
    centred = _probability(ProbabilisticFTR(ktheta=64, grid="centred"), *args)
    assert centred == pytest.approx(fine_uniform, abs=1e-6)


def test_centred_grid_is_self_consistent_under_refinement() -> None:
    """Doubling ktheta must not move the probability — the property ktheta is *for*.

    Asserted on the probability rather than on ``should_resume``'s bool, because a bool hides
    disagreement unless the two values happen to straddle ``prob_threshold``
    (``vault/observations/probftr-angular-grid.md``). The uniform grid fails this on this geometry,
    which is why ``"centred"`` is the default.
    """
    own, intr = _own(pos_ci95=6.0, vel_ci95=1.0), _ahead(
        320.0, trk=185.0, gs=17.0, bearing=4.0, pos_ci95=6.0, vel_ci95=1.0,
    )
    for ktheta in (32, 64, 128):
        coarse = _probability(ProbabilisticFTR(ktheta=ktheta), own, intr, _RPZ)
        fine = _probability(ProbabilisticFTR(ktheta=2 * ktheta), own, intr, _RPZ)
        assert coarse == pytest.approx(fine, abs=1e-6), f"ktheta={ktheta} disagrees with 2x"


def test_grids_coincide_when_the_density_fills_the_circle() -> None:
    """At low SNR the centred window opens to 2*pi, so the two rules must be the same rule."""
    own = _own(pos_ci95=10.0, vel_ci95=40.0)          # velocity uncertainty swamps the speed
    intr = _ahead(300.0, trk=180.0, gs=2.0, bearing=0.0, pos_ci95=10.0, vel_ci95=40.0)
    centred = _probability(ProbabilisticFTR(ktheta=128, grid="centred"), own, intr, _RPZ)
    uniform = _probability(ProbabilisticFTR(ktheta=128, grid="uniform"), own, intr, _RPZ)
    assert centred == uniform


def test_default_threshold_is_the_paper_default() -> None:
    assert ProbabilisticFTR().prob_threshold == 0.999


def test_own_velocity_uncertainty_is_included_by_default_and_switchable() -> None:
    """``"both"`` sums Sigma_Vo + Sigma_Vi, so declaring ownship velocity error widens the
    velocity-direction spread and lowers confidence; ``"intruder"`` ignores it entirely."""
    own = _own(pos_ci95=2.0, vel_ci95=4.0)
    intr = _ahead(600.0, trk=175.0, gs=10.0, bearing=5.0, pos_ci95=2.0, vel_ci95=0.0)

    assert ProbabilisticFTR(velocity_uncertainty="both").should_resume(own, intr, _RPZ) is False
    # the ownship's own vel_ci95 is the only difference between the two:
    assert ProbabilisticFTR(velocity_uncertainty="intruder").should_resume(own, intr, _RPZ) is True


def test_intruder_only_ignores_own_vel_ci95_entirely() -> None:
    """Under ``"intruder"`` the ownship's declared velocity accuracy cannot change the answer."""
    intr = _ahead(420.0, trk=175.0, gs=10.0, bearing=5.0, pos_ci95=5.0, vel_ci95=1.0)
    crit = ProbabilisticFTR(velocity_uncertainty="intruder")
    quiet = crit.should_resume(_own(pos_ci95=5.0, vel_ci95=0.0), intr, _RPZ)
    noisy = crit.should_resume(_own(pos_ci95=5.0, vel_ci95=8.0), intr, _RPZ)
    assert quiet == noisy


def test_unknown_velocity_uncertainty_mode_raises() -> None:
    with pytest.raises(ValueError, match="velocity_uncertainty"):
        ProbabilisticFTR(velocity_uncertainty="ownship")  # type: ignore[arg-type]


def test_returns_a_bool() -> None:
    own, intr = _own(), _ahead(500.0, trk=0.0, gs=15.0, bearing=5.0)
    result = ProbabilisticFTR().should_resume(own, intr, _RPZ)
    assert isinstance(result, bool)
