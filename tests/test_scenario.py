"""Functional tests for the conflict-encounter generator (`create_conflict`, `sample_pairwise`).

The core check: an encounter built for a requested (dcpa, tlos) reproduces exactly those
values when the generated pair is fed back through the CPA equations. For the sampler, the core
checks are that the all-default draw is unchanged and that pinning one parameter moves only that
parameter.
"""

from __future__ import annotations

import math

import pytest

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.rng import generator, root_seed_sequence
from opencdarr.scenario import create_conflict, sample_pairwise
from opencdarr.state import AircraftState

_RPZ = 50.0
_DET = StateBased()


def _own() -> AircraftState:
    return AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0)


def _cpa_and_tlos(own: AircraftState, intr: AircraftState, rpz: float) -> tuple[float, float]:
    """Recover (dcpa, t_in) for a directed pair — the inverse of create_conflict."""
    qdr, dist = geo.qdrdist(own.lat, own.lon, intr.lat, intr.lon)
    q = math.radians(qdr)
    rx, ry = dist * math.sin(q), dist * math.cos(q)
    vx = intr.gs * math.sin(math.radians(intr.trk)) - own.gs * math.sin(math.radians(own.trk))
    vy = intr.gs * math.cos(math.radians(intr.trk)) - own.gs * math.cos(math.radians(own.trk))
    v2 = vx * vx + vy * vy
    t_cpa = -(rx * vx + ry * vy) / v2
    dcpa = math.hypot(rx + vx * t_cpa, ry + vy * t_cpa)
    t_in = t_cpa - math.sqrt(rpz * rpz - dcpa * dcpa) / math.sqrt(v2)
    return dcpa, t_in


@pytest.mark.parametrize("dpsi", [30.0, 90.0, 135.0, 180.0, 225.0, 315.0])
@pytest.mark.parametrize("dcpa", [0.0, 20.0, 45.0])
def test_generated_encounter_reproduces_dcpa_and_tlos(dpsi: float, dcpa: float) -> None:
    own = _own()
    tlos = 60.0
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=dcpa, tlos=tlos, rpz=_RPZ)
    got_dcpa, got_tlos = _cpa_and_tlos(own, intr, _RPZ)
    assert got_dcpa == pytest.approx(dcpa, abs=1e-6)
    assert got_tlos == pytest.approx(tlos, abs=1e-6)


@pytest.mark.parametrize("dpsi", [30.0, 90.0, 180.0, 300.0])
def test_generated_encounter_is_detected_as_conflict(dpsi: float) -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=dpsi, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    assert _DET.detect(own, intr, _RPZ, t_lookahead=120.0) is True


def test_intruder_track_is_own_plus_dpsi() -> None:
    own = _own()
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)
    assert intr.trk == pytest.approx((own.trk + 90.0) % 360.0)


def test_side_mirrors_position_same_dcpa_tlos() -> None:
    own = _own()
    left = create_conflict(own, intr_id="L", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ, side=1)
    right = create_conflict(own, intr_id="R", dpsi=90.0, dcpa=20.0, tlos=60.0, rpz=_RPZ, side=-1)
    assert (left.lat, left.lon) != (right.lat, right.lon)
    assert _cpa_and_tlos(own, left, _RPZ)[0] == pytest.approx(_cpa_and_tlos(own, right, _RPZ)[0])


def test_zero_relative_velocity_raises() -> None:
    own = _own()
    with pytest.raises(ValueError, match="zero relative velocity"):
        create_conflict(own, intr_id="INT", dpsi=0.0, dcpa=20.0, tlos=60.0, rpz=_RPZ)


# --- sample_pairwise: one seed -> one encounter, per-slot pinned or drawn -------------------

_SAMPLE = dict(speed=10.2889, dcpa_max=50.0, tlos=60.0, rpz=_RPZ)


def _sampled(seed: int = 3, **overrides: object) -> tuple[AircraftState, AircraftState]:
    return sample_pairwise(generator(root_seed_sequence(seed)), **_SAMPLE, **overrides)


def test_all_default_sample_matches_the_pre_override_draws() -> None:
    """**Golden anchor.** Nothing pinned reproduces the pre-override sampler bit-for-bit.

    The reference is the old implementation written out longhand — three draws from the encounter's
    generator, in order, straight into ``create_conflict`` — rather than a constant copied out of
    the new code, which would only prove the code agrees with itself. This is the distribution the
    plain-MC estimator integrates over, so drifting here moves every IPR in the project.
    """
    ref_rng = generator(root_seed_sequence(3))
    dpsi_ref = float(ref_rng.uniform(5.0, 360.0 - 5.0))  # _DPSI_MIN exclusion band
    dcpa_ref = float(ref_rng.uniform(0.0, 50.0))  # U(0, dcpa_max)
    side_ref = 1 if ref_rng.random() < 0.5 else -1
    own_ref = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.2889)
    intr_ref = create_conflict(own_ref, intr_id="INT", dpsi=dpsi_ref, dcpa=dcpa_ref,
                               tlos=60.0, rpz=_RPZ, side=side_ref)

    own, intr = _sampled(seed=3)
    assert (own.lat, own.lon, own.trk, own.gs) == (52.0, 4.0, 0.0, 10.2889)
    assert (intr.lat, intr.lon, intr.trk, intr.gs) == (
        intr_ref.lat, intr_ref.lon, intr_ref.trk, intr_ref.gs
    )


def test_pinning_one_slot_leaves_the_others_untouched() -> None:
    """Pinning the crossing angle must not move the miss distance or the passing side.

    The draws happen in a fixed order regardless of which values survive, so a pinned slot consumes
    and discards its own draw (ADR 0006 §6's config-invariant-stream discipline one level down). If
    a pinned slot instead *skipped* its draw, every later slot would shift, and two conditions of a
    sweep would differ in more than the swept parameter — which is the whole point of a sweep.
    """
    own_drawn, drawn = _sampled()  # the sampler's ownship, not _own() — its speed differs
    dcpa_drawn, _ = _cpa_and_tlos(own_drawn, drawn, _RPZ)

    for dpsi in (2.0, 45.0, 90.0, 180.0):
        own, intr = _sampled(dpsi=dpsi)
        dcpa, tlos = _cpa_and_tlos(own, intr, _RPZ)
        assert intr.trk == pytest.approx(dpsi % 360.0)  # the pin took effect
        assert dcpa == pytest.approx(dcpa_drawn)  # ... and nothing else moved
        assert tlos == pytest.approx(_SAMPLE["tlos"])


def test_pinned_angle_is_not_clamped_to_the_sampling_exclusion_band() -> None:
    """A pinned ``dpsi`` below ``_DPSI_MIN`` is honoured: the published sweeps start at 2°.

    The near-0/360 band is a property of the *built-in draw* (avoiding degenerate closing speeds),
    not a constraint on the geometry generator. Silently clamping it would quietly relabel a
    shallow-crossing study as a 5° one.
    """
    _, intr = _sampled(dpsi=2.0)
    assert intr.trk == pytest.approx(2.0)


def test_a_slot_takes_a_custom_distribution() -> None:
    """A callable slot is drawn per encounter from that encounter's own generator."""
    own, intr = _sampled(dpsi=lambda rng: 90.0 + float(rng.uniform(-1.0, 1.0)))
    assert 89.0 <= intr.trk <= 91.0
    assert intr.trk != pytest.approx(90.0)  # it really drew, rather than taking a midpoint


def test_pinned_side_and_intruder_speed_reach_the_geometry() -> None:
    """``side`` and ``gs_intr`` are honoured — neither was reachable through this seam before."""
    left = _sampled(dpsi=90.0, dcpa=20.0, side=1)[1]
    right = _sampled(dpsi=90.0, dcpa=20.0, side=-1)[1]
    assert (left.lat, left.lon) != (right.lat, right.lon)

    own, faster = _sampled(dpsi=90.0, gs_intr=25.0)
    assert faster.gs == pytest.approx(25.0)
    assert _sampled(dpsi=90.0)[1].gs == pytest.approx(own.gs)  # absent -> matches the ownship
