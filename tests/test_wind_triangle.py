"""Wind-triangle helpers (Phase 5a) — the Eq 9 vector sum and its consequences.

Independent-implementation cross-checks (the [[0002-analytical-validation-of-dynamics]] discipline,
not eyeballing): the closed-form ground speed (Eq 4) equals the vector-sum magnitude; the
air/ground maps invert; and applying the Eq 3 crab makes the *ground track* equal the desired
course. Plus the ``NO_WIND`` degeneracy and the unachievable-course case.
"""

from __future__ import annotations

import math

from opencdarr.relative import (
    air_to_ground,
    ground_speed,
    ground_to_air,
    ground_track,
    wind_correction_angle,
)
from opencdarr.wind import NO_WIND, WindField

_PSIS = [0.0, 17.0, 45.0, 123.0, 200.0, 315.0, 359.0]
_WINDS = [
    NO_WIND,
    WindField.from_met(0.0, 5.0),
    WindField.from_met(90.0, 8.0),
    WindField.from_met(270.0, 4.0),
    WindField.from_met(123.0, 6.5),
]


def _air_vector(v_tas: float, psi: float) -> tuple[float, float]:
    r = math.radians(psi)
    return v_tas * math.sin(r), v_tas * math.cos(r)


def test_air_ground_maps_invert() -> None:
    """``air_to_ground`` and ``ground_to_air`` are inverses for every wind."""
    for wind in _WINDS:
        for v in ((3.0, 7.0), (-5.0, 12.0), (0.0, 0.0)):
            back = air_to_ground(ground_to_air(v, wind), wind)
            assert math.isclose(back[0], v[0], abs_tol=1e-9)
            assert math.isclose(back[1], v[1], abs_tol=1e-9)


def test_closed_form_ground_speed_matches_vector_sum() -> None:
    """Eq 4 (cosine rule) equals |airspeed vector + wind| across a heading/wind sweep."""
    v_tas = 17.0
    for wind in _WINDS:
        for psi in _PSIS:
            ge, gn = air_to_ground(_air_vector(v_tas, psi), wind)
            assert math.isclose(ground_speed(v_tas, wind, psi), math.hypot(ge, gn), abs_tol=1e-9)


def test_crab_makes_the_ground_track_equal_the_course() -> None:
    """Applying the Eq 3 crab (ψ = χ + θ_w) yields a ground track equal to the desired course."""
    v_tas = 17.0
    for wind in _WINDS:
        for chi in _PSIS:
            theta_w = wind_correction_angle(v_tas, wind, chi)
            assert theta_w is not None  # all these winds are well below v_tas -> achievable
            psi = chi + theta_w
            ge, gn = air_to_ground(_air_vector(v_tas, psi), wind)
            track = ground_track((ge, gn))
            assert abs(((track - chi + 180.0) % 360.0) - 180.0) < 1e-6


def test_no_wind_degenerates() -> None:
    """At NO_WIND every helper is its no-wind value: identity / airspeed / zero crab."""
    assert air_to_ground((4.0, 9.0), NO_WIND) == (4.0, 9.0)
    assert math.isclose(ground_speed(17.0, NO_WIND, 42.0), 17.0)
    assert wind_correction_angle(17.0, NO_WIND, 42.0) == 0.0


def test_unachievable_course_returns_none() -> None:
    """A cross-course wind exceeding the airspeed makes the course unachievable (returns None)."""
    strong = WindField.from_met(270.0, 20.0)  # 20 m/s from the west, airspeed only 17
    assert wind_correction_angle(17.0, strong, 0.0) is None  # can't hold a due-north track
    # but a course roughly *down* the wind is still achievable (small cross component)
    assert wind_correction_angle(17.0, strong, 90.0) is not None
