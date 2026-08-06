"""The vehicle-neutral :class:`MotionCommand` (ADR 0011, Phase 4a).

Two properties the layer split rests on:

1. the velocity channel round-trips — ``from_track_speed`` / ``from_velocity`` and the ``gs`` /
   ``trk`` / ``v_east`` / ``v_north`` derived reads are byte-for-byte the old velocity-vector
   command, so every existing call site is unaffected by the supersession (ADR 0008 -> 0011);
2. an under-specified command **fails fast** — reading the velocity channel of a command that
   never set ``target_velocity`` raises, rather than silently returning a zero/garbage vector
   (the missing-channel case of the ADR 0011 feasibility taxonomy).
"""

from __future__ import annotations

import math

import pytest

from opencdarr.kinematics import MotionCommand

# --- Velocity-channel round-trip (behaviour-preserving over the old Command) -----------------


def test_from_track_speed_round_trips_gs_and_trk() -> None:
    """gs/trk read back exactly what from_track_speed was given, across the compass."""
    for hdg in (0.0, 45.0, 90.0, 180.0, 270.0, 359.0):
        for spd in (1.0, 10.0, 18.0):
            cmd = MotionCommand.from_track_speed(hdg, spd)
            assert math.isclose(cmd.gs, spd, abs_tol=1e-9)
            assert math.isclose(cmd.trk, hdg, abs_tol=1e-9)


def test_from_velocity_sets_components_and_derives_polar() -> None:
    """from_velocity stores the vector; v_east/v_north/gs/trk derive from it."""
    cmd = MotionCommand.from_velocity(3.0, 4.0)
    assert cmd.target_velocity == (3.0, 4.0)
    assert cmd.v_east == 3.0
    assert cmd.v_north == 4.0
    assert math.isclose(cmd.gs, 5.0, abs_tol=1e-9)
    assert math.isclose(cmd.trk, math.degrees(math.atan2(3.0, 4.0)) % 360.0, abs_tol=1e-9)


def test_from_track_speed_matches_manual_enu() -> None:
    """The constructor is exactly the old polar->ENU formula (no drift in the supersession)."""
    hdg, spd = 30.0, 12.0
    r = math.radians(hdg)
    cmd = MotionCommand.from_track_speed(hdg, spd)
    assert cmd.target_velocity == (spd * math.sin(r), spd * math.cos(r))


def test_zero_vector_has_zero_gs_and_track_zero() -> None:
    """A zero velocity is a valid setpoint (stop/hover), not a missing channel: gs=0, trk=0."""
    cmd = MotionCommand(target_velocity=(0.0, 0.0))
    assert cmd.gs == 0.0
    assert cmd.trk == 0.0  # a zero vector has no direction -> 0 by convention, not a raise


# --- Fail-fast on an under-specified command (ADR 0011 missing-channel) ----------------------


def test_missing_velocity_channel_raises_on_each_derived_read() -> None:
    """A command with no target_velocity raises when the velocity channel is read."""
    cmd = MotionCommand()  # nothing specified
    assert cmd.target_velocity is None
    for read in (lambda: cmd.v_east, lambda: cmd.v_north, lambda: cmd.gs, lambda: cmd.trk):
        with pytest.raises(ValueError, match="no target_velocity"):
            read()


def test_other_channel_present_still_fails_the_velocity_read() -> None:
    """Specifying a *different* channel does not conjure a velocity: still fails fast."""
    cmd = MotionCommand(target_course=90.0, target_airspeed=15.0)
    with pytest.raises(ValueError, match="no target_velocity"):
        _ = cmd.gs


# --- The other channels are defined but default to None (2D pass ignores them) ---------------


def test_all_channels_default_unspecified() -> None:
    """Every field is optional and unset by default — a bare MotionCommand specifies nothing."""
    cmd = MotionCommand()
    assert cmd.target_velocity is None
    assert cmd.target_position is None
    assert cmd.target_course is None
    assert cmd.target_airspeed_direction is None
    assert cmd.target_airspeed is None
    assert cmd.target_altitude is None
    assert cmd.target_vertical_speed is None
