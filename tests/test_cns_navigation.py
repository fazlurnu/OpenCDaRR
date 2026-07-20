"""Functional tests for GPS navigation noise."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from opencdarr import geo
from opencdarr.cns import GpsNavigation
from opencdarr.kinematics import velocity_enu
from opencdarr.state import AircraftState

_TRUE = AircraftState(id="A", lat=52.0, lon=4.0, trk=30.0, gs=10.0)
_SIGMA_PER_CI95 = 1.0 / math.sqrt(5.991464547)


def _pos_offset_enu(true: AircraftState, meas: AircraftState) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(true.lat, true.lon, meas.lat, meas.lon)
    q = math.radians(qdr)
    return dist * math.sin(q), dist * math.cos(q)


def test_zero_noise_measures_true_state() -> None:
    """Default AircraftState declares perfect accuracy (pos_ci95 = vel_ci95 = 0)."""
    nav = GpsNavigation()
    msg = nav.measure(_TRUE, t=5.0, rng=np.random.default_rng(0))
    assert msg.source == "A"
    assert msg.t_meas == 5.0
    assert msg.state.lat == pytest.approx(_TRUE.lat)
    assert msg.state.lon == pytest.approx(_TRUE.lon)
    assert msg.state.trk == pytest.approx(_TRUE.trk)
    assert msg.state.gs == pytest.approx(_TRUE.gs)


def test_broadcast_declares_the_source_accuracy() -> None:
    """The measured (broadcast) state carries the source's own declared ci95."""
    true = dataclasses.replace(_TRUE, pos_ci95=20.0, vel_ci95=2.0)
    msg = GpsNavigation().measure(true, t=0.0, rng=np.random.default_rng(0))
    assert msg.state.pos_ci95 == 20.0
    assert msg.state.vel_ci95 == 2.0


def test_position_noise_is_zero_mean_and_ci95_calibrated() -> None:
    ci95 = 20.0
    true = dataclasses.replace(_TRUE, pos_ci95=ci95, vel_ci95=0.0)
    nav = GpsNavigation()
    rng = np.random.default_rng(1)
    offsets = np.array(
        [_pos_offset_enu(true, nav.measure(true, 0.0, rng).state) for _ in range(8000)]
    )
    assert abs(offsets[:, 0].mean()) < 1.0  # zero-mean per axis
    assert abs(offsets[:, 1].mean()) < 1.0
    assert abs(offsets[:, 0].std() - ci95 * _SIGMA_PER_CI95) < 0.5  # per-axis sigma
    radial = np.hypot(offsets[:, 0], offsets[:, 1])
    assert abs(float(np.quantile(radial, 0.95)) - ci95) < 1.5  # 95% radial CI


def test_velocity_noise_is_zero_mean_and_ci95_calibrated() -> None:
    vel_ci95 = 2.0
    true = dataclasses.replace(_TRUE, pos_ci95=0.0, vel_ci95=vel_ci95)
    nav = GpsNavigation()
    rng = np.random.default_rng(2)
    ve = np.array([velocity_enu(nav.measure(true, 0.0, rng).state) for _ in range(8000)])
    true_e, true_n = velocity_enu(true)
    assert abs(ve[:, 0].std() - vel_ci95 * _SIGMA_PER_CI95) < 0.2  # per-axis sigma
    assert abs(ve[:, 0].mean() - true_e) < 0.2
    assert abs(ve[:, 1].mean() - true_n) < 0.2
    err_e, err_n = ve[:, 0] - true_e, ve[:, 1] - true_n
    radial = np.hypot(err_e, err_n)
    assert abs(float(np.quantile(radial, 0.95)) - vel_ci95) < 0.3  # 95% radial CI


def test_reproducible_per_seed() -> None:
    true = dataclasses.replace(_TRUE, pos_ci95=20.0, vel_ci95=1.0)
    nav = GpsNavigation()
    a = nav.measure(true, 0.0, np.random.default_rng(42)).state
    b = nav.measure(true, 0.0, np.random.default_rng(42)).state
    assert a == b
