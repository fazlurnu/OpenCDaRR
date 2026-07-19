"""Functional tests for config loading and validation."""

from __future__ import annotations

from pathlib import Path

import pytest

from opencdarr.config import Config, load_config

_EXAMPLE = Path(__file__).resolve().parent.parent / "configs" / "pairwise.yaml"

_GOOD = """
seed: 7
n_encounters: 100
scenario: {aircraft_type: M600, speed: 10.0, dcpa_max: 50.0, tlos: 60.0}
conflict: {rpz: 50.0, t_lookahead: 120.0}
methods:
  detection: statebased
  resolution: mvp
  recovery: pastcpa
  margin: 1.05
  bouncing_guard: false
simulation: {dt: 1.0, t_max: 600.0, done_timeout: 10.0}
"""


def test_example_config_loads() -> None:
    cfg = load_config(_EXAMPLE)
    assert isinstance(cfg, Config)
    assert cfg.seed == 42
    assert cfg.scenario.aircraft_type == "M600"
    assert cfg.conflict.rpz == 50.0
    assert cfg.methods.resolution == "mvp"
    assert cfg.simulation.dt == 1.0


def test_round_trips_nested_values(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(_GOOD)
    cfg = load_config(path)
    assert cfg.n_encounters == 100
    assert cfg.methods.bouncing_guard is False
    assert cfg.scenario.tlos == 60.0


def test_null_resolution_is_none(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(_GOOD.replace("resolution: mvp", "resolution: null"))
    assert load_config(path).methods.resolution is None


def test_invalid_value_raises(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(_GOOD.replace("rpz: 50.0", "rpz: -5.0"))
    with pytest.raises(ValueError, match="rpz > 0"):
        load_config(path)


def test_missing_section_raises(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text(_GOOD.replace("conflict: {rpz: 50.0, t_lookahead: 120.0}", ""))
    with pytest.raises(ValueError, match="invalid config"):
        load_config(path)
