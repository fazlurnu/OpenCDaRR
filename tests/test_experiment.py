"""Functional tests for the experiment entry point."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest

from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.experiment import run_one_experiment


def _config() -> Config:
    return Config(
        seed=3,
        n_encounters=100,
        scenario=ScenarioConfig("M600", 10.2889, 50.0, 60.0),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 600.0, 10.0),
    )


def test_end_to_end_reproducible() -> None:
    cfg = _config()
    r1 = run_one_experiment(cfg, card_dir=None)
    r2 = run_one_experiment(cfg, card_dir=None)
    assert r1.ipr == r2.ipr
    assert r1.ipr.n_conflict == cfg.n_encounters
    assert r1.card_path is None


def test_writes_provenance_card(tmp_path: Path) -> None:
    cfg = _config()
    result = run_one_experiment(cfg, card_dir=tmp_path)
    assert result.card_path is not None and result.card_path.exists()
    text = result.card_path.read_text()
    assert f"seed: {cfg.seed}" in text
    assert "IPR:" in text
    assert "aircraft_type: M600" in text  # config was dumped


def test_unknown_method_raises() -> None:
    cfg = dataclasses.replace(
        _config(),
        methods=MethodsConfig("bogus", "mvp", "pastcpa", 1.05, False),
    )
    with pytest.raises(ValueError, match="unknown detector"):
        run_one_experiment(cfg, card_dir=None)
