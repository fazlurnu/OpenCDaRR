"""Phase 0 smoke test — the scaffolding gate.

Proves the seams work and the two load-bearing properties hold from day one:
reproducible, independent RNG substreams (ADR 0001) and clonable, hidden-state-free
aircraft state. Green here is necessary, not sufficient (``how-to-step-by-step.md`` Part A
#5) — each test is written so that breaking the thing it guards makes it fail.
"""

from __future__ import annotations

import copy
import dataclasses

import numpy as np
import pytest

import opencdarr
from opencdarr import rng
from opencdarr.state import AircraftState


def test_package_imports() -> None:
    """The package imports and reports a version (the gate's 'imports work')."""
    assert isinstance(opencdarr.__version__, str)


# --- RNG: reproducibility & independence (ADR 0001) --------------------------


def test_rng_reproducible() -> None:
    """The same seed yields identical draws — reproducibility is a tested feature."""
    a = rng.generator(rng.root_seed_sequence(42)).random(5)
    b = rng.generator(rng.root_seed_sequence(42)).random(5)
    assert np.array_equal(a, b)


def test_rng_substreams_are_independent() -> None:
    """Two spawned children produce different streams (not the shared-RNG bug)."""
    child_a, child_b = rng.spawn(rng.root_seed_sequence(42), 2)
    draw_a = rng.generator(child_a).random(5)
    draw_b = rng.generator(child_b).random(5)
    assert not np.array_equal(draw_a, draw_b)


def test_rng_spawn_tree_is_reproducible() -> None:
    """The spawn tree itself is fixed by the seed: same seed -> same children."""
    first = rng.generator(rng.spawn(rng.root_seed_sequence(7), 3)[1]).random(5)
    second = rng.generator(rng.spawn(rng.root_seed_sequence(7), 3)[1]).random(5)
    assert np.array_equal(first, second)


def test_rng_rejects_negative_seed() -> None:
    with pytest.raises(ValueError):
        rng.root_seed_sequence(-1)


def test_spawn_rejects_negative_n() -> None:
    with pytest.raises(ValueError):
        rng.spawn(rng.root_seed_sequence(0), -1)


# --- State: immutable, clonable, no hidden state -----------------------------


def _state() -> AircraftState:
    return AircraftState(id="DRO000", lat=52.0, lon=4.0, trk=90.0, gs=30.0)


def test_state_is_frozen() -> None:
    """A declared field cannot be reassigned in place."""
    with pytest.raises(dataclasses.FrozenInstanceError):
        _state().lat = 53.0  # type: ignore[misc]


def test_state_rejects_stray_attributes() -> None:
    """No hidden state can be smuggled onto an instance (the KI-1 bug class)."""
    with pytest.raises(dataclasses.FrozenInstanceError):
        _state().secret = 1  # type: ignore[attr-defined]


def test_state_replace_does_not_alias() -> None:
    """Evolving state yields a new object; the original is untouched."""
    original = _state()
    evolved = dataclasses.replace(original, lat=53.0)
    assert evolved.lat == 53.0
    assert original.lat == 52.0
    assert evolved is not original


def test_state_deepcopy_is_distinct_but_equal() -> None:
    """A clone equals its source but is a separate object (safe to clone particles)."""
    original = _state()
    clone = copy.deepcopy(original)
    assert clone == original
    assert clone is not original
