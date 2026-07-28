"""Locks for the RNG stream tree (``opencdarr/rng.py``, ADR 0001).

The load-bearing property here is that :func:`~opencdarr.rng.child` addresses the *same* tree
:func:`~opencdarr.rng.spawn` builds: a child is fixed by its parent and its index, so a parallel
worker can rebuild only its slice of a fan-out and still draw the numbers the serial run would.
Everything the parallel estimators promise about reproducibility rests on that.
"""

from __future__ import annotations

import numpy as np
import pytest

from opencdarr.rng import child, children, generator, require_fresh, root_seed_sequence, spawn


def _draw(seq: np.random.SeedSequence) -> int:
    """A single draw that fingerprints a stream (two equal sequences draw the same number)."""
    return int(generator(seq).integers(0, 2**62))


def test_child_matches_spawn_by_index() -> None:
    """``child(p, i)`` is ``spawn(p, n)[i]`` — the same tree, addressed instead of enumerated."""
    root = root_seed_sequence(12345)
    kids = spawn(root, 8)
    assert [_draw(child(root, i)) for i in range(8)] == [_draw(k) for k in kids]


def test_child_matches_spawn_for_a_nested_parent() -> None:
    """Also true when the parent is itself a spawned node (non-empty ``spawn_key``) — which is how
    IPS uses it: per-replication, then per-level, then per-particle."""
    node = spawn(root_seed_sequence(7), 4)[2]
    assert [_draw(child(node, i)) for i in range(5)] == [_draw(k) for k in spawn(node, 5)]


def test_child_index_is_independent_of_the_fan_out_width() -> None:
    """The index is absolute, so a worker rebuilding its slice need not know the total width.

    Each side gets a *fresh* root: ``spawn`` is stateful (see below), so re-spawning one object
    would compare index 1 against index 4, not two spellings of the same child.
    """
    assert (
        _draw(spawn(root_seed_sequence(99), 3)[1])
        == _draw(spawn(root_seed_sequence(99), 5000)[1])
        == _draw(child(root_seed_sequence(99), 1))
    )


def test_spawn_is_stateful_so_child_addresses_absolutely() -> None:
    """``spawn`` continues from ``n_children_spawned``; ``child`` indexes absolutely.

    They coincide only while the parent is fresh — which the ADR-0001 contract guarantees, since a
    sequence is an internal node fanned out once, or a leaf. Pinned here because mixing the two on
    one parent would silently re-use indices, and every parallel result depends on this.
    """
    root = root_seed_sequence(5)
    assert [_draw(k) for k in spawn(root, 2)] == [_draw(child(root, i)) for i in (0, 1)]
    assert [_draw(k) for k in spawn(root, 2)] == [_draw(child(root, i)) for i in (2, 3)]
    assert root.n_children_spawned == 4  # ... whereas child() left the parent untouched


def test_children_is_a_slice_of_spawn() -> None:
    """``children`` returns a half-open slice — one chunk's worth of substreams."""
    root = root_seed_sequence(2024)
    full = [_draw(k) for k in spawn(root, 10)]
    assert [_draw(k) for k in children(root, 0, 10)] == full
    assert [_draw(k) for k in children(root, 4, 7)] == full[4:7]
    assert children(root, 3, 3) == []


def test_children_chunks_tile_the_whole_fan_out() -> None:
    """Contiguous chunks reassemble the serial fan-out exactly — the parallel chunking contract."""
    root = root_seed_sequence(31)
    chunked = [k for lo in range(0, 12, 5) for k in children(root, lo, min(lo + 5, 12))]
    assert [_draw(k) for k in chunked] == [_draw(k) for k in spawn(root, 12)]


def test_require_fresh_accepts_an_unspawned_sequence() -> None:
    """Everything the module hands out is fresh: roots, spawned children, indexed children."""
    root = root_seed_sequence(3)
    require_fresh(root, "test")
    require_fresh(spawn(root, 2)[1], "test")
    require_fresh(child(root_seed_sequence(3), 4), "test")


def test_require_fresh_rejects_a_consumed_sequence() -> None:
    """Once spawned from, a sequence would hand out *different* children next time — reject it.

    The error has to name the routine and say what to do instead, because the failure it prevents
    is silent: the same call on the same object returning a different answer.
    """
    root = root_seed_sequence(3)
    spawn(root, 2)
    with pytest.raises(ValueError, match="already handed out 2 children"):
        require_fresh(root, "some_estimator")


def test_reusing_a_sequence_really_would_change_the_stream() -> None:
    """The bug the guard exists for: spawn twice from one object, get a different tree.

    Not a hypothetical — this produced a convincing false 'the results changed' signal during the
    parallel-scheduler work. Pinned so the guard's reason for existing is visible, not just its
    behaviour.
    """
    root = root_seed_sequence(3)
    first = [_draw(k) for k in spawn(root, 2)]
    second = [_draw(k) for k in spawn(root, 2)]  # continues at index 2, not back to 0
    assert first != second


def test_rejects_negative_indices() -> None:
    root = root_seed_sequence(0)
    with pytest.raises(ValueError):
        child(root, -1)
    with pytest.raises(ValueError):
        children(root, -1, 4)
    with pytest.raises(ValueError):
        children(root, 5, 2)


def test_root_seed_sequence_rejects_negative_seed() -> None:
    with pytest.raises(ValueError):
        root_seed_sequence(-1)


def test_spawn_rejects_negative_n() -> None:
    with pytest.raises(ValueError):
        spawn(root_seed_sequence(0), -1)
