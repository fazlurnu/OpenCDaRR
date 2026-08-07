"""Bit-identity locks for the parallel scheduler (``opencdarr/parallel.py``).

Unlike ``tests/test_ips.py``, these **actually start worker processes** — the whole claim of
:mod:`opencdarr.estimate.parallel` is that scheduling changes nothing, and that can only be
shown by
crossing a real process boundary. Comparisons are exact (``==``), never ``approx``: a scheduler
that perturbs the last bit of a probability has a bug, not a rounding difference.

The cases are chosen to hit both modes and the awkward edges between them — whole replications,
sharded lockstep, one replication over several workers, ragged shard bounds, and a ladder that
collapses so the drop-out branch runs under workers too. Particle counts stay tiny (a few dozen)
and ``min_shard`` is forced low, so sharding really fires without the suite getting slow.
"""

from __future__ import annotations

import importlib.util

import pytest

from opencdarr.estimate.ips import estimate_rare_prob as estimate_rare_prob_serial
from opencdarr.estimate.ips import ips_once, replication_seeds
from opencdarr.estimate.parallel import (
    _shard_bounds,
    _shard_count,
    _whole_replications,
    estimate_rare_prob,
    ips_replications,
    resolve_jobs,
)
from tests.test_ips import _build_initial, _make_build

pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("joblib") is None,
    reason="parallel execution needs the optional joblib extra",
)

LEVELS = [60.0, 55.0, 52.0, 50.0]
N = 24


def _serial(levels: list[float], reps: int, seed: int, n: int = N) -> list:
    return [ips_once(_build_initial, levels, n, s) for s in replication_seeds(seed, reps)]


@pytest.mark.parametrize(
    "reps, n_jobs",
    [
        (3, 1),  # the n_jobs=1 short circuit — no workers at all
        (2, 2),  # whole replications, one wave
        (4, 2),  # whole replications, two full waves
        (3, 2),  # reps % jobs != 0 -> lockstep, 2 shards per replication
        (1, 4),  # ONE replication over 4 workers: the case sharding exists for
        (5, 3),  # ragged: uneven shard bounds and an uneven replication count
    ],
)
def test_parallel_is_bit_identical_to_serial(reps: int, n_jobs: int) -> None:
    """Every scheduling shape reproduces the serial run exactly, field for field."""
    serial = _serial(LEVELS, reps, seed=3)
    got = ips_replications(
        _build_initial, LEVELS, N, replication_seeds(3, reps), n_jobs=n_jobs, min_shard=4
    )
    assert [r.prob for r in got] == [r.prob for r in serial]
    assert [r.survival for r in got] == [r.survival for r in serial]
    assert [r.collapsed_at for r in got] == [r.collapsed_at for r in serial]
    assert [r.n_particles for r in got] == [r.n_particles for r in serial]


@pytest.mark.parametrize("min_shard", [1, 2, 3, 5, 8, 24])
def test_shard_size_does_not_change_the_answer(min_shard: int) -> None:
    """Shard bounds change only the *grouping* of the per-level map, never the mapping.

    This is what lets the shard count vary from level to level as replications collapse out —
    if any of these disagreed, the self-refilling lockstep would be silently non-reproducible.
    """
    serial = _serial(LEVELS, 2, seed=11)
    got = ips_replications(
        _build_initial, LEVELS, N, replication_seeds(11, 2), n_jobs=3, min_shard=min_shard
    )
    assert [r.survival for r in got] == [r.survival for r in serial]


def test_collapsed_replication_drops_out_and_still_matches() -> None:
    """A ladder no particle can reach: every replication collapses, under workers as when serial.

    Exercises the branch where a replication leaves ``live`` mid-ladder — the one place the
    lockstep driver's bookkeeping differs structurally from the serial early ``return``.
    """
    build = _make_build(2.0)  # low noise: the resolver never lets 40 m happen
    levels = [70.0, 40.0]
    serial = [ips_once(build, levels, 20, s) for s in replication_seeds(5, 3)]
    got = ips_replications(build, levels, 20, replication_seeds(5, 3), n_jobs=2, min_shard=4)
    assert [r.collapsed_at for r in got] == [r.collapsed_at for r in serial]
    assert [r.survival for r in got] == [r.survival for r in serial]
    assert all(r.prob == 0.0 for r in got)


def test_estimate_rare_prob_matches_the_serial_estimator() -> None:
    """The public entry point agrees with ``opencdarr.estimate.ips.estimate_rare_prob`` — point
    and CI."""
    ref = estimate_rare_prob_serial(_build_initial, LEVELS, n_particles=N, reps=3, seed=3)
    got = estimate_rare_prob(
        _build_initial, LEVELS, n_particles=N, reps=3, seed=3, n_jobs=2, min_shard=4
    )
    assert got.prob == ref.prob
    assert got.n_collapsed == ref.n_collapsed
    assert [r.survival for r in got.reps] == [r.survival for r in ref.reps]


@pytest.mark.parametrize("n_jobs", [1, 2, 4])
def test_reusing_seeds_gives_the_same_run_on_every_path(n_jobs: int) -> None:
    """Re-running with the same seed objects reproduces the run, whichever path is taken.

    Uniformity across ``n_jobs`` is the point, and it is not automatic: whole-replication mode
    ships the seeds to workers, which would consume *copies* and leave the caller's originals
    fresh, while the serial and lockstep paths run in-process. If either spawned from the caller's
    sequence, reuse would be harmless at one ``n_jobs`` and silently change the answer at another.
    Addressing the tree by index makes all three the same.
    """
    seqs = replication_seeds(3, 2)
    first = ips_replications(_build_initial, LEVELS, N, seqs, n_jobs=n_jobs, min_shard=4)
    second = ips_replications(_build_initial, LEVELS, N, seqs, n_jobs=n_jobs, min_shard=4)
    assert [r.survival for r in first] == [r.survival for r in second]
    assert all(s.n_children_spawned == 0 for s in seqs)


def test_resolve_jobs_follows_the_joblib_convention() -> None:
    """Positive counts pass through; -1 is every core, -2 all but one; 0 is an error."""
    assert resolve_jobs(4) == 4
    everything = resolve_jobs(-1)
    assert everything >= 1
    assert resolve_jobs(-2) == max(1, everything - 1)
    with pytest.raises(ValueError):
        resolve_jobs(0)


def test_whole_replication_mode_needs_an_exact_multiple() -> None:
    """Only an exact multiple packs every wave; a remainder would idle most of the box."""
    assert _whole_replications(96, 96)
    assert _whole_replications(192, 96)
    assert not _whole_replications(100, 96)  # 4 workers run a 2nd wave, 92 wait
    assert not _whole_replications(10, 96)


def test_shard_bounds_tile_the_range_exactly() -> None:
    """Bounds are contiguous, non-empty, and cover every particle exactly once."""
    for n, shards in [(24, 1), (24, 5), (24, 24), (24, 50), (7, 3)]:
        bounds = _shard_bounds(n, shards)
        assert bounds[0][0] == 0
        assert bounds[-1][1] == n
        assert all(lo < hi for lo, hi in bounds)
        assert [hi for _, hi in bounds[:-1]] == [lo for lo, _ in bounds[1:]]


def test_shard_count_oversubscribes_but_respects_min_shard() -> None:
    """More tasks than workers (for load balancing), yet never shards below ``min_shard``."""
    assert _shard_count(1, 96, 10_000, oversubscribe=2, min_shard=64) == 156
    assert _shard_count(10, 96, 10_000, oversubscribe=2, min_shard=64) == 20
    # a small cloud is capped by min_shard, not by the worker count
    assert _shard_count(1, 96, 100, oversubscribe=2, min_shard=64) == 1
    assert _shard_count(96, 96, 10_000, oversubscribe=2, min_shard=64) == 2
