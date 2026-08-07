"""The worker budget: ``n_jobs`` spent where it helps, and never changing a number.

``n_jobs`` used to fan *conditions* out and stop there, so a one-condition experiment ran on one
core however many were free — and a rare-event cell is exactly the shape that is one condition and
hours long. The budget now goes inside a cell once there are more workers than conditions: Monte
Carlo splits its encounter fan-out into contiguous seed slices, and the splitting estimator shards
each level (ADR 0018).

Never both at once. Two pools would nest, and loky does not survive that.

Every assertion here is an equality against the serial answer. Scheduling is allowed to change the
wall time and nothing else, so a difference in any reported field is a bug and not a tolerance.
"""

from __future__ import annotations

from typing import Any

import pytest

from opencdarr.experiment import IPS, MC, Fixed, Sweep, run_experiment
from tests.test_experiment import _base, _methods

# a pinned, noisy crossing: small enough to run fast, noisy enough that a loss is reachable
_ONE = {"dpsi": Fixed(90.0), "dcpa": Fixed(0.0), "pos_ci95": Fixed(40.0), "vel_ci95": Fixed(4.0)}
_TWO = {**_ONE, "dpsi": Sweep([45.0, 90.0])}


def _mc(declaration: dict[str, Any], jobs: int) -> Any:
    return run_experiment(declaration, methods=_methods(), backend=MC(n_encounters=24),
                          base_config=_base(), seed=0, n_jobs=jobs)


def _ips(declaration: dict[str, Any], jobs: int) -> Any:
    return run_experiment(
        declaration, methods=_methods(),
        backend=IPS(shells=[70.0, 60.0, 50.0], n_particles=24, reps=2),
        base_config=_base(), seed=0, n_jobs=jobs,
    )


def test_one_condition_more_workers_than_conditions_mc() -> None:
    """One MC condition and four workers: the budget goes *inside* the cell, and pools back.

    This is the case the old scheduling could not use at all — one condition, so fanning conditions
    out leaves three workers idle. The encounter fan-out is sliced instead, and ``combine_p_los``
    pools the slices into the serial answer exactly (contiguous ``children`` of the same tree).
    """
    pytest.importorskip("joblib")
    serial, parallel = _mc(_ONE, 1), _mc(_ONE, 4)
    assert parallel.cell() == serial.cell()      # every field, min_seps included
    assert parallel.records() == serial.records()


def test_one_condition_more_workers_than_conditions_ips() -> None:
    """The same for IPS, where the budget shards each level rather than the encounter fan-out.

    Asserted on the raw estimate, so the tail fields (``tail_k``, ``tail_a``, ``n_lineages``) are
    compared too — those cross a process boundary in the sharded path, which is where an
    identity-counted lineage total would disagree.
    """
    pytest.importorskip("joblib")
    serial, parallel = _ips(_ONE, 1), _ips(_ONE, 4)
    assert parallel.cell() == serial.cell()
    assert parallel.records() == serial.records()


@pytest.mark.parametrize("jobs", [1, 2, 8])
def test_the_answer_does_not_depend_on_the_worker_count(jobs: int) -> None:
    """1, 2 and 8 workers over two conditions give one answer.

    Two conditions and 8 workers crosses the switch: there are more workers than conditions, so the
    budget stops fanning conditions and goes inside each cell instead. The point of the sweep over
    ``jobs`` is that the *switch itself* is invisible in the numbers.
    """
    pytest.importorskip("joblib")
    assert _mc(_TWO, jobs).records() == _mc(_TWO, 1).records()


def test_n_jobs_is_not_part_of_the_cache_key(tmp_path: Any) -> None:
    """A cached cell is reused whatever the worker count was, because it changes no number.

    Keying on it would mean the same experiment re-runs from scratch after a machine change, and
    would quietly store several identical copies of one cell.
    """
    pytest.importorskip("joblib")
    from opencdarr.experiment import CacheConfig

    cache = CacheConfig(dir=tmp_path)
    first = run_experiment(_ONE, methods=_methods(), backend=MC(n_encounters=24),
                           base_config=_base(), seed=0, n_jobs=1, cache=cache)
    entries = list(tmp_path.rglob("*.pkl"))
    second = run_experiment(_ONE, methods=_methods(), backend=MC(n_encounters=24),
                            base_config=_base(), seed=0, n_jobs=4, cache=cache)
    assert second.records() == first.records()
    assert list(tmp_path.rglob("*.pkl")) == entries      # no second copy was written
