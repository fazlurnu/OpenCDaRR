"""Declare an experiment once, get one row per condition — the user-facing face of the estimators.

:func:`run_experiment` turns a **declaration** of what varies into a table of results. You say
which parameters are held fixed and which are swept, hand it a backend, and it runs the
cross-product and tabulates it. The point is that the *same* objects — your detector, your
resolver, your kinematics —
run unchanged whichever backend estimates the probability::

    res = run_experiment(
        {"dpsi": Sweep([5, 45, 90, 135, 180]), "pos_ci95": Fixed(10.0)},
        methods=Methods(detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA()),
        backend=MC(n_encounters=2_000),
        seed=0,
    )
    res.records()        # one dict per condition
    res.frame()          # ... as a pandas DataFrame (pandas imported lazily)
    res.cell(dpsi=90)    # the raw estimator result for one condition

Swap ``backend=MC(...)`` for ``backend=IPS(shells=[...], n_particles=2_000, reps=8)`` and nothing
else changes — that equivalence is the whole reason the plain-MC estimator was moved onto the fleet
environment the rare-event estimator already drove (:mod:`opencdarr.estimate.montecarlo`).

**Two roles, not three.** A parameter is :class:`Fixed` (held) or :class:`Sweep` (an output axis,
one condition per level). There is deliberately no "draw it from a distribution" role here: that
already lives one level down, as a callable on the geometry slots of
:func:`~opencdarr.scenario.sample_pairwise`, where it belongs — it changes the encounter
*distribution*, not the set of conditions. A :class:`Sweep` is a fan-out over conditions, so at the
sampler boundary it collapses to a pinned value; that is why sweeping needs no sampler support.

**Marginalising a distribution over an axis is not here.** Estimating ``E_p[P(LoS)]`` for some
``p(dpsi)`` is a *reduction over* a swept response curve (weight the per-condition estimates), and
weighting rates rather than counts is the mistake
:func:`~opencdarr.estimate.montecarlo.combine_p_los` exists to warn about. Sweep the axis, then
weight the counts yourself, until that reduction earns a home.

**The file-driven entry point lives here too.** :func:`run_one_experiment` takes a YAML-loaded
:class:`~opencdarr.config.Config` and is exactly the all-:class:`Fixed`, single-condition case of
:func:`run_experiment` — it names its components as strings and resolves them through
:mod:`opencdarr.experiment.registry`. It is kept because a config file is committable and diffable
in a way a Python call is not, so ``config + seed -> result`` stays reproducible without writing
code; it is
*not* a second implementation, and there is one card writer for both.

The module grew into a package along its old section banners — ``declaration`` / ``backends`` /
``methods`` / ``conditions`` / ``cell`` / ``identity`` / ``results`` / ``card`` — and everything
is re-exported here, so ``from opencdarr.experiment import run_experiment, Methods`` is unchanged.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from opencdarr import cache
from opencdarr.config import Config
from opencdarr.estimate.parallel import _joblib, resolve_jobs
from opencdarr.experiment import registry
from opencdarr.experiment.backends import IPS, MC, Backend
from opencdarr.experiment.card import _write_card

# The two ``as``-aliased names are explicit re-exports: tests reach them as package attributes
# (``experiment._config_for`` / ``experiment._SIMULATION_FIELDS``,
# tests/test_broadcast_schedule.py), exactly as they did when this was one flat module.
from opencdarr.experiment.cell import (
    _config_for as _config_for,
)
from opencdarr.experiment.cell import (
    _run_ips,
    _run_mc,
    _validate_declared_accuracy_is_read,
    _validate_splittable,
)
from opencdarr.experiment.conditions import Condition, expand
from opencdarr.experiment.declaration import (
    _SIMULATION_FIELDS as _SIMULATION_FIELDS,
)
from opencdarr.experiment.declaration import (
    Axis,
    Fixed,
    Sweep,
)
from opencdarr.experiment.identity import CacheConfig, CacheIdentityError, _cache_params, identity
from opencdarr.experiment.methods import Methods
from opencdarr.experiment.results import ExperimentResult

__all__ = [
    "IPS",
    "MC",
    "Axis",
    "Backend",
    "CacheConfig",
    "CacheIdentityError",
    "Condition",
    "ExperimentResult",
    "Fixed",
    "Methods",
    "Sweep",
    "expand",
    "identity",
    "run_experiment",
    "run_one_experiment",
]

def _run_one(
    condition: Condition,
    base_config: Config,
    methods: Methods,
    backend: Backend,
    seed: int,
    cache_dir: Path | None,
    jobs: int = 1,
) -> Any:
    """One condition, through the cache when one is configured.

    Module-level (not a closure) so a parallel worker can pickle it. The cell is keyed on its
    resolved config, its component identities, its geometry pins, the backend and the seed — plus
    the library code fingerprint, which :func:`opencdarr.cache.run_key` adds. ``jobs`` is
    deliberately **not** in that key: it changes the wall time and no number, so keying on it would
    store several identical copies of one cell and re-run after every change of machine.
    """
    n = backend.n_encounters if isinstance(backend, MC) else backend.n_particles
    config = dataclasses.replace(_config_for(condition, base_config, n), seed=seed)

    def compute() -> Any:
        if isinstance(backend, MC):
            return _run_mc(condition, base_config, methods, backend, seed, jobs)
        return _run_ips(condition, base_config, methods, backend, seed, jobs)

    if cache_dir is None:
        return compute()
    key = cache.run_key(_cache_params(condition, config, methods, backend), seed=seed)
    return cache.load_or_run(key, compute, cache_dir=cache_dir)


def run_experiment(
    independent_vars: Mapping[str, Axis],
    *,
    methods: Methods,
    backend: Backend,
    base_config: Config,
    seed: int = 0,
    cache: bool | CacheConfig = False,
    n_jobs: int = 1,
    card_dir: Path | None = None,
) -> ExperimentResult:
    """Run every condition of ``independent_vars`` on ``backend`` and tabulate the results.

    ``base_config`` supplies the parameters no axis declares (and the numerics); anything declared
    as :class:`Fixed` or :class:`Sweep` overrides it. ``seed`` is the reproducibility root, shared
    by both backends and common to every condition — so two conditions differ by their declared
    levels and nothing else, which is what makes a sweep a comparison rather than a set of runs.

    ``cache=True`` (or a :class:`CacheConfig`) stores one entry per condition, so extending a sweep
    re-runs only the new cells. It is **off by default** and will raise
    :class:`CacheIdentityError` rather than key a component it cannot identify — see
    :func:`identity`.

    ``n_jobs`` is the worker budget (joblib's convention: ``-1`` is every core), and it is spent
    where it helps. Up to one worker per condition the conditions are fanned out, which is the
    cheapest split because nothing crosses a process boundary but a seed and a result. Past that
    the conditions run in turn and the budget goes **inside** each one: Monte Carlo slices its
    encounter fan-out, and the splitting estimator shards each level (ADR 0018). Never both at
    once — two pools would nest, and loky does not survive that.

    That second mode is the one that matters for a rare-event study, which is often a single
    condition and hours long: without it such a cell runs on one core however many are free.

    It is a pure scheduling choice with no effect on the numbers, and it is not part of the cache
    key. It needs the ``parallel`` extra, and the components must be picklable, which rules out a
    lambda held on a component instance.

    ``card_dir`` writes one provenance card for the run; ``None`` (default) writes nothing.
    """
    conditions = expand(independent_vars)
    _validate_declared_accuracy_is_read(conditions, base_config, methods)
    _validate_splittable(conditions, methods, backend)
    axes = tuple(
        (axis.name or key)
        for key, axis in independent_vars.items()
        if isinstance(axis, Sweep)
    )
    cfg = CacheConfig() if cache is True else (cache or None)
    cache_dir = cfg.dir if isinstance(cfg, CacheConfig) and cfg.enabled else None

    workers = resolve_jobs(n_jobs)
    if workers == 1:
        results: tuple[Any, ...] = tuple(
            _run_one(c, base_config, methods, backend, seed, cache_dir) for c in conditions
        )
    elif workers <= len(conditions):
        # more conditions than workers: fan the conditions out, one core each
        parallel_cls, delayed = _joblib()
        results = tuple(
            parallel_cls(n_jobs=workers)(
                delayed(_run_one)(c, base_config, methods, backend, seed, cache_dir)
                for c in conditions
            )
        )
    else:
        # more workers than conditions: run the conditions in turn and spend the whole budget
        # inside each cell. Fanning conditions out here would leave the surplus workers idle.
        results = tuple(
            _run_one(c, base_config, methods, backend, seed, cache_dir, jobs=workers)
            for c in conditions
        )

    result = ExperimentResult(
        backend=backend, seed=seed, conditions=conditions, results=results, axes=axes,
        card_path=None,
    )
    if card_dir is None:
        return result
    return dataclasses.replace(
        result, card_path=_write_card(result, independent_vars, methods, base_config, card_dir)
    )


def run_one_experiment(
    config: Config, *, card_dir: Path | None = Path("vault/experiments")
) -> ExperimentResult:
    """Run the single experiment a YAML :class:`~opencdarr.config.Config` describes.

    The all-:class:`Fixed`, one-condition case of :func:`run_experiment`, with the components
    named as strings and resolved through :mod:`opencdarr.experiment.registry`. Nothing is
    declared as an axis, so the result has one row; ``res.cell()`` reaches the
    :class:`~opencdarr.estimate.montecarlo.MonteCarloEstimate` behind it.

    Kept as its own entry point because a config file is *committable and diffable* in a way a
    Python call is not — ``config + seed + code-hash -> result`` without writing code. It is a thin
    wrapper, not a second implementation: the estimator, the cache and the card are the same ones
    :func:`run_experiment` uses, so the two cannot drift.

    The registry is the limit, not this function: only the names
    :mod:`opencdarr.experiment.registry` knows are reachable from a file. Pass instances to
    :func:`run_experiment` for anything else.
    """
    methods = Methods(
        detector=registry.make_detector(config.methods.detection),
        resolver=registry.make_resolver(config.methods.resolution, config.methods.margin),
        recovery=registry.make_recovery(config.methods.recovery, config.methods.bouncing_guard),
        perf=registry.make_perf(config.scenario.aircraft_type),
    )
    return run_experiment(
        {},  # nothing varies: base_config supplies every parameter
        methods=methods,
        backend=MC(n_encounters=config.n_encounters),
        base_config=config,
        seed=config.seed,
        card_dir=card_dir,
    )
