"""Disk cache for fleet runs — recompute only when the inputs or the code change.

A recorded run (``run_fleet(..., record=True)``) carries its full states log, and an expensive
encounter (or a whole sweep) can be slow to reproduce just to redraw a figure. This module persists
a run to disk and reloads it on the next identical request, keyed on **the run's parameters + its
seed + a fingerprint of the ``opencdarr`` source** — the ``config + seed + code-hash`` model the
rest of the project uses. Change any of the three and the key changes, so a stale result is never
returned; leave them fixed and the run is loaded, not recomputed.

Why the caller supplies the key rather than the cache reading it off ``run_fleet``: that function
takes *live* objects (a ``StateBased()`` detector, an already-spawned ``Generator``), which have no
stable identity to hash. :func:`run_key` turns the plain, JSON-able description of a run — the same
values you passed to build it — into that key.

    from opencdarr import cache
    params = {"scenario": "pairwise", "pos_ci95": 15.0, "resolver": "mvp"}
    key = cache.run_key(params, seed=20260725)
    run = cache.load_or_run(key, lambda: run_fleet(..., record=True))

The store is :mod:`pickle`; the source fingerprint in every key means a pickle written by different
code is simply never looked up (a new key), and a corrupt or unreadable file falls back to
recompute — so the cache can only ever save time, never change a result.
"""

from __future__ import annotations

import hashlib
import json
import pickle
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, TypeVar, cast

_T = TypeVar("_T")

DEFAULT_CACHE_DIR = Path(".opencdarr_cache")

_fingerprint: str | None = None  # memoised per process; the source does not change mid-run.


def code_fingerprint() -> str:
    """A short hash of every ``opencdarr`` ``.py`` source file — the "code-hash" part of a key.

    Any edit to the package changes this, so cached runs written by other code are keyed
    differently and never loaded. Computed once and reused for the life of the process.
    """
    global _fingerprint
    if _fingerprint is None:
        package_root = Path(__file__).resolve().parent
        digest = hashlib.sha256()
        for path in sorted(package_root.rglob("*.py")):
            digest.update(path.relative_to(package_root).as_posix().encode())
            digest.update(path.read_bytes())
        _fingerprint = digest.hexdigest()[:16]
    return _fingerprint


def run_key(params: Mapping[str, Any], seed: int | None = None) -> str:
    """Stable cache key from a run's JSON-able parameters, its ``seed``, and the code fingerprint.

    ``params`` is any plain description of the run whose values fully determine it (scenario
    geometry, noise level, which detector/resolver/recovery, ``dt`` …); non-JSON values are
    stringified. Two calls with equal ``params`` + ``seed`` under unchanged code get the same key.
    """
    payload = json.dumps(
        {"params": params, "seed": seed, "code": code_fingerprint()},
        sort_keys=True, default=str,
    )
    return hashlib.sha256(payload.encode()).hexdigest()[:32]


def load_or_run(
    key: str,
    compute: Callable[[], _T],
    *,
    cache_dir: Path | str = DEFAULT_CACHE_DIR,
) -> _T:
    """Return the cached result for ``key``, else run ``compute()``, store it, and return it.

    A hit reads ``<cache_dir>/<key>.pkl``; a miss (or an unreadable/stale file) calls ``compute``
    and pickles the result. Because the store only ever short-circuits an identical recomputation,
    it cannot change what ``compute`` would have produced.
    """
    directory = Path(cache_dir)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{key}.pkl"
    if path.exists():
        try:
            with path.open("rb") as handle:
                return cast(_T, pickle.load(handle))
        except (pickle.UnpicklingError, EOFError, AttributeError, ModuleNotFoundError):
            pass  # corrupt or written by incompatible code — fall through and recompute
    result = compute()
    with path.open("wb") as handle:
        pickle.dump(result, handle)
    return result
