"""Cache identity — what a key is made of. The store itself is :mod:`opencdarr.cache`."""

from __future__ import annotations

import dataclasses
import hashlib
import inspect
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from opencdarr.cache import DEFAULT_CACHE_DIR
from opencdarr.config import Config
from opencdarr.experiment.backends import Backend
from opencdarr.experiment.cell import _resolved_methods
from opencdarr.experiment.conditions import Condition
from opencdarr.experiment.declaration import _GEOMETRY_SLOTS
from opencdarr.experiment.methods import Methods

# A cache may only ever save time; it must never change a result (opencdarr.cache). That holds only
# if the key captures everything determining the numbers — and a live component object is hostile
# to keying: GnssNavigation and Comm hold *function* objects whose repr carries a memory address,
# so the obvious str(obj) would be unstable across processes, and a lambda from a factory would key
# on its qualname alone, colliding constant_latency(0) with constant_latency(5) — different
# physics, one key. So identity is derived structurally, and refused when it cannot be established.


class CacheIdentityError(ValueError):
    """A value's cache identity could not be established, so caching would risk a stale hit.

    Raised rather than falling back to a weaker key: a wrong key is worse than no cache, because it
    silently serves numbers computed by different code or different parameters. Fix it by giving
    the object a ``cache_id`` attribute naming what makes it distinct — you then own that promise::

        class MyResolver(ConflictResolver):
            cache_id = "my-resolver/v3"
    """


_PRIMITIVES = (type(None), bool, int, float, str)


def _source_digest(obj: Any) -> str:
    """A hash of the source of a class or function — what catches an edit to *user* code.

    :func:`opencdarr.cache.code_fingerprint` covers the library, so a change to a built-in resolver
    already invalidates every key. It does **not** cover a contributor's own class, and a resolver
    whose logic changed while its constructor arguments did not would otherwise be served a stale
    result. Source is unavailable for a class defined in a plain REPL, which is refused rather than
    silently keyed on its name alone.
    """
    try:
        source = inspect.getsource(obj)
    except (OSError, TypeError) as exc:  # REPL-defined, C-implemented, or otherwise sourceless
        raise CacheIdentityError(
            f"cannot read the source of {obj!r}, so a cache key cannot detect edits to it. "
            f"Give it a `cache_id` attribute, or run from an importable module."
        ) from exc
    return hashlib.sha256(source.encode()).hexdigest()[:12]


def _qualified(cls: type) -> str:
    return f"{cls.__module__}.{cls.__qualname__}"


def identity(value: Any) -> str:
    """A stable, content-derived cache identity for ``value``, or raise.

    Stable across processes (no ``repr`` of an address) and sensitive to anything that changes the
    numbers: constructor values, the class's own source, and a closure's captured arguments.

    An explicit ``cache_id`` always wins. Otherwise: primitives by value; sequences and frozen
    dataclasses structurally; a plain function by module, qualname, source and captured free
    variables; and any other instance by its class source plus its **public** attributes. Private
    (leading-underscore) attributes and free variables are treated as *derived* — the memo dict
    inside :func:`~opencdarr.cns.noise_distributions.make_mixture_gaussian` is the motivating case,
    since it is filled during a run and would otherwise make the key depend on how far the run got.
    Their *names* are still included, so gaining or losing one changes the key.
    """
    explicit = getattr(value, "cache_id", None)
    if explicit is not None:
        return f"id:{explicit}"
    if isinstance(value, _PRIMITIVES):
        return repr(value)
    if isinstance(value, (tuple, list)):
        return "[" + ",".join(identity(v) for v in value) + "]"
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        inner = ",".join(
            f"{f.name}={identity(getattr(value, f.name))}" for f in dataclasses.fields(value)
        )
        return f"{_qualified(type(value))}({inner})"
    if inspect.isfunction(value):
        return _function_identity(value)
    if isinstance(value, type):
        return f"{_qualified(value)}#{_source_digest(value)}"
    if hasattr(value, "__dict__"):
        return _instance_identity(value)
    raise CacheIdentityError(
        f"no cache identity for {value!r} (type {type(value).__name__}). "
        f"Give it a `cache_id` attribute if it is stable."
    )


def _function_identity(func: Any) -> str:
    """A function by module, qualname and source — plus the arguments its closure captured.

    The captured values are the load-bearing part: the noise and latency models in this package are
    all *factories* returning a closure, so two calls to one factory differ only in their cells.
    Without them, ``constant_latency(0)`` and ``constant_latency(5)`` would share a key.
    """
    base = f"{func.__module__}.{func.__qualname__}#{_source_digest(func)}"
    cells = func.__closure__ or ()
    if not cells:
        return base
    captured = []
    for name, cell in zip(func.__code__.co_freevars, cells, strict=True):
        if name.startswith("_"):
            captured.append(f"{name}=<derived>")  # named, not valued (see identity's docstring)
            continue
        captured.append(f"{name}={identity(cell.cell_contents)}")
    return f"{base}({','.join(captured)})"


def _instance_identity(obj: Any) -> str:
    """An instance by its class's source plus its public attribute values."""
    public = sorted(k for k in vars(obj) if not k.startswith("_"))
    inner = ",".join(f"{k}={identity(getattr(obj, k))}" for k in public)
    return f"{_qualified(type(obj))}#{_source_digest(type(obj))}({inner})"


@dataclass(frozen=True)
class CacheConfig:
    """Where per-condition results are cached, and whether they are.

    Off by default. Granularity is **one entry per condition**, so extending a sweep re-runs only
    the new cells and a crashed run loses at most the cell in flight. What is stored is the raw
    estimator result, not a reduced metric, so a metric added later recomputes from cache with no
    new simulation.
    """

    dir: Path = DEFAULT_CACHE_DIR
    enabled: bool = True


def _cache_params(
    condition: Condition, config: Config, methods: Methods, backend: Backend
) -> dict[str, Any]:
    """Everything that determines this cell's numbers, as JSON-able identities.

    The code fingerprint is added by :func:`opencdarr.cache.run_key`, so the library half of "same
    code" is already covered; :func:`identity` covers the user half.

    ``methods`` is resolved **here**, against this condition, rather than being keyed as the bundle
    the caller passed. A component declared as an axis — ``resolver=Sweep([...])`` and the other
    seven in :data:`_COMPONENTS` — differs per condition while the bundle does not, so keying the
    bundle made every level of a component sweep share one key: the first cell computed, and every
    later one was served *its* numbers under a different name. Resolving here rather than at the
    call site keeps the key and the run reading the same objects, which is the only version of this
    that cannot drift apart again.
    """
    resolved = _resolved_methods(condition, methods)
    return {
        "config": dataclasses.asdict(config),
        "methods": {
            f.name: identity(getattr(resolved, f.name)) for f in dataclasses.fields(resolved)
        },
        "geometry": {
            k: identity(v) for k, v in condition.values if k in _GEOMETRY_SLOTS
        },
        "backend": identity(backend),
    }
