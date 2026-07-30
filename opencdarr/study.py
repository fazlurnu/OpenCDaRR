"""Declare a study once, get one row per condition — the user-facing face of the estimators.

:func:`run_study` turns a **declaration** of what varies into a table of results. You say which
parameters are held fixed and which are swept, hand it a backend, and it runs the cross-product and
tabulates it. The point is that the *same* objects — your detector, your resolver, your dynamics —
run unchanged whichever backend estimates the probability::

    res = run_study(
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
environment the rare-event estimator already drove (:mod:`opencdarr.estimator`).

**Two roles, not three.** A parameter is :class:`Fixed` (held) or :class:`Sweep` (an output axis,
one condition per level). There is deliberately no "draw it from a distribution" role here: that
already lives one level down, as a callable on the geometry slots of
:func:`~opencdarr.scenario.sample_pairwise`, where it belongs — it changes the encounter
*distribution*, not the set of conditions. A :class:`Sweep` is a fan-out over conditions, so at the
sampler boundary it collapses to a pinned value; that is why sweeping needs no sampler support.

**Marginalising a distribution over an axis is not here.** Estimating ``E_p[P(LoS)]`` for some
``p(dpsi)`` is a *reduction over* a swept response curve (weight the per-condition estimates), and
weighting rates rather than counts is the mistake :func:`~opencdarr.estimator.combine_ipr` exists
to warn about. Sweep the axis, then weight the counts yourself, until that reduction earns a home.
"""

from __future__ import annotations

import dataclasses
import itertools
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from opencdarr import cache
from opencdarr.cache import DEFAULT_CACHE_DIR
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import CommunicationModel, NavigationModel, SurveillanceModel
from opencdarr.config import Config
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.dynamics import Dynamics
from opencdarr.estimator import IPRResult, estimate_ipr
from opencdarr.fleet import Agent, build_env
from opencdarr.ips import Particle, RareEventEstimate, estimate_rare_prob
from opencdarr.parallel import _joblib, resolve_jobs
from opencdarr.performance import Performance
from opencdarr.rng import generator
from opencdarr.scenario import sample_pairwise

# --- what can be declared -------------------------------------------------------------------
# Every key below is wired end to end. The list is deliberately short: it is what the estimators
# actually thread today, not an aspiration. An unknown key fails immediately with this list in the
# message, which is a better contributor experience than a silently ignored keyword.

_SCENARIO_FIELDS = frozenset({"speed", "dcpa_max", "tlos", "pos_ci95", "vel_ci95"})
_CONFLICT_FIELDS = frozenset({"rpz", "t_lookahead"})
_SIMULATION_FIELDS = frozenset({"dt", "t_max", "done_timeout", "broadcast_interval"})
_GEOMETRY_SLOTS = frozenset({"dpsi", "dcpa", "side", "gs_intr"})
_COMPONENTS = frozenset(
    {"detector", "resolver", "recovery", "navigation", "communication", "surveillance",
     "dynamics", "perf"}
)
_KNOWN_KEYS = (
    _SCENARIO_FIELDS | _CONFLICT_FIELDS | _SIMULATION_FIELDS | _GEOMETRY_SLOTS | _COMPONENTS
)


@dataclass(frozen=True)
class Fixed:
    """One parameter held at ``value`` for every condition."""

    value: Any


@dataclass(frozen=True)
class Sweep:
    """One parameter varied across ``values`` — an output axis, one condition per level.

    ``name`` labels the axis in the results table, defaulting to the declaration key. ``build``
    maps a level onto the value the run actually needs, which is what lets a *component* parameter
    be swept over a scalar: the table reads as numbers while the run receives objects.

        Sweep([1.05, 1.2], build=lambda m: MVP(margin=m), name="margin")

    Without ``build`` the levels are used as-is, so a categorical axis is simply the objects
    themselves (``Sweep([MVP(1.05), VO(1.05)])``) — readable in the table only if they have useful
    ``repr``s, which is the reason ``build`` exists.
    """

    values: tuple[Any, ...]
    name: str | None = None
    build: Callable[[Any], Any] | None = None

    def __init__(
        self,
        values: Sequence[Any],
        name: str | None = None,
        build: Callable[[Any], Any] | None = None,
    ) -> None:
        levels = tuple(values)
        if not levels:
            raise ValueError("a Sweep needs at least one level; got an empty sequence")
        object.__setattr__(self, "values", levels)
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "build", build)

    def resolve(self, level: Any) -> Any:
        """The value a run receives for ``level`` — through ``build`` when one was given."""
        return level if self.build is None else self.build(level)


Axis = Fixed | Sweep


# --- backends -------------------------------------------------------------------------------


@dataclass(frozen=True)
class MC:
    """Plain Monte Carlo: ``n_encounters`` independent encounters per condition.

    Carries exactly its own estimator's parameters, so an illegal pairing (particles on MC, an
    encounter count on IPS) is unrepresentable rather than validated.
    """

    n_encounters: int

    def __post_init__(self) -> None:
        if self.n_encounters <= 0:
            raise ValueError(f"n_encounters must be > 0, got {self.n_encounters}")


@dataclass(frozen=True)
class IPS:
    """Rare-event interacting particle system: fixed-effort multilevel splitting (ADR 0017).

    ``shells`` is the decreasing sequence of running-minimum separations to split on, ending at the
    rare boundary (``rpz`` for loss of separation). They are **explicit and per-study**, not
    derived: ADR 0017 accepts fixed shells with hand-tuned spacing and defers adaptive levels, and
    a ladder spaced too aggressively collapses (reported as ``n_collapsed``, never as a real zero).

    ``reps`` is structural, not a convenience: particles within one run interact through
    resampling, so a valid interval comes only from independent replications (ADR 0017 §5).
    """

    shells: tuple[float, ...]
    n_particles: int
    reps: int

    def __init__(self, shells: Sequence[float], n_particles: int, reps: int) -> None:
        ladder = tuple(float(s) for s in shells)
        if not ladder:
            raise ValueError("IPS needs at least one shell")
        if any(b >= a for a, b in zip(ladder, ladder[1:], strict=False)):
            raise ValueError(f"shells must be strictly decreasing, got {ladder}")
        if n_particles <= 0 or reps <= 0:
            raise ValueError(f"require n_particles > 0 and reps > 0, got {n_particles}, {reps}")
        object.__setattr__(self, "shells", ladder)
        object.__setattr__(self, "n_particles", n_particles)
        object.__setattr__(self, "reps", reps)


Backend = MC | IPS


# --- the methods bundle ---------------------------------------------------------------------


@dataclass(frozen=True)
class Methods:
    """The CDR stack and the environment it runs in — everything that is not a swept parameter.

    A plain bundle of defaults, not a new abstraction: each field is passed straight through to the
    estimator, and any of them can be overridden per condition by declaring the same name as an
    axis. ``perf`` defaults are the caller's business; the airframe defaults to the fleet's
    multirotor when ``dynamics`` is ``None`` (ADR 0007).
    """

    detector: ConflictDetector
    resolver: ConflictResolver | None = None
    recovery: RecoveryCriterion | None = None
    navigation: NavigationModel | None = None
    communication: CommunicationModel | None = None
    surveillance: SurveillanceModel | None = None
    dynamics: Dynamics | None = None
    perf: Performance | None = None


# --- conditions -----------------------------------------------------------------------------


@dataclass(frozen=True)
class Condition:
    """One cell of the study: the swept levels that label it, and the values the run receives."""

    levels: tuple[tuple[str, Any], ...]  # (column name, level) per swept axis, declaration order
    values: tuple[tuple[str, Any], ...]  # key -> resolved value, for every declared parameter

    @property
    def label(self) -> dict[str, Any]:
        """The swept levels as a dict — the identifying columns of this row."""
        return dict(self.levels)

    def get(self, key: str, default: Any = None) -> Any:
        return dict(self.values).get(key, default)


def expand(independent_vars: Mapping[str, Axis]) -> tuple[Condition, ...]:
    """The cross-product of the declared :class:`Sweep` axes, in declaration order.

    Every parameter appears in each condition's ``values``; only the swept ones appear in
    ``levels``, so an all-:class:`Fixed` declaration is the single-cell case, not a separate path.
    """
    unknown = sorted(set(independent_vars) - _KNOWN_KEYS)
    if unknown:
        raise ValueError(
            f"unknown parameter(s) {unknown}. Declarable: {sorted(_KNOWN_KEYS)}"
        )
    swept = [(key, axis) for key, axis in independent_vars.items() if isinstance(axis, Sweep)]
    fixed = [(key, axis.value) for key, axis in independent_vars.items()
             if isinstance(axis, Fixed)]

    conditions: list[Condition] = []
    for combo in itertools.product(*(axis.values for _, axis in swept)):
        levels = tuple(
            (axis.name or key, level) for (key, axis), level in zip(swept, combo, strict=True)
        )
        resolved = tuple(
            (key, axis.resolve(level)) for (key, axis), level in zip(swept, combo, strict=True)
        )
        conditions.append(Condition(levels=levels, values=tuple(fixed) + resolved))
    return tuple(conditions)


# --- running one condition ------------------------------------------------------------------


def _config_for(condition: Condition, base: Config, n_encounters: int) -> Config:
    """``base`` with this condition's config-valued parameters substituted in."""
    values = dict(condition.values)
    scenario = dataclasses.replace(
        base.scenario, **{k: v for k, v in values.items() if k in _SCENARIO_FIELDS}
    )
    conflict = dataclasses.replace(
        base.conflict, **{k: v for k, v in values.items() if k in _CONFLICT_FIELDS}
    )
    simulation = dataclasses.replace(
        base.simulation, **{k: v for k, v in values.items() if k in _SIMULATION_FIELDS}
    )
    return dataclasses.replace(
        base, n_encounters=n_encounters, scenario=scenario, conflict=conflict,
        simulation=simulation,
    )


def _resolved_methods(condition: Condition, methods: Methods) -> Methods:
    """``methods`` with any component this condition declares as an axis substituted in."""
    values = dict(condition.values)
    overrides = {k: v for k, v in values.items() if k in _COMPONENTS}
    return dataclasses.replace(methods, **overrides) if overrides else methods


def _run_mc(condition: Condition, base: Config, methods: Methods, backend: MC,
            seed: int) -> IPRResult:
    """One MC cell: ``estimate_ipr`` over this condition's config, components and geometry pins."""
    cfg = _config_for(condition, base, backend.n_encounters)
    m = _resolved_methods(condition, methods)
    geometry = {k: v for k, v in dict(condition.values).items() if k in _GEOMETRY_SLOTS}
    return estimate_ipr(
        dataclasses.replace(cfg, seed=seed),
        m.perf if m.perf is not None else _require_perf(),
        m.detector, m.resolver, m.recovery, m.navigation, m.communication, m.surveillance,
        dynamics=m.dynamics,
        **geometry,
    )


def _run_ips(condition: Condition, base: Config, methods: Methods, backend: IPS,
             seed: int) -> RareEventEstimate:
    """One IPS cell: split the *same* environment MC just ran, over this condition's shells.

    ``build_initial`` samples one geometry per particle from that particle's own seed — the initial
    cloud is drawn from the encounter distribution MC integrates over (ADR 0017 §4), which is what
    keeps the two backends comparable. The forward CNS streams are spawned per particle per level
    inside :func:`~opencdarr.ips.ips_once`, not here.
    """
    cfg = _config_for(condition, base, backend.n_particles)
    m = _resolved_methods(condition, methods)
    geometry = {k: v for k, v in dict(condition.values).items() if k in _GEOMETRY_SLOTS}
    perf = m.perf if m.perf is not None else _require_perf()

    def build_initial(seq: np.random.SeedSequence) -> Particle:
        own, intr = sample_pairwise(
            generator(seq),
            speed=cfg.scenario.speed, dcpa_max=cfg.scenario.dcpa_max, tlos=cfg.scenario.tlos,
            rpz=cfg.conflict.rpz, pos_ci95=cfg.scenario.pos_ci95,
            vel_ci95=cfg.scenario.vel_ci95, **geometry,
        )
        agents = [Agent(own, perf, dynamics=m.dynamics), Agent(intr, perf, dynamics=m.dynamics)]
        env = build_env(
            agents, rpz=cfg.conflict.rpz, t_lookahead=cfg.conflict.t_lookahead,
            dt=cfg.simulation.dt, detector=m.detector, resolver=m.resolver, recovery=m.recovery,
            navigation=m.navigation, communication=m.communication, surveillance=m.surveillance,
            t_max=cfg.simulation.t_max, done_timeout=cfg.simulation.done_timeout,
        )
        return Particle(env=env, state=env.initial_state(agents))

    return estimate_rare_prob(
        build_initial, backend.shells,
        n_particles=backend.n_particles, reps=backend.reps, seed=seed,
    )


def _require_perf() -> Performance:
    raise ValueError(
        "no aircraft performance given: set Methods(perf=...) or declare 'perf' as an axis"
    )


# --- cache identity -------------------------------------------------------------------------
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
    import hashlib
    import inspect

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
    if inspect_isfunction(value):
        return _function_identity(value)
    if isinstance(value, type):
        return f"{_qualified(value)}#{_source_digest(value)}"
    if hasattr(value, "__dict__"):
        return _instance_identity(value)
    raise CacheIdentityError(
        f"no cache identity for {value!r} (type {type(value).__name__}). "
        f"Give it a `cache_id` attribute if it is stable."
    )


def inspect_isfunction(value: Any) -> bool:
    """Whether ``value`` is a plain Python function (so it has a closure worth inspecting)."""
    import inspect

    return inspect.isfunction(value)


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
    """
    return {
        "config": dataclasses.asdict(config),
        "methods": {
            f.name: identity(getattr(methods, f.name)) for f in dataclasses.fields(methods)
        },
        "geometry": {
            k: identity(v) for k, v in condition.values if k in _GEOMETRY_SLOTS
        },
        "backend": identity(backend),
    }


# --- results --------------------------------------------------------------------------------


@dataclass(frozen=True)
class StudyResult:
    """One row per condition, plus the raw estimator result behind each.

    The columns adapt to the backend, because the two estimators do not report the same things: MC
    gives counts and a Wilson interval on a design-fixed denominator, IPS gives a replicated
    probability with a log-space interval plus the per-shell survival and a collapse count. Forcing
    them into one schema would mean inventing a value for whichever half is absent.
    """

    backend: Backend
    seed: int
    conditions: tuple[Condition, ...]
    results: tuple[Any, ...]  # IPRResult per condition, or RareEventEstimate per condition
    axes: tuple[str, ...]  # the swept column names, declaration order
    card_path: Path | None = None  # the provenance card, when one was written

    def records(self) -> list[dict[str, Any]]:
        """One dict per condition: its swept levels, then the backend's metrics."""
        return [
            {**condition.label, **_metrics(result)}
            for condition, result in zip(self.conditions, self.results, strict=True)
        ]

    def frame(self) -> Any:
        """:meth:`records` as a ``pandas.DataFrame``.

        ``pandas`` is imported here rather than at module scope: it is an optional extra (like
        ``matplotlib`` for :mod:`opencdarr.viz` and ``joblib`` for :mod:`opencdarr.parallel`), so a
        plain install stays numpy + pyyaml and :meth:`records` works without it.
        """
        try:
            import pandas as pd  # type: ignore[import-untyped]  # optional extra, no stubs
        except ModuleNotFoundError as exc:  # pragma: no cover - depends on the install
            raise ModuleNotFoundError(
                "frame() needs pandas (an optional extra: pip install 'opencdarr[examples]'). "
                "records() returns the same rows as plain dicts with no extra dependency."
            ) from exc
        return pd.DataFrame(self.records())

    def plot(self, metric: str = "p_los", *, ax: Any = None, log: bool | None = None) -> Any:
        """The response curve: the first swept axis on x, the rest as one line each, CI as a band.

        Layout comes from the axis roles, so nothing has to be restated: the first :class:`Sweep`
        becomes x, any further ones become the series, and the interval already in the table
        becomes a shaded band. ``log`` defaults to a log y-axis for :class:`IPS` (a rare-event
        probability spans decades) and linear for :class:`MC`.

        Returns the :class:`~matplotlib.figure.Figure`, as :func:`opencdarr.viz.plot_pairwise`
        does, so a caller can save it or keep tweaking. Deliberately plain — no grid, no figure
        title — on the house convention that a figure carries axes and a legend, and the prose
        carries the rest.
        """
        import matplotlib.pyplot as plt

        if not self.axes:
            raise ValueError(
                "nothing to plot against: every parameter is Fixed, so there is one condition. "
                "Declare at least one Sweep, or read the single row from records()."
            )
        rows = self.records()
        if metric not in rows[0]:
            raise KeyError(f"no metric {metric!r} on this backend; have {sorted(rows[0])}")

        x_axis, *series_axes = self.axes
        fig = plt.figure(figsize=(6.4, 4.0)) if ax is None else ax.get_figure()
        ax = fig.add_subplot(111) if ax is None else ax

        # one line per combination of the remaining swept axes, in first-seen order
        groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
        for row in rows:
            groups.setdefault(tuple(row[a] for a in series_axes), []).append(row)

        band = {f"{metric}_lo", f"{metric}_hi"} <= set(rows[0])
        for key, group in groups.items():
            group = sorted(group, key=lambda r: r[x_axis])
            xs = [r[x_axis] for r in group]
            label = ", ".join(f"{a}={k}" for a, k in zip(series_axes, key, strict=True))
            (line,) = ax.plot(xs, [r[metric] for r in group], marker="o", ms=3, lw=1.6,
                              label=label or None)
            if band:
                ax.fill_between(xs, [r[f"{metric}_lo"] for r in group],
                                [r[f"{metric}_hi"] for r in group],
                                alpha=0.18, color=line.get_color(), lw=0)

        ax.set_xlabel(x_axis)
        ax.set_ylabel(metric)
        if log if log is not None else isinstance(self.backend, IPS):
            ax.set_yscale("log")
        if series_axes:
            ax.legend(frameon=False, loc="best")
        return fig

    def cell(self, **levels: Any) -> Any:
        """The raw estimator result for the condition matching ``levels`` (exactly one must match).

        Raw on purpose: it is the material a metric nobody has written yet would be computed from,
        so it is handed back rather than reduced.
        """
        matches = [
            result for condition, result in zip(self.conditions, self.results, strict=True)
            if all(condition.label.get(k) == v for k, v in levels.items())
        ]
        if len(matches) != 1:
            raise KeyError(
                f"{levels} matches {len(matches)} conditions, expected exactly 1. "
                f"Swept axes are {list(self.axes)}."
            )
        return matches[0]

    def __len__(self) -> int:
        return len(self.conditions)


def _metrics(result: Any) -> dict[str, Any]:
    """The reported columns for one cell, per backend."""
    if isinstance(result, IPRResult):
        lo, hi = result.ci95
        return {
            "p_los": result.p_los,
            "p_los_lo": lo,
            "p_los_hi": hi,
            "ipr": result.ipr,
            "n_los": result.n_los,
            "n_encounters": result.n_encounters,
            "detection_rate": result.detection_rate,
        }
    lo, hi = result.ci
    return {
        "p_los": result.prob,
        "p_los_lo": lo,
        "p_los_hi": hi,
        "n_collapsed": result.n_collapsed,
        "reps": len(result.reps),
    }


# --- provenance -----------------------------------------------------------------------------


def _describe_axes(independent_vars: Mapping[str, Axis]) -> str:
    """The declaration as readable lines: each parameter, its role, and its levels."""
    lines = []
    for key, axis in independent_vars.items():
        if isinstance(axis, Fixed):
            lines.append(f"- `{key}`: Fixed({axis.value!r})")
        else:
            label = f" as `{axis.name}`" if axis.name else ""
            built = " via build()" if axis.build is not None else ""
            lines.append(f"- `{key}`: Sweep({list(axis.values)!r}){label}{built}")
    return "\n".join(lines)


def _write_card(
    result: StudyResult,
    independent_vars: Mapping[str, Axis],
    methods: Methods,
    base_config: Config,
    card_dir: Path,
) -> Path:
    """Write one Markdown card describing the study, and return its path.

    Generalises :func:`opencdarr.experiment.run_one_experiment`'s card from one run to a whole
    sweep: the declaration with each parameter's role, the component identities, the backend, the
    seed, the code fingerprint, and the results table. Component identities are the same strings
    the cache keys on, so a card and a cache entry cannot disagree about what was run — and
    identity is best-effort here (a card should still be written for an unkeyable component), so a
    failure is recorded as such rather than aborting the write.
    """
    card_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S_%fZ")
    path = card_dir / f"study_{stamp}_seed{result.seed}.md"

    def described(value: Any) -> str:
        try:
            return identity(value)
        except CacheIdentityError as exc:
            return f"(no stable identity: {exc.args[0].split(chr(46))[0]})"

    config_yaml = yaml.safe_dump(dataclasses.asdict(base_config), sort_keys=False)
    rows = result.records()
    columns = list(rows[0]) if rows else []
    table = "\n".join(
        ["| " + " | ".join(columns) + " |", "|" + "---|" * len(columns)]
        + ["| " + " | ".join(_format(row[c]) for c in columns) + " |" for row in rows]
    )
    path.write_text(
        f"# Study {stamp}\n\n"
        f"- backend: `{result.backend}`\n"
        f"- seed: {result.seed}\n"
        f"- conditions: {len(result.conditions)}\n"
        f"- swept axes: {list(result.axes)}\n"
        f"- code_hash: {cache.code_fingerprint()}\n\n"
        f"## Declaration\n\n{_describe_axes(independent_vars)}\n\n"
        f"## Methods\n\n"
        + "\n".join(
            f"- {f.name}: `{described(getattr(methods, f.name))}`"
            for f in dataclasses.fields(methods)
        )
        + f"\n\n## Base config\n\n```yaml\n{config_yaml}```\n"
        f"\n## Results\n\n{table}\n"
    )
    return path


def _format(value: Any) -> str:
    return f"{value:.6g}" if isinstance(value, float) else str(value)


# --- the entry point ------------------------------------------------------------------------


def _run_one(
    condition: Condition,
    base_config: Config,
    methods: Methods,
    backend: Backend,
    seed: int,
    cache_dir: Path | None,
) -> Any:
    """One condition, through the cache when one is configured.

    Module-level (not a closure) so a parallel worker can pickle it. The cell is keyed on its
    resolved config, its component identities, its geometry pins, the backend and the seed — plus
    the library code fingerprint, which :func:`opencdarr.cache.run_key` adds.
    """
    n = backend.n_encounters if isinstance(backend, MC) else backend.n_particles
    config = dataclasses.replace(_config_for(condition, base_config, n), seed=seed)

    def compute() -> Any:
        if isinstance(backend, MC):
            return _run_mc(condition, base_config, methods, backend, seed)
        return _run_ips(condition, base_config, methods, backend, seed)

    if cache_dir is None:
        return compute()
    key = cache.run_key(_cache_params(condition, config, methods, backend), seed=seed)
    return cache.load_or_run(key, compute, cache_dir=cache_dir)


def run_study(
    independent_vars: Mapping[str, Axis],
    *,
    methods: Methods,
    backend: Backend,
    base_config: Config,
    seed: int = 0,
    cache: bool | CacheConfig = False,
    n_jobs: int = 1,
    card_dir: Path | None = None,
) -> StudyResult:
    """Run every condition of ``independent_vars`` on ``backend`` and tabulate the results.

    ``base_config`` supplies the parameters no axis declares (and the numerics); anything declared
    as :class:`Fixed` or :class:`Sweep` overrides it. ``seed`` is the reproducibility root, shared
    by both backends and common to every condition — so two conditions differ by their declared
    levels and nothing else, which is what makes a sweep a comparison rather than a set of runs.

    ``cache=True`` (or a :class:`CacheConfig`) stores one entry per condition, so extending a sweep
    re-runs only the new cells. It is **off by default** and will raise
    :class:`CacheIdentityError` rather than key a component it cannot identify — see
    :func:`identity`.

    ``n_jobs`` spreads the conditions over processes (joblib's convention: ``-1`` is every core).
    Conditions are independent by construction — each is its own seeded fan-out — so this is a pure
    scheduling choice with no effect on the numbers. It needs the ``parallel`` extra, and the
    components must be picklable, which rules out a lambda held on a component instance.

    ``card_dir`` writes a provenance card per study; ``None`` (default) writes nothing.
    """
    conditions = expand(independent_vars)
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
    else:
        parallel_cls, delayed = _joblib()
        results = tuple(
            parallel_cls(n_jobs=workers)(
                delayed(_run_one)(c, base_config, methods, backend, seed, cache_dir)
                for c in conditions
            )
        )

    result = StudyResult(
        backend=backend, seed=seed, conditions=conditions, results=results, axes=axes,
        card_path=None,
    )
    if card_dir is None:
        return result
    return dataclasses.replace(
        result, card_path=_write_card(result, independent_vars, methods, base_config, card_dir)
    )
