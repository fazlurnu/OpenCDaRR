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
:mod:`opencdarr.registry`. It is kept because a config file is committable and diffable in a way a
Python call is not, so ``config + seed -> result`` stays reproducible without writing code; it is
*not* a second implementation, and there is one card writer for both.
"""

from __future__ import annotations

import dataclasses
import hashlib
import inspect
import itertools
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from opencdarr import cache, registry
from opencdarr.autopilot import WaypointAutopilot
from opencdarr.cache import DEFAULT_CACHE_DIR
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import CommunicationModel, NavigationModel, SurveillanceModel
from opencdarr.cns.broadcast import schedule_for
from opencdarr.config import Config
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import ProbabilisticFTR
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.estimate.ips import Particle, RareEventEstimate, estimate_rare_prob
from opencdarr.estimate.montecarlo import (
    MonteCarloEstimate,
    combine_p_los,
    estimate_p_los,
)
from opencdarr.estimate.parallel import _joblib, resolve_jobs
from opencdarr.estimate.parallel import estimate_rare_prob as estimate_rare_prob_parallel
from opencdarr.fleet import Agent, Airframe, EncounterBuilder, build_env
from opencdarr.kinematics import Kinematics
from opencdarr.mission import Mission
from opencdarr.performance import Performance
from opencdarr.rng import children, generator, root_seed_sequence
from opencdarr.scenario import PairwiseEncounter, Scenario
from opencdarr.scenario.base import FleetScenario
from opencdarr.wind import NO_WIND, WindField

# --- what can be declared -------------------------------------------------------------------
# Every key below is wired end to end. The list is deliberately short: it is what the estimators
# actually thread today, not an aspiration. An unknown key fails immediately with this list in the
# message, which is a better contributor experience than a silently ignored keyword.

_SCENARIO_FIELDS = frozenset(
    {"speed", "dcpa_max", "tlos", "pos_ci95", "vel_ci95",
     "pos_ci95_declared", "vel_ci95_declared"}
)
_CONFLICT_FIELDS = frozenset({"rpz", "t_lookahead"})
_SIMULATION_FIELDS = frozenset(
    {"dt", "t_max", "done_timeout", "broadcast_interval", "broadcast_jitter",
     "broadcast_random_phase"}
)
_GEOMETRY_SLOTS = frozenset({"dpsi", "dcpa", "side", "gs_intr"})

# Every field of `Methods`, so declaring one as an axis overrides the bundle per condition.
# `wind` is not a pluggable model but it is a per-run input the bundle carries, so it is swept the
# same way — `wind=Sweep([NO_WIND, WindField.from_met(270, 8)])`.
_COMPONENTS = frozenset(
    {"detector", "resolver", "recovery", "navigation", "communication", "surveillance",
     "kinematics", "perf", "wind", "airframes", "scenario"}
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
    rare boundary (``rpz`` for loss of separation). They are **explicit and per-experiment**, not
    derived: ADR 0017 accepts fixed shells with hand-tuned spacing and defers adaptive levels, and
    a ladder spaced too aggressively collapses (reported as ``n_collapsed``, never as a real zero).

    ``reps`` is structural, not a convenience: particles within one run interact through
    resampling, so a single run's spread understates the real one — only independent replications
    are independent (ADR 0017 §5).

    ``tail`` (default on) flies the final cloud past its first breach to the end of the encounter,
    which is the only way this backend can report ``p_los_ac`` and ``mean_k``: the ladder stops
    each survivor the instant it crosses, so K is 1 and A is 2 there by construction. Switching it
    off leaves those two ``nan`` and changes no other number.
    """

    shells: tuple[float, ...]
    n_particles: int
    reps: int
    tail: bool = True

    def __init__(self, shells: Sequence[float], n_particles: int, reps: int,
                 tail: bool = True) -> None:
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
        object.__setattr__(self, "tail", tail)


Backend = MC | IPS


# --- the methods bundle ---------------------------------------------------------------------


@dataclass(frozen=True)
class Methods:
    """The CDR stack and the environment it runs in — everything that is not a swept parameter.

    A plain bundle of defaults, not a new abstraction: each field is passed straight through to the
    estimator, and any of them can be overridden per condition by declaring the same name as an
    axis. ``perf`` defaults are the caller's business; the airframe defaults to the fleet's
    multirotor when ``kinematics`` is ``None`` (ADR 0007).

    **A mixed fleet is spelled ``airframes``**: one :class:`~opencdarr.fleet.Airframe` per aircraft
    (ownship first), replacing ``perf``/``kinematics`` rather than joining them. The single fields
    are the right shape when every aircraft is the same airframe, which is the ordinary case;
    ``airframes`` is how a declaration says otherwise, and it is what lets a multirotor-versus-
    fixed-wing encounter be *swept* rather than only run once through
    :func:`~opencdarr.fleet.run_fleet`. Bundling ``perf`` with ``kinematics`` also makes a
    mismatched pair unrepresentable in the declaration instead of caught a layer down.

    ``scenario`` is the **encounter model** — a :class:`~opencdarr.scenario.Scenario`, which turns
    one seed into one fleet. It defaults to
    :class:`~opencdarr.scenario.PairwiseEncounter`, so an undeclared experiment is the two-aircraft
    study it always was. Because it is a field of this bundle it is swept like any other component
    — ``scenario=Sweep([4, 8], build=lambda n: CrossingRing(n=n), name="n")`` is a fleet-size axis
    — and because *both* backends build their encounter from it, a ring or a traffic sample reaches
    MC and IPS from one declaration.

    The scenario also carries its own measurement area, so "fill this disc, measure inside that
    one" is a single declaration rather than two that can disagree. Nothing here takes an area
    separately.

    ``wind`` is the odd one out: it is a steady environment input rather than a pluggable model, so
    it has no ABC and lives here for the same reason ``perf`` does — the run needs it and no
    scenario field carries it. It reaches **both** backends; it previously reached neither,
    because :func:`~opencdarr.estimate.montecarlo.estimate_p_los` and
    :func:`~opencdarr.fleet.build_env` accept a ``wind`` this module never passed.
    """

    detector: ConflictDetector
    resolver: ConflictResolver | None = None
    recovery: RecoveryCriterion | None = None
    navigation: NavigationModel | None = None
    communication: CommunicationModel | None = None
    surveillance: SurveillanceModel | None = None
    kinematics: Kinematics | None = None
    perf: Performance | None = None
    wind: WindField = NO_WIND
    airframes: Sequence[Airframe] | None = None
    scenario: Scenario = PairwiseEncounter()

    def __post_init__(self) -> None:
        # One spelling for one thing. ``perf``/``kinematics`` say "every aircraft is this
        # airframe"; ``airframes`` says "here is each one". Both at once has no meaning, so it is
        # refused rather than silently resolved in some order.
        if self.airframes is not None and (self.perf is not None or self.kinematics is not None):
            raise ValueError(
                "give either airframes=[...] (one per aircraft) or perf=/kinematics= (one shared "
                "airframe), not both"
            )


# --- conditions -----------------------------------------------------------------------------


@dataclass(frozen=True)
class Condition:
    """One cell of the experiment: the swept levels labelling it, and the values the run gets."""

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
    """``methods`` with any component this condition declares as an axis substituted in.

    An axis **replaces** the bundle's value for that key, which for the airframe means displacing
    the other spelling rather than colliding with it: ``airframes=Sweep([...])`` over a bundle
    carrying ``perf=M600`` is a declaration that the airframe varies per condition, not a
    contradiction. Without this, the ordinary ``Methods(..., perf=M600)`` would make an airframe
    axis unwritable, because the resolved bundle would carry both spellings and
    :meth:`Methods.__post_init__` refuses that — a check meant for what the *caller* wrote.
    """
    values = dict(condition.values)
    overrides = {k: v for k, v in values.items() if k in _COMPONENTS}
    if not overrides:
        return methods
    if "airframes" in overrides:
        overrides.setdefault("perf", None)
        overrides.setdefault("kinematics", None)
    elif overrides.keys() & {"perf", "kinematics"}:
        overrides.setdefault("airframes", None)
    return dataclasses.replace(methods, **overrides)


def _reads_declared_accuracy(recovery: RecoveryCriterion | None) -> bool:
    """Whether ``recovery`` sizes anything from an aircraft's declared ``pos_ci95``/``vel_ci95``.

    Only :class:`~opencdarr.crr.ProbabilisticFTR` does today (it builds a covariance from them,
    ``crr/probabilistic_ftr.py``); :class:`~opencdarr.crr.FTR` and
    :class:`~opencdarr.crr.PastCPA` are certain-kinematics and ignore both. Kept as one named
    predicate rather than an inline ``isinstance`` so a contributed ci95-reading criterion has a
    single place to be added — the honest limit being that until it is added, such a criterion
    trips :func:`_validate_declared_accuracy_is_read` and has to declare its accuracy some other
    way.
    """
    return isinstance(recovery, ProbabilisticFTR)


def _validate_declared_accuracy_is_read(
    conditions: Sequence[Condition], base: Config, methods: Methods
) -> None:
    """Raise if any condition declares an accuracy that nothing in its stack will read.

    ``pos_ci95``/``vel_ci95`` have exactly two consumers: a
    :class:`~opencdarr.cns.base.NavigationModel`, which draws the error from them, and
    :class:`~opencdarr.crr.ProbabilisticFTR`, which sizes its uncertainty from them. With neither
    in the stack the fields are stamped onto every :class:`~opencdarr.state.AircraftState`, carried
    through the whole run, and never read — so ``pos_ci95=Sweep([0, 10, 20, 40])`` returns four
    **bit-identical** cells and the table reads "navigation accuracy has no effect on safety". That
    is a publishable-looking null result produced by a no-op, and it has already happened once
    (``vault/todo-might-be-a-bug.md`` §7: a comm-outage sweep flat at P(LoS) = 0 because the
    declaration was missing ``navigation``; adding it turned the same sweep into 0.060 -> 0.437).

    Deliberately a *contradiction* check and not an implication: a declared accuracy read only by
    :class:`~opencdarr.crr.ProbabilisticFTR`, with no noise model present, is a valid — if unusual
    — configuration, so neither field may quietly imply a navigation model (§7's own ruling).

    Checked per condition after :func:`expand`, because ``pos_ci95``, ``navigation`` and
    ``recovery`` can each be swept independently: a sweep over navigation models that includes
    ``None`` is legitimate everywhere except where the accuracy is non-zero.
    """
    for condition in conditions:
        scenario = _config_for(condition, base, base.n_encounters).scenario
        if scenario.pos_ci95 == 0.0 and scenario.vel_ci95 == 0.0:
            continue
        resolved = _resolved_methods(condition, methods)
        if resolved.navigation is not None or _reads_declared_accuracy(resolved.recovery):
            continue
        where = f" at {condition.label}" if condition.levels else ""
        raise ValueError(
            f"pos_ci95={scenario.pos_ci95}, vel_ci95={scenario.vel_ci95} declared{where}, but "
            "nothing reads them: this condition has no navigation model to draw the error and no "
            "ci95-reading recovery criterion. Every cell would come out identical. Pass "
            "methods=Methods(..., navigation=GnssNavigation()), or set the accuracy to 0.0 if a "
            "perfect sensor is what you meant."
        )


def _validate_splittable(
    conditions: Sequence[Condition], methods: Methods, backend: Backend
) -> None:
    """Raise if the splitting backend is pointed at a scenario that cannot support it.

    :meth:`~opencdarr.scenario.Scenario.supports_splitting` names the failure this prevents: on an
    open-ended scenario the running minimum separation stops discriminating between particles, so
    the ladder burns its whole budget and reports a number near 1. The method promises the
    combination "can fail at declaration time rather than after the run" — this is that check.
    Per condition, because ``scenario`` is itself sweepable: a sweep mixing a ring with a stream
    should fail before its first cell, not at the stream.
    """
    if not isinstance(backend, IPS):
        return
    for condition in conditions:
        scenario = _scenario_for(condition, _resolved_methods(condition, methods))
        if scenario.supports_splitting():
            continue
        where = f" at {condition.label}" if condition.levels else ""
        raise ValueError(
            f"{type(scenario).__name__} declares supports_splitting() = False{where}, so the "
            "IPS backend cannot use it: splitting needs the running minimum separation to "
            "discriminate between particles. Run this scenario on the MC backend."
        )


def _scenario_for(condition: Condition, m: Methods) -> Scenario:
    """This cell's scenario, with any declared geometry slot pinned onto it.

    ``with_pins`` is what keeps ``dpsi=Sweep([...])`` a real axis: the pins are declared per
    condition, so the scenario is rebuilt per condition rather than carrying one fixed geometry.
    A scenario with no slots refuses them, so declaring ``dpsi`` over a ring fails here instead of
    silently doing nothing.
    """
    pins = {k: v for k, v in dict(condition.values).items() if k in _GEOMETRY_SLOTS}
    return m.scenario.with_pins(**pins)


def _agents_from(fleet: FleetScenario, m: Methods) -> list[Agent]:
    """Pair an airframe-neutral fleet with the airframes that fly it.

    The scenario says *what geometry*; ``Methods`` says *what aircraft*. Keeping the two apart is
    what lets one ring be flown by a multirotor fleet, a fixed-wing fleet or a mixed one without
    the scenario knowing. A goal becomes a waypoint mission; ``None`` leaves the aircraft holding
    its cruise, which is what a pairwise encounter wants — there is nowhere to arrive.
    """
    perf = _base_perf(m)
    agents: list[Agent] = []
    for k, (state, goal) in enumerate(fleet):
        autopilot = (
            None if goal is None
            else WaypointAutopilot(Mission(goto=goal), cruise_airspeed=state.gs)
        )
        if m.airframes is None:
            agents.append(Agent(state, perf, kinematics=m.kinematics, autopilot=autopilot))
        else:
            agents.append(m.airframes[k].agent(state, autopilot))
    return agents


def _encounter_builder(condition: Condition, m: Methods) -> EncounterBuilder:
    """This cell's encounter model, built once and used by *both* backends.

    The single place a condition becomes a fleet. Keeping it here rather than inline in each
    backend is what stops the two describing the same encounter differently: they are compared
    cell for cell, so a difference in how they build it would read as a difference between the
    estimators (ADR 0017 §4 — the initial cloud is drawn from the distribution MC integrates over).
    """
    scenario = _scenario_for(condition, m)

    def build(rng: np.random.Generator, config: Config) -> list[Agent]:
        return _agents_from(scenario.draw(rng, config), m)

    return build


def _run_mc(condition: Condition, base: Config, methods: Methods, backend: MC,
            seed: int, jobs: int = 1) -> MonteCarloEstimate:
    """One MC cell: ``estimate_p_los`` over this condition's config, components and scenario.

    ``jobs`` above 1 splits the encounter fan-out into contiguous seed slices and pools the counts
    (:func:`~opencdarr.estimate.montecarlo.combine_p_los`). Each slice is
    ``children(root, lo, hi)`` of the *same* tree the serial run walks, and the parts are
    combined in submission order, so the pooled
    result is the serial one exactly — not merely an equivalent sample.
    """
    cfg = dataclasses.replace(_config_for(condition, base, backend.n_encounters), seed=seed)
    m = _resolved_methods(condition, methods)
    build = _encounter_builder(condition, m)
    models = (m.detector, m.resolver, m.recovery, m.navigation, m.communication, m.surveillance)

    n = backend.n_encounters
    area = _scenario_for(condition, m).measurement_area()
    if jobs <= 1 or n < 2:
        return estimate_p_los(build, cfg, *models, wind=m.wind, area=area)

    root = root_seed_sequence(cfg.seed)
    bounds = [(n * i // jobs, n * (i + 1) // jobs) for i in range(jobs)]
    parallel_cls, delayed = _joblib()
    parts = parallel_cls(n_jobs=jobs)(
        delayed(estimate_p_los)(build, cfg, *models, wind=m.wind, area=area,
                                seqs=children(root, lo, hi))
        for lo, hi in bounds if hi > lo
    )
    return combine_p_los(list(parts))


def _run_ips(condition: Condition, base: Config, methods: Methods, backend: IPS,
             seed: int, jobs: int = 1) -> RareEventEstimate:
    """One IPS cell: split the *same* environment MC just ran, over this condition's shells.

    ``build_initial`` builds one encounter per particle from that particle's own seed, through the
    *same* :func:`_encounter_builder` the MC cell uses — the initial cloud is drawn from the
    encounter distribution MC integrates over (ADR 0017 §4), which is what keeps the two backends
    comparable, and sharing the builder is what keeps that true for a fleet as well as for a pair.
    The forward CNS streams are spawned per particle per level inside
    :func:`~opencdarr.estimate.ips.ips_once`, not here.
    """
    cfg = _config_for(condition, base, backend.n_particles)
    m = _resolved_methods(condition, methods)
    build = _encounter_builder(condition, m)

    def build_initial(seq: np.random.SeedSequence) -> Particle:
        geom_rng = generator(seq)
        agents = build(geom_rng, cfg)
        env = build_env(
            agents, rpz=cfg.conflict.rpz, t_lookahead=cfg.conflict.t_lookahead,
            dt=cfg.simulation.dt, detector=m.detector, resolver=m.resolver, recovery=m.recovery,
            navigation=m.navigation, communication=m.communication, surveillance=m.surveillance,
            t_max=cfg.simulation.t_max, done_timeout=cfg.simulation.done_timeout,
            wind=m.wind, area=_scenario_for(condition, m).measurement_area(),
            # the transmit timing, which this call omitted entirely: build_env then fell back to
            # the 1 s default, so a declared broadcast_interval reached MC and was silently
            # dropped by IPS. Built through the same schedule_for MC uses, so the two cannot
            # drift apart again (the particle's broadcast stream comes from ips._streams).
            schedule=schedule_for(
                len(agents), cfg.simulation.broadcast_interval, geom_rng,
                jitter=cfg.simulation.broadcast_jitter,
                random_phase=cfg.simulation.broadcast_random_phase,
            ),
        )
        return Particle(env=env, state=env.initial_state(agents))

    if jobs > 1:
        # ADR 0018: shard each level across the workers, so the usable core count stops being
        # capped by `reps` — which is a statistical choice, not a hardware one.
        return estimate_rare_prob_parallel(
            build_initial, backend.shells,
            n_particles=backend.n_particles, reps=backend.reps, seed=seed, tail=backend.tail,
            n_jobs=jobs,
        )
    return estimate_rare_prob(
        build_initial, backend.shells,
        n_particles=backend.n_particles, reps=backend.reps, seed=seed, tail=backend.tail,
    )


def _base_perf(m: Methods) -> Performance:
    """The ``perf`` the estimator still takes positionally, even on the mixed path.

    ``airframes`` overrides it per aircraft, so on that path this is only the value the signature
    needs; the ownship's is the honest one to hand over.
    """
    if m.perf is not None:
        return m.perf
    if m.airframes:
        return m.airframes[0].perf
    return _require_perf()


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


# --- results --------------------------------------------------------------------------------


@dataclass(frozen=True)
class ExperimentResult:
    """One row per condition, plus the raw estimator result behind each.

    The columns adapt to the backend, because the two estimators do not report the same things: MC
    gives counts on a design-fixed denominator and a median achieved separation, IPS gives a
    replicated probability plus the per-shell survival and a collapse count. Forcing them into one
    schema would mean inventing a value for whichever half is absent.
    """

    backend: Backend
    seed: int
    conditions: tuple[Condition, ...]
    results: tuple[Any, ...]  # MonteCarloEstimate or RareEventEstimate, one per condition
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
        ``matplotlib`` for :mod:`opencdarr.viz` and ``joblib`` for
        :mod:`opencdarr.estimate.parallel`), so a
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

    def plot(self, metric: str = "p_los_run", *, ax: Any = None, log: bool | None = None) -> Any:
        """The response curve: the first swept axis on x, the rest as one line each.

        Layout comes from the axis roles, so nothing has to be restated: the first :class:`Sweep`
        becomes x and any further ones become the series. ``log`` defaults to a log y-axis for
        :class:`IPS` (a rare-event probability spans decades) and linear for :class:`MC`.

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

        for key, group in groups.items():
            group = sorted(group, key=lambda r: r[x_axis])
            xs = [r[x_axis] for r in group]
            label = ", ".join(f"{a}={k}" for a, k in zip(series_axes, key, strict=True))
            ax.plot(xs, [r[metric] for r in group], marker="o", ms=3, lw=1.6,
                              label=label or None)

        ax.set_xlabel(x_axis)
        ax.set_ylabel(metric)
        if log if log is not None else isinstance(self.backend, IPS):
            ax.set_yscale("log")
        if series_axes:
            ax.legend(frameon=False, loc="best")
        return fig

    def _repr_html_(self) -> str:
        """Show the results table when a notebook displays this object.

        Display only — the return value of :func:`run_experiment` stays an
        :class:`ExperimentResult`, because the table is a *reduction*: :meth:`cell` reaches the raw
        estimator result behind a row, and with it the per-encounter record every other metric is
        computed from. Handing back a bare frame would make ``pandas`` a hard dependency of the
        entry point and put ``min_seps`` out of reach.

        Falls back to plain text when ``pandas`` is absent, so displaying a result never raises on
        a core install.
        """
        head = (f"<p><code>{self.backend}</code> &middot; seed {self.seed} &middot; "
                f"{len(self.conditions)} condition(s)"
                + (f" &middot; axes {list(self.axes)}" if self.axes else "")
                + "</p>")
        try:
            return head + self.frame()._repr_html_()
        except ModuleNotFoundError:
            rows = "\n".join(str(r) for r in self.records())
            return head + f"<pre>{rows}</pre>"

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
    """The reported columns for one cell, per backend.

    ``median_min_sep`` is an MC column only, and deliberately so: it is an expectation over the
    *whole* encounter population, which is exactly what a splitting estimator cannot give. IPS
    discards the particles that fail to reach a shell and clones the survivors, so its cloud is a
    sample of the rare set, not of the population — a median over it would be the median given
    near-LoS, silently mislabelled. (Conditional-on-the-rare-set quantities *are* available from
    IPS; item 8 of the build order is where they belong.)
    """
    if isinstance(result, MonteCarloEstimate):
        return {
            "p_los_ac": result.p_los_ac,
            "p_los_run": result.p_los_run,
            "mean_k": result.mean_k,
            "median_min_sep": result.median_min_sep,
            "n_los": result.n_los,
            "n_encounters": result.n_encounters,
            "detection_rate": result.detection_rate,
        }
    return {
        # the ladder gives the per-run probability natively; the other two come from the tail leg
        "p_los_ac": result.p_los_ac,
        "p_los_run": result.p_los_run,
        "mean_k": result.mean_k,
        "n_lineages": result.n_lineages,
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
    result: ExperimentResult,
    independent_vars: Mapping[str, Axis],
    methods: Methods,
    base_config: Config,
    card_dir: Path,
) -> Path:
    """Write one Markdown card describing the experiment, and return its path.

    Generalises :func:`opencdarr.experiment.run_one_experiment`'s card from one run to a whole
    sweep: the declaration with each parameter's role, the component identities, the backend, the
    seed, the code fingerprint, and the results table. Component identities are the same strings
    the cache keys on, so a card and a cache entry cannot disagree about what was run — and
    identity is best-effort here (a card should still be written for an unkeyable component), so a
    failure is recorded as such rather than aborting the write.
    """
    card_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S_%fZ")
    path = card_dir / f"{stamp}_seed{result.seed}.md"

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
        f"# Experiment {stamp}\n\n"
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

    The all-:class:`Fixed`, one-condition case of :func:`run_experiment`, with the components named
    as strings and resolved through :mod:`opencdarr.registry`. Nothing is declared as an axis, so
    the result has one row; ``res.cell()`` reaches the
    :class:`~opencdarr.estimate.montecarlo.MonteCarloEstimate` behind it.

    Kept as its own entry point because a config file is *committable and diffable* in a way a
    Python call is not — ``config + seed + code-hash -> result`` without writing code. It is a thin
    wrapper, not a second implementation: the estimator, the cache and the card are the same ones
    :func:`run_experiment` uses, so the two cannot drift.

    The registry is the limit, not this function: only the names
    :mod:`opencdarr.registry` knows are reachable from a file. Pass instances to
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
