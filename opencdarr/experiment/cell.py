"""Running one condition — where a declaration becomes agents, a builder, and a backend call."""

from __future__ import annotations

import dataclasses
from collections.abc import Sequence

import numpy as np

from opencdarr.autopilot import WaypointAutopilot
from opencdarr.cns.broadcast import schedule_for
from opencdarr.config import Config
from opencdarr.crr import ProbabilisticFTR
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.estimate.ips import Particle, RareEventEstimate, estimate_rare_prob
from opencdarr.estimate.montecarlo import (
    MonteCarloEstimate,
    combine_p_los,
    estimate_p_los,
)
from opencdarr.estimate.parallel import _joblib
from opencdarr.estimate.parallel import estimate_rare_prob as estimate_rare_prob_parallel
from opencdarr.experiment.backends import IPS, MC, Backend
from opencdarr.experiment.conditions import Condition
from opencdarr.experiment.declaration import (
    _COMPONENTS,
    _CONFLICT_FIELDS,
    _GEOMETRY_SLOTS,
    _SCENARIO_FIELDS,
    _SIMULATION_FIELDS,
)
from opencdarr.experiment.methods import Methods
from opencdarr.fleet import Agent, EncounterBuilder, build_env
from opencdarr.mission import Mission
from opencdarr.performance import Performance
from opencdarr.rng import children, generator, root_seed_sequence
from opencdarr.scenario import Scenario
from opencdarr.scenario.base import FleetScenario


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
            stop_within=cfg.simulation.stop_within,
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
