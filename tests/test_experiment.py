"""The experiment runner: declare what varies, get one row per condition
(``opencdarr/experiment.py``).

Two groups:

1. **declaration mechanics** — the cross-product, the axis roles, and failing fast on a parameter
   that is not wired (cheap, no simulation);
2. **the release gate** — a contributed resolver *and* a contributed airframe, run under ``MC`` and
   then ``IPS`` from the *same* ``Methods`` object, asserting both backends actually used them.
   That equivalence is the v1.0.2 -> v1.0.3 promise, and it is the property that was silently false
   before plain MC was moved onto the fleet environment.
"""

from __future__ import annotations

import dataclasses
from collections.abc import Sequence
from pathlib import Path

import pytest

from opencdarr import geo
from opencdarr.cache import code_fingerprint
from opencdarr.cd import StateBased
from opencdarr.cns import Comm, GnssNavigation
from opencdarr.cns.communication import constant_latency
from opencdarr.cns.noise_distributions import make_mixture_gaussian
from opencdarr.config import (
    Config,
    ConflictConfig,
    MethodsConfig,
    ScenarioConfig,
    SimulationConfig,
)
from opencdarr.cr import MVP, VO
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr import PastCPA
from opencdarr.estimator import IPRResult
from opencdarr.experiment import (
    IPS,
    MC,
    CacheConfig,
    CacheIdentityError,
    Fixed,
    Methods,
    Sweep,
    expand,
    identity,
    run_experiment,
    run_one_experiment,
)
from opencdarr.ips import RareEventEstimate
from opencdarr.kinematics import Kinematics, MotionCommand
from opencdarr.kinematics.base import odometry_update
from opencdarr.performance import M600, Performance
from opencdarr.rng import generator, root_seed_sequence
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField


def _base() -> Config:
    return Config(
        seed=0,
        n_encounters=1,  # overridden per backend
        scenario=ScenarioConfig("M600", 10.2889, 50.0, 90.0),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(0.5, 250.0, 10.0),
    )


def _methods(**overrides: object) -> Methods:
    base = Methods(detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(True),
                   navigation=GnssNavigation(), perf=M600)
    return dataclasses.replace(base, **overrides) if overrides else base


# a pinned, noisy 90-degree crossing: small enough to run fast, noisy enough that LoS is reachable
_PINNED = {"dpsi": Fixed(90.0), "dcpa": Fixed(0.0), "pos_ci95": Fixed(40.0),
           "vel_ci95": Fixed(4.0)}


# --- declaration mechanics ------------------------------------------------------------------


def test_all_fixed_is_the_single_cell_case() -> None:
    """An all-``Fixed`` declaration is one condition, not a special path."""
    conditions = expand({"dpsi": Fixed(90.0), "pos_ci95": Fixed(10.0)})
    assert len(conditions) == 1
    assert conditions[0].levels == ()  # nothing swept, so no identifying columns
    assert dict(conditions[0].values) == {"dpsi": 90.0, "pos_ci95": 10.0}


def test_sweeps_cross_in_declaration_order() -> None:
    """Two swept axes give their cross-product, ordered by declaration then by level."""
    conditions = expand({"dpsi": Sweep([10.0, 90.0]), "pos_ci95": Sweep([3.0, 10.0])})
    assert [c.label for c in conditions] == [
        {"dpsi": 10.0, "pos_ci95": 3.0},
        {"dpsi": 10.0, "pos_ci95": 10.0},
        {"dpsi": 90.0, "pos_ci95": 3.0},
        {"dpsi": 90.0, "pos_ci95": 10.0},
    ]


def test_build_puts_a_scalar_in_the_table_and_an_object_in_the_run() -> None:
    """``build`` is how a *component* is swept over a readable scalar axis."""
    conditions = expand(
        {"resolver": Sweep([1.05, 1.4], build=lambda m: MVP(margin=m), name="margin")}
    )
    assert [c.label for c in conditions] == [{"margin": 1.05}, {"margin": 1.4}]
    resolvers = [c.get("resolver") for c in conditions]
    assert [type(r).__name__ for r in resolvers] == ["MVP", "MVP"]
    assert [r.margin for r in resolvers] == [1.05, 1.4]


def test_an_unwired_parameter_fails_fast_and_says_what_is_declarable() -> None:
    """A typo or an aspirational axis is rejected at declaration, not silently ignored."""
    with pytest.raises(ValueError, match="unknown parameter"):
        expand({"crossing_angle": Sweep([10.0])})  # it is spelled dpsi
    with pytest.raises(ValueError, match="unknown parameter"):
        expand({"ktheta": Fixed(256)})  # real knob, but not threaded by the runner


def test_degenerate_declarations_are_rejected() -> None:
    with pytest.raises(ValueError, match="at least one level"):
        Sweep([])
    with pytest.raises(ValueError, match="strictly decreasing"):
        IPS(shells=[50.0, 60.0], n_particles=10, reps=2)
    with pytest.raises(ValueError, match="n_encounters must be > 0"):
        MC(n_encounters=0)


def test_a_fixed_axis_overrides_the_base_config() -> None:
    """Declared parameters win over ``base_config``; undeclared ones are inherited."""
    res = run_experiment({**_PINNED, "t_lookahead": Fixed(60.0)}, methods=_methods(),
                    backend=MC(n_encounters=2), base_config=_base(), seed=0)
    assert len(res) == 1
    assert res.records()[0]["n_encounters"] == 2


# --- results shape --------------------------------------------------------------------------


def test_columns_adapt_to_the_backend() -> None:
    """MC reports counts and a Wilson interval; IPS reports replications and collapses.

    They are not forced into one schema, because neither can honestly fill the other's columns —
    there is no ``n_encounters`` for a splitting run and no ``n_collapsed`` for plain MC.
    """
    mc = run_experiment(_PINNED, methods=_methods(), backend=MC(n_encounters=20),
                   base_config=_base(), seed=0).records()[0]
    ips = run_experiment(_PINNED, methods=_methods(),
                    backend=IPS(shells=[70.0, 60.0, 50.0], n_particles=20, reps=2),
                    base_config=_base(), seed=0).records()[0]

    assert {"p_los", "p_los_lo", "p_los_hi"} <= set(mc) & set(ips)  # the shared core
    assert {"n_encounters", "n_los", "ipr", "detection_rate"} <= set(mc)
    assert "n_collapsed" not in mc
    assert {"n_collapsed", "reps"} <= set(ips)
    assert "n_encounters" not in ips


def test_cell_returns_the_raw_estimator_result() -> None:
    """``cell`` hands back the estimator's own value — material for a metric not yet written."""
    res = run_experiment({**_PINNED, "dpsi": Sweep([45.0, 90.0])}, methods=_methods(),
                    backend=MC(n_encounters=20), base_config=_base(), seed=0)
    got = res.cell(dpsi=90.0)
    assert isinstance(got, IPRResult)
    assert got.n_encounters == 20

    with pytest.raises(KeyError, match="matches 0 conditions"):
        res.cell(dpsi=123.0)


def test_frame_matches_records() -> None:
    """``frame()`` is a view of ``records()``, not a second computation."""
    pd = pytest.importorskip("pandas")
    res = run_experiment({**_PINNED, "dpsi": Sweep([45.0, 90.0])}, methods=_methods(),
                    backend=MC(n_encounters=20), base_config=_base(), seed=0)
    frame = res.frame()
    assert isinstance(frame, pd.DataFrame)
    assert list(frame["dpsi"]) == [45.0, 90.0]
    assert frame.to_dict("records") == res.records()


# --- the release gate -----------------------------------------------------------------------


class _Passive(ConflictResolver):
    """A contributed resolver that resolves nothing — it re-commands the current velocity.

    A deliberate test double rather than a plausible algorithm: because it *cannot* avoid, any
    encounter it is given ends in loss of separation, while the built-in MVP prevents nearly all of
    them on the same geometry. That gap is what makes "did the resolver reach the loop?" answerable
    by consequence instead of by introspection, and it needs no tuning to stay a wide gap.
    """

    def resolve(
        self,
        own: AircraftState,
        intruders: Sequence[AircraftState],
        rpz: float,
        preferred: tuple[float, float] | None = None,
    ) -> MotionCommand:
        return MotionCommand.from_track_speed(own.trk, own.gs)


class _Ballistic(Kinematics):
    """A contributed airframe that ignores every command — so no resolver can avoid anything."""

    def step(
        self,
        state: AircraftState,
        command: MotionCommand,
        perf: Performance,
        dt: float,
        wind: WindField = NO_WIND,
    ) -> AircraftState:
        lat, lon = geo.forward(state.lat, state.lon, state.trk, state.gs * dt)
        return dataclasses.replace(
            state, lat=lat, lon=lon, **odometry_update(state, state.gs, dt)
        )


# the two gate tests isolate one seam each, and each isolation rests on the *other* seam working:
# keep a working MVP and break the airframe, or keep the working airframe and break the resolver.
# A single combined test would pass with either one wired and the other silently dropped.
_SAFE = {**_PINNED, "pos_ci95": Fixed(10.0), "vel_ci95": Fixed(1.0)}  # MVP clears this easily
# ... and a near-quiet variant for the resolver half. A resolver that "does nothing" by
# re-commanding its own *perceived* velocity is not the same as no resolver at all: the noisy
# self-fix makes it re-aim slightly wrong every tick, and that random walk does real avoidance
# work. Measured on this geometry, _Passive's LoS rate falls 0.97 -> 0.20 -> 0.00 as pos_ci95 goes
# 1 -> 5 -> 10 m, purely from that drift. (resolver=None stays at 1.00 throughout, because
# CruiseAutopilot holds the *true* initial cruise rather than the noisy fix.) So the resolver
# isolation runs quiet, where "declines to avoid" actually means it.
_QUIET = {**_PINNED, "pos_ci95": Fixed(1.0), "vel_ci95": Fixed(0.1)}
_SHELLS = IPS(shells=[70.0, 60.0, 50.0], n_particles=20, reps=2)


def test_a_contributed_airframe_reaches_both_backends() -> None:
    """**The release gate**, airframe half (v1.1.1) — with a *working* resolver fitted.

    MVP clears this geometry essentially always, so a near-certain loss of separation has exactly
    one explanation: the contributed airframe was flown, and it discards every command MVP issued.
    Plain MC silently dropped ``kinematics`` before it was moved onto the fleet environment, and
    this is the assertion that would have caught it — on the MC side alone, while IPS stayed green.
    """
    fitted = _methods(resolver=MVP(1.05), kinematics=_Ballistic())

    mc = run_experiment(_SAFE, methods=fitted, backend=MC(n_encounters=30),
                   base_config=_base(), seed=0).cell()
    assert isinstance(mc, IPRResult)
    assert mc.p_los > 0.9, "MC ignored the contributed airframe"

    ips = run_experiment(_SAFE, methods=fitted, backend=_SHELLS,
                    base_config=_base(), seed=0).cell()
    assert isinstance(ips, RareEventEstimate)
    assert ips.prob > 0.9, "IPS ignored the contributed airframe"
    assert ips.n_collapsed == 0


def test_a_contributed_resolver_reaches_both_backends() -> None:
    """**The release gate**, resolver half (v1.0.2 -> v1.0.3) — on the *default* airframe.

    The multirotor can manoeuvre, so a near-certain loss of separation again has one explanation:
    the contributed resolver was consulted and it declines to avoid. Compared against MVP on the
    identical geometry and seed, which prevents nearly all of them. Runs on ``_QUIET`` — see the
    note there on why a noisy self-fix would hand the passive resolver accidental avoidance.
    """
    for backend, read in ((MC(n_encounters=20), lambda r: r.p_los), (_SHELLS, lambda r: r.prob)):
        name = type(backend).__name__
        passive = run_experiment(_QUIET, methods=_methods(resolver=_Passive()), backend=backend,
                            base_config=_base(), seed=0).cell()
        mvp = run_experiment(_QUIET, methods=_methods(resolver=MVP(1.05)), backend=backend,
                        base_config=_base(), seed=0).cell()
        # a loose floor on purpose: these run on a deliberately tiny budget (20 encounters / 20
        # particles x 2 reps) to stay fast, and an IPS estimate at that size is genuinely noisy —
        # measured per-replication values of 0.76 and 1.00 for the same setup. The claim being made
        # is "high, not ~0"; tightening this to the number it happens to produce would be tuning
        # the threshold to the run rather than to the property.
        assert read(passive) > 0.5, f"{name} ignored the contributed resolver"
        # MVP's own number is checked only as *much smaller*: at this depth a working resolver can
        # also drive an IPS ladder to collapse, and ADR 0017 §2 is explicit that a collapsed run's
        # zero is not a real zero — so a bare `< 0.1` would pass for the wrong reason.
        assert read(mvp) < read(passive) / 5.0, f"{name} did not apply MVP"


def test_a_categorical_resolver_axis_fans_out() -> None:
    """Sweeping the resolver itself gives one row per algorithm — the built-in benchmark case."""
    res = run_experiment(
        {**_PINNED, "resolver": Sweep([MVP(1.05), VO(1.05)], name="resolver")},
        methods=_methods(), backend=MC(n_encounters=20), base_config=_base(), seed=0,
    )
    names = [type(r["resolver"]).__name__ for r in res.records()]
    assert names == ["MVP", "VO"]


def test_missing_performance_fails_with_a_useful_message() -> None:
    """``perf`` has no sensible default (it is airframe-specific), so its absence must be loud."""
    with pytest.raises(ValueError, match="no aircraft performance"):
        run_experiment(_PINNED, methods=_methods(perf=None), backend=MC(n_encounters=2),
                  base_config=_base(), seed=0)


# --- cache identity -------------------------------------------------------------------------
# A cache may only save time, never change a result. That holds only if the key sees everything
# determining the numbers, and live components fight back: they hold closures and memo dicts.


def test_identity_distinguishes_constructor_values() -> None:
    assert identity(MVP(1.05)) != identity(MVP(1.4))
    assert identity(MVP(1.05)) == identity(MVP(1.05))  # and is stable between instances


def test_identity_sees_a_value_captured_in_a_closure() -> None:
    """The collision that would have made the cache unsafe: two latencies from one factory.

    ``Comm``'s latency is a closure returned by ``constant_latency``, so its qualname is identical
    for every delay. Keying on the qualname alone would give ``constant_latency(0)`` and
    ``constant_latency(5)`` — very different physics — the same cache entry. The captured value has
    to reach the key, and it does, through ``__closure__``.
    """
    quiet = Comm(0.8, constant_latency(0.0))
    slow = Comm(0.8, constant_latency(5.0))
    assert identity(quiet) != identity(slow)
    assert identity(quiet) == identity(Comm(0.8, constant_latency(0.0)))


def test_identity_is_stable_across_a_memoising_noise_model() -> None:
    """A model that fills an internal cache as it runs must not change its own key by running.

    ``make_mixture_gaussian`` solves its calibrating sigma by bisection and memoises it in a dict
    held in the closure. Including that dict's *contents* would make the key depend on how far the
    run got; excluding the whole closure would lose the parameters. Private (underscore) free
    variables are treated as derived, so the parameters key and the memo does not.
    """
    nav = GnssNavigation(pos_distribution=make_mixture_gaussian(3.0, 0.1))
    before = identity(nav)
    nav.pos_distribution(generator(root_seed_sequence(0)), 10.0)  # fills the memo
    assert identity(nav) == before
    # ... while the calibration parameters still reach the key
    other = GnssNavigation(pos_distribution=make_mixture_gaussian(3.0, 0.2))
    assert identity(other) != before


def test_identity_refuses_rather_than_keying_weakly() -> None:
    """An unidentifiable component raises: a wrong key is worse than no cache."""

    class Opaque:
        def __init__(self) -> None:
            self.thing = object()

    with pytest.raises(CacheIdentityError):
        identity(Opaque())

    class Named:
        cache_id = "mine/v1"

    assert identity(Named()) == "id:mine/v1"


# --- cache, parallelism, provenance, plotting -----------------------------------------------


def test_cache_returns_the_same_rows_and_writes_one_entry_per_condition(tmp_path: Path) -> None:
    """A warm run reproduces the cold run exactly, with one stored entry per condition."""
    cc = CacheConfig(dir=tmp_path / "cache")
    declared = {**_PINNED, "dpsi": Sweep([45.0, 90.0])}
    kw = dict(methods=_methods(), backend=MC(n_encounters=20), base_config=_base(), seed=0)

    cold = run_experiment(declared, cache=cc, **kw)
    assert len(list((tmp_path / "cache").glob("*.pkl"))) == len(cold) == 2
    warm = run_experiment(declared, cache=cc, **kw)
    assert warm.records() == cold.records()


def test_changing_a_component_changes_the_cache_key(tmp_path: Path) -> None:
    """The identity feeds the key, so a different resolver is a miss rather than a stale hit.

    This is the property the whole identity machinery exists for: if it silently failed, this run
    would return the *first* resolver's numbers under the second resolver's name.
    """
    cc = CacheConfig(dir=tmp_path / "cache")
    kw = dict(backend=MC(n_encounters=20), base_config=_base(), seed=0)
    run_experiment(_PINNED, methods=_methods(resolver=MVP(1.05)), cache=cc, **kw)
    run_experiment(_PINNED, methods=_methods(resolver=MVP(1.4)), cache=cc, **kw)
    assert len(list((tmp_path / "cache").glob("*.pkl"))) == 2  # two keys, not one


def test_parallel_conditions_give_the_serial_answer() -> None:
    """``n_jobs`` is scheduling only: conditions are independent, so the numbers cannot move."""
    pytest.importorskip("joblib")
    declared = {**_PINNED, "dpsi": Sweep([45.0, 90.0])}
    kw = dict(methods=_methods(), backend=MC(n_encounters=20), base_config=_base(), seed=0)
    assert run_experiment(declared, n_jobs=2, **kw).records() == run_experiment(declared,
    **kw).records()


def test_the_card_records_what_would_be_needed_to_reproduce(tmp_path: Path) -> None:
    """A card carries the declaration, the component identities, the seed and the code hash."""
    res = run_experiment({**_PINNED, "dpsi": Sweep([45.0, 90.0])}, methods=_methods(),
                    backend=MC(n_encounters=20), base_config=_base(), seed=0,
                    card_dir=tmp_path)
    assert res.card_path is not None
    card = res.card_path.read_text()
    assert f"code_hash: {code_fingerprint()}" in card
    assert "Sweep([45.0, 90.0])" in card  # the declaration, with its role
    assert "opencdarr.cr.mvp.MVP" in card  # the component identity the cache also keys on
    assert card.count("\n| 45") + card.count("\n| 90") == 2  # one table row per condition


def test_no_card_is_written_by_default() -> None:
    assert run_experiment(_PINNED, methods=_methods(), backend=MC(n_encounters=2),
                     base_config=_base(), seed=0).card_path is None


def test_plot_lays_itself_out_from_the_axis_roles() -> None:
    """First swept axis on x, the rest as one line each, CI as a band, no grid and no title."""
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    res = run_experiment({**_PINNED, "dpsi": Sweep([45.0, 90.0]), "pos_ci95": Sweep([10.0, 30.0])},
                    methods=_methods(), backend=MC(n_encounters=20), base_config=_base(), seed=0)
    fig = res.plot("p_los")
    ax = fig.axes[0]
    assert len(ax.lines) == 2  # one per pos_ci95 level
    assert (ax.get_xlabel(), ax.get_ylabel()) == ("dpsi", "p_los")
    assert ax.collections  # the CI bands
    assert not fig._suptitle  # house convention: detail belongs in the caption
    assert ax.get_yscale() == "linear"  # log is the IPS default, not MC's


def test_plot_refuses_when_there_is_nothing_to_plot_against() -> None:
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    res = run_experiment(_PINNED, methods=_methods(), backend=MC(n_encounters=2),
                    base_config=_base(), seed=0)
    with pytest.raises(ValueError, match="nothing to plot against"):
        res.plot()
    swept = run_experiment({**_PINNED, "dpsi": Sweep([45.0, 90.0])}, methods=_methods(),
                      backend=MC(n_encounters=2), base_config=_base(), seed=0)
    with pytest.raises(KeyError, match="no metric"):
        swept.plot("n_collapsed")  # an IPS column, absent on MC


# --- the file-driven entry point --------------------------------------------------------------
# run_one_experiment is the all-Fixed, single-condition case of run_experiment, with its components
# named as strings. These were the old tests/test_experiment.py; they still pass unchanged, which
# is the point of the merge.


def _yaml_config() -> Config:
    return Config(
        seed=3,
        n_encounters=100,
        scenario=ScenarioConfig("M600", 10.2889, 50.0, 60.0),
        conflict=ConflictConfig(50.0, 120.0),
        methods=MethodsConfig("statebased", "mvp", "pastcpa", 1.05, False),
        simulation=SimulationConfig(1.0, 600.0, 10.0),
    )


def test_run_one_experiment_is_reproducible_and_has_one_row() -> None:
    cfg = _yaml_config()
    r1 = run_one_experiment(cfg, card_dir=None)
    r2 = run_one_experiment(cfg, card_dir=None)
    assert r1.records() == r2.records()
    assert len(r1) == 1 and r1.axes == ()  # nothing declared varies
    assert r1.cell().n_encounters == cfg.n_encounters
    assert r1.card_path is None


def test_run_one_experiment_matches_the_declared_equivalent() -> None:
    """The wrapper is the all-Fixed case, so it must equal writing that declaration by hand.

    This is what keeps it a wrapper rather than a second implementation: if the two ever diverge,
    one of them has grown behaviour the other lacks.
    """
    cfg = _yaml_config()
    wrapped = run_one_experiment(cfg, card_dir=None)
    by_hand = run_experiment(
        {},
        methods=Methods(detector=StateBased(), resolver=MVP(1.05), recovery=PastCPA(False),
                        perf=M600),
        backend=MC(n_encounters=cfg.n_encounters),
        base_config=cfg,
        seed=cfg.seed,
    )
    assert wrapped.records() == by_hand.records()


def test_run_one_experiment_writes_a_card(tmp_path: Path) -> None:
    cfg = _yaml_config()
    result = run_one_experiment(cfg, card_dir=tmp_path)
    assert result.card_path is not None and result.card_path.exists()
    text = result.card_path.read_text()
    assert f"seed: {cfg.seed}" in text
    assert "aircraft_type: M600" in text  # the config was dumped
    assert "code_hash:" in text


def test_run_one_experiment_rejects_an_unknown_component_name() -> None:
    cfg = dataclasses.replace(
        _yaml_config(), methods=MethodsConfig("bogus", "mvp", "pastcpa", 1.05, False)
    )
    with pytest.raises(ValueError, match="unknown detector"):
        run_one_experiment(cfg, card_dir=None)
