"""Functional tests for GPS navigation noise."""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Sequence

import numpy as np
import pytest

from opencdarr import geo
from opencdarr.cd import StateBased
from opencdarr.cns import (
    BroadcastSchedule,
    GnssNavigation,
    NavState,
    NoiseDistribution,
    gaussian,
    make_anisotropic_gaussian,
    make_anisotropic_mixture_gaussian,
    make_mixture_gaussian,
)
from opencdarr.cns.stack import CNS, CnsState
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA, ProbabilisticFTR
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.fleet import Agent, run_fleet
from opencdarr.loop import run_encounter
from opencdarr.performance import M600
from opencdarr.relative import velocity_enu
from opencdarr.scenario import create_conflict, sample_pairwise
from opencdarr.state import AircraftState, DesiredVelocity

_TRUE = AircraftState(id="A", lat=52.0, lon=4.0, trk=30.0, gs=10.0)
_SIGMA_PER_CI95 = 1.0 / math.sqrt(5.991464547)


def _pos_offset_enu(true: AircraftState, meas: AircraftState) -> tuple[float, float]:
    qdr, dist = geo.qdrdist(true.lat, true.lon, meas.lat, meas.lon)
    q = math.radians(qdr)
    return dist * math.sin(q), dist * math.cos(q)


def test_zero_noise_measures_true_state() -> None:
    """Default AircraftState declares perfect accuracy (pos_ci95 = vel_ci95 = 0)."""
    nav = GnssNavigation()
    msg = nav.measure(nav.initial_state(), _TRUE, t=5.0, rng=np.random.default_rng(0))
    assert msg.source == "A"
    assert msg.t_meas == 5.0
    assert msg.state.lat == pytest.approx(_TRUE.lat)
    assert msg.state.lon == pytest.approx(_TRUE.lon)
    assert msg.state.trk == pytest.approx(_TRUE.trk)
    assert msg.state.gs == pytest.approx(_TRUE.gs)


def test_broadcast_declares_the_source_accuracy() -> None:
    """The measured (broadcast) state carries the source's own declared ci95."""
    true = dataclasses.replace(_TRUE, pos_ci95=20.0, vel_ci95=2.0)
    nav = GnssNavigation()
    msg = nav.measure(nav.initial_state(), true, t=0.0, rng=np.random.default_rng(0))
    assert msg.state.pos_ci95 == 20.0
    assert msg.state.vel_ci95 == 2.0


# --- a declaration that disagrees with the sensor -----------------------------------------------
# `pos_ci95`/`vel_ci95` are what the error is drawn from; `*_declared` is what the broadcast
# claims. Equal by default (`None`), so an honest transmitter needs no second number. Them
# disagreeing is the integrity failure RAIM exists to catch, and it is the experiment these fields
# exist for (`AircraftState`'s docstring).


def test_an_honest_declaration_is_the_default_and_changes_nothing() -> None:
    """``None`` claims the truth, so every pre-existing scenario measures exactly as before."""
    true = dataclasses.replace(_TRUE, pos_ci95=20.0, vel_ci95=2.0)
    assert true.pos_ci95_declared is None and true.vel_ci95_declared is None
    spelled_out = dataclasses.replace(true, pos_ci95_declared=20.0, vel_ci95_declared=2.0)
    nav = GnssNavigation()
    implicit = nav.measure(nav.initial_state(), true, 0.0, np.random.default_rng(4)).state
    explicit = nav.measure(nav.initial_state(), spelled_out, 0.0, np.random.default_rng(4)).state
    assert implicit == explicit


def test_the_broadcast_carries_the_claim_while_the_error_follows_the_sensor() -> None:
    """The two halves of the split, asserted together so neither can pass on its own."""
    true = dataclasses.replace(_TRUE, pos_ci95=40.0, vel_ci95=2.0)
    liar = dataclasses.replace(true, pos_ci95_declared=5.0, vel_ci95_declared=0.25)
    nav = GnssNavigation()

    honest_fix = nav.measure(nav.initial_state(), true, 0.0, np.random.default_rng(3)).state
    lying_fix = nav.measure(nav.initial_state(), liar, 0.0, np.random.default_rng(3)).state

    # the claim does not touch the draw: same seed, same actual accuracy, same geometry
    assert (lying_fix.lat, lying_fix.lon, lying_fix.trk, lying_fix.gs) == (
        honest_fix.lat, honest_fix.lon, honest_fix.trk, honest_fix.gs
    )
    # ... but the message declares the claim
    assert (lying_fix.pos_ci95, lying_fix.vel_ci95) == (5.0, 0.25)
    assert (honest_fix.pos_ci95, honest_fix.vel_ci95) == (40.0, 2.0)


def test_the_error_stays_calibrated_to_the_sensor_not_the_claim() -> None:
    """A 5 m claim over a 40 m sensor still scatters like 40 m -- the point of the mismatch."""
    liar = dataclasses.replace(_TRUE, pos_ci95=40.0, vel_ci95=0.0, pos_ci95_declared=5.0)
    nav = GnssNavigation()
    fresh = nav.initial_state()
    rng = np.random.default_rng(11)
    radial = np.array([
        math.hypot(*_pos_offset_enu(liar, nav.measure(fresh, liar, 0.0, rng).state))
        for _ in range(8000)
    ])
    assert abs(float(np.quantile(radial, 0.95)) - 40.0) < 2.0


def test_only_one_accuracy_travels_on_the_wire() -> None:
    """A broadcast carries the claim in ``pos_ci95`` and no second field: a receiver reads one
    number, and only a *true* state ever needs both (``AircraftState``'s docstring)."""
    liar = dataclasses.replace(_TRUE, pos_ci95=40.0, pos_ci95_declared=5.0, vel_ci95_declared=1.0)
    nav = GnssNavigation()
    fix = nav.measure(nav.initial_state(), liar, 0.0, np.random.default_rng(0)).state
    assert fix.pos_ci95 == 5.0
    assert fix.pos_ci95_declared is None and fix.vel_ci95_declared is None


def test_the_declaration_is_what_probabilistic_ftr_acts_on() -> None:
    """The consumer coupling, which is the whole reason a mismatch is interesting.

    ``ProbabilisticFTR`` sizes its covariance from the ``pos_ci95``/``vel_ci95`` it finds on the
    states it is handed, and after a broadcast those are the sender's *claim*. So with the sensor
    held fixed, changing only what the aircraft declares changes whether recovery resumes: an
    over-confident declaration resumes where an honest one would hold. That is the integrity
    failure being modelled, not a bug in the criterion -- ``_iso_cov`` takes a scalar and is
    agnostic to whatever distribution actually produced the error.
    """
    nav, ftr = GnssNavigation(), ProbabilisticFTR(prob_threshold=0.9)
    want = DesiredVelocity(v_east=0.0, v_north=10.0)
    # diverging: the intruder is abeam and departing, so the answer turns on how sure we are
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0, pos_ci95=40.0)
    intr = AircraftState(id="INT", lat=52.0, lon=4.0018, trk=90.0, gs=10.0, pos_ci95=40.0)

    def resumes(claim: float) -> bool:
        o = dataclasses.replace(own, pos_ci95_declared=claim, vel_ci95_declared=claim / 20.0)
        i = dataclasses.replace(intr, pos_ci95_declared=claim, vel_ci95_declared=claim / 20.0)
        seen_own = dataclasses.replace(
            nav.measure(nav.initial_state(), o, 0.0, np.random.default_rng(1)).state, desired=want
        )
        seen_intr = nav.measure(nav.initial_state(), i, 0.0, np.random.default_rng(2)).state
        return ftr.should_resume(seen_own, seen_intr, rpz=50.0)

    assert resumes(10.0) is True  # claims to be sure -> resumes
    assert resumes(100.0) is False  # claims to be unsure -> holds
    # the sensor never changed: only the declaration did
    assert own.pos_ci95 == intr.pos_ci95 == 40.0


def _encounter_min_sep(recovery: RecoveryCriterion, claim: float | None) -> float:
    """One seeded encounter's closest approach, with the sensor fixed at 40 m and only the
    declaration varying."""
    own, intr = sample_pairwise(
        np.random.default_rng(5), speed=10.2889, dcpa_max=50.0, tlos=60.0, rpz=50.0,
        pos_ci95=40.0, vel_ci95=2.0, pos_ci95_declared=claim,
    )
    outcome = run_encounter(
        own, intr, perf=M600, rpz=50.0, t_lookahead=120.0, dt=1.0,
        detector=StateBased(), resolver=MVP(1.05), recovery=recovery,
        navigation=GnssNavigation(), rng=np.random.default_rng(99),
    )
    return outcome.min_sep


def test_the_declaration_changes_a_whole_encounter_when_recovery_reads_it() -> None:
    """End to end: the flipped resume decision propagates into the flown trajectory.

    Claiming *less* accuracy makes ``ProbabilisticFTR`` hold the resolution longer, so the
    aircraft passes wider -- ``min_sep`` rises monotonically with the claim while the sensor and
    the seeds are unchanged. Asserting the direction, not just inequality, is what makes this a
    statement about the physics rather than about two numbers differing.
    """
    sep = [_encounter_min_sep(ProbabilisticFTR(), claim) for claim in (5.0, None, 200.0)]
    assert sep[0] < sep[1] < sep[2]


def test_the_declaration_is_inert_when_recovery_ignores_it() -> None:
    """The other half, and the one that proves the field reached the *right* consumer.

    ``PastCPA`` is certain-kinematics and never looks at ``pos_ci95``, so the same three claims
    must fly a bit-identical encounter: a declaration is metadata for whoever reads it and must
    not leak into the error draw. Without this, the test above would pass just as well if
    ``pos_ci95_declared`` had been wired into the noise magnitude by mistake.
    """
    sep = {_encounter_min_sep(PastCPA(True), claim) for claim in (5.0, None, 200.0)}
    assert len(sep) == 1


def test_position_noise_is_zero_mean_and_ci95_calibrated() -> None:
    ci95 = 20.0
    true = dataclasses.replace(_TRUE, pos_ci95=ci95, vel_ci95=0.0)
    nav = GnssNavigation()
    fresh = nav.initial_state()
    rng = np.random.default_rng(1)
    offsets = np.array(
        [_pos_offset_enu(true, nav.measure(fresh, true, 0.0, rng).state) for _ in range(8000)]
    )
    assert abs(offsets[:, 0].mean()) < 1.0  # zero-mean per axis
    assert abs(offsets[:, 1].mean()) < 1.0
    assert abs(offsets[:, 0].std() - ci95 * _SIGMA_PER_CI95) < 0.5  # per-axis sigma
    radial = np.hypot(offsets[:, 0], offsets[:, 1])
    assert abs(float(np.quantile(radial, 0.95)) - ci95) < 1.5  # 95% radial CI


def test_velocity_noise_is_zero_mean_and_ci95_calibrated() -> None:
    vel_ci95 = 2.0
    true = dataclasses.replace(_TRUE, pos_ci95=0.0, vel_ci95=vel_ci95)
    nav = GnssNavigation()
    fresh = nav.initial_state()
    rng = np.random.default_rng(2)
    ve = np.array([velocity_enu(nav.measure(fresh, true, 0.0, rng).state) for _ in range(8000)])
    true_e, true_n = velocity_enu(true)
    assert abs(ve[:, 0].std() - vel_ci95 * _SIGMA_PER_CI95) < 0.2  # per-axis sigma
    assert abs(ve[:, 0].mean() - true_e) < 0.2
    assert abs(ve[:, 1].mean() - true_n) < 0.2
    err_e, err_n = ve[:, 0] - true_e, ve[:, 1] - true_n
    radial = np.hypot(err_e, err_n)
    assert abs(float(np.quantile(radial, 0.95)) - vel_ci95) < 0.3  # 95% radial CI


def test_velocity_uses_its_own_pluggable_distribution() -> None:
    """A custom vel_distribution is honoured (and independent of the position one)."""
    vel_ci95 = 2.0
    true = dataclasses.replace(_TRUE, pos_ci95=0.0, vel_ci95=vel_ci95)
    true_e, true_n = velocity_enu(true)

    def constant_bias(rng: np.random.Generator, ci95: float) -> tuple[float, float]:
        return ci95, 0.0  # deterministic East-only velocity offset

    nav = GnssNavigation(vel_distribution=constant_bias)
    fix = nav.measure(nav.initial_state(), true, 0.0, np.random.default_rng(0)).state
    ve, vn = velocity_enu(fix)
    assert ve == pytest.approx(true_e + vel_ci95)
    assert vn == pytest.approx(true_n)


def test_reproducible_per_seed() -> None:
    true = dataclasses.replace(_TRUE, pos_ci95=20.0, vel_ci95=1.0)
    nav = GnssNavigation()
    a = nav.measure(nav.initial_state(), true, 0.0, np.random.default_rng(42)).state
    b = nav.measure(nav.initial_state(), true, 0.0, np.random.default_rng(42)).state
    assert a == b


# --- the seeded navigation stream ---------------------------------------------------------------

# Two aircraft measuring on every tick from **one shared** generator, in agent order -- exactly
# how `CNS.sense` drives the layer (`stack.py`: all navigation draws first, in `firing` order).
# Read one column per measurement, interleaved A,B for ticks 0..39, so the pin covers the
# *interleaving* as well as the values. That order is what makes `run_fleet` at n = 2 reduce
# bit-for-bit to `run_encounter`, and is what a refactor is most likely to disturb by accident.
#   radial -- the position error as tenths of that aircraft's own declared ci95, capped at 9.
#             Normalised because A and B declare different accuracies (20 m and 40 m); '9' means
#             "at or beyond the declared 95% radius", which should be about 1 column in 20.
#   sign   -- sign of the East velocity error. Moves with the *velocity* draws, which the position
#             radial cannot resolve on its own.
_PINNED_POS_RADIAL = (
    "1433570053088222261224545341432476155326"
    "4202520235695914152334854345994525167416"
)
_PINNED_VEL_EAST_SIGN = (
    "-++-----+--++-+++++-+----+++++---+--+--+"
    "+---+++-+-+--+---++-----++---+---+--+---"
)

_TRACE_A = AircraftState(id="A", lat=52.0, lon=4.0, trk=30.0, gs=10.0, pos_ci95=20.0, vel_ci95=2.0)
_TRACE_B = AircraftState(
    id="B", lat=52.01, lon=4.01, trk=210.0, gs=12.0, pos_ci95=40.0, vel_ci95=1.0
)


def test_the_seeded_navigation_stream_is_unchanged() -> None:
    """A golden trace of the layer, so restructuring it cannot silently re-base a number.

    Every other test in this file is statistical or functional, and would all still pass if the
    draw *order* moved -- a differently-ordered stream is still a calibrated zero-mean Gaussian.
    This pins the realised sequence instead, which is what ADR 0006 §6 actually protects. The
    companion pins over the channel live in ``test_cns_communication.py`` and
    ``test_cns_transceiver.py``; navigation had none until now.

    On failure the numbers moved: fix the ordering, or re-run and re-publish deliberately. Never
    edit the literals to match.
    """
    nav = GnssNavigation()
    fresh = nav.initial_state()
    rng = np.random.default_rng(7)
    radial, sign = "", ""

    for k in range(len(_PINNED_POS_RADIAL) // 2):
        for true in (_TRACE_A, _TRACE_B):
            measured = nav.measure(fresh, true, float(k), rng).state
            _, dist = geo.qdrdist(true.lat, true.lon, measured.lat, measured.lon)
            radial += str(min(int(10.0 * dist / true.pos_ci95), 9))
            true_e, _ = velocity_enu(true)
            meas_e, _ = velocity_enu(measured)
            sign += "+" if meas_e - true_e >= 0.0 else "-"

    assert radial == _PINNED_POS_RADIAL
    assert sign == _PINNED_VEL_EAST_SIGN


# --- the draw-count contract ---------------------------------------------------------------------

_DISTRIBUTIONS: dict[str, NoiseDistribution] = {
    "gaussian": gaussian,
    "mixture_gaussian": make_mixture_gaussian(),
    "anisotropic_gaussian": make_anisotropic_gaussian(),
    "anisotropic_mixture_gaussian": make_anisotropic_mixture_gaussian(),
}


def _stream_position(distribution: NoiseDistribution, ci95: float) -> object:
    """Where one call to ``distribution`` leaves a freshly-seeded generator."""
    rng = np.random.default_rng(0)
    distribution(rng, ci95)
    return rng.bit_generator.state


def test_a_distribution_consumes_the_same_randomness_whatever_the_ci95() -> None:
    """A zero ``ci95`` must cost the same draws as a non-zero one, for every distribution.

    ``pos_ci95=Sweep([0, 10, 20, 40])`` is the headline axis of this work, and a distribution
    that returns early at zero puts that first cell on a *different* stream from its neighbours
    -- so the cells stop being comparable and the zero column is not the same experiment. This is
    the rule ``LinkGate.evolve``'s docstring states and
    ``test_cns_transceiver.py::test_sweeping_a_rate_does_not_move_the_reception_draws`` pins for
    the channel (ADR 0006 §6); the mistake ``sample_pairwise``'s pinned slots also prevent.

    Sigma only scales the ziggurat's output, so the randomness consumed depends on the generator
    state and not on ``ci95`` -- a distribution that draws unconditionally lands in exactly the
    same place either way.
    """
    inconsistent = [
        name
        for name, distribution in _DISTRIBUTIONS.items()
        if _stream_position(distribution, 0.0) != _stream_position(distribution, 20.0)
    ]
    assert not inconsistent, f"these skip their draws at ci95 = 0: {inconsistent}"


def test_zero_ci95_returns_exactly_zero_error_from_every_distribution() -> None:
    """The value contract, held separately from the draw-count one above.

    Drawing unconditionally at ``sigma = 0`` must still return exactly ``(0.0, 0.0)`` -- otherwise
    fixing the draw count would quietly introduce noise into every perfect-sensor run.
    """
    for name, distribution in _DISTRIBUTIONS.items():
        assert distribution(np.random.default_rng(0), 0.0) == (0.0, 0.0), name


# --- the stateful-model seam (NavigationModel.initial_state) ------------------------------------


@dataclasses.dataclass(frozen=True)
class _TickCountNavState(NavState):
    """A nav state with a field of its own, to prove the subclass survives the whole run."""

    ticks: int = 0


class _TickCountingNav(GnssNavigation):
    """A stateful model that reads its **own** state field on every tick, upgrading nothing."""

    def initial_state(self) -> NavState:
        return _TickCountNavState()

    def evolve(
        self,
        state: NavState,
        aircraft: Sequence[AircraftState],
        t: float,
        rng: np.random.Generator,
    ) -> NavState:
        # the seam under test: never a bare NavState, not even on the first tick. Asserting it
        # here rather than upgrading with an isinstance fallback is the point -- a model written
        # this way is broken by a missing initial_state instead of quietly working around it.
        assert isinstance(state, _TickCountNavState)
        return _TickCountNavState(effects=state.effects, t_prev=t, ticks=state.ticks + 1)


def test_a_stateless_model_gets_the_plain_default() -> None:
    """The base hook is what ``GnssNavigation`` and every other stateless model wants."""
    assert GnssNavigation().initial_state() == NavState()
    assert CnsState.initial(2).nav == NavState()  # no model at all: the exact-self-fix path


def test_the_model_supplies_the_nav_layer_s_initial_value() -> None:
    nav = _TickCountingNav()
    assert CnsState.initial(2, navigation=nav).nav == _TickCountNavState(ticks=0)
    assert CNS(navigation=nav).initial_state(2).nav == _TickCountNavState(ticks=0)


def test_a_subclassed_nav_state_survives_a_whole_run() -> None:
    """A stateful model's own state is in place at t=0 and threaded to termination by run_fleet."""
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0, pos_ci95=20.0)
    intr = create_conflict(own, intr_id="INT", dpsi=90.0, dcpa=0.0, tlos=90.0, rpz=50.0)
    out = run_fleet(
        [Agent(own, M600), Agent(intr, M600)],
        rpz=50.0, t_lookahead=120.0, dt=1.0, detector=StateBased(),
        resolver=MVP(margin=1.05), recovery=PastCPA(bouncing_guard=True),
        navigation=_TickCountingNav(), rng=np.random.default_rng(0),
        schedule=BroadcastSchedule(interval=1.0), record=True,
    )
    assert out.frames is not None
    assert all(isinstance(f.cns_state.nav, _TickCountNavState) for f in out.frames)
    first, last = out.frames[0].cns_state.nav, out.frames[-1].cns_state.nav
    assert isinstance(first, _TickCountNavState) and isinstance(last, _TickCountNavState)
    assert first.ticks == 0  # the model's own state, before anything has been measured
    # dt == interval, so every step is a broadcast tick: one per frame after the initial one
    assert last.ticks == len(out.frames) - 1 > 5
