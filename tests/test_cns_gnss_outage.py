"""GnssOutage — a GNSS receiver that degrades and recovers.

Each test pins one decision from
``vault/decisions/0021-navigation-extension-by-quality-effects.md``. The hazard (when a receiver
degrades) and the modulation (what degrading does to the fix) are tested **separately**: a model
with one right and the other wrong still produces plausible numbers, which is the lesson
``vault/run-experiment-todo.md`` §10 records from the transceiver work.
"""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from opencdarr import geo
from opencdarr.cns import (
    CNS,
    BroadcastSchedule,
    CnsStreams,
    GnssNavigation,
    GnssOutage,
    GnssOutageState,
    NavQuality,
    NavState,
    gnss_outage,
)
from opencdarr.state import AircraftState

_A = AircraftState(id="A", lat=52.0, lon=4.0, trk=0.0, gs=10.0, pos_ci95=20.0, vel_ci95=2.0)
_B = dataclasses.replace(_A, id="B", lon=4.01)


def _degraded(*ids: str) -> NavState:
    """A nav state with exactly these aircraft already out — no draws involved.

    Building the health directly, with every rate at zero, is what lets the modulation tests assert
    deterministically instead of fighting a hazard draw for a state they only want as a fixture.
    """
    return NavState(effects=(GnssOutageState(out=frozenset(ids)),))


def _radial_error(nav: GnssNavigation, state: NavState, true: AircraftState, seed: int) -> float:
    measured = nav.measure(state, true, 0.0, np.random.default_rng(seed)).state
    _, dist = geo.qdrdist(true.lat, true.lon, measured.lat, measured.lon)
    return dist


# --- the hazard: when a receiver degrades -------------------------------------------------------


def _mean_time_to_outage(rate: float, interval: float, runs: int = 1500, seed: int = 0) -> float:
    """Average seconds until this aircraft's receiver first degrades, driven at ``interval``."""
    nav = GnssNavigation(effects=(GnssOutage(fail_rate=rate),))
    rng = np.random.default_rng(seed)
    times = []
    for _ in range(runs):
        state, t = nav.initial_state(), 0.0
        while "A" not in gnss_outage(state).out:
            t += interval
            state = nav.evolve(state, [_A], t, rng)
        times.append(t)
    return float(np.mean(times))


def test_the_outage_rate_is_per_hour_not_per_broadcast() -> None:
    """Mean time to outage is ``1 / fail_rate`` hours *whatever the cadence*.

    The discriminating test, and the reason the parameter is a rate at all: a probability quoted
    per broadcast would tie the mean to the interval, so doubling the cadence would **halve** it
    (~3 s here) and a cadence sweep would be moving two things at once. Over elapsed time the two
    cadences agree, up to the discrete-tick correction ``dt / (1 - exp(-rate*dt/3600))`` which is
    why neither lands exactly on 6 s.
    """
    rate = 600.0  # 1/h -> a 6 s mean, short enough to measure quickly
    slow = _mean_time_to_outage(rate, interval=1.0)
    fast = _mean_time_to_outage(rate, interval=0.5)
    assert slow == pytest.approx(6.51, abs=0.35)
    assert fast == pytest.approx(6.25, abs=0.35)
    assert abs(slow - fast) < 1.0  # not the ~3.3 s gap a per-broadcast rate would produce


def test_a_zero_recover_rate_latches_the_outage() -> None:
    """The default: once degraded, degraded for the rest of the encounter."""
    nav = GnssNavigation(effects=(GnssOutage(fail_rate=3.6e5),))  # ~0.01 s mean: out immediately
    rng = np.random.default_rng(1)
    state = nav.evolve(nav.initial_state(), [_A], 1.0, rng)
    assert gnss_outage(state).out == frozenset({"A"})
    for k in range(2, 40):
        state = nav.evolve(state, [_A], float(k), rng)
    assert gnss_outage(state).out == frozenset({"A"})


def test_a_recover_rate_brings_the_receiver_back() -> None:
    nav = GnssNavigation(effects=(GnssOutage(fail_rate=3.6e5, recover_rate=3.6e5),))
    rng = np.random.default_rng(2)
    state = nav.initial_state()
    seen = set()
    for k in range(1, 60):
        state = nav.evolve(state, [_A], float(k), rng)
        seen.add("A" in gnss_outage(state).out)
    assert seen == {True, False}  # it both fails and recovers over the run


def test_the_outage_is_per_aircraft() -> None:
    """One aircraft's receiver failing says nothing about another's.

    Rate chosen so a 1 s tick fails a receiver about half the time: a much larger rate would take
    both out on the first tick and the test could never observe them disagreeing, which is the
    whole point. Latching means only the first tick is informative, so this varies the seed rather
    than the tick.
    """
    nav = GnssNavigation(effects=(GnssOutage(fail_rate=2500.0),))  # ~0.50 per 1 s tick
    outcomes = set()
    for seed in range(20):
        state = nav.evolve(nav.initial_state(), [_A, _B], 1.0, np.random.default_rng(seed))
        outcomes.add(frozenset(gnss_outage(state).out))
    assert frozenset({"A"}) in outcomes or frozenset({"B"}) in outcomes
    assert frozenset() in outcomes and frozenset({"A", "B"}) in outcomes


# --- the modulation: what degrading does to the fix ---------------------------------------------


def test_a_degraded_receiver_scatters_by_the_factor() -> None:
    """Rates are all zero here: the health is a fixture, so this is pure modulation."""
    nav = GnssNavigation(effects=(GnssOutage(pos_factor=8.0),))
    nominal = np.array([_radial_error(nav, _degraded(), _A, s) for s in range(3000)])
    degraded = np.array([_radial_error(nav, _degraded("A"), _A, s) for s in range(3000)])
    assert degraded.mean() == pytest.approx(8.0 * nominal.mean(), rel=0.05)


def test_a_healthy_aircraft_is_bit_identical_to_no_effect() -> None:
    """An effect that is not firing must cost the fix nothing at all."""
    plain = GnssNavigation()
    gated = GnssNavigation(effects=(GnssOutage(fail_rate=1.0),))
    a = plain.measure(plain.initial_state(), _A, 0.0, np.random.default_rng(9)).state
    b = gated.measure(_degraded(), _A, 0.0, np.random.default_rng(9)).state
    assert a == b


def test_declaring_the_outage_widens_what_the_broadcast_claims() -> None:
    """``declare=True``: an honest transponder derates its own NACp/NIC."""
    nav = GnssNavigation(effects=(GnssOutage(pos_factor=8.0, vel_factor=4.0, declare=True),))
    fix = nav.measure(_degraded("A"), _A, 0.0, np.random.default_rng(0)).state
    assert fix.pos_ci95 == pytest.approx(8.0 * _A.pos_ci95)
    assert fix.vel_ci95 == pytest.approx(4.0 * _A.vel_ci95)


def test_not_declaring_it_is_the_integrity_failure() -> None:
    """``declare=False``: the fix is bad and the broadcast still claims nominal (ADR 0021 §2).

    The two arms give opposite downstream conclusions, which is why the fork is a parameter rather
    than a design commitment — a receiver acting on a confident-but-wrong number is exactly the
    case RAIM exists to catch, and it must be reachable.
    """
    nav = GnssNavigation(effects=(GnssOutage(pos_factor=8.0, declare=False),))
    state = _degraded("A")
    fix = nav.measure(state, _A, 0.0, np.random.default_rng(0)).state
    assert fix.pos_ci95 == pytest.approx(_A.pos_ci95)  # claims nominal ...
    errors = np.array([_radial_error(nav, state, _A, s) for s in range(2000)])
    nominal = np.array([_radial_error(nav, _degraded(), _A, s) for s in range(2000)])
    assert errors.mean() == pytest.approx(8.0 * nominal.mean(), rel=0.05)  # ... but is not


def test_two_effects_multiply() -> None:
    """Composition is by product with identity 1.0, where a gate's was ``all()`` with ``True``."""
    nav = GnssNavigation(effects=(GnssOutage(pos_factor=2.0), GnssOutage(pos_factor=3.0)))
    out = GnssOutageState(frozenset({"A"}))
    state = NavState(effects=(out, out))
    assert nav._quality_for(state, "A") == NavQuality(
        pos_scale=6.0, vel_scale=400.0, pos_declared=6.0, vel_declared=400.0
    )
    assert nav._quality_for(state, "B") == NavQuality()  # untouched aircraft: identity


# --- stream discipline --------------------------------------------------------------------------


def _measurement_trace(effect: GnssOutage | None, seed: int = 4) -> str:
    """The realised position errors over 30 ticks, as tenths of the declared ci95."""
    nav = GnssNavigation(effects=() if effect is None else (effect,))
    rng = np.random.default_rng(seed)
    state = nav.initial_state()
    out = ""
    for k in range(1, 31):
        state = nav.evolve(state, [_A, _B], float(k), rng)
        for true in (_A, _B):
            measured = nav.measure(state, true, float(k), rng).state
            _, dist = geo.qdrdist(true.lat, true.lon, measured.lat, measured.lon)
            out += str(min(int(10.0 * dist / true.pos_ci95), 9))
    return out


def test_sweeping_the_fail_rate_does_not_move_the_measurement_draws() -> None:
    """Every cell of a rate sweep shares one measurement stream (ADR 0006 §6).

    ``toggle`` draws once per aircraft per tick whatever the rates, so a rate too small ever to
    fire consumes exactly what a zero rate does and the fixes underneath are untouched. Without
    this the zero cell of a sweep would run on a different stream from its neighbours.
    """
    assert _measurement_trace(GnssOutage(fail_rate=0.0)) == _measurement_trace(
        GnssOutage(fail_rate=1e-12)
    )


def test_an_effect_free_model_draws_nothing_extra() -> None:
    """The property C could not have: with no effects the layer is stream-identical to pre-seam.

    ``TransceiverComm`` drew two per aircraft per tick unconditionally and so could never match
    plain ``Comm``; navigation has no unconditional per-tick state, so the empty case really is
    free — which is what makes the whole seam a no-re-base change (ADR 0021 §3).
    """
    nav = GnssNavigation()
    rng_a, rng_b = np.random.default_rng(3), np.random.default_rng(3)
    state = nav.initial_state()
    for k in range(1, 21):
        state = nav.evolve(state, [_A, _B], float(k), rng_a)
        nav.measure(state, _A, float(k), rng_a)
        nav.measure(state, _B, float(k), rng_b)  # rng_b never sees an evolve
    assert rng_a.bit_generator.state == rng_b.bit_generator.state


def test_the_outage_draws_come_from_the_existing_nav_substream() -> None:
    """No fourth substream: an effect draws from ``streams.nav`` like the measurement does.

    Built with only the substreams ADR 0006 §6 permits, so a model needing another could not run
    here at all.
    """
    nav = GnssNavigation(effects=(GnssOutage(fail_rate=3.6e5),))
    cns = CNS(navigation=nav)
    streams = CnsStreams(nav=np.random.default_rng(0))  # comm is absent, not merely unused
    state, _ = cns.sense([_A, _B], [0, 1], 1.0, cns.initial_state(2), streams)
    assert gnss_outage(state.nav).out == frozenset({"A", "B"})


# --- wiring and guards --------------------------------------------------------------------------


def test_a_bare_nav_state_is_rejected_rather_than_upgraded() -> None:
    """A model with effects handed a state that has none fails loudly, as `Comm` does for gates."""
    nav = GnssNavigation(effects=(GnssOutage(),))
    with pytest.raises(ValueError, match="effect state"):
        nav.evolve(NavState(), [_A], 1.0, np.random.default_rng(0))


def test_the_accessor_refuses_a_stack_without_the_effect() -> None:
    """Reporting "nothing is degraded" for an absent effect would read like a working receiver."""
    with pytest.raises(ValueError, match="no GnssOutage"):
        gnss_outage(GnssNavigation().initial_state())


def test_negative_rates_and_factors_are_rejected() -> None:
    for kwargs in ({"fail_rate": -1.0}, {"recover_rate": float("nan")}, {"pos_factor": -2.0}):
        with pytest.raises(ValueError):
            GnssOutage(**kwargs)  # type: ignore[arg-type]


def test_an_outage_reaches_perception_through_the_stack() -> None:
    """End to end through ``CNS.sense``: a degraded sender's broadcast is what receivers hold."""
    nav = GnssNavigation(effects=(GnssOutage(fail_rate=3.6e5, pos_factor=9.0, declare=True),))
    cns = CNS(navigation=nav)
    streams = CnsStreams(nav=np.random.default_rng(7))
    state, perception = cns.sense([_A, _B], [0, 1], 1.0, cns.initial_state(2), streams)
    assert gnss_outage(state.nav).out == frozenset({"A", "B"})
    # every aircraft's own fix now declares the degraded accuracy, and so does what it broadcast
    assert perception[0].own.pos_ci95 == pytest.approx(9.0 * _A.pos_ci95)
    assert perception[0].traffic[0].pos_ci95 == pytest.approx(9.0 * _B.pos_ci95)


def test_the_schedule_cadence_does_not_change_the_outage_semantics() -> None:
    """A jittered schedule makes the gap between calls something other than the nominal interval;
    the hazard is over *elapsed* time, so the effect needs to know nothing about the cadence."""
    schedule = BroadcastSchedule(interval=1.0, jitter=0.3)
    assert schedule.jitter == 0.3  # the fixture is live; the semantics are asserted above
    nav = GnssNavigation(effects=(GnssOutage(fail_rate=600.0),))
    rng = np.random.default_rng(0)
    state = nav.evolve(nav.initial_state(), [_A], 3600.0, rng)  # one hour in one step
    assert "A" in gnss_outage(state).out  # a 600/h rate over an hour is a certainty
