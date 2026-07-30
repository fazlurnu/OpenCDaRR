"""Radio outages: independent transmitter and receiver failure (``TransceiverComm``).

``Comm`` loses messages; this model loses radios. The tests split into the two halves that can
break independently — the **hazard** (how often a subsystem fails, and whether that is a rate or a
per-broadcast coin) and the **gate** (what a failed subsystem actually stops) — because a model
that got either one right and the other wrong would still produce plausible-looking numbers.

Gate tests construct a state with the health they want and set every rate to zero, so a
deterministic assertion is not fighting a random draw.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

import numpy as np
import pytest

from opencdarr.cd import StateBased
from opencdarr.cns import (
    BroadcastSchedule,
    Comm,
    CommState,
    GnssNavigation,
    Message,
    RadioHealthState,
    TransceiverComm,
    constant_latency,
    radio_health,
)
from opencdarr.cr import MVP
from opencdarr.crr import PastCPA
from opencdarr.fleet import Agent, run_fleet
from opencdarr.performance import M600
from opencdarr.rng import children, generator, root_seed_sequence
from opencdarr.scenario import create_conflict
from opencdarr.state import AircraftState

_RECEIVERS = ("OWN", "INT")


def _msg(source: str, t_meas: float, gs: float = 10.0) -> Message:
    """A broadcast from ``source``; ``gs`` tags it so we can tell messages apart."""
    return Message(
        source=source,
        state=AircraftState(id=source, lat=52.0, lon=4.0, trk=0.0, gs=gs),
        t_meas=t_meas,
    )


def _rng() -> np.random.Generator:
    return np.random.default_rng(0)


def _radio(
    *,
    tx_down: Iterable[str] = (),
    rx_down: Iterable[str] = (),
    **comm_state: Any,
) -> CommState:
    """A comm state carrying one :class:`RadioHealth` gate at the health asked for.

    The gate states are positional and opaque to ``CommState``, so building one by hand is the
    verbose part of a gate test; this keeps the tests reading as "a fleet with OWN's receiver out".
    """
    return CommState(
        gates=(RadioHealthState(tx_down=frozenset(tx_down), rx_down=frozenset(rx_down)),),
        **comm_state,
    )


# --- the hazard: a rate, not a probability per broadcast ---------------------------------------


def _time_to_first_failure(
    model: TransceiverComm, dt: float, rng: np.random.Generator, t_max: float = 400.0
) -> float | None:
    """When ``OWN``'s receiver first fails, stepping the model alone on a ``dt`` cadence.

    One aircraft, so the rate is exactly the configured one; no broadcasts, so the only draws are
    the outage draws and the measurement is not entangled with reception.
    """
    state = model.initial_state()
    t = 0.0
    while t <= t_max:
        state = model.step(state, [], ("OWN",), t, rng)
        if radio_health(state).rx_down:
            return t
        t += dt
    return None


def test_mean_time_to_failure_is_the_same_at_two_cadences() -> None:
    """The load-bearing property: ``rx_fail_rate`` is per **hour**, so 1 Hz and 2 Hz agree.

    A probability quoted per broadcast instead would halve the mean time to failure when the
    cadence doubles, and a cadence sweep would then be moving reliability at the same time. The
    residual gap between the two is discretisation only — failures are seen at tick boundaries, so
    the mean is ``dt / (1 - exp(-rate*dt))``, i.e. ``1/rate + dt/2`` to first order. The rate is
    per hour, so 360/h is one failure per 10 s and the two cadences must agree on that.
    """
    rate, trials = 360.0, 600  # 360 per hour = 0.1 per second, so MTTF = 10 s
    means = {}
    for dt in (1.0, 0.5):
        model = TransceiverComm(rx_fail_rate=rate)
        rng = np.random.default_rng(20260730)
        times = [_time_to_first_failure(model, dt, rng) for _ in range(trials)]
        assert all(x is not None for x in times)  # 400 s >> 10 s: none should run out
        means[dt] = float(np.mean([x for x in times if x is not None]))

    assert 9.0 < means[1.0] < 12.0  # the configured 1/rate, plus dt/2 of discretisation
    assert 9.0 < means[0.5] < 12.0
    assert abs(means[1.0] - means[0.5]) < 1.5  # a per-broadcast probability would differ by ~5 s


def test_nothing_fails_before_any_time_has_elapsed() -> None:
    """At ``t = 0`` no time has passed, so no hazard has accrued however large the rate."""
    model = TransceiverComm(tx_fail_rate=1e9, rx_fail_rate=1e9)
    first = model.step(model.initial_state(), [], _RECEIVERS, 0.0, _rng())
    assert radio_health(first).tx_down == frozenset()
    assert radio_health(first).rx_down == frozenset()
    assert first.t_prev == 0.0
    # ... and one second later, at that rate, both radios of both aircraft are certainly gone
    second = model.step(first, [], _RECEIVERS, 1.0, _rng())
    assert radio_health(second).tx_down == frozenset(_RECEIVERS)
    assert radio_health(second).rx_down == frozenset(_RECEIVERS)


def test_a_failure_latches_unless_recovery_is_asked_for() -> None:
    """``recover_rate`` defaults to 0 — the permanent failure item 7 was written for."""
    down = _radio(rx_down={"OWN"}, t_prev=0.0)
    latched = TransceiverComm().step(down, [], ("OWN",), 1000.0, _rng())
    assert radio_health(latched).rx_down == frozenset({"OWN"})  # 1000 s later, still out

    healed = TransceiverComm(rx_recover_rate=36000.0).step(down, [], ("OWN",), 1000.0, _rng())
    assert radio_health(healed).rx_down == frozenset()


def test_the_two_subsystems_fail_independently() -> None:
    """A transmitter rate does not fail receivers, and vice versa."""
    tx_only = TransceiverComm(tx_fail_rate=1e9)
    state = tx_only.step(tx_only.initial_state(), [], _RECEIVERS, 0.0, _rng())
    state = tx_only.step(state, [], _RECEIVERS, 1.0, _rng())
    assert radio_health(state).tx_down == frozenset(_RECEIVERS)
    assert radio_health(state).rx_down == frozenset()


# --- the gate: what a failed subsystem stops ---------------------------------------------------


def test_a_down_receiver_goes_blind_while_the_fleet_still_sees_it() -> None:
    """The asymmetry the two sets exist for: OWN's receiver is out, its transmitter is fine."""
    comm = TransceiverComm(reception_prob=1.0, latency=0.0)  # no failures: only the gate is live
    state = comm.step(
        _radio(rx_down={"OWN"}),
        [_msg("OWN", 0.0), _msg("INT", 0.0)],
        _RECEIVERS,
        0.0,
        _rng(),
    )
    assert ("OWN", "INT") not in state.held  # OWN hears nothing about INT
    assert ("INT", "OWN") in state.held  # but INT still sees OWN — its transmitter is fine
    # every rate is 0, so the health is unchanged
    assert radio_health(state).rx_down == frozenset({"OWN"})


def test_a_down_transmitter_goes_silent_while_still_seeing_everyone() -> None:
    comm = TransceiverComm(reception_prob=1.0, latency=0.0)
    state = comm.step(
        _radio(tx_down={"OWN"}),
        [_msg("OWN", 0.0), _msg("INT", 0.0)],
        _RECEIVERS,
        0.0,
        _rng(),
    )
    assert ("INT", "OWN") not in state.held  # nobody hears OWN
    assert ("OWN", "INT") in state.held  # OWN still hears INT — its receiver is fine


def test_an_outage_holds_stale_data_rather_than_clearing_it() -> None:
    """Not the same experiment as ``reception_prob=0``: the blind aircraft *keeps* its last fix.

    That is the whole point of the model — the CDR layers keep deciding, on data that ages for as
    long as the outage lasts (``LastKnown``), instead of dropping the neighbour and flying nominal.
    """
    comm = TransceiverComm(reception_prob=1.0, latency=0.0)
    fresh = comm.step(_radio(), [_msg("INT", 0.0, gs=10.0)], _RECEIVERS, 0.0, _rng())
    assert fresh.held[("OWN", "INT")].state.gs == 10.0

    blind = comm.step(
        _radio(held=fresh.held, rx_down={"OWN"}),
        [_msg("INT", 5.0, gs=99.0)],
        _RECEIVERS,
        5.0,
        _rng(),
    )
    assert blind.held[("OWN", "INT")].t_meas == 0.0  # still the pre-outage message
    assert blind.held[("OWN", "INT")].state.gs == 10.0


def test_a_message_already_in_flight_still_arrives() -> None:
    """The gate is applied when a broadcast is *offered*, not when it is delivered."""
    comm = TransceiverComm(reception_prob=1.0, latency=constant_latency(2.0))
    sent = comm.step(_radio(), [_msg("INT", 0.0)], _RECEIVERS, 0.0, _rng())
    assert len(sent.in_flight) == 1  # accepted, due at t = 2

    later = comm.step(
        _radio(in_flight=sent.in_flight, rx_down={"OWN"}, t_prev=0.0),
        [],
        _RECEIVERS,
        2.0,
        _rng(),
    )
    assert ("OWN", "INT") in later.held
    assert later.in_flight == ()


def test_a_working_radio_delivers_exactly_like_comm() -> None:
    """With no radio down the gate is a no-op, so the deliveries are ``Comm``'s."""
    msgs = [_msg("OWN", 0.0), _msg("INT", 0.0)]
    plain = Comm(reception_prob=1.0, latency=0.0).step(CommState(), msgs, _RECEIVERS, 0.0, _rng())
    radio = TransceiverComm(reception_prob=1.0, latency=0.0).step(
        _radio(), msgs, _RECEIVERS, 0.0, _rng()
    )
    assert dict(radio.held) == dict(plain.held)
    assert radio.in_flight == plain.in_flight


# --- stream discipline ------------------------------------------------------------------------


def _delivery_trace(model: TransceiverComm, seed: int, ticks: int = 40) -> list[set[str]]:
    """Which links delivered a *fresh* message each tick — the observable draw pattern."""
    rng = np.random.default_rng(seed)
    state = model.initial_state()
    trace: list[set[str]] = []
    for k in range(ticks):
        t = float(k)
        state = model.step(state, [_msg("OWN", t), _msg("INT", t)], _RECEIVERS, t, rng)
        trace.append(
            {f"{src}->{rcv}" for (rcv, src), m in state.held.items() if m.t_meas == t}
        )
    return trace


def test_sweeping_a_rate_does_not_move_the_reception_draws() -> None:
    """A rate too small to ever fire must give **identical** deliveries to a rate of exactly 0.

    The outage draws are made every step whatever the health and whatever the rates, so the
    reception and latency draws sit at a fixed offset behind them. Skipping the draws when a rate
    is zero would make the ``fail_rate=0`` cell of a sweep run on a different noise stream from
    every other cell — the mistake recorded for ``sample_pairwise``'s pinned slots.
    """
    quiet = _delivery_trace(TransceiverComm(reception_prob=0.5), seed=3)
    tiny = _delivery_trace(
        TransceiverComm(reception_prob=0.5, tx_fail_rate=3.6e-12, rx_fail_rate=3.6e-12), seed=3
    )
    assert quiet == tiny
    assert any(links for links in quiet)  # the trace actually has deliveries to compare


def test_the_outage_draws_happen_even_when_no_radio_can_fail() -> None:
    """So ``TransceiverComm(fail_rate=0)`` is *not* stream-identical to ``Comm`` — by design.

    This is the other side of the test above, and it is what stops that one passing vacuously: if
    the draws were skipped, both models would sit at the same stream offset and the traces would
    match. They must not. The cost is that switching a study from ``Comm`` to a zero-rate
    ``TransceiverComm`` re-bases its numbers (a new cache key, so the cache handles it); the
    benefit is that every cell of a rate sweep shares one reception stream.
    """
    quiet = _delivery_trace(TransceiverComm(reception_prob=0.5), seed=3)

    rng = np.random.default_rng(3)
    plain, state = [], CommState()
    for k in range(len(quiet)):
        t = float(k)
        state = Comm(reception_prob=0.5).step(
            state, [_msg("OWN", t), _msg("INT", t)], _RECEIVERS, t, rng
        )
        plain.append({f"{src}->{rcv}" for (rcv, src), m in state.held.items() if m.t_meas == t})

    assert quiet != plain


def test_the_outage_draws_come_from_the_existing_comm_substream() -> None:
    """No fourth stream: ``ips.py`` pins ``children(seq, 0, 3)`` (nav, comm, broadcast).

    Driving a whole fleet run with only the three streams a particle owns is the assertion — a
    model that needed its own generator could not be constructed here at all. Reproducibility from
    the seed is checked at the same time, since an outage that was not seeded would not repeat.
    """
    own = AircraftState(id="OWN", lat=52.0, lon=4.0, trk=0.0, gs=10.0, pos_ci95=10.0, vel_ci95=1.0)
    intr = create_conflict(own, intr_id="INT", dpsi=45.0, dcpa=0.0, tlos=120.0, rpz=50.0)

    def once(seed: int) -> tuple[float, frozenset[str], frozenset[str]]:
        nav_seq, comm_seq, bc_seq = children(root_seed_sequence(seed), 0, 3)
        out = run_fleet(
            [Agent(own, M600), Agent(intr, M600)],
            rpz=50.0, t_lookahead=120.0, dt=0.5, detector=StateBased(),
            resolver=MVP(margin=1.05), recovery=PastCPA(bouncing_guard=True),
            navigation=GnssNavigation(), rng=generator(nav_seq),
            communication=TransceiverComm(reception_prob=0.95, rx_fail_rate=108.0,
                                          tx_fail_rate=108.0),
            comm_rng=generator(comm_seq),
            schedule=BroadcastSchedule(interval=1.0), broadcast_rng=generator(bc_seq),
            record=True,
        )
        assert out.frames is not None
        final = out.frames[-1].cns_state.comm
        assert radio_health(final) is not None
        return out.min_sep, radio_health(final).tx_down, radio_health(final).rx_down

    first, again = once(7), once(7)
    assert first == again  # same seed, same radios down, same trajectory
    # at ~0.03/s over a run of this length some radio really does fail, so the test is live
    assert first[1] or first[2]
    assert once(11) != first  # a different seed is a different history


# One seeded run of the full model, captured from the implementation as it stood when the channel
# was a `Comm` subclass that pre-filtered `super().step()`'s arguments. Every character is a draw
# order: change how reception, latency or the outage toggles consume the stream and these move.
# Read one column per tick, ticks 0..39. Deliveries: O = OWN->INT landed, I = INT->OWN, B = both,
# - = neither. Health: O = OWN's subsystem down, I = INT's, B = both, - = all working.
# The run is live in all three — INT's transmitter latches down at tick 3, OWN's receiver at 20 and
# INT's at 32, after which two deaf receivers mean nothing lands at all.
_PINNED_DELIVERIES = "BOOO----O-O-O-OO--O--O--O-OOOOOO--------"
_PINNED_TX_DOWN = "---IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII"
_PINNED_RX_DOWN = "--------------------OOOOOOOOOOOOBBBBBBBB"


def _code(members: frozenset[str]) -> str:
    """``members`` as one character, in the legend above."""
    own, intr = "OWN" in members, "INT" in members
    return "B" if own and intr else ("O" if own else ("I" if intr else "-"))


def test_the_seeded_comm_stream_is_unchanged() -> None:
    """A golden trace, so a refactor of the channel cannot quietly re-base every published number.

    The statistical tests above would all still pass if the draw *order* shifted — a different but
    equally-Bernoulli stream is still 50% reception. This pins the actual sequence instead, which
    is the property ADR 0006 §6 is really protecting and the one a restructuring is most likely to
    break by accident. If it fails, the numbers moved: either fix the ordering or re-run and
    re-publish deliberately, never edit the literals to match.
    """
    model = TransceiverComm(reception_prob=0.5, tx_fail_rate=108.0, rx_fail_rate=108.0)
    rng = np.random.default_rng(3)
    state = model.initial_state()
    deliveries, tx_down, rx_down = "", "", ""
    for k in range(len(_PINNED_DELIVERIES)):
        t = float(k)
        state = model.step(state, [_msg("OWN", t), _msg("INT", t)], _RECEIVERS, t, rng)
        fresh = {f"{src}->{rcv}" for (rcv, src), m in state.held.items() if m.t_meas == t}
        deliveries += _code(
            frozenset({"OWN"} if "OWN->INT" in fresh else set())
            | frozenset({"INT"} if "INT->OWN" in fresh else set())
        )
        tx_down += _code(radio_health(state).tx_down)
        rx_down += _code(radio_health(state).rx_down)

    assert deliveries == _PINNED_DELIVERIES
    assert tx_down == _PINNED_TX_DOWN
    assert rx_down == _PINNED_RX_DOWN


def test_a_bare_comm_state_is_rejected_rather_than_upgraded() -> None:
    """The inverse of the item-7 workaround: fail loudly instead of quietly patching the state."""
    with pytest.raises(TypeError, match="initial_state"):
        TransceiverComm().step(CommState(), [], _RECEIVERS, 0.0, _rng())


def test_invalid_rates_raise() -> None:
    for kwargs in (
        {"tx_fail_rate": -1e-3},
        {"rx_fail_rate": -1.0},
        {"tx_recover_rate": -0.5},
        {"rx_recover_rate": float("nan")},
        {"rx_fail_rate": float("inf")},
    ):
        with pytest.raises(ValueError):
            TransceiverComm(**kwargs)
    with pytest.raises(ValueError):  # inherited Comm validation still applies
        TransceiverComm(reception_prob=1.5)
