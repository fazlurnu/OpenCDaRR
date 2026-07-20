"""Functional tests for the communication layer (reception + latency, Phase 3b).

Each test pins one decision from ``vault/decisions/0006-communication-model-design.md``.
"""

from __future__ import annotations

import numpy as np
import pytest

from opencdarr.cns import (
    Comm,
    CommState,
    Message,
    constant_latency,
    lognormal_latency,
    uniform_latency,
)
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


def test_perfect_link_delivers_instantly() -> None:
    """p=1, latency=0 reduces to Phase 3a: delivered in the step it is offered."""
    comm = Comm(reception_prob=1.0, latency=0.0)
    out = comm.step(CommState(), [_msg("OWN", 0.0), _msg("INT", 0.0)], _RECEIVERS, 0.0, _rng())
    assert out.in_flight == ()
    assert set(out.held) == {("INT", "OWN"), ("OWN", "INT")}


def test_no_self_delivery() -> None:
    """An aircraft never receives its own broadcast."""
    out = Comm().step(CommState(), [_msg("OWN", 0.0)], _RECEIVERS, 0.0, _rng())
    assert set(out.held) == {("INT", "OWN")}


def test_latency_delays_delivery() -> None:
    """A message is held in flight until t reaches t_meas + latency."""
    comm = Comm(reception_prob=1.0, latency=constant_latency(0.3))
    after_send = comm.step(CommState(), [_msg("OWN", 0.0)], _RECEIVERS, 0.0, _rng())
    assert after_send.held == {}  # not due yet
    assert len(after_send.in_flight) == 1
    assert after_send.in_flight[0].deliver_t == pytest.approx(0.3)

    # nothing new offered; sweeping at t=1.0 delivers it
    delivered = comm.step(after_send, [], _RECEIVERS, 1.0, _rng())
    assert delivered.in_flight == ()
    assert delivered.held[("INT", "OWN")].t_meas == 0.0


def test_dropped_message_leaves_the_previous_one_held() -> None:
    """A drop enqueues nothing, so the receiver keeps its stale held message."""
    perfect = Comm(reception_prob=1.0, latency=0.0)
    state = perfect.step(CommState(), [_msg("OWN", 0.0, gs=10.0)], _RECEIVERS, 0.0, _rng())
    assert state.held[("INT", "OWN")].state.gs == 10.0

    blackout = Comm(reception_prob=0.0)
    state = blackout.step(state, [_msg("OWN", 1.0, gs=99.0)], _RECEIVERS, 1.0, _rng())
    assert state.in_flight == ()
    assert state.held[("INT", "OWN")].t_meas == 0.0  # still the old one
    assert state.held[("INT", "OWN")].state.gs == 10.0


def test_never_received_source_is_absent_not_empty() -> None:
    """Before first contact the key is missing entirely (-> the pair flies nominal)."""
    state = Comm(reception_prob=0.0).step(
        CommState(), [_msg("OWN", 0.0), _msg("INT", 0.0)], _RECEIVERS, 0.0, _rng()
    )
    assert ("INT", "OWN") not in state.held
    assert ("OWN", "INT") not in state.held


def test_out_of_order_arrival_keeps_the_freshest_by_t_meas() -> None:
    """A late old message must not clobber a newer one already delivered."""
    slow = Comm(reception_prob=1.0, latency=constant_latency(5.0))  # sent t=0, due t=5
    fast = Comm(reception_prob=1.0, latency=constant_latency(0.2))  # sent t=1, due t=1.2

    state = slow.step(CommState(), [_msg("OWN", 0.0, gs=10.0)], _RECEIVERS, 0.0, _rng())
    state = fast.step(state, [_msg("OWN", 1.0, gs=20.0)], _RECEIVERS, 1.0, _rng())
    state = fast.step(state, [], _RECEIVERS, 2.0, _rng())  # newer (t_meas=1) arrives first
    assert state.held[("INT", "OWN")].t_meas == 1.0

    state = fast.step(state, [], _RECEIVERS, 6.0, _rng())  # older (t_meas=0) finally arrives
    assert state.held[("INT", "OWN")].t_meas == 1.0  # not regressed
    assert state.held[("INT", "OWN")].state.gs == 20.0


def test_reception_is_bernoulli_at_the_configured_rate() -> None:
    """Fraction of ticks whose freshly-sent message actually lands ≈ reception_prob."""
    comm = Comm(reception_prob=0.3, latency=0.0)
    rng = np.random.default_rng(7)
    n = 4000
    delivered = 0
    state = CommState()
    for i in range(n):
        state = comm.step(state, [_msg("OWN", float(i))], _RECEIVERS, float(i), rng)
        held = state.held.get(("INT", "OWN"))
        if held is not None and held.t_meas == float(i):
            delivered += 1
    assert abs(delivered / n - 0.3) < 0.03


def test_step_does_not_mutate_the_input_state() -> None:
    """Purity: the returned state is new; the input's mapping is untouched (IPS-clone safety)."""
    comm = Comm(reception_prob=1.0, latency=0.0)
    before = comm.step(CommState(), [_msg("OWN", 0.0)], _RECEIVERS, 0.0, _rng())
    snapshot = dict(before.held)
    after = comm.step(before, [_msg("INT", 1.0)], _RECEIVERS, 1.0, _rng())
    assert dict(before.held) == snapshot  # unchanged
    assert after is not before
    assert dict(after.held) != snapshot


def test_uniform_latency_is_within_bounds() -> None:
    comm = Comm(reception_prob=1.0, latency=uniform_latency(0.1, 0.5))
    rng = np.random.default_rng(3)
    state = comm.step(CommState(), [_msg("OWN", 0.0)] * 200, _RECEIVERS, -1.0, rng)
    delays = [f.deliver_t for f in state.in_flight]
    assert all(0.1 <= d <= 0.5 for d in delays)
    assert min(delays) < 0.2 and max(delays) > 0.4  # actually spread, not constant


def test_per_link_reception_is_asymmetric() -> None:
    """A→B and B→A are independent links with their own delivery rates (ADR 0004)."""
    comm = Comm(reception_prob={("OWN", "INT"): 0.8, ("INT", "OWN"): 0.99}, latency=0.0)
    rng = np.random.default_rng(11)
    n = 6000
    landed = {("OWN", "INT"): 0, ("INT", "OWN"): 0}
    state = CommState()
    for i in range(n):
        t = float(i)
        state = comm.step(state, [_msg("OWN", t), _msg("INT", t)], _RECEIVERS, t, rng)
        for source, receiver in landed:
            held = state.held.get((receiver, source))
            if held is not None and held.t_meas == t:
                landed[(source, receiver)] += 1
    assert abs(landed[("OWN", "INT")] / n - 0.8) < 0.02
    assert abs(landed[("INT", "OWN")] / n - 0.99) < 0.01


def test_links_absent_from_the_mapping_default_to_perfect() -> None:
    comm = Comm(reception_prob={("OWN", "INT"): 0.0})  # INT->OWN unlisted
    state = comm.step(CommState(), [_msg("OWN", 0.0), _msg("INT", 0.0)], _RECEIVERS, 0.0, _rng())
    assert ("INT", "OWN") not in state.held  # the 0.0 link never delivers
    assert ("OWN", "INT") in state.held  # the unlisted link defaults to 1.0


def test_lognormal_latency_is_positive_and_right_skewed() -> None:
    median, sigma = 0.15, 0.5
    draw = lognormal_latency(median, sigma)
    rng = np.random.default_rng(5)
    samples = np.array([draw(rng) for _ in range(20000)])
    assert (samples > 0).all()  # a delay is never negative
    assert abs(float(np.median(samples)) - median) < 0.01  # exp(mu) is the median
    assert samples.mean() > np.median(samples)  # right-skewed: mean pulled up by the tail
    # underlying normal recovers the configured sigma
    assert abs(float(np.log(samples).std()) - sigma) < 0.02


def test_invalid_parameters_raise() -> None:
    with pytest.raises(ValueError):
        Comm(reception_prob=1.5)
    with pytest.raises(ValueError):
        Comm(reception_prob={("OWN", "INT"): 1.5})
    with pytest.raises(ValueError):
        Comm(latency=-1.0)
    with pytest.raises(ValueError):
        uniform_latency(0.5, 0.1)
    with pytest.raises(ValueError):
        lognormal_latency(0.0, 0.5)
