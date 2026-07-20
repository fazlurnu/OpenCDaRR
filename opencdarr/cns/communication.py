"""Communication — reception and latency (the C of CNS, Phase 3b).

Implements :class:`~opencdarr.cns.base.CommunicationModel`. Design decisions (state shape,
delivery timing, ordering guard, RNG layout) are recorded in
``vault/decisions/0006-communication-model-design.md``; the reference models the *effect* of
ADS-L (noisy, dropped, stale surveillance), not its message protocol
(``docs/lesson-learnt.md``).

A broadcast is offered to every other aircraft independently: each **directed link** draws its
own reception and its own latency, so A→B can be delivered while B→A is dropped in the same tick
— the asymmetry the directed design exists for (ADR 0004).
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from opencdarr.cns.base import (
    CommState,
    CommunicationModel,
    InFlight,
    LatencyDistribution,
    Message,
)


def constant_latency(seconds: float) -> LatencyDistribution:
    """A fixed link delay — draws nothing, so it consumes no randomness."""
    if seconds < 0.0:
        raise ValueError(f"latency must be >= 0, got {seconds}")
    return lambda rng: seconds


def uniform_latency(low: float, high: float) -> LatencyDistribution:
    """Link delay ~ U(low, high) [s] — the simple jitter model."""
    if low < 0.0 or high < low:
        raise ValueError(f"require 0 <= low <= high, got {low=}, {high=}")
    return lambda rng: float(rng.uniform(low, high))


def lognormal_latency(median: float, sigma: float) -> LatencyDistribution:
    """Link delay ~ LogNormal(ln ``median``, ``sigma``) [s] — positive, right-skewed.

    Parameterised by the **median** (not the mean) because ``exp(mu)`` *is* the median of a
    lognormal, which makes the typical delay directly readable. ``sigma`` is the standard
    deviation of the underlying normal, so it sets how heavy the slow-delivery tail is. This
    shape — most messages near-typical, a thin tail of much later ones, never negative — is the
    usual first-order model for datalink delay.
    """
    if median <= 0.0:
        raise ValueError(f"median must be > 0, got {median}")
    if sigma < 0.0:
        raise ValueError(f"sigma must be >= 0, got {sigma}")
    mu = math.log(median)
    return lambda rng: float(rng.lognormal(mu, sigma))


def _as_latency(latency: float | LatencyDistribution) -> LatencyDistribution:
    return latency if callable(latency) else constant_latency(float(latency))


class Comm(CommunicationModel):
    """Bernoulli reception plus a drawn latency, per directed link.

    ``reception_prob`` — probability a broadcast reaches a receiver. Either a scalar applied to
    every link, or a per-link mapping keyed **(source, receiver)** — i.e. the transmission
    direction, read "from → to" — so A→B and B→A can differ, which is the asymmetry the directed
    design exists for (ADR 0004). Links absent from the mapping default to 1.0.

    .. note::
       The mapping's ``(source, receiver)`` order is the *opposite* of
       :attr:`~opencdarr.cns.base.CommState.held`'s ``(receiver, source)`` key. They answer
       different questions: this one is "the link **from** A **to** B", ``held`` is "what B
       **knows about** A".

    ``latency`` — seconds of delay, either a constant or a
    :class:`~opencdarr.cns.base.LatencyDistribution`; a message measured at ``t_meas`` is
    delivered once simulation time reaches ``t_meas + latency``.

    With ``reception_prob=1.0`` and ``latency=0.0`` every broadcast is delivered in the same step
    it is offered, so the layer reduces exactly to Phase 3a's instant, perfect surveillance.
    """

    def __init__(
        self,
        reception_prob: float | Mapping[tuple[str, str], float] = 1.0,
        latency: float | LatencyDistribution = 0.0,
    ) -> None:
        if isinstance(reception_prob, Mapping):
            for link, p in reception_prob.items():
                if not 0.0 <= p <= 1.0:
                    raise ValueError(f"reception_prob{link} must be in [0, 1], got {p}")
            self._per_link: Mapping[tuple[str, str], float] | None = dict(reception_prob)
            self._scalar = 1.0
        else:
            if not 0.0 <= reception_prob <= 1.0:
                raise ValueError(f"reception_prob must be in [0, 1], got {reception_prob}")
            self._per_link = None
            self._scalar = float(reception_prob)
        self.reception_prob = reception_prob
        self.latency = _as_latency(latency)

    def _reception_for(self, source: str, receiver: str) -> float:
        """Delivery probability of the directed link ``source -> receiver``."""
        if self._per_link is None:
            return self._scalar
        return self._per_link.get((source, receiver), 1.0)

    def step(
        self,
        state: CommState,
        broadcasts: Sequence[Message],
        receivers: Sequence[str],
        t: float,
        rng: np.random.Generator,
    ) -> CommState:
        # 1. offer each broadcast to every other aircraft; per link, draw reception then (only
        #    if received) latency — the fixed order ADR 0006 pins so the stream stays reproducible
        in_flight = list(state.in_flight)
        for message in broadcasts:
            for receiver in receivers:
                if receiver == message.source:
                    continue  # an aircraft does not receive its own broadcast
                if float(rng.random()) >= self._reception_for(message.source, receiver):
                    continue  # dropped: nothing is enqueued, so the receiver keeps what it held
                delay = float(self.latency(rng))
                in_flight.append(
                    InFlight(message=message, receiver=receiver, deliver_t=message.t_meas + delay)
                )

        # 2. deliver everything now due, keeping the freshest message *by t_meas* — latency may
        #    exceed the broadcast interval, so a late old message can arrive after a newer one
        #    and must not clobber it (ADR 0006 §4)
        held = dict(state.held)
        still_flying: list[InFlight] = []
        for pending in in_flight:
            if pending.deliver_t > t:
                still_flying.append(pending)
                continue
            key = (pending.receiver, pending.message.source)
            current = held.get(key)
            if current is None or pending.message.t_meas > current.t_meas:
                held[key] = pending.message

        return CommState(held=held, in_flight=tuple(still_flying))
