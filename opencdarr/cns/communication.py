"""Communication — reception and latency (the C of CNS, Phase 3b).

Implements :class:`~opencdarr.cns.base.CommunicationModel`. Design decisions (state shape,
delivery timing, ordering guard, RNG layout) are recorded in
``vault/decisions/0006-communication-model-design.md``; the reference models the *effect* of
ADS-L (noisy, dropped, stale surveillance), not its message protocol
(``docs/lesson-learnt.md``).

A broadcast is offered to every other aircraft independently: each **directed link** draws its
own reception and its own latency, so A→B can be delivered while B→A is dropped in the same tick
— the asymmetry the directed design exists for (ADR 0004).

Two models live here, and they answer different questions. :class:`Comm` loses *messages*: every
tick is an independent draw, so it is the channel. :class:`TransceiverComm` loses *radios*: a
transmitter or a receiver fails, stays failed for a stretch of time, and may recover — a stateful
model, and the first user of :meth:`~opencdarr.cns.base.CommunicationModel.initial_state`.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np

from opencdarr.cns.base import (
    CommState,
    CommunicationModel,
    InFlight,
    LatencyDistribution,
    LinkGate,
    Message,
)
from opencdarr.cns.hazard import hazard, toggle


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
        *,
        gates: Sequence[LinkGate] = (),
    ) -> None:
        self.gates = tuple(gates)
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

    def validate_ids(self, ids: frozenset[str]) -> None:
        """Reject a directed ``reception_prob`` link naming an aircraft not in the fleet.

        Because an absent link defaults to ``1.0``, a mistyped id (``("COPTER", "PLANE")`` when the
        fleet is ``COPTER`` / ``CARGO``) would silently apply *no* loss on that link instead of the
        value written. Checked at the composition root against the real roster, so the typo fails
        loudly. A scalar ``reception_prob`` keys nothing and is always accepted.
        """
        if self._per_link is None:
            return
        unknown = {
            aid
            for link in self._per_link
            for aid in link
            if aid not in ids
        }
        if unknown:
            raise ValueError(
                f"reception_prob names aircraft not in the fleet: {sorted(unknown)}. "
                f"Known ids: {sorted(ids)}. A directed link must name two aircraft that exist, "
                "else it silently applies no loss (absent links default to 1.0)."
            )

    def _offer(
        self,
        in_flight: Sequence[InFlight],
        gate_states: Sequence[object],
        broadcasts: Sequence[Message],
        receivers: Sequence[str],
        rng: np.random.Generator,
    ) -> list[InFlight]:
        """Offer each broadcast to every other aircraft, queueing whatever the link accepts.

        Per directed link: consult the gates (which draw nothing), then draw reception, then —
        **only if received** — latency. That order is what ADR 0006 §6 pins, so a dropped message
        costs one draw and a delivered one costs two (or one, since :func:`constant_latency` draws
        nothing). A drop enqueues nothing at all, which is exactly what leaves the receiver holding
        whatever it already had.

        A gate veto ``continue``s **ahead of** the reception draw, so a suppressed link spends no
        randomness — that is what makes a gate stream-identical to the argument pre-filtering it
        replaces, and why a gate cannot be spelled as a zero probability (ADR 0019 §4).
        """
        queued = list(in_flight)
        for message in broadcasts:
            for receiver in receivers:
                if receiver == message.source:
                    continue  # an aircraft does not receive its own broadcast
                if not all(
                    gate.admits(own, message.source, receiver)
                    for gate, own in zip(self.gates, gate_states, strict=True)
                ):
                    continue  # vetoed: no draw is spent, and the receiver keeps what it held
                if float(rng.random()) >= self._reception_for(message.source, receiver):
                    continue  # dropped: nothing is enqueued, so the receiver keeps what it held
                delay = float(self.latency(rng))
                queued.append(
                    InFlight(message=message, receiver=receiver, deliver_t=message.t_meas + delay)
                )
        return queued

    @staticmethod
    def _deliver(
        held: Mapping[tuple[str, str], Message],
        in_flight: Sequence[InFlight],
        t: float,
    ) -> tuple[dict[tuple[str, str], Message], tuple[InFlight, ...]]:
        """Move everything due into its receiver's slot; return those slots and what still flies.

        Each ``(receiver, source)`` slot keeps the freshest message **by ``t_meas``**, not by
        arrival order: latency may exceed the broadcast interval, so a late old message can arrive
        after a newer one and must not clobber it (ADR 0006 §4). Draws nothing — delivery is a
        function of ``deliver_t`` and ``t`` alone, which is why it is a plain static helper.
        """
        slots = dict(held)
        still_flying: list[InFlight] = []
        for pending in in_flight:
            if pending.deliver_t > t:
                still_flying.append(pending)
                continue
            key = (pending.receiver, pending.message.source)
            current = slots.get(key)
            if current is None or pending.message.t_meas > current.t_meas:
                slots[key] = pending.message
        return slots, tuple(still_flying)

    def initial_state(self) -> CommState:
        """Nothing delivered, nothing en route, and every gate at its own starting state."""
        return CommState(gates=tuple(gate.initial() for gate in self.gates))

    def step(
        self,
        state: CommState,
        broadcasts: Sequence[Message],
        receivers: Sequence[str],
        t: float,
        rng: np.random.Generator,
    ) -> CommState:
        if len(state.gates) != len(self.gates):
            raise TypeError(
                f"{type(self).__name__} has {len(self.gates)} gate(s) but was handed a state "
                f"carrying {len(state.gates)}. Gate states come from initial_state() — build the "
                "run through CnsState.initial(n, model) (which run_fleet does) rather than "
                "handing this model a bare CommState."
            )

        # 1. every gate first, at a fixed offset from the start of the step, so whatever a gate
        #    draws does not shift with how the reception draws happened to fall. No time has
        #    elapsed on the first call, so nothing can have changed at t = 0.
        elapsed = 0.0 if state.t_prev is None else max(0.0, t - state.t_prev)
        gate_states = tuple(
            gate.evolve(own, receivers, elapsed, rng)
            for gate, own in zip(self.gates, state.gates, strict=True)
        )

        # 2. the two halves of a tick that consume anything: what the links accept onto the air,
        #    then what has arrived by now. All the channel's randomness is in the first.
        in_flight = self._offer(state.in_flight, gate_states, broadcasts, receivers, rng)
        held, still_flying = self._deliver(state.held, in_flight, t)
        return CommState(held=held, in_flight=still_flying, gates=gate_states, t_prev=t)


@dataclass(frozen=True)
class RadioHealthState:
    """Which aircraft currently have a failed transmitter / receiver — `RadioHealth`'s state.

    Two sets rather than one flag because they are two pieces of hardware: an aircraft whose
    *receiver* has failed is flying blind while its transmitter keeps squittering, so the rest of
    the fleet still sees it perfectly. Absent from a set means that subsystem works — the same
    "absent ⇒ nothing has happened yet" reading :attr:`~opencdarr.cns.base.CommState.held` uses,
    which is why :meth:`RadioHealth.initial` needs no roster.
    """

    tx_down: frozenset[str] = frozenset()
    rx_down: frozenset[str] = frozenset()


@dataclass(frozen=True)
class RadioHealth(LinkGate):
    """A per-aircraft transmitter and receiver that fail — and recover — on their own.

    :class:`Comm`'s ``reception_prob`` drops an individual message and re-draws from scratch next
    tick; it has no memory, so a radio that is *out* for a stretch of time cannot be expressed with
    it. This gate adds that: each subsystem of each aircraft fails at ``tx_fail_rate`` /
    ``rx_fail_rate`` and comes back at ``tx_recover_rate`` / ``rx_recover_rate``, all in **events
    per hour** — the unit reliability is quoted in — so the mean time to failure is ``1 / rate``
    hours. ``RadioHealth(rx_fail_rate=2.0)`` is a receiver that dies every half hour on average;
    ``0.036`` is one that lasts about 28 hours. Reception and latency are untouched and apply to
    whatever the working radios still carry, so the two effects compose.

    The recover rates default to ``0``, which makes a failure **permanent** for the rest of the
    encounter — the latching radio. Give them a value and the radio is intermittent instead. All
    four are separate parameters because a transmitter and a receiver are separate hardware with no
    reason to share a reliability figure; pass only the ones you need
    (``RadioHealth(rx_fail_rate=3.6)`` fails receivers and nothing else).

    **What an outage does.** A down *transmitter* means that aircraft's broadcast is not offered to
    anyone, so it goes silent while still seeing everyone. A down *receiver* means it is offered
    nothing, so it goes blind while everyone still sees it. Either way the affected receiver keeps
    **holding** what it last received (:class:`~opencdarr.cns.surveillance.LastKnown`) and decides
    on data that ages for as long as the outage lasts — which is the behaviour worth studying, and
    is why an outage is not the same experiment as ``reception_prob=0``. Measured in
    ``vault/observations/transceiver-outage-perception.md``, which also shows the two failures are
    indistinguishable at n = 2 and only separate at n ≥ 3.

    The veto is applied when a broadcast is **offered**. A message the link already accepted stays
    in flight and is delivered when due, even if that receiver's radio failed in between; at the
    default ``latency=0`` the two readings coincide exactly.

    Draws come from the **existing** comm substream — a fourth stream would break the
    config-invariant tree (``ips.py`` pins exactly three children, ADR 0006 §6). Note that a live
    outage changes how many reception draws a tick makes, so per-tick consumption is not a function
    of tick count alone; that was already true of :class:`Comm` (a latency draw happens only on a
    delivered message) and is harmless because each particle owns its generator.

    .. warning::
       IPS will not reach a small ``fail_rate``. Radio failure is a discrete jump that ``min_sep``
       carries no information about, so the shells cannot steer toward it — the pathway measured
       collapsing 8/8 replications in ``vault/important-ips-gap.md``. Estimate an outage study by
       plain MC, or condition on the failure time and reweight.
    """

    tx_fail_rate: float = 0.0
    rx_fail_rate: float = 0.0
    tx_recover_rate: float = 0.0
    rx_recover_rate: float = 0.0

    def __post_init__(self) -> None:
        for name in ("tx_fail_rate", "rx_fail_rate", "tx_recover_rate", "rx_recover_rate"):
            rate = getattr(self, name)
            if rate < 0.0 or not math.isfinite(rate):
                raise ValueError(f"{name} must be a finite rate >= 0 [1/h], got {rate}")

    def initial(self) -> RadioHealthState:
        """Every radio working."""
        return RadioHealthState()

    def evolve(
        self,
        own: object,
        receivers: Sequence[str],
        elapsed: float,
        rng: np.random.Generator,
    ) -> RadioHealthState:
        """Age every radio by ``elapsed`` seconds — two draws per aircraft, always (ADR 0006 §6).

        The hazard is applied over the *elapsed* time rather than per call, so offset broadcast
        phases and jitter — which make the gap between calls something other than the nominal
        interval — come out right without the gate being told the cadence at all.
        """
        assert isinstance(own, RadioHealthState)
        tx_down, rx_down = set(own.tx_down), set(own.rx_down)
        p_tx_fail = hazard(self.tx_fail_rate, elapsed)
        p_tx_recover = hazard(self.tx_recover_rate, elapsed)
        p_rx_fail = hazard(self.rx_fail_rate, elapsed)
        p_rx_recover = hazard(self.rx_recover_rate, elapsed)
        for aid in receivers:  # agent order: the fleet's, so the pairwise runner's at n = 2
            toggle(tx_down, aid, p_tx_fail, p_tx_recover, rng)
            toggle(rx_down, aid, p_rx_fail, p_rx_recover, rng)
        return RadioHealthState(tx_down=frozenset(tx_down), rx_down=frozenset(rx_down))

    def admits(self, own: object, source: str, receiver: str) -> bool:
        """A silent transmitter offers nothing; a deaf receiver is offered nothing."""
        assert isinstance(own, RadioHealthState)
        return source not in own.tx_down and receiver not in own.rx_down


def radio_health(state: CommState) -> RadioHealthState:
    """The radio health carried by ``state``, for a model built with a :class:`RadioHealth` gate.

    Instrumentation, like :func:`~opencdarr.cns.surveillance.age`: the gate states are positional
    and opaque to :class:`~opencdarr.cns.base.CommState`, so this is the readable way to ask which
    radios are down without knowing the gate's index.
    """
    for own in state.gates:
        if isinstance(own, RadioHealthState):
            return own
    raise ValueError(
        "this state carries no RadioHealth gate — it came from a model built without one"
    )


class TransceiverComm(Comm):
    """:class:`Comm` with a :class:`RadioHealth` gate — the spelling that predates gates.

    Kept because a dozen scripts and notebooks construct it, and because "a channel whose radios
    fail" is a common enough stack to deserve a name. It is exactly
    ``Comm(reception_prob, latency, gates=(RadioHealth(...),))`` and holds nothing of its own;
    reach for the explicit form to put a second gate alongside this one (ADR 0019 §3).
    """

    def __init__(
        self,
        reception_prob: float | Mapping[tuple[str, str], float] = 1.0,
        latency: float | LatencyDistribution = 0.0,
        *,
        tx_fail_rate: float = 0.0,
        rx_fail_rate: float = 0.0,
        tx_recover_rate: float = 0.0,
        rx_recover_rate: float = 0.0,
    ) -> None:
        super().__init__(
            reception_prob=reception_prob,
            latency=latency,
            gates=(
                RadioHealth(
                    tx_fail_rate=tx_fail_rate,
                    rx_fail_rate=rx_fail_rate,
                    tx_recover_rate=tx_recover_rate,
                    rx_recover_rate=rx_recover_rate,
                ),
            ),
        )
