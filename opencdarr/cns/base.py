"""CNS interfaces (navigation, communication, surveillance).

The C-N-S layers are pluggable, like cd/cr/crr. **N** (navigation) is how an aircraft measures
its own state to broadcast; **C** (communication) is how that broadcast reaches — or fails to
reach — a receiver, and how late; **S** (surveillance) is what a receiver *holds* as a result.
Communication design decisions are recorded in
``vault/decisions/0006-communication-model-design.md``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from opencdarr.state import AircraftState


@dataclass(frozen=True)
class Message:
    """A broadcast: an aircraft's own (noisy) self-measurement, timestamped for delivery."""

    source: str
    state: AircraftState  # the measured self-state (noisy)
    t_meas: float  # when it was measured [s]


class NoiseDistribution(Protocol):
    """A 2D position-error distribution: ``(rng, ci95, trk_deg) -> (east, north)`` error [m]."""

    def __call__(
        self, rng: np.random.Generator, ci95: float, trk_deg: float
    ) -> tuple[float, float]: ...


class NavigationModel(ABC):
    """How an aircraft measures its own state to broadcast — the contribution surface."""

    @abstractmethod
    def measure(self, true: AircraftState, t: float, rng: np.random.Generator) -> Message:
        """Return the aircraft's (noisy) self-measurement as a broadcastable :class:`Message`."""


class LatencyDistribution(Protocol):
    """A link-delay model: ``(rng) -> delay [s]``, drawn per delivered message."""

    def __call__(self, rng: np.random.Generator) -> float: ...


@dataclass(frozen=True)
class InFlight:
    """A broadcast the link accepted but has not delivered yet."""

    message: Message
    receiver: str  # the aircraft id this copy is addressed to
    deliver_t: float  # t_meas + this link's drawn latency [s]


@dataclass(frozen=True)
class CommState:
    """What the communication layer holds: delivered messages, plus what is still en route.

    ``held`` is keyed by **(receiver, source)** because surveillance is directed (ADR 0004):
    B's view of A is an independent draw from A's view of B. A key is absent until that link has
    delivered anything at all — the receiver has simply never heard of that source
    (ADR 0006 §5: no held message ⇒ that directed pair flies nominal).

    Immutability contract: like :class:`~opencdarr.state.AircraftState` this is a frozen, clonable
    value — but ``held`` is a plain mapping, so *frozen* stops the attribute being rebound, not the
    mapping being mutated. :meth:`CommunicationModel.step` therefore always builds a **new**
    mapping rather than mutating in place, so an IPS clone can never write through to its parent.
    """

    held: Mapping[tuple[str, str], Message] = field(default_factory=dict)
    in_flight: tuple[InFlight, ...] = ()


class CommunicationModel(ABC):
    """How broadcasts reach receivers (reception + latency) — the contribution surface."""

    @abstractmethod
    def step(
        self,
        state: CommState,
        broadcasts: Sequence[Message],
        receivers: Sequence[str],
        t: float,
        rng: np.random.Generator,
    ) -> CommState:
        """Return the comm state after offering ``broadcasts`` to ``receivers`` at time ``t``.

        Pure: the state is threaded, never mutated (see :class:`CommState`).
        """


class SurveillanceModel(ABC):
    """What a receiver believes about a source, given what communication delivered — the
    contribution surface.

    The only implementation today is hold-as-is (:class:`~opencdarr.cns.surveillance.LastKnown`,
    ADR 0006 §2); this stays an interface, not a bare function, because a future dead-reckoning
    model is the explicit alternative that decision names.
    """

    @abstractmethod
    def perceived(
        self, state: CommState, receiver: str, source: str, t_now: float
    ) -> AircraftState | None:
        """Return what ``receiver`` currently believes about ``source``.

        ``None`` if ``receiver`` has never received anything from ``source`` — the loop then
        flies that directed pair nominal (ADR 0006 §5).
        """
