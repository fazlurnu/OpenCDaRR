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
    """A 2D position-error distribution: ``(rng, ci95) -> (east, north)`` error [m]."""

    def __call__(self, rng: np.random.Generator, ci95: float) -> tuple[float, float]: ...


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

    ``gates`` is one opaque state per :class:`LinkGate` the model was built with, positionally
    aligned with that model's gate tuple — the composable way to add an effect to the standard
    channel. ``t_prev`` is when the model last stepped (``None`` before the first call), so a gate
    can be told how much time really elapsed rather than assuming the nominal cadence.

    A model needing to remember something the gates cannot express **subclasses** this and returns
    its subclass from :meth:`CommunicationModel.initial_state`; see that method for the contract.
    That is the seam for replacing the channel wholesale; ``gates`` is the seam for adding one
    effect to it (ADR 0019 §3, §5).
    """

    held: Mapping[tuple[str, str], Message] = field(default_factory=dict)
    in_flight: tuple[InFlight, ...] = ()
    gates: tuple[object, ...] = ()
    t_prev: float | None = None


class LinkGate(ABC):
    """A stateful effect that can veto a directed link *before* the channel draws for it.

    The composable alternative to subclassing a communication model. A gate answers one question —
    "may this broadcast be offered on this directed link right now?" — and carries whatever state
    it needs to answer it across ticks. Several gates compose: a link is offered only if **every**
    gate admits it, so radio failure and terrain masking are two gates rather than a fourth class
    (ADR 0019 §3).

    **A veto is not ``reception_prob = 0``, and that distinction is load-bearing.** A denied link
    consumes *no* randomness at all: :meth:`admits` is consulted ahead of the reception draw and
    skips it entirely. Returning a zero probability instead would spend one draw per suppressed
    link and shift every number after it. This is why the contract is a boolean, not a multiplier —
    an effect that *modulates* the probability leaves the draw in place and belongs in
    ``Comm._reception_for``, not here (ADR 0019 §4).

    All three methods take the gate's **own** state rather than reading it off ``self``: the gate
    object is immutable shared configuration (one instance serves every IPS particle), while the
    state is a threaded value that clones with the particle — the same split
    :class:`~opencdarr.cns.stack.CNS` makes against :class:`~opencdarr.cns.stack.CnsState`.

    Implementations should be frozen dataclasses so ``experiment.identity`` can key a cache on them
    structurally; a plain object's ``repr`` carries a memory address and is not stable across
    processes.
    """

    @abstractmethod
    def initial(self) -> object:
        """This gate's state before anything happens — no roster needed, as for `CommState`."""

    @abstractmethod
    def evolve(
        self,
        own: object,
        receivers: Sequence[str],
        elapsed: float,
        rng: np.random.Generator,
    ) -> object:
        """Advance this gate's state over ``elapsed`` seconds, before the channel runs.

        Called once per step at a fixed offset from the start, in gate-registration order, so the
        draws a gate makes sit at a predictable place in the stream and do not move with how the
        reception draws happened to fall. A gate that draws should draw a **constant** number of
        times whatever its state and whatever its parameters — including when a rate is zero — so
        that sweeping a parameter moves this gate's outcomes without shifting the reception and
        latency draws underneath them (ADR 0006 §6).
        """

    @abstractmethod
    def admits(self, own: object, source: str, receiver: str) -> bool:
        """Whether ``source``'s broadcast may be offered to ``receiver``. Must not draw."""


class CommunicationModel(ABC):
    """How broadcasts reach receivers (reception + latency) — the contribution surface."""

    def initial_state(self) -> CommState:
        """The comm state at ``t = 0``: nothing delivered, nothing en route.

        The seam for a **stateful** model. :class:`CommState` is closed — exactly ``held`` and
        ``in_flight`` — so a model that must remember something across ticks (a failed radio that
        stays failed, a duty cycle, a queue) subclasses it and returns that subclass here.
        :meth:`step` then receives its own state type on **every** tick including the first,
        because :class:`~opencdarr.cns.stack.CNS` threads whatever ``step`` returns straight back
        in. Without this hook the first tick arrives as a plain :class:`CommState` and every such
        model has to detect and upgrade it by hand. A stateless model wants the default.

        Takes no arguments on purpose: per-aircraft state keys by id the way :attr:`CommState.held`
        does — absent means "nothing has happened to that aircraft yet" — so a model never needs
        the roster before the first tick. Configuration that *must* name real aircraft is checked
        by :meth:`validate_ids` instead.
        """
        return CommState()

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

    def validate_ids(self, ids: frozenset[str]) -> None:
        """Raise :class:`ValueError` if this model is configured against unknown aircraft ids.

        A per-link model keyed by aircraft id (e.g. :class:`~opencdarr.cns.communication.Comm`'s
        directed ``reception_prob``) reads an absent link as its default, so a mistyped id — a link
        naming an aircraft not in the fleet — is silently ignored rather than applied. The fleet
        composition root calls this with the actual roster so that mistake fails loudly instead. The
        base accepts anything; a model that keys configuration by id overrides it.
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
