"""CNS interfaces (navigation, communication, surveillance).

The C-N-S layers are pluggable, like cd/cr/crr. **N** (navigation) is how an aircraft measures
its own state to broadcast; **C** (communication) is how that broadcast reaches — or fails to
reach — a receiver, and how late; **S** (surveillance) is what a receiver *holds* as a result.
Communication design decisions are recorded in
``vault/decisions/0006-communication-model-design.md``; how each layer is *extended* — a
:class:`LinkGate` for C, a :class:`NavEffect` for N, and why only one of them may veto — in
``0019-channel-extension-by-link-gates.md`` and
``0021-navigation-extension-by-quality-effects.md``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
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


@dataclass(frozen=True)
class NavQuality:
    """How much worse than nominal one aircraft's fix is, and how much of that it admits to.

    ``*_scale`` multiplies the accuracy the error is actually **drawn** from; ``*_declared``
    multiplies the accuracy stamped on the **broadcast**. Honest degradation sets them equal; an
    *integrity failure* — the fix degrades while the transponder keeps claiming nominal, the case
    RAIM exists to catch — is ``declared = 1.0`` with a large ``scale`` (ADR 0021 §2). All four
    default to ``1.0``, so an effect that has nothing to say about a quantity says nothing.

    Position and velocity are separate fields for the reason
    :class:`~opencdarr.cns.communication.RadioHealth` gives four separate rates: they come from
    different measurements (pseudorange vs Doppler) with no reason to degrade together.

    Scales only, no additive offset: a *static* bias is already expressible as a
    :class:`NoiseDistribution` and needs nothing here, and only a **drifting** bias would justify
    offset fields (ADR 0021 §1's obligation).
    """

    pos_scale: float = 1.0
    vel_scale: float = 1.0
    pos_declared: float = 1.0
    vel_declared: float = 1.0


@dataclass(frozen=True)
class NavState:
    """What the navigation layer holds across ticks — the N-side twin of :class:`CommState`.

    ``effects`` is one opaque state per :class:`NavEffect` the model was built with, positionally
    aligned with that model's effect tuple. ``t_prev`` is when the layer last ran (``None`` before
    the first call), so an effect is told how much time really elapsed rather than assuming the
    nominal cadence.

    Deliberately with no ``held``-analogue: navigation holds nothing between ticks of its own
    accord — an aircraft measures itself fresh every time — so the only thing to carry is whatever
    the effects need. A model needing to remember something the effects cannot express
    **subclasses** this and returns its subclass from :meth:`NavigationModel.initial_state`.
    """

    effects: tuple[object, ...] = ()
    t_prev: float | None = None


class NavEffect(ABC):
    """A stateful effect that *modulates* an aircraft's fix quality — never vetoes it.

    The navigation analogue of :class:`LinkGate`, and it diverges in exactly one place: there is no
    ``admits``. A gate's unit of work is a directed link offer, which can be declined; navigation's
    is "produce a fix", and an aircraft always produces one — a degraded receiver broadcasts a
    *worse* position, not no position. The one nav-shaped veto, "this aircraft does not transmit",
    is already spelled :class:`~opencdarr.cns.communication.RadioHealth`, and a second spelling for
    one physical event is what ``design-philosophy.md`` #17 forbids (ADR 0021 §1).

    Several effects compose: their :class:`NavQuality` values **multiply**, where a gate's booleans
    were combined with ``all()``. Identity is ``1.0`` rather than ``True``.

    As with :class:`LinkGate`, all methods take the effect's **own** state rather than reading it
    off ``self``: the effect object is immutable shared configuration (one instance serves every
    IPS particle) while the state is a threaded value that clones with the particle.
    Implementations should be frozen dataclasses so ``experiment.identity`` can key a cache on them
    structurally.
    """

    @abstractmethod
    def initial(self) -> object:
        """This effect's state before anything happens — no roster needed, as for
        :class:`NavState`.

        Per-aircraft state keys by id the way :attr:`CommState.held` does: absent means "nothing
        has happened to that aircraft yet".
        """

    @abstractmethod
    def evolve(
        self,
        own: object,
        aircraft: Sequence[AircraftState],
        elapsed: float,
        rng: np.random.Generator,
    ) -> object:
        """Advance this effect's state over ``elapsed`` seconds, before any aircraft measures.

        Called once per tick over the **whole roster in agent order**, at a fixed offset from the
        start, in effect-registration order — the discipline :meth:`CommunicationModel.step`
        applies to its gates. An effect that draws should draw a **constant** number of times
        whatever its state and whatever its parameters, including when a rate is zero, so that
        sweeping a
        parameter moves this effect's outcomes without shifting the measurement draws underneath
        them (ADR 0006 §6).

        Receives whole :class:`~opencdarr.state.AircraftState` values rather than ids, unlike
        :meth:`LinkGate.evolve`, because a GNSS environment depends on *where* an aircraft is —
        urban canyon, terrain masking (ADR 0021 §4). An effect must not read ``desired``: intent is
        private, and steering on it would be reading another aircraft's intentions.
        """

    @abstractmethod
    def quality(self, own: object, aircraft_id: str) -> NavQuality:
        """This aircraft's degradation right now. Must not draw."""


class NavigationModel(ABC):
    """How an aircraft measures its own state to broadcast — the contribution surface."""

    def initial_state(self) -> NavState:
        """The nav layer's state at ``t = 0``.

        The seam for a **stateful** model, exactly as
        :meth:`CommunicationModel.initial_state` is for the channel: a model that must remember
        something across ticks subclasses :class:`NavState` and returns that subclass here, and
        :meth:`measure` then receives its own state type on **every** tick including the first.
        ``effects`` is the other, narrower seam — one effect added to the standard model rather
        than the model replaced (ADR 0021 §5).
        """
        return NavState()

    def evolve(
        self,
        state: NavState,
        aircraft: Sequence[AircraftState],
        t: float,
        rng: np.random.Generator,
    ) -> NavState:
        """Advance the layer's own state to ``t``, before any aircraft measures.

        Non-abstract with a safe default that **draws nothing** and only records the time, so a
        stateless model is bit-for-bit the pre-seam layer and every existing implementation keeps
        working untouched (ADR 0021 §3).
        """
        return replace(state, t_prev=t)

    @abstractmethod
    def measure(
        self, state: NavState, true: AircraftState, t: float, rng: np.random.Generator
    ) -> Message:
        """Return the aircraft's (noisy) self-measurement as a broadcastable :class:`Message`.

        ``state`` is whatever :meth:`evolve` last returned — read it, do not advance it: all
        state advance happens in :meth:`evolve`, once per tick, so the draws sit at a predictable
        place in the stream.
        """

    def validate_ids(self, ids: frozenset[str]) -> None:
        """Raise :class:`ValueError` if this model is configured against unknown aircraft ids.

        The navigation twin of :meth:`CommunicationModel.validate_ids`: a model or effect keyed by
        aircraft id reads an absent key as its default, so a mistyped id is silently ignored rather
        than applied. The fleet composition root calls this with the actual roster so that mistake
        fails loudly. The base accepts anything; a model that keys configuration by id overrides
        it.
        """


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

    A gate whose veto depends on **geometry** — out of surveillance range, terrain-masked — reads
    the roster handed to :meth:`evolve` and keeps whatever it needs of it in its own state;
    :meth:`admits` is then a lookup by id. That is why :meth:`evolve` takes states while
    :meth:`admits` takes ids: the tick's truth snapshot is taken once, at the fixed offset every
    gate evolves at, rather than re-read per link.

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
        receivers: Sequence[AircraftState],
        elapsed: float,
        rng: np.random.Generator,
    ) -> object:
        """Advance this gate's state over ``elapsed`` seconds, before the channel runs.

        ``receivers`` is the whole roster — every aircraft, in agent order, carrying its **true**
        state. True and not the broadcast fix on purpose: whether a link physically closes is a
        fact about where the aircraft are, not about what either of them believes.

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
        receivers: Sequence[AircraftState],
        t: float,
        rng: np.random.Generator,
    ) -> CommState:
        """Return the comm state after offering ``broadcasts`` to ``receivers`` at time ``t``.

        ``receivers`` is the roster as **true** states, not ids: a channel effect that depends on
        geometry (range, terrain) needs the positions, and a model that does not simply reads
        ``.id``. The truth is the right input because a link either physically closes or does not,
        whatever the two ends believe about each other — what they believe is already the
        broadcast's job (``broadcasts`` carry the noisy self-fix).

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
