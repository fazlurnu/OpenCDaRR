"""The CNS stack — one object for *how an aircraft comes to know the traffic around it*.

:mod:`~opencdarr.cns.base` defines the three layers separately (**N** navigation, **C**
communication, **S** surveillance); a run needs all three plus their substreams, and always uses
them in the same order. :class:`CNS` is that bundle, and :meth:`CNS.sense` is the one call a
runner makes per broadcast tick:

    self-fix (N) → latch + transmit (C) → what each receiver now holds (S) → :class:`Perception`

Deliberately **not** in here: the *decision* (detect → resolve → recover) that consumes a
:class:`Perception` — that is the CDR core and stays in :class:`~opencdarr.separation.\
SeparationManager` — and the transmit *timing*, which is
:class:`~opencdarr.cns.broadcast.BroadcastSchedule`'s job (a runner decides *who* fires, then
asks the stack what those aircraft know).

Pure given its inputs, and split for cloning (the IPS invariant, ADR 0001/0004): :class:`CNS`
holds only the **immutable** config (the models + ``share_intent``), shared unchanged across every
particle; the **mutable** random generators live in a separate per-particle :class:`CnsStreams`,
and the threaded value state in :class:`CnsState`. A clone deep-copies the value state and
*re-spawns* its streams, so two particles can never draw from one generator — the ADSL bug
(`lesson-learnt.md`) foreclosed one level down.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace

import numpy as np

from opencdarr.cns.base import (
    CommState,
    CommunicationModel,
    Message,
    NavigationModel,
    NavState,
    SurveillanceModel,
)
from opencdarr.cns.surveillance import LastKnown
from opencdarr.state import AircraftState

# module-level singleton, not a call in a field default: LastKnown is stateless (hold-as-is)
_DEFAULT_SURVEILLANCE: SurveillanceModel = LastKnown()


@dataclass(frozen=True)
class Perception:
    """What one aircraft knows at a decision instant — the input to its separation decision.

    ``own`` is its own (possibly noisy) self-fix, with its **own** intent intact: an aircraft
    always knows itself exactly and never learns about itself through the datalink. ``traffic``
    is every *other* aircraft it currently holds, in agent order; one it has never heard from is
    **absent** from the list, not ``None`` in it (ADR 0006 §5: no data ⇒ fly nominal).
    """

    own: AircraftState
    traffic: list[AircraftState]  # built fresh each tick, never shared between aircraft


@dataclass(frozen=True)
class CnsState:
    """The datalink's threaded value state: the comm layer's, plus each aircraft's last transmit.

    ``last_tx[i]`` is what aircraft ``i`` most recently put on the air (``None`` before its first
    transmission) — the perfect-delivery path reads it directly, and it is what the lossy path
    hands to :class:`~opencdarr.cns.base.CommunicationModel`. Threaded rather than held on the
    stack so :class:`CNS` itself stays stateless and a clone can never write through to its parent
    (the :class:`~opencdarr.cns.base.CommState` contract, ADR 0001).
    """

    comm: CommState = field(default_factory=CommState)
    nav: NavState = field(default_factory=NavState)
    last_tx: tuple[AircraftState | None, ...] = ()

    @staticmethod
    def initial(
        n: int,
        communication: CommunicationModel | None = None,
        navigation: NavigationModel | None = None,
    ) -> CnsState:
        """The state at ``t = 0``: nothing delivered, nobody has transmitted yet.

        Each layer's own starting value comes from its *model*
        (:meth:`~opencdarr.cns.base.CommunicationModel.initial_state`,
        :meth:`~opencdarr.cns.base.NavigationModel.initial_state`), so a stateful model finds its
        own state subclass already in place on the first tick. Omitting a model (or running without
        one) gives the plain default, which is what the perfect-delivery path uses.

        Prefer :meth:`CNS.initial_state`, which passes both models for you — the two most expensive
        defects in this package's history were a composition root forgetting to pass one.
        """
        comm = communication.initial_state() if communication is not None else CommState()
        nav = navigation.initial_state() if navigation is not None else NavState()
        return CnsState(comm=comm, nav=nav, last_tx=(None,) * n)


@dataclass(frozen=True)
class CnsStreams:
    """The per-particle RNG substreams the datalink draws from — navigation and communication.

    Kept **out** of :class:`CNS` and passed to :meth:`CNS.sense` on purpose (ADR 0001): a
    generator is mutable state that advances on every draw, so it belongs to the particle, not to
    the shared config. On an IPS clone these are **re-spawned** (``SeedSequence.spawn``), never
    copied, so a cloned future draws an independent stream — the property a 1e-9 estimate rests on.
    ``None`` means that layer draws nothing: ``nav=None`` ⇒ exact self-fixes, ``comm=None`` is only
    valid when the stack has no communication model (:meth:`CNS.sense` checks this).
    """

    nav: np.random.Generator | None = None
    comm: np.random.Generator | None = None


@dataclass(frozen=True)
class CNS:
    """How aircraft fix themselves, transmit, and hear each other — the three layers as one.

    Holds only the **immutable** stack config, so one :class:`CNS` is shared unchanged across every
    IPS particle; the per-particle generators are a separate :class:`CnsStreams` argument to
    :meth:`sense`. Every field is optional, and the default stack is the **perfect-information**
    one: exact self-fixes, instant and lossless delivery, private intent. Adding ``navigation``
    (and a ``nav`` stream) makes each self-fix noisy; adding ``communication`` (and a ``comm``
    stream, its own substream — ADR 0006 §6, never the same generator as ``nav``) makes delivery
    lossy and asymmetric over the n(n−1) directed links. ``share_intent`` decides whether the
    nominal velocity survives onto the air; it is stripped at *transmit* time, not at perceive
    time, so a dropped or held message can never carry intent it wasn't sent with.
    """

    navigation: NavigationModel | None = None
    communication: CommunicationModel | None = None
    surveillance: SurveillanceModel | None = None
    share_intent: bool = False

    def initial_state(self, n: int) -> CnsState:
        """This stack's state at ``t = 0``, for a fleet of ``n`` aircraft.

        Prefer this over calling :meth:`CnsState.initial` directly: it passes *this* stack's
        models, so a stateful model always finds its own state subclass in place on the first tick
        and a composition root cannot wire one layer's state while forgetting another's. That
        failure mode is not hypothetical — the two most expensive defects in this package's
        history were both a composition root omitting a model it should have passed.
        """
        return CnsState.initial(n, self.communication, self.navigation)

    def sense(
        self,
        states: Sequence[AircraftState],
        firing: Sequence[int],
        t: float,
        cns: CnsState,
        streams: CnsStreams,
    ) -> tuple[CnsState, dict[int, Perception]]:
        """Run one broadcast tick: the aircraft in ``firing`` transmit, then everyone's belief.

        Draws from ``streams`` (this particle's generators). Returns the advanced :class:`CnsState`
        and a :class:`Perception` **per firing aircraft** (keyed by index into ``states``) — the
        aircraft that did not fire this tick are not deciding, so they are not asked what they see.

        Order is load-bearing and is the source of the fleet↔pairwise bit-for-bit reduction: all
        navigation draws happen first, in ``firing`` order, then a single communication step for
        the whole tick, then surveillance (which draws nothing). Every firing aircraft therefore
        transmits from a pre-decision snapshot — nobody reacts to a manoeuvre that has not been
        broadcast yet.
        """
        if self.communication is not None and streams.comm is None:
            raise ValueError(
                "communication requires a comm RNG stream (its own substream, ADR 0006 §6)"
            )
        # The nav layer's own state advances first — once, for the whole roster, at a fixed offset
        # from the start of the tick — so what an effect draws does not shift with which aircraft
        # happened to fire (the discipline `Comm.step` applies to its gates). A model with no
        # effects draws nothing here, so a stack without them is bit-for-bit the pre-seam stack
        # (ADR 0021 §3).
        nav_state = cns.nav
        if self.navigation is not None and streams.nav is not None:
            nav_state = self.navigation.evolve(nav_state, states, t, streams.nav)
        last_tx = list(cns.last_tx)
        selfs: dict[int, AircraftState] = {}
        for i in firing:
            if self.navigation is not None and streams.nav is not None:
                fix = self.navigation.measure(nav_state, states[i], t, streams.nav).state
            else:
                fix = states[i]
            selfs[i] = replace(fix, desired=states[i].desired)
            last_tx[i] = replace(fix, desired=states[i].desired if self.share_intent else None)

        comm_state = cns.comm
        if self.communication is not None and streams.comm is not None:
            # each firing aircraft's transmit is offered to every receiver over its own directed
            # link (per-link reception + latency). Broadcasts and receivers stay in agent order so
            # the draw sequence is the pairwise runner's at n = 2. The roster goes down as **true**
            # states, not ids: a geometry-dependent gate reads the positions from it, and the model
            # itself only ever needs `.id` (ADR 0019 §4's hard-availability half).
            broadcasts = [
                Message(source=states[i].id, state=tx, t_meas=t)
                for i in firing
                if (tx := last_tx[i]) is not None  # always true for a firing aircraft
            ]
            comm_state = self.communication.step(
                comm_state, broadcasts, states, t, streams.comm
            )

        surveil = self.surveillance or _DEFAULT_SURVEILLANCE
        perception: dict[int, Perception] = {}
        for i in firing:
            if self.communication is not None:
                # the last message each link actually delivered; never heard ⇒ dropped from the set
                traffic = [
                    p
                    for j in range(len(states))
                    if j != i
                    and (p := surveil.perceived(comm_state, states[i].id, states[j].id, t))
                    is not None
                ]
            else:
                # instant, perfect delivery: the other's latch (None before its first transmit)
                traffic = [tx for j, tx in enumerate(last_tx) if j != i and tx is not None]
            perception[i] = Perception(own=selfs[i], traffic=traffic)

        return CnsState(comm=comm_state, nav=nav_state, last_tx=tuple(last_tx)), perception
