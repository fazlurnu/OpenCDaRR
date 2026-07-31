"""The N-aircraft encounter runner — the fleet environment (Phase 6b), over a stepwise interface.

:func:`run_fleet` is the multi-aircraft generalisation of
:func:`~opencdarr.loop.run_encounter`: a **list of aircraft**, each with its own
:class:`~opencdarr.autopilot.Autopilot` / :class:`~opencdarr.kinematics.Kinematics` /
:class:`~opencdarr.performance.Performance`, all advancing simultaneously. Every aircraft runs its
own detect → resolve → recover against **all the others it perceives** (the cooperative fleet — no
central controller), so in a conflict *everyone* manoeuvres, not just one side. The directed,
pairwise-primitive design (ADR 0004) makes this a change of *environment*, not of the CDR core:
detection iterates the conflict graph, resolution composes the set (MVP sums, VO unions, Phase 6a),
and recovery waits until an aircraft is clear of **all** its conflicts.

**The estimator interface (Phase 8 / ADR 0004).** The environment is split into the three pieces a
rare-event estimator needs, so Monte Carlo *and* the future IPS see only these:

- :class:`FleetEnv` — the **fixed rules** (kinematics, CDR methods, geometry, timing). Immutable
  and shared unchanged across every IPS particle.
- :class:`FleetState` — the **particle**: the whole mutable world (every aircraft's state, guidance
  and recovery memory, held command, broadcast clock, the datalink value state, the clock and the
  measured accumulators). Deeply immutable — :meth:`FleetEnv.advance` returns a *new* state and
  never mutates the old — so an IPS clone is reference-sharing, not a deep copy, and can never
  write through to its parent (the no-hidden-state invariant, the KI-1 fix at scale).
- :class:`FleetStreams` — the **per-particle RNG** (nav + comm + broadcast), re-spawned on clone,
  never copied (ADR 0001).

:meth:`FleetEnv.advance` is one ``dt`` step ``state → state``; :func:`level` is the importance
function (minimum pairwise separation — ADR 0004's starting point, a Phase-8 ADR may refine it);
:meth:`FleetEnv.is_terminal` is the stop test. :func:`run_fleet` is the plain-Monte-Carlo driver
over exactly these — the loop that IPS replaces with resample-and-split.

**Perception**: by default each aircraft sees the others' broadcasts directly (instant, perfect
delivery), with optional GNSS self-noise (``navigation`` + ``rng``). Passing ``communication`` /
``surveillance`` / ``comm_rng`` (6f) makes perception **lossy and asymmetric** over the n(n−1)
directed links — per-link reception + latency, each aircraft acting on the last message *that* link
delivered (or ``None`` before first contact ⇒ fly nominal) — mirroring :func:`run_encounter`.

**Reduces to the pairwise runner** at n = 2: ``run_fleet`` with two agents reproduces
:func:`~opencdarr.loop.run_encounter` (no-communication path) bit-for-bit — the free multi-aircraft
regression (ADR 0004). Pure given its inputs; no globals.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, replace

import numpy as np

from opencdarr import geo
from opencdarr.autopilot import Autopilot, CruiseAutopilot, GuidanceMemory, nominal_velocity
from opencdarr.cd.base import ConflictDetector
from opencdarr.cns.base import (
    CommunicationModel,
    NavigationModel,
    SurveillanceModel,
)
from opencdarr.cns.broadcast import BroadcastSchedule
from opencdarr.cns.stack import CNS, CnsState, CnsStreams
from opencdarr.cr.base import ConflictResolver
from opencdarr.crr.base import RecoveryCriterion
from opencdarr.kinematics import Kinematics, MotionCommand
from opencdarr.loop import _DEFAULT_KINEMATICS, _setpoint_adapter
from opencdarr.performance import Performance
from opencdarr.relative import Relative, relative_enu, segment_min_range
from opencdarr.separation import INACTIVE, FleetMemory, SeparationManager, SetpointAdapter
from opencdarr.state import AircraftState, DesiredVelocity
from opencdarr.wind import NO_WIND, WindField

_DEFAULT_SCHEDULE = BroadcastSchedule()  # interval 1 s, aligned, no jitter (default singleton)
_BROADCAST_EPS = 1e-9  # float guard so a tick lands on a broadcast time reached by dt steps


@dataclass(frozen=True)
class Agent:
    """One aircraft's bundle in the fleet: its state + how it navigates, flies, and is limited.

    ADR 0011 §7 deferred a per-aircraft grouping "until a real grouping need appears"; N parallel
    lists is that need, so the bundle lands here. ``kinematics`` defaults to the shared
    :class:`~opencdarr.kinematics.Multirotor`; ``autopilot`` defaults to a
    :class:`~opencdarr.autopilot.CruiseAutopilot` holding the state's initial cruise.
    """

    state: AircraftState
    perf: Performance
    kinematics: Kinematics | None = None
    autopilot: Autopilot | None = None

    def __post_init__(self) -> None:
        # Fail at the line the mismatch is written, not deep inside the first step. Only the
        # explicit case can be checked here; when ``kinematics`` is left to its default the
        # effective model is not known until the composition root, which re-validates there (see
        # ``build``).
        if self.kinematics is not None:
            self.kinematics.validate_performance(self.perf)


@dataclass(frozen=True)
class FleetOutcome:
    """What one fleet encounter produced (measured on the true states, every step).

    ``frames`` is the run's **states log** — a :class:`StatesLog` wrapping every
    :class:`FleetState` the encounter went through (``frames[0]`` the initial state, each later
    frame one ``dt`` step of :meth:`FleetEnv.advance`, ``frames[-1]`` the terminal state the
    scalars are read from). It is populated only by ``run_fleet(..., record=True)`` and is ``None``
    otherwise, so a plain run stays a cheap scalar. :class:`StatesLog` indexes and iterates like a
    tuple of frames but prints as a one-liner, so ``repr`` of the outcome stays readable. Because a
    :class:`FleetState` already holds the true ``states``, clock, memory and min-sep as an
    immutable value, this log *is* the run — nothing is recomputed. The log is raw data; plotting
    it is a separate tool (:mod:`opencdarr.viz`)."""

    conflict: bool  # was any directed pair predicted in conflict at any step?
    los: bool  # was any pair ever in loss of separation?
    min_sep: float  # minimum pairwise separation reached across all pairs [m]
    frames: StatesLog | None = None  # states log, only when record=True


@dataclass(frozen=True)
class FleetState:
    """The particle: the entire mutable world of one fleet encounter, as a deeply immutable value.

    Everything an encounter's future depends on lives here and nowhere else (ADR 0004): the true
    ``states``; each aircraft's guidance memory (``gms``), recovery memory (``mems``), held command
    (``cmds``) and broadcast clock (``next_bc``); the datalink value state (``cns_state``); the
    clock (``t``, ``done_timer``); and the outcome accumulators (``conflict`` / ``los`` /
    ``min_sep``) measured on the true states so far. Every field is itself immutable, so the whole
    value is — :meth:`FleetEnv.advance` builds a *new* ``FleetState`` and never touches the old.
    That is what makes an IPS clone free and safe: share the reference, re-spawn the streams.
    """

    states: tuple[AircraftState, ...]
    gms: tuple[GuidanceMemory, ...]
    mems: tuple[FleetMemory, ...]
    cmds: tuple[MotionCommand, ...]
    next_bc: tuple[float, ...]
    cns_state: CnsState
    t: float
    done_timer: float
    conflict: bool
    los: bool
    min_sep: float


@dataclass(frozen=True, repr=False)
class StatesLog:
    """A recorded run's states log, wrapped as one tidy object instead of a bare tuple of frames.

    Holds every :class:`FleetState` the run passed through and behaves like an immutable sequence
    of them — index it (``log[0]``, ``log[-1]``), iterate it, take its ``len`` — but **prints as a
    one-line summary** so ``repr(FleetOutcome)`` stays readable rather than dumping the whole
    trajectory. ``run_fleet(..., record=True)`` puts one of these on :attr:`FleetOutcome.frames`.
    """

    frames: tuple[FleetState, ...]

    def __len__(self) -> int:
        return len(self.frames)

    def __iter__(self) -> Iterator[FleetState]:
        return iter(self.frames)

    def __getitem__(self, index: int) -> FleetState:
        return self.frames[index]

    def __repr__(self) -> str:
        if not self.frames:
            return "StatesLog(empty)"
        return (f"StatesLog({len(self.frames)} frames, "
                f"t={self.frames[0].t:.1f}→{self.frames[-1].t:.1f}s)")


@dataclass(frozen=True)
class FleetStreams:
    """The per-particle RNG substreams the environment draws from — datalink (nav + comm) and
    broadcast jitter. Kept out of :class:`FleetState` and re-spawned (never copied) on an IPS
    clone, so a cloned future draws independent randomness (ADR 0001), as :class:`CnsStreams`."""

    cns: CnsStreams = CnsStreams()
    broadcast: np.random.Generator | None = None


def _pairwise_min_sep(states: tuple[AircraftState, ...] | list[AircraftState]) -> float:
    """Smallest separation over all unordered pairs [m], at this instant."""
    smallest = float("inf")
    for i in range(len(states)):
        for j in range(i + 1, len(states)):
            _, dist = geo.qdrdist(states[i].lat, states[i].lon, states[j].lat, states[j].lon)
            smallest = min(smallest, dist)
    return smallest


def _pairwise_relative(
    states: tuple[AircraftState, ...] | list[AircraftState]
) -> tuple[Relative, ...]:
    """Relative position of every unordered pair, in the fixed ``i < j`` order [m].

    The vector form of :func:`_pairwise_min_sep` — same one ``geo.qdrdist`` per pair, so it costs
    essentially the same, but it returns the geometry rather than only its magnitude. That is what
    :func:`_segment_min_sep` needs to close the gap *between* two sampled instants.
    """
    return tuple(
        relative_enu(states[i], states[j])
        for i in range(len(states))
        for j in range(i + 1, len(states))
    )


def _segment_min_sep(pre: tuple[Relative, ...], post: tuple[Relative, ...]) -> float:
    """Smallest separation over all pairs across a whole ``dt`` step, endpoints included [m].

    Interpolates each pair's relative position linearly between the two ends of the step and takes
    the closed-form minimum of that segment. Sampling separation only at the step *endpoints*
    misses a pass that dips inside a threshold and back out within one step, and the miss is
    one-sided — it can only report *more* separation than there was. The bias is negligible against
    ``rpz`` but severe at the small radii IPS splits on: the relative error in
    ``P(min_sep <= d)`` goes as ``(v_rel*dt)^2 / (24 d^2)``, i.e. it grows as the target tightens.
    See ``vault/observations/segment-min-separation.md`` for the measurements.

    The per-pair algebra is :func:`~opencdarr.relative.segment_min_range`, shared with
    :func:`~opencdarr.loop.run_encounter` so the two runners cannot drift apart on the n = 2
    reduction.
    """
    return min(
        (segment_min_range(r0, r1) for r0, r1 in zip(pre, post, strict=True)),
        default=float("inf"),
    )


def level(state: FleetState) -> float:
    """The importance function IPS splits on: the fleet's **current** minimum pairwise separation
    [m], smaller = closer to the rare event (ADR 0004's starting point; a Phase-8 ADR may refine
    it for simultaneous multi-aircraft conflict). A pure read of ``state``, independent of N."""
    return _pairwise_min_sep(state.states)


def _all_clear(states: list[AircraftState], mems: tuple[FleetMemory, ...], rpz: float) -> bool:
    """Is the whole fleet done — every pair past CPA and separated, and nobody still resolving?"""
    if any(m.resolving for m in mems):
        return False
    for i in range(len(states)):
        for j in range(i + 1, len(states)):
            rel = relative_enu(states[i], states[j])
            diverging = rel.rx * rel.vx + rel.ry * rel.vy > 0.0
            if not (diverging and rel.dist >= rpz):
                return False
    return True


@dataclass(frozen=True)
class FleetEnv:
    """The fixed rules of a fleet encounter — everything a particle's future depends on that is
    *not* the particle (ADR 0004). Immutable and shared unchanged across every IPS clone; only the
    state and the streams differ between particles. Built by :func:`run_fleet` from its arguments
    and the fleet's :class:`Agent` bundles; exposes the estimator interface :meth:`advance` /
    :meth:`is_terminal` (with the free function :func:`level`).
    """

    kinematics: tuple[Kinematics, ...]
    perfs: tuple[Performance, ...]
    adapters: tuple[SetpointAdapter | None, ...]
    aps: tuple[Autopilot, ...]
    separation: SeparationManager
    detector: ConflictDetector
    resolver: ConflictResolver | None
    recovery: RecoveryCriterion | None
    cns: CNS
    schedule: BroadcastSchedule
    wind: WindField
    rpz: float
    t_lookahead: float
    dt: float
    t_max: float
    done_timeout: float
    # if set, also stop once every goal-carrying aircraft is within this many metres of its goal
    stop_within: float | None = None

    def initial_state(self, agents: list[Agent]) -> FleetState:
        """The particle at ``t = 0``: each aircraft's intent stamped on its true state, its first
        nominal command computed, memories empty, broadcast clocks per the schedule."""
        n = len(agents)
        states = [
            replace(a.state, desired=DesiredVelocity.from_track_speed(a.state.trk, a.state.gs))
            for a in agents
        ]
        gms: list[GuidanceMemory] = []
        cmds: list[MotionCommand] = []
        for i in range(n):
            cmd, gm = self.aps[i].step(states[i], GuidanceMemory(), self.perfs[i])
            cmds.append(cmd)
            gms.append(gm)
        return FleetState(
            states=tuple(states),
            gms=tuple(gms),
            mems=tuple(INACTIVE for _ in range(n)),
            cmds=tuple(cmds),
            next_bc=tuple(self.schedule.initial(n)),
            cns_state=self.cns.initial_state(n),
            t=0.0,
            done_timer=0.0,
            conflict=False,
            los=False,
            min_sep=float("inf"),
        )

    def is_terminal(self, state: FleetState) -> bool:
        """Whether the encounter is over: the fleet has been clear for ``done_timeout`` (every pair
        past CPA and separated, nobody resolving) or the ``t_max`` cap is reached. The plain MC /
        v0.3 stop test; a Phase-8 ADR adds the absorbing rare-event stop for IPS on top.

        With ``stop_within`` set, the run *also* ends once every goal-carrying aircraft is within
        that many metres of its final waypoint — a mission-completion stop, independent of the
        conflict clearing (see :meth:`Autopilot.goal`)."""
        if state.t >= self.t_max or state.done_timer >= self.done_timeout:
            return True
        return self.stop_within is not None and self._at_goals(state, self.stop_within)

    def _at_goals(self, state: FleetState, radius: float) -> bool:
        """Whether every aircraft that has a goal is within ``radius`` m of its final waypoint.

        Goal-less aircraft (a :class:`~opencdarr.autopilot.CruiseAutopilot`) are skipped; returns
        ``False`` when *no* aircraft has a goal, so the stop-at-waypoint condition never fires for
        a fleet that has nowhere to arrive.
        """
        any_goal = False
        for ac, ap in zip(state.states, self.aps, strict=True):
            goal = ap.goal()
            if goal is None:
                continue
            any_goal = True
            _, dist = geo.qdrdist(ac.lat, ac.lon, goal[0], goal[1])
            if dist > radius:
                return False
        return any_goal

    def advance(self, state: FleetState, streams: FleetStreams) -> FleetState:
        """One ``dt`` step of the whole fleet, ``state → state`` (pure; the old ``while`` body).

        Measures the true states (conflict / LoS / running min-sep), lets every aircraft whose
        broadcast clock is due sense-and-decide on the datalink, holds each command while the
        kinematics integrate one ``dt``, and updates the done-timer. Draws only from ``streams``;
        the returned :class:`FleetState` is new and the input is untouched.
        """
        n = len(state.states)
        states = list(state.states)
        gms = list(state.gms)
        mems = list(state.mems)
        cmds = list(state.cmds)
        next_bc = list(state.next_bc)
        cns_state = state.cns_state
        t = state.t

        # detect on the true states, before any decision or step (top of the old loop). The
        # separation measurement needs both ends of the step, so it is taken *after* integrating;
        # only the pre-step geometry is captured here.
        rel_pre = _pairwise_relative(states)
        conflict = state.conflict or any(
            i != j and self.detector.detect(states[i], states[j], self.rpz, self.t_lookahead)
            for i in range(n)
            for j in range(n)
        )

        # aircraft whose own broadcast clock is due this tick: all of them together in the aligned
        # default, a per-aircraft subset once phases are offset
        firing = self.schedule.due(next_bc, t, _BROADCAST_EPS)
        if firing:
            # the datalink runs first and whole (fix → transmit → hear), so every firing aircraft
            # transmits a pre-decision snapshot and nobody reacts to an unbroadcast manoeuvre
            cns_state, perception = self.cns.sense(states, firing, t, cns_state, streams.cns)
            for i in firing:
                see = perception[i]
                nom, gms[i] = self.aps[i].step(see.own, gms[i], self.perfs[i])
                # intent as a velocity: what this aircraft would fly if it reverted to nominal now
                # (the live mission command), stamped for the decision and persisted on the true
                # state for the next transmit. Byte-identical for a frozen CruiseAutopilot.
                self_i = replace(see.own, desired=nominal_velocity(nom, see.own))
                states[i] = replace(states[i], desired=self_i.desired)
                cmds[i], mems[i] = self.separation.step(
                    self_i, see.traffic, nom, mems[i], self.rpz, self.t_lookahead,
                    self.detector, self.resolver, self.recovery, self.adapters[i],
                )
                # next broadcast time: a fixed interval, or dithered per transmission by the
                # schedule's jitter (ADS-B slot randomisation), drawn in agent order
                next_bc[i] = self.schedule.advance(next_bc[i], streams.broadcast)

        # advance all aircraft from their pre-step states (explicitly simultaneous)
        states = [self.kinematics[i].step(states[i], cmds[i], self.perfs[i], self.dt, self.wind)
                  for i in range(n)]
        t += self.dt

        # measure separation over the whole step just flown, not only at its ends — consecutive
        # segments share an endpoint, so the running minimum covers the trajectory continuously
        # from t=0 (the first segment's left end) rather than at a comb of sampled instants
        cur = _segment_min_sep(rel_pre, _pairwise_relative(states))
        min_sep = min(state.min_sep, cur)
        los = state.los or cur < self.rpz

        clear = _all_clear(states, tuple(mems), self.rpz)
        done_timer = state.done_timer + self.dt if clear else 0.0

        return FleetState(
            states=tuple(states),
            gms=tuple(gms),
            mems=tuple(mems),
            cmds=tuple(cmds),
            next_bc=tuple(next_bc),
            cns_state=cns_state,
            t=t,
            done_timer=done_timer,
            conflict=conflict,
            los=los,
            min_sep=min_sep,
        )


def build_env(
    agents: list[Agent],
    *,
    rpz: float,
    t_lookahead: float,
    dt: float,
    detector: ConflictDetector,
    resolver: ConflictResolver | None = None,
    recovery: RecoveryCriterion | None = None,
    wind: WindField = NO_WIND,
    navigation: NavigationModel | None = None,
    communication: CommunicationModel | None = None,
    surveillance: SurveillanceModel | None = None,
    t_max: float = 600.0,
    done_timeout: float = 10.0,
    schedule: BroadcastSchedule = _DEFAULT_SCHEDULE,
    share_intent: bool = False,
    stop_within: float | None = None,
) -> FleetEnv:
    """Assemble the fixed rules of a fleet encounter into a :class:`FleetEnv` (the shared,
    immutable half of the estimator interface). This is the composition root :func:`run_fleet`
    drives and IPS (Phase 8) reuses per particle — everything here is per-run configuration, *not*
    per-particle RNG (that is :class:`FleetStreams`) or world state (that is :class:`FleetState`).
    The per-aircraft tuples default off the ``agents`` bundle: shared
    :class:`~opencdarr.kinematics.Multirotor`, a velocity→course adapter only for a fixed-wing, and
    a frozen :class:`CruiseAutopilot` at each aircraft's initial cruise unless it carries its own
    mission autopilot.
    """
    n = len(agents)
    kinematics = tuple(a.kinematics or _DEFAULT_KINEMATICS for a in agents)
    perfs = tuple(a.perf for a in agents)
    # Backstop for the default-kinematics case: an Agent that left ``kinematics=None`` was
    # validated against nothing at construction, so check the resolved model here (explicit agents
    # re-validate harmlessly — the check is cheap and idempotent).
    for i in range(n):
        kinematics[i].validate_performance(perfs[i])
    ids = frozenset(a.state.id for a in agents)
    if communication is not None:
        communication.validate_ids(ids)
    if navigation is not None:
        navigation.validate_ids(ids)
    return FleetEnv(
        kinematics=kinematics,
        perfs=perfs,
        adapters=tuple(_setpoint_adapter(kinematics[i], perfs[i]) for i in range(n)),
        aps=tuple(a.autopilot or CruiseAutopilot(a.state.trk, a.state.gs) for a in agents),
        separation=SeparationManager(),  # stateless; memory rides in state.mems (ADR 0011 §5)
        detector=detector,
        resolver=resolver,
        recovery=recovery,
        # the datalink as one stack (N → C → S): CNS is the shared, immutable config; its streams
        # ride in the per-particle FleetStreams and its value state threads inside FleetState
        cns=CNS(
            navigation=navigation,
            communication=communication,
            surveillance=surveillance,
            share_intent=share_intent,
        ),
        schedule=schedule,
        wind=wind,
        rpz=rpz,
        t_lookahead=t_lookahead,
        dt=dt,
        t_max=t_max,
        done_timeout=done_timeout,
        stop_within=stop_within,
    )


def run_fleet(
    agents: list[Agent],
    *,
    rpz: float,
    t_lookahead: float,
    dt: float,
    detector: ConflictDetector,
    resolver: ConflictResolver | None = None,
    recovery: RecoveryCriterion | None = None,
    wind: WindField = NO_WIND,
    navigation: NavigationModel | None = None,
    rng: np.random.Generator | None = None,
    communication: CommunicationModel | None = None,
    surveillance: SurveillanceModel | None = None,
    comm_rng: np.random.Generator | None = None,
    t_max: float = 600.0,
    done_timeout: float = 10.0,
    schedule: BroadcastSchedule = _DEFAULT_SCHEDULE,
    broadcast_rng: np.random.Generator | None = None,
    share_intent: bool = False,
    stop_within: float | None = None,
    record: bool = False,
) -> FleetOutcome:
    """Advance the fleet to termination and report its outcome (see the module docstring).

    The plain-Monte-Carlo driver over the estimator interface: build the :class:`FleetEnv` (the
    fixed rules) and the initial :class:`FleetState` (the particle), then step ``advance`` until
    ``is_terminal``. IPS (Phase 8) replaces this loop with resample-and-split over the *same*
    ``advance`` / :func:`level` / ``is_terminal`` — this function is the reference it is validated
    against.

    Each aircraft decides on the broadcast cadence from its (optionally noisy) self-fix against its
    perceived traffic. Without ``communication`` this is every *other* aircraft's current broadcast
    (perfect delivery); with it, each aircraft reads :class:`SurveillanceModel`'s ``perceived`` per
    directed link — the last message *that* link delivered, or ``None`` (absent) before first
    contact, so that neighbour is dropped from the perceived set until first heard. The command
    is held while the kinematics integrate at ``dt``; all aircraft advance together. The outcome
    (conflict / LoS / min-sep) is measured on the **true** states every step. Terminates once every
    pair is diverging and separated and no aircraft is resolving for ``done_timeout``, or at
    ``t_max``.

    ``schedule`` (a :class:`~opencdarr.cns.broadcast.BroadcastSchedule`) owns the transmit timing —
    the interval, an optional per-aircraft phase offset, and optional per-transmission jitter. The
    default (interval 1 s, aligned phase, no jitter) is today's behaviour and the reduction to
    :func:`~opencdarr.loop.run_encounter` at n = 2. A non-zero ``schedule.jitter`` requires
    ``broadcast_rng`` (its own substream, ADR 0006 §6); aircraft ``i`` broadcasts *and* decides on
    its own clock, reading each other aircraft's **last** transmitted state rather than a
    synchronous snapshot.

    By default the run ends once the fleet has been clear for ``done_timeout`` (conflict resolved,
    every pair past CPA and separated) or at the ``t_max`` cap. Pass ``stop_within`` (metres) to
    add a **mission-completion** stop: the run also ends once every aircraft with a waypoint goal
    is within that distance of its final waypoint, whichever comes first. A goal-less aircraft (a
    plain cruise) is ignored; a fixed-wing *orbits* its final waypoint at its loiter radius, so
    give ``stop_within`` at least that radius (default 80 m) for a fixed-wing to register.

    With ``record=True`` the returned :class:`FleetOutcome` also carries ``frames`` — the full
    states log (every :class:`FleetState`, ``frames[0]`` initial through the terminal frame).
    Recording only appends the states the loop already produces, so the trajectory is identical to
    ``record=False``; it is opt-in because the log grows with the fleet size and run length.
    Plotting that log is a separate tool (:mod:`opencdarr.viz`).
    """
    if schedule.jitter > 0.0 and broadcast_rng is None:
        raise ValueError("broadcast jitter requires broadcast_rng (a substream, ADR 0006 §6)")
    env = build_env(
        agents, rpz=rpz, t_lookahead=t_lookahead, dt=dt, detector=detector, resolver=resolver,
        recovery=recovery, wind=wind, navigation=navigation, communication=communication,
        surveillance=surveillance, t_max=t_max, done_timeout=done_timeout, schedule=schedule,
        share_intent=share_intent, stop_within=stop_within,
    )
    streams = FleetStreams(cns=CnsStreams(nav=rng, comm=comm_rng), broadcast=broadcast_rng)

    state = env.initial_state(agents)
    frames: list[FleetState] | None = [state] if record else None
    while not env.is_terminal(state):
        state = env.advance(state, streams)
        if frames is not None:
            frames.append(state)
    return FleetOutcome(
        conflict=state.conflict, los=state.los, min_sep=state.min_sep,
        frames=StatesLog(tuple(frames)) if frames is not None else None,
    )
