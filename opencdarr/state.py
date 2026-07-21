"""The owner of state: plain, clonable data.

This is the spine of the design (``docs/design_brief.md``): *you* own the state, and
BlueSky is a library of stateless math, not the runtime. State is therefore an ordinary,
copyable value — not something hidden inside a global (``bs.traf``) or a singleton.

Invariant (load-bearing for rare-event simulation)
--------------------------------------------------
**Everything that influences an aircraft's future must live in its state object; nothing
in a global, a singleton, a module variable, or a closure.** Cloning a particle for the
interacting particle system (IPS, roadmap v0.4) means copying its state; any future-
affecting value kept *outside* the state would be silently shared between clones — which
is exactly the KI-1 bug (``docs/lesson-learnt.md``), and at 1e-9 it would corrupt the
estimate invisibly. Holding this invariant is what lets the state grow field-by-field,
step by step, and stay correct to clone.

Scope
-----
``AircraftState`` is the *certain kinematic core* — a single aircraft's 2D horizontal
point-mass state. It is deliberately not the whole IPS particle: the particle will also
carry per-aircraft CDR / recovery memory (e.g. ``resopairs``, the initial intruder
velocity a recovery criterion compares against) and an RNG substream (``ADR 0001``). Those
are added by the steps that introduce them (CDR: Steps 2-3; estimator: Steps 5-6), each
inside the clonable state, never outside it.

The model is horizontal at fixed altitude, matching every experiment on the roadmap
(recovery criteria, multi-aircraft conflict, rare events). A future 3D extension would add
``alt`` / vertical rate here *and* vertical dynamics, detection, and a 3D level function —
a deliberate, re-validated change recorded as its own ADR, not a set of dead fields now.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from opencdarr.performance import Performance


@dataclass(frozen=True)
class DesiredVelocity:
    """An aircraft's intended (desired/nominal) velocity, as East–North components [m/s].

    Stored as a velocity **vector** (``v_east``, ``v_north``), not polar ``(trk, gs)`` (ADR 0008):
    it is the same representation :class:`~opencdarr.dynamics.Command` uses, so intent and control
    target speak one language, and the intent-based recovery criteria read the components directly
    with no trig at their edge. Build one from a track and speed with
    :meth:`from_track_speed`; read ``trk`` / ``gs`` back as derived properties.

    This is *intent* — where the aircraft wants to go — and it is **private by default**: another
    aircraft perceives it only when intent-sharing is explicitly enabled (``run_encounter``'s
    ``share_intent``). Intent-based recovery (:class:`~opencdarr.crr.FTR`) reads the ownship's own
    ``desired`` to decide whether reverting to it would re-trigger a conflict.

    What the value *means* depends on whose state carries it:

    - On an aircraft's **own** state it is always declared intent, known exactly (it is the
      autopilot target), never observed kinematics.
    - On a **perceived** state it is the best available estimate of that aircraft's intent:
      declared when shared, otherwise *inferred* from its velocity when the conflict pair became
      active (``loop.PairMemory.onset_velocity``). The value does not distinguish the two — a
      consumer cannot tell declared from inferred.
    """

    v_east: float
    v_north: float

    @classmethod
    def from_track_speed(cls, trk: float, gs: float) -> DesiredVelocity:
        """Build from a track [deg, aviation convention] and ground speed [m/s]."""
        r = math.radians(trk)
        return cls(v_east=gs * math.sin(r), v_north=gs * math.cos(r))

    @property
    def gs(self) -> float:
        """Ground speed [m/s] — the vector's magnitude."""
        return math.hypot(self.v_east, self.v_north)

    @property
    def trk(self) -> float:
        """Track [deg, aviation convention 0=N, CW] — direction of the vector (0 if zero)."""
        return math.degrees(math.atan2(self.v_east, self.v_north)) % 360.0


@dataclass(frozen=True)
class AircraftState:
    """One aircraft's 2D horizontal point-mass kinematics.

    Frozen (immutable): a copy can never alias its source, and no attribute — declared
    field or stray — can be assigned after construction, so nothing can smuggle hidden
    state onto an instance. Both serve the no-hidden-state invariant above. Evolve it
    functionally with :func:`dataclasses.replace`, e.g. inside ``step_dynamics`` (Step 1).

    ``slots`` / a NumPy-backed layout is deliberately *not* used yet: it interacts badly
    with ``frozen`` (a known CPython class-recreation wart) and is a memory optimisation we
    take only when IPS profiling shows per-particle object overhead matters — measured, not
    assumed (``design-philosophy.md`` #12).

    Attributes
    ----------
    id:
        Aircraft identifier (e.g. ``"DRO000"``), unique within a scenario.
    lat, lon:
        Position in decimal degrees (WGS84).
    trk:
        Track over ground in degrees, aviation convention (0 = North, increasing
        clockwise). Detection/resolution math converts to radians at its edge.
    gs:
        Ground speed in metres per second (SI internally; unit conversions live at the
        BlueSky boundary, not here).
    turn_rate:
        Current turn rate in degrees per second, signed (positive = clockwise). This is
        *state*, not a derived quantity: the M600 caps how fast the turn rate itself can
        change (``max_dtr2``), so the next step's turn rate is bounded relative to this
        one. It must therefore travel inside the state — an IPS clone that lost it would
        turn differently from its parent. Zero for an aircraft flying straight.
    desired:
        The aircraft's intended (desired/nominal) velocity — its *intent* — or ``None`` when it has
        declared none. Held in the state (not a global) so it clones with the particle;
        :class:`DesiredVelocity` documents its privacy. Intent-based recovery reads it; the
        certain-kinematics algorithms (detection, resolution, past-CPA) ignore it.
    pos_ci95, vel_ci95:
        The aircraft's own **declared measurement accuracy** (95% radial position [m] / velocity
        [m/s]) — a property of *this* aircraft's sensor, not a fixed simulation-wide constant.
        It lives here, not on the navigation model, for the same reason ``turn_rate`` does: it can
        differ per aircraft and evolve over a run (e.g. degrading GPS coverage), so it must travel
        with the state to clone correctly. :class:`~opencdarr.cns.GpsNavigation` reads these off
        the aircraft being measured and copies them onto the broadcast — accuracy is declared
        metadata a receiver gets *with* the message, not something it has to be told separately.
        Zero (default) means a perfect, noiseless sensor.
    """

    id: str
    lat: float
    lon: float
    trk: float
    gs: float
    turn_rate: float = 0.0
    desired: DesiredVelocity | None = None
    pos_ci95: float = 0.0
    vel_ci95: float = 0.0


def create_aircraft(
    perf: Performance,
    *,
    id: str,
    lat: float,
    lon: float,
    trk: float,
    gs: float,
    turn_rate: float = 0.0,
    pos_ci95: float = 0.0,
    vel_ci95: float = 0.0,
) -> AircraftState:
    """Create an :class:`AircraftState`, validating it against the flight envelope.

    The pure-value counterpart of BlueSky's ``cre`` (which mutates a global ``bs.traf``):
    it returns a new state and touches nothing else. Unlike a speed *command* — which
    ``step_dynamics`` clamps into the envelope at runtime — an out-of-envelope *initial*
    condition is a scenario specification error, so this **fails fast** with ``ValueError``
    rather than silently clamping. Direct ``AircraftState(...)`` construction remains for
    internal state evolution (e.g. ``step_dynamics``, whose outputs are in-envelope by
    construction); ``create_aircraft`` is the validated entry point at the scenario boundary.
    """
    if not perf.v_min <= gs <= perf.v_max:
        raise ValueError(
            f"initial ground speed {gs} m/s for {id!r} is outside the envelope "
            f"[{perf.v_min}, {perf.v_max}] m/s"
        )
    if abs(turn_rate) > perf.max_tr:
        raise ValueError(
            f"initial turn rate {turn_rate} deg/s for {id!r} exceeds the max turn rate "
            f"{perf.max_tr} deg/s"
        )
    if pos_ci95 < 0.0 or vel_ci95 < 0.0:
        raise ValueError(f"pos_ci95/vel_ci95 must be >= 0; got {pos_ci95=}, {vel_ci95=}")
    return AircraftState(
        id=id, lat=lat, lon=lon, trk=trk, gs=gs, turn_rate=turn_rate,
        pos_ci95=pos_ci95, vel_ci95=vel_ci95,
    )
