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
point-mass state — plus two **odometry accumulators** (``flight_time``, ``distance_flown``)
the kinematics advance each step (ADR 0010). The accumulators are diagnostics, not kinematics
inputs: nothing reads them back to decide the next step, but they must live *here* rather than
be recomputed by the loop, because an IPS clone taken mid-flight has to carry its parent's
elapsed time and path length with it. It is deliberately not the whole IPS particle: the
particle will also carry per-aircraft CDR / recovery memory (e.g. ``resopairs``, the initial
intruder velocity a recovery criterion compares against) and an RNG substream (``ADR 0001``).
Those are added by the steps that introduce them (CDR: Steps 2-3; estimator: Steps 5-6), each
inside the clonable state, never outside it.

Not stored, on purpose (ADR 0010): the East/North velocity components — derivable from ``(trk,
gs)`` via :func:`~opencdarr.relative.velocity_enu`, so a stored copy would be a second source of
truth that can drift. A heading distinct from ``trk`` (``yaw``) *is* now stored — the
independent-yaw consumer that gives it meaning exists (the
:class:`~opencdarr.kinematics.Multirotor` model, ADR 0012), so the field lands with that model
exactly as this note anticipated, rather than as a copy of ``trk``; it defaults to ``None`` (nose
aligned with track) so every existing construction is unchanged. Altitude / vertical rate remain
deferred to a future 3D ADR.

The model is horizontal at fixed altitude, matching every experiment on the roadmap
(recovery criteria, multi-aircraft conflict, rare events). A future 3D extension would add
``alt`` / vertical rate here *and* vertical kinematics, detection, and a 3D level function —
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
    it is the same representation :class:`~opencdarr.kinematics.Command` uses, so intent and
    control target speak one language, and the intent-based recovery criteria read the components
    directly with no trig at their edge. Build one from a track and speed with
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
    functionally with :func:`dataclasses.replace`, e.g. inside a
    :class:`~opencdarr.kinematics.Kinematics` step.

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
    yaw:
        The direction the airframe's nose points, in degrees (aviation convention), **decoupled
        from the direction of travel** ``trk``. ``None`` (default) means the nose is aligned with
        the track (no independent yaw has been commanded) — so every construction that predates the
        independent-yaw model reads unchanged, and a coupled-heading airframe never has to set it.
        A concrete value is an independently-controlled heading: a
        :class:`~opencdarr.kinematics.Multirotor` can translate one way while pointing another
        (camera-pointing missions), converging ``yaw`` toward a commanded ``target_yaw`` under its
        yaw-rate limit, independent of ``trk`` (ADR 0012). It is *state*, not derived — like
        ``bank`` it must clone with the particle — and under wind it becomes the heading ``ψ``
        whose difference from track is the crab angle (Phase 5). A
        :class:`~opencdarr.kinematics.FixedWing` always carries ``yaw`` as its heading ``ψ`` (nose
        = airspeed vector); at zero wind it equals ``trk``.
    bank:
        Bank (roll) angle ``φ`` in degrees, signed (positive = right bank). *State*, not derived:
        a :class:`~opencdarr.kinematics.FixedWing` limits how fast bank can change (roll rate
        ``roll_rate_max``), so the next step's bank is bounded relative to this one — an IPS clone
        that lost it would roll differently from its parent (the same reason the deleted
        turn-rate-limited model carried its turn rate in state). The coordinated-turn yaw rate is
        ``ψ̇ = g·tan φ / V_TAS`` (ADR 0013). Zero (default) for level flight; a multirotor never
        banks and leaves it at zero.
    desired:
        The aircraft's intended (desired/nominal) velocity — its *intent* — or ``None`` when it has
        declared none. Held in the state (not a global) so it clones with the particle;
        :class:`DesiredVelocity` documents its privacy. Intent-based recovery reads it; the
        certain-kinematics algorithms (detection, resolution, past-CPA) ignore it.
    pos_ci95, vel_ci95:
        The aircraft's own **declared measurement accuracy** (95% radial position [m] / velocity
        [m/s]) — a property of *this* aircraft's sensor, not a fixed simulation-wide constant.
        It lives here, not on the navigation model, for the same reason ``bank`` does: it can
        differ per aircraft and evolve over a run (e.g. degrading GPS coverage), so it must travel
        with the state to clone correctly. :class:`~opencdarr.cns.GnssNavigation` reads these off
        the aircraft being measured and copies them onto the broadcast — accuracy is declared
        metadata a receiver gets *with* the message, not something it has to be told separately.
        Zero (default) means a perfect, noiseless sensor.
    flight_time:
        Seconds this aircraft has been advanced (odometry accumulator, ADR 0010). Every
        :class:`~opencdarr.kinematics.Kinematics` step adds ``dt``. A diagnostic (no kinematics
        reads it back), but it rides in the state so an IPS clone inherits the parent's elapsed
        time. Zero (default) for a freshly created aircraft.
    distance_flown:
        Ground path length in metres this aircraft has covered (odometry accumulator, ADR 0010).
        Every step adds ``gs * dt`` (the odometer reading), so a there-and-back path keeps growing
        even as net displacement returns toward the start. Same rationale as ``flight_time``. Zero
        (default) at creation.
    """

    id: str
    lat: float
    lon: float
    trk: float
    gs: float
    yaw: float | None = None
    bank: float = 0.0
    desired: DesiredVelocity | None = None
    pos_ci95: float = 0.0
    vel_ci95: float = 0.0
    flight_time: float = 0.0
    distance_flown: float = 0.0


def create_aircraft(
    perf: Performance,
    *,
    id: str,
    lat: float,
    lon: float,
    trk: float,
    gs: float,
    bank: float = 0.0,
    pos_ci95: float = 0.0,
    vel_ci95: float = 0.0,
) -> AircraftState:
    """Create an :class:`AircraftState`, validating it against the flight envelope.

    The pure-value counterpart of BlueSky's ``cre`` (which mutates a global ``bs.traf``):
    it returns a new state and touches nothing else. Unlike a command — which a
    :class:`~opencdarr.kinematics.Kinematics` step clamps into the envelope at runtime — an
    out-of-envelope *initial* condition is a scenario specification error, so this **fails fast**
    with ``ValueError`` rather than silently clamping. Direct ``AircraftState(...)`` construction
    remains for internal state evolution (a ``Kinematics.step``'s outputs are in-envelope by
    construction); ``create_aircraft`` is the validated entry point at the scenario boundary.
    """
    if not perf.v_min <= gs <= perf.v_max:
        raise ValueError(
            f"initial ground speed {gs} m/s for {id!r} is outside the envelope "
            f"[{perf.v_min}, {perf.v_max}] m/s"
        )
    if abs(bank) > perf.phi_max:
        raise ValueError(
            f"initial bank {bank} deg for {id!r} exceeds the max bank angle {perf.phi_max} deg"
        )
    if pos_ci95 < 0.0 or vel_ci95 < 0.0:
        raise ValueError(f"pos_ci95/vel_ci95 must be >= 0; got {pos_ci95=}, {vel_ci95=}")
    return AircraftState(
        id=id, lat=lat, lon=lon, trk=trk, gs=gs, bank=bank,
        pos_ci95=pos_ci95, vel_ci95=vel_ci95,
    )
