"""The dynamics boundary: one pure step of turn-rate-limited point-mass motion.

``step_dynamics`` is the extracted, BlueSky-free integrator that the whole design hangs on
(``design_brief.md`` Decision #4): it advances one aircraft one time step, honouring the
airframe's turn-rate, turn-acceleration, and speed limits (via ``Performance`` — the airframe
is a value it reads, not code it hard-codes, ``performance.py``), as a pure ``state -> state``
map. The position update uses our own geodesy (``opencdarr.geo``, ADR 0003), so shipping code
has no BlueSky dependency; the boundary stays swappable (``design-philosophy.md`` #5).

Governing equations: ``vault/derivations/step-dynamics-m600.md`` (symbols match the code).

:class:`Dynamics` is the contribution surface (ADR 0007) for this boundary — mirroring
:class:`~opencdarr.cd.base.ConflictDetector` and the other model-family ABCs: a new physical
effect (wind, a different airframe class) subclasses it and is passed into
:func:`~opencdarr.loop.run_encounter` as ``dynamics=...``, rather than forking the loop.
:class:`PointMassDynamics` — wrapping :func:`step_dynamics` — is the default, no-wind
implementation.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, replace

from opencdarr import geo
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


@dataclass(frozen=True)
class Command:
    """A control command: the desired ground **velocity** vector, East–North [m/s] (ADR 0008).

    A command says *where the aircraft wants to go*, as a velocity vector — not a heading and a
    speed. This keeps the control interface neutral about the airframe: a :class:`Dynamics` model
    decides how to chase the target vector (a turn-rate-limited point mass reconstructs a track
    and turns toward it; a holonomic model could drive ``v_east`` / ``v_north`` directly). The
    resolvers (:class:`~opencdarr.cr.MVP`, :class:`~opencdarr.cr.VO`) already compute a velocity
    vector internally, so they now return one with no polar round-trip.

    Build one from an aviation heading and speed with :meth:`from_track_speed`; read ``trk`` /
    ``gs`` back as derived properties. A zero vector has no defined direction (``trk`` returns 0)
    — the point-mass model reads that as "hold current heading" (see :func:`step_dynamics`).
    Backward flight (facing decoupled from travel), which the old signed-speed command could
    express, is deliberately not representable here — it belongs to a future yaw-carrying state,
    not the velocity command (ADR 0008).

    Attributes
    ----------
    v_east, v_north:
        Desired ground-velocity components, East and North, in metres per second.
    """

    v_east: float
    v_north: float

    @classmethod
    def from_track_speed(cls, hdg: float, spd: float) -> Command:
        """Build a command from an aviation heading [deg] and ground speed [m/s]."""
        r = math.radians(hdg)
        return cls(v_east=spd * math.sin(r), v_north=spd * math.cos(r))

    @property
    def gs(self) -> float:
        """Commanded ground speed [m/s] — the vector's magnitude."""
        return math.hypot(self.v_east, self.v_north)

    @property
    def trk(self) -> float:
        """Commanded track [deg, aviation convention] — direction of the vector (0 if zero)."""
        return math.degrees(math.atan2(self.v_east, self.v_north)) % 360.0


def _clip(value: float, low: float, high: float) -> float:
    """Clamp ``value`` to ``[low, high]``."""
    return max(low, min(value, high))


_SPD_EPS = 1e-9  # m/s: below this a command has no meaningful direction -> hold current heading


def step_dynamics(
    state: AircraftState,
    command: Command,
    perf: Performance,
    dt: float,
) -> AircraftState:
    """Advance one aircraft by ``dt`` seconds under a velocity-vector command.

    The point-mass model faces its direction of travel, so it reads the command's *magnitude*
    (``command.gs``) as the target speed and *direction* (``command.trk``) as the target track,
    then turn-rate-limits toward it — the polar reconstruction the airframe assumes lives here,
    not in the shared :class:`Command` (ADR 0007/0008). Pure: the returned
    :class:`AircraftState` is a function of the arguments alone; no global or module state is read
    or written. Steps (see the derivation for the full math):

    1. speed           ``target = clip(|v_cmd|, v_min, v_max)``;
                       ``gs' = gs + clip(target - gs, ±ax*dt)``  (clamp, then ramp)
    2. heading error   ``e = ((trk(v_cmd) - trk + 180) mod 360) - 180``  (signed, shortest way);
                       hold current track when ``|v_cmd|`` is ~0 (a zero vector has no direction)
    3. turn limiter    ``w_des = clip(e, ±max_tr)``;
                       ``w' = clip(w + clip(w_des - w, ±max_dtr2*dt), ±max_tr)``
    4. heading         integrate ``trk + dt*w'``, or snap to the target track if reachable
    5. position        move ``gs'*dt`` metres along ``trk'`` via ``geo.forward``
    """
    # 1. speed: clamp the command's magnitude into the envelope, then ramp toward it at no more
    #    than ax*dt (the acceleration analogue of the max_dtr2 turn-rate limit)
    cmd_gs = command.gs
    target_gs = _clip(cmd_gs, perf.v_min, perf.v_max)
    gs = state.gs + _clip(target_gs - state.gs, -perf.ax * dt, perf.ax * dt)

    # 2. heading error, signed and taken the short way round. A zero-velocity command carries no
    #    direction, so hold the current track rather than snapping toward the arbitrary trk=0.
    target_trk = command.trk if cmd_gs > _SPD_EPS else state.trk
    hdg_err = ((target_trk - state.trk + 180.0) % 360.0) - 180.0

    # 3. turn rate: proportional-but-capped desired rate, then a bounded change from the
    #    previous rate (the max_dtr2 limit is why turn_rate is carried in the state)
    desired_tr = _clip(hdg_err, -perf.max_tr, perf.max_tr)
    max_tr_step = perf.max_dtr2 * dt
    tr_step = _clip(desired_tr - state.turn_rate, -max_tr_step, max_tr_step)
    turn_rate = _clip(state.turn_rate + tr_step, -perf.max_tr, perf.max_tr)

    # 4. heading: integrate, unless the target is reachable within this step -> snap onto it
    if abs(hdg_err) > abs(dt * turn_rate):
        trk = (state.trk + dt * turn_rate) % 360.0
    else:
        trk = target_trk % 360.0

    # 5. position: great-circle forward step (metres) along the updated track
    lat, lon = geo.forward(state.lat, state.lon, trk, gs * dt)

    return replace(
        state,
        lat=float(lat),
        lon=float(lon),
        trk=trk,
        gs=gs,
        turn_rate=turn_rate,
    )


class Dynamics(ABC):
    """Base class every dynamics model implements — the contribution surface for how an
    aircraft's kinematics evolve (ADR 0007).

    A model subclasses :class:`Dynamics` and implements ``step``; it is passed into
    :func:`~opencdarr.loop.run_encounter` as ``dynamics=...`` in place of the default. This
    mirrors every other model family in the library (:class:`~opencdarr.cd.base.ConflictDetector`,
    :class:`~opencdarr.cr.base.ConflictResolver`, ...): a new physical effect adds a class, not a
    fork of the loop (``design_brief.md``: the interface is the contribution surface).

    Implementations live beside this file:

    - :class:`PointMassDynamics` — the turn-rate-limited point mass, no wind — implemented,
      the default.
    - e.g. a wind-aware dynamics model — *future, not implemented* (ADR 0007 names the shape).
    """

    @abstractmethod
    def step(
        self, state: AircraftState, command: Command, perf: Performance, dt: float
    ) -> AircraftState:
        """Advance ``state`` by ``dt`` seconds under ``command``.

        Pure — a function of the given arguments only; no global or module state is read or
        written, so a clone (IPS particle) evolved through this call stays independent of its
        source.
        """


class PointMassDynamics(Dynamics):
    """The default :class:`Dynamics`: a turn-rate-and-acceleration-limited 2D point mass, no
    wind. Airframe-agnostic — the airframe is whatever ``Performance`` is passed to ``step``,
    not something this class hard-codes (``performance.py``'s separation of airframe from
    integrator). Thin wrapper: does no math of its own, delegates to :func:`step_dynamics`,
    which remains importable and usable directly (existing tests and scripts are unaffected).
    """

    def step(
        self, state: AircraftState, command: Command, perf: Performance, dt: float
    ) -> AircraftState:
        return step_dynamics(state, command, perf, dt)
