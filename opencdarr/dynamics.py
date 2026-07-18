"""The dynamics boundary: one pure step of M600 point-mass motion.

``step_dynamics`` is the extracted, BlueSky-free integrator that the whole design hangs on
(``design_brief.md`` Decision #4): it advances one aircraft one time step, honouring the
M600's turn-rate, turn-acceleration, and speed limits, as a pure ``state -> state`` map. The
position update uses our own geodesy (``opencdarr.geo``, ADR 0003), so shipping code has no
BlueSky dependency; the boundary stays swappable (``design-philosophy.md`` #5).

Governing equations: ``vault/derivations/step-dynamics-m600.md`` (symbols match the code).
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from opencdarr import geo
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


@dataclass(frozen=True)
class Command:
    """A control command for one aircraft.

    Attributes
    ----------
    hdg:
        Commanded heading (= track, no wind) in degrees, aviation convention.
    spd:
        Commanded ground speed in metres per second.
    """

    hdg: float
    spd: float


def _clip(value: float, low: float, high: float) -> float:
    """Clamp ``value`` to ``[low, high]``."""
    return max(low, min(value, high))


def step_dynamics(
    state: AircraftState,
    command: Command,
    perf: Performance,
    dt: float,
) -> AircraftState:
    """Advance one aircraft by ``dt`` seconds under a heading/speed command.

    Pure: the returned :class:`AircraftState` is a function of the arguments alone; no global
    or module state is read or written. Steps (see the derivation for the full math):

    1. speed           ``target = clip(spd, v_min, v_max)``;
                       ``gs' = gs + clip(target - gs, ±ax*dt)``  (clamp, then ramp)
    2. heading error   ``e = ((hdg - trk + 180) mod 360) - 180``  (signed, shortest way)
    3. turn limiter    ``w_des = clip(e, ±max_tr)``;
                       ``w' = clip(w + clip(w_des - w, ±max_dtr2*dt), ±max_tr)``
    4. heading         integrate ``trk + dt*w'``, or snap to ``hdg`` if reachable this step
    5. position        move ``gs'*dt`` metres along ``trk'`` via ``geo.forward``
    """
    # 1. speed: clamp the command into the envelope, then ramp toward it at no more than
    #    ax*dt (the acceleration analogue of the max_dtr2 turn-rate limit)
    target_gs = _clip(command.spd, perf.v_min, perf.v_max)
    gs = state.gs + _clip(target_gs - state.gs, -perf.ax * dt, perf.ax * dt)

    # 2. heading error, signed and taken the short way round
    hdg_err = ((command.hdg - state.trk + 180.0) % 360.0) - 180.0

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
        trk = command.hdg % 360.0

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
