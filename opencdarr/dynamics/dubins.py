"""Dubins-flavoured point-mass dynamics: ``step_dynamics`` + :class:`DubinsDynamics`.

The extracted, BlueSky-free integrator the whole design hangs on (``design_brief.md`` Decision #4
and ADR 0002/0003): it advances one aircraft one time step, honouring the airframe's turn-rate,
turn-acceleration, and speed limits (via ``Performance``), as a pure ``state -> state`` map. The
position update uses our own geodesy (``opencdarr.geo``), so shipping code has no BlueSky
dependency.

"Dubins" is used loosely: like a Dubins car, heading is **coupled to direction of travel** and
turning is curvature-limited, which is the property that distinguishes this model from
:class:`~opencdarr.dynamics.HolonomicDynamics`. It is *not* the textbook Dubins car — that flies at
constant speed along a fixed minimum-radius arc, whereas this model has a variable,
acceleration-limited speed (``ax``) and limits the turn *rate* and its rate of change
(``max_tr``/``max_dtr2``), not a fixed radius. The name is the communicative one for the audience;
the docstring keeps it honest.

Governing equations: ``vault/derivations/step-dynamics-m600.md`` (symbols match the code).
"""

from __future__ import annotations

from dataclasses import replace

from opencdarr import geo
from opencdarr.dynamics.base import _SPD_EPS, Command, Dynamics, _clip, odometry_update
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


def step_dynamics(
    state: AircraftState,
    command: Command,
    perf: Performance,
    dt: float,
) -> AircraftState:
    """Advance one aircraft by ``dt`` seconds under a velocity-vector command.

    The model faces its direction of travel, so it reads the command's *magnitude*
    (``command.gs``) as the target speed and *direction* (``command.trk``) as the target track,
    then turn-rate-limits toward it — the polar reconstruction the airframe assumes lives here, not
    in the shared :class:`Command` (ADR 0007/0008). Pure: the returned :class:`AircraftState` is a
    function of the arguments alone. Steps (see the derivation for the full math):

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
        **odometry_update(state, gs, dt),
    )


class DubinsDynamics(Dynamics):
    """The default :class:`Dynamics`: a turn-rate-and-acceleration-limited 2D point mass with a
    heading coupled to its direction of travel (Dubins-flavoured — see the module docstring for
    why not the textbook Dubins car), no wind. Airframe-agnostic — the airframe is whatever
    ``Performance`` is passed to ``step``, not something this class hard-codes
    (``performance.py``'s separation of airframe from integrator). Thin wrapper: does no math of
    its own, delegates to :func:`step_dynamics`, which remains importable and usable directly.
    """

    def step(
        self, state: AircraftState, command: Command, perf: Performance, dt: float
    ) -> AircraftState:
        return step_dynamics(state, command, perf, dt)
