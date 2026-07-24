"""Holonomic point-mass dynamics (ADR 0009): :class:`HolonomicDynamics`.

The ground-velocity **vector** moves directly toward the commanded vector, bounded only by an
isotropic acceleration limit — no coupled heading, no turn-rate limit. Where
:class:`~opencdarr.dynamics.DubinsDynamics` reconstructs a target track and turns toward it, this
model chases ``(command.v_east, command.v_north)`` as a vector in the plane: a 90 deg direction
change is one bounded step in velocity-space, not a rate-limited sweep through intermediate
headings. This is what ADR 0008's velocity-vector ``Command`` was for — read here with no polar
reconstruction at all.

Governing equations: ``vault/decisions/0009-holonomic-dynamics.md``.
"""

from __future__ import annotations

import math
from dataclasses import replace

from opencdarr import geo
from opencdarr.dynamics.base import _SPD_EPS, Command, Dynamics, odometry_update
from opencdarr.kinematics import velocity_enu
from opencdarr.performance import Performance
from opencdarr.state import AircraftState


def _clip_magnitude(vx: float, vy: float, max_mag: float) -> tuple[float, float]:
    """Scale ``(vx, vy)`` down to at most ``max_mag``, preserving direction; leave it if within."""
    mag = math.hypot(vx, vy)
    if mag <= max_mag:
        return vx, vy
    scale = max_mag / mag
    return vx * scale, vy * scale


class HolonomicDynamics(Dynamics):
    """A holonomic point mass (ADR 0009): the ground-velocity vector moves directly toward the
    commanded vector under an isotropic acceleration limit — no coupled heading, no turn-rate
    limit.

    Reuses only ``perf.v_max`` (top speed) and ``perf.ax`` (max acceleration, now isotropic —
    equally hard in every direction, not decomposed into turn-rate and speed-ramp) from
    :class:`~opencdarr.performance.Performance`. ``perf.max_tr`` and ``perf.max_dtr2`` do not apply
    (there is no turn rate to limit); ``perf.v_min`` does not apply either — a holonomic vehicle
    has no separate "backward" capability, since facing is decoupled from travel: moving the other
    way is just a different direction, already reachable via the vector.
    ``AircraftState.turn_rate`` stays at its default (never read or set here).

    ``trk``/``gs`` mean exactly what they mean under :class:`~opencdarr.dynamics.DubinsDynamics` —
    direction and magnitude of ground travel — so CD/CR/CRR, and a Dubins-car aircraft sharing the
    same encounter, read this aircraft's state identically. Only *how the vehicle reaches* a given
    velocity differs, which is the whole point of the boundary this class sits behind.
    """

    def step(
        self, state: AircraftState, command: Command, perf: Performance, dt: float
    ) -> AircraftState:
        # 1. target: the commanded vector, clamped to the top-speed envelope (no v_min — see
        #    class docstring). Clamping first, then bounding the step toward it, keeps the result
        #    inside the v_max disk throughout (the disk is convex; both endpoints of the step are
        #    inside it, so every point on the step is too).
        tgt_e, tgt_n = _clip_magnitude(command.v_east, command.v_north, perf.v_max)

        # 2. isotropic acceleration limit: bound the *vector* step by ax*dt in any direction, not
        #    two independent 1D limits (that would be DubinsDynamics' turn-rate + speed ramp).
        cur_e, cur_n = velocity_enu(state)
        step_e, step_n = _clip_magnitude(tgt_e - cur_e, tgt_n - cur_n, perf.ax * dt)
        new_e, new_n = cur_e + step_e, cur_n + step_n

        # 3. direction/magnitude of ground travel, derived from the new vector. A ~zero vector has
        #    no defined direction -> hold the current track (mirrors step_dynamics' rule).
        new_gs = math.hypot(new_e, new_n)
        new_trk = (
            state.trk if new_gs <= _SPD_EPS else math.degrees(math.atan2(new_e, new_n)) % 360.0
        )

        # 4. position: great-circle forward step (metres) along the new track
        lat, lon = geo.forward(state.lat, state.lon, new_trk, new_gs * dt)

        return replace(
            state,
            lat=float(lat),
            lon=float(lon),
            trk=new_trk,
            gs=new_gs,
            **odometry_update(state, new_gs, dt),
        )
