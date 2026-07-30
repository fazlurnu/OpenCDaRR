"""The simplest thing that is still a working Kinematics — a scaffold for your own.

``VelocityFollower`` moves at exactly the commanded ground velocity each step. No acceleration
ramp, no speed cap, no turn radius, no wind — it does not even read the ``Performance`` envelope.
It is deliberately barer than :class:`~opencdarr.kinematics.Multirotor`, which is the same idea
(a holonomic velocity-follower) plus every limit this one drops: an ``ax`` ramp, the wind
air/ground-frame conversion, a decoupled yaw channel, and a waypoint stopping law.

``step`` is a pure function — its result depends only on its arguments and it touches no global
state, so a clone (an IPS particle) evolved through it stays independent of its source.

Run it directly to watch it move and fly a real two-aircraft encounter::

    python scripts/dummy_kinematics.py
"""

from __future__ import annotations

import math
from dataclasses import replace

from opencdarr import geo
from opencdarr.kinematics.base import Kinematics, MotionCommand, odometry_update
from opencdarr.performance import Performance
from opencdarr.state import AircraftState
from opencdarr.wind import NO_WIND, WindField

_SPD_EPS = 1e-9  # m/s: below this a velocity has no meaningful direction -> hold current track


class VelocityFollower(Kinematics):
    """Move at exactly the commanded ground velocity. The minimal working Kinematics.

    Reads only the velocity channel of the command; ``perf`` and ``wind`` are part of the contract
    but ignored (that is what makes it a dummy). To respect a top speed, cap ``speed`` at
    ``perf.v_max`` — two lines. To fly a ``WaypointAutopilot`` (a ``target_position`` command),
    add a branch that heads toward it. Everything else here is a real, correct integrator.
    """

    def step(
        self,
        state: AircraftState,
        command: MotionCommand,
        perf: Performance,
        dt: float,
        wind: WindField = NO_WIND,
    ) -> AircraftState:
        v_east, v_north = command.v_east, command.v_north  # raises if no velocity is set
        speed = math.hypot(v_east, v_north)
        trk = state.trk if speed <= _SPD_EPS else math.degrees(math.atan2(v_east, v_north)) % 360.0
        lat, lon = geo.forward(state.lat, state.lon, trk, speed * dt)
        return replace(
            state,
            lat=float(lat),
            lon=float(lon),
            trk=trk,
            gs=speed,
            **odometry_update(state, speed, dt),  # keeps flight_time / distance_flown correct
        )


def _demo_pure_steps() -> None:
    """Step the model by hand and print the trajectory — the pure-function view."""
    from opencdarr.performance import M600

    kinematics = VelocityFollower()
    state = AircraftState(id="TOY", lat=52.0, lon=4.0, trk=0.0, gs=0.0, yaw=0.0)
    cmd = MotionCommand(target_velocity=(20.0, 20.0))  # NE at 28.3 m/s, followed exactly

    print("VelocityFollower, commanded NE at 28.3 m/s (no cap: the envelope is ignored)\n")
    print(f"{'t [s]':>5} {'lat':>10} {'lon':>10} {'trk':>6} {'gs':>6} {'flown [m]':>10}")
    t, dt = 0.0, 1.0
    for _ in range(6):
        print(f"{t:5.0f} {state.lat:10.5f} {state.lon:10.5f} "
              f"{state.trk:6.1f} {state.gs:6.2f} {state.distance_flown:10.1f}")
        state = kinematics.step(state, cmd, M600, dt)
        t += dt


def _demo_fleet() -> None:
    """Drop the model into the real fleet loop, unchanged — the "it actually works" view."""
    from opencdarr.autopilot import CruiseAutopilot
    from opencdarr.cd import StateBased
    from opencdarr.cr import MVP
    from opencdarr.crr import PastCPA
    from opencdarr.fleet import Agent, run_fleet
    from opencdarr.performance import M600
    from opencdarr.scenario import create_conflict

    own = AircraftState(id="A", lat=52.0, lon=4.0, trk=0.0, gs=15.0, yaw=0.0)
    intr = create_conflict(own, intr_id="B", dpsi=90.0, dcpa=0.0,
                           tlos=30.0, rpz=50.0, gs_intr=15.0, side=1)
    agents = [
        Agent(own, M600, VelocityFollower(), CruiseAutopilot(own.trk, own.gs)),
        Agent(intr, M600, VelocityFollower(), CruiseAutopilot(intr.trk, intr.gs)),
    ]
    out = run_fleet(agents, rpz=50.0, t_lookahead=20.0, dt=0.5,
                    detector=StateBased(), resolver=MVP(), recovery=PastCPA(bouncing_guard=True),
                    done_timeout=10.0)
    print("\nfleet run (StateBased + MVP + PastCPA) on VelocityFollower:")
    print(f"  min_sep = {out.min_sep:.1f} m   loss of separation = {out.los}")


if __name__ == "__main__":
    _demo_pure_steps()
    _demo_fleet()
