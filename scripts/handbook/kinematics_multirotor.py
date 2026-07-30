"""Handbook figure: multirotor trajectories under different MotionCommands.

Drives the real :class:`~opencdarr.kinematics.Multirotor` (M600 limits) from a hover under three
command modes and plots the ground tracks and the speed profiles. Writes the PNG into the site
repo next door.

    PYTHONPATH=. python scripts/handbook/kinematics_multirotor.py
"""
from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from opencdarr import geo  # noqa: E402
from opencdarr.kinematics import Multirotor, MotionCommand  # noqa: E402
from opencdarr.performance import M600  # noqa: E402
from opencdarr.state import AircraftState  # noqa: E402

DT = 0.5
T_MAX = 16.0
YAW_TURN_T = 8.0  # when the body-forward case yaws from east to north
LAT0, LON0 = 52.0, 4.0
KINEMATICS = Multirotor()
OUT = Path.home() / "Projects/opencdarr.github.io/docs/assets/img/multirotor-trajectories.png"

CommandFn = Callable[[AircraftState, float], MotionCommand]


def run(command_fn: CommandFn, yaw0: float | None) -> tuple[list[float], list[float], list[float]]:
    """Advance from a hover (nose at ``yaw0``); return (east, north, speed) sampled each step."""
    s = AircraftState(id="M600", lat=LAT0, lon=LON0, trk=0.0, gs=0.0, yaw=yaw0)
    es, ns, spd = [0.0], [0.0], [0.0]
    t = 0.0
    while t < T_MAX:
        s = KINEMATICS.step(s, command_fn(s, t), M600, DT)
        qdr, dist = geo.qdrdist(LAT0, LON0, s.lat, s.lon)
        r = math.radians(qdr)
        es.append(dist * math.sin(r))
        ns.append(dist * math.cos(r))
        spd.append(s.gs)
        t += DT
    return es, ns, spd


# --- three command modes -------------------------------------------------------------------
def cmd_constant_north(_s: AircraftState, _t: float) -> MotionCommand:
    """Constant inertial ground velocity, due north."""
    return MotionCommand.from_track_speed(0.0, 15.0)


def cmd_constant_ne(_s: AircraftState, _t: float) -> MotionCommand:
    """Constant inertial ground velocity, to the north-east."""
    return MotionCommand.from_track_speed(45.0, 15.0)


def cmd_body_yaw(_s: AircraftState, t: float) -> MotionCommand:
    """Body-forward velocity while the nose yaws from east to north partway through."""
    target_yaw = None if t < YAW_TURN_T else 0.0  # hold east, then turn the nose to north
    return MotionCommand(target_body_velocity=(15.0, 0.0), target_yaw=target_yaw)


CASES = [
    ("target_velocity — constant (north)", "#1f77b4", None, cmd_constant_north),
    ("target_velocity — north-east", "#ff7f0e", None, cmd_constant_ne),
    ("target_body_velocity — forward + yaw 90°", "#2ca02c", 90.0, cmd_body_yaw),
]


def main() -> None:
    fig, (ax_xy, ax_v) = plt.subplots(1, 2, figsize=(12.5, 5.2))
    for label, col, yaw0, fn in CASES:
        es, ns, spd = run(fn, yaw0)
        ts = [i * DT for i in range(len(spd))]
        ax_xy.plot(es, ns, color=col, lw=2.0, label=label)
        ax_xy.plot(es[0], ns[0], "^", color=col, ms=9)   # start (hover)
        ax_xy.plot(es[-1], ns[-1], "s", color=col, ms=7)  # end
        for k in range(0, len(es), 4):  # a dot every 2 s to read speed as spacing
            ax_xy.plot(es[k], ns[k], ".", color=col, ms=4)
        ax_v.plot(ts, spd, color=col, lw=2.0, label=label)

    ax_xy.set_title("Ground tracks (△ start / hover, □ end, dots every 2 s)", fontsize=10)
    ax_xy.set_xlabel("east [m]")
    ax_xy.set_ylabel("north [m]")
    ax_xy.set_aspect("equal")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(fontsize=8, loc="upper left")

    ax_v.axhline(M600.v_max, color="0.5", ls="--", lw=1.0, label=f"v_max = {M600.v_max:.0f} m/s")
    ax_v.set_title("Ground speed — accelerate at a_x, then hold", fontsize=10)
    ax_v.set_xlabel("time [s]")
    ax_v.set_ylabel("ground speed [m/s]")
    ax_v.set_ylim(0, M600.v_max + 2)
    ax_v.grid(True, alpha=0.3)
    ax_v.legend(fontsize=8, loc="lower right")

    fig.suptitle(
        f"Multirotor (M600) under three MotionCommands — v_max {M600.v_max:.0f} m/s, "
        f"a_x {M600.ax:.0f} m/s², yaw rate {M600.yaw_rate_max:.0f} °/s",
        fontsize=11,
    )
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=130)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
